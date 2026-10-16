// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm Peripheral Image Loader
 *
 * Copyright (C) 2016 Linaro Ltd
 * Copyright (C) 2015 Sony Mobile Communications Inc
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 * Copyright (C) 2026 Dawid Wróbel <me@dawidwrobel.com>
 */

#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/elf.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/soc/qcom/mdt_loader.h>

static bool mdt_header_valid(const struct firmware *fw)
{
	const struct elf32_hdr *ehdr;
	size_t phend;
	size_t shend;

	if (fw->size < sizeof(*ehdr))
		return false;

	ehdr = (struct elf32_hdr *)fw->data;

	if (memcmp(ehdr->e_ident, ELFMAG, SELFMAG))
		return false;

	if (ehdr->e_phentsize != sizeof(struct elf32_phdr))
		return false;

	phend = size_add(size_mul(sizeof(struct elf32_phdr), ehdr->e_phnum), ehdr->e_phoff);
	if (phend > fw->size)
		return false;

	if (ehdr->e_shentsize || ehdr->e_shnum) {
		if (ehdr->e_shentsize != sizeof(struct elf32_shdr))
			return false;

		shend = size_add(size_mul(sizeof(struct elf32_shdr), ehdr->e_shnum), ehdr->e_shoff);
		if (shend > fw->size)
			return false;
	}

	return true;
}

/*
 * Does this segment contribute bytes to the image file?
 *
 * A different question from mdt_phdr_loadable(), which asks whether a segment
 * is placed into the target memory region. A hash segment is not placed there
 * -- it is metadata, handled by qcom_mdt_read_metadata() -- but it is very much
 * part of the file, as is anything else carrying payload. Consumers that
 * reassemble the image rather than load it want this one.
 */

/*
 * The image-assembly helpers below serve both ELF classes. Remoteproc firmware
 * and the first generation of QSEE applications are ELF32; newer QSEE
 * applications (QSEE64) are ELF64. qcom_mdt_load() only ever handles the
 * former, so mdt_header_valid() stays ELF32-only; these helpers are for
 * qcom_mdt_get_image_size()/qcom_mdt_read_image(), which reassemble an
 * application image for the secure world and must accept either class.
 */
struct mdt_image_hdr {
	u8 class;
	unsigned int phnum;
	const void *phdrs;
};

static bool mdt_image_hdr_parse(const struct firmware *fw,
				struct mdt_image_hdr *hdr)
{
	const struct elf32_hdr *ehdr32;
	const struct elf64_hdr *ehdr64;
	size_t phend;
	u8 class;

	if (fw->size < EI_NIDENT)
		return false;
	if (memcmp(fw->data, ELFMAG, SELFMAG))
		return false;

	class = ((const u8 *)fw->data)[EI_CLASS];
	if (class == ELFCLASS32) {
		if (fw->size < sizeof(*ehdr32))
			return false;
		ehdr32 = (const void *)fw->data;
		if (ehdr32->e_phentsize != sizeof(struct elf32_phdr))
			return false;
		hdr->phnum = ehdr32->e_phnum;
		hdr->phdrs = fw->data + ehdr32->e_phoff;
		phend = size_add(size_mul(sizeof(struct elf32_phdr),
					  ehdr32->e_phnum), ehdr32->e_phoff);
	} else if (class == ELFCLASS64) {
		if (fw->size < sizeof(*ehdr64))
			return false;
		ehdr64 = (const void *)fw->data;
		if (ehdr64->e_phentsize != sizeof(struct elf64_phdr))
			return false;
		hdr->phnum = ehdr64->e_phnum;
		hdr->phdrs = fw->data + ehdr64->e_phoff;
		phend = size_add(size_mul(sizeof(struct elf64_phdr),
					  ehdr64->e_phnum), ehdr64->e_phoff);
	} else {
		return false;
	}

	if (phend > fw->size)
		return false;

	hdr->class = class;
	return true;
}

static u64 mdt_image_phdr_offset(const struct mdt_image_hdr *hdr,
				 unsigned int i)
{
	if (hdr->class == ELFCLASS32)
		return ((const struct elf32_phdr *)hdr->phdrs)[i].p_offset;

	return ((const struct elf64_phdr *)hdr->phdrs)[i].p_offset;
}

static u64 mdt_image_phdr_filesz(const struct mdt_image_hdr *hdr,
				 unsigned int i)
{
	if (hdr->class == ELFCLASS32)
		return ((const struct elf32_phdr *)hdr->phdrs)[i].p_filesz;

	return ((const struct elf64_phdr *)hdr->phdrs)[i].p_filesz;
}

static bool mdt_phdr_loadable(const struct elf32_phdr *phdr)
{
	if (phdr->p_type != PT_LOAD)
		return false;

	if ((phdr->p_flags & QCOM_MDT_TYPE_MASK) == QCOM_MDT_TYPE_HASH)
		return false;

	if (!phdr->p_memsz)
		return false;

	return true;
}

static ssize_t mdt_load_split_segment(void *ptr, size_t filesz,
				      unsigned int segment, const char *fw_name,
				      struct device *dev)
{
	const struct firmware *seg_fw;
	ssize_t ret;

	if (strlen(fw_name) < 4)
		return -EINVAL;

	char *seg_name __free(kfree) = kstrdup(fw_name, GFP_KERNEL);
	if (!seg_name)
		return -ENOMEM;

	sprintf(seg_name + strlen(fw_name) - 3, "b%02d", segment);
	ret = request_firmware_into_buf(&seg_fw, seg_name, dev, ptr, filesz);
	if (ret) {
		dev_err(dev, "error %zd loading %s\n", ret, seg_name);
		return ret;
	}

	if (seg_fw->size != filesz) {
		dev_err(dev,
			"failed to load segment %d from truncated file %s\n",
			segment, seg_name);
		ret = -EINVAL;
	}

	release_firmware(seg_fw);

	return ret;
}

/**
 * qcom_mdt_get_size() - acquire size of the memory region needed to load mdt
 * @fw:		firmware object for the mdt file
 *
 * Returns size of the loaded firmware blob, or -EINVAL on failure.
 */
ssize_t qcom_mdt_get_size(const struct firmware *fw)
{
	const struct elf32_phdr *phdrs;
	const struct elf32_phdr *phdr;
	const struct elf32_hdr *ehdr;
	phys_addr_t min_addr = PHYS_ADDR_MAX;
	phys_addr_t max_addr = 0;
	int i;

	if (!mdt_header_valid(fw))
		return -EINVAL;

	ehdr = (struct elf32_hdr *)fw->data;
	phdrs = (struct elf32_phdr *)(fw->data + ehdr->e_phoff);

	for (i = 0; i < ehdr->e_phnum; i++) {
		phdr = &phdrs[i];

		if (!mdt_phdr_loadable(phdr))
			continue;

		if (phdr->p_paddr < min_addr)
			min_addr = phdr->p_paddr;

		if (phdr->p_paddr + phdr->p_memsz > max_addr)
			max_addr = ALIGN(phdr->p_paddr + phdr->p_memsz, SZ_4K);
	}

	return min_addr < max_addr ? max_addr - min_addr : -EINVAL;
}
EXPORT_SYMBOL_GPL(qcom_mdt_get_size);

static bool qcom_mdt_bins_are_split(const struct firmware *fw)
{
	struct mdt_image_hdr hdr;
	uint64_t seg_start, seg_end;
	unsigned int i;

	if (!mdt_image_hdr_parse(fw, &hdr))
		return false;

	for (i = 0; i < hdr.phnum; i++) {
		uint64_t filesz = mdt_image_phdr_filesz(&hdr, i);

		/*
		 * The size of the MDT file is not padded to include any
		 * zero-sized segments at the end. Ignore these, as they should
		 * not affect the decision about image being split or not.
		 */
		if (!filesz)
			continue;

		seg_start = mdt_image_phdr_offset(&hdr, i);
		seg_end = seg_start + filesz;
		if (seg_start > fw->size || seg_end > fw->size)
			return true;
	}

	return false;
}

/**
 * qcom_mdt_get_image_size() - size of the image as one contiguous blob
 * @fw:		firmware object for the mdt file
 *
 * Some Qualcomm firmware loaders -- QSEECOM's application loader among them --
 * take the whole image as a single buffer, the mdt followed by the segment
 * payloads, rather than having the segments placed at their physical
 * addresses. This returns the size such a buffer needs.
 *
 * Deliberately not qcom_mdt_get_size(), which reports the span from the lowest
 * to the highest p_paddr: that is the memory region a remoteproc needs, and it
 * bears no relation to how much file there is.
 *
 * Return: size in bytes, or negative errno on failure.
 */
ssize_t qcom_mdt_get_image_size(const struct firmware *fw)
{
	struct mdt_image_hdr hdr;
	size_t size;
	unsigned int i;

	if (!mdt_image_hdr_parse(fw, &hdr))
		return -EINVAL;

	/* A single-file image already is the blob. */
	if (!qcom_mdt_bins_are_split(fw))
		return fw->size;

	size = fw->size;

	for (i = 0; i < hdr.phnum; i++) {
		uint64_t filesz = mdt_image_phdr_filesz(&hdr, i);

		if (!filesz)
			continue;

		if (size + filesz < size)
			return -EINVAL;

		size += filesz;
	}

	return size;
}
EXPORT_SYMBOL_GPL(qcom_mdt_get_image_size);

/**
 * qcom_mdt_read_image() - assemble the image as one contiguous blob
 * @dev:	device handle to associate resources with
 * @fw:		firmware object for the mdt file
 * @fw_name:	name of the firmware, for construction of segment file names
 * @mem:	buffer to assemble into
 * @mem_size:	size of @mem, at least qcom_mdt_get_image_size()
 *
 * Copies the mdt and then every segment that has one, in program-header order,
 * one after another. This is the layout QSEECOM's application loader expects,
 * and it is not the layout qcom_mdt_load() produces -- that one places
 * segments at their p_paddr, which for some images is not a usable file layout
 * at all: a segment may declare p_offset 0, overlapping the headers, and two
 * segments may declare the same p_offset.
 *
 * Selected by a non-zero segment size rather than mdt_phdr_loadable(): this is
 * not a loading operation, so the question is which segments are part of the
 * file, not which are placed in memory. The hash in particular is skipped by
 * the latter but belongs here -- the secure world is handed the image whole and
 * authenticates it whole, so dropping a segment shifts everything after it and
 * the image is rejected.
 *
 * Return: number of bytes assembled, or negative errno on failure.
 */
ssize_t qcom_mdt_read_image(struct device *dev, const struct firmware *fw,
			    const char *fw_name, void *mem, size_t mem_size)
{
	struct mdt_image_hdr hdr;
	size_t off;
	ssize_t ret;
	unsigned int i;

	ret = qcom_mdt_get_image_size(fw);
	if (ret < 0)
		return ret;

	if (mem_size < (size_t)ret)
		return -ENOSPC;

	memcpy(mem, fw->data, fw->size);

	if (!qcom_mdt_bins_are_split(fw))
		return fw->size;

	if (!mdt_image_hdr_parse(fw, &hdr))
		return -EINVAL;

	off = fw->size;

	for (i = 0; i < hdr.phnum; i++) {
		uint64_t filesz = mdt_image_phdr_filesz(&hdr, i);

		if (!filesz)
			continue;

		ret = mdt_load_split_segment(mem + off, filesz, i, fw_name,
					     dev);
		if (ret)
			return ret;

		off += filesz;
	}

	return off;
}
EXPORT_SYMBOL_GPL(qcom_mdt_read_image);

/**
 * qcom_mdt_read_metadata() - read header and metadata from mdt or mbn
 * @fw:		firmware of mdt header or mbn
 * @data_len:	length of the read metadata blob
 * @fw_name:	name of the firmware, for construction of segment file names
 * @dev:	device handle to associate resources with
 *
 * The mechanism that performs the authentication of the loading firmware
 * expects an ELF header directly followed by the segment of hashes, with no
 * padding inbetween. This function allocates a chunk of memory for this pair
 * and copy the two pieces into the buffer.
 *
 * In the case of split firmware the hash is found directly following the ELF
 * header, rather than at p_offset described by the second program header.
 *
 * The caller is responsible to free (kfree()) the returned pointer.
 *
 * Return: pointer to data, or ERR_PTR()
 */
void *qcom_mdt_read_metadata(const struct firmware *fw, size_t *data_len,
			     const char *fw_name, struct device *dev)
{
	const struct elf32_phdr *phdrs;
	const struct elf32_hdr *ehdr;
	unsigned int hash_segment = 0;
	size_t hash_offset;
	size_t hash_size;
	size_t ehdr_size;
	unsigned int i;
	ssize_t ret;
	void *data;

	if (!mdt_header_valid(fw))
		return ERR_PTR(-EINVAL);

	ehdr = (struct elf32_hdr *)fw->data;
	phdrs = (struct elf32_phdr *)(fw->data + ehdr->e_phoff);

	if (ehdr->e_phnum < 2)
		return ERR_PTR(-EINVAL);

	if (phdrs[0].p_type == PT_LOAD)
		return ERR_PTR(-EINVAL);

	for (i = 1; i < ehdr->e_phnum; i++) {
		if ((phdrs[i].p_flags & QCOM_MDT_TYPE_MASK) == QCOM_MDT_TYPE_HASH) {
			hash_segment = i;
			break;
		}
	}

	if (!hash_segment) {
		dev_err(dev, "no hash segment found in %s\n", fw_name);
		return ERR_PTR(-EINVAL);
	}

	ehdr_size = phdrs[0].p_filesz;
	hash_size = phdrs[hash_segment].p_filesz;

	data = kmalloc(ehdr_size + hash_size, GFP_KERNEL);
	if (!data)
		return ERR_PTR(-ENOMEM);

	/* Copy ELF header */
	memcpy(data, fw->data, ehdr_size);

	if (ehdr_size + hash_size == fw->size) {
		/* Firmware is split and hash is packed following the ELF header */
		hash_offset = phdrs[0].p_filesz;
		memcpy(data + ehdr_size, fw->data + hash_offset, hash_size);
	} else if (phdrs[hash_segment].p_offset + hash_size <= fw->size) {
		/* Hash is in its own segment, but within the loaded file */
		hash_offset = phdrs[hash_segment].p_offset;
		memcpy(data + ehdr_size, fw->data + hash_offset, hash_size);
	} else {
		/* Hash is in its own segment, beyond the loaded file */
		ret = mdt_load_split_segment(data + ehdr_size,
					     phdrs[hash_segment].p_filesz,
					     hash_segment, fw_name, dev);
		if (ret) {
			kfree(data);
			return ERR_PTR(ret);
		}
	}

	*data_len = ehdr_size + hash_size;

	return data;
}
EXPORT_SYMBOL_GPL(qcom_mdt_read_metadata);

static int __qcom_mdt_pas_init(struct device *dev, const struct firmware *fw,
			       const char *fw_name, int pas_id, phys_addr_t mem_phys,
			       struct qcom_scm_pas_context *ctx)
{
	const struct elf32_phdr *phdrs;
	const struct elf32_phdr *phdr;
	const struct elf32_hdr *ehdr;
	phys_addr_t min_addr = PHYS_ADDR_MAX;
	phys_addr_t max_addr = 0;
	bool relocate = false;
	size_t metadata_len;
	void *metadata;
	int ret;
	int i;

	if (!mdt_header_valid(fw))
		return -EINVAL;

	ehdr = (struct elf32_hdr *)fw->data;
	phdrs = (struct elf32_phdr *)(fw->data + ehdr->e_phoff);

	for (i = 0; i < ehdr->e_phnum; i++) {
		phdr = &phdrs[i];

		if (!mdt_phdr_loadable(phdr))
			continue;

		if (phdr->p_flags & QCOM_MDT_RELOCATABLE)
			relocate = true;

		if (phdr->p_paddr < min_addr)
			min_addr = phdr->p_paddr;

		if (phdr->p_paddr + phdr->p_memsz > max_addr)
			max_addr = ALIGN(phdr->p_paddr + phdr->p_memsz, SZ_4K);
	}

	metadata = qcom_mdt_read_metadata(fw, &metadata_len, fw_name, dev);
	if (IS_ERR(metadata)) {
		ret = PTR_ERR(metadata);
		dev_err(dev, "error %d reading firmware %s metadata\n", ret, fw_name);
		goto out;
	}

	ret = qcom_scm_pas_init_image(pas_id, metadata, metadata_len, ctx);
	kfree(metadata);
	if (ret) {
		/* Invalid firmware metadata */
		dev_err(dev, "error %d initializing firmware %s\n", ret, fw_name);
		goto out;
	}

	if (relocate) {
		ret = qcom_scm_pas_mem_setup(pas_id, mem_phys, max_addr - min_addr);
		if (ret) {
			/* Unable to set up relocation */
			dev_err(dev, "error %d setting up firmware %s\n", ret, fw_name);
			goto out;
		}
	}

out:
	return ret;
}

/**
 * qcom_mdt_load_no_init() - load the firmware which header is loaded as fw
 * @dev:	device handle to associate resources with
 * @fw:		firmware object for the mdt file
 * @fw_name:	name of the firmware, for construction of segment file names
 * @mem_region:	allocated memory region to load firmware into
 * @mem_phys:	physical address of allocated memory region
 * @mem_size:	size of the allocated memory region
 * @reloc_base:	adjusted physical address after relocation
 *
 * Returns 0 on success, negative errno otherwise.
 */
int qcom_mdt_load_no_init(struct device *dev, const struct firmware *fw,
			  const char *fw_name, void *mem_region,
			  phys_addr_t mem_phys, size_t mem_size,
			  phys_addr_t *reloc_base)
{
	const struct elf32_phdr *phdrs;
	const struct elf32_phdr *phdr;
	const struct elf32_hdr *ehdr;
	phys_addr_t mem_reloc;
	phys_addr_t min_addr = PHYS_ADDR_MAX;
	ssize_t offset;
	bool relocate = false;
	bool is_split;
	void *ptr;
	int ret = 0;
	int i;

	if (!fw || !mem_region || !mem_phys || !mem_size)
		return -EINVAL;

	if (!mdt_header_valid(fw))
		return -EINVAL;

	is_split = qcom_mdt_bins_are_split(fw);
	ehdr = (struct elf32_hdr *)fw->data;
	phdrs = (struct elf32_phdr *)(fw->data + ehdr->e_phoff);

	for (i = 0; i < ehdr->e_phnum; i++) {
		phdr = &phdrs[i];

		if (!mdt_phdr_loadable(phdr))
			continue;

		if (phdr->p_flags & QCOM_MDT_RELOCATABLE)
			relocate = true;

		if (phdr->p_paddr < min_addr)
			min_addr = phdr->p_paddr;
	}

	if (relocate) {
		/*
		 * The image is relocatable, so offset each segment based on
		 * the lowest segment address.
		 */
		mem_reloc = min_addr;
	} else {
		/*
		 * Image is not relocatable, so offset each segment based on
		 * the allocated physical chunk of memory.
		 */
		mem_reloc = mem_phys;
	}

	for (i = 0; i < ehdr->e_phnum; i++) {
		phdr = &phdrs[i];

		if (!mdt_phdr_loadable(phdr))
			continue;

		offset = phdr->p_paddr - mem_reloc;
		if (offset < 0 || offset + phdr->p_memsz > mem_size) {
			dev_err(dev, "segment outside memory range\n");
			ret = -EINVAL;
			break;
		}

		if (phdr->p_filesz > phdr->p_memsz) {
			dev_err(dev,
				"refusing to load segment %d with p_filesz > p_memsz\n",
				i);
			ret = -EINVAL;
			break;
		}

		ptr = mem_region + offset;

		if (phdr->p_filesz && !is_split) {
			/* Firmware is large enough to be non-split */
			if (phdr->p_offset + phdr->p_filesz > fw->size) {
				dev_err(dev, "file %s segment %d would be truncated\n",
					fw_name, i);
				ret = -EINVAL;
				break;
			}

			memcpy(ptr, fw->data + phdr->p_offset, phdr->p_filesz);
		} else if (phdr->p_filesz) {
			/* Firmware not large enough, load split-out segments */
			ret = mdt_load_split_segment(ptr, phdr->p_filesz, i,
						     fw_name, dev);
			if (ret)
				break;
		}

		if (phdr->p_memsz > phdr->p_filesz)
			memset(ptr + phdr->p_filesz, 0, phdr->p_memsz - phdr->p_filesz);
	}

	if (reloc_base)
		*reloc_base = mem_reloc;

	return ret;
}
EXPORT_SYMBOL_GPL(qcom_mdt_load_no_init);

/**
 * qcom_mdt_load() - load the firmware which header is loaded as fw
 * @dev:	device handle to associate resources with
 * @fw:		firmware object for the mdt file
 * @fw_name:	name of the firmware, for construction of segment file names
 * @pas_id:	PAS identifier
 * @mem_region:	allocated memory region to load firmware into
 * @mem_phys:	physical address of allocated memory region
 * @mem_size:	size of the allocated memory region
 * @reloc_base:	adjusted physical address after relocation
 *
 * Returns 0 on success, negative errno otherwise.
 */
int qcom_mdt_load(struct device *dev, const struct firmware *fw,
		  const char *fw_name, int pas_id, void *mem_region,
		  phys_addr_t mem_phys, size_t mem_size,
		  phys_addr_t *reloc_base)
{
	int ret;

	ret = __qcom_mdt_pas_init(dev, fw, fw_name, pas_id, mem_phys, NULL);
	if (ret)
		return ret;

	return qcom_mdt_load_no_init(dev, fw, fw_name, mem_region, mem_phys,
				     mem_size, reloc_base);
}
EXPORT_SYMBOL_GPL(qcom_mdt_load);

/**
 * qcom_mdt_pas_load - Loads and authenticates the metadata of the firmware
 * (typically contained in the .mdt file), followed by loading the actual
 * firmware segments (e.g., .bXX files). Authentication of the segments done
 * by a separate call.
 *
 * The PAS context must be initialized using qcom_scm_pas_context_init()
 * prior to invoking this function.
 *
 * @ctx:        Pointer to the PAS (Peripheral Authentication Service) context
 * @fw:         Firmware object representing the .mdt file
 * @firmware:   Name of the firmware used to construct segment file names
 * @mem_region: Memory region allocated for loading the firmware
 * @reloc_base: Physical address adjusted after relocation
 *
 * Return: 0 on success or a negative error code on failure.
 */
int qcom_mdt_pas_load(struct qcom_scm_pas_context *ctx, const struct firmware *fw,
		      const char *firmware, void *mem_region, phys_addr_t *reloc_base)
{
	int ret;

	ret = __qcom_mdt_pas_init(ctx->dev, fw, firmware, ctx->pas_id, ctx->mem_phys, ctx);
	if (ret)
		return ret;

	return qcom_mdt_load_no_init(ctx->dev, fw, firmware, mem_region, ctx->mem_phys,
				     ctx->mem_size, reloc_base);
}
EXPORT_SYMBOL_GPL(qcom_mdt_pas_load);

MODULE_DESCRIPTION("Firmware parser for Qualcomm MDT format");
MODULE_LICENSE("GPL v2");
