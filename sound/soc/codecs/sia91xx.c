// SPDX-License-Identifier: GPL-2.0-only
/*
 * ASoC driver for SI-IN SIA91xx digital smart amplifiers as found in
 * OnePlus 13R / Ace 5 (giulia)
 *
 * The chip-revision specific register sequences (scene defaults, startup,
 * shutdown) are not publicly documented. They are read from the vendor
 * parameter blob "sipa.bin". It is loaded lazily at first playback so
 * a built-in driver works even when firmware is not available at probe time.
 *
 * Based on the downstream sipa driver:
 *   Copyright (C) 2018 SI-IN, Yun Shi (yun.shi@si-in.com)
 *
 * Copyright (c) 2026, Oleg Peshkov
 */

#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#define SIA91XX_REG_CHIP_ID		0x06

#define SIA91XX_FW_DEFAULT		"sipa.bin"
#define SIA91XX_FW_VERSION		0x00010005
#define SIA91XX_SCENE_NUM		6
#define SIA91XX_SCENE_PLAYBACK		0
#define SIA91XX_CHIP_TYPE_SIA9177	11
#define SIA91XX_CHIP_CFG_SIZE		192
#define SIA91XX_START_RETRIES		5

/* sipa.bin on-disk structures (all fields little-endian u32) */

struct sipa_list {
	__le32 offset;		/* relative to the data section */
	__le32 num;
	__le32 node_size;
} __packed;

struct sipa_reg_common {
	__le32 addr;
	__le32 visible;
	__le32 val[SIA91XX_SCENE_NUM];
} __packed;

enum sipa_action {
	SIPA_REG_READ = 0,
	SIPA_REG_WRITE = 1,
	SIPA_REG_CHECK = 2,
	SIPA_REG_PAD = 3,
};

struct sipa_reg_proc {
	__le32 addr;
	__le32 mask;
	__le32 action;
	__le32 visible;
	__le32 delay;		/* us */
	__le32 val[SIA91XX_SCENE_NUM];
} __packed;

struct sipa_chip_cfg {
	__le32 chip_type;
	__le32 reg_addr_width;
	__le32 reg_val_width;
	__le32 chip_id_addr;
	struct sipa_list chip_id_ranges;
	__le32 owi_mode[SIA91XX_SCENE_NUM];
	__le32 chip_en_addr;
	__le32 chip_en_mask;
	__le32 chip_en_val;
	struct sipa_list init;		/* struct sipa_reg_common */
	struct sipa_list startup;	/* struct sipa_reg_proc */
	struct sipa_list shutdown;	/* struct sipa_reg_proc */
	__le32 trim_crc_width;
	struct sipa_list trim_efuse;
	struct sipa_list trim_crc;
	struct sipa_list trim_default;
	__le32 pvdd_valid_begin;
	__le32 pvdd_valid_end;
	__le32 pvdd_bit_offset;
	__le32 pvdd_reg_addr;
	__le32 pvdd_reg_mask;
	__le32 pvdd_reg_val;
	__le32 sram_cfg_addr;
	__le32 sram_data_addr;
	struct sipa_list sram;
	__le32 en_dyn_ud_vdd;
	__le32 en_dyn_ud_pvdd;
} __packed;

static_assert(sizeof(struct sipa_chip_cfg) == SIA91XX_CHIP_CFG_SIZE);
static_assert(sizeof(struct sipa_reg_common) == 32);
static_assert(sizeof(struct sipa_reg_proc) == 44);

struct sia91xx {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;	/* asserted (1) = amplifier shut down */
	u32 channel;
	const char *fw_name;

	struct mutex lock;
	bool fw_ready;
	bool powered;

	const struct sipa_reg_common *init;
	unsigned int init_num;
	const struct sipa_reg_proc *startup;
	unsigned int startup_num;
	const struct sipa_reg_proc *shutdown;
	unsigned int shutdown_num;
};

static void sia91xx_hw_reset(struct sia91xx *sia)
{
	gpiod_set_value_cansleep(sia->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(sia->reset_gpio, 0);
	usleep_range(5000, 6000);
}

static int sia91xx_check_chip_id(struct sia91xx *sia)
{
	unsigned int id;
	int ret;

	ret = regmap_read(sia->regmap, SIA91XX_REG_CHIP_ID, &id);
	if (ret)
		return ret;

	/* known SIA9177 revisions */
	if (id == 0x5880 || (id >= 0x5890 && id <= 0x5891) ||
	    (id >= 0x58a0 && id <= 0x58a1)) {
		dev_info(sia->dev, "SIA9177 detected, chip id 0x%04x\n", id);
		return 0;
	}

	dev_err(sia->dev, "unknown chip id 0x%04x\n", id);
	return -ENODEV;
}

static int sia91xx_dup_list(struct sia91xx *sia, const u8 *data, u32 data_size,
			    const struct sipa_list *list, size_t elem_size,
			    const void **out, unsigned int *out_num)
{
	u32 n = le32_to_cpu(list->num);
	u32 off = le32_to_cpu(list->offset);
	void *p;

	*out = NULL;
	*out_num = 0;

	if (!n)
		return 0;

	if (le32_to_cpu(list->node_size) != elem_size ||
	    off > data_size || (u64)n * elem_size > data_size - off)
		return -EINVAL;

	p = devm_kmemdup(sia->dev, data + off, n * elem_size, GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	*out = p;
	*out_num = n;
	return 0;
}

static int sia91xx_parse_fw_layout(struct sia91xx *sia, const u8 *buf,
				   size_t size, unsigned int nch)
{
	/* crc + version + ch_en[nch] + chip_cfg lists + extra cfg + data_size */
	size_t hdr = 12 + 44 * (size_t)nch;
	const struct sipa_chip_cfg *cfg = NULL;
	const struct sipa_list *cfg_list;
	u32 data_size, num, node_size, offset;
	const u8 *data;
	unsigned int i;
	int ret;

	if (sia->channel >= nch || size < hdr)
		return -EINVAL;

	data_size = get_unaligned_le32(buf + hdr - 4);
	if (data_size > size - hdr)
		return -EINVAL;
	data = buf + hdr;

	if (get_unaligned_le32(buf + 8 + 4 * sia->channel) != 1) {
		dev_err(sia->dev, "channel %u not enabled in %s\n",
			sia->channel, sia->fw_name);
		return -ENOENT;
	}

	cfg_list = (const struct sipa_list *)(buf + 8 + 4 * nch) + sia->channel;
	num = le32_to_cpu(cfg_list->num);
	node_size = le32_to_cpu(cfg_list->node_size);
	offset = le32_to_cpu(cfg_list->offset);
	if (node_size != SIA91XX_CHIP_CFG_SIZE || offset > data_size ||
	    (u64)num * node_size > data_size - offset)
		return -EINVAL;

	for (i = 0; i < num; i++) {
		const struct sipa_chip_cfg *c =
			(const struct sipa_chip_cfg *)(data + offset) + i;

		if (le32_to_cpu(c->chip_type) == SIA91XX_CHIP_TYPE_SIA9177) {
			cfg = c;
			break;
		}
	}
	if (!cfg) {
		dev_err(sia->dev, "no SIA9177 config for channel %u in %s\n",
			sia->channel, sia->fw_name);
		return -ENOENT;
	}

	ret = sia91xx_dup_list(sia, data, data_size, &cfg->init,
			       sizeof(struct sipa_reg_common),
			       (const void **)&sia->init, &sia->init_num);
	if (ret)
		return ret;

	ret = sia91xx_dup_list(sia, data, data_size, &cfg->startup,
			       sizeof(struct sipa_reg_proc),
			       (const void **)&sia->startup, &sia->startup_num);
	if (ret)
		return ret;

	ret = sia91xx_dup_list(sia, data, data_size, &cfg->shutdown,
			       sizeof(struct sipa_reg_proc),
			       (const void **)&sia->shutdown, &sia->shutdown_num);
	if (ret)
		return ret;

	dev_info(sia->dev,
		 "%s: ch %u (%u slots): %u init, %u startup, %u shutdown regs\n",
		 sia->fw_name, sia->channel, nch, sia->init_num,
		 sia->startup_num, sia->shutdown_num);
	return 0;
}

static int sia91xx_load_fw(struct sia91xx *sia)
{
	const struct firmware *fw;
	u32 crc, ver;
	int ret;

	if (sia->fw_ready)
		return 0;

	ret = request_firmware(&fw, sia->fw_name, sia->dev);
	if (ret) {
		dev_err(sia->dev, "failed to load %s: %d\n", sia->fw_name, ret);
		return ret;
	}

	if (fw->size < 12) {
		ret = -EINVAL;
		goto out;
	}

	ver = get_unaligned_le32(fw->data + 4);
	if (ver != SIA91XX_FW_VERSION)
		dev_warn(sia->dev, "%s version 0x%08x, expected 0x%08x\n",
			 sia->fw_name, ver, SIA91XX_FW_VERSION);

	crc = get_unaligned_le32(fw->data);
	if (crc != ~crc32_le(~0, fw->data + 4, fw->size - 4))
		dev_warn(sia->dev, "%s CRC mismatch, using it anyway\n",
			 sia->fw_name);

	/* current blobs carry 8 channel slots, older ones 4 */
	ret = sia91xx_parse_fw_layout(sia, fw->data, fw->size, 8);
	if (ret)
		ret = sia91xx_parse_fw_layout(sia, fw->data, fw->size, 4);
	if (ret)
		dev_err(sia->dev, "cannot parse %s: %d\n", sia->fw_name, ret);
	else
		sia->fw_ready = true;
out:
	release_firmware(fw);
	return ret;
}

static int sia91xx_run_seq(struct sia91xx *sia,
			   const struct sipa_reg_proc *regs, unsigned int num)
{
	unsigned int i, val, delay;
	int ret;

	for (i = 0; i < num; i++) {
		const struct sipa_reg_proc *r = &regs[i];
		u32 addr = le32_to_cpu(r->addr);
		u32 mask = le32_to_cpu(r->mask);
		u32 want = le32_to_cpu(r->val[SIA91XX_SCENE_PLAYBACK]);

		switch (le32_to_cpu(r->action)) {
		case SIPA_REG_WRITE:
			ret = regmap_update_bits(sia->regmap, addr, mask, want);
			if (ret)
				return ret;
			break;
		case SIPA_REG_READ:
			ret = regmap_read(sia->regmap, addr, &val);
			if (ret)
				return ret;
			break;
		case SIPA_REG_CHECK:
			ret = regmap_read(sia->regmap, addr, &val);
			if (ret)
				return ret;
			if ((val & mask) != (want & mask)) {
				dev_dbg(sia->dev,
					"check failed: reg 0x%02x = 0x%04x, want 0x%04x mask 0x%04x\n",
					addr, val, want, mask);
				return -EAGAIN;
			}
			break;
		case SIPA_REG_PAD:
			break;
		default:
			return -EINVAL;
		}

		delay = le32_to_cpu(r->delay);
		if (delay)
			usleep_range(delay, delay + 100);
	}

	return 0;
}

static int sia91xx_power_up(struct sia91xx *sia)
{
	unsigned int i, attempt;
	int ret;

	ret = sia91xx_load_fw(sia);
	if (ret)
		return ret;

	if (sia->powered)
		return 0;

	if (gpiod_get_value_cansleep(sia->reset_gpio)) {
		gpiod_set_value_cansleep(sia->reset_gpio, 0);
		usleep_range(5000, 6000);
	}

	/* scene defaults ("init" list): plain register writes */
	for (i = 0; i < sia->init_num; i++) {
		const struct sipa_reg_common *r = &sia->init[i];

		ret = regmap_write(sia->regmap, le32_to_cpu(r->addr),
				   le32_to_cpu(r->val[SIA91XX_SCENE_PLAYBACK]));
		if (ret)
			return ret;
	}

	ret = -EINVAL;
	for (attempt = 0; attempt < SIA91XX_START_RETRIES; attempt++) {
		ret = sia91xx_run_seq(sia, sia->startup, sia->startup_num);
		if (!ret)
			break;
	}
	if (ret) {
		dev_err(sia->dev, "failed to start amplifier: %d\n", ret);
		return ret;
	}

	/* sipa.bin's TDM_CFG assumes the vendor ADSP frame format;
	 * mainline LPAIF emits standard I2S (1-bit delay). */
	ret = regmap_update_bits(sia->regmap, 0x14, 0x0030, 0x0000); /* 0x93a8 -> 0x9388 */

	sia->powered = true;
	return 0;
}

static int sia91xx_power_down(struct sia91xx *sia)
{
	int ret = 0;

	if (!sia->powered)
		return 0;

	if (sia->fw_ready)
		ret = sia91xx_run_seq(sia, sia->shutdown, sia->shutdown_num);

	/* hold the chip in shutdown between streams, like downstream does */
	gpiod_set_value_cansleep(sia->reset_gpio, 1);
	sia->powered = false;

	return ret;
}

static int sia91xx_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct sia91xx *sia = snd_soc_component_get_drvdata(dai->component);
	int ret;

	if (stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	mutex_lock(&sia->lock);
	if (mute)
		ret = sia91xx_power_down(sia);
	else
		ret = sia91xx_power_up(sia);
	mutex_unlock(&sia->lock);

	return ret;
}

static int sia91xx_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S &&
	    (fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_DSP_A)
		return -EINVAL;

	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_CBC_CFC)
		return -EINVAL;

	return 0;
}

static const struct snd_soc_dai_ops sia91xx_dai_ops = {
	.mute_stream = sia91xx_mute_stream,
	.set_fmt = sia91xx_set_fmt,
	.no_capture_mute = 1,
};

static struct snd_soc_dai_driver sia91xx_dai = {
	.name = "sia91xx-aif",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &sia91xx_dai_ops,
};

static const struct snd_soc_dapm_widget sia91xx_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("AIF IN", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("OUT"),
};

static const struct snd_soc_dapm_route sia91xx_dapm_routes[] = {
	{ "OUT", NULL, "AIF IN" },
};

static const struct snd_soc_component_driver sia91xx_component = {
	.dapm_widgets = sia91xx_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(sia91xx_dapm_widgets),
	.dapm_routes = sia91xx_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(sia91xx_dapm_routes),
	.endianness = 1,
};

static const struct regmap_config sia91xx_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0xff,
	.cache_type = REGCACHE_NONE,
};

static int sia91xx_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sia91xx *sia;
	int ret;

	sia = devm_kzalloc(dev, sizeof(*sia), GFP_KERNEL);
	if (!sia)
		return -ENOMEM;

	sia->dev = dev;
	mutex_init(&sia->lock);
	i2c_set_clientdata(client, sia);

	sia->regmap = devm_regmap_init_i2c(client, &sia91xx_regmap_config);
	if (IS_ERR(sia->regmap))
		return PTR_ERR(sia->regmap);

	/* asserted = amplifier shut down; deasserted = running */
	sia->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sia->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(sia->reset_gpio),
				     "failed to get reset gpio\n");

	if (device_property_read_u32(dev, "si,channel", &sia->channel))
		sia->channel = 0;

	sia->fw_name = SIA91XX_FW_DEFAULT;
	device_property_read_string(dev, "firmware-name", &sia->fw_name);

	sia91xx_hw_reset(sia);

	ret = sia91xx_check_chip_id(sia);
	if (ret)
		return ret;

	/* keep it in shutdown until playback starts */
	gpiod_set_value_cansleep(sia->reset_gpio, 1);

	return devm_snd_soc_register_component(dev, &sia91xx_component,
					       &sia91xx_dai, 1);
}

static const struct of_device_id sia91xx_of_match[] = {
	{ .compatible = "si,sia9177" },
	{ }
};
MODULE_DEVICE_TABLE(of, sia91xx_of_match);

static const struct i2c_device_id sia91xx_i2c_id[] = {
	{ "sia9177" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sia91xx_i2c_id);

static struct i2c_driver sia91xx_i2c_driver = {
	.driver = {
		.name = "sia91xx",
		.of_match_table = sia91xx_of_match,
	},
	.probe = sia91xx_i2c_probe,
	.id_table = sia91xx_i2c_id,
};
module_i2c_driver(sia91xx_i2c_driver);

MODULE_AUTHOR("Oleg Peshkov <olegos.wst@gmail.com>");
MODULE_DESCRIPTION("ASoC SI-IN SIA91xx smart amplifier driver");
MODULE_FIRMWARE(SIA91XX_FW_DEFAULT);
MODULE_LICENSE("GPL");
