// SPDX-License-Identifier: GPL-2.0-only
/*
 * QSEECOM compatibility client over smcinvoke (QTEE), kernel side.
 *
 * On SM8650 the legacy QSEECOM SMC interface (owner 50 / service 1) used by
 * qcom_scm_qseecom_app_load() is not what the secure world serves: loading a
 * QSEECOM trusted application and talking to it goes through the smcinvoke
 * object interface instead, exactly as the vendor driver does:
 *
 *     IClientEnv_registerWithCredentials(root, NULL)      -> client env
 *     IClientEnv_open(client_env, 122)                    -> app loader
 *     IQSEEComCompatAppLoader_lookupTA / loadFromBuffer   -> app controller
 *     IQSEEComCompat_sendRequest(app controller, ...)     -> TA command
 *
 * QTEE already provides all of this to the kernel through the qcomtee
 * object API (qcomtee_object_get_client_env()/get_service()/do_invoke());
 * this file only wires the QSEECOM-compat object UID and its operations on
 * top of it, and exposes a debugfs probe to exercise it.
 *
 * Every operation builds a fresh invoke context, like the driver does for its
 * own QTEE calls: reusing the driver's context would drag in the flags left
 * behind by asynchronous object releases and change how the invocation is
 * carried out.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/debugfs.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/soc/qcom/mdt_loader.h>
#include <linux/uaccess.h>

#include "qcomtee.h"
#include "qcomtee_object.h"

/* UID of the CQSEEComCompatAppLoader service object in QTEE. */
#define QSEECOM_COMPAT_APPLOADER_UID	122

/* IQSEEComCompatAppLoader operations. */
#define QSEECOM_APPLOADER_OP_loadFromRegion	0
#define QSEECOM_APPLOADER_OP_loadFromBuffer	1
#define QSEECOM_APPLOADER_OP_lookupTA		2

static struct qcomtee  *qseecom_qtee;
static struct dentry   *qseecom_dbg;

static struct qcomtee_object *
qseecom_app_loader(struct qcomtee_object_invoke_ctx *oic,
		   struct qcomtee_object **client_env_out)
{
	struct qcomtee_object *client_env;
	struct qcomtee_object *app_loader;

	client_env = qcomtee_object_get_client_env(oic);
	if (client_env == NULL_QCOMTEE_OBJECT) {
		pr_err("failed to get client env\n");
		return NULL_QCOMTEE_OBJECT;
	}

	app_loader = qcomtee_object_get_service(oic, client_env,
						QSEECOM_COMPAT_APPLOADER_UID);
	if (app_loader == NULL_QCOMTEE_OBJECT) {
		pr_err("failed to open app loader (uid %u)\n",
		       QSEECOM_COMPAT_APPLOADER_UID);
		qcomtee_object_put(client_env);
		return NULL_QCOMTEE_OBJECT;
	}

	*client_env_out = client_env;
	return app_loader;
}

static ssize_t
qseecom_read_name(const char __user *ubuf, size_t len, char *name, size_t namelen)
{
	size_t n = min(len, namelen - 1);

	if (copy_from_user(name, ubuf, n))
		return -EFAULT;

	name[n] = '\0';
	while (n > 0 && (name[n - 1] == '\n' || name[n - 1] == '\r' ||
			 name[n - 1] == ' ' || name[n - 1] == '\t'))
		name[--n] = '\0';

	return n;
}

/* lookupTA: op 2, args IB(name), OO(app controller). */
static ssize_t
qseecom_lookup_write(struct file *file, const char __user *ubuf,
		     size_t len, loff_t *ppos)
{
	struct qcomtee_object_invoke_ctx *oic;
	struct qcomtee_object *client_env = NULL_QCOMTEE_OBJECT;
	struct qcomtee_object *app_loader;
	struct qcomtee_arg u[3] = { 0 };
	char name[64];
	int result = 0;
	int ret;

	if (qseecom_read_name(ubuf, len, name, sizeof(name)) < 0)
		return -EFAULT;
	if (!name[0] || !qseecom_qtee)
		return -EINVAL;

	oic = &qseecom_qtee->oic;
	oic->ctx = qseecom_qtee->ctx;
	oic->flags = 0;
	oic->object = NULL_QCOMTEE_OBJECT;
	oic->errno = 0;

	app_loader = qseecom_app_loader(oic, &client_env);
	if (app_loader == NULL_QCOMTEE_OBJECT) {
		ret = -EIO;
		goto out;
	}

	u[0].type = QCOMTEE_ARG_TYPE_IB;
	u[0].b.addr = name;
	u[0].b.size = strlen(name);
	u[1].type = QCOMTEE_ARG_TYPE_OO;

	ret = qcomtee_object_do_invoke(oic, app_loader,
				       QSEECOM_APPLOADER_OP_lookupTA, u, &result);
	pr_info("lookupTA(%s): ret=%d result=%d controller=%s\n",
		name, ret, result, qcomtee_object_name(u[1].o));

	if (!ret && !result && u[1].o != NULL_QCOMTEE_OBJECT)
		qcomtee_object_put(u[1].o);

	qcomtee_object_put(app_loader);
	qcomtee_object_put(client_env);

out:
	return ret ? ret : (ssize_t)len;
}

/* loadFromBuffer: op 1, args IB(elf), IB(name), OB(distname), OO(controller). */
static ssize_t
qseecom_load_write(struct file *file, const char __user *ubuf,
		   size_t len, loff_t *ppos)
{
	const struct firmware *mdt = NULL;
	struct qcomtee_object_invoke_ctx *oic = NULL;
	struct qcomtee_object *client_env = NULL_QCOMTEE_OBJECT;
	struct qcomtee_object *app_loader;
	struct qcomtee_arg u[5] = { 0 };
	char name[64];
	char fw_name[80];
	char dist_name[64] = { 0 };
	void *img = NULL;
	ssize_t img_len;
	ssize_t got;
	int result = 0;
	int ret;

	if (qseecom_read_name(ubuf, len, name, sizeof(name)) < 0)
		return -EFAULT;
	if (!name[0] || !qseecom_qtee)
		return -EINVAL;

	snprintf(fw_name, sizeof(fw_name), "%s.mdt", name);

	ret = request_firmware(&mdt, fw_name, &qseecom_qtee->teedev->dev);
	if (ret) {
		pr_err("firmware %s not found: %d\n", fw_name, ret);
		return ret;
	}

	img_len = qcom_mdt_get_image_size(mdt);
	if (img_len < 0) {
		ret = img_len;
		goto out_release;
	}

	img = kvmalloc(img_len + SZ_4K, GFP_KERNEL);
	if (!img) {
		ret = -ENOMEM;
		goto out_release;
	}

	got = qcom_mdt_read_image(&qseecom_qtee->teedev->dev, mdt, fw_name,
				  img, img_len);
	release_firmware(mdt);
	mdt = NULL;
	if (got < 0) {
		ret = got;
		goto out_free;
	}

	pr_info("assembled %s image: %zd bytes\n", name, got);

	oic = &qseecom_qtee->oic;
	oic->ctx = qseecom_qtee->ctx;
	oic->flags = 0;
	oic->object = NULL_QCOMTEE_OBJECT;
	oic->errno = 0;

	app_loader = qseecom_app_loader(oic, &client_env);
	if (app_loader == NULL_QCOMTEE_OBJECT) {
		ret = -EIO;
		goto out_free;
	}

	u[0].type = QCOMTEE_ARG_TYPE_IB;
	u[0].b.addr = img;
	u[0].b.size = img_len;
	u[1].type = QCOMTEE_ARG_TYPE_IB;
	u[1].b.addr = name;
	u[1].b.size = strlen(name);
	u[2].type = QCOMTEE_ARG_TYPE_OB;
	u[2].b.addr = dist_name;
	u[2].b.size = sizeof(dist_name);
	u[3].type = QCOMTEE_ARG_TYPE_OO;

	ret = qcomtee_object_do_invoke(oic, app_loader,
				       QSEECOM_APPLOADER_OP_loadFromBuffer, u,
				       &result);
	pr_info("loadFromBuffer(%s): ret=%d result=%d dist_name=%s controller=%s\n",
		name, ret, result, dist_name, qcomtee_object_name(u[3].o));

	if (!ret && !result && u[3].o != NULL_QCOMTEE_OBJECT)
		qcomtee_object_put(u[3].o);

	qcomtee_object_put(app_loader);
	qcomtee_object_put(client_env);

out_free:
	kvfree(img);
out_release:
	if (mdt)
		release_firmware(mdt);

	return ret ? ret : (ssize_t)len;
}

static const struct file_operations qseecom_lookup_fops = {
	.open = simple_open,
	.write = qseecom_lookup_write,
	.llseek = noop_llseek,
};

/*
 * Probe the ROOT registration variants in one go, so a single boot tells us
 * which combination the secure world accepts:
 *   - op 2: IClientEnv_register        (what Android's qseecomd uses)
 *   - op 5: IClientEnv_registerWithCredentials (what the vendor kernel uses)
 * each with a NULL credentials object and with a callback object credentials.
 */
#define QSEECOM_ROOT_OP_REGISTER_WITH_CREDENTIALS	5

static ssize_t
qseecom_clientenv_write(struct file *file, const char __user *ubuf,
			size_t len, loff_t *ppos)
{
	static const u32 ops[] = {
		QCOMTEE_ROOT_OP_REG_WITH_CREDENTIALS,
		QSEECOM_ROOT_OP_REGISTER_WITH_CREDENTIALS,
	};
	struct qcomtee_object_invoke_ctx *oic;
	struct qcomtee_object *client_env;
	int i;
	bool use_cred;

	if (!qseecom_qtee)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(ops); i++) {
		for (use_cred = false; ; use_cred = true) {
			oic = &qseecom_qtee->oic;
			oic->ctx = qseecom_qtee->ctx;
			oic->flags = 0;
			oic->object = NULL_QCOMTEE_OBJECT;
			oic->errno = 0;

			client_env = qcomtee_object_get_client_env_impl(oic,
					ops[i], use_cred);
			pr_info("client_env_test: op=%u cred=%d -> %s\n",
				ops[i], use_cred,
				client_env == NULL_QCOMTEE_OBJECT ?
					"FAIL" : "OK");

			if (client_env != NULL_QCOMTEE_OBJECT)
				qcomtee_object_put(client_env);

			if (use_cred)
				break;
		}
	}

	return len;
}

static const struct file_operations qseecom_clientenv_fops = {
	.open = simple_open,
	.write = qseecom_clientenv_write,
	.llseek = noop_llseek,
};

static const struct file_operations qseecom_load_fops = {
	.open = simple_open,
	.write = qseecom_load_write,
	.llseek = noop_llseek,
};

void qcomtee_qseecom_compat_init(struct qcomtee *qtee)
{
	qseecom_qtee = qtee;
	qseecom_dbg = debugfs_create_dir("qseecom", NULL);
	if (IS_ERR_OR_NULL(qseecom_dbg)) {
		qseecom_dbg = NULL;
		return;
	}

	debugfs_create_file("lookup_ta", 0200, qseecom_dbg, NULL,
			    &qseecom_lookup_fops);
	debugfs_create_file("load_app", 0200, qseecom_dbg, NULL,
			    &qseecom_load_fops);
	debugfs_create_file("client_env_test", 0200, qseecom_dbg, NULL,
			    &qseecom_clientenv_fops);
}

void qcomtee_qseecom_compat_exit(void)
{
	debugfs_remove_recursive(qseecom_dbg);
	qseecom_dbg = NULL;
	qseecom_qtee = NULL;
}
