// SPDX-License-Identifier: GPL-2.0-only
/*
 * TEE driver for QSEE applications reached over the legacy QSEECOM interface.
 *
 * Copyright (C) 2026 Dawid Wróbel <me@dawidwrobel.com>
 *
 * QSEECOM predates smcinvoke and is not GlobalPlatform: an application is
 * named by a string rather than a UUID, and it has no notion of function IDs
 * or typed parameters. A command is simply a buffer the application knows how
 * to parse, sent with a second buffer for it to write its answer into. That
 * maps onto TEE_IOC_INVOKE with two memrefs and nothing else.
 *
 * CAVEAT, and the one part of this most likely to need changing: applications
 * are named by short ASCII strings, and TEE_IOC_OPEN_SESSION offers a 16-byte
 * UUID. This driver takes the name as the first parameter -- a memref holding
 * a NUL-terminated string -- and requires the UUID field to be zero. That is a
 * deliberate deviation from GlobalPlatform's rules for what open_session
 * parameters mean, which is defensible only because the driver does not claim
 * TEE_GEN_CAP_GP.
 *
 * The alternative considered was deriving a UUID from the name. It was
 * rejected: it would make sessions impossible to correlate with the names TZ,
 * Android's tooling and the firmware itself all use. The remaining
 * alternative -- a QSEECOM-private ioctl -- is new uapi however it is phrased.
 * This is worth settling before anyone builds on the ABI.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/capability.h>
#include <linux/cleanup.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/tee_core.h>
#include <linux/tee_drv.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/wait.h>

#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/firmware/qcom/qcom_tzmem.h>
#include <linux/soc/qcom/mdt_loader.h>

#define QSEECOM_TEE_MAX_APP_NAME	64



/*
 * TZ only accepts buffers it can reach, which on this interface means
 * qcom_tzmem memory -- SHM Bridge, where the platform has it. The sizes are
 * nothing like control-message sized: the fingerprint TA this was first
 * written against uses a 320 KiB buffer for a single command, and loading an
 * application means handing over its whole image, which for that one is just
 * under 5 MiB.
 */
#define QSEECOM_TEE_POOL_INITIAL	SZ_1M
#define QSEECOM_TEE_POOL_INCREMENT	SZ_1M
#define QSEECOM_TEE_POOL_MAX		SZ_32M

/* Physical alignment TZ requires of an application image. */
#define QSEECOM_TEE_IMAGE_ALIGN		SZ_4M

/* Sanity bound on one invoke's staging buffer, request plus response plus
 * every patched buffer.
 */
#define QSEECOM_TEE_MAX_XFER		SZ_16M

/* Sanity bound on an application image, so a bad size cannot exhaust memory. */
#define QSEECOM_TEE_MAX_IMAGE		SZ_16M

/* Addresses that may be patched into one request. Downstream allows four. */
#define QSEECOM_TEE_MAX_PATCH		4

/*
 * How long to wait for a supplicant to answer before giving up on it. An
 * application is blocked in the secure world for this whole time, and so is
 * every other QSEECOM user, so it cannot be generous. Answering late is not
 * useful either: TZ has its own patience and the watchdog behind it.
 */
#define QSEECOM_TEE_SUPP_TIMEOUT	secs_to_jiffies(10)

/**
 * struct qseecom_tee_supp - Supplicant state.
 * @mutex:    Protects the request handshake.
 * @wq:       Waited on by both sides of that handshake.
 * @req:      Request waiting to be picked up or answered, if any.
 * @taken:    @req has been handed to a supplicant and is awaiting its answer.
 * @pending:  A request is posted. Readable without @mutex, for the wait.
 * @req_gen:  Bumped for every request posted.
 * @taken_gen: The generation the supplicant last took.
 * @answered: A supplicant has answered @req.
 * @status:   That answer.
 * @users:    Number of open contexts on the supplicant device.
 * @recv_ctx: The context serving requests, claimed by the first receive.
 *
 * Only ever one request is in flight: they originate in
 * qcom_scm_qseecom_call(), which holds the QSEECOM call lock for the whole
 * exchange, so a second one cannot start until this one is answered. That is
 * what lets this be a single slot rather than a queue.
 */
struct qseecom_tee_supp {
	struct mutex mutex;
	wait_queue_head_t wq;
	struct qseecom_tee_listener *req;
	bool taken;
	bool pending;
	u32 req_gen;
	u32 taken_gen;
	bool answered;
	u32 status;
	unsigned int users;
	struct tee_context *recv_ctx;
};

/**
 * struct qseecom_tee - Driver instance.
 * @dev:         Underlying device.
 * @teedev:      Client device, /dev/teeX.
 * @supp_teedev: Supplicant device, /dev/teeprivX.
 * @pool:        Shared memory pool handed to the TEE subsystem. Ordinary
 *               cacheable memory, mapped to user space and never to TZ.
 * @mempool:     TZ memory. Kernel-only: everything TZ reads or writes is
 *               staged through here, and none of it is ever mapped to user
 *               space.
 * @apps_lock:   Protects @apps and @next_app_gen.
 * @next_app_gen: Counter stamped into each registry entry, so a session can
 *               tell the application it was opened on from a later one that
 *               TZ gave the same ID.
 * @apps:        Applications this driver loaded, since TZ will not resolve
 *               them by name afterwards.
 * @supp:        Supplicant state.
 */
struct qseecom_tee {
	struct device *dev;
	struct tee_device *teedev;
	struct tee_device *supp_teedev;
	struct tee_shm_pool *pool;
	struct qcom_tzmem_pool *mempool;
	struct qseecom_tee_supp supp;
	struct mutex apps_lock;
	u32 next_app_gen;
	struct list_head apps;
};

/**
 * struct qseecom_tee_listener - A listener service offered by a supplicant.
 * @scm:  Registration held by the SCM layer.
 * @node: Entry in the owning context's listener list.
 * @shm:  The supplicant's buffer, which it mmaps. Requests are copied into it
 *        and replies copied out of it. Referenced for as long as the listener
 *        is registered.
 * @buf:  TZ memory actually registered with the secure world, and the only
 *        one it ever touches. Kernel-only: handing the supplicant's mapped
 *        buffer to TZ would alias cacheable and non-cacheable views of the
 *        same memory.
 * @len:  Size of both, which is what was registered.
 * @qtee: Driver instance.
 */
struct qseecom_tee_listener {
	struct qcom_scm_qseecom_listener scm;
	struct list_head node;
	struct tee_shm *shm;
	void *buf;
	size_t len;
	struct qseecom_tee *qtee;
};

/**
 * struct qseecom_tee_session - One session held by a context.
 * @node:     Entry in the owning context's session list.
 * @id:       Session ID handed to user space.
 * @app_id:   QSEE application ID, for an application session.
 * @app_gen:  Generation of the registry entry this was opened on. TZ reuses
 *            application IDs, so the ID alone does not identify it.
 * @listener: The service offered, for a listener session.
 *
 * Exactly one of @app_id and @listener is set. IDs are handed out from a
 * per-context counter rather than reusing the application or listener ID,
 * because the privileged device carries both kinds and their ID spaces
 * overlap -- both are small integers.
 */
struct qseecom_tee_session {
	struct list_head node;
	u32 id;
	u32 app_id;
	u32 app_gen;
	struct qseecom_tee_listener *listener;
};

/**
 * struct qseecom_tee_app - An application this driver loaded.
 * @node:   Entry in the driver's registry.
 * @app_id: What TZ called it.
 * @gen:    Generation, distinguishing this entry from a later one that TZ
 *          happened to give the same @app_id.
 * @users:  Sessions referring to this application. The last one out unloads
 *          it; no session owns it.
 * @name:   What it is called.
 *
 * Kept because TZ will not tell us again: qcom_scm_qseecom_app_get_id()
 * resolves only applications the boot chain loaded, and returns -ENOENT for
 * one we loaded ourselves even while it is running and answering commands.
 * So the driver has to remember.
 */
struct qseecom_tee_app {
	struct list_head node;
	u32 app_id;
	u32 gen;
	unsigned int users;
	char name[QSEECOM_TEE_MAX_APP_NAME];
};

/**
 * struct qseecom_tee_context - Per-context state.
 * @qtee:      Driver instance.
 * @mutex:     Protects @sessions, @listeners and @next_session_id.
 * @sessions:  Sessions opened on this context.
 * @listeners: Listeners registered by this context, if it is a supplicant.
 * @next_session_id: Counter session IDs are taken from. Never reused, so a
 *                   stale ID from user space fails to resolve rather than
 *                   silently naming something else.
 * @closed:    Set once the device file has been closed and the context's
 *             listeners withdrawn, so the teardown cannot run twice.
 */
struct qseecom_tee_context {
	struct qseecom_tee *qtee;
	struct mutex mutex;
	struct list_head sessions;
	struct list_head listeners;
	u32 next_session_id;
	bool closed;
};

/*
 * Shared memory.
 *
 * There are two kinds here and they must not be confused:
 *
 *   - this pool, which user space mmaps. Ordinary cacheable pages.
 *   - @mempool, TZ memory from dma_alloc_coherent(), which the kernel maps
 *     non-cacheable and hands to the secure world.
 *
 * Memory of the second kind must never be mapped to user space. Doing so
 * creates two aliases of the same physical memory with different memory
 * types, which on arm64 is not merely slow but architecturally unpredictable,
 * and it showed up as writes from one side being invisible to the other.
 */

static int qseecom_tee_pool_alloc(struct tee_shm_pool *pool,
				  struct tee_shm *shm, size_t size,
				  size_t align)
{
	int ret;

	/*
	 * Ordinary cacheable pages, deliberately *not* TZ memory.
	 *
	 * Nothing TZ reads is ever taken straight from here -- the request,
	 * the response area and every buffer whose address is patched into a
	 * request are copied into kernel-only TZ memory first -- so this is
	 * only ever a staging area shared between the kernel and user space.
	 *
	 * That matters because tee_shm_fop_mmap() maps these pages to user
	 * space with default (cacheable) protection. TZ memory comes from
	 * dma_alloc_coherent() and is mapped non-cacheable in the kernel, so
	 * backing this pool with it produced two mismatched aliases of the
	 * same physical memory: user space wrote through the cache and the
	 * kernel read stale bytes through the uncached alias, which is how a
	 * perfectly good application name arrived here as an empty string.
	 * Allocating cacheable memory makes both views agree.
	 *
	 * The helper rounds the size up to whole pages, which is also what
	 * bounds tee_shm_fop_mmap(): reporting the requested size instead
	 * would make any allocation smaller than a page impossible to map.
	 */
	ret = tee_dyn_shm_alloc_helper(shm, size, align, NULL);
	if (ret)
		return ret;

	/*
	 * Nothing was registered with the secure world, so the shm must not
	 * claim to have been. tee_shm_pool_alloc_res_mem() clears it for the
	 * same reason.
	 */
	shm->flags &= ~TEE_SHM_DYNAMIC;

	return 0;
}

static void qseecom_tee_pool_free(struct tee_shm_pool *pool,
				  struct tee_shm *shm)
{
	tee_dyn_shm_free_helper(shm, NULL);
}

static void qseecom_tee_pool_destroy(struct tee_shm_pool *pool)
{
	kfree(pool);
}

static const struct tee_shm_pool_ops qseecom_tee_pool_ops = {
	.alloc = qseecom_tee_pool_alloc,
	.free = qseecom_tee_pool_free,
	.destroy_pool = qseecom_tee_pool_destroy,
};

static struct tee_shm_pool *qseecom_tee_pool_new(void)
{
	struct tee_shm_pool *pool;

	pool = kzalloc_obj(*pool);
	if (!pool)
		return ERR_PTR(-ENOMEM);

	pool->ops = &qseecom_tee_pool_ops;

	return pool;
}

/*
 * Resolve an application by name and take a reference to it.
 *
 * An application is not owned by whoever loaded it. It stays loaded for as long
 * as something refers to it and is unloaded when the last reference goes, which
 * is the shape amdtee settled on and the only one that survives a client dying
 * without closing anything: a reference tied to a session is dropped by the
 * teardown path no matter how the context goes away, whereas an owner that has
 * to close explicitly leaves the application stranded when it crashes.
 */
static u32 qseecom_tee_app_get(struct qseecom_tee *qtee, const char *name,
			       u32 *gen)
{
	struct qseecom_tee_app *app;
	u32 app_id = 0;

	guard(mutex)(&qtee->apps_lock);

	list_for_each_entry(app, &qtee->apps, node) {
		if (!strcmp(app->name, name)) {
			app_id = app->app_id;
			*gen = app->gen;
			app->users++;
			break;
		}
	}

	return app_id;
}

/*
 * Drop a reference, unloading the application when the last one goes.
 *
 * gen 0 means the boot chain loaded it: this driver has no registry entry for
 * it, does not own it, and must not unload it.
 */
static void qseecom_tee_app_put(struct qseecom_tee *qtee, u32 app_id, u32 gen)
{
	struct qseecom_tee_app *app;

	if (!gen)
		return;

	guard(mutex)(&qtee->apps_lock);

	list_for_each_entry(app, &qtee->apps, node) {
		if (app->app_id != app_id || app->gen != gen)
			continue;

		if (--app->users)
			return;

		if (qcom_scm_qseecom_app_shutdown(app_id))
			dev_err(qtee->dev,
				"app %u could not be unloaded and is stuck until reboot\n",
				app_id);

		list_del(&app->node);
		kfree(app);
		return;
	}
}

/* Context handling. */

static int qseecom_tee_open(struct tee_context *ctx)
{
	struct qseecom_tee_context *ctxdata;

	ctxdata = kzalloc_obj(*ctxdata);
	if (!ctxdata)
		return -ENOMEM;

	mutex_init(&ctxdata->mutex);
	INIT_LIST_HEAD(&ctxdata->sessions);
	INIT_LIST_HEAD(&ctxdata->listeners);
	ctxdata->qtee = tee_get_drvdata(ctx->teedev);
	ctx->data = ctxdata;

	return 0;
}

static void qseecom_tee_release(struct tee_context *ctx)
{
	struct qseecom_tee_context *ctxdata = ctx->data;
	struct qseecom_tee_session *sess, *tmp;

	if (!ctxdata)
		return;

	/*
	 * Every session holds a reference to the application it named, so this
	 * is where a client that died without closing anything gives them back.
	 * The application goes when the last reference does; it is not unloaded
	 * here just because this context is finished with it, since other
	 * contexts -- and in-kernel clients such as uefisecapp -- may still be
	 * using it.
	 */
	list_for_each_entry_safe(sess, tmp, &ctxdata->sessions, node) {
		list_del(&sess->node);
		qseecom_tee_app_put(ctxdata->qtee, sess->app_id, sess->app_gen);
		kfree(sess);
	}

	mutex_destroy(&ctxdata->mutex);
	kfree(ctxdata);
	ctx->data = NULL;
}

static struct qseecom_tee_session *
qseecom_tee_session_find(struct qseecom_tee_context *ctxdata, u32 session_id)
{
	struct qseecom_tee_session *sess;

	lockdep_assert_held(&ctxdata->mutex);

	list_for_each_entry(sess, &ctxdata->sessions, node) {
		if (sess->id == session_id)
			return sess;
	}

	return NULL;
}

/*
 * Add a session for @app_id or @listener, whichever is given, and hand back
 * its ID.
 */
static int qseecom_tee_session_add(struct qseecom_tee_context *ctxdata,
				   u32 app_id, u32 app_gen,
				   struct qseecom_tee_listener *listener,
				   u32 *session_id)
{
	struct qseecom_tee_session *sess;

	sess = kzalloc_obj(*sess);
	if (!sess)
		return -ENOMEM;

	sess->app_id = app_id;
	sess->app_gen = app_gen;
	sess->listener = listener;

	mutex_lock(&ctxdata->mutex);
	sess->id = ++ctxdata->next_session_id;
	list_add_tail(&sess->node, &ctxdata->sessions);
	mutex_unlock(&ctxdata->mutex);

	*session_id = sess->id;

	return 0;
}

/* Applications this driver loaded, since TZ will not look them up by name. */
/*
 * Is the application a session was opened against still the same one?
 *
 * TZ hands out application ids as small integers and reuses them, so an id on
 * its own does not identify anything for longer than the application lives.
 * Sessions therefore record the generation of the registry entry they resolved,
 * and a generation is never reused. gen 0 means the session named an
 * application the boot chain loaded, which this driver cannot unload and which
 * therefore cannot be replaced underneath it.
 */
static bool qseecom_tee_app_current(struct qseecom_tee *qtee, u32 app_id,
				    u32 gen)
{
	struct qseecom_tee_app *app;

	if (!gen)
		return true;

	guard(mutex)(&qtee->apps_lock);

	list_for_each_entry(app, &qtee->apps, node)
		if (app->app_id == app_id)
			return app->gen == gen;

	return false;
}

static int qseecom_tee_app_remember(struct qseecom_tee *qtee, const char *name,
				    u32 app_id, u32 *gen)
{
	struct qseecom_tee_app *app, *old;

	app = kzalloc_obj(*app);
	if (!app)
		return -ENOMEM;

	app->app_id = app_id;
	app->users = 1;
	strscpy(app->name, name, sizeof(app->name));

	mutex_lock(&qtee->apps_lock);

	/* gen 0 means "boot-loaded, cannot be replaced"; never hand it out. */
	do {
		app->gen = ++qtee->next_app_gen;
	} while (!app->gen);

	if (gen)
		*gen = app->gen;

	/*
	 * A name identifies one application, so an existing entry under this
	 * name is stale -- the application it referred to is gone, whether we
	 * unloaded it or something else did. Replace it rather than appending,
	 * because lookups take the first match and a stale entry would
	 * otherwise shadow the live one for good, failing every command with
	 * an id TZ no longer knows.
	 *
	 * An id is just as identifying, and for the same reason: TZ hands out
	 * small integers and reuses them, so an application unloaded out of
	 * band and a different one loaded afterwards can arrive here sharing an
	 * id. Evicting on either key keeps at most one entry per id, which is
	 * what makes an id lookup answer about the application that holds it
	 * now rather than about a dead one that got there first.
	 */
	list_for_each_entry(old, &qtee->apps, node) {
		if (!strcmp(old->name, name) || old->app_id == app_id) {
			list_del(&old->node);
			kfree(old);
			break;
		}
	}

	list_add_tail(&app->node, &qtee->apps);
	mutex_unlock(&qtee->apps_lock);

	return 0;
}

/*
 * Sessions.
 *
 * QSEECOM names applications with a string, and TEE_IOC_OPEN_SESSION offers a
 * 16-byte UUID. Rather than pretend a name is a UUID -- it is not, and it does
 * not fit -- the name is passed as the first parameter, a memref holding a
 * NUL-terminated string. The UUID field is reserved and must be zero, leaving
 * it free to be given a meaning later.
 *
 * This is legitimate here only because the driver does not claim
 * TEE_GEN_CAP_GP, so GlobalPlatform's rules about what open_session parameters
 * mean do not apply.
 */

static bool qseecom_tee_uuid_is_null(const u8 *uuid)
{
	return !memchr_inv(uuid, 0, TEE_IOCTL_UUID_LEN);
}

static int qseecom_tee_get_app_name(struct tee_param *param, char *name,
				    size_t name_len)
{
	size_t size;
	void *va;

	if ((param->attr & TEE_IOCTL_PARAM_ATTR_TYPE_MASK) !=
	    TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT)
		return -EINVAL;

	if (!param->u.memref.shm)
		return -EINVAL;

	size = param->u.memref.size;
	if (!size || size > name_len)
		return -EINVAL;

	va = tee_shm_get_va(param->u.memref.shm, param->u.memref.shm_offs);
	if (IS_ERR(va))
		return PTR_ERR(va);

	if (strnlen(va, size) == size)
		return -EINVAL;	/* not NUL-terminated within the memref */

	/*
	 * The name becomes part of a firmware path, so it must name a file and
	 * not a route out of the firmware directory.
	 */
	if (strchr(va, '/') || !strcmp(va, ".") || !strcmp(va, ".."))
		return -EINVAL;

	strscpy(name, va, name_len);

	return 0;
}

static int qseecom_tee_open_session(struct tee_context *ctx,
				    struct tee_ioctl_open_session_arg *arg,
				    struct tee_param *param)
{
	u32 app_gen = 0;
	struct qseecom_tee_context *ctxdata = ctx->data;
	char app_name[QSEECOM_TEE_MAX_APP_NAME];
	u32 app_id;
	int ret;

	if (!qseecom_tee_uuid_is_null(arg->uuid)) {
		dev_dbg(ctxdata->qtee->dev,
			"open_session: uuid is reserved and must be zero\n");
		return -EINVAL;
	}

	if (arg->num_params != 1)
		return -EINVAL;

	ret = qseecom_tee_get_app_name(param, app_name, sizeof(app_name));
	if (ret)
		return ret;

	/*
	 * Applications this driver loaded are looked up in its own registry,
	 * because TZ will not do it: app_get_id() resolves only what the boot
	 * chain loaded and answers -ENOENT for ours even while they are
	 * running. Fall back to asking TZ for the boot-loaded ones.
	 */
	app_id = qseecom_tee_app_get(ctxdata->qtee, app_name, &app_gen);
	if (!app_id) {
		ret = qcom_scm_qseecom_app_get_id(app_name, &app_id);
		if (ret) {
			dev_dbg(ctxdata->qtee->dev,
				"open_session: no app '%s': %d\n", app_name,
				ret);
			return ret;
		}
	}

	ret = qseecom_tee_session_add(ctxdata, app_id, app_gen, NULL,
				      &arg->session);
	if (ret)
		return ret;

	arg->ret = 0;
	arg->ret_origin = 0;

	return 0;
}

static int qseecom_tee_close_session(struct tee_context *ctx, u32 session)
{
	struct qseecom_tee_context *ctxdata = ctx->data;
	struct qseecom_tee_session *sess;

	mutex_lock(&ctxdata->mutex);
	sess = qseecom_tee_session_find(ctxdata, session);
	if (sess)
		list_del(&sess->node);
	mutex_unlock(&ctxdata->mutex);

	if (!sess)
		return -EINVAL;

	qseecom_tee_app_put(ctxdata->qtee, sess->app_id, sess->app_gen);
	kfree(sess);

	return 0;
}

/*
 * Commands.
 *
 * There is no function ID and there are no typed arguments: the application
 * reads a request buffer and writes a response buffer, and everything else is
 * its own protocol. So @func is unused and must be zero, and exactly two
 * memrefs are expected.
 *
 * Both buffers have to be TZ memory, which means they must come from this
 * driver's pool. A registered buffer -- ordinary user pages -- is not
 * reachable by TZ on this interface and is refused rather than silently
 * producing a fault in the secure world.
 */

static int qseecom_tee_memref(struct tee_param *param, u32 want, void **va,
			      size_t *size)
{
	u32 attr = param->attr & TEE_IOCTL_PARAM_ATTR_TYPE_MASK;
	struct tee_shm *shm = param->u.memref.shm;

	if (attr != want && attr != TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT)
		return -EINVAL;

	if (!shm)
		return -EINVAL;

	if (!(shm->flags & TEE_SHM_POOL))
		return -EINVAL;

	if (param->u.memref.size > tee_shm_get_size(shm) ||
	    param->u.memref.shm_offs > tee_shm_get_size(shm) -
				       param->u.memref.size)
		return -EINVAL;

	*va = tee_shm_get_va(shm, param->u.memref.shm_offs);
	if (IS_ERR(*va))
		return PTR_ERR(*va);

	*size = param->u.memref.size;

	return 0;
}

/*
 * Patch the physical address of a buffer into the request.
 *
 * Some applications are not handed everything in the request buffer. They are
 * given the *physical address* of a second buffer, written into the request at
 * an offset their own protocol defines, and they read it from there. The
 * fingerprint application this driver was written against does exactly that,
 * and dereferences the address in the secure world without checking it -- a
 * request built with zero in that field resets the machine.
 *
 * User space cannot fill the field in, since it has no business knowing a
 * physical address, so the kernel does it. This is the same arrangement as
 * the "modified command" in Android's qseecom driver, where ifd_data pairs a
 * buffer with an
 * offset into the request and qseecom patches the resolved address in before
 * the call.
 *
 * Each patch is a pair of parameters: a value giving the offset to write at
 * and the width to write (4 or 8 bytes), followed by the buffer itself.
 *
 * This only checks the pair. The address itself is written further down, once
 * the buffer has been copied into TZ memory -- what belongs in the request is
 * the address of *that* copy, since the caller's buffer is ordinary memory
 * the secure world never sees.
 */
static int qseecom_tee_check_patch(struct tee_param *voff,
				   struct tee_param *pbuf, size_t req_size)
{
	struct tee_shm *shm;
	size_t off, width;

	if ((voff->attr & TEE_IOCTL_PARAM_ATTR_TYPE_MASK) !=
	    TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT)
		return -EINVAL;

	if ((pbuf->attr & TEE_IOCTL_PARAM_ATTR_TYPE_MASK) !=
	    TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT)
		return -EINVAL;

	off = voff->u.value.a;
	width = voff->u.value.b;

	if (width != 4 && width != 8)
		return -EINVAL;

	/*
	 * Both halves matter. Without the first, a request shorter than the
	 * width makes the subtraction wrap and every offset passes, which puts
	 * an attacker-chosen offset into a put_unaligned() further down.
	 */
	if (req_size < width || off > req_size - width)
		return -EINVAL;

	shm = pbuf->u.memref.shm;
	if (!shm || !(shm->flags & TEE_SHM_POOL))
		return -EINVAL;

	if (pbuf->u.memref.size > tee_shm_get_size(shm) ||
	    pbuf->u.memref.shm_offs > tee_shm_get_size(shm) -
				      pbuf->u.memref.size)
		return -EINVAL;

	return 0;
}

static int qseecom_tee_invoke_func(struct tee_context *ctx,
				   struct tee_ioctl_invoke_arg *arg,
				   struct tee_param *param)
{
	struct qseecom_tee_context *ctxdata = ctx->data;
	struct qseecom_tee_session *sess;
	size_t req_size, rsp_size;
	u32 app_gen;
	bool found;
	void *req, *rsp;
	unsigned int i;
	u32 app_id;
	int ret;

	if (arg->func)
		return -EINVAL;

	/*
	 * Two parameters for the request and the response, then a pair per
	 * address to patch into the request.
	 */
	if (arg->num_params < 2 || (arg->num_params & 1) ||
	    arg->num_params > 2 + 2 * QSEECOM_TEE_MAX_PATCH)
		return -EINVAL;

	mutex_lock(&ctxdata->mutex);
	sess = qseecom_tee_session_find(ctxdata, arg->session);
	app_id = sess ? sess->app_id : 0;
	app_gen = sess ? sess->app_gen : 0;
	found = sess;
	mutex_unlock(&ctxdata->mutex);

	if (!found)
		return -EINVAL;

	/*
	 * The application this session names may have been unloaded since,
	 * and TZ reuses application ids. Without this the command would be
	 * delivered to whatever now holds the id -- an unprivileged client's
	 * request arriving at a trusted application it never opened.
	 */
	if (!qseecom_tee_app_current(ctxdata->qtee, app_id, app_gen))
		return -ENOENT;

	ret = qseecom_tee_memref(&param[0], TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT,
				 &req, &req_size);
	if (ret)
		return ret;

	ret = qseecom_tee_memref(&param[1], TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT,
				 &rsp, &rsp_size);
	if (ret)
		return ret;

	for (i = 2; i < arg->num_params; i += 2) {
		ret = qseecom_tee_check_patch(&param[i], &param[i + 1],
					      req_size);
		if (ret)
			return ret;
	}

	{
		struct qcom_tzmem_pool_config bc = {
			.policy = QCOM_TZMEM_POLICY_STATIC,
		};
		struct qcom_tzmem_pool *bp;
		size_t need, off;
		void *b;
		unsigned int k;

		/*
		 * Copy everything TZ will look at into memory only the kernel
		 * has ever touched: the request, the response area, and every
		 * buffer whose address is patched into the request. Buffers
		 * that user space has mapped are written through a cacheable
		 * alias, and TZ does not see those writes.
		 */
		need = req_size + rsp_size;
		for (k = 2; k < arg->num_params; k += 2)
			need += param[k + 1].u.memref.size;

		/*
		 * Every size here is bounded by the shared memory it refers to,
		 * so this should not be reachable -- but the total is what the
		 * staging buffer is sized from, and a wrapped total would size
		 * it far too small for the copies that follow.
		 */
		if (need < req_size || need > QSEECOM_TEE_MAX_XFER)
			return -EINVAL;

		bc.initial_size = PAGE_ALIGN(need) + PAGE_SIZE;
		bc.max_size = bc.initial_size;

		bp = qcom_tzmem_pool_new(&bc);
		if (IS_ERR(bp))
			return PTR_ERR(bp);

		b = qcom_tzmem_alloc(bp, bc.initial_size, GFP_KERNEL);
		if (!b) {
			qcom_tzmem_pool_free(bp);
			return -ENOMEM;
		}

		memset(b, 0, bc.initial_size);
		memcpy(b, req, req_size);

		/* Copy each patched buffer in and repoint the request at it. */
		off = PAGE_ALIGN(req_size + rsp_size);
		for (k = 2; k < arg->num_params; k += 2) {
			struct tee_shm *pshm = param[k + 1].u.memref.shm;
			size_t plen = param[k + 1].u.memref.size;
			size_t poff = param[k].u.value.a;
			size_t pw = param[k].u.value.b;
			phys_addr_t pp;
			void *pv;

			pv = tee_shm_get_va(pshm, param[k + 1].u.memref.shm_offs);
			if (IS_ERR(pv)) {
				/*
				 * Cannot happen -- check_patch() validated
				 * these memrefs -- but skipping the copy would
				 * desynchronise the staging layout from the
				 * copy-back loop *and* leave whatever user
				 * space put at that offset in a field the
				 * application dereferences in the secure
				 * world. Refuse the invoke instead.
				 */
				ret = PTR_ERR(pv);
				goto out_free;
			}

			memcpy(b + off, pv, plen);
			pp = qcom_tzmem_to_phys(b + off);
			if (!pp) {
				ret = -EINVAL;
				goto out_free;
			}

			/*
			 * Four bytes is the usual width and all this platform
			 * needs, the buffers being below 4 GiB, but the field
			 * is eight bytes wide in some protocols. Refuse to
			 * truncate rather than hand over half an address.
			 */
			if (pw == 4) {
				if (upper_32_bits(pp)) {
					ret = -EOVERFLOW;
					goto out_free;
				}
				put_unaligned_le32(lower_32_bits(pp), b + poff);
			} else {
				put_unaligned_le64(pp, b + poff);
			}

			off += plen;
		}

		ret = qcom_scm_qseecom_app_send(app_id, b, req_size,
						b + req_size, rsp_size);
		if (!ret) {
			memcpy(rsp, b + req_size, rsp_size);

			off = PAGE_ALIGN(req_size + rsp_size);
			for (k = 2; k < arg->num_params; k += 2) {
				struct tee_shm *pshm = param[k + 1].u.memref.shm;
				size_t plen = param[k + 1].u.memref.size;
				void *pv;

				pv = tee_shm_get_va(pshm,
						    param[k + 1].u.memref.shm_offs);
				if (!IS_ERR(pv))
					memcpy(pv, b + off, plen);
				off += plen;
			}
		}

out_free:
		qcom_tzmem_free(b);
		qcom_tzmem_pool_free(bp);

		if (ret)
			return ret;
	}

	/*
	 * Not GlobalPlatform return codes: an application's own status lives
	 * inside the response buffer, in whatever form its protocol uses.
	 */
	arg->ret = 0;
	arg->ret_origin = 0;

	return 0;
}

/*
 * Listener services, and the supplicant that serves them.
 *
 * A QSEE application that needs something from the normal world -- reading or
 * writing its secure storage, overwhelmingly -- blocks until it is answered.
 * That is the same arrangement as an OP-TEE RPC to tee-supplicant, so it is
 * served the same way: a privileged device, TEE_IOC_SUPPL_RECV to collect a
 * request and TEE_IOC_SUPPL_SEND to answer it.
 *
 * A supplicant declares which services it offers by opening a session on the
 * privileged device: parameter 0 carries the listener ID and parameter 1 the
 * buffer requests and replies are passed through. Closing the session
 * withdraws the service.
 *
 * The supplicant is not trusted with anything. Secure objects are encrypted
 * and integrity-protected by the application before they ever cross this
 * boundary, which is precisely why the I/O can be delegated to an ordinary
 * process.
 */

static int qseecom_tee_listener_service(struct qcom_scm_qseecom_listener *scm)
{
	struct qseecom_tee_listener *listener =
		container_of(scm, struct qseecom_tee_listener, scm);
	struct qseecom_tee *qtee = listener->qtee;
	struct qseecom_tee_supp *supp = &qtee->supp;
	void *sva;
	long left;
	u32 status;

	sva = tee_shm_get_va(listener->shm, 0);
	if (IS_ERR(sva))
		return QSEECOM_LISTENER_FAILURE;

	scoped_guard(mutex, &supp->mutex) {
		if (!supp->users) {
			dev_warn_once(qtee->dev,
				      "listener %u: no supplicant attached\n",
				      scm->id);
			return QSEECOM_LISTENER_FAILURE;
		}

		/* Show the supplicant the request TZ left in its own buffer. */
		memcpy(sva, listener->buf, listener->len);

		supp->req = listener;
		supp->taken = false;
		supp->req_gen++;
		WRITE_ONCE(supp->pending, true);
		supp->answered = false;
		supp->status = QSEECOM_LISTENER_FAILURE;
	}

	wake_up_interruptible(&supp->wq);

	/*
	 * Deliberately uninterruptible. Whoever issued the command is holding
	 * the QSEECOM call lock and an application is mid-operation in the
	 * secure world; abandoning the exchange on a signal would leave TZ
	 * waiting for an answer that is never coming, which is exactly the
	 * state that wedges the machine.
	 */
	left = wait_event_timeout(supp->wq, READ_ONCE(supp->answered),
				  QSEECOM_TEE_SUPP_TIMEOUT);

	scoped_guard(mutex, &supp->mutex) {
		status = left ? supp->status : QSEECOM_LISTENER_FAILURE;

		/*
		 * Hand the answer to TZ only if the supplicant reported
		 * success. On failure the secure world is told so and does not
		 * read the buffer, and copying a half-written reply into it
		 * would be worse than not answering at all.
		 */
		if (status == QSEECOM_LISTENER_SUCCESS)
			memcpy(listener->buf, sva, listener->len);

		supp->req = NULL;
		supp->taken = false;
		WRITE_ONCE(supp->pending, false);
		supp->answered = false;
	}

	if (!left)
		dev_warn(qtee->dev, "listener %u: supplicant timed out\n",
			 scm->id);

	return status;
}

/*
 * Withdraw a listener and release what it held.
 *
 * The SCM layer only drops a listener from its registry once TZ has confirmed
 * the deregistration; on failure the listener stays on that list, live, with
 * its ->service pointer still reachable. Freeing regardless would hand the
 * secure world a callback into freed memory the next time an application
 * raised a request for that id.
 *
 * So on failure keep everything and say so. Leaking a listener is bad; calling
 * through a freed function pointer from the secure world is worse, and there
 * is no third option while TZ can still name it.
 */
static void qseecom_tee_listener_release(struct qseecom_tee *qtee,
					 struct qseecom_tee_listener *listener)
{
	int ret;

	ret = qcom_scm_qseecom_listener_unregister(&listener->scm);
	if (ret) {
		dev_err(qtee->dev,
			"listener %u: deregistration failed (%d), leaking it -- TZ can still reach it\n",
			listener->scm.id, ret);
		return;
	}

	qcom_tzmem_free(listener->buf);
	tee_shm_put(listener->shm);
	kfree(listener);
}

static int qseecom_tee_supp_recv(struct tee_context *ctx, u32 *func,
				 u32 *num_params, struct tee_param *param)
{
	struct qseecom_tee_context *ctxdata = ctx->data;
	struct qseecom_tee *qtee = ctxdata->qtee;
	struct qseecom_tee_supp *supp = &qtee->supp;
	struct qseecom_tee_listener *listener;
	unsigned int i;
	u32 gen = 0;

	if (*num_params < 2)
		return -EINVAL;

	/* See qseecom_tee_supp_open(): whoever receives first, receives. */
	scoped_guard(mutex, &supp->mutex) {
		if (!supp->recv_ctx)
			supp->recv_ctx = ctx;
		else if (supp->recv_ctx != ctx)
			return -EBUSY;
	}

	/*
	 * The core resolved any memref the caller passed and took a reference
	 * for it, and deliberately does not drop them on this path -- see the
	 * comment on free_params() in tee_ioctl_supp_recv(). Since param[0] is
	 * about to be overwritten wholesale, dropping them is ours to do, and
	 * failing to leaves a reference on a tee_shm, which pins the context,
	 * which strands the device on unregister.
	 *
	 * Nothing meaningful can be passed in, so refuse anything that is not
	 * empty rather than silently discarding it.
	 */
	for (i = 0; i < *num_params; i++)
		if (tee_param_is_memref(param + i) && param[i].u.memref.shm)
			tee_shm_put(param[i].u.memref.shm);

	/*
	 * Only reject once every reference has been dropped. Returning from
	 * inside the loop above would strand the references of every parameter
	 * after the offending one.
	 */
	for (i = 0; i < *num_params; i++)
		if ((param[i].attr & TEE_IOCTL_PARAM_ATTR_TYPE_MASK) !=
		    TEE_IOCTL_PARAM_ATTR_TYPE_NONE)
			return -EINVAL;

	/*
	 * Take the request under the lock, and wait outside it.
	 *
	 * Evaluating a condition that can sleep from inside
	 * wait_event_interruptible() is not allowed: the condition runs with
	 * the task state already set to TASK_INTERRUPTIBLE, and mutex_lock()
	 * calls might_sleep(), which splats under CONFIG_DEBUG_ATOMIC_SLEEP
	 * and can leave the task spinning rather than sleeping. OP-TEE settled
	 * on this shape for the same reason.
	 */
	while (true) {
		scoped_guard(mutex, &supp->mutex) {
			listener = supp->req;
			if (listener && !supp->taken) {
				supp->taken = true;
				supp->taken_gen = supp->req_gen;
				/*
				 * Taken, so no longer waiting to be taken.
				 * Leaving this set spins any other receiver.
				 */
				WRITE_ONCE(supp->pending, false);
				gen = supp->req_gen;
				goto got_request;
			}
		}

		if (wait_event_interruptible(supp->wq,
					     READ_ONCE(supp->pending)))
			return -ERESTARTSYS;
	}

got_request:

	/*
	 * The request is already in the listener's buffer, which the
	 * supplicant registered and therefore already has mapped. It is handed
	 * back as a memref so the supplicant can tell which of its buffers to
	 * look at.
	 */
	*func = listener->scm.id;
	*num_params = 2;

	/*
	 * Which request this is. The supplicant echoes it back in SUPPL_SEND
	 * so a late answer can be told from an answer to the request now
	 * outstanding. A generation the kernel never shows anyone cannot do
	 * that, because the answer would carry nothing to compare against.
	 *
	 * VALUE_INPUT, not VALUE_OUTPUT: direction is from the supplicant's
	 * point of view, and this is the kernel handing it something. It is
	 * also the only thing that works -- params_to_supp() copies a, b and c
	 * for the INPUT and INOUT cases and zeroes everything else, so an
	 * OUTPUT parameter arrives as zero. OP-TEE marks its equivalent the
	 * same way.
	 */
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[0].u.value.a = gen;
	param[0].u.value.b = 0;
	param[0].u.value.c = 0;

	param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	param[1].u.memref.shm = listener->shm;
	param[1].u.memref.shm_offs = 0;
	param[1].u.memref.size = listener->scm.sb_len;

	return 0;
}

static int qseecom_tee_supp_send(struct tee_context *ctx, u32 ret,
				 u32 num_params, struct tee_param *param)
{
	struct qseecom_tee_context *ctxdata = ctx->data;
	struct qseecom_tee *qtee = ctxdata->qtee;
	struct qseecom_tee_supp *supp = &qtee->supp;

	/*
	 * VALUE_OUTPUT here, against the VALUE_INPUT the receive handed out.
	 * The asymmetry is the core's: params_to_supp() only fills a value it
	 * is passing to the supplicant when the parameter is marked INPUT, and
	 * params_from_supp() only reads one back when it is marked OUTPUT --
	 * "only out and in/out values can be updated". Both are named for the
	 * supplicant's direction, so the same number changes attribute as it
	 * turns around. Getting this wrong is silent: the value arrives zeroed
	 * rather than rejected, and every answer then looks stale.
	 */
	if (num_params < 1 ||
	    (param[0].attr & TEE_IOCTL_PARAM_ATTR_TYPE_MASK) !=
	    TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT)
		return -EINVAL;

	scoped_guard(mutex, &supp->mutex) {
		if (supp->recv_ctx != ctx)
			return -EPERM;

		if (!supp->req || !supp->taken)
			return -EINVAL;

		/*
		 * Refuse an answer to a request that is no longer the one
		 * outstanding. A supplicant that overran the timeout comes back
		 * to find its request abandoned and possibly a new one in the
		 * slot; applying the stale answer would hand one application
		 * the reply meant for another, and a bogus success is what
		 * wedges the secure world.
		 *
		 * The generation compared is the one handed out by the receive
		 * that took the request, echoed back by the supplicant. A
		 * purely internal counter cannot do this: taken_gen is set from
		 * req_gen when a request is taken, so the two are equal for as
		 * long as the request is taken and the comparison never fails.
		 */
		if (param[0].u.value.a != supp->req_gen)
			return -ESTALE;

		/*
		 * Anything other than success is reported as failure, and the
		 * distinction genuinely matters: claiming success without
		 * having filled the buffer leaves the application acting on
		 * whatever was in it, which has been observed to hang the
		 * secure world until the watchdog fires.
		 */
		supp->status = ret ? QSEECOM_LISTENER_FAILURE :
				     QSEECOM_LISTENER_SUCCESS;
		supp->answered = true;
	}

	wake_up(&supp->wq);

	return 0;
}

static int qseecom_tee_supp_open(struct tee_context *ctx)
{
	struct qseecom_tee *qtee = tee_get_drvdata(ctx->teedev);
	int ret;

	/*
	 * This device loads code into the secure world and registers the
	 * services trusted applications call back into, so it is not something
	 * to hand out on file permissions alone. The TEE core separates the
	 * privileged device from the client one by minor number and says so in
	 * TEE_IOC_VERSION, but enforces nothing itself, which leaves the whole
	 * guard resting on whatever created the node.
	 */
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	ret = qseecom_tee_open(ctx);
	if (ret)
		return ret;

	/*
	 * One *receiver*, because there is one request slot. A second process
	 * receiving on this device would race the first on TEE_IOC_SUPPL_RECV
	 * and the two would consume each other's requests -- and since a
	 * request is answered by whoever calls SUPPL_SEND next, the wrong reply
	 * would go to the wrong application. The listener a request belongs to
	 * is in arg.func precisely so that one process can serve them all.
	 *
	 * That is enforced at the first receive rather than here, because
	 * opening this device is not the same as serving it: loading an
	 * application is an open_session on this device too, and a loader is
	 * expected to run alongside a supplicant.
	 */
	scoped_guard(mutex, &qtee->supp.mutex)
		qtee->supp.users++;

	return 0;
}

/*
 * Tear a supplicant down when its device file closes, rather than waiting for
 * the context to be released.
 *
 * This has to happen here and not in ->release(), because a listener holds a
 * reference to the buffer it was registered with, and tee_shm holds a
 * reference to the context that allocated it. Dropping the buffer only on
 * release therefore cannot work: the context is what the buffer is keeping
 * alive, so release never runs, the context leaks, and tee_device_unregister()
 * waits for it for ever -- which strands the module in "going" state where
 * even a reload cannot recover it. Only a reboot clears that.
 *
 * ->close_context() is called when the file is closed even though references
 * to the context remain, which is exactly the hook needed to break the cycle.
 */
static void qseecom_tee_supp_close_context(struct tee_context *ctx)
{
	struct qseecom_tee_context *ctxdata = ctx->data;
	struct qseecom_tee_listener *listener, *tmp;
	struct qseecom_tee *qtee;

	/*
	 * Unsynchronised on purpose. ->close_context() runs from tee_release()
	 * with no ioctl in flight on this context, so there is nothing to race
	 * with; the flag exists only because close_context() and release() can
	 * both be reached and the teardown must not run twice.
	 */
	if (!ctxdata || ctxdata->closed)
		return;

	ctxdata->closed = true;
	qtee = ctxdata->qtee;

	/*
	 * Release an in-flight request first, and only then withdraw the
	 * listeners. The order matters: a supplicant that died mid-request has
	 * left a thread blocked in ->service() holding the QSEECOM call lock,
	 * and unregistering needs that same lock. Withdrawing first would mean
	 * waiting out the whole supplicant timeout to do it -- precisely the
	 * delay this is meant to avoid.
	 */
	scoped_guard(mutex, &qtee->supp.mutex) {
		bool was_receiver = qtee->supp.recv_ctx == ctx;

		if (was_receiver)
			qtee->supp.recv_ctx = NULL;

		if (qtee->supp.users)
			qtee->supp.users--;

		if ((was_receiver || !qtee->supp.users) && qtee->supp.req) {
			qtee->supp.status = QSEECOM_LISTENER_FAILURE;
			qtee->supp.answered = true;
		}
	}

	wake_up(&qtee->supp.wq);

	list_for_each_entry_safe(listener, tmp, &ctxdata->listeners, node) {
		list_del(&listener->node);
		qseecom_tee_listener_release(ctxdata->qtee, listener);
	}
}

static void qseecom_tee_supp_release(struct tee_context *ctx)
{
	/*
	 * Everything that holds a reference is gone by now; this only frees
	 * what is left. Called defensively in case a context is released
	 * without its file ever having been closed.
	 */
	qseecom_tee_supp_close_context(ctx);
	qseecom_tee_release(ctx);
}

/*
 * Load an application from firmware the kernel fetches itself, and remember
 * what TZ called it.
 *
 * The caller names an application; it does not supply one. Everything that
 * ends up in TZ comes from request_firmware(), so what may be loaded is
 * whatever the firmware search path holds -- the same contract amdtee has --
 * rather than whatever bytes a process cares to assemble. A signed image is
 * not by itself an argument for taking it from user space: the caller still
 * chooses which signed image, and TZ has no notion of which one the kernel
 * meant to run.
 *
 * qcom_mdt_load() cannot be used as-is, despite handling the same file format.
 * It solves the remoteproc problem: scatter each segment to its p_paddr inside
 * a carveout. QSEECOM's APP_START instead takes one contiguous buffer -- the
 * .mdt followed by the segment payloads, with an mdt length and a total length
 * -- so the two want different things from the same files. Hence
 * qcom_mdt_read_image(), which keeps the format knowledge next to the rest of
 * it in mdt_loader.c instead of growing a second parser here.
 *
 * The program headers cannot be read as a file layout for this image at all.
 * Taking a real one (Goodix gfenu): segment 0 has p_offset 0, which would
 * overwrite the very ELF and program headers being parsed, and segments 6 and
 * 7 declare the same p_offset and the same p_paddr with different sizes, so
 * placing by p_offset makes one overwrite the other. Concatenation in
 * program-header order is what the vendor's own loader does, and it is what
 * produces an image TZ accepts. Confirmed on hardware rather than inferred:
 * assembling the same files by p_offset, with an identical mdt length, gets
 * the image rejected, while the concatenated form loads.
 *
 * That loading is only reachable on the privileged device *is* the access
 * control. Loading puts an image into the same ID space and the same secure
 * storage machinery as every other application, so it is not something an
 * ordinary client should be able to do, and answering "who may load" with
 * "whoever can open /dev/teepriv0" needs no policy of its own.
 */
static int qseecom_tee_supp_load_app(struct tee_context *ctx,
				     struct tee_ioctl_open_session_arg *arg,
				     struct tee_param *param)
{
	u32 app_gen = 0;
	struct qseecom_tee_context *ctxdata = ctx->data;
	struct qseecom_tee *qtee = ctxdata->qtee;
	struct qcom_tzmem_pool_config pool_config = {
		.policy = QCOM_TZMEM_POLICY_STATIC,
	};
	char fw_name[QSEECOM_TEE_MAX_APP_NAME + sizeof(".mdt")];
	char app_name[QSEECOM_TEE_MAX_APP_NAME];
	size_t img_len, mdt_len, stage_len;
	phys_addr_t stage_phys, img_phys;
	const struct firmware *mdt;
	void *stage, *aligned;
	ssize_t len;
	struct qcom_tzmem_pool *pool;
	u32 app_id;
	int ret;

	if (arg->num_params != 1)
		return -EINVAL;

	ret = qseecom_tee_get_app_name(&param[0], app_name, sizeof(app_name));
	if (ret)
		return ret;

	snprintf(fw_name, sizeof(fw_name), "%s.mdt", app_name);

	ret = request_firmware(&mdt, fw_name, qtee->dev);
	if (ret)
		return ret;

	len = qcom_mdt_get_image_size(mdt);
	if (len < 0) {
		ret = len;
		goto out_release;
	}

	if (len > QSEECOM_TEE_MAX_IMAGE) {
		ret = -EFBIG;
		goto out_release;
	}

	img_len = len;
	mdt_len = mdt->size;

	/*
	 * Stage the image through a pool of its own rather than the shared one.
	 * Two reasons, and both were learned the hard way:
	 *
	 * TZ maps the image itself and wants it at a 4 MiB aligned physical
	 * address -- Android's qseecom driver reserves its TA region that way -- so the
	 * buffer has to be big enough to contain an aligned copy. The shared
	 * pool only promises page alignment.
	 *
	 * And an image is *large*: nine megabytes of staging for a five
	 * megabyte application. Taking that from the shared pool grows it
	 * permanently, since it never shrinks, which starves every other user
	 * of the same TZ memory region. A dedicated pool goes away again.
	 */
	stage_len = PAGE_ALIGN(img_len) + QSEECOM_TEE_IMAGE_ALIGN;
	pool_config.initial_size = stage_len;
	pool_config.max_size = stage_len;

	pool = qcom_tzmem_pool_new(&pool_config);
	if (IS_ERR(pool)) {
		ret = PTR_ERR(pool);
		goto out_release;
	}

	stage = qcom_tzmem_alloc(pool, stage_len, GFP_KERNEL);
	if (!stage) {
		qcom_tzmem_pool_free(pool);
		ret = -ENOMEM;
		goto out_release;
	}

	/* The area is physically contiguous, so an offset applies to both. */
	stage_phys = qcom_tzmem_to_phys(stage);
	img_phys = ALIGN(stage_phys, QSEECOM_TEE_IMAGE_ALIGN);
	aligned = stage + (img_phys - stage_phys);

	len = qcom_mdt_read_image(qtee->dev, mdt, fw_name, aligned, img_len);
	if (len < 0)
		ret = len;
	else
		ret = qcom_scm_qseecom_app_load(aligned, mdt_len, img_len,
						&app_id);

	qcom_tzmem_free(stage);
	qcom_tzmem_pool_free(pool);
	release_firmware(mdt);

	if (ret) {
		dev_warn(qtee->dev, "failed to load '%s': %d\n", app_name, ret);
		return ret;
	}

	/*
	 * From here the application is live in TZ. Anything that fails now has
	 * to put it back, because nothing else can: without a registry entry
	 * the name cannot even be resolved again, and TZ would refuse to load
	 * it a second time, leaving it stuck until a reboot.
	 *
	 * The registry entry is created holding one reference, which the
	 * session below takes over. If no session is created, that reference is
	 * what has to be given back.
	 */
	ret = qseecom_tee_app_remember(qtee, app_name, app_id, &app_gen);
	if (ret)
		goto err_shutdown;

	ret = qseecom_tee_session_add(ctxdata, app_id, app_gen, NULL,
				      &arg->session);
	if (ret) {
		qseecom_tee_app_put(qtee, app_id, app_gen);
		return ret;
	}

	dev_info(qtee->dev, "loaded '%s' as app %u\n", app_name, app_id);

	arg->ret = 0;
	arg->ret_origin = 0;

	return 0;

out_release:
	release_firmware(mdt);
	return ret;

err_shutdown:
	if (qcom_scm_qseecom_app_shutdown(app_id))
		dev_err(qtee->dev,
			"app %u loaded but could not be unloaded after a failed load; it is stuck until reboot\n",
			app_id);
	return ret;
}

/*
 * Sessions on the privileged device are one of two things, told apart by the
 * type of the first parameter:
 *
 *   value  -- register a listener service: the ID, and the buffer requests
 *             and replies pass through.
 *   memref -- load an application: its name, its image, and the .mdt length.
 */
static int qseecom_tee_supp_open_session(struct tee_context *ctx,
					 struct tee_ioctl_open_session_arg *arg,
					 struct tee_param *param)
{
	struct qseecom_tee_context *ctxdata = ctx->data;
	struct qseecom_tee *qtee = ctxdata->qtee;
	struct qseecom_tee_listener *listener;
	struct tee_shm *shm;
	void *va;
	u32 id;
	int ret;

	if (!qseecom_tee_uuid_is_null(arg->uuid))
		return -EINVAL;

	if (!arg->num_params)
		return -EINVAL;

	if ((param[0].attr & TEE_IOCTL_PARAM_ATTR_TYPE_MASK) ==
	    TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT)
		return qseecom_tee_supp_load_app(ctx, arg, param);

	if (arg->num_params != 2)
		return -EINVAL;

	if ((param[0].attr & TEE_IOCTL_PARAM_ATTR_TYPE_MASK) !=
	    TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT)
		return -EINVAL;

	if ((param[1].attr & TEE_IOCTL_PARAM_ATTR_TYPE_MASK) !=
	    TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT)
		return -EINVAL;

	if (param[0].u.value.a > U32_MAX)
		return -EINVAL;

	id = param[0].u.value.a;

	shm = param[1].u.memref.shm;
	if (!shm || !(shm->flags & TEE_SHM_POOL))
		return -EINVAL;

	if (param[1].u.memref.shm_offs || !param[1].u.memref.size ||
	    param[1].u.memref.size > tee_shm_get_size(shm))
		return -EINVAL;

	va = tee_shm_get_va(shm, 0);
	if (IS_ERR(va))
		return PTR_ERR(va);

	listener = kzalloc_obj(*listener);
	if (!listener)
		return -ENOMEM;

	/* Held until the listener is withdrawn, so TZ's buffer cannot vanish. */
	if (!tee_shm_get_from_id(ctx, tee_shm_get_id(shm))) {
		kfree(listener);
		return -EINVAL;
	}

	listener->len = param[1].u.memref.size;

	/*
	 * What gets registered with TZ is this buffer, not the supplicant's.
	 * The secure world requires memory it can reach, and it must not be
	 * memory user space has mapped: the two would be aliased with
	 * different cacheability and neither side would reliably see the
	 * other's writes.
	 */
	listener->buf = qcom_tzmem_alloc(qtee->mempool, listener->len,
					 GFP_KERNEL);
	if (!listener->buf) {
		kfree(listener);
		return -ENOMEM;
	}

	listener->qtee = qtee;
	listener->shm = shm;
	listener->scm.id = id;
	listener->scm.sb = listener->buf;
	listener->scm.sb_len = listener->len;
	listener->scm.service = qseecom_tee_listener_service;

	ret = qcom_scm_qseecom_listener_register(&listener->scm);
	if (ret) {
		qcom_tzmem_free(listener->buf);
		tee_shm_put(shm);
		kfree(listener);
		return ret;
	}

	mutex_lock(&ctxdata->mutex);
	list_add_tail(&listener->node, &ctxdata->listeners);
	mutex_unlock(&ctxdata->mutex);

	ret = qseecom_tee_session_add(ctxdata, 0, 0, listener, &arg->session);
	if (ret) {
		mutex_lock(&ctxdata->mutex);
		list_del(&listener->node);
		mutex_unlock(&ctxdata->mutex);
		qseecom_tee_listener_release(qtee, listener);
		return ret;
	}

	arg->ret = 0;
	arg->ret_origin = 0;

	return 0;
}

static int qseecom_tee_supp_close_session(struct tee_context *ctx, u32 session)
{
	struct qseecom_tee_context *ctxdata = ctx->data;
	struct qseecom_tee_listener *listener;
	struct qseecom_tee_session *sess;
	u32 app_id, app_gen;
	int ret;

	mutex_lock(&ctxdata->mutex);
	sess = qseecom_tee_session_find(ctxdata, session);
	listener = sess ? sess->listener : NULL;
	if (sess) {
		list_del(&sess->node);
		if (listener)
			list_del(&listener->node);
	}
	mutex_unlock(&ctxdata->mutex);

	if (!sess)
		return -EINVAL;

	app_id = sess->app_id;
	app_gen = sess->app_gen;
	kfree(sess);

	/*
	 * A load session holds a reference like any other, so closing it gives
	 * that reference back rather than unloading outright. The application
	 * goes when the last reference does -- which may be this one, and may
	 * not: a client can be using it on the other device.
	 *
	 * Nothing owns an application, deliberately. TZ refuses a second load
	 * of one already loaded and only a reboot clears it, so an owner that
	 * has to close explicitly strands the application every time it
	 * crashes. A reference dropped by the teardown path cannot be skipped.
	 *
	 * Sessions do not survive the application either. Each records the
	 * generation of the registry entry it resolved, and an invoke
	 * revalidates that, so a session naming an application that has since
	 * gone fails rather than addressing whatever now holds the reused id.
	 */
	if (!listener) {
		qseecom_tee_app_put(ctxdata->qtee, app_id, app_gen);
		return 0;
	}

	/*
	 * Withdrawing a listener that has a request outstanding will block
	 * here until that request times out, because the thread waiting for an
	 * answer holds the lock this needs. A supplicant that answers before
	 * closing never sees it.
	 */
	qseecom_tee_listener_release(ctxdata->qtee, listener);
	ret = 0;

	return ret;
}

static void qseecom_tee_get_version(struct tee_device *teedev,
				    struct tee_ioctl_version_data *vers)
{
	struct tee_ioctl_version_data v = {
		.impl_id = TEE_IMPL_ID_QSEECOM,
		.impl_caps = 0,
		/*
		 * Deliberately not TEE_GEN_CAP_GP: QSEECOM is not
		 * GlobalPlatform and should not claim to be. Nor
		 * TEE_GEN_CAP_REG_MEM -- TZ can only reach pool memory here.
		 */
		.gen_caps = 0,
	};

	*vers = v;
}

static const struct tee_driver_ops qseecom_tee_ops = {
	.get_version = qseecom_tee_get_version,
	.open = qseecom_tee_open,
	.release = qseecom_tee_release,
	.open_session = qseecom_tee_open_session,
	.close_session = qseecom_tee_close_session,
	.invoke_func = qseecom_tee_invoke_func,
};

static const struct tee_desc qseecom_tee_desc = {
	.name = "qseecom-clnt",
	.ops = &qseecom_tee_ops,
	.owner = THIS_MODULE,
};

static const struct tee_driver_ops qseecom_tee_supp_ops = {
	.get_version = qseecom_tee_get_version,
	.open = qseecom_tee_supp_open,
	.close_context = qseecom_tee_supp_close_context,
	.release = qseecom_tee_supp_release,
	.open_session = qseecom_tee_supp_open_session,
	.close_session = qseecom_tee_supp_close_session,
	.supp_recv = qseecom_tee_supp_recv,
	.supp_send = qseecom_tee_supp_send,
};

static const struct tee_desc qseecom_tee_supp_desc = {
	.name = "qseecom-supp",
	.ops = &qseecom_tee_supp_ops,
	.owner = THIS_MODULE,
	.flags = TEE_DESC_PRIVILEGED,
};

static int qseecom_tee_probe(struct platform_device *pdev)
{
	struct qcom_tzmem_pool_config pool_config = {
		.initial_size = QSEECOM_TEE_POOL_INITIAL,
		.policy = QCOM_TZMEM_POLICY_ON_DEMAND,
		.increment = QSEECOM_TEE_POOL_INCREMENT,
		.max_size = QSEECOM_TEE_POOL_MAX,
	};
	struct device *dev = &pdev->dev;
	struct qseecom_tee *qtee;
	int ret;

	qtee = devm_kzalloc(dev, sizeof(*qtee), GFP_KERNEL);
	if (!qtee)
		return -ENOMEM;

	qtee->dev = dev;
	ret = devm_mutex_init(dev, &qtee->supp.mutex);
	if (ret)
		return ret;

	ret = devm_mutex_init(dev, &qtee->apps_lock);
	if (ret)
		return ret;

	init_waitqueue_head(&qtee->supp.wq);
	INIT_LIST_HEAD(&qtee->apps);

	qtee->mempool = devm_qcom_tzmem_pool_new(dev, &pool_config);
	if (IS_ERR(qtee->mempool))
		return dev_err_probe(dev, PTR_ERR(qtee->mempool),
				     "failed to create TZ memory pool\n");

	qtee->pool = qseecom_tee_pool_new();
	if (IS_ERR(qtee->pool))
		return dev_err_probe(dev, PTR_ERR(qtee->pool),
				     "failed to create shared memory pool\n");

	qtee->teedev = tee_device_alloc(&qseecom_tee_desc, dev, qtee->pool,
					qtee);
	if (IS_ERR(qtee->teedev)) {
		ret = dev_err_probe(dev, PTR_ERR(qtee->teedev),
				    "failed to allocate TEE device\n");
		goto err_free_pool;
	}

	qtee->supp_teedev = tee_device_alloc(&qseecom_tee_supp_desc, dev,
					     qtee->pool, qtee);
	if (IS_ERR(qtee->supp_teedev)) {
		ret = dev_err_probe(dev, PTR_ERR(qtee->supp_teedev),
				    "failed to allocate supplicant device\n");
		goto err_unreg_teedev;
	}

	ret = tee_device_register(qtee->teedev);
	if (ret)
		goto err_unreg_supp_teedev;

	ret = tee_device_register(qtee->supp_teedev);
	if (ret)
		goto err_unreg_supp_teedev;

	platform_set_drvdata(pdev, qtee);

	return 0;

err_unreg_supp_teedev:
	tee_device_unregister(qtee->supp_teedev);
err_unreg_teedev:
	tee_device_unregister(qtee->teedev);
err_free_pool:
	tee_shm_pool_free(qtee->pool);

	return ret;
}

static void qseecom_tee_remove(struct platform_device *pdev)
{
	struct qseecom_tee *qtee = platform_get_drvdata(pdev);
	struct qseecom_tee_app *app, *tmp;

	tee_device_unregister(qtee->supp_teedev);
	tee_device_unregister(qtee->teedev);
	tee_shm_pool_free(qtee->pool);

	/*
	 * The applications themselves stay loaded in TZ -- nothing here asks
	 * for them to be shut down, since something else may be using them.
	 * Only our record of them goes away.
	 */
	list_for_each_entry_safe(app, tmp, &qtee->apps, node) {
		list_del(&app->node);
		kfree(app);
	}
}

static struct platform_driver qseecom_tee_driver = {
	.probe = qseecom_tee_probe,
	.remove = qseecom_tee_remove,
	.driver = {
		.name = "qcom_qseecom_tee",
	},
};
module_platform_driver(qseecom_tee_driver);

MODULE_ALIAS("platform:qcom_qseecom_tee");

MODULE_AUTHOR("Dawid Wróbel <me@dawidwrobel.com>");
MODULE_DESCRIPTION("Qualcomm QSEECOM TEE driver");
MODULE_LICENSE("GPL");
