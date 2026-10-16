// SPDX-License-Identifier: GPL-2.0-only
/*
 * Goodix SPI fingerprint sensor driver
 *
 * Copyright (C) 2016-2017 Goodix
 * Copyright (C) 2022 Xiaomi, Inc.
 * Copyright (C) 2026 Dawid Wróbel <me@dawidwrobel.com>
 *
 * Based on the downstream Android Goodix gf_spi driver.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/regulator/consumer.h>
#include <linux/uaccess.h>
#include <net/netlink.h>
#include <net/sock.h>

#define GF_DEV_NAME "goodix_fp"
#define GF_INPUT_NAME "uinput-goodix"

#define GF_VREG_LOAD_UA 100000
#define WAKELOCK_HOLD_TIME_MS 2000

/* Netlink ABI towards the fingerprint HAL */
#define GF_NETLINK_PROTO 25
#define GF_NET_EVENT_IRQ 1
#define MAX_NL_MSG_LEN 16

enum gf_key_event {
	GF_KEY_NONE = 0,
	GF_KEY_HOME,
	GF_KEY_POWER,
	GF_KEY_MENU,
	GF_KEY_BACK,
	GF_KEY_CAMERA,
};

struct gf_key {
	enum gf_key_event key;
	u32 value; /* key down = 1, key up = 0 */
};

enum gf_nav_event {
	GF_NAV_NONE = 0,
	GF_NAV_FINGER_UP,
	GF_NAV_FINGER_DOWN,
	GF_NAV_UP,
	GF_NAV_DOWN,
	GF_NAV_LEFT,
	GF_NAV_RIGHT,
	GF_NAV_CLICK,
	GF_NAV_HEAVY,
	GF_NAV_LONG_PRESS,
	GF_NAV_DOUBLE_CLICK,
};

struct gf_ioc_chip_info {
	u8 vendor_id;
	u8 mode;
	u8 operation;
	u8 reserved[5];
};

#define GF_IOC_MAGIC 'g'
#define GF_IOC_INIT _IOR(GF_IOC_MAGIC, 0, u8)
#define GF_IOC_EXIT _IO(GF_IOC_MAGIC, 1)
#define GF_IOC_RESET _IO(GF_IOC_MAGIC, 2)
#define GF_IOC_ENABLE_IRQ _IO(GF_IOC_MAGIC, 3)
#define GF_IOC_DISABLE_IRQ _IO(GF_IOC_MAGIC, 4)
#define GF_IOC_ENABLE_SPI_CLK _IOW(GF_IOC_MAGIC, 5, u32)
#define GF_IOC_DISABLE_SPI_CLK _IO(GF_IOC_MAGIC, 6)
#define GF_IOC_ENABLE_POWER _IO(GF_IOC_MAGIC, 7)
#define GF_IOC_DISABLE_POWER _IO(GF_IOC_MAGIC, 8)
#define GF_IOC_INPUT_KEY_EVENT _IOW(GF_IOC_MAGIC, 9, struct gf_key)
#define GF_IOC_ENTER_SLEEP_MODE _IO(GF_IOC_MAGIC, 10)
#define GF_IOC_GET_FW_INFO _IOR(GF_IOC_MAGIC, 11, u8)
#define GF_IOC_REMOVE _IO(GF_IOC_MAGIC, 12)
#define GF_IOC_CHIP_INFO _IOW(GF_IOC_MAGIC, 13, struct gf_ioc_chip_info)
#define GF_IOC_NAV_EVENT _IOW(GF_IOC_MAGIC, 14, enum gf_nav_event)

#define GF_KEY_INPUT_HOME KEY_HOME
#define GF_KEY_INPUT_MENU KEY_MENU
#define GF_KEY_INPUT_BACK KEY_BACK
#define GF_KEY_INPUT_POWER KEY_POWER
#define GF_KEY_INPUT_CAMERA KEY_CAMERA
#define GF_NAV_INPUT_UP KEY_UP
#define GF_NAV_INPUT_DOWN KEY_DOWN
#define GF_NAV_INPUT_LEFT KEY_LEFT
#define GF_NAV_INPUT_RIGHT KEY_RIGHT
#define GF_NAV_INPUT_CLICK KEY_VOLUMEDOWN
#define GF_NAV_INPUT_DOUBLE_CLICK KEY_VOLUMEUP
#define GF_NAV_INPUT_LONG_PRESS KEY_SEARCH
#define GF_NAV_INPUT_HEAVY KEY_CHAT

struct gf_dev {
	struct device *dev;
	struct miscdevice misc;

	struct input_dev *input;
	struct gpio_desc *reset_gpiod;
	struct regulator *vdd;
	const char *fw_name;
	struct wakeup_source *fp_wakelock;
	int irq;

	struct mutex lock; /* open/release and IRQ-enable state */
	bool opened;
	bool irq_enabled;
	bool device_available;
	bool key_down;
};

static const struct gf_key_map {
	unsigned int type;
	unsigned int code;
} maps[] = {
	{ EV_KEY, GF_KEY_INPUT_HOME },
	{ EV_KEY, GF_KEY_INPUT_MENU },
	{ EV_KEY, GF_KEY_INPUT_BACK },
	{ EV_KEY, GF_KEY_INPUT_POWER },
	{ EV_KEY, GF_NAV_INPUT_UP },
	{ EV_KEY, GF_NAV_INPUT_DOWN },
	{ EV_KEY, GF_NAV_INPUT_RIGHT },
	{ EV_KEY, GF_NAV_INPUT_LEFT },
	{ EV_KEY, GF_KEY_INPUT_CAMERA },
	{ EV_KEY, GF_NAV_INPUT_CLICK },
	{ EV_KEY, GF_NAV_INPUT_DOUBLE_CLICK },
	{ EV_KEY, GF_NAV_INPUT_LONG_PRESS },
	{ EV_KEY, GF_NAV_INPUT_HEAVY },
};

/*
 * Raw netlink channel towards the HAL: the HAL sends one message to
 * register its pid, the driver unicasts single-byte events back.
 */
static struct sock *nl_sk;
static u32 nl_pid;

static void gf_sendnlmsg(u8 event)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	int ret;

	if (!nl_sk || !nl_pid)
		return;

	skb = nlmsg_new(MAX_NL_MSG_LEN, GFP_KERNEL);
	if (!skb)
		return;

	nlh = nlmsg_put(skb, 0, 0, 0, MAX_NL_MSG_LEN, 0);
	if (!nlh) {
		kfree_skb(skb);
		return;
	}

	((char *)nlmsg_data(nlh))[0] = event;
	((char *)nlmsg_data(nlh))[1] = 0;
	NETLINK_CB(skb).portid = 0;
	NETLINK_CB(skb).dst_group = 0;

	ret = netlink_unicast(nl_sk, skb, nl_pid, MSG_DONTWAIT);
	if (ret < 0)
		pr_err("failed to send netlink event to user space: %d\n", ret);
}

static void gf_nl_data_ready(struct sk_buff *skb)
{
	struct nlmsghdr *nlh;

	if (skb->len >= nlmsg_total_size(0)) {
		nlh = nlmsg_hdr(skb);
		nl_pid = nlh->nlmsg_pid;
		pr_debug("HAL netlink pid %u registered\n", nl_pid);
	}
}

static int gf_netlink_init(void)
{
	struct netlink_kernel_cfg cfg = {
		.input = gf_nl_data_ready,
	};

	nl_sk = netlink_kernel_create(&init_net, GF_NETLINK_PROTO, &cfg);
	if (!nl_sk) {
		pr_err("failed to create netlink socket\n");
		return -ENOMEM;
	}

	return 0;
}

static void gf_netlink_exit(void)
{
	if (nl_sk) {
		netlink_kernel_release(nl_sk);
		nl_sk = NULL;
	}
}

/* Callers hold gf_dev->lock */
static void gf_enable_irq(struct gf_dev *gf_dev)
{
	/* Boards that report fingers through another path have no IRQ. */
	if (gf_dev->irq <= 0)
		return;

	if (gf_dev->irq_enabled) {
		dev_warn(gf_dev->dev, "IRQ has been enabled\n");
	} else {
		enable_irq(gf_dev->irq);
		gf_dev->irq_enabled = true;
	}
}

static void gf_disable_irq(struct gf_dev *gf_dev)
{
	/* Boards that report fingers through another path have no IRQ. */
	if (gf_dev->irq <= 0)
		return;

	if (!gf_dev->irq_enabled) {
		dev_warn(gf_dev->dev, "IRQ has been disabled\n");
	} else {
		disable_irq(gf_dev->irq);
		gf_dev->irq_enabled = false;
	}
}

static void gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	gpiod_set_value_cansleep(gf_dev->reset_gpiod, 1);
	usleep_range(3000, 4000);
	gpiod_set_value_cansleep(gf_dev->reset_gpiod, 0);
	if (delay_ms)
		msleep(delay_ms);
}

static irqreturn_t gf_irq(int irq, void *handle)
{
	struct gf_dev *gf_dev = handle;

	__pm_wakeup_event(gf_dev->fp_wakelock, WAKELOCK_HOLD_TIME_MS);
	gf_sendnlmsg(GF_NET_EVENT_IRQ);

	return IRQ_HANDLED;
}

static void gf_nav_event_input(struct gf_dev *gf_dev, enum gf_nav_event event)
{
	u32 nav_input = 0;

	switch (event) {
	case GF_NAV_FINGER_DOWN:
	case GF_NAV_FINGER_UP:
		return;
	case GF_NAV_DOWN:
		nav_input = GF_NAV_INPUT_DOWN;
		break;
	case GF_NAV_UP:
		nav_input = GF_NAV_INPUT_UP;
		break;
	case GF_NAV_LEFT:
		nav_input = GF_NAV_INPUT_LEFT;
		break;
	case GF_NAV_RIGHT:
		nav_input = GF_NAV_INPUT_RIGHT;
		break;
	case GF_NAV_CLICK:
		nav_input = GF_NAV_INPUT_CLICK;
		break;
	case GF_NAV_HEAVY:
		nav_input = GF_NAV_INPUT_HEAVY;
		break;
	case GF_NAV_LONG_PRESS:
		nav_input = GF_NAV_INPUT_LONG_PRESS;
		break;
	case GF_NAV_DOUBLE_CLICK:
		nav_input = GF_NAV_INPUT_DOUBLE_CLICK;
		break;
	default:
		dev_warn(gf_dev->dev, "unknown nav event: %d\n", event);
		return;
	}

	input_report_key(gf_dev->input, nav_input, 1);
	input_sync(gf_dev->input);
	input_report_key(gf_dev->input, nav_input, 0);
	input_sync(gf_dev->input);
}

static void gf_kernel_key_input(struct gf_dev *gf_dev, struct gf_key *gf_key)
{
	u32 key_input;

	switch (gf_key->key) {
	case GF_KEY_HOME:
		key_input = GF_KEY_INPUT_HOME;
		break;
	case GF_KEY_POWER:
		key_input = GF_KEY_INPUT_POWER;
		break;
	case GF_KEY_CAMERA:
		key_input = GF_KEY_INPUT_CAMERA;
		break;
	default:
		key_input = gf_key->key;
		break;
	}

	dev_dbg(gf_dev->dev, "key event[%u], key = %d, value = %u\n",
		key_input, gf_key->key, gf_key->value);

	if ((gf_key->key == GF_KEY_POWER || gf_key->key == GF_KEY_CAMERA) &&
	    gf_key->value == 1) {
		input_report_key(gf_dev->input, key_input, 1);
		input_sync(gf_dev->input);
		input_report_key(gf_dev->input, key_input, 0);
		input_sync(gf_dev->input);
	}

	if (gf_key->key == GF_KEY_HOME) {
		if (gf_dev->key_down && gf_key->value == 1) {
			input_report_key(gf_dev->input, key_input, 0);
			input_sync(gf_dev->input);
		}
		input_report_key(gf_dev->input, key_input, gf_key->value);
		input_sync(gf_dev->input);
		gf_dev->key_down = gf_key->value == 1;
	}
}

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_dev *gf_dev = filp->private_data;
	struct gf_ioc_chip_info info;
	enum gf_nav_event nav_event;
	struct gf_key gf_key;
	u8 netlink_route = GF_NETLINK_PROTO;
	int retval = 0;

	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -ENODEV;

	if (!gf_dev->device_available &&
	    cmd != GF_IOC_ENABLE_POWER && cmd != GF_IOC_DISABLE_POWER) {
		dev_dbg(gf_dev->dev,
			"cmd %d, but sensor is powered off currently\n",
			_IOC_NR(cmd));
		return -ENODEV;
	}

	switch (cmd) {
	case GF_IOC_INIT:
		dev_dbg(gf_dev->dev, "GF_IOC_INIT\n");
		if (copy_to_user((void __user *)arg, &netlink_route,
				 sizeof(u8)))
			retval = -EFAULT;
		break;

	case GF_IOC_EXIT:
	case GF_IOC_ENTER_SLEEP_MODE:
	case GF_IOC_GET_FW_INFO:
	case GF_IOC_REMOVE:
		break;

	case GF_IOC_DISABLE_IRQ:
		dev_dbg(gf_dev->dev, "GF_IOC_DISABLE_IRQ\n");
		mutex_lock(&gf_dev->lock);
		gf_disable_irq(gf_dev);
		mutex_unlock(&gf_dev->lock);
		break;

	case GF_IOC_ENABLE_IRQ:
		dev_dbg(gf_dev->dev, "GF_IOC_ENABLE_IRQ\n");
		mutex_lock(&gf_dev->lock);
		gf_enable_irq(gf_dev);
		mutex_unlock(&gf_dev->lock);
		break;

	case GF_IOC_RESET:
		dev_dbg(gf_dev->dev, "GF_IOC_RESET\n");
		gf_hw_reset(gf_dev, 3);
		break;

	case GF_IOC_INPUT_KEY_EVENT:
		if (copy_from_user(&gf_key, (void __user *)arg,
				   sizeof(gf_key))) {
			retval = -EFAULT;
			break;
		}
		gf_kernel_key_input(gf_dev, &gf_key);
		break;

	case GF_IOC_NAV_EVENT:
		dev_dbg(gf_dev->dev, "GF_IOC_NAV_EVENT\n");
		if (copy_from_user(&nav_event, (void __user *)arg,
				   sizeof(nav_event))) {
			retval = -EFAULT;
			break;
		}
		gf_nav_event_input(gf_dev, nav_event);
		break;

	case GF_IOC_ENABLE_SPI_CLK:
	case GF_IOC_DISABLE_SPI_CLK:
		/* SPI clocks are owned by the secure world */
		break;

	case GF_IOC_ENABLE_POWER:
		dev_dbg(gf_dev->dev, "GF_IOC_ENABLE_POWER\n");
		/*
		 * The supply is enabled on open; downstream only toggled
		 * an optional power GPIO here that this platform lacks.
		 */
		if (!gf_dev->device_available)
			usleep_range(10000, 11000);
		gf_dev->device_available = true;
		break;

	case GF_IOC_DISABLE_POWER:
		dev_dbg(gf_dev->dev, "GF_IOC_DISABLE_POWER\n");
		gf_dev->device_available = false;
		break;

	case GF_IOC_CHIP_INFO:
		if (copy_from_user(&info, (void __user *)arg, sizeof(info))) {
			retval = -EFAULT;
			break;
		}
		dev_dbg(gf_dev->dev,
			"chip info: vendor 0x%x, mode 0x%x, operation 0x%x\n",
			info.vendor_id, info.mode, info.operation);
		break;

	default:
		dev_warn(gf_dev->dev, "unsupported cmd: 0x%x\n", cmd);
		break;
	}

	return retval;
}

static long gf_compat_ioctl(struct file *filp, unsigned int cmd,
			    unsigned long arg)
{
	return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}

static int gf_open(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev =
		container_of(filp->private_data, struct gf_dev, misc);
	int rc;

	mutex_lock(&gf_dev->lock);

	if (gf_dev->opened) {
		rc = -EBUSY;
		goto err_unlock;
	}

	rc = regulator_set_load(gf_dev->vdd, GF_VREG_LOAD_UA);
	if (rc < 0)
		dev_warn(gf_dev->dev, "failed to set vdd load: %d\n", rc);

	rc = regulator_enable(gf_dev->vdd);
	if (rc) {
		dev_err(gf_dev->dev, "failed to enable vdd: %d\n", rc);
		goto err_unlock;
	}

	/* Hold the sensor in reset until the HAL issues GF_IOC_RESET */
	gpiod_set_value_cansleep(gf_dev->reset_gpiod, 1);

	if (gf_dev->irq > 0) {
		rc = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
					  IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					  "gf", gf_dev);
		if (rc) {
			dev_err(gf_dev->dev, "failed to request IRQ %d: %d\n",
				gf_dev->irq, rc);
			goto err_vdd;
		}

		enable_irq_wake(gf_dev->irq);
		gf_dev->irq_enabled = true;
		gf_disable_irq(gf_dev);
	}

	gf_dev->opened = true;
	filp->private_data = gf_dev;
	mutex_unlock(&gf_dev->lock);

	nonseekable_open(inode, filp);
	dev_dbg(gf_dev->dev, "device opened, irq = %d\n", gf_dev->irq);

	return 0;

err_vdd:
	regulator_disable(gf_dev->vdd);
err_unlock:
	mutex_unlock(&gf_dev->lock);
	return rc;
}

static int gf_release(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev = filp->private_data;

	mutex_lock(&gf_dev->lock);

	gpiod_set_value_cansleep(gf_dev->reset_gpiod, 1);
	if (gf_dev->irq > 0) {
		if (gf_dev->irq_enabled)
			gf_disable_irq(gf_dev);
		disable_irq_wake(gf_dev->irq);
		free_irq(gf_dev->irq, gf_dev);
	}

	gf_dev->device_available = false;
	regulator_disable(gf_dev->vdd);
	gf_dev->opened = false;

	mutex_unlock(&gf_dev->lock);

	return 0;
}

static const struct file_operations gf_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = gf_ioctl,
	.compat_ioctl = gf_compat_ioctl,
	.open = gf_open,
	.release = gf_release,
};

static int gf_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gf_dev *gf_dev;
	size_t i;
	int rc;

	gf_dev = devm_kzalloc(dev, sizeof(*gf_dev), GFP_KERNEL);
	if (!gf_dev)
		return -ENOMEM;

	gf_dev->dev = dev;
	platform_set_drvdata(pdev, gf_dev);
	mutex_init(&gf_dev->lock);

	gf_dev->reset_gpiod = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(gf_dev->reset_gpiod))
		return dev_err_probe(dev, PTR_ERR(gf_dev->reset_gpiod),
				     "failed to get reset GPIO\n");

	rc = gpiod_direction_output(gf_dev->reset_gpiod, 1);
	if (rc)
		return dev_err_probe(dev, rc,
				     "failed to configure reset GPIO\n");

	/*
	 * The interrupt is optional.  On under-display parts the finger
	 * event may be delivered by the touchscreen instead, and the sensor
	 * line is not described to the operating system at all; a client
	 * then drives the trusted application by other means.  0 means
	 * "none", which every IRQ path below treats as a no-op.
	 */
	gf_dev->irq = platform_get_irq_optional(pdev, 0);
	if (gf_dev->irq < 0)
		gf_dev->irq = 0;

	/*
	 * Which trusted application drives this sensor. Optional: the sensor
	 * works without it, but a client then has to learn the name some other
	 * way.
	 */
	if (of_property_read_string(dev->of_node, "firmware-name",
				    &gf_dev->fw_name))
		gf_dev->fw_name = NULL;

	gf_dev->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(gf_dev->vdd))
		return dev_err_probe(dev, PTR_ERR(gf_dev->vdd),
				     "failed to get vdd regulator\n");

	gf_dev->input = devm_input_allocate_device(dev);
	if (!gf_dev->input)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(maps); i++)
		input_set_capability(gf_dev->input, maps[i].type,
				     maps[i].code);
	gf_dev->input->name = GF_INPUT_NAME;

	rc = input_register_device(gf_dev->input);
	if (rc)
		return dev_err_probe(dev, rc,
				     "failed to register input device\n");

	gf_dev->fp_wakelock = wakeup_source_register(dev, "fp_wakelock");
	if (!gf_dev->fp_wakelock)
		return -ENOMEM;

	device_init_wakeup(dev, true);

	gf_dev->misc.minor = MISC_DYNAMIC_MINOR;
	gf_dev->misc.name = GF_DEV_NAME;
	gf_dev->misc.fops = &gf_fops;
	gf_dev->misc.parent = dev;

	rc = misc_register(&gf_dev->misc);
	if (rc) {
		dev_err(dev, "failed to register misc device: %d\n", rc);
		wakeup_source_unregister(gf_dev->fp_wakelock);
		return rc;
	}

	return 0;
}

static void gf_remove(struct platform_device *pdev)
{
	struct gf_dev *gf_dev = platform_get_drvdata(pdev);

	misc_deregister(&gf_dev->misc);
	wakeup_source_unregister(gf_dev->fp_wakelock);
	mutex_destroy(&gf_dev->lock);
}

static ssize_t firmware_name_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct gf_dev *gf_dev = dev_get_drvdata(dev);

	if (!gf_dev->fw_name)
		return -ENODATA;

	return sysfs_emit(buf, "%s\n", gf_dev->fw_name);
}
static DEVICE_ATTR_RO(firmware_name);

static struct attribute *gf_attrs[] = {
	&dev_attr_firmware_name.attr,
	NULL,
};
ATTRIBUTE_GROUPS(gf);

static const struct of_device_id gf_of_match[] = {
	{ .compatible = "goodix,gf3626" },
	{}
};
MODULE_DEVICE_TABLE(of, gf_of_match);

static struct platform_driver gf_driver = {
	.driver = {
		.name = GF_DEV_NAME,
		.of_match_table = gf_of_match,
		.dev_groups = gf_groups,
	},
	.probe = gf_probe,
	.remove = gf_remove,
};

static int __init gf_init(void)
{
	int rc;

	rc = gf_netlink_init();
	if (rc)
		return rc;

	rc = platform_driver_register(&gf_driver);
	if (rc)
		gf_netlink_exit();

	return rc;
}
module_init(gf_init);

static void __exit gf_exit(void)
{
	platform_driver_unregister(&gf_driver);
	gf_netlink_exit();
}
module_exit(gf_exit);

MODULE_AUTHOR("Jiangtao Yi <yijiangtao@goodix.com>");
MODULE_AUTHOR("Jandy Gou <gouqingsong@goodix.com>");
MODULE_DESCRIPTION("Goodix SPI fingerprint sensor driver");
MODULE_LICENSE("GPL");
