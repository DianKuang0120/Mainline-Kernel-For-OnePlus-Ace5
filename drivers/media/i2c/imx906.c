// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX906 (marketed as LYT-700) - 50MP sensor, bring-up driver.
 *
 * The register tables were captured at runtime from stock Android through the
 * OPLUS camera kernel's CAM_CCI debug (see camera-android-capture/):
 *   - imx906_init_regs   : the full 422-write initialisation (incl. a ~287
 *                          write firmware/OTP blob through register 0x1000)
 *   - imx906_start_regs  : 11 writes that finish the mode setup and turn
 *                          streaming on (0x0100 = 1)
 *   - per-frame AE       : exposure 0x0e20, gain 0x0e58, frame length 0x0340
 *
 * Wiring on giulia (downstream qcom,cam-sensor0): CCI0 master 1, CSI PHY1,
 * MCLK1 (gpio101, 19.2 MHz), reset gpio82, rails
 *   avdd=pm8010m_l6 (1.8V), dvdd=pm8010m_l1 (1.1V), dovdd=pm8010m_l5 (1.8V),
 *   vaf=pm8010m_l7, v_custom1=pm8010m_l3, v_custom2=pm8010m_l7.
 * Chip id 0x0906 at I2C address 0x1a (16-bit regs, 8-bit data for writes).
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/videodev2.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "imx906_tables.h"

#define IMX906_REG_CHIP_ID	0x0016
#define IMX906_CHIP_ID_MSB	0x09
#define IMX906_CHIP_ID_LSB	0x06

#define IMX906_REG_STREAM	0x0100
#define IMX906_REG_EXPOSURE	0x0e20	/* 8-bit (observed) */
#define IMX906_REG_GAIN		0x0e58	/* 8-bit (observed) */
#define IMX906_REG_FLL		0x0340	/* frame length, low byte observed */

#define IMX906_MODE_WIDTH	4096
#define IMX906_MODE_HEIGHT	3072
#define IMX906_MODE_FLL_DEF	0x0d00
#define IMX906_MODE_FLL_MIN	0x0d00
#define IMX906_MODE_LLP		0x0d00
#define IMX906_LINK_FREQ_DEF	2490000000
#define IMX906_NUM_DATA_LANES	4

#define IMX906_EXPOSURE_DEF	0x0400
#define IMX906_EXPOSURE_MAX	0x0d00
#define IMX906_GAIN_DEF		0x0100
#define IMX906_GAIN_MAX		0x0fff

/* giulia rails, in the downstream cam-sensor0 order */
static const char * const imx906_supply_names[] = {
	"avdd", "dvdd", "dovdd", "vaf", "v_custom1", "v_custom2",
};

struct imx906_mode {
	u32 width;
	u32 height;
	u32 fll_def;
	u32 fll_min;
	u32 llp;
	u32 link_freq_idx;
};

struct imx906 {
	struct device *dev;
	struct i2c_client *client;
	struct clk *clk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(imx906_supply_names)];

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;

	const struct imx906_mode *cur_mode;
};

/*
 * The MIPI link frequency is not in the downstream DT (it lives in the
 * closed sensor .ko), so make it overridable at load time:
 *   modprobe imx906 link_freq=<Hz>
 * With the CSIPHY debug the vendor kernel prints it as "datarate:" in bps;
 * mainline's link frequency is datarate / 2 (DDR clock lane).
 */
static s64 imx906_link_freq_menu[1] = { IMX906_LINK_FREQ_DEF };
static unsigned long long imx906_link_freq = IMX906_LINK_FREQ_DEF;
module_param_named(link_freq, imx906_link_freq, ullong, 0644);
MODULE_PARM_DESC(link_freq, "MIPI link frequency in Hz (default 600 MHz)");

static const struct imx906_mode imx906_modes[] = {
	{
		.width = IMX906_MODE_WIDTH,
		.height = IMX906_MODE_HEIGHT,
		.fll_def = IMX906_MODE_FLL_DEF,
		.fll_min = IMX906_MODE_FLL_MIN,
		.llp = IMX906_MODE_LLP,
		.link_freq_idx = 0,
	},
};

static inline struct imx906 *to_imx906(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx906, sd);
}

static const struct of_device_id imx906_of_match[] = {
	{ .compatible = "sony,imx906" },
	{ }
};
MODULE_DEVICE_TABLE(of, imx906_of_match);

/* ---- i2c helpers: 16-bit register address, 8-bit data ---- */

static int imx906_write_reg(struct imx906 *s, u16 reg, u8 val)
{
	u8 buf[3] = { reg >> 8, reg & 0xff, val };
	struct i2c_msg msg = {
		.addr = s->client->addr, .flags = 0, .len = 3, .buf = buf,
	};

	return i2c_transfer(s->client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}

static int imx906_write_table(struct imx906 *s, const struct imx906_reg *t,
			      unsigned int n)
{
	unsigned int i;
	int ret;

	for (i = 0; i < n; i++) {
		ret = imx906_write_reg(s, t[i].addr, t[i].val);
		if (ret)
			return dev_err_probe(s->dev, ret,
					     "write 0x%04x failed\n", t[i].addr);
	}
	return 0;
}

static int imx906_read_reg16(struct imx906 *s, u16 reg, u16 *val)
{
	u8 addr[2] = { reg >> 8, reg & 0xff };
	u8 data[2];
	struct i2c_msg msgs[2] = {
		{ .addr = s->client->addr, .flags = 0, .len = 2, .buf = addr },
		{ .addr = s->client->addr, .flags = I2C_M_RD, .len = 2, .buf = data },
	};

	if (i2c_transfer(s->client->adapter, msgs, 2) != 2)
		return -EIO;
	*val = (data[0] << 8) | data[1];
	return 0;
}

static int imx906_read_reg8(struct imx906 *s, u16 reg, u8 *val)
{
	u8 addr[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[2] = {
		{ .addr = s->client->addr, .flags = 0, .len = 2, .buf = addr },
		{ .addr = s->client->addr, .flags = I2C_M_RD, .len = 1, .buf = val },
	};

	return i2c_transfer(s->client->adapter, msgs, 2) == 2 ? 0 : -EIO;
}

/* ---- power ---- */

static int imx906_power_on(struct imx906 *s)
{
	int ret;

	ret = clk_prepare_enable(s->clk);
	if (ret)
		return dev_err_probe(s->dev, ret, "failed to enable MCLK\n");

	ret = regulator_bulk_enable(ARRAY_SIZE(imx906_supply_names),
				    s->supplies);
	if (ret) {
		dev_err_probe(s->dev, ret, "failed to enable regulators\n");
		clk_disable_unprepare(s->clk);
		return ret;
	}

	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(s->reset_gpio, 0); /* GPIO_ACTIVE_LOW: release */
	usleep_range(20000, 21000);

	return 0;
}

static void imx906_power_off(struct imx906 *s)
{
	gpiod_set_value_cansleep(s->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(imx906_supply_names), s->supplies);
	clk_disable_unprepare(s->clk);
}

static int imx906_identify(struct imx906 *s)
{
	u16 id = 0;

	if (imx906_read_reg16(s, IMX906_REG_CHIP_ID, &id))
		return -ENODEV;
	if (id != ((IMX906_CHIP_ID_MSB << 8) | IMX906_CHIP_ID_LSB))
		return dev_err_probe(s->dev, -EIO,
				     "chip id mismatch: 0x%04x\n", id);
	return 0;
}

/* ---- V4L2 subdev ---- */

static int imx906_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx906 *s = to_imx906(sd);
	int ret;

	if (enable) {
		ret = pm_runtime_resume_and_get(s->dev);
		if (ret)
			return ret;

		ret = imx906_write_table(s, imx906_init_regs,
					 ARRAY_SIZE(imx906_init_regs));
		if (ret)
			goto err;

		/* The PHY mode / lane count the part actually ended up in. */
		{
			u8 sig = 0, lanes = 0;
			if (!imx906_read_reg8(s, 0x0111, &sig) &&
			    !imx906_read_reg8(s, 0x0114, &lanes))
				dev_info(s->dev,
					 "init done: CSI_SIGNALING_MODE(0x0111)=0x%02x (%s), lanes-1(0x0114)=0x%02x\n",
					 sig, sig == 0x02 ? "D-PHY" : "not D-PHY",
					 lanes);
		}

		ret = imx906_write_table(s, imx906_start_regs,
					 ARRAY_SIZE(imx906_start_regs));
		if (ret)
			goto err;
	} else {
		ret = imx906_write_reg(s, IMX906_REG_STREAM, 0x00);
		pm_runtime_put(s->dev);
		if (ret)
			return ret;
	}

	return 0;

err:
	pm_runtime_put(s->dev);
	return ret;
}

static int imx906_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	return 0;
}

static int imx906_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(imx906_modes))
		return -EINVAL;
	fse->min_width = imx906_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = imx906_modes[fse->index].height;
	fse->max_height = fse->min_height;
	return 0;
}

static void imx906_fill_fmt(const struct imx906_mode *mode,
			    struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
}

static int imx906_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct imx906 *s = to_imx906(sd);

	fmt->format = s->fmt;
	return 0;
}

static int imx906_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct imx906 *s = to_imx906(sd);

	s->cur_mode = &imx906_modes[0];
	imx906_fill_fmt(s->cur_mode, &fmt->format);
	s->fmt = fmt->format;
	return 0;
}

static const struct v4l2_subdev_video_ops imx906_video_ops = {
	.s_stream = imx906_set_stream,
};

static const struct v4l2_subdev_pad_ops imx906_pad_ops = {
	.enum_mbus_code = imx906_enum_mbus_code,
	.enum_frame_size = imx906_enum_frame_size,
	.get_fmt = imx906_get_fmt,
	.set_fmt = imx906_set_fmt,
};

static const struct v4l2_subdev_ops imx906_subdev_ops = {
	.video = &imx906_video_ops,
	.pad = &imx906_pad_ops,
};

static int imx906_init_controls(struct imx906 *s)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &s->ctrl_handler;
	s64 pixel_rate;
	u8 max;
	int ret;

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 2);
	if (ret)
		return ret;

	imx906_link_freq_menu[0] = imx906_link_freq;
	max = ARRAY_SIZE(imx906_link_freq_menu) - 1;
	s->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, NULL,
					      V4L2_CID_LINK_FREQ, max, 0,
					      imx906_link_freq_menu);
	if (s->link_freq)
		s->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* pixel_rate = link_freq * 2 * nr_of_lanes / bits_per_sample */
	pixel_rate = imx906_link_freq * 2 * IMX906_NUM_DATA_LANES;
	do_div(pixel_rate, 10);
	s->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, NULL, V4L2_CID_PIXEL_RATE,
					  pixel_rate, pixel_rate, 1, pixel_rate);

	if (ctrl_hdlr->error) {
		v4l2_ctrl_handler_free(ctrl_hdlr);
		return ctrl_hdlr->error;
	}
	s->sd.ctrl_handler = ctrl_hdlr;
	return 0;
}

/* ---- probe ---- */

static int imx906_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx906 *s;
	unsigned int i;
	int ret;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->dev = dev;
	s->client = client;

	s->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(s->clk))
		return dev_err_probe(dev, PTR_ERR(s->clk), "no clock\n");

	s->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);

	for (i = 0; i < ARRAY_SIZE(imx906_supply_names); i++)
		s->supplies[i].supply = imx906_supply_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(imx906_supply_names),
				      s->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	ret = imx906_power_on(s);
	if (ret)
		return ret;
	ret = imx906_identify(s);
	if (ret) {
		imx906_power_off(s);
		return ret;
	}

	v4l2_i2c_subdev_init(&s->sd, client, &imx906_subdev_ops);
	s->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	s->cur_mode = &imx906_modes[0];
	imx906_fill_fmt(s->cur_mode, &s->fmt);

	ret = imx906_init_controls(s);
	if (ret) {
		v4l2_device_unregister_subdev(&s->sd);
		return ret;
	}

	s->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&s->sd.entity, 1, &s->pad);
	if (ret)
		goto err_ctrl;

	ret = v4l2_async_register_subdev_sensor(&s->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	dev_info(dev, "IMX906 (LYT-700) probed, chip id ok\n");
	return 0;

err_entity:
	media_entity_cleanup(&s->sd.entity);
err_ctrl:
	v4l2_ctrl_handler_free(&s->ctrl_handler);
	v4l2_device_unregister_subdev(&s->sd);
	return ret;
}

static void imx906_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx906 *s = to_imx906(sd);

	pm_runtime_disable(s->dev);
	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&s->ctrl_handler);
}

static int imx906_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx906 *s = to_imx906(sd);

	imx906_power_off(s);
	return 0;
}

static int imx906_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx906 *s = to_imx906(sd);

	return imx906_power_on(s);
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx906_pm_ops, imx906_runtime_suspend,
				 imx906_runtime_resume, NULL);

static struct i2c_driver imx906_driver = {
	.driver = {
		.name = "imx906",
		.of_match_table = imx906_of_match,
		.pm = pm_ptr(&imx906_pm_ops),
	},
	.probe = imx906_probe,
	.remove = imx906_remove,
};
module_i2c_driver(imx906_driver);

MODULE_DESCRIPTION("Sony IMX906 (LYT-700) sensor driver");
MODULE_LICENSE("GPL");
