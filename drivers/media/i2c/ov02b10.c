// SPDX-License-Identifier: GPL-2.0
/*
 * OmniVision OV02B10 sensor driver
 *
 * The register sequences and the hardware configuration are derived from
 * the Rockchip BSP driver (drivers/media/i2c/ov02b10.c):
 *
 * Copyright (C) 2020 Rockchip Electronics Co., Ltd.
 *
 * Rewritten to use the mainline V4L2 CCI helpers.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define OV02B10_LINK_FREQ_360MHZ	360000000ULL
#define OV02B10_XVCLK_FREQ		24000000
#define OV02B10_DATA_LANES		1
#define OV02B10_RGB_DEPTH		10

#define OV02B10_REG_CHIP_ID		CCI_REG16(0x02)
#define OV02B10_CHIP_ID			0x002b

#define OV02B10_REG_PAGE		CCI_REG8(0xfd)
#define OV02B10_PAGE_0			0x00
#define OV02B10_PAGE_1			0x01
#define OV02B10_PAGE_3			0x03

/* Software reset */
#define OV02B10_REG_SW_RESET		CCI_REG8(0xfc)
#define OV02B10_SW_RESET		BIT(0)

/* Stream control */
#define OV02B10_REG_MODE_SELECT		CCI_REG8(0xfb)
#define OV02B10_MODE_STANDBY		0x00
#define OV02B10_MODE_STREAMING		0x01

/* Page 1 registers */
#define OV02B10_REG_EXPOSURE		CCI_REG16(0x0e)
#define OV02B10_REG_ORIENTATION		CCI_REG8(0x12)
#define OV02B10_REG_VBLANK		CCI_REG16(0x14)
#define OV02B10_REG_ANALOG_GAIN		CCI_REG8(0x22)

/* Page 3 registers */
#define OV02B10_REG_DIGITAL_GAIN	CCI_REG8(0x9b)

/* Register update trigger */
#define OV02B10_REG_RESTART		CCI_REG8(0xfe)
#define OV02B10_RESTART_VALUE		0x02

#define OV02B10_VTS_MAX			0xffff

/* From spec: vstart is 0xc by default */
#define OV02B10_VSTART			0x0c

#define OV02B10_EXPOSURE_MIN		4
#define OV02B10_EXPOSURE_STEP		1
#define OV02B10_EXPOSURE_DEFAULT	0x02ea
#define OV02B10_EXPOSURE_MAX_MARGIN	7

/* Analog gain register value, 0x10 = 1x */
#define OV02B10_ANALOG_GAIN_MIN		0x10
#define OV02B10_ANALOG_GAIN_MAX		0xf8
#define OV02B10_ANALOG_GAIN_STEP	1
#define OV02B10_ANALOG_GAIN_DEFAULT	0x10

/* Digital gain in 1/64 steps, 0x40 = 1x */
#define OV02B10_DIGITAL_GAIN_MIN	0x40
#define OV02B10_DIGITAL_GAIN_MAX	0xff
#define OV02B10_DIGITAL_GAIN_STEP	1
#define OV02B10_DIGITAL_GAIN_DEFAULT	0x40

static const char * const ov02b10_supply_names[] = {
	"dovdd",	/* Digital I/O power */
	"avdd",		/* Analog power */
};

static const s64 link_freq_menu_items[] = {
	OV02B10_LINK_FREQ_360MHZ,
};

/*
 * Settings for 1600x1200@30fps, 10-bit, 1 MIPI data lane, xvclk 24 MHz.
 *
 * The sequence starts with a software reset and leaves the sensor in
 * software standby; streaming is enabled separately.
 */
static const struct cci_reg_sequence ov02b10_linear10bit_1600x1200_regs[] = {
	{ CCI_REG8(0xfc), 0x01 },
	{ CCI_REG8(0xfd), 0x00 },
	{ CCI_REG8(0xfd), 0x00 },
	{ CCI_REG8(0x24), 0x02 },
	{ CCI_REG8(0x25), 0x06 },
	{ CCI_REG8(0x29), 0x03 },
	{ CCI_REG8(0x2a), 0x34 },
	{ CCI_REG8(0x1e), 0x17 },
	{ CCI_REG8(0x33), 0x07 },
	{ CCI_REG8(0x35), 0x07 },
	{ CCI_REG8(0x4a), 0x0c },
	{ CCI_REG8(0x3a), 0x05 },
	{ CCI_REG8(0x3b), 0x02 },
	{ CCI_REG8(0x3e), 0x00 },
	{ CCI_REG8(0x46), 0x01 },
	{ CCI_REG8(0x6d), 0x03 },
	{ CCI_REG8(0xfd), 0x01 },
	{ CCI_REG8(0x0e), 0x02 },
	{ CCI_REG8(0x0f), 0x1a },
	{ CCI_REG8(0x18), 0x00 },
	{ CCI_REG8(0x22), 0xff },
	{ CCI_REG8(0x23), 0x02 },
	{ CCI_REG8(0x17), 0x2c },
	{ CCI_REG8(0x19), 0x20 },
	{ CCI_REG8(0x1b), 0x06 },
	{ CCI_REG8(0x1c), 0x04 },
	{ CCI_REG8(0x20), 0x03 },
	{ CCI_REG8(0x30), 0x01 },
	{ CCI_REG8(0x33), 0x01 },
	{ CCI_REG8(0x31), 0x0a },
	{ CCI_REG8(0x32), 0x09 },
	{ CCI_REG8(0x38), 0x01 },
	{ CCI_REG8(0x39), 0x01 },
	{ CCI_REG8(0x3a), 0x01 },
	{ CCI_REG8(0x3b), 0x01 },
	{ CCI_REG8(0x4f), 0x04 },
	{ CCI_REG8(0x4e), 0x05 },
	{ CCI_REG8(0x50), 0x01 },
	{ CCI_REG8(0x35), 0x0c },
	{ CCI_REG8(0x45), 0x2a },
	{ CCI_REG8(0x46), 0x2a },
	{ CCI_REG8(0x47), 0x2a },
	{ CCI_REG8(0x48), 0x2a },
	{ CCI_REG8(0x4a), 0x2c },
	{ CCI_REG8(0x4b), 0x2c },
	{ CCI_REG8(0x4c), 0x2c },
	{ CCI_REG8(0x4d), 0x2c },
	{ CCI_REG8(0x56), 0x3a },
	{ CCI_REG8(0x57), 0x0a },
	{ CCI_REG8(0x58), 0x24 },
	{ CCI_REG8(0x59), 0x20 },
	{ CCI_REG8(0x5a), 0x0a },
	{ CCI_REG8(0x5b), 0xff },
	{ CCI_REG8(0x37), 0x0a },
	{ CCI_REG8(0x42), 0x0e },
	{ CCI_REG8(0x68), 0x90 },
	{ CCI_REG8(0x69), 0xcd },
	{ CCI_REG8(0x6a), 0x8f },
	{ CCI_REG8(0x7c), 0x0a },
	{ CCI_REG8(0x7d), 0x0a },
	{ CCI_REG8(0x7e), 0x0a },
	{ CCI_REG8(0x7f), 0x08 },
	{ CCI_REG8(0x83), 0x14 },
	{ CCI_REG8(0x84), 0x14 },
	{ CCI_REG8(0x86), 0x14 },
	{ CCI_REG8(0x87), 0x07 },
	{ CCI_REG8(0x88), 0x0f },
	{ CCI_REG8(0x94), 0x02 },
	{ CCI_REG8(0x98), 0xd1 },
	{ CCI_REG8(0xfe), 0x02 },
	{ CCI_REG8(0xfd), 0x03 },
	{ CCI_REG8(0x97), 0x6c },
	{ CCI_REG8(0x98), 0x60 },
	{ CCI_REG8(0x99), 0x60 },
	{ CCI_REG8(0x9a), 0x6c },
	{ CCI_REG8(0xa1), 0x40 },
	{ CCI_REG8(0xaf), 0x04 },
	{ CCI_REG8(0xb1), 0x40 },
	{ CCI_REG8(0xae), 0x0d },
	{ CCI_REG8(0x88), 0x5b },
	{ CCI_REG8(0x89), 0x7c },
	{ CCI_REG8(0xb4), 0x05 },
	{ CCI_REG8(0x8c), 0x40 },
	{ CCI_REG8(0x8e), 0x40 },
	{ CCI_REG8(0x90), 0x40 },
	{ CCI_REG8(0x92), 0x40 },
	{ CCI_REG8(0x9b), 0x46 },
	{ CCI_REG8(0xac), 0x40 },
	{ CCI_REG8(0xfd), 0x00 },
	{ CCI_REG8(0x5a), 0x15 },
	{ CCI_REG8(0x74), 0x01 },
	{ CCI_REG8(0xfd), 0x00 },
	{ CCI_REG8(0x50), 0x40 },
	{ CCI_REG8(0x52), 0xb0 },
	{ CCI_REG8(0xfd), 0x01 },
	{ CCI_REG8(0x03), 0x70 },
	{ CCI_REG8(0x05), 0x10 },
	{ CCI_REG8(0x07), 0x20 },
	{ CCI_REG8(0x09), 0xb0 },
	{ CCI_REG8(0xfd), 0x03 },
	{ CCI_REG8(0xc2), 0x01 },
};

struct ov02b10_mode {
	/* Frame width in pixels */
	u32 width;

	/* Frame height in pixels */
	u32 height;

	/* Horizontal timing size */
	u32 hts;

	/* Default vertical timing */
	u32 vts_def;

	/* Min vertical timing */
	u32 vts_min;

	/* Sensor register settings for this resolution */
	const struct cci_reg_sequence *reg_list;
	u32 num_regs;
};

static const struct ov02b10_mode supported_modes[] = {
	{
		.width = 1600,
		.height = 1200,
		.hts = 0x06ac,
		.vts_def = 0x04c4,
		.vts_min = 0x04c4,
		.reg_list = ov02b10_linear10bit_1600x1200_regs,
		.num_regs = ARRAY_SIZE(ov02b10_linear10bit_1600x1200_regs),
	},
};

struct ov02b10 {
	struct device *dev;
	struct regmap *regmap;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;

	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;

	struct clk *xvclk;
	struct gpio_desc *reset;
	struct gpio_desc *pwdn;
	struct regulator_bulk_data supplies[ARRAY_SIZE(ov02b10_supply_names)];

	/* Current mode */
	const struct ov02b10_mode *cur_mode;

	/* MIPI lanes info */
	u32 link_freq_index;
	u8 mipi_lanes;
};

static inline struct ov02b10 *to_ov02b10(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov02b10, sd);
}

static u64 to_pixel_rate(u32 f_index)
{
	u64 pixel_rate = link_freq_menu_items[f_index] * 2 * OV02B10_DATA_LANES;

	do_div(pixel_rate, OV02B10_RGB_DEPTH);

	return pixel_rate;
}

static int ov02b10_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov02b10 *ov02b10 = container_of(ctrl->handler,
					       struct ov02b10, ctrl_handler);
	s64 exposure_max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	if (ctrl->id == V4L2_CID_VBLANK) {
		/* Update max exposure while meeting expected vblanking */
		exposure_max = ov02b10->cur_mode->height + ctrl->val -
			       OV02B10_EXPOSURE_MAX_MARGIN;
		ret = __v4l2_ctrl_modify_range(ov02b10->exposure,
					       ov02b10->exposure->minimum,
					       exposure_max,
					       ov02b10->exposure->step,
					       ov02b10->exposure->default_value);
		if (ret)
			return ret;
	}

	/* V4L2 controls values will be applied only when power is already up */
	if (!pm_runtime_get_if_in_use(ov02b10->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(ov02b10->regmap, OV02B10_REG_PAGE,
			  OV02B10_PAGE_1, &ret);
		cci_write(ov02b10->regmap, OV02B10_REG_EXPOSURE,
			  ctrl->val, &ret);
		cci_write(ov02b10->regmap, OV02B10_REG_RESTART,
			  OV02B10_RESTART_VALUE, &ret);
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(ov02b10->regmap, OV02B10_REG_PAGE,
			  OV02B10_PAGE_1, &ret);
		cci_write(ov02b10->regmap, OV02B10_REG_ANALOG_GAIN,
			  ctrl->val, &ret);
		cci_write(ov02b10->regmap, OV02B10_REG_RESTART,
			  OV02B10_RESTART_VALUE, &ret);
		break;

	case V4L2_CID_DIGITAL_GAIN:
		cci_write(ov02b10->regmap, OV02B10_REG_PAGE,
			  OV02B10_PAGE_3, &ret);
		cci_write(ov02b10->regmap, OV02B10_REG_DIGITAL_GAIN,
			  ctrl->val, &ret);
		cci_write(ov02b10->regmap, OV02B10_REG_RESTART,
			  OV02B10_RESTART_VALUE, &ret);
		break;

	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		cci_write(ov02b10->regmap, OV02B10_REG_PAGE,
			  OV02B10_PAGE_1, &ret);
		cci_write(ov02b10->regmap, OV02B10_REG_ORIENTATION,
			  ov02b10->hflip->val | ov02b10->vflip->val << 1,
			  &ret);
		cci_write(ov02b10->regmap, OV02B10_REG_RESTART,
			  OV02B10_RESTART_VALUE, &ret);
		break;

	case V4L2_CID_VBLANK:
		cci_write(ov02b10->regmap, OV02B10_REG_PAGE,
			  OV02B10_PAGE_1, &ret);
		/* The sensor VBLANK register is relative to VSTART */
		cci_write(ov02b10->regmap, OV02B10_REG_VBLANK,
			  ctrl->val - OV02B10_VSTART, &ret);
		cci_write(ov02b10->regmap, OV02B10_REG_RESTART,
			  OV02B10_RESTART_VALUE, &ret);
		break;

	default:
		break;
	}

	pm_runtime_put(ov02b10->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ov02b10_ctrl_ops = {
	.s_ctrl = ov02b10_set_ctrl,
};

static int ov02b10_init_controls(struct ov02b10 *ov02b10)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &ov02b10->ctrl_handler;
	const struct ov02b10_mode *mode = ov02b10->cur_mode;
	struct v4l2_fwnode_device_properties props;
	u32 vblank_min, vblank_max, vblank_def;
	s64 exposure_max, h_blank, pixel_rate;
	int ret;

	v4l2_ctrl_handler_init(ctrl_hdlr, 12);

	ov02b10->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, NULL, V4L2_CID_LINK_FREQ,
				       ov02b10->link_freq_index, 0,
				       link_freq_menu_items);
	if (ov02b10->link_freq)
		ov02b10->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	pixel_rate = to_pixel_rate(ov02b10->link_freq_index);
	ov02b10->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, NULL,
						V4L2_CID_PIXEL_RATE, 0,
						pixel_rate, 1, pixel_rate);

	vblank_min = mode->vts_min - mode->height;
	vblank_max = OV02B10_VTS_MAX - mode->height;
	vblank_def = mode->vts_def - mode->height;
	ov02b10->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &ov02b10_ctrl_ops,
					    V4L2_CID_VBLANK, vblank_min,
					    vblank_max, 1, vblank_def);

	h_blank = mode->hts - mode->width;
	ov02b10->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &ov02b10_ctrl_ops,
					    V4L2_CID_HBLANK, h_blank, h_blank,
					    1, h_blank);
	if (ov02b10->hblank)
		ov02b10->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(ctrl_hdlr, &ov02b10_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  OV02B10_ANALOG_GAIN_MIN, OV02B10_ANALOG_GAIN_MAX,
			  OV02B10_ANALOG_GAIN_STEP,
			  OV02B10_ANALOG_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &ov02b10_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  OV02B10_DIGITAL_GAIN_MIN, OV02B10_DIGITAL_GAIN_MAX,
			  OV02B10_DIGITAL_GAIN_STEP,
			  OV02B10_DIGITAL_GAIN_DEFAULT);

	exposure_max = mode->vts_def - OV02B10_EXPOSURE_MAX_MARGIN;
	ov02b10->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &ov02b10_ctrl_ops,
					      V4L2_CID_EXPOSURE,
					      OV02B10_EXPOSURE_MIN,
					      exposure_max,
					      OV02B10_EXPOSURE_STEP,
					      OV02B10_EXPOSURE_DEFAULT);

	ov02b10->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &ov02b10_ctrl_ops,
					   V4L2_CID_HFLIP, 0, 1, 1, 0);
	ov02b10->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &ov02b10_ctrl_ops,
					   V4L2_CID_VFLIP, 0, 1, 1, 0);

	ret = v4l2_fwnode_device_parse(ov02b10->dev, &props);
	if (ret)
		goto error;

	v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &ov02b10_ctrl_ops, &props);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		goto error;
	}

	ov02b10->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static void ov02b10_update_pad_format(const struct ov02b10_mode *mode,
				      struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	fmt->field = V4L2_FIELD_NONE;
}

static int ov02b10_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  u32 pad, u64 streams_mask)
{
	struct ov02b10 *ov02b10 = to_ov02b10(sd);
	const struct ov02b10_mode *mode = ov02b10->cur_mode;
	int ret;

	ret = pm_runtime_resume_and_get(ov02b10->dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(ov02b10->regmap, mode->reg_list,
				  mode->num_regs, NULL);
	if (ret) {
		dev_err(ov02b10->dev, "failed to set mode\n");
		goto out;
	}

	ret = __v4l2_ctrl_handler_setup(ov02b10->sd.ctrl_handler);
	if (ret)
		goto out;

	ret = cci_write(ov02b10->regmap, OV02B10_REG_MODE_SELECT,
			OV02B10_MODE_STREAMING, NULL);

out:
	if (ret)
		pm_runtime_put(ov02b10->dev);

	return ret;
}

static int ov02b10_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   u32 pad, u64 streams_mask)
{
	struct ov02b10 *ov02b10 = to_ov02b10(sd);

	cci_write(ov02b10->regmap, OV02B10_REG_MODE_SELECT,
		  OV02B10_MODE_STANDBY, NULL);
	pm_runtime_put(ov02b10->dev);

	return 0;
}

static int ov02b10_get_pm_resources(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov02b10 *ov02b10 = to_ov02b10(sd);
	int i;

	ov02b10->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ov02b10->reset))
		return dev_err_probe(dev, PTR_ERR(ov02b10->reset),
				     "failed to get reset gpio\n");

	ov02b10->pwdn = devm_gpiod_get_optional(dev, "pwdn", GPIOD_OUT_HIGH);
	if (IS_ERR(ov02b10->pwdn))
		return dev_err_probe(dev, PTR_ERR(ov02b10->pwdn),
				     "failed to get pwdn gpio\n");

	for (i = 0; i < ARRAY_SIZE(ov02b10_supply_names); i++)
		ov02b10->supplies[i].supply = ov02b10_supply_names[i];

	return devm_regulator_bulk_get(dev, ARRAY_SIZE(ov02b10_supply_names),
				       ov02b10->supplies);
}

static int ov02b10_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov02b10 *ov02b10 = to_ov02b10(sd);

	if (ov02b10->reset)
		gpiod_set_value_cansleep(ov02b10->reset, 1);

	clk_disable_unprepare(ov02b10->xvclk);

	if (ov02b10->pwdn)
		gpiod_set_value_cansleep(ov02b10->pwdn, 1);

	regulator_bulk_disable(ARRAY_SIZE(ov02b10_supply_names),
			       ov02b10->supplies);

	return 0;
}

static int ov02b10_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov02b10 *ov02b10 = to_ov02b10(sd);
	int ret;

	if (ov02b10->reset)
		gpiod_set_value_cansleep(ov02b10->reset, 1);
	if (ov02b10->pwdn)
		gpiod_set_value_cansleep(ov02b10->pwdn, 1);

	ret = regulator_bulk_enable(ARRAY_SIZE(ov02b10_supply_names),
				    ov02b10->supplies);
	if (ret < 0) {
		dev_err(dev, "failed to enable regulators\n");
		return ret;
	}

	usleep_range(100, 110);

	ret = clk_prepare_enable(ov02b10->xvclk);
	if (ret < 0) {
		dev_err(dev, "failed to enable imaging clock: %d\n", ret);
		goto disable_regulators;
	}

	usleep_range(5000, 6000);

	if (ov02b10->pwdn)
		gpiod_set_value_cansleep(ov02b10->pwdn, 0);

	usleep_range(4000, 5000);

	if (ov02b10->reset)
		gpiod_set_value_cansleep(ov02b10->reset, 0);

	usleep_range(5000, 6000);

	return 0;

disable_regulators:
	regulator_bulk_disable(ARRAY_SIZE(ov02b10_supply_names),
			       ov02b10->supplies);

	return ret;
}

static int ov02b10_set_format(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *fmt)
{
	struct ov02b10 *ov02b10 = to_ov02b10(sd);
	const struct ov02b10_mode *mode;
	s32 vblank_def, h_blank;
	int ret = 0;

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	ov02b10_update_pad_format(mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_state_get_format(sd_state, fmt->pad) = fmt->format;
	} else {
		ov02b10->cur_mode = mode;
		ret = __v4l2_ctrl_s_ctrl(ov02b10->link_freq,
					 ov02b10->link_freq_index);
		if (ret)
			return ret;

		ret = __v4l2_ctrl_s_ctrl_int64(ov02b10->pixel_rate,
					       to_pixel_rate(ov02b10->link_freq_index));
		if (ret)
			return ret;

		/* Update limits and set FPS to default */
		vblank_def = mode->vts_def - mode->height;
		ret = __v4l2_ctrl_modify_range(ov02b10->vblank,
					       mode->vts_min - mode->height,
					       OV02B10_VTS_MAX - mode->height,
					       1, vblank_def);
		if (ret)
			return ret;

		ret = __v4l2_ctrl_s_ctrl(ov02b10->vblank, vblank_def);
		if (ret)
			return ret;

		h_blank = mode->hts - mode->width;
		ret = __v4l2_ctrl_modify_range(ov02b10->hblank, h_blank,
					       h_blank, 1, h_blank);
	}

	return ret;
}

static int ov02b10_get_format(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *fmt)
{
	struct ov02b10 *ov02b10 = to_ov02b10(sd);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_state_get_format(sd_state, fmt->pad);
	else
		ov02b10_update_pad_format(ov02b10->cur_mode, &fmt->format);

	return 0;
}

static int ov02b10_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;

	return 0;
}

static int ov02b10_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int ov02b10_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state)
{
	ov02b10_update_pad_format(&supported_modes[0],
				  v4l2_subdev_state_get_format(sd_state, 0));

	return 0;
}

static const struct v4l2_subdev_video_ops ov02b10_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops ov02b10_pad_ops = {
	.set_fmt = ov02b10_set_format,
	.get_fmt = ov02b10_get_format,
	.enum_mbus_code = ov02b10_enum_mbus_code,
	.enum_frame_size = ov02b10_enum_frame_size,
	.enable_streams = ov02b10_enable_streams,
	.disable_streams = ov02b10_disable_streams,
};

static const struct v4l2_subdev_ops ov02b10_subdev_ops = {
	.video = &ov02b10_video_ops,
	.pad = &ov02b10_pad_ops,
};

static const struct media_entity_operations ov02b10_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops ov02b10_internal_ops = {
	.init_state = ov02b10_init_state,
};

static int ov02b10_identify_module(struct ov02b10 *ov02b10)
{
	int ret = 0;
	u64 val;

	cci_write(ov02b10->regmap, OV02B10_REG_PAGE, OV02B10_PAGE_0, &ret);
	cci_read(ov02b10->regmap, OV02B10_REG_CHIP_ID, &val, &ret);
	if (ret)
		return ret;

	if (val != OV02B10_CHIP_ID) {
		dev_err(ov02b10->dev, "chip id mismatch: %x!=%llx\n",
			OV02B10_CHIP_ID, val);
		return -ENXIO;
	}

	return 0;
}

static int ov02b10_check_hwcfg(struct ov02b10 *ov02b10)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct device *dev = ov02b10->dev;
	struct fwnode_handle *ep;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	unsigned long link_freq_bitmap;
	int ret;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "waiting for fwnode graph endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(dev, ret, "parsing endpoint failed\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != OV02B10_DATA_LANES) {
		dev_err(dev, "number of CSI2 data lanes %d is not supported\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto out_err;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		dev_err(dev, "no link frequencies defined\n");
		ret = -EINVAL;
		goto out_err;
	}

	ret = v4l2_link_freq_to_bitmap(dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       link_freq_menu_items,
				       ARRAY_SIZE(link_freq_menu_items),
				       &link_freq_bitmap);
	if (ret)
		goto out_err;

	/* v4l2_link_freq_to_bitmap() guarantees at least 1 bit is set */
	ov02b10->link_freq_index = ffs(link_freq_bitmap) - 1;
	ov02b10->mipi_lanes = bus_cfg.bus.mipi_csi2.num_data_lanes;

out_err:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static void ov02b10_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov02b10 *ov02b10 = to_ov02b10(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(ov02b10->dev);

	if (!pm_runtime_status_suspended(ov02b10->dev)) {
		ov02b10_power_off(ov02b10->dev);
		pm_runtime_set_suspended(ov02b10->dev);
	}
}

static int ov02b10_probe(struct i2c_client *client)
{
	struct ov02b10 *ov02b10;
	unsigned long freq;
	int ret;

	ov02b10 = devm_kzalloc(&client->dev, sizeof(*ov02b10), GFP_KERNEL);
	if (!ov02b10)
		return -ENOMEM;

	ov02b10->dev = &client->dev;

	ov02b10->xvclk = devm_v4l2_sensor_clk_get(ov02b10->dev, NULL);
	if (IS_ERR(ov02b10->xvclk))
		return dev_err_probe(ov02b10->dev, PTR_ERR(ov02b10->xvclk),
				     "failed to get imaging clock\n");

	freq = clk_get_rate(ov02b10->xvclk);
	if (freq != OV02B10_XVCLK_FREQ)
		return dev_err_probe(ov02b10->dev, -EINVAL,
				     "external clock %lu is not supported",
				     freq);

	v4l2_i2c_subdev_init(&ov02b10->sd, client, &ov02b10_subdev_ops);

	/* Check HW config */
	ret = ov02b10_check_hwcfg(ov02b10);
	if (ret)
		return ret;

	/* Initialize regmap and subdev */
	ov02b10->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(ov02b10->regmap))
		return PTR_ERR(ov02b10->regmap);

	ret = ov02b10_get_pm_resources(ov02b10->dev);
	if (ret)
		return ret;

	ret = ov02b10_power_on(ov02b10->dev);
	if (ret) {
		dev_err_probe(ov02b10->dev, ret, "failed to power on\n");
		return ret;
	}

	/* Check module identity */
	ret = ov02b10_identify_module(ov02b10);
	if (ret) {
		dev_err(ov02b10->dev, "failed to find sensor: %d\n", ret);
		goto probe_error_power_off;
	}

	ov02b10->cur_mode = &supported_modes[0];
	ret = ov02b10_init_controls(ov02b10);
	if (ret) {
		dev_err(ov02b10->dev, "failed to init controls: %d\n", ret);
		goto probe_error_v4l2_ctrl_handler_free;
	}

	/* Initialize subdev */
	ov02b10->sd.internal_ops = &ov02b10_internal_ops;
	ov02b10->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov02b10->sd.entity.ops = &ov02b10_subdev_entity_ops;
	ov02b10->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	ov02b10->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&ov02b10->sd.entity, 1, &ov02b10->pad);
	if (ret) {
		dev_err(ov02b10->dev, "failed to init entity pads: %d\n", ret);
		goto probe_error_v4l2_ctrl_handler_free;
	}

	ov02b10->sd.state_lock = ov02b10->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&ov02b10->sd);
	if (ret < 0) {
		dev_err(ov02b10->dev, "failed to init subdev: %d\n", ret);
		goto probe_error_media_entity_cleanup;
	}

	pm_runtime_set_active(ov02b10->dev);
	pm_runtime_enable(ov02b10->dev);

	ret = v4l2_async_register_subdev_sensor(&ov02b10->sd);
	if (ret < 0) {
		dev_err(ov02b10->dev, "failed to register V4L2 subdev: %d\n",
			ret);
		goto probe_error_v4l2_subdev_cleanup;
	}

	pm_runtime_idle(ov02b10->dev);

	return 0;

probe_error_v4l2_subdev_cleanup:
	pm_runtime_disable(ov02b10->dev);
	pm_runtime_set_suspended(ov02b10->dev);
	v4l2_subdev_cleanup(&ov02b10->sd);

probe_error_media_entity_cleanup:
	media_entity_cleanup(&ov02b10->sd.entity);

probe_error_v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(ov02b10->sd.ctrl_handler);

probe_error_power_off:
	ov02b10_power_off(ov02b10->dev);

	return ret;
}

static DEFINE_RUNTIME_DEV_PM_OPS(ov02b10_pm_ops, ov02b10_power_off,
				 ov02b10_power_on, NULL);

static const struct of_device_id ov02b10_of_match[] = {
	{ .compatible = "ovti,ov02b10" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov02b10_of_match);

static struct i2c_driver ov02b10_i2c_driver = {
	.driver = {
		.name = "ov02b10",
		.pm = pm_sleep_ptr(&ov02b10_pm_ops),
		.of_match_table = ov02b10_of_match,
	},
	.probe = ov02b10_probe,
	.remove = ov02b10_remove,
};

module_i2c_driver(ov02b10_i2c_driver);

MODULE_AUTHOR("Rockchip Electronics Co., Ltd.");
MODULE_DESCRIPTION("OmniVision OV02B10 sensor driver");
MODULE_LICENSE("GPL");
