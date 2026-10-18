// SPDX-License-Identifier: GPL-2.0
/*
 * imx480.c - imx480 sensor driver
 *
 * Copyright (C) 2025 Intel Corporation
 * Copyright (C) 2026 Kate Hsuan <hpa@redhat.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>

#define IMX480_REG_MODE_SELECT			CCI_REG8(0x0100)
#define IMX480_MODE_STANDBY			0x00
#define IMX480_MODE_STREAMING			0x01

/* Chip ID */
#define IMX480_REG_CHIP_ID			CCI_REG16(0x0016)
#define IMX480_CHIP_ID				0x0471

/* V_TIMING internal */
#define IMX480_REG_FLL				CCI_REG16(0x0340)
#define IMX480_FLL_MAX				0xffff

/* Exposure control */
#define IMX480_REG_EXPOSURE			CCI_REG16(0x0202)
#define IMX480_EXPOSURE_MIN			1
#define IMX480_EXPOSURE_STEP			1
#define IMX480_EXPOSURE_DEFAULT			1270

/* Default exposure margin */
#define IMX480_EXPOSURE_MARGIN			18

/* Analog gain control */
#define IMX480_REG_ANALOG_GAIN			CCI_REG16(0x0204)
#define IMX480_ANA_GAIN_MIN			0
#define IMX480_ANA_GAIN_MAX			800
#define IMX480_ANA_GAIN_STEP			1
#define IMX480_ANA_GAIN_DEFAULT			0

/* Digital gain control */
#define IMX480_REG_DPGA_USE_GLOBAL_GAIN		CCI_REG16(0x3ff9)
#define IMX480_REG_DIG_GAIN_GLOBAL		CCI_REG16(0x020e)
#define IMX480_DGTL_GAIN_MIN			256
#define IMX480_DGTL_GAIN_MAX			4095
#define IMX480_DGTL_GAIN_STEP			1
#define IMX480_DGTL_GAIN_DEFAULT		256

/* HFLIP and VFLIP control */
#define IMX480_REG_ORIENTATION			CCI_REG8(0x0101)
#define IMX480_HFLIP_BIT			BIT(0)
#define IMX480_VFLIP_BIT			BIT(1)

/* Test Pattern Control */
#define IMX480_REG_TEST_PATTERN			CCI_REG8(0x0600)
#define IMX480_TEST_PATTERN_DISABLED		0
#define IMX480_TEST_PATTERN_SOLID_COLOR		1
#define IMX480_TEST_PATTERN_COLOR_BARS		2
#define IMX480_TEST_PATTERN_GRAY_COLOR_BARS	3
#define IMX480_TEST_PATTERN_PN9			4

/* default link frequency and external clock */
#define IMX480_LINK_FREQ_DEFAULT		729600000LL
#define IMX480_EXT_CLK				19200000
#define IMX480_LINK_FREQ_INDEX			0

/* IMX480 native and active pixel array size */
#define IMX480_NATIVE_WIDTH			4672
#define IMX480_NATIVE_HEIGHT			3512
#define IMX480_PIXEL_ARRAY_LEFT			8
#define IMX480_PIXEL_ARRAY_TOP			8
#define IMX480_PIXEL_ARRAY_WIDTH		4656
#define IMX480_PIXEL_ARRAY_HEIGHT		3496

#define IMX480_REG_EXCK_FREQ			CCI_REG16(0x0136)
#define IMX480_EXCK_FREQ(n)			((n) * 256)	/* n in MHz */

#define IMX480_REG_CSI_DATA_FORMAT		CCI_REG16(0x0112)
#define IMX480_CSI_DATA_FORMAT_RAW10		0x0a0a

#define IMX480_REG_CSI_LANE_MODE		CCI_REG8(0x0114)
#define IMX480_CSI_2_LANE_MODE			1
#define IMX480_CSI_4_LANE_MODE			3

#define IMX480_REG_X_ADD_STA			CCI_REG16(0x0344)
#define IMX480_REG_Y_ADD_STA			CCI_REG16(0x0346)
#define IMX480_REG_X_ADD_END			CCI_REG16(0x0348)
#define IMX480_REG_Y_ADD_END			CCI_REG16(0x034a)
#define IMX480_REG_X_OUTPUT_SIZE		CCI_REG16(0x034c)
#define IMX480_REG_Y_OUTPUT_SIZE		CCI_REG16(0x034e)
#define IMX480_REG_X_EVEN_INC			CCI_REG8(0x0381)
#define IMX480_REG_X_ODD_INC			CCI_REG8(0x0383)
#define IMX480_REG_Y_EVEN_INC			CCI_REG8(0x0385)
#define IMX480_REG_Y_ODD_INC			CCI_REG8(0x0387)

#define IMX480_REG_DIG_CROP_X_OFFSET		CCI_REG16(0x0408)
#define IMX480_REG_DIG_CROP_Y_OFFSET		CCI_REG16(0x040a)
#define IMX480_REG_DIG_CROP_WIDTH		CCI_REG16(0x040c)
#define IMX480_REG_DIG_CROP_HEIGHT		CCI_REG16(0x040e)

#define to_imx480(_sd) container_of_const(_sd, struct imx480, sd)

static const char * const imx480_supply_name[] = {
	"avdd",
	"dovdd",
	"dvdd",
};

struct imx480_mode {
	u32 width;
	u32 height;

	/* V-timing */
	u32 fll_def;
	u32 fll_min;

	/* H-timing */
	u32 llp;

	u32 link_freq_index;

	const struct cci_reg_sequence *default_mode_regs;
	unsigned int default_mode_regs_length;
};

struct imx480 {
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *exposure;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(imx480_supply_name)];
	struct clk *img_clk;

	struct device *dev;
	struct regmap *regmap;
};

static const struct cci_reg_sequence imx480_global_regs[] = {
	{ CCI_REG16(0x0136), 0x1333 },	/* 19.2 MHz external clock */
	{ CCI_REG8(0x3c7e), 0x03 },
	{ CCI_REG8(0x3c7f), 0x05 },
	{ CCI_REG8(0x3e35), 0x00 },
	{ CCI_REG8(0x3e36), 0x00 },
	{ CCI_REG8(0x3e37), 0x00 },
	{ CCI_REG8(0x3f7f), 0x01 },
	{ CCI_REG8(0x4431), 0x04 },
	{ CCI_REG8(0x531c), 0x01 },
	{ CCI_REG8(0x531d), 0x02 },
	{ CCI_REG8(0x531e), 0x04 },
	{ CCI_REG8(0x5928), 0x00 },
	{ CCI_REG8(0x5929), 0x2f },
	{ CCI_REG8(0x592a), 0x00 },
	{ CCI_REG8(0x592b), 0x85 },
	{ CCI_REG8(0x592c), 0x00 },
	{ CCI_REG8(0x592d), 0x32 },
	{ CCI_REG8(0x592e), 0x00 },
	{ CCI_REG8(0x592f), 0x88 },
	{ CCI_REG8(0x5930), 0x00 },
	{ CCI_REG8(0x5931), 0x3d },
	{ CCI_REG8(0x5932), 0x00 },
	{ CCI_REG8(0x5933), 0x93 },
	{ CCI_REG8(0x5938), 0x00 },
	{ CCI_REG8(0x5939), 0x24 },
	{ CCI_REG8(0x593a), 0x00 },
	{ CCI_REG8(0x593b), 0x7a },
	{ CCI_REG8(0x593c), 0x00 },
	{ CCI_REG8(0x593d), 0x24 },
	{ CCI_REG8(0x593e), 0x00 },
	{ CCI_REG8(0x593f), 0x7a },
	{ CCI_REG8(0x5940), 0x00 },
	{ CCI_REG8(0x5941), 0x2f },
	{ CCI_REG8(0x5942), 0x00 },
	{ CCI_REG8(0x5943), 0x85 },
	{ CCI_REG8(0x5f0e), 0x6e },
	{ CCI_REG8(0x5f11), 0xc6 },
	{ CCI_REG8(0x5f17), 0x5e },
	{ CCI_REG8(0x7990), 0x01 },
	{ CCI_REG8(0x7993), 0x5d },
	{ CCI_REG8(0x7994), 0x5d },
	{ CCI_REG8(0x7995), 0xa1 },
	{ CCI_REG8(0x799a), 0x01 },
	{ CCI_REG8(0x799d), 0x00 },
	{ CCI_REG8(0x8169), 0x01 },
	{ CCI_REG8(0x8359), 0x01 },
	{ CCI_REG8(0x9302), 0x1e },
	{ CCI_REG8(0x9306), 0x1f },
	{ CCI_REG8(0x930a), 0x26 },
	{ CCI_REG8(0x930e), 0x23 },
	{ CCI_REG8(0x9312), 0x23 },
	{ CCI_REG8(0x9316), 0x2c },
	{ CCI_REG8(0x9317), 0x19 },
	{ CCI_REG8(0xb046), 0x01 },
	{ CCI_REG8(0xb048), 0x01 },
	{ CCI_REG8(0x0101), 0x03 },
};

static const struct cci_reg_sequence mode_1928x1088_regs[] = {
	{ IMX480_REG_X_ADD_STA, 8 },
	{ IMX480_REG_Y_ADD_STA, 408 },
	{ IMX480_REG_X_ADD_END, 4647 },
	{ IMX480_REG_Y_ADD_END, 3051 },
	{ IMX480_REG_X_EVEN_INC, 1 },
	{ IMX480_REG_X_ODD_INC, 1 },
	{ IMX480_REG_Y_EVEN_INC, 1 },
	{ IMX480_REG_Y_ODD_INC, 1 },
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x22 },
	{ CCI_REG8(0x0902), 0x08 },
	{ IMX480_REG_DIG_CROP_X_OFFSET, 208 },
	{ IMX480_REG_DIG_CROP_Y_OFFSET, 108 },
	{ IMX480_REG_DIG_CROP_WIDTH, 1928 },
	{ IMX480_REG_DIG_CROP_HEIGHT, 1088 },
	{ IMX480_REG_X_OUTPUT_SIZE, 1928 },
	{ IMX480_REG_Y_OUTPUT_SIZE, 1088 },
	{ CCI_REG8(0x0301), 0x06 },
	{ CCI_REG8(0x0303), 0x02 },
	{ CCI_REG8(0x0305), 0x02 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0x79 },
	{ CCI_REG8(0x030b), 0x01 },
	{ CCI_REG8(0x030d), 0x02 },
	{ CCI_REG8(0x030e), 0x00 },
	{ CCI_REG8(0x030f), 0x53 },
	{ CCI_REG8(0x0310), 0x01 },
	{ IMX480_REG_EXPOSURE, IMX480_EXPOSURE_DEFAULT },
	{ CCI_REG8(0x3f4c), 0x81 },
	{ CCI_REG8(0x3f4d), 0x81 },
	{ CCI_REG8(0x3f78), 0x01 },
	{ CCI_REG8(0x3f79), 0x31 },
	{ CCI_REG8(0x3ffe), 0x00 },
	{ CCI_REG8(0x3fff), 0x8a },
	{ CCI_REG8(0x5f0a), 0xb6 },
};


static const struct cci_reg_sequence mode_2304x1728_regs[] = {
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x03 },	/* 4-lane MIPI */
	{ CCI_REG8(0x0115), 0x30 },	/* vendor mode value read back */
	{ CCI_REG16(0x0342), 0x0a00 },	/* HTS */
	{ CCI_REG16(0x0340), 0x1d66 },	/* VTS */
	{ CCI_REG16(0x0344), 0x0000 },	/* x start */
	{ CCI_REG16(0x0346), 0x0014 },	/* y start */
	{ CCI_REG16(0x0348), 0x122f },	/* x end */
	{ CCI_REG16(0x034a), 0x0d93 },	/* y end */
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x01 },
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x22 },
	{ CCI_REG8(0x0902), 0x08 },
	{ CCI_REG8(0x3f4c), 0x81 },
	{ CCI_REG8(0x3f4d), 0x81 },
	{ CCI_REG16(0x0408), 0x000c },
	{ CCI_REG16(0x040a), 0x0000 },
	{ CCI_REG16(0x040c), 0x0900 },
	{ CCI_REG16(0x040e), 0x06c0 },
	{ CCI_REG16(0x034c), 0x0900 },	/* output width 2304 */
	{ CCI_REG16(0x034e), 0x06c0 },	/* output height 1728 */
	{ CCI_REG16(0x0301), 0x0600 },
	{ CCI_REG16(0x0303), 0x0200 },
	{ CCI_REG16(0x0305), 0x0301 },
	{ CCI_REG16(0x0307), 0x0f00 },
	{ CCI_REG16(0x030b), 0x0100 },
	{ CCI_REG16(0x030d), 0x0601 },
	{ CCI_REG16(0x030f), 0xc801 },
	{ CCI_REG16(0x0310), 0x0100 },
	{ CCI_REG8(0x3f78), 0x01 },
	{ CCI_REG8(0x3f79), 0x31 },
	{ CCI_REG8(0x3ffe), 0x00 },
	{ CCI_REG8(0x3fff), 0x8a },
	{ CCI_REG8(0x5f0a), 0xb6 },
	{ CCI_REG8(0x5f0b), 0xeb },
	{ CCI_REG16(0x0202), 0x11a4 },
	{ CCI_REG16(0x020e), 0x0001 },
	{ CCI_REG8(0x3f15), 0x00 },
};

static const char * const imx480_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"Eight Vertical Colour Bars",
	"Colour Bars With Fade to Grey",
	"Pseudorandom Sequence (PN9)",
};

static const s64 link_freq_menu_items[] = {
	IMX480_LINK_FREQ_DEFAULT,
};

/*
 * The Bayer formats for the flipping.
 * - no flip
 * - h flip
 * - v flip
 * - h and v flips
 */
static const u32 imx480_hv_flips_bayer_order[] = {
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

static const struct imx480_mode imx480_modes[] = {
	{
		.width = 1928,
		.height = 1088,
		.fll_def = 1308,
		.fll_min = 1308,
		.llp = 2328,
		.link_freq_index = IMX480_LINK_FREQ_INDEX,
		.default_mode_regs = mode_1928x1088_regs,
		.default_mode_regs_length = ARRAY_SIZE(mode_1928x1088_regs),
	},
	{
		.width = 2304,
		.height = 1728,
		.fll_def = 0x1d66,
		.fll_min = 0x1d66,
		.llp = 0x0a00,
		.link_freq_index = IMX480_LINK_FREQ_INDEX,
		.default_mode_regs = mode_2304x1728_regs,
		.default_mode_regs_length = ARRAY_SIZE(mode_2304x1728_regs),
	},
};

static int imx480_get_regulators(struct device *dev, struct imx480 *sensor)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(imx480_supply_name); i++)
		sensor->supplies[i].supply = imx480_supply_name[i];

	return devm_regulator_bulk_get(dev, ARRAY_SIZE(imx480_supply_name),
				       sensor->supplies);
}

static int imx480_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx480 *sensor = container_of_const(ctrl->handler,
						   struct imx480,
						   ctrl_handler);
	struct v4l2_subdev_state *state =
			v4l2_subdev_get_locked_active_state(&sensor->sd);
	const struct v4l2_mbus_framefmt *format =
			v4l2_subdev_state_get_format(state, 0);
	s64 exposure_max;
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		exposure_max =
			format->height + ctrl->val - IMX480_EXPOSURE_MARGIN;
		ret = __v4l2_ctrl_modify_range(sensor->exposure,
					       sensor->exposure->minimum,
					       exposure_max,
					       sensor->exposure->step,
					       exposure_max);
		if (ret)
			return ret;
	}

	/* V4L2 controls values will be applied only when power is already up */
	if (!pm_runtime_get_if_in_use(sensor->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(sensor->regmap, IMX480_REG_ANALOG_GAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = cci_write(sensor->regmap, IMX480_REG_DIG_GAIN_GLOBAL,
				ctrl->val, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(sensor->regmap, IMX480_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		/* Update FLL that meets expected vertical blanking */
		ret = cci_write(sensor->regmap, IMX480_REG_FLL,
				format->height + ctrl->val, NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(sensor->regmap, IMX480_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = cci_write(sensor->regmap, IMX480_REG_ORIENTATION,
				sensor->hflip->val | sensor->vflip->val << 1, NULL);
		break;
	default:
		ret = -EINVAL;
		dev_info(sensor->dev, "ctrl(id:0x%x,val:0x%x) is not handled",
			 ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(sensor->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx480_ctrl_ops = {
	.s_ctrl = imx480_set_ctrl,
};

static u32 imx480_get_format_code(struct imx480 *sensor)
{
	unsigned int i;

	i = (sensor->vflip->val ? 2 : 0) | (sensor->hflip->val ? 1 : 0);

	return imx480_hv_flips_bayer_order[i];
}

static int imx480_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx480 *sensor = to_imx480(sd);

	if (code->index >= (ARRAY_SIZE(imx480_hv_flips_bayer_order) / 4))
		return -EINVAL;

	code->code = imx480_get_format_code(sensor);

	return 0;
}

static int imx480_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(imx480_modes))
		return -EINVAL;

	fse->min_width = imx480_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = imx480_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void imx480_update_pad_format(struct imx480 *sensor,
				     const struct imx480_mode *mode,
				     struct v4l2_subdev_format *fmt)
{
	fmt->format.code = imx480_get_format_code(sensor);
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int imx480_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx480 *sensor = to_imx480(sd);
	const struct imx480_mode *mode;
	u64 pixel_rate;
	int h_blank;
	int ret;

	mode = v4l2_find_nearest_size(imx480_modes,
				      ARRAY_SIZE(imx480_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	imx480_update_pad_format(sensor, mode, fmt);

	*v4l2_subdev_state_get_format(sd_state, fmt->pad) = fmt->format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	if (media_entity_is_streaming(&sensor->sd.entity))
		return -EBUSY;

	pixel_rate = div_u64(IMX480_LINK_FREQ_DEFAULT * 2 * 4, 10);
	ret = __v4l2_ctrl_modify_range(sensor->pixel_rate,
				       V4L2_CID_PIXEL_RATE,
				       pixel_rate, 1, pixel_rate);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(sensor->vblank,
				       mode->fll_min - mode->height,
				       IMX480_FLL_MAX - mode->height,
				       1,
				       mode->fll_def - mode->height);
	if (ret)
		return ret;

	h_blank = mode->llp - mode->width;
	/*
	 * Currently hblank is not changeable.
	 * So FPS control is done only by vblank.
	 */
	return __v4l2_ctrl_modify_range(sensor->hblank, h_blank,
					h_blank, 1, h_blank);
}

static int imx480_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(sd_state, sel->pad);
		break;

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = IMX480_NATIVE_WIDTH;
		sel->r.height = IMX480_NATIVE_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = IMX480_PIXEL_ARRAY_TOP;
		sel->r.left = IMX480_PIXEL_ARRAY_LEFT;
		sel->r.width = IMX480_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX480_PIXEL_ARRAY_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int imx480_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.format = {
			.code = MEDIA_BUS_FMT_SRGGB10_1X10,
			.width = imx480_modes[0].width,
			.height = imx480_modes[0].height,
		},
	};

	return imx480_set_pad_format(sd, sd_state, &fmt);
}

static int imx480_identify_module(struct imx480 *sensor)
{
	int ret;
	u64 val;

	ret = cci_read(sensor->regmap, IMX480_REG_CHIP_ID, &val, NULL);
	if (ret)
		return dev_err_probe(sensor->dev, ret,
				     "failed to read chip id\n");

	if (val != IMX480_CHIP_ID)
		return dev_err_probe(sensor->dev, -EIO,
				     "chip id mismatch: %x!=%llx\n",
				     IMX480_CHIP_ID, val);

	return 0;
}

static int imx480_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx480 *sensor = to_imx480(sd);

	clk_disable_unprepare(sensor->img_clk);
	gpiod_set_value_cansleep(sensor->reset_gpio, 0);

	regulator_bulk_disable(ARRAY_SIZE(imx480_supply_name), sensor->supplies);

	return 0;
}

static int imx480_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx480 *sensor = to_imx480(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(imx480_supply_name), sensor->supplies);
	if (ret < 0) {
		dev_err(dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(sensor->img_clk);
	if (ret < 0) {
		regulator_bulk_disable(ARRAY_SIZE(imx480_supply_name), sensor->supplies);
		dev_err(dev, "failed to enable imaging clock: %d", ret);
		return ret;
	}

	gpiod_set_value_cansleep(sensor->reset_gpio, 0);

	usleep_range(30000, 35000);

	return 0;
}

static int imx480_enable_stream(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				u32 pad, u64 streams_mask)
{
	struct imx480 *sensor = to_imx480(sd);
	const struct imx480_mode *mode;
	struct v4l2_mbus_framefmt *fmt;
	int ret;

	ret = pm_runtime_resume_and_get(sensor->dev);
	if (ret)
		return ret;

	ret = imx480_identify_module(sensor);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(sensor->regmap, imx480_global_regs,
				  ARRAY_SIZE(imx480_global_regs), NULL);
	if (ret) {
		dev_err(sensor->dev, "failed to set global settings: %d", ret);
		goto error_powerdown;
	}

	state = v4l2_subdev_get_locked_active_state(&sensor->sd);
	fmt = v4l2_subdev_state_get_format(state, 0);
	mode = v4l2_find_nearest_size(imx480_modes, ARRAY_SIZE(imx480_modes),
				      width, height, fmt->width, fmt->height);
	ret = cci_multi_reg_write(sensor->regmap, mode->default_mode_regs,
				  mode->default_mode_regs_length, NULL);
	if (ret) {
		dev_err(sensor->dev, "failed to set mode: %d", ret);
		goto error_powerdown;
	}

	ret = cci_write(sensor->regmap, IMX480_REG_DPGA_USE_GLOBAL_GAIN, 1, NULL);
	if (ret)
		goto error_powerdown;

	ret = __v4l2_ctrl_handler_setup(&sensor->ctrl_handler);
	if (ret)
		goto error_powerdown;

	ret = cci_write(sensor->regmap, IMX480_REG_MODE_SELECT,
			IMX480_MODE_STREAMING, NULL);
	if (ret) {
		dev_err(sensor->dev, "stream on failed: %d\n", ret);
		goto error_powerdown;
	}
	__v4l2_ctrl_grab(sensor->vflip, true);
	__v4l2_ctrl_grab(sensor->hflip, true);

	return ret;

error_powerdown:
	pm_runtime_put(sensor->dev);

	return ret;
}

static int imx480_disable_stream(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 u32 pad, u64 streams_mask)
{
	struct imx480 *sensor = to_imx480(sd);
	int ret;

	ret = cci_write(sensor->regmap, IMX480_REG_MODE_SELECT,
			IMX480_MODE_STANDBY, NULL);
	pm_runtime_put(sensor->dev);

	if (ret)
		dev_err(sensor->dev,
			"failed to disable stream with return value: %d\n",
			ret);

	__v4l2_ctrl_grab(sensor->vflip, false);
	__v4l2_ctrl_grab(sensor->hflip, false);

	return 0;
}

static const struct v4l2_subdev_video_ops imx480_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops imx480_pad_ops = {
	.enum_mbus_code = imx480_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx480_set_pad_format,
	.get_selection = imx480_get_selection,
	.enum_frame_size = imx480_enum_frame_size,
	.enable_streams = imx480_enable_stream,
	.disable_streams = imx480_disable_stream,
};

static const struct v4l2_subdev_ops imx480_subdev_ops = {
	.video = &imx480_video_ops,
	.pad = &imx480_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx480_internal_ops = {
	.init_state = imx480_init_state,
};

static int imx480_init_controls(struct imx480 *sensor)
{
	const struct imx480_mode *mode = &imx480_modes[0];
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct v4l2_ctrl *link_freq;
	s64 exposure_max, hblank;
	u64 pixel_rate;
	int ret;

	ctrl_hdlr = &sensor->ctrl_handler;
	v4l2_ctrl_handler_init(ctrl_hdlr, 12);

	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret) {
		dev_err(sensor->dev, "failed to parse fwnode: %d", ret);
		return ret;
	}

	v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx480_ctrl_ops, &props);

	link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr,
					   &imx480_ctrl_ops,
					   V4L2_CID_LINK_FREQ,
					   ARRAY_SIZE(link_freq_menu_items) - 1,
					   0,
					   link_freq_menu_items);

	/* pixel_rate = link_freq * 2 * nr_of_lanes / bits_per_sample */
	pixel_rate = div_u64(IMX480_LINK_FREQ_DEFAULT * 2 * 4, 10);

	sensor->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx480_ctrl_ops,
					       V4L2_CID_PIXEL_RATE, pixel_rate,
					       pixel_rate, 1, pixel_rate);

	sensor->vblank = v4l2_ctrl_new_std(ctrl_hdlr,
					   &imx480_ctrl_ops,
					   V4L2_CID_VBLANK,
					   mode->fll_min - mode->height,
					   IMX480_FLL_MAX - mode->height,
					   1,
					   mode->fll_def - mode->height);

	hblank = mode->llp - mode->width;
	sensor->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx480_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank,
					   1, hblank);

	/* fll >= exposure time + adjust parameter (default value is 18) */
	exposure_max = mode->fll_def - IMX480_EXPOSURE_MARGIN;
	sensor->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx480_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX480_EXPOSURE_MIN, exposure_max,
					     IMX480_EXPOSURE_STEP,
					     IMX480_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx480_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX480_ANA_GAIN_MIN, IMX480_ANA_GAIN_MAX,
			  IMX480_ANA_GAIN_STEP, IMX480_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx480_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX480_DGTL_GAIN_MIN, IMX480_DGTL_GAIN_MAX,
			  IMX480_DGTL_GAIN_STEP, IMX480_DGTL_GAIN_DEFAULT);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx480_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx480_test_pattern_menu) - 1,
				     0, 0, imx480_test_pattern_menu);

	sensor->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx480_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);

	sensor->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx480_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);

	if (ctrl_hdlr->error) {
		dev_err(sensor->dev, "%s control init failed: %d",
			__func__, ctrl_hdlr->error);
		goto error;
	}

	link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	sensor->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	sensor->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	sensor->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	sensor->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ctrl_hdlr->error;
}

static int imx480_check_hwcfg(struct imx480 *sensor)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep, *fwnode = dev_fwnode(sensor->dev);
	unsigned long link_freq_bitmap;
	struct clk *clk;
	int ret;

	clk = devm_v4l2_sensor_clk_get(sensor->dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(sensor->dev, PTR_ERR(clk),
				     "can't get clock frequency\n");

	if (clk_get_rate(clk) != IMX480_EXT_CLK)
		return dev_err_probe(sensor->dev, -EINVAL,
				     "external clock %lu is not supported\n",
				     clk_get_rate(clk));

	ep = fwnode_graph_get_endpoint_by_id(fwnode, 0, 0, 0);
	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(sensor->dev, ret,
				     "parsing endpoint failed");

	ret = v4l2_link_freq_to_bitmap(sensor->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       link_freq_menu_items,
				       ARRAY_SIZE(link_freq_menu_items),
				       &link_freq_bitmap);

	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int imx480_probe(struct i2c_client *client)
{
	struct imx480 *sensor;
	int ret;

	sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return dev_err_probe(&client->dev, -ENOMEM,
				     "failed to allocate memory\n");

	sensor->dev = &client->dev;

	ret = imx480_check_hwcfg(sensor);
	if (ret)
		return dev_err_probe(sensor->dev, ret,
				     "failed to check hwcfg: %d\n", ret);

	ret = imx480_get_regulators(sensor->dev, sensor);
	if (ret)
		return dev_err_probe(sensor->dev, ret,
				     "failed to get regulators\n");

	sensor->reset_gpio = devm_gpiod_get_optional(sensor->dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio))
		return dev_err_probe(sensor->dev, PTR_ERR(sensor->reset_gpio),
				     "failed to get reset gpio\n");

	sensor->img_clk = devm_clk_get_optional(sensor->dev, NULL);
	if (IS_ERR(sensor->img_clk))
		return dev_err_probe(sensor->dev, PTR_ERR(sensor->img_clk),
				     "failed to get imaging clock\n");

	v4l2_i2c_subdev_init(&sensor->sd, client, &imx480_subdev_ops);

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return dev_err_probe(sensor->dev, PTR_ERR(sensor->regmap),
				     "failed to initialize CCI\n");

	ret = imx480_power_on(sensor->dev);
	if (ret)
		return dev_err_probe(sensor->dev, ret,
				     "failed to power on\n");

	ret = imx480_identify_module(sensor);
	if (ret) {
		dev_err_probe(sensor->dev, ret, "failed to find sensor: %d", ret);
		goto error_power_off;
	}

	ret = imx480_init_controls(sensor);
	if (ret) {
		dev_err_probe(sensor->dev, ret, "failed to init controls: %d", ret);
		goto error_power_off;
	}

	sensor->sd.internal_ops = &imx480_internal_ops;
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret) {
		dev_err_probe(sensor->dev, ret, "failed to init entity pads: %d", ret);
		goto error_v4l2_ctrl_handler_free;
	}

	sensor->sd.state_lock = sensor->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&sensor->sd);
	if (ret < 0) {
		dev_err_probe(sensor->dev, ret, "failed to init subdev: %d", ret);
		goto error_media_entity_pm;
	}

	pm_runtime_set_active(sensor->dev);
	pm_runtime_enable(sensor->dev);

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret < 0)
		goto error_v4l2_subdev_cleanup;

	pm_runtime_idle(sensor->dev);

	return 0;

error_v4l2_subdev_cleanup:
	pm_runtime_disable(sensor->dev);
	pm_runtime_set_suspended(sensor->dev);
	v4l2_subdev_cleanup(&sensor->sd);

error_media_entity_pm:
	media_entity_cleanup(&sensor->sd.entity);

error_v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(sensor->sd.ctrl_handler);

error_power_off:
	imx480_power_off(sensor->dev);

	return ret;
}

static void imx480_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);

	if (!pm_runtime_status_suspended(&client->dev)) {
		imx480_power_off(&client->dev);
		pm_runtime_set_suspended(&client->dev);
	}
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx480_pm_ops, imx480_power_off,
				 imx480_power_on, NULL);

static const struct acpi_device_id imx480_acpi_ids[] __maybe_unused = {
	{ "SONY471A" },
	{ "TBE20A0" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(acpi, imx480_acpi_ids);

static const struct of_device_id imx480_of_ids[] = {
	{ .compatible = "sony,imx480" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx480_of_ids);

static struct i2c_driver imx480_i2c_driver = {
	.driver = {
		.name = "imx480",
		.acpi_match_table = ACPI_PTR(imx480_acpi_ids),
		.of_match_table = imx480_of_ids,
		.pm = pm_sleep_ptr(&imx480_pm_ops),
	},
	.probe = imx480_probe,
	.remove = imx480_remove,
};
module_i2c_driver(imx480_i2c_driver);

MODULE_AUTHOR("Jimmy Su <jimmy.su@intel.com>");
MODULE_AUTHOR("Serin Yeh <serin.yeh@intel.com>");
MODULE_AUTHOR("Kate Hsuan <hpa@redhat.com>");
MODULE_DESCRIPTION("Sony imx480 sensor driver");
MODULE_LICENSE("GPL");
