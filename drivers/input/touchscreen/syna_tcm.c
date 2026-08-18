// SPDX-License-Identifier: GPL-2.0-only
/*
 * Synaptics TouchComm (TCM) v1 SPI touchscreen driver
 *
 * Synaptics S3910 as found in OnePlus 13R / Ace 5 (giulia). The chip runs
 * TouchComm v1 application firmware (identify: "S3910PA0B0-15.2", mode 0x01):
 * plain unsequenced packets without CRCs, reports pushed via ATTN. Written
 * against the protocol implemented by the downstream synaptics_hbp driver
 * (synaptics_touchcom_core_v1.c):
 *   Copyright (C) 2017-2020 Synaptics Incorporated
 *
 * Copyright (c) 2026, Oleg Peshkov
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define TCM_HDR_SIZE			4
#define TCM_MAX_PAYLOAD			512
/* largest transfer: continued-read hdr + payload + end-of-message pad */
#define TCM_MAX_XFER			(TCM_HDR_SIZE + TCM_MAX_PAYLOAD + 2)
#define TCM_MAX_OBJECTS			10

/* timings from the downstream DT / synaptics_touchcom_core_v1.c */
#define TCM_POWER_ON_DELAY_MS		200
#define TCM_RESET_ACTIVE_MS		10
#define TCM_RESET_DELAY_MS		80
#define TCM_TAT_DELAY_MIN_US		100	/* bus turnaround: cmd -> resp */
#define TCM_TAT_DELAY_MAX_US		200
#define TCM_WR_DELAY_MIN_US		500	/* between (re)transmissions */
#define TCM_WR_DELAY_MAX_US		1000
#define TCM_CMD_TIMEOUT_MS		3000
#define TCM_RETRY_COUNT			5

/* message codes < 0x10 are command responses, >= 0x10 are async reports */
#define TCM_STATUS_IDLE			0x00
#define TCM_STATUS_OK			0x01
#define TCM_STATUS_BUSY			0x02
#define TCM_STATUS_CONTINUED_READ	0x03
#define TCM_STATUS_NO_REPORT_AVAILABLE	0x04
#define TCM_STATUS_NOT_IMPLEMENTED	0x0e
#define TCM_STATUS_ERROR		0x0f
#define TCM_REPORT_IDENTIFY		0x10
#define TCM_REPORT_TOUCH		0x11

#define TCM_CMD_IDENTIFY		0x02

/*
 * v1 framing: writes are [cmd][len lo][len hi][payload]; every read
 * begins with the 0xa5 marker, [0xa5][code][len lo][len hi], and the
 * payload is fetched with continued reads ([0xa5][0x03] + data),
 * terminated by one 0x5a padding byte. No sequence bits, no CRCs.
 */
#define TCM_V1_MARKER			0xa5
#define TCM_V1_PADDING			0x5a
#define TCM_CMD_RESET			0x04
#define TCM_CMD_RUN_APP_FIRMWARE	0x14
#define TCM_CMD_GET_APPLICATION_INFO	0x20
#define TCM_CMD_GET_TOUCH_REPORT_CONFIG	0x25
#define TCM_CMD_REZERO			0x27
#define TCM_CMD_ENTER_DEEP_SLEEP	0x2c
#define TCM_CMD_EXIT_DEEP_SLEEP		0x2d

#define TCM_MODE_APPLICATION		0x01
#define TCM_APP_STATUS_OK		0x0000

/* touch report config codes (structural codes carry no size byte) */
enum tcm_touch_code {
	TOUCH_END = 0,
	TOUCH_FOREACH_ACTIVE_OBJECT,
	TOUCH_FOREACH_OBJECT,
	TOUCH_FOREACH_END,
	TOUCH_PAD_TO_NEXT_BYTE,
	TOUCH_TIMESTAMP,
	TOUCH_OBJECT_N_INDEX,
	TOUCH_OBJECT_N_CLASSIFICATION,
	TOUCH_OBJECT_N_X_POSITION,
	TOUCH_OBJECT_N_Y_POSITION,
	TOUCH_OBJECT_N_Z,
	TOUCH_OBJECT_N_X_WIDTH,
	TOUCH_OBJECT_N_Y_WIDTH,
	TOUCH_OBJECT_N_TX_POSITION_TIXELS,
	TOUCH_OBJECT_N_RX_POSITION_TIXELS,
	TOUCH_0D_BUTTONS_STATE,
	TOUCH_GESTURE_DOUBLE_TAP,
	TOUCH_FRAME_RATE,
	TOUCH_POWER_IM,
	TOUCH_CID_IM,
	TOUCH_RAIL_IM,
	TOUCH_CID_VARIANCE_IM,
	TOUCH_NSM_FREQUENCY,
	TOUCH_NSM_STATE,
	TOUCH_NUM_OF_ACTIVE_OBJECTS,
	TOUCH_NUM_OF_CPU_CYCLES_USED_SINCE_LAST_FRAME,
};

/* object classifications */
#define TOUCH_LIFT			0

struct tcm_identification {
	u8 version;
	u8 mode;
	u8 part_number[16];
	u8 build_id[4];
	u8 max_write_size[2];
	/* extension in TouchComm v2 */
	u8 max_read_size[2];
	u8 reserved[6];
} __packed;

struct tcm_app_info {
	__le16 version;
	__le16 status;
	__le16 static_config_size;
	__le16 dynamic_config_size;
	__le16 app_config_start_write_block;
	__le16 app_config_size;
	__le16 max_touch_report_config_size;
	__le16 max_touch_report_payload_size;
	u8 customer_config_id[16];
	__le16 max_x;
	__le16 max_y;
	__le16 max_objects;
	__le16 num_of_buttons;
	__le16 num_of_image_rows;
	__le16 num_of_image_cols;
	__le16 has_hybrid_data;
} __packed;

struct tcm_object {
	unsigned int x;
	unsigned int y;
	unsigned int z;
	unsigned int major;
	bool active;
};

struct syna_tcm {
	struct spi_device *spi;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[2];

	/* serializes all bus messaging (commands and IRQ report pulls) */
	struct mutex msg_lock;

	struct tcm_identification id_info;
	u8 config[256];
	u16 config_size;
	unsigned int max_objects;

	struct tcm_object objects[TCM_MAX_OBJECTS];

	u8 msg_buf[TCM_MAX_PAYLOAD];	/* assembled message payload */
	u8 tx_buf[TCM_MAX_XFER];
	u8 rx_buf[TCM_MAX_XFER];
	u8 ff_buf[TCM_MAX_XFER];
};

/*
 * Send one v1 packet: 1-byte command, 16-bit LE payload length, then
 * the payload. No sequence bits, no CRCs.
 * Must be called with msg_lock held.
 */
static int syna_tcm_write_packet(struct syna_tcm *ts, u8 cmd,
				 const u8 *payload, u16 plen)
{
	if (WARN_ON(plen > TCM_MAX_PAYLOAD))
		return -EINVAL;

	ts->tx_buf[0] = cmd;
	put_unaligned_le16(plen, &ts->tx_buf[1]);
	if (plen)
		memcpy(&ts->tx_buf[3], payload, plen);

	return spi_write(ts->spi, ts->tx_buf, 3 + plen);
}

/*
 * Read the 4-byte v1 message header: [0xa5][code][len lo][len hi].
 * Retried a few times since the device returns filler bytes while it
 * is composing a message. Must be called with msg_lock held.
 */
static int syna_tcm_read_header(struct syna_tcm *ts, u8 *code, u16 *length)
{
	struct spi_transfer xfer = {
		.tx_buf = ts->ff_buf,
		.rx_buf = ts->rx_buf,
		.len = TCM_HDR_SIZE,
	};
	unsigned int retries;
	int ret;

	for (retries = 10; retries; retries--) {
		ret = spi_sync_transfer(ts->spi, &xfer, 1);
		if (ret)
			return ret;

		if (ts->rx_buf[0] == TCM_V1_MARKER) {
			*code = ts->rx_buf[1];
			*length = get_unaligned_le16(&ts->rx_buf[2]);
			return 0;
		}

		dev_dbg(&ts->spi->dev, "bad header marker, raw %4ph\n",
			ts->rx_buf);
		usleep_range(TCM_WR_DELAY_MIN_US, TCM_WR_DELAY_MAX_US);
	}

	return -EIO;
}

/*
 * Read one complete v1 message. The header transaction returns the
 * message code and total payload length; the payload plus one trailing
 * padding byte then arrives in continued-read chunks, each prefixed
 * with [0xa5][STATUS_CONTINUED_READ].
 *
 * The payload (clamped to TCM_MAX_PAYLOAD) is left in msg_buf.
 * Must be called with msg_lock held.
 */
static int syna_tcm_read_message(struct syna_tcm *ts, u8 *code, u16 *length)
{
	unsigned int total, remaining, offset = 0, chunk, copy;
	struct spi_transfer xfer = {
		.tx_buf = ts->ff_buf,
		.rx_buf = ts->rx_buf,
	};
	u16 hdr_len;
	int ret;

	ret = syna_tcm_read_header(ts, code, &hdr_len);
	if (ret)
		return ret;

	total = hdr_len;
	*length = min_t(unsigned int, total, TCM_MAX_PAYLOAD);
	if (!total)
		return 0;

	/* payload plus the end-of-message padding byte */
	remaining = total + 1;

	while (remaining) {
		chunk = min_t(unsigned int, remaining, TCM_MAX_PAYLOAD + 1);

		xfer.len = chunk + 2;
		ret = spi_sync_transfer(ts->spi, &xfer, 1);
		if (ret)
			return ret;

		if (ts->rx_buf[0] != TCM_V1_MARKER ||
		    ts->rx_buf[1] != TCM_STATUS_CONTINUED_READ) {
			dev_dbg(&ts->spi->dev,
				"broken continued read, raw %4ph\n",
				ts->rx_buf);
			return -EBADMSG;
		}

		if (offset < total && offset < TCM_MAX_PAYLOAD) {
			copy = min3(chunk, total - offset,
				    TCM_MAX_PAYLOAD - offset);
			memcpy(&ts->msg_buf[offset], &ts->rx_buf[2], copy);
		}

		offset += chunk;
		remaining -= chunk;
	}

	return 0;
}

/*
 * Send a command and collect the answer. TouchComm v1 has no
 * retransmission protocol: while the device is still preparing the
 * response it answers with an IDLE header, so poll briefly. If the
 * response is still not ready when the polling budget expires, IDLE is
 * handed back and the caller decides how much longer to wait.
 * Must be called with msg_lock held.
 */
static int syna_tcm_transact(struct syna_tcm *ts, u8 cmd, const u8 *payload,
			     u16 plen, u8 *code, u16 *length)
{
	unsigned int attempts;
	int ret;

	ret = syna_tcm_write_packet(ts, cmd, payload, plen);
	if (ret)
		return ret;

	usleep_range(TCM_TAT_DELAY_MIN_US, TCM_TAT_DELAY_MAX_US);

	for (attempts = 50; attempts; attempts--) {
		ret = syna_tcm_read_message(ts, code, length);
		if (!ret && *code != TCM_STATUS_IDLE)
			return 0;
		if (ret && ret != -EBADMSG && ret != -EIO)
			return ret;

		usleep_range(TCM_WR_DELAY_MIN_US, TCM_WR_DELAY_MAX_US);
	}

	*code = TCM_STATUS_IDLE;
	*length = 0;
	return 0;
}

static void syna_tcm_report_touch(struct syna_tcm *ts, const u8 *data,
				  unsigned int len);

/* handle an async report currently sitting in msg_buf */
static void syna_tcm_dispatch_report(struct syna_tcm *ts, u8 code, u16 len)
{
	switch (code) {
	case TCM_REPORT_TOUCH:
		if (ts->input && ts->config_size)
			syna_tcm_report_touch(ts, ts->msg_buf, len);
		break;
	case TCM_REPORT_IDENTIFY:
		memcpy(&ts->id_info, ts->msg_buf,
		       min_t(size_t, len, sizeof(ts->id_info)));
		break;
	default:
		dev_dbg(&ts->spi->dev, "ignoring report 0x%02x (%u bytes)\n",
			code, len);
		break;
	}
}

/*
 * Execute a command and return its response payload.
 *
 * If the device is still busy (or an unrelated report arrives first),
 * the response is awaited by reading further messages off the bus.
 * RESET and RUN_APPLICATION_FIRMWARE are answered with an IDENTIFY
 * report instead of a plain response.
 *
 * Returns the response payload length (>= 0) on success.
 */
static int syna_tcm_cmd(struct syna_tcm *ts, u8 cmd, const u8 *payload,
			u16 plen, void *resp, size_t resp_size)
{
	struct device *dev = &ts->spi->dev;
	unsigned long timeout;
	u8 code;
	u16 len;
	int ret;

	guard(mutex)(&ts->msg_lock);

	ret = syna_tcm_transact(ts, cmd, payload, plen, &code, &len);
	if (ret)
		return ret;

	timeout = jiffies + msecs_to_jiffies(TCM_CMD_TIMEOUT_MS);

	while (code != TCM_STATUS_OK) {
		if (code >= TCM_REPORT_IDENTIFY) {
			syna_tcm_dispatch_report(ts, code, len);
			if (code == TCM_REPORT_IDENTIFY &&
			    (cmd == TCM_CMD_RESET ||
			     cmd == TCM_CMD_RUN_APP_FIRMWARE)) {
				len = 0;
				break;
			}
			/* unrelated report: keep polling for our response */
		} else if (code != TCM_STATUS_IDLE &&
			   code != TCM_STATUS_BUSY &&
			   code != TCM_STATUS_NO_REPORT_AVAILABLE) {
			dev_err(dev, "command 0x%02x failed, status 0x%02x\n",
				cmd, code);
			return -EIO;
		}

		if (time_after(jiffies, timeout)) {
			dev_dbg(dev, "command 0x%02x timed out\n", cmd);
			return -ETIMEDOUT;
		}

		msleep(20);

		ret = syna_tcm_read_message(ts, &code, &len);
		if (ret == -EBADMSG || ret == -EIO)
			code = TCM_STATUS_IDLE;
		else if (ret)
			return ret;
	}

	if (resp)
		memcpy(resp, ts->msg_buf, min_t(size_t, resp_size, len));

	return len;
}

/*
 * In v1 the device pushes reports on its own: ATTN asserting means a
 * message is waiting to be read off the bus.
 */
static irqreturn_t syna_tcm_irq(int irq, void *data)
{
	struct syna_tcm *ts = data;
	u8 code;
	u16 len;
	int ret;

	guard(mutex)(&ts->msg_lock);

	ret = syna_tcm_read_message(ts, &code, &len);
	if (ret) {
		dev_dbg(&ts->spi->dev, "failed to read report: %d\n", ret);
		return IRQ_HANDLED;
	}

	if (code >= TCM_REPORT_IDENTIFY)
		syna_tcm_dispatch_report(ts, code, len);

	return IRQ_HANDLED;
}

/* extract a little-endian bit field from the report */
static int syna_tcm_get_bits(const u8 *data, unsigned int total_bits,
			     unsigned int offset, unsigned int bits,
			     unsigned int *value)
{
	unsigned int v = 0;
	unsigned int i, pos;

	if (bits > 32 || offset + bits > total_bits)
		return -EINVAL;

	for (i = 0; i < bits; i++) {
		pos = offset + i;
		v |= ((data[pos >> 3] >> (pos & 7)) & 1) << i;
	}

	*value = v;
	return 0;
}

/*
 * Parse a touch report according to the touch report configuration
 * retrieved from the firmware. The config is a byte stream: one code
 * byte, followed by one size-in-bits byte for data fields. Structural
 * codes (END, FOREACH_*, PAD) have no size byte.
 */
static void syna_tcm_report_touch(struct syna_tcm *ts, const u8 *data,
				  unsigned int len)
{
	struct input_dev *input = ts->input;
	unsigned int total_bits = len * 8;
	unsigned int idx = 0, offset = 0;
	unsigned int obj = 0, next = 0;
	unsigned int active_objects = 0, found_objects = 0;
	bool active_only = false, have_active_count = false;
	unsigned int val, bits;
	unsigned int i;
	u8 code;

	memset(ts->objects, 0, sizeof(ts->objects));

	while (idx < ts->config_size) {
		code = ts->config[idx++];

		switch (code) {
		case TOUCH_END:
			goto sync;
		case TOUCH_FOREACH_ACTIVE_OBJECT:
			obj = 0;
			next = idx;
			active_only = true;
			break;
		case TOUCH_FOREACH_OBJECT:
			obj = 0;
			next = idx;
			active_only = false;
			break;
		case TOUCH_FOREACH_END:
			if (active_only) {
				if (have_active_count) {
					found_objects++;
					if (found_objects < active_objects)
						idx = next;
				} else if (offset < total_bits) {
					idx = next;
				}
			} else {
				obj++;
				if (obj < ts->max_objects)
					idx = next;
			}
			break;
		case TOUCH_PAD_TO_NEXT_BYTE:
			offset = round_up(offset, 8);
			break;
		case TOUCH_OBJECT_N_INDEX:
			bits = ts->config[idx++];
			if (syna_tcm_get_bits(data, total_bits, offset, bits,
					      &obj))
				goto sync;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_CLASSIFICATION:
			bits = ts->config[idx++];
			if (syna_tcm_get_bits(data, total_bits, offset, bits,
					      &val))
				goto sync;
			if (obj < TCM_MAX_OBJECTS)
				ts->objects[obj].active = val != TOUCH_LIFT;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_X_POSITION:
			bits = ts->config[idx++];
			if (syna_tcm_get_bits(data, total_bits, offset, bits,
					      &val))
				goto sync;
			if (obj < TCM_MAX_OBJECTS)
				ts->objects[obj].x = val;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Y_POSITION:
			bits = ts->config[idx++];
			if (syna_tcm_get_bits(data, total_bits, offset, bits,
					      &val))
				goto sync;
			if (obj < TCM_MAX_OBJECTS)
				ts->objects[obj].y = val;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Z:
			bits = ts->config[idx++];
			if (syna_tcm_get_bits(data, total_bits, offset, bits,
					      &val))
				goto sync;
			if (obj < TCM_MAX_OBJECTS)
				ts->objects[obj].z = val;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_X_WIDTH:
		case TOUCH_OBJECT_N_Y_WIDTH:
			bits = ts->config[idx++];
			if (syna_tcm_get_bits(data, total_bits, offset, bits,
					      &val))
				goto sync;
			if (obj < TCM_MAX_OBJECTS)
				ts->objects[obj].major =
					max(ts->objects[obj].major, val);
			offset += bits;
			break;
		case TOUCH_NUM_OF_ACTIVE_OBJECTS:
			bits = ts->config[idx++];
			if (syna_tcm_get_bits(data, total_bits, offset, bits,
					      &active_objects))
				goto sync;
			have_active_count = true;
			offset += bits;
			if (!active_objects && active_only) {
				/* skip the foreach block entirely */
				while (idx < ts->config_size &&
				       ts->config[idx] != TOUCH_FOREACH_END)
					idx++;
			}
			break;
		default:
			/* unknown data field: skip using its size byte */
			if (idx < ts->config_size) {
				bits = ts->config[idx++];
				offset += bits;
			}
			break;
		}
	}

sync:
	for (i = 0; i < ts->max_objects; i++) {
		struct tcm_object *o = &ts->objects[i];

		input_mt_slot(input, i);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, o->active);
		if (o->active) {
			touchscreen_report_pos(input, &ts->prop, o->x, o->y,
					       true);
			input_report_abs(input, ABS_MT_PRESSURE,
					 min(o->z, 255U));
			input_report_abs(input, ABS_MT_TOUCH_MAJOR,
					 min(o->major, 255U));
		}
	}
	input_mt_sync_frame(input);
	input_sync(input);
}

static void syna_tcm_power_off(void *data)
{
	struct syna_tcm *ts = data;

	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ts->supplies), ts->supplies);
}

/*
 * After a hard reset the controller queues an IDENTIFY report and
 * asserts ATTN; in v1 it is read straight off the bus. Fall back to an
 * explicit IDENTIFY command in case the startup report was already
 * consumed.
 */
static int syna_tcm_detect(struct syna_tcm *ts)
{
	unsigned int retries;
	u8 code;
	u16 len;
	int ret;

	scoped_guard(mutex, &ts->msg_lock) {
		for (retries = 20; retries; retries--) {
			ret = syna_tcm_read_message(ts, &code, &len);
			if (!ret && code == TCM_REPORT_IDENTIFY) {
				syna_tcm_dispatch_report(ts, code, len);
				return 0;
			}
			if (!ret && code != TCM_STATUS_IDLE)
				dev_dbg(&ts->spi->dev,
					"detect: code 0x%02x len %u\n",
					code, len);
			msleep(50);
		}
		dev_dbg(&ts->spi->dev,
			"no IDENTIFY report (last err %d, raw header %4ph), trying explicit IDENTIFY\n",
			ret, ts->rx_buf);
	}

	ret = syna_tcm_cmd(ts, TCM_CMD_IDENTIFY, NULL, 0,
			   &ts->id_info, sizeof(ts->id_info));
	if (ret < 0)
		dev_warn(&ts->spi->dev,
			 "IDENTIFY failed (%d), last raw header %4ph\n",
			 ret, ts->rx_buf);
	return ret < 0 ? ret : 0;
}

static int syna_tcm_start_application(struct syna_tcm *ts)
{
	struct device *dev = &ts->spi->dev;
	struct tcm_app_info app_info;
	unsigned int retries;
	int ret;

	ret = syna_tcm_detect(ts);
	if (ret)
		return dev_err_probe(dev, ret, "no TouchComm v1 device found\n");

	dev_info(dev, "TCM %.*s, TouchComm v%u, mode 0x%02x\n",
		 (int)sizeof(ts->id_info.part_number), ts->id_info.part_number,
		 ts->id_info.version, ts->id_info.mode);

	if (ts->id_info.mode != TCM_MODE_APPLICATION) {
		ret = syna_tcm_cmd(ts, TCM_CMD_RUN_APP_FIRMWARE, NULL, 0,
				   NULL, 0);
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "failed to start app firmware\n");
		if (ts->id_info.mode != TCM_MODE_APPLICATION)
			return dev_err_probe(dev, -ENODEV,
					     "stuck in mode 0x%02x\n",
					     ts->id_info.mode);
	}

	for (retries = 10; retries; retries--) {
		ret = syna_tcm_cmd(ts, TCM_CMD_GET_APPLICATION_INFO, NULL, 0,
				   &app_info, sizeof(app_info));
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "failed to get app info\n");
		if (le16_to_cpu(app_info.status) == TCM_APP_STATUS_OK)
			break;
		msleep(100);
	}
	if (le16_to_cpu(app_info.status) != TCM_APP_STATUS_OK)
		return dev_err_probe(dev, -ENODEV, "bad app status 0x%04x\n",
				     le16_to_cpu(app_info.status));

	ts->max_objects = min_t(unsigned int,
				le16_to_cpu(app_info.max_objects),
				TCM_MAX_OBJECTS);
	if (!ts->max_objects)
		ts->max_objects = TCM_MAX_OBJECTS;

	ret = syna_tcm_cmd(ts, TCM_CMD_GET_TOUCH_REPORT_CONFIG, NULL, 0,
			   ts->config, sizeof(ts->config));
	if (ret <= 0)
		return dev_err_probe(dev, ret ? ret : -ENODEV,
				     "failed to get touch report config\n");
	ts->config_size = min_t(unsigned int, ret, sizeof(ts->config));

	dev_dbg(dev, "max %ux%u, %u objects, %u byte report config\n",
		le16_to_cpu(app_info.max_x), le16_to_cpu(app_info.max_y),
		ts->max_objects, ts->config_size);

	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0,
			     le16_to_cpu(app_info.max_x), 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0,
			     le16_to_cpu(app_info.max_y), 0, 0);

	return 0;
}

static int syna_tcm_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct syna_tcm *ts;
	int ret;

	if (!spi->irq)
		return dev_err_probe(dev, -EINVAL, "no IRQ specified\n");

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->spi = spi;
	spi_set_drvdata(spi, ts);
	mutex_init(&ts->msg_lock);
	memset(ts->ff_buf, 0xff, sizeof(ts->ff_buf));

	spi->bits_per_word = 8;
	if (!spi->mode)
		spi->mode = SPI_MODE_0;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set up SPI\n");

	ts->supplies[0].supply = "vdd";
	ts->supplies[1].supply = "avdd";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ts->supplies),
				      ts->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	ts->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						 GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get reset GPIO\n");

	ret = regulator_bulk_enable(ARRAY_SIZE(ts->supplies), ts->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable regulators\n");

	ret = devm_add_action_or_reset(dev, syna_tcm_power_off, ts);
	if (ret)
		return ret;

	msleep(TCM_POWER_ON_DELAY_MS);
	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	msleep(TCM_RESET_ACTIVE_MS);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(TCM_RESET_DELAY_MS);

	ts->input = devm_input_allocate_device(dev);
	if (!ts->input)
		return -ENOMEM;

	ts->input->name = "Synaptics TCM Touchscreen";
	ts->input->phys = "syna_tcm/input0";
	ts->input->id.bustype = BUS_SPI;

	ret = syna_tcm_start_application(ts);
	if (ret)
		return ret;

	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	touchscreen_parse_properties(ts->input, true, &ts->prop);

	ret = input_mt_init_slots(ts->input, ts->max_objects,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init MT slots\n");

	ret = input_register_device(ts->input);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register input\n");

	/*
	 * Level-low ATTN: if a report is already pending, the handler
	 * fires and pulls it as soon as the IRQ is enabled.
	 */
	ret = devm_request_threaded_irq(dev, spi->irq, NULL, syna_tcm_irq,
					IRQF_ONESHOT, "syna_tcm", ts);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	return 0;
}

static int syna_tcm_suspend(struct device *dev)
{
	struct syna_tcm *ts = spi_get_drvdata(to_spi_device(dev));

	disable_irq(ts->spi->irq);
	syna_tcm_cmd(ts, TCM_CMD_ENTER_DEEP_SLEEP, NULL, 0, NULL, 0);

	return 0;
}

static int syna_tcm_resume(struct device *dev)
{
	struct syna_tcm *ts = spi_get_drvdata(to_spi_device(dev));

	syna_tcm_cmd(ts, TCM_CMD_EXIT_DEEP_SLEEP, NULL, 0, NULL, 0);
	syna_tcm_cmd(ts, TCM_CMD_REZERO, NULL, 0, NULL, 0);
	enable_irq(ts->spi->irq);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(syna_tcm_pm_ops,
				syna_tcm_suspend, syna_tcm_resume);

static const struct of_device_id syna_tcm_of_match[] = {
	{ .compatible = "syna,s3910" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, syna_tcm_of_match);

static const struct spi_device_id syna_tcm_spi_id[] = {
	{ "s3910" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, syna_tcm_spi_id);

static struct spi_driver syna_tcm_driver = {
	.probe = syna_tcm_probe,
	.id_table = syna_tcm_spi_id,
	.driver = {
		.name = "syna_tcm",
		.of_match_table = syna_tcm_of_match,
		.pm = pm_sleep_ptr(&syna_tcm_pm_ops),
	},
};
module_spi_driver(syna_tcm_driver);

MODULE_AUTHOR("Oleg Peshkov <olegos.wst@gmail.com>");
MODULE_DESCRIPTION("Synaptics TouchComm TCM v1 SPI touchscreen driver");
MODULE_LICENSE("GPL");
