// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022, Linaro Ltd
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */
#include <linux/auxiliary_bus.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/devm-helpers.h>
#include <linux/jiffies.h>
#include <linux/kstrtox.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/of_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/soc/qcom/pdr.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/sysfs.h>
#include <linux/math.h>
#include <linux/units.h>
#include <linux/workqueue.h>

#define BATTMGR_CHEMISTRY_LEN	4
#define BATTMGR_STRING_LEN	128

enum qcom_battmgr_variant {
	QCOM_BATTMGR_SC8280XP,
	QCOM_BATTMGR_SM8350,
	QCOM_BATTMGR_SM8550,
	QCOM_BATTMGR_OPLUS_SM8650,
	QCOM_BATTMGR_X1E80100,
};

#define BATTMGR_BAT_STATUS		0x1

#define BATTMGR_REQUEST_NOTIFICATION	0x4

#define BATTMGR_NOTIFICATION		0x7
#define NOTIF_BAT_PROPERTY		0x30
#define NOTIF_USB_PROPERTY		0x32
#define NOTIF_WLS_PROPERTY		0x34
#define NOTIF_BAT_STATUS		0x80
#define NOTIF_BAT_INFO			0x81
#define NOTIF_BAT_CHARGING_STATE	0x83

/*
 * Oplus firmware notification codes (extracted from downstream
 * oplus_hal_adsp.h). Only OTG enable/disable need dedicated handling;
 * the rest is charger state chatter which just triggers a refresh.
 */
#define NOTIF_OPLUS_FIRST		0x50
#define NOTIF_OPLUS_OTG_ENABLE		0x50
#define NOTIF_OPLUS_OTG_DISABLE		0x51
#define NOTIF_OPLUS_LAST		0x7f

#define BATTMGR_BAT_INFO		0x9

#define BATTMGR_BAT_DISCHARGE_TIME	0xc

#define BATTMGR_BAT_CHARGE_TIME		0xd

#define BATTMGR_BAT_PROPERTY_GET	0x30
#define BATTMGR_BAT_PROPERTY_SET	0x31
#define BATT_STATUS			0
#define BATT_HEALTH			1
#define BATT_PRESENT			2
#define BATT_CHG_TYPE			3
#define BATT_CAPACITY			4
#define BATT_SOH			5
#define BATT_VOLT_OCV			6
#define BATT_VOLT_NOW			7
#define BATT_VOLT_MAX			8
#define BATT_CURR_NOW			9
#define BATT_CHG_CTRL_LIM		10
#define BATT_CHG_CTRL_LIM_MAX		11
#define BATT_TEMP			12
#define BATT_TECHNOLOGY			13
#define BATT_CHG_COUNTER		14
#define BATT_CYCLE_COUNT		15
#define BATT_CHG_FULL_DESIGN		16
#define BATT_CHG_FULL			17
#define BATT_MODEL_NAME			18
#define BATT_TTF_AVG			19
#define BATT_TTE_AVG			20
#define BATT_RESISTANCE			21
#define BATT_POWER_NOW			22
#define BATT_POWER_AVG			23
#define BATT_CHG_CTRL_EN		24
#define BATT_CHG_CTRL_START_THR		25
#define BATT_CHG_CTRL_END_THR		26
#define BATTMGR_USB_PROPERTY_GET	0x32
#define BATTMGR_USB_PROPERTY_SET	0x33
#define USB_ONLINE			0
#define USB_VOLT_NOW			1
#define USB_VOLT_MAX			2
#define USB_CURR_NOW			3
#define USB_CURR_MAX			4
#define USB_INPUT_CURR_LIMIT		5
#define USB_TYPE			6
#define USB_ADAP_TYPE			7
#define USB_MOISTURE_DET_EN		8
#define USB_MOISTURE_DET_STS		9
/* Oplus-specific usb property: write 1/0 to source VBUS for OTG */
#define USB_OPLUS_OTG_VBUS_ENABLE	24

/*
 * Oplus battery properties (extracted from downstream
 * enum battery_property_id).
 */
#define BATT_OPLUS_ADSP_GAUGE_INIT	38

/* Oplus-specific request: read the OEM status buffer (see keepalive worker) */
#define BATTMGR_OPLUS_OEM_READ_BUFFER	0x10000
#define BATTMGR_OPLUS_OEM_BUFFER_WORDS	128

#define BATTMGR_OPLUS_KEEPALIVE_INTERVAL_MS	5000

/* QTI charger types reported in USB_ADAP_TYPE beyond the standard ones */
#define QTI_USB_TYPE_HVDCP		0x80

#define BATTMGR_WLS_PROPERTY_GET	0x34
#define BATTMGR_WLS_PROPERTY_SET	0x35
#define WLS_ONLINE			0
#define WLS_VOLT_NOW			1
#define WLS_VOLT_MAX			2
#define WLS_CURR_NOW			3
#define WLS_CURR_MAX			4
#define WLS_TYPE			5
#define WLS_BOOST_EN			6

#define BATTMGR_CHG_CTRL_LIMIT_EN	0x48
#define CHARGE_CTRL_START_THR_MIN	50
#define CHARGE_CTRL_START_THR_MAX	95
#define CHARGE_CTRL_END_THR_MIN		55
#define CHARGE_CTRL_END_THR_MAX		100
#define CHARGE_CTRL_DELTA_SOC		5

/* OnePlus firmware extension used to retrieve the partner PDO table. */
#define BATTMGR_ONEPLUS_PPS_READ_BUFFER	0x10004
#define BATTMGR_ONEPLUS_PPS_WORDS	128
#define BATTMGR_ONEPLUS_PPS_PDO_MAX	7
#define BATTMGR_ONEPLUS_PPS_TIMEOUT_MS	500
#define BATTMGR_ONEPLUS_BATT_SET_PDO	25
#define BATTMGR_ONEPLUS_USB_SET_PPS_VOLT	34
#define BATTMGR_ONEPLUS_USB_SET_PPS_CURR	35
#define BATTMGR_ONEPLUS_FIXED_5V_MV	5000
#define BATTMGR_ONEPLUS_TEST_PPS_MV	5500
#define BATTMGR_ONEPLUS_TEST_PPS_MA	1000
#define BATTMGR_ONEPLUS_RAMP_PPS_MAX_MV	9000
#define BATTMGR_ONEPLUS_RAMP_PPS_STEP_MV	500
#define BATTMGR_ONEPLUS_RAMP_PPS_9V2_MAX_MV	9200
#define BATTMGR_ONEPLUS_RAMP_PPS_FINE_STEP_MV	100
#define BATTMGR_ONEPLUS_RAMP_DELAY_MS	500
#define BATTMGR_ONEPLUS_CP_2TO1_VBAT_MIN_UV	3000000
#define BATTMGR_ONEPLUS_CP_2TO1_VBAT_MAX_UV	4350000
#define BATTMGR_ONEPLUS_CP_2TO1_TARGET_STEP_UV	100000
#define BATTMGR_ONEPLUS_CP_2TO1_START_DELTA_UV	450000
#define BATTMGR_ONEPLUS_CP_2TO1_START_BAND_UV	200000
#define BATTMGR_ONEPLUS_CP_2TO1_START_MARGIN_MIN_UV	0
#define BATTMGR_ONEPLUS_CP_2TO1_DIAG_MARGIN_TARGET_UV	100000
#define BATTMGR_ONEPLUS_CP_2TO1_DIAG_FINE_STEP_UV	20000
#define BATTMGR_ONEPLUS_CP_2TO1_SOURCE_MAX_MV	9400
#define BATTMGR_ONEPLUS_CP_FEEDBACK_ATTEMPTS	8
#define BATTMGR_ONEPLUS_CP_DUAL_FEEDBACK_ATTEMPTS	20
#define BATTMGR_ONEPLUS_CP_FEEDBACK_RECORDS \
	BATTMGR_ONEPLUS_CP_DUAL_FEEDBACK_ATTEMPTS
#define BATTMGR_ONEPLUS_CP_FEEDBACK_STABLE_SAMPLES	2
#define BATTMGR_ONEPLUS_CP_FEEDBACK_CONFIRM_MISSES	2
#define BATTMGR_ONEPLUS_CP_FEEDBACK_CONFIRM_TOLERANCE_UV	40000
#define BATTMGR_ONEPLUS_CP_ADC_READY_ATTEMPTS	6
#define BATTMGR_ONEPLUS_CP_ADC_READY_DELAY_MS	20
#define BATTMGR_ONEPLUS_PRIMARY_CP_NAME	"sc8547-primary"
#define BATTMGR_ONEPLUS_SECONDARY_CP_NAME	"sc8547-secondary"
#define BATTMGR_ONEPLUS_CP_SECONDARY_TEST_MS	500
#define BATTMGR_ONEPLUS_CP_DUAL_SECONDARY_RUN_MS	625000
#define BATTMGR_ONEPLUS_CP_DUAL_PRIMARY_RUN_MS	630000
#define BATTMGR_ONEPLUS_CP_DUAL_PRIMARY_SETTLE_MS	650
#define BATTMGR_ONEPLUS_CP_DUAL_ENTRY_WAIT_MS		200
#define BATTMGR_ONEPLUS_CP_DUAL_ENTRY_WAIT_RETRIES	15
#define BATTMGR_ONEPLUS_CP_DUAL_SECONDARY_SETTLE_MS	650
#define BATTMGR_ONEPLUS_CP_DUAL_IDLE_SETTLE_MS	1000
#define BATTMGR_ONEPLUS_CP_DUAL_ENTRY_DERATE_MV	40
#define BATTMGR_ONEPLUS_CP_DUAL_ENTRY_SETTLE_MS	500
#define BATTMGR_ONEPLUS_CP_DUAL_PPS_MA	1750
#define BATTMGR_ONEPLUS_CP_DUAL_BOOST_PPS_MA	3000
#define BATTMGR_ONEPLUS_CP_DUAL_BOOST_MS	500
#define BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEP_MV	40
#define BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEPS	10
#define BATTMGR_ONEPLUS_CP_DUAL_BOOST_TARGET_UA	2000000
#define BATTMGR_ONEPLUS_CP_DUAL_BOOST_DEADBAND_UA	100000
#define BATTMGR_ONEPLUS_CP_DUAL_BOOST_HANDOFF_MIN_UA	900000
#define BATTMGR_ONEPLUS_CP_DUAL_BOOST_HANDOFF_PUMP_MIN_UA	400000
#define BATTMGR_ONEPLUS_CP_DUAL_HOLD_SOFT_IBUS_UA	1050000
#define BATTMGR_ONEPLUS_CP_DUAL_LOW_IBUS_CONFIRM_SAMPLES	4
#define BATTMGR_ONEPLUS_CP_DUAL_LOW_IBUS_STEP_MV	20
#define BATTMGR_ONEPLUS_CP_DUAL_EMERGENCY_IBUS_MAX_UA	1250000
#define BATTMGR_ONEPLUS_CP_DUAL_IBUS_ADC_LSB_UA	1875
#define BATTMGR_ONEPLUS_CP_DUAL_EMERGENCY_IBUS_ADC_MAX_UA	\
	(((BATTMGR_ONEPLUS_CP_DUAL_EMERGENCY_IBUS_MAX_UA + \
	   BATTMGR_ONEPLUS_CP_DUAL_IBUS_ADC_LSB_UA - 1) / \
	  BATTMGR_ONEPLUS_CP_DUAL_IBUS_ADC_LSB_UA) * \
	 BATTMGR_ONEPLUS_CP_DUAL_IBUS_ADC_LSB_UA)
#define BATTMGR_ONEPLUS_CP_DUAL_EMERGENCY_STEP_MV	100
#define BATTMGR_ONEPLUS_CP_DUAL_EMERGENCY_MAX_STEPS	3
#define BATTMGR_ONEPLUS_CP_DUAL_PRIMARY_TAPER_UV	4300000
#define BATTMGR_ONEPLUS_CP_DUAL_SECONDARY_TAPER_UV	4450000
#define BATTMGR_ONEPLUS_CP_DUAL_SECONDARY_MAX_UV	4500000
#define BATTMGR_ONEPLUS_CP_DUAL_PATH_TAPER_UV	150000
#define BATTMGR_ONEPLUS_CP_DUAL_PATH_MAX_UV	200000
#define BATTMGR_ONEPLUS_CP_DUAL_OUTPUT_TAPER_STEP_MV	100
#define BATTMGR_ONEPLUS_CP_DUAL_FULL_VBAT_UV	4300000
#define BATTMGR_ONEPLUS_CP_DUAL_FULL_IBAT_MAX_UA	500000
#define BATTMGR_ONEPLUS_CP_DUAL_FULL_CAPACITY_MIN	95
#define BATTMGR_ONEPLUS_CP_DUAL_FULL_CHECKS	7
#define BATTMGR_ONEPLUS_CP_DUAL_VNOW_CV_ENTER_UV	4300000
#define BATTMGR_ONEPLUS_CP_DUAL_VNOW_CV_RELEASE_UV	4275000
#define BATTMGR_ONEPLUS_CP_DUAL_VNOW_CV_STEP_MV	40
#define BATTMGR_ONEPLUS_CP_DUAL_POST_STOP_SETTLE_MS	1000
#define BATTMGR_ONEPLUS_CP_DUAL_HOLD_MS		600000
#define BATTMGR_ONEPLUS_CP_DUAL_HOLD_SAMPLE_MS	500
#define BATTMGR_ONEPLUS_CP_DUAL_HOLD_RECORDS	20
#define BATTMGR_ONEPLUS_CP_DUAL_HOLD_TRACE_STRIDE	64
#define BATTMGR_ONEPLUS_CP_DUAL_MONITOR_STRIDE	4
#define BATTMGR_ONEPLUS_CP_DUAL_HOLD_SAMPLES	\
	(BATTMGR_ONEPLUS_CP_DUAL_HOLD_MS / \
	 BATTMGR_ONEPLUS_CP_DUAL_HOLD_SAMPLE_MS)
#define BATTMGR_ONEPLUS_CP_DUAL_BOOST_MAX_MV	\
	(BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEP_MV * \
	 BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEPS)
#define BATTMGR_ONEPLUS_CP_DUAL_IBUS_MAX_UA	1200000
#define BATTMGR_ONEPLUS_CP_DUAL_IBUS_RECHECKS	2
#define BATTMGR_ONEPLUS_CP_DUAL_IBUS_RECHECK_MIN_US	10000
#define BATTMGR_ONEPLUS_CP_DUAL_IBUS_RECHECK_MAX_US	20000
#define BATTMGR_ONEPLUS_CP_DUAL_SOURCE_STEP_MV	20
#define BATTMGR_ONEPLUS_CP_DUAL_SOURCE_SETTLE_MS	500
#define BATTMGR_ONEPLUS_CP_PREFLIGHT_VBUS_MIN_UV	6200000
#define BATTMGR_ONEPLUS_CP_LOADED_TARGET_MARGIN_UV	50000
#define BATTMGR_ONEPLUS_CP_RUN_5M_MS	300000
#define BATTMGR_ONEPLUS_CP_POLICY_RUN_MS	900000
#define BATTMGR_ONEPLUS_CP_POLICY_CANCEL	2
#define BATTMGR_ONEPLUS_CP_POLICY_CONTINUOUS	3
#define BATTMGR_ONEPLUS_POLICY_CHECK_MS	2000
#define BATTMGR_ONEPLUS_POLICY_VBAT_MAX_UV	4300000
#define BATTMGR_ONEPLUS_POLICY_TEMP_MIN_DECIC	100
#define BATTMGR_ONEPLUS_POLICY_TEMP_MAX_DECIC	450
#define BATTMGR_ONEPLUS_POLICY_FEEDBACK_FIRST_CHECK	15
#define BATTMGR_ONEPLUS_POLICY_FEEDBACK_INTERVAL_CHECKS	15
#define BATTMGR_ONEPLUS_POLICY_FEEDBACK_DECISIONS	6
#define BATTMGR_ONEPLUS_POLICY_FEEDBACK_STEP_MV	20
#define BATTMGR_ONEPLUS_POLICY_FEEDBACK_SETTLE_MS	500
#define BATTMGR_ONEPLUS_POLICY_FEEDBACK_TARGET_IBUS_UA	1000000
#define BATTMGR_ONEPLUS_POLICY_FEEDBACK_DEADBAND_UA	100000
#define BATTMGR_ONEPLUS_POLICY_PPS_MA		1750
#define BATTMGR_ONEPLUS_POLICY_IBUS_MIN_UA	500000
#define BATTMGR_ONEPLUS_POLICY_IBUS_LOW_CHECKS	3
#define BATTMGR_ONEPLUS_POLICY_AUTO_CHECK_MS	5000
#define BATTMGR_ONEPLUS_POLICY_AUTO_RETRIES	3
#define BATTMGR_ONEPLUS_POLICY_AUTO_VBAT_MAX_UV	4200000
#define BATTMGR_ONEPLUS_POLICY_AUTO_VBUS_MIN_UV	4000000
#define BATTMGR_ONEPLUS_DUAL_AUTO_CHECK_MS	1000
#define BATTMGR_ONEPLUS_DUAL_AUTO_OFFLINE_CHECKS	2
#define BATTMGR_ONEPLUS_DUAL_AUTO_VBUS_MIN_UV	4000000
#define BATTMGR_ONEPLUS_DUAL_AUTO_WINDOW_LIMIT	2
#define BATTMGR_ONEPLUS_POLICY_REVISION		"stage7d13-bounded-emergency-recovery"
#define BATTMGR_ONEPLUS_USB_ICL_SUSPEND_UA	0
#define BATTMGR_ONEPLUS_USB_ICL_UNSUSPEND_UA	U32_MAX
#define BATTMGR_ONEPLUS_VBAT_ZERO_RETRIES	8
#define BATTMGR_ONEPLUS_VBAT_RETRY_MS	100
#define BATTMGR_ONEPLUS_VBAT_CACHE_MAX_AGE_MS	5000

enum qcom_battmgr_oneplus_2to1_stage {
	BATTMGR_ONEPLUS_2TO1_IDLE,
	BATTMGR_ONEPLUS_2TO1_USB_ONLINE,
	BATTMGR_ONEPLUS_2TO1_VBAT,
	BATTMGR_ONEPLUS_2TO1_CAPABILITIES,
	BATTMGR_ONEPLUS_2TO1_CP_PREPARE,
	BATTMGR_ONEPLUS_2TO1_BUCK_SUSPEND,
	BATTMGR_ONEPLUS_2TO1_BUCK_ACTIVE,
	BATTMGR_ONEPLUS_2TO1_RAMP,
	BATTMGR_ONEPLUS_2TO1_FEEDBACK,
	BATTMGR_ONEPLUS_2TO1_COMPLETE,
};

enum qcom_battmgr_oneplus_policy_stage {
	BATTMGR_ONEPLUS_POLICY_IDLE,
	BATTMGR_ONEPLUS_POLICY_RAMP,
	BATTMGR_ONEPLUS_POLICY_START,
	BATTMGR_ONEPLUS_POLICY_ACTIVE,
	BATTMGR_ONEPLUS_POLICY_STOPPING,
	BATTMGR_ONEPLUS_POLICY_COMPLETE,
	BATTMGR_ONEPLUS_POLICY_FAILED,
};

enum qcom_battmgr_oneplus_policy_stop_reason {
	BATTMGR_ONEPLUS_POLICY_STOP_NONE,
	BATTMGR_ONEPLUS_POLICY_STOP_USER,
	BATTMGR_ONEPLUS_POLICY_STOP_DURATION,
	BATTMGR_ONEPLUS_POLICY_STOP_VBAT_HIGH,
	BATTMGR_ONEPLUS_POLICY_STOP_TEMP,
	BATTMGR_ONEPLUS_POLICY_STOP_USB_DETACH,
	BATTMGR_ONEPLUS_POLICY_STOP_INPUT_LOSS,
	BATTMGR_ONEPLUS_POLICY_STOP_SERVICE,
	BATTMGR_ONEPLUS_POLICY_STOP_SOURCE_RETARGET,
	BATTMGR_ONEPLUS_POLICY_STOP_CP,
	BATTMGR_ONEPLUS_POLICY_STOP_STARTUP,
	BATTMGR_ONEPLUS_POLICY_STOP_REMOVE,
};

struct qcom_battmgr_enable_request {
	struct pmic_glink_hdr hdr;
	__le32 battery_id;
	__le32 power_state;
	__le32 low_capacity;
	__le32 high_capacity;
};

struct qcom_battmgr_property_request {
	struct pmic_glink_hdr hdr;
	__le32 battery;
	__le32 property;
	__le32 value;
};

struct qcom_battmgr_update_request {
	struct pmic_glink_hdr hdr;
	__le32 battery_id;
};

struct qcom_battmgr_charge_time_request {
	struct pmic_glink_hdr hdr;
	__le32 battery_id;
	__le32 percent;
	__le32 reserved;
};

struct qcom_battmgr_discharge_time_request {
	struct pmic_glink_hdr hdr;
	__le32 battery_id;
	__le32 rate; /* 0 for current rate */
	__le32 reserved;
};

struct qcom_battmgr_charge_ctrl_request {
	struct pmic_glink_hdr hdr;
	__le32 enable;
	__le32 target_soc;
	__le32 delta_soc;
};

struct qcom_battmgr_oneplus_pps_request {
	struct pmic_glink_hdr hdr;
	__le32 data_size;
};

struct qcom_battmgr_oneplus_pps_response {
	struct pmic_glink_hdr hdr;
	__le32 data[BATTMGR_ONEPLUS_PPS_WORDS];
	__le32 data_size;
};

struct qcom_battmgr_message {
	struct pmic_glink_hdr hdr;
	union {
		struct {
			__le32 property;
			__le32 value;
			__le32 result;
		} intval;
		struct {
			__le32 property;
			char model[BATTMGR_STRING_LEN];
		} strval;
		struct {
			/*
			 * 0: mWh
			 * 1: mAh
			 */
			__le32 power_unit;
			__le32 design_capacity;
			__le32 last_full_capacity;
			/*
			 * 0 nonrechargable
			 * 1 rechargable
			 */
			__le32 battery_tech;
			__le32 design_voltage; /* mV */
			__le32 capacity_low;
			__le32 capacity_warning;
			__le32 cycle_count;
			/* thousandth of percent */
			__le32 accuracy;
			__le32 max_sample_time_ms;
			__le32 min_sample_time_ms;
			__le32 max_average_interval_ms;
			__le32 min_average_interval_ms;
			/* granularity between low and warning */
			__le32 capacity_granularity1;
			/* granularity between warning and full */
			__le32 capacity_granularity2;
			/*
			 * 0: no
			 * 1: cold
			 * 2: hot
			 */
			__le32 swappable;
			__le32 capabilities;
			char model_number[BATTMGR_STRING_LEN];
			char serial_number[BATTMGR_STRING_LEN];
			char battery_type[BATTMGR_STRING_LEN];
			char oem_info[BATTMGR_STRING_LEN];
			char battery_chemistry[BATTMGR_CHEMISTRY_LEN];
			char uid[BATTMGR_STRING_LEN];
			__le32 critical_bias;
			u8 day;
			u8 month;
			__le16 year;
			__le32 battery_id;
		} info;
		struct {
			/*
			 * BIT(0) discharging
			 * BIT(1) charging
			 * BIT(2) critical low
			 */
			__le32 battery_state;
			/* mWh or mAh, based on info->power_unit */
			__le32 capacity;
			__le32 rate;
			/* mv */
			__le32 battery_voltage;
			/*
			 * BIT(0) power online
			 * BIT(1) discharging
			 * BIT(2) charging
			 * BIT(3) battery critical
			 */
			__le32 power_state;
			/*
			 * 1: AC
			 * 2: USB
			 * 3: Wireless
			 */
			__le32 charging_source;
			__le32 temperature;
		} status;
		__le32 time;
		__le32 notification;
	};
};

#define BATTMGR_CHARGING_SOURCE_AC	1
#define BATTMGR_CHARGING_SOURCE_USB	2
#define BATTMGR_CHARGING_SOURCE_WIRELESS 3

enum qcom_battmgr_unit {
	QCOM_BATTMGR_UNIT_mWh = 0,
	QCOM_BATTMGR_UNIT_mAh = 1
};

struct qcom_battmgr_info {
	bool valid;

	bool present;
	unsigned int charge_type;
	unsigned int design_capacity;
	unsigned int last_full_capacity;
	unsigned int voltage_max_design;
	unsigned int voltage_max;
	unsigned int capacity_low;
	unsigned int capacity_warning;
	unsigned int cycle_count;
	unsigned int charge_count;
	unsigned int charge_ctrl_start;
	unsigned int charge_ctrl_end;
	char model_number[BATTMGR_STRING_LEN];
	char serial_number[BATTMGR_STRING_LEN];
	char oem_info[BATTMGR_STRING_LEN];
	unsigned char technology;
	unsigned char day;
	unsigned char month;
	unsigned short year;
};

struct qcom_battmgr_status {
	unsigned int status;
	unsigned int health;
	unsigned int capacity;
	unsigned int percent;
	int current_now;
	int power_now;
	unsigned int voltage_now;
	unsigned int voltage_ocv;
	unsigned int temperature;
	unsigned int resistance;
	unsigned int soh_percent;

	unsigned int discharge_time;
	unsigned int charge_time;
};

struct qcom_battmgr_ac {
	bool online;
};

struct qcom_battmgr_usb {
	bool online;
	unsigned int voltage_now;
	unsigned int voltage_max;
	unsigned int current_now;
	unsigned int current_max;
	unsigned int current_limit;
	unsigned int usb_type;
};

struct qcom_battmgr_wireless {
	bool online;
	unsigned int voltage_now;
	unsigned int voltage_max;
	unsigned int current_now;
	unsigned int current_max;
};

struct qcom_battmgr {
	struct device *dev;
	struct pmic_glink_client *client;

	enum qcom_battmgr_variant variant;

	struct power_supply *ac_psy;
	struct power_supply *bat_psy;
	struct power_supply *usb_psy;
	struct power_supply *wls_psy;

	enum qcom_battmgr_unit unit;

	int error;
	struct completion ack;
	struct completion oneplus_pps_ack;
	struct qcom_battmgr_oneplus_pps_response oneplus_pps;
	int oneplus_pps_error;
	bool oneplus_pps_read_buffer;
	bool oneplus_pps_fixed_5v;
	bool oneplus_pps_5v5_1a;
	bool oneplus_pps_ramp_9v_1a;
	bool oneplus_pps_ramp_9v2_1a;
	bool oneplus_pps_ramp_2to1_1a;
	bool oneplus_pps_run_2to1_5m;
	bool oneplus_pps_policy_2to1_1a;
	bool oneplus_pps_secondary_500ms;
	bool oneplus_pps_dual_500ms;
	int oneplus_pps_2to1_result;
	enum qcom_battmgr_oneplus_2to1_stage oneplus_pps_2to1_stage;
	u32 oneplus_pps_vbat_uv;
	u32 oneplus_pps_vbat_attempts;
	bool oneplus_pps_vbat_from_cache;
	u32 oneplus_pps_vbat_cache_uv;
	unsigned long oneplus_pps_vbat_cache_at;
	u32 oneplus_pps_pump_target_mv;
	u32 oneplus_pps_source_request_mv;
	u32 oneplus_pps_current_ma;
	u32 oneplus_pps_cp_vbat_uv;
	u32 oneplus_pps_cp_vin_uv;
	u32 oneplus_pps_cp_adc_attempts;
	bool oneplus_pps_cp_output_uses_vout;
	u32 oneplus_pps_companion_vin_uv;
	u32 oneplus_pps_companion_output_uv;
	u32 oneplus_pps_companion_adc_attempts;
	int oneplus_pps_companion_prepare_rc;
	u32 oneplus_pps_feedback_attempts;
	u32 oneplus_pps_feedback_stable_samples;
	u32 oneplus_pps_feedback_request_mv[BATTMGR_ONEPLUS_CP_FEEDBACK_RECORDS];
	u32 oneplus_pps_feedback_vin_uv[BATTMGR_ONEPLUS_CP_FEEDBACK_RECORDS];
	bool oneplus_pps_cp_prepared;
	int oneplus_pps_cp_prepare_rc;
	int oneplus_pps_cp_restore_rc;
	bool oneplus_pps_buck_suspended;
	int oneplus_pps_buck_suspend_rc;
	int oneplus_pps_buck_restore_rc;
	bool oneplus_pps_5m_running;
	bool oneplus_pps_5m_valid;
	int oneplus_pps_5m_result;
	int oneplus_pps_5m_ramp_rc;
	int oneplus_pps_5m_cp_run_rc;
	int oneplus_pps_5m_cp_cleanup_rc;
	int oneplus_pps_5m_source_restore_rc;
	bool oneplus_secondary_running;
	bool oneplus_secondary_valid;
	int oneplus_secondary_result;
	int oneplus_secondary_ramp_rc;
	int oneplus_secondary_cp_run_rc;
	int oneplus_secondary_cp_cleanup_rc;
	int oneplus_secondary_source_restore_rc;
	int oneplus_secondary_pre_primary_off_rc;
	int oneplus_secondary_pre_secondary_off_rc;
	int oneplus_secondary_handoff_primary_off_rc;
	int oneplus_secondary_final_primary_off_rc;
	int oneplus_secondary_final_secondary_off_rc;
	bool oneplus_dual_running;
	bool oneplus_dual_valid;
	bool oneplus_dual_stop_requested;
	bool oneplus_dual_continuous_run;
	bool oneplus_dual_hold_full_confirmed;
	bool oneplus_dual_hold_vnow_cv_active;
	bool oneplus_dual_hold_output_limited;
	bool oneplus_dual_hold_output_taper_active;
	int oneplus_dual_result;
	int oneplus_dual_ramp_rc;
	int oneplus_dual_precondition_5v_rc;
	int oneplus_dual_entry_derate_rc;
	int oneplus_dual_primary_entry_rc;
	int oneplus_dual_primary_pre_rc;
	int oneplus_dual_primary_post_rc;
	int oneplus_dual_secondary_post_rc;
	int oneplus_dual_primary_run_rc;
	int oneplus_dual_primary_cancel_rc;
	int oneplus_dual_secondary_cancel_rc;
	int oneplus_dual_source_step_rc;
	int oneplus_dual_primary_boost_pre_rc;
	int oneplus_dual_secondary_boost_pre_rc;
	int oneplus_dual_source_boost_rc;
	int oneplus_dual_primary_boost_post_rc;
	int oneplus_dual_secondary_boost_post_rc;
	int oneplus_dual_hold_rc;
	int oneplus_dual_source_reduce_rc;
	int oneplus_dual_secondary_loaded_check_rc;
	int oneplus_dual_secondary_target_rc;
	int oneplus_dual_secondary_run_rc;
	int oneplus_dual_primary_cleanup_rc;
	int oneplus_dual_secondary_cleanup_rc;
	int oneplus_dual_source_restore_rc;
	int oneplus_dual_pre_primary_off_rc;
	int oneplus_dual_pre_secondary_off_rc;
	int oneplus_dual_final_primary_off_rc;
	int oneplus_dual_final_secondary_off_rc;
	u32 oneplus_dual_primary_pre_ibus_ua;
	u32 oneplus_dual_primary_post_ibus_ua;
	u32 oneplus_dual_secondary_post_ibus_ua;
	u32 oneplus_dual_primary_entry_ibus_ua;
	u32 oneplus_dual_primary_boost_pre_ibus_ua;
	u32 oneplus_dual_secondary_boost_pre_ibus_ua;
	u32 oneplus_dual_primary_boost_post_ibus_ua;
	u32 oneplus_dual_secondary_boost_post_ibus_ua;
	u32 oneplus_dual_boost_attempts;
	u32 oneplus_dual_boost_request_mv[
		BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEPS];
	u32 oneplus_dual_boost_primary_ibus_ua[
		BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEPS];
	u32 oneplus_dual_boost_secondary_ibus_ua[
		BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEPS];
	u32 oneplus_dual_hold_samples;
	u32 oneplus_dual_hold_full_checks;
	u32 oneplus_dual_hold_vnow_cv_adjustments;
	u32 oneplus_dual_hold_adjustments;
	u32 oneplus_dual_hold_adjustments_up;
	u32 oneplus_dual_hold_adjustments_down;
	u32 oneplus_dual_hold_low_ibus_checks;
	u32 oneplus_dual_hold_emergency_adjustments;
	u32 oneplus_dual_hold_emergency_recoveries;
	u32 oneplus_dual_hold_emergency_steps;
	u32 oneplus_dual_hold_emergency_max_steps;
	u32 oneplus_dual_hold_output_taper_adjustments;
	u32 oneplus_dual_hold_primary_output_uv;
	u32 oneplus_dual_hold_secondary_output_uv;
	u32 oneplus_dual_hold_primary_output_max_uv;
	u32 oneplus_dual_hold_secondary_output_max_uv;
	u32 oneplus_dual_hold_path_drop_uv;
	u32 oneplus_dual_hold_path_drop_max_uv;
	u32 oneplus_dual_hold_records;
	u32 oneplus_dual_monitor_checks;
	u32 oneplus_dual_monitor_vbat_uv;
	u32 oneplus_dual_monitor_vbat_ocv_uv;
	u32 oneplus_dual_monitor_source_request_mv;
	u32 oneplus_dual_monitor_primary_output_uv;
	u32 oneplus_dual_monitor_secondary_output_uv;
	u32 oneplus_dual_monitor_capacity;
	u32 oneplus_dual_monitor_usb_online;
	int oneplus_dual_monitor_ibat_ua;
	int oneplus_dual_monitor_temp_decic;
	int oneplus_dual_monitor_rc;
	u32 oneplus_dual_post_stop_vbat_uv;
	u32 oneplus_dual_post_stop_vbat_ocv_uv;
	int oneplus_dual_post_stop_ibat_ua;
	int oneplus_dual_post_stop_rc;
	u32 oneplus_dual_ibus_rechecks;
	u32 oneplus_dual_ibus_recoveries;
	u32 oneplus_dual_hold_sample_no[
		BATTMGR_ONEPLUS_CP_DUAL_HOLD_RECORDS];
	u32 oneplus_dual_hold_request_mv[
		BATTMGR_ONEPLUS_CP_DUAL_HOLD_RECORDS];
	u32 oneplus_dual_hold_primary_ibus_ua[
		BATTMGR_ONEPLUS_CP_DUAL_HOLD_RECORDS];
	u32 oneplus_dual_hold_secondary_ibus_ua[
		BATTMGR_ONEPLUS_CP_DUAL_HOLD_RECORDS];
	u32 oneplus_dual_entry_derate_from_mv;
	u32 oneplus_dual_entry_derate_to_mv;
	u32 oneplus_dual_source_from_mv;
	u32 oneplus_dual_source_to_mv;
	u32 oneplus_dual_secondary_loaded_vbus_uv;
	u32 oneplus_dual_secondary_loaded_output_uv;
	u32 oneplus_dual_secondary_loaded_target_uv;
	struct work_struct oneplus_dual_auto_run_work;
	struct delayed_work oneplus_dual_auto_watch_work;
	bool oneplus_dual_auto_enabled;
	bool oneplus_dual_auto_armed;
	bool oneplus_dual_auto_connected;
	bool oneplus_dual_auto_run;
	bool oneplus_dual_auto_cancel_requested;
	bool oneplus_dual_auto_continuous;
	bool oneplus_dual_auto_full_stop;
	u32 oneplus_dual_auto_attempts;
	u32 oneplus_dual_auto_starts;
	u32 oneplus_dual_auto_stops;
	u32 oneplus_dual_auto_offline_checks;
	u32 oneplus_dual_auto_windows_started;
	u32 oneplus_dual_auto_windows_completed;
	int oneplus_dual_auto_last_rc;
	struct work_struct oneplus_policy_work;
	struct delayed_work oneplus_policy_monitor_work;
	struct delayed_work oneplus_policy_auto_work;
	bool oneplus_policy_running;
	bool oneplus_policy_valid;
	bool oneplus_policy_stop_requested;
	bool oneplus_policy_auto_start;
	bool oneplus_policy_auto_armed;
	bool oneplus_policy_auto_run;
	enum qcom_battmgr_oneplus_policy_stage oneplus_policy_stage;
	enum qcom_battmgr_oneplus_policy_stop_reason oneplus_policy_stop_reason;
	int oneplus_policy_result;
	int oneplus_policy_ramp_rc;
	int oneplus_policy_cp_run_rc;
	int oneplus_policy_cp_cancel_rc;
	int oneplus_policy_cp_cleanup_rc;
	int oneplus_policy_source_restore_rc;
	int oneplus_policy_monitor_rc;
	u32 oneplus_policy_monitor_checks;
	u32 oneplus_policy_elapsed_ms;
	u32 oneplus_policy_vbat_uv;
	u32 oneplus_policy_ibus_ua;
	u32 oneplus_policy_ibus_low_checks;
	u32 oneplus_policy_auto_attempts;
	u32 oneplus_policy_auto_starts;
	u32 oneplus_policy_feedback_decisions;
	u32 oneplus_policy_feedback_up;
	u32 oneplus_policy_feedback_down;
	u32 oneplus_policy_feedback_hold;
	u32 oneplus_policy_feedback_last_ibus_ua;
	u32 oneplus_policy_retarget_attempts;
	u32 oneplus_policy_retarget_completed;
	u32 oneplus_policy_retarget_first_from_mv;
	u32 oneplus_policy_retarget_last_to_mv;
	u32 oneplus_policy_retarget_last_pre_vbus_uv;
	u32 oneplus_policy_retarget_last_post_vbus_uv;
	int oneplus_policy_temp_decic;
	int oneplus_policy_feedback_last_delta_mv;
	int oneplus_policy_retarget_rc;
	int oneplus_policy_auto_last_rc;
	unsigned long oneplus_policy_started_at;
	bool oneplus_ext_pending;
	u32 oneplus_ext_opcode;
	u32 oneplus_ext_property;

	bool service_up;

	struct qcom_battmgr_info info;
	struct qcom_battmgr_status status;
	struct qcom_battmgr_ac ac;
	struct qcom_battmgr_usb usb;
	struct qcom_battmgr_wireless wireless;

	struct work_struct enable_work;

	/* Oplus firmware support */
	struct work_struct otg_work;
	bool otg_vbus;
	struct delayed_work oplus_keepalive_work;
	bool oplus_gauge_kicked;

	/*
	 * @lock is used to prevent concurrent power supply requests to the
	 * firmware, as it then stops responding.
	 */
	struct mutex lock;
};

struct qcom_battmgr_oneplus_cp_run_ctx {
	struct completion done;
	struct power_supply *cp_psy;
	u32 run_ms;
	int result;
};

static int qcom_battmgr_oneplus_pps_request(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_oneplus_pps_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_ONEPLUS_PPS_READ_BUFFER),
		.data_size = cpu_to_le32(sizeof(battmgr->oneplus_pps.data)),
	};
	unsigned long left;
	int ret;

	if (!battmgr->service_up)
		return -ENOTCONN;

	reinit_completion(&battmgr->oneplus_pps_ack);
	battmgr->oneplus_pps_error = -EINPROGRESS;
	memset(&battmgr->oneplus_pps, 0, sizeof(battmgr->oneplus_pps));

	ret = pmic_glink_send(battmgr->client, &request, sizeof(request));
	if (ret < 0)
		return ret;

	left = wait_for_completion_timeout(&battmgr->oneplus_pps_ack,
					   msecs_to_jiffies(BATTMGR_ONEPLUS_PPS_TIMEOUT_MS));
	if (!left)
		return -ETIMEDOUT;

	return battmgr->oneplus_pps_error;
}

static ssize_t oneplus_pps_capabilities_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	u32 data_size;
	u32 pdo_count;
	ssize_t len = 0;
	int ret;
	int i;

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_oneplus_pps_request(battmgr);
	if (ret)
		goto out_unlock;

	data_size = le32_to_cpu(battmgr->oneplus_pps.data_size);
	len += sysfs_emit_at(buf, len, "data_size=%u\n", data_size);
	len += sysfs_emit_at(buf, len, "imax_raw=%u\n",
			     le32_to_cpu(battmgr->oneplus_pps.data[0]));
	len += sysfs_emit_at(buf, len, "vmax_raw=%u\n",
			     le32_to_cpu(battmgr->oneplus_pps.data[1]));

	pdo_count = min_t(u32, data_size / sizeof(__le32) - 2,
			  BATTMGR_ONEPLUS_PPS_PDO_MAX);
	for (i = 0; i < pdo_count; i++) {
		u32 pdo = le32_to_cpu(battmgr->oneplus_pps.data[i + 2]);
		u32 type = pdo >> 30;

		len += sysfs_emit_at(buf, len, "pdo%d=0x%08x", i, pdo);
		if (type == 3)
			len += sysfs_emit_at(buf, len, " pps=%u-%umV %umA",
					     ((pdo >> 8) & 0xff) * 100,
					     ((pdo >> 17) & 0xff) * 100,
					     (pdo & 0x7f) * 50);
		else if (type == 0)
			len += sysfs_emit_at(buf, len, " fixed=%umV %umA",
					     ((pdo >> 10) & 0x3ff) * 50,
					     (pdo & 0x3ff) * 10);
		else
			len += sysfs_emit_at(buf, len, " type=%u", type);
		len += sysfs_emit_at(buf, len, "\n");
	}

out_unlock:
	mutex_unlock(&battmgr->lock);
	return ret ? ret : len;
}
static DEVICE_ATTR_RO(oneplus_pps_capabilities);

static int qcom_battmgr_request_property(struct qcom_battmgr *battmgr,
					 int opcode, int property, u32 value);

static bool qcom_battmgr_oneplus_has_fixed_5v(struct qcom_battmgr *battmgr)
{
	u32 data_size = le32_to_cpu(battmgr->oneplus_pps.data_size);
	u32 pdo_count;
	int i;

	pdo_count = min_t(u32, data_size / sizeof(__le32) - 2,
			  BATTMGR_ONEPLUS_PPS_PDO_MAX);
	for (i = 0; i < pdo_count; i++) {
		u32 pdo = le32_to_cpu(battmgr->oneplus_pps.data[i + 2]);

		if ((pdo >> 30) == 0 &&
		    ((pdo >> 10) & 0x3ff) * 50 == BATTMGR_ONEPLUS_FIXED_5V_MV)
			return true;
	}

	return false;
}

static bool qcom_battmgr_oneplus_has_pps(struct qcom_battmgr *battmgr,
					 u32 voltage_mv, u32 current_ma)
{
	u32 data_size = le32_to_cpu(battmgr->oneplus_pps.data_size);
	u32 pdo_count;
	int i;

	pdo_count = min_t(u32, data_size / sizeof(__le32) - 2,
			  BATTMGR_ONEPLUS_PPS_PDO_MAX);
	for (i = 0; i < pdo_count; i++) {
		u32 pdo = le32_to_cpu(battmgr->oneplus_pps.data[i + 2]);
		u32 min_mv;
		u32 max_mv;
		u32 max_ma;

		/* Only the SPR programmable-power-supply APDO is supported. */
		if ((pdo >> 30) != 3 || ((pdo >> 28) & 0x3) != 0)
			continue;

		min_mv = ((pdo >> 8) & 0xff) * 100;
		max_mv = ((pdo >> 17) & 0xff) * 100;
		max_ma = (pdo & 0x7f) * 50;
		if (voltage_mv >= min_mv && voltage_mv <= max_mv &&
		    current_ma <= max_ma)
			return true;
	}

	return false;
}

static bool qcom_battmgr_oneplus_has_pps_range(struct qcom_battmgr *battmgr,
					       u32 min_voltage_mv,
					       u32 max_voltage_mv,
					       u32 current_ma)
{
	u32 data_size = le32_to_cpu(battmgr->oneplus_pps.data_size);
	u32 pdo_count;
	int i;

	pdo_count = min_t(u32, data_size / sizeof(__le32) - 2,
			  BATTMGR_ONEPLUS_PPS_PDO_MAX);
	for (i = 0; i < pdo_count; i++) {
		u32 pdo = le32_to_cpu(battmgr->oneplus_pps.data[i + 2]);
		u32 min_mv;
		u32 max_mv;
		u32 max_ma;

		if ((pdo >> 30) != 3 || ((pdo >> 28) & 0x3) != 0)
			continue;

		min_mv = ((pdo >> 8) & 0xff) * 100;
		max_mv = ((pdo >> 17) & 0xff) * 100;
		max_ma = (pdo & 0x7f) * 50;
		if (min_voltage_mv >= min_mv && max_voltage_mv <= max_mv &&
		    current_ma <= max_ma)
			return true;
	}

	return false;
}

static int qcom_battmgr_oneplus_property_set_locked(struct qcom_battmgr *battmgr,
						     u32 opcode, u32 property,
						     u32 value)
{
	int ret;

	if (!battmgr->service_up)
		return -ENOTCONN;

	battmgr->oneplus_ext_pending = true;
	battmgr->oneplus_ext_opcode = opcode;
	battmgr->oneplus_ext_property = property;

	ret = qcom_battmgr_request_property(battmgr, opcode, property, value);

	battmgr->oneplus_ext_pending = false;
	battmgr->oneplus_ext_opcode = 0;
	battmgr->oneplus_ext_property = 0;

	return ret;
}

static int qcom_battmgr_oneplus_restore_buck_locked(
	struct qcom_battmgr *battmgr)
{
	int ret;

	ret = qcom_battmgr_oneplus_property_set_locked(
		battmgr, BATTMGR_USB_PROPERTY_SET, USB_INPUT_CURR_LIMIT,
		BATTMGR_ONEPLUS_USB_ICL_UNSUSPEND_UA);
	battmgr->oneplus_pps_buck_restore_rc = ret;
	if (!ret)
		battmgr->oneplus_pps_buck_suspended = false;

	return ret;
}

static int qcom_battmgr_oneplus_fixed_5v_locked(struct qcom_battmgr *battmgr)
{
	int ret;

	ret = qcom_battmgr_oneplus_property_set_locked(
		battmgr, BATTMGR_BAT_PROPERTY_SET,
		BATTMGR_ONEPLUS_BATT_SET_PDO, BATTMGR_ONEPLUS_FIXED_5V_MV);
	if (ret)
		return ret;
	battmgr->oneplus_pps_source_request_mv =
		BATTMGR_ONEPLUS_FIXED_5V_MV;

	/*
	 * The vendor PPS exit path first leaves PPS mode, then removes PPS_VOTER
	 * from the wired-input-suspend votable.  Its SM8650 ADSP backend maps that
	 * removal to USB_INPUT_CURR_LIMIT = U32_MAX.  Always send the restore here
	 * so a new test can recover a suspend left by an interrupted userspace run.
	 */
	return qcom_battmgr_oneplus_restore_buck_locked(battmgr);
}

static int __maybe_unused qcom_battmgr_oneplus_suspend_buck_locked(
	struct qcom_battmgr *battmgr)
{
	int ret;

	/* Match PPS_VOTER in the vendor PPS entry path. */
	ret = qcom_battmgr_oneplus_property_set_locked(
		battmgr, BATTMGR_USB_PROPERTY_SET, USB_INPUT_CURR_LIMIT,
		BATTMGR_ONEPLUS_USB_ICL_SUSPEND_UA);
	battmgr->oneplus_pps_buck_suspend_rc = ret;
	if (!ret)
		battmgr->oneplus_pps_buck_suspended = true;

	return ret;
}

static int qcom_battmgr_oneplus_set_pps_locked(struct qcom_battmgr *battmgr,
					       u32 voltage_mv, u32 current_ma)
{
	int ret;

	/* Match the vendor firmware ABI: program voltage before current. */
	ret = qcom_battmgr_oneplus_property_set_locked(
		battmgr, BATTMGR_USB_PROPERTY_SET,
		BATTMGR_ONEPLUS_USB_SET_PPS_VOLT, voltage_mv);
	if (ret)
		return ret;

	return qcom_battmgr_oneplus_property_set_locked(
		battmgr, BATTMGR_USB_PROPERTY_SET,
		BATTMGR_ONEPLUS_USB_SET_PPS_CURR, current_ma);
}

static int qcom_battmgr_oneplus_read_cp_adc(struct power_supply *cp_psy,
					    bool use_vout,
					    u32 *vin_uv, u32 *output_uv)
{
	union power_supply_propval val;
	int ret;

	ret = power_supply_get_property(cp_psy,
					POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (ret)
		return ret;
	if (val.intval <= 0)
		return -EAGAIN;
	*vin_uv = val.intval;

	ret = power_supply_get_property(cp_psy, use_vout ?
					POWER_SUPPLY_PROP_VOLTAGE_AVG :
					POWER_SUPPLY_PROP_VOLTAGE_OCV, &val);
	if (ret)
		return ret;
	if (val.intval <= 0)
		return -EAGAIN;
	*output_uv = val.intval;

	return 0;
}

static int qcom_battmgr_oneplus_read_cp_ibus(struct power_supply *cp_psy,
					     u32 *ibus_ua)
{
	union power_supply_propval val;
	int ret;

	ret = power_supply_get_property(cp_psy,
					POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	if (ret)
		return ret;
	if (val.intval < 0)
		return -EAGAIN;
	*ibus_ua = val.intval;

	return 0;
}

static int qcom_battmgr_oneplus_read_cp_output(
	struct power_supply *cp_psy, bool use_vout, u32 *output_uv)
{
	union power_supply_propval val;
	unsigned int attempt;
	int ret;

	/*
	 * A single I2C/ADC glitch (or one torn conversion) must not abort an
	 * otherwise healthy run.  Re-read a bounded number of times; a
	 * persistent error or a stuck zero still fails.
	 */
	for (attempt = 0; ; attempt++) {
		ret = power_supply_get_property(cp_psy, use_vout ?
						POWER_SUPPLY_PROP_VOLTAGE_AVG :
						POWER_SUPPLY_PROP_VOLTAGE_OCV,
						&val);
		if (!ret && val.intval > 0) {
			*output_uv = val.intval;
			return 0;
		}
		if (attempt >= 2)
			break;
		usleep_range(10000, 20000);
	}

	if (ret)
		return ret;
	return -EAGAIN;
}

static int qcom_battmgr_oneplus_set_cp_prepared(struct power_supply *cp_psy,
						 bool prepared)
{
	union power_supply_propval val = { .intval = prepared };
	int ret;

	ret = power_supply_set_property(cp_psy, POWER_SUPPLY_PROP_CALIBRATE,
					&val);
	if (ret)
		return ret;

	val.intval = !prepared;
	ret = power_supply_get_property(cp_psy, POWER_SUPPLY_PROP_CALIBRATE,
					&val);
	if (ret)
		return ret;

	return !!val.intval == prepared ? 0 : -EIO;
}

static int qcom_battmgr_oneplus_set_cp_start_target(
	struct power_supply *cp_psy, u32 target_uv)
{
	union power_supply_propval val = { .intval = target_uv };
	int ret;

	ret = power_supply_set_property(cp_psy,
					POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
					&val);
	if (ret)
		return ret;

	val.intval = -1;
	ret = power_supply_get_property(cp_psy,
					POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
					&val);
	if (ret)
		return ret;

	return val.intval == target_uv ? 0 : -EIO;
}

static int qcom_battmgr_oneplus_run_cp(struct power_supply *cp_psy,
				       u32 run_ms)
{
	union power_supply_propval val = { .intval = run_ms };

	return power_supply_set_property(cp_psy, POWER_SUPPLY_PROP_CALIBRATE,
					 &val);
}

static int qcom_battmgr_oneplus_cp_run_thread(void *data)
{
	struct qcom_battmgr_oneplus_cp_run_ctx *ctx = data;

	ctx->result = qcom_battmgr_oneplus_run_cp(ctx->cp_psy, ctx->run_ms);
	complete(&ctx->done);

	return 0;
}

static int qcom_battmgr_oneplus_require_cp_active(
	struct power_supply *cp_psy, u32 *ibus_ua)
{
	union power_supply_propval val;
	int ret;

	ret = power_supply_get_property(cp_psy, POWER_SUPPLY_PROP_ONLINE, &val);
	if (ret)
		return ret;
	if (!val.intval)
		return -EIO;

	ret = power_supply_get_property(cp_psy, POWER_SUPPLY_PROP_STATUS, &val);
	if (ret)
		return ret;
	if (val.intval != POWER_SUPPLY_STATUS_CHARGING)
		return -EIO;

	ret = qcom_battmgr_oneplus_read_cp_ibus(cp_psy, ibus_ua);
	if (ret)
		return ret;

	return *ibus_ua ? 0 : -ERANGE;
}

static int qcom_battmgr_oneplus_require_cp_active_bounded(
	struct power_supply *cp_psy, u32 *ibus_ua, u32 ibus_max_ua,
	u32 *range_rechecks, u32 *range_recoveries)
{
	unsigned int attempt;
	int ret;

	ret = qcom_battmgr_oneplus_require_cp_active(cp_psy, ibus_ua);
	if (ret && ret != -ERANGE && ret != -EIO)
		return ret;
	if (!ret && *ibus_ua <= ibus_max_ua)
		return 0;

	/*
	 * Mirror the physical driver's numeric-window confirmation.  A stopped
	 * pump, status fault or I/O error is re-sampled twice more before it
	 * becomes fatal: giulia's SC8547D briefly drops its CP_SWITCHING status
	 * during a PPS step (and the odd I2C/ADC read glitches), which must not
	 * abort an otherwise healthy session.  A genuinely stopped pump fails
	 * every recheck and still ends the run.
	 */
	for (attempt = 0;
	     attempt < BATTMGR_ONEPLUS_CP_DUAL_IBUS_RECHECKS; attempt++) {
		usleep_range(BATTMGR_ONEPLUS_CP_DUAL_IBUS_RECHECK_MIN_US,
			      BATTMGR_ONEPLUS_CP_DUAL_IBUS_RECHECK_MAX_US);
		(*range_rechecks)++;
		ret = qcom_battmgr_oneplus_require_cp_active(cp_psy, ibus_ua);
		if (ret && ret != -ERANGE && ret != -EIO)
			return ret;
		if (!ret && *ibus_ua <= ibus_max_ua) {
			(*range_recoveries)++;
			return 0;
		}
	}

	return ret ? ret : -ERANGE;
}

/*
 * The dual coordinator starts each pump on its own kthread; the physical
 * driver then runs its preflight/start sequence before REG07 is enabled.
 * giulia's start latency varies and can exceed the fixed settle time, so wait
 * for the pump to actually come up instead of sampling once.  A genuinely
 * failed pump keeps failing every retry and still ends the wait.
 */
static int qcom_battmgr_oneplus_wait_cp_active(
	struct power_supply *cp_psy, u32 *ibus_ua, u32 ibus_max_ua,
	unsigned int wait_ms, unsigned int retries,
	u32 *range_rechecks, u32 *range_recoveries)
{
	unsigned int attempt;
	int ret;

	for (attempt = 0; ; attempt++) {
		ret = qcom_battmgr_oneplus_require_cp_active_bounded(
			cp_psy, ibus_ua, ibus_max_ua,
			range_rechecks, range_recoveries);
		if (!ret)
			return 0;
		if (ret != -EIO && ret != -ERANGE)
			return ret;
		if (attempt >= retries)
			return ret;
		msleep(wait_ms);
	}
}

static int qcom_battmgr_oneplus_cancel_cp(struct power_supply *cp_psy)
{
	union power_supply_propval val = {
		.intval = BATTMGR_ONEPLUS_CP_POLICY_CANCEL,
	};

	return power_supply_set_property(cp_psy, POWER_SUPPLY_PROP_CALIBRATE,
					 &val);
}

static int qcom_battmgr_oneplus_kick_cp_watchdog(
	struct power_supply *cp_psy)
{
	union power_supply_propval val;

	if (!cp_psy)
		return 0;

	/* The board's downstream SC8547/SC8547D watchdog-reset callback reads REG07. */
	return power_supply_get_property(cp_psy, POWER_SUPPLY_PROP_ONLINE, &val);
}

static int qcom_battmgr_oneplus_kick_cp_watchdogs(
	struct power_supply *first, struct power_supply *second)
{
	int ret;

	ret = qcom_battmgr_oneplus_kick_cp_watchdog(first);
	if (ret)
		return ret;

	return qcom_battmgr_oneplus_kick_cp_watchdog(second);
}

static int qcom_battmgr_oneplus_require_cp_off(const char *name)
{
	struct power_supply *cp_psy;
	union power_supply_propval val;
	int ret;

	cp_psy = power_supply_get_by_name(name);
	if (!cp_psy)
		return -ENODEV;

	ret = power_supply_get_property(cp_psy, POWER_SUPPLY_PROP_ONLINE, &val);
	if (ret)
		goto out_put;
	if (val.intval) {
		ret = -EBUSY;
		goto out_put;
	}

	ret = power_supply_get_property(cp_psy, POWER_SUPPLY_PROP_STATUS, &val);
	if (!ret && val.intval == POWER_SUPPLY_STATUS_CHARGING)
		ret = -EBUSY;

out_put:
	power_supply_put(cp_psy);
	return ret;
}

static u32 qcom_battmgr_oneplus_feedback_step_mv(u32 error_uv)
{
	if (error_uv >= 2000000)
		return 2000;
	if (error_uv >= 1000000)
		return 1000;
	if (error_uv >= 500000)
		return 500;
	if (error_uv >= 200000)
		return 200;

	return 100;
}

static void qcom_battmgr_oneplus_pps_fallback_locked(
	struct qcom_battmgr *battmgr, int request_ret);

static int qcom_battmgr_oneplus_ramp_pps_locked(
	struct qcom_battmgr *battmgr, u32 max_voltage_mv,
	u32 current_ma, struct power_supply *watchdog_psy,
	struct power_supply *second_watchdog_psy)
{
	u32 coarse_max_mv = min(max_voltage_mv,
				BATTMGR_ONEPLUS_RAMP_PPS_MAX_MV);
	u32 last_voltage_mv = 0;
	u32 voltage_mv;
	int ret;

	for (voltage_mv = BATTMGR_ONEPLUS_TEST_PPS_MV;
	     voltage_mv <= coarse_max_mv;
	     voltage_mv += BATTMGR_ONEPLUS_RAMP_PPS_STEP_MV) {
		ret = qcom_battmgr_oneplus_set_pps_locked(
			battmgr, voltage_mv, current_ma);
		if (ret)
			goto fallback;
		last_voltage_mv = voltage_mv;
		msleep(BATTMGR_ONEPLUS_RAMP_DELAY_MS);
		ret = qcom_battmgr_oneplus_kick_cp_watchdogs(
			watchdog_psy, second_watchdog_psy);
		if (ret)
			goto fallback;
	}

	for (voltage_mv = last_voltage_mv +
			  BATTMGR_ONEPLUS_RAMP_PPS_FINE_STEP_MV;
	     voltage_mv <= max_voltage_mv;
	     voltage_mv += BATTMGR_ONEPLUS_RAMP_PPS_FINE_STEP_MV) {
		ret = qcom_battmgr_oneplus_set_pps_locked(
			battmgr, voltage_mv, current_ma);
		if (ret)
			goto fallback;
		last_voltage_mv = voltage_mv;
		msleep(BATTMGR_ONEPLUS_RAMP_DELAY_MS);
		ret = qcom_battmgr_oneplus_kick_cp_watchdogs(
			watchdog_psy, second_watchdog_psy);
		if (ret)
			goto fallback;
	}

	/*
	 * A dynamic 2:1 target commonly ends in 50 mV. Do not silently stop at
	 * the preceding 100-mV ramp point: the downstream policy can issue an
	 * exact millivolt request, and the PPS transport performs any required
	 * source-step quantization.
	 */
	if (last_voltage_mv < max_voltage_mv) {
		ret = qcom_battmgr_oneplus_set_pps_locked(
			battmgr, max_voltage_mv, current_ma);
		if (ret)
			goto fallback;
		msleep(BATTMGR_ONEPLUS_RAMP_DELAY_MS);
		ret = qcom_battmgr_oneplus_kick_cp_watchdogs(
			watchdog_psy, second_watchdog_psy);
		if (ret)
			goto fallback;
	}

	return 0;

fallback:
	qcom_battmgr_oneplus_pps_fallback_locked(battmgr, ret);
	return ret;
}

static void qcom_battmgr_oneplus_pps_fallback_locked(
	struct qcom_battmgr *battmgr, int request_ret)
{
	int fallback_ret;

	fallback_ret = qcom_battmgr_oneplus_fixed_5v_locked(battmgr);
	if (fallback_ret)
		dev_err(battmgr->dev,
			"PPS request and fixed-5V fallback failed: %d, %d\n",
			request_ret, fallback_ret);
}

static ssize_t oneplus_fixed_5v_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	int ret;

	if (!sysfs_streq(buf, "5000"))
		return -EINVAL;

	mutex_lock(&battmgr->lock);
	if (battmgr->oneplus_policy_running ||
	    battmgr->oneplus_pps_5m_running ||
	    battmgr->oneplus_secondary_running ||
	    battmgr->oneplus_dual_running) {
		ret = -EBUSY;
		goto out_unlock;
	}

	ret = qcom_battmgr_request_property(battmgr, BATTMGR_USB_PROPERTY_GET,
					   USB_ONLINE, 0);
	if (ret)
		goto out_unlock;
	if (!battmgr->usb.online) {
		ret = -ENODEV;
		goto out_unlock;
	}

	ret = qcom_battmgr_oneplus_pps_request(battmgr);
	if (ret)
		goto out_unlock;
	if (!qcom_battmgr_oneplus_has_fixed_5v(battmgr)) {
		ret = -EOPNOTSUPP;
		goto out_unlock;
	}

	ret = qcom_battmgr_oneplus_fixed_5v_locked(battmgr);

out_unlock:
	mutex_unlock(&battmgr->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_WO(oneplus_fixed_5v);

static ssize_t oneplus_pps_5v5_1a_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	mutex_lock(&battmgr->lock);
	if (battmgr->oneplus_policy_running ||
	    battmgr->oneplus_pps_5m_running ||
	    battmgr->oneplus_secondary_running ||
	    battmgr->oneplus_dual_running) {
		ret = -EBUSY;
		goto out_unlock;
	}

	ret = qcom_battmgr_request_property(battmgr, BATTMGR_USB_PROPERTY_GET,
					   USB_ONLINE, 0);
	if (ret)
		goto out_unlock;
	if (!battmgr->usb.online) {
		ret = -ENODEV;
		goto out_unlock;
	}

	ret = qcom_battmgr_oneplus_pps_request(battmgr);
	if (ret)
		goto out_unlock;
	if (!qcom_battmgr_oneplus_has_fixed_5v(battmgr) ||
	    !qcom_battmgr_oneplus_has_pps(battmgr,
					 BATTMGR_ONEPLUS_TEST_PPS_MV,
					 BATTMGR_ONEPLUS_TEST_PPS_MA)) {
		ret = -EOPNOTSUPP;
		goto out_unlock;
	}

	ret = qcom_battmgr_oneplus_set_pps_locked(
		battmgr, BATTMGR_ONEPLUS_TEST_PPS_MV,
		BATTMGR_ONEPLUS_TEST_PPS_MA);
	if (ret)
		qcom_battmgr_oneplus_pps_fallback_locked(battmgr, ret);

out_unlock:
	mutex_unlock(&battmgr->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_WO(oneplus_pps_5v5_1a);

static ssize_t oneplus_pps_ramp_9v_1a_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	mutex_lock(&battmgr->lock);
	if (battmgr->oneplus_policy_running ||
	    battmgr->oneplus_pps_5m_running ||
	    battmgr->oneplus_secondary_running ||
	    battmgr->oneplus_dual_running) {
		ret = -EBUSY;
		goto out_unlock;
	}

	ret = qcom_battmgr_request_property(battmgr, BATTMGR_USB_PROPERTY_GET,
					   USB_ONLINE, 0);
	if (ret)
		goto out_unlock;
	if (!battmgr->usb.online) {
		ret = -ENODEV;
		goto out_unlock;
	}

	ret = qcom_battmgr_oneplus_pps_request(battmgr);
	if (ret)
		goto out_unlock;
	if (!qcom_battmgr_oneplus_has_fixed_5v(battmgr) ||
	    !qcom_battmgr_oneplus_has_pps_range(
		    battmgr, BATTMGR_ONEPLUS_TEST_PPS_MV,
		    BATTMGR_ONEPLUS_RAMP_PPS_MAX_MV,
		    BATTMGR_ONEPLUS_TEST_PPS_MA)) {
		ret = -EOPNOTSUPP;
		goto out_unlock;
	}

	/*
	 * Keep both charge pumps off.  The vendor policy also starts at 5.5 V
	 * and raises VBUS in bounded steps before it attempts to start a pump.
	 * This development-only endpoint stops at 9 V and 1 A and never starts
	 * either pump; userspace must explicitly request the fixed-5-V fallback
	 * after collecting source/ADC telemetry.
	 */
	ret = qcom_battmgr_oneplus_ramp_pps_locked(
		battmgr, BATTMGR_ONEPLUS_RAMP_PPS_MAX_MV,
		BATTMGR_ONEPLUS_TEST_PPS_MA, NULL, NULL);

out_unlock:
	mutex_unlock(&battmgr->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_WO(oneplus_pps_ramp_9v_1a);

static ssize_t oneplus_pps_ramp_9v2_1a_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	mutex_lock(&battmgr->lock);
	if (battmgr->oneplus_policy_running ||
	    battmgr->oneplus_pps_5m_running ||
	    battmgr->oneplus_secondary_running ||
	    battmgr->oneplus_dual_running) {
		ret = -EBUSY;
		goto out_unlock;
	}

	ret = qcom_battmgr_request_property(battmgr, BATTMGR_USB_PROPERTY_GET,
					   USB_ONLINE, 0);
	if (ret)
		goto out_unlock;
	if (!battmgr->usb.online) {
		ret = -ENODEV;
		goto out_unlock;
	}

	ret = qcom_battmgr_oneplus_pps_request(battmgr);
	if (ret)
		goto out_unlock;
	if (!qcom_battmgr_oneplus_has_fixed_5v(battmgr) ||
	    !qcom_battmgr_oneplus_has_pps_range(
		    battmgr, BATTMGR_ONEPLUS_TEST_PPS_MV,
		    BATTMGR_ONEPLUS_RAMP_PPS_9V2_MAX_MV,
		    BATTMGR_ONEPLUS_TEST_PPS_MA)) {
		ret = -EOPNOTSUPP;
		goto out_unlock;
	}

	/*
	 * Preserve the validated 5.5-V-to-9-V coarse ramp, then follow the
	 * vendor policy's near-target 100-mV steps to 9.2 V.  This endpoint
	 * never enables a charge pump and remains capped at 1 A.
	 */
	ret = qcom_battmgr_oneplus_ramp_pps_locked(
		battmgr, BATTMGR_ONEPLUS_RAMP_PPS_9V2_MAX_MV,
		BATTMGR_ONEPLUS_TEST_PPS_MA, NULL, NULL);

out_unlock:
	mutex_unlock(&battmgr->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_WO(oneplus_pps_ramp_9v2_1a);

static const char *qcom_battmgr_oneplus_2to1_stage_name(
	enum qcom_battmgr_oneplus_2to1_stage stage)
{
	switch (stage) {
	case BATTMGR_ONEPLUS_2TO1_IDLE:
		return "idle";
	case BATTMGR_ONEPLUS_2TO1_USB_ONLINE:
		return "usb_online";
	case BATTMGR_ONEPLUS_2TO1_VBAT:
		return "vbat";
	case BATTMGR_ONEPLUS_2TO1_CAPABILITIES:
		return "capabilities";
	case BATTMGR_ONEPLUS_2TO1_CP_PREPARE:
		return "cp_prepare";
	case BATTMGR_ONEPLUS_2TO1_BUCK_SUSPEND:
		return "buck_suspend";
	case BATTMGR_ONEPLUS_2TO1_BUCK_ACTIVE:
		return "buck_active";
	case BATTMGR_ONEPLUS_2TO1_RAMP:
		return "ramp";
	case BATTMGR_ONEPLUS_2TO1_FEEDBACK:
		return "feedback";
	case BATTMGR_ONEPLUS_2TO1_COMPLETE:
		return "complete";
	default:
		return "unknown";
	}
}

static ssize_t oneplus_pps_ramp_2to1_1a_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	enum qcom_battmgr_oneplus_2to1_stage stage;
	u32 pump_target_mv;
	u32 source_request_mv;
	u32 current_ma;
	u32 feedback_request_mv[BATTMGR_ONEPLUS_CP_FEEDBACK_RECORDS];
	u32 feedback_vin_uv[BATTMGR_ONEPLUS_CP_FEEDBACK_RECORDS];
	u32 feedback_attempts;
	u32 feedback_stable_samples;
	u32 cp_vbat_uv;
	u32 cp_vin_uv;
	u32 cp_adc_attempts;
	bool cp_output_uses_vout;
	u32 companion_vin_uv;
	u32 companion_output_uv;
	u32 companion_adc_attempts;
	u32 attempts;
	u32 vbat_uv;
	bool cp_prepared;
	bool buck_suspended;
	bool from_cache;
	int cp_prepare_rc;
	int companion_prepare_rc;
	int cp_restore_rc;
	int buck_restore_rc;
	int buck_suspend_rc;
	int result;
	int len;
	unsigned int i;

	mutex_lock(&battmgr->lock);
	result = battmgr->oneplus_pps_2to1_result;
	stage = battmgr->oneplus_pps_2to1_stage;
	vbat_uv = battmgr->oneplus_pps_vbat_uv;
	attempts = battmgr->oneplus_pps_vbat_attempts;
	from_cache = battmgr->oneplus_pps_vbat_from_cache;
	pump_target_mv = battmgr->oneplus_pps_pump_target_mv;
	source_request_mv = battmgr->oneplus_pps_source_request_mv;
	current_ma = battmgr->oneplus_pps_current_ma;
	cp_vbat_uv = battmgr->oneplus_pps_cp_vbat_uv;
	cp_vin_uv = battmgr->oneplus_pps_cp_vin_uv;
	cp_adc_attempts = battmgr->oneplus_pps_cp_adc_attempts;
	cp_output_uses_vout = battmgr->oneplus_pps_cp_output_uses_vout;
	companion_vin_uv = battmgr->oneplus_pps_companion_vin_uv;
	companion_output_uv = battmgr->oneplus_pps_companion_output_uv;
	companion_adc_attempts =
		battmgr->oneplus_pps_companion_adc_attempts;
	companion_prepare_rc = battmgr->oneplus_pps_companion_prepare_rc;
	feedback_attempts = min_t(u32,
		battmgr->oneplus_pps_feedback_attempts,
		BATTMGR_ONEPLUS_CP_FEEDBACK_RECORDS);
	feedback_stable_samples = battmgr->oneplus_pps_feedback_stable_samples;
	memcpy(feedback_request_mv, battmgr->oneplus_pps_feedback_request_mv,
	       sizeof(feedback_request_mv));
	memcpy(feedback_vin_uv, battmgr->oneplus_pps_feedback_vin_uv,
	       sizeof(feedback_vin_uv));
	cp_prepared = battmgr->oneplus_pps_cp_prepared;
	cp_prepare_rc = battmgr->oneplus_pps_cp_prepare_rc;
	cp_restore_rc = battmgr->oneplus_pps_cp_restore_rc;
	buck_suspended = battmgr->oneplus_pps_buck_suspended;
	buck_suspend_rc = battmgr->oneplus_pps_buck_suspend_rc;
	buck_restore_rc = battmgr->oneplus_pps_buck_restore_rc;
	mutex_unlock(&battmgr->lock);

	len = sysfs_emit(buf,
		"rc=%d stage=%s vbat_uv=%u vbat_source=%s vbat_attempts=%u cp_output_source=%s cp_output_uv=%u pump_target_mv=%u source_request_mv=%u current_ma=%u cp_prepared=%u cp_prepare_rc=%d cp_restore_rc=%d buck_suspended=%u buck_suspend_rc=%d buck_restore_rc=%d feedback_attempts=%u feedback_stable_samples=%u\n",
		result, qcom_battmgr_oneplus_2to1_stage_name(stage),
		vbat_uv, from_cache ? "cache" : "live", attempts,
		cp_output_uses_vout ? "vout" : "vbat", cp_vbat_uv,
		pump_target_mv, source_request_mv,
		current_ma, cp_prepared, cp_prepare_rc,
		cp_restore_rc, buck_suspended,
		buck_suspend_rc, buck_restore_rc, feedback_attempts,
		feedback_stable_samples);
	len += sysfs_emit_at(buf, len,
		"adc=selected_vin_uv:%u/selected_output_uv:%u/selected_attempts:%u/companion_vin_uv:%u/companion_output_uv:%u/companion_attempts:%u/companion_prepare_rc:%d\n",
		cp_vin_uv, cp_vbat_uv, cp_adc_attempts, companion_vin_uv,
		companion_output_uv, companion_adc_attempts,
		companion_prepare_rc);
	len += sysfs_emit_at(buf, len, "feedback_trace=");
	for (i = 0; i < feedback_attempts; i++)
		len += sysfs_emit_at(buf, len, "%s%u/%u",
				     i ? "," : "",
				     feedback_request_mv[i], feedback_vin_uv[i]);
	len += sysfs_emit_at(buf, len, "\n");

	return len;
}

static int qcom_battmgr_get_batt_voltage_uv(struct qcom_battmgr *battmgr);

/*
 * Real battery voltage for the Oplus PPS paths.  The ADSP battery service
 * reports a non-zero placeholder on these devices, so the AP-side gauge is the
 * truth.  This performs an I2C read, so it must only be called from a
 * sleepable context (the dual/policy kthreads), never from the pmic_glink RX
 * callback.
 */
static int qcom_battmgr_oneplus_vbat_uv(struct qcom_battmgr *battmgr)
{
	if (battmgr->variant == QCOM_BATTMGR_OPLUS_SM8650) {
		int uv = qcom_battmgr_get_batt_voltage_uv(battmgr);

		if (uv > 0)
			return uv;
	}

	return battmgr->status.voltage_now;
}

static int qcom_battmgr_oneplus_ramp_2to1_run(
	struct qcom_battmgr *battmgr, const char *cp_name, bool policy_owned,
	bool run_owned, bool secondary_owned, bool dual_owned, u32 current_ma)
{
	struct power_supply *cp_psy = NULL;
	struct power_supply *companion_psy = NULL;
	u32 pump_target_uv;
	u32 companion_output_uv = 0;
	u32 companion_vin_uv = 0;
	u32 cp_output_uv = 0;
	u32 cp_vin_uv = 0;
	u32 source_request_mv;
	u32 next_request_mv;
	u32 feedback_step_mv;
	u32 diagnostic_target_uv;
	u32 error_uv;
	u32 vbat_uv = 0;
	u32 stable_samples = 0;
	u32 stable_misses = 0;
	u32 stable_request_mv = 0;
	bool companion_prepared = false;
	unsigned int feedback_attempt_limit = dual_owned ?
		BATTMGR_ONEPLUS_CP_DUAL_FEEDBACK_ATTEMPTS :
		BATTMGR_ONEPLUS_CP_FEEDBACK_ATTEMPTS;
	unsigned int selected_adc_attempts;
	unsigned int attempt;
	int ret;

	mutex_lock(&battmgr->lock);
	if ((battmgr->oneplus_policy_running && !policy_owned) ||
	    (battmgr->oneplus_pps_5m_running && !run_owned) ||
	    (battmgr->oneplus_secondary_running && !secondary_owned) ||
	    (battmgr->oneplus_dual_running && !dual_owned)) {
		ret = -EBUSY;
		goto out_unlock;
	}
	battmgr->oneplus_pps_2to1_result = -EINPROGRESS;
	battmgr->oneplus_pps_2to1_stage = BATTMGR_ONEPLUS_2TO1_USB_ONLINE;
	battmgr->oneplus_pps_vbat_uv = 0;
	battmgr->oneplus_pps_vbat_attempts = 0;
	battmgr->oneplus_pps_vbat_from_cache = false;
	battmgr->oneplus_pps_pump_target_mv = 0;
	battmgr->oneplus_pps_source_request_mv = 0;
	battmgr->oneplus_pps_current_ma = current_ma;
	battmgr->oneplus_pps_cp_vbat_uv = 0;
	battmgr->oneplus_pps_cp_vin_uv = 0;
	battmgr->oneplus_pps_cp_adc_attempts = 0;
	battmgr->oneplus_pps_cp_output_uses_vout = secondary_owned;
	battmgr->oneplus_pps_companion_vin_uv = 0;
	battmgr->oneplus_pps_companion_output_uv = 0;
	battmgr->oneplus_pps_companion_adc_attempts = 0;
	battmgr->oneplus_pps_companion_prepare_rc = 0;
	battmgr->oneplus_pps_feedback_attempts = 0;
	battmgr->oneplus_pps_feedback_stable_samples = 0;
	memset(battmgr->oneplus_pps_feedback_request_mv, 0,
	       sizeof(battmgr->oneplus_pps_feedback_request_mv));
	memset(battmgr->oneplus_pps_feedback_vin_uv, 0,
	       sizeof(battmgr->oneplus_pps_feedback_vin_uv));
	battmgr->oneplus_pps_cp_prepared = false;
	battmgr->oneplus_pps_cp_prepare_rc = 0;
	battmgr->oneplus_pps_cp_restore_rc = 0;
	battmgr->oneplus_pps_buck_suspend_rc = 0;
	battmgr->oneplus_pps_buck_restore_rc = 0;

	ret = qcom_battmgr_request_property(battmgr, BATTMGR_USB_PROPERTY_GET,
					   USB_ONLINE, 0);
	if (ret)
		goto out_unlock;
	if (!battmgr->usb.online) {
		ret = -ENODEV;
		goto out_unlock;
	}

	battmgr->oneplus_pps_2to1_stage = BATTMGR_ONEPLUS_2TO1_VBAT;
	/*
	 * This firmware can acknowledge BATT_VOLT_NOW with a transient zero while
	 * its gauge cache is being refreshed. Allow up to 700 ms for a live value.
	 * If every response is zero, accept only a non-zero reading observed by this
	 * driver in the previous five seconds. The selected value still has to pass
	 * the strict charge-pump voltage window below.
	 */
	for (attempt = 0; attempt < BATTMGR_ONEPLUS_VBAT_ZERO_RETRIES;
	     attempt++) {
		ret = qcom_battmgr_request_property(
			battmgr, BATTMGR_BAT_PROPERTY_GET, BATT_VOLT_NOW, 0);
		if (ret)
			goto out_unlock;
		battmgr->oneplus_pps_vbat_attempts = attempt + 1;
		vbat_uv = qcom_battmgr_oneplus_vbat_uv(battmgr);
		if (vbat_uv)
			break;
		if (attempt + 1 < BATTMGR_ONEPLUS_VBAT_ZERO_RETRIES)
			msleep(BATTMGR_ONEPLUS_VBAT_RETRY_MS);
	}
	if (!vbat_uv) {
		u32 cached_vbat_uv = READ_ONCE(battmgr->oneplus_pps_vbat_cache_uv);
		unsigned long cached_at =
			READ_ONCE(battmgr->oneplus_pps_vbat_cache_at);

		if (cached_vbat_uv &&
		    time_before(jiffies, cached_at +
			msecs_to_jiffies(BATTMGR_ONEPLUS_VBAT_CACHE_MAX_AGE_MS))) {
			vbat_uv = cached_vbat_uv;
			battmgr->oneplus_pps_vbat_from_cache = true;
		} else {
			ret = -EAGAIN;
			goto out_unlock;
		}
	}
	battmgr->oneplus_pps_vbat_uv = vbat_uv;
	if (vbat_uv < BATTMGR_ONEPLUS_CP_2TO1_VBAT_MIN_UV ||
	    vbat_uv > BATTMGR_ONEPLUS_CP_2TO1_VBAT_MAX_UV) {
		ret = -ERANGE;
		goto out_unlock;
	}

	/*
	 * The downstream OnePlus device tree does not set
	 * oplus,lift_vbus_use_cpvout, so oplus_pps_charge_start() derives its
	 * 2:1 start target from GAUGE_ITEM_VOL_MAX.  BATT_VOLT_NOW is the matching
	 * PMIC-provided value here.  Keep using the primary pump ADC for the live
	 * VIN safety/feedback check, but do not substitute its lower local VBAT
	 * sample into the vendor target formula. The same formula is used for the
	 * explicitly selected physical pump during the independent role test.
	 */
	cp_psy = power_supply_get_by_name(cp_name);
	if (!cp_psy) {
		ret = -ENODEV;
		goto out_unlock;
	}

	/*
	 * Initialize the selected pump before its first local ADC read while the
	 * source is still in the checked low-voltage state. This is required for a
	 * cold dual-pump start as well as for the independent secondary path.
	 * On both caihong (SC8547A) and giulia (SC8547D) the secondary's local
	 * VBAT ADC is unconnected and its VOUT sense path is used instead.
	 */
	if (secondary_owned || dual_owned) {
		battmgr->oneplus_pps_2to1_stage =
			BATTMGR_ONEPLUS_2TO1_CP_PREPARE;
		ret = qcom_battmgr_oneplus_set_cp_prepared(cp_psy, false);
		battmgr->oneplus_pps_cp_prepare_rc = ret;
		if (ret)
			goto out_put;
	}
	if (dual_owned) {
		companion_psy = power_supply_get_by_name(
			BATTMGR_ONEPLUS_SECONDARY_CP_NAME);
		if (!companion_psy) {
			ret = -ENODEV;
			goto out_put;
		}
		battmgr->oneplus_pps_2to1_stage =
			BATTMGR_ONEPLUS_2TO1_CP_PREPARE;
		ret = qcom_battmgr_oneplus_set_cp_prepared(
			companion_psy, false);
		battmgr->oneplus_pps_companion_prepare_rc = ret;
		if (ret)
			goto out_put;
		for (attempt = 0;
		     attempt < BATTMGR_ONEPLUS_CP_ADC_READY_ATTEMPTS;
		     attempt++) {
			ret = qcom_battmgr_oneplus_read_cp_adc(
				companion_psy, true, &companion_vin_uv,
				&companion_output_uv);
			battmgr->oneplus_pps_companion_adc_attempts = attempt + 1;
			battmgr->oneplus_pps_companion_vin_uv = companion_vin_uv;
			battmgr->oneplus_pps_companion_output_uv =
				companion_output_uv;
			if (!ret &&
			    companion_output_uv >=
				BATTMGR_ONEPLUS_CP_2TO1_VBAT_MIN_UV &&
			    companion_output_uv <=
				BATTMGR_ONEPLUS_CP_2TO1_VBAT_MAX_UV)
				break;
			if (!ret)
				ret = -ERANGE;
			else if (ret != -EAGAIN)
				goto out_put;
			if (companion_output_uv >
			    BATTMGR_ONEPLUS_CP_2TO1_VBAT_MAX_UV)
				goto out_put;
			if (attempt + 1 ==
			    BATTMGR_ONEPLUS_CP_ADC_READY_ATTEMPTS)
				goto out_put;
			msleep(BATTMGR_ONEPLUS_CP_ADC_READY_DELAY_MS);
		}
	}

	/* A freshly initialized dual-path primary also needs ADC-ready retries. */
	selected_adc_attempts = secondary_owned || dual_owned ?
		BATTMGR_ONEPLUS_CP_ADC_READY_ATTEMPTS : 1;
	for (attempt = 0; attempt < selected_adc_attempts; attempt++) {
		ret = qcom_battmgr_oneplus_read_cp_adc(cp_psy, secondary_owned,
							&cp_vin_uv,
							&cp_output_uv);
		battmgr->oneplus_pps_cp_adc_attempts = attempt + 1;
		battmgr->oneplus_pps_cp_vin_uv = cp_vin_uv;
		if (!ret) {
			battmgr->oneplus_pps_cp_vbat_uv = cp_output_uv;
			if (cp_output_uv > BATTMGR_ONEPLUS_CP_2TO1_VBAT_MAX_UV) {
				ret = -ERANGE;
				goto out_put;
			}
			if (cp_output_uv >= BATTMGR_ONEPLUS_CP_2TO1_VBAT_MIN_UV)
				break;
			ret = -ERANGE;
		} else if (ret != -EAGAIN) {
			goto out_put;
		}

		if (attempt + 1 == selected_adc_attempts)
			goto out_put;
		msleep(BATTMGR_ONEPLUS_CP_ADC_READY_DELAY_MS);
	}
	if (cp_output_uv < BATTMGR_ONEPLUS_CP_2TO1_VBAT_MIN_UV ||
	    cp_output_uv > BATTMGR_ONEPLUS_CP_2TO1_VBAT_MAX_UV) {
		ret = -ERANGE;
		goto out_put;
	}

	pump_target_uv = vbat_uv * 2;
	pump_target_uv = pump_target_uv /
		BATTMGR_ONEPLUS_CP_2TO1_TARGET_STEP_UV *
		BATTMGR_ONEPLUS_CP_2TO1_TARGET_STEP_UV;
	pump_target_uv += BATTMGR_ONEPLUS_CP_2TO1_START_DELTA_UV;
	source_request_mv = pump_target_uv / 1000;
	battmgr->oneplus_pps_pump_target_mv = pump_target_uv / 1000;
	battmgr->oneplus_pps_source_request_mv = source_request_mv;

	battmgr->oneplus_pps_2to1_stage = BATTMGR_ONEPLUS_2TO1_CAPABILITIES;
	ret = qcom_battmgr_oneplus_pps_request(battmgr);
	if (ret)
		goto out_put;
	if (!qcom_battmgr_oneplus_has_fixed_5v(battmgr) ||
	    !qcom_battmgr_oneplus_has_pps_range(
		    battmgr, BATTMGR_ONEPLUS_TEST_PPS_MV,
		    BATTMGR_ONEPLUS_CP_2TO1_SOURCE_MAX_MV,
		    current_ma)) {
		ret = -EOPNOTSUPP;
		goto out_put;
	}

	/*
	 * Configure the pump while the source is still at its low PPS entry
	 * voltage.  Keep the normal buck path active for this bounded diagnostic:
	 * the historical Stage 4B pulse entered switching with that topology,
	 * whereas every later test that sent USB_INPUT_CURR_LIMIT = 0 was rejected
	 * immediately by the SC8547 hardware start interlock.  This is a
	 * single-variable comparison; current_ma is independently bounded by the
	 * caller and the advertised APDO capability check above, while
	 * all pump-side preflight limits remain enforced.
	 */
	battmgr->oneplus_pps_2to1_stage = BATTMGR_ONEPLUS_2TO1_CP_PREPARE;
	ret = qcom_battmgr_oneplus_set_cp_start_target(cp_psy,
						       pump_target_uv);
	if (ret)
		goto out_put;
	ret = qcom_battmgr_oneplus_set_cp_prepared(cp_psy, true);
	battmgr->oneplus_pps_cp_prepare_rc = ret;
	if (ret)
		goto target_fallback;
	battmgr->oneplus_pps_cp_prepared = true;
	if (dual_owned) {
		ret = qcom_battmgr_oneplus_set_cp_start_target(
			companion_psy, pump_target_uv);
		if (ret)
			goto prepared_fallback;
		ret = qcom_battmgr_oneplus_set_cp_prepared(
			companion_psy, true);
		battmgr->oneplus_pps_companion_prepare_rc = ret;
		if (ret)
			goto prepared_fallback;
		companion_prepared = true;
	}

	battmgr->oneplus_pps_2to1_stage = BATTMGR_ONEPLUS_2TO1_BUCK_ACTIVE;
	battmgr->oneplus_pps_buck_suspended = false;

	battmgr->oneplus_pps_2to1_stage = BATTMGR_ONEPLUS_2TO1_RAMP;
	ret = qcom_battmgr_oneplus_ramp_pps_locked(battmgr,
						   source_request_mv,
						   current_ma, cp_psy,
						   companion_psy);
	if (ret)
		goto prepared_fallback;

	/*
	 * Mirror the original OnePlus 2:1 start gate: target through
	 * target+200 mV.  Keep our additional settle proof, requiring two accepted
	 * readings at the same source request before handoff.  The physical driver
	 * independently rechecks the same vendor window immediately before enable
	 * and then confirms that the pump is switching.  Once the first strict
	 * reading is captured, tolerate up to two adjacent readings no more than
	 * 40 mV outside the strict band and resample without moving the source. A
	 * miss itself never counts as acceptance evidence.
	 */
	battmgr->oneplus_pps_2to1_stage = BATTMGR_ONEPLUS_2TO1_FEEDBACK;
	for (attempt = 0; attempt < feedback_attempt_limit;
	     attempt++) {
		ret = qcom_battmgr_oneplus_kick_cp_watchdog(cp_psy);
		if (ret)
			goto prepared_fallback;
		ret = qcom_battmgr_oneplus_kick_cp_watchdog(companion_psy);
		if (ret)
			goto prepared_fallback;
		ret = qcom_battmgr_oneplus_read_cp_adc(cp_psy, secondary_owned,
							&cp_vin_uv,
							&cp_output_uv);
		if (ret)
			goto prepared_fallback;
		battmgr->oneplus_pps_cp_vin_uv = cp_vin_uv;
		if (cp_output_uv < BATTMGR_ONEPLUS_CP_2TO1_VBAT_MIN_UV ||
		    cp_output_uv > BATTMGR_ONEPLUS_CP_2TO1_VBAT_MAX_UV) {
			ret = -ERANGE;
			goto prepared_fallback;
		}

		/* Keep the target source identical to the vendor default path. */
		pump_target_uv = vbat_uv * 2;
		pump_target_uv = pump_target_uv /
			BATTMGR_ONEPLUS_CP_2TO1_TARGET_STEP_UV *
			BATTMGR_ONEPLUS_CP_2TO1_TARGET_STEP_UV;
		pump_target_uv += BATTMGR_ONEPLUS_CP_2TO1_START_DELTA_UV;
		battmgr->oneplus_pps_cp_vbat_uv = cp_output_uv;
		battmgr->oneplus_pps_pump_target_mv = pump_target_uv / 1000;
		battmgr->oneplus_pps_feedback_request_mv[attempt] =
			source_request_mv;
		battmgr->oneplus_pps_feedback_vin_uv[attempt] = cp_vin_uv;
		battmgr->oneplus_pps_feedback_attempts = attempt + 1;

		if (cp_vin_uv >= pump_target_uv +
				 BATTMGR_ONEPLUS_CP_2TO1_START_MARGIN_MIN_UV &&
		    cp_vin_uv <= pump_target_uv +
				 BATTMGR_ONEPLUS_CP_2TO1_START_BAND_UV) {
			if (!stable_samples ||
			    stable_request_mv != source_request_mv) {
				stable_samples = 0;
				stable_misses = 0;
				stable_request_mv = source_request_mv;
			}
			stable_samples++;
			battmgr->oneplus_pps_feedback_stable_samples =
				stable_samples;
			if (stable_samples >=
			    BATTMGR_ONEPLUS_CP_FEEDBACK_STABLE_SAMPLES) {
				battmgr->oneplus_pps_2to1_stage =
					BATTMGR_ONEPLUS_2TO1_COMPLETE;
				ret = 0;
				goto out_put;
			}

			msleep(BATTMGR_ONEPLUS_RAMP_DELAY_MS);
			continue;
		}
		if (stable_samples && stable_request_mv == source_request_mv &&
		    stable_misses <
			BATTMGR_ONEPLUS_CP_FEEDBACK_CONFIRM_MISSES &&
		    cp_vin_uv >= pump_target_uv +
			BATTMGR_ONEPLUS_CP_2TO1_START_MARGIN_MIN_UV -
			BATTMGR_ONEPLUS_CP_FEEDBACK_CONFIRM_TOLERANCE_UV &&
		    cp_vin_uv <= pump_target_uv +
			BATTMGR_ONEPLUS_CP_2TO1_START_BAND_UV +
			BATTMGR_ONEPLUS_CP_FEEDBACK_CONFIRM_TOLERANCE_UV) {
			stable_misses++;
			msleep(BATTMGR_ONEPLUS_RAMP_DELAY_MS);
			continue;
		}
		stable_samples = 0;
		stable_misses = 0;
		stable_request_mv = 0;
		battmgr->oneplus_pps_feedback_stable_samples = 0;

		diagnostic_target_uv = pump_target_uv +
			BATTMGR_ONEPLUS_CP_2TO1_DIAG_MARGIN_TARGET_UV;
		error_uv = cp_vin_uv < diagnostic_target_uv ?
			diagnostic_target_uv - cp_vin_uv :
			cp_vin_uv - diagnostic_target_uv;
		if (error_uv < BATTMGR_ONEPLUS_CP_2TO1_START_BAND_UV) {
			feedback_step_mv = max_t(u32,
				DIV_ROUND_CLOSEST(error_uv,
					BATTMGR_ONEPLUS_CP_2TO1_DIAG_FINE_STEP_UV) *
				(BATTMGR_ONEPLUS_CP_2TO1_DIAG_FINE_STEP_UV / 1000),
				BATTMGR_ONEPLUS_CP_2TO1_DIAG_FINE_STEP_UV / 1000);
		} else {
			feedback_step_mv =
				qcom_battmgr_oneplus_feedback_step_mv(error_uv);
		}
		if (cp_vin_uv < diagnostic_target_uv)
			next_request_mv = source_request_mv + feedback_step_mv;
		else if (source_request_mv > feedback_step_mv)
			next_request_mv = source_request_mv - feedback_step_mv;
		else
			next_request_mv = 0;

		if (next_request_mv < BATTMGR_ONEPLUS_TEST_PPS_MV ||
		    next_request_mv > BATTMGR_ONEPLUS_CP_2TO1_SOURCE_MAX_MV ||
		    next_request_mv == source_request_mv) {
			ret = -ERANGE;
			goto prepared_fallback;
		}

		ret = qcom_battmgr_oneplus_set_pps_locked(
			battmgr, next_request_mv, current_ma);
		if (ret)
			goto prepared_fallback;
		source_request_mv = next_request_mv;
		battmgr->oneplus_pps_source_request_mv = source_request_mv;
		msleep(BATTMGR_ONEPLUS_RAMP_DELAY_MS);
	}

	ret = -ERANGE;

prepared_fallback:
	if (companion_prepared) {
		qcom_battmgr_oneplus_set_cp_prepared(companion_psy, false);
		companion_prepared = false;
	}
	if (battmgr->oneplus_pps_cp_prepared) {
		battmgr->oneplus_pps_cp_restore_rc =
			qcom_battmgr_oneplus_set_cp_prepared(cp_psy, false);
		if (!battmgr->oneplus_pps_cp_restore_rc)
			battmgr->oneplus_pps_cp_prepared = false;
	}
target_fallback:
	if (companion_psy)
		qcom_battmgr_oneplus_set_cp_start_target(companion_psy, 0);
	qcom_battmgr_oneplus_set_cp_start_target(cp_psy, 0);
	qcom_battmgr_oneplus_pps_fallback_locked(battmgr, ret);

out_put:
	if (companion_psy)
		power_supply_put(companion_psy);
	power_supply_put(cp_psy);

out_unlock:
	battmgr->oneplus_pps_2to1_result = ret;
	mutex_unlock(&battmgr->lock);

	return ret;
}

static ssize_t oneplus_pps_ramp_2to1_1a_store(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t count)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	ret = qcom_battmgr_oneplus_ramp_2to1_run(
		battmgr, BATTMGR_ONEPLUS_PRIMARY_CP_NAME, false, false, false,
		false,
		BATTMGR_ONEPLUS_TEST_PPS_MA);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(oneplus_pps_ramp_2to1_1a);

static ssize_t oneplus_pps_run_2to1_5m_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	bool running;
	int result;
	int ramp_rc;
	int cp_run_rc;
	int cp_cleanup_rc;
	int source_restore_rc;
	bool valid;

	mutex_lock(&battmgr->lock);
	running = battmgr->oneplus_pps_5m_running;
	valid = battmgr->oneplus_pps_5m_valid;
	result = battmgr->oneplus_pps_5m_result;
	ramp_rc = battmgr->oneplus_pps_5m_ramp_rc;
	cp_run_rc = battmgr->oneplus_pps_5m_cp_run_rc;
	cp_cleanup_rc = battmgr->oneplus_pps_5m_cp_cleanup_rc;
	source_restore_rc = battmgr->oneplus_pps_5m_source_restore_rc;
	mutex_unlock(&battmgr->lock);
	if (!running && !valid)
		return sysfs_emit(buf, "not_run\n");

	return sysfs_emit(buf,
		"revision=stage4av-kernel-handoff-5m rc=%d running=%u ramp_rc=%d cp_run_ms=%u cp_run_rc=%d cp_cleanup_rc=%d source_restore_rc=%d\n",
		result, running, ramp_rc, BATTMGR_ONEPLUS_CP_RUN_5M_MS,
		cp_run_rc, cp_cleanup_rc, source_restore_rc);
}

static ssize_t oneplus_pps_run_2to1_5m_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	struct power_supply *cp_psy = NULL;
	int source_restore_rc;
	int cp_cleanup_rc = -ECANCELED;
	int cp_run_rc = -ECANCELED;
	int ramp_rc;
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	mutex_lock(&battmgr->lock);
	if (battmgr->oneplus_pps_5m_running ||
	    battmgr->oneplus_policy_running ||
	    battmgr->oneplus_secondary_running ||
	    battmgr->oneplus_dual_running) {
		mutex_unlock(&battmgr->lock);
		return -EBUSY;
	}
	battmgr->oneplus_pps_5m_running = true;
	battmgr->oneplus_pps_5m_valid = false;
	battmgr->oneplus_pps_5m_result = -EINPROGRESS;
	battmgr->oneplus_pps_5m_ramp_rc = -EINPROGRESS;
	battmgr->oneplus_pps_5m_cp_run_rc = -EINPROGRESS;
	battmgr->oneplus_pps_5m_cp_cleanup_rc = 0;
	battmgr->oneplus_pps_5m_source_restore_rc = -EINPROGRESS;
	mutex_unlock(&battmgr->lock);

	/*
	 * Reuse the validated gauge-derived PPS loop, then enter the pump through
	 * its power-supply callback before returning to userspace.  qcom_battmgr is
	 * built in while the SC8547 driver is a module, so this callback preserves
	 * module lifetime without a built-in-to-module symbol dependency.
	 */
	ramp_rc = qcom_battmgr_oneplus_ramp_2to1_run(
		battmgr, BATTMGR_ONEPLUS_PRIMARY_CP_NAME, false, true, false,
		false,
		BATTMGR_ONEPLUS_TEST_PPS_MA);
	if (ramp_rc) {
		ret = ramp_rc;
		goto restore_source;
	}

	cp_psy = power_supply_get_by_name(BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
	if (!cp_psy) {
		ret = -ENODEV;
		goto restore_source;
	}

	/* This call blocks only the invoking test task, not battmgr->lock. */
	cp_run_rc = qcom_battmgr_oneplus_run_cp(
		cp_psy, BATTMGR_ONEPLUS_CP_RUN_5M_MS);
	ret = cp_run_rc;

	/* Idempotent cleanup also covers a pump-side preflight rejection. */
	cp_cleanup_rc = qcom_battmgr_oneplus_set_cp_prepared(cp_psy, false);
	qcom_battmgr_oneplus_set_cp_start_target(cp_psy, 0);
	if (!ret && cp_cleanup_rc)
		ret = cp_cleanup_rc;

restore_source:
	if (cp_psy)
		power_supply_put(cp_psy);

	mutex_lock(&battmgr->lock);
	source_restore_rc = qcom_battmgr_oneplus_fixed_5v_locked(battmgr);
	if (cp_psy && !cp_cleanup_rc)
		battmgr->oneplus_pps_cp_prepared = false;
	if (!ret && source_restore_rc)
		ret = source_restore_rc;
	battmgr->oneplus_pps_5m_ramp_rc = ramp_rc;
	battmgr->oneplus_pps_5m_cp_run_rc = cp_run_rc;
	battmgr->oneplus_pps_5m_cp_cleanup_rc = cp_cleanup_rc;
	battmgr->oneplus_pps_5m_source_restore_rc = source_restore_rc;
	battmgr->oneplus_pps_5m_result = ret;
	battmgr->oneplus_pps_5m_running = false;
	battmgr->oneplus_pps_5m_valid = true;
	mutex_unlock(&battmgr->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(oneplus_pps_run_2to1_5m);

static ssize_t oneplus_pps_secondary_500ms_show(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	bool running;
	bool valid;
	int result;
	int ramp_rc;
	int cp_run_rc;
	int cp_cleanup_rc;
	int source_restore_rc;
	int pre_primary_off_rc;
	int pre_secondary_off_rc;
	int handoff_primary_off_rc;
	int final_primary_off_rc;
	int final_secondary_off_rc;

	mutex_lock(&battmgr->lock);
	running = battmgr->oneplus_secondary_running;
	valid = battmgr->oneplus_secondary_valid;
	result = battmgr->oneplus_secondary_result;
	ramp_rc = battmgr->oneplus_secondary_ramp_rc;
	cp_run_rc = battmgr->oneplus_secondary_cp_run_rc;
	cp_cleanup_rc = battmgr->oneplus_secondary_cp_cleanup_rc;
	source_restore_rc = battmgr->oneplus_secondary_source_restore_rc;
	pre_primary_off_rc = battmgr->oneplus_secondary_pre_primary_off_rc;
	pre_secondary_off_rc =
		battmgr->oneplus_secondary_pre_secondary_off_rc;
	handoff_primary_off_rc =
		battmgr->oneplus_secondary_handoff_primary_off_rc;
	final_primary_off_rc = battmgr->oneplus_secondary_final_primary_off_rc;
	final_secondary_off_rc =
		battmgr->oneplus_secondary_final_secondary_off_rc;
	mutex_unlock(&battmgr->lock);

	if (!running && !valid)
		return sysfs_emit(buf, "not_run\n");

	return sysfs_emit(buf,
		"revision=%s role=secondary state=%s running=%u rc=%d run_ms=%u current_ma=%u ramp_rc=%d cp_run_rc=%d\n"
		"guards=pre_primary_off_rc:%d/pre_secondary_off_rc:%d/handoff_primary_off_rc:%d/final_primary_off_rc:%d/final_secondary_off_rc:%d\n"
		"cleanup=cp_rc:%d/source_5v_rc:%d\n",
		BATTMGR_ONEPLUS_POLICY_REVISION,
		running ? "active" : (result ? "failed" : "complete"),
		running, result, BATTMGR_ONEPLUS_CP_SECONDARY_TEST_MS,
		BATTMGR_ONEPLUS_TEST_PPS_MA, ramp_rc, cp_run_rc,
		pre_primary_off_rc, pre_secondary_off_rc,
		handoff_primary_off_rc, final_primary_off_rc,
		final_secondary_off_rc, cp_cleanup_rc, source_restore_rc);
}

static ssize_t oneplus_pps_secondary_500ms_store(struct device *dev,
						  struct device_attribute *attr,
						  const char *buf, size_t count)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	struct power_supply *cp_psy = NULL;
	int source_restore_rc;
	int cp_cleanup_rc = 0;
	int cp_run_rc = -ECANCELED;
	int ramp_rc = -ECANCELED;
	int pre_primary_off_rc;
	int pre_secondary_off_rc;
	int handoff_primary_off_rc = -ECANCELED;
	int final_primary_off_rc;
	int final_secondary_off_rc;
	int cleanup_ret;
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	mutex_lock(&battmgr->lock);
	if (battmgr->oneplus_secondary_running ||
	    battmgr->oneplus_policy_running ||
	    battmgr->oneplus_pps_5m_running ||
	    battmgr->oneplus_dual_running) {
		mutex_unlock(&battmgr->lock);
		return -EBUSY;
	}
	battmgr->oneplus_secondary_running = true;
	battmgr->oneplus_secondary_valid = false;
	battmgr->oneplus_secondary_result = -EINPROGRESS;
	battmgr->oneplus_secondary_ramp_rc = -EINPROGRESS;
	battmgr->oneplus_secondary_cp_run_rc = -EINPROGRESS;
	battmgr->oneplus_secondary_cp_cleanup_rc = -EINPROGRESS;
	battmgr->oneplus_secondary_source_restore_rc = -EINPROGRESS;
	battmgr->oneplus_secondary_pre_primary_off_rc = -EINPROGRESS;
	battmgr->oneplus_secondary_pre_secondary_off_rc = -EINPROGRESS;
	battmgr->oneplus_secondary_handoff_primary_off_rc = -EINPROGRESS;
	battmgr->oneplus_secondary_final_primary_off_rc = -EINPROGRESS;
	battmgr->oneplus_secondary_final_secondary_off_rc = -EINPROGRESS;
	mutex_unlock(&battmgr->lock);

	/* Refuse to alter the source unless both physical pumps begin off. */
	pre_primary_off_rc = qcom_battmgr_oneplus_require_cp_off(
		BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
	pre_secondary_off_rc = qcom_battmgr_oneplus_require_cp_off(
		BATTMGR_ONEPLUS_SECONDARY_CP_NAME);
	if (pre_primary_off_rc) {
		ret = pre_primary_off_rc;
		goto restore_source;
	}
	if (pre_secondary_off_rc) {
		ret = pre_secondary_off_rc;
		goto restore_source;
	}

	ramp_rc = qcom_battmgr_oneplus_ramp_2to1_run(
		battmgr, BATTMGR_ONEPLUS_SECONDARY_CP_NAME, false, false, true,
		false,
		BATTMGR_ONEPLUS_TEST_PPS_MA);
	if (ramp_rc) {
		ret = ramp_rc;
		goto restore_source;
	}

	/* Recheck the primary after PPS reaches the secondary handoff window. */
	handoff_primary_off_rc = qcom_battmgr_oneplus_require_cp_off(
		BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
	if (handoff_primary_off_rc) {
		ret = handoff_primary_off_rc;
		goto cleanup_cp;
	}

	cp_psy = power_supply_get_by_name(BATTMGR_ONEPLUS_SECONDARY_CP_NAME);
	if (!cp_psy) {
		ret = -ENODEV;
		goto cleanup_cp;
	}

	/* The pump callback contains the 500-ms active safety sample and restore. */
	cp_run_rc = qcom_battmgr_oneplus_run_cp(
		cp_psy, BATTMGR_ONEPLUS_CP_SECONDARY_TEST_MS);
	ret = cp_run_rc;

cleanup_cp:
	if (!cp_psy)
		cp_psy = power_supply_get_by_name(
			BATTMGR_ONEPLUS_SECONDARY_CP_NAME);
	if (cp_psy) {
		cp_cleanup_rc = qcom_battmgr_oneplus_set_cp_prepared(cp_psy,
			false);
		cleanup_ret = qcom_battmgr_oneplus_set_cp_start_target(cp_psy, 0);
		if (!cp_cleanup_rc)
			cp_cleanup_rc = cleanup_ret;
		power_supply_put(cp_psy);
		cp_psy = NULL;
		if (!ret && cp_cleanup_rc)
			ret = cp_cleanup_rc;
	} else if (!ret) {
		cp_cleanup_rc = -ENODEV;
		ret = cp_cleanup_rc;
	}

restore_source:
	mutex_lock(&battmgr->lock);
	source_restore_rc = qcom_battmgr_oneplus_fixed_5v_locked(battmgr);
	if (!cp_cleanup_rc)
		battmgr->oneplus_pps_cp_prepared = false;
	mutex_unlock(&battmgr->lock);
	if (!ret && source_restore_rc)
		ret = source_restore_rc;

	/* A successful result requires both pumps to be observably off at exit. */
	final_primary_off_rc = qcom_battmgr_oneplus_require_cp_off(
		BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
	final_secondary_off_rc = qcom_battmgr_oneplus_require_cp_off(
		BATTMGR_ONEPLUS_SECONDARY_CP_NAME);
	if (!ret && final_primary_off_rc)
		ret = final_primary_off_rc;
	if (!ret && final_secondary_off_rc)
		ret = final_secondary_off_rc;

	mutex_lock(&battmgr->lock);
	battmgr->oneplus_secondary_ramp_rc = ramp_rc;
	battmgr->oneplus_secondary_cp_run_rc = cp_run_rc;
	battmgr->oneplus_secondary_cp_cleanup_rc = cp_cleanup_rc;
	battmgr->oneplus_secondary_source_restore_rc = source_restore_rc;
	battmgr->oneplus_secondary_pre_primary_off_rc = pre_primary_off_rc;
	battmgr->oneplus_secondary_pre_secondary_off_rc = pre_secondary_off_rc;
	battmgr->oneplus_secondary_handoff_primary_off_rc =
		handoff_primary_off_rc;
	battmgr->oneplus_secondary_final_primary_off_rc =
		final_primary_off_rc;
	battmgr->oneplus_secondary_final_secondary_off_rc =
		final_secondary_off_rc;
	battmgr->oneplus_secondary_result = ret;
	battmgr->oneplus_secondary_running = false;
	battmgr->oneplus_secondary_valid = true;
	mutex_unlock(&battmgr->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(oneplus_pps_secondary_500ms);

static ssize_t oneplus_pps_dual_500ms_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	enum qcom_battmgr_oneplus_2to1_stage ramp_stage;
	bool running;
	bool valid;
	bool continuous_run;
	bool ramp_vbat_from_cache;
	bool hold_full_confirmed;
	bool hold_vnow_cv_active;
	bool hold_output_limited;
	bool hold_output_taper_active;
	bool boost_low_handoff;
	int result;
	int ramp_rc;
	int precondition_5v_rc;
	int entry_derate_rc;
	int ramp_primary_prepare_rc;
	int ramp_secondary_prepare_rc;
	int primary_entry_rc;
	int primary_pre_rc;
	int primary_post_rc;
	int secondary_post_rc;
	int primary_run_rc;
	int primary_cancel_rc;
	int secondary_cancel_rc;
	int source_step_rc;
	int primary_boost_pre_rc;
	int secondary_boost_pre_rc;
	int source_boost_rc;
	int primary_boost_post_rc;
	int secondary_boost_post_rc;
	int hold_rc;
	int source_reduce_rc;
	int secondary_loaded_check_rc;
	int secondary_target_rc;
	int secondary_run_rc;
	int primary_cleanup_rc;
	int secondary_cleanup_rc;
	int source_restore_rc;
	int pre_primary_off_rc;
	int pre_secondary_off_rc;
	int final_primary_off_rc;
	int final_secondary_off_rc;
	u32 primary_pre_ibus_ua;
	u32 primary_post_ibus_ua;
	u32 secondary_post_ibus_ua;
	u32 primary_entry_ibus_ua;
	u32 primary_boost_pre_ibus_ua;
	u32 secondary_boost_pre_ibus_ua;
	u32 primary_boost_post_ibus_ua;
	u32 secondary_boost_post_ibus_ua;
	u32 boost_attempts;
	u32 boost_request_mv[BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEPS];
	u32 boost_primary_ibus_ua[BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEPS];
	u32 boost_secondary_ibus_ua[BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEPS];
	u32 hold_samples;
	u32 hold_full_checks;
	u32 hold_vnow_cv_adjustments;
	u32 hold_adjustments;
	u32 hold_adjustments_up;
	u32 hold_adjustments_down;
	u32 hold_low_ibus_checks;
	u32 hold_emergency_adjustments;
	u32 hold_emergency_recoveries;
	u32 hold_emergency_steps;
	u32 hold_emergency_max_steps;
	u32 hold_output_taper_adjustments;
	u32 hold_primary_output_uv;
	u32 hold_secondary_output_uv;
	u32 hold_primary_output_max_uv;
	u32 hold_secondary_output_max_uv;
	u32 hold_path_drop_uv;
	u32 hold_path_drop_max_uv;
	u32 hold_records;
	u32 hold_max_request_mv;
	u32 monitor_checks;
	u32 monitor_vbat_uv;
	u32 monitor_vbat_ocv_uv;
	u32 monitor_source_request_mv;
	u32 monitor_primary_output_uv;
	u32 monitor_secondary_output_uv;
	u32 monitor_capacity;
	u32 monitor_usb_online;
	u32 post_stop_vbat_uv;
	u32 post_stop_vbat_ocv_uv;
	u32 ibus_rechecks;
	u32 ibus_recoveries;
	u32 hold_sample_no[BATTMGR_ONEPLUS_CP_DUAL_HOLD_RECORDS];
	u32 hold_request_mv[BATTMGR_ONEPLUS_CP_DUAL_HOLD_RECORDS];
	u32 hold_primary_ibus_ua[BATTMGR_ONEPLUS_CP_DUAL_HOLD_RECORDS];
	u32 hold_secondary_ibus_ua[BATTMGR_ONEPLUS_CP_DUAL_HOLD_RECORDS];
	u32 source_from_mv;
	u32 source_to_mv;
	u32 entry_derate_from_mv;
	u32 entry_derate_to_mv;
	u32 secondary_loaded_vbus_uv;
	u32 secondary_loaded_output_uv;
	u32 secondary_loaded_target_uv;
	u32 ramp_vbat_uv;
	u32 ramp_vbat_attempts;
	u32 ramp_primary_vin_uv;
	u32 ramp_primary_output_uv;
	u32 ramp_primary_adc_attempts;
	u32 ramp_secondary_vin_uv;
	u32 ramp_secondary_output_uv;
	u32 ramp_secondary_adc_attempts;
	u32 ramp_pump_target_mv;
	u32 ramp_source_request_mv;
	u32 ramp_feedback_attempts;
	u32 ramp_feedback_stable_samples;
	u32 ramp_feedback_request_mv[BATTMGR_ONEPLUS_CP_FEEDBACK_RECORDS];
	u32 ramp_feedback_vin_uv[BATTMGR_ONEPLUS_CP_FEEDBACK_RECORDS];
	unsigned int i;
	int monitor_temp_decic;
	int monitor_ibat_ua;
	int monitor_rc;
	int post_stop_ibat_ua;
	int post_stop_rc;
	bool voltage_snapshot_valid;
	bool voltage_relax_valid;
	s64 terminal_minus_ocv_uv;
	s64 primary_minus_terminal_uv;
	s64 secondary_minus_terminal_uv;
	s64 source_minus_2x_terminal_uv;
	s64 source_minus_2x_primary_uv;
	s64 post_stop_relax_uv;
	int len;

	mutex_lock(&battmgr->lock);
	running = battmgr->oneplus_dual_running;
	valid = battmgr->oneplus_dual_valid;
	continuous_run = battmgr->oneplus_dual_continuous_run;
	result = battmgr->oneplus_dual_result;
	ramp_rc = battmgr->oneplus_dual_ramp_rc;
	precondition_5v_rc = battmgr->oneplus_dual_precondition_5v_rc;
	entry_derate_rc = battmgr->oneplus_dual_entry_derate_rc;
	ramp_stage = battmgr->oneplus_pps_2to1_stage;
	ramp_vbat_uv = battmgr->oneplus_pps_vbat_uv;
	ramp_vbat_attempts = battmgr->oneplus_pps_vbat_attempts;
	ramp_vbat_from_cache = battmgr->oneplus_pps_vbat_from_cache;
	ramp_primary_vin_uv = battmgr->oneplus_pps_cp_vin_uv;
	ramp_primary_output_uv = battmgr->oneplus_pps_cp_vbat_uv;
	ramp_primary_adc_attempts = battmgr->oneplus_pps_cp_adc_attempts;
	ramp_primary_prepare_rc = battmgr->oneplus_pps_cp_prepare_rc;
	ramp_secondary_vin_uv = battmgr->oneplus_pps_companion_vin_uv;
	ramp_secondary_output_uv =
		battmgr->oneplus_pps_companion_output_uv;
	ramp_secondary_adc_attempts =
		battmgr->oneplus_pps_companion_adc_attempts;
	ramp_secondary_prepare_rc =
		battmgr->oneplus_pps_companion_prepare_rc;
	ramp_pump_target_mv = battmgr->oneplus_pps_pump_target_mv;
	ramp_source_request_mv = battmgr->oneplus_pps_source_request_mv;
	ramp_feedback_attempts = battmgr->oneplus_pps_feedback_attempts;
	ramp_feedback_stable_samples =
		battmgr->oneplus_pps_feedback_stable_samples;
	memcpy(ramp_feedback_request_mv,
	       battmgr->oneplus_pps_feedback_request_mv,
	       sizeof(ramp_feedback_request_mv));
	memcpy(ramp_feedback_vin_uv,
	       battmgr->oneplus_pps_feedback_vin_uv,
	       sizeof(ramp_feedback_vin_uv));
	primary_entry_rc = battmgr->oneplus_dual_primary_entry_rc;
	primary_pre_rc = battmgr->oneplus_dual_primary_pre_rc;
	primary_post_rc = battmgr->oneplus_dual_primary_post_rc;
	secondary_post_rc = battmgr->oneplus_dual_secondary_post_rc;
	primary_run_rc = battmgr->oneplus_dual_primary_run_rc;
	primary_cancel_rc = battmgr->oneplus_dual_primary_cancel_rc;
	secondary_cancel_rc = battmgr->oneplus_dual_secondary_cancel_rc;
	source_step_rc = battmgr->oneplus_dual_source_step_rc;
	primary_boost_pre_rc = battmgr->oneplus_dual_primary_boost_pre_rc;
	secondary_boost_pre_rc = battmgr->oneplus_dual_secondary_boost_pre_rc;
	source_boost_rc = battmgr->oneplus_dual_source_boost_rc;
	primary_boost_post_rc = battmgr->oneplus_dual_primary_boost_post_rc;
	secondary_boost_post_rc =
		battmgr->oneplus_dual_secondary_boost_post_rc;
	hold_rc = battmgr->oneplus_dual_hold_rc;
	source_reduce_rc = battmgr->oneplus_dual_source_reduce_rc;
	secondary_loaded_check_rc =
		battmgr->oneplus_dual_secondary_loaded_check_rc;
	secondary_target_rc = battmgr->oneplus_dual_secondary_target_rc;
	secondary_run_rc = battmgr->oneplus_dual_secondary_run_rc;
	primary_cleanup_rc = battmgr->oneplus_dual_primary_cleanup_rc;
	secondary_cleanup_rc = battmgr->oneplus_dual_secondary_cleanup_rc;
	source_restore_rc = battmgr->oneplus_dual_source_restore_rc;
	pre_primary_off_rc = battmgr->oneplus_dual_pre_primary_off_rc;
	pre_secondary_off_rc = battmgr->oneplus_dual_pre_secondary_off_rc;
	final_primary_off_rc = battmgr->oneplus_dual_final_primary_off_rc;
	final_secondary_off_rc =
		battmgr->oneplus_dual_final_secondary_off_rc;
	primary_pre_ibus_ua = battmgr->oneplus_dual_primary_pre_ibus_ua;
	primary_post_ibus_ua = battmgr->oneplus_dual_primary_post_ibus_ua;
	secondary_post_ibus_ua =
		battmgr->oneplus_dual_secondary_post_ibus_ua;
	primary_entry_ibus_ua = battmgr->oneplus_dual_primary_entry_ibus_ua;
	primary_boost_pre_ibus_ua =
		battmgr->oneplus_dual_primary_boost_pre_ibus_ua;
	secondary_boost_pre_ibus_ua =
		battmgr->oneplus_dual_secondary_boost_pre_ibus_ua;
	primary_boost_post_ibus_ua =
		battmgr->oneplus_dual_primary_boost_post_ibus_ua;
	secondary_boost_post_ibus_ua =
		battmgr->oneplus_dual_secondary_boost_post_ibus_ua;
	boost_attempts = battmgr->oneplus_dual_boost_attempts;
	memcpy(boost_request_mv, battmgr->oneplus_dual_boost_request_mv,
	       sizeof(boost_request_mv));
	memcpy(boost_primary_ibus_ua,
	       battmgr->oneplus_dual_boost_primary_ibus_ua,
	       sizeof(boost_primary_ibus_ua));
	memcpy(boost_secondary_ibus_ua,
	       battmgr->oneplus_dual_boost_secondary_ibus_ua,
	       sizeof(boost_secondary_ibus_ua));
	hold_samples = battmgr->oneplus_dual_hold_samples;
	hold_full_confirmed = battmgr->oneplus_dual_hold_full_confirmed;
	hold_full_checks = battmgr->oneplus_dual_hold_full_checks;
	hold_vnow_cv_active = battmgr->oneplus_dual_hold_vnow_cv_active;
	hold_vnow_cv_adjustments =
		battmgr->oneplus_dual_hold_vnow_cv_adjustments;
	hold_adjustments = battmgr->oneplus_dual_hold_adjustments;
	hold_adjustments_up = battmgr->oneplus_dual_hold_adjustments_up;
	hold_adjustments_down = battmgr->oneplus_dual_hold_adjustments_down;
	hold_low_ibus_checks = battmgr->oneplus_dual_hold_low_ibus_checks;
	hold_emergency_adjustments =
		battmgr->oneplus_dual_hold_emergency_adjustments;
	hold_emergency_recoveries =
		battmgr->oneplus_dual_hold_emergency_recoveries;
	hold_emergency_steps = battmgr->oneplus_dual_hold_emergency_steps;
	hold_emergency_max_steps =
		battmgr->oneplus_dual_hold_emergency_max_steps;
	hold_output_limited = battmgr->oneplus_dual_hold_output_limited;
	hold_output_taper_active =
		battmgr->oneplus_dual_hold_output_taper_active;
	hold_output_taper_adjustments =
		battmgr->oneplus_dual_hold_output_taper_adjustments;
	hold_primary_output_uv = battmgr->oneplus_dual_hold_primary_output_uv;
	hold_secondary_output_uv =
		battmgr->oneplus_dual_hold_secondary_output_uv;
	hold_primary_output_max_uv =
		battmgr->oneplus_dual_hold_primary_output_max_uv;
	hold_secondary_output_max_uv =
		battmgr->oneplus_dual_hold_secondary_output_max_uv;
	hold_path_drop_uv = battmgr->oneplus_dual_hold_path_drop_uv;
	hold_path_drop_max_uv =
		battmgr->oneplus_dual_hold_path_drop_max_uv;
	hold_records = battmgr->oneplus_dual_hold_records;
	monitor_checks = battmgr->oneplus_dual_monitor_checks;
	monitor_vbat_uv = battmgr->oneplus_dual_monitor_vbat_uv;
	monitor_vbat_ocv_uv = battmgr->oneplus_dual_monitor_vbat_ocv_uv;
	monitor_source_request_mv =
		battmgr->oneplus_dual_monitor_source_request_mv;
	monitor_primary_output_uv =
		battmgr->oneplus_dual_monitor_primary_output_uv;
	monitor_secondary_output_uv =
		battmgr->oneplus_dual_monitor_secondary_output_uv;
	monitor_capacity = battmgr->oneplus_dual_monitor_capacity;
	monitor_usb_online = battmgr->oneplus_dual_monitor_usb_online;
	monitor_ibat_ua = battmgr->oneplus_dual_monitor_ibat_ua;
	monitor_temp_decic = battmgr->oneplus_dual_monitor_temp_decic;
	monitor_rc = battmgr->oneplus_dual_monitor_rc;
	post_stop_vbat_uv = battmgr->oneplus_dual_post_stop_vbat_uv;
	post_stop_vbat_ocv_uv = battmgr->oneplus_dual_post_stop_vbat_ocv_uv;
	post_stop_ibat_ua = battmgr->oneplus_dual_post_stop_ibat_ua;
	post_stop_rc = battmgr->oneplus_dual_post_stop_rc;
	ibus_rechecks = battmgr->oneplus_dual_ibus_rechecks;
	ibus_recoveries = battmgr->oneplus_dual_ibus_recoveries;
	memcpy(hold_sample_no, battmgr->oneplus_dual_hold_sample_no,
	       sizeof(hold_sample_no));
	memcpy(hold_request_mv, battmgr->oneplus_dual_hold_request_mv,
	       sizeof(hold_request_mv));
	memcpy(hold_primary_ibus_ua,
	       battmgr->oneplus_dual_hold_primary_ibus_ua,
	       sizeof(hold_primary_ibus_ua));
	memcpy(hold_secondary_ibus_ua,
	       battmgr->oneplus_dual_hold_secondary_ibus_ua,
	       sizeof(hold_secondary_ibus_ua));
	source_from_mv = battmgr->oneplus_dual_source_from_mv;
	source_to_mv = battmgr->oneplus_dual_source_to_mv;
	hold_max_request_mv = BATTMGR_ONEPLUS_CP_2TO1_SOURCE_MAX_MV;
	entry_derate_from_mv = battmgr->oneplus_dual_entry_derate_from_mv;
	entry_derate_to_mv = battmgr->oneplus_dual_entry_derate_to_mv;
	secondary_loaded_vbus_uv =
		battmgr->oneplus_dual_secondary_loaded_vbus_uv;
	secondary_loaded_output_uv =
		battmgr->oneplus_dual_secondary_loaded_output_uv;
	secondary_loaded_target_uv =
		battmgr->oneplus_dual_secondary_loaded_target_uv;
	mutex_unlock(&battmgr->lock);
	voltage_snapshot_valid = monitor_checks && !monitor_rc &&
		monitor_source_request_mv && monitor_vbat_uv &&
		monitor_primary_output_uv && monitor_secondary_output_uv;
	voltage_relax_valid = monitor_checks && !monitor_rc &&
		!post_stop_rc && monitor_vbat_uv && post_stop_vbat_uv;
	terminal_minus_ocv_uv = (s64)monitor_vbat_uv - monitor_vbat_ocv_uv;
	primary_minus_terminal_uv = voltage_snapshot_valid ?
		(s64)monitor_primary_output_uv - monitor_vbat_uv : 0;
	secondary_minus_terminal_uv = voltage_snapshot_valid ?
		(s64)monitor_secondary_output_uv - monitor_vbat_uv : 0;
	source_minus_2x_terminal_uv = voltage_snapshot_valid ?
		(s64)monitor_source_request_mv * 1000 -
			(s64)monitor_vbat_uv * 2 : 0;
	source_minus_2x_primary_uv = voltage_snapshot_valid ?
		(s64)monitor_source_request_mv * 1000 -
			(s64)monitor_primary_output_uv * 2 : 0;
	post_stop_relax_uv = (s64)monitor_vbat_uv - post_stop_vbat_uv;
	boost_low_handoff =
		boost_attempts == BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEPS &&
		primary_boost_post_ibus_ua >=
			BATTMGR_ONEPLUS_CP_DUAL_BOOST_HANDOFF_PUMP_MIN_UA &&
		secondary_boost_post_ibus_ua >=
			BATTMGR_ONEPLUS_CP_DUAL_BOOST_HANDOFF_PUMP_MIN_UA &&
		primary_boost_post_ibus_ua + secondary_boost_post_ibus_ua >=
			BATTMGR_ONEPLUS_CP_DUAL_BOOST_HANDOFF_MIN_UA &&
		primary_boost_post_ibus_ua + secondary_boost_post_ibus_ua <
			BATTMGR_ONEPLUS_CP_DUAL_BOOST_TARGET_UA -
			BATTMGR_ONEPLUS_CP_DUAL_BOOST_DEADBAND_UA;

	if (!running && !valid)
		return sysfs_emit(buf, "not_run\n");

	len = sysfs_emit(buf,
		"revision=%s state=%s mode=%s running=%u rc=%d current_ma=%u boost_ma=%u boost_ms=%u boost_step_mv=%u boost_steps=%u boost_target_ua=%u primary_run_ms=%u secondary_run_ms=%u primary_settle_ms=%u idle_settle_ms=%u precondition_5v_rc=%d ramp_rc=%d\n"
		"ramp_diag=stage:%s/vbat_uv:%u/vbat_attempts:%u/vbat_cache:%u/primary_vin_uv:%u/primary_output_uv:%u/primary_adc_attempts:%u/primary_prepare_rc:%d/secondary_vin_uv:%u/secondary_output_uv:%u/secondary_adc_attempts:%u/secondary_prepare_rc:%d/pump_target_mv:%u/source_request_mv:%u/feedback_attempts:%u/feedback_stable_samples:%u\n"
		"entry_derate=rc:%d/from_mv:%u/to_mv:%u/step_mv:%u/settle_ms:%u\n"
		"handoff=primary_entry_rc:%d/primary_entry_ibus_ua:%u/source_step_rc:%d/source_from_mv:%u/source_to_mv:%u/loaded_check_rc:%d/secondary_vbus_uv:%u/secondary_output_uv:%u/secondary_target_uv:%u/secondary_target_rc:%d\n"
		"boost=primary_pre_rc:%d/primary_pre_ibus_ua:%u/secondary_pre_rc:%d/secondary_pre_ibus_ua:%u/source_rc:%d/primary_post_rc:%d/primary_post_ibus_ua:%u/secondary_post_rc:%d/secondary_post_ibus_ua:%u/reduce_rc:%d/handoff_low:%u/handoff_min_ua:%u/handoff_pump_min_ua:%u\n"
		"overlap=stop_mode:%s/primary_pre_rc:%d/primary_pre_ibus_ua:%u/primary_post_rc:%d/primary_post_ibus_ua:%u/secondary_post_rc:%d/secondary_post_ibus_ua:%u/primary_run_rc:%d/secondary_run_rc:%d/primary_cancel_rc:%d/secondary_cancel_rc:%d\n"
		"guards=pre_primary_off_rc:%d/pre_secondary_off_rc:%d/final_primary_off_rc:%d/final_secondary_off_rc:%d\n"
		"cleanup=primary_rc:%d/secondary_rc:%d/source_5v_rc:%d\n",
		BATTMGR_ONEPLUS_POLICY_REVISION,
		running ? "active" : (result ? "failed" : "complete"),
		continuous_run ? "continuous" : "bounded",
		running, result, BATTMGR_ONEPLUS_CP_DUAL_PPS_MA,
		BATTMGR_ONEPLUS_CP_DUAL_BOOST_PPS_MA,
		BATTMGR_ONEPLUS_CP_DUAL_BOOST_MS,
		BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEP_MV,
		BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEPS,
		BATTMGR_ONEPLUS_CP_DUAL_BOOST_TARGET_UA,
		continuous_run ? 0 : BATTMGR_ONEPLUS_CP_DUAL_PRIMARY_RUN_MS,
		continuous_run ? 0 : BATTMGR_ONEPLUS_CP_DUAL_SECONDARY_RUN_MS,
		BATTMGR_ONEPLUS_CP_DUAL_PRIMARY_SETTLE_MS,
		BATTMGR_ONEPLUS_CP_DUAL_IDLE_SETTLE_MS,
		precondition_5v_rc, ramp_rc,
		qcom_battmgr_oneplus_2to1_stage_name(ramp_stage),
		ramp_vbat_uv, ramp_vbat_attempts, ramp_vbat_from_cache,
		ramp_primary_vin_uv, ramp_primary_output_uv,
		ramp_primary_adc_attempts, ramp_primary_prepare_rc,
		ramp_secondary_vin_uv, ramp_secondary_output_uv,
		ramp_secondary_adc_attempts, ramp_secondary_prepare_rc,
		ramp_pump_target_mv, ramp_source_request_mv,
		ramp_feedback_attempts, ramp_feedback_stable_samples,
		entry_derate_rc, entry_derate_from_mv, entry_derate_to_mv,
		BATTMGR_ONEPLUS_CP_DUAL_ENTRY_DERATE_MV,
		BATTMGR_ONEPLUS_CP_DUAL_ENTRY_SETTLE_MS,
		primary_entry_rc, primary_entry_ibus_ua, source_step_rc,
		source_from_mv, source_to_mv, secondary_loaded_check_rc,
		secondary_loaded_vbus_uv, secondary_loaded_output_uv,
		secondary_loaded_target_uv, secondary_target_rc,
		primary_boost_pre_rc, primary_boost_pre_ibus_ua,
		secondary_boost_pre_rc, secondary_boost_pre_ibus_ua,
		source_boost_rc, primary_boost_post_rc,
		primary_boost_post_ibus_ua, secondary_boost_post_rc,
		secondary_boost_post_ibus_ua, source_reduce_rc,
		boost_low_handoff,
		BATTMGR_ONEPLUS_CP_DUAL_BOOST_HANDOFF_MIN_UA,
		BATTMGR_ONEPLUS_CP_DUAL_BOOST_HANDOFF_PUMP_MIN_UA,
		hold_full_confirmed ? "vnow-full-immediate" :
			(hold_output_limited ? "output-limit-immediate" :
			 "validated"),
		primary_pre_rc, primary_pre_ibus_ua, primary_post_rc,
		primary_post_ibus_ua, secondary_post_rc,
		secondary_post_ibus_ua, primary_run_rc, secondary_run_rc,
		primary_cancel_rc, secondary_cancel_rc, pre_primary_off_rc,
		pre_secondary_off_rc,
		final_primary_off_rc, final_secondary_off_rc,
		primary_cleanup_rc, secondary_cleanup_rc, source_restore_rc);
	len += sysfs_emit_at(buf, len,
		"hold=rc:%d/duration_ms:%u/sample_ms:%u/samples:%u/records:%u/adjustments:%u/up:%u/down:%u/up_pending:%u/up_required:%u/up_step_mv:%u/down_step_mv:%u/emergency:%u/emergency_recovered:%u/emergency_streak:%u/emergency_streak_max:%u/emergency_streak_limit:%u/full_confirmed:%u/full_checks:%u/full_vnow_uv:%u/full_ibat_max_ua:%u/full_capacity_min:%u/full_required:%u/vnow_cv_active:%u/vnow_cv_steps:%u/vnow_cv_enter_uv:%u/vnow_cv_release_uv:%u/vnow_cv_step_mv:%u/output_taper:%u/output_taper_steps:%u/soft_ibus_ua:%u/emergency_max_ua:%u/emergency_step_mv:%u/max_mv:%u/output_limited:%u\n",
		hold_rc, continuous_run ? 0 : BATTMGR_ONEPLUS_CP_DUAL_HOLD_MS,
		BATTMGR_ONEPLUS_CP_DUAL_HOLD_SAMPLE_MS, hold_samples,
		hold_records, hold_adjustments, hold_adjustments_up,
		hold_adjustments_down, hold_low_ibus_checks,
		BATTMGR_ONEPLUS_CP_DUAL_LOW_IBUS_CONFIRM_SAMPLES,
		BATTMGR_ONEPLUS_CP_DUAL_LOW_IBUS_STEP_MV,
		BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEP_MV,
		hold_emergency_adjustments,
		hold_emergency_recoveries, hold_emergency_steps,
		hold_emergency_max_steps,
		BATTMGR_ONEPLUS_CP_DUAL_EMERGENCY_MAX_STEPS,
		hold_full_confirmed, hold_full_checks,
		BATTMGR_ONEPLUS_CP_DUAL_FULL_VBAT_UV,
		BATTMGR_ONEPLUS_CP_DUAL_FULL_IBAT_MAX_UA,
		BATTMGR_ONEPLUS_CP_DUAL_FULL_CAPACITY_MIN,
		BATTMGR_ONEPLUS_CP_DUAL_FULL_CHECKS,
		hold_vnow_cv_active, hold_vnow_cv_adjustments,
		BATTMGR_ONEPLUS_CP_DUAL_VNOW_CV_ENTER_UV,
		BATTMGR_ONEPLUS_CP_DUAL_VNOW_CV_RELEASE_UV,
		BATTMGR_ONEPLUS_CP_DUAL_VNOW_CV_STEP_MV,
		hold_output_taper_active, hold_output_taper_adjustments,
		BATTMGR_ONEPLUS_CP_DUAL_HOLD_SOFT_IBUS_UA,
		BATTMGR_ONEPLUS_CP_DUAL_EMERGENCY_IBUS_MAX_UA,
		BATTMGR_ONEPLUS_CP_DUAL_EMERGENCY_STEP_MV,
		hold_max_request_mv, hold_output_limited);
	len += sysfs_emit_at(buf, len,
		"output_guard=primary_soft_uv:%u/primary_hard_uv:%u/secondary_soft_uv:%u/secondary_hard_uv:%u/path_soft_uv:%u/path_hard_uv:%u/primary_uv:%u/secondary_uv:%u/path_uv:%u/primary_max_uv:%u/secondary_max_uv:%u/path_max_uv:%u\n",
		BATTMGR_ONEPLUS_CP_DUAL_PRIMARY_TAPER_UV,
		BATTMGR_ONEPLUS_CP_2TO1_VBAT_MAX_UV,
		BATTMGR_ONEPLUS_CP_DUAL_SECONDARY_TAPER_UV,
		BATTMGR_ONEPLUS_CP_DUAL_SECONDARY_MAX_UV,
		BATTMGR_ONEPLUS_CP_DUAL_PATH_TAPER_UV,
		BATTMGR_ONEPLUS_CP_DUAL_PATH_MAX_UV,
		hold_primary_output_uv, hold_secondary_output_uv,
		hold_path_drop_uv, hold_primary_output_max_uv,
		hold_secondary_output_max_uv, hold_path_drop_max_uv);
	len += sysfs_emit_at(buf, len,
		"ibus_guard=max_ua:%u/physical_max_ua:%u/adc_ceiling_ua:%u/rechecks:%u/recoveries:%u\n",
		BATTMGR_ONEPLUS_CP_DUAL_IBUS_MAX_UA,
		BATTMGR_ONEPLUS_CP_DUAL_EMERGENCY_IBUS_MAX_UA,
		BATTMGR_ONEPLUS_CP_DUAL_EMERGENCY_IBUS_ADC_MAX_UA,
		ibus_rechecks, ibus_recoveries);
	len += sysfs_emit_at(buf, len,
		"monitor=checks:%u/rc:%d/usb_online:%u/vbat_terminal_uv:%u/vbat_ocv_est_uv:%u/ibat_ua:%d/capacity:%u/vnow_start_max_uv:%u/vnow_hard_max_uv:%u/temp_decic:%d/temp_min_decic:%d/temp_max_decic:%d\n",
		monitor_checks, monitor_rc, monitor_usb_online,
		monitor_vbat_uv, monitor_vbat_ocv_uv, monitor_ibat_ua,
		monitor_capacity, BATTMGR_ONEPLUS_POLICY_VBAT_MAX_UV,
		BATTMGR_ONEPLUS_CP_2TO1_VBAT_MAX_UV,
		monitor_temp_decic, BATTMGR_ONEPLUS_POLICY_TEMP_MIN_DECIC,
		BATTMGR_ONEPLUS_POLICY_TEMP_MAX_DECIC);
	len += sysfs_emit_at(buf, len,
		"post_stop=rc:%d/settle_ms:%u/vbat_terminal_uv:%u/vbat_ocv_est_uv:%u/ibat_ua:%d\n",
		post_stop_rc, BATTMGR_ONEPLUS_CP_DUAL_POST_STOP_SETTLE_MS,
		post_stop_vbat_uv, post_stop_vbat_ocv_uv,
		post_stop_ibat_ua);
	len += sysfs_emit_at(buf, len,
		"voltage_sources=policy:gauge-vnow/full:gauge-vnow/ocv:gauge-ocv-diagnostic/primary:sc8547-vbat/secondary:sc8547-vout\n");
	len += sysfs_emit_at(buf, len,
		"voltage_snapshot=valid:%u/source_request_mv:%u/terminal_uv:%u/ocv_est_uv:%u/terminal_minus_ocv_uv:%lld/primary_uv:%u/primary_minus_terminal_uv:%lld/secondary_uv:%u/secondary_minus_terminal_uv:%lld/source_minus_2x_terminal_uv:%lld/source_minus_2x_primary_uv:%lld\n",
		voltage_snapshot_valid, monitor_source_request_mv,
		monitor_vbat_uv, monitor_vbat_ocv_uv,
		terminal_minus_ocv_uv, monitor_primary_output_uv,
		primary_minus_terminal_uv, monitor_secondary_output_uv,
		secondary_minus_terminal_uv, source_minus_2x_terminal_uv,
		source_minus_2x_primary_uv);
	len += sysfs_emit_at(buf, len,
		"voltage_relax=valid:%u/loaded_terminal_uv:%u/post_stop_terminal_uv:%u/relax_uv:%lld/loaded_ibat_ua:%d/post_stop_ibat_ua:%d\n",
		voltage_relax_valid, monitor_vbat_uv, post_stop_vbat_uv,
		post_stop_relax_uv, monitor_ibat_ua, post_stop_ibat_ua);
	len += sysfs_emit_at(buf, len, "ramp_trace=");
	for (i = 0; i < min_t(u32, ramp_feedback_attempts,
				     BATTMGR_ONEPLUS_CP_FEEDBACK_RECORDS); i++)
		len += sysfs_emit_at(buf, len, "%s%u/%u", i ? "," : "",
				     ramp_feedback_request_mv[i],
				     ramp_feedback_vin_uv[i]);
	len += sysfs_emit_at(buf, len, "\n");
	len += sysfs_emit_at(buf, len, "boost_trace=");
	for (i = 0; i < min_t(u32, boost_attempts,
				     BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEPS); i++)
		len += sysfs_emit_at(buf, len, "%s%u/%u/%u", i ? "," : "",
				     boost_request_mv[i],
				     boost_primary_ibus_ua[i],
				     boost_secondary_ibus_ua[i]);
	len += sysfs_emit_at(buf, len, "\n");
	len += sysfs_emit_at(buf, len, "hold_trace=");
	for (i = 0; i < min_t(u32, hold_records,
				     BATTMGR_ONEPLUS_CP_DUAL_HOLD_RECORDS); i++) {
		len += sysfs_emit_at(buf, len, "%s%u:%u/%u/%u",
				     i ? "," : "", hold_sample_no[i],
				     hold_request_mv[i],
				     hold_primary_ibus_ua[i],
				     hold_secondary_ibus_ua[i]);
	}
	len += sysfs_emit_at(buf, len, "\n");

	return len;
}

static int qcom_battmgr_oneplus_dual_system_check(
	struct qcom_battmgr *battmgr, u32 *usb_online, u32 *vbat_uv,
	u32 *vbat_ocv_uv, int *ibat_ua, u32 *capacity, int *temp_decic)
{
	int ret;

	mutex_lock(&battmgr->lock);
	if (!battmgr->service_up) {
		ret = -ENOTCONN;
		goto out_unlock;
	}

	ret = qcom_battmgr_request_property(battmgr,
		BATTMGR_USB_PROPERTY_GET, USB_ONLINE, 0);
	if (ret)
		goto out_unlock;
	*usb_online = battmgr->usb.online;
	if (!*usb_online) {
		ret = -ENODEV;
		goto out_unlock;
	}

	ret = qcom_battmgr_request_property(battmgr,
		BATTMGR_BAT_PROPERTY_GET, BATT_VOLT_NOW, 0);
	if (ret)
		goto out_unlock;
	*vbat_uv = qcom_battmgr_oneplus_vbat_uv(battmgr);
	if (!*vbat_uv) {
		ret = -EAGAIN;
		goto out_unlock;
	}
	if (*vbat_uv > BATTMGR_ONEPLUS_CP_2TO1_VBAT_MAX_UV) {
		ret = -ERANGE;
		goto out_unlock;
	}

	/*
	 * The original OnePlus charging stack uses the gauge VOLTAGE_NOW path
	 * for charge-state decisions. Keep OCV best-effort and diagnostic only:
	 * on this target it can track a modelled charging value and must never
	 * block or complete an active PPS run.
	 */
	*vbat_ocv_uv = 0;
	ret = qcom_battmgr_request_property(battmgr,
		BATTMGR_BAT_PROPERTY_GET, BATT_VOLT_OCV, 0);
	if (!ret)
		*vbat_ocv_uv = battmgr->status.voltage_ocv;

	ret = qcom_battmgr_request_property(battmgr,
		BATTMGR_BAT_PROPERTY_GET, BATT_CURR_NOW, 0);
	if (ret)
		goto out_unlock;
	*ibat_ua = battmgr->status.current_now;

	ret = qcom_battmgr_request_property(battmgr,
		BATTMGR_BAT_PROPERTY_GET, BATT_CAPACITY, 0);
	if (ret)
		goto out_unlock;
	*capacity = battmgr->status.percent;

	ret = qcom_battmgr_request_property(battmgr,
		BATTMGR_BAT_PROPERTY_GET, BATT_TEMP, 0);
	if (ret)
		goto out_unlock;
	*temp_decic = battmgr->status.temperature;
	if (*temp_decic < BATTMGR_ONEPLUS_POLICY_TEMP_MIN_DECIC ||
	    *temp_decic > BATTMGR_ONEPLUS_POLICY_TEMP_MAX_DECIC)
		ret = -ERANGE;

out_unlock:
	mutex_unlock(&battmgr->lock);
	return ret;
}

static int qcom_battmgr_oneplus_dual_battery_sample(
	struct qcom_battmgr *battmgr, u32 *vbat_uv, u32 *vbat_ocv_uv,
	int *ibat_ua)
{
	int ret;

	mutex_lock(&battmgr->lock);
	if (!battmgr->service_up) {
		ret = -ENOTCONN;
		goto out_unlock;
	}

	ret = qcom_battmgr_request_property(battmgr,
		BATTMGR_BAT_PROPERTY_GET, BATT_VOLT_NOW, 0);
	if (ret)
		goto out_unlock;
	*vbat_uv = qcom_battmgr_oneplus_vbat_uv(battmgr);
	if (!*vbat_uv) {
		ret = -EAGAIN;
		goto out_unlock;
	}

	/* OCV remains best-effort and diagnostic only. */
	*vbat_ocv_uv = 0;
	ret = qcom_battmgr_request_property(battmgr,
		BATTMGR_BAT_PROPERTY_GET, BATT_VOLT_OCV, 0);
	if (!ret)
		*vbat_ocv_uv = battmgr->status.voltage_ocv;

	ret = qcom_battmgr_request_property(battmgr,
		BATTMGR_BAT_PROPERTY_GET, BATT_CURR_NOW, 0);
	if (ret)
		goto out_unlock;
	*ibat_ua = battmgr->status.current_now;

out_unlock:
	mutex_unlock(&battmgr->lock);
	return ret;
}

static bool qcom_battmgr_oneplus_dual_full_check(
	u32 vbat_uv, int ibat_ua, u32 capacity, u32 *checks)
{
	/* CURRENT_NOW is negative while this platform is charging. */
	if (vbat_uv >= BATTMGR_ONEPLUS_CP_DUAL_FULL_VBAT_UV &&
	    ibat_ua <= 0 &&
	    ibat_ua >= -BATTMGR_ONEPLUS_CP_DUAL_FULL_IBAT_MAX_UA &&
	    capacity >= BATTMGR_ONEPLUS_CP_DUAL_FULL_CAPACITY_MIN)
		(*checks)++;
	else
		*checks = 0;

	return *checks >= BATTMGR_ONEPLUS_CP_DUAL_FULL_CHECKS;
}

static bool qcom_battmgr_oneplus_dual_request_stop(
	struct qcom_battmgr *battmgr)
{
	struct power_supply *primary_psy;
	struct power_supply *secondary_psy;
	bool running;

	mutex_lock(&battmgr->lock);
	running = battmgr->oneplus_dual_running;
	if (running)
		battmgr->oneplus_dual_stop_requested = true;
	mutex_unlock(&battmgr->lock);
	if (!running)
		return false;

	primary_psy = power_supply_get_by_name(
		BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
	secondary_psy = power_supply_get_by_name(
		BATTMGR_ONEPLUS_SECONDARY_CP_NAME);
	if (primary_psy) {
		qcom_battmgr_oneplus_cancel_cp(primary_psy);
		power_supply_put(primary_psy);
	}
	if (secondary_psy) {
		qcom_battmgr_oneplus_cancel_cp(secondary_psy);
		power_supply_put(secondary_psy);
	}

	return true;
}

static ssize_t qcom_battmgr_oneplus_dual_run(struct qcom_battmgr *battmgr,
					      size_t count, bool automatic)
{
	struct qcom_battmgr_oneplus_cp_run_ctx primary_ctx;
	struct qcom_battmgr_oneplus_cp_run_ctx secondary_ctx;
	struct power_supply *primary_psy = NULL;
	struct power_supply *secondary_psy = NULL;
	struct task_struct *primary_task;
	struct task_struct *secondary_task;
	bool primary_started = false;
	bool secondary_started = false;
	int primary_cancel_rc = 0;
	int secondary_cancel_rc = 0;
	int primary_cleanup_rc = 0;
	int secondary_cleanup_rc = 0;
	int primary_entry_rc = -ECANCELED;
	int primary_run_rc = -ECANCELED;
	int secondary_run_rc = -ECANCELED;
	int primary_pre_rc = -ECANCELED;
	int primary_post_rc = -ECANCELED;
	int secondary_post_rc = -ECANCELED;
	int source_step_rc = -ECANCELED;
	int primary_boost_pre_rc = -ECANCELED;
	int secondary_boost_pre_rc = -ECANCELED;
	int source_boost_rc = -ECANCELED;
	int primary_boost_post_rc = -ECANCELED;
	int secondary_boost_post_rc = -ECANCELED;
	int hold_rc = -ECANCELED;
	int primary_hold_check_rc = -ECANCELED;
	int secondary_hold_check_rc = -ECANCELED;
	int source_reduce_rc = -ECANCELED;
	int secondary_loaded_check_rc = -ECANCELED;
	int secondary_target_rc = -ECANCELED;
	int source_restore_rc;
	int precondition_5v_rc = -ECANCELED;
	int ramp_rc = -ECANCELED;
	int entry_derate_rc = -ECANCELED;
	int pre_primary_off_rc;
	int pre_secondary_off_rc;
	int final_primary_off_rc;
	int final_secondary_off_rc;
	u32 primary_pre_ibus_ua = 0;
	u32 primary_post_ibus_ua = 0;
	u32 secondary_post_ibus_ua = 0;
	u32 primary_entry_ibus_ua = 0;
	u32 primary_boost_pre_ibus_ua = 0;
	u32 secondary_boost_pre_ibus_ua = 0;
	u32 primary_boost_post_ibus_ua = 0;
	u32 secondary_boost_post_ibus_ua = 0;
	u32 boost_attempts = 0;
	u32 boost_request_mv[BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEPS] = { 0 };
	u32 boost_primary_ibus_ua[BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEPS] = { 0 };
	u32 boost_secondary_ibus_ua[BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEPS] = { 0 };
	u32 aggregate_boost_ibus_ua = 0;
	u32 hold_samples = 0;
	u32 hold_full_checks = 0;
	u32 hold_vnow_cv_adjustments = 0;
	u32 hold_adjustments = 0;
	u32 hold_adjustments_up = 0;
	u32 hold_adjustments_down = 0;
	u32 hold_low_ibus_checks = 0;
	u32 hold_emergency_adjustments = 0;
	u32 hold_emergency_recoveries = 0;
	u32 hold_emergency_steps = 0;
	u32 hold_emergency_max_steps = 0;
	u32 hold_output_taper_adjustments = 0;
	u32 hold_primary_output_uv = 0;
	u32 hold_secondary_output_uv = 0;
	u32 hold_primary_output_max_uv = 0;
	u32 hold_secondary_output_max_uv = 0;
	u32 hold_path_drop_uv = 0;
	u32 hold_path_drop_max_uv = 0;
	u32 hold_records = 0;
	u32 monitor_checks = 0;
	u32 monitor_vbat_uv = 0;
	u32 monitor_vbat_ocv_uv = 0;
	u32 monitor_source_request_mv = 0;
	u32 monitor_primary_output_uv = 0;
	u32 monitor_secondary_output_uv = 0;
	u32 monitor_capacity = 0;
	u32 monitor_usb_online = 0;
	u32 post_stop_vbat_uv = 0;
	u32 post_stop_vbat_ocv_uv = 0;
	u32 ibus_rechecks = 0;
	u32 ibus_recoveries = 0;
	u32 hold_sample_no[BATTMGR_ONEPLUS_CP_DUAL_HOLD_RECORDS] = { 0 };
	u32 hold_request_mv[BATTMGR_ONEPLUS_CP_DUAL_HOLD_RECORDS] = { 0 };
	u32 hold_primary_ibus_ua[BATTMGR_ONEPLUS_CP_DUAL_HOLD_RECORDS] = { 0 };
	u32 hold_secondary_ibus_ua[BATTMGR_ONEPLUS_CP_DUAL_HOLD_RECORDS] = { 0 };
	u32 primary_hold_ibus_ua = 0;
	u32 secondary_hold_ibus_ua = 0;
	u32 aggregate_hold_ibus_ua;
	u32 hold_active_request_mv = 0;
	u32 hold_sample_request_mv = 0;
	u32 hold_max_request_mv = 0;
	u32 hold_next_request_mv;
	u32 source_from_mv = 0;
	u32 source_to_mv = 0;
	u32 source_boost_mv = 0;
	u32 entry_derate_from_mv = 0;
	u32 entry_derate_to_mv = 0;
	u32 secondary_loaded_vbus_uv = 0;
	u32 secondary_loaded_output_uv = 0;
	u32 secondary_loaded_target_uv = 0;
	u32 secondary_loaded_ratio_error_uv;
	bool primary_emergency;
	bool secondary_emergency;
	bool hold_full_confirmed = false;
	bool hold_vnow_cv_active = false;
	bool vnow_cv_reduce_pending = false;
	bool hold_output_limited = false;
	bool hold_output_taper_active = false;
	bool continuous;
	unsigned int i;
	int cleanup_ret;
	int monitor_temp_decic = 0;
	int monitor_ibat_ua = 0;
	int monitor_rc = 0;
	int post_stop_ibat_ua = 0;
	int post_stop_rc = -EINPROGRESS;
	int ret = 0;

	mutex_lock(&battmgr->lock);
	continuous = automatic && battmgr->oneplus_dual_auto_continuous;
	if ((automatic && (!battmgr->oneplus_dual_auto_enabled ||
			   !battmgr->oneplus_dual_auto_run)) ||
	    (!automatic && battmgr->oneplus_dual_auto_run)) {
		mutex_unlock(&battmgr->lock);
		return -ECANCELED;
	}
	if (battmgr->oneplus_dual_running ||
	    battmgr->oneplus_secondary_running ||
	    battmgr->oneplus_policy_running ||
	    battmgr->oneplus_pps_5m_running) {
		mutex_unlock(&battmgr->lock);
		return -EBUSY;
	}
	battmgr->oneplus_dual_running = true;
	battmgr->oneplus_dual_valid = false;
	battmgr->oneplus_dual_stop_requested = false;
	battmgr->oneplus_dual_continuous_run = continuous;
	battmgr->oneplus_dual_hold_full_confirmed = false;
	battmgr->oneplus_dual_hold_vnow_cv_active = false;
	battmgr->oneplus_dual_hold_output_limited = false;
	battmgr->oneplus_dual_hold_output_taper_active = false;
	battmgr->oneplus_dual_result = -EINPROGRESS;
	battmgr->oneplus_dual_ramp_rc = -EINPROGRESS;
	battmgr->oneplus_dual_precondition_5v_rc = -EINPROGRESS;
	battmgr->oneplus_dual_entry_derate_rc = -EINPROGRESS;
	battmgr->oneplus_dual_primary_entry_rc = -EINPROGRESS;
	battmgr->oneplus_dual_primary_pre_rc = -EINPROGRESS;
	battmgr->oneplus_dual_primary_post_rc = -EINPROGRESS;
	battmgr->oneplus_dual_secondary_post_rc = -EINPROGRESS;
	battmgr->oneplus_dual_primary_run_rc = -EINPROGRESS;
	battmgr->oneplus_dual_primary_cancel_rc = 0;
	battmgr->oneplus_dual_secondary_cancel_rc = 0;
	battmgr->oneplus_dual_source_step_rc = -EINPROGRESS;
	battmgr->oneplus_dual_primary_boost_pre_rc = -EINPROGRESS;
	battmgr->oneplus_dual_secondary_boost_pre_rc = -EINPROGRESS;
	battmgr->oneplus_dual_source_boost_rc = -EINPROGRESS;
	battmgr->oneplus_dual_primary_boost_post_rc = -EINPROGRESS;
	battmgr->oneplus_dual_secondary_boost_post_rc = -EINPROGRESS;
	battmgr->oneplus_dual_hold_rc = -EINPROGRESS;
	battmgr->oneplus_dual_source_reduce_rc = -EINPROGRESS;
	battmgr->oneplus_dual_secondary_loaded_check_rc = -EINPROGRESS;
	battmgr->oneplus_dual_secondary_target_rc = -EINPROGRESS;
	battmgr->oneplus_dual_secondary_run_rc = -EINPROGRESS;
	battmgr->oneplus_dual_primary_cleanup_rc = -EINPROGRESS;
	battmgr->oneplus_dual_secondary_cleanup_rc = -EINPROGRESS;
	battmgr->oneplus_dual_source_restore_rc = -EINPROGRESS;
	battmgr->oneplus_dual_pre_primary_off_rc = -EINPROGRESS;
	battmgr->oneplus_dual_pre_secondary_off_rc = -EINPROGRESS;
	battmgr->oneplus_dual_final_primary_off_rc = -EINPROGRESS;
	battmgr->oneplus_dual_final_secondary_off_rc = -EINPROGRESS;
	battmgr->oneplus_dual_primary_pre_ibus_ua = 0;
	battmgr->oneplus_dual_primary_post_ibus_ua = 0;
	battmgr->oneplus_dual_secondary_post_ibus_ua = 0;
	battmgr->oneplus_dual_primary_entry_ibus_ua = 0;
	battmgr->oneplus_dual_primary_boost_pre_ibus_ua = 0;
	battmgr->oneplus_dual_secondary_boost_pre_ibus_ua = 0;
	battmgr->oneplus_dual_primary_boost_post_ibus_ua = 0;
	battmgr->oneplus_dual_secondary_boost_post_ibus_ua = 0;
	battmgr->oneplus_dual_boost_attempts = 0;
	memset(battmgr->oneplus_dual_boost_request_mv, 0,
	       sizeof(battmgr->oneplus_dual_boost_request_mv));
	memset(battmgr->oneplus_dual_boost_primary_ibus_ua, 0,
	       sizeof(battmgr->oneplus_dual_boost_primary_ibus_ua));
	memset(battmgr->oneplus_dual_boost_secondary_ibus_ua, 0,
	       sizeof(battmgr->oneplus_dual_boost_secondary_ibus_ua));
	battmgr->oneplus_dual_hold_samples = 0;
	battmgr->oneplus_dual_hold_full_checks = 0;
	battmgr->oneplus_dual_hold_vnow_cv_adjustments = 0;
	battmgr->oneplus_dual_hold_adjustments = 0;
	battmgr->oneplus_dual_hold_adjustments_up = 0;
	battmgr->oneplus_dual_hold_adjustments_down = 0;
	battmgr->oneplus_dual_hold_low_ibus_checks = 0;
	battmgr->oneplus_dual_hold_emergency_adjustments = 0;
	battmgr->oneplus_dual_hold_emergency_recoveries = 0;
	battmgr->oneplus_dual_hold_emergency_steps = 0;
	battmgr->oneplus_dual_hold_emergency_max_steps = 0;
	battmgr->oneplus_dual_hold_output_taper_adjustments = 0;
	battmgr->oneplus_dual_hold_primary_output_uv = 0;
	battmgr->oneplus_dual_hold_secondary_output_uv = 0;
	battmgr->oneplus_dual_hold_primary_output_max_uv = 0;
	battmgr->oneplus_dual_hold_secondary_output_max_uv = 0;
	battmgr->oneplus_dual_hold_path_drop_uv = 0;
	battmgr->oneplus_dual_hold_path_drop_max_uv = 0;
	battmgr->oneplus_dual_hold_records = 0;
	battmgr->oneplus_dual_monitor_checks = 0;
	battmgr->oneplus_dual_monitor_vbat_uv = 0;
	battmgr->oneplus_dual_monitor_vbat_ocv_uv = 0;
	battmgr->oneplus_dual_monitor_source_request_mv = 0;
	battmgr->oneplus_dual_monitor_primary_output_uv = 0;
	battmgr->oneplus_dual_monitor_secondary_output_uv = 0;
	battmgr->oneplus_dual_monitor_capacity = 0;
	battmgr->oneplus_dual_monitor_usb_online = 0;
	battmgr->oneplus_dual_monitor_ibat_ua = 0;
	battmgr->oneplus_dual_monitor_temp_decic = 0;
	battmgr->oneplus_dual_monitor_rc = -EINPROGRESS;
	battmgr->oneplus_dual_post_stop_vbat_uv = 0;
	battmgr->oneplus_dual_post_stop_vbat_ocv_uv = 0;
	battmgr->oneplus_dual_post_stop_ibat_ua = 0;
	battmgr->oneplus_dual_post_stop_rc = -EINPROGRESS;
	battmgr->oneplus_dual_ibus_rechecks = 0;
	battmgr->oneplus_dual_ibus_recoveries = 0;
	memset(battmgr->oneplus_dual_hold_sample_no, 0,
	       sizeof(battmgr->oneplus_dual_hold_sample_no));
	memset(battmgr->oneplus_dual_hold_request_mv, 0,
	       sizeof(battmgr->oneplus_dual_hold_request_mv));
	memset(battmgr->oneplus_dual_hold_primary_ibus_ua, 0,
	       sizeof(battmgr->oneplus_dual_hold_primary_ibus_ua));
	memset(battmgr->oneplus_dual_hold_secondary_ibus_ua, 0,
	       sizeof(battmgr->oneplus_dual_hold_secondary_ibus_ua));
	battmgr->oneplus_dual_source_from_mv = 0;
	battmgr->oneplus_dual_source_to_mv = 0;
	battmgr->oneplus_dual_entry_derate_from_mv = 0;
	battmgr->oneplus_dual_entry_derate_to_mv = 0;
	battmgr->oneplus_dual_secondary_loaded_vbus_uv = 0;
	battmgr->oneplus_dual_secondary_loaded_output_uv = 0;
	battmgr->oneplus_dual_secondary_loaded_target_uv = 0;
	mutex_unlock(&battmgr->lock);

	/* Both pumps must be observably off before any source transition. */
	pre_primary_off_rc = qcom_battmgr_oneplus_require_cp_off(
		BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
	pre_secondary_off_rc = qcom_battmgr_oneplus_require_cp_off(
		BATTMGR_ONEPLUS_SECONDARY_CP_NAME);
	if (pre_primary_off_rc) {
		ret = pre_primary_off_rc;
		goto cleanup_cp;
	}
	if (pre_secondary_off_rc) {
		ret = pre_secondary_off_rc;
		goto cleanup_cp;
	}

	/* Make cold boots and immediate retries begin from one settled baseline. */
	mutex_lock(&battmgr->lock);
	precondition_5v_rc = qcom_battmgr_oneplus_fixed_5v_locked(battmgr);
	mutex_unlock(&battmgr->lock);
	if (precondition_5v_rc) {
		ret = precondition_5v_rc;
		goto cleanup_cp;
	}
	msleep(BATTMGR_ONEPLUS_CP_DUAL_IDLE_SETTLE_MS);
	pre_primary_off_rc = qcom_battmgr_oneplus_require_cp_off(
		BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
	pre_secondary_off_rc = qcom_battmgr_oneplus_require_cp_off(
		BATTMGR_ONEPLUS_SECONDARY_CP_NAME);
	if (pre_primary_off_rc || pre_secondary_off_rc) {
		ret = pre_primary_off_rc ? pre_primary_off_rc :
			pre_secondary_off_rc;
		goto cleanup_cp;
	}
	if (READ_ONCE(battmgr->oneplus_dual_stop_requested)) {
		ret = -ECANCELED;
		goto cleanup_cp;
	}

	/* Reject an unsafe battery/system state before raising the source. */
	monitor_checks++;
	monitor_rc = qcom_battmgr_oneplus_dual_system_check(
		battmgr, &monitor_usb_online, &monitor_vbat_uv,
		&monitor_vbat_ocv_uv, &monitor_ibat_ua, &monitor_capacity,
		&monitor_temp_decic);
	if (monitor_rc) {
		ret = monitor_rc;
		goto cleanup_cp;
	}
	if (monitor_vbat_uv > BATTMGR_ONEPLUS_POLICY_VBAT_MAX_UV) {
		ret = -ERANGE;
		goto cleanup_cp;
	}

	/* Prepare both pumps at low voltage, then perform one shared PPS ramp. */
	ramp_rc = qcom_battmgr_oneplus_ramp_2to1_run(
		battmgr, BATTMGR_ONEPLUS_PRIMARY_CP_NAME, false, false, false,
		true, BATTMGR_ONEPLUS_CP_DUAL_PPS_MA);
	if (ramp_rc) {
		ret = ramp_rc;
		goto cleanup_cp;
	}
	if (READ_ONCE(battmgr->oneplus_dual_stop_requested)) {
		ret = -ECANCELED;
		goto cleanup_cp;
	}

	/*
	 * The shared ramp deliberately proves the upper part of the vendor start
	 * window while both pumps are off.  At a gauge target boundary that can
	 * leave the first pump alone close to the independent 1.2-A ceiling.  Apply
	 * one vendor-sized downward control step before enabling it, then settle at
	 * that still-valid target-window voltage.  The later boost retains the same
	 * step count, so this can only lower the experiment's maximum request.
	 */
	mutex_lock(&battmgr->lock);
	entry_derate_from_mv = battmgr->oneplus_pps_source_request_mv;
	if (entry_derate_from_mv <
	    BATTMGR_ONEPLUS_TEST_PPS_MV +
		BATTMGR_ONEPLUS_CP_DUAL_ENTRY_DERATE_MV) {
		entry_derate_rc = -ERANGE;
	} else {
		entry_derate_to_mv = entry_derate_from_mv -
			BATTMGR_ONEPLUS_CP_DUAL_ENTRY_DERATE_MV;
		if (!qcom_battmgr_oneplus_has_pps(
		    battmgr, entry_derate_to_mv,
		    BATTMGR_ONEPLUS_CP_DUAL_PPS_MA)) {
			entry_derate_rc = -EOPNOTSUPP;
		} else {
			entry_derate_rc = qcom_battmgr_oneplus_set_pps_locked(
				battmgr, entry_derate_to_mv,
				BATTMGR_ONEPLUS_CP_DUAL_PPS_MA);
			if (!entry_derate_rc)
				battmgr->oneplus_pps_source_request_mv =
					entry_derate_to_mv;
		}
	}
	mutex_unlock(&battmgr->lock);
	if (entry_derate_rc) {
		ret = entry_derate_rc;
		goto cleanup_cp;
	}
	msleep(BATTMGR_ONEPLUS_CP_DUAL_ENTRY_SETTLE_MS);
	pre_primary_off_rc = qcom_battmgr_oneplus_require_cp_off(
		BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
	pre_secondary_off_rc = qcom_battmgr_oneplus_require_cp_off(
		BATTMGR_ONEPLUS_SECONDARY_CP_NAME);
	if (pre_primary_off_rc || pre_secondary_off_rc) {
		ret = pre_primary_off_rc ? pre_primary_off_rc :
			pre_secondary_off_rc;
		goto cleanup_cp;
	}

	primary_psy = power_supply_get_by_name(
		BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
	secondary_psy = power_supply_get_by_name(
		BATTMGR_ONEPLUS_SECONDARY_CP_NAME);
	if (!primary_psy || !secondary_psy) {
		ret = -ENODEV;
		goto cleanup_cp;
	}

	init_completion(&primary_ctx.done);
	primary_ctx.cp_psy = primary_psy;
	primary_ctx.run_ms = continuous ?
		BATTMGR_ONEPLUS_CP_POLICY_CONTINUOUS :
		BATTMGR_ONEPLUS_CP_DUAL_PRIMARY_RUN_MS;
	primary_ctx.result = -EINPROGRESS;
	primary_task = kthread_run(qcom_battmgr_oneplus_cp_run_thread,
		&primary_ctx, "oneplus-pps-primary");
	if (IS_ERR(primary_task)) {
		primary_run_rc = PTR_ERR(primary_task);
		ret = primary_run_rc;
		goto cleanup_cp;
	}
	primary_started = true;

	msleep(BATTMGR_ONEPLUS_CP_DUAL_PRIMARY_SETTLE_MS);
	primary_entry_rc = qcom_battmgr_oneplus_wait_cp_active(
		primary_psy, &primary_entry_ibus_ua,
		BATTMGR_ONEPLUS_CP_DUAL_IBUS_MAX_UA,
		BATTMGR_ONEPLUS_CP_DUAL_ENTRY_WAIT_MS,
		BATTMGR_ONEPLUS_CP_DUAL_ENTRY_WAIT_RETRIES,
		&ibus_rechecks, &ibus_recoveries);
	if (primary_entry_rc) {
		ret = primary_entry_rc;
		goto cancel_primary;
	}

	/*
	 * Stage 7B0 showed that starting the primary moves the shared bus from the
	 * unloaded vendor-start window into the normal 2:1 active window. Give the
	 * already-active primary one previously validated +20-mV source step, then
	 * characterize the secondary's loaded VBUS/VBAT pair. This does not raise
	 * the 1.75-A source-current cap.
	 */
	mutex_lock(&battmgr->lock);
	source_from_mv = battmgr->oneplus_pps_source_request_mv;
	if (!source_from_mv ||
	    source_from_mv > BATTMGR_ONEPLUS_CP_2TO1_SOURCE_MAX_MV -
			     BATTMGR_ONEPLUS_CP_DUAL_SOURCE_STEP_MV) {
		source_step_rc = -ERANGE;
	} else {
		source_to_mv = source_from_mv +
			BATTMGR_ONEPLUS_CP_DUAL_SOURCE_STEP_MV;
		source_step_rc = qcom_battmgr_oneplus_set_pps_locked(
			battmgr, source_to_mv, BATTMGR_ONEPLUS_CP_DUAL_PPS_MA);
		if (!source_step_rc)
			battmgr->oneplus_pps_source_request_mv = source_to_mv;
	}
	mutex_unlock(&battmgr->lock);
	if (source_step_rc) {
		ret = source_step_rc;
		goto cancel_primary;
	}

	msleep(BATTMGR_ONEPLUS_CP_DUAL_SOURCE_SETTLE_MS);
	primary_pre_rc = qcom_battmgr_oneplus_require_cp_active_bounded(
		primary_psy, &primary_pre_ibus_ua,
		BATTMGR_ONEPLUS_CP_DUAL_IBUS_MAX_UA,
		&ibus_rechecks, &ibus_recoveries);
	if (primary_pre_rc) {
		ret = primary_pre_rc;
		goto cancel_primary;
	}

	secondary_loaded_check_rc = qcom_battmgr_oneplus_read_cp_adc(
		secondary_psy, true, &secondary_loaded_vbus_uv,
		&secondary_loaded_output_uv);
	if (secondary_loaded_check_rc) {
		ret = secondary_loaded_check_rc;
		goto cancel_primary;
	}
	secondary_loaded_ratio_error_uv =
		secondary_loaded_vbus_uv >= secondary_loaded_output_uv * 2 ?
		secondary_loaded_vbus_uv - secondary_loaded_output_uv * 2 :
		secondary_loaded_output_uv * 2 - secondary_loaded_vbus_uv;
	if (secondary_loaded_vbus_uv <
			BATTMGR_ONEPLUS_CP_PREFLIGHT_VBUS_MIN_UV ||
	    secondary_loaded_vbus_uv > BATTMGR_ONEPLUS_CP_2TO1_SOURCE_MAX_MV *
					 1000 ||
	    secondary_loaded_output_uv < BATTMGR_ONEPLUS_CP_2TO1_VBAT_MIN_UV ||
	    secondary_loaded_output_uv > BATTMGR_ONEPLUS_CP_2TO1_VBAT_MAX_UV ||
	    secondary_loaded_ratio_error_uv >
			BATTMGR_ONEPLUS_CP_2TO1_START_BAND_UV) {
		secondary_loaded_check_rc = -ERANGE;
		ret = secondary_loaded_check_rc;
		goto cancel_primary;
	}

	/*
	 * The first pump already proved the unloaded vendor-start condition. The
	 * follower receives a narrow target 50 mV below its measured loaded VBUS;
	 * the physical driver still enforces the absolute 7.65-V preflight floor,
	 * 200-mV target window and its independent active 2:1 ratio limit.
	 */
	secondary_loaded_target_uv = max_t(u32,
		secondary_loaded_vbus_uv -
			BATTMGR_ONEPLUS_CP_LOADED_TARGET_MARGIN_UV,
		BATTMGR_ONEPLUS_CP_PREFLIGHT_VBUS_MIN_UV);
	secondary_target_rc = qcom_battmgr_oneplus_set_cp_start_target(
		secondary_psy, secondary_loaded_target_uv);
	if (secondary_target_rc) {
		ret = secondary_target_rc;
		goto cancel_primary;
	}

	/* Run both callbacks across the aggregate-IBUS control window. */
	init_completion(&secondary_ctx.done);
	secondary_ctx.cp_psy = secondary_psy;
	secondary_ctx.run_ms = continuous ?
		BATTMGR_ONEPLUS_CP_POLICY_CONTINUOUS :
		BATTMGR_ONEPLUS_CP_DUAL_SECONDARY_RUN_MS;
	secondary_ctx.result = -EINPROGRESS;
	secondary_task = kthread_run(qcom_battmgr_oneplus_cp_run_thread,
		&secondary_ctx, "oneplus-pps-secondary");
	if (IS_ERR(secondary_task)) {
		secondary_run_rc = PTR_ERR(secondary_task);
		ret = secondary_run_rc;
		goto cancel_primary;
	}
	secondary_started = true;

	msleep(BATTMGR_ONEPLUS_CP_DUAL_SECONDARY_SETTLE_MS);
	primary_boost_pre_rc =
		qcom_battmgr_oneplus_wait_cp_active(
			primary_psy, &primary_boost_pre_ibus_ua,
			BATTMGR_ONEPLUS_CP_DUAL_IBUS_MAX_UA,
			BATTMGR_ONEPLUS_CP_DUAL_ENTRY_WAIT_MS,
			BATTMGR_ONEPLUS_CP_DUAL_ENTRY_WAIT_RETRIES,
			&ibus_rechecks, &ibus_recoveries);
	secondary_boost_pre_rc =
		qcom_battmgr_oneplus_wait_cp_active(
			secondary_psy, &secondary_boost_pre_ibus_ua,
			BATTMGR_ONEPLUS_CP_DUAL_IBUS_MAX_UA,
			BATTMGR_ONEPLUS_CP_DUAL_ENTRY_WAIT_MS,
			BATTMGR_ONEPLUS_CP_DUAL_ENTRY_WAIT_RETRIES,
			&ibus_rechecks, &ibus_recoveries);
	if (primary_boost_pre_rc || secondary_boost_pre_rc) {
		ret = primary_boost_pre_rc ? primary_boost_pre_rc :
			secondary_boost_pre_rc;
		goto cancel_dual;
	}

	if (!source_to_mv ||
	    source_to_mv > BATTMGR_ONEPLUS_CP_2TO1_SOURCE_MAX_MV -
			   BATTMGR_ONEPLUS_CP_DUAL_BOOST_MAX_MV) {
		ret = -ERANGE;
		goto cancel_dual;
	}

	/*
	 * Follow the vendor third-party PPS policy shape: adjust source voltage in
	 * 40-mV increments against aggregate CP IBUS. Keep the fast acquisition
	 * bounded to ten samples and retain an independent 1.2-A ceiling on each
	 * pump. If both pumps remain balanced and finish narrowly below the target
	 * window, let the slower hold controller complete acquisition instead of
	 * treating the fixed ten-step boundary as an electrical failure.
	 */
	for (i = 0; i < BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEPS; i++) {
		if (READ_ONCE(battmgr->oneplus_dual_stop_requested)) {
			ret = -ECANCELED;
			break;
		}
		source_boost_mv = source_to_mv +
			(i + 1) * BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEP_MV;
		mutex_lock(&battmgr->lock);
		if (!qcom_battmgr_oneplus_has_pps(
		    battmgr, source_boost_mv,
		    BATTMGR_ONEPLUS_CP_DUAL_BOOST_PPS_MA))
			source_boost_rc = -EOPNOTSUPP;
		else
			source_boost_rc = qcom_battmgr_oneplus_set_pps_locked(
				battmgr, source_boost_mv,
				BATTMGR_ONEPLUS_CP_DUAL_BOOST_PPS_MA);
		mutex_unlock(&battmgr->lock);
		if (source_boost_rc) {
			ret = source_boost_rc;
			break;
		}

		msleep(BATTMGR_ONEPLUS_CP_DUAL_BOOST_MS);
		primary_boost_post_rc =
			qcom_battmgr_oneplus_require_cp_active_bounded(
				primary_psy, &primary_boost_post_ibus_ua,
				BATTMGR_ONEPLUS_CP_DUAL_IBUS_MAX_UA,
				&ibus_rechecks, &ibus_recoveries);
		secondary_boost_post_rc =
			qcom_battmgr_oneplus_require_cp_active_bounded(
				secondary_psy, &secondary_boost_post_ibus_ua,
				BATTMGR_ONEPLUS_CP_DUAL_IBUS_MAX_UA,
				&ibus_rechecks, &ibus_recoveries);

		boost_request_mv[i] = source_boost_mv;
		boost_primary_ibus_ua[i] = primary_boost_post_ibus_ua;
		boost_secondary_ibus_ua[i] = secondary_boost_post_ibus_ua;
		boost_attempts = i + 1;
		if (primary_boost_post_rc || secondary_boost_post_rc) {
			ret = primary_boost_post_rc ? primary_boost_post_rc :
				secondary_boost_post_rc;
			break;
		}

		aggregate_boost_ibus_ua = primary_boost_post_ibus_ua +
			secondary_boost_post_ibus_ua;
		if (aggregate_boost_ibus_ua >=
		    BATTMGR_ONEPLUS_CP_DUAL_BOOST_TARGET_UA -
		    BATTMGR_ONEPLUS_CP_DUAL_BOOST_DEADBAND_UA)
			break;
	}
	if (!ret && aggregate_boost_ibus_ua <
	    BATTMGR_ONEPLUS_CP_DUAL_BOOST_TARGET_UA -
	    BATTMGR_ONEPLUS_CP_DUAL_BOOST_DEADBAND_UA &&
	    (aggregate_boost_ibus_ua <
		BATTMGR_ONEPLUS_CP_DUAL_BOOST_HANDOFF_MIN_UA ||
	     primary_boost_post_ibus_ua <
		BATTMGR_ONEPLUS_CP_DUAL_BOOST_HANDOFF_PUMP_MIN_UA ||
	     secondary_boost_post_ibus_ua <
		BATTMGR_ONEPLUS_CP_DUAL_BOOST_HANDOFF_PUMP_MIN_UA))
		ret = -ERANGE;

	/*
	 * B29 completed 116 hold samples before slow current drift put one pump only
	 * 3.75 mA above the independent 1.2-A stop.  Match both arms of the vendor
	 * third-party current loop, but avoid using its full 40-mV target-voltage
	 * step as an instantaneous low-current correction.  After a sustained low
	 * indication, raise only 20 mV; lower 40 mV above the aggregate deadband.
	 * A 1.05-A per-pump soft edge requests the downward
	 * step before either physical worker reaches its unchanged 1.2-A hard stop.
	 * If the coordinator nevertheless confirms 1.20--1.25 A while the pump is
	 * still active, issue the vendor over-current branch's immediate 100-mV
	 * reduction.  Permit at most three consecutive reductions while every
	 * over-current sample remains inside that narrow recovery envelope; this
	 * matches the vendor policy's bounded multi-sample response without
	 * weakening the physical ceiling.  The 1.25-A edge is rounded only to the
	 * next representable 1.875-mA ADC code; that code remains an emergency
	 * reduction, while the following code is immediately fatal.
	 * Vendor-current physical runs use the same quantized terminal comparison
	 * so they do not turn one pump off before this branch can reduce the source;
	 * the coordinator still reacts at 1.2 A, and anything outside the narrow
	 * recovery envelope remains fatal.  The bounded run also checks each pump's
	 * role-correct local output every 500 ms.  Primary VBAT retains the
	 * 4.30/4.35-V cell limits.  Secondary VOUT is upstream of the measured path
	 * drop, so it uses 4.45/4.50-V limits plus an independent 150/200-mV delta
	 * check against primary VBAT.  A soft crossing latches voltage-taper mode,
	 * lowers PPS by 100 mV and no longer permits the normal low-current arm to
	 * raise PPS again. USB presence, vendor gauge VOLTAGE_NOW, battery current,
	 * SOC and temperature are checked every two seconds. OCV is sampled only as
	 * diagnostic telemetry. At 4.30 V, the loaded terminal enters a hysteretic
	 * constant-voltage phase: 40-mV source reductions continue while at or above
	 * that edge, and the normal upward arm remains disabled until terminal
	 * voltage falls to 4.275 V. This regulates rather than declares full. Like
	 * the original low-current-full path, completion requires voltage and low
	 * charging current for consecutive checks; the SOC floor substitutes for the
	 * original stack's final-gear state. Every adjustment must be confirmed by a
	 * later sample; an uncorrectable sample or system guard fails closed.
	 */
	if (!ret) {
		hold_rc = 0;
		hold_active_request_mv = source_boost_mv;
		/*
		 * The vendor third-party loop's independent voltage target uses 40-mV
		 * steps as battery voltage rises, while its low-current branch holds.
		 * Our constrained experiment keeps the validated current target but uses
		 * a finer low-current step.  Retain the advertised-APDO check on every
		 * write and the existing 9.4-V cap.
		 */
		hold_max_request_mv = BATTMGR_ONEPLUS_CP_2TO1_SOURCE_MAX_MV;
		for (i = 0;
		     continuous || i < BATTMGR_ONEPLUS_CP_DUAL_HOLD_SAMPLES;
		     i++) {
			vnow_cv_reduce_pending = false;
			if (READ_ONCE(battmgr->oneplus_dual_stop_requested)) {
				hold_rc = -ECANCELED;
				ret = hold_rc;
				break;
			}

			hold_sample_request_mv = hold_active_request_mv;
			msleep(BATTMGR_ONEPLUS_CP_DUAL_HOLD_SAMPLE_MS);
			/*
			 * The stop store sets this flag before cancelling both physical
			 * pumps.  Recheck after sleeping so an intentional cancellation
			 * is not observed first as a pump-active -EIO failure.
			 */
			if (READ_ONCE(battmgr->oneplus_dual_stop_requested)) {
				hold_rc = -ECANCELED;
				ret = hold_rc;
				break;
			}
			primary_hold_check_rc =
				qcom_battmgr_oneplus_require_cp_active_bounded(
					primary_psy, &primary_hold_ibus_ua,
					BATTMGR_ONEPLUS_CP_DUAL_IBUS_MAX_UA,
					&ibus_rechecks, &ibus_recoveries);
			secondary_hold_check_rc =
				qcom_battmgr_oneplus_require_cp_active_bounded(
					secondary_psy, &secondary_hold_ibus_ua,
					BATTMGR_ONEPLUS_CP_DUAL_IBUS_MAX_UA,
					&ibus_rechecks, &ibus_recoveries);

			hold_samples = i + 1;
			primary_emergency = primary_hold_check_rc == -ERANGE &&
				primary_hold_ibus_ua >
					BATTMGR_ONEPLUS_CP_DUAL_IBUS_MAX_UA &&
				primary_hold_ibus_ua <=
					BATTMGR_ONEPLUS_CP_DUAL_EMERGENCY_IBUS_ADC_MAX_UA;
			secondary_emergency = secondary_hold_check_rc == -ERANGE &&
				secondary_hold_ibus_ua >
					BATTMGR_ONEPLUS_CP_DUAL_IBUS_MAX_UA &&
				secondary_hold_ibus_ua <=
					BATTMGR_ONEPLUS_CP_DUAL_EMERGENCY_IBUS_ADC_MAX_UA;
			if (primary_hold_check_rc || secondary_hold_check_rc) {
				if (!(primary_emergency || !primary_hold_check_rc) ||
				    !(secondary_emergency || !secondary_hold_check_rc) ||
				    !(primary_emergency || secondary_emergency) ||
				    hold_emergency_steps >=
					BATTMGR_ONEPLUS_CP_DUAL_EMERGENCY_MAX_STEPS) {
					hold_rc = primary_hold_check_rc ?
						primary_hold_check_rc :
						secondary_hold_check_rc;
					ret = hold_rc;
					goto record_hold_sample;
				}
				if ((!continuous &&
				     i + 1 == BATTMGR_ONEPLUS_CP_DUAL_HOLD_SAMPLES) ||
				    hold_active_request_mv < source_to_mv +
					BATTMGR_ONEPLUS_CP_DUAL_EMERGENCY_STEP_MV) {
					hold_rc = -ERANGE;
					ret = hold_rc;
					goto record_hold_sample;
				}

				hold_low_ibus_checks = 0;
				hold_next_request_mv = hold_active_request_mv -
					BATTMGR_ONEPLUS_CP_DUAL_EMERGENCY_STEP_MV;
				mutex_lock(&battmgr->lock);
				if (!qcom_battmgr_oneplus_has_pps(
				    battmgr, hold_next_request_mv,
				    BATTMGR_ONEPLUS_CP_DUAL_BOOST_PPS_MA))
					hold_rc = -EOPNOTSUPP;
				else
					hold_rc =
						qcom_battmgr_oneplus_set_pps_locked(
							battmgr,
							hold_next_request_mv,
							BATTMGR_ONEPLUS_CP_DUAL_BOOST_PPS_MA);
				mutex_unlock(&battmgr->lock);
				if (hold_rc) {
					ret = hold_rc;
					goto record_hold_sample;
				}
				hold_active_request_mv = hold_next_request_mv;
				hold_adjustments++;
				hold_adjustments_down++;
				hold_emergency_adjustments++;
				hold_emergency_steps++;
				hold_emergency_max_steps = max(
					hold_emergency_max_steps,
					hold_emergency_steps);
				if (!(i %
				      BATTMGR_ONEPLUS_CP_DUAL_MONITOR_STRIDE)) {
					monitor_checks++;
					monitor_rc =
						qcom_battmgr_oneplus_dual_system_check(
							battmgr,
							&monitor_usb_online,
							&monitor_vbat_uv,
							&monitor_vbat_ocv_uv,
							&monitor_ibat_ua,
							&monitor_capacity,
							&monitor_temp_decic);
					if (monitor_rc) {
						hold_rc = monitor_rc;
						ret = hold_rc;
					} else {
						/*
						 * The emergency write precedes this gauge sample,
						 * while the local-output values belong to the
						 * previous request.  Do not publish them as one
						 * synchronized voltage snapshot.
						 */
						monitor_source_request_mv = 0;
						monitor_primary_output_uv = 0;
						monitor_secondary_output_uv = 0;
						if (qcom_battmgr_oneplus_dual_full_check(
						    monitor_vbat_uv, monitor_ibat_ua,
						    monitor_capacity,
						    &hold_full_checks))
							hold_full_confirmed = true;
						if (monitor_vbat_uv >=
						    BATTMGR_ONEPLUS_CP_DUAL_VNOW_CV_ENTER_UV)
							hold_vnow_cv_active = true;
						else if (monitor_vbat_uv <=
							 BATTMGR_ONEPLUS_CP_DUAL_VNOW_CV_RELEASE_UV)
							hold_vnow_cv_active = false;
					}
				}
				goto record_hold_sample;
			}
			if (hold_emergency_steps) {
				hold_emergency_recoveries++;
				hold_emergency_steps = 0;
			}

			hold_rc = qcom_battmgr_oneplus_read_cp_output(
				primary_psy, false, &hold_primary_output_uv);
			if (!hold_rc)
				hold_rc = qcom_battmgr_oneplus_read_cp_output(
					secondary_psy, true,
					&hold_secondary_output_uv);
			if (hold_rc) {
				ret = hold_rc;
				goto record_hold_sample;
			}
			hold_primary_output_max_uv = max(hold_primary_output_max_uv,
							 hold_primary_output_uv);
			hold_secondary_output_max_uv = max(
				hold_secondary_output_max_uv,
				hold_secondary_output_uv);
			hold_path_drop_uv = hold_secondary_output_uv >
				hold_primary_output_uv ?
				hold_secondary_output_uv - hold_primary_output_uv : 0;
			hold_path_drop_max_uv = max(hold_path_drop_max_uv,
						 hold_path_drop_uv);
			if (hold_primary_output_uv >
					BATTMGR_ONEPLUS_CP_2TO1_VBAT_MAX_UV ||
			    hold_secondary_output_uv >
					BATTMGR_ONEPLUS_CP_DUAL_SECONDARY_MAX_UV ||
			    hold_path_drop_uv >
					BATTMGR_ONEPLUS_CP_DUAL_PATH_MAX_UV) {
				hold_rc = -ERANGE;
				ret = hold_rc;
				goto record_hold_sample;
			}

			if (!(i % BATTMGR_ONEPLUS_CP_DUAL_MONITOR_STRIDE)) {
				monitor_checks++;
				monitor_rc = qcom_battmgr_oneplus_dual_system_check(
					battmgr, &monitor_usb_online,
					&monitor_vbat_uv, &monitor_vbat_ocv_uv,
					&monitor_ibat_ua, &monitor_capacity,
					&monitor_temp_decic);
				if (monitor_rc) {
					hold_rc = monitor_rc;
					ret = hold_rc;
					goto record_hold_sample;
				}
				/*
				 * These five values belong to the same settled
				 * control iteration.  Preserve their provenance so
				 * loaded gauge voltage is not mistaken for either a
				 * charge-pump ADC or the best-effort OCV estimate.
				 */
				monitor_source_request_mv = hold_sample_request_mv;
				monitor_primary_output_uv = hold_primary_output_uv;
				monitor_secondary_output_uv =
					hold_secondary_output_uv;
				if (qcom_battmgr_oneplus_dual_full_check(
				    monitor_vbat_uv, monitor_ibat_ua,
				    monitor_capacity, &hold_full_checks)) {
					hold_full_confirmed = true;
					goto record_hold_sample;
				}
				if (monitor_vbat_uv >=
				    BATTMGR_ONEPLUS_CP_DUAL_VNOW_CV_ENTER_UV)
					hold_vnow_cv_active = true;
				else if (monitor_vbat_uv <=
					 BATTMGR_ONEPLUS_CP_DUAL_VNOW_CV_RELEASE_UV)
					hold_vnow_cv_active = false;
				vnow_cv_reduce_pending = hold_vnow_cv_active &&
					monitor_vbat_uv >=
					BATTMGR_ONEPLUS_CP_DUAL_VNOW_CV_ENTER_UV &&
					!hold_full_checks;
			}

			aggregate_hold_ibus_ua = primary_hold_ibus_ua +
				secondary_hold_ibus_ua;
			if (hold_primary_output_uv >=
					BATTMGR_ONEPLUS_CP_DUAL_PRIMARY_TAPER_UV ||
			    hold_secondary_output_uv >=
					BATTMGR_ONEPLUS_CP_DUAL_SECONDARY_TAPER_UV ||
			    hold_path_drop_uv >=
					BATTMGR_ONEPLUS_CP_DUAL_PATH_TAPER_UV) {
				hold_output_taper_active = true;
				hold_low_ibus_checks = 0;
				if ((!continuous &&
				     i + 1 == BATTMGR_ONEPLUS_CP_DUAL_HOLD_SAMPLES) ||
				    hold_active_request_mv < source_to_mv +
					BATTMGR_ONEPLUS_CP_DUAL_OUTPUT_TAPER_STEP_MV) {
					hold_output_limited = true;
					goto record_hold_sample;
				}

				hold_next_request_mv = hold_active_request_mv -
					BATTMGR_ONEPLUS_CP_DUAL_OUTPUT_TAPER_STEP_MV;
				mutex_lock(&battmgr->lock);
				if (!qcom_battmgr_oneplus_has_pps(
				    battmgr, hold_next_request_mv,
				    BATTMGR_ONEPLUS_CP_DUAL_BOOST_PPS_MA))
					hold_rc = -EOPNOTSUPP;
				else
					hold_rc =
						qcom_battmgr_oneplus_set_pps_locked(
							battmgr,
							hold_next_request_mv,
							BATTMGR_ONEPLUS_CP_DUAL_BOOST_PPS_MA);
				mutex_unlock(&battmgr->lock);
				if (hold_rc) {
					ret = hold_rc;
					goto record_hold_sample;
				}
				hold_active_request_mv = hold_next_request_mv;
				hold_adjustments++;
				hold_adjustments_down++;
				hold_output_taper_adjustments++;
				goto record_hold_sample;
			}
			if (vnow_cv_reduce_pending) {
				hold_low_ibus_checks = 0;
				if (!continuous &&
				    i + 1 == BATTMGR_ONEPLUS_CP_DUAL_HOLD_SAMPLES)
					goto record_hold_sample;
				if (hold_active_request_mv < source_to_mv +
				    BATTMGR_ONEPLUS_CP_DUAL_VNOW_CV_STEP_MV) {
					hold_output_limited = true;
					goto record_hold_sample;
				}

				hold_next_request_mv = hold_active_request_mv -
					BATTMGR_ONEPLUS_CP_DUAL_VNOW_CV_STEP_MV;
				mutex_lock(&battmgr->lock);
				if (!qcom_battmgr_oneplus_has_pps(
				    battmgr, hold_next_request_mv,
				    BATTMGR_ONEPLUS_CP_DUAL_BOOST_PPS_MA))
					hold_rc = -EOPNOTSUPP;
				else
					hold_rc =
						qcom_battmgr_oneplus_set_pps_locked(
							battmgr,
							hold_next_request_mv,
							BATTMGR_ONEPLUS_CP_DUAL_BOOST_PPS_MA);
				mutex_unlock(&battmgr->lock);
				if (hold_rc) {
					ret = hold_rc;
					goto record_hold_sample;
				}
				hold_active_request_mv = hold_next_request_mv;
				hold_adjustments++;
				hold_adjustments_down++;
				hold_vnow_cv_adjustments++;
				goto record_hold_sample;
			}
			if (primary_hold_ibus_ua >=
				    BATTMGR_ONEPLUS_CP_DUAL_HOLD_SOFT_IBUS_UA ||
			    secondary_hold_ibus_ua >=
				    BATTMGR_ONEPLUS_CP_DUAL_HOLD_SOFT_IBUS_UA ||
			    aggregate_hold_ibus_ua >
				    BATTMGR_ONEPLUS_CP_DUAL_BOOST_TARGET_UA +
				    BATTMGR_ONEPLUS_CP_DUAL_BOOST_DEADBAND_UA) {
				hold_low_ibus_checks = 0;
				if (!continuous &&
				    i + 1 == BATTMGR_ONEPLUS_CP_DUAL_HOLD_SAMPLES)
					goto record_hold_sample;
				if (hold_active_request_mv < source_to_mv +
					BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEP_MV) {
					hold_rc = -ERANGE;
					ret = hold_rc;
					goto record_hold_sample;
				}

				hold_next_request_mv = hold_active_request_mv -
					BATTMGR_ONEPLUS_CP_DUAL_BOOST_STEP_MV;
				mutex_lock(&battmgr->lock);
				if (!qcom_battmgr_oneplus_has_pps(
				    battmgr, hold_next_request_mv,
				    BATTMGR_ONEPLUS_CP_DUAL_BOOST_PPS_MA))
					hold_rc = -EOPNOTSUPP;
				else
					hold_rc =
						qcom_battmgr_oneplus_set_pps_locked(
							battmgr,
							hold_next_request_mv,
							BATTMGR_ONEPLUS_CP_DUAL_BOOST_PPS_MA);
				mutex_unlock(&battmgr->lock);
				if (hold_rc) {
					ret = hold_rc;
					goto record_hold_sample;
				}
				hold_active_request_mv = hold_next_request_mv;
				hold_adjustments++;
				hold_adjustments_down++;
				goto record_hold_sample;
			}
			if (!hold_output_taper_active &&
			    !hold_vnow_cv_active &&
			    aggregate_hold_ibus_ua <
			    BATTMGR_ONEPLUS_CP_DUAL_BOOST_TARGET_UA -
			    BATTMGR_ONEPLUS_CP_DUAL_BOOST_DEADBAND_UA) {
				hold_low_ibus_checks++;
				if ((!continuous &&
				     i + 1 == BATTMGR_ONEPLUS_CP_DUAL_HOLD_SAMPLES) ||
				    hold_low_ibus_checks <
				    BATTMGR_ONEPLUS_CP_DUAL_LOW_IBUS_CONFIRM_SAMPLES)
					goto record_hold_sample;
				hold_low_ibus_checks = 0;
				if (hold_active_request_mv > hold_max_request_mv -
					BATTMGR_ONEPLUS_CP_DUAL_LOW_IBUS_STEP_MV) {
					hold_rc = -ERANGE;
					ret = hold_rc;
					goto record_hold_sample;
				}

				hold_next_request_mv = hold_active_request_mv +
					BATTMGR_ONEPLUS_CP_DUAL_LOW_IBUS_STEP_MV;
				mutex_lock(&battmgr->lock);
				if (!qcom_battmgr_oneplus_has_pps(
				    battmgr, hold_next_request_mv,
				    BATTMGR_ONEPLUS_CP_DUAL_BOOST_PPS_MA))
					hold_rc = -EOPNOTSUPP;
				else
					hold_rc =
						qcom_battmgr_oneplus_set_pps_locked(
							battmgr,
							hold_next_request_mv,
							BATTMGR_ONEPLUS_CP_DUAL_BOOST_PPS_MA);
				mutex_unlock(&battmgr->lock);
				if (hold_rc) {
					ret = hold_rc;
					goto record_hold_sample;
				}
				hold_active_request_mv = hold_next_request_mv;
				hold_adjustments++;
				hold_adjustments_up++;
			} else {
				hold_low_ibus_checks = 0;
			}

record_hold_sample:
			if (!i ||
			    (!continuous &&
			     i + 1 == BATTMGR_ONEPLUS_CP_DUAL_HOLD_SAMPLES) ||
			    !((i + 1) %
			      BATTMGR_ONEPLUS_CP_DUAL_HOLD_TRACE_STRIDE) || ret ||
			    hold_output_limited || hold_full_confirmed) {
				if (hold_records ==
				    BATTMGR_ONEPLUS_CP_DUAL_HOLD_RECORDS) {
					memmove(&hold_sample_no[0], &hold_sample_no[1],
						(BATTMGR_ONEPLUS_CP_DUAL_HOLD_RECORDS - 1) *
						sizeof(hold_sample_no[0]));
					memmove(&hold_request_mv[0], &hold_request_mv[1],
						(BATTMGR_ONEPLUS_CP_DUAL_HOLD_RECORDS - 1) *
						sizeof(hold_request_mv[0]));
					memmove(&hold_primary_ibus_ua[0],
						&hold_primary_ibus_ua[1],
						(BATTMGR_ONEPLUS_CP_DUAL_HOLD_RECORDS - 1) *
						sizeof(hold_primary_ibus_ua[0]));
					memmove(&hold_secondary_ibus_ua[0],
						&hold_secondary_ibus_ua[1],
						(BATTMGR_ONEPLUS_CP_DUAL_HOLD_RECORDS - 1) *
						sizeof(hold_secondary_ibus_ua[0]));
					hold_records--;
				}
				hold_sample_no[hold_records] = i + 1;
				hold_request_mv[hold_records] =
					hold_sample_request_mv;
				hold_primary_ibus_ua[hold_records] =
					primary_hold_ibus_ua;
				hold_secondary_ibus_ua[hold_records] =
					secondary_hold_ibus_ua;
				hold_records++;
			}
			if (ret || hold_output_limited || hold_full_confirmed)
				break;
		}
	}

	if (ret)
		goto cancel_dual;
	source_reduce_rc = 0;

	/*
	 * Keep the last stable source request while stopping both pumps.  On normal
	 * duration completion, prove both pumps healthy before arming cancellation.
	 * If local-output taper exhausted the safe source floor, or the gauge NOW /
	 * low-current / SOC confirmation completed, do not spend two more serial ADC
	 * transactions near the voltage boundary: arm both cancellation flags
	 * immediately. Each worker performs its normal fail-closed disable and
	 * profile restore; only after both return may cleanup request fixed 5 V.
	 */
	if (hold_output_limited || hold_full_confirmed) {
		primary_post_rc = 0;
		secondary_post_rc = 0;
	} else {
		primary_post_rc = qcom_battmgr_oneplus_require_cp_active_bounded(
			primary_psy, &primary_post_ibus_ua,
			BATTMGR_ONEPLUS_CP_DUAL_IBUS_MAX_UA,
			&ibus_rechecks, &ibus_recoveries);
		secondary_post_rc = qcom_battmgr_oneplus_require_cp_active_bounded(
			secondary_psy, &secondary_post_ibus_ua,
			BATTMGR_ONEPLUS_CP_DUAL_IBUS_MAX_UA,
			&ibus_rechecks, &ibus_recoveries);
		if (primary_post_rc || secondary_post_rc) {
			ret = primary_post_rc ? primary_post_rc : secondary_post_rc;
			goto cancel_dual;
		}
	}

	primary_cancel_rc = qcom_battmgr_oneplus_cancel_cp(primary_psy);
	secondary_cancel_rc = qcom_battmgr_oneplus_cancel_cp(secondary_psy);
	wait_for_completion(&primary_ctx.done);
	wait_for_completion(&secondary_ctx.done);
	primary_started = false;
	secondary_started = false;
	primary_run_rc = primary_ctx.result;
	secondary_run_rc = secondary_ctx.result;
	if (!primary_cancel_rc && primary_run_rc == -ECANCELED)
		primary_run_rc = 0;
	if (!secondary_cancel_rc && secondary_run_rc == -ECANCELED)
		secondary_run_rc = 0;
	if (primary_cancel_rc)
		ret = primary_cancel_rc;
	else if (secondary_cancel_rc)
		ret = secondary_cancel_rc;
	else if (primary_run_rc)
		ret = primary_run_rc;
	else
		ret = secondary_run_rc;
	goto cleanup_cp;

cancel_dual:
cancel_primary:
	/* Preserve the same simultaneous-stop ordering on every failure path. */
	if (primary_started)
		primary_cancel_rc = qcom_battmgr_oneplus_cancel_cp(primary_psy);
	if (secondary_started)
		secondary_cancel_rc = qcom_battmgr_oneplus_cancel_cp(
			secondary_psy);
	if (primary_started) {
		wait_for_completion(&primary_ctx.done);
		primary_started = false;
		primary_run_rc = primary_ctx.result;
		if (!ret && primary_cancel_rc)
			ret = primary_cancel_rc;
	}
	if (secondary_started) {
		wait_for_completion(&secondary_ctx.done);
		secondary_started = false;
		secondary_run_rc = secondary_ctx.result;
		if (!ret && secondary_cancel_rc)
			ret = secondary_cancel_rc;
	}

cleanup_cp:
	/*
	 * A manual stop may race any in-flight pump/status transaction after the
	 * stop store has already cancelled both workers.  Preserve phase-specific
	 * cleanup diagnostics, but attribute the requested interruption itself to
	 * -ECANCELED instead of whichever transaction first observed the teardown.
	 */
	if (READ_ONCE(battmgr->oneplus_dual_stop_requested)) {
		if ((continuous ||
		     hold_samples < BATTMGR_ONEPLUS_CP_DUAL_HOLD_SAMPLES) &&
		    !hold_output_limited && !hold_full_confirmed)
			hold_rc = -ECANCELED;
		ret = -ECANCELED;
	}

	/* A started task owns its power_supply until the callback returns. */
	if (secondary_started) {
		secondary_cancel_rc = qcom_battmgr_oneplus_cancel_cp(
			secondary_psy);
		wait_for_completion(&secondary_ctx.done);
		secondary_run_rc = secondary_ctx.result;
		secondary_started = false;
		if (!ret && secondary_cancel_rc)
			ret = secondary_cancel_rc;
	}
	if (primary_started) {
		primary_cancel_rc = qcom_battmgr_oneplus_cancel_cp(primary_psy);
		wait_for_completion(&primary_ctx.done);
		primary_run_rc = primary_ctx.result;
		primary_started = false;
		if (!ret && primary_cancel_rc)
			ret = primary_cancel_rc;
	}

	if (!secondary_psy)
		secondary_psy = power_supply_get_by_name(
			BATTMGR_ONEPLUS_SECONDARY_CP_NAME);
	if (secondary_psy) {
		secondary_cleanup_rc = qcom_battmgr_oneplus_set_cp_prepared(
			secondary_psy, false);
		cleanup_ret = qcom_battmgr_oneplus_set_cp_start_target(
			secondary_psy, 0);
		if (!secondary_cleanup_rc)
			secondary_cleanup_rc = cleanup_ret;
		if (!ret && secondary_cleanup_rc)
			ret = secondary_cleanup_rc;
	} else {
		secondary_cleanup_rc = -ENODEV;
		if (!ret)
			ret = secondary_cleanup_rc;
	}

	if (!primary_psy)
		primary_psy = power_supply_get_by_name(
			BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
	if (primary_psy) {
		primary_cleanup_rc = qcom_battmgr_oneplus_set_cp_prepared(
			primary_psy, false);
		cleanup_ret = qcom_battmgr_oneplus_set_cp_start_target(
			primary_psy, 0);
		if (!primary_cleanup_rc)
			primary_cleanup_rc = cleanup_ret;
		if (!ret && primary_cleanup_rc)
			ret = primary_cleanup_rc;
	} else {
		primary_cleanup_rc = -ENODEV;
		if (!ret)
			ret = primary_cleanup_rc;
	}

	if (secondary_psy)
		power_supply_put(secondary_psy);
	if (primary_psy)
		power_supply_put(primary_psy);

	mutex_lock(&battmgr->lock);
	source_restore_rc = qcom_battmgr_oneplus_fixed_5v_locked(battmgr);
	if (!primary_cleanup_rc)
		battmgr->oneplus_pps_cp_prepared = false;
	mutex_unlock(&battmgr->lock);
	if (!ret && source_restore_rc)
		ret = source_restore_rc;

	/* Success also requires both physical pumps to be off after fallback. */
	final_primary_off_rc = qcom_battmgr_oneplus_require_cp_off(
		BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
	final_secondary_off_rc = qcom_battmgr_oneplus_require_cp_off(
		BATTMGR_ONEPLUS_SECONDARY_CP_NAME);
	if (!ret && final_primary_off_rc)
		ret = final_primary_off_rc;
	if (!ret && final_secondary_off_rc)
		ret = final_secondary_off_rc;

	/*
	 * Capture one non-CP sample after fixed-5-V fallback. This remains
	 * diagnostic and never changes the completed run result. Read the battery
	 * directly because USB may already be offline during fixed-5-V fallback.
	 */
	if (!source_restore_rc && !final_primary_off_rc &&
	    !final_secondary_off_rc) {
		msleep(BATTMGR_ONEPLUS_CP_DUAL_POST_STOP_SETTLE_MS);
		post_stop_rc = qcom_battmgr_oneplus_dual_battery_sample(
			battmgr, &post_stop_vbat_uv, &post_stop_vbat_ocv_uv,
			&post_stop_ibat_ua);
	}

	mutex_lock(&battmgr->lock);
	battmgr->oneplus_dual_ramp_rc = ramp_rc;
	battmgr->oneplus_dual_precondition_5v_rc = precondition_5v_rc;
	battmgr->oneplus_dual_entry_derate_rc = entry_derate_rc;
	battmgr->oneplus_dual_primary_entry_rc = primary_entry_rc;
	battmgr->oneplus_dual_primary_pre_rc = primary_pre_rc;
	battmgr->oneplus_dual_primary_post_rc = primary_post_rc;
	battmgr->oneplus_dual_secondary_post_rc = secondary_post_rc;
	battmgr->oneplus_dual_primary_run_rc = primary_run_rc;
	battmgr->oneplus_dual_primary_cancel_rc = primary_cancel_rc;
	battmgr->oneplus_dual_secondary_cancel_rc = secondary_cancel_rc;
	battmgr->oneplus_dual_source_step_rc = source_step_rc;
	battmgr->oneplus_dual_primary_boost_pre_rc = primary_boost_pre_rc;
	battmgr->oneplus_dual_secondary_boost_pre_rc = secondary_boost_pre_rc;
	battmgr->oneplus_dual_source_boost_rc = source_boost_rc;
	battmgr->oneplus_dual_primary_boost_post_rc = primary_boost_post_rc;
	battmgr->oneplus_dual_secondary_boost_post_rc =
		secondary_boost_post_rc;
	battmgr->oneplus_dual_hold_rc = hold_rc;
	battmgr->oneplus_dual_source_reduce_rc = source_reduce_rc;
	battmgr->oneplus_dual_secondary_loaded_check_rc =
		secondary_loaded_check_rc;
	battmgr->oneplus_dual_secondary_target_rc = secondary_target_rc;
	battmgr->oneplus_dual_secondary_run_rc = secondary_run_rc;
	battmgr->oneplus_dual_primary_cleanup_rc = primary_cleanup_rc;
	battmgr->oneplus_dual_secondary_cleanup_rc = secondary_cleanup_rc;
	battmgr->oneplus_dual_source_restore_rc = source_restore_rc;
	battmgr->oneplus_dual_pre_primary_off_rc = pre_primary_off_rc;
	battmgr->oneplus_dual_pre_secondary_off_rc = pre_secondary_off_rc;
	battmgr->oneplus_dual_final_primary_off_rc = final_primary_off_rc;
	battmgr->oneplus_dual_final_secondary_off_rc =
		final_secondary_off_rc;
	battmgr->oneplus_dual_primary_pre_ibus_ua = primary_pre_ibus_ua;
	battmgr->oneplus_dual_primary_post_ibus_ua = primary_post_ibus_ua;
	battmgr->oneplus_dual_secondary_post_ibus_ua =
		secondary_post_ibus_ua;
	battmgr->oneplus_dual_primary_entry_ibus_ua = primary_entry_ibus_ua;
	battmgr->oneplus_dual_primary_boost_pre_ibus_ua =
		primary_boost_pre_ibus_ua;
	battmgr->oneplus_dual_secondary_boost_pre_ibus_ua =
		secondary_boost_pre_ibus_ua;
	battmgr->oneplus_dual_primary_boost_post_ibus_ua =
		primary_boost_post_ibus_ua;
	battmgr->oneplus_dual_secondary_boost_post_ibus_ua =
		secondary_boost_post_ibus_ua;
	battmgr->oneplus_dual_boost_attempts = boost_attempts;
	memcpy(battmgr->oneplus_dual_boost_request_mv, boost_request_mv,
	       sizeof(boost_request_mv));
	memcpy(battmgr->oneplus_dual_boost_primary_ibus_ua,
	       boost_primary_ibus_ua, sizeof(boost_primary_ibus_ua));
	memcpy(battmgr->oneplus_dual_boost_secondary_ibus_ua,
	       boost_secondary_ibus_ua, sizeof(boost_secondary_ibus_ua));
	battmgr->oneplus_dual_hold_samples = hold_samples;
	battmgr->oneplus_dual_hold_full_confirmed = hold_full_confirmed;
	battmgr->oneplus_dual_hold_full_checks = hold_full_checks;
	battmgr->oneplus_dual_hold_vnow_cv_active = hold_vnow_cv_active;
	battmgr->oneplus_dual_hold_vnow_cv_adjustments =
		hold_vnow_cv_adjustments;
	battmgr->oneplus_dual_hold_adjustments = hold_adjustments;
	battmgr->oneplus_dual_hold_adjustments_up = hold_adjustments_up;
	battmgr->oneplus_dual_hold_adjustments_down = hold_adjustments_down;
	battmgr->oneplus_dual_hold_low_ibus_checks = hold_low_ibus_checks;
	battmgr->oneplus_dual_hold_emergency_adjustments =
		hold_emergency_adjustments;
	battmgr->oneplus_dual_hold_emergency_recoveries =
		hold_emergency_recoveries;
	battmgr->oneplus_dual_hold_emergency_steps = hold_emergency_steps;
	battmgr->oneplus_dual_hold_emergency_max_steps =
		hold_emergency_max_steps;
	battmgr->oneplus_dual_hold_output_limited = hold_output_limited;
	battmgr->oneplus_dual_hold_output_taper_active =
		hold_output_taper_active;
	battmgr->oneplus_dual_hold_output_taper_adjustments =
		hold_output_taper_adjustments;
	battmgr->oneplus_dual_hold_primary_output_uv = hold_primary_output_uv;
	battmgr->oneplus_dual_hold_secondary_output_uv =
		hold_secondary_output_uv;
	battmgr->oneplus_dual_hold_primary_output_max_uv =
		hold_primary_output_max_uv;
	battmgr->oneplus_dual_hold_secondary_output_max_uv =
		hold_secondary_output_max_uv;
	battmgr->oneplus_dual_hold_path_drop_uv = hold_path_drop_uv;
	battmgr->oneplus_dual_hold_path_drop_max_uv = hold_path_drop_max_uv;
	battmgr->oneplus_dual_hold_records = hold_records;
	battmgr->oneplus_dual_monitor_checks = monitor_checks;
	battmgr->oneplus_dual_monitor_vbat_uv = monitor_vbat_uv;
	battmgr->oneplus_dual_monitor_vbat_ocv_uv = monitor_vbat_ocv_uv;
	battmgr->oneplus_dual_monitor_source_request_mv =
		monitor_source_request_mv;
	battmgr->oneplus_dual_monitor_primary_output_uv =
		monitor_primary_output_uv;
	battmgr->oneplus_dual_monitor_secondary_output_uv =
		monitor_secondary_output_uv;
	battmgr->oneplus_dual_monitor_capacity = monitor_capacity;
	battmgr->oneplus_dual_monitor_usb_online = monitor_usb_online;
	battmgr->oneplus_dual_monitor_ibat_ua = monitor_ibat_ua;
	battmgr->oneplus_dual_monitor_temp_decic = monitor_temp_decic;
	battmgr->oneplus_dual_monitor_rc = monitor_rc;
	battmgr->oneplus_dual_post_stop_vbat_uv = post_stop_vbat_uv;
	battmgr->oneplus_dual_post_stop_vbat_ocv_uv = post_stop_vbat_ocv_uv;
	battmgr->oneplus_dual_post_stop_ibat_ua = post_stop_ibat_ua;
	battmgr->oneplus_dual_post_stop_rc = post_stop_rc;
	battmgr->oneplus_dual_ibus_rechecks = ibus_rechecks;
	battmgr->oneplus_dual_ibus_recoveries = ibus_recoveries;
	memcpy(battmgr->oneplus_dual_hold_sample_no, hold_sample_no,
	       sizeof(hold_sample_no));
	memcpy(battmgr->oneplus_dual_hold_request_mv, hold_request_mv,
	       sizeof(hold_request_mv));
	memcpy(battmgr->oneplus_dual_hold_primary_ibus_ua,
	       hold_primary_ibus_ua, sizeof(hold_primary_ibus_ua));
	memcpy(battmgr->oneplus_dual_hold_secondary_ibus_ua,
	       hold_secondary_ibus_ua, sizeof(hold_secondary_ibus_ua));
	battmgr->oneplus_dual_source_from_mv = source_from_mv;
	battmgr->oneplus_dual_source_to_mv = source_to_mv;
	battmgr->oneplus_dual_entry_derate_from_mv = entry_derate_from_mv;
	battmgr->oneplus_dual_entry_derate_to_mv = entry_derate_to_mv;
	battmgr->oneplus_dual_secondary_loaded_vbus_uv =
		secondary_loaded_vbus_uv;
	battmgr->oneplus_dual_secondary_loaded_output_uv =
		secondary_loaded_output_uv;
	battmgr->oneplus_dual_secondary_loaded_target_uv =
		secondary_loaded_target_uv;
	battmgr->oneplus_dual_result = ret;
	battmgr->oneplus_dual_running = false;
	battmgr->oneplus_dual_valid = true;
	battmgr->oneplus_dual_stop_requested = false;
	mutex_unlock(&battmgr->lock);

	return ret ? ret : count;
}

static ssize_t oneplus_pps_dual_500ms_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;
	if (!enable) {
		mutex_lock(&battmgr->lock);
		if (battmgr->oneplus_dual_auto_run)
			battmgr->oneplus_dual_auto_cancel_requested = true;
		mutex_unlock(&battmgr->lock);
		qcom_battmgr_oneplus_dual_request_stop(battmgr);
		return count;
	}

	return qcom_battmgr_oneplus_dual_run(battmgr, count, false);
}
static DEVICE_ATTR_RW(oneplus_pps_dual_500ms);

static const char *qcom_battmgr_oneplus_policy_stage_name(
	enum qcom_battmgr_oneplus_policy_stage stage)
{
	switch (stage) {
	case BATTMGR_ONEPLUS_POLICY_IDLE:
		return "idle";
	case BATTMGR_ONEPLUS_POLICY_RAMP:
		return "ramp";
	case BATTMGR_ONEPLUS_POLICY_START:
		return "start";
	case BATTMGR_ONEPLUS_POLICY_ACTIVE:
		return "active";
	case BATTMGR_ONEPLUS_POLICY_STOPPING:
		return "stopping";
	case BATTMGR_ONEPLUS_POLICY_COMPLETE:
		return "complete";
	case BATTMGR_ONEPLUS_POLICY_FAILED:
		return "failed";
	default:
		return "unknown";
	}
}

static const char *qcom_battmgr_oneplus_policy_reason_name(
	enum qcom_battmgr_oneplus_policy_stop_reason reason)
{
	switch (reason) {
	case BATTMGR_ONEPLUS_POLICY_STOP_NONE:
		return "none";
	case BATTMGR_ONEPLUS_POLICY_STOP_USER:
		return "user";
	case BATTMGR_ONEPLUS_POLICY_STOP_DURATION:
		return "duration";
	case BATTMGR_ONEPLUS_POLICY_STOP_VBAT_HIGH:
		return "vbat-high";
	case BATTMGR_ONEPLUS_POLICY_STOP_TEMP:
		return "temperature";
	case BATTMGR_ONEPLUS_POLICY_STOP_USB_DETACH:
		return "usb-detach";
	case BATTMGR_ONEPLUS_POLICY_STOP_INPUT_LOSS:
		return "input-loss";
	case BATTMGR_ONEPLUS_POLICY_STOP_SERVICE:
		return "service-down";
	case BATTMGR_ONEPLUS_POLICY_STOP_SOURCE_RETARGET:
		return "source-retarget";
	case BATTMGR_ONEPLUS_POLICY_STOP_CP:
		return "charge-pump";
	case BATTMGR_ONEPLUS_POLICY_STOP_STARTUP:
		return "startup";
	case BATTMGR_ONEPLUS_POLICY_STOP_REMOVE:
		return "driver-remove";
	default:
		return "unknown";
	}
}

static int qcom_battmgr_oneplus_policy_cancel_cp(
	struct qcom_battmgr *battmgr)
{
	struct power_supply *cp_psy;
	int ret;

	cp_psy = power_supply_get_by_name(BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
	if (!cp_psy) {
		ret = -ENODEV;
	} else {
		ret = qcom_battmgr_oneplus_cancel_cp(cp_psy);
		power_supply_put(cp_psy);
	}

	mutex_lock(&battmgr->lock);
	battmgr->oneplus_policy_cp_cancel_rc = ret;
	mutex_unlock(&battmgr->lock);

	return ret;
}

static void qcom_battmgr_oneplus_policy_stop_locked(
	struct qcom_battmgr *battmgr,
	enum qcom_battmgr_oneplus_policy_stop_reason reason, int monitor_rc)
{
	if (!battmgr->oneplus_policy_stop_requested) {
		battmgr->oneplus_policy_stop_requested = true;
		battmgr->oneplus_policy_stop_reason = reason;
		battmgr->oneplus_policy_monitor_rc = monitor_rc;
	}
	battmgr->oneplus_policy_stage = BATTMGR_ONEPLUS_POLICY_STOPPING;
}

static int qcom_battmgr_oneplus_policy_start_locked(
	struct qcom_battmgr *battmgr, bool automatic)
{
	bool queued;

	if (battmgr->oneplus_policy_running ||
	    battmgr->oneplus_pps_5m_running ||
	    battmgr->oneplus_secondary_running ||
	    battmgr->oneplus_dual_running)
		return -EBUSY;

	battmgr->oneplus_policy_running = true;
	battmgr->oneplus_policy_valid = false;
	battmgr->oneplus_policy_stop_requested = false;
	battmgr->oneplus_policy_auto_run = automatic;
	if (!automatic)
		battmgr->oneplus_policy_auto_armed = false;
	battmgr->oneplus_policy_stage = BATTMGR_ONEPLUS_POLICY_RAMP;
	battmgr->oneplus_policy_stop_reason =
		BATTMGR_ONEPLUS_POLICY_STOP_NONE;
	battmgr->oneplus_policy_result = -EINPROGRESS;
	battmgr->oneplus_policy_ramp_rc = -EINPROGRESS;
	battmgr->oneplus_policy_cp_run_rc = -EINPROGRESS;
	battmgr->oneplus_policy_cp_cancel_rc = 0;
	battmgr->oneplus_policy_cp_cleanup_rc = -EINPROGRESS;
	battmgr->oneplus_policy_source_restore_rc = -EINPROGRESS;
	battmgr->oneplus_policy_monitor_rc = 0;
	battmgr->oneplus_policy_monitor_checks = 0;
	battmgr->oneplus_policy_elapsed_ms = 0;
	battmgr->oneplus_policy_vbat_uv = 0;
	battmgr->oneplus_policy_ibus_ua = 0;
	battmgr->oneplus_policy_ibus_low_checks = 0;
	battmgr->oneplus_policy_temp_decic = 0;
	battmgr->oneplus_policy_feedback_decisions = 0;
	battmgr->oneplus_policy_feedback_up = 0;
	battmgr->oneplus_policy_feedback_down = 0;
	battmgr->oneplus_policy_feedback_hold = 0;
	battmgr->oneplus_policy_feedback_last_ibus_ua = 0;
	battmgr->oneplus_policy_feedback_last_delta_mv = 0;
	battmgr->oneplus_policy_retarget_attempts = 0;
	battmgr->oneplus_policy_retarget_completed = 0;
	battmgr->oneplus_policy_retarget_first_from_mv = 0;
	battmgr->oneplus_policy_retarget_last_to_mv = 0;
	battmgr->oneplus_policy_retarget_last_pre_vbus_uv = 0;
	battmgr->oneplus_policy_retarget_last_post_vbus_uv = 0;
	battmgr->oneplus_policy_retarget_rc = -EINPROGRESS;
	battmgr->oneplus_policy_started_at = 0;

	queued = queue_work(system_long_wq, &battmgr->oneplus_policy_work);
	if (queued)
		return 0;

	battmgr->oneplus_policy_running = false;
	battmgr->oneplus_policy_valid = true;
	battmgr->oneplus_policy_auto_run = false;
	battmgr->oneplus_policy_stage = BATTMGR_ONEPLUS_POLICY_FAILED;
	battmgr->oneplus_policy_stop_reason =
		BATTMGR_ONEPLUS_POLICY_STOP_STARTUP;
	battmgr->oneplus_policy_result = -EBUSY;
	return -EBUSY;
}

static int qcom_battmgr_oneplus_policy_feedback_locked(
	struct qcom_battmgr *battmgr)
{
	struct power_supply *cp_psy;
	union power_supply_propval val;
	u32 cp_vbat_uv;
	u32 ibus_ua;
	u32 target_mv;
	int delta_mv = 0;
	int ret;

	cp_psy = power_supply_get_by_name(BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
	if (!cp_psy)
		return -ENODEV;

	ret = qcom_battmgr_oneplus_read_cp_adc(
		cp_psy, false,
		&battmgr->oneplus_policy_retarget_last_pre_vbus_uv,
		&cp_vbat_uv);
	if (ret)
		goto out_put;
	ret = qcom_battmgr_oneplus_read_cp_ibus(cp_psy, &ibus_ua);
	if (ret)
		goto out_put;

	if (!battmgr->oneplus_policy_feedback_decisions)
		battmgr->oneplus_policy_retarget_first_from_mv =
			battmgr->oneplus_pps_source_request_mv;
	battmgr->oneplus_policy_feedback_decisions++;
	battmgr->oneplus_policy_feedback_last_ibus_ua = ibus_ua;
	if (ibus_ua < BATTMGR_ONEPLUS_POLICY_FEEDBACK_TARGET_IBUS_UA -
			      BATTMGR_ONEPLUS_POLICY_FEEDBACK_DEADBAND_UA) {
		delta_mv = BATTMGR_ONEPLUS_POLICY_FEEDBACK_STEP_MV;
		battmgr->oneplus_policy_feedback_up++;
	} else if (ibus_ua >
		   BATTMGR_ONEPLUS_POLICY_FEEDBACK_TARGET_IBUS_UA +
			   BATTMGR_ONEPLUS_POLICY_FEEDBACK_DEADBAND_UA) {
		delta_mv = -BATTMGR_ONEPLUS_POLICY_FEEDBACK_STEP_MV;
		battmgr->oneplus_policy_feedback_down++;
	} else {
		battmgr->oneplus_policy_feedback_hold++;
	}
	battmgr->oneplus_policy_feedback_last_delta_mv = delta_mv;

	target_mv = battmgr->oneplus_pps_source_request_mv;
	if (delta_mv > 0) {
		if (target_mv > BATTMGR_ONEPLUS_CP_2TO1_SOURCE_MAX_MV -
				BATTMGR_ONEPLUS_POLICY_FEEDBACK_STEP_MV) {
			ret = -ERANGE;
			goto out_put;
		}
		target_mv += BATTMGR_ONEPLUS_POLICY_FEEDBACK_STEP_MV;
	} else if (delta_mv < 0) {
		if (target_mv < BATTMGR_ONEPLUS_TEST_PPS_MV +
				BATTMGR_ONEPLUS_POLICY_FEEDBACK_STEP_MV) {
			ret = -ERANGE;
			goto out_put;
		}
		target_mv -= BATTMGR_ONEPLUS_POLICY_FEEDBACK_STEP_MV;
	}
	battmgr->oneplus_policy_retarget_last_to_mv = target_mv;

	if (delta_mv &&
	    !qcom_battmgr_oneplus_has_pps(battmgr, target_mv,
					 BATTMGR_ONEPLUS_POLICY_PPS_MA)) {
		ret = -ERANGE;
		goto out_put;
	}

	if (!delta_mv) {
		battmgr->oneplus_policy_retarget_last_post_vbus_uv =
			battmgr->oneplus_policy_retarget_last_pre_vbus_uv;
		goto verify_active;
	}

	battmgr->oneplus_policy_retarget_attempts++;

	ret = qcom_battmgr_oneplus_set_pps_locked(
		battmgr, target_mv, BATTMGR_ONEPLUS_POLICY_PPS_MA);
	if (ret)
		goto out_put;
	battmgr->oneplus_pps_source_request_mv = target_mv;

	msleep(BATTMGR_ONEPLUS_POLICY_FEEDBACK_SETTLE_MS);
	ret = qcom_battmgr_oneplus_read_cp_adc(
		cp_psy, false,
		&battmgr->oneplus_policy_retarget_last_post_vbus_uv,
		&cp_vbat_uv);
	if (ret)
		goto out_put;

verify_active:
	ret = power_supply_get_property(cp_psy, POWER_SUPPLY_PROP_ONLINE,
					&val);
	if (ret)
		goto out_put;
	if (!val.intval) {
		ret = -EIO;
		goto out_put;
	}
	ret = power_supply_get_property(cp_psy, POWER_SUPPLY_PROP_STATUS,
					&val);
	if (!ret && val.intval != POWER_SUPPLY_STATUS_CHARGING)
		ret = -EIO;
	if (!ret && delta_mv)
		battmgr->oneplus_policy_retarget_completed++;

out_put:
	power_supply_put(cp_psy);
	return ret;
}

static void qcom_battmgr_oneplus_policy_monitor_worker(
	struct work_struct *work)
{
	struct qcom_battmgr *battmgr = container_of(
		to_delayed_work(work), struct qcom_battmgr,
		oneplus_policy_monitor_work);
	bool cancel_cp = false;
	bool reschedule = false;
	struct power_supply *cp_psy;
	u32 ibus_ua;
	u32 vbat_uv;
	int temp_decic;
	int ret;

	mutex_lock(&battmgr->lock);
	if (!battmgr->oneplus_policy_running ||
	    battmgr->oneplus_policy_stage != BATTMGR_ONEPLUS_POLICY_ACTIVE)
		goto out_unlock;

	battmgr->oneplus_policy_elapsed_ms = jiffies_to_msecs(
		jiffies - battmgr->oneplus_policy_started_at);
	battmgr->oneplus_policy_monitor_checks++;
	if (battmgr->oneplus_policy_stop_requested) {
		cancel_cp = true;
		goto out_unlock;
	}

	if (!battmgr->service_up) {
		qcom_battmgr_oneplus_policy_stop_locked(
			battmgr, BATTMGR_ONEPLUS_POLICY_STOP_SERVICE,
			-ENOTCONN);
		cancel_cp = true;
		goto out_unlock;
	}

	ret = qcom_battmgr_request_property(battmgr,
		BATTMGR_USB_PROPERTY_GET, USB_ONLINE, 0);
	if (ret || !battmgr->usb.online) {
		qcom_battmgr_oneplus_policy_stop_locked(
			battmgr,
			ret ? BATTMGR_ONEPLUS_POLICY_STOP_SERVICE :
			      BATTMGR_ONEPLUS_POLICY_STOP_USB_DETACH,
			ret ?: -ENODEV);
		cancel_cp = true;
		goto out_unlock;
	}

	ret = qcom_battmgr_request_property(battmgr,
		BATTMGR_BAT_PROPERTY_GET, BATT_VOLT_NOW, 0);
	if (ret) {
		qcom_battmgr_oneplus_policy_stop_locked(
			battmgr, BATTMGR_ONEPLUS_POLICY_STOP_SERVICE, ret);
		cancel_cp = true;
		goto out_unlock;
	}
	vbat_uv = qcom_battmgr_oneplus_vbat_uv(battmgr);
	if (vbat_uv)
		battmgr->oneplus_policy_vbat_uv = vbat_uv;
	if (vbat_uv > BATTMGR_ONEPLUS_POLICY_VBAT_MAX_UV) {
		qcom_battmgr_oneplus_policy_stop_locked(
			battmgr, BATTMGR_ONEPLUS_POLICY_STOP_VBAT_HIGH, 0);
		cancel_cp = true;
		goto out_unlock;
	}

	ret = qcom_battmgr_request_property(battmgr,
		BATTMGR_BAT_PROPERTY_GET, BATT_TEMP, 0);
	if (ret) {
		qcom_battmgr_oneplus_policy_stop_locked(
			battmgr, BATTMGR_ONEPLUS_POLICY_STOP_SERVICE, ret);
		cancel_cp = true;
		goto out_unlock;
	}
	temp_decic = battmgr->status.temperature;
	battmgr->oneplus_policy_temp_decic = temp_decic;
	if (temp_decic < BATTMGR_ONEPLUS_POLICY_TEMP_MIN_DECIC ||
	    temp_decic > BATTMGR_ONEPLUS_POLICY_TEMP_MAX_DECIC) {
		qcom_battmgr_oneplus_policy_stop_locked(
			battmgr, BATTMGR_ONEPLUS_POLICY_STOP_TEMP, -ERANGE);
		cancel_cp = true;
		goto out_unlock;
	}

	cp_psy = power_supply_get_by_name(BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
	if (!cp_psy) {
		qcom_battmgr_oneplus_policy_stop_locked(
			battmgr, BATTMGR_ONEPLUS_POLICY_STOP_CP, -ENODEV);
		cancel_cp = true;
		goto out_unlock;
	}
	ret = qcom_battmgr_oneplus_read_cp_ibus(cp_psy, &ibus_ua);
	power_supply_put(cp_psy);
	if (ret) {
		qcom_battmgr_oneplus_policy_stop_locked(
			battmgr, BATTMGR_ONEPLUS_POLICY_STOP_CP, ret);
		cancel_cp = true;
		goto out_unlock;
	}
	battmgr->oneplus_policy_ibus_ua = ibus_ua;
	if (ibus_ua < BATTMGR_ONEPLUS_POLICY_IBUS_MIN_UA) {
		battmgr->oneplus_policy_ibus_low_checks++;
		if (battmgr->oneplus_policy_ibus_low_checks >=
		    BATTMGR_ONEPLUS_POLICY_IBUS_LOW_CHECKS) {
			qcom_battmgr_oneplus_policy_stop_locked(
				battmgr, BATTMGR_ONEPLUS_POLICY_STOP_INPUT_LOSS,
				-ENOLINK);
			cancel_cp = true;
			goto out_unlock;
		}
	} else {
		battmgr->oneplus_policy_ibus_low_checks = 0;
	}

	if (battmgr->oneplus_policy_feedback_decisions <
		    BATTMGR_ONEPLUS_POLICY_FEEDBACK_DECISIONS &&
	    battmgr->oneplus_policy_monitor_checks >=
		    BATTMGR_ONEPLUS_POLICY_FEEDBACK_FIRST_CHECK +
		    battmgr->oneplus_policy_feedback_decisions *
		    BATTMGR_ONEPLUS_POLICY_FEEDBACK_INTERVAL_CHECKS) {
		ret = qcom_battmgr_oneplus_policy_feedback_locked(battmgr);
		battmgr->oneplus_policy_retarget_rc = ret;
		if (ret) {
			qcom_battmgr_oneplus_policy_stop_locked(
				battmgr,
				BATTMGR_ONEPLUS_POLICY_STOP_SOURCE_RETARGET,
				ret);
			cancel_cp = true;
			goto out_unlock;
		}
	}

	battmgr->oneplus_policy_monitor_rc = 0;
	reschedule = true;

out_unlock:
	mutex_unlock(&battmgr->lock);

	if (cancel_cp)
		qcom_battmgr_oneplus_policy_cancel_cp(battmgr);
	else if (reschedule)
		queue_delayed_work(system_long_wq,
			&battmgr->oneplus_policy_monitor_work,
			msecs_to_jiffies(BATTMGR_ONEPLUS_POLICY_CHECK_MS));
}

static int qcom_battmgr_oneplus_policy_source_connected_locked(
	struct qcom_battmgr *battmgr, bool *connected)
{
	struct power_supply *cp_psy;
	union power_supply_propval val;
	int ret;

	*connected = false;
	cp_psy = power_supply_get_by_name(BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
	if (!cp_psy)
		return -ENODEV;

	ret = power_supply_get_property(cp_psy, POWER_SUPPLY_PROP_PRESENT,
					&val);
	if (ret)
		goto out_put;
	if (!val.intval)
		goto out_ok;

	ret = power_supply_get_property(cp_psy,
					POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (ret)
		goto out_put;
	if (val.intval < BATTMGR_ONEPLUS_POLICY_AUTO_VBUS_MIN_UV)
		goto out_ok;

	ret = qcom_battmgr_request_property(battmgr,
		BATTMGR_USB_PROPERTY_GET, USB_ONLINE, 0);
	if (ret)
		goto out_put;
	*connected = battmgr->usb.online;

out_ok:
	ret = 0;
out_put:
	power_supply_put(cp_psy);
	return ret;
}

static void qcom_battmgr_oneplus_policy_auto_worker(
	struct work_struct *work)
{
	struct qcom_battmgr *battmgr = container_of(
		to_delayed_work(work), struct qcom_battmgr,
		oneplus_policy_auto_work);
	bool connected;
	u32 vbat_uv;
	int temp_decic;
	int ret;

	mutex_lock(&battmgr->lock);
	if (!battmgr->oneplus_policy_auto_start)
		goto out_unlock;
	if (!battmgr->service_up || battmgr->oneplus_policy_running ||
	    battmgr->oneplus_pps_5m_running ||
	    battmgr->oneplus_secondary_running ||
	    battmgr->oneplus_dual_running)
		goto out_unlock;

	ret = qcom_battmgr_oneplus_policy_source_connected_locked(
		battmgr, &connected);
	battmgr->oneplus_policy_auto_last_rc = ret;
	if (ret)
		goto out_unlock;
	if (!connected) {
		battmgr->oneplus_policy_auto_armed = true;
		battmgr->oneplus_policy_auto_attempts = 0;
		goto out_unlock;
	}
	if (!battmgr->oneplus_policy_auto_armed ||
	    battmgr->oneplus_policy_auto_attempts >=
		BATTMGR_ONEPLUS_POLICY_AUTO_RETRIES)
		goto out_unlock;

	ret = qcom_battmgr_request_property(battmgr,
		BATTMGR_BAT_PROPERTY_GET, BATT_VOLT_NOW, 0);
	if (ret)
		goto record_error;
	vbat_uv = qcom_battmgr_oneplus_vbat_uv(battmgr);
	if (!vbat_uv) {
		ret = -EAGAIN;
		goto record_error;
	}
	if (vbat_uv > BATTMGR_ONEPLUS_POLICY_AUTO_VBAT_MAX_UV) {
		ret = -ERANGE;
		goto record_error;
	}

	ret = qcom_battmgr_request_property(battmgr,
		BATTMGR_BAT_PROPERTY_GET, BATT_TEMP, 0);
	if (ret)
		goto record_error;
	temp_decic = battmgr->status.temperature;
	if (temp_decic < BATTMGR_ONEPLUS_POLICY_TEMP_MIN_DECIC ||
	    temp_decic > BATTMGR_ONEPLUS_POLICY_TEMP_MAX_DECIC) {
		ret = -ERANGE;
		goto record_error;
	}

	battmgr->oneplus_policy_auto_attempts++;
	ret = qcom_battmgr_oneplus_policy_start_locked(battmgr, true);
record_error:
	battmgr->oneplus_policy_auto_last_rc = ret;

out_unlock:
	mutex_unlock(&battmgr->lock);
	if (READ_ONCE(battmgr->oneplus_policy_auto_start))
		queue_delayed_work(system_long_wq,
			&battmgr->oneplus_policy_auto_work,
			msecs_to_jiffies(
				BATTMGR_ONEPLUS_POLICY_AUTO_CHECK_MS));
}

static int qcom_battmgr_oneplus_dual_auto_connected_locked(
	struct qcom_battmgr *battmgr, bool *connected)
{
	struct power_supply *cp_psy;
	union power_supply_propval val;
	bool cp_connected = false;
	bool usb_online = false;
	int cp_ret = -ENODEV;
	int usb_ret;

	*connected = false;
	usb_ret = qcom_battmgr_request_property(battmgr,
		BATTMGR_USB_PROPERTY_GET, USB_ONLINE, 0);
	if (!usb_ret)
		usb_online = battmgr->usb.online;

	cp_psy = power_supply_get_by_name(BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
	if (cp_psy) {
		cp_ret = power_supply_get_property(cp_psy,
			POWER_SUPPLY_PROP_PRESENT, &val);
		if (!cp_ret && val.intval) {
			cp_ret = power_supply_get_property(cp_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
			if (!cp_ret && val.intval >=
					BATTMGR_ONEPLUS_DUAL_AUTO_VBUS_MIN_UV)
				cp_connected = true;
		}
		power_supply_put(cp_psy);
	}

	if (usb_ret && cp_ret)
		return usb_ret;

	/* A live physical input masks firmware-online transition glitches. */
	*connected = (!usb_ret && usb_online) || (!cp_ret && cp_connected);

	return 0;
}

static int qcom_battmgr_oneplus_dual_auto_wait_connected(
	struct qcom_battmgr *battmgr)
{
	bool connected;
	unsigned int check;
	int ret;

	/*
	 * A completed window deliberately turns both pumps off and restores fixed
	 * 5 V.  Firmware USB_ONLINE and the pump-side VBUS observation can both be
	 * transiently unavailable at that exact boundary.  Give the same one-second
	 * cadence and two-sample confirmation used by the attach watcher a chance
	 * to distinguish that protocol edge from a real cable removal.
	 */
	for (check = 0;
	     check < BATTMGR_ONEPLUS_DUAL_AUTO_OFFLINE_CHECKS;
	     check++) {
		msleep(BATTMGR_ONEPLUS_DUAL_AUTO_CHECK_MS);

		mutex_lock(&battmgr->lock);
		if (!battmgr->oneplus_dual_auto_enabled ||
		    !battmgr->oneplus_dual_auto_run ||
		    battmgr->oneplus_dual_auto_cancel_requested) {
			ret = -ECANCELED;
		} else if (!battmgr->service_up) {
			ret = -EHOSTDOWN;
		} else {
			ret = qcom_battmgr_oneplus_dual_auto_connected_locked(
				battmgr, &connected);
			if (!ret && connected) {
				mutex_unlock(&battmgr->lock);
				return 0;
			}
		}
		mutex_unlock(&battmgr->lock);
		if (ret)
			return ret;
	}

	return -ENODEV;
}

static void qcom_battmgr_oneplus_dual_auto_run_worker(
	struct work_struct *work)
{
	struct qcom_battmgr *battmgr = container_of(work,
		struct qcom_battmgr, oneplus_dual_auto_run_work);
	bool continuous;
	bool full_confirmed;
	ssize_t ret = -ECANCELED;
	int connected_ret;

	for (;;) {
		mutex_lock(&battmgr->lock);
		continuous = battmgr->oneplus_dual_auto_continuous;
		if (!battmgr->oneplus_dual_auto_enabled ||
		    !battmgr->oneplus_dual_auto_run ||
		    battmgr->oneplus_dual_auto_cancel_requested) {
			ret = -ECANCELED;
			mutex_unlock(&battmgr->lock);
			break;
		}
		if (!continuous &&
		    battmgr->oneplus_dual_auto_windows_started >=
			BATTMGR_ONEPLUS_DUAL_AUTO_WINDOW_LIMIT) {
			ret = 0;
			mutex_unlock(&battmgr->lock);
			break;
		}
		battmgr->oneplus_dual_auto_windows_started++;
		mutex_unlock(&battmgr->lock);

		ret = qcom_battmgr_oneplus_dual_run(battmgr, 1, true);

		mutex_lock(&battmgr->lock);
		if (ret >= 0)
			battmgr->oneplus_dual_auto_windows_completed++;
		full_confirmed = battmgr->oneplus_dual_hold_full_confirmed;
		if (full_confirmed)
			battmgr->oneplus_dual_auto_full_stop = true;
		/*
		 * A continuous call returns only for a terminal condition; never
		 * translate that return into another pump/PPS startup on the same
		 * physical attachment.  The attach watcher rearms after a real detach.
		 */
		if (continuous || ret < 0 || full_confirmed ||
		    !battmgr->oneplus_dual_auto_enabled ||
		    !battmgr->oneplus_dual_auto_run ||
		    battmgr->oneplus_dual_auto_cancel_requested ||
		    (!continuous &&
		     battmgr->oneplus_dual_auto_windows_completed >=
			BATTMGR_ONEPLUS_DUAL_AUTO_WINDOW_LIMIT)) {
			mutex_unlock(&battmgr->lock);
			break;
		}

		mutex_unlock(&battmgr->lock);

		connected_ret = qcom_battmgr_oneplus_dual_auto_wait_connected(
			battmgr);
		if (connected_ret) {
			ret = connected_ret;
			mutex_lock(&battmgr->lock);
			battmgr->oneplus_dual_auto_cancel_requested = true;
			mutex_unlock(&battmgr->lock);
			break;
		}
	}

	mutex_lock(&battmgr->lock);
	battmgr->oneplus_dual_auto_last_rc = ret < 0 ? ret : 0;
	battmgr->oneplus_dual_auto_run = false;
	mutex_unlock(&battmgr->lock);
}

static void qcom_battmgr_oneplus_dual_auto_watch_worker(
	struct work_struct *work)
{
	struct qcom_battmgr *battmgr = container_of(to_delayed_work(work),
		struct qcom_battmgr, oneplus_dual_auto_watch_work);
	bool connected;
	bool stop = false;
	bool reschedule = false;
	bool service_recovered = false;
	bool queued;
	int ret;

	mutex_lock(&battmgr->lock);
	if (!battmgr->oneplus_dual_auto_enabled)
		goto out_unlock;
	reschedule = true;

	if (!READ_ONCE(battmgr->service_up)) {
		/*
		 * A missed or late PDR-up notification must not permanently blind
		 * plug detection.  Probe the already serialized PMIC-GLINK path with
		 * a read-only property request.  A real reply proves the endpoint is
		 * usable; a failed request retains the fail-closed service-loss path.
		 */
		ret = qcom_battmgr_request_property(battmgr,
			BATTMGR_USB_PROPERTY_GET, USB_ONLINE, 0);
		if (ret) {
			battmgr->oneplus_dual_auto_last_rc = -EHOSTDOWN;
			if (battmgr->oneplus_dual_auto_run)
				battmgr->oneplus_dual_auto_cancel_requested = true;
			if (battmgr->oneplus_dual_auto_run &&
			    battmgr->oneplus_dual_running &&
			    !battmgr->oneplus_dual_stop_requested) {
				battmgr->oneplus_dual_stop_requested = true;
				battmgr->oneplus_dual_auto_stops++;
				stop = true;
			}
			goto out_unlock;
		}

		WRITE_ONCE(battmgr->service_up, true);
		service_recovered = true;
	}

	ret = qcom_battmgr_oneplus_dual_auto_connected_locked(
		battmgr, &connected);
	if (ret) {
		battmgr->oneplus_dual_auto_last_rc = ret;
		goto out_unlock;
	}
	if (!battmgr->oneplus_dual_auto_run &&
	    (battmgr->oneplus_dual_auto_last_rc == -EHOSTDOWN ||
	     battmgr->oneplus_dual_auto_last_rc == -EINPROGRESS))
		battmgr->oneplus_dual_auto_last_rc = 0;

	if (!connected) {
		if (battmgr->oneplus_dual_auto_offline_checks <
		    BATTMGR_ONEPLUS_DUAL_AUTO_OFFLINE_CHECKS)
			battmgr->oneplus_dual_auto_offline_checks++;
		if (battmgr->oneplus_dual_auto_offline_checks <
		    BATTMGR_ONEPLUS_DUAL_AUTO_OFFLINE_CHECKS)
			goto out_unlock;

		battmgr->oneplus_dual_auto_connected = false;
		if (!battmgr->oneplus_dual_auto_run)
			battmgr->oneplus_dual_auto_armed = true;
		else {
			battmgr->oneplus_dual_auto_cancel_requested = true;
			if (battmgr->oneplus_dual_running &&
			    !battmgr->oneplus_dual_stop_requested) {
				battmgr->oneplus_dual_stop_requested = true;
				battmgr->oneplus_dual_auto_stops++;
				stop = true;
			}
		}
		goto out_unlock;
	}
	battmgr->oneplus_dual_auto_offline_checks = 0;

	if (!battmgr->oneplus_dual_auto_connected &&
	    battmgr->oneplus_dual_auto_armed) {
		battmgr->oneplus_dual_auto_armed = false;
		battmgr->oneplus_dual_auto_attempts++;
		if (battmgr->oneplus_dual_auto_run ||
		    battmgr->oneplus_dual_running ||
		    battmgr->oneplus_secondary_running ||
		    battmgr->oneplus_policy_running ||
		    battmgr->oneplus_pps_5m_running) {
			battmgr->oneplus_dual_auto_last_rc = -EBUSY;
		} else {
			battmgr->oneplus_dual_auto_run = true;
			battmgr->oneplus_dual_auto_cancel_requested = false;
			battmgr->oneplus_dual_auto_windows_started = 0;
			battmgr->oneplus_dual_auto_windows_completed = 0;
			battmgr->oneplus_dual_auto_full_stop = false;
			queued = queue_work(system_long_wq,
				&battmgr->oneplus_dual_auto_run_work);
			if (queued) {
				battmgr->oneplus_dual_auto_starts++;
				battmgr->oneplus_dual_auto_last_rc =
					-EINPROGRESS;
			} else {
				battmgr->oneplus_dual_auto_run = false;
				battmgr->oneplus_dual_auto_last_rc = -EBUSY;
			}
		}
	}
	battmgr->oneplus_dual_auto_connected = true;

out_unlock:
	mutex_unlock(&battmgr->lock);
	if (service_recovered)
		schedule_work(&battmgr->enable_work);
	if (stop)
		qcom_battmgr_oneplus_dual_request_stop(battmgr);
	if (reschedule && READ_ONCE(battmgr->oneplus_dual_auto_enabled))
		mod_delayed_work(system_long_wq,
			&battmgr->oneplus_dual_auto_watch_work,
			msecs_to_jiffies(BATTMGR_ONEPLUS_DUAL_AUTO_CHECK_MS));
}

static ssize_t oneplus_pps_dual_auto_once_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	bool enabled;
	bool armed;
	bool connected;
	bool auto_run;
	bool cancel_requested;
	bool continuous;
	bool full_stop;
	bool dual_running;
	bool service_up;
	u32 attempts;
	u32 starts;
	u32 stops;
	u32 offline_checks;
	u32 windows_started;
	u32 windows_completed;
	u32 window_limit;
	int last_rc;

	mutex_lock(&battmgr->lock);
	enabled = battmgr->oneplus_dual_auto_enabled;
	armed = battmgr->oneplus_dual_auto_armed;
	connected = battmgr->oneplus_dual_auto_connected;
	auto_run = battmgr->oneplus_dual_auto_run;
	cancel_requested = battmgr->oneplus_dual_auto_cancel_requested;
	continuous = battmgr->oneplus_dual_auto_continuous;
	full_stop = battmgr->oneplus_dual_auto_full_stop;
	dual_running = battmgr->oneplus_dual_running;
	service_up = READ_ONCE(battmgr->service_up);
	attempts = battmgr->oneplus_dual_auto_attempts;
	starts = battmgr->oneplus_dual_auto_starts;
	stops = battmgr->oneplus_dual_auto_stops;
	offline_checks = battmgr->oneplus_dual_auto_offline_checks;
	windows_started = battmgr->oneplus_dual_auto_windows_started;
	windows_completed = battmgr->oneplus_dual_auto_windows_completed;
	window_limit = continuous ? 0 :
		BATTMGR_ONEPLUS_DUAL_AUTO_WINDOW_LIMIT;
	last_rc = battmgr->oneplus_dual_auto_last_rc;
	mutex_unlock(&battmgr->lock);

	return sysfs_emit(buf,
		"revision=%s mode=%s service_up=%u enabled=%u armed=%u connected=%u auto_run=%u dual_running=%u cancel=%u full_stop=%u attempts=%u starts=%u stops=%u offline=%u/%u window_limit=%u window_starts=%u window_completes=%u last_rc=%d\n",
		BATTMGR_ONEPLUS_POLICY_REVISION,
		continuous ? "continuous" : "bounded",
		service_up, enabled, armed, connected, auto_run, dual_running,
		cancel_requested, full_stop, attempts, starts, stops,
		offline_checks, BATTMGR_ONEPLUS_DUAL_AUTO_OFFLINE_CHECKS,
		window_limit, windows_started, windows_completed, last_rc);
}

static ssize_t qcom_battmgr_oneplus_dual_auto_store(
	struct qcom_battmgr *battmgr, bool enable, bool continuous,
	size_t count)
{
	bool connected = false;
	bool auto_run;
	int ret = 0;

	if (enable) {
		mutex_lock(&battmgr->lock);
		if (battmgr->oneplus_dual_auto_enabled &&
		    battmgr->oneplus_dual_auto_continuous != continuous) {
			mutex_unlock(&battmgr->lock);
			return -EBUSY;
		}
		if (!battmgr->oneplus_dual_auto_enabled) {
			battmgr->oneplus_dual_auto_enabled = true;
			battmgr->oneplus_dual_auto_continuous = continuous;
			battmgr->oneplus_dual_auto_armed = false;
			battmgr->oneplus_dual_auto_connected = true;
			battmgr->oneplus_dual_auto_cancel_requested = false;
			battmgr->oneplus_dual_auto_full_stop = false;
			battmgr->oneplus_dual_auto_attempts = 0;
			battmgr->oneplus_dual_auto_starts = 0;
			battmgr->oneplus_dual_auto_stops = 0;
			battmgr->oneplus_dual_auto_offline_checks = 0;
			battmgr->oneplus_dual_auto_windows_started = 0;
			battmgr->oneplus_dual_auto_windows_completed = 0;
			if (battmgr->service_up)
				ret = qcom_battmgr_oneplus_dual_auto_connected_locked(
					battmgr, &connected);
			else
				ret = -EHOSTDOWN;
			battmgr->oneplus_dual_auto_last_rc = ret;
		}
		mutex_unlock(&battmgr->lock);
		mod_delayed_work(system_long_wq,
			&battmgr->oneplus_dual_auto_watch_work, 0);
		return count;
	}

	mutex_lock(&battmgr->lock);
	battmgr->oneplus_dual_auto_enabled = false;
	battmgr->oneplus_dual_auto_armed = false;
	battmgr->oneplus_dual_auto_cancel_requested = true;
	battmgr->oneplus_dual_auto_offline_checks = 0;
	auto_run = battmgr->oneplus_dual_auto_run;
	mutex_unlock(&battmgr->lock);

	cancel_delayed_work_sync(&battmgr->oneplus_dual_auto_watch_work);
	if (auto_run)
		qcom_battmgr_oneplus_dual_request_stop(battmgr);
	cancel_work_sync(&battmgr->oneplus_dual_auto_run_work);

	mutex_lock(&battmgr->lock);
	battmgr->oneplus_dual_auto_run = false;
	mutex_unlock(&battmgr->lock);

	return count;
}

static ssize_t oneplus_pps_dual_auto_once_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	return qcom_battmgr_oneplus_dual_auto_store(
		battmgr, enable, false, count);
}
static DEVICE_ATTR_RW(oneplus_pps_dual_auto_once);

static ssize_t oneplus_pps_dual_auto_continuous_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return oneplus_pps_dual_auto_once_show(dev, attr, buf);
}

static ssize_t oneplus_pps_dual_auto_continuous_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	return qcom_battmgr_oneplus_dual_auto_store(
		battmgr, enable, true, count);
}
static DEVICE_ATTR_RW(oneplus_pps_dual_auto_continuous);

static void qcom_battmgr_oneplus_policy_worker(struct work_struct *work)
{
	struct qcom_battmgr *battmgr = container_of(
		work, struct qcom_battmgr, oneplus_policy_work);
	struct power_supply *cp_psy = NULL;
	enum qcom_battmgr_oneplus_policy_stop_reason reason;
	int source_restore_rc;
	int cp_cleanup_rc = -ECANCELED;
	int cp_run_rc = -ECANCELED;
	int ramp_rc;
	int ret = 0;

	/* Stage 6C13 is strictly single-primary; neither pump may pre-exist on. */
	ret = qcom_battmgr_oneplus_require_cp_off(
		BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
	if (!ret)
		ret = qcom_battmgr_oneplus_require_cp_off(
			BATTMGR_ONEPLUS_SECONDARY_CP_NAME);
	if (ret) {
		ramp_rc = ret;
		mutex_lock(&battmgr->lock);
		qcom_battmgr_oneplus_policy_stop_locked(
			battmgr, BATTMGR_ONEPLUS_POLICY_STOP_STARTUP, ret);
		mutex_unlock(&battmgr->lock);
		goto restore_source;
	}

	ramp_rc = qcom_battmgr_oneplus_ramp_2to1_run(
		battmgr, BATTMGR_ONEPLUS_PRIMARY_CP_NAME, true, false, false,
		false,
		BATTMGR_ONEPLUS_POLICY_PPS_MA);
	if (ramp_rc) {
		ret = ramp_rc;
		mutex_lock(&battmgr->lock);
		qcom_battmgr_oneplus_policy_stop_locked(
			battmgr, BATTMGR_ONEPLUS_POLICY_STOP_STARTUP, ramp_rc);
		mutex_unlock(&battmgr->lock);
		goto restore_source;
	}

	mutex_lock(&battmgr->lock);
	battmgr->oneplus_policy_ramp_rc = ramp_rc;
	battmgr->oneplus_policy_vbat_uv = battmgr->oneplus_pps_vbat_uv;
	if (battmgr->oneplus_policy_stop_requested) {
		ret = -ECANCELED;
		mutex_unlock(&battmgr->lock);
		goto cleanup_cp;
	}
	if (battmgr->oneplus_policy_vbat_uv >
	    BATTMGR_ONEPLUS_POLICY_VBAT_MAX_UV) {
		qcom_battmgr_oneplus_policy_stop_locked(
			battmgr, BATTMGR_ONEPLUS_POLICY_STOP_VBAT_HIGH, 0);
		mutex_unlock(&battmgr->lock);
		goto cleanup_cp;
	}
	ret = qcom_battmgr_request_property(battmgr,
		BATTMGR_BAT_PROPERTY_GET, BATT_TEMP, 0);
	if (ret) {
		qcom_battmgr_oneplus_policy_stop_locked(
			battmgr, BATTMGR_ONEPLUS_POLICY_STOP_STARTUP, ret);
		mutex_unlock(&battmgr->lock);
		goto cleanup_cp;
	}
	battmgr->oneplus_policy_temp_decic = battmgr->status.temperature;
	if (battmgr->oneplus_policy_temp_decic <
			BATTMGR_ONEPLUS_POLICY_TEMP_MIN_DECIC ||
	    battmgr->oneplus_policy_temp_decic >
			BATTMGR_ONEPLUS_POLICY_TEMP_MAX_DECIC) {
		ret = -ERANGE;
		qcom_battmgr_oneplus_policy_stop_locked(
			battmgr, BATTMGR_ONEPLUS_POLICY_STOP_TEMP, ret);
		mutex_unlock(&battmgr->lock);
		goto cleanup_cp;
	}
	battmgr->oneplus_policy_stage = BATTMGR_ONEPLUS_POLICY_START;
	mutex_unlock(&battmgr->lock);

	cp_psy = power_supply_get_by_name(BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
	if (!cp_psy) {
		ret = -ENODEV;
		mutex_lock(&battmgr->lock);
		qcom_battmgr_oneplus_policy_stop_locked(
			battmgr, BATTMGR_ONEPLUS_POLICY_STOP_STARTUP, ret);
		mutex_unlock(&battmgr->lock);
		goto restore_source;
	}

	mutex_lock(&battmgr->lock);
	if (battmgr->oneplus_policy_stop_requested) {
		ret = -ECANCELED;
		mutex_unlock(&battmgr->lock);
		goto cleanup_cp;
	}
	battmgr->oneplus_policy_stage = BATTMGR_ONEPLUS_POLICY_ACTIVE;
	battmgr->oneplus_policy_started_at = jiffies;
	if (battmgr->oneplus_policy_auto_run) {
		battmgr->oneplus_policy_auto_armed = false;
		battmgr->oneplus_policy_auto_starts++;
		battmgr->oneplus_policy_auto_last_rc = 0;
	}
	mutex_unlock(&battmgr->lock);
	queue_delayed_work(system_long_wq,
		&battmgr->oneplus_policy_monitor_work,
		msecs_to_jiffies(BATTMGR_ONEPLUS_POLICY_CHECK_MS));

	cp_run_rc = qcom_battmgr_oneplus_run_cp(
		cp_psy, BATTMGR_ONEPLUS_CP_POLICY_RUN_MS);
	ret = cp_run_rc;
	/* Expose the physical return before potentially blocking monitor cleanup. */
	mutex_lock(&battmgr->lock);
	battmgr->oneplus_policy_cp_run_rc = cp_run_rc;
	if (battmgr->oneplus_policy_stage == BATTMGR_ONEPLUS_POLICY_ACTIVE)
		battmgr->oneplus_policy_stage = BATTMGR_ONEPLUS_POLICY_STOPPING;
	mutex_unlock(&battmgr->lock);
	cancel_delayed_work_sync(&battmgr->oneplus_policy_monitor_work);

cleanup_cp:
	if (!cp_psy)
		cp_psy = power_supply_get_by_name(
			BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
	if (cp_psy) {
		cp_cleanup_rc = qcom_battmgr_oneplus_set_cp_prepared(cp_psy,
			false);
		qcom_battmgr_oneplus_set_cp_start_target(cp_psy, 0);
		power_supply_put(cp_psy);
		cp_psy = NULL;
		if (!ret && cp_cleanup_rc)
			ret = cp_cleanup_rc;
	}

restore_source:
	mutex_lock(&battmgr->lock);
	reason = battmgr->oneplus_policy_stop_reason;
	if (!cp_run_rc && reason == BATTMGR_ONEPLUS_POLICY_STOP_NONE) {
		reason = BATTMGR_ONEPLUS_POLICY_STOP_DURATION;
		battmgr->oneplus_policy_stop_reason = reason;
	} else if (cp_run_rc && cp_run_rc != -ECANCELED &&
		   reason == BATTMGR_ONEPLUS_POLICY_STOP_NONE) {
		reason = BATTMGR_ONEPLUS_POLICY_STOP_CP;
		battmgr->oneplus_policy_stop_reason = reason;
	}
	if (cp_run_rc == -ECANCELED &&
	    (reason == BATTMGR_ONEPLUS_POLICY_STOP_USER ||
	     reason == BATTMGR_ONEPLUS_POLICY_STOP_VBAT_HIGH))
		ret = 0;
	else if (cp_run_rc == -ECANCELED &&
		 battmgr->oneplus_policy_monitor_rc)
		ret = battmgr->oneplus_policy_monitor_rc;

	/* Pump cleanup is complete before this source-contract fallback. */
	source_restore_rc = qcom_battmgr_oneplus_fixed_5v_locked(battmgr);
	if (!ret && source_restore_rc)
		ret = source_restore_rc;
	if (battmgr->oneplus_policy_started_at)
		battmgr->oneplus_policy_elapsed_ms = jiffies_to_msecs(
			jiffies - battmgr->oneplus_policy_started_at);
	battmgr->oneplus_policy_ramp_rc = ramp_rc;
	battmgr->oneplus_policy_cp_run_rc = cp_run_rc;
	battmgr->oneplus_policy_cp_cleanup_rc = cp_cleanup_rc;
	battmgr->oneplus_policy_source_restore_rc = source_restore_rc;
	battmgr->oneplus_policy_result = ret;
	battmgr->oneplus_policy_running = false;
	battmgr->oneplus_policy_valid = true;
	battmgr->oneplus_policy_auto_run = false;
	battmgr->oneplus_policy_stage = ret ?
		BATTMGR_ONEPLUS_POLICY_FAILED :
		BATTMGR_ONEPLUS_POLICY_COMPLETE;
	mutex_unlock(&battmgr->lock);
}

static ssize_t oneplus_pps_policy_2to1_1a_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	enum qcom_battmgr_oneplus_policy_stop_reason reason;
	enum qcom_battmgr_oneplus_policy_stage stage;
	u32 source_request_mv;
	u32 feedback_decisions;
	u32 feedback_up;
	u32 feedback_down;
	u32 feedback_hold;
	u32 feedback_last_ibus_ua;
	u32 retarget_attempts;
	u32 retarget_completed;
	u32 retarget_first_from_mv;
	u32 retarget_last_to_mv;
	u32 retarget_last_pre_vbus_uv;
	u32 retarget_last_post_vbus_uv;
	u32 monitor_checks;
	u32 elapsed_ms;
	u32 vbat_uv;
	u32 ibus_ua;
	u32 ibus_low_checks;
	u32 auto_attempts;
	u32 auto_starts;
	bool auto_enabled;
	bool auto_armed;
	bool running;
	bool valid;
	int source_restore_rc;
	int cp_cleanup_rc;
	int cp_cancel_rc;
	int cp_run_rc;
	int monitor_rc;
	int temp_decic;
	int feedback_last_delta_mv;
	int retarget_rc;
	int auto_last_rc;
	int ramp_rc;
	int result;
	size_t len = 0;

	mutex_lock(&battmgr->lock);
	running = battmgr->oneplus_policy_running;
	valid = battmgr->oneplus_policy_valid;
	stage = battmgr->oneplus_policy_stage;
	reason = battmgr->oneplus_policy_stop_reason;
	result = battmgr->oneplus_policy_result;
	ramp_rc = battmgr->oneplus_policy_ramp_rc;
	cp_run_rc = battmgr->oneplus_policy_cp_run_rc;
	cp_cancel_rc = battmgr->oneplus_policy_cp_cancel_rc;
	cp_cleanup_rc = battmgr->oneplus_policy_cp_cleanup_rc;
	source_restore_rc = battmgr->oneplus_policy_source_restore_rc;
	monitor_rc = battmgr->oneplus_policy_monitor_rc;
	monitor_checks = battmgr->oneplus_policy_monitor_checks;
	elapsed_ms = battmgr->oneplus_policy_elapsed_ms;
	vbat_uv = battmgr->oneplus_policy_vbat_uv;
	ibus_ua = battmgr->oneplus_policy_ibus_ua;
	ibus_low_checks = battmgr->oneplus_policy_ibus_low_checks;
	auto_enabled = battmgr->oneplus_policy_auto_start;
	auto_armed = battmgr->oneplus_policy_auto_armed;
	auto_attempts = battmgr->oneplus_policy_auto_attempts;
	auto_starts = battmgr->oneplus_policy_auto_starts;
	auto_last_rc = battmgr->oneplus_policy_auto_last_rc;
	temp_decic = battmgr->oneplus_policy_temp_decic;
	source_request_mv = battmgr->oneplus_pps_source_request_mv;
	feedback_decisions = battmgr->oneplus_policy_feedback_decisions;
	feedback_up = battmgr->oneplus_policy_feedback_up;
	feedback_down = battmgr->oneplus_policy_feedback_down;
	feedback_hold = battmgr->oneplus_policy_feedback_hold;
	feedback_last_ibus_ua =
		battmgr->oneplus_policy_feedback_last_ibus_ua;
	feedback_last_delta_mv =
		battmgr->oneplus_policy_feedback_last_delta_mv;
	retarget_attempts = battmgr->oneplus_policy_retarget_attempts;
	retarget_completed = battmgr->oneplus_policy_retarget_completed;
	retarget_first_from_mv =
		battmgr->oneplus_policy_retarget_first_from_mv;
	retarget_last_to_mv = battmgr->oneplus_policy_retarget_last_to_mv;
	retarget_last_pre_vbus_uv =
		battmgr->oneplus_policy_retarget_last_pre_vbus_uv;
	retarget_last_post_vbus_uv =
		battmgr->oneplus_policy_retarget_last_post_vbus_uv;
	retarget_rc = battmgr->oneplus_policy_retarget_rc;
	if (running && battmgr->oneplus_policy_started_at)
		elapsed_ms = max_t(u32, elapsed_ms, jiffies_to_msecs(
			jiffies - battmgr->oneplus_policy_started_at));
	mutex_unlock(&battmgr->lock);

	if (!running && !valid) {
		if (!auto_enabled)
			return sysfs_emit(buf, "not_run\n");
		return sysfs_emit(buf,
			"revision=%s state=idle running=0 rc=0 reason=none elapsed_ms=0\n"
			"auto=enabled:1/armed:%u/attempts:%u/starts:%u/last_rc:%d\n",
			BATTMGR_ONEPLUS_POLICY_REVISION, auto_armed,
			auto_attempts, auto_starts, auto_last_rc);
	}

	len += sysfs_emit_at(buf, len,
		"revision=%s state=%s running=%u rc=%d reason=%s elapsed_ms=%u\n",
		BATTMGR_ONEPLUS_POLICY_REVISION,
		qcom_battmgr_oneplus_policy_stage_name(stage), running,
		result, qcom_battmgr_oneplus_policy_reason_name(reason),
		elapsed_ms);
	len += sysfs_emit_at(buf, len,
		"auto=enabled:%u/armed:%u/attempts:%u/starts:%u/last_rc:%d\n",
		auto_enabled, auto_armed, auto_attempts, auto_starts,
		auto_last_rc);
	len += sysfs_emit_at(buf, len,
		"limits=2to1/1750mA/900000ms monitor_ms=%u vbat_stop_uv=%u temp_decic=%d..%d secondary=off\n",
		BATTMGR_ONEPLUS_POLICY_CHECK_MS,
		BATTMGR_ONEPLUS_POLICY_VBAT_MAX_UV,
		BATTMGR_ONEPLUS_POLICY_TEMP_MIN_DECIC,
		BATTMGR_ONEPLUS_POLICY_TEMP_MAX_DECIC);
	len += sysfs_emit_at(buf, len,
		"telemetry=checks:%u/vbat_uv:%u/temp_decic:%d/source_mv:%u monitor_rc=%d\n",
		monitor_checks, vbat_uv, temp_decic, source_request_mv,
		monitor_rc);
	len += sysfs_emit_at(buf, len,
		"input_guard=ibus_ua:%u/low_checks:%u/limit:%u/min_ua:%u\n",
		ibus_ua, ibus_low_checks,
		BATTMGR_ONEPLUS_POLICY_IBUS_LOW_CHECKS,
		BATTMGR_ONEPLUS_POLICY_IBUS_MIN_UA);
	len += sysfs_emit_at(buf, len,
		"feedback=decisions:%u/limit:%u/up:%u/down:%u/hold:%u/target_ua:%u/deadband_ua:%u/last_ibus_ua:%u/last_delta_mv:%d\n",
		feedback_decisions, BATTMGR_ONEPLUS_POLICY_FEEDBACK_DECISIONS,
		feedback_up, feedback_down, feedback_hold,
		BATTMGR_ONEPLUS_POLICY_FEEDBACK_TARGET_IBUS_UA,
		BATTMGR_ONEPLUS_POLICY_FEEDBACK_DEADBAND_UA,
		feedback_last_ibus_ua, feedback_last_delta_mv);
	len += sysfs_emit_at(buf, len,
		"requests=attempts:%u/completed:%u/from_mv:%u/to_mv:%u/pre_vbus_uv:%u/post_vbus_uv:%u/rc:%d\n",
		retarget_attempts, retarget_completed,
		retarget_first_from_mv, retarget_last_to_mv,
		retarget_last_pre_vbus_uv, retarget_last_post_vbus_uv,
		retarget_rc);
	len += sysfs_emit_at(buf, len,
		"cleanup=ramp_rc:%d/cp_run_rc:%d/cp_cancel_rc:%d/cp_cleanup_rc:%d/source_5v_rc:%d\n",
		ramp_rc, cp_run_rc, cp_cancel_rc, cp_cleanup_rc,
		source_restore_rc);

	return len;
}

static ssize_t oneplus_pps_policy_2to1_1a_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	mutex_lock(&battmgr->lock);
	if (!enable) {
		/* A user stop suppresses restart until a real detach rearms it. */
		battmgr->oneplus_policy_auto_armed = false;
		if (!battmgr->oneplus_policy_running) {
			mutex_unlock(&battmgr->lock);
			return count;
		}
		qcom_battmgr_oneplus_policy_stop_locked(
			battmgr, BATTMGR_ONEPLUS_POLICY_STOP_USER, 0);
		mutex_unlock(&battmgr->lock);
		qcom_battmgr_oneplus_policy_cancel_cp(battmgr);
		return count;
	}

	ret = qcom_battmgr_oneplus_policy_start_locked(battmgr, false);
	mutex_unlock(&battmgr->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(oneplus_pps_policy_2to1_1a);

static struct attribute *qcom_battmgr_oneplus_attrs[] = {
	&dev_attr_oneplus_pps_capabilities.attr,
	NULL,
};

static const struct attribute_group qcom_battmgr_oneplus_group = {
	.attrs = qcom_battmgr_oneplus_attrs,
};

static struct attribute *qcom_battmgr_oneplus_fixed_5v_attrs[] = {
	&dev_attr_oneplus_fixed_5v.attr,
	NULL,
};

static const struct attribute_group qcom_battmgr_oneplus_fixed_5v_group = {
	.attrs = qcom_battmgr_oneplus_fixed_5v_attrs,
};

static struct attribute *qcom_battmgr_oneplus_pps_test_attrs[] = {
	&dev_attr_oneplus_pps_5v5_1a.attr,
	NULL,
};

static const struct attribute_group qcom_battmgr_oneplus_pps_test_group = {
	.attrs = qcom_battmgr_oneplus_pps_test_attrs,
};

static struct attribute *qcom_battmgr_oneplus_pps_ramp_test_attrs[] = {
	&dev_attr_oneplus_pps_ramp_9v_1a.attr,
	NULL,
};

static const struct attribute_group qcom_battmgr_oneplus_pps_ramp_test_group = {
	.attrs = qcom_battmgr_oneplus_pps_ramp_test_attrs,
};

static struct attribute *qcom_battmgr_oneplus_pps_ramp_9v2_test_attrs[] = {
	&dev_attr_oneplus_pps_ramp_9v2_1a.attr,
	NULL,
};

static const struct attribute_group qcom_battmgr_oneplus_pps_ramp_9v2_test_group = {
	.attrs = qcom_battmgr_oneplus_pps_ramp_9v2_test_attrs,
};

static struct attribute *qcom_battmgr_oneplus_pps_ramp_2to1_test_attrs[] = {
	&dev_attr_oneplus_pps_ramp_2to1_1a.attr,
	NULL,
};

static const struct attribute_group qcom_battmgr_oneplus_pps_ramp_2to1_test_group = {
	.attrs = qcom_battmgr_oneplus_pps_ramp_2to1_test_attrs,
};

static struct attribute *qcom_battmgr_oneplus_pps_run_2to1_test_attrs[] = {
	&dev_attr_oneplus_pps_run_2to1_5m.attr,
	NULL,
};

static const struct attribute_group qcom_battmgr_oneplus_pps_run_2to1_test_group = {
	.attrs = qcom_battmgr_oneplus_pps_run_2to1_test_attrs,
};

static struct attribute *qcom_battmgr_oneplus_pps_secondary_test_attrs[] = {
	&dev_attr_oneplus_pps_secondary_500ms.attr,
	NULL,
};

static const struct attribute_group qcom_battmgr_oneplus_pps_secondary_test_group = {
	.attrs = qcom_battmgr_oneplus_pps_secondary_test_attrs,
};

static struct attribute *qcom_battmgr_oneplus_pps_dual_test_attrs[] = {
	&dev_attr_oneplus_pps_dual_500ms.attr,
	&dev_attr_oneplus_pps_dual_auto_once.attr,
	&dev_attr_oneplus_pps_dual_auto_continuous.attr,
	NULL,
};

static const struct attribute_group qcom_battmgr_oneplus_pps_dual_test_group = {
	.attrs = qcom_battmgr_oneplus_pps_dual_test_attrs,
};

static struct attribute *qcom_battmgr_oneplus_pps_policy_attrs[] = {
	&dev_attr_oneplus_pps_policy_2to1_1a.attr,
	NULL,
};

static const struct attribute_group qcom_battmgr_oneplus_pps_policy_group = {
	.attrs = qcom_battmgr_oneplus_pps_policy_attrs,
};

static int qcom_battmgr_request(struct qcom_battmgr *battmgr, void *data, size_t len)
{
	unsigned long left;
	int ret;

	reinit_completion(&battmgr->ack);

	battmgr->error = 0;

	ret = pmic_glink_send(battmgr->client, data, len);
	if (ret < 0)
		return ret;

	left = wait_for_completion_timeout(&battmgr->ack, HZ);
	if (!left)
		return -ETIMEDOUT;

	return battmgr->error;
}

static int qcom_battmgr_request_property(struct qcom_battmgr *battmgr, int opcode,
					 int property, u32 value)
{
	struct qcom_battmgr_property_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(opcode),
		.battery = cpu_to_le32(0),
		.property = cpu_to_le32(property),
		.value = cpu_to_le32(value),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static int qcom_battmgr_update_status(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_update_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_BAT_STATUS),
		.battery_id = cpu_to_le32(0),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static int qcom_battmgr_update_info(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_update_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_BAT_INFO),
		.battery_id = cpu_to_le32(0),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static int qcom_battmgr_update_charge_time(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_charge_time_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_BAT_CHARGE_TIME),
		.battery_id = cpu_to_le32(0),
		.percent = cpu_to_le32(100),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static int qcom_battmgr_update_discharge_time(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_discharge_time_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_BAT_DISCHARGE_TIME),
		.battery_id = cpu_to_le32(0),
		.rate = cpu_to_le32(0),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static const u8 sm8350_bat_prop_map[] = {
	[POWER_SUPPLY_PROP_STATUS] = BATT_STATUS,
	[POWER_SUPPLY_PROP_HEALTH] = BATT_HEALTH,
	[POWER_SUPPLY_PROP_PRESENT] = BATT_PRESENT,
	[POWER_SUPPLY_PROP_CHARGE_TYPE] = BATT_CHG_TYPE,
	[POWER_SUPPLY_PROP_CAPACITY] = BATT_CAPACITY,
	[POWER_SUPPLY_PROP_VOLTAGE_OCV] = BATT_VOLT_OCV,
	[POWER_SUPPLY_PROP_VOLTAGE_NOW] = BATT_VOLT_NOW,
	[POWER_SUPPLY_PROP_VOLTAGE_MAX] = BATT_VOLT_MAX,
	[POWER_SUPPLY_PROP_CURRENT_NOW] = BATT_CURR_NOW,
	[POWER_SUPPLY_PROP_TEMP] = BATT_TEMP,
	[POWER_SUPPLY_PROP_TECHNOLOGY] = BATT_TECHNOLOGY,
	[POWER_SUPPLY_PROP_CHARGE_COUNTER] =  BATT_CHG_COUNTER,
	[POWER_SUPPLY_PROP_CYCLE_COUNT] = BATT_CYCLE_COUNT,
	[POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN] =  BATT_CHG_FULL_DESIGN,
	[POWER_SUPPLY_PROP_CHARGE_FULL] = BATT_CHG_FULL,
	[POWER_SUPPLY_PROP_MODEL_NAME] = BATT_MODEL_NAME,
	[POWER_SUPPLY_PROP_TIME_TO_FULL_AVG] = BATT_TTF_AVG,
	[POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG] = BATT_TTE_AVG,
	[POWER_SUPPLY_PROP_INTERNAL_RESISTANCE] = BATT_RESISTANCE,
	[POWER_SUPPLY_PROP_STATE_OF_HEALTH] = BATT_SOH,
	[POWER_SUPPLY_PROP_POWER_NOW] = BATT_POWER_NOW,
	[POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD] = BATT_CHG_CTRL_START_THR,
	[POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD] = BATT_CHG_CTRL_END_THR,
};

static int qcom_battmgr_bat_sm8350_update(struct qcom_battmgr *battmgr,
					  enum power_supply_property psp)
{
	unsigned int prop;
	int ret;

	if (psp >= ARRAY_SIZE(sm8350_bat_prop_map))
		return -EINVAL;

	prop = sm8350_bat_prop_map[psp];

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_request_property(battmgr, BATTMGR_BAT_PROPERTY_GET, prop, 0);
	mutex_unlock(&battmgr->lock);

	return ret;
}

static int qcom_battmgr_bat_sc8280xp_update(struct qcom_battmgr *battmgr,
					    enum power_supply_property psp)
{
	int ret;

	mutex_lock(&battmgr->lock);

	if (!battmgr->info.valid) {
		ret = qcom_battmgr_update_info(battmgr);
		if (ret < 0)
			goto out_unlock;
		battmgr->info.valid = true;
	}

	ret = qcom_battmgr_update_status(battmgr);
	if (ret < 0)
		goto out_unlock;

	if (psp == POWER_SUPPLY_PROP_TIME_TO_FULL_AVG) {
		ret = qcom_battmgr_update_charge_time(battmgr);
		if (ret < 0) {
			ret = -ENODATA;
			goto out_unlock;
		}
	}

	if (psp == POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG) {
		ret = qcom_battmgr_update_discharge_time(battmgr);
		if (ret < 0) {
			ret = -ENODATA;
			goto out_unlock;
		}
	}

out_unlock:
	mutex_unlock(&battmgr->lock);
	return ret;
}

static int qcom_battmgr_bat_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);
	enum qcom_battmgr_unit unit = battmgr->unit;
	int ret;

	if (!battmgr->service_up)
		return -EAGAIN;

	if (battmgr->variant == QCOM_BATTMGR_SC8280XP ||
	    battmgr->variant == QCOM_BATTMGR_X1E80100)
		ret = qcom_battmgr_bat_sc8280xp_update(battmgr, psp);
	else
		ret = qcom_battmgr_bat_sm8350_update(battmgr, psp);
	if (ret < 0)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = battmgr->status.status;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = battmgr->info.charge_type;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = battmgr->status.health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = battmgr->info.present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = battmgr->info.technology;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = battmgr->info.cycle_count;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = battmgr->info.voltage_max_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = battmgr->info.voltage_max;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battmgr->status.voltage_now;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = battmgr->status.voltage_ocv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = battmgr->status.current_now;
		break;
	case POWER_SUPPLY_PROP_POWER_NOW:
		val->intval = battmgr->status.power_now;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if (unit != QCOM_BATTMGR_UNIT_mAh)
			return -ENODATA;
		val->intval = battmgr->info.design_capacity;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		if (unit != QCOM_BATTMGR_UNIT_mAh)
			return -ENODATA;
		val->intval = battmgr->info.last_full_capacity;
		break;
	case POWER_SUPPLY_PROP_CHARGE_EMPTY:
		if (unit != QCOM_BATTMGR_UNIT_mAh)
			return -ENODATA;
		val->intval = battmgr->info.capacity_low;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (unit != QCOM_BATTMGR_UNIT_mAh)
			return -ENODATA;
		val->intval = battmgr->status.capacity;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = battmgr->info.charge_count;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		if (unit != QCOM_BATTMGR_UNIT_mWh)
			return -ENODATA;
		val->intval = battmgr->info.design_capacity;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		if (unit != QCOM_BATTMGR_UNIT_mWh)
			return -ENODATA;
		val->intval = battmgr->info.last_full_capacity;
		break;
	case POWER_SUPPLY_PROP_ENERGY_EMPTY:
		if (unit != QCOM_BATTMGR_UNIT_mWh)
			return -ENODATA;
		val->intval = battmgr->info.capacity_low;
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		if (unit != QCOM_BATTMGR_UNIT_mWh)
			return -ENODATA;
		val->intval = battmgr->status.capacity;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (battmgr->status.percent == (unsigned int)-1)
			return -ENODATA;
		val->intval = battmgr->status.percent;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = battmgr->status.temperature;
		break;
	case POWER_SUPPLY_PROP_INTERNAL_RESISTANCE:
		val->intval = battmgr->status.resistance;
		break;
	case POWER_SUPPLY_PROP_STATE_OF_HEALTH:
		val->intval = battmgr->status.soh_percent;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		val->intval = battmgr->status.discharge_time;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		val->intval = battmgr->status.charge_time;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		val->intval = battmgr->info.charge_ctrl_start;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		val->intval = battmgr->info.charge_ctrl_end;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURE_YEAR:
		val->intval = battmgr->info.year;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURE_MONTH:
		val->intval = battmgr->info.month;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURE_DAY:
		val->intval = battmgr->info.day;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = battmgr->info.model_number;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = battmgr->info.oem_info;
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = battmgr->info.serial_number;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int qcom_battmgr_set_charge_control(struct qcom_battmgr *battmgr,
					   u32 target_soc, u32 delta_soc)
{
	struct qcom_battmgr_charge_ctrl_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_CHG_CTRL_LIMIT_EN),
		.enable = cpu_to_le32(1),
		.target_soc = cpu_to_le32(target_soc),
		.delta_soc = cpu_to_le32(delta_soc),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static int qcom_battmgr_set_charge_start_threshold(struct qcom_battmgr *battmgr, int start_soc)
{
	u32 target_soc, delta_soc;
	int ret;

	start_soc = clamp(start_soc, CHARGE_CTRL_START_THR_MIN, CHARGE_CTRL_START_THR_MAX);

	/*
	 * If the new start threshold is larger than the old end threshold,
	 * move the end threshold one step (DELTA_SOC) after the new start
	 * threshold.
	 */
	if (start_soc > battmgr->info.charge_ctrl_end) {
		target_soc = start_soc + CHARGE_CTRL_DELTA_SOC;
		target_soc = min_t(u32, target_soc, CHARGE_CTRL_END_THR_MAX);
		delta_soc = target_soc - start_soc;
		delta_soc = min_t(u32, delta_soc, CHARGE_CTRL_DELTA_SOC);
	} else {
		target_soc =  battmgr->info.charge_ctrl_end;
		delta_soc = battmgr->info.charge_ctrl_end - start_soc;
	}

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_set_charge_control(battmgr, target_soc, delta_soc);
	mutex_unlock(&battmgr->lock);
	if (!ret) {
		battmgr->info.charge_ctrl_start = start_soc;
		battmgr->info.charge_ctrl_end = target_soc;
	}

	return 0;
}

static int qcom_battmgr_set_charge_end_threshold(struct qcom_battmgr *battmgr, int end_soc)
{
	u32 delta_soc = CHARGE_CTRL_DELTA_SOC;
	int ret;

	end_soc = clamp(end_soc, CHARGE_CTRL_END_THR_MIN, CHARGE_CTRL_END_THR_MAX);

	if (battmgr->info.charge_ctrl_start && end_soc > battmgr->info.charge_ctrl_start)
		delta_soc = end_soc - battmgr->info.charge_ctrl_start;

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_set_charge_control(battmgr, end_soc, delta_soc);
	mutex_unlock(&battmgr->lock);
	if (!ret) {
		battmgr->info.charge_ctrl_start = end_soc - delta_soc;
		battmgr->info.charge_ctrl_end = end_soc;
	}

	return 0;
}

static int qcom_battmgr_charge_control_thresholds_init(struct qcom_battmgr *battmgr)
{
	int ret;
	u8 en, end_soc, start_soc, delta_soc;

	ret = nvmem_cell_read_u8(battmgr->dev->parent, "charge_limit_en", &en);
	if (!ret && en != 0) {
		ret = nvmem_cell_read_u8(battmgr->dev->parent, "charge_limit_end", &end_soc);
		if (ret < 0)
			return ret;

		ret = nvmem_cell_read_u8(battmgr->dev->parent, "charge_limit_delta", &delta_soc);
		if (ret < 0)
			return ret;

		if (delta_soc >= end_soc)
			return -EINVAL;

		start_soc = end_soc - delta_soc;
		end_soc = clamp(end_soc, CHARGE_CTRL_END_THR_MIN, CHARGE_CTRL_END_THR_MAX);
		start_soc = clamp(start_soc, CHARGE_CTRL_START_THR_MIN, CHARGE_CTRL_START_THR_MAX);

		battmgr->info.charge_ctrl_start = start_soc;
		battmgr->info.charge_ctrl_end = end_soc;
	}

	return 0;
}

static int qcom_battmgr_bat_is_writeable(struct power_supply *psy,
					 enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return 1;
	default:
		return 0;
	}

	return 0;
}

static int qcom_battmgr_bat_set_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 const union power_supply_propval *pval)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);

	if (!battmgr->service_up)
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		return qcom_battmgr_set_charge_start_threshold(battmgr, pval->intval);
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return qcom_battmgr_set_charge_end_threshold(battmgr, pval->intval);
	default:
		return -EINVAL;
	}

	return 0;
}

static const enum power_supply_property sc8280xp_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_EMPTY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_EMPTY,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MANUFACTURE_YEAR,
	POWER_SUPPLY_PROP_MANUFACTURE_MONTH,
	POWER_SUPPLY_PROP_MANUFACTURE_DAY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static const struct power_supply_desc sc8280xp_bat_psy_desc = {
	.name = "qcom-battmgr-bat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = sc8280xp_bat_props,
	.num_properties = ARRAY_SIZE(sc8280xp_bat_props),
	.get_property = qcom_battmgr_bat_get_property,
};

static const enum power_supply_property x1e80100_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_EMPTY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_EMPTY,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MANUFACTURE_YEAR,
	POWER_SUPPLY_PROP_MANUFACTURE_MONTH,
	POWER_SUPPLY_PROP_MANUFACTURE_DAY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
};

static const struct power_supply_desc x1e80100_bat_psy_desc = {
	.name = "qcom-battmgr-bat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = x1e80100_bat_props,
	.num_properties = ARRAY_SIZE(x1e80100_bat_props),
	.get_property = qcom_battmgr_bat_get_property,
	.set_property = qcom_battmgr_bat_set_property,
	.property_is_writeable = qcom_battmgr_bat_is_writeable,
};

static const enum power_supply_property sm8350_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_INTERNAL_RESISTANCE,
	POWER_SUPPLY_PROP_STATE_OF_HEALTH,
	POWER_SUPPLY_PROP_POWER_NOW,
};

static const struct power_supply_desc sm8350_bat_psy_desc = {
	.name = "qcom-battmgr-bat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = sm8350_bat_props,
	.num_properties = ARRAY_SIZE(sm8350_bat_props),
	.get_property = qcom_battmgr_bat_get_property,
};

static const enum power_supply_property sm8550_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_INTERNAL_RESISTANCE,
	POWER_SUPPLY_PROP_STATE_OF_HEALTH,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
};

static const struct power_supply_desc sm8550_bat_psy_desc = {
	.name = "qcom-battmgr-bat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = sm8550_bat_props,
	.num_properties = ARRAY_SIZE(sm8550_bat_props),
	.get_property = qcom_battmgr_bat_get_property,
	.set_property = qcom_battmgr_bat_set_property,
	.property_is_writeable = qcom_battmgr_bat_is_writeable,
};

static int qcom_battmgr_ac_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);
	int ret;

	if (!battmgr->service_up)
		return -EAGAIN;

	ret = qcom_battmgr_bat_sc8280xp_update(battmgr, psp);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = battmgr->ac.online;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const enum power_supply_property sc8280xp_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc sc8280xp_ac_psy_desc = {
	.name = "qcom-battmgr-ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = sc8280xp_ac_props,
	.num_properties = ARRAY_SIZE(sc8280xp_ac_props),
	.get_property = qcom_battmgr_ac_get_property,
};

static const u8 sm8350_usb_prop_map[] = {
	[POWER_SUPPLY_PROP_ONLINE] = USB_ONLINE,
	[POWER_SUPPLY_PROP_VOLTAGE_NOW] = USB_VOLT_NOW,
	[POWER_SUPPLY_PROP_VOLTAGE_MAX] = USB_VOLT_MAX,
	[POWER_SUPPLY_PROP_CURRENT_NOW] = USB_CURR_NOW,
	[POWER_SUPPLY_PROP_CURRENT_MAX] = USB_CURR_MAX,
	[POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT] = USB_INPUT_CURR_LIMIT,
	[POWER_SUPPLY_PROP_USB_TYPE] = USB_TYPE,
};

static int qcom_battmgr_usb_sm8350_update(struct qcom_battmgr *battmgr,
					  enum power_supply_property psp)
{
	unsigned int prop;
	int ret;

	if (psp >= ARRAY_SIZE(sm8350_usb_prop_map))
		return -EINVAL;

	prop = sm8350_usb_prop_map[psp];

	/* Oplus firmware reports the effective charger type in USB_ADAP_TYPE */
	if (battmgr->variant == QCOM_BATTMGR_OPLUS_SM8650 &&
	    psp == POWER_SUPPLY_PROP_USB_TYPE)
		prop = USB_ADAP_TYPE;

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_request_property(battmgr, BATTMGR_USB_PROPERTY_GET, prop, 0);
	mutex_unlock(&battmgr->lock);

	return ret;
}

static int qcom_battmgr_usb_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);
	int ret;

	if (!battmgr->service_up)
		return -EAGAIN;

	if (battmgr->variant == QCOM_BATTMGR_SC8280XP ||
	    battmgr->variant == QCOM_BATTMGR_X1E80100)
		ret = qcom_battmgr_bat_sc8280xp_update(battmgr, psp);
	else
		ret = qcom_battmgr_usb_sm8350_update(battmgr, psp);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = battmgr->usb.online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battmgr->usb.voltage_now;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = battmgr->usb.voltage_max;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = battmgr->usb.current_now;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = battmgr->usb.current_max;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = battmgr->usb.current_limit;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = battmgr->usb.usb_type;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const enum power_supply_property sc8280xp_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc sc8280xp_usb_psy_desc = {
	.name = "qcom-battmgr-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sc8280xp_usb_props,
	.num_properties = ARRAY_SIZE(sc8280xp_usb_props),
	.get_property = qcom_battmgr_usb_get_property,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
		     BIT(POWER_SUPPLY_USB_TYPE_SDP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_ACA)     |
		     BIT(POWER_SUPPLY_USB_TYPE_C)       |
		     BIT(POWER_SUPPLY_USB_TYPE_PD)      |
		     BIT(POWER_SUPPLY_USB_TYPE_PD_DRP)  |
		     BIT(POWER_SUPPLY_USB_TYPE_PD_PPS)  |
		     BIT(POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID),
};

static const enum power_supply_property sm8350_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static const struct power_supply_desc sm8350_usb_psy_desc = {
	.name = "qcom-battmgr-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sm8350_usb_props,
	.num_properties = ARRAY_SIZE(sm8350_usb_props),
	.get_property = qcom_battmgr_usb_get_property,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
		     BIT(POWER_SUPPLY_USB_TYPE_SDP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_ACA)     |
		     BIT(POWER_SUPPLY_USB_TYPE_C)       |
		     BIT(POWER_SUPPLY_USB_TYPE_PD)      |
		     BIT(POWER_SUPPLY_USB_TYPE_PD_DRP)  |
		     BIT(POWER_SUPPLY_USB_TYPE_PD_PPS)  |
		     BIT(POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID),
};

static const u8 sm8350_wls_prop_map[] = {
	[POWER_SUPPLY_PROP_ONLINE] = WLS_ONLINE,
	[POWER_SUPPLY_PROP_VOLTAGE_NOW] = WLS_VOLT_NOW,
	[POWER_SUPPLY_PROP_VOLTAGE_MAX] = WLS_VOLT_MAX,
	[POWER_SUPPLY_PROP_CURRENT_NOW] = WLS_CURR_NOW,
	[POWER_SUPPLY_PROP_CURRENT_MAX] = WLS_CURR_MAX,
};

static int qcom_battmgr_wls_sm8350_update(struct qcom_battmgr *battmgr,
					  enum power_supply_property psp)
{
	unsigned int prop;
	int ret;

	if (psp >= ARRAY_SIZE(sm8350_wls_prop_map))
		return -EINVAL;

	prop = sm8350_wls_prop_map[psp];

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_request_property(battmgr, BATTMGR_WLS_PROPERTY_GET, prop, 0);
	mutex_unlock(&battmgr->lock);

	return ret;
}

static int qcom_battmgr_wls_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);
	int ret;

	if (!battmgr->service_up)
		return -EAGAIN;

	if (battmgr->variant == QCOM_BATTMGR_SC8280XP ||
	    battmgr->variant == QCOM_BATTMGR_X1E80100)
		ret = qcom_battmgr_bat_sc8280xp_update(battmgr, psp);
	else
		ret = qcom_battmgr_wls_sm8350_update(battmgr, psp);
	if (ret < 0)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = battmgr->wireless.online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battmgr->wireless.voltage_now;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = battmgr->wireless.voltage_max;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = battmgr->wireless.current_now;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = battmgr->wireless.current_max;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const enum power_supply_property sc8280xp_wls_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc sc8280xp_wls_psy_desc = {
	.name = "qcom-battmgr-wls",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = sc8280xp_wls_props,
	.num_properties = ARRAY_SIZE(sc8280xp_wls_props),
	.get_property = qcom_battmgr_wls_get_property,
};

static const enum power_supply_property sm8350_wls_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static const struct power_supply_desc sm8350_wls_psy_desc = {
	.name = "qcom-battmgr-wls",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = sm8350_wls_props,
	.num_properties = ARRAY_SIZE(sm8350_wls_props),
	.get_property = qcom_battmgr_wls_get_property,
};

static void qcom_battmgr_notification(struct qcom_battmgr *battmgr,
				      const struct qcom_battmgr_message *msg,
				      int len)
{
	size_t payload_len = len - sizeof(struct pmic_glink_hdr);
	unsigned int notification;

	if (payload_len != sizeof(msg->notification)) {
		dev_warn(battmgr->dev, "ignoring notification with invalid length\n");
		return;
	}

	notification = le32_to_cpu(msg->notification);
	notification &= 0xff;
	switch (notification) {
	case NOTIF_BAT_INFO:
		battmgr->info.valid = false;
		fallthrough;
	case NOTIF_BAT_STATUS:
	case NOTIF_BAT_PROPERTY:
	case NOTIF_BAT_CHARGING_STATE:
		if (battmgr->bat_psy)
			power_supply_changed(battmgr->bat_psy);
		break;
	case NOTIF_USB_PROPERTY:
		power_supply_changed(battmgr->usb_psy);
		break;
	case NOTIF_WLS_PROPERTY:
		if (battmgr->wls_psy)
			power_supply_changed(battmgr->wls_psy);
		break;
	case NOTIF_OPLUS_OTG_ENABLE:
	case NOTIF_OPLUS_OTG_DISABLE:
		if (battmgr->variant != QCOM_BATTMGR_OPLUS_SM8650)
			goto unknown;
		battmgr->otg_vbus = notification == NOTIF_OPLUS_OTG_ENABLE;
		dev_info(battmgr->dev, "ADSP requested OTG VBUS %s\n",
			 battmgr->otg_vbus ? "on" : "off");
		schedule_work(&battmgr->otg_work);
		power_supply_changed(battmgr->usb_psy);
		break;
	default:
		if (battmgr->variant == QCOM_BATTMGR_OPLUS_SM8650 &&
		    notification >= NOTIF_OPLUS_FIRST &&
		    notification <= NOTIF_OPLUS_LAST) {
			/* charger state chatter: just refresh the charger */
			dev_dbg(battmgr->dev, "oplus notification: %#x\n",
				notification);
			power_supply_changed(battmgr->usb_psy);
			break;
		}
unknown:
		/* Vendor firmware may send private notification IDs. The Qualcomm
		 * downstream driver ignores these as well; property reads still query
		 * the remote service synchronously.
		 */
		dev_dbg_ratelimited(battmgr->dev, "unknown notification: %#x\n",
				    notification);
		break;
	}
}

static void qcom_battmgr_sc8280xp_strcpy(char *dest, const char *src)
{
	size_t len = src[0];

	/* Some firmware versions return Pascal-style strings */
	if (len < BATTMGR_STRING_LEN && len == strnlen(src + 1, BATTMGR_STRING_LEN - 1)) {
		memcpy(dest, src + 1, len);
		dest[len] = '\0';
	} else {
		strscpy(dest, src, BATTMGR_STRING_LEN);
	}
}

static unsigned int qcom_battmgr_sc8280xp_parse_technology(const char *chemistry)
{
	if ((!strncmp(chemistry, "LIO", BATTMGR_CHEMISTRY_LEN)) ||
	    (!strncmp(chemistry, "OOI", BATTMGR_CHEMISTRY_LEN)))
		return POWER_SUPPLY_TECHNOLOGY_LION;
	if (!strncmp(chemistry, "LIP", BATTMGR_CHEMISTRY_LEN) ||
	    !strncmp(chemistry, "LiP", BATTMGR_CHEMISTRY_LEN))
		return POWER_SUPPLY_TECHNOLOGY_LIPO;

	pr_err("Unknown battery technology '%s'\n", chemistry);
	return POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
}

static unsigned int qcom_battmgr_sc8280xp_convert_temp(unsigned int temperature)
{
	return DIV_ROUND_CLOSEST(temperature, 10);
}

static void qcom_battmgr_sc8280xp_callback(struct qcom_battmgr *battmgr,
					   const struct qcom_battmgr_message *resp,
					   size_t len)
{
	unsigned int opcode = le32_to_cpu(resp->hdr.opcode);
	unsigned int source;
	unsigned int state;
	size_t payload_len = len - sizeof(struct pmic_glink_hdr);

	if (payload_len < sizeof(__le32)) {
		dev_warn(battmgr->dev, "invalid payload length for %#x: %zd\n",
			 opcode, len);
		return;
	}

	switch (opcode) {
	case BATTMGR_REQUEST_NOTIFICATION:
		battmgr->error = 0;
		break;
	case BATTMGR_BAT_INFO:
		/* some firmware versions report an extra __le32 at the end of the payload */
		if (payload_len != sizeof(resp->info) &&
		    payload_len != (sizeof(resp->info) + sizeof(__le32))) {
			dev_warn(battmgr->dev,
				 "invalid payload length for battery information request: %zd\n",
				 payload_len);
			battmgr->error = -ENODATA;
			return;
		}

		battmgr->unit = le32_to_cpu(resp->info.power_unit);

		battmgr->info.present = true;
		battmgr->info.design_capacity = le32_to_cpu(resp->info.design_capacity) * 1000;
		battmgr->info.last_full_capacity = le32_to_cpu(resp->info.last_full_capacity) * 1000;
		battmgr->info.voltage_max_design = le32_to_cpu(resp->info.design_voltage) * 1000;
		battmgr->info.capacity_low = le32_to_cpu(resp->info.capacity_low) * 1000;
		battmgr->info.cycle_count = le32_to_cpu(resp->info.cycle_count);
		qcom_battmgr_sc8280xp_strcpy(battmgr->info.model_number, resp->info.model_number);
		qcom_battmgr_sc8280xp_strcpy(battmgr->info.serial_number, resp->info.serial_number);
		battmgr->info.technology = qcom_battmgr_sc8280xp_parse_technology(resp->info.battery_chemistry);
		qcom_battmgr_sc8280xp_strcpy(battmgr->info.oem_info, resp->info.oem_info);
		battmgr->info.day = resp->info.day;
		battmgr->info.month = resp->info.month;
		battmgr->info.year = le16_to_cpu(resp->info.year);
		break;
	case BATTMGR_BAT_STATUS:
		if (payload_len != sizeof(resp->status)) {
			dev_warn(battmgr->dev,
				 "invalid payload length for battery status request: %zd\n",
				 payload_len);
			battmgr->error = -ENODATA;
			return;
		}

		state = le32_to_cpu(resp->status.battery_state);
		if (state & BIT(0))
			battmgr->status.status = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (state & BIT(1))
			battmgr->status.status = POWER_SUPPLY_STATUS_CHARGING;
		else
			battmgr->status.status = POWER_SUPPLY_STATUS_NOT_CHARGING;

		battmgr->status.capacity = le32_to_cpu(resp->status.capacity) * 1000;
		battmgr->status.power_now = le32_to_cpu(resp->status.rate) * 1000;
		battmgr->status.voltage_now = le32_to_cpu(resp->status.battery_voltage) * 1000;
		battmgr->status.temperature = qcom_battmgr_sc8280xp_convert_temp(le32_to_cpu(resp->status.temperature));

		source = le32_to_cpu(resp->status.charging_source);
		battmgr->ac.online = source == BATTMGR_CHARGING_SOURCE_AC;
		battmgr->usb.online = source == BATTMGR_CHARGING_SOURCE_USB;
		battmgr->wireless.online = source == BATTMGR_CHARGING_SOURCE_WIRELESS;
		if (battmgr->info.last_full_capacity != 0) {
			/*
			 * 100 * battmgr->status.capacity can overflow a 32bit
			 * unsigned integer. FW readings are in m{W/A}h, which
			 * are multiplied by 1000 converting them to u{W/A}h,
			 * the format the power_supply API expects.
			 * To avoid overflow use the original value for dividend
			 * and convert the divider back to m{W/A}h, which can be
			 * done without any loss of precision.
			 */
			battmgr->status.percent =
				(100 * le32_to_cpu(resp->status.capacity)) /
				(battmgr->info.last_full_capacity / 1000);
		} else {
			/*
			 * Let the sysfs handler know no data is available at
			 * this time.
			 */
			battmgr->status.percent = (unsigned int)-1;
		}
		break;
	case BATTMGR_BAT_DISCHARGE_TIME:
		battmgr->status.discharge_time = le32_to_cpu(resp->time);
		break;
	case BATTMGR_BAT_CHARGE_TIME:
		battmgr->status.charge_time = le32_to_cpu(resp->time);
		break;
	case BATTMGR_CHG_CTRL_LIMIT_EN:
		battmgr->error = 0;
		break;
	default:
		dev_warn(battmgr->dev, "unknown message %#x\n", opcode);
		break;
	}

	complete(&battmgr->ack);
}

static int qcom_battmgr_get_batt_voltage_uv(struct qcom_battmgr *battmgr);

static void qcom_battmgr_sm8350_callback(struct qcom_battmgr *battmgr,
					 const struct qcom_battmgr_message *resp,
					 size_t len)
{
	unsigned int property;
	unsigned int opcode = le32_to_cpu(resp->hdr.opcode);
	size_t payload_len = len - sizeof(struct pmic_glink_hdr);
	unsigned int result;
	unsigned int val;

	if (payload_len < sizeof(__le32)) {
		dev_warn(battmgr->dev, "invalid payload length for %#x: %zd\n",
			 opcode, len);
		return;
	}

	switch (opcode) {
	case BATTMGR_BAT_PROPERTY_GET:
		property = le32_to_cpu(resp->intval.property);
		if (property == BATT_MODEL_NAME) {
			if (payload_len != sizeof(resp->strval)) {
				dev_warn(battmgr->dev,
					 "invalid payload length for BATT_MODEL_NAME request: %zd\n",
					 payload_len);
				battmgr->error = -ENODATA;
				return;
			}
		} else {
			if (payload_len != sizeof(resp->intval)) {
				dev_warn(battmgr->dev,
					 "invalid payload length for %#x request: %zd\n",
					 property, payload_len);
				battmgr->error = -ENODATA;
				return;
			}

			battmgr->error = le32_to_cpu(resp->intval.result);
			if (battmgr->error)
				goto out_complete;
		}

		switch (property) {
		case BATT_STATUS:
			battmgr->status.status = le32_to_cpu(resp->intval.value);
			break;
		case BATT_HEALTH:
			battmgr->status.health = le32_to_cpu(resp->intval.value);
			break;
		case BATT_PRESENT:
			battmgr->info.present = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_TYPE:
			battmgr->info.charge_type = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CAPACITY:
			battmgr->status.percent = le32_to_cpu(resp->intval.value) / 100;
			break;
		case BATT_SOH:
			battmgr->status.soh_percent = le32_to_cpu(resp->intval.value);
			break;
		case BATT_VOLT_OCV:
			battmgr->status.voltage_ocv = le32_to_cpu(resp->intval.value);
			break;
		case BATT_VOLT_NOW:
			val = le32_to_cpu(resp->intval.value);
			/*
			 * Do NOT read the AP-side gauge here: this runs in the
			 * pmic_glink RX callback context, where the gauge's I2C
			 * transfer is not safe (it hangs the device on plug-in).
			 * Oplus callers use qcom_battmgr_oneplus_vbat_uv() from a
			 * sleepable context instead.
			 */
			battmgr->status.voltage_now = val;
			if (val) {
				WRITE_ONCE(battmgr->oneplus_pps_vbat_cache_uv, val);
				WRITE_ONCE(battmgr->oneplus_pps_vbat_cache_at,
					   jiffies);
			}
			break;
		case BATT_VOLT_MAX:
			battmgr->info.voltage_max = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CURR_NOW:
			battmgr->status.current_now = le32_to_cpu(resp->intval.value);
			break;
		case BATT_TEMP:
			val = le32_to_cpu(resp->intval.value);
			battmgr->status.temperature = DIV_ROUND_CLOSEST(val, 10);
			break;
		case BATT_TECHNOLOGY:
			battmgr->info.technology = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_COUNTER:
			battmgr->info.charge_count = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CYCLE_COUNT:
			battmgr->info.cycle_count = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_FULL_DESIGN:
			battmgr->info.design_capacity = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_FULL:
			battmgr->info.last_full_capacity = le32_to_cpu(resp->intval.value);
			break;
		case BATT_MODEL_NAME:
			strscpy(battmgr->info.model_number, resp->strval.model, BATTMGR_STRING_LEN);
			break;
		case BATT_TTF_AVG:
			battmgr->status.charge_time = le32_to_cpu(resp->intval.value);
			break;
		case BATT_TTE_AVG:
			battmgr->status.discharge_time = le32_to_cpu(resp->intval.value);
			break;
		case BATT_RESISTANCE:
			battmgr->status.resistance = le32_to_cpu(resp->intval.value);
			break;
		case BATT_POWER_NOW:
			battmgr->status.power_now = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_CTRL_START_THR:
			battmgr->info.charge_ctrl_start = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_CTRL_END_THR:
			battmgr->info.charge_ctrl_end = le32_to_cpu(resp->intval.value);
			break;
		default:
			dev_warn(battmgr->dev, "unknown property %#x\n", property);
			break;
		}
		break;
	case BATTMGR_USB_PROPERTY_GET:
		property = le32_to_cpu(resp->intval.property);
		if (payload_len != sizeof(resp->intval)) {
			dev_warn(battmgr->dev,
				 "invalid payload length for %#x request: %zd\n",
				 property, payload_len);
			battmgr->error = -ENODATA;
			return;
		}

		battmgr->error = le32_to_cpu(resp->intval.result);
		if (battmgr->error)
			goto out_complete;

		switch (property) {
		case USB_ONLINE:
			battmgr->usb.online = le32_to_cpu(resp->intval.value);
			break;
		case USB_VOLT_NOW:
			battmgr->usb.voltage_now = le32_to_cpu(resp->intval.value);
			break;
		case USB_VOLT_MAX:
			battmgr->usb.voltage_max = le32_to_cpu(resp->intval.value);
			break;
		case USB_CURR_NOW:
			battmgr->usb.current_now = le32_to_cpu(resp->intval.value);
			break;
		case USB_CURR_MAX:
			battmgr->usb.current_max = le32_to_cpu(resp->intval.value);
			break;
		case USB_INPUT_CURR_LIMIT:
			battmgr->usb.current_limit = le32_to_cpu(resp->intval.value);
			break;
		case USB_TYPE:
			battmgr->usb.usb_type = le32_to_cpu(resp->intval.value);
			break;
		case USB_ADAP_TYPE:
			val = le32_to_cpu(resp->intval.value);
			if (val >= QTI_USB_TYPE_HVDCP)
				battmgr->usb.usb_type = POWER_SUPPLY_USB_TYPE_DCP;
			else if (val <= POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID)
				battmgr->usb.usb_type = val;
			else
				battmgr->usb.usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			break;
		default:
			dev_warn(battmgr->dev, "unknown property %#x\n", property);
			break;
		}
		break;
	case BATTMGR_WLS_PROPERTY_GET:
		property = le32_to_cpu(resp->intval.property);
		if (payload_len != sizeof(resp->intval)) {
			dev_warn(battmgr->dev,
				 "invalid payload length for %#x request: %zd\n",
				 property, payload_len);
			battmgr->error = -ENODATA;
			return;
		}

		battmgr->error = le32_to_cpu(resp->intval.result);
		if (battmgr->error)
			goto out_complete;

		switch (property) {
		case WLS_ONLINE:
			battmgr->wireless.online = le32_to_cpu(resp->intval.value);
			break;
		case WLS_VOLT_NOW:
			battmgr->wireless.voltage_now = le32_to_cpu(resp->intval.value);
			break;
		case WLS_VOLT_MAX:
			battmgr->wireless.voltage_max = le32_to_cpu(resp->intval.value);
			break;
		case WLS_CURR_NOW:
			battmgr->wireless.current_now = le32_to_cpu(resp->intval.value);
			break;
		case WLS_CURR_MAX:
			battmgr->wireless.current_max = le32_to_cpu(resp->intval.value);
			break;
		default:
			dev_warn(battmgr->dev, "unknown property %#x\n", property);
			break;
		}
		break;
	case BATTMGR_BAT_PROPERTY_SET:
	case BATTMGR_USB_PROPERTY_SET:
		property = le32_to_cpu(resp->intval.property);
		if (payload_len != sizeof(resp->intval)) {
			dev_warn(battmgr->dev,
				 "invalid OnePlus SET response length for %#x: %zd\n",
				 opcode, payload_len);
			battmgr->error = -EMSGSIZE;
			goto out_complete;
		}
		if (!battmgr->oneplus_ext_pending ||
		    opcode != battmgr->oneplus_ext_opcode ||
		    property != battmgr->oneplus_ext_property) {
			dev_warn(battmgr->dev,
				 "unexpected OnePlus SET response %#x property %#x\n",
				 opcode, property);
			battmgr->error = -EPROTO;
			goto out_complete;
		}

		result = le32_to_cpu(resp->intval.result);
		if (result) {
			dev_warn(battmgr->dev,
				 "OnePlus SET %#x property %#x failed: %#x\n",
				 opcode, battmgr->oneplus_ext_property, result);
			battmgr->error = -EREMOTEIO;
		} else {
			battmgr->error = 0;
		}
		break;
	case BATTMGR_REQUEST_NOTIFICATION:
	case BATTMGR_CHG_CTRL_LIMIT_EN:
		battmgr->error = 0;
		break;
	default:
		dev_warn(battmgr->dev, "unknown message %#x\n", opcode);
		break;
	}

out_complete:
	complete(&battmgr->ack);
}

static void qcom_battmgr_oneplus_pps_callback(struct qcom_battmgr *battmgr,
					       const void *data, size_t len)
{
	const struct qcom_battmgr_oneplus_pps_response *resp = data;
	u32 data_size;

	if (len != sizeof(*resp)) {
		dev_warn(battmgr->dev,
			 "invalid OnePlus PPS response length: %zu (expected %zu)\n",
			 len, sizeof(*resp));
		battmgr->oneplus_pps_error = -EMSGSIZE;
		goto out_complete;
	}

	data_size = le32_to_cpu(resp->data_size);
	if (data_size < 2 * sizeof(__le32) ||
	    data_size > sizeof(resp->data) ||
	    data_size % sizeof(__le32)) {
		dev_warn(battmgr->dev,
			 "invalid OnePlus PPS data length: %u\n", data_size);
		battmgr->oneplus_pps_error = -ENODATA;
		goto out_complete;
	}

	memcpy(&battmgr->oneplus_pps, resp, sizeof(*resp));
	battmgr->oneplus_pps_error = 0;

out_complete:
	complete(&battmgr->oneplus_pps_ack);
}

static void qcom_battmgr_callback(const void *data, size_t len, void *priv)
{
	const struct pmic_glink_hdr *hdr = data;
	struct qcom_battmgr *battmgr = priv;
	unsigned int opcode = le32_to_cpu(hdr->opcode);

	if (opcode == BATTMGR_ONEPLUS_PPS_READ_BUFFER &&
	    battmgr->oneplus_pps_read_buffer)
		qcom_battmgr_oneplus_pps_callback(battmgr, data, len);
	else if (opcode == BATTMGR_NOTIFICATION)
		qcom_battmgr_notification(battmgr, data, len);
	else if (battmgr->variant == QCOM_BATTMGR_OPLUS_SM8650 &&
		 opcode == BATTMGR_OPLUS_OEM_READ_BUFFER)
		complete(&battmgr->ack);	/* keepalive poll; payload unused */
	else if (battmgr->variant == QCOM_BATTMGR_SC8280XP ||
		 battmgr->variant == QCOM_BATTMGR_X1E80100)
		qcom_battmgr_sc8280xp_callback(battmgr, data, len);
	else
		qcom_battmgr_sm8350_callback(battmgr, data, len);
}

static void qcom_battmgr_enable_worker(struct work_struct *work)
{
	struct qcom_battmgr *battmgr = container_of(work, struct qcom_battmgr, enable_work);
	struct qcom_battmgr_enable_request req = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_NOTIFY),
		.hdr.opcode = cpu_to_le32(BATTMGR_REQUEST_NOTIFICATION),
	};
	int ret;

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_request(battmgr, &req, sizeof(req));
	mutex_unlock(&battmgr->lock);
	if (ret) {
		dev_err(battmgr->dev, "failed to request power notifications\n");
		return;
	}

	/* The supplies are registered before the remote service is up, so their
	 * initial synthetic uevents can fail with -EAGAIN. Refresh them once the
	 * service has accepted notifications to give userspace valid state.
	 */
	if (battmgr->bat_psy)
		power_supply_changed(battmgr->bat_psy);
	power_supply_changed(battmgr->usb_psy);
	if (battmgr->wls_psy)
		power_supply_changed(battmgr->wls_psy);
}

static void qcom_battmgr_oplus_otg_worker(struct work_struct *work)
{
	struct qcom_battmgr *battmgr = container_of(work, struct qcom_battmgr, otg_work);
	bool enable = battmgr->otg_vbus;
	int ret;

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_request_property(battmgr, BATTMGR_USB_PROPERTY_SET,
					    USB_OPLUS_OTG_VBUS_ENABLE, enable);
	mutex_unlock(&battmgr->lock);

	if (ret)
		dev_err(battmgr->dev, "failed to %s OTG VBUS: %d\n",
			enable ? "enable" : "disable", ret);
	else
		dev_info(battmgr->dev, "OTG VBUS %sd\n",
			 enable ? "enable" : "disable");
}

struct qcom_battmgr_oem_read_request {
	struct pmic_glink_hdr hdr;
	__le32 data_size;
};

/*
 * The stock Oplus charger daemon continuously polls the ADSP OEM status
 * buffer, and the firmware treats those reads as an "AP charger stack is
 * alive" signal: without them, requests such as OTG VBUS enable are
 * acknowledged but never acted upon. Keep the firmware fed with a periodic
 * poll; the returned data itself is not interesting (the fuel gauge is an
 * AP-side I2C chip on these devices).
 */
static void qcom_battmgr_oplus_keepalive_worker(struct work_struct *work)
{
	struct qcom_battmgr *battmgr = container_of(work, struct qcom_battmgr,
						    oplus_keepalive_work.work);
	struct qcom_battmgr_oem_read_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_OPLUS_OEM_READ_BUFFER),
		.data_size = cpu_to_le32(sizeof(u32) * BATTMGR_OPLUS_OEM_BUFFER_WORDS),
	};
	int ret;

	if (!battmgr->service_up)
		return;

	mutex_lock(&battmgr->lock);

	ret = qcom_battmgr_request(battmgr, &request, sizeof(request));
	if (ret)
		dev_dbg(battmgr->dev, "OEM keepalive poll failed: %d\n", ret);

	mutex_unlock(&battmgr->lock);

	schedule_delayed_work(&battmgr->oplus_keepalive_work,
			      msecs_to_jiffies(BATTMGR_OPLUS_KEEPALIVE_INTERVAL_MS));
}

/*
 * Read the battery voltage from the AP-side fuel gauge (BQ27Z561).
 * Returns 0 if the gauge isn't registered yet.
 */
static int qcom_battmgr_get_batt_voltage_uv(struct qcom_battmgr *battmgr)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = power_supply_get_by_name("bq27z561-0");
	if (!psy)
		return 0;

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	power_supply_put(psy);

	if (ret)
		return 0;

	return val.intval;
}

/* ---------- Oplus sysfs ---------- */

static ssize_t otg_vbus_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", battmgr->otg_vbus);
}

static ssize_t otg_vbus_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	battmgr->otg_vbus = enable;
	schedule_work(&battmgr->otg_work);

	return count;
}
static DEVICE_ATTR_RW(otg_vbus);

static struct attribute *qcom_battmgr_oplus_attrs[] = {
	&dev_attr_otg_vbus.attr,
	NULL
};

static const struct attribute_group qcom_battmgr_oplus_group = {
	.attrs = qcom_battmgr_oplus_attrs,
};

static void qcom_battmgr_pdr_notify(void *priv, int state)
{
	struct qcom_battmgr *battmgr = priv;

	if (state == SERVREG_SERVICE_STATE_UP) {
		WRITE_ONCE(battmgr->service_up, true);
		schedule_work(&battmgr->enable_work);
		if (battmgr->variant == QCOM_BATTMGR_OPLUS_SM8650) {
			battmgr->oplus_gauge_kicked = false;
			schedule_delayed_work(&battmgr->oplus_keepalive_work, 0);
		}
		if (READ_ONCE(battmgr->oneplus_dual_auto_enabled))
			mod_delayed_work(system_long_wq,
				&battmgr->oneplus_dual_auto_watch_work, 0);
		if (READ_ONCE(battmgr->oneplus_policy_auto_start))
			mod_delayed_work(system_long_wq,
				&battmgr->oneplus_policy_auto_work,
				msecs_to_jiffies(
					BATTMGR_ONEPLUS_POLICY_AUTO_CHECK_MS));
	} else {
		WRITE_ONCE(battmgr->service_up, false);
		if (battmgr->variant == QCOM_BATTMGR_OPLUS_SM8650)
			cancel_delayed_work(&battmgr->oplus_keepalive_work);
		if (READ_ONCE(battmgr->oneplus_dual_auto_enabled))
			mod_delayed_work(system_long_wq,
				&battmgr->oneplus_dual_auto_watch_work, 0);
		if (READ_ONCE(battmgr->oneplus_policy_running))
			mod_delayed_work(system_long_wq,
				&battmgr->oneplus_policy_monitor_work, 0);
	}
}

static const struct of_device_id qcom_battmgr_of_variants[] = {
	/* Oplus sm8650 phones ship Oplus-flavoured battmgr firmware */
	{ .compatible = "oplus,sm8650-pmic-glink", .data = (void *)QCOM_BATTMGR_OPLUS_SM8650 },
	{ .compatible = "qcom,glymur-pmic-glink", .data = (void *)QCOM_BATTMGR_X1E80100 },
	{ .compatible = "qcom,kaanapali-pmic-glink", .data = (void *)QCOM_BATTMGR_SM8550 },
	{ .compatible = "qcom,sc8180x-pmic-glink", .data = (void *)QCOM_BATTMGR_SC8280XP },
	{ .compatible = "qcom,sc8280xp-pmic-glink", .data = (void *)QCOM_BATTMGR_SC8280XP },
	{ .compatible = "qcom,sm8550-pmic-glink", .data = (void *)QCOM_BATTMGR_SM8550 },
	{ .compatible = "qcom,x1e80100-pmic-glink", .data = (void *)QCOM_BATTMGR_X1E80100 },
	/* Unmatched devices falls back to QCOM_BATTMGR_SM8350 */
	{}
};

static char *qcom_battmgr_battery[] = { "battery" };

static void qcom_battmgr_oplus_keepalive_cancel(void *data)
{
	struct qcom_battmgr *battmgr = data;

	cancel_delayed_work_sync(&battmgr->oplus_keepalive_work);
}

static int qcom_battmgr_probe(struct auxiliary_device *adev,
			      const struct auxiliary_device_id *id)
{
	const struct power_supply_desc *psy_desc;
	struct power_supply_config psy_cfg_supply = {};
	struct power_supply_config psy_cfg = {};
	const struct of_device_id *match;
	struct qcom_battmgr *battmgr;
	struct device *dev = &adev->dev;
	int ret;

	battmgr = devm_kzalloc(dev, sizeof(*battmgr), GFP_KERNEL);
	if (!battmgr)
		return -ENOMEM;

	battmgr->dev = dev;
	dev_set_drvdata(dev, battmgr);

	psy_cfg.drv_data = battmgr;
	psy_cfg.fwnode = dev_fwnode(&adev->dev);

	psy_cfg_supply.drv_data = battmgr;
	psy_cfg_supply.fwnode = dev_fwnode(&adev->dev);
	psy_cfg_supply.supplied_to = qcom_battmgr_battery;
	psy_cfg_supply.num_supplicants = 1;

	INIT_WORK(&battmgr->oneplus_dual_auto_run_work,
		  qcom_battmgr_oneplus_dual_auto_run_worker);
	INIT_DELAYED_WORK(&battmgr->oneplus_dual_auto_watch_work,
			  qcom_battmgr_oneplus_dual_auto_watch_worker);
	INIT_WORK(&battmgr->oneplus_policy_work,
		  qcom_battmgr_oneplus_policy_worker);
	INIT_DELAYED_WORK(&battmgr->oneplus_policy_monitor_work,
			  qcom_battmgr_oneplus_policy_monitor_worker);
	INIT_DELAYED_WORK(&battmgr->oneplus_policy_auto_work,
			  qcom_battmgr_oneplus_policy_auto_worker);
	mutex_init(&battmgr->lock);
	init_completion(&battmgr->ack);
	init_completion(&battmgr->oneplus_pps_ack);

	INIT_WORK(&battmgr->otg_work, qcom_battmgr_oplus_otg_worker);
	INIT_DELAYED_WORK(&battmgr->oplus_keepalive_work,
			  qcom_battmgr_oplus_keepalive_worker);

	ret = devm_add_action_or_reset(dev, qcom_battmgr_oplus_keepalive_cancel,
				       battmgr);
	if (ret)
		return ret;

	battmgr->oneplus_pps_read_buffer =
		device_property_read_bool(dev->parent, "oneplus,pps-read-buffer");
	battmgr->oneplus_pps_fixed_5v =
		device_property_read_bool(dev->parent, "oneplus,pps-fixed-5v");
	battmgr->oneplus_pps_5v5_1a =
		device_property_read_bool(dev->parent, "oneplus,pps-5v5-1a");
	battmgr->oneplus_pps_ramp_9v_1a =
		device_property_read_bool(dev->parent,
					  "oneplus,pps-ramp-9v-1a");
	battmgr->oneplus_pps_ramp_9v2_1a =
		device_property_read_bool(dev->parent,
					  "oneplus,pps-ramp-9v2-1a");
	battmgr->oneplus_pps_ramp_2to1_1a =
		device_property_read_bool(dev->parent,
					  "oneplus,pps-ramp-2to1-1a");
	battmgr->oneplus_pps_run_2to1_5m =
		device_property_read_bool(dev->parent,
					  "oneplus,pps-run-2to1-5m");
	battmgr->oneplus_pps_policy_2to1_1a =
		device_property_read_bool(dev->parent,
					  "oneplus,pps-policy-2to1-1a");
	battmgr->oneplus_pps_secondary_500ms =
		device_property_read_bool(dev->parent,
					  "oneplus,pps-secondary-500ms");
	battmgr->oneplus_pps_dual_500ms =
		device_property_read_bool(dev->parent,
					  "oneplus,pps-dual-500ms");
	battmgr->oneplus_policy_auto_start =
		device_property_read_bool(dev->parent,
					  "oneplus,pps-policy-auto-start");
	battmgr->oneplus_policy_auto_armed =
		battmgr->oneplus_policy_auto_start;
	/*
	 * The validated dual-pump policy is a normal plug-driven service.  Start
	 * armed so either a cable already present at probe or the next physical
	 * attach launches one continuous session without a userspace write.
	 */
	battmgr->oneplus_dual_auto_enabled =
		battmgr->oneplus_pps_dual_500ms;
	battmgr->oneplus_dual_auto_continuous =
		battmgr->oneplus_pps_dual_500ms;
	battmgr->oneplus_dual_auto_armed =
		battmgr->oneplus_pps_dual_500ms;
	battmgr->oneplus_dual_auto_connected = false;
	battmgr->oneplus_dual_auto_last_rc = -EINPROGRESS;
	if (battmgr->oneplus_pps_fixed_5v && !battmgr->oneplus_pps_read_buffer)
		return dev_err_probe(dev, -EINVAL,
				     "fixed-5V support requires PPS capability reads\n");
	if (battmgr->oneplus_pps_5v5_1a && !battmgr->oneplus_pps_fixed_5v)
		return dev_err_probe(dev, -EINVAL,
				     "PPS test support requires fixed-5V fallback\n");
	if (battmgr->oneplus_pps_ramp_9v_1a && !battmgr->oneplus_pps_5v5_1a)
		return dev_err_probe(dev, -EINVAL,
				     "PPS ramp test requires the 5.5-V test gate\n");
	if (battmgr->oneplus_pps_ramp_9v2_1a &&
	    !battmgr->oneplus_pps_ramp_9v_1a)
		return dev_err_probe(dev, -EINVAL,
				     "9.2-V PPS test requires the 9-V ramp gate\n");
	if (battmgr->oneplus_pps_ramp_2to1_1a &&
	    !battmgr->oneplus_pps_ramp_9v_1a)
		return dev_err_probe(dev, -EINVAL,
				     "2:1 PPS test requires the 9-V ramp gate\n");
	if (battmgr->oneplus_pps_run_2to1_5m &&
	    !battmgr->oneplus_pps_ramp_2to1_1a)
		return dev_err_probe(dev, -EINVAL,
				     "2:1 five-minute run requires the 2:1 PPS ramp gate\n");
	if (battmgr->oneplus_pps_policy_2to1_1a &&
	    !battmgr->oneplus_pps_run_2to1_5m)
		return dev_err_probe(dev, -EINVAL,
				     "2:1 policy requires the five-minute run gate\n");
	if (battmgr->oneplus_pps_secondary_500ms &&
	    !battmgr->oneplus_pps_ramp_2to1_1a)
		return dev_err_probe(dev, -EINVAL,
				     "secondary pulse requires the 2:1 PPS ramp gate\n");
	if (battmgr->oneplus_pps_dual_500ms &&
	    !battmgr->oneplus_pps_secondary_500ms)
		return dev_err_probe(dev, -EINVAL,
				     "dual pulse requires the secondary pulse gate\n");
	if (battmgr->oneplus_policy_auto_start &&
	    !battmgr->oneplus_pps_policy_2to1_1a)
		return dev_err_probe(dev, -EINVAL,
				     "policy auto-start requires the 2:1 policy gate\n");
	match = of_match_device(qcom_battmgr_of_variants, dev->parent);
	if (match)
		battmgr->variant = (unsigned long)match->data;
	else
		battmgr->variant = QCOM_BATTMGR_SM8350;

	if (battmgr->variant == QCOM_BATTMGR_OPLUS_SM8650) {
		ret = devm_device_add_group(dev, &qcom_battmgr_oplus_group);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to add oplus sysfs group\n");
	}

	ret = qcom_battmgr_charge_control_thresholds_init(battmgr);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "failed to init battery charge control thresholds\n");

	if (battmgr->variant == QCOM_BATTMGR_SC8280XP ||
	    battmgr->variant == QCOM_BATTMGR_X1E80100) {
		if (battmgr->variant == QCOM_BATTMGR_X1E80100)
			psy_desc = &x1e80100_bat_psy_desc;
		else
			psy_desc = &sc8280xp_bat_psy_desc;

		battmgr->bat_psy = devm_power_supply_register(dev, psy_desc, &psy_cfg);
		if (IS_ERR(battmgr->bat_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->bat_psy),
					     "failed to register battery power supply\n");

		battmgr->ac_psy = devm_power_supply_register(dev, &sc8280xp_ac_psy_desc, &psy_cfg_supply);
		if (IS_ERR(battmgr->ac_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->ac_psy),
					     "failed to register AC power supply\n");

		battmgr->usb_psy = devm_power_supply_register(dev, &sc8280xp_usb_psy_desc, &psy_cfg_supply);
		if (IS_ERR(battmgr->usb_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->usb_psy),
					     "failed to register USB power supply\n");

		battmgr->wls_psy = devm_power_supply_register(dev, &sc8280xp_wls_psy_desc, &psy_cfg_supply);
		if (IS_ERR(battmgr->wls_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->wls_psy),
					     "failed to register wireless charing power supply\n");
	} else {
		/*
		 * Oplus sm8650 devices use an AP-side I2C fuel gauge and the
		 * ADSP battery service only reports placeholder data, so do
		 * not register battery and wireless supplies for them.
		 */
		if (battmgr->variant != QCOM_BATTMGR_OPLUS_SM8650) {
			if (battmgr->variant == QCOM_BATTMGR_SM8550)
				psy_desc = &sm8550_bat_psy_desc;
			else
				psy_desc = &sm8350_bat_psy_desc;

			battmgr->bat_psy = devm_power_supply_register(dev, psy_desc, &psy_cfg);
			if (IS_ERR(battmgr->bat_psy))
				return dev_err_probe(dev, PTR_ERR(battmgr->bat_psy),
						     "failed to register battery power supply\n");
		}

		battmgr->usb_psy = devm_power_supply_register(dev, &sm8350_usb_psy_desc, &psy_cfg_supply);
		if (IS_ERR(battmgr->usb_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->usb_psy),
					     "failed to register USB power supply\n");

		if (battmgr->variant != QCOM_BATTMGR_OPLUS_SM8650) {
			battmgr->wls_psy = devm_power_supply_register(dev, &sm8350_wls_psy_desc, &psy_cfg_supply);
			if (IS_ERR(battmgr->wls_psy))
				return dev_err_probe(dev, PTR_ERR(battmgr->wls_psy),
						     "failed to register wireless charing power supply\n");
		}
	}

	ret = devm_work_autocancel(dev, &battmgr->enable_work,
				   qcom_battmgr_enable_worker);
	if (ret)
		return ret;

	battmgr->client = devm_pmic_glink_client_alloc(dev, PMIC_GLINK_OWNER_BATTMGR,
						       qcom_battmgr_callback,
						       qcom_battmgr_pdr_notify,
						       battmgr);
	if (IS_ERR(battmgr->client))
		return PTR_ERR(battmgr->client);

	pmic_glink_client_register(battmgr->client);

	if (battmgr->oneplus_pps_read_buffer) {
		ret = devm_device_add_group(dev, &qcom_battmgr_oneplus_group);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to add OnePlus PPS attributes\n");
	}

	if (battmgr->oneplus_pps_fixed_5v) {
		ret = devm_device_add_group(dev,
					     &qcom_battmgr_oneplus_fixed_5v_group);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to add OnePlus fixed-5V attribute\n");
	}

	if (battmgr->oneplus_pps_5v5_1a) {
		ret = devm_device_add_group(dev,
					     &qcom_battmgr_oneplus_pps_test_group);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to add OnePlus PPS test attribute\n");
	}

	if (battmgr->oneplus_pps_ramp_9v_1a) {
		ret = devm_device_add_group(
			dev, &qcom_battmgr_oneplus_pps_ramp_test_group);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to add OnePlus PPS ramp test attribute\n");
	}

	if (battmgr->oneplus_pps_ramp_9v2_1a) {
		ret = devm_device_add_group(
			dev, &qcom_battmgr_oneplus_pps_ramp_9v2_test_group);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to add OnePlus 9.2-V PPS test attribute\n");
	}

	if (battmgr->oneplus_pps_ramp_2to1_1a) {
		ret = devm_device_add_group(
			dev, &qcom_battmgr_oneplus_pps_ramp_2to1_test_group);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to add OnePlus 2:1 PPS test attribute\n");
	}

	if (battmgr->oneplus_pps_run_2to1_5m) {
		ret = devm_device_add_group(
			dev, &qcom_battmgr_oneplus_pps_run_2to1_test_group);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to add OnePlus 2:1 five-minute run attribute\n");
	}

	if (battmgr->oneplus_pps_secondary_500ms) {
		ret = devm_device_add_group(
			dev, &qcom_battmgr_oneplus_pps_secondary_test_group);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to add OnePlus secondary-pump test attribute\n");
	}

	if (battmgr->oneplus_pps_dual_500ms) {
		ret = devm_device_add_group(
			dev, &qcom_battmgr_oneplus_pps_dual_test_group);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to add OnePlus dual-pump test attribute\n");
	}

	if (battmgr->oneplus_pps_policy_2to1_1a) {
		ret = devm_device_add_group(
			dev, &qcom_battmgr_oneplus_pps_policy_group);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to add OnePlus 2:1 policy attribute\n");
	}
	if (battmgr->oneplus_policy_auto_start)
		queue_delayed_work(system_long_wq,
			&battmgr->oneplus_policy_auto_work,
			msecs_to_jiffies(BATTMGR_ONEPLUS_POLICY_AUTO_CHECK_MS));
	if (battmgr->oneplus_dual_auto_enabled)
		queue_delayed_work(system_long_wq,
			&battmgr->oneplus_dual_auto_watch_work, 0);

	return 0;
}

static void qcom_battmgr_remove(struct auxiliary_device *adev)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(&adev->dev);
	bool dual_auto_run;
	bool cancel_cp = false;

	if (!battmgr)
		return;

	WRITE_ONCE(battmgr->oneplus_dual_auto_enabled, false);
	cancel_delayed_work_sync(&battmgr->oneplus_dual_auto_watch_work);
	mutex_lock(&battmgr->lock);
	battmgr->oneplus_dual_auto_armed = false;
	battmgr->oneplus_dual_auto_cancel_requested = true;
	dual_auto_run = battmgr->oneplus_dual_auto_run;
	mutex_unlock(&battmgr->lock);
	if (dual_auto_run)
		qcom_battmgr_oneplus_dual_request_stop(battmgr);
	cancel_work_sync(&battmgr->oneplus_dual_auto_run_work);

	WRITE_ONCE(battmgr->oneplus_policy_auto_start, false);
	cancel_delayed_work_sync(&battmgr->oneplus_policy_auto_work);
	mutex_lock(&battmgr->lock);
	if (battmgr->oneplus_policy_running) {
		qcom_battmgr_oneplus_policy_stop_locked(
			battmgr, BATTMGR_ONEPLUS_POLICY_STOP_REMOVE,
			-ESHUTDOWN);
		cancel_cp = true;
	}
	mutex_unlock(&battmgr->lock);
	if (cancel_cp)
		qcom_battmgr_oneplus_policy_cancel_cp(battmgr);
	cancel_delayed_work_sync(&battmgr->oneplus_policy_monitor_work);
	cancel_work_sync(&battmgr->oneplus_policy_work);
	cancel_work_sync(&battmgr->enable_work);
}

static const struct auxiliary_device_id qcom_battmgr_id_table[] = {
	{ .name = "pmic_glink.power-supply", },
	{},
};
MODULE_DEVICE_TABLE(auxiliary, qcom_battmgr_id_table);

static struct auxiliary_driver qcom_battmgr_driver = {
	.name = "pmic_glink_power_supply",
	.probe = qcom_battmgr_probe,
	.remove = qcom_battmgr_remove,
	.id_table = qcom_battmgr_id_table,
};

module_auxiliary_driver(qcom_battmgr_driver);

MODULE_DESCRIPTION("Qualcomm PMIC GLINK battery manager driver");
MODULE_LICENSE("GPL");
