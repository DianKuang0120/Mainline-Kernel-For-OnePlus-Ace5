// SPDX-License-Identifier: GPL-2.0-only
/*
 * Southchip SC8547/SC8547A/SC8547D charge-pump bring-up driver.
 *
 * Standalone mainline-style port.  The original image targets OnePlus Pad Pro
 * (caihong, SC8547A).  The SC8547D branches added below target OnePlus
 * 13R / Ace 5 (giulia), whose charge-pump core and protection/ADC map are
 * shared but whose VOOC/UFCS timing (predata) register lives at 0x34 instead of
 * 0x3a and whose downstream SVOOC work setting differs slightly.
 *
 * Normal device nodes are telemetry-only apart from ADC enable.  Development
 * write controls are hidden behind explicit DT opt-ins.  Stage 4 provides
 * manual or policy-owned charge-pump control.  A narrow cancellation hook lets
 * an external policy stop an active run, but this physical driver still owns no
 * PD/PPS policy, dual-pump coordination or VOOC/UFCS handling.
 */

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>

#include "sc8547_api.h"

#define SC8547_REG_BAT_OVP		0x00
#define SC8547_BAT_OVP_DIS		BIT(7)
#define SC8547_BAT_OVP_MASK		GENMASK(5, 0)
#define SC8547_BAT_OVP_BASE_MV		3500
#define SC8547_BAT_OVP_STEP_MV		25

#define SC8547_REG_BAT_OCP		0x01
#define SC8547_BAT_OCP_MASK		GENMASK(5, 0)
#define SC8547_BAT_OCP_BASE_MA		2000
#define SC8547_BAT_OCP_STEP_MA		100

#define SC8547_REG_AC_OVP		0x02
#define SC8547_AC_OVP_STAT		BIT(5)
#define SC8547_AC_OVP_INT_MASK		BIT(3)
#define SC8547_AC_OVP_MASK		GENMASK(2, 0)
#define SC8547_AC_OVP_CONFIG_MASK	(SC8547_AC_OVP_INT_MASK | \
					 SC8547_AC_OVP_MASK)
#define SC8547_AC_OVP_BASE_MV		11000
#define SC8547_AC_OVP_STEP_MV		1000

#define SC8547_REG_VBUS_OVP		0x04
#define SC8547_VBUS_OVP_DIS		BIT(7)
#define SC8547_VBUS_OVP_MASK		GENMASK(6, 0)
#define SC8547_VBUS_OVP_BASE_MV	6000
#define SC8547_VBUS_OVP_STEP_MV	50

#define SC8547_REG_IBUS_PROT		0x05
#define SC8547_IBUS_UCP_DIS		BIT(7)
#define SC8547_IBUS_OCP_DIS		BIT(6)
#define SC8547_IBUS_OCP_MASK		GENMASK(3, 0)
#define SC8547_IBUS_OCP_BASE_MA	1200
#define SC8547_IBUS_OCP_STEP_MA	300
#define SC8547_UCP_DEGLITCH_SC8547	BIT(5)
#define SC8547A_UCP_DEGLITCH_MASK	GENMASK(5, 4)

#define SC8547_REG_STATUS_06		0x06
#define SC8547_TSHUT_STAT		BIT(6)
#define SC8547_VBUS_ERRORLO_STAT	BIT(5)
#define SC8547_VBUS_ERRORHI_STAT	BIT(4)
#define SC8547_SS_TIMEOUT_FLAG		BIT(3)
#define SC8547_CP_SWITCHING_STAT	BIT(2)
#define SC8547_BLOCKING_06		(SC8547_TSHUT_STAT | \
					 SC8547_VBUS_ERRORLO_STAT | \
					 SC8547_VBUS_ERRORHI_STAT | \
					 SC8547_SS_TIMEOUT_FLAG)
#define SC8547_FATAL_06			(SC8547_TSHUT_STAT | \
					 SC8547_VBUS_ERRORHI_STAT)

#define SC8547_REG_CHG_CTRL		0x07
#define SC8547_CHG_EN			BIT(7)
#define SC8547_REG_RESET		BIT(6)

#define SC8547_REG_SS_CTRL		0x08
#define SC8547_SS_TIMEOUT_MASK		GENMASK(7, 5)

#define SC8547_REG_MODE_CTRL		0x09
#define SC8547_CHARGE_MODE		BIT(7)
#define SC8547_IBUS_UCP_RISE_FLAG	BIT(5)
#define SC8547_IBUS_UCP_RISE_MASK	BIT(4)
#define SC8547_WD_TIMEOUT2_FLAG		BIT(3)
#define SC8547_WATCHDOG_MASK		GENMASK(2, 0)

#define SC8547_REG_IBUS_CTRL		0x0c

#define SC8547_REG_PMID2OUT		0x0d
#define SC8547_PMID2OUT_UVP_MASK	GENMASK(7, 6)
#define SC8547_PMID2OUT_OVP_MASK	GENMASK(5, 4)

#define SC8547_REG_STATUS_0E		0x0e
#define SC8547_VOUT_OVP_STAT		BIT(7)
#define SC8547_VBAT_OVP_STAT		BIT(6)
#define SC8547_IBAT_OCP_STAT		BIT(5)
#define SC8547_VBUS_OVP_STAT		BIT(4)
#define SC8547_IBUS_OCP_STAT		BIT(3)
#define SC8547_IBUS_UCP_FALL_STAT	BIT(2)
#define SC8547_ADAPTER_INSERT_STAT	BIT(1)
#define SC8547_VBAT_INSERT_STAT		BIT(0)
#define SC8547_BLOCKING_0E		(SC8547_VOUT_OVP_STAT | \
					 SC8547_VBAT_OVP_STAT | \
					 SC8547_IBAT_OCP_STAT | \
					 SC8547_VBUS_OVP_STAT | \
					 SC8547_IBUS_OCP_STAT | \
					 SC8547_IBUS_UCP_FALL_STAT)
#define SC8547_FATAL_0E			(SC8547_VOUT_OVP_STAT | \
					 SC8547_VBAT_OVP_STAT | \
					 SC8547_IBAT_OCP_STAT | \
					 SC8547_VBUS_OVP_STAT | \
					 SC8547_IBUS_OCP_STAT)

#define SC8547_REG_FAULT_0F		0x0f
#define SC8547_IBUS_UCP_FALL_FLAG	BIT(2)

#define SC8547_REG_INT_MASK		0x10

#define SC8547_REG_ADC_CTRL		0x11
#define SC8547_ADC_EN			BIT(7)
#define SC8547_ADC_RATE_ONESHOT		BIT(6)
#define SC8547_ADC_FREEZE		BIT(5)
#define SC8547_ADC_MODE_MASK		(SC8547_ADC_EN | \
					 SC8547_ADC_RATE_ONESHOT | \
					 SC8547_ADC_FREEZE)

#define SC8547_REG_ADC_FN_DISABLE	0x12
#define SC8547_VBUS_ADC_DIS		BIT(6)
#define SC8547_VOUT_ADC_DIS		BIT(4)
#define SC8547_VBAT_ADC_DIS		BIT(3)
#define SC8547_REQUIRED_ADC_DIS_MASK	(SC8547_VBUS_ADC_DIS | \
					 SC8547_VOUT_ADC_DIS | \
					 SC8547_VBAT_ADC_DIS)

#define SC8547_REG_LOOSE_DET		0x33
#define SC8547_REG_VOOCPHY_TIMING	0x3a
#define SC8547_REG_VOOCPHY_CTRL		0x2b
#define SC8547_REG_SLAVE_CTRL		0x30
#define SC8547A_REG_OTG_CTRL		0x3b
#define SC8547_REG_VBUS_RANGE_CTRL	0x3c

/*
 * OnePlus' wired PPS startup selects the 2:1 work mode, raises PPS VBUS, then
 * invokes CP_ENABLE followed by CP_SET_WORK_START. CP_ENABLE is intentionally
 * a no-op in oplus_chg_pps.c; the live path therefore asserts REG07 directly
 * and never runs the generic SC8547A reset/FORCE_VAC_OK helper. Preserve the
 * board profile's bounded OVP/OCP values while reproducing that live path.
 */
#define SC8547A_VENDOR_PPS_MODE_MASK	(SC8547_CHARGE_MODE | \
					 SC8547_IBUS_UCP_RISE_MASK)
#define SC8547A_VENDOR_PPS_MODE_2TO1	SC8547_IBUS_UCP_RISE_MASK
#define SC8547A_VENDOR_PPS_WATCHDOG	4
#define SC8547A_VENDOR_PPS_PMID2OUT	0x70
#define SC8547A_VENDOR_PPS_LOOSE_DET	0xd1
#define SC8547A_VENDOR_PPS_TIMING	0x60
#define SC8547A_VENDOR_VOOCPHY_DISABLE	0x00
#define SC8547A_VENDOR_INSERT_IRQ_MASK	0x02
#define SC8547A_VENDOR_OTG_CTRL		0x04
#define SC8547A_VENDOR_RESET_SS_TIMEOUT	0xe0
#define SC8547A_VENDOR_LOW_CURRENT_SS_TIMEOUT	0x00
#define SC8547_VENDOR_SECONDARY_CTRL	0x7f
#define SC8547_VENDOR_SECONDARY_VBUS_RANGE	0x40

/*
 * SC8547D (giulia / OnePlus 13R / Ace 5) image.  The charge-pump core and the
 * protection/ADC map are shared with SC8547A, but the VOOC/UFCS timing
 * (predata) register moved from 0x3a to 0x34, and the downstream SVOOC work
 * setting uses a 1-s watchdog and keeps the profile's IBUS UCP field instead of
 * forcing it off.  The A image is left byte-for-byte unchanged; the helpers
 * below select these values per detected variant.
 *
 * Downstream sources: oplus_hal_sc8547d.c sc8547_svooc_hw_setting() /
 * sc8547_init_device(), and oplus_sc8547.h.
 */
#define SC8547D_REG_VOOCPHY_TIMING	0x34
#define SC8547D_VENDOR_PPS_WATCHDOG	3	/* 1000 ms */
#define SC8547D_VENDOR_PPS_PMID2OUT	0x70
#define SC8547D_VENDOR_PPS_LOOSE_DET	0xd1
#define SC8547D_VENDOR_PPS_TIMING	0x60
#define SC8547D_VENDOR_VOOCPHY_DISABLE	0x00
#define SC8547D_VENDOR_INSERT_IRQ_MASK	0x02
#define SC8547D_VENDOR_OTG_CTRL		0x04
#define SC8547D_VENDOR_LOW_CURRENT_SS_TIMEOUT	0x00
#define SC8547D_VENDOR_SECONDARY_VBUS_RANGE	0x60

#define SC8547_EXPERIMENT_REVISION	"stage7d13-bounded-emergency-recovery"
#define SC8547_EXPERIMENT_SEQUENCE	"vendor-pps-2to1"

/* oplus_pps_charge_start(): one initial start plus three 500-ms retries. */
#define SC8547_VENDOR_START_RETRY_MAX	3
#define SC8547_VENDOR_START_ATTEMPTS	(SC8547_VENDOR_START_RETRY_MAX + 1)
#define SC8547_VENDOR_START_CHECK_MS	500

#define SC8547_PREP_MISMATCH_REG02_CONFIG	BIT(0)
#define SC8547_PREP_MISMATCH_REG02_STATUS	BIT(1)
#define SC8547_PREP_MISMATCH_REG04		BIT(2)
#define SC8547_PREP_MISMATCH_REG05		BIT(3)
#define SC8547_PREP_MISMATCH_REG08		BIT(4)
#define SC8547_PREP_MISMATCH_REG09		BIT(5)
#define SC8547_PREP_MISMATCH_REG0D		BIT(6)
#define SC8547_PREP_MISMATCH_REG10		BIT(7)
#define SC8547_PREP_MISMATCH_REG11		BIT(8)
#define SC8547_PREP_MISMATCH_REG2B		BIT(9)
#define SC8547_PREP_MISMATCH_REG3A		BIT(10)
#define SC8547_PREP_MISMATCH_REG3B		BIT(11)
#define SC8547_PREP_MISMATCH_REG30		BIT(12)
#define SC8547_PREP_MISMATCH_REG3C		BIT(13)

#define SC8547_DIAG_FIRST_REG		0x00
#define SC8547_DIAG_LAST_REG		0x11
#define SC8547_DIAG_REG_COUNT		(SC8547_DIAG_LAST_REG + 1)

#define SC8547_REG_IBUS_ADC_H		0x13
#define SC8547_REG_VBUS_ADC_H		0x15
#define SC8547_REG_VAC_ADC_H		0x17
#define SC8547_REG_VOUT_ADC_H		0x19
#define SC8547_REG_VBAT_ADC_H		0x1b
#define SC8547_REG_TDIE_ADC_H		0x1f
#define SC8547_REG_DEVICE_ID		0x36

#define SC8547_ADC_12BIT_H_MASK		GENMASK(3, 0)
#define SC8547_TDIE_H_MASK		BIT(0)

#define SC8547A_DEVICE_ID		0x67
#define SC8547D_DEVICE_ID		0x49

#define SC8547_IBUS_UA_PER_LSB		1875
#define SC8547_VBUS_UV_PER_LSB		3750
#define SC8547_VAC_UV_PER_LSB		5000
#define SC8547_VOUT_UV_PER_LSB		1250
#define SC8547_VBAT_UV_PER_LSB		1250
#define SC8547_TDIE_MC_PER_LSB		500

#define SC8547_POST_ENABLE_MS		500
#define SC8547_TEST_PULSE_MS		500
#define SC8547_SHORT_RUN_MS		2000
#define SC8547_DUAL_RUN_MS		4000
#define SC8547_DUAL_EXTENDED_PRIMARY_RUN_MS	6000
#define SC8547_DUAL_CLOSED_LOOP_SECONDARY_RUN_MS	5000
#define SC8547_DUAL_CLOSED_LOOP_PRIMARY_RUN_MS	7000
#define SC8547_DUAL_BOOST_SECONDARY_RUN_MS	8000
#define SC8547_DUAL_BOOST_PRIMARY_RUN_MS	10000
#define SC8547_DUAL_SUSTAIN_SECONDARY_RUN_MS	16000
#define SC8547_DUAL_SUSTAIN_PRIMARY_RUN_MS	18000
#define SC8547_DUAL_LONG_SUSTAIN_SECONDARY_RUN_MS	66000
#define SC8547_DUAL_LONG_SUSTAIN_PRIMARY_RUN_MS	68000
#define SC8547_DUAL_SOAK_SECONDARY_RUN_MS	325000
#define SC8547_DUAL_SOAK_PRIMARY_RUN_MS	330000
#define SC8547_DUAL_ENDURANCE_SECONDARY_RUN_MS	625000
#define SC8547_DUAL_ENDURANCE_PRIMARY_RUN_MS	630000
#define SC8547_STABILITY_RUN_MS		10000
#define SC8547_LONG_RUN_MS		60000
#define SC8547_SOAK_RUN_MS		300000
#define SC8547_POLICY_RUN_MS		900000
#define SC8547_POLICY_CANCEL_CMD		2
#define SC8547_POLICY_CONTINUOUS_CMD	3
#define SC8547_STABILITY_SAMPLE_MS	500
#define SC8547_RUNTIME_RANGE_RECHECKS	2
#define SC8547_RUNTIME_RANGE_RECHECK_MIN_US	10000
#define SC8547_RUNTIME_RANGE_RECHECK_MAX_US	20000
#define SC8547_POST_PREPARE_RANGE_RECHECKS	2
#define SC8547_RUNTIME_RECORDS		8
#define SC8547_SHORT_REPORT_INTERVAL	5
#define SC8547_LONG_REPORT_INTERVAL	20
#define SC8547_SOAK_REPORT_INTERVAL	100
#define SC8547_TEST_IBUS_MAX_UA		1200000
#define SC8547_VENDOR_CURRENT_IBUS_MAX_UA	1250000
#define SC8547_VENDOR_CURRENT_IBUS_ADC_MAX_UA	\
	(((SC8547_VENDOR_CURRENT_IBUS_MAX_UA + SC8547_IBUS_UA_PER_LSB - 1) / \
	  SC8547_IBUS_UA_PER_LSB) * SC8547_IBUS_UA_PER_LSB)
#define SC8547_TEST_EARLY_SAMPLE_MS	10
#define SC8547_RESET_SETTLE_MS		10
#define SC8547_ADC_READY_POLL_MS		10
#define SC8547_ADC_READY_TIMEOUT_MS	500
#define SC8547_ADC_READY_STABLE_SAMPLES	1
#define SC8547_TEST_RATIO_ERROR_MAX_UV	500000

enum sc8547_variant {
	SC8547_VARIANT_UNKNOWN,
	SC8547_VARIANT_SC8547,
	SC8547_VARIANT_SC8547A,
	SC8547_VARIANT_SC8547D,
};

struct sc8547_chip_info {
	enum sc8547_variant variant;
	const char *name;
};

static const struct sc8547_chip_info sc8547_info = {
	.variant = SC8547_VARIANT_SC8547,
	.name = "sc8547",
};

static const struct sc8547_chip_info sc8547a_info = {
	.variant = SC8547_VARIANT_SC8547A,
	.name = "sc8547a",
};

static const struct sc8547_chip_info sc8547d_info = {
	.variant = SC8547_VARIANT_SC8547D,
	.name = "sc8547d",
};

struct sc8547_raw_profile {
	u8 reg00;
	u8 reg01;
	u8 reg02;
	u8 reg04;
	u8 reg05;
	u8 reg0d;
	bool has_reg01;
	bool has_reg0d;
	bool complete;
};

struct sc8547_enable_window {
	u32 vbus_min_uv;
	u32 vbus_max_uv;
	u32 pulse_vbus_min_uv;
	u32 pulse_ratio_error_max_uv;
	u32 start_delta_uv;
	u32 start_overshoot_max_uv;
	u32 start_reference_max_uv;
	u32 vbat_min_uv;
	u32 vbat_max_uv;
	bool complete;
};

struct sc8547_profile_check {
	bool attempted;
	bool actual_valid;
	bool mismatch;
	int result;
	u8 reg;
	u8 mask;
	u8 expected;
	u8 actual;
};

struct sc8547_active_sample {
	bool valid;
	u16 ordinal;
	u8 reg06;
	u8 reg07;
	u8 reg0e;
	u8 reg33;
	u8 reg3a;
	u8 reg3b;
	u8 reg3c;
	int vac_uv;
	int vbus_uv;
	int vout_uv;
	int vbat_uv;
	int output_reference_uv;
	int headroom_uv;
	int ratio_error_uv;
	int ibus_ua;
	int tdie_mc;
};

struct sc8547_pulse_result {
	bool valid;
	bool preflight_snapshot_valid;
	bool control_snapshot_valid;
	bool prep_snapshot_valid;
	bool ready_snapshot_valid;
	bool first_adc_snapshot_valid;
	bool post_prepare_adc_snapshot_valid;
	bool ready_extended_snapshot_valid;
	bool enable_snapshot_valid;
	bool early_snapshot_valid;
	bool on_snapshot_valid;
	bool on_extended_snapshot_valid;
	bool final_snapshot_valid;
	bool reject_dump_valid;
	bool reject_extended_snapshot_valid;
	bool restore_attempted;
	bool rearm_attempted;
	bool rearm_snapshot_valid;
	bool policy_target_valid;
	bool runtime_observed;
	struct sc8547_profile_check restore_check;
	int result;
	int disable_result;
	int restore_result;
	int first_vbus_uv;
	int first_vbat_uv;
	int first_output_reference_uv;
	int pre_vbus_uv;
	int pre_vbat_uv;
	int pre_output_reference_uv;
	int target_vbus_uv;
	int adc_wait_ms;
	int post_prepare_vbus_uv;
	int post_prepare_vbat_uv;
	int post_prepare_output_reference_uv;
	int ready_vac_uv;
	int ready_vout_uv;
	int on_vac_uv;
	int on_vout_uv;
	int reject_vac_uv;
	int reject_vbus_uv;
	int reject_vout_uv;
	int reject_vbat_uv;
	int vbus_uv;
	int vbat_uv;
	int output_reference_uv;
	int headroom_uv;
	int active_target_vbus_uv;
	int ratio_error_uv;
	int ibus_ua;
	int tdie_mc;
	int runtime_vac_min_uv;
	int runtime_vac_max_uv;
	int runtime_vbus_min_uv;
	int runtime_vbus_max_uv;
	int runtime_ratio_min_uv;
	int runtime_ratio_max_uv;
	int runtime_ibus_min_ua;
	int runtime_ibus_max_ua;
	int runtime_tdie_min_mc;
	int runtime_tdie_max_mc;
	u32 requested_run_ms;
	bool vendor_current_control;
	u32 runtime_samples;
	u32 runtime_range_rechecks;
	u32 runtime_range_recoveries;
	u32 post_prepare_range_rechecks;
	u32 post_prepare_range_recoveries;
	u8 runtime_recorded;
	u8 start_attempts;
	u8 start_checks;
	u32 prep_mismatch;
	u8 pre_reg05;
	u8 pre_reg08;
	u8 pre_reg09;
	u8 pre_reg0c;
	u8 rearm_entry_reg09;
	u8 rearm_entry_reg11;
	u8 rearm_final_reg09;
	u8 rearm_final_reg11;
	u8 prep_reg02;
	u8 prep_reg04;
	u8 prep_reg05;
	u8 prep_reg08;
	u8 prep_reg09;
	u8 prep_reg0c;
	u8 prep_reg0d;
	u8 prep_reg10;
	u8 prep_reg11;
	u8 prep_reg2b;
	u8 prep_reg30;
	u8 prep_reg3a;
	u8 prep_reg3b;
	u8 prep_reg3c;
	u8 ready_reg06;
	u8 ready_reg07;
	u8 ready_reg0e;
	u8 ready_reg11;
	u8 ready_reg12;
	u8 ready_reg33;
	u8 ready_reg3a;
	u8 ready_reg3b;
	u8 ready_reg3c;
	u8 enable_reg07;
	u8 early_reg06;
	u8 early_reg07;
	u8 early_reg0e;
	u8 on_reg06;
	u8 on_reg07;
	u8 on_reg0e;
	u8 on_reg33;
	u8 on_reg3a;
	u8 on_reg3b;
	u8 on_reg3c;
	u8 final_reg06;
	u8 final_reg07;
	u8 reject_reg33;
	u8 reject_reg3a;
	u8 reject_reg3b;
	u8 reject_reg3c;
	u8 start_immediate_reg07[SC8547_VENDOR_START_ATTEMPTS];
	u8 start_check_reg06[SC8547_VENDOR_START_ATTEMPTS];
	u8 start_check_reg07[SC8547_VENDOR_START_ATTEMPTS];
	u8 start_check_reg0e[SC8547_VENDOR_START_ATTEMPTS];
	struct sc8547_active_sample runtime[SC8547_RUNTIME_RECORDS];
	u8 reject_regs[SC8547_DIAG_REG_COUNT];
};

struct sc8547_device {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct power_supply_desc psy_desc;
	const struct sc8547_chip_info *match_info;
	const char *role;
	enum sc8547_variant variant;
	struct mutex lock;
	struct sc8547_raw_profile profile;
	struct sc8547_enable_window window;
	struct sc8547_pulse_result pulse;
	struct sc8547_profile_check profile_check;
	atomic_t policy_cancel;
	atomic_t pulse_running;
	bool allow_experimental_control;
	bool allow_experimental_cp_enable;
	bool allow_experimental_cp_pulse;
	bool init_done;
	bool pps_prepared;
	u32 pps_target_vbus_uv;
	u32 pulse_tdie_max_mc;
	u8 device_id;
};

static const struct regmap_config sc8547_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
};

static const char *sc8547_variant_name(enum sc8547_variant variant)
{
	switch (variant) {
	case SC8547_VARIANT_SC8547:
		return "sc8547";
	case SC8547_VARIANT_SC8547A:
		return "sc8547a";
	case SC8547_VARIANT_SC8547D:
		return "sc8547d";
	default:
		return "unknown";
	}
}

static bool sc8547_variant_control_supported(enum sc8547_variant variant)
{
	return variant == SC8547_VARIANT_SC8547 ||
	       variant == SC8547_VARIANT_SC8547A ||
	       variant == SC8547_VARIANT_SC8547D;
}

static bool sc8547_bounded_role_supported(const struct sc8547_device *sc)
{
	if (!strcmp(sc->role, "primary"))
		return sc->variant == SC8547_VARIANT_SC8547A ||
		       sc->variant == SC8547_VARIANT_SC8547D;

	if (!strcmp(sc->role, "secondary"))
		return sc8547_variant_control_supported(sc->variant);

	return false;
}

static enum sc8547_variant
sc8547_detect_variant(struct sc8547_device *sc, u8 device_id)
{
	if (device_id == SC8547A_DEVICE_ID)
		return SC8547_VARIANT_SC8547A;
	if (device_id == SC8547D_DEVICE_ID)
		return SC8547_VARIANT_SC8547D;
	if (sc->match_info)
		return sc->match_info->variant;

	return SC8547_VARIANT_UNKNOWN;
}

static bool sc8547_variant_is_d(const struct sc8547_device *sc)
{
	return sc->variant == SC8547_VARIANT_SC8547D;
}

/*
 * The SC8547D moved the VOOC/UFCS timing (predata) register from 0x3a to 0x34.
 * Select the address per detected variant; the programmed value (0x60) is the
 * same on both parts.
 */
static unsigned int sc8547_vendor_timing_reg(const struct sc8547_device *sc)
{
	return sc8547_variant_is_d(sc) ? SC8547D_REG_VOOCPHY_TIMING :
					 SC8547_REG_VOOCPHY_TIMING;
}

static u8 sc8547_vendor_timing_value(const struct sc8547_device *sc)
{
	return sc8547_variant_is_d(sc) ? SC8547D_VENDOR_PPS_TIMING :
					 SC8547A_VENDOR_PPS_TIMING;
}

static unsigned int sc8547_vendor_watchdog(const struct sc8547_device *sc)
{
	return sc8547_variant_is_d(sc) ? SC8547D_VENDOR_PPS_WATCHDOG :
					 SC8547A_VENDOR_PPS_WATCHDOG;
}

/*
 * caihong's SC8547A bring-up forced IBUS UCP off because its limited 1-A test
 * source could sit below the UCP threshold and autonomously stop the pump.
 * giulia's SC8547D downstream SVOOC path keeps the profile's UCP field intact
 * (REG05 = 0x10 | IBUS_OCP), so only force the disable on the non-D parts.
 */
static bool sc8547_forces_ucp_disable(const struct sc8547_device *sc)
{
	return !sc8547_variant_is_d(sc);
}


static int sc8547_apply_experimental_init(struct sc8547_device *sc);
static int sc8547_prepare_vendor_pps_startup(
	struct sc8547_device *sc, struct sc8547_pulse_result *p, bool *touched);
static int sc8547_bounded_pps_pulse(struct sc8547_device *sc,
				    unsigned int run_ms,
				    bool vendor_current_control);
static int sc8547_continuous_pps_run(struct sc8547_device *sc);

/*
 * Match the downstream role/variant-specific whole-register writes.  In
 * particular, an SC8547A primary uses 0x80/0x00 rather than inheriting the
 * non-A 0x04 reset value.  Secondary devices carry the downstream bit-0 role
 * setting. Bounded control is separately gated per physical pump role.
 */
static u8 sc8547_charge_command(const struct sc8547_device *sc, bool enable)
{
	u8 value = enable ? SC8547_CHG_EN : 0;

	if (sc->variant == SC8547_VARIANT_SC8547)
		value |= BIT(2);
	if (!strcmp(sc->role, "secondary"))
		value |= BIT(0);

	return value;
}

static int sc8547_set_charge_enabled(struct sc8547_device *sc, bool enable)
{
	return regmap_write(sc->regmap, SC8547_REG_CHG_CTRL,
			    sc8547_charge_command(sc, enable));
}

static int sc8547_set_work_mode(struct sc8547_device *sc, bool bypass)
{
	return regmap_update_bits(sc->regmap, SC8547_REG_MODE_CTRL,
				 SC8547_CHARGE_MODE,
				 bypass ? SC8547_CHARGE_MODE : 0);
}

static int sc8547_set_adc_enabled(struct sc8547_device *sc, bool enable)
{
	return regmap_update_bits(sc->regmap, SC8547_REG_ADC_CTRL,
				 SC8547_ADC_EN, enable ? SC8547_ADC_EN : 0);
}

static int sc8547_enable_required_adc_channels(struct sc8547_device *sc)
{
	return regmap_update_bits(sc->regmap, SC8547_REG_ADC_FN_DISABLE,
				 SC8547_REQUIRED_ADC_DIS_MASK, 0);
}

static int sc8547_set_watchdog_code(struct sc8547_device *sc, unsigned int code)
{
	if (code > 5)
		return -EINVAL;

	return regmap_update_bits(sc->regmap, SC8547_REG_MODE_CTRL,
				 SC8547_WATCHDOG_MASK, code);
}

static int sc8547_watchdog_code_from_ms(unsigned int timeout_ms,
					unsigned int *code)
{
	switch (timeout_ms) {
	case 0:
		*code = 0;
		return 0;
	case 200:
		*code = 1;
		return 0;
	case 500:
		*code = 2;
		return 0;
	case 1000:
		*code = 3;
		return 0;
	case 5000:
		*code = 4;
		return 0;
	case 30000:
		*code = 5;
		return 0;
	default:
		return -EINVAL;
	}
}

static int sc8547_read_adc(struct sc8547_device *sc, unsigned int reg,
			   unsigned int high_mask, int scale)
{
	u8 raw[2];
	int ret;

	/*
	 * The ADC high and low bytes are one sample.  Reading them through two
	 * independent I2C transactions can splice adjacent conversions when the
	 * continuous ADC updates between reads.  Keep each pair in one transfer;
	 * runtime range confirmation below still rejects persistent bad data.
	 */
	ret = regmap_bulk_read(sc->regmap, reg, raw, sizeof(raw));
	if (ret)
		return ret;

	return (((raw[0] & high_mask) << 8) | raw[1]) * scale;
}

static int sc8547_get_vbus_uv(struct sc8547_device *sc)
{
	return sc8547_read_adc(sc, SC8547_REG_VBUS_ADC_H,
			       SC8547_ADC_12BIT_H_MASK, SC8547_VBUS_UV_PER_LSB);
}

static int sc8547_get_ibus_ua(struct sc8547_device *sc)
{
	return sc8547_read_adc(sc, SC8547_REG_IBUS_ADC_H,
			       SC8547_ADC_12BIT_H_MASK, SC8547_IBUS_UA_PER_LSB);
}

static int sc8547_get_vbat_uv(struct sc8547_device *sc)
{
	return sc8547_read_adc(sc, SC8547_REG_VBAT_ADC_H,
			       SC8547_ADC_12BIT_H_MASK, SC8547_VBAT_UV_PER_LSB);
}

static int sc8547_get_vout_uv(struct sc8547_device *sc)
{
	return sc8547_read_adc(sc, SC8547_REG_VOUT_ADC_H,
			       SC8547_ADC_12BIT_H_MASK, SC8547_VOUT_UV_PER_LSB);
}

/*
 * Caihong's secondary pump has a valid VOUT sense path but its local VBAT
 * ADC is not connected: hardware reads show VOUT tracking the system battery
 * while VBAT remains fixed at one 5-mV conversion.  The downstream legacy
 * slave PPS path likewise consumes REG19 (VOUT), not REG1B (VBAT).  Keep the
 * validated primary on VBAT and select VOUT only for the secondary role.
 */
static bool sc8547_uses_vout_reference(const struct sc8547_device *sc)
{
	/*
	 * Both caihong (SC8547A) and giulia (SC8547D) leave the secondary
	 * pump's local VBAT ADC unconnected (it reads ~7.5 mV, one LSB), while
	 * its VOUT sense path tracks the battery.  The secondary therefore uses
	 * VOUT as the output reference; the primary uses VBAT.
	 */
	return !strcmp(sc->role, "secondary");
}

static const char *
sc8547_output_reference_source(const struct sc8547_device *sc)
{
	return sc8547_uses_vout_reference(sc) ? "vout" : "vbat";
}

static int sc8547_get_output_reference_uv(struct sc8547_device *sc)
{
	if (sc8547_uses_vout_reference(sc))
		return sc8547_get_vout_uv(sc);

	return sc8547_get_vbat_uv(sc);
}

static int sc8547_select_output_reference_uv(const struct sc8547_device *sc,
					       int vout_uv, int vbat_uv)
{
	return sc8547_uses_vout_reference(sc) ? vout_uv : vbat_uv;
}

static int sc8547_get_vac_uv(struct sc8547_device *sc)
{
	return sc8547_read_adc(sc, SC8547_REG_VAC_ADC_H,
			       SC8547_ADC_12BIT_H_MASK, SC8547_VAC_UV_PER_LSB);
}

static int sc8547_capture_path_snapshot(struct sc8547_device *sc,
					 int *vac_uv, int *vout_uv,
					 u8 *reg33, u8 *reg3a,
					 u8 *reg3b, u8 *reg3c)
{
	unsigned int val;
	int ret;

	*vac_uv = sc8547_get_vac_uv(sc);
	if (*vac_uv < 0)
		return *vac_uv;
	*vout_uv = sc8547_get_vout_uv(sc);
	if (*vout_uv < 0)
		return *vout_uv;

	ret = regmap_read(sc->regmap, SC8547_REG_LOOSE_DET, &val);
	if (ret)
		return ret;
	*reg33 = val;
	ret = regmap_read(sc->regmap, sc8547_vendor_timing_reg(sc), &val);
	if (ret)
		return ret;
	*reg3a = val;
	ret = regmap_read(sc->regmap, SC8547A_REG_OTG_CTRL, &val);
	if (ret)
		return ret;
	*reg3b = val;
	ret = regmap_read(sc->regmap, SC8547_REG_VBUS_RANGE_CTRL, &val);
	if (ret)
		return ret;
	*reg3c = val;

	return 0;
}

static int sc8547_get_tdie_mc(struct sc8547_device *sc)
{
	return sc8547_read_adc(sc, SC8547_REG_TDIE_ADC_H,
			       SC8547_TDIE_H_MASK, SC8547_TDIE_MC_PER_LSB);
}

static int sc8547_read_status(struct sc8547_device *sc,
			       unsigned int *reg06, unsigned int *reg0e)
{
	int ret;

	ret = regmap_read(sc->regmap, SC8547_REG_STATUS_06, reg06);
	if (ret)
		return ret;

	return regmap_read(sc->regmap, SC8547_REG_STATUS_0E, reg0e);
}

static bool sc8547_has_blocking_fault(unsigned int reg06, unsigned int reg0e)
{
	return !!((reg06 & SC8547_BLOCKING_06) ||
		  (reg0e & SC8547_BLOCKING_0E));
}

static bool sc8547_has_fatal_fault(unsigned int reg06, unsigned int reg0e)
{
	return !!((reg06 & SC8547_FATAL_06) ||
		  (reg0e & SC8547_FATAL_0E));
}


static enum sc8547_api_variant
sc8547_to_api_variant(enum sc8547_variant variant)
{
	switch (variant) {
	case SC8547_VARIANT_SC8547:
		return SC8547_API_VARIANT_SC8547;
	case SC8547_VARIANT_SC8547A:
		return SC8547_API_VARIANT_SC8547A;
	case SC8547_VARIANT_SC8547D:
		return SC8547_API_VARIANT_SC8547D;
	default:
		return SC8547_API_VARIANT_UNKNOWN;
	}
}

static struct sc8547_device *sc8547_from_client(struct i2c_client *client)
{
	if (!client || !client->dev.driver ||
	    strcmp(client->dev.driver->name, "sc8547"))
		return NULL;

	return i2c_get_clientdata(client);
}

int sc8547_get_state(struct i2c_client *client, struct sc8547_state *state)
{
	struct sc8547_device *sc = sc8547_from_client(client);
	unsigned int reg06, reg07, reg09, reg0e;
	int ret;

	if (!sc || !state)
		return -ENODEV;

	mutex_lock(&sc->lock);
	ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg07);
	if (ret)
		goto out;
	ret = regmap_read(sc->regmap, SC8547_REG_MODE_CTRL, &reg09);
	if (ret)
		goto out;
	ret = sc8547_read_status(sc, &reg06, &reg0e);
	if (ret)
		goto out;

	state->variant = sc8547_to_api_variant(sc->variant);
	state->device_id = sc->device_id;
	state->initialized = sc->init_done;
	state->stage4_authorized = sc->allow_experimental_cp_enable &&
				   sc->window.complete &&
				   sc8547_variant_control_supported(sc->variant);
	state->enabled = !!(reg07 & SC8547_CHG_EN);
	state->switching = !!(reg06 & SC8547_CP_SWITCHING_STAT);
	state->bypass = !!(reg09 & SC8547_CHARGE_MODE);
	state->blocking_fault = sc8547_has_blocking_fault(reg06, reg0e);

	state->vbus_uv = sc8547_get_vbus_uv(sc);
	if (state->vbus_uv < 0) {
		ret = state->vbus_uv;
		goto out;
	}
	state->vbat_uv = sc8547_get_vbat_uv(sc);
	if (state->vbat_uv < 0) {
		ret = state->vbat_uv;
		goto out;
	}
	state->ibus_ua = sc8547_get_ibus_ua(sc);
	if (state->ibus_ua < 0) {
		ret = state->ibus_ua;
		goto out;
	}
	ret = 0;
out:
	mutex_unlock(&sc->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547_get_state);

static enum power_supply_property sc8547_psy_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_CALIBRATE,
};

static int sc8547_psy_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct sc8547_device *sc = power_supply_get_drvdata(psy);
	unsigned int reg;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = regmap_read(sc->regmap, SC8547_REG_STATUS_06, &reg);
		if (ret)
			return ret;
		val->intval = reg & SC8547_CP_SWITCHING_STAT ?
			POWER_SUPPLY_STATUS_CHARGING : POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	case POWER_SUPPLY_PROP_PRESENT:
		ret = regmap_read(sc->regmap, SC8547_REG_STATUS_0E, &reg);
		if (ret)
			return ret;
		val->intval = !!(reg & SC8547_ADAPTER_INSERT_STAT);
		return 0;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg);
		if (ret)
			return ret;
		val->intval = !!(reg & SC8547_CHG_EN);
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = sc8547_get_vbus_uv(sc);
		if (ret < 0)
			return ret;
		val->intval = ret;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		/* Board-local coordination input: expose the pump's VBAT ADC. */
		ret = sc8547_get_vbat_uv(sc);
		if (ret < 0)
			return ret;
		val->intval = ret;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		/* Board-local coordination input: expose the pump's VOUT ADC. */
		ret = sc8547_get_vout_uv(sc);
		if (ret < 0)
			return ret;
		val->intval = ret;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = sc8547_get_ibus_ua(sc);
		if (ret < 0)
			return ret;
		val->intval = ret;
		return 0;
	case POWER_SUPPLY_PROP_TEMP:
		ret = sc8547_get_tdie_mc(sc);
		if (ret < 0)
			return ret;
		val->intval = ret / 100;
		return 0;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		val->intval = sc->pps_target_vbus_uv;
		return 0;
	case POWER_SUPPLY_PROP_CALIBRATE:
		val->intval = sc->pps_prepared;
		return 0;
	default:
		return -EINVAL;
	}
}

/*
 * Development-only coordination hook used by qcom_battmgr's explicitly
 * opted-in PPS test.  Preparing before the source ramp reproduces the vendor
 * order; clearing the property restores the bounded, pump-off raw profile.
 */
static int sc8547_psy_set_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	struct sc8547_device *sc = power_supply_get_drvdata(psy);
	struct sc8547_pulse_result prep = { };
	bool touched = false;
	int ret;

	if (psp != POWER_SUPPLY_PROP_CALIBRATE &&
	    psp != POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT)
		return -EINVAL;

	/*
	 * Cancellation must not wait behind the mutex held by the bounded run.
	 * It changes no register here; the run observes it within one 500-ms
	 * sample and executes its normal disable/readback/restore path.
	 */
	if (psp == POWER_SUPPLY_PROP_CALIBRATE &&
	    val->intval == SC8547_POLICY_CANCEL_CMD) {
		atomic_set(&sc->policy_cancel, 1);
		return 0;
	}

	mutex_lock(&sc->lock);
	if (psp == POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT) {
		if (val->intval == 0) {
			sc->pps_target_vbus_uv = 0;
			ret = 0;
			goto out;
		}
		if (val->intval < 0 || !sc->allow_experimental_cp_pulse ||
		    !sc->window.complete ||
		    !sc8547_bounded_role_supported(sc)) {
			ret = -EPERM;
			goto out;
		}
		if ((u32)val->intval < sc->window.vbus_min_uv ||
		    (u32)val->intval > sc->window.vbus_max_uv -
					 sc->window.start_overshoot_max_uv) {
			ret = -ERANGE;
			goto out;
		}
		/*
		 * The policy-owned target is the first controlled CP operation.
		 * Make it self-contained after boot by applying the same validated,
		 * fail-closed profile previously reached through apply_init.
		 */
		if (!sc->init_done) {
			ret = sc8547_apply_experimental_init(sc);
			if (ret)
				goto out;
		}
		sc->pps_target_vbus_uv = val->intval;
		ret = 0;
		goto out;
	}
	if (val->intval == SC8547_TEST_PULSE_MS ||
	    val->intval == SC8547_SHORT_RUN_MS ||
	    val->intval == SC8547_DUAL_RUN_MS ||
	    val->intval == SC8547_DUAL_EXTENDED_PRIMARY_RUN_MS ||
	    val->intval == SC8547_DUAL_CLOSED_LOOP_SECONDARY_RUN_MS ||
	    val->intval == SC8547_DUAL_CLOSED_LOOP_PRIMARY_RUN_MS ||
	    val->intval == SC8547_DUAL_BOOST_SECONDARY_RUN_MS ||
	    val->intval == SC8547_DUAL_BOOST_PRIMARY_RUN_MS ||
	    val->intval == SC8547_DUAL_SUSTAIN_SECONDARY_RUN_MS ||
	    val->intval == SC8547_DUAL_SUSTAIN_PRIMARY_RUN_MS ||
	    val->intval == SC8547_DUAL_LONG_SUSTAIN_SECONDARY_RUN_MS ||
	    val->intval == SC8547_DUAL_LONG_SUSTAIN_PRIMARY_RUN_MS ||
	    val->intval == SC8547_DUAL_SOAK_SECONDARY_RUN_MS ||
	    val->intval == SC8547_DUAL_SOAK_PRIMARY_RUN_MS ||
	    val->intval == SC8547_DUAL_ENDURANCE_SECONDARY_RUN_MS ||
	    val->intval == SC8547_DUAL_ENDURANCE_PRIMARY_RUN_MS) {
		if (val->intval == SC8547_DUAL_EXTENDED_PRIMARY_RUN_MS &&
		    strcmp(sc->role, "primary")) {
			ret = -EPERM;
			goto out;
		}
		if (val->intval == SC8547_DUAL_CLOSED_LOOP_SECONDARY_RUN_MS &&
		    strcmp(sc->role, "secondary")) {
			ret = -EPERM;
			goto out;
		}
		if (val->intval == SC8547_DUAL_CLOSED_LOOP_PRIMARY_RUN_MS &&
		    strcmp(sc->role, "primary")) {
			ret = -EPERM;
			goto out;
		}
		if (val->intval == SC8547_DUAL_BOOST_SECONDARY_RUN_MS &&
		    strcmp(sc->role, "secondary")) {
			ret = -EPERM;
			goto out;
		}
		if (val->intval == SC8547_DUAL_BOOST_PRIMARY_RUN_MS &&
		    strcmp(sc->role, "primary")) {
			ret = -EPERM;
			goto out;
		}
		if (val->intval == SC8547_DUAL_SUSTAIN_SECONDARY_RUN_MS &&
		    strcmp(sc->role, "secondary")) {
			ret = -EPERM;
			goto out;
		}
		if (val->intval == SC8547_DUAL_SUSTAIN_PRIMARY_RUN_MS &&
		    strcmp(sc->role, "primary")) {
			ret = -EPERM;
			goto out;
		}
		if (val->intval == SC8547_DUAL_LONG_SUSTAIN_SECONDARY_RUN_MS &&
		    strcmp(sc->role, "secondary")) {
			ret = -EPERM;
			goto out;
		}
		if (val->intval == SC8547_DUAL_LONG_SUSTAIN_PRIMARY_RUN_MS &&
		    strcmp(sc->role, "primary")) {
			ret = -EPERM;
			goto out;
		}
		if (val->intval == SC8547_DUAL_SOAK_SECONDARY_RUN_MS &&
		    strcmp(sc->role, "secondary")) {
			ret = -EPERM;
			goto out;
		}
		if (val->intval == SC8547_DUAL_SOAK_PRIMARY_RUN_MS &&
		    strcmp(sc->role, "primary")) {
			ret = -EPERM;
			goto out;
		}
		if (val->intval == SC8547_DUAL_ENDURANCE_SECONDARY_RUN_MS &&
		    strcmp(sc->role, "secondary")) {
			ret = -EPERM;
			goto out;
		}
		if (val->intval == SC8547_DUAL_ENDURANCE_PRIMARY_RUN_MS &&
		    strcmp(sc->role, "primary")) {
			ret = -EPERM;
			goto out;
		}
		ret = sc8547_bounded_pps_pulse(
			sc, val->intval,
			val->intval == SC8547_DUAL_BOOST_SECONDARY_RUN_MS ||
			val->intval == SC8547_DUAL_BOOST_PRIMARY_RUN_MS ||
			val->intval == SC8547_DUAL_SUSTAIN_SECONDARY_RUN_MS ||
			val->intval == SC8547_DUAL_SUSTAIN_PRIMARY_RUN_MS ||
			val->intval ==
				SC8547_DUAL_LONG_SUSTAIN_SECONDARY_RUN_MS ||
			val->intval == SC8547_DUAL_LONG_SUSTAIN_PRIMARY_RUN_MS ||
			val->intval == SC8547_DUAL_SOAK_SECONDARY_RUN_MS ||
			val->intval == SC8547_DUAL_SOAK_PRIMARY_RUN_MS ||
			val->intval ==
				SC8547_DUAL_ENDURANCE_SECONDARY_RUN_MS ||
			val->intval == SC8547_DUAL_ENDURANCE_PRIMARY_RUN_MS);
		goto out;
	}
	if (val->intval == SC8547_POLICY_CONTINUOUS_CMD) {
		ret = sc8547_continuous_pps_run(sc);
		goto out;
	}
	if (val->intval == SC8547_SOAK_RUN_MS ||
	    val->intval == SC8547_POLICY_RUN_MS) {
		if (strcmp(sc->role, "primary"))
			ret = -EPERM;
		else
			ret = sc8547_bounded_pps_pulse(sc, val->intval, false);
		goto out;
	}
	if (val->intval != 0 && val->intval != 1) {
		ret = -EINVAL;
		goto out;
	}
	if (!val->intval) {
		ret = sc8547_apply_experimental_init(sc);
		goto out;
	}

	if (!sc->allow_experimental_cp_pulse || !sc->window.complete ||
	    !sc8547_bounded_role_supported(sc)) {
		ret = -EPERM;
		goto out;
	}
	if (!sc->init_done) {
		ret = sc8547_apply_experimental_init(sc);
		if (ret)
			goto out;
	}
	atomic_set(&sc->policy_cancel, 0);

	ret = sc8547_prepare_vendor_pps_startup(sc, &prep, &touched);
	if (!ret)
		sc->pps_prepared = true;
	else if (touched)
		sc8547_apply_experimental_init(sc);
out:
	mutex_unlock(&sc->lock);
	return ret;
}

static int sc8547_psy_property_is_writeable(struct power_supply *psy,
					     enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CALIBRATE ||
	       psp == POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT;
}

static ssize_t device_id_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);

	return sysfs_emit(buf, "0x%02x\n", sc->device_id);
}
static DEVICE_ATTR_RO(device_id);

static ssize_t variant_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", sc8547_variant_name(sc->variant));
}
static DEVICE_ATTR_RO(variant);

static ssize_t role_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", sc->role);
}
static DEVICE_ATTR_RO(role);

static ssize_t charge_enabled_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%u\n", !!(val & SC8547_CHG_EN));
}
static DEVICE_ATTR_RO(charge_enabled);

static ssize_t charge_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(sc->regmap, SC8547_REG_MODE_CTRL, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%s\n",
			  val & SC8547_CHARGE_MODE ? "bypass" : "2:1");
}
static DEVICE_ATTR_RO(charge_mode);

static ssize_t switching_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(sc->regmap, SC8547_REG_STATUS_06, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%u\n", !!(val & SC8547_CP_SWITCHING_STAT));
}
static DEVICE_ATTR_RO(switching);

static ssize_t adapter_present_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(sc->regmap, SC8547_REG_STATUS_0E, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%u\n", !!(val & SC8547_ADAPTER_INSERT_STAT));
}
static DEVICE_ATTR_RO(adapter_present);

static ssize_t battery_present_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(sc->regmap, SC8547_REG_STATUS_0E, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%u\n", !!(val & SC8547_VBAT_INSERT_STAT));
}
static DEVICE_ATTR_RO(battery_present);

#define SC8547_ADC_ATTR(_name, _fn) \
static ssize_t _name##_show(struct device *dev, \
			    struct device_attribute *attr, char *buf) \
{ \
	struct sc8547_device *sc = dev_get_drvdata(dev); \
	int ret = _fn(sc); \
	if (ret < 0) \
		return ret; \
	return sysfs_emit(buf, "%d\n", ret); \
} \
static DEVICE_ATTR_RO(_name)

SC8547_ADC_ATTR(vbus_uv, sc8547_get_vbus_uv);
SC8547_ADC_ATTR(ibus_ua, sc8547_get_ibus_ua);
SC8547_ADC_ATTR(vbat_uv, sc8547_get_vbat_uv);
SC8547_ADC_ATTR(vout_uv, sc8547_get_vout_uv);
SC8547_ADC_ATTR(vac_uv, sc8547_get_vac_uv);
SC8547_ADC_ATTR(tdie_mc, sc8547_get_tdie_mc);

static ssize_t status_regs_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	unsigned int reg06, reg0e;
	int ret;

	ret = sc8547_read_status(sc, &reg06, &reg0e);
	if (ret)
		return ret;

	return sysfs_emit(buf, "06=0x%02x 0e=0x%02x\n", reg06, reg0e);
}
static DEVICE_ATTR_RO(status_regs);

static ssize_t faults_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	unsigned int reg06, reg0e;
	size_t len = 0;
	int ret;

	ret = sc8547_read_status(sc, &reg06, &reg0e);
	if (ret)
		return ret;

	if (reg06 & SC8547_TSHUT_STAT)
		len += sysfs_emit_at(buf, len, "thermal_shutdown ");
	if (reg06 & SC8547_VBUS_ERRORLO_STAT)
		len += sysfs_emit_at(buf, len, "vbus_low ");
	if (reg06 & SC8547_VBUS_ERRORHI_STAT)
		len += sysfs_emit_at(buf, len, "vbus_high ");
	if (reg06 & SC8547_SS_TIMEOUT_FLAG)
		len += sysfs_emit_at(buf, len, "ss_timeout ");
	if (reg0e & SC8547_VOUT_OVP_STAT)
		len += sysfs_emit_at(buf, len, "vout_ovp ");
	if (reg0e & SC8547_VBAT_OVP_STAT)
		len += sysfs_emit_at(buf, len, "vbat_ovp ");
	if (reg0e & SC8547_IBAT_OCP_STAT)
		len += sysfs_emit_at(buf, len, "ibat_ocp ");
	if (reg0e & SC8547_VBUS_OVP_STAT)
		len += sysfs_emit_at(buf, len, "vbus_ovp ");
	if (reg0e & SC8547_IBUS_OCP_STAT)
		len += sysfs_emit_at(buf, len, "ibus_ocp ");
	if (reg0e & SC8547_IBUS_UCP_FALL_STAT)
		len += sysfs_emit_at(buf, len, "ibus_ucp_fall ");

	if (!len)
		return sysfs_emit(buf, "none\n");

	buf[len - 1] = '\n';
	return len;
}
static DEVICE_ATTR_RO(faults);

static int sc8547_dump_range(struct sc8547_device *sc, char *buf, size_t *len,
			     unsigned int first, unsigned int last)
{
	unsigned int reg, val;
	int ret;

	for (reg = first; reg <= last; reg++) {
		ret = regmap_read(sc->regmap, reg, &val);
		if (ret)
			return ret;
		*len += sysfs_emit_at(buf, *len, "%02x:%02x\n", reg, val);
	}

	return 0;
}

static ssize_t register_dump_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	size_t len = 0;
	unsigned int val;
	int ret;

	ret = sc8547_dump_range(sc, buf, &len, 0x00, 0x23);
	if (ret)
		return ret;
	ret = sc8547_dump_range(sc, buf, &len, 0x2b, 0x33);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, 0x36, &val);
	if (ret)
		return ret;
	len += sysfs_emit_at(buf, len, "36:%02x\n", val);
	ret = sc8547_dump_range(sc, buf, &len,
				sc8547_vendor_timing_reg(sc), 0x3c);
	if (ret)
		return ret;

	return len;
}
static DEVICE_ATTR_RO(register_dump);

static int sc8547_ucp_deglitch_us(struct sc8547_device *sc, unsigned int reg05)
{
	unsigned int code;

	if (sc->variant == SC8547_VARIANT_SC8547)
		return reg05 & SC8547_UCP_DEGLITCH_SC8547 ? 5000 : 10;

	if (sc->variant != SC8547_VARIANT_SC8547A &&
	    sc->variant != SC8547_VARIANT_SC8547D)
		return -EINVAL;

	code = FIELD_GET(SC8547A_UCP_DEGLITCH_MASK, reg05);
	switch (code) {
	case 0:
		return 10;
	case 1:
		return 5000;
	case 2:
		return 50000;
	case 3:
		return 100000;
	default:
		return -EINVAL;
	}
}

static int sc8547_watchdog_ms(unsigned int reg09)
{
	switch (FIELD_GET(SC8547_WATCHDOG_MASK, reg09)) {
	case 0:
		return 0;
	case 1:
		return 200;
	case 2:
		return 500;
	case 3:
		return 1000;
	case 4:
		return 5000;
	case 5:
		return 30000;
	default:
		return -EINVAL;
	}
}

static ssize_t protection_state_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	unsigned int r00, r01, r02, r04, r05, r08, r09, r0d;
	unsigned int bat_ovp_code, bat_ocp_code, ac_ovp_code;
	unsigned int vbus_ovp_code, ibus_ocp_code;
	int deglitch_us, watchdog_ms;
	size_t len = 0;
	int ret;

	ret = regmap_read(sc->regmap, SC8547_REG_BAT_OVP, &r00);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_BAT_OCP, &r01);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_AC_OVP, &r02);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_VBUS_OVP, &r04);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_IBUS_PROT, &r05);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_SS_CTRL, &r08);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_MODE_CTRL, &r09);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_PMID2OUT, &r0d);
	if (ret)
		return ret;

	bat_ovp_code = FIELD_GET(SC8547_BAT_OVP_MASK, r00);
	bat_ocp_code = FIELD_GET(SC8547_BAT_OCP_MASK, r01);
	ac_ovp_code = FIELD_GET(SC8547_AC_OVP_MASK, r02);
	vbus_ovp_code = FIELD_GET(SC8547_VBUS_OVP_MASK, r04);
	ibus_ocp_code = FIELD_GET(SC8547_IBUS_OCP_MASK, r05);
	deglitch_us = sc8547_ucp_deglitch_us(sc, r05);
	watchdog_ms = sc8547_watchdog_ms(r09);

	len += sysfs_emit_at(buf, len,
		"raw 00=%02x 01=%02x 02=%02x 04=%02x 05=%02x 08=%02x 09=%02x 0d=%02x\n",
		r00, r01, r02, r04, r05, r08, r09, r0d);
	len += sysfs_emit_at(buf, len,
		"bat_ovp enabled=%u code=%u header_mv=%u\n",
		!(r00 & SC8547_BAT_OVP_DIS), bat_ovp_code,
		SC8547_BAT_OVP_BASE_MV + bat_ovp_code * SC8547_BAT_OVP_STEP_MV);
	len += sysfs_emit_at(buf, len,
		"bat_ocp code=%u header_ma=%u\n", bat_ocp_code,
		SC8547_BAT_OCP_BASE_MA + bat_ocp_code * SC8547_BAT_OCP_STEP_MA);
	len += sysfs_emit_at(buf, len,
		"ac_ovp code=%u header_formula_mv=%u\n", ac_ovp_code,
		SC8547_AC_OVP_BASE_MV + ac_ovp_code * SC8547_AC_OVP_STEP_MV);
	len += sysfs_emit_at(buf, len,
		"vbus_ovp enabled=%u code=%u header_mv=%u\n",
		!(r04 & SC8547_VBUS_OVP_DIS), vbus_ovp_code,
		SC8547_VBUS_OVP_BASE_MV + vbus_ovp_code * SC8547_VBUS_OVP_STEP_MV);
	len += sysfs_emit_at(buf, len,
		"ibus_ucp enabled=%u ibus_ocp enabled=%u code=%u header_ma=%u\n",
		!(r05 & SC8547_IBUS_UCP_DIS), !(r05 & SC8547_IBUS_OCP_DIS),
		ibus_ocp_code,
		SC8547_IBUS_OCP_BASE_MA + ibus_ocp_code * SC8547_IBUS_OCP_STEP_MA);
	if (deglitch_us >= 0)
		len += sysfs_emit_at(buf, len, "ibus_ucp_deglitch_us=%d\n",
				     deglitch_us);
	else
		len += sysfs_emit_at(buf, len,
				     "ibus_ucp_deglitch_us=unknown_for_variant\n");
	len += sysfs_emit_at(buf, len,
		"ss_timeout_code=%u pmid2out_uvp_code=%u pmid2out_ovp_code=%u\n",
		(unsigned int)FIELD_GET(SC8547_SS_TIMEOUT_MASK, r08),
		(unsigned int)FIELD_GET(SC8547_PMID2OUT_UVP_MASK, r0d),
		(unsigned int)FIELD_GET(SC8547_PMID2OUT_OVP_MASK, r0d));
	if (watchdog_ms >= 0)
		len += sysfs_emit_at(buf, len, "watchdog_ms=%d\n", watchdog_ms);
	else
		len += sysfs_emit_at(buf, len, "watchdog_ms=reserved_code_%u\n",
				     (unsigned int)FIELD_GET(SC8547_WATCHDOG_MASK, r09));
	len += sysfs_emit_at(buf, len,
		"note=header_* values are vendor-header decodes, not yet hardware-validated thresholds\n");

	return len;
}
static DEVICE_ATTR_RO(protection_state);

static struct attribute *sc8547_attrs[] = {
	&dev_attr_device_id.attr,
	&dev_attr_variant.attr,
	&dev_attr_role.attr,
	&dev_attr_charge_enabled.attr,
	&dev_attr_charge_mode.attr,
	&dev_attr_switching.attr,
	&dev_attr_adapter_present.attr,
	&dev_attr_battery_present.attr,
	&dev_attr_vbus_uv.attr,
	&dev_attr_ibus_ua.attr,
	&dev_attr_vbat_uv.attr,
	&dev_attr_vout_uv.attr,
	&dev_attr_vac_uv.attr,
	&dev_attr_tdie_mc.attr,
	&dev_attr_status_regs.attr,
	&dev_attr_faults.attr,
	&dev_attr_register_dump.attr,
	&dev_attr_protection_state.attr,
	NULL,
};

static const struct attribute_group sc8547_attr_group = {
	.name = "sc8547",
	.attrs = sc8547_attrs,
};

static int sc8547_read_u8_property(struct device *dev, const char *name, u8 *val)
{
	u32 tmp;
	int ret;

	ret = device_property_read_u32(dev, name, &tmp);
	if (ret)
		return ret;
	if (tmp > 0xff)
		return -ERANGE;

	*val = tmp;
	return 0;
}

static int sc8547_read_optional_u8_property(struct device *dev,
					    const char *name, u8 *val,
					    bool *present)
{
	u32 tmp;
	int ret;

	if (!device_property_present(dev, name)) {
		*present = false;
		return 0;
	}

	ret = device_property_read_u32(dev, name, &tmp);
	if (ret)
		return ret;
	if (tmp > 0xff)
		return -ERANGE;

	*present = true;
	*val = tmp;
	return 0;
}

static void sc8547_parse_experimental_profile(struct sc8547_device *sc)
{
	struct device *dev = sc->dev;
	bool complete = true;

	sc->allow_experimental_control =
		device_property_read_bool(dev, "southchip,allow-experimental-control");
	if (!sc->allow_experimental_control)
		return;

	if (sc8547_read_u8_property(dev, "southchip,experimental-reg00",
				    &sc->profile.reg00))
		complete = false;
	if (sc8547_read_u8_property(dev, "southchip,experimental-reg02",
				    &sc->profile.reg02))
		complete = false;
	if (sc8547_read_u8_property(dev, "southchip,experimental-reg04",
				    &sc->profile.reg04))
		complete = false;
	if (sc8547_read_u8_property(dev, "southchip,experimental-reg05",
				    &sc->profile.reg05))
		complete = false;
	if (sc8547_read_optional_u8_property(dev, "southchip,experimental-reg01",
					     &sc->profile.reg01,
					     &sc->profile.has_reg01))
		complete = false;
	if (sc8547_read_optional_u8_property(dev, "southchip,experimental-reg0d",
					     &sc->profile.reg0d,
					     &sc->profile.has_reg0d))
		complete = false;

	sc->profile.complete = complete;
	if (!complete)
		dev_warn(dev,
			 "experimental controls enabled but raw protection profile is incomplete/invalid\n");
}

static void sc8547_parse_enable_window(struct sc8547_device *sc)
{
	struct device *dev = sc->dev;
	struct sc8547_enable_window *w = &sc->window;
	u64 max_target_uv;
	u64 min_target_uv;
	bool enable_requested;
	bool pulse_requested;
	int ret;

	enable_requested = device_property_read_bool(
		dev, "southchip,allow-experimental-cp-enable");
	pulse_requested = device_property_read_bool(
		dev, "southchip,allow-experimental-cp-pulse");
	if (!enable_requested && !pulse_requested)
		return;

	if (!sc->allow_experimental_control) {
		dev_warn(dev,
			 "CP control opt-in ignored without southchip,allow-experimental-control\n");
		return;
	}

	ret = device_property_read_u32(dev, "southchip,experimental-vbus-min-uv",
				       &w->vbus_min_uv);
	ret |= device_property_read_u32(dev, "southchip,experimental-vbus-max-uv",
					&w->vbus_max_uv);
	ret |= device_property_read_u32(dev, "southchip,experimental-vbat-min-uv",
					&w->vbat_min_uv);
	ret |= device_property_read_u32(dev, "southchip,experimental-vbat-max-uv",
					&w->vbat_max_uv);
	if (ret || !w->vbus_min_uv || !w->vbus_max_uv ||
	    !w->vbat_min_uv || !w->vbat_max_uv ||
	    w->vbus_min_uv >= w->vbus_max_uv ||
	    w->vbat_min_uv >= w->vbat_max_uv) {
		dev_warn(dev,
			 "experimental CP-enable requested with incomplete/invalid voltage window\n");
		return;
	}

	w->complete = true;
	sc->allow_experimental_cp_enable = enable_requested;
	if (!pulse_requested)
		return;

	if (!sc8547_bounded_role_supported(sc)) {
		dev_warn(dev,
			 "bounded CP pulse ignored for unsupported %s/%s pairing\n",
			 sc8547_variant_name(sc->variant), sc->role);
		return;
	}

	ret = device_property_read_u32(dev,
				       "southchip,experimental-tdie-max-mc",
				       &sc->pulse_tdie_max_mc);
	if (ret || !sc->pulse_tdie_max_mc) {
		dev_warn(dev,
			 "bounded CP pulse ignored without a die-temperature limit\n");
		return;
	}

	ret = device_property_read_u32(
		dev, "southchip,experimental-pulse-vbus-min-uv",
		&w->pulse_vbus_min_uv);
	if (ret || !w->pulse_vbus_min_uv ||
	    w->pulse_vbus_min_uv > w->vbus_min_uv ||
	    w->pulse_vbus_min_uv >= w->vbus_max_uv) {
		dev_warn(dev,
			 "bounded CP pulse ignored without a valid active VBUS lower limit\n");
		return;
	}

	ret = device_property_read_u32(
		dev, "southchip,experimental-start-delta-uv",
		&w->start_delta_uv);
	ret |= device_property_read_u32(
		dev, "southchip,experimental-start-overshoot-max-uv",
		&w->start_overshoot_max_uv);
	if (device_property_read_u32(
		dev, "southchip,experimental-start-reference-max-uv",
		&w->start_reference_max_uv))
		w->start_reference_max_uv = w->vbat_max_uv;
	ret |= device_property_read_u32(
		dev, "southchip,experimental-pulse-ratio-error-max-uv",
		&w->pulse_ratio_error_max_uv);
	if (ret || !w->start_delta_uv || !w->start_overshoot_max_uv ||
	    w->start_reference_max_uv < w->vbat_min_uv ||
	    w->start_reference_max_uv > w->vbat_max_uv ||
	    !w->pulse_ratio_error_max_uv ||
	    w->pulse_ratio_error_max_uv > SC8547_TEST_RATIO_ERROR_MAX_UV) {
		dev_warn(dev,
			 "bounded CP pulse ignored without valid startup/2:1 ratio limits\n");
		return;
	}

	/* Match the vendor 2:1 startup target: floor(2 * VBAT, 100 mV) + delta. */
	min_target_uv = ((u64)w->vbat_min_uv * 2 / 100000) * 100000;
	min_target_uv += w->start_delta_uv;
	max_target_uv = ((u64)w->start_reference_max_uv * 2 / 100000) *
		100000;
	max_target_uv += w->start_delta_uv;
	if (min_target_uv < w->vbus_min_uv ||
	    max_target_uv + w->start_overshoot_max_uv > w->vbus_max_uv) {
		dev_warn(dev,
			 "bounded CP pulse startup target does not fit the VBUS window\n");
		return;
	}

	sc->allow_experimental_cp_pulse = true;
}

static int sc8547_fail_closed(struct sc8547_device *sc)
{
	int first = 0;
	int ret;

	ret = sc8547_set_charge_enabled(sc, false);
	if (ret)
		first = ret;
	ret = sc8547_set_watchdog_code(sc, 0);
	if (ret && !first)
		first = ret;

	return first;
}

static int sc8547_verify_reg(struct sc8547_device *sc, unsigned int reg,
			     u8 expected, u8 mask)
{
	struct sc8547_profile_check *check = &sc->profile_check;
	unsigned int val;
	int ret;

	check->attempted = true;
	check->actual_valid = false;
	check->reg = reg;
	check->mask = mask;
	check->expected = expected;
	check->actual = 0;
	ret = regmap_read(sc->regmap, reg, &val);
	if (ret) {
		check->result = ret;
		return ret;
	}

	check->actual = val;
	check->actual_valid = true;
	if ((val & mask) != (expected & mask)) {
		check->mismatch = true;
		check->result = -EIO;
		return -EIO;
	}

	check->result = 0;
	return 0;
}

static int sc8547_profile_readback(struct sc8547_device *sc)
{
	const struct sc8547_raw_profile *p = &sc->profile;
	int ret;

	memset(&sc->profile_check, 0, sizeof(sc->profile_check));
	ret = sc8547_verify_reg(sc, SC8547_REG_BAT_OVP, p->reg00,
				SC8547_BAT_OVP_DIS | SC8547_BAT_OVP_MASK);
	if (ret)
		return ret;
	ret = sc8547_verify_reg(sc, SC8547_REG_AC_OVP, p->reg02,
				SC8547_AC_OVP_CONFIG_MASK);
	if (ret)
		return ret;
	/* A latched flag is diagnostic; an active AC overvoltage is blocking. */
	if (sc->profile_check.actual & SC8547_AC_OVP_STAT) {
		sc->profile_check.mismatch = true;
		sc->profile_check.result = -EIO;
		return -EIO;
	}
	ret = sc8547_verify_reg(sc, SC8547_REG_VBUS_OVP, p->reg04, 0xff);
	if (ret)
		return ret;
	ret = sc8547_verify_reg(sc, SC8547_REG_IBUS_PROT, p->reg05, 0xff);
	if (ret)
		return ret;
	if (p->has_reg01) {
		ret = sc8547_verify_reg(sc, SC8547_REG_BAT_OCP, p->reg01,
					0xff);
		if (ret)
			return ret;
	}
	if (p->has_reg0d) {
		ret = sc8547_verify_reg(sc, SC8547_REG_PMID2OUT, p->reg0d,
					SC8547_PMID2OUT_UVP_MASK |
					SC8547_PMID2OUT_OVP_MASK);
		if (ret)
			return ret;
	}
	if (!strcmp(sc->role, "secondary")) {
		ret = sc8547_verify_reg(sc, SC8547_REG_VOOCPHY_CTRL,
					SC8547A_VENDOR_VOOCPHY_DISABLE, 0xff);
		if (ret)
			return ret;
		if (!sc8547_variant_is_d(sc)) {
			ret = sc8547_verify_reg(sc, SC8547_REG_SLAVE_CTRL,
						SC8547_VENDOR_SECONDARY_CTRL,
						0xff);
			if (ret)
				return ret;
		}
		ret = sc8547_verify_reg(sc, SC8547_REG_VBUS_RANGE_CTRL,
					sc8547_variant_is_d(sc) ?
					SC8547D_VENDOR_SECONDARY_VBUS_RANGE :
					SC8547_VENDOR_SECONDARY_VBUS_RANGE,
					0xff);
		if (ret)
			return ret;
	}

	return 0;
}

static int sc8547_apply_experimental_init(struct sc8547_device *sc)
{
	const struct sc8547_raw_profile *p = &sc->profile;
	int ret;

	if (!sc8547_variant_control_supported(sc->variant))
		return -EOPNOTSUPP;
	if (!p->complete)
		return -EINVAL;

	memset(&sc->profile_check, 0, sizeof(sc->profile_check));
	sc->init_done = false;
	sc->pps_prepared = false;
	sc->pps_target_vbus_uv = 0;
	ret = sc8547_fail_closed(sc);
	if (ret)
		return ret;

	ret = regmap_update_bits(sc->regmap, SC8547_REG_CHG_CTRL,
				 SC8547_REG_RESET, SC8547_REG_RESET);
	if (ret)
		goto fail;
	/* The vendor UFCS path allows 10 ms for reset to settle before init. */
	msleep(SC8547_RESET_SETTLE_MS);

	ret = sc8547_fail_closed(sc);
	if (ret)
		goto fail;

	ret = regmap_write(sc->regmap, SC8547_REG_BAT_OVP, p->reg00);
	if (ret)
		goto fail;
	if (p->has_reg01) {
		ret = regmap_write(sc->regmap, SC8547_REG_BAT_OCP, p->reg01);
		if (ret)
			goto fail;
	}
	ret = regmap_write(sc->regmap, SC8547_REG_AC_OVP, p->reg02);
	if (ret)
		goto fail;
	ret = regmap_write(sc->regmap, SC8547_REG_VBUS_OVP, p->reg04);
	if (ret)
		goto fail;
	ret = regmap_write(sc->regmap, SC8547_REG_IBUS_PROT, p->reg05);
	if (ret)
		goto fail;
	if (p->has_reg0d) {
		ret = regmap_write(sc->regmap, SC8547_REG_PMID2OUT, p->reg0d);
		if (ret)
			goto fail;
	}
	if (!strcmp(sc->role, "secondary")) {
		/* Complete the downstream slave idle profile after reset. */
		ret = regmap_write(sc->regmap, SC8547_REG_VOOCPHY_CTRL,
				   SC8547A_VENDOR_VOOCPHY_DISABLE);
		if (ret)
			goto fail;
		ret = regmap_write(sc->regmap, SC8547_REG_SLAVE_CTRL,
				   SC8547_VENDOR_SECONDARY_CTRL);
		if (ret)
			goto fail;
		ret = regmap_write(sc->regmap, SC8547_REG_VBUS_RANGE_CTRL,
				   sc8547_variant_is_d(sc) ?
				   SC8547D_VENDOR_SECONDARY_VBUS_RANGE :
				   SC8547_VENDOR_SECONDARY_VBUS_RANGE);
		if (ret)
			goto fail;
	}

	ret = sc8547_fail_closed(sc);
	if (ret)
		goto fail;
	ret = sc8547_set_adc_enabled(sc, true);
	if (ret)
		goto fail;
	ret = sc8547_enable_required_adc_channels(sc);
	if (ret)
		goto fail;
	ret = sc8547_profile_readback(sc);
	if (ret)
		goto fail;

	sc->init_done = true;
	return 0;

fail:
	sc8547_fail_closed(sc);
	sc->init_done = false;
	sc->pps_prepared = false;
	return ret;
}

static ssize_t profile_raw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	const struct sc8547_raw_profile *p = &sc->profile;
	size_t len = 0;

	if (!p->complete)
		return sysfs_emit(buf, "incomplete\n");

	len += sysfs_emit_at(buf, len,
			     "required 00=%02x 02=%02x 04=%02x 05=%02x\n",
			     p->reg00, p->reg02, p->reg04, p->reg05);
	if (p->has_reg01)
		len += sysfs_emit_at(buf, len, "optional 01=%02x ", p->reg01);
	else
		len += sysfs_emit_at(buf, len, "optional 01=absent ");
	if (p->has_reg0d)
		len += sysfs_emit_at(buf, len, "0d=%02x\n", p->reg0d);
	else
		len += sysfs_emit_at(buf, len, "0d=absent\n");

	return len;
}
static DEVICE_ATTR_RO(profile_raw);

static ssize_t init_state_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n",
			  sc->init_done ? "initialized" : "not_initialized");
}
static DEVICE_ATTR_RO(init_state);

static ssize_t apply_init_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	mutex_lock(&sc->lock);
	ret = sc8547_apply_experimental_init(sc);
	mutex_unlock(&sc->lock);
	if (ret)
		return ret;

	return count;
}
static DEVICE_ATTR_WO(apply_init);

static ssize_t watchdog_ms_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	unsigned int reg;
	int ret;

	ret = regmap_read(sc->regmap, SC8547_REG_MODE_CTRL, &reg);
	if (ret)
		return ret;

	ret = sc8547_watchdog_ms(reg);
	if (ret < 0)
		return sysfs_emit(buf, "reserved_code_%u\n",
				  (unsigned int)FIELD_GET(SC8547_WATCHDOG_MASK, reg));

	return sysfs_emit(buf, "%d\n", ret);
}

static ssize_t watchdog_ms_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	unsigned int timeout_ms, code;
	int ret;

	ret = kstrtouint(buf, 0, &timeout_ms);
	if (ret)
		return ret;
	ret = sc8547_watchdog_code_from_ms(timeout_ms, &code);
	if (ret)
		return ret;

	mutex_lock(&sc->lock);
	if (!sc->init_done) {
		ret = -EPERM;
		goto out;
	}
	ret = sc8547_set_watchdog_code(sc, code);
out:
	mutex_unlock(&sc->lock);
	if (ret)
		return ret;

	return count;
}
static DEVICE_ATTR_RW(watchdog_ms);

static ssize_t enable_window_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	const struct sc8547_enable_window *w = &sc->window;

	if (!w->complete)
		return sysfs_emit(buf, "incomplete\n");

	if (sc->allow_experimental_cp_pulse)
		return sysfs_emit(buf,
			"preflight_vbus_uv=%u..%u start_delta_uv=%u start_overshoot_max_uv=%u start_reference_max_uv=%u pulse_vbus_min_uv=%u pulse_ratio_error_max_uv=%u output_uv=%u..%u\n",
			w->vbus_min_uv, w->vbus_max_uv,
			w->start_delta_uv, w->start_overshoot_max_uv,
			w->start_reference_max_uv,
			w->pulse_vbus_min_uv, w->pulse_ratio_error_max_uv,
			w->vbat_min_uv, w->vbat_max_uv);

	return sysfs_emit(buf, "vbus_uv=%u..%u vbat_uv=%u..%u\n",
			  w->vbus_min_uv, w->vbus_max_uv,
			  w->vbat_min_uv, w->vbat_max_uv);
}
static DEVICE_ATTR_RO(enable_window);

static ssize_t work_mode_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	unsigned int reg;
	int ret;

	ret = regmap_read(sc->regmap, SC8547_REG_MODE_CTRL, &reg);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%s\n",
			  reg & SC8547_CHARGE_MODE ? "bypass" : "2:1");
}

int sc8547_set_manual_mode(struct i2c_client *client, bool bypass)
{
	struct sc8547_device *sc = sc8547_from_client(client);
	unsigned int reg;
	int ret;

	if (!sc)
		return -ENODEV;

	mutex_lock(&sc->lock);
	if (!sc->allow_experimental_cp_enable || !sc->init_done ||
	    !sc->window.complete ||
	    !sc8547_variant_control_supported(sc->variant)) {
		ret = -EPERM;
		goto out;
	}

	ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg);
	if (ret)
		goto out;
	if (reg & SC8547_CHG_EN) {
		ret = -EBUSY;
		goto out;
	}

	ret = sc8547_set_work_mode(sc, bypass);
	if (ret)
		goto out;
	ret = regmap_read(sc->regmap, SC8547_REG_MODE_CTRL, &reg);
	if (ret)
		goto out;
	if (!!(reg & SC8547_CHARGE_MODE) != bypass)
		ret = -EIO;
out:
	mutex_unlock(&sc->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547_set_manual_mode);

static ssize_t work_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	bool bypass;
	int ret;

	if (sysfs_streq(buf, "2:1"))
		bypass = false;
	else if (sysfs_streq(buf, "bypass"))
		bypass = true;
	else
		return -EINVAL;

	ret = sc8547_set_manual_mode(to_i2c_client(dev), bypass);
	if (ret)
		return ret;

	return count;
}
static DEVICE_ATTR_RW(work_mode);

static int sc8547_window_check(struct sc8547_device *sc)
{
	const struct sc8547_enable_window *w = &sc->window;
	int output_reference_uv, vbus_uv;

	vbus_uv = sc8547_get_vbus_uv(sc);
	if (vbus_uv < 0)
		return vbus_uv;
	output_reference_uv = sc8547_get_output_reference_uv(sc);
	if (output_reference_uv < 0)
		return output_reference_uv;

	if ((u32)vbus_uv < w->vbus_min_uv || (u32)vbus_uv > w->vbus_max_uv ||
	    (u32)output_reference_uv < w->vbat_min_uv ||
	    (u32)output_reference_uv > w->vbat_max_uv)
		return -ERANGE;

	return 0;
}

static int sc8547_enable_preflight(struct sc8547_device *sc)
{
	unsigned int reg06, reg07, reg0e;
	int ret;

	if (!sc->init_done || !sc->window.complete)
		return -EPERM;
	if (!sc8547_variant_control_supported(sc->variant))
		return -EOPNOTSUPP;

	ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg07);
	if (ret)
		return ret;
	if (reg07 & SC8547_CHG_EN)
		return -EBUSY;

	ret = sc8547_read_status(sc, &reg06, &reg0e);
	if (ret)
		return ret;
	if (!(reg0e & SC8547_ADAPTER_INSERT_STAT) ||
	    !(reg0e & SC8547_VBAT_INSERT_STAT))
		return -ENODEV;
	if (reg06 & SC8547_CP_SWITCHING_STAT)
		return -EBUSY;
	if (sc8547_has_blocking_fault(reg06, reg0e))
		return -EIO;

	return sc8547_window_check(sc);
}

static int sc8547_pulse_preflight(struct sc8547_device *sc)
{
	unsigned int reg06, reg07, reg0e;
	int ret;

	if (!sc->init_done || !sc->window.complete)
		return -EPERM;
	if (!sc8547_variant_control_supported(sc->variant))
		return -EOPNOTSUPP;

	ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg07);
	if (ret)
		return ret;
	if (reg07 & SC8547_CHG_EN)
		return -EBUSY;

	ret = sc8547_read_status(sc, &reg06, &reg0e);
	if (ret)
		return ret;
	if (!(reg0e & SC8547_ADAPTER_INSERT_STAT) ||
	    !(reg0e & SC8547_VBAT_INSERT_STAT))
		return -ENODEV;
	if (reg06 & SC8547_CP_SWITCHING_STAT)
		return -EBUSY;

	/*
	 * SC8547A reports VBUS_ERRORLO while the pump is stopped even after the
	 * source has reached the vendor 2:1 start voltage. The vendor start path
	 * does not reject that status before CP_SET_WORK_START. Keep every other
	 * blocking fault fatal here; sc8547_pulse_wait_start_window() performs the
	 * authoritative, repeated ADC check against the live 2 * VBAT target.
	 */
	if ((reg06 & (SC8547_BLOCKING_06 & ~SC8547_VBUS_ERRORLO_STAT)) ||
	    (reg0e & SC8547_BLOCKING_0E))
		return -EIO;

	return 0;
}

static int sc8547_pulse_rearm_runtime(struct sc8547_device *sc,
				       struct sc8547_pulse_result *p)
{
	unsigned int reg07, reg09, reg11;
	int ret;

	ret = regmap_read(sc->regmap, SC8547_REG_MODE_CTRL, &reg09);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_ADC_CTRL, &reg11);
	if (ret)
		return ret;
	p->rearm_entry_reg09 = reg09;
	p->rearm_entry_reg11 = reg11;
	p->rearm_attempted = true;

	/*
	 * The vendor policy explicitly enables ADC and its five-second watchdog
	 * before work-start. Any I2C access refreshes that watchdog; its dedicated
	 * kick helper reads REG36, while each monitor sample performs several
	 * register/ADC reads. A userspace split between the PPS ramp and bounded pulse
	 * can expose the watchdog-expired state (REG09[3]) and ADC=0. Reproduce
	 * those harmless runtime setup calls only after the pump-off preflight;
	 * live VBUS/VBAT are sampled and checked again before REG07 is enabled.
	 */
	ret = regmap_write(sc->regmap, SC8547_REG_ADC_CTRL, SC8547_ADC_EN);
	if (ret)
		return ret;
	ret = sc8547_set_watchdog_code(sc, sc8547_vendor_watchdog(sc));
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg07);
	if (ret)
		return ret;

	ret = regmap_read(sc->regmap, SC8547_REG_MODE_CTRL, &reg09);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_ADC_CTRL, &reg11);
	if (ret)
		return ret;
	p->rearm_final_reg09 = reg09;
	p->rearm_final_reg11 = reg11;
	p->rearm_snapshot_valid = true;

	if ((reg09 & SC8547_WATCHDOG_MASK) !=
	    sc8547_vendor_watchdog(sc) ||
	    (reg11 & SC8547_ADC_MODE_MASK) != SC8547_ADC_EN)
		return -EIO;

	return 0;
}

static int sc8547_pulse_wait_start_window(struct sc8547_device *sc,
					   struct sc8547_pulse_result *p)
{
	const struct sc8547_enable_window *w = &sc->window;
	unsigned int reg06, reg0e;
	unsigned int stable_samples = 0;
	unsigned int elapsed_ms = 0;
	u64 target_uv;
	int ret;

	/*
	 * Authorize the source while the already-running ADC and insertion
	 * detection are still valid. A later register reset makes the pump-side
	 * VBUS ADC read low until CHG_EN on this board, so the reset must happen
	 * only after an in-window sample has established a valid source and
	 * battery. This deliberately matches oplus_pps_charge_start(), which
	 * proceeds on its first CP_VIN sample in target..target+200 mV; requiring
	 * consecutive 10 ms samples here caused valid PPS ripple to time out even
	 * when both the first and final readings were in range. Keep the absolute
	 * voltage limits, hard timeout, and fatal-fault rejection intact.
	 */
	for (;;) {
		p->pre_vbus_uv = sc8547_get_vbus_uv(sc);
		if (p->pre_vbus_uv < 0)
			return p->pre_vbus_uv;
		p->pre_vbat_uv = sc8547_get_vbat_uv(sc);
		if (p->pre_vbat_uv < 0)
			return p->pre_vbat_uv;
		if (sc8547_uses_vout_reference(sc))
			p->pre_output_reference_uv = sc8547_get_vout_uv(sc);
		else
			p->pre_output_reference_uv = p->pre_vbat_uv;
		if (p->pre_output_reference_uv < 0)
			return p->pre_output_reference_uv;

		if (!p->first_adc_snapshot_valid) {
			p->first_vbus_uv = p->pre_vbus_uv;
			p->first_vbat_uv = p->pre_vbat_uv;
			p->first_output_reference_uv =
				p->pre_output_reference_uv;
			p->first_adc_snapshot_valid = true;
		}

		/*
		 * The vendor policy derives this target from the system gauge, not
		 * from the pump-local VBAT ADC. qcom_battmgr hands that exact value
		 * over before preparing the pump. Retain the local derivation only as
		 * a fail-closed fallback for older test paths.
		 */
		if (sc->pps_target_vbus_uv) {
			target_uv = sc->pps_target_vbus_uv;
			p->policy_target_valid = true;
		} else {
			target_uv = ((u64)p->pre_output_reference_uv * 2 /
				     100000) * 100000;
			target_uv += w->start_delta_uv;
			p->policy_target_valid = false;
		}
		p->target_vbus_uv = (int)target_uv;
		p->adc_wait_ms = elapsed_ms;
		p->preflight_snapshot_valid = true;

		/* High readings are unsafe, not merely conversion-not-ready. */
		if ((u32)p->pre_vbus_uv > w->vbus_max_uv ||
		    (u32)p->pre_output_reference_uv > w->vbat_max_uv)
			return -ERANGE;

		if ((u32)p->pre_vbus_uv >= w->vbus_min_uv &&
		    (u32)p->pre_output_reference_uv >= w->vbat_min_uv &&
		    (u64)p->pre_vbus_uv >= target_uv &&
		    (u64)p->pre_vbus_uv <=
				target_uv + w->start_overshoot_max_uv) {
			stable_samples++;
			if (stable_samples >= SC8547_ADC_READY_STABLE_SAMPLES)
				return 0;
		} else {
			stable_samples = 0;
		}

		ret = sc8547_read_status(sc, &reg06, &reg0e);
		if (ret)
			return ret;
		if (sc8547_has_fatal_fault(reg06, reg0e))
			return -EIO;
		if (elapsed_ms >= SC8547_ADC_READY_TIMEOUT_MS)
			return -ERANGE;

		msleep(SC8547_ADC_READY_POLL_MS);
		elapsed_ms += SC8547_ADC_READY_POLL_MS;
	}
}

static int
sc8547_prepare_vendor_pps_startup(struct sc8547_device *sc,
				  struct sc8547_pulse_result *p,
				  bool *touched)
{
	const struct sc8547_raw_profile *idle = &sc->profile;
	unsigned int reg02, reg04, reg05, reg08, reg09, reg0c, reg0d, reg10;
	unsigned int reg11, reg2b, reg30, reg3a, reg3b, reg3c;
	bool secondary = !strcmp(sc->role, "secondary");
	u8 pps_ibus_prot;
	int ret;

	*touched = false;
	if (!sc8547_bounded_role_supported(sc))
		return -EOPNOTSUPP;

	ret = regmap_read(sc->regmap, SC8547_REG_IBUS_PROT, &reg05);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_SS_CTRL, &reg08);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_MODE_CTRL, &reg09);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_IBUS_CTRL, &reg0c);
	if (ret)
		return ret;

	p->pre_reg05 = reg05;
	p->pre_reg08 = reg08;
	p->pre_reg09 = reg09;
	p->pre_reg0c = reg0c;
	p->control_snapshot_valid = true;

	/*
	 * Match the wired-PPS path, not the generic CP_ENABLE helper. In
	 * oplus_chg_pps.c the CP_ENABLE call is intentionally commented out; the
	 * policy uses SET_WORK_MODE followed by CP_SET_WORK_START. Keeping the
	 * pump out of reset also preserves the already-authorized adapter path.
	 */
	*touched = true;
	ret = regmap_write(sc->regmap, SC8547_REG_AC_OVP, idle->reg02);
	if (ret)
		return ret;
	ret = regmap_write(sc->regmap, SC8547_REG_VBUS_OVP, idle->reg04);
	if (ret)
		return ret;
	/*
	 * CP_HW_INIT runs once before the downstream PPS state machine. Preserve
	 * the live ADC path by avoiding its reset here. The primary and secondary
	 * vendor drivers use different side-register profiles, so only write the
	 * fields documented for this physical role.
	 */
	ret = regmap_write(sc->regmap, SC8547_REG_VOOCPHY_CTRL,
			   SC8547A_VENDOR_VOOCPHY_DISABLE);
	if (ret)
		return ret;
	if (!secondary) {
		ret = regmap_write(sc->regmap, SC8547_REG_INT_MASK,
				   SC8547A_VENDOR_INSERT_IRQ_MASK);
		if (ret)
			return ret;
		ret = regmap_write(sc->regmap, SC8547A_REG_OTG_CTRL,
				   SC8547A_VENDOR_OTG_CTRL);
		if (ret)
			return ret;
	}
	/*
	 * caihong's SC8547A low-current/FCL test path keeps IBUS UCP and the
	 * coupled soft-start timeout disabled because its deliberately limited
	 * 1-A source measured only 0.2--0.4 A and could autonomously exceed the
	 * 81.92-s REG08 timeout. giulia's SC8547D downstream SVOOC work setting
	 * instead keeps the profile's UCP field, so only the non-D parts force
	 * the disable. All OCP/OVP fields are preserved either way and runtime
	 * VBUS/VAC/ratio/temperature checks stay fail-closed every 500 ms.
	 */
	pps_ibus_prot = idle->reg05;
	if (sc8547_forces_ucp_disable(sc))
		pps_ibus_prot |= SC8547_IBUS_UCP_DIS;
	ret = regmap_write(sc->regmap, SC8547_REG_IBUS_PROT,
			   pps_ibus_prot);
	if (ret)
		return ret;
	ret = regmap_update_bits(sc->regmap, SC8547_REG_SS_CTRL,
				 SC8547_SS_TIMEOUT_MASK,
				 sc8547_variant_is_d(sc) ?
				 SC8547D_VENDOR_LOW_CURRENT_SS_TIMEOUT :
				 SC8547A_VENDOR_LOW_CURRENT_SS_TIMEOUT);
	if (ret)
		return ret;
	ret = regmap_update_bits(sc->regmap, SC8547_REG_MODE_CTRL,
				 SC8547A_VENDOR_PPS_MODE_MASK,
				 SC8547A_VENDOR_PPS_MODE_2TO1);
	if (ret)
		return ret;
	ret = regmap_write(sc->regmap, SC8547_REG_ADC_CTRL, SC8547_ADC_EN);
	if (ret)
		return ret;
	ret = sc8547_enable_required_adc_channels(sc);
	if (ret)
		return ret;
	ret = regmap_write(sc->regmap, SC8547_REG_PMID2OUT,
			   sc8547_variant_is_d(sc) ?
			   SC8547D_VENDOR_PPS_PMID2OUT :
			   SC8547A_VENDOR_PPS_PMID2OUT);
	if (ret)
		return ret;
	if (secondary) {
		/* Values used by the downstream SC8547 slave init/SVOOC path. */
		ret = regmap_write(sc->regmap, SC8547_REG_SLAVE_CTRL,
				   SC8547_VENDOR_SECONDARY_CTRL);
		if (ret)
			return ret;
		ret = regmap_write(sc->regmap, SC8547_REG_VBUS_RANGE_CTRL,
				   sc8547_variant_is_d(sc) ?
				   SC8547D_VENDOR_SECONDARY_VBUS_RANGE :
				   SC8547_VENDOR_SECONDARY_VBUS_RANGE);
		if (ret)
			return ret;
		if (sc8547_variant_is_d(sc)) {
			/*
			 * The D downstream SVOOC image also sets LOOSE_DET and
			 * the 0x34 timing on the secondary pump.
			 */
			ret = regmap_write(sc->regmap, SC8547_REG_LOOSE_DET,
					   SC8547D_VENDOR_PPS_LOOSE_DET);
			if (ret)
				return ret;
			ret = regmap_write(sc->regmap,
					   sc8547_vendor_timing_reg(sc),
					   sc8547_vendor_timing_value(sc));
			if (ret)
				return ret;
		}
	} else {
		ret = regmap_write(sc->regmap, SC8547_REG_LOOSE_DET,
				   sc8547_variant_is_d(sc) ?
				   SC8547D_VENDOR_PPS_LOOSE_DET :
				   SC8547A_VENDOR_PPS_LOOSE_DET);
		if (ret)
			return ret;
		ret = regmap_write(sc->regmap, sc8547_vendor_timing_reg(sc),
				   sc8547_vendor_timing_value(sc));
		if (ret)
			return ret;
	}
	/* oplus_chg_pps.c enables the five-second watchdog before its VBUS ramp. */
	ret = sc8547_set_watchdog_code(sc, sc8547_vendor_watchdog(sc));
	if (ret)
		return ret;

	/* REG07 off is role/variant-specific (primary 0x00, secondary 0x05). */
	ret = sc8547_set_charge_enabled(sc, false);
	if (ret)
		return ret;

	ret = regmap_read(sc->regmap, SC8547_REG_AC_OVP, &reg02);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_VBUS_OVP, &reg04);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_IBUS_PROT, &reg05);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_SS_CTRL, &reg08);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_MODE_CTRL, &reg09);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_IBUS_CTRL, &reg0c);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_PMID2OUT, &reg0d);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_INT_MASK, &reg10);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_ADC_CTRL, &reg11);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_VOOCPHY_CTRL, &reg2b);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_SLAVE_CTRL, &reg30);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, sc8547_vendor_timing_reg(sc), &reg3a);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547A_REG_OTG_CTRL, &reg3b);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_VBUS_RANGE_CTRL, &reg3c);
	if (ret)
		return ret;

	p->prep_reg02 = reg02;
	p->prep_reg04 = reg04;
	p->prep_reg05 = reg05;
	p->prep_reg08 = reg08;
	p->prep_reg09 = reg09;
	p->prep_reg0c = reg0c;
	p->prep_reg0d = reg0d;
	p->prep_reg10 = reg10;
	p->prep_reg11 = reg11;
	p->prep_reg2b = reg2b;
	p->prep_reg30 = reg30;
	p->prep_reg3a = reg3a;
	p->prep_reg3b = reg3b;
	p->prep_reg3c = reg3c;
	p->prep_snapshot_valid = true;

	if ((reg02 & SC8547_AC_OVP_CONFIG_MASK) !=
	    (idle->reg02 & SC8547_AC_OVP_CONFIG_MASK))
		p->prep_mismatch |= SC8547_PREP_MISMATCH_REG02_CONFIG;
	if (reg02 & SC8547_AC_OVP_STAT)
		p->prep_mismatch |= SC8547_PREP_MISMATCH_REG02_STATUS;
	if (reg04 != idle->reg04)
		p->prep_mismatch |= SC8547_PREP_MISMATCH_REG04;
	if (reg05 != pps_ibus_prot)
		p->prep_mismatch |= SC8547_PREP_MISMATCH_REG05;
	if ((reg08 & SC8547_SS_TIMEOUT_MASK) !=
	    (sc8547_variant_is_d(sc) ? SC8547D_VENDOR_LOW_CURRENT_SS_TIMEOUT :
				       SC8547A_VENDOR_LOW_CURRENT_SS_TIMEOUT))
		p->prep_mismatch |= SC8547_PREP_MISMATCH_REG08;
	if ((reg09 & (SC8547A_VENDOR_PPS_MODE_MASK | SC8547_WATCHDOG_MASK)) !=
	    (SC8547A_VENDOR_PPS_MODE_2TO1 | sc8547_vendor_watchdog(sc)))
		p->prep_mismatch |= SC8547_PREP_MISMATCH_REG09;
	if ((reg0d & (SC8547_PMID2OUT_UVP_MASK | SC8547_PMID2OUT_OVP_MASK)) !=
	    (sc8547_variant_is_d(sc) ? SC8547D_VENDOR_PPS_PMID2OUT :
				       SC8547A_VENDOR_PPS_PMID2OUT))
		p->prep_mismatch |= SC8547_PREP_MISMATCH_REG0D;
	if ((reg11 & SC8547_ADC_MODE_MASK) != SC8547_ADC_EN)
		p->prep_mismatch |= SC8547_PREP_MISMATCH_REG11;
	if (reg2b != SC8547A_VENDOR_VOOCPHY_DISABLE)
		p->prep_mismatch |= SC8547_PREP_MISMATCH_REG2B;
	if (secondary) {
		if (!sc8547_variant_is_d(sc) &&
		    reg30 != SC8547_VENDOR_SECONDARY_CTRL)
			p->prep_mismatch |= SC8547_PREP_MISMATCH_REG30;
		if (reg3c != (sc8547_variant_is_d(sc) ?
			      SC8547D_VENDOR_SECONDARY_VBUS_RANGE :
			      SC8547_VENDOR_SECONDARY_VBUS_RANGE))
			p->prep_mismatch |= SC8547_PREP_MISMATCH_REG3C;
		if (sc8547_variant_is_d(sc) &&
		    reg3a != sc8547_vendor_timing_value(sc))
			p->prep_mismatch |= SC8547_PREP_MISMATCH_REG3A;
	} else {
		if (reg10 != SC8547A_VENDOR_INSERT_IRQ_MASK)
			p->prep_mismatch |= SC8547_PREP_MISMATCH_REG10;
		if (reg3a != sc8547_vendor_timing_value(sc))
			p->prep_mismatch |= SC8547_PREP_MISMATCH_REG3A;
		if (reg3b != SC8547A_VENDOR_OTG_CTRL)
			p->prep_mismatch |= SC8547_PREP_MISMATCH_REG3B;
	}

	if (p->prep_mismatch)
		return -EIO;

	return 0;
}

/*
 * The vendor policy prepares the pump while the source is still at its low
 * PPS entry voltage, then suspends the buck and raises VBUS.  qcom_battmgr
 * invokes the CALIBRATE coordination property at that point.  The bounded
 * pulse must only verify the preserved state here; rewriting work-mode
 * registers after VBUS is high was the ordering error in earlier stages.
 */
static int sc8547_prepare_pps_startup(struct sc8547_device *sc,
				      struct sc8547_pulse_result *p,
				      bool *touched)
{
	const struct sc8547_raw_profile *idle = &sc->profile;
	unsigned int reg02, reg04, reg05, reg08, reg09, reg0c, reg0d, reg10;
	unsigned int reg11, reg2b, reg30, reg3a, reg3b, reg3c;
	bool secondary = !strcmp(sc->role, "secondary");
	u8 pps_ibus_prot;
	int ret;

	*touched = false;
	if (!sc8547_bounded_role_supported(sc))
		return -EOPNOTSUPP;
	if (!sc->pps_prepared)
		return -EPERM;
	*touched = true;

	ret = regmap_read(sc->regmap, SC8547_REG_AC_OVP, &reg02);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_VBUS_OVP, &reg04);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_IBUS_PROT, &reg05);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_SS_CTRL, &reg08);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_MODE_CTRL, &reg09);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_IBUS_CTRL, &reg0c);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_PMID2OUT, &reg0d);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_INT_MASK, &reg10);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_ADC_CTRL, &reg11);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_VOOCPHY_CTRL, &reg2b);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_SLAVE_CTRL, &reg30);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, sc8547_vendor_timing_reg(sc), &reg3a);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547A_REG_OTG_CTRL, &reg3b);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_VBUS_RANGE_CTRL, &reg3c);
	if (ret)
		return ret;

	p->pre_reg05 = reg05;
	p->pre_reg08 = reg08;
	p->pre_reg09 = reg09;
	p->pre_reg0c = reg0c;
	p->control_snapshot_valid = true;
	p->prep_reg02 = reg02;
	p->prep_reg04 = reg04;
	p->prep_reg05 = reg05;
	p->prep_reg08 = reg08;
	p->prep_reg09 = reg09;
	p->prep_reg0c = reg0c;
	p->prep_reg0d = reg0d;
	p->prep_reg10 = reg10;
	p->prep_reg11 = reg11;
	p->prep_reg2b = reg2b;
	p->prep_reg30 = reg30;
	p->prep_reg3a = reg3a;
	p->prep_reg3b = reg3b;
	p->prep_reg3c = reg3c;
	p->prep_snapshot_valid = true;
	pps_ibus_prot = idle->reg05;
	if (sc8547_forces_ucp_disable(sc))
		pps_ibus_prot |= SC8547_IBUS_UCP_DIS;

	if ((reg02 & SC8547_AC_OVP_CONFIG_MASK) !=
	    (idle->reg02 & SC8547_AC_OVP_CONFIG_MASK))
		p->prep_mismatch |= SC8547_PREP_MISMATCH_REG02_CONFIG;
	if (reg02 & SC8547_AC_OVP_STAT)
		p->prep_mismatch |= SC8547_PREP_MISMATCH_REG02_STATUS;
	if (reg04 != idle->reg04)
		p->prep_mismatch |= SC8547_PREP_MISMATCH_REG04;
	if (reg05 != pps_ibus_prot)
		p->prep_mismatch |= SC8547_PREP_MISMATCH_REG05;
	if ((reg08 & SC8547_SS_TIMEOUT_MASK) !=
	    (sc8547_variant_is_d(sc) ? SC8547D_VENDOR_LOW_CURRENT_SS_TIMEOUT :
				       SC8547A_VENDOR_LOW_CURRENT_SS_TIMEOUT))
		p->prep_mismatch |= SC8547_PREP_MISMATCH_REG08;
	if ((reg09 & (SC8547A_VENDOR_PPS_MODE_MASK | SC8547_WATCHDOG_MASK)) !=
	    (SC8547A_VENDOR_PPS_MODE_2TO1 | sc8547_vendor_watchdog(sc)))
		p->prep_mismatch |= SC8547_PREP_MISMATCH_REG09;
	if ((reg0d & (SC8547_PMID2OUT_UVP_MASK | SC8547_PMID2OUT_OVP_MASK)) !=
	    (sc8547_variant_is_d(sc) ? SC8547D_VENDOR_PPS_PMID2OUT :
				       SC8547A_VENDOR_PPS_PMID2OUT))
		p->prep_mismatch |= SC8547_PREP_MISMATCH_REG0D;
	if ((reg11 & SC8547_ADC_MODE_MASK) != SC8547_ADC_EN)
		p->prep_mismatch |= SC8547_PREP_MISMATCH_REG11;
	if (reg2b != SC8547A_VENDOR_VOOCPHY_DISABLE)
		p->prep_mismatch |= SC8547_PREP_MISMATCH_REG2B;
	if (secondary) {
		if (!sc8547_variant_is_d(sc) &&
		    reg30 != SC8547_VENDOR_SECONDARY_CTRL)
			p->prep_mismatch |= SC8547_PREP_MISMATCH_REG30;
		if (reg3c != (sc8547_variant_is_d(sc) ?
			      SC8547D_VENDOR_SECONDARY_VBUS_RANGE :
			      SC8547_VENDOR_SECONDARY_VBUS_RANGE))
			p->prep_mismatch |= SC8547_PREP_MISMATCH_REG3C;
		if (sc8547_variant_is_d(sc) &&
		    reg3a != sc8547_vendor_timing_value(sc))
			p->prep_mismatch |= SC8547_PREP_MISMATCH_REG3A;
	} else {
		if (reg10 != SC8547A_VENDOR_INSERT_IRQ_MASK)
			p->prep_mismatch |= SC8547_PREP_MISMATCH_REG10;
		if (reg3a != sc8547_vendor_timing_value(sc))
			p->prep_mismatch |= SC8547_PREP_MISMATCH_REG3A;
		if (reg3b != SC8547A_VENDOR_OTG_CTRL)
			p->prep_mismatch |= SC8547_PREP_MISMATCH_REG3B;
	}

	if (p->prep_mismatch)
		return -EIO;

	return 0;
}

static int sc8547_post_prepare_check(struct sc8547_device *sc,
				      struct sc8547_pulse_result *p)
{
	unsigned int reg06, reg07, reg0e, reg11, reg12;
	int output_reference_uv;
	int vbat_uv;
	int vbus_uv;
	int ret;

	ret = sc8547_read_status(sc, &reg06, &reg0e);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg07);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_ADC_CTRL, &reg11);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_ADC_FN_DISABLE, &reg12);
	if (ret)
		return ret;

	p->ready_reg06 = reg06;
	p->ready_reg07 = reg07;
	p->ready_reg0e = reg0e;
	p->ready_reg11 = reg11;
	p->ready_reg12 = reg12;
	p->ready_snapshot_valid = true;

	/* The no-reset PPS work-start path must retain live input telemetry. */
	vbus_uv = sc8547_get_vbus_uv(sc);
	if (vbus_uv < 0)
		return vbus_uv;
	vbat_uv = sc8547_get_vbat_uv(sc);
	if (vbat_uv < 0)
		return vbat_uv;
	p->post_prepare_vbus_uv = vbus_uv;
	p->post_prepare_vbat_uv = vbat_uv;
	p->post_prepare_adc_snapshot_valid = true;
	ret = sc8547_capture_path_snapshot(sc, &p->ready_vac_uv,
					   &p->ready_vout_uv,
					   &p->ready_reg33,
					   &p->ready_reg3a,
					   &p->ready_reg3b,
					   &p->ready_reg3c);
	if (ret)
		return ret;
	p->ready_extended_snapshot_valid = true;
	output_reference_uv = sc8547_select_output_reference_uv(
		sc, p->ready_vout_uv, vbat_uv);
	p->post_prepare_output_reference_uv = output_reference_uv;

	if ((reg07 & SC8547_CHG_EN) ||
	    (reg06 & SC8547_CP_SWITCHING_STAT))
		return -EBUSY;
	if ((reg11 & SC8547_ADC_MODE_MASK) != SC8547_ADC_EN ||
	    (reg12 & SC8547_REQUIRED_ADC_DIS_MASK))
		return -EIO;
	if (!(reg0e & SC8547_ADAPTER_INSERT_STAT) ||
	    !(reg0e & SC8547_VBAT_INSERT_STAT))
		return -ENODEV;
	/*
	 * The live vendor PPS path does not inspect the pre-start VBUS_ERRORLO
	 * bit before CP_SET_WORK_START. The independently sampled VBUS window
	 * below remains authoritative; every other blocking condition still
	 * aborts.
	 */
	if ((reg06 & (SC8547_BLOCKING_06 & ~SC8547_VBUS_ERRORLO_STAT)) ||
	    (reg0e & SC8547_BLOCKING_0E))
		return -EIO;
	if ((u32)vbus_uv < sc->window.vbus_min_uv ||
	    (u32)vbus_uv > sc->window.vbus_max_uv ||
	    (u32)output_reference_uv < sc->window.vbat_min_uv ||
	    (u32)output_reference_uv > sc->window.vbat_max_uv ||
	    vbus_uv < p->target_vbus_uv ||
	    vbus_uv > p->target_vbus_uv +
			(int)sc->window.start_overshoot_max_uv)
		return -ERANGE;

	return 0;
}

static int sc8547_post_prepare_check_confirmed(
	struct sc8547_device *sc, struct sc8547_pulse_result *p)
{
	unsigned int attempt;
	int ret;

	ret = sc8547_post_prepare_check(sc, p);
	if (ret != -ERANGE)
		return ret;

	/*
	 * Preserve the exact vendor target-through-target+200-mV window, but do
	 * not reject an otherwise ready pump on one torn ADC conversion. Status,
	 * insertion, register and I/O failures remain immediate. This mirrors the
	 * bounded numeric-only confirmation already used by the active monitor.
	 */
	for (attempt = 0;
	     attempt < SC8547_POST_PREPARE_RANGE_RECHECKS; attempt++) {
		usleep_range(SC8547_RUNTIME_RANGE_RECHECK_MIN_US,
			      SC8547_RUNTIME_RANGE_RECHECK_MAX_US);
		p->post_prepare_range_rechecks++;
		ret = sc8547_post_prepare_check(sc, p);
		if (ret != -ERANGE) {
			if (!ret)
				p->post_prepare_range_recoveries++;
			return ret;
		}
	}

	return ret;
}

static int sc8547_capture_reject_dump(struct sc8547_device *sc,
				       struct sc8547_pulse_result *p)
{
	unsigned int val;
	unsigned int reg;
	int ret;

	p->reject_dump_valid = false;
	for (reg = SC8547_DIAG_FIRST_REG; reg <= SC8547_DIAG_LAST_REG; reg++) {
		ret = regmap_read(sc->regmap, reg, &val);
		if (ret)
			return ret;
		p->reject_regs[reg] = val;
	}
	p->reject_dump_valid = true;

	p->reject_vbus_uv = sc8547_get_vbus_uv(sc);
	if (p->reject_vbus_uv < 0)
		return 0;
	p->reject_vbat_uv = sc8547_get_vbat_uv(sc);
	if (p->reject_vbat_uv < 0)
		return 0;
	ret = sc8547_capture_path_snapshot(sc, &p->reject_vac_uv,
					   &p->reject_vout_uv,
					   &p->reject_reg33,
					   &p->reject_reg3a,
					   &p->reject_reg3b,
					   &p->reject_reg3c);
	if (!ret)
		p->reject_extended_snapshot_valid = true;

	return 0;
}

static int sc8547_retry_start_preflight(struct sc8547_device *sc,
					 struct sc8547_pulse_result *p)
{
	const struct sc8547_enable_window *w = &sc->window;
	unsigned int reg06, reg0e;
	int output_reference_uv;
	int tdie_mc;
	int vbus_uv;
	int ret;

	ret = sc8547_read_status(sc, &reg06, &reg0e);
	if (ret)
		return ret;
	if (!(reg0e & SC8547_ADAPTER_INSERT_STAT) ||
	    !(reg0e & SC8547_VBAT_INSERT_STAT))
		return -ENODEV;
	if ((reg06 & SC8547_CP_SWITCHING_STAT) ||
	    sc8547_has_fatal_fault(reg06, reg0e))
		return -EIO;

	vbus_uv = sc8547_get_vbus_uv(sc);
	if (vbus_uv < 0)
		return vbus_uv;
	output_reference_uv = sc8547_get_output_reference_uv(sc);
	if (output_reference_uv < 0)
		return output_reference_uv;
	tdie_mc = sc8547_get_tdie_mc(sc);
	if (tdie_mc < 0)
		return tdie_mc;
	if ((u32)vbus_uv < w->vbus_min_uv ||
	    (u32)vbus_uv > w->vbus_max_uv ||
	    (u32)output_reference_uv < w->vbat_min_uv ||
	    (u32)output_reference_uv > w->vbat_max_uv ||
	    (u32)tdie_mc > sc->pulse_tdie_max_mc ||
	    vbus_uv < p->target_vbus_uv ||
	    vbus_uv > p->target_vbus_uv +
			(int)w->start_overshoot_max_uv)
		return -ERANGE;

	return 0;
}


int sc8547_manual_preflight(struct i2c_client *client)
{
	struct sc8547_device *sc = sc8547_from_client(client);
	int ret;

	if (!sc)
		return -ENODEV;

	mutex_lock(&sc->lock);
	if (!sc->allow_experimental_cp_enable)
		ret = -EPERM;
	else
		ret = sc8547_enable_preflight(sc);
	mutex_unlock(&sc->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547_manual_preflight);

static int sc8547_post_enable_check(struct sc8547_device *sc)
{
	unsigned int reg06, reg0e;
	int ret;

	ret = sc8547_read_status(sc, &reg06, &reg0e);
	if (ret)
		return ret;
	if (!(reg0e & SC8547_ADAPTER_INSERT_STAT) ||
	    !(reg0e & SC8547_VBAT_INSERT_STAT))
		return -ENODEV;
	if (!(reg06 & SC8547_CP_SWITCHING_STAT))
		return -EIO;
	if (sc8547_has_blocking_fault(reg06, reg0e))
		return -EIO;

	return sc8547_window_check(sc);
}

static int sc8547_manual_disable(struct sc8547_device *sc)
{
	unsigned int reg;
	int ret;

	ret = sc8547_set_charge_enabled(sc, false);
	if (ret)
		return ret;
	usleep_range(10000, 20000);

	ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg);
	if (ret)
		return ret;
	if (reg & SC8547_CHG_EN)
		return -EIO;

	return 0;
}

static int sc8547_capture_active_sample(struct sc8547_device *sc,
					 struct sc8547_pulse_result *p,
					 struct sc8547_active_sample *s)
{
	const struct sc8547_enable_window *w = &sc->window;
	unsigned int reg06, reg07, reg0e;
	u32 ibus_max_ua = p->vendor_current_control ?
		SC8547_VENDOR_CURRENT_IBUS_ADC_MAX_UA :
		SC8547_TEST_IBUS_MAX_UA;
	int headroom_uv;
	int ret;

	memset(s, 0, sizeof(*s));
	ret = sc8547_read_status(sc, &reg06, &reg0e);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg07);
	if (ret)
		return ret;

	s->reg06 = reg06;
	s->reg07 = reg07;
	s->reg0e = reg0e;
	s->vbus_uv = sc8547_get_vbus_uv(sc);
	if (s->vbus_uv < 0)
		return s->vbus_uv;
	s->vbat_uv = sc8547_get_vbat_uv(sc);
	if (s->vbat_uv < 0)
		return s->vbat_uv;
	ret = sc8547_capture_path_snapshot(sc, &s->vac_uv, &s->vout_uv,
					   &s->reg33, &s->reg3a,
					   &s->reg3b, &s->reg3c);
	if (ret)
		return ret;
	s->output_reference_uv = sc8547_select_output_reference_uv(
		sc, s->vout_uv, s->vbat_uv);
	s->ibus_ua = sc8547_get_ibus_ua(sc);
	if (s->ibus_ua < 0)
		return s->ibus_ua;
	s->tdie_mc = sc8547_get_tdie_mc(sc);
	if (s->tdie_mc < 0)
		return s->tdie_mc;

	/*
	 * The vendor's +100-mV third-party PPS floor constrains the requested
	 * source voltage and is derived from gauge VBAT.  It is not a physical
	 * worker shutdown threshold against a pump-local ADC: on this board the
	 * primary VBAT reading can sit above the gauge terminal reading, while the
	 * secondary VBAT channel is not connected.  Keep the role-correct local
	 * output as a diagnostic and reject only excessive low-side conversion
	 * error here.  Positive headroom remains bounded independently by the
	 * absolute VBUS/VAC, output, current, temperature and hardware-fault gates.
	 */
	headroom_uv = s->vbus_uv - 2 * s->output_reference_uv;
	s->headroom_uv = headroom_uv;
	s->ratio_error_uv = s->vbus_uv - 2 * s->output_reference_uv;
	if (s->ratio_error_uv < 0)
		s->ratio_error_uv = -s->ratio_error_uv;
	s->valid = true;

	if (!(reg07 & SC8547_CHG_EN) ||
	    !(reg06 & SC8547_CP_SWITCHING_STAT) ||
	    sc8547_has_blocking_fault(reg06, reg0e))
		return -EIO;
	if ((u32)s->vbus_uv < w->pulse_vbus_min_uv ||
	    (u32)s->vbus_uv > w->vbus_max_uv ||
	    (u32)s->vac_uv < w->pulse_vbus_min_uv ||
	    (u32)s->vac_uv > w->vbus_max_uv ||
	    (u32)s->output_reference_uv < w->vbat_min_uv ||
	    (u32)s->output_reference_uv > w->vbat_max_uv ||
	    (p->vendor_current_control ?
	     headroom_uv < -(int)w->pulse_ratio_error_max_uv :
	     (u32)s->ratio_error_uv > w->pulse_ratio_error_max_uv) ||
	    (u32)s->ibus_ua > ibus_max_ua ||
	    (u32)s->tdie_mc > sc->pulse_tdie_max_mc)
		return -ERANGE;

	return 0;
}

static int sc8547_capture_confirmed_active_sample(
	struct sc8547_device *sc, struct sc8547_pulse_result *p,
	struct sc8547_active_sample *s)
{
	unsigned int attempt;
	int ret;

	ret = sc8547_capture_active_sample(sc, p, s);
	if (ret != -ERANGE)
		return ret;

	/*
	 * Status faults, a stopped pump and I2C errors remain immediate failures.
	 * Only a numeric-window rejection is sampled twice more.  This filters a
	 * torn or transient ADC conversion without relaxing any hardware limit.
	 */
	for (attempt = 0; attempt < SC8547_RUNTIME_RANGE_RECHECKS; attempt++) {
		if (atomic_read(&sc->policy_cancel))
			return -ECANCELED;
		usleep_range(SC8547_RUNTIME_RANGE_RECHECK_MIN_US,
			      SC8547_RUNTIME_RANGE_RECHECK_MAX_US);
		p->runtime_range_rechecks++;
		ret = sc8547_capture_active_sample(sc, p, s);
		if (ret != -ERANGE) {
			if (!ret)
				p->runtime_range_recoveries++;
			return ret;
		}
	}

	return ret;
}

static void sc8547_publish_active_sample(struct sc8547_pulse_result *p,
					  const struct sc8547_active_sample *s)
{
	p->on_reg06 = s->reg06;
	p->on_reg07 = s->reg07;
	p->on_reg0e = s->reg0e;
	p->on_reg33 = s->reg33;
	p->on_reg3a = s->reg3a;
	p->on_reg3b = s->reg3b;
	p->on_reg3c = s->reg3c;
	p->on_vac_uv = s->vac_uv;
	p->vbus_uv = s->vbus_uv;
	p->on_vout_uv = s->vout_uv;
	p->vbat_uv = s->vbat_uv;
	p->output_reference_uv = s->output_reference_uv;
	p->headroom_uv = s->headroom_uv;
	p->active_target_vbus_uv = 2 * s->output_reference_uv;
	p->ratio_error_uv = s->ratio_error_uv;
	p->ibus_ua = s->ibus_ua;
	p->tdie_mc = s->tdie_mc;
	p->on_snapshot_valid = true;
	p->on_extended_snapshot_valid = true;
}

static bool sc8547_record_runtime_sample(unsigned int sample,
					  unsigned int count,
					  bool continuous)
{
	unsigned int interval;

	if (continuous)
		return !sample ||
		       !((sample + 1) % SC8547_SOAK_REPORT_INTERVAL);
	if (count <= SC8547_VENDOR_START_ATTEMPTS)
		return true;
	if (count <= SC8547_STABILITY_RUN_MS /
		     SC8547_STABILITY_SAMPLE_MS)
		interval = SC8547_SHORT_REPORT_INTERVAL;
	else if (count <= SC8547_LONG_RUN_MS /
			  SC8547_STABILITY_SAMPLE_MS)
		interval = SC8547_LONG_REPORT_INTERVAL;
	else if (count <= SC8547_SOAK_RUN_MS /
			  SC8547_STABILITY_SAMPLE_MS)
		interval = SC8547_SOAK_REPORT_INTERVAL;
	else
		interval = DIV_ROUND_UP(count, SC8547_RUNTIME_RECORDS - 1);

	return !sample || !((sample + 1) % interval) || sample + 1 == count;
}

static void sc8547_update_runtime_range(struct sc8547_pulse_result *p,
					 const struct sc8547_active_sample *s)
{
	if (!p->runtime_observed) {
		p->runtime_vac_min_uv = p->runtime_vac_max_uv = s->vac_uv;
		p->runtime_vbus_min_uv = p->runtime_vbus_max_uv = s->vbus_uv;
		p->runtime_ratio_min_uv =
			p->runtime_ratio_max_uv = s->ratio_error_uv;
		p->runtime_ibus_min_ua = p->runtime_ibus_max_ua = s->ibus_ua;
		p->runtime_tdie_min_mc = p->runtime_tdie_max_mc = s->tdie_mc;
		p->runtime_observed = true;
		return;
	}

	p->runtime_vac_min_uv = min(p->runtime_vac_min_uv, s->vac_uv);
	p->runtime_vac_max_uv = max(p->runtime_vac_max_uv, s->vac_uv);
	p->runtime_vbus_min_uv = min(p->runtime_vbus_min_uv, s->vbus_uv);
	p->runtime_vbus_max_uv = max(p->runtime_vbus_max_uv, s->vbus_uv);
	p->runtime_ratio_min_uv =
		min(p->runtime_ratio_min_uv, s->ratio_error_uv);
	p->runtime_ratio_max_uv =
		max(p->runtime_ratio_max_uv, s->ratio_error_uv);
	p->runtime_ibus_min_ua = min(p->runtime_ibus_min_ua, s->ibus_ua);
	p->runtime_ibus_max_ua = max(p->runtime_ibus_max_ua, s->ibus_ua);
	p->runtime_tdie_min_mc = min(p->runtime_tdie_min_mc, s->tdie_mc);
	p->runtime_tdie_max_mc = max(p->runtime_tdie_max_mc, s->tdie_mc);
}

static int sc8547_pps_run(struct sc8547_device *sc, unsigned int run_ms,
			  bool vendor_current_control, bool continuous)
{
	struct sc8547_pulse_result *p = &sc->pulse;
	struct sc8547_active_sample active;
	unsigned int attempt;
	unsigned int sample;
	unsigned int sample_goal;
	unsigned int reg06, reg07, reg0e;
	bool prep_touched = false;
	int recovery_ret;
	int disable_ret = 0;
	int final_ret;
	int ret;

	atomic_set(&sc->pulse_running, 1);
	memset(p, 0, sizeof(*p));
	p->requested_run_ms = run_ms;
	p->vendor_current_control = vendor_current_control;
	if ((!continuous &&
	     (!run_ms || run_ms % SC8547_STABILITY_SAMPLE_MS ||
	      run_ms > SC8547_POLICY_RUN_MS)) ||
	    (continuous && run_ms)) {
		ret = -EINVAL;
		goto record;
	}
	sample_goal = continuous ? 0 :
		run_ms / SC8547_STABILITY_SAMPLE_MS;
	if (!sc->allow_experimental_cp_pulse ||
	    !sc8547_bounded_role_supported(sc)) {
		ret = -EPERM;
		goto record;
	}
	if (!sc->pps_prepared) {
		ret = -EPERM;
		goto record;
	}

	/* Preserve the vendor watchdog while confirming the role-specific off state. */
	ret = sc8547_manual_disable(sc);
	if (ret)
		goto record;
	usleep_range(10000, 20000);

	ret = sc8547_pulse_preflight(sc);
	if (ret)
		goto record;
	ret = sc8547_pulse_rearm_runtime(sc, p);
	if (ret)
		goto record;

	p->tdie_mc = sc8547_get_tdie_mc(sc);
	if (p->tdie_mc < 0) {
		ret = p->tdie_mc;
		goto record;
	}
	if ((u32)p->tdie_mc > sc->pulse_tdie_max_mc) {
		ret = -ERANGE;
		goto record;
	}
	ret = sc8547_pulse_preflight(sc);
	if (ret)
		goto record;
	ret = sc8547_pulse_wait_start_window(sc, p);
	if (ret)
		goto record;
	ret = sc8547_prepare_pps_startup(sc, p, &prep_touched);
	if (ret)
		goto disable;

	/*
	 * Re-check controls and live input after the no-reset PPS work-mode setup.
	 * Only then reproduce CP_SET_WORK_START with the bounded REG07 write.
	 */
	ret = sc8547_post_prepare_check_confirmed(sc, p);
	if (ret)
		goto disable;

	/*
	 * Match oplus_pps_charge_start() rather than failing its first immediate
	 * REG07 readback. The vendor state machine writes work-start, waits 500
	 * ms, checks REG07[7], and permits three failed checks before its terminal
	 * disable path. Revalidate the live target window before each retry and
	 * retain a compact per-attempt trace for the test result.
	 */
	for (attempt = 0; attempt < SC8547_VENDOR_START_ATTEMPTS; attempt++) {
		if (atomic_read(&sc->policy_cancel)) {
			ret = -ECANCELED;
			goto disable;
		}
		if (attempt) {
			ret = sc8547_retry_start_preflight(sc, p);
			if (ret)
				goto disable;
		}

		/* Use the downstream role/variant-specific work-start command. */
		ret = sc8547_set_charge_enabled(sc, true);
		if (ret)
			goto disable;
		p->start_attempts++;

		ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg07);
		if (ret)
			goto disable;
		p->start_immediate_reg07[attempt] = reg07;
		if (!attempt) {
			p->enable_reg07 = reg07;
			p->enable_snapshot_valid = true;
		}

		msleep(SC8547_VENDOR_START_CHECK_MS);
		ret = sc8547_read_status(sc, &reg06, &reg0e);
		if (ret)
			goto disable;
		ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg07);
		if (ret)
			goto disable;
		p->start_check_reg06[attempt] = reg06;
		p->start_check_reg07[attempt] = reg07;
		p->start_check_reg0e[attempt] = reg0e;
		p->start_checks++;

		if (reg07 & SC8547_CHG_EN)
			break;
		if (sc8547_has_fatal_fault(reg06, reg0e)) {
			ret = -EIO;
			goto reject;
		}
	}

	if (!(reg07 & SC8547_CHG_EN)) {
		ret = -EIO;
		goto reject;
	}

	/*
	 * The +450 mV target belongs only to the pre-start source ramp. For legacy
	 * bounded tests, retain the narrow 2:1 ratio-error window. The vendor-shaped
	 * current-controlled run instead uses the vendor runtime floor of
	 * 2 * VBAT + 100 mV. VOUT remains a diagnostic conversion reference, not
	 * the floor reference. Absolute VBUS, IBUS, temperature and hardware-fault
	 * limits remain unchanged. The first sample is the vendor 500-ms start
	 * check; longer endpoints continue fail-closed sampling at 500-ms intervals
	 * while REG07 reads refresh the watchdog.
	 */
	for (sample = 0; continuous || sample < sample_goal; sample++) {
		if (sample)
			msleep(SC8547_STABILITY_SAMPLE_MS);
		if (atomic_read(&sc->policy_cancel)) {
			ret = -ECANCELED;
			goto disable;
		}
		ret = sc8547_capture_confirmed_active_sample(sc, p, &active);
		p->runtime_samples = sample + 1;
		if (active.valid) {
			sc8547_publish_active_sample(p, &active);
			sc8547_update_runtime_range(p, &active);
		}
		if ((ret || sc8547_record_runtime_sample(sample, sample_goal,
							 continuous)) &&
		    p->runtime_recorded < SC8547_RUNTIME_RECORDS) {
			active.ordinal = sample + 1;
			p->runtime[p->runtime_recorded++] = active;
		}
		if (ret)
			goto reject;
	}
	goto disable;

reject:
	/* Match the downstream terminal dump before restore clears the cause. */
	sc8547_capture_reject_dump(sc, p);

disable:
	/* Always turn the pump off before returning to userspace. */
	disable_ret = sc8547_manual_disable(sc);
	if (disable_ret) {
		/* Retry through the broader fail-closed path if readback/write failed. */
		recovery_ret = sc8547_fail_closed(sc);
		if (!recovery_ret)
			usleep_range(10000, 20000);
	}
	if (!ret && disable_ret)
		ret = disable_ret;
	if (prep_touched) {
		p->restore_attempted = true;
		p->restore_result = sc8547_apply_experimental_init(sc);
		p->restore_check = sc->profile_check;
		if (!ret && p->restore_result)
			ret = p->restore_result;
	}

record:
	p->disable_result = disable_ret;
	final_ret = sc8547_read_status(sc, &reg06, &reg0e);
	if (!final_ret)
		final_ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg07);
	if (!final_ret) {
		p->final_reg06 = reg06;
		p->final_reg07 = reg07;
		p->final_snapshot_valid = true;
		if ((reg06 & SC8547_CP_SWITCHING_STAT) ||
		    (reg07 & SC8547_CHG_EN)) {
			if (!ret)
				ret = -EIO;
		}
	} else if (!ret) {
		ret = final_ret;
	}
	p->result = ret;
	p->valid = true;
	atomic_set(&sc->pulse_running, 0);

	return ret;
}

static int sc8547_bounded_pps_pulse(struct sc8547_device *sc,
				    unsigned int run_ms,
				    bool vendor_current_control)
{
	return sc8547_pps_run(sc, run_ms, vendor_current_control, false);
}

static int sc8547_continuous_pps_run(struct sc8547_device *sc)
{
	return sc8547_pps_run(sc, 0, true, true);
}

static ssize_t cp_pulse_500ms_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	mutex_lock(&sc->lock);
	ret = sc8547_bounded_pps_pulse(sc, SC8547_TEST_PULSE_MS, false);
	mutex_unlock(&sc->lock);
	if (sc->psy)
		power_supply_changed(sc->psy);

	return ret ? ret : count;
}
static DEVICE_ATTR_WO(cp_pulse_500ms);

static ssize_t cp_run_2s_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	mutex_lock(&sc->lock);
	ret = sc8547_bounded_pps_pulse(sc, SC8547_SHORT_RUN_MS, false);
	mutex_unlock(&sc->lock);
	if (sc->psy)
		power_supply_changed(sc->psy);

	return ret ? ret : count;
}
static DEVICE_ATTR_WO(cp_run_2s);

static ssize_t cp_run_10s_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	mutex_lock(&sc->lock);
	ret = sc8547_bounded_pps_pulse(sc, SC8547_STABILITY_RUN_MS, false);
	mutex_unlock(&sc->lock);
	if (sc->psy)
		power_supply_changed(sc->psy);

	return ret ? ret : count;
}
static DEVICE_ATTR_WO(cp_run_10s);

static ssize_t cp_run_60s_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	mutex_lock(&sc->lock);
	ret = sc8547_bounded_pps_pulse(sc, SC8547_LONG_RUN_MS, false);
	mutex_unlock(&sc->lock);
	if (sc->psy)
		power_supply_changed(sc->psy);

	return ret ? ret : count;
}
static DEVICE_ATTR_WO(cp_run_60s);

static ssize_t cp_run_5m_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	mutex_lock(&sc->lock);
	ret = sc8547_bounded_pps_pulse(sc, SC8547_SOAK_RUN_MS, false);
	mutex_unlock(&sc->lock);
	if (sc->psy)
		power_supply_changed(sc->psy);

	return ret ? ret : count;
}
static DEVICE_ATTR_WO(cp_run_5m);

static const char *
sc8547_pulse_result_cause(const struct sc8547_pulse_result *p)
{
	if (!p->result)
		return "none";
	if (p->result == -ECANCELED)
		return "cancelled";
	if (!p->reject_dump_valid)
		return "setup";
	if (p->reject_regs[SC8547_REG_FAULT_0F] &
	    SC8547_IBUS_UCP_FALL_FLAG)
		return "ibus-ucp-fall";
	if (p->reject_regs[SC8547_REG_FAULT_0F])
		return "hardware-protection";
	if (p->reject_regs[SC8547_REG_STATUS_06] & SC8547_SS_TIMEOUT_FLAG)
		return "soft-start-timeout";
	if (!(p->reject_regs[SC8547_REG_CHG_CTRL] & SC8547_CHG_EN))
		return "pump-stopped";
	if (p->result == -ERANGE)
		return "runtime-window";

	return "runtime";
}

static ssize_t pulse_result_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	struct sc8547_pulse_result p;
	unsigned int sample;
	size_t len = 0;

	if (!mutex_trylock(&sc->lock))
		return sysfs_emit(buf, "revision=%s running=%u busy=1\n",
				  SC8547_EXPERIMENT_REVISION,
				  !!atomic_read(&sc->pulse_running));
	p = sc->pulse;
	mutex_unlock(&sc->lock);

	if (!p.valid)
		return sysfs_emit(buf, "not_run\n");

	len += sysfs_emit_at(buf, len,
		"revision=%s sequence=%s role=%s device_id=%02x variant=%s output_source=%s run_ms=%u rc=%d cause=%s\n",
		SC8547_EXPERIMENT_REVISION, SC8547_EXPERIMENT_SEQUENCE,
		sc->role, sc->device_id, sc8547_variant_name(sc->variant),
		sc8547_output_reference_source(sc),
		p.requested_run_ms, p.result,
		sc8547_pulse_result_cause(&p));
	len += sysfs_emit_at(buf, len,
		"pre_vbus_uv=%d pre_vbat_uv=%d pre_output_reference_uv=%d target_vbus_uv=%d target_source=%s pre_wait_ms=%d\n",
		p.pre_vbus_uv, p.pre_vbat_uv, p.pre_output_reference_uv,
		p.target_vbus_uv,
		p.policy_target_valid ? "qcom-gauge" : "cp-local",
		p.adc_wait_ms);
	len += sysfs_emit_at(buf, len,
		"start_attempts=%u start_checks=%u start_check_ms=%u\n",
		p.start_attempts, p.start_checks,
		SC8547_VENDOR_START_CHECK_MS);
	len += sysfs_emit_at(buf, len,
		"run_samples=%u reported_samples=%u sample_ms=%u range_rechecks=%u range_recoveries=%u envelope=%s active_min_uv=%u undershoot_max_uv=%u ratio_max_uv=%u ibus_max_ua=%u ibus_adc_ceiling_ua=%u\n",
		p.runtime_samples, p.runtime_recorded,
		SC8547_STABILITY_SAMPLE_MS,
		p.runtime_range_rechecks, p.runtime_range_recoveries,
		p.vendor_current_control ? "vendor-current" : "fixed-ratio",
		sc->window.pulse_vbus_min_uv,
		p.vendor_current_control ?
			sc->window.pulse_ratio_error_max_uv : 0,
		p.vendor_current_control ? 0 :
			sc->window.pulse_ratio_error_max_uv,
		p.vendor_current_control ? SC8547_VENDOR_CURRENT_IBUS_MAX_UA :
			SC8547_TEST_IBUS_MAX_UA,
		p.vendor_current_control ?
			SC8547_VENDOR_CURRENT_IBUS_ADC_MAX_UA :
			SC8547_TEST_IBUS_MAX_UA);
	for (sample = 0; sample < p.runtime_recorded; sample++) {
		len += sysfs_emit_at(buf, len,
			"sample=%u elapsed_ms=%u valid=%u enabled=%u switching=%u vac_uv=%d vbus_uv=%d vout_uv=%d vbat_uv=%d output_reference_uv=%d headroom_uv=%d ratio_error_uv=%d ibus_ua=%d tdie_mc=%d\n",
			p.runtime[sample].ordinal,
			p.runtime[sample].ordinal * SC8547_STABILITY_SAMPLE_MS,
			p.runtime[sample].valid,
			!!(p.runtime[sample].reg07 & SC8547_CHG_EN),
			!!(p.runtime[sample].reg06 &
			   SC8547_CP_SWITCHING_STAT),
			p.runtime[sample].vac_uv,
			p.runtime[sample].vbus_uv,
			p.runtime[sample].vout_uv,
			p.runtime[sample].vbat_uv,
			p.runtime[sample].output_reference_uv,
			p.runtime[sample].headroom_uv,
			p.runtime[sample].ratio_error_uv,
			p.runtime[sample].ibus_ua,
			p.runtime[sample].tdie_mc);
	}
	if (p.runtime_observed)
		len += sysfs_emit_at(buf, len,
			"observed_vac_uv=%d..%d observed_vbus_uv=%d..%d observed_ratio_error_uv=%d..%d observed_ibus_ua=%d..%d observed_tdie_mc=%d..%d\n",
			p.runtime_vac_min_uv, p.runtime_vac_max_uv,
			p.runtime_vbus_min_uv, p.runtime_vbus_max_uv,
			p.runtime_ratio_min_uv, p.runtime_ratio_max_uv,
			p.runtime_ibus_min_ua, p.runtime_ibus_max_ua,
			p.runtime_tdie_min_mc, p.runtime_tdie_max_mc);
	len += sysfs_emit_at(buf, len,
		"disable_rc=%d restore_rc=%d restore_mismatch=%u final_valid=%u final_enabled=%u final_switching=%u\n",
		p.disable_result, p.restore_result, p.restore_check.mismatch,
		p.final_snapshot_valid,
		p.final_snapshot_valid &&
		!!(p.final_reg07 & SC8547_CHG_EN),
		p.final_snapshot_valid &&
		!!(p.final_reg06 & SC8547_CP_SWITCHING_STAT));
	if (p.result)
		len += sysfs_emit_at(buf, len,
			"details=sc8547_experimental/pulse_diagnostics\n");

	return len;
}
static DEVICE_ATTR_RO(pulse_result);

static ssize_t pulse_diagnostics_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	struct sc8547_pulse_result p;
	unsigned int reg;
	size_t len = 0;

	if (!mutex_trylock(&sc->lock))
		return sysfs_emit(buf, "revision=%s running=%u busy=1\n",
				  SC8547_EXPERIMENT_REVISION,
				  !!atomic_read(&sc->pulse_running));
	p = sc->pulse;
	mutex_unlock(&sc->lock);

	if (!p.valid)
		return sysfs_emit(buf, "not_run\n");

	len += sysfs_emit_at(buf, len,
		"revision=%s sequence=%s role=%s device_id=%02x variant=%s output_source=%s pulse_on_cmd=%02x pulse_off_cmd=%02x run_ms=%u rc=%d disable_rc=%d restore_attempted=%u restore_rc=%d\n",
		SC8547_EXPERIMENT_REVISION, SC8547_EXPERIMENT_SEQUENCE,
		sc->role, sc->device_id,
		sc8547_variant_name(sc->variant),
		sc8547_output_reference_source(sc),
		sc8547_charge_command(sc, true),
		sc8547_charge_command(sc, false),
		p.requested_run_ms, p.result, p.disable_result,
		p.restore_attempted, p.restore_result);
	len += sysfs_emit_at(buf, len,
		"restore_check_attempted=%u restore_check_rc=%d restore_actual_valid=%u restore_mismatch=%u restore_reg=%02x restore_mask=%02x restore_expected=%02x restore_actual=%02x\n",
		p.restore_check.attempted, p.restore_check.result,
		p.restore_check.actual_valid, p.restore_check.mismatch,
		p.restore_check.reg, p.restore_check.mask,
		p.restore_check.expected,
		p.restore_check.actual);
	len += sysfs_emit_at(buf, len,
		"rearm_attempted=%u rearm_valid=%u rearm_entry_reg09=%02x rearm_entry_reg11=%02x rearm_wd_timeout=%u rearm_final_reg09=%02x rearm_final_reg11=%02x\n",
		p.rearm_attempted, p.rearm_snapshot_valid,
		p.rearm_entry_reg09, p.rearm_entry_reg11,
		!!(p.rearm_entry_reg09 & SC8547_WD_TIMEOUT2_FLAG),
		p.rearm_final_reg09, p.rearm_final_reg11);
	len += sysfs_emit_at(buf, len,
		"pre_first_valid=%u pre_first_vbus_uv=%d pre_first_vbat_uv=%d pre_first_output_reference_uv=%d pre_wait_ms=%d\n",
		p.first_adc_snapshot_valid, p.first_vbus_uv,
		p.first_vbat_uv, p.first_output_reference_uv, p.adc_wait_ms);
	len += sysfs_emit_at(buf, len,
		"pre_valid=%u pre_vbus_uv=%d pre_vbat_uv=%d pre_output_reference_uv=%d target_vbus_uv=%d target_source=%s\n",
		p.preflight_snapshot_valid, p.pre_vbus_uv, p.pre_vbat_uv,
		p.pre_output_reference_uv, p.target_vbus_uv,
		p.policy_target_valid ? "qcom-gauge" : "cp-local");
	len += sysfs_emit_at(buf, len,
		"control_valid=%u pre_reg05=%02x pre_reg08=%02x pre_reg09=%02x pre_reg0c=%02x\n",
		p.control_snapshot_valid, p.pre_reg05, p.pre_reg08,
		p.pre_reg09, p.pre_reg0c);
	len += sysfs_emit_at(buf, len,
		"prep_valid=%u prep_mismatch=%08x prep_reg02=%02x prep_reg04=%02x prep_reg05=%02x prep_reg08=%02x prep_reg09=%02x prep_reg0c=%02x prep_reg0d=%02x prep_reg10=%02x prep_reg11=%02x prep_reg2b=%02x prep_reg30=%02x prep_reg3a=%02x prep_reg3b=%02x prep_reg3c=%02x\n",
		p.prep_snapshot_valid, p.prep_mismatch, p.prep_reg02,
		p.prep_reg04, p.prep_reg05, p.prep_reg08, p.prep_reg09,
		p.prep_reg0c, p.prep_reg0d, p.prep_reg10, p.prep_reg11,
		p.prep_reg2b, p.prep_reg30, p.prep_reg3a, p.prep_reg3b,
		p.prep_reg3c);
	len += sysfs_emit_at(buf, len,
		"ready_valid=%u ready_reg06=%02x ready_reg07=%02x ready_reg0e=%02x ready_reg11=%02x ready_reg12=%02x\n",
		p.ready_snapshot_valid, p.ready_reg06, p.ready_reg07,
		p.ready_reg0e, p.ready_reg11, p.ready_reg12);
	len += sysfs_emit_at(buf, len,
		"post_prepare_adc_valid=%u post_prepare_vbus_uv=%d post_prepare_vbat_uv=%d post_prepare_output_reference_uv=%d range_rechecks=%u range_recoveries=%u\n",
		p.post_prepare_adc_snapshot_valid, p.post_prepare_vbus_uv,
		p.post_prepare_vbat_uv, p.post_prepare_output_reference_uv,
		p.post_prepare_range_rechecks,
		p.post_prepare_range_recoveries);
	len += sysfs_emit_at(buf, len,
		"ready_path_valid=%u ready_vac_uv=%d ready_vout_uv=%d ready_reg33=%02x ready_reg3a=%02x ready_reg3b=%02x ready_reg3c=%02x vbus_range_disabled=%u\n",
		p.ready_extended_snapshot_valid, p.ready_vac_uv,
		p.ready_vout_uv, p.ready_reg33, p.ready_reg3a,
		p.ready_reg3b, p.ready_reg3c, !!(p.ready_reg3c & BIT(6)));
	len += sysfs_emit_at(buf, len,
		"enable_valid=%u enable_reg07=%02x early_valid=%u early_enabled=%u early_switching=%u early_reg06=%02x early_reg07=%02x early_reg0e=%02x\n",
		p.enable_snapshot_valid, p.enable_reg07,
		p.early_snapshot_valid,
		p.early_snapshot_valid && !!(p.early_reg07 & SC8547_CHG_EN),
		p.early_snapshot_valid &&
		!!(p.early_reg06 & SC8547_CP_SWITCHING_STAT),
		p.early_reg06, p.early_reg07, p.early_reg0e);
	len += sysfs_emit_at(buf, len,
		"start_attempts=%u start_checks=%u start_check_ms=%u start_trace=",
		p.start_attempts, p.start_checks,
		SC8547_VENDOR_START_CHECK_MS);
	for (reg = 0; reg < p.start_attempts; reg++)
		len += sysfs_emit_at(buf, len, "%s%u:%02x/%02x/%02x/%02x",
				     reg ? "," : "", reg + 1,
				     p.start_immediate_reg07[reg],
				     p.start_check_reg06[reg],
				     p.start_check_reg07[reg],
				     p.start_check_reg0e[reg]);
	len += sysfs_emit_at(buf, len, "\n");
	len += sysfs_emit_at(buf, len,
		"run_samples=%u reported_samples=%u sample_ms=%u range_rechecks=%u range_recoveries=%u envelope=%s active_vbus_min_uv=%u active_vac_min_uv=%u undershoot_max_uv=%u ratio_max_uv=%u ibus_max_ua=%u ibus_adc_ceiling_ua=%u\n",
		p.runtime_samples, p.runtime_recorded,
		SC8547_STABILITY_SAMPLE_MS,
		p.runtime_range_rechecks, p.runtime_range_recoveries,
		p.vendor_current_control ? "vendor-current" : "fixed-ratio",
		sc->window.pulse_vbus_min_uv,
		sc->window.pulse_vbus_min_uv,
		p.vendor_current_control ?
			sc->window.pulse_ratio_error_max_uv : 0,
		p.vendor_current_control ? 0 :
			sc->window.pulse_ratio_error_max_uv,
		p.vendor_current_control ? SC8547_VENDOR_CURRENT_IBUS_MAX_UA :
			SC8547_TEST_IBUS_MAX_UA,
		p.vendor_current_control ?
			SC8547_VENDOR_CURRENT_IBUS_ADC_MAX_UA :
			SC8547_TEST_IBUS_MAX_UA);
	for (reg = 0; reg < p.runtime_recorded; reg++) {
		len += sysfs_emit_at(buf, len,
			"run_sample=%u elapsed_ms=%u valid=%u reg06=%02x reg07=%02x reg0e=%02x vac_uv=%d vbus_uv=%d vout_uv=%d vbat_uv=%d output_reference_uv=%d headroom_uv=%d ratio_error_uv=%d ibus_ua=%d tdie_mc=%d reg33=%02x reg3a=%02x reg3b=%02x reg3c=%02x\n",
			p.runtime[reg].ordinal,
			p.runtime[reg].ordinal * SC8547_STABILITY_SAMPLE_MS,
			p.runtime[reg].valid, p.runtime[reg].reg06,
			p.runtime[reg].reg07, p.runtime[reg].reg0e,
			p.runtime[reg].vac_uv, p.runtime[reg].vbus_uv,
			p.runtime[reg].vout_uv, p.runtime[reg].vbat_uv,
			p.runtime[reg].output_reference_uv,
			p.runtime[reg].headroom_uv,
			p.runtime[reg].ratio_error_uv,
			p.runtime[reg].ibus_ua, p.runtime[reg].tdie_mc,
			p.runtime[reg].reg33, p.runtime[reg].reg3a,
			p.runtime[reg].reg3b, p.runtime[reg].reg3c);
	}
	len += sysfs_emit_at(buf, len, "reject_dump_valid=%u regs00_11=",
			     p.reject_dump_valid);
	for (reg = SC8547_DIAG_FIRST_REG; reg <= SC8547_DIAG_LAST_REG; reg++)
		len += sysfs_emit_at(buf, len, "%s%02x",
				     reg == SC8547_DIAG_FIRST_REG ? "" : ",",
				     p.reject_regs[reg]);
	len += sysfs_emit_at(buf, len, "\n");
	len += sysfs_emit_at(buf, len,
		"reject_path_valid=%u reject_vac_uv=%d reject_vbus_uv=%d reject_vout_uv=%d reject_vbat_uv=%d reject_reg33=%02x reject_reg3a=%02x reject_reg3b=%02x reject_reg3c=%02x\n",
		p.reject_extended_snapshot_valid, p.reject_vac_uv,
		p.reject_vbus_uv, p.reject_vout_uv, p.reject_vbat_uv,
		p.reject_reg33, p.reject_reg3a, p.reject_reg3b,
		p.reject_reg3c);
	len += sysfs_emit_at(buf, len,
		"on_valid=%u enabled=%u switching=%u reg06=%02x reg07=%02x reg0e=%02x vbus_uv=%d vbat_uv=%d output_reference_uv=%d headroom_uv=%d active_target_vbus_uv=%d target_error_uv=%d ibus_ua=%d tdie_mc=%d\n",
		p.on_snapshot_valid,
		p.on_snapshot_valid && !!(p.on_reg07 & SC8547_CHG_EN),
		p.on_snapshot_valid &&
		!!(p.on_reg06 & SC8547_CP_SWITCHING_STAT),
		p.on_reg06, p.on_reg07, p.on_reg0e, p.vbus_uv, p.vbat_uv,
		p.output_reference_uv, p.headroom_uv, p.active_target_vbus_uv,
		p.ratio_error_uv,
		p.ibus_ua, p.tdie_mc);
	len += sysfs_emit_at(buf, len,
		"on_path_valid=%u on_vac_uv=%d on_vout_uv=%d on_reg33=%02x on_reg3a=%02x on_reg3b=%02x on_reg3c=%02x\n",
		p.on_extended_snapshot_valid, p.on_vac_uv, p.on_vout_uv,
		p.on_reg33, p.on_reg3a, p.on_reg3b, p.on_reg3c);
	len += sysfs_emit_at(buf, len,
		"final_valid=%u final_enabled=%u final_switching=%u final_reg06=%02x final_reg07=%02x\n",
		p.final_snapshot_valid,
		p.final_snapshot_valid && !!(p.final_reg07 & SC8547_CHG_EN),
		p.final_snapshot_valid &&
		!!(p.final_reg06 & SC8547_CP_SWITCHING_STAT),
		p.final_reg06, p.final_reg07);

	return len;
}
static DEVICE_ATTR_RO(pulse_diagnostics);

static ssize_t cp_enable_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct sc8547_device *sc = dev_get_drvdata(dev);
	unsigned int reg;
	int ret;

	ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &reg);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%u\n", !!(reg & SC8547_CHG_EN));
}

int sc8547_manual_enable(struct i2c_client *client)
{
	struct sc8547_device *sc = sc8547_from_client(client);
	int ret;

	if (!sc)
		return -ENODEV;

	mutex_lock(&sc->lock);
	if (!sc->allow_experimental_cp_enable) {
		ret = -EPERM;
		goto out;
	}

	ret = sc8547_enable_preflight(sc);
	if (ret)
		goto out;
	ret = sc8547_set_charge_enabled(sc, true);
	if (ret)
		goto out;

	msleep(SC8547_POST_ENABLE_MS);
	ret = sc8547_post_enable_check(sc);
	if (ret)
		sc8547_set_charge_enabled(sc, false);
out:
	mutex_unlock(&sc->lock);
	if (sc->psy)
		power_supply_changed(sc->psy);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547_manual_enable);

int sc8547_manual_disable_client(struct i2c_client *client)
{
	struct sc8547_device *sc = sc8547_from_client(client);
	int ret;

	if (!sc)
		return -ENODEV;

	mutex_lock(&sc->lock);
	if (!sc->allow_experimental_cp_enable)
		ret = -EPERM;
	else
		ret = sc8547_manual_disable(sc);
	mutex_unlock(&sc->lock);
	if (sc->psy)
		power_supply_changed(sc->psy);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547_manual_disable_client);

static ssize_t cp_enable_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	if (enable)
		ret = sc8547_manual_enable(to_i2c_client(dev));
	else
		ret = sc8547_manual_disable_client(to_i2c_client(dev));
	if (ret)
		return ret;

	return count;
}
static DEVICE_ATTR_RW(cp_enable);

static struct attribute *sc8547_experimental_attrs[] = {
	&dev_attr_profile_raw.attr,
	&dev_attr_init_state.attr,
	&dev_attr_apply_init.attr,
	&dev_attr_watchdog_ms.attr,
	&dev_attr_enable_window.attr,
	&dev_attr_work_mode.attr,
	&dev_attr_cp_enable.attr,
	&dev_attr_cp_pulse_500ms.attr,
	&dev_attr_cp_run_2s.attr,
	&dev_attr_cp_run_10s.attr,
	&dev_attr_cp_run_60s.attr,
	&dev_attr_cp_run_5m.attr,
	&dev_attr_pulse_result.attr,
	&dev_attr_pulse_diagnostics.attr,
	NULL,
};

static umode_t sc8547_experimental_is_visible(struct kobject *kobj,
					      struct attribute *attr, int n)
{
	struct device *dev = kobj_to_dev(kobj);
	struct sc8547_device *sc = dev_get_drvdata(dev);

	if (attr == &dev_attr_enable_window.attr) {
		if (!sc || (!sc->allow_experimental_cp_enable &&
			    !sc->allow_experimental_cp_pulse))
			return 0;
	} else if (attr == &dev_attr_work_mode.attr ||
	    attr == &dev_attr_cp_enable.attr) {
		if (!sc || !sc->allow_experimental_cp_enable)
			return 0;
	} else if (attr == &dev_attr_cp_pulse_500ms.attr ||
		   attr == &dev_attr_pulse_result.attr ||
		   attr == &dev_attr_pulse_diagnostics.attr) {
		if (!sc || !sc->allow_experimental_cp_pulse)
			return 0;
	} else if (attr == &dev_attr_cp_run_2s.attr ||
		   attr == &dev_attr_cp_run_10s.attr ||
		   attr == &dev_attr_cp_run_60s.attr ||
		   attr == &dev_attr_cp_run_5m.attr) {
		if (!sc || !sc->allow_experimental_cp_pulse ||
		    strcmp(sc->role, "primary"))
			return 0;
	}

	return attr->mode;
}

static const struct attribute_group sc8547_experimental_attr_group = {
	.name = "sc8547_experimental",
	.attrs = sc8547_experimental_attrs,
	.is_visible = sc8547_experimental_is_visible,
};

static int sc8547_probe(struct i2c_client *client)
{
	struct power_supply_config psy_cfg = {};
	struct sc8547_device *sc;
	unsigned int id, mode, enabled, status;
	const char *role;
	const char *psy_override;
	char *psy_name;
	int ret;

	sc = devm_kzalloc(&client->dev, sizeof(*sc), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;

	sc->dev = &client->dev;
	sc->match_info = device_get_match_data(&client->dev);
	mutex_init(&sc->lock);
	atomic_set(&sc->policy_cancel, 0);
	atomic_set(&sc->pulse_running, 0);
	sc->regmap = devm_regmap_init_i2c(client, &sc8547_regmap_config);
	if (IS_ERR(sc->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(sc->regmap),
				     "failed to initialize regmap\n");

	i2c_set_clientdata(client, sc);

	ret = regmap_read(sc->regmap, SC8547_REG_DEVICE_ID, &id);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to read device ID\n");

	sc->device_id = id;
	sc->variant = sc8547_detect_variant(sc, sc->device_id);
	if (device_property_read_string(&client->dev, "southchip,role", &role)) {
		/*
		 * The existing giulia DTS marks the master pump with
		 * "southchip,master"; accept it as the primary role when no explicit
		 * "southchip,role" is given.  The slave node still needs an explicit
		 * "southchip,role = \"secondary\"".
		 */
		if (device_property_read_bool(&client->dev, "southchip,master"))
			role = "primary";
		else
			role = "standalone";
	}
	sc->role = role;

	if (sc->match_info && sc->match_info->variant != sc->variant &&
	    sc->variant != SC8547_VARIANT_UNKNOWN)
		dev_warn(&client->dev,
			 "DT compatible suggests %s but device ID 0x%02x identifies %s\n",
			 sc->match_info->name, sc->device_id,
			 sc8547_variant_name(sc->variant));

	sc8547_parse_experimental_profile(sc);
	sc8547_parse_enable_window(sc);

	/* ADC enable alone does not start charge pumping. */
	ret = sc8547_set_adc_enabled(sc, true);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to enable ADC\n");
	ret = sc8547_enable_required_adc_channels(sc);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to enable required ADC channels\n");

	/*
	 * power_supply may register a thermal zone using this name; thermal zone
	 * names are limited to THERMAL_NAME_LENGTH (20 bytes including NUL).
	 * Keep role in the dedicated sysfs attribute and use the bus/address here.
	 *
	 * "southchip,psy-name" lets a board (for example giulia, whose two SC8547D
	 * pumps share address 0x6a on different buses) pin a stable name that the
	 * qcom_battmgr OnePlus PPS patch can look up without depending on dynamic
	 * i2c adapter numbering.  Keep it short and unique.
	 */
	if (device_property_read_string(&client->dev, "southchip,psy-name",
					&psy_override) || !psy_override)
		psy_name = devm_kasprintf(&client->dev, GFP_KERNEL,
					   "sc8547-%d-%02x",
					   client->adapter->nr, client->addr);
	else
		psy_name = devm_kstrdup(&client->dev, psy_override, GFP_KERNEL);
	if (!psy_name)
		return -ENOMEM;

	sc->psy_desc.name = psy_name;
	sc->psy_desc.type = POWER_SUPPLY_TYPE_MAINS;
	sc->psy_desc.properties = sc8547_psy_props;
	sc->psy_desc.num_properties = ARRAY_SIZE(sc8547_psy_props);
	sc->psy_desc.get_property = sc8547_psy_get_property;
	sc->psy_desc.set_property = sc8547_psy_set_property;
	sc->psy_desc.property_is_writeable = sc8547_psy_property_is_writeable;
	psy_cfg.drv_data = sc;
	psy_cfg.fwnode = dev_fwnode(&client->dev);

	sc->psy = devm_power_supply_register(&client->dev, &sc->psy_desc, &psy_cfg);
	if (IS_ERR(sc->psy))
		return dev_err_probe(&client->dev, PTR_ERR(sc->psy),
				     "failed to register power supply\n");

	ret = devm_device_add_group(&client->dev, &sc8547_attr_group);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to create bring-up attributes\n");

	if (sc->allow_experimental_control) {
		ret = devm_device_add_group(&client->dev,
					    &sc8547_experimental_attr_group);
		if (ret)
			return dev_err_probe(&client->dev, ret,
					     "failed to create experimental attributes\n");
		dev_warn(&client->dev,
			 "development-only controls exposed (manual-enable=%u bounded-%s-pulse=%u)\n",
			 sc->allow_experimental_cp_enable,
			 sc->role,
			 sc->allow_experimental_cp_pulse);
	}

	ret = regmap_read(sc->regmap, SC8547_REG_CHG_CTRL, &enabled);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_MODE_CTRL, &mode);
	if (ret)
		return ret;
	ret = regmap_read(sc->regmap, SC8547_REG_STATUS_06, &status);
	if (ret)
		return ret;

	dev_info(&client->dev,
		 "%s ID=0x%02x role=%s CP=%s switching=%s mode=%s%s\n",
		 sc8547_variant_name(sc->variant), sc->device_id, sc->role,
		 enabled & SC8547_CHG_EN ? "on" : "off",
		 status & SC8547_CP_SWITCHING_STAT ? "yes" : "no",
		 mode & SC8547_CHARGE_MODE ? "bypass" : "2:1",
		 sc->allow_experimental_control ?
		 " experimental-control" : " telemetry-only");

	if (!sc8547_variant_control_supported(sc->variant))
		dev_warn(&client->dev,
			 "silicon variant is not enabled for experimental writes\n");

	return 0;
}

static void sc8547_shutdown(struct i2c_client *client)
{
	struct sc8547_device *sc = i2c_get_clientdata(client);

	if (!sc || !sc->allow_experimental_control)
		return;

	/* Best-effort fail closed; never leave development CP/WDT active. */
	atomic_set(&sc->policy_cancel, 1);
	sc8547_fail_closed(sc);
}

static const struct of_device_id sc8547_of_match[] = {
	{ .compatible = "southchip,sc8547", .data = &sc8547_info },
	{ .compatible = "southchip,sc8547a", .data = &sc8547a_info },
	{ .compatible = "southchip,sc8547d", .data = &sc8547d_info },
	/* Downstream aliases are accepted only to simplify bring-up. */
	{ .compatible = "oplus,sc8547a", .data = &sc8547a_info },
	{ .compatible = "oplus,sc8547d", .data = &sc8547d_info },
	{ .compatible = "slave_vphy_sc8547", .data = &sc8547_info },
	{ }
};
MODULE_DEVICE_TABLE(of, sc8547_of_match);

static const struct i2c_device_id sc8547_i2c_id[] = {
	{ "sc8547" },
	{ "sc8547a" },
	{ "sc8547d" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc8547_i2c_id);

static struct i2c_driver sc8547_driver = {
	.driver = {
		.name = "sc8547",
		.of_match_table = sc8547_of_match,
	},
	.probe = sc8547_probe,
	.shutdown = sc8547_shutdown,
	.id_table = sc8547_i2c_id,
};
module_i2c_driver(sc8547_driver);

MODULE_DESCRIPTION("Southchip SC8547/SC8547A/SC8547D charge-pump bring-up driver");
MODULE_LICENSE("GPL");
