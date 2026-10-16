// SPDX-License-Identifier: GPL-2.0-only
/*
 * Read-only source-contract diagnostics for the Caihong dual-SC8547 bring-up.
 *
 * Stage 6A correlates Qualcomm USB power-supply telemetry with the virtual
 * charge-pump state. It deliberately contains no source-contract or CP writes.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#include "sc8547_dual_api.h"

struct sc8547_policy_diag {
	struct device *dev;
	struct device_node *dual_np;
	const char *usb_psy_name;
};

struct sc8547_source_state {
	int online;
	int usb_type;
	int voltage_now_uv;
	int voltage_max_uv;
	int current_now_ua;
	int current_max_ua;
	int input_current_limit_ua;
};

static const char *sc8547_usb_type_name(int type)
{
	switch (type) {
	case POWER_SUPPLY_USB_TYPE_UNKNOWN:
		return "unknown";
	case POWER_SUPPLY_USB_TYPE_SDP:
		return "sdp";
	case POWER_SUPPLY_USB_TYPE_DCP:
		return "dcp";
	case POWER_SUPPLY_USB_TYPE_CDP:
		return "cdp";
	case POWER_SUPPLY_USB_TYPE_ACA:
		return "aca";
	case POWER_SUPPLY_USB_TYPE_C:
		return "type-c";
	case POWER_SUPPLY_USB_TYPE_PD:
		return "pd";
	case POWER_SUPPLY_USB_TYPE_PD_DRP:
		return "pd-drp";
	case POWER_SUPPLY_USB_TYPE_PD_PPS:
		return "pd-pps";
	case POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID:
		return "apple-brick-id";
	default:
		return "other";
	}
}

static int sc8547_policy_get_prop(struct power_supply *psy,
				  enum power_supply_property prop, int *value)
{
	union power_supply_propval val;
	int ret;

	ret = power_supply_get_property(psy, prop, &val);
	if (ret)
		return ret;

	*value = val.intval;
	return 0;
}

static int sc8547_policy_get_source(struct sc8547_policy_diag *diag,
				    struct power_supply **psy_out,
				    struct sc8547_source_state *state)
{
	struct power_supply *psy;
	int ret;

	psy = power_supply_get_by_name(diag->usb_psy_name);
	if (!psy)
		return -ENODEV;

	ret = sc8547_policy_get_prop(psy, POWER_SUPPLY_PROP_ONLINE,
				      &state->online);
	if (ret)
		goto err_put;
	ret = sc8547_policy_get_prop(psy, POWER_SUPPLY_PROP_USB_TYPE,
				      &state->usb_type);
	if (ret)
		goto err_put;
	ret = sc8547_policy_get_prop(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW,
				      &state->voltage_now_uv);
	if (ret)
		goto err_put;
	ret = sc8547_policy_get_prop(psy, POWER_SUPPLY_PROP_VOLTAGE_MAX,
				      &state->voltage_max_uv);
	if (ret)
		goto err_put;
	ret = sc8547_policy_get_prop(psy, POWER_SUPPLY_PROP_CURRENT_NOW,
				      &state->current_now_ua);
	if (ret)
		goto err_put;
	ret = sc8547_policy_get_prop(psy, POWER_SUPPLY_PROP_CURRENT_MAX,
				      &state->current_max_ua);
	if (ret)
		goto err_put;
	ret = sc8547_policy_get_prop(psy, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
				      &state->input_current_limit_ua);
	if (ret)
		goto err_put;

	*psy_out = psy;
	return 0;

err_put:
	power_supply_put(psy);
	return ret;
}

static int sc8547_policy_get_dual(struct sc8547_policy_diag *diag,
				  struct platform_device **dual_pdev,
				  struct sc8547_dual_state *state)
{
	struct platform_device *pdev;
	int ret;

	pdev = of_find_device_by_node(diag->dual_np);
	if (!pdev)
		return -ENODEV;

	if (!pdev->dev.driver || strcmp(pdev->dev.driver->name, "sc8547-dual")) {
		put_device(&pdev->dev);
		return -ENODEV;
	}

	ret = sc8547_dual_get_state(pdev, state);
	if (ret) {
		put_device(&pdev->dev);
		return ret;
	}

	*dual_pdev = pdev;
	return 0;
}

static ssize_t usb_supply_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct sc8547_policy_diag *diag = dev_get_drvdata(dev);
	struct power_supply *psy;

	psy = power_supply_get_by_name(diag->usb_psy_name);
	if (!psy)
		return sysfs_emit(buf, "name=%s available=0\n", diag->usb_psy_name);

	power_supply_put(psy);
	return sysfs_emit(buf, "name=%s available=1\n", diag->usb_psy_name);
}
static DEVICE_ATTR_RO(usb_supply);

static ssize_t source_state_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct sc8547_policy_diag *diag = dev_get_drvdata(dev);
	struct sc8547_source_state state = {};
	struct power_supply *psy;
	ssize_t len;
	int ret;

	ret = sc8547_policy_get_source(diag, &psy, &state);
	if (ret)
		return ret;

	len = sysfs_emit(buf,
		"online=%d usb_type=%d usb_type_name=%s pps_detected=%u voltage_now_uv=%d voltage_max_uv=%d current_now_ua=%d current_max_ua=%d input_current_limit_ua=%d\n",
		state.online, state.usb_type, sc8547_usb_type_name(state.usb_type),
		state.usb_type == POWER_SUPPLY_USB_TYPE_PD_PPS,
		state.voltage_now_uv, state.voltage_max_uv,
		state.current_now_ua, state.current_max_ua,
		state.input_current_limit_ua);

	power_supply_put(psy);
	return len;
}
static DEVICE_ATTR_RO(source_state);

static ssize_t combined_state_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct sc8547_policy_diag *diag = dev_get_drvdata(dev);
	struct sc8547_source_state source = {};
	struct sc8547_dual_state cp = {};
	struct platform_device *dual_pdev;
	struct power_supply *psy;
	size_t len = 0;
	int ret;

	ret = sc8547_policy_get_source(diag, &psy, &source);
	if (ret)
		return ret;

	ret = sc8547_policy_get_dual(diag, &dual_pdev, &cp);
	if (ret)
		goto out_put_psy;

	len += sysfs_emit_at(buf, len,
		"source supply=%s online=%d usb_type=%d usb_type_name=%s pps_detected=%u voltage_now_uv=%d voltage_max_uv=%d current_now_ua=%d current_max_ua=%d input_current_limit_ua=%d\n",
		diag->usb_psy_name, source.online, source.usb_type,
		sc8547_usb_type_name(source.usb_type),
		source.usb_type == POWER_SUPPLY_USB_TYPE_PD_PPS,
		source.voltage_now_uv, source.voltage_max_uv,
		source.current_now_ua, source.current_max_ua,
		source.input_current_limit_ua);
	len += sysfs_emit_at(buf, len,
		"primary initialized=%u stage4_authorized=%u enable=%u switching=%u mode=%s blocking_fault=%u vbus_uv=%d vbat_uv=%d ibus_ua=%d\n",
		cp.primary.initialized, cp.primary.stage4_authorized,
		cp.primary.enabled, cp.primary.switching,
		cp.primary.bypass ? "bypass" : "2:1",
		cp.primary.blocking_fault, cp.primary.vbus_uv,
		cp.primary.vbat_uv, cp.primary.ibus_ua);
	len += sysfs_emit_at(buf, len,
		"secondary initialized=%u stage4_authorized=%u enable=%u switching=%u mode=%s blocking_fault=%u vbus_uv=%d vbat_uv=%d ibus_ua=%d\n",
		cp.secondary.initialized, cp.secondary.stage4_authorized,
		cp.secondary.enabled, cp.secondary.switching,
		cp.secondary.bypass ? "bypass" : "2:1",
		cp.secondary.blocking_fault, cp.secondary.vbus_uv,
		cp.secondary.vbat_uv, cp.secondary.ibus_ua);
	len += sysfs_emit_at(buf, len, "aggregate_ibus_ua=%d\n",
			     cp.aggregate_ibus_ua);

	put_device(&dual_pdev->dev);
out_put_psy:
	power_supply_put(psy);
	return ret ? ret : (ssize_t)len;
}
static DEVICE_ATTR_RO(combined_state);

static struct attribute *sc8547_policy_attrs[] = {
	&dev_attr_usb_supply.attr,
	&dev_attr_source_state.attr,
	&dev_attr_combined_state.attr,
	NULL,
};

static const struct attribute_group sc8547_policy_attr_group = {
	.name = "sc8547_policy",
	.attrs = sc8547_policy_attrs,
};

static void sc8547_policy_put_node(void *data)
{
	of_node_put(data);
}

static int sc8547_policy_diag_probe(struct platform_device *pdev)
{
	struct sc8547_policy_diag *diag;
	int ret;

	diag = devm_kzalloc(&pdev->dev, sizeof(*diag), GFP_KERNEL);
	if (!diag)
		return -ENOMEM;

	diag->dev = &pdev->dev;
	diag->dual_np = of_parse_phandle(pdev->dev.of_node,
					"southchip,charge-pump", 0);
	if (!diag->dual_np)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "missing southchip,charge-pump phandle\n");

	ret = devm_add_action_or_reset(&pdev->dev, sc8547_policy_put_node,
				       diag->dual_np);
	if (ret)
		return ret;

	ret = device_property_read_string(&pdev->dev,
					  "southchip,usb-power-supply-name",
					  &diag->usb_psy_name);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "missing southchip,usb-power-supply-name\n");
	if (!diag->usb_psy_name[0])
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "USB power-supply name must not be empty\n");

	platform_set_drvdata(pdev, diag);

	ret = devm_device_add_group(&pdev->dev, &sc8547_policy_attr_group);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to create Stage-6A diagnostic group\n");

	dev_info(&pdev->dev,
		 "read-only source/SC8547 diagnostics ready for USB supply '%s'; no source or CP writes are implemented\n",
		 diag->usb_psy_name);
	return 0;
}

static const struct of_device_id sc8547_policy_diag_of_match[] = {
	{ .compatible = "southchip,sc8547-policy-diagnostic" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc8547_policy_diag_of_match);

static struct platform_driver sc8547_policy_diag_driver = {
	.driver = {
		.name = "sc8547-policy-diag",
		.of_match_table = sc8547_policy_diag_of_match,
	},
	.probe = sc8547_policy_diag_probe,
};
module_platform_driver(sc8547_policy_diag_driver);

MODULE_DESCRIPTION("Read-only SC8547/USB source-contract diagnostic layer");
MODULE_LICENSE("GPL");
