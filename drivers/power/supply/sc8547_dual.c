// SPDX-License-Identifier: GPL-2.0-only
/*
 * Dual-SC8547 coordinator for OnePlus Pad Pro (caihong) bring-up.
 *
 * Stage 5A exposes paired telemetry. Stage 5B optionally exposes explicit
 * laboratory dual-pump controls, but all physical safety checks/writes remain
 * owned by the SC8547 physical driver through sc8547_api.h.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#include "sc8547_api.h"
#include "sc8547_dual_api.h"

enum sc8547_dual_result {
	SC8547_DUAL_NEVER_RUN,
	SC8547_DUAL_MODE_SET,
	SC8547_DUAL_ENABLED,
	SC8547_DUAL_DISABLED,
	SC8547_DUAL_PAIR_UNAVAILABLE,
	SC8547_DUAL_NOT_READY,
	SC8547_DUAL_MODE_MISMATCH,
	SC8547_DUAL_PRIMARY_PREFLIGHT_FAILED,
	SC8547_DUAL_SECONDARY_PREFLIGHT_FAILED,
	SC8547_DUAL_PRIMARY_ENABLE_FAILED,
	SC8547_DUAL_SECONDARY_ENABLE_FAILED,
	SC8547_DUAL_FINAL_CHECK_FAILED,
	SC8547_DUAL_DISABLE_FAILED,
};

struct sc8547_dual {
	struct device *dev;
	struct device_node *primary_np;
	struct device_node *secondary_np;
	struct mutex lock;
	bool allow_dual_cp;
	enum sc8547_dual_result last_result;
	int last_errno;
};

static const char *sc8547_dual_result_name(enum sc8547_dual_result result)
{
	switch (result) {
	case SC8547_DUAL_MODE_SET:
		return "mode_set";
	case SC8547_DUAL_ENABLED:
		return "enabled";
	case SC8547_DUAL_DISABLED:
		return "disabled";
	case SC8547_DUAL_PAIR_UNAVAILABLE:
		return "pair_unavailable";
	case SC8547_DUAL_NOT_READY:
		return "not_ready";
	case SC8547_DUAL_MODE_MISMATCH:
		return "mode_mismatch";
	case SC8547_DUAL_PRIMARY_PREFLIGHT_FAILED:
		return "primary_preflight_failed";
	case SC8547_DUAL_SECONDARY_PREFLIGHT_FAILED:
		return "secondary_preflight_failed";
	case SC8547_DUAL_PRIMARY_ENABLE_FAILED:
		return "primary_enable_failed";
	case SC8547_DUAL_SECONDARY_ENABLE_FAILED:
		return "secondary_enable_failed";
	case SC8547_DUAL_FINAL_CHECK_FAILED:
		return "final_check_failed";
	case SC8547_DUAL_DISABLE_FAILED:
		return "disable_failed";
	default:
		return "never_run";
	}
}

static const char *sc8547_dual_variant_name(enum sc8547_api_variant variant)
{
	switch (variant) {
	case SC8547_API_VARIANT_SC8547:
		return "sc8547";
	case SC8547_API_VARIANT_SC8547A:
		return "sc8547a";
	case SC8547_API_VARIANT_SC8547D:
		return "sc8547d";
	default:
		return "unknown";
	}
}

static bool sc8547_dual_client_bound(struct i2c_client *client)
{
	return client->dev.driver &&
	       !strcmp(client->dev.driver->name, "sc8547");
}

static bool sc8547_dual_role_matches(struct i2c_client *client,
				     const char *expected)
{
	const char *role;

	if (device_property_read_string(&client->dev, "southchip,role", &role))
		return false;

	return !strcmp(role, expected);
}

static int sc8547_dual_get_clients(struct sc8547_dual *dual,
				   struct i2c_client **primary,
				   struct i2c_client **secondary)
{
	struct i2c_client *p, *s;

	p = of_find_i2c_device_by_node(dual->primary_np);
	if (!p)
		return -ENODEV;

	s = of_find_i2c_device_by_node(dual->secondary_np);
	if (!s) {
		put_device(&p->dev);
		return -ENODEV;
	}

	if (p == s || !sc8547_dual_client_bound(p) ||
	    !sc8547_dual_client_bound(s) ||
	    !sc8547_dual_role_matches(p, "primary") ||
	    !sc8547_dual_role_matches(s, "secondary")) {
		put_device(&s->dev);
		put_device(&p->dev);
		return -ENODEV;
	}

	*primary = p;
	*secondary = s;
	return 0;
}

static void sc8547_dual_put_clients(struct i2c_client *primary,
				    struct i2c_client *secondary)
{
	put_device(&secondary->dev);
	put_device(&primary->dev);
}

static int sc8547_dual_get_states(struct sc8547_dual *dual,
				  struct i2c_client **primary,
				  struct i2c_client **secondary,
				  struct sc8547_state *pstate,
				  struct sc8547_state *sstate)
{
	int ret;

	ret = sc8547_dual_get_clients(dual, primary, secondary);
	if (ret)
		return ret;

	ret = sc8547_get_state(*primary, pstate);
	if (ret)
		goto err_put;
	ret = sc8547_get_state(*secondary, sstate);
	if (ret)
		goto err_put;

	return 0;

err_put:
	sc8547_dual_put_clients(*primary, *secondary);
	return ret;
}

static struct sc8547_dual *
sc8547_dual_from_pdev(struct platform_device *pdev)
{
	if (!pdev || !pdev->dev.driver ||
	    strcmp(pdev->dev.driver->name, "sc8547-dual"))
		return NULL;

	return platform_get_drvdata(pdev);
}

int sc8547_dual_get_state(struct platform_device *pdev,
			  struct sc8547_dual_state *state)
{
	struct sc8547_dual *dual = sc8547_dual_from_pdev(pdev);
	struct i2c_client *primary, *secondary;
	int ret;

	if (!dual || !state)
		return -ENODEV;

	mutex_lock(&dual->lock);
	ret = sc8547_dual_get_states(dual, &primary, &secondary,
				     &state->primary, &state->secondary);
	if (ret)
		goto out;

	state->aggregate_ibus_ua = state->primary.ibus_ua +
				   state->secondary.ibus_ua;
	sc8547_dual_put_clients(primary, secondary);
out:
	mutex_unlock(&dual->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547_dual_get_state);

static bool sc8547_dual_state_ready(const struct sc8547_state *state)
{
	return state->initialized && state->stage4_authorized &&
	       !state->enabled && !state->switching &&
	       !state->blocking_fault;
}

static int sc8547_dual_stop_clients(struct i2c_client *primary,
				    struct i2c_client *secondary)
{
	int first = 0;
	int ret;

	ret = sc8547_manual_disable_client(secondary);
	if (ret)
		first = ret;

	ret = sc8547_manual_disable_client(primary);
	if (ret && !first)
		first = ret;

	return first;
}

static int sc8547_dual_start_clients(struct sc8547_dual *dual,
				     struct i2c_client *primary,
				     struct i2c_client *secondary)
{
	struct sc8547_state p = {}, s = {};
	int ret;

	ret = sc8547_get_state(primary, &p);
	if (ret)
		return ret;
	ret = sc8547_get_state(secondary, &s);
	if (ret)
		return ret;

	if (!sc8547_dual_state_ready(&p) || !sc8547_dual_state_ready(&s)) {
		dual->last_result = SC8547_DUAL_NOT_READY;
		return -EPERM;
	}
	if (p.bypass != s.bypass) {
		dual->last_result = SC8547_DUAL_MODE_MISMATCH;
		return -EINVAL;
	}

	ret = sc8547_manual_preflight(primary);
	if (ret) {
		dual->last_result = SC8547_DUAL_PRIMARY_PREFLIGHT_FAILED;
		return ret;
	}
	ret = sc8547_manual_preflight(secondary);
	if (ret) {
		dual->last_result = SC8547_DUAL_SECONDARY_PREFLIGHT_FAILED;
		return ret;
	}

	ret = sc8547_manual_enable(primary);
	if (ret) {
		dual->last_result = SC8547_DUAL_PRIMARY_ENABLE_FAILED;
		sc8547_manual_disable_client(primary);
		return ret;
	}

	ret = sc8547_manual_enable(secondary);
	if (ret) {
		dual->last_result = SC8547_DUAL_SECONDARY_ENABLE_FAILED;
		sc8547_manual_disable_client(secondary);
		sc8547_manual_disable_client(primary);
		return ret;
	}

	ret = sc8547_get_state(primary, &p);
	if (ret)
		goto final_fail;
	ret = sc8547_get_state(secondary, &s);
	if (ret)
		goto final_fail;
	if (!p.enabled || !p.switching || p.blocking_fault ||
	    !s.enabled || !s.switching || s.blocking_fault) {
		ret = -EIO;
		goto final_fail;
	}

	dual->last_result = SC8547_DUAL_ENABLED;
	return 0;

final_fail:
	dual->last_result = SC8547_DUAL_FINAL_CHECK_FAILED;
	sc8547_dual_stop_clients(primary, secondary);
	return ret;
}

static ssize_t peer_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct sc8547_dual *dual = dev_get_drvdata(dev);
	struct i2c_client *primary, *secondary;
	struct sc8547_state p = {}, s = {};
	ssize_t len;
	int ret;

	ret = sc8547_dual_get_states(dual, &primary, &secondary, &p, &s);
	if (ret)
		return sysfs_emit(buf, "unavailable\n");

	len = sysfs_emit(buf,
		"primary=%s role=primary id=0x%02x variant=%s secondary=%s role=secondary id=0x%02x variant=%s\n",
		dev_name(&primary->dev), p.device_id,
		sc8547_dual_variant_name(p.variant),
		dev_name(&secondary->dev), s.device_id,
		sc8547_dual_variant_name(s.variant));
	sc8547_dual_put_clients(primary, secondary);
	return len;
}
static DEVICE_ATTR_RO(peer);

static ssize_t pair_state_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct sc8547_dual *dual = dev_get_drvdata(dev);
	struct i2c_client *primary, *secondary;
	struct sc8547_state p = {}, s = {};
	size_t len = 0;
	int ret;

	ret = sc8547_dual_get_states(dual, &primary, &secondary, &p, &s);
	if (ret)
		return ret;

	len += sysfs_emit_at(buf, len,
		"primary dev=%s id=0x%02x variant=%s initialized=%u stage4_authorized=%u enable=%u switching=%u mode=%s blocking_fault=%u vbus_uv=%d vbat_uv=%d ibus_ua=%d\n",
		dev_name(&primary->dev), p.device_id,
		sc8547_dual_variant_name(p.variant), p.initialized,
		p.stage4_authorized, p.enabled, p.switching,
		p.bypass ? "bypass" : "2:1", p.blocking_fault,
		p.vbus_uv, p.vbat_uv, p.ibus_ua);
	len += sysfs_emit_at(buf, len,
		"secondary dev=%s id=0x%02x variant=%s initialized=%u stage4_authorized=%u enable=%u switching=%u mode=%s blocking_fault=%u vbus_uv=%d vbat_uv=%d ibus_ua=%d\n",
		dev_name(&secondary->dev), s.device_id,
		sc8547_dual_variant_name(s.variant), s.initialized,
		s.stage4_authorized, s.enabled, s.switching,
		s.bypass ? "bypass" : "2:1", s.blocking_fault,
		s.vbus_uv, s.vbat_uv, s.ibus_ua);
	len += sysfs_emit_at(buf, len, "aggregate_ibus_ua=%d\n",
			     p.ibus_ua + s.ibus_ua);

	sc8547_dual_put_clients(primary, secondary);
	return len;
}
static DEVICE_ATTR_RO(pair_state);

static ssize_t aggregate_ibus_ua_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct sc8547_dual *dual = dev_get_drvdata(dev);
	struct i2c_client *primary, *secondary;
	struct sc8547_state p = {}, s = {};
	int ret;

	ret = sc8547_dual_get_states(dual, &primary, &secondary, &p, &s);
	if (ret)
		return ret;

	sc8547_dual_put_clients(primary, secondary);
	return sysfs_emit(buf, "%d\n", p.ibus_ua + s.ibus_ua);
}
static DEVICE_ATTR_RO(aggregate_ibus_ua);

static ssize_t last_result_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sc8547_dual *dual = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&dual->lock);
	len = sysfs_emit(buf, "%s errno=%d\n",
			 sc8547_dual_result_name(dual->last_result),
			 dual->last_errno);
	mutex_unlock(&dual->lock);
	return len;
}
static DEVICE_ATTR_RO(last_result);

static ssize_t work_mode_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct sc8547_dual *dual = dev_get_drvdata(dev);
	struct i2c_client *primary, *secondary;
	struct sc8547_state p = {}, s = {};
	ssize_t len;
	int ret;

	ret = sc8547_dual_get_states(dual, &primary, &secondary, &p, &s);
	if (ret)
		return ret;

	if (p.bypass == s.bypass)
		len = sysfs_emit(buf, "%s\n", p.bypass ? "bypass" : "2:1");
	else
		len = sysfs_emit(buf, "mismatch primary=%s secondary=%s\n",
				 p.bypass ? "bypass" : "2:1",
				 s.bypass ? "bypass" : "2:1");

	sc8547_dual_put_clients(primary, secondary);
	return len;
}

static ssize_t work_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct sc8547_dual *dual = dev_get_drvdata(dev);
	struct i2c_client *primary, *secondary;
	struct sc8547_state p = {}, s = {};
	bool bypass;
	int ret;

	if (sysfs_streq(buf, "2:1"))
		bypass = false;
	else if (sysfs_streq(buf, "bypass"))
		bypass = true;
	else
		return -EINVAL;

	mutex_lock(&dual->lock);
	ret = sc8547_dual_get_states(dual, &primary, &secondary, &p, &s);
	if (ret) {
		dual->last_result = SC8547_DUAL_PAIR_UNAVAILABLE;
		goto out_result;
	}

	if (!sc8547_dual_state_ready(&p) || !sc8547_dual_state_ready(&s)) {
		ret = -EPERM;
		dual->last_result = SC8547_DUAL_NOT_READY;
		goto out_put;
	}

	ret = sc8547_set_manual_mode(primary, bypass);
	if (ret) {
		dual->last_result = SC8547_DUAL_NOT_READY;
		goto out_put;
	}
	ret = sc8547_set_manual_mode(secondary, bypass);
	if (ret) {
		/* No pump has been enabled; restore the primary's previous mode. */
		sc8547_set_manual_mode(primary, p.bypass);
		dual->last_result = SC8547_DUAL_NOT_READY;
		goto out_put;
	}

	dual->last_result = SC8547_DUAL_MODE_SET;
	ret = 0;
out_put:
	sc8547_dual_put_clients(primary, secondary);
out_result:
	dual->last_errno = ret;
	mutex_unlock(&dual->lock);
	if (ret)
		return ret;

	return count;
}
static DEVICE_ATTR_RW(work_mode);

static ssize_t dual_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sc8547_dual *dual = dev_get_drvdata(dev);
	struct i2c_client *primary, *secondary;
	struct sc8547_state p = {}, s = {};
	int ret;

	ret = sc8547_dual_get_states(dual, &primary, &secondary, &p, &s);
	if (ret)
		return ret;

	sc8547_dual_put_clients(primary, secondary);
	return sysfs_emit(buf, "%u\n",
			  p.enabled && p.switching && s.enabled && s.switching);
}

static ssize_t dual_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct sc8547_dual *dual = dev_get_drvdata(dev);
	struct i2c_client *primary, *secondary;
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	mutex_lock(&dual->lock);
	ret = sc8547_dual_get_clients(dual, &primary, &secondary);
	if (ret) {
		dual->last_result = SC8547_DUAL_PAIR_UNAVAILABLE;
		goto out_result;
	}

	if (enable) {
		ret = sc8547_dual_start_clients(dual, primary, secondary);
	} else {
		ret = sc8547_dual_stop_clients(primary, secondary);
		dual->last_result = ret ? SC8547_DUAL_DISABLE_FAILED :
					  SC8547_DUAL_DISABLED;
	}

	sc8547_dual_put_clients(primary, secondary);
out_result:
	dual->last_errno = ret;
	mutex_unlock(&dual->lock);
	if (ret)
		return ret;

	return count;
}
static DEVICE_ATTR_RW(dual_enable);

static struct attribute *sc8547_dual_attrs[] = {
	&dev_attr_peer.attr,
	&dev_attr_pair_state.attr,
	&dev_attr_aggregate_ibus_ua.attr,
	&dev_attr_last_result.attr,
	&dev_attr_work_mode.attr,
	&dev_attr_dual_enable.attr,
	NULL,
};

static umode_t sc8547_dual_is_visible(struct kobject *kobj,
				      struct attribute *attr, int n)
{
	struct device *dev = kobj_to_dev(kobj);
	struct sc8547_dual *dual = dev_get_drvdata(dev);

	if (attr == &dev_attr_work_mode.attr ||
	    attr == &dev_attr_dual_enable.attr) {
		if (!dual || !dual->allow_dual_cp)
			return 0;
	}

	return attr->mode;
}

static const struct attribute_group sc8547_dual_attr_group = {
	.name = "sc8547_dual",
	.attrs = sc8547_dual_attrs,
	.is_visible = sc8547_dual_is_visible,
};

static void sc8547_dual_put_node(void *data)
{
	of_node_put(data);
}

static void sc8547_dual_fail_closed(struct sc8547_dual *dual)
{
	struct i2c_client *primary, *secondary;

	if (!dual->allow_dual_cp)
		return;

	mutex_lock(&dual->lock);
	if (!sc8547_dual_get_clients(dual, &primary, &secondary)) {
		sc8547_dual_stop_clients(primary, secondary);
		sc8547_dual_put_clients(primary, secondary);
	}
	mutex_unlock(&dual->lock);
}

static int sc8547_dual_probe(struct platform_device *pdev)
{
	struct sc8547_dual *dual;
	int ret;

	dual = devm_kzalloc(&pdev->dev, sizeof(*dual), GFP_KERNEL);
	if (!dual)
		return -ENOMEM;

	dual->dev = &pdev->dev;
	mutex_init(&dual->lock);
	dual->last_result = SC8547_DUAL_NEVER_RUN;
	dual->primary_np = of_parse_phandle(pdev->dev.of_node,
					   "southchip,primary", 0);
	if (!dual->primary_np)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "missing southchip,primary phandle\n");
	ret = devm_add_action_or_reset(&pdev->dev, sc8547_dual_put_node,
				       dual->primary_np);
	if (ret)
		return ret;

	dual->secondary_np = of_parse_phandle(pdev->dev.of_node,
					     "southchip,secondary", 0);
	if (!dual->secondary_np)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "missing southchip,secondary phandle\n");
	ret = devm_add_action_or_reset(&pdev->dev, sc8547_dual_put_node,
				       dual->secondary_np);
	if (ret)
		return ret;

	if (dual->primary_np == dual->secondary_np)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "primary and secondary phandles must differ\n");

	dual->allow_dual_cp = device_property_read_bool(&pdev->dev,
						 "southchip,allow-experimental-dual-cp");
	platform_set_drvdata(pdev, dual);

	ret = devm_device_add_group(&pdev->dev, &sc8547_dual_attr_group);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to create dual-pump group\n");

	if (dual->allow_dual_cp)
		dev_warn(&pdev->dev,
			 "development-only dual-pump controls exposed; no charger policy/source negotiation is present\n");
	else
		dev_info(&pdev->dev,
			 "read-only SC8547 dual-pump coordinator ready\n");

	return 0;
}

static void sc8547_dual_remove(struct platform_device *pdev)
{
	sc8547_dual_fail_closed(platform_get_drvdata(pdev));
}

static void sc8547_dual_shutdown(struct platform_device *pdev)
{
	sc8547_dual_fail_closed(platform_get_drvdata(pdev));
}

static const struct of_device_id sc8547_dual_of_match[] = {
	{ .compatible = "southchip,sc8547-dual-experimental" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc8547_dual_of_match);

static struct platform_driver sc8547_dual_driver = {
	.driver = {
		.name = "sc8547-dual",
		.of_match_table = sc8547_dual_of_match,
	},
	.probe = sc8547_dual_probe,
	.remove = sc8547_dual_remove,
	.shutdown = sc8547_dual_shutdown,
};
module_platform_driver(sc8547_dual_driver);

MODULE_DESCRIPTION("Dual SC8547 charge-pump bring-up coordinator");
MODULE_LICENSE("GPL");
