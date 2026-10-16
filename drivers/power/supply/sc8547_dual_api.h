/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _SC8547_DUAL_BRINGUP_API_H
#define _SC8547_DUAL_BRINGUP_API_H

#include "sc8547_api.h"

struct platform_device;

struct sc8547_dual_state {
	struct sc8547_state primary;
	struct sc8547_state secondary;
	int aggregate_ibus_ua;
};

/*
 * Development-branch read-only API for layers above the virtual charge-pump
 * coordinator. This API never changes mode, enable state or source contract.
 *
 * The caller must hold a reference to @pdev for the duration of the call.
 * The coordinator validates that @pdev is bound to the sc8547-dual driver and
 * returns a coherent per-call pair snapshot using the physical-driver API.
 */
int sc8547_dual_get_state(struct platform_device *pdev,
			  struct sc8547_dual_state *state);

#endif /* _SC8547_DUAL_BRINGUP_API_H */
