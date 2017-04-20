/*
 * linux/arch/arm/mach-w55fa93/pm.c
 *
 * Copyright (c) 2014 Nuvoton corporation.
 *
 * W55FA93 Power Management
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/module.h>

extern void w55fa93_pm_suspend(int nvt_mode);
static int w55fa93_pm_enter(suspend_state_t state)
{
	int ret = 0;

	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		w55fa93_pm_suspend(0);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static struct platform_suspend_ops w55fa93_pm_ops = {
	.enter		= w55fa93_pm_enter,
	.valid		= suspend_valid_only_mem,
};

static int __init w55fa93_pm_init(void)
{
	suspend_set_ops(&w55fa93_pm_ops);

	return 0;
}

late_initcall(w55fa93_pm_init);
