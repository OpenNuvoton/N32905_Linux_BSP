/*
 * linux/arch/arm/mach-w55fa93/clock.c
 *
 * Copyright (c) 2013 Nuvoton technology corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/w55fa93_reg.h>
#include "clock.h"

static DEFINE_SPINLOCK(clocks_lock);
extern unsigned int w55fa93_external_clock;
extern unsigned int w55fa93_system_clock;
extern unsigned int w55fa93_cpu_clock;
extern unsigned int w55fa93_ahb_clock;
extern unsigned int w55fa93_apb_clock;

int clk_enable(struct clk *clk)
{
	unsigned long flags;

	spin_lock_irqsave(&clocks_lock, flags);
	if (clk->enabled++ == 0)
		(clk->enable)(clk, 1);
	spin_unlock_irqrestore(&clocks_lock, flags);

	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	unsigned long flags;

	WARN_ON(clk->enabled == 0);

	spin_lock_irqsave(&clocks_lock, flags);
	if (--clk->enabled == 0)
		(clk->enable)(clk, 0);
	spin_unlock_irqrestore(&clocks_lock, flags);
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	return w55fa93_external_clock;
}
EXPORT_SYMBOL(clk_get_rate);

unsigned long get_system_clk(void)
{
	return w55fa93_system_clock;
}
EXPORT_SYMBOL(get_system_clk);

unsigned long get_cpu_clk(void)
{
	return w55fa93_cpu_clock;
}
EXPORT_SYMBOL(get_cpu_clk);

unsigned long get_ahb_clk(void)
{
	return w55fa93_ahb_clock;
}
EXPORT_SYMBOL(get_ahb_clk);

unsigned long get_apb_clk(void)
{
	return w55fa93_apb_clock;
}
EXPORT_SYMBOL(get_apb_clk);

void w55fa93_ahbclk_enable(struct clk *clk, int enable)
{
	unsigned int clocks = clk->cken;
	unsigned long clken;

	clken = __raw_readl(REG_AHBCLK);
	//printk("\tREG_AHBCLK=0x%x, clocks=0x%x\n", readl(REG_AHBCLK), clocks);

	if (enable)
		clken |= clocks;
	else {
		// ignore HCLK3 and HCLK4 clock disable
		if ((clocks != HCLK3_CKE) && (clocks != HCLK4_CKE))
			clken &= ~clocks;
	}

	__raw_writel(clken, REG_AHBCLK);
	//printk("\tREG_AHBCLK=0x%x\n", readl(REG_AHBCLK));
}

void w55fa93_apbclk_enable(struct clk *clk, int enable)
{
	unsigned int clocks = clk->cken;
	unsigned long clken;

	clken = __raw_readl(REG_APBCLK);
	//printk("\tREG_APBCLK=0x%x, clocks=0x%x\n", readl(REG_APBCLK), clocks);

	if (enable)
		clken |= clocks;
	else
		clken &= ~clocks;

	__raw_writel(clken, REG_APBCLK);
	//printk("\tREG_APBCLK=0x%x\n", readl(REG_APBCLK));
}

