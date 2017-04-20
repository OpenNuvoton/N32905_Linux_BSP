/*
 * linux/arch/arm/mach-w55fa93/irq.c
 *
 * based on linux/arch/arm/plat-s3c24xx/irq.c by Ben Dooks
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/ptrace.h>
#include <linux/sysdev.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/w55fa93_reg.h>

static void w55fa93_irq_mask(unsigned int irq)
{
	__raw_writel(1 << irq, REG_AIC_MDCR);
}

static void w55fa93_irq_ack(unsigned int irq)
{
	__raw_writel(0x01, REG_AIC_EOSCR);
}

static void w55fa93_irq_unmask(unsigned int irq)
{
	__raw_writel(1 << irq, REG_AIC_MECR);
}

/*
 * w55fa93 startup function
 */
static unsigned int w55fa93_irq_startup(unsigned int irq)
{
	w55fa93_irq_unmask(irq);
	return 0;
}

/*
 * w55fa93 shutdown function
 */
static void w55fa93_irq_shutdown(unsigned int irq)
{
	w55fa93_irq_mask(irq);
}

static struct irq_chip w55fa93_irq_chip = {
	.ack		= w55fa93_irq_ack,
	.mask		= w55fa93_irq_mask,
	.unmask		= w55fa93_irq_unmask,
	.startup	= w55fa93_irq_startup,
	.shutdown	= w55fa93_irq_shutdown,
};

void __init w55fa93_init_irq(void)
{
	int irqno;

	__raw_writel(0xFFFFFFFF, REG_AIC_MDCR);

	for (irqno = IRQ_WDT; irqno < NR_IRQS; irqno++) {
		set_irq_chip(irqno, &w55fa93_irq_chip);
		set_irq_handler(irqno, handle_level_irq);
		set_irq_flags(irqno, IRQF_VALID);
	}
}
