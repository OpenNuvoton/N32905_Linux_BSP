/*
 * linux/arch/arm/mach-w55fa93/time.c
 *
 * Based on linux/arch/arm/plat-s3c24xx/time.c by Ben Dooks
 *
 * Copyright (c) 2009 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>

#include <asm/mach-types.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>

#include <mach/w55fa93_reg.h>

#define TMR_PERIOD		(0x01 << 27)
#define TMR_ONESHOT		(0x00 << 27)
#define TMR_COUNTEN		(0x01 << 30)
#define TMR_INTEN		(0x01 << 29)
#define TMR_TDREN		(0x01 << 16)

#define	TDR_SHIFT	32
#define	TDR_MASK	((1 << TDR_SHIFT) - 1)

static void w55fa93_clockevent_setmode(enum clock_event_mode mode,
		struct clock_event_device *clk)
{
	unsigned int val;

	val = __raw_readl(REG_TCSR0);
	val &= ~(0x03 << 27);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		__raw_writel(CLOCK_TICK_RATE/HZ, REG_TICR0);
		val |= (TMR_PERIOD | TMR_COUNTEN | TMR_INTEN);
		break;

	case CLOCK_EVT_MODE_ONESHOT:
		val |= (TMR_ONESHOT | TMR_COUNTEN | TMR_INTEN);
		break;

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_RESUME:
		break;
	}

	__raw_writel(val, REG_TCSR0);
}

static int w55fa93_clockevent_setnextevent(unsigned long evt,
		struct clock_event_device *clk)
{
	unsigned int val;
	unsigned int tcsr, tdelta;

	tcsr = __raw_readl(REG_TCSR0);
	tdelta = __raw_readl(REG_TICR0) - __raw_readl(REG_TDR0);

	__raw_writel(evt, REG_TICR0);

	// 170214, to fix HRT issue
	if (!(tcsr & TMR_COUNTEN) && ((tdelta > 2) || (tdelta == 0))) {
		val = __raw_readl(REG_TCSR0);
		val |= (TMR_COUNTEN | TMR_INTEN);
		__raw_writel(val, REG_TCSR0);
	}

	return 0;
}

static struct clock_event_device w55fa93_clockevent_device = {
	.name		= "w55fa93-timer0",
	.shift		= 32,
	.features	= CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode	= w55fa93_clockevent_setmode,
	.set_next_event	= w55fa93_clockevent_setnextevent,
	.rating		= 300,
};

/*IRQ handler for the timer*/

static irqreturn_t w55fa93_timer0_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &w55fa93_clockevent_device;

	__raw_writel(0x01, REG_TISR); /* clear TIF0 */

	evt->event_handler(evt);
	return IRQ_HANDLED;
}

static struct irqaction w55fa93_timer0_irq = {
	.name		= "w55fa93-timer0",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= w55fa93_timer0_interrupt,
};

static void __init w55fa93_clockevents_init(void)
{
	struct clk *clk = clk_get(NULL, "timer0");

	BUG_ON(IS_ERR(clk));
	
	clk_enable(clk);

	__raw_writel(0x00, REG_TCSR0);
	__raw_writel(0x01, REG_TISR);

	setup_irq(IRQ_TIMER0, &w55fa93_timer0_irq);

	w55fa93_clockevent_device.mult = div_sc(CLOCK_TICK_RATE, NSEC_PER_SEC,
					w55fa93_clockevent_device.shift);
	w55fa93_clockevent_device.max_delta_ns = clockevent_delta2ns(0xffffffff,
					&w55fa93_clockevent_device);
	w55fa93_clockevent_device.min_delta_ns = clockevent_delta2ns(0xf,
					&w55fa93_clockevent_device);
	w55fa93_clockevent_device.cpumask = cpumask_of(0);

	clockevents_register_device(&w55fa93_clockevent_device);
}

static cycle_t w55fa93_get_cycles(struct clocksource *cs)
{
	//return (__raw_readl(REG_TDR1) & TDR_MASK);
	return (__raw_readl(REG_TDR1));
}

static struct clocksource clocksource_w55fa93 = {
	.name	= "w55fa93-timer1",
	.rating	= 200,
	.read	= w55fa93_get_cycles,
	.mask	= CLOCKSOURCE_MASK(TDR_SHIFT),
	.shift	= 10,
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void __init w55fa93_clocksource_init(void)
{
	struct clk *clk = clk_get(NULL, "timer1");

	BUG_ON(IS_ERR(clk));

	clk_enable(clk);

	__raw_writel(0x00, REG_TCSR1);
	__raw_writel(0x02, REG_TISR);
	__raw_writel(0xffffffff, REG_TICR1);
	__raw_writel(__raw_readl(REG_TCSR1) | TMR_COUNTEN | TMR_PERIOD | TMR_TDREN, REG_TCSR1);

	clocksource_w55fa93.mult =
		clocksource_khz2mult(CLOCK_TICK_RATE/1000, clocksource_w55fa93.shift);
	clocksource_register(&clocksource_w55fa93);
}

static void __init w55fa93_timer_init(void)
{
	w55fa93_clockevents_init();
	w55fa93_clocksource_init();
}

struct sys_timer w55fa93_timer = {
	.init		= w55fa93_timer_init,
};
