/*
 * linux/arch/arm/mach-w55fa93/include/mach/gpio.h
 *
 * Generic W55FA93 GPIO handling
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_W55FA93_GPIO_H
#define __ASM_ARCH_W55FA93_GPIO_H

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm-generic/gpio.h>

#define gpio_get_value	__gpio_get_value
#define gpio_set_value	__gpio_set_value
#define gpio_cansleep	__gpio_cansleep

static inline int gpio_to_irq(unsigned gpio)
{
	return IRQ_GPIO_START + gpio;
}

static inline int irq_to_gpio(unsigned irq)
{
	return irq - IRQ_GPIO_START;
}

#endif
