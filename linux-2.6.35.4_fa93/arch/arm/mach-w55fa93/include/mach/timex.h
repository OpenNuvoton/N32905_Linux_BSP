/*
 * arch/arm/mach-w55fa93/include/mach/timex.h
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 *
 * Based on arch/arm/mach-s3c2410/include/mach/timex.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __ASM_ARCH_TIMEX_H
#define __ASM_ARCH_TIMEX_H

/* CLOCK_TICK_RATE Now, I don't use it. */

#ifdef CONFIG_EXTCLK_12M
	#define CLOCK_TICK_RATE 12000000
#elif defined CONFIG_EXTCLK_27M
	#define CLOCK_TICK_RATE 27000000
#endif

#endif /* __ASM_ARCH_TIMEX_H */
