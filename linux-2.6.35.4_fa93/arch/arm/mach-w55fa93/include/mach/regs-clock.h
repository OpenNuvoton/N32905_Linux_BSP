/* arch/arm/mach-w55fa93/include/mach/regs-clock.h
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 * wan zongshun <zswan@nuvoton.com>
 * Based on arch/arm/mach-s3c2410/include/mach/regs-clock.h
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */


#ifndef __ASM_ARM_REGS_CLOCK
#define __ASM_ARM_REGS_CLOCK "$Id: clock.h,v 1.0 2008/10/06 15:40:56 wan zongshun Exp $"


#define PLL0		0x00
#define PLL1		0x01
#define EXTAL15M	0x03

/* Define PLL initialization data structure */
typedef struct PLL_INIT_STRUCT//zswan add it
{
		unsigned int		pll0;
    unsigned int		pll1;
    unsigned int		cpu_src;
    unsigned int		ahb_clk;
    unsigned int		apb_clk;
} pll_config;


/* Define PLL freq. setting */
#define PLL_DISABLE	0x12B63
#define	PLL_66MHZ	0x2B63
#define	PLL_100MHZ	0x4F64
#define PLL_120MHZ	0x4F63
#define	PLL_166MHZ	0x4124
#define	PLL_200MHZ	0x4F24


/* Define CPU clock source */
#define CPU_FROM_PLL0			0
#define CPU_FROM_PLL1			1
#define CPU_FROM_PLL0_DIV_2		2
#define CPU_FROM_EXTAL15M		3


/* Define AHB clock */
#define	AHB_CPUCLK_1_1	0
#define	AHB_CPUCLK_1_2	1
#define	AHB_CPUCLK_1_4	2
#define	AHB_CPUCLK_1_8	3

/* Define APB clock */
#define APB_AHB_1_2		1
#define APB_AHB_1_4		2
#define APB_AHB_1_8		3

#ifndef __ASSEMBLY__

#include <asm/div64.h>

#endif /* __ASSEMBLY__ */


#endif /* __ASM_ARM_REGS_CLOCK */
