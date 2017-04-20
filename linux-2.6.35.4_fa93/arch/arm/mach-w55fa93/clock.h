/*
 * linux/arch/arm/mach-w55fa93/clock.h
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 */

#include <asm/clkdev.h>

void w55fa93_ahbclk_enable(struct clk *clk, int enable);
void w55fa93_apbclk_enable(struct clk *clk, int enable);

struct clk {
	unsigned long		cken;
	unsigned int		enabled;
	void			(*enable)(struct clk *, int enable);
};

#define DEFINE_AHBCLK(_name, _ctrlbit)			\
struct clk clk_##_name = {				\
		.enable	= w55fa93_ahbclk_enable,	\
		.cken	= (1 << _ctrlbit),		\
	}

#define DEFINE_APBCLK(_name, _ctrlbit)			\
struct clk clk_##_name = {				\
		.enable	= w55fa93_apbclk_enable,	\
		.cken	= (1 << _ctrlbit),		\
	}


#define DEF_CLKLOOK(_clk, _devname, _conname)		\
	{						\
		.clk		= _clk,			\
		.dev_id		= _devname,		\
		.con_id		= _conname,		\
	}

