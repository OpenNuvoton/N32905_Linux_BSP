/*
 * arch/arm/mach-w55fa93/include/mach/uncompress.h
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 *
 * Based on arch/arm/mach-s3c2410/include/mach/uncompress.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __ASM_ARCH_UNCOMPRESS_H
#define __ASM_ARCH_UNCOMPRESS_H

/* Defines for UART registers */
#include <mach/w55fa93_reg.h>
#include <mach/map.h>

#define uart_base W55FA93_PA_UART

static __inline__ void
uart_wr(unsigned int reg, unsigned int val)
{
	volatile unsigned int *ptr;

	ptr = (volatile unsigned int *)(reg + uart_base);
	*ptr = val;
}

static __inline__ unsigned int
uart_rd(unsigned int reg)
{
	volatile unsigned int *ptr;

	ptr = (volatile unsigned int *)(reg + uart_base);
	return *ptr;
}

/*
 * we can deal with the case the UARTs are being run
 * in FIFO mode, so that we don't hold up our execution
 * waiting for tx to happen...
*/

static void putc(int ch)
{
	/* not using fifos */
	while (!(uart_rd(W55FA93_COM_FSR) & UART_FSR_TEMT))
			barrier();

	/* write byte to transmission register */
	uart_wr(W55FA93_COM_TX, ch);
}

static inline void flush(void)
{
}

static void arch_decomp_setup(void)
{
}

#endif /* __ASM_ARCH_UNCOMPRESS_H */
