/* arch/arm/mach-w55fa93/include/mach/serial.h
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 * wan zongshun <zswan@nuvoton.com>
 * Based on arch/arm/mach-s3c2410/include/mach/regs-serial.h
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __ASM_ARM_SERIAL_H
#define __ASM_ARM_SERIAL_H

#ifndef __ASSEMBLY__

/* struct W55FA93_uart_clksrc
 *
*/

struct w55fa93_uart_clksrc {
	const char	*name;
	unsigned int	 divisor;
	unsigned int	 min_baud;
	unsigned int	 max_baud;
};

/* configuration structure for per-machine configurations for the
 * serial port
*/

struct w55fa93_uartcfg {
	unsigned char	   hwport;	 /* hardware port number */
	unsigned char	   unused;
	unsigned short	   flags;
	unsigned long	   uart_flags;	 /* default uart flags */

	unsigned long	   ucon;	 /* value of ucon for port */
	unsigned long	   ulcon;	 /* value of ulcon for port */
	unsigned long	   ufcon;	 /* value of ufcon for port */

	struct w55fa93_uart_clksrc *clocks;
	unsigned int		    clocks_size;
};

/* W55FA93_uart_devs
 *
*/

extern struct platform_device *w55fa93_uart_devs[2];

#endif /* __ASSEMBLY__ */

#endif /* __ASM_ARM_SERIAL_H */

