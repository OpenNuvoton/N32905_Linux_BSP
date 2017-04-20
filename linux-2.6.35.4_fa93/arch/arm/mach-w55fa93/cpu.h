/* 
 * arch/arm/mach-w55fa93/cpu.h
 *
 * Based on linux/include/asm-arm/plat-s3c24xx/cpu.h by Ben Dooks
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 *
 * Header file for W55FA93 CPU support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

//#define IODESC_ENT(y) { (unsigned long)W55FA93_VA_##y, __phys_to_pfn(W55FA93_PA_##y), W55FA93_SZ_##y, MT_DEVICE }
#define IODESC_ENT(y)						\
{								\
	.virtual	= (unsigned long)W55FA93_VA_##y,	\
	.pfn		= __phys_to_pfn(W55FA93_PA_##y),	\
	.length		= W55FA93_SZ_##y,			\
	.type		= MT_DEVICE,				\
}

#ifndef MHZ
#define MHZ (1000*1000)
#endif
#define print_mhz(m) ((m) / MHZ), ((m / 1000) % 1000)

/* forward declaration */
struct w55fa93_uartcfg;
struct map_desc;
struct sys_timer;
extern struct sys_timer w55fa93_timer;

/* CPU identifier register*/

#define W55FA93_CPUID	0x00FA5C30

/* core initialisation functions */

extern void w55fa93_init_irq(void);
extern void w55fa93_init_clocks(void);
extern void w55fa93_init_uarts(struct w55fa93_uartcfg *cfg, int no);
extern void w55fa93_map_io(void);
extern void cpu_map_io(struct map_desc *mach_desc, int mach_size);
extern void w55fa93_board_init(void);
extern void w55fa93_dev_init(void);

/* for public w55fa93 */

extern struct platform_device *w55fa93_uart_devs[2];
//extern struct platform_device w55fa93_serial_device;
extern struct platform_device w55fa93_device_usb;
extern struct platform_device w55fa93_device_usbh;
extern struct platform_device w55fa93_device_lcd;
extern struct platform_device w55fa93_device_ts;
extern struct platform_device w55fa93_device_i2c;
//extern struct platform_device w55fa93_device_wdt;
extern struct platform_device w55fa93_device_rtc;
extern struct platform_device w55fa93_device_fmi;
extern struct platform_device w55fa93_device_kpi;
