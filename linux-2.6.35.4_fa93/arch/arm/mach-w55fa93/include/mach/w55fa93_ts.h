/* linux/include/asm-arm/arch-w55fa93/w55fa93_ts.h
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Changelog:
 *
 *   2006/08/26     vincen.zswan add this file for nuvoton w55fa93 evb.
 */

#ifndef W55FA93_ADC_H
#define W55FA93_ADC_H


//#define ADC_INT             0x040000
//#define WT_INT              0x100000


#define ADC_IOC_MAGIC     0xd2

#define ADC_SENSITIVITY   _IO(ADC_IOC_MAGIC, 0)
#define ADC_SET_CLKDIV    _IO(ADC_IOC_MAGIC, 1)
#define ADC_SET_BITS      _IO(ADC_IOC_MAGIC, 2)

#define ADC_MAXNR         2

int w55fa93ts_open_again(void);
int w55fa93ts_close_again(void);

#endif
