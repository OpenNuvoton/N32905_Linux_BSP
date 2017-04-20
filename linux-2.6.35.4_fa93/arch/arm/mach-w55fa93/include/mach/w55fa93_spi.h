/*
 * arch/arm/mach-w55fa93/include/mach/w55fa93_spi.h
 *
 * Copyright (c) 2009 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#ifndef __ASM_ARCH_SPI_H
#define __ASM_ARCH_SPI_H

extern void mfp_set_groupg(struct device *dev);

struct w55fa93_spi_info {
	unsigned int num_cs;
	unsigned int lsb;
	unsigned int txneg;
	unsigned int rxneg;
	unsigned int divider;
	unsigned int sleep;
	unsigned int txnum;
	unsigned int txbitlen;
	unsigned int byte_endin;
	int bus_num;
};

struct w55fa93_spi_chip {
	unsigned char bits_per_word;
};

#endif /* __ASM_ARCH_SPI_H */
