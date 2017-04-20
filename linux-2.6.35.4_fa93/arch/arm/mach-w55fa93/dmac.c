/* linux/arch/arm/mach-w55fa93/dmac.c
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
 *   2008/08/28     YAchen add this file for nuvoton DMA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/highmem.h>
#include <linux/blkdev.h>
#include <linux/string.h>

#include <scsi/scsi.h>
#include <scsi/scsi_device.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/scatterlist.h>
#include <asm/cacheflush.h>
#include <mach/hardware.h>
#include <mach/map.h>

DECLARE_WAIT_QUEUE_HEAD(dmac_wq);
DECLARE_MUTEX(dmac_sem); 


EXPORT_SYMBOL(dmac_sem);


#define DMA_BASE_ADDR		        W55FA93_VA_DMAC_BUF_BASE
#define W55FA93_DMACCSR				(0x400 + DMA_BASE_ADDR) 
#define W55FA93_DMACISR				(0x414 + DMA_BASE_ADDR)

static int __init dmac_init(void)
{
	return 0;
}


static void __exit dmac_exit(void)
{

}

module_init(dmac_init);
module_exit(dmac_exit);


