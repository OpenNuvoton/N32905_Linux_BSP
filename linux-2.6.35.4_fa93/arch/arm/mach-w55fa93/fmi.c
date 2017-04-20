/* linux/arch/arm/mach-w55fa93/fmi.c
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
 *   2008/08/28     YAchen add this file for nuvoton fmi.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/highmem.h>
#include <linux/blkdev.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
// semaphore for preventing concurrent FMI devices activity 
DECLARE_MUTEX(fmi_sem);
EXPORT_SYMBOL(fmi_sem);

