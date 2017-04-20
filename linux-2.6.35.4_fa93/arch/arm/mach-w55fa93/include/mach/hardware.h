/* arch/arm/mach-w55fa93/include/mach/hardware.h
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 *
 * Based on arch/arm/mach-s3c2410/include/mach/hardware.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/sizes.h>
#include <mach/map.h>

#define CONFIG_NO_MULTIWORD_IO
//#define PCIMEM_BASE		0xC0000000
#define PCIBIOS_MIN_IO          0xE0080000
#define PCIBIOS_MIN_MEM         0xE0000000
#define pcibios_assign_all_busses()     0

#endif /* __ASM_ARCH_HARDWARE_H */
