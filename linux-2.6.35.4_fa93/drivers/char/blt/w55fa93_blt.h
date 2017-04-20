/* bltpriv.h
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 * <ccchang12@nuvoton.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __ASM_ARM_W55FA93_BLT_H
#define __ASM_ARM_W55FA93_BLT_H

//#include <asm/semaphore.h>
#include <linux/semaphore.h>
//#include <asm/arch/DrvEDMA.h>
#include <mach/DrvEDMA.h>
#include <mach/DrvBLT.h>

#define BLT_MAJOR	198
#define BLT_MINOR	0
#define BLT2_MAJOR	198
#define BLT2_MINOR	1

#if 0
#define FB_SIZE		(320*240*4)
#define MAP_SIZE	(FB_SIZE+1024*1024)
#define MAP_SIZE_ORDER	9
#endif

typedef enum blt_state {
	BLT_CLOSE	= 0,
	BLT_IDLE	= 1,
	BLT_RUNNING	= 2,
	BLT_FINISH	= 3,
	BLT_ERROR	= 9,
} blt_state_t;

#define IS_FINISH(state)	(state == BLT_FINISH)
#define IS_ERROR(state)		(state == BLT_ERROR)
#define IS_DONE(state)		(IS_FINISH(state)|IS_ERROR(state))

typedef enum blt_func {
	BLT_NONE	= 0,
	BLT_BLIT	= 1,
	BLT_FILL	= 2,
} blt_func_t;

typedef struct blt_priv {
	struct semaphore	lock;
	blt_state_t		state;
	blt_func_t		func;
	unsigned char		size_order;
	unsigned int		size;
	unsigned int		vaddr;		// if vaddr_src/vaddr_dst == 0, will using vaddr address
	unsigned int		paddr;		// if paddr_src/paddr_dst == 0, will using paddr address
	unsigned int		paddr_src;
	unsigned int		paddr_dst;
} blt_priv_t;

#endif//__ASM_ARM_W55FA93_BLT_H

