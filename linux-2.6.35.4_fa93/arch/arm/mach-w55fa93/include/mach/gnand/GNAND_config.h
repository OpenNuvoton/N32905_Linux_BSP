 /* linux/include/asm/arch/gnand/GNAND_config.h
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
 *   2008/08/19     jcao add this file for nuvoton all nand driver.
 */
 
#ifndef _GNAND_CONFIG_H_
#define _GNAND_CONFIG_H_

#define W90P910

#ifdef W90P910
#define NON_CACHE_BIT		0x80000000
#endif

#ifdef W90P710
#define NON_CACHE_BIT		0x80000000
#endif

#ifdef W99702
#define NON_CACHE_BIT		0x10000000
#endif

#define OP_THRESHOLD		256             	/* maximum number of operation history */
#define OP_CMD_LEN			32

#define L2PN_RESERVED		8
#define P2LN_RESERVED		8


#define GNAND_GET16_L(bptr,n)   	(bptr[n] | (bptr[n+1] << 8))
#define GNAND_GET32_L(bptr,n)   	(bptr[n] | (bptr[n+1] << 8) | (bptr[n+2] << 16) | (bptr[n+3] << 24))
#define GNAND_PUT16_L(bptr,n,val)	bptr[n] = val & 0xFF;				\
									bptr[n+1] = (val >> 8) & 0xFF;
#define GNAND_PUT32_L(bptr,n,val)	bptr[n] = val & 0xFF;				\
									bptr[n+1] = (val >> 8) & 0xFF;		\
									bptr[n+2] = (val >> 16) & 0xFF;		\
									bptr[n+3] = (val >> 24) & 0xFF;

#define GNAND_GET16_B(bptr,n)   	((bptr[n]) << 8 | bptr[n+1])
#define GNAND_GET32_B(bptr,n)   	((bptr[n] << 24) | (bptr[n+1] << 16) | (bptr[n+2] << 8) | bptr[n+3])
#define GNAND_PUT16_B(bptr,n,val)	bptr[n+1] = val & 0xFF;				\
									bptr[n] = (val >> 8) & 0xFF;
#define GNAND_PUT32_B(bptr,n,val)	bptr[n+3] = val & 0xFF;				\
									bptr[n+2] = (val >> 8) & 0xFF;		\
									bptr[n+1] = (val >> 16) & 0xFF;		\
									bptr[n] = (val >> 24) & 0xFF;

#define	GNAND_min(x,y)			(((x) <	(y)) ? (x) : (y))
#define	GNAND_MIN(x,y)			(((x) <	(y)) ? (x) : (y))
#define	GNAND_max(x,y)			(((x) >	(y)) ? (x) : (y))
#define	GNAND_MAX(x,y)			(((x) >	(y)) ? (x) : (y))


#endif 	/* _GNAND_CONFIG_H_ */
