/* linux/include/asm-arm/arch-w55fa93/w55fa93_mfid.h
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
 */


#ifndef W55FA93_MFID_H
#define W55FA93_MFID_H


#define MFID_MAGIC 'm'
#define MFID_MAXNR  2



#define MFID_BLOCK			_IOW('m', 1, unsigned int)
#define MFID_NONBLOCK		_IOW('m', 2, unsigned int)


typedef unsigned char     UINT8;    
typedef unsigned short    UINT16;
typedef unsigned int      UINT32;
typedef void              VOID;
typedef int               INT;
typedef int				  INT32;

#define outp32(addr,value)	outl(value, addr)
#define inp32(addr)		inl(addr)

typedef struct {
    int    	IdCode;
    int    	Isdown;
    int		ChkCnt;
} mfidItemTag;

#endif
