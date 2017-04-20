/* linux/include/asm-arm/arch-w55fa93/w55fa93_keypad.h
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
 *   2008/12/34     jcao add this file for nuvoton w55fa93 kaypad driver.
 */


#ifndef W55FA93_KEYPAD_H
#define W55FA93_KEYPAD_H

#define KPD_MAJOR 192
#define KPD_MINOR 0

#define KEYPAD_MAGIC 'k'
#define KEYPAD_MAXNR  4

/*
#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
*/

#define BUTTON_PRESSED_MENU		0x01
#define BUTTON_PRESSED_D		0x02
#define BUTTON_PRESSED_CANCEL	0x04
#define BUTTON_PRESSED_OK		0x08
#define BUTTON_PRESSED_UP		0x10
#define BUTTON_PRESSED_DOWN		0x20
#define BUTTON_PRESSED_LEFT		0x40
#define BUTTON_PRESSED_RIGHT	0x80



#define KPD_BLOCK			_IOW('k', 1, unsigned int)
#define KPD_NONBLOCK		_IOW('k', 2, unsigned int)
#define KPD_DELAY		    _IOW('k', 3, unsigned int)
#define KPD_GET_KEY_STATUS        _IOR('k', 4, unsigned int)

typedef unsigned char     UINT8;    
typedef unsigned short    UINT16;
typedef unsigned int      UINT32;
typedef void              VOID;
typedef int               INT;

#define outp32(addr,value)	outl(value, addr)
#define inp32(addr)		inl(addr)


/* Define De-bounce counter clock source select*/
typedef enum {
	eDBCLK_XIN = 0,
	eDBCLK_X32K	
} E_DEBOUNCECLK_SRC;

/* Define De-bounce Enable*/
typedef enum {
	eDB_DISABLE = 0,
	eDB_ENABLE	
} E_DEBOUNCE_ENABLE;

/* Define De-bounce sampling cycle*/
typedef enum {
	eDEBOUNCE_1CLK = 0,
	eDEBOUNCE_2CLK,
	eDEBOUNCE_4CLK,
	eDEBOUNCE_8CLK,
	eDEBOUNCE_16CLK,
	eDEBOUNCE_32CLK,
	eDEBOUNCE_64CLK,
	eDEBOUNCE_128CLK,
	eDEBOUNCE_256CLK,
	eDEBOUNCE_512CLK,
	eDEBOUNCE_1024CLK,
	eDEBOUNCE_2048CLK,
	eDEBOUNCE_4096CLK,
	eDEBOUNCE_8192CLK		
} E_DEBOUNCE_CLK;

#define KPD_ROWNUM  	(CONFIG_INPUT_KEYPAD_ROW - 1)
#define KPD_COLNUM  	(CONFIG_INPUT_KEYPAD_COLUMN - 1)

#endif
