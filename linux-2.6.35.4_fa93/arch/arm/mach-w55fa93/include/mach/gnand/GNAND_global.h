/* linux/include/asm/arch/gnand/GNAND_global.h                                       */
/*-----------------------------------------------------------------------------------*/
/* Nuvoton Electronics Corporation confidential                                      */
/*                                                                                   */
/* Copyright (c) 2008 by Nuvoton Electronics Corporation                             */
/* All rights reserved                                                               */
/*                                                                                   */
/* This program is free software; you can redistribute it and/or modify              */
/* it under the terms of the GNU General Public License as published by              */
/* the Free Software Foundation; either version 2 of the License, or                 */
/* (at your option) any later version.                                               */
/*-----------------------------------------------------------------------------------*/
/* Changelog:
 * 2008/08/19     jcao add this file for nuvoton all nand driver.
 *
 * 2011/02/18 by CJChen1@nuvoton.com
 *          To support new feature wear-leveling.
 *
 * 2011/04/27 by CJChen1@nuvoton.com
 *          Backward compatible to GNAND format version V1.00 and V1.01 on NAND chip
 */
 
#ifndef _GNAND_GLOBAL_H_
#define _GNAND_GLOBAL_H_

//#define DBG_MSG		printk
//#define DBG_MSG		sysprintf
#define DBG_MSG(...)	

// DBG_INFO() should show minimum information for RD under development stage.
//      For example, GNAND version, fetal error, critical check point, and so on.
#define DBG_INFO        printk

#define FREE_BLOCK			0xFFFF
#define BAD_BLOCK			0xFFF0
#define OP_BLOCK			0xFFAA
#define L2PN_BLOCK			0xFF55
#define P2LN_BLOCK			0xFF55

/* global string define */
#define P2LN_INFO_MAGIC		    "GNAND"

// P2LN_INFO_LIB_VERSION means the version of running GNAND library.
//      Lib version 1.02 and 1.01 use same format on NAND chip, that is, P2LN_INFO_VERSION 1.01
#define P2LN_INFO_LIB_VERSION	"V1.02.3"

// P2LN_INFO_VERSION means the version of GNAND data format on NAND chip.
#define P2LN_INFO_VERSION	    "V1.01"
#define P2LN_INFO_VERSION_OLD	"V1.00"
#define P2LN_INFO_DATE			"20101125_0"

#define P2LN_INFO_T		struct p2ln_info_t

struct p2ln_info_t
{
	CHAR    magic[8];		/* "GNAND"                               */
	CHAR    version[8];		/* "V1.00"                               */
	CHAR    date_code[16];	/* for example: "20080220_1"             */
	UINT16	op_block;    	/* OP block address                      */
	UINT16  old_op_block;	/* Old OP block address                  */
	UINT16  old_p2ln;		/* old P2LN block address                */
	UINT16  old_p2ln1;		/* old P2LN1 block address               */
	UINT32  block;			/* block per zone                        */
};


/*-------------------------------------------------------------------*/
/* Operation history                                                 */
/*-------------------------------------------------------------------*/
#define OP_LINK				"LINK"
#define OP_RELINK			"RELINK"
#define OP_P2LN				"P2LN"

#define GNOP_LINK_T		struct gnop_link_t
#define GNOP_RELINK_T	struct gnop_relink_t
#define GNOP_P2LN_T		struct gnop_p2ln_t

struct gnop_link_t
{
	CHAR  	op[8];			/* string "LINK"  */
	UINT16	lba;
	UINT16	pba;
	CHAR    reserved[20];
} ;


struct gnop_relink_t
{
	CHAR  	op[8];  		/* string "RELINK" */
	UINT16	lba;
	UINT16	old_pba;
	UINT16	new_pba;
	UINT16	start_page;
	UINT16	page_cnt;
	UINT16	check_mark;
	CHAR    reserved[12];
} ;


struct gnop_p2ln_t
{
	CHAR	op[8];			/* string "P2LN" */
	UINT16	old_p2ln;
	UINT16	old_op;
	UINT16	new_p2ln;
	UINT16	new_op;
	UINT16	old_p2ln1;
	UINT16	new_p2ln1;
	CHAR	reserved[12];
} ;


/*-------------------------------------------*/
/* global variable extern					 */
/*-------------------------------------------*/
extern UINT8 *_gnand_pDMABuffer;
extern UINT8 *_gnand_pUBBuffer;

//--- To support new feature wear-leveling
// all 3 global variables support up to 8 zones that like as db_idx[16] in struct ndisk_t
extern UINT32 age_sum[16];             // sum of age of all valid blocks
extern UINT16 valid_block_count[16];   // number of all valid blocks
extern UINT16 age_average[16];         // average of age of all valid blocks = (UINT16)(age_sum[zone] / valid_block_count[zone])

/*-------------------------------------------------------------------*/
/* Export functions (GNAND library internal)                         */
/*-------------------------------------------------------------------*/
INT  GNAND_OP_LinkNew(NDISK_T *ptNDisk, UINT16 LBlockAddr, UINT16 *PBlockAddr);
INT  GNAND_OP_ReLink(NDISK_T *ptNDisk, UINT16 LBlockAddr, UINT16 *PBlockAddr, INT nStartPage, INT nPageCnt, BOOL bIsBegin);
INT  GNAND_OP_ReCover(NDISK_T *ptNDisk, UINT16 LBlockAddr, INT nStartPage, INT nPageCnt);
INT  GNAND_OP_P2LN(NDISK_T *ptNDisk);
INT  GNAND_UpdateP2LN(NDISK_T *ptNDisk);
INT  GNAND_IsValidP2LN(NDISK_T *ptNDisk, UINT16 pba, P2LN_INFO_T *p2ln_info);

INT  GNAND_ParseNandROM(NDISK_T *ptNDisk);
INT  GNAND_ParseNandDisk(NDISK_T *ptNDisk, BOOL bEraseIfNotGnandFormat);
INT  GNAND_get_new_data_block(NDISK_T *ptNDisk, UINT16 LBlockAddr, UINT16 *PBlockAddr);
INT  GNAND_check_empty(NDISK_T *ptNDisk, INT PBA);

VOID GNAND_DirtyPageSet(NDISK_T *ptNDisk, INT pba, INT page);
VOID GNAND_DirtyPageClear(NDISK_T *ptNDisk, INT pba, INT page);
VOID GNAND_DirtyPageClearBlock(NDISK_T *ptNDisk, INT pba);

extern VOID fsDumpBufferHex(UINT8 *pucBuff, INT nSize);

INT GNAND_show_op(NDISK_T *ptNDisk);
INT GNAND_show_l2pm(NDISK_T *ptNDisk);

#endif 	/* _GNAND_GLOBAL_H_ */
