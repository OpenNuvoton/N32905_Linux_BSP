/* linux/include/asm/arch/NandDrv.h
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

#ifndef _NAND_DRV_H_
#define _NAND_DRV_H_

#define FMI_ERR_ID	0xFFFF0100

#define FMI_TIMEOUT				(FMI_ERR_ID|0x01)
/* NAND error */
#define FMI_SM_INIT_ERROR		(FMI_ERR_ID|0x20)
#define FMI_SM_RB_ERR			(FMI_ERR_ID|0x21)
#define FMI_SM_STATE_ERROR		(FMI_ERR_ID|0x22)
#define FMI_SM_ECC_ERROR		(FMI_ERR_ID|0x23)
#define FMI_SM_STATUS_ERR		(FMI_ERR_ID|0x24)
#define FMI_SM_ID_ERR			(FMI_ERR_ID|0x25)
#define FMI_SM_INVALID_BLOCK	(FMI_ERR_ID|0x26)
#define FMI_SM_NO_MEMORY		(FMI_ERR_ID|0x27)


#define NAND_TYPE_SLC		0x01
#define NAND_TYPE_MLC		0x00

#define NAND_PAGE_512B		512
#define NAND_PAGE_2KB		2048
#define NAND_PAGE_4KB		4096
#define NAND_PAGE_8KB		8192


typedef struct fmi_sm_info_t
{
	u32	uSectorPerFlash;
	u32	uBlockPerFlash;
	u32	uPagePerBlock;
	u32	uSectorPerBlock;
	u32	uLibStartBlock;
//	char	nPageSize;
	u32		nPageSize;
	char	bIsMulticycle;
	char	bIsMLCNand;
	char	bIsNandECC4;
	char	bIsNandECC8;
	char	bIsNandECC12;
	char	bIsNandECC15;			
	char	bIsCheckECC;
} FMI_SM_INFO_T;
extern FMI_SM_INFO_T *pSM0;

int nandInit0(NDISK_T *NDISK_info);
int nandpread0(int PBA, int page, u8 *buff);
int nandpwrite0(int PBA, int page, u8 *buff);
int nand_is_page_dirty0(int PBA, int page);
int nand_is_valid_block0(int PBA);
int nand_block_erase0(int PBA);
int nand_chip_erase0(void);
int nand_ioctl_0(int param1, int param2, int param3, int param4);

int nandInit1(NDISK_T *NDISK_info);
int nandpread1(int PBA, int page, u8 *buff);
int nandpwrite1(int PBA, int page, u8 *buff);
int nand_is_page_dirty1(int PBA, int page);
int nand_is_valid_block1(int PBA);
int nand_block_erase1(int PBA);
int nand_chip_erase1(void);
int nand_ioctl_1(int param1, int param2, int param3, int param4);

//--- Definition for IOCTL
#define NAND_IOC_MAGIC      'n'
#define NAND_IOC_NR         4

#define NAND_IOC_GET_CHIP_INFO      _IOR(NAND_IOC_MAGIC, 0, FMI_IOCTL_INFO_T *)
#define NAND_IOC_IO_BLOCK_ERASE     _IOW(NAND_IOC_MAGIC, 1, FMI_IOCTL_INFO_T *)
#define NAND_IOC_IO_PAGE_READ       _IOR(NAND_IOC_MAGIC, 2, FMI_IOCTL_INFO_T *)
#define NAND_IOC_IO_PAGE_WRITE      _IOW(NAND_IOC_MAGIC, 3, FMI_IOCTL_INFO_T *)

typedef struct fmi_ioctl_info_t     // the data structure for ioctl argument
{
    u32             uBlock; // the physical block number to access
    u32             uPage;  // the page number in block to access
    u8              uWrite_nandloader;  // TRUE to write "check marker" for NandLoader
    u8              *pData; // the pointer to data buffer
    FMI_SM_INFO_T   *pSM;   // the pointer to chip information
} FMI_IOCTL_INFO_T;

#endif
