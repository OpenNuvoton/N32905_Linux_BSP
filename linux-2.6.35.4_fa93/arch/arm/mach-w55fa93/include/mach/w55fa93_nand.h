 /* linux/include/asm/arch/gnand/w55fa93_nand.h
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
 
#ifndef _W55FA93_NAND_H_
#define _W55FA93_NAND_H_



#define DMA_BLOCK_SIZE		0x200

/* FMI Global Control and Status Register(FMICSR) */
#define FMICSR_SW_RST		(1)
#define FMICSR_SD_EN		(1<<1)
#define FMICSR_MS_EN		(1<<2)
#define FMICSR_SM_EN		(1<<3)
#define FMICSR_CF_EN		(1<<4)

/* FMI Global Interrupt Control Register(FMIIER) */
#define FMIIER_DTA_IE		(1)

/* FMI Global Interrupt Status Register (FMIISR) */
#define FMIISR_DTA_IF		(1)


/* DMAC Control and Status Register (DMACCSR) */
#define DMACCSR_DMACEN		(1)
#define DMACCSR_SW_RST		(1<<1)
#define DMACCSR_SG_EN1		(1<<2)
#define DMACCSR_SG_EN2		(1<<3)
#define DMACCSR_ATA_BUSY	(1<<8)
#define DMACCSR_FMI_BUSY	(1<<9)

/* DMAC Interrupt Enable Register (DMACIER) */
#define DMACIER_TABORT_IE	(1)
#define DMACIER_WEOT_IE		(1<<1)

/* DMAC Interrupt Status Register (DMACISR) */
#define DMACISR_TABORT_IF	(1)
#define DMACISR_WEOT_IF		(1<<1)

/* DMAC BIST Control and Status Register (DMACBIST) */
#define DMACBIST_BIST_EN	(1)
#define DMACBIST_FINISH		(1<<1)
#define DMACBIST_FAILED		(1<<2)

#define NAND_SHORT_DELAY HZ/2
#define NAND_LONG_DELAY  HZ

/* SD error code internal using */
#define SD_SUCCESS			0x00
#define SD_FAILED			0x01
#define SD_TIMEOUT			0x02
#define SD_REMOVED			0x03
#define SD_STATE_ERR		0x04

/* sd host operation state */
#define SD_STATE_NOP				0x00
#define SD_STATE_READ				0x10

#define SD_STATE_WRITE				0x20
#define SD_STATE_WRITE_BUS_BUSY	0x21
#define SD_STATE_WRITE_START		0x22

/* card type */
#define CARD_TYPE_SD				0x100
#define CARD_TYPE_MMC				0x200
#define CARD_TYPE_SDIO				0x400

/* time out value ( in jiffies ) */
#define SD_CMD_TIMEOUT		100
#define SD_RESP_TIMEOUT	100
#define SD_BUS_TIMEOUT		10
#define SD_TICKCOUNT 200

#define SD_SHORT_DELAY 20
#define SD_LONG_DELAY  50


/* fmi reference clock : up to 25MHz*/
#ifdef CONFIG_PLL0_66MHZ
#define FMI_INPUT_CLOCK		66000
#define _fmi_uFMIReferenceClock		66000
#endif

#ifdef CONFIG_PLL0_100MHZ
#define FMI_INPUT_CLOCK		100000
#define _fmi_uFMIReferenceClock		100000
#endif

#ifdef CONFIG_PLL0_133MHZ
#define FMI_INPUT_CLOCK		133000
#define _fmi_uFMIReferenceClock		133000
#endif

#ifdef CONFIG_PLL0_166MHZ
#define FMI_INPUT_CLOCK		166000
#define _fmi_uFMIReferenceClock		166000
#endif

#ifdef CONFIG_PLL0_200MHZ
#define FMI_INPUT_CLOCK		200000
#define _fmi_uFMIReferenceClock		200000
#endif		

/* Driver thread command */
#define SD_EVENT_NONE				0
#define SD_EVENT_ADD				1
#define SD_EVENT_REMOVE			2
#define SD_EVENT_QUIT				-1

/* SCSI Sense Key/Additional Sense Code/ASC Qualifier values */
#define SS_NO_SENSE				0
#define SS_COMMUNICATION_FAILURE		0x040800
#define SS_INVALID_COMMAND			0x052000
#define SS_INVALID_FIELD_IN_CDB			0x052400
#define SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE	0x052100
#define SS_LOGICAL_UNIT_NOT_SUPPORTED		0x052500
#define SS_MEDIUM_NOT_PRESENT			0x023a00
#define SS_MEDIUM_REMOVAL_PREVENTED		0x055302
#define SS_NOT_READY_TO_READY_TRANSITION	0x062800
#define SS_RESET_OCCURRED			0x062900
#define SS_SAVING_PARAMETERS_NOT_SUPPORTED	0x053900
#define SS_UNRECOVERED_READ_ERROR		0x031100
#define SS_WRITE_ERROR				0x030c02
#define SS_WRITE_PROTECTED			0x072700
#define SS_INVALID_MEDIUM			0x033000
#define SS_MEDIUM_CHANGED			0x062800


typedef struct _FLAG_SETTING {
	int bWriteProtect;
	int bCrcCheck;
	int bInitSuccess;
	int bMediaChanged;
	volatile int bCardExist;
	volatile int bBusBusy;
	volatile int sdRemove;
	
	
	int bWriteProtect1;
	int bCrcCheck1;
	int bInitSuccess1;
	int bMediaChanged1;
	volatile int bCardExist1;
	volatile int bBusBusy1;
	volatile int sdRemove1;
	
	volatile int curCard;
	volatile int update;
	volatile int needReset;
} NAND_FLAG_SETTING, *PFLAG_SETTING;




struct nand_hostdata {
	int myID;
	volatile unsigned int state, sense;

	int curDBuf, curSBuf, curCount, bufCount, bakCount;

	struct scatterlist *firstList;
	unsigned int nTotalLists, curOffset, curList;
	unsigned int DMAvaddr, DMApaddr;
	
	


	int cardType;

	/* for card info */
	unsigned char CSD[16];
	unsigned char CID[16];
	unsigned int RCA, sdioRCA, sdioFun;

	unsigned int nSectorSize;
	unsigned int nTotalSectors;
	unsigned int nCapacityInByte;

	// for SCSI sense
	uint8_t senseKey;
	uint8_t ASCkey;
	uint8_t ASCQkey;

	struct{
		int busy;
		int hasData;
	}interBuf[2];


	struct Scsi_Host *shost;
	struct scsi_cmnd *cmd;
	struct device dev;
	
	
	
};


#define Swap32(val)				((val << 24) |\
								((val << 8) & 0xff0000) |\
								((val >> 8) & 0xff00) |\
								(val >> 24))
								
#define Make32(c4, c3, c2, c1)		(((c4<<24)  & 0xff000000) |\
								((c3 << 16) & 0x00ff0000) |\
								((c2 << 8) & 0x0000ff00) |\
								(c1 & 0x000000ff))

// Used to stuff Cmnd->result
#define SD_CMD_RESULT( x, y, c )     ( ( ( x )<< 16 )|( ( y )<< 8 ) | ( c ) )

#define SD_COMPUTE_START_LBA(cmd)		(((uint32_t)cmd->cmnd[2] << 24 ) + \
					                                   ((uint32_t)cmd->cmnd[3] << 16 ) + \
		       			                            ((uint32_t)cmd->cmnd[4] << 8)   + \
		                            			       ((uint32_t)cmd->cmnd[5] ))
		                            			       
#define SD_COMPUTE_BLK_COUNT(cmd)	((((uint32_t)cmd->cmnd[7]<<8) + \
										((uint32_t)cmd->cmnd[8])) / DMA_BLOCK_SIZE)

#if 0
#define WB_NAND_DRIVER {									\
	      .name 					= "NAND0",						\
        .info					= nand_info,						\
        .queuecommand			= nand_queue_cmd,		      		\
        .eh_abort_handler 		= nand_abort,					\
        .eh_host_reset_handler		= nand_reset,					\
        .bios_param				= nand_bios_param,				\
        .ioctl					= nand_ioctl,						\
        .can_queue	     			= 1,							\
        .this_id        				= -1,							\
        .sg_tablesize   			= 128,							\
        .cmd_per_lun    			= 1,							\
        .unchecked_isa_dma		= 0,							\
        .use_clustering  			= ENABLE_CLUSTERING,			\
        .module					= THIS_MODULE,				\
}
#endif

// IOCTL 

#define SD_IOC_MAGIC		's'
#define SD_IOC_NR			2

#define SD_IOC_GET_CARD_TYPE		_IOR(SD_IOC_MAGIC, 0, int)
#define SD_IOC_IO_READ_WRITE		_IOR(SD_IOC_MAGIC, 1, int)

#define outphw(port,value)    (*((u16 volatile *) (port))=value)
#define inphw(port)           (*((u16 volatile *) (port)))
#define outpw(port,value)     (*((u32 volatile *) (port))=value)
#define inpw(port)            (*((u32 volatile *) (port)))
#define outp(port,value)     (*((u8 volatile *) (port))=value)
#define inp(port)            (*((u8 volatile *) (port)))

#endif


