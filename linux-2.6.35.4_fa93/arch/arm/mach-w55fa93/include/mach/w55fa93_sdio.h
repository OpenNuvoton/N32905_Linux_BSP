/* linux/include/asm-arm/arch-w55fa93/w55fa93_sdio.h
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
 *   2006/08/26     vincen.zswan add this file for nuvoton w55fa93 evb.
 */


#ifndef _W55FA93_SDIO_H_
#define _W55FA93_SDIO_H_


#define DMA_BLOCK_SIZE		0x200
#define SD_BLOCK_SIZE		  0x200


#define CLKMAN_SDH_EN           (1<<3)
#define PINFUN_SPI1PIN_EN       (1<<12) // SPI 1 pin enable
#define PINFUN_SDHPIN_EN        (1<<11) // SD Card Host Function Pin Enabled
#define PINFUN2_SDDET_SEL       (0xf<<5) // SD Card detect Pin Selection(bit 5~ 8)
#define RSTCON_SDHRST           (1<<2)  // SD Card Host Controller Reset

#define GPIO_DOUT0             (1) // GPIO port A data output value 

#if 0
	/* FMI Global Control and  Status Register(FMICSR) */
	#define FMI_SWRST       (1)
	#define FMI_SD_EN       (1<<1)
	#define FMI_SM_EN       (1<<3)
	
	/* FMI Global Interrupt Control Register(FMIIER) */
	#define FMI_DAT_IE		  (1)
	
	/* FMI Global Interrupt Status Register (FMIISR) */
	#define FMI_DAT_IF		  (1)
	
	
	/* SD Control and Status Register (SDCR) */
	#define SDCR_CO_EN		    (1)
	#define SDCR_RI_EN		    (1<<1)
	#define SDCR_DI_EN		    (1<<2)
	#define SDCR_DO_EN		    (1<<3)
	#define SDCR_R2_EN		    (1<<4)
	#define SDCR_74CLK_OE		  (1<<5)
	#define SDCR_8CLK_OE		  (1<<6)
	#define SDCR_CLK_KEEP 		(1<<7)
	#define SDCR_SD_SWRST	    (1<<14)
	#define SDCR_DBW		      (1<<15)
	#define SDCR_SDNWR        (0xF<<24)
	
	
	/* SD Interrupt Control Register (SDIER) */
	#define SDIER_BLKD_IEN		  (1)
	#define SDIER_CRC_IEN		    (1<<1)
	#define SDIER_CD_IEN		    (1<<8)
	#define SDIER_SDIO_IEN		  (1<<10)
	#define SDIER_RITO_IEN		  (1<<12)
	#define SDIER_DITO_IEN		  (1<<13)
	#define SDIER_WKUP_EN		    (1<<14)
	#define SDIER_CDSRC		      (1<<30)
	
	
	/* SD Interrupt Status Register (SDISR) */
	#define SDISR_BLKD_IF	    (1)
	#define SDISR_CRC_IF		  (1<<1)
	#define SDISR_CRC_7		    (1<<2)
	#define SDISR_CRC_16		  (1<<3)
	#define SDISR_SDDAT0		  (1<<7)
	#define SDISR_CD_IF		    (1<<8)
	#define SDISR_SDIO_IF 		(1<<10)
	#define SDISR_RITO_IF	  	(1<<12)
	#define SDISR_DITO_IF	  	(1<<13)
	#define SDISR_CD_Card	    (1<<16)
	#define SDISR_SD_DATA1		(1<<18)
	
	
	/* DMAC Control and Status Register (DMACCSR) */
	#define DMACCSR_DMAC_EN		    (1)
	#define DMACCSR_DMAC_SWRST		(1<<1)
	#define DMACCSR_SG_EN		      (1<<3)
	#define DMACCSR_FMI_BUSY	    (1<<9)
	
	/* DMAC Interrupt Enable Register (DMACIER) */
	#define DMACIER_TABORT_IE	(1)
	#define DMACIER_WEOT_IE		(1<<1)
	
	/* DMAC Interrupt Status Register (DMACISR) */
	#define DMACISR_TABORT_IF	(1)
	#define DMACISR_WEOT_IF		(1<<1)
#endif	

/* SD error code internal using */
#define SD_SUCCESS			0x00
#define SD_FAILED			  0x01
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
#define CARD_TYPE_SDHC				0x800


/* time out value ( in jiffies ) */
#define SD_CMD_TIMEOUT		100
#define SD_RESP_TIMEOUT	  100
#define SD_BUS_TIMEOUT		10
#define SD_TICKCOUNT      200

#define SD_SHORT_DELAY 1//10
#define SD_LONG_DELAY  1 //30

#if 0
	/* fmi reference clock : up to 25MHz*/
	#ifdef CONFIG_PLL0_66MHZ
	#define FMI_INPUT_CLOCK		66000
	#endif
	
	#ifdef CONFIG_PLL0_100MHZ
	#define FMI_INPUT_CLOCK		100000
	#endif
	
	#ifdef CONFIG_PLL0_133MHZ
	#define FMI_INPUT_CLOCK		133000
	#endif
	
	#ifdef CONFIG_PLL0_166MHZ
	#define FMI_INPUT_CLOCK		166000
	#endif
	
	#ifdef CONFIG_PLL0_200MHZ
	#define FMI_INPUT_CLOCK		200000
	#endif		
#endif

extern unsigned int w55fa93_ahb_clock;
//default
#ifndef FMI_INPUT_CLOCK 
#define FMI_INPUT_CLOCK		w55fa93_ahb_clock//400000
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

#if defined(CONFIG_W55FA93_SD0)
	typedef struct _FLAG_SETTING {
		int bWriteProtect;
		int bCrcCheck;
		int bInitSuccess;
		int bMediaChanged;
	  int bCardExist;	
		int r7_cmd;
		
	  int update;
	  int needReset;
	} FLAG_SETTING;

#elif defined(CONFIG_W55FA93_SD_BOTH)

	typedef struct _FLAG_SETTING {
		int bWriteProtect;
		int bCrcCheck;
		int bInitSuccess;
		int bMediaChanged;
	  	int bCardExist;	
		int r7_cmd;
	  	int update;
	  	int needReset;
	  	
		int bWriteProtect1;
		int bCrcCheck1;
		int bInitSuccess1;
		int bMediaChanged1;
	  	int bCardExist1;	
		int r7_cmd1;
	  	int needReset1;

		volatile int sdRemove;	  		  	
	  	int curCard;
	  	int oldCard;	  	
	  	
	  	
	} FLAG_SETTING;
#endif	

#if defined(CONFIG_W55FA93_SD0)
	struct sd_hostdata {

		volatile unsigned int state, sense;
		int curCount;
	
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
		unsigned int nCapacityInByte; // don't use this entry for SDHC since it'll overflow
	
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

#else
	struct sd_hostdata {
		int myID;	
		volatile unsigned int state, sense;
		int curCount;
	
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
		unsigned int nCapacityInByte; // don't use this entry for SDHC since it'll overflow
	
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
#endif


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

struct sdio_op{
	int regAddr;
	int funNo;
	int bWrite:1;
	int bReadAfterWrite:1;
	int bBlockMode:1;
	int bOpCode:1;
	int dataLen;
	unsigned char * data;
	int actualLen;
};
	

// IOCTL 

#define SD_IOC_MAGIC		's'
#define SD_IOC_NR			2

#define SD_IOC_GET_CARD_TYPE		_IOR(SD_IOC_MAGIC, 0, int)
#define SD_IOC_IO_READ_WRITE		_IOR(SD_IOC_MAGIC, 1, int)

#endif


