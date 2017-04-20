/*
 * Copyright (c) 2010 Nuvoton technology corporation.
 *
 * Wan ZongShun <mcuos.com@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#ifndef _W55FA93_I2S_H
#define _W55FA93_I2S_H

#include <linux/io.h>

#if 1
/* Audio Control Registers */
#define ACTL_CON		0x00
#define ACTL_RESET		0x04
#define ACTL_RDSTB		0x08
#define ACTL_RDST_LENGTH	0x0C
#define ACTL_RDSTC		0x10
#define ACTL_RSR		0x14
#define ACTL_PDSTB		0x18
#define ACTL_PDST_LENGTH	0x1C
#define ACTL_PDSTC		0x20
#define ACTL_PSR		0x24
#define ACTL_IISCON		0x28
#define ACTL_ACCON		0x2C
#define ACTL_ACOS0		0x30
#define ACTL_ACOS1		0x34
#define ACTL_ACOS2		0x38
#define ACTL_ACIS0		0x3C
#define ACTL_ACIS1		0x40
#define ACTL_ACIS2		0x44
#define ACTL_COUNTER		0x48

/* bit definition of REG_ACTL_CON register */
//#define R_DMA_IRQ		0x1000
//#define T_DMA_IRQ		0x0800
//#define IIS_AC_PIN_SEL		0x0100
//#define FIFO_TH			0x0080
//#define ADC_EN			0x0010
//#define M80_EN			0x0008
//#define ACLINK_EN		0x0004
//#define IIS_EN			0x0002

/* bit definition of REG_ACTL_RESET register */
#define W5691_PLAY		0x20000
#define ACTL_RESET_BIT		0x10000
#define RECORD_RIGHT_CHANNEL	0x08000
#define RECORD_LEFT_CHANNEL	0x04000
#define PLAY_RIGHT_CHANNEL	0x02000
#define PLAY_LEFT_CHANNEL	0x01000
#define DAC_PLAY		0x00800
#define ADC_RECORD		0x00400
#define M80_PLAY		0x00200
#define AC_RECORD		0x00100
#define AC_PLAY			0x00080
#define IIS_RECORD		0x00040
#define IIS_PLAY		0x00020
#define DAC_RESET		0x00010
#define ADC_RESET		0x00008
#define M80_RESET		0x00004
//#define AC_RESET		0x00002
#define IIS_RESET		0x00001

/* bit definition of REG_ACTL_ACCON register */
#define AC_BCLK_PU_EN		0x20
#define AC_R_FINISH		0x10
#define AC_W_FINISH		0x08
#define AC_W_RES		0x04
#define AC_C_RES		0x02

/* bit definition of ACTL_RSR register */
//#define R_FIFO_EMPTY		0x04
#define R_DMA_END_IRQ		0x02
#define R_DMA_MIDDLE_IRQ	0x01

/* bit definition of ACTL_PSR register */
//#define P_FIFO_EMPTY		0x04
//#define P_DMA_END_IRQ		0x02
//#define P_DMA_MIDDLE_IRQ	0x01

/* bit definition of ACTL_ACOS0 register */
#define SLOT1_VALID		0x01
#define SLOT2_VALID		0x02
#define SLOT3_VALID		0x04
#define SLOT4_VALID		0x08
#define VALID_FRAME		0x10

/* bit definition of ACTL_ACOS1 register */
#define R_WB			0x80
#endif

/*----- bit definition of REG_ACTL_IISCON register -----*/
#define IIS					0x0
#define MSB_Justified		0x0008
#define SCALE_1				0x0
#define SCALE_2				0x10000
#define SCALE_3				0x20000
#define SCALE_4				0x30000
#define SCALE_5				0x40000
#define SCALE_6				0x50000
#define SCALE_7				0x60000
#define SCALE_8				0x70000
#define SCALE_10			0x90000
#define SCALE_12			0xB0000
#define SCALE_14			0xD0000
#define SCALE_16			0xF0000
#define FS_384				0x20
#define FS_256				0x0
#define BCLK_32				0x00
#define BCLK_48				0x40

/* bit definition of L3DATA register */
#define EX_256FS 		0x20		/*-- system clock --*/
#define EX_384FS 		0x10		
#define EX_IIS			0x00		/*-- data input format  --*/
#define EX_MSB			0x08
#define EX_1345ADDR 	0x14		//The address of the UDA1345TS
#define EX_STATUS		0x02		//data transfer type (STATUS)
#define EX_DATA			0x00		//data transfer type (DATA)
#define EX_ADC_On		0xC2		//turn on the ADC
#define EX_DAC_On		0xC1		//turn on the DAC

#define W55FA93_AUDIO_SAMPLECLK		0x00
#define W55FA93_AUDIO_CLKDIV		0x01

#define CODEC_READY		0x10
#define RESET_PRSR		0x00

#endif /*end _W55FA93_I2S_H */
