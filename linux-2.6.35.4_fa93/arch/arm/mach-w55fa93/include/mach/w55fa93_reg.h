/* linux/include/asm-arm/arch-w55fa93/w55fa93_reg.h
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
 *   2009/10/13     CCChang add this file for nuvoton W55FA93 MCU ip REG.
 */

#ifndef _W55FA93_REG_H
#define _W55FA93_REG_H

#include <mach/map.h>

// Define one bit mask
#define BIT0	0x00000001
#define BIT1	0x00000002
#define BIT2	0x00000004
#define BIT3	0x00000008
#define BIT4	0x00000010
#define BIT5	0x00000020
#define BIT6	0x00000040
#define BIT7	0x00000080
#define BIT8	0x00000100
#define BIT9	0x00000200
#define BIT10	0x00000400
#define BIT11	0x00000800
#define BIT12	0x00001000
#define BIT13	0x00002000
#define BIT14	0x00004000
#define BIT15	0x00008000
#define BIT16	0x00010000
#define BIT17	0x00020000
#define BIT18	0x00040000
#define BIT19	0x00080000
#define BIT20	0x00100000
#define BIT21	0x00200000
#define BIT22	0x00400000
#define BIT23	0x00800000
#define BIT24	0x01000000
#define BIT25	0x02000000
#define BIT26	0x04000000
#define BIT27	0x08000000
#define BIT28	0x10000000
#define BIT29	0x20000000
#define BIT30	0x40000000
#define BIT31	0x80000000

// Define bits mask
#define NVTBIT(start,end) ((0xFFFFFFFF >> (31 - start)) & (0xFFFFFFFF << end))


#define GCR_BASE	W55FA93_VA_GCR		/* GCR */

#define REG_CHIPID	(GCR_BASE+0x00)	// R	Chip Identification Register
	#define CHIP_VER		NVTBIT(27, 24)	// Chip Version
	#define CHIP_ID			NVTBIT(23, 0)	// Chip Identification

#define REG_CHIPCFG	(GCR_BASE+0x04)	// R/W	Chip Power-On Configuration Register
	#define UDFMODE			NVTBIT(27,24)	// User-Defined Power-On setting mode
	#define MAPSDR			BIT16		// Map SDRAM
	#define USBDEV			BIT7 		// USB Host Selection
	#define CLK_SRC			BIT6		// System Clock Source Selection		
	#define SDRAMSEL		NVTBIT(5, 4)	// SDRAM Type Selection
	#define COPMODE			NVTBIT(3, 0)	// Chip Operation Mode Selection

#define REG_AHBCTL	(GCR_BASE+0x10)	// R/W	AHB Bus Arbitration Control Register
	#define IPACT			BIT5 		// Interrupt Active Status
	#define IPEN			BIT4 		// CPU Priority Raising Enable during Interrupt Period
	#define PRTMOD1			BIT1 		// Priority Mode Control 1
	#define PRTMOD0			BIT0 		// Priority Mode Control 0

#define REG_AHBIPRST	(GCR_BASE+0x14)	// R/W	AHB IP Reset Control Resister
	#define JPGRST			BIT17 		// JPG Reset
	#define BLTRST			BIT16		// 2D Accelerator Reset	 
	#define AESRST			BIT15 		// AES Reset
	#define FSCRST			BIT14 		// FSC Reset
	#define GE4PRST			BIT13 		// GE4P Reset
	#define GPURST			BIT12 		// GPU Reset
	#define CAPRST			BIT11 		// CAP Reset
	#define VPOSTRST		BIT10 		// VPOST Reset
	#define I2SRST			BIT9 		// I2S Reset
	#define SPURST			BIT8 		// SPU Reset
	#define UHCRST			BIT7 		// UHC Reset
	#define UDCRST			BIT6 		// UDC Reset
	#define SICRST			BIT5 		// SIC Reset	
	#define TICRST			BIT4 		// TIC Reset
	#define EDMARST			BIT3 		// EDMA Reset
	#define SRAMRST			BIT2 		// SRAM Reset
	#define SDICRST			BIT0 		// SDIC Reset
	
#define REG_APBIPRST	(GCR_BASE+0x18)	// R/W	APB IP Reset Control Resister
	#define ADCRST			BIT14		// ADC Reset		
	#define SPI1RST			BIT13 		// SPI 1 Reset
	#define SPI0RST			BIT12 		// SPI 0 Reset
	#define PWMRST			BIT10 		// PWM Reset
	#define I2CRST			BIT8 		// I2C Reset		
		
	#define UART1RST		BIT7 		// UART 1 Reset
	#define UART0RST		BIT6 		// UART 0 Reset
	#define TMR1RST			BIT5 		// TMR1 Reset
	#define TMR0RST			BIT4 		// TMR0 Reset
	#define WDTRST			BIT3 		// WDT Reset
	#define RTCRST			BIT2 		// RTC Reset
	#define GPIORST			BIT1 		// RTC Reset
	#define AICRST			BIT0 		// AIC Reset
	
#define REG_MISCR	(GCR_BASE+0x20)	// R/W	Miscellaneous Control Register
	#define LVR_RDY			BIT9 		// Low Voltage Reset Function Ready
	#define LVR_EN			BIT8 		// Low Voltage Reset Function Enable
	#define CPURSTON		BIT1 		// CPU always keep in reset state for TIC 
	#define CPURST		 	BIT0 		// CPU one shutte reset.

#define REG_SDRBIST	(GCR_BASE+0x24)	// R/W	Power Management Control Register
	#define TEST_BUSY		BIT31 		// Test BUSY
	#define CON_BUSY		BIT30		// Connection Test Busy
	#define SDRBIST_BUSY		BIT29		// BIST Test Busy
	#define TEST_FAIL		BIT28 		// Test Failed
	#define CON_FAIL		BIT27		// Connection Test Failed
	#define SDRBIST_FAIL		BIT26 		// BIST Test Failed	

#define REG_CRBIST	(GCR_BASE+0x28)	// R/W	Cache RAM BIST Control & Status Register
	#define ICV_F			BIT29		// I-Cache Valid RAM BIST Failed Flag
	#define ICT_F			BIT28		// I-Cache Tag RAM BIST Failed Flag
	#define ICD3_F			BIT27		// I-Cache Data RAM 3 BIST Failed Flag	
	#define ICD2_F			BIT26		// I-Cache Data RAM 2 BIST Failed Flag
	#define ICD1_F			BIT25		// I-Cache Data RAM 1 BIST Failed Flag
	#define ICD0_F			BIT24		// I-Cache Data RAM 0 BIST Failed Flag
	#define MMU_F			BIT23		// MMU RAM BIST Failed Flag
	#define DCDIR_F			BIT22		// D-Cache Dirty RAM BIST Failed Flag
	#define DCV_F			BIT21		// D-Cache Valid RAM BIST Failed Flag	
	#define DCT_F			BIT20		// D-Cache Tag RAM BIST Failed Flag	
	#define DCD3_F			BIT19		// D-Cache Data RAM 3 BIST Failed Flag
	#define DCD2_F			BIT18		// D-Cache Data RAM 2 BIST Failed Flag
	#define DCD1_F			BIT17		// D-Cache Data RAM 1 BIST Failed Flag
	#define DCD0_F			BIT16		// D-Cache Data RAM 0 BIST Failed Flag
	#define BISTEN			BIT15		// Cache RAM BIST Test Enable

	#define ICV_R			BIT13		// I-Cache Valid RAM BIST Running Flag
	#define ICT_R			BIT12		// I-Cache Tag RAM BIST Running Flag
	#define ICD3_R			BIT11		// I-Cache Data RAM 3 BIST Running Flag
	#define ICD2_R			BIT10		// I-Cache Data RAM 2 BIST Running Flag
	#define ICD1_R			BIT9		// I-Cache Data RAM 1 BIST Running Flag
	#define ICD0_R			BIT8		// I-Cache Data RAM 0 BIST Running Flag	
	#define MMU_R			BIT7		// MMU RAM BIST Running Flag
	#define DCDIR_R			BIT6		// D-Cache Dirty RAM BIST Running Flag
	#define DCV_R			BIT5		// D-Cache Valid RAM BIST Running Flag	
	#define DCT_R			BIT4		// D-Cache Tag RAM BIST Running Flag
	#define DCD3_R			BIT3		// D-Cache Data RAM 3 BIST Running Flag
	#define DCD2_R			BIT2		// D-Cache Data RAM 2 BIST Running Flag
	#define DCD1_R			BIT1		// D-Cache Data RAM 1 BIST Running Flag
	#define DCD0_R			BIT0		// D-Cache Data RAM 0 BIST Running Flag	

#define REG_EDSSR	(GCR_BASE+0x2C)	// R/W	EDMA Service Selection Control Register
	#define CH1_RXSEL		NVTBIT(2, 0)	// EDMA Channel 1 Rx Selection
	#define CH2_RXSEL		NVTBIT(6, 4)	// EDMA Channel 2 Rx Selection
	#define CH3_RXSEL		NVTBIT(10, 8)	// EDMA Channel 3 Rx Selection
	#define CH4_RXSEL		NVTBIT(14, 12)	// EDMA Channel 4 Rx Selection
	#define CH1_TXSEL		NVTBIT(18, 16)	// EDMA Channel 1 Tx Selection
	#define CH2_TXSEL		NVTBIT(22, 20)	// EDMA Channel 2 Tx Selection
	#define CH3_TXSEL		NVTBIT(26, 24)	// EDMA Channel 3 Tx Selection
	#define CH4_TXSEL		NVTBIT(30, 28)	// EDMA Channel 4 Tx Selection
	
#define REG_MISSR	(GCR_BASE+0x30)	// R/W	Miscellaneous Status Register
	#define KPI_WS			BIT31		// KPI Wake-Up Status
	#define ADC_WS			BIT30		// ADC Wake-Up Status
	#define UHC_WS			BIT29		// UHC Wake-Up Status
	#define UDC_WS			BIT28		// UDC Wake-Up Status
	#define UART_WS			BIT27		// UART Wake-Up Status
	#define SDH_WS			BIT26		// SDH Wake-Up Status
	#define RTC_WS			BIT25		// RTC Wake-Up Status
	#define GPIO_WS			BIT24		// GPIO Wake-Up Status
	#define KPI_WE			BIT23		// KPI Wake-Up Enable
	#define ADC_WE			BIT22		// ADC Wake-Up Enable
	#define UHC_WE			BIT21		// UHC Wake-Up Enable
	#define UDC_WE			BIT20		// UDC Wake-Up Enable
	#define UART_WE			BIT19		// UART Wake-Up Enable
	#define SDH_WE			BIT18		// SDH Wake-Up Enable
	#define RTC_WE			BIT17		// RTC Wake-Up Enable
	#define GPIO_WE			BIT16		// GPIO Wake-Up Enable
	#define CPU_RST			BIT4		// CPU Reset Active Status
	#define WDT_RST			BIT3		// WDT Reset Active Status
	#define KPI_RST			BIT2		// KPI Reset Active Status
	#define LVR_RST			BIT1		// LVR Reset Active Status
	#define EXT_RST			BIT0		// External Reset Pin Active Status
	
#define REG_OTP_CTRL	(GCR_BASE+0x40)	// R/W	OTP Control Register
	#define OTP_STAT		NVTBIT(25, 24)	// OTP Burned Status
	#define IBR4_STAT		NVTBIT(23, 22)	// OTP_IBR4 Burned Status
	#define IBR3_STAT		NVTBIT(21, 20)	// OTP_IBR3 Burned Status
	#define IBR2_STAT		NVTBIT(19, 18)	// OTP_IBR2 Burned Status
	#define IBR1_STAT		NVTBIT(17, 16)	// OTP_IBR1 Burned Status			
	#define TEST_OK			BIT4		// MARGIN Read Mode Test OK Flag
	#define MARGIN			BIT1		// OTP MARGIN Read Mode
	#define OTPRD_EN		BIT0		// OTP Read Enable
	
#define REG_OTP_PROG	(GCR_BASE+0x44)	// R/W	OTP Program Control Register
	#define BURN_CYC		NVTBIT(29, 16)	// OTP Program Cycle	
	#define OTP_EN			NVTBIT(12, 4)	// OTP Enable		
	#define VPP_STA			BIT1		// VPP State Indicator
	#define BURN_EN			BIT0		// OTP Program Enable
	
#define REG_OTP_DIS	(GCR_BASE+0x48)	// R/W	OTP Disable Register
	#define CNTRL_DIS		BIT16		// OTP Register Control Disable

#define REG_OTP_KEY1	(GCR_BASE+0x50)	// R/W	OTP Key 1 Register

#define REG_OTP_KEY2	(GCR_BASE+0x54)	// R/W	OTP Key 2 Register	

#define REG_OTP_KEY3	(GCR_BASE+0x58)	// R/W	OTP Key 2 Register	

#define REG_OTP_KEY4	(GCR_BASE+0x5C)	// R/W	OTP Key 2 Register	

#define REG_OTP_IBR1	(GCR_BASE+0x60)	// R/W	OTP IBR Option 1 Register

#define REG_OTP_IBR2	(GCR_BASE+0x64)	// R/W	OTP IBR Option 2 Register

#define REG_OTP_IBR3	(GCR_BASE+0x68)	// R/W	OTP IBR Option 3 Register

#define REG_OTP_IBR4	(GCR_BASE+0x6C)	// R/W	OTP IBR Option 4 Register

#define REG_OTP_CID	(GCR_BASE+0x70)	// R/W	OTP IBR Option 4 Register
	#define UDOption		NVTBIT(31, 8)	// User Defined Option
	#define OTP_CID_CHIP_VER	NVTBIT(7,  4)	// Chip version
	#define CHIP_COD		NVTBIT(29, 28)	// Chip mode
				
#define REG_GPAFUN	(GCR_BASE+0x80)	// R/W	Multi Function Pin Control Register 0
	#define MF_GPA15		NVTBIT(31, 30)	// GPA[15] Multi Function
	#define MF_GPA14		NVTBIT(29, 28)	// GPA[14] Multi Function
	#define MF_GPA13		NVTBIT(27, 26)	// GPA[13] Multi Function	
	#define MF_GPA12		NVTBIT(25, 24)	// GPA[12] Multi Function	
	#define MF_GPA11		NVTBIT(23, 22)	// GPA[11] Multi Function
	#define MF_GPA10		NVTBIT(21, 20)	// GPA[10] Multi Function
	#define MF_GPA9			NVTBIT(19, 18)	// GPA[9] Multi Function
	#define MF_GPA8			NVTBIT(17, 16)	// GPA[8] Multi Function
	#define MF_GPA7			NVTBIT(15, 14)	// GPA[7] Multi Function 
	#define MF_GPA6			NVTBIT(13, 12)	// GPA[6] Multi Function
	#define MF_GPA5			NVTBIT(11, 10)	// GPA[5] Multi Function
	#define MF_GPA4			NVTBIT(9, 8)	// GPA[4] Multi Function
	#define MF_GPA3			NVTBIT(7, 6)	// GPA[3] Multi Function
	#define MF_GPA2			NVTBIT(5, 4)	// GPA[2] Multi Function
	#define MF_GPA1			NVTBIT(3, 2)	// GPA[1] Multi Function
	#define MF_GPA0			NVTBIT(1, 0)	// GPA[0] Multi Function

#define REG_GPBFUN	(GCR_BASE+0x84)	// R/W	Multi Function Pin Control Register 0
	#define MF_GPB15		NVTBIT(31, 30)	// GPB[15] Multi Function
	#define MF_GPB14		NVTBIT(29, 28)	// GPB[14] Multi Function
	#define MF_GPB13		NVTBIT(27, 26)	// GPB[13] Multi Function	
	#define MF_GPB12		NVTBIT(25, 24)	// GPB[12] Multi Function	
	#define MF_GPB11		NVTBIT(23, 22)	// GPB[11] Multi Function
	#define MF_GPB10		NVTBIT(21, 20)	// GPB[10] Multi Function
	#define MF_GPB9			NVTBIT(19, 18)	// GPB[9] Multi Function
	#define MF_GPB8			NVTBIT(17, 16)	// GPB[8] Multi Function
	#define MF_GPB7			NVTBIT(15, 14)	// GPB[7] Multi Function 
	#define MF_GPB6			NVTBIT(13, 12)	// GPB[6] Multi Function
	#define MF_GPB5			NVTBIT(11, 10)	// GPB[5] Multi Function
	#define MF_GPB4			NVTBIT(9, 8)	// GPB[4] Multi Function
	#define MF_GPB3			NVTBIT(7, 6)	// GPB[3] Multi Function
	#define MF_GPB2			NVTBIT(5, 4)	// GPB[2] Multi Function
	#define MF_GPB1			NVTBIT(3, 2)	// GPB[1] Multi Function
	#define MF_GPB0			NVTBIT(1, 0)	// GPB[0] Multi Function

#define REG_GPCFUN	(GCR_BASE+0x88)	// R/W	Multi Function Pin Control Register 0
	#define MF_GPC15		NVTBIT(31, 30)	// GPC[15] Multi Function
	#define MF_GPC14		NVTBIT(29, 28)	// GPC[14] Multi Function
	#define MF_GPC13		NVTBIT(27, 26)	// GPC[13] Multi Function	
	#define MF_GPC12		NVTBIT(25, 24)	// GPC[12] Multi Function	
	#define MF_GPC11		NVTBIT(23, 22)	// GPC[11] Multi Function
	#define MF_GPC10		NVTBIT(21, 20)	// GPC[10] Multi Function
	#define MF_GPC9			NVTBIT(19, 18)	// GPC[9] Multi Function
	#define MF_GPC8			NVTBIT(17, 16)	// GPC[8] Multi Function
	#define MF_GPC7			NVTBIT(15, 14)	// GPC[7] Multi Function 
	#define MF_GPC6			NVTBIT(13, 12)	// GPC[6] Multi Function
	#define MF_GPC5			NVTBIT(11, 10)	// GPC[5] Multi Function
	#define MF_GPC4			NVTBIT(9, 8)	// GPC[4] Multi Function
	#define MF_GPC3			NVTBIT(7, 6)	// GPC[3] Multi Function
	#define MF_GPC2			NVTBIT(5, 4)	// GPC[2] Multi Function
	#define MF_GPC1			NVTBIT(3, 2)	// GPC[1] Multi Function
	#define MF_GPC0			NVTBIT(1, 0)	// GPC[0] Multi Function
	
#define REG_GPDFUN	(GCR_BASE+0x8C)	// R/W	Multi Function Pin Control Register 0
	#define MF_GPD15		NVTBIT(31, 30)	// GPD[15] Multi Function
	#define MF_GPD14		NVTBIT(29, 28)	// GPD[14] Multi Function
	#define MF_GPD13		NVTBIT(27, 26)	// GPD[13] Multi Function	
	#define MF_GPD12		NVTBIT(25, 24)	// GPD[12] Multi Function	
	#define MF_GPD11		NVTBIT(23, 22)	// GPD[11] Multi Function
	#define MF_GPD10		NVTBIT(21, 20)	// GPD[10] Multi Function
	#define MF_GPD9			NVTBIT(19, 18)	// GPD[9] Multi Function
	#define MF_GPD8			NVTBIT(17, 16)	// GPD[8] Multi Function
	#define MF_GPD7			NVTBIT(15, 14)	// GPD[7] Multi Function 
	#define MF_GPD6			NVTBIT(13, 12)	// GPD[6] Multi Function
	#define MF_GPD5			NVTBIT(11, 10)	// GPD[5] Multi Function
	#define MF_GPD4			NVTBIT(9, 8)	// GPD[4] Multi Function
	#define MF_GPD3			NVTBIT(7, 6)	// GPD[3] Multi Function
	#define MF_GPD2			NVTBIT(5, 4)	// GPD[2] Multi Function
	#define MF_GPD1			NVTBIT(3, 2)	// GPD[1] Multi Function
	#define MF_GPD0			NVTBIT(1, 0)	// GPD[0] Multi Function

#define REG_GPEFUN	(GCR_BASE+0x90)	// R/W	Multi Function Pin Control Register 0
	#define MF_GPE15		NVTBIT(31, 30)	// GPE[15] Multi Function
	#define MF_GPE14		NVTBIT(29, 28)	// GPE[14] Multi Function
	#define MF_GPE13		NVTBIT(27, 26)	// GPE[13] Multi Function	
	#define MF_GPE12		NVTBIT(25, 24)	// GPE[12] Multi Function	
	#define MF_GPE11		NVTBIT(23, 22)	// GPE[11] Multi Function
	#define MF_GPE10		NVTBIT(21, 20)	// GPE[10] Multi Function
	#define MF_GPE9			NVTBIT(19, 18)	// GPE[9] Multi Function
	#define MF_GPE8			NVTBIT(17, 16)	// GPE[8] Multi Function
	#define MF_GPE7			NVTBIT(15, 14)	// GPE[7] Multi Function 
	#define MF_GPE6			NVTBIT(13, 12)	// GPE[6] Multi Function
	#define MF_GPE5			NVTBIT(11, 10)	// GPE[5] Multi Function
	#define MF_GPE4			NVTBIT(9, 8)	// GPE[4] Multi Function
	#define MF_GPE3			NVTBIT(7, 6)	// GPE[3] Multi Function
	#define MF_GPE2			NVTBIT(5, 4)	// GPE[2] Multi Function
	#define MF_GPE1			NVTBIT(3, 2)	// GPE[1] Multi Function
	#define MF_GPE0			NVTBIT(1, 0)	// GPE[0] Multi Function

#define REG_MISFUN	(GCR_BASE+0x94)	// R/W	Miscellaneous Multi Function Control Register
	#define MF_NCS0			NVTBIT(5, 4)	// MF_NCS0_ Multi Function
	#define MF_EWAIT		NVTBIT(3, 2)	// MF_EWAIT_ Multi Function		
	#define MF_ECS1			NVTBIT(1, 0)	// MF_ECS1_ Multi Function		
	
#define REG_MISCPCR	(GCR_BASE+0xA0)	// R/W	Miscellaneous Pin Control Register
	#define SL_MD			BIT7		// MD Pin Slew Rate Control
	#define SL_MA			BIT6		// MA Pin Slew Rate Control	
	#define SL_MCTL			BIT5		// Memory I/F Control Pin Slew Rate Control		
	#define SL_MCLK			BIT4		// MCLK Pin Rate Control
	#define DS_MD			BIT3		// MD Pins Driving Strength Control	
	#define DS_MA			BIT2		// MA Pins Driving Strength Control	
	#define DS_MCTL			BIT1		// MCTL Pins Driving Strength Control	
	#define DS_MCLK			BIT0		// MCLK Pins Driving Strength Control			

#define REG_PWRCON	(GCR_BASE+0x200)
	#define PRE_SCALAR		NVTBIT(23, 8)		// Pre-Scalar counter
	#define UP2HCLK3X		BIT5				// Ratio of CPU to HCLK
	#define SEN_OFF_ST		BIT4				// Sensor clock level if clock off state
	#define INT_EN			BIT3				// Power On Interrupt Enable
	#define INTSTS			BIT2				// Power Down interrupt status
	#define XIN_CTL			BIT1				// Crystal pre-divide control for Wake-up from power down mode
	#define XTAL_EN			BIT0				// Crystal (Power Down) Control
#define REG_AHBCLK	(GCR_BASE+0x204)
	#define ADO_CKE			BIT30					// Audio DAC Engine Clock Enable Control0 = Disable1 = Enable
	#define SEN_CKE			BIT29					// Sensor Interface Clock Enable Control0 = Disable1 = Enable
	#define CAP_CKE			BIT28					// Capture Clock Enable Control (Also is Capture engine clock enable control)0 = Disable1 = Enable
	#define VPOST_CKE		BIT27					// VPOST Clock Enable Control (Also is VPOST engine clock enable control)0 = Disable1 = Enable
	#define I2S_CKE			BIT26					// I2S Controller Clock Enable Control0 = Disable1 = Enable
	#define SPU_CKE			BIT25					// SPU Clock Enable Control0 = Disable1 = Enable
	#define HCLK4_CKE		BIT24					// HCLK4 Clock Enable Control0 = Disable1 = Enable
	#define SD_CKE			BIT23					// SD Card Controller Engine Clock Enable Control0 = Disable1 = ENable
	#define NAND_CKE		BIT22					// NAND Controller Clock Enable Control0 = Disable1 = ENable
	#define SIC_CKE			BIT21					// SIC Clock Enable Control0 = Disable1 = ENable
	#define GPU_CKE			BIT20					// Graphic Processing Unit Clock Enable Control0 = Disable1 = ENable
	#define GE4P_CKE		BIT19					// GE4P Clock Enable Control0 = Disable1 = ENable
	#define USBD_CKE		BIT18					// USB Device Clock Enable Control0 = Disable1 = Enable
	#define USBH_CKE		BIT17					// USB Host Controller Clock Enable Control0 = Disable1 = Enable
	#define HCLK3_CKE		BIT16					// HCLK3 Clock Enable Control.0 = Disable1 = Enable
	#define DES_CKE			BIT15					// DES Codec Clock Enable Control
	#define EDMA4_CKE		BIT14					// EDMA Controller Channel 4 Clock Enable Control
	#define EDMA3_CKE		BIT13					// EDMA Controller Channel 4 Clock Enable Control
	#define EDMA2_CKE		BIT12					// EDMA Controller Channel 4 Clock Enable Control
	#define EDMA1_CKE		BIT11					// EDMA Controller Channel 4 Clock Enable Control
	#define EDMA0_CKE		BIT10					// EDMA Controller Channel 4 Clock Enable Control			
	#define EBI_CKE		 	BIT9					// EBI	Clock Enable Control0 = Disable1 = Enable
	#define HCLK1_CKE		BIT8					// HCLK1 Clock Enable Control.0 = Disable1 = Enable
	#define JPG_CKE			BIT7					// JPEG Clock Enable	
	#define FSC_CKE			BIT6					// FSC Clock Enable
	#define BLT_CKE		    BIT5					// GE2D Clock Enable Control0 = Disable1 = Enable
	#define DRAM_CKE		BIT4					// SDRAM and SDRAM Controller Clock Enable Control.0 = Disable1 = Enable
	#define SRAM_CKE		BIT3					// SRAM Controller Clock Enable Control.0 = Disable1 = Enable
	#define HCLK_CKE		BIT2					// HCLK Clock Enable Control. (This clock is used for DRAM controller, SRAM controller and AHB-to-AHB bridge)0 = Disable1 = Enable
	#define APBCLK_CKE		BIT1					// APB Clock Enable Control.0 = Disable1 = Enable
	#define CPU_CKE	 		BIT0		

#define REG_APBCLK	(GCR_BASE+0x208)
	#define KPI_CKE			BIT25 					// KPI Clock Enable Control
	#define	TIC_CKE			BIT24 					// TIC Clock Enable
	#define WDCLK_CKE		BIT15					// Watch Dog Clock Enable Control (Also is Watch Dog engine clock enable control)
	#define TMR1_CKE		BIT9					// Timer1 Clock Enable Control0 = Disable1 = Enable
	#define TMR0_CKE		BIT8					// Timer0 Clock Enable Control0 = Disable1 = Enable
	#define SPIMS1_CKE		BIT7					// SPIM (Master Only) Clock Enable Control0 = Disable1 = Enable
	#define SPIMS0_CKE		BIT6					// SPIMS (Master / Slave) Clock Enable Control0 = Disable1 = Enable
	#define PWM_CKE			BIT5					// PWM Clock Enable Control0 = Disable1 = Enable
	#define UART1_CKE		BIT4					// UART1 Clock Enable Control0 = Disable1 = Enable
	#define UART0_CKE		BIT3					// UART0 Clock Enable Control0 = Disable1 = Enable		
	#define RTC_CKE			BIT2					// RTC Clock Enable Control (NOT X32K clock enable control)0 = Disable1 = Enable
	#define I2C_CKE			BIT1					// I2C Clock Enable Control0 = Disable1 = Enable
	#define ADC_CKE			BIT0					// ADC Clock Enable Control (Also is ADC engine clock enable control)0 = Disable1 = Enable

#define REG_CLKDIV0	(GCR_BASE+0x20C)
	#define SENSOR_N1		NVTBIT(27, 24)			// Sensor clock divide number from sensor clock source
	#define KPI_N1			NVTBIT(23, 21)			// KPI Engine Clock Divider Bits [6:4]
	#define SENSOR_S		NVTBIT(20, 19)			// Sensor clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define SENSOR_N0		NVTBIT(18, 16)			// Sensor clock pre-divider number from Sensor clock source if Sensor clock source select is APLL or UPLL
	#define KPI_N0			NVTBIT(15, 12)			// KPI Engine Clock Divider Bits [3:0]
	
	#define SYSTEM_N1		NVTBIT(11, 8)			// SYSTEM clock divide number from system clock source
	#define KPI_S			BIT5						// KPI Engine Clock Source Selection
	#define SYSTEM_S		NVTBIT(4, 3)			// System clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define SYSTEM_N0		NVTBIT(2, 0)			// SYSTEM clock pre-divider number from system clock source if System clock source select is APLL or UPLL

#define REG_CLKDIV1	(GCR_BASE+0x210)
	#define ADO_N1			NVTBIT(31, 24)			// Audio DAC engine clock divide number from Audio DAC engine clock source
	#define ADO_S			NVTBIT(20, 19)			// Audio DAC engine clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define ADO_N0			NVTBIT(18, 16)			// Audio DAC engine clock pre-divide number
	#define VPOST_N1		NVTBIT(15, 8)			// VPOST engine clock divide number from VPOST engine clock source
	#define VPOST_S			NVTBIT(4, 3)			// VPOST engine clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define VPOST_N0		NVTBIT(2, 0)			// VPOST engine clock pre-divide number

#define REG_CLKDIV2	(GCR_BASE+0x214)
	#define SD_N1			NVTBIT(31, 24)			// SD engine clock divide number from SD engine clock source
	#define SD_S			NVTBIT(20, 19)			// SD engine clock source select  00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define SD_N0			NVTBIT(18, 16)			// SD engine clock pre-divide number
	#define USB_N1			NVTBIT(11, 8)			// USB engine clock divide number from USB engine clock source
	#define USB_S			NVTBIT(4, 3)			// USB engine clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define USB_N0			NVTBIT(2, 0)			// USB engine clock Pre-divide number	

#define REG_CLKDIV3	(GCR_BASE+0x218)
	#define ADC_N1			NVTBIT(31, 24) 			// ADC engine clock divide number from ADC engine clock source
	#define ADC_S			NVTBIT(20, 19) 			// ADC engine clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define ADC_N0			NVTBIT(18, 16) 			// ADC engine clock pre-divide number from ADC engine clock source
	#define UART1_N1		NVTBIT(15, 13) 			// UART1 engine clock divide number from UART1 engine clock source
	#define UART1_S			NVTBIT(12, 11) 			// UART1 engine clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define UART1_N0		NVTBIT(10, 8)  			// UART1 engine clock pre-divide number from UART1 engine clock source
	#define UART0_N1		NVTBIT(7, 5) 				// UART0 engine clock divide number from UART1 engine clock source
	#define UART0_S			NVTBIT(4, 3) 				// UART0 engine clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define UART0_N0		NVTBIT(2, 0)   			// UART0 engine clock pre-divide number from UART1 engine clock source

#define REG_CLKDIV4	(GCR_BASE+0x21C)
	#define JPG_N			NVTBIT(26, 24)			// JPG 	engine clock divide number from HCLK3
	#define GPIO_N			NVTBIT(23, 17)			// GPIO engine clock divide number from GPIO engine clock source
	#define GPIO_S			BIT16 					// GPIO engine clock source select0 = XIN1 = X32K
	#define CAP_N			NVTBIT(14, 12)			// Capture engine clock divide number from HCLK4 clock.Engine Clock frequency = HCLK4 / (CAP_N + 1)
	#define APB_N			NVTBIT(11, 8) 			// APB clock divide number from HCLK1 clock. The HCLK1 clock frequency is the lower of system clock divided by 2 or the CPU clockThe APB clock frequency = (HCLK1 frequency) / (APB_N + 1)
	#define HCLK234_N		NVTBIT(7, 	04)			// HCLK2, HCLK3 and HCLK4 clock divide number from HCLK clock. The HCLK clock frequency is the system clock frequency divided by two.The HCLK2,3,4 clock frequency = (HCLK frequency) / (HCLK234_N + 1)
	#define CPU_N			NVTBIT(3, 	0)	 		// CPU clock divide number from System clock.The CPU clock frequency = (System frequency) / (CPU_N + 1)

#define REG_APLLCON	(GCR_BASE+0x220)
#define REG_UPLLCON	(GCR_BASE+0x224)
	#define OE				BIT18					// PLL OE (FOUT enable) pin Control
	#define BP				BIT17					// PLL Bypass Control
	#define PD				BIT16					// Power Down Mode	
	#define OUT_DV			NVTBIT(15,14)			// PLL Output Divider Control Pins (PLL_OD[1:0])
	#define IN_DV			NVTBIT(13,9)			// PLL Input Divider Control Pins (PLL_R[4:0])
	#define FB_DV			NVTBIT(8,0)				// PLL Feedback Divider Control Pins (PLL_F[6:0])	

#define REG_CLK_TREG	(GCR_BASE+0x230)


#define SDIC_BASE	W55FA93_VA_SDIC		/* SDIC */

/*
	SDRAM controller registers	
*/
#define REG_SDOPM 	(SDIC_BASE + 0x00)	// R/W	SDRAM Controller Operation Mode Control Register
		#define RD2WR_CTL	BIT20			// Read-To-Write Turn Around Control
		#define OEDELAY		BIT19			// Output Enable Delay Half MCLK
		#define LOWFREQ		BIT18			// Low Frequency Mode
		#define PREACTBNK     	BIT17			// Pre-Active Bank
		#define AUTOPDN       	BIT16			// Auto Power Down Mode
		#define RDBUFTH       	NVTBIT(10, 8)		// The AHB read SDRAM read buffer threshold control 
		#define SD_TYPE       	NVTBIT(6, 5)		// SDRAM Type
		#define PCHMODE      	BIT4				// SDRAM Type
		#define OPMODE        	BIT3				// Open Page Mode
		#define MCLKMODE      	BIT2				// Auto Pre-Charge Mode
		#define DRAM_EN      	BIT1				// SDRAM Controller Enable
		
#define REG_SDCMD 	(SDIC_BASE + 0x04)	// R/W	SDRAM Command Register
		#define AUTOEXSELFREF	BIT5			// Auto Exit Self-Refresh
		#define SELF_REF       	BIT4				// Self-Refresh Command
		#define REF_CMD       	BIT3				// Auto Refresh Command
		#define PALL_CMD       	BIT2				// Pre-Charge All Bank Command
		#define CKE_H           	BIT1    			// CKE High
		#define INITSTATE		BIT0				// Initial State
#define REG_SDREF 	(SDIC_BASE + 0x08)	// R/W	SDRAM Controller Refresh Control Register
		#define REF_EN		BIT15			// Refresh Period Counter Enable
		#define REFRATE		NVTBIT(14, 0)		// Refresh Count Value
#define REG_SDSIZE0	(SDIC_BASE + 0x10)	// R/W	SDRAM 0 Size Register
#define REG_SDSIZE1	(SDIC_BASE + 0x14)	// R/W	SDRAM 0 Size Register
		#define BASADDR		NVTBIT(28, 21) 	// Base Address
		#define BUSWD		BIT3				// SDRAM Data Bus width
		#define DRAMSIZE		NVTBIT(2, 0)		// Size of SDRAM Device

#define REG_SDMR		(SDIC_BASE + 0x18)	// R/W	SDRAM Mode Register
		#define SDMR_CONFIGURE	NVTBIT(17, 7) // SDRAM Dependent Configuration
		#define LATENCY			NVTBIT(6, 4) 	// CAS Latency
		#define BRSTTYPE			BIT3				// Burst Type
		#define BRSTLENGTH		NVTBIT(2, 0) 	// Burst Length
#define REG_SDEMR	(SDIC_BASE + 0x1C)	// R/W	SDRAM Extended Mode Register
		#define SDEMR_CONFIGURE	NVTBIT(17, 2) // SDRAM Dependent Configuration
		#define DRVSTRENGTH	BIT1 			// Output Drive Strength
		#define DLLEN			BIT0				// DLL Enable
#define REG_SDEMR2	(SDIC_BASE + 0x20)	// R/W	SDRAM Extended Mode Register 2
		#define SDEMR2_MR_DEF		NVTBIT(17, 15) 	// Mode Register Definition
		#define SDEMR2_CONFIGURE		NVTBIT(13, 0)		// SDRAM Dependent Configuration
#define REG_SDEMR3		(SDIC_BASE+ 0x24)// R/W	SDRAM Extended Mode Register 3
		#define SDEMR3_MR_DEF		NVTBIT(17, 15) 	// Mode Register Definition
		#define SDEMR3_CONFIGURE		NVTBIT(13, 0)	  	// SDRAM Dependent Configuration
#define REG_SDTIME		(SDIC_BASE + 0x28)	// R/W	SDRAM Timing Control Register
		#define TWTR				NVTBIT(30, 29) 		// Internal Write to Read Command Delay
		#define TRRD				NVTBIT(28, 27) 		// Active Bank a to Active Bank b Command Delay
		#define TRC				NVTBIT(26, 22) 		// Active to Active Command Delay
		#define TXSR				NVTBIT(21, 17) 		// Exit SELF REFRESH to ACTIVE Command Delay
		#define TRFC				NVTBIT(16, 12) 		// AUTO REFRESH Period
		#define TRAS				NVTBIT(11, 8) 		// ACTIVE to PRECHARGE Command Delay
		#define TRCD				NVTBIT(7, 5) 			// Active to READ or WRITE Delay
		#define TRP				NVTBIT(4, 2) 			// PRECHARGE Command Period
		#define TWR 				NVTBIT(1, 0) 			// WRITE Recovery Time
#define REG_DQSODS		(SDIC_BASE + 0x30)	// R/W	DQS Output Delay Selection Register
		#define DQSINVEN			BIT13 				// DQS Invert Enable
		#define DQS1_ODS			NVTBIT(12, 8) 		// DQS1 Output Delay Selection
		#define DQS0_ODS			NVTBIT(4, 0) 			// DQS0 Output Delay Selection
#define REG_CKDQSDS		(SDIC_BASE + 0x34)	// R/W	Clock and DQS Delay Selection Register
		#define DQS1_DS1			NVTBIT(23, 20) 		// DQS1 Input Delay Selection 1
		#define DQS1_DS0			NVTBIT(19, 16) 		// DQS1 Input Delay Selection 0
		#define DQS0_DS1			NVTBIT(15, 12) 		// DQS0 Input Delay Selection 1
		#define DQS0_DS0			NVTBIT(11, 8) 		// DQS0 Input Delay Selection 0
		#define DCLK_DS			NVTBIT(7, 4) 			// Data Clock Delay Selection 
		#define DCLKSRCSEL		BIT3 				// Data Clock Source Selection	
		#define MCLK_ODS		NVTBIT(2, 0) 			// MCLK Output Delay Selection	
#define REG_TESTCR		(SDIC_BASE + 0x40)	// R/W	SDRAM test control register
		#define STATUS_CLR		BIT31 				// Test Status Clear
		#define TEST_EN			BIT30				// Connection Test Enable
		#define BIST_EN			BIT29 				// SDRAM BIST Enable
		#define MARCH_C			BIT28				// MARCH_C Algorithm Used
		#define MAX_ADDR		NVTBIT(26, 0) 		// Maximum Test Address
#define REG_TSTATUS		(SDIC_BASE + 0x44)	// R	SDRAM test status register
		#define TEST_BUSY		BIT31 				// Test BUSY
		#define CON_BUSY		BIT30				// Connection Test Busy
		#define BIST_BUSY		BIT29				// BIST Test Busy
		#define TEST_FAIL			BIT28 				// Test Failed
		#define CON_FAIL			BIT27				// Connection Test Failed
		#define BIST_FAIL			BIT26 				// BIST Test Failed	
#define REG_TFDATA		(SDIC_BASE+ 0x48)	// R	SDRAM test fail data
#define REG_TGDATA		(SDIC_BASE + 0x4C)	// R	SDRAM test Gold data
		#define TGDATA			NVTBIT(31, 0) 		// Test Gold data

/* Timer Registers */
#define TMR_BA		W55FA93_VA_TIMER	/* Timer */

#define REG_TCSR0	(TMR_BA+0x00)  /* Timer Control and Status Register 0 */
#define REG_TCSR1	(TMR_BA+0x04)  /* Timer Control and Status Register 1 */
#define REG_TICR0	(TMR_BA+0x08)  /* Timer Initial Control Register 0 */
#define REG_TICR1	(TMR_BA+0x0C)  /* Timer Initial Control Register 1 */
#define REG_TDR0	(TMR_BA+0x10)  /* Timer Data Register 0 */
#define REG_TDR1	(TMR_BA+0x14)  /* Timer Data Register 1 */
#define REG_TISR	(TMR_BA+0x18)  /* Timer Interrupt Status Register */
#define REG_WTCR	(TMR_BA+0x1C)  /* Watchdog Timer Control Register */


#define LCM_BA		W55FA93_VA_VPOST	/* Display, LCM Interface */

/*
 VPOST Control Registers
*/
#define REG_LCM_LCDCCtl  	 		(LCM_BA+0x00)  	// R/W: LCD Controller Control Register
	#define LCDCCtl_FSADDR_SEL		BIT31
	#define LCDCCtl_HAW_656			BIT30
	#define LCDCCtl_PRDB_SEL		NVTBIT(21,20)
	#define LCDCCtl_YUVBL			BIT16				// YUV big endian(0) or little endian(1)
	#define LCDCCtl_FBDS 			NVTBIT(3,1)			// Frame Buffer Data Selection
	#define LCDCCtl_LCDRUN 			BIT0				// LCD Controller Run.

#define REG_LCM_LCDCPrm   		(LCM_BA+0x04)		// R/W: LCD Controller Parameter Register
	#define LCDCPrm_Even_Field_AL	NVTBIT(31,28)
	#define LCDCPrm_Odd_Field_AL	NVTBIT(27,24)
	#define LCDCPrm_F1_EL			NVTBIT(23,15)
	#define LCDCPrm_LCDSynTv		BIT8				// LCD timming Synch with TV
	#define LCDCPrm_SRGB_EL_SEL		NVTBIT(7,6)
	#define LCDCPrm_SRGB_OL_SEL		NVTBIT(5,4)
	#define LCDCPrm_LCDDataSel		NVTBIT(3,2)			// LCD data interface Select
	#define LCDCPrm_LCDTYPE	  		NVTBIT(1,0)			// LCD device Type Select.

#define REG_LCM_LCDCInt 		(LCM_BA+0x08)		// R/W: LCD Controller Interrupt Register
	#define LCDCInt_MPUCPLINTEN		BIT20 				// MPU Frame Complete Enable
	#define LCDCInt_TVFIELDINTEN	BIT18				// TV Even/Odd Field Interrupt Enable.		
	#define LCDCInt_VINTEN			BIT17				// LCD VSYNC Interrupt Enable.
	#define LCDCInt_HINTEN			BIT16				// LCD HSYNC Interrupt Enable.
	#define LCDCInt_MPUCPL			BIT4				// MPU Frame Complete
	#define LCDCInt_TVFIELDINT		BIT2				// TV Odd/Even Field Interrupt.	
	#define LCDCInt_VINT			BIT1				// LCD VSYNC/RD End Interrupt.	
	#define LCDCInt_HINT			BIT0				// LCD HSYNC/WR End Interrupt.


	#define LCDCInt_VINT			BIT1				// LCD VSYNC/RD End Interrupt.
	

#define REG_FEADDR 			(LCM_BA+0x0c)		// Reserved

#define REG_LCM_TCON1 			(LCM_BA+0x10)		// R/W: Timing Control Register 1
	#define TCON1_HSPW   			NVTBIT(23,16)		// Horizontal sync pulse width determines the HSYNC pulse's high level width by counting the number of the LCD Pixel Clock.
	#define TCON1_HBPD   			NVTBIT(15,8)		// Horizontal back porch is the number of LCD Pixel Clock periods between the falling edge of HSYNC and the start of active data.
	#define TCON1_HFPD   			NVTBIT(7,0)			// Horizontal front porch is the number of LCD Pixel Clock periods between the end of active data and the rising edge of HSYNC.

#define REG_LCM_TCON2 			(LCM_BA+0x14)		// R/W: Timing Control Register 2
	#define TCON2_VSPW				NVTBIT(23,16)		// Vertical sync pulse width determines the VSYNC pulse's high level width by counting the number of inactive lines.
	#define TCON2_VBPD				NVTBIT(15,8)		// Vertical back porch is the number of inactive lines at the start of a frame, after vertical synchronization period.
	#define TCON2_VFPD				NVTBIT(7,0)		// Vertical front porch is the number of inactive lines at the end of a frame, before vertical synchronization period.

#define REG_LCM_TCON3 			(LCM_BA+0x18)		// R/W: Timing Control Register 3
	#define TCON3_PPL				NVTBIT(31,16)		// Pixel Per-LineThe PPL bit field specifies the number of pixels in each line or row of screen.
	#define TCON3_LPP				NVTBIT(15,0)		// Lines Per-Panel The LPP bit field specifies the number of active lines per screen.

#define REG_LCM_TCON4 			(LCM_BA+0x1c)		// R : Timing Control Register 4
	#define TCON4_TAPN				NVTBIT(25,16)		// Horizontal Total Active Pixel Number
	#define TCON4_MVPW				NVTBIT(15,8)
	#define TCON4_MPU_FMARKP		BIT5
	#define TCON4_MPU_VSYNCP		BIT4
	#define TCON4_VSP				BIT3				// LCD VSYNC Polarity.
	#define TCON4_HSP				BIT2				// LCD HSYNC Polarity.
	#define TCON4_DEP				BIT1				// LCD VDEN Polarity.
	#define TCON4_PCLKP				BIT0				// LCD Pixel Clock Polarity.

#define REG_LCM_MPUCMD 			(LCM_BA+0x20)		// R/W: MPU-type LCD Command Register
	#define MPUCMD_MPU_VFPIN_SEL	BIT31
	#define MPUCMD_DIS_SEL			BIT30
	#define MPUCMD_CMD_DISn			BIT29				// Select command mode or display mode
	#define MPUCMD_MPU_CS			BIT28				// Set CS pin
	#define MPUCMD_MPU_ON			BIT27				// Trig to write or read from MPU in command mode
	#define MPUCMD_BUSY				BIT26				// Command interface is busy.
	#define MPUCMD_WR_RS			BIT25				// Write/Read RS Setting.
	#define MPUCMD_MPU_RWn			BIT24				// Read Status or data.
	#define MPUCMD_MPU68			BIT23				// MPU interface selection, reserved in w55fa93
	#define MPUCMD_FMARK			BIT22				// Frame Mark Detection Disable/Enable
	#define MPUCMD_MPU_SI_SEL		NVTBIT(19,16)
	#define MPUCMD_MPU_CMD			VTBIT(15,0)			// MPU-type LCD command/parameter data, read data

#define REG_LCM_MPUTS 			(LCM_BA+0x24)		// R/W: MPU type LCD timing setting
	#define MPUTS_CSnF2DCt			NVTBIT(31,24)		// CSn fall edge to data change clock counter
	#define MPUTS_WRnR2CSnRt		NVTBIT(23,16)		// WRn rising edge to CSn rising clock counter
	#define MPUTS_WRnLWt			NVTBIT(15, 8)		// WR low pulse clock counter	
	#define MPUTS_CSnF2WRnFt		NVTBIT( 7, 0)		// CSn falling edge to WR falling edge clock counter	

#define REG_LCM_OSD_CTL 		(LCM_BA+0x28)		// R/W : OSD Control Register
	#define OSD_CTL_OSD_EN			BIT31
	#define OSD_CTL_OSD_FSEL		NVTBIT(27,24)
	#define OSD_CTL_OSD_TC			NVTBIT(23,0)

#define REG_LCM_OSD_SIZE 		(LCM_BA+0x2C)		// R/W: OSD Picture Size
	#define OSD_SIZE_OSD_VSIZE		NVTBIT(25,16)
	#define OSD_SIZE_OSD_HSIZE		NVTBIT(9,0)

#define REG_LCM_OSD_SP	 		(LCM_BA+0x30)		// R/W: OSD Start Position
	#define OSD_SP_OSD_SY			NVTBIT(25,16)
	#define OSD_SP_OSD_SX			NVTBIT(9,0)

#define REG_LCM_OSD_BEP	 		(LCM_BA+0x34)		// R/W: OSD Bar End Position
	#define OSD_BEP_OSD_1BEY		NVTBIT(25,16)
	#define OSD_BEP_OSD_1BEX		NVTBIT(9,0)

#define REG_LCM_OSD_BO			(LCM_BA+0x38)		// R/W: OSD Bar Offset
	#define OSD_BO_OSD_BOY			NVTBIT(25,16)
	#define OSD_BO_OSD_BOX			NVTBIT(9,0)

#define REG_LCM_CBAR	  		(LCM_BA+0x3C)		// R/W: Color Burst Avtive Region
	#define CBAR_CTL_EQ6SEL			BIT28
	#define CBAR_CTL_HCBEPC			NVTBIT(25,16)
	#define CBAR_CTL_HCBBPC			NVTBIT(9,0)

#define REG_LCM_TVCtl			(LCM_BA+0x40)		// R/W: TvControl Register
	#define TVCtl_TvField 			BIT31				// Tv field status (read only)
														//   	 1 = Odd field, 0 = even field
	#define TVCtl_TvCMM 			BIT16				// TV Color Modulation Method, reserved in w55fa93
															//   1 = 27 MHz, 0 = 13.5 MHz 
	#define TVCtl_FBSIZE 			NVTBIT(15,14)		// Frame Buffer Size in Tv NonInterlance Mode
															//	00 = 320x240 (QVGA)
															//  01 = 640x240 (HVGA)
															//  10 = 640x480 (VGA)											
															//	11 = reserved
	#define TVCtl_LCDSrc 			NVTBIT(11,10)		// LCD image source selection
	#define TVCtl_TvSrc 			NVTBIT(9,8)			// TV image source selection
	#define TVCtl_TvLBSA 			BIT6				// Tv Line Buffer Scaling Alograthim (320->640)
	#define TVCtl_NotchE 			BIT5				// Notch Filter Enable/Disable
	#define TVCtl_Tvdac 			BIT4				// Tv DAC Enable/Disable
	#define TVCtl_TvInter 			BIT3				// Interlance or Non Interlance
	#define TVCtl_TvSys 			BIT2				// TV System Selection.
	#define TVCtl_TvColor 			BIT1				// TV Color Selection Color/Black.
	#define TVCtl_TvSleep 			BIT0				// TV Encoder Enable/Disable.

#define REG_LCM_IIRA			(LCM_BA+0x44)		// R/W: IIR Denominator Coefficient Register
	#define IIRA_IIRA3				NVTBIT(26,18)		// IIR Denominator A3 Coefficient
	#define IIRA_IIRA2				NVTBIT(17,9)		// IIR Denominator A2 Coefficient
	#define IIRA_IIRA1				NVTBIT(8,0)			// IIR Denominator A1 Coefficient

#define REG_LCM_IIRB			(LCM_BA+0x48)		// R/W: IIR notch filter Numberator Coefficient
	#define IIRB_IIRB3				NVTBIT(26,18)		// IIR Denominator B3 Coefficient
	#define IIRB_IIRB2				NVTBIT(17,9)		// IIR Denominator B2 Coefficient
	#define IIRB_IIRB1				NVTBIT(8,0)			// IIR Denominator B1 Coefficient

#define REG_LCM_COLORSET  		(LCM_BA+0x4C)		// R/W: Backdraw Color Setting Register
	#define  COLORSET_Color_R		NVTBIT(23,16)		// Color R value
	#define  COLORSET_Color_G		NVTBIT(15,8)		// Color G value
	#define  COLORSET_Color_B		NVTBIT(7,0)			// Color B value

#define REG_LCM_FSADDR    		(LCM_BA+0x50)		// R/W: Frame Buffer Start Address

#define REG_LCM_TVDisCtl  		(LCM_BA+0x54)		// R/W: TV Display Start Control Register
	#define TVDisCtl_FFRHS			NVTBIT(31,24)
	#define TVDisCtl_LCDHB			NVTBIT(23,16)		// LCD H bland setting for Syn TV Display
	#define TVDisCtl_TVDVS			NVTBIT(15,8)		// TV Display Start Line Register
	#define TVDisCtl_TVDHS			NVTBIT(7,0)			// TV Display Start Pixel Register

#define REG_LCM_CBACtl  		(LCM_BA+0x58)		// R/W: Color Burst Amplitude Control Register
	#define CBACtl_CBA_CB4			NVTBIT(29,24)
	#define CBACtl_CBA_CB3			NVTBIT(21,16)
	#define CBACtl_CBA_CB2			NVTBIT(13,8)
	#define CBACtl_CBA_CB1			NVTBIT(5,0)

#define REG_LCM_OSD_ADDR  		(LCM_BA+0x5C)		// R/W: OSD Frame Buffer Start Address
#define REG_RESERVED2 	  	(LCM_BA+0x60)		// Reserved

#define REG_LCM_TVContrast		(LCM_BA+0x64)		// R/W: Tv contrast adjust setting register
	#define TVContrast_Cr_contrast	NVTBIT(23,16)		// Cr compoment contrast adjust
	#define TVContrast_Cb_contrast	NVTBIT(15,8)		// Cb compoment contrast adjust
	#define TVContrast_Y_contrast	NVTBIT(7,0)			// Y  compoment contrast adjust

#define REG_LCM_TVBright  		(LCM_BA+0x68)		// R/W: Tv Bright adjust setting register
	#define TVBright_Cr_gain		NVTBIT(23,16)		// Cr compoment bright adjust
	#define TVBright_Cb_gain		NVTBIT(15,8)		// Cb compoment bright adjust
	#define TVBright_Y_bright		NVTBIT(7,0)			// Y  compoment bright adjust

#define REG_RESERVED3	  	(LCM_BA+0x6C)		// Reserved

#define REG_LCM_LINE_STRIPE		(LCM_BA+0x70)		// R/W : Line Stripe Offset
	#define LINE_STRIPE_F1_LSL		NVTBIT(15,0)
	
#define REG_LCM_RGBin		  	(LCM_BA+0x74)		// RGB888 data input for RGB2YCbCr equation

#define REG_LCM_YCbCrout  		(LCM_BA+0x78)		// YCbCr data output for RGB2YCbCr equation

#define REG_LCM_YCbCrin	  		(LCM_BA+0x7C)		// YCbCr data input for YCbCr2RGB equation
	#define YCbCrin_Yin				NVTBIT(23,16)		// Y byte data input
	#define YCbCrin_Cbin			NVTBIT(15, 8)		// Cb byte data input
	#define YCbCrin_Crin			NVTBIT( 7, 0)		// Cr byte data input

#define REG_LCM_RGBout	  		(LCM_BA+0x80)		// RGB data output for YCbCr2RGB equation
	#define RGBout_Rout				NVTBIT(23,16)		// R byte data output
	#define RGBout_Gout				NVTBIT(15, 8)		// G byte data output
	#define RGBout_Bout				NVTBIT( 7, 0)		// B byte data output


/*
 SIC Control Registers
*/
#define DMAC_BA		W55FA93_VA_SIC		/* SIC Control */

#define     REG_FB_0			(DMAC_BA+0x000)  /* Shared Buffer (FIFO) */

#define	REG_DMACCSR			(DMAC_BA+0x400)  /* DMAC Control and Status Register */
	#define	FMI_BUSY			BIT9			// FMI DMA transfer is in progress
	#define SG_EN				BIT3			// DMAC Scatter-gather function enable
	#define DMAC_SWRST			BIT1			// DMAC software reset enable 
	#define DMAC_EN				BIT0			// DMAC enable


#define     REG_DMACSAR	(DMAC_BA+0x408)  /* DMAC Transfer Starting Address Register */
#define     REG_DMACBCR		(DMAC_BA+0x40C)  /* DMAC Transfer Byte Count Register */
#define     REG_DMACIER		(DMAC_BA+0x410)  /* DMAC Interrupt Enable Register */
	#define	WEOT_IE				BIT1			// Wrong EOT encounterred interrupt enable
	#define TABORT_IE			BIT0			// DMA R/W target abort interrupt enable

#define     REG_DMACISR		(DMAC_BA+0x414)  /* DMAC Interrupt Status Register */
	#define	WEOT_IF				BIT1			// Wrong EOT encounterred interrupt flag
	#define TABORT_IF			BIT0			// DMA R/W target abort interrupt flag


#define FMI_BA		(W55FA93_VA_SIC+0x800)	/* Flash Memory Card Interface */
/* Flash Memory Card Interface Registers */
#define REG_FMICR				(FMI_BA+0x000)   	/* FMI Control Register */
	#define	FMI_SM_EN				BIT3				// enable FMI SM function
	#define FMI_SD_EN				BIT1				// enable FMI SD function		
	#define FMI_SWRST				BIT0				// enable FMI software reset

#define REG_FMIIER				(FMI_BA+0x004)   	/* FMI DMA Transfer Starting Address Register */
	#define	FMI_DAT_IE				BIT0				// enable DMAC READ/WRITE targe abort interrupt generation
	
#define REG_FMIISR				(FMI_BA+0x008)   	/* FMI DMA Byte Count Register */
	#define	FMI_DAT_IF				BIT0				// DMAC READ/WRITE targe abort interrupt flag register

/* Secure Digit Registers */
#define REG_SDCR				(FMI_BA+0x020)   	/* SD Control Register */
	#define	SDCR_CLK_KEEP1			BIT31				// SD-1 clock keep control
	#define	SDCR_SDPORT				NVTBIT(30,29)		// SD port select
	#define	SDCR_SDPORT_0			0					// SD-0 port selected 
	#define	SDCR_SDPORT_1			BIT29				// SD-1 port selected 	
	#define	SDCR_SDPORT_2			BIT30				// SD-2 port selected 		
	#define	SDCR_CLK_KEEP2			BIT28				// SD-1 clock keep control	
	#define	SDCR_SDNWR				NVTBIT(27,24)		// Nwr paramter for Block Write operation
	#define SDCR_BLKCNT				NVTBIT(23,16)		// Block conut to be transferred or received
	#define	SDCR_DBW				BIT15				// SD data bus width selection
	#define	SDCR_SWRST				BIT14				// enable SD software reset		
	#define	SDCR_CMD_CODE			NVTBIT(13,8)		// SD Command Code
	#define	SDCR_CLK_KEEP			BIT7				// SD Clock Enable
	#define SDCR_8CLK_OE			BIT6				// 8 Clock Cycles Output Enable
	#define SDCR_74CLK_OE			BIT5				// 74 Clock Cycle Output Enable
	#define SDCR_R2_EN				BIT4				// Response R2 Input Enable
	#define SDCR_DO_EN				BIT3				// Data Output Enable
	#define SDCR_DI_EN				BIT2				// Data Input Enable
	#define SDCR_RI_EN				BIT1				// Response Input Enable
	#define SDCR_CO_EN				BIT0				// Command Output Enable

#define REG_SDARG 				(FMI_BA+0x024)   	/* SD command argument register */

#define REG_SDIER				(FMI_BA+0x028)   	/* SD interrupt enable register */
	#define	SDIER_CDSRC				BIT30				// SD card detection source selection: SD-DAT3 or GPIO
	#define	SDIER_R1B_IEN			BIT24				// R1b interrupt enable 
	#define	SDIER_WKUP_EN			BIT14				// SDIO wake-up signal geenrating enable
	#define	SDIER_DITO_IEN			BIT13				// SD data input timeout interrupt enable
	#define	SDIER_RITO_IEN			BIT12				// SD response input timeout interrupt enable		
	#define SDIER_SDIO1_IEN			BIT11				// SDIO1 Interrupt Status Enable (SDIO issue interrupt via DAT[1]
	#define SDIER_SDIO_IEN			BIT10				// SDIO0 Interrupt Status Enable (SDIO issue interrupt via DAT[1]
	#define SDIER_CD_IEN			BIT8				// CD# Interrupt Status Enable		
	#define SDIER_SDIO2_IEN			BIT2				// SDIO2 Interrupt Status Enable (SDIO issue interrupt via DAT[1]
	#define SDIER_CRC_IEN			BIT1				// CRC-7, CRC-16 and CRC status error interrupt enable
	#define SDIER_BLKD_IEN			BIT0				// Block transfer done interrupt interrupt enable

#define REG_SDISR				(FMI_BA+0x02C)   	/* SD interrupt status register */
	#define	SDISR_R1B_IF			BIT24				// R1b interrupt flag                                         
	#define	SDISR_SDIO2_IF			BIT22			    // SDIO2 interrupt flag (SDIO issue interrupt via DAT[1]		  
	#define SDISR_SD2_DATA1			BIT20			    // SD2 DAT1 pin status                                         
	#define SDISR_SD1_DATA1			BIT19			    // SD1 DAT1 pin status                                         
	#define SDISR_SD_DATA1			BIT18			    // SD DAT1 pin status                                         
	#define SDISR_CD_Card			BIT16			    // CD detection pin status                                    
	#define	SDISR_DITO_IF			BIT13			    // SD data input timeout interrupt flag                       
	#define	SDISR_RITO_IF			BIT12			    // SD response input timeout interrupt flag                   
	#define	SDISR_SDIO1_IF			BIT11			    // SDIO1 interrupt flag (SDIO issue interrupt via DAT[1]		  
	#define	SDISR_SDIO_IF			BIT10			    // SDIO0 interrupt flag (SDIO issue interrupt via DAT[1]		  
	#define	SDISR_CD_IF				BIT8			    // CD# interrupt flag                                         
	#define SDISR_SD_DATA0			BIT7			    // SD DATA0 pin status                                        
	#define SDISR_CRC				NVTBIT(6,4)		    // CRC status                                                 
	#define SDISR_CRC_16			BIT3			    // CRC-16 Check Result Status                                 
	#define SDISR_CRC_7				BIT2			    // CRC-7 Check Result Status                                  
	#define	SDISR_CRC_IF			BIT1			    // CRC-7, CRC-16 and CRC status error interrupt status        
	#define	SDISR_BLKD_IF			BIT0			    // Block transfer done interrupt interrupt status             

#define REG_SDRSP0				(FMI_BA+0x030)   	/* SD receive response token register 0 */     
#define REG_SDRSP1				(FMI_BA+0x034)   	/* SD receive response token register 1 */     
#define REG_SDBLEN				(FMI_BA+0x038)   	/* SD block length register */                 
#define REG_SDTMOUT 			(FMI_BA+0x03C)   	/* SD block length register */                 
 
/* NAND-type Flash Registers */
// old nuc930
#define     REG_SM_ECC48_ST0		(FMI_BA+0x0D4)   /* ECC Register */
#define     REG_SM_ECC48_ST1		(FMI_BA+0x0D8)   /* ECC Register */
#define     REG_BCH_ECC_BIT_ADDR0	(FMI_BA+0x220)   /* NAND Flash BCH error bit address for error bit 0-7 Register */
#define     REG_BCH_ECC_BIT_ADDR1	(FMI_BA+0x224)   /* NAND Flash BCH error bit address for error bit 8-14 Register */

// new FA93
#define REG_SMCSR				(FMI_BA+0x0A0)   	/* NAND Flash Control and Status Register */
	#define SMCR_CS1				BIT26				// SM chip select 	
	#define SMCR_CS0				BIT25				// SM chip select 
	#define SMCR_CS					BIT25				// SM chip select 		
	#define SMCR_ECC_EN				BIT23				// SM chip select 	
	#define SMCR_BCH_TSEL 			NVTBIT(22,19)			// BCH T4/8/12/15 selection
		#define BCH_T15 				BIT22			// BCH T15 selected
		#define BCH_T12 				BIT21			// BCH T12 selected
		#define BCH_T8	 				BIT20			// BCH T8 selected
		#define BCH_T4	 				BIT19			// BCH T4 selected	
		
	#define SMCR_PSIZE 				NVTBIT(17,16)			// SM page size selection
		#define PSIZE_8K 			BIT17+BIT16			// page size 8K selected 
		#define PSIZE_4K 			BIT17				// page size 4K selected
		#define PSIZE_2K 			BIT16				// page size 2K selected
		#define PSIZE_512			0					// page size 512 selected

	#define SMCR_SRAM_INIT			BIT9				// SM RA0_RA1 initial bit (to 0xFFFF_FFFF)
	#define SMCR_ECC_3B_PROTECT		BIT8				// ECC protect redundant 3 bytes
	#define SMCR_ECC_CHK			BIT7				// ECC parity check enable bit during read page
	#define SMCR_REDUN_AUTO_WEN 	BIT4				// Redundant auto write enable		
	#define SMCR_REDUN_REN 			BIT3				// Redundant read enable		
	#define SMCR_DWR_EN 			BIT2				// DMA write data enable
	#define SMCR_DRD_EN 			BIT1				// DMA read data enable		
	#define SMCR_SM_SWRST 			BIT0				// SM software reset


#define REG_SMTCR				(FMI_BA+0x0A4)   	/* NAND Flash Timing Control Register */

#define REG_SMIER				(FMI_BA+0x0A8)   	/* NAND Flash Interrupt Control Register */
	#define	SMIER_RB1_IE			BIT11				// RB1 pin rising-edge detection interrupt enable	
	#define	SMIER_RB0_IE			BIT10				// RB0 pin rising-edge detection interrupt enable
	#define	SMIER_RB_IE				BIT10				// RB0 pin rising-edge detection interrupt enable		
	#define SMIER_ECC_FIELD_IE 		BIT2				// ECC field error check interrupt enable
	#define SMIER_DMA_IE			BIT0				// DMA RW data complete interrupr enable

#define REG_SMISR				(FMI_BA+0x0AC)   	/* NAND Flash Interrupt Status Register */
	#define	SMISR_RB1		 		BIT19				// RB1 pin status
	#define	SMISR_RB0 				BIT18				// RB0 pin status			
	#define	SMISR_RB 				BIT18				// RB pin status
	#define	SMISR_RB1_IF		 	BIT11				// RB pin rising-edge detection interrupt flag		
	#define	SMISR_RB0_IF	 		BIT10				// RB pin rising-edge detection interrupt flag
	#define SMISR_ECC_FIELD_IF 		BIT2				// ECC field error check interrupt flag
	#define SMISR_DMA_IF			BIT0				// DMA RW data complete interrupr flag


#define REG_SMCMD				(FMI_BA+0x0B0)   	/* NAND Flash Command Port Register */

#define REG_SMADDR				(FMI_BA+0x0B4)   	/* NAND Flash Address Port Register */
	#define	EOA_SM	 				BIT31				// end of SM address for last SM address
	
#define REG_SMDATA				(FMI_BA+0x0B8)   	/* NAND Flash Data Port Register */

#define REG_SMREAREA_CTL 		(FMI_BA+0x0BC)  	/* NAND Flash redundnat area control register */
	#define SMRE_MECC 				NVTBIT(31,16)		// Mask ECC parity code to NAND during Write Page Data to NAND by DMAC	
	#define	SMRE_REA128_EXT			NVTBIT(8,0)			// Redundant area enabled byte number	

#define REG_SM_ECC_ST0			(FMI_BA+0x0D0)   	/* ECC Register */                                                   
	#define ECCST_F4_ECNT			NVTBIT(28,26)		// error count of ECC for field 4
	#define ECCST_F4_STAT 			NVTBIT(25,24)		// error status of ECC for field 4		
	#define ECCST_F3_ECNT 	        NVTBIT(21,18)		// error count of ECC for field 3        
	#define ECCST_F3_STAT 	        NVTBIT(17,16)		// error status of ECC for field 3		
	#define ECCST_F2_ECNT 	        NVTBIT(13,10)		// error count of ECC for field 2        
	#define ECCST_F2_STAT 	        NVTBIT(9,8)			// error status of ECC for field 2		
	#define ECCST_F1_ECNT 	        NVTBIT(5,2)			// error count of ECC for field 1        
	#define ECCST_F1_STAT 	        NVTBIT(1,0)			// error status of ECC for field 1		

#define REG_SM_ECC_ST1		    (FMI_BA+0x0D4)   	/* ECC Register */                                                   
	#define ECCST_F8_ECNT			NVTBIT(28,26)		// error count of ECC for field 8
	#define ECCST_F8_STAT 			NVTBIT(25,24)		// error status of ECC for field 8		
	#define ECCST_F7_ECNT 	        NVTBIT(21,18)		// error count of ECC for field 7        
	#define ECCST_F7_STAT 	        NVTBIT(17,16)		// error status of ECC for field 7		
	#define ECCST_F6_ECNT 	        NVTBIT(13,10)		// error count of ECC for field 6        
	#define ECCST_F6_STAT 	        NVTBIT(9,8)			// error status of ECC for field 6		
	#define ECCST_F5_ECNT 	        NVTBIT(5,2)			// error count of ECC for field 5        
	#define ECCST_F5_STAT 	        NVTBIT(1,0)			// error status of ECC for field 5		

#define REG_SM_ECC_ST2		    (FMI_BA+0x0D8)   	/* ECC Register */                                                   
	#define ECCST_F12_ECNT			NVTBIT(28,26)		// error count of ECC for field 12
	#define ECCST_F12_STAT 			NVTBIT(25,24)		// error status of ECC for field 12	
	#define ECCST_F11_ECNT 	        NVTBIT(21,18)		// error count of ECC for field 11       
	#define ECCST_F11_STAT 	        NVTBIT(17,16)		// error status of ECC for field 11
	#define ECCST_F10_ECNT 	        NVTBIT(13,10)		// error count of ECC for field 10       
	#define ECCST_F10_STAT 	        NVTBIT(9,8)			// error status of ECC for field 10	
	#define ECCST_F9_ECNT 	        NVTBIT(5,2)			// error count of ECC for field 9        
	#define ECCST_F9_STAT 	        NVTBIT(1,0)			// error status of ECC for field 9		

#define REG_SM_ECC_ST3		    (FMI_BA+0x0DC)   	/* ECC Register */                                                   
	#define ECCST_F16_ECNT			NVTBIT(28,26)		// error count of ECC for field 16
	#define ECCST_F16_STAT 			NVTBIT(25,24)		// error status of ECC for field 16	
	#define ECCST_F15_ECNT 	        NVTBIT(21,18)		// error count of ECC for field 15       
	#define ECCST_F15_STAT 	        NVTBIT(17,16)		// error status of ECC for field 15	
	#define ECCST_F14_ECNT 	        NVTBIT(13,10)		// error count of ECC for field 14       
	#define ECCST_F14_STAT 	        NVTBIT(9,8)			// error status of ECC for field 14	
	#define ECCST_F13_ECNT 	        NVTBIT(5,2)			// error count of ECC for field 13       
	#define ECCST_F13_STAT 	        NVTBIT(1,0)			// error status of ECC for field 13	

                                                                                                                         
#define REG_BCH_ECC_ADDR0	    (FMI_BA+0x100)   	/* NAND Flash BCH error byte address for error bit 0-1 Register */   
#define REG_BCH_ECC_ADDR1	    (FMI_BA+0x104)   	/* NAND Flash BCH error byte address for error bit 2-3 Register */   
#define REG_BCH_ECC_ADDR2	    (FMI_BA+0x108)   	/* NAND Flash BCH error byte address for error bit 4-5 Register */   
#define REG_BCH_ECC_ADDR3	    (FMI_BA+0x10C)   	/* NAND Flash BCH error byte address for error bit 6-7 Register */   
#define REG_BCH_ECC_ADDR4	    (FMI_BA+0x110)   	/* NAND Flash BCH error byte address for error bit 8-9 Register */   
#define REG_BCH_ECC_ADDR5	    (FMI_BA+0x114)   	/* NAND Flash BCH error byte address for error bit 10-11 Register */ 
#define REG_BCH_ECC_ADDR6	    (FMI_BA+0x118)   	/* NAND Flash BCH error byte address for error bit 12-13 Register */ 
#define REG_BCH_ECC_ADDR7	    (FMI_BA+0x11C)   	/* NAND Flash BCH error byte address for error bit 14 Register */    
#define REG_BCH_ECC_DATA0	    (FMI_BA+0x160)   	/* NAND Flash BCH error data for error bit 0-3 Register */           
#define REG_BCH_ECC_DATA1	    (FMI_BA+0x164)   	/* NAND Flash BCH error data for error bit 4-7 Register */           
#define REG_BCH_ECC_DATA2	    (FMI_BA+0x168)   	/* NAND Flash BCH error data for error bit 8-11 Register */          
#define REG_BCH_ECC_DATA3	    (FMI_BA+0x16C)   	/* NAND Flash BCH error data for error bit 12-14 Register */         

#define REG_SMRA_0				(FMI_BA+0x200)   /* NAND Flash Redundant Area Register */
#define REG_SMRA_1		        (FMI_BA+0x204)                                           
#define REG_SMRA_2		        (FMI_BA+0x208)                                           
#define REG_SMRA_3		        (FMI_BA+0x20C)                                           
#define REG_SMRA_4		        (FMI_BA+0x210)                                           
#define REG_SMRA_5		        (FMI_BA+0x214)                                           
#define REG_SMRA_6		        (FMI_BA+0x218)                                           
#define REG_SMRA_7		        (FMI_BA+0x21C)                                           
#define REG_SMRA_8		        (FMI_BA+0x220)                                           
#define REG_SMRA_9		        (FMI_BA+0x224)                                           
#define REG_SMRA_10		        (FMI_BA+0x228)                                           
#define REG_SMRA_11		        (FMI_BA+0x22C)                                           
#define REG_SMRA_12		        (FMI_BA+0x230)                                           
#define REG_SMRA_13		        (FMI_BA+0x234)                                           
#define REG_SMRA_14		        (FMI_BA+0x238)                                           
#define REG_SMRA_15		        (FMI_BA+0x23C)                                           
#define REG_SMRA_16		        (FMI_BA+0x240)                                           
#define REG_SMRA_17		        (FMI_BA+0x244)                                           
#define REG_SMRA_18		        (FMI_BA+0x248)                                           
#define REG_SMRA_19		        (FMI_BA+0x24C)                                           
#define REG_SMRA_20		        (FMI_BA+0x250)                                           
#define REG_SMRA_21		        (FMI_BA+0x254)                                           
#define REG_SMRA_22		        (FMI_BA+0x258)                                           
#define REG_SMRA_23		        (FMI_BA+0x25C)                                           
#define REG_SMRA_24		        (FMI_BA+0x260)                                           
#define REG_SMRA_25		        (FMI_BA+0x264)                                           
#define REG_SMRA_26		        (FMI_BA+0x268)                                           
#define REG_SMRA_27		        (FMI_BA+0x26C)                                           
#define REG_SMRA_28		        (FMI_BA+0x270)                                           
#define REG_SMRA_29		        (FMI_BA+0x274)                                           
#define REG_SMRA_30		        (FMI_BA+0x278)                                           
#define REG_SMRA_31		        (FMI_BA+0x27C)                                           
#define REG_SMRA_32		        (FMI_BA+0x280)                                           
#define REG_SMRA_33		        (FMI_BA+0x284)                                           
#define REG_SMRA_34		        (FMI_BA+0x288)                                           
#define REG_SMRA_35		        (FMI_BA+0x28C)                                           
#define REG_SMRA_36		        (FMI_BA+0x290)                                           
#define REG_SMRA_37		        (FMI_BA+0x294)                                           
#define REG_SMRA_38		        (FMI_BA+0x298)                                           
#define REG_SMRA_39		        (FMI_BA+0x29C)                                           
#define REG_SMRA_40		        (FMI_BA+0x2A0)                                           
#define REG_SMRA_41		        (FMI_BA+0x2A4)                                           
#define REG_SMRA_42		        (FMI_BA+0x2A8)                                           
#define REG_SMRA_43		        (FMI_BA+0x2AC)                                           
#define REG_SMRA_44		        (FMI_BA+0x2B0)                                           
#define REG_SMRA_45		        (FMI_BA+0x2B4)                                           
#define REG_SMRA_46		        (FMI_BA+0x2B8)                                           
#define REG_SMRA_47		        (FMI_BA+0x2BC)                                           
#define REG_SMRA_48		        (FMI_BA+0x2C0)                                           
#define REG_SMRA_49		        (FMI_BA+0x2C4)                                           
#define REG_SMRA_50		        (FMI_BA+0x2C8)                                           
#define REG_SMRA_51		        (FMI_BA+0x2CC)                                           
#define REG_SMRA_52		        (FMI_BA+0x2D0)                                           
#define REG_SMRA_53		        (FMI_BA+0x2D4)                                           
#define REG_SMRA_54		        (FMI_BA+0x2D8)   /* NAND Flash Redundant Area Register */

/* UHC Control Registers */
#define USBH_BA	W55FA93_VA_USBH		/* UHC Control */
#define REG_HC_REVISION         		(USBH_BA+0x000)	// HcRevision - Revision Register            
#define REG_HC_CONTROL          		(USBH_BA+0x004)	// HcControl  - Control Register				
#define REG_HC_CMD_STATUS       		(USBH_BA+0x008)	// HcCommandStatus - Command Status Register 
#define REG_HC_INT_STATUS       		(USBH_BA+0x00C)	// HcInterruptStatus  - Interrupt Status  Register 							
#define REG_HC_INT_ENABLE       		(USBH_BA+0x010)	// HcInterruptEnable - Interrupt Enable Register 								
#define REG_HC_INT_DISABLE      		(USBH_BA+0x014)	// HcInterruptDisable - Interrupt Disable Regster		
#define REG_HC_HCCA             			(USBH_BA+0x018)	// HcHCCA - Communication Area Register      								
#define REG_HC_PERIOD_CURED     		(USBH_BA+0x01C)	// HcPeriodCurrentED  										
#define REG_HC_CTRL_HEADED      		(USBH_BA+0x020)	// HcControlHeadED - Control Head ED Register   
#define REG_HC_CTRL_CURED       		(USBH_BA+0x024)	// HcControlCurrentED - Control Current ED Regist								
#define REG_HC_BULK_HEADED      		(USBH_BA+0x028)	// HcBulkHeadED - Bulk Head ED Register        
#define REG_HC_BULK_CURED       		(USBH_BA+0x02C)	// HcBulkCurrentED - Bulk Current ED Register 				
#define REG_HC_DONE_HEAD        		(USBH_BA+0x030)	// HcBulkCurrentED - Done Head Register         
#define REG_HC_FM_INTERVAL      		(USBH_BA+0x034)	// HcFmInterval - Frame Interval Register        						
#define REG_HC_FM_REMAINING     		(USBH_BA+0x038)	// HcFrameRemaining - Frame Remaining Register   		
#define REG_HC_FM_NUMBER        		(USBH_BA+0x03C)	// HcFmNumber - Frame Number Register            					
#define REG_HC_PERIOD_START     		(USBH_BA+0x040)	// HcPeriodicStart - Periodic Start Register    
#define REG_HC_LS_THRESHOLD     	(USBH_BA+0x044)	// HcLSThreshold - Low Speed Threshold Register  			
#define REG_HC_RH_DESCRIPTORA   	(USBH_BA+0x048)	// HcRhDescriptorA - Root Hub Descriptor A Register
#define REG_HC_RH_DESCRIPTORB   	(USBH_BA+0x04C)	// HcRevision - Root Hub Descriptor B Register
#define REG_HC_RH_STATUS        		(USBH_BA+0x050)	// HcRhStatus - Root Hub Status Register
#define REG_HC_RH_PORT_STATUS1  	(USBH_BA+0x054)	// HcRevision - Root Hub Port Status [1]
#define REG_HC_RH_PORT_STATUS2  	(USBH_BA+0x058)	// HcRevision - Root Hub Port Status [2]
#define REG_HC_RH_OP_MODE       		(USBH_BA+0x204)	
		#define DISPRT2			BIT17				// Disable Port 2
		#define DISPRT1			BIT16				// Disable Port 1
		#define SIEPDIS			BIT8				// SIE Pipeline Disable
		#define PPCALOW		BIT4				// Port Power Control Active Low
		#define OCALOW		BIT3				// Over Current Active Low
		#define HCABORT		BIT1				// AHB Bus ERROR Response
		#define DBR16			BIT0				// Data Buffer Region 16


/* Jpeg Control Registers */
#define JPEG_BASE	W55FA93_VA_JPEG		/* Jpeg Control */

#define	REG_JMCR		(JPEG_BASE+0x00)				// R/W: JPEG Mode Control Register
	#define	RESUMEI		BIT9				// Resume JPEG Operation for Input On-the-Fly Mode
	#define	RESUMEO		BIT8				// Resume JPEG Operation for Output On-the-Fly Mode
	#define	ENC_DEC		BIT7				// JPEG Encode/Decode Mode
	#define	WIN_DEC		BIT6				// JPEG Window Decode Mode
	#define	PRI			BIT5				// Encode Primary Image
	#define	THB			BIT4				// Encode Thumbnail Image
	#define	EY422		BIT3				// Encode Image Format
	#define	QT_BUSY		BIT2				// Quantization-Table Busy Status (Read-Only)
	#define	ENG_RST		BIT1				// Soft Reset JPEG Engine (Except JPEG Control Registers)
	#define	JPG_EN		BIT0				// JPEG Engine Operation Control

#define	REG_JHEADER		(JPEG_BASE+0x04)			// R/W: JPEG Encode Header Control Register
	#define	P_JFIF		BIT7				// Primary JPEG Bit-stream Include JFIF Header
	#define	P_HTAB		BIT6				// Primary JPEG Bit-stream Include Huffman-Table
	#define	P_QTAB		BIT5				// Primary JPEG Bit-stream Include Quantization-Table
	#define	P_DRI		BIT4				// Primary JPEG Bit-stream Include Restart Interval
	#define	T_JFIF		BIT3				// Thumbnail JPEG Bit-stream Include JFIF Header
	#define	T_HTAB		BIT2				// Thumbnail JPEG Bit-stream Include Huffman-Table
	#define	T_QTAB		BIT1				// Thumbnail JPEG Bit-stream Include Quantization-Table
	#define	T_DRI		BIT0				// Thumbnail JPEG Bit-stream Include Restart Interval

#define	REG_JITCR		(JPEG_BASE+0x08)			// R/W: JPEG Image Type Control Register
	#define	PLANAR_ON	BIT15				// Packet On
	#define	ORDER		BIT14				// Decode Packet Data Order
	#define	ROTATE		NVTBIT(12,11)			// Encode Image Rotate
	#define	DYUV_MODE	NVTBIT(10,8)			// Decoded Image YUV Color Format (Read-Only)
	#define	EXIF		BIT7				// Encode Quantization-Table & Huffman-Table Header Format Selection
	#define	EY_ONLY		BIT6				// Encode Gray-level (Y-component Only) Image
	#define	DHEND		BIT5				// Header Decode Complete Stop Enable
	#define	DTHB		BIT4				// Decode Thumbnail Image Only
	#define	E3QTAB		BIT3				// Numbers of Quantization-Table are Used For Encode
	#define	D3QTAB		BIT2				// Numbers of Quantization-Table are Used For Decode (Read-Only)
	#define	ERR_DIS		BIT1				// Decode Error Engine Abort
	#define	PDHTAB		BIT0				// Programmable Huffman-Table Function For Decode
	
#define	REG_JPRIQC		(JPEG_BASE+0x10)			// R/W: JPEG Primary Q-Table Control Register
	#define	P_QADJUST	NVTBIT(7,4)			// Primary Quantization-Table Adjustment
	#define	P_QVS		NVTBIT(3,0)			// Primary Quantization-Table Scaling Control

#define	REG_JTHBQC		(JPEG_BASE+0x14)			// R/W: JPEG Thumbnail Q-Table Control Register
	#define	T_QADJUST	NVTBIT(7,4)			// Thumbnail Quantization-Table Adjustment
	#define	T_QVS		NVTBIT(3,0)			// Thumbnail Quantization-Table Scaling Control

#define	REG_JPRIWH		(JPEG_BASE+0x18)			// R/W: JPEG Encode Primary Width/Height Register
	#define	P_HEIGHT	NVTBIT(27,16)			// Primary Encode Image Height
	#define	P_WIDTH		NVTBIT(11,0)			// Primary Encode Image Width

#define	REG_JTHBWH		(JPEG_BASE+0x1C)			// R/W: JPEG Encode Thumbnail Width/Height Register
	#define	T_HEIGHT	NVTBIT(27,16)			// Thumbnail Encode Image Height
	#define	T_WIDTH		NVTBIT(11,0)			// Thumbnail Encode Image Width

#define	REG_JPRST		(JPEG_BASE+0x20)			// R/W:  JPEG Encode Primary Restart Interval Register
	#define	P_RST		NVTBIT(7,0)				// Primary Encode Restart Interval Value
		
#define	REG_JTRST		(JPEG_BASE+0x24)			// R/W: JPEG Encode Thumbnail Restart Interval
	#define	T_RST		NVTBIT(7,0)				// Thumbnail Encode Restart Interval Value
		
#define	REG_JDECWH		(JPEG_BASE+0x28)			// R:  JPEG Decode Image Width/Height Register
	#define	DEC_HEIGHT	NVTBIT(31,16)			// 13-bit Bit Stream Buffer threshold 
	#define	DEC_WIDTH	NVTBIT(15,0)			// 13-bit Header Offset Address

#define	REG_JINTCR		(JPEG_BASE+0x2C)		// R/W:  JPEG Interrupt Control and Status Register
	#define	JPG_WAITI	BIT23				// JPEG Input Wait Status (Read-Only)
	#define	JPG_WAITO	BIT22				// JPEG Output Wait Status (Read-Only)
	#define	BAbort		BIT16				// JPEG Memory Access Error Status (Read-Only)
	#define	CER_INTE	BIT15				// Un-complete Capture On-The-Fly Frame Occur Interrupt Enable
	#define	DHE_INTE	BIT14				// JPEG Header Decode End Wait Interrupt Enable
	#define	IPW_INTE	BIT13				// Input Wait Interrupt Enable
	#define	OPW_INTE	BIT12				// Output Wait Interrupt Enable
	#define	ENC_INTE	BIT11				// Encode Complete Interrupt Enable
	#define	DEC_INTE	BIT10				// Decode Complete Interrupt Enable
	#define	DER_INTE	BIT9				// Decode Error Interrupt Enable
	#define	EER_INTE	BIT8				// Encode (On-The-Fly) Error Interrupt Enable
	#define	CER_INTS	BIT7				// Un-complete Capture On-The-Fly Frame Occur Interrupt Status
	#define	DHE_INTS	BIT6				// JPEG  Header Decode End Wait Interrupt Status
	#define	IPW_INTS	BIT5				// Input Wait Interrupt Status
	#define	OPW_INTS	BIT4				// Output Wait Interrupt Status	
	#define	ENC_INTS	BIT3				// Encode Complete Interrupt Status
	#define	DEC_INTS	BIT2				// Decode Complete Interrupt Status
	#define	DER_INTS	BIT1				// Decode Error Interrupt Status
	#define	EER_INTS	BIT0				// Encode (On-The-Fly) Error Interrupt Status
	
#define	REG_JPEG_BSBAD	(JPEG_BASE+0x40)		// R/W:  JPEG Test Control Register
	#define	BIST_ST		NVTBIT(23,16)			// Internal SRAM BIST Status (Read-Only)
	#define	TEST_DOUT	NVTBIT(15,8)			// Test Data Output (Read-Only)
	#define	TEST_ON		BIT7				// Test Enable
	#define	BIST_ON		BIT6				// Internal SRAM BIST Mode Enable
	#define	BIST_FINI	BIT5				// Internal SRAM BIST Mode Finish (Read-Only)
	#define	BSBAD_BIST_FAIL	BIT4				// Internal SRAM BIST Mode Fail (Read-Only)
	#define	TEST_SEL	BIT(3,0)			// Test Data Selection

#define	REG_JWINDEC0	(JPEG_BASE+0x44)			// R/W: JPEG Window Decode Mode Control Register 0
	#define	MCU_S_Y		NVTBIT(24,16)			// MCU Start Position Y For Window Decode Mode
	#define	MCU_S_X		NVTBIT(8,0)			// MCU Start Position X For Window Decode Mode

#define	REG_JWINDEC1	(JPEG_BASE+0x48)			// R/W: JPEG Window Decode Mode Control Register 1
	#define	MCU_E_Y		NVTBIT(24,16)			// MCU End Position Y For Window Decode Mode
	#define	MCU_E_X		NVTBIT(8,0)			// MCU End Position X For Window Decode Mode
	
#define	REG_JWINDEC2	(JPEG_BASE+0x4C)			// R/W:  JPEG Window Decode Mode Control Register 2
	#define	WD_WIDTH	NVTBIT(11,0)			// Image Width (Y-Stride) For Window Decode Mode
	
#define	REG_JMACR		(JPEG_BASE+0x50)			// R/W: JPEG Memory Address Mode Control Register
	#define	FLY_SEL		NVTBIT(29,24)			// Hardware Memory On-the-Fly Access Image Buffer-Size Selection for Encode
	#define	FLY_TYPE	NVTBIT(23,22)			// 
	#define	BSF_SEL		NVTBIT(17,8)			// Memory On-the-Fly Access Bitstream Buffer-Size Selection
	#define	FLY_ON		BIT7				// Hardware Memory On-the-Fly Access Mode
	#define	IP_SF_ON	BIT3				// Software Memory On-the-Fly Access Mode for Data Input
	#define	OP_SF_ON	BIT2				// Software Memory On-the-Fly Access Mode for Data Output
	#define	ENC_MODE	NVTBIT(1,0)			// JPEG Memory Address Mode Control

#define	REG_JPSCALU		(JPEG_BASE+0x54)			// R/W: JPEG Primary Scaling-Up Control Register
	#define	JPSCALU_8X	BIT6				// Primary Image Up-Scaling For Encode
	#define	A_JUMP		BIT2				// Reserve Buffer Size In JPEG Bit-stream For Software Application

			
#define	REG_JPSCALD		(JPEG_BASE+0x58)		// R/W: JPEG Primary Scaling-Down Control Register
	#define	PSX_ON		BIT15				// Primary Image Horizontal Down-Scaling For Encode/Decode
	#define	PS_LPF_ON	BIT14				// Primary Image Down-Scaling Low Pass Filter For Decode
	#define	PSCALX_F	NVTBIT(12,8)			// Primary Image Horizontal Down-Scaling Factor
	#define	PSCALY_F	NVTBIT(5,0)			// Primary Image Vertical Down-Scaling Factor
	
#define	REG_JTSCALD		(JPEG_BASE+0x5C)			// R/W: JPEG Thumbnail  Scaling-Down Control Register
	#define	TSX_ON		BIT15				// Thumbnail Image Horizontal Down-Scaling For Encode/Decode
	#define	TSCALX_F	NVTBIT(12,8)			// Thumbnail Image Horizontal Down-Scaling Factor
	#define	TSCALY_F	NVTBIT(5,0)			// Thumbnail Image Vertical Down-Scaling Factor

#define	REG_JDBCR		(JPEG_BASE+0x60)			// R/W: JPEG Dual-Buffer Control Register
	#define	DBF_EN		BIT7				// Dual Buffering Control
	#define	IP_BUF		BIT4				// Input Dual Buffer Control

#define	REG_JRESERVE	(JPEG_BASE+0x70)			// R/W: JPEG Encode Primary Bit-stream Reserved Size Register
	#define	RES_SIZE	NVTBIT(15,0)			// Primary Encode Bit-stream Reserved Size

#define	REG_JOFFSET		(JPEG_BASE+0x74)			// R/W: JPEG Offset Between Primary & Thumbnail Register
	#define	OFFSET_SIZE	NVTBIT(23,0)		// Primary/Thumbnail Starting Address Offset Size

#define	REG_JFSTRIDE	(JPEG_BASE+0x78)			// R/W: JPEG Encode Bit-stream Frame Stride Register
	#define	F_STRIDE	NVTBIT(23,0)		// JPEG Encode Bit-stream Frame Stride

#define	REG_JYADDR0		(JPEG_BASE+0x7C)			// R/W: JPEG Y Component Frame Buffer-0 Starting Address Register
	#define	Y_IADDR0	NVTBIT(31,0)			// JPEG Y Component Frame Buffer-0 Starting Address

#define	REG_JUADDR0		(JPEG_BASE+0x80)			// R/W: JPEG U Component Frame Buffer-0 Starting Address Register
	#define	U_IADDR0	NVTBIT(31,0)			// JPEG U Component Frame Buffer-0 Starting Address

#define	REG_JVADDR0		(JPEG_BASE+0x84)			// R/W: JPEG V Component Frame Buffer-0 Starting Address Register
	#define	V_IADDR0	NVTBIT(31,0)			// JPEG V Component Frame Buffer-0 Starting Address

#define	REG_JYADDR1		(JPEG_BASE+0x88)			// R/W: JPEG Y Component Frame Buffer-1 Starting Address Register
	#define	Y_IADDR1	NVTBIT(31,0)			// JPEG Y Component Frame Buffer-1 Starting Address

#define	REG_JUADDR1		(JPEG_BASE+0x8C)			// R/W: JPEG U Component Frame Buffer-1 Starting Address Register
	#define	U_IADDR1	NVTBIT(31,0)			// JPEG U Component Frame Buffer-1 Starting Address

#define	REG_JVADDR1		(JPEG_BASE+0x90)			// R/W: JPEG V Component Frame Buffer-1 Starting Address Register
	#define	V_IADDR1	NVTBIT(31,0)			// JPEG V Component Frame Buffer-1 Starting Address

#define	REG_JYSTRIDE	(JPEG_BASE+0x94)			// R/W: JPEG Y Component Frame Buffer Stride Register
	#define	Y_STRIDE	NVTBIT(11,0)			// JPEG Y Component Frame Buffer Stride

#define	REG_JUSTRIDE	(JPEG_BASE+0x98)			// R/W: JPEG U Component Frame Buffer Stride Register
	#define	U_STRIDE	NVTBIT(11,0)			// JPEG U Component Frame Buffer Stride

#define	REG_JVSTRIDE	(JPEG_BASE+0x9C)			// R/W: JPEG V Component Frame Buffer Stride Register
	#define	V_STRIDE	NVTBIT(11,0)			// JPEG V Component Frame Buffer Stride

#define	REG_JIOADDR0	(JPEG_BASE+0xA0)			// R/W: JPEG Bit-stream Frame Buffer-0 Starting Address Register
	#define	IO_IADDR	NVTBIT(31,0)			// JPEG Bit-stream Frame Buffer-0 Starting Address

#define	REG_JIOADDR1 	(JPEG_BASE+0xA4)			// R/W: JPEG Bit-stream Frame Buffer-1 Starting Address Register
	#define	IO_IADDR1	NVTBIT(31,0)			// JPEG Bit-stream Frame Buffer-1 Starting Address

#define	REG_JPRI_SIZE 	(JPEG_BASE+0xA8)		// R  : JPEG Encode Primary Image Bit-stream Size Register
	#define	PRI_SIZE	NVTBIT(23,0)			// JPEG Primary Image Encode Bit-stream Size

#define	REG_JTHB_SIZE 	(JPEG_BASE+0xAC)		// R  : JPEG Encode Thumbnail Image Bit-stream Size Register
	#define	THB_SIZE	NVTBIT(15,0)			// JPEG Thumbnail Image Encode Bit-stream Size

#define	REG_JUPRAT 		(JPEG_BASE+0xB0)			// R/W: JPEG Encode Up-Scale Ratio Register
	#define	S_HEIGHT	NVTBIT(29,16)			// JPEG Image Height Up-Scale Ratio
	#define	S_WIDTH		NVTBIT(13,0)			// JPEG Image Width Up-Scale Ratio

#define	REG_JBSFIFO 	(JPEG_BASE+0xB4)			// R/W: JPEG Bit-stream FIFO Control Register
	#define	BSFIFO_HT	NVTBIT(6,4)			// Bit-stream FIFO High-Threshold Control
	#define	BSFIFO_LT	NVTBIT(2,0)			// Bit-stream FIFO Low-Threshold Control

#define	REG_JSRCH		(JPEG_BASE+0xB8)			// R/W: JPEG Bit-stream FIFO Control Register
	#define	JSRCH_JSRCH	NVTBIT(11,0)				// JPEG Encode Source Image Height
	
#define	REG_JQTAB0		(JPEG_BASE+0x100)		// R/W: JPEG Quantization-Table 0 Register
	
#define	REG_JQTAB1		(JPEG_BASE+0x140)		// R/W: JPEG Quantization-Table 1 Register
	
#define	REG_JQTAB2		(JPEG_BASE+0x180)		// R/W: JPEG Quantization-Table 2 Register


/* Advance Interrupt Controller (AIC) Registers */
#define AIC_BA		W55FA93_VA_IRQ		/* Interrupt Controller */
#define REG_AIC_SCR1    (AIC_BA+0x000)  
#define REG_AIC_SCR2    (AIC_BA+0x004)  
#define REG_AIC_SCR3    (AIC_BA+0x008)  
#define REG_AIC_SCR4    (AIC_BA+0x00c)  
#define REG_AIC_SCR5    (AIC_BA+0x010)  
#define REG_AIC_SCR6    (AIC_BA+0x014)  
#define REG_AIC_SCR7    (AIC_BA+0x018)  
#define REG_AIC_SCR8    (AIC_BA+0x01c)  
#define REG_AIC_IRSR    (AIC_BA+0x100)   /* Interrupt raw status register */
#define REG_AIC_IASR    (AIC_BA+0x104)   /* Interrupt active status register */
#define REG_AIC_ISR     (AIC_BA+0x108)   /* Interrupt status register */
#define REG_AIC_IPER    (AIC_BA+0x10C)   /* Interrupt priority encoding register */
#define REG_AIC_ISNR    (AIC_BA+0x110)   /* Interrupt source number register */
#define REG_AIC_IMR     (AIC_BA+0x114)   /* Interrupt mask register */
#define REG_AIC_OISR    (AIC_BA+0x118)   /* Output interrupt status register */
#define REG_AIC_MECR    (AIC_BA+0x120)   /* Mask enable command register */
#define REG_AIC_MDCR    (AIC_BA+0x124)   /* Mask disable command register */
#define REG_AIC_SSCR    (AIC_BA+0x128)	 /* Source set command register */
#define REG_AIC_SCCR    (AIC_BA+0x12C)   /* Source command register*/
#define REG_AIC_EOSCR   (AIC_BA+0x130)   /* End of service command register */
#define AIC_IPER	(0x10C)
#define AIC_ISNR	(0x110)
#define AIC_EOSCR	(0x130)


#define UART_BA		W55FA93_VA_UART		/* UART Control (console) */

#define W55FA93_COM_TX	  (0x00)
#define W55FA93_COM_RX	  (0x00)
#define W55FA93_COM_IER	  (0x04)
#define W55FA93_COM_FCR	  (0x08)
#define W55FA93_COM_LCR	  (0x0C)
#define W55FA93_COM_MCR	  (0x10)
#define W55FA93_COM_MSR	  (0x14)
#define W55FA93_COM_FSR	  (0x18)
#define W55FA93_COM_ISR	  (0x1C)
#define W55FA93_COM_TOR	  (0x20)
#define W55FA93_COM_BAUD  (0x24)

#define UARTx_FCR_FIFO_LEVEL1	0x00
#define UARTx_FCR_FIFO_LEVEL4	0x10
#define UARTx_FCR_FIFO_LEVEL8	0x20
#define UARTx_FCR_FIFO_LEVEL14	0x30
#define UARTx_FCR_FIFO_LEVEL30	0x40
#define UARTx_FCR_FIFO_LEVEL46	0x50
#define UARTx_FCR_FIFO_LEVEL62	0x60

#define UART_FCR_RFR	0x02
#define UART_FCR_TFR	0x04

#define UART_TXRXFIFO_RESET	(UART_FCR_RFR | UART_FCR_TFR)

#define UART_FSR_ROE	0x00000000	// Rx Overrun error
#define UART_FSR_PE	0x00000010	// Parity error
#define UART_FSR_FE	0x00000020	// Frame error
#define UART_FSR_BI	0x00000040	// Break interrupt
#define UART_FSR_RFE	0x00004000	// Rx FIFO empty
#define UART_FSR_RFF	0x00008000	// Rx FIFO full
#define UART_FSR_RPMASK	(0x00003F00)	// Rx FIFO pointer
#define UART_FSR_TFE	0x00400000	// Tx FIFO empty
#define UART_FSR_TFF	0x00800000	// Tx FIFO full
#define UART_FSR_TPMASK	(0x003F0000)	// Tx FIFO pointer
#define UART_FSR_TOE	0x01000000	// Tx Overrun error
#define UART_FSR_TEMT	0x10000000	// Transmitter empty
	
#define W55FA93_FSRSTAT_ANY	(UART_FSR_ROE | UART_FSR_TOE | UART_FSR_FE | UART_FSR_BI)

#define UART_LCR_WLEN5	0x00
#define UART_LCR_WLEN6	0x01
#define UART_LCR_WLEN7	0x02
#define UART_LCR_WLEN8	0x03
#define UART_LCR_CSMASK	(0x3)
#define UART_LCR_PARITY	0x08
#define UART_LCR_NPAR	0x00
#define UART_LCR_OPAR	0x00
#define UART_LCR_EPAR	0x10
#define UART_LCR_PMMASK	(0x30)
#define UART_LCR_SPAR	0x20
#define UART_LCR_SBC	0x40
#define UART_LCR_NSB	0x00
#define UART_LCR_NSB1_5	0x04

#define UART_IER_TOUT	BIT11
#define UART_IER_RTO	BIT4
#define UART_IER_MSI	BIT3
#define UART_IER_RLSI	BIT2
#define UART_IER_THRI	BIT1
#define UART_IER_RDI	BIT0

#define UART_ISR_EDMA_RX_Flag		BIT31		// EDMA RX Mode Flag
#define UART_ISR_HW_Wake_INT		BIT30		// Wake up Interrupt pin status
#define UART_ISR_HW_Buf_Err_INT		BIT29		// Buffer Error Interrupt pin status
#define UART_ISR_HW_Tout_INT		BIT28		// Time out Interrupt pin status
#define UART_ISR_HW_Modem_INT		BIT27		// MODEM Status Interrupt pin status
#define UART_ISR_HW_RLS_INT		BIT26		// Receive Line Status Interrupt pin status
#define UART_ISR_Rx_ack_st		BIT25		// TX ack pin status
#define UART_ISR_Rx_req_St		BIT24		// TX req pin status
#define UART_ISR_EDMA_TX_Flag		BIT23		// EDMA TX Mode Flag
#define UART_ISR_HW_Wake_IF		BIT22		// Wake up Flag
#define UART_ISR_HW_Buf_Err_IF		BIT21		// Buffer Error Flag
#define UART_ISR_HW_Tout_IF		BIT20		// Time out Flag
#define UART_ISR_HW_Modem_IF		BIT19		// MODEM Status Flag
#define UART_ISR_HW_RLS_IF		BIT18		// Receive Line Status Flag
#define UART_ISR_Tx_ack_st		BIT17		// TX ack pin status
#define UART_ISR_Tx_req_St		BIT16		// TX req pin status
#define UART_ISR_Soft_RX_Flag		BIT15		// Software RX Mode Flag
#define UART_ISR_Wake_INT		BIT14		// Wake up Interrupt pin status
#define UART_ISR_Buf_Err_INT		BIT13		// Buffer Error Interrupt pin status
#define UART_ISR_Tout_INT		BIT12		// Time out interrupt Interrupt pin status
#define UART_ISR_Modem_INT		BIT11		// MODEM Status Interrupt pin status
#define UART_ISR_RLS_INT		BIT10		// Receive Line Status Interrupt pin status
#define UART_ISR_THRE_INT		BIT9		// Transmit Holding Register Empty Interrupt pin status
#define UART_ISR_RDA_INT		BIT8		// Receive Data Available Interrupt pin status
#define UART_ISR_Soft_TX_Flag		BIT7		// Software TX Mode Flag
#define UART_ISR_Wake_IF		BIT6		// Wake up Flag
#define UART_ISR_Buf_Err_IF		BIT5		// Buffer Error Flag
#define UART_ISR_Tout_IF		BIT4		// Time out interrupt Flag
#define UART_ISR_Modem_IF		BIT3		// MODEM Status Flag
#define UART_ISR_RLS_IF			BIT2		// Receive Line Status Flag
#define UART_ISR_THRE_IF		BIT1		// Transmit Holding Register Empty Flag
#define UART_ISR_RDA_IF			BIT0		// Receive Data Available Flag


/* SPU Control Registers */
#define SPU_BA		W55FA93_VA_SPU

#define REG_SPU_CTRL			(SPU_BA+0x00)		// SPU control and status register
	#define SPU_FIFO_SIZE	 		NVTBIT(28,24)		// FIFO buffer size control 
	#define SPU_SWRST				BIT16				// SPU SW reset
	#define SPU_I2S_JUSTIFIED		BIT10				// SPU I2S/Justified interface select
	#define SPU_I2S_EN				BIT8				// SPU I2S interface enable
	#define SPU_EN					BIT0				// SPU enable/disable	

#define REG_SPU_DAC_PAR			(SPU_BA+0x04)		// DAC parameter register
	#define DAC_ZERO_EN				BIT28				// DAC zero cross detection for DAC volume control
	#define ZERO_EN					BIT25				// Zero cross detection enable
	#define EQU_EN					BIT24				// Equalizer enable
	#define DISCHARGE_EN			BIT14				// Eenable the discharge path for output coupling capacitor	
	#define DISCHARGE_CON	 		NVTBIT(13,12)		// Control the register on the discharging path
	#define HP_POP_CLEAR_EN			BIT6				// Headphone output pop sound clear enable
	#define POP_CON					NVTBIT(5,4)			// Pop noise cintrol register

#define REG_SPU_DAC_VOL			(SPU_BA+0x08)		// Sub block reset control
	#define DWA_SEL					NVTBIT(31,30)		// ???
	#define LOW_POWER_EN			BIT28				// Low power enable for volume control OP
	#define ANA_PD					NVTBIT(23,16)		// Audio DAC Power Down
	#define LHPVL					NVTBIT(13,8)		// Headphone Left Channel Volume
	#define RHPVL					NVTBIT(5,0)			// Headphone Right Channel Volume

#define REG_SPU_EQGain0			(SPU_BA+0x0C)		// Equalizer bands 08 - 01 gain control
	#define Gain08					NVTBIT(31,28)		// Gain08 control 
	#define Gain07					NVTBIT(27,24)		// Gain07 control 
	#define Gain06					NVTBIT(23,20)		// Gain06 control 
	#define Gain05					NVTBIT(19,16)		// Gain05 control 
	#define Gain04					NVTBIT(15,12)		// Gain04 control 
	#define Gain03					NVTBIT(11,8)		// Gain03 control 
	#define Gain02					NVTBIT(07,4)		// Gain02 control 
	#define Gain01					NVTBIT(03,0)		// Gain01 control 

#define REG_SPU_EQGain1			(SPU_BA+0x10)		// Equalizer bands 10 - 09 and DC gain control
	#define Gaindc					NVTBIT(19,16)		// DC control 
	#define Gain10					NVTBIT(07,4)		// Gain10 control 
	#define Gain09					NVTBIT(03,0)		// Gain09 control 

#define REG_SPU_CH_EN			(SPU_BA+0x14)		// Channel enable register
	
#define REG_SPU_CH_IRQ			(SPU_BA+0x018)		// Channel iterrupt request flag register

#define REG_SPU_CH_PAUSE		(SPU_BA+0x1C)		// Channel PAUSE register

#define REG_SPU_CH_CTRL			(SPU_BA+0x20)		// Channel control register
	#define CH_NO					NVTBIT(28,24)		// Select chanel index number
	#define FN_IRQ_FG				BIT12				// Channel function done interrupt flag
	#define FN_IRQ_EN				BIT8				// Channel function done interrupt enable
//	#define CH_RST					BIT8				// Channel reset register	
	#define UP_IRQ					BIT7				// Interrupt for DFA update in partial update function	
	#define UP_DFA					BIT6				// DFA update in partial update function
	#define UP_PAN					BIT5				// PAN update in partial update function	
	#define UP_VOL					BIT4				// Volume update in partial update function		
	#define UP_PAUSE_ADDR			BIT3				// Pause Address update in partial update function	(only for mono/stereo PCM16 use)	
//	#define FN_IRQ_EN				BIT2				// Channel function done interrupt enable	
	#define CH_FN					NVTBIT(1,0)			// Channel function register
	
#define REG_SPU_S_ADDR			(SPU_BA+0x24)		// Source start (base) address register

#define REG_SPU_M_ADDR			(SPU_BA+0x28)		// Threshold address register

#define REG_SPU_E_ADDR			(SPU_BA+0x2C)		// End start address register

#define REG_SPU_TONE_PULSE		(SPU_BA+0x28)		// Tone Pulse control register
	#define TONE_P1					NVTBIT(31,16)		// Tone Pulse 1 
	#define TONE_P0					NVTBIT(15,0)		// Tone Pulse 0 
	
#define REG_SPU_TONE_AMP		(SPU_BA+0x2C)		// Tone Amplitude control register
	#define TONE_AMP1				NVTBIT(31,16) 		// Tone Amplitude 1  
	#define TONE_AMP0				NVTBIT(15,0)		// Tone Amplitude 0  

#define REG_SPU_CH_PAR_1		(SPU_BA+0x30)		// Channel parameter 1 register
	#define CH_VOL					NVTBIT(30,24) 		// Channel volume register
	#define PAN_L					NVTBIT(20,16)		// Output right channel PAN
	#define PAN_R					NVTBIT(12,8)		// Output left channel PAN
	#define SRC_TYPE				NVTBIT(2,0)			// Channel sound type

#define REG_SPU_CH_PAR_2		(SPU_BA+0x34)		// Channel parameter 2 register
	#define DFA						NVTBIT(12,0)		// DFA

#define REG_SPU_CH_EVENT		(SPU_BA+0x38)		// DMA down counter register
	#define SUB_IDX					NVTBIT(31,24)		// Sub-index of user event
	#define EVENT_IDX				NVTBIT(23,16)		// Index of user event	
	#define EV_USR_FG				BIT13				// User event interrupt flag
	#define EV_SLN_FG				BIT12				// Slient event interrupt flag	
	#define EV_LP_FG				BIT11				// Loop Start event interrupt flag		
	#define EV_PAUSE_FG				BIT11				// Pause Address interrupt flag		
	#define EV_END_FG				BIT10				// End event interrupt flag			
	#define END_FG					BIT9				// End address interrupt flag
	#define TH_FG					BIT8				// Threshold address interrupt flag	

	#define AT_CLR_EN				BIT7				// Enable Bit for auto interrupt flag clear after read event register
	#define EV_USR_EN				BIT5				// Enable Bit for User event interrupt flag
	#define EV_SLN_EN				BIT4				// Enable Bit for Slient event interrupt flag	
	#define EV_LP_EN				BIT3				// Enable Bit for Loop Start event interrupt flag		
	#define EV_PAUSE_EN				BIT3				// Enable Bit for Pause Address event interrupt flag			
	#define EV_END_EN				BIT2				// Enable Bit for End event interrupt flag			
	#define END_EN					BIT1				// Enable Bit for End address
	#define TH_EN					BIT0				// Enable Bit for Threshold address	

#define REG_SPU_CUR_ADDR		(SPU_BA+0x40)			// DMA down counter register

#define REG_SPU_LP_ADDR			(SPU_BA+0x44)			// DMA down counter register
#define REG_SPU_PA_ADDR			(SPU_BA+0x44)			// Pause Address for mono/stereo PCM16 

#define REG_SPU_P_BYTES			(SPU_BA+0x48)			// Chanel loop paly byte conuts

/* I2S Control Registers */
#define I2S_BA		W55FA93_VA_I2SM

#define REG_I2S_ACTL_CON		(I2S_BA+0x00)		// Audio Control Register
	#define	R_DMA_IRQ_EN			BIT21				// Recording DMA Interrupt Request enable Bit
	#define P_DMA_IRQ_EN			BIT20				// Playback DMA Interrupt Request enable Bit	
	#define R_FIFO_FULL_IRQ_EN		BIT19				// Recording FIFO full Interrupt Request enable Bit		
	#define R_FIFO_EMPTY_IRQ_EN		BIT18				// Recording FIFO empty Interrupt Request enable Bit			
	#define P_FIFO_FULL_IRQ_EN		BIT17				// Playback FIFO full Interrupt Request enable Bit		
	#define P_FIFO_EMPTY_IRQ_EN		BIT16				// Playback FIFO empty Interrupt Request enable Bit			
	#define R_DMA_IRQ_SEL			NVTBIT(15,14)		// Recording DMA Interrupt Request selection Bits	
	#define P_DMA_IRQ_SEL			NVTBIT(13,12)		// Playback DMA Interrupt Request selection Bits		

	#define R_DMA_IRQ				BIT11				// Playback DMA Interrupt Request Bit
	#define P_DMA_IRQ				BIT10				// Recording DMA Interrupt Request Bit
	#define I2S_BITS_16_24			BIT9				// 16/24 bits selection
	#define FIFO_TH					BIT7				// FIFO Threshold Control Bit
	#define IRQ_DMA_CNTER_EN		BIT4				// IRQ_DMA counter function enable Bit
	#define IRQ_DMA_DATA_ZERO_EN	BIT3				// IRQ_DMA_DATA zero and sign detect enable Bit
	#define I2S_EN					BIT1				// I2S interface enable Bit	

#define	REG_I2S_ACTL_RESET		(I2S_BA+0x04)		// Sub block reset control
	#define ACTL_RESET_				BIT16				// Audio Controller Reset Control Bit
	#define RECORD_SINGLE			NVTBIT(15,14)		// Record Single/Dual Channel Select Bits

	#define RECORD_RIGHT_CHNNEL 	BIT15
	#define RECORD_LEFT_CHNNEL 		BIT14
	#define PLAY_STEREO				BIT12				// Playback Single/Dual Channel Select Bits	
	
	#define I2S_RECORD				BIT6				// I2S Record Control Bit
	#define I2S_PLAY				BIT5				// I2S Playback Control Bit
	#define DMA_CNTER_EN			BIT4				// DMA counter function enable Bit
	#define DMA_DATA_ZERO_EN		BIT3				// DMA_DATA zero and sign detect enable Bit
	#define AC_RESET				BIT1				// AC link Sub Block RESET Control Bit
	#define I2S_RESET				BIT0				// I2S Sub Block RESET Control Bit

#define	REG_I2S_ACTL_RDSTB		(I2S_BA+0x08)		// DMA record destination base address
#define REG_I2S_ACTL_RDST_LENGTH (I2S_BA+0x0C)		// DMA record destination address length
#define REG_I2S_ACTL_RDSTC		(I2S_BA+0x10)		// DMA record destination current address
#define REG_I2S_ACTL_PDSTB		(I2S_BA+0x14)		// DMA play destination base address
#define REG_I2S_ACTL_PDST_LENGTH (I2S_BA+0x18)		// DMA play destination address length
#define REG_I2S_ACTL_PDSTC		(I2S_BA+0x1C)		// DMA play destination current address

#define REG_I2S_ACTL_RSR 		(I2S_BA+0x20)		// Audio controller FIFO and DMA status register for playback
	#define R_DMA_RIA_SN			NVTBIT(7,5)			// Recording DMA inidicative address selection number Bits
	#define R_FIFO_FULL				BIT2				// Playback FIFO Full Indicatior Bit	
	#define R_FIFO_EMPTY			BIT1				// Playback FIFO Empty Indicatior Bit
	#define R_DMA_RIA_IRQ			BIT0				// Recording DMA inidicative address interrupt Request Bit

#define REG_I2S_ACTL_PSR		(I2S_BA+0x24)		// Audio controller FIFO and DMA status register for playback
	#define P_DMA_RIA_SN			NVTBIT(7,5)			// Playback DMA inidicative address selection number Bits
	#define DMA_CNTER_IRQ			BIT4				// DMA counter IRQ
	#define DMA_DATA_ZERO_IRQ		BIT3				// DMA_DATA zero IRQ
	#define P_FIFO_FULL				BIT2				// Playback FIFO Full Indicatior Bit
	#define P_FIFO_EMPTY			BIT1				// Playback FIFO Empty Indicatior Bit	
	#define P_DMA_RIA_IRQ			BIT0				// Playback DMA inidicative address interrupt Request Bit

#define REG_I2S_ACTL_I2SCON		(I2S_BA+0x28)		// I2S controll register
	#define PRS						NVTBIT(19,16)		// I2S Frequency Pre-scaler Selection Bits
//	define	MCLK_SEL1				BIT9				// MCLK clock selection when MCLK_CON is active
//	#define MCLK_CON				BIT8				// MCLK clock selection
	#define BCLK_SEL				NVTBIT(7,6)			// I2S Serial Data Clock Frequency Selection Bit
	#define FS_SEL					BIT5				// I2S Sampling Frequency Selection Bit
	#define MCLK_SEL				BIT4				// I2S MCLK Output Selection Bit
	#define I2S_FORMAT				BIT3				// I2S Format Selection Bit

#define REG_I2S_ACTL_COUNTER	(I2S_BA+0x2C)		// DMA down counter register


/* ADC Control Registers */
#define ADC_BA		W55FA93_VA_ADC
#define REG_ADC_CON		(ADC_BA+0x0000)	// R/W	ADC control register
	#define ADC_SYSCK_EN	BIT29			// R/W	Enable to System Clock Divider to ADC Input Clock
	#define ADC_SYS_DIV 	NVTBIT(28, 24)		//R/W	System Clock Divider to ADC Input Clock
	#define WT_INT_EN	BIT23			// R/W	Waiting for trigger interrupt enable bit
	#define LVD_INT_EN	BIT22			// R/W	Low voltage detector interrupt enable bit
	#define ADC_INT_EN	BIT21			// R/W	ADC interrupt enable bit
	#define WT_INT		BIT20			// R/W	Waiting for trigger interrupt status bit
	#define LVD_INT		BIT19			// R/W	Low voltage detector (LVD) interrupt status bit
	#define ADC_INT		BIT18			// R/W	ADC interrupt status bit
	#define ADC_CON_ADC_EN	BIT17			// R/W	ADC block enable bit 
	#define ADC_RST	BIT16				// R/W	ADC reset control bit
	#define ADC_TSC_MODE	NVTBIT(15,14)		// R/W	The touch screen conversion mode control bits
	#define ADC_CONV	BIT13			// R	ADC conversion control bit
	#define ADC_READ_CONV	BIT12		// R/W	This bit control if next conversion start after ADC_XDATA register is read in normal conversion mode.
	#define ADC_MUX		NVTBIT(11,9)		// R/W	These bits select ADC input from the 8 analog inputs in normal conversion mode.
	#define ADC_DIV		NVTBIT(8,1)		// R/W	The ADC input clock divider. The real ADC operating clock is the input clock divide (ADC_DIV+1).
	#define ADC_FINISH	BIT0				// R	This bit indicate the ADC is in conversion or not

#define REG_ADC_TSC		(ADC_BA+0x0004)	// R/W	Touch screen control register	
	#define ADC_TSC_MAV_EN	BIT9			// R/W  MAV Filter Enable/Disable for the Touch Screen AutoMode
	#define ADC_TSC_XY	BIT8			// R/W	This bit control the X-position or Y-position detection when in semi-auto conversion mode
	#define ADC_TSC_XP	BIT7			// R/W	This bit control the interface to XP of touch screen when in normal conversion mode
	#define ADC_TSC_XM	BIT6			// R/W	This bit control the interface to XM of touch screen when in normal conversion mode
	#define ADC_TSC_YP	BIT5			// R/W	This bit control the interface to YP of touch screen when in normal conversion mode
	#define ADC_TSC_YM	BIT4			// R/W	This bit control the interface to YM of touch screen when in normal conversion mode
	#define ADC_PU_EN	BIT3			// R/W	This bit control the internal pull up PMOS in switch box is enable or disable
	#define ADC_TSC_TYPE	NVTBIT(2,1)		// R/W	The touch screen type selection bits
	#define ADC_UD		BIT0			// R	The up down state for stylus in waiting for trigger mode


#define REG_ADC_DLY		(ADC_BA+0x0008)	// R/W	ADC delay register
	#define ADC_DELAY	NVTBIT(17,0)		// R/W	Delay for Conversion. 

#define REG_ADC_XDATA	(ADC_BA+0x000C)	// R	10 bits ADC XDATA register
#define REG_ADC_YDATA	(ADC_BA+0x0010)	// R	10 bits ADC YDATA register

#define REG_LV_CON		(ADC_BA+0x0014)	// R/W	Low Voltage Detector Control register
	#define LV_EN		BIT3			// R/W	Low voltage detector enable control pin
	#define SW_CON		NVTBIT(2,0)		// R/W	The low voltage detector voltage level switch control bits

#define REG_LV_STS		(ADC_BA+0x0018)	// R	Low Voltage Detector Status register
	#define LV_status	BIT0			// R	Low voltage detector status pin

#define REG_AUDIO_CON	(ADC_BA+0x001C)	// R/W: ADC Control Register
	#define AUDIO_INT_MODE 	NVTBIT(31,30) 	// Audio interrupt mode selection
	#define AUDIO_INT		    	BIT29			// Audio interrupt flag bits
	#define AUDIO_INT_EN	  	BIT28			// Audio interrupt flag bits
	#define AUDIO_VOL_EN		BIT27			// Volume control enable bit
	#define AUDIO_HPEN		BIT26			// Record path high pass enable bit
	#define AUDIO_EN			BIT25			// Record enable
	#define AUDIO_RESET		BIT24			// Record path high pass enable bit
	#define AUDIO_CCYCLE	NVTBIT(23,16)	// Audio conversion cycle (Minimum value = 34)
	#define AUDIO_DATA		NVTBIT(15,0)		// Last converted audio data before AUD_INT

#define REG_AUDIO_BUF0 	(ADC_BA+0x0020)	// R/W: Audio data register 0
#define REG_AUDIO_BUF1 	(ADC_BA+0x0024)	// R/W: Audio data register 1
#define REG_AUDIO_BUF2 	(ADC_BA+0x0028)	// R/W: Audio data register 2
#define REG_AUDIO_BUF3 	(ADC_BA+0x002C)	// R/W: Audio data register 3
	#define AUDIO_DATA1	NVTBIT(31,16)		// Converted audio dat 1 in bufferx
	#define AUDIO_DATA0	NVTBIT(15,0)			// Converted audio dat 0 in bufferx

#define REG_AGCP1 		(ADC_BA+0x0030)	// R/W: AGC Parameter Register Setting
	#define EDMA_MODE	BIT31				// EDMA mode Enable/Dislabe
	#define MAXGAIN		NVTBIT(22,20)		// AGC MAXGAIN Control Register
	#define MINGAIN		NVTBIT(18,16)		// AGC MINGAIN Control Register
	#define OTL			NVTBIT(15,12)		// Output Target Level
	#define UPBAND		BIT11				// Up band for AGC
	#define DOWNBAND	BIT10				// Down band for AGC
	#define PRAGA		NVTBIT(9,8)				// Pre- Amplifer Gain Control
	#define AUDIO_VOL	NVTBIT(5,0)			// Audio Volume Control

#define REG_AGC_CON 	(ADC_BA+0x0034)	// R/W: AGC Control Register
	#define NG_EN		BIT31				// Noise gate enable 
	#define AGC_EN		BIT30				// Auto gain control enable bit
	#define PAVG_MODE	NVTBIT(29,28)		// Peak average mode
	#define PERIOD		NVTBIT(25,16)		// Period
	#define AGAIN_STEP 	BIT15				// Up band for AGC
	#define NG_LEVEL		NVTBIT(13,12)		// Down band for AGC
	#define ATTACK		NVTBIT(11,8)			// Attack time 
	#define RECOVERY	NVTBIT(7,4)			// Recovery time 
	#define HOLD			NVTBIT(3,0)			// Hold time

#define REG_OPOC	 	(ADC_BA+0x0038)	// R/W: AGC Parameter Register Setting
	#define MUTE_SW		BIT31				// Mute control under software mode
	#define OOC			BIT30				// Hardware OP Offset Calculation Enable = 0. Disable =1 
	#define OPOCM		BIT29				// OP Offset Cancellation Method
	#define OPOC_SW		NVTBIT(28, 24)		// OP Offset Cancellation
	#define OPOC_TCSN	NVTBIT(17, 16)		// OP Offset Total Calculation Sample Number 
	#define OPOC_DSC	NVTBIT(15, 0)			// OP Offset Caculation Delay Sample Count 

#define REG_OPOCS1 		(ADC_BA+0x0040)	// R: OP Offset Calculation Status
	#define OP_OFFSET_CAL	NVTBIT(29, 20)	// The OP Offset value after hardware calculation
	#define ADC_DATA_SUM	NVTBIT(19, 0)		// ADC Data Sum 

#define REG_TSC_SORT10 	(ADC_BA+0x0070)	// 10th Touch Screen MAV (Minimum)
#define REG_TSC_SORT9 	(ADC_BA+0x0074)	// 9th Touch Screen MAV
#define REG_TSC_SORT8 	(ADC_BA+0x0078)	// 8th Touch Screen MAV
#define REG_TSC_SORT7 	(ADC_BA+0x007C)	// 7th Touch Screen MAV
#define REG_TSC_SORT6 	(ADC_BA+0x0080)	// 6th Touch Screen MAV
#define REG_TSC_SORT5 	(ADC_BA+0x0084)	// 5th Touch Screen MAV
#define REG_TSC_SORT4 	(ADC_BA+0x0088)	// 4th Touch Screen MAV
#define REG_TSC_SORT3 	(ADC_BA+0x008C)	// 3rd Touch Screen MAV
#define REG_TSC_SORT2 	(ADC_BA+0x0090)	// 2nd Touch Screen MAV
#define REG_TSC_SORT1 	(ADC_BA+0x0094)	// 1st Touch Screen MAV	(Maximum)
	#define Y_MAV		NVTBIT(25,16)			// MAV Y-Data
	#define X_MAV		NVTBIT(9,0)				// MAV X-Data

#define REG_TSC_MAV_X 	(ADC_BA+0x0098)	// Touch Screen MAV X-Data
	#define X_MAV_AVG	NVTBIT(9, 0)			// MAV X-Data
#define REG_TSC_MAV_Y	(ADC_BA+0x009C)	// Touch Screen MAV Y-Data
	#define Y_MAV_AVG	NVTBIT(9, 0)			// MAV Y-Data

/* GPIO Control Registers */
#define GPIO_BA		W55FA93_VA_GPIO		/* GPIO Control */

#define REG_GPIOA_OMD   (GPIO_BA+0x0000)   // GPIO port A bit Output mode Enable
#define REG_GPIOA_PUEN  (GPIO_BA+0x0004)	 // GPIO port A Bit Pull-up Resistor Enable
#define REG_GPIOA_DOUT  (GPIO_BA+0x0008)   // GPIO port A data output value 
#define REG_GPIOA_PIN   (GPIO_BA+0x000C)	 // GPIO port A Pin Value

#define REG_GPIOB_OMD   (GPIO_BA+0x0010)   // GPIO port B bit Output mode Enable
#define REG_GPIOB_PUEN  (GPIO_BA+0x0014)	 // GPIO port B Bit Pull-up Resistor Enable
#define REG_GPIOB_DOUT  (GPIO_BA+0x0018)   // GPIO port B data output value 
#define REG_GPIOB_PIN   (GPIO_BA+ 0x001C)	 // GPIO port B Pin Value

#define REG_GPIOC_OMD   (GPIO_BA+0x0020)  // GPIO port C bit Output mode Enable
#define REG_GPIOC_PUEN  (GPIO_BA+0x0024)	// GPIO port C Bit Pull-up Resistor Enable
#define REG_GPIOC_DOUT  (GPIO_BA+0x0028)  // GPIO port C data output value 

#define REG_GPIOC_PIN   (GPIO_BA+0x002C)	// GPIO port C Pin Value

#define REG_GPIOD_OMD   (GPIO_BA+0x0030)  // GPIO port D bit Output mode Enable
#define REG_GPIOD_PUEN  (GPIO_BA+0x0034)	// GPIO port D Bit Pull-up Resistor Enable
#define REG_GPIOD_DOUT  (GPIO_BA+0x0038)  // GPIO port D data output value 
#define REG_GPIOD_PIN   (GPIO_BA+0x003C)	// GPIO port D Pin Value

#define REG_GPIOE_OMD   (GPIO_BA+0x0040)  // GPIO port E bit Output mode Enable
#define REG_GPIOE_PUEN  (GPIO_BA+0x0044)	// GPIO port E Bit Pull-up Resistor Enable
#define REG_GPIOE_DOUT  (GPIO_BA+0x0048)  // GPIO port E data output value 
#define REG_GPIOE_PIN   (GPIO_BA+ 0x004C)	// GPIO port E Pin Value

#define REG_DBNCECON   (GPIO_BA+0x0070)  // External Interrupt Debounce Control

#define REG_IRQSRCGPA  (GPIO_BA+0x0080)  // GPIO Port A IRQ Source Grouping
#define REG_IRQSRCGPB  (GPIO_BA+0x0084)  // GPIO Port B IRQ Source Grouping
#define REG_IRQSRCGPC  (GPIO_BA+0x0088)  // GPIO Port C IRQ Source Grouping
#define REG_IRQSRCGPD  (GPIO_BA+0x008C)  // GPIO Port D IRQ Source Grouping
#define REG_IRQSRCGPE  (GPIO_BA+0x0090)  // GPIO Port E IRQ Source Grouping

#define REG_IRQENGPA   (GPIO_BA+0x00A0)  // GPIO Port A Interrupt Enable
#define REG_IRQENGPB   (GPIO_BA+0x00A4)  // GPIO Port B Interrupt Enable
#define REG_IRQENGPC   (GPIO_BA+0x00A8)  // GPIO Port C Interrupt Enable
#define REG_IRQENGPD   (GPIO_BA+0x00AC)  // GPIO Port D Interrupt Enable
#define REG_IRQENGPE   (GPIO_BA+0x00B0)  // GPIO Port E Interrupt Enable

#define REG_IRQLHSEL   (GPIO_BA+0x00C0)  // Interrupt Latch Trigger Selection Register

#define REG_IRQLHGPA   (GPIO_BA+0x00D0)  // GPIO Port A Interrupt Latch Value
#define REG_IRQLHGPB   (GPIO_BA+0x00D4)  // GPIO Port B Interrupt Latch Value
#define REG_IRQLHGPC   (GPIO_BA+0x00D8)  // GPIO Port C Interrupt Latch Value
#define REG_IRQLHGPD   (GPIO_BA+0x00DC)  // GPIO Port D Interrupt Latch Value
#define REG_IRQLHGPE   (GPIO_BA+0x00E0)  // GPIO Port E Interrupt Latch Value

#define REG_IRQTGSRC0   (GPIO_BA+0x00F0) // IRQ0~3 Interrupt Trigger Source Indicator from GPIO Port A and GPIO Port B
#define REG_IRQTGSRC1   (GPIO_BA+0x00F4) // IRQ0~3 Interrupt Trigger Source Indicator from GPIO Port C
#define REG_IRQTGSRC2   (GPIO_BA+0x00F8) // IRQ0~3 Interrupt Trigger Source Indicator from GPIO Port E


#define PA_VA_USB_BASE	W55FA93_VA_USBD
/* USB Device Control Registers */
#define REG_USBD_IRQ_STAT_L		(PA_VA_USB_BASE+0x00)		/* interrupt status low register */
#define REG_USBD_IRQ_ENB_L		(PA_VA_USB_BASE+0x08)		/* interrupt enable low register */
#define REG_USBD_IRQ_STAT		(PA_VA_USB_BASE+0x10)		/* usb interrupt status register */
#define REG_USBD_IRQ_ENB		(PA_VA_USB_BASE+0x14)		/* usb interrupt enable register */
#define REG_USBD_OPER			(PA_VA_USB_BASE+0x18)		/* usb operation register */
#define REG_USBD_FRAME_CNT		(PA_VA_USB_BASE+0x1c)		/* usb frame count register */
#define REG_USBD_ADDR			(PA_VA_USB_BASE+0x20)		/* usb address register */
#define REG_USBD_MEM_TEST		(PA_VA_USB_BASE+0x24)		/* usb test mode register */
#define REG_USBD_CEP_DATA_BUF	(PA_VA_USB_BASE+0x28)		/* control-ep data buffer register */
#define REG_USBD_CEP_CTRL_STAT	(PA_VA_USB_BASE+0x2c)		/* control-ep control and status register */
#define REG_USBD_CEP_IRQ_ENB	(PA_VA_USB_BASE+0x30)		/* control-ep interrupt enable register */
#define REG_USBD_CEP_IRQ_STAT	(PA_VA_USB_BASE+0x34)		/* control-ep interrupt status register */
#define REG_USBD_IN_TRNSFR_CNT	(PA_VA_USB_BASE+0x38)		/* in-transfer data count register */
#define REG_USBD_OUT_TRNSFR_CNT	(PA_VA_USB_BASE+0x3c)		/* out-transfer data count register */
#define REG_USBD_CEP_CNT		(PA_VA_USB_BASE+0x40)		/* control-ep data count register */
#define REG_USBD_SETUP1_0		(PA_VA_USB_BASE+0x44)		/* setup byte1 & byte0 register */
#define REG_USBD_SETUP3_2		(PA_VA_USB_BASE+0x48)		/* setup byte3 & byte2 register */
#define REG_USBD_SETUP5_4		(PA_VA_USB_BASE+0x4c)		/* setup byte5 & byte4 register */
#define REG_USBD_SETUP7_6		(PA_VA_USB_BASE+0x50)		/* setup byte7 & byte6 register */
#define REG_USBD_CEP_START_ADDR	(PA_VA_USB_BASE+0x54)		/* control-ep ram start address register */
#define REG_USBD_CEP_END_ADDR	(PA_VA_USB_BASE+0x58)		/* control-ep ram end address register */
#define REG_USBD_DMA_CTRL_STS	(PA_VA_USB_BASE+0x5c)		/* dma control and status register */
#define REG_USBD_DMA_CNT		(PA_VA_USB_BASE+0x60)		/* dma count register */
/* endpoint A */
#define REG_USBD_EPA_DATA_BUF	(PA_VA_USB_BASE+0x64)		/* endpoint A data buffer register */
#define REG_USBD_EPA_IRQ_STAT	(PA_VA_USB_BASE+0x68)		/* endpoint A interrupt status register */
#define REG_USBD_EPA_IRQ_ENB	(PA_VA_USB_BASE+0x6c)		/* endpoint A interrupt enable register */
#define REG_USBD_EPA_DATA_CNT	(PA_VA_USB_BASE+0x70)		/* data count available in endpoint A buffer */
#define REG_USBD_EPA_RSP_SC		(PA_VA_USB_BASE+0x74)		/* endpoint A response register set/clear */
#define REG_USBD_EPA_MPS		(PA_VA_USB_BASE+0x78)		/* endpoint A max packet size register */
#define REG_USBD_EPA_TRF_CNT	(PA_VA_USB_BASE+0x7c)		/* endpoint A transfer count register */
#define REG_USBD_EPA_CFG		(PA_VA_USB_BASE+0x80)		/* endpoint A configuration register */
#define REG_USBD_EPA_START_ADDR	(PA_VA_USB_BASE+0x84)		/* endpoint A ram start address register */
#define REG_USBD_EPA_END_ADDR	(PA_VA_USB_BASE+0x88)		/* endpoint A ram end address register */
/* endpoint B */
#define REG_USBD_EPB_DATA_BUF	(PA_VA_USB_BASE+0x8c)		/* endpoint B data buffer register */
#define REG_USBD_EPB_IRQ_STAT	(PA_VA_USB_BASE+0x90)		/* endpoint B interrupt status register */
#define REG_USBD_EPB_IRQ_ENB	(PA_VA_USB_BASE+0x94)		/* endpoint B interrupt enable register */
#define REG_USBD_EPB_DATA_CNT	(PA_VA_USB_BASE+0x98)		/* data count available in endpoint B buffer */
#define REG_USBD_EPB_RSP_SC		(PA_VA_USB_BASE+0x9c)		/* endpoint B response register set/clear */
#define REG_USBD_EPB_MPS		(PA_VA_USB_BASE+0xa0)		/* endpoint B max packet size register */
#define REG_USBD_EPB_TRF_CNT	(PA_VA_USB_BASE+0xa4)		/* endpoint B transfer count register */
#define REG_USBD_EPB_CFG		(PA_VA_USB_BASE+0xa8)		/* endpoint B configuration register */
#define REG_USBD_EPB_START_ADDR	(PA_VA_USB_BASE+0xac)		/* endpoint B ram start address register */
#define REG_USBD_EPB_END_ADDR	(PA_VA_USB_BASE+0xb0)		/* endpoint B ram end addresUSBDs register */
/* endpoint C */
#define REG_USBD_EPC_DATA_BUF	(PA_VA_USB_BASE+0xb4)		/* endpoint C data buffer register */
#define REG_USBD_EPC_IRQ_STAT	(PA_VA_USB_BASE+0xb8)		/* endpoint C interrupt status register */
#define REG_USBD_EPC_IRQ_ENB	(PA_VA_USB_BASE+0xbc)		/* endpoint C interrupt enable register */
#define REG_USBD_EPC_DATA_CNT	(PA_VA_USB_BASE+0xc0)		/* data count available in endpoint C buffer */
#define REG_USBD_EPC_RSP_SC		(PA_VA_USB_BASE+0xc4)		/* endpoint C response register set/clear */
#define REG_USBD_EPC_MPS		(PA_VA_USB_BASE+0xc8)		/* endpoint C max packet size register */
#define REG_USBD_EPC_TRF_CNT	(PA_VA_USB_BASE+0xcc)		/* endpoint C transfer count register */
#define REG_USBD_EPC_CFG		(PA_VA_USB_BASE+0xd0)		/* endpoint C configuration register */
#define REG_USBD_EPC_START_ADDR	(PA_VA_USB_BASE+0xd4)		/* endpoint C ram start address register */
#define REG_USBD_EPC_END_ADDR	(PA_VA_USB_BASE+0xd8)		/* endpoint C ram end address register */
/* endpoint D */
#define REG_USBD_EPD_DATA_BUF	(PA_VA_USB_BASE+0xdc)		/* endpoint D data buffer register */
#define REG_USBD_EPD_IRQ_STAT	(PA_VA_USB_BASE+0xe0)		/* endpoint D interrupt status register */
#define REG_USBD_EPD_IRQ_ENB	(PA_VA_USB_BASE+0xe4)		/* endpoint D interrupt enable register */
#define REG_USBD_EPD_DATA_CNT	(PA_VA_USB_BASE+0xe8)		/* data count available in endpoint D buffer */
#define REG_USBD_EPD_RSP_SC		(PA_VA_USB_BASE+0xec)		/* endpoint D response register set/clear */
#define REG_USBD_EPD_MPS		(PA_VA_USB_BASE+0xf0)		/* endpoint D max packet size register */
#define REG_USBD_EPD_TRF_CNT	(PA_VA_USB_BASE+0xf4)		/* endpoint D transfer count register */
#define REG_USBD_EPD_CFG		(PA_VA_USB_BASE+0xf8)		/* endpoint D configuration register */
#define REG_USBD_EPD_START_ADDR	(PA_VA_USB_BASE+0xfc)		/* endpoint D ram start address register */
#define REG_USBD_EPD_END_ADDR	(PA_VA_USB_BASE+0x100)		/* endpoint D ram end address register */
/* endpoint E */
#define REG_USBD_EPE_DATA_BUF	(PA_VA_USB_BASE+0x104)		/* endpoint E data buffer register */
#define REG_USBD_EPE_IRQ_STAT	(PA_VA_USB_BASE+0x108)		/* endpoint E interrupt status register */
#define REG_USBD_EPE_IRQ_ENB	(PA_VA_USB_BASE+0x10c)		/* endpoint E interrupt enable register */
#define REG_USBD_EPE_DATA_CNT	(PA_VA_USB_BASE+0x110)		/* data count available in endpoint E buffer */
#define REG_USBD_EPE_RSP_SC		(PA_VA_USB_BASE+0x114)		/* endpoint E response register set/clear */
#define REG_USBD_EPE_MPS		(PA_VA_USB_BASE+0x118)		/* endpoint E max packet size register */
#define REG_USBD_EPE_TRF_CNT	(PA_VA_USB_BASE+0x11c)		/* endpoint E transfer count register */
#define REG_USBD_EPE_CFG		(PA_VA_USB_BASE+0x120)		/* endpoint E configuration register */
#define REG_USBD_EPE_START_ADDR	(PA_VA_USB_BASE+0x124)		/* endpoint E ram start address register */
#define REG_USBD_EPE_END_ADDR	(PA_VA_USB_BASE+0x128)		/* endpoint E ram end address register */
/* endpoint F */
#define REG_USBD_EPF_DATA_BUF	(PA_VA_USB_BASE+0x12c)		/* endpoint F data buffer register */
#define REG_USBD_EPF_IRQ_STAT	(PA_VA_USB_BASE+0x130)		/* endpoint F interrupt status register */
#define REG_USBD_EPF_IRQ_ENB	(PA_VA_USB_BASE+0x134)		/* endpoint F interrupt enable register */
#define REG_USBD_EPF_DATA_CNT	(PA_VA_USB_BASE+0x138)		/* data count available in endpoint F buffer */
#define REG_USBD_EPF_RSP_SC		(PA_VA_USB_BASE+0x13c)		/* endpoint F response register set/clear */
#define REG_USBD_EPF_MPS		(PA_VA_USB_BASE+0x140)		/* endpoint F max packet size register */
#define REG_USBD_EPF_TRF_CNT	(PA_VA_USB_BASE+0x144)		/* endpoint F transfer count register */
#define REG_USBD_EPF_CFG		(PA_VA_USB_BASE+0x148)		/* endpoint F configuration register */
#define REG_USBD_EPF_START_ADDR	(PA_VA_USB_BASE+0x14c)		/* endpoint F ram start address register */
#define REG_USBD_EPF_END_ADDR	(PA_VA_USB_BASE+0x150)		/* endpoint F ram end address register */

#define REG_USBD_HEAD_WORD0		(PA_VA_USB_BASE+0x158)	/* first head data */
#define REG_USBD_HEAD_WORD1		(PA_VA_USB_BASE+0x15C)	/* second head data */
#define REG_USBD_HEAD_WORD2		(PA_VA_USB_BASE+0x160)	/* third head data */
#define REG_USBD_EPA_HEAD_CNT		(PA_VA_USB_BASE+0x164)	/* EPA head count */
#define REG_USBD_EPB_HEAD_CNT		(PA_VA_USB_BASE+0x168)	/* EPB head count */
#define REG_USBD_EPC_HEAD_CNT		(PA_VA_USB_BASE+0x16C)	/* EPC head count */
#define REG_USBD_EPD_HEAD_CNT		(PA_VA_USB_BASE+0x170)	/* EPD head count */
#define REG_USBD_EPE_HEAD_CNT		(PA_VA_USB_BASE+0x174)	/* EPE head count */
#define REG_USBD_EPF_HEAD_CNT		(PA_VA_USB_BASE+0x178)	/* EPF head count */

#define REG_USBD_AHB_DMA_ADDR	(PA_VA_USB_BASE+0x700)		/* AHB_DMA address register */
#define REG_USBD_PHY_CTL    (PA_VA_USB_BASE+0x704)           /* USB PHY control register */
	#define bisten		BIT0
	#define bisterr		BIT1
	#define siddq		BIT2
	#define xo_on		BIT3
	#define clk_sel		NVTBIT(5,4)
	#define refclk		BIT6
	#define clk48		BIT7
	#define vbus_detect	BIT8
	#define Phy_suspend	BIT9								
	#define Vbus_status	BIT31


#define VIDEOIN_BASE	W55FA93_VA_VIDEOIN	/* Videoin Control */
/*
 VideoIn Control Registers
*/
#define REG_VPECTL  		(VIDEOIN_BASE + 0x00)	// R/W: Video Pre-processor Control Register
			#define VPRST		BIT24					// Video Pre-processor Reset.
			#define UPDATE 		BIT20					// Video-In Update Register at New Frame	
			#define CAPONE		BIT16					// Video-In One Shutter
			#define VPRBIST		BIT8					// Video-In One Shutter
			#define PKEN		BIT6					// Packet Output Enable
			#define PNEN		BIT5					// Planar Output Enable
			#define ADDRSW		BIT3					// Packet Buffer Address select
			#define FBMODE		BIT2					// Packet Frame Buffer Control by FSC
			#define VPEEN		BIT0					// Planar Output Enable	

#define REG_VPEPAR		(VIDEOIN_BASE + 0x04)	// R/W: Video Pre-processor Parameter Register
			#define VPEBFIN		BIT28					// BIST Finish [Read Only]
			#define BFAIL		NVTBIT(27, 24)			// BIST Fail Flag [Read Only]
			#define FLDID		BIT20					// Field ID [Read Only]
			#define FLD1EN		BIT17					// Field 1 Input Enable
			#define FLD0EN		BIT16					// Field 0 Input Enable
			#define FLDDETP		BIT15					// Field Detect Position
			#define FLDDETM 	BIT14					// Field Detect Mode (By HSYNC or input FIELD PIN)
			#define FLDSWAP	BIT13					// Swap Input Field
			#define VSP			BIT10					// Sensor Vsync Polarity.
			#define HSP			BIT9					// Sensor Hsync Polarity
			#define PCLKP		BIT8					// Sensor Pixel Clock Polarity	
			#define PNFMT		BIT7					// Planar Output Format
			#define RANGE		BIT6					// Scale Input YUV CCIR601 color range to full range
			#define OUTFMT 	NVTBIT(5, 4)				// Image Data Format Output to System Memory.
			#define PDORD		NVTBIT(3, 2)				// Sensor Output Type
			#define SNRTYPE 	BIT1					// device is CCIR601 or CCIR656
			#define INFMT 		BIT0					// Sensor Output Format


#define REG_VPEINT  		(VIDEOIN_BASE + 0x08)	// R/W: Video Pre-processor Interrupt  Register
			#define MDINTEN	BIT20					// Motion Detection Interrupt Enable
			#define ADDRMEN  	BIT19					// Address Match Interrupt Enable.	
			#define MEINTEN	BIT17					// System Memory Error Interrupt Enable.
			#define VINTEN		BIT16					// Video Frame End Interrupt Enable.
			#define MDINT		BIT4					// Motion Detection Output Finsish Interrupt		
			#define ADDRMINT	BIT3					// Memory Address Match Interrupt Flag.
			#define MEINT		BIT1					// System Memory Error Interrupt. If read this bit shows 1, 
														// Memory Error occurs. Write 0 to clear it.
			#define VINT		BIT0					// Video Frame End Interrupt. If read this bit shows 1, 
														// received a frame complete. Write 0 to clear it.


#define REG_VPEMD  		(VIDEOIN_BASE + 0x10)	// R/W: Motion Detection  Register
			#define MDTHR	  	NVTBIT(20, 16)			// MD Differential Threshold	
			#define MDDF		NVTBIT(11, 10)			// MD Detect Frequence
			#define MDSM		BIT9					// MD Save Mode
			#define MDBS		BIT8					// MD Block Size
			#define MDEN		BIT0					// MD Enable
			
#define REG_MDADDR  		(VIDEOIN_BASE + 0x14)	// R/W: Motion Detection Output Address Register
#define REG_MDYADDR  	(VIDEOIN_BASE + 0x18)	// R/W: Motion Detection Output Address Register

#define REG_VPECWSP  	(VIDEOIN_BASE + 0x20)	// R/W:  Cropping Window Starting Address Register
			#define CWSPV		NVTBIT(26, 16)			// Cropping Window Vertical Starting Address
			#define CWSPH		NVTBIT(11, 0)			// Cropping Window Horizontal  Starting Address

#define REG_VPECWS	 	(VIDEOIN_BASE + 0x24)	// R/W:  Cropping Window Size Register
			#define CWSH			NVTBIT(26, 16)		// Cropping Image Window Height
			#define CWSW		NVTBIT(11, 0)			// Cropping Image Window Width

#define REG_VPEPKDS  		(VIDEOIN_BASE + 0x28)	// R/W  : Packet Scaling Vertical/Horizontal Factor Register
#define REG_VPEPNDS 		(VIDEOIN_BASE + 0x2C)	// R?W  : Planar Scaling Vertical/Horizontal Factor Register	
			#define DSVN		NVTBIT(31, 24)			// Scaling Vertical Factor N
			#define DSVM		NVTBIT(23, 16)			// Scaling Vertical Factor M
			#define DSHN		NVTBIT(15, 8)			// Scaling Horizontal Factor N
			#define DSHM		NVTBIT(7, 0)				// Scaling Horizontal Factor M

#define REG_VPEFRC  		(VIDEOIN_BASE + 0x30)	// R/W  : Scaling Frame Rate Factor Register
			#define FRCN		NVTBIT(13, 8)			// Scaling Frame Rate Factor N
			#define FRCM		NVTBIT(5, 0)				// Scaling Frame Rate Factor M
		
/*
#define REG_VWIDTH  		(VIDEOIN_BASE + 0x34)	// R/W  : Frame Output Pixel Straight Width Register
			#define PNOW		BIT(27, 16)				// Planar Frame Output Pixel Straight Width
			#define PKOW		BIT(11, 0)				// Packet Frame Output Pixel Straight Width
*/
#define REG_VSTRIDE 		(VIDEOIN_BASE + 0x34)	// R/W  : Frame Stride Register
			#define PNSTRIDE	NVTBIT(27, 16)			// Planar Frame Stride
			#define PKSTRIDE	NVTBIT(11, 0)			// Packet Frame Stride

#define REG_VFIFO 		(VIDEOIN_BASE + 0x3C)		// R/W  : FIFO threshold Register
			#define FTHP		NVTBIT(27, 24)			// Packet FIFO Threshold 
			#define PTHY		NVTBIT(19, 16)			// Planar Y FIFO Threshold 
			#define PTHU		NVTBIT(10, 8)			// Planar U FIFO Threshold 
			#define PTHV		NVTBIT(2, 0)				// Planar V FIFO Threshold 

#define REG_CMPADDR 	(VIDEOIN_BASE + 0x40)		// R/W  : Current Packet System Memory Address Register
#define REG_CURADDRP 	(VIDEOIN_BASE + 0x50)		// R/W  : FIFO threshold Register
#define REG_CURADDRY 	(VIDEOIN_BASE + 0x54)		// R/W  : Current Planar Y System Memory Address Register
#define REG_CURADDRU 	(VIDEOIN_BASE + 0x58)		// R/W  : Current Planar U System Memory Address Register
#define REG_CURADDRV 	(VIDEOIN_BASE + 0x5C)		// R/W  : Current Planar V System Memory Address Register
#define REG_PACBA0 	(VIDEOIN_BASE + 0x60)		// R/W  : System Memory Packet 0 Base Address Register
#define REG_PACBA1 	(VIDEOIN_BASE + 0x64)		// R/W  : System Memory Packet 1 Base Address Register
#define REG_YBA0 		(VIDEOIN_BASE + 0x80)		// R/W  : System Memory Planar Y Base Address Register
#define REG_UBA0 		(VIDEOIN_BASE + 0x84)		// R/W  : System Memory Planar U Base Address Register
#define REG_VBA0 		(VIDEOIN_BASE + 0x88)		// R/W  : System Memory Planar V Base Address Register

#define FSC_BA	W55FA93_VA_FSC	/* Frame Switch  Controller */
/*
	Frame Switch Controller
*/
#define REG_FSC_CTL		(FSC_BA+0x0000)	// R/W	Frame Switch Controller Control Register
			#define FSC_BLANK_SEL		BIT24			// GPU Blank Channel Selection //?? will check with NW and CST
			#define FSC_RIP_SEL			NVTBIT(13, 12)	// FSC Read IP Selection
			#define FSC_WIP_SEL			NVTBIT(9, 8)		// FSC Write IP Selection	
			#define FSC_EN       			BIT0			// Frame Swith Block Enable/Disable

#define REG_FSINT_CTL	(FSC_BA+0x0004)	// R/W	Frame Switch Interrupt Control Register 
			#define FSC1_RE_INT_EN		BIT15			// FSC1 Read Error Interrupt Enable	1'b0: disable1'b1: enable
			#define FSC1_WE_INT_EN		BIT14			// FSC1 Write Error Interrupt Enable	1'b0: disable1'b1: enable
			#define FSC1_RS_INT_EN		BIT13			// FSC1 Read Switch Interrupt Enable	
			#define FSC1_WS_INT_EN		BIT12			// FSC1 Write Switch Interrupt Enable	
			#define FSC0_RE_INT_EN		BIT11			// FSC0 Read Error Interrupt Enable	1'b0: disable1'b1: enable
			#define FSC0_WE_INT_EN		BIT10			// FSC0 Write Error Interrupt Enable	1'b0: disable1'b1: enable
			#define FSC0_RS_INT_EN		BIT9			// FSC0 Read Switch Interrupt Enable 
			#define FSC0_WS_INT_EN		BIT8			// FSC0 Write Switch Interrupt Enable 
			
			#define FSC1_RE_INT			BIT7			// Frame Switch Channel 1 Read Error InterruptIf writing IP reads error happens 
			#define FSC1_WE_INT		BIT6			// Frame Switch Channel 1 Write Error InterruptIf reading IP reads error happnes 
			#define FSC1_RS_INT			BIT5			// Frame Switch Channel 1 Read Switch InterruptTo generate interrupt
														// when writing IP switch the Frame  
			#define FSC1_WS_INT		BIT4			// Frame Switch Channel 1 Write Switch InterruptTo generate interrupt 
														// when reading IP switch the Frame  
			#define FSC0_RE_INT			BIT3			// Frame Switch Channel 0 Read Error InterruptIf writing IP reads error happens 
			#define FSC0_WE_INT		BIT2			// Frame Switch Channel 0 Write Error InterruptIf reading IP reads error happnes 
			#define FSC0_RS_INT			BIT1			// Frame Switch Channel 0 Read Switch InterruptTo generate interrupt 
														// when writing IP switch the Frame  
			#define FSC0_WS_INT		BIT0			// Frame Switch Channel 0 Write Switch InterruptTo generate interrupt 
														// when reading IP switch the Frame  
			
#define REG_FSC0_CTL		(FSC_BA+0x0100)	// R/W	Frame Synchorize Channel 1 Control Register	0x0000_0000
#define REG_FSC1_CTL		(FSC_BA+0x0200)	// R/W	Frame Synchorize Channel 1 Control Register	0x0000_0000
			#define FSC_RST				BIT31			// Frame Synchroize Channel 1 Reset (Low Active)
			#define FSC_WIP_ABANDON	BIT30			// 	
			#define FSC_FR_SRC			BIT24			// Frame Rate Criterion
			#define FSC_RI_BA_SEL		NVTBIT(21, 20)	// FSCX Read IP Frame Base Address Selection in software mode control
			#define FSC_WI_BA_SEL		NVTBIT(17, 16)	// FSCX Write IP Frame Base Address Selection in software mode control
			//#define FSC_RIS			NVTBIT(13, 12)	// FSCX Read IP Selection
			//#define FSC_WIS			NVTBIT(9, 8)		// FSCX Write IP Selection
			#define FSC_FR				NVTBIT(7, 4)		// Frame Rate Synchorize Selection(Read/Write) for Triple Buffer
			#define FSC_FSM				NVTBIT(3, 2)		// Frame Switch Method ( 0=Double, 1=Triple)  
			#define FSC_BN				BIT1			// Frame Switch Channel 1 Buffer Number
			#define FSC_EN				BIT0			// Frame Switch Channel 1 Enable/Disable Selection
			
#define REG_FSC0_BCNT	(FSC_BA+0x0104)	// R/W	Frame Switch Channel BlankCounter	0x0000_0000
#define REG_FSC1_BCNT	(FSC_BA+0x0204)	// R/W	Frame Switch Channel BlankCounter	0x0000_0000
			#define FSCX_BCNT			NVTBIT(15, 0)	// Frame Switch Channel  Blank Counter
			
#define REG_FSC0_WBUF	(FSC_BA+0x0108)	// R	Frame Switch Write Buffer. (GPU)
#define REG_FSC0_RBUF	(FSC_BA+0x010C)	// R	Frame Switch Read Buffer. (VPOST) 
							
#define REG_FSC0_BA0		(FSC_BA+0x0110)	// R/W	Frame Switch Channel Base Address0	0x0000_0000
#define REG_FSC0_BA1		(FSC_BA+0x0114)	// R/W	Frame Switch Channel Base Address1	0x0000_0000
#define REG_FSC0_BA2		(FSC_BA+0x0118)	// R/W	Frame Switch Channel Base Address2	0x0000_0000

#define REG_FSC1_WBUF	(FSC_BA+0x0208)	// R	Frame Switch Write Buffer. (GPU)
#define REG_FSC1_RBUF	(FSC_BA+0x020C)	// R	Frame Switch Read Buffer. (VPOST) 

#define REG_FSC1_BA0		(FSC_BA+0x0210)	// R/W	Frame Switch Channel Base Address0	0x0000_0000
#define REG_FSC1_BA1		(FSC_BA+0x0214)	// R/W	Frame Switch Channel Base Address1	0x0000_0000
#define REG_FSC1_BA2		(FSC_BA+0x0218)	// R/W	Frame Switch Channel Base Address2	0x0000_0000



#define I2C_BASE	W55FA93_VA_I2C	/* I2CH Control */

/* I2CH Registers */

#define REG_I2C_CSR			(I2C_BASE+0x000)			// R/W: Control and Status  Register
        #define I2C_RXACK	BIT11		// Received ACK from Slave
        #define I2C_BUSY	BIT10		// I2C bus busy
        #define I2C_AL		BIT9		// Arbitration lost
        #define I2C_TIP		BIT8		// Transfer in progress
        #define TX_NUM		NVTBIT(5,4)	// Transmit byte count
        #define CSR_IF		BIT2		// Interrupt flag
        #define CSR_IE		BIT1		// Interrupt enable
        #define I2C_EN		BIT0		// I2C core enabl
	
#define REG_I2C_DIVIDER			(I2C_BASE+0x004)		// R/W:Clock Prescale Register
       // #define DIVIDER	NVTBIT(15,0)

#define REG_I2C_CMDR			(I2C_BASE+0x008)			// R/W:Command Register
        #define START		BIT4		// Generate start condition
        #define STOP		BIT3		// Generate stop condition
//        #define READ		BIT2		// Read data from slave
//        #define WRITE		BIT1		// Write data from slave
     	#define ACK			BIT0		// Send ACK to slave

#define REG_I2C_SWR			(I2C_BASE+0x00C)			// R/W:Software Mode Register
		#define SER		BIT5		// Serial interface SDO status
        #define SDR		BIT4		// Serial interface SDA status
        #define SCR		BIT3		// Serial interface SCK status
        #define SEW		BIT2		// Serial interface SDO output control
     	#define SDW		BIT1		// Serial interface SDW status control
     	#define SCW		BIT0		// Serial interface SCW output control
     	
#define REG_I2C_RXR			(I2C_BASE+0x010)			// R:Data Receive Register     	

#define REG_I2C_TXR			(I2C_BASE+0x014)			// R/W:Data Transmit Register

#define KPI_BASE	W55FA93_VA_KPI	/* KPI Control */

#define REG_KPICONF		(KPI_BASE+0x0000)	// R/W	Keypad controller configuration Register
	#define KROW 			NVTBIT(30,28)		// Keypad Matrix ROW number
	#define KCOL 			NVTBIT(26,24)		// Keypad Matrix COL Number
	#define DB_EN			BIT21			// Scan In Signal De-bounce Enable	
	#define DB_CLKSEL 		NVTBIT(19,16)		// Scan In De-bounce sampling cycle selection
	#define PRESCALE		NVTBIT(15,8)		// Row Scan Cycle Pre-scale Value
	#define INPU 			BIT6			// key Scan In Pull-UP Enable Register
	#define WAKEUP			BIT5			// Lower Power Wakeup Enable	
	#define ODEN				BIT4		// Open Drain Enable
	#define INTEN			BIT3		// Key Interrupt Enable Control 
	#define RKINTEN			BIT2			// Release Key Interrupt Enable Cintrol
	#define PKINTEN			BIT1			// Press Key Interrupt Enable Control
	#define ENKP				BIT0			// Keypad Scan Enable
		
#define REG_KPI3KCONF	(KPI_BASE+0x0004)	// R/W	Keypad controller 3-keys configuration register	
	#define EN3KYRST 		BIT24			// Enable Three-key Reset
	#define K32R 			NVTBIT(21,19)		// The #2 Key Row Address
	#define K32C				NVTBIT(18,16)		// The #2 Key Column Address
	#define K31R				NVTBIT(13,11)		// The #1 Key Row Address
	#define K31C 			NVTBIT(10,8)		// The #1 Key Column Address
	#define K30R				NVTBIT(5,3)		// The #0 Key Row Address
	#define K30C				NVTBIT(2,0)		//	The #0 Key Column Address

#define REG_KPISTATUS	(KPI_BASE+0x0008)	// R/O	Key Pad Interface Status Register
	#define RROW7			BIT23		// Release key row coordinate
	#define RROW6			BIT22
	#define RROW5			BIT21
	#define RROW4			BIT20
	#define RROW3			BIT19
	#define RROW2			BIT18
	#define RROW1			BIT17
	#define RROW0			BIT16
	#define PROW7			BIT15			// Press key row coordinate
	#define PROW6			BIT14
	#define PROW5			BIT13
	#define PROW4			BIT12
	#define PROW3			BIT11
	#define PROW2			BIT10
	#define PROW1			BIT9
	#define PROW0			BIT8
	#define PKEY_INT			BIT4			// Press key interrupt
	#define RKEY_INT			BIT3			// Release key interrupt
	#define KEY_INT			BIT2			// Key Interrupt
	#define RST_3KEY		BIT1			// 3-Keys Reset Flag 
	#define PDWAKE			BIT0			// Power Down Wakeup Flag	

#define REG_KPIRSTC		(KPI_BASE+0x000c)	// R/O	Keypad reset period controller register
	#define RSTC				NVTBIT(7,0)			// 3-key Reset Period Count

#define REG_KPIKEST0	(KPI_BASE+0x0010)	// R/O	Keypad state register 0	
#define REG_KPIKEST1	(KPI_BASE+0x0014)	// R/O	Keypad state register 1	
#define REG_KPIKPE0		(KPI_BASE+0x0018)	// R/O	Lower 32 Press Key event indicator
#define REG_KPIKPE1		(KPI_BASE+0x001C)	// R/O	Higher 32 Press Key event indicator
#define REG_KPIKRE0		(KPI_BASE+0x0020)	// R/O	Lower 32 Realease Key event indicator	
#define REG_KPIKRE1		(KPI_BASE+0x0024)	// R/O	Higher 32 Realease Key event indicator
#define REG_KPIPRESCALDIV	(KPI_BASE+0x0028)	// Prescale divider
	#define PRESCALDIV 		NVTBIT(7,0)

#define SPI_BASE_0	(W55FA93_VA_SPI0)

#define REG_SPI0_CNTRL		(SPI_BASE_0 + 0x00)
#define SPI0_CNTRL_GO_BUSY      BIT0
#define REG_SPI0_DIVIDER	(SPI_BASE_0 + 0x04)
#define REG_SPI0_SSR		(SPI_BASE_0 + 0x08)

#define REG_SPI0_RX0		(SPI_BASE_0 + 0x10)
#define REG_SPI0_RX1		(SPI_BASE_0 + 0x14)
#define REG_SPI0_RX2		(SPI_BASE_0 + 0x18)
#define REG_SPI0_RX3		(SPI_BASE_0 + 0x1c)

#define REG_SPI0_TX0		(SPI_BASE_0 + 0x10)
#define REG_SPI0_TX1		(SPI_BASE_0 + 0x14)
#define REG_SPI0_TX2		(SPI_BASE_0 + 0x18)
#define REG_SPI0_TX3		(SPI_BASE_0 + 0x1c)

#define SPI_BASE_1	(W55FA93_VA_SPI0 + 0x400)

#define REG_SPI1_CNTRL		(SPI_BASE_1 + 0x00)
#define REG_SPI1_DIVIDER	(SPI_BASE_1 + 0x04)
#define REG_SPI1_SSR		(SPI_BASE_1 + 0x08)

#define REG_SPI1_RX0		(SPI_BASE_1 + 0x10)
#define REG_SPI1_RX1		(SPI_BASE_1 + 0x14)
#define REG_SPI1_RX2		(SPI_BASE_1 + 0x18)
#define REG_SPI1_RX3		(SPI_BASE_1 + 0x1c)

#define REG_SPI1_TX0		(SPI_BASE_1 + 0x10)
#define REG_SPI1_TX1		(SPI_BASE_1 + 0x14)
#define REG_SPI1_TX2		(SPI_BASE_1 + 0x18)
#define REG_SPI1_TX3		(SPI_BASE_1 + 0x1c)

#define PWM_BASE	(W55FA93_VA_PWM)

#define REG_PPR         (PWM_BASE+0x000)			// R/W: PWM Prescaler Register
	#define DZI1            NVTBIT(31,24)	 	// Dead zone interval register 1
	#define DZI0            NVTBIT(23,16)			// Dead zone interval register 0
	#define CP1             NVTBIT(15,8)			// Clock prescaler 1 for PWM Timer channel 2 & 3
	#define CP0             NVTBIT(7,0)			// Clock prescaler 0 for PWM Timer channel 0 & 1
		
#define REG_PWM_CSR     (PWM_BASE+0x004)		// R/W: PWM Clock Select Register 
	#define CSR3            NVTBIT(14,12)			// Select clock input for channel 3
	#define CSR2            NVTBIT(10,8)			// Select clock input for channel 2
	#define CSR1            NVTBIT(6,4)			// Select clock input for channel 1
	#define CSR0            NVTBIT(2,0)			// Select clock input for channel 0

#define REG_PCR         (PWM_BASE+0x008)			// R/W: PWM Control Register
	#define CH3MOD          BIT27				// Channel 3 toggle/one shot mode
	#define CH3INV          BIT26				// Channel 3 inverter on/off
	#define CH3EN           BIT24				// Channel 3 enable/disable
	#define CH2MOD          BIT19				// Channel 2 toggle/one shot mode
	#define CH2INV          BIT18				// Channel 2 inverter on/off
	#define CH2EN           BIT16				// Channel 2 enable/disable		
	#define CH1MOD          BIT11				// Channel 1 toggle/one shot mode
	#define CH1INV          BIT10				// Channel 1 inverter on/off
	#define CH1EN           BIT8				// Channel 1 enable/disable
	#define DZEN1           BIT5				// Dead-Zone generator 1 enable/disable	
	#define DZEN0           BIT4				// Dead-Zone generator 0 enable/disable
	#define CH0MOD          BIT3				// Channel 0 toggle/one shot mode
	#define CH0INV          BIT2				// Channel 0 inverter on/off
	#define CH0EN           BIT0				// Channel 0 enable/disable

#define REG_CNR0        (PWM_BASE+0x00C)			// R/W: PWM Counter Register 0
#define REG_CNR1        (PWM_BASE+0x018)			// R/W: PWM Counter Register 1
#define REG_CNR2        (PWM_BASE+0x024)			// R/W: PWM Counter Register 2
#define REG_CNR3        (PWM_BASE+0x030)			// R/W: PWM Counter Register 3
	#define CNR             NVTBIT(15,0)			    // PWM counter/timer buff

#define REG_CMR0        (PWM_BASE+0x010)			// R/W: PWM Comparator Register 0
#define REG_CMR1        (PWM_BASE+0x01C)			// R/W: PWM Comparator Register 1
#define REG_CMR2        (PWM_BASE+0x028)			// R/W: PWM Comparator Register 2
#define REG_CMR3        (PWM_BASE+0x034)			// R/W: PWM Comparator Register 3
	#define CMR             NVTBIT(15,0)			    // PWM comparator register

#define REG_PDR0        (PWM_BASE+0x014)			// R  : PWM Data Register 0
#define REG_PDR1        (PWM_BASE+0x020)			// R  : PWM Data Register 1
#define REG_PDR2        (PWM_BASE+0x02C)			// R  : PWM Data Register 2
#define REG_PDR3        (PWM_BASE+0x038)			// R  : PWM Data Register 3
	#define PDR             NVTBIT(15,0)			// PWM data register

#define REG_PIER        (PWM_BASE+0x040)			// R/W: PWM Interrupt Enable Register
	#define PIER3           BIT3				// PWM timer channel 3 interrupt enable/disable
	#define PIER2           BIT2				// PWM timer channel 2 interrupt enable/disable
	#define PIER1           BIT1				// PWM timer channel 1 interrupt enable/disable
	#define PIER0           BIT0				// PWM timer channel 0 interrupt enable/disable

#define REG_PIIR        (PWM_BASE+0x044)			// R/C: PWM Interrupt Identification Register
	#define PIIR3           BIT3				// PWM timer channel 3 interrupt flag
	#define PIIR2           BIT2				// PWM timer channel 2 interrupt flag
	#define PIIR1           BIT1				// PWM timer channel 1 interrupt flag
	#define PIIR0           BIT0				// PWM timer channel 0 interrupt flag

#define REG_CCR0        (PWM_BASE+0x050)			//R/W: Capture Control Register
	#define CFLRD1          BIT23			//CFLR1 dirty bit
	#define CRLRD1          BIT22			//CRLR1 dirty bit
	#define CIIR1           BIT20			//Capture Interrupt Indication 1 Enable/Disable
	#define CAPCH1EN        BIT19			//Capture Channel 1 transition Enable/Disable
	#define FL_IE1          BIT18			//Channel1 Falling Interrupt Enable ON/OFF
	#define RL_IE1          BIT17			//Channel1 Rising Interrupt Enable ON/OFF
	#define INV1            BIT16			//Channel 1 Inverter ON/OFF
	#define CFLRD0          BIT7			//CFLR0 dirty bit
	#define CRLRD0          BIT6			//CRLR0 dirty bit
	#define CIIR0           BIT4			//Capture Interrupt Indication 0 Enable/Disable
	#define CAPCH0EN        BIT3			//Capture Channel 0 transition Enable/Disable
	#define FL_IE0          BIT2			//Channel0 Falling Interrupt Enable ON/OFF
	#define RL_IE0          BIT1			//Channel0 Rising Interrupt Enable ON/OFF
	#define INV0            BIT0			//Channel 0 Inverter ON/OFF
		
	
#define REG_CCR1        (PWM_BASE+0x054)			//R/W: Capture Control Register
	#define CFLRD3          BIT23			//CFLR3 dirty bit
	#define CRLRD3          BIT22			//CRLR3 dirty bit
	#define CIIR3           BIT20			//Capture Interrupt Indication 3 Enable/Disable
	#define CAPCH3EN        BIT19			//Capture Channel 3 transition Enable/Disable
	#define FL_IE3          BIT18			//Channel3 Falling Interrupt Enable ON/OFF
	#define RL_IE3          BIT17			//Channel3 Rising Interrupt Enable ON/OFF
	#define INV3            BIT16			//Channel 3 Inverter ON/OFF
	#define CFLRD2          BIT7			//CFLR2 dirty bit
	#define CRLRD2          BIT6			//CRLR2 dirty bit
	#define CIIR2           BIT4			//Capture Interrupt Indication 2 Enable/Disable
	#define CAPCH2EN        BIT3			//Capture Channel 2 transition Enable/Disable
	#define FL_IE2          BIT2			//Channel2 Falling Interrupt Enable ON/OFF
	#define RL_IE2          BIT1			//Channel2 Rising Interrupt Enable ON/OFF
	#define INV2            BIT0			//Channel 2 Inverter ON/OFF

#define REG_CRLR0       (PWM_BASE+0x058)		//R/W: Capture Rising Latch Register (channel 0)
#define REG_CRLR1       (PWM_BASE+0x060)		//R/W: Capture Rising Latch Register (channel 1)
#define REG_CRLR2       (PWM_BASE+0x068)		//R/W: Capture Rising Latch Register (channel 2)
#define REG_CRLR3       (PWM_BASE+0x070)		//R/W: Capture Rising Latch Register (channel 3)
	#define PWM_CRLR0       NVTBIT(15,0)		//Capture Rising Latch Register	

#define REG_CFLR0       (PWM_BASE+0x05C)		//R/W: Capture Falling Latch Register (channel 0)
#define REG_CFLR1       (PWM_BASE+0x064)		//R/W: Capture Falling Latch Register (channel 1)
#define REG_CFLR2       (PWM_BASE+0x06C)		//R/W: Capture Falling Latch Register (channel 2)
#define REG_CFLR3       (PWM_BASE+0x074)		//R/W: Capture Falling Latch Register (channel 3)
	#define PWM_CFLR0       NVTBIT(15,0)		//Capture Falling Latch Register

#define REG_CAPENR      (PWM_BASE+0x078)		//R/W: Capture Input Enable Register
	#define PWM_CAPENR      NVTBIT(3,0)		//Capture Input Enable
	
#define REG_POE         (PWM_BASE+0x07C)			//R/W: PWM Output Enable Register
	#define PWM3            BIT3			//PWM 3 Output Enable
	#define PWM2            BIT2			//PWM 2 Output Enable
	#define PWM1            BIT1			//PWM 1 Output Enable
	#define PWM0            BIT0			//PWM 0 Output Enable

#define EDMA_BASE	(W55FA93_VA_EDMA)

#define REG_VDMA_CSR  (EDMA_BASE + 0x0000)		// VDMA Control and Status Register CH0
#define REG_PDMA_CSR1  (EDMA_BASE +, 0x0100)		// PDMA Control and Status Register CH1
#define REG_PDMA_CSR2  (EDMA_BASE + 0x0200)		// PDMA Control and Status Register CH2
#define REG_PDMA_CSR3  (EDMA_BASE + 0x0300)		// PDMA Control and Status Register CH3
#define REG_PDMA_CSR4  (EDMA_BASE + 0x0400)		// PDMA Control and Status Register CH4
	#define TRIG_EN	    BIT23		// Enalbe EDMA Data Read or Write Transfer
	#define APB_TWS       NVTBIT(20, 19)// Peripheral Transfer Width Select
	#define WAR_BCR_SEL   NVTBIT(15, 12)// Wrap Around Transfer Byte Count Interrupt Select
	#define EDMASG_EN         BIT9		// EDMA Scatter-Gather Function Enable
	#define EDMA_RST      BIT8		// EDMA Rest
	#define DAD_SEL       NVTBIT(7, 6)	// Transfer Destination Address Direction Select
	#define SAD_SEL       NVTBIT(5, 4)	// Transfer Source Address Direction Select
	#define MODE_SEL      NVTBIT(3, 2)	// EDMA Mode Select
	#define SW_RST        BIT1		// Software Engine Reset	
	#define EDMACEN       BIT0		// EDMA Channel Enable

#define REG_VDMA_SAR  (EDMA_BASE + 0x0004)		// VDMA Transfer Source Address Register CH0
#define REG_PDMA_SAR1  (EDMA_BASE + 0x0104)		// PDMA Transfer Source Address Register CH1
#define REG_PDMA_SAR2  (EDMA_BASE + 0x0204)		// PDMA Transfer Source Address Register CH2
#define REG_PDMA_SAR3  (EDMA_BASE + 0x0304)		// PDMA Transfer Source Address Register CH3
#define REG_PDMA_SAR4  (EDMA_BASE + 0x0404)		// PDMA Transfer Source Address Register CH4


#define REG_VDMA_DAR  (EDMA_BASE + 0x0008)		// VDMA Transfer Destination Address Register CH0
#define REG_PDMA_DAR1  (EDMA_BASE + 0x0108)		// PDMA Transfer Destination Address Register CH1
#define REG_PDMA_DAR2  (EDMA_BASE + 0x0208)		// PDMA Transfer Destination Address Register CH2
#define REG_PDMA_DAR3  (EDMA_BASE + 0x0308)		// PDMA Transfer Destination Address Register CH3
#define REG_PDMA_DAR4  (EDMA_BASE + 0x0408)		// PDMA Transfer Destination Address Register CH4


#define REG_VDMA_BCR  (EDMA_BASE + 0x000C)		// VDMA Transfer Byte Count Register CH0
#define REG_PDMA_BCR1  (EDMA_BASE + 0x010C)		// PDMA Transfer Byte Count Register CH1
#define REG_PDMA_BCR2  (EDMA_BASE + 0x020C)		// PDMA Transfer Byte Count Register CH2
#define REG_PDMA_BCR3  (EDMA_BASE + 0x030C)		// PDMA Transfer Byte Count Register CH3
#define REG_PDMA_BCR4  (EDMA_BASE + 0x040C)		// PDMA Transfer Byte Count Register CH4
	#define WAR_BCR_IF    NVTBIT(23,  0)// PDMA Transfer Byte Count Reigster
	
#define REG_VDMA_SGAR  (EDMA_BASE + 0x0010)		// VDMA Scatter-Gather Table Start Address Register	
#define REG_PDMA_SGAR1  (EDMA_BASE + 0x0110)		// PDMA Scatter-Gather Table Start Address Register	CH1
#define REG_PDMA_SGAR2  (EDMA_BASE + 0x0210)		// PDMA Scatter-Gather Table Start Address Register	CH2
#define REG_PDMA_SGAR3  (EDMA_BASE + 0x0310)		// PDMA Scatter-Gather Table Start Address Register	CH3
#define REG_PDMA_SGAR4  (EDMA_BASE + 0x0410)		// PDMA Scatter-Gather Table Start Address Register	CH4

	
#define REG_VDMA_CSAR  (EDMA_BASE + 0x0014)		// VDMA Current Source Address Register CH0
#define REG_PDMA_CSAR1  (EDMA_BASE + 0x0114)		// PDMA Current Source Address Register CH1
#define REG_PDMA_CSAR2  (EDMA_BASE + 0x0214)		// PDMA Current Source Address Register CH2
#define REG_PDMA_CSAR3  (EDMA_BASE + 0x0314)		// PDMA Current Source Address Register CH3
#define REG_PDMA_CSAR4  (EDMA_BASE + 0x0414)		// PDMA Current Source Address Register CH4


#define REG_VDMA_CDAR  (EDMA_BASE + 0x0018)		// VDMA Current Destination Address Register CH0
#define REG_PDMA_CDAR1  (EDMA_BASE + 0x0118)		// PDMA Current Destination Address Register CH1
#define REG_PDMA_CDAR2  (EDMA_BASE + 0x0218)		// PDMA Current Destination Address Register CH2
#define REG_PDMA_CDAR3  (EDMA_BASE + 0x0318)		// PDMA Current Destination Address Register CH3
#define REG_PDMA_CDAR4  (EDMA_BASE + 0x0418)		// PDMA Current Destination Address Register CH4


#define REG_VDMA_CBCR  (EDMA_BASE + 0x001C)		// VDMA Current Byte Counte Register CH0
#define REG_PDMA_CBCR1  (EDMA_BASE + 0x011C)		// PDMA Current Byte Counte Register CH1
#define REG_PDMA_CBCR2  (EDMA_BASE + 0x021C)		// PDMA Current Byte Counte Register CH2
#define REG_PDMA_CBCR3  (EDMA_BASE + 0x031C)		// PDMA Current Byte Counte Register CH3
#define REG_PDMA_CBCR4  (EDMA_BASE + 0x041C)		// PDMA Current Byte Counte Register CH4


#define REG_VDMA_IER  (EDMA_BASE + 0x0020)		// VDMA Interrupt Enable Control Register CH0
#define REG_PDMA_IER1  (EDMA_BASE + 0x0120)		// PDMA Interrupt Enable Control Register CH1
#define REG_PDMA_IER2  (EDMA_BASE + 0x0220)		// PDMA Interrupt Enable Control Register CH2
#define REG_PDMA_IER3  (EDMA_BASE + 0x0320)		// PDMA Interrupt Enable Control Register CH3
#define REG_PDMA_IER4  (EDMA_BASE + 0x0420)		// PDMA Interrupt Enable Control Register CH4
	#define SG_IEN	    BIT3		// PDMA Scatter-Gather Interrupt Enable
	#define WAR_IE	    BIT2		// PDMA Wrap Around Interrupt Enable
	#define BLKD_IE	    BIT1		// PDMA Block Transfer Done Interrupt Enable
	#define EDMATABORT_IE	   BIT0	// PDMA Read/Write Target Abort Interrupt Enable

#define REG_VDMA_ISR  (EDMA_BASE + 0x0024)		// VDMA Interrupt Status Register CH0
#define REG_PDMA_ISR1  (EDMA_BASE + 0x0124)		// PDMA Interrupt Status Register CH1
#define REG_PDMA_ISR2  (EDMA_BASE + 0x0224)		// PDMA Interrupt Status Register CH2
#define REG_PDMA_ISR3  (EDMA_BASE + 0x0324)		// PDMA Interrupt Status Register CH3
#define REG_PDMA_ISR4  (EDMA_BASE + 0x0424)		// PDMA Interrupt Status Register CH4
	#define INTR  	    BIT31		// Interrupt Pin Status
	#define INTR4 	    BIT28		// Interrupt Pin Status of Channel 4
	#define INTR3 	    BIT27		// Interrupt Pin Status of Channel 3
	#define INTR2 	    BIT26		// Interrupt Pin Status of Channel 2
	#define INTR1 	    BIT25		// Interrupt Pin Status of Channel 1
	#define INTR0 	    BIT24		// Interrupt Pin Status of Channel 0				
	#define EDMABUSY	    BIT15		// EDMA Transfer is in Progress
	#define EDMAWAR_BCR_IF NVTBIT(11,  8)// Wrap Around Transfer Byte Count Interrupt Flag
	#define EDMASG_IF	      BIT3		// Scatter-Gather Interrupt Flag	
	#define EDMABLKD_IF	  BIT1		// Block Transfer Done Interrupt Flag
	#define EDMATABORT_IF	  BIT0		// PDMA Read/Write Target Abort Interrupt Flag
		

#define REG_VDMA_CTCSR  (EDMA_BASE + 0x0028)		// VDMA Color Transfer Control Register CH0
	#define SOUR_FORMAT NVTBIT(27, 24)   // Source Address Color Format
	#define DEST_FORMAT NVTBIT(19, 16)   // Destination Address Color Format
	#define CLAMPING_EN	  BIT7		// Clamping Enable
	#define COL_TRA_EN	  BIT1		// Color Transfer Enable
	#define STRIDE_EN 	  BIT0		// Stride Mode Enable

#define REG_VDMA_SASOCR  (EDMA_BASE + 0x002C)	// VDMA Source Address Stride Offset Control Register
	#define STBC          NVTBIT(31, 16)// PDMA Stride Transfer Byte Count
	#define SASTOBL       NVTBIT(15,  0)// PDMA Source Address Stride Offset Byte Length

#define REG_VDMA_DASOCR  (EDMA_BASE + 0x0030)	// PDMA Destination Address Stride Offset Control Register
	#define DASTOBL       NVTBIT(15,  0)// PDMA Destination Address Stride Offset Byte Length
	
#define REG_PDMA_POINT1  (EDMA_BASE + 0x013C)	// PDMA Internal Buffer Pointer Register CH1
#define REG_PDMA_POINT2  (EDMA_BASE + 0x023C)	// PDMA Internal Buffer Pointer Register CH2
#define REG_PDMA_POINT3  (EDMA_BASE + 0x033C)	// PDMA Internal Buffer Pointer Register CH3
#define REG_PDMA_POINT4  (EDMA_BASE + 0x043C)	// PDMA Internal Buffer Pointer Register CH4
	#define PDMA_POINT    NVTBIT(4,  0) // PDMA Internal Buffer Pointer Reigster	
	
#define REG_EDMA_SBUF0_C0  (EDMA_BASE + 0x0080)	// VDMA Shared Buffer FIFO 0 Reigster CH0
#define REG_EDMA_SBUF0_C1  (EDMA_BASE + 0x0180)	// PDMA Shared Buffer FIFO 0 Reigster CH1
#define REG_EDMA_SBUF0_C2  (EDMA_BASE + 0x0280)	// PDMA Shared Buffer FIFO 0 Reigster CH2
#define REG_EDMA_SBUF0_C3  (EDMA_BASE + 0x0380)	// PDMA Shared Buffer FIFO 0 Reigster CH3
#define REG_EDMA_SBUF0_C4  (EDMA_BASE + 0x0480)	// PDMA Shared Buffer FIFO 0 Reigster CH4

#define REG_EDMA_SBUF1_C0  (EDMA_BASE + 0x0084)	// VDMA Shared Buffer FIFO 1 Reigster CH0
#define REG_EDMA_SBUF1_C1  (EDMA_BASE + 0x0184)	// PDMA Shared Buffer FIFO 1 Reigster CH1
#define REG_EDMA_SBUF1_C2  (EDMA_BASE + 0x0284)	// PDMA Shared Buffer FIFO 1 Reigster CH2
#define REG_EDMA_SBUF1_C3  (EDMA_BASE + 0x0384)	// PDMA Shared Buffer FIFO 1 Reigster CH3
#define REG_EDMA_SBUF1_C4  (EDMA_BASE + 0x0484)	// PDMA Shared Buffer FIFO 1 Reigster CH4

#define REG_EDMA_SBUF2_C0  (EDMA_BASE + 0x0088)	// VDMA Shared Buffer FIFO 2 Reigster CH0
#define REG_EDMA_SBUF2_C1  (EDMA_BASE + 0x0188)	// PDMA Shared Buffer FIFO 2 Reigster CH1
#define REG_EDMA_SBUF2_C2  (EDMA_BASE + 0x0288)	// PDMA Shared Buffer FIFO 2 Reigster CH2
#define REG_EDMA_SBUF2_C3  (EDMA_BASE + 0x0388)	// PDMA Shared Buffer FIFO 2 Reigster CH3
#define REG_EDMA_SBUF2_C4  (EDMA_BASE + 0x0488)	// PDMA Shared Buffer FIFO 2 Reigster CH4

#define REG_EDMA_SBUF3_C0  (EDMA_BASE + 0x008C)	// VDMA Shared Buffer FIFO 3 Reigster CH0
#define REG_EDMA_SBUF3_C1  (EDMA_BASE + 0x018C)	// PDMA Shared Buffer FIFO 3 Reigster CH1
#define REG_EDMA_SBUF3_C2  (EDMA_BASE + 0x028C)	// PDMA Shared Buffer FIFO 3 Reigster CH2
#define REG_EDMA_SBUF3_C3  (EDMA_BASE + 0x038C)	// PDMA Shared Buffer FIFO 3 Reigster CH3
#define REG_EDMA_SBUF3_C4  (EDMA_BASE + 0x048C)	// PDMA Shared Buffer FIFO 3 Reigster CH4

#define BLT_BASE	(W55FA93_VA_BLT)
#define REG_SET2DA	(BLT_BASE+0x0000)	// BitBlitting Accelerator Enable Set Up Register
	#define FILL_OP		BIT11		// Fill operation to rectangle	
	#define FILL_STYLE	NVTBIT(10, 8)	// Bitmap Fill Style
	#define TRCOLOR_E	BIT7		// RGB565 Transparent Color Enable/Disable
	#define TRANS_FLAG	NVTBIT( 6, 4)	// Transform Flag
	#define S_ALPHA		BIT3		// Reveal Source Image Alpha during Transparency
	#define FILL_BLEND	BIT2		// Alpha blending for Fill operation	
	#define L_ENDIAN	BIT1		// Palette Format Data Order
	#define BLIT_EN		BIT0		// Blit a bitmap to the frame buffer using the hardware accelerator
	
#define REG_SFMT	(BLT_BASE+0x0004)	// Pixel Format of Source Bitmap Register
	#define S_RGB_bpp8	BIT5		// Bitmap Pixel Format bpp8_clutRGB
	#define S_RGB_bpp4	BIT4		// Bitmap Pixel Format bpp4_clutRGB
	#define S_RGB_bpp2	BIT3		// Bitmap Pixel Format bpp2_clutRGB
	#define S_RGB_bpp1	BIT2		// Bitmap Pixel Format bpp1_clutRGB
	#define S_RGB565	BIT1		// Bitmap Pixel Format bpp16_RGB565
	#define S_ARGB8888	BIT0		// Bitmap Pixel Format bpp32_ARGB8888			
	
#define REG_DFMT	(BLT_BASE+0x0008)	// Pixel Format of Destination Bitmap Register
	#define D_RGB555	BIT2		// Display format RGB555 16 bit format
	#define D_RGB565	BIT1		// Display format RGB565 16 bit format
	#define D_ARGB8888	BIT0		// Display format ARGB8888 32 bit format		
	
#define REG_BLTINTCR	(BLT_BASE+0x000C)	// BLT Interrupt Control and Status Register
	#define BLT_INTE	BIT1		// Blitting Complete Interrupt Enable
	#define BLT_INTS	BIT0		// Blitting Complete Interrupt Status

	
#define REG_MLTA	(BLT_BASE+0x0010)	// Alpha Multiplier Register
	#define OFFSET_A	NVTBIT(31,16)	// Fixed point 8.8 A Color Offset Value
	#define MULTIPLIER_A	NVTBIT(15, 0)	// Fixed point 8.8 A Color Multiplier Value
	
#define REG_MLTR	(BLT_BASE+0x0014)	// Red Multiplier Register
	#define OFFSET_R	NVTBIT(31,16)	// Fixed point 8.8 R Color Offset Value
	#define MULTIPLIER_R	NVTBIT(15, 0)	// Fixed point 8.8 R Color Multiplier Value
	
#define REG_MLTG	(BLT_BASE+0x0018)	// Green Multiplier Register
	#define OFFSET_G	NVTBIT(31,16)	// Fixed point 8.8 G Color Offset Value
	#define MULTIPLIER_G	NVTBIT(15, 0)	// Fixed point 8.8 G Color Multiplier Value
	
#define REG_MLTB	(BLT_BASE+0x001C)	// Blue Multiplier Register
	#define OFFSET_B	NVTBIT(31,16)	// Fixed point 8.8 B Color Offset Value
	#define MULTIPLIER_B	NVTBIT(15, 0)	// Fixed point 8.8 B Color Multiplier Value
	
#define REG_SWIDTH	(BLT_BASE+0x0020)	// Width of Source Register
	#define WIDTH_S	NVTBIT(15, 0)		// The width of source Register
	
#define REG_SHEIGHT	(BLT_BASE+0x0024)	// Height of Source Register
	#define HEIGHT_S	NVTBIT(15, 0)	// The Height of source Register
	
#define REG_DWIDTH	(BLT_BASE+0x0028)	// Width of Destination Register
	#define WIDTH_D		NVTBIT(15, 0)	// The width of Destination Register
	
#define REG_DHEIGHT	(BLT_BASE+0x002C)	// Height of Destination Register
	#define HEIGHT_D	NVTBIT(15, 0)	// The Height of Destination Register
	
#define REG_ELEMENTA	(BLT_BASE+0x0030)	// Transform Element A Register
#define REG_ELEMENTB	(BLT_BASE+0x0034)	// Transform Element B Register
#define REG_ELEMENTC	(BLT_BASE+0x0038)	// Transform Element C Register
#define REG_ELEMENTD	(BLT_BASE+0x003C)	// Transform Element D Register
#define REG_SADDR	(BLT_BASE+0x0040)	// Source Remap Start Address Register
#define REG_DADDR	(BLT_BASE+0x0044)	// Frame Buffer Address Register
#define REG_SSTRIDE	(BLT_BASE+0x0048)	// Source Stride Register
	#define STRIDE_S	NVTBIT(15, 0)	// The Source Stride
	
#define REG_DSTRIDE	(BLT_BASE+0x004C)	// Destination Stride Register
	#define STRIDE_D	NVTBIT(15, 0)	// The Destination Stride
#define REG_OFFSETSX	(BLT_BASE+0x0050)	// Offset of Source X Register
#define REG_OFFSETSY	(BLT_BASE+0x0054)	// Offset of Source Y Register
#define REG_TRCOLOR	(BLT_BASE+0x0058)	// RGB565 Trasnparent Color REgister
	#define TR_RGB565	NVTBIT(15, 0)	// RGB565 Transparent Color
#define REG_OFFSETDY	(BLT_BASE+0x005C)	// Offset of Destination Y Register
	#define OFFSETY_D	NVTBIT(15, 0)	// The Y offset into the Destination Frame Buffer
	
#define REG_FILLARGB	(BLT_BASE+0x0060)	// ARGB Color Values for Fill Operation Register
#define REG_PALETTE	(BLT_BASE+0x0400)	// Color Palette Register

#define RTC_BASE	W55FA93_VA_RTC
/* Real Time Clock Control Registers */
#define REG_RTC_INIR	(RTC_BASE+0x00)   /* RTC Initiation Register */
#define REG_RTC_AER		(RTC_BASE+0x04)   /* RTC Access Enable Register */
#define REG_RTC_FCR		(RTC_BASE+0x08)   /* RTC Frequency Compensation Register */
#define REG_RTC_TLR		(RTC_BASE+0x0C)   /* Time Loading Register */
#define REG_RTC_CLR		(RTC_BASE+0x10)   /* Calendar Loading Register */
#define REG_RTC_TSSR	(RTC_BASE+0x14)   /* Time Scale Selection Register */
#define REG_RTC_DWR		(RTC_BASE+0x18)   /* Day of the Week Register */
#define REG_RTC_TAR		(RTC_BASE+0x1C)   /* Time Alarm Register */
#define REG_RTC_CAR		(RTC_BASE+0x20)   /* Calendar Alarm Register */
#define REG_RTC_LIR		(RTC_BASE+0x24)   /* Leap year Indicator Register */
#define REG_RTC_RIER	(RTC_BASE+0x28)   /* RTC Interrupt Enable Register */
#define REG_RTC_RIIR	(RTC_BASE+0x2C)   /* RTC Interrupt Indicator Register */
#define REG_RTC_TTR		(RTC_BASE+0x30)   /* RTC Time Tick Register */
#define REG_RTC_PWRON	(RTC_BASE+0x34)   /* RTC Power Time Out Register */

#endif
