/* 
 * w55fa93adc.c   w55fa93adc Driver based on wm9878.c
 * Copyright (c) 2010 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Changelog:
 */
/*
 * wm8978.c  --  WM8978 ALSA SoC Audio Codec driver
 *
 * Copyright (C) 2009-2010 Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 * Copyright (C) 2007 Carlos Munoz <carlos@kenati.com>
 * Copyright 2006-2009 Wolfson Microelectronics PLC.
 * Based on wm8974 and wm8990 by Liam Girdwood <lrg@slimlogic.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h> 
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <asm/div64.h>
#include <asm/io.h>
#include <mach/w55fa93_reg.h>
#include "w55fa93adc.h"

#define __TEST__
//#define ERR1

//#define DBG
#ifdef DBG
#define ENTER()		printk("ENTER : %s  : %s\n",__FILE__, __FUNCTION__)
#define LEAVE()		printk("LEAVE : %s  : %s\n",__FILE__, __FUNCTION__)
#define ERRLEAVE()	printk("ERRLEAVE : %s  : %s\n",__FILE__, __FUNCTION__)
#define PUSH()		printk("                                                                                        \n")	

#define SDBG		printk
#else
#define ENTER(...)
#define LEAVE(...)	
#define ERRLEAVE(...)	
#define SDBG(...)
#endif

static struct snd_soc_codec *w55fa93adc_codec;


/* w55fa93adc register cache. Note that register 0 is not included in the cache. */
#if 0
static const u16 w55fa93adc_reg[w55fa93adc_CACHEREGNUM] = {
        0x0000, 0x0000, 0x0000, 0x0000,	/* 0x00...0x03 */
        0x0050, 0x0000, 0x0140, 0x0000,	/* 0x04...0x07 */
        0x0000, 0x0000, 0x0000, 0x00ff,	/* 0x08...0x0b */
        0x00ff, 0x0000, 0x0100, 0x00ff,	/* 0x0c...0x0f */
        0x00ff, 0x0000, 0x012c, 0x002c,	/* 0x10...0x13 */
        0x002c, 0x002c, 0x002c, 0x0000,	/* 0x14...0x17 */
        0x0032, 0x0000, 0x0000, 0x0000,	/* 0x18...0x1b */
        0x0000, 0x0000, 0x0000, 0x0000,	/* 0x1c...0x1f */
        0x0038, 0x000b, 0x0032, 0x0000,	/* 0x20...0x23 */
        0x0008, 0x000c, 0x0093, 0x00e9,	/* 0x24...0x27 */
        0x0000, 0x0000, 0x0000, 0x0000,	/* 0x28...0x2b */
        0x0033, 0x0010, 0x0010, 0x0100,	/* 0x2c...0x2f */
        0x0100, 0x0002, 0x0001, 0x0001,	/* 0x30...0x33 */
        0x0039, 0x0039, 0x0039, 0x0039,	/* 0x34...0x37 */
        0x0001,	0x0001,			/* 0x38...0x3b */
};
#endif 
/* codec private data */
#define W55FA93_ADC_CACHEREGNUM	2
#define W55FA93_ADC_FIRSTREG	0x01
#define W55FA93_ADC_LASTREG		0x02
struct w55fa93adc_priv {
        struct snd_soc_codec codec; 
		 u16 reg_cache[W55FA93_ADC_CACHEREGNUM];
};

struct w55fa93adc_priv *w55fa93adc;

static const u16 w55fa93_adc_reg[W55FA93_ADC_CACHEREGNUM] = {
        0x1E, 0x0e	/* 0x00...0x01 */		// default value of pseudo registers for w55fa92 adc
};
#if 1

/****************************************************************************************************
 * w55fa93 driver layer
 ****************************************************************************************************/

#define ERRCODE		unsigned int
#define UINT8		unsigned char
#define UINT16		unsigned short int
#define UINT32		unsigned int
#define INT8		char
#define INT16		short int
#define INT32		int	

#define BOOL	 	unsigned int
#define FALSE		0
#define TRUE		1
#define PBOOL		BOOL*
#define PUINT32		UINT32*
#define PUINT16		UINT16*
#define PUINT8		UINT8*
#define PINT32		INT32*
#define PINT16		INT16*
#define PINT8		INT8*
#define Successful	0	


/* error code */
#define E_DRVADC_ARGUMENT       	0xB800E001//(ADC_BA + 0x01)
#define E_DRVADC_CLOCK              0xB800E002//(ADC_BA + 0x02)
#define E_DRVADC_INVALID_INT		0xB800E003//(ADC_BA + 0x01)

typedef enum{
	eDRVADC_NORMAL = 0, 
	eDRVADC_RECORD
} E_DRVADC_MODE;
                                                                                                                                                                                                      
typedef enum{ 
	eDRVADC_RECORD_MODE_0 = 0,                                             
	eDRVADC_RECORD_MODE_1,
	eDRVADC_RECORD_MODE_2,
	eDRVADC_RECORD_MODE_3
}E_DRVADC_RECORD_MODE;
   
typedef enum{
	eDRVADC_ADC_INT = 0,                                           
   	eDRVADC_AUD_INT,
	eDRVADC_LVD_INT,
	eDRVADC_WT_INT 
}E_DRVADC_INT;
                                                                                                                                                                       
typedef enum{
	eDRVADC_BAND_P0P5 = 0,                                             
	eDRVADC_BAND_P0P75
}E_DRVADC_UPBAND;
    
typedef enum{
	eDRVADC_BAND_N0P5 = 0,                                             
	eDRVADC_BAND_N0P75
}E_DRVADC_DOWNBAND;   

typedef enum{
	eDRVADC_MAX_N6P75 = 0,                                            
	eDRVADC_MAX_N0P75,
	eDRVADC_MAX_P5P25,
	eDRVADC_MAX_P11P25,
	eDRVADC_MAX_P17P25,
	eDRVADC_MAX_P23P25,
	eDRVADC_MAX_P29P25,
	eDRVADC_MAX_P35P25
}E_DRVADC_MAX_CLAMP;   


typedef enum{ 
	eDRVADC_MIN_N12 = 0,                                             
	eDRVADC_MIN_N6,
	eDRVADC_MIN_P0,
	eDRVADC_MIN_P6,
	eDRVADC_MIN_P12,
	eDRVADC_MIN_P18,
	eDRVADC_MIN_P24,
	eDRVADC_MIN_P30
}E_DRVADC_MIN_CLAMP;    

typedef enum{ 
	eDRVADC_PRE_P0 = 0,                                             
	eDRVADC_PRE_P8,
	eDRVADC_PRE_P14,
	eDRVADC_PRE_P20
}E_DRVADC_PREGAIN;  

typedef enum{ 
	eDRVADC_NG_N30 = 0,                                             
	eDRVADC_NG_N36,
	eDRVADC_NG_N42,
	eDRVADC_NG_N48
}E_DRVADC_NOISEGATE;  

typedef enum{ 
	eDRVADC_POST_N12 = 0,                                             
	eDRVADC_POST_N11P25,
	eDRVADC_POST_N10P5,
	eDRVADC_POST_N9P75,
	eDRVADC_POST_N9,
	eDRVADC_POST_N8P25,
	eDRVADC_POST_N7P5,	
	eDRVADC_POST_N6P75,
	eDRVADC_POST_N6,
	eDRVADC_POST_N5P25,
	eDRVADC_POST_N4P5,
	eDRVADC_POST_N3P75,
	eDRVADC_POST_N3,
	eDRVADC_POST_N2P25,
	eDRVADC_POST_N1P5,
	eDRVADC_POST_N0P75,
	eDRVADC_POST_P0,
	eDRVADC_POST_P0P75,
	eDRVADC_POST_P1P5,
	eDRVADC_POST_P2P25,
	eDRVADC_POST_P3,
	eDRVADC_POST_P3P75,
	eDRVADC_POST_P4P5,
	eDRVADC_POST_P5P25,
	eDRVADC_POST_P6,
	eDRVADC_POST_P6P75,
	eDRVADC_POST_P7P5,
	eDRVADC_POST_P8P25,
	eDRVADC_POST_P9,
	eDRVADC_POST_P9P75,
	eDRVADC_POST_P10P5,
	eDRVADC_POST_P11P25,
	eDRVADC_POST_P12,
	eDRVADC_POST_P12P75,
	eDRVADC_POST_P13P5,
	eDRVADC_POST_P14P25,
	eDRVADC_POST_P15,
	eDRVADC_POST_P15P75,
	eDRVADC_POST_P16P5,
	eDRVADC_POST_P17P25,
	eDRVADC_POST_P18,
	eDRVADC_POST_P18P75,
	eDRVADC_POST_P19P5,
	eDRVADC_POST_P20P25,
	eDRVADC_POST_P21,
	eDRVADC_POST_P21P75,
	eDRVADC_POST_P22P5,
	eDRVADC_POST_P23P25,
	eDRVADC_POST_P24,
	eDRVADC_POST_P24P75,
	eDRVADC_POST_P25P5,
	eDRVADC_POST_P26P25,
	eDRVADC_POST_P27,
	eDRVADC_POST_P27P75,
	eDRVADC_POST_P28P5,
	eDRVADC_POST_P29P25,
	eDRVADC_POST_P30,
	eDRVADC_POST_P30P75,
	eDRVADC_POST_P31P5,
	eDRVADC_POST_P32P25,
	eDRVADC_POST_P33,
	eDRVADC_POST_P33P75,
	eDRVADC_POST_P34P5,
	eDRVADC_POST_P35P25E_DRVSYS_SRC
}E_DRVADC_POSTGAIN;

typedef enum{                            
	eDRVADC_TSCREEN_NORMAL = 0,         	  
	eDRVADC_TSCREEN_SEMI,       
	eDRVADC_TSCREEN_AUTO,
	eDRVADC_TSCREEN_TRIG   
}E_DRVADC_TSC_MODE;  

typedef enum{                           
	eDRVADC_TSCREEN_4WIRE = 0,         	  
	eDRVADC_TSCREEN_5WIRE,       
	eDRVADC_TSCREEN_8WIRE,
	eDRVADC_TSCREEN_UNUSED                                                 
}E_DRVADC_TSC_TYPE;  


typedef enum{                            
	eDRVADC_PEAK_1 = 0,         
	eDRVADC_PEAK_2,       
	eDRVADC_PEAK_4,
	eDRVADC_PEAK_8      
}E_DRVADC_PEAK_METHOD; 

typedef enum{
	eDRVSYS_EXT = 0,														
	eDRVSYS_X32K = 1,													
	eDRVSYS_APLL = 2,													
	eDRVSYS_UPLL = 3
}E_SYS_SRC;

typedef void (*PFN_DRVADC_CALLBACK)(void);

#define outp32(addr, val) 	writel(val, addr)
#define inp32(addr)			readl(addr)  
extern unsigned int w55fa93_upll_clock, w55fa93_apll_clock, w55fa93_ahb_clock;
/*----------------------------------------------------------------------------------------------------------
 Global file scope (static) variables                                                                   	 			
----------------------------------------------------------------------------------------------------------*/

/*
	System deafult use UPLL as system clock. 
	APLL is for TV, ADC and SPU use.
	Sample rate for 8000, 12000, 20000, 24000 use APLL 153.6MHz
*/
void DrvADC_AudioRecordSampleRate(UINT32 eSrcClock, UINT32 u32SampleRateHz)
{
	UINT32 u32TotalDiv, u32Tmp;
	UINT32 u32IdxN0, u32IdxN1; 
	UINT32 u32IdxN00=1, u32IdxN11=1; 					
	UINT32 u32PllOutKHz;
	ENTER();

	if(eSrcClock == eDRVSYS_UPLL)
		u32PllOutKHz = w55fa93_upll_clock; 	
	else
		u32PllOutKHz = w55fa93_apll_clock; 	
	SDBG("ADC PLL clock = %d Hz\n", u32PllOutKHz);												
	u32TotalDiv = (u32PllOutKHz*1000)/(1280*u32SampleRateHz);
	if(u32TotalDiv>(8*256)){
		printk("Can not find the adc divider for the sampling rate\n");
		return; 				
	}
	for(u32IdxN1=1;u32IdxN1<=256;u32IdxN1=u32IdxN1+1)
	{
		for(u32IdxN0=2;u32IdxN0 <= 8;u32IdxN0=u32IdxN0+1)								
		{//u32IdxN0 != 1
			if(u32TotalDiv==(u32IdxN0*u32IdxN1))
			{
				u32IdxN00 = u32IdxN0;
				u32IdxN11 = u32IdxN1;									
				break; 
			}	
		}							
		if(u32TotalDiv==((u32IdxN00)*u32IdxN11))											
			break;
		
	}	
	SDBG("DIV0 = %d \n", u32IdxN00);	
	SDBG("DIV1 = %d \n", u32IdxN11);	
	u32Tmp = (inp32(REG_CLKDIV3) & ~(ADC_N1 | ADC_S | ADC_N0)) | 
					( (((u32IdxN11-1) <<24) | ((u32IdxN00-1) << 16) | (eSrcClock<<19) ));
	outp32(REG_CLKDIV3, u32Tmp);																					
}	



extern int w55fa93_set_apll_clock(unsigned int clock);
/*----------------------------------------------------------------------------------------------------------
 FUNCTION                                                                                                					
 		DrvADC_StartRecord()		                                                                       			
                                                                                                         						
 DESCRIPTION                                                                                             					
     	Start to record Audio data. This function only can be used in                                      		
      ADC_RECORD mode.                                                                                   				
                                                                                                         						
 INPUTS                                                                                                  					
      none														                        
                                                                                                         						
 OUTPUTS                                                                                                 					
      none														                        
                                                                                                         						
 RETURN                                                                                                  					
      none				                                                                               				
                                                                                                         						
----------------------------------------------------------------------------------------------------------*/
void 
DrvADC_StartRecord(
	E_DRVADC_RECORD_MODE eRecordMode
	)
{
	ENTER();
    	// Clean INT status for safe 
    	outp32(REG_AUDIO_CON, inp32(REG_AUDIO_CON) | 
    				(AUDIO_RESET | AUDIO_INT) );
	outp32(REG_AUDIO_CON, inp32(REG_AUDIO_CON) & 
					~(AUDIO_RESET | AUDIO_INT) );
     	//Enable record function 
    	outp32(REG_AUDIO_CON,  (inp32(REG_AUDIO_CON) & ~AUDIO_INT_MODE) | 
						   (((eRecordMode << 30) &  AUDIO_INT_MODE) |
						   AUDIO_EN) );
}


/*----------------------------------------------------------------------------------------------------------
 FUNCTION                                                                                                					
 		DrvADC_StopRecord()                                                                                			
                                                                                                         						
 DESCRIPTION                                                                                             					
     	Stop recording Audio data. This function only can be used in                                       		
      ADC_RECORD mode.                                                                                   				
                                                                                                         						
 INPUTS                                                                                                  					
      none														                        
                                                                                                         						
 OUTPUTS                                                                                                 					
      none														                        
                                                                                                         						
 RETURN                                                                                                  					
      none				                                                                               				
                                                                                                         						
----------------------------------------------------------------------------------------------------------*/
void 
DrvADC_StopRecord(
	void
	)
{
	ENTER();
    	//Disable record function 
	outp32(REG_AUDIO_CON, inp32(REG_AUDIO_CON) & (~AUDIO_EN));
}


/*----------------------------------------------------------------------------------------------------------                                                                                                 						
 FUNCTION                                                                                                					
 		DrvADC_StartConvert()		                                                                       			
                                                                                                         						
 DESCRIPTION                                                                                             					
     	Start to convert ADC data. This function only can be used in                                       	
      ADC_NORMAL mode.                                                                                   				
                                                                                                         						
 INPUTS                                                                                                  					
      u32Channel: The analog input channel and it could be ch2~ch7.                                      		
                                                                                                         						
 OUTPUTS                                                                                                 					
      none														                        
                                                                                                         						
 RETURN                                                                                                  					
      none				                                                                               				
                                                                                                         						
----------------------------------------------------------------------------------------------------------*/
void 
DrvADC_StartConvert(
	UINT32 u32Channel
	)
{
	ENTER();
    	
	if((u32Channel >= 2) && (u32Channel <= 7))
    	{
         	//Clean INT flag for safe and trigger start 
	    	outp32(REG_ADC_CON, (inp32(REG_ADC_CON)|ADC_CONV) | 
	    				((u32Channel << 9)| ADC_INT) );
    	}
}


/*----------------------------------------------------------------------------------------------------------                                                                                               						
 FUNCTION                                                                                                					
 		DrvADC_IsConvertReady()	                                                                           		
                                                                                                         						
 DESCRIPTION                                                                                             					
     	check if ADC (not audio) is converted OK       			    	                                   	
                                                                                                         						
 INPUTS                                                                                                  					
      none														                        
                                                                                                         						
 OUTPUTS                                                                                                 					
      none 														                        
                                                                                                         						
 RETURN                                                                                                  					
      TURE : Conversion finished		    						                                       	
      FALSE: Under converting         							                                       		
                                                                                                         						
----------------------------------------------------------------------------------------------------------*/
BOOL 
DrvADC_IsConvertReady(
	void
	)
{
	ENTER();

	return ((inp32(REG_ADC_CON)&ADC_FINISH)? TRUE:FALSE);		//Check finished?	
}					

BOOL 
DrvADC_Polling_ADCIntStatus(
	void
	)
{
	ENTER();

	return ((inp32(REG_ADC_CON)&ADC_INT)? TRUE:FALSE);			//Check Int?
}
/*----------------------------------------------------------------------------------------------------------                                                                                                         						
 FUNCTION                                                                                                					
 		DrvADC_GetConvertData()	                                                                           		
                                                                                                         						
 DESCRIPTION                                                                                             					
     	Get the converted ADC value in ADC_NORMAL mode    				                        
                                                                                                         						
 INPUTS                                                                                                  					
      none														                        
                                                                                                         						
 OUTPUTS                                                                                                 					
      none 														                       	
                                                                                                         						
 RETURN                                                                                                  					
      The ADC value            		    						                                       		
                                                                                                         						
----------------------------------------------------------------------------------------------------------*/
UINT32 
DrvADC_GetConvertData(
	void
	)
{
	ENTER();
	
    	return (inp32(REG_ADC_XDATA));
}


/*---------------------------------------------------------------------------------------------------------                                                                                                     
 FUNCTION                                                                                                
 		DrvADC_GetRecordData()	                                                                           
                                                                                                         
 DESCRIPTION                                                                                             
     	Get the converted ADC value in ADC_RECORD mode    				                                   
                                                                                                         
 INPUTS                                                                                                  
      none														                                       
                                                                                                         
 OUTPUTS                                                                                                 
      none 														                                       
                                                                                                         
 RETURN                                                                                                  
      The pointer to the record data. The data length is 8 samples                                        
                                                                                                         
---------------------------------------------------------------------------------------------------------*/
PINT16 
DrvADC_GetRecordData(void)
{
	ENTER();
     	
	//Return the base address of converted data. There are 8 samples 
    	return (PINT16)REG_AUDIO_BUF0;
}
/*
void DrvADC_EnableHighPassFilter(
	BOOL bIsEnableFilter
	)
{
	UINT32 u32RegData = inp32(REG_AUDIO_CON);
	bIsEnableFilter?(u32RegData=u32RegData|AUDIO_HPEN):(u32RegData=u32RegData&(~AUDIO_HPEN));
	outp32(REG_AUDIO_CON, u32RegData);
}	

BOOL DrvADC_IsEnableHighPassFilter(void)
{
	return ((inp32(REG_AUDIO_CON) & AUDIO_HPEN)>>26);
}
*/

/*---------------------------------------------------------------------------------------------------------
FUNCTION                                                                                               
	DrvADC_EnableLvd()		                                                                           
                                                                                                 
DESCRIPTION                                                                                             
	Enable low voltage detection function                                                              
                                                                                               
INPUTS                                                                                                
	u32Level - [in], Set trigger voltage. it could be 0~7 and the relative voltages are 2.0, 2.1, 2.2  
               2.3, 2.4, 2.5, 2.6, 2.7 Volt.  			                                       
                                                                                                 
OUTPUTS                                                                                               
	none														                                       
                                                                                                
RETURN                                                                                                  
	none				                                                                               
                                                                                                
REMARKS                                                                                                
	LVD interrupt enable must be set first if using LVD interrupt.	                                   
	Because the LVD interrupt status bit will be set after LV_EN 	                                   
	bit was set. 													                                  
                                                                                                        
---------------------------------------------------------------------------------------------------------*/
void 
DrvADC_EnableLvd(
	UINT32 u32Level
	)
{
	ENTER();

	if(u32Level > 7)
		u32Level = 7;
		
	outp32(REG_LV_CON, (inp32(REG_LV_CON) & (~SW_CON)) |
						 (LV_EN | u32Level));
}


/*---------------------------------------------------------------------------------------------------------
                                                                                                         
 FUNCTION                                                                                                
 		DrvADC_DisableLvd()		                                                                           
                                                                                                         
 DESCRIPTION                                                                                             
     	Disable the low voltage detection function      			                                       
                                                                                                         
 INPUTS                                                                                                  
      none														                                       
                                                                                                         
 OUTPUTS                                                                                                 
      none														                                       
                                                                                                         
 RETURN                                                                                                  
      none				                                                                               
                                                                                                         
                                                                                                         
---------------------------------------------------------------------------------------------------------*/
void 
DrvADC_DisableLvd(
	void
	)
{
	ENTER();

	outp32(REG_LV_CON,inp32(REG_LV_CON)&(~LV_EN));
}


/*---------------------------------------------------------------------------------------------------------
                                                                                                         
 FUNCTION                                                                                                
 		DrvADC_EnableInt()		                                   	   	                                   
                                                                                                         
 DESCRIPTION                                                                                             
     	enable ADC interrupt and setup callback function 	        	                                   
                                                                                                         
 INPUTS                                                                                                  
      callback                                                                                           
          The callback funciton                                                                          
                                                                                                         
      u32UserData                                                                                        
          The user's data to pass to the callback function                                               
                                                                                                         
 OUTPUTS                                                                                                 
      none 														                                       
                                                                                                         
 RETURN                                                                                                  
      none														                                       
                                                                                                         
---------------------------------------------------------------------------------------------------------*/
ERRCODE 
DrvADC_EnableInt(
	E_DRVADC_INT eIntType
	)
{
	ENTER();

   	// Enable adc interrupt 
    	switch(eIntType)
    	{
    		case eDRVADC_ADC_INT: 
		 	outp32(REG_ADC_CON, inp32(REG_ADC_CON) | ADC_INT_EN);	    		                                     
    	    	break; 
	   	case eDRVADC_AUD_INT:
	   		outp32(REG_AUDIO_CON, inp32(REG_AUDIO_CON) | AUDIO_INT_EN);	 
	   		break;
		case eDRVADC_LVD_INT:
			outp32(REG_ADC_CON, inp32(REG_ADC_CON) | LVD_INT_EN);	 
			break;
		case eDRVADC_WT_INT:     
			outp32(REG_ADC_CON, inp32(REG_ADC_CON) | WT_INT_EN);	 
			break;	
		default:
			return E_DRVADC_INVALID_INT;
    	}			
    	return Successful;
}

ERRCODE 
DrvADC_DisableInt(
	E_DRVADC_INT eIntType
	)
{
	ENTER();

	 // Enable adc interrupt 
    	switch(eIntType)
    	{
    		case eDRVADC_ADC_INT:
    			outp32(REG_ADC_CON, inp32(REG_ADC_CON) & ~ADC_INT_EN);	                                      
    	    	break; 
	   	case eDRVADC_AUD_INT:
	   		outp32(REG_AUDIO_CON, inp32(REG_AUDIO_CON) & ~AUDIO_INT_EN);	 
	   		break;
		case eDRVADC_LVD_INT:
			outp32(REG_ADC_CON, inp32(REG_ADC_CON) & ~LVD_INT_EN);	
			break;
		case eDRVADC_WT_INT:
			outp32(REG_ADC_CON, inp32(REG_ADC_CON) & ~WT_INT_EN);     
			break;	
		default:
			return E_DRVADC_INVALID_INT;
    }			
    return Successful;
}

ERRCODE DrvADC_ClearInt(E_DRVADC_INT eIntType)
{
	ENTER();

    	switch(eIntType)
    	{
    		case eDRVADC_ADC_INT: 
		 	outp32(REG_ADC_CON, (inp32(REG_ADC_CON) & ~(ADC_INT|LVD_INT|WT_INT)) | 
		 					ADC_INT);	    		                                     
    	    	break; 
	   	case eDRVADC_AUD_INT:
	   		outp32(REG_AUDIO_CON, inp32(REG_AUDIO_CON) | AUDIO_INT);	 
	   		break;
		case eDRVADC_LVD_INT:
			outp32(REG_ADC_CON, (inp32(REG_ADC_CON) & ~(ADC_INT|LVD_INT|WT_INT)) | 
							LVD_INT);	 
			break;
		case eDRVADC_WT_INT:     
			outp32(REG_ADC_CON, (inp32(REG_ADC_CON) & ~(ADC_INT|LVD_INT|WT_INT)) |  
							WT_INT);	 
			break;	
		default:
			return E_DRVADC_INVALID_INT;	
    }			
   	return Successful;    	
}	

BOOL DrvADC_PollInt(E_DRVADC_INT eIntType)
{	
   	UINT32 u32IntSt = 0;
	
	ENTER();
    	switch(eIntType)
    	{
    		case eDRVADC_ADC_INT: 
		 	u32IntSt = inp32(REG_ADC_CON) & ADC_INT;	    		                                     
    	    	break; 
	   	case eDRVADC_AUD_INT:
	   		u32IntSt = inp32(REG_AUDIO_CON) & AUDIO_INT;	 
	   		break;
		case eDRVADC_LVD_INT:
			u32IntSt = inp32(REG_ADC_CON) & LVD_INT;	 
			break;
		case eDRVADC_WT_INT:     
			u32IntSt = inp32(REG_ADC_CON) & WT_INT;	 
			break;	
    }			
    if( u32IntSt != 0)
    	return TRUE;
	else
    	return FALSE;		    	
}


/*---------------------------------------------------------------------------------------------------------
                                                                                                         
 FUNCTION                                                                                                
 		DrvADC_IsLowVoltage()		                                                                       
                                                                                                         
 DESCRIPTION                                                                                             
     	Get the low voltage status.              	 					                                   
                                                                                                         
 INPUTS                                                                                                  
      None														                                       
                                                                                                         
 OUTPUTS                                                                                                 
      None 														                                       
                                                                                                         
 RETURN                                                                                                  
      TURE: low voltage detected									                                       
      FALSE: not low voltage detected    						                                           
                                                                                                         
---------------------------------------------------------------------------------------------------------*/
BOOL 
DrvADC_IsLowVoltage(
	void
	)
{
	ENTER();

	return ((inp32(REG_LV_STS)&LV_status)? TRUE:FALSE);
}						


/*---------------------------------------------------------------------------------------------------------
                                                                                                         
 FUNCTION                                                                                                
 		DrvADC_IsAudioDataReady()	                                                                       
                                                                                                         
 DESCRIPTION                                                                                             
     	Check if the recording data is converted OK.        			                                   
                                                                                                         
 INPUTS                                                                                                  
      none														                                       
                                                                                                         
 OUTPUTS                                                                                                 
      none 														                                       
                                                                                                         
 RETURN                                                                                                  
      TURE: data is ready											                                       
      FALSE: data is not ready									                                       
                                                                                                         
---------------------------------------------------------------------------------------------------------*/
BOOL 
DrvADC_GetRecordReadyFlag(
	void
	)
{
	ENTER();
	
	return ((inp32(REG_AUDIO_CON)&AUDIO_INT)? TRUE:FALSE);
}	


/*---------------------------------------------------------------------------------------------------------
                                                                                                         
 FUNCTION                                                                                                
 		DrvADC_ClearRecordReadyFlag()	                                                                   
                                                                                                         
 DESCRIPTION                                                                                             
     	To clear the recording ready flag.       	        			                                   
                                                                                                         
 INPUTS                                                                                                  
      None														                                       
                                                                                                         
 OUTPUTS                                                                                                 
      None 														                                       
                                                                                                         
 RETURN                                                                                                  
      None	                									                                       
                                                                                                         
---------------------------------------------------------------------------------------------------------*/
void DrvADC_ClearRecordReadyFlag(
	void
	)
{
	ENTER();

    	outp32(REG_AUDIO_CON, inp32(REG_AUDIO_CON) & ~AUDIO_INT);
    	outp32(REG_AUDIO_CON, inp32(REG_AUDIO_CON) | AUDIO_INT);
}	


/*---------------------------------------------------------------------------------------------------------
                                                                                                         
 FUNCTION                                                                                                
 		DrvADC_SetMICGain()	                      		                                                   
                                                                                                         
 DESCRIPTION                                                                                             
     	Set record volume gain       									                                   
                                                                                                         
 INPUTS                                                                                                  
      u16MicGainLevel						    					                                       
          The volume gain could be 0 ~ 31 dB.                                                            
                                                                                                         
 OUTPUTS                                                                                                 
      none 														                                       
                                                                                                         
 RETURN                                                                                                  
      none														                                       
                                                                                                         
---------------------------------------------------------------------------------------------------------*/
void DrvADC_SetMICGain(
	UINT16 u16MicGainLevel
	)
{
	ENTER();

	outp32(REG_AGCP1, (inp32(REG_AGCP1)&(~AUDIO_VOL)) |
	       (u16MicGainLevel & AUDIO_VOL));

	outp32(REG_AUDIO_CON, inp32(REG_AUDIO_CON) | AUDIO_VOL_EN);       
}	


/*---------------------------------------------------------------------------------------------------------
                                                                                                         
 FUNCTION                                                                                                
 		DrvADC_GetMICGain()	                      		                                                   
                                                                                                         
 DESCRIPTION                                                                                             
     	Get record volume gain.       									                                   
                                                                                                         
 INPUTS                                                                                                  
      None        						    					                                       
                                                                                                         
 OUTPUTS                                                                                                 
      none 														                                       
                                                                                                         
 RETURN                                                                                                  
      Recording gain in dB.										                                       
                                                                                                         
---------------------------------------------------------------------------------------------------------*/
INT16 DrvADC_GetMICGain(
	void
	)
{
	ENTER();

	return inp32(REG_AGCP1) & AUDIO_VOL;
}	

/*---------------------------------------------------------------------------------------------------------
                                                                                                         
 FUNCTION                                                                                                
 		DrvADC_SetGainControl()	                      		                                               
                                                                                                         
 DESCRIPTION                                                                                             
     	Set Pre-Amplifer and Post-Amplifer					                   							   
                                                                                                         
 INPUTS                                                                                                  
      None        						    					                                       
                                                                                                         
 OUTPUTS                                                                                                 
      none 														                                       
                                                                                                         
 RETURN                                                                                                  
      Recording gain in dB.										                                       
                                                                                                         
---------------------------------------------------------------------------------------------------------*/
void DrvADC_SetGainControl(
	E_DRVADC_PREGAIN ePreGain, 
	E_DRVADC_POSTGAIN ePostGain
	)
{
	ENTER();

     	outp32(REG_AGCP1, (inp32(REG_AGCP1) & ~(AUDIO_VOL|PRAGA)) | 
     				((ePreGain<<8)|ePostGain)); 								     				
}	

void DrvADC_GetGainControl(E_DRVADC_PREGAIN* pePreGain, E_DRVADC_POSTGAIN* pePostGain)
{
	UINT32 u32RegData = inp32(REG_AGCP1);
	ENTER();

	*pePreGain =  (u32RegData & PRAGA)>>8;
	*pePostGain = u32RegData & AUDIO_VOL;							     				
}	
/*---------------------------------------------------------------------------------------------------------
                                                                                                         
 FUNCTION                                                                                                
 		DrvADC_SetOffsetCancellation()	                      		                                       
                                                                                                         
 DESCRIPTION                                                                                             
      The function is only for OP offset callcellation				    								
                                                                                                         
 INPUTS                                                                                                  
      None        						    					                                       
                                                                                                         
 OUTPUTS                                                                                                 
      none 														                                       
                                                                                                         
 RETURN                                                                                                  
      Recording gain in dB.										                                       
                                                                                                         
---------------------------------------------------------------------------------------------------------*/
void DrvADC_SetOffsetCancellation(
	BOOL bIsMuteEnable,
	BOOL bIsOffsetCalibration,
	BOOL bIsHardwareMode,
	UINT32 u32Offset
	)
{
	ENTER();

	outp32(REG_OPOC, (inp32(REG_OPOC) & ~(MUTE_SW | OOC | OPOCM |OPOC_SW)) |
				 ((((bIsMuteEnable ? MUTE_SW:0) |
				 (bIsOffsetCalibration ? OOC:0)) |
				 (bIsHardwareMode ? OPOCM:0)) | 
				 ((u32Offset<<24)&OPOC_SW)) );
}	

void DrvADC_GetOffsetCancellation(
	PBOOL pbIsMuteEnable,
	PBOOL pbIsOffsetCalibration,
	PBOOL pbIsHardwareMode,
	PUINT32 pu32Offset
	)
{
	UINT32 u32RegData = inp32(REG_OPOC);
	ENTER();

	*pbIsMuteEnable = (u32RegData & MUTE_SW)>>31;
	*pbIsOffsetCalibration = (u32RegData & OOC)>>30;
	*pbIsHardwareMode = (u32RegData & OPOCM)>>29;
	*pu32Offset = (u32RegData & OPOC_SW)>>24;
}	

void DrvADC_Mute(BOOL bIsMuteEnable)
{
	if(bIsMuteEnable)
		outp32(REG_OPOC, inp32(REG_OPOC) | MUTE_SW);
	else
		outp32(REG_OPOC, inp32(REG_OPOC) & ~(MUTE_SW));

}

void DrvADC_SetOffsetCancellationEx(
	UINT32 u32SampleNumber,
	UINT32 u32DelaySampleCount
	)
{
	ENTER();

	outp32(REG_OPOC, (inp32(REG_OPOC) & ~(OPOC_TCSN | OPOC_DSC)) |
				 (((u32SampleNumber<<16) & OPOC_TCSN) |
				 (u32DelaySampleCount & OPOC_DSC)) );
}
void DrvADC_GetOffsetCancellationEx(
	PUINT32 pu32SampleNumber,
	PUINT32 pu32DelaySampleCount
	)
{
	UINT32 u32RegData = inp32(REG_OPOC);	
	ENTER();

	*pu32SampleNumber = u32RegData & OPOC_TCSN;
	*pu32DelaySampleCount = u32RegData & OPOC_DSC;
}
void DrvADC_GetOffsetSummarry(
	PUINT32 pu32Offset,
	PUINT32 pu32Summation
	)
{
	UINT32 u32RegData = inp32(REG_OPOCS1);
	ENTER();
	
	*pu32Offset = u32RegData & OP_OFFSET_CAL;
	*pu32Summation = u32RegData & ADC_DATA_SUM;
}
/*---------------------------------------------------------------------------------------------------------
                                                                                                         
 FUNCTION                                                                                                
 		DrvADC_SetNoiseGate()	                      		                                                   
                                                                                                         
 DESCRIPTION                                                                                             
     	Set Pre-Amplifer, Post-Amplifer and offset(Offset Cancellation					                   
                                                                                                         
 INPUTS                                                                                                  
      None        						    					                                       
                                                                                                         
 OUTPUTS                                                                                                 
      none 														                                       
                                                                                                         
 RETURN                                                                                                  
      Noise gate Level gain in -24, -30, -36, -42dB.										               
                                                                                                         
---------------------------------------------------------------------------------------------------------*/
void DrvADC_SetNoiseGate(
	BOOL bIsEnable, 
	E_DRVADC_NOISEGATE eNoiseGateLevel
	)
{
	ENTER();

     	outp32(REG_AGC_CON, (inp32(REG_AGC_CON) & ~(NG_EN |NG_LEVEL)) | 
     				(((bIsEnable <<31)& NG_EN) |
     				((eNoiseGateLevel <<12)& NG_LEVEL)) );									     				
    				
}	

/*---------------------------------------------------------------------------------------------------------
                                                                                                         sound/oss/w55fa93_adc.c:1387: error: expected ';' before ')' token

 FUNCTION                                                                                                
 		DrvADC_GetNoiseGate()	                      		                                                   
                                                                                                         
 DESCRIPTION                                                                                             
     	Set Pre-Amplifer, Post-Amplifer and offset(Offset Cancellation					                   
                                                                                                         
 INPUTS                                                                                                  
      None        						    					                                       
                                                                                                         
 OUTPUTS                                                                                                 
      none 														                                       
                                                                                                         
 RETURN                                                                                                  
      Noise gate Level gain in -24, -30, -36, -42dB.										               
                                                                                                         
---------------------------------------------------------------------------------------------------------*/
void DrvADC_GetNoiseGate(
	PBOOL pbIsEnable, 
	E_DRVADC_NOISEGATE* peNoiseGateLevel
	)
{	
	UINT32 u32RegData = inp32(REG_AGC_CON);
	ENTER();


	*pbIsEnable = (u32RegData & NG_EN)>>31;
	*peNoiseGateLevel = (u32RegData & NG_LEVEL)>>12;
    						     				
}	

/*---------------------------------------------------------------------------------------------------------
                                                                                                         
 FUNCTION                                                                                                
 		DrvADC_SetAutoGainControl()	                      		                                               
                                                                                                         
 DESCRIPTION                                                                                             
     	Set the parameter for AGC														                   
                                                                                                         
 INPUTS                                                                                                  
      bIsEnable    	Enable AGC    						    					                       
      u32OutputLevel  Output target level      						    					           
      eUpBand        	A band in the uper side from u32OutputLevel+-eUpBand							   
      eDownBand       A band in the buttom side from u32OutputLevel+-eUpBand					           
                                                                                                         
 OUTPUTS                                                                                                 
      none 														                                       
                                                                                                         
 RETURN                                                                                                  
      none																				               
                                                                                                         
---------------------------------------------------------------------------------------------------------*/
void DrvADC_SetAutoGainControl(
	BOOL bIsEnable, 
	UINT32 u32OutputLevel,
	E_DRVADC_UPBAND eAdcUpBand,
	E_DRVADC_DOWNBAND eAdcDownBand
	)
{
	ENTER();

     	outp32(REG_AGC_CON, (inp32(REG_AGC_CON) & ~AGC_EN) | 
     				((bIsEnable <<30)& AGC_EN) );     				
     	outp32(REG_AGCP1, ( (inp32(REG_AGCP1) & ~(OTL | UPBAND | DOWNBAND)) | 
     				((u32OutputLevel<<12) & OTL) ) |
     				(((eAdcUpBand <<11)& UPBAND) | 
     				((eAdcDownBand <<10)& DOWNBAND)) );											     				     											     											     											     				
}	
	
/*---------------------------------------------------------------------------------------------------------
                                                                                                         
 FUNCTION                                                                                                
 		DrvADC_GetAutoGainControl()	                      		                                           
                                                                                                         
 DESCRIPTION                                                                                             
     	Set Pre-Amplifer, Post-Amplifer and offset(Offset Cancellation					                   
                                                                                                         
 INPUTS                                                                                                  
      None        						    					                                       
 OUTPUTS                                                                                                 
      bIsEnable    	Enable AGC    						    					                       
      u32OutputLevel  Output target level      						    					           
      eUpBand        	A band in the uper side from u32OutputLevel+-eUpBand							   
      eDownBand       A band in the buttom side from u32OutputLevel+-eUpBand					           
                                                                                                         
 RETURN                                                                                                  
      None.										               										   
                                                                                                         
---------------------------------------------------------------------------------------------------------*/
void DrvADC_GetAutoGainControl(
	PBOOL pbIsEnable, 
	PUINT32 pu32OutputLevel,
	E_DRVADC_UPBAND* peAdcUpBand,
	E_DRVADC_DOWNBAND* peAdcDownBand
	)
{
	
	UINT32 u32RegData = inp32(REG_AGC_CON);
	ENTER();

	*pbIsEnable = (u32RegData & AGC_EN)>>30;
	u32RegData = inp32(REG_AGCP1);
	*pu32OutputLevel = (u32RegData & OTL)>>12; 
    	*peAdcUpBand = 	(u32RegData & UPBAND)>>11; 					     				
    	*peAdcDownBand = (u32RegData & DOWNBAND)>>10; 						     				    						     						     				
}	


void DrvADC_SetAutoGainControlEx(
	BOOL bAttachGainFast, 
	E_DRVADC_PEAK_METHOD ePeakMethod
	)
{
	ENTER();

     	outp32(REG_AGC_CON, (inp32(REG_AGC_CON) & ~(PAVG_MODE | AGAIN_STEP)) | 
     				(((bAttachGainFast <<15)& AGAIN_STEP) |
     				((ePeakMethod<<28) & PAVG_MODE)) );     											     				     											     											     											     				
}

void DrvADC_GetAutoGainControlEx(
	PBOOL pbAttachGainFast, 
	E_DRVADC_PEAK_METHOD* pePeakMethod
	)
{
	UINT32 u32RegData = inp32(REG_AGC_CON);
	ENTER();

	*pbAttachGainFast = (u32RegData & AGAIN_STEP)>>15;
	*pePeakMethod = (u32RegData & PAVG_MODE)>>28; 			     				    						     						     				
}	

/*---------------------------------------------------------------------------------------------------------
                                                                                                         
 FUNCTION                                                                                                
 		DrvADC_SetClampingAGC()	                      		                                               
                                                                                                         
 DESCRIPTION                                                                                             
     	Set the parameter for AGC														                   
                                                                                                         
 INPUTS                                                                                                  
      eAdcMaxClamp    Clamp AGC gain. The output level will be  					                       
						1. Input level + Max gain(db) if the value less OTL								   
						2. OTL if input level + Max gain(db) great OTL									   						         						    					           
      eAdcMinClamp    A band in the uper side from u32OutputLevel+-eUpBand							   
                                                                                                         
 OUTPUTS                                                                                                 
      none 														                                       
                                                                                                         
 RETURN                                                                                                  
      none																				               
                                                                                                         
---------------------------------------------------------------------------------------------------------*/
void DrvADC_SetClampingAGC(
	E_DRVADC_MAX_CLAMP eAdcMaxClamp,
	E_DRVADC_MIN_CLAMP eAdcMinClamp
	)
{		
	ENTER();

     	outp32(REG_AGCP1, (inp32(REG_AGCP1) & ~(MAXGAIN | MINGAIN )) | 
     				(((eAdcMaxClamp << 20) & MAXGAIN)  |
     				((eAdcMinClamp << 16) & MINGAIN)) );											     				     											     											     											     				
}	

void DrvADC_GetClampingAGC(
	E_DRVADC_MAX_CLAMP* peAdcMaxClamp,
	E_DRVADC_MIN_CLAMP* peAdcMinClamp
	)
{
	UINT32 u32RegData = inp32(REG_AGCP1);
	ENTER();

	*peAdcMaxClamp = (u32RegData & MAXGAIN)>>20;
	*peAdcMinClamp = (u32RegData & MINGAIN)>>16;				     				    						     						     				
}	

/*---------------------------------------------------------------------------------------------------------
                                                                                                         
 FUNCTION                                                                                                
 		DrvADC_SetAutoGainTiming()	                      		                                           
                                                                                                         
 DESCRIPTION                                                                                             
     	Set the parameter for AGC														                   
                                                                                                         
 INPUTS                                                                                                  
      u32Period    	Detect max peak in the how many samples    					                       
      u32Attack 		      						    					           					   
      u32Recovery        	A band in the uper side from u32OutputLevel+-eUpBand						   
      u32Hold       A band in the buttom side from u32OutputLevel+-eUpBand					           
                                                                                                         
 OUTPUTS                                                                                                 
      none 														                                       
                                                                                                         
 RETURN                                                                                                  
      none																				               
                                                                                                         
---------------------------------------------------------------------------------------------------------*/
void DrvADC_SetAutoGainTiming(
	UINT32 u32Period,
	UINT32 u32Attack,
	UINT32 u32Recovery,
	UINT32 u32Hold
	)
{  		
	ENTER();
	
     	outp32(REG_AGC_CON, ( (inp32(REG_AGC_CON) & ~(PERIOD | ATTACK | RECOVERY| HOLD)) | 
     				( ((u32Period<<16) & PERIOD) |
     				((u32Attack<<8) & ATTACK)) ) |     				
     				( ((u32Recovery <<4)& RECOVERY) | 
     				(u32Hold & HOLD) ) );
    															     				     											     											     											     				
}	
void DrvADC_GetAutoGainTiming(
	PUINT32 pu32Period,
	PUINT32 pu32Attack,
	PUINT32 pu32Recovery,
	PUINT32 pu32Hold
	)
{  			
	UINT32 u32RegData = inp32(REG_AGC_CON);		

	ENTER();
	*pu32Period = (u32RegData & PERIOD) >> 16;
    	*pu32Attack = (u32RegData & ATTACK) >> 8;
    	*pu32Recovery = (u32RegData & RECOVERY) >> 4;
    	*pu32Hold = u32RegData & HOLD; 								     				     	
}   


/*---------------------------------------------------------------------------------------------------------
                                                                                                         
 FUNCTION                                                                                                
 		DrvADC_SetTouchScreen()	                      		                                               
                                                                                                         
 DESCRIPTION                                                                                             
     	Set the parameter for TSC														                   
                                                                                                         
 INPUTS                                                                                                  
      eTscMode    	Normal mode, Semi-Auto, Auto or Wait for trigger   					               
      eTscWire 		4 wire, 5 wire, 8 wire or unused   					           					   
      bIsPullup		Control the internal pull up PMOS in switch box									   
      bMAVFilter      Enable or disable MAV filter in TSC auto mode                                      
 OUTPUTS                                                                                                 
      none 														                                       
                                                                                                         
 RETURN                                                                                                  
      none																				               
                                                                                                         
---------------------------------------------------------------------------------------------------------*/
void DrvADC_SetTouchScreen(
	E_DRVADC_TSC_MODE eTscMode,
	E_DRVADC_TSC_TYPE eTscWire,
	BOOL bIsPullup,
	BOOL bMAVFilter	
	)
{  			
	ENTER();
	
    	outp32(REG_ADC_CON, (inp32(REG_ADC_CON) & ~(ADC_TSC_MODE)) | 	
    				(eTscMode << 14) );
    	outp32(REG_ADC_TSC, (inp32(REG_ADC_TSC) & ~(ADC_TSC_TYPE | ADC_PU_EN | ADC_TSC_MAV_EN)) | 	
    				( (((eTscWire << 1) & ADC_TSC_TYPE) | ((bIsPullup <<3) & ADC_PU_EN)) |
    				((bMAVFilter<<9) & ADC_TSC_MAV_EN) ));				
}    
void DrvADC_GetTouchScreen(
	E_DRVADC_TSC_MODE* peTscMode,
	E_DRVADC_TSC_TYPE* peTscWire,
	PBOOL pbIsPullup,
	PBOOL pbMAVFilter	
	)
{  		
	UINT32 u32RegData = inp32(REG_ADC_TSC);		
	ENTER();

	*peTscMode = (inp32(REG_ADC_CON) & ADC_TSC_MODE) >> 14;
    	*peTscWire = (u32RegData & ADC_TSC_TYPE) >> 1;
    	*pbIsPullup = (u32RegData & ADC_PU_EN) >> 3;
    	*pbMAVFilter = (u32RegData & ADC_TSC_MAV_EN) >> 9;						     				     											     											     											     				
}    

/*---------------------------------------------------------------------------------------------------------
                                                                                                         
 FUNCTION                                                                                                
 		DrvADC_GetMovingAverage()	                  		                                               
                                                                                                         
 DESCRIPTION                                                                                             
     	Get the moving average for TSC if MAV filter enable								                   
                                                                                                         
 INPUTS                                                                                                  
      pu32AverageX    10 bit moving average for TSC auto mode if MAV enable				               
      pu32AverageY 	10 bit moving average for TSC auto mode if MAV enable  							   
                                                                                                         
 OUTPUTS                                                                                                 
      none 														                                       
                                                                                                         
 RETURN                                                                                                  
      none																				               
                                                                                                         
---------------------------------------------------------------------------------------------------------*/
void DrvADC_GetMovingAverage(
	PUINT16 	pu16AverageX,
	PUINT16 	pu16AverageY
	)
{  		
	ENTER();

	*pu16AverageX = inp32(REG_TSC_MAV_X) & X_MAV_AVG;
	*pu16AverageY = inp32(REG_TSC_MAV_Y) & Y_MAV_AVG;			
}   

void DrvADC_GetMovingData(
	PUINT16 	pu16ArrayX,
	PUINT16 	pu16ArrayY
	)
{  		
	UINT32 u32Idx, u32RegData;
	ENTER();


	for(u32Idx=0;u32Idx<10;u32Idx=u32Idx+1)
	{
		u32RegData = inp32(REG_TSC_SORT10+u32Idx*4);
		*pu16ArrayX++ =  u32RegData & X_MAV;
		*pu16ArrayY++ =  (u32RegData & Y_MAV) >>16;										     				
	}	
}                                     

BOOL 
DrvADC_GetTouchScreenUpDownState(void)
{
	return (inp32(REG_ADC_TSC)&ADC_UD);
}

void DrvADC_GetTscData(
	PUINT16 	pu16XData,
	PUINT16 	pu16YData
	)
{  		
	ENTER();
	*pu16XData = inp32(REG_ADC_XDATA);
	*pu16YData = inp32(REG_ADC_YDATA);					     				     											     											     											     				
}
/*---------------------------------------------------------------------------------------------------------                                                                                                 
 FUNCTION                                                                                                
      DrvADC_Open()			                                                                           
                                                                                                         
 DESCRIPTION                                                                                             
      Open the ADC conversion or Audio record function  			                                       
                                                                                                         
 INPUTS                                                                                                  
      mode:   The work mode of ADC. It could be in normal                                                
              ADC conversion mode or audio recording mode		                                           
	                                                                                                         
      u32ConvClock:                                                                                      
              If working in ADC_NORMAL mode, u32ConvClock is the                                         
              conversion rate.                                                                           
              If working in ADC_RECORD mode, u32ConvClock is the                                         
              sampling rate.                                                                             
                                                                                                         
 OUTPUTS                                                                                                 
      none                                                                                               
                                                                                                         
 RETURN                                                                                                  
      E_SUCCESS           Success			                                                               
      E_DRVADC_ARGUMENT   Wrong argument                                                                 
      E_DRVADC_CLOCK      Unable to output a suitable clock                                              
                                                                                                         
---------------------------------------------------------------------------------------------------------*/

#define REAL_CHIP

static UINT32 u32SrcClk;
static UINT32 u32ApllReg; 
static UINT32 u32ApllClock; 
BOOL bIsAudioInitialize = FALSE;
ERRCODE 
DrvADC_Open(
	E_DRVADC_MODE eDRVADC_Mode, 
	E_SYS_SRC eSrcClock, 
	UINT32 u32ConvClock
	)
{
	UINT32 u32PllOutKHz;
	UINT32 u32Tmp;
	UINT32 u32Reg;
	struct clk *clk;
	
	ENTER();

	/* Back up APLL content */
	u32ApllClock = w55fa93_apll_clock;
	u32ApllReg = inp32(REG_APLLCON);
#if 0
	outp32(REG_APLLCON, 0x867E);	/* APLL to 153.6MHz */  
	w55fa93_apll_clock = 153600;	
#else
	w55fa93_set_apll_clock(153600);
#endif
	if((eDRVADC_Mode != eDRVADC_NORMAL) && (eDRVADC_Mode != eDRVADC_RECORD))
	{													    
	    	return E_DRVADC_ARGUMENT;
	}	           
    	 /* Enable clock and IP reset */
    //outp32(REG_APBCLK, inp32(REG_APBCLK) | ADC_CKE);
	clk = clk_get(NULL, "ADC");
	clk_enable(clk);	
    
	outp32(REG_APBIPRST, inp32(REG_APBIPRST) | ADCRST);
   	outp32(REG_APBIPRST,  inp32(REG_APBIPRST) & ~ADCRST);
	/* Default to use conv bit to control conversion */
	u32Reg = 0;
	u32Reg = u32Reg | (ADC_CON_ADC_EN); /* Enable ADC */

    	/* Use the same clock source as system */
	outp32(REG_CLKDIV3, (inp32(REG_CLKDIV3) & ~ADC_S) | 
					(eSrcClock << 19));	
	u32SrcClk = eSrcClock;					
	switch(eSrcClock)
	{
		case eDRVSYS_X32K:	
					return E_DRVADC_CLOCK;			/* Wrong clock source */			
					break;
		case eDRVSYS_APLL:												
		case eDRVSYS_UPLL:			
					{
						UINT32 u32TotalDiv;
						UINT32 u32IdxN0, u32IdxN1; 
						UINT32 u32IdxN00=1, u32IdxN11=1; 					
#if 0
						u32ExtFreq = sysGetExternalClock();						
						u32PllOutKHz = sysGetPLLOutputKhz(eSYS_UPLL, u32ExtFreq);						
#else
						
						if(eSrcClock == eDRVSYS_UPLL)
							u32PllOutKHz = w55fa93_upll_clock; 	
						else
							u32PllOutKHz = w55fa93_apll_clock; 	
#endif
						SDBG("PLL clock = %d KHz\n", u32PllOutKHz);												
						if(eDRVADC_Mode == eDRVADC_RECORD)
							u32TotalDiv = u32PllOutKHz/(1280*u32ConvClock);
						else
					 		u32TotalDiv = (u32PllOutKHz/50)/u32ConvClock;	
						
						if(u32TotalDiv>(8*256))						
							return E_DRVADC_CLOCK;						
						/*	
						if(u32TotalDiv%2 !=0)	
							u32TotalDiv = u32TotalDiv +1;						
						*/	
						for(u32IdxN1=1;u32IdxN1<=256;u32IdxN1=u32IdxN1+1)
						{
							for(u32IdxN0=2;u32IdxN0 <= 8;u32IdxN0=u32IdxN0+1)								
							{//u32IdxN0 != 1
								if(u32TotalDiv==(u32IdxN0*u32IdxN1))
								{
									u32IdxN00 = u32IdxN0;
									u32IdxN11 = u32IdxN1;		
									SDBG("ADC DIV0 = %d \n", u32IdxN00);		
									SDBG("ADC DIV1 = %d \n", u32IdxN11);									
									break; 
								}	
							}							
							if(u32TotalDiv==((u32IdxN00)*u32IdxN11))											
								break;
							
						}	
						u32Tmp = (inp32(REG_CLKDIV3) & ~(ADC_N1 | ADC_S | ADC_N0)) | 
										( (((u32IdxN11-1) <<24) | ((u32IdxN00-1) << 16) | (eSrcClock<<19) ));
						outp32(REG_CLKDIV3, u32Tmp);															
					}					
					break;
		case eDRVSYS_EXT:	
					{
						UINT32 u32ExtClk, u32AdcDivN1;
						//u32ExtClk = sysGetExternalClock();	
						u32ExtClk = 12000;  	// 12MHz										
						u32AdcDivN1 = (u32ExtClk)/u32ConvClock;	
						if(u32AdcDivN1>256)
							return E_DRVADC_CLOCK;
						outp32(REG_CLKDIV3, (inp32(REG_CLKDIV3) & ~(ADC_N1 | ADC_N0)) |
											((u32AdcDivN1-1) <<24) );	
					}									
					break;
	}														  

	outp32(REG_ADC_CON, u32Reg);

	if(eDRVADC_Mode == eDRVADC_RECORD)
	{
		/* Reset Record Function */
    	volatile UINT32 u32Dly=0x100;
    	outp32(REG_AUDIO_CON, inp32(REG_AUDIO_CON) | AUDIO_RESET);
    	while(u32Dly--);
    	outp32(REG_AUDIO_CON, inp32(REG_AUDIO_CON) & ~(AUDIO_RESET));	    
    	/* Default to Audio Interrupt Mode 0, op offset:b'1000, interrupt enabled */
    	outp32(REG_AUDIO_CON, (AUDIO_HPEN | AUDIO_INT | AUDIO_INT_EN |AUDIO_VOL_EN | AUDIO_RESET));
	   	outp32(REG_AUDIO_CON, (inp32(REG_AUDIO_CON) & ~(AUDIO_CCYCLE | AUDIO_RESET)) | (0x50<<16));//ADC cycle = 50
	    
	   	/* Hardware offset calibration */       
		if(bIsAudioInitialize==FALSE)
		{
			bIsAudioInitialize = TRUE;
	        outp32(REG_AUDIO_CON, inp32(REG_AUDIO_CON) | 
			    			 ( AUDIO_INT | AUDIO_EN ) );
	    	DrvADC_SetOffsetCancellation(FALSE, //BOOL bIsMuteEnable,
								FALSE, 			//BOOL bIsOffsetCalibration,
								TRUE, 			//BOOL bIsHardwareMode,
								0x10);	  		//UINT32 u32Offset   
									  		   				  		   				
			DrvADC_SetOffsetCancellationEx(1,		//255 sample
											256);	//Delay sample count   							
			{
				unsigned long j = 0;
				j = jiffies + 50*HZ/100;			/* Calibration time took about 500ms */
				while(time_before(jiffies, j))
					schedule();							
			}			
        	outp32(REG_AUDIO_CON, inp32(REG_AUDIO_CON) & ~AUDIO_EN);
      	 	DrvADC_SetOffsetCancellation(FALSE,   //BOOL bIsMuteEnable,
												FALSE, //BOOL bIsOffsetCalibration,
												FALSE, //BOOL bIsHardwareMode,
												0x10);	  //UINT32 u32Offset 						
		}													       	     
	}
	else
	{
		 outp32(REG_AUDIO_CON, (inp32(REG_AUDIO_CON) & ~AUDIO_CCYCLE) | (50<<16));
	}	
	return 0;	
}


/*---------------------------------------------------------------------------------------------------------                                                                                             
 FUNCTION                                                                                                
      DrvADC_Close()			                                                                           
                                                                                                         
 DESCRIPTION                                                                                             
     	close ADC								 					                                       
                                                                                                         
 INPUTS                                                                                                  
      none														                                       
                                                                                                         
 OUTPUTS                                                                                                 
      none														                                       
                                                                                                         
 RETURN                                                                                                  
      none				                                                                               
                                                                                                         
---------------------------------------------------------------------------------------------------------*/
void 
DrvADC_Close(
	void
	)
{     
	struct clk *clk;	
	ENTER();

	while(!DrvADC_IsConvertReady());
	outp32(REG_APBIPRST, inp32(REG_APBIPRST) | ADCRST);
	outp32(REG_APBIPRST,  inp32(REG_APBIPRST) & ~ADCRST);
	//sysEnableInterrupt(IRQ_ADC);
	outp32(REG_ADC_CON, inp32(REG_ADC_CON)&~ADC_CON_ADC_EN);
	//outp32(REG_APBCLK, inp32(REG_APBCLK) & ~ADC_CKE);
	clk = clk_get(NULL, "ADC");
	clk_disable(clk);

	/* restore APLL */ 
	outp32(REG_APLLCON, u32ApllReg & 0xFFFF);	/* APLL to 153.6MHz */  
	w55fa93_apll_clock = u32ApllClock;	

	bIsAudioInitialize = FALSE;	
}

/****************************************************************************************************
 * wrapper layer
 ****************************************************************************************************/
#ifdef CONFIG_TOUCHSCREEN_W55FA93
extern int w55fa93ts_open_again(void);
extern int w55fa93ts_close_again(void);
#endif
void adcStopRecord(void)
{
	UINT32 u32Idx;
	ENTER();

	for(	u32Idx=0; u32Idx<0x60; u32Idx=u32Idx+0x4)
	{
		SDBG("0x%x = 0x%x\n", u32Idx, inp32(W55FA93_VA_ADC+u32Idx));
		u32Idx = u32Idx+0x04;
		SDBG("0x%x = 0x%x\n", u32Idx, inp32(W55FA93_VA_ADC+u32Idx));
		u32Idx = u32Idx+0x04;
		SDBG("0x%x = 0x%x\n", u32Idx, inp32(W55FA93_VA_ADC+u32Idx));
		u32Idx = u32Idx+0x04;
		SDBG("0x%x = 0x%x\n", u32Idx, inp32(W55FA93_VA_ADC+u32Idx));	
	}
	//2011-0506 w55fa93_adc_close();
	//2011-0506 w55fa93ts_open_again();
	LEAVE();
	
	return;
}
int adcStartRecord(void)
{
	ENTER();
	//g_u32Period = 4; 
	//adcSetRecordCallBackFunction(fnCallBack);
	DrvADC_EnableInt(eDRVADC_AUD_INT);		
	DrvADC_StartRecord(eDRVADC_RECORD_MODE_1);

	LEAVE();
	
	return 0;
}
void w55fa93_adc_recording_setup(int nsampleRate)
{
	
	ENTER();

	DrvADC_Open(eDRVADC_RECORD,			//Record mode
					eDRVSYS_APLL, 		//Source clock come from UPLL
					8);					//Deafult 8K sample rate. 
	//install_drvadc_calibration_callback(adc_calibration_callback);
	DrvADC_SetGainControl(eDRVADC_PRE_P14, 
							eDRVADC_POST_P0);  

	DrvADC_SetAutoGainTiming(4,		//Period
								4,		//Attack
								4,		//Recovery	
								4);		//Hold
	SDBG("Original REG_AGC_CON = 0x%x\n", inp32(REG_AGC_CON));
	DrvADC_SetAutoGainControl(TRUE,
					    		//11, 			//Output target -12db
								//15, 			//Output target -6db
								//13, 			//Output target -9db
								12, 			//Output target -10.5db
					    		eDRVADC_BAND_P0P5,
					    		eDRVADC_BAND_N0P5);

	DrvADC_SetOffsetCancellation(FALSE,   	//BOOL bIsMuteEnable,
									FALSE, 	//BOOL bIsOffsetCalibration,
									FALSE, 	//BOOL bIsHardwareMode,
									0x10);	//UINT32 u32Offset

	DrvADC_SetOffsetCancellationEx(1,		//255 sample
									512);	//Delay sample count
    DrvADC_SetNoiseGate(FALSE, eDRVADC_NG_N48);

	outp32(REG_AUDIO_CON, inp32(REG_AUDIO_CON) & 
			~AUDIO_INT_MODE & ~AUDIO_INT_EN);	// one sample if finish    
	outp32(REG_AGCP1,inp32(REG_AGCP1) | 0x80000000);	// Enabe EDMA for ADC
	
}
INT32 adcInit(int nsampleRate)
{
	ENTER();
#ifdef CONFIG_TOUCHSCREEN_W55FA93
	w55fa93ts_close_again();
#endif 
	SDBG("Close touch panel \n");
	w55fa93_adc_recording_setup(nsampleRate);
	adcStartRecord();
	LEAVE();
	return 0;	
}

#define W55FA92_ADC_RESET					0x00		
#define W55FA93_ADC_POWER_MANAGEMENT		0x01
#define W55FA93_ADC_LEFT_DIGITAL_VOLUME		0x02
#define W55FA93_ADC_RIGHT_DIGITAL_VOLUME	0x03
#if 0
static const DECLARE_TLV_DB_SCALE(digital_tlv, -2400, 160, 0);
											//	 | 	   |  |--> least value is not muted
											//   |	   |------> step size = 1.6dB (160*0.01)
											//   |-------------> least value = -24dB (-2400*0.01)
#else
static const DECLARE_TLV_DB_SCALE(digital_tlv, -2850, 150, 1);
											//	 | 	   |  |--> least value is muted
											//   |	   |------> step size = 1.5dB (150*0.01)
											//   |-------------> least value = -28.5dB (-2850*0.01)
#endif
/****************************************************************************************************
 * Alsa codec layer
 ****************************************************************************************************/
static const struct snd_kcontrol_new w55fa93adc_snd_controls[] = {
	/* Input PGA volume */
#if 1
	SOC_DOUBLE_R_TLV("PCM Volume",  //"Input PGA Volume",
		(0x0), 					/* Left channel address */
		(0x1),					/* Right channel address */
		0, 								/* min value 0x0 (-28.5db) */
		0xF, 							/* max value 0xF (-6db)*/ 
		0, 								/* Offset */
		digital_tlv),
#else
	SOC_DOUBLE_R_TLV("PCM Volume", W55FA93_ADC_LEFT_DIGITAL_VOLUME, W55FA93_ADC_RIGHT_DIGITAL_VOLUME,
        				0, 15, 1, digital_tlv),			/* set (24-0)/1.6+1 volume levels */
#endif
};

/* Mixer #1: Output (OUT1, OUT2) Mixer: mix AUX, Input mixer output and DAC */
static const struct snd_kcontrol_new w55fa93adc_left_out_mixer[] = {

};

static const struct snd_kcontrol_new w55fa93adc_right_out_mixer[] = {

};

/* OUT3/OUT4 Mixer not implemented */

/* Mixer #2: Input PGA Mute */
static const struct snd_kcontrol_new w55fa93adc_left_input_mixer[] = {      

};
static const struct snd_kcontrol_new w55fa93adc_right_input_mixer[] = {

};

static const struct snd_soc_dapm_widget w55fa93adc_dapm_widgets[] = {  
		SND_SOC_DAPM_ADC("Left ADC", "Power Down",
        W55FA93_ADC_POWER_MANAGEMENT, 1, 1),
        SND_SOC_DAPM_ADC("Right ADC", "Power Down",
        W55FA93_ADC_POWER_MANAGEMENT, 2, 1),
        
        SND_SOC_DAPM_OUTPUT("LHP"),
        SND_SOC_DAPM_OUTPUT("RHP"),
};

static const struct snd_soc_dapm_route audio_map[] = {  
	{"LHP", NULL, "Left ADC"},	// "destination <-- switch <-- source", define left DAC path
    //{"RHP", NULL, "Right ADC"},	// "destination <-- switch <-- source", define Right DAC path
};
#if 1
static int w55fa93adc_add_widgets(struct snd_soc_codec *codec)
{
	ENTER();
        snd_soc_dapm_new_controls(codec, w55fa93adc_dapm_widgets,
                                  ARRAY_SIZE(w55fa93adc_dapm_widgets));

        /* set up the w55fa93adc audio map */
        snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));
	LEAVE();
        return 0;
} 
#endif 

#endif
/*
 * Configure w55fa93adc clock dividers.
 */
static int w55fa93adc_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
                                 int div_id, int div)
{
	ENTER();
	LEAVE();
    return 0;
}

/*
 * @freq:	when .set_pll() us not used, freq is codec MCLK input frequency
 */
static int w55fa93adc_set_dai_sysclk(struct snd_soc_dai *codec_dai, int clk_id,
                                 unsigned int freq, int dir)
{
	ENTER();
	LEAVE();
	return 0;
}

/*
 * Set ADC and Voice DAC format.
 */
static int w55fa93adc_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{	
	ENTER();
	SDBG("Record from ADC is only support sample rate 8K/11025/16K sample rate");
	LEAVE();
	return 0;
}

/*
 * Set PCM DAI bit size and sample rate.
 */
static int w55fa93adc_hw_params(struct snd_pcm_substream *substream,
                            struct snd_pcm_hw_params *params,
                            struct snd_soc_dai *dai)
{
	unsigned int nSamplingRate;

	ENTER();
	nSamplingRate = params_rate(params);
	adcInit(nSamplingRate);
	if(nSamplingRate==16000)
		w55fa93_set_apll_clock(184320);
	else if(nSamplingRate==11025)	
		w55fa93_set_apll_clock(169344);
	else
		w55fa93_set_apll_clock(184320);	/* 8K sample rate */ 
	SDBG("Sample Rate = %d", nSamplingRate);
	DrvADC_AudioRecordSampleRate((UINT32)eDRVSYS_APLL, nSamplingRate);	/* Hz unit */	
	LEAVE();
	return 0;
}

static int w55fa93adc_mute(struct snd_soc_dai *dai, int mute)
{	
	struct snd_soc_codec *codec = dai->codec;
	ENTER();
	ENTER();
    dev_dbg(codec->dev, "%s: %d\n", __func__, mute);

    if (mute){
        DrvADC_Mute(TRUE);
		printk("       Enable Mute\n");
	}else{
        DrvADC_Mute(FALSE);
		printk("       Disable Mute\n");			
	}
	LEAVE();
	return 0;


	LEAVE();
	return 0;
}

static int w55fa93adc_set_bias_level(struct snd_soc_codec *codec,
                                 enum snd_soc_bias_level level)
{
	ENTER();
	LEAVE();
    return 0;
}

#define NAU8822_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops w55fa93adc_codec_dai_ops = {
        .hw_params		= w55fa93adc_hw_params,
        .digital_mute	= w55fa93adc_mute,
        .set_fmt		= w55fa93adc_set_dai_fmt,
        .set_clkdiv		= w55fa93adc_set_dai_clkdiv,
        .set_sysclk		= w55fa93adc_set_dai_sysclk,
};

/* Also supports 12kHz */
struct snd_soc_dai w55fa93adc_codec_dai = {
        .name = "W55FA93ADC HiFi",
        .id = 1,
	#if 0	
        .playback = {
                .stream_name = "Playback",
                .channels_min = 1,
                .channels_max = 2,
                .rates = SNDRV_PCM_RATE_8000_48000,
                .formats = NAU8822_FORMATS,
        },
	#endif	
        .capture = {
                .stream_name = "Capture",
                .channels_min = 1,
                .channels_max = 1,
                .rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |SNDRV_PCM_RATE_16000),
                .formats = NAU8822_FORMATS,
        },
        .ops = &w55fa93adc_codec_dai_ops,
       };
EXPORT_SYMBOL_GPL(w55fa93adc_codec_dai);

static int w55fa93adc_suspend(struct platform_device *pdev, pm_message_t state)
{	
    //struct snd_soc_device *socdev = platform_get_drvdata(pdev);
    //struct snd_soc_codec *codec = socdev->card->codec;

	ENTER();
        
	LEAVE();
    return 0;
}

static int w55fa93adc_resume(struct platform_device *pdev)
{
	ENTER();
	LEAVE();
    return 0;
}

static int w55fa93adc_probe(struct platform_device *pdev)
{
    struct snd_soc_device *socdev = platform_get_drvdata(pdev);
    struct snd_soc_codec *codec;
    int ret = 0;

	ENTER();
    if (w55fa93adc_codec == NULL) {
       	dev_err(&pdev->dev, "Codec device not registered\n");
		ERRLEAVE();
       	return -ENODEV;
    }

    socdev->card->codec = w55fa93adc_codec;
    codec = w55fa93adc_codec;

    /* register pcms */
    ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
    if (ret < 0) {
            dev_err(codec->dev, "failed to create pcms: %d\n", ret);
			ERRLEAVE();	
            goto pcm_err;
    }
#if 1
    snd_soc_add_controls(codec, w55fa93adc_snd_controls,
                            ARRAY_SIZE(w55fa93adc_snd_controls));
    w55fa93adc_add_widgets(codec);
#endif 
	LEAVE();
	return ret;
pcm_err:
	ERRLEAVE();
    return ret;
}

/* power down chip */
static int w55fa93adc_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	ENTER();
        snd_soc_free_pcms(socdev);
        snd_soc_dapm_free(socdev);
	LEAVE();
        return 0;
}

struct snd_soc_codec_device soc_codec_dev_w55fa93adc = {
        .probe		= w55fa93adc_probe,
        .remove		= w55fa93adc_remove,
        .suspend	= w55fa93adc_suspend,
        .resume		= w55fa93adc_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_w55fa93adc);


static __devinit int w55fa93adc_register(struct w55fa93adc_priv *w55fa93adc)
{	
	int ret;
	struct snd_soc_codec *codec = &w55fa93adc->codec;

	ENTER();
	
	if (w55fa93adc_codec) {
		dev_err(codec->dev, "Another w55fa93adc is registered\n");
		ERRLEAVE();
		return -EINVAL;
	}

	/*
	 * Set default system clock to PLL, it is more precise, this is also the
	 * default hardware setting
	 */
/*
	nau8822->sysclk = NAU8822_PLL;
*/
	mutex_init(&codec->mutex);

#if 1
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
#endif 
	snd_soc_codec_set_drvdata(codec, w55fa93adc);
	codec->name = "codecADC";	/* The name can not be NULL */
	codec->owner = THIS_MODULE;
	codec->bias_level = SND_SOC_BIAS_OFF;

	codec->set_bias_level = w55fa93adc_set_bias_level;
    codec->dai = &w55fa93adc_codec_dai;
	codec->num_dai = 1;

	codec->reg_cache_size = W55FA93_ADC_CACHEREGNUM;
   	codec->reg_cache = &w55fa93adc->reg_cache;
	memcpy(codec->reg_cache, w55fa93_adc_reg, sizeof(w55fa93_adc_reg));

//	ret = snd_soc_codec_set_cache_io(codec, 7, 9, SND_SOC_I2C);
  	ret = snd_soc_codec_set_cache_io(codec, 8, 8, SND_SOC_I2C);
    if (ret < 0) {
            dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
            goto err;
    }

    w55fa93adc_codec_dai.dev = codec->dev;

	w55fa93adc_codec = codec;
	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		ERRLEAVE();
		goto err;
	}	

#if 0	
    w55fa93adc_codec_dai.dev = codec->dev;

	w55fa93adc_codec = codec;
	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		ERRLEAVE();
		goto err;
	}	
#endif
	ret = snd_soc_register_dai(&w55fa93adc_codec_dai);
	if (ret != 0) {
			dev_err(codec->dev, "Failed to register DAI: %d\n", ret);
	ERRLEAVE();
			goto err_codec;
	}
	SDBG("After snd_soc_register_dai\n");

	LEAVE();
    return 0;

err_codec:
    snd_soc_unregister_codec(codec);
err:
    kfree(w55fa93adc);
	ERRLEAVE();
        return ret;
}

static __devexit void w55fa93adc_unregister(struct w55fa93adc_priv *w55fa93adc)
{
	ENTER();
        w55fa93adc_set_bias_level(&w55fa93adc->codec, SND_SOC_BIAS_OFF);
        snd_soc_unregister_dai(&w55fa93adc_codec_dai);
        snd_soc_unregister_codec(&w55fa93adc->codec);
        kfree(w55fa93adc);
        w55fa93adc_codec = NULL;
	LEAVE();
}
#if 0
struct w55fa93adc_priv *w55fa93adc;
static __devinit int w55fa93adc_moduleinit(void)
{
    struct snd_soc_codec *codec;
	int ret;

	ENTER();
    w55fa93adc = kzalloc(sizeof(struct w55fa93adc_priv), GFP_KERNEL);
    if (w55fa93adc == NULL)
            return -ENOMEM;
#if 0
    codec = &w55fa93adc->codec;
    codec->hw_write = NULL;  //(hw_write_t)i2c_master_send;
    codec->dev = (void*)"CPU"; /* A string is only */
#else	
	 codec = &w55fa93adc->codec;
	 codec->hw_write = (hw_write_t)i2c_master_send;
	 codec->control_data = i2c;
     i2c_set_clientdata(i2c, w55fa93adc);   
     codec->dev = &i2c->dev;
#endif
	ret = w55fa93adc_register(w55fa93adc);
	LEAVE();
    return ret;
}
#endif 

static __devinit int w55fa93_adc_i2c_probe(struct i2c_client *i2c,
                                      const struct i2c_device_id *id)
{
	
#if 0	//Due to volume, it need to be a global variable. 
        struct w55fa93adc_priv *w55fa93adc;
#endif 
        struct snd_soc_codec *codec;
		ENTER();	
        w55fa93adc = kzalloc(sizeof(struct w55fa93adc_priv), GFP_KERNEL);
        if (w55fa93adc == NULL)
                return -ENOMEM;

        codec = &w55fa93adc->codec;
#if 0
		codec->read = read_reg_cache;
		codec->write = w55fa93_adc_write;
		codec->control_data = i2c;
#else
		codec->hw_write = (hw_write_t)i2c_master_send;
		codec->control_data = i2c;
#endif
        i2c_set_clientdata(i2c, w55fa93adc);
        
        codec->dev = &i2c->dev;
//        codec->dev = 0x11;	// temp value, non-zero 

        return w55fa93adc_register(w55fa93adc);
}
static __devexit int w55fa93_adc_i2c_remove(struct i2c_client *client)
{
        struct w55fa93adc_priv *w55fa93adc_priv = i2c_get_clientdata(client);
        w55fa93adc_unregister(w55fa93adc_priv);
        return 0;
}

static const struct i2c_device_id w55fa93_adc_i2c_id[] = {
        { "w55fa93_adc_i2c", 0 },	/* link mach-w55fa92.c */
        { }
};
MODULE_DEVICE_TABLE(i2c, w55fa93_dac_i2c_id);

static struct i2c_driver w55fa93_adc_i2c_driver = {
        .driver = {
                .name = "W55FA93_ADC_I2C",		/* Same as codec->name */
                .owner = THIS_MODULE,
        },
        .probe =    w55fa93_adc_i2c_probe,
        .remove =   __devexit_p(w55fa93_adc_i2c_remove),
        .id_table = w55fa93_adc_i2c_id,
};
static int __init w55fa93adc_modinit(void)
{
	int ret =0;
	ENTER();
#if 1
	ret = i2c_add_driver(&w55fa93_adc_i2c_driver);
	SDBG("w55fa92-dac-i2c ret = 0x%x \n", ret);        
	return 0;
#else
	w55fa93adc_moduleinit();
#endif
	LEAVE();
	return ret;
}
module_init(w55fa93adc_modinit);

static void __exit w55fa93adc_exit(void)
{
	ENTER();
	kfree(w55fa93adc);
	LEAVE();
}
module_exit(w55fa93adc_exit);

MODULE_DESCRIPTION("ASoC W55FA93ADC codec driver");
MODULE_LICENSE("GPL");
