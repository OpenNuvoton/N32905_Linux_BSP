/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.                                             */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef __DRVADC_H__
#define __DRVADC_H__


/*---------------------------------------------------------------------------------------------------------*/
/* Includes of system headers                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
//#include "wblib.h"
#include <asm/arch/w55fa93_reg.h>
#define ERRCODE		unsigned int
#define UINT8			unsigned char
#define UINT16		unsigned short int
#define UINT32		unsigned int
#define INT8			char
#define INT16			short int
#define INT32			int	

#define BOOL	 		unsigned int
#define FALSE		0
#define TRUE			1
#define PBOOL		BOOL*
#define PUINT32		UINT32*
#define PUINT16		UINT16*
#define PUINT8		UINT8*
#define PINT32		INT32*
#define PINT16		INT16*
#define PINT8			INT8*
#define Successful	0	

#if 0
#ifdef  __cplusplus
extern "C"
{
#endif
#endif
/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
/* version define with SysInfra */
#define	DRVADC_MAJOR_NUM 3
#define	DRVADC_MINOR_NUM 60
#define	DRVADC_BUILD_NUM 001
//#define 	DRVADC_VERSION_NUM    _SYSINFRA_VERSION(DRVADC_MAJOR_NUM, DRVADC_MINOR_NUM, DRVADC_BUILD_NUM)



/* error code */
#define E_DRVADC_ARGUMENT       	0xB800E001//(ADC_BA + 0x01)
#define E_DRVADC_CLOCK              	0xB800E002//(ADC_BA + 0x02)
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

typedef void (*PFN_DRVADC_CALLBACK)(VOID);

		
/*----------------------------------------------------------------------------------------------------------*/
/* inline function declarations                                                                             */
/*----------------------------------------------------------------------------------------------------------*/
static __inline
void DrvADC_Conv(void)
{
	outp32(REG_ADC_CON, inp32(REG_ADC_CON)|ADC_CONV); 
}

static __inline
void DrvADC_TSC_Conv_X(void)
{													
	/* Select X-position dectection	*/				
	outp32(REG_ADC_TSC,inp32(REG_ADC_TSC)&(~ADC_TSC_XY));	
	/* Enable to detect X-position*/				
	outp32(REG_ADC_CON, inp32(REG_ADC_CON)|ADC_CONV);		
}	
		
		
static __inline
void DrvADC_TSC_Conv_Y(void)
{													
	/* Select Y-position dectection */				
	outp32(REG_ADC_TSC,inp32(REG_ADC_TSC)|ADC_TSC_XY);		
	/* Enable to detect Y-position */				
	outp32(REG_ADC_CON, inp32(REG_ADC_CON)|ADC_CONV);		
}

static __inline
UINT32 
DrvADC_TSC_Get_Xdata(void)
{
	return inp32(REG_ADC_XDATA);
}

static __inline
UINT32 
DrvADC_TSC_Get_Ydata(void)
{
	return inp32(REG_ADC_YDATA);
}

static __inline
void DrvADC_Check_Conv_Finish(void)
{
	while((inp32(REG_ADC_CON)&ADC_INT)==0);		
	outp32(REG_ADC_CON, inp32(REG_ADC_CON)&~ADC_INT);			
}

static __inline
void DrvADC_EnableEDMA(
	BOOL bIsEnableEDMA
)
{
	outp32(REG_AGCP1, (inp32(REG_AGCP1) & ~EDMA_MODE) |
					(bIsEnableEDMA? EDMA_MODE: 0) );	
}	  

static __inline
BOOL 
DrvADC_IsEnableEDMA(void)
{
	return 	((inp32(REG_AGCP1) & EDMA_MODE)>>31); 
}	
/*----------------------------------------------------------------------------------------------------------*/
/* Interface function declarations                                                                         		*/
/*----------------------------------------------------------------------------------------------------------*/
void DrvADC_AudioRecordSampleRate(E_SYS_SRC eSysSrc, UINT32 u32SampleRateKHz);
ERRCODE DrvADC_Open(E_DRVADC_MODE mode, E_SYS_SRC eSrcClock, UINT32 u32ConvClock);
//__u32 DrvADC_Open(__u32 mode, __u32 eSrcClock, __u32 u32ConvClock);

void DrvADC_Close(void);

ERRCODE 
DrvADC_EnableInt(
	E_DRVADC_INT eIntType
);

ERRCODE 
DrvADC_DisableInt(
	E_DRVADC_INT eIntType
);

BOOL
DrvADC_PollInt(
	E_DRVADC_INT eIntType
);

ERRCODE 
DrvADC_ClearInt(
	E_DRVADC_INT eIntType
);


BOOL 
DrvADC_Polling_ADCIntStatus(void);

ERRCODE 
DrvADC_InstallCallback(
	E_DRVADC_INT eIntType,
	PFN_DRVADC_CALLBACK pfnCallback,
	PFN_DRVADC_CALLBACK* pfnOldCallback
);

BOOL 
DrvADC_IsLowVoltage(void);

BOOL 
DrvADC_IsConvertReady(void);

BOOL 
DrvADC_GetRecordReadyFlag(void);

void DrvADC_ClearRecordReadyFlag(void);


void DrvADC_EnableLvd(
	UINT32 u32VoltageDivider
);

void DrvADC_DisableLvd(void);

void DrvADC_EnableHighPassFilter(
	BOOL bIsEnableFilter
);

BOOL DrvADC_IsEnableHighPassFilter(void);

UINT32 
DrvADC_GetVersion(void);

void DrvADC_StartRecord(
	E_DRVADC_RECORD_MODE eRecordMode
);

INT16* 
DrvADC_GetRecordData(void);

void DrvADC_StopRecord(void);

void DrvADC_StartConvert(
	UINT32 u32Channel
);

void DrvADC_SetMICGain(
	UINT16 u16MicGainLevel
);

INT16 
DrvADC_GetMICGain(void);

UINT32 
DrvADC_GetConvertClock(void);

UINT32 
DrvADC_GetConvertData(void);



void DrvADC_SetGainControl(
	E_DRVADC_PREGAIN ePreGain, 
	E_DRVADC_POSTGAIN ePostGain
);

void DrvADC_GetGainControl(
	E_DRVADC_PREGAIN* pePreGain, 
	E_DRVADC_POSTGAIN* pePostGain
);
	
void DrvADC_SetNoiseGate(
	BOOL bIsEnable, 
	E_DRVADC_NOISEGATE eNoiseGateLevel
);
	
void DrvADC_GetNoiseGate(
	PBOOL pbIsEnable, 
	E_DRVADC_NOISEGATE* peNoiseGateLevel
);

void DrvADC_SetAutoGainControl(
	BOOL bIsEnable, 
	UINT32 u32OutputLevel,
	E_DRVADC_UPBAND eAdcUpBand,
	E_DRVADC_DOWNBAND eAdcDownBand
);	
	
void DrvADC_GetAutoGainControl(
	PBOOL pbIsEnable, 
	PUINT32 pu32OutputLevel,
	E_DRVADC_UPBAND* peAdcUpBand,
	E_DRVADC_DOWNBAND* peAdcDownBand
);

void DrvADC_SetAutoGainControlEx(
	BOOL bAttachGainFast, 
	E_DRVADC_PEAK_METHOD ePeakMethod
);

void DrvADC_GetAutoGainControlEx(
	PBOOL pbAttachGainFast, 
	E_DRVADC_PEAK_METHOD* pePeakMethod
);
	
void DrvADC_SetClampingAGC(
	E_DRVADC_MAX_CLAMP eAdcMaxClamp,
	E_DRVADC_MIN_CLAMP eAdcMinClamp
);

void DrvADC_GetClampingAGC(
	E_DRVADC_MAX_CLAMP* peAdcMaxClamp,
	E_DRVADC_MIN_CLAMP* peAdcMinClamp
);

void DrvADC_GetAutoGainTiming(
	PUINT32 pu32Period,
	PUINT32 pu32Attack,
	PUINT32 pu32Recovery,
	PUINT32 pu32Hold
);	
	
void DrvADC_SetAutoGainTiming(
	UINT32 u32Period,
	UINT32 u32Attack,
	UINT32 u32Recovery,
	UINT32 u32Hold
);
	
void DrvADC_GetAutoGainTiming(
	PUINT32 pu32Period,
	PUINT32 pu32Attack,
	PUINT32 pu32Recovery,
	PUINT32 pu32Hold
);	
	
	
void DrvADC_SetOffsetCancellation(
	BOOL bIsMuteEnable,
	BOOL bIsOffsetCalibration,
	BOOL bIsHardwareMode,
	UINT32 u32Offset
);	

void DrvADC_GetOffsetCancellation(
	PBOOL pbIsMuteEnable,
	PBOOL pbIsOffsetCalibration,
	PBOOL pbIsHardwareMode,
	PUINT32 pu32Offset
);
	
void DrvADC_SetOffsetCancellationEx(
	UINT32 u32SampleNumber,
	UINT32 u32DelaySampleCount
);

void DrvADC_GetOffsetCancellationEx(
	PUINT32 pu32SampleNumber,
	PUINT32 pu32DelaySampleCount
);

void DrvADC_GetOffsetSummarry(
	PUINT32 pu32Offset,
	PUINT32 pu32Summation
);

void DrvADC_SetTouchScreen(
	E_DRVADC_TSC_MODE eTscMode,
	E_DRVADC_TSC_TYPE eTscWire,
	BOOL bIsPullup,
	BOOL bMAVFilter
);
   
void DrvADC_GetTouchScreen(
	E_DRVADC_TSC_MODE* peTscMode,
	E_DRVADC_TSC_TYPE* peTscWire,
	PBOOL pbIsPullup,
	PBOOL pbMAVFilter
);

void DrvADC_GetMovingAverage(
	PUINT16 	pu16AverageX,
	PUINT16 	pu16AverageY
);

void DrvADC_GetMovingData(
	PUINT16 	pu16ArrayX,
	PUINT16 	pu16ArrayY
);

BOOL 
DrvADC_GetTouchScreenUpDownState(void);
	
void DrvADC_GetTscData(
	PUINT16 	pu16XData,
	PUINT16 	pu16YData
);	


typedef void (*PFN_DRVADC_CAL_CALLBACK)(void); 			/* function pointer */
typedef void (*PFN_ADC_CAL_CALLBACK)(void); 			/* function pointer */

#if 0	
#ifdef  __cplusplus
}
#endif
#endif

#endif

