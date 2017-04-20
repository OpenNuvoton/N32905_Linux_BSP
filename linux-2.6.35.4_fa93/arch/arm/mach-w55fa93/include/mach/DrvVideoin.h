/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved. *
 *                                                              *
 ****************************************************************/
 
#ifndef __DRVVIDEOIN_H__
#define __DRVVIDEOIN_H__

#include <mach/w55fa93_reg.h>

#define ERRCODE		UINT32

// #define Constant
#define DRVVIDEOIN_MAJOR_NUM 3
#define DRVVIDEOIN_MINOR_NUM 60
#define DRVVIDEOIN_BUILD_NUM 001

//Error message
// E_DRVVIDEOIN_PIN_UNAVAILABLE 			GPIO pin unavailable for VideoIn IP
// E_DRVVIDEOIN_WRITE_SENSOR_FAIL 			Writing sensor through I2c is fail
// E_DRVVIDEOIN_READ_SENSOR_FAIL			Reading sensor through I2c is fail
// E_DRVVIDEOIN_INTERLACE_SCALE_FAIL		Set the downscale factor fail 
// E_DRVVIDEOIN_INVALID_INT				Invalid interrupt
// E_DRVVIDEOIN_INVALID_BUF				Invalid buffer
// E_DRVVIDEOIN_INVALID_PIPE				Invalid pipe

#define E_DRVVIDEOIN_PIN_UNAVAILABLE   		0xFFFF3001
#define E_DRVVIDEOIN_WRITE_SENSOR_FAIL   	0xFFFF3002
#define E_DRVVIDEOIN_READ_SENSOR_FAIL   	0xFFFF3003
#define E_DRVVIDEOIN_INTERLACE_SCALE_FAIL  0xFFFF3004
#define E_DRVVIDEOIN_INVALID_INT   			0xFFFF3005
#define E_DRVVIDEOIN_INVALID_BUF   			0xFFFF3006
#define E_DRVVIDEOIN_INVALID_PIPE  			0xFFFF3007

#define UINT8		__u8
#define UINT16	__u16
#define UINT32	__u32
#define INT8		__s8
#define INT16		__s16
#define INT32		__s32

//typedef enum
//{	
//	FALSE =0,
//	TRUE =1
//}BOOL;

#define BOOL	 		UINT32 
#define FALSE		0
#define TRUE			1


#define PBOOL		BOOL*
#define PUINT8		UINT8*
#define PUINT16		UINT16*
#define PUINT32		UINT32*
#define Successful	0
#define ERRCODE 		UINT32 

typedef void (*PFN_DRVVIDEOIN_CALLBACK)(
				UINT8 u8PacketBufID, 
				UINT8 u8PlanarBufID, 
				UINT8 u8FrameRate, 
				UINT8 u8Filed);

/* Input device  */
typedef enum
{
	eDrvVideoIn_SNR_CCIR601 =0,
	eDrvVideoIn_SNR_CCIR656,
	eDrvVideoIn_TVD_CCIR601,
	eDrvVideoIn_TVD_CCIR656,
	eDrvVideoIn_2nd_SNR_CCIR601,
	eDrvVideoIn_3rd_SNR_CCIR601
}E_DRVVIDEOIN_DEV_TYPE;

/* Interrupt type */
typedef enum
{
	eDRVVIDEOIN_MDINT = 0x100000,
	eDRVVIDEOIN_ADDRMINT = 0x80000,
	eDRVVIDEOIN_MEINT = 0x20000,
	eDRVVIDEOIN_VINT = 0x10000	
}E_DRVVIDEOIN_INT_TYPE;


/* Pipe enable */
typedef enum
{
	eDRVVIDEOIN_BOTH_PIPE_DISABLE = 0,
	eDRVVIDEOIN_PLANAR = 1,
	eDRVVIDEOIN_PACKET = 2,
	eDRVVIDEOIN_BOTH_PIPE_ENABLE = 3	
}E_DRVVIDEOIN_PIPE;

/* Base address */
typedef enum
{
	eDRVVIDEOIN_BUF0 =0,
	eDRVVIDEOIN_BUF1,	
	eDRVVIDEOIN_BUF2
}E_DRVVIDEOIN_BUFFER;

/* For DrvVideoIn_SetOperationMode */
#define DRVVIDEOIN_CONTINUE   1   

/* Input Data Order For YCbCr */
typedef enum
{
	eDRVVIDEOIN_IN_UYVY =0,
	eDRVVIDEOIN_IN_YUYV,
	eDRVVIDEOIN_IN_VYUY,		
	eDRVVIDEOIN_IN_YVYU
}E_DRVVIDEOIN_ORDER;


typedef enum
{
	eDRVVIDEOIN_IN_YUV422 = 0,
	eDRVVIDEOIN_IN_RGB565
}E_DRVVIDEOIN_IN_FORMAT;                                  
                                                                
typedef enum
{
	eDRVVIDEOIN_OUT_YUV422 = 0,
	eDRVVIDEOIN_OUT_ONLY_Y,
	eDRVVIDEOIN_OUT_RGB555,		
	eDRVVIDEOIN_OUT_RGB565
}E_DRVVIDEOIN_OUT_FORMAT;	

typedef enum
{
	eDRVVIDEOIN_TYPE_CCIR601 = 0,
	eDRVVIDEOIN_TYPE_CCIR656
}E_DRVVIDEOIN_TYPE;     

typedef enum
{
	eDRVVIDEOIN_SNR_APLL = 2,
	eDRVVIDEOIN_SNR_UPLL = 3
}E_DRVVIDEOIN_SNR_SRC;  

/* Define data type (struct, union…) */

/* Define function */
void DrvVideoIn_Init(
	BOOL bIsEnableSnrClock,
	E_DRVVIDEOIN_SNR_SRC eSnrSrc,	
	UINT32 u32SensorFreq,						//KHz unit
	E_DRVVIDEOIN_DEV_TYPE eDevType
);

ERRCODE 
DrvVideoIn_Open(
	UINT32 u32EngFreqKHz, 
	UINT32 u32SensorFreq
);			
	
void DrvVideoIn_Close(void);
	
void DrvVideoIn_Reset(void);

ERRCODE 
DrvVideoIn_InstallCallback(
	E_DRVVIDEOIN_INT_TYPE eIntType, 
	PFN_DRVVIDEOIN_CALLBACK pfnCallback,
	PFN_DRVVIDEOIN_CALLBACK *pfnOldCallback
);	

ERRCODE 
DrvVideoIn_EnableInt(
	E_DRVVIDEOIN_INT_TYPE eIntType
);

ERRCODE
DrvVideoIn_DisableInt(
	E_DRVVIDEOIN_INT_TYPE eIntType
);

BOOL
DrvVideoIn_IsIntEnabled(
	E_DRVVIDEOIN_INT_TYPE eIntType
);

BOOL
DrvVideoIn_PollInt(
	E_DRVVIDEOIN_INT_TYPE eIntType
);
	
void DrvVideoIn_SetPipeEnable(
	BOOL bEngEnable, 
	E_DRVVIDEOIN_PIPE ePipeEnable
);    
	
void DrvVideoIn_GetPipeEnable(
	PBOOL pbEngEnable, 
	E_DRVVIDEOIN_PIPE* pePipeEnable
);
	
void DrvVideoIn_SetShadowRegister(void);
	
void DrvVideoIn_SetSensorPolarity(
	BOOL bVsync, 
	BOOL bHsync, 
	BOOL bPixelClk
);
	
void DrvVideoIn_GetSensorPolarity(
	PBOOL pbVsync, 
	PBOOL pbHsync, 
	PBOOL pbPixelClk
);
	
void DrvVideoIn_SetDataFormatAndOrder(
	E_DRVVIDEOIN_ORDER eInputOrder, 
	E_DRVVIDEOIN_IN_FORMAT eInputFormat, 
	E_DRVVIDEOIN_OUT_FORMAT eOutputFormat
);
	
void DrvVideoIn_GetDataFormatAndOrder(
	E_DRVVIDEOIN_ORDER* pe8InputOrder, 
	E_DRVVIDEOIN_IN_FORMAT* peInputFormat, 
	E_DRVVIDEOIN_OUT_FORMAT* peOutputFormat
	);
	
void DrvVideoIn_SetPlanarFormat(
	BOOL bIsYUV420
);	
BOOL DrvVideoIn_GetPlanarFormat(void);	
	
void DrvVideoIn_SetMotionDet(
	BOOL bEnable,
	BOOL bBlockSize,	
	BOOL bSaveMode
);
	
void DrvVideoIn_GetMotionDet(
	PBOOL pbEnable,
	PBOOL pbBlockSize,	
	PBOOL pbSaveMode
);
		
void DrvVideoIn_SetMotionDetEx(	
	UINT32 u32Threshold,
	UINT32 u32OutBuffer,
	UINT32 u32LumBuffer
);

void DrvVideoIn_GetMotionDetEx(	
	PUINT32 pu32Threshold,
	PUINT32 pu32OutBuffer,
	PUINT32 pu32LumBuffer
);

void DrvVideoIn_SetMotionDetFreq(UINT32 u32DetFreq);

	
void DrvVideoIn_SetOperationMode(
	BOOL bIsOneSutterMode
);



	
BOOL 
DrvVideoIn_GetOperationMode(void);
	
UINT32 
DrvVideoIn_GetProcessedDataCount(
	E_DRVVIDEOIN_PIPE ePipe
);
	
void DrvVideoIn_SetCropWinStartAddr(
	UINT32 u32VerticalStart, 
	UINT32 u32HorizontalStart
);
	
void DrvVideoIn_GetCropWinStartAddr(
	PUINT32 pu32VerticalStart, 
	PUINT32 pu32HorizontalStart
);

void DrvVideoIn_SetCropWinSize(
	UINT32 u32Height, 
	UINT32 u32Width
);

void DrvVideoIn_GetCropWinSize(
	PUINT32 pu32Height, 
	PUINT32 pu32Width
);

ERRCODE 
DrvVideoIn_SetVerticalScaleFactor(
	E_DRVVIDEOIN_PIPE ePipe,
	UINT8 u8Numerator, 
	UINT8 u8Denominator
);
	
ERRCODE 
DrvVideoIn_GetVerticalScaleFactor(
	E_DRVVIDEOIN_PIPE ePipe,
	PUINT8 pu8Numerator, 
	PUINT8 pu8Denominator
);
	
ERRCODE 
DrvVideoIn_SetHorizontalScaleFactor(
	E_DRVVIDEOIN_PIPE ePipe,
	UINT8 u8Numerator, 
	UINT8 u8Denominator
);
	
ERRCODE 
DrvVideoIn_GetHorizontalScaleFactor(
	E_DRVVIDEOIN_PIPE ePipe,
	PUINT8 pu8Numerator, 
	PUINT8 pu8Denominator
);
	
void DrvVideoIn_SetFrameRateScaleFactor(
	UINT8 u8Numerator, 
	UINT8 u8Denominator
);

void DrvVideoIn_GetFrameRateScaleFactor(
	PUINT8 pu8Numerator, 
	PUINT8 pu8Denominator
);
	
void DrvVideoIn_SetAddressMatch(
	UINT32 u32AddressMatch
);
	
void DrvVideoIn_GetAddressMatch(
	PUINT32 pu32AddressMatch
);
	
void DrvVideoIn_SetStride(
	UINT32 u32PacketStride, 
	UINT32 u32PlanarStride
);
	
void DrvVideoIn_GetStride(
	PUINT32 pu32PacketStride, 
	PUINT32 pu32PlanarStride
);
	
ERRCODE 
DrvVideoIn_SetBaseStartAddress(
	E_DRVVIDEOIN_PIPE ePipe,
	E_DRVVIDEOIN_BUFFER eBuf,
	UINT32 u32BaseStartAddr
);	
	

ERRCODE 
DrvVideoIn_GetBaseStartAddress(
	E_DRVVIDEOIN_PIPE ePipe,
	E_DRVVIDEOIN_BUFFER eBuf,
	PUINT32 pu32BaseStartAddr
);
	
void DrvVideoIn_SetInputType(
	UINT32 u32FieldEnable,	
	E_DRVVIDEOIN_TYPE bInputType, 
	BOOL bFieldSwap
);	
	
void DrvVideoIn_GetInputType(
	PUINT32 pu32FieldEnable, 
	E_DRVVIDEOIN_TYPE* pbInputType, 
	PBOOL pbFieldSwap
);	
			
void DrvVideoIn_SetFieldDetection(
	BOOL bDetPosition,
	BOOL bFieldDetMethod
);	
	
void DrvVideoIn_GetFieldDetection(
	PBOOL pbDetPosition, 
	PBOOL pbFieldDetMethod
);
	
void DrvVideoIn_SetPacketFrameBufferControl(
	BOOL bFrameSwitch,
	BOOL bFrameBufferSel
);
	
void DrvVideoIn_GetPacketFrameBufferControl(
	PBOOL pbFrameSwitch,
	PBOOL pbFrameBufferSel
);
	
void DrvVideoIn_SetInitFrame(void);
	
UINT32 
DrvVideoIn_GetSkipFrame(void);
	
UINT32 
DrvVideoIn_GetVersion(void);



#endif














