/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved. *
 *                                                              *
 ****************************************************************/
 
#ifndef __DRVVIDEOIN_H__
#define __DRVVIDEOIN_H__


#ifdef CONFIG_ARCH_W55FA93
	#define DBG_PRINTF(...)
#else
	#define DBG_PRINTF	sysprintf
#endif

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

#define E_VIDEOIN_PIN_UNAVAILABLE   			0xFFFF3001
#define E_VIDEOIN_WRITE_SENSOR_FAIL   			0xFFFF3002
#define E_VIDEOIN_READ_SENSOR_FAIL   			0xFFFF3003
#define E_VIDEOIN_INTERLACE_SCALE_FAIL  		0xFFFF3004
#define E_VIDEOIN_INVALID_INT   					0xFFFF3005
#define E_VIDEOIN_INVALID_BUF   				0xFFFF3006
#define E_VIDEOIN_INVALID_PIPE  					0xFFFF3007
#define E_VIDEOIN_INVALID_COLOR_MODE  		0xFFFF3008
#define E_VIDEOIN_WRONG_COLOR_PARAMETER  	0xFFFF3009

#define UINT8		__u8
#define UINT16		__u16
#define UINT32		__u32
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

#define outp32(addr, value)		writel(value, addr)
#define inp32(addr)				readl(addr)


#define VIDEO_PALETTE_YUV420P_MACRO		50		/* YUV 420 Planar Macro */

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
	eDrvVideoIn_3rd_SNR_CCIR601,
	eDrvVideoIn_2nd_TVD_CCIR656,
	eDrvVideoIn_3rd_TVD_CCIR656
}E_VIDEOIN_DEV_TYPE;

/* Interrupt type */
typedef enum
{
	eVIDEOIN_MDINT = 0x100000,
	eVIDEOIN_ADDRMINT = 0x80000,
	eVIDEOIN_MEINT = 0x20000,
	eVIDEOIN_VINT = 0x10000	
}E_VIDEOIN_INT_TYPE;


/* Pipe enable */
typedef enum
{
	eVIDEOIN_BOTH_PIPE_DISABLE = 0,
	eVIDEOIN_PLANAR = 1,
	eVIDEOIN_PACKET = 2,
	eVIDEOIN_BOTH_PIPE_ENABLE = 3	
}E_VIDEOIN_PIPE;

/* Base address */
typedef enum
{
	eVIDEOIN_BUF0 =0,
	eVIDEOIN_BUF1,	
	eVIDEOIN_BUF2
}E_VIDEOIN_BUFFER;

/* For DrvVideoIn_SetOperationMode */
#define VIDEOIN_CONTINUE   1   

/* Input Data Order For YCbCr */
typedef enum
{
	eVIDEOIN_IN_UYVY =0,
	eVIDEOIN_IN_YUYV,
	eVIDEOIN_IN_VYUY,		
	eVIDEOIN_IN_YVYU
}E_VIDEOIN_ORDER;


typedef enum
{
	eVIDEOIN_IN_YUV422 = 0,
	eVIDEOIN_IN_RGB565
}E_VIDEOIN_IN_FORMAT;                             
                                                                
typedef enum
{
	eVIDEOIN_OUT_YUV422 = 0,
	eVIDEOIN_OUT_ONLY_Y,
	eVIDEOIN_OUT_RGB555,		
	eVIDEOIN_OUT_RGB565
}E_VIDEOIN_OUT_FORMAT;	

typedef enum
{
	eVIDEOIN_PLANAR_YUV422 = 0,
	eVIDEOIN_PLANAR_YUV420,
	eVIDEOIN_MACRO_PLANAR_YUV420
}E_VIDEOIN_PLANAR_FORMAT;

typedef enum
{
	eVIDEOIN_TYPE_CCIR601 = 0,
	eVIDEOIN_TYPE_CCIR656
}E_VIDEOIN_TYPE;   

typedef enum
{
	eVIDEOIN_SNR_APLL = 2,
	eVIDEOIN_SNR_UPLL = 3
}E_VIDEOIN_SNR_SRC;  

typedef enum
{
	eVIDEOIN_CEF_NORMAL = 0,
	eVIDEOIN_CEF_SEPIA = 1,
	eVIDEOIN_CEF_NEGATIVE = 2,
	eVIDEOIN_CEF_POSTERIZE = 3
}E_VIDEOIN_CEF;   

/* Define data type (struct, union…) */

/* Define function */
void DrvVideoIn_Init(
	BOOL bIsEnableSnrClock,
	E_VIDEOIN_SNR_SRC eSnrSrc,	
	UINT32 u32SensorFreq,						//KHz unit
	E_VIDEOIN_DEV_TYPE eDevType
);

INT32
DrvVideoIn_Open(
	UINT32 u32EngFreqKHz, 
	UINT32 u32SensorFreq
);			
	
void DrvVideoIn_Close(void);
	
void DrvVideoIn_Reset(void);

/*
ERRCODE 
DrvVideoIn_InstallCallback(
	E_DRVVIDEOIN_INT_TYPE eIntType, 
	PFN_VIDEOIN_CALLBACK pfnCallback,
	PFN_DRVVIDEOIN_CALLBACK *pfnOldCallback
);	
*/
INT32 
DrvVideoIn_EnableInt(
	E_VIDEOIN_INT_TYPE eIntType
);

INT32
DrvVideoIn_DisableInt(
	E_VIDEOIN_INT_TYPE eIntType
);

BOOL
DrvVideoIn_IsIntEnabled(
	E_VIDEOIN_INT_TYPE eIntType
);

BOOL
DrvVideoIn_PollInt(
	void
);
	
void DrvVideoIn_SetPipeEnable(
	BOOL bEngEnable, 
	E_VIDEOIN_PIPE ePipeEnable
);    
	
void DrvVideoIn_GetPipeEnable(
	PBOOL pbEngEnable, 
	E_VIDEOIN_PIPE* pePipeEnable
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
	E_VIDEOIN_ORDER eInputOrder, 
	E_VIDEOIN_IN_FORMAT eInputFormat, 
	E_VIDEOIN_OUT_FORMAT eOutputFormat
);
	
void DrvVideoIn_GetDataFormatAndOrder(
	E_VIDEOIN_ORDER* pe8InputOrder, 
	E_VIDEOIN_IN_FORMAT* peInputFormat, 
	E_VIDEOIN_OUT_FORMAT* peOutputFormat
	);
	
void DrvVideoIn_SetPlanarFormat(
	//BOOL bIsYUV420
	E_VIDEOIN_PLANAR_FORMAT ePlanarFmt
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
void DrvVideoIn_GetMotionDetFreq(PUINT32 pu32DetFreq);
	
void DrvVideoIn_SetOperationMode(
	BOOL bIsOneSutterMode
);



	
BOOL 
DrvVideoIn_GetOperationMode(void);
	
UINT32 
DrvVideoIn_GetProcessedDataCount(
	E_VIDEOIN_PIPE ePipe
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

INT32 
DrvVideoIn_SetVerticalScaleFactor(
	E_VIDEOIN_PIPE ePipe,
	UINT16 u16Numerator, 
	UINT16 u16Denominator
);
	
INT32 
DrvVideoIn_GetVerticalScaleFactor(
	E_VIDEOIN_PIPE ePipe,
	PUINT16 pu16Numerator, 
	PUINT16 pu16Denominator
);
	
INT32 
DrvVideoIn_SetHorizontalScaleFactor(
	E_VIDEOIN_PIPE ePipe,
	UINT16 u16Numerator, 
	UINT16 u16Denominator
);
	
INT32 
DrvVideoIn_GetHorizontalScaleFactor(
	E_VIDEOIN_PIPE ePipe,
	PUINT16 pu16Numerator, 
	PUINT16 pu16Denominator
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
	
INT32
DrvVideoIn_SetBaseStartAddress(
	E_VIDEOIN_PIPE ePipe,
	E_VIDEOIN_BUFFER eBuf,
	UINT32 u32BaseStartAddr
);	
	

INT32 
DrvVideoIn_GetBaseStartAddress(
	E_VIDEOIN_PIPE ePipe,
	E_VIDEOIN_BUFFER eBuf,
	PUINT32 pu32BaseStartAddr
);
	
void DrvVideoIn_SetInputType(
	UINT32 u32FieldEnable,	
	E_VIDEOIN_TYPE bInputType, 
	BOOL bFieldSwap
);	
	
void DrvVideoIn_GetInputType(
	PUINT32 pu32FieldEnable, 
	E_VIDEOIN_TYPE* pbInputType, 
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


typedef struct
{
	void (*Init)(BOOL bIsEnableSnrClock, E_VIDEOIN_SNR_SRC eSnrSrc, UINT32 u32SensorFreqKHz, E_VIDEOIN_DEV_TYPE eDevType);
	INT32 (*Open)(UINT32 u32EngFreqKHz, UINT32 u32SensorFreqKHz);
	void (*Close)(void);
	void (*SetPipeEnable)(BOOL bEngEnable, E_VIDEOIN_PIPE ePipeEnable);
	void (*SetPlanarFormat)(E_VIDEOIN_PLANAR_FORMAT ePlanarFmt);
	void (*SetCropWinSize)(UINT32 u32height, UINT32 u32width);
	void (*SetCropWinStartAddr)(UINT32 u32VerticalStart, UINT32 u32HorizontalStart);
	INT32 (*PreviewPipeSize)(UINT16 u32height, UINT16 u32width);
	INT32 (*EncodePipeSize)(UINT16 u32height, UINT16 u32width);
	void (*SetStride)(UINT32 u16packetstride, UINT32 u32planarstride);  								
	void (*GetStride)(PUINT32 pu32PacketStride, PUINT32 pu32PlanarStride);
	INT32 (*EnableInt)(E_VIDEOIN_INT_TYPE eIntType);
	INT32 (*DisableInt)(E_VIDEOIN_INT_TYPE eIntType);
#ifndef CONFIG_ARCH_W55FA93
	INT32 (*InstallCallback)(E_VIDEOIN_INT_TYPE eIntType, PFN_VIDEOIN_CALLBACK pfnCallback, PFN_VIDEOIN_CALLBACK *pfnOldCallback);
#endif
	INT32 (*SetBaseStartAddress)(E_VIDEOIN_PIPE ePipe, E_VIDEOIN_BUFFER eBuf, UINT32 u32BaseStartAddr);					
	void (*SetOperationMode)(BOOL bIsOneSutterMode);
	BOOL (*GetOperationMode)(void);
	void (*SetPacketFrameBufferControl)(BOOL bFrameSwitch, BOOL bFrameBufferSel);
	void (*SetSensorPolarity)(BOOL bVsync, BOOL bHsync, BOOL bPixelClk);	
	
	//INT32 (*SetColorEffect)(E_VIDEOIN_CEF eColorMode);			
	//INT32 (*SetColorEffectParameter)(UINT8 u8YComp, UINT8 u8UComp, UINT8 u8VComp);
	void (*SetDataFormatAndOrder)(E_VIDEOIN_ORDER eInputOrder, E_VIDEOIN_IN_FORMAT eInputFormat, E_VIDEOIN_OUT_FORMAT eOutputFormat);
	void (*SetMotionDet)(BOOL bEnable, BOOL bBlockSize,	BOOL bSaveMode);
	void (*SetMotionDetEx)(UINT32 u32Threshold, UINT32 u32OutBuffer, UINT32 u32LumBuffer);
	void (*SetMotionDetFreq)(UINT32 u32DetFreq);					
	void (*SetInputType)(UINT32 u32FieldEnable, E_VIDEOIN_TYPE eInputType,	BOOL bFieldSwap);	
	void (*SetFieldDetection)(BOOL bDetPosition, BOOL bFieldDetMethod);
	void (*SetFrameRateScaleFactor)(UINT8 u8Numerator, UINT8 u8Denominator);
	void (*SetShadowRegister)(void);

	void (*SetInitFrame)(void);
	UINT32 (*GetSkipFrame)(void);
	BOOL (*IsIntEnabled)(E_VIDEOIN_INT_TYPE eIntTyp);
	void (*GetPacketFrameBufferControl)(PBOOL pbFrameSwitch, PBOOL pbFrameBufferSel);
	INT32 (*ClearInt)(E_VIDEOIN_INT_TYPE eIntType);

	void (*Reset)(void);
	BOOL (*PollInt)(void);
	void (*GetPipeEnable)(PBOOL pbEngEnable,E_VIDEOIN_PIPE* pePipeEnable);
	void (*GetSensorPolarity)(PBOOL pbVsync, PBOOL pbHsync, PBOOL pbPixelClk);
	void (*GetDataFormatAndOrder)(E_VIDEOIN_ORDER* peInputOrder, E_VIDEOIN_IN_FORMAT* peInputFormat,E_VIDEOIN_OUT_FORMAT* peOutputFormat);
	E_VIDEOIN_PLANAR_FORMAT (*GetPlanarFormat)(void);
	void (*GetMotionDet)(PBOOL pbEnable, PBOOL pbBlockSize,PBOOL pbSaveMode);
	void (*GetMotionDetEx)(PUINT32 pu32Threshold, PUINT32 pu32OutBuffer,PUINT32 pu32LumBuffer);
	UINT32 (*GetProcessedDataCount)(E_VIDEOIN_PIPE ePipe);
	void (*GetCropWinStartAddr)(PUINT32 pu32VerticalStart, PUINT32 pu32HorizontalStart);
	void (*GetMotionDetFreq)(PUINT32 pu32DetFreq);
	INT32 (*GetBaseStartAddress)(E_VIDEOIN_PIPE ePipe, E_VIDEOIN_BUFFER eBuf, PUINT32 pu32BaseStartAddr);
}VINDEV_T;
INT32 register_vin_device(UINT32 u32port, VINDEV_T* pVinDev);

#endif














