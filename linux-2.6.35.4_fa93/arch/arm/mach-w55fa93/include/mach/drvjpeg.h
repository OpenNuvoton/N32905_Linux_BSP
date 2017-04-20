/* drvjpeg.h
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 * <clyu2@nuvoton.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __DRVJPEG_H__
#define __DRVJPEG_H__

//Include header file
//#include <asm/arch/w55fa93_reg.h>
#include <mach/w55fa93_reg.h>

#define outp32(addr,value)	writel(value, addr)
#define inp32(addr)			readl(addr)


#define DRVJPEG_ENC_PRIMARY		0
#define DRVJPEG_ENC_THUMBNAIL	1

//Define for Interrupt Status
#define DRVJPEG_EER_INTS	ERR_INTS
#define DRVJPEG_DER_INTS	DER_INTS
#define DRVJPEG_DEC_INTS	DEC_INTS
#define DRVJPEG_ENC_INTS	ENC_INTS
#define DRVJPEG_DHE_INTS	DHE_INTS
#define DRVJPEG_IPW_INTS	IPW_INTS

//Define for Scaling
#define DRVJPEG_ENC_UPSCALE_MODE			0
#define DRVJPEG_DEC_PACKET_DOWNSCALE_MODE	1
#define DRVJPEG_DEC_PLANAR_DOWNSCALE_MODE	2
#define DRVJPEG_ENC_PLANAR_DOWNSCALE_MODE	3
	
//Define for Interrupt Enable
#define DRVJPEG_EER_INTE	ERR_INTE
#define DRVJPEG_DER_INTE	DER_INTE
#define DRVJPEG_DEC_INTE	DEC_INTE
#define DRVJPEG_ENC_INTE	ENC_INTE
#define DRVJPEG_DHE_INTE	DHE_INTE
#define DRVJPEG_IPW_INTE	IPW_INTE

//Define for Encode input Format
#define DRVJPEG_ENC_SOURCE_PLANAR	0
#define DRVJPEG_ENC_SOURCE_PACKET	1

//Version definition


//Export functions
#define _DRVJPEG_SET_YADDR(u32Address)				outp32(REG_JYADDR0, u32Address)				
#define _DRVJPEG_SET_UADDR(u32Address)				outp32(REG_JUADDR0, u32Address)
#define _DRVJPEG_SET_VADDR(u32Address)				outp32(REG_JVADDR0, u32Address)
#define _DRVJPEG_GET_YADDR()						inp32(REG_JYADDR0)				
#define _DRVJPEG_GET_UADDR()						inp32(REG_JUADDR0)
#define _DRVJPEG_GET_VADDR()						inp32(REG_JVADDR0)
#define _DRVJPEG_SET_YSTRIDE(u32Stride)				outp32(REG_JYSTRIDE, u32Stride)	
#define _DRVJPEG_SET_USTRIDE(u32Stride)				outp32(REG_JUSTRIDE, u32Stride)	
#define _DRVJPEG_SET_VSTRIDE(u32Stride)				outp32(REG_JVSTRIDE, u32Stride)	
#define _DRVJPEG_GET_YSTRIDE()						inp32(REG_JYSTRIDE)	
#define _DRVJPEG_GET_USTRIDE()						inp32(REG_JUSTRIDE)
#define _DRVJPEG_GET_VSTRIDE()						inp32(REG_JVSTRIDE)
#define _DRVJPEG_SET_BITSTREAM_ADDR(u32Address)		outp32(REG_JIOADDR0,u32Address)	
#define _DRVJPEG_GET_BITSTREAM_ADDR()				inp32(REG_JIOADDR0)
#define _DRVJPEG_SET_ENC_DEC(u8Mode)			outp32(REG_JMCR, (inp32(REG_JMCR) & ~ENC_DEC) | (u8Mode << 7));

//Encode
#define _DRVJPEG_GET_ENC_PRIMARY_BITSTREAM_SIZE()	inp32(REG_JPRI_SIZE)	
#define _DRVJPEG_GET_ENC_THUMBNAIL_BITSTREAM_SIZE()	inp32(REG_JTHB_SIZE)	
#define _DRVJPEG_SET_SOURCE_IMAGE_HEIGHT(u16Size)	outp32(REG_JSRCH,u16Size)	
#define _DRVJPEG_GET_SOURCE_IMAGE_HEIGHT()			inp32(REG_JSRCH)	
#define _DRVJPEG_ENC_ENABLE_UPSCALING()				outp32(REG_JPSCALU,inp32(REG_JPSCALU) | JPSCALU_8X)	
#define _DRVJPEG_ENC_DISABLE_UPSCALING()			outp32(REG_JPSCALU,inp32(REG_JPSCALU) & ~JPSCALU_8X)	
#define _DRVJPEG_ENC_ISENABLE_UPSCALING()			((inp32(REG_JPSCALU) & JPSCALU_8X) >> 6)
#define _DRVJPEG_ENC_SET_HEADER_CONTROL(u8Control)	outp32(REG_JHEADER, u8Control)
#define _DRVJPEG_ENC_GET_HEADER_CONTROL()			inp32(REG_JHEADER)
#define _DRVJPEG_ENC_SET_RDI_VALUE(u8Value)			outp32(REG_JPRST,u8Value)
#define _DRVJPEG_ENC_GET_RDI_VALUE()				inp32(REG_JPRST)

//Decode
#define _DRVJPEG_DEC_ENABLE_DOWNSCALING()			outp32(REG_JPSCALD,PSX_ON)	
#define _DRVJPEG_DEC_ISENABLE_DOWNSCALING()			((inp32(REG_JPSCALD) & PSX_ON) >> 15) 	
#define _DRVJPEG_DEC_DISABLE_DOWNSCALING()			outp32(REG_JPSCALD,~PSX_ON)	
#define _DRVJPEG_DEC_GET_DECODED_IMAGE_FORMAT()		(inp32(REG_JITCR) & DYUV_MODE)
#define _DRVJPEG_DEC_ENABLE_LOW_PASS_FILTER()		outp32(REG_JPSCALD,inp32(REG_JPSCALD) | PS_LPF_ON) 
#define _DRVJPEG_DEC_DISABLE_LOW_PASS_FILTER()		outp32(REG_JPSCALD,inp32(REG_JPSCALD) & ~PS_LPF_ON) 
#define _DRVJPEG_DEC_ISENABLE_LOW_PASS_FILTER()		((inp32(REG_JPSCALD) & PS_LPF_ON) >> 14)
#define _DRVJPEG_DEC_SET_INPUT_WAIT(u16Size)		outp32(REG_JMACR, 0x00400008 | ((u16Size & 0x3FF)<< 8) );
#define _DRVJPEG_DEC_RESUME_INPUT_WAIT()			outp32(REG_JMCR,inp32(REG_JMCR)|RESUMEI);
#define _DRVJPEG_DEC_DISABLE_WINDOWDECODE()			outp32(REG_JMCR, inp32(REG_JMCR) & ~(WIN_DEC));
//Interrupt
#define _DRVJPEG_INT_ENABLE(u32Intflag)				outp32(REG_JINTCR, u32Intflag)
#define _DRVJPEG_INT_DISABLE(u32Intflag)			outp32(REG_JINTCR, inp32 (REG_JINTCR) & ~(u32Intflag))
#define _DRVJPEG_GET_INT_STATUS()					(inp32 (REG_JINTCR) & 0xFF)
#define _DRVJPEG_CLEAR_INT(u32Intflag)				outp32(REG_JINTCR, (inp32 (REG_JINTCR) & ~0xFF) | u32Intflag)

//Define inline function

__s32 w55fa93_Jpeg_Open(void);

static __inline
void w55fa93_Jpeg_Close(void)
{
	outp32(REG_AHBCLK, (inp32(REG_AHBCLK) & ~JPG_CKE));
}
void w55fa93_Jpeg_Init(void);

void w55fa93_Jpeg_Trigger(void);

__s32
w55fa93_Jpeg_SetEncodeMode(
	__u8 u8SourceFormat,
	__u16 u16JpegFormat
);

__s32
w55fa93_Jpeg_SetDecodeMode(
	__u32 u32OutputFormat
);

__s32 w55fa93_Jpeg_CalScalingFactor(
	__u8	u8Mode,						//Up / Down Scaling
	__u16	u16Height,					//Original Height
	__u16	u16Width,					//Original Width
	__u16	u16ScalingHeight,			//Scaled Height
	__u16	u16ScalingWidth,			//Scaled Width
	__u16*	pu16RatioH,					//Horizontal Ratio
	__u16*	pu16RatioW					//Vertical Ratio		
);

__s32 w55fa93_Jpeg_SetScalingFactor(
	__u8	u8Mode,					//Up / Down Scaling
	__u16	u16FactorH,				//Vertical Scaling Factor
	__u16	u16FactorW				//Horizontal Scaling Factor
);

void w55fa93_Jpeg_GetDecodedDimension(
	__u16* pu16Height,			//Decode/Encode Height
	__u16*	pu16Width			//Decode/Encode Width
);

void w55fa93_Jpeg_SetDimension(
	__u16 u16Height,			//Decode/Encode Height
	__u16 u16Width				//Decode/Encode Width
);

void w55fa93_Jpeg_GetDimension(
	__u16* pu16Height,			//Decoded Height from bit stream
	__u16*	pu16Width			//Decoded Width  from bit stream
);

void w55fa93_Jpeg_GetScalingFactor(
	__u8	u8Mode,				//Up / Down Scaling
	__u16* pu16FactorH,		//Vertical Scaling Factor
	__u16*	pu16FactorW			//Horizontal Scaling Factor
);

__s32
w55fa93_Jpeg_SetWindowDecode(	
	__u16	u16StartMCUX,	//Start X MCU
	__u16	u16StartMCUY,	//Horizontal Scaling Factor
	__u16	u16EndMCUX,		//Vertical Scaling Factor
	__u16	u16EndMCUY,		//Horizontal Scaling Factor	
	__u32	u32Stride		//Decode Output Stride
);

__s32
w55fa93_Jpeg_AdjustQTAB(
	__u8 u8Mode,
	__u8 u8Qadjust,
	__u8 u8Qscaling
);

__s32
w55fa93_Jpeg_SetQTAB(
	__u8* puQTable0,
	__u8* puQTable1,
	__u8* puQTable2,
	__u8 u8num);

__u32 w55fa93_Jpeg_GetVersion(void);


#endif



