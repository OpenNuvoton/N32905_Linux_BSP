/* w55fa93jpeg.c
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

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/smp_lock.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/videodev.h>

#include <asm/io.h>
#include <asm/irq.h>
//#include <asm/hardware.h>
//#include <asm/arch/w55fa93_reg.h>
#include <mach/w55fa93_reg.h>
//#include <asm/arch/jpegcodec.h>
#include <mach/jpegcodec.h>
//#include <asm/arch/drvjpeg.h>
#include <mach/drvjpeg.h>

__u32 w55fa93_JPEG_Power(__u32 u32Index, __u32 u32Exp)
{
	if(u32Exp==0)
		return 1;
	else
	{	
		__u32 u32Idx;
		for(u32Idx=1; u32Idx<u32Exp; u32Idx = u32Idx+1)
		{
			u32Index = 2 * u32Index;	
		}	
	}	
	return u32Index;
}

extern unsigned int w55fa93_ahb_clock;
__s32 w55fa93_Jpeg_Open(void)
{
	// 1.Check I/O pins. If I/O pins are used by other IPs, return error code (check PINFUN)
	// 2.Enable IP's clock (check CLKMAN)
	//outp32(REG_AHBCLK, (inp32(REG_AHBCLK) | JPG_CKE));
	// 3.Reset IP (check RSTCON)
	outp32(REG_AHBIPRST, JPGRST);
	outp32(REG_AHBIPRST, 0);
	if(w55fa93_ahb_clock > 96000)
	{
		outp32(REG_CLKDIV4, (inp32(REG_CLKDIV4) & ~JPG_N) | 0x1000000);
		//printk("Set JPG Divider to 2\n");
	}
	// 4.Configure IP according to inputted arguments (check CLKSEL)
	// 5.Enable IP I/O pins
	// 6.Return 0 to present success

	return 0;
}

void w55fa93_Jpeg_Init(void)
{
	/* Set the default values of the JPEG registers */
	outp32(REG_JPRIQC, 0x000000F4);
	outp32(REG_JTHBQC, 0x000000F4);
	outp32(REG_JPRST, 0x00000004);
	outp32(REG_JTRST, 0x00000004);

	// Disable the Primary Up-scaling & Scaling-down
	outp32(REG_JPSCALU, 0x00000000);
	outp32(REG_JPSCALD, 0x00000000);

	// Reset JUPRAT and JSRCH
	outp32(REG_JUPRAT, 0x00000000);
	outp32(REG_JSRCH, 0x00000FFF);
	//-------------------------------------------

	/* Reset JPEG (JMCR [1]) */
//	outp32(JMCR,0x00000002);
	outp32( REG_JMCR, ((inp32(REG_JMCR) & ~(ENG_RST | JPG_EN)) | ENG_RST) );
//	outp32(JMCR,0x00000000);
	outp32( REG_JMCR, inp32(REG_JMCR) & ~(ENG_RST | JPG_EN) );
	outp32(REG_JMACR, 0x00400000);  //Can't use single buffer
}

__s32 w55fa93_Jpeg_SetEncodeMode(
	__u8 u8SourceFormat,
	__u16 u16JpegFormat
)
{
	__u8 u8Gray;
	switch(u16JpegFormat)
	{
		case DRVJPEG_ENC_PRIMARY_YUV420:
		case DRVJPEG_ENC_PRIMARY_YUV422:		
		case DRVJPEG_ENC_THUMBNAIL_YUV420:
		case DRVJPEG_ENC_THUMBNAIL_YUV422:
			outp32(REG_JMCR,u16JpegFormat);
			u8Gray = 0;
			break;
		case DRVJPEG_ENC_PRIMARY_GRAY:	
		case DRVJPEG_ENC_THUMBNAIL_GRAY:	
			if(u8SourceFormat == DRVJPEG_ENC_SOURCE_PACKET)
				return (-2);
			else
				outp32(REG_JMCR,0xA0);
				u8Gray = EY_ONLY;	
			break;
		default:
			return (-2);
	}	
	
	if(u8SourceFormat == DRVJPEG_ENC_SOURCE_PLANAR)
		outp32(REG_JITCR,PLANAR_ON | u8Gray);
	else if(u8SourceFormat == DRVJPEG_ENC_SOURCE_PACKET)
		outp32(REG_JITCR, inp32(REG_JITCR) & ~PLANAR_ON); 	
	else
		return (-2);
			
	return 0;
}

__s32 w55fa93_Jpeg_SetDecodeMode(
	__u32 u32OutputFormat
)
{	
	switch(u32OutputFormat)
	{
		case DRVJPEG_DEC_PRIMARY_PLANAR_YUV:
		case DRVJPEG_DEC_PRIMARY_PACKET_YUV422:
		case DRVJPEG_DEC_PRIMARY_PACKET_RGB555:
		case DRVJPEG_DEC_PRIMARY_PACKET_RGB565:
		case DRVJPEG_DEC_PRIMARY_PACKET_RGB888:
		case DRVJPEG_DEC_THUMBNAIL_PLANAR_YUV:
		case DRVJPEG_DEC_THUMBNAIL_PACKET_YUV422:
		case DRVJPEG_DEC_THUMBNAIL_PACKET_RGB555:
			outp32(REG_JITCR,u32OutputFormat);
			outp32(REG_JMCR,inp32(REG_JMCR) & ~ENC_DEC);
			break;
		default:
			return (-2);
	}
	return 0;
}

void w55fa93_Jpeg_Trigger(void)
{
    outp32(REG_JMCR, JPG_EN | inp32(REG_JMCR));    
	outp32(REG_JMCR, ~JPG_EN & inp32(REG_JMCR));
}


__s32 w55fa93_Jpeg_CalScalingFactor(
	__u8	u8Mode,				//Up / Down Scaling
	__u16	u16Height,			//Original Height
	__u16	u16Width,			//Original Width
	__u16	u16ScalingHeight,	//Scaled Height
	__u16	u16ScalingWidth,	//Scaled Width
	__u16*	pu16RatioH,			//Horizontal Ratio
	__u16*	pu16RatioW			//Vertical Ratio		
)
{
	__u32 w, h;
 	if(u8Mode == DRVJPEG_ENC_UPSCALE_MODE)
 	{
		if(u16ScalingHeight < u16Height || u16ScalingWidth < u16Width)
			return (-2); 	
		
		w = ((u16ScalingWidth - 1) * 1024) / (u16Width - 2);
		h = ((u16ScalingHeight - 1) * 1024) / (u16Height - 2);
	    *pu16RatioW = (__u32)w;
	    *pu16RatioH = (__u32)h;
 	}
 	else if(u8Mode == DRVJPEG_DEC_PACKET_DOWNSCALE_MODE) 
	{
		if(u16ScalingHeight > u16Height || u16ScalingWidth> u16Width)
			return (-2);
		
		
		w = (u16ScalingWidth * 8192) / (u16Width -1);
		h = (u16ScalingHeight * 8192) / (u16Height-1);
		
		if(w > 8192)
			w = 8192;
		if(h > 8192)
			h = 8192;


	    *pu16RatioW = (__u32)w;
	    *pu16RatioH = (__u32)h;	        	
	}
	else if(u8Mode == DRVJPEG_DEC_PLANAR_DOWNSCALE_MODE || u8Mode == DRVJPEG_ENC_PLANAR_DOWNSCALE_MODE) 
	{
		__u16 u16RatioW,u16RatioH;
		if(u16ScalingHeight > u16Height || u16ScalingWidth> u16Width)
			return (-2);			
		if(u16Height % u16ScalingHeight)	
			return (-2);
		if(u16Width % u16ScalingWidth)			
			return (-2);
		
		u16RatioW = u16Width / u16ScalingWidth;
		
		if(u16RatioW == 1)
			return (-2);
			
		u16RatioW = u16RatioW / 2 - 1;
		
		if(u16RatioW > 31)
			return (-2);		
		
		u16RatioH = u16Height / u16ScalingHeight - 1;
		
		if(u16RatioH > 63)
			return (-2);		
			
		*pu16RatioW = u16RatioW;
		*pu16RatioH = u16RatioH;					
	}
	else	     	
		return (-2);
	
	return 0;

}

__s32 w55fa93_Jpeg_SetScalingFactor(
	__u8	u8Mode,			//Up / Down Scaling
	__u16	u16FactorH,		//Vertical Scaling Factor
	__u16	u16FactorW		//Horizontal Scaling Factor
)
{
	if(u8Mode == DRVJPEG_ENC_UPSCALE_MODE)
	{
		_DRVJPEG_DEC_DISABLE_DOWNSCALING();
		_DRVJPEG_ENC_ENABLE_UPSCALING();		
	}		
	else if(u8Mode == DRVJPEG_DEC_PACKET_DOWNSCALE_MODE || u8Mode == DRVJPEG_DEC_PLANAR_DOWNSCALE_MODE|| u8Mode == DRVJPEG_ENC_PLANAR_DOWNSCALE_MODE)
	{
		_DRVJPEG_DEC_ENABLE_DOWNSCALING();
		_DRVJPEG_ENC_DISABLE_UPSCALING();
		_DRVJPEG_DEC_ENABLE_LOW_PASS_FILTER();
	}
	else
		return (-2);
		
	if(u8Mode == DRVJPEG_DEC_PLANAR_DOWNSCALE_MODE || u8Mode == DRVJPEG_ENC_PLANAR_DOWNSCALE_MODE)
	{
		outp32(REG_JPSCALD,  (inp32(REG_JPSCALD) & ~(PSCALX_F | PSCALY_F)) | ((u16FactorW & 0x1F) << 8) | (u16FactorH & 0x1F));
	}
	else
	{
		outp32(REG_JPSCALD, inp32(REG_JPSCALD) & ~(PSCALX_F | PSCALY_F));
		outp32(REG_JUPRAT, ((u16FactorH & 0x3FFF) << 16) | (u16FactorW & 0x3FFF));
	}
	return 0;
}

void w55fa93_Jpeg_GetDecodedDimension(
	__u16* pu16Height,			//Decode/Encode Height
	__u16*	pu16Width			//Decode/Encode Width
)
{	
	*pu16Width = inp32(REG_JDECWH) & 0x00001FFF;
	*pu16Height = inp32(REG_JDECWH) >> 16;   	
}


void w55fa93_Jpeg_SetDimension(
	__u16 u16Height,			//Decode/Encode Height
	__u16 u16Width				//Decode/Encode Width
)
{	
	outp32(REG_JPRIWH,((u16Height & 0xFFF)<<16) | (u16Width & 0xFFF) );
}

void w55fa93_Jpeg_GetDimension(
	__u16* pu16Height,			//Decoded Height from bit stream
	__u16*	pu16Width			//Decoded Width  from bit stream
)
{	
	*pu16Height = inp32(REG_JPRIWH) >> 16; 
	*pu16Width = inp32(REG_JPRIWH) & 0xFFF;
	  	
}

void w55fa93_Jpeg_GetScalingFactor(
	__u8	u8Mode,				//Up / Down Scaling	
	__u16* pu16FactorH,		//Vertical Scaling Factor
	__u16*	pu16FactorW			//Horizontal Scaling Factor
)
{	
	if(u8Mode == DRVJPEG_DEC_PLANAR_DOWNSCALE_MODE)
	{
		*pu16FactorH = inp32(REG_JPSCALD) & 0x3F;
		*pu16FactorW = (inp32(REG_JPSCALD) >> 8) & 0x1F;		
	}
	else
	{
		*pu16FactorH = (inp32(REG_JUPRAT) >> 16) & 0x3FFF;
		*pu16FactorW = inp32(REG_JUPRAT) & 0x3FFF;
	}
}
__s32 w55fa93_Jpeg_SetWindowDecode(	
	__u16	u16StartMCUX,	//Start X MCU
	__u16	u16StartMCUY,	//Horizontal Scaling Factor
	__u16	u16EndMCUX,		//Vertical Scaling Factor
	__u16	u16EndMCUY,		//Horizontal Scaling Factor	
	__u32	u32Stride		//Decode Output Stride
)
{	
    if(u16StartMCUX >= u16EndMCUX || u16StartMCUY >= u16EndMCUY)
    	return (-2);
    
	outp32(REG_JWINDEC0, u16StartMCUY << 16 | u16StartMCUX);      
    outp32(REG_JWINDEC1, u16EndMCUY << 16 | u16EndMCUX); 
    outp32(REG_JWINDEC2, u32Stride);     
	outp32(REG_JMCR, WIN_DEC);
	return 0;
}



__s32 w55fa93_Jpeg_AdjustQTAB(
	__u8 u8Mode,
	__u8 u8Qadjust,
	__u8 u8Qscaling
)
{
	__u32 u32Addr;
	if(u8Mode == DRVJPEG_ENC_PRIMARY)
		u32Addr = REG_JPRIQC;
	else if(u8Mode == DRVJPEG_ENC_THUMBNAIL)
		u32Addr = REG_JTHBQC;
	else
		return (-2);
	
	outp32(u32Addr,((u8Qadjust & 0xF) << 4 )| (u8Qscaling & 0xF));
	return 0;
}

__s32 w55fa93_Jpeg_SetQTAB(
	__u8* puQTable0,
	__u8* puQTable1,
	__u8* puQTable2,
	__u8 u8num
)
{
    __u32 u32value;
    __u32 u32TimeOut;
    int i;

	u32TimeOut = 0xFFFFFF;
    for(i = 0; i < 64; i=i+4)
    {
        while((inp32(REG_JMCR) & QT_BUSY) & u32TimeOut)
	        u32TimeOut--;
	        
	    if(!u32TimeOut)    
	    	return (-1);
	        
        u32value = puQTable0[i] | (puQTable0[i+1]<<8) | (puQTable0[i+2]<<16) | (puQTable0[i+3]<<24);
        outp32((REG_JQTAB0 + i),u32value); 
    }     
    
	u32TimeOut = 0xFFFFFF;    
    for(i = 0; i < 64; i=i+4)
    {
        while((inp32(REG_JMCR) & QT_BUSY) & u32TimeOut)
	        u32TimeOut--;    
	        
	    if(!u32TimeOut)    
	    	return (-1);
	    		            
        u32value = puQTable1[i] | (puQTable1[i+1]<<8) | (puQTable1[i+2]<<16) | (puQTable1[i+3]<<24);
        outp32((REG_JQTAB1 + i),u32value); 
    }          
 
    if (u8num <3)
        return 0;
        
	u32TimeOut = 0xFFFFFF;
	
    for(i = 0; i < 64; i=i+4)
    {
        while((inp32(REG_JMCR) & QT_BUSY) & u32TimeOut)
	        u32TimeOut--;      
	        
	    if(!u32TimeOut)    
	    	return (-1);	        
	    	
        u32value = puQTable2[i] | (puQTable2[i+1]<<8) | (puQTable2[i+2]<<16) | (puQTable2[i+3]<<24);
        outp32((REG_JQTAB2 + i),u32value); 
    }   
    
	u32TimeOut = 0xFFFFFF;    
    while((inp32(REG_JMCR) & QT_BUSY) & u32TimeOut)             
	        u32TimeOut--;     

    if(!u32TimeOut)    
    	return (-1);       
    else
    	return 0;
	        
}
