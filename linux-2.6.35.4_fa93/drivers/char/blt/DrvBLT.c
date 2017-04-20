//--------------------------------------------------------------
//
// Copyright (c) Nuvoton Technology Corp. All rights reserved.
//
//--------------------------------------------------------------

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
#include <mach/hardware.h>
//#include <asm/arch/w55fa93_reg.h>
#include <mach/w55fa93_reg.h>
//#include <asm/arch/DrvBLT.h>
#include <mach/DrvBLT.h>

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/err.h>

struct clk  *blt_clk;

unsigned int
DrvBLT_Open(void)
{
	unsigned long volatile flags;
    int ret;	
	    
	// 1.Check I/O pins. If I/O pins are used by other IPs, return error code.
	// 2.Enable IP¡¦s clock
	//outp32(REG_AHBCLK, inp32(REG_AHBCLK) | BLT_CKE);
	local_irq_save(flags);		
	//outp32(REG_AHBCLK, inp32(REG_AHBCLK) | BLT_CKE);	
    blt_clk = clk_get(NULL,"blt");    
    if (IS_ERR(blt_clk)) {
        printk("clk_get(..) error\n");
            ret = -ENODEV;
            return ret;
    }
    clk_enable(blt_clk);
        	
	local_irq_restore(flags);	
	// 3.Reset IP
	outp32(REG_AHBIPRST, inp32(REG_AHBIPRST) | BLTRST);
	outp32(REG_AHBIPRST, inp32(REG_AHBIPRST) & ~BLTRST);
	// 4.Configure IP according to inputted arguments.
	// 5.Enable IP I/O pins
	// 6.Return 0 to present success
	return E_SUCCESS;
}


void DrvBLT_Close(void)
{
	unsigned long volatile flags;
	    
	// 1.Disable IP I/O pins
	// 2.Disable IP¡¦s clock
	local_irq_save(flags);		
	//outp32(REG_AHBCLK, inp32(REG_AHBCLK) & ~BLT_CKE);
    clk_disable(blt_clk);	
	local_irq_restore(flags);	
}

//	Set	Transform Matrix | a c |
//				 | b d |
void DrvBLT_SetTransformMatrix(
	S_DRVBLT_MATRIX sMatrix		// [in] Transformation Matrix 
)
{
	outp32(REG_ELEMENTA, sMatrix.a);
	outp32(REG_ELEMENTB, sMatrix.b);
	outp32(REG_ELEMENTC, sMatrix.c);
	outp32(REG_ELEMENTD, sMatrix.d);	
}

//	Set Pixel Format of Source Image
int 
DrvBLT_SetSrcFormat(
	E_DRVBLT_BMPIXEL_FORMAT eSrcFmt	// [in] Source Image Format 
)
{
    if (eSrcFmt & ~0x3F)
        return E_FAIL;
        
	outp32(REG_SFMT, eSrcFmt);
	return 	E_SUCCESS;	
}

//	Set Pixel Format of Display 
int 
DrvBLT_SetDisplayFormat(
	E_DRVBLT_DISPLAY_FORMAT eDisplayFmt	// [in] Display Format 
)
{
    if (eDisplayFmt & ~0x07)
        return E_FAIL;
        
	outp32(REG_DFMT, eDisplayFmt);        
	return 	E_SUCCESS;	
}

//	Enable BLT Interrupt
void DrvBLT_EnableInt(void)
{
	outp32(REG_BLTINTCR, inp32(REG_BLTINTCR) | BLT_INTE);
}

//	Disable BLT Interrupt
void DrvBLT_DisableInt(void)
{
	outp32(REG_BLTINTCR, inp32(REG_BLTINTCR) & ~BLT_INTE);
}

//	Get Interrupt enable/disable status
bool 
DrvBLT_IsIntEnabled(void)
{
	return inp32(REG_BLTINTCR) & BLT_INTE;
}

//	Clear BLT Interrupt status
void DrvBLT_ClearInt(void)
{
	outp32(REG_BLTINTCR,  inp32(REG_BLTINTCR) | BLT_INTS);
}

//	Polling BLT Interrupt status
bool 
DrvBLT_PollInt(void)
{
	return inp32(REG_BLTINTCR) & BLT_INTS;
}


// Set A/R/G/B Color Multiplier
int 
DrvBLT_SetColorMultiplier(
	S_DRVBLT_ARGB16 sARGB16		// [in] ARGB Multiplier 
)
{
/*
	if ((sARGB16.i16Blue & 0xFFFF0000) || (sARGB16.i16Green & 0xFFFF0000) 
		|| (sARGB16.i16Red & 0xFFFF0000) || (sARGB16.i16Alpha & 0xFFFF0000))
        return E_DRVBLT_FALSE_INPUT;	
*/        
	outp32(REG_MLTB, (inp32(REG_MLTB)  & 0xFFFF0000 ) | (sARGB16.i16Blue  & 0x0FFFF));
	outp32(REG_MLTG, (inp32(REG_MLTG)  & 0xFFFF0000 ) | (sARGB16.i16Green & 0x0FFFF));
	outp32(REG_MLTR, (inp32(REG_MLTR)  & 0xFFFF0000 ) | (sARGB16.i16Red   & 0x0FFFF));	        
	outp32(REG_MLTA, (inp32(REG_MLTA)  & 0xFFFF0000 ) | (sARGB16.i16Alpha & 0x0FFFF));
	
	return 	E_SUCCESS;
}

// Set A/R/G/B Color Offset
int 
DrvBLT_SetColorOffset(
	S_DRVBLT_ARGB16 sARGB16		// [in] ARGB offset 
)
{
//	if ((sARGB16.i16Blue & 0xFFFF0000) || (sARGB16.i16Green & 0xFFFF0000) 
//		|| (sARGB16.i16Red & 0xFFFF0000) || (sARGB16.i16Alpha & 0xFFFF0000))
//        return E_DRVBLT_FALSE_INPUT;
        
	outp32(REG_MLTB, (inp32(REG_MLTB)  & 0x0000FFFF ) | ((unsigned int)(sARGB16.i16Blue  & 0x0FFFF) << 16));
	outp32(REG_MLTG, (inp32(REG_MLTG)  & 0x0000FFFF ) | ((unsigned int)(sARGB16.i16Green & 0x0FFFF) << 16));
	outp32(REG_MLTR, (inp32(REG_MLTR)  & 0x0000FFFF ) | ((unsigned int)(sARGB16.i16Red   & 0x0FFFF) << 16));	        
	outp32(REG_MLTA, (inp32(REG_MLTA)  & 0x0000FFFF ) | ((unsigned int)(sARGB16.i16Alpha & 0x0FFFF) << 16));
	        
	return 	E_SUCCESS;        
}

// Set Source Image
void DrvBLT_SetSrcImage(
	S_DRVBLT_SRC_IMAGE sSrcImage		// [in] Source Image Setting
)
{

	outp32(REG_SADDR,    sSrcImage.u32SrcImageAddr);	// Set Source Image Start Addr
	outp32(REG_SSTRIDE,  sSrcImage.i32Stride);		// Set Source Image Stride	
	outp32(REG_OFFSETSX, sSrcImage.i32XOffset);		// Set X offset into the source to start rendering from
	outp32(REG_OFFSETSY, sSrcImage.i32YOffset);		// Set Y offset into the source to start rendering from
	outp32(REG_SWIDTH,   sSrcImage.i16Width);		// Set width of source image
	outp32(REG_SHEIGHT,  sSrcImage.i16Height);		// Set height of source image
}

// Set Destination Frame Buffer
void DrvBLT_SetDestFrameBuf(
	S_DRVBLT_DEST_FB sFrameBuf		// [in] Frame Buffer Setting
)
{
	outp32(REG_DADDR,    sFrameBuf.u32FrameBufAddr);	// Set Frame Buffer Start Addr
	//outp32(REG_OFFSETDX, sFrameBuf.i32XOffset);		// X offset into the frame buffer in pixels
	//outp32(REG_OFFSETDY, sFrameBuf.i32YOffset);		// Y offset into the frame buffer in pixels
	outp32(REG_DSTRIDE,  sFrameBuf.i32Stride);		// Set Frame Buffer Stride
	outp32(REG_DWIDTH,   sFrameBuf.i16Width);		// Set width of Frame Buffer
	outp32(REG_DHEIGHT,  sFrameBuf.i16Height);		// Set height of Frame Buffer
}

//	Set ARGB color for Fill Operation
void DrvBLT_SetARGBFillColor(
	S_DRVBLT_ARGB8 sARGB8		// [in] ARGB value for fill operation
)
{
	unsigned int u32ARGB;
	
	u32ARGB = ((unsigned int)sARGB8.u8Alpha << 24) | ((unsigned int)sARGB8.u8Red << 16) 
			  | ((unsigned int)sARGB8.u8Green << 8) | sARGB8.u8Blue;  
	outp32(REG_FILLARGB, u32ARGB);			  
}

// 	Get BLT engine busy(rendering) status
bool 
DrvBLT_GetBusyStatus(void)
{
	return inp32(REG_SET2DA) & BLIT_EN;
}


void DrvBLT_SetFilAlpha(bool bEnable)
{
	outp32(REG_SET2DA, (inp32(REG_SET2DA) & ~FILL_BLEND) | (bEnable << 2));
}


// 	Set TransformFlag
void DrvBLT_SetTransFormFlag(
	unsigned int u32TransFlag		// [in] A combination of the enum E_DRVBLT_TRANSFORM_FLAG
)
{
	outp32(REG_SET2DA, (inp32(REG_SET2DA) & ~TRANS_FLAG) | (u32TransFlag << 4));
}

void DrvBLT_SetPaletteEndian(
	E_DRVBLT_PALETTE_ORDER eEndian	// [in] Palette Endian Type
)
{
	outp32(REG_SET2DA, (inp32(REG_SET2DA) & ~L_ENDIAN) | (eEndian << 1));
}

// Set the Color Palette used in BLT
// The format pointer by pu32ARGB are [31:24] -> Alpha, [23:16] -> Red, [15:8] -> Green, [7:0] -> Blue
void DrvBLT_SetColorPalette(
	unsigned int u32PaletteInx, 		// [in] Color Palette Start index
	unsigned int u32Num, 			// [in] Color Palette number to set
	S_DRVBLT_ARGB8* psARGB		// [in] pointer for Color palette from u32PaletteInx
)
{
	unsigned int u32PaletteStartInx;
	unsigned int u32ARGB;
	int i;
	
	u32PaletteStartInx = ((unsigned int)REG_PALETTE) + u32PaletteInx;
	for(i=0;i<u32Num;i++)
	{
		u32ARGB = ((unsigned int)(psARGB-> u8Alpha) << 24) | ((unsigned int)(psARGB-> u8Red) << 16) | 
				  ((unsigned int)(psARGB-> u8Green) << 8) | psARGB-> u8Blue; 
		outp32(u32PaletteStartInx+i*4, u32ARGB);
		psARGB++;
	}
}

void DrvBLT_SetFillOP(
	E_DRVBLT_FILLOP eOP			// [in] Enable/Disable FillOP
)
{
	outp32(REG_SET2DA,(inp32(REG_SET2DA) &  ~FILL_OP) | (eOP<<11));
}

void DrvBLT_SetFillStyle(
	E_DRVBLT_FILL_STYLE eStyle		// [in] Fill Style for Fill Operation
)
{
	outp32(REG_SET2DA,(inp32(REG_SET2DA) &  ~FILL_STYLE) | (eStyle <<8));
}


void DrvBLT_SetRevealAlpha(
	E_DRVBLT_REVEAL_ALPHA eAlpha		// [in] need / no need un-multiply alpha on source image
)
{
	outp32(REG_SET2DA,(inp32(REG_SET2DA) &  ~S_ALPHA) | (eAlpha << 3));
}


//	Trigger BLT engine to render image
void DrvBLT_Trigger(void)
{
	outp32(REG_SET2DA,inp32(REG_SET2DA) | BLIT_EN);
}

// BitBlt ISR
void DrvBLT_ISR(void)
{
	outp32(REG_BLTINTCR,  inp32(REG_BLTINTCR) | BLT_INTS);
}


void DrvBLT_SetRGB565TransparentColor(
	unsigned int u16RGB565	// [in] RGB565 Transparent Color
)
{
	outp32(REG_TRCOLOR, u16RGB565 & TR_RGB565);
    
}

void DrvBLT_SetRGB565TransparentCtl(
bool bEnable
)
{
    outp32(REG_SET2DA, (inp32(REG_SET2DA) & ~TRCOLOR_E) | (bEnable << 7));
}

