/****************************************************************
 *								*
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.	*
 *								*
 ****************************************************************/
 
#ifndef __DRVBLT_H__
#define __DRVBLT_H__

#include <linux/ioctl.h>

#ifdef	__cplusplus
extern "C"
{
#endif

#ifndef __DRVEDMA_H__

#define E_SUCCESS	0
#define E_FAIL		1

#define outp32(addr,value)  outl((unsigned long)value, (unsigned int)addr)
#define inp32(addr)         inl((unsigned int)addr)

/*
typedef enum {
#undef FALSE
	FALSE	= 0,
#undef TRUE
	TRUE	= 1
} bool;
*/
#endif // __DRVEDMA_H__

typedef enum
{
	eDRVBLT_DISABLE,	
	eDRVBLT_ENABLE	
} E_DRVBLT_FILLOP;

typedef enum
{
	eDRVBLT_EFFECTIVE,	
	eDRVBLT_NO_EFFECTIVE	
} E_DRVBLT_REVEAL_ALPHA;

typedef enum
{
	eDRVBLT_BIG_ENDIAN,	
	eDRVBLT_LITTLE_ENDIAN	
} E_DRVBLT_PALETTE_ORDER;

typedef enum {
	eDRVBLT_HASTRANSPARENCY = 1,	
	eDRVBLT_HASCOLORTRANSFORM = 2, 
	eDRVBLT_HASALPHAONLY = 4		
} E_DRVBLT_TRANSFORM_FLAG;

typedef enum {
	eDRVBLT_SRC_ARGB8888 = 1,	
	eDRVBLT_SRC_RGB565 = 2,
	eDRVBLT_SRC_1BPP = 4,
	eDRVBLT_SRC_2BPP = 8,
	eDRVBLT_SRC_4BPP = 16,
	eDRVBLT_SRC_8BPP = 32
} E_DRVBLT_BMPIXEL_FORMAT;

typedef enum {
	eDRVBLT_DEST_ARGB8888 = 1,	
	eDRVBLT_DEST_RGB565 = 2,
	eDRVBLT_DEST_RGB555 = 4
} E_DRVBLT_DISPLAY_FORMAT;

typedef enum {
	eDRVBLT_REPEAT_TEXTURE = 1,
	eDRVBLT_NOTSMOOTH = 2,
	eDRVBLT_CLIP = 4
} E_DRVBLT_FILL_STYLE;

typedef struct {
	int a;
	int b;
	int c;
	int d;
} S_DRVBLT_MATRIX;

typedef struct {
	unsigned char	u8Blue;
	unsigned char	u8Green;
	unsigned char	u8Red;
	unsigned char	u8Alpha;
} S_DRVBLT_ARGB8;

typedef struct {
	short	i16Blue;
	short	i16Green;
	short	i16Red;
	short	i16Alpha;
} S_DRVBLT_ARGB16;

typedef struct {
	short	i16Xmin;
	short	i16Xmax;
	short	i16Ymin;
	short	i16Ymax;
} S_DRVBLT_RECT;

typedef struct {
	S_DRVBLT_ARGB8*	pSARGB8;
	unsigned int	u32PaletteInx;
	unsigned int	u32Num;
	unsigned int	u32SrcImageAddr;
	int		i32Stride;
	int		i32XOffset;
	int		i32YOffset;
	short		i16Width;
	short		i16Height;
} S_DRVBLT_SRC_IMAGE;

typedef struct {
	unsigned int	u32FrameBufAddr;
	int		i32XOffset;
	int		i32YOffset;
	int		i32Stride;
	short		i16Width;
	short		i16Height;
} S_DRVBLT_DEST_FB;

typedef struct {
	int rotationDx, rotationDy;
	S_DRVBLT_MATRIX matrix;
	E_DRVBLT_BMPIXEL_FORMAT srcFormat;
	E_DRVBLT_DISPLAY_FORMAT destFormat;
	int flags;
	S_DRVBLT_ARGB16 colorMultiplier;
	S_DRVBLT_ARGB16 colorOffset;
	E_DRVBLT_FILL_STYLE	fillStyle;
	void *userData;
} S_DRVBLT_BLIT_TRANSFORMATION;

typedef struct {
	S_DRVBLT_RECT rect;
	S_DRVBLT_ARGB8 color;
//	char *frameBuffer;
	unsigned int    u32FrameBufAddr;
	int rowBytes;
	E_DRVBLT_DISPLAY_FORMAT format;
	int blend;
} S_DRVBLT_FILL_OP;

typedef struct {
	S_DRVBLT_SRC_IMAGE src;
	S_DRVBLT_DEST_FB dest;
	S_DRVBLT_BLIT_TRANSFORMATION *transformation;
} S_DRVBLT_BLIT_OP;

// for EDMA support
typedef struct {
	unsigned int	src_virt_addr;
	unsigned int	dest_phys_addr;
	unsigned int	dma_length;
} S_DRVBLT_EDMA_OP;

typedef enum
{
	eDRVBLT_RGB888 = 1,
	eDRVBLT_RGB555 = 2,
	eDRVBLT_RGB565 = 4,
	eDRVBLT_YCbCr422 = 8
} E_DRVBLT_COLOR_FORMAT;

typedef struct {
	E_DRVBLT_COLOR_FORMAT	src_fmt;
	E_DRVBLT_COLOR_FORMAT	dest_fmt;
	unsigned int		src_phys_addr;
	unsigned int		dest_phys_addr;
	unsigned int		dma_length;
} S_DRVBLT_EDMA_CST_OP;


#define BLT_TRIGGER			_IO ('v', 130)
#define BLT_SET_BLIT			_IOW('v', 131, S_DRVBLT_BLIT_OP)
#define BLT_GET_BLIT			_IOR('v', 132, S_DRVBLT_BLIT_OP)
#define BLT_SET_FILL			_IOW('v', 133, S_DRVBLT_FILL_OP)
#define BLT_GET_FILL			_IOR('v', 134, S_DRVBLT_FILL_OP)
#define BLT_FLUSH			_IO ('v', 135)
#define BLT_EDMA_REQUEST		_IO ('v', 136)
#define BLT_EDMA_FREE			_IO ('v', 137)
#define BLT_EDMA_SINGLE			_IOW('v', 138, S_DRVBLT_EDMA_OP)
#define BLT_EDMA_SG			_IOW('v', 139, S_DRVBLT_EDMA_OP)
#define BLT_EDMA_TRIGGER		_IO ('v', 140)
#define BLT_EDMA_WAIT			_IO ('v', 141)
#define BLT_EDMA_CST			_IOW('v', 142, S_DRVBLT_EDMA_CST_OP)
#define BLT_SET_RGB565_COLORKEY 	_IOW('v', 143, unsigned int)
#define BLT_ENABLE_RGB565_COLORCTL 	_IO ('v', 144)
#define BLT_DISABLE_RGB565_COLORCTL 	_IO ('v', 145)
#define BLT_SRCFMT_PREMULALPHA          _IO('v', 146)
#define BLT_SRCFMT_NONPREMULALPHA       _IO('v', 147)

// APIs declaration

unsigned int
DrvBLT_Open(void);

void DrvBLT_Close(void);

void DrvBLT_SetTransformMatrix(
	S_DRVBLT_MATRIX sMatrix		
);

int 
DrvBLT_SetSrcFormat(
	E_DRVBLT_BMPIXEL_FORMAT eSrcFmt	// [in] Source Image Format 
);

int 
DrvBLT_SetDisplayFormat(
	E_DRVBLT_DISPLAY_FORMAT eDisplayFmt	// [in] Display Format 
);

void DrvBLT_EnableInt(void);

void DrvBLT_DisableInt(void);

bool 
DrvBLT_IsIntEnabled(void);

void DrvBLT_ClearInt(void);

bool 
DrvBLT_PollInt(void);

int 
DrvBLT_SetColorMultiplier(
	S_DRVBLT_ARGB16 sARGB16		// [in] ARGB Multiplier 
);

int 
DrvBLT_SetColorOffset(
	S_DRVBLT_ARGB16 sARGB16		// [in] ARGB offset 
);

void DrvBLT_SetSrcImage(
	S_DRVBLT_SRC_IMAGE sSrcImage		// [in] Source Image Setting
);

void DrvBLT_SetDestFrameBuf(
	S_DRVBLT_DEST_FB sFrameBuf		// [in] Frame Buffer Setting
);

void DrvBLT_SetARGBFillColor(
	S_DRVBLT_ARGB8 sARGB8		// [in] ARGB value for fill operation
);

bool 
DrvBLT_GetBusyStatus(void);

void DrvBLT_SetFilAlpha(
	bool bEnable
);

void DrvBLT_SetTransFormFlag(
	unsigned int u32TransFlag		// [in] A combination of the enum E_DRVBLT_TRANSFORM_FLAG
);

void DrvBLT_SetPaletteEndian(
	E_DRVBLT_PALETTE_ORDER eEndian	// [in] Palette Endian Type
);

void DrvBLT_SetColorPalette(
	unsigned int u32PaletteInx, 		// [in] Color Palette Start index
	unsigned int u32Num, 			// [in] Color Palette number to set
	S_DRVBLT_ARGB8* psARGB		// [in] pointer for Color palette from u32PaletteInx
);

void DrvBLT_SetFillOP(
	E_DRVBLT_FILLOP eOP			// [in] Enable/Disable FillOP
);

void DrvBLT_SetFillStyle(
	E_DRVBLT_FILL_STYLE eStyle		// [in] Fill Style for Fill Operation
);

void DrvBLT_SetRevealAlpha(
	E_DRVBLT_REVEAL_ALPHA eAlpha		// [in] need / no need un-multiply alpha on source image
);

void DrvBLT_Trigger(void);


#ifdef	__cplusplus
}
#endif

#endif	// __DRVBLT_H__

