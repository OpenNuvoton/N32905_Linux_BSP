/* vin_ioctl.c
 *
 * Copyright (c) Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
//#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/smp_lock.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#if 0
#include <linux/videodev.h>
#else
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#endif 
#include <linux/jiffies.h>
#include <asm/cacheflush.h>

#include <asm/io.h>
#if 0
#include <asm/arch/w55fa95_reg.h>
#include <asm/arch/fb.h>
#include <asm/arch/w55fa95_fb.h>
#include <asm/arch/videoin.h>
#include <asm/arch/DrvVideoin.h>
#else
#include <mach/w55fa93_reg.h>
#include <mach/fb.h>
#include <mach/w55fa93_fb.h>
#include <mach/videoin.h>
//#include <mach/DrvVideoin.h>
#include "DrvVideoin.h"
#endif
#include <mach/videodev_ex.h>
#include "videoinpriv.h"

#define ERR_PRINTF		printk
//#define DBG_PRINTF		printk
/* Global variable */
#if 0
extern volatile videoIn_buf_t videoIn_encode_buf[];
extern volatile videoIn_buf_t videoIn_preview_buf[];
#endif

//extern planar_buf_t FramePlanarAddr[];
extern UINT32 u32PortAddr;
/* =====================================================
	Recording the packet physicak start address.
	It depend on the LCD dimension and the struct videowin 
	
 =====================================================*/
//extern unsigned int FramePackerAddr[];				 

//extern volatile int i32FrameNumber;
//extern volatile int i32ReadFrameNum;

extern VINDEV_T* pDevVin;

#ifdef _PREVIEW_
extern unsigned int bIsVideoInEnable; //for preview
#endif

//#define outp32(addr, value)		outl(value, addr)
//#define inp32(addr)			inl(addr)

static void schedule_delay(uint32_t u32Delayms)
{
	volatile unsigned long j=0;	
	j = jiffies + u32Delayms*HZ/1000; 	/* u32Delayms~ u32Delayms+10ms */			
	while( time_before(jiffies,j) )
		schedule();	
}
#if 0
/*================================================*/
struct video_capability
{
	char name[32];
	int type;
	int channels;		/* Num channels */
	int audios;			/* Num audio devices */
	int maxwidth;		/* Supported width */
	int maxheight;		/* And height */
	int minwidth;		/* Supported width */
	int minheight;		/* And height */
};
/*================================================*/
#endif 
static unsigned int vin_ioctl_get_capability(struct file *file,
									 	unsigned int cmd,
										void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

	DBG_PRINTF("videoin_ioctl VIDIOCGCAP\n");
	if (copy_to_user((void*)arg, (void *)&priv->videocap, sizeof(struct video_capability))) {
		ERR_PRINTF("copy_to_user error VIDIOCGCAP\n");
		return -EFAULT;
	}
	return 0;
}
#if 0
/*================================================*/
struct video_clip
{
	__s32	x,y;
	__s32	width, height;
	struct	video_clip *next;	/* For user use/driver use only */
};

struct video_window
{
	__u32	x,y;						/* Position of window */
	__u32	width,height;				/* Its size */
	__u32	chromakey;
	__u32	flags;
	struct	video_clip __user *clips;		/* Set only */
	int	clipcount;
#define VIDEO_WINDOW_INTERLACE	1
#define VIDEO_WINDOW_CHROMAKEY	16	/* Overlay by chromakey */
#define VIDEO_CLIP_BITMAP	-1
/* bitmap is 1024x625, a '1' bit represents a clipped pixel */
#define VIDEO_CLIPMAP_SIZE	(128 * 625)
};
/*================================================*/
#endif 
static unsigned int vin_ioctl_get_overlay_window(struct file *file,
										 	unsigned int cmd,
											void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

	DBG_PRINTF("videoin_ioctl VIDIOCGWIN\n");
	if (copy_to_user((void*)arg, (void *)&priv->videowin, sizeof(struct video_window))) {
		ERR_PRINTF("copy_to_user error VIDIOCGWIN\n");
		return -EFAULT;
	}
	return 0;
}
/* Specified Packet dimension */
static unsigned int vin_ioctl_set_overlay_window(struct file *file,
										 	unsigned int cmd,
											void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;
	struct video_window *videowin;
	UINT32 u32PacStride, u32PlaStride, i;

	DBG_PRINTF("videoin_ioctl VIDIOCSWIN\n");
	videowin = (struct video_window *)arg;

	memcpy(&priv->videowin, videowin, sizeof(struct video_window));

	DBG_PRINTF("ViewWindow Width =%d\n", priv->videowin.width);
	DBG_PRINTF("ViewWindow Height =%d\n", priv->videowin.height);
	DBG_PRINTF("ViewWindow PosX =%d\n", priv->videowin.x);
	DBG_PRINTF("ViewWindow PosY =%d\n",priv->videowin.y);
	if(priv->videowin.width>priv->i32packet_width)
	{
		printk("Specified width must less or equal priv->i32packet_width\n");
		return 0;
	}	
	if(priv->videowin.height>priv->i32packet_height)
	{
		printk("Specified height must less or equal priv->i32packet_height\n");
		return 0;
	}
	for(i=0;i<VIDEOIN_PREVIEW_BUFFER_NUM;i++)
	{/* Capture engine only supoort 2-byte/pixel */
		priv->pFramePackerAddr[i] = priv->videoIn_preview_buf[i].u32PhysAddr+ priv->i32packet_width*priv->videowin.y*2 + priv->videowin.x*2; 
		DBG_PRINTF("Packet buffer %d physical address =0x%x\n", i, priv->pFramePackerAddr[i]);
		
	}
	priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, priv->pFramePackerAddr[0]);
	priv->pDevVin->GetStride(&u32PacStride, &u32PlaStride);			/* Packet Stride depend on the LCM Width */ 
#if 0
  #ifdef CONFIG_SPECIFIED_PACKET_DIMENSION_DEV1
	priv->pDevVin->SetStride(priv->i32packet_width, u32PlaStride);	// Depend on kernel option specified width. (for two pipe streamming) 
  #else 
	priv->pDevVin->SetStride(priv->videowin.width, u32PlaStride);	// The stride is set by set view window cmd. 
  #endif
#else
	if(priv->i32IsEnablePreview)
		priv->pDevVin->SetStride(LCDWIDTH, u32PlaStride);				// preview on depends on the LCM's width 
	else
		priv->pDevVin->SetStride(priv->videowin.width, u32PlaStride);	// preview off depend on the VIDIOCSWIN cmd
#endif

	DBG_PRINTF("Packet Pallete = %d\n", priv->videopic.palette);
	//Default.
	if(priv->videopic.palette == VIDEO_PALETTE_RGB565)
		priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_UYVY, eVIDEOIN_IN_YUV422, eVIDEOIN_OUT_RGB565);
	if(priv->videopic.palette == VIDEO_PALETTE_YUV422)
		priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_UYVY, eVIDEOIN_IN_YUV422, eVIDEOIN_OUT_YUV422);
#ifdef CONFIG_SENSOR_OV7725_DEV1
	if(priv->videopic.palette == VIDEO_PALETTE_RGB565)
		priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_UYVY, eVIDEOIN_IN_YUV422, eVIDEOIN_OUT_RGB565);
	if(priv->videopic.palette == VIDEO_PALETTE_YUV422)
		priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_UYVY, eVIDEOIN_IN_YUV422, eVIDEOIN_OUT_YUV422);
#endif
#if defined(CONFIG_SENSOR_OV9660_DEV1) || defined(CONFIG_SENSOR_NT99140_DEV1)
	if(priv->videopic.palette == VIDEO_PALETTE_RGB565)
		priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_UYVY, eVIDEOIN_IN_YUV422, eVIDEOIN_OUT_RGB565);
	if(priv->videopic.palette == VIDEO_PALETTE_YUV422)
		priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_UYVY, eVIDEOIN_IN_YUV422, eVIDEOIN_OUT_YUV422);
#endif
#if defined(CONFIG_SENSOR_TW9912_DEV1) 
	if(priv->videopic.palette == VIDEO_PALETTE_RGB565)
		priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_VYUY, eVIDEOIN_IN_YUV422, eVIDEOIN_OUT_RGB565);
	if(priv->videopic.palette == VIDEO_PALETTE_YUV422)
		priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_VYUY, eVIDEOIN_IN_YUV422, eVIDEOIN_OUT_YUV422);
#endif
#if defined(CONFIG_SENSOR_TVP5150_DEV1) || defined(CONFIG_SENSOR_TW9900_DEV1) 
	if(priv->videopic.palette == VIDEO_PALETTE_RGB565)
		priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_VYUY, eVIDEOIN_IN_YUV422, eVIDEOIN_OUT_RGB565);
	if(priv->videopic.palette == VIDEO_PALETTE_YUV422)
		priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_VYUY, eVIDEOIN_IN_YUV422, eVIDEOIN_OUT_YUV422);
#endif

#if 0 //MOVE to Planar 
	if(priv->sensor_intf->change_image_resolution){
		if(priv->sensor_intf->change_image_resolution(priv, priv->videowin.width, priv->videowin.height)){
		}
		else{
		}
	}
#endif	
	#if 0
	priv->pDevVin->SetVerticalScaleFactor(eVIDEOIN_PACKET, priv->videowin.height, priv->sensor_intf->u16CurImgHeight);
	DBG_PRINTF("Packet V DDA: %d/%d\n", priv->videowin.height, priv->sensor_intf->u16CurImgHeight);
	priv->pDevVin->SetHorizontalScaleFactor(eVIDEOIN_PACKET,	priv->videowin.width, priv->sensor_intf->u16CurImgWidth);
	DBG_PRINTF("Packet H DDA: %d/%d\n", priv->videowin.width, priv->sensor_intf->u16CurImgWidth);
	#else

	priv->pDevVin->PreviewPipeSize(priv->videowin.height, priv->videowin.width);

	#endif
	
	//priv->pDevVin->SetSensorPolarity(FALSE, FALSE, TRUE); //It was be set at sensor init function
	//priv->pDevVin->SetPipeEnable(TRUE, eVIDEOIN_PACKET);
	//priv->pDevVin->SetShadowRegister();

	return 0;
}
#if 0
/*================================================*/
struct video_picture
{
	__u16	brightness;
	__u16	hue;
	__u16	colour;
	__u16	contrast;
	__u16	whiteness;	/* Black and white only */
	__u16	depth;		/* Capture depth */
	__u16   	palette;	/* Palette in use */
#define VIDEO_PALETTE_GREY	1	/* Linear greyscale */
#define VIDEO_PALETTE_HI240	2	/* High 240 cube (BT848) */
#define VIDEO_PALETTE_RGB565	3	/* 565 16 bit RGB */
#define VIDEO_PALETTE_RGB24	4	/* 24bit RGB */
#define VIDEO_PALETTE_RGB32	5	/* 32bit RGB */
#define VIDEO_PALETTE_RGB555	6	/* 555 15bit RGB */
#define VIDEO_PALETTE_YUV422	7	/* YUV422 capture */
#define VIDEO_PALETTE_YUYV	8
#define VIDEO_PALETTE_UYVY	9	/* The great thing about standards is ... */
#define VIDEO_PALETTE_YUV420	10
#define VIDEO_PALETTE_YUV411	11	/* YUV411 capture */
#define VIDEO_PALETTE_RAW	12	/* RAW capture (BT848) */
#define VIDEO_PALETTE_YUV422P	13	/* YUV 4:2:2 Planar */
#define VIDEO_PALETTE_YUV411P	14	/* YUV 4:1:1 Planar */
#define VIDEO_PALETTE_YUV420P	15	/* YUV 4:2:0 Planar */
#define VIDEO_PALETTE_YUV410P	16	/* YUV 4:1:0 Planar */
#define VIDEO_PALETTE_PLANAR	13	/* start of planar entries */
#define VIDEO_PALETTE_COMPONENT 7	/* start of component entries */
};
/*================================================*/
#endif 
static unsigned int vin_ioctl_get_picture_properties(struct file *file,
											 	unsigned int cmd,
												void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

	DBG_PRINTF("videoin_ioctl VIDIOCGPICT\n");
	
	if(priv->sensor_intf->read_write_brightness)
		priv->sensor_intf->read_write_brightness(priv, (INT32 *)&priv->videopic.brightness, TRUE);

	if(priv->sensor_intf->read_write_contrast)
		priv->sensor_intf->read_write_contrast(priv, (INT32 *)&priv->videopic.contrast, TRUE);

	if (copy_to_user((void*)arg, (void *)&priv->videopic, sizeof(struct video_picture))) {
		ERR_PRINTF("copy_to_user error VIDIOCGPICT\n");
		return -EFAULT;
	}
	return 0;
}

static unsigned int vin_ioctl_set_picture_properties(struct file *file,
											 	unsigned int cmd,
												void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;
	E_VIDEOIN_ORDER eInputOrder;
	E_VIDEOIN_IN_FORMAT eInputFormat;
	E_VIDEOIN_OUT_FORMAT eOutputFormat;	
#if 1	
	struct video_picture pict; 

	DBG_PRINTF("videoin_ioctl VIDIOCSPICT\n");

	memcpy((void*)&pict, (void *)arg, sizeof(struct video_picture));
	if( (pict.palette != VIDEO_PALETTE_YUV422)&&
		 (pict.palette != VIDEO_PALETTE_RGB565)&&
		 (pict.palette != VIDEO_PALETTE_RGB555)&&
		 (pict.palette != VIDEO_PALETTE_GREY)){	
		/* support picture property test*/
		printk("Only support packet format VIDEO_PALETTE_YUV42/VIDEO_PALETTE_RGB565/RGB555/Grey\n");
		return 0;
	}
#endif
	if (copy_from_user((void*)&priv->videopic, (void *)arg, sizeof(struct video_picture))) {
		ERR_PRINTF("copy_from_user error VIDIOCSPICT\n");
		return -EFAULT;
	}
	if( (priv->videopic.palette == VIDEO_PALETTE_YUV422)||
		 (priv->videopic.palette == VIDEO_PALETTE_RGB565)||
		 (priv->videopic.palette == VIDEO_PALETTE_RGB555)||
		 (priv->videopic.palette == VIDEO_PALETTE_GREY)){	
		priv->pDevVin->GetDataFormatAndOrder(&eInputOrder, &eInputFormat, &eOutputFormat);
		switch(priv->videopic.palette){
			case VIDEO_PALETTE_YUV422:
			priv->pDevVin->SetDataFormatAndOrder(eInputOrder, eInputFormat, eVIDEOIN_OUT_YUV422);
			break;
			case VIDEO_PALETTE_GREY:
			priv->pDevVin->SetDataFormatAndOrder(eInputOrder, eInputFormat, eVIDEOIN_OUT_ONLY_Y);
			break;
			case VIDEO_PALETTE_RGB555:
			priv->pDevVin->SetDataFormatAndOrder(eInputOrder, eInputFormat, eVIDEOIN_OUT_RGB555);
			break;
			case VIDEO_PALETTE_RGB565:
			priv->pDevVin->SetDataFormatAndOrder(eInputOrder, eInputFormat, eVIDEOIN_OUT_RGB565);
			break;
		}	
	}
	while(priv->i32FrameNumber != -1);
	if(priv->sensor_intf->read_write_brightness)
		priv->sensor_intf->read_write_brightness(priv, (INT32 *)&priv->videopic.brightness, FALSE);
//	else
//		return -EFAULT;

	if(priv->sensor_intf->read_write_contrast)
		priv->sensor_intf->read_write_contrast(priv, (INT32 *)&priv->videopic.contrast, FALSE);
//	else
//		return -EFAULT;

//	priv->pDevVin->I2cWriteOV(0x42, 0x55, priv->videopic.brightness);
//	priv->pDevVin->I2cWriteOV(0x42, 0x56, priv->videopic.contrast);
	return 0;
}

#if 0
/*================================================*/ 
VIDIOCCAPTURE 
STILL_IMAGE = -1    One shutter
VIDEO_START = 0  	Continuous mode 
VIDEO_STOP = 1   	Stop capture

example, 
if (ioctl(fd,VIDIOCCAPTURE,VIDEO_START) == -1)
{
	perror("ioctl (VIDIOCCAPTURE)");
	return -1;
}

/*================================================*/
#endif
static void vin_set_init(struct file *file)
{
	UINT32 i;
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

	/* Clear the ram out, no junk to the user */
	for(i=0;i<VIDEOIN_ENCODE_BUFFER_NUM;i++)
	{
		//memset((char *)priv->videoIn_encode_buf[i].u32VirtAddr, 0, VIDEOIN_ENCODE_BUFFER_SIZE);
		priv->videoIn_encode_buf[i].u8FrameState = FRAME_READY;
	} 
	priv->i32FrameNumber = -1;
	priv->i32ReadFrameNum = 0;
}

static unsigned int vin_ioctl_start_end_capture(struct file *file,
										 	unsigned int cmd,
											void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

	UINT32 u32Reg=0;

	DBG_PRINTF("videoin_ioctl VIDIOCCAPTURE\n");
	if (copy_from_user((void*)&priv->start_end_capture, (void *)arg, sizeof(int))) {
		ERR_PRINTF("copy_from_user error VIDIOCCAPTURE\n");
		return -EFAULT;
	}
	if(priv->start_end_capture==0)
	{//Start Capture
		DBG_PRINTF("Star Capture\n");
		priv->i32FrameNumber = 0;
		vin_set_init(file);
#if 0
		for(buf_num=0; buf_num< VIDEOIN_PREVIEW_BUFFER_NUM; buf_num=buf_num+1)
		{//Clear packet buffer, Black in YUV422 is Y=0x0, U=V=0x80 
			unsigned int* pu32Addr =  (unsigned int*)(priv->videoIn_preview_buf[buf_num].u32VirtAddr);							
			unsigned int i;
			for(i=0; i<priv->i32packet_width*priv->i32packet_height*16/8;i=i+4)
			{
				*pu32Addr++=0x80008000; //2 Pixel
			}					
		}
		flush_cache_all();	
#endif
		priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 0, priv->pFramePlanarAddr[0].u32PlaY);//Planar Y
		priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 1, priv->pFramePlanarAddr[0].u32PlaU);//Planar U
		priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 2, priv->pFramePlanarAddr[0].u32PlaV);//Planar V
		priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, priv->pFramePackerAddr[0]);
		u32Reg = inp32(REG_PACBA0+ priv->u32PortAddr);
		priv->i32StopCapture = 0;
		priv->pDevVin->SetPipeEnable(TRUE, eVIDEOIN_BOTH_PIPE_ENABLE);			
		//while(u32Reg==inp32(REG_PACBA0+ priv->u32PortAddr));		/* wait new frame in */
		schedule_delay(30);
	}
	else if(priv->start_end_capture==1)
	{//Stop Capture
		DBG_PRINTF("Stop Capture\n");
		vin_set_init(file);
		u32Reg = inp32(REG_PACBA0+ priv->u32PortAddr);
		priv->pDevVin->SetShadowRegister();
		priv->i32StopCapture = 1;
		//priv->pDevVin->SetOperationMode(TRUE);						//TRUE:One shutter mode
		//while(u32Reg==inp32(REG_PACBA0+ priv->u32PortAddr));		/* wait new frame in */
		schedule_delay(30);
	}
	
	return 0;
}

#if 0
/*================================================*/
struct video_mmap
{
	unsigned	int frame;		/* Frame (0 - n) for double buffer */
	int			height,width;
	unsigned	int format;		/* should be VIDEO_PALETTE_* */
};
/*================================================*/
#endif 
static unsigned int vin_ioctl_grab_frames(struct file *file,
								 	unsigned int cmd,
									void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;
	UINT32 u32PacStride, u32PlaStride;
	UINT32 u32Width, u32Height;
#if 1
	struct video_mmap vmmap;
	memcpy((void*)&vmmap, (void *)arg, sizeof(struct video_mmap));
	if( (vmmap.format != VIDEO_PALETTE_YUV422P)&&
		 (vmmap.format != VIDEO_PALETTE_YUV420P)){	
		/* support picture property test*/
		printk("Only support planar format VIDEO_PALETTE_YUV422P/VIDEO_PALETTE_YUV420P\n");
		return 0;
	}
#endif
	if (copy_from_user((void*)&priv->videommap, (void *)arg, sizeof(struct video_mmap))) {
		ERR_PRINTF("copy_from_user error VIDIOCMCAPTURE\n");
		return -EFAULT;
	}

	/* Checking supported format */
#ifdef CONFIG_PLANAR_YUV422_YUV420_DEV1
	if((priv->videommap.format!=VIDEO_PALETTE_YUV422P)&&\
		(priv->videommap.format!=VIDEO_PALETTE_YUV420P)){
		printk("Only support format VIDEO_PALETTE_YUV422P or VIDEO_PALETTE_YUV420P\n");
	}
#endif
#ifdef CONFIG_PLANAR_YUV420_DEV1
	if( (priv->videommap.format!=VIDEO_PALETTE_YUV420P) ){
		printk("Only support format VIDEO_PALETTE_YUV420P or VIDEO_PALETTE_YUV420P_MACRO\n");
	}
#endif
	
	DBG_PRINTF("frame = %d\n", priv->videommap.frame);
	DBG_PRINTF("(W H) = %d, %d\n", priv->videommap.width, priv->videommap.height);
	DBG_PRINTF("format = %d\n", priv->videommap.format);
	priv->videoIn_encode_buf[priv->videommap.frame].u8FrameState = FRAME_READY;
	/* SW MOVE */
	if(priv->videommap.width != priv->sensor_intf->u16CurImgWidth){//Encode dimension != Current sensor's width			
		if(priv->sensor_intf->change_image_resolution){
			if(priv->sensor_intf->change_image_resolution(priv, priv->videommap.width, priv->videommap.height)){
				printk("Change resolution successful\n");
				printk("videommap resolution %d * %d\n", priv->videommap.width, priv->videommap.height);
				printk("sensor resolution = %d * %d\n", priv->sensor_intf->u16CurImgWidth, priv->sensor_intf->u16CurImgHeight);					
			}
			else{
				printk("Change resolution successful\n");
				printk("videommap resolution %d * %d\n", priv->videommap.width, priv->videommap.height);
				printk("sensor resolution = %d * %d\n", priv->sensor_intf->u16CurImgWidth, priv->sensor_intf->u16CurImgHeight);			
			}
		}
	}
	DBG_PRINTF(" priv->i32FrameNumber = %d\n", priv->i32FrameNumber );
	if (priv->i32FrameNumber == -1)
	{	
		
		priv->pDevVin->GetStride(&u32PacStride, &u32PlaStride);			/* Packet Stride depend on the LCM Width */ 

		if(priv->videocrop.c.width < priv->videommap.width){
			priv->pDevVin->SetStride(u32PacStride, priv->videocrop.c.width);
			printk("Function %s - Line %d: Encode WxH = %dx%d", __FUNCTION__, __LINE__, priv->videocrop.c.width, priv->videocrop.c.height);	
			priv->pDevVin->EncodePipeSize(priv->videocrop.c.height, priv->videocrop.c.width);	
			u32Width = priv->videocrop.c.width;		
			u32Height = priv->videocrop.c.height;		
		}else{
			priv->pDevVin->SetStride(u32PacStride, priv->videommap.width); 
			printk("Function %s - Line %d: Encode WxH = %dx%d", __FUNCTION__, __LINE__, priv->videommap.width, priv->videommap.height);		
			priv->pDevVin->EncodePipeSize(priv->videommap.height, priv->videommap.width);
			u32Width = priv->videommap.width;
			u32Height = priv->videommap.height;	
		}	

		priv->i32FrameNumber = priv->videommap.frame;	
		{
			UINT32 u32StartAddr;
			if(priv->videommap.format ==VIDEO_PALETTE_YUV422P){
				DBG_PRINTF("Specified encode planar YUV422\n");				
				priv->pDevVin->SetPlanarFormat(FALSE);
				priv->pFramePlanarAddr[0].u32PlaY = priv->videoIn_encode_buf[0].u32PhysAddr;
				u32StartAddr = priv->pFramePlanarAddr[0].u32PlaY;
				u32StartAddr = u32StartAddr + u32Height * u32Width; //U for Planar YUV422
				priv->pFramePlanarAddr[0].u32PlaU = u32StartAddr;
				u32StartAddr = u32StartAddr + u32Height * u32Width/2; //V for Planar YUV422
				priv->pFramePlanarAddr[0].u32PlaV = u32StartAddr;
#if CONFIG_VIN_DEV1_ENCODE_BUF_NO >= 2				
				priv->pFramePlanarAddr[1].u32PlaY = priv->videoIn_encode_buf[1].u32PhysAddr;
				u32StartAddr = priv->pFramePlanarAddr[1].u32PlaY;
				u32StartAddr = u32StartAddr + u32Height * u32Width; //U for Planar YUV422
				priv->pFramePlanarAddr[1].u32PlaU = u32StartAddr;
				u32StartAddr = u32StartAddr + u32Height * u32Width/2; //V for Planar YUV422
				priv->pFramePlanarAddr[1].u32PlaV = u32StartAddr;
#endif
//#ifdef CONFIG_SUPPORT_EXTRA_BUFFER_DEV1
#if CONFIG_VIN_DEV1_ENCODE_BUF_NO == 3
				priv->pFramePlanarAddr[2].u32PlaY = priv->videoIn_encode_buf[2].u32PhysAddr;
				u32StartAddr = priv->pFramePlanarAddr[2].u32PlaY;
				u32StartAddr = u32StartAddr + u32Height * u32Width; //U for Planar YUV422
				priv->pFramePlanarAddr[2].u32PlaU = u32StartAddr;
				u32StartAddr = u32StartAddr + u32Height * u32Width/2; //V for Planar YUV422
				priv->pFramePlanarAddr[2].u32PlaV = u32StartAddr;
#endif
//				priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 0, priv->pFramePlanarAddr[0].u32PlaY);			
//				priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 1, priv->pFramePlanarAddr[0].u32PlaU);	
//				priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 2, priv->pFramePlanarAddr[0].u32PlaV);
			}
			else if(priv->videommap.format ==VIDEO_PALETTE_YUV420P){	
				DBG_PRINTF("Specified encode planar YUV420\n");
				priv->pDevVin->SetPlanarFormat(TRUE);
				priv->pFramePlanarAddr[0].u32PlaY = priv->videoIn_encode_buf[0].u32PhysAddr;
				u32StartAddr = priv->pFramePlanarAddr[0].u32PlaY;
				u32StartAddr = u32StartAddr + u32Height * u32Width; //U for Planar YUV420
				priv->pFramePlanarAddr[0].u32PlaU = u32StartAddr;
				u32StartAddr = u32StartAddr + u32Height * u32Width/4; //V for Planar YUV420
				priv->pFramePlanarAddr[0].u32PlaV = u32StartAddr;
#if CONFIG_VIN_DEV1_ENCODE_BUF_NO >= 2				
				priv->pFramePlanarAddr[1].u32PlaY = priv->videoIn_encode_buf[1].u32PhysAddr;
				u32StartAddr = priv->pFramePlanarAddr[1].u32PlaY;
				u32StartAddr = u32StartAddr + u32Height * u32Width; //U for Planar YUV420
				priv->pFramePlanarAddr[1].u32PlaU = u32StartAddr;
				u32StartAddr = u32StartAddr + u32Height * u32Width/4; //V for Planar YUV420
				priv->pFramePlanarAddr[1].u32PlaV = u32StartAddr;
#endif
//#ifdef CONFIG_SUPPORT_EXTRA_BUFFER_DEV1
#if CONFIG_VIN_DEV1_ENCODE_BUF_NO == 3
				priv->pFramePlanarAddr[2].u32PlaY = priv->videoIn_encode_buf[2].u32PhysAddr;
				u32StartAddr = priv->pFramePlanarAddr[2].u32PlaY;
				u32StartAddr = u32StartAddr + u32Height * u32Width; //U for Planar YUV420
				priv->pFramePlanarAddr[2].u32PlaU = u32StartAddr;
				u32StartAddr = u32StartAddr + u32Height * u32Width/4; //V for Planar YUV420
				priv->pFramePlanarAddr[2].u32PlaV = u32StartAddr;
#endif
//				priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 0, priv->pFramePlanarAddr[0].u32PlaY);			
//				priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 1, priv->pFramePlanarAddr[0].u32PlaU);	
//				priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PLANAR, 2, priv->pFramePlanarAddr[0].u32PlaV);		
			}else{
				printk("Specified encode format NOT support\n");
			}
			DBG_PRINTF("Planar Buf 0 Y phy = 0x%x\n", priv->pFramePlanarAddr[0].u32PlaY);
			DBG_PRINTF("Planar Buf 0 U phy = 0x%x\n", priv->pFramePlanarAddr[0].u32PlaU);
			DBG_PRINTF("Planar Buf 0 V phy = 0x%x\n", priv->pFramePlanarAddr[0].u32PlaV);
#if CONFIG_VIN_DEV1_ENCODE_BUF_NO >= 2
			DBG_PRINTF("Planar Buf 1 Y phy = 0x%x\n", priv->pFramePlanarAddr[1].u32PlaY);
			DBG_PRINTF("Planar Buf 1 U phy = 0x%x\n", priv->pFramePlanarAddr[1].u32PlaU);
			DBG_PRINTF("Planar Buf 1 V phy = 0x%x\n", priv->pFramePlanarAddr[1].u32PlaV);
#endif
//#ifdef CONFIG_SUPPORT_EXTRA_BUFFER_DEV1
#if CONFIG_VIN_DEV1_ENCODE_BUF_NO == 3
			DBG_PRINTF("Planar Buf 2 Y phy = 0x%x\n", priv->pFramePlanarAddr[2].u32PlaY);
			DBG_PRINTF("Planar Buf 2 U phy = 0x%x\n", priv->pFramePlanarAddr[2].u32PlaU);
			DBG_PRINTF("Planar Buf 2 V phy = 0x%x\n", priv->pFramePlanarAddr[2].u32PlaV);	
#endif
		}	
		priv->pDevVin->SetShadowRegister();
		priv->videoIn_encode_buf[priv->i32FrameNumber].u8FrameState=FRAME_GRABBING;		
	}
	else
	{
		DBG_PRINTF("NOT capture %d\n",priv->videommap.frame);
	}
	
	/* Only one encode buffer, to capture new frmae by VIDIOCMCAPTURE cmd */
	/* if encode buffer > 1, capture new frame in interrupt handler */
#if  CONFIG_VIN_DEV1_ENCODE_BUF_NO == 1
	priv->pDevVin->SetOperationMode(TRUE);
#endif


	return 0;
}
#if 0
/*================================================*/
struct video_mbuf
{
	int	size;		/* Total memory to map */
	int	frames;		/* Frames */
	int	offsets[VIDEO_MAX_FRAME];
};
/*================================================*/
#endif 
static unsigned int vin_ioctl_get_map_buffer_info(struct file *file,
										 	unsigned int cmd,
											void *arg)
{
	int ret = 0;//0:mmap, -1:read
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;
	if(!ret)
	{
		DBG_PRINTF("videoin_ioctl VIDIOCGMBUF set window\n");
		vin_ioctl_set_overlay_window(file, cmd , &(priv->videowin));
	}
	DBG_PRINTF("videoin_ioctl VIDIOCGMBUF\n");
	if (copy_to_user((void*)arg, (void *)&priv->videombuf, sizeof(struct video_mbuf))) {
		ERR_PRINTF("copy_to_user error VIDIOCGMBUF\n");
		ret = -EFAULT;
	}
	return ret;
}

static unsigned int vin_ioctl_sync(struct file *file,
						 	unsigned int cmd,
							void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

	if (copy_from_user((void*)&priv->grab_sync, (void *)arg, sizeof(int))) {
		ERR_PRINTF("copy_from_user error VIDIOCSYNC\n");
		return -EFAULT;
	}
	DBG_PRINTF("videoin_ioctl VIDIOCSYNC %d \n", priv->grab_sync);
	if(priv->videoIn_encode_buf[priv->grab_sync].u8FrameState != FRAME_CAPTUREDONE)
	{
		DBG_PRINTF("NOT sync frame %d FrameState %d\n", priv->grab_sync, priv->videoIn_encode_buf[priv->grab_sync].u8FrameState);
		return -EAGAIN;
	}
	else
	{
		DBG_PRINTF("sync frame %d\n",priv->grab_sync);
	}
	
	if(priv->i32IRLedMode == 2){     //2:auto
		if(priv->sensor_intf->low_lux_detect){
			priv->i32IsIRLedOn = priv->sensor_intf->low_lux_detect(priv);
		}
	}
	return 0;
}

static unsigned int vin_ioctl_get_capture_time(struct file *file,
									 	unsigned int cmd,
										void *arg)
{
	struct v4l2_buffer v4l2buff;
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

	DBG_PRINTF("videoin_ioctl VIDIOCGCAPTIME\n");
	if (copy_from_user((void*)&v4l2buff, (void *)arg, sizeof(struct v4l2_buffer))) {
		ERR_PRINTF("copy_from_user error VIDIOCGCAPTIME\n");
		return -EFAULT;
	}
	if (v4l2buff.index >= VIDEOIN_ENCODE_BUFFER_NUM)
	{
		ERR_PRINTF("VIDIOCGCAPTIME index %d error\n", v4l2buff.index);
		return -EINVAL;
	}

	v4l2buff.timestamp = priv->videoIn_encode_buf[v4l2buff.index].tvCapTime;
	v4l2buff.m.userptr = priv->videoIn_encode_buf[v4l2buff.index].u32PhysAddr;	
	DBG_PRINTF("sec %d, usec %d\n", (UINT32)v4l2buff.timestamp.tv_sec, (UINT32)v4l2buff.timestamp.tv_usec);
	if (copy_to_user((void *)arg, (void*)&v4l2buff, sizeof(struct v4l2_buffer))) {
		ERR_PRINTF("copy_to_user error VIDIOCGCAPTIME\n");
		return -EFAULT;
	}
	return 0;
}


static unsigned int vin_ioctl_get_sys_up_time(struct file *file,
									 	unsigned int cmd,
										void *arg)
{
	struct timeval sSysTime;	
	DBG_PRINTF("videoin_ioctl VIDIOCGSYSUPTIME\n");
	jiffies_to_timeval((jiffies - INITIAL_JIFFIES), &sSysTime);
	if (copy_to_user((void *)arg, (void*)&sSysTime, sizeof(struct timeval))) {
		ERR_PRINTF("copy_to_user error VIDIOCGSYSBOOTTIME\n");
		return -EFAULT;
	}
	return 0;
}

static unsigned int vin_ioctl_set_brightness(struct file *file,
									 	unsigned int cmd,
										void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

	DBG_PRINTF("videoin_ioctl VIDIOCSBRIGHTNESS\n");
	if (copy_from_user((void*)&priv->i32Brightness, (void *)arg, sizeof(INT32))) {
		ERR_PRINTF("copy_from_user error VIDIOCSBRIGHTNESS\n");
		return -EFAULT;
	}
	if (priv->i32Brightness >= 128 || priv->i32Brightness < -128)
	{
		ERR_PRINTF("VIDIOCSBRIGHTNESS brightness 0x%x error\n", priv->i32Brightness);
		return -EINVAL;
	}
	while(priv->i32FrameNumber != -1);

	if(priv->sensor_intf->read_write_brightness)
		priv->sensor_intf->read_write_brightness(priv, &priv->i32Brightness, FALSE);
	else
		return -EFAULT;


//	priv->pDevVin->I2cWriteOV(0x42, 0x55, priv->brightness);

	return 0;
}

static unsigned int vin_ioctl_get_brightness(struct file *file,
									 	unsigned int cmd,
										void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;
	INT32 brightness;

	DBG_PRINTF("videoin_ioctl VIDIOCGBRIGHTNESS\n");

	if(priv->sensor_intf->read_write_brightness)
		priv->sensor_intf->read_write_brightness(priv, &brightness, TRUE);
	else
		return -EFAULT;


//	brightness = ((INT8)pDevVin->I2cReadOV(0x42, 0x55)&0xff);

	if (brightness >= 128 || brightness < -128)
	{
		ERR_PRINTF("VIDIOCGBRIGHTNESS brightness 0x%x error\n", brightness);
		return -EINVAL;
	}

	if (copy_to_user((void *)arg, (void*)&brightness, sizeof(INT32))) {
		ERR_PRINTF("copy_to_user error VIDIOCGBRIGHTNESS\n");
		return -EFAULT;
	}
	return 0;
}

static unsigned int vin_ioctl_set_contrast(struct file *file,
								 	unsigned int cmd,
									void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

	DBG_PRINTF("videoin_ioctl VIDIOCSCONTRAST\n");
	if (copy_from_user((void*)&priv->i32Contrast, (void *)arg, sizeof(INT32))) {
		ERR_PRINTF("copy_from_user error VIDIOCSCONTRAST\n");
		return -EFAULT;
	}
	if (priv->i32Contrast >= 256)
	{
		ERR_PRINTF("VIDIOCSCONTRAST contrast 0x%x error\n", priv->i32Contrast);
		return -EINVAL;
	}
	while(priv->i32FrameNumber != -1);

	if(priv->sensor_intf->read_write_contrast)
		priv->sensor_intf->read_write_contrast(priv, &priv->i32Contrast, FALSE);
	else
		return -EFAULT;


//	pDevVin->I2cWriteOV(0x42, 0x56, priv->contrast);

	return 0;
}

static unsigned int vin_ioctl_get_contrast(struct file *file,
								 	unsigned int cmd,
									void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;
	INT32 contrast;

	DBG_PRINTF("videoin_ioctl VIDIOCGCONTRAST\n");
	
	if(priv->sensor_intf->read_write_contrast)
		priv->sensor_intf->read_write_contrast(priv, &contrast, TRUE);
	else
		return -EFAULT;


//	contrast = ((UINT8)pDevVin->I2cReadOV(0x42, 0x56)&0xff);

	if (contrast >= 256)
	{
		ERR_PRINTF("VIDIOCGCONTRAST contrast 0x%x error\n", contrast);
		return -EINVAL;
	}

	if (copy_to_user((void *)arg, (void*)&contrast, sizeof(INT32))) {
		ERR_PRINTF("copy_to_user error VIDIOCGCONTRAST\n");
		return -EFAULT;
	}
	return 0;
}

static unsigned int vin_ioctl_set_sharpness(struct file *file,
									 	unsigned int cmd,
										void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

	DBG_PRINTF("videoin_ioctl VIDIOCSSHARPNESS\n");
	if (copy_from_user((void*)&priv->i32Sharpness, (void *)arg, sizeof(INT32))) {
		ERR_PRINTF("copy_from_user error VIDIOCSSHARPNESS\n");
		return -EFAULT;
	}
	if (priv->i32Sharpness >= 32)
	{
		ERR_PRINTF("VIDIOCSSHARPNESS sharpness 0x%x error\n", priv->i32Sharpness);
		return -EINVAL;
	}
	while(priv->i32FrameNumber != -1);

	if(priv->sensor_intf->read_write_sharpness)
		priv->sensor_intf->read_write_sharpness(priv, &priv->i32Sharpness, FALSE);
	else
		return -EFAULT;


//	pDevVin->I2cWriteOV(0x42, 0x3f, priv->sharpness);

	return 0;
}

static unsigned int vin_ioctl_get_sharpness(struct file *file,
									 	unsigned int cmd,
										void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;
	INT32 sharpness;

	DBG_PRINTF("videoin_ioctl VIDIOCGSHARPNESS\n");

	if(priv->sensor_intf->read_write_sharpness)
		priv->sensor_intf->read_write_sharpness(priv, &sharpness, TRUE);
	else
		return -EFAULT;


//	sharpness = ((UINT8)pDevVin->I2cReadOV(0x42, 0x3f)&0x1f);

	if (sharpness >= 32)
	{
		ERR_PRINTF("VIDIOCGSHARPNESS sharpness 0x%x error\n", sharpness);
		return -EINVAL;
	}

	if (copy_to_user((void *)arg, (void*)&sharpness, sizeof(INT32))) {
		ERR_PRINTF("copy_to_user error VIDIOCGSHARPNESS\n");
		return -EFAULT;
	}
	return 0;
}

static unsigned int vin_ioctl_set_white_balance(struct file *file,
										 	unsigned int cmd,
											void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

	DBG_PRINTF("videoin_ioctl VIDIOCSWHITEBALANCE\n");
	if (copy_from_user((void*)&priv->i32WhiteBalance, (void *)arg, sizeof(INT32))) {
		ERR_PRINTF("copy_from_user error VIDIOCSWHITEBALANCE\n");
		return -EFAULT;
	}
	if (priv->i32WhiteBalance >= 256)
	{
		ERR_PRINTF("VIDIOCSWHITEBALANCE white_balance 0x%x error\n", priv->i32WhiteBalance);
		return -EINVAL;
	}
	while(priv->i32FrameNumber != -1);

	if(priv->sensor_intf->read_write_white_balance)
		priv->sensor_intf->read_write_white_balance(priv, &priv->i32WhiteBalance, FALSE);
	else
		return -EFAULT;

//	pDevVin->I2cWriteOV(0x42, 0x6f, priv->white_balance);

	return 0;
}

static unsigned int vin_ioctl_get_white_balance(struct file *file,
										 	unsigned int cmd,
											void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;
	INT32 white_balance;

	DBG_PRINTF("videoin_ioctl VIDIOCGWHITEBALANCE\n");

	if(priv->sensor_intf->read_write_white_balance)
		priv->sensor_intf->read_write_white_balance(priv, &white_balance, TRUE);
	else
		return -EFAULT;


//	white_balance = ((UINT8)pDevVin->I2cReadOV(0x42, 0x6f)&0xff);

	if (white_balance >= 256)
	{
		ERR_PRINTF("VIDIOCGWHITEBALANCE white_balance 0x%x error\n", white_balance);
		return -EINVAL;
	}

	if (copy_to_user((void *)arg, (void*)&white_balance, sizeof(INT32))) {
		ERR_PRINTF("copy_to_user error VIDIOCGWHITEBALANCE\n");
		return -EFAULT;
	}
	return 0;
}

static unsigned int vin_ioctl_set_noise_reduction(struct file *file,
										 	unsigned int cmd,
											void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

	DBG_PRINTF("videoin_ioctl VIDIOCSNOISEREDUCTION\n");
	if (copy_from_user((void*)&priv->i32NoiseReduction, (void *)arg, sizeof(INT32))) {
		ERR_PRINTF("copy_from_user error VIDIOCSNOISEREDUCTION\n");
		return -EFAULT;
	}
	if (priv->i32NoiseReduction >= 256)
	{
		ERR_PRINTF("VIDIOCSNOISEREDUCTION noise_reduction 0x%x error\n", priv->i32NoiseReduction);
		return -EINVAL;
	}
	while(priv->i32FrameNumber != -1);

	if(priv->sensor_intf->read_write_noise_reduction)
		priv->sensor_intf->read_write_noise_reduction(priv, &priv->i32NoiseReduction, FALSE);
	else
		return -EFAULT;


//	pDevVin->I2cWriteOV(0x42, 0x4c, priv->noise_reduction);

	return 0;
}

static unsigned int vin_ioctl_get_noise_reduction(struct file *file,
										 	unsigned int cmd,
											void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;
	INT32 noise_reduction;

	DBG_PRINTF("videoin_ioctl VIDIOCGNOISEREDUCTION\n");

	if(priv->sensor_intf->read_write_noise_reduction)
		priv->sensor_intf->read_write_noise_reduction(priv, &noise_reduction, TRUE);
	else
		return -EFAULT;


//	noise_reduction = ((UINT8)pDevVin->I2cReadOV(0x42, 0x4c)&0xff);

	if (noise_reduction >= 256)
	{
		ERR_PRINTF("VIDIOCGNOISEREDUCTION noise_reduction 0x%x error\n", noise_reduction);
		return -EINVAL;
	}

	if (copy_to_user((void *)arg, (void*)&noise_reduction, sizeof(INT32))) {
		ERR_PRINTF("copy_to_user error VIDIOCGNOISEREDUCTION\n");
		return -EFAULT;
	}
	return 0;
}

static unsigned int vin_ioctl_set_color_saturation(struct file *file,
										 	unsigned int cmd,
											void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

	DBG_PRINTF("videoin_ioctl VIDIOCSCOLORSATURATION\n");
	if (copy_from_user((void*)&priv->i32ColorSaturation, (void *)arg, sizeof(INT32))) {
		ERR_PRINTF("copy_from_user error VIDIOCSCOLORSATURATION\n");
		return -EFAULT;
	}
	if (priv->i32ColorSaturation >= 16)
	{
		ERR_PRINTF("VIDIOCSCOLORSATURATION color_saturation 0x%x error\n", priv->i32ColorSaturation);
		return -EINVAL;
	}
	while(priv->i32FrameNumber != -1);

	if(priv->sensor_intf->read_write_color_saturation)
		priv->sensor_intf->read_write_color_saturation(priv, &priv->i32ColorSaturation, FALSE);
	else
		return -EFAULT;


//	pDevVin->I2cWriteOV(0x42, 0xc9, priv->color_saturation);

	return 0;
}

static unsigned int vin_ioctl_get_color_saturation(struct file *file,
										 	unsigned int cmd,
											void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;
	INT32 color_saturation;

	DBG_PRINTF("videoin_ioctl VIDIOCGCOLORSATURATION\n");

	if(priv->sensor_intf->read_write_color_saturation)
		priv->sensor_intf->read_write_color_saturation(priv, &color_saturation, TRUE);
	else
		return -EFAULT;


//	color_saturation = ((UINT8)pDevVin->I2cReadOV(0x42, 0xc9) & 0xf);

	if (color_saturation >= 16)
	{
		ERR_PRINTF("VIDIOCGCOLORSATURATION color_saturation 0x%x error\n", color_saturation);
		return -EINVAL;
	}

	if (copy_to_user((void *)arg, (void*)&color_saturation, sizeof(INT32))) {
		ERR_PRINTF("copy_to_user error VIDIOCGCOLORSATURATION\n");
		return -EFAULT;
	}
	return 0;
}

static unsigned int vin_ioctl_set_flicker_freq(struct file *file,
									 	unsigned int cmd,
										void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;
	INT32 i32FlickerFreq;

	DBG_PRINTF("videoin_ioctl VIDIOCSFLICKERFREQ\n");
	if (copy_from_user((void*)&i32FlickerFreq, (void *)arg, sizeof(INT32))) {
		ERR_PRINTF("copy_from_user error VIDIOCSFLICKERFREQ\n");
		return -EFAULT;
	}
	if((i32FlickerFreq != 50) && (i32FlickerFreq != 60)){
		ERR_PRINTF("VIDIOCSFLICKERFREQ flicker freq 0x%d error\n", i32FlickerFreq);
		return -EINVAL;
	}

	if(priv->sensor_intf->set_flicker_freq)
		priv->sensor_intf->set_flicker_freq(priv, i32FlickerFreq);
	else
		return -EFAULT;

	return 0;
}


static unsigned int vin_ioctl_set_IR_led(struct file *file,
								 	unsigned int cmd,
									void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;
	INT32 i32IRLedMode;

	DBG_PRINTF("videoin_ioctl VIDIOCSIRLED\n");
	if (copy_from_user((void*)&i32IRLedMode, (void *)arg, sizeof(INT32))) {
		ERR_PRINTF("copy_from_user error VIDIOCSIRLED\n");
		return -EFAULT;
	}
	if(i32IRLedMode > 2){
		ERR_PRINTF("VIDIOCSIRLED IR led mode 0x%d error\n", i32IRLedMode);
		return -EINVAL;
	}

	if(priv->sensor_intf->control_IR_led == NULL){
		ERR_PRINTF("VIDIOCSIRLED IR led mode 0x%d error\n", i32IRLedMode);
		return -EINVAL;
	}

	if(i32IRLedMode == 0){
		priv->sensor_intf->control_IR_led(priv, FALSE);
		priv->i32IsIRLedOn = FALSE;
	}
	else if(i32IRLedMode == 1){
		priv->sensor_intf->control_IR_led(priv, TRUE);
		priv->i32IsIRLedOn = TRUE;
	}

	priv->i32IRLedMode = i32IRLedMode;
	return 0;

}


static unsigned int vin_ioctl_get_IR_led(struct file *file,
								 	unsigned int cmd,
									void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

	if (copy_to_user((void *)arg, (void*)&(priv->i32IsIRLedOn), sizeof(INT32))) {
		ERR_PRINTF("copy_to_user error VIDIOCGIRLEDONOFF\n");
		return -EFAULT;
	}
	return 0;
}


/*
	ioctl VIDIOC_CROPCAP
		
	Name:	
		VIDIOC_CROPCAP??Information about the video cropping and scaling abilities
	Synopsis:	
		int ioctl(int fd, int request, struct v4l2_cropcap *argp);
	Arguments:
		fd			File descriptor returned by open().
		request			VIDIOC_CROPCAP
		argp
	Description
		Applications use this function to query the cropping limits, the pixel aspect of images and to
		calculate scale factors. They set the type field of a v4l2_cropcap structure to the respective buffer
		(stream) type and call the VIDIOC_CROPCAP ioctl with a pointer to this structure. Drivers fill the rest
		of the structure. The results are constant except when switching the video standard. Remember this
		switch can occur implicit when switching the video input or output.
	-------------------------------------------------------------------------------------------		
	struct v4l2_cropcap
		struct v4l2_cropcap {
				enum v4l2_buf_type type;
				struct v4l2_rect bounds;
				struct v4l2_rect defrect;
				struct v4l2_fract pixelaspect;
			};

		enum v4l2_buf_type type 		Type of the data stream, set by the application.Only these types are valid here:
							V4L2_BUF_TYPE_VIDEO_CAPTURE,
							V4L2_BUF_TYPE_VIDEO_OUTPUT,
							V4L2_BUF_TYPE_VIDEO_OVERLAY, and custom(driver defined) types with code
							V4L2_BUF_TYPE_PRIVATE and higher.

		struct v4l2_rect bounds			The cropping rectangle cannot exceed these limits. 
							Width and height are defined in pixels,

		struct v4l2_rect defrect		Each capture device has a default source rectangle,	

		﻿struct v4l2_fract pixelaspect 	 	This is the pixel aspect (y / x) when no scaling is applied, the ratio of the actual sampling
							frequency and the frequency required to get square pixels.
							**** We us the structure for zooming step. 
							**** It means the zooming step will be (16, 12)	for VGA sensor. The aspect ratio is 4:3   						
							
	-------------------------------------------------------------------------------------------				
		struct v4l2_rect {
			__s32 left;
			__s32 top;
			__s32 width;
			__s32 height;
		};
		
		struct v4l2_fract {
			__u32 numerator;
			__u32  denominator;
		};
		

*/
static unsigned int vin_ioctl_get_cropping_capability(struct file *file,
											 	unsigned int cmd,
												void *arg)
{
	int ret = 0;
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

	printk("videoin_ioctl VIDIOC_CROPCAP\n");
		
	if (copy_to_user((void*)arg, (void *)&priv->videocropcap, sizeof(struct v4l2_cropcap))) {
		ERR_PRINTF("copy_to_user error VIDIOC_CROPCAP\n");
		ret = -EFAULT;
	}
	return ret;												
}

/*
	Applications can use the following API to select an area in the video signal, query the default area
	and the hardware limits. 
	Despite their name, the VIDIOC_CROPCAP, VIDIOC_G_CROP and VIDIOC_S_CROP ioctls apply to input as well as output devices.	
	
	ioctl VIDIOC_G_CROP, VIDIOC_S_CROP

	Name:
		VIDIOC_G_CROP, VIDIOC_S_CROP??Get or set the current cropping rectangle
	Synopsis:
		int ioctl(int fd, int request, struct v4l2_crop *argp);
		int ioctl(int fd, int request, const struct v4l2_crop *argp);

	Arguments
		fd:		File descriptor returned by open().
		request:	VIDIOC_G_CROP, VIDIOC_S_CROP
		argp:		

	--------------------------------------------------------------------------------------------			
	struct v4l2_crop
		enum v4l2_buf_type type 	Type of the data stream, set by the application.
						Only these types are valid here:
						V4L2_BUF_TYPE_VIDEO_CAPTURE,
						V4L2_BUF_TYPE_VIDEO_OUTPUT,
						V4L2_BUF_TYPE_VIDEO_OVERLAY, and custom (driver defined) types with code
						V4L2_BUF_TYPE_PRIVATE and higher.

		struct v4l2_rect c 		Cropping rectangle. The same co-ordinate system as for struct v4l2_cropcap bounds is used.
	--------------------------------------------------------------------------------------------		
	struct v4l2_rect
		__s32 left 			Horizontal offset of the top, left corner of the rectangle, in pixels.
		__s32 top 			Vertical offset of the top, left corner of the rectangle, in pixels. 
						Offsets increase to the right and down.
		__s32 width 			Width of the rectangle, in pixels.
		__s32 height 			Height of the rectangle, in pixels. 
						Width and height cannot be negative, the fields are signed for hysterical reasons.
*/
#if 0
static void clearFrameBuf(UINT32* ptr)
{
	INT32 	i32Index;
	for(i32Index =0; i32Index < priv->i32packet_width*priv->i32packet_height/2; i32Index=i32Index+1)
	{
		*ptr++ = 0x80108010;				
	}	
}
#endif

static unsigned int vin_ioctl_set_cropping_window(struct file *file,
										 	unsigned int cmd,
											void *arg)
{
	int ret = 0;
	UINT32 u32PacStride, u32PlaStride;
	volatile UINT32 u32StartAddr;
	UINT32 u32Width, u32Height;
	//struct v4l2_crop videocrop;	
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;


	printk("videoin_ioctl VIDIOC_S_CROP\n");
	if (copy_from_user((void*)&(priv->videocrop), (void *)arg, sizeof(struct v4l2_crop))) {
		ERR_PRINTF("copy_from_user error VIDIOC_S_CROP\n");
		return -EFAULT;
	}
	priv->i32Zooming = TRUE;	 			/* Disable Update register */

	//priv->pDevVin->SetPipeEnable(TRUE, eVIDEOIN_BOTH_PIPE_DISABLE);
	
	printk("Cropping Size (W, H) = %d * %d\n", priv->videocrop.c.width, priv->videocrop.c.height);
	printk("Cropping Pos (X, Y) = %d * %d\n", priv->videocrop.c.left, priv->videocrop.c.top);

	priv->pDevVin->SetCropWinStartAddr(priv->videocrop.c.top,	// Y
									priv->videocrop.c.left);	// X
	priv->pDevVin->SetCropWinSize(priv->videocrop.c.height,		//UINT16 u16Height, 
								priv->videocrop.c.width);	//UINT16 u16Width;		
	
#if 0
	//Packet downscale
	u32GCD = vinGCD(priv->videowin.height,				// Preview height
			priv->videocrop.c.height); 			// Crop height
	DrvVideoIn_SetVerticalScaleFactor(eVIDEOIN_PACKET,		
						priv->videowin.height/u32GCD,
						priv->videocrop.c.height/u32GCD);
	u32GCD = vinGCD(priv->videowin.width, 
			priv->videocrop.c.width);
	DrvVideoIn_SetHorizontalScaleFactor(eVIDEOIN_PACKET,		
						priv->videowin.width/u32GCD,	// Preview width
						priv->videocrop.c.width/u32GCD);	//Crop width
	DrvVideoIn_GetStride(&u32PacStride, &u32PlaStride);
#else
	priv->pDevVin->PreviewPipeSize(priv->videowin.height, priv->videowin.width);
	priv->pDevVin->GetStride(&u32PacStride, &u32PlaStride);
#endif
	
	
	if(priv->videocrop.c.width < priv->videommap.width)
	{//Cropping width < encode size, start up JPEG upscale, capturing cropping size.
		printk("Set planar stride = %d\n", priv->videocrop.c.width);			
		priv->pDevVin->SetStride(u32PacStride, priv->videocrop.c.width);
		printk("Function %s - Line %d: Encode WxH = %dx%d", __FUNCTION__, __LINE__, priv->videocrop.c.width, priv->videocrop.c.height);	
		priv->pDevVin->EncodePipeSize(priv->videocrop.c.height, priv->videocrop.c.width);
		u32Width = priv->videocrop.c.width;		
		u32Height = priv->videocrop.c.height;
	}
	else
	{//Cropping width >= encode width, by capture downscale to target image
		priv->pDevVin->SetStride(u32PacStride, priv->videommap.width); 
		printk("Function %s - Line %d: Encode WxH = %dx%d", __FUNCTION__, __LINE__, priv->videommap.width, priv->videommap.height);		
		priv->pDevVin->EncodePipeSize(priv->videommap.height, priv->videommap.width);
		u32Width = priv->videommap.width;
		u32Height = priv->videommap.height;
	}
	if(priv->videommap.format ==VIDEO_PALETTE_YUV422P){
//#ifdef CONFIG_SUPPORT_EXTRA_BUFFER_DEV1
#if CONFIG_VIN_DEV1_ENCODE_BUF_NO == 3
		UINT32 ibuf, ibufno= 3;
#elif CONFIG_VIN_DEV1_ENCODE_BUF_NO == 2
		UINT32 ibuf, ibufno= 2;
#elif CONFIG_VIN_DEV1_ENCODE_BUF_NO == 1
		UINT32 ibuf, ibufno= 1;
#endif

		DBG_PRINTF("Specified encode planar YUV422\n");		
		for(ibuf=0; ibuf<ibufno; ibuf=ibuf+1){
			priv->pFramePlanarAddr[ibuf].u32PlaY = priv->videoIn_encode_buf[ibuf].u32PhysAddr;
			u32StartAddr = priv->pFramePlanarAddr[ibuf].u32PlaY;
			u32StartAddr = u32StartAddr + u32Height * u32Width; //U for Planar YUV422
			priv->pFramePlanarAddr[ibuf].u32PlaU = u32StartAddr;
			u32StartAddr = u32StartAddr + u32Height * u32Width/2; //V for Planar YUV422
			priv->pFramePlanarAddr[ibuf].u32PlaV = u32StartAddr;
		}
	}
	else if(priv->videommap.format ==VIDEO_PALETTE_YUV420P){	
//#ifdef CONFIG_SUPPORT_EXTRA_BUFFER_DEV1
#if CONFIG_VIN_DEV1_ENCODE_BUF_NO == 3
		UINT32 ibuf, ibufno= 3;
#elif CONFIG_VIN_DEV1_ENCODE_BUF_NO == 2
		UINT32 ibuf, ibufno= 2;
#elif CONFIG_VIN_DEV1_ENCODE_BUF_NO == 1
		UINT32 ibuf, ibufno= 1;
#endif
		printk("Specified encode planar YUV420\n");
		for(ibuf=0; ibuf<ibufno; ibuf=ibuf+1){
			priv->pFramePlanarAddr[ibuf].u32PlaY = priv->videoIn_encode_buf[ibuf].u32PhysAddr;
			u32StartAddr = priv->pFramePlanarAddr[ibuf].u32PlaY;
			u32StartAddr = u32StartAddr + priv->videocrop.c.height * priv->videocrop.c.width; //U for Planar YUV420
			priv->pFramePlanarAddr[ibuf].u32PlaU = u32StartAddr;
			u32StartAddr = u32StartAddr + priv->videocrop.c.height * priv->videocrop.c.width/4; //V for Planar YUV420
			priv->pFramePlanarAddr[ibuf].u32PlaV = u32StartAddr;
		}	
	}
	else if(priv->videommap.format == VIDEO_PALETTE_YUV420P_MACRO){
//#ifdef CONFIG_SUPPORT_EXTRA_BUFFER_DEV1
#if CONFIG_VIN_DEV1_ENCODE_BUF_NO == 3
		UINT32 ibuf, ibufno= 3;
#elif CONFIG_VIN_DEV1_ENCODE_BUF_NO == 2
		UINT32 ibuf, ibufno= 2;
#elif CONFIG_VIN_DEV1_ENCODE_BUF_NO == 1
		UINT32 ibuf, ibufno= 1;
#endif
		DBG_PRINTF("Specified encode planar \n");
		for(ibuf=0; ibuf<ibufno; ibuf=ibuf+1){
			priv->pFramePlanarAddr[ibuf].u32PlaY = priv->videoIn_encode_buf[ibuf].u32PhysAddr;
			u32StartAddr = priv->pFramePlanarAddr[ibuf].u32PlaY;
			u32StartAddr = u32StartAddr + priv->videocrop.c.height * priv->videocrop.c.width; //U and V for Planar YUV420 macro
			priv->pFramePlanarAddr[ibuf].u32PlaU = u32StartAddr;
			priv->pFramePlanarAddr[ibuf].u32PlaV = u32StartAddr;
		}	
	}	
	else
	{
		printk("Specified encode format NOT support\n");
	}
#if CONFIG_VIN_DEV1_ENCODE_BUF_NO >= 1
	DBG_PRINTF("Planar Buf 0 Y phy = 0x%x\n", priv->pFramePlanarAddr[0].u32PlaY);
	DBG_PRINTF("Planar Buf 0 U phy = 0x%x\n", priv->pFramePlanarAddr[0].u32PlaU);
	DBG_PRINTF("Planar Buf 0 V phy = 0x%x\n", priv->pFramePlanarAddr[0].u32PlaV);
#endif
#if CONFIG_VIN_DEV1_ENCODE_BUF_NO >= 2
	DBG_PRINTF("Planar Buf 1 Y phy = 0x%x\n", priv->pFramePlanarAddr[1].u32PlaY);
	DBG_PRINTF("Planar Buf 1 U phy = 0x%x\n", priv->pFramePlanarAddr[1].u32PlaU);
	DBG_PRINTF("Planar Buf 1 V phy = 0x%x\n", priv->pFramePlanarAddr[1].u32PlaV);
#endif
#if CONFIG_VIN_DEV1_ENCODE_BUF_NO >= 3
	DBG_PRINTF("Planar Buf 2 Y phy = 0x%x\n", priv->pFramePlanarAddr[2].u32PlaY);
	DBG_PRINTF("Planar Buf 2 U phy = 0x%x\n", priv->pFramePlanarAddr[2].u32PlaU);
	DBG_PRINTF("Planar Buf 2 V phy = 0x%x\n", priv->pFramePlanarAddr[2].u32PlaV);
#endif
	printk("ViewWindow Width =%d\n", priv->videowin.width);
	printk("ViewWindow Height =%d\n", priv->videowin.height);
	printk("ViewWindow PosX =%d\n", priv->videowin.x);
	printk("ViewWindow PosY =%d\n",priv->videowin.y);

#if 0
	if( (priv->videocrop.c.width < priv->videowin.width) && 
		(priv->videocrop.c.height < priv->videowin.height) )
	{//Crop width less than specified packet dimension ==> reset the packet start address, downscale size and packet buffer start address.
		UINT32 i, j;
		UINT32 u32NewFrameAddr[3];
		UINT32* ptr;		
		for(i=0;i<VIDEOIN_PREVIEW_BUFFER_NUM;i++)
		{
			u32NewFrameAddr[i] = priv->videoIn_preview_buf[i].u32PhysAddr+ (priv->i32packet_height -priv->videocrop.c.height)/2*priv->i32packet_width*2 + priv->i32packet_width-priv->videocrop.c.width;
			DBG_PRINTF("Packet buffer %d physical address =0x%x\n", i, u32NewFrameAddr[i]);
		}
		//DrvVideoIn_SetPipeEnable(TRUE, eVIDEOIN_PLANAR);
		if( inp32(REG_PACBA0) == priv->pFramePackerAddr[0])	/* !!!!Set the address first will block the LCM to be updated !!!*/
		{
			DrvVideoIn_SetBaseStartAddress(eVIDEOIN_PACKET, 0, u32NewFrameAddr[0]);	
			ptr = priv->videoIn_preview_buf[1].u32VirtAddr;		
			clearFrameBuf(ptr);
			ptr = priv->videoIn_preview_buf[2].u32VirtAddr;		
			clearFrameBuf(ptr);	
			ptr = priv->videoIn_preview_buf[0].u32VirtAddr;		
			clearFrameBuf(ptr);
			clearFrameBuf(ptr);						
		}
		else if ( inp32(REG_PACBA0) == priv->pFramePackerAddr[1])
		{
			DrvVideoIn_SetBaseStartAddress(eVIDEOIN_PACKET, 0, u32NewFrameAddr[1]);	
			ptr = priv->videoIn_preview_buf[2].u32VirtAddr;		
			clearFrameBuf(ptr);
			ptr = priv->videoIn_preview_buf[0].u32VirtAddr;		
			clearFrameBuf(ptr);	
			ptr = priv->videoIn_preview_buf[1].u32VirtAddr;		
			clearFrameBuf(ptr);	
			clearFrameBuf(ptr);
		}		
		else if (  inp32(REG_PACBA0) == priv->pFramePackerAddr[2])
		{
			DrvVideoIn_SetBaseStartAddress(eVIDEOIN_PACKET, 0, u32NewFrameAddr[2]);	
			ptr = priv->videoIn_preview_buf[0].u32VirtAddr;		
			clearFrameBuf(ptr);
			ptr = priv->videoIn_preview_buf[1].u32VirtAddr;		
			clearFrameBuf(ptr);	
			ptr = priv->videoIn_preview_buf[2].u32VirtAddr;		
			clearFrameBuf(ptr);	
			clearFrameBuf(ptr);
		}
		for(i=0;i<VIDEOIN_PREVIEW_BUFFER_NUM;i++)
		{
			priv->pFramePackerAddr[i] = u32NewFrameAddr[i];
		}
		DrvVideoIn_SetVerticalScaleFactor(eVIDEOIN_PACKET, 1, 1);
		DrvVideoIn_SetHorizontalScaleFactor(eVIDEOIN_PACKET, 1, 1);				
	}
	else
	{//Crop width bigger than specified packet dimension.	
		UINT32 u32GCD, i;
		UINT32 u32NewFrameAddr[3];
		u32GCD = vinGCD(priv->videowin.height,		// Preview height
			    	priv->videocrop.c.height); 	// Crop height
		DrvVideoIn_SetVerticalScaleFactor(eVIDEOIN_PACKET, priv->videowin.height/u32GCD, priv->videocrop.c.height/u32GCD);
		DBG_PRINTF("Packet V DDA: %d/%d\n", priv->videowin.height/u32GCD, priv->videocrop.c.height/u32GCD);
		
		u32GCD = vinGCD(priv->videowin.width,			// Preview width
				priv->videocrop.c.width); 	// Crop width
		DrvVideoIn_SetHorizontalScaleFactor(eVIDEOIN_PACKET, priv->videowin.width/u32GCD, priv->videocrop.c.width/u32GCD);
		DBG_PRINTF("Packet H DDA: %d/%d\n", priv->videowin.width/u32GCD, priv->videocrop.c.width/u32GCD);
			
		for(i=0;i<VIDEOIN_PREVIEW_BUFFER_NUM;i++)
		{/* Capture engine only supoort 2-byte/pixel */
			u32NewFrameAddr[i] = priv->videoIn_preview_buf[i].u32PhysAddr+ priv->i32packet_width*priv->videowin.y*2 + priv->videowin.x*2; 
		}
		if( inp32(REG_PACBA0) == priv->pFramePackerAddr[0])	/* !!!!Set the new address first will block the LCM to be updated !!!*/
			DrvVideoIn_SetBaseStartAddress(eVIDEOIN_PACKET, 0, u32NewFrameAddr[0]);			
		else if ( inp32(REG_PACBA0) == priv->pFramePackerAddr[1])
			DrvVideoIn_SetBaseStartAddress(eVIDEOIN_PACKET, 0, u32NewFrameAddr[1]);	
		
		else if (  inp32(REG_PACBA0) == priv->pFramePackerAddr[2])
			DrvVideoIn_SetBaseStartAddress(eVIDEOIN_PACKET, 0, u32NewFrameAddr[2]);	
		for(i=0;i<VIDEOIN_PREVIEW_BUFFER_NUM;i++)
		{
			priv->pramePackerAddr[i] = u32NewFrameAddr[i];
		}				
	}
#endif
	//priv->pDevVin->SetPipeEnable(TRUE, eVIDEOIN_BOTH_PIPE_ENABLE);
	priv->i32Zooming = FALSE;				/* Update register */		
#if 0
	u32StartAddr = inp32(REG_PACBA0);		/* Wait a new frame */
	while(u32StartAddr==inp32(REG_PACBA0))
	{
		schedule_delay(30);					/* Delay another frame at least */
	}	
	printk("Wait a new frame done\n");
#endif
	return ret;	
}
static unsigned int vin_ioctl_get_cropping_window(struct file *file,
										 	unsigned int cmd,
											void *arg)
{
	int ret = 0;
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

	DBG_PRINTF("videoin_ioctl VIDIOC_G_CROP\n");


	DBG_PRINTF("priv->videocropcap.bounds.left = %d\n", priv->videocropcap.bounds.left); 
	DBG_PRINTF("priv->videocropcap.bounds.top = %d\n", priv->videocropcap.bounds.top);
	DBG_PRINTF("priv->videocropcap.bounds.width = %d\n", priv->videocropcap.bounds.width); 
	DBG_PRINTF("priv->videocropcap.bounds.height = %d\n", priv->videocropcap.bounds.height);

	DBG_PRINTF("priv->videocropcap.defrect.left = %d\n", priv->videocropcap.defrect.left); 
	DBG_PRINTF("priv->videocropcap.defrect.top = %d\n", priv->videocropcap.defrect.top );
	DBG_PRINTF("priv->videocropcap.defrect.width = %d\n", priv->videocropcap.defrect.width); 
	DBG_PRINTF("priv->videocropcap.defrect.height = %d\n", priv->videocropcap.defrect.height);

	DBG_PRINTF("priv->videocropcap.pixelaspect.numerator = %d\n", priv->videocropcap.pixelaspect.numerator);		/* Suppose current image size VGA */
	DBG_PRINTF("priv->videocropcap.pixelaspect.denominator = %d\n", priv->videocropcap.pixelaspect.denominator);	/* Zoomming step */


		
	if (copy_to_user((void*)arg, (void *)&priv->videocrop, sizeof(struct v4l2_crop))) {
		ERR_PRINTF("copy_to_user error VIDIOC_G_CROP\n");
		ret = -EFAULT;
	}
	return ret;		
}


#if 0
/*================================================== V4L2 User Ctrl ================================================== */
struct v4l2_queryctrl
{
	__u32		     id;
	enum v4l2_ctrl_type  type;
	__u8		 name[32];	/* Whatever */
	__s32		 minimum;	/* Note signedness */
	__s32		 maximum;
	__s32		 step;
	__s32		 default_value;
	__u32               flags;
	__u32		reserved[2];
};

/*
 *	C O N T R O L S
 */
struct v4l2_control
{
	__u32		     id;
	__s32		     value;
};

#endif
static int vin_ioctl_query_user_control(struct file *file,
									unsigned int cmd,
									void *arg)
{
	
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;
	struct v4l2_queryctrl queryctrl;


	DBG_PRINTF("videoin_ioctl VIDIOC_QUERYCTRL\n");
	if (copy_from_user(&queryctrl, (void *)arg, sizeof(struct v4l2_queryctrl))) {
		ERR_PRINTF("Error vin_ioctl_query_user_control\n");
		return -EFAULT;
	}

	if(priv->sensor_intf->query_private_user_ctrl)
		return priv->sensor_intf->query_private_user_ctrl(file,
															cmd,
															arg);
	else{
		ERR_PRINTF("Error vin_ioctl_query_user_control\n");
		return -EFAULT;
	}

	return 0;
} 

static int vin_ioctl_query_sensor_id(struct file *file,
                                    unsigned int cmd,
                                    void *arg)
{
	int ret = 0;
        struct video_device *dev = video_devdata(file);
        videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

        if (copy_to_user((void*)arg, &priv->i32SensorID, sizeof(UINT32))) {
                ERR_PRINTF("Error copy_to_user VIDIOC_QUERY_SENSOR_ID \n");
                ret = -EFAULT;
        }
        return ret;
}

static unsigned int vin_ioctl_user_ctrl(struct file *file,
									unsigned int cmd,
									void *arg)
{

	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;
	//struct v4l2_queryctrl queryctrl;
	struct v4l2_control ctrl;

	DBG_PRINTF("videoin_ioctl VIDIOC_G/S_CTRL\n");
	switch(cmd){
	case VIDIOC_G_CTRL:
		if(priv->sensor_intf->sensor_get_ctrl)
			priv->sensor_intf->sensor_get_ctrl(priv, arg);					/* arg is return value */
		break;
	case VIDIOC_S_CTRL:
		//if (copy_from_user(&queryctrl, (void *)arg, sizeof(struct v4l2_queryctrl))) {
		if (copy_from_user(&ctrl, (void *)arg, sizeof(struct v4l2_control))) {
			ERR_PRINTF("Error user control\n");
			return -EFAULT;
		}
		if(priv->sensor_intf->sensor_set_ctrl)
			//priv->sensor_intf->sensor_set_ctrl(priv, &queryctrl);			/* specified value is queryctrl.value */
			priv->sensor_intf->sensor_set_ctrl(priv, &ctrl);
		break;
	}
	
	return 0;
}


static unsigned int 
vin_ioctl_query_packet_info(struct file *file,
						 	unsigned int cmd,
							void *arg)
{
	int ret = 0;
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;	
	
	S_PIPE_INFO s_packet_info;
	
	s_packet_info.i32PipeBufNo = VIDEOIN_PREVIEW_BUFFER_NUM;	
	s_packet_info.i32PipeBufSize = priv->i32packet_width*priv->i32packet_height *16/8;
	s_packet_info.i32CurrPipePhyAddr = priv->videoIn_preview_buf[priv->grab_sync].u32PhysAddr;
	if (copy_to_user((void*)arg, (void *)&s_packet_info, sizeof(S_PIPE_INFO))) {
		ERR_PRINTF("copy_to_user error VIDIOC_G_PACKET_INFO\n");
		ret = -EFAULT;
	}
	return ret;		
}


static unsigned int 
vin_ioctl_query_planar_info(struct file *file,
						 	unsigned int cmd,
							void *arg)
{
	int ret = 0;
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;	
	
	S_PIPE_INFO s_planar_info;
	
	s_planar_info.i32PipeBufNo = VIDEOIN_ENCODE_BUFFER_NUM;	
	s_planar_info.i32PipeBufSize = priv->sensor_intf->u16MaxImgHeight * priv->sensor_intf->u16MaxImgWidth * 2;
	s_planar_info.i32CurrPipePhyAddr = priv->videoIn_encode_buf[priv->grab_sync].u32PhysAddr;
	if (copy_to_user((void*)arg, (void *)&s_planar_info, sizeof(S_PIPE_INFO))) {
		ERR_PRINTF("copy_to_user error VIDIOC_G_PLANAR_INFO\n");
		ret = -EFAULT;
	}
	return ret;		
}

static unsigned int vin_ioctl_set_mapping_buffer(struct file *file,
												 	unsigned int cmd,
													void *arg)
{
	struct video_device *dev = video_devdata(file);
	videoin_priv_t *priv = (videoin_priv_t *)dev->priv;

	printk("videoin_ioctl VIDIOC_S_MAP_BUF\n");
	if (copy_from_user((void*)&priv->i32MappingBuf, (void *)arg, sizeof(INT32))) {
		printk("copy_from_user error VIDIOC_S_MAP_BUF\n");
		return -EFAULT;
	}
	if (priv->i32MappingBuf >= (VIDEOIN_ENCODE_BUFFER_NUM + VIDEOIN_PREVIEW_BUFFER_NUM))
	{//start form 0
		printk("Specified mapping buffer is wrong 0x%x error\n", priv->i32MappingBuf);
		printk("Start from 0, 0 for planar bufer 0, ... packet buffer 0 start from (total planar buffer no -1 ) \n");
		return -EINVAL;
	}
	printk("Specified mapping buffer = %d\n", priv->i32MappingBuf);
	return 0;
}


#if 0
unsigned int vin_ioctl_query_packet_size(struct file *file,
									 	unsigned int cmd,
										void *arg)
{
	int ret = 0;
	unsigned int size; 

	DBG_PRINTF("videoin_ioctl VIDIOC_G_PACKET_SIZE\n");

	offset = priv->videombuf.offsets[priv->grab_sync+VIDEOIN_ENCODE_BUFFER_NUM];
	size = 	videoIn_md_Offset_info[1].u32Size;
	if (copy_to_user((void*)arg, &size, sizeof(unsigned long))) {
		ERR_PRINTF("copy_to_user error VIDIOC_G_DIFF_SIZE\n");
		ret = -EFAULT;
	}
	return ret;		
}

unsigned int vin_ioctl_query_packet_physical_addr(struct file *file,
									 	unsigned int cmd,
										void *arg)
{
	DBG_PRINTF("videoin_ioctl VIDIOC_G_PACKET_ADDR\n");
	size = 	videoIn_md_Offset_info[1].u32Size;
	if (copy_to_user((void*)arg, &size, sizeof(unsigned long))) {
		ERR_PRINTF("copy_to_user error VIDIOC_G_DIFF_SIZE\n");
		ret = -EFAULT;
	}
	return ret;		
}
#endif

static VINIOCTL_T nvt_vin_ioctl =
{
	vin_ioctl_get_capability, 
	vin_ioctl_get_overlay_window, 
	vin_ioctl_set_overlay_window,
	vin_ioctl_get_picture_properties,
	vin_ioctl_set_picture_properties,
	vin_ioctl_start_end_capture,
	vin_ioctl_grab_frames,
	vin_ioctl_get_map_buffer_info,
	vin_ioctl_sync,
	vin_ioctl_get_capture_time,
	vin_ioctl_get_sys_up_time,
	vin_ioctl_set_brightness,
	vin_ioctl_get_brightness,
	vin_ioctl_set_contrast,
	vin_ioctl_get_contrast,
	vin_ioctl_set_sharpness,
	vin_ioctl_get_sharpness,
	vin_ioctl_set_white_balance,
	vin_ioctl_get_white_balance,
	vin_ioctl_set_noise_reduction,
	vin_ioctl_get_noise_reduction,
	vin_ioctl_set_color_saturation,
	vin_ioctl_get_color_saturation,
	vin_ioctl_set_flicker_freq,
	vin_ioctl_set_IR_led,
	vin_ioctl_get_IR_led,
	vin_ioctl_query_user_control,
	vin_ioctl_query_sensor_id,
	vin_ioctl_user_ctrl,
	
	vin_ioctl_get_cropping_capability,
	vin_ioctl_get_cropping_window,
	vin_ioctl_set_cropping_window,
	
	NULL,
	NULL,
	NULL,

	vin_ioctl_query_packet_info,
	vin_ioctl_query_planar_info,
	vin_ioctl_set_mapping_buffer,
	//vin_ioctl_query_packet_size,
	//vin_ioctl_query_packet_physical_addr, /* for two pipe encode */
};


INT32 register_vin_port1_ioctl(VINIOCTL_T* pVinIoctl)
{
	*pVinIoctl = nvt_vin_ioctl;
	return Successful;	
}

