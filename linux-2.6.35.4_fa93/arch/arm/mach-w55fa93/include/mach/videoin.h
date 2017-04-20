/* videoin.h
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

#ifndef __ASM_ARM_W55FA93_VIDEOIN_H
#define __ASM_ARM_W55FA93_VIDEOIN_H
#if 0
#include <linux/config.h>
#endif 
#include <linux/videodev.h>
#include <linux/videodev.h>
#include <linux/videodev.h>

#include <asm/io.h>

#define	CONFIG_JPEG

#define CONFIG_VIDEOIN_BUFFER_COUNT				1
//#define CONFIG_VIDEOIN_VPOST_OVERLAY_BUFFER		

#define CONFIG_VIDEOIN_PREVIEW_BUFFER_SIZE  (LCDWIDTH*LCDHEIGHT*LCDBPP/8)

#define OUTPUT_2_IDLE	1
#define OUTPUT_2_LCD		2
#define OUTPUT_2_JPEG	3

//shot mode
#define CONTINUOUS		1//continue to shot
#define ONESHOT			2//one shot
#define CON_NUMBER		3//continue shot the number of jpegs

//parameters for set/get
typedef struct videoin_param{
	__u32	vaddr;
	__u32	paddr;
	struct{
		__u32	input_format;			/* VideoIn input format from Sensor  */
		__u32	output_format;		/* VideoIn output format to Panel */		
	}format;
	struct{
		__u32 x;						/* Preview resolution */
		__u32 y;
	}resolution;
	struct{
		__u32 bVsync;       // TRUE: High Active, FALSE: Low Active
		__u32 bHsync;       // TRUE: High Active, FALSE: Low Active
		__u32 bPixelClk;     // TRUE: Falling Edge, FALSE: Rising Edge;
	}polarity;
}videoin_param_t;

typedef struct videoin_viewwindow{
	struct{
		__u32	u32ViewWidth;		//Packet Width	
		__u32	u32ViewHeight;		//Packet Height		
	}ViewWindow;
	struct{			
		__u32	u32PosX;			// Packet output start position to pannel  
		__u32	u32PosY;			
	}ViewStartPos;
}videoin_window_t;

typedef struct videoin_encode{
		__u32	u32Width;		//Planar Width	
		__u32	u32Height;		//Planar Height			
		__u32	u32Format;		//Planar YUV422 or YUV420
		__u32	u32Enable;		//Planar pipe enable?		
}videoin_encode_t;

//information for get
typedef struct videoin_info{
    __u32	bufferend;		/*video in buffer end for round-robin[0, CONFIG_VIDEOIN_BUFFER_COUNT)*/
}videoin_info_t;

#define VIDEOIN_PREVIEW_PIPE_START			_IOW('v',130, struct videoin_param)
#define VIDEOIN_S_PARAM					_IOW('v',131, struct videoin_param)
//#define VIDEOIN_G_PARAM					_IOR('v',132, struct videoin_param)
#define VIDEOIN_STATE						_IOR('v',133, struct videoin_param)
//#define VIDEOIN_G_INFO					_IOR('v',134, struct videoin_info)
#define VIDEOIN_S_RESOLUTION				_IOW('v',135, struct videoin_info)
#define VIDEOIN_PREVIEW_PIPE_CTL			_IOW('v',136, __u32)
#define VIDEOIN_ENCODE_PIPE_CTL			_IOW('v',137, __u32)
#define VIDEOIN_SELECT_FRAME_BUFFER		_IOW('v',138, __u32)
#define VIDEOIN_S_VIEW_WINDOW			_IOW('v',139, struct videoin_viewwindow)
#define VIDEOIN_S_JPG_PARAM				_IOW('v',140, struct videoin_encode)

typedef __s32 (*set_engine)(void *dev_id, void* sbuffer);
typedef __s32 (*get_status)(void *dev_id);
typedef __s32 (*set_status)(void *dev_id, __s32);
typedef __s8 (*is_finished)(void *dev_id);

typedef struct videoin_output_engine_ops{
	set_engine		setengine;
	get_status		getstatus;//for videoin to get jpeg status
	set_status		setstatus;
	is_finished		isfinished;
}vout_ops_t;

void videoin_register_outputdev(void *dev_id, vout_ops_t *fn);

/* IO CTL */
#if 0
void vin_ioctl_start(struct inode *inode, \
					struct file *file, \
				 	unsigned int cmd, \
					void *arg);

void vin_ioctl_stop(struct inode *inode, \
					struct file *file, \
				 	unsigned int cmd, \
					void *arg);

unsigned int vin_ioctl_s_para(struct inode *inode, \
					struct file *file, \
				 	unsigned int cmd, \
					void *arg);

unsigned int vin_ioctl_s_view_window(struct inode *inode, \
					struct file *file, \
				 	unsigned int cmd, \
					void *arg);

unsigned int vin_ioctl_g_info(struct inode *inode, \
					struct file *file, \
				 	unsigned int cmd, \
					void *arg);

unsigned int vin_ioctl_clr_buffer(void);
#endif
#endif//__ASM_ARM_W55FA93_VIDEOIN_H
