/* videoinpriv.h
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

#ifndef __ASM_ARM_W55FA93_VIDEOIN_DEV1_PRIV_H
#define __ASM_ARM_W55FA93_VIDEOIN_DEV1_PRIV_H

#include <linux/semaphore.h>

//#define DBG_REGISTER_MESSAGE
//#define DBG_MESSAGE 
#ifdef DBG_MESSAGE 
#define pushout()	printk("%s Enter:--------------------------------------------------------------------\n", __FUNCTION__);
#define LEAVE()		printk("%s Leave:\n", __FUNCTION__);
#define ERRLEAVE()	printk("%s Leave:\n", __FUNCTION__);
#else
#define pushout(...)
#define LEAVE(...)		
#define ERRLEAVE(...)	
#endif
	
#define ISCAPTURED(priv)	(priv->videoin_bufferend_bak != priv->videoin_bufferend)
#define VIDEOIN_ENCODE_BUFFER_SIZE	(640*480*2)	//Max support 640*480*2
#define VIDEOIN_PREVIEW_BUFFER_NUM 		3


#if CONFIG_VIN_DEV1_ENCODE_BUF_NO == 1
	#define VIDEOIN_ENCODE_BUFFER_NUM	1
#elif CONFIG_VIN_DEV1_ENCODE_BUF_NO == 2
	#define VIDEOIN_ENCODE_BUFFER_NUM	2
#elif CONFIG_VIN_DEV1_ENCODE_BUF_NO == 3
	#define VIDEOIN_ENCODE_BUFFER_NUM	3
#endif

#define VIDEOIN_BUFFER_SIZE	(VIDEOIN_ENCODE_BUFFER_SIZE * VIDEOIN_BUFFER_NUM)
enum {
	FRAME_READY=0,		/* Ready to grab into */
	FRAME_GRABBING,		/* In the process of being grabbed into */
	FRAME_CAPTUREDONE,		/* Finished grabbing, but not been synced yet */
};


#define SPI0_ON		1
#define SENSOR_ON	2
#define _PREVIEW_

typedef struct
{
	UINT32		u32PhysAddr;
	UINT32		u32VirtAddr;
	struct timeval	tvCapTime;
	UINT8		u8FrameState;
}videoIn_buf_t;

typedef struct
{
	UINT32 u32PlaY;
	UINT32 u32PlaU;
	UINT32 u32PlaV;
}planar_buf_t;	

//#ifdef CONFIG_MOTION_DETECTION
typedef struct
{
	UINT32 u32MapOffsetAddr;
	UINT32 u32Size;	
}videoin_motion_t;
#define VIDEOIN_DIFF_BUFFER_NUM			2	/* One is diff buffer, following one is OutY buffer */
//#endif

typedef struct
{
	unsigned int (*get_capability)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*get_overlay_window)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*set_overlay_window)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*get_picture_properties)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*set_picture_properties)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*start_end_capture)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*grab_frames)(struct file *file, unsigned int cmd,void *arg);
	unsigned int (*get_map_buffer_info)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*sync)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*get_capture_time)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*get_sys_up_time)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*set_brightness)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*get_brightness)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*set_contrast)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*get_contrast)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*set_sharpness)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*get_sharpness)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*set_white_balance)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*get_white_balance)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*set_noise_reduction)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*get_noise_reduction)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*set_color_saturation)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*get_color_saturation)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*set_flicker_freq)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*set_IR_led)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*get_IR_led)(struct file *file, unsigned int cmd, void *arg);
	
	int (*query_user_control)(struct file *file, unsigned int cmd, void *arg);
	int (*query_sensor_id)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*user_ctrl)(struct file *file, unsigned int cmd, void *arg);

	unsigned int (*get_cropping_capability)(struct file *file, unsigned int cmd, void *arg);	
	unsigned int (*get_cropping_window)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*set_cropping_window)(struct file *file, unsigned int cmd, void *arg);

	unsigned int (*query_diff_offset)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*query_diff_size)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*set_motion_threshold)(struct file *file, unsigned int cmd, void *arg);
	
	unsigned int (*query_packet_info)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*query_planar_info)(struct file *file, unsigned int cmd, void *arg);
	unsigned int (*set_mapping_buffer)(struct file *file, unsigned int cmd, void *arg);
	//unsigned int (*query_packet_size)(struct file *file, unsigned int cmd, void *arg);

}VINIOCTL_T;

INT32 register_vin_ioctl(UINT32 u32port, VINIOCTL_T* pVinIoctl);

typedef struct nvt_sensor_t{
	__s32	(*sensor_init)(__u32 u32Sensor, void *priv);
	BOOL	(*sensor_poweron)(BOOL);
	BOOL	(*sensor_suspend)(BOOL);
	BOOL	(*sensor_reset)(BOOL);	

	BOOL	(*read_write_brightness)(void *priv, INT32 *pi32Value, BOOL bIsRead);			/* Should be disable in FA95 */
	BOOL	(*read_write_contrast)(void *priv, INT32 *pi32Value, BOOL bIsRead);				/* Should be disable in FA95 */	
	BOOL	(*read_write_sharpness)(void *priv, INT32 *pi32Value, BOOL bIsRead);			/* Should be disable in FA95 */
	BOOL	(*read_write_white_balance)(void *priv, INT32 *pi32Value, BOOL bIsRead);		/* Should be disable in FA95 */
	BOOL	(*read_write_noise_reduction)(void *priv, INT32 *pi32Value, BOOL bIsRead);		/* Should be disable in FA95 */	
	BOOL	(*read_write_color_saturation)(void *priv, INT32 *pi32Value, BOOL bIsRead);		/* Should be disable in FA95 */

	INT32	(*query_private_user_ctrl)(struct file *file, unsigned int cmd, unsigned long *arg);			/* new create for application adjust sensor table in fa95 */	
	INT32	(*sensor_i2c_setRegAddr)(void *priv, struct v4l2_control *c);												/* new create for application adjust sensor table in fa95 */
	INT32	(*sensor_set_ctrl)(void *priv, struct v4l2_control *c);													/* new create for application adjust sensor table in fa95 */
	INT32	(*sensor_get_ctrl)(void *priv, struct v4l2_control *c);																/* new create for application adjust sensor table in fa95 */
	
	BOOL	(*change_image_resolution)(void *priv, UINT16 u16ImageWidth, UINT16 u16ImageHeight);
	BOOL	(*set_flicker_freq)(void *priv, UINT32 u32FlickerFreq);
	BOOL	(*low_lux_detect)(void *priv);
	BOOL	(*control_IR_led)(void *priv, BOOL bIsOn);

	UINT8	u8SensorDevID;
	UINT16  	u16MaxImgHeight;
	UINT16  	u16MaxImgWidth;
	UINT16  	u16CurImgHeight;	
	UINT16  	u16CurImgWidth;
}NVT_SENSOR_T;

typedef struct videoin_priv {
	struct video_device 	jdev;				/* Must be the first field! */
	struct video_capability videocap;			//for v4l
	struct video_picture 	videopic;			//for v4l
	struct video_window 	videowin;			//for v4l
	struct video_mbuf 		videombuf;			//for v4l
	struct video_mmap 		videommap;			//for v4l
	
	struct v4l2_cropcap 	videocropcap;		//for v4l 	
	struct v4l2_crop  		videocrop;			//for v4l 

	unsigned int* 			pFramePackerAddr; 
	planar_buf_t* 			pFramePlanarAddr;
	unsigned int 			u32FbPhyAddr;		//Backup frame buffer address 

	struct semaphore        lock;
	int 					i32FrameNumber;
	int 					i32ReadFrameNum;
	unsigned int 			u32PortAddr;		//Device offset address (alway 0 or 0x800)			


	__u32	vaddr;								//if vaddr_src/vaddr_dst == 0, they will point to vaddr
	__u32	paddr;								//if paddr_src/paddr_dst == 0, they will point to paddunsigned int 
//	__u32	mmap_bufsize;						/*mmap buffer size, user can set buffer address or using mmap*/
//	__u32	input_format;						/*for captured*/
//	__u32	output_format;						/*for captured*/
//	__u32	preview_width;						/*for captured*/
//  __u32	preview_height;						/*for captured*/
//  __u32	engine;								//output to which ip engine: LCD or jpeg
//  __u32	shotmode;							//jpeg shot mode
//  __u32	shotnumber;							//jpeg shot number for CON_NUMBER shot mode
//  __u32	preview_resolution_changed;			/*for captured*/
//  __u32 	byte2pixel;							//is 2 times of actual byte
//	__u32	videoin_buffersize;					/*each video in buffer size*/
//	__u32	videoin_bufferend;
//	__u32	videoin_bufferend_bak;
	__u32	start_end_capture;					//for v4l
	__u32	grab_sync;							//for v4l
	INT32	i32SensorID;						//for detect Sensor ID
	INT32	i32Brightness;						//for sensor_control
	INT32	i32Contrast;						//for sensor_control
	INT32	i32Sharpness;						//for sensor_control
	INT32	i32WhiteBalance;					//for sensor_control
	INT32	i32NoiseReduction;					//for sensor_control
	INT32	i32ColorSaturation;					//for sensor_control
	INT32	i32IRLedMode;						//for IR LED control, 0: off, 1:on, 2: auto
	INT32	i32IsIRLedOn;						//Is IR Led on 
	INT32 	i32Zooming;

	INT32 	i32packet_width;					/* specified the packet buffer dimension if kernel option */
	INT32 	i32packet_height;					

	INT32 	i32MappingBuf;						/* specified the mapping buffer */
	INT32 	i32IsEnablePreview;
	INT32	i32AllocBufSize;					/* Allocate size */
	INT32	i32RemainingBufSize;				/* Remaining size */
	INT32 	i32StopCapture;						/* Stop capture in ISR according to IOCTL */			

	NVT_SENSOR_T	*sensor_intf;
//	__u8	**videoin_buffer;					/*A pointer store those allocated buffer address  */
//	void	*dev_id;
//	vout_ops_t *callback;

	videoIn_buf_t* videoIn_preview_buf;
	videoIn_buf_t* videoIn_encode_buf;
//#ifdef CONFIG_MOTION_DETECTION
	videoIn_buf_t* videoIn_diff_buf;
	videoin_motion_t* videoIn_md_Offset_info;
//#endif
	VINDEV_T* 	   pDevVin;
	VINIOCTL_T*    pDevIoctl;

}videoin_priv_t;



#if 0
unsigned int vin_ioctl_get_capability(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_get_overlay_window(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_set_overlay_window(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_get_picture_properties(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_set_picture_properties(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_start_end_capture(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_grab_frames(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_get_map_buffer_info(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_sync(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_get_capture_time(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_get_sys_up_time(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_set_brightness(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_get_brightness(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_set_contrast(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_get_contrast(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_set_sharpness(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_get_sharpness(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_set_white_balance(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_get_white_balance(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_set_noise_reduction(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_get_noise_reduction(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_set_color_saturation(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_get_color_saturation(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_set_flicker_freq(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_set_IR_led(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_get_IR_led(struct file *file,unsigned int cmd,void *arg);

int vin_ioctl_query_user_control(struct file *file,unsigned int cmd,void *arg);
unsigned int vin_ioctl_user_ctrl(struct file *file,unsigned int cmd,void *arg);

unsigned int vin_ioctl_query_diff_offset(struct file *file, unsigned int cmd, void *arg);
unsigned int vin_ioctl_query_diff_size(struct file *file, unsigned int cmd, void *arg);
unsigned int vin_ioctl_set_motion_threshold(struct file *file, unsigned int cmd, void *arg);

unsigned int vin_ioctl_get_cropping_capability(struct file *file, unsigned int cmd, void *arg);
unsigned int vin_ioctl_set_cropping_window(struct file *file, unsigned int cmd, void *arg);
unsigned int vin_ioctl_get_cropping_window(struct file *file, unsigned int cmd, void *arg);
int vin_ioctl_query_sensor_id(struct file *file, unsigned int cmd, void *arg);
void vin_set_init(struct file *file);
#endif 


#if 0
struct sensor_priv {
	struct v4l2_subdev		subdev;
	int						model;
	int						revision;

	bool					flag_vflip;
	bool					flag_hflip;
};
#endif

void ResetSensor(void);
void PowerdownSensor(BOOL bIsEnable);




/* If not defined frame buffer */
#if !defined(CONFIG_FB_W55FA93) && !defined(CONFIG_FB_W55FA93_MODULE)
#define LCDWIDTH	160		
#define LCDHEIGHT	120	
#define LCDBPP		16
#endif


#endif//__ASM_ARM_W55FA93_JPEG_PRIV_H

