/* sensor.c
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
#include <linux/delay.h>

#if 0
#include <linux/videodev.h>
#else
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#endif 

#include <linux/jiffies.h>
#if 0
#include <asm/arch/videoin.h>
#include <asm/arch/DrvVideoin.h>
#include <asm/arch/w55fa95_reg.h>
#include <asm/arch/fb.h>
#include <asm/arch/w55fa95_fb.h>
#include <asm/arch/w55fa95_gpio.h>
#else
#include <mach/w55fa93_reg.h>
#include <mach/fb.h>
#include <mach/w55fa93_fb.h>
#include <mach/videoin.h>
//#include <mach/DrvVideoin.h>
#include "DrvVideoin.h"
#include <mach/videodev_ex.h>
#include <mach/w55fa93_gpio.h>
#endif

#include <linux/moduleparam.h>
#include <asm/io.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/i2c-id.h>
#include <linux/i2c-dev.h>

#include "videoinpriv.h"
#include "DrvI2C.h"	/* */
#include "DrvVideoin.h"

#include <media/v4l2-chip-ident.h>
#include <media/v4l2-common.h>
#include <media/soc_camera.h>
//IMPORT_SYMBOL(w55fa95_FB_BG_PHY_ADDR);
//extern unsigned int w55fa95_FB_BG_PHY_ADDR;
//#define outp32(addr, value)		writel(value, addr)
//#define inp32(addr)				readl(addr)
 

#define _REG_TABLE_SIZE(nTableName)	sizeof(nTableName)/sizeof(struct OV_RegValue)

#define ERR_PRINTF			printk


//#define DBG
#ifdef DBG
#define ENTER()		printk("ENTER : %s  : %s\n",__FILE__, __FUNCTION__)
#define LEAVE()		printk("LEAVE : %s  : %s\n",__FILE__, __FUNCTION__)
#define ERRLEAVE()	printk("ERRLEAVE : %s  : %s : %d\n",__FILE__, __FUNCTION__, __LINE__)
#define SDBG		printk	
#else
#define ENTER(...)
#define LEAVE(...)	
#define ERRLEAVE(...)	
#define SDBG(...)
#endif

//extern VINDEV_T* pDevVin;
#define __STANDARD_I2C__

//extern videoIn_buf_t videoIn_preview_buf[];


struct OV_RegValue{
	__u8	uRegAddr;
	__u8	uValue;
};

struct OV_RegTable{
	struct OV_RegValue *sRegTable;
	__u16 uTableSize;
};

#ifdef CONFIG_SENSOR_OV7670_DEV1
static struct OV_RegValue g_sOV7670_RegValue[] = 
{
#if 0
	{0x12, 0x80},{0x11, 0x80},{0x3A, 0x04},{0x12, 0x00},{0x17, 0x13},{0x18, 0x01},{0x32, 0xB6}, 
	{0x2B, 0x10},{0x19, 0x02},{0x1A, 0x7A},{0x03, 0x0F},{0x0C, 0x00},{0x3E, 0x00},{0x70, 0x3A},
	{0x71, 0x35},{0x72, 0x11},{0x73, 0xF0},{0xA2, 0x3B},{0x1E, 0x07},{0x7a, 0x1e},{0x7b, 0x09},
	{0x7c, 0x14},{0x7d, 0x29},{0x7e, 0x50},{0x7f, 0x5F},{0x80, 0x6C},{0x81, 0x79},{0x82, 0x84},
	{0x83, 0x8D},{0x84, 0x96},{0x85, 0xA5},{0x86, 0xB0},{0x87, 0xC6},{0x88, 0xD8},{0x89, 0xE9},
	{0x55, 0x00},{0x13, 0xE0},{0x00, 0x00},{0x10, 0x00},{0x0D, 0x40},{0x42, 0x00},{0x14, 0x18},
	{0xA5, 0x02},{0xAB, 0x03},{0x24, 0x48},{0x25, 0x40},{0x26, 0x82},{0x9F, 0x78},{0xA0, 0x68},
	{0xA1, 0x03},{0xA6, 0xd2},{0xA7, 0xd2},{0xA8, 0xF0},{0xA9, 0x80},{0xAA, 0x14},{0x13, 0xE5},
	{0x0E, 0x61},{0x0F, 0x4B},{0x16, 0x02},{0x21, 0x02},{0x22, 0x91},{0x29, 0x07},{0x33, 0x0B},
	{0x35, 0x0B},{0x37, 0x1D},{0x38, 0x71},{0x39, 0x2A},{0x3C, 0x78},{0x4D, 0x40},{0x4E, 0x20},
	{0x69, 0x00},{0x6B, 0x0A},{0x74, 0x10},{0x8D, 0x4F},{0x8E, 0x00},{0x8F, 0x00},{0x90, 0x00},
	{0x91, 0x00},{0x96, 0x00},{0x9A, 0x80},{0xB0, 0x84},{0xB1, 0x0C},{0xB2, 0x0E},{0xB3, 0x7e},
	{0xB1, 0x00},{0xB1, 0x0c},{0xB8, 0x0A},{0x44, 0xfF},{0x43, 0x00},{0x45, 0x4a},{0x46, 0x6c},
	{0x47, 0x26},{0x48, 0x3a},{0x59, 0xd6},{0x5a, 0xff},{0x5c, 0x7c},{0x5d, 0x44},{0x5b, 0xb4},
	{0x5e, 0x10},{0x6c, 0x0a},{0x6d, 0x55},{0x6e, 0x11},{0x6f, 0x9e},{0x6A, 0x40},{0x01, 0x40},
	{0x02, 0x40},{0x13, 0xf7},{0x4f, 0x78},{0x50, 0x72},{0x51, 0x06},{0x52, 0x24},{0x53, 0x6c},
	{0x54, 0x90},{0x58, 0x1e},{0x62, 0x08},{0x63, 0x10},{0x64, 0x08},{0x65, 0x00},{0x66, 0x05},
	{0x41, 0x08},{0x3F, 0x00},{0x75, 0x44},{0x76, 0xe1},{0x4C, 0x00},{0x77, 0x01},{0x3D, 0xC2},
	{0x4B, 0x09},{0xC9, 0x60},{0x41, 0x08/*0x18*/},{0x56, 0x40},{0x34, 0x11},{0x3b, 0x02},{0xa4, 0x89},
	{0x92, 0x00},{0x96, 0x00},{0x97, 0x30},{0x98, 0x20},{0x99, 0x20},{0x9A, 0x84},{0x9B, 0x29},
	{0x9C, 0x03},{0x9D, 0x99},{0x9E, 0x7F},{0x78, 0x00},{0x94, 0x08},{0x95, 0x0D},{0x79, 0x01},
	{0xc8, 0xf0},{0x79, 0x0f},{0xc8, 0x00},{0x79, 0x10},{0xc8, 0x7e},{0x79, 0x0a},{0xc8, 0x80},
	{0x79, 0x0b},{0xc8, 0x01},{0x79, 0x0c},{0xc8, 0x0f},{0x79, 0x0d},{0xc8, 0x20},{0x79, 0x09},
	{0xc8, 0x80},{0x79, 0x02},{0xc8, 0xc0},{0x79, 0x03},{0xc8, 0x40},{0x79, 0x05},{0xc8, 0x30},
	{0x79, 0x26},{0x3b, 0x82},{0x43, 0x02},{0x44, 0xf2}
#else
	{0x12, 0x80},{0x11, 0x80},{0x3A, 0x04},{0x12, 0x00},{0x17, 0x13},{0x18, 0x01},{0x32, 0xB6}, 
	{0x2B, 0x10},{0x19, 0x02},{0x1A, 0x7A},{0x03, 0x0F},{0x0C, 0x00},{0x3E, 0x00},{0x70, 0x3A},
	{0x71, 0x35},{0x72, 0x11},{0x73, 0xF0},{0xA2, 0x3B},{0x1E, 0x07},{0x7a, 0x1e},{0x7b, 0x09},
	{0x7c, 0x14},{0x7d, 0x29},{0x7e, 0x50},{0x7f, 0x5F},{0x80, 0x6C},{0x81, 0x79},{0x82, 0x84},
	{0x83, 0x8D},{0x84, 0x96},{0x85, 0xA5},{0x86, 0xB0},{0x87, 0xC6},{0x88, 0xD8},{0x89, 0xE9},
	{0x55, 0x00},{0x13, 0xE0},{0x00, 0x00},{0x10, 0x00},{0x0D, 0x40},{0x42, 0x00},{0x14, 0x18},
	{0xA5, 0x02},{0xAB, 0x03},{0x24, 0x48},{0x25, 0x40},{0x26, 0x82},{0x9F, 0x78},{0xA0, 0x68},
	{0xA1, 0x03},{0xA6, 0xd2},{0xA7, 0xd2},{0xA8, 0xF0},{0xA9, 0x80},{0xAA, 0x14},{0x13, 0xE5},
	{0x0E, 0x61},{0x0F, 0x4B},{0x16, 0x02},{0x21, 0x02},{0x22, 0x91},{0x29, 0x07},{0x33, 0x0B},
	{0x35, 0x0B},{0x37, 0x1D},{0x38, 0x71},{0x39, 0x2A},{0x3C, 0x78},{0x4D, 0x40},{0x4E, 0x20},
	{0x69, 0x00},{0x6B, 0x0A},{0x74, 0x10},{0x8D, 0x4F},{0x8E, 0x00},{0x8F, 0x00},{0x90, 0x00},
	{0x91, 0x00},{0x96, 0x00},{0x9A, 0x80},{0xB0, 0x84},{0xB1, 0x0C},{0xB2, 0x0E},{0xB3, 0x7e},
	{0xB1, 0x00},{0xB1, 0x0c},{0xB8, 0x0A},{0x44, 0xfF},{0x43, 0x00},{0x45, 0x4a},{0x46, 0x6c},
	{0x47, 0x26},{0x48, 0x3a},{0x59, 0xd6},{0x5a, 0xff},{0x5c, 0x7c},{0x5d, 0x44},{0x5b, 0xb4},
	{0x5e, 0x10},{0x6c, 0x0a},{0x6d, 0x55},{0x6e, 0x11},{0x6f, 0x9e},{0x6A, 0x40},{0x01, 0x40},
	{0x02, 0x40},{0x13, 0xf7},{0x4f, 0x78},{0x50, 0x72},{0x51, 0x06},{0x52, 0x24},{0x53, 0x6c},
	{0x54, 0x90},{0x58, 0x1e},{0x62, 0x08},{0x63, 0x10},{0x64, 0x08},{0x65, 0x00},{0x66, 0x05},
	{0x41, 0x08},{0x3F, 0x00},{0x75, 0x44},{0x76, 0xe1},{0x4C, 0x00},{0x77, 0x01},{0x3D, 0xC2},
	{0x4B, 0x09},{0xC9, 0x60},{0x41, 0x18},{0x56, 0x40},{0x34, 0x11},{0x3b, 0x02},{0xa4, 0x89},
	{0x92, 0x00},{0x96, 0x00},{0x97, 0x30},{0x98, 0x20},{0x99, 0x20},{0x9A, 0x84},{0x9B, 0x29},
	{0x9C, 0x03},{0x9D, 0x99},{0x9E, 0x7F},{0x78, 0x00},{0x94, 0x08},{0x95, 0x0D},{0x79, 0x01},
	{0xc8, 0xf0},{0x79, 0x0f},{0xc8, 0x00},{0x79, 0x10},{0xc8, 0x7e},{0x79, 0x0a},{0xc8, 0x80},
	{0x79, 0x0b},{0xc8, 0x01},{0x79, 0x0c},{0xc8, 0x0f},{0x79, 0x0d},{0xc8, 0x20},{0x79, 0x09},
	{0xc8, 0x80},{0x79, 0x02},{0xc8, 0xc0},{0x79, 0x03},{0xc8, 0x40},{0x79, 0x05},{0xc8, 0x30},
	{0x79, 0x26},{0x3b, 0x82},{0x43, 0x02},{0x44, 0xf2}
#endif
};
#endif
#ifdef CONFIG_SENSOR_OV9660_DEV1
static struct OV_RegValue g_sOV9660_RegValue[]=
{//OV9660
		{0x12, 0x80},
	#if 1
		{0xd5, 0xff}, {0xd6, 0x3f}, {0x3d, 0x3c}, {0x11, 0x80},	{0x2a, 0x00}, {0x2b, 0x00},	//PCLK = SCLK
	#else		
		{0xd5, 0xff}, {0xd6, 0x3f}, {0x3d, 0x3c}, {0x11, 0x81},	{0x2a, 0x00}, {0x2b, 0x00},	//PCLK = SCLK/2
	#endif	
		{0x3a, 0xd9}, {0x3b, 0x00},	{0x3c, 0x58}, {0x3e, 0x50},	{0x71, 0x00}, {0x15, 0x00},
		{0xD7, 0x10}, {0x6a, 0x24},	{0x85, 0xe7}, {0x63, 0x00}, {0x12, 0x40}, {0x4d, 0x09},
	#if 0		
		{0x17, 0x0c}, {0x18, 0x5c},	{0x19, 0x02}, {0x1a, 0x3f},	{0x03, 0x03}, {0x32, 0xb4},	//641*48?
	#else
		{0x17, 0x0b}, {0x18, 0x5c},	{0x19, 0x02}, {0x1a, 0x3f},	{0x03, 0x03}, {0x32, 0xb4}, //648x48?
	#endif		
		{0x2b, 0x00}, {0x5c, 0x80},	{0x36, 0xb4}, {0x65, 0x10}, {0x70, 0x02}, {0x71, 0x9f},
		{0x64, 0xa4}, {0x5c, 0x80},	{0x43, 0x00}, {0x5D, 0x55}, {0x5E, 0x57}, {0x5F, 0x21},
		{0x24, 0x3e}, {0x25, 0x38},	{0x26, 0x72}, {0x14, 0x68}, {0x0C, 0x38}, {0x4F, 0x4f},
		{0x50, 0x42}, {0x5A, 0x67}, {0x7d, 0x30}, {0x7e, 0x00}, {0x82, 0x03}, {0x7f, 0x00},
		{0x83, 0x07}, {0x80, 0x03}, {0x81, 0x04}, {0x96, 0xf0}, {0x97, 0x00}, {0x92, 0x33},
		{0x94, 0x5a}, {0x93, 0x3a},	{0x95, 0x48}, {0x91, 0xfc}, {0x90, 0xff}, {0x8e, 0x4e},
		{0x8f, 0x4e}, {0x8d, 0x13},	{0x8c, 0x0c}, {0x8b, 0x0c},	{0x86, 0x9e}, {0x87, 0x11},
		{0x88, 0x22}, {0x89, 0x05},	{0x8a, 0x03}, {0x9b, 0x0e},	{0x9c, 0x1c}, {0x9d, 0x34},
		{0x9e, 0x5a}, {0x9f, 0x68},	{0xa0, 0x76}, {0xa1, 0x82},	{0xa2, 0x8e}, {0xa3, 0x98},
		{0xa4, 0xa0}, {0xa5, 0xb0},	{0xa6, 0xbe}, {0xa7, 0xd2},	{0xa8, 0xe2}, {0xa9, 0xee},
		{0xaa, 0x18}, {0xAB, 0xe7},	{0xb0, 0x43}, {0xac, 0x04},	{0x84, 0x40}, {0xad, 0x82},
   #if 0		
		{0xd9, 0x11}, {0xda, 0x00},	{0xae, 0x10}, {0xab, 0xe7},	{0xb9, 0x50}, {0xba, 0x3c},		//641*48?	
		{0xbb, 0x50}, {0xbc, 0x3c},	{0xbd, 0x8},  {0xbe, 0x19},	{0xbf, 0x2},  {0xc0, 0x8},
   #else
		{0xd9, 0x11}, {0xda, 0x00},	{0xae, 0x10}, {0xab, 0xe7},	{0xb9, 0x51}, {0xba, 0x3c},		//648x48?
		{0xbb, 0x51}, {0xbc, 0x3c},	{0xbd, 0x8},  {0xbe, 0x19},	{0xbf, 0x2},  {0xc0, 0x8},	
   #endif		
		{0xc1, 0x2a}, {0xc2, 0x34},	{0xc3, 0x2d}, {0xc4, 0x2d},	{0xc5, 0x0},  {0xc6, 0x98},
		{0xc7, 0x18}, {0x69, 0x48},	{0x74, 0xc0}, {0x7c, 0x28},	{0x65, 0x11}, {0x66, 0x00},
		{0x41, 0xc0}, {0x5b, 0x24},	{0x60, 0x82}, {0x05, 0x07},	{0x03, 0x03}, {0xd2, 0x94},
		{0xc8, 0x06}, {0xcb, 0x40},	{0xcc, 0x40}, {0xcf, 0x00},	{0xd0, 0x20}, {0xd1, 0x00},
		{0xc7, 0x18}, {0x0d, 0x92},	{0x0d, 0x90}		
};
#endif
#ifdef CONFIG_SENSOR_OV7725_DEV1
static struct OV_RegValue g_sOV7725_RegValue[] = 
{
	//NewKen sensor module initial for OV7725 20110613
		{0x12, 0x80},
//		{0x00, 0x00}, {0x00, 0x00}, {0x00, 0x64},	 	//Newken only
		{0x12, 0x00}, {0x3D, 0x03},
		{0x17, 0x22}, {0x18, 0xA4}, {0x19, 0x07}, {0x1A, 0xF0},
		{0x32, 0x02}, {0x29, 0xA0}, {0x2C, 0xF0},
		{0x2A, 0x02}, {0x65, 0x20}, {0x11, 0x01},
//		{0x15, 0x03},								//Newken olny
		{0x42, 0x7F}, {0x63, 0xE0}, {0x64, 0xFF},
		{0x66, 0x00},		
//		{0x66, 0xC0}, 								// For Video_In
		{0x67, 0x48}, {0x0D, 0x41},
		{0x0E, 0x01}, {0x0F, 0xC5}, {0x14, 0x11},
		{0x22, 0x7F}, {0x23, 0x03}, {0x24, 0x40},
		{0x25, 0x30}, {0x26, 0xA1}, {0x2B, 0x00},
		{0x6B, 0xAA}, {0x13, 0xEF}, {0x90, 0x05},
		{0x91, 0x01}, {0x92, 0x03}, {0x93, 0x00},
		{0x94, 0x90}, {0x95, 0x8A}, {0x96, 0x06},
		{0x97, 0x0B}, {0x98, 0x95}, {0x99, 0xA0},
		{0x9A, 0x1E}, {0x9B, 0x08}, {0x9C, 0x20},
		{0x9E, 0x81}, {0xA6, 0x04}, {0x7E, 0x0C},
		{0x7F, 0x24}, {0x80, 0x3A}, {0x81, 0x60},
		{0x82, 0x70}, {0x83, 0x7E}, {0x84, 0x8A},
		{0x85, 0x94}, {0x86, 0x9E}, {0x87, 0xA8},
		{0x88, 0xB4}, {0x89, 0xBE}, {0x8A, 0xCA},
		{0x8B, 0xD8}, {0x8C, 0xE2}, {0x8D, 0x28},
		{0x46, 0x05}, {0x47, 0x00}, {0x48, 0x00},
		{0x49, 0x12}, {0x4A, 0x00}, {0x4B, 0x13},
		{0x4C, 0x21}, 
		{0x0C, 0x10}, 
//		{0x0C, 0x00},								// For Video_In
		{0x09, 0x00},
		{0xFF, 0xFF}, {0xFF, 0xFF}	
};
#endif 
#ifdef CONFIG_SENSOR_OV7740_DEV1
static struct OV_RegValue g_sOV7740_RegValue[] = 
{
	/************************
	Input clock 24Mhz
	VGA YUV 25fps
	************************/
	{0x12, 0x80},
	{0x13, 0x00},
	
	//25fps ~3.125fps night mode for 50HZ light environment
	{0x11, 0x01},
	{0x55, 0x40},
	{0x2b, 0x5E},
	{0x2c, 0x02},
	{0x15, 0xF4},	

	{0x12, 0x00},
	{0xd5, 0x10},
	{0x0c, 0x12},
	{0x0E, 0xE1},
	{0x0d, 0x34},
	{0x17, 0x25},
	{0x18, 0xA0},
	{0x19, 0x03},
	{0x1a, 0xF0},
	{0x1b, 0x89},
	{0x22, 0x03},
	{0x29, 0x17},

	{0x31, 0xa0},
	{0x32, 0xf0},
	{0x33, 0xC4},//0xC4
	{0x35, 0x05},
	{0x36, 0x3f},
	{0x04, 0x60},
	{0x27, 0x80},
	{0x3d, 0x0f},
	{0x3e, 0x81},
	{0x3f, 0x40},
	{0x40, 0x7f},
	{0x41, 0x6a},
	{0x42, 0x29},
	{0x44, 0xe5},
	{0x45, 0x41},
	{0x47, 0x42},
	{0x48, 0x00},
	{0x49, 0x61},
	{0x4a, 0xa1},
	{0x4b, 0x5e},
	{0x4c, 0x18},
	{0x4d, 0x50},
	{0x4e, 0x13},
	{0x64, 0x00},
	{0x67, 0x88},
	{0x68, 0x1a},
	{0x14, 0x38},
	{0x24, 0x3c},
	{0x25, 0x30},
	{0x26, 0x72},
	{0x50, 0x97},
	{0x51, 0x7e},
	{0x52, 0x00},
	{0x53, 0x00},
	{0x20, 0x00},
	{0x21, 0x23},
	{0x38, 0x14},
	{0xe9, 0x00},
	{0x56, 0x55},
	{0x57, 0xff},
	{0x58, 0xff},
	{0x59, 0xff},
	{0x5f, 0x04},
	{0xec, 0x00},
	{0x13, 0xff},
	{0x80, 0x7D},
	{0x81, 0x3f},
	{0x82, 0x32},
	{0x83, 0x01},
	{0x38, 0x11},
	{0x84, 0x70},
	{0x85, 0x00},
	{0x86, 0x03},
	{0x87, 0x01},
	{0x88, 0x05},
	{0x89, 0x30},
	{0x8d, 0x30},
	{0x8f, 0x85},
	{0x93, 0x30},
	{0x95, 0x85},
	{0x99, 0x30},
	{0x9b, 0x85},
	{0x9c, 0x08},
	{0x9d, 0x12},
	{0x9e, 0x23},
	{0x9f, 0x45},
	{0xa0, 0x55},
	{0xa1, 0x64},
	{0xa2, 0x72},
	{0xa3, 0x7f},
	{0xa4, 0x8b},
	{0xa5, 0x95},
	{0xa6, 0xa7},
	{0xa7, 0xb5},
	{0xa8, 0xcb},
	{0xa9, 0xdd},
	{0xaa, 0xec},
	{0xab, 0x1a},
	{0xce, 0x78},
	{0xcf, 0x6e},
	{0xd0, 0x0a},
	{0xd1, 0x0c},
	{0xd2, 0x84},
	{0xd3, 0x90},
	{0xd4, 0x1e},
	{0x5a, 0x24},
	{0x5b, 0x1f},
	{0x5c, 0x88},
	{0x5d, 0x60},
	{0xac, 0x6e},
	{0xbe, 0xff},
	{0xbf, 0x00},
	{0x70, 0x00},
	{0x71, 0x34},
	{0x74, 0x28},
	{0x75, 0x98},
	{0x76, 0x00},
	{0x77, 0x08},
	{0x78, 0x01},
	{0x79, 0xc2},
	{0x7d, 0x02},
	{0x7a, 0x4e},
	{0x7b, 0x1f},
	{0xec, 0x00},
	{0x7c, 0x0c},
};
#endif 
struct OV_RegTable g_OV_InitTable[] =
{
#ifdef CONFIG_SENSOR_OV9660_DEV1
	{g_sOV9660_RegValue,_REG_TABLE_SIZE(g_sOV9660_RegValue)},		
#elif defined  CONFIG_SENSOR_OV7670_DEV1
	{g_sOV7670_RegValue,_REG_TABLE_SIZE(g_sOV7670_RegValue)},
#elif defined  CONFIG_SENSOR_OV7725_DEV1
	{g_sOV7725_RegValue,_REG_TABLE_SIZE(g_sOV7725_RegValue)},
#elif defined  CONFIG_SENSOR_OV7740_DEV1
	{g_sOV7740_RegValue,_REG_TABLE_SIZE(g_sOV7740_RegValue)},
#else
	{0,0}
#endif
};

__u8 g_uOvDeviceID[]= 
{
#ifdef CONFIG_SENSOR_OV9660_DEV1
	0x60,		// 0v9660
#elif defined  CONFIG_SENSOR_OV7670_DEV1
	0x42,		// ov7670	
#elif defined  CONFIG_SENSOR_OV7725_DEV1
	0x42,		// ov7725
#elif defined  CONFIG_SENSOR_OV7740_DEV1
	0x42,		// ov7725	
#else
	0x42		// not a device ID
#endif
};

static struct i2c_client *save_client;
 
static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	int ret = 0;
	
	ENTER();
	save_client = client;
	LEAVE();
	return ret;
}

static int sensor_remove(struct i2c_client *client)
{
	ENTER();
	LEAVE();
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
#ifdef CONFIG_SENSOR_OV9660_DEV1
	{ "ov9660", 0 },
#elif defined  CONFIG_SENSOR_OV7670_DEV1
	{ "ov7670", 0 },
#elif defined  CONFIG_SENSOR_OV7725_DEV1
	{ "ov7725", 0 },
#elif defined  CONFIG_SENSOR_OV7740_DEV1
	{ "ov7740", 0 },
#endif
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
#ifdef CONFIG_SENSOR_OV9660_DEV1
	.driver = {
		.name = "ov9660",
	},
#elif defined  CONFIG_SENSOR_OV7670_DEV1
	.driver = {
		.name = "ov7670",
	},
#elif defined  CONFIG_SENSOR_OV7725_DEV1
	.driver = {
		.name = "ov7725",
	},
#elif defined  CONFIG_SENSOR_OV7740_DEV1
	.driver = {
		.name = "ov7740",
	},
#endif
	.probe    = sensor_probe,
	.remove   = sensor_remove,
	.id_table = sensor_id,
};
 
#ifndef __STANDARD_I2C__
static void I2C_Delay(UINT32 u32Delay)
{
	volatile UINT32 i;
	for(;u32Delay!=0;u32Delay--)
		for(i=0;i<50;i++);
}

static BOOL 
I2C_Write_8bitSlaveAddr_8bitReg_8bitData(UINT8 uAddr, UINT8 uRegAddr, UINT8 uData)
{
	volatile unsigned int u32Delay = 0x100;

	ENTER();
	DrvI2C_SendStart();
	while(u32Delay--);		
	if ( (DrvI2C_WriteByte(uAddr,DrvI2C_Ack_Have,8)==FALSE) ||			// Write ID address to sensor
		 (DrvI2C_WriteByte(uRegAddr,DrvI2C_Ack_Have,8)==FALSE) ||	// Write register address to sensor
		 (DrvI2C_WriteByte(uData,DrvI2C_Ack_Have,8)==FALSE) )		// Write data to sensor
	{
		DrvI2C_SendStop();
		
		printk("Non-standard I2C error, Slave addr: reg_addr = 0x%x : 0x%x\n", uAddr, uRegAddr);
		return FALSE;
	}
	DrvI2C_SendStop();
	if (uRegAddr==0x12 && (uData&0x80)!=0)
	{
		mdelay(20);			
	}
	LEAVE();
	return TRUE;
}


static UINT8 I2C_Read_8bitSlaveAddr_8bitReg_8bitData(UINT8 uAddr, UINT8 uRegAddr)
{

	UINT8 u8Data;

	ENTER();

	// 2-Phase(ID address, register address) write transmission
	DrvI2C_SendStart();
	DrvI2C_WriteByte(uAddr,DrvI2C_Ack_Have,8);		// Write ID address to sensor
	DrvI2C_WriteByte(uRegAddr,DrvI2C_Ack_Have,8);	// Write register address to sensor
	DrvI2C_SendStop();

	// 2-Phase(ID-address, data(8bits)) read transmission
	DrvI2C_SendStart();
	DrvI2C_WriteByte(uAddr|0x01,DrvI2C_Ack_Have,8);		// Write ID address to sensor
	u8Data = DrvI2C_ReadByte(DrvI2C_Ack_Have,8);		// Read data from sensor
	DrvI2C_SendStop();

	LEAVE();
	return u8Data;

}
#endif 
static int32_t DrvVideoIn_I2cWriteOV(__u8 uAddr, __u8 uRegAddr, __u8 uData)
{
	
	int ret=0;	
#ifdef __STANDARD_I2C__
	#if 1
		int i;
		struct i2c_msg msg;
		u8 buf[2];
		
		msg.flags=!I2C_M_RD;
		msg.addr=save_client->addr;
		msg.len=2;
		msg.buf=buf;		

		buf[0]=(u8)uRegAddr;
		buf[1]=uData;
		ret=i2c_transfer(save_client->adapter,&msg,1);
		for(i=0;i<10; i=i+1)
			udelay(500);
		return ret;
	#else
		i2c_smbus_write_byte_data(save_client, uRegAddr, uData);	
		udelay(10);
		printk("write value = 0x%x\n", uData);	
		return ret;
	#endif
#else		
		ENTER();
		if(I2C_Write_8bitSlaveAddr_8bitReg_8bitData(uAddr, uRegAddr, uData)==FALSE)
			ret = -1;
		LEAVE();
		return ret;
#endif
}
 
static UINT8 DrvVideoIn_I2cReadOV(__u8 uAddr, __u8 uRegAddr)
{
#ifdef __STANDARD_I2C__
	#if 1
		struct i2c_msg msgs;
		int ret=-1;
		u8 buf[3];
		
		msgs.flags=!I2C_M_RD;
		msgs.addr=save_client->addr;
		msgs.len=1;
		msgs.buf=buf;
		buf[0]=(u8)(uRegAddr);

		ret=i2c_transfer(save_client->adapter,&msgs,1);
		
		msgs.flags=I2C_M_RD;
		msgs.addr=save_client->addr;
		msgs.len=1;
		msgs.buf=buf;

		ret=i2c_transfer(save_client->adapter,&msgs,1);
		return buf[0];
	#else
		UINT8 val;
		//int ret;
		ENTER();
		i2c_smbus_write_byte(save_client, uRegAddr);
		val = i2c_smbus_read_byte(save_client);
		SDBG("read value = 0x%x\n", val);	
		return val;		
	#endif
#else
		int ret=0;
		ENTER();
		I2C_Read_8bitSlaveAddr_8bitReg_8bitData(uAddr, uRegAddr);
		LEAVE();
		return 0;
#endif	
}






static __s32 OvRegConfig(__u32 nIndex)
{
	__u32 i;
	__u16 uTableSize;
	__u8  uDeviceID;
	__s32 res = 0; 
	struct OV_RegValue *psRegValue;
	ENTER();

#ifdef __STANDARD_I2C__
	printk("Standard I2C.\n");	
#else	
	printk("Non Standard I2C.\n");
	outp32(REG_GPBFUN, inp32(REG_GPBFUN)& ~(MF_GPB14|MF_GPB13));
	DrvI2C_Open(eDRVGPIO_GPIOB, 					
				eDRVGPIO_PIN13, 
				eDRVGPIO_GPIOB,
				eDRVGPIO_PIN14, 
				(PFN_DRVI2C_TIMEDELY)I2C_Delay);
	
#endif
	//SDBG("nIndex = %d\n", nIndex);	
	if ( nIndex >= (sizeof(g_uOvDeviceID)/sizeof(__u8)) ){
		printk("Specified sensor not exist\n");
		return -EBUSY;	
	}
	uTableSize = g_OV_InitTable[nIndex].uTableSize;
	psRegValue = g_OV_InitTable[nIndex].sRegTable;
	uDeviceID = g_uOvDeviceID[nIndex];

	if ( psRegValue == 0 ){
		printk("Specified sensor table not exist\n");
		return -EBUSY;	
	}

	for(i=0;i<uTableSize; i++, psRegValue++)
	{
		int32_t ret;
		udelay(100);	
		ret = DrvVideoIn_I2cWriteOV(uDeviceID, (psRegValue->uRegAddr), (psRegValue->uValue));
		if(ret<0)
		{
			printk("Wrong to write register addr = 0x%x, write data = 0x%x , ret = %d\n", (psRegValue->uRegAddr), (psRegValue->uValue), ret);		
		}	
	} 
	if(res>=0)
		printk("driver i2c initial done\n");
	else
		printk("driver i2c initial fail\n");	
	return res;
}


static __s32 InitSensor(__u32 u32Sensor, void *priv)
{
	__u32 u32PacStride, u32PlaStride;
	__s32 res = 0; 
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;

	ENTER();

#ifdef CONFIG_HV_FROM_GPE0_GPE1_DEV1
	//outp32(REG_GPDFUN, inp32(REG_GPDFUN) &~(MF_GPD2));
	//w55fa93_gpio_configure(GPIO_GROUP_D, 2);
	//w55fa93_gpio_set_output(GPIO_GROUP_D, 2);
	//w55fa93_gpio_set(GPIO_GROUP_D, 2, 1);		//GPIOD2 set high
#endif

	switch (u32Sensor)
	{//OV9660 will be 30f/s @ sensor clock 24MHz for VGA output.  
		case OV_7725:
		case OV_7740:
			if(u32Sensor == OV_7725)
			printk("Init OV_7725 \n"); 								
			else
				printk("Init OV_7740 \n");
#ifdef CONFIG_HV_FROM_GPB2_GPB3_DEV1
        	vin_priv->pDevVin->Init(TRUE,                              // BOOL bIsEnableSnrClock,
                                0,                                      // E_DRVVIDEOIN_SNR_SRC eSnrSrc,        
                                24000,                                  // UINT32 u32SensorFreq,
                                eDrvVideoIn_2nd_SNR_CCIR601);   		// E_DRVVIDEOIN_DEV_TYPE eDevType
#endif
#ifdef CONFIG_HV_FROM_GPE0_GPE1_DEV1
        	vin_priv->pDevVin->Init(TRUE,                                         
                                0,                                    
                                24000,                                  
                                eDrvVideoIn_3rd_SNR_CCIR601);   		        
#endif
			vin_priv->pDevVin->Open(72000, 24000);	
#if 0
			/* IN's demo board has sensor reset and power down control */		
			/* Cliff's demo board has not sensor reset and power down control */
			/* Aircraft demo board HV from GPB with reset and power down pin */
#endif	
		#if defined(CONFIG_SENSOR_POWER_DOWN)	
		PowerdownSensor(FALSE);
		#endif
#if defined(CONFIG_SENSOR_RESET)
	  	ResetSensor();
#endif	

			res = OvRegConfig(0);
			if( res<0 ){
				printk("Sensor initial fail \n");
				return res;	
			}
			else
				printk("Sensor initial successful \n");
			vin_priv->sensor_intf->u8SensorDevID = g_uOvDeviceID[0];//g_uOvDeviceID[pDevVin->ov7725];
			vin_priv->sensor_intf->u16CurImgHeight = vin_priv->sensor_intf->u16MaxImgHeight;
			vin_priv->sensor_intf->u16CurImgWidth = vin_priv->sensor_intf->u16MaxImgWidth;

			vin_priv->videocrop.c.left = 0;	/*X*/
			vin_priv->videocrop.c.top = 0;	/*Y*/	
			vin_priv->videocrop.c.width = vin_priv->sensor_intf->u16CurImgWidth;
			vin_priv->videocrop.c.height = vin_priv->sensor_intf->u16CurImgHeight;	

			vin_priv->videocropcap.bounds.left = 0; 
			vin_priv->videocropcap.bounds.top = 0;
			vin_priv->videocropcap.bounds.width = vin_priv->sensor_intf->u16CurImgWidth; 
			vin_priv->videocropcap.bounds.height = vin_priv->sensor_intf->u16CurImgHeight;

			vin_priv->videocropcap.defrect.left = 0; 
			vin_priv->videocropcap.defrect.top = 0;
			vin_priv->videocropcap.defrect.width = vin_priv->sensor_intf->u16CurImgWidth; 
			vin_priv->videocropcap.defrect.height =  vin_priv->sensor_intf->u16CurImgHeight;

			if(vin_priv->sensor_intf->u16CurImgWidth==1280){
				vin_priv->videocropcap.pixelaspect.numerator = 18;	/* Suppose current image size HD */
				vin_priv->videocropcap.pixelaspect.denominator = 32;	/* Zoomming step */
			}else {
				vin_priv->videocropcap.pixelaspect.numerator = 12;	/* Suppose current image size VGA/SVGA */
				vin_priv->videocropcap.pixelaspect.denominator = 16;	/* Zoomming step */
			}


			//vin_priv->pDevVin->Open(72000, 24000);		
			if(vin_priv->videommap.format ==VIDEO_PALETTE_YUV420P)			
				vin_priv->pDevVin->SetPlanarFormat(TRUE);	/* Planar YUV420 */
			else	
				vin_priv->pDevVin->SetPlanarFormat(FALSE);	/* Planar YUV422 */		
			vin_priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, vin_priv->videoIn_preview_buf[0].u32PhysAddr);
	
			vin_priv->pDevVin->EnableInt(eVIDEOIN_VINT);
			
			vin_priv->pDevVin->SetSensorPolarity(TRUE, 
										FALSE, 
										TRUE);
			vin_priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_UYVY, 
											eVIDEOIN_IN_YUV422, 									
											eVIDEOIN_OUT_YUV422);											
			vin_priv->pDevVin->SetCropWinStartAddr(0,					//UINT16 u16VerticalStart, 		Y
											0);					//UINT16 u16HorizontalStart, 	X
			/* Sensor subsample resolution (640, 480)*/
			vin_priv->pDevVin->SetCropWinSize(vin_priv->sensor_intf->u16MaxImgHeight,		//UINT16 u16Height, 
									 vin_priv->sensor_intf->u16MaxImgWidth);		//UINT16 u16Width;									 
			
			vin_priv->pDevVin->PreviewPipeSize(vin_priv->videowin.height, vin_priv->videowin.width);
			
			vin_priv->pDevVin->GetStride(&u32PacStride, &u32PlaStride);
			vin_priv->pDevVin->SetStride(vin_priv->videowin.width, u32PlaStride);															
			vin_priv->pDevVin->SetPipeEnable(FALSE,							// It means planar disable
										eVIDEOIN_PACKET);

			vin_priv->pDevVin->SetShadowRegister();
			printk("Sensor initial successful \n");
		break;
		case OV_7670:
			printk("Init OV_7670 \n"); 								
#ifdef CONFIG_HV_FROM_GPB2_GPB3_DEV1
        	vin_priv->pDevVin->Init(TRUE,                              // BOOL bIsEnableSnrClock,
                                0,                                      // E_DRVVIDEOIN_SNR_SRC eSnrSrc,        
                                24000,                                  // UINT32 u32SensorFreq,
                                eDrvVideoIn_2nd_SNR_CCIR601);   		// E_DRVVIDEOIN_DEV_TYPE eDevType
#endif
#ifdef CONFIG_HV_FROM_GPE0_GPE1_DEV1
        	vin_priv->pDevVin->Init(TRUE,                                         
                                0,                                    
                                24000,                                  
                                eDrvVideoIn_3rd_SNR_CCIR601);   		        
#endif
		vin_priv->pDevVin->Open(64000, 24000);
#ifdef CONFIG_HV_FROM_GPE0_GPE1_DEV1
			/* IN's demo board has sensor reset and power down control */
		#if defined(CONFIG_SENSOR_POWER_DOWN)	
			PowerdownSensor(FALSE);
		#endif
		#if defined(CONFIG_SENSOR_RESET)
		  	ResetSensor();
		#endif
#else	
		/* Cliff's demo board has not sensor reset and power down control */
#endif
			res = OvRegConfig(0);
			if( res<0 ){
				printk("Sensor initial fail \n");
				return res;	
			}
			else
				printk("Sensor initial successful \n");
			vin_priv->sensor_intf->u8SensorDevID = g_uOvDeviceID[0];//g_uOvDeviceID[pDevVin->ov7725];
			vin_priv->sensor_intf->u16CurImgHeight = vin_priv->sensor_intf->u16MaxImgHeight;
			vin_priv->sensor_intf->u16CurImgWidth = vin_priv->sensor_intf->u16MaxImgWidth;

			vin_priv->videocrop.c.left = 0;	/*X*/
			vin_priv->videocrop.c.top = 0;	/*Y*/	
			vin_priv->videocrop.c.width = vin_priv->sensor_intf->u16CurImgWidth;
			vin_priv->videocrop.c.height = vin_priv->sensor_intf->u16CurImgHeight;	

			vin_priv->videocropcap.bounds.left = 4; 
			vin_priv->videocropcap.bounds.top = 0;
			vin_priv->videocropcap.bounds.width = vin_priv->sensor_intf->u16CurImgWidth; 
			vin_priv->videocropcap.bounds.height = vin_priv->sensor_intf->u16CurImgHeight;

			vin_priv->videocropcap.defrect.left = 4; 
			vin_priv->videocropcap.defrect.top = 0;
			vin_priv->videocropcap.defrect.width = vin_priv->sensor_intf->u16CurImgWidth; 
			vin_priv->videocropcap.defrect.height =  vin_priv->sensor_intf->u16CurImgHeight;

			if(vin_priv->sensor_intf->u16CurImgWidth==1280){
				vin_priv->videocropcap.pixelaspect.numerator = 18;	/* Suppose current image size HD */
				vin_priv->videocropcap.pixelaspect.denominator = 32;	/* Zoomming step */
			}else {
				vin_priv->videocropcap.pixelaspect.numerator = 12;	/* Suppose current image size VGA/SVGA */
				vin_priv->videocropcap.pixelaspect.denominator = 16;	/* Zoomming step */
			}


			//vin_priv->pDevVin->Open(72000, 24000);		
			if(vin_priv->videommap.format ==VIDEO_PALETTE_YUV420P)			
				vin_priv->pDevVin->SetPlanarFormat(TRUE);	/* Planar YUV420 */
			else	
				vin_priv->pDevVin->SetPlanarFormat(FALSE);	/* Planar YUV422 */	
			vin_priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, vin_priv->videoIn_preview_buf[0].u32PhysAddr);
	
			vin_priv->pDevVin->EnableInt(eVIDEOIN_VINT);
			
			vin_priv->pDevVin->SetSensorPolarity(TRUE, 
										FALSE, 
										TRUE);
			vin_priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_UYVY, 
											eVIDEOIN_IN_YUV422, 									
											eVIDEOIN_OUT_YUV422);											
			vin_priv->pDevVin->SetCropWinStartAddr(0,					//UINT16 u16VerticalStart, 		Y
													4);					//UINT16 u16HorizontalStart, 	X
			/* Sensor subsample resolution (640, 480)*/
			vin_priv->pDevVin->SetCropWinSize(vin_priv->sensor_intf->u16MaxImgHeight,		//UINT16 u16Height, 
									 vin_priv->sensor_intf->u16MaxImgWidth);		//UINT16 u16Width;									 
			
			vin_priv->pDevVin->PreviewPipeSize(vin_priv->videowin.height, vin_priv->videowin.width);
			
			vin_priv->pDevVin->GetStride(&u32PacStride, &u32PlaStride);
			vin_priv->pDevVin->SetStride(vin_priv->videowin.width, u32PlaStride);															
			vin_priv->pDevVin->SetPipeEnable(FALSE,							// It means planar disable
										eVIDEOIN_PACKET);

			vin_priv->pDevVin->SetShadowRegister();
		break;		
		case OV_9660:
			printk("Init OV_9660 \n"); 				
#ifdef CONFIG_HV_FROM_GPB2_GPB3_DEV1
        	vin_priv->pDevVin->Init(TRUE,                              // BOOL bIsEnableSnrClock,
                                0,                                      // E_DRVVIDEOIN_SNR_SRC eSnrSrc,        
                                24000,                                  // UINT32 u32SensorFreq,
                                eDrvVideoIn_2nd_SNR_CCIR601);   		// E_DRVVIDEOIN_DEV_TYPE eDevType
#endif
#ifdef CONFIG_HV_FROM_GPE0_GPE1_DEV1
        	vin_priv->pDevVin->Init(TRUE,                                         
                                0,                                    
                                24000,                                  
                                eDrvVideoIn_3rd_SNR_CCIR601);   		        
#endif
			vin_priv->pDevVin->Open(72000, 24000);	
#ifdef CONFIG_HV_FROM_GPE0_GPE1_DEV1
			/* IN's demo board has sensor reset and power down control */
		#if defined(CONFIG_SENSOR_POWER_DOWN)	
			PowerdownSensor(FALSE);
		#endif
		#if defined(CONFIG_SENSOR_RESET)
		  	ResetSensor();
		#endif
#else	
		/* Cliff's demo board has not sensor reset and power down control */
#endif

			res = OvRegConfig(0);
				if( res<0 )
					return res;
			vin_priv->sensor_intf->u8SensorDevID = g_uOvDeviceID[0];
			vin_priv->sensor_intf->u16CurImgWidth = vin_priv->sensor_intf->u16MaxImgWidth;
			vin_priv->sensor_intf->u16CurImgHeight = vin_priv->sensor_intf->u16MaxImgHeight;

			vin_priv->videocrop.c.left = 0;	/*X*/
			vin_priv->videocrop.c.top = 0;	/*Y*/	
			vin_priv->videocrop.c.width = vin_priv->sensor_intf->u16CurImgWidth;
			vin_priv->videocrop.c.height = vin_priv->sensor_intf->u16CurImgHeight;	

			vin_priv->videocropcap.bounds.left = 0; 
			vin_priv->videocropcap.bounds.top = 0;
			vin_priv->videocropcap.bounds.width = vin_priv->sensor_intf->u16CurImgWidth; 
			vin_priv->videocropcap.bounds.height = vin_priv->sensor_intf->u16CurImgHeight;

			vin_priv->videocropcap.defrect.left = 0; 
			vin_priv->videocropcap.defrect.top = 0;
			vin_priv->videocropcap.defrect.width = vin_priv->sensor_intf->u16CurImgWidth; 
			vin_priv->videocropcap.defrect.height =  vin_priv->sensor_intf->u16CurImgHeight;

			if(vin_priv->sensor_intf->u16CurImgWidth==1280){
				vin_priv->videocropcap.pixelaspect.numerator = 18;	/* Suppose current image size HD */
				vin_priv->videocropcap.pixelaspect.denominator = 32;	/* Zoomming step */
			}else {
				vin_priv->videocropcap.pixelaspect.numerator = 12;	/* Suppose current image size VGA/SVGA */
				vin_priv->videocropcap.pixelaspect.denominator = 16;	/* Zoomming step */
			}			

			//vin_priv->pDevVin->Open(72000, 24000);		
			if(vin_priv->videommap.format ==VIDEO_PALETTE_YUV420P)			
				vin_priv->pDevVin->SetPlanarFormat(TRUE);	/* Planar YUV420 */
			else	
				vin_priv->pDevVin->SetPlanarFormat(FALSE);	/* Planar YUV422 */		
			vin_priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, vin_priv->videoIn_preview_buf[0].u32PhysAddr);

			vin_priv->pDevVin->EnableInt(eVIDEOIN_VINT);
			
			vin_priv->pDevVin->SetSensorPolarity(TRUE, 
										FALSE, 
										TRUE);
			vin_priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_UYVY,
											eVIDEOIN_IN_YUV422, 									
											eVIDEOIN_OUT_YUV422);											
			vin_priv->pDevVin->SetCropWinStartAddr(0,					//UINT16 u16VerticalStart, 	Y
										4);					//UINT16 u16HorizontalStart, 	X
			/* Sensor subsample resolution (640, 480)*/
			vin_priv->pDevVin->SetCropWinSize(vin_priv->sensor_intf->u16MaxImgHeight,		//UINT16 u16Height, 
									 vin_priv->sensor_intf->u16MaxImgWidth);		//UINT16 u16Width;									 				 
		
			vin_priv->pDevVin->PreviewPipeSize(vin_priv->videowin.height, vin_priv->videowin.width);
			vin_priv->pDevVin->GetStride(&u32PacStride, &u32PlaStride);
			vin_priv->pDevVin->SetStride(vin_priv->videowin.width, u32PlaStride);							
					
			vin_priv->pDevVin->SetPipeEnable(FALSE,							// It means planar disable
									eVIDEOIN_PACKET);		
			vin_priv->pDevVin->SetShadowRegister();				

		break;	

		default:
			return FALSE;
		break;
	}	// switch (u32Sensor)	
	return 0;	
}

static BOOL	
OVReadWriteBrightness(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, 0x55)&0xff);
	else
		DrvVideoIn_I2cWriteOV(vin_priv->sensor_intf->u8SensorDevID, 0x55, *pi32Value);
	return TRUE;
}

static BOOL	
OVReadWriteContrast(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, 0x56)&0xff);
	else
		DrvVideoIn_I2cWriteOV(vin_priv->sensor_intf->u8SensorDevID, 0x56, *pi32Value);
	return TRUE;
}

static BOOL	
OVReadWriteSharpness(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, 0x3f)&0xff);
	else
		DrvVideoIn_I2cWriteOV(vin_priv->sensor_intf->u8SensorDevID, 0x3f, *pi32Value);

	return TRUE;
}


static BOOL	
OVReadWriteWhiteBalance(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, 0x6f)&0xff);
	else
		DrvVideoIn_I2cWriteOV(vin_priv->sensor_intf->u8SensorDevID, 0x6f, *pi32Value);

	return TRUE;
}

static BOOL	
OVReadWriteNoiseReduction(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, 0x4c)&0xff);
	else
		DrvVideoIn_I2cWriteOV(vin_priv->sensor_intf->u8SensorDevID, 0x4c, *pi32Value);

	return TRUE;
}

static BOOL	
OVReadWriteColorSaturation(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, 0xc9)&0xff);
	else
		DrvVideoIn_I2cWriteOV(vin_priv->sensor_intf->u8SensorDevID, 0xc9, *pi32Value);

	return TRUE;
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

static UINT16 u16SensRegAddr = 0; 
#define V4L2_CID_PRIVATE_I2C_SET_REG_ADDR     	(V4L2_CID_PRIVATE_BASE + 0)
#define V4L2_CID_PRIVATE_I2C_WRITE     			(V4L2_CID_PRIVATE_BASE + 1)
#define V4L2_CID_PRIVATE_I2C_READ     			(V4L2_CID_PRIVATE_BASE + 2)
#define V4L2_CID_PRIVATE_LASTP1     				 (V4L2_CID_PRIVATE_BASE + 3)

static const struct v4l2_queryctrl no_ctrl = {
	.name  = "42",
	.flags = V4L2_CTRL_FLAG_DISABLED,
};
static const struct v4l2_queryctrl video_ctrls[] = {
	/* --- private --- */
	{
		.id            	= V4L2_CID_PRIVATE_I2C_SET_REG_ADDR,
		.name          	= "i2c_set_addr",
		.minimum       = 0,
		.maximum       = 255,
		.step          	= 1,
		.type          	= V4L2_CTRL_TYPE_INTEGER,
	},{
		.id            	= V4L2_CID_PRIVATE_I2C_WRITE,
		.name          	= "i2c_write",
		.minimum       = 0,
		.maximum       = 255,
		.step          	= 1,
		.type          	= V4L2_CTRL_TYPE_INTEGER,
	},{
		.id            = V4L2_CID_PRIVATE_I2C_READ,
		.name          = "i2c_read",
		.minimum       = 0,
		.maximum       = 255,
		.step          	= 1,
		.type          = V4L2_CTRL_TYPE_INTEGER,
	}
};
static const unsigned int CTRLS = ARRAY_SIZE(video_ctrls);

static const struct v4l2_queryctrl* ctrl_by_id(unsigned int id)
{
	unsigned int i;

	for (i = 0; i < CTRLS; i++)
		if (video_ctrls[i].id == id)
			return video_ctrls+i;
	return NULL;
}

static int SensorUserPrivateCtrl(struct file *file,
		 							unsigned int cmd, 
									unsigned long *arg)
{
	const struct v4l2_queryctrl *ctrl;
	struct v4l2_queryctrl *c = (struct v4l2_queryctrl *)arg;

	if ((c->id <  V4L2_CID_BASE ||
	     c->id >= V4L2_CID_LASTP1) &&
	    (c->id <  V4L2_CID_PRIVATE_BASE ||
	     c->id >= V4L2_CID_PRIVATE_LASTP1))
		return -EINVAL;
	ctrl = ctrl_by_id(c->id);
	*c = (NULL != ctrl) ? *ctrl : no_ctrl;
	return 0;
}

static BOOL	
SensorI2cWriteData(
	void *priv, 
	struct v4l2_control *c
)
{
	const struct v4l2_queryctrl* ctrl;
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;

	ctrl = ctrl_by_id(c->id);
	if (NULL == ctrl)
		return -EINVAL;
	SDBG("set_control name=%s val=%d\n",ctrl->name,c->value);
	switch (ctrl->type) {
	case V4L2_CTRL_TYPE_BOOLEAN:
	case V4L2_CTRL_TYPE_MENU:
	case V4L2_CTRL_TYPE_INTEGER:
		if (c->value < ctrl->minimum)
			c->value = ctrl->minimum;
		if (c->value > ctrl->maximum)
			c->value = ctrl->maximum;
		break;
	default:
		/* nothing */;
	};

	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	DrvVideoIn_I2cWriteOV(vin_priv->sensor_intf->u8SensorDevID, u16SensRegAddr, c->value);
	return TRUE;
}
static BOOL	
SensorI2cReadData(
	void *priv, 
	struct v4l2_control *c
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();	
	if(vin_priv->sensor_intf->u8SensorDevID != g_uOvDeviceID[0])
		return FALSE;
	c->value = (DrvVideoIn_I2cReadOV(vin_priv->sensor_intf->u8SensorDevID, u16SensRegAddr)&0xff);
	return TRUE;
}

static INT32 
SensorI2cSetRegAddr(
	void *priv, 
	struct v4l2_control *c
)
{
	u16SensRegAddr  = c->value;
	SDBG("Specified sensor addr = 0x%x\n", u16SensRegAddr);
	return 0;
}

/* ------------------------------------------------------------------ */
static INT32 SensorI2cReadCtrl(void *priv,
				 				struct v4l2_control *c)
{
	const struct v4l2_queryctrl* ctrl;

	ctrl = ctrl_by_id(c->id);
	SDBG("Get_control name=%s\n",ctrl->name);
	if (NULL == ctrl)
		return -EINVAL;
	switch (c->id) {
/*
	case V4L2_CID_PRIVATE_I2C_WRITE:
		break;
*/
	case V4L2_CID_PRIVATE_I2C_READ:
		if( SensorI2cReadData(priv, c) == FALSE)
		{
			printk("i2c read faIL\n");	
			return -EINVAL;	/* I2c read fail */
		}	
		break;
	case V4L2_CID_PRIVATE_I2C_SET_REG_ADDR:
		c->value = u16SensRegAddr;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
 

static int SensorI2cWriteCtrl(void *priv, 
					struct v4l2_control *c)
{
	const struct v4l2_queryctrl* ctrl;
	//unsigned long flags;
	//int restart_overlay = 0;

	ctrl = ctrl_by_id(c->id);
	if (NULL == ctrl)
		return -EINVAL;
	SDBG("set_control name=%s val=%d\n",ctrl->name,c->value);
	switch (ctrl->type) {
	case V4L2_CTRL_TYPE_BOOLEAN:
	case V4L2_CTRL_TYPE_MENU:
	case V4L2_CTRL_TYPE_INTEGER:
		if (c->value < ctrl->minimum)
			c->value = ctrl->minimum;
		if (c->value > ctrl->maximum)
			c->value = ctrl->maximum;
		break;
	default:
		/* nothing */;
	};
	switch (c->id) {
	case V4L2_CID_PRIVATE_I2C_WRITE:
		if(SensorI2cWriteData(priv, c)==FALSE)
		{
			printk("i2c write faIl\n");	
			return -EINVAL;	/* I2c write fail */
		}
		break;	
/*
	case V4L2_CID_PRIVATE_I2C_READ:
		break;	
*/
	case V4L2_CID_PRIVATE_I2C_SET_REG_ADDR:
		u16SensRegAddr  = c->value;
		printk("Specified sensor addr = 0x%x\n", u16SensRegAddr);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}


#if 0
/*================================================== V4L2 User Ctrl ================================================== */
#endif 

static NVT_SENSOR_T nvt_sensor_ov = {
	sensor_init:				InitSensor,
	sensor_poweron:				NULL,

#ifdef CONFIG_SENSOR_POWER_DOWN
	sensor_suspend:				PowerdownSensor,
#else
	sensor_suspend:				NULL,
#endif
#ifdef CONFIG_SENSOR_RESET
	sensor_reset:				ResetSensor,
#else	
	sensor_reset:				NULL,
#endif
	read_write_brightness:			OVReadWriteBrightness,
	read_write_contrast:			OVReadWriteContrast,
	read_write_sharpness:			OVReadWriteSharpness,
	read_write_white_balance:		OVReadWriteWhiteBalance,
	read_write_noise_reduction:		OVReadWriteNoiseReduction,
	read_write_color_saturation:	OVReadWriteColorSaturation,

	query_private_user_ctrl:		SensorUserPrivateCtrl,    /* OK */
	sensor_i2c_setRegAddr:			SensorI2cSetRegAddr, 	/* OK */
	sensor_set_ctrl:				SensorI2cWriteCtrl,
	sensor_get_ctrl:				SensorI2cReadCtrl,

	change_image_resolution: 		NULL,
	set_flicker_freq:				NULL,
	low_lux_detect:					NULL,
	control_IR_led:					NULL,

	u16MaxImgHeight:				480,		 
	u16MaxImgWidth: 				640,
};
#ifdef CONFIG_W55FA93_VIDEOIN_DEV1
INT32 register_vin_port1_Sensor(NVT_SENSOR_T **sensor_intf)
{
	*sensor_intf = (NVT_SENSOR_T*)(&nvt_sensor_ov);
	return 0;
}
#endif 
//#ifdef CONFIG_W55FA93_VIDEOIN_DEV2
//INT32 register_vin_port2_Sensor(NVT_SENSOR_T **sensor_intf)
//{
	//*sensor_intf = (NVT_SENSOR_T*)(&nvt_sensor_ov;
	//return 0;
//}
//#endif 


#ifdef __STANDARD_I2C__ 
static int __init i2c_init(void)
{
	int ret;

	ENTER();
	ret = i2c_add_driver(&sensor_i2c_driver);
	if(ret)
		ERRLEAVE();
	else
		LEAVE();
	return ret;
}
static void __exit i2c_cleanup(void)
{
	ENTER();
	i2c_del_driver(&sensor_i2c_driver);
	LEAVE();
}

MODULE_LICENSE("GPL");
module_init(i2c_init);
module_exit(i2c_cleanup);
#endif
