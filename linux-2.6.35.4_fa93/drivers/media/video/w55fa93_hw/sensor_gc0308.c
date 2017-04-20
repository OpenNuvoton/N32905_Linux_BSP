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
 

#define _REG_TABLE_SIZE(nTableName)	sizeof(nTableName)/sizeof(struct GC_RegValue)

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


//struct GC_RegValue{
struct GC_RegValue{
	__u8	uRegAddr;
	__u8	uValue;
};

struct GC_RegTable{
	struct GC_RegValue *sRegTable;
	__u16 uTableSize;
};


static struct GC_RegValue g_sGC0308_RegValue[] = 
{
	#include "GC0308/GC0308_VGA.dat"
};


struct GC_RegTable g_GC_InitTable[] =
{
	{g_sGC0308_RegValue,_REG_TABLE_SIZE(g_sGC0308_RegValue)},
	{0,0}

};

__u8 g_uGcDeviceID[]= 
{
	0x42,		// gc0308	
};
#ifdef __STANDARD_I2C__
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
	{ "gc0308", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name = "gc0308",
	},
	.probe    = sensor_probe,
	.remove   = sensor_remove,
	.id_table = sensor_id,
};
#endif 
#ifndef __STANDARD_I2C__
static void I2C_Delay(UINT32 u32Delay)
{
	volatile UINT32 i;
	for(;u32Delay!=0;u32Delay--)
		for(i=0;i<50;i++);
}

BOOL 
I2C_Write_8bitSlaveAddr_8bitReg_8bitData(UINT8 uAddr, UINT8 uRegAddr, UINT8 uData)
{
	// 3-Phase(ID address, regiseter address, data(8bits)) write transmission
	volatile __u32 u32Delay = 0x200;
	DrvI2C_SendStart();
	while(u32Delay--);		
	if ( (DrvI2C_WriteByte(uAddr,DrvI2C_Ack_Have,8)==FALSE) ||		// Write ID address to sensor
		 (DrvI2C_WriteByte(uRegAddr,DrvI2C_Ack_Have,8)==FALSE) ||	// Write register address to sensor
		 (DrvI2C_WriteByte(uData,DrvI2C_Ack_Have,8)==FALSE) )		// Write data to sensor
	{
		DrvI2C_SendStop();
		return FALSE;
	}
	DrvI2C_SendStop();

	return TRUE;
}

UINT8 I2C_Read_8bitSlaveAddr_8bitReg_8bitData(UINT8 uAddr, UINT8 uRegAddr)
{
	UINT8 u8Data;
	
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
	
	return u8Data;
}
#endif 
s8  DrvVideoIn_I2cWriteGC(__u8 uAddr, __u8 uRegAddr, __u8 uData)
{
	//DBG_PRINTF("%s\n",__FUNCTION__);	
#ifdef __STANDARD_I2C__
	i2c_smbus_write_byte_data(save_client, uRegAddr, uData);
#else
	I2C_Write_8bitSlaveAddr_8bitReg_8bitData(uAddr, uRegAddr, uData);
#endif
	return TRUE;
}
 
__s8  DrvVideoIn_I2cReadGC(__u8 uAddr, __u8 uRegAddr)
{
#ifdef __STANDARD_I2C__
	i2c_smbus_write_byte(save_client, uRegAddr);
	return i2c_smbus_read_byte(save_client);
#else
	return I2C_Read_8bitSlaveAddr_8bitReg_8bitData(uAddr, uRegAddr);
#endif
	//return i2c_smbus_read_byte_data(save_client, uRegAddr);		
}
static __s32 GCRegConfig(__u32 nIndex)
{
	__u32 i;
	__u16 uTableSize;
	__u8  uDeviceID;
	__s32 res = 0; 
	struct GC_RegValue *psRegValue;
	ENTER();

#ifdef __STANDARD_I2C__
	printk("Standard I2C.\n");	
#else	
	printk("Non Standard I2C.\n");
		#if 0
	outp32(REG_GPDFUN, inp32(REG_GPDFUN)& ~(MF_GPD10|MF_GPD9));
        DrvI2C_Open(eDRVGPIO_GPIOD,
                                eDRVGPIO_PIN9,
                                eDRVGPIO_GPIOD,
                                eDRVGPIO_PIN10,
                                (PFN_DRVI2C_TIMEDELY)I2C_Delay);
   #else
   outp32(REG_GPBFUN, inp32(REG_GPBFUN)& ~(MF_GPB14|MF_GPB13));
	 DrvI2C_Open(eDRVGPIO_GPIOB, 					
				eDRVGPIO_PIN13, 
				eDRVGPIO_GPIOB,
				eDRVGPIO_PIN14, 
				(PFN_DRVI2C_TIMEDELY)I2C_Delay);   
	#endif	
	
#endif
	//SDBG("nIndex = %d\n", nIndex);	
	if ( nIndex >= (sizeof(g_uGcDeviceID)/sizeof(__u8)) ){
		printk("Specified sensor not exist\n");
		return -EBUSY;	
	}
	uTableSize = g_GC_InitTable[nIndex].uTableSize;
	psRegValue = g_GC_InitTable[nIndex].sRegTable;
	uDeviceID = g_uGcDeviceID[nIndex];

	if ( psRegValue == 0 ){
		printk("Specified sensor table not exist\n");
		return -EBUSY;	
	}

	for(i=0;i<uTableSize; i++, psRegValue++)
	{
		int32_t ret;
		udelay(100);	
		ret = DrvVideoIn_I2cWriteGC(uDeviceID, (psRegValue->uRegAddr), (psRegValue->uValue));
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

#if 0
#ifdef CONFIG_HV_FROM_GPE0_GPE1_DEV1
	outp32(REG_GPDFUN, inp32(REG_GPDFUN) &~(MF_GPD2));
	w55fa93_gpio_configure(GPIO_GROUP_D, 2);
	w55fa93_gpio_set_output(GPIO_GROUP_D, 2);
	w55fa93_gpio_set(GPIO_GROUP_D, 2, 1);		//GPIOD2 set high
#endif
#endif
	
	printk("Init GC0308 \n"); 								
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


	res = GCRegConfig(0);
	if( res<0 ){
		printk("Sensor initial fail \n");
		return res;	
	}
	else
		printk("Sensor initial successful \n");
	vin_priv->sensor_intf->u8SensorDevID = g_uGcDeviceID[0];//g_uGcDeviceID[pDevVin->ov7725];
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
	
	vin_priv->pDevVin->SetSensorPolarity(FALSE, 
											TRUE, 
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
	
	return 0;	
}

static BOOL	
GCReadWriteBrightness(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
#if 0	
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uGcDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadGC(vin_priv->sensor_intf->u8SensorDevID, 0x55)&0xff);
	else
		DrvVideoIn_I2cWriteGC(vin_priv->sensor_intf->u8SensorDevID, 0x55, *pi32Value);
#endif		
	return TRUE;
}

static BOOL	
GCReadWriteContrast(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
#if 0		
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uGcDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadGC(vin_priv->sensor_intf->u8SensorDevID, 0x56)&0xff);
	else
		DrvVideoIn_I2cWriteGC(vin_priv->sensor_intf->u8SensorDevID, 0x56, *pi32Value);
#endif			
	return TRUE;
}

static BOOL	
GCReadWriteSharpness(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
#if 0		
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uGcDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadGC(vin_priv->sensor_intf->u8SensorDevID, 0x3f)&0xff);
	else
		DrvVideoIn_I2cWriteGC(vin_priv->sensor_intf->u8SensorDevID, 0x3f, *pi32Value);
#endif	
	return TRUE;
}


static BOOL	
GCReadWriteWhiteBalance(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
#if 0		
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uGcDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadGC(vin_priv->sensor_intf->u8SensorDevID, 0x6f)&0xff);
	else
		DrvVideoIn_I2cWriteGC(vin_priv->sensor_intf->u8SensorDevID, 0x6f, *pi32Value);
#endif	
	return TRUE;
}

static BOOL	
GCReadWriteNoiseReduction(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
#if 0		
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uGcDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadGC(vin_priv->sensor_intf->u8SensorDevID, 0x4c)&0xff);
	else
		DrvVideoIn_I2cWriteGC(vin_priv->sensor_intf->u8SensorDevID, 0x4c, *pi32Value);
#endif	
	return TRUE;
}

static BOOL	
GCReadWriteColorSaturation(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
#if 0		
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	ENTER();
	if(vin_priv->sensor_intf->u8SensorDevID != g_uGcDeviceID[0])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadGC(vin_priv->sensor_intf->u8SensorDevID, 0xc9)&0xff);
	else
		DrvVideoIn_I2cWriteGC(vin_priv->sensor_intf->u8SensorDevID, 0xc9, *pi32Value);
#endif	
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

	if(vin_priv->sensor_intf->u8SensorDevID != g_uGcDeviceID[0])
		return FALSE;
	DrvVideoIn_I2cWriteGC(vin_priv->sensor_intf->u8SensorDevID, u16SensRegAddr, c->value);
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
	if(vin_priv->sensor_intf->u8SensorDevID != g_uGcDeviceID[0])
		return FALSE;
	c->value = (DrvVideoIn_I2cReadGC(vin_priv->sensor_intf->u8SensorDevID, u16SensRegAddr)&0xff);
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

static NVT_SENSOR_T nvt_sensor_gc = {
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
	read_write_brightness:			GCReadWriteBrightness,
	read_write_contrast:			GCReadWriteContrast,
	read_write_sharpness:			GCReadWriteSharpness,
	read_write_white_balance:		GCReadWriteWhiteBalance,
	read_write_noise_reduction:		GCReadWriteNoiseReduction,
	read_write_color_saturation:	GCReadWriteColorSaturation,

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
	*sensor_intf = (NVT_SENSOR_T*)(&nvt_sensor_gc);
	return 0;
}
#endif 
//#ifdef CONFIG_W55FA93_VIDEOIN_DEV2
//INT32 register_vin_port2_Sensor(NVT_SENSOR_T **sensor_intf)
//{
	//*sensor_intf = (NVT_SENSOR_T*)(&nvt_sensor_gc;
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
