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

#include <linux/videodev2.h>
#include <media/v4l2-device.h>

#include <linux/jiffies.h>
#include <mach/w55fa93_reg.h>
#include <mach/fb.h>
#include <mach/w55fa93_fb.h>
#include <mach/videoin.h>
//#include <mach/DrvVideoin.h>
#include "DrvVideoin.h"
#include <mach/videodev_ex.h>
#include <mach/w55fa93_gpio.h>

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
 

#define _REG_TABLE_SIZE(nTableName)	sizeof(nTableName)/sizeof(struct TW9912_RegValue)

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
#define COLLEX_BOARD

#define 	TW9912_INITIAL 		0
#define	TW9912_NTSC		1
#define	TW9912_PAL		2


#ifdef CONFIG_NTSC_SYSTEM_DEV1
#define CROP_START_X		CONFIG_NTSC_CROP_START_X	//0x2A 
#define CROP_START_Y		CONFIG_NTSC_CROP_START_Y	//0x25 
#endif
#ifdef CONFIG_PAL_SYSTEM_DEV1	
#define CROP_START_X		CONFIG_PAL_CROP_START_X		//0x2F 
#define CROP_START_Y		CONFIG_PAL_CROP_START_Y		//0x24 
#endif

//extern videoIn_buf_t videoIn_preview_buf[];

struct TW9912_RegValue{
	__u8	uRegAddr;
	__u8	uValue;
};

struct TW9912_RegTable{
	struct TW9912_RegValue *sRegTable;
	__u16 uTableSize;
};


static struct TW9912_RegValue g_sTW9912_Init[] = 
{
	#include "TW9912/TW9912_Init.dat"
};

static struct TW9912_RegValue g_sTW9912_NTSC[] = 
{
	#include "TW9912/TW9912_NTSC.dat"
};

static struct TW9912_RegValue g_sTW9912_PAL[] = 
{
	#include "TW9912/TW9912_PAL.dat"
};

static struct TW9912_RegTable g_TW9912_InitTable[] =
{
	{g_sTW9912_Init,_REG_TABLE_SIZE(g_sTW9912_Init)},	
	{g_sTW9912_NTSC,_REG_TABLE_SIZE(g_sTW9912_NTSC)},	
	{g_sTW9912_PAL,_REG_TABLE_SIZE(g_sTW9912_PAL)},	
};

__u8 g_uTW9912DeviceID[]= 
{
#if 0
#ifdef __STANDARD_I2C__
	0x45,		// tw9912
	0x45,		// tw9912
	0x45,		// tw9912	
#else	
	0x8A,		// tw9912
	0x8A,		// tw9912
	0x8A,		// tw9912
#endif
#endif

#ifdef __STANDARD_I2C__
	0x44,		// tw9912
	0x44,		// tw9912
	0x44,		// tw9912	
#else	
	0x88,		// tw9912
	0x88,		// tw9912
	0x88,		// tw9912
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
#if 0	
	{ "tw9912", 0x45 },
	{ }
#endif

	{ "tw9912", 0x44 },
	{ }

};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
#ifdef CONFIG_SENSOR_TW9912_DEV1
	.driver = {
		.name = "tw9912",
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

s8  DrvVideoIn_I2cWriteTW9912(__u8 uAddr, __u8 uRegAddr, __u8 uData)
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

__s8  DrvVideoIn_I2cReadTW9912(__u8 uAddr, __u8 uRegAddr)
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


void TW9912SetVideoformat(int fmtid)
{
	__u16 i;
	__u16 uTableSize;
	__u8  uDeviceID;
	struct TW9912_RegValue *psRegValue;

	uTableSize = g_TW9912_InitTable[fmtid].uTableSize;
	psRegValue = g_TW9912_InitTable[fmtid].sRegTable;
	uDeviceID  = g_uTW9912DeviceID[fmtid];	

	for(i=0;i<uTableSize; i++, psRegValue++)
	{
		DrvVideoIn_I2cWriteTW9912(uDeviceID, (psRegValue->uRegAddr), (psRegValue->uValue));
	}

	DBG_PRINTF("TW9912 Video format : %02x\r\n",fmtid);
}

static __s32 TW9912RegConfig(__u32 nIndex)
{
	__u32 i;
	__u16 uTableSize;
	__u8  uDeviceID;
	__s32 res = 0; 
	__u8  RetValue;
	
	struct TW9912_RegValue *psRegValue;
	ENTER();

#ifdef __STANDARD_I2C__
	printk("Standard I2C.\n");	
#else	
	printk("Non Standard I2C.\n");
	#if 0
	outp32(REG_GPBFUN, inp32(REG_GPBFUN)& ~(MF_GPB14|MF_GPB13));
	DrvI2C_Open(eDRVGPIO_GPIOB, 					
				eDRVGPIO_PIN13, 
				eDRVGPIO_GPIOB,
				eDRVGPIO_PIN14, 
				(PFN_DRVI2C_TIMEDELY)I2C_Delay);
	#endif

	DBG_PRINTF("REG_GPDFUN = 0x%x\n", inp32(REG_GPDFUN));
	//CLK-GPD15 SDA-GPD14
	DrvI2C_Open(eDRVGPIO_GPIOD, 					
				eDRVGPIO_PIN15, 
				eDRVGPIO_GPIOD,
				eDRVGPIO_PIN14, 
				(PFN_DRVI2C_TIMEDELY)I2C_Delay);
#endif

	//SDBG("nIndex = %d\n", nIndex);	
	if ( nIndex >= (sizeof(g_uTW9912DeviceID)/sizeof(__u8)) ){
		printk("Specified sensor not exist\n");
		return -EBUSY;	
	}
	uTableSize = g_TW9912_InitTable[nIndex].uTableSize;
	psRegValue = g_TW9912_InitTable[nIndex].sRegTable;
	uDeviceID = g_uTW9912DeviceID[nIndex];

	if ( psRegValue == 0 ){
		printk("Specified sensor table not exist\n");
		return -EBUSY;	
	}
	
	if(1)//for test
	{
		RetValue=DrvVideoIn_I2cReadTW9912(uDeviceID,0x00);
		printk("tw9912 reg[0x00,0x60]=0x%x\n", RetValue);
	}
	
	for(i=0;i<uTableSize; i++, psRegValue++)
	{
		int32_t ret;
		udelay(100);	
		ret = DrvVideoIn_I2cWriteTW9912(uDeviceID, (psRegValue->uRegAddr), (psRegValue->uValue));
		if(ret<0)
		{
			printk("Wrong to write register addr = 0x%x, write data = 0x%x , ret = %d\n", (psRegValue->uRegAddr), (psRegValue->uValue), ret);		
		}else{
			printk("Successful to write register addr = 0x%x, write data = 0x%x\n", (psRegValue->uRegAddr), (psRegValue->uValue));	
		}
	} 
	if(res>=0)
		printk("driver i2c initial done\n");
	else
		printk("driver i2c initial fail\n");	

#if defined(CONFIG_SENSOR_TW9912_PAL)
	TW9912SetVideoformat(TW9912_PAL);
#elif defined(CONFIG_SENSOR_TW9912_NTSC)
	TW9912SetVideoformat(TW9912_NTSC);
#else		// default NTSC
	TW9912SetVideoformat(TW9912_NTSC);
#endif	
	
	return res;
}


void SnrReset(void)
{
	/* GPE9 reset:	H->L->H */	
	DBG_PRINTF("%s\n",__FUNCTION__);		
	//while(1);
	w55fa93_gpio_configure(GPIO_GROUP_E, 9);
	w55fa93_gpio_set(GPIO_GROUP_E, 9, 0);
	w55fa93_gpio_set_output(GPIO_GROUP_E, 9);
	mdelay(50);
	w55fa93_gpio_set(GPIO_GROUP_E, 9, 0);	//GPIOE 9 set low
	mdelay(50);
	w55fa93_gpio_set(GPIO_GROUP_E, 9, 1);	//GPIOE 9 set high
}

void SnrPowerDown(BOOL bIsEnable)
{
	#if 0
	/* GPD12 power down, Low for power down */
	DBG_PRINTF("%s\n",__FUNCTION__);	
	w55fa93_gpio_configure(GPIO_GROUP_D, 12);
	w55fa93_gpio_set(GPIO_GROUP_D, 12, 0);
	w55fa93_gpio_set_output(GPIO_GROUP_D, 12);
	if(bIsEnable)
		w55fa93_gpio_set(GPIO_GROUP_D, 12, 0);		//GPIOD 12 set low
	else			
		w55fa93_gpio_set(GPIO_GROUP_D, 12, 1);	//GPIOD 12 set high	
	#endif
}

void SnrPoweronSet(BOOL bIsEnable)
{
	#if 1
	/*Set GPIOB0*/
	DBG_PRINTF("%s\n",__FUNCTION__);
	w55fa93_gpio_configure(GPIO_GROUP_B, 0);
	w55fa93_gpio_set_output(GPIO_GROUP_B, 0);
	w55fa93_gpio_set(GPIO_GROUP_B, 0, 1);	//GPIOB 0 set high
	#endif
}


static __s32 InitSensor(__u32 u32Sensor, void *priv)
{
	__u32 u32PacStride, u32PlaStride;
	__s32 res = 0; 
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;

	ENTER();

#ifdef CONFIG_HV_FROM_GPE0_GPE1_DEV1
	outp32(REG_GPDFUN, inp32(REG_GPDFUN) &~(MF_GPD2));
	w55fa93_gpio_configure(GPIO_GROUP_D, 2);
	w55fa93_gpio_set_output(GPIO_GROUP_D, 2);
	w55fa93_gpio_set(GPIO_GROUP_D, 2, 1);		//GPIOD2 set high
#endif
	
	printk("Init TW9912 \n"); 		


    vin_priv->pDevVin->Init(TRUE,                                         
                        0,                                    
                        24000,                                  
                        eDrvVideoIn_3rd_SNR_CCIR601);   		        

	vin_priv->pDevVin->Open(72000, 24000);	
	//vin_priv->pDevVin->Open(48000, 24000);	
	
	SnrPoweronSet(TRUE);

	SnrPowerDown(FALSE);

  	SnrReset();

	res = TW9912RegConfig(0);
	if( res<0 ){
		printk("Sensor initial fail \n");
		return res;	
	}
	else
		printk("Sensor initial successful \n");
	
	vin_priv->sensor_intf->u8SensorDevID = g_uTW9912DeviceID[0];
	vin_priv->sensor_intf->u16CurImgHeight = vin_priv->sensor_intf->u16MaxImgHeight;
	vin_priv->sensor_intf->u16CurImgWidth = vin_priv->sensor_intf->u16MaxImgWidth;

	vin_priv->videocrop.c.left = CROP_START_X;	/*X*/
	vin_priv->videocrop.c.top = CROP_START_Y;	/*Y*/	
	vin_priv->videocrop.c.width = vin_priv->sensor_intf->u16CurImgWidth;
	vin_priv->videocrop.c.height = vin_priv->sensor_intf->u16CurImgHeight;	

	vin_priv->videocropcap.bounds.left = CROP_START_X; 
	vin_priv->videocropcap.bounds.top = CROP_START_Y;
	vin_priv->videocropcap.bounds.width = vin_priv->sensor_intf->u16CurImgWidth; 
	vin_priv->videocropcap.bounds.height = vin_priv->sensor_intf->u16CurImgHeight;

	vin_priv->videocropcap.defrect.left = CROP_START_X; 
	vin_priv->videocropcap.defrect.top = CROP_START_Y;
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
											FALSE, 
											TRUE);

	vin_priv->pDevVin->SetInputType(0,eVIDEOIN_TYPE_CCIR656,FALSE);	

	vin_priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_VYUY, 
									eVIDEOIN_IN_YUV422, 									
									eVIDEOIN_OUT_YUV422);	
	
	vin_priv->pDevVin->SetCropWinStartAddr(CROP_START_Y,					//UINT16 u16VerticalStart, 		Y
									CROP_START_X);					//UINT16 u16HorizontalStart, 	X

	/* Sensor subsample resolution (640, 480)*/
	vin_priv->pDevVin->SetCropWinSize(vin_priv->sensor_intf->u16MaxImgHeight,		//UINT16 u16Height, 
							 vin_priv->sensor_intf->u16MaxImgWidth);		//UINT16 u16Width;									 
	
	vin_priv->pDevVin->PreviewPipeSize(vin_priv->videowin.height, vin_priv->videowin.width);
	
	vin_priv->pDevVin->GetStride(&u32PacStride, &u32PlaStride);
	vin_priv->pDevVin->SetStride(vin_priv->videowin.width, u32PlaStride);															
	vin_priv->pDevVin->SetFrameRateScaleFactor(1, 2);	
	vin_priv->pDevVin->SetPipeEnable(FALSE,							// It means planar disable
								eVIDEOIN_PACKET);
		
	vin_priv->pDevVin->SetShadowRegister();
	printk("REG-04 = 0x%x\n", inp32(REG_VPEPAR));

	
	return 0;	
}

static BOOL	
OVReadWriteBrightness(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	#define TW9912_BRIGHT_CTL 0x10
	
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uTW9912DeviceID[TW9912_INITIAL])
		return FALSE;
	
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadTW9912(vin_priv->sensor_intf->u8SensorDevID, TW9912_BRIGHT_CTL)&0xff);
	else
		DrvVideoIn_I2cWriteTW9912(vin_priv->sensor_intf->u8SensorDevID, TW9912_BRIGHT_CTL, *pi32Value);

	return TRUE;

}

static BOOL	
OVReadWriteContrast(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	#define TW9912_CONTRAST_CTL 0x11
	
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uTW9912DeviceID[TW9912_INITIAL])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadTW9912(vin_priv->sensor_intf->u8SensorDevID, TW9912_CONTRAST_CTL)&0xff);
	else
		DrvVideoIn_I2cWriteTW9912(vin_priv->sensor_intf->u8SensorDevID, TW9912_CONTRAST_CTL, *pi32Value);

	return TRUE;
}

static BOOL	
OVReadWriteSharpness(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	#define TW9912_SHARPNESS_CTL 0x12
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uTW9912DeviceID[TW9912_INITIAL])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadTW9912(vin_priv->sensor_intf->u8SensorDevID, TW9912_SHARPNESS_CTL)&0xff);
	else
		DrvVideoIn_I2cWriteTW9912(vin_priv->sensor_intf->u8SensorDevID, TW9912_SHARPNESS_CTL, *pi32Value);

	return TRUE;
}


static BOOL	
OVReadWriteWhiteBalance(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
#if 0
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uTW9912DeviceID[TW9912_INITIAL])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadTW9912(vin_priv->sensor_intf->u8SensorDevID, 0x6f)&0xff);
	else
		DrvVideoIn_I2cWriteTW9912(vin_priv->sensor_intf->u8SensorDevID, 0x6f, *pi32Value);
#endif
	return TRUE;
}

static BOOL	
OVReadWriteNoiseReduction(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
#if 0
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uTW9912DeviceID[TW9912_INITIAL])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadTW9912(vin_priv->sensor_intf->u8SensorDevID, 0x4c)&0xff);
	else
		DrvVideoIn_I2cWriteTW9912(vin_priv->sensor_intf->u8SensorDevID, 0x4c, *pi32Value);
#endif
	return TRUE;
}

static BOOL	
OVReadWriteColorSaturation(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
#if 0
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uTW9912DeviceID[TW9912_INITIAL])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadTW9912(vin_priv->sensor_intf->u8SensorDevID, TW9912_SATURATION_CTL)&0xff);
	else
		DrvVideoIn_I2cWriteTW9912(vin_priv->sensor_intf->u8SensorDevID, TW9912_SATURATION_CTL, *pi32Value);
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

static UINT8 u8SensRegAddr = 0; 
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

	if(vin_priv->sensor_intf->u8SensorDevID != g_uTW9912DeviceID[0])
		return FALSE;
	DrvVideoIn_I2cWriteTW9912(vin_priv->sensor_intf->u8SensorDevID, u8SensRegAddr, c->value);
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
	if(vin_priv->sensor_intf->u8SensorDevID != g_uTW9912DeviceID[0])
		return FALSE;
	c->value = (DrvVideoIn_I2cReadTW9912(vin_priv->sensor_intf->u8SensorDevID, u8SensRegAddr)&0xff);
	return TRUE;
}

static INT32 
SensorI2cSetRegAddr(
	void *priv, 
	struct v4l2_control *c
)
{
	u8SensRegAddr  = c->value;
	SDBG("Specified sensor addr = 0x%x\n", u8SensRegAddr);
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
		c->value = u8SensRegAddr;
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
		u8SensRegAddr  = c->value;
		printk("Specified sensor addr = 0x%x\n", u8SensRegAddr);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
#ifdef CONFIG_VGA_RESOLUTION
	#define VIN_DEV1_ENCODE_WIDTH		640
	#define VIN_DEV1_ENCODE_HEIGHT		480	
#elif defined  CONFIG_D1_RESOLUTION
	#define VIN_DEV1_ENCODE_WIDTH		720
	#define VIN_DEV1_ENCODE_HEIGHT		480	
#endif

#if 0
/*================================================== V4L2 User Ctrl ================================================== */
#endif 

static NVT_SENSOR_T nvt_sensor_ov = {
	sensor_init:				InitSensor,
	sensor_poweron:				NULL,

#ifdef CONFIG_SENSOR_POWER_DOWN
	sensor_suspend:				SnrPowerDown,
#else
	sensor_suspend:				NULL,
#endif
#ifdef CONFIG_SENSOR_RESET
	sensor_reset:				SnrReset,
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

	u16MaxImgHeight:				VIN_DEV1_ENCODE_HEIGHT,		 
	u16MaxImgWidth: 				VIN_DEV1_ENCODE_WIDTH,
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











