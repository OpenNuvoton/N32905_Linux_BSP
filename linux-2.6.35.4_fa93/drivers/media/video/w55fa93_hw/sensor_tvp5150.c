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

#include "sensor_tvp5150.h"

#define __STANDARD_I2C__
#define TVP5150_Initial 		0
#define	TVP5150_REG_VGA			1
#define	TVP5150_REG_SVGA		2
#define _REG_TABLE_SIZE(nTableName)	sizeof(nTableName)/sizeof(struct TVP5150_RegValue)

#define CONFIG_ARCH_W55FA93_DEMOBOARD
extern unsigned int w55fa93_FB_BG_PHY_ADDR;
extern unsigned int w55fa92_upll_clock, w55fa92_apll_clock, w55fa92_ahb_clock;


#define ERR_PRINTF	printk
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

#ifdef CONFIG_NTSC_SYSTEM_DEV1
#define CROP_START_X		CONFIG_NTSC_CROP_START_X	//0x4	//OK
#define CROP_START_Y		CONFIG_NTSC_CROP_START_Y	//0x6  
#endif
#ifdef CONFIG_PAL_SYSTEM_DEV1	
#define CROP_START_X		CONFIG_PAL_CROP_START_X		//0x4	//Need Verify
#define CROP_START_Y		CONFIG_PAL_CROP_START_Y		//0x6	 
#endif
/* NTSC mode crop start position (X, Y) = (4, 6) */

struct TVP5150_RegValue{
	__u8	uRegAddr;
	__u8	uValue;
};


struct TVP5150_RegTable{
	struct TVP5150_RegValue *sRegTable;
	__u16 uTableSize;
};

struct TVP5150_RegValue g_sTVP5150_Init[] = 
{
	{ /* 0x00 */
		TVP5150_VD_IN_SRC_SEL_1,0x00
	},
	{ /* 0x01 */
		TVP5150_ANAL_CHL_CTL,0x15
	},
	{ /* 0x02 */
		TVP5150_OP_MODE_CTL,0x00
	},
	{ /* 0x03 */
		TVP5150_MISC_CTL,0x09
	},
	{ /* 0x06 */
		TVP5150_COLOR_KIL_THSH_CTL,0x10
	},
	{ /* 0x07 */
		TVP5150_LUMA_PROC_CTL_1,0x20	// Luma signal delay ::  0 pixel clocks delay 0x20, 0x23, 0x28
	},
	{ /* 0x08 */
		TVP5150_LUMA_PROC_CTL_2,0x4C	//Luminance filter select: 0x48 comb filter disabled, chroma trap filter enable !!!!! 
	},
	{ /* 0x09 */
		TVP5150_BRIGHT_CTL,0x80
	},
	{ /* 0x0a */
		TVP5150_SATURATION_CTL,0xA0
	},
	{ /* 0x0b */
		TVP5150_HUE_CTL,0x00
	},
	{ /* 0x0c */
		TVP5150_CONTRAST_CTL,0x84
	},
	{ /* 0x0d */
		TVP5150_DATA_RATE_SEL,0x47
	},
	{ /* 0x0e */
		TVP5150_LUMA_PROC_CTL_3,0x00	//Luminance filter stop band bandwidth:: No notch
	},
	{ /* 0x0f */
		TVP5150_CONF_SHARED_PIN,0x08
	},
	{ /* 0x11 */
		TVP5150_ACT_VD_CROP_ST_MSB,0x00
	},
	{ /* 0x12 */
		TVP5150_ACT_VD_CROP_ST_LSB,0x00
	},
	{ /* 0x13 */
		TVP5150_ACT_VD_CROP_STP_MSB,0x00
	},
	{ /* 0x14 */
		TVP5150_ACT_VD_CROP_STP_LSB,0x00
	},
	{ /* 0x15 */
		TVP5150_GENLOCK,0x01
	},
	{ /* 0x16 */
		TVP5150_HORIZ_SYNC_START,0x80
	},
	{ /* 0x18 */
		TVP5150_VERT_BLANKING_START,0x00
	},
	{ /* 0x19 */
		TVP5150_VERT_BLANKING_STOP,0x00
	},
	{ /* 0x1a */
		TVP5150_CHROMA_PROC_CTL_1,0x0c
	},
	{ /* 0x1b */
		TVP5150_CHROMA_PROC_CTL_2,0x15	//Chrominance filter select: No notch = 0x10
	},
	{ /* 0x1c */
		TVP5150_INT_RESET_REG_B,0x00
	},
	{ /* 0x1d */
		TVP5150_INT_ENABLE_REG_B,0x00
	},
	{ /* 0x1e */
		TVP5150_INTT_CONFIG_REG_B,0x00
	},
	{ /* 0x2e */
		TVP5150_MACROVISION_ON_CTR,0x0f
	},
	{ /* 0x2f */
		TVP5150_MACROVISION_OFF_CTR,0x01
	},
	{ /* 0xbb */
		TVP5150_TELETEXT_FIL_ENA,0x00
	},
	{ /* 0xc0 */
		TVP5150_INT_STATUS_REG_A,0x00
	},
	{ /* 0xc1 */
		TVP5150_INT_ENABLE_REG_A,0x00
	},
	{ /* 0xc2 */
		TVP5150_INT_CONF,0x04
	},
	{ /* 0xc8 */
		TVP5150_FIFO_INT_THRESHOLD,0x80
	},
	{ /* 0xc9 */
		TVP5150_FIFO_RESET,0x00
	},
	{ /* 0xca */
		TVP5150_LINE_NUMBER_INT,0x00
	},
	{ /* 0xcb */
		TVP5150_PIX_ALIGN_REG_LOW,0x4e
	},
	{ /* 0xcc */
		TVP5150_PIX_ALIGN_REG_HIGH,0x00
	},
	{ /* 0xcd */
		TVP5150_FIFO_OUT_CTRL,0x01
	},
	{ /* 0xcf */
		TVP5150_FULL_FIELD_ENA,0x00
	},
	{ /* 0xd0 */
		TVP5150_LINE_MODE_INI,0x00
	},
	{ /* 0xfc */
		TVP5150_FULL_FIELD_MODE_REG,0x7f
	},
};

struct TVP5150_RegTable g_TVP5150_InitTable[] =
{
	{g_sTVP5150_Init,_REG_TABLE_SIZE(g_sTVP5150_Init)},	
	//{g_sTVP5150_Init,_REG_TABLE_SIZE(g_sTVP5150_Init)},	
	//{g_sTVP5150_Init,_REG_TABLE_SIZE(g_sTVP5150_Init)},	
};

__u8 g_uTVP5150DeviceID[]= 
{
	0xBA,		// tvp5150
	//0xBA,		// tvp5150
	//0xBA,		// tvp5150
};

static struct i2c_client *save_client;
static struct i2c_client i2c_client; 
static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret = 0;
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_I2C.\n");
		return -EIO;
	}
	memcpy(&i2c_client, client, sizeof(struct i2c_client));
	save_client = &i2c_client;
	return ret;
}

static int sensor_remove(struct i2c_client *client)
{
	client->driver = NULL;
	return 0;
}

static const struct i2c_device_id sensor_id[] = {

	{ "tvp5150", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name = "tvp5150",
	},
	.probe    = sensor_probe,
	.remove   = sensor_remove,
	.id_table = sensor_id,
};

#ifndef __STANDARD_I2C__
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
static void I2C_Delay(UINT32 u32Delay)
{
	volatile UINT32 i;
	for(;u32Delay!=0;u32Delay--)
		for(i=0;i<500;i++);
}
#endif 

static s8  DrvVideoIn_I2cWriteTVP5150(__u8 uAddr, __u16 uRegAddr, __u8 uData)
{
#ifdef __STANDARD_I2C__
		struct i2c_msg msg;
		u8 buf[2];
		int ret=-1;
		
		msg.flags=!I2C_M_RD;
		msg.addr=save_client->addr;
		msg.len=2;
		msg.buf=buf;		

		buf[0]=(u8)(uRegAddr&0xff);
		buf[1]=uData;

		ret=i2c_transfer(save_client->adapter,&msg,1);
		udelay(5);
		return ret;		
#else
	I2C_Write_8bitSlaveAddr_8bitReg_8bitData(uAddr, uRegAddr, uData);
#endif
	return TRUE;
}


static __s8  DrvVideoIn_I2cReadTVP5150(__u8 uAddr, __u16 uRegAddr)
{
#ifdef __STANDARD_I2C__
	struct i2c_msg msgs;
	int ret=-1;
	u8 buf[2];
	
	msgs.flags=!I2C_M_RD;
	msgs.addr=save_client->addr;
	msgs.len=1;
	msgs.buf=buf;
	buf[0]=(u8)(uRegAddr&0xff);

	ret=i2c_transfer(save_client->adapter,&msgs,1);
	
	msgs.flags=I2C_M_RD;
	msgs.addr=save_client->addr;
	msgs.len=1;
	msgs.buf=buf;

	ret=i2c_transfer(save_client->adapter,&msgs,1);
	return buf[0];
#else
	return I2C_Read_8bitSlaveAddr_8bitReg_8bitData(uAddr,uRegAddr);
#endif
}

static BOOL bIsI2CAdd = FALSE; 
static int i2c_init(void)
{
	int ret = 0;
#ifdef __STANDARD_I2C__	

	ENTER();
	ret = i2c_add_driver(&sensor_i2c_driver);	

	if(ret){
		ERRLEAVE();
	}	
	else{		
		bIsI2CAdd = TRUE; 
		printk("I2C added\n");
		LEAVE();
	}
#endif
	return ret;
}
static void i2c_cleanup(void)
{
#ifdef __STANDARD_I2C__	
	ENTER();
	i2c_del_driver(&sensor_i2c_driver);
	LEAVE();
#endif
}

#ifdef CONFIG_SHARE_SENSOR	
static UINT16 u16SensorWidth, u16SensorHeight;
static void SensorOutputResolution(UINT16 u16ImageWidth, 
			 	   UINT16 u16ImageHeight)
{
	u16SensorWidth = u16ImageWidth;
	u16SensorHeight = u16ImageHeight;
}
void GetVideo0SensorResoultion(UINT16* pSenWidth, UINT16* pSenHeight)
{
	*pSenWidth = u16SensorWidth;
	*pSenHeight = u16SensorHeight;
}
#endif


void TVP5150SetVideoformat(int std)
{
	__u8  uDeviceID;
	__u8  fmt = 0;
	__s8  ret;	
	
	uDeviceID  = g_uTVP5150DeviceID[TVP5150_Initial];
	
	/* First tests should be against specific std */
	if (std == TVP_STD_ALL) {
		fmt=0;	/* Autodetect mode */
	} else if (std & TVP_STD_NTSC_443) {
		fmt=0xa;
	} else if (std & TVP_STD_PAL_M) {
		fmt=0x6;
	} else if (std & (TVP_STD_PAL_N| TVP_STD_PAL_Nc)) {
		fmt=0x8;
	} else {
		/* Then, test against generic ones */
		if (std & TVP_STD_NTSC) {
			fmt=0x2;
		} else if (std & TVP_STD_PAL) {
			fmt=0x4;
		}
	}
	ERR_PRINTF("TVP5150 Video format : %02x. \r\n",fmt);
	ret = DrvVideoIn_I2cWriteTVP5150(uDeviceID, TVP5150_VIDEO_STD, fmt);
	if(ret<0)
		printk("Programminmg TVP5150 REG 0x%x fail\n", TVP5150_VIDEO_STD);
	else
		printk("Programminmg TVP5150 REG 0x%x successful\n", TVP5150_VIDEO_STD);
}

void TVP5150SetInputSource(int src)
{
	__u8  uDeviceID;
	__u8  inputsrc = 0;
	
	uDeviceID  = g_uTVP5150DeviceID[TVP5150_Initial];
	
	/* First tests should be against specific std */
	if (src == TVP5150_A1P1A) {
		inputsrc=0x00;	/* A1P1A mode */
		ERR_PRINTF("TVP5150 Video Input Source : %02x. TVP5150 Channel = A1P1A mode. \r\n",inputsrc);
	} else if (src == TVP5150_A1P1B) {
		inputsrc=0x02;	/* A1P1B mode */
		ERR_PRINTF("TVP5150 Video Input Source : %02x. TVP5150 Channel = A1P1B mode. \r\n",inputsrc);
	} else {
		inputsrc=0x01;	/* S-Video mode */
		ERR_PRINTF("TVP5150 Video Input Source : %02x. TVP5150 Channel = SVIDEO mode. \r\n",inputsrc);
	}

	DrvVideoIn_I2cWriteTVP5150(uDeviceID, TVP5150_VD_IN_SRC_SEL_1, inputsrc);
}

void TVP5150SetResolution(int index)
{
  	__u32 i;
	__u16 uTableSize;
	__u8  uDeviceID;
	
	struct TVP5150_RegValue *psRegValue;

	ERR_PRINTF("TVP5150_SetResolution:%d\n",index);

	if(index>TVP5150_REG_SVGA)
		return ;
	
	uTableSize = g_TVP5150_InitTable[index].uTableSize;
	psRegValue = g_TVP5150_InitTable[index].sRegTable;
	uDeviceID  = g_uTVP5150DeviceID[index];
	
	if ( psRegValue == 0 ){
		ERR_PRINTF("TVP5150_RegConfig psRegValue == 0");
		return;	
	}
	
	for(i=0;i<uTableSize; i++, psRegValue++)
	{
		udelay(10);
		DrvVideoIn_I2cWriteTVP5150(uDeviceID, (psRegValue->uRegAddr), (psRegValue->uValue));
	}
}

static void schedule_mdelay(UINT32 u32Delay_ms)
{
	unsigned long j=0;	
	j = jiffies + u32Delay_ms*HZ/1000; 	/* 2Xms~30ms */			
	while( time_before(jiffies,j) )
		schedule();	
}

static __s32 TVP5150RegConfig(__u32 nIndex)
{
  	__u32 i;
	__u16 uTableSize;
	__u8  uDeviceID;
	struct TVP5150_RegValue *psRegValue;
	__s32 res = 0; 
	__s8  ret; 	 

	ERR_PRINTF("%s\n",__FUNCTION__);	

#ifdef __STANDARD_I2C__	
	
#else	
	ERR_PRINTF("Non Standard I2C.\n");
	outp32(REG_GPBFUN, inp32(REG_GPBFUN)& ~(MF_GPB14|MF_GPB13));
	DrvI2C_Open(eDRVGPIO_GPIOB, 					
				eDRVGPIO_PIN13, 
				eDRVGPIO_GPIOB,
				eDRVGPIO_PIN14, 
				(PFN_DRVI2C_TIMEDELY)I2C_Delay); 
#endif
	
	uTableSize = g_TVP5150_InitTable[nIndex].uTableSize;
	psRegValue = g_TVP5150_InitTable[nIndex].sRegTable;
	uDeviceID  = g_uTVP5150DeviceID[nIndex];

#ifndef __STANDARD_I2C__		 
	ERR_PRINTF("uDeviceID = 0x%x\n", uDeviceID);
	ERR_PRINTF("REG_GPDFUN = 0x%x\n", inp32(REG_GPDFUN));
#endif

	if ( psRegValue == 0 ) {
		printk("Specified sensor table not exist\n");
		return -EBUSY;	
	}
	
	for(i=0;i<uTableSize; i++, psRegValue++)
	{
		__u8 u8Data;
		udelay(100);
		ret = DrvVideoIn_I2cWriteTVP5150(uDeviceID, (psRegValue->uRegAddr), (psRegValue->uValue));		
		u8Data = DrvVideoIn_I2cReadTVP5150(uDeviceID, (psRegValue->uRegAddr));	
		if(psRegValue->uRegAddr != 0xC0)
		{		
			if(u8Data != (psRegValue->uValue)){
				SDBG("Programminmg TVP5150 REG 0x%x fail\n", (psRegValue->uRegAddr));
				i=i-1;
				psRegValue = psRegValue-1;
			}
			else
				SDBG("Programminmg TVP5150 REG 0x%x successful\n", (psRegValue->uRegAddr));
		}
	}
	
#if defined(CONFIG_PAL_SYSTEM_DEV1)
	TVP5150SetVideoformat(TVP_STD_PAL);	
#elif defined(CONFIG_NTSC_SYSTEM_DEV1)
	TVP5150SetVideoformat(TVP_STD_NTSC);
#else		// default PAL
	TVP5150SetVideoformat(TVP_STD_PAL);	
#endif
	TVP5150SetInputSource(TVP5150_A1P1A);
	
	//dump_reg(uDeviceID);

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

	SDBG("Init Sensor Module TVP5150 in the first port \n");

	outp32(REG_GPDFUN, inp32(REG_GPDFUN)&(~MF_GPD2));
	w55fa93_gpio_set(GPIO_GROUP_D, 2, 1);    	/* Set high */
	w55fa93_gpio_set_output(GPIO_GROUP_D, 2);	/* Output */

	//gpio_setportval(GPIO_PORTD, 0x04, 0x04);    //GPIOD2 high to enable Amplifier 
	//gpio_setportpull(GPIO_PORTD, 0x04, 0x04);	//GPIOD2 pull high
	//gpio_setportdir(GPIO_PORTD, 0x04, 0x04);	//GPIOD2 output mode


#ifdef CONFIG_HV_FROM_GPB2_GPB3_DEV1
	vin_priv->pDevVin->Init(TRUE,                              // BOOL bIsEnableSnrClock,
                        0,                                      // E_DRVVIDEOIN_SNR_SRC eSnrSrc,        
                        24000,                                  // UINT32 u32SensorFreq,
                        eDrvVideoIn_2nd_TVD_CCIR656);   		// E_DRVVIDEOIN_DEV_TYPE eDevType
#endif
#ifdef CONFIG_HV_FROM_GPE0_GPE1_DEV1
    vin_priv->pDevVin->Init(TRUE,                                         
                        0,                                    
                        24000,                                  
	                    eDrvVideoIn_3rd_TVD_CCIR656);   		        
#endif

	if(vin_priv->sensor_intf->sensor_reset)
		vin_priv->sensor_intf->sensor_reset(0);	
	schedule_mdelay(10);
	i2c_init();
	schedule_mdelay(10);
							
	res = TVP5150RegConfig(TVP5150_Initial);
	if( res<0 ){
		printk("Sensor initial fail \n");
		return res;	
	}
	else
		printk("Sensor initial successful \n");

	vin_priv->pDevVin->Open(96000, 48000);

	vin_priv->sensor_intf->u8SensorDevID = g_uTVP5150DeviceID[TVP5150_Initial];//TVP5150_Initial
	vin_priv->sensor_intf->u16CurImgHeight = vin_priv->sensor_intf->u16MaxImgHeight;
	vin_priv->sensor_intf->u16CurImgWidth = vin_priv->sensor_intf->u16MaxImgWidth;

#ifdef CONFIG_SHARE_SENSOR											
	SensorOutputResolution(vin_priv->sensor_intf->u16CurImgWidth, vin_priv->sensor_intf->u16CurImgHeight);				
#endif

	if(vin_priv->videommap.format ==VIDEO_PALETTE_YUV420P)			
		vin_priv->pDevVin->SetPlanarFormat(TRUE);	/* Planar YUV420 */
	else	
		vin_priv->pDevVin->SetPlanarFormat(FALSE);	/* Planar YUV422 */		
	vin_priv->pDevVin->SetBaseStartAddress(eVIDEOIN_PACKET, 0, vin_priv->videoIn_preview_buf[0].u32PhysAddr);

	vin_priv->pDevVin->EnableInt(eVIDEOIN_VINT);
	vin_priv->pDevVin->SetSensorPolarity(FALSE, FALSE, TRUE);
	vin_priv->pDevVin->SetDataFormatAndOrder(eVIDEOIN_IN_VYUY, 
						 eVIDEOIN_IN_YUV422, 									
						 eVIDEOIN_OUT_YUV422);

	vin_priv->pDevVin->SetCropWinStartAddr(CROP_START_Y,	//UINT16 u16VerticalStart, 	Y
						CROP_START_X);	//UINT16 u16HorizontalStart, 	X
	/* Zooming Info */
	vin_priv->videocrop.c.left = CROP_START_X; 
	vin_priv->videocrop.c.top = CROP_START_Y;
	vin_priv->videocrop.c.width = vin_priv->sensor_intf->u16CurImgWidth; 
	vin_priv->videocrop.c.height = vin_priv->sensor_intf->u16CurImgHeight;
		
	vin_priv->videocropcap.bounds.left = CROP_START_X;
	vin_priv->videocropcap.bounds.top = CROP_START_Y;
	vin_priv->videocropcap.bounds.width = vin_priv->sensor_intf->u16CurImgWidth; 
	vin_priv->videocropcap.bounds.height = vin_priv->sensor_intf->u16CurImgHeight;
	vin_priv->videocropcap.defrect.left = CROP_START_X; 
	vin_priv->videocropcap.defrect.top = CROP_START_Y;
	vin_priv->videocropcap.defrect.width = vin_priv->sensor_intf->u16CurImgWidth; 
	vin_priv->videocropcap.defrect.height = vin_priv->sensor_intf->u16CurImgHeight;
	

	/* Sensor subsample resolution (640, 480)*/
	vin_priv->pDevVin->SetCropWinSize(vin_priv->sensor_intf->u16CurImgHeight, vin_priv->sensor_intf->u16CurImgWidth);
	vin_priv->pDevVin->PreviewPipeSize(vin_priv->videowin.height, vin_priv->videowin.width);	
	
	vin_priv->pDevVin->GetStride(&u32PacStride, &u32PlaStride);
	vin_priv->pDevVin->SetStride(vin_priv->videowin.width, u32PlaStride);
#ifdef CONFIG_TWO_FIELDS	
	vin_priv->pDevVin->SetInputType(3, eVIDEOIN_TYPE_CCIR656, NULL);
#endif
#ifdef CONFIG_ONE_FIELD
	vin_priv->pDevVin->SetInputType(1, eVIDEOIN_TYPE_CCIR656, NULL);
#endif
	vin_priv->pDevVin->SetPipeEnable(FALSE,				// It means planar disable
					 eVIDEOIN_PACKET);		//	

	vin_priv->pDevVin->SetShadowRegister();

	return 0;	
}

static BOOL PoweronSensor(BOOL bIsPowerOn)
{
	if(bIsPowerOn == TRUE){		
		/* do after Open() */
	}else{
		if(bIsI2CAdd==TRUE){
			i2c_cleanup();
			printk("I2C removed\n");
			bIsI2CAdd = FALSE;	
		}
	}
	return TRUE;	
}

BOOL	
TVP5150ReadWriteBrightness(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uTVP5150DeviceID[TVP5150_Initial])
		return FALSE;
	
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadTVP5150(vin_priv->sensor_intf->u8SensorDevID, TVP5150_BRIGHT_CTL)&0xff);
	else
		DrvVideoIn_I2cWriteTVP5150(vin_priv->sensor_intf->u8SensorDevID, TVP5150_BRIGHT_CTL, *pi32Value);

	return TRUE;
}

BOOL	
TVP5150ReadWriteContrast(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uTVP5150DeviceID[TVP5150_Initial])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadTVP5150(vin_priv->sensor_intf->u8SensorDevID, TVP5150_CONTRAST_CTL)&0xff);
	else
		DrvVideoIn_I2cWriteTVP5150(vin_priv->sensor_intf->u8SensorDevID, TVP5150_CONTRAST_CTL, *pi32Value);

	return TRUE;
}

BOOL	
TVP5150ReadWriteColorSaturation(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uTVP5150DeviceID[TVP5150_Initial])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cReadTVP5150(vin_priv->sensor_intf->u8SensorDevID, TVP5150_SATURATION_CTL)&0xff);
	else
		DrvVideoIn_I2cWriteTVP5150(vin_priv->sensor_intf->u8SensorDevID, TVP5150_SATURATION_CTL, *pi32Value);

	return TRUE;
}

typedef struct
{
	UINT16 u16ImageWidth; 
	UINT16 u16ImageHeight;
	UINT8 i8ResolIdx;
}S_TVP5150SuppResol;

#define TVP5150_RESOL_SUPP_CNT  1

S_TVP5150SuppResol s_asTVP5150SuppResolTable[TVP5150_RESOL_SUPP_CNT] = {
#ifdef CONFIG_TWO_FIELDS
	{640, 480, TVP5150_REG_VGA},	/* Capture VGA, JPEG encode VGA */
#endif
#ifdef CONFIG_ONE_FIELD
	{640, 240, TVP5150_REG_VGA},	/* Capture Half VGA, JPEG encode upscale to VGA */
#endif
	
};



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
#define V4L2_CID_PRIVATE_I2C_SET_REG_ADDR     		(V4L2_CID_PRIVATE_BASE + 0)
#define V4L2_CID_PRIVATE_I2C_WRITE     			(V4L2_CID_PRIVATE_BASE + 1)
#define V4L2_CID_PRIVATE_I2C_READ     			(V4L2_CID_PRIVATE_BASE + 2)
#define V4L2_CID_PRIVATE_GET_SENSOR_CLOCK		(V4L2_CID_PRIVATE_BASE + 3)	
#define V4L2_CID_PRIVATE_SET_SENSOR_CLOCK		(V4L2_CID_PRIVATE_BASE + 4)
#define V4L2_CID_PRIVATE_LASTP1     			(V4L2_CID_PRIVATE_BASE + 5)

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
		.maximum       = 65535,
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
	},
	{
		.id            = V4L2_CID_PRIVATE_GET_SENSOR_CLOCK,
		.name          = "get_sensor_clock",
		.minimum       = 12000000,
		.maximum       = 24000000,
		.step          	= 1000000,
		.type          = V4L2_CTRL_TYPE_INTEGER,
	},
	{
		.id            = V4L2_CID_PRIVATE_SET_SENSOR_CLOCK,
		.name          = "set_sensor_clock",
		.minimum       = 12000000,
		.maximum       = 24000000,
		.step          	= 1000000,
		.type          = V4L2_CTRL_TYPE_INTEGER,
	},

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

	if(vin_priv->sensor_intf->u8SensorDevID != g_uTVP5150DeviceID[TVP5150_Initial])
		return FALSE;
	DrvVideoIn_I2cWriteTVP5150(vin_priv->sensor_intf->u8SensorDevID, u16SensRegAddr, c->value);
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
	if(vin_priv->sensor_intf->u8SensorDevID != g_uTVP5150DeviceID[TVP5150_Initial])
		return FALSE;
	c->value = (DrvVideoIn_I2cReadTVP5150(vin_priv->sensor_intf->u8SensorDevID, u16SensRegAddr)&0xff);
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
			printk("i2c read fail\n");	
			return -EINVAL;	/* I2c read fail */
		}	
		break;
	case V4L2_CID_PRIVATE_I2C_SET_REG_ADDR:
		c->value = u16SensRegAddr;
		break;
	case V4L2_CID_PRIVATE_GET_SENSOR_CLOCK:
		printk("get sensor clock successfully.\n");	
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
			printk("i2c write fail\n");	
			return -EINVAL;	/* I2c write fail */
		}
		break;	

	case V4L2_CID_PRIVATE_I2C_SET_REG_ADDR:
		u16SensRegAddr  = c->value;
		printk("Specified sensor addr = 0x%x\n", u16SensRegAddr);
		break;
	case V4L2_CID_PRIVATE_SET_SENSOR_CLOCK:
		printk("set sensor clock successfully.\n");	
		break;		
	default:
		return -EINVAL;
	}
	return 0;
}



#if 0
/*================================================== V4L2 User Ctrl ================================================== */
#endif 
static NVT_SENSOR_T nvt_sensor_tvp5150 = {
	sensor_init:					InitSensor,
	sensor_poweron:					PoweronSensor,
#ifdef CONFIG_SENSOR_POWER_DOWN
	sensor_suspend:					PowerdownSensor,
#else
	sensor_suspend:					NULL,
#endif
#ifdef CONFIG_SENSOR_RESET
	sensor_reset:					ResetSensor,
#else	
	sensor_reset:					NULL,
#endif

	read_write_brightness:			TVP5150ReadWriteBrightness,
	read_write_contrast:			TVP5150ReadWriteContrast,
	read_write_sharpness:			NULL,
	read_write_white_balance:		NULL,
	read_write_noise_reduction:		NULL,
	read_write_color_saturation:	TVP5150ReadWriteColorSaturation, 

	query_private_user_ctrl:	SensorUserPrivateCtrl,    /* OK */
	sensor_i2c_setRegAddr:		SensorI2cSetRegAddr, 	/* OK */
	sensor_set_ctrl:			SensorI2cWriteCtrl,
	sensor_get_ctrl:			SensorI2cReadCtrl,

	change_image_resolution: 	NULL, 
	set_flicker_freq:			NULL, 
	low_lux_detect:				NULL, 
	control_IR_led:				NULL, 
#ifdef CONFIG_TWO_FIELDS
	u16MaxImgHeight:			480,		 
	u16MaxImgWidth: 			640,
#endif
#ifdef CONFIG_ONE_FIELD
	u16MaxImgHeight:			240,		 
	u16MaxImgWidth: 			640,
#endif
};

#ifdef CONFIG_W55FA93_VIDEOIN_DEV1
INT32 register_vin_port1_Sensor(NVT_SENSOR_T **sensor_intf)
{
	*sensor_intf = (NVT_SENSOR_T*)(&nvt_sensor_tvp5150);
	return 0;
}
#endif 


