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


#define __STANDARD_I2C__
#define TW9900_Initial 		0
#define	TW9900_REG_VGA		0

#define TW9900_NTSC			1	
#define TW9900_PAL			2

#define _REG_TABLE_SIZE(nTableName)	sizeof(nTableName)/sizeof(struct TW9900_RegValue)

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

struct TW9900_RegValue{
	__u8	uRegAddr;
	__u8	uValue;
};


struct TW9900_RegTable{
	struct TW9900_RegValue *sRegTable;
	__u16 uTableSize;
};

struct TW9900_RegValue g_sTW9900_Init[] = 
{
	#include "TW9900/TW9900_Init.dat"
};

static struct TW9900_RegValue g_sTW9900_NTSC[]=
{
	#include "TW9900/TW9900_NTSC.dat"
};
static struct TW9900_RegValue g_sTW9900_PAL[]=
{
	#include "TW9900/TW9900_PAL.dat"
};

struct TW9900_RegTable g_TW9900_InitTable[] =
{
	{g_sTW9900_Init,_REG_TABLE_SIZE(g_sTW9900_Init)},	
	{g_sTW9900_NTSC, _REG_TABLE_SIZE(g_sTW9900_NTSC)},	
	{g_sTW9900_PAL,_REG_TABLE_SIZE(g_sTW9900_PAL)},
};

__u8 g_uTW9900DeviceID[]= 
{
	0x8A,		// TW9900
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

	{ "tw9900", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name = "tw9900",
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

static s8  DrvVideoIn_I2cWrite(__u8 uAddr, __u16 uRegAddr, __u8 uData)
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


static __s8  DrvVideoIn_I2cRead(__u8 uAddr, __u16 uRegAddr)
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

void TW9900SetResolution(int index)
{
  	__u32 i;
	__u16 uTableSize;
	__u8  uDeviceID;
	
	struct TW9900_RegValue *psRegValue;

	ERR_PRINTF("TW9900SetResolution:%d\n",index);

	if(index>TW9900_REG_VGA)
		return ;
	
	uTableSize = g_TW9900_InitTable[index].uTableSize;
	psRegValue = g_TW9900_InitTable[index].sRegTable;
	uDeviceID  = g_uTW9900DeviceID[index];
	
	if ( psRegValue == 0 ){
		ERR_PRINTF("TW9900_RegConfig psRegValue == 0");
		return;	
	}
	
	for(i=0;i<uTableSize; i++, psRegValue++)
	{
		udelay(10);
		DrvVideoIn_I2cWrite(uDeviceID, (psRegValue->uRegAddr), (psRegValue->uValue));
	}
}

static void schedule_mdelay(UINT32 u32Delay_ms)
{
	unsigned long j=0;	
	j = jiffies + u32Delay_ms*HZ/1000; 	/* 2Xms~30ms */			
	while( time_before(jiffies,j) )
		schedule();	
}

static __s32 SensorRegConfig(__u32 nIndex)
{
  	__u32 i;
	__u16 uTableSize;
	__u8  uDeviceID, uProductID;
	struct TW9900_RegValue *psRegValue;
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
	
	uTableSize = g_TW9900_InitTable[nIndex].uTableSize;
	psRegValue = g_TW9900_InitTable[nIndex].sRegTable;
	uDeviceID  = g_uTW9900DeviceID[nIndex];

#ifndef __STANDARD_I2C__		 
	ERR_PRINTF("uDeviceID = 0x%x\n", uDeviceID);
	ERR_PRINTF("REG_GPDFUN = 0x%x\n", inp32(REG_GPDFUN));
#endif

	if ( psRegValue == 0 ) {
		printk("Specified sensor table not exist\n");
		return -EBUSY;	
	}
	
	for(i=0;i<uTableSize; i++, psRegValue++){
		DrvVideoIn_I2cWrite(uDeviceID, (psRegValue->uRegAddr), (psRegValue->uValue));		
		udelay(100);
	}
	
#if defined(CONFIG_PAL_SYSTEM_DEV1)
	uTableSize = g_TW9900_InitTable[TW9900_PAL].uTableSize;
	psRegValue = g_TW9900_InitTable[TW9900_PAL].sRegTable;
#elif defined(CONFIG_NTSC_SYSTEM_DEV1)
	uTableSize = g_TW9900_InitTable[TW9900_NTSC].uTableSize;
	psRegValue = g_TW9900_InitTable[TW9900_NTSC].sRegTable;
#endif
	for(i=0;i<uTableSize; i++, psRegValue++){
		DrvVideoIn_I2cWrite(uDeviceID, (psRegValue->uRegAddr), (psRegValue->uValue));		
		udelay(100);
	}
	uProductID = DrvVideoIn_I2cRead(uDeviceID, 0x0); //Device ID will read back 0x60
	printk("Sensor Product ID0 = 0x%x\n", uProductID);
	for(i=0;i<0x6F;i++)
	{
		uProductID = DrvVideoIn_I2cRead(uDeviceID, i); //Device ID will read back 0x60
		printk("0x%02x = 0x%02x\n", i, uProductID);
	}
	return res;	
}

static __s32 InitSensor(__u32 u32Sensor, void *priv)
{
	__u32 u32PacStride, u32PlaStride;
	__s32 res = 0; 
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;

	SDBG("Init Sensor Module TW9900 in the first port \n");

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
							
	res = SensorRegConfig(TW9900_Initial);
	if( res<0 ){
		printk("Sensor initial fail \n");
		return res;	
	}
	else
		printk("Sensor initial successful \n");

	vin_priv->pDevVin->Open(96000, 48000);

	vin_priv->sensor_intf->u8SensorDevID = g_uTW9900DeviceID[TW9900_Initial];//TW9900_Initial
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
	vin_priv->pDevVin->SetSensorPolarity(FALSE, TRUE, TRUE);
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
	vin_priv->pDevVin->SetInputType(3, eVIDEOIN_TYPE_CCIR656, 0);
#endif
#ifdef CONFIG_ONE_FIELD
	vin_priv->pDevVin->SetInputType(1, eVIDEOIN_TYPE_CCIR656, 0);
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

#define TW9900_BRIGHT_CTL			0x10
#define TW9900_CONTRAST_CTL			0x11
#define TW9900_SATURATION_CTL		0x13
	
BOOL	
TW9900ReadWriteBrightness(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uTW9900DeviceID[TW9900_Initial])
		return FALSE;
	
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cRead(vin_priv->sensor_intf->u8SensorDevID, TW9900_BRIGHT_CTL)&0xff);
	else
		DrvVideoIn_I2cWrite(vin_priv->sensor_intf->u8SensorDevID, TW9900_BRIGHT_CTL, *pi32Value);

	return TRUE;
}

BOOL	
TW9900ReadWriteContrast(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uTW9900DeviceID[TW9900_Initial])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cRead(vin_priv->sensor_intf->u8SensorDevID, TW9900_CONTRAST_CTL)&0xff);
	else
		DrvVideoIn_I2cWrite(vin_priv->sensor_intf->u8SensorDevID, TW9900_CONTRAST_CTL, *pi32Value);

	return TRUE;
}

BOOL	
TW9900ReadWriteColorSaturation(
	void *priv, 
	INT32 *pi32Value, 
	BOOL bIsRead
)
{
	videoin_priv_t *vin_priv = (videoin_priv_t *) priv;
	if(vin_priv->sensor_intf->u8SensorDevID != g_uTW9900DeviceID[TW9900_Initial])
		return FALSE;
	if(bIsRead)
		*pi32Value = (DrvVideoIn_I2cRead(vin_priv->sensor_intf->u8SensorDevID, TW9900_SATURATION_CTL)&0xff);
	else{
		DrvVideoIn_I2cWrite(vin_priv->sensor_intf->u8SensorDevID, TW9900_SATURATION_CTL, *pi32Value);		//U Saturation
		DrvVideoIn_I2cWrite(vin_priv->sensor_intf->u8SensorDevID, TW9900_SATURATION_CTL+1, *pi32Value);		//V Saturation
	}
	return TRUE;
}

typedef struct
{
	UINT16 u16ImageWidth; 
	UINT16 u16ImageHeight;
	UINT8 i8ResolIdx;
}S_TW9900SuppResol;

#define TW9900_RESOL_SUPP_CNT  1

S_TW9900SuppResol s_asTW9900SuppResolTable[TW9900_RESOL_SUPP_CNT] = {
#ifdef CONFIG_TWO_FIELDS
	{640, 480, 	TW9900_REG_VGA},	/* Capture VGA, JPEG encode VGA */
#endif
#ifdef CONFIG_ONE_FIELD
	{640, 240, 	TW9900_REG_VGA},	/* Capture Half VGA, JPEG encode upscale to VGA */
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

	if(vin_priv->sensor_intf->u8SensorDevID != g_uTW9900DeviceID[TW9900_Initial])
		return FALSE;
	DrvVideoIn_I2cWrite(vin_priv->sensor_intf->u8SensorDevID, u16SensRegAddr, c->value);
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
	if(vin_priv->sensor_intf->u8SensorDevID != g_uTW9900DeviceID[TW9900_Initial])
		return FALSE;
	c->value = (DrvVideoIn_I2cRead(vin_priv->sensor_intf->u8SensorDevID, u16SensRegAddr)&0xff);
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
static NVT_SENSOR_T nvt_sensor_tw9900 = {
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

	read_write_brightness:			TW9900ReadWriteBrightness,
	read_write_contrast:			TW9900ReadWriteContrast,
	read_write_sharpness:			NULL,
	read_write_white_balance:		NULL,
	read_write_noise_reduction:		NULL,
	read_write_color_saturation:	TW9900ReadWriteColorSaturation, 

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
	*sensor_intf = (NVT_SENSOR_T*)(&nvt_sensor_tw9900);
	return 0;
}
#endif 


