/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/
#include <asm/io.h>
#if 0
#include <asm/arch/w55fa93_reg.h>
#include <asm/arch/videoin.h>
#else
#include <mach/w55fa93_reg.h>
#include <mach/videoin.h>
#endif 
#include "DrvI2C.h"

#define ERR_PRINTF				printk
#define outp32(addr, value)		writel(value, addr)
#define inp32(addr)				readl(addr)
#define DBG_PRINTF(...)		
//-------------------
// I2C functions
//-------------------

static S_I2C_Channel s_sChannel;
static PFN_DRVI2C_TIMEDELY pfntimedelay=NULL; 


ERRCODE
DrvI2C_Open(
	UINT32 u32SCKPortIndex,
	UINT32 u32SCKPinMask,
	UINT32 u32SDAPortIndex,
	UINT32 u32SDAPinMask,
	PFN_DRVI2C_TIMEDELY pfnDrvI2C_Delay	
)
{
	// switch pin function to GPIO	
	UINT32 u32Idx=0, u32SckBit=0, u32SdaBit=0; 

	s_sChannel.u32SCKPortIndex    = u32SCKPortIndex;
	s_sChannel.u32SCKPinMask     = u32SCKPinMask;
	s_sChannel.u32SDAPortIndex    = u32SDAPortIndex;
	s_sChannel.u32SDAPinMask     = u32SDAPinMask;
 

	printk("SCK bit = %d\n", u32SCKPinMask);
	printk("SDA bit = %d\n", u32SDAPinMask);
	for(u32Idx=0;u32Idx<16;u32Idx=u32Idx+1)
	{
		if((u32SCKPinMask&0x01)==0x01)
		{
			u32SckBit = u32Idx;
			printk("SCK bit = %d\n", u32SckBit);
		}	
		if((u32SDAPinMask&0x01)==0x01)
		{
			u32SdaBit = u32Idx;
			printk("SDA bit = %d\n", u32SdaBit);
		}
		u32SCKPinMask = u32SCKPinMask >> 1;
		u32SDAPinMask = u32SDAPinMask >> 1;
	}
	
	outp32( (REG_GPAFUN+ (u32SCKPortIndex>>4)*4), inp32( (REG_GPAFUN+ (u32SCKPortIndex>>4)*4))& ~(3<<(u32SckBit *2)) );
	outp32( (REG_GPAFUN+ (u32SDAPortIndex>>4)*4), inp32( (REG_GPAFUN+ (u32SDAPortIndex>>4)*4))& ~(3<<(u32SdaBit *2)) );


	
	// 1.Check I/O pins. If I/O pins are used by other IPs, return error code.
	// 2.Enable IP¡¦s clock
	// 3.Reset IP
	// 4.Configure IP according to inputted arguments.
	// 5.Enable IP I/O pins
	// eq:GPIOB pin1, pin2 as output mode ( DRVGPIO_PIN1 | DRVGPIO_PIN2)
	// eq:Let clock pin and data pin to be high
	
	_DRVI2C_SCK_SETOUT(s_sChannel.u32SCKPortIndex, s_sChannel.u32SCKPinMask );
	_DRVI2C_SDA_SETOUT(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask );
	_DRVI2C_SCK_SETHIGH(s_sChannel.u32SCKPortIndex, s_sChannel.u32SCKPinMask);
	_DRVI2C_SDA_SETHIGH(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask);
	outp32(REG_GPIOA_PUEN+(s_sChannel.u32SDAPortIndex<<4), inp32(REG_GPIOA_PUEN+(s_sChannel.u32SDAPortIndex<<4))|s_sChannel.u32SDAPinMask);
	
	pfntimedelay=pfnDrvI2C_Delay;
	// 6.Return 0 to present success
	return Successful;

}	//DrvI2C_Open()

void DrvI2C_Close(void)
{
	// 1.Disable IP I/O pins
	// 2.Disable IP¡¦s clock
}

static void DrvI2C_Delay( 
	UINT32 nCount 
)
{
	volatile UINT32 i;
	if (pfntimedelay!=NULL)
	{
		pfntimedelay(nCount);		
	}else{
		for(;nCount!=0;nCount--)
			for(i=0;i<20;i++);
	}
}

void DrvI2C_SendStart(void)
{
	// Assert start bit
	_DRVI2C_SDA_SETOUT(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask);
	_DRVI2C_SDA_SETHIGH(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask);		// serial data pin high
	DrvI2C_Delay(2);
	_DRVI2C_SCK_SETHIGH(s_sChannel.u32SCKPortIndex, s_sChannel.u32SCKPinMask);		// serial clock pin high
	DrvI2C_Delay(2);
	_DRVI2C_SDA_SETLOW(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask);		// serial data pin low
	DrvI2C_Delay(2);
	_DRVI2C_SCK_SETLOW(s_sChannel.u32SCKPortIndex, s_sChannel.u32SCKPinMask);		// serial clock pin low
	DrvI2C_Delay(2);
}

void DrvI2C_SendStop(void)
{
	// Assert stop bit
	_DRVI2C_SDA_SETOUT(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask);
	_DRVI2C_SDA_SETLOW(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask);		// serial data pin low
	DrvI2C_Delay(2);
	_DRVI2C_SCK_SETHIGH(s_sChannel.u32SCKPortIndex, s_sChannel.u32SCKPinMask);		// serial clock pin high
	DrvI2C_Delay(2);
	_DRVI2C_SDA_SETHIGH(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask);		// serial data pin high
	DrvI2C_Delay(2);
	//DrvI2cDisable(s_sChannel.u32EnablePortIndex, s_sChannel.u32EnablePinMask);
}

//-------------------------
//master write bytes to slave device
ERRCODE 
DrvI2C_WriteByte(
	UINT8 u8Data, 
	UINT8 u8AckType,
	UINT8 u8Length
)
{
	UINT8   u8DataCount;
	UINT32 	i32HoldPinValue;
	_DRVI2C_SDA_SETOUT(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask);
	// Write data to device and the most signification bit(MSB) first
	for ( u8DataCount=0; u8DataCount<u8Length; u8DataCount++ )
	{
		if ( u8Data&0x80 )
			_DRVI2C_SDA_SETHIGH(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask);
		else
			_DRVI2C_SDA_SETLOW(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask);
		DrvI2C_Delay(3);
		_DRVI2C_SCK_SETHIGH(s_sChannel.u32SCKPortIndex, s_sChannel.u32SCKPinMask);
		u8Data<<=1;
		DrvI2C_Delay(2);
		_DRVI2C_SCK_SETLOW(s_sChannel.u32SCKPortIndex, s_sChannel.u32SCKPinMask);
		DrvI2C_Delay(2);
	}
	
	// No Ack 
	if ( u8AckType == DrvI2C_Ack_No )
		return Successful;

	// Have a Ack
	// Wait Device Ack bit
	_DRVI2C_SDA_SETLOW(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask);
	_DRVI2C_SDA_SETIN(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask);
	DrvI2C_Delay(3);
	_DRVI2C_SCK_SETHIGH(s_sChannel.u32SCKPortIndex, s_sChannel.u32SCKPinMask);
	DrvI2C_Delay(2);
	i32HoldPinValue = _DRVI2C_SDA_GETVALUE(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask);
	_DRVI2C_SCK_SETLOW(s_sChannel.u32SCKPortIndex, s_sChannel.u32SCKPinMask);
	DrvI2C_Delay(2);

	return (i32HoldPinValue == 0 ?E_DRVI2C_WRITE_FAIL: Successful);
}

//-------------------------------
//master read bytes data from slave device
UINT32 
DrvI2C_ReadByte(
	UINT8 u8AckType,
	UINT8 u8Length
)
{
	UINT32   u32Data  = 0;
	UINT8    u8DataCount;
	
	_DRVI2C_SDA_SETIN(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask);
	// Read data from slave device and the most signification bit(MSB) first
	for ( u8DataCount=0; u8DataCount<u8Length; u8DataCount++ )
	{
		u32Data = u32Data<<1;
		DrvI2C_Delay(3);
		_DRVI2C_SCK_SETHIGH(s_sChannel.u32SCKPortIndex, s_sChannel.u32SCKPinMask);		
		if (_DRVI2C_SDA_GETVALUE(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask)==s_sChannel.u32SDAPinMask)
			u32Data = u32Data|0x01;
		DrvI2C_Delay(2);
		_DRVI2C_SCK_SETLOW(s_sChannel.u32SCKPortIndex, s_sChannel.u32SCKPinMask);
		DrvI2C_Delay(2);
	}
	// No write Ack
	if ( u8AckType == DrvI2C_Ack_No )
		return u32Data;
	
	// Have a Ack
	// write a ACK bit to slave device 
	_DRVI2C_SDA_SETOUT(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask);
	_DRVI2C_SDA_SETHIGH(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask);
	DrvI2C_Delay(3);
	_DRVI2C_SCK_SETHIGH(s_sChannel.u32SCKPortIndex, s_sChannel.u32SCKPinMask);
	DrvI2C_Delay(2);
	_DRVI2C_SCK_SETLOW(s_sChannel.u32SCKPortIndex, s_sChannel.u32SCKPinMask);
	DrvI2C_Delay(2);
	_DRVI2C_SDA_SETLOW(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask);
	return u32Data;	
}

//-------------------------------
//slave read one byte data from master device
UINT8 
DrvI2C_Slave_ReadByte(
	UINT8 u8AckType
)
{
	UINT8 u8Data, u8DataCount;
	u8Data = 0;
	// Read data from device and the most signification bit(MSB) first	
	for ( u8DataCount=0; u8DataCount<8;  )
	{
		if (_DRVI2C_SCK_GETVALUE(s_sChannel.u32SCKPortIndex, s_sChannel.u32SCKPinMask)==0)
		{
    		DrvI2C_Delay(3);
    		
    		if (_DRVI2C_SCK_GETVALUE(s_sChannel.u32SCKPortIndex, s_sChannel.u32SCKPinMask)==s_sChannel.u32SCKPinMask)
    		{
				
				u8Data = u8Data<<1;
				if ( _DRVI2C_SDA_GETVALUE(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask)==s_sChannel.u32SDAPinMask )
        			 u8Data = u8Data|0x01;
        		
       			u8DataCount++;
      		}
		}
	}
    //  No Ack
	if ( u8AckType == DrvI2C_Ack_No )
		return u8Data;
	// Assert ACK bit
	
	DrvI2C_Delay(2);
	_DRVI2C_SDA_SETOUT(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask);
	_DRVI2C_SDA_SETHIGH(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask);
	DrvI2C_Delay(7);
	_DRVI2C_SDA_SETIN(s_sChannel.u32SDAPortIndex, s_sChannel.u32SDAPinMask);
	return u8Data;
}
