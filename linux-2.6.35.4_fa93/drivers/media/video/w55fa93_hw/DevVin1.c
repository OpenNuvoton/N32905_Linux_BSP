/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/
#ifdef CONFIG_ARCH_W55FA93
#include <asm/io.h>
#include <mach/w55fa93_reg.h>
#include <mach/videoin.h>
//#include "DrvVideoin.h"
#include "DrvVideoin.h"
#include <linux/delay.h>
#include <linux/clk.h>
extern unsigned int w55fa93_upll_clock, w55fa93_apll_clock, w55fa93_ahb_clock;
#else
#include <stdio.h>
#include <string.h>
#include "w55fa93_reg.h"
#include "W55FA93_VideoIn.h"
#include "wblib.h"
PFN_VIDEOIN_CALLBACK (pfnvideoIn_IntHandlerTable)[4]={0};
#endif
//#define REAL_CHIP

/*---------------------------------------------------------------------------------------------------------*/
/* Function: videoIn_IntHandler                                                                             */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*      None                                                                                               */
/*                                                                                                         */
/* Returns:                                                                                                */
/*      None                                                                                               */
/*                                                                                                         */
/* Description:                                                                                            */
/*      Driver internal use API to process the interrupt of Video-IN									   */
/*      As interrupt occurrence, the register call back function will be executed                          */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
static UINT32 u32EscapeFrame = 0;
static UINT32 g_u32DeviceType = 0;
//static UINT32 g_u32PortOffset = 0x0;
/*---------------------------------------------------------------------------------------------------------*/
/* Function: videoIn_Port                                                                             */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*      None                                                                                               */
/*                                                                                                         */
/* Returns:                                                                                                */
/*      None                                                                                               */
/*                                                                                                         */
/* Description:                                                                                            */
/*      Using the function to initialize the both videoIn ports 										   */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#if 0
void videoIn_Port(UINT32 u32Port)
{
	if(u32Port==0)
		g_u32PortOffset = 0;	
	else
		g_u32PortOffset = 0x800;
}
#else
#define g_u32PortOffset  	0
#endif

#ifndef CONFIG_ARCH_W55FA93
static void 
videoIn_IntHandler(
	void
	)
{
	UINT32 u32CapInt;
	UINT32 uBuf=0;
	BOOL bIsFiled;
	
	if((inp32(REG_AHBCLK)&VIN0_CKE)==VIN0_CKE)
	{//Check videoIn Enable 
		bIsFiled = (inp32(REG_VPEPAR) & FLDID) >>20;			
		u32CapInt = inp32(REG_VPEINT);
		if( (u32CapInt & (VINTEN | VINT)) == (VINTEN | VINT))
		{
			if(pfnvideoIn_IntHandlerTable[0]!=0)	
				pfnvideoIn_IntHandlerTable[0](uBuf, uBuf, bIsFiled, u32EscapeFrame);
			outp32(REG_VPEINT, (u32CapInt & ~(MDINT | ADDRMINT | MEINT)));		/* Clear Frame end interrupt */
			u32EscapeFrame=u32EscapeFrame+1;
		}	
		else if((u32CapInt & (ADDRMEN|ADDRMINT)) == (ADDRMEN|ADDRMINT))
		{
			if(pfnvideoIn_IntHandlerTable[1]!=0)	
				pfnvideoIn_IntHandlerTable[1](uBuf, uBuf, bIsFiled, u32EscapeFrame);
			outp32(REG_VPEINT, (u32CapInt & ~(MDINT | VINT | MEINT)));			/* Clear Address match interrupt */
		}	
		else if ((u32CapInt & (MEINTEN|MEINT)) == (MEINTEN|MEINT))
		{
			if(pfnvideoIn_IntHandlerTable[2]!=0)	
				pfnvideoIn_IntHandlerTable[2](uBuf, uBuf, bIsFiled, u32EscapeFrame);	
			outp32(REG_VPEINT, (u32CapInt & ~(MDINT | VINT|ADDRMINT)));			/* Clear Memory error interrupt */	
		}	
		else if ((u32CapInt & (MDINTEN|MDINT)) == (MDINTEN|MDINT))
		{
			if(pfnvideoIn_IntHandlerTable[3]!=0)	
				pfnvideoIn_IntHandlerTable[3](uBuf, uBuf, bIsFiled, u32EscapeFrame);	
			outp32(REG_VPEINT, (u32CapInt & ~( VINT | MEINT | ADDRMINT)));			/* Clear Memory error interrupt */	
		}
	}	
	//outp32(REG_VPECTL,  inp32(REG_VPECTL) | UPDATE);
} 
#endif 
/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvVideoIn_SetInitFrame &  DrvVideoIn_GetSkipFrame                                            */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*      None                                                                                               */
/*                                                                                                         */
/* Returns:                                                                                                */
/*      None                                                                                               */
/*                                                                                                         */
/* Description:                                                                                            */
/*     	If enable interrupt, there is internal counter that records how many frames have pass. 			   */
/*      Set the internal counters to zero. The internal counter may be not a constant                      */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
static void 
videoIn_SetInitFrame(
	void
	)
{
	u32EscapeFrame = 0;
}

static UINT32
videoIn_GetSkipFrame(
	void
	)
{
	return u32EscapeFrame;
}


static void videoIn_Init(
	BOOL bIsEnableSnrClock,
	E_VIDEOIN_SNR_SRC eSnrSrc,	
	UINT32 u32SensorFreq,						//KHz unit
	E_VIDEOIN_DEV_TYPE eDevType
	)
{
	//INT32 i32Idx0, i32Idx1;
	UINT32 u32PllClock, u32SenSrc, u32SenDiv;
	UINT32 u32Div0, u32Div1;
	struct clk *clk;

	g_u32DeviceType = eDevType;

	u32PllClock = w55fa93_upll_clock;//sysGetPLLOutputKhz(eSYS_UPLL, u32ExtFreq);
	u32SenSrc = 0x18<<16;   
	if(u32PllClock==0) 
	{ 
		u32PllClock = w55fa93_apll_clock;//sysGetPLLOutputKhz(eSYS_APLL, u32ExtFreq);      
		u32SenSrc = 0x10<<16;      
	}   
	DBG_PRINTF("Sensor clock = %d KHz\n", u32SensorFreq);
	u32SenDiv = u32PllClock/u32SensorFreq;    
	DBG_PRINTF("Sensor divider = %d\n", u32SenDiv);
	if(u32PllClock%u32SensorFreq!=0)   
		u32SenDiv = u32SenDiv+1;      
	for(u32Div0=2; u32Div0<=8; u32Div0 = u32Div0+1)	   
	{//u32Div0 should be start from 2   
		for(u32Div1=1; u32Div1<=16; u32Div1 = u32Div1+1)	         
		{      
			if(u32SenDiv==u32Div0*u32Div1)         
				break;            
		}      
		if(u32SenDiv==u32Div0*u32Div1)      
				break;            
	}	     
	u32Div0 = u32Div0-1; 
	u32Div1 = u32Div1-1; 
	outp32(REG_CLKDIV0, (inp32(REG_CLKDIV0) & ~(SENSOR_N1 | SENSOR_S | SENSOR_N0)) | 
				 (u32SenSrc | ((u32Div0<<16) | (u32Div1<<24)))  );   
	
	if(bIsEnableSnrClock)	
	{	
		//outp32(REG_AHBCLK, inp32(REG_AHBCLK) | SEN_CKE );						/* Enable Sensor clock */
		clk = clk_get(NULL, "SEN");
		clk_enable(clk);
	}
	else
	{
		//outp32(REG_AHBCLK, inp32(REG_AHBCLK) & ~SEN_CKE );						/* Disable Sensor clock */	
		clk = clk_get(NULL, "SEN");
		clk_disable(clk);
	}
	if(eDevType == eDrvVideoIn_SNR_CCIR656)
	{//VSYNC and HSYN by data
	#if 0
		outp32(REG_GPBFUN, inp32(REG_GPBFUN) | ( MF_GPB0 | MF_GPB1 | 						//SCLKO, PCLK
										MF_GPB5 | MF_GPB6 | MF_GPB7 | MF_GPB8 | 	//VDATA[7:0]
										MF_GPB9 | MF_GPB10 | MF_GPB11 | MF_GPB12) );	
	#else
		outp32(REG_GPBFUN, inp32(REG_GPBFUN) | 0x03FFFC0F );									
	#endif																		   								   
	}
	else if(eDevType == eDrvVideoIn_SNR_CCIR601)
	{//VSYNC and HSYN by pins
		outp32(REG_GPBFUN, inp32(REG_GPBFUN) | ( MF_GPB0 | MF_GPB1 | MF_GPB2 | MF_GPB3 |	//SCLKO, PCLK, HSYNC and VSYNC
										MF_GPB5 | MF_GPB6 | MF_GPB7 | MF_GPB8 | 	//VDATA[7:0]
										MF_GPB9 | MF_GPB10 | MF_GPB11 | MF_GPB12) );										
	}
	else if(eDevType == eDrvVideoIn_TVD_CCIR601)
	{
		outp32(REG_GPBFUN, inp32(REG_GPBFUN) | ( MF_GPB0 | MF_GPB1 | MF_GPB2 | MF_GPB3 | MF_GPB4 |	//SCLKO, PCLK, HSYNC, VSYNC and SFILED
										MF_GPB5 | MF_GPB6 | MF_GPB7 | MF_GPB8 | 	//VDATA[7:0]
										MF_GPB9 | MF_GPB10 | MF_GPB11 | MF_GPB12) );								  									
	}
	else if(eDevType == eDrvVideoIn_TVD_CCIR656)
	{
		outp32(REG_GPBFUN, inp32(REG_GPBFUN) | (  MF_GPB1 |					//PCLK
										MF_GPB5 | MF_GPB6 | MF_GPB7 | MF_GPB8 | 	//VDATA[7:0]
										MF_GPB9 | MF_GPB10 | MF_GPB11 | MF_GPB12) );												
	}
	else if(eDevType == eDrvVideoIn_2nd_SNR_CCIR601)
	{
	  	//outp32(REG_GPBFUN, inp32(REG_GPBFUN) & 0xFC00000F);	//The 1st sensor must other function(GPIO or other) if enable 2nd sensor
		outp32(REG_GPCFUN, (inp32(REG_GPCFUN) & ~(MF_GPC15 | MF_GPC14 | MF_GPC13 | MF_GPC12 |
										  MF_GPC11 | MF_GPC10 | MF_GPC9 | MF_GPC8)) 
										  | (0x5555<<16));	//Data
	#if 0 
		//H and VSYNC from GPE1 and GPE0 										  
		outp32(REG_GPEFUN, (inp32(REG_GPEFUN) & ~(MF_GPE1 | MF_GPE0))
										  | 0x5);  			//Sync
	#else
		//H and VSYNC from GPB2 and GPE3  
		outp32(REG_GPBFUN, (inp32(REG_GPBFUN) | 0xF0));	
	#endif										  										  								 										  
		outp32(REG_GPBFUN, (inp32(REG_GPBFUN) | 0xF));
										    			//Clk in/out
	}
	else if(eDevType == eDrvVideoIn_2nd_TVD_CCIR656)
	{
	  	outp32(REG_GPBFUN, inp32(REG_GPBFUN) & 0xFC00000F);	//The 1st sensor must other function(GPIO or other) if enable 2nd sensor
		outp32(REG_GPCFUN, (inp32(REG_GPCFUN) & ~(MF_GPC15 | MF_GPC14 | MF_GPC13 | MF_GPC12 |
										  MF_GPC11 | MF_GPC10 | MF_GPC9 | MF_GPC8)) 
										  | (0x5555<<16));	//Data
		//H and VSYNC from GPB2 and GPB3  
		//outp32(REG_GPBFUN, (inp32(REG_GPBFUN) | 0xF0));										  										  								 										  
		outp32(REG_GPBFUN, (inp32(REG_GPBFUN) | 0xF));
										    			//Clk in/out
	}
	else if(eDevType == eDrvVideoIn_3rd_SNR_CCIR601)
	{
	  	//outp32(REG_GPBFUN, inp32(REG_GPBFUN) & 0xFC00000F);	//The 1st sensor must other function(GPIO or other) if enable 2nd sensor
		outp32(REG_GPCFUN, (inp32(REG_GPCFUN) & ~(MF_GPC15 | MF_GPC14 | MF_GPC13 | MF_GPC12 |
										  MF_GPC11 | MF_GPC10 | MF_GPC9 | MF_GPC8)) 
										  | (0x5555<<16));	//Data
	#if 1
		//H and VSYNC from GPE1 and GPE0 										  
		outp32(REG_GPEFUN, (inp32(REG_GPEFUN) & ~(MF_GPE1 | MF_GPE0))
										  | 0x5);  			//Sync
	#else
		//H and VSYNC from GPB2 and GPE3  
		outp32(REG_GPBFUN, (inp32(REG_GPBFUN) | 0xF0));	
	#endif										  										  											  
		outp32(REG_GPBFUN, (inp32(REG_GPBFUN) | 0xF));
										    			//Clk in/out
	}
	else if(eDevType == eDrvVideoIn_3rd_TVD_CCIR656)
	{//IN's demo board
	  	outp32(REG_GPBFUN, inp32(REG_GPBFUN) & 0xFC00000F);	//The 1st sensor must other function(GPIO or other) if enable 2nd sensor
		outp32(REG_GPCFUN, (inp32(REG_GPCFUN) & ~(MF_GPC15 | MF_GPC14 | MF_GPC13 | MF_GPC12 |
										  MF_GPC11 | MF_GPC10 | MF_GPC9 | MF_GPC8)) 
										  | (0x5555<<16));	//Data											
										  // For 93QDN 
		//H and VSYNC from GPE1 and GPE0 										  
		//outp32(REG_GPEFUN, (inp32(REG_GPEFUN) & ~(MF_GPE1 | MF_GPE0))
		//								  | 0x5);  			//Sync
										  										  								 										  
		outp32(REG_GPBFUN, (inp32(REG_GPBFUN) | 0xF));
										    			//Clk in/out
	}					
}
/*----------------------------------------------------------------------------------------------------------------*/
/* Function: videoIn_Open 									                                           	*/
/*                                                                                                         							*/
/* Parameters:                                                                                             						*/
/*      u32EngFreqKHz : Engine working Freq                                                                				*/
/* 			CAP0 : the engine clock divider need to be filled with 0							*/
/* 			CAP1 : the engine clock divider is useless										*/
/*      u32SensorFreqKHz : Sensor working Freq                                                                				*/
/* Returns:                                                                                                						*/
/*      None                                                                                               						*/
/*                                                                                                         							*/
/* Description:                                                                                            						*/
/*     	Initialize the VIdeo-In engine. Register a call back for driver internal using                     		*/
/*                                                                                                         							*/
/*----------------------------------------------------------------------------------------------------------------*/
static INT32 
videoIn_Open(
	UINT32 u32EngFreqKHz, 								//VideoIn eng working frequency  
	UINT32 u32SensorFreq								//Specify the sensor clock 
)
{
	struct clk *clk;
	UINT32  u32AHB234, u32PllClock; 
	INT32 i32EngDiv, u32Div0, u32Div1, u32SenDiv, u32SenSrc;							
				   									
	//outp32(REG_AHBCLK, inp32(REG_AHBCLK) | (CAP_CKE | HCLK4_CKE));				/* Enable Cap clock */
	clk = clk_get(NULL, "CAP");
	clk_enable(clk);

	outp32(REG_AHBIPRST, inp32(REG_AHBIPRST) | CAPRST );							/* Global reset Capture */
	outp32(REG_AHBIPRST, inp32(REG_AHBIPRST) & (~CAPRST) );								


	u32AHB234 = w55fa93_ahb_clock;//DrvSYS_GetAHB234Clock();
	i32EngDiv = (u32AHB234 / u32EngFreqKHz);
	if(i32EngDiv>=1)
		i32EngDiv = i32EngDiv -1;
	outp32(REG_CLKDIV4, (inp32(REG_CLKDIV4) & ~CAP_N) |
					(i32EngDiv<<12) );		
	

	u32PllClock = w55fa93_upll_clock;//sysGetPLLOutputKhz(eSYS_UPLL, u32ExtFreq);
	u32SenSrc = 0x18<<16;   
	if(u32PllClock==0) 
	{ 
		u32PllClock = w55fa93_apll_clock;//sysGetPLLOutputKhz(eSYS_APLL, u32ExtFreq);      
		u32SenSrc = 0x10<<16;      
	}   
	DBG_PRINTF("Sensor clock = %d KHz\n", u32SensorFreq);
	u32SenDiv = u32PllClock/u32SensorFreq;    
	DBG_PRINTF("Sensor divider = %d\n", u32SenDiv);
	if(u32PllClock%u32SensorFreq!=0)   
		u32SenDiv = u32SenDiv+1;      
	for(u32Div0=2; u32Div0<=8; u32Div0 = u32Div0+1)	   
	{//u32Div0 should be start from 2   
		for(u32Div1=1; u32Div1<=16; u32Div1 = u32Div1+1)	         
		{      
			if(u32SenDiv==u32Div0*u32Div1)         
				break;            
		}      
		if(u32SenDiv==u32Div0*u32Div1)      
				break;            
	}	     
	u32Div0 = u32Div0-1; 
	u32Div1 = u32Div1-1; 
	outp32(REG_CLKDIV0, (inp32(REG_CLKDIV0) & ~(SENSOR_N1 | SENSOR_S | SENSOR_N0)) | 
				 (u32SenSrc | ((u32Div0<<16) | (u32Div1<<24)))  );  

/*			
	sysInstallISR(IRQ_LEVEL_1, 
						IRQ_VIDEOIN, 
						(PVOID)VideoInIntHandler);
	sysEnableInterrupt(IRQ_VIDEOIN);	
*/
	return Successful;
} // videoIn_Open
/*---------------------------------------------------------------------------------------------------------*/
/* Function: videoIn_Close 									                                           */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*      None: 													                                           */
/* Returns:                                                                                                */
/*      None                                                                                               */
/*                                                                                                         */
/* Description:                                                                                            */
/*     	Disable pin function,engine clock and interruot                 			                       */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
static void
videoIn_Close(
	void
	)
{
	struct clk *clk;
	// 1. Disable IP's interrupt
	//sysDisableInterrupt(IRQ_VIDEOIN);	
	// 2. Disable IP?™s clock
	//outp32(REG_AHBCLK, inp32(REG_AHBCLK) & ~CAP_CKE);						
	clk = clk_get(NULL, "CAP");
	clk_disable(clk);
	clk = clk_get(NULL, "SEN");
	clk_disable(clk);

	// 3. Disable Capture pin function 
	if(g_u32DeviceType == eDrvVideoIn_SNR_CCIR656)
	{//VSYNC and HSYN by data
		outp32(REG_GPBFUN, inp32(REG_GPBFUN) & ~( MF_GPB0 | MF_GPB1 | 						//SCLKO, PCLK
										MF_GPB5 | MF_GPB6 | MF_GPB7 | MF_GPB8 | 	//VDATA[7:0]
										MF_GPB9 | MF_GPB10 | MF_GPB11 | MF_GPB12) );		
																   										   		
	}								   
	else if(g_u32DeviceType == eDrvVideoIn_SNR_CCIR601)
	{//VSYNC and HSYN by pins
		outp32(REG_GPBFUN, inp32(REG_GPBFUN) & ~( MF_GPB0 | MF_GPB1 | MF_GPB2 | MF_GPB3 |	//SCLKO, PCLK, HSYNC and VSYNC
										MF_GPB5 | MF_GPB6 | MF_GPB7 | MF_GPB8 | 	//VDATA[7:0]
										MF_GPB9 | MF_GPB10 | MF_GPB11 | MF_GPB12) );	
	}
	else if(g_u32DeviceType == eDrvVideoIn_TVD_CCIR601)
	{
		outp32(REG_GPBFUN, inp32(REG_GPBFUN) & ~( MF_GPB0 | MF_GPB1 | MF_GPB2 | MF_GPB3 | MF_GPB4 |	//SCLKO, PCLK, HSYNC, VSYNC and SFILED
										MF_GPB5 | MF_GPB6 | MF_GPB7 | MF_GPB8 | 	//VDATA[7:0]
										MF_GPB9 | MF_GPB10 | MF_GPB11 | MF_GPB12) );									   					
	}
	else if(g_u32DeviceType == eDrvVideoIn_TVD_CCIR656)
	{
		outp32(REG_GPBFUN, inp32(REG_GPBFUN) & ~( MF_GPB1 | 	//SCLKO and PCLK 
										MF_GPB5 | MF_GPB6 | MF_GPB7 | MF_GPB8 | 	//VDATA[7:0]
										MF_GPB9 | MF_GPB10 | MF_GPB11 |  MF_GPB12) );				   
	}
	else if(g_u32DeviceType == eDrvVideoIn_2nd_SNR_CCIR601)
	{
	  	//outp32(REG_GPBFUN, inp32(REG_GPBFUN) & 0xFC00000F);	  	//Disable 1st sensor interface	  
		outp32(REG_GPCFUN, (inp32(REG_GPCFUN) & (~0xFFFF0000)) );		
		outp32(REG_GPEFUN, (inp32(REG_GPEFUN) & (~0xF)) );			//Disable 2nd sensor interface			  
		outp32(REG_GPBFUN, (inp32(REG_GPBFUN) & (~0xF)));													    			
	}	
	else if(g_u32DeviceType == eDrvVideoIn_3rd_SNR_CCIR601)
	{
	  	//outp32(REG_GPBFUN, inp32(REG_GPBFUN) & 0xFC00000F);	//The 1st sensor must other function(GPIO or other) if enable 2nd sensor
		outp32(REG_GPCFUN, inp32(REG_GPCFUN) & ~(MF_GPC15 | MF_GPC14 | MF_GPC13 | MF_GPC12 |
										  MF_GPC11 | MF_GPC10 | MF_GPC9 | MF_GPC8) );	//Data
		//H and VSYNC from GPE1 and GPE0 										  
		outp32(REG_GPEFUN, inp32(REG_GPEFUN) & ~(MF_GPE1 | MF_GPE0));  			
							  										  											  
		outp32(REG_GPBFUN, (inp32(REG_GPBFUN) & ~0xF));
										    			//Clk in/out
	}								 		   	
} // DrvVideoIn_Close

//UINT32 uCapIntCallback = eVIDEOIN_VINT;
/*---------------------------------------------------------------------------------------------------------*/
/* Function: videoIn_SetPacketFrameBufferControl 									                   */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*      bFrameSwitch: Software mode buffer select  				                                           */
/*					0: Packet buffer 0																	   */
/*					1: Packet buffer 1																	   */
/*      bFrameBufferSel :   Buffer control by FSC IP                                                       */
/* Returns:                                                                                                */
/*      None                                                                                               */
/*                                                                                                         */
/* Description:                                                                                            */
/*     	Disable pin function,engine clock and interruot                 			                       */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
static void videoIn_SetPacketFrameBufferControl(
	BOOL bFrameSwitch,
	BOOL bFrameBufferSel
	)
{
	UINT32 u32Ctl;
	u32Ctl = inp32(REG_VPECTL+g_u32PortOffset) & ~(ADDRSW | FBMODE);
	outp32(REG_VPECTL+g_u32PortOffset, u32Ctl |
					((bFrameBufferSel?FBMODE:0) | 	
					(bFrameSwitch?ADDRSW:0)));					
}

static void 
videoIn_GetPacketFrameBufferControl(
	PBOOL pbFrameSwitch,
	PBOOL pbFrameBufferSel
	)
{
	UINT32 u32Ctl = inp32(REG_VPECTL+g_u32PortOffset);
	*pbFrameBufferSel = (u32Ctl & FBMODE) >> 2;
	*pbFrameSwitch = (u32Ctl & ADDRSW) >> 3;
}
#ifndef CONFIG_ARCH_W55FA93
/*---------------------------------------------------------------------------------------------------------*/
/* Function: videoIn_InstallCallback			  									                       */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*      eIntType: Interrupt type				  				                                           */
/*      pvIsr: Call back fucntion					                                                       */
/* Returns:                                                                                                */
/*      None                                                                                               */
/*                                                                                                         */
/* Description:                                                                                            */
/*     	Disable pin function,engine clock and interrupt                 			                       */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
static INT32 
videoIn_InstallCallback(
	E_VIDEOIN_INT_TYPE eIntType, 
	PFN_VIDEOIN_CALLBACK pfnCallback,
	PFN_VIDEOIN_CALLBACK *pfnOldCallback
	)
{		
	if(eIntType == eVIDEOIN_VINT)
	{
		*pfnOldCallback = pfnvideoIn_IntHandlerTable[0];
		pfnvideoIn_IntHandlerTable[0] = (PFN_VIDEOIN_CALLBACK)(pfnCallback);
	}	
	else if(eIntType == eVIDEOIN_ADDRMINT)
	{
		*pfnOldCallback = pfnvideoIn_IntHandlerTable[1];
		pfnvideoIn_IntHandlerTable[1] = (PFN_VIDEOIN_CALLBACK)(pfnCallback);
	}
	else if(eIntType == eVIDEOIN_MEINT)
	{
		*pfnOldCallback = pfnvideoIn_IntHandlerTable[2];
		pfnvideoIn_IntHandlerTable[2] = (PFN_VIDEOIN_CALLBACK)(pfnCallback);
	}	
	else if(eIntType == eVIDEOIN_MDINT)
	{
		*pfnOldCallback = pfnvideoIn_IntHandlerTable[3];
		pfnvideoIn_IntHandlerTable[3] = (PFN_VIDEOIN_CALLBACK)(pfnCallback);
	}	
	else
		return E_VIDEOIN_INVALID_INT;			
	return Successful;	
}
#endif


static void
videoIn_Reset(
	void
	)
{	
	outp32(REG_VPECTL+g_u32PortOffset, inp32(REG_VPECTL+g_u32PortOffset) | VPRST);			
	outp32(REG_VPECTL+g_u32PortOffset, inp32(REG_VPECTL+g_u32PortOffset) & (~VPRST));			
} // VideoIn_Reset

static INT32 
videoIn_EnableInt(
	E_VIDEOIN_INT_TYPE eIntType
	)
{
	switch(eIntType)
	{
		case eVIDEOIN_MDINT:
		case eVIDEOIN_ADDRMINT:							
		case eVIDEOIN_MEINT:				
		case eVIDEOIN_VINT:				
				outp32(REG_VPEINT+g_u32PortOffset, inp32(REG_VPEINT+g_u32PortOffset) | eIntType);
				break;		
		default: 				 		
			return E_VIDEOIN_INVALID_INT;
	}
	return Successful;
} // VideoIn_EnableInt

static INT32
videoIn_DisableInt(
	E_VIDEOIN_INT_TYPE eIntType
	)
{
	switch(eIntType)
	{
		case eVIDEOIN_MDINT:
		case eVIDEOIN_ADDRMINT:							
		case eVIDEOIN_MEINT:				
		case eVIDEOIN_VINT:				
				outp32(REG_VPEINT+g_u32PortOffset, inp32(REG_VPEINT+g_u32PortOffset) & ~eIntType );
				break;		
		default: 				 		
			return E_VIDEOIN_INVALID_INT;
	}
	return Successful;
} // videoIn_DisableInt

static BOOL
videoIn_IsIntEnabled(
	E_VIDEOIN_INT_TYPE eIntType
	)
{
	UINT32 u32IntEnable = inp32(REG_VPEINT+g_u32PortOffset);	
	switch(eIntType)
	{
		case eVIDEOIN_MDINT:
				u32IntEnable = u32IntEnable & eVIDEOIN_MDINT;
				break;		
		case eVIDEOIN_ADDRMINT:	
				u32IntEnable = u32IntEnable & eVIDEOIN_ADDRMINT;
				break;					
		case eVIDEOIN_MEINT:			
				u32IntEnable = u32IntEnable & eVIDEOIN_MEINT;
				break;	
		case eVIDEOIN_VINT:				
				u32IntEnable = u32IntEnable & eVIDEOIN_VINT;
				break;		
	}
	return (u32IntEnable?TRUE:FALSE);	
} // DrvVideoIn_IsIntEnabled
//===================================================================================
//
//	Clear the interrupt status. //Write one clear 
//
//
//
//
//
//
//
//
//
//
//===================================================================================
static INT32
videoIn_ClearInt(
	E_VIDEOIN_INT_TYPE eIntType
	)
{
	UINT32 u32IntChannel = eIntType >>16;
	switch(eIntType)
	{
		case eVIDEOIN_MDINT:
				outp32(REG_VPEINT+g_u32PortOffset, (inp32(REG_VPEINT+g_u32PortOffset) & ~((eVIDEOIN_ADDRMINT | eVIDEOIN_MEINT | eVIDEOIN_VINT)>>16)) | 
								u32IntChannel);
				break;					
		case eVIDEOIN_ADDRMINT:		
				outp32(REG_VPEINT+g_u32PortOffset, (inp32(REG_VPEINT+g_u32PortOffset) & ~((eVIDEOIN_MDINT | eVIDEOIN_MEINT | eVIDEOIN_VINT)>>16)) | 
								u32IntChannel);
				break;					
		case eVIDEOIN_MEINT:
				outp32(REG_VPEINT+g_u32PortOffset, (inp32(REG_VPEINT+g_u32PortOffset) & ~((eVIDEOIN_MDINT | eVIDEOIN_ADDRMINT | eVIDEOIN_VINT)>>16)) | 
								u32IntChannel);
				break;					
		case eVIDEOIN_VINT:				
				outp32(REG_VPEINT+g_u32PortOffset, (inp32(REG_VPEINT+g_u32PortOffset) & ~((eVIDEOIN_MDINT | eVIDEOIN_MEINT | eVIDEOIN_ADDRMINT)>>16)) | 
								u32IntChannel);
				break;	
		default: 				 		
			return E_VIDEOIN_INVALID_INT;
	}
	return Successful;


} // DrvVideoIn_ClearInt

//===================================================================================
//
//	Polling the interrupt status 
//
//
//	return value could be 
//	
//	VIDEOIN_ADDRMINT | VIDEOIN_MEINT | VIDEOIN_VINT
//
//
//
//
//
//===================================================================================
static UINT32
videoIn_PollInt(
	void	
	)
{
	UINT32 u32IntStatus = inp32(REG_VPEINT+g_u32PortOffset);	
	return u32IntStatus;
#if 0
	switch(eIntType)
	{
		case eVIDEOIN_MDINT:
				u32IntStatus = u32IntStatus & (eVIDEOIN_MDINT>>16);
				break;		
		case eVIDEOIN_ADDRMINT:	
				u32IntStatus = u32IntStatus & (eVIDEOIN_ADDRMINT>>16);
				break;					
		case eVIDEOIN_MEINT:			
				u32IntStatus = u32IntStatus & (eVIDEOIN_MEINT>>16);
				break;	
		case eVIDEOIN_VINT:				
				u32IntStatus = u32IntStatus & (eVIDEOIN_VINT>>16);
				break;		
	}
	return (u32IntStatus?TRUE:FALSE);
#endif 
} // DrvVideoIn_PollInt

//===================================================================================
//
//	Enable engine and turn on the pipe.
//
//
//
//
//
//
//	u32PipeEnable = 0. Both pipe disable.
//					1. Packet pipe enable.	
//					2. Planar pipe enable.
//					3. Both pipe enable.
//===================================================================================
static void videoIn_SetPipeEnable(
	BOOL bEngEnable,     				// TRUE: Enable, FALSE: Disable
	E_VIDEOIN_PIPE ePipeEnable    
	)
{
	outp32(REG_VPECTL+g_u32PortOffset, (inp32(REG_VPECTL+g_u32PortOffset) & ~(VPEEN | PKEN | PNEN)) 
    | (((bEngEnable ? VPEEN : 0x0))
    // | ((ePipeEnable & ~(PKEN | PNEN))<<5)) );
    | ((ePipeEnable & 0x03)<<5)) );
} // DrvVideoIn_SetPipeEnable

static void videoIn_GetPipeEnable(
	PBOOL pbEngEnable,      			// TRUE: Enable, FALSE: Disable
	E_VIDEOIN_PIPE* pePipeEnable     // TRUE: Enable, FALSE: Disable
	)
{
	UINT32 u32Temp = inp32(REG_VPECTL+g_u32PortOffset);
	
	*pbEngEnable = (u32Temp & VPEEN) ? TRUE : FALSE;
	*pePipeEnable = (u32Temp & (PKEN | PNEN))>>5;
} // DrvVideoIn_GetPipeEnable


static void 
videoIn_SetShadowRegister(
	void
	)
{
	outp32(REG_VPECTL+g_u32PortOffset,  inp32(REG_VPECTL+g_u32PortOffset) | UPDATE);
} // DrvVideoIn_SetShadowRegister

static void 
videoIn_SetSensorPolarity(
	BOOL bVsync,       					// TRUE: High Active, FALSE: Low Active
	BOOL bHsync,       					// TRUE: High Active, FALSE: Low Active
	BOOL bPixelClk     					// TRUE: Falling Edge, FALSE: Rising Edge
	)
{
#if 1
	UINT32 u32Polarity, u32RegVal;
	u32RegVal = inp32(REG_VPEPAR+g_u32PortOffset);
	//DBG_PRINTF("Enter Register addr = 0x%x\n", (REG_VPEPAR+g_u32PortOffset));
	//DBG_PRINTF("Enter Register value = 0x%x\n", u32RegVal);
	u32Polarity =  (((bVsync ? VSP : 0x0) | (bHsync ? HSP : 0x0)) | (bPixelClk ? PCLKP : 0x0));
	u32RegVal =  (inp32(REG_VPEPAR+g_u32PortOffset) & ~(VSP| HSP | PCLKP)) ; 
	//DBG_PRINTF("REG_VPEPAR = 0x%x", (u32RegVal | u32Polarity));
	outp32((REG_VPEPAR+g_u32PortOffset), (u32RegVal | u32Polarity));
#else	
	outp32((REG_VPEPAR+g_u32PortOffset), (inp32(REG_VPEPAR+g_u32PortOffset) & ~(VSP| HSP | PCLKP)) 
    | (((bVsync ? VSP : 0x0)
    | (bHsync ? HSP : 0x0))
    | (bPixelClk ? PCLKP : 0x0)) );
#endif    
} // DrvVideoIn_SetSensorPolarity

static void 
videoIn_GetSensorPolarity(
	PBOOL pbVsync,      				// TRUE: High Active, FALSE: Low Active
	PBOOL pbHsync,      				// TRUE: High Active, FALSE: Low Active
	PBOOL pbPixelClk    				// TRUE: Falling Edge, FALSE: Rising Edge
	)
{
	UINT32 u32Temp = inp32(REG_VPEPAR+g_u32PortOffset);
	
	*pbVsync = (u32Temp & VSP)  ? TRUE : FALSE;
	*pbHsync = (u32Temp & HSP) ? TRUE : FALSE;
	*pbPixelClk = (u32Temp & PCLKP) ? TRUE : FALSE;
} // DrvVideoIn_GetSensorPolarity
/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvVideoIn_InstallInterrupt			  									                   */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*      eInputOrder: Data order for input format 														   */ 
/*			00 = Y0 U0 Y1 V0																			   */
/*			01 = Y0 V0 Y1 U0																			   */
/*			10 = U0 Y0 V0 Y1																			   */
/*			11 = V0 Y0 U0 Y1    	  				                                          			   */
/*      eInputFormat: 																					   */
/*			0 = YUV.	1=RGB			 					                                               */
/* 		eOutputFormat: 																				       */
/*			00=YCbCr422.	01=only output Y.   10=RGB555	11=RGB565									   */
/* Returns:                                                                                                */
/*      None                                                                                               */
/*                                                                                                         */
/* Description:                                                                                            */
/*     	Set the input format, input order and output format               			                       */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
static void 
videoIn_SetDataFormatAndOrder(
	E_VIDEOIN_ORDER eInputOrder, 
	E_VIDEOIN_IN_FORMAT eInputFormat, 
	E_VIDEOIN_OUT_FORMAT eOutputFormat
	)
{
	outp32((REG_VPEPAR+g_u32PortOffset), (inp32(REG_VPEPAR+g_u32PortOffset) & ~(OUTFMT | PDORD | INFMT)) 
    | ((((eInputOrder << 2) & PDORD)
    | (eInputFormat & INFMT))
    | ((eOutputFormat <<4 ) & OUTFMT)) );  
} // DrvVideoIn_SetDataFormatAndOrder

static void videoIn_GetDataFormatAndOrder(
	E_VIDEOIN_ORDER* peInputOrder, 
	E_VIDEOIN_IN_FORMAT* peInputFormat, 
	E_VIDEOIN_OUT_FORMAT* peOutputFormat
	)
{
	UINT32 u32Temp = inp32(REG_VPEPAR+g_u32PortOffset);
	
	*peInputOrder = (u32Temp & PDORD) >> 2;
	*peInputFormat = u32Temp & INFMT;
	*peOutputFormat = (u32Temp & OUTFMT) >> 4;
} // DrvVideoIn_GetDataFormatAndOrder

/*----------------------------------------------------------------------------------------------------------*/
/* Function: DrvVideoIn_SetPlanarFormat			  									        */
/*                                                                                                         							*/
/* Parameters:                                                                                             						*/
/*      E_VIDEOIN_PLANAR_FORMAT: Planar Format											*/ 
/*			0 : Planar YUV422														*/
/*			1 : Planar YUV420														*/
/* 			2 : Macro Planar YUV420													*/
/* Returns:                                                                                                						*/
/*      None                                                                                               						*/
/*                                                                                                         							*/
/* Description:                                                                                            						*/
/*     	Set the planar output format, 						               			                       	*/
/*                                                                                                         							*/
/*----------------------------------------------------------------------------------------------------------*/
static void videoIn_SetPlanarFormat(
	BOOL bIsYUV420
	//E_VIDEOIN_PLANAR_FORMAT ePlanarFmt
)
{
	outp32(REG_VPEPAR, (inp32(REG_VPEPAR) & ~PNFMT) | 
					(bIsYUV420 ? PNFMT : 0));		
}

static E_VIDEOIN_PLANAR_FORMAT 
videoIn_GetPlanarFormat(void)
{
	return ((inp32(REG_VPEPAR) & PNFMT)>>7);
}


/*----------------------------------------------------------------------------------------------------------*/
/* Function: DrvVideoIn_SetMotionDet			  									                	   	*/
/*                                                                                                         	*/
/* Parameters:                                                                                             	*/
/*      bEnable: Enable Motion Detection																	*/ 
/*			FALSE : Disable MD																			   	*/
/*			TRUE : Enable MD																			   	*/
/*      bBlockSize: Motion Detection Block Size																*/ 
/*			FALSE : 16x16																				   	*/
/*			TRUE : 8x8																					   	*/
/*      bSaveMode: Motion Detection Save Mode  																*/ 
/*			FALSE : 1 bit DIFF + 7 Y Differential															*/
/*			TRUE : 	1 bit DIFF only 													   					*/
/* Returns:                                                                                                	*/
/*      None                                                                                               	*/
/*                                                                                                         	*/
/* Description:                                                                                            	*/
/*     	Set the motion detection parameter					               			                       	*/
/*                                                                                                         	*/
/*----------------------------------------------------------------------------------------------------------*/	
static void videoIn_SetMotionDet(
	BOOL bEnable,
	BOOL bBlockSize,	
	BOOL bSaveMode
)
{
	outp32(REG_VPEMD+g_u32PortOffset, (inp32(REG_VPEMD+g_u32PortOffset) & ~(MDSM | MDBS | MDEN)) |
			(((bEnable?MDEN:0) | (bBlockSize?MDBS:0)) | 
				(bSaveMode?MDSM:0)));	
}
	
static void videoIn_GetMotionDet(
	PBOOL pbEnable,
	PBOOL pbBlockSize,	
	PBOOL pbSaveMode
)
{
	UINT32 u32RegData = inp32(REG_VPEMD+g_u32PortOffset);
	*pbEnable = (u32RegData & MDEN);
	*pbBlockSize = (u32RegData & MDBS)>>8; 
	*pbSaveMode = (u32RegData & MDSM)>>9; 		
}
/*----------------------------------------------------------------------------------------------------------*/
/* Function: DrvVideoIn_SetMotionDetEx			  									                	   	*/
/*                                                                                                         	*/
/* Parameters:                                                                                             	*/
/*      u32DetFreq: MD detect frequency	(0~3)																*/ 
/*			0 : Each frame  																			   	*/
/*			1 :	Every 2 frame																	   			*/
/*			2 : every 3 frame																				*/
/*			3 : every 4 frame																				*/
/*      u32Threshold: Motion detection threshold (0~31)														*/ 
/*			(Value << 1) : Differential threshold 															*/
/*			Value 16 meaning threshold = 32																   	*/
/*      u32OutBuffer: Output buffer addree		  															*/ 
/* Returns:                                                                                                	*/
/*      None                                                                                               	*/
/*                                                                                                         	*/
/* Description:                                                                                            	*/
/*     	Set the motion detection parameter extention 					               			                       	*/
/*                                                                                                         	*/
/*----------------------------------------------------------------------------------------------------------*/			
static void videoIn_SetMotionDetEx(	
	UINT32 u32Threshold,
	UINT32 u32OutBuffer,
	UINT32 u32LumBuffer
)
{
	outp32(REG_VPEMD+g_u32PortOffset, (inp32(REG_VPEMD+g_u32PortOffset) & ~MDTHR) |	
				((u32Threshold <<16) & MDTHR) );
	outp32(REG_MDADDR+g_u32PortOffset, u32OutBuffer);	
	outp32(REG_MDYADDR+g_u32PortOffset, u32LumBuffer);		
}
static void videoIn_GetMotionDetEx(	
	PUINT32 pu32Threshold,
	PUINT32 pu32OutBuffer,
	PUINT32 pu32LumBuffer
)
{
	UINT32 u32RegData;
	u32RegData = inp32(REG_VPEMD+g_u32PortOffset); 
	//*pu32DetFreq = u32RegData & MDDF;
	*pu32Threshold = u32RegData & MDTHR;
	*pu32OutBuffer = inp32(REG_MDADDR+g_u32PortOffset);
	*pu32LumBuffer = inp32(REG_MDYADDR+g_u32PortOffset);
}	
static void videoIn_SetMotionDetFreq(UINT32 u32DetFreq)
{
	outp32(REG_VPEMD+g_u32PortOffset, (inp32(REG_VPEMD+g_u32PortOffset) & ~MDDF) |
				((u32DetFreq <<10) & MDDF) );
}	
static void videoIn_GetMotionDetFreq(PUINT32 pu32DetFreq)
{
	UINT32 u32RegData;
	u32RegData = inp32(REG_VPEMD+g_u32PortOffset); 
	*pu32DetFreq = u32RegData & MDDF;
}
//===================================================================================
//
//	One shutte or continuous mode
//
//
//
//
//
//
//	bIsOneSutterMode =  0. Continuous mode 
//		     			1. One shutte mode
//===================================================================================
static void 
videoIn_SetOperationMode(
	BOOL bIsOneSutterMode
	)
{
	outp32(REG_VPECTL+g_u32PortOffset, (inp32(REG_VPECTL+g_u32PortOffset) & ~CAPONE) | 
			((bIsOneSutterMode <<16) & CAPONE) );
} // DrvVideoIn_SetOperationMode

static BOOL 
videoIn_GetOperationMode(
	void
	)
{
	return ( (inp32(REG_VPECTL+g_u32PortOffset) & CAPONE) ? TRUE : FALSE );
} // DrvVideoIn_GetOperationMode


static UINT32 
videoIn_GetProcessedDataCount(    // Read Only
	E_VIDEOIN_PIPE ePipe	 		 // Planar or packet pipe		
	)
{
	if(ePipe == eVIDEOIN_PACKET)
		return inp32(REG_CURADDRP+g_u32PortOffset);				/* Packet pipe */
	else if (ePipe == eVIDEOIN_PLANAR)
		return inp32(REG_CURADDRY+g_u32PortOffset);				/* Planar pipe */
	else 
		return 0;		
} // DrvVideoIn_GetProcessedDataCount


static void 
videoIn_SetCropWinStartAddr(
	UINT32 u32VerticalStart, 
	UINT32 u32HorizontalStart
	)
{
	outp32(REG_VPECWSP+g_u32PortOffset, (inp32(REG_VPECWSP+g_u32PortOffset) & ~(CWSPV | CWSPH)) //(Y|X)
    | ((u32VerticalStart << 16)
    | u32HorizontalStart));
} // DrvVideoIn_SetCropWinStartAddr


static void 
videoIn_GetCropWinStartAddr(
	PUINT32 pu32VerticalStart, 
	PUINT32 pu32HorizontalStart
	)
{
	UINT32 u32Temp = inp32(REG_VPECWSP+g_u32PortOffset);
	
	*pu32VerticalStart = (u32Temp & CWSPV) >> 16;
	*pu32HorizontalStart = u32Temp & CWSPH;
} // DrvVideoIn_GetCropWinStartAddr

static void 
videoIn_SetCropWinSize(
	UINT32 u32Height, 
	UINT32 u32Width
	)
{
	outp32(REG_VPECWS+g_u32PortOffset, (inp32(REG_VPECWS+g_u32PortOffset) & ~(CWSH | CWSW)) 
    | ((u32Height << 16)
    | u32Width));
} // DrvVideoIn_SetCropWinSize


static void 
videoIn_GetCropWinSize(
	PUINT32 pu32Height, 
	PUINT32 pu32Width
	)
{
	UINT32 u32Temp = inp32(REG_VPECWS+g_u32PortOffset);
	
	*pu32Height = (u32Temp & CWSH) >> 16;
	*pu32Width = u32Temp & CWSW;
} // DrvVideoIn_GetCropWinSize

static INT32 
videoIn_SetVerticalScaleFactor(
	E_VIDEOIN_PIPE ePipe,
	UINT8 u8Numerator, 
	UINT8 u8Denominator
	)
{//Y direction 
	if(ePipe == eVIDEOIN_PACKET)
	{
		outp32(REG_VPEPKDS, (inp32(REG_VPEPKDS) & ~(DSVN | DSVM)) 
    	| ((u8Numerator << 24)
    	| (u8Denominator << 16)));
    }	
    else if(ePipe == eVIDEOIN_PLANAR)
    {
    	outp32(REG_VPEPNDS, (inp32(REG_VPEPNDS) & ~(DSVN | DSVM)) 
    	| ((u8Numerator << 24)
    	| (u8Denominator << 16)));	
    }
    else
		return E_VIDEOIN_INVALID_PIPE;
	return Successful;	
} // DrvVideoIn_SetVerticalScaleFactor
#if 0
static INT32 
DrvvideoIn_GetVerticalScaleFactor(
	E_VIDEOIN_PIPE ePipe,
	PUINT16 pu16Numerator, 
	PUINT16 pu16Denominator
	)
{

	UINT32 u32Temp1, u32Temp2; 
	if(ePipe == eVIDEOIN_PACKET)
	{
		u32Temp1 = inp32(REG_VPEPKDSL+g_u32PortOffset);
		u32Temp2 = inp32(REG_VPEPKDSH+g_u32PortOffset);
	}	
	else if(ePipe == eVIDEOIN_PLANAR)
	{
		u32Temp1 = inp32(REG_VPEPNDSL+g_u32PortOffset);
		u32Temp2 = inp32(REG_VPEPNDSH+g_u32PortOffset);
	}	
	else
		return E_VIDEOIN_INVALID_PIPE;	
	*pu16Numerator = ((u32Temp1 & DSVNL) >> 24) | (((u32Temp2 & DSVNH) >> 24)<<8);
	*pu16Denominator = (u32Temp1 & DSVML) >> 16 | (((u32Temp2 & DSVMH) >> 16)<<8);			

	return Successful;
} // DrvVideoIn_GetVerticalScaleFactor
#endif 

static INT32 
videoIn_SetHorizontalScaleFactor(
	E_VIDEOIN_PIPE bPipe,
	UINT8 u8Numerator, 
	UINT8 u8Denominator
	)
{
	DBG_PRINTF("%s\n",__FUNCTION__);	
	if(bPipe == eVIDEOIN_PACKET)
	{
		outp32(REG_VPEPKDS, (inp32(REG_VPEPKDS) & ~(DSHN | DSHM)) 
	    | ((u8Numerator << 8)
	    | u8Denominator));
	}
	else  if(bPipe == eVIDEOIN_PLANAR)
	{
		outp32(REG_VPEPNDS, (inp32(REG_VPEPNDS) & ~(DSHN | DSHM)) 
	    | ((u8Numerator << 8)
	    | u8Denominator));
	}	    
	else
		return E_VIDEOIN_INVALID_PIPE;	
	return Successful;
} // DrvVideoIn_SetHorizontalScaleFactor
#if 0
static INT32 
DrvvideoIn_GetHorizontalScaleFactor(
	E_VIDEOIN_PIPE bPipe,
	PUINT16 pu16Numerator, 
	PUINT16 pu16Denominator
	)
{
	UINT32 u32Temp1, u32Temp2;
	if(bPipe == eVIDEOIN_PACKET)
	{
		u32Temp1 = inp32(REG_VPEPKDSL+g_u32PortOffset);
		u32Temp2 = inp32(REG_VPEPKDSH+g_u32PortOffset);
	}	
	else  if(bPipe == eVIDEOIN_PLANAR)
	{
		u32Temp1 = inp32(REG_VPEPNDSL+g_u32PortOffset);
		u32Temp2 = inp32(REG_VPEPNDSH+g_u32PortOffset);
	}	
	else
		return E_VIDEOIN_INVALID_PIPE;				
	*pu16Numerator = ((u32Temp1 & DSHNL) >> 8) | (u32Temp2 & DSHNH);
	*pu16Denominator = (u32Temp1 & DSHML) | ((u32Temp2 & DSHMH)<<8);\
	return Successful;
} // DrvVideoIn_GetHorizontalScaleFactor
#endif
static void 
videoIn_SetFrameRateScaleFactor(
	UINT8 u8Numerator, 
	UINT8 u8Denominator
	)
{
	outp32(REG_VPEFRC+g_u32PortOffset, (inp32(REG_VPEFRC+g_u32PortOffset) & ~(FRCN | FRCM)) 
    | (((u8Numerator << 8) & FRCN)
    | (u8Denominator & FRCM)));
} // DrvVideoIn_SetFrameRateScaleFactor
#if 0
static void 
DrvvideoIn_GetFrameRateScaleFactor(
	PUINT8 pu8Numerator, 
	PUINT8 pu8Denominator
	)
{
	UINT32 u32Temp = inp32(REG_VPEFRC+g_u32PortOffset);
	
	*pu8Numerator = (u32Temp & FRCN) >> 8;
	*pu8Denominator = u32Temp & FRCM;
} // DrvVideoIn_GetFrameRateScaleFactor
#endif
#if 0
static void 
DrvvideoIn_SetAddressMatch(
	UINT32 u32AddressMatch
	)
{
	outp32(REG_CMPADDR+g_u32PortOffset, u32AddressMatch);
}
static void 
videoIn_GetAddressMatch(
	PUINT32 pu32AddressMatch
	)
{
	*pu32AddressMatch = inp32(REG_CMPADDR+g_u32PortOffset);
}
#endif
static void 
videoIn_SetStride(
	UINT32 u32PacketStride,
	UINT32 u32PlanarStride	
	)
{
	outp32(REG_VSTRIDE+g_u32PortOffset, ((u32PlanarStride<<16) & PNSTRIDE) | 
					(u32PacketStride & PKSTRIDE) );
}

static void 
videoIn_GetStride(
	PUINT32	pu32PacketStride,
	PUINT32 pu32PlanarStride	
	)
{
	UINT32 u32Tmp =  inp32(REG_VSTRIDE+g_u32PortOffset);
	*pu32PlanarStride = (u32Tmp & PNSTRIDE) >>16;
	*pu32PacketStride = u32Tmp & PKSTRIDE;	
}

static INT32 
videoIn_SetBaseStartAddress(
	E_VIDEOIN_PIPE ePipe,
	E_VIDEOIN_BUFFER eBuf,
	UINT32 u32BaseStartAddr
	)	
{
	if(ePipe==eVIDEOIN_PACKET)
	{
		if(eBuf>eVIDEOIN_BUF1)
			return E_VIDEOIN_INVALID_BUF;
		outp32(REG_PACBA0+g_u32PortOffset+eBuf*4, u32BaseStartAddr);				
	}	
	else if(ePipe==eVIDEOIN_PLANAR)
	{
		if(eBuf>eVIDEOIN_BUF2)
			return E_VIDEOIN_INVALID_BUF;
		outp32(REG_YBA0+g_u32PortOffset+eBuf*4, u32BaseStartAddr);						
	}	
	else
		return E_VIDEOIN_INVALID_PIPE;
	return Successful;	
}

static INT32 
videoIn_GetBaseStartAddress(
	E_VIDEOIN_PIPE ePipe,
	E_VIDEOIN_BUFFER eBuf,
	PUINT32 pu32BaseStartAddr
	)
{
	if(ePipe==eVIDEOIN_PACKET)
	{
		if(eBuf>eVIDEOIN_BUF1)
			return E_VIDEOIN_INVALID_BUF;
		*pu32BaseStartAddr = inp32(REG_PACBA0+g_u32PortOffset+eBuf*4);		
	}	
	else if(ePipe==eVIDEOIN_PLANAR)
	{
		if(eBuf>eVIDEOIN_BUF2)
			return E_VIDEOIN_INVALID_BUF;	
		*pu32BaseStartAddr = inp32(REG_YBA0+g_u32PortOffset+eBuf*4);						
	}	
	else
		return E_VIDEOIN_INVALID_PIPE;
	return Successful;		
}


/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvVideoIn_InstallInterrupt			  									                   */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*      u32FieldEnable:  																				   */ 
/*			00 = Both fields are disabled																   */
/*			01 = Field 1 enable																			   */
/*			10 = Field 2 enable																			   */
/*			11 = Both fields are enabled  	  		                                          			   */
/*      eInputType: 																					   */
/*			0 = CCIR601.	1=CCIR656	 					                                               */
/* 		bFieldSwap: 																					   */
/*			Swap field 1 and field 2																	   */
/* Returns:                                                                                                */
/*      None                                                                                               */
/*                                                                                                         */
/* Description:                                                                                            */
/*     	Set the input type, fileds enable or disable and fields swap       			                       */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

static void 
videoIn_SetInputType(
	UINT32 	u32FieldEnable,						//0: Both fields are disabled. 1: Field 1 enable. 2: Field 2 enable. 3: Both fields are enable
	E_VIDEOIN_TYPE eInputType,				//0: CCIR601.	1: CCIR656 	
	BOOL 	bFieldSwap							//1: Swap two field 
	)	
{
	outp32(REG_VPEPAR+g_u32PortOffset, (inp32(REG_VPEPAR+g_u32PortOffset) & ~(FLDSWAP | FLD1EN | FLD0EN | FLDSWAP)) | 
									((u32FieldEnable << 16) & (FLD1EN | FLD0EN )) |
									((eInputType <<1 ) & SNRTYPE) | 									
    								((bFieldSwap << 13) & FLDSWAP) );
} // DrvVideoIn_SetInputType
#if 0
static void 
videoIn_GetInputType(
	PUINT32 pu32FieldEnable,					//0: Both fields are disabled. 1: Field 1 enable. 2: Field 2 enable. 3: Both fields are enable
	E_VIDEOIN_TYPE* peInputType,				//0: CCIR601.	1: CCIR656 	
	PBOOL 	pbFieldSwap							//1: Swap two field 
	)	
{
	UINT32 u32Tmp = inp32(REG_VPEPAR+g_u32PortOffset);	
	*pu32FieldEnable = (u32Tmp & ( FLD1EN | FLD0EN ))>>16;
	*peInputType = (u32Tmp & SNRTYPE )>>1;
	*pbFieldSwap = (u32Tmp & FLDSWAP )>>13;
} // DrvVideoIn_GetInputType
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvVideoIn_IsStandardCCIR656			  								*/
/*                                                                                                         						*/
/* Parameters:                                                                                             					*/
/*      bIsStandard:           															*/ 
/*			0 = Non-Standard CCIR656 mode (OV7725 or Hynix 702) 					*/
/*			1 = Standard CCIR656 mode										  	*/
/* Returns:                                                                                                					*/
/*      None                                                                                              	 					*/
/*                                                                                                         						*/
/* Description:                                                                                            					*/
/*     	Set the CCIR656 mode  							    			                       	*/
/*                                                                                                         						*/
/*---------------------------------------------------------------------------------------------------------*/
#if 0
static void videoIn_IsStandardCCIR656(BOOL bIsStandard)
{
	if(bIsStandard==TRUE)
		outp32(REG_VPEPAR+g_u32PortOffset, inp32(REG_VPEPAR+g_u32PortOffset) & ~BIT18);		// Standard
	else
		outp32(REG_VPEPAR+g_u32PortOffset, inp32(REG_VPEPAR+g_u32PortOffset) | BIT18);			// Non-Standard 
}
#endif 
/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvVideoIn_SetFieldDetection			  									                   */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*      bDetPosition: Available if "bFieldDetMethod"=0              									   */ 
/*			00 = Vsync start 																   			   */
/*			01 = Vsync end																			 	   */
/*      bFieldDetMethod: 																			       */
/*				0: Detect field by Vsync & Hsync(SA711113). 											   */
/*				1: Detect field by field pin(WT8861)													   */
/* Returns:                                                                                                */
/*      None                                                                                               */
/*                                                                                                         */
/* Description:                                                                                            */
/*     	Set the input type, fileds enable or disable and fields swap       			                       */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
static void videoIn_SetFieldDetection(
	BOOL 	bDetPosition,					 	 	
	BOOL 	bFieldDetMethod						//0: Detect field by Vsync & Hsync(SA711113). 1:Detect field by field pin(WT8861)	 
)
{
	outp32(REG_VPEPAR+g_u32PortOffset, (inp32(REG_VPEPAR+g_u32PortOffset) & ~(FLDDETP | FLDDETM)) | 
									((bDetPosition << 15) & FLDDETP) |
    								((bFieldDetMethod << 14) & FLDDETM) );
} // DrvVideoIn_SetFieldDetection

#if 0
static void videoIn_GetFieldDetection(
	PBOOL 	pbDetPosition,	//0: Both fields are disabled. 1: Field 1 enable. 2: Field 2 enable. 3: Both fields are enable. It is only available as CCIR601 	 
	PBOOL 	pbFieldDetMethod//0: Detect field by Vsync & Hsync. 1:Detect field by CCIR656. It is only available as CCIR601	 
)
{
	UINT32 u32Tmp = inp32(REG_VPEPAR+g_u32PortOffset);		
	*pbDetPosition = (u32Tmp & FLDDETP) >> 15;
	*pbFieldDetMethod = (u32Tmp & FLDDETM) >> 14;	
} // DrvVideoIn_GetFieldDetection
#endif
/*----------------------------------------------------------------------------------------------------------*/
/* Function: DrvVideoIn_SetColorEffect			  									                   		*/
/*                                                                                                         	*/
/* Parameters:                                                                                             	*/
/*      eColorMode: Available as following             									  				 	*/ 
/*			eVIDEOIN_CEF_NORMAL = 0,																		*/
/*			eVIDEOIN_CEF_SEPIA = 1,																			*/
/*			eVIDEOIN_CEF_NEGATIVE = 2,																		*/
/*			eVIDEOIN_CEF_POSTERIZE = 3													   					*/
/* Returns:                                                                                                	*/
/*      None                                                                                               	*/
/*                                                                                                         	*/
/* Description:                                                                                            	*/
/*     	Set the special color mode     			                      						 				*/
/*                                                                                                         	*/
/*----------------------------------------------------------------------------------------------------------*/
#if 0
static INT32 videoIn_SetColorEffect(
	E_VIDEOIN_CEF 	eColorMode	
)					
{
	if(eColorMode>eVIDEOIN_CEF_POSTERIZE)
		return E_VIDEOIN_INVALID_COLOR_MODE;
	outp32(REG_VPEPAR+g_u32PortOffset, (inp32(REG_VPEPAR+g_u32PortOffset) & ~SCEF) | 
									(eColorMode<<11) );
	return Successful;								
} // DrvVideoIn_SetColorEffect
static INT32 
videoIn_SetColorEffectParameter(
	UINT8 	u8YComp,
	UINT8  	u8UComp,
	UINT8 	u8VComp
	)
{
	UINT32 u32Tmp = inp32(REG_VPEPAR+g_u32PortOffset);		
	UINT32 u32ColorMode = (u32Tmp & SCEF) >> 11;
	if(u32ColorMode == eVIDEOIN_CEF_SEPIA)
	{
		outp32(REG_VSEPIA+g_u32PortOffset,  (((UINT32)u8UComp<<8) | u8VComp));	
	}
	else if(u32ColorMode == eVIDEOIN_CEF_POSTERIZE)
	{	
		outp32(REG_VPOSTERIZE+g_u32PortOffset,  (((UINT32)u8YComp<<16) | ((UINT32)u8UComp<<8) | u8VComp));	
	}
	else
	{
		return E_VIDEOIN_WRONG_COLOR_PARAMETER;
	}		
	return Successful;		
}
#endif
#if 0
static void DrvvideoIn_GetColorEffect(
	E_VIDEOIN_CEF* 	peColorMode						
)
{
	UINT32 u32Tmp = inp32(REG_VPEPAR+g_u32PortOffset);		
	*peColorMode = (u32Tmp & SCEF) >> 11;
} // DrvvideoIn_GetColorEffect
static INT32
videoIn_GetColorEffectParameter(
	PUINT8 	pu8YComp,
	PUINT8  	pu8UComp,
	PUINT8 	pu8VComp
	)
{
	UINT32 u32Tmp = inp32(REG_VPEPAR+g_u32PortOffset);		
	UINT32 u32ColorMode = (u32Tmp & SCEF) >> 11;
	if(u32ColorMode == eVIDEOIN_CEF_SEPIA)
	{
		u32Tmp = inp32(REG_VSEPIA+g_u32PortOffset);
		*pu8UComp = (u32Tmp & 0xFF00)>>8;	
		*pu8VComp = u32Tmp & 0xFF;	
	}
	else if(u32ColorMode == eVIDEOIN_CEF_POSTERIZE)
	{	
		u32Tmp = inp32(REG_VPOSTERIZE+g_u32PortOffset);
		*pu8YComp = (u32Tmp & 0xFF0000)>>16;
		*pu8UComp = (u32Tmp & 0xFF00)>>8;	
		*pu8VComp = u32Tmp & 0xFF;	
	}
	else
	{
		return E_VIDEOIN_WRONG_COLOR_PARAMETER;
	}		
	return Successful;		
}
#endif
/* ===================================================================================================================================== */
unsigned short int vinGCD(UINT16 m1, UINT16 m2)
{
	UINT16 m;
	if(m1<m2)
	{
		m=m1; m1=m2; m2=m;
	}
	if(m1%m2==0)
		return m2;
	else
		return (vinGCD(m2,m1%m2));		
}

static INT32 videoIn_PreviewPipeSize(UINT16 u16height, UINT16 u16width)
{
	INT32 i32ErrCode;
	UINT32 u32cropwidth,  u32cropheight;
	UINT16 u16GCD;

	videoIn_GetCropWinSize(&u32cropheight, &u32cropwidth);	
	u16GCD = vinGCD(u16height,			
					u32cropheight); 	
	i32ErrCode = videoIn_SetVerticalScaleFactor(eVIDEOIN_PACKET, u16height/u16GCD, u32cropheight/u16GCD);
	if(i32ErrCode!=Successful)
		return i32ErrCode;

	u16GCD = vinGCD(u16width,			
					u32cropwidth); 		
	i32ErrCode = videoIn_SetHorizontalScaleFactor(eVIDEOIN_PACKET, u16width/u16GCD, u32cropwidth/u16GCD);
	return Successful;	
}
static INT32 videoIn_EncodePipeSize(UINT16 u16height, UINT16 u16width)
{
	INT32 i32ErrCode;
	UINT32 u32cropwidth,  u32cropheight;
	UINT16 u16GCD;

	videoIn_GetCropWinSize(&u32cropheight, &u32cropwidth);	
	u16GCD = vinGCD(u16height,			
					u32cropheight); 	
	i32ErrCode = videoIn_SetVerticalScaleFactor(eVIDEOIN_PLANAR, u16height/u16GCD, u32cropheight/u16GCD);
	if(i32ErrCode != Successful)
		return i32ErrCode;

	u16GCD = vinGCD(u16width,			
					u32cropwidth); 		
	i32ErrCode = videoIn_SetHorizontalScaleFactor(eVIDEOIN_PLANAR, u16width/u16GCD, u32cropwidth/u16GCD);
	return i32ErrCode;		
}



static VINDEV_T nvt_vin =
{
	videoIn_Init,				 	//void (*Init)(BOOL bIsEnableSnrClock, E_VIDEOIN_SNR_SRC eSnrSrc, UINT32 u32SensorFreqKHz, E_VIDEOIN_DEV_TYPE eDevType):
	videoIn_Open,					//INT32 (*Open)(UINT32 u32EngFreqKHz, UINT32 u32SensorFreqKHz);
	videoIn_Close,					//void (*Close)(void);
	videoIn_SetPipeEnable,			//void (*SetPipeEnable)(BOOL bEngEnable, E_VIDEOIN_PIPE ePipeEnable);
	videoIn_SetPlanarFormat,		//void (*SetPlanarFormat)(E_VIDEOIN_PLANAR_FORMAT ePlanarFmt);
	videoIn_SetCropWinSize,		//void (*SetCropWinSize)(UINT32 u32height, UINT32 u32width);
	videoIn_SetCropWinStartAddr,	//void (*SetCropWinStartAddr)(UINT32 u32VerticalStart, UINT32 u32HorizontalStart);
	videoIn_PreviewPipeSize,		//INT32 (*PreviewPipeSize)(UINT16 u32height, UINT16 u32width);
	videoIn_EncodePipeSize,		//INT32 (*EncodePipeSize)(UINT16 u32height, UINT16 u32width);
	videoIn_SetStride,				//void (*SetStride)(UINT32 u16packetstride, UINT32 u32planarstride);
	videoIn_GetStride,				//void (*GetStride)(PUINT32 pu32PacketStride, PUINT32 pu32PlanarStride);  								
	videoIn_EnableInt,				//INT32 (*EnableInt)(E_VIDEOIN_INT_TYPE eIntType);
	videoIn_DisableInt,			//INT32 (*DisableInt)(E_VIDEOIN_INT_TYPE eIntType);
#ifndef CONFIG_ARCH_W55FA93
	videoIn_InstallCallback,		//INT32 (*InstallCallback)(E_VIDEOIN_INT_TYPE eIntType, PFN_VIDEOIN_CALLBACK pfnCallback, PFN_VIDEOIN_CALLBACK *pfnOldCallback);
#endif
	videoIn_SetBaseStartAddress,	//INT32 (*SetBaseStartAddress(E_VIDEOIN_PIPE ePipe, E_VIDEOIN_BUFFER eBuf, UINT32 u32BaseStartAddr);					
	videoIn_SetOperationMode,		//void (*SetOperationMode(BOOL bIsOneSutterMode)			
	videoIn_GetOperationMode,				//BOOL (*GetOperationMode)(void);
	videoIn_SetPacketFrameBufferControl, 	//void (*videoIn1_SetPacketFrameBufferControl)(BOOL bFrameSwitch, BOOL bFrameBufferSel);
	videoIn_SetSensorPolarity,				//void (*videoIn1_SetSensorPolarity)(BOOL bVsync, BOOL bHsync,  BOOL bPixelClk);

	//videoIn_SetColorEffect, 				//INT32 (*videoIn_SetColorEffect)(E_VIDEOIN_CEF eColorMode);					
	//videoIn_SetColorEffectParameter,		//INT32 (*SetColorEffectParameter)(UINT8 u8YComp, UINT8 u8UComp, UINT8 u8VComp);
	videoIn_SetDataFormatAndOrder, 		//void (*SetDataFormatAndOrder)(E_VIDEOIN_ORDER eInputOrder, E_VIDEOIN_IN_FORMAT eInputFormat, E_VIDEOIN_OUT_FORMAT eOutputFormat)
	videoIn_SetMotionDet,				//void (*SetMotionDet)(BOOL bEnable, BOOL bBlockSize,	BOOL bSaveMode);
	videoIn_SetMotionDetEx,			//void (*SetMotionDetEx)(UINT32 u32Threshold, UINT32 u32OutBuffer, UINT32 u32LumBuffer)
	videoIn_SetMotionDetFreq,			//void (*SetMotionDetFreq)(UINT32 u32DetFreq);
	videoIn_SetInputType,				//void (*SetInputType)(UINT32 u32FieldEnable, E_VIDEOIN_TYPE eInputType,	BOOL bFieldSwap);
	videoIn_SetFieldDetection,			//void (*SetFieldDetection)(BOOL bDetPosition, BOOL bFieldDetMethod);
	videoIn_SetFrameRateScaleFactor,	//void SetFrameRateScaleFactor(UINT8 u8Numerator, UINT8 u8Denominator);

	videoIn_SetShadowRegister,			//void(*SetShadowRegister)(void);

	/* Not use */
	videoIn_SetInitFrame,				//void (*SetInitFrame)(void);
	videoIn_GetSkipFrame,				//UINT32 (*GetSkipFrame)(void);
	videoIn_IsIntEnabled, 				//BOOL (*IsIntEnabled)(E_VIDEOIN_INT_TYPE eIntTyp);
	videoIn_GetPacketFrameBufferControl,	//void (*GetPacketFrameBufferControl)(PBOOL pbFrameSwitch, PBOOL pbFrameBufferSel);
	videoIn_ClearInt,					//INT32 (*ClearInt)(E_VIDEOIN_INT_TYPE eIntType)

	videoIn_Reset,						//void (*Reset)(void);
	videoIn_PollInt,					//BOOL (*PollInt)(void);
	videoIn_GetPipeEnable,				//void (*GetPipeEnable)(PBOOL pbEngEnable,E_VIDEOIN_PIPE* pePipeEnable);
	videoIn_GetSensorPolarity,			//void (*GetSensorPolarity)(PBOOL pbVsync, PBOOL pbHsync, PBOOL pbPixelClk);
	videoIn_GetDataFormatAndOrder,		//void (*GetDataFormatAndOrder)(E_VIDEOIN_ORDER* peInputOrder, E_VIDEOIN_IN_FORMAT* peInputFormat,E_VIDEOIN_OUT_FORMAT* peOutputFormat);
	videoIn_GetPlanarFormat,			//E_VIDEOIN_PLANAR_FORMAT (*GetPlanarFormat)(void);
	videoIn_GetMotionDet,				//void (*GetMotionDet)(PBOOL pbEnable, PBOOL pbBlockSize,PBOOL pbSaveMode);
	videoIn_GetMotionDetEx, 			//void (*GetMotionDetEx(PUINT32 pu32Threshold, PUINT32 pu32OutBuffer,PUINT32 pu32LumBuffer);
	videoIn_GetProcessedDataCount,		//UINT32 (*GetProcessedDataCount)(E_VIDEOIN_PIPE ePipe);
	videoIn_GetCropWinStartAddr, 		//void (*GetCropWinStartAddr)(PUINT32 pu32VerticalStart, PUINT32 pu32HorizontalStart);
	videoIn_GetMotionDetFreq,			//void (*GetMotionDetFreq)(PUINT32 pu32DetFreq);

	videoIn_GetBaseStartAddress,		//INT32 (*GetBaseStartAddress)(E_VIDEOIN_PIPE ePipe, E_VIDEOIN_BUFFER eBuf, PUINT32 pu32BaseStartAddr);
};

INT32 register_vin_port1_device(VINDEV_T* pVinDev)
{
	*pVinDev = nvt_vin;
	return Successful;	
}

