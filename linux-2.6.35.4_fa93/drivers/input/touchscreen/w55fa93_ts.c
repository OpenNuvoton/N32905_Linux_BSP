/* linux/driver/input/w55fa93_ts.c
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Changelog:
 *
 *   2007/01/26     vincen.zswan add this file for nuvoton touch screen driver.
 *   2008/08/29     Ccwang add spinlock for QT use bug of touch screen driver.
 */
 
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/interrupt.h>
//#include <linux/input.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/w55fa93_ts.h>
#include <mach/w55fa93_reg.h>
#undef BIT
#include <linux/input.h>
#include <linux/clk.h>

#define BIT(x)  (1UL<<((x)%BITS_PER_LONG))
#define LONG(x) ((x)/BITS_PER_LONG)
#undef inl
#undef outl
#define inl			readl
#define outl		writel

#define INTERVAL_TIME  HZ/50	/* HZ = 100. INTERVAL_TIME=2 ==> 20ms */
/* For  touch panel from channel 5 and 6*/
#define SET_WT_MODE	 do{\
								outl( ((inl(REG_ADC_CON) & ~(((WT_INT_EN |  LVD_INT) | ADC_INT_EN) | ADC_TSC_MODE))  |\
								ADC_TSC_MODE), REG_ADC_CON);\
						}while(0)
#define SET_WT_MODE_I	 do{\
								outl( ((inl(REG_ADC_CON) & ~(((WT_INT_EN  |  LVD_INT) |ADC_INT_EN) | ADC_TSC_MODE)) | \
								(ADC_TSC_MODE | WT_INT_EN)), REG_ADC_CON);\
							}while(0)
#define SET_AUTO_MODE    do{\
								outl( ((inl(REG_ADC_CON) & ~(((WT_INT_EN |  LVD_INT) | ADC_INT_EN) | ADC_TSC_MODE))  |\
								((ADC_INT_EN | (2<<14)) |ADC_CONV ) ), REG_ADC_CON);\
							}while(0)

/* For low voltage detection from channel 2 or 3 or 4*/
#define SET_NORMAL_AIN2  do{\
								outl( (((((ADC_INT_EN | ADC_CON_ADC_EN) | ADC_CONV) | ADC_INT)) | (2<<9)), \
								REG_ADC_CON);\
							}while(0)
#define SET_NORMAL_AIN3  do{\
								outl( (ADC_INT_EN | ADC_CON_ADC_EN | ADC_CONV | ADC_INT | (3<<9)), \
								REG_ADC_CON);\
							}while(0)
#define SET_NORMAL_AIN4  do{\
								outl( (ADC_INT_EN  | ADC_CON_ADC_EN | ADC_CONV |  ADC_INT | (4<<9)), \
								REG_ADC_CON);\
							}while(0)

#define SET_NORMAL_AIN2_WI  do{\
								outl( (ADC_CON_ADC_EN | ADC_CONV | ADC_INT | (2<<9)), \
								REG_ADC_CON);\
							}while(0)

#define SET_NORMAL_AIN2_WI_WR \
							while(  (inl(REG_ADC_CON) & ADC_INT) != ADC_INT ) 


#define SET_NORMAL_AIN3_WI  do{\
								outl( (ADC_CON_ADC_EN | ADC_CONV | ADC_INT | (3<<9)), \
								REG_ADC_CON);\
							}while(0)

#define SET_NORMAL_AIN3_WI_WR \
							while(  (inl(REG_ADC_CON) & ADC_INT) != ADC_INT ) 

#define W55FA93_ADC_STATE_WT       	0
#define W55FA93_ADC_STATE_AUTO     	1
#define W55FA93_ADC_STATE_LIGHT    	2
#define W55FA93_ADC_STATE_BATTERY  	3


static struct input_dev *w55fa93_dev;

u8 volatile state;
static int irqnum;
static struct timer_list ts_timer;
static u32 count;



spinlock_t spin_ts_opc = SPIN_LOCK_UNLOCKED;	
int i32TsOpenCount = 0;	
/*
	The spin lock is used to protect interrupt channel of ADC 
*/
spinlock_t spin_adc_int = SPIN_LOCK_UNLOCKED;			
#define LOCK(x)		do{\
						spin_lock_irqsave(&spin_adc_int, x);\
					}while(0)

#define UNLOCK(x)	do{\
						spin_unlock_irqrestore(&spin_adc_int, x);\
					}while(0)

u32 w55fa93_ts_pressing = 0;
EXPORT_SYMBOL(w55fa93_ts_pressing);

//#define CONFIG_ADC_LIGHT_BATTERY
#ifdef CONFIG_ADC_LIGHT_BATTERY
// Check ambient light and voltage no faster than every 30 sec. 
//#define AIN_FREQ 30* HZ
#define AIN_FREQ 1*HZ

static u32 lastCheck = 0; // record last time check ambient light and volt.
static struct timer_list ain_timer;	
static unsigned int w55fa93_lux = 0;
static unsigned int w55fa93_vol = 0;
static unsigned int do_check = 1;
//EXPORT_SYMBOL(w55fa93_lux);
//EXPORT_SYMBOL(w55fa93_vol);

static struct platform_device *sys_adc;

#endif
//#define DBG_PRINTF 	printk
#define DBG_PRINTF(...)		
#define DBG_PRINTF2(...)
/*========================================
	For sysmgr to disable touch if backlight disable 
========================================*/
static u32 bisEnableTouch=1;
void disable_touch(void)
{
	printk("ADC: %s\n",__FUNCTION__);
	bisEnableTouch = 0;
}
void enable_touch(void)
{
	printk("ADC: %s\n",__FUNCTION__);
	bisEnableTouch = 1;
}
EXPORT_SYMBOL(disable_touch);	
EXPORT_SYMBOL(enable_touch);	

void adc_enable_irq(void)
{
	outl((1 << irqnum ), REG_AIC_MECR);		
  	//enable_irq(irqnum);
}
void adc_disable_irq(void)
{
	outl((1 << irqnum ), REG_AIC_MDCR);
  	//disable_irq(irqnum);
}
/*========================================*/
uint16_t u16LastX=0;
uint16_t u16LastY=0;
void report_touch(u32 u32x, u32 u32y)
{	
	if(bisEnableTouch==1)
	{
		input_report_key(w55fa93_dev, BTN_TOUCH, 1);
		u16LastX = u32x; 
		u16LastY = u32y;	
		input_report_abs(w55fa93_dev, ABS_X, u32x);
		input_report_abs(w55fa93_dev, ABS_Y, u32y);
		input_report_abs(w55fa93_dev, ABS_PRESSURE, 1000);	
		input_sync(w55fa93_dev);
	}
}

/*======================================================
FA91
	AIN2: Light
	AIN3: Battery		

FA93:
	AIN2: Battery
	The function is the registered timer callback function
	for low voltage detection(AIN2) and LCD lux detection(AIN3).  
======================================================*/
static void timer_check_touch(unsigned long dummy)
{	
	//DBG_PRINTF("ADC: %s\n",__FUNCTION__);	

	unsigned long flags;
	

  	//outl((1 << irqnum ), REG_AIC_MDCR);
    adc_disable_irq();

	LOCK(flags);	
	/*2012-04-24*/
	if(state != W55FA93_ADC_STATE_WT)
	{
		mod_timer(&ts_timer, jiffies + INTERVAL_TIME); 
		UNLOCK(flags);	
		//outl((1 << irqnum ), REG_AIC_MECR); 
		adc_enable_irq();		
		return;
	}

	if( ((inl(REG_ADC_TSC)&0x1)==0x1) ) 
	{//report down/up state in WT mode. The bit is only workable in wake for trigger mode. 
		SET_AUTO_MODE;
		state = W55FA93_ADC_STATE_AUTO;
		w55fa93_ts_pressing = 1;
		DBG_PRINTF("SET_AUTO_MODE\n");
	}
	else
	{
		if( (u16LastX!=0) && (u16LastY!=0) ){
			input_report_abs(w55fa93_dev, ABS_X, u16LastX);
			input_report_abs(w55fa93_dev, ABS_Y, u16LastX);	  	
		  	input_report_key(w55fa93_dev, BTN_TOUCH, 0);
			input_report_abs(w55fa93_dev, ABS_PRESSURE, 0);
			input_sync(w55fa93_dev);
		}
		count = 0;
#ifdef CONFIG_ADC_LIGHT_BATTERY
	#if 0
		if(lastCheck>jiffies)	/* To avoid the jiffies overflow. jiffies start from FFFF0000*/
			lastCheck=0;	
		if(jiffies >= lastCheck + AIN_FREQ && do_check == 1) 
		{  
			state = W55FA93_ADC_STATE_BATTERY;
			DBG_PRINTF("SET_NORMAL_AIN2\n");
			SET_NORMAL_AIN2;				
		} 
		else 
		{
			DBG_PRINTF("SET_WT_MODE_I\n");
			SET_WT_MODE_I;
			state = W55FA93_ADC_STATE_WT;						
			w55fa93_ts_pressing = 0;
		}
	#else
		SET_WT_MODE_I;
		state = W55FA93_ADC_STATE_WT;		 				
		w55fa93_ts_pressing = 0;	
	#endif	
#else
		DBG_PRINTF("SET_WT_MODE_I");
		SET_WT_MODE_I;
		state = W55FA93_ADC_STATE_WT;
		w55fa93_ts_pressing = 0;
#endif
	}
	mod_timer(&ts_timer, jiffies + INTERVAL_TIME);
		UNLOCK(flags);	
	//outl((1 << irqnum ), REG_AIC_MECR);
	adc_enable_irq();

	return;
}

#ifdef CONFIG_ADC_LIGHT_BATTERY
/*
	Disable ADC Interupt to avoid the race condition
	

*/
static void timer_check_ain(unsigned long dummy)
{
	//DBG_PRINTF2("ADC: %s\n",__FUNCTION__);

	unsigned long flags;	
	LOCK(flags);	

  	//outl((1 << irqnum ), REG_AIC_MDCR);
  	adc_disable_irq();					
  	if(state != W55FA93_ADC_STATE_WT) 
	{// pen down, touch gets higher priority				

		mod_timer(&ain_timer, jiffies + 1); 	/* Pending the battery detection after 10ms if state is not WT mode */	
		DBG_PRINTF("ADC: %s meet auto mode\n",__FUNCTION__);
		UNLOCK(flags);	
		adc_enable_irq();
		return;	
  	}
  	if(do_check == 1) 
	{  	
		//printk("Ain conversion\n");
	#if 0		
		state = W55FA93_ADC_STATE_BATTERY;
		DBG_PRINTF("SET_NORMAL_AIN2");
  		SET_NORMAL_AIN2;
	#else
		state = W55FA93_ADC_STATE_LIGHT
		DBG_PRINTF("SET_NORMAL_AIN3");
  		SET_NORMAL_AIN3;
	#endif
		/* After conversion done , timer will be created */
  	} 
	else
		mod_timer(&ain_timer, jiffies + AIN_FREQ);
	//outl((1 << irqnum ), REG_AIC_MECR);		//5/9 ADD
  	adc_enable_irq();
	UNLOCK(flags);
  	return;
} 

#endif

//static irqreturn_t adc_isr(int irq, void *dev_id, struct pt_regs *regs)
static irqreturn_t adc_isr(int irq, void *dev_id)
{
	unsigned long reg;
	u32 volatile xPos, yPos;
	//unsigned long flags;	

	DBG_PRINTF2("ADC: %s\n",__FUNCTION__);
    	reg = inl(REG_ADC_CON);
	if( (reg & WT_INT) && (state == W55FA93_ADC_STATE_WT) ) 
	{	//wait for trigger	
		outl((inl(REG_ADC_TSC) | ADC_PU_EN), REG_ADC_TSC); 
		__raw_writel(__raw_readl(REG_ADC_CON) & ~WT_INT, REG_ADC_CON);
		__raw_writel(__raw_readl(REG_ADC_CON) | WT_INT, REG_ADC_CON);
		outl((inl(REG_ADC_TSC) & ~ADC_PU_EN), REG_ADC_TSC); 

		outl( inl(REG_ADC_CON) | WT_INT | LVD_INT | ADC_INT, REG_ADC_CON);	
		DBG_PRINTF("WT INT = 0x%x\n", inl(REG_ADC_CON));
		

		w55fa93_ts_pressing = 1;
		state = W55FA93_ADC_STATE_AUTO;
		SET_AUTO_MODE;			/* It will disable WT_INT_EN. Otherwise  WT_INT always interrupt system */
		DBG_PRINTF("SET_AUTO_MODE");
  	} 
	else if((reg & ADC_INT) && (state == W55FA93_ADC_STATE_AUTO)) 
	{  // auto conv end		
		outl( inl(REG_ADC_CON) | WT_INT | LVD_INT | ADC_INT, REG_ADC_CON);	
		if((inl(REG_ADC_TSC) & ADC_TSC_MAV_EN) )
		{				
			xPos= inl(REG_TSC_MAV_X);
			yPos= inl(REG_TSC_MAV_Y);
		}
		else
		{	
			xPos = inl(REG_ADC_XDATA) & 0xFFFF;
			yPos = inl(REG_ADC_YDATA) & 0xFFFF;	
		}

		//Add 2012-04-24
		state = W55FA93_ADC_STATE_WT;	/* Why add the statement will cause crash ???*/

		SET_WT_MODE;
		udelay(2);	//Wait for WT state stable (not stable)
		//udelay(100);	//Wait for WT state stable 

		if((inl(REG_ADC_TSC)&0x1)==0x1)	//Pen still in down state
		{
			DBG_PRINTF("X Y = %d, %d\n", xPos, yPos);
			report_touch(xPos, yPos);
			count = 0;		
		}
		else
			count = count+1;
	  	mod_timer(&ts_timer, jiffies + INTERVAL_TIME);
	}
#ifdef CONFIG_ADC_LIGHT_BATTERY
	else if((reg & ADC_INT) && (state == W55FA93_ADC_STATE_BATTERY)) 
	{
		outl( inl(REG_ADC_CON) | WT_INT | LVD_INT | ADC_INT, REG_ADC_CON);	
		
		w55fa93_vol = inl(REG_ADC_XDATA);
		DBG_PRINTF("vol = %x\n", w55fa93_vol);

		lastCheck = jiffies;
	
		mod_timer(&ain_timer, jiffies + AIN_FREQ);
		state = W55FA93_ADC_STATE_WT;
		w55fa93_ts_pressing = 0;
		SET_WT_MODE_I;
		DBG_PRINTF("SET_WT_MODE_I");			
	} 
	else if((reg & ADC_INT) && (state == W55FA93_ADC_STATE_LIGHT)) 
	{
		outl( inl(REG_ADC_CON) | WT_INT | LVD_INT | ADC_INT, REG_ADC_CON);	
		w55fa93_lux = inl(REG_ADC_XDATA);
		DBG_PRINTF("lux = %x\n", w55fa93_lux);
		state = W55FA93_ADC_STATE_BATTERY;
		//outl((inl(REG_ADC_CON) & ~0xFFFE00) | 0x222600, REG_ADC_CON);
		SET_NORMAL_AIN2;
		DBG_PRINTF("SET_NORMAL_AIN2");	
	}
    
#endif
	else 
	{ // darn, unknown status. It should be wait for trigger interrupt. Change state to AUTO mode.
		outl((inl(REG_ADC_TSC) | ADC_PU_EN), REG_ADC_TSC); 
		__raw_writel(__raw_readl(REG_ADC_CON) & ~WT_INT, REG_ADC_CON);
		__raw_writel(__raw_readl(REG_ADC_CON) | WT_INT, REG_ADC_CON);
		outl((inl(REG_ADC_TSC) & ~ADC_PU_EN), REG_ADC_TSC); 

		outl( inl(REG_ADC_CON) | WT_INT | LVD_INT | ADC_INT, REG_ADC_CON);	
		DBG_PRINTF("WT INT = 0x%x\n", inl(REG_ADC_CON));
		

		w55fa93_ts_pressing = 1;
		state = W55FA93_ADC_STATE_AUTO;
		SET_AUTO_MODE;			/* It will disable WT_INT_EN. Otherwise  WT_INT always interrupt system */
		DBG_PRINTF("SET_AUTO_MODE");
	}
	return IRQ_HANDLED;	
}

//static int w55fa93ts_open(struct inode *inode, struct file *file)
static int w55fa93ts_open(struct input_dev *dev)
{  
		    
	//int result;
	//int irq;

	DBG_PRINTF2("ADC: %s\n",__FUNCTION__);

	spin_lock(&spin_ts_opc);
	i32TsOpenCount++;
	if(i32TsOpenCount != 1)
	{// Ts has open,  
		spin_unlock(&spin_ts_opc);
		return 0;
	}
	else
		spin_unlock(&spin_ts_opc);

	init_timer(&ts_timer);
#ifdef CONFIG_ADC_LIGHT_BATTERY
	init_timer(&ain_timer);
#endif
	
	w55fa93_ts_pressing = 0;
  	ts_timer.function = timer_check_touch;	/* timer handler */

#ifdef CONFIG_ADC_LIGHT_BATTERY
	ain_timer.function = timer_check_ain;
	mod_timer(&ain_timer, jiffies + AIN_FREQ);
#endif

#if 0
  	/* reset */
	outl(inl(REG_APBIPRST) | ADCRST, REG_APBIPRST);
	udelay(10);
	outl(inl(REG_APBIPRST) & ~ADCRST, REG_APBIPRST);	
	udelay(10);
#endif
	outl(inl(REG_AUDIO_CON) | AUDIO_RESET, REG_AUDIO_CON);
	outl(inl(REG_AUDIO_CON) & ~AUDIO_RESET, REG_AUDIO_CON);
	outl(0x500000, REG_AUDIO_CON);
	outl(inl(REG_ADC_CON) | ADC_RST, REG_ADC_CON);
	outl(inl(REG_ADC_CON) & ~ADC_RST, REG_ADC_CON);
	outl(inl(REG_AGCP1) & ~0x80000000, REG_AGCP1);	// Disable EDMA for ADC
	


	//Need to modify this value if need
	outl(0x180, REG_ADC_DLY);	

#ifdef CONFIG_ADC_LIGHT_BATTERY
	/* For report 1th battery */	
	SET_NORMAL_AIN2_WI;
	SET_NORMAL_AIN2_WI_WR;
	w55fa93_vol = inl(REG_ADC_XDATA);
	outl( inl(REG_ADC_CON) | WT_INT | LVD_INT | ADC_INT, REG_ADC_CON);	
	SET_NORMAL_AIN3_WI;
	SET_NORMAL_AIN3_WI_WR;
	w55fa93_lux = inl(REG_ADC_XDATA);
	outl( inl(REG_ADC_CON) | WT_INT | LVD_INT | ADC_INT, REG_ADC_CON);	
#endif

	/* waiting for trigger mode */
	outl( (inl(REG_ADC_CON) | (WT_INT_EN | ADC_TSC_MODE | WT_INT | ADC_INT | ADC_CON_ADC_EN)) & 
			~ADC_DIV, REG_ADC_CON);
	state = W55FA93_ADC_STATE_WT;	
  	DBG_PRINTF("Open: W55FA93_ADC_STATE_WT");	

	outl((inl(REG_ADC_TSC) | ADC_PU_EN), REG_ADC_TSC); 
	__raw_writel(__raw_readl(REG_ADC_CON) & ~WT_INT, REG_ADC_CON);
	__raw_writel(__raw_readl(REG_ADC_CON) | WT_INT, REG_ADC_CON);
	outl((inl(REG_ADC_TSC) & ~ADC_PU_EN), REG_ADC_TSC); 

	//adc_enable_irq();	
	enable_irq(irqnum);	
	return 0;
}



//static int w55fa93ts_close(struct inode *inode, struct file *file)
static void w55fa93ts_close(struct input_dev *dev)
{
  	DBG_PRINTF2("ADC: %s\n",__FUNCTION__);
	
	spin_lock(&spin_ts_opc);
	i32TsOpenCount--;
	if(i32TsOpenCount<0)
		i32TsOpenCount = 0;
	if(i32TsOpenCount != 0)
	{// Ts has open,  
		spin_unlock(&spin_ts_opc);
		return;
	}
	else
		spin_unlock(&spin_ts_opc);

	spin_adc_int = SPIN_LOCK_UNLOCKED;
	
  	//outl((1 << irqnum ), REG_AIC_MDCR);
	disable_irq(irqnum);

	del_timer(&ts_timer);
#ifdef CONFIG_ADC_LIGHT_BATTERY
	del_timer(&ain_timer);
#endif
	//return 0;
}
/*
	Both functions for audio record from ADC. 
	Touch panel and audio record form ADC share same IP. 
	If enable audio record from ADC, the touch panel and LVD is useless.
*/
//#ifdef CONFIG_SOUND_W55FA93_RECORD_ADC
#ifdef CONFIG_SND_SOC_W55FA93_ADC
EXPORT_SYMBOL(w55fa93ts_open_again);
int w55fa93ts_open_again(void)
{
	//int result;
	//int irq;
	struct clk *clk;

	DBG_PRINTF("ADC: %s\n",__FUNCTION__);
	
	//outl(inl(REG_APBCLK) | ADC_CKE, REG_APBCLK);		
	clk = clk_get(NULL, "ADC");
	clk_enable(clk);

	outl(inl(REG_CLKDIV3) & ~(ADC_N1 | ADC_S | ADC_N0), REG_CLKDIV3);	//Come from XTAL directly 
	init_timer(&ts_timer);
#ifdef CONFIG_ADC_LIGHT_BATTERY
	init_timer(&ain_timer);
#endif
#if 1
	adc_enable_irq();	
#else
	//outl((1 << irqnum ), REG_AIC_MDCR);
  	adc_disable_irq();		
#endif
	w55fa93_ts_pressing = 0;
  	ts_timer.function = timer_check_touch;	/* timer handler */

#ifdef CONFIG_ADC_LIGHT_BATTERY
	ain_timer.function = timer_check_ain;
	mod_timer(&ain_timer, jiffies + HZ/50);
#endif

  	/* reset */
	outl(inl(REG_CLKDIV3) & ~(ADC_N1 | ADC_S | ADC_N0), REG_CLKDIV3);

	outl(inl(REG_APBIPRST) | ADCRST, REG_APBIPRST);
	udelay(10);
	outl(inl(REG_APBIPRST) & ~ADCRST, REG_APBIPRST);	
	udelay(10);	
	outl(inl(REG_AUDIO_CON) | AUDIO_RESET, REG_AUDIO_CON);
	udelay(10);	
	outl(inl(REG_AUDIO_CON) & ~AUDIO_RESET, REG_AUDIO_CON);
	udelay(10);	
	outl(inl(REG_ADC_CON) | ADC_RST, REG_ADC_CON);
	udelay(10);	
	outl(inl(REG_ADC_CON) & ~ADC_RST, REG_ADC_CON);
	udelay(10);	
	//Need to modify this value if need
	outl(0x180, REG_ADC_DLY);	

#ifdef CONFIG_ADC_LIGHT_BATTERY
	/* For report 1th battery */	
	SET_NORMAL_AIN2_WI;
	SET_NORMAL_AIN2_WI_WR;
	w55fa93_vol = inl(REG_ADC_XDATA);
#endif

	/* waiting for trigger mode */
	outl( (inl(REG_ADC_CON) | (WT_INT_EN | ADC_TSC_MODE | WT_INT | ADC_INT | ADC_CON_ADC_EN)) & 
			~ADC_DIV, REG_ADC_CON);
	state = W55FA93_ADC_STATE_WT;	
  	DBG_PRINTF("Open: W55FA93_ADC_STATE_WT");	
	
	return 0;
}
EXPORT_SYMBOL(w55fa93ts_close_again);
int w55fa93ts_close_again(void)
{
  	DBG_PRINTF("ADC: %s\n",__FUNCTION__);
  	//outl((1 << irqnum ), REG_AIC_MDCR);
	adc_disable_irq();
	del_timer(&ts_timer);
#ifdef CONFIG_ADC_LIGHT_BATTERY
	del_timer(&ain_timer);
#endif
	return 0;
} 
#endif

static int __devinit w55fa93ts_probe(struct platform_device *pdev)
{
	int irq,result,err;
	struct clk* clk;

	DBG_PRINTF("ADC: %s\n",__FUNCTION__);
	//outl(inl(REG_APBCLK) | ADC_CKE, REG_APBCLK);
	clk = clk_get(NULL, "ADC");
	clk_enable(clk);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq for device\n");
		return -ENOENT;
	} else {
	  irqnum = irq;
	}

	if (!request_mem_region((unsigned long)W55FA93_VA_ADC, SZ_4K, "w55fa93-ts")) {
		return -EBUSY;
	}


	if (!(w55fa93_dev = input_allocate_device())) {
		printk(KERN_ERR "w55fa93_dev: not enough memory\n");
		err = -ENOMEM;
		goto fail;
	}

	w55fa93_dev->name = "W55FA93 TouchScreen";
	w55fa93_dev->phys = "w55fa93/event0";
	w55fa93_dev->id.bustype = BUS_HOST;
	w55fa93_dev->id.vendor  = 0x0005;
	w55fa93_dev->id.product = 0x0001;
	w55fa93_dev->id.version = 0x0100;

	w55fa93_dev->open    = w55fa93ts_open;
	w55fa93_dev->close   = w55fa93ts_close;

	w55fa93_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_ABS) | BIT(EV_SYN);
	w55fa93_dev->keybit[LONG(BTN_TOUCH)] = BIT(BTN_TOUCH);
	input_set_abs_params(w55fa93_dev, ABS_X, 0, 0x400, 0, 0);
	input_set_abs_params(w55fa93_dev, ABS_Y, 0, 0x400, 0, 0);
	input_set_abs_params(w55fa93_dev, ABS_PRESSURE, 0, 1000, 0, 0);

  	result = request_irq(irq, adc_isr,  IRQF_DISABLED | IRQF_SHARED, "ADC", w55fa93_dev);
	//outl((1 << irqnum ), REG_AIC_MDCR);
  	disable_irq(irqnum);
	if(result!=0)
		printk("register ADC ISR failed!\n");
	input_register_device(w55fa93_dev);
	DBG_PRINTF("Register touch screen success\n");
	
	return 0;
fail:	
	input_free_device(w55fa93_dev);
	return err;
}

static int w55fa93ts_remove(struct platform_device *pdev)
{
	DBG_PRINTF2("ADC: %s\n",__FUNCTION__);
	input_unregister_device(w55fa93_dev);
    return 0;
}

static struct platform_driver w55fa93ts_driver = {
	.remove		= w55fa93ts_remove,
	.driver		= {
		.name	= "w55fa93-ts",
		.owner	= THIS_MODULE,
	},
};



#ifdef CONFIG_ADC_LIGHT_BATTERY
#include <linux/platform_device.h>

static ssize_t read_adc(struct device *dev, struct device_attribute *attr, char *buf)
{
	DBG_PRINTF2("ADC: %s\n",__FUNCTION__);

	//printk("w55fa93_vol =0x%x\n",w55fa93_vol );

        buf[0] = (char)(w55fa93_lux & 0xff);
	buf[1] = (char)((w55fa93_lux & 0xff00) >> 8);
	buf[2] = (char)(w55fa93_vol & 0xff);
	buf[3] = (char)((w55fa93_vol & 0xff00) >> 8);

        return 4;
}

static ssize_t write_adc(struct device *dev, struct device_attribute *attr, const char *buffer, size_t count)
{
	DBG_PRINTF2("ADC: %s\n",__FUNCTION__);
	if(buffer[0] == '0') {
		do_check = 0;
		DBG_PRINTF("do_check = 0\n");
	} else if(buffer[0] == '1') {
		do_check = 1;	
		DBG_PRINTF("do_check = 1\n");
	}
	return count;
}


/* Attach the sysfs read method */
DEVICE_ATTR(adc, 0622, read_adc, write_adc);

/* Attribute Descriptor */
static struct attribute *adc_attrs[] = {
        &dev_attr_adc.attr,
        NULL
};

/* Attribute group */
static struct attribute_group adc_attr_group = {
        .attrs = adc_attrs,
         };

#endif

static int __init w55fa93ts_init(void)
{

    	DBG_PRINTF2("ADC: %s\n",__FUNCTION__);
#ifdef CONFIG_ADC_LIGHT_BATTERY
    	sys_adc = platform_device_register_simple("w55fa93-adc", -1, NULL, 0);
    	if(sys_adc == NULL)
        	printk("register adc detection module failed\n");
    	sysfs_create_group(&sys_adc->dev.kobj, &adc_attr_group);
#endif
	return platform_driver_probe(&w55fa93ts_driver, &w55fa93ts_probe);
}

static void __exit w55fa93ts_exit(void)
{	
	struct clk *clk;
	clk = clk_get(NULL, "adc");
	clk_disable(clk);

	DBG_PRINTF2("ADC: %s\n",__FUNCTION__);
	platform_driver_unregister(&w55fa93ts_driver);
#ifdef CONFIG_ADC_LIGHT_BATTERY
	platform_device_unregister(sys_adc);
#endif
}

module_init(w55fa93ts_init);
module_exit(w55fa93ts_exit);


MODULE_AUTHOR("PT50 zswan <zswan@nuvoton.com>");
MODULE_DESCRIPTION("w55fa93 touch screen driver!");
MODULE_LICENSE("GPL");
