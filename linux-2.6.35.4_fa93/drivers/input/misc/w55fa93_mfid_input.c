/* linux/driver/input/w55fa93_mfid_input.c
 *
 * Copyright (c) 2011 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Changelog:
 *
 */

#include <linux/init.h>
#include <linux/slab.h>
#include <asm/errno.h>
#include <asm/delay.h>
#include <linux/mm.h>
#include <linux/poll.h>
#include <linux/module.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <linux/cdev.h>
#include <asm/arch/irqs.h>
#include <linux/interrupt.h>
#include  <linux/completion.h>
#include <asm/arch/w55fa93_reg.h>
#include <asm/arch/regs-clock.h>
#undef BIT
#include <linux/input.h>

#include <asm/arch/w55fa93_mfid.h>

#define MFID_IRQ_NUM             W55FA93_IRQ(5)  // nIRQ0
#define PIN_TAGIN		 	 	 14
#define PIN_SEL		  	 	 	 3
#define PIN_CMD		  	 	 	 12
#define PIN_CLK		  	 	 	 13
#define PIN_DATA		  	 	 15

#define DEF_DELAY           	HZ/100 // HZ/20

static struct mutex mfid_mutex;


#define W55FA93_DEBUG //printk("##==##"); printk

static struct input_dev *w55fa93_mfid_input_dev;
static struct timer_list mfid_timer;
static char timer_active = 0;

static u32 old_key;
static u32 new_key;
static u32 open_cnt = 0;



#define MID50_WakeUp       	1
#define MID50_ReloadFIFO    2
#define MID50_ReadCR       	3
#define MID50_WriteCR       4
#define MID50_PowerDown     5
#define gpioD_get(num) (inl(REG_GPIOD_PIN) & (1 << num) ? 1:0)

#define MFID_CLK_DELAY		2  // us

#define	MFID_ITEM_CNT		5
static mfidItemTag mfidItem[MFID_ITEM_CNT];


VOID W55MID50_Command(UINT8 u8Cmd)
{
	UINT8 i;
		
	outl(inl(REG_GPIOD_DOUT) & ~(1 << PIN_CMD),REG_GPIOD_DOUT); 	//set low
	outl(inl(REG_GPIOD_DOUT) & ~(1 << PIN_CLK),REG_GPIOD_DOUT); 	//set low
	udelay(MFID_CLK_DELAY);

	outl(inl(REG_GPIOD_DOUT) | (1 << PIN_CMD),REG_GPIOD_DOUT); 	//set high
	udelay(MFID_CLK_DELAY);

	for(i=0; i<u8Cmd; i++)
	{
		outl(inl(REG_GPIOD_DOUT) | (1 << PIN_CLK),REG_GPIOD_DOUT); 	//set high
		udelay(MFID_CLK_DELAY);
		outl(inl(REG_GPIOD_DOUT) & ~(1 << PIN_CLK),REG_GPIOD_DOUT); 	//set low
		udelay(MFID_CLK_DELAY);
	}

	udelay(MFID_CLK_DELAY);
	outl(inl(REG_GPIOD_DOUT) & ~(1 << PIN_CMD),REG_GPIOD_DOUT); 	//set low
	udelay(MFID_CLK_DELAY);
	
}

VOID W55MID50_WriteCR(UINT8 u8Data)
{
	UINT8 i;
	
	W55FA93_DEBUG("Enter %s\n", __FUNCTION__);
	
	W55MID50_Command(MID50_WriteCR);	

	for (i=0;i<8;++i)
	{	
		if (u8Data & 0x80)	
			outl(inl(REG_GPIOD_DOUT) | (1 << PIN_CMD),REG_GPIOD_DOUT); 	//set high
		else
			outl(inl(REG_GPIOD_DOUT) & ~(1 << PIN_CMD),REG_GPIOD_DOUT); 	//set low	

		outl(inl(REG_GPIOD_DOUT) | (1 << PIN_CLK),REG_GPIOD_DOUT); 	//set high
		udelay(MFID_CLK_DELAY);
		outl(inl(REG_GPIOD_DOUT) & ~(1 << PIN_CLK),REG_GPIOD_DOUT); 	//set low

		u8Data<<=1;	
	}	
	
}

INT W55MID50_ReadCR(VOID)
{
	UINT8 i;	
	INT  i32Data, i32Reg;
	
	W55FA93_DEBUG("Enter %s\n", __FUNCTION__);
	
	i32Data =0;		
	
	W55MID50_Command(MID50_ReadCR);	

	for (i=0;i<8;++i)
	{
		outl(inl(REG_GPIOD_DOUT) | (1 << PIN_CLK),REG_GPIOD_DOUT); 	//set high
		udelay(MFID_CLK_DELAY);
		
		i32Reg = gpioD_get(PIN_DATA);	
		
		outl(inl(REG_GPIOD_DOUT) & ~(1 << PIN_CLK),REG_GPIOD_DOUT); 	//set low		
		udelay(MFID_CLK_DELAY);

		i32Data = i32Data << 1;	
		if(i32Reg)
			i32Data |= 0x01;		
	}	

	return i32Data;
	
}

INT W55MID50_ReadID(VOID)
{
	UINT8 i;	
	INT  i32Data, i32Reg;

	i32Data =0;		

	W55FA93_DEBUG("Enter %s\n", __FUNCTION__);
	
	for (i=0;i<16;++i)
	{
		outl(inl(REG_GPIOD_DOUT) | (1 << PIN_CLK),REG_GPIOD_DOUT); 	//set high	
		udelay(MFID_CLK_DELAY);
		
		i32Reg = gpioD_get(PIN_DATA);
		
		outl(inl(REG_GPIOD_DOUT) & ~(1 << PIN_CLK),REG_GPIOD_DOUT); 	//set low					
		udelay(MFID_CLK_DELAY);

		i32Data = i32Data << 1;	
		if(i32Reg)
			i32Data |= 0x01;		
	}
	
//	udelay(MFID_CLK_DELAY);
	
	return i32Data;

}


static int __chkIndexInList(int IdCode)
{
	int i;
	int ret=-1;
	for(i=0; i< MFID_ITEM_CNT; i++)
    {
    	if( mfidItem[i].IdCode	== IdCode )
    	{	
    		mfidItem[i].ChkCnt = 1;
    		ret = i;
    	}
    }
    return ret;
}	

static int __addIntoList(int IdCode)
{
	int i;
	int ret=-1;
	for(i=0; i< MFID_ITEM_CNT; i++)
    {
    	if( mfidItem[i].Isdown	== 0 )
    	{
    		mfidItem[i].IdCode = IdCode;
    		mfidItem[i].Isdown = 1;
    		mfidItem[i].ChkCnt = 1;
    		ret = i;
    		return ret;
    	}	
    }	
    return ret;
}	

static void read_key(unsigned long arg)
{
        u32 i;
#if 0
        // ISR detect key press, disable irq, use timer to read following key press until released
        if (!timer_active) {
                outl(1 << MFID_IRQ_NUM, REG_AIC_MDCR);
                disable_irq(MFID_IRQ_NUM);
        }
#endif
		int i32Data = W55MID50_ReadID();
		i32Data = i32Data & 0x3FF;	
		W55FA93_DEBUG("=== MFID code[%d]\n",i32Data);	
		if( i32Data != 0x3FF )
		{
			if( __chkIndexInList(i32Data) < 0)
			{
				__addIntoList(i32Data);
        		input_report_key(w55fa93_mfid_input_dev, i32Data, 1);	//down event
           		input_sync(w55fa93_mfid_input_dev);			
            	W55FA93_DEBUG("=== Down MFID code[%d]\n",i32Data);
        	}
           
		} else {				

        // purge up event
        	for(i=0; i< MFID_ITEM_CNT; i++)
        	{
//        		W55FA93_DEBUG("=== ChkCnt=[%d], i32Data=0x%x, IdCode=%d, Isdown=%d\n",
//        			mfidItem[i].ChkCnt, i32Data, mfidItem[i].IdCode, mfidItem[i].Isdown);
        		if( (mfidItem[i].ChkCnt == 0 ) && 
        			(mfidItem[i].IdCode > 0 ) && (mfidItem[i].Isdown ==1) )
        		{
        			input_report_key(w55fa93_mfid_input_dev, mfidItem[i].IdCode, 0);	//key up
            		W55FA93_DEBUG("=== Up MFID code[%d]\n",mfidItem[i].IdCode);
            		memset(&mfidItem[i],0x00, sizeof(mfidItemTag));
            		input_sync(w55fa93_mfid_input_dev);			
        		}
        		mfidItem[i].ChkCnt = 0;
        			   
        	}
        }		     		

#if 0
        if (arg !=0 && new_key == 0) {
                del_timer(&mfid_timer);
                timer_active = 0;
                enable_irq(MFID_IRQ_NUM);
        } else {
                timer_active = 1;
                mod_timer(&mfid_timer, jiffies + DEF_DELAY);
        }
#else
	#if 0
		mutex_lock(&mfid_mutex);
		if( i32Data == 0x3FF )
		{	
        	del_timer(&mfid_timer);
        	timer_active = 0;
        }	
        mutex_unlock(&mfid_mutex);
   #endif     
#endif
        return;

}


static irqreturn_t w55fa93_mfid_irq(int irq, void *dev_id, struct pt_regs *regs) {

        u32 src;
		W55FA93_DEBUG("Enter %s\n", __FUNCTION__);
		
        src = inl(REG_IRQTGSRC1);
        // clear source
		outl(src | ((1 << PIN_TAGIN)<<16), REG_IRQTGSRC1); // clear source
	
#if 0
		read_key(0);
#else
		mutex_lock(&mfid_mutex);
        timer_active = 1;
        mod_timer(&mfid_timer, jiffies + DEF_DELAY);	
        mutex_unlock(&mfid_mutex);			
#endif		


        return IRQ_HANDLED;
}


int w55fa93_mfid_open(struct input_dev *dev) {
		int i;
		
		W55FA93_DEBUG("Enter %s\n", __FUNCTION__);
        if (open_cnt > 0) {
                goto exit;
        }

		for(i=0; i< MFID_ITEM_CNT; i++)
			memset(&mfidItem[i],0x00, sizeof(mfidItemTag));

        init_timer(&mfid_timer);
        mfid_timer.function = read_key;	/* timer handler */
        mfid_timer.data = 1;
        outl((1 << MFID_IRQ_NUM),  REG_AIC_SCCR); // force clear previous interrupt, if any.
		outl(inl(REG_IRQTGSRC1) | ((1 << PIN_TAGIN)<<16), REG_IRQTGSRC1); // clear source
		W55FA93_DEBUG(" request_irq /n");
        if (request_irq(MFID_IRQ_NUM, w55fa93_mfid_irq, SA_INTERRUPT, "MFID",NULL) != 0) {
                printk("register the mfid_irq failed!\n");
                return -1;
        }

        int falling=1;
        int rising=0;
		outl(inl(REG_IRQENGPD) & ~(((1<<PIN_TAGIN) << 16) | (1<<PIN_TAGIN)), REG_IRQENGPD);	// Set int mode 
		outl(inl(REG_IRQENGPD) | (((rising <<PIN_TAGIN) << 16) | (falling <<PIN_TAGIN)), REG_IRQENGPD);  //// falling/rising edge trigger
		
		W55MID50_WriteCR(0x93);	//35 mode	

		printk("MFID CR %x \n", W55MID50_ReadCR());

exit:
        open_cnt++;
        return 0;
}



void w55fa93_mfid_close(struct input_dev *dev) {
        open_cnt--;
        if (open_cnt == 0) {
			//disable falling edge triggers
			outl(inl(REG_IRQENGPD) & ~(((1<<PIN_TAGIN) << 16) | (1<<PIN_TAGIN)), REG_IRQENGPD);
            del_timer(&mfid_timer);
            free_irq(MFID_IRQ_NUM,NULL);
        }
        return;
}


static int __init w55fa93_mfid_reg(void) {

        int i, err;

        // init GPIO
        int falling=1;
        int rising=0;



        W55FA93_DEBUG("Start w55fa93_mfid_reg\n");
        
        mutex_init(&mfid_mutex);
        
        //set share pin funcion
        outl(inl(REG_GPDFUN) &~ (0x3 << (PIN_TAGIN<<1)),REG_GPDFUN);
        outl(inl(REG_GPDFUN) &~ (0x3 << (PIN_DATA<<1)), REG_GPDFUN);
        outl(inl(REG_GPDFUN) &~ (0x3 << (PIN_CMD<<1)), REG_GPDFUN);
        outl(inl(REG_GPDFUN) &~ (0x3 << (PIN_CLK<<1)), REG_GPDFUN);

		//set output mode
		outl(inl(REG_GPIOD_OMD) | (1 << PIN_CMD), REG_GPIOD_OMD); 
		outl(inl(REG_GPIOD_OMD) | (1 << PIN_CLK), REG_GPIOD_OMD); 		
		//set input mode
		outl(inl(REG_GPIOD_OMD) & ~(1 << PIN_TAGIN), REG_GPIOD_OMD); 
		outl(inl(REG_GPIOD_OMD) & ~(1 << PIN_DATA), REG_GPIOD_OMD); 				

		//set pull up
        outl(inl(REG_GPIOD_PUEN) | (1 << PIN_TAGIN),REG_GPIOD_PUEN); 
        outl(inl(REG_GPIOD_PUEN) | (1 << PIN_DATA),REG_GPIOD_PUEN); 

        //set IRQ
		outl((inl(REG_IRQSRCGPD) &~ (0x3 << (PIN_TAGIN<<1))) | (PIN_SEL << (PIN_TAGIN<<1)), REG_IRQSRCGPD); //nIRQ0 source
		outl(inl(REG_IRQENGPD) & ~(((1<<PIN_TAGIN) << 16) | (1<<PIN_TAGIN)), REG_IRQENGPD);					// Set int mode
		outl(inl(REG_IRQENGPD) | (((rising <<PIN_TAGIN) << 16) | (falling <<PIN_TAGIN)), REG_IRQENGPD);  //// falling/rising edge trigger
		
        if (!(w55fa93_mfid_input_dev = input_allocate_device())) {
                printk("W55FA93 MFID Drvier Allocate Memory Failed!\n");
                err = -ENOMEM;
                goto fail;
        }

        w55fa93_mfid_input_dev->name = "W55FA93 MFID";
        w55fa93_mfid_input_dev->phys = "input/event2";
        w55fa93_mfid_input_dev->id.bustype = BUS_HOST;
        w55fa93_mfid_input_dev->id.vendor  = 0x0005;
        w55fa93_mfid_input_dev->id.product = 0x0001;
        w55fa93_mfid_input_dev->id.version = 0x0100;

        w55fa93_mfid_input_dev->open    = w55fa93_mfid_open;
        w55fa93_mfid_input_dev->close   = w55fa93_mfid_close;

        w55fa93_mfid_input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_SYN); // |  BIT(EV_REP);

        for (i = 0; i < 240; i++)
                set_bit(i+1, w55fa93_mfid_input_dev->keybit);

		W55FA93_DEBUG("Start input_register_device\n");
        err = input_register_device(w55fa93_mfid_input_dev);
        if (err) {

                input_free_device(w55fa93_mfid_input_dev);
                return err;
        }

	// must set after input device register!!!
//        w55fa93_mfid_input_dev->rep[REP_DELAY] = 200; //250ms
//        w55fa93_mfid_input_dev->rep[REP_PERIOD] = 100; //ms

        printk("W55FA93 MFID driver has been initialized successfully!\n");

        return 0;

fail:
        input_free_device(w55fa93_mfid_input_dev);
        return err;
}

static void __exit w55fa93_mfid_exit(void) {
        free_irq(MFID_IRQ_NUM, NULL);
        input_unregister_device(w55fa93_mfid_input_dev);
}

module_init(w55fa93_mfid_reg);
module_exit(w55fa93_mfid_exit);

MODULE_DESCRIPTION("W55FA93 mfid driver");
MODULE_LICENSE("GPL");
