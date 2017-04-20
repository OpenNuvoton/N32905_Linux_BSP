/* linux/drivers/i2c/busses/i2c-w55fa93.c
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
 *   2008/09/09     Vincen.zswan add this file for nuvoton i2c.
 *   2008/09/10     CCwang change this file for delete compile warning.
 */
 
#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

//#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <mach/iic.h>
#include <mach/w55fa93_reg.h>

//#define W55FA93_I2C_DEBUG
#define W55FA93_I2C_DEBUG_ENTER_LEAVE
#define W55FA93_I2C_DEBUG_MSG
#define W55FA93_I2C_DEBUG_MSG2

#ifdef W55FA93_I2C_DEBUG
#define DBG(fmt, arg...)			printk(fmt, ##arg)
#else
#define DBG(fmt, arg...)
#endif


#ifdef W55FA93_I2C_DEBUG_ENTER_LEAVE
#define ENTER()					DBG("[%-10s] : Enter\n", __FUNCTION__)
#define LEAVE()					DBG("[%-10s] : Leave\n", __FUNCTION__)
#else
#define ENTER()
#define LEAVE()
#endif
#ifdef CONFIG_SND_SOC_W55FA93_ADC		

#endif

/* i2c controller state */

enum w55fa93_i2c_state {
	STATE_IDLE,
	STATE_START,
	STATE_READ,
	STATE_WRITE,
	STATE_STOP
};

struct w55fa93_i2c {
	spinlock_t		lock;
	wait_queue_head_t	wait;

	struct i2c_msg		*msg;
	unsigned int		msg_num;
	unsigned int		msg_idx;
	unsigned int		msg_ptr;

	enum w55fa93_i2c_state	state;

	void __iomem		*regs;
	struct clk			*clk;
	struct device		*dev;
	struct resource		*irq;
	struct resource		*ioarea;
	struct i2c_adapter	adap;
};

extern unsigned int w55fa93_apb_clock;
static struct clk *clk;
/* default platform data to use if not supplied in the platform_device
*/

static struct w55fa93_platform_i2c w55fa93_i2c_default_platform = {
	.flags		= 0,
	.slave_addr	= 0x10,
//	.bus_freq	= 200,	//200 kHz
	.bus_freq	= 100,	//200 kHz
	.max_freq	= 400, //400 kHz
	.channel	= 0,
};

/* w55fa93_i2c_get_platformdata
 *
 * get the platform data associated with the given device, or return
 * the default if there is none
*/

static inline struct w55fa93_platform_i2c *w55fa93_i2c_get_platformdata(struct device *dev)
{
	if (dev->platform_data != NULL)
		return (struct w55fa93_platform_i2c *)dev->platform_data;
	
	return &w55fa93_i2c_default_platform;
}

/* w55fa93_i2c_master_complete
 *
 * complete the message and wake up the caller, using the given return code,
 * or zero to mean ok.
*/

static inline void w55fa93_i2c_master_complete(struct w55fa93_i2c *i2c, int ret)
{
	struct w55fa93_platform_i2c *pdata;
	unsigned int reg;

	dev_dbg(i2c->dev, "master_complete %d\n", ret);

	i2c->msg_ptr = 0;
	i2c->msg = NULL;
	i2c->msg_idx ++;
	i2c->msg_num = 0;
	if (ret)
		i2c->msg_idx = ret;
		
	if((readl(REG_I2C_SWR) & 0x18) == 0x18	&& 	//SDR and SCR keep high 
		(readl(REG_I2C_CSR) & 0x0400) != 0	){  	//I2C_BUSY is false

	printk("!!!I2CRST\n");	
	writel(readl(REG_APBIPRST) | I2CRST, REG_APBIPRST);	//reset i2c
	writel(readl(REG_APBIPRST) & ~I2CRST, REG_APBIPRST);	
		
	/* get the plafrom data */	
	pdata = w55fa93_i2c_get_platformdata(i2c->adap.dev.parent);
				
	/* set the i2c speed */
	reg = w55fa93_apb_clock/(pdata->bus_freq * 5) - 1;
	writel( reg & 0xffff, REG_I2C_DIVIDER);

	//writel(readl(REG_I2C_CSR) | SGMST_EN, REG_I2C_CSR);
		
	}

	wake_up(&i2c->wait);
}

/* irq enable/disable functions */

static inline void w55fa93_i2c_disable_irq(struct w55fa93_i2c *i2c)
{
	writel(readl(REG_I2C_CSR) & ~(CSR_IE |I2C_EN), REG_I2C_CSR);
}

static inline void w55fa93_i2c_enable_irq(struct w55fa93_i2c *i2c)
{	
	writel(readl(REG_I2C_CSR) | CSR_IE |I2C_EN, REG_I2C_CSR);
}


/* w55fa93_i2c_message_start
 *
 * put the start of a message onto the bus 
*/

static void w55fa93_i2c_message_start(struct w55fa93_i2c *i2c, 
				      struct i2c_msg *msg)
{
	unsigned int addr = (msg->addr & 0x7f) << 1;	//slave addr
			
	//printk("w55fa93_i2c_message_start, addr = 0x%x, flag = 0x%x\n", addr, msg->flags);		
			
	if (msg->flags & I2C_M_RD)	
		addr |= 1;
	
	writel(addr & 0xff, REG_I2C_TXR);	/* send first byte */
	writel(I2C_CMD_START | I2C_CMD_WRITE, REG_I2C_CMDR);
}

static inline void w55fa93_i2c_stop(struct w55fa93_i2c *i2c, int ret)
{		
	/* stop the transfer */
	i2c->state = STATE_STOP;
		
	w55fa93_i2c_master_complete(i2c, ret);
	w55fa93_i2c_disable_irq(i2c);
}

/* i2s_s3c_irq_nextbyte
 *
 * process an interrupt and work out what to do
 */

static int i2c_irq_nextbyte(struct w55fa93_i2c *i2c, unsigned long iiccsr)
{
	unsigned char byte;
	int ret = 0;
					
	if(iiccsr & 0x100)	//transfer in pregress ?
		goto out;
	
	switch (i2c->state) 
	{

	case STATE_START:
		if (i2c->msg->flags & I2C_M_RD)
			i2c->state = STATE_READ;
		else
			i2c->state = STATE_WRITE;

		if (i2c->state == STATE_READ)
			goto prepare_read;


	case STATE_WRITE:
retry_write:
		if (i2c->msg_ptr < i2c->msg->len) 
		{
			byte = i2c->msg->buf[i2c->msg_ptr++];
			writel(byte, REG_I2C_TXR);			
			if (i2c->msg_ptr == i2c->msg->len &&  i2c->msg_idx == i2c->msg_num - 1)		//end data ?
				writel( I2C_CMD_WRITE | I2C_CMD_STOP, REG_I2C_CMDR);
			else								
				writel( I2C_CMD_WRITE, REG_I2C_CMDR);			
		}
		else if (i2c->msg_idx < i2c->msg_num - 1) 	//restart a new transmittion
		{			
			i2c->msg_ptr = 0;
			i2c->msg_idx ++;
			i2c->msg++;
			
			/* check to see if we need to do another message */
			if (i2c->msg->flags & I2C_M_NOSTART) {

				if (i2c->msg->flags & I2C_M_RD) {
					/* cannot do this, the controller
					 * forces us to send a new START
					 * when we change direction */

					w55fa93_i2c_stop(i2c, -EINVAL);
				}

				goto retry_write;
			} else {			
				/* send the new start */
				w55fa93_i2c_message_start(i2c, i2c->msg);
				i2c->state = STATE_START;
			}

		}
		else if(i2c->msg->len == 0)
		{	
			writel( I2C_CMD_STOP, REG_I2C_CMDR);		
			w55fa93_i2c_stop(i2c, 0);
		}
		else
			w55fa93_i2c_stop(i2c, 0);
					
		break;

	case STATE_READ:
		byte = readl(REG_I2C_RXR) & 0xff;		//skip first read
		i2c->msg->buf[i2c->msg_ptr-1] = byte;
				
prepare_read:
		i2c->msg_ptr ++;
		if (i2c->msg_ptr < i2c->msg->len + 1) 
		{			
			if(i2c->msg_ptr == i2c->msg->len)	//reach the end data
				writel( I2C_CMD_READ | I2C_CMD_STOP | I2C_CMD_NACK, REG_I2C_CMDR);
			else
				writel( I2C_CMD_READ, REG_I2C_CMDR);
		}
		else
			w55fa93_i2c_stop(i2c, 0);
		
		break;
		
		case STATE_IDLE:
		case STATE_STOP:
			break;
		
	}
	
 out:
	return ret;
}

/* w55fa93_i2c_irq
 *
 * top level IRQ servicing routine
*/

static irqreturn_t w55fa93_i2c_irq(int irqno, void *dev_id)			   
{
	struct w55fa93_i2c *i2c = dev_id;
	unsigned long status;
				
	if(!(readl(REG_AIC_IASR) & 0x40000000))	//check irq for i2c
		goto out;
		
	status = readl(REG_I2C_CSR);	
		
	/* mark interrupt flag */
	writel(  status | 0x04, REG_I2C_CSR);
	
	if (status & 0x200) // deal with arbitration loss
	{		
		dev_err(i2c->dev, "deal with arbitration loss\n");		
		goto out;
	}
	
	if((status & 0x800) && (i2c->state == STATE_WRITE || i2c->state == STATE_START)) // deal with NACK
	{		
		dev_err(i2c->dev, "deal with nack\n");
		writel( I2C_CMD_STOP, REG_I2C_CMDR);
		i2c->state = STATE_STOP;
		w55fa93_i2c_disable_irq(i2c);
		goto out;
	}
	
	if (i2c->state == STATE_IDLE) {
		dev_dbg(i2c->dev, "IRQ: error i2c->state == IDLE\n");		
		goto out;
	}
	
	/* pretty much this leaves us with the fact that we've
	 * transmitted or received whatever byte we last sent */

	i2c_irq_nextbyte(i2c, status);

 out:
	return IRQ_HANDLED;
}


/* w55fa93_i2c_set_master
 *
 * get the i2c bus for a master transaction
*/

static int w55fa93_i2c_set_master(struct w55fa93_i2c *i2c)
{	
	int timeout = 400;
	struct w55fa93_platform_i2c *pdata;
	
	pdata = w55fa93_i2c_get_platformdata(i2c->adap.dev.parent);
	while (timeout-- > 0) {		
		if(	(readl(REG_I2C_SWR) & 0x18) == 0x18	&& 	//SDR and SCR keep high 
			(readl(REG_I2C_CSR) & 0x0400) == 0	){  	//I2C_BUSY is false
			return 0;
		}

		msleep(1);
	}

	dev_dbg(i2c->dev, "timeout\n");

	return -ETIMEDOUT;
}

/* w55fa93_i2c_doxfer
 *
 * this starts an i2c transfer
*/

static int w55fa93_i2c_doxfer(struct w55fa93_i2c *i2c, struct i2c_msg *msgs, int num)
{
	unsigned long timeout;
	int ret;
		
	/* enable i2c clock */
	clk_enable(clk);
	writel(readl(REG_I2C_CSR) | I2C_EN, REG_I2C_CSR);		
	
	ret = w55fa93_i2c_set_master(i2c);	//check bus
	if (ret != 0) {
		dev_err(i2c->dev, "cannot get bus (error %d)\n", ret);
		ret = -EAGAIN;
		goto out;
	}
	
	spin_lock_irq(&i2c->lock);

	i2c->msg     = msgs;
	i2c->msg_num = num;
	i2c->msg_ptr = 0;
	i2c->msg_idx = 0;
	i2c->state   = STATE_START;
		
	w55fa93_i2c_enable_irq(i2c);
	w55fa93_i2c_message_start(i2c, msgs);
	spin_unlock_irq(&i2c->lock);
	
	timeout = wait_event_timeout(i2c->wait, i2c->msg_num == 0, HZ * 2);

	ret = i2c->msg_idx;
		
	/* having these next two as dev_err() makes life very 
	 * noisy when doing an i2cdetect */

	/* disable i2c clock */
	writel(readl(REG_I2C_CSR) & ~I2C_EN, REG_I2C_CSR);
	clk_disable(clk);

	if (timeout == 0)
	{
		dev_dbg(i2c->dev, "timeout\n");
		ret = -EIO;
	}
	else if (ret != num)
	{	
		dev_dbg(i2c->dev, "incomplete xfer (%d)\n", ret);
		ret = -EIO;
	}

	/* ensure the stop has been through the bus */

	//msleep(1);

 out:
	return ret;
}

/* w55fa93_i2c_xfer
 *
 * first port of call from the i2c bus code when an message needs
 * transferring across the i2c bus.
*/

static int w55fa93_i2c_xfer(struct i2c_adapter *adap,
			struct i2c_msg *msgs, int num)
{
	struct w55fa93_i2c *i2c = (struct w55fa93_i2c *)adap->algo_data;
	int retry;
	int ret;
	 
	int ii;
	
	DBG("msgs->addr = 0x%x !!!\n", msgs->addr);
	DBG("msgs->len = 0x%x !!!\n", msgs->len);	
	DBG("msgs->buf ...\n");		
	for (ii=0; ii<msgs->len; ii++)
		DBG("%x. 0x%x !!!\n", ii, msgs->buf[ii]);			
		
#ifdef CONFIG_SND_SOC_W55FA93_SPU		
	if (msgs->addr == 0x7F)				// 55fa93_dac i2c_ID = 0x7F (given value
	{
		DBG("REG_SPU_DAC_VOL = 0x%x  !! (before setting)\n", readl(REG_SPU_DAC_VOL));		
		if (msgs->buf[0] == 0x04)		// left volume control
		{
			writel((readl(REG_SPU_DAC_VOL)& ~0x3F00) | (msgs->buf[1]<<8), REG_SPU_DAC_VOL);	
		}
		else if (msgs->buf[0] == 0x06)	// right volume control
		{
			writel((readl(REG_SPU_DAC_VOL)& ~0x3F) | (msgs->buf[1]), REG_SPU_DAC_VOL);	
		}			
		
		DBG("REG_SPU_DAC_VOL = 0x%x  !! \n", readl(REG_SPU_DAC_VOL));
		return 0;
	}		
#endif
#ifdef CONFIG_SND_SOC_W55FA93_ADC	
	//msgs->addr : 55fa93adc i2c_ID = 0x6F (given value)
	//msgs->buf[0] = 0x22 : // left volume control
	//msgs->buf[0] = 0x23 : // right volume control	
	//msgs->buf[1] = Volume	
	if (msgs->addr == 0x6F)				// 55fa93adc i2c_ID = 0x6F (given value
	{
		if((readl(REG_APBCLK)&ADC_CKE)!= ADC_CKE) {
			printk("ADC driver is not opened\n");
			return 0;
		}
		if(msgs->buf[0]==0x0){
			writel((readl(REG_AGCP1) & ~OTL) | (((msgs->buf[1]&0x0f)<<12) & OTL), REG_AGCP1);
			//printk("OTL = 0x%x\n", readl(REG_AGCP1));
			if((msgs->buf[1] & 0xf)==0x0){			
				writel((readl(REG_AGC_CON) & ~(NG_EN |NG_LEVEL)) |\
     				(((1 <<31)& NG_EN) |\
     				((0 <<12)& NG_LEVEL)), REG_AGC_CON );
					writel(readl(REG_OPOC) | MUTE_SW, REG_OPOC);
					//printk("Driver volume =0,  msgs->buf[1] = %x\n", msgs->buf[1]);
				
			}else{
				writel((readl(REG_AGC_CON) & ~(NG_EN |NG_LEVEL)) |\
     				(((0 <<31)& NG_EN) |\
     				((3 <<12)& NG_LEVEL)), REG_AGC_CON);
				writel(readl(REG_OPOC) & ~MUTE_SW, REG_OPOC);
				//printk("Driver volume !=0,  msgs->buf[1] = %x\n", msgs->buf[1]);
			}
			//printk("REG_AGC_CON = %x\n", readl(REG_AGC_CON));
			//printk("REG_OPOC = %x\n", readl(REG_OPOC));
		}
		
		return 0;
	}		
#endif 

		
	for (retry = 0; retry < adap->retries; retry++) {

		ret = w55fa93_i2c_doxfer(i2c, msgs, num);
	
		if (ret != -EAGAIN)
			return ret;

		dev_dbg(i2c->dev, "Retrying transmission (%d)\n", retry);

		udelay(100);
	}

	return -EREMOTEIO;
}

/* declare our i2c functionality */
static u32 w55fa93_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_PROTOCOL_MANGLING;
}

/* i2c bus registration info */

static struct i2c_algorithm w55fa93_i2c_algorithm = {
	.master_xfer		= w55fa93_i2c_xfer,
	.functionality		= w55fa93_i2c_func,
};

static struct w55fa93_i2c w55fa93_i2c = {
	.lock	= SPIN_LOCK_UNLOCKED,
	.wait	= __WAIT_QUEUE_HEAD_INITIALIZER(w55fa93_i2c.wait),
	.adap	= {
		.name			= "w55fa93-i2c",
		.owner			= THIS_MODULE,
		.algo			= &w55fa93_i2c_algorithm,
		.retries		= 2,
		.class			= I2C_CLASS_HWMON,
	},
};

/* w55fa93_i2c_init
 *
 * initialise the controller, set the IO lines and frequency 
*/

static int w55fa93_i2c_init(struct w55fa93_i2c *i2c)
{
	struct w55fa93_platform_i2c *pdata;
	unsigned int reg;
	
	/* setup I2C share pin for I2C */
	writel(readl(REG_GPBFUN) | (MF_GPB13 | MF_GPB14), REG_GPBFUN);	//gpiob(13,14)	

	writel(readl(REG_APBIPRST) | I2CRST, REG_APBIPRST);	//reset i2c
	writel(readl(REG_APBIPRST) & ~I2CRST, REG_APBIPRST);	
			
	/* get the plafrom data */	
	pdata = w55fa93_i2c_get_platformdata(i2c->adap.dev.parent);
					
	/* set the i2c speed */
	reg = w55fa93_apb_clock/(pdata->bus_freq * 5) - 1;
	writel( reg & 0xffff, REG_I2C_DIVIDER);	
		
	dev_info(i2c->dev, "bus frequency set to %ld KHz\n", pdata->bus_freq);

	//outl(inl(REG_I2C_CSR) | SGMST_EN, REG_I2C_CSR);
				
	return 0;
}

static void w55fa93_i2c_free(struct w55fa93_i2c *i2c)
{
	free_irq(IRQ_I2C, i2c);
}

/* w55fa93_i2c_probe
 *
 * called by the bus driver when a suitable device is found
*/

static int w55fa93_i2c_probe(struct platform_device *pdev)
{
	struct w55fa93_i2c *i2c = &w55fa93_i2c;
	int ret;	
	
	printk("w55fa93_i2c_probe()\n");	
	
	/* setup info block for the i2c core */
	i2c->dev = &pdev->dev;	
	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &pdev->dev;
	i2c->adap.retries = 2;
       i2c->adap.class   = I2C_CLASS_HWMON | I2C_CLASS_SPD;

	   /* enable i2c clock */
        clk = clk_get(&pdev->dev, NULL);
        clk_enable(clk);
		
	/* initialise the i2c controller */
	ret = w55fa93_i2c_init(i2c);
	if (ret != 0)
		goto out;
		
	ret = request_irq(IRQ_I2C, w55fa93_i2c_irq, IRQF_DISABLED,  pdev->name, i2c);
	if (ret != 0) {
		dev_err(&pdev->dev, "cannot claim IRQ\n");
		goto out;
	}
						
//	ret = i2c_add_adapter(&i2c->adap);
	i2c->adap.nr = 0;
	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0) {		
		dev_err(&pdev->dev, "failed to add bus to i2c core\n");
		goto out;
	}
	
	platform_set_drvdata(pdev, i2c);	
	
	dev_info(&pdev->dev, "Add W55FA93 I2C port adapter\n");
	
 out:
	if (ret < 0)
		w55fa93_i2c_free(i2c);

	/* disable i2c clock */
	writel(readl(REG_I2C_CSR) & ~I2C_EN, REG_I2C_CSR);
	clk_disable(clk);

	return ret;
}

/* w55fa93_i2c_remove
 *
 * called when device is removed from the bus
*/

static int w55fa93_i2c_remove(struct platform_device *pdev)
{
	struct w55fa93_i2c *i2c = platform_get_drvdata(pdev);
	
	if (i2c != NULL) {
		w55fa93_i2c_free(i2c);
		platform_set_drvdata(pdev, NULL);
	}	

	return 0;
}

#ifdef CONFIG_PM
static int w55fa93_i2c_resume(struct platform_device *dev)
{
	struct w55fa93_i2c *i2c = platform_get_drvdata(dev);

	if (i2c != NULL)
		w55fa93_i2c_init(i2c);

	return 0;
}

#else
#define w55fa93_i2c_resume NULL
#endif

/* device driver for platform bus bits */

static struct platform_driver w55fa93_i2c_driver = {
	.probe		= w55fa93_i2c_probe,
	.remove		= w55fa93_i2c_remove,
	.resume		= w55fa93_i2c_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "w55fa93-i2c",
	},
};

static int __init i2c_adap_w55fa93_init(void)
{
	int ret;

	printk("i2c_adap_w55fa93_init\n");
	ret = platform_driver_register(&w55fa93_i2c_driver);

	return ret;
}

static void __exit i2c_adap_w55fa93_exit(void)
{
	platform_driver_unregister(&w55fa93_i2c_driver);	
}

module_init(i2c_adap_w55fa93_init);
module_exit(i2c_adap_w55fa93_exit);

MODULE_DESCRIPTION("W55FA93 I2C Bus driver");
MODULE_LICENSE("GPL");
