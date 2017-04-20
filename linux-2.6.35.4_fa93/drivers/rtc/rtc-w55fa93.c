/* linux/driver/rtc/rtc-w55fa93.c
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
 *   2008/09/02     vincen.zswan add this file for nuvoton RTC.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/clk.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <asm/bitops.h>
//#include <asm/hardware.h>

#include <asm/irq.h>
//#include <asm/rtc.h>
#include <linux/rtc.h>
#include <asm/io.h>
#include <mach/w55fa93_reg.h>
#include <mach/w55fa93_rtc.h>


#define TIMER_FREQ		CLOCK_TICK_RATE
#define RTC_DEF_DIVIDER		32768 - 1
#define RTC_DEF_TRIM		0
static const unsigned char days_in_mo[] = 
{0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

static spinlock_t w55fa93_rtc_lock = SPIN_LOCK_UNLOCKED;
struct clk  *rtc_clk;
static struct timer_list rtc_int_timer;
#define RTC_INTERVAL_TIME  HZ	

static irqreturn_t w55fa93_rtc_interrupt(int irq, void *dev_id,
		struct pt_regs *regs)
{
	struct platform_device *pdev = to_platform_device(dev_id);
	struct rtc_device *rtc = platform_get_drvdata(pdev);
	unsigned long events = 0;
	static unsigned int rtc_irq = 0;
	unsigned int irq_clear=0;
	
	clk_enable(rtc_clk);	
	spin_lock(&w55fa93_rtc_lock);

	rtc_irq = readl(REG_RTC_RIIR);
	irq_clear=rtc_irq;
	
	if(rtc_irq&ALARMINTENB){
		irq_clear = ALARMINTENB;
		writel(irq_clear ,REG_RTC_RIIR);
		events |= RTC_AF | RTC_IRQF;
	}
	
	if(rtc_irq&TICKINTENB){
		irq_clear = TICKINTENB;
		writel(irq_clear ,REG_RTC_RIIR);
		events |= RTC_PF | RTC_IRQF;
	}

	if(rtc_irq&PSWINTENB){
		irq_clear = PSWINTENB;
		writel(irq_clear ,REG_RTC_RIIR);
	}
	//rtc_update_irq(&rtc->class_dev, 1, events);
	rtc_update_irq(rtc, 1, events);
	spin_unlock(&w55fa93_rtc_lock);	
	clk_disable(rtc_clk);
	return IRQ_HANDLED;
}


static void check_rtc_power(void)
{
	int i;
	
	writel(INIRRESET, REG_RTC_INIR); /* init value */
	mdelay(10);
	writel(AERPOWERON,REG_RTC_AER);/* power on */

	for (i = 0 ; i < 1000000 ; i++)	/* may take up to 1 second... */
	{
		if (readl(REG_RTC_AER) & AERRWENB)
		{
			break;
		}
	}
	return;
} 

static int w55fa93_rtc_open(struct device *dev)
{
	int ret;
	//ret = request_irq(IRQ_RTC, w55fa93_rtc_interrupt, SA_INTERRUPT, "rtc", dev);
	ret = request_irq(IRQ_RTC, (irq_handler_t) w55fa93_rtc_interrupt, IRQF_DISABLED | IRQF_IRQPOLL, "rtc", dev);

	if (ret) {
		printk("RTC IRQ %d already in use.\n", IRQ_RTC);
        return -1;
	}
	return 0;

}

static void w55fa93_rtc_release(struct device *dev)
{
	free_irq (IRQ_RTC, dev);
}


static int w55fa93_rtc_ioctl(struct device *dev, unsigned int cmd,
		unsigned long arg)
{
	int scale=0;
	int tickount=0;
	int tickcount=0;
	clk_enable(rtc_clk);	
	switch(cmd) {
	case RTC_AIE_OFF:
		spin_lock_irq(&w55fa93_rtc_lock);
		writel(readl(REG_RTC_RIER)&(~ALARMINTENB),REG_RTC_RIER);
		spin_unlock_irq(&w55fa93_rtc_lock);
		clk_disable(rtc_clk);
		return 0;
		
	case RTC_AIE_ON:
		spin_lock_irq(&w55fa93_rtc_lock);
		writel(readl(REG_RTC_RIER)|(ALARMINTENB),REG_RTC_RIER);
		spin_unlock_irq(&w55fa93_rtc_lock);
		clk_disable(rtc_clk);
		return 0;
		
	case RTC_TICK_OFF:
		spin_lock_irq(&w55fa93_rtc_lock);
		writel(readl(REG_RTC_RIER)&(~TICKINTENB),REG_RTC_RIER);
		spin_unlock_irq(&w55fa93_rtc_lock);
		clk_disable(rtc_clk);
		return 0;
		
	case RTC_TICK_ON:
		spin_lock_irq(&w55fa93_rtc_lock);
		writel(readl(REG_RTC_RIER)|(TICKINTENB),REG_RTC_RIER);
		spin_unlock_irq(&w55fa93_rtc_lock);
		clk_disable(rtc_clk);
		return 0;
		
	case RTC_TIME_SCALE:
		
		scale=	*(int*)arg; 
			if(scale==TIME24)
				writel(readl(REG_RTC_TSSR)|(HR24),REG_RTC_TSSR);
			if(scale==TIME12)
				writel(readl(REG_RTC_TSSR)|(HR12),REG_RTC_TSSR);
		clk_disable(rtc_clk);
		return 0;
		
	case RTC_TICK_READ:
		
		spin_lock_irq(&w55fa93_rtc_lock);
		tickount=(readl(REG_RTC_TTR)&0x7F);
		spin_unlock_irq(&w55fa93_rtc_lock);
		*(int *)arg=tickount;
		clk_disable(rtc_clk);
		return 0;
		
	case RTC_TICK_SET:	
		
		tickcount=(*(int*)arg);
		
		if(tickcount>7||tickcount<0){	
			printk("bad tickcount is %d\n",tickcount);	
			clk_disable(rtc_clk);
			return -EINVAL;	
		}
		spin_lock_irq(&w55fa93_rtc_lock);
		check_rtc_power();

		writel(0, REG_RTC_TTR);		
		writel(tickcount, REG_RTC_TTR);		
		spin_unlock_irq(&w55fa93_rtc_lock);
		clk_disable(rtc_clk);
		return 0;

	}
	clk_disable(rtc_clk);
	return -ENOIOCTLCMD;
}

static int w55fa93_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
//	printk("Get time\n");
	//check_rtc_power();
	clk_enable(rtc_clk);	
	tm->tm_sec =  RTC_SECONDS;
	tm->tm_min =  RTC_MINUTES;
	tm->tm_hour = RTC_HOURS;
	tm->tm_mday = RTC_DAY_OF_MONTH;
	tm->tm_mon =  RTC_MONTH - 1;
	tm->tm_year = RTC_YEAR + 100;
	tm->tm_wday = RTC_DAYOFWEEK;
	clk_disable(rtc_clk);
	//printk("To Lunix %d/%d/%d %d:%d:%d\n", tm->tm_year + 1900,tm->tm_mon,tm->tm_mday ,tm->tm_hour ,tm->tm_min ,tm->tm_sec);
	return 0;
}
int w55fa93_rtc_read_time_wrap(struct rtc_time *tm)
{
	return w55fa93_rtc_read_time(NULL, tm);
}
EXPORT_SYMBOL(w55fa93_rtc_read_time_wrap);

static int w55fa93_rtc_set_time(struct device *dev, struct rtc_time *tm)
{

	unsigned char mon, day, hrs, min, sec, leap_yr,wday;
	unsigned int yrs;
//	printk("Set time\n");
	//printk("From Lunix %d/%d/%d %d:%d:%d\n", tm->tm_year + 1900,tm->tm_mon,tm->tm_mday,tm->tm_hour,tm->tm_min,tm->tm_sec);
	clk_enable(rtc_clk);	
	yrs = 1900 + tm->tm_year;
	if(((yrs % 4) == 0) )
	{	
	    	if(((yrs % 100) == 0) && ((yrs % 400) != 0) )
	    		leap_yr = 0;	
	    	else
	    		leap_yr = 1;
	}
	else
		leap_yr = 0;

	yrs = tm->tm_year - 100;
	mon = tm->tm_mon + 1;
	day = tm->tm_mday;
	hrs = tm->tm_hour;
	min = tm->tm_min;
	sec = tm->tm_sec;
	wday = tm->tm_wday;

	check_rtc_power();

	if ((mon > 12) || (day == 0))		
	{
		clk_disable(rtc_clk);
		return -EINVAL;
	}
	if (day > (days_in_mo[mon] + ((mon == 2) && leap_yr)))
	{
		clk_disable(rtc_clk);
		return -EINVAL;
	}
	if ((hrs >= 24) || (min >= 60) || (sec >= 60))
	{
		clk_disable(rtc_clk);
		return -EINVAL;
	}
	writel((yrs/10)<<20|(yrs%10)<<16|(mon/10)<<12|(mon%10)<<8|(day/10)<<4|(day%10)<<0, REG_RTC_CLR);
	writel((hrs/10)<<20|(hrs%10)<<16|(min/10)<<12|(min%10)<<8|(sec/10)<<4|(sec%10)<<0, REG_RTC_TLR);
	writel(wday, REG_RTC_DWR);		
	clk_disable(rtc_clk);
	
	return 0;
}
int w55fa93_rtc_set_time_wrap(struct rtc_time *tm)
{
	return w55fa93_rtc_set_time(NULL, tm);
}
EXPORT_SYMBOL(w55fa93_rtc_set_time_wrap);

static int w55fa93_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	clk_enable(rtc_clk);
	check_rtc_power();
	(alrm->time).tm_sec = RTC_SECONDS_ALARM;
	(alrm->time).tm_min = RTC_MINUTES_ALARM;
	(alrm->time).tm_hour = RTC_HOURS_ALARM;
	(alrm->time).tm_mday = RTC_DAY_OF_MONTH_ALARM;
	(alrm->time).tm_mon = RTC_MONTH_ALARM - 1;
	(alrm->time).tm_year = RTC_YEAR_ALARM + 100;
	(alrm->time).tm_year = (alrm->time).tm_year;
	clk_disable(rtc_clk);
	return 0;
}
int w55fa93_rtc_read_alarm_wrap(struct rtc_wkalrm *alrm)
{
	return w55fa93_rtc_read_alarm(NULL, alrm);
}
EXPORT_SYMBOL(w55fa93_rtc_read_alarm_wrap);

static int w55fa93_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	unsigned char mon, day, hrs, min, sec,yrs ;
	clk_enable(rtc_clk);
	spin_lock_irq(&w55fa93_rtc_lock);

		yrs = (alrm->time).tm_year - 100;
		mon = (alrm->time).tm_mon + 1;
		day = (alrm->time).tm_mday;
		hrs = (alrm->time).tm_hour;
		min = (alrm->time).tm_min;
		sec = (alrm->time).tm_sec;

	if(alrm->enabled == 0){
		yrs = RTC_YEAR;
		mon = RTC_MONTH;
		day = RTC_DAY_OF_MONTH;
	}

	check_rtc_power();
	
	writel((yrs/10)<<20|(yrs%10)<<16|(mon/10)<<12|(mon%10)<<8|(day/10)<<4|(day%10)<<0, REG_RTC_CAR);
	writel((hrs/10)<<20|(hrs%10)<<16|(min/10)<<12|(min%10)<<8|(sec/10)<<4|(sec%10)<<0, REG_RTC_TAR);

		//if (alrm->enabled)
			enable_irq_wake(IRQ_RTC);
		//else
		//	disable_irq_wake(IRQ_RTC);
		
	spin_unlock_irq(&w55fa93_rtc_lock);
	clk_disable(rtc_clk);
	return 0;
}
int w55fa93_rtc_set_alarm_wrap(struct rtc_wkalrm *alrm)
{
	return w55fa93_rtc_set_alarm(NULL, alrm);
}
EXPORT_SYMBOL(w55fa93_rtc_set_alarm_wrap);


static struct rtc_class_ops w55fa93_rtc_ops = {
	.open = w55fa93_rtc_open,
	.release = w55fa93_rtc_release,
	.ioctl = w55fa93_rtc_ioctl,
	.read_time = w55fa93_rtc_read_time,
	.set_time = w55fa93_rtc_set_time,
	.read_alarm = w55fa93_rtc_read_alarm,
	.set_alarm = w55fa93_rtc_set_alarm,
	
};


static void timer_check_rtc_interrupt(unsigned long dummy)
{	
	clk_enable(rtc_clk);
	mod_timer(&rtc_int_timer, jiffies + RTC_INTERVAL_TIME); 	
	clk_disable(rtc_clk);
	return;
}

static int w55fa93_rtc_probe(struct platform_device *pdev)
{

	struct rtc_device *rtc;
	if (!request_region((unsigned long)REG_RTC_INIR, (unsigned long)RTC_IO_EXTENT, "rtc"))
	{
		printk(KERN_ERR "rtc: I/O port 0x%x is not free.\n", (unsigned int)REG_RTC_INIR);
		return -EIO;
	}


	rtc = rtc_device_register(pdev->name, &pdev->dev, &w55fa93_rtc_ops,
				THIS_MODULE);

	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	platform_set_drvdata(pdev, rtc);

        rtc_clk = clk_get(NULL, "rtc");
        if (IS_ERR(rtc_clk)) {
              
		printk("failed to get rtc clock\n");
        }
        clk_enable(rtc_clk);
	// enable long time press power disable
	if ((readl(REG_RTC_AER) & 0x10000) == 0x0) {
		// set RTC register access enable password
		writel(0xA965, REG_RTC_AER);
		// make sure RTC register read/write enable
		while ((readl(REG_RTC_AER) & 0x10000) == 0x0) ;
	}
	// check rtc power off is set or not
	if ((readl(REG_RTC_PWRON) & 0x5) != 0x5) {
		// press power key during 6 sec to power off (0x'6'0005)
		writel(0x60005, REG_RTC_PWRON);
		writel(readl(REG_RTC_RIER) & ~0x4, REG_RTC_RIER);
		writel(0x4, REG_RTC_RIIR);
	}
	clk_disable(rtc_clk);
	init_timer(&rtc_int_timer);
	rtc_int_timer.function = timer_check_rtc_interrupt;	/* timer handler */
	mod_timer(&rtc_int_timer, jiffies + RTC_INTERVAL_TIME); 

	printk("Init Nuvoton RTC!\n");

	return 0;
}

static int w55fa93_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *rtc = platform_get_drvdata(pdev);

 	if (rtc)
		rtc_device_unregister(rtc);
	release_region ((unsigned long)REG_RTC_INIR, (unsigned long)RTC_IO_EXTENT);
	return 0;
}

static struct platform_driver w55fa93_rtc_driver = {
	.probe		= w55fa93_rtc_probe,
	.remove		= w55fa93_rtc_remove,
	.driver		= {
		.name		= "w55fa93-rtc",
	},
};

static int __init w55fa93_rtc_init(void)
{
	return platform_driver_register(&w55fa93_rtc_driver);
}

static void __exit w55fa93_rtc_exit(void)
{
	platform_driver_unregister(&w55fa93_rtc_driver);
}

module_init(w55fa93_rtc_init);
module_exit(w55fa93_rtc_exit);

MODULE_DESCRIPTION("Nuvoton Realtime Clock Driver (RTC)");
MODULE_LICENSE("GPL");
