/* linux/drivers/misc/w55fa93_pwm.c
 *
 * Copyright (c) 2009 Nuvoton technology.
 * Wan ZongShun <mcuos.com@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>

#include <linux/platform_device.h>
#include <mach/w55fa93_reg.h>

extern unsigned int w55fa93_apb_clock;

#define w55fa93_PWM_MAX_COUNT	0xFFFF

struct w55fa93_pwm {	
	struct clk	*clk;
	u32		enable;
	u32		duty_percent;
	long		freq;
	long		duty_cycle;
	long		period;
};

/*
 * /sys/devices/platform/w55fa92-pwm.N
 *   /enable       read-write   Enable/disable the PWM signal. (0 - disabled, 1 - enabled)
 *   /period          read-write  The total period of the PWM signal. Value is in nanoseconds.
 *   /duty_cycle  read-write  The active time of the PWM signal. Value is in nanoseconds.
 *   /polarity        read-write  Changes the polarity of the PWM signal. Value is the string "normal" or "inversed".
 */

static ssize_t w55fa93_pwm_get_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct w55fa93_pwm *pwm = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", pwm->enable);
}

static ssize_t w55fa93_pwm_set_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct w55fa93_pwm *pwm = platform_get_drvdata(pdev);
	long val;
	int err;	

	err = strict_strtol(buf, 10, &val);
	if (err)
		return -EINVAL;

	pwm->enable = val;

	if(val != 0)
	{
		__raw_writel((__raw_readl(REG_POE) | (0x01 << (pdev->id))), REG_POE);		
		__raw_writel((__raw_readl(REG_PCR) | (0x01 << (pdev->id * 8))), REG_PCR);
	}
	else
	{
		__raw_writel((__raw_readl(REG_POE) & ~(0x01 << (pdev->id))), REG_POE);		
		__raw_writel((__raw_readl(REG_PCR) & ~(0x01 << (pdev->id * 8))), REG_PCR);
	}

	return count;	
}

static ssize_t w55fa93_pwm_get_period(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct w55fa93_pwm *pwm = platform_get_drvdata(pdev);

	return sprintf(buf, "%ld\n", pwm->period);
}

uint32_t w55fa93_pwm_config(uint32_t u32ChannelNum, uint32_t u32Frequency, uint32_t u32DutyCycle)
{
    uint32_t i;
    uint32_t u32PWM_CLock=0;
    uint8_t  u8Divider = 1, u8Prescale = 0xFF;
    uint16_t u16CNR = 0xFFFF;
	uint32_t u32SFR;
	
	u32PWM_CLock = w55fa93_apb_clock * 1000;	

    for(; u8Divider < 17; u8Divider <<= 1) {  // clk divider could only be 1, 2, 4, 8, 16
        i = (u32PWM_CLock / u32Frequency) / u8Divider;
        // If target value is larger than CNR * prescale, need to use a larger divider
        if(i > (0x10000 * 0x100))
            continue;

        // CNR = 0xFFFF + 1, get a prescaler that CNR value is below 0xFFFF
        u8Prescale = (i + 0xFFFF)/ 0x10000;

        // u8Prescale must at least be 2, otherwise the output stop
        if(u8Prescale < 3)
            u8Prescale = 2;

        i /= u8Prescale;

        if(i <= 0x10000) {
            if(i == 1)
                u16CNR = 1;     // Too fast, and PWM cannot generate expected frequency...
            else
                u16CNR = i;
            break;
        }

    }
    // Store return value here 'cos we're gonna change u8Divider & u8Prescale & u16CNR to the real value to fill into register
    i = u32PWM_CLock / (u8Prescale * u8Divider * u16CNR);

    u8Prescale -= 1;
    u16CNR -= 1;
    // convert to real register value
    if(u8Divider == 1)
        u8Divider = 4;
    else if (u8Divider == 2)
        u8Divider = 0;
    else if (u8Divider == 4)
        u8Divider = 1;
    else if (u8Divider == 8)
        u8Divider = 2;
    else // 16
        u8Divider = 3;

    // every two channels share a prescaler
	__raw_writel((__raw_readl(REG_PPR) & ~(CP0 << ((u32ChannelNum >> 1) * 8))) | (u8Prescale << ((u32ChannelNum >> 1) * 8)), REG_PPR);
	__raw_writel((__raw_readl(REG_PWM_CSR) & ~(CSR0 << (4 * u32ChannelNum))) | (u8Divider << (4 * u32ChannelNum)), REG_PWM_CSR);
	__raw_writel((__raw_readl(REG_PCR) | (1 << (3 + 8 * u32ChannelNum))), REG_PCR);

	u32SFR = (u32)REG_CMR0 + 12 * u32ChannelNum;
	if(u32DutyCycle == 0)
		__raw_writel(0, u32SFR);
	else
		__raw_writel((u32DutyCycle * (u16CNR + 1) / 100 - 1), u32SFR);

	u32SFR = (u32)REG_CNR0 + 12 * u32ChannelNum;
	__raw_writel(u16CNR, u32SFR);    
   
    return(i);
}

static ssize_t w55fa93_pwm_set_period(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct w55fa93_pwm *pwm = platform_get_drvdata(pdev);
	long val, freq=0;
	int err;	

	err = strict_strtol(buf, 10, &val);
	if (err)
		return -EINVAL;

	if(val != 0)
		freq = (1 * 1000000) / (val / 1000);

	pwm->freq = freq;
	pwm->period = val;

	if (val == 0) {				
		__raw_writel((__raw_readl(REG_PCR) & ~(0x01 << (pdev->id * 8))), REG_PCR);
		
	} else if (freq <= ((w55fa93_apb_clock * 1000) / 2 /2)) {

		w55fa93_pwm_config(pdev->id, pwm->freq, pwm->duty_percent);	
		
	} else {
		return -EINVAL;
	}

	return count;
}

static ssize_t w55fa93_pwm_get_duty_cycle(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct w55fa93_pwm *pwm = platform_get_drvdata(pdev);

	return sprintf(buf, "%ld\n", pwm->duty_cycle);
}

static ssize_t w55fa93_pwm_set_duty_cycle(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct w55fa93_pwm *pwm = platform_get_drvdata(pdev);
	long val;
	int err;
	u32 term;

	err = strict_strtol(buf, 10, &val);
	if (err)
		return -EINVAL;
	
	pwm->duty_cycle = val;

	if(pwm->period != 0)
	{
		pwm->duty_percent = 100 * val / pwm->period;
		
		term = __raw_readl(REG_CNR0 + 12 * pdev->id);	
		
		if(pwm->duty_percent != 0)
			__raw_writel(((term + 1) * pwm->duty_percent / 100) - 1, REG_CMR0 + 12 * pdev->id);
		else
			__raw_writel(0, REG_CMR0 + 12 * pdev->id);
		
	}

	return count;
}

static ssize_t w55fa93_pwm_get_polarity(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);	
	u32 u32Value;

	u32Value = __raw_readl(REG_PCR);
	u32Value = u32Value & (0x04 << (pdev->id * 8));

	if(u32Value == 0)
		return sprintf(buf, "normal\n");
	else
		return sprintf(buf, "inversed\n");
}

static ssize_t w55fa93_pwm_set_polarity(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);	
	u32 u32Value;	

	u32Value = __raw_readl(REG_PCR);

	if (memcmp("normal", buf, 6) == 0)
		u32Value &= ~(0x04 << (pdev->id * 8));		
	else if (memcmp("inversed", buf, 8) == 0)
		u32Value |= (0x04 << (pdev->id * 8));		
	else
		return -EINVAL;

	__raw_writel(u32Value, REG_PCR);

	return count;
}

static DEVICE_ATTR(enable, S_IWUGO | S_IRUGO,
		   w55fa93_pwm_get_enable, w55fa93_pwm_set_enable);
static DEVICE_ATTR(period, S_IWUGO | S_IRUGO,
		   w55fa93_pwm_get_period, w55fa93_pwm_set_period);
static DEVICE_ATTR(duty_cycle, S_IWUGO | S_IRUGO,
		   w55fa93_pwm_get_duty_cycle, w55fa93_pwm_set_duty_cycle);
static DEVICE_ATTR(polarity, S_IWUGO | S_IRUGO,
		   w55fa93_pwm_get_polarity, w55fa93_pwm_set_polarity);


static struct attribute *w55fa93_pwm_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_period.attr,
	&dev_attr_duty_cycle.attr,
	&dev_attr_polarity.attr,
	NULL
};

static const struct attribute_group w55fa93_pwm_sysfs_files = {
	.attrs	= w55fa93_pwm_attrs,
};

static int __init w55fa93_pwm_probe(struct platform_device *pdev)
{
	struct w55fa93_pwm *pwm;
	struct resource *res;
	int err;	

	pwm = kzalloc(sizeof(struct w55fa93_pwm), GFP_KERNEL);
	if (!pwm) {
		err = -ENOMEM;
		goto fail_no_mem;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		err = -ENXIO;
		goto fail_no_mem_resource;
	}

	res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (res == NULL) {
		err = -EBUSY;
		goto fail_no_mem_resource;
	}	

	err = sysfs_create_group(&pdev->dev.kobj, &w55fa93_pwm_sysfs_files);
	if (err)
		goto fail_no_sysfs;

	pwm->clk = clk_get(&pdev->dev, "w55fa93-pwm");
	if (IS_ERR(pwm->clk)) {
		err = PTR_ERR(pwm->clk);
		goto fail_no_clk;
	}
	
	if(pdev->id == 0)
		__raw_writel((__raw_readl(REG_GPDFUN) & ~0x00000003) | 0x00000002, REG_GPDFUN);	
	else if(pdev->id == 1)	
		__raw_writel((__raw_readl(REG_GPDFUN) & ~0x0000000C) | 0x00000008, REG_GPDFUN);
	else if(pdev->id == 2)
		__raw_writel((__raw_readl(REG_GPDFUN) & ~0x00000030) | 0x00000020, REG_GPDFUN);
	else if(pdev->id == 3)
		__raw_writel((__raw_readl(REG_GPDFUN) & ~0x000000C0) | 0x00000080, REG_GPDFUN);

	pwm->duty_percent = 50;

	platform_set_drvdata(pdev, pwm);	

	clk_enable(pwm->clk);

	return 0;

fail_no_clk:
	sysfs_remove_group(&pdev->dev.kobj, &w55fa93_pwm_sysfs_files);
fail_no_sysfs:	
	release_mem_region(res->start, resource_size(res));
fail_no_mem_resource:
	kfree(pwm);
fail_no_mem:
	return err;
}

static int __exit w55fa93_pwm_remove(struct platform_device *pdev)
{
	struct w55fa93_pwm *pwm = platform_get_drvdata(pdev);
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		
	__raw_writel((__raw_readl(REG_PCR) & ~(0x01 << (pdev->id * 8))), REG_PCR);
	
	clk_disable(pwm->clk);
	clk_put(pwm->clk);
	platform_set_drvdata(pdev, NULL);
	sysfs_remove_group(&pdev->dev.kobj, &w55fa93_pwm_sysfs_files);	
	release_mem_region(res->start, resource_size(res));
	kfree(pwm);
	if(pdev->id == 0)		
		__raw_writel(__raw_readl(REG_GPDFUN) & ~0x00000003, REG_GPDFUN);	
	else if(pdev->id == 1)		
		__raw_writel(__raw_readl(REG_GPDFUN) & ~0x0000000C, REG_GPDFUN);
	else if(pdev->id == 2)
		__raw_writel(__raw_readl(REG_GPDFUN) & ~0x00000030, REG_GPDFUN);
	else if(pdev->id == 3)
		__raw_writel(__raw_readl(REG_GPDFUN) & ~0x000000C0, REG_GPDFUN);

	return 0;
}

static struct platform_driver w55fa93_pwm_driver = {
	.driver		= {
		.name	= "w55fa93-pwm",
		.owner	= THIS_MODULE,
	},
	.remove		= __exit_p(w55fa93_pwm_remove),
};

static int __init w55fa93_pwm_init(void)
{
	return platform_driver_probe(&w55fa93_pwm_driver, w55fa93_pwm_probe);
}

static void __exit w55fa93_pwm_exit(void)
{
	platform_driver_unregister(&w55fa93_pwm_driver);
}

module_init(w55fa93_pwm_init);
module_exit(w55fa93_pwm_exit);

MODULE_DESCRIPTION("W55FA93 PWM driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:w55fa93-pwm");
