/*
 * Copyright (c) 2009-2010 Nuvoton technology corporation.
 *
 * Wan ZongShun <mcuos.com@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/suspend.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <linux/device.h>
#include <linux/clk.h>
#ifdef ERR1
#include <mach/mfp.h>
#endif
#include <mach/map.h>
#include <mach/regs-clock.h>

#include "w55fa93adc-audio.h"

//#define DBG
#ifdef DBG
#define ENTER()		printk("ENTER : %s  : %s\n",__FILE__, __FUNCTION__)
#define LEAVE()		printk("LEAVE : %s  : %s\n",__FILE__, __FUNCTION__)
#define ERRLEAVE()	printk("ERRLEAVE : %s  : %s\n",__FILE__, __FUNCTION__)
#else
#define ENTER(...)
#define LEAVE(...)	
#define ERRLEAVE(...)	
#endif

static DEFINE_MUTEX(i2s_mutex);
struct w55fa93adc_audio *g_pw55fa93_adc_data;

static int w55fa93_adc_set_fmt(struct snd_soc_dai *cpu_dai,
                              unsigned int fmt)
{	
	ENTER();
	LEAVE();
	return 0;
}

static int w55fa93_adc_set_sysclk(struct snd_soc_dai *cpu_dai,
                                 int clk_id, unsigned int freq, int dir)
{	
	ENTER();

	LEAVE();
    return 0;
}

typedef enum{ 
	eDRVADC_RECORD_MODE_0 = 0,                                             
	eDRVADC_RECORD_MODE_1,
	eDRVADC_RECORD_MODE_2,
	eDRVADC_RECORD_MODE_3
}E_DRVADC_RECORD_MODE;

extern void DrvADC_StartRecord(E_DRVADC_RECORD_MODE eRecordMode);
extern void DrvADC_StopRecord(void);

static int w55fa93_adc_trigger(struct snd_pcm_substream *substream,
                              int cmd, struct snd_soc_dai *dai)
{	
	//struct w55fa93adc_audio *w55fa93adc_audio = g_pw55fa93_adc_data;
	int ret = 0;
	ENTER();
	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:	
			DrvADC_StartRecord(eDRVADC_RECORD_MODE_1);	
			break;	
		case SNDRV_PCM_TRIGGER_STOP:
        case SNDRV_PCM_TRIGGER_SUSPEND:
			DrvADC_StopRecord();
			break;
	}
	LEAVE();
    return ret;
}

static int w55fa93_adc_probe(struct platform_device *pdev,
                            struct snd_soc_dai *dai)
{	
	int ret = 0;	
	ENTER();
	LEAVE();
   	return ret;
}

static void w55fa93_adc_remove(struct platform_device *pdev,
                              struct snd_soc_dai *dai)
{	
	ENTER();
	LEAVE();
    return;
}

static struct snd_soc_dai_ops w55fa93adc_cpu_dai_ops = {
	.trigger	= w55fa93_adc_trigger,
	.set_fmt	= w55fa93_adc_set_fmt,
	.set_sysclk	= w55fa93_adc_set_sysclk,
};

struct snd_soc_dai w55fa93adc_cpu_dai = {
	.name			= "w55fa93adc_cpu_dai",
	.id 			= 0,
	.probe			= w55fa93_adc_probe,
	.remove			= w55fa93_adc_remove,
#if 0
	.playback = {
		.rates		= SNDRV_PCM_RATE_8000_48000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min	= 1,
		.channels_max	= 2,
	},
#endif
	.capture = {
		.rates		= (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |SNDRV_PCM_RATE_16000),
		.formats	= SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min	= 1,
		.channels_max	= 1,
	},
	.ops = &w55fa93adc_cpu_dai_ops,
};
EXPORT_SYMBOL_GPL(w55fa93adc_cpu_dai);

static int __devinit w55fa93_adc_drvprobe(struct platform_device *pdev)
{	
    struct w55fa93adc_audio *w55fa93adc_audio;
    int ret;

	ENTER();
    if (g_pw55fa93_adc_data)
	{
		ERRLEAVE();	
        return -EBUSY;
	}
    w55fa93adc_audio = kzalloc(sizeof(struct w55fa93adc_audio), GFP_KERNEL);
    if (!w55fa93adc_audio)
	{
		ERRLEAVE();
        return -ENOMEM; 
	}
    spin_lock_init(&w55fa93adc_audio->lock);
	spin_lock_init(&w55fa93adc_audio->irqlock);		
        
    w55fa93adc_audio->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!w55fa93adc_audio->res) {
        ret = -ENODEV;
        goto out0;
    }

	if (!request_mem_region(w55fa93adc_audio->res->start,
							resource_size(w55fa93adc_audio->res), pdev->name)) {
		ret = -EBUSY;
		goto out0;
	}

	w55fa93adc_audio->mmio = ioremap(w55fa93adc_audio->res->start,
								 resource_size(w55fa93adc_audio->res));
	if (!w55fa93adc_audio->mmio) {
		ret = -ENOMEM;
		goto out1;
	}
#if 0
	w55fa93adc_audio->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(w55fa93adc_audio->clk)) {
		ret = PTR_ERR(w55fa93adc_audio->clk);
		goto out2;
	}
#endif
	w55fa93adc_audio->irq_num = platform_get_irq(pdev, 0);
	if (!w55fa93adc_audio->irq_num) {
		ret = -EBUSY;
		goto out2;
	}
		
	ret = w55fa93adc_dma_create(w55fa93adc_audio);
	if (ret != 0)
	{
		ERRLEAVE();	
		return ret;
	}
	g_pw55fa93_adc_data = w55fa93adc_audio;

	w55fa93adc_audio->dev = w55fa93adc_cpu_dai.dev =  &pdev->dev;

	ret = snd_soc_register_dai(&w55fa93adc_cpu_dai);
	if (ret)
		goto out3;
	#if 0
    mfp_set_groupg(w55fa93adc_audio->dev); /* enbale i2s multifunction pin*/
	#endif
	printk("******* global g_pw55fa93_adc_data address = 0x%x\n", (unsigned int)g_pw55fa93_adc_data);
	LEAVE();
    return 0;

out3:
#if 0
    clk_put(w55fa93adc_audio->clk);
#endif
out2:
    iounmap(w55fa93adc_audio->mmio);
out1:
    release_mem_region(w55fa93adc_audio->res->start,
                           resource_size(w55fa93adc_audio->res));
out0:
    kfree(w55fa93adc_audio);
	ERRLEAVE();
    return ret;
}

static int __devexit w55fa93_adc_drvremove(struct platform_device *pdev)
{	
	ENTER();
	w55fa93adc_dma_destroy(g_pw55fa93_adc_data);
		
    snd_soc_unregister_dai(&w55fa93adc_cpu_dai);
#if 0
    clk_put(g_pw55fa93_adc_data->clk);
#endif
    iounmap(g_pw55fa93_adc_data->mmio);
    release_mem_region(g_pw55fa93_adc_data->res->start,
                        resource_size(g_pw55fa93_adc_data->res));

    g_pw55fa93_adc_data = NULL;
	LEAVE();	
        return 0;
}

static struct platform_driver w55fa93_adc_driver = {
        .driver	= {
                .name	= "w55fa93-audio-adc",	/* !!!!The name need match with /arch/arm/mach-w55fa93/dev.c */
                .owner	= THIS_MODULE,		/* It may need change to ADC's or EDMA's mapping address */	
        },					/* Otherwise, the w55fa93_adc_drvprobe does not launch */
        .probe		= w55fa93_adc_drvprobe,			
        .remove		= __devexit_p(w55fa93_adc_drvremove),
};

static int __init w55fa93_adc_init(void)
{	
	int ret;	
	ENTER();
	ret = platform_driver_register(&w55fa93_adc_driver);
	printk(" %d = platform_driver_register\n ", ret);
	LEAVE();
	return ret;
}

static void __exit w55fa93_adc_exit(void)
{	
	ENTER();
	platform_driver_unregister(&w55fa93_adc_driver);
	LEAVE();
}

module_init(w55fa93_adc_init);
module_exit(w55fa93_adc_exit);

MODULE_AUTHOR("Wan ZongShun <mcuos.com@gmail.com>");
MODULE_DESCRIPTION("W55FA93 ADC SoC driver!");
MODULE_LICENSE("GPL");
