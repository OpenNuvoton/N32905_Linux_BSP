/*
 * Copyright (c) 2010 Nuvoton technology corporation.
 *
 * Wan ZongShun <mcuos.com@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "../codecs/ac97.h"
#include "w55fa93adc-audio.h"
#include "../codecs/w55fa93adc.h"

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

static int w55fa93adc_audio_hw_params(struct snd_pcm_substream *substream,
                                  struct snd_pcm_hw_params *params)
{	
	ENTER();
	LEAVE();
	return 0;
}

static struct snd_soc_ops w55fa93adc_audio_ops = {
        .hw_params = w55fa93adc_audio_hw_params,
};

static struct snd_soc_dai_link w55fa93adc_dai_link = {
	.name		= "IIS",
	.stream_name	= "IIS HiFi",
	/* nuc900-i2s.c:  struct snd_soc_dai w55fa93adc_cpu_dai */
	.cpu_dai	= &w55fa93adc_cpu_dai,
	/* ../codec/w55fa93adc.c: struct snd_soc_dai w55fa93adc_codec_dai */
	.codec_dai	= &w55fa93adc_codec_dai,
	.ops		= &w55fa93adc_audio_ops,
};

static struct snd_soc_card w55fa93adc_audio_machine = {
	.name		= "mach-W55FA93_ADC",
	.dai_link	= &w55fa93adc_dai_link,   
	.num_links	= 1,
	/* nuc900-pcm.c: struct snd_soc_platform w55fa93adc_soc_platform */
	.platform	= &w55fa93adc_soc_platform,	
};

static struct snd_soc_device w55fa93adc_devdata = {
	.card		= &w55fa93adc_audio_machine,
	.codec_dev	= &soc_codec_dev_w55fa93adc,
};

static struct platform_device *w55fa93adc_asoc_dev;

static int __init w55FA93adc_audio_init(void)
{	
	int ret;

	ENTER();
	ret = -ENOMEM;
	w55fa93adc_asoc_dev = platform_device_alloc("soc-audio", 0);
	if (!w55fa93adc_asoc_dev)
			goto out;

	platform_set_drvdata(w55fa93adc_asoc_dev, &w55fa93adc_devdata);
	w55fa93adc_devdata.dev = &w55fa93adc_asoc_dev->dev;	
		
	ret = platform_device_add(w55fa93adc_asoc_dev);
	if (ret) {	
		platform_device_put(w55fa93adc_asoc_dev);
		w55fa93adc_asoc_dev = NULL;
	}
out:	
	if(ret==0)
		LEAVE();
	else
		ERRLEAVE();
		
	return ret;
}

static void __exit w55fa93adc_audio_exit(void)
{	
	ENTER();
	platform_device_unregister(w55fa93adc_asoc_dev);
	LEAVE();
}

module_init(w55FA93adc_audio_init);
module_exit(w55fa93adc_audio_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("W55FA93 Series ASoC audio support");
MODULE_AUTHOR("Wan ZongShun");
