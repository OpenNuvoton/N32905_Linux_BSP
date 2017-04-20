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

//#include "../codecs/ac97.h"
//#include "nuc900-audio.h"
#include "../codecs/nau8822.h"

#include <mach/irqs.h>

#include <mach/w55fa93_audio.h>
#include <mach/w55fa93_spu.h>
#include <mach/w55fa93_reg.h>

//#define AUDIO_DEBUG
#define AUDIO_DEBUG_ENTER_LEAVE
#define AUDIO_DEBUG_MSG
#define AUDIO_DEBUG_MSG2

#ifdef AUDIO_DEBUG
#define DBG(fmt, arg...)			printk(fmt, ##arg)
#else
#define DBG(fmt, arg...)
#endif

#ifdef AUDIO_DEBUG_ENTER_LEAVE
#define ENTER()					DBG("[%-10s] : Enter\n", __FUNCTION__)
#define LEAVE()					DBG("[%-10s] : Leave\n", __FUNCTION__)
#else
#define ENTER()
#define LEAVE()
#endif

#undef 	outl 
#undef 	inl
#define outl 	writel
#define inl 	readl

#ifdef CONFIG_HEADSET_ENABLED		// for headset_detect
	#define HEADSET_IRQ_NUM 		W55FA93_IRQ(4)  // nIRQ_GPIO2
	#define Enable_IRQ(n)     		outl(1 << (n),REG_AIC_MECR)
	#define Disable_IRQ(n)    		outl(1 << (n),REG_AIC_MDCR)
	extern int enable_earphone_detect(void);	
#endif


#ifdef CONFIG_SND_SOC_W55FA93_SPU

static int w55fa93_audio_hw_params(struct snd_pcm_substream *substream,
                                  struct snd_pcm_hw_params *params)
{
		int ret;
	    struct snd_soc_pcm_runtime *rtd = substream->private_data;	
	    struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;	
	    unsigned int sample_rate = params_rate(params);

	ENTER();	

        ret = snd_soc_dai_set_sysclk(cpu_dai, W55FA93_AUDIO_CLKDIV, sample_rate, SND_SOC_CLOCK_OUT);        
        return 0;
        
	LEAVE();        
}

static struct snd_soc_ops w55fa93_audio_ops = {
        .hw_params = w55fa93_audio_hw_params,
};

static struct snd_soc_dai_link w55fa93evb_spu_dai = {
        .name		= "SPU",
        .stream_name	= "SPU HiFi",
        .cpu_dai	= &w55fa93_spu_dai,
        .codec_dai	= &w55fa93_dac_dai,
        .ops		= &w55fa93_audio_ops,        
};

static struct snd_soc_card w55fa93evb_audio_machine = {
        .name		= "W55FA93_SPU",
        .dai_link	= &w55fa93evb_spu_dai,
        .num_links	= 1,
        .platform	= &w55fa93_soc_platform,			// for pcm DMA
};

static struct snd_soc_device w55fa93evb_spu_devdata = {
        .card		= &w55fa93evb_audio_machine,		// for spu data link and pcm DMA
        .codec_dev	= &soc_codec_dev_w55fa93_dac,		// for w55fa93_dac probe/remove/suspend/resume
};

#endif

#ifdef CONFIG_SND_SOC_W55FA93_I2S
static int w55fa93_audio_hw_params(struct snd_pcm_substream *substream,
                                  struct snd_pcm_hw_params *params)
{
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
        struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
        struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
        unsigned int sample_rate = params_rate(params);
        int ret;
        unsigned int clk = 0;

		ENTER();
	
	    /* set codec DAI configuration */
        ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
                                  SND_SOC_DAIFMT_NB_NF |
                                  SND_SOC_DAIFMT_CBS_CFS);
        if (ret < 0)
                return ret;

        /* set cpu DAI configuration */
        ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
                                  SND_SOC_DAIFMT_NB_NF |
                                  SND_SOC_DAIFMT_CBS_CFS);
        if (ret < 0)
                return ret;

        switch (sample_rate) {
        case 8000:
        case 16000:
        case 24000:        
        case 32000:
        case 48000:
        case 96000:
                clk = 12288000;
                break;
        case 11025:
        case 22050:
        case 44100:
        case 88200:
                clk = 16934000;
                break;
        }

        /* set the codec system clock for DAC and ADC */
        ret = snd_soc_dai_set_sysclk(codec_dai, NAU8822_MCLK, clk, SND_SOC_CLOCK_OUT);
        if (ret < 0)
                return ret;

        /* set MCLK division for sample rate */
        DBG("sample_rate = %d !!!\n", sample_rate);
        
        ret = snd_soc_dai_set_sysclk(cpu_dai, W55FA93_AUDIO_SAMPLECLK, sample_rate, SND_SOC_CLOCK_OUT);
        if (ret < 0)
                return ret;

        /* set prescaler division for sample rate */
        ret = snd_soc_dai_set_sysclk(cpu_dai, W55FA93_AUDIO_CLKDIV, sample_rate, SND_SOC_CLOCK_OUT);
        if (ret < 0)
                return ret;

	LEAVE();        
        return 0;
}

static struct snd_soc_ops w55fa93_audio_ops = {
        .hw_params = w55fa93_audio_hw_params,
};

static struct snd_soc_dai_link w55fa93evb_i2s_dai = {
        .name		= "IIS",
        .stream_name	= "IIS HiFi",
        .cpu_dai	= &w55fa93_i2s_dai,
        .codec_dai	= &nau8822_dai,
        .ops		= &w55fa93_audio_ops,
};

static struct snd_soc_card w55fa93evb_audio_machine = {
        .name		= "W55FA93_IIS",
        .dai_link	= &w55fa93evb_i2s_dai,
        .num_links	= 1,
        .platform	= &w55fa93_soc_platform,
};

static struct snd_soc_device w55fa93_i2s_devdata = {
        .card		= &w55fa93evb_audio_machine,
        .codec_dev	= &soc_codec_dev_nau8822,
};
#endif


static struct platform_device *w55fa93evb_asoc_dev;

static int __init w55fa93_audio_init(void)
{
        int ret;
        
	ENTER();		        

        ret = -ENOMEM;
//        w55fa93evb_asoc_dev = platform_device_alloc("soc-audio", -1);
        w55fa93evb_asoc_dev = platform_device_alloc("soc-audio", 1);        
        
		printk("w55fa93evb_asoc_dev = 0x%x !!! \n", (unsigned int) w55fa93evb_asoc_dev);
	        
        if (!w55fa93evb_asoc_dev)
                goto out;

        /* w55fa93 audio device */
#ifdef CONFIG_SND_SOC_W55FA93_SPU
        platform_set_drvdata(w55fa93evb_asoc_dev, &w55fa93evb_spu_devdata);

        w55fa93evb_spu_devdata.dev = &w55fa93evb_asoc_dev->dev;
#endif

#ifdef CONFIG_SND_SOC_W55FA93_I2S
        platform_set_drvdata(w55fa93evb_asoc_dev, &w55fa93_i2s_devdata);

        w55fa93_i2s_devdata.dev = &w55fa93evb_asoc_dev->dev;
#endif

        ret = platform_device_add(w55fa93evb_asoc_dev);
	printk("ret = 0x%x !!!, platform device added \n", ret);        
        if (ret) {
                platform_device_put(w55fa93evb_asoc_dev);
                w55fa93evb_asoc_dev = NULL;
        }
	        
#ifdef CONFIG_HEADSET_ENABLED		// for headset_detect
    	enable_earphone_detect();    
#endif
out:
	LEAVE();        
        return ret;
}

static void __exit w55fa93_audio_exit(void)
{
	ENTER();		
        platform_device_unregister(w55fa93evb_asoc_dev);
	LEAVE();                
}

module_init(w55fa93_audio_init);
module_exit(w55fa93_audio_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("W55FA93 Series ASoC audio support");
MODULE_AUTHOR("Wan ZongShun");
