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
#include <sound/soc-dapm.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <sound/tlv.h>

//#include <mach/mfp.h>

//#include "nuc900-audio.h"

#include <mach/w55fa93_audio.h>
#include <mach/w55fa93_spu.h>
#include <mach/w55fa93_reg.h>

//#define DAC_DEBUG
#define DAC_DEBUG_ENTER_LEAVE
#define DAC_DEBUG_MSG
#define DAC_DEBUG_MSG2

#ifdef DAC_DEBUG
#define DBG(fmt, arg...)			printk(fmt, ##arg)
#else
#define DBG(fmt, arg...)
#endif


#ifdef DAC_DEBUG_ENTER_LEAVE
#define ENTER()					DBG("[%-10s] : Enter\n", __FUNCTION__)
#define LEAVE()					DBG("[%-10s] : Leave\n", __FUNCTION__)
#else
#define ENTER()
#define LEAVE()
#endif

#ifdef DAC_DEBUG_MSG
#define MSG(fmt)					DBG("[%-10s] : "fmt, __FUNCTION__)
#else
#define MSG(fmt)
#endif

#ifdef DAC_DEBUG_MSG2
#define MSG2(fmt, arg...)			DBG("[%-10s] : "fmt, __FUNCTION__, ##arg)
#else
#define MSG2(fmt, arg...)
#endif


//static AUDIO_T	_tSpu;

static int s_rightVol = 0, s_leftVol = 0;
extern int _u8Channel0, _u8Channel1;
	
static volatile int _bPlayDmaToggle, _bRecDmaToggle;

static struct snd_soc_codec *w55fa93_dac;
extern struct platform_device w55fa93_device_audio_spu;
extern struct w55fa93_audio *w55fa93_spu_data;

#define OPT_SPU_I2C_CONTROL		// by I2C driver to control SPU volume level

//===========================================================
#define W55FA93_DAC_CACHEREGNUM				4

// pseudo registers definition for SPU volume control in ALSA
#define W55FA93_DAC_RESET					0x00		
#define W55FA93_DAC_POWER_MANAGEMENT		0x01
#define W55FA93_DAC_LEFT_DIGITAL_VOLUME		0x02
#define W55FA93_DAC_RIGHT_DIGITAL_VOLUME	0x03

//static const DECLARE_TLV_DB_SCALE(digital_tlv, -3100, 1000, 1);
static const DECLARE_TLV_DB_SCALE(digital_tlv, -3100, 50, 1);
											//	 | 	   |  |--> least value is muted B
											//   |	   |------> step size = 0.5dB (50*0.01)
											//   |-------------> least value = -31dB (-3,100*0.01)
											
static const u16 nau8822_reg[W55FA93_DAC_CACHEREGNUM] = {
        0x0000, 0x00FF, 0x0000, 0x0000,	/* 0x00...0x03 */		// default value of pseudo registers for w55fa93 dac
};

enum w55fa93_dac_sysclk_src {
	W55FA93_DAC_PLL,
	W55FA93_DAC_MCLK
};

/* codec private data */
struct w55fa93_dac_priv {
        struct snd_soc_codec codec;
        unsigned int f_pllout;
        unsigned int f_mclk;
        unsigned int f_256fs;
        unsigned int f_opclk;
        int mclk_idx;
        enum w55fa93_dac_sysclk_src sysclk;
        u16 reg_cache[W55FA93_DAC_CACHEREGNUM];
};

static const struct snd_kcontrol_new w55fa93_dac_snd_controls[] = {

        SOC_DOUBLE_R_TLV("PCM Volume",
        W55FA93_DAC_LEFT_DIGITAL_VOLUME, W55FA93_DAC_RIGHT_DIGITAL_VOLUME,
        0, 63, 0, digital_tlv),			// set 64 volume levels 
};


static const struct snd_soc_dapm_widget w55fa93_dac_dapm_widgets[] = {

//        SND_SOC_DAPM_DAC("Ramp", "Power Down",
//        W55FA93_DAC_POWER_MANAGEMENT, 0, 1),		// DAMP: dynamic audio power management
	
        SND_SOC_DAPM_DAC("Left DAC", "Power Down",
        W55FA93_DAC_POWER_MANAGEMENT, 1, 1),
        SND_SOC_DAPM_DAC("Right DAC", "Power Down",
        W55FA93_DAC_POWER_MANAGEMENT, 2, 1),

//        SND_SOC_DAPM_DAC("Left DAC", "Power Down of Volume Control",
//        W55FA93_DAC_POWER_MANAGEMENT, 3, 1),
//        SND_SOC_DAPM_DAC("Right DAC", "Power Down of Volume Control",
//        W55FA93_DAC_POWER_MANAGEMENT, 4, 1),

//        SND_SOC_DAPM_DAC("Left/Right Headphone Amp", "Power Down",
//        W55FA93_DAC_POWER_MANAGEMENT, 5, 1),

//        SND_SOC_DAPM_DAC("Reference Voltage", "Power Down",
//        W55FA93_DAC_POWER_MANAGEMENT, 6, 1),

//        SND_SOC_DAPM_DAC("Current Bias", "Power Down",
//        W55FA93_DAC_POWER_MANAGEMENT, 7, 1),
        
        SND_SOC_DAPM_OUTPUT("LHP"),
        SND_SOC_DAPM_OUTPUT("RHP"),
        
};

static const struct snd_soc_dapm_route audio_map[] = {
	
        {"LHP", NULL, "Left DAC"},	// "destination <-- switch <-- source", define left DAC path
        {"RHP", NULL, "Right DAC"},	// "destination <-- switch <-- source", define Right DAC path
};	

static int w55fa93_dac_add_widgets(struct snd_soc_codec *codec)
{
        snd_soc_dapm_new_controls(codec, w55fa93_dac_dapm_widgets,
                                  ARRAY_SIZE(w55fa93_dac_dapm_widgets));

        /* set up the W55FA93_DAC audio map */
        snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

        return 0;
}

/*
 * Set PCM DAI bit size and sample rate.
 */
static int w55fa93_dac_hw_params(struct snd_pcm_substream *substream,
                            struct snd_pcm_hw_params *params,
                            struct snd_soc_dai *dai)
{
//        struct snd_soc_pcm_runtime *rtd = substream->private_data;
//        struct snd_soc_device *socdev = rtd->socdev;
//        struct snd_soc_codec *codec = socdev->card->codec;
//        struct w55fa93_dac_priv *w55fa93_dac = snd_soc_codec_get_drvdata(codec);

        struct w55fa93_audio *w55fa93_audio = w55fa93_spu_data;

	ENTER();
	
		//Disable_Int(IRQ_ACTL);
		AUDIO_WRITE(REG_AIC_MDCR,(1<<IRQ_SPU));
	
		/* enable audio enigne clock */
	    clk_enable(w55fa93_audio->eng_clk);        		
    	clk_enable(w55fa93_audio->spu_clk);        	

		return 0;
}

static int w55fa93_dac_mute(struct snd_soc_dai *dai, int mute)
{
	int buf;
	
	
	return 0;
	ENTER();	
	
		buf = AUDIO_READ(REG_SPU_DAC_VOL) & 0xFFFF;
		s_rightVol = buf & RHPVL;
		s_leftVol = (buf & LHPVL) >> 8;
		
		AUDIO_WRITE(REG_SPU_DAC_VOL, AUDIO_READ(REG_SPU_DAC_VOL) & ~(LHPVL | RHPVL));	
		
		DBG("REG_SPU_DAC_VOL = 0x%x  !!!\n", AUDIO_READ(REG_SPU_DAC_VOL));		
	LEAVE();		
		return 0;
}

/*
 * Set ADC and Voice DAC format.
 */
static int w55fa93_dac_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	ENTER();		
        return 0;
}

/*
 * Configure NAU8822 clock dividers.
 */
static int w55fa93_dac_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
                                 int div_id, int div)
{
	ENTER();		
        return 0;
}

/*
 * @freq:	when .set_pll() is not used, freq is codec MCLK input frequency
 */
static int w55fa93_dac_set_dai_sysclk(struct snd_soc_dai *codec_dai, int clk_id,
                                 unsigned int freq, int dir)
{
	ENTER();		
		return 0;
}


#define W55FA93_DAC_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops w55fa93_dac_dai_ops = {
        .hw_params	= w55fa93_dac_hw_params,
        .digital_mute = w55fa93_dac_mute,
        .set_fmt	= w55fa93_dac_set_dai_fmt,
        .set_clkdiv	= w55fa93_dac_set_dai_clkdiv,
        .set_sysclk	= w55fa93_dac_set_dai_sysclk,
};

struct snd_soc_dai w55fa93_dac_dai = {
        .name = "W55FA93_DAC HiFi",
        .id = 1,
        .playback = {
                .stream_name = "Playback",
                .channels_min = 1,
                .channels_max = 2,
                .rates = SNDRV_PCM_RATE_8000_48000,
                .formats = W55FA93_DAC_FORMATS,
        },
        .capture = {
                .stream_name = "Capture",
                .channels_min = 1,
                .channels_max = 2,
                .rates = SNDRV_PCM_RATE_8000_48000,
                .formats = W55FA93_DAC_FORMATS,
        },
        .ops = &w55fa93_dac_dai_ops,
       };
EXPORT_SYMBOL_GPL(w55fa93_dac_dai);


static int w55fa93_dac_suspend(struct platform_device *pdev, pm_message_t state)
{
		int buf;

	ENTER();			
		
		buf = AUDIO_READ(REG_SPU_DAC_VOL) & 0xFFFF;
		s_rightVol = buf & RHPVL;
		s_leftVol = (buf & LHPVL) >> 8;
		
		AUDIO_WRITE(REG_SPU_DAC_VOL, AUDIO_READ(REG_SPU_DAC_VOL) & ~(LHPVL | RHPVL));	

        return 0;
}

static int w55fa93_dac_resume(struct platform_device *pdev)
{
	ENTER();	
		
		AUDIO_WRITE(REG_SPU_DAC_VOL, (AUDIO_READ(REG_SPU_DAC_VOL) & ~(LHPVL | RHPVL)) | (s_rightVol & 0x3F) | (s_leftVol & 0x3F) << 8);	
        return 0;
}

static int w55fa93_dac_probe(struct platform_device *pdev)
{
        struct snd_soc_device *socdev = platform_get_drvdata(pdev);
        struct snd_soc_codec *codec;
        int ret = 0;

        if (w55fa93_dac == NULL) {
                dev_err(&pdev->dev, "Codec device not registered\n");
                return -ENODEV;
        }

        socdev->card->codec = w55fa93_dac;
        codec = w55fa93_dac;

        /* register pcms */
        ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
        if (ret < 0) {
                dev_err(codec->dev, "failed to create pcms: %d\n", ret);
                goto pcm_err;
        }

        snd_soc_add_controls(codec, w55fa93_dac_snd_controls,
                             ARRAY_SIZE(w55fa93_dac_snd_controls));


                             
        w55fa93_dac_add_widgets(codec);

pcm_err:
        return ret;
}

/* power down chip */
static int w55fa93_dac_remove(struct platform_device *pdev)
{
        struct snd_soc_device *socdev = platform_get_drvdata(pdev);

        snd_soc_free_pcms(socdev);
        snd_soc_dapm_free(socdev);
        return 0;
}

struct snd_soc_codec_device soc_codec_dev_w55fa93_dac = {
        .probe		= w55fa93_dac_probe,		
        .remove		= w55fa93_dac_remove,
        .suspend	= w55fa93_dac_suspend,
        .resume		= w55fa93_dac_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_w55fa93_dac);

static const int update_reg[] = {
		W55FA93_DAC_LEFT_DIGITAL_VOLUME,
		W55FA93_DAC_RIGHT_DIGITAL_VOLUME,
};

static __devinit int w55fa93_register(struct w55fa93_dac_priv *w55fa93)
{
        int ret;
        struct snd_soc_codec *codec = &w55fa93->codec;

        if (w55fa93_dac) {
                dev_err(codec->dev, "Another NAU8822 is registered\n");
                return -EINVAL;
        }
		
        /*
         * Set default system clock to PLL, it is more precise, this is also the
         * default hardware setting
         */
//        nau8822->sysclk = NAU8822_PLL;

        mutex_init(&codec->mutex);
        INIT_LIST_HEAD(&codec->dapm_widgets);
        INIT_LIST_HEAD(&codec->dapm_paths);

        snd_soc_codec_set_drvdata(codec, w55fa93);
        codec->name = "W55FA93_DAC";
        codec->owner = THIS_MODULE;
        codec->bias_level = SND_SOC_BIAS_OFF;
//        codec->set_bias_level = w55fa93_set_bias_level;
        codec->dai = &w55fa93_dac_dai;
        codec->num_dai = 1;
        codec->reg_cache_size = W55FA93_DAC_CACHEREGNUM;
        codec->reg_cache = &w55fa93->reg_cache;

        ret = snd_soc_codec_set_cache_io(codec, 7, 9, SND_SOC_I2C);
//        ret = snd_soc_codec_set_cache_io(codec, 8, 8, SND_SOC_I2C);
        if (ret < 0) {
                dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
                goto err;
        }

        memcpy(codec->reg_cache, nau8822_reg, sizeof(nau8822_reg));

        /*
         * Set the update bit in all registers, that have one. This way all
         * writes to those registers will also cause the update bit to be
         * written.
         */
//        for (i = 0; i < ARRAY_SIZE(update_reg); i++)
//                ((u16 *)codec->reg_cache)[update_reg[i]] |= 0x100;

        /* Reset the codec */
//        ret = snd_soc_write(codec, NAU8822_RESET, 0);
//        if (ret < 0) {
//                dev_err(codec->dev, "Failed to issue reset\n");
//                goto err;
//        }

        w55fa93_dac_dai.dev = codec->dev;
//        w55fa93_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
        w55fa93_dac = codec;
        ret = snd_soc_register_codec(codec);
        if (ret != 0) {
                dev_err(codec->dev, "Failed to register codec: %d\n", ret);
                goto err;
        }

        ret = snd_soc_register_dai(&w55fa93_dac_dai);
        if (ret != 0) {
                dev_err(codec->dev, "Failed to register DAI: %d\n", ret);
                goto err_codec;
        }

//        snd_soc_update_bits(codec, NAU8822_POWER_MANAGEMENT_1, 0x10, 0x10);
        return 0;

err_codec:
        snd_soc_unregister_codec(codec);
err:
        kfree(w55fa93);
        return ret;
}

static __devexit void w55fa93_unregister(struct w55fa93_dac_priv *w55fa93)
{
        snd_soc_unregister_dai(&w55fa93_dac_dai);
        snd_soc_unregister_codec(&w55fa93->codec);
        kfree(w55fa93);
        w55fa93_dac = NULL;
}

static __devinit int w55fa93_dac_i2c_probe(struct i2c_client *i2c,
                                      const struct i2c_device_id *id)
{
        struct w55fa93_dac_priv *w55fa93;
        struct snd_soc_codec *codec;

		ENTER();	

        w55fa93 = kzalloc(sizeof(struct w55fa93_dac_priv), GFP_KERNEL);
        if (w55fa93 == NULL)
                return -ENOMEM;

        codec = &w55fa93->codec;
        codec->hw_write = (hw_write_t)i2c_master_send;
        i2c_set_clientdata(i2c, w55fa93);
        codec->control_data = i2c;
        codec->dev = &i2c->dev;
//        codec->dev = 0x11;	// temp value, non-zero 

        return w55fa93_register(w55fa93);
}

static __devexit int w55fa93_dac_i2c_remove(struct i2c_client *client)
{
        struct w55fa93_dac_priv  *w55fa93 = i2c_get_clientdata(client);
        w55fa93_unregister(w55fa93);
        return 0;
}

static const struct i2c_device_id w55fa93_dac_i2c_id[] = {
        { "w55fa93_dac", 0 },
        { }
};
MODULE_DEVICE_TABLE(i2c, w55fa93_dac_i2c_id);

static struct i2c_driver w55fa93_dac_i2c_driver = {
        .driver = {
                .name = "W55FA93_DAC",
                .owner = THIS_MODULE,
        },
        .probe =    w55fa93_dac_i2c_probe,
        .remove =   __devexit_p(w55fa93_dac_i2c_remove),
        .id_table = w55fa93_dac_i2c_id,
};

static int __init w55fa93_dac_modinit(void)
{
		ENTER();
			
        return i2c_add_driver(&w55fa93_dac_i2c_driver);		// removed by mhkuo
//        ret = i2c_add_driver(&w55fa93_dac_i2c_driver);		
//		printk("w55fa93-dac-i2c ret = 0x%x \n", ret);        
//		return 0;
}

module_init(w55fa93_dac_modinit);

static void __exit w55fa93_dac_exit(void)
{
	ENTER();		
		return;
}
module_exit(w55fa93_dac_exit);



MODULE_DESCRIPTION("ASoC W55FA93_DAC codec driver");
MODULE_LICENSE("GPL");

