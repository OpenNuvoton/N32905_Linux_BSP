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

#include <linux/platform_device.h>
//#include <mach/mfp.h>
//#include <mach/map.h>
//#include <mach/regs-clock.h>

//#include <arch/arm/mach-w55fa93>

#include <mach/w55fa93_audio.h>
#include <mach/w55fa93_i2s.h>
#include <mach/w55fa93_spu.h>
#include <mach/w55fa93_reg.h>


//#include "nuc900-audio.h"


//#define SPU_DEBUG
#define SPU_DEBUG_ENTER_LEAVE
#define SPU_DEBUG_MSG
#define SPU_DEBUG_MSG2

#ifdef SPU_DEBUG
#define DBG(fmt, arg...)			printk(fmt, ##arg)
#else
#define DBG(fmt, arg...)
#endif


#ifdef SPU_DEBUG_ENTER_LEAVE
#define ENTER()					DBG("[%-10s] : Enter\n", __FUNCTION__)
#define LEAVE()					DBG("[%-10s] : Leave\n", __FUNCTION__)
#else
#define ENTER()
#define LEAVE()
#endif

#ifdef SPU_DEBUG_MSG
#define MSG(fmt)					DBG("[%-10s] : "fmt, __FUNCTION__)
#else
#define MSG(fmt)
#endif

#ifdef SPU_DEBUG_MSG2
#define MSG2(fmt, arg...)			DBG("[%-10s] : "fmt, __FUNCTION__, ##arg)
#else
#define MSG2(fmt, arg...)
#endif

//static int s_rightVol = 0, s_leftVol = 0;
static int s_spuInit = 0;

static int _bApuVolumeActive = 0;
int _bSpuActive = 0;
int _u8Channel0 = 0, _u8Channel1 = 1;
	
static volatile int _bPlayDmaToggle, _bRecDmaToggle;

int DrvSPU_SetBaseAddress(u32 u32Channel, u32 u32Address);
int DrvSPU_GetBaseAddress(u32 u32Channel);
int DrvSPU_SetThresholdAddress(u32 u32Channel, u32 u32Address);
int DrvSPU_GetThresholdAddress(u32 u32Channel);
int DrvSPU_SetEndAddress(u32 u32Channel, u32 u32Address);
int DrvSPU_GetEndAddress(u32 u32Channel);
int DrvSPU_GetCurrentAddress(u32 u32Channel);

int DrvSPU_SetDFA(u32 u32Channel, u16 u16DFA);
int DrvSPU_GetDFA(u32 u32Channel);
int DrvSPU_SetPAN(u32 u32Channel, u16 u16PAN);	// MSB 8-bit = left channel; LSB 8-bit = right channel
int DrvSPU_SetPauseAddress(u32 u32Channel, u32 u32Address);
int DrvSPU_GetPAN(u32 u32Channel);
int DrvSPU_SetSrcType(u32 u32Channel, u8 u8DataFormat);
int DrvSPU_SetChannelVolume(u32 u32Channel, u8 u8Volume);

static int DrvSPU_EnableInt(u32 u32Channel, u32 u32InterruptFlag);
static int DrvSPU_ChannelOpen(u32 u32Channel);
static int DrvSPU_ChannelClose(u32 u32Channel);
static int DrvSPU_DisableInt(u32 u32Channel, u32 u32InterruptFlag);
static int DrvSPU_ClearInt(u32 u32Channel, u32 u32InterruptFlag);

static int spuInit(void);
static int spuStartPlay(int nChannels);
static void spuStopPlay(void);
static void spuSetPlaySampleRate(int nSamplingRate);


// PLL clock settings
extern unsigned int w55fa93_apll_clock;
extern unsigned int w55fa93_upll_clock;
extern int w55fa93_set_apll_clock(unsigned int clock);

#ifdef CONFIG_SPU_WITH_I2S_OUTPUT 

//#define OPT_I2S_KEEP
	extern volatile int g_keep_i2s;
#endif

// Linux 2.6.35
static DEFINE_MUTEX(spu_mutex);
struct w55fa93_audio *w55fa93_spu_data;

//===========================================================
static int w55fa93_spu_set_fmt(struct snd_soc_dai *cpu_dai,
                              unsigned int fmt)
{
		ENTER();

	#ifdef CONFIG_SPU_WITH_I2S_OUTPUT 	
        AUDIO_WRITE(REG_I2S_ACTL_I2SCON, AUDIO_READ(REG_I2S_ACTL_I2SCON) & ~MSB_Justified);
	#endif        
	
//		struct w55fa93_audio *w55fa93_audio = w55fa93_spu_data;
//     	unsigned long val = 0;
        return 0;
}

static int w55fa93_spu_set_sysclk(struct snd_soc_dai *cpu_dai,
                                 int clk_id, unsigned int freq, int dir)
{
//        unsigned int val;
        struct w55fa93_audio *w55fa93_audio = w55fa93_spu_data;

		ENTER();

	    clk_enable(w55fa93_audio->eng_clk);        
    	clk_enable(w55fa93_audio->spu_clk);        

		spuSetPlaySampleRate(freq);
		LEAVE();		
        return 0;
}

static int w55fa93_spu_trigger(struct snd_pcm_substream *substream,
                              int cmd, struct snd_soc_dai *dai)
{
        int ret = 0;
		ENTER();        
        
        switch (cmd) {
        case SNDRV_PCM_TRIGGER_START:
        	DBG("SPU trigger start !!! \n");
				if (spuStartPlay(substream->runtime->channels))
                	ret = -EINVAL;					
        
        case SNDRV_PCM_TRIGGER_RESUME:
        	DBG("SPU trigger resume !!! \n");        
				AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) | SPU_EN);
			DBG("REG_SPU_DAC_VOL = 0x%x  !!!\n", AUDIO_READ(REG_SPU_DAC_VOL));						
			
	#ifdef CONFIG_SPU_WITH_I2S_OUTPUT 
		#ifdef OPT_I2S_KEEP            
			if (g_keep_i2s)
			{
	            AUDIO_WRITE(REG_I2S_ACTL_RESET, AUDIO_READ(REG_I2S_ACTL_RESET) & ~I2S_PLAY);            
	            AUDIO_WRITE(REG_I2S_ACTL_CON, AUDIO_READ(REG_I2S_ACTL_CON) & ~I2S_EN);            
			}	            
		#endif

            AUDIO_WRITE(REG_I2S_ACTL_CON, AUDIO_READ(REG_I2S_ACTL_CON) | I2S_EN);
            AUDIO_WRITE(REG_I2S_ACTL_RESET, AUDIO_READ(REG_I2S_ACTL_RESET) | I2S_PLAY);            
            
	#endif            
                break;
                
        case SNDRV_PCM_TRIGGER_STOP:
        	DBG("SPU trigger stop !!! \n");                
				spuStopPlay(); 

        case SNDRV_PCM_TRIGGER_SUSPEND:
        	DBG("SPU trigger suspend !!! \n");                
			AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) & ~SPU_EN);        

	#ifdef CONFIG_SPU_WITH_I2S_OUTPUT
	
		#ifdef OPT_I2S_KEEP
			if (!g_keep_i2s)
			{
	            AUDIO_WRITE(REG_I2S_ACTL_RESET, AUDIO_READ(REG_I2S_ACTL_RESET) & ~I2S_PLAY);            
	            AUDIO_WRITE(REG_I2S_ACTL_CON, AUDIO_READ(REG_I2S_ACTL_CON) & ~I2S_EN);            
			}										
		#else
            AUDIO_WRITE(REG_I2S_ACTL_RESET, AUDIO_READ(REG_I2S_ACTL_RESET) & ~I2S_PLAY);            
            AUDIO_WRITE(REG_I2S_ACTL_CON, AUDIO_READ(REG_I2S_ACTL_CON) & ~I2S_EN);            
		#endif            		
		
	#endif            
	            break;
        default:
                ret = -EINVAL;
        }

		LEAVE();        
        return ret;
}

static int w55fa93_spu_probe(struct platform_device *pdev,
                            struct snd_soc_dai *dai)
{
        struct w55fa93_audio *w55fa93_audio = w55fa93_spu_data;

		ENTER();
        mutex_lock(&spu_mutex);

        /* enable unit clock */
//        clk_enable(w55fa93_audio->clk);        
        clk_enable(w55fa93_audio->eng_clk);        
        clk_enable(w55fa93_audio->spu_clk);        

		if (!s_spuInit)
		{
			s_spuInit = 1;	
			spuInit();
		}					

	#ifdef CONFIG_SPU_WITH_I2S_OUTPUT 
		// enable I2S pins
		AUDIO_WRITE(REG_GPBFUN, (AUDIO_READ(REG_GPBFUN) & (~0x3FF0)) | 0x1550);	// GPB[6:2] to be I2S signals
		AUDIO_WRITE(REG_MISFUN, AUDIO_READ(REG_MISFUN) & (~0x01));				// I2S interface for I2S, but not SPU

		// set Play & Record interrupt encountered in half of DMA buffer length
		AUDIO_WRITE(REG_I2S_ACTL_CON, (AUDIO_READ(REG_I2S_ACTL_CON) & (~R_DMA_IRQ_SEL)) | (0x01 << 14)); 	
		AUDIO_WRITE(REG_I2S_ACTL_CON, (AUDIO_READ(REG_I2S_ACTL_CON) & (~P_DMA_IRQ_SEL)) | (0x01 << 12)); 		

		// enable I2S engine clock		
        clk_enable(w55fa93_audio->i2s_clk);        		
	#endif

        mutex_unlock(&spu_mutex);
		LEAVE();        
        return 0;
}

static void w55fa93_spu_remove(struct platform_device *pdev,
                              struct snd_soc_dai *dai)
{
        struct w55fa93_audio *w55fa93_audio = w55fa93_spu_data;

	ENTER();
	
		spuStopPlay();         
	LEAVE();		
//        clk_disable(w55fa93_audio->clk);

#ifdef CONFIG_SPU_WITH_I2S_OUTPUT 

        clk_disable(w55fa93_audio->i2s_clk);
#endif
        clk_disable(w55fa93_audio->spu_clk);
        clk_disable(w55fa93_audio->eng_clk);
}

static struct snd_soc_dai_ops w55fa93_spu_dai_ops = {
        .trigger	= w55fa93_spu_trigger,
        .set_fmt	= w55fa93_spu_set_fmt,
        .set_sysclk	= w55fa93_spu_set_sysclk,
};

struct snd_soc_dai w55fa93_spu_dai = {
        .name			= "w55fa93-spu",
        .id 			= 0,
        .probe			= w55fa93_spu_probe,
        .remove			= w55fa93_spu_remove,
        .playback = {
                .rates		= SNDRV_PCM_RATE_8000_48000,
                .formats	= SNDRV_PCM_FMTBIT_S16_LE,
                .channels_min	= 1,
                .channels_max	= 2,
        },
        .capture = {
                .rates		= SNDRV_PCM_RATE_8000_48000,
                .formats	= SNDRV_PCM_FMTBIT_S16_LE,
                .channels_min	= 1,
                .channels_max	= 2,
        },
        .ops = &w55fa93_spu_dai_ops,
};
EXPORT_SYMBOL_GPL(w55fa93_spu_dai);

static int __devinit w55fa93_spu_drvprobe(struct platform_device *pdev)
{
        struct w55fa93_audio *w55fa93_audio;
        int ret;

	ENTER();
	
        if (w55fa93_spu_data)
                return -EBUSY;

        w55fa93_audio = kzalloc(sizeof(struct w55fa93_audio), GFP_KERNEL);
        if (!w55fa93_audio)
                return -ENOMEM;

        spin_lock_init(&w55fa93_audio->lock);
		spin_lock_init(&w55fa93_audio->irqlock);		
        
        w55fa93_audio->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (!w55fa93_audio->res) {
                ret = -ENODEV;
                goto out0;
        }

        if (!request_mem_region(w55fa93_audio->res->start,
                                resource_size(w55fa93_audio->res), pdev->name)) {
                ret = -EBUSY;
                goto out0;
        }

        w55fa93_audio->mmio = ioremap(w55fa93_audio->res->start,
                                     resource_size(w55fa93_audio->res));
        if (!w55fa93_audio->mmio) {
                ret = -ENOMEM;
                goto out1;
        }

//        w55fa93_audio->clk = clk_get(&pdev->dev, NULL);
//        if (IS_ERR(w55fa93_audio->clk)) {
//                ret = PTR_ERR(w55fa93_audio->clk);
//                goto out2;
//        }


        w55fa93_audio->spu_clk = clk_get(NULL, "SPU");
        if (IS_ERR(w55fa93_audio->spu_clk)) {
                ret = PTR_ERR(w55fa93_audio->spu_clk);
                goto out2;
        }

        w55fa93_audio->eng_clk = clk_get(NULL, "ADO_ENGINE");
        if (IS_ERR(w55fa93_audio->eng_clk)) {
                ret = PTR_ERR(w55fa93_audio->eng_clk);
                goto out2;
        }

	#ifdef CONFIG_SPU_WITH_I2S_OUTPUT 
        w55fa93_audio->i2s_clk = clk_get(NULL, "I2S");
        if (IS_ERR(w55fa93_audio->i2s_clk)) {
                ret = PTR_ERR(w55fa93_audio->i2s_clk);
                goto out2;
        }
	#endif


//        w55fa93_audio->irq_num = platform_get_irq(pdev, 0);
        w55fa93_audio->irq_num = IRQ_SPU;
        if (!w55fa93_audio->irq_num) {
                ret = -EBUSY;
                goto out2;
        }
		
		ret = w55fa93_dma_create(w55fa93_audio);
		if (ret != 0)
			return ret;
		
        w55fa93_spu_data = w55fa93_audio;

        w55fa93_audio->dev = w55fa93_spu_dai.dev =  &pdev->dev;

        ret = snd_soc_register_dai(&w55fa93_spu_dai);
        
        if (ret)
                goto out3;

 //       mfp_set_groupg(w55fa93_audio->dev); /* enbale spu multifunction pin*/

	LEAVE();
        return 0;

out3:
//        clk_put(w55fa93_audio->clk);
out2:
        iounmap(w55fa93_audio->mmio);
out1:
        release_mem_region(w55fa93_audio->res->start,
                           resource_size(w55fa93_audio->res));
out0:
        kfree(w55fa93_audio);

	LEAVE();        
        return ret;
}

static int __devexit w55fa93_spu_drvremove(struct platform_device *pdev)
{
	ENTER();	
	
		w55fa93_dma_destroy(w55fa93_spu_data);
		
        snd_soc_unregister_dai(&w55fa93_spu_dai);

//        clk_put(w55fa93_spu_data->clk);
        iounmap(w55fa93_spu_data->mmio);
        release_mem_region(w55fa93_spu_data->res->start,
                           resource_size(w55fa93_spu_data->res));

        w55fa93_spu_data = NULL;
	LEAVE();
        return 0;
}

static struct platform_driver w55fa93_spu_driver = {
        .driver	= {
              .name	= "w55fa93-audio-spu",
//                .name	= "w55fa93-audio",
                .owner	= THIS_MODULE,
        },
        .probe		= w55fa93_spu_drvprobe,
        .remove		= __devexit_p(w55fa93_spu_drvremove),
                     };

static int __init w55fa93_spu_init(void)
{
        return platform_driver_register(&w55fa93_spu_driver);
}

static void __exit w55fa93_spu_exit(void)
{
	ENTER();
		
        platform_driver_unregister(&w55fa93_spu_driver);
	LEAVE();        
}


int DrvSPU_SetBaseAddress(
	u32 u32Channel, 
	u32 u32Address
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		AUDIO_WRITE(REG_SPU_S_ADDR, u32Address);		

		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_UPDATE_ALL_SETTINGS);				
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
		return E_SUCCESS;
	}
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}

int DrvSPU_GetCurrentAddress(
	u32 u32Channel
)
{
//	ENTER();	
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
//	LEAVE();		
		return AUDIO_READ(REG_SPU_CUR_ADDR);
	}

	else
	{
		LEAVE();					
		return 0;	   
	}		
		 
}

int DrvSPU_GetBaseAddress(
	u32 u32Channel
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		return (AUDIO_READ(REG_SPU_S_ADDR));
	}
	else
		return 0;
}

int DrvSPU_SetThresholdAddress(
	u32 u32Channel, 
	u32 u32Address
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		AUDIO_WRITE(REG_SPU_M_ADDR, u32Address);		

		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_UPDATE_ALL_SETTINGS);				
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
		return E_SUCCESS;
	}
	 
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}

int DrvSPU_GetThresholdAddress(
	u32 u32Channel
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		return AUDIO_READ(REG_SPU_M_ADDR);
	}
	else
		return 0;
}

int DrvSPU_SetEndAddress(
	u32 u32Channel, 
	u32 u32Address
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		AUDIO_WRITE(REG_SPU_E_ADDR, u32Address);		

		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_UPDATE_ALL_SETTINGS);				
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
		return E_SUCCESS;
	}
	 
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}

int DrvSPU_GetEndAddress(
	u32 u32Channel
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		return AUDIO_READ(REG_SPU_E_ADDR);
	}
	else
		return 0;
}



int DrvSPU_SetPauseAddress(
	u32 u32Channel, 
	u32 u32Address
)
{
	
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
//		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
//		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		AUDIO_WRITE(REG_SPU_PA_ADDR, u32Address);		

		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~0xFF) | DRVSPU_UPDATE_PAUSE_PARTIAL);				
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
#if 1
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | ((u32Channel+1) << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~0xFF) | DRVSPU_UPDATE_PAUSE_PARTIAL);				
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
#endif 		
		
		return E_SUCCESS;
	}
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}


int DrvSPU_GetPauseAddress(
	u32 u32Channel
)
{
	
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		return AUDIO_READ(REG_SPU_PA_ADDR);		
	}
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}

#if 0 
	static int DrvSPU_GetLoopStartAddress(
		u32 u32Channel, 
		u32 u32Address
	)
	{
		if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
		{
			// wait to finish previous channel settings
			while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
			
			// load previous channel settings		
			AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
			AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
			while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
			
			return AUDIO_READ(REG_SPU_LP_ADDR);
		}
		else return 0;	   
	}
#endif

int DrvSPU_SetDFA(u32 u32Channel, u16 u16DFA)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		AUDIO_WRITE(REG_SPU_CH_PAR_2, u16DFA);		

		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) & ~DRVSPU_UPDATE_ALL_PARTIALS);		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | (DRVSPU_UPDATE_DFA_PARTIAL + DRVSPU_UPDATE_PARTIAL_SETTINGS));				

		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
		return E_SUCCESS;
	}
	 
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}


int DrvSPU_GetDFA(u32 u32Channel)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		return AUDIO_READ(REG_SPU_CH_PAR_2) & 0x1FFF;
	}
	else
		return 0;
}

// MSB 8-bit = left channel; LSB 8-bit = right channel
//static int DrvSPU_SetPAN(
int DrvSPU_SetPAN(u32 u32Channel, u16 u16PAN)
{
	u32 u32PAN;
	
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		u32PAN = u16PAN;
		u32PAN <<= 8;			
		u32PAN &= (PAN_L + PAN_R);
		AUDIO_WRITE(REG_SPU_CH_PAR_1, (AUDIO_READ(REG_SPU_CH_PAR_1) & (~(PAN_L+PAN_R))) | u32PAN);		

		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) & ~DRVSPU_UPDATE_ALL_PARTIALS);		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | (DRVSPU_UPDATE_PAN_PARTIAL + DRVSPU_UPDATE_PARTIAL_SETTINGS));				

		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
		return E_SUCCESS;
	}
	 
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}

int DrvSPU_GetPAN(u32 u32Channel)
{
	u32 u32PAN;
	
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		u32PAN = AUDIO_READ(REG_SPU_CH_PAR_1);
		u32PAN >>= 8;
		return (u32PAN & 0xFFFF);
	}
	else
		return 0;
}


int DrvSPU_SetSrcType(
	u32 u32Channel, 
	u8  u8DataFormat
)
{

	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		AUDIO_WRITE(REG_SPU_CH_PAR_1, (AUDIO_READ(REG_SPU_CH_PAR_1) & ~SRC_TYPE) | u8DataFormat);		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_UPDATE_ALL_SETTINGS );				

		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
		return E_SUCCESS;
	}
	 
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}

#if 0
	static int DrvSPU_GetSrcType(
		u32 u32Channel
	)
	{
		u8 u8DataFormat;
		
		if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
		{
			// wait to finish previous channel settings
			while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
			
			// load previous channel settings		
			AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
			AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
			while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
			
			u8DataFormat = AUDIO_READ(REG_SPU_CH_PAR_1);
			return (u8DataFormat & 0x07);
		}
		else
			return 0;
	}
#endif	

int DrvSPU_SetChannelVolume(
	u32 u32Channel, 
	u8 	u8Volume
)
{
	u32 u32PAN;
	
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		u32PAN = u8Volume;
		u32PAN <<= 24;
		AUDIO_WRITE(REG_SPU_CH_PAR_1, (AUDIO_READ(REG_SPU_CH_PAR_1) & 0x00FFFFFF) | u32PAN);		

		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) & ~DRVSPU_UPDATE_ALL_PARTIALS);		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | (DRVSPU_UPDATE_VOL_PARTIAL + DRVSPU_UPDATE_PARTIAL_SETTINGS));				

		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
		return E_SUCCESS;
	}
	 
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}

#if 0
	static int DrvSPU_GetChannelVolume(
		u32 u32Channel
	)
	{
		u32 u32PAN;
		
		if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
		{
			// wait to finish previous channel settings
			while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
			
			// load previous channel settings		
			AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
			AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
			while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
			
			u32PAN = AUDIO_READ(REG_SPU_CH_PAR_1);
			u32PAN >>= 24;
			return (u32PAN & 0xFF);
		}
		else
			return 0;
	}
#endif

#if 0
	static void DrvSPU_EqOpen(
		E_DRVSPU_EQ_BAND eEqBand,
		E_DRVSPU_EQ_GAIN eEqGain		
	)
	{
		switch (eEqBand)
		{
			case eDRVSPU_EQBAND_DC:
				AUDIO_WRITE(REG_SPU_EQGain1, (AUDIO_READ(REG_SPU_EQGain1) & (~Gaindc)) | eEqGain <<16);
				break;
		
			case eDRVSPU_EQBAND_1:
				AUDIO_WRITE(REG_SPU_EQGain0, (AUDIO_READ(REG_SPU_EQGain0) & (~Gain01)) | eEqGain);
				break;
		
			case eDRVSPU_EQBAND_2:
				AUDIO_WRITE(REG_SPU_EQGain0, (AUDIO_READ(REG_SPU_EQGain0) & (~Gain02)) | eEqGain <<4);
				break;
	
			case eDRVSPU_EQBAND_3:
				AUDIO_WRITE(REG_SPU_EQGain0, (AUDIO_READ(REG_SPU_EQGain0) & (~Gain03)) | eEqGain <<8);
				break;
	
			case eDRVSPU_EQBAND_4:
				AUDIO_WRITE(REG_SPU_EQGain0, (AUDIO_READ(REG_SPU_EQGain0) & (~Gain04)) | eEqGain <<12);
				break;
	
			case eDRVSPU_EQBAND_5:
				AUDIO_WRITE(REG_SPU_EQGain0, (AUDIO_READ(REG_SPU_EQGain0) & (~Gain05)) | eEqGain <<16);
				break;
	
			case eDRVSPU_EQBAND_6:
				AUDIO_WRITE(REG_SPU_EQGain0, (AUDIO_READ(REG_SPU_EQGain0) & (~Gain06)) | eEqGain <<20);
				break;
	
			case eDRVSPU_EQBAND_7:
				AUDIO_WRITE(REG_SPU_EQGain0, (AUDIO_READ(REG_SPU_EQGain0) & (~Gain07)) | eEqGain <<24);
				break;
	
			case eDRVSPU_EQBAND_8:
				AUDIO_WRITE(REG_SPU_EQGain0, (AUDIO_READ(REG_SPU_EQGain0) & (~Gain08)) | eEqGain <<28);
				break;
	
			case eDRVSPU_EQBAND_9:
				AUDIO_WRITE(REG_SPU_EQGain1, (AUDIO_READ(REG_SPU_EQGain1) & (~Gain09)) | eEqGain);
				break;
	
			default:
			case eDRVSPU_EQBAND_10:
				AUDIO_WRITE(REG_SPU_EQGain1, (AUDIO_READ(REG_SPU_EQGain1) & (~Gain10)) | eEqGain <<4);
				break;
		}
		
		AUDIO_WRITE(REG_SPU_DAC_PAR, AUDIO_READ(REG_SPU_DAC_PAR) | EQU_EN | ZERO_EN);
	}
#endif	

#if 0
	static void DrvSPU_EqClose(void)
	{
		AUDIO_WRITE(REG_SPU_DAC_PAR, AUDIO_READ(REG_SPU_DAC_PAR) & (~EQU_EN) & (~ZERO_EN));
	}
#endif

#if 0
static void DrvSPU_SetVolume(
	u16 u16Volume	// MSB: left channel; LSB right channel
)	
{
	AUDIO_WRITE(REG_SPU_DAC_VOL, (AUDIO_READ(REG_SPU_DAC_VOL) & ~(DWA_SEL | ANA_PD | LHPVL | RHPVL)) | (u16Volume & 0x3F3F));
}
#endif

static int DrvSPU_EnableInt(
	u32 u32Channel, 
	u32 u32InterruptFlag 
)
{
	
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);

		MSG2("*** DrvSPU_EnableInt *** \n");
		MSG2("*** download channel data *** \n");	
		MSG2("==>REG_SPU_CH_CTRL=0x%x\n", AUDIO_READ(REG_SPU_CH_CTRL));
		MSG2("==>REG_SPU_CH_EVENT=0x%x\n", AUDIO_READ(REG_SPU_CH_EVENT));		
		
		// set new channel settings for previous channel settings						
		if (u32InterruptFlag & DRVSPU_USER_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) | EV_USR_EN);		
		}
		if (u32InterruptFlag & DRVSPU_SILENT_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) | EV_SLN_EN);				
		}
		if (u32InterruptFlag & DRVSPU_LOOPSTART_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) | EV_LP_EN);						
		}
		if (u32InterruptFlag & DRVSPU_END_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) | EV_END_EN);						
		}

		if (u32InterruptFlag & DRVSPU_ENDADDRESS_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) | END_EN);						
		}

		if (u32InterruptFlag & DRVSPU_THADDRESS_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) | TH_EN);														
		}
		AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) & ~AT_CLR_EN);
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) & ~DRVSPU_UPDATE_ALL_PARTIALS);		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | (DRVSPU_UPDATE_IRQ_PARTIAL + DRVSPU_UPDATE_PARTIAL_SETTINGS));				
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
	MSG2("*** upload channel data *** \n");	
	MSG2("==>REG_SPU_CH_CTRL=0x%x\n", AUDIO_READ(REG_SPU_CH_CTRL));
	MSG2("==>REG_SPU_CH_EVENT=0x%x\n", AUDIO_READ(REG_SPU_CH_EVENT));		
		
		
		return E_SUCCESS;
	}
	 
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}

static int DrvSPU_DisableInt(
	u32 u32Channel, 
	u32 u32InterruptFlag
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		MSG2("wait to finish previous channel settings  11\n"); 						
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		MSG2("load previous channel settings  11\n"); 						
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// set new channel settings for previous channel settings						
		MSG2("set new channel settings for previous channel settings 11\n");		
		if (u32InterruptFlag & DRVSPU_USER_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) & ~EV_USR_EN);		
		}
		if (u32InterruptFlag & DRVSPU_SILENT_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) & ~EV_SLN_EN);				
		}
		if (u32InterruptFlag & DRVSPU_LOOPSTART_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) & ~EV_LP_EN);						
		}
		if (u32InterruptFlag & DRVSPU_END_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) & ~EV_END_EN);						
		}
		if (u32InterruptFlag & DRVSPU_ENDADDRESS_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) & ~END_EN);						
		}
		if (u32InterruptFlag & DRVSPU_THADDRESS_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT) & ~TH_EN);
		}
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) & ~DRVSPU_UPDATE_ALL_PARTIALS);		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | (DRVSPU_UPDATE_IRQ_PARTIAL + DRVSPU_UPDATE_PARTIAL_SETTINGS));				
		
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
	
		return E_SUCCESS;
	}
	 
	else
		return E_DRVSPU_WRONG_CHANNEL;	   
}


static int DrvSPU_ClearInt(
	u32 u32Channel, 
	u32 u32InterruptFlag
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		// wait to finish previous channel settings
		MSG2("wait to finish previous channel settings\n"); 				
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
		
		// load previous channel settings		
		MSG2("load previous channel settings\n"); 				
		AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (u32Channel << 24));		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);

		MSG2("*** DrvSPU_ClearInt *** \n");
		MSG2("*** download channel data *** \n");	
		MSG2("==>REG_SPU_CH_CTRL=0x%x\n", AUDIO_READ(REG_SPU_CH_CTRL));
		MSG2("==>REG_SPU_CH_EVENT=0x%x\n", AUDIO_READ(REG_SPU_CH_EVENT));		
		
		// set new channel settings for previous channel settings
		MSG2("set new channel settings for previous channel settings\n");
		if (u32InterruptFlag & DRVSPU_USER_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, (AUDIO_READ(REG_SPU_CH_EVENT) & ~0x3F00) | EV_USR_FG);		
		}
		if (u32InterruptFlag & DRVSPU_SILENT_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, (AUDIO_READ(REG_SPU_CH_EVENT) & ~0x3F00) | EV_SLN_FG);				
		}
		if (u32InterruptFlag & DRVSPU_LOOPSTART_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, (AUDIO_READ(REG_SPU_CH_EVENT) & ~0x3F00) | EV_LP_FG);						
		}
		if (u32InterruptFlag & DRVSPU_END_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, (AUDIO_READ(REG_SPU_CH_EVENT) & ~0x3F00) | EV_END_FG);						
		}
		if (u32InterruptFlag & DRVSPU_ENDADDRESS_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, (AUDIO_READ(REG_SPU_CH_EVENT) & ~0x3F00) | END_FG);						
		}
		if (u32InterruptFlag & DRVSPU_THADDRESS_INT)
		{
			AUDIO_WRITE(REG_SPU_CH_EVENT, (AUDIO_READ(REG_SPU_CH_EVENT) & ~0x3F00) | TH_FG);														
		}
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) & ~DRVSPU_UPDATE_ALL_PARTIALS);		
		AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | (DRVSPU_UPDATE_IRQ_PARTIAL + DRVSPU_UPDATE_PARTIAL_SETTINGS));				
		
		MSG2("wait to finish previous channel settings 00\n"); 						
		while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
		
		MSG2("wait to finish previous channel settings OK\n"); 								
	
		MSG2("*** upload channel data *** \n");	
		MSG2("==>REG_SPU_CH_CTRL=0x%x\n", AUDIO_READ(REG_SPU_CH_CTRL));
		MSG2("==>REG_SPU_CH_EVENT=0x%x\n", AUDIO_READ(REG_SPU_CH_EVENT));		
	
		return E_SUCCESS;
	}
	else
	{
		MSG2("WORNG CHANNEL\n"); 									
		return E_DRVSPU_WRONG_CHANNEL;	   
	}		
		
}

static int DrvSPU_ChannelOpen(
	u32 u32Channel
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		AUDIO_WRITE(REG_SPU_CH_EN, AUDIO_READ(REG_SPU_CH_EN) | (0x0001 << u32Channel));
		return E_SUCCESS;
	}
	else		
		return E_DRVSPU_WRONG_CHANNEL;	   	
}

static int DrvSPU_ChannelClose(
	u32 u32Channel
)
{
	if ( (u32Channel >=eDRVSPU_CHANNEL_0) && (u32Channel <=eDRVSPU_CHANNEL_31) )
	{
		AUDIO_WRITE(REG_SPU_CH_EN, AUDIO_READ(REG_SPU_CH_EN) & ~(0x0001 << u32Channel));
		return E_SUCCESS;
	}		
	else		
		return E_DRVSPU_WRONG_CHANNEL;	   	
}

static int spuInit(void)
{
	int ii;

		ENTER();

		MSG2("init SPU register BEGIN !!\n");
			
		// disable SPU engine 
		AUDIO_WRITE(REG_SPU_CTRL, 0x00);
		
		// given FIFO size = 4
		AUDIO_WRITE(REG_SPU_CTRL, 0x04000000);		
	
		// reset SPU engine 
	//	AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) | SPU_EN);
		AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) & ~SPU_SWRST);	
		
		AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) | SPU_SWRST);
		AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) & ~SPU_SWRST);	
		
		// disable all channels
		AUDIO_WRITE(REG_SPU_CH_EN, 0x00);		

		for (ii=0; ii<32; ii++)
		{
			DrvSPU_ClearInt(ii, DRVSPU_ALL_INT);
			DrvSPU_DisableInt(ii, DRVSPU_ALL_INT);
		}
		
		LEAVE();
	
	return 0;	
}

#if 0
	static int spuSetPcmVolume(int ucLeftVol, int ucRightVol)
	{
		ENTER();
		MSG2("Set PCM volume to : %d-%d\n", ucLeftVol, ucRightVol);
		
		//save the flag that ap already sets the volume
		_bApuVolumeActive = 1;
	
	#if defined(USE_DAC_ON_OFF_API)
		AUDIO_WRITE(REG_SPU_DAC_VOL, (AUDIO_READ(REG_SPU_DAC_VOL) & ~(/*ANA_PD | */LHPVL | RHPVL)) | (ucRightVol & 0x3F) | (ucLeftVol & 0x3F) << 8);
	#else
		AUDIO_WRITE(REG_SPU_DAC_VOL, (AUDIO_READ(REG_SPU_DAC_VOL) & ~(ANA_PD | LHPVL | RHPVL)) | (ucRightVol & 0x3F) | (ucLeftVol & 0x3F) << 8);
	#endif		
		LEAVE();
		
		return 0;
	}
#endif

static int spuStartPlay(int nChannels)
{
	ENTER();

	if (_bSpuActive & SPU_PLAY_ACTIVE)
		return -1;		

	
	/* set default pcm volume */
//	if(!_bApuVolumeActive)
//		spuSetPcmVolume(15,15);
		
	/* start playing */
	MSG("SPU start playing...\n");
		
#ifdef CONFIG_GPA7_FOR_SPEAKER_ENABLED
	AUDIO_WRITE(REG_GPAFUN, AUDIO_READ(REG_GPAFUN) & ~MF_GPA7);		
	AUDIO_WRITE(REG_GPIOA_OMD, AUDIO_READ(REG_GPIOA_OMD) | BIT7);		
	AUDIO_WRITE(REG_GPIOA_DOUT, AUDIO_READ(REG_GPIOA_DOUT) | BIT7);		
#endif
	_bPlayDmaToggle = 0;
	
	if (nChannels ==1)
	{
		DrvSPU_ChannelOpen(_u8Channel0);		
		DrvSPU_ChannelOpen(_u8Channel1);
	}
	else
	{	
		DrvSPU_ChannelOpen(_u8Channel0);	//left channel 
		DrvSPU_ChannelOpen(_u8Channel1);	// right channel
	}				

	/* set DFA */
//	#ifdef CONFIG_I2S_MCLK_256WS || CONFIG_SPU_WITH_I2S_OUTPUT
	#ifdef CONFIG_SPU_WITH_I2S_OUTPUT
		DrvSPU_SetDFA(_u8Channel0, 0x200);				
		DrvSPU_SetDFA(_u8Channel1, 0x200);			
       	printk("set DFA = 0x200 !!!!\n");	    								
	#else
		DrvSPU_SetDFA(_u8Channel0, 0x400);					
		DrvSPU_SetDFA(_u8Channel1, 0x400);			
	#endif		

	/* enable interrupt */
	DrvSPU_ClearInt(_u8Channel0, DRVSPU_ALL_INT);
	DrvSPU_DisableInt(_u8Channel0, DRVSPU_ALL_INT);		
	DrvSPU_ClearInt(_u8Channel1, DRVSPU_ALL_INT);
	DrvSPU_DisableInt(_u8Channel1, DRVSPU_ALL_INT);		
	
	
	if (nChannels ==1)
	{
		DrvSPU_EnableInt(_u8Channel0, DRVSPU_ENDADDRESS_INT);
		DrvSPU_EnableInt(_u8Channel0, DRVSPU_THADDRESS_INT);		
	}
	else
	{	/* just open one channel interrupt */
		DrvSPU_EnableInt(_u8Channel0, DRVSPU_ENDADDRESS_INT);
		DrvSPU_EnableInt(_u8Channel0, DRVSPU_THADDRESS_INT);		
	}				

	AUDIO_WRITE(REG_SPU_CH_IRQ,AUDIO_READ(REG_SPU_CH_IRQ));		
	
	_bSpuActive |= SPU_PLAY_ACTIVE;
	
	AUDIO_WRITE(REG_AIC_MECR,(1<<IRQ_SPU));
	

	LEAVE();
	return 0;
}

static void spuStopPlay(void)
{
	int volume;
	
	ENTER();
	
	if (!(_bSpuActive & SPU_PLAY_ACTIVE))
		return;
		
	if(_bApuVolumeActive)	//save the volume before reset audio engine
		volume = AUDIO_READ(REG_SPU_DAC_VOL) & (LHPVL | RHPVL);		

	/* channel close (before SPU disabled) */
	DrvSPU_ChannelClose(_u8Channel0);	//left channel 
	DrvSPU_ChannelClose(_u8Channel1);	// right channel
			
	/* disable audio play interrupt */
	if (!_bSpuActive)
 	{		
		AUDIO_WRITE(REG_AIC_MDCR,(1<<IRQ_SPU));
		AUDIO_WRITE(REG_AIC_MECR,(1<<IRQ_SPU));		
	}
	
	/* disable SPU (after channel being closed) */
	AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) & ~SPU_EN);	/*disable spu*/	
	
	/* reset SPU engine */
	AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) & ~SPU_SWRST);	
	AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) | SPU_SWRST);
	AUDIO_WRITE(REG_SPU_CTRL, AUDIO_READ(REG_SPU_CTRL) & ~SPU_SWRST);	

	//restore volume
	if(_bApuVolumeActive)	// if ever set the volume then restore it 
		AUDIO_WRITE(REG_SPU_DAC_VOL, (AUDIO_READ(REG_SPU_DAC_VOL) | volume));

	_bSpuActive &= ~SPU_PLAY_ACTIVE;       

#ifdef CONFIG_GPA7_FOR_SPEAKER_ENABLED
	AUDIO_WRITE(REG_GPIOA_DOUT, AUDIO_READ(REG_GPIOA_DOUT) & ~BIT7);		
#endif
	
	LEAVE();
}

static void  spuSetPlaySampleRate(int nSamplingRate)
{

//#define OPT_SPU_FROM_UPLL
#ifdef OPT_SPU_FROM_UPLL
	unsigned int PllFreq = w55fa93_upll_clock;
#else	
	unsigned int PllFreq = w55fa93_apll_clock;
#endif	
	
	int u32ClockDivider;
	
	ENTER();
	
	#if !defined(CONFIG_W55FA93_TV_FROM_APLL)
		#ifdef CONFIG_SPU_WITH_I2S_OUTPUT 
	//	#ifdef CONFIG_I2S_MCLK_256WS || CONFIG_SPU_WITH_I2S_OUTPUT 		// "CONFIG_SPU_WITH_I2S_OUTPUT" must TV clock not from APLL
			if ( (nSamplingRate == AU_SAMPLE_RATE_48000) ||
				 (nSamplingRate == AU_SAMPLE_RATE_24000) ||
				 (nSamplingRate == AU_SAMPLE_RATE_12000) ||
				 (nSamplingRate == AU_SAMPLE_RATE_20000) )		 	// added
			{
				w55fa93_set_apll_clock(208896);
				
			}
			else if ( (nSamplingRate == AU_SAMPLE_RATE_192000) ||
				 	  (nSamplingRate == AU_SAMPLE_RATE_96000) ||		
				 	  (nSamplingRate == AU_SAMPLE_RATE_64000) ||						 	  				 	  
				 	  (nSamplingRate == AU_SAMPLE_RATE_32000) )						 	  
			{
				w55fa93_set_apll_clock(147456);
			}
			else if ( (nSamplingRate == AU_SAMPLE_RATE_16000) ||
				 	  (nSamplingRate == AU_SAMPLE_RATE_8000) )		
			{
				w55fa93_set_apll_clock(184320);
			}
			else if ( (nSamplingRate == AU_SAMPLE_RATE_44100) ||
				 	  (nSamplingRate == AU_SAMPLE_RATE_22050) ||
				 	  (nSamplingRate == AU_SAMPLE_RATE_11025) )		
			{
				w55fa93_set_apll_clock(169344);
			}
			else
				w55fa93_set_apll_clock(135475);			// 88.2KHz	
		
			PllFreq = w55fa93_apll_clock;
			PllFreq *= 1000;
				
			MSG2("==>PllFreq=0x%x\n", PllFreq);
			MSG2("==>nSamplingRate=0x%x\n", nSamplingRate);	

			u32ClockDivider = (PllFreq / (256*nSamplingRate));
			MSG2("==>u32ClockDivider=0x%x\n", u32ClockDivider);			
		
			AUDIO_WRITE(REG_CLKDIV1, (AUDIO_READ(REG_CLKDIV1) & (~ADO_S)) | (0x02 << 19) );	// SPU clock from APLL	
			AUDIO_WRITE(REG_CLKDIV1, AUDIO_READ(REG_CLKDIV1) & (~ADO_N0));		
			
//			AUDIO_WRITE(REG_I2S_ACTL_I2SCON, 0x0000);				
			AUDIO_WRITE(REG_I2S_ACTL_I2SCON, AUDIO_READ(REG_I2S_ACTL_I2SCON) & 0x08);			
			MSG2("==>REG_I2S_ACTL_I2SCON = 0x%x\n", AUDIO_READ(REG_I2S_ACTL_I2SCON));			
							
		#else
			if ( (nSamplingRate == AU_SAMPLE_RATE_48000) ||
				 (nSamplingRate == AU_SAMPLE_RATE_32000) ||
				 (nSamplingRate == AU_SAMPLE_RATE_24000) ||
				 (nSamplingRate == AU_SAMPLE_RATE_12000) ||
				 (nSamplingRate == AU_SAMPLE_RATE_20000) )		 	// added
			{
				w55fa93_set_apll_clock(208896);
				
			}
			else if ( (nSamplingRate == AU_SAMPLE_RATE_64000) ||
				 (nSamplingRate == AU_SAMPLE_RATE_96000) )
			{
				w55fa93_set_apll_clock(147456);
			}
			else if ( (nSamplingRate == AU_SAMPLE_RATE_16000) ||
				 	  (nSamplingRate == AU_SAMPLE_RATE_8000) )		
			{
				w55fa93_set_apll_clock(184320);  
			}
			else
				w55fa93_set_apll_clock(169344);		
		
			PllFreq = w55fa93_apll_clock;
			PllFreq *= 1000;
				
			MSG2("==>PllFreq=0x%x\n", PllFreq);
			MSG2("==>nSamplingRate=0x%x\n", nSamplingRate);	
		
			u32ClockDivider = (PllFreq / (128*nSamplingRate));
			MSG2("==>u32ClockDivider=0x%x\n", u32ClockDivider);			
		
			AUDIO_WRITE(REG_CLKDIV1, (AUDIO_READ(REG_CLKDIV1) & (~ADO_S)) | (0x02 << 19) );	// SPU clock from APLL	
			AUDIO_WRITE(REG_CLKDIV1, AUDIO_READ(REG_CLKDIV1) & (~ADO_N0));			
		#endif	// CONFIG_I2S_MCLK_256WS || CONFIG_SPU_WITH_I2S_OUTPUT
	#else	
		//	PllFreq *= 1000000;
			PllFreq *= 1000;
				
			MSG2("==>PllFreq=0x%x\n", PllFreq);
			MSG2("==>nSamplingRate=0x%x\n", nSamplingRate);	
		
			u32ClockDivider = PllFreq / (128*nSamplingRate);
			u32Remainder = PllFreq % (128*nSamplingRate);
			MSG2("==>u32ClockDivider=0x%x\n", u32ClockDivider);			
			
			u32Remainder *= 2;
				
			if (u32Remainder >= 128*nSamplingRate)
				u32ClockDivider++;	
			
		#ifdef OPT_SPU_FROM_UPLL	
			AUDIO_WRITE(REG_CLKDIV1, (AUDIO_READ(REG_CLKDIV1) & (~ADO_S)) | (0x03 << 19) );	// SPU clock from UPLL				
			AUDIO_WRITE(REG_CLKDIV1, AUDIO_READ(REG_CLKDIV1) & (~ADO_N0));		
			AUDIO_WRITE(REG_CLKDIV1, AUDIO_READ(REG_CLKDIV1) | (0x01<<16));		
			u32ClockDivider /= 2;
		#else
			AUDIO_WRITE(REG_CLKDIV1, (AUDIO_READ(REG_CLKDIV1) & (~ADO_S)) | (0x02 << 19) );	// SPU clock from APLL				
			AUDIO_WRITE(REG_CLKDIV1, AUDIO_READ(REG_CLKDIV1) & (~ADO_N0));		
		#endif
	#endif		// CONFIG_W55FA93_TV_FROM_APLL

	AUDIO_WRITE(REG_CLKDIV1, AUDIO_READ(REG_CLKDIV1) & (~ADO_N1));		
	u32ClockDivider &= 0xFF;
	u32ClockDivider	--;
	AUDIO_WRITE(REG_CLKDIV1, AUDIO_READ(REG_CLKDIV1) | (u32ClockDivider<<24));	
	
	MSG2("==>REG_CLKDIV1=0x%x\n", AUDIO_READ(REG_CLKDIV1));			

	LEAVE();
}

void spuDacOn(void)
{
	// level-2
	AUDIO_WRITE(REG_SPU_DAC_PAR, AUDIO_READ(REG_SPU_DAC_PAR) | 0x30);		//disable
	AUDIO_WRITE(REG_SPU_DAC_PAR, AUDIO_READ(REG_SPU_DAC_PAR) & ~0x10);	//delay time, p0=2s

	AUDIO_WRITE(REG_SPU_DAC_VOL, AUDIO_READ(REG_SPU_DAC_VOL) & ~0x0800000);	//P7	
	mdelay(10);
	AUDIO_WRITE(REG_SPU_DAC_VOL, AUDIO_READ(REG_SPU_DAC_VOL) & ~0x0400000);	//P6
	mdelay(10);
	AUDIO_WRITE(REG_SPU_DAC_VOL, AUDIO_READ(REG_SPU_DAC_VOL) & ~0x01e0000);	//P1-4
	mdelay(10);
	AUDIO_WRITE(REG_SPU_DAC_VOL, AUDIO_READ(REG_SPU_DAC_VOL) & ~0x0200000);	//P5	
	mdelay(10);
	AUDIO_WRITE(REG_SPU_DAC_VOL, AUDIO_READ(REG_SPU_DAC_VOL) & ~0x00010000);	//P0			
	mdelay(300);
}

void spuDacOff(void)
{		
	mdelay(10);
	AUDIO_WRITE(REG_SPU_DAC_VOL, AUDIO_READ(REG_SPU_DAC_VOL) | 0x10000);	//P0
	mdelay(400);
	AUDIO_WRITE(REG_SPU_DAC_VOL, AUDIO_READ(REG_SPU_DAC_VOL) | 0x200000);	//P5
	mdelay(10);
	AUDIO_WRITE(REG_SPU_DAC_VOL, AUDIO_READ(REG_SPU_DAC_VOL) | 0x1e0000);	//P1-4
	mdelay(10);
	AUDIO_WRITE(REG_SPU_DAC_VOL, AUDIO_READ(REG_SPU_DAC_VOL) | 0x400000);	//P6
	mdelay(10);
	AUDIO_WRITE(REG_SPU_DAC_VOL, AUDIO_READ(REG_SPU_DAC_VOL) | 0x800000);	//P7	
	mdelay(10);
	AUDIO_WRITE(REG_SPU_DAC_PAR, AUDIO_READ(REG_SPU_DAC_PAR) | 0x30);  //disable
}

EXPORT_SYMBOL_GPL(spuDacOn);
EXPORT_SYMBOL_GPL(spuDacOff);


module_init(w55fa93_spu_init);
module_exit(w55fa93_spu_exit);

MODULE_AUTHOR("Wan ZongShun <mcuos.com@gmail.com>");
MODULE_DESCRIPTION("W55FA93 IIS SoC driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:w55fa93-spu");
