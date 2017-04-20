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
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/delay.h>

#include <mach/hardware.h>

//#include "w55fa93-audio.h"

#include <mach/w55fa93_audio.h>
#include <mach/w55fa93_spu.h>
#include <mach/w55fa93_i2s.h>
#include <mach/w55fa93_reg.h>

//#define PCM_DEBUG
#define PCM_DEBUG_ENTER_LEAVE
#define PCM_DEBUG_MSG
#define PCM_DEBUG_MSG2

#ifdef PCM_DEBUG
#define DBG(fmt, arg...)			printk(fmt, ##arg)
#else
#define DBG(fmt, arg...)
#endif


#ifdef PCM_DEBUG_ENTER_LEAVE
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
	
	static int earphone = -1;	
	int enable_earphonr_detect(void);	
#endif

#if defined (CONFIG_HEADSET_GPD3_AND_SPEAKER_GPD4)			
	#if defined(CONFIG_HEADSET_GPD3_HIGH)
	#define EARPHONE_STATE		1
	#else
	#define EARPHONE_STATE		0
	#endif
	#define SPEAKER_STATE		1	
#endif

static int s_PlayChannels = 0;

#ifdef CONFIG_SND_SOC_W55FA93_SPU
	extern int _u8Channel0, _u8Channel1;
	extern int DrvSPU_SetBaseAddress(u32 u32Channel, u32 u32Address);
	extern int DrvSPU_GetBaseAddress(u32 u32Channel);
	extern int DrvSPU_SetThresholdAddress(u32 u32Channel, u32 u32Address);
	extern int DrvSPU_GetThresholdAddress(u32 u32Channel);
	extern int DrvSPU_SetEndAddress(u32 u32Channel, u32 u32Address);
	extern int DrvSPU_GetEndAddress(u32 u32Channel);
	extern int DrvSPU_GetCurrentAddress(u32 u32Channel);
	extern int DrvSPU_SetPauseAddress(u32 u32Channel, u32 u32Address);
	
	extern int DrvSPU_SetDFA(u32 u32Channel, u16 u16DFA);
	extern int DrvSPU_GetDFA(u32 u32Channel);
	extern int DrvSPU_SetPAN(u32 u32Channel, u16 u16PAN);	// MSB 8-bit = left channel; LSB 8-bit = right channel
	extern int DrvSPU_SetPauseAddress(u32 u32Channel, u32 u32Address);
	extern int DrvSPU_GetPAN(u32 u32Channel);
	extern int DrvSPU_SetSrcType(u32 u32Channel, u8 u8DataFormat);
	extern int DrvSPU_SetChannelVolume(u32 u32Channel, u8 u8Volume);
#endif                                      

static const struct snd_pcm_hardware w55fa93_pcm_hardware = {
        .info			= SNDRV_PCM_INFO_INTERLEAVED |
        SNDRV_PCM_INFO_BLOCK_TRANSFER |
        SNDRV_PCM_INFO_MMAP |
        SNDRV_PCM_INFO_MMAP_VALID |
        SNDRV_PCM_INFO_PAUSE |
        SNDRV_PCM_INFO_RESUME,
        .formats		= SNDRV_PCM_FMTBIT_S16_LE,
        .channels_min		= 1,
        .channels_max		= 2,
        .buffer_bytes_max	= 64*1024,
        .period_bytes_min	= 1*1024,
        .period_bytes_max	= 16*1024,
        .periods_min		= 1,
        .periods_max		= 1024,
};

static int w55fa93_dma_hw_params(struct snd_pcm_substream *substream,
                                struct snd_pcm_hw_params *params)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct w55fa93_audio *w55fa93_audio = runtime->private_data;
        unsigned long flags;
        int ret = 0;

	ENTER();

        spin_lock_irqsave(&w55fa93_audio->irqlock, flags);

        if(runtime->dma_addr == 0) {
            ret = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
            if (ret < 0)
                    return ret;

        	w55fa93_audio->substream[substream->stream] = substream;        	
        }

        w55fa93_audio->dma_addr[substream->stream] = runtime->dma_addr;
        w55fa93_audio->buffersize[substream->stream] =
                params_buffer_bytes(params);

        spin_unlock_irqrestore(&w55fa93_audio->irqlock, flags);

        return ret;
}

static void w55fa93_update_dma_register(struct snd_pcm_substream *substream,
                                       dma_addr_t dma_addr, size_t count)
{

#ifdef CONFIG_SND_SOC_W55FA93_I2S

//        struct snd_pcm_runtime *runtime = substream->runtime;
//        struct w55fa93_audio *w55fa93_audio = runtime->private_data;
        void __iomem *mmio_addr, *mmio_len;
		ENTER();
		
        if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
                mmio_addr = REG_I2S_ACTL_PDSTB;
                mmio_len = REG_I2S_ACTL_PDST_LENGTH;
        } else {
                mmio_addr = REG_I2S_ACTL_RDSTB;
                mmio_len = REG_I2S_ACTL_RDST_LENGTH;
        }

        AUDIO_WRITE(mmio_addr, dma_addr);
        AUDIO_WRITE(mmio_len, count);
#endif        
        
#ifdef CONFIG_SND_SOC_W55FA93_SPU

	#ifdef CONFIG_HEADSET_ENABLED		// for headset_detect
		Disable_IRQ(HEADSET_IRQ_NUM);	
	#endif	

		//left channel 			
		DrvSPU_SetBaseAddress(_u8Channel0, dma_addr);		
		DrvSPU_SetThresholdAddress(_u8Channel0, dma_addr + count/2);		
		DrvSPU_SetEndAddress(_u8Channel0, dma_addr + count);		
		
		// right channel			
		DrvSPU_SetBaseAddress(_u8Channel1, dma_addr);
		DrvSPU_SetThresholdAddress(_u8Channel1, dma_addr + count/2);
		DrvSPU_SetEndAddress(_u8Channel1, dma_addr+ count);

	#ifdef CONFIG_HEADSET_ENABLED		// for headset_detect
		Enable_IRQ(HEADSET_IRQ_NUM);	    
	#endif

	#ifdef CONFIG_SPU_WITH_I2S_OUTPUT 			
        AUDIO_WRITE(REG_I2S_ACTL_PDSTB, dma_addr);
        AUDIO_WRITE(REG_I2S_ACTL_PDST_LENGTH, count);
	#endif		
#endif        

	LEAVE();
}

static irqreturn_t w55fa93_dma_interrupt(int irq, void *dev_id)
{
        struct w55fa93_audio *w55fa93_audio = dev_id;
		unsigned long flags;
		int stream;

#ifdef CONFIG_SND_SOC_W55FA93_SPU	
//		int bPlayLastBlock;
		u8 ii;
		u32 u32Channel, u32InterruptFlag;	
#else
		u32 val;		
#endif		
		ENTER();
		
        spin_lock_irqsave(&w55fa93_audio->irqlock, flags);

#ifdef CONFIG_SND_SOC_W55FA93_I2S

        val = AUDIO_READ(REG_I2S_ACTL_CON);
        if (val & R_DMA_IRQ) {
    		stream = SNDRV_PCM_STREAM_CAPTURE;    		

            AUDIO_WRITE(REG_I2S_ACTL_RSR, AUDIO_READ(REG_I2S_ACTL_RSR));    		
            AUDIO_WRITE(REG_I2S_ACTL_CON, val | R_DMA_IRQ);
        	snd_pcm_period_elapsed(w55fa93_audio->substream[stream]);        
		} 
		if (val & P_DMA_IRQ) {
        	stream = SNDRV_PCM_STREAM_PLAYBACK;        	
        	
            AUDIO_WRITE(REG_I2S_ACTL_PSR, AUDIO_READ(REG_I2S_ACTL_PSR));
            AUDIO_WRITE(REG_I2S_ACTL_CON, val | P_DMA_IRQ);
        	snd_pcm_period_elapsed(w55fa93_audio->substream[stream]);        
		}
		if (!(val & (P_DMA_IRQ|R_DMA_IRQ))) {
                dev_err(w55fa93_audio->dev, "Wrong DMA interrupt status!\n");                
                spin_unlock_irqrestore(&w55fa93_audio->irqlock, flags);
                return IRQ_HANDLED;
        }
        spin_unlock_irqrestore(&w55fa93_audio->irqlock, flags);
		LEAVE();
        return IRQ_HANDLED;
#endif

#ifdef CONFIG_SND_SOC_W55FA93_SPU	

       	stream = SNDRV_PCM_STREAM_PLAYBACK;        		
		u32Channel = 1;
	
		for (ii=0; ii<2; ii++)
//		for (ii=1; ii<2; ii++)
		{
			if (!(AUDIO_READ(REG_SPU_CH_EN) & u32Channel))
			{
				continue;
			}			
			if (AUDIO_READ(REG_SPU_CH_IRQ) & u32Channel)
			{
				while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);
	
				// load previous channel settings		
				AUDIO_WRITE(REG_SPU_CH_CTRL, (AUDIO_READ(REG_SPU_CH_CTRL) & ~CH_NO) | (ii << 24));		
				AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | DRVSPU_LOAD_SELECTED_CHANNEL);
				while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);

				u32InterruptFlag = AUDIO_READ(REG_SPU_CH_EVENT);
				DBG("REG_SPU_CH_EVENT = 0x%x !! \n", u32InterruptFlag);
				AUDIO_WRITE(REG_SPU_CH_EVENT, AUDIO_READ(REG_SPU_CH_EVENT));						

				/* clear int */
				if (u32InterruptFlag & DRVSPU_ENDADDRESS_INT)
				{			
					AUDIO_WRITE(REG_SPU_CH_EVENT, (AUDIO_READ(REG_SPU_CH_EVENT) & ~0x3F00) | END_FG);						
					DBG("END !! \n");	        					
				}				
	
				/* clear int */	
				if (u32InterruptFlag & DRVSPU_THADDRESS_INT)
				{
					AUDIO_WRITE(REG_SPU_CH_EVENT, (AUDIO_READ(REG_SPU_CH_EVENT) & ~0x3F00) | TH_FG);														
					DBG("MIDDLE !! \n");	        										
				}											

				AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) & ~DRVSPU_UPDATE_ALL_PARTIALS);		
				AUDIO_WRITE(REG_SPU_CH_CTRL, AUDIO_READ(REG_SPU_CH_CTRL) | (DRVSPU_UPDATE_IRQ_PARTIAL + DRVSPU_UPDATE_PARTIAL_SETTINGS));
				while(AUDIO_READ(REG_SPU_CH_CTRL) & CH_FN);		
			}
		
			u32Channel <<= 1; 
		}
		AUDIO_WRITE(REG_SPU_CH_IRQ, AUDIO_READ(REG_SPU_CH_IRQ));			
        snd_pcm_period_elapsed(w55fa93_audio->substream[stream]);        
        spin_unlock_irqrestore(&w55fa93_audio->irqlock, flags);
		LEAVE();
	
        return IRQ_HANDLED;
#endif
}

static int w55fa93_dma_hw_free(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct w55fa93_audio *w55fa93_audio = runtime->private_data;

	ENTER();
	
        snd_pcm_lib_free_pages(substream);
        w55fa93_audio->substream[substream->stream] = NULL;

	LEAVE();
        return 0;
}

static int w55fa93_dma_prepare(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct w55fa93_audio *w55fa93_audio = runtime->private_data;
        unsigned long flags;
        
#ifdef CONFIG_SND_SOC_W55FA93_I2S        
		int val;
#endif

	ENTER();
	
        spin_lock_irqsave(&w55fa93_audio->irqlock, flags);

        w55fa93_update_dma_register(substream,
                                   w55fa93_audio->dma_addr[substream->stream],
                                   w55fa93_audio->buffersize[substream->stream]);
                              
		s_PlayChannels =runtime->channels;
		
#ifdef CONFIG_SND_SOC_W55FA93_I2S

        val = AUDIO_READ(REG_I2S_ACTL_RESET);

        switch (runtime->channels) {
        case 1:
                if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
                        val &= ~PLAY_STEREO;
                } else {
                        //NOTE:
                        //if record source device (mic) is connected at left channel,
                        //user needs to modify the following flag(RECORD_RIGHT_CHANNEL -> RECORD_LEFT_CHANNEL)
                        val &= ~(RECORD_LEFT_CHANNEL | RECORD_RIGHT_CHANNEL);
                        //val |= RECORD_RIGHT_CHANNEL;
                        val |= RECORD_LEFT_CHANNEL;                       
                }
                AUDIO_WRITE(REG_I2S_ACTL_RESET, val);                
                break;
        case 2:
                if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
                        val |= PLAY_STEREO;
                else
                        val |= (RECORD_LEFT_CHANNEL | RECORD_RIGHT_CHANNEL);

                AUDIO_WRITE(REG_I2S_ACTL_RESET, val);
                break;
        default:
                return -EINVAL;
        }
#endif

#ifdef CONFIG_SND_SOC_W55FA93_SPU	

        switch (runtime->channels) 
        {
        	case 1:
				DrvSPU_SetSrcType(_u8Channel0, DRVSPU_MONO_PCM16);        
//				DrvSPU_SetChannelVolume(_u8Channel0, 0x4F);	        
				DrvSPU_SetChannelVolume(_u8Channel0, 0x7F);
				DrvSPU_SetPAN(_u8Channel0, 0x1F1F);	// MSB 8-bit = left channel; LSB 8-bit = right channel	
				DrvSPU_SetDFA(_u8Channel0, 0x400);			

				DrvSPU_SetSrcType(_u8Channel1, DRVSPU_MONO_PCM16);
				DrvSPU_SetChannelVolume(_u8Channel1, 0x0);
				DrvSPU_SetPAN(_u8Channel1, 0x0000);
				DrvSPU_SetDFA(_u8Channel1, 0x400);			
                break;
                
        	case 2:
				DrvSPU_SetSrcType(_u8Channel0, DRVSPU_STEREO_PCM16_LEFT);		// left channel 
//				DrvSPU_SetChannelVolume(_u8Channel0, 0x4F);	
				DrvSPU_SetChannelVolume(_u8Channel0, 0x7F);	
				DrvSPU_SetPAN(_u8Channel0, 0x1F00);	// MSB 8-bit = left channel; LSB 8-bit = right channel			
				DrvSPU_SetDFA(_u8Channel0, 0x400);			
        
				DrvSPU_SetSrcType(_u8Channel1, DRVSPU_STEREO_PCM16_RIGHT);		// right channel	
//				DrvSPU_SetChannelVolume(_u8Channel1, 0x4F);
				DrvSPU_SetChannelVolume(_u8Channel1, 0x7F);
				DrvSPU_SetPAN(_u8Channel1, 0x001F);	// MSB 8-bit = left channel; LSB 8-bit = right channel	
				DrvSPU_SetDFA(_u8Channel1, 0x400);			
                break;
                
        	default:
                return -EINVAL;
        }
        
	#ifdef CONFIG_SPU_WITH_I2S_OUTPUT 
        switch (runtime->channels)
        {
        	case 1:
				AUDIO_WRITE(REG_I2S_ACTL_RESET, AUDIO_READ(REG_I2S_ACTL_RESET) & ~PLAY_STEREO);        		
        		break;        	

        	case 2:
        	default:
				AUDIO_WRITE(REG_I2S_ACTL_RESET, AUDIO_READ(REG_I2S_ACTL_RESET) | PLAY_STEREO);        		        	
        		break;        	
		}        		
	#endif        
	
//		enable_earphonr_detect();
#endif
        
        spin_unlock_irqrestore(&w55fa93_audio->irqlock, flags);

	LEAVE();
        return 0;
}

int w55fa93_dma_getposition(struct snd_pcm_substream *substream,
                           dma_addr_t *src, dma_addr_t *dst)
{
//        struct snd_pcm_runtime *runtime = substream->runtime;
//        struct w55fa93_audio *w55fa93_audio = runtime->private_data;

	ENTER();
	
#ifdef CONFIG_SND_SOC_W55FA93_I2S
        if (src != NULL)
                *src = AUDIO_READ(REG_I2S_ACTL_PDSTC);

        if (dst != NULL)
                *dst = AUDIO_READ(REG_I2S_ACTL_RDSTC);
#endif

#ifdef CONFIG_SND_SOC_W55FA93_SPU	

	#ifdef CONFIG_HEADSET_ENABLED		// for headset_detect
		Disable_IRQ(HEADSET_IRQ_NUM);	
	#endif	

	#ifdef CONFIG_SPU_WITH_I2S_OUTPUT 			
		// in SPU playback with I2S, the current address must check I2S current addr, 
		// otherwise a little "pop" noise will be heared in I2S (mhkuo@20121109)
         *src = AUDIO_READ(REG_I2S_ACTL_PDSTC);	
	#else
        if (src != NULL)
        {
        	{
				AUDIO_WRITE(REG_AIC_MDCR,(1<<IRQ_SPU));
				*src = DrvSPU_GetCurrentAddress(0);		// get channel-0 current address
				AUDIO_WRITE(REG_AIC_MECR,(1<<IRQ_SPU));						
			}        		
		}              
	#endif		
	
	#ifdef CONFIG_HEADSET_ENABLED		// for headset_detect
		Enable_IRQ(HEADSET_IRQ_NUM);	    
	#endif
	
#endif                
	LEAVE();
        return 0;
}

static snd_pcm_uframes_t w55fa93_dma_pointer(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        dma_addr_t src, dst;
        unsigned long res;
        struct w55fa93_audio *w55fa93_audio = runtime->private_data;
        snd_pcm_uframes_t frames;
	
		ENTER();

		spin_lock(&w55fa93_audio->lock);
        w55fa93_dma_getposition(substream, &src, &dst);
        if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
                res = dst - runtime->dma_addr;
        else
                res = src - runtime->dma_addr;

        frames = bytes_to_frames(substream->runtime, res);
        spin_unlock(&w55fa93_audio->lock);

		LEAVE();        
		return frames;
}

static int w55fa93_dma_open(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct w55fa93_audio *w55fa93_audio;

		ENTER();
	
        snd_soc_set_runtime_hwparams(substream, &w55fa93_pcm_hardware);

#ifdef CONFIG_SND_SOC_W55FA93_SPU
        w55fa93_audio = w55fa93_spu_data;
#endif

#ifdef CONFIG_SND_SOC_W55FA93_I2S
        w55fa93_audio = w55fa93_i2s_data;
#endif
        runtime->private_data = w55fa93_audio;              
		
		LEAVE();		
        return 0;
}

static int w55fa93_dma_close(struct snd_pcm_substream *substream)
{
		ENTER();
	
        return 0;
}

int w55fa93_dma_create(struct w55fa93_audio *w55fa93_audio)
{
		int ret;
		ENTER();
		
		DBG("w55fa93_audio->irq_num = 0x%x !!! \n", w55fa93_audio->irq_num);
		ret = request_irq(w55fa93_audio->irq_num, w55fa93_dma_interrupt, IRQF_DISABLED, "w55fa93-dma", w55fa93_audio);			    if(ret)          
	    	return -EBUSY;
	    
	    return ret;
}
EXPORT_SYMBOL_GPL(w55fa93_dma_create);

int w55fa93_dma_destroy(struct w55fa93_audio *w55fa93_audio)
{
	ENTER();
	
		 free_irq(w55fa93_audio->irq_num, w55fa93_audio);
		 
		 return 0;    
}
EXPORT_SYMBOL_GPL(w55fa93_dma_destroy);

static int w55fa93_dma_mmap(struct snd_pcm_substream *substream,
                           struct vm_area_struct *vma)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
		ENTER();
		
        return dma_mmap_writecombine(substream->pcm->card->dev, vma,
                                     runtime->dma_area,
                                     runtime->dma_addr,
                                     runtime->dma_bytes);
}

static struct snd_pcm_ops w55fa93_dma_ops = {
        .open		= w55fa93_dma_open,
        .close		= w55fa93_dma_close,
        .ioctl		= snd_pcm_lib_ioctl,
        .hw_params	= w55fa93_dma_hw_params,
        .hw_free	= w55fa93_dma_hw_free,
        .prepare	= w55fa93_dma_prepare,
        .pointer	= w55fa93_dma_pointer,
        .mmap		= w55fa93_dma_mmap,
};

static void w55fa93_dma_free_dma_buffers(struct snd_pcm *pcm)
{
	ENTER();
		
        snd_pcm_lib_preallocate_free_for_all(pcm);
}

static u64 w55fa93_pcm_dmamask = DMA_BIT_MASK(32);
static int w55fa93_dma_new(struct snd_card *card,
                          struct snd_soc_dai *dai, struct snd_pcm *pcm)
{
	ENTER();
		
        if (!card->dev->dma_mask)
                card->dev->dma_mask = &w55fa93_pcm_dmamask;
        if (!card->dev->coherent_dma_mask)
                card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

        snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
                                              card->dev, 16 * 1024, (16 * 1024) - 1);

        return 0;
}

struct snd_soc_platform w55fa93_soc_platform = {
        .name		= "w55fa93-dma",
        .pcm_ops	= &w55fa93_dma_ops,
        .pcm_new	= w55fa93_dma_new,
        .pcm_free	= w55fa93_dma_free_dma_buffers,
};
EXPORT_SYMBOL_GPL(w55fa93_soc_platform);

static int __init w55fa93_soc_platform_init(void)
{
        return snd_soc_register_platform(&w55fa93_soc_platform);
}

static void __exit w55fa93_soc_platform_exit(void)
{
        snd_soc_unregister_platform(&w55fa93_soc_platform);
}

module_init(w55fa93_soc_platform_init);
module_exit(w55fa93_soc_platform_exit);

#ifdef CONFIG_HEADSET_ENABLED		

#define GPIOA	0
#define GPIOB	1
#define GPIOC	2
#define GPIOD	3
#define GPIOE	4

	int checkGPIO(int uGPIO, int uPin)
	{
		int port;
		
		port = inl(REG_GPIOA_PIN + uGPIO*0x10);		      			
		if (port & (0x01 << uPin))
			return 1;
		else
			return 0;
	}		
	
	void setGPIO(int uGPIO, int uPin, int uValue)
	{
		if (uValue)
			outl(inl(REG_GPIOA_DOUT+uGPIO*0x10)|(0x01 << uPin), REG_GPIOA_DOUT+uGPIO*0x10); 	// GPD4 = low
		else
			outl(inl(REG_GPIOA_DOUT+uGPIO*0x10)&(~(0x01 << uPin)), REG_GPIOA_DOUT+uGPIO*0x10); 	// GPD4 = low			
	}		

	void headset_detection(void);
	static irqreturn_t headset_detect_irq(int irq, void *dev_id)
	{
	
	        u32 src;
	
	      	Disable_IRQ(HEADSET_IRQ_NUM);		
	      		
	#if defined (CONFIG_HEADSET_GPB2_AND_SPEAKER_GPB3)
	        //printk("headset detect irq \n");
	        src = inl(REG_IRQTGSRC0);
	        outl(src & 0x00040000, REG_IRQTGSRC0);
	        	      		        
			if(src & 0x00040000)
				headset_detection();	        

	#elif defined (CONFIG_HEADSET_GPD14_AND_SPEAKER_GPA2)
	        //printk("headset detect irq \n");
	        src = inl(REG_IRQTGSRC1);
	        outl(src & 0x40000000, REG_IRQTGSRC1);
	        	      		        
			if(src & 0x40000000)
				headset_detection();	        
	
	#elif defined (CONFIG_HEADSET_GPE0_AND_SPEAKER_GPE1)			
	        //printk("headset detect irq \n");
	        src = inl(REG_IRQTGSRC2);
	        outl(src & 0x00000001, REG_IRQTGSRC2);
	        	      		        
			if(src & 0x00000001)
				headset_detection();	        

	#elif defined (CONFIG_HEADSET_GPD3_AND_SPEAKER_GPD4)
	        printk("headset detect irq \n");  
	        src = inl(REG_IRQTGSRC1);
	        outl(src & 0x00080000, REG_IRQTGSRC1);
	        	      		        
			if(src & 0x00080000)
				headset_detection();	        
					
	#endif				
				
	 //       Disable_IRQ(HEADSET_IRQ_NUM);

	        // clear source

	        
	      	Enable_IRQ(HEADSET_IRQ_NUM);			        
	        return IRQ_HANDLED;
	}
	
	void headset_detection(void)
	{
	        int val;
	
	#if defined (CONFIG_HEADSET_GPB2_AND_SPEAKER_GPB3)	
	        //detect headset is plugged in or not
	        val = inl(REG_GPIOB_PIN);
	
	        //disable/enable speaker
	        if (val & 0x004) 		// GPIOB_2
	        {			
				if(earphone == 1)
					return;
	
		      	mdelay(30);		
	        	val = inl(REG_GPIOB_PIN);		      	
		        if (val & 0x004) 		// GPIOB_2
		        {			
	                printk("headset plugged in!!\n");		        	
	                earphone = 1;
					outl(inl(REG_GPIOB_DOUT) & ~(1 << 3), REG_GPIOB_DOUT);	// switch GGPIOB_3
					
				}
				else
				{
	                printk("headset plugged out!!\n");		        						
	                earphone = 0;
					outl(inl(REG_GPIOB_DOUT) | (1 << 3), REG_GPIOB_DOUT);	// switch GGPIOB_3
				}										
				
	        } 
	        else 
	        {
				if(earphone == 0)
					return;

		      	mdelay(30);		
	        	val = inl(REG_GPIOB_PIN);		      	
		        if (val & 0x004) 		// GPIOB_2
		        {			
	                printk("headset plugged in!!\n");		        	
	                earphone = 1;
					outl(inl(REG_GPIOB_DOUT) & ~(1 << 3), REG_GPIOB_DOUT);	// switch GGPIOB_3
					
				}
				else
				{
	                printk("headset plugged out!!\n");		        						
	                earphone = 0;
					outl(inl(REG_GPIOB_DOUT) | (1 << 3), REG_GPIOB_DOUT);	// switch GGPIOB_3
				}										
					
	    	}
	    	
	#elif defined (CONFIG_HEADSET_GPD14_AND_SPEAKER_GPA2)			
	
	        //detect headset is plugged in or not
	        val = inl(REG_GPIOD_PIN);
	
	        //disable/enable speaker
	        if (val & 0x4000) 		// GPIOD_14
	        {			
				if(earphone == 0)
					return;

		      	mdelay(30);		
	        	val = inl(REG_GPIOD_PIN);		      	
		        if (val & 0x4000) 		// GPIOD_14
		        {			
	                printk("headset plugged out!!\n");		        								        	
		        	outl(inl(REG_GPIOA_DOUT)|(0x0004), REG_GPIOA_DOUT); 	// GPA2 = high        		                
	                earphone = 0;
	                
	                
		#ifdef CONFIG_SOUND_W55FA93_PLAY_SPU				                
				//	spu_SetPAN(0, 0x1F00);			// MSB 8-bit = left channel; LSB 8-bit = right channel	
					if (s_PlayChannels == 1) {
						spu_SetPAN(0, 0x1F00);			// MSB 8-bit = left channel; LSB 8-bit = right channel		
					}
					else if (s_PlayChannels == 2) {
						spu_SetPAN(1, 0x1F00);			// MSB 8-bit = left channel; LSB 8-bit = right channel		
					}
		#endif					
				}
				else
				{
	                printk("headset plugged in!!\n");		        	
		        	outl(inl(REG_GPIOA_DOUT)&(~0x0004), REG_GPIOA_DOUT); 	// GPA2 = low        		                	                
	                earphone = 1;

		#ifdef CONFIG_SOUND_W55FA93_PLAY_SPU				                	                
				//	spu_SetPAN(0, 0x1F00);			// MSB 8-bit = left channel; LSB 8-bit = right channel				
					if (s_PlayChannels == 1) {
						spu_SetPAN(0, 0x1F1F);			// MSB 8-bit = left channel; LSB 8-bit = right channel		
					}
					else if (s_PlayChannels == 2) {					
						spu_SetPAN(1, 0x001F);			// MSB 8-bit = left channel; LSB 8-bit = right channel				
					}
		#endif					
				}										
				
	        } 
	        else 
	        {
				if(earphone == 1)
					return;

		      	mdelay(30);		
	        	val = inl(REG_GPIOD_PIN);		      	
		        if (val & 0x4000) 		// GPIOD_14
		        {			
	                printk("headset plugged out!!\n");		        								        	
		        	outl(inl(REG_GPIOA_DOUT)|(0x0004), REG_GPIOA_DOUT); 	// GPA2 = high        		                	                
	                earphone = 0;
	                
		#ifdef CONFIG_SOUND_W55FA93_PLAY_SPU				                	                
				//	spu_SetPAN(0, 0x1F00);			// MSB 8-bit = left channel; LSB 8-bit = right channel	
	
					if (s_PlayChannels == 1) {
						spu_SetPAN(0, 0x1F00);			// MSB 8-bit = left channel; LSB 8-bit = right channel		
					}
					else if (s_PlayChannels == 2) {
						spu_SetPAN(1, 0x1F00);			// MSB 8-bit = left channel; LSB 8-bit = right channel		
					}
		#endif					
				}
				else
				{
	                printk("headset plugged in!!\n");		        	
		        	outl(inl(REG_GPIOA_DOUT)&(~0x0004), REG_GPIOA_DOUT); 	// GPA2 = low        		                	                	                
	                earphone = 1;
	                
		#ifdef CONFIG_SOUND_W55FA93_PLAY_SPU				                	                
				//	spu_SetPAN(0, 0x1F00);			// MSB 8-bit = left channel; LSB 8-bit = right channel				
					if (s_PlayChannels == 1) {
						spu_SetPAN(0, 0x1F1F);			// MSB 8-bit = left channel; LSB 8-bit = right channel		
					}
					else if (s_PlayChannels == 2) {					
						spu_SetPAN(1, 0x001F);			// MSB 8-bit = left channel; LSB 8-bit = right channel				
					}
		#endif					
				}										
	    	}
			
	#elif defined (CONFIG_HEADSET_GPE0_AND_SPEAKER_GPE1)			
	
	        //detect headset is plugged in or not
	        val = inl(REG_GPIOE_PIN);
	
	        //disable/enable speaker
	        if (val & 0x001) 		// GPIOE_0
	        {			
				if(earphone == 1)
					return;
	
		      	mdelay(30);		
	        	val = inl(REG_GPIOE_PIN);		      	
		        if (val & 0x001) 		// GPIOE_0
		        {			
	                printk("headset plugged in!!\n");		        	
	                earphone = 1;
					outl(inl(REG_GPIOE_DOUT) & ~(1 << 1), REG_GPIOE_DOUT);	// switch GGPIOE_1

				}
				else
				{
	                printk("headset plugged out!!\n");		        						
	                earphone = 0;
					outl(inl(REG_GPIOE_DOUT) | (1 << 1), REG_GPIOE_DOUT);	// switch GGPIOE_1
				}										
				
	        } 
	        else 
	        {
				if(earphone == 0)
					return;

		      	mdelay(30);		
	        	val = inl(REG_GPIOE_PIN);		      	
		        if (val & 0x001) 		// GPIOE_0
		        {			
	                printk("headset plugged in!!\n");		        	
	                earphone = 1;
					outl(inl(REG_GPIOE_DOUT) & ~(1 << 1), REG_GPIOE_DOUT);	// switch GGPIOE_1
				
				}
				else
				{
	                printk("headset plugged out!!\n");		        						
	                earphone = 0;
					outl(inl(REG_GPIOE_DOUT) | (1 << 1), REG_GPIOE_DOUT);	// switch GGPIOE_1
				}										
	    	}
	    	
	#elif defined (CONFIG_HEADSET_GPD3_AND_SPEAKER_GPD4)			
	
	        //detect headset is plugged in or not; disable/enable speaker
	        
	        if (checkGPIO(GPIOD, 3) != EARPHONE_STATE) 		// GPIOD_3
	        {			
				if(earphone == 0)
					return;

		      	mdelay(30);		
				val = checkGPIO(GPIOD, 3);
	        	if (val != EARPHONE_STATE) 		// GPIOD_3
		        {			
	                printk("headset plugged out!!\n");		        								        	
       				setGPIO(GPIOD, 4, SPEAKER_STATE);
	                earphone = 0;
	                
		#ifdef CONFIG_SOUND_W55FA93_PLAY_SPU				                
				//	spu_SetPAN(0, 0x1F00);			// MSB 8-bit = left channel; LSB 8-bit = right channel	
					if (s_PlayChannels == 1) {
						spu_SetPAN(0, 0x1F00);			// MSB 8-bit = left channel; LSB 8-bit = right channel		
					}
					else if (s_PlayChannels == 2) {
						spu_SetPAN(1, 0x1F00);			// MSB 8-bit = left channel; LSB 8-bit = right channel		
					}
		#endif					
				}
				else
				{
	                printk("headset plugged in!!\n");		        	
       				setGPIO(GPIOD, 4, ~SPEAKER_STATE);
	                earphone = 1;

		#ifdef CONFIG_SOUND_W55FA93_PLAY_SPU				                	                
					if (s_PlayChannels == 1) {
						spu_SetPAN(0, 0x1F1F);			// MSB 8-bit = left channel; LSB 8-bit = right channel		
					}
					else if (s_PlayChannels == 2) {					
						spu_SetPAN(1, 0x001F);			// MSB 8-bit = left channel; LSB 8-bit = right channel				
					}
		#endif					
				}										
				
	        } 
	        else 
	        {
				if(earphone == 1)
					return;

		      	mdelay(30);		
				val = checkGPIO(GPIOD, 3);
		        if (val != EARPHONE_STATE) 		// GPIOD_3
		        {			
	                printk("headset plugged out!!\n");		        								        	
       				setGPIO(GPIOD, 4, SPEAKER_STATE);	                
	                earphone = 0;
	                
		#ifdef CONFIG_SOUND_W55FA93_PLAY_SPU				                	                
					if (s_PlayChannels == 1) {
						spu_SetPAN(0, 0x1F00);			// MSB 8-bit = left channel; LSB 8-bit = right channel		
					}
					else if (s_PlayChannels == 2) {
						spu_SetPAN(1, 0x1F00);			// MSB 8-bit = left channel; LSB 8-bit = right channel		
					}
		#endif					
				}
				else
				{
	                printk("headset plugged in!!\n");		        	
       				setGPIO(GPIOD, 4, ~SPEAKER_STATE);	                	                
	                earphone = 1;
	                
		#ifdef CONFIG_SOUND_W55FA93_PLAY_SPU				                	                
					if (s_PlayChannels == 1) {
						spu_SetPAN(0, 0x1F1F);			// MSB 8-bit = left channel; LSB 8-bit = right channel		
					}
					else if (s_PlayChannels == 2) {					
						spu_SetPAN(1, 0x001F);			// MSB 8-bit = left channel; LSB 8-bit = right channel				
					}
		#endif					
				}										
	    	}
	    	
	#endif			    	
	}

	int get_headset_status(void)
	{
	        return(earphone);
	}

	int enable_earphone_detect(void)
	{
	#if defined (CONFIG_HEADSET_GPB2_AND_SPEAKER_GPB3)
        outl(inl(REG_GPBFUN) & ~MF_GPB2, REG_GPBFUN);				// headset detect plug IN/OUT
        outl(inl(REG_GPBFUN) & ~MF_GPB3, REG_GPBFUN);				// speaker control signal
        outl(inl(REG_GPIOB_OMD) & ~(0x0004), REG_GPIOB_OMD); 		// port B2 input
        outl(inl(REG_GPIOB_PUEN) | (0x0004), REG_GPIOB_PUEN); 		// port B2 pull-up
        
		outl(inl(REG_GPIOB_OMD)  | (0x0008), REG_GPIOB_OMD); 		// port B3 output
        outl(inl(REG_GPIOB_PUEN) | (0x0008), REG_GPIOB_PUEN); 		// port B3 pull-up

		if ( inl(REG_GPIOB_PIN) & 0x0004)	// headset plug_in
		{
        	outl(inl(REG_GPIOB_DOUT) &~(0x0008), REG_GPIOB_DOUT);	// port B3 = low		
			earphone = 1;
		}        	
		else			
		{					// headset plug_out		
        	outl(inl(REG_GPIOB_DOUT) | (0x0008), REG_GPIOB_DOUT); 	// port B3 = high        	
			earphone = 0;
		}			

        outl(inl(REG_IRQTGSRC0) & 0x00040000, REG_IRQTGSRC0);               
        outl((inl(REG_IRQSRCGPB) & ~(0x30)) | 0x20, REG_IRQSRCGPB); // port B2 as nIRQ2 source
        outl(inl(REG_IRQENGPB) | 0x00040004, REG_IRQENGPB); 		// falling/rising edge trigger
        
        outl(0x10,  REG_AIC_SCCR); // force clear previous interrupt, 

//        printk("register the headset_detect_irq\n");
        
	    if (request_irq(HEADSET_IRQ_NUM, headset_detect_irq, IRQF_DISABLED, "FA93_headset_DETECT", NULL) != 0) {
	            printk("register the headset_detect_irq failed!\n");
	            return -1;
	    }
	    
        //Enable_IRQ(HEADSET_IRQ_NUM);	    
		outl((1<<4), REG_AIC_MECR);		        
	
	#elif (CONFIG_HEADSET_GPD14_AND_SPEAKER_GPA2)				

        outl(inl(REG_GPDFUN) & ~MF_GPD14, REG_GPDFUN);				// headset detect plug IN/OUT
        outl(inl(REG_GPIOD_OMD) & ~(0x4000), REG_GPIOD_OMD); 		// port D14 input
        outl(inl(REG_GPIOD_PUEN) | (0x4000), REG_GPIOD_PUEN); 		// port D14 pull-up

        outl(inl(REG_GPAFUN) & ~MF_GPA2, REG_GPAFUN);				// speaker control by GPA2
        outl(inl(REG_GPIOA_OMD) | (0x0004), REG_GPIOA_OMD); 		
        outl(inl(REG_GPIOA_PUEN) | (0x0004), REG_GPIOA_PUEN); 		
        
		if ( inl(REG_GPIOD_PIN) & 0x4000)	// headset plug_out
		{
	       	outl(inl(REG_GPIOA_DOUT)|(0x0004), REG_GPIOA_DOUT); 	// GPA2 = high
	       	
	#ifdef CONFIG_SOUND_W55FA93_PLAY_SPU				                	                
			spu_SetPAN(0, 0x1F00);			// MSB 8-bit = left channel; LSB 8-bit = right channel	
			spu_SetPAN(1, 0x1F00);			// MSB 8-bit = left channel; LSB 8-bit = right channel		
	#endif				
			earphone = 0;		
		}		
		else								// headset plug_in		
		{
	       	outl(inl(REG_GPIOA_DOUT)&(~0x0004), REG_GPIOA_DOUT); 	// GPA2 = low
		       	
	#ifdef CONFIG_SOUND_W55FA93_PLAY_SPU				                	                			
			DrvSPU_SetPAN(0, 0x1F00);			// MSB 8-bit = left channel; LSB 8-bit = right channel				
			DrvSPU_SetPAN(1, 0x001F);			// MSB 8-bit = left channel; LSB 8-bit = right channel				
	#endif			
			earphone = 1;	
		}			

        outl(inl(REG_IRQTGSRC1) & 0x40000000, REG_IRQTGSRC1);               
               
        outl((inl(REG_IRQSRCGPD) & ~(0x30000000)) | 0x20000000, REG_IRQSRCGPD); // port D14 as nIRQ2 source
        outl(inl(REG_IRQENGPD) | 0x40004000, REG_IRQENGPD); 		// falling/rising edge trigger
        
        outl(0x10,  REG_AIC_SCCR); // force clear previous interrupt, 

//        printk("register the headset_detect_irq\n");
        
	    if (request_irq(HEADSET_IRQ_NUM, headset_detect_irq, IRQF_DISABLED, "FA93_headset_DETECT", NULL) != 0) {
	            printk("register the headset_detect_irq failed!\n");
	            return -1;
	    }
	    
        //Enable_IRQ(HEADSET_IRQ_NUM);	    
		outl((1<<4), REG_AIC_MECR);		        
        
	#elif (CONFIG_HEADSET_GPE0_AND_SPEAKER_GPE1)				

        outl(inl(REG_GPEFUN) & ~MF_GPE0, REG_GPEFUN);				// headset detect plug IN/OUT
        outl(inl(REG_GPEFUN) & ~MF_GPE1, REG_GPEFUN);				// speaker control signal
        outl(inl(REG_GPIOE_OMD) & ~(0x0001), REG_GPIOE_OMD); 		// port E0 input
        outl(inl(REG_GPIOE_PUEN) | (0x0001), REG_GPIOE_PUEN); 		// port E0 pull-up
        
		outl(inl(REG_GPIOE_OMD)  | (0x0002), REG_GPIOE_OMD); 		// port E1 output
        outl(inl(REG_GPIOE_PUEN) | (0x0002), REG_GPIOE_PUEN); 		// port E1 pull-up

		if ( inl(REG_GPIOE_PIN) & 0x0001)	// headset plug_in
		{
        	outl(inl(REG_GPIOE_DOUT) &~(0x0002), REG_GPIOE_DOUT);	// port E1 = low		
			earphone = 1;
		}			        	
		else								// headset plug_out		
		{
        	outl(inl(REG_GPIOE_DOUT) | (0x0002), REG_GPIOE_DOUT); 	// port E1 = high        	
			earphone = 0;
		}			
               

        outl(inl(REG_IRQTGSRC2) & 0x00000001, REG_IRQTGSRC2);               
               
        outl((inl(REG_IRQSRCGPE) & ~(0x03)) | 0x02, REG_IRQSRCGPE); // port E0 as nIRQ2 source
        outl(inl(REG_IRQENGPE) | 0x00010001, REG_IRQENGPE); 		// falling/rising edge trigger
        
//		outl((inl(REG_AIC_SCR2) & 0xFFFFFF00) | 0x000000c7, REG_AIC_SCR2);

        outl(0x10,  REG_AIC_SCCR); // force clear previous interrupt, 

//        printk("register the headset_detect_irq\n");
        
	    if (request_irq(HEADSET_IRQ_NUM, headset_detect_irq, IRQF_DISABLED, "FA93_headset_DETECT", NULL) != 0) {
	            printk("register the headset_detect_irq failed!\n");
	            return -1;
	    }
	    
        //Enable_IRQ(HEADSET_IRQ_NUM);	    
		outl((1<<4), REG_AIC_MECR);		        

	#elif (CONFIG_HEADSET_GPD3_AND_SPEAKER_GPD4)				

        outl(inl(REG_GPDFUN) & ~MF_GPD3, REG_GPDFUN);				// headset detect plug IN/OUT
        outl(inl(REG_GPIOD_OMD) & ~(0x0008), REG_GPIOD_OMD); 		// port D3 input
        outl(inl(REG_GPIOD_PUEN) | (0x0008), REG_GPIOD_PUEN); 		// port D3 pull-up

        outl(inl(REG_GPDFUN) & ~MF_GPD4, REG_GPDFUN);				// speaker control by GPD4
        outl(inl(REG_GPIOD_OMD) | (0x0010), REG_GPIOD_OMD); 		
        outl(inl(REG_GPIOD_PUEN) | (0x0010), REG_GPIOD_PUEN); 		
        
        if (checkGPIO(GPIOD, 3) != EARPHONE_STATE) 		// GPIOD_3
		{
			setGPIO(GPIOD, 4, SPEAKER_STATE);			
	       	
	#ifdef CONFIG_SOUND_W55FA93_PLAY_SPU				                	                
			DrvSPU_SetPAN(0, 0x1F00);			// MSB 8-bit = left channel; LSB 8-bit = right channel	
			DrvSPU_SetPAN(1, 0x1F00);			// MSB 8-bit = left channel; LSB 8-bit = right channel		
	#endif				
			earphone = 0;		
		}		
		else								// headset plug_in		
		{
			setGPIO(GPIOD, 4, ~SPEAKER_STATE);						
		       	
	#ifdef CONFIG_SOUND_W55FA93_PLAY_SPU				                	                			
			DrvSPU_SetPAN(0, 0x1F00);			// MSB 8-bit = left channel; LSB 8-bit = right channel				
			DrvSPU_SetPAN(1, 0x001F);			// MSB 8-bit = left channel; LSB 8-bit = right channel				
	#endif			
			earphone = 1;
		}			

        outl(inl(REG_IRQTGSRC1) & 0x00080000, REG_IRQTGSRC1);               
               
        outl((inl(REG_IRQSRCGPD) & ~(0x000000c0)) | 0x00000080, REG_IRQSRCGPD); // port D3 as nIRQ2 source
        outl(inl(REG_IRQENGPD) | 0x00080008, REG_IRQENGPD); 		// falling/rising edge trigger
        
        outl(0x10,  REG_AIC_SCCR); // force clear previous interrupt, 

//        printk("register the headset_detect_irq\n");
        
	    if (request_irq(HEADSET_IRQ_NUM, headset_detect_irq, IRQF_DISABLED, "FA93_headset_DETECT", NULL) != 0) {
	            printk("register the headset_detect_irq failed!\n");
	            return -1;
	    }
	    
        //Enable_IRQ(HEADSET_IRQ_NUM);	    
		outl((1<<4), REG_AIC_MECR);		        
	#endif
	
		return 0;
	}
#endif


MODULE_AUTHOR("Wan ZongShun, <mcuos.com@gmail.com>");
MODULE_DESCRIPTION("w55fa93 Audio DMA module");
MODULE_LICENSE("GPL");
