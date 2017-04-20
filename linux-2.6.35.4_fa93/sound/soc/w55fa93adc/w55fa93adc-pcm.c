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
#include <mach/hardware.h>

#include "w55fa93adc-audio.h"
#include <mach/w55fa93_reg.h>
#include <mach/DrvEDMA.h>

//#define DBG
#ifdef DBG
#define ENTER()		printk("ENTER : %s  : %s\n",__FILE__, __FUNCTION__)
#define LEAVE()		printk("LEAVE : %s  : %s\n",__FILE__, __FUNCTION__)
#define ERRLEAVE()	printk("ERRLEAVE : %s  : %s\n",__FILE__, __FUNCTION__)
#define VDBG(x)		printk("\t%s\n", x);
#define SDBG		printk		
#else
#define ENTER(...)
#define LEAVE(...)	
#define ERRLEAVE(...)	
#define VDBG(...)	
#define SDBG(...)		
#endif

static const struct snd_pcm_hardware w55fa93adc_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
	SNDRV_PCM_INFO_BLOCK_TRANSFER |
	SNDRV_PCM_INFO_MMAP |
	SNDRV_PCM_INFO_MMAP_VALID |
	SNDRV_PCM_INFO_PAUSE |
	SNDRV_PCM_INFO_RESUME,
	.formats			= SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min			= 8000,
	.rate_max 			= 16000,
	.channels_min		= 1,
	.channels_max		= 1,
	.buffer_bytes_max	= 64*1024,
	.period_bytes_min	= 1*1024,
	.period_bytes_max	= 16*1024,
	.periods_min		= 1,
	.periods_max		= 1024,
};
/*
 *	Allocate snd page 
 */
static int w55fa93adc_dma_hw_params(struct snd_pcm_substream *substream,
                                struct snd_pcm_hw_params *params)
{	
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct w55fa93adc_audio *w55fa93adc_audio = runtime->private_data;
	unsigned long flags;
	int ret = 0;
	int edma_channel;
	int dmalen;

	ENTER();
	spin_lock_irqsave(&w55fa93adc_audio->irqlock, flags);
	edma_channel = w55fa93adc_audio->edma_channel;
	if(runtime->dma_addr == 0) {
		SDBG("******runtime->dma_addr=0\n");
		ret = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
		if (ret < 0)
		{
			ERRLEAVE();	
			return ret;
		}
		w55fa93adc_audio->substream[substream->stream] = substream;        	
	}

	w55fa93adc_audio->dma_addr[substream->stream] = runtime->dma_addr;
	w55fa93adc_audio->buffersize[substream->stream] = params_buffer_bytes(params);
	
	dmalen = ((w55fa93adc_audio->buffersize[substream->stream]%16)==0)? (w55fa93adc_audio->buffersize[substream->stream]):((w55fa93adc_audio->buffersize[substream->stream]/16+1)*16);
	SDBG("!!!!! Total size for audio = 0x%x\n", dmalen);

	w55fa93_edma_setup_single(edma_channel,									// int channel, 
					0xB800E020,												// unsigned int src_addr,
					w55fa93adc_audio->dma_addr[substream->stream],			// unsigned int dest_addr,
					//w55fa93adc_audio->buffersize[substream->stream]*2);	// Wrong
					//w55fa93adc_audio->buffersize[substream->stream]/2); 	// record data is only half. 
					//w55fa93adc_audio->buffersize[substream->stream]); 	// 8K and 16K is workable, 11025 is hang up.  
					dmalen);
	
	w55fa93_edma_trigger(edma_channel);				
	spin_unlock_irqrestore(&w55fa93adc_audio->irqlock, flags);
	LEAVE();
	return ret;
}
/*
static void nuc900_update_dma_register(struct snd_pcm_substream *substream,
                                       dma_addr_t dma_addr, size_t count)
{	
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct w55fa93adc_audio *w55fa93adc_audio = runtime->private_data;
        void __iomem *mmio_addr, *mmio_len;

	ENTER();
        if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
                mmio_addr = w55fa93adc_audio->mmio + ACTL_PDSTB;
                mmio_len = w55fa93adc_audio->mmio + ACTL_PDST_LENGTH;
        } else {
                mmio_addr = w55fa93adc_audio->mmio + ACTL_RDSTB;
                mmio_len = w55fa93adc_audio->mmio + ACTL_RDST_LENGTH;
        }

        AUDIO_WRITE(mmio_addr, dma_addr);
        AUDIO_WRITE(mmio_len, count);
	LEAVE();
} 
*/
extern void DrvADC_GetAutoGainTiming(unsigned int* pu32Period, unsigned int* pu32Attack, unsigned int* pu32Recovery, unsigned int* pu32Hold);
extern void DrvADC_SetAutoGainTiming(unsigned int u32Period, unsigned int u32Attack, unsigned int u32Recovery, unsigned int u32Hold);
static irqreturn_t w55fa93_dma_interrupt(int irq, void *dev_id)
{	
	struct w55fa93adc_audio *w55fa93adc_audio = g_pw55fa93_adc_data;//dev_id;//
	int stream = SNDRV_PCM_STREAM_CAPTURE;
	unsigned int u32Period, u32Attack, u32Recovery, u32Hold;
	ENTER();
	SDBG("******* interrupt g_pw55fa93_adc_data address = 0x%x\n", (unsigned int)g_pw55fa93_adc_data);	
	SDBG("******* interrupt w55fa93adc_audio address = 0x%x\n", (unsigned int)w55fa93adc_audio);
	DrvADC_GetAutoGainTiming(&u32Period,		//Period
								&u32Attack,		//Attack
								&u32Recovery,	//Recovery	
								&u32Hold);		//Hold
	if(u32Period <= 88){
		u32Period = u32Period+16;
		DrvADC_SetAutoGainTiming(u32Period,			//Period
									u32Attack,		//Attack
									u32Recovery,	//Recovery	
									u32Hold);		//Hold
		//printk("REG_AGC_CON = %x\n\n", inp32(REG_AGC_CON));
	}
	/* the need to be called, then the kernel will polling the dma pointer */
	snd_pcm_period_elapsed(w55fa93adc_audio->substream[stream]);   
	LEAVE();
    return IRQ_HANDLED;	
}

static int w55fa93adc_dma_hw_free(struct snd_pcm_substream *substream)
{	
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct w55fa93adc_audio *w55fa93adc_audio = runtime->private_data;
	
	ENTER();
	snd_pcm_lib_free_pages(substream);
	w55fa93adc_audio->substream[substream->stream] = NULL;
	LEAVE();
	return 0;
}
/*
 *	Setup to record/playback left channel, right channel or both
 */
static int w55fa93adc_dma_prepare(struct snd_pcm_substream *substream)
{		
	ENTER();			
	LEAVE();
    return 0;
}

int w55fa93adc_dma_getposition(struct snd_pcm_substream *substream,
                           dma_addr_t *src, dma_addr_t *dst)
{	
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct w55fa93adc_audio *w55fa93adc_audio = runtime->private_data;
	unsigned long flags;
	int edma_channel;

	ENTER();
	spin_lock_irqsave(&w55fa93adc_audio->irqlock, flags);
	edma_channel = w55fa93adc_audio->edma_channel;
	spin_unlock_irqrestore(&w55fa93adc_audio->irqlock, flags);

	if (src != NULL)
		*src = readl(REG_VDMA_CSR+0x100*edma_channel+0x04);

	if (dst != NULL)
		*dst = readl(REG_VDMA_CSR+0x100*edma_channel+0x18);
	LEAVE();
    return 0;
}

static snd_pcm_uframes_t w55fa93adc_dma_pointer(struct snd_pcm_substream *substream)
{	
	struct snd_pcm_runtime *runtime = substream->runtime;
	dma_addr_t src, dst;
	unsigned long res;
	struct w55fa93adc_audio *w55fa93adc_audio = runtime->private_data;	  
	snd_pcm_uframes_t frames;
		
	ENTER();      
	spin_lock(&w55fa93adc_audio->lock);
	
	w55fa93adc_dma_getposition(substream, &src, &dst);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
	{
		SDBG("Stream capture\n");	
		SDBG("runtime->dma_addr = %x\n", runtime->dma_addr);
		SDBG("dst address = %x\n", dst);		
                res = dst - runtime->dma_addr;
		SDBG("res = %x\n", (unsigned int)res);		
	}
    else
	{	
		SDBG("Stream playback\n");	
        res = src - runtime->dma_addr;
		SDBG("runtime->dma_addr = %x\n", runtime->dma_addr);
		SDBG("src address = %x\n", src);	
		SDBG("res = %x\n", (unsigned int)res);		
	}
    frames = bytes_to_frames(substream->runtime, res);
	spin_unlock(&w55fa93adc_audio->lock);
	LEAVE();
    return frames;
}
/*
 *¡@Set run time hardware capability 
 *  hook on the private data to run time stream
*/
#define CLIENT_ADC_NAME "w55fa93-adc"
static int w55fa93adc_dma_open(struct snd_pcm_substream *substream)
{	
	int edma_channel;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct w55fa93adc_audio *w55fa93adc_audio;
	
	ENTER();
	w55fa93adc_audio = runtime->private_data;
        	
	edma_channel = w55fa93_pdma_find_and_request(CLIENT_ADC_NAME); /* request EDMA channel */
	if(edma_channel == -ENODEV){
		printk("Run out of EDMA channel\n");
		return -ENODEV;
	}
	SDBG("EDMA channel = %d\n", edma_channel);
	g_pw55fa93_adc_data->edma_channel = edma_channel;
	w55fa93_edma_setAPB(edma_channel,					//int channel, 
				eDRVEDMA_ADC,			//E_DRVEDMA_APB_DEVICE eDevice, 
				eDRVEDMA_READ_APB,		//E_DRVEDMA_APB_RW eRWAPB, 
				eDRVEDMA_WIDTH_32BITS);		//E_DRVEDMA_TRANSFER_WIDTH eTransferWidth	

	w55fa93_edma_setup_handlers(edma_channel, 				//int channel
					eDRVEDMA_WAR, 				//int interrupt,	
					//adc_edma_irq_handler, 		//void (*irq_handler) (void *),
					(void*)w55fa93_dma_interrupt,
					w55fa93adc_audio);					//void *data

	w55fa93_edma_set_wrapINTtype(edma_channel , 
					eDRVEDMA_WRAPAROUND_EMPTY | 
					eDRVEDMA_WRAPAROUND_HALF);		//int channel, WR int type

	w55fa93_edma_set_direction(edma_channel , eDRVEDMA_DIRECTION_FIXED, eDRVEDMA_DIRECTION_WRAPAROUND);

#if 0
	w55fa93_edma_setup_single(edma_channel,		// int channel, 
					0xB800E020,					// unsigned int src_addr,  (ADC data port physical address) 
					phaddrrecord,				// unsigned int dest_addr,
					r_fragment_size*2);			// unsigned int dma_length /* Lenth equal 2 half buffer */

	w55fa93_edma_trigger(edma_channel);							
#endif 	
								
	snd_soc_set_runtime_hwparams(substream, &w55fa93adc_pcm_hardware);
	
	w55fa93adc_audio = g_pw55fa93_adc_data;
    	runtime->private_data = w55fa93adc_audio; 

	LEAVE();         	
    return 0;
}

static int w55fa93adc_dma_close(struct snd_pcm_substream *substream)
{	 
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct w55fa93adc_audio *w55fa93adc_audio = runtime->private_data;
	ENTER();
	w55fa93_edma_free(w55fa93adc_audio->edma_channel);
	LEAVE();
    return 0;
}

/*
 *	driver mount stage
 */
int w55fa93adc_dma_create(struct w55fa93adc_audio *w55fa93adc_audio)
{	
	ENTER();
	LEAVE();	
	return 0;		
}
EXPORT_SYMBOL_GPL(w55fa93adc_dma_create);
/*
 *	driver unmoount stage 
 */
int w55fa93adc_dma_destroy(struct w55fa93adc_audio *w55fa93adc_audio)
{	
	ENTER();	
	LEAVE();
	return 0;    
}
EXPORT_SYMBOL_GPL(w55fa93adc_dma_destroy);

static int w55fa93adc_dma_mmap(struct snd_pcm_substream *substream,
                           struct vm_area_struct *vma)
{	
	int ret;
    struct snd_pcm_runtime *runtime = substream->runtime;
	ENTER();
	ret = dma_mmap_writecombine(substream->pcm->card->dev, vma,
								 runtime->dma_area,
								 runtime->dma_addr,
								 runtime->dma_bytes);
	if(ret!=0)
		ERRLEAVE();
	else
		LEAVE();	
	return ret; 
}

static struct snd_pcm_ops w55fa93adc_dma_ops = {
        .open		= w55fa93adc_dma_open,
        .close		= w55fa93adc_dma_close,
        .ioctl		= snd_pcm_lib_ioctl,
        .hw_params	= w55fa93adc_dma_hw_params,
        .hw_free	= w55fa93adc_dma_hw_free,
        .prepare	= w55fa93adc_dma_prepare,
        .pointer	= w55fa93adc_dma_pointer,
        .mmap		= w55fa93adc_dma_mmap,
};

static void w55fa93adc_dma_free_dma_buffers(struct snd_pcm *pcm)
{	
	ENTER();
	snd_pcm_lib_preallocate_free_for_all(pcm);
	LEAVE();
}

static u64 w55fa93adc_pcm_dmamask = DMA_BIT_MASK(32);
static int w55fa93adc_dma_new(struct snd_card *card,
                          struct snd_soc_dai *dai, struct snd_pcm *pcm)
{	
	ENTER();
    if (!card->dev->dma_mask)
		card->dev->dma_mask = &w55fa93adc_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,				/* Preallocate for continuous memory */
										  card->dev, 16 * 1024, (16 * 1024) - 1);
	LEAVE();	
        return 0;
}

struct snd_soc_platform w55fa93adc_soc_platform = {
        .name		= "w55fa93adc-dma",
        .pcm_ops	= &w55fa93adc_dma_ops,
        .pcm_new	= w55fa93adc_dma_new,
        .pcm_free	= w55fa93adc_dma_free_dma_buffers,
};
EXPORT_SYMBOL_GPL(w55fa93adc_soc_platform);

static int __init w55fa93adc_soc_platform_init(void)
{	
	int ret;
	ENTER();
	ret = snd_soc_register_platform(&w55fa93adc_soc_platform);
	if(ret!=0)
		ERRLEAVE();
	else
		LEAVE();
	return ret;
}

static void __exit w55fa93adc_soc_platform_exit(void)
{	
	ENTER();
	snd_soc_unregister_platform(&w55fa93adc_soc_platform);
}

module_init(w55fa93adc_soc_platform_init);
module_exit(w55fa93adc_soc_platform_exit);

MODULE_DESCRIPTION("w55fa93adc Audio DMA module");
MODULE_LICENSE("GPL");
