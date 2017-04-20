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

#ifndef _W55FA93_AUDIO_H
#define _W55FA93_AUDIO_H

#include <linux/io.h>

#define AUDIO_WRITE(addr, val)		__raw_writel(val, addr)
#define AUDIO_READ(addr)			__raw_readl(addr)


#define AU_SAMPLE_RATE_192000	192000
#define AU_SAMPLE_RATE_96000	96000
#define AU_SAMPLE_RATE_88200	88200
#define AU_SAMPLE_RATE_64000	64000
#define AU_SAMPLE_RATE_48000	48000
#define AU_SAMPLE_RATE_44100	44100
#define AU_SAMPLE_RATE_32000	32000
#define AU_SAMPLE_RATE_24000	24000
#define AU_SAMPLE_RATE_22050	22050
#define AU_SAMPLE_RATE_20000	20000	/* Only for audio recording from ADC */ 	
#define AU_SAMPLE_RATE_16000	16000
#define AU_SAMPLE_RATE_12000	12000 	/* Only for audio recording from ADC */ 
#define AU_SAMPLE_RATE_11025	11025
#define AU_SAMPLE_RATE_8000		8000


#define W55FA93_AUDIO_SAMPLECLK		0x00
#define W55FA93_AUDIO_CLKDIV		0x01


struct w55fa93_audio {
	void __iomem *mmio;
	spinlock_t irqlock, lock;
	dma_addr_t dma_addr[2];
	unsigned long buffersize[2];
	unsigned long irq_num;
	struct snd_pcm_substream *substream[2];
	struct resource *res;
//	struct clk *clk;
	struct clk *spu_clk;
	struct clk *i2s_clk;	
	struct clk *eng_clk;	
	struct device *dev;	
};

#ifdef CONFIG_SND_SOC_W55FA93_SPU
extern struct w55fa93_audio *w55fa93_spu_data;
extern struct snd_soc_dai w55fa93_spu_dai;
extern struct snd_soc_platform w55fa93_soc_platform;

extern struct snd_soc_dai w55fa93_dac_dai;
extern struct snd_soc_codec_device soc_codec_dev_w55fa93_dac;
#endif

#ifdef CONFIG_SND_SOC_W55FA93_I2S
extern struct w55fa93_audio *w55fa93_i2s_data;
extern struct snd_soc_dai w55fa93_i2s_dai;
extern struct snd_soc_platform w55fa93_soc_platform;
#endif

int w55fa93_dma_create(struct w55fa93_audio *w55fa93_audio);
int w55fa93_dma_destroy(struct w55fa93_audio *w55fa93_audio);

#endif /*end _W55FA93_AUDIO_H */
