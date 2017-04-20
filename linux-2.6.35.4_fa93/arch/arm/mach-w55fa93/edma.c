/* linux/arch/arm/mach-w55fa93/edma.c
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
 *   2010/06/01     CCChang add this file for nuvoton EDMA.
 */

#include <linux/module.h>
//#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
//#include <linux/poll.h>
#include <asm/errno.h>
//#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/irq.h>
//#include <asm/hardware.h>
#include <mach/w55fa93_reg.h>
#include <mach/DrvEDMA.h>

#define DRIVER_NAME "w55fa93-blt"
#define EDMA_USE_IRQ
//#define pr_debug printk

/*
 * struct w55fa93_edma_channel - W55FA93 specific EDMA extension
 * @name: name specified by EDMA client
 * @irq_handler: client callback for end of transfer
 * @err_handler: client callback for error condition
 * @data: clients context data for callbacks
 * @dma_mode: direction of the transfer %EDMA_MODE_READ or %EDMA_MODE_WRITE
 * @sg: pointer to the actual read/written chunk for scatter-gather emulation
 * @resbytes: total residual number of bytes to transfer
 *            (it can be lower or same as sum of SG mapped chunk sizes)
 * @sgcount: number of chunks to be read/written
 *
 * Structure is used for IMX EDMA processing. It would be probably good
 * @struct dma_struct in the future for external interfacing and use
 * @struct w55fa93_edma_channel only as extension to it.
 */

struct w55fa93_edma_channel {
	const char *name;
	S_DRVEDMA_DESCRIPT_FORMAT *sg;
	unsigned int resbytes;
	void *data;
	int dma_num;
	int in_use;
//	void (*irq_handler) (int, void *);
//	void (*err_handler) (int, void *, int errcode);
//	void (*prog_handler) (int, void *, struct scatterlist *);
//	unsigned int dma_mode;
//	struct scatterlist *sg;
//	u32 ccr_from_device;
//	u32 ccr_to_device;
};

//static char _dbg_name[256];
static struct w55fa93_edma_channel w55fa93_edma_channels[MAX_CHANNEL_NUM+1];

struct w55fa93_edma_direction {
	int src_dir;
	int dest_dir;
};

static struct w55fa93_edma_direction w55fa93_edma_set_dir[MAX_CHANNEL_NUM+1];

/**
 * w55fa93_edma_setup_cst - setup W55FA93 EDMA channel for color space transform
 *
 * @eSrcFormat: the source color format
 * @eDestFormat: the destination color format
 *
 * Return value: if incorrect parameters are provided -%EBUSY.
 *		Zero indicates success.
 */
int
w55fa93_edma_setup_cst(E_DRVEDMA_COLOR_FORMAT eSrcFormat, E_DRVEDMA_COLOR_FORMAT eDestFormat)
{
	struct w55fa93_edma_channel *w55fa93edma = &w55fa93_edma_channels[0];

	if (w55fa93edma->in_use)
		return -EBUSY;

	DrvEDMA_SetColorTransformFormat(eSrcFormat, eDestFormat);
	DrvEDMA_SetColorTransformOperation(eDRVEDMA_ENABLE, eDRVEDMA_DISABLE);

	return 0;
}
EXPORT_SYMBOL(w55fa93_edma_setup_cst);

/**
 * w55fa93_edma_clear_cst - clear W55FA93 EDMA channel for color space transform
 *
 * Return value: if incorrect parameters are provided -%EBUSY.
 *		Zero indicates success.
 */
int
w55fa93_edma_clear_cst(void)
{
	//struct w55fa93_edma_channel *w55fa93edma = &w55fa93_edma_channels[0];

	//if (w55fa93edma->in_use)
		//return -EBUSY;

	DrvEDMA_SetColorTransformOperation(eDRVEDMA_DISABLE, eDRVEDMA_DISABLE);

	return 0;
}
EXPORT_SYMBOL(w55fa93_edma_clear_cst);

/**
 * w55fa93_edma_setup_single - setup W55FA93 EDMA channel for linear memory to/from
 * device transfer
 *
 * @channel: W55FA93 EDMA channel number
 * @src_addr: the source EDMA/physical memory address of the linear data block
 * @dest_addr: the destination EDMA/physical memory address of the linear data block
 * @dma_length: length of the data block in bytes
 *
 * Return value: if incorrect parameters are provided -%EINVAL.
 *		Zero indicates success.
 */
int
w55fa93_edma_setup_single(int channel, unsigned int src_addr, unsigned int dest_addr,
			  unsigned int dma_length)
{
	struct w55fa93_edma_channel *w55fa93edma = &w55fa93_edma_channels[channel];
	S_DRVEDMA_CH_ADDR_SETTING sSrcAddr, sDestAddr;

	w55fa93edma->sg = NULL;

	if (w55fa93edma->in_use)
		return -EBUSY;

	if ((!src_addr)||(!dest_addr)) {
		printk(KERN_ERR "w55fa93edma%d: w55fa93_edma_setup_single null address\n",
		       channel);
		return -EINVAL;
	}

	if (!dma_length) {
		printk(KERN_ERR "w55fa93edma%d: w55fa93_edma_setup_single zero length\n",
		       channel);
		return -EINVAL;
	}

	sSrcAddr.u32Addr = src_addr;
	sSrcAddr.eAddrDirection = eDRVEDMA_DIRECTION_INCREMENTED;
	sDestAddr.u32Addr = dest_addr;
	sDestAddr.eAddrDirection = eDRVEDMA_DIRECTION_INCREMENTED;
	
	if (channel != 0)
	{
		if (w55fa93_edma_set_dir[channel].src_dir != -1)
			sSrcAddr.eAddrDirection = w55fa93_edma_set_dir[channel].src_dir;		
		if (w55fa93_edma_set_dir[channel].dest_dir != -1)			
			sDestAddr.eAddrDirection = w55fa93_edma_set_dir[channel].dest_dir;		
	}
	

	DrvEDMA_SetTransferSetting(channel, &sSrcAddr, &sDestAddr, dma_length);

	return 0;
}
EXPORT_SYMBOL(w55fa93_edma_setup_single);

#if 0
/*
 * w55fa93_edma_sg_next - prepare next chunk for scatter-gather EDMA emulation
 */
static inline int w55fa93_edma_sg_next(int channel, struct scatterlist *sg)
{
	struct w55fa93_edma_channel *w55fa93edma = &w55fa93_edma_channels[channel];
	unsigned long now;

	if (!w55fa93edma->name) {
		printk(KERN_CRIT "%s: called for not allocated channel %d\n",
		       __func__, channel);
		return 0;
	}

	now = min(w55fa93edma->resbytes, sg->length);
	if (w55fa93edma->resbytes != W55FA93_EDMA_LENGTH_LOOP)
		w55fa93edma->resbytes -= now;

	if ((w55fa93edma->dma_mode & EDMA_MODE_MASK) == EDMA_MODE_READ)
		__raw_writel(sg->dma_address, EDMA_DAR(channel));
	else
		__raw_writel(sg->dma_address, EDMA_SAR(channel));

	__raw_writel(now, EDMA_CNTR(channel));

	pr_debug("w55fa93edma%d: next sg chunk dst 0x%08x, src 0x%08x, "
		"size 0x%08x\n", channel,
		 __raw_readl(EDMA_DAR(channel)),
		 __raw_readl(EDMA_SAR(channel)),
		 __raw_readl(EDMA_CNTR(channel)));

	return now;
}

/**
 * w55fa93_edma_setup_sg - setup W55FA93 EDMA channel SG list to/from device transfer
 * @channel: W55FA93 EDMA channel number
 * @sg: pointer to the scatter-gather list/vector
 * @sgcount: scatter-gather list hungs count
 * @dma_length: total length of the transfer request in bytes
 * @dev_addr: physical device port address
 * @dmamode: EDMA transfer mode, %EDMA_MODE_READ from the device to the memory
 *           or %EDMA_MODE_WRITE from memory to the device
 *
 * The function sets up EDMA channel state and registers to be ready for
 * transfer specified by provided parameters. The scatter-gather emulation
 * is set up according to the parameters.
 *
 * The full preparation of the transfer requires setup of more register
 * by the caller before w55fa93_edma_enable() can be called.
 *
 * Return value: if incorrect parameters are provided -%EINVAL.
 * Zero indicates success.
 */
int
w55fa93_edma_setup_sg(int channel,
		 struct scatterlist *sg, unsigned int sgcount,
		 unsigned int dma_length, unsigned int dev_addr,
		 unsigned int dmamode)
{
	struct w55fa93_edma_channel *w55fa93edma = &w55fa93_edma_channels[channel];

	if (w55fa93edma->in_use)
		return -EBUSY;

	w55fa93edma->sg = sg;
	w55fa93edma->dma_mode = dmamode;
	w55fa93edma->resbytes = dma_length;

	if (!sg || !sgcount) {
		printk(KERN_ERR "w55fa93edma%d: w55fa93_edma_setup_sg epty sg list\n",
		       channel);
		return -EINVAL;
	}

	if (!sg->length) {
		printk(KERN_ERR "w55fa93edma%d: w55fa93_edma_setup_sg zero length\n",
		       channel);
		return -EINVAL;
	}

	if ((dmamode & EDMA_MODE_MASK) == EDMA_MODE_READ) {
		pr_debug("w55fa93edma%d: %s sg=%p sgcount=%d total length=%d "
			"dev_addr=0x%08x for read\n",
			channel, __func__, sg, sgcount, dma_length, dev_addr);

		__raw_writel(dev_addr, EDMA_SAR(channel));
		__raw_writel(w55fa93edma->ccr_from_device, EDMA_CCR(channel));
	} else if ((dmamode & EDMA_MODE_MASK) == EDMA_MODE_WRITE) {
		pr_debug("w55fa93edma%d: %s sg=%p sgcount=%d total length=%d "
			"dev_addr=0x%08x for write\n",
			channel, __func__, sg, sgcount, dma_length, dev_addr);

		__raw_writel(dev_addr, EDMA_DAR(channel));
		__raw_writel(w55fa93edma->ccr_to_device, EDMA_CCR(channel));
	} else {
		printk(KERN_ERR "w55fa93edma%d: w55fa93_edma_setup_sg bad dmamode\n",
		       channel);
		return -EINVAL;
	}

	w55fa93_edma_sg_next(channel, sg);

	return 0;
}
EXPORT_SYMBOL(w55fa93_edma_setup_sg);
#endif

/**
 * w55fa93_edma_setup_pages - setup W55FA93 EDMA channel SG list for source pages
 * @length: total length of the transfer request in bytes
 * @src_addr: source pages
 * @dest_addr: destination physical address
 *
 * The function sets up EDMA channel state and registers to be ready for
 * transfer specified by provided parameters. The scatter-gather emulation
 * is set up according to the parameters.
 *
 * The full preparation of the transfer requires setup of more register
 * by the caller before w55fa93_edma_enable() can be called.
 *
 * Return value: if incorrect parameters are provided -%EINVAL.
 * Zero indicates success.
 */
int
w55fa93_edma_setup_pages(struct page **pages, unsigned int nr_pages,
			unsigned int dest_addr, unsigned int dma_length)
{
	struct w55fa93_edma_channel *w55fa93edma = &w55fa93_edma_channels[0];
	S_DRVEDMA_DESCRIPT_FORMAT *psSGFmt;
	unsigned int u32Value, u32TranferByte;	 
	unsigned int u32DestAddr;
	int i;

	if (w55fa93edma->in_use)
		return -EBUSY;

	if (dma_length <= 0) {
		printk(KERN_ERR "w55fa93_edma_setup_pages zero length\n");
		return -EINVAL;
	}

	w55fa93edma->sg = kmalloc(nr_pages * sizeof(S_DRVEDMA_DESCRIPT_FORMAT), GFP_KERNEL);
	w55fa93edma->resbytes = dma_length;

	// Set channel 0 transfer address and Scatter-Gather
	u32Value = inp32(REG_VDMA_CSR); 
	u32Value = (u32Value & ~SAD_SEL) | (eDRVEDMA_DIRECTION_INCREMENTED << SOURCE_DIRECTION_BIT);
	u32Value = (u32Value & ~DAD_SEL) | (eDRVEDMA_DIRECTION_INCREMENTED << DESTINATION_DIRECTION_BIT);
	outp32(REG_VDMA_CSR, u32Value); 
	DrvEDMA_EnableScatterGather(0);
	DrvEDMA_SetScatterGatherTblStartAddr(0, virt_to_phys(w55fa93edma->sg));
	//printk("DrvEDMA_SetScatterGatherTblStartAddr: %p\n", virt_to_phys(w55fa93edma->sg));

	psSGFmt = w55fa93edma->sg;
	u32DestAddr = dest_addr;
	u32TranferByte = 0;
	
	/* Not contiguous, writeout per-page instead.. */
	for (i = 0; i < nr_pages; i++) {
		u32TranferByte = (w55fa93edma->resbytes >= PAGE_SIZE) ? PAGE_SIZE : w55fa93edma->resbytes;
		w55fa93edma->resbytes -= u32TranferByte;

		// set source and destination address
		psSGFmt->u32SourceAddr = page_to_phys(pages[i]);
		psSGFmt->u32DestAddr = u32DestAddr;
		//printk("SG idx=%d, src_phys=%p, dest_phys=%p, lengh=%d\n", i,psSGFmt->u32SourceAddr,psSGFmt->u32DestAddr,u32TranferByte);
		// set stride transfer byte count & byte count
		psSGFmt->u32StrideAndByteCount = u32TranferByte;
		// set source offset byte length and destination offset byte length
		psSGFmt->u32Offset = 0;
		// set EOT for last descript format  	   						  
		if (w55fa93edma->resbytes == 0)
{
			//printk("set EOT for last descript format\n");
			psSGFmt->u32Offset |= 0x80000000;
}
		// set next Scatter-Gather table address
		psSGFmt->u32NextSGTblAddr = (unsigned int)(psSGFmt+1); 
		psSGFmt++;
		
		u32DestAddr += u32TranferByte;
	}
	
	return 0;
}
EXPORT_SYMBOL(w55fa93_edma_setup_pages);

/**
 * w55fa93_edma_setup_virtual - setup W55FA93 EDMA channel SG list for source virtual address
 * @length: total length of the transfer request in bytes
 * @src_addr: source virtual address
 * @dest_addr: destination physical address
 *
 * The function sets up EDMA channel state and registers to be ready for
 * transfer specified by provided parameters. The scatter-gather emulation
 * is set up according to the parameters.
 *
 * The full preparation of the transfer requires setup of more register
 * by the caller before w55fa93_edma_enable() can be called.
 *
 * Return value: if incorrect parameters are provided -%EINVAL.
 * Zero indicates success.
 */
int
w55fa93_edma_setup_virtual(unsigned int src_addr, unsigned int dest_addr,
				unsigned int dma_length)
{
	struct w55fa93_edma_channel *w55fa93edma = &w55fa93_edma_channels[0];
	unsigned int sgcount;
	S_DRVEDMA_DESCRIPT_FORMAT *psSGFmt;
	unsigned int u32Value, u32TranferByte;	 
	unsigned int u32SrcAddr, u32DestAddr;

	if (w55fa93edma->in_use)
		return -EBUSY;

	if (dma_length <= 0) {
		printk(KERN_ERR "w55fa93_edma_setup_virtual zero length\n");
		return -EINVAL;
	}

	if ((src_addr & 0x0FFF) || (src_addr & 0x0FFF)) {
		printk(KERN_ERR "w55fa93_edma_setup_virtual address is not PAGE_SIZE alignment\n");
		return -EINVAL;
	}

	sgcount = (dma_length + PAGE_SIZE - 1) / PAGE_SIZE;
	w55fa93edma->sg = kmalloc(sgcount * sizeof(S_DRVEDMA_DESCRIPT_FORMAT), GFP_KERNEL);
	w55fa93edma->resbytes = dma_length;

	// Set channel 0 transfer address and Scatter-Gather
	u32Value = inp32(REG_VDMA_CSR); 
	u32Value = (u32Value & ~SAD_SEL) | (eDRVEDMA_DIRECTION_INCREMENTED << SOURCE_DIRECTION_BIT);
	u32Value = (u32Value & ~DAD_SEL) | (eDRVEDMA_DIRECTION_INCREMENTED << DESTINATION_DIRECTION_BIT);
	outp32(REG_VDMA_CSR, u32Value); 
	DrvEDMA_EnableScatterGather(0);
	DrvEDMA_SetScatterGatherTblStartAddr(0, virt_to_phys(w55fa93edma->sg));

	psSGFmt = w55fa93edma->sg;
	u32SrcAddr = src_addr;
	u32DestAddr = dest_addr;
	u32TranferByte = 0;
	
	do {
		u32TranferByte = (w55fa93edma->resbytes >= PAGE_SIZE) ? PAGE_SIZE : w55fa93edma->resbytes;
		w55fa93edma->resbytes -= u32TranferByte;

		// set source and destination address
		psSGFmt->u32SourceAddr = virt_to_phys((void*)u32SrcAddr);
		psSGFmt->u32DestAddr = u32DestAddr;
		// set stride transfer byte count & byte count
		psSGFmt->u32StrideAndByteCount = u32TranferByte;
		// set source offset byte length and destination offset byte length
		psSGFmt->u32Offset = 0;
		// set EOT for last descript format  	   						  
		if (w55fa93edma->resbytes == 0)
			psSGFmt->u32Offset |= 0x80000000;
		// set next Scatter-Gather table address
		//psSGFmt->u32NextSGTblAddr = (unsigned int)(psSGFmt+1); 
		psSGFmt++;
		
		u32SrcAddr += u32TranferByte;
		u32DestAddr += u32TranferByte;
	} while (w55fa93edma->resbytes > 0);
	
	return 0;
}
EXPORT_SYMBOL(w55fa93_edma_setup_virtual);

/**
 * w55fa93_edma_free - release previously acquired channel
 * @channel: W55FA93 EDMA channel number
 */
void w55fa93_edma_free_sg(int channel)
{
	struct w55fa93_edma_channel *w55fa93edma = &w55fa93_edma_channels[channel];
	S_DRVEDMA_DESCRIPT_FORMAT *psSGFmt;
	S_DRVEDMA_DESCRIPT_FORMAT *psSGFree;

	if (!w55fa93edma->name) {
		printk(KERN_CRIT
		       "%s: trying to free free channel %d\n",
		       __func__, channel);
		return;
	}

	/* Free Scatter-Gather table */
	psSGFmt = w55fa93edma->sg;
	while (psSGFmt) {
		psSGFree = psSGFmt;
		if (psSGFmt->u32Offset & (1<<15))
			psSGFmt = (S_DRVEDMA_DESCRIPT_FORMAT *)psSGFmt->u32NextSGTblAddr;
		else
			psSGFmt = NULL;
		kfree(psSGFree);
	}
}
//EXPORT_SYMBOL(w55fa93_edma_free_sg);

/**
 * w55fa93_edma_setup_handlers - setup W55FA93 EDMA channel notification handlers
 * @channel: W55FA93 EDMA channel number
 * @interrupt: W55FA93 EDMA interrupt enable
 * @irq_handler: the pointer to the function called if the transfer
 *		ends successfully
 * @data: user specified value to be passed to the handlers
 */
int
w55fa93_edma_setup_handlers(int channel, int interrupt,
		       void (*irq_handler) (void *),
		       void *data)
{
	struct w55fa93_edma_channel *w55fa93edma = &w55fa93_edma_channels[channel];
	unsigned long flags;
	int ret;

	if (!w55fa93edma->name) {
		printk(KERN_CRIT "%s: called for not allocated channel %d\n",
		       __func__, channel);
		return -ENODEV;
	}

	local_irq_save(flags);

//	__raw_writel(1 << channel, EDMA_DISR);
	ret = DrvEDMA_InstallCallBack(channel, interrupt, (PFN_DRVEDMA_CALLBACK)irq_handler, data);

	local_irq_restore(flags);

	return ret;
}
EXPORT_SYMBOL(w55fa93_edma_setup_handlers);

/**
 * w55fa93_edma_enable - function to start W55FA93 EDMA channel operation
 * @channel: W55FA93 EDMA channel number
 *
 * The channel has to be allocated by driver through w55fa93_edma_request()
 * or w55fa93_pdma_find_and_request() function.
 * The transfer parameters has to be set to the channel registers through
 * call of the w55fa93_edma_setup_single() or w55fa93_edma_setup_sg() function
 */
void w55fa93_edma_enable(int channel)
{
	struct w55fa93_edma_channel *w55fa93edma = &w55fa93_edma_channels[channel];
	unsigned long flags;

	pr_debug("w55fa93edma%d: w55fa93_edma_enable\n", channel);

	if (!w55fa93edma->name) {
		printk(KERN_CRIT "%s: called for not allocated channel %d\n",
		       __func__, channel);
		return;
	}

	local_irq_save(flags);

	DrvEDMA_EnableCH(channel, eDRVEDMA_ENABLE);
#ifdef EDMA_USE_IRQ
	DrvEDMA_EnableInt(channel, eDRVEDMA_SG | eDRVEDMA_BLKD | eDRVEDMA_TABORT);
#endif

	local_irq_restore(flags);
}
EXPORT_SYMBOL(w55fa93_edma_enable);

/**
 * w55fa93_edma_disable - stop, finish W55FA93 EDMA channel operatin
 * @channel: W55FA93 EDMA channel number
 */
void w55fa93_edma_disable(int channel)
{	
	unsigned long flags;

	pr_debug("w55fa93edma%d: w55fa93_edma_disable\n", channel);

	local_irq_save(flags);

#ifdef EDMA_USE_IRQ
	DrvEDMA_DisableInt(channel, eDRVEDMA_SG | eDRVEDMA_BLKD | eDRVEDMA_TABORT);
#endif
	DrvEDMA_EnableCH(channel, eDRVEDMA_DISABLE);

	local_irq_restore(flags);
}
EXPORT_SYMBOL(w55fa93_edma_disable);

/**
 * w55fa93_edma_request - request/allocate specified channel number
 * @channel: W55FA93 EDMA channel number
 * @name: the driver/caller own non-%NULL identification
 */
int w55fa93_edma_request(int channel, const char *name)
{
	struct w55fa93_edma_channel *w55fa93edma = &w55fa93_edma_channels[channel];
	unsigned long flags;
	int ret = 0;

	if (w55fa93edma->in_use)
		return -EBUSY;

	/* basic sanity checks */
	if (!name)
		return -EINVAL;

	if (channel > MAX_CHANNEL_NUM) {
		printk(KERN_CRIT "%s: called for non-existed channel %d\n",
		       __func__, channel);
		return -EINVAL;
	}

	local_irq_save(flags);

	if (w55fa93edma->name) {
	        //printk("EDMA name = %s\n",w55fa93edma->name);
		local_irq_restore(flags);
		return -EBUSY;
	}
	memset(w55fa93edma, 0, sizeof(w55fa93edma));
	w55fa93edma->name = name;

	DrvEDMA_EnableCH(channel, eDRVEDMA_ENABLE);
#ifdef EDMA_USE_IRQ
	DrvEDMA_EnableInt(channel, eDRVEDMA_SG | eDRVEDMA_BLKD | eDRVEDMA_TABORT);
#endif

	local_irq_restore(flags); /* request_irq() can block */

	//KC DrvEDMA_CHSoftwareReset(channel);

	return ret;
}
EXPORT_SYMBOL(w55fa93_edma_request);

/**
 * w55fa93_edma_free - release previously acquired channel
 * @channel: W55FA93 EDMA channel number
 */
void w55fa93_edma_free(int channel)
{
	unsigned long flags;
	struct w55fa93_edma_channel *w55fa93edma = &w55fa93_edma_channels[channel];
	unsigned int regval;

	if (!w55fa93edma->name) {
//printk("Last Name = %s\n",_dbg_name);
		printk(KERN_CRIT
		       "%s: trying to free channel %d\n",
		       __func__, channel);
		return;
	}

	local_irq_save(flags);

	/* Disable interrupts */
	//w55fa93_edma_disable(channel);

	if (w55fa93edma->in_use)
		w55fa93edma->in_use = 0;
	if (w55fa93edma->sg)
		w55fa93_edma_free_sg(channel);

	w55fa93edma->name = NULL;
	
	regval = inp32(REG_VDMA_CSR + channel * 0x100);
	// Reset channel if source for destination in wrap-around mode
	if (((regval & 0xC0) == 0xC0) || ((regval & 0x30) == 0x30)) 
	{
		regval = regval & ~0xF0;
		outp32(REG_VDMA_CSR + channel * 0x100, regval | 0x02);
		outp32(REG_VDMA_ISR + channel * 0x100, 0xF00);
	}

#ifdef EDMA_USE_IRQ
	DrvEDMA_DisableInt(channel, eDRVEDMA_SG | eDRVEDMA_BLKD | eDRVEDMA_TABORT | eDRVEDMA_WAR);
#endif
	// do not disable EDMA channel clocks to fix hang up issue
	DrvEDMA_EnableCH(channel, eDRVEDMA_DISABLE);	

	local_irq_restore(flags);
}
EXPORT_SYMBOL(w55fa93_edma_free);

/**
 * w55fa93_pdma_find_and_request - find and request some of free channels
 * @name: the driver/caller own non-%NULL identification
 *
 * This function tries to find a free channel in the specified priority group
 *
 * Return value: If there is no free channel to allocate, -%ENODEV is returned.
 *               On successful allocation channel is returned.
 */
int w55fa93_pdma_find_and_request(const char *name)
{
	int i;

	for (i = 1; i <= MAX_CHANNEL_NUM; i++)
		if (!w55fa93_edma_request(i, name))
			return i;

	printk(KERN_ERR "%s: no free PDMA channel found\n", __func__);

	return -ENODEV;
}
EXPORT_SYMBOL(w55fa93_pdma_find_and_request);

/**
 * w55fa93_edma_triggrt - function to start W55FA93 EDMA channel transfer
 * @channel: W55FA93 EDMA channel number
 */
void w55fa93_edma_trigger(int channel)
{
	unsigned long flags;
	struct w55fa93_edma_channel *w55fa93edma = &w55fa93_edma_channels[channel];

	//pr_debug("w55fa93edma%d: w55fa93_edma_trigger\n", channel);

	local_irq_save(flags);

	if (w55fa93edma->in_use)
		return;

	w55fa93edma->in_use = 1;
	DrvEDMA_CHEnablelTransfer(channel);

	local_irq_restore(flags);
}
EXPORT_SYMBOL(w55fa93_edma_trigger);

/**
 * w55fa93_edma_triggrt_done - function to set W55FA93 EDMA channel transfer done
 * @channel: W55FA93 EDMA channel number
 */
void w55fa93_edma_trigger_done(int channel)
{
	unsigned long flags;
	struct w55fa93_edma_channel *w55fa93edma = &w55fa93_edma_channels[channel];

	pr_debug("w55fa93edma%d: w55fa93_edma_trigger\n", channel);

	local_irq_save(flags);

	w55fa93edma->in_use = 0;

	local_irq_restore(flags);
}
EXPORT_SYMBOL(w55fa93_edma_trigger_done);

/**
 * w55fa93_edma_isbusy - function to query W55FA93 EDMA channel is busy or not
 * @channel: W55FA93 EDMA channel number
 */
int w55fa93_edma_isbusy(int channel)
{
	pr_debug("w55fa93edma%d: w55fa93_edma_isbusy\n", channel);

	return DrvEDMA_IsCHBusy(channel);
}
EXPORT_SYMBOL(w55fa93_edma_isbusy);

static irqreturn_t edma_irq_handler(int irq, void *dev_id)
{
	DrvEDMA_ISR();

	return IRQ_HANDLED;
}

static int __init w55fa93_edma_init(void)
{
	int ret = 0;
	int i;

	printk("w55fa93_edma_init\n");
	if (!request_mem_region((unsigned long)W55FA93_VA_EDMA, W55FA93_SZ_EDMA, DRIVER_NAME))
	{
		printk("%s: request_mem_region failed!\n", __FUNCTION__);
		return -EBUSY;
	}

	// EDMA open
	DrvEDMA_Open();

#ifdef EDMA_USE_IRQ
	ret = request_irq(IRQ_EDMA, edma_irq_handler, IRQF_DISABLED | IRQF_IRQPOLL, DRIVER_NAME, NULL);
	if (ret) {
		printk("cannot get irq %d - err %d\n", IRQ_EDMA, ret);
//		DrvEDMA_DisableInt();
		DrvEDMA_Close();
		return -EBUSY;
	}
#endif 	

	for (i = 0; i <= MAX_CHANNEL_NUM; i++) {
		w55fa93_edma_channels[i].name = NULL;
		w55fa93_edma_channels[i].sg = NULL;
		w55fa93_edma_channels[i].dma_num = i;
		
		w55fa93_edma_set_dir[i].src_dir = -1;		
		w55fa93_edma_set_dir[i].dest_dir = -1;				
	}

	return 0;
}


static void __exit w55fa93_edma_exit(void)
{
	release_mem_region((unsigned long)W55FA93_VA_EDMA, W55FA93_SZ_EDMA);
}

/**
 */
int w55fa93_edma_setAPB(int channel, E_DRVEDMA_APB_DEVICE eDevice, E_DRVEDMA_APB_RW eRWAPB, E_DRVEDMA_TRANSFER_WIDTH eTransferWidth)
{
	struct w55fa93_edma_channel *w55fa93edma = &w55fa93_edma_channels[channel];

	if (w55fa93edma->in_use)
		return -EBUSY;	

	pr_debug("w55fa93_edma_setAPB ch:%d: device =%d, width:%d: read/write=%d\n", channel,eDevice, eTransferWidth,eRWAPB);
	
	DrvEDMA_SetAPBTransferWidth(channel, eTransferWidth);
	DrvEDMA_SetCHForAPBDevice(channel, eDevice, eRWAPB);

	return 0;
}
EXPORT_SYMBOL(w55fa93_edma_setAPB);

int w55fa93_edma_set_wrapINTtype(int channel, int type)
{
	struct w55fa93_edma_channel *w55fa93edma = &w55fa93_edma_channels[channel];

	if (w55fa93edma->in_use)
		return -EBUSY;	

	pr_debug("w55fa93_edma_setAPBtransferwidth ch:%d: WrapIntType:%d:\n", channel,type);
	
	DrvEDMA_SetWrapIntType(channel, type);
	if (type !=0)
	{
		DrvEDMA_DisableInt(channel,eDRVEDMA_SG | eDRVEDMA_BLKD | eDRVEDMA_TABORT);
		DrvEDMA_EnableInt(channel, eDRVEDMA_WAR);
	}
	else
	{
		DrvEDMA_DisableInt(channel,eDRVEDMA_WAR);
		DrvEDMA_EnableInt(channel, eDRVEDMA_SG | eDRVEDMA_BLKD | eDRVEDMA_TABORT);		
	}
	
	

	return 0;
}
EXPORT_SYMBOL(w55fa93_edma_set_wrapINTtype);

int w55fa93_edma_set_direction(int channel, int src_dir, int dest_dir)
{
	struct w55fa93_edma_channel *w55fa93edma = &w55fa93_edma_channels[channel];

	if (w55fa93edma->in_use)
		return -EBUSY;	

	pr_debug("w55fa93_edma_SetTransferDirection ch:%d: Src Dir:%d, Dest Dir:%d\n", channel,src_dir, dest_dir);
	
	if ((channel > 0) && (channel <=MAX_CHANNEL_NUM))
	{
		w55fa93_edma_set_dir[channel].src_dir = src_dir;
		w55fa93_edma_set_dir[channel].dest_dir = dest_dir;	
	}

	return 0;
}
EXPORT_SYMBOL(w55fa93_edma_set_direction);



module_init(w55fa93_edma_init);
module_exit(w55fa93_edma_exit);
//arch_initcall(w55fa93_edma_init);
