/* w55fa93_blt.c
 *
 * Copyright (c) 2009 Nuvoton technology corporation
 * All rights reserved.
 * <ccchang12@nuvoton.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/pagemap.h>
#include <linux/interrupt.h>

#include <linux/slab.h>

#include <linux/poll.h>
#include <asm/errno.h>
#include <asm/cacheflush.h>
#include <mach/w55fa93_reg.h>

#include <mach/DrvEDMA.h>
//#include <asm/arch/w55fa93_reg.h>

#include "w55fa93_blt.h"


//#define DRIVER_NAME "w55fa93-blt"
#define DRIVER_NAME "blt"
#define BLT_USE_IRQ
#define EDMA_USE_IRQ

#define BLT_DYNAMIC_ALLOC_DEVICE_NUM
#define BLT_AUTOMATIC_CREATE_DEVICE_FILE


static int major_no = 198;
module_param (major_no, int, 0644);
static int minor_no = 0;
module_param (minor_no, int, 0644);
static int dev_no_created = 0;

static int blt_dev_created = 0;

#ifdef BLT_AUTOMATIC_CREATE_DEVICE_FILE
static struct class *blt_class = NULL;
static struct device *blt_device = NULL;
#endif	// #ifdef BLT_AUTOMATIC_CREATE_DEVICE_FILE


static struct cdev blt_dev;
static DECLARE_WAIT_QUEUE_HEAD(blt_wq);
static DECLARE_WAIT_QUEUE_HEAD(edma_wq);
blt_state_t edma_state = BLT_CLOSE;
//static int blt_nr = -1;
//module_param(blt_nr, int, 0);

void DrvBLT_ISR(void);
void DrvBLT_SetRGB565TransparentColor(unsigned int u16RGB565);
void DrvBLT_SetRGB565TransparentCtl(bool bEnable);

static irqreturn_t blt_irq_handler(int irq, void *dev_id, struct pt_regs *r)
{
	blt_priv_t *priv = (blt_priv_t *)dev_id;

	//printk("blt_irq_handler\n");
	DrvBLT_ISR();
	priv->state = BLT_FINISH;
	//wake_up_interruptible(&blt_wq);
	wake_up(&blt_wq);

	return IRQ_HANDLED;
}

void blt_edma_irq_handler(unsigned int arg)
{
	//printk("blt_edma_irq_handler\n");
	edma_state = BLT_FINISH;
	//wake_up_interruptible(&edma_wq);
	wake_up(&edma_wq);
}

static int w55fa93_blt_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	blt_priv_t *priv = (blt_priv_t *)kmalloc(sizeof(blt_priv_t), GFP_KERNEL);
	file->private_data = priv;
	
	//printk("blt_open\n");	
	/* initialize locks */
	init_MUTEX(&priv->lock);

#if 0	
	priv->state = BLT_CLOSE;
	priv->size_order = MAP_SIZE_ORDER;
	priv->vaddr = __get_free_pages(GFP_KERNEL, MAP_SIZE_ORDER);
	priv->paddr = virt_to_phys(priv->vaddr);
	priv->paddr_src = priv->paddr + FB_SIZE;
	priv->paddr_dst = priv->paddr;
	printk("get free pages from 0x%p\n", priv->vaddr);
#endif

	// BLT open
	DrvBLT_Open();

#ifdef BLT_USE_IRQ
	DrvBLT_EnableInt();
	//ret = request_irq(IRQ_BLT, blt_irq_handler, SA_INTERRUPT, DRIVER_NAME, priv);
	ret = request_irq(IRQ_BLT, (irq_handler_t)blt_irq_handler, IRQF_DISABLED | IRQF_IRQPOLL, DRIVER_NAME, priv);	

	
	if (ret) {
		printk("cannot get irq %d - err %d\n", IRQ_BLT, ret);
		ret = -EBUSY;
		DrvBLT_DisableInt();
		DrvBLT_Close();
	}
#endif 	

	return ret;
}

static int w55fa93_blt_close(struct inode *inode, struct file *file)
{
	blt_priv_t *priv = file->private_data;
	
	//printk("blt_close\n");	
	priv->state = BLT_CLOSE;

#ifdef BLT_USE_IRQ
	free_irq(IRQ_BLT, priv);
	DrvBLT_DisableInt();
#endif 	

	// BLT close
	DrvBLT_Close();
#if 0	
	if (priv->size_order >= 0)
		free_pages(priv->vaddr, priv->size_order);
#endif
	kfree(priv);

	// free VDMA resource
	if (edma_state != BLT_CLOSE) {
		w55fa93_edma_free(0);
	}

	return 0;
}

/*
* read information after blt finished
* don't support read data
*/
static ssize_t w55fa93_blt_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	blt_priv_t *priv = file->private_data;
	int nonblock = file->f_flags & O_NONBLOCK;
	int ret;
	
	//printk("blt_read\n");
#ifdef BLT_USE_IRQ
	if (!IS_DONE(priv->state))
	{
		if (nonblock)
		{
			ret = -EAGAIN;
			goto out;
		}
		//wait_event_interruptible(blt_wq, IS_DONE(priv->state));
		wait_event(blt_wq, IS_DONE(priv->state));
	}
#else
	while (DrvBLT_GetBusyStatus())
		;
	DrvBLT_ClearInt();
	priv->state = BLT_FINISH;
#endif
	
	if (IS_FINISH(priv->state))
		ret = 0;
	else
		ret = -1;

out:
	return ret;
}

/*
* write data for blt
*/
static ssize_t w55fa93_blt_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	blt_priv_t *priv = file->private_data;
	int nonblock = file->f_flags & O_NONBLOCK;
	int ret;
	
	//printk("blt_write\n");
	if (!IS_DONE(priv->state))
	{
		if (nonblock)
		{
			ret = -EAGAIN;
			goto out;
		}
		//wait_event_interruptible(blt_wq, IS_DONE(priv->state));
		wait_event(blt_wq, IS_DONE(priv->state));
	}
	down(&priv->lock);
	
	if (copy_from_user((void *)priv->vaddr, (void *)buf, count)) {
		ret = -EFAULT;
		goto out;
	}

	up(&priv->lock);
	ret = count;
out:
	return ret;
}

#if 0
static int w55fa93_blt_mmap(struct file *file, struct vm_area_struct *vma)
{	
	blt_priv_t *priv = file->private_data;
	unsigned long start = vma->vm_start;
	unsigned long size  = vma->vm_end-vma->vm_start;
	unsigned long page, pos;
	
	//printk("blt_mmap\n");
	//### Device memory size is too small for mapping
	if ((1 << priv->size_order) * 4096 < (vma->vm_end - vma->vm_start)) {
		//### Free previousone
		free_pages(priv->vaddr, priv->size_order);
		//### Realloc
		int pageOrder = 0;
		int pages = (vma->vm_end - vma->vm_start + 4095) / 4096;
		for (pageOrder = 0; pages > 0; pageOrder++)
			pages = (pages >> 1);

		priv->size_order = pageOrder;
		priv->vaddr = __get_free_pages(GFP_KERNEL, pageOrder);
		printk("Rellocated Free Page from: 0x%p, order:%d\n", priv->vaddr, priv->size_order);
		if (priv->vaddr == 0) {
			priv->size_order = -1;
			return -EAGAIN;
		}
		//memset(priv->vaddr, 0x77, vma->vm_end - vma->vm_start);
	}

	priv->size = vma->vm_end - vma->vm_start;
	priv->paddr = virt_to_phys(priv->vaddr);
	priv->paddr_src = priv->paddr + FB_SIZE;
	priv->paddr_dst = priv->paddr;
	vma->vm_pgoff = (priv->paddr) >> PAGE_SHIFT;

	printk("vm_start:0x%p, vm_pgoff:0x%p, size:%d, vm_page_prot:0x%p\n",
		vma->vm_start, vma->vm_pgoff, vma->vm_end - vma->vm_start,vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
		vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;

	//printk("mmap sucessfully\n");

	priv->state = BLT_IDLE;

	return 0;
}
#endif

//static int w55fa93_blt_ioctl(struct inode *inode, struct file *file, unsigned int cmd, void *arg)
static int w55fa93_blt_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	blt_priv_t *priv = file->private_data;
	static S_DRVBLT_BLIT_OP blitop;
	static S_DRVBLT_FILL_OP fillop;
	static S_DRVBLT_EDMA_OP edmaop;
	static S_DRVBLT_EDMA_CST_OP edmacstop;
	S_DRVBLT_DEST_FB dest;
	struct page **pages; 
	int nr_pages;
	int pixel_size, offset_addr;
	int ret = 0, ret1, i;
	unsigned int RGB565TransparentKey=0;
	
	//printk("blt ioctl\n");
	switch (cmd) {
	case BLT_FLUSH:
		while (DrvBLT_GetBusyStatus() != 0)
			;
		DrvBLT_ClearInt();
		priv->state = BLT_FINISH;
		break;

	case BLT_TRIGGER:
		file->f_pos = 0;
		priv->state = BLT_RUNNING;

		// flush kernel space cache
		flush_cache_all();
		//dmac_flush_range(priv->vaddr, priv->vaddr + priv->size);
		//__cpuc_coherent_kern_range(priv->vaddr, priv->vaddr + priv->size);
		// trigger BLT engine
		DrvBLT_Trigger();
		break;

	case BLT_SET_BLIT:
		if (copy_from_user((void *)&blitop, (void *)arg, sizeof(blitop))) {
			ret = -EFAULT;
			break;
		}
		priv->func = BLT_BLIT;

		DrvBLT_SetFillOP(FALSE);
		if (blitop.src.pSARGB8 != NULL)
			DrvBLT_SetColorPalette(blitop.src.u32PaletteInx, blitop.src.u32Num, blitop.src.pSARGB8);
		//blitop.src.u32SrcImageAddr = priv->paddr_src;
//printk("u32SrcImageAddr = 0x%p\n", blitop.src.u32SrcImageAddr);
		DrvBLT_SetSrcImage(blitop.src);

		if (blitop.transformation->destFormat == eDRVBLT_DEST_ARGB8888)
			pixel_size = 4;
		else if ((blitop.transformation->destFormat == eDRVBLT_DEST_RGB565)
			|| (blitop.transformation->destFormat == eDRVBLT_DEST_RGB555))
			pixel_size = 2;
		offset_addr = (blitop.dest.i32YOffset * blitop.dest.i32Stride + blitop.dest.i32XOffset) * pixel_size;
		//blitop.dest.u32FrameBufAddr = priv->paddr_dst + offset_addr;
//printk("u32FrameBufAddr = 0x%p\n", blitop.dest.u32FrameBufAddr);

		DrvBLT_SetDestFrameBuf(blitop.dest);
		DrvBLT_SetTransformMatrix(blitop.transformation->matrix);
		DrvBLT_SetSrcFormat(blitop.transformation->srcFormat);
		DrvBLT_SetDisplayFormat(blitop.transformation->destFormat);
		DrvBLT_SetTransFormFlag(blitop.transformation->flags);
		DrvBLT_SetColorMultiplier(blitop.transformation->colorMultiplier);
		DrvBLT_SetColorOffset(blitop.transformation->colorOffset);
		DrvBLT_SetFillStyle(blitop.transformation->fillStyle);
		break;

	case BLT_GET_BLIT:
		if (copy_to_user((void *)arg, (void *)&blitop, sizeof(blitop))) {
			ret = -EFAULT;
			break;
		}
		break;

	case BLT_SET_FILL:
		if (copy_from_user((void *)&fillop, (void *)arg, sizeof(fillop))) {
			ret = -EFAULT;
			break;
		}
		priv->func = BLT_FILL;

		DrvBLT_SetFillOP(TRUE);
		DrvBLT_SetARGBFillColor(fillop.color);

		if (fillop.format == eDRVBLT_DEST_ARGB8888)
			pixel_size = 4;
		else if ((fillop.format == eDRVBLT_DEST_RGB565)
			|| (fillop.format == eDRVBLT_DEST_RGB555))
			pixel_size = 2;
		offset_addr = (fillop.rect.i16Ymin * fillop.rowBytes + fillop.rect.i16Xmin) * pixel_size;

		//dest.u32FrameBufAddr = priv->paddr_dst + offset_addr;
		dest.u32FrameBufAddr = fillop.u32FrameBufAddr;
		dest.i32XOffset = fillop.rect.i16Xmin;
		dest.i32YOffset = fillop.rect.i16Ymin;
		dest.i32Stride = fillop.rowBytes;
		dest.i16Width = fillop.rect.i16Xmax - fillop.rect.i16Xmin;
		dest.i16Height = fillop.rect.i16Ymax - fillop.rect.i16Ymin;
		DrvBLT_SetDestFrameBuf(dest);
		DrvBLT_SetDisplayFormat(fillop.format);
		DrvBLT_SetFilAlpha(!!(fillop.blend));
		break;

	case BLT_GET_FILL:
		if (copy_to_user((void *)arg, (void *)&fillop, sizeof(fillop))) {
			ret = -EFAULT;
			break;
		}
		break;

	case BLT_EDMA_REQUEST:
		ret = w55fa93_edma_request(0, DRIVER_NAME);
		if (ret >= 0)
			edma_state = BLT_IDLE;
		break;

	case BLT_EDMA_FREE:
		w55fa93_edma_free(0);
		edma_state = BLT_CLOSE;
		break;

	case BLT_EDMA_SINGLE:
		if (copy_from_user((void *)&edmaop, (void *)arg, sizeof(edmaop))) {
			ret = -EFAULT;
			break;
		}

		nr_pages = 1;
		pages = kmalloc(nr_pages * sizeof(struct page *), GFP_KERNEL);
		if (!pages)
			return -ENOMEM;
		
		down_read(&current->mm->mmap_sem);
		ret1 = get_user_pages(current, current->mm, (unsigned long)edmaop.src_virt_addr,
				     nr_pages, 1, 0, pages, NULL);
		up_read(&current->mm->mmap_sem);
				
		if (ret1 < 1) {
			nr_pages = ret1;
			ret = -EINVAL;
			goto out_unmap_single;
		}
		
		ret = w55fa93_edma_setup_single(0, page_to_phys(pages[0]), edmaop.dest_phys_addr, edmaop.dma_length);
		if (ret < 0) {
			printk("w55fa93_edma_setup_single failed and returns %d\n", ret);
			goto out_unmap_single;
		}

		ret = w55fa93_edma_setup_handlers(0, eDRVEDMA_BLKD, (void *) blt_edma_irq_handler, NULL);
		if (ret < 0) {
			printk("w55fa93_edma_setup_handlers failed and returns %d\n", ret);
			break;
		}

out_unmap_single:
		if (nr_pages > 0)
			for (i = 0; i < nr_pages; i++)
				page_cache_release(pages[i]);
		kfree(pages);

		break;

	case BLT_EDMA_CST:
		if (copy_from_user((void *)&edmacstop, (void *)arg, sizeof(edmacstop))) {
			ret = -EFAULT;
			break;
		}

		ret = w55fa93_edma_setup_cst(edmacstop.src_fmt, edmacstop.dest_fmt);
		if (ret < 0) {
			printk("w55fa93_edma_setup_cst failed and returns %d\n", ret);
			break;
		}

		ret = w55fa93_edma_setup_single(0, edmacstop.src_phys_addr, edmacstop.dest_phys_addr, edmacstop.dma_length);
		if (ret < 0) {
			printk("w55fa93_edma_setup_single failed and returns %d\n", ret);
			break;
		}

		ret = w55fa93_edma_setup_handlers(0, eDRVEDMA_BLKD, (void *)blt_edma_irq_handler, NULL);
		if (ret < 0) {
			printk("w55fa93_edma_setup_handlers failed and returns %d\n", ret);
			break;
		}

		break;

	case BLT_EDMA_SG:
		if (copy_from_user((void *)&edmaop, (void *)arg, sizeof(edmaop))) {
			ret = -EFAULT;
			break;
		}

		nr_pages = (edmaop.dma_length + PAGE_SIZE - 1) >> PAGE_SHIFT;
		pages = kmalloc(nr_pages * sizeof(struct page *), GFP_KERNEL);
		if (!pages)
			return -ENOMEM;
		
		down_read(&current->mm->mmap_sem);
		ret1 = get_user_pages(current, current->mm, (unsigned long)edmaop.src_virt_addr,
				     nr_pages, 1, 0, pages, NULL);
		up_read(&current->mm->mmap_sem);
				
		if (ret1 < nr_pages) {
			nr_pages = ret1;
			ret = -EINVAL;
			goto out_unmap;
		}
		
		ret = w55fa93_edma_setup_pages(pages, nr_pages, edmaop.dest_phys_addr, edmaop.dma_length);
		if (ret < 0) {
			printk("w55fa93_edma_setup_pages failed and returns %d\n", ret);
			goto out_unmap;
		}

		ret = w55fa93_edma_setup_handlers(0, eDRVEDMA_BLKD, (void *)blt_edma_irq_handler, NULL);
		if (ret < 0) {
			printk("w55fa93_edma_setup_handlers failed and returns %d\n", ret);
			goto out_unmap;
		}

out_unmap:
		if (nr_pages > 0)
			for (i = 0; i < nr_pages; i++)
				page_cache_release(pages[i]);
		kfree(pages);

		break;

	case BLT_EDMA_TRIGGER:
		edma_state = BLT_RUNNING;
		// flush kernel space cache
		flush_cache_all();

		//w55fa93_edma_enable(0);
		// trigger EDMA engine
		w55fa93_edma_trigger(0);
		break;

	case BLT_EDMA_WAIT:
#ifdef EDMA_USE_IRQ
		if (!IS_DONE(edma_state))
		{
			if (file->f_flags & O_NONBLOCK)
			{
				ret = -EAGAIN;
				break;
			}
			//wait_event_interruptible(edma_wq, IS_DONE(edma_state));
			wait_event(edma_wq, IS_DONE(edma_state));
		}
		if (IS_FINISH(edma_state))
			ret = 0;
		else
			ret = -1;
#else
		while (DrvEDMA_IsCHBusy(0))
			;
		edma_state = BLT_FINISH;
#endif
		DrvEDMA_SetColorTransformOperation(eDRVEDMA_DISABLE, eDRVEDMA_DISABLE);
		//w55fa93_edma_disable(0);
		w55fa93_edma_trigger_done(0);
		break;
	case BLT_SET_RGB565_COLORKEY:
	    /*
		if (copy_from_user((unsigned int *)&RGB565TransparentKey, (void *)arg, 1*sizeof(unsigned int))) {
			ret = -EFAULT;
			break;
		}
		*/		
		RGB565TransparentKey = arg;			
		DrvBLT_SetRGB565TransparentColor(RGB565TransparentKey);
		break;
	case BLT_ENABLE_RGB565_COLORCTL:
		DrvBLT_SetRGB565TransparentCtl(1);
		break;
	case BLT_DISABLE_RGB565_COLORCTL:
		DrvBLT_SetRGB565TransparentCtl(0);
		break;
    case BLT_SRCFMT_PREMULALPHA:
        DrvBLT_SetRevealAlpha(eDRVBLT_EFFECTIVE);
        break;
    case BLT_SRCFMT_NONPREMULALPHA:
        DrvBLT_SetRevealAlpha(eDRVBLT_NO_EFFECTIVE);
        break;
	default:
		return -ENOIOCTLCMD;
	}
	return ret;
}

static struct file_operations w55fa93_blt_fops = {
	.owner		= THIS_MODULE,
	.open		= w55fa93_blt_open,
	.release	= w55fa93_blt_close,
	.read		= w55fa93_blt_read,
	.write		= w55fa93_blt_write,
//	.mmap		= w55fa93_blt_mmap,
    .ioctl      = w55fa93_blt_ioctl
//	.poll		= w55fa93_blt_poll,
};


static void w55fa93_blt_cleanup (void)
{
	dev_t dev_no = MKDEV(major_no, minor_no);
	
#ifdef BLT_AUTOMATIC_CREATE_DEVICE_FILE
	if (blt_device) {
		device_destroy(blt_class, dev_no);
		blt_device = NULL;
	}
	if (blt_class) {
		class_destroy(blt_class);
		blt_class = NULL;
	}
#endif	// #ifdef BLT_AUTOMATIC_CREATE_DEVICE_FILE
	
	if (blt_dev_created) {
		cdev_del(&blt_dev);
		blt_dev_created = 0;
	}
	
	if (dev_no_created) {
		unregister_chrdev_region(dev_no, 1);
		dev_no_created = 0;
	}
}

static int __devinit w55fa93_blt_init(void)
{
	int err = 0;
	
	do {    
#ifdef BLT_DYNAMIC_ALLOC_DEVICE_NUM
		dev_t dev_no;
		err = alloc_chrdev_region(&dev_no, minor_no, 1, DRIVER_NAME);
		if (err) {
			printk("[BLT Driver] alloc_chrdev_region failed\n");
			break;
		}
		major_no = MAJOR(dev_no);
		printk("93BLT major_no = %d, minor_no = %d\n",major_no, minor_no);
#else	    
    	dev_t dev_no = MKDEV(BLT_MAJOR, BLT_MINOR);
    
    	if (register_chrdev_region(dev_no, 1, DRIVER_NAME)) {
    		printk("initial the device error!\n");
    		err=-1;
    		//return -1;
    	}
#endif   //BLT_DYNAMIC_ALLOC_DEVICE_NUM  	
		dev_no_created = 1;
    
    	cdev_init(&blt_dev, &w55fa93_blt_fops);
    	blt_dev.owner = THIS_MODULE;
    	blt_dev.ops = &w55fa93_blt_fops;
    
    	if (cdev_add(&blt_dev, dev_no, 1))
    		printk(KERN_NOTICE "Error adding w55fa93 BLT char device!\n");

    	blt_dev_created = 1;    
    	
#ifdef BLT_AUTOMATIC_CREATE_DEVICE_FILE
		blt_class = class_create(THIS_MODULE, DRIVER_NAME);
		if(IS_ERR(blt_class)) {
			printk("[BLT Driver] class_create failed\n");
			err = PTR_ERR(blt_class);
			blt_class = NULL;
			break;
		}
		
		blt_device = device_create(blt_class, NULL, MKDEV(major_no, minor_no), NULL, DRIVER_NAME);
		if (IS_ERR(blt_device)) {
			printk("[BLT Driver] device_create failed\n");
			err = PTR_ERR(blt_device);
			blt_device = NULL;
			break;
		}

#endif	// #ifdef BLT_AUTOMATIC_CREATE_DEVICE_FILE
    } while(0);    	
    
    	if (!request_mem_region((unsigned long)W55FA93_VA_BLT, W55FA93_SZ_BLT, DRIVER_NAME))
    	{
    		printk("%s: request_mem_region failed!\n", __FUNCTION__);
    		//cdev_del(&blt_dev);
    		//unregister_chrdev_region(dev_no, 1);
    		//return -EBUSY;
    		err = -EBUSY;    		
    	}
    
    	if (err) {
    		w55fa93_blt_cleanup ();
    	}
    	else
    	{
        	init_waitqueue_head(&blt_wq);
        	init_waitqueue_head(&edma_wq);
        	printk("w55fa93 BLT driver has been initialized successfully!\n");    	    
    	}
	
    	return err;
}

static void __exit w55fa93_blt_exit(void)
{

	release_mem_region((unsigned long)W55FA93_VA_BLT, W55FA93_SZ_BLT);
	w55fa93_blt_cleanup();
}

module_init(w55fa93_blt_init);
module_exit(w55fa93_blt_exit);

MODULE_DESCRIPTION("HW BLT driver for W55FA93");
MODULE_LICENSE("GPL");

