/* linux/driver/input/w55fa93_sysmgr.c
 *
 * Copyright (c) 2010 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Changelog:
 *
 */
 /* This is a skeleton of system manager driver. A real one is highly system dependent, so not implemented here.*/

#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <mach/w55fa93_sysmgr.h>
#include <mach/w55fa93_reg.h>

#define SYSMGR_DRIVER_NAME	"w55fa93-sysmgr"
#define MAX_SYSMGR_MINORS	4

#define SYSMGR_DYNAMIC_ALLOC_DEVICE_NUM
#define SYSMGR_AUTOMATIC_CREATE_DEVICE_FILE

#ifdef SYSMGR_AUTOMATIC_CREATE_DEVICE_FILE
#define SYSMGR_DEVICE_FILE_NAME	"sysmgr"
#endif

static int major_no = SYSMGR_MAJOR;
module_param (major_no, int, 0644);
static int minor_no = SYSMGR_MINOR0;
module_param (minor_no, int, 0644);
static int dev_no_created = 0;
static int sysmgr_dev_created = 0;

#ifdef SYSMGR_AUTOMATIC_CREATE_DEVICE_FILE
static struct class *sysmgr_class = NULL;
static struct device *sysmgr_device = NULL;
#endif  // #ifdef SYSMGR_AUTOMATIC_CREATE_DEVICE_FILE

char w55fa93_nand_dev = '-';
char w55fa93_sd_dev = '-';
EXPORT_SYMBOL(w55fa93_nand_dev);
EXPORT_SYMBOL(w55fa93_sd_dev);

typedef struct
{
	// no spmaphore protect for data here. seems not necessary for now. 
	u32 num;
	u32 opened;
	u32 status;
} sysmgr_data;

static DECLARE_WAIT_QUEUE_HEAD(sysmgr_q);
static struct cdev sysmgr_dev;
static sysmgr_data sdata[MAX_SYSMGR_MINORS];
static u32 usbd_state, audio_state, usbh_state;

#if defined(CONFIG_RTC_DRV_W55FA93)
static struct clk *rtc_clk;
#endif

#ifdef CONFIG_TOUCHSCREEN_W55FA93
extern void enable_touch(void);
extern void disable_touch(void);
#endif

void sysmgr_report(unsigned status)
{
	int i;

	for (i = 0; i < MAX_SYSMGR_MINORS; i++) {
		// 2013/10/14, update status for all devices from booting
		//if (sdata[i].opened != 0) {
			if (status == SYSMGR_STATUS_SD_INSERT)
				if (sdata[i].status & SYSMGR_STATUS_SD_REMOVE)
					sdata[i].status &= ~SYSMGR_STATUS_SD_REMOVE;
				else
					sdata[i].status |= SYSMGR_STATUS_SD_INSERT;
			else if (status == SYSMGR_STATUS_SD_REMOVE)
				if (sdata[i].status & SYSMGR_STATUS_SD_INSERT)
					sdata[i].status &= ~SYSMGR_STATUS_SD_INSERT;
				else
					sdata[i].status |= SYSMGR_STATUS_SD_REMOVE;
			else if ((status == SYSMGR_STATUS_USBD_PLUG) || (status == SYSMGR_STATUS_USBD_CONNECT_PC))
				if (sdata[i].status & SYSMGR_STATUS_USBD_UNPLUG)
					sdata[i].status &= ~SYSMGR_STATUS_USBD_UNPLUG;
				else
					sdata[i].status |= status;
			else if (status == SYSMGR_STATUS_USBD_UNPLUG) {
				if (sdata[i].status & SYSMGR_STATUS_USBD_PLUG)
					sdata[i].status &= ~SYSMGR_STATUS_USBD_PLUG;
				else
					sdata[i].status |= SYSMGR_STATUS_USBD_UNPLUG;
				if (sdata[i].status & SYSMGR_STATUS_USBD_CONNECT_PC)
					sdata[i].status &= ~SYSMGR_STATUS_USBD_CONNECT_PC;
			}
			else if (status == SYSMGR_STATUS_USBH_PLUG)
				if (sdata[i].status & SYSMGR_STATUS_USBH_UNPLUG)
					sdata[i].status &= ~SYSMGR_STATUS_USBH_UNPLUG;
				else
					sdata[i].status |= SYSMGR_STATUS_USBH_PLUG;
			else if (status == SYSMGR_STATUS_USBH_UNPLUG)
				if (sdata[i].status & SYSMGR_STATUS_USBH_PLUG)
					sdata[i].status &= ~SYSMGR_STATUS_USBH_PLUG;
				else
					sdata[i].status |= SYSMGR_STATUS_USBH_UNPLUG;
			else if (status == SYSMGR_STATUS_NAND_INSERT)
				if (sdata[i].status & SYSMGR_STATUS_NAND_REMOVE)
					sdata[i].status &= ~SYSMGR_STATUS_NAND_REMOVE;
				else
					sdata[i].status |= SYSMGR_STATUS_NAND_INSERT;
			else if (status == SYSMGR_STATUS_NAND_REMOVE)
				if (sdata[i].status & SYSMGR_STATUS_NAND_INSERT)
					sdata[i].status &= ~SYSMGR_STATUS_NAND_INSERT;
				else
					sdata[i].status |= SYSMGR_STATUS_NAND_REMOVE;
			else if (status == SYSMGR_STATUS_DISPLAY_LCD)
				if (sdata[i].status & SYSMGR_STATUS_DISPLAY_TV)
					sdata[i].status &= ~SYSMGR_STATUS_DISPLAY_TV;
				else
					sdata[i].status |= SYSMGR_STATUS_DISPLAY_LCD;
			else if (status == SYSMGR_STATUS_DISPLAY_TV)
				if (sdata[i].status & SYSMGR_STATUS_DISPLAY_LCD)
					sdata[i].status &= ~SYSMGR_STATUS_DISPLAY_LCD;
				else
					sdata[i].status |= SYSMGR_STATUS_DISPLAY_TV;
			else if ((status >= SYSMGR_STATUS_NORMAL) && (status <= SYSMGR_STATUS_POWER_DOWN)) {
				if (!(sdata[i].status & (SYSMGR_STATUS_RTC_POWER_OFF|SYSMGR_STATUS_POWER_OFF)))
					sdata[i].status = (sdata[i].status & ~(0x7F000000)) | status;
			}
			else if ((status == SYSMGR_STATUS_RTC_POWER_OFF) || (status == SYSMGR_STATUS_POWER_OFF))
				sdata[i].status = (sdata[i].status & ~(0x7F000000)) | status;
			else
				sdata[i].status |= status;
		//}
	}

	if (status == SYSMGR_STATUS_USBD_PLUG)
		usbd_state = 1;
	else if (status == SYSMGR_STATUS_USBD_CONNECT_PC)
		usbd_state = 2;
	else if (status == SYSMGR_STATUS_USBD_UNPLUG)
		usbd_state = 0;
	else if (status == SYSMGR_STATUS_USBH_PLUG)
		usbh_state = 1;
	else if (status == SYSMGR_STATUS_USBH_UNPLUG)
		usbh_state = 0;
	else if (status == SYSMGR_STATUS_AUDIO_OPEN)
		audio_state = 1;
	else if (status == SYSMGR_STATUS_AUDIO_CLOSE)
		audio_state = 0;

	wake_up_interruptible(&sysmgr_q);
}
EXPORT_SYMBOL(sysmgr_report);

int w55fa93_sysmgr_open(struct inode* i,struct file* file)
{
	sdata[MINOR(i->i_rdev)].opened++;
	file->private_data = &sdata[MINOR(i->i_rdev)];

	return 0;
}

int w55fa93_sysmgr_close(struct inode* i,struct file* file)
{
	int num;

	sdata[(num = MINOR(i->i_rdev))].opened--;
	// 2013/10/14, update status for all devices from booting
	//if (sdata[num].opened == 0)
	//	sdata[num].status = 0;

	return 0;
}

ssize_t w55fa93_sysmgr_read(struct file *file, char *buff, size_t read_mode, loff_t *offp)
{
	sysmgr_data *data = (sysmgr_data *)file->private_data;

	while (data->status == 0) {
		if (wait_event_interruptible(sysmgr_q, (data->status != 0))) {
			return -ERESTARTSYS;
		}
	}
	copy_to_user(buff, &data->status, 4);
	data->status = 0;

	return 4;
}

ssize_t w55fa93_sysmgr_write(struct file *file, const char *buf, size_t count, loff_t *f_pos)
{
	unsigned int cmd;

	if (copy_from_user(&cmd, buf, 4))
		return -EFAULT;

	return 4;
}

static int w55fa93_sysmgr_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned int cmd_data, state;
	int ret = 0;
#if defined(CONFIG_RTC_DRV_W55FA93)
	int rtc_time_out;
#endif

	switch (cmd) {
	case SYSMGR_IOC_SET_POWER_STATE:
		if (copy_from_user((void *)&cmd_data, (void *)arg, sizeof(unsigned int))) {
			ret = -EFAULT;
			break;
		}
	 	if (cmd_data == SYSMGR_CMD_NORMAL) {
#ifdef CONFIG_TOUCHSCREEN_W55FA93
			enable_touch();
#endif
			sysmgr_report(SYSMGR_STATUS_NORMAL);
		}
	 	else if (cmd_data == SYSMGR_CMD_DISPLAY_OFF) {
#ifdef CONFIG_TOUCHSCREEN_W55FA93
			disable_touch();
#endif
			sysmgr_report(SYSMGR_STATUS_DISPLAY_OFF);
		}
	 	else if (cmd_data == SYSMGR_CMD_IDLE)
			sysmgr_report(SYSMGR_STATUS_IDLE);
		else if (cmd_data == SYSMGR_CMD_MEMORY_IDLE)
			sysmgr_report(SYSMGR_STATUS_MEMORY_IDLE);
		else if (cmd_data == SYSMGR_CMD_POWER_DOWN)
			sysmgr_report(SYSMGR_STATUS_POWER_DOWN);
		else if (cmd_data == SYSMGR_CMD_RTC_POWER_OFF)
			sysmgr_report(SYSMGR_STATUS_RTC_POWER_OFF);
		else if (cmd_data == SYSMGR_CMD_POWER_OFF)
			sysmgr_report(SYSMGR_STATUS_POWER_OFF);
		else
			ret = -ENOIOCTLCMD;
		break;

	case SYSMGR_IOC_GET_USBD_STATE:
		if (copy_to_user((void *)arg, (void *)&usbd_state, sizeof(unsigned int))) {
			ret = -EFAULT;
			break;
		}
		break;

	case SYSMGR_IOC_GET_USBH_STATE:
		if (copy_to_user((void *)arg, (void *)&usbh_state, sizeof(unsigned int))) {
			ret = -EFAULT;
			break;
		}
		break;

	case SYSMGR_IOC_GET_AUDIO_STATE:
		if (copy_to_user((void *)arg, (void *)&audio_state, sizeof(unsigned int))) {
			ret = -EFAULT;
			break;
		}
		break;

	case SYSMGR_IOC_GET_NAND_DEV:
		//printk("NAND is %c\n", w55fa93_nand_dev);
		if (copy_to_user((char *)arg, (void *)&w55fa93_nand_dev, sizeof(char))) {
			ret = -EFAULT;
			break;
		}
		break;

	case SYSMGR_IOC_GET_SD_DEV:
		//printk("SD is %c\n", w55fa93_sd_dev);
		if (copy_to_user((char *)arg, (void *)&w55fa93_sd_dev, sizeof(char))) {
			ret = -EFAULT;
			break;
		}
		break;

	case SYSMGR_IOC_GET_POWER_KEY:
#if defined(CONFIG_RTC_DRV_W55FA93)
		// enable RTC clcok
		clk_enable(rtc_clk);

		while(1) {	
			rtc_time_out = 0;
			// enable long time press power disable
			if ((readl(REG_RTC_AER) & 0x10000) == 0x0) {
				// set RTC register access enable password
				writel(0xA965, REG_RTC_AER);
				// make sure RTC register read/write enable
				while ((readl(REG_RTC_AER) & 0x10000) == 0x0) ;

				rtc_time_out++;
				if (rtc_time_out > 0xFFFFFF) {
					printk("RTC Access Eanble Fail\n");
					break;
				}
					
				// FA93 does not have REG_RTC_REG_FLAG	
				//rtc_wait_ready();

				if ((readl(REG_RTC_AER) & 0x10000) == 0x10000) 
					break;			
			}
			else
				break;
		}

		if ((readl(REG_RTC_PWRON) & BIT7))
			state = 1;
		else
			state = 0;

		// disable RTC clcok
		clk_disable(rtc_clk);
#endif
		if (copy_to_user((void *)arg, (void *)&state, sizeof(unsigned int))) {
			ret = -EFAULT;
			break;
		}
		break;

	default:
		ret = -ENOIOCTLCMD;
	}

	return ret;
}

static unsigned int w55fa93_sysmgr_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;

	sysmgr_data *data = (sysmgr_data *)file->private_data;
	poll_wait(file, &sysmgr_q, wait);
	if (data->status != 0)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

struct file_operations w55fa93_sysmgr_fops = {
owner:
	THIS_MODULE,
open:
	w55fa93_sysmgr_open,
read:
	w55fa93_sysmgr_read,
write:
	w55fa93_sysmgr_write,
ioctl:
	w55fa93_sysmgr_ioctl,
poll:
	w55fa93_sysmgr_poll,
release:
	w55fa93_sysmgr_close,
};


static void sysmgr_cleanup(void)
{
	dev_t dev_no = MKDEV(major_no, minor_no);
	int i;

#ifdef SYSMGR_AUTOMATIC_CREATE_DEVICE_FILE
	if (sysmgr_device) {
		for (i = 1; i < MAX_SYSMGR_MINORS; i++)
			device_destroy(sysmgr_class, dev_no + i);
		sysmgr_device = NULL;
	}
	if (sysmgr_class) {
		class_destroy(sysmgr_class);
		sysmgr_class = NULL;
	}
#endif  // #ifdef SYSMGR_AUTOMATIC_CREATE_DEVICE_FILE

	if (sysmgr_dev_created) {
		cdev_del(&sysmgr_dev);
		sysmgr_dev_created = 0;
	}

	if (dev_no_created) {
		unregister_chrdev_region(dev_no, MAX_SYSMGR_MINORS);
		dev_no_created = 0;
	}
}

static int __init w55fa93_sysmgr_reg(void)
{
	int err, i;

	do {
#ifdef SYSMGR_DYNAMIC_ALLOC_DEVICE_NUM
		dev_t dev_no;
		err = alloc_chrdev_region(&dev_no, minor_no, MAX_SYSMGR_MINORS, SYSMGR_DRIVER_NAME);
		if (err) {
			printk(KERN_ERR "[%s] alloc_chrdev_region failed\n", __func__);
			break;
		}
		major_no = MAJOR(dev_no);
#else
		dev_t dev_no = MKDEV(SYSMGR_MAJOR, SYSMGR_MINOR0);

		err = register_chrdev_region(dev_no, MAX_SYSMGR_MINORS, SYSMGR_DRIVER_NAME);
		if (err) {
			printk(KERN_ERR "[%s] register_chrdev_region failed\n", __func__);
			break;
		}
#endif  // #ifdef SYSMGR_DYNAMIC_ALLOC_DEVICE_NUM
		dev_no_created = 1;

		cdev_init(&sysmgr_dev, &w55fa93_sysmgr_fops);
		sysmgr_dev.owner = THIS_MODULE;
		err = cdev_add(&sysmgr_dev, dev_no, MAX_SYSMGR_MINORS);
		if (err) {
			printk(KERN_ERR "[%s] cdev_add failed\n", __func__);
			break;
		}
		sysmgr_dev_created = 1;

#ifdef SYSMGR_AUTOMATIC_CREATE_DEVICE_FILE
		sysmgr_class = class_create(THIS_MODULE, SYSMGR_DRIVER_NAME);
		if (IS_ERR(sysmgr_class)) {
			printk(KERN_ERR "[%s] class_create failed\n", __func__);
			err = PTR_ERR(sysmgr_class);
			sysmgr_class = NULL;
			break;
		}

		// for backward compatible
		sysmgr_device = device_create(sysmgr_class, NULL, MKDEV(major_no, minor_no), 
				NULL, SYSMGR_DEVICE_FILE_NAME);
		for (i = 1; i < MAX_SYSMGR_MINORS; i++) {
			sysmgr_device = device_create(sysmgr_class, NULL, MKDEV(major_no, minor_no + i), 
					NULL, SYSMGR_DEVICE_FILE_NAME"%d", i);
		}
		if (IS_ERR(sysmgr_device)) {
			printk(KERN_ERR "[%s] device_create failed\n", __func__);
			err = PTR_ERR(sysmgr_device);
			sysmgr_device = NULL;
			break;
		}
#endif  // #ifdef SYSMGR_AUTOMATIC_CREATE_DEVICE_FILE
	} while (0);

	if (err) {
		sysmgr_cleanup();
	}

	memset(sdata, 0, sizeof(sdata));
	for (i = 0; i < MAX_SYSMGR_MINORS; i++)
		sdata[i].num = i;

#if defined(CONFIG_RTC_DRV_W55FA93)
	// get RTC clcok
	rtc_clk = clk_get(NULL, "rtc");
	if (IS_ERR(rtc_clk)) {
		printk(KERN_ERR "[%s] RTC clk_get failed\n", __func__);
	}
#endif

	init_waitqueue_head(&sysmgr_q);
	printk("w55fa93 SysMgr driver has been initialized successfully!\n");

	return err;
}

static void __exit w55fa93_sysmgr_exit(void)
{
	sysmgr_cleanup();
}

module_init(w55fa93_sysmgr_reg);
module_exit(w55fa93_sysmgr_exit);

MODULE_DESCRIPTION("W55FA93 SysMgr driver");
MODULE_LICENSE("GPL");

