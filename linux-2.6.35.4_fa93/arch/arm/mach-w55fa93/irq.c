/*
 * linux/arch/arm/mach-w55fa93/irq.c
 *
 * based on linux/arch/arm/plat-s3c24xx/irq.c by Ben Dooks
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/ptrace.h>
#include <linux/sysdev.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/w55fa93_reg.h>

static void w55fa93_irq_ack(unsigned int irq)
{
	__raw_writel(0x01, REG_AIC_EOSCR);
}

static void w55fa93_irq_mask(unsigned int irq)
{
	__raw_writel(1 << irq, REG_AIC_MDCR);
}

static void w55fa93_irq_unmask(unsigned int irq)
{
	__raw_writel(1 << irq, REG_AIC_MECR);
}

/*
 * w55fa93 startup function
 */
static unsigned int w55fa93_irq_startup(unsigned int irq)
{
	w55fa93_irq_unmask(irq);
	return 0;
}

/*
 * w55fa93 shutdown function
 */
static void w55fa93_irq_shutdown(unsigned int irq)
{
	w55fa93_irq_mask(irq);
}

static struct irq_chip w55fa93_irq_chip = {
	.ack		= w55fa93_irq_ack,
	.mask		= w55fa93_irq_mask,
	.unmask		= w55fa93_irq_unmask,
	.startup	= w55fa93_irq_startup,
	.shutdown	= w55fa93_irq_shutdown,
};

/*
 * GPIO IRQs for GPIO 3
 */	
#ifdef CONFIG_GPIO_W55FA93
#define GPIO_OFFSET 0x20
#define GPIO_PORT_NUM 5
 
static unsigned short FType[GPIO_PORT_NUM];
static unsigned short RType[GPIO_PORT_NUM];

static void w55fa93_irq_gpio_ack(unsigned int irq)
{
	__raw_writel(0x01, REG_AIC_EOSCR);
}

static void w55fa93_irq_gpio_mask(unsigned int irq)
{
	unsigned int port,num;
	port =(irq-IRQ_GPIO_START)/GPIO_OFFSET;
	num  =(irq-IRQ_GPIO_START)%GPIO_OFFSET;
	__raw_writel(__raw_readl(REG_IRQENGPA + (port << 2)) &~((0x1 << num) | (0x1 << (num + 16))),REG_IRQENGPA + (port << 2));
}

static void w55fa93_irq_gpio_unmask(unsigned int irq)
{
	unsigned int port,num,tmp;
	port =(irq-IRQ_GPIO_START)/GPIO_OFFSET;
	num  =(irq-IRQ_GPIO_START)%GPIO_OFFSET;
	
	tmp = (RType[port] & (0x1<<num)) << 16;
	tmp |= (FType[port] & (0x1<<num));
	__raw_writel(__raw_readl(REG_IRQENGPA + (port << 2)) | tmp,REG_IRQENGPA + (port << 2));
}

static int w55fa93_irq_gpio_set_type(unsigned int irq, unsigned int type)
{
	unsigned int port,num;
	__raw_writel(__raw_readl(REG_DBNCECON) |0x08, REG_DBNCECON);
	
	port =(irq-IRQ_GPIO_START)/GPIO_OFFSET;
	num  =(irq-IRQ_GPIO_START)%GPIO_OFFSET;

	__raw_writel(__raw_readl(REG_IRQSRCGPA + (port << 2)) | (0x3 << (num << 1)),REG_IRQSRCGPA + (port << 2));

	if (type & IRQ_TYPE_EDGE_RISING) 
	{
		__raw_writel(__raw_readl(REG_IRQENGPA + (port << 2)) | (0x1 << (num + 16)),REG_IRQENGPA + (port << 2));
		RType[port] |= (0x1<<num);
	}
	else
	{
		__raw_writel(__raw_readl(REG_IRQENGPA + (port << 2)) &~ (0x1 << (num + 16)),REG_IRQENGPA + (port << 2));	
		RType[port] &= ~(0x1<<num);
	}
		
	
	if(type & IRQ_TYPE_EDGE_FALLING)
	{
		__raw_writel(__raw_readl(REG_IRQENGPA + (port << 2)) | (0x1 << num),REG_IRQENGPA + (port << 2));
		FType[port] |= (0x1<<num);
	}
	else
	{
		__raw_writel(__raw_readl(REG_IRQENGPA + (port << 2)) &~ (0x1 << num),REG_IRQENGPA + (port << 2));	
		FType[port] &= ~(0x1<<num);
	}
	
	return 0;
}

static struct irq_chip w55fa93_irq_gpio_chip = {
	.name		= "GPIO-IRQ",
	.ack		= w55fa93_irq_gpio_ack,
	.mask		= w55fa93_irq_gpio_mask,
	.unmask		= w55fa93_irq_gpio_unmask,
	.set_type	= w55fa93_irq_gpio_set_type,
};

static irqreturn_t w55fa93_gpio_irq(int irq, void *dev_id) 
{
	unsigned int i,j,isr,check_isr,tmp;
		
	for(i=0; i<GPIO_PORT_NUM; i++)
	{
		isr = __raw_readl(REG_IRQTGSRC0 + (i/2)*4);
		tmp = RType[i] | FType[i];
		
		if((isr != 0) && (tmp != 0))
		{
			//printk("w55fa93_gpio_irq!0x%x 0x%x\n",irq ,isr);
			check_isr = isr >> (16*(i%2));
			for(j=0; j<16; j++)
			{
				if((tmp & (1<<j)) & (check_isr & 0xFFFF))
				{
					__raw_writel((1<<(j+16*(i%2))), REG_IRQTGSRC0 + (i/2)*4);
					generic_handle_irq(IRQ_GPIO_START + i*0x20+j);
				}
			}
		}
	}
	
	return IRQ_HANDLED;
}

static int __init w55fa93_init_irq_gpio(void)
{
	int irqno;

        if (request_irq(IRQ_GPIO3, w55fa93_gpio_irq, IRQF_DISABLED, "gpio",NULL) != 0) {
                printk("register the gpio_irq failed!\n");
                return -1;
        }	

	for (irqno = IRQ_GPIO_START; irqno < IRQ_GPIO_END; irqno++) {
		set_irq_chip(irqno, &w55fa93_irq_gpio_chip);
		set_irq_handler(irqno, handle_level_irq);
		set_irq_flags(irqno, IRQF_VALID);		
	}

	return 0;
}
#endif

void __init w55fa93_init_irq(void)
{
	int irqno;

	__raw_writel(0xFFFFFFFF, REG_AIC_MDCR);

	for (irqno = IRQ_WDT; irqno < NR_IRQS; irqno++) {
		set_irq_chip(irqno, &w55fa93_irq_chip);
		set_irq_handler(irqno, handle_level_irq);
		set_irq_flags(irqno, IRQF_VALID);
	}

#ifdef CONFIG_GPIO_W55FA93
    /* Set GPIO IRQ */
	w55fa93_init_irq_gpio();
#endif
}
