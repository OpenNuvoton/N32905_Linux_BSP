/* linux/driver/input/w55fa93_keypad_1x4.c
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

#include <linux/init.h>
#include <linux/slab.h>
#include <asm/errno.h>
#include <asm/delay.h>
#include <linux/mm.h>
#include <linux/poll.h>
#include <linux/module.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <linux/cdev.h>
#include <mach/irqs.h>
#include <linux/interrupt.h>
#include  <linux/completion.h>
#include <mach/w55fa93_keypad.h>
#include <mach/w55fa93_reg.h>
#include <mach/regs-clock.h>
#undef BIT
#include <linux/input.h>
#define BIT(x)  (1UL<<((x)%BITS_PER_LONG))

#define KPD_IRQ_NUM             W55FA93_IRQ(2)  // nIRQ0
#define DEF_KPD_DELAY           HZ/20
#define KEY_COUNT			4

//#define W55FA93_DEBUG printk

static struct input_dev *w55fa93_keypad_input_dev;
static struct timer_list kpd_timer;
static char timer_active = 0;

static u32 old_key;
static u32 new_key;
static u32 open_cnt = 0;

u32 w55fa93_key_pressing = 0;
EXPORT_SYMBOL(w55fa93_key_pressing);

/*
	Flash Player reserved key code as below, please don't conflict with it.
    FI_KEY_LEFT     = 1,    // Four-way directional navigation
    FI_KEY_RIGHT    = 2,    // Four-way directional navigation
    FI_KEY_HOME     = 3,    // Send Key
    FI_KEY_END      = 4,    // End Key
	FI_KEY_INSERT   = 5,	// Insert Key
    FI_KEY_DELETE   = 6,	// Delete Key
	FI_KEY_BACKSPACE = 8,	// Backspace
    FI_KEY_SELECT   = 13,   // Select key
    FI_KEY_UP       = 14,   // Four-way directional navigation
    FI_KEY_DOWN     = 15,   // Four-way directional navigation
    FI_KEY_PAGEUP   = 16,   // Left soft key
    FI_KEY_PAGEDOWN = 17,   // Right soft key
    FI_KEY_FORWARD  = 18,   // Two-way directional navigation
    FI_KEY_BACKWARD = 26,   // Two-way directional navigation
	FI_KEY_ESCAPE	= 19,	// Escape Key
    FI_KEY_ENTER	= 31,   // Enter key
	FI_KEY_TAB		= 300,	// Tab Key
	FI_KEY_CAPS		= 302,	// Capslock Key
	FI_KEY_SHIFT    = 303,	// Shift Key
	FI_KEY_CTRL	    = 304	// Ctrl Key

	Other key could just follow PC's key code definition.
*/

#define ASC_KEY_UP			14 //38
#define ASC_KEY_DOWN 		15 //40
#define ASC_KEY_LEFT		1 //37
#define ASC_KEY_RIGHT		2 //39
#define ASC_KEY_ENTER		13
#define ASC_KEY_ESC			19 //27
#define ASC_KEY_VOLUMEUP	175
#define ASC_KEY_VOLUMEDOWN	174

const int key_map[KEY_COUNT] = {
	ASC_KEY_VOLUMEDOWN, ASC_KEY_ENTER, ASC_KEY_VOLUMEUP, ASC_KEY_ESC        
};

// arg = 0, from isr, 1 from timer.
static void read_key(unsigned long arg)
{
        u32 i;


        // ISR detect key press, disable irq, use timer to read following key press until released
        if (!timer_active) {
                writel(1 << KPD_IRQ_NUM, REG_AIC_MDCR);
		w55fa93_key_pressing = 1;
                disable_irq_nosync(KPD_IRQ_NUM);	//disable_irq(KPD_IRQ_NUM);
        }

        new_key = readl(REG_GPIOA_PIN) & 0x3C;

        if ((new_key & 0x3C) == 0x3C) { // all key released
//	  printk("->0\n");
                for (i = 0; i < KEY_COUNT; i++) {
                        if (old_key & (1 << i)) {
                                input_report_key(w55fa93_keypad_input_dev, key_map[i], 0);     //key up
                                input_sync(w55fa93_keypad_input_dev);
                        }
                }
                old_key = 0;
                del_timer(&kpd_timer);
                timer_active = 0;
                enable_irq(KPD_IRQ_NUM);
		w55fa93_key_pressing = 0;
                return;
        }
        
	new_key = (~(readl(REG_GPIOA_PIN)) & 0x3C) >> 2;

	for (i = 0; i < KEY_COUNT; i++) {
                if ((new_key ^ old_key) & (1 << i)) {// key state change
                        if (new_key & (1 << i)) {
                                //W55FA93_DEBUG("=== key down3 code[%d]\n",key_map[i]);
                                input_report_key(w55fa93_keypad_input_dev, key_map[i], 1);	//key down
                        } else {
                                input_report_key(w55fa93_keypad_input_dev, key_map[i], 0);	//key up
                                //W55FA93_DEBUG("=== key up3 code[%d]\n",key_map[i]);
                        }
                        input_sync(w55fa93_keypad_input_dev);
                }

        }

        old_key = new_key;

	 timer_active = 1;
        if ( arg == 0 )
                mod_timer(&kpd_timer, jiffies + DEF_KPD_DELAY*2); //*3); //### to avoid key too sensitive
        else
                mod_timer(&kpd_timer, jiffies + DEF_KPD_DELAY);        

        return;

}


static irqreturn_t w55fa93_kpd_irq(int irq, void *dev_id) {

        u32 src;

        src = readl(REG_IRQTGSRC0);

        read_key(0);

        // clear source
        writel(src & 0x000003C, REG_IRQTGSRC0);
        return IRQ_HANDLED;
}


int w55fa93_kpd_open(struct input_dev *dev) {

        if (open_cnt > 0) {
                goto exit;
        }

        new_key = old_key = 0;

        init_timer(&kpd_timer);
        kpd_timer.function = read_key;	/* timer handler */
        kpd_timer.data = 1;
        writel((1 << KPD_IRQ_NUM),  REG_AIC_SCCR); // force clear previous interrupt, if any.
        writel(readl(REG_IRQTGSRC0) & 0x000003C, REG_IRQTGSRC0); // clear source

        if (request_irq(KPD_IRQ_NUM, w55fa93_kpd_irq, IRQF_DISABLED, "Keypad",NULL) != 0) {
                printk("register the keypad_irq failed!\n");
                return -1;
        }

	//enable falling edge triggers
	writel((readl(REG_IRQENGPA)& ~(0x003C0000)) | 0x000003C, REG_IRQENGPA); 

exit:
        open_cnt++;
        return 0;
}



void w55fa93_kpd_close(struct input_dev *dev) {
        open_cnt--;
        if (open_cnt == 0) {
		//disable falling edge triggers
		writel((readl(REG_IRQENGPA)& ~(0x003C003C)), REG_IRQENGPA); 
                del_timer(&kpd_timer);
                free_irq(KPD_IRQ_NUM,NULL);
        }
        return;
}


static int __init w55fa93_kpd_reg(void) {

        int i, err;

        // init GPIO
        // PORTA[2-5]
        writel(readl(REG_GPIOA_OMD) & ~((1 << 2) | (1 << 3) | (1 << 4) | (1 << 5)), REG_GPIOA_OMD); // input
        writel(readl(REG_GPIOA_PUEN) | ((1 << 2) | (1 << 3) | (1 << 4) | (1 << 5)), REG_GPIOA_PUEN); // pull-up
        writel(readl(REG_IRQSRCGPA) & ~(0xFF0), REG_IRQSRCGPA); // GPA[2~5] as nIRQ0 source
	writel((readl(REG_IRQENGPA)& ~(0x003C0000)) | 0x000003C, REG_IRQENGPA); // falling edge trigger
	writel((readl(REG_AIC_SCR1)& ~(0x00C70000)) | 0x00470000, REG_AIC_SCR1);

	writel(readl(REG_DBNCECON) |0x71, REG_DBNCECON);
        if (!(w55fa93_keypad_input_dev = input_allocate_device())) {
                printk("W55FA93 Keypad Drvier Allocate Memory Failed!\n");
                err = -ENOMEM;
                goto fail;
        }

        w55fa93_keypad_input_dev->name = "W55FA93 Keypad";
        w55fa93_keypad_input_dev->phys = "input/event1";
        w55fa93_keypad_input_dev->id.bustype = BUS_HOST;
        w55fa93_keypad_input_dev->id.vendor  = 0x0005;
        w55fa93_keypad_input_dev->id.product = 0x0001;
        w55fa93_keypad_input_dev->id.version = 0x0100;

        w55fa93_keypad_input_dev->open    = w55fa93_kpd_open;
        w55fa93_keypad_input_dev->close   = w55fa93_kpd_close;

        w55fa93_keypad_input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_SYN) |  BIT(EV_REP);

        for (i = 0; i < KEY_MAX; i++)
                set_bit(i+1, w55fa93_keypad_input_dev->keybit);

        err = input_register_device(w55fa93_keypad_input_dev);
        if (err) {

                input_free_device(w55fa93_keypad_input_dev);
                return err;
        }

	// must set after input device register!!!
        w55fa93_keypad_input_dev->rep[REP_DELAY] = 200; //250ms
        w55fa93_keypad_input_dev->rep[REP_PERIOD] = 100; //ms

        printk("W55FA93 keypad driver has been initialized successfully!\n");

        return 0;

fail:
        input_free_device(w55fa93_keypad_input_dev);
        return err;
}

static void __exit w55fa93_kpd_exit(void) {
        free_irq(KPD_IRQ_NUM, NULL);
        input_unregister_device(w55fa93_keypad_input_dev);
}

module_init(w55fa93_kpd_reg);
module_exit(w55fa93_kpd_exit);

MODULE_DESCRIPTION("W55FA93 keypad driver");
MODULE_LICENSE("GPL");
