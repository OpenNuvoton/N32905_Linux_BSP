/*
 * linux/arch/arm/mach-w55fa93/mach-w55fa93.c
 *
 * Based on mach-s3c2410/mach-smdk2410.c by Jonas Dietsche
 *
 * Copyright (C) 2008 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation;version 2 of the License.
 *   history:
 *     Wang Qiang (rurality.linux@gmail.com) add LCD support
 *
 */

#include <linux/platform_device.h>
#include <linux/delay.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>

#include <mach/map.h>
#include <mach/fb.h>
#include <mach/irqs.h>
#include <mach/serial.h>
#include <mach/w55fa93_reg.h>
#include "cpu.h"
#include <linux/i2c.h> /* sw add */

//#define DBG
#ifdef DBG
#define ENTER()		printk("ENTER : %s  : %s\n",__FILE__, __FUNCTION__)
#define LEAVE()		printk("LEAVE : %s  : %s\n",__FILE__, __FUNCTION__)
#define ERRLEAVE()	printk("ERRLEAVE : %s  : %s\n",__FILE__, __FUNCTION__)
#else
#define ENTER(...)
#define LEAVE(...)	
#define ERRLEAVE(...)	
#endif

static struct w55fa93_uartcfg w55fa93_uartcfgs[] = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = 0,
		.ulcon	     = 0,
		.ufcon	     = 0,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = 0,
		.ulcon	     = 0,
		.ufcon	     = 0,
	}
};

/* I2C clients */ /* sw add */
static struct i2c_board_info __initdata w55fa93_i2c_clients[] = {

        {
                I2C_BOARD_INFO("nau8822", 0x1a),
        },
        
#ifdef CONFIG_SND_SOC_W55FA93_SPU
        {
                I2C_BOARD_INFO("w55fa93_dac", 0x7f),     //            
        },
#endif       
#ifdef CONFIG_SND_SOC_W55FA93_ADC
        {
                I2C_BOARD_INFO("w55fa93_adc_i2c", 0x6f),     //            
        },
#endif       
#if defined(CONFIG_SENSOR_OV7670) || defined(CONFIG_SENSOR_OV7670_DEV1)
		{
                I2C_BOARD_INFO("ov7670", 0x21),
        },
#endif
#if defined(CONFIG_SENSOR_OV9660) || defined(CONFIG_SENSOR_OV9660_DEV1)
		{
                I2C_BOARD_INFO("ov9660", 0x30),
        },
#endif
#if defined(CONFIG_SENSOR_OV7725) || defined(CONFIG_SENSOR_OV7725_DEV1)
		{
                I2C_BOARD_INFO("ov7725", 0x21),
        },
#endif 
#if defined(CONFIG_SENSOR_OV7740) || defined(CONFIG_SENSOR_OV7740_DEV1)
	{
		I2C_BOARD_INFO("ov7740", 0x21),
	},
#endif
#if defined(CONFIG_SENSOR_NT99050) || defined(CONFIG_SENSOR_NT99050_DEV1)
		{
                I2C_BOARD_INFO("nt99050", 0x21),
        },
#endif 
#if defined(CONFIG_SENSOR_NT99140) || defined(CONFIG_SENSOR_NT99140_DEV1)
		{
                I2C_BOARD_INFO("nt99140", 0x2a),
        },
#endif
#if defined(CONFIG_SENSOR_NT99141) || defined(CONFIG_SENSOR_NT99141_DEV1)
		{
                I2C_BOARD_INFO("nt99141", 0x2a),
        },
#endif
#if defined(CONFIG_SENSOR_OV7675) || defined(CONFIG_SENSOR_OV7675_DEV1)
		{
                I2C_BOARD_INFO("ov7675", 0x21),
        },
#endif
#if defined(CONFIG_SENSOR_GC0308) || defined(CONFIG_SENSOR_GC0308_DEV1)
		{
                I2C_BOARD_INFO("gc0308", 0x21),
        },
#endif
#if defined(CONFIG_SENSOR_TW9912) || defined(CONFIG_SENSOR_TW9912_DEV1)
		{
                I2C_BOARD_INFO("tw9912", 0x44),
        },
#endif
#if defined(CONFIG_SENSOR_TW9900) || defined(CONFIG_SENSOR_TW9900_DEV1)
		{
                I2C_BOARD_INFO("tw9900", 0x45),
        },
#endif
#if defined(CONFIG_SENSOR_TVP5150) || defined(CONFIG_SENSOR_TVP5150_DEV1)
		{
                I2C_BOARD_INFO("tvp5150", 0x5D),
        },
#endif
#ifdef CONFIG_EEPROM_AT24
        {
                I2C_BOARD_INFO("at24", 0x50),
                .type="24c16",
        },        
#endif
#ifdef CONFIG_I2C_TS
        {
                I2C_BOARD_INFO("tsc2007", 0x48),
                .type = "tsc2007",
                .platform_data = &nuc900_tsc2007_data,
                .irq = IRQ_GROUP0,
        },
#endif 
#ifdef CONFIG_RTC_DRV_PCF8563        
        {
		I2C_BOARD_INFO("rtc-pcf8563", 0x51),
		.type = "pcf8563",
        },
#endif        
#ifdef CONFIG_RTC_DRV_HT1382
        {
                I2C_BOARD_INFO("rtc-ht1382", 0x68),
                .type = "ht1382",
        },        
#endif
};

#if defined (CONFIG_I2C_BUS_W55FA93) && defined (CONFIG_I2C_GPIO_W55FA93)
static struct i2c_board_info __initdata w55fa93_i2c_port1_clients[] = {


};
#endif

extern void w55fa93_poweroff(void);
static void __init mach_w55fa93_map_io(void)
{
	w55fa93_map_io();
	w55fa93_init_clocks();
	w55fa93_init_uarts(w55fa93_uartcfgs, ARRAY_SIZE(w55fa93_uartcfgs));
	pm_power_off = w55fa93_poweroff;
}

static void __init w55fa93_init(void)
{
	w55fa93_board_init();
	ENTER();
	i2c_register_board_info(0, w55fa93_i2c_clients, sizeof(w55fa93_i2c_clients)/sizeof(struct i2c_board_info)); /* sw add */
#if defined (CONFIG_I2C_BUS_W55FA93) && defined (CONFIG_I2C_GPIO_W55FA93)	
	i2c_register_board_info(1, w55fa93_i2c_port1_clients, sizeof(w55fa93_i2c_port1_clients)/sizeof(struct i2c_board_info));
#endif

	LEAVE();	
}

MACHINE_START(W55FA93, "W55FA93")
	.phys_io	= W55FA93_PA_UART,
	.io_pg_offst	= (((u32)W55FA93_VA_UART) >> 18) & 0xfffc,
#ifdef CONFIG_INITRAMFS_ROOT_UID
	.boot_params	= 0x0,
#else
	.boot_params	= 0x100,
#endif
	.map_io		= mach_w55fa93_map_io,
	.init_irq	= w55fa93_init_irq,
	.init_machine	= w55fa93_init,
	.timer		= &w55fa93_timer,
MACHINE_END
