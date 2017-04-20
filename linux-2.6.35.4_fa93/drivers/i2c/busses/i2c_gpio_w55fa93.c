/* linux/drivers/i2c/busses/i2c-gpio-w55fa93.c
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
 *
 *   2008/11/10     First version.
 *	 2008/11/26     Add group check
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/platform_device.h>
#include <asm/io.h>

#include <mach/w55fa93_gpio.h>

#define NAME "w55fa93_gpio_i2c"

static int active_group_sda, active_group_scl;
static int active_pin_sda, active_pin_scl;

#define I2C_GPIO_DEBUG

static void w55fa93_gpioi2c_setscl(void *data, int state)
{
        w55fa93_gpio_set(active_group_scl, active_pin_scl, state);
}

static void w55fa93_gpioi2c_setsda(void *data, int state)
{
        if (state == 254)	//set to input mode
                w55fa93_gpio_set_input(active_group_sda, active_pin_sda);
        else if (state == 253)	//set to output mode
                w55fa93_gpio_set_output(active_group_sda, active_pin_sda);
        else
                w55fa93_gpio_set(active_group_sda, active_pin_sda, state);
}

static int w55fa93_gpioi2c_getscl(void *data)
{
        return w55fa93_gpio_get(active_group_scl, active_pin_scl);
}

static int w55fa93_gpioi2c_getsda(void *data)
{
        return w55fa93_gpio_get(active_group_sda, active_pin_sda);
}

/* ------------------------------------------------------------------------
 * Encapsulate the above functions in the correct operations structure.
 * This is only done when more than one hardware adapter is supported.
 */

static struct i2c_algo_bit_data w55fa93_gpioi2c_data = {
        .setsda		= w55fa93_gpioi2c_setsda,
        .setscl		= w55fa93_gpioi2c_setscl,
        .getsda		= w55fa93_gpioi2c_getsda,
        .getscl		= w55fa93_gpioi2c_getscl,
        .udelay		= 10,
        .timeout	= HZ,
};

static struct i2c_adapter w55fa93_gpioi2c_ops = {
        .owner			= THIS_MODULE,
        .class          = I2C_CLASS_HWMON | I2C_CLASS_SPD,
#ifdef CONFIG_I2C_BUS_W55FA93
	.nr 			= 1,
#else
        .nr 			= 0,
#endif        
        .algo_data	   	= &w55fa93_gpioi2c_data,
        .name			= "Nuvoton W55FA93 I2C GPIO",
};

static int w55fa93_gpioi2c_probe(struct platform_device *plat_dev)
{
        int ret;

	/* Configure GPIOs for SCL pin*/
#ifdef CONFIG_I2C_GPIO_W55FA93_SCL_GROUP_A
	active_group_scl = GPIO_GROUP_A;
	active_pin_scl = CONFIG_I2C_GPIO_W55FA93_SCL_PIN;	
	
#elif defined (CONFIG_I2C_GPIO_W55FA93_SCL_GROUP_B)
	active_group_scl = GPIO_GROUP_B;
	active_pin_scl = CONFIG_I2C_GPIO_W55FA93_SCL_PIN;	
	
#elif defined (CONFIG_I2C_GPIO_W55FA93_SCL_GROUP_C)
	active_group_scl = GPIO_GROUP_C;
	active_pin_scl = CONFIG_I2C_GPIO_W55FA93_SCL_PIN;	
	
#elif defined (CONFIG_I2C_GPIO_W55FA93_SCL_GROUP_D)
	active_group_scl = GPIO_GROUP_D;
	active_pin_scl = CONFIG_I2C_GPIO_W55FA93_SCL_PIN;	

#elif defined (CONFIG_I2C_GPIO_W55FA93_SCL_GROUP_E)
	active_group_scl = GPIO_GROUP_E;
	active_pin_scl = CONFIG_I2C_GPIO_W55FA93_SCL_PIN;
#endif	
	
	/* Configure GPIOs for SDA pin*/
#ifdef CONFIG_I2C_GPIO_W55FA93_SDA_GROUP_A	
	active_group_sda = GPIO_GROUP_A;
	active_pin_sda = CONFIG_I2C_GPIO_W55FA93_SDA_PIN;	
	
#elif defined (CONFIG_I2C_GPIO_W55FA93_SDA_GROUP_B)
	active_group_sda = GPIO_GROUP_B;
	active_pin_sda = CONFIG_I2C_GPIO_W55FA93_SDA_PIN;	
	
#elif defined (CONFIG_I2C_GPIO_W55FA93_SDA_GROUP_C)
	active_group_sda = GPIO_GROUP_C;
	active_pin_sda = CONFIG_I2C_GPIO_W55FA93_SDA_PIN;	
	
#elif defined (CONFIG_I2C_GPIO_W55FA93_SDA_GROUP_D)
	active_group_sda = GPIO_GROUP_D;
	active_pin_sda = CONFIG_I2C_GPIO_W55FA93_SDA_PIN;	

#elif defined (CONFIG_I2C_GPIO_W55FA93_SDA_GROUP_E)
	active_group_sda = GPIO_GROUP_E;
	active_pin_sda = CONFIG_I2C_GPIO_W55FA93_SDA_PIN;	
#endif	
	

#ifdef I2C_GPIO_DEBUG
	printk("i2c gpio scl=(%d)%d, sda=(%d)%d\n", active_group_scl, active_pin_scl, active_group_sda, active_pin_sda);
#endif
       
        //pull high the both pin
        w55fa93_gpioi2c_setscl(NULL, 1);
        w55fa93_gpioi2c_setsda(NULL, 1);

        ret = w55fa93_gpio_configure(active_group_scl, active_pin_scl);
        ret = w55fa93_gpio_configure(active_group_sda, active_pin_sda);

        if (!ret) {
                printk(KERN_ERR NAME ": adapter %s registration failed\n",
                       w55fa93_gpioi2c_ops.name);
                return -ENODEV;
        }

        if (i2c_bit_add_numbered_bus(&w55fa93_gpioi2c_ops) < 0) {
                printk(KERN_ERR NAME ": adapter %s registration failed\n",
                       w55fa93_gpioi2c_ops.name);
                return -ENODEV;
        }

        return 0;
}

static int  w55fa93_gpioi2c_remove(struct platform_device *plat_dev)
{
        i2c_del_adapter(&w55fa93_gpioi2c_ops);
        return 0;
}

static struct platform_driver w55fa93_gpioi2c_driver = {
	.probe		= w55fa93_gpioi2c_probe,
	.remove		= w55fa93_gpioi2c_remove,
	.driver		= {
		.name	= "w55fa93-gpioi2c",
		.owner	= THIS_MODULE,
	},
};

static int __init w55fa93_gpioi2c_init(void)
{
	return platform_driver_register(&w55fa93_gpioi2c_driver);
}

static void __exit w55fa93_gpioi2c_exit(void)
{
	platform_driver_unregister(&w55fa93_gpioi2c_driver);
}

module_init(w55fa93_gpioi2c_init);
module_exit(w55fa93_gpioi2c_exit);

MODULE_AUTHOR("nuvoton");
MODULE_DESCRIPTION("Nuvoton W55FA93 GPIO I2C Driver");
MODULE_LICENSE("GPL");
