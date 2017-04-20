/* linux/include/linux/w55fa93.h
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
 *   2008/11/10     First version
 */

#include <mach/w55fa93_reg.h>


/* GPIO group definition */
#define GPIO_GROUP_A 0
#define GPIO_GROUP_B 1
#define GPIO_GROUP_C 2
#define GPIO_GROUP_D 3
#define GPIO_GROUP_E 4

/* GPIO register offset definition */
static int __iomem * gpio_reg_dir[7] = { REG_GPIOA_OMD, REG_GPIOB_OMD, REG_GPIOC_OMD, REG_GPIOD_OMD, REG_GPIOE_OMD};
static int __iomem * gpio_reg_out[7] = { REG_GPIOA_DOUT, REG_GPIOB_DOUT, REG_GPIOC_DOUT, REG_GPIOD_DOUT, REG_GPIOE_DOUT};
static int __iomem * gpio_reg_in[7] = { REG_GPIOA_PIN, REG_GPIOB_PIN, REG_GPIOC_PIN, REG_GPIOD_PIN, REG_GPIOE_PIN};

/* returns the value of the GPIO pin */
static inline int w55fa93_gpio_get(int group, int num) 
{
	return readl(gpio_reg_in[group]) & (1 << num) ? 1:0;
}

/* set direction of pin to input mode */
static inline void w55fa93_gpio_set_input(int group, int num)
{
	writel (readl(gpio_reg_dir[group]) & ~(1 << num), gpio_reg_dir[group]); 
}

/* set direction of pin to output mode */
static inline void w55fa93_gpio_set_output(int group, int num)
{
	writel (readl(gpio_reg_dir[group]) | (1 << num), gpio_reg_dir[group]); 
}

/* drive the GPIO signal to state */
static inline void w55fa93_gpio_set(int group, int num, int state) 
{	
	if(state)
		writel (readl(gpio_reg_out[group]) | (1 << num), gpio_reg_out[group]); 		//set high			
	else
		writel (readl(gpio_reg_out[group]) & ~(1 << num), gpio_reg_out[group]); 	//set low
}

/* set share pin and direction of gpios */
static inline int w55fa93_gpio_configure(int group, int num)
{
	printk("w55fa93_gpio_configure()-%d,%d\n", group, num);
	if(num > 16)
		goto err;
		
	switch(group)
	{
		case GPIO_GROUP_A:	
			if(num > 11)
				goto err;
			writel (readl(REG_GPAFUN) &~ (0x3 << (num<<1)), REG_GPAFUN);		//set share pin funcion				
			break;
		
		case GPIO_GROUP_B:
			writel (readl(REG_GPBFUN) &~ (0x3 << (num<<1)), REG_GPBFUN);	//set share pin funcion			
			break;

		case GPIO_GROUP_C:
			writel (readl(REG_GPCFUN) &~ (0x3 << (num<<1)), REG_GPCFUN);	//set share pin funcion			
			break;
				
		case GPIO_GROUP_D:			
			writel (readl(REG_GPDFUN) &~ (0x3 << (num<<1)), REG_GPDFUN);		//set share pin funcion			
			break;
		
		case GPIO_GROUP_E:
			if(num > 11)
				goto err;
			writel (readl(REG_GPEFUN) &~ (0x3 << (num<<1)), REG_GPEFUN);	//set share pin funcion			
			break;
		default:
			break;
	}
	w55fa93_gpio_set_output(group, num);
	return 1;
	
err:
	return 0;
}
