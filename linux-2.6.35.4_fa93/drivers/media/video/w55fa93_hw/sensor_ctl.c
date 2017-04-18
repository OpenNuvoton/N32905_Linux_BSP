#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <asm/io.h>

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/smp_lock.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

#if 0
#include <linux/videodev.h>
#else
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#endif 

#include <linux/jiffies.h>
#if 0
#include <asm/arch/videoin.h>
#include <asm/arch/DrvVideoin.h>
#include <asm/arch/w55fa95_reg.h>
#include <asm/arch/fb.h>
#include <asm/arch/w55fa95_fb.h>
#include <asm/arch/w55fa95_gpio.h>
#else
#include <mach/w55fa93_reg.h>
#include <mach/fb.h>
#include <mach/w55fa93_fb.h>
#include <mach/videoin.h>
//#include <mach/DrvVideoin.h>
#include "DrvVideoin.h"
#include <mach/videodev_ex.h>
#include <mach/w55fa93_gpio.h>
#endif

#include <linux/moduleparam.h>
#include <asm/io.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/i2c-id.h>
#include <linux/i2c-dev.h>

#include "videoinpriv.h"
#include "DrvI2C.h"	/* */
#include "DrvVideoin.h"

#include <media/v4l2-chip-ident.h>
#include <media/v4l2-common.h>
#include <media/soc_camera.h>

#include "videoinpriv.h"


  #if defined(CONFIG_SENSOR_RESET_GROUP_A)
static int 	group_reset = GPIO_GROUP_A;
static int 	pin_reset = CONFIG_SENSOR_RESET_PIN;
#elif defined(CONFIG_SENSOR_RESET_GROUP_B)
static int 	group_reset = GPIO_GROUP_B;
static int 	pin_reset = CONFIG_SENSOR_RESET_PIN;
#elif defined(CONFIG_SENSOR_RESET_GROUP_C)
static int 	group_reset = GPIO_GROUP_C;
static int 	pin_reset = CONFIG_SENSOR_RESET_PIN;
#elif defined(CONFIG_SENSOR_RESET_GROUP_D)
static int 	group_reset = GPIO_GROUP_D;
static int 	pin_reset = CONFIG_SENSOR_RESET_PIN;
#elif defined(CONFIG_SENSOR_RESET_GROUP_E)
static int 	group_reset = GPIO_GROUP_E;
static int 	pin_reset = CONFIG_SENSOR_RESET_PIN;
#else
static int      group_reset = -1;
static int      pin_reset = -1 ;
#endif

 #if defined(CONFIG_SENSOR_PD_GROUP_A)
static int 	group_pd = GPIO_GROUP_A;
static int 	pin_pd = CONFIG_SENSOR_PD_PIN;
#elif defined(CONFIG_SENSOR_PD_GROUP_B)
static int 	group_pd = GPIO_GROUP_B;
static int 	pin_pd = CONFIG_SENSOR_PD_PIN;
#elif defined(CONFIG_SENSOR_PD_GROUP_C)
static int 	group_pd = GPIO_GROUP_C;
static int 	pin_pd = CONFIG_SENSOR_PD_PIN;
#elif defined(CONFIG_SENSOR_PD_GROUP_D)
static int 	group_pd = GPIO_GROUP_D;
static int 	pin_pd = CONFIG_SENSOR_PD_PIN;
#elif defined(CONFIG_SENSOR_PD_GROUP_E)
static int 	group_pd = GPIO_GROUP_E;
static int 	pin_pd = CONFIG_SENSOR_PD_PIN;
#else
static int      group_pd = -1;
static int      pin_pd = -1 ;
#endif

static void SnrReset(void)
{/* GPB02 reset:	H->L->H 	*/		
	printk("%s\n",__FUNCTION__);		
	printk("%x-%x\n",group_reset, pin_reset);					
#if 1		
	outp32((REG_GPAFUN+4*group_reset), (inp32((REG_GPAFUN+4*group_reset)) & ~(3<<(pin_reset*2))) );
#else
	outp32(REG_GPBFUN, inp32(REG_GPBFUN)&(~MF_GPB2));
#endif
	printk("REG_GPBFUN = 0x%x\n", inp32(REG_GPBFUN));

	w55fa93_gpio_set(group_reset, pin_reset, 1);    	/* Set high */
	w55fa93_gpio_set_output(group_reset, pin_reset);
	udelay(100);
	w55fa93_gpio_set(group_reset, pin_reset, 0);		/* Set low */
	udelay(100);		
	w55fa93_gpio_set(group_reset, pin_reset, 1);		/* Set high */

}
static void SnrPowerDown(BOOL bIsEnable)
{/* GPB3 power down, HIGH for power down */
	printk("%s\n",__FUNCTION__);
	printk("%x-%x\n",group_pd, pin_pd);				
#if 1
	outp32((REG_GPAFUN+4*group_pd), (inp32((REG_GPAFUN+4*group_pd)) & ~(3<<(pin_pd*2))) );
#else
	outp32(REG_GPBFUN, inp32(REG_GPBFUN)&(~MF_GPB3));
#endif
	printk("REG_GPBFUN = 0x%x\n", inp32(REG_GPBFUN));

	w55fa93_gpio_set(group_pd, pin_pd, 1);
	w55fa93_gpio_set_output(group_pd, pin_pd);
	
	if(bIsEnable)
		w55fa93_gpio_set(group_pd, pin_pd, 1);	// set high for power down 
	else			
		w55fa93_gpio_set(group_pd, pin_pd, 0);	// set low for running

}
void ResetSensor(void)
{
	if ( group_reset>=0 && pin_reset>=0 )
		SnrReset();
}
void PowerdownSensor(BOOL bIsEnable)
{
	if ( group_pd>=0 && pin_pd>=0 )
		SnrPowerDown(bIsEnable);
}


