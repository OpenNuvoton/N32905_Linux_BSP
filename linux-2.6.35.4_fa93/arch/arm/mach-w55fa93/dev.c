/*
 * linux/arch/arm/mach-w55fa93/dev.c
 *
 * Copyright (C) 2009 Nuvoton corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/mtd/physmap.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/mach-types.h>

#include <mach/fb.h>
#include <mach/w55fa93_spi.h>
#include <mach/w55fa93_keypad.h>
#include <mach/w55fa93_reg.h>

#include "cpu.h"
#include "dev.h"

/* W55FA93 SPI flash driver data */
#define W55FA93_FLASH_BASE	0xA0000000
#define W55FA93_FLASH_SIZE	0x400000
#define SPIOFFSET		0x200
#define SPIOREG_SIZE		0x100

/* Serial port registrations */
struct platform_device *w55fa93_uart_devs[2];

static struct mtd_partition w55fa93_flash_partitions[] = {
        {
                .name	=	"MTD reserved for user app",
                .size	=	0x100000,
                .offset	=	0x300000,
        }
};

static struct physmap_flash_data w55fa93_flash_data = {
        .width		=	2,
        .parts		=	w55fa93_flash_partitions,
        .nr_parts	=	ARRAY_SIZE(w55fa93_flash_partitions),
};

static struct resource w55fa93_flash_resources[] = {
        {
                .start	=	W55FA93_FLASH_BASE,
                .end	=	W55FA93_FLASH_BASE + W55FA93_FLASH_SIZE - 1,
                .flags	=	IORESOURCE_MEM,
        }
};

static struct platform_device w55fa93_flash_device = {
        .name		=	"physmap-flash",
        .id		=	0,
        .dev		= {
                .platform_data = &w55fa93_flash_data,
        },
        .resource	=	w55fa93_flash_resources,
        .num_resources	=	ARRAY_SIZE(w55fa93_flash_resources),
};

/* USB OHCI Host Controller */

static struct resource w55fa93_ohci_resource[] = {
        [0] = {
                .start = W55FA93_PA_USBH,
                .end   = W55FA93_PA_USBH + W55FA93_SZ_USBH - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_USBH,
                .end   = IRQ_USBH,
                .flags = IORESOURCE_IRQ,
        }
};

static u64 w55fa93_device_usb_ohci_dmamask = 0xffffffffUL;
static struct platform_device w55fa93_device_ohci = {
        .name		  = "w55fa93-ohci",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(w55fa93_ohci_resource),
        .resource	  = w55fa93_ohci_resource,
        .dev              = {
                .dma_mask = &w55fa93_device_usb_ohci_dmamask,
                .coherent_dma_mask = 0xffffffffUL
        }
};

/* USB Device (Gadget)*/

static struct resource w55fa93_usbgadget_resource[] = {
        [0] = {
                .start = W55FA93_PA_USBD,
                .end   = W55FA93_PA_USBD + W55FA93_SZ_USBD - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_USBD,
                .end   = IRQ_USBD,
                .flags = IORESOURCE_IRQ,
        }
};

static u64 w55fa93_device_udc_dmamask = 0xffffffffUL;
static struct platform_device w55fa93_device_usbgadget = {
	.name		= "w55fa93-usbgadget",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(w55fa93_usbgadget_resource),
	.resource	= w55fa93_usbgadget_resource,
	.dev              = {
		.dma_mask = &w55fa93_device_udc_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};

/* SPI device */

static struct w55fa93_spi_info w55fa93_spiflash_data = {        
        .num_cs		= 2,        
        .lsb		= 0,
        .txneg		= 1,
        .rxneg		= 0,
        .divider	= 0,
        .sleep		= 0,
        .txnum		= 0,
        .txbitlen	= 8,
        .byte_endin = 0,
        .bus_num	= 0,
};

static struct resource w55fa93_spi_resource[] = {
        [0] = {
                .start = W55FA93_PA_SPI0,
                .end   = W55FA93_PA_SPI0 + W55FA93_SZ_SPI0 - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_SPI0,
                .end   = IRQ_SPI0,
                .flags = IORESOURCE_IRQ,
        }
};

static struct platform_device w55fa93_device_spi = {
        .name		= "w55fa93-spi",
        .id		= -1,
        .num_resources	= ARRAY_SIZE(w55fa93_spi_resource),
        .resource	= w55fa93_spi_resource,
        .dev		= {
                .platform_data = &w55fa93_spiflash_data,
        }
};

/* spi device, spi flash info */
#ifdef CONFIG_MTD_PARTITIONS  
#ifdef CONFIG_MTD_M25P80
static struct mtd_partition w55fa93_spi_flash_partitions[] = {
        [0] = {
        	.name   = "UserData",
		.size   = CONFIG_SPIFLASH_PARTITION_SIZE,
		.offset = CONFIG_SPIFLASH_PARTITION_OFFSET,
	},
	
};
#endif
#endif

static struct flash_platform_data w55fa93_spi_flash_data = {
#ifdef CONFIG_MTD_PARTITIONS 
#ifdef CONFIG_MTD_M25P80
        .name = "m25p80",
        .parts =  w55fa93_spi_flash_partitions,
        .nr_parts = ARRAY_SIZE(w55fa93_spi_flash_partitions),          
        .type = "w25q32",
#endif         
#endif         
};

static struct spi_board_info w55fa93_spi_board_info[] __initdata = {
#ifdef CONFIG_FA93_SPI_CS0_ENABLE
        [0] = {
#ifdef CONFIG_FA93_SPI_CS0_MTD			
                .modalias = "m25p80",
#endif
#ifdef CONFIG_FA93_SPI_CS0_SPIDEV
		.modalias = "spidev",
#endif
                .max_speed_hz = 50000000,
                .bus_num = 0,
                .chip_select = 0,                
                .platform_data = &w55fa93_spi_flash_data,
                .mode = SPI_MODE_0,
        },
#endif     

#ifdef CONFIG_FA93_SPI_CS1_ENABLE
#if defined(CONFIG_FA93_SPI_CS0_ENABLE) && defined(CONFIG_FA93_SPI_CS1_ENABLE)
        [1] = { 
#else
	[0] = { 
#endif
#ifdef CONFIG_FA93_SPI_CS1_MTD			
                .modalias = "m25p80",
#endif
#ifdef CONFIG_FA93_SPI_CS1_SPIDEV
		.modalias = "spidev",
#endif
                .max_speed_hz = 50000000,
                .bus_num = 0,                
                .chip_select = 1,
                .platform_data = &w55fa93_spi_flash_data,
                .mode = SPI_MODE_0,
        },
#endif        
};

/* WDT Device */

static struct resource w55fa93_wdt_resource[] = {
        [0] = {
                .start = W55FA93_PA_TIMER,
                .end   = W55FA93_PA_TIMER + W55FA93_SZ_TIMER - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_WDT,
                .end   = IRQ_WDT,
                .flags = IORESOURCE_IRQ,
        }
};

static struct platform_device w55fa93_device_wdt = {
        .name		= "w55fa93-wdt",
        .id		= -1,
        .num_resources	= ARRAY_SIZE(w55fa93_wdt_resource),
        .resource	= w55fa93_wdt_resource,
};

/*
 * public device definition between 910 and 920, or 910
 * and 950 or 950 and 960...,their dev platform register
 * should be in specific file such as nuc950, nuc960 c
 * files rather than the public dev.c file here. so the
 * corresponding platform_device definition should not be
 * static.
*/

/* RTC controller*/

static struct resource w55fa93_rtc_resource[] = {
        [0] = {
                .start = W55FA93_PA_RTC,
                .end   = W55FA93_PA_RTC + 0xff,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_RTC,
                .end   = IRQ_RTC,
                .flags = IORESOURCE_IRQ,
        },
};

struct platform_device w55fa93_device_rtc = {
        .name		= "w55fa93-rtc",
        .id		= -1,
        .num_resources	= ARRAY_SIZE(w55fa93_rtc_resource),
        .resource	= w55fa93_rtc_resource,
};

/*TouchScreen controller*/

static struct resource w55fa93_ts_resource[] = {
        [0] = {
                .start = W55FA93_PA_ADC,
                .end   = W55FA93_PA_ADC + W55FA93_SZ_ADC-1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_ADC,
                .end   = IRQ_ADC,
                .flags = IORESOURCE_IRQ,
        },
};

struct platform_device w55fa93_device_ts = {
        .name		= "w55fa93-ts",
        .id		= -1,
        .resource	= w55fa93_ts_resource,
        .num_resources	= ARRAY_SIZE(w55fa93_ts_resource),
};

/* FMI Device */

static struct resource w55fa93_fmi_resource[] = {
        [0] = {
                .start = W55FA93_PA_SIC,
                .end   = W55FA93_PA_SIC + W55FA93_SZ_SIC - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_SIC,
                .end   = IRQ_SIC,
                .flags = IORESOURCE_IRQ,
        }
};

static u64 w55fa93_device_fmi_dmamask = 0xffffffffUL;
struct platform_device w55fa93_device_fmi = {
	.name		= "w55fa93-fmi",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(w55fa93_fmi_resource),
	.resource	= w55fa93_fmi_resource,
	.dev              = {
		.dma_mask = &w55fa93_device_fmi_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};

/* KPI controller */

static struct resource w55fa93_kpi_resource[] = {
        [0] = {
                .start = W55FA93_PA_KPI,
                .end   = W55FA93_PA_KPI + W55FA93_SZ_KPI - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_KPI,
                .end   = IRQ_KPI,
                .flags = IORESOURCE_IRQ,
        }

};

struct platform_device w55fa93_device_kpi = {
        .name		= "w55fa93-kpi",
        .id		= -1,
        .num_resources	= ARRAY_SIZE(w55fa93_kpi_resource),
        .resource	= w55fa93_kpi_resource,
};


/* VPOST controller*/
static struct resource w55fa93_lcd_resource[] = {
	[0] = {
		.start = W55FA93_PA_VPOST,
		.end   = W55FA93_PA_VPOST + W55FA93_SZ_VPOST - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_VPOST,
		.end   = IRQ_VPOST,
		.flags = IORESOURCE_IRQ,
	}
};

static u64 w55fa93_device_lcd_dmamask = -1;
struct platform_device w55fa93_device_lcd = {
	.name             = "w55fa93-lcd",
	.id               = -1,
	.num_resources    = ARRAY_SIZE(w55fa93_lcd_resource),
	.resource         = w55fa93_lcd_resource,
	.dev              = {
		.dma_mask               = &w55fa93_device_lcd_dmamask,
		.coherent_dma_mask      = -1,
//		.platform_data = &w55fa93_fb_info,
	}
};

EXPORT_SYMBOL(w55fa93_device_lcd);
//W55FA93_RECS(VPOST);		// defined in "device.h"
//W55FA93_DEVICE(lcddevice,VPOST,0,"w55fa93-lcd");
//EXPORT_SYMBOL(w55fa93_lcddevice);

void w55fa93_fb_set_platdata(struct w55fa93fb_mach_info *pd)
{
        struct w55fa93fb_mach_info *npd;

        npd = kmalloc(sizeof(*npd), GFP_KERNEL);
        if (npd) {
                memcpy(npd, pd, sizeof(*npd));
//                w55fa93_lcddevice.dev.platform_data = npd;
                w55fa93_device_lcd.dev.platform_data = npd;
        } else {
                printk(KERN_ERR "no memory for W55FA93 LCD platform data\n");
        }
}
EXPORT_SYMBOL(w55fa93_fb_set_platdata);

/* AUDIO controller */

static u64 w55fa93_device_audio_dmamask = -1;
#if 0
static struct resource w55fa93_ac97_resource[] = {
        [0] = {
                .start = W55FA93_PA_SPU,
                .end   = W55FA93_PA_SPU + W55FA93_SZ_SPU - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_SPU,
                .end   = IRQ_SPU,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device w55fa93_device_audio_ac97 = {
        .name		= "w55fa93-audio-ac97",
        .id		= -1,
        .num_resources	= ARRAY_SIZE(w55fa93_ac97_resource),
        .resource	= w55fa93_ac97_resource,
        .dev              = {
                .dma_mask               = &w55fa93_device_audio_dmamask,
                .coherent_dma_mask      = -1,
        }
};
#endif

static struct resource w55fa93_spu_resource[] = {
        [0] = {
                .start = W55FA93_PA_SPU,
                .end   = W55FA93_PA_SPU + W55FA93_SZ_SPU - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_SPU,
                .end   = IRQ_SPU,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device w55fa93_device_audio_spu = {
        .name		= "w55fa93-audio-spu",
        .id		= -1,
        .num_resources	= ARRAY_SIZE(w55fa93_spu_resource),
        .resource	= w55fa93_spu_resource,
        .dev              = {
                .dma_mask               = &w55fa93_device_audio_dmamask,
                .coherent_dma_mask      = -1,
        }
};

static struct resource w55fa93_i2s_resource[] = {
        [0] = {
                .start = W55FA93_PA_I2SM,
                .end   = W55FA93_PA_I2SM + W55FA93_SZ_I2SM - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_I2S,
                .end   = IRQ_I2S,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device w55fa93_device_audio_i2s = {
        .name		= "w55fa93-audio-i2s",
        .id		= -1,
        .num_resources	= ARRAY_SIZE(w55fa93_i2s_resource),
        .resource	= w55fa93_i2s_resource,
        .dev              = {
                .dma_mask               = &w55fa93_device_audio_dmamask,
                .coherent_dma_mask      = -1,
        }
};



static struct resource w55fa93_adc_resource[] = {
        [0] = {
                .start = W55FA93_PA_ADC,
                .end   = W55FA93_PA_ADC + W55FA93_SZ_ADC - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_EDMA,
                .end   = IRQ_EDMA,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device w55fa93_device_audio_adc = {
        .name		= "w55fa93-audio-adc",
        .id		= -1,
        .num_resources	= ARRAY_SIZE(w55fa93_adc_resource),
        .resource	= w55fa93_adc_resource,
        .dev              = {
                .dma_mask               = &w55fa93_device_audio_dmamask,
                .coherent_dma_mask      = -1,
        }
};

/* I2C */

static struct resource w55fa93_i2c_resource[] = {
        [0] = {
                .start = W55FA93_PA_I2C,
                .end   = W55FA93_PA_I2C + W55FA93_SZ_I2C - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_I2C,
                .end   = IRQ_I2C,
                .flags = IORESOURCE_IRQ,
        }

};

struct platform_device w55fa93_device_i2c = {
        .name		  = "w55fa93-i2c",
        .id		  = -1,
        .num_resources	  = ARRAY_SIZE(w55fa93_i2c_resource),
        .resource	  = w55fa93_i2c_resource,
};

EXPORT_SYMBOL(w55fa93_device_i2c);

static struct platform_device w55fa93_device_gpioi2c = {
	.name		= "w55fa93-gpioi2c",
	.id		= 0,
	.dev		= {
		.platform_data = NULL,
	},
	.num_resources	= 0
};

/* PWM */

#ifdef CONFIG_W55FA93_PWM0_PD0
static struct resource w55fa93_pwm0_resource[] = {
        [0] = {
                .start = W55FA93_PA_PWM + 0x0C,
                .end   = W55FA93_PA_PWM + 0x14 - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_PWM,
                .end   = IRQ_PWM,
                .flags = IORESOURCE_IRQ,
        }

};

struct platform_device w55fa93_device_pwm0 = {
        .name		  = "w55fa93-pwm",
        .id		  = 0,
        .num_resources	  = ARRAY_SIZE(w55fa93_pwm0_resource),
        .resource	  = w55fa93_pwm0_resource,
};

EXPORT_SYMBOL(w55fa93_device_pwm0);
#endif

#ifdef CONFIG_W55FA93_PWM1_PD1
static struct resource w55fa93_pwm1_resource[] = {
        [0] = {
                .start = W55FA93_PA_PWM + 0x18,
                .end   = W55FA93_PA_PWM + 0x20 - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_PWM,
                .end   = IRQ_PWM,
                .flags = IORESOURCE_IRQ,
        }

};

struct platform_device w55fa93_device_pwm1 = {
        .name		  = "w55fa93-pwm",
        .id		  = 1,
        .num_resources	  = ARRAY_SIZE(w55fa93_pwm1_resource),
        .resource	  = w55fa93_pwm1_resource,
};

EXPORT_SYMBOL(w55fa93_device_pwm1);
#endif

#ifdef CONFIG_W55FA93_PWM2_PD2
static struct resource w55fa93_pwm2_resource[] = {
        [0] = {
                .start = W55FA93_PA_PWM + 0x24,
                .end   = W55FA93_PA_PWM + 0x2C - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_PWM,
                .end   = IRQ_PWM,
                .flags = IORESOURCE_IRQ,
        }

};

struct platform_device w55fa93_device_pwm2 = {
        .name		  = "w55fa93-pwm",
        .id		  = 2,
        .num_resources	  = ARRAY_SIZE(w55fa93_pwm2_resource),
        .resource	  = w55fa93_pwm2_resource,
};

EXPORT_SYMBOL(w55fa93_device_pwm2);
#endif

#ifdef CONFIG_W55FA93_PWM3_PD3
static struct resource w55fa93_pwm3_resource[] = {
        [0] = {
                .start = W55FA93_PA_PWM + 0x30,
                .end   = W55FA93_PA_PWM + 0x38 - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_PWM,
                .end   = IRQ_PWM,
                .flags = IORESOURCE_IRQ,
        }

};

struct platform_device w55fa93_device_pwm3 = {
        .name		  = "w55fa93-pwm",
        .id		  = 3,
        .num_resources	  = ARRAY_SIZE(w55fa93_pwm3_resource),
        .resource	  = w55fa93_pwm3_resource,
};

EXPORT_SYMBOL(w55fa93_device_pwm3);
#endif

static struct platform_device *w55fa93_public_dev[] __initdata = {
//	&w55fa93_serial_device,
	&w55fa93_flash_device,
	&w55fa93_device_ohci,
	&w55fa93_device_usbgadget,
	&w55fa93_device_spi,
	&w55fa93_device_wdt,
	&w55fa93_device_rtc,
	&w55fa93_device_ts,
	&w55fa93_device_fmi,
	&w55fa93_device_kpi,
	&w55fa93_device_lcd,
//	&w55fa93_device_audio_ac97,
	&w55fa93_device_audio_i2s,
	&w55fa93_device_audio_adc,
	&w55fa93_device_audio_spu,
	&w55fa93_device_i2c,
	&w55fa93_device_gpioi2c,
#ifdef CONFIG_W55FA93_PWM0_PD0	
	&w55fa93_device_pwm0,
#endif	
#ifdef CONFIG_W55FA93_PWM1_PD1
	&w55fa93_device_pwm1,
#endif
#ifdef CONFIG_W55FA93_PWM2_PD2
	&w55fa93_device_pwm2,
#endif	
#ifdef CONFIG_W55FA93_PWM3_PD3
	&w55fa93_device_pwm3,
#endif	
};

/* Provide adding specific CPU platform devices API */

void __init w55fa93_dev_init()
{
        platform_add_devices(w55fa93_public_dev, ARRAY_SIZE(w55fa93_public_dev));
        spi_register_board_info(w55fa93_spi_board_info, ARRAY_SIZE(w55fa93_spi_board_info));
}
