/* 
 * linux/arch/arm/mach-w55fa93/w55fa93.c
 *
 * Based on linux/arch/arm/plat-s3c24xx/s3c244x.c by Ben Dooks
 *
 *Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
*/

#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <mach/w55fa93_reg.h>
#include <mach/hardware.h>
#include <mach/serial.h>
#include "cpu.h"
#include "dev.h"
#include "clock.h"

/* Initial IO mappings */
static struct map_desc w55fa93_iodesc[] __initdata = {
	IODESC_ENT(IRQ),
	IODESC_ENT(UART),
	
	IODESC_ENT(TIMER),
	IODESC_ENT(USBD),
	IODESC_ENT(GCR),

	IODESC_ENT(SDIC),
	IODESC_ENT(SIC),

	IODESC_ENT(EDMA),

	IODESC_ENT(VPOST),
	IODESC_ENT(BLT),

	IODESC_ENT(VIDEOIN),
	IODESC_ENT(I2SM),
	IODESC_ENT(FSC),

	IODESC_ENT(SPU),

	IODESC_ENT(JPEG),
	IODESC_ENT(USBH),

	IODESC_ENT(PWM),
	IODESC_ENT(GPIO),
	IODESC_ENT(ADC),

	IODESC_ENT(SPI0),	

	IODESC_ENT(RTC),
	IODESC_ENT(I2C),
	IODESC_ENT(KPI),

	IODESC_ENT(IBR),	
};

/* Initial clock declarations. */
static DEFINE_AHBCLK(cpu, 0);
static DEFINE_AHBCLK(apbclk, 1);
static DEFINE_AHBCLK(hclk, 2);
static DEFINE_AHBCLK(sram, 3);
static DEFINE_AHBCLK(dram, 4);
static DEFINE_AHBCLK(blt, 5);
static DEFINE_AHBCLK(fsc, 6);
static DEFINE_AHBCLK(jpg, 7);
static DEFINE_AHBCLK(hclk1, 8);
static DEFINE_AHBCLK(edma0, 10);
static DEFINE_AHBCLK(edma1, 11);
static DEFINE_AHBCLK(edma2, 12);
static DEFINE_AHBCLK(edma3, 13);
static DEFINE_AHBCLK(edma4, 14);
static DEFINE_AHBCLK(des, 15);
static DEFINE_AHBCLK(hclk3, 16);
static DEFINE_AHBCLK(usbh, 17);
static DEFINE_AHBCLK(usbd, 18);
static DEFINE_AHBCLK(ge4p, 19);
static DEFINE_AHBCLK(gpu, 20);
static DEFINE_AHBCLK(sic, 21);
static DEFINE_AHBCLK(nand, 22);
static DEFINE_AHBCLK(sd, 23);
static DEFINE_AHBCLK(hclk4, 24);
static DEFINE_AHBCLK(spu, 25);
static DEFINE_AHBCLK(i2s, 26);
static DEFINE_AHBCLK(vpost, 27);
static DEFINE_AHBCLK(cap, 28);
static DEFINE_AHBCLK(sen, 29);
static DEFINE_AHBCLK(ado, 30);
static DEFINE_AHBCLK(spu1, 31);

static DEFINE_APBCLK(adc, 0);
static DEFINE_APBCLK(i2c, 1);
static DEFINE_APBCLK(rtc, 2);
static DEFINE_APBCLK(uart0, 3);
static DEFINE_APBCLK(uart1, 4);
static DEFINE_APBCLK(pwm, 5);
static DEFINE_APBCLK(spims0, 6);
static DEFINE_APBCLK(spims1, 7);
static DEFINE_APBCLK(timer0, 8);
static DEFINE_APBCLK(timer1, 9);
static DEFINE_APBCLK(wdt, 15);
static DEFINE_APBCLK(tic, 24);
static DEFINE_APBCLK(kpi, 25);

static struct clk_lookup w55fa93_clkregs[] = {
	DEF_CLKLOOK(&clk_cpu, NULL, "cpu"),
	DEF_CLKLOOK(&clk_apbclk, NULL, "apbclk"),
	DEF_CLKLOOK(&clk_hclk, NULL, "hclk"),
	DEF_CLKLOOK(&clk_sram, NULL, "sram"),
	DEF_CLKLOOK(&clk_dram, NULL, "dram"),
	DEF_CLKLOOK(&clk_hclk1, NULL, "hclk1"),
	DEF_CLKLOOK(&clk_hclk3, NULL, "hclk3"),
	DEF_CLKLOOK(&clk_hclk4, NULL, "hclk4"),

	DEF_CLKLOOK(&clk_blt, NULL, "blt"),	
	DEF_CLKLOOK(&clk_fsc, "w55fa93-fsc", NULL),
	DEF_CLKLOOK(&clk_edma0, NULL, "edma0"),
	DEF_CLKLOOK(&clk_edma1, NULL, "edma1"),
	DEF_CLKLOOK(&clk_edma2, NULL, "edma2"),
	DEF_CLKLOOK(&clk_edma3, NULL, "edma3"),
	DEF_CLKLOOK(&clk_edma4, NULL, "edma4"),
	DEF_CLKLOOK(&clk_des, "w55fa93-des", NULL),
	DEF_CLKLOOK(&clk_jpg, NULL, "w55fa93-jpeg"),
	DEF_CLKLOOK(&clk_usbh, "w55fa93-ohci", NULL),
	DEF_CLKLOOK(&clk_usbd, "w55fa93-usbgadget", NULL),
	DEF_CLKLOOK(&clk_ge4p, "w55fa93-ge4p", NULL),
	DEF_CLKLOOK(&clk_gpu, "w55fa93-gpu", NULL),
	DEF_CLKLOOK(&clk_sic, NULL, "SIC"),
	DEF_CLKLOOK(&clk_nand, NULL, "NAND"),
	DEF_CLKLOOK(&clk_sd, NULL, "SD"),
	DEF_CLKLOOK(&clk_spu, NULL, "SPU"),
	DEF_CLKLOOK(&clk_i2s, NULL, "I2S"),
	DEF_CLKLOOK(&clk_vpost, "w55fa93-lcd", NULL),
	DEF_CLKLOOK(&clk_cap, NULL, "CAP"),
	DEF_CLKLOOK(&clk_sen, NULL, "SEN"),
	DEF_CLKLOOK(&clk_ado, NULL, "ADO_ENGINE"),
	DEF_CLKLOOK(&clk_spu1, NULL, "SPU1"),

	DEF_CLKLOOK(&clk_adc, NULL, "ADC"),
	DEF_CLKLOOK(&clk_i2c, "w55fa93-i2c", NULL),
	DEF_CLKLOOK(&clk_rtc, NULL, "rtc"),
	DEF_CLKLOOK(&clk_uart0, "w55fa93-uart0", NULL),
	DEF_CLKLOOK(&clk_uart1, "w55fa93-uart1", NULL),
	DEF_CLKLOOK(&clk_pwm, NULL, "w55fa93-pwm"),
	DEF_CLKLOOK(&clk_spims0, "w55fa93-spi", "MS0"),
	DEF_CLKLOOK(&clk_spims1, "w55fa93-spi", "MS1"),
	DEF_CLKLOOK(&clk_timer0, NULL, "timer0"),
	DEF_CLKLOOK(&clk_timer1, NULL, "timer1"),
	DEF_CLKLOOK(&clk_wdt, "w55fa93-wdt", NULL),
	DEF_CLKLOOK(&clk_tic, "w55fa93-tic", NULL),
	DEF_CLKLOOK(&clk_kpi, "w55fa93-kpi", NULL),
};

struct resource w55fa93_HUART_resource[]= {
	[0] = {
		.start = W55FA93_PA_UART,
		.end   = W55FA93_PA_UART + 0x0ff,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_HUART,
		.end   = IRQ_HUART,
		.flags = IORESOURCE_IRQ,
	}
};

struct resource w55fa93_UART_resource[]= {
	[0] = {
		.start = W55FA93_PA_UART + 0x100,
		.end   = W55FA93_PA_UART + 0x1ff,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_UART,
		.end   = IRQ_UART,
		.flags = IORESOURCE_IRQ,
	}
};

/*Init the plat dev*/
W55FA93_DEVICE(uart0,HUART,0,"w55fa93-uart");
W55FA93_DEVICE(uart1,UART,1,"w55fa93-uart");

static struct platform_device *uart_devices[] __initdata = {
	&w55fa93_uart0,
	&w55fa93_uart1
};

static int w55fa93_uart_count = 0;

/* uart registration process */

void __init w55fa93_init_uarts(struct w55fa93_uartcfg *cfg, int no)
{
	struct platform_device *platdev;
	int uart;

	// enable UART clock
	//__raw_writel(__raw_readl(REG_CLKMAN) | 1 << 11, REG_CLKMAN);

	uart_devices[0]->dev.init_name = "w55fa93-uart0";
	uart_devices[1]->dev.init_name = "w55fa93-uart1";
	for (uart = 0; uart < no; uart++, cfg++) {
		platdev = uart_devices[cfg->hwport];

		w55fa93_uart_devs[uart] = platdev;
		platdev->dev.platform_data = cfg;
	}

	w55fa93_uart_count = uart;
}

/* w55fa93_map_io
 *
 * register the standard cpu IO areas, and any passed in from the
 * machine specific initialisation.
*/

void __init w55fa93_map_io()
{
	/* register our io-tables */

	cpu_map_io(w55fa93_iodesc, ARRAY_SIZE(w55fa93_iodesc));
}

static DEFINE_SPINLOCK(nvt_general_lock);
static unsigned long nvt_general_flags;

void nvt_lock(void)
{
	spin_lock_irqsave(&nvt_general_lock, nvt_general_flags);
}
EXPORT_SYMBOL(nvt_lock);

void nvt_unlock(void)
{
	spin_unlock_irqrestore(&nvt_general_lock, nvt_general_flags);
}
EXPORT_SYMBOL(nvt_unlock);

unsigned int w55fa93_external_clock;
unsigned int w55fa93_apll_clock;
unsigned int w55fa93_upll_clock;
unsigned int w55fa93_system_clock;
unsigned int w55fa93_cpu_clock;
unsigned int w55fa93_ahb_clock;
unsigned int w55fa93_apb_clock;
EXPORT_SYMBOL(w55fa93_external_clock);
EXPORT_SYMBOL(w55fa93_apll_clock);
EXPORT_SYMBOL(w55fa93_upll_clock);
EXPORT_SYMBOL(w55fa93_system_clock);
EXPORT_SYMBOL(w55fa93_cpu_clock);
EXPORT_SYMBOL(w55fa93_ahb_clock);
EXPORT_SYMBOL(w55fa93_apb_clock);

// pll = 0 for APLL, 1 for UPLL
unsigned int w55fa93_get_pllcon(char pll, unsigned int targetKHz)
{
	unsigned char NOMap[4] = {1, 2, 2, 4};
	unsigned int NR, NF, NO, out_dv, in_dv, fb_dv;
	unsigned int FinKHz, FoutKHz, pllcon;

	FinKHz = w55fa93_external_clock / 1000;
	for (out_dv = 0; out_dv < 4; out_dv++) {
		NO = NOMap[out_dv];
		for (in_dv = 0; in_dv < 32; in_dv++) {
			NR = (in_dv + 2) * 2;
			if (((FinKHz / NR) < 1000) || ((FinKHz / NR) > 15000))
				continue;
			for (fb_dv = 0; fb_dv < 512; fb_dv++) {
				NF = (fb_dv + 2) * 2;
				if (((FinKHz * NF / NR) < 100000) || ((FinKHz * NF / NR) > 500000))
					continue;
				FoutKHz = FinKHz * NF / NR / NO;
				//printk("FoutKHz = %d\n", FoutKHz);
				if (targetKHz == FoutKHz) {
					pllcon = (out_dv << 14) | (in_dv << 9) | fb_dv;
					return pllcon;
				}
			}
		}
	}

	return 0;
}
EXPORT_SYMBOL(w55fa93_get_pllcon);

int w55fa93_set_apll_clock(unsigned int clock)
{
	int ret = 0;

#if 1
	if (w55fa93_external_clock == 12000000) {
		if (clock == 208896) {		// for SPU/I2S 48/32KHz * 128, TD = 0.0063%
			__raw_writel(0x937D, REG_APLLCON);
		}
		else if (clock == 184320) {	// for ADC 16KHz * 16 * 80, TD = 0.0237%
			__raw_writel(0x12A7, REG_APLLCON);
		}
		else if (clock == 169344) {	// for SPU/I2S 44.1KHz * 384 and ADC 11.025KHz * 16 * 80, TD = 0.0063%
			__raw_writel(0x4EFC, REG_APLLCON);
		}
		else if (clock == 153600) {	// for ADC 24/20KHz * 16 * 80, TD = 0%
			__raw_writel(0x063E, REG_APLLCON);
		}
		else if (clock == 147456) {	// for SPU/I2S 64/96KHz * 256, TD = 0.0186%
			__raw_writel(0x0A54, REG_APLLCON);
		}
		else if (clock == 135475) {	// for SPU/I2S 88.2KHz * 256, TD = 0.0183%
			__raw_writel(0x550D, REG_APLLCON);
		}
		else if (clock == 135000) {	// for TV 27MHz, TD = 0%, 8KHz TD = 0.1243%, 11.025KHz TD = 0.3508%
			__raw_writel(0x4458, REG_APLLCON);
		}
		else if (clock == 106496) {	// for SPU/I2S 32KHz * 256, TD = 0.0038%
			__raw_writel(0x4445, REG_APLLCON);
		}
		else {
			printk("%s does not support %dKHz APLL clock!\n", __FUNCTION__, clock);
			ret = -1;
		}
	}
	else if (w55fa93_external_clock == 27000000) {
		if (clock == 208896) {
			__raw_writel(0x6324, REG_APLLCON);
		}
		else if (clock == 184320) {
			__raw_writel(0x1E72, REG_APLLCON);
		}
		else if (clock == 169344) {
			__raw_writel(0x1243, REG_APLLCON);
		}
		else if (clock == 153600) {
			__raw_writel(0xAD0F, REG_APLLCON);
		}
		else if (clock == 147456) {
			__raw_writel(0x1645, REG_APLLCON);
		}
		else if (clock == 135475) {
			__raw_writel(0x730D, REG_APLLCON);
		}
		else if (clock == 135000) {
			__raw_writel(0x4426, REG_APLLCON);
		}
		else if (clock == 106496) {
			__raw_writel(0x4E45, REG_APLLCON);
		}
		else {
			printk("%s does not support %dKHz APLL clock!\n", __FUNCTION__, clock);
			ret = -1;
		}
	}
	else {
		printk("%s does not support %d.%dMHz xtal clock!\n", __FUNCTION__, print_mhz(w55fa93_external_clock));
		ret = -1;
	}
#else
	unsigned int pllcon;

	if (clock != w55fa93_apll_clock) {
		pllcon = w55fa93_get_pllcon(0, clock);
		if (pllcon == 0) {
			printk("Cannot calculate APLL %d KHz!!\n", clock);
			ret = -1;
		} else
			__raw_writel(pllcon, REG_APLLCON);
	}

#endif

	if (ret == 0)
		w55fa93_apll_clock = clock;

	return ret;
}
EXPORT_SYMBOL(w55fa93_set_apll_clock);

// pll = 0 for APLL, 1 for UPLL
static inline unsigned int
w55fa93_get_pll(char pll, unsigned int xtal)
{
	unsigned char NOMap[4] = {1, 2, 2, 4};
	unsigned int pllcon;
	unsigned int NR, NF, NO;
	uint64_t fvco = 0;

	if (pll == 0)
		pllcon = __raw_readl(REG_APLLCON);
	else if (pll == 1)
		pllcon = __raw_readl(REG_UPLLCON);
	else
		return 0;

	NF = ((pllcon & FB_DV) + 2) * 2;
	NR = (((pllcon & IN_DV) >> 9) + 2) * 2;
	NO = NOMap[((pllcon & OUT_DV) >> 14)];

	fvco = (uint64_t)xtal * NF;
	do_div(fvco, (NR * NO));

	return (unsigned int)fvco;
}

/* w55fa93_init_clocks
 *
 * Initialise the clock subsystem and associated information from the
 * given master crystal value.
 *
 */
void __init w55fa93_init_clocks(void)
{
	int xtal;

	if (__raw_readl(0xFFFF3EB4) == 0x50423238) {
		// G version chip
		xtal = 12000000;
	} else {
		if ((__raw_readl(REG_CHIPCFG) & 0xC) == 0x8)
			xtal = 12000000;
		else 
			xtal = 27000000;
	}
	w55fa93_external_clock = xtal;
	w55fa93_apll_clock = w55fa93_get_pll(0, xtal) / 1000;
	w55fa93_upll_clock = w55fa93_get_pll(1, xtal) / 1000;
	if ((__raw_readl(REG_CLKDIV0) & 0x18) == 0x10)
		w55fa93_system_clock = w55fa93_apll_clock/((__raw_readl(REG_CLKDIV0)&0x7) + 1); 
	else if ((__raw_readl(REG_CLKDIV0) & 0x18) == 0x18)
		w55fa93_system_clock = w55fa93_upll_clock/((__raw_readl(REG_CLKDIV0)&0x7) + 1); 
	else
		printk("w55fa93_system_clock is unsupported!\n");
	w55fa93_cpu_clock = w55fa93_system_clock/((__raw_readl(REG_CLKDIV4)&0xF) + 1);
	w55fa93_ahb_clock = w55fa93_system_clock/((((__raw_readl(REG_CLKDIV4)>>4)&0xF) + 1) * 2);
	if (w55fa93_system_clock < (w55fa93_cpu_clock * 2))
		w55fa93_apb_clock = w55fa93_system_clock/((((__raw_readl(REG_CLKDIV4)>>8)&0xF) + 1) * 2);
	else
		w55fa93_apb_clock = w55fa93_cpu_clock/(((__raw_readl(REG_CLKDIV4)>>8)&0xF) + 1);
#if 1
	printk("w55fa93_external_clock	= %d.%d MHz\n", print_mhz(w55fa93_external_clock));
	printk("w55fa93_apll_clock	= %d KHz\n", w55fa93_apll_clock);
	printk("w55fa93_upll_clock	= %d KHz\n", w55fa93_upll_clock);
	printk("w55fa93_system_clock	= %d KHz\n", w55fa93_system_clock);
	printk("w55fa93_cpu_clock	= %d KHz\n", w55fa93_cpu_clock);
	printk("w55fa93_ahb_clock	= %d KHz\n", w55fa93_ahb_clock);
	printk("w55fa93_apb_clock	= %d KHz\n", w55fa93_apb_clock);
#endif
	clkdev_add_table(w55fa93_clkregs, ARRAY_SIZE(w55fa93_clkregs));
}

void __init w55fa93_board_init(void)
{
	platform_add_devices(w55fa93_uart_devs, w55fa93_uart_count);
	w55fa93_dev_init();
}
