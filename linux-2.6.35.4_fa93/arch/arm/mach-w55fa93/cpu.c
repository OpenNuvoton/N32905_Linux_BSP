/*
 * linux/arch/arm/mach-w55fa93/cpu.c
 *
 * Copyright (c) 2014 Nuvoton corporation.
 *
 * W55FA93 CPU Support
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
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/rtc.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/tlbflush.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/w55fa93_reg.h>
#include <mach/w55fa93_sysmgr.h>
#include <mach/w55fa93_rtc.h>
#include "cpu.h"
#include "dev.h"

#if defined(CONFIG_W55FA93_SYSMGR) || defined (CONFIG_W55FA93_SYSMGR_MODULE)
extern void sysmgr_report(unsigned);
#endif
extern int w55fa93_edma_isbusy(int);
extern int w55fa93_rtc_read_time_wrap(struct rtc_time *);
extern int w55fa93_rtc_set_time_wrap(struct rtc_time *);
extern int w55fa93_rtc_read_alarm_wrap(struct rtc_wkalrm *);
extern int w55fa93_rtc_set_alarm_wrap(struct rtc_wkalrm *);
extern unsigned int w55fa93_get_pllcon(char, unsigned int);

/* Initial serial platform data */
/*
struct plat_serial8250_port w55fa93_uart_data[] = {
	[0] = W55FA93_8250PORT(UART0),
	// MFSEL and CLKEN must set accordingly
	//[1] = W55FA93_8250PORT(UART1),
	{},
};

struct platform_device w55fa93_serial_device = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM,
	.dev			= {
		.platform_data	= w55fa93_uart_data,
	},
};
*/
/* Init W55FA93 io */

void __init cpu_map_io(struct map_desc *mach_desc, int mach_size)
{
	unsigned long idcode = 0x0;

	iotable_init(mach_desc, mach_size);

	idcode = __raw_readl(REG_CHIPID);
	if (idcode == W55FA93_CPUID)
		printk(KERN_INFO "CPU type 0x%08lx is W55FA93\n", idcode);
}


static struct platform_device *sys_clk;
extern unsigned int w55fa93_upll_clock;
extern unsigned int w55fa93_system_clock;
extern unsigned int w55fa93_cpu_clock;
extern unsigned int w55fa93_ahb_clock;
extern unsigned int w55fa93_apb_clock;
#ifdef CONFIG_W55FA93_KEYPAD
extern u32 w55fa93_key_pressing;
#endif
#ifdef CONFIG_TOUCHSCREEN_W55FA93
extern u32 w55fa93_ts_pressing;
#endif
static u32 *sram_vaddr;

void enter_clock_setting(u32, u8, u8, u8, u8) __attribute__ ((section ("enter_cs")));
void enter_clock_setting(u32 pllcon, u8 sys_div, u8 cpu_div, u8 apb_div, u8 dll_dis)
{
	unsigned int volatile i;

	// push register pages to TLB cache entry
	__raw_readl(REG_CLKDIV0);
	__raw_readl(REG_SDEMR);

	if (pllcon) {
		// enable SDRAM auto refresh mode
		__raw_writel(__raw_readl(REG_SDCMD) | AUTOEXSELFREF | REF_CMD, REG_SDCMD);
		for (i = 0; i < 1000; i++) ;
	}

	if (dll_dis) {
		// SDRAM does not need to disable DLL
		if ((__raw_readl(REG_CHIPCFG) & SDRAMSEL) != 0x00) {
			// disable DLL of SDRAM device
			__raw_writel(__raw_readl(REG_SDEMR) | DLLEN, REG_SDEMR);
			// disable Low Frequency mode
			__raw_writel(__raw_readl(REG_SDOPM) | LOWFREQ, REG_SDOPM);
		}
	} else {
		// DDR2 always need to disable DLL, due to DLL enable only when HCLK > 133MHz
		if ((__raw_readl(REG_CHIPCFG) & SDRAMSEL) != 0x20) {
			// enable DLL of SDRAM device
			__raw_writel(__raw_readl(REG_SDEMR) & ~DLLEN, REG_SDEMR);
		}
		// enable Low Frequency mode
		__raw_writel(__raw_readl(REG_SDOPM) & ~LOWFREQ, REG_SDOPM);
	}

	if (pllcon) {
		__raw_writel(pllcon, REG_UPLLCON);
		for (i = 0; i < 1000; i++) ;
	}
	__raw_writel((__raw_readl(REG_CLKDIV0) & ~SYSTEM_N1) | (sys_div<<8), REG_CLKDIV0);
	__raw_writel((__raw_readl(REG_CLKDIV4) & ~(APB_N|CPU_N)) | ((apb_div<<8)|cpu_div), REG_CLKDIV4);
	for (i = 0; i < 1000; i++) ;

	if (pllcon) {
		// exit SDRAM auto refresh mode
		__raw_writel(__raw_readl(REG_SDCMD) & ~REF_CMD, REG_SDCMD);
		for (i = 0; i < 1000; i++) ;
	}
}

int set_system_clocks(u32 tmp_upll_clock, u32 sys_clock, u32 cpu_clock, u32 apb_clock)
{
	void (*cs_func)(u32, u8, u8, u8, u8);
	unsigned long volatile flags;
	u32 pllcon, int_mask, ahbclk, hclk_limit;
	u32 tmp_system_clock, tmp_cpu_clock, tmp_hclk1_clock;
	u8 sys_div, cpu_div, apb_div, dll_dis;

	pllcon = 0;
	if (tmp_upll_clock != w55fa93_upll_clock) {
		pllcon = w55fa93_get_pllcon(1, tmp_upll_clock);
		if (pllcon == 0) {
			printk("Cannot calculate UPLL %d KHz !!\n", tmp_upll_clock);
			return -1;
		}
	}

	sys_div = tmp_upll_clock / sys_clock - 1 + (tmp_upll_clock%sys_clock ? 1:0);
	tmp_system_clock = tmp_upll_clock / (sys_div+1);
	cpu_div = tmp_system_clock / cpu_clock - 1 + (tmp_system_clock%cpu_clock ? 1:0);
	if (cpu_div > 1) {
		printk("CPU divider must be 0 or 1 !!\n");
		return -1;
	}
	tmp_cpu_clock = tmp_system_clock / (cpu_div+1);

	tmp_hclk1_clock = (tmp_system_clock < tmp_cpu_clock*2) ? tmp_cpu_clock/2:tmp_cpu_clock;
	apb_div = tmp_hclk1_clock / apb_clock - 1 + (tmp_hclk1_clock%apb_clock ? 1:0);
	//printk("pllcon=0x%08X, sys_div=%d, cpu_div=%d, apb_div=%d\n", pllcon, sys_div, cpu_div, apb_div);

	w55fa93_upll_clock = tmp_upll_clock;
	w55fa93_system_clock = tmp_system_clock;
	w55fa93_cpu_clock = tmp_cpu_clock;
	w55fa93_ahb_clock = w55fa93_system_clock / 2;
	w55fa93_apb_clock = (w55fa93_cpu_clock/2) / (apb_div+1);

	// check SDRAM is DDR2 or not
	if ((__raw_readl(REG_CHIPCFG) & SDRAMSEL) == 0x20)
		hclk_limit = 96000;
	else
		hclk_limit = 64000;
	if (tmp_hclk1_clock < hclk_limit)
		dll_dis = 1;
	else
		dll_dis = 0;

#if 0
	printk("PLL clock = %d\n", w55fa93_upll_clock);
	printk("SYS clock = %d\n", w55fa93_system_clock);
	printk("CPU clock = %d\n", w55fa93_cpu_clock);
	printk("AHB clock = %d\n", w55fa93_ahb_clock);
	printk("APB clock = %d\n", w55fa93_apb_clock);
	printk("REG_CHIPCFG = 0x%x\n", __raw_readl(REG_CHIPCFG));
	printk("REG_SDEMR   = 0x%x\n", __raw_readl(REG_SDEMR));
	printk("REG_SDOPM   = 0x%x\n", __raw_readl(REG_SDOPM));
	printk("REG_UPLLCON = 0x%x\n", __raw_readl(REG_UPLLCON));
	printk("REG_CLKDIV0 = 0x%x\n", __raw_readl(REG_CLKDIV0));
	printk("REG_CLKDIV4 = 0x%x\n", __raw_readl(REG_CLKDIV4));
#endif

	local_irq_save(flags);
	int_mask = __raw_readl(REG_AIC_IMR);
	// disable all interrupts
	__raw_writel(0xFFFFFFFF, REG_AIC_MDCR);
	//restore_flags(flags);

	// save AHB clock registers
	ahbclk = __raw_readl(REG_AHBCLK);
	// open SRAM engine clock
	__raw_writel(ahbclk | SRAM_CKE, REG_AHBCLK);

	// put enter_clock_setting into SRAM
	memcpy(sram_vaddr, enter_clock_setting, 512);
	cs_func = (void(*)(u32, u8, u8, u8, u8)) (sram_vaddr);

	// flush all TLB cache entries
	local_flush_tlb_all();
	// change the system clocks
	cs_func(pllcon, sys_div, cpu_div, apb_div, dll_dis);

	// restore AHB registers
	__raw_writel(ahbclk, REG_AHBCLK);

	//save_flags(flags);
	//cli();
	__raw_writel(0xFFFFFFFF, REG_AIC_MDCR);
	// restore interrupt mask
	__raw_writel(int_mask, REG_AIC_MECR);
	local_irq_restore(flags);

#if 0
	printk("REG_UPLLCON = 0x%x\n", __raw_readl(REG_UPLLCON));
	printk("REG_CLKDIV0 = 0x%x\n", __raw_readl(REG_CLKDIV0));
	printk("REG_CLKDIV4 = 0x%x\n", __raw_readl(REG_CLKDIV4));
#endif
	return 0;
}

void enter_power_saving(u8) __attribute__ ((section ("enter_ps")));
// power saving mode be 0:standby, 1:power down
void enter_power_saving(u8 stop_xtal)
{
	unsigned int volatile i;

	// push register pages to TLB cache entry
	__raw_readl(REG_CLKDIV0);

	// enable SDRAM self refresh mode
	for (i = 0; i < 0x100; i++) ;
	__raw_writel((__raw_readl(REG_SDCMD)|SELF_REF) & ~AUTOEXSELFREF, REG_SDCMD);
	for (i = 0; i < 0x100; i++) ;

	if (stop_xtal == 1) {
		// change system clock source to external crystal
		__raw_writel(__raw_readl(REG_CLKDIV0) & ~SYSTEM_S, REG_CLKDIV0);
		for (i = 0; i < 0x100; i++) ;
		// stop APLL clcok
		__raw_writel(__raw_readl(REG_APLLCON) | PD, REG_APLLCON);
		// stop UPLL clock
		__raw_writel(__raw_readl(REG_UPLLCON) | PD, REG_UPLLCON);
		for (i = 0; i < 0x300; i++) ;

#if 0
		// stop both of external and CPU clock
		__asm__ __volatile__ \
		( \
			"mov	r2, %0			@ read clock register \n\
			ldmia	r2, {r0, r1}		@ load registers to r0 and r1 \n\
			bic	r0, r0, #0x01		@ \n\
			bic	r1, r1, #0x01		@ \n\
			stmia	r2, {r0, r1}		@ " \
				: /* no output registers */ \
				: "r" (REG_PWRCON) \
				: "r0", "r1", "r2" \
		);
#else
		// stop only external crystal
		__raw_writel((__raw_readl(REG_PWRCON) & ~(PRE_SCALAR|XTAL_EN)) | (0xFF << 8), REG_PWRCON);
#endif
	}
	else {
		// stop APLL clcok
		__raw_writel(__raw_readl(REG_APLLCON) | PD, REG_APLLCON);
		for (i = 0; i < 0x300; i++) ;
		// stop CPU clock
		__raw_writel(__raw_readl(REG_AHBCLK) & ~CPU_CKE, REG_AHBCLK);
	}
	for (i = 0; i < 0x300; i++) ;

	if (stop_xtal == 1) {
		// enable APLL clock
		__raw_writel(__raw_readl(REG_APLLCON) & ~PD, REG_APLLCON);
		// enable UPLL clock
		__raw_writel(__raw_readl(REG_UPLLCON) & ~PD, REG_UPLLCON);
		for (i = 0; i < 0x3000; i++) ;
		// restore system clock source to UPLL
		__raw_writel(__raw_readl(REG_CLKDIV0) | SYSTEM_S, REG_CLKDIV0);
		for (i = 0; i < 0x500; i++) ;
	}
	else {
		// enable APLL clock
		__raw_writel(__raw_readl(REG_APLLCON) & ~PD, REG_APLLCON);
		for (i = 0; i < 0x3000; i++) ;
	}

	// exit SDRAM self refresh mode
	__raw_writel(__raw_readl(REG_SDCMD) & ~SELF_REF, REG_SDCMD);
	for (i = 0; i < 0x100; i++) ;
}

#if defined(CONFIG_RTC_DRV_W55FA93)
int rtc_add_day(struct rtc_wkalrm *alrm)
{
	unsigned char mdays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	unsigned char leap_year_add_day;

	leap_year_add_day = ((__raw_readl(REG_RTC_LIR) & LEAPYEAR) && ((alrm->time).tm_mon == 1)) ? 1 : 0;
	(alrm->time).tm_mday += 1;
	if ((alrm->time).tm_mday > (mdays[(alrm->time).tm_mon] + leap_year_add_day)) {
		(alrm->time).tm_mday = 1;
		(alrm->time).tm_mon += 1;
	}
	if ((alrm->time).tm_mon > 11) {
		(alrm->time).tm_mon = 0;
		(alrm->time).tm_year += 1;
	}

	return 0;
}
#endif

#define SHUTDOWN_TIME	30				// seconds
#define SHUTDOWN_COUNT	CLOCK_TICK_RATE/HZ*SHUTDOWN_TIME	// ticks

static ssize_t
read_clk(struct device *dev, struct device_attribute *attr, char *buffer)
{
	return snprintf(buffer, PAGE_SIZE, "[UPLL]%d:[SYS]%d:[CPU]%d:[HCLK]%d:[APB]%d\n", 
			w55fa93_upll_clock, w55fa93_system_clock, w55fa93_cpu_clock, 
			w55fa93_ahb_clock, w55fa93_apb_clock);
}

// nvt_mode: 0 - standard mode (control by drivers), 1 - nuvoton mode (control by this function)
void w55fa93_pm_suspend(int nvt_mode)
{
	void (*ps_func)(u8);
	unsigned long volatile flags;
	int i;
	u32 int_mask, ahbclk, apbclk;
	u32 adc_con, audio_con, spu_dac_val, lcm_tvctl, usbd_phy_ctl, misc_ctrl;
	u8 shutdown_flag;
	u32 gpio_fun[5];
	u32 gpio_omd[5];
	u32 gpio_pull[5];
#if defined(CONFIG_RTC_DRV_W55FA93)
	struct rtc_wkalrm alrm, alrm_bak;
#endif
#if defined(CONFIG_W55FA93_USB_HOST)
	u32 rh_op_mode = 0;
#endif

	// save the GPIO settings
	for (i = 0; i < 5; i++) {
		gpio_fun[i]  = __raw_readl(REG_GPAFUN + i*0x04);
		gpio_omd[i]  = __raw_readl(REG_GPIOA_OMD + i*0x10);
		gpio_pull[i] = __raw_readl(REG_GPIOA_PUEN + i*0x10);
	}

	local_irq_save(flags);
	int_mask = __raw_readl(REG_AIC_IMR);
	// disable all interrupts
	__raw_writel(0xFFFFFFFF, REG_AIC_MDCR);
	//restore_flags(flags);

	//if (nvt_mode == 1) {
	if (1) {
		// set all SPU channels to be pause state
		__raw_writel(0xFFFFFFFF, REG_SPU_CH_PAUSE);
	}

	// save clock registers
	ahbclk = __raw_readl(REG_AHBCLK);
	apbclk = __raw_readl(REG_APBCLK);

	//if (nvt_mode == 1) {
	if (1) {
		// turn off back light
		__raw_writel(__raw_readl(REG_GPDFUN) & ~(MF_GPD1), REG_GPDFUN);
		__raw_writel(__raw_readl(REG_GPIOD_OMD) | (1<<1), REG_GPIOD_OMD);
		__raw_writel(__raw_readl(REG_GPIOD_DOUT) & ~(1<<1), REG_GPIOD_DOUT);

		// open VPOST engine clock
		__raw_writel(__raw_readl(REG_AHBCLK) | VPOST_CKE, REG_AHBCLK);

		// set VPOST to grab build in color instead of SDRAM
		__raw_writel((__raw_readl(REG_LCM_TVCtl) & ~TVCtl_LCDSrc) | 0x800, REG_LCM_TVCtl);

		// disable TV DAC
		lcm_tvctl = __raw_readl(REG_LCM_TVCtl);
		__raw_writel(__raw_readl(REG_LCM_TVCtl) | TVCtl_Tvdac, REG_LCM_TVCtl);

#if defined(CONFIG_FB_W55FA93)
		// wait vsync
		while (!(__raw_readl(REG_LCM_LCDCInt) & LCDCInt_VINT));
		__raw_writel((__raw_readl(REG_LCM_LCDCInt) & ~LCDCInt_VINT), REG_LCM_LCDCInt);
		while (!(__raw_readl(REG_LCM_LCDCInt) & LCDCInt_VINT));		
		__raw_writel((__raw_readl(REG_LCM_LCDCInt) & ~LCDCInt_VINT), REG_LCM_LCDCInt);		
#endif

		// close VPOST engine clock
		__raw_writel(__raw_readl(REG_AHBCLK) & ~VPOST_CKE, REG_AHBCLK);

		// open RTC engine clock
		__raw_writel(__raw_readl(REG_APBCLK) | RTC_CKE, REG_APBCLK);
	}

	// clear wake-up status
	__raw_writel(__raw_readl(REG_MISSR) | 0xFF000000, REG_MISSR);
#if defined(CONFIG_SND_SOC_W55FA93_ADC)
	if (__raw_readl(REG_AUDIO_CON) & AUDIO_EN) {
		// enable wake-up source
		__raw_writel((__raw_readl(REG_MISSR) & ~0x00FF0000) | (GPIO_WE|RTC_WE), REG_MISSR);
	} else
#endif
	{
		// for ADC wake up
		// enable wake-up source
		__raw_writel((__raw_readl(REG_MISSR) & ~0x00FF0000) | (ADC_WE|GPIO_WE|RTC_WE), REG_MISSR);
		// enable pull up PMOS
		//__raw_writel(__raw_readl(REG_ADC_TSC) | ADC_PU_EN, REG_ADC_TSC);
		// set ADC config for power down mode
		__raw_writel((WT_INT_EN|ADC_INT_EN|ADC_CON_ADC_EN|ADC_TSC_MODE), REG_ADC_CON);
	}

	//if (nvt_mode == 1) {
	if (1) {
		// for GPIO wake up
		// enable IRQ0 wake up and latch
		__raw_writel(0x11, REG_IRQLHSEL);
		// clear IRQ0 interrupt status
		__raw_writel(__raw_readl(REG_IRQTGSRC0) | 0x1C, REG_IRQTGSRC0);
	}

	//if (nvt_mode == 1) {
	if (1) {
#if defined(CONFIG_RTC_DRV_W55FA93)
		// for RTC wake up
		// set RTC alarm time
		memset(&alrm, 0, sizeof(struct rtc_wkalrm));
		memset(&alrm_bak, 0, sizeof(struct rtc_wkalrm));
		w55fa93_rtc_read_alarm_wrap(&alrm_bak);
		w55fa93_rtc_read_time_wrap(&alrm.time);
		alrm.enabled = 1;
		alrm.pending = 0;
#if 0
		printk("RTC = %d/%d/%d %d:%d:%d\n", alrm.time.tm_year, alrm.time.tm_mon, alrm.time.tm_mday, 
						alrm.time.tm_hour, alrm.time.tm_min, alrm.time.tm_sec);
#endif
		alrm.time.tm_sec += SHUTDOWN_TIME;
		while (alrm.time.tm_sec > 59) {
			alrm.time.tm_sec -= 60;
			alrm.time.tm_min += 1;
		}
		while (alrm.time.tm_min > 59) {
			alrm.time.tm_min -= 60;
			alrm.time.tm_hour += 1;
		}
		while (alrm.time.tm_hour > 23) {
			alrm.time.tm_hour -= 24;
			rtc_add_day(&alrm);
		}
		w55fa93_rtc_set_alarm_wrap(&alrm);
#endif
	}
	shutdown_flag = 0;

	if (nvt_mode == 1) {
	//if (1) {
#if defined(CONFIG_W55FA93_USB_HOST)
		// USB host port suspend
		if (__raw_readl(REG_HC_RH_PORT_STATUS1) & 0x01) {
			__raw_writel(0x04, REG_HC_RH_PORT_STATUS1);
			for (i = 0; i < 0x100000; i++)
				if (__raw_readl(REG_HC_RH_PORT_STATUS1) & 0x04)
			 		break;
			if (i >= 0x100000)
				printk("RH_PS_PSS not set !!\n");
			__raw_writel(0x08, REG_HC_RH_PORT_STATUS1);
		}

		// disable USBH port
		rh_op_mode = __raw_readl(REG_HC_RH_OP_MODE);
		__raw_writel( (__raw_readl(REG_HC_RH_OP_MODE) | 0x30000), REG_HC_RH_OP_MODE);
		do {
			printk("Port 1 enable state is 0x%x\n", (__raw_readl(REG_HC_RH_PORT_STATUS1) & 0x02));
			if ((__raw_readl(REG_HC_RH_PORT_STATUS1) & 0x02) == 0x02)
				mdelay(10);
		} while ((__raw_readl(REG_HC_RH_PORT_STATUS1) & 0x02) == 0x02);
		//printk("Port 1 Current Connect status 0x%x\n",(__raw_readl(REG_HC_RH_PORT_STATUS1) & 0x01));
#endif
	}

	// disable ADC and LVR
	__raw_writel(__raw_readl(REG_APBCLK) | ADC_CKE, REG_APBCLK);
	adc_con = __raw_readl(REG_ADC_CON);
	audio_con = __raw_readl(REG_AUDIO_CON);
	__raw_writel(0x0, REG_ADC_CON);
	__raw_writel(0x0, REG_AUDIO_CON);
	// enable LVR will keep about 70uA, but disable it may cause a side effect
	//__raw_writel(__raw_readl(REG_MISCR) & ~LVR_EN, REG_MISCR);
	__raw_writel(__raw_readl(REG_APBCLK) & ~ADC_CKE, REG_APBCLK);

	// disable DAC SPU HPVDD33 and ADO
	__raw_writel(__raw_readl(REG_AHBCLK) | (SPU_CKE|ADO_CKE), REG_AHBCLK);
	spu_dac_val = __raw_readl(REG_SPU_DAC_VOL);
	__raw_writel(spu_dac_val | ANA_PD, REG_SPU_DAC_VOL);
	__raw_writel(__raw_readl(REG_AHBCLK) & ~(SPU_CKE|ADO_CKE), REG_AHBCLK);

	// disable USB phy
	__raw_writel(__raw_readl(REG_AHBCLK) | USBD_CKE, REG_AHBCLK);
	usbd_phy_ctl = __raw_readl(REG_USBD_PHY_CTL);
	__raw_writel(0x0, REG_USBD_PHY_CTL);
	__raw_writel(__raw_readl(REG_AHBCLK) & ~USBD_CKE, REG_AHBCLK);

	// disable USB Host Transceiver
	__raw_writel(__raw_readl(REG_AHBCLK) | USBH_CKE, REG_AHBCLK);
	misc_ctrl = __raw_readl(USBH_BA+0x200);
	__raw_writel(BIT27, USBH_BA+0x200);
	__raw_writel(__raw_readl(REG_AHBCLK) & ~USBH_CKE, REG_AHBCLK);

#if 0
	// close audio engine clocks
	__raw_writel(__raw_readl(REG_AHBCLK) & ~(ADO_CKE|I2S_CKE|SPU_CKE), REG_AHBCLK);
	// close all edma engine clocks
	__raw_writel(__raw_readl(REG_AHBCLK) & ~(EDMA0_CKE|EDMA1_CKE|EDMA2_CKE|EDMA3_CKE|EDMA4_CKE), REG_AHBCLK);
#else
	//set I2C multi-function pin to GPIO
	__raw_writel(__raw_readl(REG_APBCLK) | I2C_CKE, REG_APBCLK);
	__raw_writel(__raw_readl(REG_GPBFUN) & ~0x3C000000, REG_GPBFUN);
	__raw_writel(__raw_readl(REG_APBCLK) & ~I2C_CKE, REG_APBCLK);
	
	// close all engine clocks
	__raw_writel(0x0000011F , REG_AHBCLK);
#endif

#if 1
	// change SD card pin function
	// GPA1 for SD-0 card detection
	__raw_writel(__raw_readl(REG_GPAFUN) & ~MF_GPA1, REG_GPAFUN);
	// SD0_CLK/CMD/DAT0_3 pins selected
	__raw_writel(__raw_readl(REG_GPEFUN) & ~0x0000FFF0, REG_GPEFUN);
	__raw_writel(__raw_readl(REG_GPAFUN) & ~MF_GPA0, REG_GPAFUN);	
	
	__raw_writel((MF_GPA11|MF_GPA10), REG_GPAFUN);
	__raw_writel(0x0, REG_GPBFUN);
	__raw_writel(0x0, REG_GPCFUN);
	__raw_writel((MF_GPD4|MF_GPD3|MF_GPD2|MF_GPD1|MF_GPD0), REG_GPDFUN);
	__raw_writel(0x0, REG_GPEFUN);

#if 0
	printk("GPIOA STATUS = 0x%x\n", __raw_readl(REG_GPIOA_PIN));
	printk("GPIOB STATUS = 0x%x\n", __raw_readl(REG_GPIOB_PIN));
	printk("GPIOC STATUS = 0x%x\n", __raw_readl(REG_GPIOC_PIN));
	printk("GPIOD STATUS = 0x%x\n", __raw_readl(REG_GPIOD_PIN));
	printk("GPIOE STATUS = 0x%x\n", __raw_readl(REG_GPIOE_PIN));
#endif
	__raw_writel(0x0, REG_GPIOA_OMD);
	__raw_writel(0x0, REG_GPIOB_OMD);
	__raw_writel(0x0, REG_GPIOC_OMD);
	__raw_writel(0x0, REG_GPIOD_OMD);
	__raw_writel(0x0, REG_GPIOE_OMD);
	__raw_writel(0x3FF, REG_GPIOA_PUEN);
	__raw_writel(0xFFFF, REG_GPIOB_PUEN);
	__raw_writel(0xFFFF, REG_GPIOC_PUEN);
	__raw_writel(0xFFE0, REG_GPIOD_PUEN);
	__raw_writel(0x0FFF, REG_GPIOE_PUEN);	
#endif

	// kick into power down mode, make SDRAM enter self refresh mode
	// put enter_power_saving into SRAM
	memcpy(sram_vaddr, enter_power_saving, 1024);
	ps_func = (void(*)(u8)) sram_vaddr;
	// flush all TLB cache entries
	local_flush_tlb_all();

	//if (nvt_mode == 1) {
	if (0) {
#if defined(CONFIG_W55FA93_USB_HOST)
		// USB tranceiver standby enable
//		__raw_writel(BIT27, USBH_BA+200);
		// close USB host engine clocks
		__raw_writel(ahbclk & ~(USBH_CKE), REG_AHBCLK);
#endif
	}

	// enter to power down mode
	ps_func(0x1);

	// resotre the GPIO settings
	for (i = 0; i < 5; i++) {
		__raw_writel(gpio_fun[i], (REG_GPAFUN + i*0x04));
		__raw_writel(gpio_omd[i], (REG_GPIOA_OMD + i*0x10));
		__raw_writel(gpio_pull[i], (REG_GPIOA_PUEN + i*0x10));
	}

	printk("REG_MISSR=0x%x\n", __raw_readl(REG_MISSR));
#if defined(CONFIG_W55FA93_SYSMGR) || defined(CONFIG_W55FA93_SYSMGR_MODULE)
	if (__raw_readl(REG_MISSR) & RTC_WS) {
//		shutdown_flag = 1;
		sysmgr_report(SYSMGR_STATUS_RTC_POWER_OFF);
		printk("sysmgr_report(SYSMGR_STATUS_RTC_POWER_OFF)\n");
	}
#endif
	// clear wake-up status
	__raw_writel(__raw_readl(REG_MISSR) | 0xFF000000, REG_MISSR);

	// restore ADC and LVR
	__raw_writel(__raw_readl(REG_APBCLK) | ADC_CKE, REG_APBCLK);
	//printk("adc_con=0x%x\n", adc_con);
	__raw_writel(adc_con, REG_ADC_CON);
	//printk("REG_ADC_CON=0x%x\n", __raw_readl(REG_ADC_CON));
	//printk("audio_con=0x%x\n", audio_con);
	__raw_writel(audio_con, REG_AUDIO_CON);
	//printk("REG_AUDIO_CON=0x%x\n", __raw_readl(REG_AUDIO_CON));
	//__raw_writel(__raw_readl(REG_MISCR) & ~LVR_EN, REG_MISCR);

	// restore DAC SPU HPVDD33 and ADO
	__raw_writel(__raw_readl(REG_AHBCLK) | (HCLK4_CKE|SPU_CKE|ADO_CKE), REG_AHBCLK);
	//printk("spu_dac_val=0x%x\n", spu_dac_val);
	__raw_writel(spu_dac_val, REG_SPU_DAC_VOL);
	//printk("REG_SPU_DAC_VOL=0x%x\n", __raw_readl(REG_SPU_DAC_VOL));

	// restore TV DAC
	__raw_writel(__raw_readl(REG_AHBCLK) | VPOST_CKE, REG_AHBCLK);
	//printk("lcm_tvctl=0x%x\n", lcm_tvctl);
	__raw_writel(lcm_tvctl, REG_LCM_TVCtl);
	//printk("REG_LCM_TVCtl=0x%x\n", __raw_readl(REG_LCM_TVCtl));

	// restore USB phy
	__raw_writel(__raw_readl(REG_AHBCLK) | (HCLK3_CKE|USBD_CKE), REG_AHBCLK);
	//printk("usbd_phy_ctl=0x%x\n", usbd_phy_ctl);
	__raw_writel(usbd_phy_ctl, REG_USBD_PHY_CTL);
	//printk("REG_USBD_PHY_CTL=0x%x\n", __raw_readl(REG_USBD_PHY_CTL));

	// restore USB Host Transceiver
	__raw_writel(__raw_readl(REG_AHBCLK) | USBH_CKE, REG_AHBCLK);
	//printk("misc_ctrl=0x%x\n", misc_ctrl);
	__raw_writel(misc_ctrl, USBH_BA+0x200);
	//printk("REG_MISC_CTRL=0x%x\n", __raw_readl(USBH_BA+0x200));

	//if (nvt_mode == 1) {
	if (1) {
#if defined(CONFIG_RTC_DRV_W55FA93)
		// restore RTC alarm time
		w55fa93_rtc_set_alarm_wrap(&alrm_bak);
#endif
	}

	//if (nvt_mode == 1) {
	if (1) {
		if (ahbclk & VPOST_CKE) {
			__raw_writel(__raw_readl(REG_AHBCLK) | VPOST_CKE, REG_AHBCLK);
#if defined(CONFIG_HANNSTARR_HSD043I9W1_480x272)
			mdelay(300);
#elif defined(CONFIG_GOWORLD_GW8973_480x272)
			mdelay(200);
#endif
		}
	}

	// restore registers
	__raw_writel(ahbclk, REG_AHBCLK);
	__raw_writel(apbclk, REG_APBCLK);

	if (nvt_mode == 1) {
	//if (1) {
#if defined(CONFIG_W55FA93_USB_HOST)
		// enable USBH port
		__raw_writel(rh_op_mode, REG_HC_RH_OP_MODE);
		do {
			printk("Port 1 enable state is 0x%x\n", (__raw_readl(REG_HC_RH_PORT_STATUS1) & 0x02));
			if ((__raw_readl(REG_HC_RH_PORT_STATUS1) & 0x02) == 0x02)
				mdelay(10);
		} while ((__raw_readl(REG_HC_RH_PORT_STATUS1) & 0x02) == 0x02);

		// USB tranceiver standby disable
		//__raw_writel(0x0, USBH_BA+200);

		// clear USB host suspend status
		if (__raw_readl(REG_HC_RH_PORT_STATUS1) & 0x01) {
			__raw_writel(0x08, REG_HC_RH_PORT_STATUS1);
			for (i = 0; i < 0x100000; i++)
				if (!(__raw_readl(REG_HC_RH_PORT_STATUS1) & 0x04)) {
					__raw_writel(0x40000, REG_HC_RH_PORT_STATUS1);
					break;
				}
			if (i >= 0x100000)
				printk("Port 1 is still suspend !!\n");
		}
#endif
	}

	//if (nvt_mode == 1) {
	if (1) {
		// set all SPU channels to be normal state
		__raw_writel(0x00000000, REG_SPU_CH_PAUSE);
	}

	//save_flags(flags);
	//cli();
	// restore interrupt mask
	__raw_writel(int_mask, REG_AIC_MECR);
	local_irq_restore(flags);

	//if (nvt_mode == 1) {
	if (1) {
		if (shutdown_flag == 0) {
			// VPOST get SDRAM data
			__raw_writel((__raw_readl(REG_LCM_TVCtl) & ~TVCtl_LCDSrc) | 0x400, REG_LCM_TVCtl);

			// turn on back light
			__raw_writel(__raw_readl(REG_GPIOD_DOUT) | (1<<1), REG_GPIOD_DOUT);
		}
	}
}

// you can decide what clocks are disabled in w55fa93_pm_idle()
void w55fa93_pm_idle(void)
{
#if 0
	u32 int_mask, ahbclk, apbclk, tcsr0, ticr0;
	u8 shutdown_flag, skip_check_shutdown, do_half_clock;

	if (buffer[0] == 'm' && buffer[1] == 'i') {
		// turn off back light
		__raw_writel(__raw_readl(REG_GPDFUN) & ~(MF_GPD1), REG_GPDFUN);
		__raw_writel(__raw_readl(REG_GPIOD_OMD) | (1<<1), REG_GPIOD_OMD);
		__raw_writel(__raw_readl(REG_GPIOD_DOUT) & ~(1<<1), REG_GPIOD_DOUT);

		// set VPOST to grab build in color instead of SDRAM
		__raw_writel((__raw_readl(REG_LCM_TVCtl) & ~TVCtl_LCDSrc) | 0x800, REG_LCM_TVCtl);
	}

	local_irq_save(flags);
	int_mask = __raw_readl(REG_AIC_IMR);
	// disable all interrupts
	__raw_writel(0xFFFFFFFF, REG_AIC_MDCR);
	//restore_flags(flags);

	// set all SPU channels to be pause state
	__raw_writel(0xFFFFFFFF, REG_SPU_CH_PAUSE);
	// close unused engine clocks
	ahbclk = __raw_readl(REG_AHBCLK);
	apbclk = __raw_readl(REG_APBCLK);
	if (buffer[0] == 'i' && buffer[1] == 'd') {
		// make sure no DMA transaction in progress
		mdelay(100);

		__raw_writel(ahbclk & ~(SEN_CKE|CAP_CKE|SD_CKE|NAND_CKE|SIC_CKE|JPG_CKE), REG_AHBCLK);
		__raw_writel(apbclk & ~(KPI_CKE|WDCLK_CKE|SPIMS1_CKE|SPIMS0_CKE), REG_APBCLK);
	}
	else if (buffer[0] == 'm' && buffer[1] == 'i') {
		// make sure no DMA transaction in progress
		mdelay(100);
		for (i = 0; i < 5; i++)
			while (w55fa93_edma_isbusy(i));

		__raw_writel(ahbclk & ~(SEN_CKE|CAP_CKE|VPOST_CKE|SD_CKE|NAND_CKE|SIC_CKE|EDMA4_CKE|EDMA3_CKE|EDMA2_CKE|EDMA1_CKE|EDMA0_CKE|JPG_CKE|BLT_CKE), REG_AHBCLK);
		__raw_writel(apbclk & ~(KPI_CKE|WDCLK_CKE|SPIMS1_CKE|SPIMS0_CKE|PWM_CKE), REG_APBCLK);
	}
	// divide clocks by 2, ex: drop system clock from 240MHz to 120MHz
	do_half_clock = 0;
	if ((w55fa93_system_clock == 240000) || (w55fa93_system_clock == 192000)) {
		set_system_clocks(w55fa93_system_clock/2, w55fa93_cpu_clock/2, w55fa93_apb_clock/2);
		do_half_clock = 1;
	}

	// store timer register
	tcsr0 = __raw_readl(REG_TCSR0);
	ticr0 = __raw_readl(REG_TICR0);
	// reset timer
	__raw_writel(__raw_readl(REG_APBIPRST) | TMR0RST, REG_APBIPRST);
	__raw_writel(__raw_readl(REG_APBIPRST) & ~TMR0RST, REG_APBIPRST);

	// wait reset complete
	for (i = 0; i < 0x1000; i++) ;
	__raw_writel(0x01, REG_TISR);
	__raw_writel(SHUTDOWN_COUNT, REG_TICR0);
	__raw_writel(0x60010063, REG_TCSR0);

	// enable wake up interrupts
	__raw_writel((1<<IRQ_ADC)|(1<<IRQ_GPIO0)|(1<<IRQ_TIMER0), REG_AIC_MECR);

	shutdown_flag = 0;
	skip_check_shutdown = 0;

	// close audio engine clocks
	__raw_writel(ahbclk & ~(ADO_CKE|I2S_CKE|SPU_CKE), REG_AHBCLK);
	// avoid key and touch pressing
#if defined(CONFIG_W55FA93_KEYPAD) && defined(CONFIG_TOUCHSCREEN_W55FA93)
	if ((w55fa93_key_pressing == 0) && (w55fa93_ts_pressing == 0)) {
#elif defined(CONFIG_W55FA93_KEYPAD)
	if (w55fa93_key_pressing == 0) {
#elif defined(CONFIG_TOUCHSCREEN_W55FA93)
	if (w55fa93_ts_pressing == 0) {
#else
	if (1) {
#endif
		if (buffer[0] == 'i' && buffer[1] == 'd') {
			// stop APLL clcok
			__raw_writel(__raw_readl(REG_APLLCON) | PD, REG_APLLCON);
			for (i = 0; i < 0x300; i++) ;
			// stop CPU clock
			__raw_writel(__raw_readl(REG_AHBCLK) & ~CPU_CKE, REG_AHBCLK);
			for (i = 0; i < 0x300; i++) ;
			// enable APLL clock
			__raw_writel(__raw_readl(REG_APLLCON) & ~PD, REG_APLLCON);
			for (i = 0; i < 0x3000; i++) ;
		}
		else if (buffer[0] == 'm' && buffer[1] == 'i') {
			// kick into memory idle mode, make SDRAM enter self refresh mode
			// put enter_power_saving into SRAM
			memcpy(sram_vaddr, enter_power_saving, 1024);
			ps_func = (void(*)(u8)) sram_vaddr;

			// flush all TLB cache entries
			local_flush_tlb_all();
			// enter to memory idle mode
			ps_func(0x0);
		}
	}
	else
		skip_check_shutdown = 1;

	__raw_writel(0x20000063, REG_TCSR0);
	if ((__raw_readl(REG_TDR0) == 0x1) && (!skip_check_shutdown))
		shutdown_flag = 1;

	if ((ahbclk & VPOST_CKE) && !(__raw_readl(REG_AHBCLK) & VPOST_CKE)) {
		__raw_writel(__raw_readl(REG_AHBCLK) | VPOST_CKE, REG_AHBCLK);
#if defined(CONFIG_HANNSTARR_HSD043I9W1_480x272)
		mdelay(300);
#elif defined(CONFIG_GOWORLD_GW8973_480x272)
		mdelay(200);
#endif
	}

	// restore clocks to full speed
	if (do_half_clock)
		set_system_clocks(w55fa93_system_clock*2, w55fa93_cpu_clock*2, w55fa93_apb_clock*2);
	// restore registers
	__raw_writel(ahbclk, REG_AHBCLK);
	__raw_writel(apbclk, REG_APBCLK);
	// set all SPU channels to be normal state
	__raw_writel(0x00000000, REG_SPU_CH_PAUSE);

	if (shutdown_flag == 1) {
#if defined(CONFIG_W55FA93_SYSMGR) || defined(CONFIG_W55FA93_SYSMGR_MODULE)
		sysmgr_report(SYSMGR_STATUS_RTC_POWER_OFF);
		printk("sysmgr_report(SYSMGR_STATUS_RTC_POWER_OFF)\n");
#endif
	}

	// reset timer
	__raw_writel(__raw_readl(REG_APBIPRST) | TMR0RST, REG_APBIPRST);
	__raw_writel(__raw_readl(REG_APBIPRST) & ~TMR0RST, REG_APBIPRST);
	// wait reset complete
	for (i = 0; i < 0x1000; i++) ;
	// store timer register
	__raw_writel(0x01, REG_TISR);
	__raw_writel(tcsr0, REG_TCSR0);
	__raw_writel(ticr0, REG_TICR0);

	//save_flags(flags);
	//cli();
	__raw_writel(0xFFFFFFFF, REG_AIC_MDCR);
	// restore interrupt mask
	__raw_writel(int_mask, REG_AIC_MECR);
	local_irq_restore(flags);

	if (shutdown_flag == 0) {
		// VPOST get SDRAM data
		__raw_writel((__raw_readl(REG_LCM_TVCtl) & ~TVCtl_LCDSrc) | 0x400, REG_LCM_TVCtl);

		// turn on back light
		__raw_writel(__raw_readl(REG_GPIOD_DOUT) | (1<<1), REG_GPIOD_DOUT);
	}
#endif
}

void w55fa93_poweroff(void)
{
	unsigned long volatile flags;
	int rtc_time_out;

	printk("enter to w55fa93_poweroff()\n");
	msleep(10);

	// disable LVR
	__raw_writel(__raw_readl(REG_MISCR) & ~(LVR_RDY | LVR_EN), REG_MISCR);

	// turn off speaker
#if defined(CONFIG_HANNSTARR_HSD043I9W1_480x272)
	__raw_writel(__raw_readl(REG_GPIOB_OMD) | (1 << 3), REG_GPIOB_OMD);
	__raw_writel(__raw_readl(REG_GPIOB_DOUT) & ~(1 << 3), REG_GPIOB_DOUT);
#elif defined(CONFIG_GOWORLD_GWMTF9360A_320x240)
	__raw_writel(__raw_readl(REG_GPIOE_OMD) | (1 << 1), REG_GPIOE_OMD);
	__raw_writel(__raw_readl(REG_GPIOE_DOUT) & ~(1 << 1), REG_GPIOE_DOUT);
#endif

	// turn off video out
	__raw_writel((__raw_readl(REG_LCM_TVCtl) & ~TVCtl_LCDSrc) | 0x800, REG_LCM_TVCtl);

	// disable system interrupts
	local_irq_save(flags);

#if defined(CONFIG_RTC_DRV_W55FA93)
	__raw_writel(__raw_readl(REG_APBCLK) | RTC_CKE, REG_APBCLK);
	while (1) {
		rtc_time_out = 0;
		// enable long time press power disable
		if ((__raw_readl(REG_RTC_AER) & 0x10000) == 0x0) {
			// set RTC register access enable password
			__raw_writel(INIRRESET, REG_RTC_INIR); /* init value */
			__raw_writel(0xA965, REG_RTC_AER);
			// make sure RTC register read/write enable
			while ((__raw_readl(REG_RTC_AER) & 0x10000) == 0x0) {
				rtc_time_out++;
				if (rtc_time_out > 0xFFFFFF) {
					printk("RTC Access Eanble Fail\n");
					break;
				}
			}

			// FA93 does not have REG_RTC_REG_FLAG
			//rtc_wait_ready();

			if ((__raw_readl(REG_RTC_AER) & 0x10000) == 0x10000)
				break;
		}
		else
			break;
	}

	// RTC will power off
	__raw_writel((__raw_readl(REG_RTC_PWRON) & ~0x5) | 0x2, REG_RTC_PWRON);
#else
	// turn off power
	__raw_writel(__raw_readl(REG_GPIOD_OMD) | (1<<12), REG_GPIOD_OMD);
	__raw_writel(__raw_readl(REG_GPIOD_DOUT) & ~(1<<12), REG_GPIOD_DOUT);
#endif

	// enable system interrupts
	local_irq_restore(flags);

	// stop CPU clock
	//__raw_writel(__raw_readl(REG_AHBCLK) & ~CPU_CKE, REG_AHBCLK);
	// fix RTC may wakeup fail issue
	__raw_writel(0x0, REG_AHBCLK);

	// wait system enter power off
	while (1) ;
}

void w55fa93_reboot(void)
{
	unsigned long volatile flags;

	local_irq_save(flags);
	printk("enter to w55fa93_reboot()\n");

	// turn off speaker
#if defined(CONFIG_HANNSTARR_HSD043I9W1_480x272)
	__raw_writel(__raw_readl(REG_GPIOB_OMD) | (1 << 3), REG_GPIOB_OMD);
	__raw_writel(__raw_readl(REG_GPIOB_DOUT) & ~(1 << 3), REG_GPIOB_DOUT);
#elif defined(CONFIG_GOWORLD_GWMTF9360A_320x240)
	__raw_writel(__raw_readl(REG_GPIOE_OMD) | (1 << 1), REG_GPIOE_OMD);
	__raw_writel(__raw_readl(REG_GPIOE_DOUT) & ~(1 << 1), REG_GPIOE_DOUT);
#endif

	// turn off video out
	__raw_writel((__raw_readl(REG_LCM_TVCtl) & ~TVCtl_LCDSrc) | 0x800, REG_LCM_TVCtl);

	// turn off power
	__raw_writel((__raw_readl(REG_WTCR) & ~(3<<4|1<<10))|0x2C2, REG_WTCR);

	// wait system enter power off
	while (1) ;
	local_irq_restore(flags);
}

static ssize_t
write_clk(struct device *dev, struct device_attribute *attr,
	  const char *buffer, size_t count)
{
	// power down mode
	if (buffer[0] == 'p' && buffer[1] == 'd') {
		w55fa93_pm_suspend(1);
	}

#if 0
	// idle mode or memory idle mode
	else if ((buffer[0] == 'i' && buffer[1] == 'd') || (buffer[0] == 'm' && buffer[1] == 'i')) {
		w55fa93_pm_idle();
	}
#endif

#if defined(CONFIG_RTC_DRV_W55FA93)
	// RTC power off mode
	else if (buffer[0] == 'r' && buffer[1] == 'p' && buffer[2] == 'o') {
		w55fa93_poweroff();
	}
#else
	// power off mode
	else if (buffer[0] == 'p' && buffer[1] == 'o') {
		w55fa93_poweroff();
	}
#endif

	// power reset mode
	else if (buffer[0] == 'p' && buffer[1] == 'r') {
		w55fa93_reboot();
	}

	// CPU:PLL clock change
	else {
		u32 pll_clock, sys_clock, cpu_clock, apb_clock;
		char clock_buf[64];
		char *clock1, *clock2, *next;

		strncpy(clock_buf, buffer, count);
		next = &clock_buf[0];
		pll_clock = w55fa93_upll_clock;
		clock1 = strsep(&next, ":");
//printk("clock1 = %s\n", clock1);
		cpu_clock = simple_strtol(clock1, NULL, 10) * 1000;
		if (cpu_clock == 0) {
			printk("Command \"%s\" does not support !!\n", clock1);
			return -1;
		}
		if (next) {
			clock2 = strsep(&next, ":");
//printk("clock2 = %s\n", clock2);
			pll_clock = simple_strtol(clock2, NULL, 10) * 1000;
			if (pll_clock == 0) {
				printk("Command \"%s\" does not support !!\n", clock2);
				return -1;
			}
		}

		if (pll_clock % cpu_clock) {
			printk("UPLL clock(%d) is not a multiple of CPU clock(%d) !!\n", 
				pll_clock, cpu_clock);
			return -1;
		}
		if ((pll_clock / cpu_clock) > 8) {
			printk("Cannot get valid System clcok !!\n");
			return -1;
		}
		sys_clock = cpu_clock;
		apb_clock = (cpu_clock == sys_clock) ? cpu_clock/4 : cpu_clock/2;

		// PLL:SYS:CPU:AHB:APB = pll_clock:sys_clock:cpu_clock:sys_clock/2:apb_clock
		set_system_clocks(pll_clock, sys_clock, cpu_clock, apb_clock);
	}

	return count;
}

/* Attach the sysfs write method */
DEVICE_ATTR(clock, 0644, read_clk, write_clk);

/* Attribute Descriptor */
static struct attribute *clk_attrs[] = {
	&dev_attr_clock.attr,
	NULL
};

/* Attribute group */
static struct attribute_group clk_attr_group = {
	.attrs = clk_attrs,
};

static int __init w55fa93_system_clock_init(void)
{
	/* Register a platform device */
	printk("register clock device\n");

	sys_clk = platform_device_register_simple("w55fa93-clk", -1, NULL, 0);
	if (sys_clk == NULL)
		printk("register failed\n");
	sysfs_create_group(&sys_clk->dev.kobj, &clk_attr_group);
	sram_vaddr = ioremap(0xFF000000, 4*1024);

	return 0;
}

module_init(w55fa93_system_clock_init);

#if 0
static int __init w55fa93_arch_init(void)
{
	int ret;
//	struct platform_device **ptr = w55fa93_board.devices;
	struct platform_device *ptr;
	ptr= &w55fa93_lcddevice;
	ret = platform_device_register(&w55fa93_lcddevice);
	printk("### Call platform_device_register in %s \n", __FUNCTION__);
	if (ret) {
		printk(KERN_ERR "w55fa93: failed to add board device %s (%d) @%p\n", (ptr)->name, ret, ptr);
	}

	return 0;
}

arch_initcall(w55fa93_arch_init);
#endif
