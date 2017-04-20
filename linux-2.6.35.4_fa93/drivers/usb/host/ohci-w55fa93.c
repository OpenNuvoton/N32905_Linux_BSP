/***********************************************************************
 *
 * 
 * Copyright (c) 2008 Nuvoton Technology
 * All rights reserved.
 *
 * 
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 * Changelog:
 *  
 *
 ***********************************************************************/


#include <linux/platform_device.h>
#include <linux/signal.h>

#include <linux/clk.h>
#include <mach/w55fa93_reg.h>
//#include <asm/arch/w55fa93_reg.h>
//#include <asm/arch/regs-clock.h>
//#include <asm/arch/regs-gcr.h>

#define DBG_PRINTF			printk
//#define DBG_PRINTF(...)

struct clk  *usbh11_clk;

static void usbh_DisablePorts(
	u8 bDisablePort1,
	u8 bDisablePort2
)
{
	writel(  (readl(REG_HC_RH_OP_MODE) & ~(DISPRT2 | DISPRT1)) |  \
						(((bDisablePort1 & 1)<<16) | ((bDisablePort2 & 1)<<17)), \
						REG_HC_RH_OP_MODE); 	
}
static void usbh_LikeModePort(unsigned int u32HlmPort)
{
	if(u32HlmPort==1)
	{
		writel ( (readl(REG_GPBFUN) & ~(MF_GPB1 | MF_GPB0)) | 0x05, REG_GPBFUN); 
	}
	else if(u32HlmPort==2)
	{
		writel ( (readl(REG_GPAFUN) & ~(MF_GPA4 | MF_GPA3)) | (0x0A<<6), REG_GPAFUN); 
	}
}

static int  get_pll(void)
{
	__u32 u32PLLSetting; 	
	int NF, NR, NO;
	int map[4]= { 1, 2, 2, 4};
	unsigned int u32PorSetting;
	unsigned  int u32ExtClk;
	DBG_PRINTF("%s\n",__FUNCTION__);	
	u32PorSetting = (readl(REG_CHIPCFG) & COPMODE)>>2;
	if(u32PorSetting==2)
	{
		printk("External clock = 12MHz\n");
		u32ExtClk = 12000000;
	}
	else if(u32PorSetting==3)
	{
		printk("External clock = 27MHz\n");
		u32ExtClk = 27000000;
	}
	else
	{//Default set external clock is 12MHz
		printk("Unknown external clock\n");
		u32ExtClk = 12000000;
	}
	/* USB host always come from UPLL */ 
	u32PLLSetting = readl(REG_UPLLCON) & 
						(OUT_DV | IN_DV | FB_DV);

	NF = (u32PLLSetting & FB_DV) + 2;
	NR = ((u32PLLSetting & IN_DV) >>9) + 2;
	NO = (u32PLLSetting & OUT_DV)>>14;
	NO = map[NO];
	DBG_PRINTF("NF: %d, NR: %d, NO: %d\n", NF, NR, NO);
	return (u32ExtClk / (NR * NO)) * NF;
}



/**
 * usb_hcd_ppc_soc_probe - initialize On-Chip HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller.
 *
 * Store this function in the HCD's struct pci_driver as probe().
 */

static int usb_hcd_w55fa93_probe(const struct hc_driver *driver,
			  struct platform_device *pdev)
{
	int retval;
	struct usb_hcd *hcd;
	struct ohci_hcd *ohci ;
	unsigned long pll;
	int	div, div0, div1;


	DBG_PRINTF("%s\n",__FUNCTION__);	
	
        usbh11_clk = clk_get(&pdev->dev, NULL);
        if (IS_ERR(usbh11_clk)) {
              
		printk("failed to get usbh1.1 clock\n");
		goto  err2;
        }

	/*
	 * Config GPIO
	 */
	//writel(readl(REG_GPIOA_OMD) & ~0x01, REG_GPIOA_OMD);	// GPIOA0 input mode
	//writel(readl(REG_GPIOB_OMD) | 0x5000, REG_GPIOB_OMD);	// GPIOB12,B14 output mode
	//writel(readl(REG_GPIOB_DOUT) | 0x5000, REG_GPIOB_DOUT);	// GPIOB12,B14 output high

	/* 
	 * Config multi-function
	 */	
	// No over currnet pin assigned on EV board
	// writel((readl(REG_PINFUN) & 0x00ffffff)/* | 0x11000000 */, REG_PINFUN);

	pll = get_pll() / 1000000;
	printk("PLL is %d   ---\n", (int)pll);	

	if((pll%48)!=0)
	{
		printk("Error found! Please set the UPLL as multiple of 48MHz\n");	
		return -ENOMEM;
	}
	/* set clock dividor */
	div = (pll / 48);

	for(div1=1; div1<17; div1=div1+1)
	{
		DBG_PRINTF("div1 = %d\n", div1);	
		for(div0=1; div0<9; div0=div0+1)
		{
			DBG_PRINTF("div0 = %d\n", div0);	
			if(div ==(div0*div1))
				break;
		}	
		if(div ==(div0*div1))
		{			
			div0 = div0-1;
			div1 = div1-1;
			break;
		}
	}

	DBG_PRINTF("Div1 = %d,  Div0 = %d\n", div1, div0);		
	writel( (readl(REG_CLKDIV2) & ~(USB_N1 | USB_S | USB_N0)) | 
					( ((div1 << 8) | (3<<3)) | (div0) ), 
					REG_CLKDIV2);

//#if defined(CONFIG_W55FA93_USB_HOST_PORT1_DISABLE) || 
//	defined(CONFIG_W55FA93_USB_HOST_PORT2_DISABLE) 
	
	usbh_DisablePorts(0, 0);	//Default two port enable	
//#endif

	
        clk_enable(usbh11_clk);
	/* enable USB Host clock (UHC_EN, USB48_EN) */
//	writel(readl(REG_AHBCLK) | USBH_CKE, 
//			REG_AHBCLK);
	/* reset USB host */
	writel( readl(REG_AHBIPRST) | UHCRST, 
			REG_AHBIPRST);
	writel( readl(REG_AHBIPRST) & ~UHCRST, 
			REG_AHBIPRST);
	DBG_PRINTF("USBH IP  Reset\n");

#if defined(CONFIG_W55FA93_USB_HOST_LIKE_PORT1) || \
	defined(CONFIG_W55FA93_USB_HOST_LIKE_PORT2)
	printk("CONFIG_W55FA93_USB_HOST_LIKE_PORT1\n");
	usbh_DisablePorts(0, 1);	/* Host like port 1 and 2 through host port 1*/
#endif

#ifdef CONFIG_W55FA93_USB_HOST_PORT1_DISABLE
	printk("CONFIG_W55FA93_USB_HOST_PORT1_DISABLE\n");
	usbh_DisablePorts(1, 0);	//1: Disable, 0:Enable ==>Disable port 1
#endif
#ifdef CONFIG_W55FA93_USB_HOST_PORT2_DISABLE
	printk("CONFIG_W55FA93_USB_HOST_PORT2_DISABLE\n");
	usbh_DisablePorts(0, 1);	//1: Disable, 0:Enable ==>Disabke port 2
#endif
	
	// Set 0x08 for over current low active, 0 for high active
	// writel(0x08, W55FA93_VA_USBH_BASE + 0x204);

	hcd = usb_create_hcd(driver, &pdev->dev, "w55fa93-ohci");
	if (!hcd)
		return -ENOMEM;
		
	hcd->rsrc_start = pdev->resource[0].start;
	hcd->rsrc_len = pdev->resource[0].end - pdev->resource[0].start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		printk(__FILE__ ": request_mem_region failed\n");
		retval = -EBUSY;
		goto err1;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		printk(__FILE__ ": ioremap failed\n");
		retval = -ENOMEM;
		goto err2;
	}

	ohci = hcd_to_ohci(hcd);
	ohci_hcd_init(ohci);

	retval = usb_add_hcd(hcd, pdev->resource[1].start, IRQF_DISABLED);

	//return 0;
	
	if (retval == 0)
		return retval;

	printk("Due to some thing wrong! Removing W55FA93 USB Controller\n");

	iounmap(hcd->regs);
 err2:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
 err1:
 
 	usb_put_hcd(hcd);
	return retval;
}


/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_ppc_soc_remove - shutdown processing for On-Chip HCDs
 * @pdev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_ppc_soc_probe().
 * It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
static void usb_hcd_w55fa93_remove(struct usb_hcd *hcd,
		struct platform_device *dev)
{
	DBG_PRINTF("%s\n",__FUNCTION__);		
	usb_remove_hcd(hcd);

	//pr_debug("stopping W55FA93 USB Controller\n");

	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
}


static int ohci_w55fa93_start (struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci (hcd);
	int ret;
	DBG_PRINTF("%s\n",__FUNCTION__);	
	if ((ret = ohci_init(ohci)) < 0)
		return ret;

	if ((ret = ohci_run (ohci)) < 0) {
		printk ("can't start %s", hcd->self.bus_name);
		ohci_stop (hcd);
		return ret;
	}

	return 0;
}


static const struct hc_driver ohci_w55fa93_hc_driver = {
	.description =		hcd_name,
	.product_desc = 	"Nuvoton W55FA93 OHCI Host Controller",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.start =        ohci_w55fa93_start,
	.stop =			ohci_stop,
	.shutdown = 		ohci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend =		ohci_bus_suspend,
	.bus_resume =		ohci_bus_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};


static int ohci_hcd_w55fa93_drv_probe(struct platform_device *pdev)
{
	int ret;
	DBG_PRINTF("%s\n",__FUNCTION__);	
	if (usb_disabled())
		return -ENODEV;

	ret = usb_hcd_w55fa93_probe(&ohci_w55fa93_hc_driver, pdev);
	return ret;
}

static int ohci_hcd_w55fa93_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_hcd_w55fa93_remove(hcd, pdev);
	return 0;
}

static struct platform_driver ohci_hcd_w55fa93_driver = {
	.probe		= ohci_hcd_w55fa93_drv_probe,
	.remove		= ohci_hcd_w55fa93_drv_remove,
#ifdef	CONFIG_PM

#endif
	.driver		= {
		.name	= "w55fa93-ohci",
		.owner	= THIS_MODULE,
	},
};



