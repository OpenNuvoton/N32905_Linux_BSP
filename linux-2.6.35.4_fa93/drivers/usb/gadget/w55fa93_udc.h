/* linux/include/asm-arm/arch-W55FA93/w55fa93_reg.h
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
 *   2006/08/26     vincen.zswan add this file for nuvoton W55FA93 MCU ip REG.
 */
#ifndef _W55FA93_UDC_H
#define _W55FA93_UDC_H

#define DMA_ADDR_INVALID (~(dma_addr_t)0)

#define W55FA93_ENDPOINTS       7

struct w55fa93_ep {
        struct list_head		queue;
        struct usb_gadget		*gadget;
        struct w55fa93_udc		*dev;
        const struct usb_endpoint_descriptor *desc;
        struct usb_ep			ep;
        u8				index;

        u8				buffer_disabled;
        u8				bEndpointAddress;//w/ direction

        u8 EP_Mode;//auto/manual/fly
        u8 EP_Num;//no direction ep address
        u8 EP_Dir;//0 OUT, 1 IN
        u8 EP_Type;//bulk/in/iso
        u32 irq_enb;

};


struct w55fa93_request {
        struct list_head		queue;		/* ep's requests */
        struct usb_request		req;
        u32				dma_mapped;
};

enum ep0_state {
        EP0_IDLE,
        EP0_IN_DATA_PHASE,
        EP0_OUT_DATA_PHASE,
        EP0_END_XFER,
        EP0_STALL,
};


struct w55fa93_udc {
        spinlock_t			lock;

        struct w55fa93_ep		ep[W55FA93_ENDPOINTS];
        struct usb_gadget		gadget;
        struct usb_gadget_driver	*driver;
        struct platform_device		*pdev;

        struct clk                      *clk;
        struct resource                 *res;
        void __iomem                    *reg;
        int                             irq;

        enum ep0_state 	    	    	ep0state;

        u8				usb_devstate;
        u8				usb_address;


        u8				usb_dma_dir;

        u8				usb_dma_trigger;//bool. dma triggered
        u8				usb_dma_trigger_next;//need trigger again
        u8				usb_less_mps;
        u32				usb_dma_cnt;//one dma transfer count
        u32				usb_dma_loop;//for short packet only;dma loop, each loop 32byte;
        u32 			        usb_dma_owner;

        struct usb_ctrlrequest	        crq;
        s32				setup_ret;

        u32                             irq_enbl;
};


#endif


