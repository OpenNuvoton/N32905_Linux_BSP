/* linux/arch/arm/mach-nuc900/include/mach/regs_usbd.h
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
 *   2006/08/26     vincen.zswan add this file for nuvoton nuc900 evb.
 */

 
#ifndef __W55FA93_USBD_H
#define __W55FA93_USBD_H


#define USBD_DMA_LEN		0x10000
#define USBD_ISO_DMA_LEN	0xF0000
#define USB_HIGHSPEED	2
#define USB_FULLSPEED	1
#define EPSTADDR        0x400
#define CBW_SIZE	64

#define DMA_READ	1
#define DMA_WRITE	2

/*
 * Standard requests
 */
#define USBR_GET_STATUS			0x00
#define USBR_CLEAR_FEATURE		0x01
#define USBR_SET_FEATURE		0x03
#define USBR_SET_ADDRESS		0x05
#define USBR_GET_DESCRIPTOR		0x06
#define USBR_SET_DESCRIPTOR		0x07
#define USBR_GET_CONFIGURATION          0x08
#define USBR_SET_CONFIGURATION	        0x09
#define USBR_GET_INTERFACE		0x0A
#define USBR_SET_INTERFACE		0x0B
#define USBR_SYNCH_FRAME		0x0C


//Bit Definitions of IRQ_ENB/STAT register
#define	IRQ_USB_STAT		        0x01
#define IRQ_CEP				0x02
#define IRQ_NCEP			0xfc   

//Definition of Bits in USB_IRQ_STS register
#define USB_SOF			0x01	
#define USB_RST_STS		0x02
#define	USB_RESUME		0x04
#define	USB_SUS_REQ		0x08
#define	USB_HS_SETTLE	        0x10
#define	USB_DMA_REQ		0x20
#define USABLE_CLK		0x40
#define USB_VBUS_STS		0x100


//Definition of Bits in USB_OPER register
#define USB_GEN_RES             0x1
#define USB_HS		        0x2
#define USB_CUR_SPD_HS          0x4

//Definition of Bits in CEP_IRQ_STS register
#define CEP_SUPTOK	 	0x0001
#define CEP_SUPPKT		0x0002
#define CEP_OUT_TOK		0x0004
#define CEP_IN_TOK		0x0008
#define CEP_PING_TOK	        0x0010
#define CEP_DATA_TXD	        0x0020
#define CEP_DATA_RXD	        0x0040
#define CEP_NAK_SENT	        0x0080
#define CEP_STALL_SENT	        0x0100
#define CEP_USB_ERR		0x0200
#define CEP_STS_END		0x0400
#define CEP_BUFF_FULL	        0x0800
#define CEP_BUFF_EMPTY	        0x1000

//Definition of Bits in CEP_CTRL_STS register
#define CEP_NAK_CLEAR		0x00  //writing zero clears the nak bit
#define CEP_SEND_STALL		0x02
#define CEP_ZEROLEN		0x04
#define CEP_FLUSH		0x08

//Definition of Bits in EP_IRQ_STS register
#define EP_BUFF_FULL	        0x001
#define EP_BUFF_EMPTY	        0x002
#define EP_SHORT_PKT	        0x004
#define EP_DATA_TXD		0x008
#define EP_DATA_RXD		0x010
#define EP_OUT_TOK		0x020
#define EP_IN_TOK		0x040
#define EP_PING_TOK		0x080
#define EP_NAK_SENT		0x100
#define EP_STALL_SENT	        0x200
#define EP_USB_ERR		0x800
#define EP_BO_SHORT_PKT		0x1000

//Bit Definitons of EP_RSP_SC Register
#define EP_BUFF_FLUSH           0x01
#define EP_MODE                 0x06
#define EP_MODE_AUTO	        0x00
#define EP_MODE_MAN 	        0x02
#define EP_MODE_FLY		0x04
#define EP_TOGGLE		0x8
#define EP_HALT			0x10
#define EP_ZERO_IN              0x20
#define EP_PKT_END              0x40

//Bit Definitons of EP_CFG Register
#define EP_VALID		0x01
#define EP_TYPE			0x06 //2-bit size	
#define EP_TYPE_BLK		0x01
#define EP_TYPE_INT		0x02
#define EP_TYPE_ISO		0x03
#define EP_DIR			0x08
#define EP_NO			0xf0 //4-bit size

/* Define Endpoint feature */
#define Ep_In                   0x01
#define Ep_Out                  0x00


#endif /* __W55FA93_USBD_H */
