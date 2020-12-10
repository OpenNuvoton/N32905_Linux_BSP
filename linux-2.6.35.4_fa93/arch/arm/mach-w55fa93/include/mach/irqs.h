/*
 * arch/arm/mach-w55fa93/include/mach/irqs.h
 *
 * Copyright (c) 2008 Nuvoton technology corporation.
 *
 * Based on arch/arm/mach-s3c2410/include/mach/irqs.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

/*
 * we keep the first set of CPU IRQs out of the range of
 * the ISA space, so that the PC104 has them to itself
 * and we don't end up having to do horrible things to the
 * standard ISA drivers....
 *
 */

#define W55FA93_IRQ(x) (x)

/* Main cpu interrupts */

#define IRQ_WDT        W55FA93_IRQ(1)
#define IRQ_GPIO0      W55FA93_IRQ(2)
#define IRQ_GPIO1      W55FA93_IRQ(3)
#define IRQ_GPIO2      W55FA93_IRQ(4)
#define IRQ_GPIO3      W55FA93_IRQ(5)
#define IRQ_SPU        W55FA93_IRQ(6)
#define IRQ_I2S        W55FA93_IRQ(7)
#define IRQ_VPOST      W55FA93_IRQ(8)
#define IRQ_CAP        W55FA93_IRQ(9)
#define IRQ_BLT        W55FA93_IRQ(11)
#define IRQ_FSC        W55FA93_IRQ(12)
#define IRQ_HUART      W55FA93_IRQ(13)
#define IRQ_TIMER0     W55FA93_IRQ(14)
#define IRQ_TIMER1     W55FA93_IRQ(15)
#define IRQ_USBD       W55FA93_IRQ(16)
#define IRQ_SIC        W55FA93_IRQ(17)
#define IRQ_USBH       W55FA93_IRQ(18)
#define IRQ_EDMA       W55FA93_IRQ(19)
#define IRQ_SPI0       W55FA93_IRQ(20)
#define IRQ_SPI1       W55FA93_IRQ(21)
#define IRQ_ADC        W55FA93_IRQ(22)
#define IRQ_RTC        W55FA93_IRQ(23)
#define IRQ_UART       W55FA93_IRQ(24)
#define IRQ_PWM        W55FA93_IRQ(25)
#define IRQ_JPEG       W55FA93_IRQ(26)
#define IRQ_KPI        W55FA93_IRQ(28)
#define IRQ_I2C        W55FA93_IRQ(30)
#define IRQ_PWR        W55FA93_IRQ(31)

#define IRQ_GPIO_START 	W55FA93_IRQ(W55FA93_IRQ(0x100))
#define IRQ_GPIO_END 	W55FA93_IRQ(W55FA93_IRQ(0x100+0xA0))
#ifndef CONFIG_GPIO_W55FA93
#define NR_IRQS        32
#else
#define NR_IRQS        (IRQ_GPIO_END + 1)
#endif

#endif /* __ASM_ARCH_IRQ_H */
