/* linux/include/asm-arm/arch-w55fa93/w55fa93_rtc.h
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
 *   2006/08/26     vincen.zswan add this file for nuvoton w55fa93 evb.
 */


#ifndef _W55FA93_RTC_H
#define _W55FA93_RTC_H



#define SIGRTC 26
#define ALARM_IRQ 1
#define TICK_IRQ 2
#define RTC_IO_EXTENT	0x34

#define RTC_TICK_ON     _IO('p', 0x03)  /* Update int. enable on        */
#define RTC_TICK_OFF    _IO('p', 0x04)  /* ... off                      */
#define RTC_TICK_SET    _IO('p', 0x05)  /* Periodic int. enable on      */
#define RTC_TICK_READ   _IO('p', 0x06)  /* ... off                      */

#if 0
#define RTC_AIE_ON      _IO('p', 0x01)  /* Alarm int. enable on         */
#define RTC_AIE_OFF     _IO('p', 0x02)  /* ... off                      */
#define RTC_TICK_ON     _IO('p', 0x03)  /* Update int. enable on        */
#define RTC_TICK_OFF    _IO('p', 0x04)  /* ... off                      */
#define RTC_TICK_SET    _IO('p', 0x05)  /* Periodic int. enable on      */
#define RTC_TICK_READ   _IO('p', 0x06)  /* ... off                      */
#define RTC_ALM_SET     _IOW('p', 0x07, struct rtc_time) /* Set alarm time  */
#define RTC_ALM_READ    _IOR('p', 0x08, struct rtc_time) /* Read alarm time */
#define RTC_RD_TIME     _IOR('p', 0x09, struct rtc_time) /* Read RTC time   */
#define RTC_SET_TIME    _IOW('p', 0x0a, struct rtc_time) /* Set RTC time    */
#define RTC_CALLBACK    _IOR('p', 0x11, unsigned long)   /* rtc callback   */

#define RTC_TIME_SCALE	0x10

#endif

#define INIRRESET        0xa5eb1357 /* init value */

#define AERPOWERON	0xA965 /* RTC access enable */
#define AERPOWEROFF	0x0000 /* RTC access disable */

/* Mask Value */
#define RTCSET		0x01
#define AERRWENB	0x10000 /* check if rtc r/w register enable */

#define HR24			0x0001 /* 24 hr */
#define HR12			0x0000 /* 12 hr */
#define LEAPYEAR	0x0001
#define TICKENB		0x80
#define TICKINTENB	0x0002
#define ALARMINTENB	0x0001
#define PSWINTENB	0x0004

#define _INIR			(readl(REG_RTC_INIR)) /*	RTC Initiation Register */
#define _AER			(readl(REG_RTC_AER)) /*	RTC Access Enable Register */
#define _FCR			(readl(REG_RTC_FCR)) /*	RTC Frequency Compensation Register */
#define _TLR			(readl(REG_RTC_TLR)) /*	Time Loading Register */
#define _CLR			(readl(REG_RTC_CLR)) /*	Calendar Loading Register */
#define _TSSR		(readl(REG_RTC_TSSR)) /*	Time Scale Selection Register */
#define _DWR			(readl(REG_RTC_DWR)) /*	Day of the Week Register */
#define _TAR			(readl(REG_RTC_TAR)) /*	Time Alarm Register */
#define _CAR			(readl(REG_RTC_CAR)) /*	Calendar Alarm Register */
#define _LIR			(readl(REG_RTC_LIR)) /*	Leap year Indicator Register */
#define _RIER			(readl(REG_RTC_RIER)) /*	RTC Interrupt Enable Register */
#define _RIIR			(readl(REG_RTC_RIIR)) /*	RTC Interrupt Indicator Register */
#define _TTR			(readl(REG_RTC_TTR)) /*	RTC Time Tick Register */

#define RTC_SECONDS			(((_TLR>>4)&0x07)*10+(_TLR&0x0F))
#define RTC_MINUTES			(((_TLR>>12)&0x07)*10+((_TLR>>8)&0x0F))
#define RTC_HOURS			(((_TLR>>20)&0x07)*10+((_TLR>>16)&0x0F))
#define RTC_DAY_OF_MONTH	(((_CLR>>4)&0x03)*10+(_CLR&0x0F))
#define RTC_MONTH			(((_CLR>>12)&0x01)*10+((_CLR>>8)&0x0F))
#define RTC_YEAR			(((_CLR>>20)&0x0F)*10+((_CLR>>16)&0x0F))

/* Alarm time */
#define RTC_SECONDS_ALARM	(((_TAR>>4)&0x07)*10+(_TAR&0x0F))
#define RTC_MINUTES_ALARM	(((_TAR>>12)&0x07)*10+((_TAR>>8)&0x0F))
#define RTC_HOURS_ALARM		(((_TAR>>20)&0x07)*10+((_TAR>>16)&0x0F))
#define RTC_DAY_OF_MONTH_ALARM	(((_CAR>>4)&0x03)*10+(_CAR&0x0F))
#define RTC_MONTH_ALARM			(((_CAR>>12)&0x01)*10+((_CAR>>8)&0x0F))
#define RTC_YEAR_ALARM			(((_CAR>>20)&0x0F)*10+((_CAR>>16)&0x0F))
#define RTC_DAYOFWEEK			(_DWR&0x07)

#define RTC_TIME_SCALE	0x10  /* Time scale selection	*/
#define TIME24			1
#define TIME12			0

#define CMOS_READ(reg)		readl(reg)
#define CMOS_WRITE(val,reg)	writel(val, reg)

#ifndef BCD_TO_BIN
#define BCD_TO_BIN(val) ((val)=((val)&15) + ((val)>>4)*10)
#endif

#ifndef BIN_TO_BCD
#define BIN_TO_BCD(val) ((val)=(((val)/10)<<4) + (val)%10)
#endif

#endif /*_W55FA93_RTC_H*/
