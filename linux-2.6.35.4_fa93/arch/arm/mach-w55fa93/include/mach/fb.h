/* arch/arm/mach-w55fa93/include/mach/fb.h
 *
 * Copyright (c) 2008 Jhe <Jhe@nuvoton.com>
 *
 * Inspired by pxafb.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARM_FB_H
#define __ASM_ARM_FB_H


#include <mach/w55fa93_reg.h>

struct w55fa93fb_val {
	unsigned int	defval;
	unsigned int	min;
	unsigned int	max;
};

struct w55fa93fb_hw {
  unsigned long	lcdcon1;
  unsigned long	lcdcon2;
  unsigned long	lcdcon3;
  unsigned long	lcdcon4;
  unsigned long	lcdcon5;
  unsigned long lcd_dccs;
  unsigned long lcd_device_ctrl;
  unsigned long lcd_mpulcd_cmd;
  unsigned long lcd_int_cs;
  unsigned long lcd_crtc_size;
  unsigned long lcd_crtc_dend;
  unsigned long lcd_crtc_hr;
  unsigned long lcd_crtc_hsync;
  unsigned long lcd_crtc_vr;
  unsigned long lcd_va_baddr0;
  unsigned long lcd_va_baddr1;
  unsigned long lcd_va_fbctrl;
  unsigned long lcd_va_scale;
  unsigned long lcd_va_test;
  unsigned long lcd_va_win;
  unsigned long lcd_va_stuff;
};

struct w55fa93fb_mach_info {
  unsigned char	fixed_syncs;	/* do not update sync/border */
  
  /* Screen size */
  int		width;
  int		height;
  
  /* Screen info */
  struct w55fa93fb_val xres;
  struct w55fa93fb_val yres;
  struct w55fa93fb_val bpp;
  
  /* lcd configuration registers */
  struct w55fa93fb_hw  regs;
  
  /* lcd other configuration  */
  u_long		pixclock;
  u_char		hsync_len;
  u_char		left_margin;
  u_char		right_margin;
  
  u_char		vsync_len;
  u_char		upper_margin;
  u_char		lower_margin;
  u_char		sync;
  
  u_int		cmap_grayscale:1,
					cmap_inverse:1,
					cmap_static:1,
					unused:29;
  u_int		state;
    
};

// bycc, to fix system crash issue when FB driver is built as module
//void __init w55fa93_fb_set_platdata(struct w55fa93fb_mach_info *);
void w55fa93_fb_set_platdata(struct w55fa93fb_mach_info *);

#endif /* __ASM_ARM_FB_H */
