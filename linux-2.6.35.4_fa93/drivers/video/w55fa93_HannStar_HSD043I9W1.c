
struct w55fa93fb_mach_info w55fa93_lcd_platdata = {
			.width	= LCDWIDTH,
			.height	= LCDWIDTH,

.xres = {
			.defval	= LCDWIDTH,
			.min		= LCDWIDTH,
			.max		= LCDWIDTH,
		},
		
.yres = {
			.defval	= LCDHEIGHT,
			.min		= LCDHEIGHT,
			.max		= LCDHEIGHT,
},

.bpp = {
			.defval	= LCDBPP,
			.min		= LCDBPP,
			.max		= LCDBPP,
},


			hsync_len   :  64, 
			vsync_len    :  6,
			left_margin :  125, 
			upper_margin :  70,
			right_margin:  115,  
			lower_margin :  36,
			sync:		0,		
			cmap_static:	0,

};

extern unsigned int w55fa93_upll_clock;

#ifdef CONFIG_W55FA93_FB_INIT 
	static int w55fa93fb_init_device(struct w55fa93fb_info *fbi)
	{
		unsigned int u32PllFreq, u32ClockDivider;
	
	  	// Reset IP
		outl(inl(REG_AHBIPRST) | VPOSTRST, REG_AHBIPRST);
		outl(inl(REG_AHBIPRST) & ~VPOSTRST, REG_AHBIPRST);	
	
		// given clock divider for VPOST 
		u32PllFreq = w55fa93_upll_clock;
		u32ClockDivider = u32PllFreq / 9000;
		
		u32ClockDivider /= 2;
		outl((inl(REG_CLKDIV1) & ~VPOST_S) | (0x03 << 3), REG_CLKDIV1);	// 00:clock from CLK_in, 01:32K, 10:APLL, 11:UPLL	
		outl((inl(REG_CLKDIV1) & ~VPOST_N0) | 1, REG_CLKDIV1);		
		outl((inl(REG_CLKDIV1) & ~VPOST_N1) | (u32ClockDivider << 8), REG_CLKDIV1);	// divider value
	
		// enable VPOST function pins
	   	outl(inl(REG_GPBFUN) | MF_GPB15, REG_GPBFUN);						// enable LPCLK pin
	   	outl(inl(REG_GPCFUN) | 0xFFFFFFFF, REG_GPCFUN);						// enable LVDATA[15:0] pins
	   	outl(inl(REG_GPDFUN) | (MF_GPD11+MF_GPD10+MF_GPD9), REG_GPDFUN);	// enable HSYNC/VSYNC/VDEN pins	
	   	outl(inl(REG_GPEFUN) | MF_GPE0 | MF_GPE1, REG_GPEFUN);				// enable LVDATA[17:16] pins		
	   	
		// configure LCD interface  // enable sync with TV, LCD type select 
	   	outl(inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDSynTv, REG_LCM_LCDCPrm);	// async with TV
	   	outl(inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDTYPE, REG_LCM_LCDCPrm);	// High Resolution mode
		/*
		0x0  // High Resolution mode
		0x1  // Sync-type TFT LCD
		0x2  // Sync-type Color STN LCD
		0x3  // MPU-type LCD
		*/
	
		// configure LCD parallel data bus 
	//   	outl( (inl(REG_LCM_LCDCCtl)& ~LCDCCtl_PRDB_SEL) | (0x00 << 20), REG_LCM_LCDCCtl);	// 16-bit mode 
	   	outl( (inl(REG_LCM_LCDCCtl)& ~LCDCCtl_PRDB_SEL) | (0x01 << 20), REG_LCM_LCDCCtl);	// 18-bit mode 
		/*
		0x0  // 16-bit mode (RGB565 output)
		0x1  // 18-bit mode (RGB666 output)
		0x2  // 24-bit mode (RGB888 output)		
		*/
	
		// LCDSrc, TVSrc: Frame buffer; NotchE; enable TV encoder; NTSC; Frame Buffer Size:640x480; Interlace
	  	outl((inl(REG_LCM_TVCtl)& 0xFFFF30DA)|(0x00008529), REG_LCM_TVCtl);  
	
			  
		outl((inl(REG_LCM_TVDisCtl)& 0xFE000000)|(0x00F016A0), REG_LCM_TVDisCtl);
	
	   	// set Horizontal scanning line timing for Syn type LCD 
		outl((inl(REG_LCM_TCON1)& 0xFF000000)|(0x00012732), REG_LCM_TCON1);
		
		// set Vertical scanning line timing for Syn type LCD   
		outl((inl(REG_LCM_TCON2)& 0xFF000000)|(0x00020727), REG_LCM_TCON2);
	
		// set both "active pixel per line" and "active lines per screen" for Syn type LCD   
	   	outl(0x01DF0110, REG_LCM_TCON3);   // 480x272
	//   	outl((inl(REG_LCM_TCON4)& 0xFC00FFF0)|(0x01E000103), REG_LCM_TCON4);	
	   	outl(0x01E00103, REG_LCM_TCON4);	
	
		// set TV control register and LCD from frame buffer source
	   	outl((inl(REG_LCM_TVCtl)& 0xFFFFF0DA)|(0x00000410), REG_LCM_TVCtl);   // DAC disable	
	   	
		// enable LCD controller
		outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10003), REG_LCM_LCDCCtl);
		return 0;
	}
#endif	
	
static int w55fa93fb_ioctl_device(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	unsigned int buffer[5];
	
	memset(buffer,0,5);
	
	switch(cmd)
	{
		case IOCTLCLEARSCREEN:	
			memset((void *)video_cpu_mmap, 0xff, video_alloc_len); 		
			break;
		#if 0		
		case VIDEO_ACTIVE_WINDOW_COORDINATES:
			/* Get the start line of video window */
			copy_from_user((unsigned int *)buffer, (void *)arg, 1*sizeof(unsigned int));			
			/* Reset the original start line of video window */
			outl(0x000107FF, REG_LCM_VA_WIN);
			outl( ((buffer[0]&0x7FF) << 16) | inl(REG_LCM_VA_WIN), REG_LCM_VA_WIN);
			break;
		#endif
		case IOCTL_LCD_BRIGHTNESS:
			// Need to implement PWM backlight control here !!!
			// Get the duty value
			copy_from_user((unsigned int *)buffer, (void *)arg, 1*sizeof(unsigned int));
			w55fa93fb_set_CMR(buffer[0]);
			break;

		case VIDEO_DISPLAY_ON:
			outl(inl(REG_GPDFUN) & ~MF_GPD1, REG_GPDFUN);	
			outl(inl(REG_GPIOD_OMD) | 0x02, REG_GPIOD_OMD);				
			outl(inl(REG_GPIOD_DOUT) | 0x02, REG_GPIOD_DOUT);		// backlight ON (for Nuvoton FA93 demo board only)

			break;
			
		case VIDEO_DISPLAY_OFF:
			outl(inl(REG_GPDFUN) & ~MF_GPD1, REG_GPDFUN);	
			outl(inl(REG_GPIOD_OMD) | 0x02, REG_GPIOD_OMD);				
			outl(inl(REG_GPIOD_DOUT) & ~0x02, REG_GPIOD_DOUT);		// backlight OFF (for Nuvoton FA93 demo board only)
			break;
			
#if !defined(CONFIG_W55FA93_TV_LCD_SWITCH)

		 case IOCTL_LCD_ENABLE_INT:
             outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x20000, REG_LCM_LCDCInt); // enable VIN
	         //_auto_disable = 0;
                break;
		 case IOCTL_LCD_DISABLE_INT:
	         outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) & ~0x20000, REG_LCM_LCDCInt); // disable VIN
#ifdef CONFIG_TWO_FB_BUF
             while (w55fa93_edma_isbusy(VDMA_CH));
#endif            
             _auto_disable = 1;
             break;
#endif	                

#ifdef CONFIG_TWO_FB_BUF
		case IOCTL_LCD_GET_DMA_BASE:
		  if(copy_to_user((unsigned int *)arg, (unsigned int *)&_bg_mem_p, sizeof(unsigned int)) < 0) {
		    printk("failed..\n");
		    return(-EFAULT);
		  }
			printk("!!!%x\n", *(unsigned int *)&_bg_mem_p);
			break;
#endif            
		case DUMP_LCD_REG:		//ken add
		{
			unsigned int reg_array[5];
			reg_array[0] = inl(REG_LCM_TCON1);
			reg_array[1] = inl(REG_LCM_TCON2);
			reg_array[2] = inl(REG_LCM_TCON3);
			reg_array[3] = inl(REG_LCM_TCON4);
			reg_array[4] = inl(REG_LCM_LCDCCtl);
			if(copy_to_user((unsigned int *)arg, (unsigned int *)&reg_array, 5*sizeof(unsigned int)) < 0) {
		    		printk("failed..\n");
		    		return(-EFAULT);
		  	}	
		}		
			break;
			
		default:
			break;	
	}
		
	return 0;    
}
	

static int w55fa93fb_blank_device(int blank_mode, struct fb_info *info)
{
    return 0;
}


void w55fa93fb_init_pwm_device(void)
{
	w55fa93fb_set_pwm_channel(PWM0);
}

static int w55fa93fb_probe_device(void)
{
#ifndef CONFIG_W55FA93_TV_LCD_SWITCH	
	outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x20000, REG_LCM_LCDCInt); // enable VINTEN
#endif	
	
	/* enable lcd*/
	outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10003), REG_LCM_LCDCCtl);
	
	return 0;
}

static void w55fa93fb_stop_device(void)
{
	outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFFE), REG_LCM_LCDCCtl);
}

#ifdef CONFIG_W55FA93_TV_LCD_SWITCH
	static void switch_vpost_clcok_for_lcm(void)
	{
		unsigned int clock_div;				
		
	#if !defined(CONFIG_W55FA93_TV_FROM_APLL)			
			clock_div = w55fa93_upll_clock / 9000;
			clock_div /= 2;
			clock_div &= 0xFF;
			clock_div --;
			
			outl((inl(REG_CLKDIV1) & ~VPOST_S) | (3<<3), REG_CLKDIV1);		// VPOST clock from UPLL			
			outl((inl(REG_CLKDIV1) & ~VPOST_N0) | 1, REG_CLKDIV1);					// divider 2 in VPOST_N0
			outl((inl(REG_CLKDIV1) & ~VPOST_N1) | (clock_div << 8), REG_CLKDIV1);	
	#else
			w55fa93_set_apll_clock(135000);		
			clock_div = w55fa93_apll_clock / 9000;
			clock_div &= 0xFF;
			clock_div --;
			
			outl((inl(REG_CLKDIV1) & ~VPOST_S) | (2<<3), REG_CLKDIV1);		// VPOST clock from APLL			
			outl((inl(REG_CLKDIV1) & ~VPOST_N0), REG_CLKDIV1);				
			outl((inl(REG_CLKDIV1) & ~VPOST_N1) | (clock_div << 8), REG_CLKDIV1);	
	#endif			
	
	}
#endif
