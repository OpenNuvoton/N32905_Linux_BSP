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
extern unsigned int w55fa93_apll_clock;

#ifdef CONFIG_W55FA93_FB_INIT 
	static int w55fa93fb_init_device(struct w55fa93fb_info *fbi)
	{
		unsigned int clock_div;
		
	  	// Reset IP
		outl(inl(REG_AHBIPRST) | VPOSTRST, REG_AHBIPRST);
		outl(inl(REG_AHBIPRST) & ~VPOSTRST, REG_AHBIPRST);	
	
	//	printk("w55fa93_upll_clock = 0x%x\n", w55fa93_upll_clock);		   	
	
	//	w55fa93_upll_clock = 192000;
		clock_div = w55fa93_upll_clock / 27000;
		clock_div /= 2;
		clock_div &= 0xFF;
		
		// given clock divider for VPOST 
		outl((inl(REG_CLKDIV1) & ~VPOST_N0) | 1, REG_CLKDIV1);					// divider 2 in VPOST_N0
		outl((inl(REG_CLKDIV1) & ~VPOST_N1) | (clock_div << 8), REG_CLKDIV1);	
		
	//	outl((inl(REG_CLKDIV1) & ~VPOST_S), REG_CLKDIV1);				// VPOST clock from UPLL
		outl((inl(REG_CLKDIV1) & ~VPOST_S) | (3<<3), REG_CLKDIV1);		// VPOST clock from UPLL
	
		// enable VPOST function pins
	   	outl(inl(REG_GPBFUN) | MF_GPB15, REG_GPBFUN);						// enable LPCLK pin
	   	outl(inl(REG_GPCFUN) | 0x0000FFFF, REG_GPCFUN);						// enable LVDATA[7:0] pins
	   	outl(inl(REG_GPDFUN) | (MF_GPD11+MF_GPD10+MF_GPD9), REG_GPDFUN);	// enable HSYNC/VSYNC/VDEN pins	   			
	   	
		// configure LCD interface  // enable sync with TV, LCD type select 
	   	outl(inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDSynTv, REG_LCM_LCDCPrm);			// async with TV
	   	
	   	outl((inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDTYPE) | 0x01, REG_LCM_LCDCPrm);	// Sync-type TFT LCD
		/*
		0x0  // High Resolution mode
		0x1  // Sync-type TFT LCD
		0x2  // Sync-type Color STN LCD
		0x3  // MPU-type LCD
		*/
	
		outl((inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDDataSel) | (0x03 << 2), REG_LCM_LCDCPrm);	// RGB ThroughMode	
		/*
		0x0  // CCIR601 
		0x1  // RGB Dummy
		0x2  // CCIR656
		0x3  // RGB Through
		*/
	
		outl( inl(REG_LCM_LCDCPrm)& ~LCDCPrm_SRGB_EL_SEL, REG_LCM_LCDCPrm);	// even line RGB sequence
		outl( inl(REG_LCM_LCDCPrm)& ~LCDCPrm_SRGB_OL_SEL, REG_LCM_LCDCPrm);	// odd line RGB sequence
	
		// LCD source from frame buufer 
	  	outl((inl(REG_LCM_TVCtl)& (~TVCtl_LCDSrc))|(0x00000400), REG_LCM_TVCtl);  
	
			  
		// set Horizontal scanning line timing for Syn type LCD 
		outl((inl(REG_LCM_TCON1)& 0xFF000000)|(0x000080A7), REG_LCM_TCON1);	
		
		// set Vertical scanning line timing for Syn type LCD   
		outl((inl(REG_LCM_TCON2)& 0xFF000000)|(0x00080102), REG_LCM_TCON2);
	
		// set both "active pixel per line" and "active lines per screen" for Syn type LCD   
	   	outl(0x05BF0110, REG_LCM_TCON3);	// 480x272
	   	
	   	outl(0x01E00102, REG_LCM_TCON4);	// signal polarity
	
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
			
		case IOCTL_LCD_BRIGHTNESS:
			// Need to implement PWM backlight control here !!!			
			printk("Set Backlight\n");
			copy_from_user((unsigned int *)buffer, (void *)arg, 1*sizeof(unsigned int));
			w55fa93fb_set_CMR(buffer[0]);
			break;

		case VIDEO_DISPLAY_LCD:

			printk("video displayed by LCD only\n");


			// given clock divider for VPOST 
//			outl((inl(REG_CLKDIV1) & ~VPOST_S) | (0x03 << 3), REG_CLKDIV1);	// 00:clock from CLK_in, 01:32K, 10:APLL, 11:UPLL
		
			// enable VPOST function pins
		   	outl(inl(REG_GPBFUN) | MF_GPB15, REG_GPBFUN);						// enable LPCLK pin
   			outl(inl(REG_GPCFUN) | 0x0000FFFF, REG_GPCFUN);						// enable LVDATA[7:0] pins
		   	outl(inl(REG_GPDFUN) | (MF_GPD11+MF_GPD10+MF_GPD9), REG_GPDFUN);	// enable HSYNC/VSYNC/VDEN pins	   			
			
			// configure LCD interface  // enable sync with TV, LCD type select 
		   	outl(inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDSynTv, REG_LCM_LCDCPrm);		// async with TV
   			outl((inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDTYPE) | 0x01, REG_LCM_LCDCPrm);	// Sync-type TFT LCD
			break;				


		case VIDEO_DISPLAY_TV:
			printk("video displayed by TV only\n");

			// enable VPOST function pins
		   	outl(inl(REG_LCM_LCDCPrm) | LCDCPrm_LCDSynTv, REG_LCM_LCDCPrm);	// sync with TV
			
			outl((inl(REG_LCM_LCDCCtl) & ~LCDCCtl_LCDRUN), REG_LCM_LCDCCtl); // LCD off
			
	    	outl((inl(REG_LCM_TVCtl)& 0xFFFFF0CA)|(0x00000501), REG_LCM_TVCtl);   // DAC enabled
	    	
			outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10002), REG_LCM_LCDCCtl); // LCD off, RGB565 format

			break;				


		 case IOCTL_LCD_ENABLE_INT:
             outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x20000, REG_LCM_LCDCInt); // enable VIN
	         //_auto_disable = 0;
                break;
	        case IOCTL_LCD_DISABLE_INT:
	          outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) & ~0x20000, REG_LCM_LCDCInt); // disable VIN
#if DUAL_BUF
            		while (w55fa93_edma_isbusy(VDMA_CH));
#endif            
	                
	                _auto_disable = 1;
	                break;

#if DUAL_BUF
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
	outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x20000, REG_LCM_LCDCInt); // enable VINTEN
	
	/* enable lcd*/
	outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10003), REG_LCM_LCDCCtl);

	return 0;   
}

static void w55fa93fb_stop_device(void)
{
	outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFFE), REG_LCM_LCDCCtl);
}


