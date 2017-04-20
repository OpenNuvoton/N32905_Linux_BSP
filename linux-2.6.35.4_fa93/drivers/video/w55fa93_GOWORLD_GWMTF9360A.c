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

extern int w55fa93_set_apll_clock(unsigned int clock);

static void delay_loop(unsigned char u8delay)
{
	volatile unsigned char ii, jj;
	for (jj=0; jj<u8delay; jj++)
		for (ii=0; ii<200; ii++);
}

#ifdef OPT_SPI_CS_BY_GPD2	
	#define SpiEnable()		outl(inl(REG_GPIOD_DOUT)& ~0x0004, REG_GPIOD_DOUT)
	#define SpiDisable()	outl(inl(REG_GPIOD_DOUT) | 0x0004, REG_GPIOD_DOUT)
#else	
	#define SpiEnable()		outl(inl(REG_GPIOD_DOUT)& ~0x0800, REG_GPIOD_DOUT)
	#define SpiDisable()	outl(inl(REG_GPIOD_DOUT) | 0x0800, REG_GPIOD_DOUT)
#endif
	
#define SpiHighSCK()	outl(inl(REG_GPIOB_DOUT) | 0x2000, REG_GPIOB_DOUT)
#define SpiLowSCK()		outl(inl(REG_GPIOB_DOUT)& ~0x2000, REG_GPIOB_DOUT)
#define SpiHighSDA()	outl(inl(REG_GPIOB_DOUT) | 0x4000, REG_GPIOB_DOUT)
#define SpiLowSDA()		outl(inl(REG_GPIOB_DOUT)& ~0x4000, REG_GPIOB_DOUT)


void GWMTF9360A_WriteReg(unsigned char u8Reg, unsigned short u16Data)
{
	unsigned short ii;
	unsigned int u32SentData;
	
	// write register index
	u32SentData = 0x70;
	u32SentData <<= 16;
	u32SentData |= u8Reg;
	u32SentData <<= 8;

	SpiDisable();
	SpiHighSCK();
	SpiHighSDA();
	delay_loop(2);
	SpiEnable();
	delay_loop(1);	

	for (ii=0; ii<16; ii++)
	{
		SpiLowSCK();
		delay_loop(1);			
		if (u32SentData & BIT31)
			SpiHighSDA();
		else
			SpiLowSDA();
		delay_loop(1);						
		SpiHighSCK();			
		delay_loop(1);		
		u32SentData <<= 1;
	}
	SpiHighSCK();				
	SpiHighSDA();	
	delay_loop(1);									
	SpiDisable();	
	delay_loop(5);						
	
	// write register contents
	u32SentData = 0x72;
	u32SentData <<= 16;
	u32SentData |= u16Data;
	u32SentData <<= 8;

	SpiHighSCK();
	SpiHighSDA();
	SpiEnable();
	delay_loop(1);	

	for (ii=0; ii<16; ii++)
	{
		SpiLowSCK();
		delay_loop(1);			
		if (u32SentData & BIT31)
			SpiHighSDA();
		else
			SpiLowSDA();
		delay_loop(1);						
		SpiHighSCK();			
		delay_loop(1);		
		u32SentData <<= 1;								
	}
	SpiHighSCK();				
	SpiHighSDA();	
	delay_loop(1);									
	SpiDisable();	
	delay_loop(5);						
}

void GOWORLD_GWMTF9360A_Init(void)
{
#ifdef OPT_SPI_CS_BY_GPD2	
	outl(inl(REG_GPDFUN)& ~0x00000030, REG_GPDFUN);				// set GPD_2 to be GPIO pin
#else	
	outl(inl(REG_GPDFUN)& ~0x00C00000, REG_GPDFUN);				// set GPD_11 to be GPIO pin
#endif	
	
	outl(inl(REG_GPBFUN)& ~0x3C000000, REG_GPBFUN);				// set GPB_14/GPB_13 to be GPIO pins
	
	// set output mode	
	outl(inl(REG_GPIOB_OMD) | (BIT14+BIT13), REG_GPIOB_OMD);				// set GPB_14/GPB_13 to be output mode

#ifdef OPT_SPI_CS_BY_GPD2		
	outl(inl(REG_GPIOD_OMD) | (BIT2), REG_GPIOD_OMD);				// set GPD_2 to be output mode
#else	
	outl(inl(REG_GPIOD_OMD) | (BIT11), REG_GPIOD_OMD);				// set GPD_11 to be output mode
#endif	

	// set internal pull-up resistor	
	outl(inl(REG_GPIOB_PUEN) | (BIT14+BIT13), REG_GPIOB_PUEN);				// set GPB_14/GPB_13 to be internal pullup
	
#ifdef OPT_SPI_CS_BY_GPD2	
	outl(inl(REG_GPIOD_PUEN) | (BIT2), REG_GPIOD_PUEN);				// set GPD_2 to be internal pullup
#else	
	outl(inl(REG_GPIOD_PUEN) | (BIT11), REG_GPIOD_PUEN);				// set GPD_11 to be internal pullup
#endif	
	
	// initial LCM register
	delay_loop(10);		
	GWMTF9360A_WriteReg(0x01, 0x0000);
	GWMTF9360A_WriteReg(0x02, 0x0200);
	GWMTF9360A_WriteReg(0x03, 0x6364);	
	GWMTF9360A_WriteReg(0x04, 0x0400);		
//	GWMTF9360A_WriteReg(0x05, 0x0000);
	GWMTF9360A_WriteReg(0x0A, 0x4008);
	GWMTF9360A_WriteReg(0x0B, 0xD400);	
	GWMTF9360A_WriteReg(0x0D, 0x3229);		
	GWMTF9360A_WriteReg(0x0E, 0x3200);
	GWMTF9360A_WriteReg(0x0F, 0x0000);
	GWMTF9360A_WriteReg(0x16, 0x9F80);	
//	GWMTF9360A_WriteReg(0x17, 0x0000);		
	GWMTF9360A_WriteReg(0x30, 0x0000);
	GWMTF9360A_WriteReg(0x31, 0x0407);
	GWMTF9360A_WriteReg(0x32, 0x0202);	
	GWMTF9360A_WriteReg(0x33, 0x0000);		
	GWMTF9360A_WriteReg(0x34, 0x0505);
	GWMTF9360A_WriteReg(0x35, 0x0003);
	GWMTF9360A_WriteReg(0x36, 0x0707);	
	GWMTF9360A_WriteReg(0x37, 0x0000);		
	GWMTF9360A_WriteReg(0x3A, 0x0904);
	GWMTF9360A_WriteReg(0x3B, 0x0904);
	GWMTF9360A_WriteReg(0x01, 0x0000);	
	GWMTF9360A_WriteReg(0x01, 0x0000);		
	GWMTF9360A_WriteReg(0x01, 0x0000);
	GWMTF9360A_WriteReg(0x01, 0x0000);
	GWMTF9360A_WriteReg(0x01, 0x0000);	
	GWMTF9360A_WriteReg(0x01, 0x0000);		
}

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
		clock_div --;
		
		// given clock divider for VPOST 
		outl((inl(REG_CLKDIV1) & ~VPOST_N0) | 1, REG_CLKDIV1);					// divider 2 in VPOST_N0
		outl((inl(REG_CLKDIV1) & ~VPOST_N1) | (clock_div << 8), REG_CLKDIV1);	
		
	//	outl((inl(REG_CLKDIV1) & ~VPOST_S), REG_CLKDIV1);				// VPOST clock from UPLL
		outl((inl(REG_CLKDIV1) & ~VPOST_S) | (3<<3), REG_CLKDIV1);		// VPOST clock from UPLL
		
		// enable VPOST function pins
	   	outl(inl(REG_GPBFUN) | MF_GPB15, REG_GPBFUN);						// enable LPCLK pin
	   	outl(inl(REG_GPCFUN) | 0x0000FFFF, REG_GPCFUN);						// enable LVDATA[7:0] pins
	   	outl(inl(REG_GPDFUN) | (MF_GPD10+MF_GPD9), REG_GPDFUN);	// enable HSYNC/VSYNC pins	   			
	   	
		// configure LCD interface  // enable sync with TV, LCD type select 
	   	outl(inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDSynTv, REG_LCM_LCDCPrm);	// async with TV
	   	outl((inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDTYPE) | 0x01, REG_LCM_LCDCPrm);	// Sync-type TFT LCD
		/*
		0x0  // High Resolution mode
		0x1  // Sync-type TFT LCD
		0x2  // Sync-type Color STN LCD
		0x3  // MPU-type LCD
		*/
		outl((inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDDataSel) | 0x0C, REG_LCM_LCDCPrm);	// RGB ThroughMode	
	
		GOWORLD_GWMTF9360A_Init();
	
		// configure LCD parallel data bus 
	   	outl( (inl(REG_LCM_LCDCCtl)& ~LCDCCtl_PRDB_SEL) | (0x00 << 20), REG_LCM_LCDCCtl);	// 16-bit mode 
	   	//outl( (inl(REG_LCM_LCDCCtl)& ~LCDCCtl_PRDB_SEL) | (0x01 << 20), REG_LCM_LCDCCtl);	// 18-bit mode 
		/*
		0x0  // 16-bit mode (RGB565 output)
		0x1  // 18-bit mode (RGB666 output)
		0x2  // 24-bit mode (RGB888 output)		
		*/
		
		// LCDSrc, TVSrc: Frame buffer; NotchE; enable TV encoder; NTSC; Frame Buffer Size:640x480; 
	  	outl((inl(REG_LCM_TVCtl)& ~(TVCtl_LCDSrc+TVCtl_LCDSrc))|(0x00008500), REG_LCM_TVCtl);  
	
			  
		///outl((inl(REG_LCM_TVDisCtl)& 0xFE000000)|(0x00F016A0), REG_LCM_TVDisCtl);
	
	   	// set Horizontal scanning line timing for Syn type LCD 
	//	outl((inl(REG_LCM_TCON1)& 0xFF000000)|(0x00035555), REG_LCM_TCON1);
	//	outl((inl(REG_LCM_TCON1)& 0xFF000000)|(0x000941D1), REG_LCM_TCON1);
		outl((inl(REG_LCM_TCON1)& 0xFF000000)|(0x0001C93B), REG_LCM_TCON1);	
		
		// set Vertical scanning line timing for Syn type LCD   
		outl((inl(REG_LCM_TCON2)& 0xFF000000)|(0x00001203), REG_LCM_TCON2);
	
		// set both "active pixel per line" and "active lines per screen" for Syn type LCD   
	   	outl(0x03BF00F0, REG_LCM_TCON3);	// 320x240
	   	
	   	outl(0x01400102, REG_LCM_TCON4);	// signal polarity
	
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
			clock_div = w55fa93_upll_clock / 27000;
			clock_div /= 2;
			clock_div &= 0xFF;
			clock_div --;
			
			outl((inl(REG_CLKDIV1) & ~VPOST_S) | (3<<3), REG_CLKDIV1);		// VPOST clock from UPLL			
			outl((inl(REG_CLKDIV1) & ~VPOST_N0) | 1, REG_CLKDIV1);					// divider 2 in VPOST_N0
			outl((inl(REG_CLKDIV1) & ~VPOST_N1) | (clock_div << 8), REG_CLKDIV1);	
	#else
			w55fa93_set_apll_clock(135000);		
			clock_div = w55fa93_apll_clock / 27000;
			clock_div &= 0xFF;
			clock_div --;
			
			outl((inl(REG_CLKDIV1) & ~VPOST_S) | (2<<3), REG_CLKDIV1);		// VPOST clock from APLL			
			outl((inl(REG_CLKDIV1) & ~VPOST_N0), REG_CLKDIV1);				
			outl((inl(REG_CLKDIV1) & ~VPOST_N1) | (clock_div << 8), REG_CLKDIV1);	
	#endif			
	
	}
#endif
