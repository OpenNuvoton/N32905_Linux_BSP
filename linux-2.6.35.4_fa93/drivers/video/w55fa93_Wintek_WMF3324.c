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

#define CS_HIGH  do { outl( inl(REG_GPIOB_DOUT) | 1<<4, REG_GPIOB_DOUT); }while(0);
#define CS_LOW   do { outl( inl(REG_GPIOB_DOUT) & 0xFFFFFFEF, REG_GPIOB_DOUT); }while(0);
#define SCL_HIGH do { outl( inl(REG_GPIOB_DOUT) | 1<<5, REG_GPIOB_DOUT); }while(0);
#define SCL_LOW  do { outl( inl(REG_GPIOB_DOUT) & 0xFFFFFFDF, REG_GPIOB_DOUT); }while(0);
#define SDA_HIGH do { outl( inl(REG_GPIOB_DOUT) | 1<<6, REG_GPIOB_DOUT); }while(0);
#define SDA_LOW  do { outl( inl(REG_GPIOB_DOUT) & 0xFFFFFFBF, REG_GPIOB_DOUT); }while(0);


//void sharp_register_set(unsigned char usRegIndex, unsigned char usRegDataHB, unsigned char usRegDataLB)
void wintek_register_set(unsigned short usRegIndex, unsigned short usRegData)
{
	int i;
	char regcmd = 0x74;
	char datacmd = 0x76;
	
	CS_HIGH
	SCL_HIGH
	SDA_HIGH
	CS_LOW  

	/* register setting */
	for(i=7;i>=0;i--){
 		SCL_LOW
  		if((regcmd >> i ) & 0x1)
   			SDA_HIGH 
  		else
   			SDA_LOW
   
  		LCDDelay(0x50);  
  		SCL_HIGH
  		LCDDelay(0x50);
 	}
	for(i=15;i>=0;i--){
 		SCL_LOW
  		if((usRegIndex >> i ) & 0x1)
   			SDA_HIGH 
  		else
   			SDA_LOW
   
  		LCDDelay(0x50); 
  		SCL_HIGH
  		LCDDelay(0x50);
 	}

	SDA_HIGH
	LCDDelay(0x25);
	CS_HIGH
	LCDDelay(0x50);

	/* data transfor  */
	CS_LOW

	for(i=7;i>=0;i--)
 	{
 		SCL_LOW
  		if((datacmd >> i ) & 0x1)
   			SDA_HIGH 
  		else
   			SDA_LOW

  		LCDDelay(0x50); 
  		SCL_HIGH
  		LCDDelay(0x50);
 	}
	for(i=15;i>=0;i--)
 	{
 		SCL_LOW
  		if((usRegData >> i ) & 0x1)
   			SDA_HIGH 
  		else
   			SDA_LOW

  		LCDDelay(0x50); 
  		SCL_HIGH
  		LCDDelay(0x50);
 	}

	SDA_HIGH
	LCDDelay(0x25);
	CS_HIGH
	LCDDelay(0x50);
	
} 

void Wintek_LCD_INIT(void)
{
	// set GPB[6:4] to GPIO mode
	outl(inl(REG_GPBFUN) & ~MF_GPB4, REG_GPBFUN);
	outl(inl(REG_GPBFUN) & ~MF_GPB5, REG_GPBFUN);
	outl(inl(REG_GPBFUN) & ~MF_GPB6, REG_GPBFUN);		
	
	// GPB[4] set OUTPUT mode
	outl(inl(REG_GPIOB_OMD) | 0x00000010, REG_GPIOB_OMD);
	// GPB[5] set OUTPUT mode
	outl(inl(REG_GPIOB_OMD) | 0x00000020, REG_GPIOB_OMD);
	// GPB[6] set OUTPUT mode
	outl(inl(REG_GPIOB_OMD) | 0x00000040, REG_GPIOB_OMD);
	
	msleep(100);

	/* Initial sequence */
	wintek_register_set(0x01, 0x2AEF);    // Driver output control
	wintek_register_set(0x02, 0x0300);    // LCD driving AC control
	wintek_register_set(0x03, 0x080E);    // Power control 1
	wintek_register_set(0x0B, 0xD000);    // Frame cycle control
	wintek_register_set(0x0C, 0x0005);    // Power control 2
	wintek_register_set(0x0D, 0x000F);    // Power control 3
	wintek_register_set(0x0E, 0x2C00);    // Power control 4
	wintek_register_set(0x11, 0x0000);    // Shut and 8 color 
	wintek_register_set(0x12, 0x0064);    // Entry mode  // SYNC mode
//	wintek_register_set(0x12, 0x0060);    // Entry mode  // DEN mode
	wintek_register_set(0x16, 0x9F86);    // Pixel per line
	wintek_register_set(0x17, 0x0002);    // Vertical porch
	wintek_register_set(0x1E, 0x0000);    // Power control 5
	wintek_register_set(0x28, 0x0006);    // Extended command 1
	wintek_register_set(0x2A, 0x0187);    // Extended command 2
	wintek_register_set(0x30, 0x0000);    // Gamma Control 1
	wintek_register_set(0x31, 0x0103);    // Gamma Control 2
	wintek_register_set(0x32, 0x0001);    // Gamma Control 3
	wintek_register_set(0x33, 0x0501);    // Gamma Control 4
	wintek_register_set(0x34, 0x0607);    // Gamma Control 5
	wintek_register_set(0x35, 0x0406);    // Gamma Control 6
	wintek_register_set(0x36, 0x0707);    // Gamma Control 7
	wintek_register_set(0x37, 0x0305);    // Gamma Control 8
	wintek_register_set(0x3A, 0x0F0F);    // Gamma Control 9
	wintek_register_set(0x3B, 0x0F02);    // Gamma Control 10
	
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
		clock_div = w55fa93_upll_clock / 6400;
		clock_div /= 2;
		clock_div &= 0xFF;
		
		// given clock divider for VPOST 
		outl((inl(REG_CLKDIV1) & ~VPOST_N0) | 1, REG_CLKDIV1);					// divider 2 in VPOST_N0
		outl((inl(REG_CLKDIV1) & ~VPOST_N1) | (clock_div << 8), REG_CLKDIV1);	
		
	//	outl((inl(REG_CLKDIV1) & ~VPOST_S), REG_CLKDIV1);				// VPOST clock from UPLL
		outl((inl(REG_CLKDIV1) & ~VPOST_S) | (3<<3), REG_CLKDIV1);		// VPOST clock from UPLL
	
		// enable VPOST function pins
	   	outl(inl(REG_GPBFUN) | MF_GPB15, REG_GPBFUN);						// enable LPCLK pin
	   	outl(inl(REG_GPCFUN) | 0xFFFFFFFF, REG_GPCFUN);						// enable LVDATA[15:0] pins
	   	outl(inl(REG_GPDFUN) | (MF_GPD11+MF_GPD10+MF_GPD9), REG_GPDFUN);	// enable HSYNC/VSYNC/VDEN pins	
	 //  	outl(inl(REG_GPEFUN) | MF_GPE0 | MF_GPE1, REG_GPEFUN);				// enable LVDATA[17:16] pins		
	   	
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
	   	outl( (inl(REG_LCM_LCDCCtl)& ~LCDCCtl_PRDB_SEL) | (0x00 << 20), REG_LCM_LCDCCtl);	// 16-bit mode 
	//   	outl( (inl(REG_LCM_LCDCCtl)& ~LCDCCtl_PRDB_SEL) | (0x01 << 20), REG_LCM_LCDCCtl);	// 18-bit mode 
		/*
		0x0  // 16-bit mode (RGB565 output)
		0x1  // 18-bit mode (RGB666 output)
		0x2  // 24-bit mode (RGB888 output)		
		*/
		
		//Wintek_LCD_INIT();
		
		// LCDSrc, TVSrc: Frame buffer; NotchE; enable TV encoder; NTSC; Frame Buffer Size:640x480; Interlace
	  	outl((inl(REG_LCM_TVCtl)& 0xFFFF30DA)|(0x00008529), REG_LCM_TVCtl);  
	
			  
		outl((inl(REG_LCM_TVDisCtl)& 0xFE000000)|(0x00F016A0), REG_LCM_TVDisCtl);
	
	   	// set Horizontal scanning line timing for Syn type LCD 
	//	outl((inl(REG_LCM_TCON1)& 0xFF000000)|(0x00035555), REG_LCM_TCON1);
	//	outl((inl(REG_LCM_TCON1)& 0xFF000000)|(0x00030455), REG_LCM_TCON1);
		outl((inl(REG_LCM_TCON1)& 0xFF000000)|(0x00024055), REG_LCM_TCON1);
		
		// set Vertical scanning line timing for Syn type LCD   
		outl((inl(REG_LCM_TCON2)& 0xFF000000)|(0x00010C02), REG_LCM_TCON2);
	
		// set both "active pixel per line" and "active lines per screen" for Syn type LCD   
	   	outl(0x013F00F0, REG_LCM_TCON3);	// 320x240
	   	
	   	outl(0x01400001, REG_LCM_TCON4);	// signal polarity
	
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
			// Get the duty value
			copy_from_user((unsigned int *)buffer, (void *)arg, 1*sizeof(unsigned int));
			w55fa93fb_set_CMR(buffer[0]);
			break;

		case VIDEO_DISPLAY_LCD:

			printk("video displayed by LCD only\n");


			// given clock divider for VPOST 
//			outl((inl(REG_CLKDIV1) & ~VPOST_S) | (0x03 << 3), REG_CLKDIV1);	// 00:clock from CLK_in, 01:32K, 10:APLL, 11:UPLL
		
			// enable VPOST function pins
		   	outl(inl(REG_GPBFUN) | MF_GPB15, REG_GPBFUN);						// enable LPCLK pin
		   	outl(inl(REG_GPCFUN) | 0xFFFFFFFF, REG_GPCFUN);						// enable LVDATA[15:0] pins
		   	outl(inl(REG_GPDFUN) | (MF_GPD11+MF_GPD10+MF_GPD9), REG_GPDFUN);	// enable HSYNC/VSYNC/VDEN pins	
			
			// configure LCD interface  // enable sync with TV, LCD type select 
		   	outl(inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDSynTv, REG_LCM_LCDCPrm);	// async with TV
		   	outl(inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDTYPE, REG_LCM_LCDCPrm);	// High Resolution mode
		
			// configure LCD parallel data bus 
//		   	outl( (inl(REG_LCM_LCDCCtl)& ~LCDCCtl_PRDB_SEL) | (0x00 << 20), REG_LCM_LCDCCtl);	// 16-bit mode 
		   	outl( (inl(REG_LCM_LCDCCtl)& ~LCDCCtl_PRDB_SEL) | (0x01 << 20), REG_LCM_LCDCCtl);	// 18-bit mode 		   	
			
//			printk("REG_LCM_LCDCCtl = 0x%x\n", inl(REG_LCM_LCDCCtl));		   	
			/*
			0x0  // 16-bit mode (RGB565 output)
			0x1  // 18-bit mode (RGB666 output)
			0x2  // 24-bit mode (RGB888 output)		
			*/
		
			// set both "active pixel per line" and "active lines per screen" for Syn type LCD   
   			outl(0x013F00F0, REG_LCM_TCON3);	// 320x240

   			outl(0x01400001, REG_LCM_TCON4);	// signal polarity
		
			// set TV control register and LCD from frame buffer source
		   	outl((inl(REG_LCM_TVCtl)& 0xFFFFF0DA)|(0x00000410), REG_LCM_TVCtl);   // DAC disable	
		   	
			// enable LCD controller
			outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10003), REG_LCM_LCDCCtl);  // RGB565, little-endian

			break;				


		case VIDEO_DISPLAY_TV:
			printk("video displayed by TV only\n");


			// enable VPOST function pins
		   	outl(inl(REG_GPBFUN) & ~MF_GPB15, REG_GPBFUN);						// disable LPCLK pin
		   	outl(inl(REG_GPCFUN) & ~0xFFFFFFFF, REG_GPCFUN);					// disable LVDATA[15:0] pins
		   	outl(inl(REG_GPDFUN) & ~(MF_GPD11+MF_GPD10+MF_GPD9), REG_GPDFUN);	// disable HSYNC/VSYNC/VDEN pins	

			outl((inl(REG_CLKDIV1) & ~VPOST_S) | (0x00 << 3), REG_CLKDIV1);	// 00:clock from CLK_in, 01:32K, 10:APLL, 11:UPLL
			outl((inl(REG_CLKDIV1) & ~VPOST_N1) | (0x00 << 8), REG_CLKDIV1);	// divider value = 0x01 (temp)

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
#ifdef CONFIG_TWO_FB_BUF
            		while (w55fa93_edma_isbusy(VDMA_CH));
#endif            
	                
	                _auto_disable = 1;
	                break;

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
	outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x20000, REG_LCM_LCDCInt); // enable VINTEN
	
	/* enable lcd*/
	outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10003), REG_LCM_LCDCCtl);
   
   	return 0;
}

static void w55fa93fb_stop_device(void)
{
	outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFFE), REG_LCM_LCDCCtl);
}


