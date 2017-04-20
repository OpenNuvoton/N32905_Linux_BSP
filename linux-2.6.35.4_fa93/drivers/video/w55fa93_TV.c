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
extern unsigned int w55fa93_external_clock;

#ifdef CONFIG_W55FA93_FB_INIT 
	#if defined(CONFIG_TVOUT_D1_720x480)
		static int w55fa93fb_init_device(struct w55fa93fb_info *fbi)
		{
			unsigned int divider;
		
		  	// Reset IP
			outl(inl(REG_AHBIPRST) | VPOSTRST, REG_AHBIPRST);
			outl(inl(REG_AHBIPRST) & ~VPOSTRST, REG_AHBIPRST);	
	
			if (w55fa93_external_clock == 27000000)
			{
				outl(inl(REG_CLKDIV1) & ~VPOST_S, REG_CLKDIV1);		// 00:clock from CLK_in, 01:32K, 10:APLL, 11:UPLL
				outl(inl(REG_CLKDIV1) & ~VPOST_N0, REG_CLKDIV1);
				outl(inl(REG_CLKDIV1) & ~VPOST_N1, REG_CLKDIV1);	
			}
			else
			{
				divider = w55fa93_upll_clock / 27000;
					
				// given clock divider for VPOST 
			//	outl((inl(REG_CLKDIV1) & ~VPOST_S) | (0x00 << 3), REG_CLKDIV1);	// 00:clock from CLK_in, 01:32K, 10:APLL, 11:UPLL
				outl((inl(REG_CLKDIV1) & ~VPOST_S) | (0x03 << 3), REG_CLKDIV1);	// 00:clock from CLK_in, 01:32K, 10:APLL, 11:UPLL
				outl((inl(REG_CLKDIV1) & ~VPOST_N0), REG_CLKDIV1);				// 00:clock from CLK_in, 01:32K, 10:APLL, 11:UPLL	
				
				// set VPOST_N0 = 1
				divider /= 2;
				outl((inl(REG_CLKDIV1) | 0x01), REG_CLKDIV1);					// VPOST_N0 = 0x01;
			
				// set VPOST_N1
				divider --;
				outl((inl(REG_CLKDIV1) & ~VPOST_N1) | (divider << 8), REG_CLKDIV1);	// divider value = 0x0A (temp)
			}
		
			// enable VPOST function pins
		//   	outl(inl(REG_GPBFUN) | MF_GPB15, REG_GPBFUN);						// enable LPCLK pin
		//   	outl(inl(REG_GPCFUN) | 0xFFFFFFFF, REG_GPCFUN);						// enable LVDATA[15:0] pins
		//   	outl(inl(REG_GPDFUN) | (MF_GPD11+MF_GPD10+MF_GPD9), REG_GPDFUN);	// enable HSYNC/VSYNC/VDEN pins	
		   	//outl(inl(REG_GPEFUN) | MF_GPE0 | MF_GPE1, REG_GPEFUN);				// enable LVDATA[17:16] pins		
		   	
			// configure LCD interface  // enable sync with TV, LCD type select 
		   	outl(inl(REG_LCM_LCDCPrm) | LCDCPrm_LCDSynTv, REG_LCM_LCDCPrm);	// async with TV
		   	
			// LCDSrc, TVSrc: Frame buffer; NotchE; enable TV encoder; NTSC; Frame Buffer Size:720x480; Interlace
		  	outl((inl(REG_LCM_TVCtl)& 0xFFFF30DA)|(0x0000C529), REG_LCM_TVCtl);  
		
				  
		//	outl((inl(REG_LCM_TVDisCtl)& 0xFE000000)|(0x00F016A0), REG_LCM_TVDisCtl);
			outl((inl(REG_LCM_TVDisCtl)& 0xFE000000)|(0x00F01680), REG_LCM_TVDisCtl);
			
			outl(inl(REG_LCM_TVCtl)& ~0x10, REG_LCM_TVCtl);  	// enable TV DAC	
		
			return 0;
		}
		
	#elif defined(CONFIG_TVOUT_VGA_640x480)
		static int w55fa93fb_init_device(struct w55fa93fb_info *fbi)
		{
			unsigned int divider;
		
		  	// Reset IP
			outl(inl(REG_AHBIPRST) | VPOSTRST, REG_AHBIPRST);
			outl(inl(REG_AHBIPRST) & ~VPOSTRST, REG_AHBIPRST);	
	
			if (w55fa93_external_clock == 27000000)
			{
				outl(inl(REG_CLKDIV1) & ~VPOST_S, REG_CLKDIV1);		// 00:clock from CLK_in, 01:32K, 10:APLL, 11:UPLL
				outl(inl(REG_CLKDIV1) & ~VPOST_N0, REG_CLKDIV1);
				outl(inl(REG_CLKDIV1) & ~VPOST_N1, REG_CLKDIV1);	
			}
			else
			{
				divider = w55fa93_upll_clock / 27000;
					
				// given clock divider for VPOST 
			//	outl((inl(REG_CLKDIV1) & ~VPOST_S) | (0x00 << 3), REG_CLKDIV1);	// 00:clock from CLK_in, 01:32K, 10:APLL, 11:UPLL
				outl((inl(REG_CLKDIV1) & ~VPOST_S) | (0x03 << 3), REG_CLKDIV1);	// 00:clock from CLK_in, 01:32K, 10:APLL, 11:UPLL
				outl((inl(REG_CLKDIV1) & ~VPOST_N0), REG_CLKDIV1);				// 00:clock from CLK_in, 01:32K, 10:APLL, 11:UPLL	
				
				// set VPOST_N0 = 1
				divider /= 2;
				outl((inl(REG_CLKDIV1) | 0x01), REG_CLKDIV1);					// VPOST_N0 = 0x01;
			
				// set VPOST_N1
				divider --;
				outl((inl(REG_CLKDIV1) & ~VPOST_N1) | (divider << 8), REG_CLKDIV1);	// divider value = 0x0A (temp)
			}			
	
			// enable VPOST function pins
		//   	outl(inl(REG_GPBFUN) | MF_GPB15, REG_GPBFUN);						// enable LPCLK pin
		//   	outl(inl(REG_GPCFUN) | 0xFFFFFFFF, REG_GPCFUN);						// enable LVDATA[15:0] pins
		//   	outl(inl(REG_GPDFUN) | (MF_GPD11+MF_GPD10+MF_GPD9), REG_GPDFUN);	// enable HSYNC/VSYNC/VDEN pins	
		   	//outl(inl(REG_GPEFUN) | MF_GPE0 | MF_GPE1, REG_GPEFUN);				// enable LVDATA[17:16] pins		
		   	
			// configure LCD interface  // enable sync with TV, LCD type select 
		   	outl(inl(REG_LCM_LCDCPrm) | LCDCPrm_LCDSynTv, REG_LCM_LCDCPrm);	// async with TV
		   	
			// LCDSrc, TVSrc: Frame buffer; NotchE; enable TV encoder; NTSC; Frame Buffer Size:640x480; Interlace
		  	outl((inl(REG_LCM_TVCtl)& 0xFFFF30DA)|(0x00008529), REG_LCM_TVCtl);  
		
				  
		//	outl((inl(REG_LCM_TVDisCtl)& 0xFE000000)|(0x00F016A0), REG_LCM_TVDisCtl);
			outl((inl(REG_LCM_TVDisCtl)& 0xFE000000)|(0x00F016B0), REG_LCM_TVDisCtl);
			
			outl(inl(REG_LCM_TVCtl)& ~0x10, REG_LCM_TVCtl);  	// enable TV DAC	
		
			return 0;
		}
	
	#elif defined(CONFIG_TVOUT_QVGA_320x240)
	
		static int w55fa93fb_init_device(struct w55fa93fb_info *fbi)
		{
			unsigned int divider;
		
		  	// Reset IP
			outl(inl(REG_AHBIPRST) | VPOSTRST, REG_AHBIPRST);
			outl(inl(REG_AHBIPRST) & ~VPOSTRST, REG_AHBIPRST);	
		
			if (w55fa93_external_clock == 27000000)
			{
				outl(inl(REG_CLKDIV1) & ~VPOST_S, REG_CLKDIV1);		// 00:clock from CLK_in, 01:32K, 10:APLL, 11:UPLL
				outl(inl(REG_CLKDIV1) & ~VPOST_N0, REG_CLKDIV1);
				outl(inl(REG_CLKDIV1) & ~VPOST_N1, REG_CLKDIV1);	
			}
			else
			{
				divider = w55fa93_upll_clock / 27000;
					
				// given clock divider for VPOST 
			//	outl((inl(REG_CLKDIV1) & ~VPOST_S) | (0x00 << 3), REG_CLKDIV1);	// 00:clock from CLK_in, 01:32K, 10:APLL, 11:UPLL
				outl((inl(REG_CLKDIV1) & ~VPOST_S) | (0x03 << 3), REG_CLKDIV1);	// 00:clock from CLK_in, 01:32K, 10:APLL, 11:UPLL
				outl((inl(REG_CLKDIV1) & ~VPOST_N0), REG_CLKDIV1);				// 00:clock from CLK_in, 01:32K, 10:APLL, 11:UPLL	
				
				// set VPOST_N0 = 1
				divider /= 2;
				outl((inl(REG_CLKDIV1) | 0x01), REG_CLKDIV1);					// VPOST_N0 = 0x01;
			
				// set VPOST_N1
				divider --;
				outl((inl(REG_CLKDIV1) & ~VPOST_N1) | (divider << 8), REG_CLKDIV1);	// divider value = 0x0A (temp)
			}			
	
			// enable VPOST function pins
		//   	outl(inl(REG_GPBFUN) | MF_GPB15, REG_GPBFUN);						// enable LPCLK pin
		//   	outl(inl(REG_GPCFUN) | 0xFFFFFFFF, REG_GPCFUN);						// enable LVDATA[15:0] pins
		//   	outl(inl(REG_GPDFUN) | (MF_GPD11+MF_GPD10+MF_GPD9), REG_GPDFUN);	// enable HSYNC/VSYNC/VDEN pins	
		   	//outl(inl(REG_GPEFUN) | MF_GPE0 | MF_GPE1, REG_GPEFUN);				// enable LVDATA[17:16] pins		
		   	
			// configure LCD interface  // enable sync with TV, LCD type select 
		   	outl(inl(REG_LCM_LCDCPrm) | LCDCPrm_LCDSynTv, REG_LCM_LCDCPrm);	// sync with TV
		   	
			// LCDSrc, TVSrc: Frame buffer; NotchE; enable TV encoder; NTSC; Frame Buffer Size:320x240; Interlace
		  	outl((inl(REG_LCM_TVCtl)& 0xFFFF30D2)|(0x00000521), REG_LCM_TVCtl);  
		
				  
		//	outl((inl(REG_LCM_TVDisCtl)& 0xFE000000)|(0x00F016A0), REG_LCM_TVDisCtl);
		//	outl((inl(REG_LCM_TVDisCtl)& 0xFE000000)|(0x00F016B0), REG_LCM_TVDisCtl);
			outl((inl(REG_LCM_TVDisCtl)& 0xFE000000)|(0x10F016A5), REG_LCM_TVDisCtl);	
		
		  	outl(inl(REG_LCM_TVCtl)& ~0x10, REG_LCM_TVCtl);  	// enable TV DAC
		  	
			return 0;
		}
	#endif
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
			outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10003), REG_LCM_LCDCCtl);
			break;
			
		case VIDEO_DISPLAY_OFF:
			outl((inl(REG_LCM_LCDCCtl)& 0xFFFFFFFE), REG_LCM_LCDCCtl);

			
			break;
		case IOCTL_LCD_BRIGHTNESS:
			// Need to implement PWM backlight control here !!!
			printk("PWM backlight control is not implemented yet!\n");
			break;

		case VIDEO_DISPLAY_LCD:
			printk("video displayed by LCD only\n");
			break;				


		case VIDEO_DISPLAY_TV:
			printk("video displayed by TV only\n");
  			outl(inl(REG_LCM_TVCtl)& ~0x10, REG_LCM_TVCtl);  	// enable TV DAC
			break;				
              
		 case IOCTL_LCD_ENABLE_INT:
//            	outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x20000, REG_LCM_LCDCInt); // enable VIN
//	                _auto_disable = 0; 
//				_auto_disable = 1;               
                outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x40000, REG_LCM_LCDCInt); // enable VIN

                break;
                
	        case IOCTL_LCD_DISABLE_INT:
//	            outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) & ~0x20000, REG_LCM_LCDCInt); // disable VIN
	            outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) & ~0x40000, REG_LCM_LCDCInt); // disable VIN	                

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
}

static int w55fa93fb_probe_device(void)
{

	outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x40000, REG_LCM_LCDCInt); // enable TVFILED_INTEN
	
	/* enable lcd*/
	outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10003), REG_LCM_LCDCCtl);
	//outl(inl(REG_GPIOE_DOUT) & ~(1 << 7), REG_GPIOE_DOUT); // turn off back light
   
   	return 0;
}

static void w55fa93fb_stop_device(void)
{
	outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFFE), REG_LCM_LCDCCtl);
	outl(inl(REG_LCM_TVCtl)|0x10, REG_LCM_TVCtl);  	// disable TV DAC	
}

