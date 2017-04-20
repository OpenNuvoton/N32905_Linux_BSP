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

#include "TM022HDH31.h"
extern unsigned int w55fa93_upll_clock;
extern unsigned int w55fa93_apll_clock;

#define GPIO_PORTA		1
#define GPIO_PORTB		2
#define GPIO_PORTC		4
#define GPIO_PORTD		8
#define GPIO_PORTE		16


static int gpio_setportdir(unsigned char port, unsigned short mask, unsigned short dir)
{
	switch (port) {

		case GPIO_PORTA:
			outl(inl(REG_GPIOA_OMD) & ~(mask & (mask ^ dir)), REG_GPIOA_OMD);
			outl(inl(REG_GPIOA_OMD) | (mask & dir), REG_GPIOA_OMD);			
			break;
		case GPIO_PORTB:
			outl(inl(REG_GPIOB_OMD) & ~(mask & (mask ^ dir)), REG_GPIOB_OMD);
			outl(inl(REG_GPIOB_OMD) | (mask & dir), REG_GPIOB_OMD);	
			break;			
		case GPIO_PORTC:
			outl(inl(REG_GPIOC_OMD) & ~(mask & (mask ^ dir)), REG_GPIOC_OMD);
			outl(inl(REG_GPIOC_OMD) | (mask & dir), REG_GPIOC_OMD);	
			break;		
		case GPIO_PORTD:
			outl(inl(REG_GPIOD_OMD) & ~(mask & (mask ^ dir)), REG_GPIOD_OMD);
			outl(inl(REG_GPIOD_OMD) | (mask & dir), REG_GPIOD_OMD);	
			break;	
		case GPIO_PORTE:
			outl(inl(REG_GPIOE_OMD) & ~(mask & (mask ^ dir)), REG_GPIOE_OMD);
			outl(inl(REG_GPIOE_OMD) | (mask & dir), REG_GPIOE_OMD);	
			break;		
		default:
			return(-1);
	}
	return(0);
}

static int gpio_setportval(unsigned char port, unsigned short mask, unsigned short val)
{
	switch (port) {
	
		case GPIO_PORTA:
			outl(inl(REG_GPIOA_DOUT) & ~(mask & (mask ^ val)), REG_GPIOA_DOUT);
			outl(inl(REG_GPIOA_DOUT) | (mask & val), REG_GPIOA_DOUT);			
			break;
		case GPIO_PORTB:
			outl(inl(REG_GPIOB_DOUT) & ~(mask & (mask ^ val)), REG_GPIOB_DOUT);
			outl(inl(REG_GPIOB_DOUT) | (mask & val), REG_GPIOB_DOUT);
			break;			
		case GPIO_PORTC:
			outl(inl(REG_GPIOC_DOUT) & ~(mask & (mask ^ val)), REG_GPIOC_DOUT);
			outl(inl(REG_GPIOC_DOUT) | (mask & val), REG_GPIOC_DOUT);
			break;		
		case GPIO_PORTD:
			outl(inl(REG_GPIOD_DOUT) & ~(mask & (mask ^ val)), REG_GPIOD_DOUT);
			outl(inl(REG_GPIOD_DOUT) | (mask & val), REG_GPIOD_DOUT);
			break;	
		case GPIO_PORTE:
			outl(inl(REG_GPIOE_DOUT) & ~(mask & (mask ^ val)), REG_GPIOE_DOUT);
			outl(inl(REG_GPIOE_DOUT) | (mask & val), REG_GPIOE_DOUT);
			break;		
		default:
			return(-1);
	}
	return(0);
}

static int gpio_setportpull(unsigned char port, unsigned short mask, unsigned short pull)
{
	switch (port) {
	
		case GPIO_PORTA:
			outl(inl(REG_GPIOA_PUEN) & ~(mask & (mask ^ pull)), REG_GPIOA_PUEN);
			outl(inl(REG_GPIOA_PUEN) | (mask & pull), REG_GPIOA_PUEN);			
			break;
		case GPIO_PORTB:
			outl(inl(REG_GPIOB_PUEN) & ~(mask & (mask ^ pull)), REG_GPIOB_PUEN);
			outl(inl(REG_GPIOB_PUEN) | (mask & pull), REG_GPIOB_PUEN);	
			break;			
		case GPIO_PORTC:
			outl(inl(REG_GPIOC_PUEN) & ~(mask & (mask ^ pull)), REG_GPIOC_PUEN);
			outl(inl(REG_GPIOC_PUEN) | (mask & pull), REG_GPIOC_PUEN);	
			break;	
		case GPIO_PORTD:
			outl(inl(REG_GPIOD_PUEN) & ~(mask & (mask ^ pull)), REG_GPIOD_PUEN);
			outl(inl(REG_GPIOD_PUEN) | (mask & pull), REG_GPIOD_PUEN);	
			break;
		case GPIO_PORTE:
			outl(inl(REG_GPIOE_PUEN) & ~(mask & (mask ^ pull)), REG_GPIOE_PUEN);
			outl(inl(REG_GPIOE_PUEN) | (mask & pull), REG_GPIOE_PUEN);	
			break;		
		default:
			return(-1);
	}
	return(0);
}

static int gpio_readport(unsigned char port, unsigned int *val)
{
	switch (port) {
	
		case GPIO_PORTA:
			*val = (inl(REG_GPIOA_PIN) & 0x0fff);
			break;
		case GPIO_PORTB:
			*val = (inl(REG_GPIOB_PIN) & 0xffff);
			break;			
		case GPIO_PORTC:
			*val = (inl(REG_GPIOC_PIN) & 0xffff);
			break;		
		case GPIO_PORTD:
			*val = (inl(REG_GPIOD_PIN) & 0xffff);
			break;	
		case GPIO_PORTE:
			*val = (inl(REG_GPIOE_PIN) & 0x0fff);
			break;		
		default:
			return(-1);

	}
	return(0);
}

#define	SET_TFT_BL()	gpio_setportval(GPIO_PORTE, 0x1<<0, 0x1<<0)
#define	CLR_TFT_BL()	gpio_setportval(GPIO_PORTE, 0x1<<0, 0x0<<0)

#define	SET_TFT_RST()	gpio_setportval(GPIO_PORTE, 0x1<<1, 0x1<<1)
#define	CLR_TFT_RST()	gpio_setportval(GPIO_PORTE, 0x1<<1, 0x0<<1)

#define	SET_TFT_CS()	gpio_setportval(GPIO_PORTB, 0x1<<15, 0x1<<15)
#define	CLR_TFT_CS()	gpio_setportval(GPIO_PORTB, 0x1<<15, 0x0<<15)

#define	SET_TFT_ADDR()	gpio_setportval(GPIO_PORTD, 0x1<<11, 0x1<<11)
#define	CLR_TFT_ADDR()	gpio_setportval(GPIO_PORTD, 0x1<<11, 0x0<<11)

#define	SET_TFT_WR()	gpio_setportval(GPIO_PORTD, 0x1<<9, 0x1<<9)
#define	CLR_TFT_WR()	gpio_setportval(GPIO_PORTD, 0x1<<9, 0x0<<9)

#define	SET_TFT_RD()	gpio_setportval(GPIO_PORTD, 0x1<<10, 0x1<<10)
#define	CLR_TFT_RD()	gpio_setportval(GPIO_PORTD, 0x1<<10, 0x0<<10)

#define	OUT_TFT_DATA()	gpio_setportdir(GPIO_PORTC, 0xFF<<0, 0xFF<<0)
#define	IN_TFT_DATA()	gpio_setportdir(GPIO_PORTC, 0xFF<<0, 0x00<<0)


static unsigned char LandFlag = 0;

typedef enum{
	xInc_yInc=0,
	xInc_yDec,
	xDec_yInc,
	xDec_yDec
}LCD_INC_MODE;

static void DispBackLight(unsigned char cMode)
{
  	if(cMode == 1)
		SET_TFT_BL();	
	else
		CLR_TFT_BL(); 	
}

void TFT_WriteData(char Data)
{
	gpio_setportval(GPIO_PORTC, 0xFF<<0, (int)(Data<<0));
}

char TFT_ReadData(void)
{
	int wData;

	gpio_readport(GPIO_PORTC, &wData);

	return (char)(wData>>0);
}

static void LCD_WriteIndex(unsigned char index)
{
	//configure data bus as OUTPUT
	OUT_TFT_DATA();

	//CS low, chipselect sensor
	CLR_TFT_CS();
	
	SET_TFT_RD();

	//A0 low, means to write cmd
	CLR_TFT_ADDR();

	// set address register
	TFT_WriteData(index);

	//WR low, enable write
	CLR_TFT_WR();

	//Wait for stable
	//LCD_Sleep(4);

	//WR high, disable write
	SET_TFT_WR();
	
	//CS high, deselect
	SET_TFT_CS();

	//Wait for stable
	//LCD_Sleep(10);
}

static void LCD_WriteData(unsigned char val)
{
	//configure data bus as OUTPUT
	OUT_TFT_DATA();

	//CS low, chipselect
	CLR_TFT_CS();
	
	SET_TFT_RD();

	//A0 high means to write dat
	SET_TFT_ADDR();

	// set address register
	TFT_WriteData(val);

	//WR low, enable write
	CLR_TFT_WR();

	//Wait for stable
	//LCD_Sleep(4);

	//WR high, disable write
	SET_TFT_WR();

	//CS high, deselect
	SET_TFT_CS();

	//Wait for stable
	//LCD_Sleep(10);
}

char LCD_ReadData(void)
{
	char dat;
		
	//configure data bus as OUTPUT
	IN_TFT_DATA();

	//CS low, chipselect
	CLR_TFT_CS();
	
	SET_TFT_WR();

	//A0 high means to read dat
	SET_TFT_ADDR();

	//RD low, enable read
	CLR_TFT_RD();

	//Wait for stable
	//LCD_Sleep(1);
	
	//Read Data from TFT
	dat = TFT_ReadData();

	//RD high, disable read
	SET_TFT_RD();
	
	//CS high, deselect
	SET_TFT_CS();

	//Wait for stable
	//LCD_Sleep(10);
	
	return dat;
}


static void LCD_SetAddrIncMode(LCD_INC_MODE xyDirection, unsigned char yIncFirst)
{
	unsigned int Mode;

	LCD_WriteIndex(RDDMADCTL);
	LCD_ReadData();
	Mode = LCD_ReadData();
	Mode &=0x1F;
	switch(xyDirection)
	{
		case xInc_yInc:
			Mode|=0x00;
			break;
		case xInc_yDec:
			Mode|=0x80;
			break;
		case xDec_yInc:
			Mode|=0x40;
			break;
		case xDec_yDec:
			Mode|=0xC0;
			break;	
	}
	if(yIncFirst)
		Mode |=0x20;
	LCD_WriteIndex(MADCTL);
	LCD_WriteData(Mode);	
}

volatile int s_mpu_init = 0;

static void TFT_ConfigGPIO(void)
{

	if (s_mpu_init==0)
	{
//		s_mpu_init = 1;

	//TFT_WE
	gpio_setportpull(GPIO_PORTD, 0x1<<9, 0x1<<9);
	gpio_setportdir(GPIO_PORTD, 0x1<<9, 0x1<<9);
	gpio_setportval(GPIO_PORTD, 0x1<<9, 0x1<<9);
	}	
	//TFT_OE
	gpio_setportpull(GPIO_PORTD, 0x1<<10, 0x1<<10);
	gpio_setportdir(GPIO_PORTD, 0x1<<10, 0x1<<10);
	gpio_setportval(GPIO_PORTD, 0x1<<10, 0x1<<10);
	//TFT_A0
	gpio_setportpull(GPIO_PORTD, 0x1<<11, 0x1<<11);
	gpio_setportdir(GPIO_PORTD, 0x1<<11, 0x1<<11);
	gpio_setportval(GPIO_PORTD, 0x1<<11, 0x1<<11);
	//TFT_CS
	gpio_setportpull(GPIO_PORTB, 0x1<<15, 0x1<<15);
	gpio_setportdir(GPIO_PORTB, 0x1<<15, 0x1<<15);
	gpio_setportval(GPIO_PORTB, 0x1<<15, 0x1<<15);	

	
	//TFT_RST
	gpio_setportpull(GPIO_PORTE, 0x1<<1, 0x1<<1);
	gpio_setportdir(GPIO_PORTE, 0x1<<1, 0x1<<1);
	gpio_setportval(GPIO_PORTE, 0x1<<1, 0x1<<1);
	//TFT_BL
	gpio_setportpull(GPIO_PORTE, 0x1<<0, 0x1<<0);
	gpio_setportdir(GPIO_PORTE, 0x1<<0, 0x1<<0);
	gpio_setportval(GPIO_PORTE, 0x1<<0, 0x1<<0);

	//TCS_DATA
	gpio_setportpull(GPIO_PORTC, 0xFF<<0, 0xFF<<0);
	gpio_setportdir(GPIO_PORTC, 0xFF<<0, 0xFF<<0);
	gpio_setportval(GPIO_PORTC, 0xFF<<0, 0xFF<<0);
}

void LCD_Identification(void)
{
    char tmp=0xAA;
    LCD_WriteIndex(RDDIDIF);
	tmp=LCD_ReadData();
    tmp=LCD_ReadData();
    tmp=LCD_ReadData();
    tmp=LCD_ReadData();
    tmp = tmp;    
}

void LCD_DisplayPwrMode(void)
{
    char tmp=0xAA;
    LCD_WriteIndex(RDDPM);
  
	tmp=LCD_ReadData();
    tmp=LCD_ReadData();
    tmp = tmp;   
}

void LCD_LandModeOn(void)
{
	LCD_SetAddrIncMode(xInc_yDec,TRUE);
	LandFlag = 1;
}


void LCD_DisplayStatus(void)
{
    char tmp=0xAA;
    LCD_WriteIndex(RDDST);
 
	tmp=LCD_ReadData();
    tmp=LCD_ReadData();
    tmp=LCD_ReadData();
    tmp=LCD_ReadData();
    tmp=LCD_ReadData();
    tmp = tmp;   
}

void LCD_Reset(void)
{
#if 1
	CLR_TFT_CS();
    mdelay(50);					   
	SET_TFT_RST();    
    mdelay(50);					   
    CLR_TFT_RST();		 	 
	mdelay(100);
    SET_TFT_RST();	
    mdelay(50);	
	SET_TFT_CS();    				       

#else	
	SET_TFT_RST();    
    mdelay(50);					   
    CLR_TFT_RST();		 	 
	mdelay(100);
    SET_TFT_RST();	
#endif    
}

void LCD_PowerOn(void)
{	
    LCD_Reset();

    LCD_Identification();
    LCD_DisplayStatus();
    LCD_DisplayPwrMode();
    
	LCD_WriteIndex(PWCTLB);//0xCF Power control B
	LCD_WriteData(0x00);//0x00;
	LCD_WriteData(0x81);//0x99;0x83
	LCD_WriteData(0x30);//0x30;0xb0
    mdelay(10);
	LCD_WriteIndex(PWONSCTL);//0xED,Power on sequence control
	LCD_WriteData(0x64);
	LCD_WriteData(0x03);
	LCD_WriteData(0x12);
	LCD_WriteData(0x81);
    mdelay(10);
	LCD_WriteIndex(DTIMCTLA);//0xE8,Driver timing control A
	LCD_WriteData(0x85);
	LCD_WriteData(0x10);
	LCD_WriteData(0x78);
    mdelay(10);
	LCD_WriteIndex(PWCTLA);//0xCB,Power control A
	LCD_WriteData(0x39);
	LCD_WriteData(0x2C);
	LCD_WriteData(0x00);
	LCD_WriteData(0x34);
	LCD_WriteData(0x02);
    mdelay(10);
	LCD_WriteIndex(PRCTL);//0xF7,Pump ratio control
	LCD_WriteData(0x20);
    mdelay(10);
	LCD_WriteIndex(DTIMCTLB);//0xEA,Driver timing control B
	LCD_WriteData(0x00);
	LCD_WriteData(0x00);
    mdelay(10);
    LCD_WriteIndex(FRMCTR1);//0xB1,Frame Rate Control (In Normal Mode / Full colors
	LCD_WriteData(0x00);
	LCD_WriteData(0x1B);//0x1b,70Hz;0x19   
    mdelay(10);    	
	LCD_WriteIndex(DISCTRL);//0xB6,Display Function Control
	LCD_WriteData(0x0A);
	LCD_WriteData(0xA2);
    mdelay(10);
	LCD_WriteIndex(PWCTRL1);//0xC0,Power Control 1
	LCD_WriteData(0x22);//0x35,5.50V;0x22,4.55V
    mdelay(10);
	LCD_WriteIndex(PWCTRL2);//0xC1,Power Control 2
	LCD_WriteData(0x11);//0x11,0x01
	LCD_WriteIndex(VMCTRL1);//0xC5,VCOM Control 1
	LCD_WriteData(0x5C);//0x45,4.425V;0x5c,5.000V
	LCD_WriteData(0x4C);//0x45,-0.775V;0x4c,-0.600V
    mdelay(10);
	LCD_WriteIndex(VMCTRL2);//0xC7,VCOM Control 2
	LCD_WriteData(0xA2);//0xa2,VM-30;0x8f,VM-49
    mdelay(10);
	LCD_WriteIndex(EN3G);//0xF2,Enable_3G
	LCD_WriteData(0x00);
    mdelay(10);
	LCD_WriteIndex(GAMSET);//0x26,Gamma Set
	LCD_WriteData(0x01);
    mdelay(10);
	LCD_WriteIndex(PGAMCTRL);//0xE0 Positive Gamma Control
	LCD_WriteData(0x0F);//0x0f;0x0f
	LCD_WriteData(0x21);//0x28;0x21
	LCD_WriteData(0x21);//0x24;0x21
	LCD_WriteData(0x0a);//0x0b;0x0a
	LCD_WriteData(0x10);//0x0e;0x10
	LCD_WriteData(0x0b);//0x08;0x0b
	LCD_WriteData(0x4e);//0x50;0x4e
	LCD_WriteData(0xe5);//0xb8;0xe5
	LCD_WriteData(0x3c);//0x3e;0x3c
	LCD_WriteData(0x09);//0x07;0x09
	LCD_WriteData(0x14);//0x15;0x14
	LCD_WriteData(0x09);//0x08;0x09
	LCD_WriteData(0x18);//0x1b;0x18
	LCD_WriteData(0x0b);//0x0d;0x0b
	LCD_WriteData(0x08);//0x08;0x08
    mdelay(10);
	LCD_WriteIndex(NGAMCTRL);//0xE1 Negative Gamma Correction
	LCD_WriteData(0x00);//0x08;0x00
	LCD_WriteData(0x1e);//0x17;0x1e
	LCD_WriteData(0x1e);//0x1b;0x1e
	LCD_WriteData(0x05);//0x04;0x05
	LCD_WriteData(0x0f);//0x11;0x0f
	LCD_WriteData(0x04);//0x07;0x04
	LCD_WriteData(0x31);//0x2f;0x31
	LCD_WriteData(0x73);//0x74;0x73
	LCD_WriteData(0x43);//0x41;0x43
	LCD_WriteData(0x06);//0x08;0x06
	LCD_WriteData(0x0b);//0x0a;0x0b
	LCD_WriteData(0x06);//0x07;0x06
	LCD_WriteData(0x27);//0x24;0x27
	LCD_WriteData(0x34);//0x32;0x34
	LCD_WriteData(0x0f);//0x0f;0x0f
	LCD_WriteData(0x06);
	LCD_WriteData(0x27);//0x30,0x27
	LCD_WriteData(0x34);//0x38,0x34
	LCD_WriteData(0x0F);
    mdelay(10);
    LCD_WriteIndex(MADCTL);//0x36,Memory Access Control
	LCD_WriteData(0x08);//0x08,48
    mdelay(10);
    LCD_WriteIndex(CASET);//0x2A,Column Address Set 
	LCD_WriteData(0x00);
	LCD_WriteData(0x00);
	LCD_WriteData(0x00);
	LCD_WriteData(0xEF);
    mdelay(10);
	LCD_WriteIndex(PASET);//0x2B,Page Address Set 
	LCD_WriteData(0x00);
	LCD_WriteData(0x00);
	LCD_WriteData(0x01);
	LCD_WriteData(0x3F);
    mdelay(10);
    LCD_WriteIndex(PIXSET);//0x3A,Pixel Format Set 
	LCD_WriteData(0x05);//0x05,0x66
    mdelay(10);
    LCD_WriteIndex(SLPOUT);//0x11,Sleep Out
	mdelay(200);
	LCD_WriteIndex(DISPON);//0x29,Display ON
    mdelay(200);		
    LCD_WriteIndex(RAMWR);//0x2C,Memory Write  
    mdelay(200);
}

static void LCD_SetRegion(int x_start,int y_start,int x_end,int y_end)
{		
	LCD_WriteIndex(CASET);  
	LCD_WriteData(x_start >> 8);     
	LCD_WriteData(x_start & 0xff);
	LCD_WriteData(x_end >> 8);     
	LCD_WriteData(x_end & 0xff);

	LCD_WriteIndex(PASET);  
	LCD_WriteData(y_start >> 8);     
	LCD_WriteData(y_start & 0xff);
	LCD_WriteData(y_end >> 8);     
	LCD_WriteData(y_end & 0xff);
}

static void LCD_BulkWriteDataStart(void)
{
	LCD_WriteIndex(RAMWRC);
    LCD_WriteIndex(RAMWR);
}

void LCD_SetDisplayRegion(int x,int y,int w,int h)
{
	LCD_SetRegion(x,y,x+w-1,y+h-1);
	LCD_BulkWriteDataStart();
}


void LCD_FillArea(int x,int y,int w,int h,int Color)
{
	int Count,i;
	Count=w*h;
	
	LCD_SetRegion(x,y,x+w-1,y+h-1);
	LCD_BulkWriteDataStart();
	
	// dummy write 
//	LCD_WriteData(0x00);
	
	// write data
	for(i=0;i<Count;i++)
    {
		LCD_WriteData(Color>>8);
        LCD_WriteData(Color&0xFF);
    }
}

void TIANMA_LCD_INIT(void)
{
	
//	printk("Tiana LCD initialized !!! \n");

		// set GPC[15:8] to GPIO mode (LCD data bus)
		outl(inl(REG_GPCFUN) & 0xFFFF0000, REG_GPCFUN);
	
		// set GPD[11] to GPIO mode (LCD RS)
		outl(inl(REG_GPDFUN) & ~MF_GPD11, REG_GPDFUN);
	
		// set GPB[15] to GPIO mode (LCD CS)
		outl(inl(REG_GPBFUN) & ~MF_GPB15, REG_GPBFUN);
	
		// set GPE[1] to GPIO mode (LCD RST)
		outl(inl(REG_GPEFUN) & ~MF_GPE1, REG_GPEFUN);
		
		// set GPE[0] to GPIO mode (LCD BL, backlight)
		outl(inl(REG_GPEFUN) & ~MF_GPE0, REG_GPEFUN);
	
		// set GPD[10] to GPIO mode (LCD RD)
		outl(inl(REG_GPDFUN) & ~MF_GPD10, REG_GPDFUN);
	
		// set GPD[9] to GPIO mode (LCD WR)
		outl(inl(REG_GPDFUN) & ~MF_GPD9, REG_GPDFUN);

	mdelay(100);

	TFT_ConfigGPIO();
	DispBackLight(1);

	LCD_PowerOn();
	LCD_LandModeOn();
	
	// dummy write
	LCD_WriteData(0x00);	
	LCD_SetDisplayRegion(0,0,320,240);
	
//   	LCD_FillArea(0,0,320,240,0xF800);	// red
//   	LCD_FillArea(0,0,320,240,0x07E0);   // green	
//   	LCD_FillArea(0,0,320,240,0x001F);	// blue
}

static int update_mpu_contents(int dataAddr)
{
	int i;
	char* ptr;
	static int s_swtich_pic = 0;

	gpio_setportdir(GPIO_PORTA, 0x1<<0, 0x1<<0);	// GPIOA_0 is switched to input mode in USB Host-like mode
	ptr = (char*) dataAddr;
	LCD_SetRegion(0, 0, 319, 239);
	LCD_BulkWriteDataStart();
	
	// write data
	for(i=0;i<320*240;i++)
    {
    	if (!s_swtich_pic)
    	{
	    	ptr++;
			LCD_WriteData(*ptr);
			ptr--;
			LCD_WriteData(*ptr);
			ptr+=2;
		}
		else
		{
			LCD_WriteData(i);    				
			LCD_WriteData(i);    							
		}						
    }
	return 0;
}	


#ifdef CONFIG_W55FA93_FB_INIT 

	static int w55fa93fb_mpu_trigger(void)
	{
		#if 0	
			printk("Single trigger MPU !!!\n");
			
			// single trigger mode	
			outl(inl(REG_LCM_MPUCMD)&~(MPUCMD_CMD_DISn|MPUCMD_MPU_ON|MPUCMD_MPU_CS|MPUCMD_MPU_RWn), REG_LCM_MPUCMD);
			outl(inl(REG_LCM_MPUCMD)|MPUCMD_WR_RS|MPUCMD_MPU_CS, REG_LCM_MPUCMD);						
			outl(inl(REG_LCM_MPUCMD)&~MPUCMD_DIS_SEL, REG_LCM_MPUCMD);		// select Single Mode
			outl(inl(REG_LCM_MPUCMD)&~MPUCMD_CMD_DISn, REG_LCM_MPUCMD);		// turn on Display Mode			
			outl(inl(REG_LCM_MPUCMD)|MPUCMD_MPU_ON, REG_LCM_MPUCMD);		// trigger command output		
	
		#else
			printk("Continue trigger MPU !!!\n");

			// continue trigger mode
			outl(inl(REG_LCM_MPUCMD)&~(MPUCMD_CMD_DISn|MPUCMD_MPU_ON|MPUCMD_MPU_CS|MPUCMD_MPU_RWn), REG_LCM_MPUCMD);
			outl(inl(REG_LCM_MPUCMD)|MPUCMD_WR_RS|MPUCMD_MPU_CS, REG_LCM_MPUCMD);			
			outl(inl(REG_LCM_MPUCMD)|MPUCMD_DIS_SEL, REG_LCM_MPUCMD);		// select Continue Mode
			outl(inl(REG_LCM_MPUCMD)&~MPUCMD_CMD_DISn, REG_LCM_MPUCMD);		// turn on Display Mode			
			outl(inl(REG_LCM_MPUCMD)|MPUCMD_MPU_ON, REG_LCM_MPUCMD);		// trigger command output		
		#endif
		return 0;
	}

	static int w55fa93fb_init_device(struct w55fa93fb_info *fbi)
	{
		unsigned int clock_div;
		
	  	// Reset IP
		outl(inl(REG_AHBIPRST) | VPOSTRST, REG_AHBIPRST);
		outl(inl(REG_AHBIPRST) & ~VPOSTRST, REG_AHBIPRST);	
	
		clock_div = w55fa93_upll_clock / 40000;
		clock_div &= 0xFF;
		clock_div --;
		
		// given clock divider for VPOST 
		outl((inl(REG_CLKDIV1) & ~VPOST_N0) | 0, REG_CLKDIV1);					
		outl((inl(REG_CLKDIV1) & ~VPOST_N1) | (clock_div << 8), REG_CLKDIV1);	
		
	//	outl((inl(REG_CLKDIV1) & ~VPOST_S), REG_CLKDIV1);				// VPOST clock from UPLL
		outl((inl(REG_CLKDIV1) & ~VPOST_S) | (3<<3), REG_CLKDIV1);		// VPOST clock from UPLL
	
		// configure LCD interface  // enable sync with TV, LCD type select 
	   	outl(inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDSynTv, REG_LCM_LCDCPrm);			// async with TV
	   	outl((inl(REG_LCM_LCDCPrm)& ~LCDCPrm_LCDTYPE) | 0x03, REG_LCM_LCDCPrm);	// MPU-type LCD
		/*
		0x0  // High Resolution mode
		0x1  // Sync-type TFT LCD
		0x2  // Sync-type Color STN LCD
		0x3  // MPU-type LCD
		*/

		TIANMA_LCD_INIT();

		// enable VPOST 8-bit pins 
		outl(inl(REG_GPBFUN) | 0xC0000000, REG_GPBFUN);		// enable LPCLK pin
		outl(inl(REG_GPCFUN) | 0x0000FFFF, REG_GPCFUN);		// enable LVDATA[7:0] pins
		outl(inl(REG_GPDFUN) | 0x00FC0000, REG_GPDFUN);		// enable HSYNC/VSYNC/VDEN pins	
		
		// async TV 
		outl((inl(REG_LCM_LCDCPrm) & (~LCDCPrm_LCDSynTv)), REG_LCM_LCDCPrm);		
		
		// MPU 8+8 mode
		outl(inl(REG_LCM_MPUCMD) & ~MPUCMD_MPU_SI_SEL, REG_LCM_MPUCMD);	// MPU 8+8 mode			
		
		// MPU timing
//		outl(0x05050a06, REG_LCM_MPUTS);								// MPU timing
		outl(0x01010101, REG_LCM_MPUTS);								// MPU timing
		
		// set both "active pixel per line" and "active lines per screen" for Syn type LCD   
	   	outl(0x013F00F0, REG_LCM_TCON3);	// 320x240
	   	outl(0x01400000, REG_LCM_TCON4);	// 320
	   		
		// set TV control register and LCD from frame buffer source
	   	outl((inl(REG_LCM_TVCtl)& 0xFFFFF0DA)|(0x00000410), REG_LCM_TVCtl);   // DAC disable	
	   	
		// enable LCD controller
		outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10003), REG_LCM_LCDCCtl);

		// trigger in Continue Mode
		w55fa93fb_mpu_trigger();	
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
		case VIDEO_DISPLAY_ON:
			outl(inl(REG_GPDFUN) & ~MF_GPD1, REG_GPDFUN);	
			outl(inl(REG_GPIOD_OMD) | 0x02, REG_GPIOD_OMD);				
			outl(inl(REG_GPIOD_DOUT) | 0x02, REG_GPIOD_DOUT);	// backlight ON (for Nuvoton FA93 demo board only)
			break;
			
		case VIDEO_DISPLAY_OFF:
			outl(inl(REG_GPDFUN) & ~MF_GPD1, REG_GPDFUN);	
			outl(inl(REG_GPIOD_OMD) | 0x02, REG_GPIOD_OMD);				
			outl(inl(REG_GPIOD_DOUT) & ~0x02, REG_GPIOD_DOUT);	// backlight OFF (for Nuvoton FA93 demo board only)
			break;
						
		case IOCTL_LCD_BRIGHTNESS:
			// Need to implement PWM backlight control here !!!
			// Get the duty value
			copy_from_user((unsigned int *)buffer, (void *)arg, 1*sizeof(unsigned int));
			w55fa93fb_set_CMR(buffer[0]);
			break;

		case VIDEO_DISPLAY_LCD:
			printk("video displayed by LCD only\n");
			break;				


		case VIDEO_DISPLAY_TV:
			printk("video displayed by TV only\n");
			break;				


		case IOCTL_LCD_ENABLE_INT:
        	outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x100000, REG_LCM_LCDCInt); // enable VIN
	        //_auto_disable = 0;
            break;
                
		case IOCTL_LCD_DISABLE_INT:
	    	outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) & ~0x100000, REG_LCM_LCDCInt); // disable VIN
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
	outl((inl(REG_LCM_LCDCInt) & 0xFFFFFF00) | 0x100000, REG_LCM_LCDCInt); // enable VINTEN
	
	/* enable lcd*/
	outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFF0)|(0x10003), REG_LCM_LCDCCtl);
   
   	return 0;
}

static void w55fa93fb_stop_device(void)
{
	outl((inl(REG_LCM_LCDCCtl)& 0xFFFEFFFE), REG_LCM_LCDCCtl);
}


