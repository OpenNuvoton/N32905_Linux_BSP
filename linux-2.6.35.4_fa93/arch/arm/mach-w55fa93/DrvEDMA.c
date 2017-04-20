/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved. *
 *                                                              *
 ****************************************************************/

//#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/clk.h>
#include <mach/w55fa93_reg.h>
#include <mach/DrvEDMA.h>

static PFN_DRVEDMA_CALLBACK  g_pfnEDMACallback[MAX_CHANNEL_NUM+1][4]  = {
	{0,0,0,0},
	{0,0,0,0},
	{0,0,0,0},
	{0,0,0,0},
	{0,0,0,0}
};	

static struct clk *g_clk[MAX_CHANNEL_NUM+1];

#define	MAX_GS_TRANSFER_SIZE	32*1024

void DrvEDMA_EnableScatterGather(
    E_DRVEDMA_CHANNEL_INDEX eChannel
);

  
void DrvEDMA_DisableScatterGather(
    E_DRVEDMA_CHANNEL_INDEX eChannel
);

unsigned int  
DrvEDMA_Open(void)
{
	// 1.Check I/O pins. If I/O pins are used by other IPs, return error code.
	// 2.Enable IP¡¦s clock --> Enable Channel clock in DrvEDMA_SetCHOperation( ) function
	// 3.Reset IP
	// 4.Configure IP according to inputted arguments.
	// 5.Enable IP I/O pins
	// 6.Return 0 to present success
	g_clk [0] = clk_get(NULL, "edma0");
	g_clk [1] = clk_get(NULL, "edma1");
	g_clk [2] = clk_get(NULL, "edma2");
	g_clk [3] = clk_get(NULL, "edma3");
	g_clk [4] = clk_get(NULL, "edma4");
	return E_SUCCESS;    
}

void DrvEDMA_Close(void)
{
    //unsigned int u32Mask;
    unsigned int i;
    
	// 1.Disable IP I/O pins
	// 2.Disable IP¡¦s clock --> Disable Channel Clock in DrvEDMA_SetCHOperation( ) function
	//u32Mask = EDMA0_CKE | EDMA1_CKE | EDMA2_CKE | EDMA3_CKE | EDMA4_CKE;
	//outp32(REG_AHBCLK,inp32(REG_AHBCLK) & ~u32Mask);
	for(i=0; i<=MAX_CHANNEL_NUM; i++)
	{		
		clk_put(g_clk[i]);
	}
	
}

// Get Channel Enable/Disable status
bool_type_t  
DrvEDMA_IsCHBusy(
	E_DRVEDMA_CHANNEL_INDEX eChannel
)
{
    unsigned int u32SFR;
    
    if (eChannel > MAX_CHANNEL_NUM)
        return E_DRVEDMA_FALSE_INPUT;    
    
    u32SFR = (u32)REG_VDMA_CSR + eChannel * 0x100;
    
    if (inp32(u32SFR) &  TRIG_EN)
        return TRUE;
    else
        return FALSE;    
}

// Set Channel Enable/Disable & Enable Channel Clock
void DrvEDMA_EnableCH(
	E_DRVEDMA_CHANNEL_INDEX eChannel,
	E_DRVEDMA_OPERATION eOP
)
{
    unsigned int u32SFR;
		        
    u32SFR = (u32)REG_VDMA_CSR + eChannel * 0x100;
  
    if (eOP == eDRVEDMA_DISABLE)
    {
    	outp32(u32SFR, inp32(u32SFR) & ~EDMACEN); 
	clk_disable(g_clk[eChannel]);    
	}    	
	else 
	{
	clk_enable(g_clk[eChannel]);
    	outp32(u32SFR, inp32(u32SFR) | EDMACEN); 	   	
	}    	
}

// Get Channel Enable/Disable status
bool_type_t  
DrvEDMA_IsEnabledCH(
	E_DRVEDMA_CHANNEL_INDEX eChannel
)
{
    unsigned int u32SFR;
    
    if (eChannel > MAX_CHANNEL_NUM)
        return E_DRVEDMA_FALSE_INPUT;    
    
    u32SFR = (u32)REG_VDMA_CSR + eChannel * 0x100;
    
    return inp32(u32SFR) & EDMACEN;
}

// Set Source/Destination Address and Direction and Transfer Length for Channelx
int  
DrvEDMA_SetTransferSetting(
    E_DRVEDMA_CHANNEL_INDEX eChannel, 
    S_DRVEDMA_CH_ADDR_SETTING* psSrcAddr, 
    S_DRVEDMA_CH_ADDR_SETTING* psDestAddr, 
    unsigned int u32TransferLength
)
{
    unsigned int u32SFR, u32Value;    
    
    if (eChannel > MAX_CHANNEL_NUM)
        return E_DRVEDMA_FALSE_INPUT;    
    
    DrvEDMA_DisableScatterGather(eChannel);
    
    u32SFR = (u32)REG_VDMA_CSR + eChannel * 0x100;
    u32Value = inp32(u32SFR);        
        
    outp32(REG_VDMA_SAR + eChannel * 0x100, psSrcAddr->u32Addr);
    
    u32Value = (u32Value & ~SAD_SEL) | (psSrcAddr->eAddrDirection << SOURCE_DIRECTION_BIT);
    
    outp32(REG_VDMA_DAR + eChannel * 0x100, psDestAddr->u32Addr);
    u32Value = (u32Value & ~DAD_SEL) | (psDestAddr->eAddrDirection << DESTINATION_DIRECTION_BIT);
    outp32(u32SFR,u32Value);

    if (u32TransferLength > MAX_TRANSFER_BYTE_COUNT)
        return E_DRVEDMA_FALSE_INPUT;
        
    outp32(REG_VDMA_BCR + eChannel * 0x100,u32TransferLength);
    
    return E_SUCCESS;     
}

// Get Source/Destination Address and Direction from Channelx
int    
DrvEDMA_GetTransferSetting(
    E_DRVEDMA_CHANNEL_INDEX eChannel, 
    E_DRVEDMA_TARGET eTarget, 
    unsigned int* pu32Addr, 
    E_DRVEDMA_DIRECTION_SELECT* peDirection
)
{
    unsigned int u32SFR, u32Value;

    if (eChannel > MAX_CHANNEL_NUM)
        return E_DRVEDMA_FALSE_INPUT;     
    
    u32SFR = (u32)REG_VDMA_CSR + eChannel * 0x100;
    
    if ((eTarget != eDRVEDMA_TARGET_SOURCE) && (eTarget != eDRVEDMA_TARGET_DESTINATION))
        return E_DRVEDMA_FALSE_INPUT;
    
    u32Value = inp32(u32SFR);
    
    if (eTarget == eDRVEDMA_TARGET_SOURCE)
    {
        *pu32Addr = inp32(REG_VDMA_SAR + eChannel * 0x100);
        *peDirection = (u32Value & SAD_SEL) >> SOURCE_DIRECTION_BIT;
    }
    else
    {
        *pu32Addr = inp32(REG_VDMA_DAR + eChannel * 0x100);
        *peDirection = (u32Value & DAD_SEL) >> DESTINATION_DIRECTION_BIT;        
    }
    
    return E_SUCCESS;     
}

// Get Transfer Length from Channelx
int    
DrvEDMA_GetTransferLength(
    E_DRVEDMA_CHANNEL_INDEX eChannel, 
    unsigned int* pu32TransferLength
)
{
    unsigned int u32SFR;
    
    if (eChannel > MAX_CHANNEL_NUM)
        return E_DRVEDMA_FALSE_INPUT; 
            
    u32SFR = (u32)REG_VDMA_BCR + eChannel * 0x100;
    
    *pu32TransferLength = inp32(u32SFR);
    
    return E_SUCCESS;       
}


// Set APB Transfer Width for Channelx
int  
DrvEDMA_SetAPBTransferWidth(
    E_DRVEDMA_CHANNEL_INDEX eChannel, 
    E_DRVEDMA_TRANSFER_WIDTH eTransferWidth
)
{
    unsigned int u32SFR;
    
    if ((eChannel > MAX_CHANNEL_NUM) || (eChannel ==0))
        return E_DRVEDMA_FALSE_INPUT; 
            
    u32SFR = (u32)REG_VDMA_CSR + eChannel * 0x100;
    outp32(u32SFR,(inp32(u32SFR) & ~APB_TWS) | (eTransferWidth << TRANSFER_WIDTH_BIT));
        
    return E_SUCCESS;     
}

// Get Transfer Width from Channelx
int    
DrvEDMA_GetAPBTransferWidth(
    E_DRVEDMA_CHANNEL_INDEX eChannel, 
    E_DRVEDMA_TRANSFER_WIDTH* peTransferWidth
)
{
    unsigned int u32SFR;
    
    if (eChannel > MAX_CHANNEL_NUM)
        return E_DRVEDMA_FALSE_INPUT; 
            
    u32SFR = (u32)REG_VDMA_CSR + eChannel * 0x100;
    *peTransferWidth = (inp32(u32SFR) & APB_TWS) >> TRANSFER_WIDTH_BIT;    
    
    return E_SUCCESS;    
}

// Select EDMA channel for APB Device
int  
DrvEDMA_SetCHForAPBDevice(
    E_DRVEDMA_CHANNEL_INDEX eChannel, 
    E_DRVEDMA_APB_DEVICE eDevice,
    E_DRVEDMA_APB_RW eRWAPB    
)
{
    unsigned int u32Value,u32Mask,i,u32OrMask; 
    unsigned int u32SFR;       
    
    if ((eChannel > MAX_CHANNEL_NUM) || (eChannel ==0))
        return E_DRVEDMA_FALSE_INPUT; 
            
    if ((eRWAPB == eDRVEDMA_WRITE_APB) && (eDevice == eDRVEDMA_ADC))
        return E_DRVEDMA_FALSE_INPUT;
            	
    u32SFR = (u32)REG_VDMA_CSR + eChannel * 0x100;
                	
    // Let Tx and Rx does not use the same channel         	
    if (eRWAPB == eDRVEDMA_WRITE_APB)
    {
	    outp32(u32SFR,(inp32(u32SFR) & ~MODE_SEL) | (eDRVEDMA_MEMORY_TO_DEVICE << MODE_SELECT_BIT));    
        u32Mask = 0x000F0000 << ((eChannel-1)*4);
        u32OrMask = 0x0007 << ((eChannel-1)*4);
	}        
    else
    {
	    outp32(u32SFR,(inp32(u32SFR) & ~MODE_SEL) | (eDRVEDMA_DEVICE_TO_MEMORY << MODE_SELECT_BIT));      
        u32Mask = 0x0000000F << ((eChannel-1)*4);
        u32OrMask = 0x00070000 << ((eChannel-1)*4);        
	}        

    u32Value = inp32(REG_EDSSR) & ~u32Mask;
    
    if ((eDevice == eDRVEDMA_SPIMS0) || (eDevice == eDRVEDMA_SPIMS1))
	    u32Value &= ~u32OrMask;    
    else
	    u32Value |= u32OrMask;

	// let Tx or Rx does not use two channel concurrently    
    if (eRWAPB == eDRVEDMA_WRITE_APB)
    {
	    u32Mask = 0x070000;
       	for(i=1;i<=4;i++)
       	{
       		if (i != eChannel)
       		{
	       		if (((u32Value & u32Mask)>>((i-1)*4+16)) == eDevice)
	       		{
	       			u32Value |= u32Mask;
	       			
			   		if ((eDevice == eDRVEDMA_SPIMS0) || (eDevice == eDRVEDMA_SPIMS1))
			   			u32Value |=  0x07 << ((i-1)*4);	       			
				}	       			
			}       			
       		u32Mask <<= 4;	
		}        
		

   		u32Value |=  eDevice << ((eChannel-1)*4 + 16);
   		
   		// TX and RX use same channel for SPIM0 & 1 
   		if ((eDevice == eDRVEDMA_SPIMS0) || (eDevice == eDRVEDMA_SPIMS1))
   			u32Value |=  eDevice << ((eChannel-1)*4);
    }
    else
    {
	    u32Mask = 0x07;
       	for(i=1;i<=4;i++)
       	{
       		if (i != eChannel)
       		{
	      		if (((u32Value & u32Mask)>>(i-1)*4) == eDevice)
	      		{
	       			u32Value |= u32Mask;
	       			
			   		if ((eDevice == eDRVEDMA_SPIMS0) || (eDevice == eDRVEDMA_SPIMS1))
			   			u32Value |=  0x070000 << ((i-1)*4);	  	       			
				}			   			
       		}	
       		u32Mask <<= 4;	
		}  
		    
    	u32Value |=  eDevice << ((eChannel-1)*4);    
    	
    	// TX and RX use same channel for SPIM0 & 1 
   		if ((eDevice == eDRVEDMA_SPIMS0) || (eDevice == eDRVEDMA_SPIMS1))
   			u32Value |=  eDevice << ((eChannel-1)*4+16);    	
    }

    outp32(REG_EDSSR,u32Value);
        
    return E_SUCCESS;     
}

// Get EDMA channel for APB Device
E_DRVEDMA_CHANNEL_INDEX  
DrvEDMA_GetCHForAPBDevice(
    E_DRVEDMA_APB_DEVICE eDevice,
    E_DRVEDMA_APB_RW eRWAPB    
)
{
	unsigned int u32Value,i;
	
    if ((eDevice < eDRVEDMA_SPIMS0) || (eDevice > eDRVEDMA_ADC))
    	return E_DRVEDMA_FALSE_INPUT;
    	
	if (eRWAPB == eDRVEDMA_WRITE_APB)
	{
		u32Value = inp32(REG_EDSSR)>>16;
	}
	else
	{
		u32Value = inp32(REG_EDSSR);
	}
	
   	for(i=1;i<=4;i++)
   	{
   		if (((u32Value >> (i-1)*4) & 0x07) == eDevice)
   			return i;
	} 	
	
   	return 0;  
}

// Set Wrap Around Transfer Byte count interrupt Select for Channelx
int  
DrvEDMA_SetWrapIntType(
    E_DRVEDMA_CHANNEL_INDEX eChannel,
    E_DRVEDMA_WRAPAROUND_SELECT eType
)
{
    unsigned int u32SFR;
    
    if ((eChannel > MAX_CHANNEL_NUM) || (eChannel == 0))
        return E_DRVEDMA_FALSE_INPUT;
            
    u32SFR = (u32)REG_VDMA_CSR + eChannel * 0x100;
    
    if (eType > 0x0F)  
        return E_DRVEDMA_FALSE_INPUT;    
        
    outp32(u32SFR,(inp32(u32SFR) & ~WAR_BCR_SEL) | (eType<<12));    
    
    return E_SUCCESS;  
}

// Get Wrap Around Transfer Byte count interrupt Select from Channelx
E_DRVEDMA_WRAPAROUND_SELECT  
DrvEDMA_GetWrapIntType(
    E_DRVEDMA_CHANNEL_INDEX eChannel
)
{
    unsigned int u32SFR;
    
    if ((eChannel > MAX_CHANNEL_NUM) || (eChannel == 0))
        return E_DRVEDMA_FALSE_INPUT;
            
    u32SFR = (u32)REG_VDMA_CSR + eChannel * 0x100;  
    
    return (inp32(u32SFR) & WAR_BCR_SEL )>> 12;  
}

// Software reset Channelx
int  
DrvEDMA_CHSoftwareReset(
    E_DRVEDMA_CHANNEL_INDEX eChannel
)
{
    unsigned int u32SFR;
    
    if (eChannel > MAX_CHANNEL_NUM)
        return E_DRVEDMA_FALSE_INPUT;
            
    u32SFR = (u32)REG_VDMA_CSR + eChannel * 0x100;  
    
    outp32(u32SFR, inp32(u32SFR) | SW_RST);  
    
    return E_SUCCESS;     
}

// Enable EDMA data read or write Transfer
int  
DrvEDMA_CHEnablelTransfer(
    E_DRVEDMA_CHANNEL_INDEX eChannel
)
{
    unsigned int u32SFR;
    
    if (eChannel > MAX_CHANNEL_NUM)
        return E_DRVEDMA_FALSE_INPUT;
            
    u32SFR = (u32)REG_VDMA_CSR + eChannel * 0x100;  
    
    outp32(u32SFR, inp32(u32SFR) | TRIG_EN | EDMACEN);  
    
    return E_SUCCESS;      
}

// Get Current Source Address from Channelx
unsigned int  
DrvEDMA_GetCurrentSourceAddr(
    E_DRVEDMA_CHANNEL_INDEX eChannel
)
{
    unsigned int u32SFR;
    
    u32SFR = (u32)REG_VDMA_CSAR + eChannel * 0x100;   
    
    return inp32(u32SFR);      
}

// Get Current Destination Address from Channelx
unsigned int  
DrvEDMA_GetCurrentDestAddr(
    E_DRVEDMA_CHANNEL_INDEX eChannel
)
{
    unsigned int u32SFR;
    
    u32SFR = (u32)REG_VDMA_CDAR + eChannel * 0x100;   
    
    return inp32(u32SFR);      
}

// Get Current Transfer Count from Channelx
unsigned int  
DrvEDMA_GetCurrentTransferCount(
    E_DRVEDMA_CHANNEL_INDEX eChannel
)
{
    unsigned int u32SFR;
    
    u32SFR = (u32)REG_VDMA_CBCR + eChannel * 0x100;   
    
    return inp32(u32SFR);   
}

// Enable Interrupt for Channelx
int  
DrvEDMA_EnableInt(
    E_DRVEDMA_CHANNEL_INDEX eChannel, 
    E_DRVEDMA_INT_ENABLE eIntSource
)
{
    unsigned int u32SFR;
    
    u32SFR = (u32)REG_VDMA_IER + eChannel * 0x100;        
    
    if ((eIntSource <1) || (eIntSource >15)) 
        return E_DRVEDMA_FALSE_INPUT;

    outp32(u32SFR, inp32(u32SFR) | eIntSource);  
    
    return E_SUCCESS;      
}

// Disable Interrupt for Channelx
void DrvEDMA_DisableInt(
    E_DRVEDMA_CHANNEL_INDEX eChannel, 
    E_DRVEDMA_INT_ENABLE eIntSource
)
{
    unsigned int u32SFR;
    
    u32SFR = (u32)REG_VDMA_IER + eChannel * 0x100;        
    
    outp32(u32SFR, inp32(u32SFR) & ~eIntSource);  
              
}

// Check if the specified interrupt source is enabled in Channelx
unsigned int  
DrvEDMA_IsIntEnabled(
    E_DRVEDMA_CHANNEL_INDEX eChannel,
	E_DRVEDMA_INT_ENABLE eIntSource    
)
{
    unsigned int u32SFR;
    
    u32SFR = (u32)REG_VDMA_ISR + eChannel * 0x100;    
    
    switch(eIntSource)
    {
    	case eDRVEDMA_TABORT:
		    return inp32(u32SFR) & EDMATABORT_IE;     	
    		break;
    	case eDRVEDMA_BLKD:
		    return inp32(u32SFR) & BLKD_IE;     	
    		break;
    	case eDRVEDMA_WAR:
		    return inp32(u32SFR) & WAR_IE;     	
    		break;
    	case eDRVEDMA_SG:
		    return inp32(u32SFR) & SG_IEN;     	
    		break;    		
		default :
	    	return E_DRVEDMA_FALSE_INPUT;		    		
	}    
}

// Clear Interrupt Status for Channelx
void DrvEDMA_ClearInt(
    E_DRVEDMA_CHANNEL_INDEX eChannel, 
    E_DRVEDMA_INT_FLAG eIntFlag
)
{
    unsigned int u32SFR;
    
    u32SFR = (u32)REG_VDMA_ISR + eChannel * 0x100;    
    
    outp32(u32SFR, eIntFlag);
}

bool_type_t
DrvEDMA_PollInt(
    E_DRVEDMA_CHANNEL_INDEX eChannel, 
    E_DRVEDMA_INT_FLAG eIntFlag
)
{
    unsigned int u32SFR;
    
    u32SFR = (u32)REG_VDMA_ISR + eChannel * 0x100;    
    
    return inp32(u32SFR) & eIntFlag;
}

// Set Color Format Transform for Channel0
int  
DrvEDMA_SetColorTransformFormat(
	E_DRVEDMA_COLOR_FORMAT eSourceFormat,
	E_DRVEDMA_COLOR_FORMAT eDestFormat
)
{ 
	outp32(REG_VDMA_CTCSR, (inp32(REG_VDMA_CTCSR) & ~(SOUR_FORMAT | DEST_FORMAT)) | eSourceFormat<<24 | eDestFormat<<16);
	return E_SUCCESS;  
}

// Get Color Format Transform from Channel0
void DrvEDMA_GetColorTransformFormat(
    E_DRVEDMA_COLOR_FORMAT* peSourceFormat,
    E_DRVEDMA_COLOR_FORMAT* peDestFormat
)
{
	*peSourceFormat = (inp32(REG_VDMA_CTCSR) & SOUR_FORMAT) >> 24;
	*peDestFormat = (inp32(REG_VDMA_CTCSR) & DEST_FORMAT) >> 16;	
}

int  
DrvEDMA_SetColorTransformOperation(
	E_DRVEDMA_OPERATION eColorSpaceTran,
	E_DRVEDMA_OPERATION eStrideMode
)
{ 
	outp32(REG_VDMA_CTCSR, (inp32(REG_VDMA_CTCSR) & ~(COL_TRA_EN | STRIDE_EN)) | eColorSpaceTran<<1 | eStrideMode);
	return E_SUCCESS;  
}

void DrvEDMA_GetColorTransformOperation(
	E_DRVEDMA_OPERATION* peColorSpaceTran,
	E_DRVEDMA_OPERATION* peStrideMode
)
{ 

	*peColorSpaceTran = (inp32(REG_VDMA_CTCSR) & COL_TRA_EN) >> 1;
	*peStrideMode = inp32(REG_VDMA_CTCSR) & STRIDE_EN ;	
}



// Set Source Stride Transfer Byte Count & Offset Byte Length
// Only Channel0 support this function
int  
DrvEDMA_SetSourceStride(
    unsigned int u32StrideByteCount,
    unsigned int u32OffsetByteLength
)
{
    
    outp32(REG_VDMA_SASOCR,(u32StrideByteCount<<16) |u32OffsetByteLength);   
    
    return E_SUCCESS;     
}

// Get Source/Destination Stride Transfer Byte Count & Offset Byte Length
void DrvEDMA_GetSourceStride(
    unsigned int* pu32StrideByteCount,
    unsigned int* pu32OffsetByteLength
)
{
     
    *pu32StrideByteCount = (inp32(REG_VDMA_SASOCR) & STBC) >> 16;   
    *pu32OffsetByteLength = (inp32(REG_VDMA_SASOCR) & SASTOBL);     
}

// Set Destination Stride Transfer Byte Count & Offset Byte Length
// Only Channel0 support this function
int  
DrvEDMA_SetDestinationStrideOffset(
    unsigned int u32OffsetByteLength
)
{
    
    outp32(REG_VDMA_DASOCR,u32OffsetByteLength & DASTOBL);   
    
    return E_SUCCESS;     
}

// Get Destination Stride Offset Byte Length
void DrvEDMA_GetDestinationStrideOffset(
    unsigned int* pu32OffsetByteLength
)
{
     
    *pu32OffsetByteLength = (inp32(REG_VDMA_DASOCR) & DASTOBL);     
}

// Set Channel0 Clamping function 
void DrvEDMA_SetClamping(
	E_DRVEDMA_OPERATION eOP
)
{
    
    if (eOP == eDRVEDMA_DISABLE)
    	outp32(REG_VDMA_CTCSR, inp32(REG_VDMA_CTCSR) & ~CLAMPING_EN);     
	else 
    	outp32(REG_VDMA_CTCSR, inp32(REG_VDMA_CTCSR) | CLAMPING_EN); 	   	
}

// Get Channel0 Clamping status
E_DRVEDMA_OPERATION  
DrvEDMA_GetClamping(void)
{
    
    return (inp32(REG_VDMA_CTCSR) & CLAMPING_EN) >> 7;
}

// Get Channel 1 ~ 4 Internal Buffer Pointer
unsigned int  
DrvEDMA_GetInternalBufPointer(
	E_DRVEDMA_CHANNEL_INDEX eChannel
)
{
    unsigned int u32SFR;
    
    //if ((eChannel > MAX_CHANNEL_NUM) || (eChannel == 0))
    //    return E_DRVEDMA_FALSE_INPUT;    
    
    u32SFR = (u32)REG_PDMA_POINT1 + (eChannel-1) * 0x100; 
    
    return (inp32(u32SFR) & PDMA_POINT);
}

// Get Shared Buffer Content from Channelx
unsigned int  
DrvEDMA_GetSharedBufData(
    E_DRVEDMA_CHANNEL_INDEX eChannel, 
    unsigned int u32BufIndex
)
{
    unsigned int u32SFR;
    
    u32SFR = (u32)REG_EDMA_SBUF0_C0 + eChannel * 0x100 + u32BufIndex * 4;
    
    return inp32(u32SFR);
    
    
}

// EDMA ISR
void DrvEDMA_ISR(void)
//static irqreturn_t DrvEDMA_ISR(int irq, void *dev_id)
{
    unsigned int u32IntStatus;
    unsigned int u32WraparoundStatus;
    
    if (inp32(REG_VDMA_ISR) & INTR)
    {
    	if (inp32(REG_VDMA_ISR) & INTR0)
    	{
	    	u32IntStatus = inp32(REG_VDMA_ISR) & inp32(REG_VDMA_IER);
    		if (u32IntStatus & EDMATABORT_IF)
    		{
    			outp32(REG_VDMA_ISR,EDMATABORT_IF);
    			if (g_pfnEDMACallback[0][0] != 0)
	    			(*g_pfnEDMACallback[0][0])(0);    			
			}    			
    		else 
    		{
    			if (u32IntStatus & EDMABLKD_IF)
    			{
//printk("ISR EDMABLKD_IF\n");
				outp32(REG_VDMA_ISR,EDMABLKD_IF);
	    			if (g_pfnEDMACallback[0][1] != 0)
		    			(*g_pfnEDMACallback[0][1])(0);	    				    			
				}
			else if (u32IntStatus & EDMASG_IF)
			{
//printk("ISR EDMASG_IF\n");
				outp32(REG_VDMA_ISR,EDMASG_IF);
	    			if (g_pfnEDMACallback[0][3] != 0)
		    			(*g_pfnEDMACallback[0][3])(0);
	    								
			}
					    			
    		}
    	}
    	else if (inp32(REG_VDMA_ISR) & INTR1)
    	{
    		if (inp32(REG_PDMA_IER1) & WAR_IE)
	    		u32IntStatus = inp32(REG_PDMA_ISR1) & (inp32(REG_PDMA_IER1) | 0x0F00);
	    	else
	    		u32IntStatus = inp32(REG_PDMA_ISR1) & inp32(REG_PDMA_IER1);	    	
	    	
    		if (u32IntStatus & EDMATABORT_IF)
    		{
    			outp32(REG_PDMA_ISR1,EDMATABORT_IF);
    			if (g_pfnEDMACallback[1][0] != 0)    		
	    			(*g_pfnEDMACallback[1][0])(0);    			
			}    			
    		else
    		{
    			if (u32IntStatus & EDMABLKD_IF)
    			{
    				outp32(REG_PDMA_ISR1,EDMABLKD_IF);
	    			if (g_pfnEDMACallback[1][1] != 0)    			
		    			(*g_pfnEDMACallback[1][1])(0);	    				    			
				}	   
	    		else if (u32IntStatus & EDMASG_IF)
	    		{
	    			outp32(REG_PDMA_ISR1,EDMASG_IF);
	    			if (g_pfnEDMACallback[1][3] != 0)
		    			(*g_pfnEDMACallback[1][3])(0);	    				    		
	    		}				 			
    			else
    			{
	    			u32WraparoundStatus = inp32(REG_PDMA_ISR1) & 0x0F00;
	    			if (u32WraparoundStatus)
	    			{
		    			if (u32WraparoundStatus & 0x0200)
		    				u32WraparoundStatus = 0x0200;
		    			else if (u32WraparoundStatus & 0x0400)
		    				u32WraparoundStatus = 0x0400;
		    			else  if (u32WraparoundStatus & 0x0800)
		    				u32WraparoundStatus = 0x0800;
		    			else		   
		    				u32WraparoundStatus = 0x0100; 	

					outp32(REG_PDMA_ISR1,u32WraparoundStatus);				
		    			if (g_pfnEDMACallback[1][2] != 0)		    						
			    			(*g_pfnEDMACallback[1][2])(u32WraparoundStatus); 		    					    			   			
	    			}
				}	    			
    		}    	
    	}
    	else if (inp32(REG_VDMA_ISR) & INTR2)
    	{
    		if (inp32(REG_PDMA_IER2) & WAR_IE)
	    		u32IntStatus = inp32(REG_PDMA_ISR2) & (inp32(REG_PDMA_IER2) | 0x0F00);
	    	else
	    		u32IntStatus = inp32(REG_PDMA_ISR2) & inp32(REG_PDMA_IER2);	    	
	    	
    		if (u32IntStatus & EDMATABORT_IF)
    		{
    			outp32(REG_PDMA_ISR2,EDMATABORT_IF);
    			if (g_pfnEDMACallback[2][0] != 0)    		
	    			(*g_pfnEDMACallback[2][0])(0);    			
			}    			
    		else 
    		{
    			if (u32IntStatus & EDMABLKD_IF)
    			{
    				outp32(REG_PDMA_ISR2,EDMABLKD_IF);
	    			if (g_pfnEDMACallback[2][1] != 0)    			
		    			(*g_pfnEDMACallback[2][1])(0);	    				    			
				}	
	    		else if (u32IntStatus & EDMASG_IF)
	    		{
	    			outp32(REG_PDMA_ISR2,EDMASG_IF);
	    			if (g_pfnEDMACallback[2][3] != 0)
		    			(*g_pfnEDMACallback[2][3])(0);	    				    		
	    		}					    			
    			else
    			{
	    			u32WraparoundStatus = inp32(REG_PDMA_ISR2) & 0x0F00;
	    			if (u32WraparoundStatus)
	    			{
		    			if (u32WraparoundStatus & 0x0200)
		    				u32WraparoundStatus = 0x0200;
		    			else if (u32WraparoundStatus & 0x0400)
		    				u32WraparoundStatus = 0x0400;
		    			else  if (u32WraparoundStatus & 0x0800)
		    				u32WraparoundStatus = 0x0800;
		    			else		   
		    				u32WraparoundStatus = 0x0100; 	

					outp32(REG_PDMA_ISR2,u32WraparoundStatus);
		    			if (g_pfnEDMACallback[2][2] != 0)		    						
			    			(*g_pfnEDMACallback[2][2])(u32WraparoundStatus); 		    					    			   			
	    			}
				}	    			
    		}     	
    	}
    	else if (inp32(REG_VDMA_ISR) & INTR3)
    	{
    		if (inp32(REG_PDMA_IER3) & WAR_IE)
	    		u32IntStatus = inp32(REG_PDMA_ISR3) & (inp32(REG_PDMA_IER3) | 0x0F00);
	    	else
	    		u32IntStatus = inp32(REG_PDMA_ISR3) & inp32(REG_PDMA_IER3);	    	
	    	
    		if (u32IntStatus & EDMATABORT_IF)
    		{
    			outp32(REG_PDMA_ISR3,EDMATABORT_IF);
    			if (g_pfnEDMACallback[3][0] != 0)    		
	    			(*g_pfnEDMACallback[3][0])(0);    			
			}    			
    		else 
    		{
    			if (u32IntStatus & EDMABLKD_IF)
    			{
    				outp32(REG_PDMA_ISR3,EDMABLKD_IF);
	    			if (g_pfnEDMACallback[3][1] != 0)    			
		    			(*g_pfnEDMACallback[3][1])(0);	    				    			
				}	    
	    		else if (u32IntStatus & EDMASG_IF)
	    		{
	    			outp32(REG_PDMA_ISR3,EDMASG_IF);
	    			if (g_pfnEDMACallback[3][3] != 0)
		    			(*g_pfnEDMACallback[3][3])(0);	    				    		
	    		}						
    			else
    			{
	    			u32WraparoundStatus = inp32(REG_PDMA_ISR3) & 0x0F00;
	    			if (u32WraparoundStatus)
	    			{
		    			if (u32WraparoundStatus & 0x0200)
		    				u32WraparoundStatus = 0x0200;
		    			else if (u32WraparoundStatus & 0x0400)
		    				u32WraparoundStatus = 0x0400;
		    			else  if (u32WraparoundStatus & 0x0800)
		    				u32WraparoundStatus = 0x0800;
		    			else		   
		    				u32WraparoundStatus = 0x0100; 	

					outp32(REG_PDMA_ISR3,u32WraparoundStatus);
		    			if (g_pfnEDMACallback[3][2] != 0)		    						
			    			(*g_pfnEDMACallback[3][2])(u32WraparoundStatus); 		    					    			   			
	    			}
				}	    			
    		}     	
    	}
    	else
    	{
    		if (inp32(REG_PDMA_IER4) & WAR_IE)
	    		u32IntStatus = inp32(REG_PDMA_ISR4) & (inp32(REG_PDMA_IER4) | 0x0F00);
	    	else
	    		u32IntStatus = inp32(REG_PDMA_ISR4) & inp32(REG_PDMA_IER4);	    	
	    	
    		if (u32IntStatus & EDMATABORT_IF)
    		{
    			outp32(REG_PDMA_ISR4,EDMATABORT_IF);
    			if (g_pfnEDMACallback[4][0] != 0)    		
	    			(*g_pfnEDMACallback[4][0])(0);    			
			}    			
    		else 
    		{
    			if (u32IntStatus & EDMABLKD_IF)
    			{
    				outp32(REG_PDMA_ISR4,EDMABLKD_IF);
	    			if (g_pfnEDMACallback[4][1] != 0)    			
		    			(*g_pfnEDMACallback[4][1])(0);	    				    			
				}	
	    		else if (u32IntStatus & EDMASG_IF)
	    		{
	    			outp32(REG_PDMA_ISR4,EDMASG_IF);
	    			if (g_pfnEDMACallback[4][3] != 0)
		    			(*g_pfnEDMACallback[4][3])(0);	    				    		
	    		}						
    			else
    			{
	    			u32WraparoundStatus = inp32(REG_PDMA_ISR4) & 0x0F00;
	    			if (u32WraparoundStatus)
	    			{
		    			if (u32WraparoundStatus & 0x0200)
		    				u32WraparoundStatus = 0x0200;
		    			else if (u32WraparoundStatus & 0x0400)
		    				u32WraparoundStatus = 0x0400;
		    			else  if (u32WraparoundStatus & 0x0800)
		    				u32WraparoundStatus = 0x0800;
		    			else		   
		    				u32WraparoundStatus = 0x0100; 	

					outp32(REG_PDMA_ISR4,u32WraparoundStatus);				
		    			if (g_pfnEDMACallback[4][2] != 0)		    						
			    			(*g_pfnEDMACallback[4][2])(u32WraparoundStatus); 		    					    			   			
	    			}
				}	    			
    		}     	
    	}
	}
//	return IRQ_HANDLED;
}
//EXPORT_SYMBOL(DrvEDMA_ISR);

// Install Call Back Function for Channelx & Interrupt source
int  
DrvEDMA_InstallCallBack(
    E_DRVEDMA_CHANNEL_INDEX eChannel, 
    E_DRVEDMA_INT_ENABLE eIntSource,
	PFN_DRVEDMA_CALLBACK pfncallback,    
	PFN_DRVEDMA_CALLBACK *pfnOldcallback  	
)
{
    unsigned int u32SFR, index;
    
    u32SFR = (u32)REG_VDMA_IER + eChannel * 0x100;        
    
    if (eChannel > MAX_CHANNEL_NUM)
        return E_DRVEDMA_FALSE_INPUT;
            
    if ((eIntSource !=1) && (eIntSource !=2) && (eIntSource !=4) && (eIntSource !=8)) 
        return E_DRVEDMA_FALSE_INPUT;
	
	for (index=0; index<4; index++)
		if ((1<<index) == eIntSource)
			break;
	if (pfnOldcallback != NULL)
		*pfnOldcallback = g_pfnEDMACallback[eChannel][index];
	g_pfnEDMACallback[eChannel][index] = pfncallback;
#if 0
	DrvAIC_InstallISR(eDRVAIC_INT_LEVEL7 ,eDRVAIC_INT_EDMA, (PVOID)DrvEDMA_ISR, 0);
    DrvAIC_EnableInt(eDRVAIC_INT_EDMA);
    DrvAIC_SetIntType(eDRVAIC_INT_EDMA, eDRVAIC_HIGH_LEVEL);	
#endif
    
    return E_SUCCESS;      
}

// Get Information about Scatter Gather Descript Table size and Max Transfer Size per Descript Table  
void DrvEDMA_GetScatterGatherInfo(
	unsigned int *pu32TblSize,
	unsigned int *pu32MaxTransferBytePerTbl
)
{
	*pu32TblSize = sizeof(S_DRVEDMA_DESCRIPT_FORMAT);
	*pu32MaxTransferBytePerTbl = MAX_GS_TRANSFER_SIZE;
	
	return;
}

// Set Scatter Gather Descript Format Table Start Address
void DrvEDMA_SetScatterGatherTblStartAddr(
    E_DRVEDMA_CHANNEL_INDEX eChannel, 
	unsigned int	u32TblStartAddr
)
{
    unsigned int u32SFR;
    
    u32SFR = (u32)REG_VDMA_SGAR + eChannel * 0x100;	
    outp32(u32SFR, u32TblStartAddr);
}	

// Enable Scatter Gather Function  
void DrvEDMA_EnableScatterGather(
    E_DRVEDMA_CHANNEL_INDEX eChannel 
)
{
    unsigned int u32SFR;
    
    u32SFR = (u32)REG_VDMA_CSR + eChannel * 0x100;	
    outp32(u32SFR, inp32(u32SFR)| EDMASG_EN);
}

// Disable Scatter Gather Function
void DrvEDMA_DisableScatterGather(
    E_DRVEDMA_CHANNEL_INDEX eChannel 
)
{
    unsigned int u32SFR;
    
    u32SFR = (u32)REG_VDMA_CSR + eChannel * 0x100;	
    outp32(u32SFR, inp32(u32SFR) & ~EDMASG_EN);
}

// Set Descript Format for Scatter Gather
// This function only support Order =0 
int 
DrvEDMA_SetScatterGatherSetting(
    E_DRVEDMA_CHANNEL_INDEX eChannel,
	unsigned int u32SGTblStartAddr,
	S_DRVEDMA_DESCRIPT_SETTING* psDescript
)
{
    unsigned int u32SFR, u32Value;     
	S_DRVEDMA_DESCRIPT_FORMAT *psSGFmt;
	unsigned int u32TranferByte=0;
	unsigned int u32TotalByteCount;
	unsigned int u32NewSrcAddr, u32NewDestAddr, u32AddSrcAddr, u32AddDestAddr;	
	E_DRVEDMA_COLOR_FORMAT eSrcFormat, eDestFormat;
	unsigned int u32MaxTxfBytePerTbl;	
	
	if ((int)psDescript->u32TransferByteCount <=0)
        return E_DRVEDMA_FALSE_INPUT;
        
    eSrcFormat = (inp32(REG_VDMA_CTCSR) & SOUR_FORMAT) >> 24;
    eDestFormat = (inp32(REG_VDMA_CTCSR) & DEST_FORMAT) >> 16;        	
        
    u32SFR = (u32)REG_VDMA_CSR + eChannel * 0x100;
    u32Value = inp32(u32SFR);         
    u32Value = (u32Value & ~SAD_SEL) | (psDescript->eSrcDirection << SOURCE_DIRECTION_BIT);    
    u32Value = (u32Value & ~DAD_SEL) | (psDescript->eDestDirection << DESTINATION_DIRECTION_BIT);
    outp32(u32SFR,u32Value); 
            
	DrvEDMA_EnableScatterGather(eChannel);
	DrvEDMA_SetScatterGatherTblStartAddr(eChannel, u32SGTblStartAddr);
	psSGFmt = (S_DRVEDMA_DESCRIPT_FORMAT *)u32SGTblStartAddr;
	u32TotalByteCount = psDescript->u32TransferByteCount;
	u32NewSrcAddr = psDescript->u32SourceAddr;
	u32NewDestAddr = psDescript->u32DestAddr;	
	u32AddSrcAddr=0;
	u32AddDestAddr=0;	
	
	// Update TransferByteCount for stride mode
	if ((psDescript->u32Stride) && (inp32(REG_VDMA_CTCSR) & STRIDE_EN))
	    u32MaxTxfBytePerTbl = MAX_GS_TRANSFER_SIZE / psDescript->u32Stride * psDescript->u32Stride;	
	else    
	    u32MaxTxfBytePerTbl = MAX_GS_TRANSFER_SIZE;
	
    do {
		// Set Phsyical Source Address and Destination Address
		// Set WrapAround and Fixed Direction in Scatter Gather as Fixed Direction    
		if (psDescript->eSrcDirection == eDRVEDMA_DIRECTION_INCREMENTED)
		{
    	    psSGFmt->u32SourceAddr = u32NewSrcAddr + u32AddSrcAddr;
    	    u32NewSrcAddr += u32AddSrcAddr;
		} 
    	else 
    	{
    	    if (psDescript->eSrcDirection == eDRVEDMA_DIRECTION_DECREMENTED)
    	    {
    	        psSGFmt->u32SourceAddr	= u32NewSrcAddr - u32AddSrcAddr;
	    	    u32NewSrcAddr -= u32AddSrcAddr;    	        
    	    }  
    	    else    
	    	    psSGFmt->u32SourceAddr	= u32NewSrcAddr;    	        
    	}
    	
    	if (psDescript->eDestDirection == eDRVEDMA_DIRECTION_INCREMENTED)
    	{
    	    psSGFmt->u32DestAddr = u32NewDestAddr + u32AddDestAddr;
    	    u32NewDestAddr += u32AddDestAddr;
		} 
    	else
    	{
    	    if (psDescript->eDestDirection == eDRVEDMA_DIRECTION_DECREMENTED)
    	    {
    	        psSGFmt->u32DestAddr = u32NewDestAddr - u32AddDestAddr; 
    	         u32NewDestAddr -= u32AddDestAddr;   	    
			}     	    
			else    
				psSGFmt->u32DestAddr = u32NewDestAddr;    	        
    	}
    	
    	if (u32TotalByteCount >= u32MaxTxfBytePerTbl)
    	{
	    	u32TranferByte = u32MaxTxfBytePerTbl;
	    	u32TotalByteCount -= u32MaxTxfBytePerTbl;	    	
		}	    	
	    else
	    {
	    	u32TranferByte = u32TotalByteCount;	  
	    	u32TotalByteCount = 0;  	
		}	  
		
		// Src/Dest format must be set before DrvEDMA_SetScatterGatterSetting() function to calcuate the next address
		if ((psDescript->u32Stride) && (inp32(REG_VDMA_CTCSR) & STRIDE_EN) && (eChannel ==0))
		{   // Calculate the next Src/Destination for Stride mode
		    u32AddSrcAddr = (u32TranferByte / psDescript->u32Stride) * (psDescript->u32SrcOffset + psDescript->u32Stride) + 
		                    u32TranferByte % psDescript->u32Stride;

	    	if (eSrcFormat == eDRVEDMA_RGB888)
	    	{
	    		if (eDestFormat== eDRVEDMA_RGB888)
	    			u32AddDestAddr = (u32TranferByte / psDescript->u32Stride) * (psDescript->u32DestOffset + psDescript->u32Stride) + 
		                    u32TranferByte % psDescript->u32Stride;	
	    		else	
		    		u32AddDestAddr = (u32TranferByte / psDescript->u32Stride) * (psDescript->u32DestOffset + psDescript->u32Stride/2) + 
		                    u32TranferByte % psDescript->u32Stride/2;
	    	}
	    	else
	    	{
	    		if (eDestFormat== eDRVEDMA_RGB888)
	    			u32AddDestAddr = (u32TranferByte / psDescript->u32Stride) * (psDescript->u32DestOffset + psDescript->u32Stride*2) + 
		                    u32TranferByte % psDescript->u32Stride*2;
	    		else	
		    		u32AddDestAddr = (u32TranferByte / psDescript->u32Stride) * (psDescript->u32DestOffset + psDescript->u32Stride) + 
		                    u32TranferByte % psDescript->u32Stride;	    	
	    	}
              
		}
		else
		{   // Calculate the next Src/Destination for non-Stride mode
		    u32AddSrcAddr = u32TranferByte;
		    
		    if (eChannel ==0)
		    {
		    	if (eSrcFormat == eDRVEDMA_RGB888)
		    	{
		    		if (eDestFormat== eDRVEDMA_RGB888)
		    			u32AddDestAddr = u32TranferByte;
		    		else	
			    		u32AddDestAddr = u32TranferByte/2;
		    	}
		    	else
		    	{
		    		if (eDestFormat== eDRVEDMA_RGB888)
		    			u32AddDestAddr = u32TranferByte*2;
		    		else	
			    		u32AddDestAddr =u32TranferByte;    	
		    	}
	    	}
	    	else
	    	{
		    	u32AddDestAddr = u32TranferByte;
	    	}		    
		}		  
		
		// Set Stride Transfer Byte Count & Byte Count	
    	psSGFmt->u32StrideAndByteCount   = ((psDescript->u32Stride & 0x7FFF) << 17) | u32TranferByte;
    	
    	// Set Source Offset Byte Length and Destination Offset Byte Length
  	   	psSGFmt->u32Offset = ((psDescript->u32DestOffset & 0x7FFF) << 16) |
  	   						  (psDescript->u32SrcOffset & 0x7FFF);      	
  	   						  
		// Set EOT for last Descript Format  	   						  
		if (u32TotalByteCount == 0)
			psSGFmt->u32Offset |= 0x80000000;
			
		// Set Next Scatter Gether Table Address	
		psSGFmt->u32NextSGTblAddr = (unsigned int)(psSGFmt+1); 	
		
		psSGFmt++;		
  	   	
	} while(u32TotalByteCount > 0);    
	
	return E_SUCCESS;  
}

