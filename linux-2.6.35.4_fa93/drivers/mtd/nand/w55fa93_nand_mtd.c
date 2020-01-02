/*
 * Copyright © 2009 Nuvoton technology corporation.
 *
 * Wan ZongShun <mcuos.com@gmail.com>
 * ZM.Song <zmsong001@gmail.com> modified - 2011.01.07
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#if defined(CONFIG_NUVOTON_W55FA93_SM)
    #error "Sorry, the driver is conflict with CONFIG_NUVOTON_W55FA93_SM."
#endif

#if defined(CONFIG_MTD_NAND_VERIFY_WRITE)
    #error "Sorry, the driver doesn't support VERIFY"
#endif


#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/blkdev.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/w55fa93_reg.h>
#include <linux/dma-mapping.h>

#define RESET_FMI   0x01
#define NAND_EN     0x08
#define READYBUSY   (0x01 << 18)

#define SWRST       0x01
#define PSIZE       (0x01 << 3)
#define DMARWEN     (0x03 << 1)
#define BUSWID      (0x01 << 4)
#define ECC4EN      (0x01 << 5)
#define WP          (0x01 << 24)
#define NANDCS      (0x01 << 25)
#define ENDADDR     (0x01 << 31)

#define Enable_IRQ(n)   writel( 1<<(n), REG_AIC_MECR)
#define Disable_IRQ(n)  writel( 1<<(n), REG_AIC_MDCR)


#define W55FA93_DRV_VERSION "20191118"
#define DEF_RESERVER_OOB_SIZE_FOR_MARKER 4
#define DMAC_TRANSFER

// RESERVED_FOR_FS is configured for reserved oob size of filesystem.
// Bad block marker (Fixed 4Bytes) + RESERVED_FOR_FS (User-defied) + Parity code (Size by algorithm)
// Default is 0 Bytes for in-band tag file system.
#define RESERVED_FOR_FS     0

//#define W55FA93_NAND_DEBUG
#ifndef W55FA93_NAND_DEBUG
    #define DBG(fmt, arg...)
    #define ENTER()
    #define LEAVE()
#else
    #define DBG(fmt, arg...)    printk(fmt, ##arg)
    #define ENTER() u32 U32_MTDNAND_START=0; u32 U32_MTDNAND_STOP=0; DBG("[%s, %d] : %d\n", __FUNCTION__, __LINE__, __raw_readl(REG_TDR1)); U32_MTDNAND_START=(__raw_readl(REG_TDR1));
    #define LEAVE() U32_MTDNAND_STOP=(__raw_readl(REG_TDR1)); DBG("[%s, %d] : Leave(%d)\n", __FUNCTION__, __LINE__, (U32_MTDNAND_STOP-U32_MTDNAND_START)/(CLOCK_TICK_RATE/1000));
#endif

extern struct semaphore fmi_sem;
extern struct semaphore dmac_sem;

#define read_data_reg(dev)          \
    readl(REG_SMDATA)

#define write_data_reg(dev, val)    \
    writel((val), REG_SMDATA)

#define write_cmd_reg(dev, val)     \
    writel((val), REG_SMCMD)

#define write_addr_reg(dev, val)    \
    writel((val), REG_SMADDR)


struct w55fa93_nand_ecclayout {
    struct nand_ecclayout   m_ecclayout;
    int                     m_BCHAlgoidx;
    int                     m_SMRASize;
};

static struct w55fa93_nand_ecclayout w55fa93_nand_SYSTEM_oob;
static struct w55fa93_nand_ecclayout w55fa93_nand_EXECUTE_oob;

static struct mtd_partition partitions[] = {
/*****************************************/
/* Notice: Don't modify this section     */
/* Keep SYSTEM partition in first    */
/* The size will auto-produce        */
/*****************************************/
    {
    .name = "SYSTEM",
    .offset = 0,
    .ecclayout = (struct nand_ecclayout*)&w55fa93_nand_SYSTEM_oob
    },
    {
    .name = "EXECUTE",
    .size = 16 * 1024 * 1024,
    .offset = MTDPART_OFS_APPEND,
    .ecclayout = (struct nand_ecclayout*)&w55fa93_nand_EXECUTE_oob
    },
/******************************************/
    {
    .name = "NAND1-1",
    .offset = MTDPART_OFS_APPEND,
    .size = 32 * 1024 * 1024
    },
    {
    .name = "NAND1-2",
    .offset = MTDPART_OFS_APPEND,
    .size = MTDPART_SIZ_FULL
    }
};

struct w55fa93_nand_info {
    struct nand_hw_control  controller;
    struct mtd_info         mtd;
    struct nand_chip        chip;
    struct mtd_partition    *parts;     // mtd partition
    int                     nr_parts;   // mtd partition number
    struct platform_device  *pdev;
    struct clk              *clk;
    struct clk              *sic_clk;

    void __iomem            *reg;
    int                     eBCHAlgo;
    int                     m_i32SMRASize;
    int                     m_ePageSize;

    unsigned char           *pnand_vaddr;
    unsigned char           *pnand_phyaddr;

    int                     m_i32MyShowTime;
    spinlock_t              lock;
};

typedef enum  {
    eBCH_T4,
    eBCH_T8,
    eBCH_T12,
    eBCH_T15,
    eBCH_CNT
} E_BCHALGORITHM;

typedef enum {
    ePageSize_512,
    ePageSize_2048,
    ePageSize_4096,
    ePageSize_8192,
    ePageSize_CNT
} E_PAGESIZE;

static const int g_i32BCHAlgoIdx[eBCH_CNT] = { BCH_T4, BCH_T8, BCH_T12, BCH_T15 };
static const char *g_pcBCHAlgoIdx[eBCH_CNT] = { "BCH_T4", "BCH_T8", "BCH_T12", "BCH_T15" };
static const int g_SYSAREA_NAND_EXTRA_SIZE[ePageSize_CNT] = { 16, 64, 128, 376 };

static void dump_chip_info( struct nand_chip * chip )
{
#ifndef W55FA93_NAND_DEBUG
    return;
#endif

    printk( "===== Nand Chip Info =====================\n" );

    printk("chip->chip_delay: %d\n",    chip->chip_delay );
    printk("chip->options: 0x%08X\n",   chip->options );

    printk("chip->page_shift (page size): %d Bytes\n",          1<<chip->page_shift );
    printk("chip->phys_erase_shift (block size): %d Bytes\n",   1<<chip->phys_erase_shift );

    printk("chip->bbt_erase_shift: %d\n",   1<<chip->bbt_erase_shift );

    printk("chip->chip_shift: 0x%08X\n",    chip->chip_shift );
    printk("chip->numchips: %d\n",          chip->numchips );

    printk("chip->subpagesize: %d\n",       1<<chip->subpagesize );
    printk("chip->cellinfo: 0x%08X\n",      chip->cellinfo );

    printk( "==========================================\n" );
}


static void dump_regs(int i32Line)
{
#ifndef W55FA93_NAND_DEBUG
    return;
#endif

    printk("============[%d]==============\n", i32Line);

    printk("REG_FMICR[00] : 0x%08X\n",  readl(REG_FMICR));

    printk("REG_SMCSR[A0] : 0x%08X\n",  readl(REG_SMCSR));
    printk("REG_SMISR[AC] : 0x%08X\n",  readl(REG_SMISR));
    printk("REG_SMIER[A8] : 0x%08X\n",  readl(REG_SMIER));

    printk("REG_SMCMD[B0] : 0x%08X\n",  readl(REG_SMCMD));
    printk("REG_SMADDR[B4] : 0x%08X\n", readl(REG_SMADDR));
    printk("REG_SMDATA[B8] : 0x%08X\n", readl(REG_SMDATA));

    printk("REG_SMTCR[A4] : 0x%08X\n",  readl(REG_SMTCR));

    printk("REG_DMACSAR[08] : 0x%08X\n",    readl(REG_DMACSAR));

    printk("============[%d]==============\n", i32Line);
}


/*
 * w55fa93_nand_hwecc_init - Initialize hardware ECC IP
 */
static void w55fa93_nand_hwecc_init (struct mtd_info *mtd)
{
    struct w55fa93_nand_info *nand = container_of(mtd, struct w55fa93_nand_info, mtd);

    writel( readl(REG_SMCSR)|SMCR_SM_SWRST, REG_SMCSR);    // reset SM controller

    // Mask ECC During Write Page Data. Do not mask the ECC parity for each field.
    //writel( readl(REG_SMREAREA_CTL) & ~SMRE_MECC, REG_SMREAREA_CTL);

    // Redundant area size
    //writel( (readl(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | mtd->oobsize , REG_SMREAREA_CTL );
    writel( nand->m_i32SMRASize, REG_SMREAREA_CTL);

    // Protect redundant 3 bytes
    // because we need to implement write_oob function to partial data to oob available area.
    // Please note we skip 4 bytes
    writel( readl(REG_SMCSR) | SMCR_ECC_3B_PROTECT, REG_SMCSR);

    // To read/write the ECC parity codes automatically from/to NAND Flash after data area field written.
    writel( readl(REG_SMCSR) | SMCR_REDUN_AUTO_WEN, REG_SMCSR);

    if ( nand->eBCHAlgo >= 0 ) {
        // Set BCH algorithm
        writel( (readl(REG_SMCSR) & (~SMCR_BCH_TSEL)) | g_i32BCHAlgoIdx[nand->eBCHAlgo], REG_SMCSR);

        // Enable H/W ECC, ECC parity check enable bit during read page
        writel( readl(REG_SMCSR) | SMCR_ECC_EN | SMCR_ECC_CHK, REG_SMCSR);

    } else {
        // Disable H/W ECC / ECC parity check enable bit during read page
        writel( readl(REG_SMCSR) & (~SMCR_ECC_EN) &(~SMCR_ECC_CHK), REG_SMCSR);
    }
}

/*
 * w55fa93_nand_hwecc_fini - Finalize hardware ECC IP
 */
static void w55fa93_nand_hwecc_fini (struct mtd_info *mtd)
{
    struct nand_chip *chip = mtd->priv;
    if ( chip->ecc.mode == NAND_ECC_HW_OOB_FIRST )
        writel(readl(REG_SMCSR)&(~SMCR_ECC_EN), REG_SMCSR); // ECC disable
}


static void w55fa93_nand_initialize ( void )
{
    ENTER();

    //--- initial FA93 NAND port 0
    writel(readl(REG_GPDFUN) | 0x0003CC00, REG_GPDFUN); // enable NAND NWR/NRD/RB0 pins
    writel(readl(REG_GPEFUN) | 0x00F30000, REG_GPEFUN); // enable NAND ALE/CLE/CS0 pins
    writel(readl(REG_SMCSR) |  SMCR_CS1, REG_SMCSR);    // disable NAND port 1
    writel(readl(REG_SMCSR) & ~SMCR_CS0, REG_SMCSR);    // enable NAND port 0

    // Enable SM_EN
    writel(FMI_SM_EN, REG_FMICR);

    // Timing control
    //writel(0x3050b, REG_SMTCR);
    // tCLS= (2+1)TAHB,
    // tCLH= (2*2+2)TAHB,
    // tALS= (2*1+1)TAHB,
    // tALH= (2*2+2)TAHB,
    writel(0x20305, REG_SMTCR);

    // NAND Reset
    writel(readl(REG_SMCSR) | SMCR_SM_SWRST, REG_SMCSR);    // software reset

    LEAVE();
}


/*
 * w55fa93_wait_sem - Wait a semaphore
 */
static int w55fa93_wait_sem (struct mtd_info *mtd)
{
    struct w55fa93_nand_info *nand = container_of(mtd, struct w55fa93_nand_info, mtd);
#if defined(CONFIG_MMC_W55FA93_SD)
    if ( !nand->m_i32MyShowTime )
        if ( down_interruptible(&fmi_sem) )
            return -1;
#else
    if ( nand->m_i32MyShowTime )
        return 0;
#endif

    if ( readl(REG_FMICR) != FMI_SM_EN )
        writel(FMI_SM_EN, REG_FMICR);

    writel(readl(REG_GPDFUN) | 0x0003CC00, REG_GPDFUN); // enable NAND NWR/NRD/RB0 pins
    writel(readl(REG_GPEFUN) | 0x00F30000, REG_GPEFUN); // enable NAND ALE/CLE/CS0 pins
    writel(readl(REG_SMCSR) |  SMCR_CS1, REG_SMCSR);    // disable NAND port 1
    writel(readl(REG_SMCSR) & ~SMCR_CS0, REG_SMCSR);    // enable NAND port 0

    nand->m_i32MyShowTime++;

    return 0;
}

/*
 * w55fa93_release_sem - Release a semaphore
 */
static void w55fa93_release_sem (struct mtd_info *mtd)
{
    struct w55fa93_nand_info *nand = container_of(mtd, struct w55fa93_nand_info, mtd);
#if defined(CONFIG_MMC_W55FA93_SD)
    nand->m_i32MyShowTime--;
    if ( !nand->m_i32MyShowTime )
        up(&fmi_sem);
#else
    nand->m_i32MyShowTime--;
#endif
}

/*-----------------------------------------------------------------------------
 * Define some constants for BCH
 *---------------------------------------------------------------------------*/
// define the total padding bytes for 512/1024 data segment
#define BCH_PADDING_LEN_512     32
#define BCH_PADDING_LEN_1024    64
// define the BCH parity code lenght for 512 bytes data pattern
#define BCH_PARITY_LEN_T4       8
#define BCH_PARITY_LEN_T8       15
#define BCH_PARITY_LEN_T12      23
#define BCH_PARITY_LEN_T15      29

static struct nand_ecclayout w55fa93_nand_oob;

static const int g_i32ParityNum[ePageSize_CNT][eBCH_CNT] = {
    { 8,    15,     23,     29  },  // For 512
    { 32,   60,     92,     116 },  // For 2K
    { 64,   120,    184,    232 },  // For 4K
    { 128,  240,    368,    464 },  // For 8K
};


static int w55fa93_hwecc_choice_bch_algorithm ( E_PAGESIZE ePagesize, int oob_size, int oob_available )
{
    int i;
    /* Rule: select a BCH algorithm as the oob_size/2 large than parity number */
    for ( i=eBCH_T15; i>=0; i-- )
        if ( g_i32ParityNum[ePagesize][i] > 0 )
            if ( g_i32ParityNum[ePagesize][i] <= (oob_size-oob_available-DEF_RESERVER_OOB_SIZE_FOR_MARKER) )    // use half ecc size
            break;
    return i;
}

static void w55fa93_choice_bch_algo ( struct mtd_info *mtd, int page )
{
    int i;
    struct w55fa93_nand_info *nand = container_of(mtd, struct w55fa93_nand_info, mtd);

    struct mtd_partition *part;
    struct nand_ecclayout *part_ecclayout;

    int BCHAlgoIdx=0;
    int SMRASize=0;

    u32 page_addr = page * mtd->writesize;

    for ( i=0; i<nand->nr_parts; i++ )
    {
        part = &nand->parts[i];
        part_ecclayout = part->ecclayout;
        if ( part_ecclayout && part_ecclayout != mtd->ecclayout ) {
            if ( part->offset <= page_addr && page_addr < part->offset+part->size )
            {
                struct w55fa93_nand_ecclayout * pSW55FA93NandEccLayout=(struct w55fa93_nand_ecclayout *)part_ecclayout;
                BCHAlgoIdx = pSW55FA93NandEccLayout->m_BCHAlgoidx;
                SMRASize = pSW55FA93NandEccLayout->m_SMRASize;
                break;
            }
        }
    }
    if ( i == nand->nr_parts )
    {
        BCHAlgoIdx  = w55fa93_hwecc_choice_bch_algorithm ( nand->m_ePageSize, mtd->oobsize, mtd->oobavail );
        SMRASize    = mtd->oobsize;
    }

    if ( BCHAlgoIdx != nand->eBCHAlgo )
    {
        if ( BCHAlgoIdx >= 0 )
            printk("(0x%08X)BCH change to %s: %dB\n",  page_addr, g_pcBCHAlgoIdx[BCHAlgoIdx],  g_i32ParityNum[nand->m_ePageSize][BCHAlgoIdx] );
        else
            printk("(0x%08X)BCH change to No-ECC\n",  page_addr );
    }
    nand->eBCHAlgo = BCHAlgoIdx;

    if ( SMRASize != nand->m_i32SMRASize )
        printk("(0x%08X)SMRA size change to %dB\n",  page_addr, SMRASize );
    nand->m_i32SMRASize = SMRASize;
}

/*-----------------------------------------------------------------------------
 * Correct data by BCH alrogithm.
 *      Support 8K page size NAND and BCH T4/8/12/15.
 *---------------------------------------------------------------------------*/
void fmiSM_CorrectData_BCH(u8 ucFieidIndex, u8 ucErrorCnt, u8* pDAddr)
{
    u32 uaData[24], uaAddr[24];
    u32 uaErrorData[4];
    u8  ii, jj;
    u32 uPageSize;
    u32 field_len, padding_len, parity_len;
    u32 total_field_num;
    u8  *smra_index;

    ENTER();

    //--- assign some parameters for different BCH and page size
    switch (readl(REG_SMCSR) & SMCR_BCH_TSEL)
    {
        case BCH_T15:
            field_len   = 512;
            padding_len = BCH_PADDING_LEN_512;
            parity_len  = BCH_PARITY_LEN_T15;
            break;
        case BCH_T12:
            field_len   = 512;
            padding_len = BCH_PADDING_LEN_512;
            parity_len  = BCH_PARITY_LEN_T12;
            break;
        case BCH_T8:
            field_len   = 512;
            padding_len = BCH_PADDING_LEN_512;
            parity_len  = BCH_PARITY_LEN_T8;
            break;
        case BCH_T4:
            field_len   = 512;
            padding_len = BCH_PADDING_LEN_512;
            parity_len  = BCH_PARITY_LEN_T4;
            break;
        default:
            printk("NAND ERROR: %s(): invalid SMCR_BCH_TSEL = 0x%08X\n", __FUNCTION__, (u32)(readl(REG_SMCSR) & SMCR_BCH_TSEL));
            LEAVE();
            return;
    }

    uPageSize = readl(REG_SMCSR) & SMCR_PSIZE;
    switch (uPageSize)
    {
        case PSIZE_8K:  total_field_num = 8192 / field_len; break;
        case PSIZE_4K:  total_field_num = 4096 / field_len; break;
        case PSIZE_2K:  total_field_num = 2048 / field_len; break;
        case PSIZE_512: total_field_num =  512 / field_len; break;
        default:
            printk("NAND ERROR: %s(): invalid SMCR_PSIZE = 0x%08X\n", __FUNCTION__, uPageSize);
            LEAVE();
            return;
    }

    //--- got valid BCH_ECC_DATAx and parse them to uaData[]
    // got the valid register number of BCH_ECC_DATAx since one register include 4 error bytes
    jj = ucErrorCnt/4;
    jj ++;
    if (jj > 4)
        jj = 4;     // there are 4 BCH_ECC_DATAx registers to support BCH T15

    for(ii=0; ii<jj; ii++)
    {
        uaErrorData[ii] = readl(REG_BCH_ECC_DATA0 + ii*4);
    }

    for(ii=0; ii<jj; ii++)
    {
        uaData[ii*4+0] = uaErrorData[ii] & 0xff;
        uaData[ii*4+1] = (uaErrorData[ii]>>8) & 0xff;
        uaData[ii*4+2] = (uaErrorData[ii]>>16) & 0xff;
        uaData[ii*4+3] = (uaErrorData[ii]>>24) & 0xff;
    }

    //--- got valid REG_BCH_ECC_ADDRx and parse them to uaAddr[]
    // got the valid register number of REG_BCH_ECC_ADDRx since one register include 2 error addresses
    jj = ucErrorCnt/2;
    jj ++;
    if (jj > 8)
        jj = 8;    // there are 8 REG_BCH_ECC_ADDRx registers to support BCH T15

    for(ii=0; ii<jj; ii++)
    {
        uaAddr[ii*2+0] = readl(REG_BCH_ECC_ADDR0 + ii*4) & 0x07ff;   // 11 bits for error address
        uaAddr[ii*2+1] = (readl(REG_BCH_ECC_ADDR0 + ii*4)>>16) & 0x07ff;
    }

    //--- pointer to begin address of field that with data error
    pDAddr += (ucFieidIndex-1) * field_len;

    //--- correct each error bytes
    for(ii=0; ii<ucErrorCnt; ii++)
    {
        // for wrong data in field
        if (uaAddr[ii] < field_len)
        {
#ifdef W55FA93_NAND_DEBUG
            printk("BCH error corrected for data: address 0x%08X, data [0x%02X] --> ",
                (unsigned int)(pDAddr+uaAddr[ii]), (unsigned int)(*(pDAddr+uaAddr[ii])));
#endif

            *(pDAddr+uaAddr[ii]) ^= uaData[ii];

#ifdef W55FA93_NAND_DEBUG
            printk("[0x%02X]\n", *(pDAddr+uaAddr[ii]));
#endif
        }
        // for wrong first-3-bytes in redundancy area
        else if (uaAddr[ii] < (field_len+3))
        {
            uaAddr[ii] -= field_len;
            uaAddr[ii] += (parity_len*(ucFieidIndex-1));    // field offset

#ifdef W55FA93_NAND_DEBUG
            printk("BCH error corrected for 3 bytes: address 0x%08X, data [0x%02X] --> ",
                (unsigned int)((u8 *)REG_SMRA_0+uaAddr[ii]), (unsigned int)(*((u8 *)REG_SMRA_0+uaAddr[ii])));
#endif
            *((u8 *)REG_SMRA_0+uaAddr[ii]) ^= uaData[ii];

#ifdef W55FA93_NAND_DEBUG
            printk("[0x%02X]\n", *((u8 *)REG_SMRA_0+uaAddr[ii]));
#endif
        }
        // for wrong parity code in redundancy area
        else
        {
            // BCH_ERR_ADDRx = [data in field] + [3 bytes] + [xx] + [parity code]
            //                                   |<--     padding bytes      -->|
            // The BCH_ERR_ADDRx for last parity code always = field size + padding size.
            // So, the first parity code = field size + padding size - parity code length.
            // For example, for BCH T12, the first parity code = 512 + 32 - 23 = 521.
            // That is, error byte address offset within field is
            uaAddr[ii] = uaAddr[ii] - (field_len + padding_len - parity_len);

            // smra_index point to the first parity code of first field in register SMRA0~n
            smra_index = (u8 *)
                         (REG_SMRA_0 + (readl(REG_SMREAREA_CTL) & SMRE_REA128_EXT) - // bottom of all parity code -
                           (parity_len * total_field_num)                            // byte count of all parity code
                         );

            // final address = first parity code of first field +
            //                 offset of fields +
            //                 offset within field

#ifdef W55FA93_NAND_DEBUG
            printk("BCH error corrected for parity: address 0x%08X, data [0x%02X] --> ",
                (unsigned int)(smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii]),
                (unsigned int)(*(smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii])));
#endif

            *((u8 *)smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii]) ^= uaData[ii];

#ifdef W55FA93_NAND_DEBUG
            printk("[0x%02X]\n",
                *((u8 *)smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii]));
#endif

        }
    }   // end of for (ii<ucErrorCnt)

    LEAVE();
}

int fmiSMCorrectData (struct mtd_info *mtd, unsigned long uDAddr )
{
    int uStatus, ii, jj, i32FieldNum=0;
    volatile int uErrorCnt = 0;

    ENTER();

    if ( readl ( REG_SMISR ) & SMISR_ECC_FIELD_IF )
    {
        i32FieldNum = mtd->writesize / 512;

        if ( i32FieldNum < 4 )
            i32FieldNum  = 1;
        else
            i32FieldNum /= 4;

        for ( jj=0; jj<i32FieldNum; jj++ )
        {
            uStatus = readl ( REG_SM_ECC_ST0+jj*4 );
            if ( !uStatus )
                continue;

            for ( ii=1; ii<5; ii++ )
            {
                if ( !(uStatus & 0x03) ) { // No error

                    uStatus >>= 8;
                    continue;

                } else if ( (uStatus & 0x03)==0x01 ) { // Correctable error

                    uErrorCnt = (uStatus >> 2) & 0x1F;
#ifdef W55FA93_NAND_DEBUG
                    printk("Field (%d, %d) have %d error!!\n", jj, ii, uErrorCnt);
#endif
                    fmiSM_CorrectData_BCH(jj*4+ii, uErrorCnt, (char*)uDAddr);

                    break;
                } else // uncorrectable error or ECC error
                {
#ifdef W55FA93_NAND_DEBUG
                    printk("SM uncorrectable error is encountered, %4x !!\n", uStatus);
#endif
                    LEAVE();
                    return -1;
                }
                uStatus >>= 8;
            } // end of for loop (ii)
        } //jj
    }

    LEAVE();
    return uErrorCnt;
}


/*
 * HW ECC Correction
 * function called after a read
 * mtd:        MTD block structure
 * dat:        raw data read from the chip
 * read_ecc:   ECC from the chip (unused)
 * isnull:     unused
 */
static int w55fa93_nand_correct_data(struct mtd_info *mtd, u_char *dat,
                     u_char *read_ecc, u_char *calc_ecc)
{
    return 0;
}


/*
 * Enable HW ECC : unused on most chips
 */
void w55fa93_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
    ENTER();
#ifdef W55FA93_NAND_DEBUG
    {
        char * ptr=REG_SMRA_0;
        int i=0;
        if( mode == NAND_ECC_READ )
            printk("[R]=\n");
        else
            printk("[W]=\n");

        for(i=0; i<mtd->oobsize; i++)
        {
            printk("%X ",  *(ptr+i) );
            if ( i % 32 == 31)
                printk("\n");
        }
        printk("\n");
    }
#endif
    LEAVE();
}


/*
 * w55fa93_nand_dmac_init - Initialize dma controller
 */
static void w55fa93_nand_dmac_init( void )
{
    // DMAC enable
    writel( readl(REG_DMACCSR) | DMAC_EN,       REG_DMACCSR);
    writel( readl(REG_DMACCSR) | DMAC_SWRST,    REG_DMACCSR);
    writel( readl(REG_DMACCSR) & (~DMAC_SWRST), REG_DMACCSR);

    // Clear DMA finished flag
    writel( readl(REG_SMISR) | SMISR_DMA_IF,    REG_SMISR);

    // Disable Interrupt
    writel(readl(REG_SMIER) & ~(SMIER_DMA_IE),  REG_SMIER);
}

/*
 * w55fa93_nand_dmac_fini - Finalize dma controller
 */
static void w55fa93_nand_dmac_fini( void)
{
    // Clear DMA finished flag
    writel(readl(REG_SMISR) | SMISR_DMA_IF, REG_SMISR);

    // Disable Interrupt
    // writel(readl(REG_SMIER) & ~(SMIER_DMA_IE),   REG_SMIER);

    // DMAC disable
    // reflect mmc driver writel(readl(REG_DMACCSR) &(~DMAC_EN),        REG_DMACCSR);
}

/*
 * w55fa93_nand_read_byte - read a byte from NAND controller into buffer
 * @mtd: MTD device structure
 */
static unsigned char w55fa93_nand_read_byte(struct mtd_info *mtd)
{
    unsigned char ret;
    struct w55fa93_nand_info *nand;

    ENTER();

    if ( w55fa93_wait_sem(mtd) < 0 )
        return -1;

    nand = container_of(mtd, struct w55fa93_nand_info, mtd);

    ret = (unsigned char)read_data_reg(nand);

    w55fa93_release_sem(mtd);

    LEAVE();

    return ret;
}


/*
 * w55fa93_nand_read_buf - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: virtual address in RAM of source
 * @len: number of data bytes to be transferred
 */
static void w55fa93_nand_read_buf(struct mtd_info *mtd,
                 unsigned char *buf, int len)
{
    int i;
    struct w55fa93_nand_info *nand;

    ENTER();

    nand = container_of(mtd, struct w55fa93_nand_info, mtd);

    if ( w55fa93_wait_sem(mtd) < 0 )
        return;

    for (i = 0; i < len; i++)
        buf[i] = (unsigned char)read_data_reg(nand);

    w55fa93_release_sem(mtd);

    LEAVE();
}

/*
 * w55fa93_nand_write_buf - write data from buffer into NAND controller
 * @mtd: MTD device structure
 * @buf: virtual address in RAM of source
 * @len: number of data bytes to be transferred
 */
static void w55fa93_nand_write_buf(struct mtd_info *mtd,
                  const unsigned char *buf, int len)
{
    int i;
    struct w55fa93_nand_info *nand;

    ENTER();

    nand = container_of(mtd, struct w55fa93_nand_info, mtd);

    if ( w55fa93_wait_sem(mtd) < 0 )
        return;

    for (i = 0; i < len; i++)
        write_data_reg(nand, buf[i]);

    w55fa93_release_sem(mtd);

    LEAVE();
}


/*
 * w55fa93_verify_buf: verify data between buf and chip.
 * @mtd: MTD device structure
 * @buf: virtual address in RAM of source
 * @len: number of data bytes to be transferred
 */
static int w55fa93_verify_buf(struct mtd_info *mtd,
                 const unsigned char *buf, int len)
{
    int i;
    struct w55fa93_nand_info *nand;

    ENTER();

    nand = container_of(mtd, struct w55fa93_nand_info, mtd);

    if ( w55fa93_wait_sem(mtd) < 0 )
        return -1;

    for (i = 0; i < len; i++) {
        if (buf[i] != (unsigned char)read_data_reg(nand))
        {
            w55fa93_release_sem(mtd);
            return -EFAULT;
        }
    }

    w55fa93_release_sem(mtd);

    LEAVE();

    return 0;
}


/*
 * _w55fa93_nand_dma_transfer: configer and start dma transfer
 * @mtd: MTD device structure
 * @addr: virtual address in RAM of source/destination
 * @len: number of data bytes to be transferred
 * @is_write: flag for read/write operation
 */
static inline int _w55fa93_nand_dma_transfer(struct mtd_info *mtd, const u_char *addr,
                        unsigned int len, int is_write)
{
    struct w55fa93_nand_info *nand = container_of(mtd, struct w55fa93_nand_info, mtd);
    dma_addr_t dma_addr = (dma_addr_t)nand->pnand_phyaddr;

    ENTER();

    if ( down_interruptible(&dmac_sem) )
        return -1;

    // For save, wait DMAC to ready
    while ( readl(REG_DMACCSR) & FMI_BUSY );

    // Reinitial dmac
    w55fa93_nand_dmac_init();

    // Fill dma_addr
    writel((unsigned long)dma_addr, REG_DMACSAR);

    // Enable target abort interrupt generation during DMA transfer.
    writel(TABORT_IE, REG_DMACIER);

    // Clear Ready/Busy 0 Rising edge detect flag
    writel(SMISR_RB0_IF, REG_SMISR);

    // Set which BCH algorithm
    if ( nand->eBCHAlgo >= 0 ) {
        // Set BCH algorithm
        writel( (readl(REG_SMCSR) & (~SMCR_BCH_TSEL)) | g_i32BCHAlgoIdx[nand->eBCHAlgo], REG_SMCSR);
        // Enable H/W ECC, ECC parity check enable bit during read page
        writel( readl(REG_SMCSR) | SMCR_ECC_EN | SMCR_ECC_CHK, REG_SMCSR);

    } else  {
        // Disable H/W ECC / ECC parity check enable bit during read page
        writel( readl(REG_SMCSR) & (~SMCR_ECC_EN) &(~SMCR_ECC_CHK), REG_SMCSR);
    }

    writel( nand->m_i32SMRASize , REG_SMREAREA_CTL );

    writel( readl(REG_SMIER) & (~SMIER_ECC_FIELD_IE), REG_SMIER );

    writel ( SMISR_ECC_FIELD_IF, REG_SMISR );

    // Enable SM_CS0
    //writel((readl(REG_SMCSR)&(~0x06000000))|0x04000000, REG_SMCSR);
    writel(readl(REG_SMCSR) |  SMCR_CS1, REG_SMCSR);    // disable NAND port 1
    writel(readl(REG_SMCSR) & ~SMCR_CS0, REG_SMCSR);    // enable NAND port 0

    /* setup and start DMA using dma_addr */

    if ( is_write ) {
        register char * ptr=REG_SMRA_0;
        // To mark this page as dirty.
        if ( ptr[3] == 0xFF )
            ptr[3] = 0;
        if ( ptr[2] == 0xFF )
            ptr[2] = 0;

        if ( addr )
            memcpy( (void*)nand->pnand_vaddr, (void*)addr, len);

        writel ( readl(REG_SMCSR) | SMCR_DWR_EN, REG_SMCSR );

        while ( !(readl(REG_SMISR) & SMISR_DMA_IF) );

    } else {
        // Blocking for reading
        // Enable DMA Read

        writel ( readl(REG_SMCSR) | SMCR_DRD_EN, REG_SMCSR);

        if ( readl(REG_SMCSR)&SMCR_ECC_CHK ) {
            do {
                int stat=0;
                if ( (stat=fmiSMCorrectData ( mtd,  (unsigned long)nand->pnand_vaddr)) < 0 )
                {
                    mtd->ecc_stats.failed++;
                    writel ( SMISR_ECC_FIELD_IF, REG_SMISR );
                    writel ( DMAC_EN|DMAC_SWRST, REG_DMACCSR);          // reset DMAC
                    writel ( readl(REG_SMCSR)|SMCR_SM_SWRST, REG_SMCSR);    // reset SM controller
                    break;
                }
                else if ( stat > 0 ) {
                    //mtd->ecc_stats.corrected += stat; //Occure: MLC UBIFS mount error
                    writel ( SMISR_ECC_FIELD_IF, REG_SMISR );
                }

            } while ( !(readl(REG_SMISR) & SMISR_DMA_IF) || (readl(REG_SMISR) & SMISR_ECC_FIELD_IF) );
        } else
            while ( !(readl(REG_SMISR) & SMISR_DMA_IF) );

        if ( addr )
            memcpy( (void*)addr, (void*)nand->pnand_vaddr,  len );
    }

    w55fa93_nand_dmac_fini();

    up(&dmac_sem);

    LEAVE();
    return 0;
}

/**
 * w55fa93_read_buf_dma_pref - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: buffer to store date
 * @len: number of bytes to read
 */
static void w55fa93_read_buf_dma(struct mtd_info *mtd, u_char *buf, int len)
{
    ENTER();

    if ( w55fa93_wait_sem(mtd) < 0 )
        return;

    if ( len == mtd->writesize ) /* start transfer in DMA mode */
        _w55fa93_nand_dma_transfer ( mtd, buf, len, 0x0);

    else {
        w55fa93_nand_read_buf(mtd, buf, len);

#ifdef W55FA93_NAND_DEBUG
        {
        int i;
        printk("R OOB %d\n", len );
        for ( i=0; i<len; i++ )
        {
            printk("%02X ", buf[i] );
            if ( i%32 == 31 )   printk("\n");
        }
        printk("\n");
        }
#endif

    }

    w55fa93_release_sem(mtd);

    LEAVE();
}

/**
 * w55fa93_write_buf_dma_pref - write buffer to NAND controller
 * @mtd: MTD device structure
 * @buf: data buffer
 * @len: number of bytes to write
 */
static void w55fa93_write_buf_dma(struct mtd_info *mtd,
                    const u_char *buf, int len)
{
    ENTER();

    if ( w55fa93_wait_sem(mtd) < 0 )
        return;

    if ( len == mtd->writesize ) /* start transfer in DMA mode */
        _w55fa93_nand_dma_transfer(mtd, (u_char *)buf, len, 0x1);
    else
    {
#ifdef W55FA93_NAND_DEBUG
        int i;
        printk("W OOB %d\n", len);
        for ( i=0; i<len; i++ )
        {
            printk("%02X ", buf[i] );
            if ( i%32 == 31 )   printk("\n");
        }
#endif
        w55fa93_nand_write_buf(mtd, buf, len);
    }

    w55fa93_release_sem(mtd);

    LEAVE();
}


/**
 * w55fa93_check_rb - check ready/busy pin
 * @mtd: MTD device structure
 */
static int w55fa93_check_rb(struct w55fa93_nand_info *nand)
{
    unsigned int val;

    ENTER();
    spin_lock(&nand->lock);
    val = readl(REG_SMISR) & READYBUSY;
    spin_unlock(&nand->lock);
    LEAVE();

    return val;
}

static int w55fa93_nand_devready(struct mtd_info *mtd)
{
    struct w55fa93_nand_info *nand;
    int ready;

    ENTER();

    if ( w55fa93_wait_sem(mtd) < 0 )
        return -1;

    nand = container_of(mtd, struct w55fa93_nand_info, mtd);

    ready = (w55fa93_check_rb(nand)) ? 1 : 0;

    w55fa93_release_sem(mtd);

    LEAVE();

    return ready;
}

static void w55fa93_nand_command_lp(struct mtd_info *mtd, unsigned int command,
                   int column, int page_addr)
{
    register struct nand_chip *chip = mtd->priv;
    struct w55fa93_nand_info *nand;

    ENTER();

    if ( w55fa93_wait_sem(mtd) < 0 )
    {
        LEAVE();
        return;
    }

    nand = container_of(mtd, struct w55fa93_nand_info, mtd);

    if ( page_addr != -1 && ( command == NAND_CMD_SEQIN || command == NAND_CMD_READ0 || command == NAND_CMD_READOOB ) )
        w55fa93_choice_bch_algo ( mtd, page_addr );

    if (command == NAND_CMD_READOOB) {
        column += mtd->writesize;
        command = NAND_CMD_READ0;
    }

#ifdef W55FA93_NAND_DEBUG
    //if ( mtd->erasesize )
    //    printk("command=0x%02X, column=0x%X, page_addr=0x%X, BlockIdx=0x%X\n", command, column, page_addr, (page_addr*mtd->writesize)/mtd->erasesize );
#endif

    write_cmd_reg(nand, command & 0xff);

    if (command == NAND_CMD_READID) {
        /* Send READ ID command by only one write_addr_reg() */
        /* Some NAND flash don't accept multiple write_addr_reg() */
        write_addr_reg(nand, ENDADDR);
    } else {
        if (column != -1 || page_addr != -1) {

            if (column != -1) {
                //if (chip->options & NAND_BUSWIDTH_16)
                //  column >>= 1;

                write_addr_reg(nand, (column&0xFF) );
                if ( page_addr != -1 )
                    write_addr_reg(nand, (column >> 8) );
                else
                    write_addr_reg(nand, (column >> 8) | ENDADDR);
            }

            if (page_addr != -1) {

                write_addr_reg(nand, (page_addr&0xFF) );

                if ( chip->chipsize > (128 << 20) ) {
                    write_addr_reg(nand, (page_addr >> 8)&0xFF );
                    write_addr_reg(nand, ((page_addr >> 16)&0xFF)|ENDADDR );
                } else {
                    write_addr_reg(nand, ((page_addr >> 8)&0xFF)|ENDADDR );
                }
            }
        }
    }

    switch (command) {
    case NAND_CMD_CACHEDPROG:
    case NAND_CMD_PAGEPROG:
    case NAND_CMD_ERASE1:
    case NAND_CMD_ERASE2:
    case NAND_CMD_SEQIN:
    case NAND_CMD_RNDIN:
    case NAND_CMD_STATUS:
    case NAND_CMD_DEPLETE1:
    case NAND_CMD_STATUS_ERROR:
    case NAND_CMD_STATUS_ERROR0:
    case NAND_CMD_STATUS_ERROR1:
    case NAND_CMD_STATUS_ERROR2:
    case NAND_CMD_STATUS_ERROR3:
        LEAVE();
        w55fa93_release_sem(mtd);
        return;

    case NAND_CMD_RESET:
        if (chip->dev_ready)
            break;

        if ( chip->chip_delay )
            udelay(chip->chip_delay);

        write_cmd_reg(nand, NAND_CMD_STATUS);
        write_cmd_reg(nand, command);

        while (!w55fa93_check_rb(nand));

        LEAVE();

        w55fa93_release_sem(mtd);
        return;

    case NAND_CMD_RNDOUT:
        write_cmd_reg(nand, NAND_CMD_RNDOUTSTART);
        w55fa93_release_sem(mtd);
        LEAVE();
        return;

    case NAND_CMD_READ0:
        write_cmd_reg(nand, NAND_CMD_READSTART);
        break;
    default:

        if (!chip->dev_ready) {
            if ( chip->chip_delay )
                udelay(chip->chip_delay);
            LEAVE();
            w55fa93_release_sem(mtd);
            return;
        }
    }

    while (!w55fa93_check_rb(nand));

    w55fa93_release_sem(mtd);

    LEAVE();
}

/* select chip */
static void w55fa93_nand_select_chip(struct mtd_info *mtd, int chip)
{
    //writel((readl(REG_SMCSR)&(~0x06000000))|0x04000000, REG_SMCSR);
    writel(readl(REG_SMCSR) |  SMCR_CS1, REG_SMCSR);    // disable NAND port 1
    writel(readl(REG_SMCSR) & ~SMCR_CS0, REG_SMCSR);    // enable NAND port 0

    return;
}

/*
 * Calculate HW ECC
 * function called after a write
 * mtd:        MTD block structure
 * dat:        raw data (unused)
 * ecc_code:   buffer for ECC
 */
static int w55fa93_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
                      u_char *ecc_code)
{
    return 0;
}

/**
 * nand_write_page_hwecc - [REPLACABLE] hardware ecc based page write function
 * @mtd:        mtd info structure
 * @chip:       nand chip info structure
 * @buf:        data buffer
 */
static void w55fa93_nand_write_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
                                  const uint8_t *buf)
{
    uint8_t *ecc_calc = chip->buffers->ecccalc;
    uint32_t hweccbytes=chip->ecc.layout->eccbytes;
    register char * ptr=REG_SMRA_0;

    ENTER();

    if ( w55fa93_wait_sem(mtd) < 0 )
    {
        LEAVE();
        return;
    }

    memset ( (void*)ptr, 0xFF, mtd->oobsize );
    memcpy ( (void*)ptr, (void*)chip->oob_poi,  mtd->oobsize - chip->ecc.total );

    _w55fa93_nand_dma_transfer( mtd, buf, mtd->writesize , 0x1);

    // Copy parity code in SMRA to calc
    memcpy ( (void*)ecc_calc,  (void*)( REG_SMRA_0 + ( mtd->oobsize - chip->ecc.total ) ), chip->ecc.total );

    // Copy parity code in calc to oob_poi
    memcpy ( (void*)(chip->oob_poi+hweccbytes), (void*)ecc_calc, chip->ecc.total);

    w55fa93_release_sem(mtd);

    LEAVE();
}

/**
 * w55fa93_nand_read_page_hwecc_oob_first - hardware ecc based page write function
 * @mtd:        mtd info structure
 * @chip:       nand chip info structure
 * @buf:        buffer to store read data
 * @page:       page number to read
 */
static int w55fa93_nand_read_page_hwecc_oob_first(struct mtd_info *mtd,
        struct nand_chip *chip, uint8_t *buf, int page)
{
    int eccsize = chip->ecc.size;
    uint8_t *p = buf;
    char * ptr=REG_SMRA_0;

    ENTER();

    if ( w55fa93_wait_sem(mtd) < 0 )
    {
        LEAVE();
        return -1;
    }

    //w55fa93_choice_bch_algo ( mtd, page );

    /* At first, read the OOB area  */
    w55fa93_nand_command_lp(mtd, NAND_CMD_READOOB, 0, page);
    w55fa93_nand_read_buf(mtd, chip->oob_poi, mtd->oobsize);

    // Second, copy OOB data to SMRA for page read
    memcpy ( (void*)ptr, (void*)chip->oob_poi, mtd->oobsize );

    // Third, read data from nand
    w55fa93_nand_command_lp(mtd, NAND_CMD_READ0, 0, page);
    _w55fa93_nand_dma_transfer(mtd, p, eccsize, 0x0);

    // Fouth, restore OOB data from SMRA
    memcpy ( (void*)chip->oob_poi, (void*)ptr, mtd->oobsize );

    w55fa93_release_sem(mtd);

    LEAVE();

    return 0;
}

static void w55fa93_layout_oob_table ( struct nand_ecclayout* pNandOOBTbl, int oobsize , int eccbytes )
{
    pNandOOBTbl->eccbytes = eccbytes;

    pNandOOBTbl->oobavail = oobsize - DEF_RESERVER_OOB_SIZE_FOR_MARKER - eccbytes;

    pNandOOBTbl->oobfree[0].offset = DEF_RESERVER_OOB_SIZE_FOR_MARKER;  // Bad block marker size

    pNandOOBTbl->oobfree[0].length = oobsize - eccbytes - pNandOOBTbl->oobfree[0].offset;
}

/**
 * nand_read_oob_std - [REPLACABLE] the most common OOB data read function
 * @mtd:        mtd info structure
 * @chip:       nand chip info structure
 * @page:       page number to read
 * @sndcmd:     flag whether to issue read command or not
 */
static int w55fa93_nand_read_oob_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
                             int page, int sndcmd)
{
    char * ptr=REG_SMRA_0;

    ENTER();

    if ( w55fa93_wait_sem(mtd) < 0 )
    {
        LEAVE();
        return -1;
    }

    if ( sndcmd ) {
        w55fa93_nand_command_lp(mtd, NAND_CMD_READOOB, 0, page);
        sndcmd = 0;
    } else
        w55fa93_choice_bch_algo ( mtd, page );

    w55fa93_nand_read_buf(mtd, &chip->oob_poi[0], mtd->oobsize);

    // Second, copy OOB data to SMRA for page read
    memcpy ( (void*)ptr, (void*)chip->oob_poi, mtd->oobsize );

    // Third, read data from nand
    w55fa93_nand_command_lp(mtd, NAND_CMD_READ0, 0, page);
    _w55fa93_nand_dma_transfer(mtd, NULL, mtd->writesize, 0x0);

    // Fouth, recovery OOB data for SMRA
    memcpy ( (void*)chip->oob_poi, (void*)ptr, mtd->oobsize );

    w55fa93_release_sem(mtd);

    return sndcmd;
}

/**
 * nand_write_page - [REPLACEABLE] write one page
 * @mtd:        MTD device structure
 * @chip:       NAND chip descriptor
 * @buf:        the data to write
 * @page:       page number to write
 * @cached:     cached programming
 * @raw:        use _raw version of write_page
 */
static int w55fa93_nand_write_page(struct mtd_info *mtd, struct nand_chip *chip,
                             const uint8_t *buf, int page, int cached, int raw)
{
    int status;

    ENTER();

    if ( w55fa93_wait_sem(mtd) < 0 )
    {
        LEAVE();
        return -1;
    }

    w55fa93_nand_command_lp(mtd, NAND_CMD_SEQIN, 0x00, page);

    if (unlikely(raw))
        chip->ecc.write_page_raw(mtd, chip, buf);
    else
    {
        if ( page >= 0 && page < (1<<(chip->phys_erase_shift+2)) / mtd->writesize ) // four blocks
        {
            // Special pattern
            char * ptr=REG_SMRA_0;
            memset ( (void*)ptr, 0xFF, mtd->oobsize );
            ptr[3] = 0x00;
            ptr[2] = page&0xff;
            ptr[1] = 0x5A;
            _w55fa93_nand_dma_transfer( mtd, buf, mtd->writesize , 0x1 );
        } else
        {
            w55fa93_nand_write_page_hwecc ( mtd, chip, buf );
            //Wayne chip->ecc.write_page(mtd, chip, buf);
        }
    }

    /*
     * Cached progamming disabled for now, Not sure if its worth the
     * trouble. The speed gain is not very impressive. (2.3->2.6Mib/s)
     */
    cached = 0;

    if (!cached || !(chip->options & NAND_CACHEPRG)) {
        w55fa93_nand_command_lp(mtd, NAND_CMD_PAGEPROG, -1, -1);
        status = chip->waitfunc(mtd, chip);
        /*
         * See if operation failed and additional status checks are
         * available
         */
        if ((status & NAND_STATUS_FAIL) && (chip->errstat))
            status = chip->errstat(mtd, chip, FL_WRITING, status, page);

        if (status & NAND_STATUS_FAIL)
        {
            w55fa93_release_sem(mtd);
            return -EIO;
        }
    } else {
        w55fa93_nand_command_lp(mtd, NAND_CMD_CACHEDPROG, -1, -1);
        status = chip->waitfunc(mtd, chip);
    }

#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
    /* Send command to read back the data */
    w55fa93_nand_command_lp(mtd, NAND_CMD_READ0, 0, page);

    if (chip->verify_buf(mtd, buf, mtd->writesize))
    {
        w55fa93_release_sem(mtd);
        return -EIO;
    }
#endif

    w55fa93_release_sem(mtd);

    return 0;
}

static int __devinit w55fa93_nand_probe(struct platform_device *pdev)
{
    struct nand_chip *chip;
    struct w55fa93_nand_info *w55fa93_nand;
    struct mtd_info *mtd;

    int retval=0;

    E_PAGESIZE ePageSize;

    ENTER();

    w55fa93_nand = kzalloc(sizeof(struct w55fa93_nand_info), GFP_KERNEL);
    if (!w55fa93_nand)
        return -ENOMEM;

    w55fa93_nand->pnand_vaddr = (unsigned char *) dma_alloc_writecombine(NULL, 512*16, (dma_addr_t *)&w55fa93_nand->pnand_phyaddr, GFP_KERNEL);
    if(w55fa93_nand->pnand_vaddr == NULL){
        printk(KERN_ERR "fa93_nand: failed to allocate ram for nand data.\n");
        return -ENOMEM;
    }

    platform_set_drvdata(pdev, w55fa93_nand);

    spin_lock_init(&w55fa93_nand->controller.lock);
    init_waitqueue_head(&w55fa93_nand->controller.wq);

    w55fa93_nand->pdev = pdev;

    mtd=&w55fa93_nand->mtd;
    chip = &(w55fa93_nand->chip);

    w55fa93_nand->mtd.priv  = chip;
    w55fa93_nand->mtd.owner = THIS_MODULE;
    spin_lock_init(&w55fa93_nand->lock);

    /*
     * Get Clock
     */
//    w55fa93_nand->sic_clk = clk_get(NULL, "sic");
    w55fa93_nand->sic_clk = clk_get(NULL, "SIC");
    if (IS_ERR(w55fa93_nand->sic_clk)) {
        printk("Error: mtd: no sic_clk?\n");
        retval = -ENXIO;
        goto fail1;
    }

//    w55fa93_nand->clk = clk_get(NULL, "nand");
    w55fa93_nand->clk = clk_get(NULL, "NAND");
    if (IS_ERR(w55fa93_nand->clk)) {
        printk("Error: mtd: no nand_clk?\n");
        goto fail2;
    }

    clk_enable(w55fa93_nand->sic_clk);
    clk_enable(w55fa93_nand->clk);

    w55fa93_nand->chip.controller = &w55fa93_nand->controller;

    chip->cmdfunc       = w55fa93_nand_command_lp;

    chip->read_byte     = w55fa93_nand_read_byte;
    chip->select_chip   = w55fa93_nand_select_chip;
    chip->verify_buf    = w55fa93_verify_buf;

#ifndef DMAC_TRANSFER
    chip->write_buf = w55fa93_nand_write_buf;
    chip->read_buf  = w55fa93_nand_read_buf;
#else
    chip->read_buf  = w55fa93_read_buf_dma;
    chip->write_buf = w55fa93_write_buf_dma;
#endif

    /*
    * If RDY/BSY line is connected to OMAP then use the omap ready
    * funcrtion and the generic nand_wait function which reads the status
    * register after monitoring the RDY/BSY line.Otherwise use a standard
    * chip delay which is slightly more than tR (AC Timing) of the NAND
    * device and read status register until you get a failure or success
    */
    // Check NAND device NBUSY0 pin
    chip->dev_ready     = w55fa93_nand_devready;
    //chip->options     |= NAND_SKIP_BBTSCAN;

    w55fa93_nand->reg   = 0x00;

    // Read OOB data first, then HW read page
    chip->write_page    = w55fa93_nand_write_page;
    chip->ecc.mode      = NAND_ECC_HW_OOB_FIRST;
    chip->ecc.hwctl     = w55fa93_nand_enable_hwecc;
    chip->ecc.calculate = w55fa93_nand_calculate_ecc;
    chip->ecc.correct   = w55fa93_nand_correct_data;
    chip->ecc.write_page= w55fa93_nand_write_page_hwecc;
    chip->ecc.read_page = w55fa93_nand_read_page_hwecc_oob_first;
    chip->ecc.read_oob  = w55fa93_nand_read_oob_hwecc;
    chip->ecc.layout    = &w55fa93_nand_oob;

    w55fa93_nand_initialize ( );

    w55fa93_nand->m_i32MyShowTime = 0;

    /* first scan to find the device and get the page size */
    if (nand_scan_ident(&(w55fa93_nand->mtd), 1, NULL)) {
        retval = -ENXIO;
        goto fail3;
    }

    //Set PSize bits of SMCSR register to select NAND card page size
    switch (mtd->writesize) {
        case 2048:
            writel( (readl(REG_SMCSR)&(~SMCR_PSIZE)) + PSIZE_2K,    REG_SMCSR);
            ePageSize = ePageSize_2048;
            break;

        case 4096:
            writel( (readl(REG_SMCSR)&(~SMCR_PSIZE)) + PSIZE_4K,    REG_SMCSR);
            ePageSize = ePageSize_4096;
            break;

        case 8192:
            writel( (readl(REG_SMCSR)&(~SMCR_PSIZE)) + PSIZE_8K,    REG_SMCSR);
            ePageSize = ePageSize_8192;
            break;

        /* Not support now. */
        case 512:
            //writel( (readl(REG_SMCSR)&(~SMCR_PSIZE)) + PSIZE_512,     REG_SMCSR);
            //ePageSize = ePageSize_512;
            //break;

        default:
            printk("W55FA93 NAND CONTROLLER IS NOT SUPPORT THE PAGE SIZE. (%d, %d)\n", mtd->writesize, mtd->oobsize );
            goto fail3;
    }
    w55fa93_nand->m_ePageSize = ePageSize;

    {
        // System area
        int i32eBCHAlgo = 0;
        switch(ePageSize) {
            case 0:
            case 1: i32eBCHAlgo = eBCH_T4;  break;
            case 2: i32eBCHAlgo = eBCH_T8;  break;
            case 3: i32eBCHAlgo = eBCH_T12; break;
            default:
                break;
        }
        w55fa93_nand_SYSTEM_oob.m_BCHAlgoidx = i32eBCHAlgo;
        w55fa93_nand_SYSTEM_oob.m_SMRASize   = g_SYSAREA_NAND_EXTRA_SIZE[ePageSize];
        w55fa93_layout_oob_table ( (struct nand_ecclayout*)&w55fa93_nand_SYSTEM_oob, w55fa93_nand_SYSTEM_oob.m_SMRASize, g_i32ParityNum[ePageSize][i32eBCHAlgo] );
        printk("SYSTEM: USE %s HWECC algorithm(SMRA size: %d, Parity number:%d bytes)\n",
                g_pcBCHAlgoIdx[i32eBCHAlgo],
                w55fa93_nand_SYSTEM_oob.m_SMRASize,
                g_i32ParityNum[ePageSize][i32eBCHAlgo] );


        i32eBCHAlgo = w55fa93_hwecc_choice_bch_algorithm ( ePageSize, mtd->oobsize, 0 );
        w55fa93_nand_EXECUTE_oob.m_BCHAlgoidx = i32eBCHAlgo;

        if ( ePageSize==3 && mtd->oobsize >= g_SYSAREA_NAND_EXTRA_SIZE[ePageSize] ) // 8K, oob >= 376Byte
        {
            w55fa93_nand_EXECUTE_oob.m_SMRASize = g_SYSAREA_NAND_EXTRA_SIZE[ePageSize];
            mtd->oobsize = w55fa93_nand_EXECUTE_oob.m_SMRASize;
        }
        else
            w55fa93_nand_EXECUTE_oob.m_SMRASize = mtd->oobsize;

        if ( i32eBCHAlgo >= 0 ) {
            w55fa93_layout_oob_table ( (struct nand_ecclayout*)&w55fa93_nand_EXECUTE_oob, w55fa93_nand_EXECUTE_oob.m_SMRASize, g_i32ParityNum[ePageSize][i32eBCHAlgo] );
            printk("EXECUTE: USE %s HWECC algorithm(SMRA size: %d, Parity number:%d bytes)\n",
                    g_pcBCHAlgoIdx[i32eBCHAlgo],
                    w55fa93_nand_EXECUTE_oob.m_SMRASize,
                    g_i32ParityNum[ePageSize][i32eBCHAlgo] );
        }

        // System area
        partitions[0].size = 1<<(chip->phys_erase_shift+2);       // 4 block
    }

    w55fa93_nand->parts = (struct mtd_partition*)partitions;
    w55fa93_nand->nr_parts = ARRAY_SIZE(partitions);

    w55fa93_nand->m_i32SMRASize = mtd->oobsize;
    w55fa93_nand->eBCHAlgo = w55fa93_hwecc_choice_bch_algorithm ( ePageSize, mtd->oobsize, RESERVED_FOR_FS );
    if ( w55fa93_nand->eBCHAlgo < 0 ) {
        w55fa93_layout_oob_table ( &w55fa93_nand_oob, mtd->oobsize, 0 );
        chip->ecc.bytes = 0;
        chip->ecc.size  = mtd->writesize;
        printk("Choice BCH algorithm: No match.\n");
    } else {
        w55fa93_layout_oob_table ( &w55fa93_nand_oob, mtd->oobsize, g_i32ParityNum[ePageSize][w55fa93_nand->eBCHAlgo] );
        chip->ecc.bytes = w55fa93_nand_oob.eccbytes;
        chip->ecc.size  = mtd->writesize;
        printk("USE %s HWECC algorithm(Parity number:%d bytes)\n", g_pcBCHAlgoIdx[w55fa93_nand->eBCHAlgo],  g_i32ParityNum[ePageSize][w55fa93_nand->eBCHAlgo] );
    }

    /* second phase scan */
    if ( nand_scan_tail( &(w55fa93_nand->mtd) ) ) {
        retval = -ENXIO;
        goto fail3;
    }

    if ( w55fa93_nand->eBCHAlgo >= 0 )
        w55fa93_nand_hwecc_init (&(w55fa93_nand->mtd));
    else
        w55fa93_nand_hwecc_fini (&(w55fa93_nand->mtd));

    dump_chip_info( chip );

#ifdef CONFIG_MTD_PARTITIONS
    if ( add_mtd_partitions(&(w55fa93_nand->mtd), partitions, ARRAY_SIZE(partitions) ) < 0 )
    {
        printk("add_mtd_partitions: ENOMEM\n");
        goto fail3;
    }
#endif

    LEAVE();

    dump_regs(__LINE__);

    printk("fmi-sm: registered successfully! \n");
    return retval;
fail3:
fail2:
fail1:
    kfree(w55fa93_nand);
    return retval;
}

static int __devexit w55fa93_nand_remove(struct platform_device *pdev)
{
    struct w55fa93_nand_info *w55fa93_nand = platform_get_drvdata(pdev);

    struct mtd_info *mtd=&w55fa93_nand->mtd;

#ifdef CONFIG_MTD_PARTITIONS
    del_mtd_partitions(mtd);
#endif

    w55fa93_nand_hwecc_fini(mtd);

    clk_disable(w55fa93_nand->clk);

    clk_put(w55fa93_nand->clk);

    dma_free_coherent(NULL, 512*16, w55fa93_nand->pnand_vaddr, (dma_addr_t )w55fa93_nand->pnand_phyaddr);

    kfree(w55fa93_nand);

    platform_set_drvdata(pdev, NULL);

    return 0;
}

/* FMI-SM Device */
struct platform_device w55fa93_device_fmi_sm = {
    .name   = "w55fa93-fmi-sm",
    .id     = -1,
};

static struct platform_driver w55fa93_nand_driver = {
    .driver = {
        .name   = "w55fa93-fmi-sm",
        .owner  = THIS_MODULE,
    },
    .probe  = w55fa93_nand_probe,
    .remove = __devexit_p(w55fa93_nand_remove),
};

static int __init w55fa93_nand_init(void)
{
    int ret;
    printk("w55fa93 mtd nand driver version: %s\n", W55FA93_DRV_VERSION );
    ret = platform_device_register(&w55fa93_device_fmi_sm);
    if (ret) {
        printk("fmi-sm: failed to add board device %s \n", w55fa93_device_fmi_sm.name);
        return ret;
    }

    ret = platform_driver_register(&w55fa93_nand_driver);
    if (ret) {
        printk("fmi-sm: failed to add device driver %s \n", w55fa93_device_fmi_sm.name);
        return ret;
    }

    return ret;
}

static void __exit w55fa93_nand_exit(void)
{
    platform_driver_unregister(&w55fa93_nand_driver);
    platform_device_unregister(&w55fa93_device_fmi_sm);
    printk("fmi-sm: unregistered successfully! \n");
}

module_init(w55fa93_nand_init);
module_exit(w55fa93_nand_exit);

MODULE_AUTHOR("nuvoton");
MODULE_DESCRIPTION("w55fa93 nand driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:w55fa93-fmi");
