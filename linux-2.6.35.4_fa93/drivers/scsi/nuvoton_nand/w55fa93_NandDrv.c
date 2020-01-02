/* linux/driver/scsi/nuvoton_nand/w55fa93_NandDrv.c
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
 *   2008/08/19     jcao add this file for nuvoton all nand driver.
 *   2011/03/04     C.J.Chen, to support NAND ROM chip "Infinite IM90A001GT"
 *                  Problem: This chip use different command to read redundancy data
 *                     and result in chip response "busy" always for
 *                     fmiSM_Read_RA_512() and fmiSM2BufferM_RA().
 *                  Solution: Since ROM chip don't need to check ECC, so we response "ready"
 *                     for ROM chip always.
 *   2011/03/22     C.J.Chen, to support NAND ROM chip "Infinite IM90A001GT" on both MTD and RS.
 *                  Problem: fmiNormalCheckBlock() and fmiCheckInvalidBlock() will check the
 *                      redundancy data that fmiSM2BufferM_RA() read back.
 *                      At 2011/03/04 code, we just return 0 but don't prepare the data for them to check.
 *                  Solution: Force to assign 0xFF to read back data for ROM chip
 *                      within fmiNormalCheckBlock() and fmiCheckInvalidBlock().
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/highmem.h>
#include <linux/blkdev.h>
#include <linux/string.h>
//#include <linux/autoconf.h>
#include <linux/slab.h>

#include <mach/w55fa93_reg.h>
#include <mach/gnand/GNAND.h>
#include <mach/gnand/GNAND_global.h>
#include <mach/NandDrv.h>
#include <mach/w55fa93_nand.h>

#define FALSE             0
#define TRUE              1

#undef  outl
#undef  inl
#define outl    writel
#define inl     readl

#define OPT_FIRST_4BLOCKS_ECC4
#define OPT_MARK_BAD_BLOCK_WHILE_ERASE_FAIL
#define OPT_SUPPORT_H27UAG8T2A

u8 volatile _fmi_bIsNandFirstAccess = 1;
extern char volatile _fmi_bIsSMDataReady;
static int _nand_init0 = 0, _nand_init1 = 0;

//---2013/11/29, write "check marker" to spare area if write_nandloader is TRUE
static u8 write_nandloader = 0;

extern u32 _fmi_ucNANDBuffer;
extern u32 _fmi_pNANDBuffer;
extern u8 *_fmi_gptr1;


#if 0
#define DEBUG
#define SD_DEBUG
#define DUMP
#define SD_DEBUG_ENABLE_ENTER_LEAVE
#define SD_DEBUG_ENABLE_MSG
#define SD_DEBUG_ENABLE_MSG2
#define SD_DEBUG_PRINT_LINE
#endif

#ifdef SD_DEBUG
#define PDEBUG(fmt, arg...)     printk(fmt, ##arg)
#else
#define PDEBUG(fmt, arg...)
#endif

#ifdef SD_DEBUG_PRINT_LINE
#define PRN_LINE()              PDEBUG("[%-20s] : %d\n", __FUNCTION__, __LINE__)
#else
#define PRN_LINE()
#endif

#ifdef SD_DEBUG_ENABLE_ENTER_LEAVE
#define ENTER()                 PDEBUG("[%-20s] : Enter...\n", __FUNCTION__)
#define LEAVE()                 PDEBUG("[%-20s] : Leave...\n", __FUNCTION__)
#else
#define ENTER()
#define LEAVE()
#endif

#ifdef SD_DEBUG_ENABLE_MSG
#define MSG(msg)                PDEBUG("[%-20s] : %s\n", __FUNCTION__, msg)
#else
#define MSG(msg)
#endif

#ifdef SD_DEBUG_ENABLE_MSG2
#define MSG2(fmt, arg...)           PDEBUG("[%-20s] : "fmt, __FUNCTION__, ##arg)
#define PRNBUF(buf, count)      {int i;MSG2("CID Data: ");for(i=0;i<count;i++)\
                                    PDEBUG("%02x ", buf[i]);PDEBUG("\n");}
#else
#define MSG2(fmt, arg...)
#define PRNBUF(buf, count)
#endif

extern struct semaphore fmi_sem;
extern struct semaphore dmac_sem;
/* function pointer */
FMI_SM_INFO_T *pSM0 = NULL, *pSM1 = NULL;
#define NAND_RESERVED_BLOCK     10

#if defined(CONFIG_W55FA93_NAND_BOTH) || defined(CONFIG_W55FA93_NAND1)
    extern int nand_card_status(void);
#endif

#define OPT_NAND_ENABLED_PROTECT

/* functions */

int fmiSMCheckRB(FMI_SM_INFO_T *pSM)
{
        unsigned long volatile count1, count2;

        count1 = jiffies + NAND_SHORT_DELAY;

        ENTER();
        while (1) {

    #if defined(CONFIG_W55FA93_TWO_RB_PINS) || defined(CONFIG_W55FA93_NAND1)
                if(pSM == pSM0)
                {
                    if (inpw(REG_SMISR) & SMISR_RB0_IF)
                    {
                        outpw(REG_SMISR, SMISR_RB0_IF);
                        outpw(REG_SMCMD, 0x70);     // status read command
                        if (inpw(REG_SMDATA) & 0x40) {  // 1:ready; 0:busy
                                outpw(REG_SMCMD, 0x00);
                                return 1;
                        } else {
                                count2 = jiffies + NAND_SHORT_DELAY;
                                while (1) {
                                        outpw(REG_SMCMD, 0x70);     // status read command
                                        if (inpw(REG_SMDATA) & 0x40) {  // 1:ready; 0:busy
                                                outpw(REG_SMISR, SMISR_RB0_IF);
                                                outpw(REG_SMCMD, 0x00);
                                                 return 1;
                                        }

                                        //if (jiffies >= count2) {
                                        if (time_after(jiffies, count2)) {
                                                printk("R/B# timeout 1\n");
                                                return 0;   // timeout
                                        }
                                        schedule();
                                #ifdef OPT_NAND_ENABLED_PROTECT
                                        outpw(REG_FMICR, FMI_SM_EN);    // 20121205 mhkuo
                                #endif
                                }
                        }
                    }
                }
                else
                {
                    if (inpw(REG_SMISR) & SMISR_RB1_IF)
                    {
                        outpw(REG_SMISR, SMISR_RB1_IF);
                        outpw(REG_SMCMD, 0x70);     // status read command
                        if (inpw(REG_SMDATA) & 0x40) {  // 1:ready; 0:busy
                                outpw(REG_SMCMD, 0x00);
                                return 1;
                        } else {
                                count2 = jiffies + NAND_SHORT_DELAY;
                                while (1) {
                                        outpw(REG_SMCMD, 0x70);     // status read command
                                        if (inpw(REG_SMDATA) & 0x40) {  // 1:ready; 0:busy
                                                outpw(REG_SMISR, SMISR_RB1_IF);
                                                outpw(REG_SMCMD, 0x00);
                                                 return 1;
                                        }

                                        //if (jiffies >= count2) {
                                        if (time_after(jiffies, count2)) {
                                                printk("R/B# timeout 1\n");
                                                return 0;   // timeout
                                        }
                                        schedule();
                                #ifdef OPT_NAND_ENABLED_PROTECT
                                        outpw(REG_FMICR, FMI_SM_EN);    // 20121205 mhkuo
                                #endif
                                }
                        }
                    }
                }
    #else
                {
                    if (inpw(REG_SMISR) & SMISR_RB0_IF)
                    {
                        outpw(REG_SMISR, SMISR_RB0_IF);
                        outpw(REG_SMCMD, 0x70);     // status read command
                        if (inpw(REG_SMDATA) & 0x40) {  // 1:ready; 0:busy
                                outpw(REG_SMCMD, 0x00);
                                return 1;
                        } else {
                                count2 = jiffies + NAND_SHORT_DELAY;
                                while (1) {
                                        outpw(REG_SMCMD, 0x70);     // status read command
                                        if (inpw(REG_SMDATA) & 0x40) {  // 1:ready; 0:busy
                                                outpw(REG_SMISR, SMISR_RB0_IF);
                                                outpw(REG_SMCMD, 0x00);
                                                 return 1;
                                        }

                                        //if (jiffies >= count2) {
                                        if (time_after(jiffies, count2)) {
                                                printk("R/B# timeout 1\n");
                                                return 0;   // timeout
                                        }
                                        schedule();
                                #ifdef OPT_NAND_ENABLED_PROTECT
                                        outpw(REG_FMICR, FMI_SM_EN);    // 20121205 mhkuo
                                #endif
                                }
                        }
                    }
                }
    #endif

//                if (jiffies >= count1)
                if (time_after(jiffies, count1)) {
                        printk("R/B# timeout 2 <0x%x>, jiffies %x, count1 %x\n", inpw(REG_SMISR), (u32)jiffies, (u32)count1);
                        return 0;   // timeout
                }
                schedule();
        #ifdef OPT_NAND_ENABLED_PROTECT
                outpw(REG_FMICR, FMI_SM_EN);    // 20121205 mhkuo
        #endif
        }
        return 0;
}


int fmiSMCheckStatus(FMI_SM_INFO_T *pSM)
{
    u32 status, ret;

    ret = 0;
    outpw(REG_SMCMD, 0x70);     // Status Read command for NAND flash
    status = inpw(REG_SMDATA);

    if (status & BIT0)          // BIT0: Chip status: 1:fail; 0:pass
    {
        printk("ERROR: NAND device status: FAIL!!\n");
        ret = FMI_SM_STATE_ERROR;
    }

    if ((status & BIT7) == 0)   // BIT7: Write Protect: 1:unprotected; 0:protected
    {
        printk("WARNING: NAND device status: Write Protected!!\n");
        ret = FMI_SM_STATE_ERROR;
    }

    return ret;
}


// SM functions
int fmiSM_Reset(FMI_SM_INFO_T *pSM)
{
#if defined(CONFIG_W55FA93_TWO_RB_PINS) || defined(CONFIG_W55FA93_NAND1)
        u32 volatile i;

        ENTER();

        if(pSM == pSM0)
            outpw(REG_SMISR, SMISR_RB0_IF);
        else
            outpw(REG_SMISR, SMISR_RB1_IF);

        outpw(REG_SMCMD, 0xff);
        for (i=100; i>0; i--);

        if (!fmiSMCheckRB(pSM))
            return FMI_SM_RB_ERR;
        return 0;

#else
        u32 volatile i;

        ENTER();

        outpw(REG_SMISR, SMISR_RB0_IF);
        outpw(REG_SMCMD, 0xff);
        for (i=100; i>0; i--);

        if (!fmiSMCheckRB(pSM))
                return FMI_SM_RB_ERR;
        return 0;

#endif  // CONFIG_W55FA93_TWO_RB_PINS
}


void fmiSM_Initial(FMI_SM_INFO_T *pSM)
{
        ENTER();
        if (pSM->nPageSize == NAND_PAGE_8KB)
            outpw(REG_SMCSR, (inpw(REG_SMCSR)&(~SMCR_PSIZE)) | (PSIZE_8K));     // psize:8192
        else if (pSM->nPageSize == NAND_PAGE_4KB)
            outpw(REG_SMCSR, (inpw(REG_SMCSR)&(~SMCR_PSIZE)) | (PSIZE_4K));     // psize:4096
        else if (pSM->nPageSize == NAND_PAGE_2KB)
            outpw(REG_SMCSR, (inpw(REG_SMCSR)&(~SMCR_PSIZE)) | (PSIZE_2K));     // psize:2048
        else    // pSM->nPageSize == NAND_PAGE_512B
            outpw(REG_SMCSR, (inpw(REG_SMCSR)&(~SMCR_PSIZE)) | (PSIZE_512));    // psize:512

        outpw(REG_SMCSR,  inpw(REG_SMCSR) | SMCR_ECC_EN);   // enable ECC

        if (pSM->bIsCheckECC)
            outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_ECC_CHK);   // enable ECC check
        else
            outpw(REG_SMCSR, inpw(REG_SMCSR) & ~SMCR_ECC_CHK);  // disable ECC check


        // set BCH_Tx and redundant area size
        outpw(REG_SMREAREA_CTL, inpw(REG_SMREAREA_CTL) & ~SMRE_MECC);       // Redundant area size
        if (pSM->nPageSize == NAND_PAGE_8KB)
        {
            outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
            outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T12);            // BCH_12 is selected
            outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 376);  // Redundant area size
        }
        else if (pSM->nPageSize == NAND_PAGE_4KB)
        {
            if (pSM->bIsNandECC12 == TRUE)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T12);            // BCH_12 is selected
    #ifdef OPT_SUPPORT_H27UAG8T2A
            outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 224);;  // Redundant area size
    #else
            outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 216);;  // Redundant area size
    #endif
            }
            else if (pSM->bIsNandECC8 == TRUE)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T8);             // BCH_8 is selected
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 128);  // Redundant area size
            }
            else
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T4);             // BCH_4 is selected
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 128);  // Redundant area size
            }
        }
        else if (pSM->nPageSize == NAND_PAGE_2KB)
        {
            if (pSM->bIsNandECC8 == TRUE)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T8);             // BCH_8 is selected
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 64);  // Redundant area size
            }
            else
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T4);             // BCH_4 is selected
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 64);  // Redundant area size
            }
        }
        else
        {
            outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
            outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T4);             // BCH_4 is selected
            outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 16);  // Redundant area size
        }

        if ((_nand_init0 == 0) && (_nand_init1 == 0))
                outpw(REG_SMIER, SMIER_DMA_IE);

        LEAVE();
}


int fmiSM_ReadID(FMI_SM_INFO_T *pSM, NDISK_T *NDISK_info)
{
        int volatile tempID[5];
        ENTER();
        fmiSM_Reset(pSM);
        outpw(REG_SMCMD, 0x90);     // read ID command
        outpw(REG_SMADDR, 0x80000000);  // address 0x00

        tempID[0] = inpw(REG_SMDATA);
        tempID[1] = inpw(REG_SMDATA);
        tempID[2] = inpw(REG_SMDATA);
        tempID[3] = inpw(REG_SMDATA);
        tempID[4] = inpw(REG_SMDATA);

        NDISK_info->vendor_ID = tempID[0];
        NDISK_info->device_ID = tempID[1];

        if (tempID[0] == 0xC2)
                pSM->bIsCheckECC = FALSE;
        else
                pSM->bIsCheckECC = TRUE;

        pSM->bIsNandECC4 = FALSE;
        pSM->bIsNandECC8 = FALSE;
        pSM->bIsNandECC12 = FALSE;
        pSM->bIsNandECC15 = FALSE;

        switch (tempID[1]) {
                /* page size 512B */
        case 0x79:  // 128M
                pSM->uSectorPerFlash = 255744;
                pSM->uBlockPerFlash = 8191;
                pSM->uPagePerBlock = 32;
                pSM->uSectorPerBlock = 32;
                pSM->bIsMulticycle = TRUE;
                pSM->nPageSize = NAND_PAGE_512B;
                pSM->bIsNandECC4 = TRUE;
                pSM->bIsMLCNand = FALSE;

                NDISK_info->NAND_type = NAND_TYPE_SLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                NDISK_info->nZone = 1;              /* number of zones */
                NDISK_info->nBlockPerZone = 8192;   /* blocks per zone */
                NDISK_info->nPagePerBlock = 32;     /* pages per block */
                NDISK_info->nLBPerZone = 8000;      /* logical blocks per zone */
                NDISK_info->nPageSize = 512;
                break;

        case 0x76:  // 64M
        case 0x5A:  // 64M XtraROM
                pSM->uSectorPerFlash = 127872;
                pSM->uBlockPerFlash = 4095;
                pSM->uPagePerBlock = 32;
                pSM->uSectorPerBlock = 32;
                pSM->bIsMulticycle = TRUE;
                pSM->nPageSize = NAND_PAGE_512B;
                pSM->bIsNandECC4 = TRUE;
                pSM->bIsMLCNand = FALSE;

                NDISK_info->NAND_type = NAND_TYPE_SLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                NDISK_info->nZone = 1;              /* number of zones */
                NDISK_info->nBlockPerZone = 4096;   /* blocks per zone */
                NDISK_info->nPagePerBlock = 32;     /* pages per block */
                NDISK_info->nLBPerZone = 4000;      /* logical blocks per zone */
                NDISK_info->nPageSize = 512;
                break;

        case 0x75:  // 32M
                pSM->uSectorPerFlash = 63936;
                pSM->uBlockPerFlash = 2047;
                pSM->uPagePerBlock = 32;
                pSM->uSectorPerBlock = 32;
                pSM->bIsMulticycle = FALSE;
                pSM->nPageSize = NAND_PAGE_512B;
                pSM->bIsNandECC4 = TRUE;
                pSM->bIsMLCNand = FALSE;

                NDISK_info->NAND_type = NAND_TYPE_SLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                NDISK_info->nZone = 1;              /* number of zones */
                NDISK_info->nBlockPerZone = 2048;   /* blocks per zone */
                NDISK_info->nPagePerBlock = 32;     /* pages per block */
                NDISK_info->nLBPerZone = 2000;      /* logical blocks per zone */
                NDISK_info->nPageSize = 512;
                break;

        case 0x73:  // 16M
                pSM->uSectorPerFlash = 31968;   // max. sector no. = 999 * 32
                pSM->uBlockPerFlash = 1023;
                pSM->uPagePerBlock = 32;
                pSM->uSectorPerBlock = 32;
                pSM->bIsMulticycle = FALSE;
                pSM->nPageSize = NAND_PAGE_512B;
                pSM->bIsNandECC4 = TRUE;
                pSM->bIsMLCNand = FALSE;

                NDISK_info->NAND_type = NAND_TYPE_SLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                NDISK_info->nZone = 1;              /* number of zones */
                NDISK_info->nBlockPerZone = 1024;   /* blocks per zone */
                NDISK_info->nPagePerBlock = 32;     /* pages per block */
                NDISK_info->nLBPerZone = 1000;      /* logical blocks per zone */
                NDISK_info->nPageSize = 512;
                break;

                /* page size 2KB */
        case 0xf1:  // 128M
        case 0xd1:
                pSM->uBlockPerFlash = 1023;
                pSM->uPagePerBlock = 64;
                pSM->uSectorPerBlock = 256;
                pSM->uSectorPerFlash = 255744;
                pSM->bIsMulticycle = FALSE;
                pSM->nPageSize = NAND_PAGE_2KB;
                pSM->bIsNandECC8 = TRUE;
                pSM->bIsMLCNand = FALSE;

                NDISK_info->NAND_type = NAND_TYPE_SLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                NDISK_info->nZone = 1;              /* number of zones */
                NDISK_info->nBlockPerZone = 1024;   /* blocks per zone */
                NDISK_info->nPagePerBlock = 64;     /* pages per block */
                NDISK_info->nLBPerZone = 1000;      /* logical blocks per zone */
                NDISK_info->nPageSize = 2048;

                // 2013/10/22, support MXIC MX30LF1G08AA NAND flash
                // 2015/06/22, support MXIC MX30LF1G18AC NAND flash
                if ( ((tempID[0]==0xC2)&&(tempID[1]==0xF1)&&(tempID[2]==0x80)&&(tempID[3]==0x1D)) ||
                     ((tempID[0]==0xC2)&&(tempID[1]==0xF1)&&(tempID[2]==0x80)&&(tempID[3]==0x95)&&(tempID[4]==0x02)) )
                {
                    // The first ID of this NAND is 0xC2 BUT it is NOT NAND ROM (read only)
                    // So, we MUST modify the configuration of it
                    //      1. change pSM->bIsCheckECC to TRUE to enable ECC feature;
                    //      2. assign a fake vendor_ID to make NVTFAT can write data to this NAND disk.
                    //         (GNAND will check vendor_ID and set disk to DISK_TYPE_READ_ONLY if it is 0xC2)
                    pSM->bIsCheckECC = TRUE;
                    NDISK_info->vendor_ID = 0xFF;   // fake vendor_ID
                }

                // 2014/10/16, support Winbond W29N01GV NAND flash
            // 2017/09/14, support Samsung K9F1G08U0B NAND flash
            // 2017/09/19, support Winbond W29N01HV NAND flash
            if (   ((tempID[0]==0xEF)&&(tempID[1]==0xF1)&&(tempID[2]==0x80)&&(tempID[3]==0x95))
                || ((tempID[0]==0xEC)&&(tempID[1]==0xF1)&&(tempID[2]==0x00)&&(tempID[3]==0x95))
                || ((tempID[0]==0xEF)&&(tempID[1]==0xF1)&&(tempID[2]==0x00)&&(tempID[3]==0x95))
               )
                {
                    NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                }
                break;

        case 0xda:  // 256M
                if ((tempID[3] & 0x33) == 0x11)
                {
                        pSM->uBlockPerFlash = 2047;
                        pSM->uPagePerBlock = 64;
                        pSM->uSectorPerBlock = 256;
                        pSM->bIsMLCNand = FALSE;

                        NDISK_info->NAND_type = NAND_TYPE_SLC;
                        NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                        NDISK_info->nZone = 1;              /* number of zones */
                        NDISK_info->nPagePerBlock = 64;     /* pages per block */
                        NDISK_info->nBlockPerZone = 2048;       /* blocks per zone */
                        NDISK_info->nLBPerZone = 2000;          /* logical blocks per zone */
                }
                else if ((tempID[3] & 0x33) == 0x21)
                {
                        pSM->uBlockPerFlash = 1023;
                        pSM->uPagePerBlock = 128;
                        pSM->uSectorPerBlock = 512;
                        pSM->bIsMLCNand = TRUE;

                        NDISK_info->NAND_type = NAND_TYPE_MLC;
                        NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                        NDISK_info->nZone = 1;              /* number of zones */
                        NDISK_info->nPagePerBlock = 128;    /* pages per block */
                        NDISK_info->nBlockPerZone = 1024;       /* blocks per zone */
                        NDISK_info->nLBPerZone = 1000;          /* logical blocks per zone */
                }
                pSM->uSectorPerFlash = 511488;
                pSM->bIsMulticycle = TRUE;
                pSM->nPageSize = NAND_PAGE_2KB;
                pSM->bIsNandECC8 = TRUE;

                // 2018/10/29, support MXIC MX30LF2G18AC NAND flash
                if ((tempID[0]==0xC2)&&(tempID[1]==0xDA)&&(tempID[2]==0x90)&&(tempID[3]==0x95)&&(tempID[4]==0x06))
                {
                    // The first ID of this NAND is 0xC2 BUT it is NOT NAND ROM (read only)
                    // So, we MUST modify the configuration of it
                    //      1. change pSM->bIsCheckECC to TRUE to enable ECC feature;
                    //      2. assign a fake vendor_ID to make NVTFAT can write data to this NAND disk.
                    //         (GNAND will check vendor_ID and set disk to DISK_TYPE_READ_ONLY if it is 0xC2)
                    pSM->bIsCheckECC = TRUE;
                    NDISK_info->vendor_ID = 0xFF;   // fake vendor_ID
                }

                NDISK_info->nPageSize = 2048;
                break;

        case 0xdc:  // 512M
                // 2017/9/19, To support both Maker Founder MP4G08JAA
                //                        and Toshiba TC58NVG2S0HTA00 512MB NAND flash
                if ((tempID[0]==0x98)&&(tempID[2]==0x90)&&(tempID[3]==0x26)&&(tempID[4]==0x76))
                {
                    pSM->uBlockPerFlash  = 2047;        // block index with 0-base. = physical blocks - 1
                    pSM->uPagePerBlock   = 64;
                    pSM->nPageSize       = NAND_PAGE_4KB;
                    pSM->uSectorPerBlock = pSM->nPageSize / 512 * pSM->uPagePerBlock;
                    pSM->bIsMLCNand      = FALSE;
                    pSM->bIsMulticycle   = TRUE;
                    pSM->bIsNandECC8     = TRUE;

                    NDISK_info->NAND_type     = (pSM->bIsMLCNand ? NAND_TYPE_MLC : NAND_TYPE_SLC);
                    NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                    NDISK_info->nZone         = 1;      // number of zones
                    NDISK_info->nBlockPerZone = pSM->uBlockPerFlash + 1;   // blocks per zone
                    NDISK_info->nPagePerBlock = pSM->uPagePerBlock;
                    NDISK_info->nPageSize     = pSM->nPageSize;
                    NDISK_info->nLBPerZone    = 4000;   // logical blocks per zone

                    pSM->uSectorPerFlash = pSM->uSectorPerBlock * NDISK_info->nLBPerZone / 1000 * 999;
                    break;
                }

                if ((tempID[3] & 0x33) == 0x11)
                {
                        pSM->uBlockPerFlash = 4095;
                        pSM->uPagePerBlock = 64;
                        pSM->uSectorPerBlock = 256;
                        pSM->bIsMLCNand = FALSE;

                        NDISK_info->NAND_type = NAND_TYPE_SLC;
                        NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                        NDISK_info->nZone = 1;              /* number of zones */
                        NDISK_info->nPagePerBlock = 64;     /* pages per block */
                        NDISK_info->nBlockPerZone = 4096;       /* blocks per zone */
                        NDISK_info->nLBPerZone = 4000;          /* logical blocks per zone */
                }
                else if ((tempID[3] & 0x33) == 0x21)
                {
                        pSM->uBlockPerFlash = 2047;
                        pSM->uPagePerBlock = 128;
                        pSM->uSectorPerBlock = 512;
                        pSM->bIsMLCNand = TRUE;

                        NDISK_info->NAND_type = NAND_TYPE_MLC;
                        NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                        NDISK_info->nZone = 1;              /* number of zones */
                        NDISK_info->nPagePerBlock = 128;    /* pages per block */
                        NDISK_info->nBlockPerZone = 2048;       /* blocks per zone */
                        NDISK_info->nLBPerZone = 2000;          /* logical blocks per zone */
                }
                pSM->uSectorPerFlash = 1022976;
                pSM->bIsMulticycle = TRUE;
                pSM->nPageSize = NAND_PAGE_2KB;
                pSM->bIsNandECC8 = TRUE;

                NDISK_info->nPageSize = 2048;

                // 2018/10/29, support MXIC MX30LF4G18AC NAND flash
                if ((tempID[0]==0xC2)&&(tempID[1]==0xDC)&&(tempID[2]==0x90)&&(tempID[3]==0x95)&&(tempID[4]==0x56))
                {
                    // The first ID of this NAND is 0xC2 BUT it is NOT NAND ROM (read only)
                    // So, we MUST modify the configuration of it
                    //      1. change pSM->bIsCheckECC to TRUE to enable ECC feature;
                    //      2. assign a fake vendor_ID to make NVTFAT can write data to this NAND disk.
                    //         (GNAND will check vendor_ID and set disk to DISK_TYPE_READ_ONLY if it is 0xC2)
                    pSM->bIsCheckECC = TRUE;
                    NDISK_info->vendor_ID = 0xFF;   // fake vendor_ID
                }
                break;

        case 0xd3:  // 1024M
                // 2014/4/2, To support Samsung K9WAG08U1D 512MB NAND flash
                if ((tempID[0]==0xEC)&&(tempID[2]==0x51)&&(tempID[3]==0x95)&&(tempID[4]==0x58))
                {
                    pSM->uBlockPerFlash  = 4095;        // block index with 0-base. = physical blocks - 1
                    pSM->uPagePerBlock   = 64;
                    pSM->nPageSize       = NAND_PAGE_2KB;
                    pSM->uSectorPerBlock = pSM->nPageSize / 512 * pSM->uPagePerBlock;
                    pSM->bIsMLCNand      = FALSE;
                    pSM->bIsMulticycle   = TRUE;
                    pSM->bIsNandECC8     = TRUE;
                    pSM->uSectorPerFlash = 1022976;

                    NDISK_info->NAND_type     = NAND_TYPE_MLC;
                    NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                    NDISK_info->nZone         = 1;      // number of zones
                    NDISK_info->nBlockPerZone = pSM->uBlockPerFlash + 1;   // blocks per zone
                    NDISK_info->nPagePerBlock = pSM->uPagePerBlock;
                    NDISK_info->nPageSize     = pSM->nPageSize;
                    NDISK_info->nLBPerZone    = 4000;   // logical blocks per zone
                    break;
                }

                // 2016/9/29, support MXIC MX60LF8G18AC NAND flash
                if ((tempID[0]==0xC2)&&(tempID[1]==0xD3)&&(tempID[2]==0xD1)&&(tempID[3]==0x95)&&(tempID[4]==0x5A))
                {
                    // The first ID of this NAND is 0xC2 BUT it is NOT NAND ROM (read only)
                    // So, we MUST modify the configuration of it
                    //      1. change pSM->bIsCheckECC to TRUE to enable ECC feature;
                    //      2. assign a fake vendor_ID to make NVTFAT can write data to this NAND disk.
                    //         (GNAND will check vendor_ID and set disk to DISK_TYPE_READ_ONLY if it is 0xC2)
                    pSM->bIsCheckECC = TRUE;
                    NDISK_info->vendor_ID = 0xFF;   // fake vendor_ID
                }

                if ((tempID[3] & 0x33) == 0x32)
                {
                        pSM->uBlockPerFlash = 2047;
                        pSM->uPagePerBlock = 128;
                        pSM->uSectorPerBlock = 1024;    /* 128x8 */
                        pSM->nPageSize = NAND_PAGE_4KB;
                        pSM->bIsMLCNand = TRUE;

                        NDISK_info->NAND_type = NAND_TYPE_MLC;
                        NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                        NDISK_info->nZone = 1;              /* number of zones */
                        NDISK_info->nPagePerBlock = 128;    /* pages per block */
                        NDISK_info->nPageSize = 4096;
                        NDISK_info->nBlockPerZone = 2048;       /* blocks per zone */
                        NDISK_info->nLBPerZone = 2000;          /* logical blocks per zone */
                }
                else if ((tempID[3] & 0x33) == 0x11)
                {
                        pSM->uBlockPerFlash = 8191;
                        pSM->uPagePerBlock = 64;
                        pSM->uSectorPerBlock = 256;
                        pSM->nPageSize = NAND_PAGE_2KB;
                        pSM->bIsMLCNand = FALSE;

                        NDISK_info->NAND_type = NAND_TYPE_SLC;
                        NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                        NDISK_info->nZone = 1;              /* number of zones */
                        NDISK_info->nPagePerBlock = 64;     /* pages per block */
                        NDISK_info->nPageSize = 2048;
                        NDISK_info->nBlockPerZone = 8192;       /* blocks per zone */
                        NDISK_info->nLBPerZone = 8000;          /* logical blocks per zone */
                }
                else if ((tempID[3] & 0x33) == 0x21)
                {
                        pSM->uBlockPerFlash = 4095;
                        pSM->uPagePerBlock = 128;
                        pSM->uSectorPerBlock = 512;
                        pSM->nPageSize = NAND_PAGE_2KB;
                        pSM->bIsMLCNand = TRUE;

                        NDISK_info->NAND_type = NAND_TYPE_MLC;
                        NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                        NDISK_info->nZone = 1;              /* number of zones */
                        NDISK_info->nPagePerBlock = 128;    /* pages per block */
                        NDISK_info->nPageSize = 2048;
                        NDISK_info->nBlockPerZone = 4096;       /* blocks per zone */
                        NDISK_info->nLBPerZone = 4000;          /* logical blocks per zone */
                }
                else if ((tempID[3] & 0x33) == 0x22)
                {
                        pSM->uBlockPerFlash = 4095;
                        pSM->uPagePerBlock = 64;
                        pSM->uSectorPerBlock = 512;
                        pSM->nPageSize = NAND_PAGE_4KB;
                        pSM->bIsMLCNand = FALSE;

                        NDISK_info->NAND_type = NAND_TYPE_SLC;
                        NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                        NDISK_info->nZone = 1;              /* number of zones */
                        NDISK_info->nPagePerBlock = 64;     /* pages per block */
                        NDISK_info->nPageSize = 4096;
                        NDISK_info->nBlockPerZone = 4096;   /* blocks per zone */
                        NDISK_info->nLBPerZone = 4000;      /* logical blocks per zone */
                }

                pSM->uSectorPerFlash = 2045952;
                pSM->bIsMulticycle = TRUE;
                pSM->bIsNandECC8 = TRUE;
                break;

        case 0xd5:  // 2048M
    #ifdef OPT_SUPPORT_H27UAG8T2A
            if ((tempID[0]==0xAD)&&(tempID[2] == 0x94)&&(tempID[3] == 0x25))
            {
                pSM->uBlockPerFlash = 4095;
                pSM->uPagePerBlock = 128;
                pSM->uSectorPerBlock = 1024;
                pSM->nPageSize = NAND_PAGE_4KB;
                pSM->bIsMLCNand = TRUE;

                NDISK_info->NAND_type = NAND_TYPE_MLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                NDISK_info->nZone = 1;              /* number of zones */
                NDISK_info->nPagePerBlock = 128;    /* pages per block */
                NDISK_info->nPageSize = 4096;
                NDISK_info->nBlockPerZone = 4096;   /* blocks per zone */
                NDISK_info->nLBPerZone = 4000;      /* logical blocks per zone */

                pSM->uSectorPerFlash = 4091904;
                pSM->bIsMulticycle = TRUE;
                pSM->bIsNandECC12 = TRUE;
                break;
            }
            else
            {
                if ((tempID[3] & 0x33) == 0x32)
                {
                    pSM->uBlockPerFlash = 4095;
                    pSM->uPagePerBlock = 128;
                    pSM->uSectorPerBlock = 1024;
                    pSM->nPageSize = NAND_PAGE_4KB;
                    pSM->bIsMLCNand = TRUE;

                    NDISK_info->NAND_type = NAND_TYPE_MLC;
                    NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                    NDISK_info->nZone = 1;              /* number of zones */
                    NDISK_info->nPagePerBlock = 128;    /* pages per block */
                    NDISK_info->nPageSize = 4096;
                    NDISK_info->nBlockPerZone = 4096;   /* blocks per zone */
                    NDISK_info->nLBPerZone = 4000;      /* logical blocks per zone */
                }
                else if ((tempID[3] & 0x33) == 0x11)
                {
                    pSM->uBlockPerFlash = 16383;
                    pSM->uPagePerBlock = 64;
                    pSM->uSectorPerBlock = 256;
                    pSM->nPageSize = NAND_PAGE_2KB;
                    pSM->bIsMLCNand = FALSE;

                    NDISK_info->NAND_type = NAND_TYPE_SLC;
                    NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                    NDISK_info->nZone = 1;              /* number of zones */
                    NDISK_info->nPagePerBlock = 64;     /* pages per block */
                    NDISK_info->nPageSize = 2048;
                    NDISK_info->nBlockPerZone = 16384;  /* blocks per zone */
                    NDISK_info->nLBPerZone = 16000;     /* logical blocks per zone */
                }
                else if ((tempID[3] & 0x33) == 0x21)
                {
                    pSM->uBlockPerFlash = 8191;
                    pSM->uPagePerBlock = 128;
                    pSM->uSectorPerBlock = 512;
                    pSM->nPageSize = NAND_PAGE_2KB;
                    pSM->bIsMLCNand = TRUE;

                    NDISK_info->NAND_type = NAND_TYPE_MLC;
                    NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                    NDISK_info->nZone = 1;              /* number of zones */
                    NDISK_info->nPagePerBlock = 128;    /* pages per block */
                    NDISK_info->nPageSize = 2048;
                    NDISK_info->nBlockPerZone = 8192;   /* blocks per zone */
                    NDISK_info->nLBPerZone = 8000;      /* logical blocks per zone */
                }

                pSM->uSectorPerFlash = 4091904;
                pSM->bIsMulticycle = TRUE;
                pSM->bIsNandECC8 = TRUE;
                break;
            }
    #else
            if ((tempID[3] & 0x33) == 0x32)
            {
                pSM->uBlockPerFlash = 4095;
                pSM->uPagePerBlock = 128;
                pSM->uSectorPerBlock = 1024;
                pSM->nPageSize = NAND_PAGE_4KB;
                pSM->bIsMLCNand = TRUE;

                NDISK_info->NAND_type = NAND_TYPE_MLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                NDISK_info->nZone = 1;              /* number of zones */
                NDISK_info->nPagePerBlock = 128;    /* pages per block */
                NDISK_info->nPageSize = 4096;
                NDISK_info->nBlockPerZone = 4096;   /* blocks per zone */
                NDISK_info->nLBPerZone = 4000;      /* logical blocks per zone */
            }
            else if ((tempID[3] & 0x33) == 0x11)
            {
                pSM->uBlockPerFlash = 16383;
                pSM->uPagePerBlock = 64;
                pSM->uSectorPerBlock = 256;
                pSM->nPageSize = NAND_PAGE_2KB;
                pSM->bIsMLCNand = FALSE;

                NDISK_info->NAND_type = NAND_TYPE_SLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                NDISK_info->nZone = 1;              /* number of zones */
                NDISK_info->nPagePerBlock = 64;     /* pages per block */
                NDISK_info->nPageSize = 2048;
                NDISK_info->nBlockPerZone = 16384;  /* blocks per zone */
                NDISK_info->nLBPerZone = 16000;     /* logical blocks per zone */
            }
            else if ((tempID[3] & 0x33) == 0x21)
            {
                pSM->uBlockPerFlash = 8191;
                pSM->uPagePerBlock = 128;
                pSM->uSectorPerBlock = 512;
                pSM->nPageSize = NAND_PAGE_2KB;
                pSM->bIsMLCNand = TRUE;

                NDISK_info->NAND_type = NAND_TYPE_MLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_IN_SEQ;
                NDISK_info->nZone = 1;              /* number of zones */
                NDISK_info->nPagePerBlock = 128;    /* pages per block */
                NDISK_info->nPageSize = 2048;
                NDISK_info->nBlockPerZone = 8192;   /* blocks per zone */
                NDISK_info->nLBPerZone = 8000;      /* logical blocks per zone */
            }

            pSM->uSectorPerFlash = 4091904;
            pSM->bIsMulticycle = TRUE;
            pSM->bIsNandECC8 = TRUE;
            break;
    #endif

        default:
            // 2013/9/25, support MXIC MX30LF1208AA NAND flash
            if ((tempID[0]==0xC2)&&(tempID[1]==0xF0)&&(tempID[2]==0x80)&&(tempID[3]==0x1D))
            {
                pSM->uBlockPerFlash  = 511;         // block index with 0-base. = physical blocks - 1
                pSM->uPagePerBlock   = 64;
                pSM->nPageSize       = NAND_PAGE_2KB;
                pSM->uSectorPerBlock = pSM->nPageSize / 512 * pSM->uPagePerBlock;
                pSM->bIsMLCNand      = FALSE;
                pSM->bIsMulticycle   = FALSE;
                pSM->bIsNandECC8     = TRUE;

                NDISK_info->NAND_type     = NAND_TYPE_SLC;
                NDISK_info->write_page_in_seq = NAND_TYPE_PAGE_OUT_SEQ;
                NDISK_info->nZone         = 1;      // number of zones
                NDISK_info->nBlockPerZone = pSM->uBlockPerFlash + 1;    // blocks per zone
                NDISK_info->nPagePerBlock = pSM->uPagePerBlock;
                NDISK_info->nPageSize     = pSM->nPageSize;
                // why nLBPerZone is not 512 ? sicSMInit() had reserved %2 blocks for bad block base on this value.
                NDISK_info->nLBPerZone    = 500;    // logical blocks per zone

                pSM->uSectorPerFlash = pSM->uSectorPerBlock * NDISK_info->nLBPerZone / 1000 * 999;

                // The first ID of this NAND is 0xC2 BUT it is NOT NAND ROM (read only)
                // So, we MUST modify the configuration of it
                //      1. change pSM->bIsCheckECC to TRUE to enable ECC feature;
                //      2. assign a fake vendor_ID to make NVTFAT can write data to this NAND disk.
                //         (GNAND will check vendor_ID and set disk to DISK_TYPE_READ_ONLY if it is 0xC2)
                pSM->bIsCheckECC = TRUE;
                NDISK_info->vendor_ID = 0xFF;   // fake vendor_ID

                break;
            }

            printk("NAND ERROR: SM ID not support!! %02X-%02X-%02X-%02X\n", tempID[0], tempID[1], tempID[2], tempID[3]);
            LEAVE();
                return FMI_SM_ID_ERR;
        }

        printk("NAND: Found %s NAND, ID %02X-%02X-%02X-%02X, page size %d, BCH T%d\n",
            pSM->bIsMLCNand ? "MLC" : "SLC",
            tempID[0], tempID[1], tempID[2], tempID[3],
            pSM->nPageSize,
            pSM->bIsNandECC4*4 + pSM->bIsNandECC8*8 + pSM->bIsNandECC12*12 + pSM->bIsNandECC15*15
            );
        LEAVE();
        return 0;
}


int fmiSM2BufferM(FMI_SM_INFO_T *pSM, u32 uSector, u8 ucColAddr)
{
        ENTER();
        /* clear R/B flag */
#if defined(CONFIG_W55FA93_TWO_RB_PINS) || defined(CONFIG_W55FA93_NAND1)
        if(pSM == pSM0)
        {
            while (!(inpw(REG_SMISR) & SMISR_RB0));
            outpw(REG_SMISR, SMISR_RB0_IF);
        }
        else
        {
            while (!(inpw(REG_SMISR) & SMISR_RB1));
            outpw(REG_SMISR, SMISR_RB1_IF);
        }
#else
        while (!(inpw(REG_SMISR) & SMISR_RB0));
        outpw(REG_SMISR, SMISR_RB0_IF);
#endif

        outpw(REG_SMCMD, 0x00);     // read command
        outpw(REG_SMADDR, ucColAddr);   // CA0 - CA7
        outpw(REG_SMADDR, uSector & 0xff);  // PA0 - PA7
        if (!pSM->bIsMulticycle)
                outpw(REG_SMADDR, ((uSector >> 8) & 0xff)|EOA_SM);      // PA8 - PA15
        else {
                outpw(REG_SMADDR, (uSector >> 8) & 0xff);       // PA8 - PA15
                outpw(REG_SMADDR, ((uSector >> 16) & 0xff)|EOA_SM);     // PA16 - PA17
        }

        if (!fmiSMCheckRB(pSM)) {
                return FMI_SM_RB_ERR;
        }
        LEAVE();
        return 0;
}


int fmiSM2BufferM_RA(FMI_SM_INFO_T *pSM, u32 uSector, u8 ucColAddr)
{
        ENTER();
        /* 2011/03/04, to support NAND ROM chip "Infinite IM90A001GT" */
        if (! pSM->bIsCheckECC)
            return 0;
        /* clear R/B flag */
#if defined(CONFIG_W55FA93_TWO_RB_PINS) || defined(CONFIG_W55FA93_NAND1)
        if(pSM == pSM0)
        {
            while (!(inpw(REG_SMISR) & SMISR_RB0));
            outpw(REG_SMISR, SMISR_RB0_IF);
        }
        else
        {
            while (!(inpw(REG_SMISR) & SMISR_RB1));
            outpw(REG_SMISR, SMISR_RB1_IF);
        }
#else

        while (!(inpw(REG_SMISR) & SMISR_RB0));
        outpw(REG_SMISR, SMISR_RB0_IF);
#endif

        outpw(REG_SMCMD, 0x50);     // read command
        outpw(REG_SMADDR, ucColAddr);   // CA0 - CA7
        outpw(REG_SMADDR, uSector & 0xff);  // PA0 - PA7
        if (!pSM->bIsMulticycle)
                outpw(REG_SMADDR, ((uSector >> 8) & 0xff)|EOA_SM);      // PA8 - PA15
        else {
                outpw(REG_SMADDR, (uSector >> 8) & 0xff);       // PA8 - PA15
                outpw(REG_SMADDR, ((uSector >> 16) & 0xff)|EOA_SM);     // PA16 - PA17
        }

        if (!fmiSMCheckRB(pSM))
                return FMI_SM_RB_ERR;

        LEAVE();
        return 0;
}


void fmiSM_CorrectData_BCH(u8 ucFieidIndex, u8 ucErrorCnt, u8* pDAddr)
{
    u32 uaData[16], uaAddr[16];
    u32 uaErrorData[4];
    u8  ii, jj;
    u32 uPageSize, spareSize;

    ENTER();

    uPageSize = inpw(REG_SMCSR) & SMCR_PSIZE;

    jj = ucErrorCnt/4;
    jj ++;
    if (jj > 4) jj = 4;

    for(ii=0; ii<jj; ii++)
    {
        uaErrorData[ii] = inpw(REG_BCH_ECC_DATA0 + ii*4);
    }

    for(ii=0; ii<jj; ii++)
    {
        uaData[ii*4+0] = uaErrorData[ii] & 0xff;
        uaData[ii*4+1] = (uaErrorData[ii]>>8) & 0xff;
        uaData[ii*4+2] = (uaErrorData[ii]>>16) & 0xff;
        uaData[ii*4+3] = (uaErrorData[ii]>>24) & 0xff;
    }

    jj = ucErrorCnt/2;
    jj ++;
    if (jj > 8) jj = 8;

    for(ii=0; ii<jj; ii++)
    {
        uaAddr[ii*2+0] = inpw(REG_BCH_ECC_ADDR0 + ii*4) & 0x1fff;
        uaAddr[ii*2+1] = (inpw(REG_BCH_ECC_ADDR0 + ii*4)>>16) & 0x1fff;
    }

    pDAddr += (ucFieidIndex-1)*0x200;

    for(ii=0; ii<ucErrorCnt; ii++)
    {
        if (uPageSize == PSIZE_8K)
        {
            switch(inpw(REG_SMCSR) & SMCR_BCH_TSEL)
            {
                case BCH_T4:    // 8K+256
                default:
                    if (uaAddr[ii] < 512)
                        *(pDAddr+uaAddr[ii]) ^=  uaData[ii];
                    else
                    {
                        if (uaAddr[ii] < 515)
                        {
                            uaAddr[ii] -= 512;
                            uaAddr[ii] += 8*(ucFieidIndex-1);   // field offset, only Field-0
                            *((u8*)REG_SMRA_0+uaAddr[ii]) ^=  uaData[ii];
                        }
                        else
                        {
                            uaAddr[ii] = 543 - uaAddr[ii];
                            uaAddr[ii] = 7 - uaAddr[ii];
                            uaAddr[ii] += 8*(ucFieidIndex-1);   // field offset
                            *((u8*)REG_SMRA_32+uaAddr[ii]) ^=  uaData[ii];
                        }
                    }
                    break;

                case BCH_T8:    // 8K+256
                    if (uaAddr[ii] < 512)
                        *(pDAddr+uaAddr[ii]) ^=  uaData[ii];
                    else
                    {
                        if (uaAddr[ii] < 515)
                        {
                            uaAddr[ii] -= 512;
                            uaAddr[ii] += 15*(ucFieidIndex-1);  // field offset
                            *((u8*)REG_SMRA_0+uaAddr[ii]) ^=  uaData[ii];
                        }
                        else
                        {
                            uaAddr[ii] = 543 - uaAddr[ii];
                            uaAddr[ii] = 14 - uaAddr[ii];
                            uaAddr[ii] += 15*(ucFieidIndex-1);  // field offset
                            *((u8*)REG_SMRA_4+uaAddr[ii]) ^=  uaData[ii];
                        }
                    }
                    break;

                case BCH_T12:   // 8K+376
                    if (uaAddr[ii] < 512)
                        *(pDAddr+uaAddr[ii]) ^=  uaData[ii];
                    else
                    {
                        if (uaAddr[ii] < 515)
                        {
                            uaAddr[ii] -= 512;
                            uaAddr[ii] += 23*(ucFieidIndex-1);  // field offset
                            *((u8*)REG_SMRA_0+uaAddr[ii]) ^=  uaData[ii];
                        }
                        else
                        {
                            uaAddr[ii] = 543 - uaAddr[ii];
                            uaAddr[ii] = 22 - uaAddr[ii];
                            uaAddr[ii] += 23*(ucFieidIndex-1);  // field offset
                            *((u8*)REG_SMRA_2+uaAddr[ii]) ^=  uaData[ii];
                        }
                    }
                    break;

                case BCH_T15:
                    break;
            }
        }
        else if (uPageSize == PSIZE_4K)
        {
            switch(inpw(REG_SMCSR) & SMCR_BCH_TSEL)
            {
                case BCH_T4:    // 4K+128
                default:
                    if (uaAddr[ii] < 512)
                        *(pDAddr+uaAddr[ii]) ^=  uaData[ii];
                    else
                    {
                        if (uaAddr[ii] < 515)
                        {
                            uaAddr[ii] -= 512;
                            uaAddr[ii] += 8*(ucFieidIndex-1);   // field offset, only Field-0
                            *((u8*)REG_SMRA_0+uaAddr[ii]) ^=  uaData[ii];
                        }
                        else
                        {
                            uaAddr[ii] = 543 - uaAddr[ii];
                            uaAddr[ii] = 7 - uaAddr[ii];
                            uaAddr[ii] += 8*(ucFieidIndex-1);   // field offset
                            *((u8*)REG_SMRA_16+uaAddr[ii]) ^=  uaData[ii];
                        }
                    }
                    break;

                case BCH_T8:    // 4K+128
                    if (uaAddr[ii] < 512)
                        *(pDAddr+uaAddr[ii]) ^=  uaData[ii];
                    else
                    {
                        if (uaAddr[ii] < 515)
                        {
                            uaAddr[ii] -= 512;
                            uaAddr[ii] += 15*(ucFieidIndex-1);  // field offset
                            *((u8*)REG_SMRA_0+uaAddr[ii]) ^=  uaData[ii];
                        }
                        else
                        {
                            uaAddr[ii] = 543 - uaAddr[ii];
                            uaAddr[ii] = 14 - uaAddr[ii];
                            uaAddr[ii] += 15*(ucFieidIndex-1);  // field offset
                            *((u8*)REG_SMRA_2+uaAddr[ii]) ^=  uaData[ii];
                        }
                    }

                    break;

                case BCH_T12:   // 4K+216
                    if (uaAddr[ii] < 512)
                        *(pDAddr+uaAddr[ii]) ^=  uaData[ii];
                    else
                    {
                        if (uaAddr[ii] < 515)
                        {
                            uaAddr[ii] -= 512;
                            uaAddr[ii] += 23*(ucFieidIndex-1);  // field offset
                            *((u8*)REG_SMRA_0+uaAddr[ii]) ^=  uaData[ii];
                        }
                        else
                        {
                            uaAddr[ii] = 543 - uaAddr[ii];
                            uaAddr[ii] = 22 - uaAddr[ii];
                            uaAddr[ii] += 23*(ucFieidIndex-1);  // field offset

                    #ifdef OPT_SUPPORT_H27UAG8T2A
                            // 4K+224
                            spareSize = inpw(REG_SMREAREA_CTL) & SMRE_REA128_EXT;
                             *((u8*)REG_SMRA_0+(spareSize-184)+uaAddr[ii]) ^=  uaData[ii];
                    #else
                            // 4K+216
                            *((u8*)REG_SMRA_8+uaAddr[ii]) ^=  uaData[ii];
                    #endif
                        }
                    }
                    break;
            }
        }
        else if (uPageSize == PSIZE_2K)
        {
            switch(inpw(REG_SMCSR) & SMCR_BCH_TSEL)
            {
                case BCH_T4:
                default:
                    if (uaAddr[ii] < 512)
                        *(pDAddr+uaAddr[ii]) ^=  uaData[ii];
                    else
                    {
                        if (uaAddr[ii] < 515)
                        {
                            uaAddr[ii] -= 512;
                            uaAddr[ii] += 8*(ucFieidIndex-1);   // field offset, only Field-0
                            *((u8*)REG_SMRA_0+uaAddr[ii]) ^=  uaData[ii];
                        }
                        else
                        {
                            uaAddr[ii] = 543 - uaAddr[ii];
                            uaAddr[ii] = 7 - uaAddr[ii];
                            uaAddr[ii] += 8*(ucFieidIndex-1);   // field offset
                            *((u8*)REG_SMRA_8+uaAddr[ii]) ^=  uaData[ii];
                        }
                    }
                    break;

                case BCH_T8:
                    if (uaAddr[ii] < 512)
                        *(pDAddr+uaAddr[ii]) ^=  uaData[ii];
                    else
                    {
                        if (uaAddr[ii] < 515)
                        {
                            uaAddr[ii] -= 512;
                            uaAddr[ii] += 15*(ucFieidIndex-1);  // field offset
                            *((u8*)REG_SMRA_0+uaAddr[ii]) ^=  uaData[ii];
                        }
                        else
                        {
                            uaAddr[ii] = 543 - uaAddr[ii];
                            uaAddr[ii] = 14 - uaAddr[ii];
                            uaAddr[ii] += 15*(ucFieidIndex-1);  // field offset
                            *((u8*)REG_SMRA_1+uaAddr[ii]) ^=  uaData[ii];
                        }
                        }
                    break;
            }
        }
        else
        {
            if (uaAddr[ii] < 512)
                *(pDAddr+uaAddr[ii]) ^=  uaData[ii];
            else
            {
                if (uaAddr[ii] < 515)
                {
                    uaAddr[ii] -= 512;
                    *((u8*)REG_SMRA_0+uaAddr[ii]) ^=  uaData[ii];
                }
                else
                {
                    uaAddr[ii] = 543 - uaAddr[ii];
                    uaAddr[ii] = 7 - uaAddr[ii];
                    *((u8*)REG_SMRA_2+uaAddr[ii]) ^=  uaData[ii];
                }
            }
        }
    }
    LEAVE();
}

int fmiSM_Read_512(FMI_SM_INFO_T *pSM, u32 uSector, u32 uDAddr)
{
        int volatile ret=0;
        u32 uStatus;
        u32 uErrorCnt;

        ENTER();

        if (down_interruptible(&dmac_sem)) {
                //printk("io err\n");
                return(GNERR_IO_ERR);
        }
        while (inpw(REG_DMACCSR)&FMI_BUSY); //Wait IP finished... for safe

        outpw(REG_DMACSAR, _fmi_ucNANDBuffer);

        _fmi_bIsSMDataReady = FALSE;
        ret = fmiSM2BufferM(pSM, uSector, 0);
        if (ret < 0) {
                printk("read error 0x%x\n", ret);
                up(&dmac_sem);
                return ret;
        }

        outpw(REG_SMISR, SMISR_DMA_IF);                 // clear DMA flag
        outpw(REG_SMISR, SMISR_ECC_FIELD_IF);           // clear ECC_FIELD flag

        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_DRD_EN);

        while (!_fmi_bIsSMDataReady);

        if (pSM->bIsCheckECC)
        {
            while(1)
            {
                if (inpw(REG_SMISR) & SMISR_ECC_FIELD_IF)
                {
                    if (inpw(REG_SMCSR) & BCH_T4)   // BCH_ECC selected
                    {
                        uStatus = inpw(REG_SM_ECC_ST0);
                        uStatus &= 0x3f;

                        if ((uStatus & 0x03)==0x01)         // correctable error in 1st field
                        {
                            uErrorCnt = uStatus >> 2;
                            fmiSM_CorrectData_BCH(1, uErrorCnt, (u8*)uDAddr);

                    #ifdef DEBUG
                            printk("Field 1 have %d error!!\n", uErrorCnt);
                    #endif
                        }
                        else if (((uStatus & 0x03)==0x02)
                              ||((uStatus & 0x03)==0x03))   // uncorrectable error or ECC error
                        {
                    #ifdef DEBUG
                            printk("SM uncorrectable error is encountered, %4x !!\n", uStatus);
                    #endif
                            outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
                            up(&dmac_sem);
                            outpw(REG_DMACCSR, DMAC_EN+DMAC_SWRST);         // reset DMAC
                            outpw(REG_SMCSR, inpw(REG_SMCSR)|SMCR_SM_SWRST);// reset SM controller
                            return FMI_SM_ECC_ERROR;
                        }
                    }

                    else
                    {
                    #ifdef DEBUG
                        printk("Wrong BCH setting for page-512 NAND !!\n");
                    #endif
                    }

                    outpw(REG_SMISR, SMISR_ECC_FIELD_IF);       // clear ECC_FLD_Error
                }
            #if 1
                if ( !(inpw(REG_SMISR) & SMISR_ECC_FIELD_IF) )
                    break;
            #else
                if (inpw(REG_SMISR) & SMISR_DMA_IF)      // wait to finish DMAC transfer.
                {
                    printk("regiater SM_ISR = %4x \n ", inpw(REG_SMISR));
                    if ( !(inpw(REG_SMISR) & SMISR_ECC_FIELD_IF) )
                    {
                        break;
                    }
                }
            #endif
            }
        }
        else
            outpw(REG_SMISR, SMISR_ECC_FIELD_IF);

        memcpy((char *)uDAddr, (char *)_fmi_pNANDBuffer, 512);
        outpw(REG_SMISR, SMISR_DMA_IF);                 // clear DMA flag

        up(&dmac_sem);
        LEAVE();
        return 0;
}


void fmiBuffer2SMM(FMI_SM_INFO_T *pSM, u32 uSector, u8 ucColAddr)
{
        ENTER();

        if (write_nandloader)
            // add "check marker" (0xFF5Axx00, xx is page number) in spare area for NandLoader
            outpw(REG_SMRA_0, 0x00005AFF | (((uSector % pSM->uPagePerBlock) & 0xFF) << 16));
        else
            // set the spare area configuration
            /* write byte 514, 515 as used page */
            outpw(REG_SMRA_0, 0x0000FFFF);

        outpw(REG_SMRA_1, 0xFFFFFFFF);

        // send command
        outpw(REG_SMCMD, 0x80);     // serial data input command
        outpw(REG_SMADDR, ucColAddr);   // CA0 - CA7
        outpw(REG_SMADDR, uSector & 0xff);  // PA0 - PA7
        if (!pSM->bIsMulticycle)
                outpw(REG_SMADDR, ((uSector >> 8) & 0xff)|EOA_SM);      // PA8 - PA15
        else {
                outpw(REG_SMADDR, (uSector >> 8) & 0xff);       // PA8 - PA15
                outpw(REG_SMADDR, ((uSector >> 16) & 0xff)|EOA_SM);     // PA16 - PA17
        }
}


int fmiSM_Write_512(FMI_SM_INFO_T *pSM, u32 uSector, u32 uSAddr)
{
//  printk("fmiSM_Write_512!!\n");
        ENTER();
        /* clear R/B flag */
#if defined(CONFIG_W55FA93_TWO_RB_PINS) || defined(CONFIG_W55FA93_NAND1)
        if(pSM == pSM0)
        {
            while (!(inpw(REG_SMISR) & SMISR_RB0));
            outpw(REG_SMISR, SMISR_RB0_IF);
        }
        else
        {
            while (!(inpw(REG_SMISR) & SMISR_RB1));
            outpw(REG_SMISR, SMISR_RB1_IF);
        }
#else

        while (!(inpw(REG_SMISR) & SMISR_RB0));
        outpw(REG_SMISR, SMISR_RB0_IF);
#endif

        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_REDUN_AUTO_WEN);

        memcpy((char *)_fmi_pNANDBuffer, (char *)uSAddr, 512);

        if (down_interruptible(&dmac_sem))
                return(GNERR_IO_ERR);
        while (inpw(REG_DMACCSR)&FMI_BUSY); //Wait IP finished... for safe

        _fmi_bIsSMDataReady = FALSE;
        outpw(REG_DMACSAR, _fmi_ucNANDBuffer);

        fmiBuffer2SMM(pSM, uSector, 0);

        outpw(REG_SMISR, SMISR_DMA_IF);                 // clear DMA flag
        outpw(REG_SMISR, SMISR_ECC_FIELD_IF);           // clear ECC_FIELD flag
        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_DWR_EN);
        while (!_fmi_bIsSMDataReady);

        outpw(REG_SMISR, SMISR_DMA_IF);     // clear DMA flag
        outpw(REG_SMCMD, 0x10);     // auto program command

        if (!fmiSMCheckRB(pSM)) {
                up(&dmac_sem);
                return FMI_SM_RB_ERR;
        }


        if (fmiSMCheckStatus(pSM) != 0) {
                up(&dmac_sem);
                return FMI_SM_STATE_ERROR;
        }
        up(&dmac_sem);
        LEAVE();
        return 0;
}

int fmiSM_Read_2K(FMI_SM_INFO_T *pSM, u32 uPage, u32 uDAddr)
{
        u32 uStatus;
        u32 uErrorCnt, ii;

        ENTER();
//  printk("fmiSM_Read_2K uPage=%d   uDAddr=%x\n",uPage,uDAddr);

        if (down_interruptible(&dmac_sem))
        {
                return(GNERR_IO_ERR);
        }

        while (inpw(REG_DMACCSR)&FMI_BUSY); //Wait IP finished... for safe

        outpw(REG_DMACCSR, inpw(REG_DMACCSR) | DMAC_EN);
    //  outpw(REG_DMACSAR, uDAddr); // set DMA transfer starting address
        outpw(REG_DMACSAR, _fmi_ucNANDBuffer);

        /* clear R/B flag */
#if defined(CONFIG_W55FA93_TWO_RB_PINS) || defined(CONFIG_W55FA93_NAND1)
        if(pSM == pSM0)
        {
            while (!(inpw(REG_SMISR) & SMISR_RB0));
            outpw(REG_SMISR, SMISR_RB0_IF);
        }
        else
        {
            while (!(inpw(REG_SMISR) & SMISR_RB1));
            outpw(REG_SMISR, SMISR_RB1_IF);
        }
#else

        while (!(inpw(REG_SMISR) & SMISR_RB0));
        outpw(REG_SMISR, SMISR_RB0_IF);
#endif

        if (inpw(REG_SMISR) & SMISR_ECC_FIELD_IF)   /* ECC_FLD_IF */
        {
            printk("read: ECC error!!\n");
            outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
        }

        outpw(REG_SMCMD, 0x00);             // read command
        outpw(REG_SMADDR, 0);               // CA0 - CA7
        outpw(REG_SMADDR, 0);               // CA8 - CA11
        outpw(REG_SMADDR, uPage & 0xff);    // PA0 - PA7
        if (!pSM->bIsMulticycle)
                outpw(REG_SMADDR, ((uPage >> 8) & 0xff)|EOA_SM);        // PA8 - PA15
        else {
                outpw(REG_SMADDR, (uPage >> 8) & 0xff);                 // PA8 - PA15
                outpw(REG_SMADDR, ((uPage >> 16) & 0xff)|EOA_SM);       // PA16 - PA17
        }
        outpw(REG_SMCMD, 0x30);     // read command

        if (!fmiSMCheckRB(pSM)) {

                up(&dmac_sem);
                return FMI_SM_RB_ERR;
        }
        _fmi_bIsSMDataReady = FALSE;

        outpw(REG_SMISR, SMISR_DMA_IF);                 // clear DMA flag
        outpw(REG_SMISR, SMISR_ECC_FIELD_IF);           // clear ECC_FIELD flag
        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_DRD_EN);

        if (pSM->bIsCheckECC)
        {
            while(1)
            {
                if (inpw(REG_SMISR) & SMISR_ECC_FIELD_IF)
                {
                    uStatus = inpw(REG_SM_ECC_ST0);
                    for (ii=1; ii<5; ii++)
                    {
                        if ((uStatus & 0x03)==0x01)  // correctable error in 1st field
                        {
                            uErrorCnt = uStatus >> 2;
                            fmiSM_CorrectData_BCH(ii, uErrorCnt, (u8*)_fmi_pNANDBuffer);
                    #ifdef DEBUG
                            printk("Field %d have %d error!!\n", ii, uErrorCnt);
                    #endif
                            break;
                        }
                        else if (((uStatus & 0x03)==0x02)
                              ||((uStatus & 0x03)==0x03)) // uncorrectable error or ECC error in 1st field
                        {
                            #if 0
                                printk("register REG_DMACCSR = %4x \n ", inpw(REG_DMACCSR));
                                printk("register REG_DMACSAR = %4x \n ", inpw(REG_DMACSAR));
                                printk("register REG_DMACBCR = %4x \n ", inpw(REG_DMACBCR));
                                printk("register REG_DMACIER = %4x \n ", inpw(REG_DMACIER));
                                printk("register REG_DMACISR = %4x \n ", inpw(REG_DMACISR));
                                printk("register REG_FMICR = %4x \n ", inpw(REG_FMICR));
                                printk("register REG_FMIIER = %4x \n ", inpw(REG_FMIIER));
                                printk("register REG_FMIISR = %4x \n ", inpw(REG_FMIISR));
                                printk("register REG_SMCSR = %4x \n ", inpw(REG_SMCSR));
                                printk("register REG_SMTCR = %4x \n ", inpw(REG_SMTCR));
                                printk("register REG_SMIER = %4x \n ", inpw(REG_SMIER));
                                printk("register REG_SMISR = %4x \n ", inpw(REG_SMISR));

                                    printk("physical PageNo = 0x%4x \n ", uPage);
                                    printk("SMRA contents: \n ");
                                        for(ii=0; ii<4; ii+=0x10)
                                        {
                                            for(jj=0; jj<0x10; jj++)
                                            {
                                                printk("0x%2x ", __raw_readl(REG_SMRA_0+ii*0x10+jj));
                                            }
                                                printk("\n");
                                        }
                                    printk("SMRA contents: \n ");
                            #endif

                            printk("SM uncorrectable error is encountered, %4x !!\n", uStatus);
                            outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
                            up(&dmac_sem);
                            outpw(REG_DMACCSR, DMAC_EN+DMAC_SWRST);         // reset DMAC
                            outpw(REG_SMCSR, inpw(REG_SMCSR)|SMCR_SM_SWRST);// reset SM controller
                            memcpy((char *)uDAddr, (char *)_fmi_pNANDBuffer, 512*4);
                            return FMI_SM_ECC_ERROR;
                        //    break;
                        }
                        uStatus >>= 8;
                    }

                    outpw(REG_SMISR, SMISR_ECC_FIELD_IF);       // clear ECC_FLD_Error
                }

                if (_fmi_bIsSMDataReady)
                {
                    if ( !(inpw(REG_SMISR) & SMISR_ECC_FIELD_IF) )
                        break;
                }
            }
        }
        else
        {
            while(1)
            {
                if (_fmi_bIsSMDataReady)
                {
                    outpw(REG_SMISR, SMISR_DMA_IF);                 // clear DMA flag
                    outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
                    break;
                }
            }
        }

        up(&dmac_sem);

        memcpy((char *)uDAddr, (char *)_fmi_pNANDBuffer, 512*4);

//  printk("fmiSM_Read_2K ---> OK \n" );
        return 0;
}


int fmiSM_Read_RA(FMI_SM_INFO_T *pSM, u32 uPage, u32 ucColAddr)
{
        ENTER();

        /* clear R/B flag */
#if defined(CONFIG_W55FA93_TWO_RB_PINS) || defined(CONFIG_W55FA93_NAND1)
        if(pSM == pSM0)
        {
            while (!(inpw(REG_SMISR) & SMISR_RB0));
            outpw(REG_SMISR, SMISR_RB0_IF);
        }
        else
        {
            while (!(inpw(REG_SMISR) & SMISR_RB1));
            outpw(REG_SMISR, SMISR_RB1_IF);
        }
#else

        while (!(inpw(REG_SMISR) & SMISR_RB0));
        outpw(REG_SMISR, SMISR_RB0_IF);
#endif

        outpw(REG_SMCMD, 0x00);     // read command
        outpw(REG_SMADDR, ucColAddr);   // CA0 - CA7
        outpw(REG_SMADDR, (ucColAddr >> 8) & 0xff); // CA8 - CA11
        outpw(REG_SMADDR, uPage & 0xff);    // PA0 - PA7
        if (!pSM->bIsMulticycle)
                outpw(REG_SMADDR, ((uPage >> 8) & 0xff)|EOA_SM);        // PA8 - PA15
        else {
                outpw(REG_SMADDR, (uPage >> 8) & 0xff);     // PA8 - PA15
                outpw(REG_SMADDR, ((uPage >> 16) & 0xff)|EOA_SM);       // PA16 - PA17
        }
        outpw(REG_SMCMD, 0x30);     // read command

        if (!fmiSMCheckRB(pSM)) {
                printk("*** R/B error !!! ***\n");
                return FMI_SM_RB_ERR;
        }
        return 0;
}

int fmiSM_Read_RA_512(FMI_SM_INFO_T *pSM, u32 uPage, u32 uColumm)
{
    ENTER();
    /* 2011/03/04, to support NAND ROM chip "Infinite IM90A001GT" */
    if (! pSM->bIsCheckECC)
        return 0;
    /* clear R/B flag */
#if defined(CONFIG_W55FA93_TWO_RB_PINS) || defined(CONFIG_W55FA93_NAND1)
    if(pSM == pSM0)
    {
        while (!(inpw(REG_SMISR) & SMISR_RB0));
        outpw(REG_SMISR, SMISR_RB0_IF);
    }
    else
    {
        while (!(inpw(REG_SMISR) & SMISR_RB1));
        outpw(REG_SMISR, SMISR_RB1_IF);
    }
#else

    while(!(inpw(REG_SMISR) & SMISR_RB0));
    outpw(REG_SMISR, SMISR_RB0_IF);
#endif

    outpw(REG_SMCMD, 0x50);             // read command
    outpw(REG_SMADDR, uColumm);
    outpw(REG_SMADDR, uPage & 0xff);    // PA0 - PA7
    if (!pSM->bIsMulticycle)
        outpw(REG_SMADDR, ((uPage >> 8) & 0xff)|EOA_SM);        // PA8 - PA15
    else
    {
        outpw(REG_SMADDR, (uPage >> 8) & 0xff);                 // PA8 - PA15
        outpw(REG_SMADDR, ((uPage >> 16) & 0xff)|EOA_SM);       // PA16 - PA17
    }

    if (!fmiSMCheckRB(pSM))
        return FMI_SM_RB_ERR;

    return 0;
}

int fmiSM_Write_2K(FMI_SM_INFO_T *pSM, u32 uSector, u32 ucColAddr, u32 uSAddr)
{
        ENTER();

        if (down_interruptible(&dmac_sem))
                return(GNERR_IO_ERR);
        while (inpw(REG_DMACCSR)&FMI_BUSY); //Wait IP finished... for safe

        memcpy((char *)_fmi_pNANDBuffer, (char *)uSAddr, 512*4);

//  outpw(REG_DMACSAR, uSAddr); // set DMA transfer starting address
        outpw(REG_DMACSAR, _fmi_ucNANDBuffer);

        if (write_nandloader)
            // add "check marker" (0xFF5Axx00, xx is page number) in spare area for NandLoader
            outpw(REG_SMRA_0, 0x00005AFF | (((uSector % pSM->uPagePerBlock) & 0xFF) << 16));
        else
            // set the spare area configuration
            /* write byte 2050, 2051 as used page */
            outpw(REG_SMRA_0, 0x0000FFFF);

        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_REDUN_AUTO_WEN);

        /* clear R/B flag */
#if defined(CONFIG_W55FA93_TWO_RB_PINS) || defined(CONFIG_W55FA93_NAND1)
        if(pSM == pSM0)
        {
            while (!(inpw(REG_SMISR) & SMISR_RB0));
            outpw(REG_SMISR, SMISR_RB0_IF);
        }
        else
        {
            while (!(inpw(REG_SMISR) & SMISR_RB1));
            outpw(REG_SMISR, SMISR_RB1_IF);
        }
#else

        while (!(inpw(REG_SMISR) & SMISR_RB0));
        outpw(REG_SMISR, SMISR_RB0_IF);
#endif

        if (inpw(REG_SMISR) & SMISR_ECC_FIELD_IF) { /* ECC_FLD_IF */
                printk("error sector !!\n");
                outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
        }

        // send command
        outpw(REG_SMCMD, 0x80);     // serial data input command
        outpw(REG_SMADDR, ucColAddr);   // CA0 - CA7
        outpw(REG_SMADDR, (ucColAddr >> 8) & 0xff); // CA8 - CA11
        outpw(REG_SMADDR, uSector & 0xff);  // PA0 - PA7
        if (!pSM->bIsMulticycle)
                outpw(REG_SMADDR, ((uSector >> 8) & 0xff)|EOA_SM);      // PA8 - PA15
        else {
                outpw(REG_SMADDR, (uSector >> 8) & 0xff);       // PA8 - PA15
                outpw(REG_SMADDR, ((uSector >> 16) & 0xff)|EOA_SM);     // PA16 - PA17
        }

        _fmi_bIsSMDataReady = FALSE;

        outpw(REG_SMISR, SMISR_DMA_IF);                 // clear DMA flag
        outpw(REG_SMISR, SMISR_ECC_FIELD_IF);           // clear ECC_FIELD flag
        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_DWR_EN);

        while (!_fmi_bIsSMDataReady) {
//                if (inpw(REG_SMISR) & 0x04) { /* ECC_FLD_IF */
//                      printk("write: error sector !!\n");
//                      outpw(REG_SMISR, 0x04);
//                }
        }

        outpw(REG_SMCMD, 0x10);     // auto program command
        if (!fmiSMCheckRB(pSM)) {
                up(&dmac_sem);
                return FMI_SM_RB_ERR;
        }

        if (fmiSMCheckStatus(pSM) != 0) {
                up(&dmac_sem);
                return FMI_SM_STATE_ERROR;
        }
        up(&dmac_sem);
        return 0;
}

int fmiSM_Read_4K(FMI_SM_INFO_T *pSM, u32 uPage, u32 uDAddr)
{
        u32 uStatus;
        u32 uErrorCnt, ii, jj;

        ENTER();
//  printk("fmiSM_Read_4K uPage=%d   uDAddr=%x\n",uPage,uDAddr);
        if (down_interruptible(&dmac_sem))
                return(GNERR_IO_ERR);

        while (inpw(REG_DMACCSR)&FMI_BUSY); //Wait IP finished... for safe

//  outpw(REG_DMACSAR, uDAddr); // set DMA transfer starting address
        outpw(REG_DMACSAR, _fmi_ucNANDBuffer);

        /* clear R/B flag */
#if defined(CONFIG_W55FA93_TWO_RB_PINS) || defined(CONFIG_W55FA93_NAND1)
        if(pSM == pSM0)
        {
            while (!(inpw(REG_SMISR) & SMISR_RB0));
            outpw(REG_SMISR, SMISR_RB0_IF);
        }
        else
        {
            while (!(inpw(REG_SMISR) & SMISR_RB1));
            outpw(REG_SMISR, SMISR_RB1_IF);
        }
#else

        while (!(inpw(REG_SMISR) & SMISR_RB0));
        outpw(REG_SMISR, SMISR_RB0_IF);
#endif

        outpw(REG_SMCMD, 0x00);             // read command
        outpw(REG_SMADDR, 0);               // CA0 - CA7
        outpw(REG_SMADDR, 0);               // CA8 - CA11
        outpw(REG_SMADDR, uPage & 0xff);    // PA0 - PA7
        if (!pSM->bIsMulticycle)
                outpw(REG_SMADDR, ((uPage >> 8) & 0xff)|EOA_SM);        // PA8 - PA15
        else {
                outpw(REG_SMADDR, (uPage >> 8) & 0xff);                 // PA8 - PA15
                outpw(REG_SMADDR, ((uPage >> 16) & 0xff)|EOA_SM);       // PA16 - PA17
        }
        outpw(REG_SMCMD, 0x30);     // read command

        if (!fmiSMCheckRB(pSM)) {
                up(&dmac_sem);
                return FMI_SM_RB_ERR;
        }
        _fmi_bIsSMDataReady = FALSE;

        outpw(REG_SMISR, SMISR_DMA_IF);                 // clear DMA flag
        outpw(REG_SMISR, SMISR_ECC_FIELD_IF);           // clear ECC_FIELD flag
        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_DRD_EN);

        if ((pSM->bIsCheckECC) || (inpw(REG_SMCSR)&SMCR_ECC_CHK) )
        {
            while(1)
            {
                if (inpw(REG_SMISR) & SMISR_ECC_FIELD_IF)
                {
                    for (jj=0; jj<2; jj++)
                    {
                        uStatus = inpw(REG_SM_ECC_ST0+jj*4);
                        if (!uStatus)
                            continue;

                        for (ii=1; ii<5; ii++)
                        {
                            if (!(uStatus & 0x03))
                            {
                                uStatus >>= 8;
                                continue;
                            }

                            if ((uStatus & 0x03)==0x01)  // correctable error in 1st field
                            {
                                uErrorCnt = uStatus >> 2;
                                fmiSM_CorrectData_BCH(jj*4+ii, uErrorCnt, (u8*)_fmi_pNANDBuffer);
                        #ifdef DEBUG
                                printk("Field %d have %d error!!\n", jj*4+ii, uErrorCnt);
                        #endif
                                break;
                            }
                            else if (((uStatus & 0x03)==0x02)
                                  ||((uStatus & 0x03)==0x03)) // uncorrectable error or ECC error in 1st field
                            {
                                printk("SM uncorrectable BCH error is encountered !!\n");
                                outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
                                up(&dmac_sem);
                                outpw(REG_DMACCSR, DMAC_EN+DMAC_SWRST);         // reset DMAC
                                outpw(REG_SMCSR, inpw(REG_SMCSR)|SMCR_SM_SWRST);// reset SM controller
                                memcpy((char *)uDAddr, (char *)_fmi_pNANDBuffer, 512*8);
                                return FMI_SM_ECC_ERROR;
                            //    break;
                            }
                            uStatus >>= 8;
                        }
                    }
                    outpw(REG_SMISR, SMISR_ECC_FIELD_IF);       // clear ECC_FLD_Error
                }

                if (_fmi_bIsSMDataReady)
                {
                    if ( !(inpw(REG_SMISR) & SMISR_ECC_FIELD_IF) )
                        break;
                }
            }
        }
        else
        {
            while(1)
            {
                if (_fmi_bIsSMDataReady)
                {
                    outpw(REG_SMISR, SMISR_DMA_IF);                 // clear DMA flag
                    outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
                    break;
                }
            }
        }
        up(&dmac_sem);

        memcpy((char *)uDAddr, (char *)_fmi_pNANDBuffer, 512*8);
        return 0;
}


static void sicSMselect(int chipSel)
{
    if (chipSel == 0)
    {
        outpw(REG_GPDFUN, inpw(REG_GPDFUN) | 0x0003CC00);       // enable NAND NWR/NRD/RB0 pins
        outpw(REG_GPEFUN, inpw(REG_GPEFUN) | 0x00F30000);       // enable NAND ALE/CLE/CS0 pins
        outpw(REG_SMCSR, inpw(REG_SMCSR) & ~SMCR_CS0);
        outpw(REG_SMCSR, inpw(REG_SMCSR) |  SMCR_CS1);
    }
    else
    {
        outpw(REG_GPDFUN, inpw(REG_GPDFUN) | 0x0003F000);       // enable NAND NWR/NRD/RB1 pins
        outpw(REG_GPEFUN, inpw(REG_GPEFUN) | 0x00FC0000);       // enable NAND ALE/CLE/CS1 pins
        outpw(REG_SMCSR, inpw(REG_SMCSR) & ~SMCR_CS1);
        outpw(REG_SMCSR, inpw(REG_SMCSR) |  SMCR_CS0);
    }

    //--- 2014/2/26, Reset NAND controller and DMAC to keep clean status for next access.
    // Reset DMAC engine and interrupt satus
    outpw(REG_DMACCSR, inpw(REG_DMACCSR) | DMAC_SWRST | DMAC_EN);
    while(inpw(REG_DMACCSR) & DMAC_SWRST);
    outpw(REG_DMACCSR, inpw(REG_DMACCSR) | DMAC_EN);
    outpw(REG_DMACISR, WEOT_IF | TABORT_IF);    // clear all interrupt flag

    // Reset FMI engine and interrupt status
    outpw(REG_FMICR, FMI_SWRST);
    while(inpw(REG_FMICR) & FMI_SWRST);
    outpw(REG_FMIISR, FMI_DAT_IF);              // clear all interrupt flag

    // Reset NAND engine and interrupt status
    outpw(REG_FMICR, FMI_SM_EN);
    outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_SM_SWRST);
    while(inpw(REG_SMCSR) & SDCR_SWRST);
    outpw(REG_SMISR, 0xFFFFFFFF);               // clear all interrupt flag
}


int fmiSM_Write_4K(FMI_SM_INFO_T *pSM, u32 uSector, u32 ucColAddr, u32 uSAddr)
{
        ENTER();
        if (down_interruptible(&dmac_sem))
                return(GNERR_IO_ERR);
        while (inpw(REG_DMACCSR)&FMI_BUSY); //Wait IP finished... for safe

        memcpy((char *)_fmi_pNANDBuffer, (char *)uSAddr, 512*8);

        outpw(REG_DMACSAR, _fmi_ucNANDBuffer);

        if (write_nandloader)
            // add "check marker" (0xFF5Axx00, xx is page number) in spare area for NandLoader
            outpw(REG_SMRA_0, 0x00005AFF | (((uSector % pSM->uPagePerBlock) & 0xFF) << 16));
        else
            // set the spare area configuration
            /* write byte 4098, 4099 as used page */
            outpw(REG_SMRA_0, 0x0000FFFF);

        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_REDUN_AUTO_WEN);

        /* clear R/B flag */
#if defined(CONFIG_W55FA93_TWO_RB_PINS) || defined(CONFIG_W55FA93_NAND1)
        if(pSM == pSM0)
        {
            while (!(inpw(REG_SMISR) & SMISR_RB0));
            outpw(REG_SMISR, SMISR_RB0_IF);
        }
        else
        {
            while (!(inpw(REG_SMISR) & SMISR_RB1));
            outpw(REG_SMISR, SMISR_RB1_IF);
        }
#else

        while (!(inpw(REG_SMISR) & SMISR_RB0));
        outpw(REG_SMISR, SMISR_RB0_IF);
#endif

        // send command
        outpw(REG_SMCMD, 0x80);                     // serial data input command
        outpw(REG_SMADDR, ucColAddr);               // CA0 - CA7
        outpw(REG_SMADDR, (ucColAddr >> 8) & 0xff); // CA8 - CA11
        outpw(REG_SMADDR, uSector & 0xff);          // PA0 - PA7
        if (!pSM->bIsMulticycle)
                outpw(REG_SMADDR, ((uSector >> 8) & 0xff)|EOA_SM);      // PA8 - PA15
        else {
                outpw(REG_SMADDR, (uSector >> 8) & 0xff);               // PA8 - PA15
                outpw(REG_SMADDR, ((uSector >> 16) & 0xff)|EOA_SM);     // PA16 - PA17
        }

        _fmi_bIsSMDataReady = FALSE;

        outpw(REG_SMISR, SMISR_DMA_IF);                 // clear DMA flag
        outpw(REG_SMISR, SMISR_ECC_FIELD_IF);           // clear ECC_FIELD flag
        outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_DWR_EN);

        while (!_fmi_bIsSMDataReady) {
       //printk("--> fmiSM_Write_4K(): wait _fmi_bIsSMDataReady, uSector=%d\n", uSector);
       //         if (inpw(REG_SMISR) & 0x04) { /* ECC_FLD_IF */
       //                 printk("write: error sector !!\n");
       //                outpw(REG_SMISR, 0x04);
       //         }
        }

        outpw(REG_SMCMD, 0x10);     // auto program command
        if (!fmiSMCheckRB(pSM)) {
                up(&dmac_sem);
                return FMI_SM_RB_ERR;
        }

        if (fmiSMCheckStatus(pSM) != 0) {
                up(&dmac_sem);
                return FMI_SM_STATE_ERROR;
        }
        up(&dmac_sem);
        return 0;
}


static int sicSMpread(int NandPort, int PBA, int page, u8 *buff)
{
        FMI_SM_INFO_T *pSM;
        int pageNo;
        int status;
        int i;
        char *ptr;

#ifdef OPT_SUPPORT_H27UAG8T2A
    int spareSize;
#endif
//        printk("sicSMpread NandPort=%d, PBA=%2x, page=%2x, buffer=%4x\n",NandPort, PBA, page, (u32)buff);
        ENTER();

        // enable SM
        outpw(REG_FMICR, FMI_SM_EN);

        sicSMselect(NandPort);
        if (NandPort == 0)
            pSM = pSM0;
        else
            pSM = pSM1;

        fmiSM_Initial(pSM);

        PBA += pSM->uLibStartBlock;
        pageNo = PBA * pSM->uPagePerBlock + page;
//   printk("sicSMpread NandPort=%d, PBA=%2x, pageNo=%2x, buffer=%4x\n",NandPort, PBA, pageNo, (u32)buff);

#ifdef OPT_FIRST_4BLOCKS_ECC4
    if (PBA <= 3)
    {
    #ifdef OPT_SUPPORT_H27UAG8T2A
        // set to ECC8 for Block 0-3
        if (pSM->nPageSize == NAND_PAGE_4KB)    /* 4KB */
        {
            if (pSM->bIsNandECC12 == TRUE)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T8);             // BCH_8 is selected
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 128);  // Redundant area size
            }
        }
        // set to ECC4 for Block 0-3
        else if (pSM->nPageSize == NAND_PAGE_2KB)   /* 2KB */
        {
            if (pSM->bIsNandECC8 == TRUE)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T4);             // BCH_4 is selected
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 64);  // Redundant area size
            }
        }
    #else
        // set to ECC4 for Block 0-3
        if (pSM->nPageSize == NAND_PAGE_2KB)    /* 2KB */
        {
            if (pSM->bIsNandECC8 == TRUE)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T4);             // BCH_4 is selected
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 64);  // Redundant area size
            }
        }
    #endif
    }
#endif

        if (pSM->nPageSize == NAND_PAGE_2KB)    /* 2KB */
        {
            LEAVE();
            ptr = (char *)REG_SMRA_0;
            fmiSM_Read_RA(pSM, pageNo, 2048);
            for (i=0; i<64; i++)
            {
                *ptr++ = inpw(REG_SMDATA) & 0xff;
            }

            status = fmiSM_Read_2K(pSM, pageNo, (u32)buff);
 //         return (status);
        }
        else if (pSM->nPageSize == NAND_PAGE_4KB)   /* 4KB */
        {
            LEAVE();

#ifdef OPT_SUPPORT_H27UAG8T2A
        spareSize = inpw(REG_SMREAREA_CTL) & SMRE_REA128_EXT;
        ptr = (char *)REG_SMRA_0;
        fmiSM_Read_RA(pSM, pageNo, 4096);
        for (i=0; i<spareSize; i++)
            *ptr++ = inpw(REG_SMDATA) & 0xff;
#else
        ptr = (char *)REG_SMRA_0;
        fmiSM_Read_RA(pSM, pageNo, 4096);
//      for (i=0; i<216; i++)
        for (i=0; i<128; i++)
            *ptr++ = inpw(REG_SMDATA) & 0xff;
#endif
            status = fmiSM_Read_4K(pSM, pageNo, (u32)buff);
        }
        else {  /* 512B */
            LEAVE();
            ptr = (char *)REG_SMRA_0;
            fmiSM_Read_RA_512(pSM, pageNo, 0);
            for (i=0; i<16; i++)
                *ptr++ = inpw(REG_SMDATA) & 0xff;

            status = fmiSM_Read_512(pSM, pageNo, (u32)buff);
        }

#ifdef OPT_FIRST_4BLOCKS_ECC4
    if (PBA <= 3)
    {
    #ifdef OPT_SUPPORT_H27UAG8T2A
        // restore to ECC12
        if (pSM->nPageSize == NAND_PAGE_4KB)    /* 4KB */
        {
            if (pSM->bIsNandECC12 == TRUE)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T12);            // BCH_8 is selected
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 224);  // Redundant area size
            }
        }
        // restore to ECC8
        else if (pSM->nPageSize == NAND_PAGE_2KB)   /* 2KB */
        {
            if (pSM->bIsNandECC8 == TRUE)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T8);             // BCH_8 is selected
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 64);  // Redundant area size
            }
        }
    #else
        if (pSM->nPageSize == NAND_PAGE_2KB)    /* 2KB */
        {
            if (pSM->bIsNandECC8 == TRUE)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T8);             // BCH_8 is selected
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 64);  // Redundant area size
            }
        }
    #endif
    }
#endif
        return (status);
}

static int sicSMpwrite(int NandPort, int PBA, int page, u8 *buff)
{
        FMI_SM_INFO_T *pSM;
        int pageNo;
        int status;

//        printk("sicSMpwrite NandPort=%d, PBA=%2x, page=%2x, buffer=%4x\n",NandPort, PBA, page, (u32)buff);
        ENTER();

        // enable SM
        outpw(REG_FMICR, FMI_SM_EN);

        sicSMselect(NandPort);
        if (NandPort == 0)
            pSM = pSM0;
        else
            pSM = pSM1;

        fmiSM_Initial(pSM);

        PBA += pSM->uLibStartBlock;
        pageNo = PBA * pSM->uPagePerBlock + page;

#ifdef OPT_FIRST_4BLOCKS_ECC4
    if (PBA <= 3)
    {
    #ifdef OPT_SUPPORT_H27UAG8T2A
        // set to ECC8 for Block 0-3
        if (pSM->nPageSize == NAND_PAGE_4KB)    /* 4KB */
        {
            if (pSM->bIsNandECC12 == TRUE)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T8);             // BCH_8 is selected
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 128);  // Redundant area size
            }
        }
        // set to ECC4 for Block 0-3
        else if (pSM->nPageSize == NAND_PAGE_2KB)   /* 2KB */
        {
            if (pSM->bIsNandECC8 == TRUE)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T4);             // BCH_4 is selected
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 64);  // Redundant area size
            }
        }
    #else
        // set to ECC4 for Block 0-3
        if (pSM->nPageSize == NAND_PAGE_2KB)    /* 2KB */
        {
            if (pSM->bIsNandECC8 == TRUE)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T4);             // BCH_4 is selected
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 64);  // Redundant area size
            }
        }
    #endif
    }
#endif

        if (pSM->nPageSize == NAND_PAGE_2KB) {  /* 2KB */
                LEAVE();
                status = fmiSM_Write_2K(pSM, pageNo, 0, (u32)buff);
                return (status);
        } else if (pSM->nPageSize == NAND_PAGE_4KB) {   /* 4KB */
                LEAVE();
                status = fmiSM_Write_4K(pSM, pageNo, 0, (u32)buff);
                return (status);
        } else {    /* 512B */
                LEAVE();
                status = fmiSM_Write_512(pSM, pageNo, (u32)buff);
                return (status);
        }

#ifdef OPT_FIRST_4BLOCKS_ECC4
    if (PBA <= 3)
    {

    #ifdef OPT_SUPPORT_H27UAG8T2A
        // restore to ECC12
        if (pSM->nPageSize == NAND_PAGE_4KB)    /* 4KB */
        {
            if (pSM->bIsNandECC12 == TRUE)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T12);            // BCH_8 is selected
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 224);  // Redundant area size
            }
        }
        // restore to ECC8
        else if (pSM->nPageSize == NAND_PAGE_2KB)   /* 2KB */
        {
            if (pSM->bIsNandECC8 == TRUE)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T8);             // BCH_8 is selected
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 64);  // Redundant area size
            }
        }
    #else
        // restore to ECC8
        if (pSM->nPageSize == NAND_PAGE_2KB)    /* 2KB */
        {
            if (pSM->bIsNandECC8 == TRUE)
            {
                outpw(REG_SMCSR, inpw(REG_SMCSR) &  ~SMCR_BCH_TSEL);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | BCH_T8);             // BCH_8 is selected
                outpw(REG_SMREAREA_CTL, (inpw(REG_SMREAREA_CTL) & ~SMRE_REA128_EXT) | 64);  // Redundant area size
            }
        }
    #endif
    }
#endif
}


int sicSMChangeBadBlockMark(FMI_SM_INFO_T *pSM)
{
    int status=0;
    int chipPort = 0;

    // enable SM
    outpw(REG_FMICR, FMI_SM_EN);
    fmiSM_Initial(pSM);

    /* read physical block 0 - image information */
    if (pSM == pSM0)
        chipPort = 0;
    else
        chipPort = 1;

    status = sicSMpread(chipPort, 0, pSM->uPagePerBlock-1, (u8*)_fmi_gptr1);
    if (status < 0)
        return status;

    /* write specific mark */
    *(_fmi_gptr1+pSM->nPageSize-6) = '5';
    *(_fmi_gptr1+pSM->nPageSize-5) = '5';
    *(_fmi_gptr1+pSM->nPageSize-4) = '0';
    *(_fmi_gptr1+pSM->nPageSize-3) = '0';
    *(_fmi_gptr1+pSM->nPageSize-2) = '9';
    *(_fmi_gptr1+pSM->nPageSize-1) = '1';

#if 1
    // doesn't write "550091" to block-0
//  status = sicSMpwrite(chipPort, 0, pSM->uPagePerBlock-1, (u8*)_fmi_gptr1);
#else
        if (pSM->nPageSize == NAND_PAGE_2KB) {  /* 2KB */
                status = fmiSM_Write_2K(pSM, pSM->uPagePerBlock-1, 0, (u32)_fmi_gptr1);
        } else if (pSM->nPageSize == NAND_PAGE_4KB) {   /* 4KB */
                if (pSM->bIsNandECC4)
                        status = fmiSM_Write_4K_ECC4(pSM, pSM->uPagePerBlock-1, 0, (u32)_fmi_gptr1);
                else
                        status = fmiSM_Write_4K(pSM, pSM->uPagePerBlock-1, 0, (u32)_fmi_gptr1);
    }
#endif
    return status;
}


int fmiSMCheckBootHeader(FMI_SM_INFO_T *pSM)
{

        int chipPort;
        int volatile status, imageCount, i, infoPage, block;
        unsigned int *pImageList = (unsigned int *)_fmi_gptr1;
        volatile int ii;

#define OPT_FOUR_BOOT_IMAGE

        //  ENTER();
        int  fmiNandSysArea = 0;

        memset(_fmi_gptr1, 0xff, 4096);
        infoPage = pSM->uPagePerBlock-1;

        /* read physical block 0 - image information */
        if (pSM == pSM0)
            chipPort = 0;
        else
            chipPort = 1;

#ifdef OPT_FOUR_BOOT_IMAGE
        for (ii=0; ii<4; ii++)
        {
            status = sicSMpread(chipPort, ii, pSM->uPagePerBlock-1, (u8*)_fmi_gptr1);
            if (!status)
            {
                if (((*(pImageList+0)) == 0x574255aa) && ((*(pImageList+3)) == 0x57425963))
                    break;
            }
        }
#else
        status = sicSMpread(chipPort, 0, pSM->uPagePerBlock-1, (u8*)_fmi_gptr1);
#endif

        if (status < 0)
            return status;

    /* check specific mark */
    if (pSM->nPageSize != NAND_PAGE_512B)
    {
        if ((*(_fmi_gptr1+pSM->nPageSize-6) == '5') && (*(_fmi_gptr1+pSM->nPageSize-5) == '5') &&
            (*(_fmi_gptr1+pSM->nPageSize-4) == '0') && (*(_fmi_gptr1+pSM->nPageSize-3) == '0') &&
            (*(_fmi_gptr1+pSM->nPageSize-2) == '9') && (*(_fmi_gptr1+pSM->nPageSize-1) == '1'))
        {
            _fmi_bIsNandFirstAccess = 0;
                printk("Boot ID is found !!!\n");
        }
        else
        {
            //printk("Boot ID NOT found !!!\n");
            sicSMChangeBadBlockMark(pSM);
        }
    }
        printk("fmiSMCheckBootHeader %d\n", _fmi_bIsNandFirstAccess);

        if (((*(pImageList+0)) == 0x574255aa) && ((*(pImageList+3)) == 0x57425963)) {
                fmiNandSysArea = *(pImageList+1);
        }

        if (fmiNandSysArea != 0xFFFFFFFF && fmiNandSysArea != 0) {
                pSM->uLibStartBlock = (fmiNandSysArea / pSM->uSectorPerBlock) + 1;
        } else {

                /* read physical block 0 - image information */

    #ifdef OPT_FOUR_BOOT_IMAGE
                for (ii=0; ii<4; ii++)
                {
                    status = sicSMpread(chipPort, ii, pSM->uPagePerBlock-2, (u8*)_fmi_gptr1);
                    if (!status)
                    {
                        if (((*(pImageList+0)) == 0x574255aa) && ((*(pImageList+3)) == 0x57425963))
                            break;
                    }
                }

    #else
                status = sicSMpread(chipPort, 0, pSM->uPagePerBlock-2, (u8*)_fmi_gptr1);
    #endif

                if (status < 0)
                    return status;

                if (((*(pImageList+0)) == 0x574255aa) && ((*(pImageList+3)) == 0x57425963)) {
                        imageCount = *(pImageList+1);

                        /* pointer to image information */
                        pImageList = pImageList+4;
                        for (i=0; i<imageCount; i++) {
                                block = (*(pImageList + 1) & 0xFFFF0000) >> 16;
                                if (block > pSM->uLibStartBlock)
                                        pSM->uLibStartBlock = block;

                                /* pointer to next image */
//                                pImageList = pImageList+8;
                                pImageList = pImageList+12; //### 2010.10.01 KC Modified
                        }
                        pSM->uLibStartBlock++;
                }
        }
        LEAVE();
        return 0;
}

int fmiCheckInvalidBlock(FMI_SM_INFO_T *pSM, u32 BlockNo)
{
        int volatile status=0;
        unsigned int volatile sector;
        unsigned char volatile data512=0xff, data517=0xff;
        unsigned char volatile blockStatus;

#define OPT_FA93
#ifdef OPT_FA93
    if (pSM->bIsMLCNand == TRUE)
        sector = (BlockNo+1) * pSM->uPagePerBlock - 1;
    else
        sector = BlockNo * pSM->uPagePerBlock;
#else
        sector = BlockNo * pSM->uPagePerBlock;
#endif

        sector = BlockNo * pSM->uPagePerBlock;

    if (pSM->nPageSize == NAND_PAGE_512B)
        status = fmiSM2BufferM_RA(pSM, sector, 0);
    else
        status = fmiSM_Read_RA(pSM, sector, pSM->nPageSize);
    if (status < 0) {
        //printk("fmiCheckInvalidBlock 0x%x\n", status);
        return 1;
    }

    if (pSM->nPageSize == NAND_PAGE_512B)
    {
        /* 2011/03/22, to support NAND ROM chip "Infinite IM90A001GT" on both MTD and RS. */
        if (! pSM->bIsCheckECC)
        {
            data512 = 0xff;
            data517 = 0xff;
        }
        else
    {
        data512 = inpw(REG_SMDATA) & 0xff;
        data517 = inpw(REG_SMDATA);
        data517 = inpw(REG_SMDATA);
        data517 = inpw(REG_SMDATA);
        data517 = inpw(REG_SMDATA);
        data517 = inpw(REG_SMDATA) & 0xff;
        }
    //  if ((data512 != 0xFF) || (data517 != 0xFF))
        if ((data512 == 0xFF) && (data517 == 0xFF))
        {
            fmiSM_Reset(pSM);
            status = fmiSM2BufferM_RA(pSM, sector+1, 0);
            if (status < 0)
            {
    #ifdef DEBUG
                    printk("fmiCheckInvalidBlock 0x%x\n", status);
    #endif
                    return 1;
            }
            /* 2011/03/22, to support NAND ROM chip "Infinite IM90A001GT" on both MTD and RS. */
            if (! pSM->bIsCheckECC)
            {
                data512 = 0xff;
                data517 = 0xff;
            }
            else
            {
            data512 = inpw(REG_SMDATA) & 0xff;
            data517 = inpw(REG_SMDATA);
            data517 = inpw(REG_SMDATA);
            data517 = inpw(REG_SMDATA);
            data517 = inpw(REG_SMDATA);
            data517 = inpw(REG_SMDATA) & 0xff;
            }
            if ((data512 != 0xFF) || (data517 != 0xFF))
            {
                fmiSM_Reset(pSM);
                return 1;   // invalid block
            }
        }
        else
        {
            fmiSM_Reset(pSM);
            return 1;   // invalid block
        }
    }
    else
    {
        blockStatus = inpw(REG_SMDATA) & 0xff;
        if (blockStatus == 0xFF)
        {
            fmiSM_Reset(pSM);

            if (pSM->bIsMLCNand == TRUE)
                sector = (BlockNo+1) * pSM->uPagePerBlock - 1;  // check last page
            else
                sector++;                                       // check next page

            status = fmiSM_Read_RA(pSM, sector, pSM->nPageSize);
            if (status < 0)
            {
    #ifdef DEBUG
                    printk("fmiCheckInvalidBlock 0x%x\n", status);
    #endif
                    return 1;
            }
            blockStatus = inpw(REG_SMDATA) & 0xff;
            if (blockStatus != 0xFF)
            {
                fmiSM_Reset(pSM);
                return 1;   // invalid block
            }
        }
        else
        {
            fmiSM_Reset(pSM);
            return 1;   // invalid block
        }
    }

    fmiSM_Reset(pSM);
    return(0);
}


int fmiNormalCheckBlock(FMI_SM_INFO_T *pSM, u32 BlockNo)
{
    int volatile status=0;
    unsigned int volatile sector;
    unsigned char data, data517;

    /* MLC check the 2048/4096 byte of last page per block */
    if (pSM->bIsMLCNand == 1)
    {
        if (pSM->nPageSize == NAND_PAGE_2KB)
        {
            sector = (BlockNo+1) * pSM->uPagePerBlock - 1;
            /* Read 2048 byte */
            status = fmiSM_Read_RA(pSM, sector, 2048);
            if (status < 0)
            {
#ifdef DEBUG
                printk("fmiNormalCheckBlock 0x%x\n", status);
#endif
                return 1;
            }
            data = inpw(REG_SMDATA) & 0xff;
            if (data != 0xFF)
                return 1;   // invalid block
        }
        else if (pSM->nPageSize == NAND_PAGE_4KB)
        {
            sector = (BlockNo+1) * pSM->uPagePerBlock - 1;
            /* Read 4096 byte */
            status = fmiSM_Read_RA(pSM, sector, 4096);
            if (status < 0)
            {
#ifdef DEBUG
                printk("fmiNormalCheckBlock 0x%x\n", status);
#endif
                return 1;
            }
            data = inpw(REG_SMDATA) & 0xff;
            if (data != 0xFF)
                return 1;   // invalid block
        }
    }
    /* SLC check the 2048 byte of 1st or 2nd page per block */
    else    // SLC
    {
        sector = BlockNo * pSM->uPagePerBlock;
        if (pSM->nPageSize == NAND_PAGE_4KB)
        {
            status = fmiSM_Read_RA(pSM, sector, 4096);
            if (status < 0)
            {
#ifdef DEBUG
                printk("fmiNormalCheckBlock 0x%x\n", status);
#endif
                return 1;
            }
            data = inpw(REG_SMDATA) & 0xff;
    //      if (data != 0xFF)
            if (data == 0xFF)
            {
#ifdef DEBUG
    //          printk("find bad block, check next page to confirm it.\n");
#endif
                status = fmiSM_Read_RA(pSM, sector+1, 4096);
                if (status < 0)
                {
#ifdef DEBUG
                    printk("fmiNormalCheckBlock 0x%x\n", status);
#endif
                    return 1;
                }
                data = inpw(REG_SMDATA) & 0xff;
                if (data != 0xFF)
                {
#ifdef DEBUG
                    printk("find bad block is conformed.\n");
#endif
                    return 1;   // invalid block
                }
            }
            else
            {
#ifdef DEBUG
                printk("find bad block is conformed.\n");
#endif
                return 1;   // invalid block
            }
        }
        else if (pSM->nPageSize == NAND_PAGE_2KB)
        {
            status = fmiSM_Read_RA(pSM, sector, 2048);
            if (status < 0)
            {
#ifdef DEBUG
                printk("fmiNormalCheckBlock 0x%x\n", status);
#endif
                return 1;
            }
            data = inpw(REG_SMDATA) & 0xff;
    //      if (data != 0xFF)
            if (data == 0xFF)
            {
#ifdef DEBUG
    //          printk("find bad block, check next page to confirm it.\n");
#endif
                status = fmiSM_Read_RA(pSM, sector+1, 2048);
                if (status < 0)
                {
#ifdef DEBUG
                    printk("fmiNormalCheckBlock 0x%x\n", status);
#endif
                    return 1;
                }
                data = inpw(REG_SMDATA) & 0xff;
                if (data != 0xFF)
                {
#ifdef DEBUG
                    printk("find bad block is conformed.\n");
#endif
                    return 1;   // invalid block
                }
            }
            else
            {
#ifdef DEBUG
                printk("find bad block is conformed.\n");
#endif
                return 1;   // invalid block
            }
        }
        else    /* page size 512B */
        {
            status = fmiSM2BufferM_RA(pSM, sector, 0);
            if (status < 0)
            {
#ifdef DEBUG
                printk("fmiNormalCheckBlock 0x%x\n", status);
#endif
                return 1;
            }
            /* 2011/03/22, to support NAND ROM chip "Infinite IM90A001GT" on both MTD and RS. */
            if (! pSM->bIsCheckECC)
            {
                data = 0xff;
                data517 = 0xff;
            }
            else
            {
            data = inpw(REG_SMDATA) & 0xff;
            data517 = inpw(REG_SMDATA);
            data517 = inpw(REG_SMDATA);
            data517 = inpw(REG_SMDATA);
            data517 = inpw(REG_SMDATA);
            data517 = inpw(REG_SMDATA) & 0xff;
            }
    //      if ((data != 0xFF) || (data517 != 0xFF))
            if ((data == 0xFF) && (data517 == 0xFF))
            {
                fmiSM_Reset(pSM);
                status = fmiSM2BufferM_RA(pSM, sector+1, 0);
                if (status < 0)
                {
#ifdef DEBUG
                    printk("fmiNormalCheckBlock 0x%x\n", status);
#endif
                    return 1;
                }
                /* 2011/03/22, to support NAND ROM chip "Infinite IM90A001GT" on both MTD and RS. */
                if (! pSM->bIsCheckECC)
                {
                    data = 0xff;
                    data517 = 0xff;
                }
                else
                {
                data = inpw(REG_SMDATA) & 0xff;
                data517 = inpw(REG_SMDATA);
                data517 = inpw(REG_SMDATA);
                data517 = inpw(REG_SMDATA);
                data517 = inpw(REG_SMDATA);
                data517 = inpw(REG_SMDATA) & 0xff;
                }
                if ((data != 0xFF) || (data517 != 0xFF))
                {
                    fmiSM_Reset(pSM);
                    return 1;   // invalid block
                }
            }
            else
            {
#ifdef DEBUG
                printk("find bad block is conformed.\n");
#endif

                fmiSM_Reset(pSM);
                return 1;   // invalid block
            }
            fmiSM_Reset(pSM);
        }
    }
    return 0;
}

#ifdef OPT_MARK_BAD_BLOCK_WHILE_ERASE_FAIL

    static int sicSMMarkBadBlock_WhileEraseFail(FMI_SM_INFO_T *pSM, u32 BlockNo)
    {
        u32 uSector, ucColAddr = 0;

        /* check if MLC NAND */
        if (pSM->bIsMLCNand == TRUE)
        {
            uSector = (BlockNo+1) * pSM->uPagePerBlock - 1; // write last page
            if (pSM->nPageSize == NAND_PAGE_2KB)
            {
                ucColAddr = 2048;       // write 2048th byte
            }
            else if (pSM->nPageSize == NAND_PAGE_4KB)
            {
                ucColAddr = 4096;       // write 4096th byte
            }

            // send command
            outpw(REG_SMCMD, 0x80);     // serial data input command
            outpw(REG_SMADDR, ucColAddr);   // CA0 - CA7
            outpw(REG_SMADDR, (ucColAddr >> 8) & 0xff); // CA8 - CA11
            outpw(REG_SMADDR, uSector & 0xff);  // PA0 - PA7
            if (!pSM->bIsMulticycle)
                outpw(REG_SMADDR, ((uSector >> 8) & 0xff)|EOA_SM);      // PA8 - PA15
            else
            {
                outpw(REG_SMADDR, (uSector >> 8) & 0xff);       // PA8 - PA15
                outpw(REG_SMADDR, ((uSector >> 16) & 0xff)|EOA_SM);     // PA16 - PA17
            }
            outpw(REG_SMDATA, 0xf0);    // mark bad block (use 0xf0 instead of 0x00 to differ from Old (Factory) Bad Blcok Mark)
            outpw(REG_SMCMD, 0x10);

            if (! fmiSMCheckRB(pSM))
                return FMI_SM_RB_ERR;

            fmiSM_Reset(pSM);
            return 0;
        }
        /* SLC check the 2048 byte of 1st or 2nd page per block */
        else    // SLC
        {
            uSector = BlockNo * pSM->uPagePerBlock;     // write lst page
            if (pSM->nPageSize == NAND_PAGE_2KB)
            {
                ucColAddr = 2048;       // write 2048th byte
            }
            else if (pSM->nPageSize == NAND_PAGE_4KB)
            {
                ucColAddr = 4096;       // write 4096th byte
            }
            else if (pSM->nPageSize == NAND_PAGE_512B)
            {
                ucColAddr = 0;          // write 4096th byte
                goto _mark_512;
            }

            // send command
            outpw(REG_SMCMD, 0x80);     // serial data input command
            outpw(REG_SMADDR, ucColAddr);   // CA0 - CA7
            outpw(REG_SMADDR, (ucColAddr >> 8) & 0xff); // CA8 - CA11
            outpw(REG_SMADDR, uSector & 0xff);  // PA0 - PA7
            if (!pSM->bIsMulticycle)
                outpw(REG_SMADDR, ((uSector >> 8) & 0xff)|EOA_SM);      // PA8 - PA15
            else
            {
                outpw(REG_SMADDR, (uSector >> 8) & 0xff);       // PA8 - PA15
                outpw(REG_SMADDR, ((uSector >> 16) & 0xff)|EOA_SM);     // PA16 - PA17
            }
            outpw(REG_SMDATA, 0xf0);    // mark bad block (use 0xf0 instead of 0x00 to differ from Old (Factory) Bad Blcok Mark)
            outpw(REG_SMCMD, 0x10);

            if (! fmiSMCheckRB(pSM))
                return FMI_SM_RB_ERR;

            fmiSM_Reset(pSM);
            return 0;

    _mark_512:

            outpw(REG_SMCMD, 0x50);     // point to redundant area
            outpw(REG_SMCMD, 0x80);     // serial data input command
            outpw(REG_SMADDR, ucColAddr);   // CA0 - CA7
            outpw(REG_SMADDR, uSector & 0xff);  // PA0 - PA7
            if (!pSM->bIsMulticycle)
                outpw(REG_SMADDR, ((uSector >> 8) & 0xff)|EOA_SM);      // PA8 - PA15
            else
            {
                outpw(REG_SMADDR, (uSector >> 8) & 0xff);       // PA8 - PA15
                outpw(REG_SMADDR, ((uSector >> 16) & 0xff)|EOA_SM);     // PA16 - PA17
            }

            outpw(REG_SMDATA, 0xf0);    // 512
            outpw(REG_SMDATA, 0xff);
            outpw(REG_SMDATA, 0xff);
            outpw(REG_SMDATA, 0xff);
            outpw(REG_SMDATA, 0xf0);    // 516
            outpw(REG_SMDATA, 0xf0);    // 517
            outpw(REG_SMCMD, 0x10);
            if (! fmiSMCheckRB(pSM))
                return FMI_SM_RB_ERR;

            fmiSM_Reset(pSM);
            return 0;
        }
    }

#endif      // OPT_MARK_BAD_BLOCK_WHILE_ERASE_FAIL

#if 0
    int sicSMMarkBadBlock(FMI_SM_INFO_T *pSM, u32 BlockNo)
    {
        u32 sector, column;

        /* page 0 */
        sector = BlockNo * pSM->uPagePerBlock;
        column = 512;

        // send command
        outpw(REG_SMCMD, 0x80);     // serial data input command
        outpw(REG_SMADDR, column);  // CA0 - CA7
        outpw(REG_SMADDR, (column >> 8) & 0x1f);    // CA8 - CA12
        outpw(REG_SMADDR, sector & 0xff);   // PA0 - PA7
        if (!pSM->bIsMulticycle)
            outpw(REG_SMADDR, ((sector >> 8) & 0xff)|0x80000000);       // PA8 - PA15
        else
        {
            outpw(REG_SMADDR, (sector >> 8) & 0xff);        // PA8 - PA15
            outpw(REG_SMADDR, ((sector >> 16) & 0xff)|0x80000000);      // PA16 - PA17
        }

        outpw(REG_SMDATA, 0xf0);    // 512
        outpw(REG_SMDATA, 0xff);
        outpw(REG_SMDATA, 0xff);
        outpw(REG_SMDATA, 0xff);
        outpw(REG_SMDATA, 0xf0);    // 516
        outpw(REG_SMDATA, 0xf0);    // 517
        outpw(REG_SMCMD, 0x10);
        if (! fmiSMCheckRB(pSM))
            return FMI_SM_RB_ERR;
        fmiSM_Reset(pSM);

        /* page 1 */
        sector++;
        // send command
        outpw(REG_SMCMD, 0x80);     // serial data input command
        outpw(REG_SMADDR, column);  // CA0 - CA7
        outpw(REG_SMADDR, (column >> 8) & 0x1f);    // CA8 - CA12
        outpw(REG_SMADDR, sector & 0xff);   // PA0 - PA7
        if (!pSM->bIsMulticycle)
            outpw(REG_SMADDR, ((sector >> 8) & 0xff)|0x80000000);       // PA8 - PA15
        else
        {
            outpw(REG_SMADDR, (sector >> 8) & 0xff);        // PA8 - PA15
            outpw(REG_SMADDR, ((sector >> 16) & 0xff)|0x80000000);      // PA16 - PA17
        }

        outpw(REG_SMDATA, 0xf0);    // 512
        outpw(REG_SMDATA, 0xff);
        outpw(REG_SMDATA, 0xff);
        outpw(REG_SMDATA, 0xff);
        outpw(REG_SMDATA, 0xf0);    // 516
        outpw(REG_SMDATA, 0xf0);    // 517
        outpw(REG_SMCMD, 0x10);
        if (! fmiSMCheckRB(pSM))
            return FMI_SM_RB_ERR;
        fmiSM_Reset(pSM);

        return 0;
    }
#endif


/* function pointer */
//FMI_SM_INFO_T *pSM0, *pSM1;
static int sicSMInit(int NandPort, NDISK_T *NDISK_info)
{
        int status=0, count;

        ENTER();

        outpw(REG_DMACCSR, inpw(REG_DMACCSR) | DMAC_EN);
        outpw(REG_DMACCSR, inpw(REG_DMACCSR) | DMAC_SWRST);
        outpw(REG_DMACCSR, inpw(REG_DMACCSR) & ~DMAC_SWRST);
        outpw(REG_FMICR, FMI_SM_EN);

        // if (down_interruptible(&fmi_sem))
        //        return GNERR_IO_ERR;

        if ((_nand_init0 == 0) && (_nand_init1 == 0))
        {
                // enable SM
                /* select NAND control pin used */
                outpw(REG_SMTCR, 0x20305);
                outpw(REG_SMCSR, (inpw(REG_SMCSR) & ~SMCR_PSIZE) | PSIZE_512);
                outpw(REG_SMCSR, inpw(REG_SMCSR) |  SMCR_ECC_3B_PROTECT);
                outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_ECC_CHK);

                /* init SM interface */
                outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_REDUN_AUTO_WEN);
        }

        sicSMselect(NandPort);
        mdelay(1);

        if (NandPort == 0) {
                if (_nand_init0)
                        return 0;

                pSM0 = kmalloc(sizeof(FMI_SM_INFO_T),GFP_KERNEL);
                if (pSM0 == NULL)
                    return FMI_SM_NO_MEMORY;

                memset((char *)pSM0, 0, sizeof(FMI_SM_INFO_T));

                if ((status = fmiSM_ReadID(pSM0, NDISK_info)) < 0) {
                        //              up(&fmi_sem);
                        if (pSM0 != NULL) {
                                kfree(pSM0);
                                pSM0 = 0;
                        }
                        return status;
                }
                fmiSM_Initial(pSM0);

                // check NAND boot header
                fmiSMCheckBootHeader(pSM0);

                while (1) {
            #if 1
                        if (!fmiNormalCheckBlock(pSM0, pSM0->uLibStartBlock))
                                break;
                        else
                                pSM0->uLibStartBlock++;
            #else
                        if (fmiCheckInvalidBlock(pSM0, pSM0->uLibStartBlock) != 1)  // valid block
                                break;
                        else
                                pSM0->uLibStartBlock++;
            #endif
                }
                if (pSM0->bIsCheckECC) {
                        if (pSM0->uLibStartBlock == 0)
                                pSM0->uLibStartBlock++;
                }
                printk("Nand0: uLibStartBlock=%d\n",pSM0->uLibStartBlock);
                NDISK_info->nStartBlock = pSM0->uLibStartBlock;     /* available start block */
                pSM0->uBlockPerFlash -= pSM0->uLibStartBlock;
                count = NDISK_info->nBlockPerZone * 2 / 100 + NAND_RESERVED_BLOCK;

                //    up(&fmi_sem);
                NDISK_info->nBlockPerZone = (NDISK_info->nBlockPerZone * NDISK_info->nZone - NDISK_info->nStartBlock) / NDISK_info->nZone;
                NDISK_info->nLBPerZone = NDISK_info->nBlockPerZone - count;
                NDISK_info->nNandNo = NandPort;
                _nand_init0 = 1;
        } else if (NandPort == 1) {
                if (_nand_init1)
                        return 0;

                pSM1 = kmalloc(sizeof(FMI_SM_INFO_T),GFP_KERNEL);
                memset((char *)pSM1, 0, sizeof(FMI_SM_INFO_T));

                if ((status = fmiSM_ReadID(pSM1, NDISK_info)) < 0) {
                        //          up(&fmi_sem);
                        if (pSM1 != NULL) {
                                kfree(pSM1);
                                pSM1 = 0;
                        }
                        return status;
                }
                fmiSM_Initial(pSM1);

                // check NAND boot header
                fmiSMCheckBootHeader(pSM1);

                while (1) {
            #if 1
                        if (!fmiNormalCheckBlock(pSM1, pSM1->uLibStartBlock))
                                break;
                        else
                                pSM1->uLibStartBlock++;
            #else

                        if (fmiCheckInvalidBlock(pSM1, pSM1->uLibStartBlock) != 1)  // valid block
                                break;
                        else
                                pSM1->uLibStartBlock++;
            #endif
            #if defined(CONFIG_W55FA93_NAND_BOTH) || defined(CONFIG_W55FA95_NAND1)
                        if (nand_card_status() == 1)    // nand card removed now
                        {
                            printk("NAND Warning: Nand Card initial fail since card had beed removed.\n");
                            if (pSM1 != NULL)
                            {
                                kfree(pSM1);
                                pSM1 = 0;
                            }
                            LEAVE();
                            return FMI_SM_INIT_ERROR;
                        }
            #endif
                }
                if (pSM1->bIsCheckECC) {
                        if (pSM1->uLibStartBlock == 0)
                                pSM1->uLibStartBlock++;
                }
                printk("Nand1: uLibStartBlock=%d\n",pSM1->uLibStartBlock);
                NDISK_info->nStartBlock = pSM1->uLibStartBlock;     /* available start block */
                pSM1->uBlockPerFlash -= pSM1->uLibStartBlock;
                count = NDISK_info->nBlockPerZone * 2 / 100 + NAND_RESERVED_BLOCK;

                //up(&fmi_sem);
                NDISK_info->nBlockPerZone = (NDISK_info->nBlockPerZone * NDISK_info->nZone - NDISK_info->nStartBlock) / NDISK_info->nZone;
                NDISK_info->nLBPerZone = NDISK_info->nBlockPerZone - count;
                NDISK_info->nNandNo = NandPort;
                _nand_init1 = 1;
        }
        LEAVE();
        return 0;
}


static int sicSM_is_page_dirty(int NandPort, int PBA, int page)
{
        FMI_SM_INFO_T *pSM;
        int pageNo;
        u8 data0, data1;

        ENTER();

        // enable SM
        outpw(REG_FMICR, FMI_SM_EN);

        sicSMselect(NandPort);
        if (NandPort == 0)
            pSM = pSM0;
        else
            pSM = pSM1;

        fmiSM_Initial(pSM);

        PBA += pSM->uLibStartBlock;
        pageNo = PBA * pSM->uPagePerBlock + page;

        if (pSM->nPageSize == NAND_PAGE_2KB)    /* 2KB */
            fmiSM_Read_RA(pSM, pageNo, 2050);
        else if (pSM->nPageSize == NAND_PAGE_4KB)   /* 4KB */
            fmiSM_Read_RA(pSM, pageNo, 4098);
        else if (pSM->nPageSize == NAND_PAGE_8KB)   /* 8KB */
            fmiSM_Read_RA(pSM, pageNo, 8194);
        else    /* 512B */
            fmiSM_Read_RA_512(pSM, pageNo, 2);

        data0 = inpw(REG_SMDATA) & 0xff;
        data1 = inpw(REG_SMDATA) & 0xff;

        if (pSM->nPageSize == NAND_PAGE_512B)   /* 512B */
                fmiSM_Reset(pSM);

//        if ((data0 == 0) && (data1 == 0x00))
        if (data0 == 0x00)
            return 1;   // used page

        else if (data0 != 0xff)
            return 1;   // used page

        return 0;   // un-used page
}


static int sicSM_is_valid_block(int NandPort, int PBA)
{
        FMI_SM_INFO_T *pSM;

        ENTER();

        // enable SM
        outpw(REG_FMICR, FMI_SM_EN);

        sicSMselect(NandPort);
        if (NandPort == 0)
            pSM = pSM0;
        else
            pSM = pSM1;

        PBA += pSM->uLibStartBlock;

        // enable SM
        fmiSM_Initial(pSM);

//        PBA += pSM->uLibStartBlock;
        if (fmiCheckInvalidBlock(pSM, PBA) == 1)    // invalid block
                return 0;
        else
                return 1;   // valid block
}


static int sicSMblock_erase(int NandPort, int PBA)
{
        FMI_SM_INFO_T *pSM;
        u32 page_no;

        ENTER();

        // enable SM
        outpw(REG_FMICR, FMI_SM_EN);

        sicSMselect(NandPort);
        if (NandPort == 0)
            pSM = pSM0;
        else
            pSM = pSM1;

        fmiSM_Initial(pSM);

        PBA += pSM->uLibStartBlock;
        if (fmiCheckInvalidBlock(pSM, PBA) != 1) {
                page_no = PBA * pSM->uPagePerBlock;     // get page address

                //fmiSM_Reset(pSM);

                /* clear R/B flag */
        #if defined(CONFIG_W55FA93_TWO_RB_PINS) || defined(CONFIG_W55FA93_NAND1)
                if(pSM == pSM0)
                {
                    while (!(inpw(REG_SMISR) & SMISR_RB0));
                    outpw(REG_SMISR, SMISR_RB0_IF);
                }
                else
                {
                    while (!(inpw(REG_SMISR) & SMISR_RB1));
                    outpw(REG_SMISR, SMISR_RB1_IF);
                }
        #else

                while (!(inpw(REG_SMISR) & SMISR_RB0));
                outpw(REG_SMISR, SMISR_RB0_IF);
        #endif

                if (inpw(REG_SMISR) & SMISR_ECC_FIELD_IF)   /* ECC_FLD_IF */
                {
        #ifdef DEBUG
                    printk("erase: error sector !!\n");
        #endif
                    outpw(REG_SMISR, SMISR_ECC_FIELD_IF);
                }

                outpw(REG_SMCMD, 0x60);     // erase setup command

                outpw(REG_SMADDR, (page_no & 0xff));        // PA0 - PA7
                if (!pSM->bIsMulticycle)
                        outpw(REG_SMADDR, ((page_no >> 8) & 0xff)|EOA_SM);      // PA8 - PA15
                else {
                        outpw(REG_SMADDR, ((page_no >> 8) & 0xff));     // PA8 - PA15
                        outpw(REG_SMADDR, ((page_no >> 16) & 0xff)|EOA_SM);     // PA16 - PA17
                }

                outpw(REG_SMCMD, 0xd0);     // erase command
                if (!fmiSMCheckRB(pSM))
                        return FMI_SM_RB_ERR;

                if (fmiSMCheckStatus(pSM) != 0) {
        #ifdef OPT_MARK_BAD_BLOCK_WHILE_ERASE_FAIL
                        sicSMMarkBadBlock_WhileEraseFail(pSM,PBA);
        #endif
                        return FMI_SM_STATUS_ERR;
                }
        } else
                return FMI_SM_INVALID_BLOCK;

        LEAVE();
        return 0;
}


static int sicSMchip_erase(int NandPort)
{
        int i, status=0;
        FMI_SM_INFO_T *pSM;

        ENTER();

        // enable SM
        outpw(REG_FMICR, FMI_SM_EN);

        sicSMselect(NandPort);
        if (NandPort == 0)
            pSM = pSM0;
        else
            pSM = pSM1;

        fmiSM_Initial(pSM);

        // erase all chip
        for (i=0; i<=pSM->uBlockPerFlash; i++) {
                status = sicSMblock_erase(NandPort, i);
                if (status < 0)
                        printk("SM block erase fail <%d>!!\n", i);
        }

        LEAVE();
        return 0;
}

/* driver function */
int nandInit0(NDISK_T *NDISK_info)
{
        ENTER();
        return (sicSMInit(0, NDISK_info));
}

int nandpread0(int PBA, int page, u8 *buff)
{
        ENTER();
        return (sicSMpread(0, PBA, page, buff));
}

int nandpwrite0(int PBA, int page, u8 *buff)
{
        ENTER();
        return (sicSMpwrite(0, PBA, page, buff));
}

int nand_is_page_dirty0(int PBA, int page)
{
        ENTER();
        return (sicSM_is_page_dirty(0, PBA, page));
}

int nand_is_valid_block0(int PBA)
{
        ENTER();
        return (sicSM_is_valid_block(0, PBA));
}

int nand_block_erase0(int PBA)
{
        ENTER();
        return (sicSMblock_erase(0, PBA));
}

int nand_chip_erase0(void)
{
        ENTER();
        return (sicSMchip_erase(0));
}


//u8 ioctl_buf[8192];     // The data buffer for IOCTL operation.
u8 ioctl_buf[8192] __attribute__((aligned (32)));
/*-----------------------------------------------------------------------------
 * 2013/11/26, support IOCTL for NAND port 0.
 * Input:
 *      cmd: the command for ioctl. Define at NandDrv.h
 *      arg: the pointer to FMI_IOCTL_INFO_T.
 *---------------------------------------------------------------------------*/
int nand_ioctl_0(int cmd, int arg, int param3, int param4)
{
    FMI_IOCTL_INFO_T *pArg = (FMI_IOCTL_INFO_T *)arg;
    int result;

    switch (cmd)
    {
        case NAND_IOC_GET_CHIP_INFO:
            copy_to_user(pArg->pSM, pSM0, sizeof(FMI_SM_INFO_T));
            break;
        case NAND_IOC_IO_BLOCK_ERASE:
            nand_block_erase0(pArg->uBlock - pSM0->uLibStartBlock);
            break;
        case NAND_IOC_IO_PAGE_READ:
            nandpread0(pArg->uBlock - pSM0->uLibStartBlock, pArg->uPage, ioctl_buf);
            copy_to_user(pArg->pData, ioctl_buf, pSM0->nPageSize);
            break;
        case NAND_IOC_IO_PAGE_WRITE:
            write_nandloader = pArg->uWrite_nandloader; // write "check marker" or not
            if (access_ok(VERIFY_READ, pArg->pData, pSM0->nPageSize))
            {
                result = copy_from_user(ioctl_buf, pArg->pData, pSM0->nPageSize);
                if (result != 0)
                    printk("ERROR: nand_ioctl_0: copy_from_user() fail, return %d, block %d page %d\n", result, pArg->uBlock, pArg->uPage);
            }
            else
                printk("ERROR: nand_ioctl_0: access_ok() fail, block %d page %d\n", pArg->uBlock, pArg->uPage);
            result = nandpwrite0(pArg->uBlock - pSM0->uLibStartBlock, pArg->uPage, ioctl_buf);
            write_nandloader = 0;   // MUST set back to 0 for normal page writing (no "check marker")
            break;
        default:
            printk("Warning SCSI/NAND: Unknown command to SCSI/NAND IOCTL 0x%X.\n", cmd);
            break;
    }
        return 0;
}

int nandInit1(NDISK_T *NDISK_info)
{
        ENTER();
        return (sicSMInit(1, NDISK_info));
}

int nandpread1(int PBA, int page, u8 *buff)
{
        ENTER();
        return (sicSMpread(1, PBA, page, buff));
}

int nandpwrite1(int PBA, int page, u8 *buff)
{
        ENTER();
        return (sicSMpwrite(1, PBA, page, buff));
}

int nand_is_page_dirty1(int PBA, int page)
{
        ENTER();
        return (sicSM_is_page_dirty(1, PBA, page));
}

int nand_is_valid_block1(int PBA)
{
        ENTER();
        return (sicSM_is_valid_block(1, PBA));
}

int nand_block_erase1(int PBA)
{
        ENTER();
        return (sicSMblock_erase(1, PBA));
}

int nand_chip_erase1(void)
{
        ENTER();
        return (sicSMchip_erase(1));
}

int nand_ioctl_1(int param1, int param2, int param3, int param4)
{
        ENTER();
        return 0;
}

void fmiSMClose(int NandPort)
{
        ENTER();
        if (NandPort == 0)
        {
            _nand_init0 = 0;
            if (pSM0 != 0) {
                kfree(pSM0);
                pSM0 = 0;
            }
        }
        else
        {
            _nand_init1 = 0;
            if (pSM1 != 0) {
                kfree(pSM1);
                pSM1 = 0;
            }
        }

        if ((_nand_init0 == 0) && (_nand_init1 == 0)) {
        printk("nand close !!!\n");
            outpw(REG_FMICR, FMI_SM_EN);
            outpw(REG_SMCSR, inpw(REG_SMCSR)|0x06000000);
            outpw(REG_SMISR, 0xfff);
            outpw(REG_FMICR, 0x00);
            outpw(REG_GPDFUN, inpw(REG_GPDFUN) & ~0x0003FC00);      // enable NAND NWR/NRD/RB0/RB1 pins
            outpw(REG_GPEFUN, inpw(REG_GPEFUN) & ~0x00FF0000);      // enable NAND ALE/CLE/CS0/CS1 pins
        }
}
