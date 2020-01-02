
/* linux/driver/scsi/nuvoton_nand/w55fa93_nand.c
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
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/highmem.h>
#include <linux/blkdev.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/clk.h>

#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi.h>
#include <scsi/scsi_device.h>

#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/scatterlist.h>
#include <asm/cacheflush.h>
#include <asm/atomic.h>
#include <linux/dma-mapping.h>

#include <mach/w55fa93_reg.h>
#include <mach/irqs.h>
#include <mach/w55fa93_nand.h>
#include <mach/gnand/GNAND.h>
#include <mach/NandDrv.h>

/**for auto mount/umount**/
#include <linux/kmod.h>
#include <asm/uaccess.h>

#if defined(CONFIG_W55FA93_SYSMGR) || defined (CONFIG_W55FA93_SYSMGR_MODULE)
#include <mach/w55fa93_sysmgr.h>
extern void sysmgr_report(unsigned status);
#endif

// define DATE CODE and show it when running to make maintaining easy.
#define DATE_CODE   "20191218"

#if 1	// for nand card
#define NAND_CD_IRQ_NUM W55FA93_IRQ(3)  // nIRQ1
#endif

#define FALSE             0
#define TRUE              1

#undef 	outl
#undef 	inl
#define outl 	writel
#define inl 	readl

#define CONFIG_NANDCARD_SCSI

#if defined(CONFIG_W55FA93_NAND0)
	#define ONE_NAND

#elif defined(CONFIG_W55FA93_NAND1) && defined(CONFIG_NANDCARD_SCSI)
	#define ONE_NANDCARD

#elif defined(CONFIG_W55FA93_NAND_BOTH) && defined(CONFIG_NANDCARD_SCSI)
	#define ONE_NAND
	#define TWO_NAND
	#define NANDCARD_2_DETECT

#elif defined(CONFIG_W55FA93_NAND1) && defined(CONFIG_NANDCARD_MTD)
	#define ONE_NANDCARD_MTD

#elif defined(CONFIG_W55FA93_NAND_BOTH) && defined(CONFIG_NANDCARD_MTD)
	#define ONE_NAND
	#define TWO_NAND_MTD
	#define NANDCARD_2_DETECT_MTD

#else
	//#error ha ha ha
	#error Please select NAND according to the board
#endif

#if defined(CONFIG_NANDCARD_MTD) || defined(CONFIG_NAND0_MTD)
	extern int w55fa93_nand_init();
	extern void w55fa93_nand_exit();
#endif

#if 0
#define SD_DEBUG
#define DUMP
#define SD_DEBUG_ENABLE_ENTER_LEAVE
#define SD_DEBUG_ENABLE_MSG
#define SD_DEBUG_ENABLE_MSG2
#define SD_DEBUG_PRINT_LINE
#endif

#ifdef SD_DEBUG
#define PDEBUG(fmt, arg...)		printk(fmt, ##arg)
#else
#define PDEBUG(fmt, arg...)
#endif

#ifdef SD_DEBUG_PRINT_LINE
#define PRN_LINE()				PDEBUG("[%-20s] : %d\n", __FUNCTION__, __LINE__)
#else
#define PRN_LINE()
#endif

#ifdef SD_DEBUG_ENABLE_ENTER_LEAVE
#define ENTER()					PDEBUG("[%-20s] : Enter...\n", __FUNCTION__)
#define LEAVE()					PDEBUG("[%-20s] : Leave...\n", __FUNCTION__)
#else
#define ENTER()
#define LEAVE()
#endif

#ifdef SD_DEBUG_ENABLE_MSG
#define MSG(msg)				PDEBUG("[%-20s] : %s\n", __FUNCTION__, msg)
#else
#define MSG(msg)
#endif

#ifdef SD_DEBUG_ENABLE_MSG2
#define MSG2(fmt, arg...)			PDEBUG("[%-20s] : "fmt, __FUNCTION__, ##arg)
#define PRNBUF(buf, count)		{int i;MSG2("CID Data: ");for(i=0;i<count;i++)\
									PDEBUG("%02x ", buf[i]);PDEBUG("\n");}
#else
#define MSG2(fmt, arg...)
#define PRNBUF(buf, count)
#endif


#define Enable_IRQ(n)     				outl(1 << (n),REG_AIC_MECR)
#define Disable_IRQ(n)    				outl(1 << (n),REG_AIC_MDCR)

#define sd_inl(addr)				  	__raw_readl(addr)
#define sd_outl(data, addr)			__raw_writel(data, addr)

extern struct semaphore fmi_sem;
extern struct semaphore dmac_sem;

extern void GNAND_CloseNAND(NDISK_T *ptNDisk)  __attribute__ ((weak));
extern int  GNAND_read(NDISK_T *ptNDisk, u32 nSectorNo, int nSectorCnt, u8 *buff)  __attribute__ ((weak));
extern int  GNAND_InitNAND(NDRV_T *ndriver, NDISK_T *ptNDisk, s8 bEraseIfNotGnandFormat)  __attribute__ ((weak));
extern int  GNAND_write(NDISK_T *ptNDisk, u32 nSectorNo, int nSectorCnt, u8 *buff)  __attribute__ ((weak));
extern void fmiSMClose(int NandPort);
extern u8 *_gnand_pDMABuffer __attribute__ ((weak));
int nand_detect = 0; //ya
static atomic_t _wp = ATOMIC_INIT(2); // resource is busy if both read/detect sub this value to 0

#if 0
u8 static u8_buffer[512*8];
#endif


#if defined(ONE_NAND) || (defined(ONE_NANDCARD) && defined(CONFIG_W55FA93_CS0_RB0_PIN))
NDRV_T _nandDiskDriver0 = {
        nandInit0,
        nandpread0,
        nandpwrite0,
        nand_is_page_dirty0,
        nand_is_valid_block0,
        nand_ioctl_0,
        nand_block_erase0,
        nand_chip_erase0,
        0
};
#endif
NDISK_T *ptNDisk;

#if defined (TWO_NAND) || (defined(ONE_NANDCARD) && defined(CONFIG_W55FA93_CS1_RB1_PIN))
NDRV_T _nandDiskDriver1 = {
        nandInit1,
        nandpread1,
        nandpwrite1,
        nand_is_page_dirty1,
        nand_is_valid_block1,
        nand_ioctl_1,
        nand_block_erase1,
        nand_chip_erase1,
        0
};
#endif
NDISK_T *ptNDisk1;


struct nand_hostdata nand_host, nand_host1;
NAND_FLAG_SETTING nand_flagSetting;
unsigned int nand_registered = 0;
unsigned int nand_registered1 = 0;
volatile int nand_has_command = 0, nand_has_command1 = 0, nand_event = 0, nand_event1 = 0;
static DECLARE_WAIT_QUEUE_HEAD(sd_wq);
static DECLARE_WAIT_QUEUE_HEAD(scsi_wq);
static DECLARE_WAIT_QUEUE_HEAD(scsi_wq1);
static DECLARE_WAIT_QUEUE_HEAD(nand_event_wq);
static DECLARE_WAIT_QUEUE_HEAD(nand_event_wq1);
static DECLARE_MUTEX(sem);
static DECLARE_MUTEX(sem_r);
static DECLARE_MUTEX(sem_w);
static int nand_alloc0 = 0;

#if defined(NANDCARD_2_DETECT) || defined(ONE_NANDCARD) || defined(NANDCARD_2_DETECT_MTD) || defined(ONE_NANDCARD_MTD)
    static int nand_alloc1 = 0;
#endif

int nand_curMap[2];
int nand_curMap2[6];
//volatile int nandDetect, nand_init_Detect= 0;
volatile int nand_i_o = 0;
volatile int nand1_i_o = 0;
int nand_lastPort = 0;
static int register_disk_name = 0;

#if defined(CONFIG_W55FA93_SYSMGR) || defined (CONFIG_W55FA93_SYSMGR_MODULE)
extern char w55fa93_nand_dev;
#endif


u8 _fmi_pNANDBuffer1[1024*512] __attribute__((aligned (32)));
u8 *_fmi_gptr1;

u32 _fmi_ucNANDBuffer;
u32 _fmi_pNANDBuffer;
u32 can_remove = 1;


#if defined(NANDCARD_2_DETECT) || defined(ONE_NANDCARD) || defined(NANDCARD_2_DETECT_MTD) || defined(ONE_NANDCARD_MTD)
	static void nand_Long_TimeOut(unsigned long data);
	static struct timer_list nand_timer;	// handle debunce
#endif


char volatile  _fmi_bIsSMDataReady=FALSE;

#if defined(NANDCARD_2_DETECT) || defined(ONE_NANDCARD) || defined(NANDCARD_2_DETECT_MTD) || defined(ONE_NANDCARD_MTD)
// Simply return really Nand Card status in this moment that don't care about SD card debounce issue.
// If you care about SD card debounce issue, refer to nand_card_detect_irq().
int nand_card_status(void)
{
    #if defined (CONFIG_NANDCARD_GPD15)
        if ((inl(REG_GPIOD_PIN) & BIT15))
    #elif defined (CONFIG_NANDCARD_GPB6)
        if ((inl(REG_GPIOD_PIN) & BIT6))
    #else
        if (0)  // never detect NAND card since no define GPIO for it. always return 0 for Nand Card Insert.
    #endif
            return 1;   // nand card remove
        else
            return 0;   // nand card insert
}
#endif

static void nand_done(struct scsi_cmnd  *cmd )
{
        cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );
        wake_up_interruptible(&sd_wq);
}

#if defined(NANDCARD_2_DETECT) || defined(ONE_NANDCARD) || defined(NANDCARD_2_DETECT_MTD) || defined(ONE_NANDCARD_MTD)
	static void nand_Short_TimeOut(unsigned long data)
	{
	//        Enable_IRQ(NAND_CD_IRQ_NUM);
	        // set long timeout event
	        nand_timer.data = 0UL;
	        nand_timer.expires = jiffies +  NAND_LONG_DELAY;
	        nand_timer.function = nand_Long_TimeOut;
	        add_timer(&nand_timer);
	        return;
	}

	static void nand_Long_TimeOut(unsigned long data)
	{
	        struct nand_hostdata *dev = NULL;

	        dev = &nand_host1;

		#if defined (CONFIG_NANDCARD_GPB6)
	        if ((inl(REG_GPIOB_PIN) & 0x40)) {

		#elif (CONFIG_NANDCARD_GPD15)
	        if ((inl(REG_GPIOD_PIN) & 0x8000)) {

		#endif
	                nand_detect = 0;
	                if (nand_flagSetting.bCardExist1 == 0) {
	                        goto exit;
	                }
	                nand_event1 = SD_EVENT_REMOVE;
	                wake_up_interruptible(&nand_event_wq1);

	                printk("Card1 Removed\n");
	#if defined(CONFIG_W55FA93_SYSMGR) || defined (CONFIG_W55FA93_SYSMGR_MODULE)
	                sysmgr_report(SYSMGR_STATUS_NAND_REMOVE);
	                w55fa93_nand_dev = '-';
	#endif
	         //       nand_flagSetting.bCardExist1 = 0;
	                if (dev->state != SD_STATE_NOP) {
	                        dev->sense = SS_MEDIUM_NOT_PRESENT;
	                        goto nand_inter_quit;
	                }
	        }
	        else
	        {

	#if defined(CONFIG_W55FA93_SYSMGR) || defined (CONFIG_W55FA93_SYSMGR_MODULE)
	                printk("insert nand\n");
	                sysmgr_report(SYSMGR_STATUS_NAND_INSERT);
	#endif
	                nand_event1 = SD_EVENT_ADD;
	        //        nand_flagSetting.bCardExist1 = 1;
	                wake_up_interruptible(&nand_event_wq1);
	        } //card insert
	exit:
	        atomic_inc(&_wp);
	        Enable_IRQ(NAND_CD_IRQ_NUM);
	        return;

	nand_inter_quit:
	        atomic_inc(&_wp);
	        dev->state = SD_STATE_NOP;
	        Enable_IRQ(NAND_CD_IRQ_NUM);
	        return;
	}

#endif  // end of NANDCARD_2_DETECT || ...

static irqreturn_t fmi_interrupt(int irq, void *dev_id)
{
        unsigned int volatile isr;
//  printk("fmi_interrupt\n");
        // SM interrupt status
        isr = inpw(REG_SMISR);

        //DMA read/write transfer is done
        if (isr & 0x01) {
                _fmi_bIsSMDataReady = TRUE;
                outpw(REG_SMISR, 0x01);
                return IRQ_HANDLED;
        } else {
                return IRQ_NONE;
        }
}

#if defined(NANDCARD_2_DETECT) || defined(ONE_NANDCARD) || defined(NANDCARD_2_DETECT_MTD) || defined(ONE_NANDCARD_MTD)

	// for nand card
static irqreturn_t nand_card_detect_irq(int irq, void *dev_id)
{
        u32 src;

        Disable_IRQ(NAND_CD_IRQ_NUM);

	#if defined (CONFIG_NANDCARD_GPB6)
        src = inl(REG_IRQTGSRC0);
        // clear source
        outl(src & 0x00400000, REG_IRQTGSRC0);
	#elif (CONFIG_NANDCARD_GPD15)
        src = inl(REG_IRQTGSRC1);
        outl(src & 0x80000000, REG_IRQTGSRC1);
	#endif

        del_timer(&nand_timer);
        nand_timer.data = 0UL;
        nand_timer.expires = jiffies +  NAND_SHORT_DELAY;
        nand_timer.function = nand_Short_TimeOut;
        add_timer(&nand_timer);

//        Enable_IRQ(NAND_CD_IRQ_NUM);
        return IRQ_HANDLED;
}
#endif

static u16 inline get_be16(u8 *buf)
{
        return ((u16) buf[0] << 8) | ((u16) buf[1]);
}

static u32 inline get_be32(u8 *buf)
{
        return ((u32) buf[0] << 24) | ((u32) buf[1] << 16) |
               ((u32) buf[2] << 8) | ((u32) buf[3]);
}

static void inline put_be16(u16 val, u8 *buf)
{
        buf[0] = val >> 8;
        buf[1] = val;
}

static void inline put_be32(u32 val, u8 *buf)
{
        buf[0] = val >> 24;
        buf[1] = val >> 16;
        buf[2] = val >> 8;
        buf[3] = val;
}

static void nand_make_sense_buffer(struct nand_hostdata *dev,int key, int asc, int ascq)
{
        unsigned char *sense_buffer;

        sense_buffer = dev->cmd->sense_buffer;
        memset(sense_buffer, 0, 18);

        sense_buffer[0] = 0x70;		// error code
        sense_buffer[2] = key;
        sense_buffer[7] = 0x0A;		// sdditional sense length
        sense_buffer[12] = asc;
        sense_buffer[13] = ascq;
}


static int nand_nand_card_reset(void)
{
        int retval;
        int nSectorPerPage;
        struct nand_hostdata *dev=NULL;

#if defined(ONE_NAND) || (defined(ONE_NANDCARD) && defined(CONFIG_W55FA93_CS0_RB0_PIN))
        if (nand_flagSetting.curCard == 0)
                dev = &nand_host;
#endif
#if defined(TWO_NAND) || (defined(ONE_NANDCARD) && defined(CONFIG_W55FA93_CS1_RB1_PIN))
        if (nand_flagSetting.curCard == 1)
                dev = &nand_host1;
#endif
        ENTER();

        mdelay(50);
#if defined(ONE_NAND) || (defined(ONE_NANDCARD) && defined(CONFIG_W55FA93_CS0_RB0_PIN))
        if (dev->myID == 0) {
                ptNDisk = (NDISK_T *)kmalloc(sizeof(NDISK_T),GFP_KERNEL);
                if (ptNDisk == NULL)
                        printk("malloc error!!\n");

                retval=GNAND_InitNAND(&_nandDiskDriver0, ptNDisk, FALSE);
                if (retval) {
                        printk("GNAND init failed !!!!!!!! \n");
                        return SD_FAILED;
                }
                nSectorPerPage = ptNDisk->nPageSize / 512;
                nand_host.nTotalSectors = ptNDisk->nZone * (ptNDisk->nLBPerZone-1) * ptNDisk->nPagePerBlock * nSectorPerPage;

                nand_host.nSectorSize = 512;
                nand_host.nCapacityInByte = nand_host.nTotalSectors * nand_host.nSectorSize;

                printk("on board NAND size %d MB (%d sectors)\n", nand_host.nTotalSectors/2048, nand_host.nTotalSectors);
        }
#endif
#if defined(TWO_NAND) || (defined(ONE_NANDCARD) && defined(CONFIG_W55FA93_CS1_RB1_PIN))
        if (dev->myID == 1) {
                ptNDisk1 = (NDISK_T *)kmalloc(sizeof(NDISK_T),GFP_KERNEL);
                if (ptNDisk1 == NULL)
                        printk("malloc error!!\n");

                retval=GNAND_InitNAND(&_nandDiskDriver1, ptNDisk1, FALSE);
                if (retval) {
                        printk("GNAND init failed - nand1 0x%x! \n", retval);
                        kfree(ptNDisk1);
                        ptNDisk1 = NULL;
                        return SD_FAILED;
                }
                nSectorPerPage = ptNDisk1->nPageSize / 512;
                nand_host1.nTotalSectors = ptNDisk1->nZone * (ptNDisk1->nLBPerZone-1) * ptNDisk1->nPagePerBlock * nSectorPerPage;

                nand_host1.nSectorSize = 512;
                nand_host1.nCapacityInByte = nand_host1.nTotalSectors * nand_host1.nSectorSize;

                printk("NAND card size %d MB (%d sectors)\n", nand_host1.nTotalSectors/2048, nand_host1.nTotalSectors);
        }
#endif
        LEAVE();
        return SD_SUCCESS;
}


static void nand_host_reset(void)
{
        struct nand_hostdata *dev=NULL;

        if (nand_flagSetting.curCard == 0)
                dev = &nand_host;
        if (nand_flagSetting.curCard == 1)
                dev = &nand_host1;

        ENTER();

        /* init SM interface */
        outpw(REG_SMCSR, inpw(REG_SMCSR)&0xfdffff80);

        if (down_interruptible(&dmac_sem))
                return;
        while (sd_inl(REG_DMACCSR)&DMACCSR_FMI_BUSY); //Wait IP finished... for safe

        // enable all
        sd_outl(sd_inl(REG_DMACCSR) | DMACCSR_DMACEN,REG_DMACCSR); //enable DMAC for FMI

        /* enable all interrupt */
        sd_outl(DMACIER_TABORT_IE,REG_DMACIER); //Enable target abort interrupt generation during DMA transfer
        sd_outl(FMIIER_DTA_IE, REG_FMIIER); //Enable DMAC READ/WRITE target abort interrupt generation

        nand_host.nSectorSize = 512;
        nand_host1.nSectorSize = 512;

        dev->state = 0;
        dev->sense = 0;

        nand_flagSetting.bCrcCheck = 1;
        nand_flagSetting.needReset = 1;

        nand_host.myID = 0;
        nand_host1.myID = 1;

        up(&dmac_sem);
        LEAVE();
}


static int nand_card_init(void)
{
        int retval = SD_FAILED;
        struct nand_hostdata *dev=NULL;

        if (nand_flagSetting.curCard == 0)
                dev = &nand_host;
        if (nand_flagSetting.curCard == 1)
                dev = &nand_host1;

        ENTER();

        nand_host_reset();

        if (nand_flagSetting.curCard == 0) {
                if ( nand_flagSetting.bCardExist == 0) {
                        up(&fmi_sem);
                        return SD_REMOVED;
                }
        }

        if (nand_flagSetting.curCard == 1) {
                if ( nand_flagSetting.bCardExist1 == 0) {
                        up(&fmi_sem);
                        return SD_REMOVED;
                }
        }

        retval = nand_nand_card_reset();
        if (retval)
                return retval;

        if ((nand_flagSetting.curCard == 0) &&(nand_flagSetting.bCardExist == 0))
                nand_flagSetting.bWriteProtect = 0;
        if ((nand_flagSetting.curCard == 1) &&(nand_flagSetting.bCardExist1 == 0))
                nand_flagSetting.bWriteProtect1 = 0;
        /* check for write protect */
        if (nand_flagSetting.curCard == 0) {
                if ((ptNDisk->vendor_ID & 0xff) == 0xC2)
                        nand_flagSetting.bWriteProtect = 1;
                else
                        nand_flagSetting.bWriteProtect = 0;
        }
        if (nand_flagSetting.curCard == 1) {
                if ((ptNDisk1->vendor_ID & 0xff) == 0xC2)
                        nand_flagSetting.bWriteProtect1 = 1;
                else
                        nand_flagSetting.bWriteProtect1 = 0;
        }

        MSG2("Init SD Card Result = %d\n", retval);
        LEAVE();
        return retval;
}


/*static*/
int nand_check_valid_medium(struct nand_hostdata *dev)
{
        ENTER();

        if (dev->myID == 0) { //card 0
        	nand_flagSetting.curCard = 0;

                if (nand_flagSetting.bCardExist == 0) {
                        MSG( "card not exist\n" );
                        return(-1);
                }

                if (nand_flagSetting.bMediaChanged != 0 ) {
                        nand_flagSetting.bMediaChanged = 0;
                        if (!nand_card_init())
                                nand_flagSetting.bInitSuccess = 1;
                        else {
                                nand_flagSetting.bInitSuccess = 0;
                                return(-1);
                        }
                }
        }

        if (dev->myID == 1) { //card 1
        	nand_flagSetting.curCard = 1;

                if (nand_flagSetting.bCardExist1 == 0) {
                        MSG( "card 1 not exist\n" );
                        return(-1);
                }

                if (nand_flagSetting.bMediaChanged1 != 0 ) {
                        nand_flagSetting.bMediaChanged1 = 0;
                        if (!nand_card_init())
                                nand_flagSetting.bInitSuccess1 = 1;
                        else {
                                nand_flagSetting.bInitSuccess1 = 0;
                                return(-1);
                        }
                }
        }
        LEAVE();
        return(0);
}

static unsigned char *nand_get_buffer(struct scsi_cmnd *cmd, int * length)
{
        struct scatterlist *p;
        unsigned char * buf;
        ENTER();

        p = (struct scatterlist *)cmd->sdb.table.sgl;
        buf =(unsigned char *)page_address(sg_page(p)) + p->offset;
        *length = p->length;

        LEAVE();
        return buf;
}

static int nand_test_unit_ready(struct nand_hostdata *dev)
{
        struct scsi_cmnd  * cmd;
        int retval = 0;
        ENTER();
        cmd = dev->cmd;

        nand_check_valid_medium(dev);

        if (dev->myID == 0) { //card 0
                if (nand_flagSetting.bCardExist != 0) {
                        if (nand_flagSetting.bMediaChanged == 0) {
                                if (nand_flagSetting.bInitSuccess != 0) {
                                        // SenseKey: SCSI_SENSE_NO_SENSE
                                        // AdditionalSenseCode: SCSI_ADSENSE_NO_SENSE
                                        nand_make_sense_buffer(dev,0x00, 0x00, 0x00 );
                                        retval = 0;
                                } else {
                                        // SenseKey: SCSI_SENSE_MEDIUM_ERROR
                                        // AdditionalSenseCode: SCSI_ADSENSE_INVALID_MEDIA
                                        nand_make_sense_buffer(dev,0x03, 0x30, 0x00 );
                                        retval = 1;
                                }
                        } else {
                                nand_flagSetting.bMediaChanged = 0;
                                // SenseKey: SCSI_SENSE_UNIT_ATTENTION
                                // AdditionalSenseCode: SCSI_ADSENSE_MEDIUM_CHANGED
                                nand_make_sense_buffer(dev,0x06, 0x28, 0x00 );
                                cmd->result = SD_CMD_RESULT(DID_NO_CONNECT, DISCONNECT, SAM_STAT_CONDITION_MET);
                                retval = 1;
                        }
                } else {
                        // SenseKey: SCSI_SENSE_NOT_READY
                        // AdditionalSenseCode: SCSI_ADSENSE_NO_MEDIA_IN_DEVICE
                        nand_make_sense_buffer(dev,0x02, 0x3a, 0x00 );

                        // NOT_READY
                        //nand_make_sense_buffer(dev,0x02, 0x00, 0x00 );
                        //cmd->result = SD_CMD_RESULT(DID_NO_CONNECT, DISCONNECT, CHECK_CONDITION);
                        retval = 1;
                }
        } // myID=0

        if (dev->myID == 1) { //card 1
                if (nand_flagSetting.bCardExist1 != 0) {
                        if (nand_flagSetting.bMediaChanged1 == 0) {
                                if (nand_flagSetting.bInitSuccess1 != 0) {
                                        // SenseKey: SCSI_SENSE_NO_SENSE
                                        // AdditionalSenseCode: SCSI_ADSENSE_NO_SENSE
                                        nand_make_sense_buffer(dev,0x00, 0x00, 0x00 );
                                        retval = 0;
                                } else {
                                        // SenseKey: SCSI_SENSE_MEDIUM_ERROR
                                        // AdditionalSenseCode: SCSI_ADSENSE_INVALID_MEDIA
                                        nand_make_sense_buffer(dev,0x03, 0x30, 0x00 );
                                        retval = 1;
                                }
                        } else {
                                nand_flagSetting.bMediaChanged1 = 0;
                                // SenseKey: SCSI_SENSE_UNIT_ATTENTION
                                // AdditionalSenseCode: SCSI_ADSENSE_MEDIUM_CHANGED
                                nand_make_sense_buffer(dev,0x06, 0x28, 0x00 );
                                cmd->result = SD_CMD_RESULT(DID_NO_CONNECT, DISCONNECT, SAM_STAT_CONDITION_MET);
                                retval = 1;
                        }
                } else {
                        // SenseKey: SCSI_SENSE_NOT_READY
                        // AdditionalSenseCode: SCSI_ADSENSE_NO_MEDIA_IN_DEVICE
                        nand_make_sense_buffer(dev,0x02, 0x3a, 0x00 );

                        // NOT_READY
                        //nand_make_sense_buffer(dev,0x02, 0x00, 0x00 );
                        //cmd->result = SD_CMD_RESULT(DID_NO_CONNECT, DISCONNECT, CHECK_CONDITION);
                        retval = 1;
                }
        } // myID=1

        LEAVE();
        return retval;
}


static int nand_scsi_read(struct scsi_cmnd  *cmd , struct nand_hostdata *dev)
{
        unsigned int count, lba, retry=0;
        int volatile DMAoffset = 0, status = 0;
        int curDMAAddr;
        struct scatterlist *curList;

        ENTER();
//        printk("nand_scsi_read  !!!" );
        if ( nand_test_unit_ready(dev) )
                goto quit;

#if defined (CONFIG_W55FA93_NAND_BOTH)
        if ((register_disk_name == 1)&&((int)cmd->request->rq_disk->disk_name!= 0xc) && (dev->myID == 1) ) { //update mapping
                register_disk_name = 0; //only when there is a new car insert or remove,it needs update.

//			printk("register_disk_name, cmd->request->rq_disk->disk_name = 0x%x  !!! \n", cmd->request->rq_disk->disk_name);

#if defined(CONFIG_W55FA93_SYSMGR) || defined (CONFIG_W55FA93_SYSMGR_MODULE)
            w55fa93_nand_dev = cmd->request->rq_disk->disk_name[2];
#endif
		}
#else
        if ((register_disk_name == 1)&&((int)cmd->request->rq_disk->disk_name!= 0xc)) { //update mapping
                register_disk_name = 0; //only when there is a new car insert or remove,it needs update.

#if defined(CONFIG_W55FA93_SYSMGR) || defined (CONFIG_W55FA93_SYSMGR_MODULE)
            w55fa93_nand_dev = cmd->request->rq_disk->disk_name[2];
#endif
    	}
#endif

        if ( cmd->cmnd[0] == READ_6) {
                lba = ((cmd->cmnd[1] & 0x1f) << 16) + get_be16(&cmd->cmnd[2]);
                count = (cmd->cmnd[4] & 0xff);
        } else {
                lba = get_be32(&cmd->cmnd[2]);
                count = get_be16(&cmd->cmnd[7]);
        }

//        printk("--Read from = %d(lba),  count = %d   \n", lba, count);

        if ( lba > dev->nTotalSectors || (lba + count) > dev->nTotalSectors) {
                //printk("--Read from = %d(lba),  count = %d   \n", lba, count);
                dev->sense = SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
                goto quit_with_make_sense;
        }

	    dev->firstList = (struct scatterlist *)cmd->sdb.table.sgl;
        dev->curList = 0;
        dev->curOffset = 0;
        dev->sense = SS_NO_SENSE;

        wait_event_interruptible(sd_wq, dev->state == SD_STATE_NOP);

//	printk("nand_scsi_read--Read from = %d(lba),  count = %d, deviceID = %d \n", lba, count, dev->myID);

//  memset(_fmi_pNANDBuffer1, 0x0, 1024*512);

        if (dev->myID == 0) {
                dev->state = SD_STATE_READ;
_gnand_read_retry0_:

                status = GNAND_read(ptNDisk,lba,count,_fmi_pNANDBuffer1);
                if ((status == FMI_SM_RB_ERR) && (retry < 1)) {
                        retry++;
                        printk("gnand_read retry 0 <0x%x>\n", sd_inl(REG_SMISR));
                        goto _gnand_read_retry0_;
                } else if (status < 0) {
                        dev->sense = SS_UNRECOVERED_READ_ERROR;
                        goto quit_with_make_sense;
                }

                /* connect with SCSI */
                curList = &nand_host.firstList[nand_host.curList];
                //  MSG2("curList->offset  %d\n",curList->offset);
                MSG2("sd_host.curOffset  %d\n",nand_host.curOffset);
                MSG2("curList->length  %d\n",(curList->length)/512);
                MSG2("count  %d\n",count);
                curDMAAddr = (unsigned int )page_address(sg_page(curList)) + curList->offset + nand_host.curOffset;

                while ((curList->length)<(count*512)) {
                        memcpy((char *)curDMAAddr,(char *) _fmi_pNANDBuffer1+DMAoffset, curList->length);

                        count = count-(curList->length)/512;
                        DMAoffset+=(curList->length);
                        nand_host.curOffset = 0;
                        nand_host.curList ++;
                        curList = &nand_host.firstList[nand_host.curList];

                        curDMAAddr = (unsigned int )page_address(sg_page(curList)) + curList->offset + nand_host.curOffset;

                        MSG2("curList->length  %d\n",(curList->length)/512);
                        MSG2("count  %d\n",count);
                }
                if ((curList->length)==(count*512)) {
                        MSG2("=\n");
                        MSG2("curList->length  %d\n",curList->length/512);

                        memcpy((char *)curDMAAddr, (char *)_fmi_pNANDBuffer1+DMAoffset, curList->length);
                        DMAoffset=0;
                }
        }

        if (dev->myID == 1) {
                dev->state = SD_STATE_READ;
                can_remove = 0;
                if (nand_detect == 0) {
                        dev->sense = SS_MEDIUM_NOT_PRESENT;
                        goto quit_with_make_sense;
                }
_gnand_read_retry1_:
        //        printk("GNAND_read_1 \n");
                status = GNAND_read(ptNDisk1,lba,count,_fmi_pNANDBuffer1);
        //       printk("GNAND_read_1, status = %2x \n", status);
                if ((status == FMI_SM_RB_ERR) && (retry < 1)) {
                        retry++;
                        printk("gnand_read retry 1\n");
                        can_remove = 1;
                        if (nand_detect == 0) {
                                dev->sense = SS_MEDIUM_NOT_PRESENT;
                                goto quit_with_make_sense;
                        } else
                        goto _gnand_read_retry1_;
                } else if (status < 0) {
                        can_remove = 1;
                        if (nand_detect == 0)
                                dev->sense = SS_MEDIUM_NOT_PRESENT;
                        else
                        dev->sense = SS_UNRECOVERED_READ_ERROR;
                        goto quit_with_make_sense;
                }
                can_remove = 1;
                /* connect with SCSI */
                curList = &nand_host1.firstList[nand_host1.curList];
                //  MSG2("curList->offset  %d\n",curList->offset);

                MSG2("sd_host.curOffset  %d\n",nand_host1.curOffset);
                MSG2("curList->length  %d\n",(curList->length)/512);
                MSG2("count  %d\n",count);
                curDMAAddr = (unsigned int )page_address(sg_page(curList)) + curList->offset + nand_host1.curOffset;

                while ((curList->length)<(count*512)) {
                        memcpy((char *)curDMAAddr,(char *) _fmi_pNANDBuffer1+DMAoffset, curList->length);

                        count = count-(curList->length)/512;
                        DMAoffset+=(curList->length);
                        nand_host1.curOffset = 0;
                        nand_host1.curList ++;
                        curList = &nand_host1.firstList[nand_host1.curList];

                        curDMAAddr = (unsigned int )page_address(sg_page(curList)) + curList->offset + nand_host1.curOffset;

                        MSG2("curList->length  %d\n",(curList->length)/512);
                        MSG2("count  %d\n",count);
                }
                if ((curList->length)==(count*512)) {
                        MSG2("=\n");
                        MSG2("curList->length  %d\n",curList->length/512);

                        memcpy((char *)curDMAAddr, (char *)_fmi_pNANDBuffer1+DMAoffset, curList->length);
                        DMAoffset=0;
                }
        }

        //=====================
        nand_done(cmd );
        dev->state = SD_STATE_NOP;
        //wait_event_interruptible(sd_wq, dev->state == SD_STATE_NOP);

        LEAVE();
        return 0;

quit_with_make_sense:
        nand_make_sense_buffer(dev,dev->sense >> 16,
                               dev->sense >> 8,
                               dev->sense);
quit:
        dev->state = SD_STATE_NOP;
        return -1;
}


static int nand_scsi_write(struct scsi_cmnd  *cmd, struct nand_hostdata *dev)
{
        unsigned int count, lba,write_count;
        int volatile DMAoffset = 0;
        int curDMAAddr;
        struct scatterlist *curList;

        ENTER();
//        printk("nand_scsi_write  !!!" );

#if defined (TWO_NAND)

	#ifdef CONFIG_W55FA93_NAND1_WRITE_ENABLE
        if (1) {
	#else
        if (dev->myID == 0) {
	#endif
                while (atomic_dec_and_test(&_wp)) { // busy?
                        atomic_inc(&_wp);
                        msleep(20);
                }
                //outl(inl(REG_IRQENGPA) & ~0x00100010, REG_IRQENGPA); // disable card detect interrupt
        }
        else
        {
                dev->sense = SS_WRITE_PROTECTED;
                goto quit_with_make_sense;
        }
#endif

        if ( nand_test_unit_ready(dev)) {
                // printk("tur failed\n");
                goto quit;
        }

        if ((dev->myID == 0) &&(nand_flagSetting.bWriteProtect == 1)) {
                dev->sense = SS_WRITE_PROTECTED;
                printk("nand0: wp failed\n");
                goto quit_with_make_sense;
        }
        if ((dev->myID == 1) &&(nand_flagSetting.bWriteProtect1 == 1)) {
                dev->sense = SS_WRITE_PROTECTED;
                //printk("nand1: wp failed\n");
                goto quit_with_make_sense;
        }

        if ( cmd->cmnd[0] == WRITE_6) {
                lba = ((cmd->cmnd[1] & 0x1f) << 16) + get_be16(&cmd->cmnd[2]);
                count = (cmd->cmnd[4] & 0xff);
        } else {
                lba = get_be32(&cmd->cmnd[2]);
                count = get_be16(&cmd->cmnd[7]);
        }

        if ( lba > dev->nTotalSectors || (lba + count) > dev->nTotalSectors) {
                //printk("--write to = %d(lba),  length = %d  \n ", lba, count);
                //printk("ofr failed\n");
                dev->sense = SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
                goto quit_with_make_sense;
        }

        dev->firstList = (struct scatterlist *)cmd->sdb.table.sgl;
        dev->curOffset = 0;
        dev->curList = 0;
        dev->sense = SS_NO_SENSE;
        write_count=count;
        wait_event_interruptible(sd_wq, dev->state == SD_STATE_NOP);

//        printk("nand_scsi_write--write to = %d(lba),  count = %d  \n ", lba, count);

        /* connect with SCSI */
        if (dev->myID == 0) {
                curList = &nand_host.firstList[nand_host.curList];
                //  MSG2("curList->offset  %d\n",curList->offset);
                MSG2("sd_host.curOffset  %d\n",nand_host.curOffset);
                MSG2("curList->length  %d\n",(curList->length)/512);
                MSG2("count  %d\n",count);
                curDMAAddr = (unsigned int )page_address(sg_page(curList)) + curList->offset + nand_host.curOffset;

                while ((curList->length)<(count*512)) {
                        memcpy((char *)_fmi_pNANDBuffer1+DMAoffset, (char *)curDMAAddr, curList->length);
                        MSG2("_fmi_pSMBuffer+DMAoffset  %x\n",_fmi_pNANDBuffer1+DMAoffset);
                        count = count-(curList->length)/512;
                        DMAoffset+=(curList->length);
                        nand_host.curOffset = 0;
                        nand_host.curList ++;
                        curList = &nand_host.firstList[nand_host.curList];

                        curDMAAddr = (unsigned int )page_address(sg_page(curList)) + curList->offset + nand_host.curOffset;

                        MSG2("curList->length  %d\n",(curList->length)/512);
                        MSG2("count  %d\n",count);
                }

                if ((curList->length)==(count*512)) {
                        MSG2("=\n");
                        MSG2("curList->length  %d\n",curList->length/512);
                        MSG2("DMAoffset  %d\n",DMAoffset);

                        memcpy((char *)_fmi_pNANDBuffer1+DMAoffset, (char *)curDMAAddr, curList->length);
                        DMAoffset=0;
                }
                dev->state = SD_STATE_WRITE;
                if (GNAND_write(ptNDisk,lba,write_count,_fmi_pNANDBuffer1)) {
                        dev->sense = SS_WRITE_ERROR;
                        MSG2("GNAND_write Fail !!! \n");
                        goto quit_with_make_sense;
                }
        }

        if (dev->myID == 1) {
                curList = &nand_host1.firstList[nand_host1.curList];
                //  MSG2("curList->offset  %d\n",curList->offset);
                MSG2("sd_host.curOffset  %d\n",nand_host1.curOffset);
                MSG2("curList->length  %d\n",(curList->length)/512);
                MSG2("count  %d\n",count);
                curDMAAddr = (unsigned int )page_address(sg_page(curList)) + curList->offset + nand_host1.curOffset;

                while ((curList->length)<(count*512)) {
                        memcpy((char *)_fmi_pNANDBuffer1+DMAoffset, (char *)curDMAAddr, curList->length);
                        MSG2("_fmi_pSMBuffer+DMAoffset  %x\n",_fmi_pNANDBuffer1+DMAoffset);
                        count = count-(curList->length)/512;
                        DMAoffset+=(curList->length);
                        nand_host1.curOffset = 0;
                        nand_host1.curList ++;
                        curList = &nand_host1.firstList[nand_host1.curList];

                        curDMAAddr = (unsigned int )page_address(sg_page(curList)) + curList->offset + nand_host1.curOffset;

                        MSG2("curList->length  %d\n",(curList->length)/512);
                        MSG2("count  %d\n",count);
                }

                if ((curList->length)==(count*512)) {
                        MSG2("=\n");
                        MSG2("curList->length  %d\n",curList->length/512);

                        memcpy((char *)_fmi_pNANDBuffer1+DMAoffset, (char *)curDMAAddr, curList->length);
                        DMAoffset=0;
                }
                dev->state = SD_STATE_WRITE;
                can_remove = 0;
                if (nand_detect == 0) {
                        dev->sense = SS_MEDIUM_NOT_PRESENT;
                        goto quit_with_make_sense;
                }
                if (GNAND_write(ptNDisk1,lba,write_count,_fmi_pNANDBuffer1)) {
                        can_remove = 1;
                        if (nand_detect == 0)
                                dev->sense = SS_MEDIUM_NOT_PRESENT;
                        else
                        dev->sense = SS_WRITE_ERROR;
                        goto quit_with_make_sense;
                }
                can_remove = 1;
        }

        //=============
        nand_done(cmd );
        dev->state = SD_STATE_NOP;
        //wait_event_interruptible(sd_wq, dev->state == SD_STATE_NOP);

        LEAVE();

#ifdef CONFIG_W55FA93_NAND1_WRITE_ENABLE
        if (1) {
#else
        if (dev->myID == 0) {
#endif
                //outl(inl(REG_IRQENGPA) | 0x00100010, REG_IRQENGPA); // enable card detect interrupt
                atomic_inc(&_wp);
        }
        return 0;

quit_with_make_sense:
        nand_make_sense_buffer(dev,dev->sense >> 16,
                               dev->sense >> 8,
                               dev->sense);
quit:
#ifdef CONFIG_W55FA93_NAND1_WRITE_ENABLE
        if (1) {
#else
        if (dev->myID == 0) {
#endif
                //outl(inl(REG_IRQENGPA) | 0x00100010, REG_IRQENGPA); // enable card detect interrupt
                atomic_inc(&_wp);
        }

        dev->state = SD_STATE_NOP;
        return -1;
}


static void nand_scsi_start_stop(struct nand_hostdata *dev)
{
        struct scsi_cmnd  *cmd;
        ENTER();
        cmd = dev->cmd;

        if (cmd->cmnd[4] & 0x01) {		/* start */
                if (! nand_test_unit_ready(dev)) {
                        cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );
                }
        } else
                cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );

        LEAVE();
}


static void nand_scsi_request_sense(struct scsi_cmnd  *cmd)
{
        int len;
        unsigned char *buffer = nand_get_buffer(cmd, &len);
        ENTER();

        memcpy(buffer, cmd->sense_buffer, 18);
        cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );

        LEAVE();
}


static void nand_scsi_media_removal(struct nand_hostdata *dev)
{
        struct scsi_cmnd  *cmd = dev->cmd;
        ENTER();
        //prevent removal cmnd is illegal since SD card can be removable
        if ( ( cmd->cmnd[4] & 0x01 ) )  {
                // SenseKey: SCSI_SENSE_ILLEGAL_REQUEST
                // AdditionalSenseCode: SCSI_ADSENSE_ILLEGAL_COMMAND
                nand_make_sense_buffer(dev,0x05, 0x20, 0x00 );
        } else {
                cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );
        }
        LEAVE();
}


static void nand_scsi_test_unit_ready(struct nand_hostdata *dev)
{
        struct scsi_cmnd  *cmd = dev->cmd;
        ENTER();
        if (!nand_test_unit_ready(dev) )
                cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );
        LEAVE();
}


static void nand_scsi_inquiry(struct nand_hostdata *dev)
{
        int len;
        struct scsi_cmnd  *cmd = dev->cmd;
        unsigned char *buf = nand_get_buffer(cmd, &len);

        static char vendor_id[] = "NUVOTON";
        static char product_id[] = "GNAND DRIVER";
        static char release_id[]="2.00";

        ENTER();

        if (dev->myID == 0) {
                if (nand_registered == 0) {
                        //printk("nand_registered\n");
                        nand_registered = 1;
                        // stuff necessary inquiry data

                        memset(buf, 0, 36);
                        buf[1] = 0x80;	/* removable */
                        buf[2] = 0;		// ANSI SCSI level 2
                        buf[3] = 2;		// SCSI-2 INQUIRY data format //2
                        buf[4] = 0x1f;		// Additional length
                        // No special options

                        sprintf(buf + 8, "%-8s%-16s%-4s", vendor_id, product_id,
                                release_id);

                        cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );
                } else
                        cmd->result = SD_CMD_RESULT(DID_NO_CONNECT, 0x00, 0x00);
        }

        if (dev->myID == 1) {
                if (nand_registered1 == 0) {
                        printk("nand_registered1\n");
                        nand_registered1 = 1;
                        // stuff necessary inquiry data

                        memset(buf, 0, 36);
                        buf[1] = 0x80;	/* removable */
                        buf[2] = 0;		// ANSI SCSI level 2
                        buf[3] = 2;		// SCSI-2 INQUIRY data format //2
                        buf[4] = 0x1f;		// Additional length
                        // No special options

                        sprintf(buf + 8, "%-8s%-16s%-4s", vendor_id, product_id,
                                release_id);

                        cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );
                } else
                        cmd->result = SD_CMD_RESULT(DID_NO_CONNECT, 0x00, 0x00);
        }
        LEAVE();
}


static void nand_scsi_mode_sense(struct nand_hostdata *dev)
{
        int bProtectFlag, len;
        struct scsi_cmnd  *cmd = dev->cmd;
        unsigned char *buf = nand_get_buffer(cmd, &len);
        ENTER();

        if ( nand_test_unit_ready(dev) )
                return;

        if (dev->myID == 0) {
                if ( nand_flagSetting.bCardExist ) {
                        memset(buf, 0, 8);
                        bProtectFlag = 0;
                        if (nand_flagSetting.bWriteProtect)
                                bProtectFlag = 0x80;

                        if ( cmd->cmnd[0] == MODE_SENSE ) {
                                buf[0] = 0x03;
                                buf[2] = bProtectFlag;
                        } else {
                                buf[1] = 0x06;
                                buf[3] = bProtectFlag;
                        }

                        cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );
                } else { // card is not in
                        nand_make_sense_buffer(dev, 0x02, 0x3a, 0x00 );
                        MSG( "SD 0 card may not be inserted!\n" );
                }
        } // myID = 0

        if (dev->myID == 1) {
                if ( nand_flagSetting.bCardExist1 ) {
                        memset(buf, 0, 8);
#ifdef CONFIG_W55FA93_NAND1_WRITE_ENABLE
                        bProtectFlag = 0;
                        if (nand_flagSetting.bWriteProtect1)
#endif
                                bProtectFlag = 0x80;

                        if ( cmd->cmnd[0] == MODE_SENSE ) {
                                buf[0] = 0x03;
                                buf[2] = bProtectFlag;
                        } else {
                                buf[1] = 0x06;
                                buf[3] = bProtectFlag;
                        }

                        cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );
                } else { // card is not in
                        nand_make_sense_buffer(dev, 0x02, 0x3a, 0x00 );
                        MSG( "SD 1 card may not be inserted!\n" );
                }
        } // myID = 1
        LEAVE();
}


static void nand_scsi_read_capacity(struct nand_hostdata *dev)
{
        struct scsi_cmnd  *cmd = dev->cmd;
        int len;
        unsigned char *buf = nand_get_buffer(cmd, &len);

        ENTER();

        if (nand_test_unit_ready(dev)) {
                MSG( "SCSI_READ_CAPACITY - The unit not ready\r\n" );
                return;
        }

        memset(buf, 0, 8);
        put_be32(dev->nTotalSectors - 1, &buf[0]);	// Max logical block
        put_be32(512, &buf[4]);				// Block length
        cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_GOOD );

        LEAVE();
        return;
}

static void nand_scsi_process_cmd(struct nand_hostdata *dev)
{
        struct scsi_cmnd  * cmd;
        down_interruptible(&sem_r);

        ENTER();

//        printk("has command, CurCard = %d !!!\n", nand_flagSetting.curCard);
//        printk("dev->myID = %d !!!\n", dev->myID);

        cmd = dev->cmd;
        if (down_interruptible(&fmi_sem)) {
                cmd->result = SD_CMD_RESULT( DID_BUS_BUSY, 0, 0 );
                goto done;
        }
        cmd->result = SD_CMD_RESULT( DID_OK, COMMAND_COMPLETE, SAM_STAT_CHECK_CONDITION );

        if (nand_check_valid_medium(dev) < 0) {
                dev->sense = SS_MEDIUM_NOT_PRESENT;
                dev->cmd->result = SD_CMD_RESULT(DID_NO_CONNECT, DISCONNECT, SAM_STAT_CHECK_CONDITION);
                goto quit;
        }

        MSG2("cmd->cmnd[0] = 0x%x !!!\n", cmd->cmnd[0]);

        switch (cmd->cmnd[0]) {

        case START_STOP:
                PDEBUG("SC_START_STOP_UNIT\n");
                nand_scsi_start_stop(dev);
                break;

        case REQUEST_SENSE:
                PDEBUG("SC_PREVENT_ALLOW_MEDIUM_REMOVAL\n");
                nand_scsi_request_sense(cmd);
                break;

        case ALLOW_MEDIUM_REMOVAL:
                PDEBUG("SC_PREVENT_ALLOW_MEDIUM_REMOVAL\n" );
                nand_scsi_media_removal(dev);
                break;

        case TEST_UNIT_READY:
                PDEBUG( "SC_TEST_UNIT_READY\n" );
                nand_scsi_test_unit_ready(dev );
                break;

        case INQUIRY:
                PDEBUG( "SC_INQUIRY\n" );
                nand_scsi_inquiry( dev );
                break;

        case READ_6:
                PDEBUG( "R" );
                //if (down_interruptible(&fmi_sem))
                //        break;
                nand_scsi_read(cmd,dev);
                //up(&fmi_sem);
                break;

        case READ_10:
                PDEBUG( "R" );
                // if (down_interruptible(&fmi_sem))
                //       break;
                nand_scsi_read(cmd,dev);
                // up(&fmi_sem);
                break;

        case WRITE_6:
                PDEBUG( "W" );
                // if (down_interruptible(&fmi_sem))
                //       break;
                nand_scsi_write(cmd,dev);
                // up(&fmi_sem);
                break;

        case WRITE_10:
                PDEBUG( "W" );
                // if (down_interruptible(&fmi_sem))
                //       break;
                nand_scsi_write(cmd,dev);
                //up(&fmi_sem);
                break;

        case MODE_SENSE:
                PDEBUG( "SC_MODE_SENSE_6\n" );
                nand_scsi_mode_sense( dev );
                break;

        case MODE_SENSE_10:
                PDEBUG( "SC_MODE_SENSE_6\n" );
                nand_scsi_mode_sense( dev );
                break;

        case READ_CAPACITY:
                PDEBUG( "SC_READ_CAPACITY\n" );
                nand_scsi_read_capacity( dev );
                break;

        default:
                PDEBUG("UNKNOWN command : %02x\n", cmd->cmnd[0] );
                nand_make_sense_buffer(dev, ILLEGAL_REQUEST, 0x20, 0x00 );
                cmd->result = SD_CMD_RESULT( DID_OK, 0, 2);
                break;
        }
done:
        MSG2("Result : %08x\n", cmd->result);
        MSG2("Sense : [%02x%02x%02x]\n", cmd->sense_buffer[2],
             cmd->sense_buffer[12], cmd->sense_buffer[13]);

        if (cmd->result == 0) {
                MSG("Command Finished OK.\n");

                memset(cmd->sense_buffer, 0, SCSI_SENSE_BUFFERSIZE);
        }
quit:
        cmd->scsi_done(cmd);
//printk("scsi end\n");
        up(&fmi_sem);
        up(&sem_r);
        LEAVE();
}

static int nand_queue_cmd( struct scsi_cmnd  *cmd, void (* done )( struct scsi_cmnd  * ) )
{
        //int i;
        //struct request *rq;
        //struct gendisk *disk;
        //char * curDiskName;
        //char * diskLisk[] ={"sda","sdb","sdc","sdd","sde","sdf"};
        ENTER();

        if (done == NULL)
                return 0;

        if (cmd->device->lun > 0) {
                cmd->result = SD_CMD_RESULT(DID_NO_CONNECT, 0, 0);
                done(cmd);
                return 0;
        }

        //rq = cmd->request;
        //disk = rq->rq_disk;
        //curDiskName = disk->disk_name;

#if 0
        PDEBUG("**curDiskName**:%s\n",curDiskName);

        if ((nand_flagSetting.update == 1)&&((int)curDiskName!= 0xc)) { //update mapping
                MSG2("******setup map*********\n");
                nand_flagSetting.update = 0; //only when there is a new card insert or remove,it needs update.

                for (i=0;i<6;i++) {
                        if (!strcmp(curDiskName,diskLisk[i])) {
                                //printk("disk_0:%s\n",curDiskName);
                                nand_curMap[0]=i; //sd port 0 <==> sdx
                                nand_curMap2[i]=0;
                        }
                }
        }
#endif
        down_interruptible(&sem);
//printk("cmd start\n");
        nand_host.cmd = cmd;
        cmd->scsi_done = done;
        cmd->result = 0;

        nand_has_command = 1;
        wake_up_interruptible(&scsi_wq);
//printk("cmd end\n");
        up(&sem);
        LEAVE();
        return 0;
}

static int nand_queue_cmd1( struct scsi_cmnd  *cmd, void (* done )( struct scsi_cmnd  * ) )
{
        ENTER();

        if (done == NULL)
                return 0;

        if (cmd->device->lun > 0) {
                cmd->result = SD_CMD_RESULT(DID_NO_CONNECT, 0, 0);
                done(cmd);
                return 0;
        }

        down_interruptible(&sem);
        nand_host1.cmd = cmd;
        cmd->scsi_done = done;
        cmd->result = 0;

        nand_has_command1 = 1;
        wake_up_interruptible(&scsi_wq1);
        up(&sem);
        LEAVE();
        return 0;
}


#if defined(ONE_NAND) || (defined(ONE_NANDCARD) && defined(CONFIG_W55FA93_CS0_RB0_PIN))
static int nand_kernel_thread(void *param)
{
        ENTER();
        daemonize("nandthread");
        for (;;) {
                wait_event_interruptible(scsi_wq, (nand_has_command != 0)&&(nand1_i_o ==0)); //no card inserting
                if (nand_has_command == SD_EVENT_QUIT)
                        break;
                nand_has_command = 0;
                schedule();
                nand_scsi_process_cmd(&nand_host);
                schedule();
        }
        MSG("BUG : Nand Kernel Thread 0 Quit\n");
        return 0;
}
#endif

#if defined (TWO_NAND) || (defined(ONE_NANDCARD) && defined(CONFIG_W55FA93_CS1_RB1_PIN))
static int nand_kernel_thread1(void *param)
{
        ENTER();
        daemonize("nandthread1");
        for (;;) {
                wait_event_interruptible(scsi_wq1, (nand_has_command1 != 0)&&(nand_i_o ==0)); //no card inserting
                if (nand_has_command1 == SD_EVENT_QUIT)
                        break;
                nand_has_command1 = 0;
                schedule();
                nand_scsi_process_cmd(&nand_host1);
                schedule();
        }
        printk("BUG : Nand Kernel Thread 1 Quit\n");
        return 0;
}
#endif


static const char* nand_info( struct Scsi_Host * psh)
{
        ENTER();
        LEAVE();
        return "Nuvoton W55FA93 On Board NAND DRIVER!";
}

static const char* nand_info1( struct Scsi_Host * psh)
{
        ENTER();
        LEAVE();
        return "Nuvoton W55FA93 NAND Card DRIVER!";
}


static int nand_abort(struct scsi_cmnd * cmd)
{
        struct nand_hostdata *dev = &nand_host;
        //printk("nand_abort!!!!!!!!\n");
        ENTER();

        if (nand_flagSetting.bCardExist && dev->state != SD_STATE_NOP) {
//		sd_card_stop(dev);
                dev->sense = SS_COMMUNICATION_FAILURE;
                cmd->scsi_done(cmd);
                wake_up_interruptible(&sd_wq);
                return 0;
        }
        LEAVE();
        return 1;
}

static int nand_abort1(struct scsi_cmnd * cmd)
{
        struct nand_hostdata *dev = &nand_host1;
        //printk("nand_abort!!!!!!!!\n");
        ENTER();

        if (nand_flagSetting.bCardExist1 && dev->state != SD_STATE_NOP) {
//		sd_card_stop(dev);
                dev->sense = SS_COMMUNICATION_FAILURE;
                cmd->scsi_done(cmd);
                wake_up_interruptible(&sd_wq);
                return 0;
        }
        LEAVE();
        return 1;
}

static int nand_reset( struct scsi_cmnd *cmd)
{
        ENTER();

        if (down_interruptible(&fmi_sem))
                return SD_FAILED;

        nand_flagSetting.curCard = 0;
        nand_host_reset();

        up(&fmi_sem);
        LEAVE();
        return 0;
}

static int nand_reset1( struct scsi_cmnd *cmd)
{
        ENTER();

        if (down_interruptible(&fmi_sem))
                return SD_FAILED;

        nand_flagSetting.curCard = 1;
        nand_host_reset();

        up(&fmi_sem);
        LEAVE();
        return 0;
}

static int nand_bios_param(struct scsi_device *sdev,
                           struct block_device *bdev, sector_t capacity, int *info)
{
        ENTER();
        info[0] = 2;		// heads
        info[1] = 61;		// sectors
        //info[1] = 63;
        info[2] = capacity >> 7;
        LEAVE();
        return 0;
}


static int nand_ioctl(struct scsi_device *scsi_dev, int cmd, void  *arg)
{
    if (down_interruptible(&fmi_sem))
        return 0;

    nand_ioctl_0(cmd, (int)arg, 0, 0);

    up(&fmi_sem);
    return 0;
}


static int nand_ioctl1(struct scsi_device *scsi_dev, int cmd, void *arg)
{
        return 0;
}


static struct scsi_host_template driver_template = {
        .name 					= "NAND0",
        .info					= nand_info,
        .queuecommand			= nand_queue_cmd,
        .eh_abort_handler 		= nand_abort,
        .eh_host_reset_handler  = nand_reset,
        .bios_param				= nand_bios_param,
        .ioctl					= nand_ioctl,
        .can_queue              = 1,
        .this_id                = -1,
        .sg_tablesize           = 128,
        .cmd_per_lun            = 1,
        .unchecked_isa_dma		= 0,
        .use_clustering         = ENABLE_CLUSTERING,
        .module					= THIS_MODULE,
};

static struct scsi_host_template driver_template1 = {
        .name 					= "NAND1",
        .info					= nand_info1,
        .queuecommand			= nand_queue_cmd1,
        .eh_abort_handler 		= nand_abort1,
        .eh_host_reset_handler  = nand_reset1,
        .bios_param				= nand_bios_param,
        .ioctl                  = nand_ioctl1,
        .can_queue              = 1,
        .this_id                = -1,
        .sg_tablesize           = 128,
        .cmd_per_lun            = 1,
        .unchecked_isa_dma		= 0,
        .use_clustering         = ENABLE_CLUSTERING,
        .module					= THIS_MODULE,
};


MODULE_LICENSE( "GPL" );

static int sd_add = 0, sd_add1 = 0;

static void nand_device_release(struct device * dev)
{
        ENTER();
        LEAVE();
}

static struct device nand_device = {
        .init_name = "nand_bus",
        .release = nand_device_release,
};

static int nand_bus_match(struct device *dev, struct device_driver *dev_driver)
{
        ENTER();
        LEAVE();
        return 1;
}

static struct bus_type nand_bus = {
        .name = "nand_bus",
        .match = nand_bus_match,
};

static int nand_driver_probe(struct device *dev)
{
        struct Scsi_Host *shp;

        ENTER();
//        printk("nand_driver_probe\n");
//        printk("has command, CurCard = %d !!!\n", nand_flagSetting.curCard);

        if (nand_flagSetting.curCard == 0) { //card 0
                shp = scsi_host_alloc(&driver_template, 0);
                if ( shp == NULL) {
                        printk(KERN_ERR "%s: scsi_register failed\n", __FUNCTION__);
                        return -ENODEV;
                }

                if ( scsi_add_host(shp, &nand_host.dev) ) {
                        printk(KERN_ERR "%s: scsi_add_host 0 failed\n", __FUNCTION__);
                        scsi_host_put(shp);
                        return -ENODEV;
                }

                scsi_scan_host(shp);
                nand_host.shost = shp;
        }

        if (nand_flagSetting.curCard == 1) { //card 1
                shp = scsi_host_alloc(&driver_template1, 0);
                if ( shp == NULL) {
                        printk(KERN_ERR "%s: scsi_register failed\n", __FUNCTION__);
                        return -ENODEV;
                }

                if ( scsi_add_host(shp, &nand_host1.dev) ) {
                        printk(KERN_ERR "%s: scsi_add_host 1 failed\n", __FUNCTION__);
                        scsi_host_put(shp);
                        return -ENODEV;
                }

                scsi_scan_host(shp);
                nand_host1.shost = shp;
        }
        LEAVE();
        return 0;
}

static int nand_driver_remove(struct device *dev)
{
        ENTER();

        if (nand_flagSetting.sdRemove == 0) {
                scsi_remove_host(nand_host.shost);
                scsi_host_put(nand_host.shost);
        }

        if (nand_flagSetting.sdRemove == 1) {
                scsi_remove_host(nand_host1.shost);
                scsi_host_put(nand_host1.shost);
        }
        LEAVE();
        return 0;
}

static struct device_driver nand_driver = {
        .name 		= "nand_scsi",
        .bus			= &nand_bus,
        .probe          = nand_driver_probe,
        .remove         = nand_driver_remove,
};

static void nand_release_host(struct device *dev)
{
        ENTER();
        LEAVE();
}


static int sd_add_card(void)
{
        int err;
        ENTER();

        if ((sd_add == 0 )&&(nand_flagSetting.curCard == 0)) { //add card 0
                char name[] = "card0";
                //printk("add_card 0\n");
                sd_add = 1;
                memset(&nand_host.dev, 0, sizeof(struct device ));
#if 0 //chp
                if (nand_alloc0 == 0) {
                        nand_host.DMAvaddr = (int )dma_alloc_writecombine(NULL, (256 * DMA_BLOCK_SIZE), &nand_host.DMApaddr, GFP_KERNEL);
                        nand_alloc0 = 1;
                }
                printk("nand 0 dmavaddr 0x%x\n", nand_host.DMAvaddr);
#endif
                nand_host.dev.bus = &nand_bus;
                nand_host.dev.parent = &nand_device;
                nand_host.dev.release = nand_release_host;
                nand_host.dev.init_name = name;

                err = device_register(&nand_host.dev);
                if (err)
                        return err;

                LEAVE();
                return 0;
        }

        if ((sd_add1 == 0 )&&(nand_flagSetting.curCard == 1)) { //add card 1
                char name[] = "card1";
                printk("add_card 1\n");
                sd_add1 = 1;
                memset(&nand_host1.dev, 0, sizeof(struct device ));
                nand_host1.dev.bus = &nand_bus;
                nand_host1.dev.parent = &nand_device;
                nand_host1.dev.release = nand_release_host;
                nand_host1.dev.init_name = name;
                err = device_register(&nand_host1.dev);
                if (err){
                		printk("add_card FAIL !!!\n");
                        return err;
			}

                LEAVE();
                return 0;
        }

        if (nand_flagSetting.update == 1)
                nand_flagSetting.update = 0;
        return 0;
}

#if defined(TWO_NAND) || (defined(ONE_NANDCARD) && defined(CONFIG_W55FA93_CS1_RB1_PIN)) || (defined(ONE_NANDCARD) && defined(CONFIG_W55FA93_CS0_RB0_PIN))

static void sd_del_card(void)
{
        ENTER();

#if defined(TWO_NAND) || (defined(ONE_NANDCARD) && defined(CONFIG_W55FA93_CS1_RB1_PIN))
        if ((sd_add1 )&&(nand_flagSetting.sdRemove == 1)) {
                if(ptNDisk1 != 0) {
                	GNAND_CloseNAND(ptNDisk1);
                	ptNDisk1 = NULL;
                }
                fmiSMClose(1);
                device_unregister(&nand_host1.dev);
                sd_add1 = 0;
                nand_registered1 = 0;
        } //else
            //    printk("do nothing and return\n");
#endif

#if (defined(ONE_NANDCARD) && defined(CONFIG_W55FA93_CS0_RB0_PIN))
        if ((sd_add )&&(nand_flagSetting.sdRemove == 1)) {
                if(ptNDisk != 0) {
                	GNAND_CloseNAND(ptNDisk);
                	ptNDisk = NULL;
                }
                fmiSMClose(0);
                device_unregister(&nand_host.dev);
                sd_add = 0;
                nand_registered = 0;
        } //else
            //    printk("do nothing and return\n");
#endif
        LEAVE();
}
#endif

#if defined (TWO_NAND_MTD) || defined(ONE_NANDCARD_MTD)

	static int sd_add_card_mtd(void)
	{
	        int err;

	        ENTER();

	        if ((sd_add == 0 )&&(nand_flagSetting.curCard == 0)) { //add card 0
	                char name[] = "card0";
	                //printk("add_card 0\n");
	                sd_add = 1;
		#if 0
	                memset(&nand_host.dev, 0, sizeof(struct device ));
	                nand_host.dev.bus = &nand_bus;
	                nand_host.dev.parent = &nand_device;
	                nand_host.dev.release = nand_release_host;
	                nand_host.dev.init_name = name;

	                err = device_register(&nand_host.dev);
	                if (err)
	                        return err;
		#endif
	                LEAVE();
	                return 0;
	        }

	        if ((sd_add1 == 0 )&&(nand_flagSetting.curCard == 1)) { //add card 1
	                char name[] = "card1";
	                printk("add_card 1\n");
	                sd_add1 = 1;
					w55fa93_nand_init();
		#if 0
	                memset(&nand_host1.dev, 0, sizeof(struct device ));
	                nand_host1.dev.bus = &nand_bus;
	                nand_host1.dev.parent = &nand_device;
	                nand_host1.dev.release = nand_release_host;
	                nand_host1.dev.init_name = name;
	                err = device_register(&nand_host1.dev);
	                if (err){
	                		printk("add_card FAIL !!!\n");
	                        return err;
					}
		#endif
	                LEAVE();
	                return 0;
	        }

	        if (nand_flagSetting.update == 1)
	                nand_flagSetting.update = 0;
	        return 0;
	}

	static void sd_del_card_mtd(void)
	{
	        ENTER();

	#if defined(TWO_NAND_MTD) || defined(ONE_NANDCARD_MTD)
	        if ((sd_add1 )&&(nand_flagSetting.sdRemove == 1)) {
	                fmiSMClose(1);
	//                device_unregister(&nand_host1.dev);
	                sd_add1 = 0;
					w55fa93_nand_exit();
	                nand_registered1 = 0;
	        } //else
	            //    printk("do nothing and return\n");
	#endif
	        LEAVE();
	}
#endif

#if defined(ONE_NAND)
	static int nand_event_thread(void *unused)
	{
	        int event;

	        ENTER();
	        daemonize("nandeventd");
	        for (;;) {
	                wait_event_interruptible(nand_event_wq, nand_event != SD_EVENT_NONE);
	                event = nand_event;
	                nand_event = 0;

	             //   down_interruptible(&sem_w);

	                switch (event) {
	                case SD_EVENT_ADD:
	             		   	down_interruptible(&sem_w);
	               //         register_disk_name = 1;
	                        nand_i_o = 1;
	                        nand_flagSetting.bCardExist = 1;
	                        nand_flagSetting.bMediaChanged = 1;
	                        //nand_flagSetting.bWriteProtect = 0;
	                        nand_flagSetting.update = 1;
	                        nand_flagSetting.curCard = 0;
	                        sd_add_card();
	                        nand_i_o = 0;
							up(&sem_w);
	                        wake_up_interruptible(&scsi_wq1);
	                        break;

	                case SD_EVENT_QUIT:
	                        goto quit;

	                default:
	                        MSG("NO THIS EVENT !!\n");
	                        break;
	                }
	                // up(&sem_w);
	        }
	quit:
	        MSG("Quit Event Thread\n");
	        return 0;
	}

#elif (defined(ONE_NANDCARD) && defined(CONFIG_W55FA93_CS0_RB0_PIN))
	static int nand_event_thread(void *unused)
	{
	        int event;
	        static int card_exist = 0;
	        ENTER();
	        daemonize("nandeventd1");
	        for (;;) {
	                wait_event_interruptible(nand_event_wq, nand_event != SD_EVENT_NONE);
	                event = nand_event;
	                nand_event = 0;

	            //    down_interruptible(&sem_w);

	                switch (event) {
	                case SD_EVENT_ADD:
							down_interruptible(&sem_w);
	                        nand_i_o = 1;
	                        register_disk_name = 1;
	                        if (card_exist == 1) {
	                                //printk("orz!!!!\n");
	                                while (/*nand_host.state*//* nand_host1.state != SD_STATE_NOP*/ can_remove != 1) schedule();
	                                //printk("call del\n");
	                                nand_flagSetting.sdRemove = 1;
	                                sd_del_card();
	                        }
	                        nand_flagSetting.bCardExist = 1;
	                        nand_flagSetting.bMediaChanged = 1;
	                        //nand_flagSetting.bWriteProtect1 = 0;
	                        nand_flagSetting.update = 1;
	                        nand_flagSetting.curCard = 1;

	                        card_exist = 1;
	                        nand_detect = 1; //ya
	                        sd_add_card();
	                        nand_i_o = 0;
	                        up(&sem_w);
	                        wake_up_interruptible(&scsi_wq);
	                        break;

	                case SD_EVENT_REMOVE:
	                        //printk("remove card\n");
	                        down_interruptible(&sem_w);
	                        nand_i_o = 1;
	                        if (card_exist == 1) {
	                                while (/*nand_host.state*/ /*nand_host1.state != SD_STATE_NOP*/ can_remove != 1) schedule();
	                                nand_flagSetting.bCardExist = 0;
	                                nand_flagSetting.bMediaChanged = 0;
	                                nand_flagSetting.sdRemove = 1;
	                                sd_del_card();
	                        }
	                        card_exist = 0;
	                        nand_i_o = 0;
	                        up(&sem_w);
	                        wake_up_interruptible(&scsi_wq);
	                        break;

	                case SD_EVENT_QUIT:
	                        goto quit;

	                default:
	                        MSG("NO THIS EVENT !!\n");
	                        break;
	                }
	              //  up(&sem_w);
	        }
	quit:
	        MSG("Quit Event Thread\n");
	        return 0;
	}

#elif defined(ONE_NANDCARD_MTD)
	static int nand_event_thread_mtd(void *unused)
	{
	        int event;
	        static int card_exist = 0;

            ENTER();
	        daemonize("nandeventd1");
	        for (;;) {
	                wait_event_interruptible(nand_event_wq, nand_event != SD_EVENT_NONE);
	                event = nand_event;
	                nand_event = 0;

	                switch (event) {
	                case SD_EVENT_ADD:
							down_interruptible(&sem_w);
	                        nand_i_o = 1;
	                        register_disk_name = 1;
	                        if (card_exist == 1) {
	                                //printk("orz!!!!\n");
	                                while (/*nand_host.state*//* nand_host1.state != SD_STATE_NOP*/ can_remove != 1) schedule();
	                                //printk("call del\n");
	                                nand_flagSetting.sdRemove = 1;
	                                sd_del_card_mtd();
	                        }
	                        nand_flagSetting.bCardExist = 1;
	                        nand_flagSetting.bMediaChanged = 1;
	                        //nand_flagSetting.bWriteProtect1 = 0;
	                        nand_flagSetting.update = 1;
	                        nand_flagSetting.curCard = 1;

	                        card_exist = 1;
	                        nand_detect = 1; //ya
	                        sd_add_card_mtd();
	                        nand_i_o = 0;
	                        up(&sem_w);
	                        wake_up_interruptible(&scsi_wq);
	                        break;

	                case SD_EVENT_REMOVE:
	                        //printk("remove card\n");
	                        down_interruptible(&sem_w);
	                        nand_i_o = 1;
	                        if (card_exist == 1) {
	                                while (/*nand_host.state*/ /*nand_host1.state != SD_STATE_NOP*/ can_remove != 1) schedule();
	                                nand_flagSetting.bCardExist = 0;
	                                nand_flagSetting.bMediaChanged = 0;
	                                nand_flagSetting.sdRemove = 1;
	                                sd_del_card_mtd();
	                        }
	                        card_exist = 0;
	                        nand_i_o = 0;
	                        up(&sem_w);
	                        wake_up_interruptible(&scsi_wq);
	                        break;

	                case SD_EVENT_QUIT:
	                        goto quit;

	                default:
	                        MSG("NO THIS EVENT !!\n");
	                        break;
	                }
	              //  up(&sem_w);
	        }
	quit:
	        MSG("Quit Event Thread\n");
	        return 0;
	}

#endif

#if defined (TWO_NAND) || (defined(ONE_NANDCARD) && defined(CONFIG_W55FA93_CS1_RB1_PIN))
	static int nand_event_thread1(void *unused)
	{
	        int event;
	        static int card_exist = 0;

            ENTER();
	        daemonize("nandeventd1");
	        for (;;) {

	                wait_event_interruptible(nand_event_wq1, nand_event1 != SD_EVENT_NONE);
	                event = nand_event1;
	                nand_event1 = 0;

	            //    down_interruptible(&sem_w);

	                switch (event) {
	                case SD_EVENT_ADD:
							down_interruptible(&sem_w);
	                        nand1_i_o = 1;
	                        register_disk_name = 1;
	                        nand1_i_o = 1;
	                        if (card_exist == 1) {
	                                //printk("orz!!!!\n");
                                    while (/*nand_host.state*//* nand_host1.state != SD_STATE_NOP*/ can_remove != 1)
                                    {
                                        // 2012/9/28 by CJChen, force to sd_del_card() if Nand card be removed during initial it.
                                        if (nand_card_status() == 1)    // nand card removed now
                                        {
                                            can_remove = 1;     // exit while loop and go down.
                                        }
                                        schedule();
                                    }
	                                //printk("call del\n");
	                                nand_flagSetting.sdRemove = 1;
	                                sd_del_card();
	                        }
	                        nand_flagSetting.bCardExist1 = 1;
	                        nand_flagSetting.bMediaChanged1 = 1;
	                        //nand_flagSetting.bWriteProtect1 = 0;
	                        nand_flagSetting.update = 1;
	                        nand_flagSetting.curCard = 1;

	                        card_exist = 1;
	                        nand_detect = 1; //ya
	                        sd_add_card();
	                        nand1_i_o = 0;
	                        up(&sem_w);
	                        wake_up_interruptible(&scsi_wq);
	                        break;

	                case SD_EVENT_REMOVE:
	                        //printk("remove card\n");
	                        down_interruptible(&sem_w);
	                        nand1_i_o = 1;
	                        if (card_exist == 1) {
                                    while (/*nand_host.state*/ /*nand_host1.state != SD_STATE_NOP*/ can_remove != 1)
                                    {
                                        // 2012/9/28 by CJChen, force to sd_del_card() if Nand card be removed during initial it.
                                        if (nand_card_status() == 1)    // nand card removed now
                                        {
                                            can_remove = 1;     // exit while loop and go down.
                                        }
                                        schedule();
                                    }
	                                nand_flagSetting.bCardExist1 = 0;
	                                nand_flagSetting.bMediaChanged1 = 0;
	                                nand_flagSetting.sdRemove = 1;
	                                sd_del_card();
	                        }
	                        card_exist = 0;
	                        nand1_i_o = 0;
	                        up(&sem_w);
	                        wake_up_interruptible(&scsi_wq);
	                        break;

	                case SD_EVENT_QUIT:
	                        goto quit;

	                default:
	                        MSG("NO THIS EVENT !!\n");
	                        break;
	                }
	              //  up(&sem_w);
	        }
	quit:
	        MSG("Quit Event Thread\n");
	        return 0;
	}
#endif

#if defined (TWO_NAND_MTD) || defined(ONE_NANDCARD_MTD)
	static int nand_event_thread1_mtd(void *unused)
	{
	        int event;
	        static int card_exist = 0;

            ENTER();
	        daemonize("nandeventd1");
	        for (;;) {
	                wait_event_interruptible(nand_event_wq1, nand_event1 != SD_EVENT_NONE);
	                event = nand_event1;
	                nand_event1 = 0;

	            //    down_interruptible(&sem_w);

	                switch (event) {
	                case SD_EVENT_ADD:
							down_interruptible(&sem_w);
	                        nand1_i_o = 1;
	                        register_disk_name = 1;
	                        nand1_i_o = 1;
	                        if (card_exist == 1) {
	                                //printk("orz!!!!\n");
	                                while (/*nand_host.state*//* nand_host1.state != SD_STATE_NOP*/ can_remove != 1) schedule();
	                                //printk("call del\n");
	                                nand_flagSetting.sdRemove = 1;
	                                sd_del_card_mtd();
	                        }
	                        nand_flagSetting.bCardExist1 = 1;
	                        nand_flagSetting.bMediaChanged1 = 1;
	                        //nand_flagSetting.bWriteProtect1 = 0;
	                        nand_flagSetting.update = 1;
	                        nand_flagSetting.curCard = 1;

	                        card_exist = 1;
	                        nand_detect = 1; //ya
	                        sd_add_card_mtd();
	                        nand1_i_o = 0;
	                        up(&sem_w);
	                        wake_up_interruptible(&scsi_wq);
	                        break;

	                case SD_EVENT_REMOVE:
	                        //printk("remove card\n");
	                        down_interruptible(&sem_w);
	                        nand1_i_o = 1;
	                        if (card_exist == 1) {
	                                while (/*nand_host.state*/ /*nand_host1.state != SD_STATE_NOP*/ can_remove != 1) schedule();
	                                nand_flagSetting.bCardExist1 = 0;
	                                nand_flagSetting.bMediaChanged1 = 0;
	                                nand_flagSetting.sdRemove = 1;
	                                sd_del_card_mtd();
	                        }
	                        card_exist = 0;
	                        nand1_i_o = 0;
	                        up(&sem_w);
	                        wake_up_interruptible(&scsi_wq);
	                        break;

	                case SD_EVENT_QUIT:
	                        goto quit;

	                default:
	                        MSG("NO THIS EVENT !!\n");
	                        break;
	                }
	              //  up(&sem_w);
	        }
	quit:
	        MSG("Quit Event Thread\n");
	        return 0;
	}
#endif


extern u8 *_gnand_pDMABuffer;
extern u8 *_gnand_gDMABuffer;

static int __init nand_init(void)
{
    struct clk *fmi_clk, *nand_clk;

    ENTER();
    printk("NAND: nand_init!!!!!\n");
#if defined(CONFIG_W55FA93_SYSMGR) || defined (CONFIG_W55FA93_SYSMGR_MODULE)
	PDEBUG("system_report enabled!\n");
#endif

    // enable FMI and NAND clock
    fmi_clk = clk_get(NULL, "SIC");
    if (IS_ERR(fmi_clk)) {
        printk("Get clock fail. No SIC clock for NAND !\n");
        return -ENODEV;
    }

    nand_clk = clk_get(NULL, "NAND");
    if (IS_ERR(nand_clk)) {
        printk("Get clock fail. No NAND clock for NAND !\n");
        return -ENODEV;
    }

    clk_enable(fmi_clk);
    clk_enable(nand_clk);

		outpw(REG_GPDFUN, inpw(REG_GPDFUN) | 0x00030C00);		// enable NAND NWR/NRD pins
		outpw(REG_GPEFUN, inpw(REG_GPEFUN) | 0x00F00000);		// enable NAND ALE/CLE pins

#if defined(ONE_NAND)
		outpw(REG_GPEFUN, inpw(REG_GPEFUN) | 0x00F30000);		// enable NAND ALE/CLE/CS0 pins
		outpw(REG_GPDFUN, inpw(REG_GPDFUN) | 0x0000C000);		// enable NAND RB0 pins
#endif

#if defined (TWO_NAND)
		outpw(REG_GPEFUN, inpw(REG_GPEFUN) | 0x00FF0000);		// enable NAND ALE/CLE/CS1 pins
		outpw(REG_GPDFUN, inpw(REG_GPDFUN) | 0x00003000);		// enable NAND RB1 pins
#endif

#if defined (ONE_NANDCARD)
	#if defined (CONFIG_W55FA93_CS0_RB0_PIN)
		outpw(REG_GPEFUN, inpw(REG_GPEFUN) | 0x00F30000);		// enable NAND ALE/CLE/CS0 pins
		outpw(REG_GPDFUN, inpw(REG_GPDFUN) | 0x0000C000);		// enable NAND RB0 pins
	#else
		outpw(REG_GPEFUN, inpw(REG_GPEFUN) | 0x00FC0000);		// enable NAND ALE/CLE/CS1 pins
		outpw(REG_GPDFUN, inpw(REG_GPDFUN) | 0x00003000);		// enable NAND RB1 pins
	#endif
#endif

		outpw(REG_SMCSR, inpw(REG_SMCSR) | SMCR_CS0 | SMCR_CS1); // set both CS0 & CS1 to high state

        device_register(&nand_device);
        bus_register(&nand_bus);
        driver_register(&nand_driver);

        driver_template.proc_name = "nand_scsi_0";
        driver_template1.proc_name = "nand_scsi_1";

        if (request_irq(IRQ_SIC, fmi_interrupt, IRQF_SHARED, "W55FA93_NAND", (void*)1)) {
                printk("NAND: Request IRQ error\n");
                return -1;
        }
        nand_alloc0 = 0;

		// for nand card
#if defined(NANDCARD_2_DETECT) || defined(ONE_NANDCARD) || defined(NANDCARD_2_DETECT_MTD) || defined(ONE_NANDCARD_MTD)
	#if defined (CONFIG_NANDCARD_GPB6)
        // PORTB6
        outl(inl(REG_GPBFUN) & ~MF_GPB6, REG_GPBFUN);					// NAND card detect
        outl(inl(REG_GPIOB_OMD) & ~(0x0040), REG_GPIOB_OMD); 			// port B6 input
        outl(inl(REG_GPIOB_PUEN) | (0x0040), REG_GPIOB_PUEN); 			// port B6 pull-up

        outl(inl(REG_IRQTGSRC0) & 0x00400000, REG_IRQTGSRC0);
        outl((inl(REG_IRQSRCGPB) & ~(0x3000)) | 0x1000, REG_IRQSRCGPB); // port B6 as nIRQ1 source
        outl(inl(REG_IRQENGPB) | 0x00400040, REG_IRQENGPB); 			// falling/rising edge trigger

	#elif (CONFIG_NANDCARD_GPD15)
        // PORTD15
        outl(inl(REG_GPDFUN) & ~MF_GPD15, REG_GPDFUN);					// NAND card detect
        outl(inl(REG_GPIOD_OMD) & ~(0x8000), REG_GPIOD_OMD); 			// port D15 input
        outl(inl(REG_GPIOD_PUEN) | (0x8000), REG_GPIOD_PUEN); 			// port D15 pull-up

        outl(inl(REG_IRQTGSRC1) & 0x80000000, REG_IRQTGSRC1);
        outl((inl(REG_IRQSRCGPD) & ~(0xC0000000)) | 0x40000000, REG_IRQSRCGPD); // port D15 as nIRQ1 source
        outl(inl(REG_IRQENGPD) | 0x80008000, REG_IRQENGPD); 		// falling/rising edge trigger

	#endif

        outl((inl(REG_AIC_SCR1) & 0x00FFFFFF) | 0xc7000000, REG_AIC_SCR1);

        outl(8,  REG_AIC_SCCR); // force clear previous interrupt, if any.
        if (request_irq(NAND_CD_IRQ_NUM, nand_card_detect_irq, IRQF_DISABLED, "FA93_NAND_CD", NULL) != 0) {
                printk("register the nand_card_detect_irq failed!\n");
                return -1;
        }
        nand_alloc1 = 0;
#endif	// NANDCARD_2_DETECT

        if (down_interruptible(&dmac_sem))
                return(GNERR_IO_ERR);

        while (sd_inl(REG_DMACCSR)&DMACCSR_FMI_BUSY); //Wait IP finished... for safe
        // DMAC Initial
        outpw(REG_DMACCSR, 0x00000003);
        //Enable DMAC
        outpw(REG_DMACCSR, 0x00000001);
        // Enable target abort interrupt generation during DMA transfer.
        outpw(REG_DMACIER, 0x00000001);

        up(&dmac_sem);

        Enable_IRQ(IRQ_SIC);

#if defined(ONE_NAND) || (defined(ONE_NANDCARD) && defined(CONFIG_W55FA93_CS0_RB0_PIN))
        kernel_thread(nand_event_thread, NULL, 0);
        kernel_thread(nand_kernel_thread, NULL, 0);
#endif

#if defined (TWO_NAND) || (defined(ONE_NANDCARD) && defined(CONFIG_W55FA93_CS1_RB1_PIN))
        kernel_thread(nand_event_thread1, NULL, 0);
        kernel_thread(nand_kernel_thread1, NULL, 0);
#endif

#if defined (TWO_NAND_MTD) || defined(ONE_NANDCARD_MTD)
        kernel_thread(nand_event_thread1_mtd, NULL, 0);
//        kernel_thread(nand_kernel_thread1, NULL, 0);
#endif

        _fmi_pNANDBuffer = (int )dma_alloc_writecombine(NULL, 512*8, &_fmi_ucNANDBuffer, GFP_KERNEL);
		_fmi_gptr1 = _fmi_pNANDBuffer1;

        if (_gnand_pDMABuffer == 0)
                _gnand_pDMABuffer = (u8 *)kmalloc(512*8,GFP_KERNEL);

        nand_host.state = 0;
        nand_host.sense = 0;
        nand_host.cardType = 0;

        nand_host1.state = 0;
        nand_host1.sense = 0;
        nand_host1.cardType = 0;

        nand_host_reset();

#if defined(ONE_NAND)
        nand_event = SD_EVENT_ADD;
        wake_up_interruptible(&nand_event_wq);
#endif

#if defined(TWO_NAND)
	#ifdef NANDCARD_2_DETECT
			PDEBUG("Setup nandcard detect timer\n");
	        init_timer(&nand_timer);
	        nand_host1.state = 0;
	        nand_host1.sense = 0;
	        nand_flagSetting.bCardExist1=0;

		#if defined (CONFIG_NANDCARD_GPB6)
	        if (!(inl(REG_GPIOB_PIN) & 0x40)) {
	                printk("~~~~~ begin external NAND card inserted Timer ~~~~~~\n");
	                del_timer(&nand_timer);
	                nand_timer.data = 0UL;
	                nand_timer.expires = jiffies +  NAND_SHORT_DELAY;
	                nand_timer.function = nand_Short_TimeOut;
	                add_timer(&nand_timer);
	        }
		#elif (CONFIG_NANDCARD_GPD15)
	        if (!(inl(REG_GPIOD_PIN) & 0x8000)) {
	                printk("~~~~~ begin external NAND card inserted Timer ~~~~~~\n");
	                del_timer(&nand_timer);
	                nand_timer.data = 0UL;
	                nand_timer.expires = jiffies +  NAND_SHORT_DELAY;
	                nand_timer.function = nand_Short_TimeOut;
	                add_timer(&nand_timer);
	        }
		#endif
	        else
	        {
	                printk(" ~~~~~~ external NAND card not inserted !!! ~~~~~~~~~~~\n");
			}
	#else
        nand_event1 = SD_EVENT_ADD;				//mhkuo@20101214
        wake_up_interruptible(&nand_event_wq1);	//mhkuo@20101214
	#endif
#endif

#if defined(TWO_NAND_MTD)
	#ifdef NANDCARD_2_DETECT_MTD
			PDEBUG("Setup nandcard detect timer\n");
	        init_timer(&nand_timer);
	        nand_host1.state = 0;
	        nand_host1.sense = 0;
	        nand_flagSetting.bCardExist1=0;

		#if defined (CONFIG_NANDCARD_GPB6)
	        if (!(inl(REG_GPIOB_PIN) & 0x40)) {
	                printk("~~~~~ begin external NAND card inserted Timer ~~~~~~\n");
	                del_timer(&nand_timer);
	                nand_timer.data = 0UL;
	                nand_timer.expires = jiffies +  NAND_SHORT_DELAY;
	                nand_timer.function = nand_Short_TimeOut;
	                add_timer(&nand_timer);
	        }
		#elif (CONFIG_NANDCARD_GPD15)
	        if (!(inl(REG_GPIOD_PIN) & 0x8000)) {
	                printk("~~~~~ begin external NAND card inserted Timer ~~~~~~\n");
	                del_timer(&nand_timer);
	                nand_timer.data = 0UL;
	                nand_timer.expires = jiffies +  NAND_SHORT_DELAY;
	                nand_timer.function = nand_Short_TimeOut;
	                add_timer(&nand_timer);
	        }
		#endif
	        else
	        {
	                printk(" ~~~~~~ external NAND card not inserted !!! ~~~~~~~~~~~\n");
			}
	#else
        nand_event1 = SD_EVENT_ADD;
        wake_up_interruptible(&nand_event_wq1);
	#endif
#endif

#if defined(ONE_NANDCARD)

			PDEBUG("Setup nandcard detect timer\n");
	        init_timer(&nand_timer);
	        nand_host1.state = 0;
	        nand_host1.sense = 0;
	        nand_flagSetting.bCardExist1=0;

		#if defined (CONFIG_NANDCARD_GPB6)
	        if (!(inl(REG_GPIOB_PIN) & 0x40)) {
	                printk("~~~~~ begin external NAND card inserted Timer ~~~~~~\n");
	                del_timer(&nand_timer);
	                nand_timer.data = 0UL;
	                nand_timer.expires = jiffies +  NAND_SHORT_DELAY;
	                nand_timer.function = nand_Short_TimeOut;
	                add_timer(&nand_timer);
	        }
		#elif (CONFIG_NANDCARD_GPD15)
	        if (!(inl(REG_GPIOD_PIN) & 0x8000)) {
	                printk("~~~~~ begin external NAND card inserted Timer ~~~~~~\n");
	                del_timer(&nand_timer);
	                nand_timer.data = 0UL;
	                nand_timer.expires = jiffies +  NAND_SHORT_DELAY;
	                nand_timer.function = nand_Short_TimeOut;
	                add_timer(&nand_timer);
	        }
		#endif
	        else
	        {
	                printk(" ~~~~~~ external NAND card not inserted !!! ~~~~~~~~~~~\n");
			}
#endif

#if defined(ONE_NANDCARD_MTD)

			PDEBUG("Setup nandcard detect timer\n");
	        init_timer(&nand_timer);
	        nand_host1.state = 0;
	        nand_host1.sense = 0;
	        nand_flagSetting.bCardExist1=0;

		#if defined (CONFIG_NANDCARD_GPB6)
	        if (!(inl(REG_GPIOB_PIN) & 0x40)) {
	                printk("~~~~~ begin external NAND card inserted Timer ~~~~~~\n");
	                del_timer(&nand_timer);
	                nand_timer.data = 0UL;
	                nand_timer.expires = jiffies +  NAND_SHORT_DELAY;
	                nand_timer.function = nand_Short_TimeOut;
	                add_timer(&nand_timer);
	        }
		#elif (CONFIG_NANDCARD_GPD15)
	        if (!(inl(REG_GPIOD_PIN) & 0x8000)) {
	                printk("~~~~~ begin external NAND card inserted Timer ~~~~~~\n");
	                del_timer(&nand_timer);
	                nand_timer.data = 0UL;
	                nand_timer.expires = jiffies +  NAND_SHORT_DELAY;
	                nand_timer.function = nand_Short_TimeOut;
	                add_timer(&nand_timer);
	        }
		#endif
	        else
	        {
	                printk(" ~~~~~~ external NAND card not inserted !!! ~~~~~~~~~~~\n");
			}
#endif

		// for nand card
#if defined (TWO_NAND) || defined (ONE_NANDCARD) || defined (TWO_NAND_MTD) || defined (ONE_NANDCARD_MTD)
        Enable_IRQ(NAND_CD_IRQ_NUM);
#endif

        printk("W55FA93 NAND driver (%s) has been initialized successfully!\n", DATE_CODE);
        LEAVE();
        return 0;
}

static void __exit nand_exit(void)
{
        ENTER();

        dma_free_writecombine(NULL, (512*8), &_fmi_pNANDBuffer, _fmi_ucNANDBuffer);
	//  dma_free_writecombine(NULL, (32 * DMA_BLOCK_SIZE), _fmi_pNANDBuffer1, _fmi_ucNANDBuffer1);

#if defined(NANDCARD_2_DETECT) || defined (ONE_NANDCARD)
        free_irq(NAND_CD_IRQ_NUM, NULL);
#endif

#if 0 //chp
        if (nand_alloc0 == 1) {
                dma_free_writecombine(NULL, (256 * DMA_BLOCK_SIZE), &nand_host.DMAvaddr, nand_host.DMApaddr);
                nand_alloc0 = 0;
        }
#endif
        free_irq(IRQ_SIC, NULL);

        driver_unregister(&nand_driver);
        bus_unregister(&nand_bus);
        device_unregister(&nand_device);

        LEAVE();
}


module_init(nand_init);
module_exit(nand_exit);
