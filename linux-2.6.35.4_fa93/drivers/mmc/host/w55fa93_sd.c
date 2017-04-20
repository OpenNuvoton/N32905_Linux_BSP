/*
 *  linux/drivers/mmc/host/w55fa93_sd.c - Nuvoton W55FA93 SD Driver
 *
 *  Copyright (C) 2005 Cougar Creek Computing Devices Ltd, All Rights Reserved
 *
 *  Copyright (C) 2006 Malcolm Noyes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/atmel_pdc.h>
#include <linux/gfp.h>
#include <linux/mmc/host.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/map.h>
#include <mach/regs-clock.h>

#include <mach/w55fa93_reg.h>

// define DATE CODE and show it when running to make maintaining easy.
#define DATE_CODE   "20161207"

//--- Define compile flags depend on Kconfig
#ifdef CONFIG_FA93_SD_SD0_SD1
    #define NVT_SD_SD1
#elif defined (CONFIG_FA93_SD_SD0_SD2)
    #define NVT_SD_SD2
#endif

#define nvt_sd_debug printk
//#define ENTER()     nvt_sd_debug("[%-20s #%d] Enter ...\n", __FUNCTION__, __LINE__)
//#define LEAVE()     nvt_sd_debug("[%-20s #%d] Leave ...\n", __FUNCTION__, __LINE__)
#define ENTER()
#define LEAVE()

#define DRIVER_NAME "w55fa93-fmi"   // driver name MUST exact same to device name that defined at dev.c

#define FL_SENT_COMMAND (1 << 0)
#define FL_SENT_STOP    (1 << 1)

#define nvt_sd_read(reg)         __raw_readl(reg)
#define nvt_sd_write(reg, val)   __raw_writel((val), (reg))

#define MCI_BLKSIZE         512
#define MCI_MAXBLKSIZE      4095
#define MCI_BLKATONCE       255     // SIC support max 255 blocks in one SD CMD
#define MCI_BUFSIZE         (MCI_BLKSIZE * MCI_BLKATONCE)

/* Driver thread command */
#define SD_EVENT_NONE       0x00000000
#define SD_EVENT_CMD_OUT    0x00000001
#define SD_EVENT_RSP_IN     0x00000010
#define SD_EVENT_RSP2_IN    0x00000100
#define SD_EVENT_CLK_KEEP0  0x00001000
#define SD_EVENT_CLK_KEEP1  0x00010000

// for SD port 0
static volatile int sd_event=0, sd_state=0, sd_state_xfer=0, sd_ri_timeout=0, sd_send_cmd=0;
static DECLARE_WAIT_QUEUE_HEAD(sd_event_wq);
static DECLARE_WAIT_QUEUE_HEAD(sd_wq);
static DECLARE_WAIT_QUEUE_HEAD(sd_wq_xfer);

#ifdef NVT_SD_SD1
    // for SD port 1
    static volatile int sd1_event=0, sd1_state=0, sd1_state_xfer=0, sd1_ri_timeout=0, sd1_send_cmd=0;
    static DECLARE_WAIT_QUEUE_HEAD(sd1_event_wq);
    static DECLARE_WAIT_QUEUE_HEAD(sd1_wq);
    static DECLARE_WAIT_QUEUE_HEAD(sd1_wq_xfer);
#endif

#ifdef NVT_SD_SD2
    // for SD port 2
    static volatile int sd2_event=0, sd2_state=0, sd2_state_xfer=0, sd2_ri_timeout=0, sd2_send_cmd=0;
    static DECLARE_WAIT_QUEUE_HEAD(sd2_event_wq);
    static DECLARE_WAIT_QUEUE_HEAD(sd2_wq);
    static DECLARE_WAIT_QUEUE_HEAD(sd2_wq_xfer);
#endif

extern struct semaphore fmi_sem;
extern struct semaphore dmac_sem;

void nvt_sd_set_clock(u32 sd_clock_khz);

static int sd_init_completed = 0;   // indicate nvt_sd_init() not yet completed

/*
 * Low level type for this driver
 */
struct nvt_sd_host {
    struct mmc_host *mmc;
    struct mmc_command *cmd;
    struct mmc_request *request;

    void __iomem *sd_base;
    int irq;        // IRQ number of system for SD

    int present;    // 0 for card inserted; 1 for card removed

    struct clk *fmi_clk, *sd_clk, *dmac_clk;

    /*
     * Flag indicating when the command has been sent. This is used to
     * work out whether or not to send the stop
     */
    unsigned int flags;
    /* flag for current port */
    u32 bus_mode;   // MMC_BUS_WIDTH_1 / MMC_BUS_WIDTH_4
    u32 port;       // SD port 0 / 1 / 2

    /* DMA buffer used for transmitting */
    unsigned int* buffer;
    dma_addr_t physical_address;
    unsigned int total_length;

    /* Latest in the scatterlist that has been enabled for transfer, but not freed */
    int in_use_index;

    /* Latest in the scatterlist that has been enabled for transfer */
    int transfer_index;

    /* Timer for timeouts */
    struct timer_list timer;
};

struct nvt_sd_host *sd_host;
#ifdef NVT_SD_SD1
    struct nvt_sd_host *sd1_host;
#endif
#ifdef NVT_SD_SD2
    struct nvt_sd_host *sd2_host;
#endif

#if 0
static void nvt_sd_show_reg(char *title)
{
    nvt_sd_debug("    ---- %s ---------------------\n", title);
    nvt_sd_debug("    REG_SDCR   =0x%08X\n", nvt_sd_read(REG_SDCR));
    nvt_sd_debug("    REG_SDARG  =0x%08X\n", nvt_sd_read(REG_SDARG));
    nvt_sd_debug("    REG_SDIER  =0x%08X\n", nvt_sd_read(REG_SDIER));
    nvt_sd_debug("    REG_SDISR  =0x%08X\n", nvt_sd_read(REG_SDISR));
    nvt_sd_debug("    REG_SDBLEN =0x%08X\n", nvt_sd_read(REG_SDBLEN));
    nvt_sd_debug("    REG_SDTMOUT=0x%08X\n", nvt_sd_read(REG_SDTMOUT));
    nvt_sd_debug("    REG_SDRSP0 =0x%08X\n", nvt_sd_read(REG_SDRSP0));
    nvt_sd_debug("    REG_SDRSP1 =0x%08X\n", nvt_sd_read(REG_SDRSP1));
}
#endif


/*
 * Detect the SD card status. Present or Absent?
 */
static int nvt_sd_card_detect(struct mmc_host *mmc)
{
    struct nvt_sd_host *host = mmc_priv(mmc);
    int ret;

    if(nvt_sd_read(REG_FMICR) != FMI_SD_EN)
        nvt_sd_write(REG_FMICR, FMI_SD_EN);

    //--- always return card removed if nvt_sd_init() not yet complete
    if (sd_init_completed == 0)
    {
        host->present = 1;  // card removed
        return 0;
    }

    if (host->port == 0)
        host->present = nvt_sd_read(REG_SDISR) & SDISR_CD_Card;

#ifdef NVT_SD_SD1
    else if (host->port == 1)
    {
        host->present = 0;  // default is card inserted.
  #ifdef CONFIG_SD1_CD_GPA3
        if (nvt_sd_read(REG_GPIOA_PIN) & BIT3)
            host->present = 1;  // GPA3 high means card removed.
  #elif defined (CONFIG_SD1_CD_GPB6)
        if (nvt_sd_read(REG_GPIOB_PIN) & BIT6)
            host->present = 1;  // GPB6 high means card removed.
  #elif defined (CONFIG_SD1_CD_GPE11)
        if (nvt_sd_read(REG_GPIOE_PIN) & BIT11)
            host->present = 1;  // GPE11 high means card removed.
  #endif
    }
#endif

#ifdef NVT_SD_SD2
    else if (host->port == 2)
    {
        host->present = 0;  // default is card inserted.
  #ifdef CONFIG_SD2_CD_GPA3
        if (nvt_sd_read(REG_GPIOA_PIN) & BIT3)
            host->present = 1;  // GPA3 high means card removed.
  #elif defined (CONFIG_SD2_CD_GPB6)
        if (nvt_sd_read(REG_GPIOB_PIN) & BIT6)
            host->present = 1;  // GPB6 high means card removed.
  #elif defined (CONFIG_SD2_CD_GPE11)
        if (nvt_sd_read(REG_GPIOE_PIN) & BIT11)
            host->present = 1;  // GPE11 high means card removed.
  #elif defined (CONFIG_SD2_CD_GPE11_SW)
        if (!(nvt_sd_read(REG_GPIOE_PIN) & BIT11))
            host->present = 1;  // GPE11 low means card removed.
  #endif
    }
#endif

    // Return values for the get_cd() callback should be:
    //      0 for a absent card
    //      1 for a present card
    ret = host->present ? 0 : 1;

    // nvt_sd_debug("--> nvt_sd_card_detect(): SD port %d %s.\n", host->port, ret ? "inserted" : "removed");
    return ret;
}


//-------------------------------------------------
// Waiting SD card become to READY status.
//      Check DATA0 pin. High means READY; LOW means BUSY for write operation.
//-------------------------------------------------
static void nvt_sd_wait_card_ready(struct mmc_host *mmc)
{
    while (!(nvt_sd_read(REG_SDISR) & SDISR_SD_DATA0))
    {
        //--- SD card is busy. Keep waiting or exit if SD card removed.
        if (nvt_sd_card_detect(mmc) == 0)
            break;  // don't wait if SD card removed.
        nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) | SDCR_8CLK_OE);   // generate 8 clocks
        while (nvt_sd_read(REG_SDCR) & SDCR_8CLK_OE)    // wait for 8 clocks completed.
        {
            schedule();
        }
    }
}


/*
 * Reset the controller and restore most of the state
 */
static void nvt_sd_reset_host(struct nvt_sd_host *host)
{
    unsigned long flags;

    local_irq_save(flags);
    nvt_sd_write(REG_DMACCSR, nvt_sd_read(REG_DMACCSR) | DMAC_EN | DMAC_SWRST); // enable DMAC for FMI
    nvt_sd_write(REG_FMICR, FMI_SD_EN);     // Enable SD functionality of FMI
    local_irq_restore(flags);
}

static void nvt_sd_timeout_timer(unsigned long data)
{
    struct nvt_sd_host *host;

    host = (struct nvt_sd_host *)data;

    if (host->request) {
        dev_err(host->mmc->parent, "Timeout waiting end of packet\n");

        if (host->cmd && host->cmd->data) {
            host->cmd->data->error = -ETIMEDOUT;
        } else {
            if (host->cmd)
                host->cmd->error = -ETIMEDOUT;
            else
                host->request->cmd->error = -ETIMEDOUT;
        }

        nvt_sd_reset_host(host);
        mmc_request_done(host->mmc, host->request);
    }
}

/*
 * Copy from sg to a dma block - used for transfers
 */
static inline void nvt_sd_sg_to_dma(struct nvt_sd_host *host, struct mmc_data *data)
{
    unsigned int len, i, size;
    unsigned *dmabuf = host->buffer;

    size = data->blksz * data->blocks;
    len = data->sg_len;

    /*
     * Just loop through all entries. Size might not
     * be the entire list though so make sure that
     * we do not transfer too much.
     */
    for (i = 0; i < len; i++) {
        struct scatterlist *sg;
        int amount;
        unsigned int *sgbuffer;

        sg = &data->sg[i];

        sgbuffer = kmap_atomic(sg_page(sg), KM_BIO_SRC_IRQ) + sg->offset;
        amount = min(size, sg->length);
        size -= amount;

        {
            char *tmpv = (char *)dmabuf;
            memcpy(tmpv, sgbuffer, amount);
            tmpv += amount;
            dmabuf = (unsigned *)tmpv;
        }

        kunmap_atomic(sgbuffer, KM_BIO_SRC_IRQ);
        data->bytes_xfered += amount;

        if (size == 0)
            break;
    }

    /*
     * Check that we didn't get a request to transfer
     * more data than can fit into the SG list.
     */
    BUG_ON(size != 0);
}

/*
 * Handle after a dma read
 */
static void nvt_sd_post_dma_read(struct nvt_sd_host *host)
{
    struct mmc_command *cmd;
    struct mmc_data *data;
    unsigned int len, i, size;
    unsigned *dmabuf = host->buffer;

    cmd = host->cmd;
    if (!cmd) {
        nvt_sd_debug("no command\n");
        return;
    }

    data = cmd->data;
    if (!data) {
        nvt_sd_debug("no data\n");
        return;
    }

    size = data->blksz * data->blocks;
    len = data->sg_len;

    for (i = 0; i < len; i++) {
        struct scatterlist *sg;
        int amount;
        unsigned int *sgbuffer;

        sg = &data->sg[i];

        sgbuffer = kmap_atomic(sg_page(sg), KM_BIO_SRC_IRQ) + sg->offset;
        amount = min(size, sg->length);
        size -= amount;

        {
            char *tmpv = (char *)dmabuf;
            memcpy(sgbuffer, tmpv, amount);
            tmpv += amount;
            dmabuf = (unsigned *)tmpv;
        }

        flush_kernel_dcache_page(sg_page(sg));
        kunmap_atomic(sgbuffer, KM_BIO_SRC_IRQ);
        data->bytes_xfered += amount;
        if (size == 0)
            break;
    }
}

/*
 * Handle transmitted data
 */
static void nvt_sd_handle_transmitted(struct nvt_sd_host *host)
{
    //nvt_sd_debug("Handling the transmit\n");

    if (nvt_sd_read(REG_SDISR) & SDISR_CRC_IF)
        nvt_sd_write(REG_SDISR, SDISR_CRC_IF);

    /* check read/busy */

    if (host->port == 0)
        nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) | SDCR_CLK_KEEP);
#ifdef NVT_SD_SD1
    else if (host->port == 1)
        nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) | SDCR_CLK_KEEP1);
#endif
#ifdef NVT_SD_SD2
    else if (host->port == 2)
        nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) | SDCR_CLK_KEEP2);
#endif
    else
        printk("ERROR: Don't support SD port %d to transmitted data !\n", host->port);
}


/*
 * Update bytes tranfered count during a write operation
 */
static void nvt_sd_update_bytes_xfered(struct nvt_sd_host *host)
{
    struct mmc_data *data;

    /* always deal with the effective request (and not the current cmd) */
    if (host->request->cmd && host->request->cmd->error != 0)
        return;

    if (host->request->data) {
        data = host->request->data;
        if (data->flags & MMC_DATA_WRITE) {
            /* card is in IDLE mode now */
            data->bytes_xfered = data->blksz * data->blocks;
            //nvt_sd_debug("-> bytes_xfered %d, total_length = %d\n",
            //  data->bytes_xfered, host->total_length);
        }
    }
}


/*-----------------------------------------------------------------------------
 * Config SIC register to select SD port.
 *---------------------------------------------------------------------------*/
static int nvt_sd_select_port(u32 port)
{
    if (port == 0)
        nvt_sd_write(REG_SDCR, (nvt_sd_read(REG_SDCR) & (~SDCR_SDPORT)) | SDCR_SDPORT_0);   // SD Port 0 is selected
#ifdef NVT_SD_SD1
    else if (port == 1)
        nvt_sd_write(REG_SDCR, (nvt_sd_read(REG_SDCR) & (~SDCR_SDPORT)) | SDCR_SDPORT_1);   // SD Port 1 is selected
#endif
#ifdef NVT_SD_SD2
    else if (port == 2)
        nvt_sd_write(REG_SDCR, (nvt_sd_read(REG_SDCR) & (~SDCR_SDPORT)) | SDCR_SDPORT_2);   // SD Port 2 is selected
#endif
    else
    {
        printk("ERROR: Don't support SD port %d !\n", port);
        return -1;
    }

    //--- 2014/2/26, Reset SD controller and DMAC to keep clean status for next access.
    // Reset DMAC engine and interrupt satus
    nvt_sd_write(REG_DMACCSR, nvt_sd_read(REG_DMACCSR) | DMAC_SWRST | DMAC_EN);
    while(nvt_sd_read(REG_DMACCSR) & DMAC_SWRST);
    nvt_sd_write(REG_DMACCSR, nvt_sd_read(REG_DMACCSR) | DMAC_EN);
    nvt_sd_write(REG_DMACISR, WEOT_IF | TABORT_IF);     // clear all interrupt flag

    // Reset FMI engine and interrupt status
    nvt_sd_write(REG_FMICR, FMI_SWRST);
    while(nvt_sd_read(REG_FMICR) & FMI_SWRST);
    nvt_sd_write(REG_FMIISR, FMI_DAT_IF);               // clear all interrupt flag

    // Reset SD engine and interrupt status
    nvt_sd_write(REG_FMICR, FMI_SD_EN);
    nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) | SDCR_SWRST);
    while(nvt_sd_read(REG_SDCR) & SDCR_SWRST);
    nvt_sd_write(REG_SDISR, 0xFFFFFFFF);                // clear all interrupt flag

    return 0;
}


/*
 * Enable the controller
 */
static void nvt_sd_enable(struct nvt_sd_host *host)
{
    //--- Enable SIC and SD engine clock
    //nvt_sd_write(REG_AHBCLK, nvt_sd_read(REG_AHBCLK) | SIC_CKE | SD_CKE);

    //--- Set GPIO to SD card mode

    // SD 0 GPIO select: set GPE2~7 to SD card mode (SD0_CLK/SD0_CMD/DAT0[0~3])
    nvt_sd_write(REG_GPEFUN, (nvt_sd_read(REG_GPEFUN) & (~0x0000FFF0)) | 0x0000AAA0);

    // set GPA1 to SD mode for SD port 0 card detection
    nvt_sd_write(REG_GPAFUN, nvt_sd_read(REG_GPAFUN) | MF_GPA1);      // set GPIO to SD mode for card detect
    nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) | SDIER_CDSRC);    // set GPIO as SD card detect pin

#ifdef NVT_SD_SD1
    // SD 1 GPIO select: set GPB0~5 to SD card mode (SD1_CLK/SD1_CMD/DAT1[0~3])
    nvt_sd_write(REG_GPBFUN, (nvt_sd_read(REG_GPBFUN) & (~0x00000FFF)) | 0x00000AAA);
#endif

#ifdef NVT_SD_SD2
    // SD 2 GPIO select: set GPD5~8 and GPE8~9 to SD card mode (SD2_CLK/SD2_CMD/DAT2[0~3])
    nvt_sd_write(REG_GPEFUN, (nvt_sd_read(REG_GPEFUN) & (~0x000F0000)) | 0x00050000);
    nvt_sd_write(REG_GPDFUN, (nvt_sd_read(REG_GPDFUN) & (~0x0003FC00)) | 0x00015400);
#endif

#if defined (NVT_SD_SD1) || defined (NVT_SD_SD2)
  #if defined (CONFIG_SD1_CD_GPA3) || defined (CONFIG_SD2_CD_GPA3)
    //--- Select GPA3 as SD port 1/2 card detection. HIGH means card removed, LOW means inserted.
    // Set GPA3 to GPIO mode for SD port 1/2 card detection.
    nvt_sd_write(REG_GPAFUN, nvt_sd_read(REG_GPAFUN) & (~MF_GPA3));     // set GPA3 to GPIO mode
    nvt_sd_write(REG_GPIOA_PUEN, nvt_sd_read(REG_GPIOA_PUEN) | BIT3);   // set GPA3 internal pull high
    nvt_sd_write(REG_GPIOA_OMD, nvt_sd_read(REG_GPIOA_OMD) & (~BIT3));  // set GPA3 to input mode

    // Set GPA3 as interrupt pin and bind to GPIO1 interrupt
    nvt_sd_write(REG_IRQTGSRC0, BIT3);     // clear GPA3 interrupt status
    nvt_sd_write(REG_IRQSRCGPA, (nvt_sd_read(REG_IRQSRCGPA) & (~0x000000C0)) | 0x00000040);   // set GPA3 as GPIO1 interrupt trigger source
    nvt_sd_write(REG_IRQENGPA, nvt_sd_read(REG_IRQENGPA) | BIT3 | BIT19);   // set GPA3 trigger by both falling and rising edge
  #elif defined (CONFIG_SD1_CD_GPB6) || defined (CONFIG_SD2_CD_GPB6)
    //--- Select GPB6 as SD port 1/2 card detection. HIGH means card removed, LOW means inserted.
    // Set GPB6 to GPIO mode for SD port 1/2 card detection.
    nvt_sd_write(REG_GPBFUN, nvt_sd_read(REG_GPBFUN) & (~MF_GPB6));    // set GPB6 to GPIO mode
    nvt_sd_write(REG_GPIOB_PUEN, nvt_sd_read(REG_GPIOB_PUEN) | BIT6);  // set GPB6 internal pull high
    nvt_sd_write(REG_GPIOB_OMD, nvt_sd_read(REG_GPIOB_OMD) & (~BIT6)); // set GPB6 to input mode

    // Set GPB6 as interrupt pin and bind to GPIO1 interrupt
    nvt_sd_write(REG_IRQTGSRC0, BIT22);     // clear GPB6 interrupt status
    nvt_sd_write(REG_IRQSRCGPB, (nvt_sd_read(REG_IRQSRCGPB) & (~0x00003000)) | 0x00001000);   // set GPB6 as GPIO1 interrupt trigger source
    nvt_sd_write(REG_IRQENGPB, nvt_sd_read(REG_IRQENGPB) | BIT6 | BIT22);  // set GPB6 trigger by both falling and rising edge
  #elif defined (CONFIG_SD1_CD_GPE11) || defined (CONFIG_SD2_CD_GPE11)
    //--- Select GPE11 as SD port 1/2 card detection. HIGH means card removed, LOW means inserted.
    // Set GPE11 to GPIO mode for SD port 1/2 card detection.
    nvt_sd_write(REG_GPEFUN, nvt_sd_read(REG_GPEFUN) & (~MF_GPE11));    // set GPE11 to GPIO mode
    nvt_sd_write(REG_GPIOE_PUEN, nvt_sd_read(REG_GPIOE_PUEN) | BIT11);  // set GPE11 internal pull high
    nvt_sd_write(REG_GPIOE_OMD, nvt_sd_read(REG_GPIOE_OMD) & (~BIT11)); // set GPE11 to input mode

    // Set GPE11 as interrupt pin and bind to GPIO1 interrupt
    nvt_sd_write(REG_IRQTGSRC2, BIT11);     // clear GPE11 interrupt status
    nvt_sd_write(REG_IRQSRCGPE, (nvt_sd_read(REG_IRQSRCGPE) & (~0x00C00000)) | 0x00400000);   // set GPE11 as GPIO1 interrupt trigger source
    nvt_sd_write(REG_IRQENGPE, nvt_sd_read(REG_IRQENGPE) | BIT11 | BIT27);  // set GPE11 trigger by both falling and rising edge
  #elif defined (CONFIG_SD2_CD_GPE11_SW)
    //--- Select GPE11 OUTPUT mode as SD port 2 card detection. LOW means card removed, HIGH means inserted.
    // Set GPE11 to GPIO mode for SD port 2 card detection.
    nvt_sd_write(REG_GPEFUN, nvt_sd_read(REG_GPEFUN) & (~MF_GPE11));    // set GPE11 to GPIO mode
    nvt_sd_write(REG_GPIOE_PUEN, nvt_sd_read(REG_GPIOE_PUEN) | BIT11);  // set GPE11 internal pull high
    nvt_sd_write(REG_GPIOE_DOUT, nvt_sd_read(REG_GPIOE_DOUT) | BIT11);  // set GPE11 to output high
    nvt_sd_write(REG_GPIOE_OMD, nvt_sd_read(REG_GPIOE_OMD) | BIT11);    // set GPE11 to output mode

    // Set GPE11 as interrupt pin and bind to GPIO1 interrupt
    nvt_sd_write(REG_IRQTGSRC2, BIT11); // clear GPE11 interrupt status
    nvt_sd_write(REG_IRQSRCGPE, (nvt_sd_read(REG_IRQSRCGPE) & (~0x00C00000)) | 0x00400000); // set GPE11 as GPIO1 interrupt trigger source
    nvt_sd_write(REG_IRQENGPE, nvt_sd_read(REG_IRQENGPA) | BIT11 | BIT27);  // set GPE11 trigger by both falling and rising edge
  #endif
#endif

    //--- Initial SD engine
    nvt_sd_write(REG_DMACCSR, nvt_sd_read(REG_DMACCSR) | DMAC_EN);    // enable DMAC for FMI
    nvt_sd_write(REG_FMICR, FMI_SD_EN);     // enable SD
    nvt_sd_write(REG_SDISR, 0xFFFFFFFF);    // write bit 1 to clear all SDISR

    //--- Select SD port
    if (nvt_sd_select_port(host->port) != 0)
        return;

#ifdef CONFIG_FA93_SD0_WP
    //--- Initail GPA0 for SD card 0 write protect
    if (host->port == 0)
    {
        // set GPA0 to GPIO mode for SD port 0 write protect
        nvt_sd_write(REG_GPAFUN, nvt_sd_read(REG_GPAFUN) & (~MF_GPA0));     // set GPIO to GPIO mode for write protect
        nvt_sd_write(REG_GPIOA_OMD, nvt_sd_read(REG_GPIOA_OMD) & (~BIT0));  // set GPA0 to input mode
    }
#endif

    // SDNWR = 9+1 clock
    nvt_sd_write(REG_SDCR, (nvt_sd_read(REG_SDCR) & (~SDCR_SDNWR)) | 0x09000000);

    // SDCR_BLKCNT = 1
    nvt_sd_write(REG_SDCR, (nvt_sd_read(REG_SDCR) & (~SDCR_BLKCNT)) | 0x00010000);
}

/*
 * Disable the controller
 */
static void nvt_sd_disable(struct nvt_sd_host *host)
{
    nvt_sd_write(REG_SDISR, 0xFFFFFFFF);     // write bit 1 to clear all SDISR
    nvt_sd_write(REG_FMICR, nvt_sd_read(REG_FMICR) & (~FMI_SD_EN));   // disable SD
}

/*
 * Send a command
 */
static void nvt_sd_send_command(struct nvt_sd_host *host, struct mmc_command *cmd)
{
    unsigned int csr;
    unsigned int volatile block_length;
    struct mmc_data *data = cmd->data;
    unsigned int volatile blocks;
    int clock_free_run_status = 0;

    host->cmd = cmd;
    if (host->port == 0)
    {
        sd_host = host;
        sd_state = 0;
        sd_state_xfer = 0;
    }
#ifdef NVT_SD_SD1
    else if (host->port == 1)
    {
        sd1_host = host;
        sd1_state = 0;
        sd1_state_xfer = 0;
    }
#endif
#ifdef NVT_SD_SD2
    else if (host->port == 2)
    {
        sd2_host = host;
        sd2_state = 0;
        sd2_state_xfer = 0;
    }
#endif

    if (down_interruptible(&fmi_sem))   // get fmi_sem for whole SD command, include data read/write.
        return;

    if(nvt_sd_read(REG_FMICR) != FMI_SD_EN)
        nvt_sd_write(REG_FMICR, FMI_SD_EN);

    if (nvt_sd_select_port(host->port) != 0)
        return;

    //--- prepare initial value for SDCR register
    csr = nvt_sd_read(REG_SDCR) & 0xff00c080;   // clear BLK_CNT, CMD_CODE, and all xx_EN fields.

    //--- 2013/7/23, always disable SD clock free run to support SDIO card interrupt mode.
    if (host->port == 0)
    {
        clock_free_run_status = csr | SDCR_CLK_KEEP;
        csr = csr & (~SDCR_CLK_KEEP);
    }
#ifdef NVT_SD_SD1
    else if (host->port == 1)
    {
        clock_free_run_status = csr | SDCR_CLK_KEEP1;
        csr = csr & (~SDCR_CLK_KEEP1);
    }
#endif
#ifdef NVT_SD_SD2
    else if (host->port == 2)
    {
        clock_free_run_status = csr | SDCR_CLK_KEEP2;
        csr = csr & (~SDCR_CLK_KEEP2);
    }
#endif

    csr = csr | (cmd->opcode << 8) | SDCR_CO_EN;    // set command code and enable command out
    if (host->port == 0)
        sd_event |= SD_EVENT_CMD_OUT;
#ifdef NVT_SD_SD1
    else if (host->port == 1)
        sd1_event |= SD_EVENT_CMD_OUT;
#endif
#ifdef NVT_SD_SD2
    else if (host->port == 2)
        sd2_event |= SD_EVENT_CMD_OUT;
#endif

    if (host->bus_mode == MMC_BUS_WIDTH_4)
        csr |= SDCR_DBW;

    if (mmc_resp_type(cmd) != MMC_RSP_NONE) {
        /* if a response is expected then allow maximum response latancy */

        /* set 136 bit response for R2, 48 bit response otherwise */
        if (mmc_resp_type(cmd) == MMC_RSP_R2) {
            csr |= SDCR_R2_EN;
            if (host->port == 0)
                sd_event |= SD_EVENT_RSP2_IN;
#ifdef NVT_SD_SD1
            else if (host->port == 1)
                sd1_event |= SD_EVENT_RSP2_IN;
#endif
#ifdef NVT_SD_SD2
            else if (host->port == 2)
                sd2_event |= SD_EVENT_RSP2_IN;
#endif
        } else {
            csr |= SDCR_RI_EN;
            if (host->port == 0)
                sd_event |= SD_EVENT_RSP_IN;
#ifdef NVT_SD_SD1
            else if (host->port == 1)
                sd1_event |= SD_EVENT_RSP_IN;
#endif
#ifdef NVT_SD_SD2
            else if (host->port == 2)
                sd2_event |= SD_EVENT_RSP_IN;
#endif
        }
        nvt_sd_write(REG_SDISR, SDISR_RITO_IF);

        if (host->port == 0)
            sd_ri_timeout = 0;
#ifdef NVT_SD_SD1
        else if (host->port == 1)
            sd1_ri_timeout = 0;
#endif
#ifdef NVT_SD_SD2
        else if (host->port == 2)
            sd2_ri_timeout = 0;
#endif

        // CONFIG_SD_DISK_MOUNT_DELAY defined in Kconfig.
        // The valid value range between 0x1FFF and 0xFFFFFF.
        // The longer delay makes SD disk mount slower but more stable.
        nvt_sd_write(REG_SDTMOUT, CONFIG_SD_DISK_MOUNT_DELAY);  // timeout for CMD
    }

    if (data) {
        nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) | SDIER_BLKD_IEN); // Enable SD interrupt & select GPIO detect
        block_length = data->blksz;     // data block size, seem <= 512 for SD CMD
        blocks = data->blocks;          // number of block

        nvt_sd_write(REG_SDBLEN, block_length-1);
        if ((block_length > 512) || (blocks >= 256))
            printk("ERROR: SIC don't support read/write 256 blocks in one SD CMD !\n");
        else
            csr = (csr & (~SDCR_BLKCNT)) | (blocks << 16);
    } else {
        nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) & (~SDIER_BLKD_IEN)); // Disable SD interrupt & select GPIO detect
        block_length = 0;
        blocks = 0;
    }

    /*
     * Set the arguments and send the command
     */
    if (data) {
        data->bytes_xfered = 0;
        host->transfer_index = 0;
        host->in_use_index = 0;
        if (data->flags & MMC_DATA_READ) {
            /*
             * Handle a read
             */
            host->total_length = 0;
            nvt_sd_write(REG_DMACSAR, host->physical_address);
        } else if (data->flags & MMC_DATA_WRITE) {
            /*
             * Handle a write
             */
            if (down_interruptible(&dmac_sem))  // get dmac_sem for data writing.
                return;
            host->total_length = block_length * blocks;
            nvt_sd_sg_to_dma(host, data);
            //nvt_sd_debug("Transmitting %d bytes\n", host->total_length);
            nvt_sd_write(REG_DMACSAR, host->physical_address);
            csr = csr | SDCR_DO_EN;
        }
    }

    /*
     * Send the command and then enable the PDC - not the other way round as
     * the data sheet says
     */
    nvt_sd_set_clock(host->mmc->ios.clock / 1000);  // set SD clock for working SD port.

    nvt_sd_write(REG_SDARG, cmd->arg);
    // nvt_sd_debug("SD%d send cmd %d as 0x%08X, arg=0x%08X, blocks=%d, length=%d\n", host->port, cmd->opcode, csr, cmd->arg, blocks, block_length);
    nvt_sd_write(REG_SDCR, csr);

    if (host->port == 0)
    {
        sd_send_cmd = 1;
        wake_up_interruptible(&sd_event_wq);
        wait_event_interruptible(sd_wq, (sd_state != 0));
    }
#ifdef NVT_SD_SD1
    else if (host->port == 1)
    {
        sd1_send_cmd = 1;
        wake_up_interruptible(&sd1_event_wq);
        wait_event_interruptible(sd1_wq, (sd1_state != 0));
    }
#endif
#ifdef NVT_SD_SD2
    else if (host->port == 2)
    {
        sd2_send_cmd = 1;
        wake_up_interruptible(&sd2_event_wq);
        wait_event_interruptible(sd2_wq, (sd2_state != 0));
    }
#endif

    if (data) {
        if (data->flags & MMC_DATA_WRITE) {
            // waiting for SD card write completed and become ready.
            nvt_sd_wait_card_ready(host->mmc);
#if 0
            // SD clock don't free run any more
            if (host->port == 0)
                nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) & (~SDCR_CLK_KEEP));
#ifdef NVT_SD_SD1
            else if (host->port == 1)
                nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) & (~SDCR_CLK_KEEP1));
#endif
#ifdef NVT_SD_SD2
            else if (host->port == 2)
                nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) & (~SDCR_CLK_KEEP2));
#endif
            else
                printk("ERROR: Don't support SD port %d to stop free run SD clock !\n", host->port);
#endif
            up(&dmac_sem);  // release dmac_sem for data writing.
            nvt_sd_update_bytes_xfered(host);
        }
    }

    //--- 2013/7/23, restore SD clock free run status.
    if (clock_free_run_status)
    {
        if (host->port == 0)
            nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) | SDCR_CLK_KEEP);
#ifdef NVT_SD_SD1
        else if (host->port == 1)
            nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) | SDCR_CLK_KEEP1);
#endif
#ifdef NVT_SD_SD2
        else if (host->port == 2)
            nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) | SDCR_CLK_KEEP2);
#endif
        else
            printk("ERROR: Don't support SD port %d to stop free run SD clock !\n", host->port);
    }

    up(&fmi_sem);   // release fmi_sem for whole SD command, include data readd/write.
    mmc_request_done(host->mmc, host->request);
}


/*
 * Send stop command
 */
static void nvt_sd_send_stop(struct nvt_sd_host *host, struct mmc_command *cmd)
{
    unsigned int csr;
    unsigned int volatile block_length;
    unsigned int volatile blocks;

    host->cmd = cmd;

    if (host->port == 0)
    {
        sd_host = host;
        sd_state = 0;
        sd_state_xfer = 0;
    }
#ifdef NVT_SD_SD1
    else if (host->port == 1)
    {
        sd1_host = host;
        sd1_state = 0;
        sd1_state_xfer = 0;
    }
#endif
#ifdef NVT_SD_SD2
    else if (host->port == 2)
    {
        sd2_host = host;
        sd2_state = 0;
        sd2_state_xfer = 0;
    }
#endif

    if(nvt_sd_read(REG_FMICR) != FMI_SD_EN)
        nvt_sd_write(REG_FMICR, FMI_SD_EN);

    if (nvt_sd_select_port(host->port) != 0)
        return;

    //--- prepare initial value for SDCR register
    csr = nvt_sd_read(REG_SDCR) & 0xff00c080;   // clear BLK_CNT, CMD_CODE, and all xx_EN fields.

    csr = csr | (cmd->opcode << 8) | SDCR_CO_EN;   // set command code and enable command out
    if (host->port == 0)
        sd_event |= SD_EVENT_CMD_OUT;
#ifdef NVT_SD_SD1
    else if (host->port == 1)
        sd1_event |= SD_EVENT_CMD_OUT;
#endif
#ifdef NVT_SD_SD2
    else if (host->port == 2)
        sd2_event |= SD_EVENT_CMD_OUT;
#endif

    if (host->bus_mode == MMC_BUS_WIDTH_4)
        csr |= SDCR_DBW;

    if (mmc_resp_type(cmd) != MMC_RSP_NONE) {
        /* if a response is expected then allow maximum response latancy */

        /* set 136 bit response for R2, 48 bit response otherwise */
        if (mmc_resp_type(cmd) == MMC_RSP_R2) {
            csr |= SDCR_R2_EN;
            if (host->port == 0)
                sd_event |= SD_EVENT_RSP2_IN;
#ifdef NVT_SD_SD1
            else if (host->port == 1)
                sd1_event |= SD_EVENT_RSP2_IN;
#endif
#ifdef NVT_SD_SD2
            else if (host->port == 2)
                sd2_event |= SD_EVENT_RSP2_IN;
#endif
        } else {
            csr |= SDCR_RI_EN;
            if (host->port == 0)
                sd_event |= SD_EVENT_RSP_IN;
#ifdef NVT_SD_SD1
            else if (host->port == 1)
                sd1_event |= SD_EVENT_RSP_IN;
#endif
#ifdef NVT_SD_SD2
            else if (host->port == 2)
                sd2_event |= SD_EVENT_RSP_IN;
#endif
        }
        nvt_sd_write(REG_SDISR, SDISR_RITO_IF);

        if (host->port == 0)
            sd_ri_timeout = 0;
#ifdef NVT_SD_SD1
        else if (host->port == 1)
            sd1_ri_timeout = 0;
#endif
#ifdef NVT_SD_SD2
        else if (host->port == 2)
            sd2_ri_timeout = 0;
#endif

        nvt_sd_write(REG_SDTMOUT, CONFIG_SD_DISK_MOUNT_DELAY);  // timeout for STOP CMD
    }

    nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) & (~SDIER_BLKD_IEN)); // Disable SD interrupt & select GPIO detect
    block_length = 0;
    blocks = 0;

    /*
     * Set the arguments and send the command
     */
    nvt_sd_write(REG_SDARG, cmd->arg);
    nvt_sd_write(REG_SDCR, csr);

    if (host->port == 0)
    {
        sd_send_cmd = 1;
        wake_up_interruptible(&sd_event_wq);
    }
#ifdef NVT_SD_SD1
    else if (host->port == 1)
    {
        sd1_send_cmd = 1;
        wake_up_interruptible(&sd1_event_wq);
    }
#endif
#ifdef NVT_SD_SD2
    else if (host->port == 2)
    {
        sd2_send_cmd = 1;
        wake_up_interruptible(&sd2_event_wq);
    }
#endif
    mmc_request_done(host->mmc, host->request);
    // nvt_sd_debug("--> nvt_sd_send_stop(): SD %d, cmd %d, END !!\n", host->port, cmd->opcode);
}


/*
 * Process the request
 */
static void nvt_sd_send_request(struct nvt_sd_host *host)
{
    if (!(host->flags & FL_SENT_COMMAND)) {
        host->flags |= FL_SENT_COMMAND;
        nvt_sd_send_command(host, host->request->cmd);
    } else if ((!(host->flags & FL_SENT_STOP)) && host->request->stop) {
        host->flags |= FL_SENT_STOP;
        nvt_sd_send_stop(host, host->request->stop);
    } else {
        if (host->port == 0)
        {
            sd_state = 1;
            wake_up_interruptible(&sd_wq);
        }
#ifdef NVT_SD_SD1
        else if (host->port == 1)
        {
            sd1_state = 1;
            wake_up_interruptible(&sd1_wq);
        }
#endif
#ifdef NVT_SD_SD2
        else if (host->port == 2)
        {
            sd2_state = 1;
            wake_up_interruptible(&sd2_wq);
        }
#endif
        del_timer(&host->timer);
    }
}

/*
 * Handle a command that has been completed
 */
static void nvt_sd_completed_command(struct nvt_sd_host *host, unsigned int status)
{
    struct mmc_command *cmd = host->cmd;
    struct mmc_data *data = cmd->data;
    unsigned int i, j, tmp[5], err;
    unsigned char *ptr;

    err = nvt_sd_read(REG_SDISR);

    if ((err & SDISR_RITO_IF) || (cmd->error)) {
        //nvt_sd_debug("got SDISR_RITO_IF, response timeout, SDISR=0x%08X, error=0x%08X\n", err, cmd->error);
        nvt_sd_write(REG_SDTMOUT, 0x0);
        nvt_sd_write(REG_SDISR, SDISR_RITO_IF);
        cmd->error = -ETIMEDOUT;
        cmd->resp[0] = cmd->resp[1] = cmd->resp[2] = cmd->resp[3] = 0;
    } else {
        if (status & SD_EVENT_RSP_IN) {
            // if not R2
            //nvt_sd_debug("got response, SDISR=0x%08X\n", err);
            cmd->resp[0] = (nvt_sd_read(REG_SDRSP0) << 8)|(nvt_sd_read(REG_SDRSP1) & 0xff);
            cmd->resp[1] = cmd->resp[2] = cmd->resp[3] = 0;
        } else if (status & SD_EVENT_RSP2_IN) {
            // if R2
            //nvt_sd_debug("got response R2, SDISR=0x%08X\n", err);
            ptr = (unsigned char *)REG_FB_0;    // pointer to DMA buffer
            for (i=0, j=0; j<5; i+=4, j++)
                tmp[j] = (*(ptr+i)<<24)|(*(ptr+i+1)<<16)|(*(ptr+i+2)<<8)|(*(ptr+i+3));
            for (i=0; i<4; i++)
                cmd->resp[i] = ((tmp[i] & 0x00ffffff)<<8)|((tmp[i+1] & 0xff000000)>>24);
        }
    }
    //nvt_sd_debug("    Event = 0x%0X, Resp = [0x%08X 0x%08X 0x%08X 0x%08X]\n", status, cmd->resp[0], cmd->resp[1], cmd->resp[2], cmd->resp[3]);

    if (!cmd->error) {
        if ((err & SDISR_CRC_7) == 0) {
            if (!(mmc_resp_type(cmd) & MMC_RSP_CRC)) {
                // some response don't contain CRC-7 info (ex. R3), then software should ignore SDISR_CRC_7 bit.
                cmd->error = 0;
                nvt_sd_write(REG_SDISR, SDISR_CRC_IF);
            } else {
                // really CRC-7 error
                cmd->error = -EIO;
                nvt_sd_debug("CRC-7 error detected and set to %d/%d (cmd = %d, retries = %d)\n",
                    cmd->error, data ? data->error : 0,
                    cmd->opcode, cmd->retries);
                nvt_sd_write(REG_SDISR, SDISR_CRC_IF);  // clear CRC interrupt flag
                // When CRC error is occurred, software should reset SD engine.
                nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) | SDCR_SWRST); // software reset SD engine
                while (nvt_sd_read(REG_SDCR) & SDCR_SWRST); // waiting for reset completed
            }
        } else
            cmd->error = 0;

        if (data) {
            data->bytes_xfered = 0;
            host->transfer_index = 0;
            host->in_use_index = 0;
            if (data->flags & MMC_DATA_READ) {
                if (down_interruptible(&dmac_sem))  // get dmac_sem for data reading.
                    return;

                nvt_sd_write(REG_SDTMOUT, 0xffffff);    // longer timeout to read more data
                nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) | SDCR_DI_EN);
            }

            if (host->port == 0)
                wait_event_interruptible(sd_wq_xfer, (sd_state_xfer != 0));
#ifdef NVT_SD_SD1
            else if (host->port == 1)
                wait_event_interruptible(sd1_wq_xfer, (sd1_state_xfer != 0));
#endif
#ifdef NVT_SD_SD2
            else if (host->port == 2)
                wait_event_interruptible(sd2_wq_xfer, (sd2_state_xfer != 0));
#endif
            if (data->flags & MMC_DATA_READ)
                up(&dmac_sem);  // release dmac_sem for data reading.
        }
    }
    nvt_sd_send_request(host);
}


/*
 * Handle an MMC request
 */
static void nvt_sd_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
    struct nvt_sd_host *host = mmc_priv(mmc);
    int card_present;

    host->request = mrq;
    host->flags = 0;

    /* more than 1s timeout needed with slow SD cards */
    //mod_timer(&host->timer, jiffies +  msecs_to_jiffies(2000));
    if (down_interruptible(&fmi_sem))
        return;
    card_present = nvt_sd_card_detect(mmc);
    up(&fmi_sem);

    if (!card_present) {
        //nvt_sd_debug("no medium present\n");
        host->request->cmd->error = -ENOMEDIUM;
        mmc_request_done(host->mmc, host->request);
    } else
        nvt_sd_send_request(host);
}


extern unsigned int w55fa93_upll_clock;
/*-----------------------------------------------------------------------------
 * 2011/6/24, To set up the clock for SD_CLK
 *      SD_CLK = UPLL / ((CLKDIV2[SD_N0] + 1) * (CLKDIV2[SD_N1] + 1))
 * INPUT: sd_clock_khz: the SD clock you wanted with unit KHz.
 *---------------------------------------------------------------------------*/
// there are 3 bits for divider N0, maximum is 8
#define SD_CLK_DIV0_MAX     8
// there are 8 bits for divider N1, maximum is 256
#define SD_CLK_DIV1_MAX     256
void nvt_sd_set_clock(u32 sd_clock_khz)
{
    u32 rate, div0, div1;
    unsigned int upll_clock = w55fa93_upll_clock;

    //nvt_sd_debug("Set SD clock to %d KHz.\n", sd_clock_khz);

    //--- calculate the rate that 2 divider have to divide
    // upll_clock is the UPLL input clock with unit KHz
    if (sd_clock_khz > upll_clock)
    {
        printk("ERROR: wrong SD clock %dKHz setting since it is faster than input clock %dKHz !\n",
            sd_clock_khz, upll_clock);
        return;
    }

    if (sd_clock_khz <= 0)
    {
        printk("WARNING: cannot set SD clock to 0Hz. Ignore it !\n");
        return;
    }

    rate = upll_clock / sd_clock_khz;
    // choose slower clock if system clock cannot divisible by wanted clock
    if (upll_clock % sd_clock_khz != 0)
        rate++;
    if (rate > (SD_CLK_DIV0_MAX * SD_CLK_DIV1_MAX)) // the maximum divider for SD_CLK is (SD_CLK_DIV0_MAX * SD_CLK_DIV1_MAX)
    {
        printk("ERROR: wrong SD clock %dKHz setting since it is slower than input clock %dKHz/%d !\n",
            sd_clock_khz, upll_clock, SD_CLK_DIV0_MAX * SD_CLK_DIV1_MAX);
        return;
    }

    //--- choose a suitable value for first divider CLKDIV2[SD_N0]
    for (div0 = SD_CLK_DIV0_MAX; div0 > 0; div0--)    // choose the maximum value if can exact division
    {
        if (rate % div0 == 0)
            break;
    }
    if (div0 == 0) // cannot exact division
    {
        // if rate <= SD_CLK_DIV1_MAX, set div0 to 1 since div1 can exactly divide input clock
        div0 = (rate <= SD_CLK_DIV1_MAX) ? 1 : SD_CLK_DIV0_MAX;
    }

    //--- calculate the second divider CLKDIV2[SD_N1]
    div1 = rate / div0;
    div1 &= 0xFF;
    // nvt_sd_debug("UPLL=%dKHz (rate=%d, div0=%d, div1=%d), SD clock=%dKHz\n", upll_clock, rate, div0, div1, upll_clock/rate);

    //--- setup register
    nvt_sd_write(REG_CLKDIV2, (nvt_sd_read(REG_CLKDIV2) & ~SD_N0) | ((div0-1) << 16));  // SD clock divided by CLKDIV2[SD_N0]
    nvt_sd_write(REG_CLKDIV2, (nvt_sd_read(REG_CLKDIV2) & ~SD_N1) | ((div1-1) << 24));    // SD clock divider by CLKDIV2[SD_N1]
    nvt_sd_write(REG_CLKDIV2,  nvt_sd_read(REG_CLKDIV2) | SD_S);  // set SD clock from UPLL
    udelay(10);     // delay to wait SD clock become stable.
    return;
}


/*
 * Handle an MMC request
 */
static void nvt_sd_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
    struct nvt_sd_host *host = mmc_priv(mmc);

    host->bus_mode = ios->bus_width;

    if (down_interruptible(&fmi_sem))
        return;

    /* maybe switch power to the card */
    switch (ios->power_mode) {
        case MMC_POWER_OFF:
            // Disable the SD function in FMI controller
            nvt_sd_write(REG_FMICR, nvt_sd_read(REG_FMICR) & (~FMI_SD_EN));
            //nvt_sd_debug("SD power OFF\n");
            break;
        case MMC_POWER_UP:
        case MMC_POWER_ON:
            //if (ios->power_mode == MMC_POWER_UP)
            //    nvt_sd_debug("SD power UP.\n");
            //else
            //    nvt_sd_debug("SD power ON\n");

            // Enable the SD function in FMI controller
            nvt_sd_write(REG_FMICR, FMI_SD_EN);
            if (ios->clock == 0)
                nvt_sd_set_clock(300);      // default SD clock 300KHz
            else
                nvt_sd_set_clock(ios->clock / 1000);    // ios->clock unit is Hz

            nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) | SDCR_74CLK_OE);
            while (nvt_sd_read(REG_SDCR) & SDCR_74CLK_OE);  // waiting for 74 clock completed.
            break;
        default:
            WARN_ON(1);
    }

    if (ios->bus_width == MMC_BUS_WIDTH_4) {
        // nvt_sd_debug("MMC: Setting controller bus width to 4\n");
        nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) | SDCR_DBW);
    } else {
        // nvt_sd_debug("MMC: Setting controller bus width to 1\n");
        nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) & (~SDCR_DBW));
    }

    up(&fmi_sem);
}


/*
 * Handle CO, RI, and R2 event
 */
static int sd_event_thread(void *unused)
{
    int event = 0;
    int completed = 0;

    daemonize("sdioeventd");

    //-----------------------------------------------------------------------------------
    // 2014/1/8, the SD card could not be mount as a disk if it inserted but
    //  system not yet ready to mount disk. It could happen when system booting.
    //  To fix this problem, we have to postpone the SD card insert event till
    //  the system ready. The process are
    //  1. Don't report and handle any SD event before nvt_sd_init() completed.
    //  2. Detect SD card status in thread sd_event_thread() to trigger SD card event
    //     after nvt_sd_init() completed.
    //-----------------------------------------------------------------------------------
    //--- Waiting for nvt_sd_init() completed
    while (sd_init_completed == 0)
        schedule();

    mmc_detect_change(sd_host->mmc, msecs_to_jiffies(500));     // detect SD card status

    for (;;) {
        wait_event_interruptible(sd_event_wq, (sd_event != SD_EVENT_NONE) && (sd_send_cmd));

        completed = 0;
        event = sd_event;
        sd_event = SD_EVENT_NONE;
        sd_send_cmd = 0;
        if (event & SD_EVENT_CMD_OUT) {
            while (1) {
                if (!(nvt_sd_read(REG_SDCR) & SDCR_CO_EN)) {
                    completed = 1;
                    break;
                }
            }
        }

        if (event & SD_EVENT_RSP_IN) {
            while (1) {
                if (!(nvt_sd_read(REG_SDCR) & SDCR_RI_EN)) {
                    completed = 1;
                    break;
                }

                if (nvt_sd_read(REG_SDISR) & SDISR_RITO_IF) {
                    nvt_sd_write(REG_SDTMOUT, 0x0);
                    nvt_sd_write(REG_SDISR, SDISR_RITO_IF);

                    completed = 1;
                    sd_host->cmd->error = -ETIMEDOUT;
                    break;
                }
            }
        }

        if (event & SD_EVENT_RSP2_IN) {
            while (1) {
                if (!(nvt_sd_read(REG_SDCR) & SDCR_R2_EN)) {
                    completed = 1;
                    break;
                }
            }
        }

        if (completed) {
            //nvt_sd_debug("Completed command\n");
            nvt_sd_completed_command(sd_host, event);
        }
    }
    nvt_sd_debug("event quit\n");
    return 0;
}


#ifdef NVT_SD_SD1
/*
 * Handle CO, RI, and R2 event for SD port 1
 */
static int sd1_event_thread(void *unused)
{
    int event = 0;
    int completed = 0;

    daemonize("sdio1eventd");

    //--- Waiting for nvt_sd_init() completed
    while (sd_init_completed == 0)
        schedule();

    mmc_detect_change(sd1_host->mmc, msecs_to_jiffies(500));    // detect SD card status

    for (;;) {
        wait_event_interruptible(sd1_event_wq, (sd1_event != SD_EVENT_NONE) && (sd1_send_cmd));

        completed = 0;
        event = sd1_event;
        sd1_event = SD_EVENT_NONE;
        sd1_send_cmd = 0;
        if (event & SD_EVENT_CMD_OUT) {
            while (1) {
                if (!(nvt_sd_read(REG_SDCR) & SDCR_CO_EN)) {
                    completed = 1;
                    break;
                }
            }
        }

        if (event & SD_EVENT_RSP_IN) {
            while (1) {
                if (!(nvt_sd_read(REG_SDCR) & SDCR_RI_EN)) {
                    completed = 1;
                    break;
                }

                if (nvt_sd_read(REG_SDISR) & SDISR_RITO_IF) {
                    nvt_sd_write(REG_SDTMOUT, 0x0);
                    nvt_sd_write(REG_SDISR, SDISR_RITO_IF);

                    completed = 1;
                    sd1_host->cmd->error = -ETIMEDOUT;
                    break;
                }
            }
        }

        if (event & SD_EVENT_RSP2_IN) {
            while (1) {
                if (!(nvt_sd_read(REG_SDCR) & SDCR_R2_EN)) {
                    completed = 1;
                    break;
                }
            }
        }

        if (completed) {
            //nvt_sd_debug("Completed command\n");
            nvt_sd_completed_command(sd1_host, event);
        }
    }
    nvt_sd_debug("SD1 event quit\n");
    return 0;
}
#endif  // end of NVT_SD_SD1


#ifdef NVT_SD_SD2
/*
 * Handle CO, RI, and R2 event for SD port 2
 */
static int sd2_event_thread(void *unused)
{
    int event = 0;
    int completed = 0;

    daemonize("sdio2eventd");

    //--- Waiting for nvt_sd_init() completed
    while (sd_init_completed == 0)
        schedule();

    mmc_detect_change(sd2_host->mmc, msecs_to_jiffies(500));    // detect SD card status

    for (;;) {
        wait_event_interruptible(sd2_event_wq, (sd2_event != SD_EVENT_NONE) && (sd2_send_cmd));

        completed = 0;
        event = sd2_event;
        sd2_event = SD_EVENT_NONE;
        sd2_send_cmd = 0;
        if (event & SD_EVENT_CMD_OUT) {
            while (1) {
                if (!(nvt_sd_read(REG_SDCR) & SDCR_CO_EN)) {
                    completed = 1;
                    break;
                }
            }
        }

        if (event & SD_EVENT_RSP_IN) {
            while (1) {
                if (!(nvt_sd_read(REG_SDCR) & SDCR_RI_EN)) {
                    completed = 1;
                    break;
                }

                if (nvt_sd_read(REG_SDISR) & SDISR_RITO_IF) {
                    nvt_sd_write(REG_SDTMOUT, 0x0);
                    nvt_sd_write(REG_SDISR, SDISR_RITO_IF);

                    completed = 1;
                    sd2_host->cmd->error = -ETIMEDOUT;
                    break;
                }
            }
        }

        if (event & SD_EVENT_RSP2_IN) {
            while (1) {
                if (!(nvt_sd_read(REG_SDCR) & SDCR_R2_EN)) {
                    completed = 1;
                    break;
                }
            }
        }

        if (completed) {
            //nvt_sd_debug("Completed command\n");
            nvt_sd_completed_command(sd2_host, event);
        }
    }
    nvt_sd_debug("SD2 event quit\n");
    return 0;
}
#endif  // end of NVT_SD_SD2

/*
 * Handle an shared interrupt
 */
static irqreturn_t nvt_sd_irq(int irq, void *devid)
{
    struct nvt_sd_host *host = devid;
    unsigned int int_status;
    unsigned int reg_port_select;

    int_status = nvt_sd_read(REG_SDISR);
    reg_port_select = (nvt_sd_read(REG_SDCR) & SDCR_SDPORT) >> 29;

    //nvt_sd_debug("Check SD %d irq: SDISR status = 0x%08X, port select = %d\n", host->port, int_status, reg_port_select);

    // SD card port 0 for SDIO interrupt
    if ((host->port == 0) && (int_status & SDISR_SDIO_IF)) {
        //nvt_sd_debug("--> MMC/SDIO: Clear SDIO0 interrupt flag. DAT1=0x%x\n", int_status & SDISR_SD_DATA1);
        nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) & (~SDIER_SDIO_IEN));    // Disable SDIO0 interrupt before clear interrupt flag
        nvt_sd_write(REG_SDISR, SDISR_SDIO_IF);    // write 1 to clear interrupt flag
        mmc_signal_sdio_irq(host->mmc);     // clear IRQ flag and wake up IRQ thread
    }

#ifdef NVT_SD_SD1
    if ((host->port == 1) && (int_status & SDISR_SDIO1_IF)) {
        //nvt_sd_debug("--> MMC/SDIO: Clear SDIO1 interrupt flag. DAT1=0x%x\n", int_status & SDISR_SD1_DATA1);
        nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) & (~SDIER_SDIO1_IEN));    // Disable SDIO1 interrupt before clear interrupt flag
        nvt_sd_write(REG_SDISR, SDISR_SDIO1_IF);    // write 1 to clear interrupt flag
        mmc_signal_sdio_irq(host->mmc);     // clear IRQ flag and wake up IRQ thread
    }
#endif

#ifdef NVT_SD_SD2
    if ((host->port == 2) && (int_status & SDISR_SDIO2_IF)) {
        //nvt_sd_debug("--> MMC/SDIO: Clear SDIO2 interrupt flag. DAT1=0x%x\n", int_status & SDISR_SD2_DATA1);
        nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) & (~SDIER_SDIO1_IEN));   // Disable SDIO1 interrupt before clear interrupt flag
        nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) & (~SDIER_SDIO2_IEN));   // Disable SDIO2 interrupt before clear interrupt flag
        nvt_sd_write(REG_SDISR, SDISR_SDIO2_IF);    // write 1 to clear interrupt flag
        mmc_signal_sdio_irq(host->mmc);     // clear IRQ flag and wake up IRQ thread
    }
#endif

    if (int_status & SDISR_BLKD_IF) {
        //nvt_sd_debug("Block transfer has ended\n");
        //nvt_sd_debug("SD %d irq: port select = %d\n", host->port, reg_port_select);
        if (host->port != reg_port_select)
        {
            // This shared interrupt is not come from this device host. Ignore it.
            //nvt_sd_debug("SD %d irq: return IRQ_NONE.\n", host->port);
            return IRQ_NONE;    // interrupt was not from this device
        }

        if ((host->cmd == 0) || (host->cmd->data == 0))
        {
            nvt_sd_debug("SD %d irq: port select = %d, found NULL pointer!!\n", host->port, reg_port_select);
            return IRQ_NONE;
        }

        if (host->cmd->data->flags & MMC_DATA_WRITE) {
            nvt_sd_handle_transmitted(host);
        } else if (host->cmd->data->flags & MMC_DATA_READ) {
            nvt_sd_post_dma_read(host);

            //-- - check CRC-16 error for data-in transfer
            if (int_status & SDISR_CRC_IF)
            {
                // 2014/5/16, according to comment in mmc_sd_init_card() in core/sd.c, it said
                //      "This CRC enable is located AFTER the reading of the
                //       card registers because some SDHC cards are not able
                //       to provide valid CRCs for non-512-byte blocks."
                // The ACMD51 will read 8 bytes SCR register and could trigger invalid CRC-16 error.
                // The CMD6 will read 64 bytes SSR register and could trigger invalid CRC-16 error.
                // So, SD driver ignore CRC-16 error check for command code 51 and 6 here.
                if ((host->cmd->opcode != 51) && (host->cmd->opcode != 6))
                {
                    host->cmd->error = -EIO;
                    if ((nvt_sd_read(REG_SDISR) & SDISR_CRC_16) == 0)
                        nvt_sd_debug("CRC-16 error detected and set to %d (cmd = %d, retries = %d, SDISR=0x%08X)\n",
                                      host->cmd->error, host->cmd->opcode, host->cmd->retries, int_status);
                    nvt_sd_write(REG_SDISR, SDISR_CRC_IF);  // clear CRC interrupt flag
                    // When CRC error is occurred, software should reset SD engine.
                    nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) | SDCR_SWRST); // software reset SD engine
                    while (nvt_sd_read(REG_SDCR) & SDCR_SWRST); // waiting for reset completed
                }
            }
        }
        nvt_sd_write(REG_SDISR, SDISR_BLKD_IF);    // write 1 to clear interrupt flag

        if (host->port == 0)
        {
            sd_state_xfer = 1;
            wake_up_interruptible(&sd_wq_xfer);
        }
#ifdef NVT_SD_SD1
        else if (host->port == 1)
        {
            sd1_state_xfer = 1;
            wake_up_interruptible(&sd1_wq_xfer);
        }
#endif
#ifdef NVT_SD_SD2
        else if (host->port == 2)
        {
            sd2_state_xfer = 1;
            wake_up_interruptible(&sd2_wq_xfer);
        }
#endif
    }

    /*
     * we expect this irq on both insert and remove,
     * and use a short delay to debounce.
     */

    /* SD card port 0 detect */
    if (host->port == 0)
    {
        if (int_status & SDISR_CD_IF)
        {
            //nvt_sd_debug("--> nvt_sd_irq() GPA1 interrupt happened !\n");
            host->present = int_status & SDISR_CD_Card;
            /* 0.5s needed because of early card detect switch firing */
            mmc_detect_change(host->mmc, msecs_to_jiffies(500));
            nvt_sd_write(REG_SDISR, SDISR_CD_IF);  // write 1 to clear interrupt flag
        }
    }

    return IRQ_HANDLED;
}


#if (defined(NVT_SD_SD1) && (!defined(CONFIG_SD1_CD_NONE)))
//-------------------------------------------------
// Handle the GPIO1 interrupt for SD1 card detection.
//      SD1 card detect interrupt bind to GPIO1 interrupt
//-------------------------------------------------
static irqreturn_t nvt_sd1_card_detect_irq(int irq, void *devid)
{
    struct nvt_sd_host *host = devid;
    u32 src;

#ifdef CONFIG_SD1_CD_GPA3
    src = nvt_sd_read(REG_IRQTGSRC0);
    if (src & BIT3) // This interrupt is trigger by GPA3
    {
        //nvt_sd_debug("--> nvt_sd1_card_detect_irq() GPA3 interrupt happened !\n");
        if (nvt_sd_read(REG_GPIOA_PIN) & BIT3)
            host->present = 1;  // card removed
        else
            host->present = 0;  // card inserted
        /* 0.5s needed because of early card detect switch firing */
        mmc_detect_change(host->mmc, msecs_to_jiffies(500));
        nvt_sd_write(REG_IRQTGSRC0, BIT3); // clear GPA3 interrupt status
        return IRQ_HANDLED;
    }
    else
        return IRQ_NONE;    // interrupt was not from this device
#elif defined (CONFIG_SD1_CD_GPB6)
    src = nvt_sd_read(REG_IRQTGSRC0);
    if (src & BIT22) // This interrupt is trigger by GPB6
    {
        // nvt_sd_debug("--> nvt_sd1_card_detect_irq() GPB6 interrupt happened !\n");
        if (nvt_sd_read(REG_GPIOB_PIN) & BIT6)
            host->present = 1;  // card removed
        else
            host->present = 0;  // card inserted
        /* 0.5s needed because of early card detect switch firing */
        mmc_detect_change(host->mmc, msecs_to_jiffies(500));
        nvt_sd_write(REG_IRQTGSRC0, BIT22); // clear GPB6 interrupt status
        return IRQ_HANDLED;
    }
    else
        return IRQ_NONE;    // interrupt was not from this device
#elif defined (CONFIG_SD1_CD_GPE11)
    src = nvt_sd_read(REG_IRQTGSRC2);
    if (src & BIT11) // This interrupt is trigger by GPE11
    {
        // nvt_sd_debug("--> nvt_sd1_card_detect_irq() GPE11 interrupt happened !\n");
        if (nvt_sd_read(REG_GPIOE_PIN) & BIT11)
            host->present = 1;  // card removed
        else
            host->present = 0;  // card inserted
        /* 0.5s needed because of early card detect switch firing */
        mmc_detect_change(host->mmc, msecs_to_jiffies(500));
        nvt_sd_write(REG_IRQTGSRC2, BIT11); // clear GPE11 interrupt status
        return IRQ_HANDLED;
    }
    else
        return IRQ_NONE;    // interrupt was not from this device
#endif
}
#endif  // end of NVT_SD_SD1


#if (defined(NVT_SD_SD2) && (!defined(CONFIG_SD2_CD_NONE)))
//-------------------------------------------------
// Handle the GPIO1 interrupt for SD2 card detection.
//      SD2 card detect interrupt bind to GPIO1 interrupt
//-------------------------------------------------
static irqreturn_t nvt_sd2_card_detect_irq(int irq, void *devid)
{
    struct nvt_sd_host *host = devid;
    u32 src;

#ifdef CONFIG_SD2_CD_GPA3
    src = nvt_sd_read(REG_IRQTGSRC0);
    if (src & BIT3) // This interrupt is trigger by GPA3
    {
        //nvt_sd_debug("--> nvt_sd2_card_detect_irq() GPA3 interrupt happened !\n");
        if (nvt_sd_read(REG_GPIOA_PIN) & BIT3)
            host->present = 1;  // card removed
        else
            host->present = 0;  // card inserted
        /* 0.5s needed because of early card detect switch firing */
        mmc_detect_change(host->mmc, msecs_to_jiffies(500));
        nvt_sd_write(REG_IRQTGSRC0, BIT3); // clear GPA3 interrupt status
        return IRQ_HANDLED;
    }
    else
        return IRQ_NONE;    // interrupt was not from this device
#elif defined (CONFIG_SD2_CD_GPB6)
    src = nvt_sd_read(REG_IRQTGSRC0);
    if (src & BIT22) // This interrupt is trigger by GPB6
    {
        // nvt_sd_debug("--> nvt_sd2_card_detect_irq() GPB6 interrupt happened !\n");
        if (nvt_sd_read(REG_GPIOB_PIN) & BIT6)
            host->present = 1;  // card removed
        else
            host->present = 0;  // card inserted
        /* 0.5s needed because of early card detect switch firing */
        mmc_detect_change(host->mmc, msecs_to_jiffies(500));
        nvt_sd_write(REG_IRQTGSRC0, BIT22); // clear GPB6 interrupt status
        return IRQ_HANDLED;
    }
    else
        return IRQ_NONE;    // interrupt was not from this device
#elif defined (CONFIG_SD2_CD_GPE11)
    src = nvt_sd_read(REG_IRQTGSRC2);
    if (src & BIT11) // This interrupt is trigger by GPE11
    {
        // nvt_sd_debug("--> nvt_sd2_card_detect_irq() GPE11 interrupt happened !\n");
        if (nvt_sd_read(REG_GPIOE_PIN) & BIT11)
            host->present = 1;  // card removed
        else
            host->present = 0;  // card inserted
        /* 0.5s needed because of early card detect switch firing */
        mmc_detect_change(host->mmc, msecs_to_jiffies(500));
        nvt_sd_write(REG_IRQTGSRC2, BIT11); // clear GPE11 interrupt status
        return IRQ_HANDLED;
    }
    else
        return IRQ_NONE;    // interrupt was not from this device
#elif defined (CONFIG_SD2_CD_GPE11_SW)
    src = nvt_sd_read(REG_IRQTGSRC2);
    if (src & BIT11)    // This interrupt is trigger by GPE11
    {
        // nvt_sd_debug("--> nvt_sd2_card_detect_irq() GPE11 OUTPUT mode interrupt happened !\n");
        if (nvt_sd_read(REG_GPIOE_DOUT) & BIT11)
            host->present = 0;  // card inserted
        else
            host->present = 1;  // card removed
        /* 0.5s needed because of early card detect switch firing */
        mmc_detect_change(host->mmc, msecs_to_jiffies(20)); // wait 20ms for card de-bounce
        nvt_sd_write(REG_IRQTGSRC2, BIT11); // clear GPE11 interrupt status
        return IRQ_HANDLED;
    }
    else
        return IRQ_NONE;    // interrupt was not from this device
#else
    return IRQ_NONE;    // interrupt was not from this device
#endif
}
#endif  // end of NVT_SD_SD2


/*
 * Handle an MMC request
 */
//-------------------------------------------------
// TODO: check write protect pin.
// if write protect, it should return >0 value.
//-------------------------------------------------
static int nvt_sd_get_ro(struct mmc_host *mmc)
{
#ifdef CONFIG_FA93_SD0_WP
    struct nvt_sd_host *host = mmc_priv(mmc);

    // use GPA0 as the write protect pin for SD port 0
    if (host->port == 0)
    {
        if ((nvt_sd_read(REG_GPIOA_PIN) & BIT0) == 0)
        {
            // nvt_sd_debug("nvt_sd_get_ro(): SD port %d is write unprotected (GPA0).\n", host->port);
            return 0;   // write unprotected
        }
        else
        {
            //nvt_sd_debug("nvt_sd_get_ro(): SD port %d is write protected (GPA0).\n", host->port);
            return 1;   // write protected
        }
    }
#endif

    return 0;   // write unprotected

    /*
     * Board doesn't support read only detection; let the mmc core
     * decide what to do.
     */
    //return -ENOSYS;
}


/*
 * Handle an MMC request
 */
static void nvt_sd_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
    struct nvt_sd_host *host = mmc_priv(mmc);

    if (host->port == 0)
    {
        // disable SDIO interrupt first, and then clear interrupt flag
        nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) & (~SDIER_SDIO_IEN));    // Disable SDIO interrupt
        nvt_sd_write(REG_SDISR, SDISR_SDIO_IF);    // write 1 to clear SDIO interrupt flag
        if (enable)
        {
            // enable SDIO interrupt finally
            nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) | SDIER_WKUP_EN);    // Enable Wake-Up interrupt
            nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) | SDIER_SDIO_IEN);   // Enable SDIO interrupt
            //nvt_sd_debug("--> MMC/SDIO: Enable SDIO interrupt for SD port 0.\n");

            // MUST enable SD clock free run to latch SDIO interrupt signel.
            nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) | SDCR_CLK_KEEP);
        }
        else
        {
            nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) & (~SDIER_WKUP_EN)); // Disable Wake-Up interrupt
            //nvt_sd_debug("--> MMC/SDIO: Disable SDIO interrupt for SD port 0.\n");

            nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) & (~SDCR_CLK_KEEP));
        }
    }

#ifdef NVT_SD_SD1
    if (host->port == 1)
    {
        // disable SDIO interrupt first, and then clear interrupt flag
        nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) & (~SDIER_SDIO1_IEN));   // Disable SDIO1 interrupt
        nvt_sd_write(REG_SDISR, SDISR_SDIO1_IF);    // write 1 to clear SDIO1 interrupt flag
        if (enable)
        {
            // enable SDIO interrupt finally
            nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) | SDIER_WKUP_EN);    // Enable Wake-Up interrupt
            nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) | SDIER_SDIO1_IEN);  // Set SDIO1_IEN to 1 to enable SDIO2 interrupt
            //nvt_sd_debug("--> MMC/SDIO: Enable SDIO1 interrupt for SD port 1.\n");

            // MUST enable SD clock free run to latch SDIO interrupt signel.
            nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) | SDCR_CLK_KEEP1);
        }
        else
        {
            nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) & (~SDIER_WKUP_EN)); // Disable Wake-Up interrupt
            //nvt_sd_debug("--> MMC/SDIO: Disable SDIO1 interrupt for SD port 1.\n");

            nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) & (~SDCR_CLK_KEEP1));
        }
    }
#endif

#ifdef NVT_SD_SD2
    if (host->port == 2)
    {
        // disable SDIO interrupt first, and then clear interrupt flag
        nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) & (~SDIER_SDIO1_IEN));   // Disable SDIO1 interrupt
        nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) & (~SDIER_SDIO2_IEN));   // Disable SDIO2 interrupt
        nvt_sd_write(REG_SDISR, SDISR_SDIO2_IF);    // write 1 to clear SDIO2 interrupt flag
        if (enable)
        {
            // enable SDIO interrupt finally
            nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) | SDIER_WKUP_EN);    // Enable Wake-Up interrupt

            // 2013/7/22, set both SDIER_SDIO1_IEN and SDIER_SDIO2_IEN to 1 in order to enable SDIO2 interrupt.
            //      This is a workaround solution for SDIO2 interrupt feature.
            nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) | SDIER_SDIO1_IEN);  // Set SDIO1_IEN to 1 to enable SDIO2 interrupt
            nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) | SDIER_SDIO2_IEN);  // Enable SDIO2 interrupt
            //nvt_sd_debug("--> MMC/SDIO: Enable SDIO2 interrupt for SD port 2.\n");

            // MUST enable SD clock free run to latch SDIO interrupt signel.
            nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) | SDCR_CLK_KEEP2);
        }
        else
        {
            nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) & (~SDIER_WKUP_EN)); // Disable Wake-Up interrupt
            //nvt_sd_debug("--> MMC/SDIO: Disable SDIO2 interrupt for SD port 2.\n");

            nvt_sd_write(REG_SDCR, nvt_sd_read(REG_SDCR) & (~SDCR_CLK_KEEP2));
        }
    }
#endif
}


/*
 * Handle an MMC request
 */
static int nvt_sd_get_cd(struct mmc_host *mmc)
{
    int ret;

    if (down_interruptible(&fmi_sem))
        return -ENOSYS;
    ret = nvt_sd_card_detect(mmc);
    up(&fmi_sem);
    return ret;
}

static const struct mmc_host_ops nvt_sd_ops = {
    .request    = nvt_sd_request,
    .set_ios    = nvt_sd_set_ios,
    .get_ro     = nvt_sd_get_ro,
    .get_cd     = nvt_sd_get_cd,
    .enable_sdio_irq = nvt_sd_enable_sdio_irq,
};

/*
 * Probe for the device
 */
static int __init nvt_sd_probe(struct platform_device *pdev)
{
    struct mmc_host *mmc;
    struct nvt_sd_host *host;
    struct resource *res;
    int ret;

#ifdef NVT_SD_SD1
    struct mmc_host *mmc1;
    struct nvt_sd_host *host1;
#endif

#ifdef NVT_SD_SD2
    struct mmc_host *mmc2;
    struct nvt_sd_host *host2;
#endif

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res)
        return -ENXIO;

    if (!request_mem_region(res->start, res->end - res->start + 1, DRIVER_NAME))
        return -EBUSY;

    mmc = mmc_alloc_host(sizeof(struct nvt_sd_host), &pdev->dev);
    if (!mmc) {
        ret = -ENOMEM;
        dev_dbg(&pdev->dev, "couldn't allocate mmc host\n");
        goto fail6;
    }

    mmc->ops = &nvt_sd_ops;
    mmc->f_min = 300000;

  #if defined (CONFIG_W55FA93_SDHC_24MHZ)
    mmc->f_max = 24000000;
  #elif defined (CONFIG_W55FA93_SDHC_48MHZ)
    mmc->f_max = 48000000;
  #else
    mmc->f_max = 24000000;
  #endif

    mmc->ocr_avail = MMC_VDD_27_28|MMC_VDD_28_29|MMC_VDD_29_30|MMC_VDD_30_31|MMC_VDD_31_32|MMC_VDD_32_33 | MMC_VDD_33_34;
    mmc->caps = 0;

    mmc->max_blk_size  = MCI_MAXBLKSIZE;    // maximum size of one mmc block
    mmc->max_blk_count = MCI_BLKATONCE;     // maximum number of blocks in one req
    mmc->max_req_size  = MCI_BUFSIZE;       // maximum number of bytes in one req
    mmc->max_phys_segs = MCI_BLKATONCE;
    mmc->max_hw_segs   = MCI_BLKATONCE;
    mmc->max_seg_size  = MCI_BUFSIZE;

    host = mmc_priv(mmc);
    sd_host = host;
    host->mmc = mmc;
    host->bus_mode = MMC_BUS_WIDTH_1;
    host->port = 0;     // default SD port to check

#ifdef CONFIG_FA93_SD0_1_BIT_MODE
    mmc->caps |= (MMC_CAP_SD_HIGHSPEED);
    nvt_sd_debug("W55FA93 MMC/SD driver force SD0 downgrade bus width to 1 bit mode.\n");
#else
    mmc->caps |= (MMC_CAP_4_BIT_DATA | MMC_CAP_SD_HIGHSPEED);
#endif

    host->buffer = dma_alloc_coherent(&pdev->dev, MCI_BUFSIZE, &host->physical_address, GFP_KERNEL);
    if (!host->buffer) {
        ret = -ENOMEM;
        dev_err(&pdev->dev, "Can't allocate transmit buffer\n");
        goto fail5;
    }

    /*
     * Get Clock
     */
    host->fmi_clk = clk_get(NULL, "SIC");
    if (IS_ERR(host->fmi_clk)) {
        ret = -ENODEV;
        dev_err(&pdev->dev, "Get clock fail. No SIC clock for SD !\n");
        goto fail2;
    }

    host->sd_clk = clk_get(NULL, "SD");
    if (IS_ERR(host->sd_clk)) {
        ret = -ENODEV;
        dev_err(&pdev->dev, "Get clock fail. No SD clock for SD !\n");
        goto fail2;
    }

    /*
     * Reset hardware
     */
    clk_enable(host->fmi_clk);
    clk_enable(host->sd_clk);
    nvt_sd_disable(host);
    nvt_sd_enable(host);

    /*
     * Allocate the MCI interrupt
     */
    host->irq = platform_get_irq(pdev, 0);
    ret = request_irq(host->irq, nvt_sd_irq, IRQF_SHARED, mmc_hostname(mmc), host);
    if (ret) {
        dev_dbg(&pdev->dev, "request MCI interrupt failed\n");
        goto fail0;
    }

    /* add a thread to check CO, RI, and R2 */
    kernel_thread(sd_event_thread, NULL, 0);

    setup_timer(&host->timer, nvt_sd_timeout_timer, (unsigned long)host);

    platform_set_drvdata(pdev, mmc);

    nvt_sd_card_detect(mmc);

    /*
     * Add host to MMC layer
     */
    ret = mmc_add_host(mmc);
    if (ret != 0)
    {
        printk("mmc_add_host() fail. Return = 0x%x\n", ret);
        goto fail0;
    }

    //------ for SD1
#ifdef NVT_SD_SD1
    mmc1 = mmc_alloc_host(sizeof(struct nvt_sd_host), &pdev->dev);
    if (!mmc1) {
        ret = -ENOMEM;
        dev_dbg(&pdev->dev, "couldn't allocate mmc1 host\n");
        goto fail16;
    }

    mmc1->ops = &nvt_sd_ops;
    mmc1->f_min = 300000;

  #if defined (CONFIG_W55FA93_SDHC_24MHZ)
    mmc1->f_max = 24000000;
  #elif defined (CONFIG_W55FA93_SDHC_48MHZ)
    mmc1->f_max = 48000000;
  #else
    mmc1->f_max = 24000000;
  #endif

    mmc1->ocr_avail = MMC_VDD_27_28|MMC_VDD_28_29|MMC_VDD_29_30|MMC_VDD_30_31|MMC_VDD_31_32|MMC_VDD_32_33 | MMC_VDD_33_34;
    mmc1->caps = 0;

    mmc1->max_blk_size  = MCI_MAXBLKSIZE;    // maximum size of one mmc block
    mmc1->max_blk_count = MCI_BLKATONCE;     // maximum number of blocks in one req
    mmc1->max_req_size  = MCI_BUFSIZE;       // maximum number of bytes in one req
    mmc1->max_phys_segs = MCI_BLKATONCE;
    mmc1->max_hw_segs   = MCI_BLKATONCE;
    mmc1->max_seg_size  = MCI_BUFSIZE;

    host1 = mmc_priv(mmc1);
    sd1_host = host1;
    host1->mmc = mmc1;
    host1->bus_mode = MMC_BUS_WIDTH_1;
    host1->port = 1;

#ifdef CONFIG_FA93_SD1_1_BIT_MODE
    mmc1->caps |= (MMC_CAP_SD_HIGHSPEED);
    nvt_sd_debug("W55FA93 MMC/SD driver force SD1 downgrade bus width to 1 bit mode.\n");
#else
    mmc1->caps |= (MMC_CAP_4_BIT_DATA | MMC_CAP_SD_HIGHSPEED);
#endif

    host1->buffer = dma_alloc_coherent(&pdev->dev, MCI_BUFSIZE, &host1->physical_address, GFP_KERNEL);
    if (!host1->buffer) {
        ret = -ENOMEM;
        dev_err(&pdev->dev, "Can't allocate transmit buffer 1\n");
        goto fail15;
    }

    /*
     * Reset hardware
     */
    nvt_sd_disable(host1);
    nvt_sd_enable(host1);

    /*
     * Allocate the MCI interrupt
     */
    // interrupt handler for SD1 BLKD_IF and SDIO0_IF
    host1->irq = platform_get_irq(pdev, 0);
    ret = request_irq(host1->irq, nvt_sd_irq, IRQF_SHARED, mmc_hostname(mmc1), host1);
    if (ret) {
        dev_dbg(&pdev->dev, "request MCI interrupt failed\n");
        goto fail10;
    }

  #if defined (CONFIG_SD1_CD_GPA3) || defined (CONFIG_SD1_CD_GPB6) || defined (CONFIG_SD1_CD_GPE11)
    // Allocate the GPIO1 interrupt handler. SD1 card detect GPxx bind to GPIO1 interrupt.
    ret = request_irq(IRQ_GPIO1, nvt_sd1_card_detect_irq, IRQF_SHARED, mmc_hostname(mmc1), host1);
    if (ret) {
        dev_dbg(&pdev->dev, "Error: request GPIO1 interrupt failed!\n");
        goto fail10;
    }
  #endif

    /* add a thread to check CO, RI, and R2 */
    kernel_thread(sd1_event_thread, NULL, 0);

    setup_timer(&host1->timer, nvt_sd_timeout_timer, (unsigned long)host1);

    platform_set_drvdata(pdev, mmc1);

    nvt_sd_card_detect(mmc1);

    /*
     * Add host to MMC layer
     */
    ret = mmc_add_host(mmc1);
    if (ret != 0)
    {
        printk("mmc_add_host(mmc1) fail. Return = 0x%x\n", ret);
        goto fail10;
    }
#endif  // end of NVT_SD_SD1

    //------ for SD2
#ifdef NVT_SD_SD2
    mmc2 = mmc_alloc_host(sizeof(struct nvt_sd_host), &pdev->dev);
    if (!mmc2) {
        ret = -ENOMEM;
        dev_dbg(&pdev->dev, "couldn't allocate mmc2 host\n");
        goto fail26;
    }

    mmc2->ops = &nvt_sd_ops;
    mmc2->f_min = 300000;

  #if defined (CONFIG_W55FA93_SDHC_24MHZ)
    mmc2->f_max = 24000000;
  #elif defined (CONFIG_W55FA93_SDHC_48MHZ)
    mmc2->f_max = 48000000;
  #else
    mmc2->f_max = 24000000;
  #endif

    mmc2->ocr_avail = MMC_VDD_27_28|MMC_VDD_28_29|MMC_VDD_29_30|MMC_VDD_30_31|MMC_VDD_31_32|MMC_VDD_32_33 | MMC_VDD_33_34;
    mmc2->caps = 0;

    mmc2->max_blk_size  = MCI_MAXBLKSIZE;    // maximum size of one mmc block
    mmc2->max_blk_count = MCI_BLKATONCE;     // maximum number of blocks in one req
    mmc2->max_req_size  = MCI_BUFSIZE;       // maximum number of bytes in one req
    mmc2->max_phys_segs = MCI_BLKATONCE;
    mmc2->max_hw_segs   = MCI_BLKATONCE;
    mmc2->max_seg_size  = MCI_BUFSIZE;

    host2 = mmc_priv(mmc2);
    sd2_host = host2;
    host2->mmc = mmc2;
    host2->bus_mode = MMC_BUS_WIDTH_1;
    host2->port = 2;

#ifdef CONFIG_FA93_SD2_1_BIT_MODE
    mmc2->caps |= (MMC_CAP_SD_HIGHSPEED);
    nvt_sd_debug("W55FA93 MMC/SD driver force SD2 downgrade bus width to 1 bit mode.\n");
#else
    mmc2->caps |= (MMC_CAP_4_BIT_DATA | MMC_CAP_SD_HIGHSPEED);
#endif

    host2->buffer = dma_alloc_coherent(&pdev->dev, MCI_BUFSIZE, &host2->physical_address, GFP_KERNEL);
    if (!host2->buffer) {
        ret = -ENOMEM;
        dev_err(&pdev->dev, "Can't allocate transmit buffer 2\n");
        goto fail25;
    }

    /*
     * Reset hardware
     */
    nvt_sd_disable(host2);
    nvt_sd_enable(host2);

    /*
     * Allocate the MCI interrupt
     */
    // interrupt handler for SD2 BLKD_IF and SDIO0_IF
    host2->irq = platform_get_irq(pdev, 0);
    ret = request_irq(host2->irq, nvt_sd_irq, IRQF_SHARED, mmc_hostname(mmc2), host2);
    if (ret) {
        dev_dbg(&pdev->dev, "request MCI interrupt failed\n");
        goto fail20;
    }

  #if defined (CONFIG_SD2_CD_GPA3) || defined (CONFIG_SD2_CD_GPB6) || defined (CONFIG_SD2_CD_GPE11) || defined (CONFIG_SD2_CD_GPE11_SW)
    // Allocate the GPIO1 interrupt handler. SD2 card detect GPxx bind to GPIO1 interrupt.
    ret = request_irq(IRQ_GPIO1, nvt_sd2_card_detect_irq, IRQF_SHARED, mmc_hostname(mmc2), host2);
    if (ret) {
        dev_dbg(&pdev->dev, "Error: request GPIO1 interrupt failed!\n");
        goto fail20;
    }
  #endif

    /* add a thread to check CO, RI, and R2 */
    kernel_thread(sd2_event_thread, NULL, 0);

    setup_timer(&host2->timer, nvt_sd_timeout_timer, (unsigned long)host2);

    platform_set_drvdata(pdev, mmc2);

    nvt_sd_card_detect(mmc2);

    /*
     * Add host to MMC layer
     */
    ret = mmc_add_host(mmc2);
    if (ret != 0)
    {
        printk("mmc_add_host(mmc2) fail. Return = 0x%x\n", ret);
        goto fail20;
    }
#endif  // end of NVT_SD_SD2

    nvt_sd_debug("W55FA93 MMC/SD driver (%s) has been initialized successfully!\n", DATE_CODE);

#if defined (CONFIG_W55FA93_SDHC_24MHZ)
    nvt_sd_debug("SDHC card will run under 24MHz clock on SD port.\n");
#elif defined (CONFIG_W55FA93_SDHC_48MHZ)
    nvt_sd_debug("SDHC card will run under 48MHz clock on SD port.\n");
#else
    nvt_sd_debug("SDHC card will run under 24MHz clock on SD port.\n");
#endif

    return 0;

fail0:
    clk_disable(host->sd_clk);
    clk_put(host->sd_clk);
fail2:
    if (host->buffer)
        dma_free_coherent(&pdev->dev, MCI_BUFSIZE, host->buffer, host->physical_address);
fail5:
    mmc_free_host(mmc);
fail6:
    release_mem_region(res->start, res->end - res->start + 1);
    dev_err(&pdev->dev, "probe failed, err %d\n", ret);
    return ret;

#ifdef NVT_SD_SD1
fail10:
    if (host1->buffer)
        dma_free_coherent(&pdev->dev, MCI_BUFSIZE, host1->buffer, host1->physical_address);
fail15:
    mmc_free_host(mmc1);
fail16:
    release_mem_region(res->start, res->end - res->start + 1);
    dev_err(&pdev->dev, "probe 1 failed, err %d\n", ret);
    return ret;
#endif

#ifdef NVT_SD_SD2
fail20:
    if (host2->buffer)
        dma_free_coherent(&pdev->dev, MCI_BUFSIZE, host2->buffer, host2->physical_address);
fail25:
    mmc_free_host(mmc2);
fail26:
    release_mem_region(res->start, res->end - res->start + 1);
    dev_err(&pdev->dev, "probe 2 failed, err %d\n", ret);
    return ret;
#endif
}

/*
 * Remove a device
 */
static int __exit nvt_sd_remove(struct platform_device *pdev)
{
    struct mmc_host *mmc = platform_get_drvdata(pdev);
    struct nvt_sd_host *host;

    if (!mmc)
        return -1;

    host = mmc_priv(mmc);
    if (host->buffer)
        dma_free_coherent(&pdev->dev, MCI_BUFSIZE, host->buffer, host->physical_address);

    nvt_sd_disable(host);
    del_timer_sync(&host->timer);
    mmc_remove_host(mmc);
    free_irq(host->irq, host);

    clk_disable(host->sd_clk);
    clk_put(host->sd_clk);

    mmc_free_host(mmc);
    platform_set_drvdata(pdev, NULL);
    nvt_sd_debug("SIC/SD Removed!\n");

    return 0;
}

#ifdef CONFIG_PM

static int nvt_sd_suspend(struct platform_device *pdev, pm_message_t state)
{
    struct mmc_host *mmc = platform_get_drvdata(pdev);
    struct nvt_sd_host *host = mmc_priv(mmc);
    int ret = 0;

    if (mmc)
        ret = mmc_suspend_host(mmc);

    return ret;
}

static int nvt_sd_resume(struct platform_device *pdev)
{
    struct mmc_host *mmc = platform_get_drvdata(pdev);
    struct nvt_sd_host *host = mmc_priv(mmc);
    int ret = 0;

    if (mmc)
        ret = mmc_resume_host(mmc);

    return ret;
}

#else

#define nvt_sd_suspend   NULL
#define nvt_sd_resume    NULL

#endif  // end of CONFIG_PM

static struct platform_driver nvt_sd_driver = {
    .remove     = __exit_p(nvt_sd_remove),
    .suspend    = nvt_sd_suspend,
    .resume     = nvt_sd_resume,
    .driver     = {
        .name   = DRIVER_NAME,
        .owner  = THIS_MODULE,
    },
};

static int __init nvt_sd_init(void)
{
    int ret;
    ret =  platform_driver_probe(&nvt_sd_driver, nvt_sd_probe);     // for non-hotplug device
    nvt_sd_write(REG_SDIER, nvt_sd_read(REG_SDIER) | SDIER_CD_IEN); // enable Interrupt for card detect

    sd_init_completed = 1;  // nvt_sd_init() completed
    return ret;
}

static void __exit nvt_sd_exit(void)
{
    platform_driver_unregister(&nvt_sd_driver);
}

module_init(nvt_sd_init);
module_exit(nvt_sd_exit);

MODULE_DESCRIPTION("W55FA93 SD Card Interface driver");
MODULE_AUTHOR("HPChen / CJChen");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:w55fa93_sd");
