/* linux/drivers/spi/spi_w55fa93.c
 *
 * Copyright (c) 2009 Nuvoton technology.
 * Wan ZongShun <mcuos.com@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/slab.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <mach/w55fa93_spi.h>
#include <mach/w55fa93_reg.h>

/* usi registers offset */
#define USI_CNT		0x00
#define USI_DIV		0x04
#define USI_SSR		0x08
#define USI_RX0		0x10
#define USI_TX0		0x10

/* usi register bit */
#define BYTEENDIN		(0x01 << 20)
#define ENINT		(0x01 << 17)
#define ENFLG		(0x01 << 16)
#define TXNUM		(0x03 << 8)
#define TXBIT		(0x1F << 3)
#define TXNEG		(0x01 << 2)
#define RXNEG		(0x01 << 1)
#define LSB		(0x01 << 10)
#define SELECTLEV	(0x01 << 2)
#define SELECTPOL	(0x01 << 11)
#define SELECTSLAVE0	0x01
#define SELECTSLAVE1	(0x01 << 1)
#define GOBUSY		0x01

extern unsigned int w55fa93_apb_clock;

struct w55fa93_spi {
	struct spi_bitbang	 bitbang;
	struct completion	 done;
	void __iomem		*regs;
	int			 irq;
	int			 len;
	int			 count;
	int			 tx_num;
	const unsigned char	*tx;
	unsigned char		*rx;
	struct clk		*clk;
	struct resource		*ioarea;
	struct spi_master	*master;
	struct spi_device	*curdev;
	struct device		*dev;
	struct w55fa93_spi_info *pdata;
	spinlock_t		lock;
	struct resource		*res;
};

static inline struct w55fa93_spi *to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}

static void w55fa93_slave_select(struct spi_device *spi, unsigned int ssr)
{
	struct w55fa93_spi *hw = to_hw(spi);
	unsigned int val;
	unsigned int cs = spi->mode & SPI_CS_HIGH ? 1 : 0;
	unsigned int cpol = spi->mode & SPI_CPOL ? 1 : 0;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + USI_SSR);

	if (!cs)
		val &= ~SELECTLEV;
	else
		val |= SELECTLEV;

	if(spi->chip_select == 0)
	{
		if (!ssr)
			val &= ~SELECTSLAVE0;
		else
			val |= SELECTSLAVE0;
	}
	else
	{
		if (!ssr)
			val &= ~SELECTSLAVE1;
		else
			val |= SELECTSLAVE1;
	}

	__raw_writel(val, hw->regs + USI_SSR);

	val = __raw_readl(hw->regs + USI_CNT);

	if (!cpol)
		val &= ~SELECTPOL;
	else
		val |= SELECTPOL;

	__raw_writel(val, hw->regs + USI_CNT);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static void w55fa93_spi_chipsel(struct spi_device *spi, int value)
{	
	switch (value) {
	case BITBANG_CS_INACTIVE:
		w55fa93_slave_select(spi, 0);
		break;

	case BITBANG_CS_ACTIVE:
		w55fa93_slave_select(spi, 1);
		break;
	}
}

static void w55fa93_spi_setup_txnum(struct w55fa93_spi *hw,
							unsigned int txnum)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	hw->tx_num = txnum;

	val = __raw_readl(hw->regs + USI_CNT);

	if (!txnum)
		val &= ~TXNUM;
	else
		val |= txnum << 0x08;

	__raw_writel(val, hw->regs + USI_CNT);

	spin_unlock_irqrestore(&hw->lock, flags);

}

static void w55fa93_spi_setup_txbitlen(struct w55fa93_spi *hw,
							unsigned int txbitlen)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + USI_CNT) & ~TXBIT;

	if(txbitlen == 32)
		txbitlen = 0;

	val |= (txbitlen << 0x03);

	__raw_writel(val, hw->regs + USI_CNT);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static void w55fa93_spi_setup_byte_endin(struct w55fa93_spi *hw,
							unsigned int endin)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + USI_CNT) & ~BYTEENDIN;

	val |= (endin << 20);

	__raw_writel(val, hw->regs + USI_CNT);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static void w55fa93_spi_gobusy(struct w55fa93_spi *hw)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + USI_CNT);

	val |= GOBUSY;

	__raw_writel(val, hw->regs + USI_CNT);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static int w55fa93_spi_setupxfer(struct spi_device *spi,
				 struct spi_transfer *t)
{
	return 0;
}

static unsigned int spi_speed_hz =50000000;
static int w55fa93_spi_setup(struct spi_device *spi)
{
	struct w55fa93_spi *hw = to_hw(spi);
	unsigned long flags;
	int divider;
	
	spin_lock_irqsave(&hw->lock, flags);	
	if(spi_speed_hz != spi->max_speed_hz)
	{	
		spi_speed_hz = spi->max_speed_hz;
		divider = (w55fa93_apb_clock * 1000) / (2 * spi->max_speed_hz) - 1;
		if(divider < 0)
			divider = 0;
		//printk("spi divider %d %d %d\n", w55fa93_apb_clock * 1000, spi->max_speed_hz, divider);
				
		__raw_writel(divider, hw->regs + USI_DIV);
		
	}
	spin_unlock_irqrestore(&hw->lock, flags);
	
	return 0;
}

static inline unsigned int hw_txbyte(struct w55fa93_spi *hw, int count)
{	
	return hw->tx ? hw->tx[count] : 0;
}

static inline unsigned int hw_txword(struct w55fa93_spi *hw, int count)
{
	unsigned int * p32tmp;	

	if(hw->tx == 0)
		return 0;
	else
	{
		p32tmp = (unsigned int *)((unsigned int )(hw->tx) + count);		
		return *p32tmp;
	}		
}

static int w55fa93_spi_txrx(struct spi_device *spi, struct spi_transfer *t)
{
	int i;
	struct w55fa93_spi *hw = to_hw(spi);	

	hw->tx = t->tx_buf;
	hw->rx = t->rx_buf;
	hw->len = t->len;
	hw->count = 0;

	//printk("hw->len %d 0x%x\n", hw->len, hw->rx);
	
	if(hw->len < 4)
	{
		w55fa93_spi_setup_byte_endin(hw, 0);
		w55fa93_spi_setup_txbitlen(hw, 8);
		w55fa93_spi_setup_txnum(hw, 0);
		__raw_writel(hw_txbyte(hw, 0x0), hw->regs + USI_TX0);
	
	}
	else
	{
		w55fa93_spi_setup_byte_endin(hw, 1);
		w55fa93_spi_setup_txbitlen(hw, 32);

		if(hw->len >= 16)
		{
			w55fa93_spi_setup_txnum(hw, 3);
			for(i=0;i<4;i++)
				__raw_writel(hw_txword(hw, i*4), hw->regs + USI_TX0 + i*4);
		}
		else
		{
			w55fa93_spi_setup_txnum(hw, 0);
			__raw_writel(hw_txword(hw, 0x0), hw->regs + USI_TX0);	
		}
	}
	
	w55fa93_spi_gobusy(hw);

	wait_for_completion(&hw->done);

	return hw->count;
}

static irqreturn_t w55fa93_spi_irq(int irq, void *dev)
{
	struct w55fa93_spi *hw = dev;
	unsigned int status;
	unsigned int count = hw->count;
	unsigned int val,i;
	unsigned int * p32tmp;
	
	status = __raw_readl(hw->regs + USI_CNT);
	__raw_writel(status, hw->regs + USI_CNT);

	if (status & ENFLG) {

		val = __raw_readl(hw->regs + USI_CNT) & BYTEENDIN;

		if(val)
		{
			hw->count = hw->count + (hw->tx_num + 1)*4;						

			if (hw->rx)
			{
				p32tmp = (unsigned int *)((unsigned int )(hw->rx) + count);

				for(i=0;i<(hw->tx_num+1);i++)
				{
					*p32tmp = __raw_readl(hw->regs + USI_RX0 + i*4);
					p32tmp++;
				}								
			}

			count = count + (hw->tx_num + 1)*4;			

			if (count < hw->len)
			{
				if((count+16) <= hw->len)
				{
					for(i=0;i<4;i++)
						__raw_writel(hw_txword(hw, (count+i*4)), hw->regs + USI_TX0 + i*4);
				}
				else if((count+4) <= hw->len)
				{
					w55fa93_spi_setup_txnum(hw, 0);
					__raw_writel(hw_txword(hw, count), hw->regs + USI_TX0);
				}
				else
				{
					w55fa93_spi_setup_byte_endin(hw, 0);
					w55fa93_spi_setup_txbitlen(hw, 8);
					w55fa93_spi_setup_txnum(hw, 0);
					__raw_writel(hw_txbyte(hw, count), hw->regs + USI_TX0);
				}
				w55fa93_spi_gobusy(hw);
			}
			else
			{
				complete(&hw->done);
			}
		}
		else
		{		
			hw->count++;

			if (hw->rx)
				hw->rx[count] = __raw_readl(hw->regs + USI_RX0);
			count++;

			if (count < hw->len) {
				__raw_writel(hw_txbyte(hw, count), hw->regs + USI_TX0);
				w55fa93_spi_gobusy(hw);
			} else {
				complete(&hw->done);
			}
		}

		return IRQ_HANDLED;
	}

	complete(&hw->done);
	return IRQ_HANDLED;
}

static void w55fa93_tx_edge(struct w55fa93_spi *hw, unsigned int edge)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + USI_CNT);

	if (edge)
		val |= TXNEG;
	else
		val &= ~TXNEG;
	__raw_writel(val, hw->regs + USI_CNT);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static void w55fa93_rx_edge(struct w55fa93_spi *hw, unsigned int edge)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + USI_CNT);

	if (edge)
		val |= RXNEG;
	else
		val &= ~RXNEG;
	__raw_writel(val, hw->regs + USI_CNT);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static void w55fa93_send_first(struct w55fa93_spi *hw, unsigned int lsb)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + USI_CNT);

	if (lsb)
		val |= LSB;
	else
		val &= ~LSB;
	__raw_writel(val, hw->regs + USI_CNT);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static void w55fa93_set_sleep(struct w55fa93_spi *hw, unsigned int sleep)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + USI_CNT);

	if (sleep)
		val |= (sleep << 12);
	else
		val &= ~(0x0f << 12);
	__raw_writel(val, hw->regs + USI_CNT);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static void w55fa93_enable_int(struct w55fa93_spi *hw)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + USI_CNT);

	val |= ENINT;

	__raw_writel(val, hw->regs + USI_CNT);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static void w55fa93_set_divider(struct w55fa93_spi *hw)
{
	__raw_writel(hw->pdata->divider, hw->regs + USI_DIV);
}

static void w55fa93_init_spi(struct w55fa93_spi *hw)
{		
	clk_enable(hw->clk);
        
       writel(readl(REG_APBIPRST) | SPI0RST, REG_APBIPRST);	//reset spi0
	writel(readl(REG_APBIPRST) & ~SPI0RST, REG_APBIPRST);
	spin_lock_init(&hw->lock);

	w55fa93_tx_edge(hw, hw->pdata->txneg);
	w55fa93_rx_edge(hw, hw->pdata->rxneg);
	w55fa93_send_first(hw, hw->pdata->lsb);
	w55fa93_set_sleep(hw, hw->pdata->sleep);
	w55fa93_spi_setup_txbitlen(hw, hw->pdata->txbitlen);
	w55fa93_spi_setup_txnum(hw, hw->pdata->txnum);
	w55fa93_set_divider(hw);
	w55fa93_enable_int(hw);
}

static int __devinit w55fa93_spi_probe(struct platform_device *pdev)
{
	struct w55fa93_spi *hw;
	struct spi_master *master;
	int err = 0;	

	master = spi_alloc_master(&pdev->dev, sizeof(struct w55fa93_spi));
	if (master == NULL) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}

	hw = spi_master_get_devdata(master);
	memset(hw, 0, sizeof(struct w55fa93_spi));

	hw->master = spi_master_get(master);
	hw->pdata  = pdev->dev.platform_data;
	hw->dev = &pdev->dev;

	if (hw->pdata == NULL) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		err = -ENOENT;
		goto err_pdata;
	}

	platform_set_drvdata(pdev, hw);
	init_completion(&hw->done);

	master->mode_bits          = SPI_MODE_0;
	master->num_chipselect     = hw->pdata->num_cs;
	master->bus_num            = hw->pdata->bus_num;
	hw->bitbang.master         = hw->master;
	hw->bitbang.setup_transfer = w55fa93_spi_setupxfer;
	hw->bitbang.chipselect     = w55fa93_spi_chipsel;
	hw->bitbang.txrx_bufs      = w55fa93_spi_txrx;
	hw->bitbang.master->setup  = w55fa93_spi_setup;
	
	hw->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (hw->res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_pdata;
	}

	hw->ioarea = request_mem_region(hw->res->start,
					resource_size(hw->res), pdev->name);

	if (hw->ioarea == NULL) {
		dev_err(&pdev->dev, "Cannot reserve region\n");
		err = -ENXIO;
		goto err_pdata;
	}

	hw->regs = ioremap(hw->res->start, resource_size(hw->res));
	if (hw->regs == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_iomap;
	}

	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_irq;
	}

	err = request_irq(hw->irq, w55fa93_spi_irq, 0, pdev->name, hw);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto err_irq;
	}

	hw->clk = clk_get(&pdev->dev, "MS0");
	if (IS_ERR(hw->clk)) {
		dev_err(&pdev->dev, "No clock for device\n");
		err = PTR_ERR(hw->clk);
		goto err_clk;
	}
	
	writel((readl(REG_GPDFUN) & ~(0xF3000000)) |0xA2000000, REG_GPDFUN);	// configuer pin function
#ifdef CONFIG_FA93_SPI_CS0_ENABLE
	writel((readl(REG_GPDFUN) & ~(0x0C000000)) |0x08000000, REG_GPDFUN);
#endif
#ifdef CONFIG_FA93_SPI_CS1_ENABLE
	writel((readl(REG_GPAFUN) & ~(0x00000C00)) |0x00000800, REG_GPAFUN);
#endif
	w55fa93_init_spi(hw);

	err = spi_bitbang_start(&hw->bitbang);
	if (err) {
		dev_err(&pdev->dev, "Failed to register SPI master\n");
		goto err_register;
	}

	return 0;

err_register:	
	clk_disable(hw->clk);
err_clk:
	free_irq(hw->irq, hw);
err_irq:
	iounmap(hw->regs);
err_iomap:
	release_mem_region(hw->res->start, resource_size(hw->res));
	kfree(hw->ioarea);
err_pdata:
	spi_master_put(hw->master);;

err_nomem:
	return err;
}

static int __devexit w55fa93_spi_remove(struct platform_device *dev)
{
	struct w55fa93_spi *hw = platform_get_drvdata(dev);	

	free_irq(hw->irq, hw);

	platform_set_drvdata(dev, NULL);

	spi_unregister_master(hw->master);
	
	clk_disable(hw->clk);

	iounmap(hw->regs);

	release_mem_region(hw->res->start, resource_size(hw->res));
	kfree(hw->ioarea);

	spi_master_put(hw->master);
	return 0;
}

static struct platform_driver w55fa93_spi_driver = {
	.probe		= w55fa93_spi_probe,
	.remove		= __devexit_p(w55fa93_spi_remove),
	.driver		= {
		.name	= "w55fa93-spi",
		.owner	= THIS_MODULE,
	},
};

static int __init w55fa93_spi_init(void)
{
	return platform_driver_register(&w55fa93_spi_driver);
}

static void __exit w55fa93_spi_exit(void)
{
	platform_driver_unregister(&w55fa93_spi_driver);
}

module_init(w55fa93_spi_init);
module_exit(w55fa93_spi_exit);

MODULE_DESCRIPTION("w55fa93 spi driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:w55fa93-spi");
