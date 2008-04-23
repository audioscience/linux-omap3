/*
 * linux/arch/arm/plat-omap/mcbsp.c
 *
 * Copyright (C) 2004 Nokia Corporation
 * Author: Samuel Ortiz <samuel.ortiz@nokia.com>
 * Added DMA chaining support for 2430/34XX
 * by chandra shekhar <x0044955@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Multichannel mode not supported.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/irq.h>

#include <asm/arch/dma.h>
#include <asm/arch/mux.h>
#include <asm/arch/irqs.h>
#include <asm/arch/dsp_common.h>
#include <asm/arch/mcbsp.h>

#ifdef CONFIG_MCBSP_DEBUG
#define DBG(x...)	printk(x)
#else
#define DBG(x...)			do { } while (0)
#endif

struct omap_mcbsp {
	u32				io_base;
	u8				id;
	u8				free;
	omap_mcbsp_word_length		rx_word_length;
	omap_mcbsp_word_length		tx_word_length;

	omap_mcbsp_io_type_t		io_type; /* IRQ or poll */
	/* IRQ based TX/RX */
	int				rx_irq;
	int				tx_irq;

	/* DMA stuff */
	u8				dma_rx_sync;
	short				dma_rx_lch;
	u8				dma_tx_sync;
	short				dma_tx_lch;

	/* Completion queues */
	struct completion		tx_irq_completion;
	struct completion		rx_irq_completion;
	struct completion		tx_dma_completion;
	struct completion		rx_dma_completion;

	/* Protect the field .free, while checking if the mcbsp is in use */
	spinlock_t			lock;
	u32 				phy_base;

	u8				auto_reset;	/* Auto Reset */
	u8				txskip_alt;	/* Tx skip flags */
	u8				rxskip_alt;	/* Rx skip flags */

	void				*rx_cb_arg;
	void				*tx_cb_arg;

	omap_mcbsp_dma_cb		rx_callback;
	omap_mcbsp_dma_cb		tx_callback;

	int				rx_dma_chain_state;
	int				tx_dma_chain_state;
	int				interface_mode;
	int				srg_enabled;
};

static struct omap_mcbsp mcbsp[OMAP_MAX_MCBSP_COUNT];
#ifdef CONFIG_ARCH_OMAP1
static struct clk *mcbsp_dsp_ck;
static struct clk *mcbsp_api_ck;
static struct clk *mcbsp_dspxor_ck;
#endif
#ifdef CONFIG_ARCH_OMAP2420
static struct clk *mcbsp1_ick;
static struct clk *mcbsp1_fck;
static struct clk *mcbsp2_ick;
static struct clk *mcbsp2_fck;
#endif
#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP34XX)
static char omap_mcbsp_ick[OMAP_MAX_MCBSP_COUNT][15] = {"mcbsp1_ick\0",
							"mcbsp2_ick\0",
							"mcbsp3_ick\0",
							"mcbsp4_ick\0",
							"mcbsp5_ick\0"
							};

static char omap_mcbsp_fck[OMAP_MAX_MCBSP_COUNT][15] = {"mcbsp1_fck\0",
							"mcbsp2_fck\0",
							"mcbsp3_fck\0",
							"mcbsp4_fck\0",
							"mcbsp5_fck\0"
							};

static struct omap_mcbsp_clocks {
			struct clk *ick;
			struct clk *fck;
			} omap_mcbsp_clk[OMAP_MAX_MCBSP_COUNT];
#endif

static void omap_mcbsp_dump_reg(u8 id)
{
	DBG("**** MCBSP%d regs ****\n", mcbsp[id].id);
#if !defined(CONFIG_ARCH_OMAP2430) && !defined(CONFIG_ARCH_OMAP34XX)
	DBG("DRR2:  0x%04x\n", OMAP_MCBSP_READ(mcbsp[id].io_base, DRR2));
	DBG("DRR1:  0x%04x\n", OMAP_MCBSP_READ(mcbsp[id].io_base, DRR1));
	DBG("DXR2:  0x%04x\n", OMAP_MCBSP_READ(mcbsp[id].io_base, DXR2));
	DBG("DXR1:  0x%04x\n", OMAP_MCBSP_READ(mcbsp[id].io_base, DXR1));
#endif
	DBG("SPCR2: 0x%04x\n", OMAP_MCBSP_READ(mcbsp[id].io_base, SPCR2));
	DBG("SPCR1: 0x%04x\n", OMAP_MCBSP_READ(mcbsp[id].io_base, SPCR1));
	DBG("RCR2:  0x%04x\n", OMAP_MCBSP_READ(mcbsp[id].io_base, RCR2));
	DBG("RCR1:  0x%04x\n", OMAP_MCBSP_READ(mcbsp[id].io_base, RCR1));
	DBG("XCR2:  0x%04x\n", OMAP_MCBSP_READ(mcbsp[id].io_base, XCR2));
	DBG("XCR1:  0x%04x\n", OMAP_MCBSP_READ(mcbsp[id].io_base, XCR1));
	DBG("SRGR2: 0x%04x\n", OMAP_MCBSP_READ(mcbsp[id].io_base, SRGR2));
	DBG("SRGR1: 0x%04x\n", OMAP_MCBSP_READ(mcbsp[id].io_base, SRGR1));
	DBG("PCR0:  0x%04x\n", OMAP_MCBSP_READ(mcbsp[id].io_base, PCR0));
#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP34XX)
	DBG("XCCR:  0x%04x\n", OMAP_MCBSP_READ(mcbsp[id].io_base, XCCR));
	DBG("RCCR:  0x%04x\n", OMAP_MCBSP_READ(mcbsp[id].io_base, RCCR));
#endif
	DBG("***********************\n");
}

static irqreturn_t omap_mcbsp_tx_irq_handler(int irq, void *dev_id)
{
	struct omap_mcbsp *mcbsp_tx = dev_id;

	DBG("TX IRQ callback : 0x%x\n",
	    OMAP_MCBSP_READ(mcbsp_tx->io_base, SPCR2));

	complete(&mcbsp_tx->tx_irq_completion);

	return IRQ_HANDLED;
}

static irqreturn_t omap_mcbsp_rx_irq_handler(int irq, void *dev_id)
{
	struct omap_mcbsp *mcbsp_rx = dev_id;

	DBG("RX IRQ callback : 0x%x\n",
	    OMAP_MCBSP_READ(mcbsp_rx->io_base, SPCR2));

	complete(&mcbsp_rx->rx_irq_completion);

	return IRQ_HANDLED;
}
#if !defined(CONFIG_ARCH_OMAP2430) && !defined(CONFIG_ARCH_OMAP34XX)

static void omap_mcbsp_tx_dma_callback(int lch, u16 ch_status, void *data)
{
	struct omap_mcbsp *mcbsp_dma_tx = data;

	DBG("TX DMA callback : 0x%x\n",
	    OMAP_MCBSP_READ(mcbsp_dma_tx->io_base, SPCR2));

	/* We can free the channels */
	omap_free_dma(mcbsp_dma_tx->dma_tx_lch);
	mcbsp_dma_tx->dma_tx_lch = -1;

	complete(&mcbsp_dma_tx->tx_dma_completion);
}

static void omap_mcbsp_rx_dma_callback(int lch, u16 ch_status, void *data)
{
	struct omap_mcbsp *mcbsp_dma_rx = data;

	DBG("RX DMA callback : 0x%x\n",
	    OMAP_MCBSP_READ(mcbsp_dma_rx->io_base, SPCR2));

	/* We can free the channels */
	omap_free_dma(mcbsp_dma_rx->dma_rx_lch);
	mcbsp_dma_rx->dma_rx_lch = -1;

	complete(&mcbsp_dma_rx->rx_dma_completion);
}

#else

static void omap_mcbsp_rx_dma_callback(int chainid, u16 ch_status, void *data)
{
	u32 id;
	u32 io_base;

	id = omap_get_mcbspid[chainid];
	io_base = mcbsp[id].io_base;

	/* If we are at the last transfer, Shut down the reciever */
	if ((mcbsp[id].auto_reset & OMAP_MCBSP_AUTO_RRST)
		&& (omap_dma_chain_status(chainid) == OMAP_DMA_CHAIN_INACTIVE))
			OMAP_MCBSP_WRITE(io_base, SPCR1,
				OMAP_MCBSP_READ(io_base, SPCR1) & (~RRST));

	if (mcbsp[id].rx_callback != NULL)
		mcbsp[id].rx_callback(ch_status, data);

}

static void omap_mcbsp_tx_dma_callback(int chainid, u16 ch_status, void *data)
{
	u32 id;
	u32 io_base;

	id = omap_get_mcbspid[chainid];
	io_base = mcbsp[id].io_base;

	/* If we are at the last transfer, Shut down the Transmitter */
	if ((mcbsp[id].auto_reset & OMAP_MCBSP_AUTO_XRST)
		&& (omap_dma_chain_status(chainid) == OMAP_DMA_CHAIN_INACTIVE))
			OMAP_MCBSP_WRITE(io_base, SPCR2,
				OMAP_MCBSP_READ(io_base, SPCR2) & (~XRST));
	if (mcbsp[id].tx_callback != NULL)
		mcbsp[id].tx_callback(ch_status, data);
}
#endif

/*
 * omap_mcbsp_config simply write a config to the
 * appropriate McBSP.
 * You either call this function or set the McBSP registers
 * by yourself before calling omap_mcbsp_start().
 */
void omap_mcbsp_config(unsigned int id, const struct omap_mcbsp_reg_cfg *config)
{
	u32 io_base = mcbsp[id].io_base;

	DBG("OMAP-McBSP: McBSP%d  io_base: 0x%8x\n", id + 1, io_base);

	/* We write the given config */
	OMAP_MCBSP_WRITE(io_base, SPCR2, config->spcr2);
	OMAP_MCBSP_WRITE(io_base, SPCR1, config->spcr1);
	OMAP_MCBSP_WRITE(io_base, RCR2, config->rcr2);
	OMAP_MCBSP_WRITE(io_base, RCR1, config->rcr1);
	OMAP_MCBSP_WRITE(io_base, XCR2, config->xcr2);
	OMAP_MCBSP_WRITE(io_base, XCR1, config->xcr1);
	OMAP_MCBSP_WRITE(io_base, SRGR2, config->srgr2);
	OMAP_MCBSP_WRITE(io_base, SRGR1, config->srgr1);
	OMAP_MCBSP_WRITE(io_base, MCR2, config->mcr2);
	OMAP_MCBSP_WRITE(io_base, MCR1, config->mcr1);
	OMAP_MCBSP_WRITE(io_base, PCR0, config->pcr0);
#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP34XX)
	OMAP_MCBSP_WRITE(io_base, RCCR, config->rccr);
	OMAP_MCBSP_WRITE(io_base, XCCR, config->xccr);
#endif
}
EXPORT_SYMBOL(omap_mcbsp_config);

static int omap_mcbsp_check(unsigned int id)
{
	if (cpu_is_omap730()) {
		if (id > OMAP_MAX_MCBSP_COUNT - 1) {
		       printk(KERN_ERR "OMAP-McBSP: McBSP%d doesn't exist\n",
				id + 1);
		       return -1;
		}
		return 0;
	} else {
		if (id > OMAP_MAX_MCBSP_COUNT) {
			printk(KERN_ERR "OMAP-McBSP: McBSP%d doesn't exist\n",
				id + 1);
			return -1;
		}
		return 0;
	}

	return -1;
}

#ifdef CONFIG_ARCH_OMAP1
static void omap_mcbsp_dsp_request(void)
{
	if (cpu_is_omap15xx() || cpu_is_omap16xx()) {
		int ret;

		ret = omap_dsp_request_mem();
		if (ret < 0) {
			printk(KERN_ERR "Could not get dsp memory: %i\n", ret);
			return;
		}

		clk_enable(mcbsp_dsp_ck);
		clk_enable(mcbsp_api_ck);

		/* enable 12MHz clock to mcbsp 1 & 3 */
		clk_enable(mcbsp_dspxor_ck);

		/*
		 * DSP external peripheral reset
		 * FIXME: This should be moved to dsp code
		 */
		__raw_writew(__raw_readw(DSP_RSTCT2) | 1 | 1 << 1,
			     DSP_RSTCT2);
	}
}

static void omap_mcbsp_dsp_free(void)
{
	if (cpu_is_omap15xx() || cpu_is_omap16xx()) {
		omap_dsp_release_mem();
		clk_disable(mcbsp_dspxor_ck);
		clk_disable(mcbsp_dsp_ck);
		clk_disable(mcbsp_api_ck);
	}
}
#endif

#ifdef CONFIG_ARCH_OMAP2420
static void omap2_mcbsp2_mux_setup(void)
{
	if (cpu_is_omap2420()) {
		omap_cfg_reg(Y15_24XX_MCBSP2_CLKX);
		omap_cfg_reg(R14_24XX_MCBSP2_FSX);
		omap_cfg_reg(W15_24XX_MCBSP2_DR);
		omap_cfg_reg(V15_24XX_MCBSP2_DX);
		omap_cfg_reg(V14_24XX_GPIO117);
	}
	/*
	 * Need to add MUX settings for OMAP 2430 SDP
	 */
}
#endif

/*
 * We can choose between IRQ based or polled IO.
 * This needs to be called before omap_mcbsp_request().
 */
int omap_mcbsp_set_io_type(unsigned int id, omap_mcbsp_io_type_t io_type)
{
	if (omap_mcbsp_check(id) < 0)
		return -EINVAL;

	spin_lock(&mcbsp[id].lock);

	if (!mcbsp[id].free) {
		printk(KERN_ERR "OMAP-McBSP: McBSP%d is currently in use\n",
			id + 1);
		spin_unlock(&mcbsp[id].lock);
		return -EINVAL;
	}

	mcbsp[id].io_type = io_type;

	spin_unlock(&mcbsp[id].lock);

	return 0;
}
EXPORT_SYMBOL(omap_mcbsp_set_io_type);

int omap_mcbsp_request(unsigned int id)
{
	int err;

	if (omap_mcbsp_check(id) < 0)
		return -EINVAL;

#ifdef CONFIG_ARCH_OMAP1
	/*
	 * On 1510, 1610 and 1710, McBSP1 and McBSP3
	 * are DSP public peripherals.
	 */
	if (id == OMAP_MCBSP1 || id == OMAP_MCBSP3)
		omap_mcbsp_dsp_request();
#endif

#ifdef CONFIG_ARCH_OMAP2420
	if (cpu_is_omap2420()) {
		if (id == OMAP_MCBSP1) {
			clk_enable(mcbsp1_ick);
			clk_enable(mcbsp1_fck);
		} else {
			clk_enable(mcbsp2_ick);
			clk_enable(mcbsp2_fck);
		}
	}
#endif
#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP34XX)
	if (cpu_is_omap2430() || cpu_is_omap34xx()) {
		clk_enable(omap_mcbsp_clk[id].ick);
		clk_enable(omap_mcbsp_clk[id].fck);

	}
#endif

	spin_lock(&mcbsp[id].lock);
	if (!mcbsp[id].free) {
		printk(KERN_ERR "OMAP-McBSP: McBSP%d is currently in use\n",
			id + 1);
		spin_unlock(&mcbsp[id].lock);
		return -1;
	}

	mcbsp[id].free = 0;
	mcbsp[id].dma_rx_lch = -1;
	mcbsp[id].dma_tx_lch = -1;
	spin_unlock(&mcbsp[id].lock);

	if (mcbsp[id].io_type == OMAP_MCBSP_IRQ_IO) {
		/* We need to get IRQs here */
		err = request_irq(mcbsp[id].tx_irq, omap_mcbsp_tx_irq_handler,
					0, "McBSP", (void *) (&mcbsp[id]));
		if (err != 0) {
			printk(KERN_ERR "OMAP-McBSP: Unable to "
					"request TX IRQ %d for McBSP%d\n",
					mcbsp[id].tx_irq, mcbsp[id].id);
			return err;
		}

		init_completion(&(mcbsp[id].tx_irq_completion));

		err = request_irq(mcbsp[id].rx_irq, omap_mcbsp_rx_irq_handler,
					0, "McBSP", (void *) (&mcbsp[id]));
		if (err != 0) {
			printk(KERN_ERR "OMAP-McBSP: Unable to "
					"request RX IRQ %d for McBSP%d\n",
					mcbsp[id].rx_irq, mcbsp[id].id);
			free_irq(mcbsp[id].tx_irq, (void *) (&mcbsp[id]));
			return err;
		}

		init_completion(&(mcbsp[id].rx_irq_completion));
	}
	return 0;
}
EXPORT_SYMBOL(omap_mcbsp_request);

void omap_mcbsp_free(unsigned int id)
{
	if (omap_mcbsp_check(id) < 0)
		return;

#ifdef CONFIG_ARCH_OMAP1
	if (cpu_class_is_omap1()) {
		if (id == OMAP_MCBSP1 || id == OMAP_MCBSP3)
			omap_mcbsp_dsp_free();
	}
#endif

#ifdef CONFIG_ARCH_OMAP2420
	if (cpu_is_omap2420()) {
		if (id == OMAP_MCBSP1) {
			clk_disable(mcbsp1_ick);
			clk_disable(mcbsp1_fck);
		} else {
			clk_disable(mcbsp2_ick);
			clk_disable(mcbsp2_fck);
		}
	}
#endif
#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP34XX)
	if (mcbsp[id].dma_rx_lch != -1) {
		omap_free_dma_chain(mcbsp[id].dma_rx_lch);
		omap_get_mcbspid[mcbsp[id].dma_rx_lch] = -1;
	}
	if (mcbsp[id].dma_tx_lch != -1) {
		omap_free_dma_chain(mcbsp[id].dma_tx_lch);
		omap_get_mcbspid[mcbsp[id].dma_tx_lch] = -1;
	}
	if (cpu_is_omap2430() || cpu_is_omap34xx()) {
		clk_disable(omap_mcbsp_clk[id].ick);
		clk_disable(omap_mcbsp_clk[id].fck);
	}
#endif

	spin_lock(&mcbsp[id].lock);
	if (mcbsp[id].free) {
		printk(KERN_ERR "OMAP-McBSP: McBSP%d was not reserved\n",
			id + 1);
		spin_unlock(&mcbsp[id].lock);
		return;
	}

	mcbsp[id].free = 1;
	spin_unlock(&mcbsp[id].lock);

	if (mcbsp[id].io_type == OMAP_MCBSP_IRQ_IO) {
		/* Free IRQs */
		free_irq(mcbsp[id].rx_irq, (void *) (&mcbsp[id]));
		free_irq(mcbsp[id].tx_irq, (void *) (&mcbsp[id]));
	}
}
EXPORT_SYMBOL(omap_mcbsp_free);

/*
 * Here we start the McBSP, by enabling the sample
 * generator, both transmitter and receivers,
 * and the frame sync.
 */
void omap_mcbsp_start(unsigned int id)
{
	u32 io_base;
	u16 w;

	if (omap_mcbsp_check(id) < 0)
		return;

	io_base = mcbsp[id].io_base;

	mcbsp[id].rx_word_length = (OMAP_MCBSP_READ(io_base, RCR1) >> 5) & 0x7;
	mcbsp[id].tx_word_length = (OMAP_MCBSP_READ(io_base, XCR1) >> 5) & 0x7;

	/* Start the sample generator */
	w = OMAP_MCBSP_READ(io_base, SPCR2);
	OMAP_MCBSP_WRITE(io_base, SPCR2, w | (1 << 6));

	/* Enable transmitter and receiver */
	w = OMAP_MCBSP_READ(io_base, SPCR2);
	OMAP_MCBSP_WRITE(io_base, SPCR2, w | 1);

	w = OMAP_MCBSP_READ(io_base, SPCR1);
	OMAP_MCBSP_WRITE(io_base, SPCR1, w | 1);

	udelay(100);

	/* Start frame sync */
	w = OMAP_MCBSP_READ(io_base, SPCR2);
	OMAP_MCBSP_WRITE(io_base, SPCR2, w | (1 << 7));

	/* Dump McBSP Regs */
	omap_mcbsp_dump_reg(id);
}
EXPORT_SYMBOL(omap_mcbsp_start);

void omap_mcbsp_stop(unsigned int id)
{
	u32 io_base;
	u16 w;

	if (omap_mcbsp_check(id) < 0)
		return;

	io_base = mcbsp[id].io_base;

	/* Reset transmitter */
	w = OMAP_MCBSP_READ(io_base, SPCR2);
	OMAP_MCBSP_WRITE(io_base, SPCR2, w & ~(1));

	/* Reset receiver */
	w = OMAP_MCBSP_READ(io_base, SPCR1);
	OMAP_MCBSP_WRITE(io_base, SPCR1, w & ~(1));

	/* Reset the sample rate generator */
	w = OMAP_MCBSP_READ(io_base, SPCR2);
	OMAP_MCBSP_WRITE(io_base, SPCR2, w & ~(1 << 6));
}
EXPORT_SYMBOL(omap_mcbsp_stop);

#if !defined(CONFIG_ARCH_OMAP2430) && !defined(CONFIG_ARCH_OMAP34XX)
/* polled mcbsp i/o operations */
int omap_mcbsp_pollwrite(unsigned int id, u16 buf)
{
	u32 base = mcbsp[id].io_base;
	writew(buf, base + OMAP_MCBSP_REG_DXR1);
	/* if frame sync error - clear the error */
	if (readw(base + OMAP_MCBSP_REG_SPCR2) & XSYNC_ERR) {
		/* clear error */
		writew(readw(base + OMAP_MCBSP_REG_SPCR2) & (~XSYNC_ERR),
		       base + OMAP_MCBSP_REG_SPCR2);
		/* resend */
		return -1;
	} else {
		/* wait for transmit confirmation */
		int attemps = 0;
		while (!(readw(base + OMAP_MCBSP_REG_SPCR2) & XRDY)) {
			if (attemps++ > 1000) {
				writew(readw(base + OMAP_MCBSP_REG_SPCR2) &
				       (~XRST),
				       base + OMAP_MCBSP_REG_SPCR2);
				udelay(10);
				writew(readw(base + OMAP_MCBSP_REG_SPCR2) |
				       (XRST),
				       base + OMAP_MCBSP_REG_SPCR2);
				udelay(10);
				printk(KERN_ERR
				       " Could not write to McBSP Register\n");
				return -2;
			}
		}
	}

	return 0;
}
EXPORT_SYMBOL(omap_mcbsp_pollwrite);

int omap_mcbsp_pollread(unsigned int id, u16 *buf)
{
	u32 base = mcbsp[id].io_base;
	/* if frame sync error - clear the error */
	if (readw(base + OMAP_MCBSP_REG_SPCR1) & RSYNC_ERR) {
		/* clear error */
		writew(readw(base + OMAP_MCBSP_REG_SPCR1) & (~RSYNC_ERR),
		       base + OMAP_MCBSP_REG_SPCR1);
		/* resend */
		return -1;
	} else {
		/* wait for recieve confirmation */
		int attemps = 0;
		while (!(readw(base + OMAP_MCBSP_REG_SPCR1) & RRDY)) {
			if (attemps++ > 1000) {
				writew(readw(base + OMAP_MCBSP_REG_SPCR1) &
				       (~RRST),
				       base + OMAP_MCBSP_REG_SPCR1);
				udelay(10);
				writew(readw(base + OMAP_MCBSP_REG_SPCR1) |
				       (RRST),
				       base + OMAP_MCBSP_REG_SPCR1);
				udelay(10);
				printk(KERN_ERR
				       " Could not read from McBSP Register\n");
				return -2;
			}
		}
	}
	*buf = readw(base + OMAP_MCBSP_REG_DRR1);

	return 0;
}
EXPORT_SYMBOL(omap_mcbsp_pollread);

/*
 * IRQ based word transmission.
 */
void omap_mcbsp_xmit_word(unsigned int id, u32 word)
{
	u32 io_base;
	omap_mcbsp_word_length word_length = mcbsp[id].tx_word_length;

	if (omap_mcbsp_check(id) < 0)
		return;

	io_base = mcbsp[id].io_base;

	wait_for_completion(&(mcbsp[id].tx_irq_completion));

	if (word_length > OMAP_MCBSP_WORD_16)
		OMAP_MCBSP_WRITE(io_base, DXR2, word >> 16);
	OMAP_MCBSP_WRITE(io_base, DXR1, word & 0xffff);
}
EXPORT_SYMBOL(omap_mcbsp_xmit_word);

u32 omap_mcbsp_recv_word(unsigned int id)
{
	u32 io_base;
	u16 word_lsb, word_msb = 0;
	omap_mcbsp_word_length word_length = mcbsp[id].rx_word_length;

	if (omap_mcbsp_check(id) < 0)
		return -EINVAL;

	io_base = mcbsp[id].io_base;

	wait_for_completion(&(mcbsp[id].rx_irq_completion));

	if (word_length > OMAP_MCBSP_WORD_16)
		word_msb = OMAP_MCBSP_READ(io_base, DRR2);
	word_lsb = OMAP_MCBSP_READ(io_base, DRR1);

	return (word_lsb | (word_msb << 16));
}
EXPORT_SYMBOL(omap_mcbsp_recv_word);

int omap_mcbsp_spi_master_xmit_word_poll(unsigned int id, u32 word)
{
	u32 io_base = mcbsp[id].io_base;
	omap_mcbsp_word_length tx_word_length = mcbsp[id].tx_word_length;
	omap_mcbsp_word_length rx_word_length = mcbsp[id].rx_word_length;
	u16 spcr2, spcr1, attempts = 0, word_lsb, word_msb = 0;

	if (tx_word_length != rx_word_length)
		return -EINVAL;

	/* First we wait for the transmitter to be ready */
	spcr2 = OMAP_MCBSP_READ(io_base, SPCR2);
	while (!(spcr2 & XRDY)) {
		spcr2 = OMAP_MCBSP_READ(io_base, SPCR2);
		if (attempts++ > 1000) {
			/* We must reset the transmitter */
			OMAP_MCBSP_WRITE(io_base, SPCR2, spcr2 & (~XRST));
			udelay(10);
			OMAP_MCBSP_WRITE(io_base, SPCR2, spcr2 | XRST);
			udelay(10);
			printk(KERN_ERR "McBSP transmitter not ready\n");
			return -EAGAIN;
		}
	}

	/* Now we can push the data */
	if (tx_word_length > OMAP_MCBSP_WORD_16)
		OMAP_MCBSP_WRITE(io_base, DXR2, word >> 16);
	OMAP_MCBSP_WRITE(io_base, DXR1, word & 0xffff);

	/* We wait for the receiver to be ready */
	spcr1 = OMAP_MCBSP_READ(io_base, SPCR1);
	while (!(spcr1 & RRDY)) {
		spcr1 = OMAP_MCBSP_READ(io_base, SPCR1);
		if (attempts++ > 1000) {
			/* We must reset the receiver */
			OMAP_MCBSP_WRITE(io_base, SPCR1, spcr1 & (~RRST));
			udelay(10);
			OMAP_MCBSP_WRITE(io_base, SPCR1, spcr1 | RRST);
			udelay(10);
			printk(KERN_ERR "McBSP receiver not ready\n");
			return -EAGAIN;
		}
	}

	/* Receiver is ready, let's read the dummy data */
	if (rx_word_length > OMAP_MCBSP_WORD_16)
		word_msb = OMAP_MCBSP_READ(io_base, DRR2);
	word_lsb = OMAP_MCBSP_READ(io_base, DRR1);

	return 0;
}
EXPORT_SYMBOL(omap_mcbsp_spi_master_xmit_word_poll);

int omap_mcbsp_spi_master_recv_word_poll(unsigned int id, u32 *word)
{
	u32 io_base = mcbsp[id].io_base, clock_word = 0;
	omap_mcbsp_word_length tx_word_length = mcbsp[id].tx_word_length;
	omap_mcbsp_word_length rx_word_length = mcbsp[id].rx_word_length;
	u16 spcr2, spcr1, attempts = 0, word_lsb, word_msb = 0;

	if (tx_word_length != rx_word_length)
		return -EINVAL;

	/* First we wait for the transmitter to be ready */
	spcr2 = OMAP_MCBSP_READ(io_base, SPCR2);
	while (!(spcr2 & XRDY)) {
		spcr2 = OMAP_MCBSP_READ(io_base, SPCR2);
		if (attempts++ > 1000) {
			/* We must reset the transmitter */
			OMAP_MCBSP_WRITE(io_base, SPCR2, spcr2 & (~XRST));
			udelay(10);
			OMAP_MCBSP_WRITE(io_base, SPCR2, spcr2 | XRST);
			udelay(10);
			printk(KERN_ERR "McBSP transmitter not ready\n");
			return -EAGAIN;
		}
	}

	/* We first need to enable the bus clock */
	if (tx_word_length > OMAP_MCBSP_WORD_16)
		OMAP_MCBSP_WRITE(io_base, DXR2, clock_word >> 16);
	OMAP_MCBSP_WRITE(io_base, DXR1, clock_word & 0xffff);

	/* We wait for the receiver to be ready */
	spcr1 = OMAP_MCBSP_READ(io_base, SPCR1);
	while (!(spcr1 & RRDY)) {
		spcr1 = OMAP_MCBSP_READ(io_base, SPCR1);
		if (attempts++ > 1000) {
			/* We must reset the receiver */
			OMAP_MCBSP_WRITE(io_base, SPCR1, spcr1 & (~RRST));
			udelay(10);
			OMAP_MCBSP_WRITE(io_base, SPCR1, spcr1 | RRST);
			udelay(10);
			printk(KERN_ERR "McBSP receiver not ready\n");
			return -EAGAIN;
		}
	}

	/* Receiver is ready, there is something for us */
	if (rx_word_length > OMAP_MCBSP_WORD_16)
		word_msb = OMAP_MCBSP_READ(io_base, DRR2);
	word_lsb = OMAP_MCBSP_READ(io_base, DRR1);

	word[0] = (word_lsb | (word_msb << 16));

	return 0;
}
EXPORT_SYMBOL(omap_mcbsp_spi_master_recv_word_poll);

/*
 * Simple DMA based buffer rx/tx routines.
 * Nothing fancy, just a single buffer tx/rx through DMA.
 * The DMA resources are released once the transfer is done.
 * For anything fancier, you should use your own customized DMA
 * routines and callbacks.
 */
int omap_mcbsp_xmit_buffer(unsigned int id, dma_addr_t buffer,
				unsigned int length)
{
	int dma_tx_ch;
	int src_port = 0;
	int dest_port = 0;
	int sync_dev = 0;

	if (omap_mcbsp_check(id) < 0)
		return -EINVAL;

	if (omap_request_dma(mcbsp[id].dma_tx_sync, "McBSP TX",
				omap_mcbsp_tx_dma_callback,
				&mcbsp[id],
				&dma_tx_ch)) {
		printk(KERN_ERR "OMAP-McBSP: Unable to request DMA channel for"
				" McBSP%d TX. Trying IRQ based TX\n", id + 1);
		return -EAGAIN;
	}
	mcbsp[id].dma_tx_lch = dma_tx_ch;

	DBG("TX DMA on channel %d\n", dma_tx_ch);

	init_completion(&(mcbsp[id].tx_dma_completion));

	if (cpu_class_is_omap1()) {
		src_port = OMAP_DMA_PORT_TIPB;
		dest_port = OMAP_DMA_PORT_EMIFF;
	}
	if (cpu_is_omap24xx())
		sync_dev = mcbsp[id].dma_tx_sync;

	omap_set_dma_transfer_params(mcbsp[id].dma_tx_lch,
				     OMAP_DMA_DATA_TYPE_S16,
				     length >> 1, 1,
				     OMAP_DMA_SYNC_ELEMENT,
	 sync_dev, 0);

	omap_set_dma_dest_params(mcbsp[id].dma_tx_lch,
				 src_port,
				 OMAP_DMA_AMODE_CONSTANT,
				 mcbsp[id].io_base + OMAP_MCBSP_REG_DXR1,
				 0, 0);

	omap_set_dma_src_params(mcbsp[id].dma_tx_lch,
				dest_port,
				OMAP_DMA_AMODE_POST_INC,
				buffer,
				0, 0);

	omap_start_dma(mcbsp[id].dma_tx_lch);
	wait_for_completion(&(mcbsp[id].tx_dma_completion));

	return 0;
}
EXPORT_SYMBOL(omap_mcbsp_xmit_buffer);

int omap_mcbsp_recv_buffer(unsigned int id, dma_addr_t buffer,
				unsigned int length)
{
	int dma_rx_ch;
	int src_port = 0;
	int dest_port = 0;
	int sync_dev = 0;

	if (omap_mcbsp_check(id) < 0)
		return -EINVAL;

	if (omap_request_dma(mcbsp[id].dma_rx_sync, "McBSP RX",
				omap_mcbsp_rx_dma_callback,
				&mcbsp[id],
				&dma_rx_ch)) {
		printk(KERN_ERR "Unable to request DMA channel for McBSP%d RX."
				" Trying IRQ based RX\n", id + 1);
		return -EAGAIN;
	}
	mcbsp[id].dma_rx_lch = dma_rx_ch;

	DBG("RX DMA on channel %d\n", dma_rx_ch);

	init_completion(&(mcbsp[id].rx_dma_completion));

	if (cpu_class_is_omap1()) {
		src_port = OMAP_DMA_PORT_TIPB;
		dest_port = OMAP_DMA_PORT_EMIFF;
	}
	if (cpu_is_omap24xx())
		sync_dev = mcbsp[id].dma_rx_sync;

	omap_set_dma_transfer_params(mcbsp[id].dma_rx_lch,
					OMAP_DMA_DATA_TYPE_S16,
					length >> 1, 1,
					OMAP_DMA_SYNC_ELEMENT,
					sync_dev, 0);

	omap_set_dma_src_params(mcbsp[id].dma_rx_lch,
				src_port,
				OMAP_DMA_AMODE_CONSTANT,
				mcbsp[id].io_base + OMAP_MCBSP_REG_DRR1,
				0, 0);

	omap_set_dma_dest_params(mcbsp[id].dma_rx_lch,
					dest_port,
					OMAP_DMA_AMODE_POST_INC,
					buffer,
					0, 0);

	omap_start_dma(mcbsp[id].dma_rx_lch);
	wait_for_completion(&(mcbsp[id].rx_dma_completion));

	return 0;
}
EXPORT_SYMBOL(omap_mcbsp_recv_buffer);

/*
 * SPI wrapper.
 * Since SPI setup is much simpler than the generic McBSP one,
 * this wrapper just need an omap_mcbsp_spi_cfg structure as an input.
 * Once this is done, you can call omap_mcbsp_start().
 */
void omap_mcbsp_set_spi_mode(unsigned int id,
				const struct omap_mcbsp_spi_cfg *spi_cfg)
{
	struct omap_mcbsp_reg_cfg mcbsp_cfg;

	if (omap_mcbsp_check(id) < 0)
		return;

	memset(&mcbsp_cfg, 0, sizeof(struct omap_mcbsp_reg_cfg));

	/* SPI has only one frame */
	mcbsp_cfg.rcr1 |= (RWDLEN1(spi_cfg->word_length) | RFRLEN1(0));
	mcbsp_cfg.xcr1 |= (XWDLEN1(spi_cfg->word_length) | XFRLEN1(0));

	/* Clock stop mode */
	if (spi_cfg->clk_stp_mode == OMAP_MCBSP_CLK_STP_MODE_NO_DELAY)
		mcbsp_cfg.spcr1 |= (1 << 12);
	else
		mcbsp_cfg.spcr1 |= (3 << 11);

	/* Set clock parities */
	if (spi_cfg->rx_clock_polarity == OMAP_MCBSP_CLK_RISING)
		mcbsp_cfg.pcr0 |= CLKRP;
	else
		mcbsp_cfg.pcr0 &= ~CLKRP;

	if (spi_cfg->tx_clock_polarity == OMAP_MCBSP_CLK_RISING)
		mcbsp_cfg.pcr0 &= ~CLKXP;
	else
		mcbsp_cfg.pcr0 |= CLKXP;

	/* Set SCLKME to 0 and CLKSM to 1 */
	mcbsp_cfg.pcr0 &= ~SCLKME;
	mcbsp_cfg.srgr2 |= CLKSM;

	/* Set FSXP */
	if (spi_cfg->fsx_polarity == OMAP_MCBSP_FS_ACTIVE_HIGH)
		mcbsp_cfg.pcr0 &= ~FSXP;
	else
		mcbsp_cfg.pcr0 |= FSXP;

	if (spi_cfg->spi_mode == OMAP_MCBSP_SPI_MASTER) {
		mcbsp_cfg.pcr0 |= CLKXM;
		mcbsp_cfg.srgr1 |= CLKGDV(spi_cfg->clk_div - 1);
		mcbsp_cfg.pcr0 |= FSXM;
		mcbsp_cfg.srgr2 &= ~FSGM;
		mcbsp_cfg.xcr2 |= XDATDLY(1);
		mcbsp_cfg.rcr2 |= RDATDLY(1);
	} else {
		mcbsp_cfg.pcr0 &= ~CLKXM;
		mcbsp_cfg.srgr1 |= CLKGDV(1);
		mcbsp_cfg.pcr0 &= ~FSXM;
		mcbsp_cfg.xcr2 &= ~XDATDLY(3);
		mcbsp_cfg.rcr2 &= ~RDATDLY(3);
	}

	mcbsp_cfg.xcr2 &= ~XPHASE;
	mcbsp_cfg.rcr2 &= ~RPHASE;

	omap_mcbsp_config(id, &mcbsp_cfg);
}
EXPORT_SYMBOL(omap_mcbsp_set_spi_mode);

#else

/*
 * Set McBSP recv parameters
 * id           : McBSP interface ID
 * mcbsp_cfg    : McBSP register configuration
 * rp           : McBSP recv parameters
 */
int omap_mcbsp_set_recv_param(unsigned int id,
				struct omap_mcbsp_reg_cfg *mcbsp_cfg,
				struct omap_mcbsp_cfg_param *rp)
{
	u32 io_base;
	io_base = mcbsp[id].io_base;

	mcbsp_cfg->spcr1 = RJUST(rp->justification);
	mcbsp_cfg->rcr2 = RCOMPAND(rp->reverse_compand) |
				RDATDLY(rp->data_delay);
	if (rp->phase == OMAP_MCBSP_FRAME_SINGLEPHASE)
		mcbsp_cfg->rcr2 = mcbsp_cfg->rcr2 & ~(RPHASE);
	else
		mcbsp_cfg->rcr2 = mcbsp_cfg->rcr2  | (RPHASE);
	mcbsp_cfg->rcr1 = RWDLEN1(rp->word_length1) |
				RFRLEN1(rp->frame_length1);
	if (rp->fsync_src == OMAP_MCBSP_RXFSYNC_INTERNAL)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | FSRM;
	if (rp->clk_mode == OMAP_MCBSP_CLKRXSRC_INTERNAL)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | CLKRM;
	if (rp->clk_polarity == OMAP_MCBSP_CLKR_POLARITY_RISING)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | CLKRP;
	if (rp->fs_polarity == OMAP_MCBSP_FS_ACTIVE_LOW)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | FSRP;
	return 0;

}

/*
 * Set McBSP transmit parameters
 * id		: McBSP interface ID
 * mcbsp_cfg	: McBSP register configuration
 * tp		: McBSP transmit parameters
 */

int omap_mcbsp_set_trans_param(unsigned int id,
					struct omap_mcbsp_reg_cfg *mcbsp_cfg,
					struct omap_mcbsp_cfg_param *tp)
{
	mcbsp_cfg->xcr2 = XCOMPAND(tp->reverse_compand) |
					XDATDLY(tp->data_delay);
	if (tp->phase == OMAP_MCBSP_FRAME_SINGLEPHASE)
		mcbsp_cfg->xcr2 = mcbsp_cfg->xcr2 & ~(XPHASE);
	else
		mcbsp_cfg->xcr2 = mcbsp_cfg->xcr2 | (XPHASE);
	mcbsp_cfg->xcr1 = XWDLEN1(tp->word_length1) |
				XFRLEN1(tp->frame_length1);
	if (tp->fs_polarity == OMAP_MCBSP_FS_ACTIVE_LOW)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | FSXP;
	if (tp->fsync_src == OMAP_MCBSP_TXFSYNC_INTERNAL)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | FSXM;
	if (tp->clk_mode == OMAP_MCBSP_CLKTXSRC_INTERNAL)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | CLKXM;
	if (tp->clk_polarity == OMAP_MCBSP_CLKX_POLARITY_FALLING)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | CLKXP;
	return 0;

}
/*
 * Set McBSP SRG configuration
 * id			: McBSP interface ID
 * mcbsp_cfg		: McBSP register configuration
 * interface_mode	: Master/Slave
 * param		: McBSP SRG and FSG configuration
 */

int omap_mcbsp_set_srg_cfg_param(unsigned int id, int interface_mode,
					struct omap_mcbsp_reg_cfg *mcbsp_cfg,
					struct omap_mcbsp_srg_fsg_cfg *param)
{
	u32 io_base;
	u32 clk_rate, clkgdv;
	io_base = mcbsp[id].io_base;

	mcbsp[id].interface_mode = interface_mode;
	mcbsp_cfg->srgr1 = FWID(param->pulse_width);

	if (interface_mode == OMAP_MCBSP_MASTER) {
		clk_rate = clk_get_rate(omap_mcbsp_clk[id].fck);
		clkgdv = clk_rate / (param->sample_rate *
				(param->bits_per_sample - 1));
		mcbsp_cfg->srgr1 = mcbsp_cfg->srgr1 | CLKGDV(clkgdv);
	}
	if (param->dlb)
		mcbsp_cfg->spcr1 = mcbsp_cfg->spcr1 & ~(ALB);

	if (param->sync_mode == OMAP_MCBSP_SRG_FREERUNNING)
		mcbsp_cfg->spcr2 = mcbsp_cfg->spcr2 | FREE;
	mcbsp_cfg->srgr2 = FPER(param->period)|(param->fsgm? FSGM : 0);

	switch (param->srg_src) {

	case OMAP_MCBSP_SRGCLKSRC_CLKS:
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 & ~(SCLKME);
		mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2 & ~(CLKSM);
		/*
		 * McBSP master operation at low voltage is only possible if
		 * CLKSP=0 In Master mode, if client driver tries to configiure
		 * input clock polarity as falling edge, we force it to Rising
		 */
		if ((param->polarity == OMAP_MCBSP_CLKS_POLARITY_RISING) ||
					(interface_mode == OMAP_MCBSP_MASTER))
			mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2  & ~(CLKSP);
		else
			mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2  |  (CLKSP);

		break;

	case OMAP_MCBSP_SRGCLKSRC_FCLK:
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 & ~(SCLKME);
		mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2 | (CLKSM);

		break;

	case OMAP_MCBSP_SRGCLKSRC_CLKR:
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0   | (SCLKME);
		mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2 & ~(CLKSM);
		if (param->polarity == OMAP_MCBSP_CLKR_POLARITY_FALLING)
			mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0  & ~(CLKRP);
		else
			mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0  | (CLKRP);

		break;

	case OMAP_MCBSP_SRGCLKSRC_CLKX:
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0   | (SCLKME);
		mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2 | (CLKSM);

		if (param->polarity == OMAP_MCBSP_CLKX_POLARITY_RISING)
			mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0  & ~(CLKXP);
		else
			mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0  | (CLKXP);
		break;

	}
	if (param->sync_mode == OMAP_MCBSP_SRG_FREERUNNING)
		mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2 & ~(GSYNC);
	else if (param->sync_mode == OMAP_MCBSP_SRG_RUNNING)
		mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2 | (GSYNC);

	mcbsp_cfg->xccr = OMAP_MCBSP_READ(io_base, XCCR);
	if (param->dlb)
		mcbsp_cfg->xccr = mcbsp_cfg->xccr | (DLB);

	mcbsp_cfg->rccr = OMAP_MCBSP_READ(io_base, RCCR);
	return 0;

}

/*
 * configure the McBSP registers
 * id			: McBSP interface ID
 * interface_mode	: Master/Slave
 * rp			: McBSP recv parameters
 * tp			: McBSP transmit parameters
 * param		: McBSP SRG and FSG configuration
 */
int omap_mcbsp_params_cfg(unsigned int id, int interface_mode,
				struct omap_mcbsp_cfg_param *rp,
				struct omap_mcbsp_cfg_param *tp,
				struct omap_mcbsp_srg_fsg_cfg *param)
 {
	struct omap_mcbsp_reg_cfg mcbsp_cfg = {0};

	spin_lock(&mcbsp[id].lock);

	if (rp)
		omap_mcbsp_set_recv_param(id, &mcbsp_cfg, rp);

	if (tp)
		omap_mcbsp_set_trans_param(id, &mcbsp_cfg, tp);

	if (param)
		omap_mcbsp_set_srg_cfg_param(id,
					interface_mode, &mcbsp_cfg, param);

	omap_mcbsp_config(id, &mcbsp_cfg);
	spin_unlock(&mcbsp[id].lock);
	return (0);

 }
EXPORT_SYMBOL(omap_mcbsp_params_cfg);

/*
 * Enable/Disable the sample rate generator
 * id		: McBSP interface ID
 * state	: Enable/Disable
 */
int omap_mcbsp_set_srg_fsg(unsigned int id, u8 state)
{
	u32 io_base;

	if (omap_mcbsp_check(id) < 0)
		return -EINVAL;

	io_base = mcbsp[id].io_base;
	spin_lock(&mcbsp[id].lock);

	if (state == OMAP_MCBSP_DISABLE_FSG_SRG) {
		OMAP_MCBSP_WRITE(io_base, SPCR2,
			OMAP_MCBSP_READ(io_base, SPCR2) & (~GRST));
		OMAP_MCBSP_WRITE(io_base, SPCR2,
			OMAP_MCBSP_READ(io_base, SPCR2) & (~FRST));
	} else {
		OMAP_MCBSP_WRITE(io_base, SPCR2,
			OMAP_MCBSP_READ(io_base, SPCR2) | (GRST));
		OMAP_MCBSP_WRITE(io_base, SPCR2,
			OMAP_MCBSP_READ(io_base, SPCR2) | (FRST));
	}
	spin_unlock(&mcbsp[id].lock);
	return (0);
}

/*
 * Stop transmitting data on a McBSP interface
 * id		: McBSP interface ID
 */
int omap_mcbsp_stop_datatx(unsigned int id)
{
	u32 io_base;

	if (omap_mcbsp_check(id) < 0)
		return -EINVAL;
	spin_lock(&mcbsp[id].lock);
	io_base = mcbsp[id].io_base;

	if (mcbsp[id].dma_tx_lch != -1) {
		if (omap_stop_dma_chain_transfers(mcbsp[id].dma_tx_lch) != 0) {
			spin_unlock(&mcbsp[id].lock);
			return -EPERM;
		}
	}
	mcbsp[id].tx_dma_chain_state = 0;
	OMAP_MCBSP_WRITE(io_base, SPCR2,
		OMAP_MCBSP_READ(io_base, SPCR2) & (~XRST));

	if (!(--mcbsp[id].srg_enabled))
		omap_mcbsp_set_srg_fsg(id, OMAP_MCBSP_DISABLE_FSG_SRG);
	spin_unlock(&mcbsp[id].lock);
	return 0;
}
EXPORT_SYMBOL(omap_mcbsp_stop_datatx);

/*
 * Stop receving data on a McBSP interface
 * id		: McBSP interface ID
 */
int omap_mcbsp_stop_datarx(u32 id)
{

	u32 io_base;

	if (omap_mcbsp_check(id) < 0)
		return -EINVAL;
	spin_lock(&mcbsp[id].lock);
	io_base = mcbsp[id].io_base;

	if (mcbsp[id].dma_rx_lch != -1) {
		if (omap_stop_dma_chain_transfers(mcbsp[id].dma_rx_lch) != 0) {
			spin_unlock(&mcbsp[id].lock);
			return -EPERM;
		}
	}
	OMAP_MCBSP_WRITE(io_base, SPCR1,
		OMAP_MCBSP_READ(io_base, SPCR1) & (~RRST));
	mcbsp[id].rx_dma_chain_state = 0;
	if (!(--mcbsp[id].srg_enabled))
		omap_mcbsp_set_srg_fsg(id, OMAP_MCBSP_DISABLE_FSG_SRG);
	spin_unlock(&mcbsp[id].lock);
	return 0;
}
EXPORT_SYMBOL(omap_mcbsp_stop_datarx);

/*
 * Interface Reset
 * id	: McBSP interface ID
 * Resets the McBSP interface
 */
int omap_mcbsp_reset(unsigned int id)
{
	u32 io_base;
	int counter = 0;
	int wait_for_reset = 10000;

	if (omap_mcbsp_check(id) < 0)
		return -EINVAL;
	io_base = mcbsp[id].io_base;
	spin_lock(&mcbsp[id].lock);

	OMAP_MCBSP_WRITE(io_base, SYSCONFIG,
		OMAP_MCBSP_READ(io_base, SYSCONFIG) | (SOFTRST));

	while (OMAP_MCBSP_READ(io_base, SYSCONFIG) & SOFTRST) {
		if (!in_interrupt()) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(10);
		}
		if (counter++ > wait_for_reset) {
			printk(KERN_ERR "mcbsp[%d] Reset timeout\n", id);
			spin_unlock(&mcbsp[id].lock);
			return -ETIMEDOUT;
		}
	}
	spin_unlock(&mcbsp[id].lock);
	return 0;
}
EXPORT_SYMBOL(omap_mcbsp_reset);

/*
 * Get the element index and frame index of transmitter
 * id		: McBSP interface ID
 * ei		: element index
 * fi		: frame index
 */
int omap_mcbsp_transmitter_index(int id, int *ei, int *fi)
{
	int eix = 0, fix = 0;

	if (omap_mcbsp_check(id) < 0)
		return -EINVAL;

	if ((!ei) || (!fi)) {
		printk(KERN_ERR	"OMAP_McBSP: Invalid ei and fi params \n");
		goto txinx_err;
	}

	if (mcbsp[id].dma_tx_lch == -1) {
		printk(KERN_ERR "OMAP_McBSP: Transmitter not started\n");
		goto txinx_err;
	}

	if (omap_get_dma_chain_index
		(mcbsp[id].dma_tx_lch, &eix, &fix) != 0) {
		printk(KERN_ERR "OMAP_McBSP: Getting chain index failed\n");
		goto txinx_err;
	}

	*ei = eix;
	*fi = fix;

	return 0;

txinx_err:
	return -EINVAL;
}
EXPORT_SYMBOL(omap_mcbsp_transmitter_index);

/*
 * Get the element index and frame index of receiver
 * id	: McBSP interface ID
 * ei		: element index
 * fi		: frame index
 */
int omap_mcbsp_receiver_index(int id, int *ei, int *fi)
{
	int eix = 0, fix = 0;

	if (omap_mcbsp_check(id) < 0)
		return -EINVAL;

	if ((!ei) || (!fi)) {
		printk(KERN_ERR	"OMAP_McBSP: Invalid ei and fi params x\n");
		goto rxinx_err;
	}

	/* Check if chain exists */
	if (mcbsp[id].dma_rx_lch == -1) {
		printk(KERN_ERR "OMAP_McBSP: Receiver not started\n");
		goto rxinx_err;
	}

	/* Get dma_chain_index */
	if (omap_get_dma_chain_index
		(mcbsp[id].dma_rx_lch, &eix, &fix) != 0) {
		printk(KERN_ERR "OMAP_McBSP: Getting chain index failed\n");
		goto rxinx_err;
	}

	*ei = eix;
	*fi = fix;
	return 0;

rxinx_err:
	return -EINVAL;
}
EXPORT_SYMBOL(omap_mcbsp_receiver_index);

/*
 * Basic Reset Transmitter
 * id		: McBSP interface number
 * state	: Disable (0)/ Enable (1) the transmitter
 */
int omap_mcbsp_set_xrst(unsigned int id, u8 state)
{
	u32 io_base;
	if (omap_mcbsp_check(id) < 0)
		return -EINVAL;

	io_base = mcbsp[id].io_base;

	if (state == OMAP_MCBSP_XRST_DISABLE)
		OMAP_MCBSP_WRITE(io_base, SPCR2,
			OMAP_MCBSP_READ(io_base, SPCR2) & (~XRST));
	else
		OMAP_MCBSP_WRITE(io_base, SPCR2,
			OMAP_MCBSP_READ(io_base, SPCR2) | (XRST));
	udelay(10);

	return (0);
}
EXPORT_SYMBOL(omap_mcbsp_set_xrst);

/*
 * Reset Receiver
 * id		: McBSP interface number
 * state	: Disable (0)/ Enable (1) the receiver
 */
int omap_mcbsp_set_rrst(unsigned int id, u8 state)
{
	u32 io_base;
	if (omap_mcbsp_check(id) < 0)
		return -EINVAL;

	io_base = mcbsp[id].io_base;

	if (state == OMAP_MCBSP_RRST_DISABLE)
		OMAP_MCBSP_WRITE(io_base, SPCR1,
			OMAP_MCBSP_READ(io_base, SPCR1) & (~RRST));
	else
		OMAP_MCBSP_WRITE(io_base, SPCR1,
			OMAP_MCBSP_READ(io_base, SPCR1) | (RRST));
	udelay(10);
	return 0;
}
EXPORT_SYMBOL(omap_mcbsp_set_rrst);

/*
 * Configure the receiver parameters
 * id		: McBSP Interface ID
 * rp		: DMA Receive parameters
 */
int omap_mcbsp_dma_recv_params(unsigned int id,
				omap_mcbsp_dma_transfer_params * rp)
{
	u32 io_base;
	int err, chain_id = -1;
	struct omap_dma_channel_params rx_params;
	u32  dt = 0;

	if (omap_mcbsp_check(id) < 0)
		return -EINVAL;
	spin_lock(&mcbsp[id].lock);
	io_base = mcbsp[id].io_base;

	dt = rp->word_length1;
	if ((dt != OMAP_MCBSP_WORD_8) && (dt != OMAP_MCBSP_WORD_16) &&
						(dt != OMAP_MCBSP_WORD_32))
		return -EINVAL;
	if (dt == OMAP_MCBSP_WORD_8)
		rx_params.data_type = OMAP_DMA_DATA_TYPE_S8;
	else if (dt == OMAP_MCBSP_WORD_16)
		rx_params.data_type = OMAP_DMA_DATA_TYPE_S16;
	else
		rx_params.data_type = OMAP_DMA_DATA_TYPE_S32;

	rx_params.read_prio = DMA_CH_PRIO_HIGH;
	rx_params.write_prio = DMA_CH_PRIO_HIGH;

	omap_dma_set_global_params(DMA_DEFAULT_ARB_RATE,
						DMA_DEFAULT_FIFO_DEPTH, 0);

	rx_params.sync_mode = OMAP_DMA_SYNC_ELEMENT;
	rx_params.src_fi = 0;
	rx_params.trigger = mcbsp[id].dma_rx_sync;
	rx_params.src_or_dst_synch = 0x01;
	rx_params.src_amode = OMAP_DMA_AMODE_CONSTANT;
	rx_params.src_ei = 0x0;
	/* Indexing is always in bytes - so multiply with dt */
	dt = (rx_params.data_type == OMAP_DMA_DATA_TYPE_S8) ? 1 :
		(rx_params.data_type == OMAP_DMA_DATA_TYPE_S16) ? 2 : 4;

	if (rp->skip_alt == OMAP_MCBSP_SKIP_SECOND) {
		rx_params.dst_amode = OMAP_DMA_AMODE_DOUBLE_IDX;
		rx_params.dst_ei = (1);
		rx_params.dst_fi = (1) + ((-1) * dt);
	} else if (rp->skip_alt == OMAP_MCBSP_SKIP_FIRST) {
		rx_params.dst_amode = OMAP_DMA_AMODE_DOUBLE_IDX;
		rx_params.dst_ei = 1 + (-2) * dt;
		rx_params.dst_fi = 1 + (2) * dt;
	} else {
		rx_params.dst_amode = OMAP_DMA_AMODE_POST_INC;
		rx_params.dst_ei = 0;
		rx_params.dst_fi = 0;
	}

	mcbsp[id].rxskip_alt = rp->skip_alt;
	mcbsp[id].auto_reset &= ~OMAP_MCBSP_AUTO_RRST;
	mcbsp[id].auto_reset |=	(rp->auto_reset & OMAP_MCBSP_AUTO_RRST);

	mcbsp[id].rx_word_length = rx_params.data_type << 0x1;
	if (rx_params.data_type == 0)
		mcbsp[id].rx_word_length = 1;

	mcbsp[id].rx_callback = rp->callback;

	/* request for a chain of dma channels for data reception */
	if (mcbsp[id].dma_rx_lch == -1) {
		err = omap_request_dma_chain(id, "McBSP RX",
					 omap_mcbsp_rx_dma_callback, &chain_id,
					 omap_mcbsp_max_dmachs_rx[id],
					 OMAP_DMA_DYNAMIC_CHAIN, rx_params);
		if (err < 0) {
			printk(KERN_ERR "Receive path configuration failed \n");
			spin_unlock(&mcbsp[id].lock);
			return -EPERM;
		}
		mcbsp[id].dma_rx_lch = chain_id;
		omap_get_mcbspid[chain_id] = id;
		mcbsp[id].rx_dma_chain_state = 0;
	} else {
		/* DMA params already set, modify the same!! */
		err = omap_modify_dma_chain_params(mcbsp[id].
						 dma_rx_lch, rx_params);
		if (err < 0) {
			spin_unlock(&mcbsp[id].lock);
			return -EPERM;
		}
	}

	spin_unlock(&mcbsp[id].lock);
	return 0;
}
EXPORT_SYMBOL(omap_mcbsp_dma_recv_params);

/*
 * Configure the transmitter parameters
 * id		: McBSP Interface ID
 * tp		: DMA Transfer parameters
 */

int omap_mcbsp_dma_trans_params(unsigned int id,
				 omap_mcbsp_dma_transfer_params * tp)
{

	struct omap_dma_channel_params tx_params;
	int err = 0, chain_id = -1;
	u32 io_base;
	u32 dt = 0;


	if (omap_mcbsp_check(id) < 0)
		return -EINVAL;
	io_base = mcbsp[id].io_base;
	spin_lock(&mcbsp[id].lock);

	dt = tp->word_length1;
	if ((dt != OMAP_MCBSP_WORD_8) && (dt != OMAP_MCBSP_WORD_16)
						 && (dt != OMAP_MCBSP_WORD_32))
		return -EINVAL;
	if (dt == OMAP_MCBSP_WORD_8)
		tx_params.data_type = OMAP_DMA_DATA_TYPE_S8;
	else if (dt == OMAP_MCBSP_WORD_16)
		tx_params.data_type = OMAP_DMA_DATA_TYPE_S16;
	else
		tx_params.data_type = OMAP_DMA_DATA_TYPE_S32;

	tx_params.read_prio = DMA_CH_PRIO_HIGH;
	tx_params.write_prio = DMA_CH_PRIO_HIGH;

	omap_dma_set_global_params(DMA_DEFAULT_ARB_RATE,
					DMA_DEFAULT_FIFO_DEPTH, 0);

	tx_params.sync_mode = OMAP_DMA_SYNC_ELEMENT;
	tx_params.dst_fi = 0;

	tx_params.trigger = mcbsp[id].dma_tx_sync;
	tx_params.src_or_dst_synch = 0;
	/* Indexing is always in bytes - so multiply with dt */
	mcbsp[id].tx_word_length = tx_params.data_type << 0x1;
	if (tx_params.data_type == 0)
		mcbsp[id].tx_word_length = 1;
	dt = mcbsp[id].tx_word_length;
	if (tp->skip_alt == OMAP_MCBSP_SKIP_SECOND) {
		tx_params.src_amode = OMAP_DMA_AMODE_DOUBLE_IDX;
		tx_params.src_ei = (1);
		tx_params.src_fi = (1) + ((-1) * dt);
	} else if (tp->skip_alt == OMAP_MCBSP_SKIP_FIRST) {
		tx_params.src_amode = OMAP_DMA_AMODE_DOUBLE_IDX;
		tx_params.src_ei = 1 + (-2) * dt;
		tx_params.src_fi = 1 + (2) * dt;
	} else {
		tx_params.src_amode = OMAP_DMA_AMODE_POST_INC;
		tx_params.src_ei = 0;
		tx_params.src_fi = 0;
	}

	tx_params.dst_amode = OMAP_DMA_AMODE_CONSTANT;
	tx_params.dst_ei = 0;

	mcbsp[id].txskip_alt = tp->skip_alt;
	mcbsp[id].auto_reset &= ~OMAP_MCBSP_AUTO_XRST;
	mcbsp[id].auto_reset |=
		(tp->auto_reset & OMAP_MCBSP_AUTO_XRST);

	mcbsp[id].tx_callback = tp->callback;

	/* Based on Rjust we can do double indexing DMA params configuration */

	if (mcbsp[id].dma_tx_lch == -1) {
		err = omap_request_dma_chain(id, "McBSP TX",
					 omap_mcbsp_tx_dma_callback, &chain_id,
					 omap_mcbsp_max_dmachs_tx[id],
					 OMAP_DMA_DYNAMIC_CHAIN, tx_params);
		if (err < 0) {
			printk(KERN_ERR
				"Transmit path configuration failed \n");
			spin_unlock(&mcbsp[id].lock);
			return -EPERM;
		}
		mcbsp[id].tx_dma_chain_state = 0;
		mcbsp[id].dma_tx_lch = chain_id;
		omap_get_mcbspid[chain_id] = id;
	} else {
		/* DMA params already set, modify the same!! */
		err = omap_modify_dma_chain_params(mcbsp[id].
						 dma_tx_lch, tx_params);
		if (err < 0) {
			spin_unlock(&mcbsp[id].lock);
			return -EPERM;
		}
	}

	spin_unlock(&mcbsp[id].lock);
	return 0;
}
EXPORT_SYMBOL(omap_mcbsp_dma_trans_params);

/*
 * Start receving data on a McBSP interface
 * id			: McBSP interface ID
 * cbdata		: User data to be returned with callback
 * buf_start_addr	: The destination address [physical address]
 * buf_size		: Buffer size
 */

int omap_mcbsp_receive_data(unsigned int id, void *cbdata,
			     dma_addr_t buf_start_addr, u32 buf_size)
{
	u32 dma_chain_status = 0;
	u32 io_base;
	int enable_rx = 0;
	int e_count = 0;
	int f_count = 0;

	if (omap_mcbsp_check(id) < 0)
		return -EINVAL;
	io_base = mcbsp[id].io_base;
	spin_lock(&mcbsp[id].lock);


	/* Auto RRST handling logic - disable the Reciever before 1st dma */
	if ((mcbsp[id].auto_reset & OMAP_MCBSP_AUTO_RRST) &&
		(omap_dma_chain_status(mcbsp[id].dma_rx_lch)
				== OMAP_DMA_CHAIN_INACTIVE)) {
			OMAP_MCBSP_WRITE(io_base, SPCR1,
				OMAP_MCBSP_READ(io_base, SPCR1) & (~RRST));
			enable_rx = 1;
	}

	/*
	 * for skip_first and second, we need to set e_count =2,
	 * and f_count = number of frames = number of elements/e_count
	 */
	e_count = (buf_size / mcbsp[id].rx_word_length);

	if (mcbsp[id].rxskip_alt != OMAP_MCBSP_SKIP_NONE) {
		/*
		 * since the number of frames = total number of elements/element
		 * count, However, with double indexing for data transfers,
		 * double the number of elements need to be transmitted
		 */
		f_count = e_count;
		e_count = 2;
	} else {
		f_count = 1;
	}
	/*
	 * If the DMA is to be configured to skip the first byte, we need
	 * to jump backwards, so we need to move one chunk forward and
	 * ask dma if we dont want the client driver knowing abt this.
	 */
	if (mcbsp[id].rxskip_alt == OMAP_MCBSP_SKIP_FIRST)
		buf_start_addr += mcbsp[id].rx_word_length;

	dma_chain_status = omap_dma_chain_a_transfer(mcbsp[id]. dma_rx_lch,
				(mcbsp[id].phy_base + OMAP_MCBSP_REG_DRR),
				 buf_start_addr, e_count, f_count, cbdata);

	if (mcbsp[id].rx_dma_chain_state == 0) {
		if (mcbsp[id].interface_mode == OMAP_MCBSP_MASTER) {
			omap_mcbsp_set_srg_fsg(id, OMAP_MCBSP_ENABLE_FSG_SRG);
			mcbsp[id].srg_enabled++ ;
		}
		dma_chain_status =
			omap_start_dma_chain_transfers(mcbsp[id].dma_rx_lch);
		mcbsp[id].rx_dma_chain_state = 1;
	}

	/* Auto RRST handling logic - Enable the Reciever after 1st dma */
	if (enable_rx &&
		(omap_dma_chain_status(mcbsp[id].dma_rx_lch)
				== OMAP_DMA_CHAIN_ACTIVE))
			OMAP_MCBSP_WRITE(io_base, SPCR1,
				OMAP_MCBSP_READ(io_base, SPCR1) | (RRST));

	if (dma_chain_status < 0) {
		spin_unlock(&mcbsp[id].lock);
		return -EPERM;
	}

	spin_unlock(&mcbsp[id].lock);
	return 0;
}
EXPORT_SYMBOL(omap_mcbsp_receive_data);

/*
 * Start transmitting data through a McBSP interface
 * id			: McBSP interface ID
 * cbdata		: User data to be returned with callback
 * buf_start_addr	: The source address [This should be physical address]
 * buf_size		: Buffer size
 */
int omap_mcbsp_send_data(unsigned int id, void *cbdata,
			  dma_addr_t buf_start_addr, u32 buf_size)
{
	u32 io_base;
	u32 dma_chain_state = 0;
	u8 enable_tx = 0;
	int e_count = 0;
	int f_count = 0;

	/* Check if mcbsp interface is valid and is reserved */
	if (omap_mcbsp_check(id) < 0)
		return -EINVAL;
	io_base = mcbsp[id].io_base;

	spin_lock(&mcbsp[id].lock);

	/* Auto RRST handling logic - disable the Reciever before 1st dma */
	if ((mcbsp[id].auto_reset & OMAP_MCBSP_AUTO_XRST) &&
			(omap_dma_chain_status(mcbsp[id].dma_tx_lch)
				== OMAP_DMA_CHAIN_INACTIVE)) {
			OMAP_MCBSP_WRITE(io_base, SPCR2,
				OMAP_MCBSP_READ(io_base, SPCR2) & (~XRST));
			enable_tx = 1;
	}

	/*
	 * for skip_first and second, we need to set e_count =2, and
	 * f_count = number of frames = number of elements/e_count
	 */
	e_count = (buf_size / mcbsp[id].tx_word_length);
	if (mcbsp[id].txskip_alt != OMAP_MCBSP_SKIP_NONE) {
		/*
		 * number of frames = total number of elements/element count,
		 * However, with double indexing for data transfers, double I
		 * the number of elements need to be transmitted
		 */
		f_count = e_count;
		e_count = 2;
	} else {
		f_count = 1;
	}

	/*
	 * If the DMA is to be configured to skip the first byte, we need
	 * to jump backwards, so we need to move one chunk forward and ask
	 * dma if we dont want the client driver knowing abt this.
	 */
	if (mcbsp[id].txskip_alt == OMAP_MCBSP_SKIP_FIRST)
		buf_start_addr += mcbsp[id].tx_word_length;

	dma_chain_state = omap_dma_chain_a_transfer(mcbsp[id].dma_tx_lch,
					buf_start_addr,
					mcbsp[id].phy_base + OMAP_MCBSP_REG_DXR,
					e_count, f_count, cbdata);

	if (mcbsp[id].tx_dma_chain_state == 0) {
		if (mcbsp[id].interface_mode == OMAP_MCBSP_MASTER) {
			omap_mcbsp_set_srg_fsg(id, OMAP_MCBSP_ENABLE_FSG_SRG);
			mcbsp[id].srg_enabled++ ;
		}
		dma_chain_state =
			omap_start_dma_chain_transfers(mcbsp[id].dma_tx_lch);
		mcbsp[id].tx_dma_chain_state = 1;
	}

	if (dma_chain_state < 0) {
		spin_unlock(&mcbsp[id].lock);
		return -EPERM;
	}
	/* Auto XRST handling logic - Enable the Reciever after 1st dma */
	if (enable_tx &&
		(omap_dma_chain_status(mcbsp[id].dma_tx_lch)
		== OMAP_DMA_CHAIN_ACTIVE))
			OMAP_MCBSP_WRITE(io_base, SPCR2,
				OMAP_MCBSP_READ(io_base, SPCR2) | (XRST));
	spin_unlock(&mcbsp[id].lock);
	return 0;
}
EXPORT_SYMBOL(omap_mcbsp_send_data);

#endif

/*
 * McBSP1 and McBSP3 are directly mapped on 1610 and 1510.
 * 730 has only 2 McBSP, and both of them are MPU peripherals.
 */
struct omap_mcbsp_info {
	u32 virt_base;
	u32 phy_base;
	u8 dma_rx_sync, dma_tx_sync;
	u16 rx_irq, tx_irq;
};

#ifdef CONFIG_ARCH_OMAP730
static const struct omap_mcbsp_info mcbsp_730[] = {
	[0] = { .virt_base = io_p2v(OMAP730_MCBSP1_BASE),
		.dma_rx_sync = OMAP_DMA_MCBSP1_RX,
		.dma_tx_sync = OMAP_DMA_MCBSP1_TX,
		.rx_irq = INT_730_McBSP1RX,
		.tx_irq = INT_730_McBSP1TX },
	[1] = { .virt_base = io_p2v(OMAP730_MCBSP2_BASE),
		.dma_rx_sync = OMAP_DMA_MCBSP3_RX,
		.dma_tx_sync = OMAP_DMA_MCBSP3_TX,
		.rx_irq = INT_730_McBSP2RX,
		.tx_irq = INT_730_McBSP2TX },
};
#endif

#ifdef CONFIG_ARCH_OMAP15XX
static const struct omap_mcbsp_info mcbsp_1510[] = {
	[0] = { .virt_base = OMAP1510_MCBSP1_BASE,
		.dma_rx_sync = OMAP_DMA_MCBSP1_RX,
		.dma_tx_sync = OMAP_DMA_MCBSP1_TX,
		.rx_irq = INT_McBSP1RX,
		.tx_irq = INT_McBSP1TX },
	[1] = { .virt_base = io_p2v(OMAP1510_MCBSP2_BASE),
		.dma_rx_sync = OMAP_DMA_MCBSP2_RX,
		.dma_tx_sync = OMAP_DMA_MCBSP2_TX,
		.rx_irq = INT_1510_SPI_RX,
		.tx_irq = INT_1510_SPI_TX },
	[2] = { .virt_base = OMAP1510_MCBSP3_BASE,
		.dma_rx_sync = OMAP_DMA_MCBSP3_RX,
		.dma_tx_sync = OMAP_DMA_MCBSP3_TX,
		.rx_irq = INT_McBSP3RX,
		.tx_irq = INT_McBSP3TX },
};
#endif

#if defined(CONFIG_ARCH_OMAP16XX)
static const struct omap_mcbsp_info mcbsp_1610[] = {
	[0] = { .virt_base = OMAP1610_MCBSP1_BASE,
		.dma_rx_sync = OMAP_DMA_MCBSP1_RX,
		.dma_tx_sync = OMAP_DMA_MCBSP1_TX,
		.rx_irq = INT_McBSP1RX,
		.tx_irq = INT_McBSP1TX },
	[1] = { .virt_base = io_p2v(OMAP1610_MCBSP2_BASE),
		.dma_rx_sync = OMAP_DMA_MCBSP2_RX,
		.dma_tx_sync = OMAP_DMA_MCBSP2_TX,
		.rx_irq = INT_1610_McBSP2_RX,
		.tx_irq = INT_1610_McBSP2_TX },
	[2] = { .virt_base = OMAP1610_MCBSP3_BASE,
		.dma_rx_sync = OMAP_DMA_MCBSP3_RX,
		.dma_tx_sync = OMAP_DMA_MCBSP3_TX,
		.rx_irq = INT_McBSP3RX,
		.tx_irq = INT_McBSP3TX },
};
#endif

#if defined(CONFIG_ARCH_OMAP2420)
static const struct omap_mcbsp_info mcbsp_24xx[] = {
	[0] = { .virt_base = IO_ADDRESS(OMAP2420_MCBSP1_BASE),
		.dma_rx_sync = OMAP24XX_DMA_MCBSP1_RX,
		.dma_tx_sync = OMAP24XX_DMA_MCBSP1_TX,
		.rx_irq = INT_24XX_MCBSP1_IRQ_RX,
		.tx_irq = INT_24XX_MCBSP1_IRQ_TX,
		},
	[1] = { .virt_base = IO_ADDRESS(OMAP2420_MCBSP2_BASE),
		.dma_rx_sync = OMAP24XX_DMA_MCBSP2_RX,
		.dma_tx_sync = OMAP24XX_DMA_MCBSP2_TX,
		.rx_irq = INT_24XX_MCBSP2_IRQ_RX,
		.tx_irq = INT_24XX_MCBSP2_IRQ_TX,
		},
};
#endif

#if defined(CONFIG_ARCH_OMAP2430)
static const struct omap_mcbsp_info mcbsp_2430[] = {
	[0] = {
		.virt_base	= IO_ADDRESS(OMAP2430_MCBSP1_BASE),
		.phy_base	= OMAP2430_MCBSP1_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP1_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP1_TX,
		},
	[1] = {
		.virt_base	= IO_ADDRESS(OMAP2430_MCBSP2_BASE),
		.phy_base	= OMAP2430_MCBSP2_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP2_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP2_TX,
		},
	[2] = {
		.virt_base	= IO_ADDRESS(OMAP2430_MCBSP3_BASE),
		.phy_base	= OMAP2430_MCBSP3_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP3_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP3_TX,
		},
	[3] = {
		.virt_base	= IO_ADDRESS(OMAP2430_MCBSP4_BASE),
		.phy_base	= OMAP2430_MCBSP4_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP4_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP4_TX,
		},
	[4] = {
		.virt_base	= IO_ADDRESS(OMAP2430_MCBSP5_BASE),
		.phy_base	= OMAP2430_MCBSP5_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP5_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP5_TX,
		},
};
#endif

#if defined(CONFIG_ARCH_OMAP34XX)
static const struct omap_mcbsp_info mcbsp_34xx[] = {
	[0] = {
		.virt_base	= IO_ADDRESS(OMAP34XX_MCBSP1_BASE),
		.phy_base	= OMAP34XX_MCBSP1_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP1_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP1_TX,
		},
	[1] = {
		.virt_base	= IO_ADDRESS(OMAP34XX_MCBSP2_BASE),
		.phy_base	= OMAP34XX_MCBSP2_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP2_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP2_TX,
		},
	[2] = {
		.virt_base	= IO_ADDRESS(OMAP34XX_MCBSP3_BASE),
		.phy_base	= OMAP34XX_MCBSP3_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP3_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP3_TX,
		},
	[3] = {
		.virt_base	= IO_ADDRESS(OMAP34XX_MCBSP3_BASE),
		.phy_base	= OMAP34XX_MCBSP4_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP4_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP4_TX,
		},
	[4] = {
		.virt_base	= IO_ADDRESS(OMAP34XX_MCBSP3_BASE),
		.phy_base       = OMAP34XX_MCBSP5_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP5_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP5_TX,
		},
};
#endif

static int __init omap_mcbsp_init(void)
{
	int mcbsp_count = 0, i;
	static const struct omap_mcbsp_info *mcbsp_info;

	printk(KERN_INFO "Initializing OMAP McBSP system\n");

#ifdef CONFIG_ARCH_OMAP1
	mcbsp_dsp_ck = clk_get(0, "dsp_ck");
	if (IS_ERR(mcbsp_dsp_ck)) {
		printk(KERN_ERR "mcbsp: could not acquire dsp_ck handle.\n");
		return PTR_ERR(mcbsp_dsp_ck);
	}
	mcbsp_api_ck = clk_get(0, "api_ck");
	if (IS_ERR(mcbsp_api_ck)) {
		printk(KERN_ERR "mcbsp: could not acquire api_ck handle.\n");
		return PTR_ERR(mcbsp_api_ck);
	}
	mcbsp_dspxor_ck = clk_get(0, "dspxor_ck");
	if (IS_ERR(mcbsp_dspxor_ck)) {
		printk(KERN_ERR "mcbsp: could not acquire dspxor_ck handle.\n");
		return PTR_ERR(mcbsp_dspxor_ck);
	}
#endif
#ifdef CONFIG_ARCH_OMAP2420
	mcbsp1_ick = clk_get(0, "mcbsp1_ick");
	if (IS_ERR(mcbsp1_ick)) {
		printk(KERN_ERR "mcbsp: could not acquire "
				"mcbsp1_ick handle.\n");
		return PTR_ERR(mcbsp1_ick);
	}
	mcbsp1_fck = clk_get(0, "mcbsp1_fck");
	if (IS_ERR(mcbsp1_fck)) {
		printk(KERN_ERR "mcbsp: could not acquire "
				"mcbsp1_fck handle.\n");
		return PTR_ERR(mcbsp1_fck);
	}
	mcbsp2_ick = clk_get(0, "mcbsp2_ick");
	if (IS_ERR(mcbsp2_ick)) {
		printk(KERN_ERR "mcbsp: could not acquire "
				"mcbsp2_ick handle.\n");
		return PTR_ERR(mcbsp2_ick);
	}
	mcbsp2_fck = clk_get(0, "mcbsp2_fck");
	if (IS_ERR(mcbsp2_fck)) {
		printk(KERN_ERR "mcbsp: could not acquire "
				"mcbsp2_fck handle.\n");
		return PTR_ERR(mcbsp2_fck);
	}
#endif

#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP34XX)
	for (i = 0; i < OMAP_MAX_MCBSP_COUNT; i++) {

		omap_mcbsp_clk[i].ick = clk_get(0, omap_mcbsp_ick[i]);
		if (IS_ERR(omap_mcbsp_ick[i])) {
			printk(KERN_ERR "mcbsp[%d] could not \
						acquire ick handle\n",i+1);
			return PTR_ERR(omap_mcbsp_ick[i]);
		}

		omap_mcbsp_clk[i].fck = clk_get(0, omap_mcbsp_fck[i]);
		if (IS_ERR(omap_mcbsp_fck[i])) {
			printk(KERN_ERR "mcbsp[%d] could not \
						acquire fck handle\n",i+1);
			return PTR_ERR(omap_mcbsp_fck[i]);
		}
	}

#endif

#ifdef CONFIG_ARCH_OMAP730
	if (cpu_is_omap730()) {
		mcbsp_info = mcbsp_730;
		mcbsp_count = ARRAY_SIZE(mcbsp_730);
	}
#endif
#ifdef CONFIG_ARCH_OMAP15XX
	if (cpu_is_omap15xx()) {
		mcbsp_info = mcbsp_1510;
		mcbsp_count = ARRAY_SIZE(mcbsp_1510);
	}
#endif
#if defined(CONFIG_ARCH_OMAP16XX)
	if (cpu_is_omap16xx()) {
		mcbsp_info = mcbsp_1610;
		mcbsp_count = ARRAY_SIZE(mcbsp_1610);
	}
#endif
#if defined(CONFIG_ARCH_OMAP2420)
	if (cpu_is_omap2420()) {
		mcbsp_info = mcbsp_24xx;
		mcbsp_count = ARRAY_SIZE(mcbsp_24xx);
		omap2_mcbsp2_mux_setup();
	}
#endif
#if  defined(CONFIG_ARCH_OMAP2430)
	if (cpu_is_omap2430()) {
		mcbsp_info = mcbsp_2430;
		mcbsp_count = ARRAY_SIZE(mcbsp_2430);
	}
#endif
#if defined(CONFIG_ARCH_OMAP34XX)
	if (cpu_is_omap34xx()) {
		mcbsp_info = mcbsp_34xx;
		mcbsp_count = ARRAY_SIZE(mcbsp_34xx);
	}
#endif

	for (i = 0; i < OMAP_MAX_MCBSP_COUNT ; i++) {
		if (i >= mcbsp_count) {
			mcbsp[i].io_base = 0;
			mcbsp[i].free = 0;
			continue;
		}
		mcbsp[i].id = i + 1;
		mcbsp[i].free = 1;
		mcbsp[i].dma_tx_lch = -1;
		mcbsp[i].dma_rx_lch = -1;

		mcbsp[i].io_base = mcbsp_info[i].virt_base;
		mcbsp[i].phy_base = mcbsp_info[i].phy_base;
		/* Default I/O is IRQ based */
		mcbsp[i].io_type = OMAP_MCBSP_IRQ_IO;
		mcbsp[i].tx_irq = mcbsp_info[i].tx_irq;
		mcbsp[i].rx_irq = mcbsp_info[i].rx_irq;
		mcbsp[i].dma_rx_sync = mcbsp_info[i].dma_rx_sync;
		mcbsp[i].dma_tx_sync = mcbsp_info[i].dma_tx_sync;
		mcbsp[i].srg_enabled = 0;
		spin_lock_init(&mcbsp[i].lock);
	}

	return 0;
}

arch_initcall(omap_mcbsp_init);
