/*
 * linux/arch/arm/mach-omap2/mcbsp.c
 *
 * Copyright (C) 2008 Instituto Nokia de Tecnologia
 * Contact: Eduardo Valentin <eduardo.valentin@indt.org.br>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Multichannel mode not supported.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/wait.h>

#include <asm/arch/dma.h>
#include <asm/arch/mux.h>
#include <asm/arch/cpu.h>
#include <asm/arch/mcbsp.h>
/* No. of dma channels in a chain, setting 2 for each interface */
static int omap_mcbsp_max_dmachs_rx[OMAP_MAX_MCBSP_COUNT] = {2, 2, 2, 2, 2};
static int omap_mcbsp_max_dmachs_tx[OMAP_MAX_MCBSP_COUNT] = {2, 2, 2, 2, 2};

static int omap2_mcbsp_drr[OMAP_MAX_MCBSP_COUNT] =
{
	OMAP34XX_MCBSP1_BASE + OMAP_MCBSP_REG_DRR,
	OMAP34XX_MCBSP2_BASE + OMAP_MCBSP_REG_DRR,
	OMAP34XX_MCBSP3_BASE + OMAP_MCBSP_REG_DRR,
	OMAP34XX_MCBSP4_BASE + OMAP_MCBSP_REG_DRR,
	OMAP34XX_MCBSP5_BASE + OMAP_MCBSP_REG_DRR,
};

static int omap2_mcbsp_dxr[OMAP_MAX_MCBSP_COUNT] =
{
	OMAP34XX_MCBSP1_BASE + OMAP_MCBSP_REG_DXR,
	OMAP34XX_MCBSP2_BASE + OMAP_MCBSP_REG_DXR,
	OMAP34XX_MCBSP3_BASE + OMAP_MCBSP_REG_DXR,
	OMAP34XX_MCBSP4_BASE + OMAP_MCBSP_REG_DXR,
	OMAP34XX_MCBSP5_BASE + OMAP_MCBSP_REG_DXR,
};
struct mcbsp_internal_clk {
	struct clk clk;
	struct clk **childs;
	int n_childs;
};

/*
 * omap_mcbsp_config simply write a config to the
 * appropriate McBSP.
 * You either call this function or set the McBSP registers
 * by yourself before calling omap_mcbsp_start().
 */
void omap2_mcbsp_config(unsigned int id,
			 const struct omap_mcbsp_reg_cfg *config)
{
	u32 io_base;
	io_base = mcbsp[id].io_base;

	omap_mcbsp_config(id, config);
	if (!cpu_is_omap2420()) {
		OMAP_MCBSP_WRITE(io_base, RCCR, config->rccr);
		OMAP_MCBSP_WRITE(io_base, XCCR, config->xccr);
	}
	return;
}

#if defined(CONFIG_ARCH_OMAP24XX) || defined(CONFIG_ARCH_OMAP34XX)
static void omap_mcbsp_clk_init(struct mcbsp_internal_clk *mclk)
{
	const char *clk_names[] = { "mcbsp_ick", "mcbsp_fck" };
	int i;

	mclk->n_childs = ARRAY_SIZE(clk_names);
	mclk->childs = kzalloc(mclk->n_childs * sizeof(struct clk *),
				GFP_KERNEL);

	for (i = 0; i < mclk->n_childs; i++) {
		/* We fake a platform device to get correct device id */
		struct platform_device pdev;

		pdev.dev.bus = &platform_bus_type;
		pdev.id = mclk->clk.id;
		mclk->childs[i] = clk_get(&pdev.dev, clk_names[i]);
		if (IS_ERR(mclk->childs[i]))
			printk(KERN_ERR "Could not get clock %s (%d).\n",
				clk_names[i], mclk->clk.id);
	}
}

static int omap_mcbsp_clk_enable(struct clk *clk)
{
	struct mcbsp_internal_clk *mclk = container_of(clk,
					struct mcbsp_internal_clk, clk);
	int i;

	for (i = 0; i < mclk->n_childs; i++)
		clk_enable(mclk->childs[i]);
	return 0;
}

static void omap_mcbsp_clk_disable(struct clk *clk)
{
	struct mcbsp_internal_clk *mclk = container_of(clk,
					struct mcbsp_internal_clk, clk);
	int i;

	for (i = 0; i < mclk->n_childs; i++)
		clk_disable(mclk->childs[i]);
}

static struct mcbsp_internal_clk omap_mcbsp_clks[] = {
	{
		.clk = {
			.name 		= "mcbsp_clk",
			.id		= 1,
			.enable		= omap_mcbsp_clk_enable,
			.disable	= omap_mcbsp_clk_disable,
		},
	},
	{
		.clk = {
			.name 		= "mcbsp_clk",
			.id		= 2,
			.enable		= omap_mcbsp_clk_enable,
			.disable	= omap_mcbsp_clk_disable,
		},
	},
	{
		.clk = {
			.name 		= "mcbsp_clk",
			.id		= 3,
			.enable		= omap_mcbsp_clk_enable,
			.disable	= omap_mcbsp_clk_disable,
		},
	},
	{
		.clk = {
			.name 		= "mcbsp_clk",
			.id		= 4,
			.enable		= omap_mcbsp_clk_enable,
			.disable	= omap_mcbsp_clk_disable,
		},
	},
	{
		.clk = {
			.name 		= "mcbsp_clk",
			.id		= 5,
			.enable		= omap_mcbsp_clk_enable,
			.disable	= omap_mcbsp_clk_disable,
		},
	},
};

#define omap_mcbsp_clks_size	ARRAY_SIZE(omap_mcbsp_clks)
#else
#define omap_mcbsp_clks_size	0
static struct mcbsp_internal_clk __initdata *omap_mcbsp_clks;
static inline void omap_mcbsp_clk_init(struct clk *clk)
{ }
#endif

static void omap2_mcbsp2_mux_setup(void)
{
	omap_cfg_reg(Y15_24XX_MCBSP2_CLKX);
	omap_cfg_reg(R14_24XX_MCBSP2_FSX);
	omap_cfg_reg(W15_24XX_MCBSP2_DR);
	omap_cfg_reg(V15_24XX_MCBSP2_DX);
	omap_cfg_reg(V14_24XX_GPIO117);
	/*
	 * TODO: Need to add MUX settings for OMAP 2430 SDP
	 */
}

static void omap2_mcbsp_request(unsigned int id)
{
	if (cpu_is_omap2420() && (id == OMAP_MCBSP2))
		omap2_mcbsp2_mux_setup();
}

static int omap2_mcbsp_check(unsigned int id)
{
	if (id > OMAP_MAX_MCBSP_COUNT - 1) {
		printk(KERN_ERR "OMAP-McBSP: McBSP%d doesn't exist\n", id + 1);
		return -ENODEV;
	}
	return 0;
}

static void omap2_mcbsp_free(unsigned int id)
{
	if (mcbsp[id].dma_rx_lch != -1) {
		omap_free_dma_chain(mcbsp[id].dma_rx_lch);
		 mcbsp[id].dma_rx_lch = -1;
	}

	if (mcbsp[id].dma_tx_lch != -1) {
		omap_free_dma_chain(mcbsp[id].dma_tx_lch);
		mcbsp[id].dma_tx_lch = -1;
	}
	return;
}

static struct omap_mcbsp_ops omap2_mcbsp_ops = {
	.request	= omap2_mcbsp_request,
	.check		= omap2_mcbsp_check,
	.free 		= omap2_mcbsp_free,
};

static void omap2_mcbsp_rx_dma_callback(int lch, u16 ch_status, void *data)
{
	struct omap_mcbsp *mcbsp_dma_rx = data;
	u32 io_base;
	io_base = mcbsp_dma_rx->io_base;

	/* If we are at the last transfer, Shut down the reciever */
	if ((mcbsp_dma_rx->auto_reset & OMAP_MCBSP_AUTO_RRST)
		&& (omap_dma_chain_status(mcbsp_dma_rx->dma_rx_lch) ==
						 OMAP_DMA_CHAIN_INACTIVE))
			OMAP_MCBSP_WRITE(io_base, SPCR1,
				OMAP_MCBSP_READ(io_base, SPCR1) & (~RRST));

	if (mcbsp_dma_rx->rx_callback != NULL)
		mcbsp_dma_rx->rx_callback(ch_status, mcbsp_dma_rx->rx_cb_arg);

}

static void omap2_mcbsp_tx_dma_callback(int lch, u16 ch_status, void *data)
{
	struct omap_mcbsp *mcbsp_dma_tx = data;
	u32 io_base;
	io_base = mcbsp_dma_tx->io_base;

	/* If we are at the last transfer, Shut down the Transmitter */
	if ((mcbsp_dma_tx->auto_reset & OMAP_MCBSP_AUTO_XRST)
		&& (omap_dma_chain_status(mcbsp_dma_tx->dma_tx_lch) ==
						 OMAP_DMA_CHAIN_INACTIVE))
			OMAP_MCBSP_WRITE(io_base, SPCR2,
				OMAP_MCBSP_READ(io_base, SPCR2) & (~XRST));

	if (mcbsp_dma_tx->tx_callback != NULL)
		mcbsp_dma_tx->tx_callback(ch_status, mcbsp_dma_tx->tx_cb_arg);
}




/*
 * Set McBSP recv parameters
 * id           : McBSP interface ID
 * mcbsp_cfg    : McBSP register configuration
 * rp           : McBSP recv parameters
 */
void omap2_mcbsp_set_recv_param(unsigned int id,
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
	return;

}


/*
 * Set McBSP transmit parameters
 * id		: McBSP interface ID
 * mcbsp_cfg	: McBSP register configuration
 * tp		: McBSP transmit parameters
 */

void omap2_mcbsp_set_trans_param(unsigned int id,
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
	return;

}
/*


 * Set McBSP SRG configuration
 * id			: McBSP interface ID
 * mcbsp_cfg		: McBSP register configuration
 * interface_mode	: Master/Slave
 * param		: McBSP SRG and FSG configuration
 */

void omap2_mcbsp_set_srg_cfg_param(unsigned int id, int interface_mode,
					struct omap_mcbsp_reg_cfg *mcbsp_cfg,
					struct omap_mcbsp_srg_fsg_cfg *param)
{
	u32 io_base;
	u32 clk_rate, clkgdv;
	io_base = mcbsp[id].io_base;

	mcbsp[id].interface_mode = interface_mode;
	mcbsp_cfg->srgr1 = FWID(param->pulse_width);

	if (interface_mode == OMAP_MCBSP_MASTER) {
		/* clk_rate = clk_get_rate(omap_mcbsp_clk[id].fck); */
		clk_rate = 96000000;
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

	return;

}


/*
 * configure the McBSP registers
 * id			: McBSP interface ID
 * interface_mode	: Master/Slave
 * rp			: McBSP recv parameters
 * tp			: McBSP transmit parameters
 * param		: McBSP SRG and FSG configuration
 */
int omap2_mcbsp_params_cfg(unsigned int id, int interface_mode,
				struct omap_mcbsp_cfg_param *rp,
				struct omap_mcbsp_cfg_param *tp,
				struct omap_mcbsp_srg_fsg_cfg *param)
{
	struct omap_mcbsp_reg_cfg mcbsp_cfg = {0};

	spin_lock(&mcbsp[id].lock);

	if (rp)
		omap2_mcbsp_set_recv_param(id, &mcbsp_cfg, rp);

	if (tp)
		omap2_mcbsp_set_trans_param(id, &mcbsp_cfg, tp);

	if (param)
		omap2_mcbsp_set_srg_cfg_param(id,
					interface_mode, &mcbsp_cfg, param);

	omap2_mcbsp_config(id, &mcbsp_cfg);

	spin_unlock(&mcbsp[id].lock);
	return (0);

}
EXPORT_SYMBOL(omap2_mcbsp_params_cfg);

/*
 * Enable/Disable the sample rate generator
 * id		: McBSP interface ID
 * state	: Enable/Disable
 */
int omap2_mcbsp_set_srg_fsg(unsigned int id, u8 state)
{
	u32 io_base;

	if (omap2_mcbsp_check(id) < 0)
		return -ENXIO;

	io_base = mcbsp[id].io_base;

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
	return (0);
}

/*
 * Stop transmitting data on a McBSP interface
 * id		: McBSP interface ID
 */
int omap2_mcbsp_stop_datatx(unsigned int id)
{
	u32 io_base;

	if (omap2_mcbsp_check(id) < 0)
		return -ENXIO;
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
		omap2_mcbsp_set_srg_fsg(id, OMAP_MCBSP_DISABLE_FSG_SRG);
	spin_unlock(&mcbsp[id].lock);
	return 0;
}
EXPORT_SYMBOL(omap2_mcbsp_stop_datatx);

/*
 * Stop receving data on a McBSP interface
 * id		: McBSP interface ID
 */
int omap2_mcbsp_stop_datarx(u32 id)
{

	u32 io_base;

	if (omap2_mcbsp_check(id) < 0)
		return -ENXIO;
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
		omap2_mcbsp_set_srg_fsg(id, OMAP_MCBSP_DISABLE_FSG_SRG);
	spin_unlock(&mcbsp[id].lock);
	return 0;
}
EXPORT_SYMBOL(omap2_mcbsp_stop_datarx);

/*
 * Interface Reset
 * id	: McBSP interface ID
 * Resets the McBSP interface
 */
int omap2_mcbsp_reset(unsigned int id)
{
	u32 io_base;
	int counter = 0;
	int wait_for_reset = 10000;

	if (omap2_mcbsp_check(id) < 0)
		return -ENXIO;

	io_base = mcbsp[id].io_base;

	OMAP_MCBSP_WRITE(io_base, SYSCON,
		OMAP_MCBSP_READ(io_base, SYSCON) | (SOFTRST));

	while (OMAP_MCBSP_READ(io_base, SYSCON) & SOFTRST) {
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
	return 0;
}
EXPORT_SYMBOL(omap2_mcbsp_reset);

/*
 * Get the element index and frame index of transmitter
 * id		: McBSP interface ID
 * ei		: element index
 * fi		: frame index
 */
int omap2_mcbsp_transmitter_index(int id, int *ei, int *fi)
{
	int eix = 0, fix = 0;

	if (omap2_mcbsp_check(id) < 0)
		return -ENXIO;

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
EXPORT_SYMBOL(omap2_mcbsp_transmitter_index);

/*
 * Get the element index and frame index of receiver
 * id	: McBSP interface ID
 * ei		: element index
 * fi		: frame index
 */
int omap2_mcbsp_receiver_index(int id, int *ei, int *fi)
{
	int eix = 0, fix = 0;

	if (omap2_mcbsp_check(id) < 0)
		return -ENXIO;

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
EXPORT_SYMBOL(omap2_mcbsp_receiver_index);

/*
 * Basic Reset Transmitter
 * id		: McBSP interface number
 * state	: Disable (0)/ Enable (1) the transmitter
 */
int omap2_mcbsp_set_xrst(unsigned int id, u8 state)
{
	u32 io_base;
	if (omap2_mcbsp_check(id) < 0)
		return -ENXIO;

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
EXPORT_SYMBOL(omap2_mcbsp_set_xrst);

/*
 * Reset Receiver
 * id		: McBSP interface number
 * state	: Disable (0)/ Enable (1) the receiver
 */
int omap2_mcbsp_set_rrst(unsigned int id, u8 state)
{
	u32 io_base;
	if (omap2_mcbsp_check(id) < 0)
		return -ENXIO;

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
EXPORT_SYMBOL(omap2_mcbsp_set_rrst);

/*
 * Configure the receiver parameters
 * id		: McBSP Interface ID
 * rp		: DMA Receive parameters
 */
int omap2_mcbsp_dma_recv_params(unsigned int id,
				omap_mcbsp_dma_transfer_params *rp)
{
	u32 io_base;
	int err, chain_id = -1;
	struct omap_dma_channel_params rx_params;
	u32  dt = 0;

	if (omap2_mcbsp_check(id) < 0)
		return -EINVAL;
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
					 omap2_mcbsp_rx_dma_callback, &chain_id,
					 omap_mcbsp_max_dmachs_rx[id],
					 OMAP_DMA_DYNAMIC_CHAIN, rx_params);
		if (err < 0) {
			printk(KERN_ERR "Receive path configuration failed \n");
			return -EPERM;
		}
		mcbsp[id].dma_rx_lch = chain_id;
		mcbsp[id].rx_dma_chain_state = 0;
	} else {
		/* DMA params already set, modify the same!! */
		err = omap_modify_dma_chain_params(mcbsp[id].
						 dma_rx_lch, rx_params);
		if (err < 0)
			return -EPERM;
	}

	return 0;
}
EXPORT_SYMBOL(omap2_mcbsp_dma_recv_params);

/*
 * Configure the transmitter parameters
 * id		: McBSP Interface ID
 * tp		: DMA Transfer parameters
 */

int omap2_mcbsp_dma_trans_params(unsigned int id,
				 omap_mcbsp_dma_transfer_params *tp)
{

	struct omap_dma_channel_params tx_params;
	int err = 0, chain_id = -1;
	u32 io_base;
	u32 dt = 0;

	if (omap2_mcbsp_check(id) < 0)
		return -EINVAL;

	io_base = mcbsp[id].io_base;

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
					 omap2_mcbsp_tx_dma_callback, &chain_id,
					 omap_mcbsp_max_dmachs_tx[id],
					 OMAP_DMA_DYNAMIC_CHAIN, tx_params);
		if (err < 0) {
			printk(KERN_ERR
				"Transmit path configuration failed \n");
			return -EPERM;
		}
		mcbsp[id].tx_dma_chain_state = 0;
	mcbsp[id].dma_tx_lch = chain_id;
	} else {
		/* DMA params already set, modify the same!! */
		err = omap_modify_dma_chain_params(mcbsp[id].
						 dma_tx_lch, tx_params);
		if (err < 0)
			return -EPERM;
	}

	return 0;
}
EXPORT_SYMBOL(omap2_mcbsp_dma_trans_params);

/*
 * Start receving data on a McBSP interface
 * id			: McBSP interface ID
 * cbdata		: User data to be returned with callback
 * buf_start_addr	: The destination address [physical address]
 * buf_size		: Buffer size
 */

int omap2_mcbsp_receive_data(unsigned int id, void *cbdata,
			     dma_addr_t buf_start_addr, u32 buf_size)
{
	u32 dma_chain_status = 0;
	u32 io_base;
	int enable_rx = 0;
	int e_count = 0;
	int f_count = 0;

	if (omap2_mcbsp_check(id) < 0)
		return -EINVAL;

	spin_lock(&mcbsp[id].lock);
	io_base = mcbsp[id].io_base;

	mcbsp[id].rx_cb_arg = cbdata;

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
				omap2_mcbsp_drr[id],
				 buf_start_addr, e_count, f_count, &mcbsp[id]);

	if (mcbsp[id].rx_dma_chain_state == 0) {
		if (mcbsp[id].interface_mode == OMAP_MCBSP_MASTER) {
			omap2_mcbsp_set_srg_fsg(id, OMAP_MCBSP_ENABLE_FSG_SRG);
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
EXPORT_SYMBOL(omap2_mcbsp_receive_data);

/*
 * Start transmitting data through a McBSP interface
 * id			: McBSP interface ID
 * cbdata		: User data to be returned with callback
 * buf_start_addr	: The source address [This should be physical address]
 * buf_size		: Buffer size
 */
int omap2_mcbsp_send_data(unsigned int id, void *cbdata,
			  dma_addr_t buf_start_addr, u32 buf_size)
{
	u32 io_base;
	u32 dma_chain_state = 0;
	u8 enable_tx = 0;
	int e_count = 0;
	int f_count = 0;

	/* Check if mcbsp interface is valid and is reserved */
	if (omap2_mcbsp_check(id) < 0)
		return -EINVAL;

	spin_lock(&mcbsp[id].lock);
	io_base = mcbsp[id].io_base;

	mcbsp[id].tx_cb_arg = cbdata;

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
					omap2_mcbsp_dxr[id],
					e_count, f_count, &mcbsp[id]);

	if (mcbsp[id].tx_dma_chain_state == 0) {
		if (mcbsp[id].interface_mode == OMAP_MCBSP_MASTER) {
			omap2_mcbsp_set_srg_fsg(id, OMAP_MCBSP_ENABLE_FSG_SRG);
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
EXPORT_SYMBOL(omap2_mcbsp_send_data);

#ifdef CONFIG_ARCH_OMAP24XX
static struct omap_mcbsp_platform_data omap24xx_mcbsp_pdata[] = {
	{
		.virt_base	= IO_ADDRESS(OMAP24XX_MCBSP1_BASE),
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP1_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP1_TX,
		.rx_irq		= INT_24XX_MCBSP1_IRQ_RX,
		.tx_irq		= INT_24XX_MCBSP1_IRQ_TX,
		.ops		= &omap2_mcbsp_ops,
		.clk_name	= "mcbsp_clk",
	},
	{
		.virt_base	= IO_ADDRESS(OMAP24XX_MCBSP2_BASE),
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP2_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP2_TX,
		.rx_irq		= INT_24XX_MCBSP2_IRQ_RX,
		.tx_irq		= INT_24XX_MCBSP2_IRQ_TX,
		.ops		= &omap2_mcbsp_ops,
		.clk_name	= "mcbsp_clk",
	},
};
#define OMAP24XX_MCBSP_PDATA_SZ		ARRAY_SIZE(omap24xx_mcbsp_pdata)
#else
#define omap24xx_mcbsp_pdata		NULL
#define OMAP24XX_MCBSP_PDATA_SZ		0
#endif

#ifdef CONFIG_ARCH_OMAP34XX
static struct omap_mcbsp_platform_data omap34xx_mcbsp_pdata[] = {
	{
		.virt_base	= IO_ADDRESS(OMAP34XX_MCBSP1_BASE),
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP1_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP1_TX,
		.ops		= &omap2_mcbsp_ops,
		.clk_name	= "mcbsp_clk",
	},
	{
		.virt_base	= IO_ADDRESS(OMAP34XX_MCBSP2_BASE),
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP2_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP2_TX,
		.ops		= &omap2_mcbsp_ops,
		.clk_name	= "mcbsp_clk",
	},
	{
		.virt_base	= IO_ADDRESS(OMAP34XX_MCBSP3_BASE),
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP3_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP3_TX,
		.ops		= &omap2_mcbsp_ops,
		.clk_name	= "mcbsp_clk",
	},
	{
		.virt_base	= IO_ADDRESS(OMAP34XX_MCBSP4_BASE),
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP4_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP4_TX,
		.ops		= &omap2_mcbsp_ops,
		.clk_name	= "mcbsp_clk",
	},
	{
		.virt_base	= IO_ADDRESS(OMAP34XX_MCBSP5_BASE),
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP5_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP5_TX,
		.ops		= &omap2_mcbsp_ops,
		.clk_name	= "mcbsp_clk",
	},
};

#define OMAP34XX_MCBSP_PDATA_SZ		ARRAY_SIZE(omap34xx_mcbsp_pdata)
#else
#define omap34xx_mcbsp_pdata		NULL
#define OMAP34XX_MCBSP_PDATA_SZ		0
#endif

int __init omap2_mcbsp_init(void)
{
	int i;

	for (i = 0; i < omap_mcbsp_clks_size; i++) {
		/* Once we call clk_get inside init, we do not register it */
		omap_mcbsp_clk_init(&omap_mcbsp_clks[i]);
		clk_register(&omap_mcbsp_clks[i].clk);
	}

	if (cpu_is_omap24xx())
		omap_mcbsp_register_board_cfg(omap24xx_mcbsp_pdata,
						OMAP24XX_MCBSP_PDATA_SZ);

	if (cpu_is_omap34xx())
		omap_mcbsp_register_board_cfg(omap34xx_mcbsp_pdata,
						OMAP34XX_MCBSP_PDATA_SZ);

	return omap_mcbsp_init();
}
arch_initcall(omap2_mcbsp_init);
