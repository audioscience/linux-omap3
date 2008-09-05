/*
 * arch/arm/plat-omap2/omap-dss.c
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Leveraged code from the OMAP24xx camera driver
 * Video-for-Linux (Version 2) camera capture driver for
 * the OMAP24xx camera controller.
 *
 * Author: Andy Lowe (source@mvista.com)
 * Copyright (C) 2004 MontaVista Software, Inc.
 *
 * History:
 * 20-APR-2006  Khasim		Modified VRFB based Rotation equations,
 *				The image data is always read from 0 degree
 *				view and written to the virtual space of desired
 *				rotation angle
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <asm/system.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <asm/arch/hardware.h>
#include <asm/arch/omap-dss.h>
#include <asm/arch/clock.h>
#ifdef CONFIG_TRACK_RESOURCES
#include <linux/device.h>
#endif

#undef DEBUG

#ifdef DEBUG
#define DEBUGP printk
#else
#define DEBUGP(fmt, a...)
#endif

/* TODO This is a power management macro.  Currently not defined */
#define CONFIG_OMAP34XX_OFFMODE

/* usage count for DSS power management */
static int disp_usage;
static spinlock_t dss_lock;
short int current_colorconv_values[2][3][3];
EXPORT_SYMBOL(current_colorconv_values);
static struct omap_dss_regs dss_ctx;

static struct clk *dss1f_scale;
static struct clk *dss1f, *dss1i;
static int m_clk_rate = 24000000 * 4;
#if defined(CONFIG_OMAP_USE_DSI_PLL) || defined(CONFIG_OMAP_DSI)
static struct clk *dss2f;
#endif

struct omap_disp_dma_params {
	u32 ba0;
	u32 ba1;
	int row_inc;
	int pix_inc;
};

static struct layer_t {
	int output_dev;
	int in_use;
	int ctx_valid;

	/* one set of dma parameters each for LCD and TV */
	struct omap_disp_dma_params dma[2];

	int size_x;
	int size_y;
} layer[DSS_CTX_NUMBER] = {
	{
	.ctx_valid = 0,}, {
	.ctx_valid = 0,}, {
	.ctx_valid = 0,}, {
	.ctx_valid = 0,}, {
.ctx_valid = 0,},};

#define MAX_ISR_NR   8
static int omap_disp_irq;
static struct {
	omap_disp_isr_t isr;
	void *arg;
	unsigned int mask;
} registered_isr[MAX_ISR_NR];

/* Required function delcalarations */
static void omap_disp_restore_ctx(int ltype);
static void disp_save_ctx(int ltype);

/*
 * Modes and Encoders supported by DSS
 */
struct channel_obj channels[] = {
	{0, 0, {NULL, NULL, NULL}, 0, 0},
#ifndef CONFIG_ARCH_OMAP3410
	{0, 0, {NULL, NULL, NULL}, 0, 0}
#endif
};

/* This mode structure lists all the modes supported by DSS
 */
struct omap_mode_info modes[] = {
	{"ntsc_m", 720, 482, 0, 0, 0, 0, 0, 0, 0, 0, NULL},
	{"ntsc_j", 720, 482, 0, 0, 0, 0, 0, 0, 0, 0, NULL},
	{"ntsc_443", 720, 482, 0, 0, 0, 0, 0, 0, 0, 0, NULL},
	{"pal_bdghi", 720, 574, 0, 0, 0, 0, 0, 0, 0, 0, NULL},
	{"pal_nc", 720, 574, 0, 0, 0, 0, 0, 0, 0, 0, NULL},
	{"pal_n", 720, 574, 0, 0, 0, 0, 0, 0, 0, 0, NULL},
	{"pal_m", 720, 482, 0, 0, 0, 0, 0, 0, 0, 0, NULL},
	{"pal_60", 720, 482, 0, 0, 0, 0, 0, 0, 0, 0, NULL}
};

#ifdef CONFIG_TRACK_RESOURCES
/* device name needed for resource tracking layer */
struct device_driver display_drv = {
	.name = "display",
};
struct device display_dev = {
	.driver = &display_drv,
};
#endif
/*
 * DSS register I/O routines
 */
static inline u32 dss_reg_in(u32 offset)
{
	return omap_readl(DSS_REG_BASE + DSS_REG_OFFSET + offset);
}
static inline u32 dss_reg_out(u32 offset, u32 val)
{
	omap_writel(val, DSS_REG_BASE + DSS_REG_OFFSET + offset);
	return val;
}
static inline u32 dss_reg_merge(u32 offset, u32 val, u32 mask)
{
	u32 addr = DSS_REG_BASE + DSS_REG_OFFSET + offset;
	u32 new_val = (omap_readl(addr) & ~mask) | (val & mask);

	omap_writel(new_val, addr);
	return new_val;
}

/*
 * Display controller register I/O routines
 */
static inline u32 dispc_reg_in(u32 offset)
{
	return omap_readl(DSS_REG_BASE + DISPC_REG_OFFSET + offset);
}
static inline u32 dispc_reg_out(u32 offset, u32 val)
{
	omap_writel(val, DSS_REG_BASE + DISPC_REG_OFFSET + offset);
	return val;
}
static inline u32 dispc_reg_merge(u32 offset, u32 val, u32 mask)
{
	u32 addr = DSS_REG_BASE + DISPC_REG_OFFSET + offset;
	u32 new_val = (omap_readl(addr) & ~mask) | (val & mask);

	omap_writel(new_val, addr);
	return new_val;
}

/*
 * RFBI controller register I/O routines
 */
static inline u32 rfbi_reg_in(u32 offset)
{
	return omap_readl(DSS_REG_BASE + RFBI_REG_OFFSET + offset);
}
static inline u32 rfbi_reg_out(u32 offset, u32 val)
{
	omap_writel(val, DSS_REG_BASE + RFBI_REG_OFFSET + offset);
	return val;
}

/*
 * DSI Proto Engine register I/O routines
  */
static inline u32 dsiproto_reg_in(u32 offset)
{
	u32 val;
	val = omap_readl(DSI_PROTO_ENG_REG_BASE + offset);
	return val;
}
static inline u32 dsiproto_reg_out(u32 offset, u32 val)
{
	omap_writel(val, DSI_PROTO_ENG_REG_BASE + offset);
	return val;
}

/*
 * DSI PLL register I/O routines
 */
static inline u32 dsipll_reg_in(u32 offset)
{
	u32 val;
	val = omap_readl(DSI_PLL_CONTROLLER_REG_BASE + offset);
	return val;
}
static inline u32 dsipll_reg_out(u32 offset, u32 val)
{
	omap_writel(val, DSI_PLL_CONTROLLER_REG_BASE + offset);
	return val;
}

/*---------------------------------------------------------------------------*/
/* Local Helper Functions */

/* DSS Interrupt master service routine. */
static irqreturn_t
omap_disp_master_isr(int irq, void *arg, struct pt_regs *regs)
{
	unsigned long dispc_irqstatus = dispc_reg_in(DISPC_IRQSTATUS);
	int i;

	for (i = 0; i < MAX_ISR_NR; i++) {
		if (registered_isr[i].isr == NULL)
			continue;
		if (registered_isr[i].mask & dispc_irqstatus)
			registered_isr[i].isr(registered_isr[i].arg, regs,
					      dispc_irqstatus);
	}
	/* ack the interrupt */
	dispc_reg_out(DISPC_IRQSTATUS, dispc_irqstatus);
	return IRQ_HANDLED;
}

/*
 * Sync Lost interrupt handler
 */
static void
disp_synclost_isr(void *arg, struct pt_regs *regs, u32 irqstatus)
{
	u32 i;

#ifndef CONFIG_ARCH_OMAP3410
	struct omap_encoder_device *enc_dev;
#endif
	i = 0;
	printk(KERN_WARNING "Sync Lost %x\n",
	       dispc_reg_in(DISPC_IRQSTATUS));
	arg = NULL;
	regs = NULL;

	/*
	 * Disable and Clear all the interrupts before we start
	 */
	dispc_reg_out(DISPC_IRQENABLE, 0x00000000);
	dispc_reg_out(DISPC_IRQSTATUS, 0x0000FFFF);

	/* disable the display controller */
	omap_disp_disable(HZ / 2);

	/*
	 * Update the state of the display controller.
	 */
	dss_ctx.dispc.sysconfig &= ~DISPC_SYSCONFIG_SOFTRESET;
	dss_ctx.dispc.control &= ~(DISPC_CONTROL_GODIGITAL);

	dispc_reg_out(DISPC_SYSCONFIG, DISPC_SYSCONFIG_SOFTRESET);
	while (!(dispc_reg_in(DISPC_SYSSTATUS) & DISPC_SYSSTATUS_RESETDONE)) {
		udelay(100);
		if (i++ > 5) {
			printk(KERN_WARNING
			       "Failed to soft reset the DSS !! \n");
			break;
		}
	}

	/* Configure the encoders for the default standard */
	for (i = 0; i < ARRAY_SIZE(channels); i++) {
		enc_dev = channels[i].enc_devices[channels[i].
			current_encoder];
		if (enc_dev && enc_dev->mode_ops->setmode)
			enc_dev->mode_ops->setmode(modes[channels[i].
				current_mode].name, enc_dev);
	}
	/* Restore the registers */
	omap_disp_restore_ctx(OMAP_DSS_DISPC_GENERIC);
	omap_disp_restore_ctx(OMAP_GRAPHICS);
	omap_disp_restore_ctx(OMAP_VIDEO1);
	omap_disp_restore_ctx(OMAP_VIDEO2);

	/* enable the display controller */
	if (layer[OMAP_DSS_DISPC_GENERIC].ctx_valid)
		dispc_reg_out(DISPC_CONTROL, dss_ctx.dispc.control);

	omap_disp_reg_sync(OMAP_OUTPUT_TV);

}

/*
 * Save the DSS state before doing a GO LCD/DIGITAL
 */
static void disp_save_ctx(int ltype)
{
	int v1 = 0, v2 = 1;
	struct omap_dispc_regs *dispc = &dss_ctx.dispc;

	switch (ltype) {
	case OMAP_DSS_GENERIC:
		dss_ctx.sysconfig = dss_reg_in(DSS_SYSCONFIG);
		dss_ctx.control = dss_reg_in(DSS_CONTROL);
#ifdef CONFIG_ARCH_OMAP3430
		dss_ctx.sdi_control = dss_reg_in(DSS_SDI_CONTROL);
		dss_ctx.pll_control = dss_reg_in(DSS_PLL_CONTROL);
#endif
		break;

	case OMAP_DSS_DISPC_GENERIC:
		dispc->revision = dispc_reg_in(DISPC_REVISION);
		dispc->sysconfig = dispc_reg_in(DISPC_SYSCONFIG);
		dispc->sysstatus = dispc_reg_in(DISPC_SYSSTATUS);
		dispc->irqstatus = dispc_reg_in(DISPC_IRQSTATUS);
		dispc->irqenable = dispc_reg_in(DISPC_IRQENABLE);
		dispc->control = dispc_reg_in(DISPC_CONTROL);
		dispc->config = dispc_reg_in(DISPC_CONFIG);
		dispc->capable = dispc_reg_in(DISPC_CAPABLE);
		dispc->default_color0 = dispc_reg_in(DISPC_DEFAULT_COLOR0);
		dispc->default_color1 = dispc_reg_in(DISPC_DEFAULT_COLOR1);
		dispc->trans_color0 = dispc_reg_in(DISPC_TRANS_COLOR0);
		dispc->trans_color1 = dispc_reg_in(DISPC_TRANS_COLOR1);
		dispc->line_status = dispc_reg_in(DISPC_LINE_STATUS);
		dispc->line_number = dispc_reg_in(DISPC_LINE_NUMBER);
		dispc->data_cycle1 = dispc_reg_in(DISPC_DATA_CYCLE1);
		dispc->data_cycle2 = dispc_reg_in(DISPC_DATA_CYCLE2);
		dispc->data_cycle3 = dispc_reg_in(DISPC_DATA_CYCLE3);
		dispc->timing_h = dispc_reg_in(DISPC_TIMING_H);
		dispc->timing_v = dispc_reg_in(DISPC_TIMING_V);
		dispc->pol_freq = dispc_reg_in(DISPC_POL_FREQ);
		dispc->divisor = dispc_reg_in(DISPC_DIVISOR);
		dispc->global_alpha = dispc_reg_in(DISPC_GLOBAL_ALPHA);
		dispc->size_lcd = dispc_reg_in(DISPC_SIZE_LCD);
		dispc->size_dig = dispc_reg_in(DISPC_SIZE_DIG);

	case OMAP_VIDEO1:
		dispc->vid1_ba0 = dispc_reg_in(DISPC_VID_BA0(v1));
		dispc->vid1_ba1 = dispc_reg_in(DISPC_VID_BA0(v1));
		dispc->vid1_position =
		    dispc_reg_in(DISPC_VID_POSITION(v1));
		dispc->vid1_size = dispc_reg_in(DISPC_VID_SIZE(v1));
		dispc->vid1_attributes =
		    dispc_reg_in(DISPC_VID_ATTRIBUTES(v1));
		dispc->vid1_fifo_size =
		    dispc_reg_in(DISPC_VID_FIFO_SIZE(v1));
		dispc->vid1_fifo_threshold =
		    dispc_reg_in(DISPC_VID_FIFO_THRESHOLD(v1));
		dispc->vid1_row_inc = dispc_reg_in(DISPC_VID_ROW_INC(v1));
		dispc->vid1_pixel_inc =
		    dispc_reg_in(DISPC_VID_PIXEL_INC(v1));
		dispc->vid1_fir = dispc_reg_in(DISPC_VID_FIR(v1));
		dispc->vid1_accu0 = dispc_reg_in(DISPC_VID_ACCU0(v1));
		dispc->vid1_accu1 = dispc_reg_in(DISPC_VID_ACCU1(v1));
		dispc->vid1_picture_size =
		    dispc_reg_in(DISPC_VID_PICTURE_SIZE(v1));
		dispc->vid1_fir_coef_h0 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 0));
		dispc->vid1_fir_coef_h1 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 1));
		dispc->vid1_fir_coef_h2 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 2));
		dispc->vid1_fir_coef_h3 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 3));
		dispc->vid1_fir_coef_h4 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 4));
		dispc->vid1_fir_coef_h5 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 5));
		dispc->vid1_fir_coef_h6 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 6));
		dispc->vid1_fir_coef_h7 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 7));
		dispc->vid1_fir_coef_hv0 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 0));
		dispc->vid1_fir_coef_hv1 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 1));
		dispc->vid1_fir_coef_hv2 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 2));
		dispc->vid1_fir_coef_hv3 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 3));
		dispc->vid1_fir_coef_hv4 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 4));
		dispc->vid1_fir_coef_hv5 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 5));
		dispc->vid1_fir_coef_hv6 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 6));
		dispc->vid1_fir_coef_hv7 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 7));
		dispc->vid1_conv_coef0 =
		    dispc_reg_in(DISPC_VID_CONV_COEF0(v1));
		dispc->vid1_conv_coef1 =
		    dispc_reg_in(DISPC_VID_CONV_COEF1(v1));
		dispc->vid1_conv_coef2 =
		    dispc_reg_in(DISPC_VID_CONV_COEF2(v1));
		dispc->vid1_conv_coef3 =
		    dispc_reg_in(DISPC_VID_CONV_COEF3(v1));
		dispc->vid1_conv_coef4 =
		    dispc_reg_in(DISPC_VID_CONV_COEF4(v1));
		break;

	case OMAP_VIDEO2:
		dispc->vid2_ba0 = dispc_reg_in(DISPC_VID_BA0(v2));
		dispc->vid2_ba1 = dispc_reg_in(DISPC_VID_BA1(v2));
		dispc->vid2_position =
		    dispc_reg_in(DISPC_VID_POSITION(v2));
		dispc->vid2_size = dispc_reg_in(DISPC_VID_SIZE(v2));
		dispc->vid2_attributes =
		    dispc_reg_in(DISPC_VID_ATTRIBUTES(v2));
		dispc->vid2_fifo_size =
		    dispc_reg_in(DISPC_VID_FIFO_SIZE(v2));
		dispc->vid2_fifo_threshold =
		    dispc_reg_in(DISPC_VID_FIFO_THRESHOLD(v2));
		dispc->vid2_row_inc = dispc_reg_in(DISPC_VID_ROW_INC(v2));
		dispc->vid2_pixel_inc =
		    dispc_reg_in(DISPC_VID_PIXEL_INC(v2));
		dispc->vid2_fir = dispc_reg_in(DISPC_VID_FIR(v2));
		dispc->vid2_accu0 = dispc_reg_in(DISPC_VID_ACCU0(v2));
		dispc->vid2_accu1 = dispc_reg_in(DISPC_VID_ACCU1(v2));
		dispc->vid2_picture_size =
		    dispc_reg_in(DISPC_VID_PICTURE_SIZE(v2));
		dispc->vid2_fir_coef_h0 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 0));
		dispc->vid2_fir_coef_h1 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 1));
		dispc->vid2_fir_coef_h2 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 2));
		dispc->vid2_fir_coef_h3 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 3));
		dispc->vid2_fir_coef_h4 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 4));
		dispc->vid2_fir_coef_h5 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 5));
		dispc->vid2_fir_coef_h6 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 6));
		dispc->vid2_fir_coef_h7 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 7));
		dispc->vid2_fir_coef_hv0 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 0));
		dispc->vid2_fir_coef_hv1 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 1));
		dispc->vid2_fir_coef_hv2 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 2));
		dispc->vid2_fir_coef_hv3 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 3));
		dispc->vid2_fir_coef_hv4 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 4));
		dispc->vid2_fir_coef_hv5 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 5));
		dispc->vid2_fir_coef_hv6 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 6));
		dispc->vid2_fir_coef_hv7 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 7));
		dispc->vid2_conv_coef0 =
		    dispc_reg_in(DISPC_VID_CONV_COEF0(v2));
		dispc->vid2_conv_coef1 =
		    dispc_reg_in(DISPC_VID_CONV_COEF1(v2));
		dispc->vid2_conv_coef2 =
		    dispc_reg_in(DISPC_VID_CONV_COEF2(v2));
		dispc->vid2_conv_coef3 =
		    dispc_reg_in(DISPC_VID_CONV_COEF3(v2));
		dispc->vid2_conv_coef4 =
		    dispc_reg_in(DISPC_VID_CONV_COEF4(v2));
		break;
	}
	layer[ltype].ctx_valid = 1;
}

void config_disp_clocks(int sleep_state)
{
#ifdef CONFIG_TRACK_RESOURCES
	struct device *dev = &display_dev;
#else
	struct device *dev = NULL;
#endif
	static int start = 1;
	/*int (*clk_onoff)(struct clk *clk) = NULL; */
	if (start) {
#ifndef CONFIG_OMAP_USE_DSI_PLL
		omap_disp_set_dssfclk();
#endif
		dss1i = clk_get(dev, "dss_ick");
		dss1f =
		    clk_get(dev,
			    cpu_is_omap34xx()? "dss1_alwon_fck" :
			    "dss1_fck");
		if (IS_ERR(dss1i) || IS_ERR(dss1f)) {
			printk(KERN_WARNING
			       "Could not get DSS clocks  \n");
			return;
		}
#if defined(CONFIG_OMAP_USE_DSI_PLL) || defined(CONFIG_OMAP_DSI)
		dss2f = clk_get(dev, "dss2_fck");
		if (IS_ERR(dss2f)) {
			printk(KERN_WARNING "Could not get DSS2 FCLK\n");
			return;
		}
#endif
		start = 0;
	}
	if (sleep_state == 1) {
		clk_disable(dss1i);
		clk_disable(dss1f);
	} else {
		if (clk_enable(dss1i) != 0) {
			printk(KERN_WARNING "Unable to enable DSS ICLK\n");
			return;
		}
		if (clk_enable(dss1f) != 0) {
			printk(KERN_WARNING "Unable to enable DSS FCLK\n");
			return;
		}
#ifndef CONFIG_OMAP_USE_DSI_PLL
#ifdef CONFIG_OMAP_DSI
		if (clk_enable(dss2f) != 0) {
			printk(KERN_WARNING "Unable to enable DSS FCLK\n");
			return;
		}
#endif
#endif
	}
}

/* This function turns on/off the clocks needed for TV-out.
 *  - 2430SDP: Controls the dss_54m_fck
 *  - 3430SDP: Controls the dss_tv_fck
 *  - 3430LAB: Controls both dss_tv_fck and dss_96m_fck.
 *             By default Labrador turns off the 96MHz DAC clock for
 *             power saving reasons.
 */
#ifndef CONFIG_ARCH_OMAP3410
static void disp_ll_config_tv_clocks(int sleep_state)
{
	static int start = 1;
	static struct clk *tv_clk;
#ifdef CONFIG_MACH_OMAP_3430LABRADOR
	static struct clk *dac_clk;
#endif
	static int disabled;
	static int enabled;

	if (start) {
#ifdef CONFIG_MACH_OMAP_2430SDP
		tv_clk = clk_get(NULL, "dss_54m_fck");
#endif
#if defined(CONFIG_MACH_OMAP_3430SDP) ||  defined(CONFIG_MACH_OMAP3EVM) \
	|| defined(CONFIG_MACH_OMAP_3430LABRADOR)
		tv_clk = clk_get(NULL, "dss_tv_fck");
#endif
#if defined(CONFIG_MACH_OMAP_3430LABRADOR)
		dac_clk = clk_get(NULL, "dss_96m_fck");
		if (IS_ERR(dac_clk)) {
			printk(KERN_WARNING
			       "\n UNABLE to get dss 96MHz fclk \n");
			return;
		}
#endif
		if (IS_ERR(tv_clk)) {
			printk(KERN_WARNING
			       "\n UNABLE to get dss TV fclk \n");
			return;
		}
		start = 0;
	}

	if (sleep_state == 1) {
		if (disabled == 0) {
			clk_disable(tv_clk);
#if defined(CONFIG_MACH_OMAP_3430LABRADOR)
			clk_disable(dac_clk);
#endif
			disabled = 1;
		}
		enabled = 0;
	} else {
		if (enabled == 0) {
			if (clk_enable(tv_clk) != 0) {
				printk(KERN_WARNING
				       "\n UNABLE to enable dss TV fclk \n");
				return;
			}
#if defined(CONFIG_MACH_OMAP_3430LABRADOR)
			if (clk_enable(dac_clk) != 0) {
				printk(KERN_WARNING
				       "\n UNABLE to enable dss 96MHz fclk \n");
				return;
			}
#endif
			enabled = 1;
		}
		disabled = 0;
	}
}
#endif

/* Function used to find the VRFB Alignement */
static inline u32 pages_per_side(u32 img_side, u32 page_exp)
{
	/*  page_side = 2 ^ page_exp
	 * (page_side - 1) is added for rounding up
	 */
	return (u32) (img_side + (1 << page_exp) - 1) >> page_exp;
}

/* Update the color conversion matrix */
static void update_colorconv_mtx(int v, const short int mtx[3][3])
{
	int i, j;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			current_colorconv_values[v][i][j] = mtx[i][j];
}

/* Write the horizontal and vertical resizing coefficients to the display
 * controller registers.  Each coefficient is a signed 8-bit integer in the
 * range [-128, 127] except for the middle coefficient (vc[1][i] and hc[3][i])
 * which is an unsigned 8-bit integer in the range [0, 255].  The first index of
 * the matrix is the coefficient number (0 to 2 vertical or 0 to 4 horizontal)
 * and the second index is the phase (0 to 7).
 */
void disp_set_resize(int v, short int *vc, short int *hc, int v_scale_dir)
{
	int i;
	unsigned long reg;

	for (i = 0; i < 8; i++) {
		reg =
		    (*(hc + (8 * 0) + i) & 0xff) |
		    ((*(hc + (8 * 1) + i) & 0xff)
		     << 8)
		    | ((*(hc + (8 * 2) + i) & 0xff) << 16) |
		    ((*(hc + (8 * 3) + i) & 0xff) << 24);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v, i), reg);

		if (!v_scale_dir) {
			reg =
			    (*(hc + (8 * 4) + i) & 0xff) |
			    ((*(vc + (1 * 8) + i) & 0xff) << 8)
			    | ((*(vc + (8 * 2) + i) & 0xff) << 16)
			    | ((*(vc + (3 * 8) + i) & 0xff) << 24);
			dispc_reg_out(DISPC_VID_FIR_COEF_HV(v, i), reg);

			reg = (*(vc + (8 * 0) + i) & 0xff)
			    | ((*(vc + (4 * 8) + i) & 0xff) << 8);
			dispc_reg_out(DISPC_VID_FIR_COEF_V(v, i), reg);
		} else {
			reg = (*(hc + (8 * 4) + i) & 0xff)
			    | ((*(vc + (0 * 8) + i) & 0xff) << 8)
			    | ((*(vc + (8 * 1) + i) & 0xff) << 16)
			    | ((*(vc + (2 * 8) + i) & 0xff) << 24);
			dispc_reg_out(DISPC_VID_FIR_COEF_HV(v, i), reg);
		}
	}
}

#ifdef CONFIG_OMAP_USE_DSI_PLL
/* DSI Helper Functions */
int disp_lock_dsi_pll(u32 M, u32 N, u32 M3, u32 M4, u32 freqsel)
{

	u32 count = 1000, val;
	val = ((M4 << 23) | (M3 << 19) | (M << 8) | (N << 1) | (1));
	dsipll_reg_out(DSI_PLL_CONFIGURATION1, val);
	val =
	    ((0 << 20) | (0 << 19) | (1 << 18) | (0 << 17) | (1 << 16) |
	     (0 << 14) | (1 << 13) | (0 << 12) | (0 << 11) | (0 << 8) |
	     (freqsel << 1));

	dsipll_reg_out(DSI_PLL_CONFIGURATION2, val);

	dsipll_reg_out(DSI_PLL_GO, 1);

	while ((dsipll_reg_in(DSI_PLL_GO) != 0) && (--count))
		udelay(100);

	if (count == 0) {
		printk(KERN_WARNING "GO bit not cleared\n");
		return 0;
	}

	count = 1000;
	while (((dsipll_reg_in(DSI_PLL_STATUS) & 0x2) != 0x2) && (--count))
		udelay(100);

	if (count == 0) {
		printk(KERN_WARNING "DSI PLL lock request failed = %X\n",
		       dsipll_reg_in(DSI_PLL_STATUS));
		return 0;
	}

	return 1;
}

void disp_switch_to_dsipll_clk_source(void)
{
	u32 val;
	/*Switch DISPC FCLK to DSI PLL HS divider */
	val = dss_reg_in(DSS_CONTROL);
	val = val | (1 << 1) | (1 << 0);
	dss_reg_out(DSS_CONTROL, val);
}

int disp_power_dsi_pll(u32 cmd)
{
	u32 val, count = 10000;
	/* send the power command */
	val = dsiproto_reg_in(DSI_CLK_CTRL);
	val = ((val & ~(3 << 30)) | (cmd << 30));
	dsiproto_reg_out(DSI_CLK_CTRL, val);

	/* Check whether the power status is changed */
	do {
		val = dsiproto_reg_in(DSI_CLK_CTRL);
		val = ((val & 0x30000000) >> 28);
		udelay(100);
	} while ((val != cmd) && (--count));

	return count;
}

void disp_enable_dss2fck(void)
{
	if (clk_enable(dss2f) != 0) {
		printk(KERN_WARNING "Unable to enable DSS2 FCLK\n");
		return;
	}
}

void disp_disable_dss2fck(void)
{
	clk_disable(dss2f);
}
#endif

/* Configure the panel size in the DSS according
 * to the mode selected in decoder
 */
int disp_set_dss_mode(int ch_no, int mode_index)
{
	struct omap_dispc_regs *dispc = &dss_ctx.dispc;
	struct omap_mode_info *mode = NULL;
	u32 size;

	dispc->control = dispc_reg_in(DISPC_CONTROL);

	mode = &modes[mode_index];

	if (ch_no == 1) {
		size = ((mode->width - 1) << DISPC_SIZE_DIG_PPL_SHIFT) &
		    DISPC_SIZE_DIG_PPL;
		size |=
		    (((mode->height >> 1) - 1) << DISPC_SIZE_DIG_LPP_SHIFT)
		    & DISPC_SIZE_DIG_LPP;

		dispc->size_dig = size;
		dispc_reg_out(DISPC_SIZE_DIG, dispc->size_dig);
	}

	/* enable the display controller */
	if (layer[OMAP_DSS_DISPC_GENERIC].ctx_valid)
		dispc_reg_out(DISPC_CONTROL, dss_ctx.dispc.control);

	omap_disp_reg_sync(OMAP_OUTPUT_TV);

	dispc->size_dig = dispc_reg_in(DISPC_SIZE_DIG);

	return 0;
}

/* Set the DSS register according to the output selected
 */
int disp_set_dss_output(int ch_no, char *buffer)
{
	/* Only supported output is S-Video */
	if (ch_no == 1) {
		/* Composite and S-Video Related Changes needs to be
		   done here */
	}
	return 0;
}

/*---------------------------------------------------------------------------*/
/* Exported Functions */

/* Register the encoder with the DSS */
int omap_register_encoder(struct omap_encoder_device
			   *encoder)
{
	struct channel_obj *channel = &channels[encoder->channel_id];
	int err = -EINVAL;
	struct omap_encoder_device *enc_dev;

	if (channel == NULL)
		err = -EINVAL;
	if (channel->num_encoders < MAX_ENCODER_DEVICE) {
		channel->enc_devices[channel->num_encoders++] = encoder;
		err = 0;
	}
	enc_dev = channel->enc_devices[channel->current_encoder];
	if (channel->current_encoder == ((channel->num_encoders) - 1))
		err = enc_dev->initialize(enc_dev);
	return err;
}
EXPORT_SYMBOL(omap_register_encoder);
/* omap_unregister_decoder : This function will be called by the decoder
 * driver to un-register its functionalities.
 */
int omap_unregister_encoder(struct omap_encoder_device
			     *encoder)
{
	int i, j = 0, err = 0;
	struct channel_obj *channel = &channels[encoder->channel_id];

	for (i = 0; i < channel->num_encoders; i++) {
		if (encoder == channel->enc_devices[i]) {
			if (channel->
			    enc_devices[channel->current_encoder] ==
			    encoder)
				return -EBUSY;
			channel->enc_devices[i] = NULL;
			for (j = i; j < channel->num_encoders - 1; j++)
				channel->enc_devices[j] =
				    channel->enc_devices[j + 1];
			channel->num_encoders--;
			break;
		}
	}
	return err;
}
EXPORT_SYMBOL(omap_unregister_encoder);

/* Exported function to select the Mode in
 * the current selected encoder
 */
int omap_disp_set_mode(int ch_no, char *buffer)
{
	struct omap_encoder_device *enc_dev = NULL;
	struct omap_mode_info *mode;
	int i;

	if (ch_no >= MAX_CHANNEL || ch_no < 0)
		return -EINVAL;

	if (channels[ch_no].num_encoders <= 0)
		return -EINVAL;
	/* Check whether the mode is supported by DSS or not */
	for (i = 0; i < ARRAY_SIZE(modes); i++) {
		mode = &modes[i];
		if (!(strcmp(mode->name, buffer)))
			break;
	}

	if (i == ARRAY_SIZE(modes))
		return -EINVAL;
	/* Get the handle of the current encoder device */
	enc_dev =
	    channels[ch_no].enc_devices[channels[ch_no].current_encoder];
	/* Set the mode in current encoder device  */
	if (enc_dev->mode_ops->setmode)
		if (enc_dev->mode_ops->setmode(buffer, enc_dev))
			return -EINVAL;
	/* Set the mode in DSS */
	disp_set_dss_mode(ch_no, i);
	channels[ch_no].current_mode = i;

	return 0;
}
EXPORT_SYMBOL(omap_disp_set_mode);

/* Exported function to Get the current selected mode */
char *omap_disp_get_mode(int ch_no)
{
	struct omap_encoder_device *enc_dev;

	if (channels[ch_no].num_encoders <= 0)
		return NULL;
	/* Get the handle of the current encoder device */
	enc_dev =
	    channels[ch_no].enc_devices[channels[ch_no].current_encoder];
	/* Set the mode in current encoder device  */
	if (enc_dev->mode_ops->getmode)
		return enc_dev->mode_ops->getmode(enc_dev);
	else
		return NULL;
}
EXPORT_SYMBOL(omap_disp_get_mode);

/* Exported Function to enumerate all the outputs supported by DSS */
int omap_disp_enum_output(int ch_no, int index, char *name)
{
	struct omap_encoder_device *enc_dev = NULL;
	int index_count = 0;
	int i, j;
	char *str;

	if (channels[ch_no].num_encoders <= 0)
		return -EINVAL;
	/* Reach the encoder from the list of encoders */
	for (i = 0; i < channels[ch_no].num_encoders; i++) {
		enc_dev = channels[ch_no].enc_devices[i];
		index_count += enc_dev->no_outputs;
		if (index_count > index)
			break;
	}
	if (i == channels[ch_no].num_encoders)
		return -EINVAL;

	/* Get the output index number of the encoder; */
	for (j = 0; j < i; j++) {
		enc_dev = channels[ch_no].enc_devices[j];
		index = index - enc_dev->no_outputs;
	}

	if (enc_dev->output_ops->enumoutput) {
		str = enc_dev->output_ops->enumoutput(index, enc_dev);
		strcpy(name, str);
		return 0;
	} else
		return -EINVAL;
}
EXPORT_SYMBOL(omap_disp_enum_output);

/* Exported function to set the particular output of the DSS
 * It will iterate through all the encoders for setting the
 * output
 */
int omap_disp_set_output(int ch_no, int index)
{
	struct omap_encoder_device *enc_dev = NULL, *prev_enc_dev = NULL;
	int i, j, index_count = 0;
	char mode_name[25], *str;

	if (ch_no >= MAX_CHANNEL || ch_no < 0)
		return -EINVAL;

	/* Find the encoder for the requested output */
	for (i = 0; i < channels[ch_no].num_encoders; i++) {
		enc_dev = channels[ch_no].enc_devices[i];
		index_count += enc_dev->no_outputs;
		if (index_count > index)
			break;
	}

	if (i == channels[ch_no].num_encoders)
		return -EINVAL;
	/* Find the index number of the encoder output */
	for (j = 0; j < i; j++) {
		enc_dev = channels[ch_no].enc_devices[j];
		index = index - enc_dev->no_outputs;
	}

	/* Get the previous encoder device and de-initialize it */
	prev_enc_dev =
	    channels[ch_no].enc_devices[channels[ch_no].current_encoder];

	if (prev_enc_dev->deinitialize)
		prev_enc_dev->deinitialize(enc_dev);

	/* Set the new encoder as the current encoder */
	channels[ch_no].current_encoder = i;

	str = enc_dev->output_ops->enumoutput(index, enc_dev);
	disp_set_dss_output(ch_no, str);

	/* Initialize the new encoder */
	if (enc_dev->initialize)
		enc_dev->initialize(enc_dev);
	/* Set the output of the new encoder */
	if (enc_dev->output_ops->setoutput)
		if ((enc_dev->output_ops->setoutput(index, mode_name, enc_dev) <
		     0))
			return -EINVAL;

	/* Set the DSS panel size according to the mode set in the
	 * encoder for the selected output
	 */
	for (i = 0; i < ARRAY_SIZE(modes); i++) {
		if (!(strcmp(modes[i].name, mode_name))) {
			disp_set_dss_mode(ch_no, i);
			channels[ch_no].current_mode = i;
		}
	}

	return 0;
}
EXPORT_SYMBOL(omap_disp_set_output);

/* Exported Function to get the output of DSS */
int omap_disp_get_output(int ch_no, int *index)
{
	struct omap_encoder_device *enc_dev;
	int i;
	int enc_index = 0;

	if (channels[ch_no].num_encoders <= 0)
		return -EINVAL;
	enc_dev =
	    channels[ch_no].enc_devices[channels[ch_no].current_encoder];

	for (i = 0; i < channels[ch_no].current_encoder; i++) {
		 enc_index +=
			channels[ch_no].enc_devices[i]->no_outputs;
	}
	enc_index += enc_dev->current_output;
	*index = enc_index;

	if (enc_dev->output_ops->getoutput)
		return enc_dev->output_ops->getoutput(enc_dev);

	return 0;
}
EXPORT_SYMBOL(omap_disp_get_output);
/*---------------------------------------------------------------------------*/
/* Exported Functions */
/*
 * Functions for setting video attributes
 */
void omap_disp_set_vidattributes(unsigned int video_layer,
				  unsigned int vid_attributes)
{
	dispc_reg_out(DISPC_VID_ATTRIBUTES(video_layer), vid_attributes);
}
EXPORT_SYMBOL(omap_disp_set_vidattributes);

/* Function for setting the DMS threshold */
void omap_disp_set_fifothreshold(unsigned int video_layer)
{
	/* Set FIFO threshold to 0xFF (high) and 0xFF -
	 *(16x4bytes) = 0xC0 (low)
	 * dispc_reg_out(DISPC_VID_FIFO_THRESHOLD(v),0x00FF00C0);
	 */
	dispc_reg_out(DISPC_VID_FIFO_THRESHOLD(video_layer), 0x03FC03BC);
}
EXPORT_SYMBOL(omap_disp_set_fifothreshold);

/* Set the scaling parameters */
void omap_disp_set_scaling(struct omap_scaling_params *scale_params)
{
	unsigned long firvinc, firhinc;
	int v_scale_dir = 0;

	/* horizontal and vertical upscale resizing matrix */
	const static short int hvc_u[5][8] =
	    { {0, 0, -1, -2, -9, -5, -2, -1},
	{0, -8, -11, -11, 73, 51, 30, 13},
	{128, 124, 112, 95, 73, 95, 112, 124},
	{0, 13, 30, 51, -9, -11, -11, -8},
	{0, -1, -2, -5, 0, -2, -1, 0}
	};
	/* Vertical downscale resizing matrix */
	const static short int vc_d[3][8] =
	    { {36, 40, 45, 50, 18, 23, 27, 31},
	{56, 57, 56, 55, 55, 55, 56, 57},
	{36, 31, 27, 23, 55, 50, 45, 40}
	};
	/* horizontal down scale resizing matrix */
	const static short int hc_d[5][8] =
	    { {0, 4, 8, -12, -9, -7, -5, -2},
	{36, 40, 44, 48, 17, 22, 27, 31},
	{56, 55, 54, 53, 52, 53, 54, 55},
	{36, 31, 27, 22, 51, 48, 44, 40},
	{0, -2, -5, -7, 17, 12, 8, 4}
	};

	short int *vc = (short int *) hvc_u;
	short int *hc = (short int *) hvc_u;

	if (scale_params->win_width < scale_params->crop_width)
		hc = (short int *) hc_d;
	if (scale_params->win_height < scale_params->crop_height
	    || scale_params->flicker_filter == 1) {
		vc = (short int *) vc_d;
		v_scale_dir = 1;
	}
	disp_set_resize(scale_params->video_layer, vc, hc, v_scale_dir);

	dispc_reg_out(DISPC_VID_ACCU0(scale_params->video_layer), 0);
	if (scale_params->flicker_filter == 1)
		dispc_reg_out(DISPC_VID_ACCU1(scale_params->video_layer),
			      0x01000000);
	else
		dispc_reg_out(DISPC_VID_ACCU1(scale_params->video_layer),
			      0);
	firhinc = (1024 * (scale_params->crop_width - 1))
	    / (scale_params->win_width - 1);
	if (firhinc < 1)
		firhinc = 1;
	else if (firhinc > 2047)
		firhinc = 2047;
	firvinc = (1024 * (scale_params->crop_height - 1))
	    / (scale_params->win_height - 1);
	if (firvinc < 1)
		firvinc = 1;
	else if (firvinc > 2047)
		firvinc = 2047;

	if (scale_params->flicker_filter == 0)
		dispc_reg_out(DISPC_VID_FIR(scale_params->video_layer),
			      firhinc | (firvinc << 16));
	else
		dispc_reg_out(DISPC_VID_FIR(scale_params->video_layer),
			      0x08000000);
}
EXPORT_SYMBOL(omap_disp_set_scaling);

/* Set the video parameters */
void omap_disp_set_vid_params(struct omap_video_params *vid_params)
{
	dispc_reg_out(DISPC_VID_SIZE(vid_params->video_layer),
		      vid_params->vid_size);
	dispc_reg_out(DISPC_VID_PICTURE_SIZE(vid_params->video_layer),
		      vid_params->vid_picture_size);
	dispc_reg_out(DISPC_VID_POSITION(vid_params->video_layer),
		      vid_params->vid_position);
}
EXPORT_SYMBOL(omap_disp_set_vid_params);

/* Set the row increment and pixel increment values */
void omap_disp_set_row_pix_inc_values(int video_layer, int row_inc_value,
				       int pixel_inc_value)
{
	dispc_reg_out(DISPC_VID_ROW_INC(video_layer), row_inc_value);
	dispc_reg_out(DISPC_VID_PIXEL_INC(video_layer), pixel_inc_value);
}
EXPORT_SYMBOL(omap_disp_set_row_pix_inc_values);

/* Set the cropping parameters in the software structure */
void omap_set_crop_layer_parameters(int video_layer, int cropwidth,
				     int cropheight)
{
	layer[video_layer].size_x = cropwidth;
	layer[video_layer].size_y = cropheight;
}
EXPORT_SYMBOL(omap_set_crop_layer_parameters);

/* Write the color space conversion coefficients to the display controller
 * registers.  Each coefficient is a signed 11-bit integer in the range
 * [-1024, 1023].  The matrix coefficients are:
 *	[ RY  RCr  RCb ]
 *	[ GY  GCr  GCb ]
 *	[ BY  BCr  BCb ]
 */

void omap_disp_set_colorconv(int v, int full_range_conversion)
{
	unsigned long ccreg;
	short int mtx[3][3];
	int i, j;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			mtx[i][j] = current_colorconv_values[v][i][j];
	ccreg = (mtx[0][0] & 0x7ff) | ((mtx[0][1] & 0x7ff) << 16);
	dispc_reg_out(DISPC_VID_CONV_COEF0(v), ccreg);
	ccreg = (mtx[0][2] & 0x7ff) | ((mtx[1][0] & 0x7ff) << 16);
	dispc_reg_out(DISPC_VID_CONV_COEF1(v), ccreg);
	ccreg = (mtx[1][1] & 0x7ff) | ((mtx[1][2] & 0x7ff) << 16);
	dispc_reg_out(DISPC_VID_CONV_COEF2(v), ccreg);
	ccreg = (mtx[2][0] & 0x7ff) | ((mtx[2][1] & 0x7ff) << 16);
	dispc_reg_out(DISPC_VID_CONV_COEF3(v), ccreg);
	ccreg = mtx[2][2] & 0x7ff;
	dispc_reg_out(DISPC_VID_CONV_COEF4(v), ccreg);

	if (full_range_conversion) {
		dispc_reg_merge(DISPC_VID_ATTRIBUTES(v),
				DISPC_VID_ATTRIBUTES_VIDFULLRANGE,
				DISPC_VID_ATTRIBUTES_VIDFULLRANGE);
	}

}
EXPORT_SYMBOL(omap_disp_set_colorconv);

void omap_disp_set_default_colorconv(int ltype, int color_space)
{
	int v;

	if (ltype == OMAP_VIDEO1)
		v = 0;
	else if (ltype == OMAP_VIDEO2)
		v = 1;
	else
		return;

	switch (color_space) {
	case CC_BT601:
		/* luma (Y) range lower limit is 16, BT.601 standard */
		update_colorconv_mtx(v, cc_bt601);
		omap_disp_set_colorconv(v, !FULL_COLOR_RANGE);
		break;
	case CC_BT709:
		/* luma (Y) range lower limit is 16, BT.709 standard */
		update_colorconv_mtx(v, cc_bt709);
		omap_disp_set_colorconv(v, !FULL_COLOR_RANGE);
		break;
	case CC_BT601_FULL:
		/* full luma (Y) range, assume BT.601 standard */
		update_colorconv_mtx(v, cc_bt601_full);
		omap_disp_set_colorconv(v, FULL_COLOR_RANGE);
		break;
	}
}
EXPORT_SYMBOL(omap_disp_set_default_colorconv);

void omap_disp_get_panel_size(int output_dev, int *width, int *height)
{
	unsigned long size;

	if (output_dev == OMAP_OUTPUT_TV) {
		size = dispc_reg_in(DISPC_SIZE_DIG);
		*width = 1 + ((size & DISPC_SIZE_DIG_PPL)
			      >> DISPC_SIZE_DIG_PPL_SHIFT);
		*height = 1 + ((size & DISPC_SIZE_DIG_LPP)
			       >> DISPC_SIZE_DIG_LPP_SHIFT);
		*height = *height << 1;
	}
}
EXPORT_SYMBOL(omap_disp_get_panel_size);

void omap_disp_set_panel_size(int output_dev, int width, int height)
{
	unsigned long size;

	if (output_dev == OMAP_OUTPUT_TV) {
		height = height >> 1;
		size = ((width - 1) << DISPC_SIZE_DIG_PPL_SHIFT)
		    & DISPC_SIZE_DIG_PPL;
		size |= ((height - 1) << DISPC_SIZE_DIG_LPP_SHIFT)
		    & DISPC_SIZE_DIG_LPP;
		dispc_reg_out(DISPC_SIZE_DIG, size);
	}
}
EXPORT_SYMBOL(omap_disp_set_panel_size);

/* Turn off the video1, or video2 layer. */
void omap_disp_disable_layer(int ltype)
{
	unsigned long attributes;
	int digital, v;

	if (ltype == OMAP_VIDEO1)
		v = 0;
	else if (ltype == OMAP_VIDEO2)
		v = 1;
	else
		return;

	attributes = dispc_reg_merge(DISPC_VID_ATTRIBUTES(v), 0,
				     DISPC_VID_ATTRIBUTES_ENABLE);
	digital = attributes & DISPC_VID_ATTRIBUTES_VIDCHANNELOUT;

	if (digital) {
		/* digital output */
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_GODIGITAL,
				DISPC_CONTROL_GODIGITAL);
	}
	dispc_reg_merge(DISPC_CONTROL, 0,
			DISPC_CONTROL_OVERLAYOPTIMIZATION);
}
EXPORT_SYMBOL(omap_disp_disable_layer);

/* Turn on the GFX, or video1, or video2 layer. */
void omap_disp_enable_layer(int ltype)
{
	unsigned long attributes;
	int digital, v;

	if (ltype == OMAP_VIDEO1)
		v = 0;
	else if (ltype == OMAP_VIDEO2)
		v = 1;
	else
		return;

	attributes = dispc_reg_merge(DISPC_VID_ATTRIBUTES(v),
				     DISPC_VID_ATTRIBUTES_ENABLE,
				     DISPC_VID_ATTRIBUTES_ENABLE);
	digital = attributes & DISPC_VID_ATTRIBUTES_VIDCHANNELOUT;

	if (digital) {
		/* digital output */
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_GODIGITAL,
				DISPC_CONTROL_GODIGITAL);
	}
}
EXPORT_SYMBOL(omap_disp_enable_layer);

void omap_disp_save_initstate(int ltype)
{
	unsigned long flags;

	spin_lock_irqsave(&dss_lock, flags);
	disp_save_ctx(ltype);
	spin_unlock_irqrestore(&dss_lock, flags);
}
EXPORT_SYMBOL(omap_disp_save_initstate);

/*
 *  NOte, that VENC registers are not restored here
 *  Note, that DISPC_CONTROL register is not restored here
 */
static void omap_disp_restore_ctx(int ltype)
{
	int v1 = 0, v2 = 1;
	struct omap_dispc_regs *dispc = &dss_ctx.dispc;

	if (layer[ltype].ctx_valid == 0)
		return;

	switch (ltype) {
	case OMAP_DSS_GENERIC:
		dss_reg_out(DSS_SYSCONFIG, dss_ctx.sysconfig);
		dss_reg_out(DSS_CONTROL, dss_ctx.control);
#ifdef CONFIG_ARCH_OMAP3430
		dss_reg_out(DSS_SDI_CONTROL, dss_ctx.sdi_control);
		dss_reg_out(DSS_PLL_CONTROL, dss_ctx.pll_control);
#endif
		break;

	case OMAP_DSS_DISPC_GENERIC:
		dispc_reg_out(DISPC_SYSCONFIG, dispc->sysconfig);
		dispc_reg_out(DISPC_IRQENABLE, dispc->irqenable);
		dispc_reg_out(DISPC_CONFIG, dispc->config);
		dispc_reg_out(DISPC_DEFAULT_COLOR0, dispc->default_color0);
		dispc_reg_out(DISPC_DEFAULT_COLOR1, dispc->default_color1);
		dispc_reg_out(DISPC_TRANS_COLOR0, dispc->trans_color0);
		dispc_reg_out(DISPC_TRANS_COLOR1, dispc->trans_color1);
		dispc_reg_out(DISPC_LINE_NUMBER, dispc->line_number);
		dispc_reg_out(DISPC_DATA_CYCLE1, dispc->data_cycle1);
		dispc_reg_out(DISPC_DATA_CYCLE2, dispc->data_cycle2);
		dispc_reg_out(DISPC_DATA_CYCLE3, dispc->data_cycle3);
		dispc_reg_out(DISPC_TIMING_H, dispc->timing_h);
		dispc_reg_out(DISPC_TIMING_V, dispc->timing_v);
		dispc_reg_out(DISPC_POL_FREQ, dispc->pol_freq);
		dispc_reg_out(DISPC_DIVISOR, dispc->divisor);
		dispc_reg_out(DISPC_GLOBAL_ALPHA, dispc->global_alpha);
		dispc_reg_out(DISPC_SIZE_LCD, dispc->size_lcd);
		dispc_reg_out(DISPC_SIZE_DIG, dispc->size_dig);
		break;

	case OMAP_GRAPHICS:
		dispc_reg_out(DISPC_GFX_BA0, dispc->gfx_ba0);
		dispc_reg_out(DISPC_GFX_BA1, dispc->gfx_ba1);
		dispc_reg_out(DISPC_GFX_POSITION, dispc->gfx_position);
		dispc_reg_out(DISPC_GFX_SIZE, dispc->gfx_size);
		dispc_reg_out(DISPC_GFX_ATTRIBUTES, dispc->gfx_attributes);
		dispc_reg_out(DISPC_GFX_FIFO_SIZE, dispc->gfx_fifo_size);
		dispc_reg_out(DISPC_GFX_FIFO_THRESHOLD,
			      dispc->gfx_fifo_threshold);
		dispc_reg_out(DISPC_GFX_ROW_INC, dispc->gfx_row_inc);
		dispc_reg_out(DISPC_GFX_PIXEL_INC, dispc->gfx_pixel_inc);
		dispc_reg_out(DISPC_GFX_WINDOW_SKIP,
			      dispc->gfx_window_skip);
		dispc_reg_out(DISPC_GFX_TABLE_BA, dispc->gfx_table_ba);
		break;

	case OMAP_VIDEO1:
		dispc_reg_out(DISPC_VID_BA0(v1), dispc->vid1_ba0);
		dispc_reg_out(DISPC_VID_BA1(v1), dispc->vid1_ba1);
		dispc_reg_out(DISPC_VID_POSITION(v1),
			      dispc->vid1_position);
		dispc_reg_out(DISPC_VID_SIZE(v1), dispc->vid1_size);
		dispc_reg_out(DISPC_VID_ATTRIBUTES(v1),
			      dispc->vid1_attributes);
		dispc_reg_out(DISPC_VID_FIFO_THRESHOLD(v1),
			      dispc->vid1_fifo_threshold);
		dispc_reg_out(DISPC_VID_ROW_INC(v1), dispc->vid1_row_inc);
		dispc_reg_out(DISPC_VID_PIXEL_INC(v1),
			      dispc->vid1_pixel_inc);
		dispc_reg_out(DISPC_VID_FIR(v1), dispc->vid1_fir);
		dispc_reg_out(DISPC_VID_ACCU0(v1), dispc->vid1_accu0);
		dispc_reg_out(DISPC_VID_ACCU1(v1), dispc->vid1_accu1);
		dispc_reg_out(DISPC_VID_PICTURE_SIZE(v1),
			      dispc->vid1_picture_size);

		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 0),
			      dispc->vid1_fir_coef_h0);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 1),
			      dispc->vid1_fir_coef_h1);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 2),
			      dispc->vid1_fir_coef_h2);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 3),
			      dispc->vid1_fir_coef_h3);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 4),
			      dispc->vid1_fir_coef_h4);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 5),
			      dispc->vid1_fir_coef_h5);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 6),
			      dispc->vid1_fir_coef_h6);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 7),
			      dispc->vid1_fir_coef_h7);

		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 0),
			      dispc->vid1_fir_coef_hv0);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 1),
			      dispc->vid1_fir_coef_hv1);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 2),
			      dispc->vid1_fir_coef_hv2);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 3),
			      dispc->vid1_fir_coef_hv3);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 4),
			      dispc->vid1_fir_coef_hv4);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 5),
			      dispc->vid1_fir_coef_hv5);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 6),
			      dispc->vid1_fir_coef_hv6);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 7),
			      dispc->vid1_fir_coef_hv7);

		dispc_reg_out(DISPC_VID_CONV_COEF0(v1),
			      dispc->vid1_conv_coef0);
		dispc_reg_out(DISPC_VID_CONV_COEF1(v1),
			      dispc->vid1_conv_coef1);
		dispc_reg_out(DISPC_VID_CONV_COEF2(v1),
			      dispc->vid1_conv_coef2);
		dispc_reg_out(DISPC_VID_CONV_COEF3(v1),
			      dispc->vid1_conv_coef3);
		dispc_reg_out(DISPC_VID_CONV_COEF4(v1),
			      dispc->vid1_conv_coef4);
		break;

	case OMAP_VIDEO2:
		dispc_reg_out(DISPC_VID_BA0(v2), dispc->vid2_ba0);
		dispc_reg_out(DISPC_VID_BA1(v2), dispc->vid2_ba1);
		dispc_reg_out(DISPC_VID_POSITION(v2),
			      dispc->vid2_position);
		dispc_reg_out(DISPC_VID_SIZE(v2), dispc->vid2_size);
		dispc_reg_out(DISPC_VID_ATTRIBUTES(v2),
			      dispc->vid2_attributes);
		dispc_reg_out(DISPC_VID_FIFO_THRESHOLD(v2),
			      dispc->vid2_fifo_threshold);
		dispc_reg_out(DISPC_VID_ROW_INC(v2), dispc->vid2_row_inc);
		dispc_reg_out(DISPC_VID_PIXEL_INC(v2),
			      dispc->vid2_pixel_inc);
		dispc_reg_out(DISPC_VID_FIR(v2), dispc->vid2_fir);
		dispc_reg_out(DISPC_VID_ACCU0(v2), dispc->vid2_accu0);
		dispc_reg_out(DISPC_VID_ACCU1(v2), dispc->vid2_accu1);
		dispc_reg_out(DISPC_VID_PICTURE_SIZE(v2),
			      dispc->vid2_picture_size);

		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 0),
			      dispc->vid2_fir_coef_h0);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 1),
			      dispc->vid2_fir_coef_h1);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 2),
			      dispc->vid2_fir_coef_h2);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 3),
			      dispc->vid2_fir_coef_h3);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 4),
			      dispc->vid2_fir_coef_h4);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 5),
			      dispc->vid2_fir_coef_h5);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 6),
			      dispc->vid2_fir_coef_h6);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 7),
			      dispc->vid2_fir_coef_h7);

		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 0),
			      dispc->vid2_fir_coef_hv0);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 1),
			      dispc->vid2_fir_coef_hv1);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 2),
			      dispc->vid2_fir_coef_hv2);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 3),
			      dispc->vid2_fir_coef_hv3);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 4),
			      dispc->vid2_fir_coef_hv4);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 5),
			      dispc->vid2_fir_coef_hv5);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 6),
			      dispc->vid2_fir_coef_hv6);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 7),
			      dispc->vid2_fir_coef_hv7);

		dispc_reg_out(DISPC_VID_CONV_COEF0(v2),
			      dispc->vid2_conv_coef0);
		dispc_reg_out(DISPC_VID_CONV_COEF1(v2),
			      dispc->vid2_conv_coef1);
		dispc_reg_out(DISPC_VID_CONV_COEF2(v2),
			      dispc->vid2_conv_coef2);
		dispc_reg_out(DISPC_VID_CONV_COEF3(v2),
			      dispc->vid2_conv_coef3);
		dispc_reg_out(DISPC_VID_CONV_COEF4(v2),
			      dispc->vid2_conv_coef4);
		break;
	}
}

int omap_disp_get_vrfb_offset(u32 img_len, u32 bytes_per_pixel, int side)
{
	int page_width_exp, page_height_exp, pixel_size_exp, offset = 0;

	/* Maximum supported is 4 bytes (RGB32) */
	if (bytes_per_pixel > 4)
		return -EINVAL;

	page_width_exp = PAGE_WIDTH_EXP;
	page_height_exp = PAGE_HEIGHT_EXP;
	pixel_size_exp = bytes_per_pixel >> 1;

	if (side == SIDE_W) {
		offset = ((1 << page_width_exp) *
		(pages_per_side(img_len * bytes_per_pixel, page_width_exp)))
		>> pixel_size_exp;	/* in pixels */
	} else {
		offset = (1 << page_height_exp) *
		    (pages_per_side(img_len, page_height_exp));
	}

	return offset;
}
EXPORT_SYMBOL(omap_disp_get_vrfb_offset);

void
omap_disp_set_addr(int ltype, u32 lcd_phys_addr, u32 tv_phys_addr_f0,
		    u32 tv_phys_addr_f1)
{
	int v;
	v = (ltype == OMAP_VIDEO1) ? 0 : 1;
	layer[ltype].dma[0].ba0 = lcd_phys_addr;
	layer[ltype].dma[0].ba1 = lcd_phys_addr;

	/*
	 * Store BA0 BA1 for TV, BA1 points to the alternate row
	 */
	layer[ltype].dma[1].ba0 = tv_phys_addr_f0;
	layer[ltype].dma[1].ba1 = tv_phys_addr_f1;

	dispc_reg_out(DISPC_VID_BA0(v), tv_phys_addr_f0);

	if (omap_disp_get_output_dev(ltype) == OMAP_OUTPUT_TV) {
		dispc_reg_out(DISPC_VID_BA0(v), layer[ltype].dma[1].ba0);
		dispc_reg_out(DISPC_VID_BA1(v), layer[ltype].dma[1].ba1);
		dispc_reg_merge(DISPC_VID_ATTRIBUTES(v),
				DISPC_VID_ATTRIBUTES_ENABLE,
				DISPC_VID_ATTRIBUTES_ENABLE);
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_GODIGITAL,
				DISPC_CONTROL_GODIGITAL);
	}
}
EXPORT_SYMBOL(omap_disp_set_addr);

void omap_disp_start_video_layer(int ltype)
{

	if (ltype != OMAP_VIDEO1 && ltype != OMAP_VIDEO2)
		return;

	/* Enable the Video layer and set the Go Bit */
	omap_disp_enable_layer(ltype);
}
EXPORT_SYMBOL(omap_disp_start_video_layer);

/* Many display controller registers are shadowed. Setting the GO bit causes
 * changes to these registers to take effect in hardware.
 */
void omap_disp_reg_sync(int output_dev)
{
	unsigned long timeout;
	if (output_dev == OMAP_OUTPUT_LCD)
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_GOLCD,
				DISPC_CONTROL_GOLCD);
	else
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_GODIGITAL,
				DISPC_CONTROL_GODIGITAL);

	timeout = HZ / 3;
	timeout += jiffies;
	while (omap_disp_reg_sync_bit(output_dev) &&
			time_before(jiffies, timeout)) {
		if ((!in_interrupt()) && (!irqs_disabled())) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(10);

		}
	}
}
EXPORT_SYMBOL(omap_disp_reg_sync);

/* This function provides the status of the GO bit. After the GO bit is set
 * through software, register changes take affect at the next VFP (vertical
 * front porch) or EVSYNC. Per the specs, no further register changes
 * must be done until the GO bit is reset by hardware. This function allows
 * drivers to poll the status of the GO bit, and wait until it is reset if they
 * wish to.
 */
int omap_disp_reg_sync_done(int output_dev)
{
	u32 control = dispc_reg_in(DISPC_CONTROL);

	if (output_dev == OMAP_OUTPUT_LCD)
		return ~(control & DISPC_CONTROL_GOLCD);
	else
		return ~(control & DISPC_CONTROL_GODIGITAL);
}
EXPORT_SYMBOL(omap_disp_reg_sync_done);

void omap_disp_disable(unsigned long timeout_ticks)
{
	unsigned long timeout;

	if (dispc_reg_in(DISPC_CONTROL)
	    & (DISPC_CONTROL_DIGITALENABLE | DISPC_CONTROL_LCDENABLE)) {
		/* disable the display controller */
		dispc_reg_merge(DISPC_CONTROL, 0,
				DISPC_CONTROL_DIGITALENABLE |
				DISPC_CONTROL_LCDENABLE);

		/* wait for any frame in progress to complete */
		dispc_reg_out(DISPC_IRQSTATUS, DISPC_IRQSTATUS_FRAMEDONE);
		timeout = jiffies + timeout_ticks;
		while (!(dispc_reg_in(DISPC_IRQSTATUS)
			 & DISPC_IRQSTATUS_FRAMEDONE)
		       && time_before(jiffies, timeout)) {
			int a_ctx = (in_atomic() || irqs_disabled()
				     || in_interrupt());
			if (!a_ctx) {
				set_current_state(TASK_INTERRUPTIBLE);
				schedule_timeout(1);
			} else
				udelay(100);
		}
#ifdef CONFIG_FB
		if (!(dispc_reg_in(DISPC_IRQSTATUS)
		      & DISPC_IRQSTATUS_FRAMEDONE)) {
			DEBUGP(KERN_WARNING "DSS Library: timeout waiting for "
			       "frame-done interrupt\n");
		}
#endif
#ifndef CONFIG_ARCH_OMAP3410
		disp_ll_config_tv_clocks(1);
#endif
	}

	return;
}
EXPORT_SYMBOL(omap_disp_disable);

#ifndef CONFIG_OMAP_USE_DSI_PLL
/*
 * Set the DSS Functional clock
 * The DSS clock should be 4 times the Panel's Pixel clock
 * For TV the Pixel clock required is 13.5Mhz
 * For LCD the Pixel clock is 6Mhz
 */
void omap_disp_set_dssfclk(void)
{
	/* TODO set the LCD pixel clock rate based on the LCD configuration */
#ifdef CONFIG_VIDEO_OMAP_TVOUT
	static int TV_pixel_clk = 14000000;	/* rounded 13.5 to 14 */
#endif
	u32 ask_clkrate = 0, sup_clkrate = 0, tgt_clkrate = 0, i;

	/*ask_clkrate = LCD_pixel_clk * 4; */
	ask_clkrate = m_clk_rate;

#ifdef CONFIG_VIDEO_OMAP_TVOUT
	if (ask_clkrate < (TV_pixel_clk * 4))
		ask_clkrate = TV_pixel_clk * 4;
#endif

	tgt_clkrate = ask_clkrate;

	sup_clkrate = clk_round_rate(dss1f_scale, ask_clkrate);
	if (is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
		if (clk_get_rate(dss1f_scale) == 96000000) {
			/*96M already, dont do anything for ES 1.0 */
			return;
		}
	} else {
		for (i = 1; i <= 20; i++) {
			sup_clkrate =
			    clk_round_rate(dss1f_scale, ask_clkrate);
			if (sup_clkrate >= tgt_clkrate)
				break;
			ask_clkrate = ask_clkrate + 1000000;
		}
		if (clk_set_rate(dss1f_scale, sup_clkrate) == -EINVAL)
			printk(KERN_ERR "Unable to set the DSS"
			       "functional clock to %d\n", sup_clkrate);
	}
	return;
}
EXPORT_SYMBOL(omap_disp_set_dssfclk);
#else
void omap_disp_use_dsi_pll(void)
{
	disp_enable_dss2fck();
	/*Command to change to ON state for both PLL and HSDIVISER
	 * (no clock output to the DSI complex I/O)
	 */
	if (!disp_power_dsi_pll(2)) {
		printk(KERN_WARNING "Unable to power DSI PLL\n");
		return;
	}
#ifndef CONFIG_OMAP_DVI_SUPPORT
	if (disp_lock_dsi_pll(270, 12, 4, 0, 3)) {	/* Generate 108 MHz */
#else
	if (disp_lock_dsi_pll(297, 12, 3, 0, 3)) {	/* Generate 148.5 MHz */
#endif
		omap_disp_disable_layer(OMAP_GRAPHICS);
		disp_switch_to_dsipll_clk_source();
		omap_disp_enable_layer(OMAP_GRAPHICS);
	} else {
		printk(KERN_ERR "FATAL ERROR: DSI PLL lock failed = %X\n",
		       dsipll_reg_in(DSI_PLL_STATUS));
	}

}
#endif

/* This function must be called by any driver that needs to use the display
 * controller before calling any routine that accesses the display controller
 * registers. It increments the count of the number of users of the display
 * controller, and turns the clocks ON only when required.
 */
void omap_disp_get_all_clks(void)
{
	u32 idle_dispc;
#ifdef CONFIG_HW_SUP_TRANS
	u32 idle_dss;
#endif				/* #ifdef CONFIG_HW_SUP_TRANS */
#ifndef CONFIG_ARCH_OMAP3410
	struct omap_encoder_device *enc_dev;
#endif
	spin_lock(&dss_lock);
	if (disp_usage == 0) {
		/* turn on DSS clock */
		config_disp_clocks(0);
#ifndef CONFIG_ARCH_OMAP3410
		omap_disp_set_tvref(TVREF_ON);
		disp_ll_config_tv_clocks(0);
#endif
#ifdef CONFIG_OMAP34XX_OFFMODE
#ifndef CONFIG_ARCH_OMAP3410
		/* Set the TV standard first */
		if (channels[1].num_encoders > 0) {
			enc_dev =
			    channels[1].enc_devices[channels[1]
						.current_encoder];
			if (enc_dev && enc_dev->mode_ops->setmode)
				enc_dev->mode_ops->
				    setmode(modes[channels[1].current_mode]
						.name, enc_dev);
		}
#endif
		/* restore dss context */
		omap_disp_restore_ctx(OMAP_DSS_GENERIC);
		omap_disp_restore_ctx(OMAP_DSS_DISPC_GENERIC);
		omap_disp_restore_ctx(OMAP_GRAPHICS);
		omap_disp_restore_ctx(OMAP_VIDEO1);
		omap_disp_restore_ctx(OMAP_VIDEO2);

#endif				/* #ifdef CONFIG_OMAP34XX_OFFMODE */
#ifdef CONFIG_HW_SUP_TRANS
		/* Set smart idle for Display subsystem */
		idle_dss = dss_reg_in(DSS_SYSCONFIG);
		idle_dss |= DSS_SYSCONFIG_AUTOIDLE;
		dss_reg_out(DSS_SYSCONFIG, idle_dss);
#endif				/* #ifdef CONFIG_HW_SUP_TRANS */

		/* Set smart idle, autoidle for Display controller */
		idle_dispc = dispc_reg_in(DISPC_SYSCONFIG);
		idle_dispc &= ~(DISPC_SYSCONFIG_MIDLEMODE |
				DISPC_SYSCONFIG_SIDLEMODE);

#ifdef CONFIG_HW_SUP_TRANS
		idle_dispc |= (DISPC_SYSCONFIG_MIDLEMODE_SSTANDBY |
			       DISPC_SYSCONFIG_SIDLEMODE_SIDLE |
			       DISPC_SYSCONFIG_ENABLE_WKUP);
		idle_dispc |= DISPC_SYSCONFIG_AUTOIDLE;
#else
		idle_dispc |= DISPC_SYSCONFIG_MIDLEMODE_NSTANDBY |
		    DISPC_SYSCONFIG_SIDLEMODE_NIDLE;
#endif				/* #ifdef CONFIG_HW_SUP_TRANS */

		dispc_reg_out(DISPC_SYSCONFIG, idle_dispc);
#ifdef CONFIG_OMAP34XX_OFFMODE
		dispc_reg_out(DISPC_CONTROL, dss_ctx.dispc.control);
#endif				/* #ifdef CONFIG_OMAP34XX_OFFMODE */
	} else {
		/* enable the TV clocks, since we are not if they are */
#ifndef CONFIG_ARCH_OMAP3410
		omap_disp_set_tvref(TVREF_ON);
		disp_ll_config_tv_clocks(0);
		enc_dev =
		    channels[1].enc_devices[channels[1].current_encoder];
		if (enc_dev && enc_dev->mode_ops->setmode) {
		/* Set the default standard to ntsc_m */
			enc_dev->mode_ops->
				setmode(modes[channels[1].current_mode]
					.name, enc_dev);
		}
#endif
	}
	disp_usage++;
	spin_unlock(&dss_lock);
}
EXPORT_SYMBOL(omap_disp_get_all_clks);

/* This function must be called by a driver when it not going to use the
 * display controller anymore. E.g., when a driver suspends, it must call
 * omap_disp_put_dss. When it wakes up, it must call omap_disp_get_dss again.
 * It decrements the count of the number of users of the display
 * controller, and turns the clocks OFF when not required.
 */
void omap_disp_put_all_clks(void)
{
#ifndef CONFIG_HW_SUP_TRANS
	u32 idle_dss;
#endif				/* #ifndef CONFIG_HW_SUP_TRANS */

	spin_lock(&dss_lock);
	if (disp_usage == 0) {
		printk(KERN_ERR
		       "trying to put DSS when usage count is zero\n");
		spin_unlock(&dss_lock);
		return;
	}

	disp_usage--;

	if (disp_usage == 0) {
#ifdef CONFIG_OMAP34XX_OFFMODE
		/* save dss context */
		disp_save_ctx(OMAP_DSS_GENERIC);
		disp_save_ctx(OMAP_DSS_DISPC_GENERIC);
		disp_save_ctx(OMAP_GRAPHICS);
		disp_save_ctx(OMAP_VIDEO1);
		disp_save_ctx(OMAP_VIDEO2);
#endif				/* #ifdef CONFIG_OMAP34XX_OFFMODE */
#ifndef CONFIG_HW_SUP_TRANS
		idle_dss = dispc_reg_in(DISPC_SYSCONFIG);
		idle_dss &=
		    ~(DISPC_SYSCONFIG_MIDLEMODE |
		      DISPC_SYSCONFIG_SIDLEMODE);
		idle_dss |=
		    DISPC_SYSCONFIG_MIDLEMODE_SSTANDBY |
		    DISPC_SYSCONFIG_SIDLEMODE_SIDLE;
		dispc_reg_out(DISPC_SYSCONFIG, idle_dss);
#endif				/* #ifdef CONFIG_HW_SUP_TRANS */

		omap_disp_disable(HZ / 2);
		/* turn off TV clocks */
#ifndef CONFIG_ARCH_OMAP3410
		disp_ll_config_tv_clocks(1);
		omap_disp_set_tvref(TVREF_OFF);
#endif
		mdelay(4);

		config_disp_clocks(1);
	}
	spin_unlock(&dss_lock);
}
EXPORT_SYMBOL(omap_disp_put_all_clks);

/* This function must be called by any driver that needs to use the display
 * controller before calling any routine that accesses the display controller
 * registers. It increments the count of the number of users of the display
 * controller, and turns the clocks ON only when required.
 */
void omap_disp_get_dss(void)
{
	u32 idle_dispc;
	u32 i;
#ifdef CONFIG_HW_SUP_TRANS
	u32 idle_dss;
#endif				/* #ifdef CONFIG_HW_SUP_TRANS */
	struct omap_encoder_device *enc_dev;
	unsigned int panel_width, panel_height, size = 0;
	struct omap_dispc_regs *dispc = &dss_ctx.dispc;
	struct channel_obj *channel = &channels[1];

	spin_lock(&dss_lock);
	if (disp_usage == 0) {
		/* turn on DSS clock */
		config_disp_clocks(0);
#ifndef CONFIG_ARCH_OMAP3410

		omap_disp_set_tvref(TVREF_ON);
		disp_ll_config_tv_clocks(0);
#endif
#ifdef CONFIG_OMAP34XX_OFFMODE

		/* Set the current mode for all the channels and
		 * Set the panel size accordingly
		 */
		for (i = 0; i < ARRAY_SIZE(channels); i++) {
			enc_dev =
				channels[i].enc_devices[channels[i].
						current_encoder];
			if (enc_dev && enc_dev->mode_ops->setmode) {
				enc_dev->mode_ops->
					setmode(modes[channels[i].
					current_mode].name, enc_dev);
				panel_width = modes[channel->
					current_mode].width;
				panel_height =
					modes[channel->current_mode]
						.height;
				if (i == i) {
					panel_height = panel_height>>1;
					size = ((panel_width - 1) <<
						DISPC_SIZE_DIG_PPL_SHIFT)
						& DISPC_SIZE_DIG_PPL;
					size |= ((panel_height - 1)
						<< DISPC_SIZE_DIG_LPP_SHIFT)
						& DISPC_SIZE_DIG_LPP;
					dispc->size_dig = (size);
				}
			}
		}
		/* restore dss context */
		omap_disp_restore_ctx(OMAP_DSS_GENERIC);
		omap_disp_restore_ctx(OMAP_DSS_DISPC_GENERIC);
		omap_disp_restore_ctx(OMAP_GRAPHICS);
		omap_disp_restore_ctx(OMAP_VIDEO1);
		omap_disp_restore_ctx(OMAP_VIDEO2);

#endif				/* #ifdef CONFIG_OMAP34XX_OFFMODE */
#ifdef CONFIG_HW_SUP_TRANS
		/* Set smart idle for Display subsystem */
		idle_dss = dss_reg_in(DSS_SYSCONFIG);
		idle_dss |= DSS_SYSCONFIG_AUTOIDLE;
		dss_reg_out(DSS_SYSCONFIG, idle_dss);
#endif				/* #ifdef CONFIG_HW_SUP_TRANS */

		/* Set smart idle, autoidle for Display controller */
		idle_dispc = dispc_reg_in(DISPC_SYSCONFIG);
		idle_dispc &= ~(DISPC_SYSCONFIG_MIDLEMODE |
				DISPC_SYSCONFIG_SIDLEMODE);

#ifdef CONFIG_HW_SUP_TRANS
		idle_dispc |= (DISPC_SYSCONFIG_MIDLEMODE_SSTANDBY |
			       DISPC_SYSCONFIG_SIDLEMODE_SIDLE |
			       DISPC_SYSCONFIG_ENABLE_WKUP);
		idle_dispc |= DISPC_SYSCONFIG_AUTOIDLE;
#else
		idle_dispc |= DISPC_SYSCONFIG_MIDLEMODE_NSTANDBY |
		    DISPC_SYSCONFIG_SIDLEMODE_NIDLE;
#endif				/* #ifdef CONFIG_HW_SUP_TRANS */

		dispc_reg_out(DISPC_SYSCONFIG, idle_dispc);
#ifdef CONFIG_OMAP34XX_OFFMODE
		dispc_reg_out(DISPC_CONTROL, dss_ctx.dispc.control);
#endif				/* #ifdef CONFIG_OMAP34XX_OFFMODE */
	}
	disp_usage++;
	spin_unlock(&dss_lock);
}
EXPORT_SYMBOL(omap_disp_get_dss);

/* This function must be called by a driver when it not going to use the
 * display controller anymore. E.g., when a driver suspends, it must call
 * omap_disp_put_dss. When it wakes up, it must call omap_disp_get_dss again.
 * It decrements the count of the number of users of the display
 * controller, and turns the clocks OFF when not required.
 */
void omap_disp_put_dss(void)
{
#ifndef CONFIG_HW_SUP_TRANS
	u32 idle_dss;
#endif				/* #ifndef CONFIG_HW_SUP_TRANS */

	spin_lock(&dss_lock);
	if (disp_usage == 0) {
		printk(KERN_ERR
		       "trying to put DSS when usage count is zero\n");
		spin_unlock(&dss_lock);
		return;
	}

	disp_usage--;

	if (disp_usage == 0) {
#ifdef CONFIG_OMAP34XX_OFFMODE
		/* save dss context */
		disp_save_ctx(OMAP_DSS_GENERIC);
		disp_save_ctx(OMAP_DSS_DISPC_GENERIC);
		disp_save_ctx(OMAP_GRAPHICS);
		disp_save_ctx(OMAP_VIDEO1);
		disp_save_ctx(OMAP_VIDEO2);
#endif				/* #ifdef CONFIG_OMAP34XX_OFFMODE */
#ifndef CONFIG_HW_SUP_TRANS
		idle_dss = dispc_reg_in(DISPC_SYSCONFIG);
		idle_dss &=
		    ~(DISPC_SYSCONFIG_MIDLEMODE |
		      DISPC_SYSCONFIG_SIDLEMODE);
		idle_dss |=
		    DISPC_SYSCONFIG_MIDLEMODE_SSTANDBY |
		    DISPC_SYSCONFIG_SIDLEMODE_SIDLE;
		dispc_reg_out(DISPC_SYSCONFIG, idle_dss);
#endif				/* #ifdef CONFIG_HW_SUP_TRANS */

		omap_disp_disable(HZ / 2);
#ifndef CONFIG_ARCH_OMAP3410
		{
			disp_ll_config_tv_clocks(1);
			omap_disp_set_tvref(TVREF_OFF);
		}
#endif
		mdelay(4);
		config_disp_clocks(1);
	}
	spin_unlock(&dss_lock);
}
EXPORT_SYMBOL(omap_disp_put_dss);

/* This function must be called by any driver that wishes to use a particular
 * display pipeline (layer).
 */
int omap_disp_request_layer(int ltype)
{
	int ret;
	ret = 0;

	spin_lock(&dss_lock);
	if (!layer[ltype].in_use) {
		layer[ltype].in_use = 1;
		ret = 1;
	}
	spin_unlock(&dss_lock);

	return ret;
}
EXPORT_SYMBOL(omap_disp_request_layer);

/* This function must be called by a driver when it is done using a particular
 * display pipeline (layer).
 */
void omap_disp_release_layer(int ltype)
{
	spin_lock(&dss_lock);
	layer[ltype].in_use = 0;
	layer[ltype].ctx_valid = 0;
	spin_unlock(&dss_lock);
}
EXPORT_SYMBOL(omap_disp_release_layer);

/* Used to enable LCDENABLE or DIGITALENABLE of the display controller.
 */
void omap_disp_enable_output_dev(int output_dev)
{
	if (output_dev == OMAP_OUTPUT_LCD) {
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_LCDENABLE,
				DISPC_CONTROL_LCDENABLE);
	}
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP_OUTPUT_TV) {
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_DIGITALENABLE,
				DISPC_CONTROL_DIGITALENABLE);
	}
#endif
}
EXPORT_SYMBOL(omap_disp_enable_output_dev);

/* Used to disable LCDENABLE or DIGITALENABLE of the display controller.
 */
void omap_disp_disable_output_dev(int output_dev)
{
	if (output_dev == OMAP_OUTPUT_LCD) {
		dispc_reg_merge(DISPC_CONTROL, ~DISPC_CONTROL_LCDENABLE,
				DISPC_CONTROL_LCDENABLE);
	}
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP_OUTPUT_TV) {
		dispc_reg_merge(DISPC_CONTROL,
				~DISPC_CONTROL_DIGITALENABLE,
				DISPC_CONTROL_DIGITALENABLE);
	}
#endif
}
EXPORT_SYMBOL(omap_disp_disable_output_dev);

int omap_disp_get_output_dev(int ltype)
{
	return layer[ltype].output_dev;
}
EXPORT_SYMBOL(omap_disp_get_output_dev);

/* Used to save the DMA parameter settings for a particular layer to be
 * displayed on a particular output device. These values help the
 * omap_disp_set_output_dev() function to dynamically switch the output of a
 * layer to any output device.
 */
void
omap_disp_set_dma_params(int ltype, int output_dev,
			  u32 ba0, u32 ba1, u32 row_inc, u32 pix_inc)
{
	struct omap_disp_dma_params *dma;

	if (output_dev == OMAP_OUTPUT_LCD)
		dma = &layer[ltype].dma[0];
	else
		dma = &layer[ltype].dma[1];

	dma->ba0 = ba0;
	dma->ba1 = ba1;
	dma->row_inc = row_inc;
	dma->pix_inc = pix_inc;
}
EXPORT_SYMBOL(omap_disp_set_dma_params);

/* Sets the background color */
void omap_disp_set_bg_color(int output_dev, int color)
{
#ifndef CONFIG_ARCH_OMAP3410
	if (output_dev == OMAP_OUTPUT_TV)
		dispc_reg_out(DISPC_DEFAULT_COLOR1, color);
#endif

	omap_disp_reg_sync(output_dev);
}
EXPORT_SYMBOL(omap_disp_set_bg_color);

/* Returns the current background color */
void omap_disp_get_bg_color(int output_dev, int *color)
{
#ifndef CONFIG_ARCH_OMAP3410
	if (output_dev == OMAP_OUTPUT_TV)
		*color = dispc_reg_in(DISPC_DEFAULT_COLOR1);
#endif
}
EXPORT_SYMBOL(omap_disp_get_bg_color);

#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430) \
	&& !defined(CONFIG_ARCH_OMAP3410)
/* Turn on/off the TV reference voltage from OMAP */
void omap_disp_set_tvref(int tvref_state)
{
	switch (tvref_state) {
	case TVREF_ON:
		dss_reg_out(DSS_CONTROL, (dss_reg_in(DSS_CONTROL)
					  | DSS_CONTROL_TV_REF));
		break;
	case TVREF_OFF:
		dss_reg_out(DSS_CONTROL, (dss_reg_in(DSS_CONTROL) &
					  ~(DSS_CONTROL_TV_REF)));
		break;
	}
}
EXPORT_SYMBOL(omap_disp_set_tvref);
#endif

/* Sets the SMS settings for rotation using the VRFB.
 */
int
omap_disp_set_vrfb(int context, u32 phy_addr,
		    u32 width, u32 height, u32 bytes_per_pixel)
{
	int page_width_exp, page_height_exp, pixel_size_exp;

	if (bytes_per_pixel > 4)
		return -EINVAL;

	page_width_exp = PAGE_WIDTH_EXP;
	page_height_exp = PAGE_HEIGHT_EXP;
	pixel_size_exp = bytes_per_pixel >> 1;

	width = ((1 << page_width_exp) *
		 (pages_per_side(width * bytes_per_pixel, page_width_exp))
	    ) >> pixel_size_exp;

	height = (1 << page_height_exp) *
	    (pages_per_side(height, page_height_exp));

	SMS_ROT0_PHYSICAL_BA(context) = phy_addr;
	SMS_ROT0_SIZE(context) = 0;
	SMS_ROT0_SIZE(context) |= (width << SMS_IMAGEWIDTH_OFFSET)
	    | (height << SMS_IMAGEHEIGHT_OFFSET);
	SMS_ROT_CONTROL(context) = 0;

	SMS_ROT_CONTROL(context) |= pixel_size_exp << SMS_PS_OFFSET
	    | (page_width_exp - pixel_size_exp) << SMS_PW_OFFSET
	    | page_height_exp << SMS_PH_OFFSET;

	return 0;
}
EXPORT_SYMBOL(omap_disp_set_vrfb);

/* Sets the transparency color key type and value.
*/
void omap_disp_set_colorkey(int output_dev, int key_type, int key_val)
{
#ifndef CONFIG_ARCH_OMAP3410
	if (output_dev == OMAP_OUTPUT_TV) {
		if (key_type == OMAP_VIDEO_SOURCE)
			dispc_reg_merge(DISPC_CONFIG,
					DISPC_CONFIG_TCKDIGSELECTION,
					DISPC_CONFIG_TCKDIGSELECTION);
		else
			dispc_reg_merge(DISPC_CONFIG, 0,
					DISPC_CONFIG_TCKDIGSELECTION);
		dispc_reg_out(DISPC_TRANS_COLOR1, key_val);
	}
#endif

	omap_disp_reg_sync(output_dev);
}
EXPORT_SYMBOL(omap_disp_set_colorkey);

/* Returns the current transparency color key type and value.
*/
void omap_disp_get_colorkey(int output_dev, int *key_type, int *key_val)
{

#ifndef CONFIG_ARCH_OMAP3410
	if (output_dev == OMAP_OUTPUT_TV) {
		if (dispc_reg_in(DISPC_CONFIG) &
		    DISPC_CONFIG_TCKDIGSELECTION)
			*key_type = OMAP_VIDEO_SOURCE;
		else
			*key_type = OMAP_GFX_DESTINATION;
		*key_val = dispc_reg_in(DISPC_TRANS_COLOR1);
	}
#endif
}
EXPORT_SYMBOL(omap_disp_get_colorkey);

void omap_disp_enable_colorkey(int output_dev)
{

#ifndef CONFIG_ARCH_OMAP3410
	if (output_dev == OMAP_OUTPUT_TV)
		dispc_reg_merge(DISPC_CONFIG, DISPC_CONFIG_TCKDIGENABLE,
				DISPC_CONFIG_TCKDIGENABLE);
#endif

	omap_disp_reg_sync(output_dev);
}
EXPORT_SYMBOL(omap_disp_enable_colorkey);

void omap_disp_disable_colorkey(int output_dev)
{
#ifndef CONFIG_ARCH_OMAP3410
	if (output_dev == OMAP_OUTPUT_TV)
		dispc_reg_merge(DISPC_CONFIG, ~DISPC_CONFIG_TCKDIGENABLE,
				DISPC_CONFIG_TCKDIGENABLE);
#endif

	omap_disp_reg_sync(output_dev);
}
EXPORT_SYMBOL(omap_disp_disable_colorkey);

#ifdef CONFIG_ARCH_OMAP34XX
void omap_disp_set_alphablend(int output_dev, int value)
{

#ifndef CONFIG_ARCH_OMAP3410
	if (output_dev == OMAP_OUTPUT_TV) {
		if (value)
			dispc_reg_merge(DISPC_CONFIG,
					DISPC_CONFIG_TVALPHAENABLE,
					DISPC_CONFIG_TVALPHAENABLE);
		else
			dispc_reg_merge(DISPC_CONFIG,
					~DISPC_CONFIG_TVALPHAENABLE,
					DISPC_CONFIG_TVALPHAENABLE);
	}
#endif
	omap_disp_reg_sync(output_dev);
}
EXPORT_SYMBOL(omap_disp_set_alphablend);

void omap_disp_set_global_alphablend_value(int ltype, int value)
{
	u32  alpha_value;
	alpha_value = 0;

	if (ltype == OMAP_VIDEO2) {
		alpha_value = dispc_reg_in(DISPC_GLOBAL_ALPHA);
		alpha_value &= (~DISPC_GLOBAL_ALPHA_VID2_GALPHA);
		alpha_value |=
		    (value << DISPC_GLOBAL_ALPHA_VID2_GALPHA_SHIFT);
		dispc_reg_out(DISPC_GLOBAL_ALPHA, alpha_value);

	}
#ifndef CONFIG_ARCH_OMAP3410
	omap_disp_reg_sync(OMAP_OUTPUT_TV);
#endif
}
EXPORT_SYMBOL(omap_disp_set_global_alphablend_value);

unsigned char omap_disp_get_global_alphablend_value(int ltype)
{
	u32  alpha_value;
	alpha_value = 0;

	if (ltype == OMAP_VIDEO2) {
		alpha_value = dispc_reg_in(DISPC_GLOBAL_ALPHA);
		alpha_value &= (DISPC_GLOBAL_ALPHA_VID2_GALPHA);
		alpha_value = alpha_value >>
		    DISPC_GLOBAL_ALPHA_VID2_GALPHA_SHIFT;
	}
	return (unsigned char) alpha_value;
}
EXPORT_SYMBOL(omap_disp_get_global_alphablend_value);

int omap_disp_get_alphablend(int output_dev)
{

#ifndef CONFIG_ARCH_OMAP3410
	if (output_dev == OMAP_OUTPUT_TV) {
		if (dispc_reg_in(DISPC_CONFIG) & 0x00080000)
			return 1;
		else
			return 0;
	}
#endif
	return 0;
}
EXPORT_SYMBOL(omap_disp_get_alphablend);
#endif

int omap_disp_reg_sync_bit(int output_dev)
{
	u32 control = dispc_reg_in(DISPC_CONTROL);

	if (output_dev == OMAP_OUTPUT_LCD)
		return (control & DISPC_CONTROL_GOLCD) >> 5;
	else
		return (control & DISPC_CONTROL_GODIGITAL) >> 6;
}
EXPORT_SYMBOL(omap_disp_reg_sync_bit);

/*
 * Enables an IRQ in DSPC_IRQENABLE.
 */
int omap_disp_irqenable(omap_disp_isr_t isr, unsigned int mask)
{
	int i;
	unsigned long flags;

	if (omap_disp_irq == 0 || mask == 0)
		return -EINVAL;

	spin_lock_irqsave(&dss_lock, flags);
	for (i = 0; i < MAX_ISR_NR; i++) {
		if (registered_isr[i].isr == isr) {
			registered_isr[i].mask |= mask;
			dispc_reg_out(DISPC_IRQENABLE,
				      dispc_reg_in(DISPC_IRQENABLE) |
				      mask);
			spin_unlock_irqrestore(&dss_lock, flags);
			return 0;
		}
	}
	spin_unlock_irqrestore(&dss_lock, flags);
	return -EBUSY;
}
EXPORT_SYMBOL(omap_disp_irqenable);

/*
 * Disables an IRQ in DISPC_IRQENABLE,
 * The IRQ will be active if any other ISR is still using the same.
 * mask : should contain '0' for irq to be disable and rest should be '1'.
 */
int omap_disp_irqdisable(omap_disp_isr_t isr, unsigned int mask)
{
	int i;
	unsigned long flags;
	unsigned int new_mask;
	new_mask = 0;

	if (omap_disp_irq == 0)
		return -EINVAL;

	spin_lock_irqsave(&dss_lock, flags);
	for (i = 0; i < MAX_ISR_NR; i++)
		if (registered_isr[i].isr == isr)
			break;

	if (i == MAX_ISR_NR) {
		spin_unlock_irqrestore(&dss_lock, flags);
		return -EINVAL;
	}

	registered_isr[i].mask &= mask;

	/* disable an IRQ if every one wishes to do so */
	for (i = 0; i < MAX_ISR_NR; i++)
		new_mask |= registered_isr[i].mask;

	dispc_reg_out(DISPC_IRQENABLE, new_mask);
	spin_unlock_irqrestore(&dss_lock, flags);
	return -EBUSY;
}
EXPORT_SYMBOL(omap_disp_irqdisable);

/* Display controller interrupts are handled first by this display library.
 * Drivers that need to use certain interrupts should register their ISRs and
 * interrupt enable mask with the display library.
 */
int
omap_disp_register_isr(omap_disp_isr_t isr, void *arg, unsigned int mask)
{
	int i;
	unsigned long flags;

	if (omap_disp_irq == 0 || isr == 0 || arg == 0)
		return -EINVAL;

	/* Clear all the interrupt, so that you dont get an immediate
	 * interrupt
	 */
	dispc_reg_out(DISPC_IRQSTATUS, 0xFFFFFFFF);
	spin_lock_irqsave(&dss_lock, flags);
	for (i = 0; i < MAX_ISR_NR; i++) {
		if (registered_isr[i].isr == NULL) {
			registered_isr[i].isr = isr;
			registered_isr[i].arg = arg;
			registered_isr[i].mask = mask;

			/* Clear previous interrupts if any */
			dispc_reg_out(DISPC_IRQSTATUS, mask);
			dispc_reg_out(DISPC_IRQENABLE,
				      dispc_reg_in(DISPC_IRQENABLE) |
				      mask);
			spin_unlock_irqrestore(&dss_lock, flags);
			return 0;
		}
	}
	spin_unlock_irqrestore(&dss_lock, flags);
	return -EBUSY;
}
EXPORT_SYMBOL(omap_disp_register_isr);

int omap_disp_unregister_isr(omap_disp_isr_t isr)
{
	int i, j;
	unsigned long flags;
	unsigned int new_mask;

	new_mask = 0;
	if (omap_disp_irq == 0)
		return -EINVAL;

	spin_lock_irqsave(&dss_lock, flags);
	for (i = 0; i < MAX_ISR_NR; i++) {
		if (registered_isr[i].isr == isr) {
			registered_isr[i].isr = NULL;
			registered_isr[i].arg = NULL;
			registered_isr[i].mask = 0;

			/* The interrupt may no longer be valid, re-set
			 * the IRQENABLE */
			for (j = 0; j < MAX_ISR_NR; j++)
				new_mask |= registered_isr[j].mask;

			dispc_reg_out(DISPC_IRQENABLE, new_mask);
			spin_unlock_irqrestore(&dss_lock, flags);
			return 0;
		}
	}
	spin_unlock_irqrestore(&dss_lock, flags);
	return -EINVAL;
}
EXPORT_SYMBOL(omap_disp_unregister_isr);

int __init omap_disp_init(void)
{
	int rev, i;
	u32 dss_control;

	spin_lock_init(&dss_lock);

	/* Required for scale call */
#ifdef CONFIG_TRACK_RESOURCES
	dss1f_scale =
	    clk_get(&display_dev,
		    cpu_is_omap34xx()? "dss1_alwon_fck" : "dss1_fck");
#else
	dss1f_scale =
	    clk_get(NULL,
		    cpu_is_omap34xx()? "dss1_alwon_fck" : "dss1_fck");
#endif
	if (IS_ERR(dss1f_scale)) {
		printk(KERN_WARNING "Could not get DSS1 FCLK\n");
		return PTR_ERR(dss1f_scale);
	}

	omap_disp_get_all_clks();

	/* disable the display controller */
	omap_disp_disable(HZ / 5);

	rev = dss_reg_in(DSS_REVISION);
	printk(KERN_INFO "OMAP Display hardware version %d.%d\n",
	       (rev & DISPC_REVISION_MAJOR) >> DISPC_REVISION_MAJOR_SHIFT,
	       (rev & DISPC_REVISION_MINOR) >> DISPC_REVISION_MINOR_SHIFT);

	/* enable DAC_DEMEN and VENC_4X_CLOCK in DSS for TV operation */
	dss_control = dss_reg_in(DSS_CONTROL);

	/* Should be replaced by FPGA register read  ADD A 2420 ifdef here */

#ifdef CONFIG_ARCH_OMAP2420
	dss_control |= (DSS_CONTROL_DAC_DEMEN |
			DSS_CONTROL_VENC_CLOCK_4X_ENABLE);
#endif

#ifdef CONFIG_MACH_OMAP_2430SDP
#ifdef CONFIG_TWL4030_CORE_T2
	dss_control |= (DSS_CONTROL_TV_REF | DSS_CONTROL_DAC_DEMEN |
			DSS_CONTROL_VENC_CLOCK_4X_ENABLE);
#endif

#ifdef CONFIG_TWL4030_CORE_M1
	dss_control |= (DSS_CONTROL_DAC_DEMEN |
			DSS_CONTROL_VENC_CLOCK_4X_ENABLE);
#endif
#endif

#if defined(CONFIG_MACH_OMAP_3430SDP) ||  defined(CONFIG_MACH_OMAP3EVM) \
	|| defined(CONFIG_MACH_OMAP_3430LABRADOR)
	/* enabling S-video connector for 3430 SDP */
#ifndef CONFIG_ARCH_OMAP3410
	dss_control |= (DSS_CONTROL_DAC_DEMEN | DSS_CONTROL_TV_REF |
			DSS_CONTROL_VENC_CLOCK_4X_ENABLE |
			DSS_CONTROL_VENC_OUT);
#else
	dss_control |= (DSS_CONTROL_DAC_DEMEN |
			DSS_CONTROL_VENC_CLOCK_4X_ENABLE |
			DSS_CONTROL_VENC_OUT);
#endif
#endif

	dss_control &= ~DSS_CONTROL_VENC_CLOCK_MODE;
	dss_reg_out(DSS_CONTROL, dss_control);

	/* By default, all layers go to LCD */
	layer[OMAP_GRAPHICS].output_dev = OMAP_OUTPUT_TV;
	layer[OMAP_VIDEO1].output_dev = OMAP_OUTPUT_TV;
	layer[OMAP_VIDEO2].output_dev = OMAP_OUTPUT_TV;

	/*
	 * Set the default color conversion parameters for Video pipelines
	 * by default the color space is set to JPEG
	 */

	update_colorconv_mtx(0, cc_bt601_full);
	omap_disp_set_colorconv(0, FULL_COLOR_RANGE);

	update_colorconv_mtx(1, cc_bt601_full);
	omap_disp_set_colorconv(1, FULL_COLOR_RANGE);

	/* Disable the Alpha blending for both TV and LCD
	 * Also set the global alpha value for both
	 * graphics and video2 pipeline to 255(Completely Opaque)
	 */
	omap_disp_set_alphablend(OMAP_OUTPUT_TV, 0);

	omap_disp_set_global_alphablend_value(OMAP_VIDEO2, 0xFF);

#ifndef CONFIG_ARCH_OMAP3410
	omap_disp_set_bg_color(OMAP_OUTPUT_TV, 0x000000);
#endif

	if (request_irq(INT_24XX_DSS_IRQ, (void *) omap_disp_master_isr,
			IRQF_SHARED, "OMAP Display", registered_isr)) {
		printk(KERN_WARNING "omap_disp: request_irq failed\n");
		omap_disp_irq = 0;
	} else {
		omap_disp_irq = 1;
		for (i = 0; i < MAX_ISR_NR; i++) {
			registered_isr[i].isr = NULL;
			registered_isr[i].mask = 0;
		}
		/* Clear all the pending interrupts, if any */
		dispc_reg_out(DISPC_IRQSTATUS, 0xFFFFFFFF);
		omap_disp_register_isr(disp_synclost_isr, layer,
					DISPC_IRQSTATUS_SYNCLOST);
	}

	omap_disp_register_isr(disp_synclost_isr, layer,
				DISPC_IRQSTATUS_SYNCLOST);
	omap_disp_put_all_clks();

	return 0;

}

/* Start before devices */
subsys_initcall(omap_disp_init);
