 /*
 * arch/arm/plat-omap/dsi.c
 *
 * This file contains the low level functions for OMAP DSI interface
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

/*=======INCLUDES======================================================*/
#include "dsi.h"
#include "edisco_drv.h"

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/arch/hardware.h>
#include <asm/arch/display.h>
#include <asm/arch/clock.h>

/* YUV to RGB conversion table */
/* static S16 color_convbt601_full[][3] = {
						{298, 409, 0},
						{298, -208, -11},
						{298, 0, 517}
						};*/
/* TODO: need to update this */
/* Hardcoded values for time being. */
/* The DSI protocol engine functional clock
 * should be higher than the Byte Clock */
/* static U16	dsi1_pll_clk = 55;*/ /* Functional clock to DISPC  */
/* static U16	dsi2_pll_clk = 55;*//*Functional clock to DSI protocol engine */

/* #define RGB18_IMAGE */

#ifndef RGB18_IMAGE
#ifdef CONFIG_OMAP_DSI
static U16	dsi1_pll_clk = 100; /* Functional clock to DISPC  */
#else
static U16	dsi1_pll_clk = 50; /* Functional clock to DISPC  */
#endif
static U16	dsi2_pll_clk = 100; /*Functional clock to DSI protocol engine*/
#else
static U16	dsi1_pll_clk = 111; /* Functional clock to DISPC  */
static U16	dsi2_pll_clk = 120; /*Functional clock to DSI protocol engine*/
#endif

static inline U32
in_regl(U32 offset)
{
		return omap_readl(offset);
}

static inline U32
out_regl(U32 offset, U32 val)
{
		omap_writel(val, offset);
		return val;
}


static inline U32
disp_reg_in(U32 offset)
{
	return omap_readl(DSS_REG_BASE + DISPC_REG_OFFSET + offset);
}

static inline U32
disp_reg_out(U32 offset, U32 val)
{
	omap_writel(val, DSS_REG_BASE + DISPC_REG_OFFSET + offset);
	return val;
}
static inline U32
disp_reg_merge(U32 offset, U32 val, U32 mask)
{
	u32 addr = DSS_REG_BASE + DISPC_REG_OFFSET + offset;
	u32 new_val = (omap_readl(addr) & ~mask) | (val & mask);

	omap_writel(new_val, addr);
	return new_val;
}

static inline U32
dss_reg_in(U32 offset)
{
	return  omap_readl(DSS_REG_BASE + DSS_REG_OFFSET + offset);
}
static inline U32
dss_reg_out(U32 offset, U32 val)
{
	omap_writel(val, DSS_REG_BASE + DSS_REG_OFFSET + offset);
	return val;
}
static inline U32
dss_reg_merge(U32 offset, U32 val, U32 mask)
{
	U32 addr = DSS_REG_BASE + DSS_REG_OFFSET + offset;
	U32 new_val = (omap_readl(addr) & ~mask) | (val & mask);

	omap_writel(new_val, addr);
	return new_val;
}


/*============ FUNCTIONS ====================================================*/


/*#define NB_DATALANES_1*/

#define DSI_DATALANE_X2 74
#define DSI_DATALANE_Y2 75

/*#define SILVAL_CONFIG */
/*#define DATA_LANE2_GPIO_CTRL */
/*#define VIDEO_MODE_TX */
#define BTA_ENABLE
#define CMD_VC1

#ifdef NB_DATALANES_1
#define DATA_LANE2_GPIO_CTRL
#endif


#ifdef CMD_VC1
U8 cmd_vc = 1;
U8 video_vc;
#else
U8 cmd_vc;
U8 video_vc = 1;
#endif

/*-----------------------------------------------------------------------------
| Function    : void dsi_complexio_lane_config(U8 clk_pos, U8 clk_pol,
|		U8 data1_pos, U8 data1_pol, U8 data2_pos, U8 data2_pol)
+------------------------------------------------------------------------------
| Description : Function does the ComplexIO data and clock lane configuration
|
| Parameters  : clk_pos - clock lane position
|		clk_pol - polarity of clock lane
|		data1_pos - data lane1 position
|		data1_pol - polarity of data lane1
|		data2_pos - data lane2 position
|		data2_pol - polarity of data lane2
|
| Returns     : None
+----------------------------------------------------------------------------*/
void dsi_complexio_lane_config(U8 clk_pos, U8 clk_pol, U8 data1_pos, \
				U8 data1_pol, U8 data2_pos, U8 data2_pol)
{
	U32 val;

	/* Configure the data lane andclock lane */
	val = in_regl(DSI_COMPLEXIO_CFG1);
	val	= val |
		  ((data2_pol << 11) |
		   (data2_pos << 8) |
		   (data1_pol << 7) |
		   (data1_pos << 4) |
		   (clk_pol << 3) |
		   (clk_pos << 0));
	out_regl(DSI_COMPLEXIO_CFG1, val);


	/* To toggle the data lane 2 manually using the GPIO lines */
#ifdef DATA_LANE2_GPIO_CTRL
	/* Configure the GPIO as output */
	gpio_pin_init(DSI_DATALANE_X2, 0);
	gpio_pin_init(DSI_DATALANE_Y2, 0);

	set_gpio_output(DSI_DATALANE_X2, 1);
	set_gpio_output(DSI_DATALANE_Y2, 1);
#endif
}

/*-----------------------------------------------------------------------------
| Function    : dsi_complexio_timing_config(T_DPHY_CONFIG complexio_timing)
+------------------------------------------------------------------------------
| Description : Function does the ComplexIO (DSI DPHY) timing parameters
|
| Parameters  : complexio_timing - structure of DPHY timing configuration
|
| Returns     : None
+----------------------------------------------------------------------------*/
void dsi_complexio_timing_config(T_DPHY_CONFIG complexio_timing)
{
	U32 val;


	/* Configure the DSI PHY timing parameters */
#ifndef SILVAL_CONFIG
	val = ((complexio_timing.ths_prepare << 24) |
		   (complexio_timing.ths_prepare_ths_zero << 16) |
		   (complexio_timing.ths_trail << 8) |
		   (complexio_timing.ths_exit << 0));
	out_regl(DSIPHY_CFG0, val);

	val = in_regl(DSIPHY_CFG1); /* to preserve the reset data */
	val = val & (0xFF800000);
	val = val | ((complexio_timing.tlpx_half << 16) |
		(complexio_timing.tclk_trail << 8) |
		(complexio_timing.tclk_zero << 0));
	out_regl(DSIPHY_CFG1, val);

	val = in_regl(DSIPHY_CFG2); /* to preserve the reset data */
	if (complexio_timing.data_rate > 400)
		val = val & ~(1 << 23);
	else
		val = val | (1 << 23);

	val = val & ~(0x007FFFFF);
	val = val | (complexio_timing.tclk_prepare << 0);
	out_regl(DSIPHY_CFG2, val);
#else
	val = ((thsp << 24) |
		   (thspz << 16) |
		   (thst << 8) |
		   (thse << 0));
	out_regl(DSIPHY_CFG0, val);

	val = ((ttag0 << 29) |
		(ttasure << 27) |
		(ttaget << 24) |
		(tlpxdiv2 << 16) |
		(tclktrail << 8) |
		(tclkzero << 0));
	out_regl(DSIPHY_CFG1, val);

	val = in_regl(DSIPHY_CFG2); /* to preserve the reset data */
/*	val = val & ~(1 << 23);*/ /* assuming its two lanes and 600Mbs */
	val = val & ~(0x007FFFFF);
	val = val | (1 << 23); /* assuming its two lanes and 600Mbs */
	val = val | (tclkp << 0);
	out_regl(DSIPHY_CFG2, val);
#endif
}

/*-----------------------------------------------------------------------------
| Function    : dsi_complexio_pwr_cmd(U8 cmd)
+------------------------------------------------------------------------------
| Description : Function send the power command to complexio
|
| Parameters  : cmd - power command
|
| Returns     : Status (CSST_DAL_SUCCESS or Error)
+---------------------------------------------------------------------------*/
S32 dsi_complexio_pwr_cmd(U8 cmd)
{
	U32 val, count = 10000;


	/* send power command to complexio module  */
	val = in_regl(DSI_COMPLEXIO_CFG1);
	val = ((val & 0xE7FFFFFF) |
		(cmd << 27));

	out_regl(DSI_COMPLEXIO_CFG1, val);

	/* Check whether the power status is changed */
	do {
		val = in_regl(DSI_COMPLEXIO_CFG1);
		val = ((val & 0x06000000) >> 25);
		udelay(100);
	} while ((val != cmd) && (--count));

	if (count == 0) {
		printk(KERN_ERR "dsi_complexio_pwr_cmd\
			-----> Timed out <---: cmd=0x%x \n", cmd);
		/* Timeout happened, something seriously WRONG!! */
		return OMAP_DAL_ERROR;
	} else {
		printk(KERN_INFO "dsi_complexio_pwr_cmd -----> Success \n");
		return OMAP_DAL_SUCCESS;
	}
}

/*-----------------------------------------------------------------------------
| Function    : U8 dsi_complexio_reset_status(void)
+------------------------------------------------------------------------------
| Description : Function to check the complexio reset status
|
| Parameters  : None
|
| Returns     : Reset status
+----------------------------------------------------------------------------*/
U8 dsi_complexio_reset_status(void)
{
	U32 val1, val2;
	val1 = in_regl(DSI_COMPLEXIO_CFG1);
	val2 = in_regl(DSIPHY_CFG5);
	/* Check the RESET done bits */
	/* TODO: need to check the reset code again, where to add dummy reads */
	if ((val1 & 0x20000000) && ((val2 & 0xFC000000) == 0xFC000000))
		return RESET_COMPLETED;
	else
		return RESET_NOT_COMPLETED;
}

/*-----------------------------------------------------------------------------
| Function    : config_dphy(T_DSI_DIS *dsi_dis)
+------------------------------------------------------------------------------
| Description : Function to configure the ComplexIO module
|
| Parameters  : dsi_dis - pointer to DSI device information structure
|
| Returns     : Status (CSST_DAL_SUCCESS or Error)
+----------------------------------------------------------------------------*/
S32 config_dphy(T_DSI_DIS *dsi_dis)
{
	U32 count = 10000;
	S32 ret = OMAP_DAL_SUCCESS;
	U32 val;

	/* TODO: need to check what all things to be taken
	 * care before starting programming */
	/* Wait for the Complex IO reset complete */
	/* Send Power ON command */
	/* Need to cross check whether this is the correct place
	 * to power on the ComplexIO */
	ret = dsi_complexio_pwr_cmd(DPHY_PWR_CMD_ON);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

	do {
		ret = dsi_complexio_reset_status();
		udelay(100);
	} while ((ret != RESET_COMPLETED) && (--count));

	/* count = 1; */
	if (count == 0) {
		printk(KERN_INFO "config_dphy: dsi_complexio_reset_status\
			incomplete !!!! \n");
		val = in_regl(DSI_COMPLEXIO_CFG1);
		printk(KERN_INFO "config_dphy: DSI_COMPLESIO_CFG1=0x%x\
			!!!! \n", (unsigned int) val);
		val = in_regl(DSIPHY_CFG5);
		printk(KERN_INFO "config_dphy: DSIPHY_CFG5=0x%x\
			!!!! \n", (unsigned int) val);
		/* TODO -- Ideally we should bail out here  */
	}

	/* Lane configuration */
	dsi_complexio_lane_config(dsi_dis->dphy_config.clk_lane.pos,
					dsi_dis->dphy_config.clk_lane.pol,
					dsi_dis->dphy_config.data_lane[0].pos,
					dsi_dis->dphy_config.data_lane[0].pol,
					dsi_dis->dphy_config.data_lane[1].pos,
					dsi_dis->dphy_config.data_lane[1].pol);
	/* Complex IO timing configuration */
	dsi_complexio_timing_config(dsi_dis->dphy_config);

	/* Set GO bit */
	val = in_regl(DSI_COMPLEXIO_CFG1);
	val = val | (1 << 30);
	/*TODO: need to check whether we need to wait for
	 * the GO bit to go to reset */
	out_regl(DSI_COMPLEXIO_CFG1, val);

	count = 100;

	/* Waiting for the Go bit to be reset by HW */
	while ((in_regl(DSI_COMPLEXIO_CFG1) & (1 << 30)) && (--count))
		udelay(100);

	/* count = 1; */
	if (count == 0) {
		printk(KERN_INFO "config_dphy: dsi_complexio_HW_reset_status\
			incomplete !!!! \n");
		/* TODO -- Ideally we should bail out here  */
	}
	/* enable the COMPLEXIO interrupts */
	out_regl(DSI_COMPLEXIO_IRQSTATUS, 0xFFFFFFFF); /*clear the events*/
	out_regl(DSI_COMPLEXIO_IRQENABLE, 0x0); /*enable the interrupt events*/
	/* Send Power ON command */

	printk(KERN_INFO "ComplexIO configuration completed \n\r");
	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : config_dsipll(U16 pixel_clk, U8 bits_per_pixel,
|		U8 num_data_lanes, U8 pclk_or_sysclk, U16 clkin,
|		U8 hsdivider, U8 config_mode)
+------------------------------------------------------------------------------
| Description : Function to configure the DSI PLL module
|
| Parameters  : pixel_clk - pixel clock value of the LCD connected to
|		MIPI DPI interface of DSI device
|		bits_per_pixel - number of bits represent one pixel (16 or 18)
|		num_data_lanes - number of data lanes of the DSI device
|		pclk_or_sysclk	- input clock to DSI PLL (SYSCLK or PCLK)
|		clkin		- DSI PLL input reference clock
|		hsdivider  	- hsdivider is required or not
|		config_mode	- configuration mode (manual or automatic)
|
| Returns     : Status (OMAP_DAL_SUCCESS or Error)
+----------------------------------------------------------------------------*/
S32 config_dsipll(U16 pixel_clk, U8 bits_per_pixel, U8 num_data_lanes, \
		U8 pclk_or_sysclk, U16 clkin, U8 hsdivider, U8 config_mode)
{
	U32 count = 10000;
	S32 ret;
	U16 ddr_clk, dsiphy_clk, fint, highfreq;
	U16 regm, regn, regm3, regm4;

	fint = 2; /* ~2Mhz is the internal clock to PLL */

	/* Check if PLL input is more than 32MHz */
	if (clkin > 32)
		highfreq = 1;
	else
		highfreq = 0;

	/* Calculation of data rates and clocks divisors */
	ddr_clk = get_ddr_clk_val(pixel_clk, bits_per_pixel, num_data_lanes);

	dsiphy_clk = ddr_clk * 4;

#ifndef SILVAL_CONFIG
	regn = (clkin < 32) ? ((clkin / fint)-1) : ((clkin / (2 * fint)) - 1) ;
	regm = ((dsiphy_clk) * (highfreq + 1)*(regn + 1)) / (2 * clkin);
#else
	regn = 12;
	regm = 150;
#endif

	/* Need to configure the HSDIVIDER only if required */
	if (hsdivider == HSDIV_ENABLE) {
#ifndef SILVAL_CONFIG
		regm3 = (dsiphy_clk / dsi1_pll_clk) - 1;
		regm4 = (dsiphy_clk / dsi2_pll_clk) - 1;
		/* this is a hack to make sure M3 and M4 values
		 *will not be zero when the DSIPHY clock is low value */
		(regm3 == 0) ? (regm3++) : (regm3);
		(regm4 == 0) ? (regm4++) : (regm4);
#else
		regm3 = 11;
		regm4 = 11;
#endif
	} else {
		regm3 = regm4 = 0;
	}
	/* TODO: need to decide what all things to be enabled */
	ret = dsi_pll_pwr_cmd(PWR_CMD_STATE_ON_ALL);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

	/* TODO: need to check what all things to be taken care
	 * before starting programming */

	/* Check the reset status of the PLL */
	while ((dsi_pll_reset_status() != RESET_COMPLETED) && (--count))
	    udelay(100);

	/* Timeout happened */
	if (count == 0)
		return OMAP_DAL_ERROR;

	/* TODO: need to decide what all things to be enabled */
	ret = dsi_pll_pwr_cmd(PWR_CMD_STATE_ON_ALL);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

	/* Lock the PLL */
	ret = lock_dsi_pll(config_mode,
				pclk_or_sysclk,
				clkin,
				regn,
				regm,
				regm3,
				regm4);

	if (ret != OMAP_DAL_SUCCESS)
		return ret;

	return ret;
}

int config_dsipll_lclk(int logic_clk, int pclk_or_sysclk,\
				int clkin, int config_mode)
{
	U32 count = 10000;
	S32 ret = 0;
	U16 dsiphy_clk, fint, highfreq;
	U16 regm, regn, regm3, regm4;
	U32 val;
	printk(KERN_INFO "config_dsipll: logic_clk=%d, clkin=%d !!! \n", \
				logic_clk, clkin);
	printk(KERN_INFO "config_dsipll:pclk_sysclk=%d, config_mode=%d !!! \n",\
				pclk_or_sysclk, config_mode);

	{
		/* It seems this is needed for locking the PLL */
		val = disp_reg_in(DISPC_CONTROL);
		val =  (1 << 27) |
			(1 << 16) |
			(1 << 15) |
			(1 << 3);
		disp_reg_out(DISPC_CONTROL, val);

		ret = reset_dsi_proto_eng();
		if (ret != RESET_COMPLETED)
			return OMAP_DAL_ERROR;

		val = in_regl(DSI_SYSCONFIG);
		val = val & ~(1 << 0);
		out_regl(DSI_SYSCONFIG, val);

		/* DSI interface must be disabled before configuration change */
		disable_omap_dsi_interface();

		/* Virtual channels muest be disabled before configuration */
		disable_video_mode_vc();
		disable_cmd_mode_vc();

		/* Configure the RX and TX fifo */
		config_dsi_fifo(4, 1); /*TODO: Need to be checked once again */

		/* enable DSI IRQ */ /* moved here by Jis */
		out_regl(DSI_IRQSTATUS, 0x001FFFFF);
		out_regl(DSI_IRQENABLE, 0x00);
	}
	fint = 2; /* ~2Mhz is the internal clock to PLL */

	/* Check if PLL input is more than 32MHz */
	if (clkin > 32)
		highfreq = 1;
	else
		highfreq = 0;

	dsiphy_clk = 296;
	regn = (clkin < 32) ? ((clkin / fint) - 1) : ((clkin / (2 * fint)) - 1);
	regm = ((dsiphy_clk) * (highfreq + 1) * (regn + 1)) / (2 * clkin);

	/* Need to configure the HSDIVIDER only if required */
	regm3 = (dsiphy_clk / logic_clk) - 1;
	regm4 = (dsiphy_clk / dsi2_pll_clk) - 1;
	/* this is a hack to make sure M3 and M4 values will not be zero when
	 * the DSIPHY clock is low value */
	(regm3 == 0) ? (regm3++) : (regm3);
	(regm4 == 0) ? (regm4++) : (regm4);

	ret = dsi_pll_pwr_cmd(PWR_CMD_STATE_ON_DIV);

	if (ret != OMAP_DAL_SUCCESS)
		goto ret;

	/* TODO: need to check what all things to
	be taken care before starting programming */
	/* Check the reset status of the PLL */
	while ((dsi_pll_reset_status() != RESET_COMPLETED) && (--count))
	    udelay(100);

	/* Timeout happened */
	if (count == 0)
		return OMAP_DAL_ERROR;

	/* TODO: need to decide what all things to be enabled */
	ret = dsi_pll_pwr_cmd(PWR_CMD_STATE_ON_DIV);
	if (ret != OMAP_DAL_SUCCESS)
		goto ret;


	/* Lock the PLL */
	ret = lock_dsi_pll(config_mode,
				pclk_or_sysclk,
				clkin,
				regn,
				regm,
				regm3,
				regm4);

	if (ret != OMAP_DAL_SUCCESS)
		goto ret;

	switch_to_dsipll_clk_source(); /* debugging */
	val = disp_reg_in(DISPC_CONTROL);
		val &=  ~((1 << 27));
	disp_reg_out(DISPC_CONTROL, val);

ret:
	return ret;
}
EXPORT_SYMBOL(config_dsipll_lclk);

/*-----------------------------------------------------------------------------
| Function    : U8 dsi_pll_pwr_cmd(U8 cmd)
+------------------------------------------------------------------------------
| Description : This function is used to change the
|		power states of PLL and HSDIVIDER
| Parameters  : cmd - power command to be send
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 dsi_pll_pwr_cmd(U8 cmd)
{
	U32 val, count = 10000;

	/* send the power command */
	val = in_regl(DSI_CLK_CTRL);
	val = ((val & ~(3 << 30)) | (cmd << 30));
	out_regl(DSI_CLK_CTRL, val);

	/* Check whether the power status is changed */
	do {
		val = in_regl(DSI_CLK_CTRL);
		val = ((val & 0x30000000) >> 28);
		udelay(100);
	} while ((val != cmd) && (--count));

	if (count == 0) {
		/* Timeout happened, something seriously WRONG!! */
		return OMAP_DAL_ERROR;
	} else {
		return OMAP_DAL_SUCCESS;
	}
}

/*-----------------------------------------------------------------------------
| Function    : U8 dsi_pll_reset_status()
+------------------------------------------------------------------------------
| Description : Returns the reset status of the DSI PLL controller
|
| Parameters  : None
| Returns     : Return value (RESET_COMPLETED or NOT_COMPLETED)
+----------------------------------------------------------------------------*/
U8 dsi_pll_reset_status(void)
{
	U32 val;
	val = in_regl(DSI_PLL_STATUS);
	/* Check the RESET done bit */
	if (val & 0x00000001)
		return RESET_COMPLETED;
	else
		return RESET_NOT_COMPLETED;
}

/*-----------------------------------------------------------------------------
| Function    : S32 lock_dsi_pll(U8 mode, U8 pclk_or_sysclk, U16 regn,
|		U16 regm, U16 regm3, U16 regm4)
+------------------------------------------------------------------------------
| Description : This function configure the DSI PLL with the given
|		divisor values and lock the PLL.
| Parameters  : mode - mode of PLL update (manual or automatic)
|		pclk_or_sysclk - Input reference clock to PLL (SYSCLK or PCLK)
|		clkin	- Input clock to PLL
|		regn	- Divisor N for PLL
|		regm	- Divisor M for PLL
|		regm3	- HSDIV for DSI protocol engine clk
|		regm4	- HSDIV for DSS functional clk
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 lock_dsi_pll(U8 mode, U8 pclk_or_sysclk, U16 clkin, U16 regn, U16 regm, \
			U16 regm3, U16 regm4)
{
	/* Internal PLL frequency is 1.75MHz to 2.1MHz */
	U32 val, count = 1000, fint = 7;

	if (mode == MANUAL_MODE) {
#ifdef CONFIG_FB_OMAP_720P_STREAMING
		regn = 25;
		regm = 297;
		regm3 = 1;
		regm4 = 1;
		fint = 3;
#endif

		/* Select the manual mode of PLL update */
		/* TODO: this function can written better
		 * to reduce the number of reads */
		val = in_regl(DSI_PLL_CONTROL);
		val = val & ~(1 << 0);
		out_regl(DSI_PLL_CONTROL, val);

		/* DSIPHY clock is disabled and HSDIV in bypass mode */
		val = in_regl(DSI_PLL_CONFIGURATION2);
		val = val & ~(1 << 14);
		val = val | (1 << 20);
		out_regl(DSI_PLL_CONFIGURATION2, val);

		if (pclk_or_sysclk == PCLK) {
			/* Input reference clock to PLL is PCLK */
			val = in_regl(DSI_PLL_CONFIGURATION2);
			val = val | (1 << 11);
			out_regl(DSI_PLL_CONFIGURATION2, val);
		} else if (pclk_or_sysclk == SYSCLK) {
			/* Input reference clock to PLL is SYSCLK (DSS2_FCLK) */
			val = in_regl(DSI_PLL_CONFIGURATION2);
			val = val & ~(1 << 11);
			out_regl(DSI_PLL_CONFIGURATION2, val);
		}

		/* Enable the HIGHFREQ divider if input clock
		 * is greater than 32MHz  */
		if (clkin > 32) {
			val = in_regl(DSI_PLL_CONFIGURATION2);
			val = val | (1 << 12);
			out_regl(DSI_PLL_CONFIGURATION2, val);
		} else {
			val = in_regl(DSI_PLL_CONFIGURATION2);
			val = val & ~(1 << 12);
			out_regl(DSI_PLL_CONFIGURATION2, val);
		}

		/* Configure the divisor values */
		val = in_regl(DSI_PLL_CONFIGURATION1);
		val = val |
			  (regm4 << 23) |
			  (regm3 << 19) |
			  (regm << 8) |
			  (regn << 1) |
			  (1 << 0);

		out_regl(DSI_PLL_CONFIGURATION1, val);

		if (regm3 != 0) {
			/*Enable the DSI proto engine clock divider from HSDIV*/
			val = in_regl(DSI_PLL_CONFIGURATION2);
			val = val | (1 << 18);
			out_regl(DSI_PLL_CONFIGURATION2, val);
		}

		if (regm4 != 0) {
			/* Enable the DSS clock divider from HSDIV  */
			val = in_regl(DSI_PLL_CONFIGURATION2);
			val = val | (1 << 16);
			out_regl(DSI_PLL_CONFIGURATION2, val);
		}

		/* PLL internal reference clock is - 1.75 to 2.1MHz */
		val = in_regl(DSI_PLL_CONFIGURATION2);
		val = val | (fint << 1);
		out_regl(DSI_PLL_CONFIGURATION2, val);

		/* Enable PLL reference clock  */
		val = in_regl(DSI_PLL_CONFIGURATION2);
		val = val | (1 << 13);
		out_regl(DSI_PLL_CONFIGURATION2, val);

		/* Enable DSIPHY clock and HSDIV in normal mode */
		val = in_regl(DSI_PLL_CONFIGURATION2);
#ifndef CONFIG_FB_OMAP_720P_STREAMING
		val = val | (1 << 14);
#endif
		val = val & (~(1<<20));
		out_regl(DSI_PLL_CONFIGURATION2, val);

		/* Start the PLL locking by setting PLL GO */
		val = 1;
		out_regl(DSI_PLL_GO, val);

		/* Waiting for the lock request to be issued to PLL */
		while (((in_regl(DSI_PLL_GO)) != 0) && (--count)) {
			if (!(count % 100))
				udelay(100);
		}

		if (count == 0) {
			/* lock request timed out */
			return OMAP_DAL_ERROR;
		}

		count = 100;
		/* Waiting for the PLL to be locked */
		while (((in_regl(DSI_PLL_STATUS) & 0x02) != 0x02) && (--count))
		    udelay(100);

		if (count == 0) {
			/* Locking timed out */
			return OMAP_DAL_ERROR;
		}
	} else {
		/* Select the automatic mode. */
		/* Not sure, what needs to be done here */
		val = in_regl(DSI_PLL_CONTROL);
		val = val | 0x00000001;
		out_regl(DSI_PLL_CONTROL, val);
		/* TODO: fill it here what needs to be done in automatic mode*/
	}

	return OMAP_DAL_SUCCESS;
}

/*-----------------------------------------------------------------------------
| Function    : reset_dsi_proto_eng(void)
+------------------------------------------------------------------------------
| Description : This function to reset the DSI protocol engine
|
| Parameters  : None
| Returns     : Return value (RESET_COMPLETED or NOT_COMPLETED)
+----------------------------------------------------------------------------*/
U8 reset_dsi_proto_eng(void)
{
	U32 val, count = 10000;
	/* Reset the DSI protocol engine */
	val = in_regl(DSI_SYSCONFIG);
	val = val | (1 << 1);
	out_regl(DSI_SYSCONFIG, val);

	/* Wait for reset to complete */
	while ((in_regl(DSI_SYSSTATUS) != 0x1) && (--count))
	    udelay(100);


	count = 10000;
	if (count != 0)
		return RESET_COMPLETED;
	else
		return RESET_NOT_COMPLETED;
}

/*-----------------------------------------------------------------------------
| Function    :void dsi_video_mode(BOOLEAN flag, U8 vc, U16 pixel_per_line,
|		U8 pixel_format)
+------------------------------------------------------------------------------
| Description : This function used to enable and disable the Video mode
|		interface and start sending the Video pixels over the DSI
|		interface to DSI device
|
| Parameters  : flag - Video mode enable/disable flag
|		vc   - virtual channel value
|		pixel_per_line - number of pixels per line of LCD
|		pixel_format - pixel format (RGB565 or RGB666)
| Returns     : None
+----------------------------------------------------------------------------*/
void dsi_video_mode(BOOLEAN flag, U8 vc, U16 pixel_per_line, U8 pixel_format)
{
	U32 header;
	/* U32 val; */
	U8 data_type;
	U16 word_count;
	/* TODO:need to enabled DSI as well as the LCD power in this function */
	if (flag == ENABLE_DSI_OUTPUT) {

		/* Disable the LCD interface so that no data will
		* be send to Video Port */
/*		disable_dispc_output(); */

		/* Disable the DSI interface */
/*		disable_omap_dsi_interface(); */

		/* disable the Virtual channel */
/*		disable_video_mode_vc(); */

		/* setup the Long packet header */
		data_type = (pixel_format == RGB565) ? 0x0E : 0x2E;
		/* Word count is equal to 2 * PPL (RGB565),
		 * in case of RGB666, word count = 3 * PPL  */
		word_count = (pixel_format == RGB565) ? (pixel_per_line * 2) :\
				(pixel_per_line * 3);
		/* form the DSI video mode packet header */
		header = (0 << 24)|
			(word_count << 8)|/* word count */
			(vc << 6)|/* virtual channel to talk to DSI device */
			(data_type << 0);	/* video mode data type */

		out_regl(DSI_VC0_LONG_PACKET_HEADER+(video_vc * 0x20), header);

		disable_cmd_mode_vc();
		disable_omap_dsi_interface();;
		enable_video_mode_vc();
		/* enable the DSI interface */
		enable_omap_dsi_interface();

		/* Enable the LCD interface so that data will be
		 * send to the Video port */
		enable_dispc_output();

	} else if (flag == DISABLE_DSI_OUTPUT) {

		/* Disable the LCD interface so that no data will
		 * be send to Video Port */
		disable_dispc_output();
		/* disable the Virtual channel */
		disable_video_mode_vc();
	}
}

/*-----------------------------------------------------------------------------
| Function    : config_dsi_interface(T_DSI_DIS *dsi_dis)
+------------------------------------------------------------------------------
| Description : Function to configure the DSI interface of OMAP3430
|
| Parameters  : dsi_dis - pointer to DSI device information structure
|
| Returns     : Status (OMAP_DAL_SUCCESS or Error)
+----------------------------------------------------------------------------*/
S32 config_dsi_interface(T_DSI_DIS *dsi_dis)
{
	S32 ret = OMAP_DAL_SUCCESS;

	/* Configure DISPC interface */
	ret = config_disp_controller(dsi_dis);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

	/* Configure the DSI interface */
	ret = config_dsi_proto_engine(dsi_dis);

	if (ret != OMAP_DAL_SUCCESS)
		return ret;

	return ret;
}

/*-----------------------------------------------------------------------------
| Function    :void disable_dsi_interface()
+------------------------------------------------------------------------------
| description : function to disable dsi interface of omap3430
|
| parameters  : none
|
| returns     : none
+----------------------------------------------------------------------------*/
void disable_dsi_interface()
{
	/* disable the dsi interface */
	disable_omap_dsi_interface();

	/* switch back to the default clock source */
	switch_to_alwon_fclk_source();

	/* reset dsi protocol engine */
	reset_dsi_proto_eng();
	/* reset dispc */
}

/*-----------------------------------------------------------------------------
| Function    : config_dsi_proto_engine(T_DSI_DIS *dsi_dis)
+------------------------------------------------------------------------------
| Description : Function to configure the DSI protocol engine of OMAP3430
|
| Parameters  : dsi_dis - pointer to DSI device information structure
|
| Returns     : Status (OMAP_DAL_SUCCESS or Error)
+----------------------------------------------------------------------------*/
S32 config_dsi_proto_engine(T_DSI_DIS *dsi_dis)
{
	U32 val, ret = OMAP_DAL_SUCCESS;
	U8 bits_per_pixel = 0;

	/* Reset the DSI protocol engine */
	ret = reset_dsi_proto_eng();
	if (ret != RESET_COMPLETED)
		return OMAP_DAL_ERROR;

	val = in_regl(DSI_SYSCONFIG);
	val = val & ~(1<<0);
	out_regl(DSI_SYSCONFIG, val);

	/* Make sure the DSI interface is disabled before configuration change*/
	disable_omap_dsi_interface();

	/* Make sure the Virtual channels are disabled before configuration */
	disable_video_mode_vc();
	disable_cmd_mode_vc();

	/* Configure the video port */
	config_video_port(dsi_dis->init.output_intf_config.vsync_pol,
				dsi_dis->init.output_intf_config.dataen_pol,
				dsi_dis->init.output_intf_config.hsync_pol,
				dsi_dis->init.output_intf_config.clk_pol,
				dsi_dis->init.output_intf_config.pixel_format,
				RX_ECC_CHECK_DISABLE,
				RX_CHECKSUM_CHECK_DISABLE);


	/* Configure the Video mode timing */
	config_video_mode_timing(dsi_dis->init.video_mode_timing);


	/* Configure the Video mode and command mode,  use VC0 for Video
	 * Mode transfer and VC1 for Command Mode */
	config_video_mode_channel(video_vc,
					dsi_dis->init.ecc_flag,
					dsi_dis->init.checksum_flag);

	config_command_mode_channel(cmd_vc);

	/* Configure the RX and TX fifo */
	config_dsi_fifo(4, 1); /*TODO: Need to be checked once again */

	/* enable DSI IRQ */ /* moved here by Jis */
	out_regl(DSI_IRQSTATUS, 0x001FFFFF);
	out_regl(DSI_IRQENABLE, 0x00);
	/* out_regl(DSI_IRQENABLE, 0x001FFF00); */

	if (dsi_dis->init.output_intf_config.pixel_format == RGB565)
		bits_per_pixel = 16;
	else if (dsi_dis->init.output_intf_config.pixel_format == RGB666)
		bits_per_pixel = 18;
	else if (dsi_dis->init.output_intf_config.pixel_format == RGB888)
		bits_per_pixel = 24;


	/* Configure the DSI PLL */
	ret = config_dsipll(dsi_dis->init.output_intf_config.pixel_clk,
			bits_per_pixel,
			dsi_dis->init.num_data_lanes,
			SYSCLK, /* System clock as the input to DSI module*/
			get_sysclk(), /* system clock value */
			HSDIV_ENABLE,
			MANUAL_MODE);

	if (ret != OMAP_DAL_SUCCESS)
			return ret;

	/* Select the DSS1 Functional clock and DSI
	 * Functional clock from DSI PLL */
	switch_to_dsipll_clk_source(); /* debugging */
	/* Configure the Complex IO */
	ret = config_dphy(dsi_dis);
	if (ret != OMAP_DAL_SUCCESS)
		return ret;

	/* Configure the DSI timings */
	config_dsi_timers();

	enable_omap_dsi_interface();
	enable_cmd_mode_vc();

	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : switch_to_dsipll_clk_source(void)
+------------------------------------------------------------------------------
| description : function to switch the DSI protocol engine FCLK and DISPC FCLK
|		to DSI PLL HSDIVIDER output clocks
|
| parameters  : none
|
| returns     : none
+----------------------------------------------------------------------------*/
void switch_to_dsipll_clk_source(void)
{
	U32 val;
	/*Select the DISPC functional clock as DSI_PLL1 clock and DSI functional
	 *clock as DSI_PLL2 clock
	 */
	val = dss_reg_in(DSS_CONTROL);
	val = val | (1 << 1) | (1 << 0);
	dss_reg_out(DSS_CONTROL, val);
}

/*-----------------------------------------------------------------------------
| Function    : switch_to_alwon_fclk_source(void)
+------------------------------------------------------------------------------
| description : function to switch the DSI protocol engine FCLK and DISPC FCLK
|		to default configuration (DSS1 and DSS2 always ON functional
|		clocks)
| parameters  : none
|
| returns     : none
+----------------------------------------------------------------------------*/
void switch_to_alwon_fclk_source(void)
{
	U32 val;
	/* Switch to the DSS1_ALWON_FCLK and DSS2_ALWON_FCLK sources */
	val = dss_reg_in(DSS_CONTROL);
	val = val & ~((1 << 1) | (1 << 0));
	dss_reg_out(DSS_CONTROL, val);
}

/*-----------------------------------------------------------------------------
| Function    : send_dsi_trigger(void)
+------------------------------------------------------------------------------
| description : function to send the reset trigger message to the DSI device
|
| parameters  : none
|
| returns     : Status (OMAP_DAL_SUCCESS or Error)
+----------------------------------------------------------------------------*/
S32 send_dsi_trigger(void)
{
	U32 val, count = 10000;
	/* Make sure the DSI interface is disabled before configuration change*/
	disable_omap_dsi_interface();

	/* Enable sending then Reset Trigger */
	val = in_regl(DSI_CTRL);
	val = val | (1 << 5);
	out_regl(DSI_CTRL, val);

	enable_omap_dsi_interface();

	/* Wait for the Reset Trigger generation complete */
	do {
		val = in_regl(DSI_CTRL);
		udelay(100);
	} while ((val & 0x00000020) && (--count));

	if (count != 0)
		return OMAP_DAL_SUCCESS;
	else
		return OMAP_DAL_ERROR;

}

/*-----------------------------------------------------------------------------
| Function    : config_video_mode_timing(T_VM_TIMING vm_timing)
+------------------------------------------------------------------------------
| description : function to configure the Video Mode timings of DSI interface
|
| parameters  : vm_timing - structure which has the VM timing parameters
|
| returns     : none
+----------------------------------------------------------------------------*/
void config_video_mode_timing(T_VM_TIMING vm_timing)
{
	U32 val;
	/* this is as per the TRM */
	U8 window_sync = 4; /*TODO: need to check this again */

	val = in_regl(DSI_CLK_CTRL);

/*#ifndef SILVAL_CONFIG*/
	val = val & ~(0x00001FFF);
	val = val |
		(1 << 21) |
		(1 << 20) |
		(1 << 18) |
		(0 << 16) |
#ifdef CMD_VC1
		(0 << 15) |  /* 0 -> NULL packet generation disabled*/
#else
		(1 << 15) |  /* 1 -> NULL packet generation enabled*/
#endif
		(1 << 14) |
		(0 << 13) |
		(vid_lp_clk_div << 0); /* 50/7 => 7MHz */
	out_regl(DSI_CLK_CTRL, val);

	/* configure the Video mode timing parameters (the values need to be
	 * taken from DSI device datasheet (eDISCO's))*/
	val =	(vid_hsa << 24) |
		(vid_hfp << 12) |
		(vid_hbp << 0);
	out_regl(DSI_VM_TIMING1, val);

	val = (window_sync << 24) |
		(vid_vsa << 16) |
		(vid_vfp << 8) |
		(vid_vbp << 0);
	out_regl(DSI_VM_TIMING2, val);

	/* LT = 4 + 6 + 960 + 6 + 3 + 3 */
	val = (vid_tl << 16) |
	      (vm_timing.vact << 0);
	out_regl(DSI_VM_TIMING3, val);

	/* TODO: need to see any other information need to be
	 * programmed for Video Mode */
	val =	(0x48 << 16) |
		(0x72 << 8) |
		(0x96 << 0);
	out_regl(DSI_VM_TIMING4, val);

	val = (0x82 << 16) |
	      (0xDF << 8) |
	      (0x3B << 0);
	out_regl(DSI_VM_TIMING5, val);

	val = (0x7A67 << 16) |
		  (0x31D1 << 0);
	out_regl(DSI_VM_TIMING6, val);

	val = (enter_hs_mode_latency << 16) |
	      (exit_hs_mode_latency << 0);
	out_regl(DSI_VM_TIMING7, val);

}

/*-----------------------------------------------------------------------------
| Function    : config_dsi_timers()
+------------------------------------------------------------------------------
| description : function configure the DSI protocol timers and timeouts
|
| parameters  : none
|
| returns     : none
+----------------------------------------------------------------------------*/
void config_dsi_timers()
{
	U32 val;
	BOOLEAN turn_around_timer, force_tx_stop_mode_timer,\
				hs_tx_timer, lp_rx_timer;
	U16 ta_to_counter, stop_state_counter, hs_tx_to_counter,\
				lp_rx_to_counter, ddr_clk_pre, ddr_clk_post;
	U8 ta_to_x, stop_state_x, hs_tx_to_x, lp_rx_to_x;

	/* Timeout counter initialization */
	/*TODO: need to cross check this again */
	/*TODO: need to check the mutliplcation value also  */
	turn_around_timer = TIMER_DISABLE;
	ta_to_x	= 16;
	ta_to_counter = 0x1919;
	force_tx_stop_mode_timer = TIMER_ENABLE;
	stop_state_x = 16;
	stop_state_counter = 0x999;
	hs_tx_timer = TIMER_DISABLE;
	hs_tx_to_x = 16;
	hs_tx_to_counter = 0x0FD2;

	lp_rx_timer = TIMER_DISABLE;
	lp_rx_to_x = 16;
	lp_rx_to_counter = 0x00CD;

	/* Timer configuration */
	val = in_regl(DSI_TIMING1);
	if (turn_around_timer == TIMER_ENABLE) {
		val = val |
		((U32)1 << 31) |
			(((ta_to_x == 16) ? (1 << 30) : (1 << 29))) |
				(ta_to_counter << 16);
	} else {
		val = val & ~(1 << 31);
	}

	if (force_tx_stop_mode_timer == TIMER_ENABLE) {
		val = val |
		      (1 << 15) |
		      (((stop_state_x == 16) ? (1 << 14) : (1 << 13))) |
		      (stop_state_counter<<0);
	} else {
		val = val & ~(1 << 15);
	}

	out_regl(DSI_TIMING1, val);

	disable_cmd_mode_vc();
	enable_cmd_mode_vc();

	val = in_regl(DSI_TIMING2);
	if (hs_tx_timer == TIMER_ENABLE) {
		val = val |
		      ((U32)1 << 31) |
		      (((hs_tx_to_x == 16) ? (1 << 30) : (1 << 29))) |
		      (hs_tx_to_counter << 16);
	} else {
		val = val & ~(1<<31);
	}

	if (lp_rx_timer == TIMER_ENABLE) {
		val = val |
		      (1 << 15) |
		      (((lp_rx_to_x == 16) ? (1 << 14) : (1 << 13))) |
		      (lp_rx_to_counter << 0);
	} else {
		val = val & ~(1 << 15);
	}
	out_regl(DSI_TIMING2, val);

	/* config DDR CLK timer */
	/* Below values are not valid if the DDR_CLK is DSI_CLK_CTRL
	* ON (DDR_CLK_ALWAYS_ON bit is set) */
#ifndef SILVAL_CONFIG
	ddr_clk_pre = gddr_clk_pre;
	ddr_clk_post = gddr_clk_post;
#else
	ddr_clk_pre = gddr_clk_pre;
	ddr_clk_post = gddr_clk_post;
#endif


	val =	(ddr_clk_pre << 8) |
		(ddr_clk_post << 0);
	out_regl(DSI_CLK_TIMING, val);
}

/*-----------------------------------------------------------------------------
| Function    : config_video_port(U8 vs_pol, U8 de_pol, U8 hs_pol, U8 clk_pol,
|		U8 pixel_format, BOOLEAN ecc, BOOLEAN checksum)
+------------------------------------------------------------------------------
| description : Configure Video Port interface to DSI protocol engine
|
| parameters  : vs_pol - Polarity of the VSYNC
|		de_pol - Polarity of the Data Enable signal
|		hs_pol - Polarity of the HSYNC
|		clk_pol - Polarity of the PCLK
| 		pixel_format - Pixel format
|		ecc - ECC check enable or disable of DSI output interface (Rx)
|		checksum - Checksum check enable or disable flag of DSI output
|		interface (Rx)
|
| returns     : none
+----------------------------------------------------------------------------*/
void config_video_port(U8 vs_pol, U8 de_pol, U8 hs_pol, U8 clk_pol, \
		U8 pixel_format, BOOLEAN ecc, BOOLEAN checksum)
{
	U32 val;
/*	U8 vp_bus_width; */

	/* Make sure the DSI interface is disabled before configuration change*/

	/* configuration of the video port signals */
	val = in_regl(DSI_CTRL);
	/* hnagalla -- 050608 */
	val =	(0x1 << 23) |		/* HSA_BLANKING_MODE */
		(0x1 << 22) |		/* HBP_BLANKING_MODE */
		(0x1 << 21) |		/* HFP_BLANKING_MODE */
		(0x0 << 20) |		/* BLANKING_MODE */
		(0x0 << 18) |		/* VP_HSYNC_END */
		(0x1 << 17) |		/* VP_HSYNC_START */
		(0x0 << 16) |		/* VP_VSYNC_END */
		(0x1 << 15) |		/* VP_VSYNC_START */
		(0x1 << 14) |		/* TRIGGER_RESET_MODE */
		(0x2 << 12) |		/* LINE_BUFFER */
		(0x1 << 11) |		/* VP_VSYNC_POL */
		(0x0 << 10) |		/* VP_HSYNC_POL */
		(0x1 << 9) |		/* VP_DE_POL (VP data enable
					 * signal polarity) */
		(0x0 << 8) |		/* CP_CLK_POL (0x1 dsi captures data on
					 * the VP on pixel clock raising edge)*/
		(((pixel_format == RGB666) ?\
				(1) : (0)) << 6) |/*VP_DATA_BUS_WIDTH*/
		(0x0 << 5) |		/* TRIGGER_RESET */
		(0x0 << 4) |		/* VP_CLK_RATIO */
		(0x1 << 3) |		/* TX_FIFO_ARBITRATION */
		(0x1 << 2);		/* ECC_RX_EN */
	out_regl(DSI_CTRL, val);
}

/*-----------------------------------------------------------------------------
| Function    : config_video_mode_channel(U8 channel_id, U8 ecc, U8 checksum)
+------------------------------------------------------------------------------
| description : function to configure Video Mode virtual channel used
|
| parameters  : channel_id - Channel ID used for the Video Mode transfer
|		ecc - ECC check enable or disable of DSI output interface (Tx)
|		checksum - Checksum check enable or disable flag of DSI
|		output interface (Tx)
| returns     : none
+----------------------------------------------------------------------------*/
void config_video_mode_channel(U8 channel_id, U8 ecc, U8 checksum)
{
	U32 val;
	/* configure the VC channel to video mode (video mode,
	 * video port as input, HS, no DMA)*/
	val = in_regl(DSI_VC0_CTRL + (channel_id * 0x20));
	val = val |
		(4 << 27) |
		(4 << 21) |
		(1 << 9) |
		(1 << 4) |
		(0 << 1);

	/* ecc and checksum configuration */
	if (ecc == ECC_ENABLE)
		val = val | (1 << 8);

	if (checksum == CHECKSUM_ENABLE)
		val = val | (1 << 7);

	out_regl(DSI_VC0_CTRL + (channel_id * 0x20), val);

	/* Enable interrupt events for Video mode channel */
	/* Clear all the events*/
	out_regl(DSI_VC0_IRQSTATUS + (channel_id * 0x20), 0x000000FF);
	/* TODO -- to fixed */
	/* Enable all the events */
	out_regl(DSI_VC0_IRQENABLE + (channel_id * 0x20), 0x000000);
}


/*-----------------------------------------------------------------------------
| Function    : config_command_mode_channel(U8 channel_id)
+------------------------------------------------------------------------------
| description : function to configure Command Mode virtual channel used
|
| parameters  : channel_id - Channel ID used for the Command Mode transfer
|
| returns     : none
+----------------------------------------------------------------------------*/
void config_command_mode_channel(U8 channel_id)
{
	U32 val;
	/* configure the VC channel to command mode
	 * (command mode, L4 as input, HS, no DMA)*/
	val = in_regl(DSI_VC0_CTRL + (channel_id * 0x20));
	val = val & ~((1 << 4) | (1 << 1));
	val = val |
		(4 << 27) |
		(4 << 21) |
		(1 << 8) |
		(1 << 7) |
		(0 << 9);

	out_regl(DSI_VC0_CTRL+(channel_id*0x20), val);

	/* Enable interrupt events for Command mode channel */
	/* Clear all the events*/
	out_regl(DSI_VC0_IRQSTATUS+(channel_id*0x20), 0x000000FF);
	/* Enable all the events */
	out_regl(DSI_VC0_IRQENABLE+(channel_id*0x20), 0x000000);
}

/*-----------------------------------------------------------------------------
| Function    : enable_omap_dsi_interface()
+------------------------------------------------------------------------------
| description : function to enable the DSI interface of OMAP
|
| parameters  : none
|
| returns     : none
+----------------------------------------------------------------------------*/
void enable_omap_dsi_interface()
{
	U32 val;
	val = in_regl(DSI_CTRL);
	val = (val | (1 << 0));
	out_regl(DSI_CTRL, val);
}

/*-----------------------------------------------------------------------------
| Function    : disable_omap_dsi_interface()
+------------------------------------------------------------------------------
| description : function to disable the DSI interface of OMAP
|
| parameters  : none
|
| returns     : none
+----------------------------------------------------------------------------*/
void disable_omap_dsi_interface()
{
	U32 val;
	val = in_regl(DSI_CTRL);
	val = (val & ~(1 << 0));
	out_regl(DSI_CTRL, val);
}


/*-----------------------------------------------------------------------------
| Function    : config_dsi_fifo(U16 tx_fifo_len, U16 rx_fifo_len)
+------------------------------------------------------------------------------
| description : function to configure the DSI protocol engine TX and RX FIFO
|
| parameters  : tx_fifo_len - length of the TX fifo in x33bits
|		rx_fifo_len - length of the RX fifo in x33bits
|
| returns     : none
+----------------------------------------------------------------------------*/
void config_dsi_fifo(U16 tx_fifo_len, U16 rx_fifo_len)
{
	/*TODO: need to check whether we need to configure the other
	 * channel fifo as well as the start address for each channel */
	U32 val;
	val = (0 << 28) | 		/* VC3 Not used */
		(0 << 24) | 		/* VC3 Not used */
		(0 << 20) | 		/* VC2 Not used */
		(0 << 16) | 		/* VC2 Not used */
		(tx_fifo_len << 12) |	/* VC1 FIFO length */
		(0 << 8) |		/* VC1 FIFO start address */
		(tx_fifo_len << 4) |	/* VC0 FIFO length */
		(0 << 0);		/* VC0 FIFO start address */
	out_regl(DSI_TX_FIFO_VC_SIZE, val);
	val = (0 << 28) |		/* VC3 Not used */
	      (0 << 24) |		/* VC3 Not used */
	      (0 << 20) |		/* VC2 Not used */
	      (0 << 16) |		/* VC2 Not used */
	      (rx_fifo_len << 12) |	/* VC1 FIFO length */
	      (0 << 8) |		/* VC1 FIFO start address */
	      (rx_fifo_len << 4) | 	/* VC0 Not used for BTA */
	      (0 << 0);			/* VC0 Not used for BTA */
	out_regl(DSI_RX_FIFO_VC_SIZE, val);
}

/*-----------------------------------------------------------------------------
| Function    : config_disp_controller(T_DSI_DIS *dsi_dis)
+------------------------------------------------------------------------------
| Description : Function to configure the Display controller OMAP3430
|
| Parameters  : dsi_dis - pointer to DSI device information structure
|
| Returns     : Status (OMAP_DAL_SUCCESS or Error)
+----------------------------------------------------------------------------*/
S32 config_disp_controller(T_DSI_DIS *dsi_dis)
{
	S32 ret = OMAP_DAL_SUCCESS;

	/* enable functional and interface clock to DSS module  */

	/* Configure the DISPC timing parameters */
	config_dispc_timing(dsi_dis->init.output_intf_config);
	config_signal_pol(dsi_dis->init.output_intf_config);
	 /*TODO: we need to give correct value here */
	config_pixel_clock(dsi1_pll_clk, \
				dsi_dis->init.output_intf_config.pixel_clk);

	config_lcd_size(dsi_dis->init.output_intf_config);
	config_lcd_color(0x0000, 0x0000);
	config_dispc_out_interface(dsi_dis->\
					init.output_intf_config.pixel_format);

	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : config_dispc_timing(T_DPI_OUT_CONFIG lcd_config)
+------------------------------------------------------------------------------
| Description : Function to configure the Display controller timing parameters
|
| Parameters  : lcd_config - structure which contains the LCD timing parameters
|
| Returns     : None
+----------------------------------------------------------------------------*/
void config_dispc_timing(T_DPI_OUT_CONFIG lcd_config)
{
	U32 val;
#if 0
	/* configure horizontal timing parameters */
	val = ((lcd_config.hbp-1) << 20) |
	      ((lcd_config.hfp-1) << 8) |
	      ((lcd_config.hsa-1) << 0);
	out_regl(DISPC_TIMING_H, val);
	/* configure vertical timing parameters */
	val = (lcd_config.vbp << 20) |
	      (lcd_config.vfp << 8) |
	      ((lcd_config.vsa-1) << 0);
	out_regl(DISPC_TIMING_V, val);
#else
	/* configure horizontal timing parameters */
	val = ((dispc_hbp-1) << 20) |
	      ((dispc_hfp-1) << 8) |
	      ((dispc_hsa-1) << 0);
	disp_reg_out(DISPC_TIMING_H, val);
	/* configure vertical timing parameters */
	val = (vid_vbp << 20) |
		/* hnagalla -- 050608 */
		/* ((vid_vfp - 2) << 8) | */
		((dispc_vfp) << 8) |
		((vid_vsa-1) << 0);
	disp_reg_out(DISPC_TIMING_V, val);
#endif
	/* interconnect configuration */
	val = 	(1 << 12) |
		(1 << 3);
	disp_reg_out(DISPC_SYSCONFIG, val);
}

/*-----------------------------------------------------------------------------
| Function    : config_signal_pol(T_DPI_OUT_CONFIG lcd_config)
+------------------------------------------------------------------------------
| Description : Function to configure the Display controller Video
|		port signal polarity
| Parameters  : lcd_config - structure which contains the LCD timing parameters
|
| Returns     : None
+----------------------------------------------------------------------------*/
void config_signal_pol(T_DPI_OUT_CONFIG lcd_config)
{
	U32 val;
	/* TODO: need to be configured dynamically */
	/* For Sharp LCD, Sync and Data a sampled in rising edge of clock*/
	/* This configuration is not so important because DISPC does not
	 * interfac with the LCD directly*/
	/* Polarity will be taken care by the eDSICO' DPI output
	 * interface configuration */
	val = (1 << 17) | (1 << 16);
	disp_reg_out(DISPC_POL_FREQ, val);
}

/*-----------------------------------------------------------------------------
| Function    : config_lcd_size(T_DPI_OUT_CONFIG lcd_config)
+------------------------------------------------------------------------------
| Description : Function to configure the size of LCD configuration in DISPC
|
| Parameters  : lcd_config - structure which contains the LCD timing parameters
|
| Returns     : None
+----------------------------------------------------------------------------*/
void config_lcd_size(T_DPI_OUT_CONFIG lcd_config)
{
	U32 val;
	/* val = ((lcd_config.vact-1)<<16) | */
	/* hnagalla --050608  HW recommended fix for GOLCD issue */
	val = ((lcd_config.vact - 1 + 2) << 16) |
	      ((lcd_config.hact - 1) << 0);
	disp_reg_out(DISPC_SIZE_LCD, val);
}

/*-----------------------------------------------------------------------------
| Function    : config_pixel_clock(U32 dss_func_clk, U32 pixel_clk)
+------------------------------------------------------------------------------
| Description : Function to configure pixel clock divisor of
|		DISPC's LCD interface
| Parameters  : dss_func_clk - DISPC input functional clock value
|		pixel_clk    - lcd interface pixel clock frequency
|
| Returns     : None
+----------------------------------------------------------------------------*/
void config_pixel_clock(U32 dss_func_clk, U32 pixel_clk)
{
	U32 val, pixel_clk_div;
	/*U32 dss_clk_div = 1;*/
	pixel_clk_div = dss_func_clk/pixel_clk;

#ifdef NB_DATALANES_1
	val = 0x10004;
#else
#ifndef RGB18_IMAGE
#ifdef CONFIG_OMAP_DSI
	val = 0x10004;
#else
	val = 0x10002;
#endif
#else
	val = 0x10005;
#endif
#endif

	disp_reg_out(DISPC_DIVISOR, val);
}

/*-----------------------------------------------------------------------------
| Function    : config_lcd_color(U32 background_color, U32 transparent_color)
+------------------------------------------------------------------------------
| Description : Function to set default background color and
|		transparent color settings for LCD interface
| Parameters  : background_color     - LCD interface background color value
|	        transparent_color    - LCD interface transparent color value
|
| Returns     : None
+----------------------------------------------------------------------------*/
void config_lcd_color(U32 background_color, U32 transparent_color)
{
	U32 val;
	val = (background_color << 0);
	disp_reg_out(DISPC_DEFAULT_COLOR0, val);

	val = (transparent_color << 0);
	disp_reg_out(DISPC_TRANS_COLOR0, val);
}

/*-----------------------------------------------------------------------------
| Function    : config_dispc_out_interface(U8 pixel_format)
+------------------------------------------------------------------------------
| Description : Function  which does the DISPC general configurations
|
| Parameters  : pixel_format    - pixel format
|
| Returns     : None
+----------------------------------------------------------------------------*/
void config_dispc_out_interface(U8 pixel_format)
{
	U32 val;

	/* DISPC configuation */
	val = 	(0x2 << 1);
	disp_reg_out(DISPC_CONFIG, val);

	val =	(1 << 28) |
		(1 << 27) |
		(1 << 16) |
		(1 << 15) |
		(((pixel_format == RGB666) ? (0x2 << 8) : (0x1 << 8))) |
		(1 << 3);
	disp_reg_out(DISPC_CONTROL, val);
}

/*-----------------------------------------------------------------------------
| Function    : send_dsi_packet(U8 data_type, U8 vc, U8 operation,
|		U16 num_bytes, U8 *in_buf, BOOLEAN mode, BOOLEAN ecc,
|		BOOLEAN checksum, U8 *out_buf, U32 len)
+------------------------------------------------------------------------------
| Description : Function for sending the packets to the DSI device
|		in command mode
|
| Parameters  : data_type    - DSI packet data type
|		vc	     - virtual channel value
|		operation    - read or write request
|		num_bytes    - number of bytes to be send
|		in_buf	     - command parameter buffer
|		mode	     - mode of transfer (HS or LPS)
|		ecc   	     - ecc enable/disable flag for TX
|		checksum     - checksume enable/disable flag for TX
|		out_buf      - return buffer pointer in case of read
|		len	     - length of the return buffer
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 send_dsi_packet(U8 data_type, U8 vc, U8 operation, U16 num_bytes, \
			U8 *in_buf, BOOLEAN mode, BOOLEAN ecc, BOOLEAN \
				checksum, U8 *out_buf, U32 len)
{
	U32 val, count = 10000;
	S32 ret;
	static U8 video_packets;

#ifndef VIDEO_MODE_TX
	if (in_buf[0] == DCS_WRITE_MEMORY_START) {
		/* Send VSync packet */
			ret = send_short_packet(0x01,
				vc,
				0x00,
				0x00,
				mode,
				ecc);

		if (ret != OMAP_DAL_SUCCESS)
			return ret;

		/* Send HSync packet */
		ret = send_short_packet(0x21,
				vc,
				0x00,
				0x00,
				mode,
				ecc);

		if (ret != OMAP_DAL_SUCCESS)
			return ret;

		ret = send_long_packet(data_type,
				vc,
				num_bytes,
				in_buf,
				mode,
				ecc,
				checksum);

		if (ret != OMAP_DAL_SUCCESS)
			return ret;

		video_packets++;
    } else if (in_buf[0] == DCS_WRITE_MEMORY_CONTINUE) {
		if ((video_packets % 2) == 0) {
			/* Send HSync packet */
			ret = send_short_packet(0x21,
						vc,
						0x00,
						0x00,
						mode,
						ecc);

			if (ret != OMAP_DAL_SUCCESS)
				return ret;

		}

		ret = send_long_packet(data_type,
					vc,
					num_bytes,
					in_buf,
					mode,
					ecc,
					checksum);

		if (ret != OMAP_DAL_SUCCESS)
			return ret;

		video_packets++;
	} else
#endif
	{
		/* Decide the packet type: long or short packet */
		if (num_bytes > 2) {
			ret = send_long_packet(data_type,
					vc,
					num_bytes,
					in_buf,
					mode,
					ecc,
					checksum);
		} else {
			ret = send_short_packet(data_type,
					vc,
					in_buf[0],
					((num_bytes > 1) ? in_buf[1] : 0),
					mode,
					ecc);
		}
	}

	if (ret != OMAP_DAL_SUCCESS)
			return ret;

	if (operation == READ_CMD) {
#ifdef BTA_ENABLE
		/*TODO: this section need to be updated
		 * based on inputs from Fred */
		/* If the command is a read command,
		 * BTA needs to be initiated */
		val = in_regl(DSI_VC0_CTRL + (cmd_vc * 0x20));
		val = val | (1 << 6);
		out_regl(DSI_VC0_CTRL + (cmd_vc * 0x20), val);

		count = 10000;
		/* Check the BTA generation is completed  */
		do {
			val = in_regl(DSI_VC0_CTRL + (cmd_vc * 0x20));
		udelay(100);
		} while ((val & 0x00000040) && (--count));


		if (count == 0) {
			/* Clear BTA */
			val = val | (1 << 6);
			out_regl(DSI_VC0_CTRL + (cmd_vc * 0x20), val);
			return OMAP_DAL_ERROR;
		}

		count = 10000;
		/* Check the BTA event is active  */
		do {
			val = in_regl(DSI_VC0_IRQSTATUS+(cmd_vc*0x20));
		udelay(100);
		} while ((!(val & 0x00000020)) && (--count));

		if (count == 0)
			return OMAP_DAL_ERROR;

		/* Clear the irq status bit */
		out_regl(DSI_VC0_IRQSTATUS + (cmd_vc * 0x20), 0x00000020);


		count = 10000;
		/* Check the whether RX FIFO is not empty */
		do {
			val = in_regl(DSI_VC0_CTRL + (cmd_vc * 0x20));
		udelay(100);
		} while (!(val & 0x00100000) && (--count));

		if (count == 0)
			return OMAP_DAL_ERROR;

		do {
			*(U32 *)out_buf = in_regl(DSI_VC0_SHORT_PACKET_HEADER\
						+ (cmd_vc * 0x20));
			out_buf = out_buf + 4;
			len = len - 4;

			count = 10000;
			do {
				val = in_regl(DSI_VC0_CTRL + (cmd_vc * 0x20));
			udelay(100);
			} while (!(val & 0x00100000) && (--count));
		} while ((val & 0x00100000) && (len > 0));
#endif
	}
	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : send_long_packet(U8 data_type, U8 vc, U16 word_count,
|		U8 *buf, BOOLEAN mode, BOOLEAN ecc, BOOLEAN checksum)
+------------------------------------------------------------------------------
| Description : Function for sending the long packets to the DSI device
|		in command mode
|
| Parameters  : data_type    - DSI packet data type
|		vc	     - virtual channel value
|		word_count    - number of bytes to be send
|		buf	     - command parameter buffer
|		mode	     - mode of transfer (HS or LPS)
|		ecc   	     - ecc enable/disable flag for TX
|		checksum     - checksume enable/disable flag for TX
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 send_long_packet(U8 data_type, U8 vc, U16 word_count, U8 *buf, \
			BOOLEAN mode, BOOLEAN ecc, BOOLEAN checksum)
{
	U32 val, header = 0, count = 10000;
	U16 cnt, rem, i;
	/* Configure the Virtual Channel */

	disable_cmd_mode_vc();

	/* Speed selection (HS or LPS) */
	val = in_regl(DSI_VC0_CTRL + (cmd_vc * 0x20));
	if (mode == DSI_HS_MODE_TX)
		val = val | (1 << 9);
	else if (mode == DSI_LPS_MODE_TX)
		val = val & ~(1 << 9);

	out_regl(DSI_VC0_CTRL + (cmd_vc * 0x20), val);

	enable_cmd_mode_vc();

	/* Set Long packet header */

	header = (0 << 24)|
			(word_count << 8)|
			(vc << 6)|
			(data_type << 0);
	out_regl(DSI_VC0_LONG_PACKET_HEADER + (cmd_vc * 0x20), header);

	/* decide the packet length */
	cnt = word_count / 4;
	rem = word_count % 4;

	/* fill the packet payload with 32bit words */
	for (i = 0; i < cnt; i++) {
		out_regl(DSI_VC0_LONG_PACKET_PAYLOAD +\
				(cmd_vc * 0x20), *(U32 *)buf);
		buf = buf + 4;
	}

	/* fill the packet payload with remaining bytes */
	for (i = 0, val = 0; i < rem; i++)
		val = val | ((*(buf + i)) << (i * 8));


	out_regl(DSI_VC0_LONG_PACKET_PAYLOAD + (cmd_vc * 0x20), val);

	do {
		val = in_regl(DSI_VC0_IRQSTATUS + (cmd_vc * 0x20));
	} while ((!(val & 0x00000004)) && (--count));

	if (count != 0) {
		/* clear the irq status bit */
		out_regl(DSI_VC0_IRQSTATUS + (cmd_vc * 0x20), 0x00000004);
		return OMAP_DAL_SUCCESS;
	} else {
		/* transmission timed out */
		return OMAP_DAL_ERROR;
	}
}

/*-----------------------------------------------------------------------------
| Function    : send_short_packet(U8 data_type, U8 vc, U8 data0, U8 data1,
|		BOOLEAN mode, BOOLEAN ecc)
+------------------------------------------------------------------------------
| Description : Function for sending the long packets to the DSI device
|		in command mode
|
| Parameters  : data_type    - DSI packet data type
|		vc	     - virtual channel value
|		data0	     - first data byte to short packet
|		data1	     - second data byte to short packet
|		mode	     - mode of transfer (HS or LPS)
|		ecc   	     - ecc enable/disable flag for TX
|
| Returns     : Return value (OMAP_DAL_SUCCESS or error)
+----------------------------------------------------------------------------*/
S32 send_short_packet(U8 data_type, U8 vc, U8 data0, U8 data1, BOOLEAN mode, \
			BOOLEAN ecc)
{
	U32 val, header = 0, count = 10000;

	/* Configure the Virtual Channel */
	/*TODO: do we need to disable/enable the VC for every command? */
	disable_cmd_mode_vc();
	/* disable_omap_dsi_interface(); */

	/* speed selection (HS or LPS) */
	val = in_regl(DSI_VC0_CTRL + (cmd_vc * 0x20));
	if (mode == DSI_HS_MODE_TX)
		val = val | (1 << 9);
	else if (mode == DSI_LPS_MODE_TX)
		val = val & ~(1 << 9);


	out_regl(DSI_VC0_CTRL + (cmd_vc * 0x20), val);

	/*TODO: can be do the below step before itself, do we need to disable
	 * the DSI interface before configuring the VCs */
	/* enable_omap_dsi_interface(); */
	enable_cmd_mode_vc();

	/* Send Short packet */
	header = (0 << 24)|
		(data1 << 16)|
		(data0 << 8)|
		(vc << 6)|
		(data_type << 0);

	out_regl(DSI_VC0_SHORT_PACKET_HEADER + (cmd_vc * 0x20), header);

#ifdef CMD_VC1
	if ((data0 == DCS_SOFT_RESET) ||
	      (data0 == DCS_EXIT_SLEEP_MODE)) {
		out_regl(DSI_VC0_LONG_PACKET_HEADER+(cmd_vc*0x20), 0x00000409);
		out_regl(DSI_VC0_LONG_PACKET_PAYLOAD+(cmd_vc*0x20), 0xFFFFFFFF);
	}
#endif

	/* Wait for the data to be send */
	do {
		val = in_regl(DSI_VC0_IRQSTATUS+(cmd_vc*0x20));
	} while ((!(val & 0x00000004)) && (--count));

	if (count) {
		/*TODO: this need to be cross check,
		 * whether we need to reset the bit */
		out_regl(DSI_VC0_IRQSTATUS + (cmd_vc * 0x20), 0x00000004);
		return OMAP_DAL_SUCCESS;
	} else {
		return OMAP_DAL_ERROR;
	}
}

/*-----------------------------------------------------------------------------
| Function    : enable_video_mode_vc()
+------------------------------------------------------------------------------
| Description : Function to enable the video mode virtual channel
|
| Parameters  : None
|
| Returns     : None
+----------------------------------------------------------------------------*/
void enable_video_mode_vc()
{
	U32 val;
	val = in_regl(DSI_VC0_CTRL + (video_vc * 0x20));
	val = val | (1 << 0);
	out_regl(DSI_VC0_CTRL + (video_vc * 0x20), val);
}

/*-----------------------------------------------------------------------------
| Function    : disable_video_mode_vc()
+------------------------------------------------------------------------------
| Description : Function to disable the video mode virtual channel
|
| Parameters  : None
|
| Returns     : None
+----------------------------------------------------------------------------*/
void disable_video_mode_vc()
{
	U32 val;
	val = in_regl(DSI_VC0_CTRL + (video_vc * 0x20));
	val = val & ~(1 << 0);
	out_regl(DSI_VC0_CTRL + (video_vc * 0x20), val);
}

/*-----------------------------------------------------------------------------
| Function    : enable_cmd_mode_vc()
+------------------------------------------------------------------------------
| Description : Function to enable the command mode virtual channel
|
| Parameters  : None
|
| Returns     : None
+----------------------------------------------------------------------------*/
void enable_cmd_mode_vc()
{
	U32 val;
	val = in_regl(DSI_VC0_CTRL + (cmd_vc * 0x20));
	val = val | (1 << 0);
	out_regl(DSI_VC0_CTRL + (cmd_vc * 0x20), val);
}

/*-----------------------------------------------------------------------------
| Function    : disable_cmd_mode_vc()
+------------------------------------------------------------------------------
| Description : Function to disable the command mode virtual channel
|
| Parameters  : None
|
| Returns     : None
+----------------------------------------------------------------------------*/
void disable_cmd_mode_vc()
{
	U32 val;
	val = in_regl(DSI_VC0_CTRL + (cmd_vc * 0x20));
	val = val & ~(1 << 0);
	out_regl(DSI_VC0_CTRL + (cmd_vc * 0x20), val);
}


/*-----------------------------------------------------------------------------
| Function    : enable_dispc_output()
+------------------------------------------------------------------------------
| Description : Function to enable the DISPC LCD output interface
|
| Parameters  : None
|
| Returns     : None
+----------------------------------------------------------------------------*/
void enable_dispc_output()
{
	U32 val;
	val = disp_reg_in(DISPC_CONTROL);
	val = val | ((1 << 0) | (1 << 5));
	disp_reg_out(DISPC_CONTROL, val);
}

/*-----------------------------------------------------------------------------
| Function    : disable_dispc_output()
+------------------------------------------------------------------------------
| Description : Function to disable the DISPC LCD output interface
|
| Parameters  : None
|
| Returns     : None
+----------------------------------------------------------------------------*/
void disable_dispc_output()
{
	U32 val;
	val = disp_reg_in(DISPC_CONTROL);
	val = val & ~((1 << 0) | (1 << 5));
	disp_reg_out(DISPC_CONTROL, val);
}

/*-----------------------------------------------------------------------------
| Function    : get_sysclk()
+------------------------------------------------------------------------------
| Description : Function return the SYSCLK value
|
| Parameters  : None
|
| Returns     : SYSCLK value
+----------------------------------------------------------------------------*/
U16 get_sysclk()
{
	U32 val;

	val = in_regl(PRM_CLKSEL);

	switch (val & 0x00000007) {
	case 0:
		return 12;

	case 1:
		return 13;

	case 2:
		return 19.2;

	case 3:
		return 26;

	case 4:
		return 38.4;

	case 5:
		return 16.8;
	}
	return 0;
}


U8 check_tx_fifo(U8 channel)
{
	U32 val;
	val = in_regl(DSI_TX_FIFO_VC_EMPTINESS);
	val = (val & (0xFF << (channel*8)) >> (channel*8));
	return val;
}

/*-----------------------------------------------------------------------------
| Function    : get_ddr_clk_val(U16 pixel_clk, U8 bits_per_pixel,
|		U8 num_data_lanes)
+------------------------------------------------------------------------------
| Description : Function return the DDR CLK value
|
| Parameters  : pixel_clk - LCD interface pixel clock
|		bits_per_pixel - bits representing one pixel
|		num_data_lanes - number of data lanes used by DSI device
| Returns     : DDR CLK value
+----------------------------------------------------------------------------*/
U16 get_ddr_clk_val(U16 pixel_clk, U8 bits_per_pixel, U8 num_data_lanes)
{
#ifndef SILVAL_CONFIG
	U16 data_rate, data_rate_on_each_lane, ddr_clk;

	/* Calculation of data rates  */
	data_rate = pixel_clk * bits_per_pixel;
	data_rate_on_each_lane = data_rate/num_data_lanes;
	ddr_clk = data_rate_on_each_lane / 2;

	return 100;
#else
	return 150;
#endif
}


void RGB_to_BGR_work_around()
{
	U32 val;

	/* Enable Color Phase Rotation */
	val = disp_reg_in(DISPC_CONFIG);
	val = 	val | (0x1 << 15);
	disp_reg_out(DISPC_CONFIG, val);

	/* Fill the CPR matrix for RGB to BGR */
	val = disp_reg_in(DISPC_CPR_R);
	val = 	val | (256 << 0);
	disp_reg_out(DISPC_CPR_R, val);

	val = disp_reg_in(DISPC_CPR_G);
	val = 	val | (256 << 11);
	disp_reg_out(DISPC_CPR_G, val);

	val = disp_reg_in(DISPC_CPR_B);
	val = 	val | (256 << 22);
	disp_reg_out(DISPC_CPR_B, val);
}
