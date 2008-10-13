/*
 * arch/arm/plat-omap/brd_dsi.c
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied
 *
 * This file contains platform dependent low level functions for OMAP DSI
 * interface on reference board
 *
 */


/*=======INCLUDES======================================================*/

#include "dsi.h"
#include "brd_dsi.h"
#include <linux/module.h>
#include <linux/delay.h>
/* #define NB_DATALANES_1 */
/* #define VGA_IMAGE */
/* #define RGB18_IMAGE */

/*==== DECLARATION CONTROL =================================================*/
/* static S32 enable_dsi_power(void); */
/* static S32 disable_dsi_power(void); */
/* static void dsi_pad_config(void); */
void update_timing_parameters(U32 ddr_clk, \
				U32 num_data_lanes, U32 bits_per_pixel);

/* DISPC timings */
U8 dispc_hfp;
U8 dispc_hsa;
U8 dispc_hbp;
U8 dispc_vfp;

/* DSI Video Mode timings */
U8 vid_hfp;
U8 vid_hsa;
U8 vid_hbp;
U8 vid_vfp;
U8 vid_vsa;
U8 vid_vbp;
U8 vid_hs;
U8 vid_lp_clk_div;
U8 window_sync;

U16 vid_wc;
U16 vid_tl;
U16 vid_vact;
U16 enter_hs_mode_latency;
U16 exit_hs_mode_latency;

/* D-PHY timings */
U16 ttag0;
U16 ttasure;
U16 ttaget;
U16 tlpxdiv2;
U16 tclktrail;
U16 tclkzero;
U16 thsp;
U16 thspz;
U16 thse;
U16 tclkp;
U16 tlpx;
U16 tclk_pre;
U16 tclk_post;
U16 gddr_clk_pre;
U16 gddr_clk_post;
U16 thst;

#define DIVROUNDUP(A, B) (((A) % (B) == 0) ? ((A) / (B)) : (((A) / (B)) + 1))

#define MARGE 3
#define FOR_EDISCO_ES1 32


/*============ FUNCTIONS ===================================================*/

/*----------------------------------------------------------------------------
| Function    : S32 dsi_platform_init(T_DSI_DIS *dsi_conf)
+-----------------------------------------------------------------------------
| Description : This function does the platform specific initialization
|               requiered on the SDP platform
|
| Parameters  : dsi_conf - pointer to the DSI interface information structure
|
| Returns     : Status (CSST_DAL_SUCCESS or Error)
+----------------------------------------------------------------------------*/
S32 dsi_platform_init(T_DSI_DIS *dsi_conf)
{
	S32 ret = OMAP_DAL_SUCCESS;

	return ret;
}

/*-----------------------------------------------------------------------------
| Function    : S32 dsi_platform_deinit(T_DSI_DIS *dsi_conf)
+------------------------------------------------------------------------------
| Description : This function does the platform specific deinit required
|               on the SDP platform
|
| Parameters  : dsi_conf - pointer to the DSI interface information structure
|
| Returns     : Status (OMAP_DAL_SUCCESS or Error)
+----------------------------------------------------------------------------*/
S32 dsi_platform_deinit(T_DSI_DIS *dsi_conf)
{
	S32 ret = OMAP_DAL_SUCCESS;

	return ret;
}

/*-------------------------------------------------------------------
| Function    : void get_dphy_config(T_DPHY_CONFIG *complexio_config,
|		U8 num_data_lanes, U16 pixel_clk, U16 pixel_format)
+--------------------------------------------------------------------
| Description : This function return the DPHY
|		configuration required on SDP3430 platform
|
| Parameters  : complexio_config - pointer to the ComplexIO config structure
|		num_data_lanes	 - number of data lanes
|		pixel_clk	 - pixel clock frequency
|		pixel_format	 - pixel format
| Returns     : None
+-------------------------------------------------------------------*/
void get_dphy_config(T_DPHY_CONFIG *complexio_config, U8 num_data_lanes, \
			U16 pixel_clk, U16 pixel_format)
{
	U16 data_rate, ddr_clk;
	F32 ddr_clk_period;
	U8 bits_per_pixel = 0;

	/* Number of bits in each format, can be a macro */
	if (pixel_format == RGB565)
		bits_per_pixel = 16;
	else if (pixel_format == RGB666)
		bits_per_pixel = 18;
	else if (pixel_format == RGB888)
		bits_per_pixel = 24;

	/* Calculate DDR_CLK frequency */
	ddr_clk = get_ddr_clk_val(pixel_clk, \
				bits_per_pixel, \
				num_data_lanes);

	update_timing_parameters(ddr_clk, num_data_lanes, bits_per_pixel);
	/* Findout the DDR_CLK period */
	ddr_clk_period = (1000/ddr_clk);
	data_rate = pixel_clk * bits_per_pixel;

	complexio_config->data_rate = data_rate;

	/* Configure the Lane position and Polariy */
	/*TODO: cross check the lane position values */
	complexio_config->clk_lane.pos = LANE_POS_1;
	complexio_config->clk_lane.pol = LANE_POL_XP_YN;
	complexio_config->data_lane[0].pos = LANE_POS_2;
	complexio_config->data_lane[0].pol = LANE_POL_XP_YN;

	if (num_data_lanes == 2) {
		complexio_config->data_lane[1].pos = LANE_POS_3;
		complexio_config->data_lane[1].pol = LANE_POL_XP_YN;
	} else if (num_data_lanes == 1) {
		/* in this case, LANE2 configuration is initialized to zero */
		complexio_config->data_lane[1].pos = 0;
		complexio_config->data_lane[1].pol = 0;
	}

	/* Fill DSI protocol timing parameters */
	if (ddr_clk != 150) {

		complexio_config->ths_prepare = 8;
		complexio_config->ths_prepare_ths_zero = 25;
		complexio_config->ths_trail = 35;
		complexio_config->ths_exit = 11;
		complexio_config->tlpx_half = 3;
		complexio_config->tclk_trail = 7;
		complexio_config->tclk_zero = 23;
		complexio_config->tclk_prepare = 7;
		tclk_pre = 9;
		tclk_post = 59;
		gddr_clk_pre = (int)(tclk_pre + complexio_config->tlpx_half*2\
				+ complexio_config->tclk_zero\
				+ complexio_config->tclk_prepare)/4;
		gddr_clk_post = (int)(tclk_post +\
					complexio_config->tclk_trail)/4;
		enter_hs_mode_latency = 1\
			+ DIVROUNDUP(2 * complexio_config->tlpx_half, 4)\
			+ DIVROUNDUP(complexio_config->ths_prepare , 4)\
			+ DIVROUNDUP(complexio_config->ths_prepare_ths_zero\
			- complexio_config->ths_prepare + 3, 4);
		exit_hs_mode_latency = ((complexio_config->ths_trail\
			+ complexio_config->ths_exit)/4) + 1;
	} else {
		complexio_config->ths_prepare = 11;
		complexio_config->ths_prepare_ths_zero = 27;
		complexio_config->ths_trail = 38;
		complexio_config->ths_exit = 16;
		complexio_config->tlpx_half = 4;
		complexio_config->tclk_trail = 10;
		complexio_config->tclk_zero = 35;
		complexio_config->tclk_prepare = 10;
		vid_lp_clk_div = 8;
		}
}

/* Following settings taken from SilVal team */

void update_timing_parameters(U32 ddr_clk, U32 num_data_lanes,\
				U32 bits_per_pixel)
{
#ifndef CONFIG_FB_OMAP_LCD_VGA
		dispc_hfp = 20;
		dispc_hsa = 10;
		dispc_hbp = 20;
#else
		dispc_hfp = 20;
		dispc_hsa = 20;
		dispc_hbp = 20;
#endif

#ifndef CONFIG_FB_OMAP_LCD_VGA
#ifdef NB_DATALANES_1
	vid_hfp = 38;
	vid_hsa = 0;
	vid_hbp = 52;
	vid_tl = 580;
#else /* NB_DATALANES_1 */

#ifdef RGB18_IMAGE
	vid_hfp = 22;
	vid_hsa = 0;
	vid_hbp = 59;
	vid_tl = 326;
#else /* RGB18_IMAGE */
	vid_hfp = 19;
	vid_hsa = 0;
	vid_hbp = 26;
	vid_tl = 290;
#endif /* RGB18_IMAGE */

#endif /* NB_DATALANES_1 */

#else /* CONFIG_FB_OMAP_LCD_VGA */

#ifdef NB_DATALANES_1
	vid_hfp = 38;
	vid_hsa = 0;
	vid_hbp = 52;
	vid_tl = 1060;
#else /* NB_DATALANES_1 */
	vid_hfp = 19;
	vid_hsa = 0;
	/* vid_hbp = 26; */
	vid_hbp = 36;
	vid_tl = 540;
	/* vid_tl = 530; */
#endif /* NB_DATALANES_1 */


#endif /* CONFIG_FB_OMAP_LCD_VGA */

#ifdef CONFIG_FB_OMAP_LCD_VGA
	vid_vfp = 2;
	vid_vsa = 2;
	vid_vbp = 2;
	/* hnagalla -- HW recommended fix for GOLCD issue */
	dispc_vfp = vid_vfp - 2;
#else
	vid_vfp = 1;
	vid_vsa = 1;
	vid_vbp = 1;
#endif

	vid_hs = 4/num_data_lanes;
	vid_lp_clk_div = 8;
	window_sync = 4;

	vid_wc = 480 * 2;
	vid_vact = 640;

}
