/*
 * arch/arm/plat-omap/brd_dsi.h
 *
 * Header file for the DSI interface platform dependent low level
 * functions (brd_dsi.c)
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

/*==== DECLARATION CONTROL =================================================*/
#ifndef OMAP_BRD_DSI_H
#define OMAP_BRD_DSI_H

/*==== DEFINES ============================================================*/

/* Prototype Functions */
S32 dsi_platform_init(T_DSI_DIS *dsi_conf);
S32 dsi_platform_deinit(T_DSI_DIS *dsi_conf);
void get_dphy_config(T_DPHY_CONFIG *complexio_config, U8 num_data_lanes,\
			U16 pixel_clk, U16 pixel_format);
#endif /* OMAP_BRD_DSI_H */

