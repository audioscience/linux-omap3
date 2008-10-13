/*
 * arch/arm/plat-omap/dsi_drv.h
 *
 * Header file for the OMAP DSI low level driver file (dsi_drv.c)
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#ifndef OMAP_DSI_DRV_H
#define OMAP_DSI_DRV_H


/* #include "csst_tgt.h" */
#include "types.h"
#include "dsi_dis.h"


typedef struct {

	U8 pos;
	U8 pol;
} T_LANE_CONFIG;

typedef struct {

	U16 ths_prepare;
	U16 ths_prepare_ths_zero;
	U16 ths_trail;
	U16 ths_exit;
	U16 tlpx_half;
	U16 tclk_trail;
	U16 tclk_zero;
	U16 tclk_prepare;
	U16 data_rate;
	T_LANE_CONFIG clk_lane;	/* Clock lane config */
	T_LANE_CONFIG data_lane[2];	/* Data lane config */

} T_DPHY_CONFIG;


typedef struct {

	T_DSI_INIT init;
	T_DPHY_CONFIG dphy_config;
} T_DSI_DIS;

S32 dsi_init(const void *init_str, U32 *device_handle);
S32 dsi_read(U32 device_handle, U32 read_tag, U32 *buf_len, U8 *buf);
S32 dsi_write(U32 device_handle, U32 write_tag, U32 *len, U8 *buf);
S32 dsi_deinit(U32 device_handle);

#endif /* OMAP_DSI_DRV_H */

