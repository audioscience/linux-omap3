/*
 * arch/arm/plat-omap/brd_dsi.h
 *
 * Header file for the OMAP DSI low level driver file (dsi.c)
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

/*==== DECLARATION CONTROL ==================================================*/
#ifndef OMAP_DSI_H
#define OMAP_DSI_H

/*============================== INCLUDES ================================*/
#include "types.h"
#include "dsi_drv.h"
#include "dsi_dis.h"

/*=============================== MACROS =================================*/
#define PRM_CLKSEL		(0x48306D40)

/* DPHY power control */
#define DPHY_PWR_CMD_OFF	0
#define DPHY_PWR_CMD_ON		1
#define	DPHY_PWR_CMD_ULPS	2

/* DSI PLL power control */
#define PWR_CMD_STATE_OFF		0
#define PWR_CMD_STATE_ON_HSCLK		1
#define PWR_CMD_STATE_ON_ALL		2
#define PWR_CMD_STATE_ON_DIV		3
#define RESET_COMPLETED		1
#define RESET_NOT_COMPLETED	0
#define HSDIV_ENABLE		1
#define HSDIV_DISABLE		2

/* PLL programming modes */
#define MANUAL_MODE		1
#define AUTOMATIC_MODE		2

/* DSI PLL Input clock options */
#define PCLK			1
#define SYSCLK			2
#define ENABLE_DSI_OUTPUT	1
#define DISABLE_DSI_OUTPUT	2
/* Virtual channel control */
#define ENABLE_VC		1
#define DISABLE_VC		2
/* Timer control */
#define TIMER_ENABLE		1
#define TIMER_DISABLE		2
/* Data type field for DCS commands */
#define DCS_WRITE_LONG_CMD		0x39
#define DCS_WRITE_SHORT_CMD		0x15
#define DCS_WRITE_SHORT_NO_PARAM_CMD	0x05
#define DCS_READ_CMD			0x06
/* DCS command type */
#define WRITE_CMD			1
#define READ_CMD			2
/* RX ECC and CHECKSUM options */
#define RX_CHECKSUM_CHECK_ENABLE	1
#define RX_CHECKSUM_CHECK_DISABLE	2
#define RX_ECC_CHECK_ENABLE		1
#define RX_ECC_CHECK_DISABLE		2
#define OMAP_DAL_SUCCESS	0
#define OMAP_DAL_ERROR		0xF205000C
#define  YUV422BE      0x0
/*#define  YUV422	0x1*/
#define  YUV420	0x2
#define  RGB444	0x4
#define  RGB565	0x5
#define  RGB888_NDE	0x6
#define  RGB888_DE	0x7
#define  RAW8_NDE	0x8
#define  RAW8_DE	0x9
#define  RAW10_NDE	0xA
#define  RAW10_DE	0xB
#define  RAW12_NDE	0xC
#define  RAW12_DE	0xD
#define  JPEG8_FSP	0xE
#define  JPEG8		0xF
#define  RGB666	0x10
#define  RGB888	0x11
#define VGA		0x3
#define QVGA		0xa

/*========================= FUNCTION PROTOTYPES ==========================*/

void dsi_complexio_lane_config(U8 clk_pos, U8 clk_pol, U8 data1_pos,\
				U8 data1_pol, U8 data2_pos, U8 data2_pol);
void dsi_complexio_timing_config(T_DPHY_CONFIG complexio_timing);
S32 dsi_complexio_pwr_cmd(U8 cmd);
U8 dsi_complexio_reset_status(void);
S32 config_dphy(T_DSI_DIS *dsi_dis);
S32 config_dsipll(U16 pixel_clk, U8 pixel_format, U8 num_data_lanes,\
		U8 pclk_or_sysclk, U16 clkin, U8 hsdivider, U8 config_mode);
S32 dsi_pll_pwr_cmd(U8 cmd);
U8 dsi_pll_reset_status(void);
S32 lock_dsi_pll(U8 mode, U8 pclk_or_sysclk, U16 clkin, U16 regn, U16 regm,\
			U16 regm3, U16 regm4);
U8 reset_dsi_proto_eng(void);
void dsi_video_mode(BOOLEAN flag, U8 vc, U16 pixel_per_line, U8 pixel_format);
S32 config_dsi_interface(T_DSI_DIS *dsi_dis);
void disable_dsi_interface(void);
S32 config_dsi_proto_engine(T_DSI_DIS *dsi_dis);
void switch_to_dsipll_clk_source(void);
void switch_to_alwon_fclk_source(void);
S32 send_dsi_trigger(void);
void config_video_mode_timing(T_VM_TIMING vm_timing);
void config_dsi_timers(void);
void config_video_port(U8 vs_pol, U8 de_pol, U8 hs_pol, U8 clk_pol,\
				U8 vp_bus_width, BOOLEAN checksum, BOOLEAN ecc);
void config_video_mode_channel(U8 channel_id, U8 ecc, U8 checksum);
void config_command_mode_channel(U8 channel_id);
void enable_omap_dsi_interface(void);
void disable_omap_dsi_interface(void);
void config_dsi_fifo(U16 tx_fifo_len, U16 rx_fifo_le);
S32 config_disp_controller(T_DSI_DIS *dsi_dis);
void config_dispc_timing(T_DPI_OUT_CONFIG lcd_config);
void config_signal_pol(T_DPI_OUT_CONFIG lcd_config);
void config_lcd_size(T_DPI_OUT_CONFIG lcd_config);
void config_pixel_clock(U32 dss_func_clk, U32 pixel_clk);
void config_lcd_color(U32 background_color, U32 transparent_color);
void config_pipeline(U8 pipeline, T_DPI_OUT_CONFIG lcd_config,\
			U32 frame_buffer);
void config_graphics_pipeline(T_DPI_OUT_CONFIG lcd_config, U32 frame_buffer);
void config_video1_pipeline(T_DPI_OUT_CONFIG lcd_config, U32 frame_buffer);
void config_video2_pipeline(T_DPI_OUT_CONFIG lcd_config, U32 frame_buffer);
S32 config_image_type_conversion(U8 in_image_type, U8 out_image_type,\
					U8 pipeline);
void config_dispc_out_interface(U8 pixel_format);
S32 send_dsi_packet(U8 data_type, U8 vc, U8 operation, U16 num_bytes,\
			U8 *in_buf, BOOLEAN mode, BOOLEAN ecc, BOOLEAN\
				checksum, U8 *out_buf, U32 len);
S32 send_long_packet(U8 data_type, U8 vc, U16 word_count, U8 *buf,\
			BOOLEAN mode, BOOLEAN ecc, BOOLEAN checksum);
S32 send_short_packet(U8 data_type, U8 vc, U8 data0, U8 data1,\
				BOOLEAN mode, BOOLEAN ecc);
void enable_video_mode_vc(void);
void disable_video_mode_vc(void);
void enable_cmd_mode_vc(void);
void disable_cmd_mode_vc(void);
void enable_dispc_output(void);
void disable_dispc_output(void);
U16 get_sysclk(void);
U16 get_ddr_clk_val(U16 pixel_clk, U8 bits_per_pixel, U8 num_data_lanes);

U8 check_tx_fifo(U8 channel);
void   RGB_to_BGR_work_around(void);

/* DISPC timings */
extern U8 dispc_hfp;
extern U8 dispc_hsa;
extern U8 dispc_hbp;
extern U8 dispc_vfp;

/* DSI Video Mode timings */
extern U8 vid_hfp;
extern U8 vid_hsa;
extern U8 vid_hbp;
extern U8 vid_vfp;
extern U8 vid_vsa;
extern U8 vid_vbp;
extern U8 vid_hs;
extern U8 vid_lp_clk_div;

extern U16 vid_wc;
extern U16 vid_tl;
extern U16 vid_vact;
extern U16 enter_hs_mode_latency;
extern U16 exit_hs_mode_latency;
extern U16 gddr_clk_pre;
extern U16 gddr_clk_post;

#endif /*  OMAP_DSI_H */

