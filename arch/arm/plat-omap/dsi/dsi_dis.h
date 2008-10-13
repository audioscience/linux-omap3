/*
 * arch/arm/plat-omap/dsi_dis.h
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */


#ifndef OMAP_DSI_DIS_H
#define OMAP_DSI_DIS_H

/*==== INCLUDES =============================================================*/
#include "types.h"

#define DSI_SID		1

#define SEND_DCS_READ_CMD	10
#define SEND_DCS_WRITE_CMD	11
#define DSI_DISPLAY_OUT	12

#define ENABLE_VIDEO_TX	1
#define DISABLE_VIDEO_TX	2



/* Lane position in DSI output interface */
#define LANE_POS_1		1
#define LANE_POS_2		2
#define LANE_POS_3		3

#define LANE_POL_XP_YN		0	/* DSI_DX=+ and DSI_DY=- */
#define LANE_POL_XN_YP		1	/* DSI_DX=- and DSI_DY=+ */


#define ECC_ENABLE		1
#define ECC_DISABLE		2
#define CHECKSUM_ENABLE	3
#define CHECKSUM_DISABLE	4

#define DSI_VIDEO_MODE		1
#define DSI_COMMAND_MODE	2


#define GRAPHICS_PIPELINE 	1
#define VIDEO1_PIPELINE   	2
#define VIDEO2_PIPELINE   	3

#define ACTIVE_LOW	 	1
#define ACTIVE_HIGH	 	2

#define FALLING_EDGE		1
#define RISING_EDGE		2

#define DSI_HS_MODE_TX	1
#define DSI_LPS_MODE_TX	2


/* DSI module base addresses */

#define DSI_PROTO_ENG_BASE_ADDR	0x4804FC00
#define DSI_COMPLEXIO_BASE_ADDR	0x4804FE00
#define DSI_PLL_CONTROLLER_BASE_ADDR	0x4804FF00

/* DSI module registers */
#define DSI_REVISION		(DSI_PROTO_ENG_BASE_ADDR + 0x00)
#define DSI_SYSCONFIG		(DSI_PROTO_ENG_BASE_ADDR + 0x10)
#define DSI_SYSSTATUS		(DSI_PROTO_ENG_BASE_ADDR + 0x14)
#define DSI_IRQSTATUS		(DSI_PROTO_ENG_BASE_ADDR + 0x18)
#define DSI_IRQENABLE		(DSI_PROTO_ENG_BASE_ADDR + 0x1C)
#define DSI_CTRL		(DSI_PROTO_ENG_BASE_ADDR + 0x40)
#define DSI_COMPLEXIO_CFG1	(DSI_PROTO_ENG_BASE_ADDR + 0x48)
#define DSI_COMPLEXIO_IRQSTATUS	(DSI_PROTO_ENG_BASE_ADDR + 0x4C)
#define DSI_COMPLEXIO_IRQENABLE	(DSI_PROTO_ENG_BASE_ADDR + 0x50)
#define DSI_CLK_CTRL		(DSI_PROTO_ENG_BASE_ADDR + 0x54)
#define DSI_TIMING1		(DSI_PROTO_ENG_BASE_ADDR + 0x58)
#define DSI_TIMING2		(DSI_PROTO_ENG_BASE_ADDR + 0x5C)
#define DSI_VM_TIMING1		(DSI_PROTO_ENG_BASE_ADDR + 0x60)
#define DSI_VM_TIMING2		(DSI_PROTO_ENG_BASE_ADDR + 0x64)
#define DSI_VM_TIMING3		(DSI_PROTO_ENG_BASE_ADDR + 0x68)
#define DSI_CLK_TIMING		(DSI_PROTO_ENG_BASE_ADDR + 0x6C)
#define DSI_TX_FIFO_VC_SIZE	(DSI_PROTO_ENG_BASE_ADDR + 0x70)
#define DSI_RX_FIFO_VC_SIZE	(DSI_PROTO_ENG_BASE_ADDR + 0x74)
#define DSI_COMPLEXIO_CFG2	(DSI_PROTO_ENG_BASE_ADDR + 0x78)
#define DSI_RX_FIFO_VC_FULLNESS	(DSI_PROTO_ENG_BASE_ADDR + 0x7C)
#define DSI_VM_TIMING4		(DSI_PROTO_ENG_BASE_ADDR + 0x80)
#define DSI_TX_FIFO_VC_EMPTINESS	(DSI_PROTO_ENG_BASE_ADDR + 0x84)
#define DSI_VM_TIMING5		(DSI_PROTO_ENG_BASE_ADDR + 0x88)
#define DSI_VM_TIMING6		(DSI_PROTO_ENG_BASE_ADDR + 0x8C)
#define DSI_VM_TIMING7		(DSI_PROTO_ENG_BASE_ADDR + 0x90)
#define DSI_VC0_CTRL		(DSI_PROTO_ENG_BASE_ADDR + 0x100)
#define DSI_VC0_TE		(DSI_PROTO_ENG_BASE_ADDR + 0x104)
#define DSI_VC0_LONG_PACKET_HEADER	(DSI_PROTO_ENG_BASE_ADDR + 0x108)
#define DSI_VC0_LONG_PACKET_PAYLOAD	(DSI_PROTO_ENG_BASE_ADDR + 0x10C)
#define DSI_VC0_SHORT_PACKET_HEADER	(DSI_PROTO_ENG_BASE_ADDR + 0x110)
#define DSI_VC0_IRQSTATUS		(DSI_PROTO_ENG_BASE_ADDR + 0x118)
#define DSI_VC0_IRQENABLE		(DSI_PROTO_ENG_BASE_ADDR + 0x11C)

/* DSI ComplexIO registers */
#define	DSIPHY_CFG0		(DSI_COMPLEXIO_BASE_ADDR + 0x00)
#define	DSIPHY_CFG1		(DSI_COMPLEXIO_BASE_ADDR + 0x04)
#define	DSIPHY_CFG2		(DSI_COMPLEXIO_BASE_ADDR + 0x08)
#define	DSIPHY_CFG5		(DSI_COMPLEXIO_BASE_ADDR + 0x14)

/* DSI PLL registers */
#define DSI_PLL_CONTROL		(DSI_PLL_CONTROLLER_BASE_ADDR + 0x00)
#define DSI_PLL_STATUS		(DSI_PLL_CONTROLLER_BASE_ADDR + 0x04)
#define DSI_PLL_GO		(DSI_PLL_CONTROLLER_BASE_ADDR + 0x08)
#define DSI_PLL_CONFIGURATION1	(DSI_PLL_CONTROLLER_BASE_ADDR + 0x0C)
#define DSI_PLL_CONFIGURATION2	(DSI_PLL_CONTROLLER_BASE_ADDR + 0x10)

typedef struct {

	U32 bit_rate;
	U16 line_time;
	U16 time_hbp;
	U16 time_hact;
	U16 hact;
	U16 time_hfp;
	U16 time_hsa;
	U16 vsa;
	U16 vbp;
	U16 vact;
	U16 vfp;
} T_VM_TIMING;

typedef struct {

	U16 hact;
	U16 vact;
	U16 hsa;
	U16 hbp;
	U16 hfp;
	U16 vsa;
	U16 vbp;
	U16 vfp;
	U16 display_size;
	U16 pixel_clk;
	U16 hsync_pol;
	U16 vsync_pol;
	U16 dataen_pol;
	U16 clk_pol;
	U16 interface_width;
	U16 pixel_format;
} T_DPI_OUT_CONFIG;



typedef struct {

	U16 pid;
	U16 sid;
	U16 image_width;
	U16 image_height;
	U16 pixel_format;
	U16 data_rate;
	U16 num_data_lanes;
	U8 virtual_channel_id;
	U8 data_lane_pol;
	U8 clk_lane_pol;
	BOOLEAN ecc_flag;
	BOOLEAN checksum_flag;
	U16 max_read_resp_size;
	U8 display_mode;
	U8 dispc_pipeline;
	BOOLEAN yuv_to_rgb_conv;
	T_VM_TIMING video_mode_timing;
	T_DPI_OUT_CONFIG output_intf_config;
	U32 frame_buffer;
} T_DSI_INIT;


typedef struct {

	U32 len;
	U8 *buf;
} T_IO_BUF;


typedef struct {

	U8 type;
	U8 mode;
	T_IO_BUF req_buf;
	T_IO_BUF resp_buf;
} T_REQ_RESP_BUF;


#endif /* OMAP_DSI_DIS_H */
