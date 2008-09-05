/*
 * arch/arm/mach-omap2/board-omap3evm-dc.h
 *
 * Copyright (C) 2008 Texas Instruments Inc
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __BOARD_OMAP3EVM_DC_H_
#define __BOARD_OMAP3EVM_DC_H_

/* mux id to enable/disable signal routing to different peripherals */
enum omap3evmdc_mux {
	MUX_TVP5146 = 0,
	MUX_AIC23_MCBSP2,
	MUX_SPDIF_MCBSP2,
	MUX_HDMI_MCBSP2,
	MUX_BT_MCBSP3,
	MUX_JAC_MCSPI2,
	MUX_BT_UART1,
	MUX_MLC_NAND,		/* not connected */
	MUX_JAC_MCBSP1,
	MUX_IMAGE_SENSOR,
	MUX_JAC_MCBSP3,
	MUX_SPDIF_MCSPI2,
	NUM_MUX
};

/* enum to enable or disable mux */
enum config_mux {
	DISABLE_MUX,
	ENABLE_MUX
};

int omap3evmdc_set_mux(enum omap3evmdc_mux mux_id, enum config_mux value);

#endif				/* End of __BOARD_OMAP3EVM_DC_H_ */
