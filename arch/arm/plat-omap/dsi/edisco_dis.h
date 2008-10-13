/*
 * arch/arm/plat-omap/edisco_dis.h
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#ifndef OMAP_EDISCO_DIS_H
#define OMAP_EDISCO_DIS_H


#include "types.h"

#define EDISCO_SID	1

#define READ_DEVICE_ID			0x10
#define EDISCO_DCS_READ_CMD		0x11
#define DISPLAY_OUT				0x12
#define COLORBAR_DISPLAY		0x13
#define EDISCO_DCS_WRITE_CMD	0x14

#define DISPLAY_OUT_ENABLE	1
#define DISPLAY_OUT_DISABLE	2

typedef struct {

	U16 pid;
	U16 sid;
	U16 image_width;
	U16 image_height;
	U16 display_size;
	U16 pixel_format;
	BOOLEAN	yuv_to_rgb_conv;
	U32 frame_buffer;
} T_EDISCO_INIT;

#endif /* OMAP_EDISCO_DIS_H  */
