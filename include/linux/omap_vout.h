/*
 * drivers/media/video/omap/omap_vout.h
 *
 * Copyright (C) 2005 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef OMAP_VOUT_H
#define OMAP_VOUT_H

#include <linux/types.h>

/* This is for user apps */
#define OMAP_OUTPUT_LCD        4
#define OMAP_OUTPUT_TV         5
#define OMAP_GFX_DESTINATION   100
#define OMAP_VIDEO_SOURCE      101

struct omap_vout_colorkey {
	__u32 output_dev;
	__u32 key_type;
	__u32 key_val;
};

struct omap_vout_bgcolor {
	__u32 color;
	__u32 output_dev;
};

struct omap_vout_colconv {
	__s16 RY,RCr,RCb;
	__s16 GY,GCr,GCb;
	__s16 BY,BCr,BCb;
};

/* non-standard V4L2 ioctls that are specific to OMAP */
#define VIDIOC_S_OMAP_MIRROR		_IOW ('V', 1,  __s32)
#define VIDIOC_G_OMAP_MIRROR		_IOR ('V', 2,  __s32)
#define VIDIOC_S_OMAP_ROTATION		_IOW ('V', 3,  __s32)
#define VIDIOC_G_OMAP_ROTATION		_IOR ('V', 4,  __s32)
#define VIDIOC_S_OMAP_LINK		_IOW ('V', 5,  __s32)
#define VIDIOC_G_OMAP_LINK		_IOR ('V', 6,  __s32)
#define VIDIOC_S_OMAP_COLORKEY		_IOW ('V', 7,  struct omap_vout_colorkey)
#define VIDIOC_G_OMAP_COLORKEY		_IOW ('V', 8,  struct omap_vout_colorkey)
#define VIDIOC_S_OMAP_BGCOLOR		_IOW ('V', 9,  struct omap_vout_bgcolor)
#define VIDIOC_G_OMAP_BGCOLOR		_IOW ('V', 10, struct omap_vout_bgcolor)
#define VIDIOC_OMAP_COLORKEY_ENABLE	_IOW ('V', 11, __s32)
#define VIDIOC_OMAP_COLORKEY_DISABLE	_IOW ('V', 12, __s32)
#define VIDIOC_S_OMAP_DEFCOLORCONV	_IOW ('V', 13, __s32)
#define VIDIOC_S_OMAP_COLORCONV	_IOW ('V', 14, struct omap_vout_colconv)
#define VIDIOC_G_OMAP_COLORCONV	_IOR ('V', 15, struct omap_vout_colconv)

#endif	/* ifndef OMAP_VOUT_H */
