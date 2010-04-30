/*
 *
 * Framebuffer driver header file for TI 816x
 *
 * Copyright (C) 2009 TI
 * Author: Yihe Hu <yihehu@ti.com>
 *
 * Some code and ideas was from TI OMAP2 Driver by Tomi Valkeinen.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA	02111-1307, USA.
 */

#ifndef __LINUX_TI816XFB_H__
#define __LINUX_TI816XFB_H__

#include <linux/fb.h>
#include <linux/ioctl.h>
#include <linux/types.h>


#define TI816XFB_CLUT_ENTRY 256
#define TI816XFB_CLUT_UNIT 4
#define TI816XFB_CLUT_SIZE (TI816XFB_CLUT_ENTRY * TI816XFB_CLUT_UNIT)
#define TI816XFB_CLUT_MASK 0xFF


/*customer ioctl definitions*/
#define TI816XFB_IOW(num, dtype)  _IOW('N', num, dtype)
#define TI816XFB_IOR(num, dtype)  _IOR('N', num, dtype)
#define TI816XFB_IOWR(num, dtype) _IOWR('N', num, dtype)
#define TI816XFB_IO(num)          _IO('N', num)

#define TI816XFB_SET_PARAMS   TI816XFB_IOW(2, struct ti816xfb_region_params)
#define TI816XFB_GET_PARAMS   TI816XFB_IOR(3, struct ti816xfb_region_params)
#define TI816XFB_SET_COEFF    TI816XFB_IOW(4, struct ti816xfb_region_scparams)
#define TI816XFB_GET_COEFF    TI816XFB_IOW(5, struct ti816xfb_region_scparams)
#define TI816XFB_ALLOC_STENC  TI816XFB_IOW(6, int)
#define TI816XFB_FREE_STENC   TI816XFB_IOR(7, int)
#define TI816XFB_SETUP_MEM    TI816XFB_IOW(8, struct ti816xfb_mem_info)
#define TI816XFB_QUERY_MEM    TI816XFB_IOR(9, struct ti816xfb_mem_info)
#define TI816XFB_SET_STENC TI816XFB_IOW(10, struct ti816xfb_stenciling_params)


#define TI816XFB_FREE 0xFF
#define TI816XFB_INVALID_OFFSET 0xFFFFFFFF

#define TI816XFB_MAX_PRIORITY 0xF

enum ti816xfb_status {
	TI816XFB_FEATURE_DISABLE = 0,
	TI816XFB_FEATURE_ENABLE
};

enum ti816xfb_rotate_type {
	TI816XFB_ROTATE_NO = 0,
	TI816XFB_ROTATE_180_MIRROR,
	TI816XFB_ROTATE_MIRRORONLY,
	TI816XFB_ROTATE_180,
	TI816XFB_ROTATE_270_MIRROR,
	TI816XFB_ROTATE_270,
	TI816XFB_ROTATE_90,
	TI816XFB_ROTATE_90_MIRROR,
	TI816XFB_ROTATE_MAX
};

enum ti816xfb_mirroring {
	TI816XFB_MIRRORING_OFF = 0,
	TI816XFB_MIRRORING_ON
};

/**
 * ti816xfb_pix_format
 *	 Define the GRPX data format
 */
enum ti816xfb_data_format {
	TI816XFB_RGB565 = 0x1000,
	TI816XFB_ARGB1555,
	TI816XFB_RGBA5551,
	TI816XFB_ARGB4444,
	TI816XFB_RGBA4444,
	TI816XFB_ARGB6666,
	TI816XFB_RGBA6666,
	TI816XFB_RGB888,
	TI816XFB_ARGB8888,
	TI816XFB_RGBA8888,
	TI816XFB_BMP8 = 0x2000,
	TI816XFB_BMP4_L,
	TI816XFB_BMP4_U,
	TI816XFB_BMP2_OFF0,
	TI816XFB_BMP2_OFF1,
	TI816XFB_BMP2_OFF2,
	TI816XFB_BMP2_OFF3,
	TI816XFB_BMP1_OFF0,
	TI816XFB_BMP1_OFF1,
	TI816XFB_BMP1_OFF2,
	TI816XFB_BMP1_OFF3,
	TI816XFB_BMP1_OFF4,
	TI816XFB_BMP1_OFF5,
	TI816XFB_BMP1_OFF6,
	TI816XFB_BMP1_OFF7
};

enum ti816xfb_mem_mode {
	TI816XFB_MEM_NONTILER = 0,
	TI816XFB_MEM_TILER_PAGE ,
	TI816XFB_MEM_TILER_8,
	TI816XFB_MEM_TILER_16,
	TI816XFB_MEM_TILER_32
};

enum ti816xfb_blending_type {
	TI816XFB_BLENDING_NO = 0,
	TI816XFB_BLENDING_GLOBAL,
	TI816XFB_BLENDING_PALETTE,
	TI816XFB_BLENDING_PIXEL

};

enum ti816xfb_transparancy_type {
	TI816XFB_TRANSP_LSPMASK_NO = 0,
	TI816XFB_TRANSP_LSPMASK_1,
	TI816XFB_TRANSP_LSPMASK_2,
	TI816XFB_TRANSP_LSMMASK_3
};

struct ti816xfb_mem_info {
	__u32  size;
	/*enum ti816xfb_mem_mode*/
	__u32  type;
};

struct ti816xfb_region_params {
	__u16               ridx;
	__u16               pos_x;
	__u16               pos_y;
	__u16               priority;
	/*enum ti816xfb_status*/
	__u32               firstregion;
	/*enum ti816xfb_status*/
	__u32               lastregion;
	/*enum ti816xfb_status*/
	__u32               scalaren;
	/*enum ti816xfb_status*/
	__u32               stencilingen;
	/*enum ti816xfb_status*/
	__u32               bben;
	/*enum ti816xfb_status*/
	__u32               transen;
	/*enum ti816xfb_blending_type*/
	__u32               blendtype;
	/*enum ti816xfb_transparancy_type*/
	__u32               transtype;
	__u32               transcolor;
	__u8                bbalpha;
	__u8                blendalpha;
	__u8                reserved[2];
};


struct ti816xfb_region_scparams {
	__u16               inwidth;
	__u16               inheight;
	__u16               outwidth;
	__u16               outheight;
};

struct ti816xfb_stenciling_params {
	__u32               pitch;
	__u32               phy_addr;
};

/**
 * ti816xfb_regions_list
 *	 Defines the regions list which contain the information
 *	 for each regions in the given frame
 */
struct ti816xfb_regions_list {
	__u32                           num_reg;
	struct ti816xfb_region_params   *regions;
};

#ifdef __KERNEL__

#include <linux/fvid2.h>


#define TI816XFB_MEMTYPE_SDRAM		0u
#define TI816XFB_MEMTYPE_MAX		0u
#define TI816X_FB_NUM (3)


/**
 *	 ti816X_mem_region
 *	  Define the one frame buffer memroy information
 *
 */

struct  ti816xfb_mem_region {
	u32             paddr;
	void __iomem    *vaddr;
	unsigned long   size;
	bool            alloc; /*allocated by the driver*/
	bool            map; /*kernel mapped by the driver*/
};


/**
 *   ti816xfb_mem_desc
 *		define the memory regions for all fb
 */
struct ti816xfb_mem_desc {
	int                         region_cnt;
	struct ti816xfb_mem_region  mreg[TI816X_FB_NUM];
};

struct ti816xfb_platform_data {
	struct ti816xfb_mem_desc   mem_desc;
	void                       *ctrl_platform_data;
};

#define FVID2_Q_TIMEOUT 0xFFFFFFFF

struct ti816xfb_datamode {
	enum fvid2_dataformat           dataformat;
	u32                             bpp;
	u32                             nonstd;
	struct fb_bitfield              red;
	struct fb_bitfield              green;
	struct fb_bitfield              blue;
	struct fb_bitfield              transp;
};

extern void ti816xfb_set_platform_data(struct ti816xfb_platform_data *data);
extern void ti816xfb_reserve_sdram(void);


#endif

#endif
