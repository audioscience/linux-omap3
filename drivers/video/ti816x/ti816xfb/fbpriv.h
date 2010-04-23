/*
 *
 * Framebuffer private header file for TI 816x
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

#ifndef __DRIVERS_VIDEO_TI816X_TI816XFB_FBPRIV_H__
#define __DRIVERS_VIDEO_TI816X_TI816XFB_FBPRIV_H__

#ifdef CONFIG_FB_TI816X_DEBUG_SUPPORT
#define DEBUG
#endif

#include <linux/vps.h>
#include <linux/vps_displayctrl.h>
#include <linux/vps_graphics.h>
#include <linux/dc.h>
#include <linux/grpx.h>

#ifdef DEBUG
extern unsigned int fb_debug;
#define TFBDBG(format, ...) \
        if (fb_debug) \
		printk(KERN_INFO"TI816XFB: " format, ## __VA_ARGS__)
#else
#define TFBDBG(format, ...)
#endif

#define FB2TFB(fb_info) ((struct ti816xfb_info *)(fb_info->par))

#define TI816XFB_BPP	32

struct ti816xfb_alloc_list {
	struct list_head 		list;
	dma_addr_t 			phy_addr;
	void 				*cpu_addr;
	u32 				size;
};


/**
 *	ti816xfb_info
 *	  Define one ti816xfb windows information
 *
 */
struct ti816xfb_info {
	int 				idx;
	int	 			enable;
	struct mutex	   		rqueue_mutex;
	struct ti816xfb_device 		*fbdev;
	struct list_head 		alloc_list;
	struct ti816xfb_mem_region 	mreg;
	struct vps_grpx_ctrl 		*gctrl;
	dma_addr_t 			pclut;
	void 				*vclut;
	enum ti816xfb_data_format  	pixfmt;
	u32 				pseudo_palette[16];
	enum ti816xfb_mem_mode  		mmode;
	wait_queue_head_t 		vsync_wait;
	unsigned long 			vsync_cnt;
};

/**
 * ti816xfb_device
 *	 Define the ti816x fb device structure
 *
 */
struct ti816xfb_device {
	struct device			*dev;
	int 				num_fbs;
	struct fb_info			*fbs[TI816X_FB_NUM];
	int 				num_grpx;
	struct vps_grpx_ctrl 		*gctrl[TI816X_FB_NUM];
};

int ti816xfb_fbinfo_init(struct ti816xfb_device *fbdev, struct fb_info *fbi);

int ti816xfb_realloc_fbmem(struct fb_info *fbi, unsigned long size);
int ti816xfb_create_sysfs(struct ti816xfb_device *fbdev);
void ti816xfb_remove_sysfs(struct ti816xfb_device *fbdev);
int ti816xfb_create_dccfg(struct fb_info *fbi);
int ti816xfb_ioctl(struct fb_info *fbi, unsigned int cmd, unsigned long arg);


static inline void ti816xfb_lock(struct ti816xfb_info *tfbi)
{
	mutex_lock(&tfbi->rqueue_mutex);
}

static inline void ti816xfb_unlock(struct ti816xfb_info *tfbi)
{
	mutex_unlock(&tfbi->rqueue_mutex);
}

#endif

