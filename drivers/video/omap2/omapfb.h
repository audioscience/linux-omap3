/*
 * linux/drivers/video/omap2/omapfb.h
 *
 * Copyright (C) 2008 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * Some code and ideas taken from drivers/video/omap/ driver
 * by Imre Deak.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __DRIVERS_VIDEO_OMAP2_OMAPFB_H__
#define __DRIVERS_VIDEO_OMAP2_OMAPFB_H__

#ifdef CONFIG_FB_OMAP2_DEBUG
#define DEBUG
#endif

#ifdef DEBUG
extern unsigned int omapfb_debug;
#define DBG(format, ...) \
	if (omapfb_debug) \
		printk(KERN_DEBUG "OMAPFB: " format, ## __VA_ARGS__)
#else
#define DBG(format, ...)
#endif

#define FB2OFB(fb_info) ((struct omapfb_info *)(fb_info->par))

/* max number of overlays to which a framebuffer data can be direct */
#define OMAPFB_MAX_OVL_PER_FB 3

struct omapfb2_mem_region {
	u32		_paddr;
	void __iomem	*_vaddr;
	struct vrfb	vrfb;
	unsigned long	size;
	u8		type;		/* OMAPFB_PLANE_MEM_* */
	unsigned	alloc:1;	/* allocated by the driver */
	unsigned	map:1;		/* kernel mapped by the driver */
};

enum omapfb_rotation_type {
	OMAPFB_ROT_DMA = 0,
	OMAPFB_ROT_VRFB = 1,
};

/* appended to fb_info */
struct omapfb_info {
	int id;
	struct omapfb2_mem_region region;
	atomic_t map_count;
	int num_overlays;
	struct omap_overlay *overlays[OMAPFB_MAX_OVL_PER_FB];
	struct omapfb2_device *fbdev;
	enum omapfb_rotation_type rotation_type;
	int rotation;
	wait_queue_head_t vsync_wait;
	unsigned long vsync_cnt;
};

struct omapfb2_device {
	struct device *dev;
	struct mutex  mtx;

	u32 pseudo_palette[17];

	int state;

	int num_fbs;
	struct fb_info *fbs[10];

	int num_displays;
	struct omap_display *displays[10];
	int num_overlays;
	struct omap_overlay *overlays[10];
	int num_managers;
	struct omap_overlay_manager *managers[10];
	/*
	 * Frame-buffer PM suppor
	 */
	u32 sleep_timeout;
	struct timer_list timer;
};

u32 omapfb_get_region_paddr(struct omapfb_info *ofbi);
void *omapfb_get_region_vaddr(struct omapfb_info *ofbi);

void set_fb_fix(struct fb_info *fbi);
int check_fb_var(struct fb_info *fbi, struct fb_var_screeninfo *var);
int omapfb_realloc_fbmem(struct omapfb2_device *fbdev, int fbnum,
		unsigned long size);
int omapfb_apply_changes(struct fb_info *fbi, int init);
int omapfb_setup_overlay(struct fb_info *fbi, struct omap_overlay *ovl,
		int posx, int posy, int outw, int outh);

void omapfb_create_sysfs(struct omapfb2_device *fbdev);
void omapfb_remove_sysfs(struct omapfb2_device *fbdev);

int omapfb_ioctl(struct fb_info *fbi, unsigned int cmd, unsigned long arg);

int omapfb_mode_to_timings(const char *mode_str,
		struct omap_video_timings *timings, unsigned *bpp);

/* find the display connected to this fb, if any */
static inline struct omap_display *fb2display(struct fb_info *fbi)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	int i;

	/* XXX: returns the display connected to first attached overlay */
	for (i = 0; i < ofbi->num_overlays; i++) {
		if (ofbi->overlays[i]->manager)
			return ofbi->overlays[i]->manager->display;
	}

	return NULL;
}

static inline void omapfb_lock(struct omapfb2_device *fbdev)
{
	mutex_lock(&fbdev->mtx);
}

static inline void omapfb_unlock(struct omapfb2_device *fbdev)
{
	mutex_unlock(&fbdev->mtx);
}

#endif
