/*
 * linux/drivers/video/omap2/omapfb-main.c
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <mach/display.h>
#include <mach/omapfb.h>

#include "omapfb.h"

#define MODULE_NAME     "omapfb"

static char *def_mode;
static char *def_vram;
static int def_rotate_type = -1;
static int def_rotate = -1;

#define VRFB_WIDTH 	(2048)
#define FB_SIZE 	(1280 * 720 * 4 * 2)

#define IS_VALID_ROTATION(rotate) ((rotate == FB_ROTATE_UR) || \
				(rotate == FB_ROTATE_CW) || \
				(rotate == FB_ROTATE_UD) || \
				(rotate == FB_ROTATE_CCW))

#define IS_ROTATION_ENABLED(rotation_type) ((rotation_type == OMAPFB_ROT_DMA) \
		|| (rotation_type == OMAPFB_ROT_VRFB))

#ifdef DEBUG
unsigned int omapfb_debug;
module_param_named(debug, omapfb_debug, bool, 0644);
#endif

#ifdef DEBUG
static void fill_fb(struct fb_info *fbi)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct fb_var_screeninfo *var = &fbi->var;
	struct fb_fix_screeninfo *fix = &fbi->fix;

	const short w = var->xres_virtual;
	const short h = var->yres_virtual;
	const int bytespp = var->bits_per_pixel >> 3;
	const int row_inc = fix->line_length / bytespp - w;

	void *addr = omapfb_get_region_vaddr(ofbi);
	u8 *p = addr;
	int y, x;

	DBG("fill_fb %dx%d, line_len %d\n", w, h, fix->line_length);

	for (y = 0; y < h; y++) {
		for (x = 0; x < w; x++) {
			if (var->bits_per_pixel == 16) {
				u16 *pw = (u16 *)p;

				if (x < 20 && y < 20)
					*pw = 0xffff;
				else if (x == 20 || x == w - 20 ||
						y == 20 || y == h - 20)
					*pw = 0xffff;
				else if (x == y || w - x == h - y)
					*pw = ((1<<5)-1)<<11;
				else if (w - x == y || x == h - y)
					*pw = ((1<<6)-1)<<5;
				else {
					int t = x / (w/3);
					if (t == 0)
						*pw = y % 32;
					else if (t == 1)
						*pw = (y % 64) << 5;
					else if (t == 2)
						*pw = (y % 32) << 11;
				}
			} else if (var->bits_per_pixel == 24) {
				u8 *pb = (u8 *)p;

				int r = 0, g = 0, b = 0;

				if (x < 20 && y < 20)
					r = g = b = 0xff;
				else if (x == 20 || x == w - 20 ||
						y == 20 || y == h - 20)
					r = g = b = 0xff;
				else if (x == y || w - x == h - y)
					r = 0xff;
				else if (w - x == y || x == h - y)
					g = 0xff;
				else {
					int q = x / (w / 3);
					u8 base = 255 - (y % 256);
					if (q == 0)
						r = base;
					else if (q == 1)
						g = base;
					else if (q == 2)
						b = base;
				}

				pb[0] = b;
				pb[1] = g;
				pb[2] = r;

			} else if (var->bits_per_pixel == 32) {
				u32 *pd = (u32 *)p;

				if (x < 20 && y < 20)
					*pd = 0xffffff;
				else if (x == 20 || x == w - 20 ||
						y == 20 || y == h - 20)
					*pd = 0xffffff;
				else if (x == y || w - x == h - y)
					*pd = 0xff0000;
				else if (w - x == y || x == h - y)
					*pd = 0x00ff00;
				else {
					u8 base = 255 - (y % 256);
					*pd = base << ((x / (w/3)) << 3);
				}
			}

			p += bytespp;
		}

		p += bytespp * row_inc;
	}

	DBG("fill done\n");
}
#endif

static unsigned omapfb_get_vrfb_offset(struct omapfb_info *ofbi, int rot)
{
	struct vrfb *vrfb = &ofbi->region.vrfb;
	unsigned offset;

	switch (rot) {
	case FB_ROTATE_UR:
		offset = 0;
		break;
	case FB_ROTATE_CW:
		offset = vrfb->yoffset * vrfb->bytespp;
		break;
	case FB_ROTATE_UD:
		offset = (vrfb->yoffset * VRFB_WIDTH + vrfb->xoffset) *
			vrfb->bytespp;
		break;
	case FB_ROTATE_CCW:
		offset = vrfb->xoffset * vrfb->bytespp;
		break;
	default:
		BUG();
	}

	return offset;
}

u32 omapfb_get_region_paddr(struct omapfb_info *ofbi)
{
	if (ofbi->rotation_type == OMAPFB_ROT_VRFB) {
		int rot;
		unsigned offset;

		if (ofbi->rotation == FB_ROTATE_CW)
			rot = FB_ROTATE_CCW;
		else if (ofbi->rotation == FB_ROTATE_CCW)
			rot = FB_ROTATE_CW;
		else
			rot = ofbi->rotation;

		offset = omapfb_get_vrfb_offset(ofbi, rot);

		return ofbi->region.vrfb.paddr[rot] + offset;
	} else
		return ofbi->region._paddr;
}

void *omapfb_get_region_vaddr(struct omapfb_info *ofbi)
{
	if (ofbi->rotation_type == OMAPFB_ROT_VRFB) {
		int rot;
		unsigned offset;

		if (ofbi->rotation == FB_ROTATE_CW)
			rot = FB_ROTATE_CCW;
		else if (ofbi->rotation == FB_ROTATE_CCW)
			rot = FB_ROTATE_CW;
		else
			rot = ofbi->rotation;

		offset = omapfb_get_vrfb_offset(ofbi, rot);

		return ofbi->region.vrfb.vaddr[rot] + offset;
	} else
		return ofbi->region._vaddr;
}

static enum omap_color_mode fb_mode_to_dss_mode(struct fb_var_screeninfo *var)
{
	switch (var->nonstd) {
	case 0:
		break;
	case OMAPFB_COLOR_YUV422:
		return OMAP_DSS_COLOR_UYVY;

	case OMAPFB_COLOR_YUY422:
		return OMAP_DSS_COLOR_YUV2;

	case OMAPFB_COLOR_ARGB16:
		return OMAP_DSS_COLOR_ARGB16;

	case OMAPFB_COLOR_ARGB32:
		return OMAP_DSS_COLOR_ARGB32;

	case OMAPFB_COLOR_RGBA32:
		return OMAP_DSS_COLOR_RGBA32;

	case OMAPFB_COLOR_RGBX32:
		return OMAP_DSS_COLOR_RGBX32;

	default:
		return -EINVAL;
	}

	switch (var->bits_per_pixel) {
	case 1:
		return OMAP_DSS_COLOR_CLUT1;
	case 2:
		return OMAP_DSS_COLOR_CLUT2;
	case 4:
		return OMAP_DSS_COLOR_CLUT4;
	case 8:
		return OMAP_DSS_COLOR_CLUT8;
	case 12:
		return OMAP_DSS_COLOR_RGB12U;
	case 16:
		return OMAP_DSS_COLOR_RGB16;
	case 24:
		return OMAP_DSS_COLOR_RGB24P;
	case 32:
	{
		if (var->transp.length == 8) {
			if (var->transp.offset == 0)
				return OMAP_DSS_COLOR_RGBA32;
			else
				return OMAP_DSS_COLOR_ARGB32;
		} else
			return OMAP_DSS_COLOR_RGB24U;
		break;
	}
	default:
		return -EINVAL;
	}

	return -EINVAL;
}

void set_fb_fix(struct fb_info *fbi)
{
	struct fb_fix_screeninfo *fix = &fbi->fix;
	struct fb_var_screeninfo *var = &fbi->var;
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omapfb2_mem_region *rg = &ofbi->region;

	DBG("set_fb_fix\n");

	/* used by open/write in fbmem.c */
	fbi->screen_base = (char __iomem *)omapfb_get_region_vaddr(ofbi);

	/* used by mmap in fbmem.c */
	if (ofbi->rotation_type == OMAPFB_ROT_VRFB)
		fix->line_length = (VRFB_WIDTH * var->bits_per_pixel) >> 3;
	else
		fix->line_length = (var->xres_virtual * var->bits_per_pixel) >> 3;
	fix->smem_start = omapfb_get_region_paddr(ofbi);
	/*
	 * No need to calculate here, since we are handling size part
	 * properly in the function "omapfb_alloc_fbmem_display"
	 */
	fix->smem_len = rg->size;
	fix->type = FB_TYPE_PACKED_PIXELS;

	if (var->nonstd)
		fix->visual = FB_VISUAL_PSEUDOCOLOR;
	else {
		switch (var->bits_per_pixel) {
		case 32:
		case 24:
		case 16:
		case 12:
			fix->visual = FB_VISUAL_TRUECOLOR;
			/* 12bpp is stored in 16 bits */
			break;
		case 1:
		case 2:
		case 4:
		case 8:
			fix->visual = FB_VISUAL_PSEUDOCOLOR;
			break;
		}
	}

	fix->accel = FB_ACCEL_NONE;

	fix->xpanstep = 1;
	fix->ypanstep = 1;

	if (rg->size) {
		int w, h;
		if (ofbi->rotation == FB_ROTATE_CW ||
				ofbi->rotation == FB_ROTATE_CCW) {
			w = var->yres_virtual;
			h = var->xres_virtual;
		} else {
			w = var->xres_virtual;
			h = var->yres_virtual;
		}
		omap_vrfb_setup(rg->vrfb.context, rg->_paddr,
				w, h,
				var->bits_per_pixel / 8);
	}
}

/* check new var and possibly modify it to be ok */
int check_fb_var(struct fb_info *fbi, struct fb_var_screeninfo *var)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omap_display *display = fb2display(fbi);
	unsigned long max_frame_size;
	unsigned long line_size;
	int xres_min;
	int yres_min;
	enum omap_color_mode mode = 0;
	struct omap_overlay *ovl;

	DBG("check_fb_var %d\n", ofbi->id);

	if (ofbi->region.size == 0) {
		memset(var, 0, sizeof(*var));
		return 0;
	}

	if (ofbi->num_overlays == 0) {
		dev_err(ofbi->fbdev->dev, "no overlays, aborting\n");
		return -EINVAL;
	}

	/* XXX: uses the first overlay */
	ovl = ofbi->overlays[0];

	/* if we are using non standard mode, fix the bpp first */
	switch (var->nonstd) {
	case 0:
		break;
	case OMAPFB_COLOR_YUV422:
	case OMAPFB_COLOR_YUY422:
	case OMAPFB_COLOR_ARGB16:
		var->bits_per_pixel = 16;
		break;
	case OMAPFB_COLOR_ARGB32:
	case OMAPFB_COLOR_RGBA32:
	case OMAPFB_COLOR_RGBX32:
		var->bits_per_pixel = 32;
		break;
	default:
		DBG("invalid nonstd mode\n");
		return -EINVAL;
	}

	mode = fb_mode_to_dss_mode(var);
	if (mode < 0) {
		DBG("cannot convert var to omap dss mode\n");
		return -EINVAL;
	}

	if ((ovl->supported_modes & mode) == 0) {
		DBG("invalid mode\n");
		return -EINVAL;
	}
	if (var->rotate != fbi->var.rotate) {
		DBG("rotation changing\n");
		if (!IS_VALID_ROTATION(var->rotate)) {
			DBG("invalid rotation parameter\n");
			return -EINVAL;
		}
		if (!IS_ROTATION_ENABLED(ofbi->rotation_type))
			var->rotate = ofbi->rotation = -1;
		else
			/*TODO: If this function returns error after this
			 *	then we are setting wrong rotation value
			 *	to "ofbi"
			 */
			ofbi->rotation = var->rotate;
	}
	xres_min = OMAPFB_PLANE_XRES_MIN;
	yres_min = OMAPFB_PLANE_YRES_MIN;

	if (var->xres < xres_min)
		var->xres = xres_min;
	if (var->yres < yres_min)
		var->yres = yres_min;
	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;

	max_frame_size = ofbi->region.size;

	if ((ofbi->rotation_type == OMAPFB_ROT_VRFB) &&
			((ofbi->rotation == FB_ROTATE_CW) ||
			 (ofbi->rotation == FB_ROTATE_CCW))) {
		line_size = (VRFB_WIDTH * var->bits_per_pixel) >> 3;
		if (line_size * var->yres_virtual > max_frame_size) {
			/*Reduce yres_virtual, since we can't change xres */
			var->yres_virtual = max_frame_size / line_size;

			/* Recheck this, as the virtual size changed. */
			if (var->xres_virtual < var->xres)
				var->xres = var->xres_virtual;
			if (var->yres_virtual < var->yres)
				var->yres = var->yres_virtual;
			if (var->xres < xres_min || var->yres < yres_min) {
				DBG("Cannot fit FB to memory\n");
				return -EINVAL;
			}
		}

	} else {
		line_size = (var->xres_virtual * var->bits_per_pixel) >> 3;
		if (line_size * var->yres_virtual > max_frame_size) {
			/* Try to keep yres_virtual first */
			line_size = max_frame_size / var->yres_virtual;
			var->xres_virtual = line_size * 8 / var->bits_per_pixel;
			if (var->xres_virtual < var->xres) {
				/* Still doesn't fit. Shrink yres_virtual too */
				var->xres_virtual = var->xres;
				line_size = var->xres * var->bits_per_pixel / 8;
				var->yres_virtual = max_frame_size / line_size;
			}
			/* Recheck this, as the virtual size changed. */
			if (var->xres_virtual < var->xres)
				var->xres = var->xres_virtual;
			if (var->yres_virtual < var->yres)
				var->yres = var->yres_virtual;
			if (var->xres < xres_min || var->yres < yres_min) {
				DBG("Cannot fit FB to memory\n");
				return -EINVAL;
			}
		}
	}
	if (var->xres + var->xoffset > var->xres_virtual)
		var->xoffset = var->xres_virtual - var->xres;
	if (var->yres + var->yoffset > var->yres_virtual)
		var->yoffset = var->yres_virtual - var->yres;

	if (var->bits_per_pixel == 16) {
		var->red.offset  = 11; var->red.length   = 5;
		var->red.msb_right   = 0;
		var->green.offset = 5;  var->green.length = 6;
		var->green.msb_right = 0;
		var->blue.offset = 0;  var->blue.length  = 5;
		var->blue.msb_right  = 0;
	} else if (var->bits_per_pixel == 24) {
		var->red.offset  = 16; var->red.length   = 8;
		var->red.msb_right   = 0;
		var->green.offset = 8;  var->green.length = 8;
		var->green.msb_right = 0;
		var->blue.offset = 0;  var->blue.length  = 8;
		var->blue.msb_right  = 0;
		var->transp.offset = 0; var->transp.length = 0;
	} else if (var->bits_per_pixel == 32) {
		if (var->transp.length == 0) {
			var->red.offset  = 16; var->red.length   = 8;
			var->red.msb_right   = 0;
			var->green.offset = 8;  var->green.length = 8;
			var->green.msb_right = 0;
			var->blue.offset = 0;  var->blue.length  = 8;
			var->blue.msb_right  = 0;
			var->transp.offset = 0; var->transp.length = 0;
		} else if (var->transp.length == 8) {
			if (var->transp.offset == 0) {
				var->red.offset  = 24; var->red.length   = 8;
				var->red.msb_right   = 0;
				var->green.offset = 16;  var->green.length = 8;
				var->green.msb_right = 0;
				var->blue.offset = 8;  var->blue.length  = 8;
				var->blue.msb_right  = 0;
				var->transp.offset = 0; var->transp.length = 8;
			} else {
				var->red.offset  = 16; var->red.length   = 8;
				var->red.msb_right   = 0;
				var->green.offset = 8;  var->green.length = 8;
				var->green.msb_right = 0;
				var->blue.offset = 0;  var->blue.length  = 8;
				var->blue.msb_right  = 0;
				var->transp.offset = 24; var->transp.length = 8;
			}
		}
	} else {
		DBG("failed to setup fb color mask\n");
		return -EINVAL;
	}

	/* Reserved field is used for setting the global alpha value
	 *  which lies between 0 (full transperent) - 255 (complete opaque)
	 */
	if (var->reserved[0] > 255)
		var->reserved[0] = 255;
	if (var->reserved < 0)
		var->reserved[0] = 0;

	DBG("xres = %d, yres = %d, vxres = %d, vyres = %d\n",
			var->xres, var->yres,
			var->xres_virtual, var->yres_virtual);

	var->height             = -1;
	var->width              = -1;
	var->grayscale          = 0;

	if (display && display->get_timings) {
		struct omap_video_timings timings;
		display->get_timings(display, &timings);

		/* pixclock in ps, the rest in pixclock */
		var->pixclock = timings.pixel_clock != 0 ?
			KHZ2PICOS(timings.pixel_clock) :
			0;
		var->left_margin = timings.hfp;
		var->right_margin = timings.hbp;
		var->upper_margin = timings.vfp;
		var->lower_margin = timings.vbp;
		var->hsync_len = timings.hsw;
		var->vsync_len = timings.vsw;
	} else {
		var->pixclock = 0;
		var->left_margin = 0;
		var->right_margin = 0;
		var->upper_margin = 0;
		var->lower_margin = 0;
		var->hsync_len = 0;
		var->vsync_len = 0;
	}

	/* TODO: get these from panel->config */
	var->vmode              = FB_VMODE_NONINTERLACED;
	var->sync               = 0;

	return 0;
}

/*
 * ---------------------------------------------------------------------------
 * fbdev framework callbacks
 * ---------------------------------------------------------------------------
 */
static int omapfb_open(struct fb_info *fbi, int user)
{
	return 0;
}

static int omapfb_release(struct fb_info *fbi, int user)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omapfb2_device *fbdev = ofbi->fbdev;
	struct omap_display *display = fb2display(fbi);

	DBG("Closing fb with plane index %d\n", ofbi->id);

	omapfb_lock(fbdev);
#if 1
	if (display) {
		int w, h;
		/* XXX Is this really needed ? */
		if (display->sync)
			display->sync(display);

		display->get_resolution(display, &w, &h);

		if (display->update)
			display->update(display, 0, 0, w, h);
	}
#endif

	if (display && display->sync)
		display->sync(display);

	omapfb_unlock(fbdev);

	return 0;
}

/* setup overlay according to the fb */
int omapfb_setup_overlay(struct fb_info *fbi, struct omap_overlay *ovl,
		int posx, int posy, int outw, int outh)
{
	int r = 0;
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct fb_var_screeninfo *var = &fbi->var;
	struct fb_fix_screeninfo *fix = &fbi->fix;
	enum omap_color_mode mode = 0;
	int offset;
	u32 data_start_p;
	void *data_start_v;
	int xres, yres;
	int screen_width;

	DBG("setup_overlay %d\n", ofbi->id);

	if ((ovl->caps & OMAP_DSS_OVL_CAP_SCALE) == 0 &&
			(outw != var->xres || outh != var->yres)) {
		r = -EINVAL;
		goto err;
	}

	offset = ((var->yoffset * var->xres_virtual +
				var->xoffset) * var->bits_per_pixel) >> 3;

	if (ofbi->rotation_type == OMAPFB_ROT_VRFB) {
		/* setup DSS's view to always 0 degrees */
		data_start_p = ofbi->region.vrfb.paddr[0];
		data_start_v = ofbi->region.vrfb.vaddr[0];
		if (ofbi->rotation == FB_ROTATE_CW ||
				ofbi->rotation == FB_ROTATE_CCW) {
			offset = ((var->xoffset * VRFB_WIDTH +
				var->yoffset) * var->bits_per_pixel) >> 3;
		} else {
			offset = ((var->yoffset * VRFB_WIDTH +
				var->xoffset) * var->bits_per_pixel) >> 3;
		}
	} else {
		data_start_p = omapfb_get_region_paddr(ofbi);
		data_start_v = omapfb_get_region_vaddr(ofbi);
	}

	data_start_p += offset;
	data_start_v += offset;

	mode = fb_mode_to_dss_mode(var);

	if (mode == -EINVAL) {
		r = -EINVAL;
		goto err;
	}

	if (ofbi->rotation == FB_ROTATE_CW || ofbi->rotation == FB_ROTATE_CCW) {
		int tmp;
		xres = var->yres;
		yres = var->xres;
		tmp = outw;
		outw = outh;
		outh = tmp;
	} else {
		xres = var->xres;
		yres = var->yres;
	}
	screen_width = fix->line_length / (var->bits_per_pixel >> 3);

	r = ovl->setup_input(ovl,
			data_start_p, data_start_v,
			var->xres_virtual*var->bits_per_pixel/8,
			screen_width,
			xres, yres, mode,
			ofbi->rotation == -1 ? ofbi->rotation :ofbi->rotation*90,
			-1, var->reserved[0]);

	if (r)
		goto err;

	r = ovl->setup_output(ovl,
			posx, posy,
			outw, outh);

	if (r)
		goto err;

	return 0;

err:
	DBG("setup_overlay failed\n");
	return r;
}

/* apply var to the overlay */
int omapfb_apply_changes(struct fb_info *fbi, int init)
{
	int r = 0;
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct fb_var_screeninfo *var = &fbi->var;
	struct omap_overlay *ovl;
	int posx, posy;
	int outw, outh;
	int i;

	for (i = 0; i < ofbi->num_overlays; i++) {
		ovl = ofbi->overlays[i];

		DBG("apply_changes, fb %d, ovl %d\n", ofbi->id, ovl->id);

		if (ofbi->region.size == 0) {
			/* the fb is not available. disable the overlay */
			ovl->enable(ovl, 0);
			if (!init && ovl->manager)
				ovl->manager->apply(ovl->manager);
			continue;
		}

		if (init || (ovl->caps & OMAP_DSS_OVL_CAP_SCALE) == 0) {
			outw = var->xres;
			outh = var->yres;
		} else {
			outw = ovl->info.out_width;
			outh = ovl->info.out_height;
		}

		if (init) {
			posx = 0;
			posy = 0;
		} else {
			posx = ovl->info.pos_x;
			posy = ovl->info.pos_y;
		}
		r = omapfb_setup_overlay(fbi, ovl, posx, posy, outw, outh);
		if (r)
			goto err;

		if (!init && ovl->manager)
			ovl->manager->apply(ovl->manager);
	}
	return 0;
err:
	DBG("apply_changes failed\n");
	return r;
}

/* checks var and eventually tweaks it to something supported,
 * DO NOT MODIFY PAR */
static int omapfb_check_var(struct fb_var_screeninfo *var, struct fb_info *fbi)
{
	int r;

	DBG("check_var(%d)\n", FB2OFB(fbi)->id);

	r = check_fb_var(fbi, var);

	return r;
}

/* set the video mode according to info->var */
static int omapfb_set_par(struct fb_info *fbi)
{
	int r;

	DBG("set_par(%d)\n", FB2OFB(fbi)->id);

	set_fb_fix(fbi);
	r = omapfb_apply_changes(fbi, 0);

	return r;
}

static int omapfb_pan_display(struct fb_var_screeninfo *var,
		struct fb_info *fbi)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omapfb2_device *fbdev = ofbi->fbdev;
	int r = 0;

	DBG("pan_display(%d)\n", ofbi->id);

	omapfb_lock(fbdev);

	if (var->xoffset != fbi->var.xoffset ||
	    var->yoffset != fbi->var.yoffset) {
		struct fb_var_screeninfo new_var;

		new_var = fbi->var;
		new_var.xoffset = var->xoffset;
		new_var.yoffset = var->yoffset;

		r = check_fb_var(fbi, &new_var);

		if (r == 0) {
			fbi->var = new_var;
			set_fb_fix(fbi);
			r = omapfb_apply_changes(fbi, 0);
		}
	}

	omapfb_unlock(fbdev);

	return r;
}

static void mmap_user_open(struct vm_area_struct *vma)
{
	struct omapfb_info *ofbi = (struct omapfb_info *)vma->vm_private_data;

	atomic_inc(&ofbi->map_count);
}

static void mmap_user_close(struct vm_area_struct *vma)
{
	struct omapfb_info *ofbi = (struct omapfb_info *)vma->vm_private_data;

	atomic_dec(&ofbi->map_count);
}

static struct vm_operations_struct mmap_user_ops = {
	.open = mmap_user_open,
	.close = mmap_user_close,
};

static int omapfb_mmap(struct fb_info *fbi, struct vm_area_struct *vma)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct fb_fix_screeninfo *fix = &fbi->fix;
	unsigned long off;
	unsigned long start;
	u32 len;

	DBG("MMAP ROT %d\n", ofbi->rotation);

	if (vma->vm_end - vma->vm_start == 0)
		return 0;
	if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT))
		return -EINVAL;
	off = vma->vm_pgoff << PAGE_SHIFT;

	start = omapfb_get_region_paddr(ofbi);
		len = fix->smem_len;
	if (off >= len)
		return -EINVAL;
	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;

	off += start;
	DBG("mmap region start %lx, len %d, off %lx\n", start, len, off);

	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO | VM_RESERVED;
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vma->vm_ops = &mmap_user_ops;
	vma->vm_private_data = ofbi;
	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			     vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;
	/* vm_ops.open won't be called for mmap itself. */
	atomic_inc(&ofbi->map_count);
	return 0;
}

/* Store a single color palette entry into a pseudo palette or the hardware
 * palette if one is available. For now we support only 16bpp and thus store
 * the entry only to the pseudo palette.
 */
static int _setcolreg(struct fb_info *fbi, u_int regno, u_int red, u_int green,
		u_int blue, u_int transp, int update_hw_pal)
{
	/*struct omapfb_info *ofbi = FB2OFB(fbi);*/
	/*struct omapfb2_device *fbdev = ofbi->fbdev;*/
	struct fb_var_screeninfo *var = &fbi->var;
	int r = 0;

	enum omapfb_color_format mode = OMAPFB_COLOR_RGB24U; /* XXX */

	/*switch (plane->color_mode) {*/
	switch (mode) {
	case OMAPFB_COLOR_YUV422:
	case OMAPFB_COLOR_YUV420:
	case OMAPFB_COLOR_YUY422:
		r = -EINVAL;
		break;
	case OMAPFB_COLOR_CLUT_8BPP:
	case OMAPFB_COLOR_CLUT_4BPP:
	case OMAPFB_COLOR_CLUT_2BPP:
	case OMAPFB_COLOR_CLUT_1BPP:
		/*
		   if (fbdev->ctrl->setcolreg)
		   r = fbdev->ctrl->setcolreg(regno, red, green, blue,
		   transp, update_hw_pal);
		   */
		/* Fallthrough */
		r = -EINVAL;
		break;
	case OMAPFB_COLOR_RGB565:
	case OMAPFB_COLOR_RGB444:
	case OMAPFB_COLOR_RGB24P:
	case OMAPFB_COLOR_RGB24U:
		if (r != 0)
			break;

		if (regno < 0) {
			r = -EINVAL;
			break;
		}

		if (regno < 16) {
			u16 pal;
			pal = ((red >> (16 - var->red.length)) <<
					var->red.offset) |
				((green >> (16 - var->green.length)) <<
				 var->green.offset) |
				(blue >> (16 - var->blue.length));
			((u32 *)(fbi->pseudo_palette))[regno] = pal;
		}
		break;
	default:
		BUG();
	}
	return r;
}

static int omapfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
		u_int transp, struct fb_info *info)
{
	DBG("setcolreg\n");

	return _setcolreg(info, regno, red, green, blue, transp, 1);
}

static int omapfb_setcmap(struct fb_cmap *cmap, struct fb_info *info)
{
	int count, index, r;
	u16 *red, *green, *blue, *transp;
	u16 trans = 0xffff;

	DBG("setcmap\n");

	red     = cmap->red;
	green   = cmap->green;
	blue    = cmap->blue;
	transp  = cmap->transp;
	index   = cmap->start;

	for (count = 0; count < cmap->len; count++) {
		if (transp)
			trans = *transp++;
		r = _setcolreg(info, index++, *red++, *green++, *blue++, trans,
				count == cmap->len - 1);
		if (r != 0)
			return r;
	}

	return 0;
}

static int omapfb_blank(int blank, struct fb_info *fbi)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omapfb2_device *fbdev = ofbi->fbdev;
	struct omap_display *display = fb2display(fbi);
	int do_update = 0;
	int r = 0;

	omapfb_lock(fbdev);

	switch (blank) {
	case FB_BLANK_UNBLANK:
		if (display->state != OMAP_DSS_DISPLAY_SUSPENDED) {
			r = -EINVAL;
			goto exit;
		}

		if (display->resume)
			r = display->resume(display);

		if (r == 0 && display->get_update_mode &&
				display->get_update_mode(display) ==
				OMAP_DSS_UPDATE_MANUAL)
			do_update = 1;

		break;

	case FB_BLANK_NORMAL:
		/* FB_BLANK_NORMAL could be implemented.
		   * Needs DSS additions. */
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		if (display->state != OMAP_DSS_DISPLAY_ACTIVE) {
			r = -EINVAL;
			goto exit;
		}

		if (display->suspend)
			r = display->suspend(display);

		break;

	default:
		r = -EINVAL;
	}

exit:
	omapfb_unlock(fbdev);

	if (r == 0 && do_update && display->update) {
		int w, h;
		display->get_resolution(display, &w, &h);

		r = display->update(display, 0, 0, w, h);
	}

	return r;
}

ssize_t omapfb_write(struct fb_info *info, const char __user *buf,
		size_t count, loff_t *ppos)
{
	DBG("omapfb_write %d, %lu\n", count, (unsigned long)*ppos);
	// XXX needed for VRFB
	return count;
}

static struct fb_ops omapfb_ops = {
	.owner          = THIS_MODULE,
	.fb_open        = omapfb_open,
	.fb_release     = omapfb_release,
	.fb_fillrect    = cfb_fillrect,
	.fb_copyarea    = cfb_copyarea,
	.fb_imageblit   = cfb_imageblit,
	.fb_blank       = omapfb_blank,
	.fb_ioctl       = omapfb_ioctl,
	.fb_check_var   = omapfb_check_var,
	.fb_set_par     = omapfb_set_par,
	.fb_pan_display = omapfb_pan_display,
	.fb_mmap	= omapfb_mmap,
	.fb_setcolreg	= omapfb_setcolreg,
	.fb_setcmap	= omapfb_setcmap,
	//.fb_write     = omapfb_write,
};

static void omapfb_free_fbmem(struct omapfb2_device *fbdev, int fbnum)
{
	struct omapfb_info *ofbi = FB2OFB(fbdev->fbs[fbnum]);
	struct omapfb2_mem_region *rg;

	rg = &ofbi->region;

	if (rg->_paddr)
		if (omap_vram_free(rg->_paddr, rg->_vaddr, rg->size))
			dev_err(fbdev->dev, "VRAM FREE failed\n");

	omap_vrfb_release_ctx(&rg->vrfb);

	rg->_vaddr = NULL;
	rg->_paddr = 0;
	rg->alloc = 0;
	rg->size = 0;
}

static int omapfb_free_all_fbmem(struct omapfb2_device *fbdev)
{
	int i;

	DBG("free all fbmem\n");

	for (i = 0; i < fbdev->num_fbs; i++)
		omapfb_free_fbmem(fbdev, i);

	return 0;
}

static int omapfb_alloc_fbmem(struct omapfb2_device *fbdev, int fbnum,
		unsigned long size)
{
	struct omapfb_info *ofbi;
	struct omapfb2_mem_region *rg;
	unsigned long paddr;
	void *vaddr;
	int rot, r;

	size = PAGE_ALIGN(size);

	ofbi = FB2OFB(fbdev->fbs[fbnum]);
	rg = &ofbi->region;
	memset(rg, 0, sizeof(*rg));

	DBG("allocating %lu bytes for fb %d\n",
			size, ofbi->id);

	vaddr = omap_vram_alloc(OMAPFB_MEMTYPE_SDRAM, size, &paddr);
	DBG("allocated VRAM paddr %lx, vaddr %p\n", paddr, vaddr);

	if (vaddr == NULL) {
		dev_err(fbdev->dev,
				"failed to allocate framebuffer\n");
		return -ENOMEM;
	}

	r = omap_vrfb_create_ctx(&rg->vrfb);
	if (r) {
		dev_err(fbdev->dev, "vrfb create ctx failed\n");
		return r;
	}
	for (rot = 0; rot < 4; ++rot) {
		if (rg->vrfb.paddr[rot]) {
			rg->vrfb.vaddr[rot] = ioremap(rg->vrfb.paddr[rot],
					VRFB_SIZE);
			if (rg->vrfb.vaddr[rot] == NULL) {
				omap_vrfb_release_ctx(&rg->vrfb);
				dev_err(fbdev->dev, "failed to map VRFB\n");
				return -ENOMEM;
			}
			DBG("VRFB %d/%d: %lx -> %p\n", rg->vrfb.context, rot*90,
					rg->vrfb.paddr[rot],
					rg->vrfb.vaddr[rot]);
		}
	}
	rg->_paddr = paddr;
	rg->_vaddr = vaddr;
	rg->size = size;
	rg->alloc = 1;

	return 0;
}

int omapfb_realloc_fbmem(struct omapfb2_device *fbdev, int fbnum,
		unsigned long size)
{
	struct omapfb_info *ofbi = FB2OFB(fbdev->fbs[fbnum]);
	struct omapfb2_mem_region *rg = &ofbi->region;
	unsigned old_size = rg->size;
	int r;

	size = PAGE_ALIGN(size);

	omapfb_free_fbmem(fbdev, fbnum);

	if (size == 0)
		return 0;

	r = omapfb_alloc_fbmem(fbdev, fbnum, size);

	if (r)
		omapfb_alloc_fbmem(fbdev, fbnum, old_size);

	return r;
}

/* allocate fbmem using display resolution as reference */
static int omapfb_alloc_fbmem_display(struct omapfb2_device *fbdev, int fbnum,
		unsigned long def_vram)
{
	struct omap_display *display;
	unsigned long size;

	display =  fb2display(fbdev->fbs[fbnum]);
	if (!display)
		return 0;

	if (def_vram)
		size = def_vram;
	else if ((def_rotate_type == OMAPFB_ROT_VRFB) && (def_rotate >= 0))
		size = VRFB_SIZE;
	else
		size = FB_SIZE;

	return omapfb_alloc_fbmem(fbdev, fbnum, size);
}

static int omapfb_allocate_all_fbs(struct omapfb2_device *fbdev)
{
	int i, r;
	unsigned long vrams[10];

	memset(vrams, 0, sizeof(vrams));

	if (def_vram) {
		char str[64];
		char *tok, *s;

		if (strlen(def_vram) > sizeof(str) - 1) {
			dev_err(fbdev->dev, "Illegal vram parameters\n");
			return -EINVAL;
		}

		strcpy(str, def_vram);

		s = str;
		i = 0;

		while ((tok = strsep(&s, ","))) {
			unsigned long size;

			size = memparse(tok, NULL);

			if (size == 0) {
				dev_err(fbdev->dev, "illegal vram size\n");
				break;
			}

			vrams[i++] = size;
		}
	}

	for (i = 0; i < fbdev->num_fbs; i++) {
		/* allocate memory automatically only for fb0, or if
		 * excplicitly defined with vram option */
		if (i == 0 || vrams[i] != 0) {
			r = omapfb_alloc_fbmem_display(fbdev, i, vrams[i]);

			if (r)
				return r;
		}
	}

	for (i = 0; i < fbdev->num_fbs; i++) {
		struct omapfb_info *ofbi = FB2OFB(fbdev->fbs[i]);
		struct omapfb2_mem_region *rg;
		rg = &ofbi->region;

		DBG("region%d phys %08x virt %p size=%lu\n",
				i,
				rg->_paddr,
				rg->_vaddr,
				rg->size);
	}

	return 0;
}

/* initialize fb_info, var, fix to something sane based on the display */
static int fbinfo_init(struct omapfb2_device *fbdev, struct fb_info *fbi)
{
	struct fb_var_screeninfo *var = &fbi->var;
	struct fb_fix_screeninfo *fix = &fbi->fix;
	struct omap_display *display = fb2display(fbi);
	struct omapfb_info *ofbi = FB2OFB(fbi);
	int r = 0;

	fbi->fbops = &omapfb_ops;
	fbi->flags = FBINFO_FLAG_DEFAULT;
	fbi->pseudo_palette = fbdev->pseudo_palette;

	strncpy(fix->id, MODULE_NAME, sizeof(fix->id));

	var->nonstd = 0;

	DBG("default rotation %d\n", def_rotate);
	/* When VRFB rotation is enabled, the only possible
	 * values are 0, 1, 2, 3 for 0, 90, 180, 270 degrees.
	 * If the rotation is enabled without setting proper
	 * rotation angle then def_rotate will contain wrong value -1,
	 * may lead to some issues */
	if (IS_ROTATION_ENABLED(def_rotate_type)) {
		if (!IS_VALID_ROTATION(def_rotate)) {
			DBG("invalid rotation parameter\n");
			def_rotate = 0;
		}

	} else {
		def_rotate = -1;
	}

	ofbi->rotation_type = def_rotate_type;
	ofbi->rotation = def_rotate;
	var->rotate = ofbi->rotation;
	/* Set global alpha to 255 */
	var->reserved[0] = 255;

	if (display) {
		int w, h;
		display->get_resolution(display, &w, &h);
		if (ofbi->rotation == FB_ROTATE_CW ||
				ofbi->rotation == FB_ROTATE_CCW) {
			var->xres = h;
			var->yres = w;
		} else {
			var->xres = w;
			var->yres = h;
		}
		var->xres_virtual = var->xres;
		var->yres_virtual = var->yres;

		switch (display->panel->bpp) {
		case 16:
			var->bits_per_pixel = 16;
			break;
		case 18:
			var->bits_per_pixel = 16;
			break;
		case 24:
			var->bits_per_pixel = 32;
			break;
		default:
			dev_err(fbdev->dev, "illegal display bpp\n");
			return -EINVAL;
		}
	}

	r = check_fb_var(fbi, var);
	if (r)
		goto err;

	set_fb_fix(fbi);

	r = fb_alloc_cmap(&fbi->cmap, 256, 0);
	if (r != 0)
		dev_err(fbdev->dev, "unable to allocate color map memory\n");

#ifdef DEBUG
	if (omapfb_debug && FB2OFB(fbi)->region.size > 0)
		fill_fb(fbi);
#endif
err:
	return r;
}

static void fbinfo_cleanup(struct omapfb2_device *fbdev, struct fb_info *fbi)
{
	fb_dealloc_cmap(&fbi->cmap);
}

static void omapfb_free_resources(struct omapfb2_device *fbdev)
{
	int i;

	DBG("free_resources\n");

	if (fbdev == NULL)
		return;

	for (i = 0; i < fbdev->num_fbs; i++)
		unregister_framebuffer(fbdev->fbs[i]);

	/* free the reserved fbmem */
	omapfb_free_all_fbmem(fbdev);

	for (i = 0; i < fbdev->num_fbs; i++) {
		fbinfo_cleanup(fbdev, fbdev->fbs[i]);
		framebuffer_release(fbdev->fbs[i]);
	}

	for (i = 0; i < fbdev->num_displays; i++) {
		if (fbdev->displays[i]->state != OMAP_DSS_DISPLAY_DISABLED)
			fbdev->displays[i]->disable(fbdev->displays[i]);

		omap_dss_put_display(fbdev->displays[i]);
	}

	dev_set_drvdata(fbdev->dev, NULL);
	kfree(fbdev);
}

static ssize_t show_rotate_type(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct omapfb_info *ofbi = FB2OFB(fbi);

	return snprintf(buf, PAGE_SIZE, "%d\n", ofbi->rotation_type);
}

static ssize_t store_rotate_type(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	//struct fb_info *fbi = dev_get_drvdata(dev);
	//struct omapfb_info *ofbi = FB2OFB(fbi);
	int rotate_type;

	rotate_type = simple_strtoul(buf, NULL, 0);

	if (rotate_type != OMAPFB_ROT_DMA &&
			rotate_type != OMAPFB_ROT_VRFB)
		return -EINVAL;

	//ofbi->rotation_type = rotate_type;

	return count;
}

static DEVICE_ATTR(rotate_type, S_IRUGO | S_IWUSR,
		show_rotate_type, store_rotate_type);

static int omapfb_create_framebuffers(struct omapfb2_device *fbdev)
{
	int r, i;

	fbdev->num_fbs = 0;

	DBG("create %d framebuffers\n",	CONFIG_FB_OMAP2_NUM_FBS);

	/* allocate fb_infos */
	for (i = 0; i < CONFIG_FB_OMAP2_NUM_FBS; i++) {
		struct fb_info *fbi;
		struct omapfb_info *ofbi;

		fbi = framebuffer_alloc(sizeof(struct omapfb_info),
				fbdev->dev);

		if (fbi == NULL) {
			dev_err(fbdev->dev,
				"unable to allocate memory for plane info\n");
			return -ENOMEM;
		}

		fbdev->fbs[i] = fbi;

		ofbi = FB2OFB(fbi);
		ofbi->fbdev = fbdev;
		ofbi->id = i;
		/* initialize the vsync wait queue */
		init_waitqueue_head(&ofbi->vsync_wait);
		fbdev->num_fbs++;
	}

	DBG("fb_infos allocated\n");

	/* assign overlays for the fbs */
	for (i = 0; i < min(fbdev->num_fbs, fbdev->num_overlays); i++) {
		struct omapfb_info *ofbi = FB2OFB(fbdev->fbs[i]);

		ofbi->overlays[0] = fbdev->overlays[i];
		ofbi->num_overlays = 1;
	}

	/* allocate fb memories */
	r = omapfb_allocate_all_fbs(fbdev);
	if (r) {
		dev_err(fbdev->dev, "failed to allocate fbmem\n");
		return r;
	}

	DBG("fbmems allocated\n");

	/* setup fb_infos */
	for (i = 0; i < fbdev->num_fbs; i++) {
		r = fbinfo_init(fbdev, fbdev->fbs[i]);
		if (r) {
			dev_err(fbdev->dev, "failed to setup fb_info\n");
			return r;
		}
	}

	DBG("fb_infos initialized\n");

	for (i = 0; i < fbdev->num_fbs; i++) {
		r = register_framebuffer(fbdev->fbs[i]);
		if (r != 0) {
			dev_err(fbdev->dev,
				"registering framebuffer %d failed\n", i);
			return r;
		}
	}

	DBG("framebuffers registered\n");

	for (i = 0; i < fbdev->num_fbs; i++) {
		r = omapfb_apply_changes(fbdev->fbs[i], 1);
		if (r) {
			dev_err(fbdev->dev, "failed to change mode\n");
			return r;
		}
	}

	DBG("create sysfs for fbs\n");
	for (i = 0; i < fbdev->num_fbs; i++) {
		r = device_create_file(fbdev->fbs[i]->dev,
				&dev_attr_rotate_type);
		if (r) {
			dev_err(fbdev->dev, "failed to create sysfs file\n");
			return r;
		}
	}

	/* Enable fb0 */
	if (fbdev->num_fbs > 0) {
		struct omapfb_info *ofbi = FB2OFB(fbdev->fbs[0]);

		if (ofbi->num_overlays > 0 ) {
			struct omap_overlay *ovl = ofbi->overlays[0];

			r = ovl->enable(ovl, 1);

			if (r) {
				dev_err(fbdev->dev,
						"failed to enable overlay\n");
				return r;
			}
		}
	}

	DBG("create_framebuffers done\n");

	return 0;
}

int omapfb_mode_to_timings(const char *mode_str,
		struct omap_video_timings *timings, unsigned *bpp)
{
	struct fb_info fbi;
	struct fb_var_screeninfo var;
	struct fb_ops fbops;
	int r;

	/* this is quite a hack, but I wanted to use the modedb and for
	 * that we need fb_info and var, so we create dummy ones */

	memset(&fbi, 0, sizeof(fbi));
	memset(&var, 0, sizeof(var));
	memset(&fbops, 0, sizeof(fbops));
	fbi.fbops = &fbops;

	r = fb_find_mode(&var, &fbi, mode_str, NULL, 0, NULL, 24);

	if (r != 0) {
		timings->pixel_clock = PICOS2KHZ(var.pixclock);
		timings->hfp = var.left_margin;
		timings->hbp = var.right_margin;
		timings->vfp = var.upper_margin;
		timings->vbp = var.lower_margin;
		timings->hsw = var.hsync_len;
		timings->vsw = var.vsync_len;
		timings->x_res = var.xres;
		timings->y_res = var.yres;

		switch (var.bits_per_pixel) {
		case 16:
			*bpp = 16;
			break;
		case 24:
		case 32:
		default:
			*bpp = 24;
			break;
		}

		return 0;
	} else {
		return -EINVAL;
	}
}

static int omapfb_probe(struct platform_device *pdev)
{
	struct omapfb2_device *fbdev = NULL;
	int r = 0;
	int i, t;
	struct omap_overlay *ovl;
	struct omap_display *def_display;

	DBG("omapfb_probe\n");

	if (pdev->num_resources != 0) {
		dev_err(&pdev->dev, "probed for an unknown device\n");
		r = -ENODEV;
		goto err0;
	}

	fbdev = kzalloc(sizeof(struct omapfb2_device), GFP_KERNEL);
	if (fbdev == NULL) {
		r = -ENOMEM;
		goto err0;
	}

	mutex_init(&fbdev->mtx);

	fbdev->dev = &pdev->dev;
	platform_set_drvdata(pdev, fbdev);

	fbdev->num_displays = 0;
	t = omap_dss_get_num_displays();
	for (i = 0; i < t; i++) {
		struct omap_display *display;
		display = omap_dss_get_display(i);
		if (!display) {
			dev_err(&pdev->dev, "can't get display %d\n", i);
			r = -EINVAL;
			goto cleanup;
		}

		fbdev->displays[fbdev->num_displays++] = display;
	}

	if (fbdev->num_displays == 0) {
		dev_err(&pdev->dev, "no displays\n");
		r = -EINVAL;
		goto cleanup;
	}

	fbdev->num_overlays = omap_dss_get_num_overlays();
	for (i = 0; i < fbdev->num_overlays; i++)
		fbdev->overlays[i] = omap_dss_get_overlay(i);

	fbdev->num_managers = omap_dss_get_num_overlay_managers();
	for (i = 0; i < fbdev->num_managers; i++)
		fbdev->managers[i] = omap_dss_get_overlay_manager(i);

	/* gfx overlay should be the default one. find a display
	 * connected to that, and use it as default display */
	ovl = omap_dss_get_overlay(0);
	if (ovl->manager && ovl->manager->display) {
		def_display = ovl->manager->display;
	} else {
		dev_err(&pdev->dev, "cannot find default display\n");
		r = -EINVAL;
		goto cleanup;
	}

	if (def_mode && strlen(def_mode) > 0)
	{
		struct omap_video_timings timings;
		unsigned bpp;

		if (omapfb_mode_to_timings(def_mode, &timings, &bpp) == 0) {
			if (def_display->set_timings)
				def_display->set_timings(def_display, &timings);

			def_display->panel->bpp = bpp;
		}
	}

	r = omapfb_create_framebuffers(fbdev);
	if (r)
		goto cleanup;

	for (i = 0; i < fbdev->num_managers; i++) {
		struct omap_overlay_manager *mgr;
		mgr = fbdev->managers[i];
		r = mgr->apply(mgr);
		if (r) {
			dev_err(fbdev->dev, "failed to apply dispc config\n");
			goto cleanup;
		}
	}

	DBG("mgr->apply'ed\n");

	r = def_display->enable(def_display);
	if (r) {
		dev_err(fbdev->dev, "Failed to enable display '%s'\n",
				def_display->name);
		goto cleanup;
	}

	/* set the update mode */
	if (def_display->caps & OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE) {
#ifdef CONFIG_FB_OMAP2_FORCE_AUTO_UPDATE
		if (def_display->set_update_mode)
			def_display->set_update_mode(def_display,
					OMAP_DSS_UPDATE_AUTO);
		if (def_display->enable_te)
			def_display->enable_te(def_display, 1);
#else
		if (def_display->set_update_mode)
			def_display->set_update_mode(def_display,
					OMAP_DSS_UPDATE_MANUAL);
		if (def_display->enable_te)
			def_display->enable_te(def_display, 0);
#endif
	} else {
		if (def_display->set_update_mode)
			def_display->set_update_mode(def_display,
					OMAP_DSS_UPDATE_AUTO);
	}

	for (i = 0; i < fbdev->num_displays; i++) {
		struct omap_display *display = fbdev->displays[i];
		int w, h;

		display->get_resolution(display, &w, &h);

		if (display->update)
			display->update(display, 0, 0, w, h);
	}

	DBG("display->updated\n");

	omapfb_create_sysfs(fbdev);
	DBG("sysfs created\n");

	return 0;

cleanup:
	omapfb_free_resources(fbdev);
err0:
	dev_err(&pdev->dev, "failed to setup omapfb\n");
	return r;
}

static int omapfb_remove(struct platform_device *pdev)
{
	struct omapfb2_device *fbdev = platform_get_drvdata(pdev);
	int i;

	/* FIXME: wait till completion of pending events */

	DBG("remove sysfs for fbs\n");
	for (i = 0; i < fbdev->num_fbs; i++) {
		device_remove_file(fbdev->fbs[i]->dev, &dev_attr_rotate_type);
	}

	omapfb_remove_sysfs(fbdev);

	omapfb_free_resources(fbdev);

	return 0;
}

static struct platform_driver omapfb_driver = {
	.probe          = omapfb_probe,
	.remove         = omapfb_remove,
	.driver         = {
		.name   = "omapfb",
		.owner  = THIS_MODULE,
	},
};

static int __init omapfb_init(void)
{
	DBG("omapfb_init\n");

	if (platform_driver_register(&omapfb_driver)) {
		printk(KERN_ERR "failed to register omapfb driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit omapfb_exit(void)
{
	DBG("omapfb_exit\n");
	platform_driver_unregister(&omapfb_driver);
}

module_param_named(video_mode, def_mode, charp, 0);
module_param_named(vram, def_vram, charp, 0);
module_param_named(rotate, def_rotate, int, 0);
module_param_named(rotate_type, def_rotate_type, int, 0);

/* late_initcall to let panel/ctrl drivers loaded first.
 * I guess better option would be a more dynamic approach,
 * so that omapfb reacts to new panels when they are loaded */
late_initcall(omapfb_init);
/*module_init(omapfb_init);*/
module_exit(omapfb_exit);

MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@nokia.com>");
MODULE_DESCRIPTION("OMAP2/3 Framebuffer");
MODULE_LICENSE("GPL v2");
