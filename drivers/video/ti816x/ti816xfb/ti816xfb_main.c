/*
 * linux/drivers/video/ti816x/ti816xfb/ti816xfb_main.c
 *
 * Framebuffer driver for TI 816X
 *
 * Copyright (C) 2009 Texas Instruments
 * Author: Yihe Hu(yihehu@ti.com)
 *
 * Some codes and ideals are from TI OMAP2 by Tomi Valkeinen
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

#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/ti816xfb.h>
#include <plat/ti816x_ram.h>

#include "fbpriv.h"

#define MODULE_NAME "ti816xfb"

/*the followign are defined to as the module or bootargs parameters */
static char *def_vram;
/*normal mode*/
static int fb_mmode = 1;

#ifdef DEBUG
unsigned int fb_debug;
module_param_named(debug, fb_debug, bool, 0644);
#endif


u32 ti816xfb_get_fb_paddr(struct ti816xfb_info *tfbi)
{
	return tfbi->mreg.paddr;
}

void __iomem *ti816xfb_get_fb_vaddr(struct ti816xfb_info *tfbi)
{
	return tfbi->mreg.vaddr;
}

static struct ti816xfb_datamode tfb_datamodes[] = {
	{
		.dataformat = FVID2_DF_RGB16_565,
		.nonstd = 0, /*TI816XFB_RGB565,*/
		.red    = {.offset = 11, .length = 5, .msb_right = 0},
		.green  = {.offset = 5,  .length = 6, .msb_right = 0},
		.blue   = {.offset = 0,  .length = 5, .msb_right = 0},
		.transp = {.offset = 0,  .length = 0, .msb_right = 0},
		.bpp    = 16,
	}, {
		.dataformat = FVID2_DF_ARGB16_1555,
		.nonstd = 0,/*TI816XFB_ARGB1555,*/
		.red    = {.offset = 10, .length = 5, .msb_right = 0},
		.green  = {.offset = 5,  .length = 5, .msb_right = 0},
		.blue   = {.offset = 0,  .length = 5, .msb_right = 0},
		.transp = {.offset = 15, .length = 1, .msb_right = 0},
		.bpp    = 16,

	}, {
		.dataformat = FVID2_DF_ARGB16_4444,
		.nonstd = 0,/*TI816XFB_ARGB4444,*/
		.red    = {.offset = 8,  .length = 4, .msb_right = 0},
		.green  = {.offset = 4,  .length = 4, .msb_right = 0},
		.blue   = {.offset = 0,  .length = 4, .msb_right = 0},
		.transp = {.offset = 12, .length = 4, .msb_right = 0},
		.bpp    = 16,
	}, {
		.dataformat = FVID2_DF_RGBA16_5551,
		.nonstd = 0, /*TI816XFB_RGBA5551,*/
		.red    = {.offset = 11, .length = 5, .msb_right = 0},
		.green  = {.offset = 6,  .length = 5, .msb_right = 0},
		.blue   = {.offset = 1,  .length = 5, .msb_right = 0},
		.transp = {.offset = 0,  .length = 1, .msb_right = 0},
		.bpp    = 16,

	}, {
		.dataformat = FVID2_DF_RGBA16_4444,
		.nonstd = 0, /*TI816XFB_RGBA4444,*/
		.red    = {.offset = 12, .length = 4, .msb_right = 0},
		.green  = {.offset = 8,  .length = 4, .msb_right = 0},
		.blue   = {.offset = 4,  .length = 4, .msb_right = 0},
		.transp = {.offset = 0,  .length = 4, .msb_right = 0},
		.bpp    = 16,
	}, {

		.dataformat = FVID2_DF_ARGB24_6666,
		.nonstd = 0, /*ETRAFB_ARGB6666,*/
		.red    = {.offset = 12, .length = 6, .msb_right = 0},
		.green  = {.offset = 6,  .length = 6, .msb_right = 0},
		.blue   = {.offset = 0,  .length = 6, .msb_right = 0},
		.transp = {.offset = 18, .length = 6, .msb_right = 0},
		.bpp = 24,
	}, {
		.dataformat = FVID2_DF_RGB24_888,
		.nonstd = 0, /*TI816XFB_RGB888,*/
		.red    = {.offset = 16, .length = 8, .msb_right = 0},
		.green  = {.offset = 8,  .length = 8, .msb_right = 0},
		.blue   = {.offset = 0,  .length = 8, .msb_right = 0},
		.transp = {.offset = 0,  .length = 0, .msb_right = 0},
		.bpp = 24,
	}, {
		.dataformat = FVID2_DF_ARGB32_8888,
		.nonstd = 0, /*TI816XFB_ARGB8888,*/
		.red    = {.offset = 16, .length = 8, .msb_right = 0},
		.green  = {.offset = 8,  .length = 8, .msb_right = 0},
		.blue   = {.offset = 0,  .length = 8, .msb_right = 0},
		.transp = {.offset = 24, .length = 8, .msb_right = 0},
		.bpp    = 32,
	}, {
		.dataformat = FVID2_DF_RGBA24_6666,
		.nonstd = 0, /*TI816XFB_RGBA6666,*/
		.red    = {.offset = 18, .length = 6, .msb_right = 0},
		.green  = {.offset = 12, .length = 6, .msb_right = 0},
		.blue   = {.offset = 6,  .length = 6, .msb_right = 0},
		.transp = {.offset = 0,  .length = 6, .msb_right = 0},
		.bpp    = 24,
	}, {
		.dataformat = FVID2_DF_RGBA32_8888,
		.nonstd =  0, /*TI816XFB_RGBA8888,*/
		.red    = {.offset = 24, .length = 8, .msb_right = 0},
		.green  = {.offset = 16, .length = 8, .msb_right = 0},
		.blue   = {.offset = 8,  .length = 8, .msb_right = 0},
		.transp = {.offset = 0,  .length = 8, .msb_right = 0},
		.bpp    = 32,
	}, {
		.dataformat = FVID2_DF_BITMAP8,
		.nonstd = 0, /*TI816XFB_BMP8,*/
		.bpp    = 8,
	}, {
		.dataformat = FVID2_DF_BITMAP4_LOWER,
		.nonstd = 0,/*TI816XFB_BMP4_L.*/
		.bpp = 4,
	}, {
		.dataformat = FVID2_DF_BITMAP4_UPPER,
		.nonstd = TI816XFB_BMP4_U,
		.bpp    = 4,
	}, {
		.dataformat = FVID2_DF_BITMAP2_OFFSET0,
		.nonstd = 0, /*TI816XFB_BMP2_OFF0,*/
		.bpp = 2,
	}, {
		.dataformat = FVID2_DF_BITMAP2_OFFSET1,
		.nonstd = TI816XFB_BMP2_OFF1,
		.bpp = 2,
	}, {
		.dataformat = FVID2_DF_BITMAP2_OFFSET2,
		.nonstd = TI816XFB_BMP2_OFF2,
		.bpp = 2,
	}, {

		.dataformat = FVID2_DF_BITMAP2_OFFSET3,
		.nonstd = TI816XFB_BMP2_OFF3,
		.bpp = 2,
	}, {
		.dataformat = FVID2_DF_BITMAP1_OFFSET0,
		.nonstd = 0, /*TI816XFB_BMP1_OFF0,*/
		.bpp = 1,

	}, {
		.dataformat = FVID2_DF_BITMAP1_OFFSET1,
		.nonstd = TI816XFB_BMP1_OFF1,
		.bpp = 1,

	}, {
		.dataformat = FVID2_DF_BITMAP1_OFFSET2,
		.nonstd = TI816XFB_BMP1_OFF2,
		.bpp = 1,

	}, {
		.dataformat = FVID2_DF_BITMAP1_OFFSET3,
		.nonstd = TI816XFB_BMP1_OFF3,
		.bpp = 1,

	}, {
		.dataformat = FVID2_DF_BITMAP1_OFFSET4,
		.nonstd = TI816XFB_BMP1_OFF4,
		.bpp = 1,

	}, {
		.dataformat = FVID2_DF_BITMAP1_OFFSET5,
		.nonstd = TI816XFB_BMP1_OFF5,
		.bpp = 1,

	}, {
		.dataformat = FVID2_DF_BITMAP1_OFFSET6,
		.nonstd = TI816XFB_BMP1_OFF6,
		.bpp = 1,

	}, {
		.dataformat = FVID2_DF_BITMAP1_OFFSET6,
		.nonstd = TI816XFB_BMP1_OFF6,
		.bpp = 1,
	}, {
		.dataformat = FVID2_DF_BITMAP1_OFFSET7,
		.nonstd = TI816XFB_BMP1_OFF7,
		.bpp = 1,
	},
};

static bool cmp_var_to_vpssmode(struct fb_var_screeninfo *var,
		struct ti816xfb_datamode *dmode)
{
	bool cmp_component(struct fb_bitfield *f1, struct fb_bitfield *f2)
	{
		return f1->length == f2->length &&
			f1->offset == f2->offset &&
			f1->msb_right == f2->msb_right;
	}

	if (var->bits_per_pixel == 0 ||
			var->red.length == 0 ||
			var->blue.length == 0 ||
			var->green.length == 0)
		return 0;

	return var->bits_per_pixel == dmode->bpp &&
		cmp_component(&var->red, &dmode->red) &&
		cmp_component(&var->green, &dmode->green) &&
		cmp_component(&var->blue, &dmode->blue) &&
		cmp_component(&var->transp, &dmode->transp);
}

/*get the var from the predefine table*/
static void get_tfb_datamode(struct ti816xfb_datamode *dmode,
			struct fb_var_screeninfo *var)
{
	var->bits_per_pixel = dmode->bpp;
	var->red = dmode->red;
	var->green = dmode->green;
	var->blue = dmode->blue;
	var->transp = dmode->transp;
	var->nonstd = dmode->nonstd;
}

/*get the ti816x vpss mode from fb var infor*/
static enum fvid2_dataformat tfb_datamode_to_vpss_datamode(
				struct fb_var_screeninfo *var)
{
	int i;
	enum fvid2_dataformat  df;
	if (var->nonstd) {
		for (i = 0; i < ARRAY_SIZE(tfb_datamodes); i++) {
			struct ti816xfb_datamode *dmode = &tfb_datamodes[i];
			if (var->nonstd == dmode->nonstd) {
				get_tfb_datamode(dmode, var);
				return dmode->dataformat;
			}
		}
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(tfb_datamodes); i++) {
		struct ti816xfb_datamode *dmode = &tfb_datamodes[i];
		if (cmp_var_to_vpssmode(var, dmode)) {
			get_tfb_datamode(dmode, var);
			return dmode->dataformat;
		}
	}

	/* if user do not specify var color infor,
	  use the default setting based on the bpp, which
	  may not be right*/

	switch (var->bits_per_pixel) {
	case 32:
		df = FVID2_DF_ARGB32_8888;
		break;
	case 24:
		df = FVID2_DF_RGB24_888;
		break;
	case 16:
		df = FVID2_DF_ARGB16_1555;
		break;
	case 8:
		df = FVID2_DF_BITMAP8;
		break;
	case 4:
		df = FVID2_DF_BITMAP4_LOWER;
		break;
	case 2:
		df = FVID2_DF_BITMAP2_OFFSET0;
		break;
	case 1:
		df = FVID2_DF_BITMAP1_OFFSET0;
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(tfb_datamodes); i++) {
		struct ti816xfb_datamode *dmode = &tfb_datamodes[i];
		if (df == dmode->dataformat) {
			get_tfb_datamode(dmode, var);
			return df;
		}
	}
	return -EINVAL;

}
/*get fb mode from ti816x data format*/
static int vpss_datamode_to_tfb_datamode(enum fvid2_dataformat df,
					 struct fb_var_screeninfo *var)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(tfb_datamodes); i++) {
		struct ti816xfb_datamode *dmode = &tfb_datamodes[i];
		if (df == dmode->dataformat) {
			get_tfb_datamode(dmode, var);
			return 0;
		}
	}
	return -ENOENT;
}
/* check new var and modify it if possible */
static int check_fb_var(struct fb_info *fbi, struct fb_var_screeninfo *var)
{
	int				bpp;
	struct ti816xfb_info		*tfbi = FB2TFB(fbi);
	struct vps_grpxregionparams	regp;
	struct vps_grpx_ctrl		*gctrl = tfbi->gctrl;
	enum   fvid2_dataformat		df;
	unsigned long			line_size;
	u32				w, h;
	u8				scfmt;

	TFBDBG("check_fb_var\n");

	if (tfbi->mreg.size == 0)
		return 0;

	/*get the data format first*/
	df = tfb_datamode_to_vpss_datamode(var);
	if (df < 0) {
		TFBDBG("var infor is not valid data format.\n");
		return -EINVAL;
	}
	/*get the new output format if applicable*/
	gctrl->get_resolution(gctrl, &w, &h, &scfmt);

	bpp = var->bits_per_pixel;

	/* incase virtual res was set to 0. */
	if (var->xres_virtual == 0)
		var->xres_virtual = var->xres;

	if (var->yres_virtual == 0)
		var->yres_virtual = var->yres;

	if (var->xres > w)
		var->xres = w;

	if (var->yres > h)
		var->yres = h;

	/*boundry check for the VAR*/
	if (var->xres_virtual < var->xres)
		var->xres = var->xres_virtual;
	if (var->yres_virtual < var->yres)
		var->yres = var->yres_virtual;

	/*check if we have enough memory to support*/
	line_size = var->xres_virtual * bpp >> 3;
	if (line_size * var->yres_virtual > tfbi->mreg.size) {
		TFBDBG("can't fit fb memory, reducing y\n");
		var->yres_virtual = tfbi->mreg.size / line_size;

		if (var->yres > var->yres_virtual)
			var->yres = var->yres_virtual;
	}
	if (line_size * var->yres_virtual > tfbi->mreg.size) {
		TFBDBG("can't fit fb memory, reducing x\n");

		var->xres_virtual = tfbi->mreg.size / var->yres_virtual /
			(bpp >> 3);

		if (var->xres > var->xres_virtual)
			var->xres = var->xres_virtual;

		line_size = var->xres_virtual * bpp >> 3;

	}

	if (line_size * var->yres_virtual > tfbi->mreg.size) {
		dev_err(tfbi->fbdev->dev,
			"cannot fit fb memory\n");
		return -EINVAL;
	}

	if (var->xoffset > var->xres_virtual - var->xres)
		var->xoffset = var->xres_virtual - var->xres;
	if (var->yoffset > var->yres_virtual - var->yres)
		var->yoffset = var->yres_virtual - var->yres;
	/*pitch check*/
	if ((var->xres_virtual * bpp >> 3) & 0xF) {
		dev_err(tfbi->fbdev->dev,
			"buffer pitch %d is not on 16byte boundary",
			(var->xres_virtual * bpp >> 3));
		return -EINVAL;
	}

	TFBDBG("xres = %d, yres = %d, vxres = %d, vyres = %d\n",
			var->xres, var->yres,
			var->xres_virtual, var->yres_virtual);

	/* check whether the new size out of frame*/
	gctrl->get_regparams(gctrl, &regp);

	regp.regionheight = var->yres;
	regp.regionwidth = var->xres;
	if (gctrl->check_params(gctrl, &regp, 0)) {
		dev_err(tfbi->fbdev->dev,
			"var failed params checking.\n");
		return -EINVAL;
	}

	var->height = -1;
	var->width = -1;
	var->grayscale = 0;

	/*FIX ME timing information should got from media controller
	through FVID2 interface */
	var->pixclock = 0;
	var->left_margin = 0;
	var->right_margin = 0;
	var->upper_margin = 0;
	var->lower_margin = 0;
	var->hsync_len = 0;
	var->vsync_len = 0;
	var->sync = 0;
	/*FIX ME should vmode set by others*/
	if (tfbi->idx == 2)
		var->vmode = FB_VMODE_INTERLACED;
	else
		var->vmode = FB_VMODE_NONINTERLACED;

	return 0;
}

static void set_fb_fix(struct fb_info *fbi)
{
	struct fb_fix_screeninfo *fix = &fbi->fix;
	struct fb_var_screeninfo *var = &fbi->var;
	struct ti816xfb_info *tfbi = FB2TFB(fbi);
	struct ti816xfb_mem_region *mreg = &tfbi->mreg;
	int bpp = var->bits_per_pixel;

	TFBDBG("set_fb_fix\n");

	/* init mem*/
	fbi->screen_base = (char __iomem *) ti816xfb_get_fb_vaddr(tfbi);
	fix->line_length = (var->xres_virtual * bpp >> 3);
	/*pitch should be in 16 byte boundary*/
	if (fix->line_length & 0xF)
		fix->line_length += 16 - (fix->line_length & 0xF);

	fix->smem_start = ti816xfb_get_fb_paddr(tfbi);
	fix->smem_len = mreg->size;

	fix->type = FB_TYPE_PACKED_PIXELS;

	fix->visual = (bpp > 8) ?
	   FB_VISUAL_TRUECOLOR : FB_VISUAL_PSEUDOCOLOR;

	fix->accel = FB_ACCEL_NONE;
	fix->xpanstep = 1;
	fix->ypanstep = 1;

}

static int ti816xfb_grpx_delete(struct fb_info *fbi)
{
	int r = 0;
	struct ti816xfb_info  *tfbi = FB2TFB(fbi);
	struct vps_grpx_ctrl *gctrl = tfbi->gctrl;

	if (tfbi->open_cnt > 1) {
		tfbi->open_cnt--;
		TFBDBG("dummy close fb%d\n", tfbi->idx);
		return 0;
	} else if (tfbi->open_cnt == 0)
		return 0;


	TFBDBG("Closing fb%d\n", tfbi->idx);
	if (gctrl->handle) {
		if (gctrl->gstate.isstarted) {
			r = vps_fvid2_stop(gctrl->handle, NULL);
			if (r == 0)
				gctrl->stop(gctrl);
		}

		if (r == 0) {
			r = vps_fvid2_delete(gctrl->handle, NULL);
			if (r == 0) {
				gctrl->delete(gctrl);
				tfbi->open_cnt--;
			} else
				dev_err(tfbi->fbdev->dev,
					  "failed to delete fvid2 handle.\n");

		} else
			dev_err(tfbi->fbdev->dev,
				"failed to stop.\n");

	}
	return r;
}
static int ti816xfb_apply_changes(struct fb_info *fbi, int init)
{
	struct fb_var_screeninfo	*var = &fbi->var;
	struct ti816xfb_info		*tfbi = FB2TFB(fbi);
	struct vps_grpx_ctrl		*gctrl = tfbi->gctrl;
	struct vps_grpxregionparams	regp;
	enum   fvid2_dataformat		df;
	int				r = 0;
	int				offset;
	u32				buf_addr;

	if (tfbi->mreg.size == 0)
		return 0;

	/*get the fvid2 data format first*/
	df = tfb_datamode_to_vpss_datamode(var);
	if (df < 0) {
		TFBDBG("unsupported data format.\n");
		return -EINVAL;

	}
	/* make sure that the new changes are valid size*/
	gctrl->set_format(gctrl, var->bits_per_pixel,
		df, fbi->fix.line_length);

	gctrl->get_regparams(gctrl, &regp);


	/* update the region size*/
	if (0 == r) {

		offset = (var->xoffset * fbi->var.bits_per_pixel >> 3)
				+ var->yoffset * fbi->fix.line_length;

		buf_addr = ti816xfb_get_fb_paddr(tfbi) + offset;

		if ((regp.regionheight != var->yres) ||
			(regp.regionwidth != var->xres)) {

			regp.regionwidth = var->xres;
			regp.regionheight = var->yres;
			if (1 == init)
				r = gctrl->check_params(gctrl, &regp, 0);
			if (0 == r)
				r = gctrl->set_regparams(gctrl, &regp);
		}
		if (buf_addr != gctrl->buffer_addr)
			r = gctrl->set_buffer(tfbi->gctrl, buf_addr);

		if ((r == 0) && (gctrl->gstate.isstarted))
			r = vps_fvid2_queue(
				gctrl->handle,
				(struct fvid2_framelist *)gctrl->frmls_phy,
				0);
	}


	return r;
}

static int ti816xfb_blank(int blank, struct fb_info *fbi)
{
	int r = 0;
	struct ti816xfb_info *tfbi = FB2TFB(fbi);
	struct vps_grpx_ctrl *gctrl = tfbi->gctrl;

	ti816xfb_lock(tfbi);
	switch (blank) {
		/*FIX ME how to unblank the system*/
	case FB_BLANK_UNBLANK:
		if (tfbi->gctrl->gstate.isstarted)
			goto exit;
		r = vps_fvid2_start(tfbi->gctrl->handle, NULL);
		if (r == 0)
			gctrl->start(gctrl);


		break;
	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		if (tfbi->gctrl->gstate.isstarted == 0)
			goto exit;
		r = vps_fvid2_stop(tfbi->gctrl->handle, NULL);
		if (r == 0)
			gctrl->stop(gctrl);
		break;
	default:
		r = -EINVAL;
		break;
	}
exit:
	ti816xfb_unlock(tfbi);
	return 0;
}

static int ti816xfb_pan_display(struct fb_var_screeninfo *var,
					  struct fb_info *fbi)
{
	int r = 0;
	struct ti816xfb_info *tfbi = FB2TFB(fbi);

	TFBDBG("pan_display(%d)\n", tfbi->idx);

	if (tfbi->mreg.size == 0)
		return -EINVAL;

	if (var->xoffset > (fbi->var.xres_virtual - fbi->var.xres))
		return -EINVAL;
	if (var->yoffset > (fbi->var.yres_virtual - fbi->var.yres))
		return -EINVAL;

	ti816xfb_lock(tfbi);
	if (var->xoffset != fbi->var.xoffset ||
		var->yoffset != fbi->var.yoffset) {
		struct fb_var_screeninfo new_var;

		new_var = fbi->var;

		new_var.xoffset = var->xoffset;
		new_var.yoffset = var->yoffset;

		r = check_fb_var(fbi, var);
		if (0 == r) {
			fbi->var = new_var;
			set_fb_fix(fbi);
			r = ti816xfb_apply_changes(fbi, 0);
		}
	}
	ti816xfb_unlock(tfbi);

	return r;
}

static int ti816xfb_setcmap(struct fb_cmap *cmap, struct fb_info *fbi)
{
	int				i, index, r = 0;
	struct ti816xfb_info		*tfbi = FB2TFB(fbi);
	struct vps_grpx_ctrl		*gctrl = tfbi->gctrl;
	unsigned long			*palette = tfbi->vclut;
	u16				*red, *green, *blue, *transp;
	static u32			tempclut[256];

	TFBDBG("setcamp(%d)\n", tfbi->idx);

	red	= cmap->red;
	green	= cmap->green;
	blue	= cmap->blue;
	transp	= cmap->transp;
	index	= cmap->start;

	if (cmap->len != TI816XFB_CLUT_ENTRY) {
		dev_err(tfbi->fbdev->dev, "can not set the CLUT\n");
		return -EINVAL;
	}

	ti816xfb_lock(tfbi);
	for (i = 0; i < TI816XFB_CLUT_ENTRY; i++) {
		palette[i] =
			(((*red++) & TI816XFB_CLUT_MASK) << 24) |
			 (((*green++) & TI816XFB_CLUT_MASK) << 16) |
			 (((*blue++) & TI816XFB_CLUT_MASK) << 8) |
			 ((*transp++) & TI816XFB_CLUT_MASK);
		index++;
	}
	/*only update if these is a new settings*/
	if (memcmp(palette, tempclut, TI816XFB_CLUT_SIZE)) {
		TFBDBG("clut not same ,set\n");
		r = gctrl->set_clutptr(gctrl, (u32)tfbi->pclut);
		if ((r == 0) && (gctrl->gstate.isstarted)) {
			r = vps_fvid2_queue(
				gctrl->handle,
				(struct fvid2_framelist *)gctrl->frmls_phy,
				0);
		}
		memcpy(tempclut, palette, TI816XFB_CLUT_SIZE);
	}

	ti816xfb_unlock(tfbi);
	return r;
}


static int ti816xfb_check_var(struct fb_var_screeninfo *var,
					   struct fb_info *fbi)
{
	int r = 0;
	struct ti816xfb_info *tfbi = FB2TFB(fbi);

	if (tfbi->mreg.size == 0)
		return -EINVAL;

	TFBDBG("check_var(%d)\n", FB2TFB(fbi)->idx);

	ti816xfb_lock(tfbi);
	r = check_fb_var(fbi, var);
	ti816xfb_unlock(tfbi);
	return r;
}

static int ti816xfb_set_var(struct fb_info *fbi)
{
	int r = 0;
	struct ti816xfb_info *tfbi = FB2TFB(fbi);

	if (tfbi->mreg.size == 0)
		return -EINVAL;

	TFBDBG("set_var(%d)\n", tfbi->idx);

	ti816xfb_lock(tfbi);

	set_fb_fix(fbi);

	/*set the new fbi*/
	r = ti816xfb_apply_changes(fbi, 0);
	ti816xfb_unlock(tfbi);

	return r;
}

static int ti816xfb_open(struct fb_info *fbi, int user)
{
	int r = 0;
	struct ti816xfb_info		*tfbi = FB2TFB(fbi);
	struct vps_grpx_ctrl		*gctrl = tfbi->gctrl;
	u32				grpxinstid;


	if (tfbi->mreg.size == 0) {
		dev_err(tfbi->fbdev->dev,
			"please allocate memory first.\n");
		return -EINVAL;

	}
	ti816xfb_lock(tfbi);

	if (tfbi->open_cnt != 0) {
		tfbi->open_cnt++;
		ti816xfb_unlock(tfbi);
		TFBDBG("Dummy open fb%d\n", tfbi->idx);
		return 0;
	}

	TFBDBG("Opening fb%d\n", tfbi->idx);
	if (r == 0) {

		r = gctrl->create(gctrl);

		if (gctrl->grpx_num == 0)
			grpxinstid = VPS_DISP_INST_GRPX0;
		else if (gctrl->grpx_num == 1)
			grpxinstid = VPS_DISP_INST_GRPX1;
		else
			grpxinstid = VPS_DISP_INST_GRPX2;

		if (r == 0)
			gctrl->handle = vps_fvid2_create(
				FVID2_VPS_DISP_GRPX_DRV,
				grpxinstid,
				(void *)gctrl->gcp_phy,
				(void *)gctrl->gcs_phy,
				(struct fvid2_cbparams *)gctrl->cbp_phy);
	}


	if (gctrl->handle == NULL) {
		dev_err(tfbi->fbdev->dev,
			"fvid2 create failed.\n");
		r = -EINVAL;
	} else {

		/*Set the format from fvid2 create function*/
		if (r == 0)
			r = vps_fvid2_setformat(gctrl->handle,
					     (struct fvid2_format *)
						gctrl->inputf_phy);
		/*set the region params*/
		if (r == 0)
			r = vps_fvid2_control(gctrl->handle,
					  IOCTL_VPS_SET_GRPX_PARAMS,
					  (struct vps_grpxparamlist *)
						gctrl->glist_phy,
					   NULL);
		/*queue buffer*/
		if (r == 0) {
			gctrl->set_buffer(gctrl,
					  ti816xfb_get_fb_paddr(tfbi));
			r = vps_fvid2_queue(gctrl->handle,
					  (struct fvid2_framelist *)
					     gctrl->frmls_phy,
					  0);
		}
		/*start*/
		if (r == 0)
			r = vps_fvid2_start(gctrl->handle, NULL);

		if (r == 0) {
			gctrl->start(gctrl);
			tfbi->open_cnt++;
		} else {
			/*fail to start the grpx, delete it and start over*/
			dev_err(tfbi->fbdev->dev,
				"failed to star.\n");

			if (vps_fvid2_delete(gctrl->handle, NULL) == 0) {
				gctrl->delete(gctrl);
				gctrl->handle = NULL;
			}
		}
	}

	ti816xfb_unlock(tfbi);
	/*FIX ME we allocate the page and tiler memory from here.
	in the ti816x, DMM and make the isolated physical memory into
	continuous memory pace, so we do not need allocate the memory
	at the boot time, we can allocate in the open time to save the memory*/

	return r;
}
static int ti816xfb_release(struct fb_info *fbi, int user)
{
	int r = 0;
	struct ti816xfb_info  *tfbi = FB2TFB(fbi);

	/*FIXME in the page and tiler memory mode, the memory deallocate will be
	done in this function.*/
	ti816xfb_lock(tfbi);
	r = ti816xfb_grpx_delete(fbi);
	ti816xfb_unlock(tfbi);
	return r;
}

static int ti816xfb_mmap(struct fb_info *fbi, struct vm_area_struct *vma)
{
	struct ti816xfb_info		*tfbi = FB2TFB(fbi);
	struct ti816xfb_device		*fbdev = tfbi->fbdev;
	struct ti816xfb_alloc_list	*mem;
	unsigned long			offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long			start;
	u32				len;
	bool				found = false;


	TFBDBG("vram end %lx start %lx, offset %lx\n",
		vma->vm_end, vma->vm_start, offset);
	if ((vma->vm_end - vma->vm_start) == 0)
		return 0;

	if (offset < fbi->fix.smem_len) {
		/* mapping framebuffer memory*/
		len = fbi->fix.smem_len - offset;
		start = ti816xfb_get_fb_paddr(tfbi);
		offset += start;
		if (vma->vm_pgoff > (~0UL > PAGE_SHIFT))
			return -EINVAL;

		vma->vm_pgoff = offset >> PAGE_SHIFT;

	} else {
		/* mapping stenciling memory*/
		list_for_each_entry(mem, &tfbi->alloc_list, list) {
			if (offset == mem->phy_addr) {
				found = true;
				len = mem->size;
				start = offset;
				break;
			}
		}
		if (false == found)
			return -EINVAL;
	}


	len = PAGE_ALIGN(len);
	if ((vma->vm_end - vma->vm_start) > len)
		return -EINVAL;

	TFBDBG("user mmap regions start %lx, len %d\n",
				start, len);
	vma->vm_flags |= VM_IO | VM_RESERVED;
	/* make buffers bufferable*/
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	if (io_remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
		vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		dev_dbg(fbdev->dev, "mmap remap_pfn_range failed.\n");
		return -ENOBUFS;
	}
	return 0;
}


static struct fb_ops ti816xfb_ops = {
	.owner = THIS_MODULE,
	.fb_open = ti816xfb_open,
	.fb_release = ti816xfb_release,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_set_par = ti816xfb_set_var,
	.fb_check_var = ti816xfb_check_var,
	.fb_blank = ti816xfb_blank,
	.fb_setcmap = ti816xfb_setcmap,
	.fb_pan_display = ti816xfb_pan_display,
	.fb_ioctl = ti816xfb_ioctl,
	.fb_mmap = ti816xfb_mmap,
};

static void ti816xfb_free_fbmem(struct fb_info *fbi)
{
	struct ti816xfb_info		*tfbi = FB2TFB(fbi);
	struct ti816xfb_device		*fbdev = tfbi->fbdev;
	struct ti816xfb_mem_region	*rg = &tfbi->mreg;

	if (rg->paddr) {
		if (ti816x_vram_free(rg->paddr,
				 (void *)rg->vaddr,
				 (size_t)rg->size))
			dev_err(fbdev->dev, "VRAM FREE failed\n");
	}

	rg->vaddr = NULL;
	rg->paddr = 0;
	rg->alloc = 0;
	rg->size = 0;

}

static int ti816xfb_free_allfbmem(struct ti816xfb_device *fbdev)
{
	int i;

	for (i = 0; i < fbdev->num_fbs; i++) {
		struct fb_info *fbi = fbdev->fbs[i];
		ti816xfb_free_fbmem(fbi);
		memset(&fbi->fix, 0, sizeof(fbi->fix));
		memset(&fbi->var, 0, sizeof(fbi->var));
	}

		TFBDBG("free all fbmem\n");
	return 0;
}

static int ti816xfb_alloc_fbmem(struct fb_info *fbi, unsigned long size)
{
	struct ti816xfb_info	   *tfbi = FB2TFB(fbi);
	struct ti816xfb_device     *fbdev = tfbi->fbdev;
	struct ti816xfb_mem_region *rg = &tfbi->mreg;
	unsigned long		   paddr;
	void			   *vaddr;


	size = PAGE_ALIGN(size);
	memset(rg, 0, sizeof(*rg));

	TFBDBG("allocating %lu bytes for fb %d\n",
		size, tfbi->idx);

	vaddr = (void *)ti816x_vram_alloc(TI816XFB_MEMTYPE_SDRAM,
			(size_t)size, &paddr);

	TFBDBG("allocated VRAM paddr %lx, vaddr %p\n", paddr, vaddr);
	if (vaddr == NULL) {
		dev_err(fbdev->dev,
			"failed to allocate framebuffer\n");
		return -ENOMEM;
	}

	rg->paddr = paddr;
	rg->vaddr = vaddr;
	rg->size = size;
	rg->alloc = 1;

	fbi->screen_size = size;
	return 0;
}

static int ti816xfb_alloc_fbmem_display(struct fb_info *fbi, unsigned long size)
{
	struct ti816xfb_info  *tfbi = FB2TFB(fbi);
	struct vps_grpx_ctrl *gctrl = tfbi->gctrl;
	if (!size) {

		u32 width, height;
		u8 scformat;
		gctrl->get_resolution(gctrl, &width, &height, &scformat);
		size = (width * height * TI816XFB_BPP >> 3) * 2;

	}
	return ti816xfb_alloc_fbmem(fbi, size);
}

static int ti816xfb_parse_vram_param(const char *param, int max_entries,
		unsigned long *sizes)
{
	int		fbnum;
	unsigned long	size;
	char		*p, *start;

	start = (char *)param;

	TFBDBG("vram %s\n", param);
	while (1) {
		p = start;

		fbnum = simple_strtoul(p, &p, 10);

		if (p == param)
			return -EINVAL;

		if (*p != ':')
			return -EINVAL;

		if (fbnum >= max_entries)
			return -EINVAL;

		size = memparse(p + 1, &p);

		if (!size)
			return -EINVAL;

		sizes[fbnum] = size;

		TFBDBG("fb %d size %d\n", fbnum, (u32)size);
		if (*p == 0)
			break;

		if (*p != ',')
			return -EINVAL;

		++p;

		start = p;
	}

	return 0;
}

static int ti816xfb_allocate_fbs(struct ti816xfb_device *fbdev)
{
	int i, r;
	unsigned long vrams[TI816X_FB_NUM];

	memset(vrams, 0, sizeof(vrams));

	if (def_vram && ti816xfb_parse_vram_param(def_vram, TI816X_FB_NUM,
		 vrams)) {
		dev_err(fbdev->dev,
		"failed to parse vram parameters: %s\n", def_vram);

		memset(vrams, 0, sizeof(vrams));
	}

	if (fbdev->dev->platform_data) {
		struct ti816xfb_platform_data *npd;
		npd = fbdev->dev->platform_data;
		for (i = 0; i < npd->mem_desc.region_cnt; ++i) {
			if (!vrams[i])
				vrams[i] = npd->mem_desc.mreg[i].size;
		}
	}

	for (i = 0; i < fbdev->num_fbs; i++) {
		/* allocate memory automatically only for fb0, or if
		 * excplicitly defined with vram or plat data option */
		if (i == 0 || vrams[i] != 0) {
			r = ti816xfb_alloc_fbmem_display(fbdev->fbs[i],
					vrams[i]);

			if (r)
				return r;
		}
	}

	for (i = 0; i < fbdev->num_fbs; i++) {
		struct ti816xfb_info *tfbi = FB2TFB(fbdev->fbs[i]);
		struct ti816xfb_mem_region *rg = &tfbi->mreg;

		TFBDBG("region%d phys %08x virt %p size=%lu\n",
				i,
				rg->paddr,
				rg->vaddr,
				rg->size);
	}

	return 0;

}


int ti816xfb_realloc_fbmem(struct fb_info *fbi, unsigned long size)
{

	struct ti816xfb_info *tfbi = FB2TFB(fbi);
	struct ti816xfb_device *fbdev = tfbi->fbdev;
	struct ti816xfb_mem_region *rg = &tfbi->mreg;
	unsigned long old_size = rg->size;
	int r;

	size = PAGE_ALIGN(size);
	ti816xfb_free_fbmem(fbi);
	if (size == 0) {
		memset(&fbi->fix, 0, sizeof(fbi->fix));
		memset(&fbi->var, 0, sizeof(fbi->var));
		return 0;
	}

	r = ti816xfb_alloc_fbmem(fbi, size);

	if (r) {
		if (old_size)
			ti816xfb_alloc_fbmem(fbi, old_size);

		if (rg->size == 0) {
			memset(&fbi->fix, 0, sizeof(fbi->fix));
			memset(&fbi->var, 0, sizeof(fbi->var));
		}

		return r;
	}

	if (old_size == size)
		return 0;

	if (old_size == 0) {
		TFBDBG("initializing fb %d\n", tfbi->idx);
		r = ti816xfb_fbinfo_init(fbdev, fbi);
		if (r) {
			TFBDBG("ti816xfb_fbinfo_init failed\n");
			goto err;
		}
		r = ti816xfb_apply_changes(fbi, 1);
		if (r) {
			TFBDBG("ti816xfb_apply_changes failed\n");
			goto err;
		}
	} else {
		struct fb_var_screeninfo new_var;
		memcpy(&new_var, &fbi->var, sizeof(new_var));
		r = check_fb_var(fbi, &new_var);
		if (r)
			goto err;
		memcpy(&fbi->var, &new_var, sizeof(fbi->var));
		set_fb_fix(fbi);
	}

	return 0;
err:
	ti816xfb_free_fbmem(fbi);
	memset(&fbi->fix, 0, sizeof(fbi->fix));
	memset(&fbi->var, 0, sizeof(fbi->var));
	return r;
}


static void fbi_framebuffer_unreg(struct ti816xfb_device *fbdev)
{
	int i;

	for (i = 0; i < fbdev->num_fbs; i++)
		unregister_framebuffer(fbdev->fbs[i]);
}

static int ti816xfb_alloc_clut(struct ti816xfb_device *fbdev,
			       struct fb_info *fbi)
{
	struct ti816xfb_info *tfbi = FB2TFB(fbi);

	/*allocate CLUT */
	tfbi->vclut = dma_alloc_writecombine(fbdev->dev,
			TI816XFB_CLUT_SIZE, &tfbi->pclut, GFP_KERNEL);
	if (NULL == tfbi->vclut) {
		dev_err(fbdev->dev, "failed to alloca pallette memory.\n");
		return -ENOMEM;
	}
	return 0;
}

int ti816xfb_fbinfo_init(struct ti816xfb_device *fbdev,
			struct fb_info *fbi)
{
	struct fb_var_screeninfo	*var = &fbi->var;
	struct fb_fix_screeninfo	*fix = &fbi->fix;
	struct ti816xfb_info		*tfbi = FB2TFB(fbi);
	struct vps_grpx_ctrl		*gctrl = tfbi->gctrl;
	u32				w, h;
	u8				scfmt;

	int r = 0;

	fbi->fbops = &ti816xfb_ops;
	fbi->flags = FBINFO_FLAG_DEFAULT;
	strncpy(fix->id, MODULE_NAME, sizeof(fix->id));
	fbi->pseudo_palette = tfbi->pseudo_palette;

	/*default is ARGB8888*/
	vpss_datamode_to_tfb_datamode(FVID2_DF_ARGB32_8888, var);
	gctrl->get_resolution(gctrl, &w, &h, &scfmt);

	var->xres = w;
	var->yres = h;
	var->xres_virtual = var->xres;
	var->yres_virtual = var->yres;


	r = check_fb_var(fbi, var);
	if (0 == r) {
		set_fb_fix(fbi);

		/*allocate the cmap with trans enable*/
		r = fb_alloc_cmap(&fbi->cmap, 256, 1);
		if (0 != r)
			dev_err(fbdev->dev,
				"unable to allocate color map memory\n");
	}
	return r;
}


static void free_clut_ram(struct ti816xfb_device *fbdev, struct fb_info *fbi)
{
	struct ti816xfb_info *tfbi = FB2TFB(fbi);
	if (tfbi->vclut)
		dma_free_writecombine(fbdev->dev, TI816XFB_CLUT_SIZE,
				     tfbi->vclut, tfbi->pclut);
	tfbi->vclut = NULL;
	tfbi->pclut = 0;
}

static void ti816xfb_fbinfo_cleanup(struct ti816xfb_device *fbdev,
				   struct fb_info *fbi)
{
	fb_dealloc_cmap(&fbi->cmap);
}


static void ti816xfb_fb_cleanup(struct ti816xfb_device *fbdev)
{
	int i;
	for (i = 0; i < fbdev->num_fbs; i++) {
		ti816xfb_fbinfo_cleanup(fbdev, fbdev->fbs[i]);
		free_clut_ram(fbdev, fbdev->fbs[i]);
		framebuffer_release(fbdev->fbs[i]);
	}
}

static void ti816xfb_free_all(struct ti816xfb_device *fbdev)
{
	TFBDBG("free all resources.\n");

	if (fbdev == NULL)
		return;

	/* unregister frame buffer devices first*/
	fbi_framebuffer_unreg(fbdev);

	/* free the frame buffer memory */
	ti816xfb_free_allfbmem(fbdev);
	/* free the fbi and release framebuffer*/
	ti816xfb_fb_cleanup(fbdev);
	dev_set_drvdata(fbdev->dev, NULL);
	/* free the device*/
	kfree(fbdev);

}

static int ti816xfb_create_framebuffers(struct ti816xfb_device *fbdev)
{
	int i, r;

	fbdev->num_fbs = 0;
	TFBDBG("create %d fbs\n", CONFIG_FB_TI816X_NUM_FBS);

	/* allocate fb_info */
	for (i = 0; i < CONFIG_FB_TI816X_NUM_FBS; i++) {

		struct fb_info *fbi;
		struct ti816xfb_info *tfbi;

		fbi = framebuffer_alloc(sizeof(struct ti816xfb_info),
			fbdev->dev);
		if (NULL == fbi) {
			dev_err(fbdev->dev,
				"unable to allocate memory for fb %d\n", i);
			return -ENOMEM;
		}
		fbdev->fbs[i] = fbi;
		tfbi = FB2TFB(fbi);
		tfbi->fbdev = fbdev;
		tfbi->idx = i;
		INIT_LIST_HEAD(&tfbi->alloc_list);
		mutex_init(&tfbi->rqueue_mutex);

		/*initialize the vsync wait queue*/
		init_waitqueue_head(&tfbi->vsync_wait);
		tfbi->vsync_cnt = 0;
		tfbi->open_cnt = 0;
		fbdev->num_fbs++;
		/*FIX ME anything to assign here????*/

	}

	TFBDBG("fb_infos(%d) allocated.\n", fbdev->num_fbs);


	/* assign grpx ctrl for the fbs*/
	for (i = 0; i < min(fbdev->num_fbs, fbdev->num_grpx); i++) {
		struct ti816xfb_info *tfbi = FB2TFB(fbdev->fbs[i]);

		tfbi->gctrl = fbdev->gctrl[i];
	}

	/* allocate frame buffer memory*/
	r = ti816xfb_allocate_fbs(fbdev);
	if (r) {
		dev_err(fbdev->dev, "failed to allocate fb memory.\n");
		return r;
	}

	TFBDBG("fb memory allocated.\n");

	for (i = 0; i < fbdev->num_fbs; i++) {
		r = ti816xfb_alloc_clut(fbdev, fbdev->fbs[i]);
		if (r) {
			dev_err(fbdev->dev,
				"failed to allocate clut memory.\n");
			return r;
		}
	}

	TFBDBG("clut mem allocated.\n");

	/*setup fbs*/
	for (i = 0; i < fbdev->num_fbs; i++) {
		r = ti816xfb_fbinfo_init(fbdev, fbdev->fbs[i]);
		if (r != 0) {
			dev_err(fbdev->dev, "fbinfo %d init failed.\n", i);
			return r;
		}
	}
	TFBDBG("fb_infos initialized\n");

	for (i = 0; i < fbdev->num_fbs; i++) {
		r = register_framebuffer(fbdev->fbs[i]);
		if (r != 0) {
			dev_err(fbdev->dev,
				"registering framebuffer %d failed\n", i);
			return r;
		}
	}

	TFBDBG("framebuffers registered\n");

	for (i = 0; i < fbdev->num_fbs; i++) {
		r = ti816xfb_apply_changes(fbdev->fbs[i], 1);
		if (r) {
			dev_err(fbdev->dev, "failed to change mode\n");
			return r;
		}
	}

	r = ti816xfb_create_sysfs(fbdev);
	if (r) {
		dev_err(fbdev->dev, "failed to create sysfs entries\n");
		return r;
	}
	TFBDBG("fbs sysfs created\n");
	return 0;

}

#if 0
static int ti816xfb_parse_def_modes(struct ti816xfb_device *fbdev)

{
	char *str, *options, *this_opt;
	int r = 0;

	str = kmalloc(strlen(def_mode) + 1, GFP_KERNEL);
	strcpy(str, def_mode);
	options = str;
	TFBDBG("def_mode %s\n", def_mode);
	while (!r && (this_opt = strsep(&options, ",")) != NULL) {
		char *p, *display_str, *mode_str;
		int vid, mid;
		p = strchr(this_opt, ':');
		if (!p) {
			r = -EINVAL;
			break;
		}

		*p = 0;
		display_str = this_opt;
		mode_str = p + 1;
		if (vps_dc_get_vencid(display_str, &vid)) {
			TFBDBG("venc name(%s) not existing.\n", display_str);
			continue;
		}
		if (vps_dc_get_modeid(mode_str, &mid)) {
			TFBDBG("venc mode(%s) is not supported.\n", mode_str);
			continue;
		}
		r = vps_dc_set_vencmode(&vid,
					&mid, NULL, 1, 0);
		if (r) {
			TFBDBG("set vencmode failed\n");
			continue;
		}

	   if (options == NULL)
			break;
	}

	kfree(str);

	return r;

}
#endif
static int ti816xfb_probe(struct platform_device *dev)
{
	struct ti816xfb_device  *fbdev = NULL;
	int r = 0;
	int i = 0;
	int t;

	if (dev->num_resources != 0) {
		dev_err(&dev->dev, "probed for an unknown device\n");
		r = -ENODEV;
		goto cleanup;
	}

	if (dev->dev.platform_data == NULL) {
		dev_err(&dev->dev, "missing platform data\n");
		r = -ENOENT;
		goto cleanup;
	}

	fbdev = kzalloc(sizeof(struct ti816xfb_device), GFP_KERNEL);
	if (NULL == fbdev) {
		dev_err(&dev->dev, "unable to allocate memory for device\n");
		r = -ENOMEM;
		goto cleanup;
	}

	fbdev->dev = &dev->dev;
	platform_set_drvdata(dev, fbdev);

	fbdev->num_grpx = 0;
	t = vps_grpx_get_num_grpx();
	for (i = 0; i < t; i++) {
		struct vps_grpx_ctrl *gctrl;

		gctrl = vps_grpx_get_ctrl(i);
		if (gctrl == NULL) {
			dev_err(&dev->dev, "can't get gctrl %d\n", i);
			r = -EINVAL;
			goto cleanup;
		}
		fbdev->gctrl[fbdev->num_grpx++] = gctrl;
	}

	if (fbdev->num_grpx == 0) {
		dev_err(&dev->dev, "no grpxs\n");
		r = -EINVAL;
		goto cleanup;
	}

	r = ti816xfb_create_framebuffers(fbdev);
	if (0 != r)
		goto cleanup;

	return 0;

cleanup:
	ti816xfb_free_all(fbdev);
	return r;
}

static int ti816xfb_remove(struct platform_device *dev)
{
	struct ti816xfb_device *fbdev = platform_get_drvdata(dev);
	int i;
	/*make sure all fb has been closed*/
	for (i = 0; i < fbdev->num_fbs; i++)
		ti816xfb_grpx_delete(fbdev->fbs[i]);
	TFBDBG("remove sysfs for fbs.\n");
	ti816xfb_remove_sysfs(fbdev);
	ti816xfb_free_all(fbdev);
	return 0;
}


static struct platform_driver ti816xfb_driver = {
	.probe = ti816xfb_probe,
	.remove = ti816xfb_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
	},
};


static int __init ti816xfb_init(void)
{

	TFBDBG("ti816xfb_init\n");
	if (platform_driver_register(&ti816xfb_driver)) {
		printk(KERN_ERR "failed to register ti816xfb driver\n");
		return -ENODEV;
	}
	return 0;
}


static void __exit ti816xfb_exit(void)
{
	platform_driver_unregister(&ti816xfb_driver);
}

module_param_named(vram, def_vram, charp, 0);
module_param_named(mmode, fb_mmode, int, 0664);
/*module_param_named(mode, def_mode, charp, 0);*/

late_initcall(ti816xfb_init);
module_exit(ti816xfb_exit);

MODULE_DESCRIPTION("TI TI816X framebuffer driver");
MODULE_AUTHOR("Yihe HU<yihehu@ti.com>");
MODULE_LICENSE("GPL v2");
