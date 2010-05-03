/*
 * linux/drivers/video/ti816x/vpss/grpx.c
 *
 * VPSS graphics driver for TI 816X
 *
 * Copyright (C) 2009 TI
 * Author: Yihe Hu <yihehu@ti.com>
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
#include <linux/err.h>
#include <linux/platform_device.h>
#include <plat/ti816x_ram.h>

#include <linux/vps_proxyserver.h>
#include <linux/fvid2.h>
#include <linux/vps.h>
#include <linux/vps_displayctrl.h>
#include <linux/vps_graphics.h>
#include <linux/dc.h>
#include <linux/grpx.h>

#include "core.h"

#ifdef DEBUG
#define GRPXDBG(format, ...) \
	if (vpss_debug) \
		printk(KERN_INFO"VPSS_GRPX: " \
			format, ## __VA_ARGS__)

#define GRPXERR(format, ...) \
	if (vpss_debug) \
		printk(KERN_ERR"VPSS_GRPX: " \
			format, ## __VA_ARGS__)

#else
#define GRPXDBG(format, ...)
#define GRPXERR(format, ...)
#endif

static int num_gctrl;
static struct list_head gctrl_list;
static struct vps_grpx_ctrl *grpx_ctrls[VPS_DISP_GRPX_MAX_INST];

static struct platform_device *grpx_dev;

struct vps_grpx_ctrl *get_grpx_ctrl_from_handle(void * handle)
{
	struct vps_grpx_ctrl *gctrl;

	list_for_each_entry(gctrl, &gctrl_list, list) {
		if (handle == gctrl->handle)
			return gctrl;

	}

	return NULL;

}

static void grpx_status_reset(struct vps_grpx_ctrl *gctrl)
{

	gctrl->frames->perframecfg = NULL;
	gctrl->framelist->perlistcfg = NULL;
	gctrl->grtlist->clutptr = NULL;
	gctrl->grtlist->scparams = NULL;
	gctrl->grtconfig->stenptr = NULL;
	gctrl->grtconfig->scparams = NULL;

	gctrl->gstate.regset = 0;
	gctrl->gstate.scset = 0;
	gctrl->gstate.clutSet = 0;
	gctrl->gstate.stenset = 0;
	gctrl->gstate.varset = 0;
}

/* this will be the call back register to the VPSS m3 for the VSYNC isr*/
static int vps_grpx_vsync_cb(void *handle, void *appdata, void * reserved)
{
	struct vps_grpx_ctrl *gctrl = get_grpx_ctrl_from_handle(handle);

	if (gctrl == NULL)
		return -EINVAL;

	GRPXDBG("get CB handle 0x%08x appData 0x%08x\n",
		(u32)handle, (u32)appdata);
	if (gctrl->gstate.isstarted) {

		grpx_status_reset(gctrl);
		/*release the semphare to tell VYSNC is comming*/
		if (gctrl->vsync_cb)
			gctrl->vsync_cb(gctrl->vcb_arg);
	}
	return 0;
}

static int vps_grpx_apply_changes(struct vps_grpx_ctrl *gctrl)
{
	struct vps_grpx_state  *gstate = &gctrl->gstate;
	int r = 0;

	GRPXDBG("apply changes into FVID2_FRAME.\n");

	if (gstate->isstarted) {
		if (gstate->scset) {
			gctrl->grtlist->scparams =
				(struct vps_grpxscparams *)gctrl->gscp_phy;
			gctrl->framelist->perlistcfg =
				(struct vps_grpxscparams *)gctrl->grtlist_phy;
		}

		if (gstate->clutSet) {
			gctrl->framelist->perlistcfg =
				(struct vps_grpxscparams *)gctrl->grtlist_phy;
		}

		if ((gstate->regset) || (gstate->varset)) {
			gctrl->frames->perframecfg =
				(struct vps_grpxrtparams *)gctrl->grtc_phy;
		}

		if (gstate->stenset)
			gctrl->frames->perframecfg =
				(struct vps_grpxrtparams *)gctrl->grtc_phy;

		/*r = vps_fvid2_queue(gctrl->handle,
				(struct fvid2_framelist *)gctrl->frmls_phy,0);
				*/

	} else {
		if (gstate->scset)
			gctrl->glist->scparams =
				(struct vps_grpxscparams *)gctrl->gscp_phy;

		gctrl->glist->gparams =
			(struct vps_grpxrtparams *)gctrl->gp_phy;

		/*no runtime update so far*/
		gctrl->framelist->perlistcfg = NULL;
		gctrl->frames->perframecfg = NULL;

		}



	return r;
}

static enum fvid2_bitsperpixel grpx_get_fbpp(u8 bpp)
{
	enum fvid2_bitsperpixel fbpp = FVID2_BPP_BITS16;

	switch (bpp) {
	case 1:
		fbpp = FVID2_BPP_BITS1;
		break;
	case 2:
		fbpp = FVID2_BPP_BITS2;
		break;
	case 4:
		fbpp = FVID2_BPP_BITS4;
		break;
	case 8:
		fbpp = FVID2_BPP_BITS8;
		break;
	case 16:
		fbpp = FVID2_BPP_BITS16;
		break;
	case 24:
		fbpp = FVID2_BPP_BITS24;
		break;
	case 32:
		fbpp = FVID2_BPP_BITS32;
	}
	return fbpp;
}

static int grpx_getbpp(enum fvid2_bitsperpixel fbpp)
{
	int bpp;

	switch (fbpp) {
	case FVID2_BPP_BITS1:
		bpp = 1;
		break;
	case FVID2_BPP_BITS2:
		bpp = 2;
		break;
	case FVID2_BPP_BITS4:
		bpp = 4;
		break;
	case FVID2_BPP_BITS8:
		bpp = 8;
		break;
	case FVID2_BPP_BITS16:
		bpp = 16;
		break;
	case FVID2_BPP_BITS24:
		bpp = 24;
		break;
	case FVID2_BPP_BITS32:
		bpp = 32;
	default:
		bpp = 32;
	}

	return bpp;

}

static int grpx_pre_start(struct vps_grpx_ctrl *gctrl)
{
	u32 w, h;
	u8  fmt;
	int bpp;
	u32 pitch;

	gctrl->get_resolution(gctrl, &w, &h, &fmt);
	/*set the dimension*/
	if ((gctrl->inputf->width == 0) || (gctrl->inputf->height == 0) ||
		(gctrl->inputf->width > w) || (gctrl->inputf->height > h))
		gctrl->set_input(gctrl, w, h, fmt);

	/*check the setting and reset to resolution if necessary*/
	if (gctrl->gparams->regparams.regionwidth +
	    gctrl->gparams->regparams.regionposx > w) {
		gctrl->gparams->regparams.regionwidth = w;
		gctrl->gparams->regparams.regionposx = 0;
		bpp = grpx_getbpp(gctrl->inputf->bpp);
		pitch = (w * bpp >> 3);
		/*pitch should 16 byte boundary*/
		if (pitch & 0xF)
			pitch += 16 - (pitch & 0xF);
		gctrl->gparams->pitch[FVID2_RGB_ADDR_IDX] =
					(w * bpp >> 3);

	}
	if (gctrl->gparams->regparams.regionheight +
	    gctrl->gparams->regparams.regionposy > h) {
		gctrl->gparams->regparams.regionheight = h;
		gctrl->gparams->regparams.regionposy = 0;
	}
	return 0;
}

static int vps_grpx_set_input(struct vps_grpx_ctrl *gctrl,
			u32 width, u32 height, u8 scfmt)
{
	u32 bpp, pitch;

	GRPXDBG("set input %d x %d P:%d.\n", width, height, scfmt);
	gctrl->inputf->height = height;
	gctrl->inputf->width = width;
	gctrl->inputf->scanformat = FVID2_SF_PROGRESSIVE;
	bpp = grpx_getbpp(gctrl->inputf->bpp);
	pitch = (width * bpp >> 3);
	/*pitch should 16 byte boundary*/
	if (pitch & 0xF)
		pitch += 16 - (pitch & 0xF);
	gctrl->inputf->pitch[FVID2_RGB_ADDR_IDX] = pitch;
	return 0;
}

static int vps_grpx_get_resolution(struct vps_grpx_ctrl *gctrl,
			 u32 *fwidth, u32 *fheight, u8 *scfmt)
{
	GRPXDBG("get resolution.\n");


	vps_dc_get_outpfmt(
		gctrl->enodes[0],
		&gctrl->framewidth,
		&gctrl->frameheight,
		&gctrl->scformat,
		DC_BLEND_ID);


	*fwidth = gctrl->framewidth;
	*fheight = gctrl->frameheight;
	*scfmt = gctrl->scformat;

	return 0;
}
static int vps_grpx_set_format(struct vps_grpx_ctrl *gctrl,
	u8 bpp, u32 df, u32 pitch)
{
	enum fvid2_bitsperpixel fbpp;


	fbpp = grpx_get_fbpp(bpp);

	gctrl->gparams->format = df;
	gctrl->gparams->pitch[FVID2_RGB_ADDR_IDX] = pitch;

	gctrl->inputf->bpp = fbpp;
	gctrl->inputf->channelnum = 0;
	gctrl->inputf->dataformat = df;
	gctrl->inputf->pitch[FVID2_RGB_ADDR_IDX] = pitch;

	if (gctrl->gstate.isstarted == false) {
		gctrl->gstate.varset = true;

		GRPXDBG("set format bpp %d df %d, pitch %d.\n",
			bpp, df, pitch);

	} else {
		if (df != gctrl->grtconfig->format) {
			gctrl->grtconfig->format = df;
			gctrl->gstate.varset = true;
			GRPXDBG("set format df %d.\n", df);
		}
		if (pitch != gctrl->grtconfig->pitch[FVID2_RGB_ADDR_IDX]) {
			gctrl->grtconfig->pitch[FVID2_RGB_ADDR_IDX] = pitch;
			gctrl->gstate.varset = true;
			GRPXDBG("set format pitch %d.\n", pitch);
		}
	}


	vps_grpx_apply_changes(gctrl);
	return 0;
}

static int vps_grpx_check_regparams(struct vps_grpx_ctrl *gctrl,
			      struct vps_grpxregionparams *regp, u16 ridx)
{
	u16 fw, fh;
	u16 xend, yend;
	u8 vscaled, hscaled;
	struct vps_grpxscparams *scparam = gctrl->gscparams;

	/*FIX ME fw and fh should be get from the display controller,
	 by know we just put a default number here*/

	fw = gctrl->framewidth;
	fh = gctrl->frameheight;

	/* does not support stencling until stenciling buffer is set*/
	if (regp->stencilingenable == true)
		if (gctrl->gstate.stenset == false) {
			GRPXERR("grpx%d sten enable without valid pointer.\n",
				gctrl->grpx_num);
			return -1;
		}
	/**
	 * due to the hardware scaler limitation, the scaler will output more lines
	 * and pixes than what asked, this should be taken into
	 * consideration when doing the frame boundary check and regions
	 * overlap
	 */
	if (regp->scenable) {
		if (gctrl->gstate.scset == false) {
			GRPXERR("grpx%d scaling enable without coeff.\n",
				gctrl->grpx_num);
			return -1;
		}
		/*get the current region scaled type*/
		if (scparam->inheight > scparam->outheight)
			vscaled = 2;
		else if (scparam->inheight < scparam->outheight)
			vscaled = 1;
		else
			/*anti-flicker*/
			vscaled = 0;

		if (scparam->inwidth != scparam->outwidth)
			hscaled = 1;
		else
			hscaled = 0;
	} else {
		hscaled = 0;
		vscaled = 0;
	}
	/*output at most 2 line and 2 pixes when scaled*/
	if (0 != vscaled)
		yend = regp->regionposy + scparam->outheight +
				GRPX_SCALED_REGION_EXTRA_LINES;

	else
		yend = regp->regionposy + regp->regionheight;

	if (0 != hscaled)
		xend = regp->regionposx + scparam->outwidth +
				GRPX_SCALED_REGION_EXTRA_PIXES;
	else
		xend = regp->regionposx + regp->regionwidth;

	/*make sure the size is not out of the frame*/
	if ((xend > fw) ||
	   (yend > fh)) {
		GRPXERR("grpx%d region(%dx%d) out of frame(%dx%d).\n",
			gctrl->grpx_num, xend, yend,
			gctrl->framewidth, gctrl->frameheight);
		return -1;
	}
	/* we are expecting both first region and last
	 * region for single region case
	 */
	if (!((ridx == 0) && (regp->firstregion == true))) {
		GRPXERR("grpx%d first region wrong.\n", gctrl->grpx_num);
		return -1;
	}


	if (!((ridx == (gctrl->glist->numregions - 1)) &&
		(true == regp->lastregion))) {
		GRPXERR("grpx%d last region wrong.\n", gctrl->grpx_num);
		return -1;
	}
	return 0;
}
static int vps_grpx_set_stenparams(struct vps_grpx_ctrl *gctrl,
			  u32 stenptr, u32 pitch)
{
	int r = 0;

	gctrl->gparams->stenptr = (void *)stenptr;
	gctrl->grtconfig->stenptr = (void *)stenptr;
	gctrl->gparams->stenpitch = pitch;
	gctrl->grtconfig->stenpitch = pitch;
	if (stenptr == 0)
		gctrl->gstate.stenset = false;
	else
		gctrl->gstate.stenset = true;


	GRPXDBG("set stenciling %#x\n", (u32)gctrl->gparams->stenptr);
	r = vps_grpx_apply_changes(gctrl);
	return r;
}

static int vps_grpx_get_stenparams(struct vps_grpx_ctrl *gctrl,
			  u32 *stenaddr,
			  u32 *pitch)
{
	int r = 0;
	if (gctrl->gstate.stenset == true) {
		*stenaddr = (u32)gctrl->gparams->stenptr;
		*pitch = gctrl->gparams->stenpitch;
	} else {
		*stenaddr = 0;
		r = -1;
	}
	GRPXDBG("get stenciling %#x with stride 0x%x\n",
		(u32)*stenaddr, (u32)*pitch);
	return -1;
}

static int vps_grpx_set_scparams(struct vps_grpx_ctrl *gctrl,
		struct vps_grpxscparams *sci)
{
	int r = 0;

	/*need make sure that out_widht and out_height is inside the frame*/
	if ((sci->outwidth > gctrl->inputf->width) ||
		(sci->outheight > gctrl->inputf->height))
		return -1;

	GRPXDBG("set sc params %dx%d->%dx%d\n", sci->inwidth,
		sci->inheight, sci->outwidth, sci->outheight);
	/*set the scaling information*/
	memcpy(gctrl->gscparams, sci, sizeof(struct vps_grpxscparams));
	gctrl->gstate.scset = true;

	r = vps_grpx_apply_changes(gctrl);
	return r;
}

static int vps_grpx_get_scparams(struct vps_grpx_ctrl *gctrl,
			   struct vps_grpxscparams *sci)
{
	GRPXDBG("get sc params.\n");
	if (gctrl->gstate.scset == false)
		return -1;

	memcpy(sci, gctrl->gscparams, sizeof(struct vps_grpxscparams));
	return 0;

}

/* get the region parameters*/
static int vps_grpx_get_regparams(struct vps_grpx_ctrl *gctrl,
			    struct vps_grpxregionparams *gparams)
{
	GRPXDBG("get region params.\n");
	if (gctrl->gstate.isstarted)
		memcpy(gparams, &gctrl->grtconfig->regparams,
		       sizeof(struct vps_grpxregionparams));
	else
		memcpy(gparams, &gctrl->gparams->regparams,
		       sizeof(struct vps_grpxregionparams));

	return 0;
}

/* set the region parameters*/
static int vps_grpx_set_regparams(struct vps_grpx_ctrl *gctrl,
			    struct vps_grpxregionparams *gparams)
{
	int r = 0;

	GRPXDBG("set region params.\n");
	memcpy(&gctrl->grtconfig->regparams, gparams,
	       sizeof(struct vps_grpxregionparams));
	memcpy(&gctrl->gparams->regparams, gparams,
		sizeof(struct vps_grpxregionparams));

	/*store this is for the reopen the fb usage*/
	gctrl->inputf->width = gparams->regionwidth;
	gctrl->inputf->height = gparams->regionheight;

	gctrl->gstate.regset = true;

	r = vps_grpx_apply_changes(gctrl);
	return r;
}

static int vps_grpx_set_clutptr(struct vps_grpx_ctrl  *gctrl, u32 pclut)
{
	int r = 0;
	GRPXDBG("set clut %#x\n", pclut);

	gctrl->glist->clutptr = (void *)pclut;
	gctrl->grtlist->clutptr = (void *)pclut;

	gctrl->gstate.clutSet = true;
	r = vps_grpx_apply_changes(gctrl);
	return 0;
}

static void vps_grpx_add_ctrl(struct vps_grpx_ctrl *gctrl)
{
	++num_gctrl;
	list_add_tail(&gctrl->list, &gctrl_list);
}

static int vps_grpx_set_buffer(struct vps_grpx_ctrl *gctrl,
			 u32 buffer_addr)
{
	int r = 0;

	GRPXDBG("add buffer %#x\n", buffer_addr);


	gctrl->buffer_addr = buffer_addr;
	gctrl->frames->addr[FVID2_RGB_ADDR_IDX]
		[FVID2_RGB_ADDR_IDX] = (void *)buffer_addr;

	r = vps_grpx_apply_changes(gctrl);
	return r;
}



static int vps_grpx_create(struct vps_grpx_ctrl *gctrl)
{
	GRPXDBG("create grpx\n");
	gctrl->cbparams->cbfxn = vps_grpx_vsync_cb;
	gctrl->cbparams->appdata = NULL;
	gctrl->cbparams->errlist = NULL;
	gctrl->cbparams->errcbfnx = NULL;


	grpx_pre_start(gctrl);
	return 0;
}

static int vps_grpx_delete(struct vps_grpx_ctrl *gctrl)
{
	int r = 0;

	GRPXDBG("delete GRPX\n");

	/*set all state value back to default*/
	gctrl->gstate.clutSet = 0;
	gctrl->gstate.regset = 0;
	gctrl->gstate.scset = 0;
	gctrl->gstate.varset = 0;
	gctrl->gstate.stenset = 0;

	gctrl->grtlist->scparams = NULL;
	gctrl->grtlist->clutptr = NULL;
	gctrl->grtconfig->scparams = NULL;
	gctrl->glist->clutptr = NULL;

	gctrl->framelist->perlistcfg = NULL;
	gctrl->frames->perframecfg = NULL;

	gctrl->handle = NULL;

	return r;
}

static int vps_grpx_start(struct vps_grpx_ctrl *gctrl)
{
	gctrl->gstate.isstarted = true;
	return 0;
}
static int vps_grpx_stop(struct vps_grpx_ctrl *gctrl)
{
	gctrl->gstate.isstarted = false;
	return 0;
}

static int vps_grpx_enable(struct vps_grpx_ctrl *gctrl, bool en)
{
	int r = 0;

	if (gctrl->handle == NULL)
		return -EINVAL;

	if (en) {

		/*create the node and config it*/
		r = gctrl->create_dcconfig(gctrl);
		r = gctrl->set_dcconfig(gctrl, 1);
		if (r == 0) {
			grpx_pre_start(gctrl);
			/*start everything over, set format,
			  params, queue buffer*/
			r = vps_fvid2_setformat(
				gctrl->handle,
				(struct fvid2_format *)gctrl->inputf_phy);

			if (r == 0)
				r = vps_fvid2_control(gctrl->handle,
						  IOCTL_VPS_SET_GRPX_PARAMS,
						  (struct vps_grpxparamlist *)
							gctrl->glist_phy,
						   NULL);

			if (r == 0)
				r = vps_fvid2_queue(gctrl->handle,
						  (struct fvid2_framelist *)
						     gctrl->frmls_phy,
						  0);

			if (r == 0)
				r = vps_fvid2_start(gctrl->handle, NULL);
			/*set flag or clear the path if any errors are present*/
			if (r == 0)
				gctrl->start(gctrl);
			else
				r = gctrl->set_dcconfig(gctrl, 0);
		}
	} else {
		r = vps_fvid2_stop(gctrl->handle, NULL);
		if (r == 0) {
			gctrl->stop(gctrl);
			r = gctrl->set_dcconfig(gctrl, 0);
		}
	}
	return r;

}

static int vps_grpx_register_vsync_cb(struct vps_grpx_ctrl *gctrl,
				vsync_callback_t cb, void *arg)
{
	int r = 0;

	gctrl->vsync_cb = cb;
	gctrl->vcb_arg = arg;
	return r;
}

static int vps_grpx_unregister_vsync_cb(struct vps_grpx_ctrl *gctrl)
{
	gctrl->vsync_cb = NULL;
	gctrl->vcb_arg = NULL;
	return 0;
}

static int vps_grpx_create_dcconfig(struct vps_grpx_ctrl *gctrl)
{
	int i, r = 0;
	struct vps_dcconfig *cfg = &gctrl->dccfg;
	enum vps_dcmodeid  mid = VPS_DC_MODE_1080P_60;
	/*FIXME should check the node is already enable or not*/
	cfg->usecase = VPS_DC_USERSETTINGS;
	cfg->numedges = gctrl->numends;
	cfg->vencinfo.numvencs = gctrl->numends;
	vps_dc_get_clksrc(&cfg->dvo2clksrc, &cfg->hdcompclksrc);
	for (i = 0; i < gctrl->numends && (r == 0); i++)  {
		cfg->edgeinfo[i].startnode = gctrl->snode;
		cfg->edgeinfo[i].endnode = gctrl->enodes[i];
		r = vps_dc_get_vencmode(gctrl->enodes[i],
				       &cfg->vencinfo.modeinfo[i],
				       DC_BLEND_ID);

		if (i == 0)
			mid = cfg->vencinfo.modeinfo[i].modeid;
		else
			if (mid != cfg->vencinfo.modeinfo[i].modeid) {
				GRPXERR("the tied venc should have  \
						the same timing.\n");
				return -EINVAL;
			}

		/* if connect more then one blend, then the venc associated
		with blend need be tied and in the same format*/
		cfg->vencinfo.tiedvencs |= cfg->vencinfo.modeinfo[i].vencid;
	}
	/*clear the tie vence if only one blend is connect*/
	if (gctrl->numends == 1)
		cfg->vencinfo.tiedvencs = 0;

	return r;
}


static int vps_grpx_set_dcconfig(struct vps_grpx_ctrl *gctrl, u8 setflag)
{
	int r = 0;
	if (gctrl->gstate.isdcConfig != setflag) {
		r = vps_dc_set_config(&gctrl->dccfg, setflag);
		if (r == 0)
			gctrl->gstate.isdcConfig = setflag;
	}
	return r;
}
/*GRPX sysfs related function*/
/*show current grpx enabled status*/
static ssize_t graphics_enabled_show(struct vps_grpx_ctrl *gctrl,
				char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", gctrl->gstate.isstarted);
}
/*enable the current grpx*/
static ssize_t graphics_enabled_store(struct vps_grpx_ctrl *gctrl,
				      const char *buf, size_t size)
{
	bool enabled;
	int r;
	if (gctrl->handle == NULL) {
		GRPXERR("please open fb node first.\n");
		return -EINVAL;
	}

	enabled = simple_strtoul(buf, NULL, 10);
	if (gctrl->gstate.isstarted == enabled) {
		GRPXERR("Nothing to do.\n");
		return size;
	}

	grpx_lock(gctrl);
	r = vps_grpx_enable(gctrl, enabled);
	grpx_unlock(gctrl);

	if (r)
		return r;

	return size;
}

/*show how many blender are connecting to this grpx*/
static ssize_t graphics_nodes_show(struct vps_grpx_ctrl *gctrl,
				char *buf)
{
	int i, r;
	int l = 0;
	char name[10];

	for (i = 0; i < gctrl->numends; i++)  {
		if (i != 0)
			l += snprintf(buf + l, PAGE_SIZE - l, ",");

		r = vps_dc_get_node_name(gctrl->enodes[i], name);
		l += snprintf(buf + l, PAGE_SIZE - l, "%d:%s", i, name);
	}
	l += snprintf(buf + l, PAGE_SIZE - l, "\n");

	return l;

}
/*set the enodes for grpx in the format 0:XXXX,1,XXXX,2:XXXX,3:XXXX*/
static ssize_t graphics_nodes_store(struct vps_grpx_ctrl *gctrl,
				      const char *buf, size_t size)
{
	int r = 0;
	int total = 0;
	int i;
	u32 mid = VPS_DC_MODE_1080P_60;
	u32 tiedvenc = 0;
	char *input = (char *)buf, *this_opt;

	struct vps_dcmodeinfo minfo;
	if (gctrl->gstate.isstarted) {
		GRPXERR("please stop before continue.\n");
		return -EINVAL;
	}

	/*remove the "\n"*/
	input = strsep(&input, "\n");
	grpx_lock(gctrl);
	while (!r && (this_opt = strsep(&input, ",")) != NULL) {
		char *p, *endnode;
		int idx, nid;
		if (total == VPS_DC_MAX_VENC) {
			r = -EINVAL;
			goto exit;
		}
		p = strchr(this_opt, ':');
		if (!p) {
			r = -EINVAL;
			goto exit;
		}
		*p = 0;
		idx = simple_strtoul(this_opt, &this_opt, 10);
		if (idx >= VPS_DC_MAX_VENC) {
			r = -EINVAL;
			goto exit;
		}
		endnode = p + 1;
		if (vps_dc_get_node_id(&nid, endnode)) {
			r = -EINVAL;
			goto exit;
		}
		gctrl->enodes[idx] = nid;

		total++;
		if (input == NULL)
			break;


	}
	gctrl->numends = total;
	/*error check*/
	GRPXDBG("numedge :%d\n", gctrl->numends);
	for (i = 0; i < gctrl->numends; i++) {
		r = vps_dc_get_vencmode(gctrl->enodes[i],
				       &minfo,
				       DC_BLEND_ID);

		if (i == 0)
			mid = minfo.modeid;
		else
			if (mid != minfo.modeid) {
				GRPXERR("the tied venc should have  \
						the same timing.\n");
				r = -EINVAL;
				goto exit;
			}



		GRPXDBG("s: %d e:%d\n",
			gctrl->snode,
			gctrl->enodes[i]);
		GRPXDBG("vid:%d, mid:%d\n",
			minfo.vencid,
			minfo.modeid);
		tiedvenc |= minfo.vencid;

	}
	if (gctrl->numends == 1)
		tiedvenc = 0;
	GRPXDBG("tiedvecn :%d.\n", tiedvenc);
exit:
	grpx_unlock(gctrl);
	if (r)
		return r;
	return size;
}


struct graphics_attribute {
	struct attribute attr;
	ssize_t (*show)(struct vps_grpx_ctrl *, char *);
	ssize_t (*store)(struct vps_grpx_ctrl *, const char *, size_t);
};
#define GRAPHICS_ATTR(_name, _mode, _show, _store) \
	struct graphics_attribute graphics_attr_##_name = \
	__ATTR(_name, _mode, _show, _store)

static GRAPHICS_ATTR(enabled, S_IRUGO|S_IWUSR,
	graphics_enabled_show, graphics_enabled_store);

static GRAPHICS_ATTR(nodes, S_IRUGO | S_IWUSR,
	graphics_nodes_show, graphics_nodes_store);

static struct attribute *graphics_sysfs_attrs[] = {
	&graphics_attr_enabled.attr,
	&graphics_attr_nodes.attr,
	NULL
};

static ssize_t graphics_attr_show(struct kobject *kobj,
				  struct attribute *attr,
				  char *buf)
{
	struct vps_grpx_ctrl *gctrl;
	struct graphics_attribute *grpx_attr;

	gctrl = container_of(kobj, struct vps_grpx_ctrl, kobj);
	grpx_attr = container_of(attr, struct graphics_attribute, attr);

	if (!grpx_attr->show)
		return -ENOENT;

	return grpx_attr->show(gctrl, buf);
}

static ssize_t graphics_attr_store(struct kobject *kobj, struct attribute *attr,
		const char *buf, size_t size)
{
	struct vps_grpx_ctrl *gctrl;
	struct graphics_attribute *grpx_attr;

	gctrl = container_of(kobj, struct vps_grpx_ctrl, kobj);
	grpx_attr = container_of(attr, struct graphics_attribute, attr);

	if (!grpx_attr->store)
		return -ENOENT;

	return grpx_attr->store(gctrl, buf, size);
}


static const struct sysfs_ops graphics_sysfs_ops = {
	.show = graphics_attr_show,
	.store = graphics_attr_store,
};

static struct kobj_type graphics_ktype = {
	.sysfs_ops = &graphics_sysfs_ops,
	.default_attrs = graphics_sysfs_attrs,
};

/*end of GRPX sysfs*/
static int vps_grpx_create_sysfs(struct vps_grpx_ctrl *gctrl)
{
	int r;
	if (gctrl == NULL)
		return -EINVAL;

	r = kobject_init_and_add(&gctrl->kobj,
				 &graphics_ktype,
				 &grpx_dev->dev.kobj,
				 "graphics%d",
				 gctrl->grpx_num);
	if (r)
		GRPXERR("failed to create grpx%d sysfs file.\n",
			gctrl->grpx_num);
	return r;

}

static void vps_grpx_remove_sysfs(struct vps_grpx_ctrl *gctrl)
{
	kobject_del(&gctrl->kobj);
	kobject_put(&gctrl->kobj);

}

struct vps_grpx_ctrl *vps_grpx_get_ctrl(int num)
{
	int i = 0;
	struct vps_grpx_ctrl *gctrl;

	list_for_each_entry(gctrl, &gctrl_list, list) {
		if (i++ == num)
			return gctrl;
	}

	return NULL;
}
EXPORT_SYMBOL(vps_grpx_get_ctrl);

int vps_grpx_get_num_grpx(void)
{
	return num_gctrl;
}
EXPORT_SYMBOL(vps_grpx_get_num_grpx);

void __init vps_fvid2_grpx_ctrl_init(struct vps_grpx_ctrl *gctrl)
{
	gctrl->inputf->scanformat = FVID2_SF_PROGRESSIVE;

	gctrl->gcparam->memtype = VPS_VPDMA_MT_NONTILEDMEM;
	gctrl->gcparam->drvmode = VPS_GRPX_FRAME_BUFFER_MODE;

	/* init parameters*/
	gctrl->glist->numregions = 1;

	gctrl->gparams->regparams.firstregion = true;
	gctrl->gparams->regparams.lastregion = true;
	gctrl->gparams->regparams.disppriority = 1;

	gctrl->grtconfig->regparams.firstregion = true;
	gctrl->grtconfig->regparams.lastregion = true;
	gctrl->grtconfig->regparams.disppriority = 1;

	/* init the FVID2 related structures*/
	gctrl->framelist->numframes = 1;
	gctrl->framelist->frames[0] = (struct fvid2_frame *)gctrl->frm_phy;

	/*assign the function pointer*/
	gctrl->apply_changes = vps_grpx_apply_changes;
	gctrl->set_input = vps_grpx_set_input;
	gctrl->set_buffer = vps_grpx_set_buffer;
	gctrl->get_resolution = vps_grpx_get_resolution;
	gctrl->set_format = vps_grpx_set_format;
	gctrl->check_params = vps_grpx_check_regparams;
	gctrl->set_clutptr = vps_grpx_set_clutptr;
	gctrl->set_scparams = vps_grpx_set_scparams;
	gctrl->get_scparams = vps_grpx_get_scparams;
	gctrl->set_regparams = vps_grpx_set_regparams;
	gctrl->get_regparams = vps_grpx_get_regparams;
	gctrl->set_stenparams = vps_grpx_set_stenparams;
	gctrl->get_stenparams = vps_grpx_get_stenparams;
	gctrl->create = vps_grpx_create;
	gctrl->delete = vps_grpx_delete;
	gctrl->register_vsync_cb = vps_grpx_register_vsync_cb;
	gctrl->unregister_vsync_cb = vps_grpx_unregister_vsync_cb;
	gctrl->start = vps_grpx_start;
	gctrl->stop = vps_grpx_stop;
	gctrl->create_dcconfig = vps_grpx_create_dcconfig;
	gctrl->set_dcconfig = vps_grpx_set_dcconfig;

}

 void __init vps_grpx_params_alloc(struct vps_grpx_ctrl *gctrl)
{
	gctrl->gcparam = kzalloc(sizeof(struct vps_grpxcreateparams),
					GFP_KERNEL);
	BUG_ON(gctrl->gcparam == NULL);
	gctrl->gcp_phy = virt_to_phys(gctrl->gcparam);

	gctrl->gcstatus = kzalloc(sizeof(struct vps_grpxcreatestatus),
					 GFP_KERNEL);
	BUG_ON(gctrl->gcstatus == NULL);
	gctrl->gcs_phy = virt_to_phys(gctrl->gcstatus);

	gctrl->gscparams = kzalloc(sizeof(struct vps_grpxscparams),
				   GFP_KERNEL);
	BUG_ON(gctrl->gscparams == NULL);
	gctrl->gscp_phy = virt_to_phys(gctrl->gscparams);

	gctrl->gsccoeff = kzalloc(sizeof(struct vps_grpxsccoeff),
				  GFP_KERNEL);
	BUG_ON(gctrl->gsccoeff == NULL);
	gctrl->gscoff_phy = virt_to_phys(gctrl->gsccoeff);

	gctrl->gparams = kzalloc(sizeof(struct vps_grpxrtparams),
				 GFP_KERNEL);
	BUG_ON(gctrl->gparams == NULL);
	gctrl->gp_phy = virt_to_phys(gctrl->gparams);

	gctrl->grtconfig = kzalloc(sizeof(struct vps_grpxrtparams),
				   GFP_KERNEL);
	BUG_ON(gctrl->grtconfig == NULL);
	gctrl->grtc_phy = virt_to_phys(gctrl->grtconfig);

	gctrl->grtlist = kzalloc(sizeof(struct vps_grpxrtlist),
				 GFP_KERNEL);
	BUG_ON(gctrl->grtlist == NULL);
	gctrl->grtlist_phy = virt_to_phys(gctrl->grtlist);

	gctrl->glist = kzalloc(sizeof(struct vps_grpxparamlist),
			       GFP_KERNEL);
	BUG_ON(gctrl->glist == NULL);
	gctrl->glist_phy = virt_to_phys(gctrl->glist);

	/* allocate FVID2 structures*/
	gctrl->cbparams = kzalloc(sizeof(struct fvid2_cbparams),
				  GFP_KERNEL);
	BUG_ON(gctrl->cbparams == NULL);
	gctrl->cbp_phy = virt_to_phys(gctrl->cbparams);

	gctrl->inputf = kzalloc(sizeof(struct fvid2_format),
				GFP_KERNEL);
	BUG_ON(gctrl->inputf == NULL);
	gctrl->inputf_phy = virt_to_phys(gctrl->inputf);

	gctrl->framelist = kzalloc(sizeof(struct fvid2_framelist),
				   GFP_KERNEL);
	BUG_ON(gctrl->framelist == NULL);
	gctrl->frmls_phy = virt_to_phys(gctrl->framelist);

	gctrl->frames = kzalloc(sizeof(struct fvid2_frame),
				GFP_KERNEL);
	BUG_ON(gctrl->frames == NULL);
	gctrl->frm_phy = virt_to_phys(gctrl->frames);


}

 void __exit vps_grpx_params_dealloc(struct vps_grpx_ctrl *gctrl)
{
	/*free all resources*/
	kfree(gctrl->gcparam);

	kfree(gctrl->gcstatus);

	kfree(gctrl->gscparams);

	kfree(gctrl->gsccoeff);

	kfree(gctrl->gparams);

	kfree(gctrl->grtconfig);

	kfree(gctrl->grtlist);

	kfree(gctrl->glist);

	/* for fvid2 interface variables*/
	kfree(gctrl->cbparams);

	kfree(gctrl->inputf);

	kfree(gctrl->framelist);

	kfree(gctrl->frames);

}

int __init vps_grpx_init(struct platform_device *pdev)
{
	int i;

	GRPXDBG("GRPX INIT\n");
	INIT_LIST_HEAD(&gctrl_list);

	num_gctrl = 0;
	grpx_dev = pdev;
	for (i = 0; i < VPS_DISP_GRPX_MAX_INST; i++) {
		struct vps_grpx_ctrl *gctrl;
		gctrl = kzalloc(sizeof(*gctrl), GFP_KERNEL);

		if (gctrl == NULL) {
			GRPXERR("failed to allocate grpx%d\n", i);
			BUG_ON(gctrl == NULL);
		}

		vps_grpx_params_alloc(gctrl);
		vps_fvid2_grpx_ctrl_init(gctrl);
		gctrl->grpx_num = i;
		vps_grpx_add_ctrl(gctrl);
		grpx_ctrls[i] = gctrl;
		mutex_init(&gctrl->gmutex);
		gctrl->numends = 1;
		switch (i) {
		case 0:

			gctrl->snode = VPS_DC_GRPX0_INPUT_PATH;
			gctrl->enodes[0] = VPS_DC_HDMI_BLEND;
			break;
		case 1:
			gctrl->snode = VPS_DC_GRPX1_INPUT_PATH;
			gctrl->enodes[0] = VPS_DC_DVO2_BLEND;
			break;
		case 2:
			gctrl->snode = VPS_DC_GRPX2_INPUT_PATH;
			gctrl->enodes[0] = VPS_DC_SDVENC_BLEND;
			break;
		}
		vps_grpx_create_sysfs(gctrl);

	}
	return 0;

}

void __exit vps_grpx_deinit(struct platform_device *pdev)
{
	struct vps_grpx_ctrl *gctrl;
	GRPXDBG("GRPX DEINIT\n");
	while (!list_empty(&gctrl_list)) {
		gctrl = list_first_entry(&gctrl_list,
					 struct vps_grpx_ctrl, list);
		vps_grpx_remove_sysfs(gctrl);
		vps_grpx_params_dealloc(gctrl);
		list_del(&gctrl->list);
		kfree(gctrl);
	}
	num_gctrl = 0;
	grpx_dev = NULL;
}

