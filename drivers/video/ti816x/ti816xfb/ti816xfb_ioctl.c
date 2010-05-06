/*
 * linux/drivers/video/ti816x/ti816xfb/ti816xfb_ioctl.c
 *
 * Copyright (C) 2009 Texas Instruments
 * Author: Yihe Hu <yihehu@ti.com>
 *
 * Some codes and ideals are from TI OMAP2 by Tomi Valkeinen
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

#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/ti816xfb.h>

#include "fbpriv.h"

static void set_trans_key(struct fb_info *fbi,
			  struct ti816xfb_region_params *rp,
			  struct vps_grpxregionparams *regp)
{

	regp->transenable = 1;
	if (rp->transen == TI816XFB_FEATURE_DISABLE)
		regp->transenable = 0;

	if (rp->transen) {
		if (rp->transcolor > 0xFFFFFF)
			regp->transcolorrgb24 = 0xFFFFFF;
		else
			regp->transcolorrgb24 = rp->transcolor;

		regp->transtype = rp->transtype;
	}
}

static void set_boundbox(struct fb_info *fbi,
			 struct ti816xfb_region_params *rp,
			 struct vps_grpxregionparams *regp)
{

	if (rp->bben)
		regp->bbalpha = rp->bbalpha;

	regp->bbenable = 1;
	if (rp->bben == TI816XFB_FEATURE_DISABLE)
		regp->bbenable = 0;
}


static void set_blend(struct fb_info *fbi,
		      struct ti816xfb_region_params *rp,
		      struct vps_grpxregionparams *regp)
{

	if (rp->blendtype == TI816XFB_BLENDING_GLOBAL)
		regp->blendalpha = rp->blendalpha;

	regp->blendtype = (u32)rp->blendtype;

}

static int ti816xfb_set_region_params(struct fb_info *fbi,
				     struct ti816xfb_region_params *regparam)
{
	struct ti816xfb_info *tfbi = FB2TFB(fbi);
	struct ti816xfb_device *fbdev = tfbi->fbdev;
	struct vps_grpx_ctrl *gctrl = tfbi->gctrl;
	struct vps_grpxregionparams regp;
	struct fb_var_screeninfo *var = &fbi->var;
	int r = 0;

	TFBDBG("ti816xfb_set_regparams\n");


	if (regparam->priority >= TI816XFB_MAX_PRIORITY) {
		dev_err(fbdev->dev, "priority out of range");
		return -EINVAL;
	}
	ti816xfb_lock(tfbi);
	gctrl->get_regparams(gctrl, &regp);
	/* only update if it are not same*/


	regp.regionposx = regparam->pos_x;
	regp.regionposy = regparam->pos_y;
	regp.regionwidth = var->xres;
	regp.regionheight = var->yres;

	regp.disppriority = regparam->priority;
	regp.firstregion = 1;
	if (regparam->firstregion == TI816XFB_FEATURE_DISABLE)
		regp.firstregion = 0;

	regp.lastregion = 1;
	if (regparam->lastregion == TI816XFB_FEATURE_DISABLE)
		regp.lastregion = 0;

	regp.scenable = 1;
	if (regparam->scalaren == TI816XFB_FEATURE_DISABLE)
		regp.scenable = 0;

	regp.stencilingenable = 1;
	if (regparam->stencilingen == TI816XFB_FEATURE_DISABLE)
		regp.stencilingenable = 0;

	set_boundbox(fbi, regparam, &regp);
	set_blend(fbi, regparam, &regp);
	set_trans_key(fbi, regparam, &regp);

	r = gctrl->check_params(gctrl, &regp, regparam->ridx);

	if (r == 0)
		r = gctrl->set_regparams(gctrl, &regp);

	if (0 == r) {
		TFBDBG("set params handle %x\n", (u32)gctrl->handle);
		if (gctrl->gstate.isstarted)
			r = vps_fvid2_queue(gctrl->handle,
					    (struct fvid2_framelist *)
						gctrl->frmls_phy,
					    0);
		fbi->var.xres = regp.regionwidth;
		fbi->var.yres = regp.regionheight;
	}

	ti816xfb_unlock(tfbi);
	if (r)
		dev_err(fbdev->dev, "setup_plane failed %d\n", r);
	return r;
}

static int ti816xfb_get_region_params(struct fb_info *fbi,
				     struct ti816xfb_region_params *regparam)
{

	struct ti816xfb_info *tfbi = FB2TFB(fbi);
	struct vps_grpx_ctrl *gctrl = tfbi->gctrl;
	struct vps_grpxregionparams regp;
	int r = 0;

	ti816xfb_lock(tfbi);
	r = gctrl->get_regparams(gctrl, &regp);

	if (0 == r) {
		regparam->ridx = 0;
		regparam->bbalpha = regp.bbalpha;

		regparam->bben = TI816XFB_FEATURE_DISABLE;
		if (regp.bbenable == 1)
			regparam->bben = TI816XFB_FEATURE_ENABLE;
		regparam->blendalpha = regp.blendalpha;
		regparam->blendtype = regp.blendtype;
		regparam->pos_x = regp.regionposx;
		regparam->pos_y = regp.regionposy;
		regparam->priority = regp.disppriority;

		regparam->firstregion = TI816XFB_FEATURE_DISABLE;
		if (regp.firstregion == 1)
			regparam->firstregion = TI816XFB_FEATURE_ENABLE;

		regparam->lastregion = TI816XFB_FEATURE_DISABLE;
		if (regp.lastregion == 1)
			regparam->lastregion = TI816XFB_FEATURE_ENABLE;

		regparam->scalaren = TI816XFB_FEATURE_DISABLE;
		if (regp.scenable == 1)
			regparam->scalaren = TI816XFB_FEATURE_ENABLE;

		regparam->stencilingen = TI816XFB_FEATURE_DISABLE;
		if (regp.stencilingenable == 1)
			regparam->stencilingen = TI816XFB_FEATURE_ENABLE;

		regparam->transcolor = regp.transcolorrgb24;
		regparam->transtype = regp.transtype;
		regparam->transen = TI816XFB_FEATURE_DISABLE;
		if (regp.transenable == 1)
			regparam->transen = TI816XFB_FEATURE_ENABLE;
	}

    ti816xfb_unlock(tfbi);
	return r;
}

static int ti816xfb_set_scparams(struct fb_info *fbi,
				struct ti816xfb_region_scparams *scp)
{
	int r = 0;
	struct ti816xfb_info *tfbi = FB2TFB(fbi);
	struct vps_grpx_ctrl *gctrl = tfbi->gctrl;
	struct vps_grpxscparams  gscp;
	ti816xfb_lock(tfbi);
	/* check the scparamter first */

	gscp.horfineoffset = 0;
	gscp.verfineoffset = 0;
	gscp.sccoeff = NULL;
	gscp.inwidth = scp->inwidth;
	gscp.inheight = scp->inheight;
	gscp.outwidth = scp->outwidth;
	gscp.outheight = scp->outheight;
	r = gctrl->set_scparams(gctrl, &gscp);
	if (0 == r) {
		if (gctrl->gstate.isstarted)
			r = vps_fvid2_queue(gctrl->handle,
					    (struct fvid2_framelist *)
						gctrl->frmls_phy,
					    0);
	}
	ti816xfb_unlock(tfbi);

	return r;
}

static int ti816xfb_get_scparams(struct fb_info *fbi,
				struct ti816xfb_region_scparams *scp)
{
	int r = 0;
	struct ti816xfb_info *tfbi = FB2TFB(fbi);
	struct vps_grpx_ctrl *gctrl = tfbi->gctrl;
	struct vps_grpxscparams gscp;
	ti816xfb_lock(tfbi);
	r = gctrl->get_scparams(gctrl, &gscp);
	if (r == 0) {
		scp->inwidth = gscp.inwidth;
		scp->inheight = gscp.inheight;
		scp->outwidth = gscp.outwidth;
		scp->outheight = gscp.outheight;
	}
	ti816xfb_unlock(tfbi);

	return r;

}
static void ti816xfb_vsync_callback(void *arg)
{
	struct fb_info *fbi = (struct  fb_info *)arg;
	struct ti816xfb_info *tfbi = FB2TFB(fbi);

	++tfbi->vsync_cnt;

	wake_up_interruptible(&tfbi->vsync_wait);

}
static int ti816xfb_wait_for_vsync(struct fb_info *fbi)
{
	wait_queue_t  wqt;
	struct ti816xfb_info *tfbi = FB2TFB(fbi);
	struct vps_grpx_ctrl *gctrl = tfbi->gctrl;
	int r = 0;
	u32 cnt = 0;
	unsigned long tout = msecs_to_jiffies(500);

	r = gctrl->register_vsync_cb(gctrl,
				     ti816xfb_vsync_callback,
				     (void *)fbi);

	init_waitqueue_entry(&wqt, current);
	cnt = tfbi->vsync_cnt;

	/* split between simulator and silicon*/
	#ifndef CONFIG_TI8168_SIM
	r = wait_event_interruptible_timeout(tfbi->vsync_wait,
					     (cnt != tfbi->vsync_cnt),
					     tout);
	gctrl->unregister_vsync_cb(gctrl);

	if (r == 0)
		r = -ETIMEDOUT;
	else if (r > 0)
		r = 0;

	#else
	r = wait_event_interruptible(tfbi->vsync_wait,
				     (cnt != tfbi->vsync_cnt));
	gctrl->unregister_vsync_cb(gctrl);

	#endif


	return r;
}
static int ti816xfb_allocate_sten_mem(struct fb_info *fbi,
				     u32 size,
				     dma_addr_t *paddr)
{
	struct ti816xfb_alloc_list *mlist;
	struct ti816xfb_info *tfbi = FB2TFB(fbi);
	mlist = kzalloc(sizeof(*mlist), GFP_KERNEL);

	if (mlist == NULL) {
		*paddr = 0;
		return -ENOMEM;
	}
	ti816xfb_lock(tfbi);
	/* allocate the buffer*/
	mlist->size = PAGE_ALIGN(size);
	mlist->cpu_addr = dma_alloc_writecombine(tfbi->fbdev->dev,
						 size,
						 &mlist->phy_addr,
						 GFP_DMA);

	if (mlist->cpu_addr == NULL) {
		kfree(mlist);
		*paddr = 0;
		ti816xfb_unlock(tfbi);
		return -ENOMEM;
	}
	/* add into the list*/
	list_add(&mlist->list, &tfbi->alloc_list);
	TFBDBG("Sten allocated %d bytes @ 0x%08X\n",
			mlist->size, mlist->phy_addr);

	*paddr = mlist->phy_addr;
	ti816xfb_unlock(tfbi);
	return 0;

}

static int ti816xfb_free_sten_mem(struct fb_info *fbi, int offset)
{
	struct ti816xfb_info *tfbi = FB2TFB(fbi);
	struct vps_grpx_ctrl *gctrl = tfbi->gctrl;
	struct ti816xfb_alloc_list *mlist;
	u32 stenaddr;
	u32 stride;
	int r = -EINVAL;

	/* check to make sure that the  sten buffer to
	free is not the one being used*/
	ti816xfb_lock(tfbi);
	r = gctrl->get_stenparams(gctrl, &stenaddr, &stride);
	if ((r == 0) && (stenaddr != 0)) {
		struct vps_grpxregionparams regp;

		r = gctrl->get_regparams(gctrl, &regp);
		if (r == 0)
			if ((stenaddr == offset) &&
				(regp.stencilingenable == 1))
				return -EINVAL;
	}
	r = -EINVAL;
	/* loop the list to find out the offset and free it*/
	list_for_each_entry(mlist, &tfbi->alloc_list, list) {
		if (mlist->phy_addr == offset) {
			dma_free_writecombine(tfbi->fbdev->dev,
					      mlist->size,
					      mlist->cpu_addr,
					      mlist->phy_addr);
			kfree(mlist);
			r = 0;
			break;
		}
	}

	/* this is the last member in the list, remove the sten ptr to NULL */
	if (!list_empty(&tfbi->alloc_list)) {
		if (stenaddr != 0)
			gctrl->set_stenparams(gctrl, 0, stride);

	}
    ti816xfb_unlock(tfbi);
	return r;
}

static int ti816xfb_set_sten(struct fb_info *fbi,
			    struct ti816xfb_stenciling_params *stparams)
{
	struct ti816xfb_info *tfbi = FB2TFB(fbi);
	struct vps_grpx_ctrl *gctrl = tfbi->gctrl;
	struct ti816xfb_alloc_list *mlist;
	bool found = false;
	int r = 0;
	int offset = stparams->phy_addr;

	/* loop the list to find out the offset*/
	list_for_each_entry(mlist, &tfbi->alloc_list, list) {
		if (mlist->phy_addr == offset) {
			found = true;
			break;
		}
	}

	if (stparams->pitch & 0xF) {
		dev_err(tfbi->fbdev->dev,
			"stride should be 16 byte boundry.\n");
		return -EINVAL;
	}

    ti816xfb_lock(tfbi);
	r = gctrl->set_stenparams(gctrl, offset, stparams->pitch);
	if ((r == 0) && (gctrl->gstate.isstarted))
		vps_fvid2_queue(gctrl->handle,
				(struct fvid2_framelist *)gctrl->frmls_phy,
				0);
    ti816xfb_unlock(tfbi);
	return r;

}

static int ti816xfb_setup_mem(struct fb_info *fbi, struct ti816xfb_mem_info *mi)
{
	struct ti816xfb_info *tfbi = FB2TFB(fbi);
	struct ti816xfb_device *fbdev = tfbi->fbdev;
	struct ti816xfb_mem_region *rg;
	int r;
	size_t size;

	if (mi->type != TI816XFB_MEM_NONTILER)
		return -EINVAL;

	size = PAGE_ALIGN(mi->size);

	rg = &tfbi->mreg;
	ti816xfb_lock(tfbi);

	/*make sure that the Fb is not used */
	if (tfbi->gctrl->gstate.isstarted)
		return -EBUSY;

	if (rg->size != size || tfbi->mmode != mi->type) {
		r = ti816xfb_realloc_fbmem(fbi, size);
		if (r) {
			dev_err(fbdev->dev, "realloc fbmem failed\n");
			goto out;
		}
	}

	r = 0;
out:
	ti816xfb_unlock(tfbi);

	return r;
}


static int ti816xfb_query_mem(struct fb_info *fbi, struct ti816xfb_mem_info *mi)
{
	struct ti816xfb_info *tfbi = FB2TFB(fbi);
	struct ti816xfb_mem_region *rg;

	rg = &tfbi->mreg;
	memset(mi, 0, sizeof(*mi));

	ti816xfb_lock(tfbi);
	mi->size = rg->size;
	mi->type = tfbi->mmode;
	ti816xfb_unlock(tfbi);

	return 0;
}


int ti816xfb_ioctl(struct fb_info *fbi, unsigned int cmd,
		  unsigned long arg)
{
	union {
		struct ti816xfb_region_params regparams;
		struct ti816xfb_region_scparams scparams;
		struct ti816xfb_mem_info   minfo;
		struct ti816xfb_stenciling_params stparams;
		int mirror;
		int size;
		int offset;
	} param;
	struct ti816xfb_info *tfbi = FB2TFB(fbi);
	int r = 0;

	BUG_ON(!fbi->fbops);
	BUG_ON(!tfbi->mreg.size);

	switch (cmd) {

	case FBIO_WAITFORVSYNC:
		TFBDBG("ioctl WAITFORVSYNC\n");
		return ti816xfb_wait_for_vsync(fbi);
		break;

	case TI816XFB_SET_PARAMS:
		TFBDBG("ioctl SET_PARAMS\n");

		if (copy_from_user(&param.regparams, (void __user *)arg,
				   sizeof(struct ti816xfb_region_params)))
			r = -EFAULT;
		else
			r = ti816xfb_set_region_params(fbi, &param.regparams);

		break;

	case TI816XFB_GET_PARAMS:
		TFBDBG("ioctl GET_PARAMS\n");
		r = ti816xfb_get_region_params(fbi, &param.regparams);

		if (r < 0)
			break;

		if (copy_to_user((void __user *)arg, &param.regparams,
				 sizeof(param.regparams)))
			r = -EFAULT;
		break;

	case TI816XFB_SET_COEFF:
		TFBDBG("ioctl SET_COEFF\n");
		if (copy_from_user(&param.scparams, (void __user *)arg,
				   sizeof(param.scparams))) {
			r = -EFAULT;
			break;
		}

		r = ti816xfb_set_scparams(fbi, &param.scparams);

		break;

	case  TI816XFB_GET_COEFF:
		TFBDBG("ioctl GET_COEFF");
		r = ti816xfb_get_scparams(fbi, &param.scparams);
		if (r == 0) {
			if (copy_to_user((void __user *)arg, &param.scparams,
					 sizeof(param.scparams)))
				r = -EFAULT;

		}

		break;

	case TI816XFB_ALLOC_STENC:
	{
		dma_addr_t paddr;
		TFBDBG("ioctl ALLOC_STEN\n");

		if (get_user(param.size, (int __user *)arg)) {
			r = -EFAULT;
			break;
		}

		r = ti816xfb_allocate_sten_mem(fbi, param.size, &paddr);
		if ((r == 0) && (paddr != 0)) {
			if (put_user(paddr, (int __user *)arg))
				r = -EFAULT;
		}
		break;
	}

	case TI816XFB_FREE_STENC:
		TFBDBG("ioctl FREE_STENC\n");
		if (get_user(param.offset, (int __user *)arg)) {
			r = -EFAULT;
			break;
		}

		r = ti816xfb_free_sten_mem(fbi, param.offset);
		break;

	case TI816XFB_SETUP_MEM:
		if (copy_from_user(&param.minfo, (void __user *)arg,
					sizeof(param.minfo)))
			r = -EFAULT;
		else
			r = ti816xfb_setup_mem(fbi, &param.minfo);
		break;


	case TI816XFB_QUERY_MEM:
		TFBDBG("ioctl QUERY_MEM\n");
		r = ti816xfb_query_mem(fbi, &param.minfo);
		if (r < 0)
			break;
		if (copy_to_user((void __user *)arg, &param.minfo,
					sizeof(param.minfo)))
			r = -EFAULT;

		break;
	case TI816XFB_SET_STENC:
		TFBDBG("ioctl SET_STEN.\n");
		if (copy_from_user(&param.stparams, (int __user *)arg,
				   sizeof(param.stparams))) {
			r = -EFAULT;
			break;
		}
		r = ti816xfb_set_sten(fbi, &param.stparams);
		break;
	default:
		dev_err(FB2TFB(fbi)->fbdev->dev, "Unknown ioctl 0x%x\n", cmd);
		r = -EFAULT;
		break;
	}

	if (r < 0)
		TFBDBG("ioctl 0x%x failed: %d\n", cmd , r);

	return r;
}



