/*
 * linux/drivers/video/omap2/omapfb-ioctl.c
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

#include <linux/fb.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/mm.h>

#include <mach/display.h>
#include <mach/omapfb.h>

#include "omapfb.h"

static int omapfb_setup_plane(struct fb_info *fbi, struct omapfb_plane_info *pi)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omapfb2_device *fbdev = ofbi->fbdev;
	struct omap_display *display = fb2display(fbi);
	struct omap_overlay *ovl;
	int r = 0;

	DBG("omapfb_setup_plane\n");

	omapfb_lock(fbdev);

	if (ofbi->num_overlays != 1) {
		r = -EINVAL;
		goto out;
	}

	/* XXX uses only the first overlay */
	ovl = ofbi->overlays[0];

	if (pi->enabled && !ofbi->region.size) {
		/*
		 * This plane's memory was freed, can't enable it
		 * until it's reallocated.
		 */
		r = -EINVAL;
		goto out;
	}

	if (pi->enabled) {
		r = omapfb_setup_overlay(fbi, ovl, pi->pos_x, pi->pos_y,
				pi->out_width, pi->out_height);
		if (r)
			goto out;
	}

	ovl->enable(ovl, pi->enabled);

	if (ovl->manager)
		ovl->manager->apply(ovl->manager);

	if (display) {
		int w, h;

		if (display->sync)
			display->sync(display);

		display->get_resolution(display, &w, &h);

		if (display->update)
			display->update(display, 0, 0, w, h);
	}

out:
	omapfb_unlock(fbdev);
	if (r)
		dev_err(fbdev->dev, "setup_plane failed\n");
	return r;
}

static int omapfb_query_plane(struct fb_info *fbi, struct omapfb_plane_info *pi)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omapfb2_device *fbdev = ofbi->fbdev;

	omapfb_lock(fbdev);

	if (ofbi->num_overlays != 1) {
		memset(pi, 0, sizeof(*pi));
	} else {
		struct omap_overlay_info *ovli;
		struct omap_overlay *ovl;

		ovl = ofbi->overlays[0];
		ovli = &ovl->info;

		pi->pos_x = ovli->pos_x;
		pi->pos_y = ovli->pos_y;
		pi->enabled = ovli->enabled;
		pi->channel_out = 0; /* xxx */
		pi->mirror = 0;
		pi->out_width = ovli->out_width;
		pi->out_height = ovli->out_height;
	}

	omapfb_unlock(fbdev);

	return 0;
}

static int omapfb_setup_mem(struct fb_info *fbi, struct omapfb_mem_info *mi)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omapfb2_device *fbdev = ofbi->fbdev;
	struct omapfb2_mem_region *rg;
	struct omap_display *display = fb2display(fbi);
	int r, i;
	size_t size;

	if (mi->type > OMAPFB_MEMTYPE_MAX)
		return -EINVAL;

	size = PAGE_ALIGN(mi->size);

	rg = &ofbi->region;

	omapfb_lock(fbdev);

	for (i = 0; i < ofbi->num_overlays; i++) {
		if (ofbi->overlays[i]->info.enabled) {
			r = -EBUSY;
			goto out;
		}
	}

	if (rg->size != size || rg->type != mi->type) {
		struct fb_var_screeninfo new_var;
		unsigned long old_size = rg->size;

		if (display->sync)
			display->sync(display);

		r = omapfb_realloc_fbmem(fbdev, ofbi->id, size);
		if (r)
			goto out;

		if (old_size != size) {
			if (size) {
				memcpy(&new_var, &fbi->var, sizeof(new_var));
				r = check_fb_var(fbi, &new_var);
				if (r < 0)
					goto out;
				memcpy(&fbi->var, &new_var, sizeof(fbi->var));
				set_fb_fix(fbi);
			} else {
				/*
				 * Set these explicitly to indicate that the
				 * plane memory is dealloce'd, the other
				 * screen parameters in var / fix are invalid.
				 */
				fbi->fix.smem_start = 0;
				fbi->fix.smem_len = 0;
			}
		}
	}

	r = 0;
out:
	omapfb_unlock(fbdev);

	return r;
}

static int omapfb_query_mem(struct fb_info *fbi, struct omapfb_mem_info *mi)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omapfb2_device *fbdev = ofbi->fbdev;
	struct omapfb2_mem_region *rg;

	rg = &ofbi->region;
	memset(mi, 0, sizeof(*mi));

	omapfb_lock(fbdev);
	mi->size = rg->size;
	mi->type = rg->type;
	omapfb_unlock(fbdev);

	return 0;
}

static int omapfb_update_window(struct fb_info *fbi,
		u32 x, u32 y, u32 w, u32 h)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omapfb2_device *fbdev = ofbi->fbdev;
	struct omap_display *display = fb2display(fbi);
	int dw, dh;

	if (!display)
		return 0;

	if (w == 0 || h == 0)
		return 0;

	display->get_resolution(display, &dw, &dh);

	if (x + w > dw || y + h > dh)
		return -EINVAL;

	omapfb_lock(fbdev);
	display->update(display, x, y, w, h);
	omapfb_unlock(fbdev);

	return 0;
}

static int omapfb_set_update_mode(struct fb_info *fbi,
				   enum omapfb_update_mode mode)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omapfb2_device *fbdev = ofbi->fbdev;
	struct omap_display *display = fb2display(fbi);
	enum omap_dss_update_mode um;
	int r;

	if (!display || !display->set_update_mode)
		return -EINVAL;

	switch (mode) {
	case OMAPFB_UPDATE_DISABLED:
		um = OMAP_DSS_UPDATE_DISABLED;
		break;

	case OMAPFB_AUTO_UPDATE:
		um = OMAP_DSS_UPDATE_AUTO;
		break;

	case OMAPFB_MANUAL_UPDATE:
		um = OMAP_DSS_UPDATE_MANUAL;
		break;

	default:
		return -EINVAL;
	}

	omapfb_lock(fbdev);
	r = display->set_update_mode(display, um);
	omapfb_unlock(fbdev);

	return r;
}

static int omapfb_get_update_mode(struct fb_info *fbi,
		enum omapfb_update_mode *mode)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omapfb2_device *fbdev = ofbi->fbdev;
	struct omap_display *display = fb2display(fbi);
	enum omap_dss_update_mode m;

	if (!display || !display->get_update_mode)
		return -EINVAL;

	omapfb_lock(fbdev);
	m = display->get_update_mode(display);
	omapfb_unlock(fbdev);

	switch (m) {
	case OMAP_DSS_UPDATE_DISABLED:
		*mode = OMAPFB_UPDATE_DISABLED;
		break;
	case OMAP_DSS_UPDATE_AUTO:
		*mode = OMAPFB_AUTO_UPDATE;
		break;
	case OMAP_DSS_UPDATE_MANUAL:
		*mode = OMAPFB_MANUAL_UPDATE;
		break;
	default:
		BUG();
	}

	return 0;
}

/* Interrupt service routine. */
static void omapfb_isr(void *arg, unsigned int irqstatus)
{
	struct fb_info *fbi = (struct fb_info *) arg;
	struct omapfb_info *ofbi = FB2OFB(fbi);

	++ofbi->vsync_cnt;
	wake_up_interruptible(&ofbi->vsync_wait);
}

static int omapfb_wait_for_vsync(struct fb_info *fbi)
{
	wait_queue_t wqt;
	unsigned long cnt, timeout = HZ/5;
	int ret;
	void *handle = NULL;
	u32 mask = 0;
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omap_display *display = fb2display(fbi);

	mask = DISPC_IRQ_VSYNC | DISPC_IRQ_EVSYNC_EVEN |
			DISPC_IRQ_EVSYNC_ODD;

	handle = omap_dispc_register_isr(omapfb_isr, fbi, mask);
	if (!handle)
		return -EINVAL;

	init_waitqueue_entry(&wqt, current);

	cnt = ofbi->vsync_cnt;
	ret = wait_event_interruptible_timeout(ofbi->vsync_wait,
			cnt != ofbi->vsync_cnt, timeout);
	/*
	 * If the GFX is on TV, then wait for another VSYNC
	 * to compensate for Interlaced scan
	 */
	if (display->type == OMAP_DISPLAY_TYPE_VENC) {
		if (ret > 0) {
			cnt = ofbi->vsync_cnt;
			ret = wait_event_interruptible_timeout(
					ofbi->vsync_wait,
					cnt != ofbi->vsync_cnt,
					timeout);
		}
	}
	omap_dispc_unregister_isr(handle);

	if (ret < 0)
		return ret;
	if (ret == 0)
		return -ETIMEDOUT;

	return 0;
}

int omapfb_ioctl(struct fb_info *fbi, unsigned int cmd, unsigned long arg)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omapfb2_device *fbdev = ofbi->fbdev;
	struct omap_display *display = fb2display(fbi);

	union {
		struct omapfb_update_window_old	uwnd_o;
		struct omapfb_update_window	uwnd;
		struct omapfb_plane_info	plane_info;
		struct omapfb_caps		caps;
		struct omapfb_mem_info          mem_info;
		enum omapfb_update_mode		update_mode;
		int test_num;
	} p;

	int r = 0;

	DBG("ioctl %x (%d)\n", cmd, cmd & 0xff);

	switch (cmd) {
	case OMAPFB_SYNC_GFX:
		if (!display || !display->sync) {
			/* DSS1 never returns an error here, so we neither */
			/*r = -EINVAL;*/
			break;
		}

		omapfb_lock(fbdev);
		r = display->sync(display);
		omapfb_unlock(fbdev);
		break;

	case OMAPFB_UPDATE_WINDOW_OLD:
		if (!display || !display->update) {
			r = -EINVAL;
			break;
		}

		if (copy_from_user(&p.uwnd_o,
					(void __user *)arg,
					sizeof(p.uwnd_o))) {
			r = -EFAULT;
			break;
		}

		r = omapfb_update_window(fbi, p.uwnd_o.x, p.uwnd_o.y,
				p.uwnd_o.width, p.uwnd_o.height);
		break;

	case OMAPFB_UPDATE_WINDOW:
		if (!display || !display->update) {
			r = -EINVAL;
			break;
		}

		if (copy_from_user(&p.uwnd, (void __user *)arg,
					sizeof(p.uwnd))) {
			r = -EFAULT;
			break;
		}

		r = omapfb_update_window(fbi, p.uwnd.x, p.uwnd.y,
				p.uwnd.width, p.uwnd.height);
		break;

	case OMAPFB_SETUP_PLANE:
		if (copy_from_user(&p.plane_info, (void __user *)arg,
					sizeof(p.plane_info)))
			r = -EFAULT;
		else
			r = omapfb_setup_plane(fbi, &p.plane_info);
		break;

	case OMAPFB_QUERY_PLANE:
		r = omapfb_query_plane(fbi, &p.plane_info);
		if (r < 0)
			break;
		if (copy_to_user((void __user *)arg, &p.plane_info,
					sizeof(p.plane_info)))
			r = -EFAULT;
		break;

	case OMAPFB_SETUP_MEM:
		if (copy_from_user(&p.mem_info, (void __user *)arg,
					sizeof(p.mem_info)))
			r = -EFAULT;
		else
			r = omapfb_setup_mem(fbi, &p.mem_info);
		break;

	case OMAPFB_QUERY_MEM:
		r = omapfb_query_mem(fbi, &p.mem_info);
		if (r < 0)
			break;
		if (copy_to_user((void __user *)arg, &p.mem_info,
					sizeof(p.mem_info)))
			r = -EFAULT;
		break;

	case OMAPFB_GET_CAPS:
		if (!display) {
			r = -EINVAL;
			break;
		}

		p.caps.ctrl = display->caps;

		if (copy_to_user((void __user *)arg, &p.caps, sizeof(p.caps)))
			r = -EFAULT;
		break;

	case OMAPFB_SET_UPDATE_MODE:
		if (get_user(p.update_mode, (int __user *)arg))
			r = -EFAULT;
		else
			r = omapfb_set_update_mode(fbi, p.update_mode);
		break;

	case OMAPFB_GET_UPDATE_MODE:
		r = omapfb_get_update_mode(fbi, &p.update_mode);
		if (r)
			break;
		if (put_user(p.update_mode,
					(enum omapfb_update_mode __user *)arg))
			r = -EFAULT;
		break;

	/* LCD and CTRL tests do the same thing for backward
	 * compatibility */
	case OMAPFB_LCD_TEST:
		if (get_user(p.test_num, (int __user *)arg)) {
			r = -EFAULT;
			break;
		}
		if (!display || !display->run_test) {
			r = -EINVAL;
			break;
		}

		r = display->run_test(display, p.test_num);

		break;

	case OMAPFB_CTRL_TEST:
		if (get_user(p.test_num, (int __user *)arg)) {
			r = -EFAULT;
			break;
		}
		if (!display || !display->run_test) {
			r = -EINVAL;
			break;
		}

		r = display->run_test(display, p.test_num);

		break;
	case OMAPFB_WAIT_FOR_VSYNC:
		return omapfb_wait_for_vsync(fbi);

	default:
		DBG("ioctl unhandled\n");
		r = -EINVAL;
	}

	return r;
}


