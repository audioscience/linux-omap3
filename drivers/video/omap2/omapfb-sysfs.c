/*
 * linux/drivers/video/omap2/omapfb-sysfs.c
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
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>

#include <mach/pm.h>
#include <mach/display.h>
#include <mach/omapfb.h>

#include "omapfb.h"

static int omapfb_attach_framebuffer(struct fb_info *fbi,
		struct omap_overlay *ovl)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omapfb2_device *fbdev = ofbi->fbdev;
	int i, t;
	int r;

	if (ofbi->num_overlays >= OMAPFB_MAX_OVL_PER_FB) {
		dev_err(fbdev->dev, "fb has max number of overlays already\n");
		return -EINVAL;
	}

	for (i = 0; i < ofbi->num_overlays; i++) {
		if (ofbi->overlays[i] == ovl) {
			dev_err(fbdev->dev, "fb already attached to overlay\n");
			return -EINVAL;
		}
	}

	for (i = 0; i < fbdev->num_fbs; i++) {
		struct omapfb_info *ofbi2 = FB2OFB(fbdev->fbs[i]);
		for (t = 0; t < ofbi2->num_overlays; t++) {
			if (ofbi2->overlays[t] == ovl) {
				dev_err(fbdev->dev, "overlay already in use\n");
				return -EINVAL;
			}
		}
	}

	ofbi->overlays[ofbi->num_overlays++] = ovl;

/*
	if (ovl->manager && ovl->manager->display)
		omapfb_adjust_fb(fbi, ovl, 0, 0);
*/
	r = omapfb_apply_changes(fbi, 1);
	if (r)
		return r;

	if (ovl->manager)
		ovl->manager->apply(ovl->manager);

	return 0;
}

static int omapfb_detach_framebuffer(struct fb_info *fbi,
		struct omap_overlay *ovl)
{
	int i;
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omapfb2_device *fbdev = ofbi->fbdev;

	for (i = 0; i < ofbi->num_overlays; i++) {
		if (ofbi->overlays[i] == ovl)
			break;
	}

	if (i == ofbi->num_overlays) {
		dev_err(fbdev->dev, "cannot detach fb, overlay not attached\n");
		return -EINVAL;
	}

	ovl->enable(ovl, 0);

	if (ovl->manager)
		ovl->manager->apply(ovl->manager);

	for (i = i + 1; i < ofbi->num_overlays; i++)
		ofbi->overlays[i-1] = ofbi->overlays[i];

	ofbi->num_overlays--;

	return 0;
}

static ssize_t show_framebuffers(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omapfb2_device *fbdev = platform_get_drvdata(pdev);
	ssize_t l = 0, size = PAGE_SIZE;
	int i, t;

	omapfb_lock(fbdev);

	for (i = 0; i < fbdev->num_fbs; i++) {
		struct omapfb_info *ofbi = FB2OFB(fbdev->fbs[i]);
		struct omapfb2_mem_region *rg;

		rg = &ofbi->region;

		l += snprintf(buf + l, size - l, "%d p:%08x v:%p size:%lu t:",
				ofbi->id,
				rg->_paddr, rg->_vaddr, rg->size);

		if (ofbi->num_overlays == 0)
			l += snprintf(buf + l, size - l, "none");

		for (t = 0; t < ofbi->num_overlays; t++) {
			struct omap_overlay *ovl;
			ovl = ofbi->overlays[t];

			l += snprintf(buf + l, size - l, "%s%s",
					t == 0 ? "" : ",",
					ovl->name);
		}

		l += snprintf(buf + l, size - l, "\n");
	}

	omapfb_unlock(fbdev);

	return l;
}

static struct omap_overlay *find_overlay_by_name(struct omapfb2_device *fbdev,
		char *name)
{
	int i;

	for (i = 0; i < fbdev->num_overlays; i++)
		if (strcmp(name, fbdev->overlays[i]->name) == 0)
			return fbdev->overlays[i];

	return NULL;
}

static struct omap_display *find_display_by_name(struct omapfb2_device *fbdev,
		char *name)
{
	int i;

	for (i = 0; i < fbdev->num_displays; i++)
		if (strcmp(name, fbdev->displays[i]->name) == 0)
			return fbdev->displays[i];

	return NULL;
}

static struct omap_overlay_manager *find_manager_by_name(
		struct omapfb2_device *fbdev,
		char *name)
{
	int i;

	for (i = 0; i < fbdev->num_managers; i++)
		if (strcmp(name, fbdev->managers[i]->name) == 0)
			return fbdev->managers[i];

	return NULL;
}

static int parse_overlays(struct omapfb2_device *fbdev, char *str,
		struct omap_overlay *ovls[])
{
	int num_ovls = 0;
	int s, e = 0;
	char ovlname[10];

	while (1) {
		struct omap_overlay *ovl;

		s = e;

		while (e < strlen(str) && str[e] != ',')
			e++;

		strncpy(ovlname, str + s, e - s);
		ovlname[e-s] = 0;

		DBG("searching for '%s'\n", ovlname);
		ovl = find_overlay_by_name(fbdev, ovlname);

		if (ovl) {
			DBG("found an overlay\n");
			ovls[num_ovls] = ovl;
			num_ovls++;
		} else {
			DBG("unknown overlay %s\n", str);
			return 0;
		}

		if (e == strlen(str))
			break;

		e++;
	}

	return num_ovls;
}

static ssize_t store_framebuffers(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omapfb2_device *fbdev = platform_get_drvdata(pdev);
	int idx;
	char fbname[3];
	unsigned long fbnum;
	char ovlnames[40];
	int num_ovls = 0;
	struct omap_overlay *ovls[OMAPFB_MAX_OVL_PER_FB];
	struct fb_info *fbi;
	struct omapfb_info *ofbi;
	int r, i;

	idx = 0;
	while (idx < count && buf[idx] != ' ')
		++idx;

	if (idx == count)
		return -EINVAL;

	if (idx >= sizeof(fbname))
		return -EINVAL;

	strncpy(fbname, buf, idx);
	fbname[idx] = 0;
	idx++;

	if (strict_strtoul(fbname, 10, &fbnum))
		return -EINVAL;

	r = sscanf(buf + idx, "t:%39s", ovlnames);

	if (r != 1) {
		r = -EINVAL;
		goto err;
	}

	omapfb_lock(fbdev);

	if (fbnum >= fbdev->num_fbs) {
		dev_err(dev, "fb not found\n");
		r = -EINVAL;
		goto err;
	}

	fbi = fbdev->fbs[fbnum];
	ofbi = FB2OFB(fbi);

	if (strcmp(ovlnames, "none") == 0) {
		num_ovls = 0;
	} else {
		num_ovls = parse_overlays(fbdev, ovlnames, ovls);

		if (num_ovls == 0) {
			dev_err(dev, "overlays not found\n");
			r = -EINVAL;
			goto err;
		}
	}

	for (i = 0; i < ofbi->num_overlays; i++) {
		r = omapfb_detach_framebuffer(fbi, ofbi->overlays[i]);
		if (r) {
			dev_err(dev, "detach failed\n");
			goto err;
		}
	}

	if (num_ovls > 0) {
		for (i = 0; i < num_ovls; i++) {
			r = omapfb_attach_framebuffer(fbi, ovls[i]);
			if (r) {
				dev_err(dev, "attach failed\n");
				goto err;
			}
		}
	}

	omapfb_unlock(fbdev);
	return count;

err:
	omapfb_unlock(fbdev);
	return r;
}

static ssize_t show_overlays(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omapfb2_device *fbdev = platform_get_drvdata(pdev);
	ssize_t l = 0, size = PAGE_SIZE;
	int i, mgr_num;

	omapfb_lock(fbdev);

	for (i = 0; i < fbdev->num_overlays; i++) {
		struct omap_overlay *ovl;
		struct omap_overlay_manager *mgr;

		ovl = fbdev->overlays[i];
		mgr = ovl->manager;

		for (mgr_num = 0; mgr_num < fbdev->num_managers; mgr_num++)
			if (fbdev->managers[mgr_num] == mgr)
				break;

		l += snprintf(buf + l, size - l,
			"%s t:%s x:%d y:%d iw:%d ih:%d w:%d h:%d e:%d\n",
			ovl->name,
			mgr ? mgr->name : "none",
			ovl->info.pos_x,
			ovl->info.pos_y,
			ovl->info.width,
			ovl->info.height,
			ovl->info.out_width,
			ovl->info.out_height,
			ovl->info.enabled);
	}

	omapfb_unlock(fbdev);

	return l;
}

static ssize_t store_overlays(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omapfb2_device *fbdev = platform_get_drvdata(pdev);
	int idx;
	struct omap_overlay *ovl = NULL;
	struct omap_overlay_manager *mgr;
	int r;
	char ovlname[10];
	int posx, posy, outw, outh;
	int enabled;

	idx = 0;
	while (idx < count && buf[idx] != ' ')
		++idx;

	if (idx == count)
		return -EINVAL;

	if (idx >= sizeof(ovlname))
		return -EINVAL;

	strncpy(ovlname, buf, idx);
	ovlname[idx] = 0;
	idx++;

	omapfb_lock(fbdev);

	ovl = find_overlay_by_name(fbdev, ovlname);

	if (!ovl) {
		dev_err(dev, "ovl not found\n");
		r = -EINVAL;
		goto err;
	}

	DBG("ovl %s found\n", ovl->name);

	mgr = ovl->manager;

	posx = ovl->info.pos_x;
	posy = ovl->info.pos_y;
	outw = ovl->info.out_width;
	outh = ovl->info.out_height;
	enabled = ovl->info.enabled;

	while (idx < count) {
		char c;
		int val;
		int len;
		char sval[10];

		r = sscanf(buf + idx, "%c:%d%n", &c, &val, &len);

		if (r != 2) {
			val = 0;

			r = sscanf(buf + idx, "%c:%9s%n", &c, sval, &len);

			if (r != 2) {
				dev_err(dev, "sscanf failed, aborting\n");
				r = -EINVAL;
				goto err;
			}
		} else {
			sval[0] = 0;
		}

		switch (c) {
		case 't':
			if (strcmp(sval, "none") == 0) {
				mgr = NULL;
			} else {
				mgr = find_manager_by_name(fbdev, sval);

				if (mgr == NULL) {
					dev_err(dev, "no such manager\n");
					r = -EINVAL;
					goto err;
				}

				DBG("manager %s found\n", mgr->name);
			}

			break;

		case 'x':
			posx = val;
			break;

		case 'y':
			posy = val;
			break;

		case 'w':
			if (ovl->caps & OMAP_DSS_OVL_CAP_SCALE)
				outw = val;
			break;

		case 'h':
			if (ovl->caps & OMAP_DSS_OVL_CAP_SCALE)
				outh = val;
			break;

		case 'e':
			enabled = val;
			break;

		default:
			dev_err(dev, "unknown option %c\n", c);
			r = -EINVAL;
			goto err;
		}

		idx += len + 1;
	}

	r = ovl->setup_output(ovl, posx, posy, outw, outh);

	if (r) {
		dev_err(dev, "setup overlay failed\n");
		goto err;
	}

	if (mgr != ovl->manager) {
		/* detach old manager */
		if (ovl->manager) {
			r = ovl->unset_manager(ovl);
			if (r) {
				dev_err(dev, "detach failed\n");
				goto err;
			}
		}

		if (mgr) {
			r = ovl->set_manager(ovl, mgr);
			if (r) {
				dev_err(dev, "Failed to attach overlay\n");
				goto err;
			}
		}
	}

	r = ovl->enable(ovl, enabled);

	if (r) {
		dev_err(dev, "enable overlay failed\n");
		goto err;
	}

	if (mgr) {
		r = mgr->apply(mgr);
		if (r) {
			dev_err(dev, "failed to apply dispc config\n");
			goto err;
		}
	} else {
		ovl->enable(ovl, 0);
	}

	if (mgr && mgr->display && mgr->display->update) {
		int w, h;
		mgr->display->get_resolution(mgr->display, &w, &h);
		mgr->display->update(mgr->display, 0, 0, w, h);
	}

	omapfb_unlock(fbdev);
	return count;

err:
	omapfb_unlock(fbdev);
	return r;
}

static ssize_t show_managers(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omapfb2_device *fbdev = platform_get_drvdata(pdev);
	ssize_t l = 0, size = PAGE_SIZE;
	int i;

	omapfb_lock(fbdev);

	for (i = 0; i < fbdev->num_managers; i++) {
		struct omap_display *display;
		struct omap_overlay_manager *mgr;

		mgr = fbdev->managers[i];
		display = mgr->display;

		l += snprintf(buf + l, size - l, "%s t:%s\n",
				mgr->name, display ? display->name : "none");
	}

	omapfb_unlock(fbdev);

	return l;
}

static ssize_t store_managers(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omapfb2_device *fbdev = platform_get_drvdata(pdev);
	int idx;
	struct omap_overlay_manager *mgr;
	struct omap_display *display;
	char mgrname[10];
	char displayname[10];
	int r;

	idx = 0;
	while (idx < count && buf[idx] != ' ')
		++idx;

	if (idx == count)
		return -EINVAL;

	if (idx >= sizeof(mgrname))
		return -EINVAL;

	strncpy(mgrname, buf, idx);
	mgrname[idx] = 0;
	idx++;

	omapfb_lock(fbdev);

	mgr = find_manager_by_name(fbdev, mgrname);

	if (!mgr) {
		dev_err(dev, "manager not found\n");
		r = -EINVAL;
		goto err;
	}

	r = sscanf(buf + idx, "t:%9s", displayname);

	if (r != 1) {
		r = -EINVAL;
		goto err;
	}

	if (strcmp(displayname, "none") == 0) {
		display = NULL;
	} else {
		display = find_display_by_name(fbdev, displayname);

		if (!display) {
			dev_err(dev, "display not found\n");
			r = -EINVAL;
			goto err;
		}
	}

	if (mgr->display) {
		r = mgr->unset_display(mgr);
		if (r) {
			dev_err(dev, "failed to unset display\n");
			goto err;
		}
	}

	if (display) {
		r = mgr->set_display(mgr, display);
		if (r) {
			dev_err(dev, "failed to set manager\n");
			goto err;
		}

		r = mgr->apply(mgr);
		if (r) {
			dev_err(dev, "failed to apply dispc config\n");
			goto err;
		}
	}

	omapfb_unlock(fbdev);
	return count;

err:
	omapfb_unlock(fbdev);
	return r;
}

static ssize_t show_displays(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omapfb2_device *fbdev = platform_get_drvdata(pdev);
	ssize_t l = 0, size = PAGE_SIZE;
	int i;
	struct omap_video_timings timings;

	omapfb_lock(fbdev);

	for (i = 0; i < fbdev->num_displays; i++) {
		struct omap_display *display;
		enum omap_dss_update_mode mode = -1;
		int te = 0;
		int rot = 0, mir = 0;

		display = fbdev->displays[i];

		if (display->get_update_mode)
			mode = display->get_update_mode(display);

		if (display->get_te)
			te = display->get_te(display);

		if (display->get_timings)
			display->get_timings(display, &timings);
		else
			memset(&timings, 0, sizeof(timings));

		if (display->get_rotate)
			rot = display->get_rotate(display);

		if (display->get_mirror)
			mir = display->get_mirror(display);

		l += snprintf(buf + l, size - l,
				"%s e:%d u:%d t:%d h:%u/%u/%u/%u "
				"v:%u/%u/%u/%u p:%u r:%d i:%d\n",
				display->name,
				display->state != OMAP_DSS_DISPLAY_DISABLED,
				mode, te,
				timings.x_res,
				timings.hfp, timings.hbp, timings.hsw,
				timings.y_res,
				timings.vfp, timings.vbp, timings.vsw,
				timings.pixel_clock,
				rot, mir);
	}

	omapfb_unlock(fbdev);

	return l;
}

static ssize_t store_displays(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omapfb2_device *fbdev = platform_get_drvdata(pdev);
	int enable;
	struct omap_video_timings old_timings;
	struct omap_video_timings new_timings;
	enum omap_dss_update_mode mode;
	struct omap_display *display = NULL;
	int r;
	int te, rot, mir;
	char str[128];
	char *s, *tok;

	if (strlen(buf) > sizeof(str) - 1)
		return -EINVAL;

	strcpy(str, buf);

	/* remove trailing linefeeds */
	s = str + strlen(str) - 1;
	while (s >= str	&& *s == '\n') {
		*s = 0;
		s--;
	}

	s = str;

	if ((tok = strsep(&s, " ")) == 0)
		return -EINVAL;

	omapfb_lock(fbdev);

	display = find_display_by_name(fbdev, tok);

	if (!display) {
		dev_err(dev, "display not found\n");
		r = -EINVAL;
		goto err;
	}

	enable = display->state != OMAP_DSS_DISPLAY_DISABLED;
	if (display->get_update_mode)
		mode = display->get_update_mode(display);
	else
		mode = 0;

	if (display->get_te)
		te = display->get_te(display);
	else
		te = 0;

	if (display->get_rotate)
		rot = display->get_rotate(display);
	else
		rot = 0;

	if (display->get_mirror)
		mir = display->get_mirror(display);
	else
		mir = 0;

	if (display->get_timings)
		display->get_timings(display, &old_timings);
	else
		memset(&old_timings, 0, sizeof(old_timings));

	memcpy(&new_timings, &old_timings, sizeof(new_timings));

	while ((tok = strsep(&s, " "))) {
		char c, *o;

		if (strlen(tok) < 3 || tok[1] != ':') {
			dev_err(dev, "illegal option\n");
			r = -EINVAL;
			goto err;
		}

		c = tok[0];
		o = tok + 2;

		switch (c) {
		case 'e':
			enable = simple_strtoul(o, NULL, 0);
			break;

		case 'u':
			mode = simple_strtoul(o, NULL, 0);
			break;

		case 't':
			te = simple_strtoul(o, NULL, 0);
			break;

		case 'r':
			rot = simple_strtoul(o, NULL, 0);
			break;

		case 'i':
			mir = simple_strtoul(o, NULL, 0);
			break;

		case 'm': {
			unsigned bpp;
			if (omapfb_mode_to_timings(o, &new_timings, &bpp) != 0)
				memset(&new_timings, 0, sizeof(new_timings));

			break;
		}

		case 'h': {
			unsigned xres, hfp, hbp, hsw;

			if (sscanf(o, "%u/%u/%u/%u",
						&xres, &hfp, &hbp, &hsw) != 4) {
				dev_err(dev, "illegal horizontal timings\n");
				r = -EINVAL;
				goto err;
			}

			new_timings.x_res = xres;
			new_timings.hfp = hfp;
			new_timings.hbp = hbp;
			new_timings.hsw = hsw;
			break;
		}

		case 'v': {
			unsigned yres, vfp, vbp, vsw;

			if (sscanf(o, "%u/%u/%u/%u",
						&yres, &vfp, &vbp, &vsw) != 4) {
				dev_err(dev, "illegal vertical timings\n");
				r = -EINVAL;
				goto err;
			}

			new_timings.y_res = yres;
			new_timings.vfp = vfp;
			new_timings.vbp = vbp;
			new_timings.vsw = vsw;
			break;
		}

		case 'p':
			new_timings.pixel_clock = simple_strtoul(o, NULL, 0);
			break;

		default:
			dev_err(dev, "unknown option %c\n", c);
			r = -EINVAL;
			goto err;
		}
	}

	if (memcmp(&new_timings, &old_timings, sizeof(new_timings)) != 0) {
		if (display->set_timings)
			display->set_timings(display, &new_timings);

		/* sigh, bpp is not a setting of the display, but
		 * the overlay. */
		//def_display->panel->bpp = bpp;
	}

	if (enable) {
		/*
		 * We are not checking return value of
		 * display->enable call, since it only returns if
		 * display is alreay enabled.
		 */
		r = display->enable(display);
/*		if (r)
			dev_err(dev, "failed to enable display\n");
*/
	} else {
		display->disable(display);
	}

	if (display->set_update_mode && display->get_update_mode) {
		if (mode != display->get_update_mode(display))
			display->set_update_mode(display, mode);
	}

	if (display->enable_te && display->get_te) {
		if (te != display->get_te(display))
			display->enable_te(display, te);
	}

	if (display->set_rotate && display->get_rotate) {
		if (rot != display->get_rotate(display))
			display->set_rotate(display, rot);
	}

	if (display->set_mirror && display->get_mirror) {
		if (mir != display->get_mirror(display))
			display->set_mirror(display, mir);
	}

	r = count;
err:
	omapfb_unlock(fbdev);
	return r;
}
#ifdef CONFIG_PM
/*
 * Default time-out value for Fbdev
 */
#define OMAP2FB_DEF_SLEEP_TIMEOUT (1 * 20 * HZ)

static int omap2fb_can_sleep = -1;
/*
 * TODO: This needs to removed, had to keep this due to uart
 * wakeup hook.
 */
static struct omapfb2_device *omap2fb;

/*
 * TODO: Try to accomodate these variables in omapfb2_device
 * structure.
 */
static struct workqueue_struct *irq_work_queues; /* workqueue*/
static struct work_struct irq_work_queue;        /* work entry */
/*
 * Resumes the FBDEV Module
 * Here Clocks will be turned-on, Context will be restored
 */
void omap2fb_resume_idle(void)
{
	if (omap2fb_can_sleep == 2) {
		omap2fb_can_sleep = 3;
		queue_work(irq_work_queues, &irq_work_queue);
	} else if (omap2fb_can_sleep != -1)

	if (omap2fb->sleep_timeout)
		mod_timer(&omap2fb->timer, jiffies + omap2fb->sleep_timeout);
}
EXPORT_SYMBOL(omap2fb_resume_idle);
/*
 * Timer Call-back function
 */
static void omap2fb_timer_clbk(unsigned long data)
{
	omap2fb_can_sleep = 1;
	queue_work(irq_work_queues, &irq_work_queue);
	del_timer(&omap2fb->timer);
}

void omap2fb_workqueue_handler(struct work_struct *work)
{
	int i;
	struct omap_display *display;

	DEFINE_WAIT(wait);

	if (omap2fb_can_sleep == 1) {
		for (i = 0; i < omap2fb->num_fbs; i++) {
			if (omap2fb->overlays[i]->manager &&
					omap2fb->overlays[i]->manager->display) {
				display =
					omap2fb->overlays[i]->manager->display;
				display->disable(display);
			}
		}
		omap2fb_can_sleep = 2;
	} else if (omap2fb_can_sleep == 3){
		for (i = 0; i < omap2fb->num_fbs; i++) {
			if (omap2fb->overlays[i]->manager &&
					omap2fb->overlays[i]->manager->display) {
				display = omap2fb->overlays[i]->manager->display;
				display->enable(display);
			}
		}
		omap2fb_can_sleep = 0;
	}
}
/*
 * Initialize the Timer fofr DSS, configure the timer to default value
 * of 10 Sec.
 */
void dss_init_timer(struct omapfb2_device *fbdev)
{
	omap2fb = fbdev;
	omap2fb_can_sleep = 0;
	fbdev->sleep_timeout = OMAP2FB_DEF_SLEEP_TIMEOUT;
	setup_timer(&fbdev->timer, omap2fb_timer_clbk,
			(unsigned long) fbdev);
	mod_timer(&fbdev->timer, jiffies + fbdev->sleep_timeout);

	/*
	 * Enable auto-Idle mode here
	 */
}
/*
 * SYSFS entry to show Time-Out value for DSS
 */
static ssize_t dss_sleep_show_timeout(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omapfb2_device *fbdev = platform_get_drvdata(pdev);

	return sprintf(buf, "%u\n", fbdev->sleep_timeout / HZ);
}

/*
 * SYSFS entry to reconfigure Time-Out value for DSS
 */
static ssize_t dss_sleep_store_timeout(struct device *dev,
               struct device_attribute *attr, const char *buf, size_t n)
{
	unsigned int value;
	struct platform_device *pdev = to_platform_device(dev);
	struct omapfb2_device *fbdev = platform_get_drvdata(pdev);

	if (sscanf(buf, "%u", &value) != 1) {
		printk(KERN_ERR "sleep_timeout_store: Invalid value\n");
		return -EINVAL;
	}
	if (value == 0) {
		fbdev->sleep_timeout = 0;
		del_timer(&fbdev->timer);
	} else {
		fbdev->sleep_timeout = value * HZ;
		omap2fb_can_sleep = 0;
		mod_timer(&fbdev->timer, jiffies + fbdev->sleep_timeout);
	}

	return n;
}
#endif

static DEVICE_ATTR(framebuffers, S_IRUGO | S_IWUSR,
		show_framebuffers, store_framebuffers);
static DEVICE_ATTR(overlays, S_IRUGO | S_IWUSR,
		show_overlays, store_overlays);
static DEVICE_ATTR(managers, S_IRUGO | S_IWUSR,
		show_managers, store_managers);
static DEVICE_ATTR(displays, S_IRUGO | S_IWUSR,
		show_displays, store_displays);
#ifdef CONFIG_PM
static DEVICE_ATTR (sleep_timeout, S_IRUGO | S_IWUSR,
        dss_sleep_show_timeout, dss_sleep_store_timeout);
#endif

static struct attribute *omapfb_attrs[] = {
	&dev_attr_framebuffers.attr,
	&dev_attr_overlays.attr,
	&dev_attr_managers.attr,
	&dev_attr_displays.attr,
#ifdef CONFIG_PM
	&dev_attr_sleep_timeout.attr,
#endif
	NULL,
};

static struct attribute_group omapfb_attr_group = {
	.attrs = omapfb_attrs,
};

void omapfb_create_sysfs(struct omapfb2_device *fbdev)
{
	int r;

	r = sysfs_create_group(&fbdev->dev->kobj, &omapfb_attr_group);
	if (r)
		dev_err(fbdev->dev, "failed to create sysfs clk file\n");

#ifdef CONFIG_PM
	/*
	 * Create Work queue for the FBDEV time out handling.
	 * This is required since PM and UART are linked up with
	 * each other under interrupt disable context, and if we tie
	 * this up with Uart then twl4030 related calls will not work.
	 * So we need to have seperate Work Queue to handle prepare_idle
	 * and resume_idle scenarios.
	 */
	irq_work_queues = create_singlethread_workqueue("omapfb");
	if (irq_work_queues == NULL) {
		printk("Could not create omap2fb workqueue\n");
		return;
	}
	INIT_WORK(&irq_work_queue, omap2fb_workqueue_handler);

	dss_init_timer(fbdev);
#endif

}

void omapfb_remove_sysfs(struct omapfb2_device *fbdev)
{
	sysfs_remove_group(&fbdev->dev->kobj, &omapfb_attr_group);
}

