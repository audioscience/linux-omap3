/*
 * linux/drivers/video/ti816x/ti816xfb/ti816xfb_sysfs.c
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

#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/ti816xfb.h>

#include "fbpriv.h"

static ssize_t show_size(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct ti816xfb_info *nfbi = FB2TFB(fbi);

	return snprintf(buf, PAGE_SIZE, "%lu\n", nfbi->mreg.size);
}


static ssize_t store_size(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf,
			  size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct ti816xfb_info *nfbi = FB2TFB(fbi);
	unsigned long size;
	int r;

	size = PAGE_ALIGN(simple_strtoul(buf, NULL, 0));
	ti816xfb_lock(nfbi);

	/* FIX ME make sure that the FB is not actived,
		or we can not change it*/

	if (nfbi->gctrl->gstate.isstarted) {
		r = -EBUSY;
		goto out;
	}
	if (size != nfbi->mreg.size) {
		r = ti816xfb_realloc_fbmem(fbi, size);
		if (r) {
			dev_err(dev, "realloc fbmem failed\n");
			goto out;
		}
	}

	r = count;
out:
	ti816xfb_unlock(nfbi);

	return r;
}

static ssize_t show_phys(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct ti816xfb_info *nfbi = FB2TFB(fbi);

	return snprintf(buf, PAGE_SIZE, "%0x\n", nfbi->mreg.paddr);
}

static ssize_t show_virt(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct ti816xfb_info *nfbi = FB2TFB(fbi);

	return snprintf(buf, PAGE_SIZE, "%p\n", nfbi->mreg.vaddr);
}

static struct device_attribute ti816xfb_attrs[] = {
	__ATTR(size, S_IRUGO | S_IWUSR, show_size, store_size),
	__ATTR(phy_addr, S_IRUGO, show_phys, NULL),
	__ATTR(virt_addr, S_IRUGO, show_virt, NULL),
};

int ti816xfb_create_sysfs(struct ti816xfb_device *fbdev)
{
	int i;
	int r;

	for (i = 0; i < fbdev->num_fbs; i++) {
		int t;
		for (t = 0; t < ARRAY_SIZE(ti816xfb_attrs); t++) {
			r = device_create_file(fbdev->fbs[i]->dev,
					&ti816xfb_attrs[t]);

			if (r) {
				dev_err(fbdev->dev,
					"failed to create sysfs file\n");
				return r;
			}
		}
	}

	return 0;
}

void ti816xfb_remove_sysfs(struct ti816xfb_device *fbdev)
{
	int i, t;

	for (i = 0; i < fbdev->num_fbs; i++) {
		for (t = 0; t < ARRAY_SIZE(ti816xfb_attrs); t++)
			device_remove_file(fbdev->fbs[i]->dev,
					&ti816xfb_attrs[t]);
	}
}


