/*
 * linux/drivers/video/ti816x/vpss/core.c
 *
 * VPSS Core driver for TI 816X
 *
 * Copyright (C) 2009 TI
 * Author: Yihe Hu <yihehu@ti.com>
 *
 * Some code and ideas taken from drivers/video/omap2/ driver
 * by Tomi Valkeinen.
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

#include <linux/vmalloc.h>
#include <linux/platform_device.h>

#include "core.h"

#define VPS_DRIVER_NAME  "vpss"



#ifdef DEBUG
unsigned int vpss_debug;
module_param_named(debug, vpss_debug, bool, 0644);
#endif

#ifdef DEBUG
#define VPSDBG(format, ...) \
	if (vpss_debug) \
		printk(KERN_INFO "VPSS_CORE: " format, ## __VA_ARGS__)
#define VPSERR(format, ...) \
	if (vpss_debug) \
		printk(KERN_ERR"VPSS_CORE: " format, ## __VA_ARGS__)


#else
#define VPSDBG(format, ...)
#define VPSERR(format, ...)
#endif

static int vps_probe(struct platform_device *pdev)
{
	int r;
	r = vps_fvid2_init(NULL);
	if (r) {
		VPSERR("Failed to init fvid2 interface,\n");
		return r;
	}

	r = vps_dc_init(pdev);
	if (r) {
		VPSERR("failed to int display controller.\n");
		goto exit1;
	}
	r = vps_grpx_init(pdev);
	if (r) {
		VPSERR("failed to int graphics.\n");
		goto exit2;

	}

	return 0;

exit2:
	vps_dc_deinit(pdev);
exit1:
	vps_fvid2_deinit(NULL);

	return r;
}

static int vps_remove(struct platform_device *pdev)
{
	int r;

	vps_grpx_deinit(pdev);
	r = vps_dc_deinit(pdev);
	if (r) {
		VPSERR("failed to remove display controller.\n");
		return r;
	}

	vps_fvid2_deinit(NULL);
	return 0;
}

static void vps_device_release(struct device *dev)
{
}
static struct platform_device vps_device = {
	.name = VPS_DRIVER_NAME,
	.id = -1,
	.dev = {
		.release = vps_device_release,
	},
};

static struct platform_driver vps_driver = {
	.probe = vps_probe,
	.remove = vps_remove,
	.driver = {
		.name = VPS_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init vps_init(void)
{
	if (platform_driver_register(&vps_driver)) {
		VPSERR("failed to register ti816x-vpss driver\n");
		return -ENODEV;
	}
#ifdef CONFIG_TI816X_VPSS_MODULE
	if (platform_device_register(&vps_device)) {
		VPSERR("failed to register ti816x_vps device\n");
		platform_driver_unregister(&vps_driver);
		return -ENODEV;
	}
#endif
	return 0;
}

static void __exit vps_cleanup(void)
{
#ifdef CONFIG_TI816X_VPSS_MODULE
	platform_device_unregister(&vps_device);
#endif
	platform_driver_unregister(&vps_driver);
}



subsys_initcall(vps_init);
module_exit(vps_cleanup);


MODULE_AUTHOR("Yihe Hu <yihehu@ti.com");
MODULE_DESCRIPTION("TI816X Video Processing Subsystem");
MODULE_LICENSE("GPL v2");
