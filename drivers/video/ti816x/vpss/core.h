/*
 *
 * VPSS Core  driver for TI 816X
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

#ifndef __DRIVERS_VIDEO_TI816X_VPSS_CORE_H__
#define __DRIVERS_VIDEO_TI816X_VPSS_CORE_H__


#ifdef CONFIG_TI816X_VPSS_DEBUG_SUPPORT
#define DEBUG
extern unsigned int vpss_debug;
#endif

#include <linux/platform_device.h>

#ifdef DEBUG
#define VPSSDBG(format, ...) \
	do { \
		if (vpss_debug) \
			printk(KERN_INFO "VPSS_" VPSS_SUBMODULE_NAME ": " \
				format, ## __VA_ARGS__); \
	} while (0)

#define VPSSERR(format, ...) \
	do { \
		if (vpss_debug)  \
			printk(KERN_ERR "VPSS_" VPSS_SUBMODULE_NAME ": " \
				format, ## __VA_ARGS__); \
	} while (0)

#else
#define VPSSDBG(format, ...)
#define VPSSERR(format, ...)
#endif

/*defined the memory informaton shared between A8 and M3 for each submodule*/
struct vps_payload_info {
	u32           paddr;
	void          *vaddr;
	u32	      size;
};


/*grpx*/
int __init vps_grpx_init(struct platform_device *pdev);
void __exit vps_grpx_deinit(struct platform_device *pdev);

/*display control*/
int __init vps_dc_init(struct platform_device *pdev,
		       char *mode,
		       int tied_vencs);

int __exit vps_dc_deinit(struct platform_device *pdev);

/*fvid2*/
int vps_fvid2_init(struct platform_device *pdev);
void vps_fvid2_deinit(struct platform_device *pdev);


/*shared buffer functions*/
int __init vps_sbuf_init(void);
int __exit vps_sbuf_deinit(void);
void *vps_sbuf_alloc(size_t size, u32 *paddr);
int vps_sbuf_free(u32 paddr, void *vaddr, size_t size);


/*si9022a offchip evm*/
#ifdef CONFIG_TI816X_VPSS_SII9022A
int __init sii9022a_init(struct platform_device *pdev);
int __exit sii9022a_deinit(struct platform_device *pdev);
int sii9022a_setmode(u32 mode);
int sii9022a_start(void);
int sii9022a_stop(void);
#endif

#endif

