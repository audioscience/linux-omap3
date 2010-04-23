/*
 *
 * Framebuffer device registration for TI TI816x platforms
 *
 * Copyright (C) 2009 Texas Instruments Inc.
 * Author: Yihe Hu <yihehu@ti.com>
 *
 * Some code and ideas taken from TI OMAP2 Platforms
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
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/bootmem.h>
#include <linux/io.h>
#include <linux/ti816xfb.h>

#include <mach/hardware.h>
#include <asm/mach/map.h>


#if defined(CONFIG_FB_TI816X) || defined(CONFIG_FB_TI816X_MODULE)

static u64 ti816x_fb_dma_mask = ~(u32)0;
static struct ti816xfb_platform_data ti816xfb_config;
#if 0
	{
	.mem_desc =  {
		.region_cnt = 3,
		.mreg[0] = {0, NULL, 1920*1080*8, 0, 0,},
		.mreg[1] = {0, NULL, 1920*1080*8, 0, 0,},
		.mreg[2] = {0, NULL, 720*480*8,   0, 0,},
	},
	.ctrl_platform_data = NULL,
};
#endif
static struct platform_device ti816x_fb_device = {
	.name		= "ti816xfb",
	.id		= -1,
	.dev = {
		.dma_mask		= &ti816x_fb_dma_mask,
		.coherent_dma_mask	= ~(u32)0,
		.platform_data		= &ti816xfb_config,
	},
	.num_resources = 0,
};

void ti816xfb_set_platform_data(struct ti816xfb_platform_data *data)
{
	ti816xfb_config = *data;
}

static inline int ti816x_init_fb(void)
{
	printk(KERN_INFO "Registered ti816x_fb_device\n");
	return platform_device_register(&ti816x_fb_device);
}

arch_initcall(ti816x_init_fb);

#else

void ti816xfb_reserve_sdram(void) {}
unsigned long ti816xfb_reserve_sram(unsigned long sram_pstart,
				  unsigned long sram_vstart,
				  unsigned long sram_size,
				  unsigned long start_avail,
				  unsigned long size_avail)
{
	return 0;
}


#endif
