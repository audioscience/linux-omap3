/*
 * board-omap3517evm-flash.c
 *
 * Copyright (c) 2008 Texas Instruments,
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/types.h>
#include <linux/io.h>

#include <asm/mach/flash.h>
#include <mach/board.h>
#include <mach/gpmc.h>
#include <mach/nand.h>

static struct mtd_partition omap3517evm_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "xloader-nand",
		.offset		= 0,
		.size		= 4*(128 * 1024),
		.mask_flags	= MTD_WRITEABLE
	},
	{
		.name		= "uboot-nand",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 14*(128 * 1024),
		.mask_flags	= MTD_WRITEABLE
	},
	{
		.name		= "params-nand",

		.offset		= MTDPART_OFS_APPEND,
		.size		= 2*(128 * 1024)
	},
	{
		.name		= "linux-nand",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 40*(128 * 1024)
	},
	{
		.name		= "jffs2-nand",
		.size		= MTDPART_SIZ_FULL,
		.offset		= MTDPART_OFS_APPEND,
	},
};

static struct omap_nand_platform_data omap3517evm_nand_data = {
	.parts		= omap3517evm_nand_partitions,
	.nr_parts	= ARRAY_SIZE(omap3517evm_nand_partitions),
	.nand_setup	= NULL,
	.dma_channel	= -1,		/* disable DMA in OMAP NAND driver */
	.dev_ready	= NULL,
};

static struct resource omap3517evm_nand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device omap3517evm_nand_device = {
	.name		= "omap2-nand",
	.id		= 0,
	.dev		= {
		.platform_data	= &omap3517evm_nand_data,
	},
	.num_resources	= 1,
	.resource	= &omap3517evm_nand_resource,
};


void __init omap3517evm_flash_init(void)
{
	u8		cs = 0;
	u8		nandcs = GPMC_CS_NUM + 1;
	u32		gpmc_base_add = OMAP34XX_GPMC_VIRT;

	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		/*
		* xloader/Uboot would have programmed the NAND/oneNAND
		* base address for us This is a ugly hack. The proper
		* way of doing this is to pass the setup of u-boot up
		* to kernel using kernel params - something on the
		* lines of machineID. Check if NAND/oneNAND is configured
		*/
		if ((ret & 0xC00) == 0x800) {
			/* Found it!! */
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}
	if ((nandcs > GPMC_CS_NUM)) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				" in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		omap3517evm_nand_data.cs	= nandcs;
		omap3517evm_nand_data.gpmc_cs_baseaddr = (void *)(gpmc_base_add +
					GPMC_CS0_BASE + nandcs*GPMC_CS_SIZE);
		omap3517evm_nand_data.gpmc_baseaddr   = (void *) (gpmc_base_add);

		if (platform_device_register(&omap3517evm_nand_device) < 0) {
			printk(KERN_ERR "Unable to register NAND device\n");
		}
	}
}
