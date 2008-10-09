/*
 * linux/arch/arm/mach-omap2/board-3430sdp-flash.c
 *
 * Copyright (c) 2007 Texas Instruments
 *
 * Modified from mach-omap2/board-2430sdp-flash.c
 * Author: Rohit Choraria <rohitkc@ti.com>
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
#include <linux/mtd/onenand_regs.h>
#include <linux/types.h>
#include <linux/io.h>

#include <asm/mach/flash.h>
#include <mach/onenand.h>
#include <mach/board.h>
#include <mach/gpmc.h>
#include <mach/nand.h>

#define REG_FPGA_REV			0x10
#define REG_FPGA_DIP_SWITCH_INPUT2	0x60

/*
 * Number of frequencies supported by gpmc
 */
#define NO_GPMC_FREQ_SUPPORTED		2
#define SUPPORTED_FREQ1			83
#define SUPPORTED_FREQ2			166
#define MAX_SUPPORTED_GPMC_CONFIG	3

#define PDC_NOR		1
#define PDC_NAND	2
#define PDC_ONENAND	3
#define DBG_MPDB	4

/* SDP3430 V2 Board CS organization
 * Different from SDP3430 V1. Now 4 switches used to specify CS
 */
static const unsigned char chip_sel_sdp[][GPMC_CS_NUM] = {
/* GPMC CS Indices (ON=0, OFF=1)*/
/* S8-1 2 3 4 IDX   CS0,       CS1,      CS2 ..                    CS7  */
/*ON ON ON ON*/{PDC_NOR, PDC_NAND, PDC_ONENAND, DBG_MPDB, 0, 0, 0, 0},
/*ON ON ON OFF*/{PDC_ONENAND, PDC_NAND, PDC_NOR, DBG_MPDB, 0, 0, 0, 0},
/*ON ON OFF ON */{PDC_NAND, PDC_ONENAND, PDC_NOR, DBG_MPDB, 0, 0, 0, 0},
};

#ifdef CONFIG_OMAP3_PM
/*
 * TBD: At 83 Mhz currently same values as 166 Mhz are used for all cs
 * except for OneNand in synchronous burst mode.Change the
 * values for other cs also according to frequency
 */

struct gpmc_cs_config pdc_sibnor_gpmc_setting[] = {
	{0x1200, 0x00101000, 0x00040401, 0x0E050E05, 0x010A1010, 0x100802C1},
	{0x1200, 0x001f1f00, 0x00080802, 0x1C091C09, 0x01131F1F, 0x1F0F03C2}
};

struct gpmc_cs_config pdc_stnor_gpmc_setting[] = {
	{0x3, 0x00151501, 0x00060602, 0x11091109, 0x01141F1F, 0x1F0F04c4},
	{0x3, 0x00151501, 0x00060602, 0x11091109, 0x01141F1F, 0x1F0F04c4}
};

struct gpmc_cs_config pdc_nand_gpmc_setting[] = {
	{0x800, 0x00030300, 0x00030200, 0x03000400, 0x00050606, 0x030001C0},
	{0x800, 0x00060600, 0x00060401, 0x05010801, 0x00090B0B, 0x050001C0}
};

struct gpmc_cs_config pdc_onenand_gpmc_setting[] = {
	{0x1200, 0x000F0F01, 0x00030301, 0x0F040F04, 0x010F1010, 0x1F060000},
	{0x1200, 0x000F0F01, 0x00030301, 0x0F040F04, 0x010F1010, 0x1F060000}
};

struct gpmc_cs_config fpga_gpmc_setting[] = {
	{0x00611200, 0x00101000, 0x00040402, 0x0F050F05, 0x020F1010, 0x0F0502C2},
	{0x00611200, 0x001F1F01, 0x00080803, 0x1D091D09, 0x041D1F1F, 0x1D0904C4}
};

/*
 * Structure containing the gpmc cs values at different frequencies
 * This structure will be populated run time depending on the
 * values read from FPGA registers..On 3430 SDP FPGA is always on CS3
 */
struct gpmc_freq_config freq_config[NO_GPMC_FREQ_SUPPORTED];
#endif


static struct mtd_partition sdp_nor_partitions[] = {
	/* bootloader (U-Boot, etc) in first sector */
	{
		.name		= "Bootloader-NOR",
		.offset		= 0,
		.size		= SZ_256K,
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
	},
	/* bootloader params in the next sector */
	{
		.name		= "Params-NOR",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_256K,
		.mask_flags	= 0,
	},
	/* kernel */
	{
		.name		= "Kernel-NOR",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_2M,
		.mask_flags	= 0
	},
	/* file system */
	{
		.name		= "Filesystem-NOR",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0
	}
};

static struct flash_platform_data sdp_nor_data = {
	.map_name	= "cfi_probe",
	.width		= 2,
	.parts		= sdp_nor_partitions,
	.nr_parts	= ARRAY_SIZE(sdp_nor_partitions),
};

static struct resource sdp_nor_resource = {
	.start		= 0,
	.end		= 0,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device sdp_nor_device = {
	.name		= "omapflash",
	.id		= 0,
	.dev		= {
			.platform_data = &sdp_nor_data,
	},
	.num_resources	= 1,
	.resource	= &sdp_nor_resource,
};

static int sdp_onenand_setup(void __iomem *, int freq);

static struct mtd_partition sdp_onenand_partitions[] = {
	{
		.name		= "X-Loader-OneNAND",
		.offset		= 0,
		.size		= 4 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE	 /* force read-only */
	},
	{
		.name		= "U-Boot-OneNAND",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 2 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE	 /* force read-only */
	},
	{
		.name		= "U-Boot Environment-OneNAND",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 1 * (64 * 2048),
	},
	{
		.name		= "Kernel-OneNAND",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 16 * (64 * 2048),
	},
	{
		.name		= "File System-OneNAND",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct omap_onenand_platform_data sdp_onenand_data = {
	.parts		= sdp_onenand_partitions,
	.nr_parts	= ARRAY_SIZE(sdp_onenand_partitions),
	.onenand_setup	= sdp_onenand_setup,
	.dma_channel	= -1,	/* disable DMA in OMAP OneNAND driver */
};

static struct platform_device sdp_onenand_device = {
	.name		= "omap2-onenand",
	.id		= -1,
	.dev = {
		.platform_data = &sdp_onenand_data,
	},
};

/*
 * sdp_onenand_setup - The function configures the onenand flash.
 * @onenand_base: Onenand base address
 *
 * @return int:	Currently always returning zero.
 */
static int sdp_onenand_setup(void __iomem *onenand_base, int freq)
{
	/* Onenand setup does nothing at present */
	return 0;
}

/**
 * get_gpmc0_type - Reads the FPGA DIP_SWITCH_INPUT_REGISTER2 to get
 * the various cs values.
 */
u32 get_gpmc0_type(void)
{
	u8 cs;
	void __iomem *fpga_map_addr;

	fpga_map_addr = ioremap(DEBUG_BASE, 4096);
	if (!(__raw_readw(fpga_map_addr + REG_FPGA_REV))) {
		/* we dont have an DEBUG FPGA??? */
		/* Depend on #defines!! default to strata boot return param */
		return 0x0;
	}
	/* S8-DIP-OFF = 1, S8-DIP-ON = 0 */
	cs = (u8) (__raw_readw(fpga_map_addr + REG_FPGA_DIP_SWITCH_INPUT2)
			& 0xF);
	/* ES2.0 SDP's onwards 4 dip switches are provided for CS */
	if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0))
		/* change (S8-1:4=DS-2:0) to (S8-4:1=DS-2:0) */
		cs = ((cs & 8) >> 3) | ((cs & 4) >> 1) |
			((cs & 2) << 1) | ((cs & 1) << 3);
	else
		/* change (S8-1:3=DS-2:0) to (S8-3:1=DS-2:0) */
		cs = ((cs & 4) >> 2) | (cs & 2) | ((cs & 1) << 2);
	iounmap(fpga_map_addr);
	return (cs);
}
static struct mtd_partition sdp_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "X-Loader-NAND",
		.offset         = 0,
		.size           = 4 * NAND_BLOCK_SIZE,
		.mask_flags     = MTD_WRITEABLE,        /* force read-only */
	},
	{
		.name           = "U-Boot-NAND",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x80000 */
		.size           = 4 * NAND_BLOCK_SIZE,
		.mask_flags     = MTD_WRITEABLE,        /* force read-only */
	},
	{
		.name           = "Boot Env-NAND",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x100000 */
		.size           = 2 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "Kernel-NAND",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x140000 */
		.size           = 32 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "File System - NAND",
		.size           = MTDPART_SIZ_FULL,
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x540000 */
	},
};

static struct omap_nand_platform_data sdp_nand_data = {
	.parts          = sdp_nand_partitions,
	.nr_parts       = ARRAY_SIZE(sdp_nand_partitions),
	.nand_setup     = NULL,
	.dma_channel    = -1,           /* disable DMA in OMAP NAND driver */
	.dev_ready      = NULL,
};

static struct resource sdp_nand_resource = {
	.flags          = IORESOURCE_MEM,
};

static struct platform_device sdp_nand_device = {
	.name           = "omap2-nand",
	.id             = 0,
	.dev            = {
	.platform_data  = &sdp_nand_data,
	},
	.num_resources  = 1,
	.resource       = &sdp_nand_resource,
};


/**
 * sdp3430_flash_init - Identify devices connected to GPMC and register.
 *
 * @return - void.
 */
void __init sdp3430_flash_init(void)
{
	u8		cs = 0;
	u8		nandcs = GPMC_CS_NUM + 1;
	u8		onenandcs = GPMC_CS_NUM + 1;
	u8		idx;
	unsigned char	*config_sel = NULL;
	unsigned long	gpmc_base_add;

	gpmc_base_add   = OMAP34XX_GPMC_VIRT;

	idx = get_gpmc0_type();
	if (idx >= MAX_SUPPORTED_GPMC_CONFIG) {
		printk(KERN_ERR "Invalid Chip select Selection..\
				sdp3430_flash_init returned with error\n");
		return;
	}
	config_sel = (unsigned char *)(chip_sel_sdp[idx]);

	/* Configure start address and size of NOR device */
	if (system_rev > OMAP3430_REV_ES1_0) {
		sdp_nor_resource.start  = FLASH_BASE_SDPV2;
		sdp_nor_resource.end    = FLASH_BASE_SDPV2
						+ FLASH_SIZE_SDPV2 - 1;
	} else {
		sdp_nor_resource.start  = FLASH_BASE_SDPV1;
		sdp_nor_resource.end    = FLASH_BASE_SDPV1
						+ FLASH_SIZE_SDPV1 - 1;
	}

	if (platform_device_register(&sdp_nor_device) < 0)
		printk(KERN_ERR "Unable to register NOR device\n");

#ifdef CONFIG_OMAP3_PM
	freq_config[0].freq = SUPPORTED_FREQ1;
	freq_config[1].freq = SUPPORTED_FREQ2;

	/* On 3430 SDP FPGA is always on CS3 */
	freq_config[0].gpmc_cfg[3] = fpga_gpmc_setting[0];
	freq_config[1].gpmc_cfg[3] = fpga_gpmc_setting[1];
#endif
	cs = 0;
	while (cs < GPMC_CS_NUM) {
		switch (config_sel[cs]) {
		case PDC_NOR:
#ifdef CONFIG_OMAP3_PM
			if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0)) {
				freq_config[0].gpmc_cfg[cs] =
					pdc_sibnor_gpmc_setting[0];
				freq_config[1].gpmc_cfg[cs] =
					pdc_sibnor_gpmc_setting[1];
			} else {
				freq_config[0].gpmc_cfg[cs] =
					pdc_stnor_gpmc_setting[0];
				freq_config[1].gpmc_cfg[cs] =
					pdc_stnor_gpmc_setting[1];
			}
#endif
			break;
		case PDC_NAND:
			if (nandcs > GPMC_CS_NUM) {
				nandcs = cs;
#ifdef CONFIG_OMAP3_PM
				freq_config[0].gpmc_cfg[cs] =
					pdc_nand_gpmc_setting[0];
				freq_config[1].gpmc_cfg[cs] =
					pdc_nand_gpmc_setting[1];
#endif
			}
			break;
		case PDC_ONENAND:
			if (onenandcs > GPMC_CS_NUM) {
				onenandcs = cs;
#ifdef CONFIG_OMAP3_PM
				freq_config[0].gpmc_cfg[cs] =
					pdc_onenand_gpmc_setting[0];
				freq_config[1].gpmc_cfg[cs] =
					pdc_onenand_gpmc_setting[1];
#endif
			}
			break;
		};
		cs++;
	}

	cs = 0;
	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		/*
		* xloader/Uboot would have programmed the NAND/oneNAND
		* base address for us This is a ugly hack. The proper
		* way of doing this is to pass the setup of u-boot up
		* to kernel using kernel params - something on the
		* lines of machineID. Check if oneNAND is configured
		*/
		if ((ret & 0xC00) == 0x800) {
			/* Found it!! */
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		} else {
			ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG7);
			if ((ret & 0x3F) == (ONENAND_MAP >> 24))
			onenandcs = cs;
		}
		cs++;
	}
	if ((nandcs > GPMC_CS_NUM) && (onenandcs > GPMC_CS_NUM)) {
		printk(KERN_INFO "NAND/OneNAND: Unable to find configuration "
				" in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		sdp_nand_data.cs        = nandcs;
		sdp_nand_data.gpmc_cs_baseaddr   = (void *)(gpmc_base_add +
					GPMC_CS0_BASE + nandcs*GPMC_CS_SIZE);
		sdp_nand_data.gpmc_baseaddr     = (void *) (gpmc_base_add);

		if (platform_device_register(&sdp_nand_device) < 0)
			printk(KERN_ERR "Unable to register NAND device\n");
	}

	if (onenandcs < GPMC_CS_NUM) {
		sdp_onenand_data.cs = onenandcs;
		if (platform_device_register(&sdp_onenand_device) < 0)
			printk(KERN_ERR "Unable to register OneNAND device\n");
	}

#ifdef CONFIG_OMAP3_PM
	/*
	 * Setting up gpmc_freq_cfg so tat gpmc module is aware of the
	 * frequencies supported and the various config values for cs
	 */
	gpmc_freq_cfg.total_no_of_freq = NO_GPMC_FREQ_SUPPORTED;
	gpmc_freq_cfg.freq_cfg = freq_config;
#endif
}

