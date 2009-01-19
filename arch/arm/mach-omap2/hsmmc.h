/*
 * linux/arch/arm/mach-omap2/hsmmc.h
 *
 * Copyright (C) 2007-2008 Texas Instruments
 * Author:
 * 	Manikandan Pillai <mani.pillai@ti.com>
 *
 * MMC definitions for OMAP2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __HSMMC_H
#define __HSMMC_H

#define OMAP_NO_OF_MMC_CTRL	2

struct hsmmc_info {
	u8	mmc;		/* controller 1/2/3 */
	u8	wires;		/* 1/4/8 wires */
	int	gpio_cd;	/* or -EINVAL */
	int	gpio_wp;	/* or -EINVAL */
	int     ext_clock:1;    /* use external pin for input clock */
};

#define HSMMC_NAME_LEN		9
struct hsmmc_controller {
	struct          omap_mmc_platform_data  *mmc;
	u32             devconf_loopback_clock;
	u16             control_devconf_offset;
	u8              hsmmc_dev_grp;
	u8              hsmmc_dedicated;
	char            name[HSMMC_NAME_LEN];
};

#if     defined(CONFIG_TWL4030_CORE) && \
	(defined(CONFIG_MMC_OMAP) || defined(CONFIG_MMC_OMAP_MODULE) || \
	defined(CONFIG_MMC_OMAP_HS) || defined(CONFIG_MMC_OMAP_HS_MODULE))

void twl4030_mmc_init(struct hsmmc_info *);

#else

static inline void twl4030_mmc_init(struct hsmmc_info *info)
{
}

#endif

#if 	defined(CONFIG_OMAP3EVM_PR785) && \
	defined(CONFIG_MMC_OMAP) || defined(CONFIG_MMC_OMAP_MODULE) || \
	defined(CONFIG_MMC_OMAP_HS) || defined(CONFIG_MMC_OMAP_HS_MODULE)

void pr785_hsmmc_init(struct hsmmc_info *);

#else

static inline void pr785_hsmmc_init(struct hsmmc_info *info)
{
}

#endif

int hsmmc_card_detect(int irq);
int hsmmc_get_ro(struct device *dev, int slot);
int hsmmc_late_init(struct device *dev);
void hsmmc_cleanup(struct device *dev);
#ifdef CONFIG_PM
int hsmmc_suspend(struct device *dev, int slot);
int hsmmc_resume(struct device *dev, int slot);
#endif

int hsmmc1_set_power(struct device *dev, int slot, int power_on, int vdd);
int hsmmc2_set_power(struct device *dev, int slot, int power_on, int vdd);

#endif /*__HSMMC_H */
