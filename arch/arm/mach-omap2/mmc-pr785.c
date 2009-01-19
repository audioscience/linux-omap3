/*
 * linux/arch/arm/mach-omap2/mmc-pr785.c
 *
 * Copyright (C) 2007-2008 Texas Instruments
 * Author:
 * 	Manikandan Pillai <mani.pillai@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <mach/control.h>
#include <mach/mmc.h>
#include <mach/board.h>

#include "hsmmc.h"

#if	defined(CONFIG_OMAP3EVM_PR785) &&\
	(defined(CONFIG_MMC_OMAP_HS) || defined(CONFIG_MMC_OMAP_HS_MODULE))

#define	VMMC1_DEV_GRP		0x27
#define	VMMC1_DEDICATED		0x2A

#define	VMMC2_DEV_GRP		0x2B
#define	VMMC2_DEDICATED		0x2E

#define	OMAP_PR785_MMC1_VSET	0
#define	OMAP_PR785_MMC1_CD	140
#define	OMAP_PR785_MMC1_EN	9
#define	OMAP_PR785_MMC1_CD_MSK	((1<<12))

u16 control_pbias_offset;
u16 control_devconf1_offset;

struct hsmmc_controller hsmmc[OMAP_NO_OF_MMC_CTRL] = {
	{
		.hsmmc_dev_grp		= VMMC1_DEV_GRP,
		.hsmmc_dedicated	= VMMC1_DEDICATED,
	},
	{
		/* control_devconf_offset set dynamically */
		.hsmmc_dev_grp		= VMMC2_DEV_GRP,
		.hsmmc_dedicated	= VMMC2_DEDICATED,
	},
};

/*
 * Sets the MMC voltage
 * The MMC voltage on PR785 can only be set to 3.15 V due to hardware bug
 */
int pr785_mmc_set_voltage(struct hsmmc_controller *c, int vdd)
{
	if (vdd)
		gpio_direction_output(OMAP_PR785_MMC1_VSET, 1);
	else
		gpio_direction_output(OMAP_PR785_MMC1_VSET, 0);
	return 0;
}

static struct omap_mmc_platform_data *hsmmc_data[OMAP34XX_NR_MMC] __initdata;

void __init pr785_hsmmc_init(struct hsmmc_info *controllers)
{
	struct hsmmc_info *c;
	int nr_hsmmc = ARRAY_SIZE(hsmmc_data);

	if (cpu_is_omap2430()) {
		control_pbias_offset = OMAP243X_CONTROL_PBIAS_LITE;
		hsmmc[1].control_devconf_offset = OMAP243X_CONTROL_DEVCONF1;
		nr_hsmmc = 2;
	} else {
		control_pbias_offset = OMAP343X_CONTROL_PBIAS_LITE;
		hsmmc[1].control_devconf_offset = OMAP343X_CONTROL_DEVCONF1;
	}

#if defined(CONFIG_MACH_OMAP3EVM) && defined(CONFIG_OMAP3EVM_PR785)
	/* setup the GPIO for MMC for PR785 Rev ES2 power module */
	/* Setup GPIO 0 - MMC1_VSET */
	gpio_direction_output(OMAP_PR785_MMC1_VSET, 1);
	/* Setup GPIO 9 - MMC1_EN */
	gpio_direction_output(OMAP_PR785_MMC1_EN, 1);

	/* Setup GPIO 140 - MMC1_CD as input */
	if (gpio_request(OMAP_PR785_MMC1_CD, "AF6_34XX_GPIO140_UP") < 0) {
		printk(KERN_ERR "Failed to request MMC IRQ %d \n",
			OMAP_PR785_MMC1_CD);
		return ;
	}
	gpio_direction_input(OMAP_PR785_MMC1_CD);
#endif


	for (c = controllers; c->mmc; c++) {
		struct hsmmc_controller *pr785 = hsmmc + c->mmc - 1;
		struct omap_mmc_platform_data *mmc = hsmmc_data[c->mmc - 1];

		if (!c->mmc || c->mmc > nr_hsmmc) {
			pr_debug("MMC%d: no such controller\n", c->mmc);
			continue;
		}
		if (mmc) {
			pr_debug("MMC%d: already configured\n", c->mmc);
			continue;
		}

		mmc = kzalloc(sizeof(struct omap_mmc_platform_data),
				GFP_KERNEL);
		if (!mmc) {
			pr_err("Cannot allocate memory for mmc device!\n");
			return;
		}

		sprintf(pr785->name, "mmc%islot%i", c->mmc, 1);
		mmc->slots[0].name = pr785->name;
		mmc->nr_slots = 1;
		mmc->slots[0].ocr_mask = MMC_VDD_165_195 |
					MMC_VDD_26_27 | MMC_VDD_27_28 |
					MMC_VDD_29_30 |
					MMC_VDD_30_31 | MMC_VDD_31_32;
		mmc->slots[0].wires = c->wires;
		mmc->dma_mask = 0xffffffff;

		if (c->gpio_cd != -EINVAL) {
			mmc->init = hsmmc_late_init;
			mmc->cleanup = hsmmc_cleanup;
			mmc->suspend = hsmmc_suspend;
			mmc->resume = hsmmc_resume;

			mmc->slots[0].switch_pin = OMAP_PR785_MMC1_CD;
			mmc->slots[0].card_detect_irq = gpio_to_irq(c->gpio_cd);
			mmc->slots[0].card_detect = hsmmc_card_detect;
		} else
			mmc->slots[0].switch_pin = -EINVAL;

		/* write protect normally uses an OMAP gpio */
		if (c->gpio_wp != -EINVAL) {
			gpio_request(c->gpio_wp, "mmc_wp");
			gpio_direction_input(c->gpio_wp);

			mmc->slots[0].gpio_wp = c->gpio_wp;
			mmc->slots[0].get_ro = hsmmc_get_ro;
		} else
			mmc->slots[0].gpio_wp = -EINVAL;

		switch (c->mmc) {
		case 1:
			mmc->slots[0].set_power = hsmmc1_set_power;
			break;
		case 2:
			mmc->slots[0].set_power = hsmmc2_set_power;
			break;
		default:
			pr_err("MMC%d configuration not supported!\n", c->mmc);
			continue;
		}
		hsmmc_data[c->mmc - 1] = mmc;
	}
	omap2_init_mmc(hsmmc_data, OMAP34XX_NR_MMC);
}

#endif
