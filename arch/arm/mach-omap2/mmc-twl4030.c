/*
 * linux/arch/arm/mach-omap2/mmc-twl4030.c
 *
 * Copyright (C) 2007-2008 Texas Instruments
 * Copyright (C) 2008 Nokia Corporation
 * Author: Texas Instruments
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
#include <linux/i2c/twl4030.h>

#include <mach/hardware.h>
#include <mach/control.h>
#include <mach/mmc.h>
#include <mach/board.h>

#include "hsmmc.h"

#if defined(CONFIG_TWL4030_CORE) && \
	(defined(CONFIG_MMC_OMAP_HS) || defined(CONFIG_MMC_OMAP_HS_MODULE))

#define LDO_CLR			0x00
#define VSEL_S2_CLR		0x40

#define VMMC1_DEV_GRP		0x27
#define VMMC1_CLR		0x00
#define VMMC1_315V		0x03
#define VMMC1_300V		0x02
#define VMMC1_285V		0x01
#define VMMC1_185V		0x00
#define VMMC1_DEDICATED		0x2A

#define VMMC2_DEV_GRP		0x2B
#define VMMC2_CLR		0x40
#define VMMC2_315V		0x0c
#define VMMC2_300V		0x0b
#define VMMC2_285V		0x0a
#define VMMC2_260V		0x08
#define VMMC2_185V		0x06
#define VMMC2_DEDICATED		0x2E

#define VMMC_DEV_GRP_P1		0x20

u16 control_pbias_offset;
u16 control_devconf1_offset;


struct hsmmc_controller hsmmc[OMAP_NO_OF_MMC_CTRL] = {
	{
		.hsmmc_dev_grp		= VMMC1_DEV_GRP,
		.hsmmc_dedicated	= VMMC1_DEDICATED,
	},
	{
		.hsmmc_dev_grp		= VMMC2_DEV_GRP,
		.hsmmc_dedicated	= VMMC2_DEDICATED,
	},
};

/*
 * Sets the MMC voltage in twl4030
 */
int twl_mmc_set_voltage(struct hsmmc_controller *c, int vdd)
{
	int ret = 0;
	u8 vmmc, dev_grp_val;

	switch (1 << vdd) {
	case MMC_VDD_35_36:
	case MMC_VDD_34_35:
	case MMC_VDD_33_34:
	case MMC_VDD_32_33:
	case MMC_VDD_31_32:
	case MMC_VDD_30_31:
		if (c->hsmmc_dev_grp == VMMC1_DEV_GRP)
			vmmc = VMMC1_315V;
		else
			vmmc = VMMC2_315V;
		break;
	case MMC_VDD_29_30:
		if (c->hsmmc_dev_grp == VMMC1_DEV_GRP)
			vmmc = VMMC1_315V;
		else
			vmmc = VMMC2_300V;
		break;
	case MMC_VDD_27_28:
	case MMC_VDD_26_27:
		if (c->hsmmc_dev_grp == VMMC1_DEV_GRP)
			vmmc = VMMC1_285V;
		else
			vmmc = VMMC2_285V;
		break;
	case MMC_VDD_25_26:
	case MMC_VDD_24_25:
	case MMC_VDD_23_24:
	case MMC_VDD_22_23:
	case MMC_VDD_21_22:
	case MMC_VDD_20_21:
		if (c->hsmmc_dev_grp == VMMC1_DEV_GRP)
			vmmc = VMMC1_285V;
		else
			vmmc = VMMC2_260V;
		break;
	case MMC_VDD_165_195:
		if (c->hsmmc_dev_grp == VMMC1_DEV_GRP)
			vmmc = VMMC1_185V;
		else
			vmmc = VMMC2_185V;
		break;
	default:
		vmmc = 0;
		break;
	}

	if (vmmc)
		dev_grp_val = VMMC_DEV_GRP_P1;	/* Power up */
	else
		dev_grp_val = LDO_CLR;		/* Power down */

#if defined(CONFIG_TWL4030_CORE)
	ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					dev_grp_val, c->hsmmc_dev_grp);
	if (ret)
		return ret;

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					vmmc, c->hsmmc_dedicated);
#endif
	return ret;
}

static struct omap_mmc_platform_data *hsmmc_data[OMAP34XX_NR_MMC] __initdata;

void __init twl4030_mmc_init(struct hsmmc_info *controllers)
{
	struct hsmmc_info *c;
	int nr_hsmmc = ARRAY_SIZE(hsmmc_data);

	if (cpu_is_omap2430()) {
		control_pbias_offset = OMAP243X_CONTROL_PBIAS_LITE;
		control_devconf1_offset = OMAP243X_CONTROL_DEVCONF1;
		nr_hsmmc = 2;
	} else {
		control_pbias_offset = OMAP343X_CONTROL_PBIAS_LITE;
		control_devconf1_offset = OMAP343X_CONTROL_DEVCONF1;
	}

	for (c = controllers; c->mmc; c++) {
		struct hsmmc_controller *twl = hsmmc + c->mmc - 1;
		struct omap_mmc_platform_data *mmc = hsmmc_data[c->mmc - 1];

		if (!c->mmc || c->mmc > nr_hsmmc) {
			pr_debug("MMC%d: no such controller\n", c->mmc);
			continue;
		}
		if (mmc) {
			pr_debug("MMC%d: already configured\n", c->mmc);
			continue;
		}

		mmc = kzalloc(sizeof(struct omap_mmc_platform_data), GFP_KERNEL);
		if (!mmc) {
			pr_err("Cannot allocate memory for mmc device!\n");
			return;
		}

		sprintf(twl->name, "mmc%islot%i", c->mmc, 1);
		mmc->slots[0].name = twl->name;
		mmc->nr_slots = 1;
		mmc->slots[0].ocr_mask = MMC_VDD_165_195 |
					MMC_VDD_26_27 | MMC_VDD_27_28 |
					MMC_VDD_29_30 |
					MMC_VDD_30_31 | MMC_VDD_31_32;
		mmc->slots[0].wires = c->wires;
		mmc->slots[0].internal_clock = !c->ext_clock;
		mmc->dma_mask = 0xffffffff;

		/* note: twl4030 card detect GPIOs normally switch VMMCx ... */
		if (gpio_is_valid(c->gpio_cd)) {
			mmc->init = hsmmc_late_init;
			mmc->cleanup = hsmmc_cleanup;
			mmc->suspend = hsmmc_suspend;
			mmc->resume = hsmmc_resume;

			mmc->slots[0].switch_pin = c->gpio_cd;
			mmc->slots[0].card_detect_irq = gpio_to_irq(c->gpio_cd);
			mmc->slots[0].card_detect = hsmmc_card_detect;
		} else
			mmc->slots[0].switch_pin = -EINVAL;

		/* write protect normally uses an OMAP gpio */
		if (gpio_is_valid(c->gpio_wp)) {
			gpio_request(c->gpio_wp, "mmc_wp");
			gpio_direction_input(c->gpio_wp);

			mmc->slots[0].gpio_wp = c->gpio_wp;
			mmc->slots[0].get_ro = hsmmc_get_ro;
		} else
			mmc->slots[0].gpio_wp = -EINVAL;

		/* NOTE:  we assume OMAP's MMC1 and MMC2 use
		 * the TWL4030's VMMC1 and VMMC2, respectively;
		 * and that OMAP's MMC3 isn't used.
		 */

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
