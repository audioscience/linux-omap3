/*
 * linux/arch/arm/mach-omap2/hsmmc.c
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

#include <mach/hardware.h>
#include <mach/control.h>
#include <mach/mmc.h>
#include <mach/board.h>

#include "hsmmc.h"

#if (defined(CONFIG_MMC_OMAP_HS) || defined(CONFIG_MMC_OMAP_HS_MODULE))

extern struct hsmmc_controller hsmmc[OMAP_NO_OF_MMC_CTRL];
extern u16 control_pbias_offset;
extern u16 control_devconf1_offset;
int pr785_mmc_set_voltage(struct hsmmc_controller *c, int vdd);
int twl_mmc_set_voltage(struct hsmmc_controller *c, int vdd);

int hsmmc_card_detect(int irq)
{
	unsigned i;

	for (i = 0; i < OMAP_NO_OF_MMC_CTRL; i++) {
		struct omap_mmc_platform_data *mmc;

		mmc = hsmmc[i].mmc;
		if (!mmc)
			continue;
		if (irq != mmc->slots[0].card_detect_irq)
			continue;

		/* NOTE: assumes card detect signal is active-low */
		return !gpio_get_value_cansleep(mmc->slots[0].switch_pin);
	}
	return -ENOSYS;
}

int hsmmc_get_ro(struct device *dev, int slot)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	/* NOTE: assumes write protect signal is active-high */
	return gpio_get_value_cansleep(mmc->slots[0].gpio_wp);
}

/*
 * MMC Slot Initialization.
 */
int hsmmc_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;
	int ret = 0;
	int i;

	ret = gpio_request(mmc->slots[0].switch_pin, "mmc_cd");
	if (ret)
		goto done;
	ret = gpio_direction_input(mmc->slots[0].switch_pin);
	if (ret)
		goto err;

	for (i = 0; i < ARRAY_SIZE(hsmmc); i++) {
		if (hsmmc[i].name == mmc->slots[0].name) {
			hsmmc[i].mmc = mmc;
			break;
		}
	}

	return 0;

err:
	gpio_free(mmc->slots[0].switch_pin);
done:
	mmc->slots[0].card_detect_irq = 0;
	mmc->slots[0].card_detect = NULL;

	dev_err(dev, "err %d configuring card detect\n", ret);
	return ret;
}

void hsmmc_cleanup(struct device *dev)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	gpio_free(mmc->slots[0].switch_pin);
}

#ifdef CONFIG_PM

int hsmmc_suspend(struct device *dev, int slot)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	disable_irq(mmc->slots[0].card_detect_irq);
	return 0;
}

int hsmmc_resume(struct device *dev, int slot)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	enable_irq(mmc->slots[0].card_detect_irq);
	return 0;
}

#else
#define hsmmc_suspend	NULL
#define hsmmc_resume	NULL
#endif

int hsmmc1_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	u32 reg;
	int ret = 0;
	struct hsmmc_controller *c = &hsmmc[0];
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	if (power_on) {
		if (cpu_is_omap2430()) {
			reg = omap_ctrl_readl(OMAP243X_CONTROL_DEVCONF1);
			if ((1 << vdd) >= MMC_VDD_30_31)
				reg |= OMAP243X_MMC1_ACTIVE_OVERWRITE;
			else
				reg &= ~OMAP243X_MMC1_ACTIVE_OVERWRITE;
			omap_ctrl_writel(reg, OMAP243X_CONTROL_DEVCONF1);
		}

		if (mmc->slots[0].internal_clock) {
			reg = omap_ctrl_readl(OMAP2_CONTROL_DEVCONF0);
			reg |= OMAP2_MMCSDIO1ADPCLKISEL;
			omap_ctrl_writel(reg, OMAP2_CONTROL_DEVCONF0);
		}

		reg = omap_ctrl_readl(control_pbias_offset);
		reg |= OMAP2_PBIASSPEEDCTRL0;
		reg &= ~OMAP2_PBIASLITEPWRDNZ0;
		omap_ctrl_writel(reg, control_pbias_offset);

#if defined(CONFIG_TWL4030_CORE)
		ret = twl_mmc_set_voltage(c, vdd);
#endif
#if defined(CONFIG_OMAP3EVM_PR785)
		ret = pr785_mmc_set_voltage(c, vdd);
#endif
		/* 100ms delay required for PBIAS configuration */
		msleep(100);
		reg = omap_ctrl_readl(control_pbias_offset);
		reg |= (OMAP2_PBIASLITEPWRDNZ0 | OMAP2_PBIASSPEEDCTRL0);
		if ((1 << vdd) <= MMC_VDD_165_195)
			reg &= ~OMAP2_PBIASLITEVMODE0;
		else
			reg |= OMAP2_PBIASLITEVMODE0;
		omap_ctrl_writel(reg, control_pbias_offset);
	} else {
		reg = omap_ctrl_readl(control_pbias_offset);
		reg &= ~OMAP2_PBIASLITEPWRDNZ0;
		omap_ctrl_writel(reg, control_pbias_offset);

#if defined(CONFIG_TWL4030_CORE)
		ret = twl_mmc_set_voltage(c, 0);
#endif
#if defined(CONFIG_OMAP3EVM_PR785)
		ret = pr785_mmc_set_voltage(c, 0);
#endif

		/* 100ms delay required for PBIAS configuration */
		msleep(100);
		reg = omap_ctrl_readl(control_pbias_offset);
		reg |= (OMAP2_PBIASSPEEDCTRL0 | OMAP2_PBIASLITEPWRDNZ0 |
			OMAP2_PBIASLITEVMODE0);
		omap_ctrl_writel(reg, control_pbias_offset);
	}

	return ret;
}

int hsmmc2_set_power(struct device *dev, int slot, int power_on, int vdd)
{
	int ret;
	struct hsmmc_controller *c = &hsmmc[1];
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	if (power_on) {
		if (mmc->slots[0].internal_clock) {
			u32 reg;

			reg = omap_ctrl_readl(control_devconf1_offset);
			reg |= OMAP2_MMCSDIO2ADPCLKISEL;
			omap_ctrl_writel(reg, control_devconf1_offset);
		}
#if defined(CONFIG_TWL4030_CORE)
		ret = twl_mmc_set_voltage(c, vdd);
#endif
#if defined(CONFIG_OMAP3EVM_PR785)
		ret = pr785_mmc_set_voltage(c, vdd);
#endif
	} else {
#if defined(CONFIG_TWL4030_CORE)
		ret = twl_mmc_set_voltage(c, 0);
#endif
#if defined(CONFIG_OMAP3EVM_PR785)
		ret = pr785_mmc_set_voltage(c, 0);
#endif
	}

	return ret;
}

#endif
