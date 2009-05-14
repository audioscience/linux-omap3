/*
 * linux/arch/arm/mach-omap2/clockdomain3517.h
 * 
 * OMAP3505/3517 clock domain definitions
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef ARCH_ARM_MACH_OMAP2_CLOCKDOMAIN3517_H
#define ARCH_ARM_MACH_OMAP2_CLOCKDOMAIN3517_H

#include <mach/clockdomain.h>
#include "cm-regbits-34xx.h"

/* TBD: chip type to be changed to OMAP3505/3517 */

static struct clockdomain prm_clkdm = {
	.name		= "prm_clkdm",
	.pwrdm		= { .name = "alwon_pwrdm" },
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

static struct clockdomain cm_clkdm = {
	.name		= "cm_clkdm",
	.pwrdm		= { .name = "alwon_pwrdm" },
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

static struct clockdomain mpu_clkdm = {
	.name		= "mpu_clkdm",
	.pwrdm		= { .name = "alwon_pwrdm" },
	.clktrctrl_mask = OMAP3430_CLKTRCTRL_MPU_MASK,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

static struct clockdomain neon_clkdm = {
	.name		= "neon_clkdm",
	.pwrdm		= { .name = "alwon_pwrdm" },
	.clktrctrl_mask = OMAP3430_CLKTRCTRL_NEON_MASK,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

static struct clockdomain sgx_clkdm = {
	.name		= "sgx_clkdm",
	.pwrdm		= { .name = "alwon_pwrdm" },
	.clktrctrl_mask = OMAP3430ES2_CLKTRCTRL_SGX_MASK,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

static struct clockdomain core_l3_clkdm = {
	.name		= "core_l3_clkdm",
	.pwrdm		= { .name = "alwon_pwrdm" },
	.clktrctrl_mask = OMAP3430_CLKTRCTRL_L3_MASK,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

static struct clockdomain core_l4_clkdm = {
	.name		= "core_l4_clkdm",
	.pwrdm		= { .name = "alwon_pwrdm" },
	.clktrctrl_mask = OMAP3430_CLKTRCTRL_L4_MASK,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

static struct clockdomain dss_clkdm = {
	.name		= "dss_clkdm",
	.pwrdm		= { .name = "alwon_pwrdm" },
	.clktrctrl_mask = OMAP3430_CLKTRCTRL_DSS_MASK,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

static struct clockdomain usbhost_clkdm = {
	.name		= "usbhost_clkdm",
	.pwrdm		= { .name = "alwon_pwrdm" },
	.clktrctrl_mask = OMAP3430ES2_CLKTRCTRL_USBHOST_MASK,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_GE_OMAP3430ES2),
};

static struct clockdomain per_clkdm = {
	.name		= "per_clkdm",
	.pwrdm		= { .name = "alwon_pwrdm" },
	.clktrctrl_mask = OMAP3430_CLKTRCTRL_PER_MASK,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

static struct clockdomain emu_clkdm = {
	.name		= "emu_clkdm",
	.pwrdm		= { .name = "alwon_pwrdm" },
	.clktrctrl_mask = OMAP3430_CLKTRCTRL_EMU_MASK,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

static struct clockdomain dpll1_clkdm = {
	.name		= "dpll1_clkdm",
	.pwrdm		= { .name = "alwon_pwrdm" },
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

static struct clockdomain dpll3_clkdm = {
	.name		= "dpll3_clkdm",
	.pwrdm		= { .name = "alwon_pwrdm" },
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

static struct clockdomain dpll4_clkdm = {
	.name		= "dpll4_clkdm",
	.pwrdm		= { .name = "alwon_pwrdm" },
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

static struct clockdomain dpll5_clkdm = {
	.name		= "dpll5_clkdm",
	.pwrdm		= { .name = "alwon_pwrdm" },
	.omap_chip	= OMAP_CHIP_INIT(CHIP_GE_OMAP3430ES2),
};

/*
 * Clockdomain-powerdomain hwsup dependencies (34XX only)
 */

static struct clkdm_pwrdm_autodep clkdm_pwrdm_autodeps[] = {
	{
		.pwrdm	   = { .name = NULL },
	}
};

/*
 *
 */

static struct clockdomain *clockdomains_omap[] = {

	&cm_clkdm,
	&prm_clkdm,
	&mpu_clkdm,
	&neon_clkdm,
	&sgx_clkdm,
	&core_l3_clkdm,
	&core_l4_clkdm,
	&dss_clkdm,
	&usbhost_clkdm,
	&per_clkdm,
	&emu_clkdm,
	&dpll1_clkdm,
	&dpll3_clkdm,
	&dpll4_clkdm,
	&dpll5_clkdm,

	NULL,
};

#endif /* ARCH_ARM_MACH_OMAP2_CLOCKDOMAIN3517_H */

