/*
 * linux/arch/arm/mach-omap2/powerdomain3517.h
 *
 * OMAP3505/3517 powerdomain definitions
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

#ifndef ARCH_ARM_MACH_OMAP2_POWERDOMAIN3517_H
#define ARCH_ARM_MACH_OMAP2_POWERDOMAIN3517_H

#include <mach/powerdomain.h>

#include "prcm-common.h"
#include "prm.h"
#include "cm.h"

/* OMAP3505/3517 is an OMAP3430 derivative and is a single power domain device. 
 * The device has one always on power domain.
 */
 
static struct powerdomain alwon_pwrdm = {
	.name		= "alwon_pwrdm",
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430), /* TBD change to 3505/3517 */
};

static struct powerdomain *powerdomains_omap[] __initdata = {

	&alwon_pwrdm,

	NULL
};

#endif /* ARCH_ARM_MACH_OMAP2_POWERDOMAIN3517_H */
