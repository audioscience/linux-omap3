/*
 * linux/arch/arm/mach-omap2/control3517.c
 *
 * OMAP3505/3517 control module access routines
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

#include <linux/module.h>

#include <mach/control.h>
#include "control3517.h"

void omap3517_ipss_reset_ctrl(u16 module_bit, char reset_ctrl)
{
	u32 v;
	u16 reg_offset = OMAP3517_IP_SW_RESET;

	v = omap_ctrl_readl(reg_offset);

	if (reset_ctrl == IN_RESET) {
		/* Put module in reset */
		v |= module_bit;
		omap_ctrl_writel(v, reg_offset);
	} else if (reset_ctrl == OUT_OF_RESET) {
		/* Put module out of reset */
		v &= ~(module_bit);
		omap_ctrl_writel(v, reg_offset);
	} else {
		/* error */
        pr_debug("control3517: Invalid reset_ctrl");
		WARN_ON(1);
	}
}
EXPORT_SYMBOL(omap3517_ipss_reset_ctrl);

void omap3517_ipss_int_status_clr(u16 int_clr_bit)
{
    u32 v;
    u16 reg_offset = OMAP3517_LVL_INT_CLEAR;

    v = omap_ctrl_readl(reg_offset);

    /* Writing a '1' will clear interrupt; this bit will self clear after that */
    v |= int_clr_bit;
    omap_ctrl_writel(v, reg_offset);
}
EXPORT_SYMBOL(omap3517_ipss_int_status_clr);

