/*
 * OMAP3-specific clock framework functions
 *
 * Copyright (C) 2007-2008 Texas Instruments, Inc.
 * Copyright (C) 2007-2009 Nokia Corporation
 *
 * Written by Paul Walmsley
 * Testing and integration fixes by Jouni Högander
 *
 * Parts of this code are based on code written by
 * Richard Woodruff, Tony Lindgren, Tuukka Tikkanen, Karthik Dasu
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/clk.h>

#include <plat/control.h>
#include <plat/clkdev_omap.h>

#include "clock.h"
#include "clock816x.h"
#include "cm.h"
#include "cm-regbits-44xx.h"
#include "prm.h"
#include "prm-regbits-44xx.h"

/*
 * Secure 32K clock is selected outside of PRCM using SECURE_CLK_CTRL. This is
 * fed to Timer0, WDT and RTC. For now, we skip access to SECURE_CLK_CTRL.
 *
 * !@2 TODO: Considering only SYSCLK18 for now
 */
static struct clk secure_32k_fck = {
	.name		= "secure_32k_fck",
	.ops		= &clkops_null,
	.rate		= 32768,
	.flags		= RATE_IN_TI816X | DEFAULT_RATE,
};

static struct clk omap_32k_ck = {
	.name		= "omap_32k_ck",
	.ops		= &clkops_null,
	.rate		= 32768,
	.flags		= RATE_IN_TI816X | DEFAULT_RATE,
};

static struct clk tclkin_ck = {
	.name		= "tclkin_ck",
	.ops		= &clkops_null,
	.rate		= 32768,		/* FIXME */
	.flags		= RATE_IN_TI816X | DEFAULT_RATE,
};

/*
 * PLL data
 *
 * !@1 TODO: Add macros and reorganize as appropriate
 */



/*
 * Assuming CLKIN fixed @ 27MHz for now. This allows getting rid of "virtual"
 * clocks for osc.
 * !@2 TODO: Need to sample CLK_CTL (0x648) to get the device oscillator
 * frequency selection (but this will just give the range and not the actual
 * frequency of SYS_CLKIN as done for OMAP4 with SYS_CLKSEL.
 */

static struct clk sys_clkin_ck = {
	.name		= "sys_clkin_ck",
	.ops		= &clkops_null,
	.init		= &omap2_init_clksel_parent,	/* !@3 Can probably get
							   rid of this as we are
							   anyway using fixed
							   clock input value */
	.clksel_reg	= NULL,		/* This will catch the case where these
					   clksel values are reuired by some
					   clock functions, e.g.,
					   omap2_clksel_get_divisor */
	.clksel_mask	= 0,
	.clksel		= NULL,		/* !@3 Hardcoded rate skips executing
					   'init' for rate setting */
	/* REVISIT: deal with autoextclkmode? */
	.recalc		= NULL,		/* !@3 Not supported for sys_clkin
					   currently */
};

/*
 * In Netra, DPLL control is outside PRCM thus, we skip following in the
 * subsequent data:
 *	1. Setting clock source
 *	2. Divisor as fed into PLL
 *
 * Ofcourse, we will control internal clock sources (SYSCLKx etc) using CM_DPLL
 * register configurations.
 */

/* DPLLS */
/* ................................ skip! (see above) */

/*
 * TODO: Update flags as -
 *	- RATE_IN_TI816X for clksel_rate data
 *	- CLOCK_IN_TI816X for clk data
 *
 *	This needs also to be taken care at initialization so that the flags
 *	match. !@0
 */

#if 0
static const struct clksel_rate div_1_0_rates[] = {
	{ .div = 1, .val = 0, .flags = RATE_FIXED },
	{ .div = 0 },
};

static const struct clksel_rate div_1_1_rates[] = {
	{ .div = 1, .val = 1, .flags = RATE_FIXED },
	{ .div = 0 },
};

static const struct clksel_rate div_1_2_rates[] = {
	{ .div = 1, .val = 2, .flags = RATE_FIXED },
	{ .div = 0 },
};

static const struct clksel ti816x_gpt2to8_clksel[] = {
	{ .parent = &sys_clkin_ck, .rates = div_1_0_rates },
	{ .parent = &sysclk18_ck, .rates = div_1_1_rates },
	{ .parent = &tclkin_ck, .rates = div_1_2_rates },
	{ .parent = NULL}
};

static struct clk gpt2to8_refclk_mux_ck = {
	.name		= "gpt2to8_refclk_mux_ck",
	.parent		= &sysclk18_ck,
	.clksel		= ti816x_gpt2to8_clksel,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= TI816X_CM_TIMER1_CLKSEL,
	.clksel_mask	= TI816X_CLKSEL_0_1_MASK,
	.recalc		= &omap2_clksel_recalc,
	.ops		= &clkops_null,
	.flags		= RATE_FIXED,
};
#endif

static struct clk gpt2_fck = {
	.name		= "gpt2_fck",
	.ops		= &clkops_omap2_dflt_wait,
#ifndef CONFIG_TI8168_SIM
	.init		= &omap2_init_clksel_parent,
#else
	.rate		= 1024*1024,	/* !@@ Skip CM module access for parent.
					   Also, keep clock rate lower to have
					   lower timer count for simulators. */
#endif
	.enable_reg	= NULL,
	.enable_bit	= -1,
	.clksel_reg	= NULL,
	.clksel_mask	= -1,
#ifndef CONFIG_TI8168_SIM
	.clksel		= gpt2to8_refclk_mux_ck,
#endif
	.clkdm_name	= "per_clkdm",
#ifndef CONFIG_TI8168_SIM
	.recalc		= &omap2_clksel_recalc,
#endif
};

/*
 * !@1 XXX: Just added for clocksource (free-running. Check if really needed and
 * if omap2_gp_clocksource_init called from omap2_gp_timer_init can be avoided
 */
static struct clk gpt3_fck = {
	.name		= "gpt3_fck",
	.ops		= &clkops_omap2_dflt_wait,
#ifndef CONFIG_TI8168_SIM
	.init		= &omap2_init_clksel_parent,
#else
	.rate		= 1024*1024,	/* !@@ Skip CM module access for parent.
					   Also, keep clock rate lower to have
					   lower timer count for simulators. */
#endif
	.enable_reg	= NULL,
	.enable_bit	= -1,
	.clksel_reg	= NULL,
	.clksel_mask	= -1,
#ifndef CONFIG_TI8168_SIM
	.clksel		= omap343x_gpt_clksel,
#endif
	.clkdm_name	= "per_clkdm",
#ifndef CONFIG_TI8168_SIM
	.recalc		= &omap2_clksel_recalc,
#endif
};

/*
 *
 *	TODO:	1. Approriate enable_reg macro
 *		2. clock frequencies
 *		3. Appropriate enable_bit value
 */
static struct clk mmchs1_ick = {
	.name		= "mmchs1_ick",
	.ops		= &clkops_omap2_dflt_wait,
	.parent		= &omap_32k_ck,
	.clkdm_name	= "core_l4_clkdm",
	.enable_reg	= 0,
	.enable_bit	= 0,
	.recalc		= &followparent_recalc,
};

/*
 *
 *	TODO:	1. Approriate enable_reg macro
 *		2. clock frequencies
 *		3. Appropriate enable_bit value
 */
static struct clk mmchs1_fck = {
	.name		= "mmchs1_ick",
	.ops		= &clkops_omap2_dflt_wait,
	.parent		= &omap_32k_ck,
	.clkdm_name	= "core_l3_clkdm",
	.enable_reg	= 0,
	.enable_bit	= 0,
	.recalc		= &followparent_recalc,
};

/*
 *
 *	TODO:	1. Approriate enable_reg macro
 *		2. clock frequencies
 *		3. Appropriate enable_bit value
 */
static struct clk mmchsdb1_fck = {
	.name		= "mmchsdb1_fck",
	.ops		= &clkops_omap2_dflt_wait,
	.parent		= &omap_32k_ck,
	.clkdm_name	= "core_l4_clkdm",
	.enable_reg	= 0,
	.enable_bit	= 0,
	.recalc		= &followparent_recalc,
};

/*
 * clkdev
 */
static struct omap_clk ti816x_clks[] = {
	CLK(NULL,	"omap_32k_ck",	&omap_32k_ck,	CK_TI816X),
	CLK(NULL,	"sys_clkin_ck",	&sys_clkin_ck,	CK_TI816X),
	CLK(NULL,	"gpt2_fck",	&gpt2_fck,	CK_TI816X),
	CLK(NULL,	"gpt3_fck",	&gpt3_fck,	CK_TI816X),
	CLK("mmci-omap-hs.0",	"ick",	&mmchs1_ick,	CK_TI816X),
	CLK("mmci-omap-hs.0",	"fck",	&mmchs1_fck,	CK_TI816X),
	CLK("mmci-omap-hs.0",	"mmchsdb_fck",	&mmchsdb1_fck,	CK_TI816X),
};

int __init ti816x_clk_init(void)
{
	struct omap_clk *c;
	u32 cpu_clkflg;

	if (cpu_is_ti816x()) {
		cpu_mask = RATE_IN_TI816X;
		cpu_clkflg = CK_TI816X;
	}

	clk_init(&omap2_clk_functions);

	for (c = ti816x_clks; c < ti816x_clks + ARRAY_SIZE(ti816x_clks); c++)
		clk_preinit(c->lk.clk);

	for (c = ti816x_clks; c < ti816x_clks + ARRAY_SIZE(ti816x_clks); c++)
		if (c->cpu & cpu_clkflg) {
			clkdev_add(&c->lk);
			clk_register(c->lk.clk);
			omap2_init_clk_clkdm(c->lk.clk);
		}

	recalculate_root_clocks();

	/*
	 * Only enable those clocks we will need, let the drivers
	 * enable other clocks as necessary
	 */
	clk_enable_init_clocks();

	return 0;
}

