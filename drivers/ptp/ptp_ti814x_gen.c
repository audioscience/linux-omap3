/*
 * Generic frequency adjust interface for arbitrary DM814X clock
 *
 * Copyright (C) 2012 AudioScience Inc
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

#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <plat/clock.h>

#include <linux/ptp_clock_kernel.h>

#define DRIVER		"ptp_ti814x_gen"
#define REF_CLOCK_NAME		"video0_dpll_ck"

struct clock_info {
	struct ptp_clock *ptp_clock;
	struct ptp_clock_info caps;
	struct clk *ref_clk;
	int initial_freq;
};

static struct clock_info clock_info;

/*
 * PTP clock operations
 */

static int ptp_ti814x_gen_settick(struct ptp_clock_info *ptp, long tick_scaled_ns)
{
	int ret;
	u32 target_freq;
	struct clock_info *clock_info = container_of(ptp, struct clock_info,
			caps);

	if (!tick_scaled_ns) {
		printk(KERN_ERR "Failed to set the rate. tick duration is zero\n");
		return -EINVAL;
	}

	target_freq = div_u64((1000000000ULL << 32), tick_scaled_ns) >> 8;

	ret = clk_set_rate(clock_info->ref_clk, target_freq);
	if (ret) {
		printk(KERN_ERR "Failed to set the rate of " REF_CLOCK_NAME " to %uHz, %d\n", target_freq, ret);
	} else {
		clock_info->initial_freq = target_freq;
		printk(KERN_INFO "Set " REF_CLOCK_NAME " freq to %uHz\n", target_freq);
	}

	return ret;
}

static int ptp_ti814x_gen_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	int ret;
	unsigned long freq = 0, target_freq = 0;
	int diff;
	int neg_adj = 0;
	u64 adj;
	struct clock_info *clock_info = container_of(ptp, struct clock_info,
			caps);

	freq = clock_info->initial_freq;

	if (ppb < 0) {
		neg_adj = 1;
		ppb = -ppb;
	}

	adj = freq;
	adj *= ppb;
	diff = div_u64(adj, 1000000000ULL);
	target_freq = (neg_adj ? freq - diff : freq + diff);

	ret = clk_set_rate(clock_info->ref_clk, target_freq);
	if (ret)
		printk(KERN_ERR "Failed to set the rate of " REF_CLOCK_NAME " to %uHz, %d\n", target_freq, ret);

	return 0;
}

static int ptp_ti814x_gen_enable(struct ptp_clock_info *ptp,
			  struct ptp_clock_request *rq, int on)
{
	/* External event timestamping is not supported */
	return -EOPNOTSUPP;
}

static struct ptp_clock_info ptp_ti814x_gen_caps = {
	.owner		= THIS_MODULE,
	.name		= REF_CLOCK_NAME,
	.max_adj	= 66666655,
	.n_ext_ts	= 0,
	.pps		= 0,
	.adjfreq	= ptp_ti814x_gen_adjfreq,
	.settick	= ptp_ti814x_gen_settick,
	.adjtime	= NULL,
	.gettime	= NULL,
	.settime	= NULL,
	.enable		= ptp_ti814x_gen_enable,
};

/* module operations */

static void __exit ptp_ti814x_gen_exit(void)
{
	clk_disable(clock_info.ref_clk);
	clk_put(clock_info.ref_clk);
	ptp_clock_unregister(clock_info.ptp_clock);
}

static int __init ptp_ti814x_gen_init(void)
{
	clock_info.ref_clk = clk_get(NULL, REF_CLOCK_NAME);
	if (IS_ERR(clock_info.ref_clk)) {
		printk(KERN_ERR "Could not get %s clk\n", REF_CLOCK_NAME);
		return PTR_ERR(clock_info.ref_clk);
	} else {
		printk(KERN_INFO "ptp_ti814x_gen, selected %s clk\n", REF_CLOCK_NAME);
	}
	clk_enable(clock_info.ref_clk);

	clock_info.initial_freq = clock_info.ref_clk->recalc(
			clock_info.ref_clk);

	clock_info.caps = ptp_ti814x_gen_caps;

	clock_info.ptp_clock = ptp_clock_register(&clock_info.caps);

	if (IS_ERR(clock_info.ptp_clock))
		return PTR_ERR(clock_info.ptp_clock);

	return 0;
}

module_init(ptp_ti814x_gen_init);
module_exit(ptp_ti814x_gen_exit);

MODULE_AUTHOR("AudioScience Inc");
MODULE_DESCRIPTION("Generic frequency adjust interface for arbitrary DM814X clock");
MODULE_LICENSE("GPL");
