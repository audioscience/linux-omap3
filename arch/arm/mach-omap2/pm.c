/*
 * linux/arch/arm/mach-omap2/pm.c
 *
 * OMAP Power Management Common Routines
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Copyright (C) 2006-2008 Nokia Corporation
 *
 * Written by:
 * Richard Woodruff <r-woodruff2@ti.com>
 * Tony Lindgren
 * Juha Yrjola
 * Amit Kucheria <amit.kucheria@nokia.com>
 * Igor Stoppa <igor.stoppa@nokia.com>
 * Jouni Hogander
 *
 * Based on pm.c for omap1
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/suspend.h>
#include <linux/time.h>

#include <plat/cpu.h>
#include <asm/mach/time.h>
#include <asm/atomic.h>

#include "prm-regbits-34xx.h"
#include "pm.h"

atomic_t sleep_block = ATOMIC_INIT(0);

static ssize_t idle_show(struct kobject *, struct kobj_attribute *, char *);
static ssize_t idle_store(struct kobject *k, struct kobj_attribute *,
			  const char *buf, size_t n);

static ssize_t idle_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	return -EINVAL;
}

static ssize_t idle_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t n)
{
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1 ||
	    (value != 0 && value != 1)) {
		printk(KERN_ERR "idle_store: Invalid value\n");
		return -EINVAL;
	}

	return n;
}

void omap2_block_sleep(void)
{
	atomic_inc(&sleep_block);
}

void omap2_allow_sleep(void)
{
	int i;

	i = atomic_dec_return(&sleep_block);
	BUG_ON(i < 0);
}

static int __init omap_pm_init(void)
{
	int error = -1;

	return error;
}
late_initcall(omap_pm_init);
