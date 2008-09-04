/*
 * linux/arch/arm/mach-omap2/cpuidle34xx.c
 *
 * OMAP3 CPU IDLE Routines
 *
 * Copyright (C) 2007-2008 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Karthik Dasu <karthik-dp@ti.com>
 *
 * Copyright (C) 2006 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * Based on pm.c for omap2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpuidle.h>
#include <asm/arch/pm.h>
#include <asm/arch/prcm.h>
#include <asm/arch/powerdomain.h>
#include <asm/arch/control.h>
#include <linux/sched.h>

#include "cpuidle34xx.h"

#ifdef CONFIG_CPU_IDLE

struct omap3_processor_cx omap3_power_states[OMAP3_MAX_STATES];
struct omap3_processor_cx current_cx_state;

static int omap3_idle_bm_check(void)
{
	if (!omap3_can_sleep())
		return 1;
	return 0;
}

int set_pwrdm_state(struct powerdomain *pwrdm, u32 state);
/* omap3_enter_idle - Programs OMAP3 to enter the specified state.
 * returns the total time during which the system was idle.
 */
static int omap3_enter_idle(struct cpuidle_device *dev,
			struct cpuidle_state *state)
{
	struct omap3_processor_cx *cx = cpuidle_get_statedata(state);
	struct timespec ts_preidle, ts_postidle, ts_idle;
	struct powerdomain *mpu_pd, *core_pd;;

	current_cx_state = *cx;

	if (cx->type == OMAP3_STATE_C0) {
		/* Do nothing for C0, not even a wfi */
		return 0;
	}

	local_irq_disable();
	local_fiq_disable();

	/* Used to keep track of the total time in idle */
	getnstimeofday(&ts_preidle);

	if (cx->type > OMAP3_STATE_C1)
		sched_clock_idle_sleep_event(); /* about to enter deep idle */

	mpu_pd = pwrdm_lookup("mpu_pwrdm");
	core_pd = pwrdm_lookup("core_pwrdm");

	set_pwrdm_state(mpu_pd, cx->mpu_state);
	set_pwrdm_state(core_pd, cx->core_state);

	if (omap_irq_pending())
		goto return_sleep_time;

	/* Execute ARM wfi */
	omap_sram_idle();

return_sleep_time:
	getnstimeofday(&ts_postidle);
	ts_idle = timespec_sub(ts_postidle, ts_preidle);

	if (cx->type > OMAP3_STATE_C1)
		sched_clock_idle_wakeup_event(timespec_to_ns(&ts_idle));

	local_irq_enable();
	local_fiq_enable();

	return (u32)timespec_to_ns(&ts_idle)/1000;
}

static int omap3_enter_idle_bm(struct cpuidle_device *dev,
			       struct cpuidle_state *state)
{
	struct cpuidle_state *new_state = NULL;
	int i, j;

	if ((state->flags & CPUIDLE_FLAG_CHECK_BM) && omap3_idle_bm_check()) {

		/* Find current state in list */
		for (i = 0; i < OMAP3_MAX_STATES; i++)
			if (state == &dev->states[i])
				break;
		BUG_ON(i == OMAP3_MAX_STATES);

		/* Back up to non 'CHECK_BM' state */
		for (j = i - 1;  j > 0; j--) {
			struct cpuidle_state *s = &dev->states[j];

			if (!(s->flags & CPUIDLE_FLAG_CHECK_BM)) {
				new_state = s;
				break;
			}
		}

		pr_debug("%s: Bus activity: Entering %s (instead of %s)\n",
			__func__, new_state->name, state->name);
	}

	return omap3_enter_idle(dev, new_state ? : state);
}

DEFINE_PER_CPU(struct cpuidle_device, omap3_idle_dev);

/* omap3_init_power_states - Initialises the OMAP3 specific C states.
 * Below is the desciption of each C state.
 *
	C0 . System executing code
	C1 . MPU WFI + Core active
	C2 . MPU CSWR + Core active
	C3 . MPU OFF + Core active
	C4 . MPU CSWR + Core CSWR
	C5 . MPU OFF + Core CSWR
	C6 . MPU OFF + Core OFF
 */
void omap_init_power_states(void)
{
	/* C0 . System executing code */
	omap3_power_states[0].valid = 1;
	omap3_power_states[0].type = OMAP3_STATE_C0;
	omap3_power_states[0].sleep_latency = 0;
	omap3_power_states[0].wakeup_latency = 0;
	omap3_power_states[0].threshold = 0;
	omap3_power_states[0].mpu_state = PWRDM_POWER_ON;
	omap3_power_states[0].core_state = PWRDM_POWER_ON;
	omap3_power_states[0].flags = CPUIDLE_FLAG_SHALLOW;

	/* C1 . MPU WFI + Core active */
	omap3_power_states[1].valid = 1;
	omap3_power_states[1].type = OMAP3_STATE_C1;
	omap3_power_states[1].sleep_latency = 10;
	omap3_power_states[1].wakeup_latency = 10;
	omap3_power_states[1].threshold = 30;
	omap3_power_states[1].mpu_state = PWRDM_POWER_ON;
	omap3_power_states[1].core_state = PWRDM_POWER_ON;
	omap3_power_states[1].flags = CPUIDLE_FLAG_TIME_VALID |
						CPUIDLE_FLAG_SHALLOW;

	/* C2 . MPU CSWR + Core active */
	omap3_power_states[2].valid = 1;
	omap3_power_states[2].type = OMAP3_STATE_C2;
	omap3_power_states[2].sleep_latency = 50;
	omap3_power_states[2].wakeup_latency = 50;
	omap3_power_states[2].threshold = 300;
	omap3_power_states[2].mpu_state = PWRDM_POWER_RET;
	omap3_power_states[2].core_state = PWRDM_POWER_ON;
	omap3_power_states[2].flags = CPUIDLE_FLAG_TIME_VALID |
						CPUIDLE_FLAG_BALANCED;

	/* C3 . MPU OFF + Core active */
	omap3_power_states[3].valid = 1;
	omap3_power_states[3].type = OMAP3_STATE_C3;
	omap3_power_states[3].sleep_latency = 1500;
	omap3_power_states[3].wakeup_latency = 1800;
	omap3_power_states[3].threshold = 4000;
	omap3_power_states[3].mpu_state = PWRDM_POWER_OFF;
	omap3_power_states[3].core_state = PWRDM_POWER_ON;
	omap3_power_states[3].flags = CPUIDLE_FLAG_TIME_VALID |
			CPUIDLE_FLAG_BALANCED;

	/* C4 . MPU CSWR + Core CSWR*/
	omap3_power_states[4].valid = 1;
	omap3_power_states[4].type = OMAP3_STATE_C4;
	omap3_power_states[4].sleep_latency = 2500;
	omap3_power_states[4].wakeup_latency = 7500;
	omap3_power_states[4].threshold = 12000;
	omap3_power_states[4].mpu_state = PWRDM_POWER_RET;
	omap3_power_states[4].core_state = PWRDM_POWER_RET;
	omap3_power_states[4].flags = CPUIDLE_FLAG_TIME_VALID |
			CPUIDLE_FLAG_BALANCED | CPUIDLE_FLAG_CHECK_BM;

	/* C5 . MPU OFF + Core CSWR */
	omap3_power_states[5].valid = 1;
	omap3_power_states[5].type = OMAP3_STATE_C5;
	omap3_power_states[5].sleep_latency = 3000;
	omap3_power_states[5].wakeup_latency = 8500;
	omap3_power_states[5].threshold = 15000;
	omap3_power_states[5].mpu_state = PWRDM_POWER_OFF;
	omap3_power_states[5].core_state = PWRDM_POWER_RET;
	omap3_power_states[5].flags = CPUIDLE_FLAG_TIME_VALID |
			CPUIDLE_FLAG_BALANCED | CPUIDLE_FLAG_CHECK_BM;

	/* C6 . MPU OFF + Core OFF */
	omap3_power_states[6].valid = 0;
	omap3_power_states[6].type = OMAP3_STATE_C6;
	omap3_power_states[6].sleep_latency = 10000;
	omap3_power_states[6].wakeup_latency = 30000;
	omap3_power_states[6].threshold = 300000;
	omap3_power_states[6].mpu_state = PWRDM_POWER_OFF;
	omap3_power_states[6].core_state = PWRDM_POWER_OFF;
	omap3_power_states[6].flags = CPUIDLE_FLAG_TIME_VALID |
			CPUIDLE_FLAG_DEEP | CPUIDLE_FLAG_CHECK_BM;
}

struct cpuidle_driver omap3_idle_driver = {
	.name = 	"omap3_idle",
	.owner = 	THIS_MODULE,
};
/*
 * omap3_idle_init - Init routine for OMAP3 idle. Registers the OMAP3 specific
 * cpuidle driver with the cpuidle f/w with the valid set of states.
 */
int omap3_idle_init(void)
{
	int i, count = 0;
	struct omap3_processor_cx *cx;
	struct cpuidle_state *state;
	struct cpuidle_device *dev;

	omap3_clear_scratchpad_contents();
	omap3_save_scratchpad_contents();

	omap_init_power_states();
	cpuidle_register_driver(&omap3_idle_driver);

	dev = &per_cpu(omap3_idle_dev, smp_processor_id());

	for (i = 0; i < OMAP3_MAX_STATES; i++) {
		cx = &omap3_power_states[i];
		state = &dev->states[count];

		if (!cx->valid)
			continue;
		cpuidle_set_statedata(state, cx);
		state->exit_latency = cx->sleep_latency + cx->wakeup_latency;
		state->target_residency = cx->threshold;
		state->flags = cx->flags;
		state->enter = (state->flags & CPUIDLE_FLAG_CHECK_BM) ?
			omap3_enter_idle_bm : omap3_enter_idle;
		sprintf(state->name, "C%d", count+1);
		count++;
	}

	if (!count)
		return -EINVAL;
	dev->state_count = count;

	if (cpuidle_register_device(dev)) {
		printk(KERN_ERR "%s: CPUidle register device failed\n",
		       __func__);
		return -EIO;
	}

	return 0;
}
__initcall(omap3_idle_init);
#endif /* CONFIG_CPU_IDLE */
