/*
 * linux/arch/arm/mach-omap2/cpuidle34xx.h
 *
 * OMAP3 cpuidle structure definitions
 *
 * Copyright (C) 2007-2008 Texas Instruments, Inc.
 * Written by Rajendra Nayak <rnayak@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 */

#ifndef ARCH_ARM_MACH_OMAP2_CPUIDLE_34XX
#define ARCH_ARM_MACH_OMAP2_CPUIDLE_34XX

#define OMAP3_MAX_STATES 7
#define OMAP3_STATE_C0 0 /* C0 - System executing code */
#define OMAP3_STATE_C1 1 /* C1 - MPU WFI + Core active */
#define OMAP3_STATE_C2 2 /* C2 - MPU CSWR + Core active */
#define OMAP3_STATE_C3 3 /* C3 - MPU OFF + Core active */
#define OMAP3_STATE_C4 4 /* C4 - MPU RET + Core RET */
#define OMAP3_STATE_C5 5 /* C5 - MPU OFF + Core RET */
#define OMAP3_STATE_C6 6 /* C6 - MPU OFF + Core OFF */

/* Currently, we support only upto C2 */
#define MAX_SUPPORTED_STATES 3

extern void omap_sram_idle(void);
extern int omap_irq_pending(void);
extern int omap3_can_sleep(void);

struct omap3_processor_cx {
	u8 valid;
	u8 type;
	u32 sleep_latency;
	u32 wakeup_latency;
	u32 mpu_state;
	u32 core_state;
	u32 threshold;
	u32 flags;
};

void omap_init_power_states(void);
int omap3_idle_init(void);

#endif /* ARCH_ARM_MACH_OMAP2_CPUIDLE_34XX */

