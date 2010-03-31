/*
 * ti816x.h
 *
 * This file contains the processor specific definitions of the ti816x family
 * SoCs (currently only has ti816x).
 *
 * Copyright (C) 2010, Texas Instruments, Incorporated
 *
 * See file CREDITS for list of people who contributed to this
 * project.
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

#ifndef __ASM_ARCH_TI816X_H
#define __ASM_ARCH_TI816X_H

#define L3_TI816X_BASE		0x44000000
#define L4_FAST_TI816X_BASE	0x4a000000
#define L4_SLOW_TI816X_BASE	0x48000000

#define L4_TI816X_BASE		L4_SLOW_TI816X_BASE	/* !@@@ REMOVE */

#define TI816X_SCM_BASE		0x48140000
#define TI816X_CTRL_BASE	TI816X_SCM_BASE
#define TI816X_PRCM_BASE	0x48180000

#define TI816X_ARM_INTC_BASE	0x48200000
#define TI816X_IC_BASE		TI816X_ARM_INTC_BASE	/* !@@@ REMOVE */

#define TI816X_GPMC_BASE	0x50000000

#endif /* __ASM_ARCH_TI816X_H */
