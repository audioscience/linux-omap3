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

#define TI816X_USBSS_BASE	0x47400000
#define TI816X_USBSS_LEN	0xFFF
#define TI816X_USB0_BASE	0x47401000
#define TI816X_USB1_BASE	0x47401800
#define TI816X_USB_CPPIDMA_BASE	0x47402000
#define TI816X_USB_CPPIDMA_LEN	0x5FFF

#ifdef TI816X_PRE_SILICON_HAPS
/* only for HAPS54 platform */
#define TI816X_USBSS_IRQ	10 /* usb subsystem interrupt*/
#define TI816X_USB0_IRQ		11 /* usb controller0 intrpt */
#define TI816X_USB1_IRQ		12 /* usb controller1 intrpt */
#else
#define TI816X_USBSS_IRQ	49 /* usb subsystem interrupt*/
#define TI816X_USB0_IRQ		50 /* usb controller0 intrpt */
#define TI816X_USB1_IRQ		51 /* usb controller1 intrpt */
#endif

#endif /* __ASM_ARCH_TI816X_H */
