/*
 * arch/arm/plat-omap/include/plat/ti816x.h
 *
 * This file contains the address data for various ti816x modules.
 *
 * Copyright (C) 2010 Texas Instruments, Inc. - http://www.ti.com/
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

#define TI816X_SCM_BASE		0x48140000
#define TI816X_CTRL_BASE	TI816X_SCM_BASE
#define TI816X_PRCM_BASE	0x48180000

#define TI816X_ARM_INTC_BASE	0x48200000

#define TI816X_GPMC_BASE	0x50000000

#define TI816X_PCIE_REG_BASE    0x51000000
#define TI816X_PCIE_MEM_BASE    0x20000000
#define TI816X_PCIE_IO_BASE     0x40000000	/* Using 3MB reserved space */

#define TI816X_USBSS_BASE	0x47400000
#define TI816X_USBSS_LEN	0xFFF
#define TI816X_USB0_BASE	0x47401000
#define TI816X_USB1_BASE	0x47401800
#define TI816X_USB_CPPIDMA_BASE	0x47402000
#define TI816X_USB_CPPIDMA_LEN	0x5FFF

#define TI816X_SATA_BASE	0x4A140000

/* TODO move below dma event macros to dma header */
#define TI816X_DMA_SPI1_TX0			16
#define TI816X_DMA_SPI1_RX0			17
#define TI816X_DMA_SPI1_TX1			18
#define TI816X_DMA_SPI1_RX1			19
#define TI816X_DMA_SPI1_TX2			20
#define TI816X_DMA_SPI1_RX2			21
#define TI816X_DMA_SPI1_TX3			22
#define TI816X_DMA_SPI1_RX3			23

#endif /* __ASM_ARCH_TI816X_H */
