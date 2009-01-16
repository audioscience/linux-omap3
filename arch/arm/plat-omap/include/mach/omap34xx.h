/*
 * arch/arm/plat-omap/include/mach/omap34xx.h
 *
 * This file contains the processor specific definitions of the TI OMAP34XX.
 *
 * Copyright (C) 2007 Texas Instruments.
 * Copyright (C) 2007 Nokia Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __ASM_ARCH_OMAP34XX_H
#define __ASM_ARCH_OMAP34XX_H

/*
 * Please place only base defines here and put the rest in device
 * specific headers.
 */

#define L4_34XX_BASE		0x48000000
#define L4_WK_34XX_BASE		0x48300000
#define L4_WK_OMAP_BASE		L4_WK_34XX_BASE
#define L4_PER_34XX_BASE	0x49000000
#define L4_PER_OMAP_BASE	L4_PER_34XX_BASE
#define L4_EMU_34XX_BASE	0x54000000
#define L4_EMU_BASE		L4_EMU_34XX_BASE
#define L3_34XX_BASE		0x68000000
#define L3_OMAP_BASE		L3_34XX_BASE

#define OMAP3430_32KSYNCT_BASE	0x48320000
#define OMAP3430_CM_BASE	0x48004800
#define OMAP3430_PRM_BASE	0x48306800
#define OMAP343X_SMS_BASE	0x6C000000
#define OMAP343X_SDRC_BASE	0x6D000000
#define OMAP34XX_GPMC_BASE	0x6E000000
#define OMAP343X_SCM_BASE	0x48002000
#define OMAP343X_CTRL_BASE	OMAP343X_SCM_BASE

#define OMAP34XX_IC_BASE	0x48200000
#define OMAP34XX_IVA_INTC_BASE	0x40000000
#define OMAP34XX_HSUSB_OTG_BASE	(L4_34XX_BASE + 0xAB000)
#define OMAP34XX_HSUSB_HOST_BASE	(L4_34XX_BASE + 0x64000)
#define OMAP34XX_USBTLL_BASE	(L4_34XX_BASE + 0x62000)
#define OMAP34XX_SR1_BASE	0x480C9000
#define OMAP34XX_SR2_BASE	0x480CB000

#define OMAP34XX_CAMERA_BASE		(L4_34XX_BASE + 0xBC000)
#define OMAP34XX_MAILBOX_BASE		(L4_34XX_BASE + 0x94000)

#define OMAP34XX_VRFB_CTX0	0x70000000
#define OMAP34XX_VRFB_CTX1	0x74000000
#define OMAP34XX_VRFB_CTX2	0x78000000
#define OMAP34XX_VRFB_CTX3	0x7C000000
#define OMAP34XX_VRFB_CTX4	0xE0000000
#define OMAP34XX_VRFB_CTX5	0xE4000000
#define OMAP34XX_VRFB_CTX6	0xE8000000
#define OMAP34XX_VRFB_CTX7	0xEC000000
#define OMAP34XX_VRFB_CTX8	0xF0000000
#define OMAP34XX_VRFB_CTX9	0xF4000000
#define OMAP34XX_VRFB_CTX10	0xF8000000
#define OMAP34XX_VRFB_CTX11	0xFC000000
#define OMAP34XX_VRFB_CTX_SIZE	0x4000000

#if defined(CONFIG_ARCH_OMAP3430)

#define OMAP2_32KSYNCT_BASE		OMAP3430_32KSYNCT_BASE
#define OMAP2_CM_BASE			OMAP3430_CM_BASE
#define OMAP2_PRM_BASE			OMAP3430_PRM_BASE
#define OMAP2_VA_IC_BASE		IO_ADDRESS(OMAP34XX_IC_BASE)

#endif

#define OMAP34XX_DSP_BASE	0x58000000
#define OMAP34XX_DSP_MEM_BASE	(OMAP34XX_DSP_BASE + 0x0)
#define OMAP34XX_DSP_IPI_BASE	(OMAP34XX_DSP_BASE + 0x1000000)
#define OMAP34XX_DSP_MMU_BASE	(OMAP34XX_DSP_BASE + 0x2000000)

/* VDD OPP identifiers */
#define VDD1_OPP	0x1
#define VDD2_OPP	0x2

/* VDD1 OPPS */
#define VDD1_OPP1	0x1
#define VDD1_OPP2	0x2
#define VDD1_OPP3	0x3
#define VDD1_OPP4	0x4
#define VDD1_OPP5	0x5

/* VDD2 OPPS */
#define VDD2_OPP1	0x1
#define VDD2_OPP2	0x2
#define VDD2_OPP3	0x3

#define MIN_VDD1_OPP	VDD1_OPP1
#define MAX_VDD1_OPP	VDD1_OPP5
#define MIN_VDD2_OPP	VDD2_OPP1
#define MAX_VDD2_OPP	VDD2_OPP3

#endif /* __ASM_ARCH_OMAP34XX_H */

