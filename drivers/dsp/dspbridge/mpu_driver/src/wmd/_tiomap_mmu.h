/*
 * dspbridge/src/wmd/linux/omap/2430/_tiomap_mmu.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */


/*
 *  ======== _tiomap_mmu.h ========
 *  Description:
 *      Definitions and types for the DSP MMU modules
 *
 *! Revision History
 *! ================
 *! 19-Apr-2004 sb:  Renamed HW types. Removed dspMmuTlbEntry
 *! 05-Jan-2004 vp:  Moved the file to a platform specific folder from common.
 *! 21-Mar-2003 sb:  Added macro definition TIHEL_LARGEPAGESIZE
 *! 08-Oct-2002 rr:  Created.
 */

#ifndef _TIOMAP_MMU_
#define _TIOMAP_MMU_

#include "_tiomap.h"

#define ARM_DSP_MMU_START               0xfffed200
#define ARM_DSP_MMU_LENGTH              0xff

/* Size in bytes of DSP MMU PAGES */
#define TIHEL_SMALLPAGESIZE             0x1000	/* 4Kb  */
#define TIHEL_LARGEPAGESIZE             0x10000	/* 64Kb */
#define TIHEL_SECTION                   0x100000	/* 1Mb  */

/* DSP MMU related */
#define CNTL_REG_OFFSET                 0x08
#define MMU_FAULT_AD_H_REG_OFFSET       0x0c
#define MMU_FAULT_AD_L_REG_OFFSET       0x10
#define MMU_F_ST_REG_OFFSET             0x14
#define MMU_IT_ACK_REG_OFFSET           0x18
#define LOCK_REG_OFFSET                 0x24
#define LD_TLB_REG_OFFSET               0x28
#define CAM_H_REG_OFFSET                0x2C
#define CAM_L_REG_OFFSET                0x30
#define RAM_H_REG_OFFSET                0x34
#define RAM_L_REG_OFFSET                0x38
#define FLUSH_ENTRY_REG_OFFSET          0x40

#define MMU_IT_ACK_MASK                 0x01

/* Control Register */
#define RESET_SW_MMU_MASK               0x0001
#define MMU_ENABLE_MASK                 0x0002
#define WTL_ENABLE_MASK                 0x0004
#define STREAMING_BUFF_EN_MASK          0x0010
#define ENDIANISM_EN_MASK               0x0020
#define BURST_16MNGT_EN_MASK            0x0040

typedef enum {
	SECTION = 0,
	LARGE_PAGE = 1,
	SMALL_PAGE = 2,
	TINY_PAGE = 3
} SLST_t;

typedef enum {
	ENTRY_NOT_PRESERVED = 0,
	ENTRY_PRESERVED = 1
} PRESERVED_t;

typedef enum {
	NOT_ACCESSIBLE = 0,
	READ_ONLY = 2,
	FULL_ACCESS = 3
} AP_t;

/* Function prototypes */
/*
 *  ======== configureDspMmu ========
 *
 *  Make DSP MMu page table entries.
 *  Note: Not utilizing Coarse / Fine page tables.
 *  SECTION = 1MB, LARGE_PAGE = 64KB, SMALL_PAGE = 4KB, TINY_PAGE = 1KB.
 *  DSP Byte address 0x40_0000 is word addr 0x20_0000.
 */
extern VOID configureDspMmu(struct WMD_DEV_CONTEXT *pDevContext,
			    DWORD dataBasePhys,
			    DWORD dspBaseVirt,
			    DWORD sizeInBytes,
			    INT nEntryStart,
			    HW_Endianism_t endianism,
			    HW_ElementSize_t elemSize,
			    HW_MMUMixedSize_t mixedSize);

#endif				/* _TIOMAP_MMU_ */
