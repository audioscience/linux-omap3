/*
 * dspbridge/src/hal/common/inc/hal_mmu.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
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
 *  ======== hal_mmu.h ========
 *  Description:
 *      MMU types and API declarations
 *
 *! Revision History:
 *! ================
 *! 19-Apr-2004 sb  Moved & renamed endianness, page size, element size
		    TLBAdd takes in MMUMapAttrs instead of separate arguments
 *! 08-Mar-2004 sb  Added the Page Table management APIs
 *! 16 Feb 2003 sb: Initial version
 */
#ifndef __HAL_MMU_H
#define __HAL_MMU_H

/*
 * INCLUDE FILES (only if necessary)
 */

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * EXPORTED DEFINITIONS
 */

/*
* DEFINITION:
*
* DESCRIPTION:  Bitmasks for interrupt sources
*/
#define HAL_MMU_TLB_MISS	    0x1
#define HAL_MMU_TRANSLATION_FAULT   0x2
#define HAL_MMU_EMU_MISS	    0x4
#define HAL_MMU_TABLE_WALK_FAULT    0x8
#define HAL_MMU_MULTI_HIT_FAULT     0x10
#define HAL_MMU_ALL_INTERRUPTS      0x1F

#define HAL_MMU_COARSE_PAGE_SIZE 0x400

/*
 * EXPORTED TYPES
 */

/*
* TYPE:	 HAL_MMUMixedSize_t
*
* DESCRIPTION:  Enumerated Type used to specify whether to follow CPU/TLB
* 		Element size
*/
typedef enum HAL_MMUMixedSize {
    HAL_MMU_TLBES,
    HAL_MMU_CPUES

} HAL_MMUMixedSize_t;

/*
* TYPE:	 HAL_MMUMapAttrs_t
*
* DESCRIPTION:  Struct containing MMU mapping attributes
*/
struct HAL_MMUMapAttrs_t {
    HAL_Endianism_t     endianism;
    HAL_ElementSize_t   elementSize;
    HAL_MMUMixedSize_t  mixedSize;
} ;

/*
 * EXPORTED VARIABLES
 */


/*
 * EXPORTED FUNCTIONS
 */

extern HAL_STATUS HAL_MMU_Enable(const UWORD32 baseAddress);

extern HAL_STATUS HAL_MMU_Disable(const UWORD32 baseAddress);

extern HAL_STATUS HAL_MMU_NumLockedSet(const UWORD32 baseAddress,
					UWORD32 numLockedEntries);

extern HAL_STATUS HAL_MMU_VictimNumSet(const UWORD32 baseAddress,
					UWORD32 victimEntryNum);

/* For MMU faults */
extern HAL_STATUS HAL_MMU_EventAck(const UWORD32 baseAddress,
				    UWORD32 irqMask);

extern HAL_STATUS HAL_MMU_EventDisable(const UWORD32 baseAddress,
					UWORD32 irqMask);

extern HAL_STATUS HAL_MMU_EventEnable(const UWORD32 baseAddress,
				       UWORD32 irqMask);

extern HAL_STATUS HAL_MMU_EventStatus(const UWORD32 baseAddress,
				       UWORD32 *irqMask);

extern HAL_STATUS HAL_MMU_FaultAddrRead(const UWORD32 baseAddress,
					 UWORD32 *addr);

/* Set the TT base address */
extern HAL_STATUS HAL_MMU_TTBSet(const UWORD32 baseAddress,
				  UWORD32 TTBPhysAddr);

extern HAL_STATUS HAL_MMU_TWLEnable(const UWORD32 baseAddress);

extern HAL_STATUS HAL_MMU_TWLDisable(const UWORD32 baseAddress);

extern HAL_STATUS HAL_MMU_TLBFlush(const UWORD32 baseAddress,
				    UWORD32 virtualAddr,
				    UWORD32 pageSize);

extern HAL_STATUS HAL_MMU_TLBFlushAll(const UWORD32 baseAddress);

extern HAL_STATUS HAL_MMU_TLBAdd(const UWORD32     baseAddress,
				  UWORD32	   physicalAddr,
				  UWORD32	   virtualAddr,
				  UWORD32	   pageSize,
				  UWORD32	    entryNum,
				  struct HAL_MMUMapAttrs_t *mapAttrs,
				  HAL_SetClear_t    preservedBit,
				  HAL_SetClear_t    validBit);


/* For PTEs */
extern HAL_STATUS HAL_MMU_PteSet(const UWORD32     pgTblVa,
				  UWORD32	   physicalAddr,
				  UWORD32	   virtualAddr,
				  UWORD32	   pageSize,
				  struct HAL_MMUMapAttrs_t *mapAttrs);

extern HAL_STATUS HAL_MMU_PteClear(const UWORD32   pgTblVa,
				    UWORD32	 pgSize,
				    UWORD32	 virtualAddr);

static inline UWORD32 HAL_MMU_PteAddrL1(UWORD32 L1_base, UWORD32 va)
{
    UWORD32 VA_31_to_20;

    VA_31_to_20  = va >> (20 - 2); /* Left-shift by 2 here itself */
    VA_31_to_20 &= 0xFFFFFFFCUL;

    /* return ( (L1_base & 0xFFFFC000) | VA_31_to_20 ); */
    return (L1_base + VA_31_to_20);
}

static inline UWORD32 HAL_MMU_PteAddrL2(UWORD32 L2_base, UWORD32 va)
{
    return ((L2_base & 0xFFFFFC00) | ((va >> 10) & 0x3FC));
}

static inline UWORD32 HAL_MMU_PteCoarseL1(UWORD32 pteVal)
{
    return (pteVal & 0xFFFFFC00);
}

static inline UWORD32 HAL_MMU_PteSizeL1(UWORD32 pteVal)
{
    UWORD32 pteSize = 0;

    if ((pteVal & 0x3) == 0x1) {
	/* Points to L2 PT */
	pteSize = HAL_MMU_COARSE_PAGE_SIZE;
    }

    if ((pteVal & 0x3) == 0x2) {
	if (pteVal & (1 << 18))
	    pteSize = HAL_PAGE_SIZE_16MB;
	else
	    pteSize = HAL_PAGE_SIZE_1MB;
    }

    return pteSize;
}

static inline UWORD32 HAL_MMU_PteSizeL2(UWORD32 pteVal)
{
    UWORD32 pteSize = 0;

    if (pteVal & 0x2)
	pteSize = HAL_PAGE_SIZE_4KB;
    else if (pteVal & 0x1)
	pteSize = HAL_PAGE_SIZE_64KB;

    return pteSize;
}

#ifdef __cplusplus
}
#endif
#endif  /* __HAL_MMU_H */
