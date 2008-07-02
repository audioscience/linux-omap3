/*
 * dspbridge/src/hal/common/mmu/hal_mmu.c
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
 *  ======== hal_mmu.c ========
 *  Description:
 *      API definitions to setup MMU TLB and PTE
 *
 *! Revision History:
 *! ================
 *! 19-Apr-2004 sb  TLBAdd and TLBFlush input the page size in bytes instead
		    of an enum. TLBAdd inputs mapping attributes struct instead
		    of individual arguments.
		    Removed MMU.h and other cosmetic updates.
 *! 08-Mar-2004 sb  Added the Page Table Management APIs
 *! 16 Feb 2003 sb: Initial version
 */

/*
 * PROJECT SPECIFIC INCLUDE FILES
 */
#include <GlobalTypes.h>
#include "MMURegAcM.h"
#include <hal_defs.h>
#include <hal_mmu.h>

/*
 * LOCAL TYPES AND DEFINITIONS
 */
#define MMU_BASE_VAL_MASK	0xFC00
#define MMU_PAGE_MAX	     3
#define MMU_ELEMENTSIZE_MAX      3
#define MMU_ADDR_MASK	    0xFFFFF000
#define MMU_TTB_MASK	     0xFFFFC000
#define MMU_SECTION_ADDR_MASK    0xFFF00000
#define MMU_SSECTION_ADDR_MASK   0xFF000000
#define MMU_PAGE_TABLE_MASK      0xFFFFFC00
#define MMU_LARGE_PAGE_MASK      0xFFFF0000
#define MMU_SMALL_PAGE_MASK      0xFFFFF000

#define MMU_LOAD_TLB	0x00000001

/*
 * TYPE:	 HAL_MMUPageSize_t
 * DESCRIPTION:  Enumerated Type used to specify the MMU Page Size(SLSS)
 */
typedef enum HAL_MMUPageSize {
    HAL_MMU_SECTION,
    HAL_MMU_LARGE_PAGE,
    HAL_MMU_SMALL_PAGE,
    HAL_MMU_SUPERSECTION
} HAL_MMUPageSize_t;

/*
* FUNCTION	      : MMU_FlushEntry
*
* INPUTS:
*
*       Identifier      : baseAddress
*       Type		: const UWORD32
*       Description     : Base Address of instance of MMU module
*
* RETURNS:
*
*       Type		: HAL_STATUS
*       Description     : RET_OK		 -- No errors occured
*			 RET_BAD_NULL_PARAM     -- A Pointer
*						Paramater was set to NULL
*
* PURPOSE:	      : Flush the TLB entry pointed by the
*			lock counter register
*			even if this entry is set protected
*
* METHOD:	       : Check the Input parameter and Flush a
*			 single entry in the TLB.
*/
static HAL_STATUS MMU_FlushEntry(const UWORD32 baseAddress);

/*
* FUNCTION	      : MMU_SetCAMEntry
*
* INPUTS:
*
*       Identifier      : baseAddress
*       TypE		: const UWORD32
*       Description     : Base Address of instance of MMU module
*
*       Identifier      : pageSize
*       TypE		: const UWORD32
*       Description     : It indicates the page size
*
*       Identifier      : preservedBit
*       Type		: const UWORD32
*       Description     : It indicates the TLB entry is preserved entry
*							or not
*
*       Identifier      : validBit
*       Type		: const UWORD32
*       Description     : It indicates the TLB entry is valid entry or not
*
*
*       Identifier      : virtualAddrTag
*       Type	    	: const UWORD32
*       Description     : virtual Address
*
* RETURNS:
*
*       Type	    	: HAL_STATUS
*       Description     : RET_OK		 -- No errors occured
*			 RET_BAD_NULL_PARAM     -- A Pointer Paramater
*						   was set to NULL
*			 RET_PARAM_OUT_OF_RANGE -- Input Parameter out
*						   of Range
*
* PURPOSE:	      	: Set MMU_CAM reg
*
* METHOD:	       	: Check the Input parameters and set the CAM entry.
*/
static HAL_STATUS MMU_SetCAMEntry(const UWORD32    baseAddress,
				   const UWORD32    pageSize,
				   const UWORD32    preservedBit,
				   const UWORD32    validBit,
				   const UWORD32    virtualAddrTag);

/*
* FUNCTION	      : MMU_SetRAMEntry
*
* INPUTS:
*
*       Identifier      : baseAddress
*       Type	    	: const UWORD32
*       Description     : Base Address of instance of MMU module
*
*       Identifier      : physicalAddr
*       Type	    	: const UWORD32
*       Description     : Physical Address to which the corresponding
*			 virtual   Address shouldpoint
*
*       Identifier      : endianism
*       Type	    	: HAL_Endianism_t
*       Description     : endianism for the given page
*
*       Identifier      : elementSize
*       Type	    	: HAL_ElementSize_t
*       Description     : The element size ( 8,16, 32 or 64 bit)
*
*       Identifier      : mixedSize
*       Type	    	: HAL_MMUMixedSize_t
*       Description     : Element Size to follow CPU or TLB
*
* RETURNS:
*
*       Type	    	: HAL_STATUS
*       Description     : RET_OK		 -- No errors occured
*			 RET_BAD_NULL_PARAM     -- A Pointer Paramater
*							was set to NULL
*			 RET_PARAM_OUT_OF_RANGE -- Input Parameter
*							out of Range
*
* PURPOSE:	      : Set MMU_CAM reg
*
* METHOD:	       : Check the Input parameters and set the RAM entry.
*/
static HAL_STATUS MMU_SetRAMEntry(const UWORD32	baseAddress,
				   const UWORD32	physicalAddr,
				   HAL_Endianism_t      endianism,
				   HAL_ElementSize_t    elementSize,
				   HAL_MMUMixedSize_t   mixedSize);

/* =========================================================================
* HAL FUNCTIONS
* =========================================================================
*/

HAL_STATUS HAL_MMU_Enable(const UWORD32 baseAddress)
{
    HAL_STATUS status = RET_OK;

    MMUMMU_CNTLMMUEnableWrite32(baseAddress, HAL_SET);

    return status;
}

HAL_STATUS HAL_MMU_Disable(const UWORD32 baseAddress)
{
    HAL_STATUS status = RET_OK;

    MMUMMU_CNTLMMUEnableWrite32(baseAddress, HAL_CLEAR);

    return status;
}

HAL_STATUS HAL_MMU_NumLockedSet(const UWORD32 baseAddress,
				UWORD32 numLockedEntries)
{
    HAL_STATUS status = RET_OK;

    MMUMMU_LOCKBaseValueWrite32(baseAddress, numLockedEntries);

    return status;
}

HAL_STATUS HAL_MMU_VictimNumSet(const UWORD32 baseAddress,
				UWORD32 victimEntryNum)
{
    HAL_STATUS status = RET_OK;

    MMUMMU_LOCKCurrentVictimWrite32(baseAddress, victimEntryNum);

    return status;
}

HAL_STATUS HAL_MMU_TLBFlushAll(const UWORD32 baseAddress)
{
    HAL_STATUS status = RET_OK;

    MMUMMU_GFLUSHGlobalFlushWrite32(baseAddress, HAL_SET);

    return status;
}

HAL_STATUS HAL_MMU_EventAck(const UWORD32 baseAddress, UWORD32 irqMask)
{
    HAL_STATUS status = RET_OK;

    MMUMMU_IRQSTATUSWriteRegister32(baseAddress, irqMask);

    return status;
}

HAL_STATUS HAL_MMU_EventDisable(const UWORD32 baseAddress,
				UWORD32 irqMask)
{
    HAL_STATUS status = RET_OK;
    UWORD32 irqReg;

    irqReg = MMUMMU_IRQENABLEReadRegister32(baseAddress);

    MMUMMU_IRQENABLEWriteRegister32(baseAddress, irqReg & ~irqMask);

    return status;
}

HAL_STATUS HAL_MMU_EventEnable(const UWORD32 baseAddress, UWORD32 irqMask)
{
    HAL_STATUS status = RET_OK;
    UWORD32 irqReg;

    irqReg = MMUMMU_IRQENABLEReadRegister32(baseAddress);

    MMUMMU_IRQENABLEWriteRegister32(baseAddress, irqReg | irqMask);

    return status;
}


HAL_STATUS HAL_MMU_EventStatus(const UWORD32 baseAddress, UWORD32 *irqMask)
{
    HAL_STATUS status = RET_OK;

    *irqMask = MMUMMU_IRQSTATUSReadRegister32(baseAddress);

    return status;
}


HAL_STATUS HAL_MMU_FaultAddrRead(const UWORD32 baseAddress, UWORD32 *addr)
{
    HAL_STATUS status = RET_OK;

    /*Check the input Parameters*/
    CHECK_INPUT_PARAM(baseAddress, 0, RET_BAD_NULL_PARAM,
		      RES_MMU_BASE + RES_INVALID_INPUT_PARAM);

    /* read values from register */
    *addr = MMUMMU_FAULT_ADReadRegister32(baseAddress);

    return status;
}

HAL_STATUS HAL_MMU_TTBSet(const UWORD32 baseAddress, UWORD32 TTBPhysAddr)
{
    HAL_STATUS status = RET_OK;
    UWORD32 loadTTB;

   /*Check the input Parameters*/
   CHECK_INPUT_PARAM(baseAddress, 0, RET_BAD_NULL_PARAM,
		     RES_MMU_BASE + RES_INVALID_INPUT_PARAM);

   loadTTB = TTBPhysAddr & ~0x7FUL;
   /* write values to register */
   MMUMMU_TTBWriteRegister32(baseAddress, loadTTB);

   return status;
}

HAL_STATUS HAL_MMU_TWLEnable(const UWORD32 baseAddress)
{
    HAL_STATUS status = RET_OK;

    MMUMMU_CNTLTWLEnableWrite32(baseAddress, HAL_SET);

    return status;
}

HAL_STATUS HAL_MMU_TWLDisable(const UWORD32 baseAddress)
{
    HAL_STATUS status = RET_OK;

    MMUMMU_CNTLTWLEnableWrite32(baseAddress, HAL_CLEAR);

    return status;
}

HAL_STATUS HAL_MMU_TLBFlush(const UWORD32 baseAddress, UWORD32 virtualAddr,
			     UWORD32 pageSize)
{
    HAL_STATUS status = RET_OK;
    UWORD32 virtualAddrTag;
    HAL_MMUPageSize_t pgSizeBits;

    switch (pageSize) {
    case HAL_PAGE_SIZE_4KB:
	pgSizeBits = HAL_MMU_SMALL_PAGE;
	break;

    case HAL_PAGE_SIZE_64KB:
	pgSizeBits = HAL_MMU_LARGE_PAGE;
	break;

    case HAL_PAGE_SIZE_1MB:
	pgSizeBits = HAL_MMU_SECTION;
	break;

    case HAL_PAGE_SIZE_16MB:
	pgSizeBits = HAL_MMU_SUPERSECTION;
	break;

    default:
	return RET_FAIL;
    }

    /* Generate the 20-bit tag from virtual address */
    virtualAddrTag = ((virtualAddr & MMU_ADDR_MASK) >> 12);

    MMU_SetCAMEntry(baseAddress, pgSizeBits, 0, 0, virtualAddrTag);

    MMU_FlushEntry(baseAddress);

    return status;
}

HAL_STATUS HAL_MMU_TLBAdd(const UWORD32	baseAddress,
			   UWORD32	      physicalAddr,
			   UWORD32	      virtualAddr,
			   UWORD32	      pageSize,
			   UWORD32	      entryNum,
			   struct HAL_MMUMapAttrs_t    *mapAttrs,
			   HAL_SetClear_t       preservedBit,
			   HAL_SetClear_t       validBit)
{
    HAL_STATUS  status = RET_OK;
    UWORD32 lockReg;
    UWORD32 virtualAddrTag;
    HAL_MMUPageSize_t mmuPgSize;

    /*Check the input Parameters*/
    CHECK_INPUT_PARAM(baseAddress, 0, RET_BAD_NULL_PARAM,
		      RES_MMU_BASE + RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_RANGE_MIN0(pageSize, MMU_PAGE_MAX, RET_PARAM_OUT_OF_RANGE,
			   RES_MMU_BASE + RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_RANGE_MIN0(mapAttrs->elementSize, MMU_ELEMENTSIZE_MAX,
			RET_PARAM_OUT_OF_RANGE, RES_MMU_BASE +
			RES_INVALID_INPUT_PARAM);

    switch (pageSize) {
    case HAL_PAGE_SIZE_4KB:
	mmuPgSize = HAL_MMU_SMALL_PAGE;
	break;

    case HAL_PAGE_SIZE_64KB:
	mmuPgSize = HAL_MMU_LARGE_PAGE;
	break;

    case HAL_PAGE_SIZE_1MB:
	mmuPgSize = HAL_MMU_SECTION;
	break;

    case HAL_PAGE_SIZE_16MB:
	mmuPgSize = HAL_MMU_SUPERSECTION;
	break;

    default:
	return RET_FAIL;
    }

    lockReg = MMUMMU_LOCKReadRegister32(baseAddress);

    /* Generate the 20-bit tag from virtual address */
    virtualAddrTag = ((virtualAddr & MMU_ADDR_MASK) >> 12);

    /* Write the fields in the CAM Entry Register */
    MMU_SetCAMEntry(baseAddress,  mmuPgSize, preservedBit, validBit,
		    virtualAddrTag);

    /* Write the different fields of the RAM Entry Register */
    /* endianism of the page,Element Size of the page (8, 16, 32, 64 bit)*/
    MMU_SetRAMEntry(baseAddress, physicalAddr, mapAttrs->endianism,
		    mapAttrs->elementSize, mapAttrs->mixedSize);

    /* Update the MMU Lock Register */
    /* currentVictim between lockedBaseValue and (MMU_Entries_Number - 1)*/
    MMUMMU_LOCKCurrentVictimWrite32(baseAddress, entryNum);

    /* Enable loading of an entry in TLB by writing 1
	   into LD_TLB_REG register */
    MMUMMU_LD_TLBWriteRegister32(baseAddress, MMU_LOAD_TLB);


    MMUMMU_LOCKWriteRegister32(baseAddress, lockReg);

    return status;
}

HAL_STATUS HAL_MMU_PteSet(const UWORD32	pgTblVa,
			   UWORD32	      physicalAddr,
			   UWORD32	      virtualAddr,
			   UWORD32	      pageSize,
			   struct HAL_MMUMapAttrs_t    *mapAttrs)
{
    HAL_STATUS status = RET_OK;
    UWORD32 pteAddr, pteVal;
    WORD32 numEntries = 1;

    switch (pageSize) {
    case HAL_PAGE_SIZE_4KB:
	pteAddr = HAL_MMU_PteAddrL2(pgTblVa,
				    virtualAddr & MMU_SMALL_PAGE_MASK);
	pteVal = ((physicalAddr & MMU_SMALL_PAGE_MASK) |
		    (mapAttrs->endianism << 9) |
		    (mapAttrs->elementSize << 4) |
		    (mapAttrs->mixedSize << 11) | 2
		  );
	break;

    case HAL_PAGE_SIZE_64KB:
	numEntries = 16;
	pteAddr = HAL_MMU_PteAddrL2(pgTblVa,
				    virtualAddr & MMU_LARGE_PAGE_MASK);
	pteVal = ((physicalAddr & MMU_LARGE_PAGE_MASK) |
		    (mapAttrs->endianism << 9) |
		    (mapAttrs->elementSize << 4) |
		    (mapAttrs->mixedSize << 11) | 1
		  );
	break;

    case HAL_PAGE_SIZE_1MB:
	pteAddr = HAL_MMU_PteAddrL1(pgTblVa,
				    virtualAddr & MMU_SECTION_ADDR_MASK);
	pteVal = ((((physicalAddr & MMU_SECTION_ADDR_MASK) |
		     (mapAttrs->endianism << 15) |
		     (mapAttrs->elementSize << 10) |
		     (mapAttrs->mixedSize << 17)) &
		     ~0x40000) | 0x2
		 );
	break;

    case HAL_PAGE_SIZE_16MB:
	numEntries = 16;
	pteAddr = HAL_MMU_PteAddrL1(pgTblVa,
				    virtualAddr & MMU_SSECTION_ADDR_MASK);
	pteVal = (((physicalAddr & MMU_SSECTION_ADDR_MASK) |
		      (mapAttrs->endianism << 15) |
		      (mapAttrs->elementSize << 10) |
		      (mapAttrs->mixedSize << 17)
		    ) | 0x40000 | 0x2
		  );
	break;

    case HAL_MMU_COARSE_PAGE_SIZE:
	pteAddr = HAL_MMU_PteAddrL1(pgTblVa,
				    virtualAddr & MMU_SECTION_ADDR_MASK);
	pteVal = (physicalAddr & MMU_PAGE_TABLE_MASK) | 1;
	break;

    default:
	return RET_FAIL;
    }

    while (--numEntries >= 0)
	((UWORD32 *)pteAddr)[numEntries] = pteVal;

    return status;
}

HAL_STATUS HAL_MMU_PteClear(const UWORD32  pgTblVa,
			     UWORD32	virtualAddr,
			     UWORD32	pgSize)
{
    HAL_STATUS status = RET_OK;
    UWORD32 pteAddr;
    WORD32 numEntries = 1;

    switch (pgSize) {
    case HAL_PAGE_SIZE_4KB:
	pteAddr = HAL_MMU_PteAddrL2(pgTblVa,
				    virtualAddr & MMU_SMALL_PAGE_MASK);
	break;

    case HAL_PAGE_SIZE_64KB:
	numEntries = 16;
	pteAddr = HAL_MMU_PteAddrL2(pgTblVa,
				    virtualAddr & MMU_LARGE_PAGE_MASK);
	break;

    case HAL_PAGE_SIZE_1MB:
    case HAL_MMU_COARSE_PAGE_SIZE:
	pteAddr = HAL_MMU_PteAddrL1(pgTblVa,
				    virtualAddr & MMU_SECTION_ADDR_MASK);
	break;

    case HAL_PAGE_SIZE_16MB:
	numEntries = 16;
	pteAddr = HAL_MMU_PteAddrL1(pgTblVa,
				    virtualAddr & MMU_SSECTION_ADDR_MASK);
	break;

    default:
	return RET_FAIL;
    }

    while (--numEntries >= 0)
	((UWORD32 *)pteAddr)[numEntries] = 0;

    return status;
}

/* MMU_FlushEntry */
static HAL_STATUS MMU_FlushEntry(const UWORD32 baseAddress)
{
   HAL_STATUS status = RET_OK;
   UWORD32 flushEntryData = 0x1;

   /*Check the input Parameters*/
   CHECK_INPUT_PARAM(baseAddress, 0, RET_BAD_NULL_PARAM,
		     RES_MMU_BASE + RES_INVALID_INPUT_PARAM);

   /* write values to register */
   MMUMMU_FLUSH_ENTRYWriteRegister32(baseAddress, flushEntryData);

   return status;
}

/* MMU_SetCAMEntry */
static HAL_STATUS MMU_SetCAMEntry(const UWORD32    baseAddress,
				   const UWORD32    pageSize,
				   const UWORD32    preservedBit,
				   const UWORD32    validBit,
				   const UWORD32    virtualAddrTag)
{
   HAL_STATUS status = RET_OK;
   UWORD32 mmuCamReg;

   /*Check the input Parameters*/
   CHECK_INPUT_PARAM(baseAddress, 0, RET_BAD_NULL_PARAM,
		     RES_MMU_BASE + RES_INVALID_INPUT_PARAM);

   mmuCamReg = (virtualAddrTag << 12);
   mmuCamReg = (mmuCamReg) | (pageSize) |  (validBit << 2) |
	       (preservedBit << 3) ;

   /* write values to register */
   MMUMMU_CAMWriteRegister32(baseAddress, mmuCamReg);

   return status;
}

/* MMU_SetRAMEntry */
static HAL_STATUS MMU_SetRAMEntry(const UWORD32       baseAddress,
				   const UWORD32       physicalAddr,
				   HAL_Endianism_t     endianism,
				   HAL_ElementSize_t   elementSize,
				   HAL_MMUMixedSize_t  mixedSize)
{
   HAL_STATUS status = RET_OK;
   UWORD32 mmuRamReg;

   /*Check the input Parameters*/
   CHECK_INPUT_PARAM(baseAddress, 0, RET_BAD_NULL_PARAM,
		     RES_MMU_BASE + RES_INVALID_INPUT_PARAM);
   CHECK_INPUT_RANGE_MIN0(elementSize, MMU_ELEMENTSIZE_MAX,
		   RET_PARAM_OUT_OF_RANGE, RES_MMU_BASE +
		   RES_INVALID_INPUT_PARAM);


   mmuRamReg = (physicalAddr & MMU_ADDR_MASK);
   mmuRamReg = (mmuRamReg) | ((endianism << 9) |  (elementSize << 7) |
	       (mixedSize << 6));

   /* write values to register */
   MMUMMU_RAMWriteRegister32(baseAddress, mmuRamReg);

   return status;

}
