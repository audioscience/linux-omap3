/*
 * dspbridge/src/wmd/linux/omap/2430/hal_mmu.c
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
 *  ======== hal_mmu.c ========
 *  Purpose:
 *      Implements lower edge MMU functions.
 *
 *! Revision History:
 *! ================
 *! 19-Apr-2004 sb  Implemented HAL_DspMmuFlushTlbEntry & HAL API updates
 *! 16-Feb-2004 vp  Fixed a compiler warning in DspMmuIntrClear function.
 *! 05-Jan-2004 vp  Updated for the 24xx HAL Library and moved to platform
 *!		    specific folder.
 *! 19-Feb-2003 vp  Code Review Updates.
 *! 16-Dec-2002 map  Added HAL_DspMmuFlushTlbEntry
 *! 18-Oct-2002 sb  Ported to Linux platform.
 *! 21-Sep-2001 kc  created.
 */

/*  ----------------------------------- Host OS */
#include <host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <std.h>
#include <dbdefs.h>
#include <cfgdefs.h>

/*  ----------------------------------- Trace & Debug */
#include <dbc.h>
#include <dbg.h>
#include <dbg_zones.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <cfg.h>
#include <drv.h>

/* ------------------------------------ Hardware Abstraction Layer */
#include <hal_defs.h>
#include <hal_mmu.h>

/*  ----------------------------------- Link Driver */
#include "_tiomap.h"
#include "_tiomap_mmu.h"

/*  ----------------------------------- This */
#include "_hal_mmu.h"

/* ========= HAL_DspMmuFlushTlbEntry ========
 * purpose:
 *     Flushes an MMU entry.
 */
VOID HAL_DspMmuFlushTlbEntry(IN struct WMD_DEV_CONTEXT *pDevContext,
			    IN ULONG ulVirAddr, IN ULONG pageSize)
{
	struct CFG_HOSTRES resources;
	DSP_STATUS status;
	status = CFG_GetHostResources(
			(struct CFG_DEVNODE *)DRV_GetFirstDevExtension(),
			&resources);
	if (DSP_SUCCEEDED(status)) {
		HAL_MMU_TLBFlush(resources.dwDmmuBase, ulVirAddr &
		 ~(pageSize - 1), pageSize);
	}
}

/* ======== HAL_DspMmuIntrClear ========
 * purpose:
 *     Makes an MMU entry, Acknowledges DSP and removes the MMU entry.
 *     TODO:The below implementation needs to be changed if WTL is enabled.
 */
VOID DspMmuIntrClear(IN struct WMD_DEV_CONTEXT *pDevContext,
		    struct DSP_ERRORINFO *pErrInfo)
{
	/* Fixed the compiler warning by initializing to zero */
	DWORD newEntry = 0;
	DWORD faultAddr;
	UWORD32 dmmuEventMask;
	struct CFG_HOSTRES resources;
	DSP_STATUS status;

	DBC_Require(pDevContext);
	DBC_Require(pErrInfo);
	DBG_Trace(DBG_LEVEL1, "Entering DspMmuIntrClear: 0x%x, 0x%x\n",
		 pDevContext, pErrInfo);
	status = CFG_GetHostResources(
			(struct CFG_DEVNODE *)DRV_GetFirstDevExtension(),
			&resources);
	if (!DSP_SUCCEEDED(status))
		DBG_Trace(DBG_LEVEL7, "** Failed to get Host resources in "
			 "mmu intr clear** \n");
	/* Read DSP MMU fault status register */
	HAL_MMU_FaultAddrRead(resources.dwDmmuBase, &faultAddr);
	if (faultAddr != 0) {
		struct HAL_MMUMapAttrs_t mapAttrs = {HAL_BIG_ENDIAN,
						    HAL_ELEM_SIZE_16BIT,
						    HAL_MMU_CPUES };
		/* Align on 4KB byte boundary */
		newEntry = faultAddr & 0xFFFFF000;
		/*  MMU entry set to avoid DSP side CCS hang up Physical
		 *  address: SHM. Writes to arbitraty address on host OS can
		 *  be fatal. */
		/*  Virtual address:  includes fault address */
		HAL_MMU_TLBAdd(resources.dwDmmuBase, pDevContext->
			      aTLBEntry[0].ulGppPa, newEntry, HAL_PAGE_SIZE_4KB,
			      31, &mapAttrs, HAL_CLEAR, HAL_CLEAR);
	}
	dmmuEventMask = HAL_MMU_TLB_MISS;
	/* Acknowledge DSP MMU fault */
	HAL_MMU_EventAck(resources.dwDmmuBase, dmmuEventMask);
	/*  Remove TLB entry to trap subsequent MMU Faults */
	if (faultAddr != 0) {
		ULONG delay = 1000;
		/* Let DSP re-execute the faulting instruction
		 * before removing the entry */
		for (; delay--; ) ;

		HAL_MMU_TLBFlush(resources.dwDmmuBase, newEntry,
				 HAL_PAGE_SIZE_4KB);

	}
	pErrInfo->dwVal1 = (faultAddr & 0xFFFF0000) >> 16;
	pErrInfo->dwVal2 = faultAddr & 0xFFFFUL;
	pErrInfo->dwVal3 = 0;

	DBG_Trace(DBG_LEVEL1, "HAL_DspMmuIntrClear: 0x%x, 0x%x, 0x%x\n",
		 pErrInfo->dwVal1, pErrInfo->dwVal2, pErrInfo->dwVal3);
	DBG_Trace(DBG_LEVEL1, "Exiting HAL_DspMmuIntrClear.\n");
}

