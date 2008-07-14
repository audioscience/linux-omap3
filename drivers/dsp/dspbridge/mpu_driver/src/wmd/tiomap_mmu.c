/*
 * dspbridge/src/wmd/linux/omap/2430/tiomap_mmu.c
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
 *  ======== _tiomap_mmu.c ========
 *  Description:
 *      Implementation of the  DSP MMU modules.
 *
 *! Revision History
 *! ================
 *! 19-Apr-2004 sb:  configureDspMmu calls HW_MMU_TLBAdd directly now.
 *! 05-Jan-2004 vp:  Moved to platform specific folder and updated for the 24xx
 *! 					platform
 *! 21-Mar-2003 sb:  Support 64 KB DSP MMU entries
 *! 19-Feb-2003 vp:  Ported to Linux platform.
 *! 08-Oct-2002 rr:  Created.
 */
#include <host_os.h>
/*  ----------------------------------- DSP/BIOS Bridge */
#include <dbdefs.h>
#include <errbase.h>
#include <cfgdefs.h>
/*  ----------------------------------- Trace & Debug */
#include <dbc.h>
#include <dbg.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <mem.h>
#include <util.h>
#include <cfg.h>
#include <drv.h>

/* ------------------------------------ Hardware Abstraction Layer */
#include <hw_defs.h>
#include <hw_mmu.h>

/*  ----------------------------------- Local Headers */
#include "_tiomap.h"
#include "_tiomap_mmu.h"

/*
 *  ======== configureDspMmu ========
 *  Purpose:
 *      Make DSP MMU page table entries.
 */
void configureDspMmu(struct WMD_DEV_CONTEXT *pDevContext, DWORD dataBasePhys,
		    DWORD dspBaseVirt, DWORD sizeInBytes, INT nEntryStart,
		    HW_Endianism_t endianism, HW_ElementSize_t elemSize,
		    HW_MMUMixedSize_t mixedSize)
{
	struct CFG_HOSTRES resources;
	struct HW_MMUMapAttrs_t mapAttrs = { endianism, elemSize, mixedSize };
	DSP_STATUS status = DSP_SOK;

	DBC_Require(sizeInBytes > 0);
	DBG_Trace(DBG_LEVEL1,
		 "configureDspMmu entry %x pa %x, va %x, bytes %x ",
		 nEntryStart, dataBasePhys, dspBaseVirt, sizeInBytes);

	DBG_Trace(DBG_LEVEL1, "endianism %x, elemSize %x, mixedSize %x\n",
		 endianism, elemSize, mixedSize);
	status = CFG_GetHostResources(
		 (struct CFG_DEVNODE *)DRV_GetFirstDevExtension(), &resources);
	status = HW_MMU_TLBAdd(pDevContext->dwDSPMmuBase, dataBasePhys,
				dspBaseVirt, sizeInBytes, nEntryStart,
				&mapAttrs, HW_SET, HW_SET);
}
