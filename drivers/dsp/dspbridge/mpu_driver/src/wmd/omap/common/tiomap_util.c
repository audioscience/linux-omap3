/*
 * dspbridge/src/wmd/linux/omap/common/tiomap_util.c
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
 *  ======== _tiomap_util.c ========
 *  Description:
 *      Implementation for the utility routines.
 *
 *! Revision History
 *! ================
 *! 05-Jan-2004 vp:  Updated for 24xx platform.
 *! 08-Oct-2002 rr:  Created.
 */

/*  ----------------------------------- DSP/BIOS Bridge */
#include <dbdefs.h>
#include <errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <dbg.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <mem.h>
#include <util.h>

/*  ----------------------------------- Local */
#include "_tiomap.h"
#include "_tiomap_api.h"
#include "_tiomap_util.h"

/*
 *  ======== WaitForStart ========
 *  purpose:
 *      Wait for the singal from DSP that it has started, or time out.
 */
BOOL WaitForStart(struct WMD_DEV_CONTEXT *pDevContext, DWORD dwSyncAddr)
{
	USHORT usCount = TIHELEN_ACKTIMEOUT;

	/*  Wait for response from board */
	while (*((volatile WORD *)dwSyncAddr) && --usCount)
		UTIL_Wait(TIHELEN_WRITE_DELAY);

	/*  If timed out: return FALSE */
	if (!usCount) {
		DBG_Trace(DBG_LEVEL7, "Timed out Waiting for DSP to Start\n");
		return (FALSE);
	}
	return (TRUE);
}

