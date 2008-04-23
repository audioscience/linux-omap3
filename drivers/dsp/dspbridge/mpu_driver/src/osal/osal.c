/*
 * dspbridge/src/osal/linux/osal.c
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
 *  ======== osalce.c ========
 *  Purpose:
 *      Provide OSAL loading.
 *
 *  Public Functions:
 *      OSAL_Exit
 *      OSAL_Init
 *
 *! Revision History
 *! ================
 *! 20-Nov-2000 rr: NTFY_Init/Exit added.
 *! 06-Jul-2000 rr: PROC prefix changed to PRCS to accomodate RM.
 *! 01-Feb-2000 kc: Created.
 */

#include <host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <std.h>
#include <dbdefs.h>

/*  ----------------------------------- Trace & Debug */
#include <dbc.h>
#include <dbg_zones.h>
#include <gt.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <cfg.h>
#include <csl.h>
#include <dbg.h>
#include <dpc.h>
#include <isr.h>
#include <kfile.h>
#ifndef LINUX
#include <ldr.h>
#endif
#include <list.h>
#include <mem.h>
#include <ntfy.h>
#ifdef PERF
#include <perf.h>
#endif
#include <prcs.h>
#include <reg.h>
#include <sync.h>
#include <clk.h>
#include <util.h>

/*  ----------------------------------- This */
#include <osal.h>

/*  ----------------------------------- Globals */
#if GT_TRACE
static struct GT_Mask OSAL_debugMask = { 0, 0 };	/* GT trace var. */
#endif

static ULONG cRefs = 0;		/* OSAL module reference count */

/*
 *  ======== OSAL_Exit ========
 *  Purpose:
 *      Discontinue usage of module; free resources when reference count
 *      reaches 0.
 */
VOID OSAL_Exit()
{
	DBC_Require(cRefs > 0);

	GT_1trace(OSAL_debugMask, GT_5CLASS, "OSAL_Exit: cRefs 0x%x\n", cRefs);

	cRefs--;
	if (cRefs == 0) {
		/* Uninitialize all OSAL modules here */
		NTFY_Exit();
		UTIL_Exit();
		SYNC_Exit();
		CLK_Exit();
		REG_Exit();
		PRCS_Exit();
#ifdef PERF
		PERF_Exit();
#endif
		LST_Exit();
#ifndef LINUX
		LDR_Exit();
#endif
		KFILE_Exit();
		ISR_Exit();
		DPC_Exit();
		DBG_Exit();
		CSL_Exit();
		CFG_Exit();
		MEM_Exit();

		GT_exit();
	}

	DBC_Ensure(cRefs >= 0);
}

/*
 *  ======== OSAL_Init ========
 *  Purpose:
 *      Initializes OSAL modules.
 */
BOOL OSAL_Init()
{
	BOOL fInit = TRUE;
	BOOL fCFG, fCSL, fDBG, fDPC, fISR, fKFILE, fLST, fMEM;
	BOOL fPRCS, fREG, fSYNC, fCLK, fUTIL, fNTFY;
#ifndef LINUX
	BOOL fLDR;
#endif
#ifdef PERF
	BOOL fPERF
#endif
	 DBC_Require(cRefs >= 0);

	if (cRefs == 0) {

		GT_init();
		GT_create(&OSAL_debugMask, "OS");	/* OS for OSal */

		GT_0trace(OSAL_debugMask, GT_ENTER, "OSAL_Init: entered\n");

		/* Perform required initialization of OSAL modules. */
		fMEM = MEM_Init();
		fREG = REG_Init();
		fCFG = CFG_Init();
		fCSL = CSL_Init();
		fDBG = DBG_Init();
		fDPC = DPC_Init();
		fISR = ISR_Init();
		fKFILE = KFILE_Init();
#ifndef LINUX
		fLDR = LDR_Init();
#endif
		fLST = LST_Init();
#ifdef PERF
		fPERF = PERF_Init();
#endif
		fPRCS = PRCS_Init();
		/* fREG = REG_Init(); */
		fSYNC = SYNC_Init();
		fCLK  = CLK_Init();
		fUTIL = UTIL_Init();
		fNTFY = NTFY_Init();

		fInit = fCFG && fCSL && fDBG && fDPC && fISR && fKFILE &&
			fLST && fMEM && fPRCS && fREG && fSYNC && fCLK && fUTIL;
#ifndef LINUX
		fInit = fInit && fLDR;
#endif
#ifdef PERF
		fInit = fInit && fPERF;
#endif

		if (!fInit) {
			if (fNTFY)
				NTFY_Exit();

			if (fUTIL)
				UTIL_Exit();

			if (fSYNC)
				SYNC_Exit();

			if (fCLK)
				CLK_Exit();

			if (fREG)
				REG_Exit();

			if (fPRCS)
				PRCS_Exit();

#ifdef PERF
			if (fPERF)
				PERF_Exit();

#endif
			if (fLST)
				LST_Exit();

#ifndef LINUX
			if (fLDR)
				LDR_Exit();

#endif
			if (fKFILE)
				KFILE_Exit();

			if (fISR)
				ISR_Exit();

			if (fDPC)
				DPC_Exit();

			if (fDBG)
				DBG_Exit();

			if (fCSL)
				CSL_Exit();

			if (fCFG)
				CFG_Exit();

			if (fMEM)
				MEM_Exit();

		}
	}

	if (fInit)
		cRefs++;

	GT_1trace(OSAL_debugMask, GT_5CLASS, "OSAL_Init: cRefs 0x%x\n", cRefs);

	DBC_Ensure((fInit && (cRefs > 0)) || (!fInit && (cRefs >= 0)));

	return (fInit);
}

