/*
 * dspbridge/inc/perf.h
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
 *  ======== perf.h ========
 *  Purpose:
 *      Performance statistics monitoring definitions.
 *
 *  Public Functions:
#ifdef PERF
 *      PERF_DeregisterModule
 *      PERF_DeregisterStat
 *      PERF_Exit
 *      PERF_GetPerfData
 *      PERF_Init
 *      PERF_RegisterModule
 *      PERF_RegisterStat
#endif
 *
 *! Revision History:
 *! ================
 *! 22-Nov-2000 kc: Added PERF_GetPerfData and modified PERF_RegisterStat;
 *!                 moved PERF specific defs into "perfdefs.h"
 *! 11-Dec-1996 gp: Added PERF_MAXNAMELEN.
 *! 12-Sep-1996 gp: Created.
 */

#ifndef PERF_
#define PERF_

#ifdef PERF

#include <wcdioctl.h>
#include <perfdefs.h>

#ifdef __cplusplus
extern "C" {
#endif

	typedef HANDLE PERF_HMOD;
	struct PERF_STAT;
	/*typedef struct PERF_STAT *PERF_HSTAT;*/
	struct PERF_MANAGER;
	/*typedef struct PERF_MANAGER *PERF_HMANAGER;*/

/*
 *  ======== PERF_DeregisterModule ========
 *  Purpose:
 *      Deregister previously registered module (server).
 *  Parameters:
 *      hModule:    Handle to a module registered with PERF.
 *  Returns:
 *  Requires:
 *  Ensures:
 *      This module's statistics will no longer be accessed by the system.
 */
	extern VOID PERF_DeregisterModule(IN PERF_HMOD hModule);

/*
 *  ======== PERF_DeregisterStat ========
 *  Purpose:
 *      Unregister the statistic with the PERF module.
 *  Parameters:
 *      hStat:  Handle returned by PERF_RegisterStat().
 *  Returns:
 *  Requires:
 *  Ensures:
 *      No further accesses to the statistic variable will occur, if this
 *      statistic was not registered more than once.
 */
	extern VOID PERF_DeregisterStat(IN struct PERF_STAT *hStat);

/*
 *  ======== PERF_Exit ========
 *  Purpose:
 *      Discontinue usage of the PERF module.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      PERF_Init() was previously called.
 *  Ensures:
 *      Resources acquired in PERF_Init(), if any, are freed.
 */
	extern VOID PERF_Exit();

/*
 *  ======== PERF_GetPerfData ========
 *  Purpose:
 *      Get perf stats.
 *  Parameters:
 *      pMsg: Pointer to msg structure.
 *  Returns:
 *      TRUE: success.
 *      FALSE: failure.
 *  Requires:
 *  Ensures:
 */
	extern BOOL PERF_GetPerfData(IN PVOID pMsg);

/*
 *  ======== PERF_Init ========
 *  Purpose:
 *      On first usage, at reference count == 0, checks the BRIDGE Config key
 *      in the registry for a "Perf" value.
 *      If the "Perf" key is not found, or is 0, performance statistics
 *      registering will be disabled.  This is the default, since this "Perf"
 *      key is not created by the BRIDGE Run Time installation program.
 *  Parameters:
 *  Returns:
 *      TRUE if the operating system offers the requisite supporting performance
 *      statistics monitoring services; FALSE otherwise.
 *  Requires:
 *  Ensures:
 *      A requirement for each of the other public PERF functions.
 */
	extern BOOL PERF_Init();

/*
 *  ======== PERF_RegisterModule ========
 *  Purpose:
 *      Register a module (server) with the performance monitor.
 *  Parameters:
 *      pstrServerName: Name of the module, as displayed by a Windows GUI. Must
 *                      not exceed PERF_MAXNAMELEN characters.
 *      pstrRegName:    Name to use as registry key for this module. Must
 *                      not exceed PERF_MAXNAMELEN characters.
 *  Returns:
 *      A handle to use in subsequent calls to add statistics for this module;
 *      NULL if failure.
 *  Requires:
 *  Ensures:
 */
	extern PERF_HMOD PERF_RegisterModule(IN PSTR pstrServerName,
					     IN PSTR pstrRegName);

/*
 *  ======== PERF_RegisterStat ========
 *  Purpose:
 *      Register a particular statistic (variable) of a given module to be
 *      peeked at periodic intervals by the PERF module.
 *  Parameters:
 *      hModule:        Handle to a module registered with PERF.
 *      dwType:         Type of statistic.  One of:
 *                      PERF_STATCOUNT:  GUI displays absolute value of stat..
 *                      PERF_STATRATE:   GUI computes rate in value/sec.
 *      dwId:           Pre-allocated statistic identifier.
 *      pdwData:        Pointer to a DWORD containing the statistic.
 *  Returns:
 *      A handle to a performance stat registered with PERF, or NULL if failure.
 *  Requires:
 *      The data variable holding the statistic must be in memory at all times
 *      until the statistic is deregistered.
 *  Ensures:
 */
	extern struct PERF_STAT *PERF_RegisterStat(IN PERF_HMOD hModule,
					    IN DWORD dwType,
					    IN DWORD dwId, OUT DWORD *pdwData);

/*
 *  ======== PERF_RequestHandler ========
 *  Purpose:
 *      Handle a request for performance stats.
 *  Parameters:
 *      pulMemAllocations:
 *  Returns:
 *      A handle to use in subsequent calls to add statistics for this module;
 *      NULL if failure.
 *  Requires:
 *  Ensures:
 */
	extern DWORD PERF_RequestHandler(IN Trapped_Args * args);

#ifdef __cplusplus
}
#endif
#endif				/* PERF */
#endif				/* PERF_ */
