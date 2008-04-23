/*
 * dspbridge/src/pmgr/linux/common/chnl.c
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
 *  ======== chnl.c ========
 *  Description:
 *      WCD channel interface: multiplexes data streams through the single
 *      physical link managed by a 'Bridge mini-driver.
 *
 *  Public Functions:
 *      CHNL_AddIOReq
 *      CHNL_AllocBuffer
 *      CHNL_CancelIO
 *      CHNL_Close
 *      CHNL_CloseOrphans
 *      CHNL_Create
 *      CHNL_Destroy
 *      CHNL_Exit
 *      CHNL_FlushIO
 *      CHNL_FreeBuffer
 *      CHNL_GetEventHandle
 *      CHNL_GetHandle
 *      CHNL_GetIOCompletion
 *      CHNL_GetId
 *      CHNL_GetMgr
 *      CHNL_GetMode
 *      CHNL_GetPosition
 *      CHNL_GetProcessHandle
 *      CHNL_Init
 *      CHNL_Open
 *
 *  Notes:
 *      This interface is basically a pass through to the WMD CHNL functions,
 *      except for the CHNL_Get() accessor functions which call
 *      WMD_CHNL_GetInfo().
 *
 *! Revision History:
 *! ================
 *! 24-Feb-2003 swa PMGR Code review comments incorporated.
 *! 07-Jan-2002 ag  CHNL_CloseOrphans() now closes supported # of channels.
 *! 17-Nov-2000 jeh Removed IRQ, shared memory stuff from CHNL_Create.
 *! 28-Feb-2000 rr: New GT USage Implementation
 *! 03-Feb-2000 rr: GT and Module init/exit Changes.(Done up front from OSAL)
 *! 21-Jan-2000 ag: Added code review comments.
 *! 13-Jan-2000 rr: CFG_Get/SetPrivateDword renamed to CFG_Get/SetDevObject.
 *! 08-Dec-1999 ag: CHNL_[Alloc|Free]Buffer bufs taken from client process heap.
 *! 02-Dec-1999 ag: Implemented CHNL_GetEventHandle().
 *! 17-Nov-1999 ag: CHNL_AllocBuffer() allocs extra word for process mapping.
 *! 28-Oct-1999 ag: WinCE port. Search for "WinCE" for changes(TBR).
 *! 07-Jan-1998 gp: CHNL_[Alloc|Free]Buffer now call MEM_UMB functions.
 *! 22-Oct-1997 gp: Removed requirement in CHNL_Open that hReserved1 != NULL.
 *! 30-Aug-1997 cr: Renamed cfg.h wbwcd.h b/c of WINNT file name collision.
 *! 10-Mar-1997 gp: Added GT trace.
 *! 14-Jan-1997 gp: Updated based on code review feedback.
 *! 03-Jan-1997 gp: Moved CHNL_AllocBuffer/CHNL_FreeBuffer code from udspsys.
 *! 14-Dec-1996 gp: Added uChnlId parameter to CHNL_Open().
 *! 09-Sep-1996 gp: Added CHNL_GetProcessHandle().
 *! 15-Jul-1996 gp: Created.
 */

/*  ----------------------------------- Host OS */
#include <host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <std.h>
#include <dbdefs.h>
#include <errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <dbc.h>
#include <dbg_zones.h>
#include <gt.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <cfg.h>
#include <csl.h>
#include <dpc.h>
#include <isr.h>
#include <list.h>
#include <mem.h>
#include <perf.h>
#include <sync.h>

/*  ----------------------------------- Platform Manager */
#include <proc.h>
#include <dev.h>

/*  ----------------------------------- Others */
#include <chnlpriv.h>
#include <chnlobj.h>

/*  ----------------------------------- This */
#include <chnl.h>

/*  ----------------------------------- Globals */
static ULONG cRefs = 0;
#if GT_TRACE
static struct GT_Mask CHNL_DebugMask = { 0, 0 };	/* WCD CHNL Mask */
#endif

/*  ----------------------------------- Function Prototypes */
static DSP_STATUS GetNumOpenChannels(struct CHNL_MGR *hChnlMgr,
				    OUT ULONG *pcOpenChannels);

static DSP_STATUS GetNumChannels(struct CHNL_MGR *hChnlMgr,
				 OUT ULONG *pcChannels);

#ifdef DEAD_CODE

/*
 *  ======== CHNL_AddIOReq ========
 *  Purpose:
 *      Enqueue an I/O request for data transfer with the DSP on this channel.
 *      The direction (mode) is specified in the channel object.
 */
DSP_STATUS CHNL_AddIOReq(struct CHNL_OBJECT *hChnl, PVOID pHostBuf,
			 ULONG cBytes)
{
	DWORD dwArg = 0;
	DSP_STATUS status;
	struct CHNL_OBJECT_ *pChnl = (struct CHNL_OBJECT_ *)hChnl;
	struct WMD_DRV_INTERFACE *pIntfFxns;

	DBC_Require(cRefs > 0);

	GT_3trace(CHNL_DebugMask, GT_ENTER,
		  "Entered CHNL_AddIOReq: hChnl: 0x%x\t"
		  "pHostBuf: 0x%x\tcBytes: 0x%x\n", hChnl, pHostBuf, cBytes);
	if (CHNL_IsValidChnl(pChnl)) {
		pIntfFxns = pChnl->pChnlMgr->pIntfFxns;
		status = (*pIntfFxns->pfnChnlAddIOReq)
		(hChnl, pHostBuf, cBytes, 0, 0L, dwArg);
	} else {
		status = DSP_EHANDLE;
		GT_0trace(CHNL_DebugMask, GT_7CLASS,
			  "CHNL_AddIOReq:Invalid Handle\n");
	}

	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Exiting CHNL_AddIOReq: hChnl: 0x%x, "
		  "status: 0x%x\n", hChnl, status);

	return (status);
}

/*
 *  ======== CHNL_AllocBuffer ========
 *  Purpose:
 *      Channel buffer allocation may depend on the type of channel driver
 *      we're using.
 */
DSP_STATUS CHNL_AllocBuffer(OUT PVOID *ppBuf, struct CHNL_MGR *hChnlMgr,
			    ULONG cBytes)
{
	DSP_STATUS status = DSP_SOK;
/*    CHNL_MGR_       * pChnlMgr = (CHNL_MGR_ *)hChnlMgr;*/

	DBC_Require(cRefs > 0);

	GT_3trace(CHNL_DebugMask, GT_ENTER,
		  "Entered CHNL_AddIOReq: ppBuf: 0x%x\t"
		  "hChnlMgr: 0x%x\tcBytes: 0x%x\n", ppBuf, hChnlMgr, cBytes);
	if (ppBuf != NULL) {
		if (CHNL_IsValidMgr((struct CHNL_MGR_ *)hChnlMgr)) {
			if (cBytes <= 0) {
				status = DSP_EINVALIDARG;
			}
		} else {
			status = DSP_EHANDLE;
			GT_0trace(CHNL_DebugMask, GT_7CLASS,
				  "CHNL_AllocBuffer: Invalid "
				  "Handle\n");
		}
	} else {
		status = DSP_EPOINTER;
	}
	return (status);
}

/*
 *  ======== CHNL_CancelIO ========
 *  Purpose:
 *      Return all I/O requests to the client which have not yet been
 *      transferred.  The channel's I/O completion object is
 *      signalled, and all the I/O requests are queued as IOC's, with the
 *      status field set to CHNL_IOCSTATCANCEL.
 *      This call is typically used in abort situations, and is a prelude to
 *      CHNL_Close();
 */
DSP_STATUS CHNL_CancelIO(struct CHNL_OBJECT *hChnl)
{
	DSP_STATUS status;
	struct CHNL_OBJECT_ *pChnl = (struct CHNL_OBJECT_ *)hChnl;
	struct WMD_DRV_INTERFACE *pIntfFxns;

	DBC_Require(cRefs > 0);

	GT_1trace(CHNL_DebugMask, GT_ENTER,
		  "Entered CHNL_CancelIO: hChnl: 0x%x\n", hChnl);
	if (CHNL_IsValidChnl(pChnl)) {
		pIntfFxns = pChnl->pChnlMgr->pIntfFxns;
		status = (*pIntfFxns->pfnChnlCancelIO) (hChnl);
	} else {
		GT_0trace(CHNL_DebugMask, GT_7CLASS,
			  "CHNL_CancelIO:Invalid Handle\n");
		status = DSP_EHANDLE;
	}
	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Exiting CHNL_CancelIO: hChnl: 0x%x, "
		  "status: 0x%x\n", hChnl, status);
	return (status);
}

#endif

/*
 *  ======== CHNL_Close ========
 *  Purpose:
 *      Ensures all pending I/O on this channel is cancelled, discards all
 *      queued I/O completion notifications, then frees the resources
 *      allocated for this channel, and makes the corresponding logical
 *      channel id available for subsequent use.
 */
DSP_STATUS CHNL_Close(struct CHNL_OBJECT *hChnl)
{
	DSP_STATUS status;
	struct CHNL_OBJECT_ *pChnl = (struct CHNL_OBJECT_ *)hChnl;
	struct WMD_DRV_INTERFACE *pIntfFxns;

	DBC_Require(cRefs > 0);

	GT_1trace(CHNL_DebugMask, GT_ENTER,
		  "Entered CHNL_Close:hChnl: 0x%x\n",
		  hChnl);

	if (CHNL_IsValidChnl(pChnl)) {
		pIntfFxns = pChnl->pChnlMgr->pIntfFxns;
		status = (*pIntfFxns->pfnChnlClose) (hChnl);
	} else {
		GT_0trace(CHNL_DebugMask, GT_7CLASS,
			  "CHNL_Close:Invalid Handle\n");
		status = DSP_EHANDLE;
	}
	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Exiting CHNL_Close:hChnl: 0x%x, status:"
		  " 0x%x\n", hChnl, status);
	return (status);
}

/*
 *  ======== CHNL_CloseOrphans ========
 *  Purpose:
 *      Close open channels orphaned by a closing process.
 */
DSP_STATUS CHNL_CloseOrphans(struct CHNL_MGR *hChnlMgr, HANDLE hProcess)
{
	ULONG uChnlID;
	DSP_STATUS status = DSP_SFALSE;
	HANDLE hProc;
	ULONG cOpenChannels;
	ULONG cTotalChnls;
	struct CHNL_OBJECT *hChnl;

	DBC_Require(cRefs > 0);

	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Enter CHNL_CloseOrphans hChnlMgr "
		  "0x%x\t\nhProcess: 0x%x\n", hChnlMgr, hProcess);
	if (!CHNL_IsValidMgr((struct CHNL_MGR_ *)hChnlMgr)) {
		status = DSP_EHANDLE;
		goto func_end;
	}
	if (DSP_SUCCEEDED(GetNumOpenChannels(hChnlMgr, &cOpenChannels)) &&
			 (cOpenChannels > 0)) {
		if (!DSP_SUCCEEDED(GetNumChannels(hChnlMgr, &cTotalChnls)))
			goto func_end;

		/* For each channel (except for RMS), get process handle: */
		for (uChnlID = 2; uChnlID < cTotalChnls; uChnlID++) {
			if (!DSP_SUCCEEDED(CHNL_GetHandle(hChnlMgr, uChnlID,
			    &hChnl))) {
				continue;
			}
			if (!DSP_SUCCEEDED(CHNL_GetProcessHandle(hChnl,
			    &hProc))) {
				continue;
			}
			/* See if channel owned by this process: */
			if (hProc == hProcess) {
				/* If so, close it now. */
				CHNL_Close(hChnl);
				status = DSP_SOK;
			}
		}
	}
func_end:
	GT_1trace(CHNL_DebugMask, GT_ENTER, "CHNL_CloseOrphans status 0x%x\n",
		  status);

	return (status);
}

/*
 *  ======== CHNL_Create ========
 *  Purpose:
 *      Create a channel manager object, responsible for opening new channels
 *      and closing old ones for a given 'Bridge board.
 */
DSP_STATUS CHNL_Create(OUT struct CHNL_MGR **phChnlMgr,
		       struct DEV_OBJECT *hDevObject,
		       IN CONST struct CHNL_MGRATTRS *pMgrAttrs)
{
	DSP_STATUS status;
	struct CHNL_MGR *hChnlMgr;
	struct CHNL_MGR_ *pChnlMgr = NULL;

	DBC_Require(cRefs > 0);
	DBC_Require(phChnlMgr != NULL);
	DBC_Require(pMgrAttrs != NULL);

	GT_3trace(CHNL_DebugMask, GT_ENTER,
		  "Entered CHNL_Create: phChnlMgr: 0x%x\t"
		  "hDevObject: 0x%x\tpMgrAttrs:0x%x\n",
		  phChnlMgr, hDevObject, pMgrAttrs);

	*phChnlMgr = NULL;

	/* Validate args: */
	if ((0 < pMgrAttrs->cChannels) &&
	   (pMgrAttrs->cChannels <= CHNL_MAXCHANNELS)) {
		status = DSP_SOK;
	} else if (pMgrAttrs->cChannels == 0) {
		status = DSP_EINVALIDARG;
		GT_0trace(CHNL_DebugMask, GT_7CLASS,
			  "CHNL_Create:Invalid Args\n");
	} else {
		status = CHNL_E_MAXCHANNELS;
		GT_0trace(CHNL_DebugMask, GT_7CLASS,
			  "CHNL_Create:Error Max Channels\n");
	}
	if (pMgrAttrs->uWordSize == 0) {
		status = CHNL_E_INVALIDWORDSIZE;
		GT_0trace(CHNL_DebugMask, GT_7CLASS,
			  "CHNL_Create:Invalid Word size\n");
	}
	if (DSP_SUCCEEDED(status)) {
		status = DEV_GetChnlMgr(hDevObject, &hChnlMgr);
		if (DSP_SUCCEEDED(status) && hChnlMgr != NULL)
			status = CHNL_E_MGREXISTS;

	}

	if (DSP_SUCCEEDED(status)) {
		struct WMD_DRV_INTERFACE *pIntfFxns;
		DEV_GetIntfFxns(hDevObject, &pIntfFxns);
		/* Let WMD channel module finish the create: */
		status = (*pIntfFxns->pfnChnlCreate)(&hChnlMgr, hDevObject,
			  pMgrAttrs);
		if (DSP_SUCCEEDED(status)) {
			/* Fill in WCD channel module's fields of the
			 * CHNL_MGR structure */
			pChnlMgr = (struct CHNL_MGR_ *)hChnlMgr;
			pChnlMgr->pIntfFxns = pIntfFxns;
			/* Finally, return the new channel manager handle: */
			*phChnlMgr = hChnlMgr;
			GT_1trace(CHNL_DebugMask, GT_1CLASS,
				  "CHNL_Create: Success pChnlMgr:"
				  "0x%x\n", pChnlMgr);
		}
	}

	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Exiting CHNL_Create: pChnlMgr: 0x%x,"
		  "status: 0x%x\n", pChnlMgr, status);
	DBC_Ensure(DSP_FAILED(status) || CHNL_IsValidMgr(pChnlMgr));

	return (status);
}

/*
 *  ======== CHNL_Destroy ========
 *  Purpose:
 *      Close all open channels, and destroy the channel manager.
 */
DSP_STATUS CHNL_Destroy(struct CHNL_MGR *hChnlMgr)
{
	struct CHNL_MGR_ *pChnlMgr = (struct CHNL_MGR_ *)hChnlMgr;
	struct WMD_DRV_INTERFACE *pIntfFxns;
	DSP_STATUS status;

	DBC_Require(cRefs > 0);

	GT_1trace(CHNL_DebugMask, GT_ENTER,
		  "Entered CHNL_Destroy: hChnlMgr: 0x%x\n", hChnlMgr);
	if (CHNL_IsValidMgr(pChnlMgr)) {
		pIntfFxns = pChnlMgr->pIntfFxns;
		/* Let WMD channel module destroy the CHNL_MGR: */
		status = (*pIntfFxns->pfnChnlDestroy)(hChnlMgr);
	} else {
		GT_0trace(CHNL_DebugMask, GT_7CLASS,
			  "CHNL_Destroy:Invalid Handle\n");
		status = DSP_EHANDLE;
	}

	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Exiting CHNL_Destroy: pChnlMgr: 0x%x,"
		  " status:0x%x\n", pChnlMgr, status);
	DBC_Ensure(DSP_FAILED(status) || !CHNL_IsValidMgr(pChnlMgr));

	return (status);
}

/*
 *  ======== CHNL_Exit ========
 *  Purpose:
 *      Discontinue usage of the CHNL module.
 */
void CHNL_Exit()
{
	DBC_Require(cRefs > 0);

	cRefs--;

	GT_1trace(CHNL_DebugMask, GT_5CLASS,
		  "Entered CHNL_Exit, ref count: 0x%x\n", cRefs);

	DBC_Ensure(cRefs >= 0);
}

#ifdef DEAD_CODE

/*
 *  ======== CHNL_FlushIO ========
 */
DSP_STATUS CHNL_FlushIO(struct CHNL_OBJECT *hChnl, DWORD dwTimeOut)
{
	DSP_STATUS status;
	struct CHNL_OBJECT_ *pChnl = (struct CHNL_OBJECT_ *)hChnl;
	struct WMD_DRV_INTERFACE *pIntfFxns;

	DBC_Require(cRefs > 0);

	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Entered CHNL_FlushIO: hChnl: 0x%x\t"
		  "dwTimeOut: 0x%x\n", hChnl, dwTimeOut);
	if (CHNL_IsValidChnl(pChnl)) {
		pIntfFxns = pChnl->pChnlMgr->pIntfFxns;
		status = (*pIntfFxns->pfnChnlFlushIO)(hChnl, dwTimeOut);
	} else {
		GT_0trace(CHNL_DebugMask, GT_7CLASS,
			  "CHNL_FlushIO:Invalid Handle\n");
		status = DSP_EHANDLE;
	}

	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Exiting CHNL_FlushIO: hChnl: 0x%x,"
		  " status: 0x%x\n", hChnl, status);
	return (status);
}

/*
 *  ======== CHNL_FreeBuffer ========
 */
DSP_STATUS CHNL_FreeBuffer(struct CHNL_MGR *hChnlMgr, ULONG cBytes, PVOID pBuf)
{
	DSP_STATUS status = DSP_SOK;
	/*struct CHNL_MGR_       * pChnlMgr = (struct CHNL_MGR_ *)hChnlMgr; */

	DBC_Require(cRefs > 0);

	GT_3trace(CHNL_DebugMask, GT_ENTER,
		  "Entered CHNL_FreeBuffer: hChnlMgr: 0x"
		  "%x\tcBytes: 0x%x\t\npBuf: ox%x\n", hChnlMgr, cBytes, pBuf);

	if (CHNL_IsValidMgr((struct CHNL_MGR_ *)hChnlMgr)) {
		if (pBuf == NULL) {
			status = DSP_EINVALIDARG;
		}
	} else {
		GT_0trace(CHNL_DebugMask, GT_7CLASS,
			  "CHNL_FreeBuffer:Invalid Handle\n");
		status = DSP_EHANDLE;
	}
	GT_1trace(CHNL_DebugMask, GT_ENTER,
		  "Exit CHNL_Freebuffer: status = 0x%x\n", status);
	return (status);
}

/*
 *  ======== CHNL_GetEventHandle ========
 *  Purpose:
 *      Retrieve this channel's I/O completion auto-reset event.
 */
DSP_STATUS CHNL_GetEventHandle(struct CHNL_OBJECT *hChnl, OUT HANDLE *phEvent)
{
	DSP_STATUS status;
	struct CHNL_OBJECT_ *pChnl = (struct CHNL_OBJECT_ *)hChnl;
	struct WMD_DRV_INTERFACE *pIntfFxns;
	struct CHNL_INFO chnlInfo;

	DBC_Require(cRefs > 0);
	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Entered CHNL_GetEventHandle: hChnl: "
		  "0x%x\tphEvent: 0x%x\n", hChnl, phEvent);

	if (phEvent) {
		if (CHNL_IsValidChnl(pChnl)) {
			pIntfFxns = pChnl->pChnlMgr->pIntfFxns;
			status = (*pIntfFxns->pfnChnlGetInfo)(hChnl, &chnlInfo);
			if (DSP_SUCCEEDED(status)) {
				*phEvent = chnlInfo.hEvent;
			}
		} else {
			GT_0trace(CHNL_DebugMask, GT_7CLASS,
				  "CHNL_FreeBuffer:Invalid "
				  "Handle\n");
			status = DSP_EHANDLE;
		}
	} else {
		status = DSP_EPOINTER;
	}
	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Exit CHNL_GetEventHandle: status: "
		  "0x%x\t\n hEvent: 0x%x\n", status, *phEvent);
	return (status);
}

#endif

/*
 *  ======== CHNL_GetHandle ========
 *  Purpose:
 *      Retrieve the channel handle given the logical ID and channel manager.
 */
DSP_STATUS CHNL_GetHandle(struct CHNL_MGR *hChnlMgr, ULONG uChnlID,
			  OUT struct CHNL_OBJECT **phChnl)
{
	DSP_STATUS status = DSP_SOK;
	struct CHNL_MGR_ *pChnlMgr = (struct CHNL_MGR_ *)hChnlMgr;
	struct WMD_DRV_INTERFACE *pIntfFxns;
	struct CHNL_MGRINFO chnlMgrInfo;

	DBC_Require(cRefs > 0);

	GT_3trace(CHNL_DebugMask, GT_ENTER, "Entered CHNL_GetHandle: hChnlMgr: "
		  "0x%x\tuChnlID: 0x%x\t\nphChnl: 0x%x\n", hChnlMgr, uChnlID,
		  phChnl);
	if (phChnl) {
		*phChnl = NULL;
		if (CHNL_IsValidMgr(pChnlMgr)) {
			pIntfFxns = pChnlMgr->pIntfFxns;
			status = (*pIntfFxns->pfnChnlGetMgrInfo)(hChnlMgr,
				  uChnlID, &chnlMgrInfo);
			if (DSP_SUCCEEDED(status)) {
				*phChnl = chnlMgrInfo.hChnl;
			}
		} else {
			status = DSP_EHANDLE;
			GT_0trace(CHNL_DebugMask, GT_7CLASS,
				  "CHNL_GetHandle:Invalid Handle\n");
		}
	} else {
		status = DSP_EPOINTER;
	}
	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Exit CHNL_GetHandle: status: 0x%x\t\n"
		  "hChnl: 0x%x\n", status, *phChnl);
	return (status);
}

#ifdef DEAD_CODE

/*
 *  ======== CHNL_GetId ========
 *  Purpose:
 *      Retrieve the channel logical ID of this channel.
 */
DSP_STATUS CHNL_GetId(struct CHNL_OBJECT *hChnl, OUT ULONG *pdwID)
{
	DSP_STATUS status;
	struct CHNL_OBJECT_ *pChnl = (struct CHNL_OBJECT_ *)hChnl;
	struct WMD_DRV_INTERFACE *pIntfFxns;
	struct CHNL_INFO chnlInfo;

	DBC_Require(cRefs > 0);
	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Enter CHNL_GetId: hChnl: 0x%x\t\npdwID:"
		  "0x%x\n", hChnl, pdwID);
	if (pdwID) {
		*pdwID = 0;
		if (CHNL_IsValidChnl(pChnl)) {
			pIntfFxns = pChnl->pChnlMgr->pIntfFxns;
			status = (*pIntfFxns->pfnChnlGetInfo)(hChnl, &chnlInfo);
			if (DSP_SUCCEEDED(status)) {
				*pdwID = chnlInfo.dwID;
			}
		} else {
			GT_0trace(CHNL_DebugMask, GT_7CLASS,
				  "CHNL_GetId:Invalid Handle\n");
			status = DSP_EHANDLE;
		}
	} else {
		status = DSP_EPOINTER;
	}
	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Exit CHNL_GetID: status:0x%x\t\nChannel"
		  "ID: 0x%x\n", status, *pdwID);
	return (status);
}

/*
 *  ======== CHNL_GetIOCompletion ========
 *  Purpose:
 *      Optionally wait for I/O completion on a channel.  Dequeue an I/O
 *      completion record, which contains information about the completed
 *      I/O request.
 */
DSP_STATUS CHNL_GetIOCompletion(struct CHNL_OBJECT *hChnl, DWORD dwTimeOut,
				OUT struct CHNL_IOC *pIOC)
{
	DSP_STATUS status;
	struct CHNL_OBJECT_ *pChnl = (struct CHNL_OBJECT_ *)hChnl;
	struct WMD_DRV_INTERFACE *pIntfFxns;

	DBC_Require(cRefs > 0);

	GT_3trace(CHNL_DebugMask, GT_ENTER,
		  "Entering CHNL_GetIOCompletion: hChnl:"
		  "0x%x\tdwTimeOut: 0x%x\tpIOC: 0x%x\n", hChnl, dwTimeOut,
		  pIOC);
	if (CHNL_IsValidChnl(pChnl)) {
		pIntfFxns = pChnl->pChnlMgr->pIntfFxns;
		status = (*pIntfFxns->pfnChnlGetIOC) (hChnl, dwTimeOut, pIOC);
	} else {
		if (pIOC) {
			pIOC->pBuf = NULL;
			pIOC->cBytes = 0;
		}
		status = DSP_EHANDLE;
	}
	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Exiting CHNL_GetIOCompletion: hChnl: "
		  "0x%x,status: 0x%x\n", hChnl, status);
	return (status);
}

/*
 *  ======== CHNL_GetMgr ========
 *  Purpose:
 *      Retrieve a channel manager handle, required for opening new channels
 *      and closing old ones on a given 'Bridge board.
 */
DSP_STATUS CHNL_GetMgr(struct CFG_DEVNODE *hDevNode,
			OUT struct CHNL_MGR **phChnlMgr)
{
	struct DEV_OBJECT *hDevObject;
	DSP_STATUS status;

	DBC_Require(cRefs > 0);
	DBC_Require(phChnlMgr != NULL);

	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Enter CHNL_GetMgr: hDevNode: 0x%x\t\n "
		  "phChnlMgr: 0x%x\n", hDevNode, *phChnlMgr);
	if (CFG_GetDevObject(hDevNode, (DWORD *)&hDevObject) ==
	   CFG_E_INVALIDHDEVNODE) {
		status = DSP_EHANDLE;
	} else {
		status = DEV_GetChnlMgr(hDevObject, phChnlMgr);
		if (DSP_SUCCEEDED(status) && *phChnlMgr == NULL) {
			status = CHNL_E_NOMGR;
			GT_0trace(CHNL_DebugMask, GT_7CLASS,
				  "CHNL_GetMgr: Failed");
		}
	}

	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Exit CHNL_GetMgr: status: 0x%x\t\n"
		  "hChnlMgr: 0x%x\n", status, *phChnlMgr);
	DBC_Ensure((DSP_FAILED(status) && *phChnlMgr == NULL) ||
		   (DSP_SUCCEEDED(status) &&
			CHNL_IsValidMgr((struct CHNL_MGR_ *)*phChnlMgr)));

	return (status);
}

/*
 *  ======== CHNL_GetMode ========
 *  Purpose:
 *      Retrieve the mode flags of this channel.
 */
DSP_STATUS CHNL_GetMode(struct CHNL_OBJECT *hChnl, OUT CHNL_MODE *pMode)
{
	DSP_STATUS status;
	struct CHNL_OBJECT_ *pChnl = (struct CHNL_OBJECT_ *)hChnl;
	struct WMD_DRV_INTERFACE *pIntfFxns;
	struct CHNL_INFO chnlInfo;

	DBC_Require(cRefs > 0);

	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Enter CHNL_GetMode: hChnl: 0x%x\t\n"
		  "pMode:0x%x\n", hChnl, *pMode);
	if (pMode) {
		*pMode = 0x00;
		if (CHNL_IsValidChnl(pChnl)) {
			pIntfFxns = pChnl->pChnlMgr->pIntfFxns;
			status = (*pIntfFxns->pfnChnlGetInfo)(hChnl, &chnlInfo);
			if (DSP_SUCCEEDED(status)) {
				*pMode = chnlInfo.dwMode;
			}
		} else {
			GT_0trace(CHNL_DebugMask, GT_7CLASS,
				  "CHNL_GetMode:Invalid Handle\n");
			status = DSP_EHANDLE;
		}
	} else {
		status = DSP_EPOINTER;
	}
	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Exit CHNL_GetMode: status: 0x%x\t\n"
		  "Mode:0x%x\n", status, *pMode);

	return (status);
}

/*
 *  ======== CHNL_GetPosition ========
 *  Purpose:
 *      Retrieve the total number of bytes transferred on this channel.
 */
DSP_STATUS CHNL_GetPosition(struct CHNL_OBJECT *hChnl, OUT ULONG *pcPosition)
{
	DSP_STATUS status;
	struct CHNL_OBJECT_ *pChnl = (struct CHNL_OBJECT_ *)hChnl;
	struct WMD_DRV_INTERFACE *pIntfFxns;
	struct CHNL_INFO chnlInfo;

	DBC_Require(cRefs > 0);

	GT_2trace(CHNL_DebugMask, GT_ENTER, "Enter CHNL_GetPosition: hChnl: "
		  "0x%x\t\n pcPosition: 0x%x\n", hChnl, pcPosition);

	if (pcPosition) {
		*pcPosition = 0;
		if (CHNL_IsValidChnl(pChnl)) {
			pIntfFxns = pChnl->pChnlMgr->pIntfFxns;
			status = (*pIntfFxns->pfnChnlGetInfo)(hChnl, &chnlInfo);
			if (DSP_SUCCEEDED(status)) {
				*pcPosition = chnlInfo.cPosition;
			}
		} else {
			GT_0trace(CHNL_DebugMask, GT_7CLASS,
				  "CHNL_GetPosition: Invalid Handle\n");
			status = DSP_EHANDLE;
		}
	} else {
		status = DSP_EPOINTER;
	}
	GT_2trace(CHNL_DebugMask, GT_ENTER, "Exit CHNL_GetPosition: status: "
		  "0x%x\t\npcPosition: 0x%x\n", status, *pcPosition);
	return (status);
}

#endif

/*
 *  ======== CHNL_GetProcessHandle ========
 *  Purpose:
 *      Retrieve the handle of the process owning this channel.
 */
DSP_STATUS CHNL_GetProcessHandle(struct CHNL_OBJECT *hChnl,
				 OUT HANDLE *phProcess)
{
	DSP_STATUS status = DSP_SOK;
	struct CHNL_OBJECT_ *pChnl = (struct CHNL_OBJECT_ *)hChnl;
	struct WMD_DRV_INTERFACE *pIntfFxns;
	struct CHNL_INFO chnlInfo;

	DBC_Require(cRefs > 0);

	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Enter CHNL_GetProcessHandle: hChnl: "
		  "0x%x\t\n phProcess: 0x%x\n", hChnl, phProcess);
	if (phProcess) {
		*phProcess = NULL;
		if (CHNL_IsValidChnl(pChnl)) {
			pIntfFxns = pChnl->pChnlMgr->pIntfFxns;
			status = (*pIntfFxns->pfnChnlGetInfo)(hChnl, &chnlInfo);
			if (DSP_SUCCEEDED(status)) {
				*phProcess = chnlInfo.hProcess;
			}
		} else {
			status = DSP_EHANDLE;
		}
	} else {
		status = DSP_EPOINTER;
	}
	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Exit CHNL_GetProcessHandle: status: "
		  "0x%x\t\n phProcess: 0x%x\n", status, *phProcess);
	return (status);
}

/*
 *  ======== CHNL_Init ========
 *  Purpose:
 *      Initialize the CHNL module's private state.
 */
BOOL CHNL_Init()
{
	BOOL fRetval = TRUE;

	DBC_Require(cRefs >= 0);

	if (cRefs == 0) {
		DBC_Assert(!CHNL_DebugMask.flags);
		GT_create(&CHNL_DebugMask, "CH");   /* "CH" for CHannel */
	}

	if (fRetval)
		cRefs++;

	GT_1trace(CHNL_DebugMask, GT_5CLASS,
		  "Entered CHNL_Init, ref count: 0x%x\n",
		  cRefs);

	DBC_Ensure((fRetval && (cRefs > 0)) || (!fRetval && (cRefs >= 0)));

	return (fRetval);
}

#ifdef DEAD_CODE
/*
 *  ======== CHNL_Open ========
 *  Purpose:
 *      Open a new half-duplex channel to the DSP board.
 *
 */
DSP_STATUS CHNL_Open(OUT struct CHNL_OBJECT **phChnl, struct CHNL_MGR *hChnlMgr,
		     CHNL_MODE uMode, ULONG uChnlId,
		     CONST IN struct CHNL_ATTRS *pAttrs)
{
	DSP_STATUS status;
	struct CHNL_MGR_ *pChnlMgr = (struct CHNL_MGR_ *)hChnlMgr;
	struct WMD_DRV_INTERFACE *pIntfFxns;

	DBC_Require(cRefs > 0);
	DBC_Require(phChnl != NULL);
	DBC_Require(pAttrs != NULL);
	DBC_Require(pAttrs->hEvent != NULL);

	GT_5trace(CHNL_DebugMask, GT_ENTER,
		  "Entering CHNL_Open: phChnl: 0x%x\t"
		  "hChnlMgr: 0x%x\tuMode: 0x%x\tuChnlId: 0x%x\t pAttrs: 0x%x\n",
		  phChnl, hChnlMgr, uMode, uChnlId, pAttrs);
	if (CHNL_IsValidMgr(pChnlMgr)) {
		pIntfFxns = pChnlMgr->pIntfFxns;
		/* Let WMD channel module open the channel: */
		status = (*pIntfFxns->pfnChnlOpen)(phChnl, hChnlMgr, uMode,
			 uChnlId, pAttrs);
	} else {
		status = DSP_EHANDLE;
		GT_0trace(CHNL_DebugMask, GT_7CLASS,
			  "CHNL_Open:Invalid Handle\n");
	}

	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Exiting CHNL_Open: phChnl: 0x%x,status:"
		  "0x%x\n", phChnl, status);

	DBC_Ensure((DSP_FAILED(status) && *phChnl == NULL) ||
		   (DSP_SUCCEEDED(status) &&
			CHNL_IsValidChnl((struct CHNL_OBJECT_ *)*phChnl)));

	return (status);
}
#endif

/*
 *  ======== GetNumOpenChannels ========
 *  Purpose:
 *      Retrieve number of open channels
 *  Parameters:
 *      hChnlMgr:       Handle to a valid channel manager, or NULL.
 *      pcOpenChannels: Location to store number of open channels.
 *  Returns:
 *      DSP_SOK:        Success;
 *      DSP_EHANDLE:    Invalid hChnlMgr.
 *      E_POINTER:      pcOpenChannels == NULL.
 *  Requires:
 *  Ensures:
 *      DSP_SOK:        *pcOpenChannels points to a valid number
 *                      if pcOpenChannels != NULL.
 */
static DSP_STATUS GetNumOpenChannels(struct CHNL_MGR *hChnlMgr,
				     OUT ULONG *pcOpenChannels)
{
	DSP_STATUS status = DSP_SOK;
	struct CHNL_MGR_ *pChnlMgr = (struct CHNL_MGR_ *)hChnlMgr;
	struct WMD_DRV_INTERFACE *pIntfFxns;
	struct CHNL_MGRINFO chnlMgrInfo;

	DBC_Require(cRefs > 0);
	if (pcOpenChannels) {
		*pcOpenChannels = 0;
		if (CHNL_IsValidMgr(pChnlMgr)) {
			pIntfFxns = pChnlMgr->pIntfFxns;
			status = (*pIntfFxns->pfnChnlGetMgrInfo)(hChnlMgr, 0,
				 &chnlMgrInfo);
			if (DSP_SUCCEEDED(status)) {
				*pcOpenChannels = chnlMgrInfo.cOpenChannels;
			}
		} else {
			status = DSP_EHANDLE;
		}
	} else {
		status = DSP_EPOINTER;
	}
	return (status);
}

/*
 *  ======== GetNumOpenChannels ========
 *  Purpose:
 *      Retrieve number of total channels supported.
 *  Parameters:
 *      hChnlMgr:       Handle to a valid channel manager, or NULL.
 *      pcChannels:     Location to store number of channels.
 *  Returns:
 *      DSP_SOK:        Success;
 *      DSP_EHANDLE:    Invalid hChnlMgr.
 *      E_POINTER:      pcOpenChannels == NULL.
 *  Requires:
 *  Ensures:
 *      DSP_SOK:        *pcChannels points to a valid number
 *                      if pcOpenChannels != NULL.
 */
static DSP_STATUS GetNumChannels(struct CHNL_MGR *hChnlMgr,
				 OUT ULONG *pcChannels)
{
	DSP_STATUS status = DSP_SOK;
	struct CHNL_MGR_ *pChnlMgr = (struct CHNL_MGR_ *)hChnlMgr;
	struct WMD_DRV_INTERFACE *pIntfFxns;
	struct CHNL_MGRINFO chnlMgrInfo;

	DBC_Require(cRefs > 0);

	if (pcChannels) {
		*pcChannels = 0;
		if (CHNL_IsValidMgr(pChnlMgr)) {
			pIntfFxns = pChnlMgr->pIntfFxns;
			status = (*pIntfFxns->pfnChnlGetMgrInfo)(hChnlMgr, 0,
				 &chnlMgrInfo);
			if (DSP_SUCCEEDED(status)) {
				*pcChannels = chnlMgrInfo.cChannels;
			}
		} else {
			status = DSP_EHANDLE;
		}
	} else {
		status = DSP_EPOINTER;
	}
	return (status);
}

