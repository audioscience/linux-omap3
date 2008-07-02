/*
 * dspbridge/src/wmd/linux/omap/common/ue_deh.c
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
 *  ======== ue_deh.c ========
 *  Description:
 *      Implements upper edge DSP exception handling (DEH) functions.
 *
 *! Revision History:
 *! ================
 *! 03-Jan-2005 hn: Support for IVA DEH.
 *! 05-Jan-2004 vp: Updated for the 24xx HAL library.
 *! 19-Feb-2003 vp: Code review updates.
 *!                 - Cosmetic changes.
 *! 18-Oct-2002 sb: Ported to Linux platform.
 *! 10-Dec-2001 kc: Updated DSP error reporting in DEBUG mode.
 *! 10-Sep-2001 kc: created.
 */

/*  ----------------------------------- Host OS */
#include <host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <std.h>
#include <dbdefs.h>
#include <errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <dbc.h>
#include <dbg.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <csl.h>
#include <cfg.h>
#include <dpc.h>
#include <isr.h>
#include <mem.h>
#include <ntfy.h>
#include <drv.h>

/*  ----------------------------------- Link Driver */
#include <wmddeh.h>

/*  ----------------------------------- Platform Manager */
#include <dev.h>
#include <wcd.h>

/* ------------------------------------ Hardware Abstraction Layer */
#include <hal_defs.h>
#include <hal_mmu.h>

/*  ----------------------------------- This */
#include "mmu_fault.h"
#include "_tiomap.h"
#include "_deh.h"
#include <_tiomap_mmu.h>

struct HAL_MMUMapAttrs_t  mapAttrs = { HAL_LITTLE_ENDIAN, HAL_ELEM_SIZE_16BIT,
					HAL_MMU_CPUES} ;
#define VirtToPhys(x)       ((x) - PAGE_OFFSET + PHYS_OFFSET)
/*
 *  ======== WMD_DEH_Create ========
 *  purpose:
 *      Creates DEH manager object.
 */
DSP_STATUS WMD_DEH_Create(OUT struct DEH_MGR **phDehMgr,
			 struct DEV_OBJECT *hDevObject)
{
	DSP_STATUS status = DSP_SOK;
	struct DEH_MGR *pDehMgr = NULL;
	struct CFG_HOSTRES cfgHostRes;
	struct CFG_DEVNODE *hDevNode;
	struct WMD_DEV_CONTEXT *hWmdContext = NULL;

	DBG_Trace(DBG_LEVEL1, "Entering DEH_Create: 0x%x\n", phDehMgr);
	 /*  Message manager will be created when a file is loaded, since
	 *  size of message buffer in shared memory is configurable in
	 *  the base image.  */
	/* Get WMD context info. */
	DEV_GetWMDContext(hDevObject, &hWmdContext);
	DBC_Assert(hWmdContext);
	/* Allocate IO manager object: */
	MEM_AllocObject(pDehMgr, struct DEH_MGR, SIGNATURE);
	if (pDehMgr == NULL) {
		status = DSP_EMEMORY;
	} else {
		/* Create an NTFY object to manage notifications */
		if (DSP_SUCCEEDED(status))
			status = NTFY_Create(&pDehMgr->hNtfy);

		/* Create a DPC object. */
		status = DPC_Create(&pDehMgr->hMmuFaultDpc, MMU_FaultDpc,
				   (PVOID)pDehMgr);
		if (DSP_SUCCEEDED(status))
			status = DEV_GetDevNode(hDevObject, &hDevNode);

		if (DSP_SUCCEEDED(status))
			status = CFG_GetHostResources(hDevNode, &cfgHostRes);

		if (DSP_SUCCEEDED(status)) {
			/* Fill in context structure */
			pDehMgr->hWmdContext = hWmdContext;
			pDehMgr->errInfo.dwErrMask = 0L;
			pDehMgr->errInfo.dwVal1 = 0L;
			pDehMgr->errInfo.dwVal2 = 0L;
			pDehMgr->errInfo.dwVal3 = 0L;
			/* Install ISR function for DSP MMU fault */
			status = ISR_Install(&pDehMgr->hMmuFaultIsr,
				 &cfgHostRes, (ISR_PROC)MMU_FaultIsr,
				 DSP_MMUFAULT, (PVOID)pDehMgr);
		}
	}
	if (DSP_FAILED(status)) {
		/* If create failed, cleanup */
		WMD_DEH_Destroy((struct DEH_MGR *)pDehMgr);
		*phDehMgr = NULL;
	} else {
		*phDehMgr = (struct DEH_MGR *)pDehMgr;
	}
	DBG_Trace(DBG_LEVEL1, "Exiting DEH_Create.\n");
	return (status);
}

/*
 *  ======== WMD_DEH_Destroy ========
 *  purpose:
 *      Destroys DEH manager object.
 */
DSP_STATUS WMD_DEH_Destroy(struct DEH_MGR *hDehMgr)
{
	DSP_STATUS status = DSP_SOK;
	struct DEH_MGR *pDehMgr = (struct DEH_MGR *)hDehMgr;

	DBG_Trace(DBG_LEVEL1, "Entering DEH_Destroy: 0x%x\n", pDehMgr);
	if (MEM_IsValidHandle(pDehMgr, SIGNATURE)) {
		/* If notification object exists, delete it */
		if (pDehMgr->hNtfy)
			(VOID)NTFY_Delete(pDehMgr->hNtfy);

		/* Disable DSP MMU fault */
		(VOID)ISR_Uninstall(pDehMgr->hMmuFaultIsr);
		(VOID)DPC_Destroy(pDehMgr->hMmuFaultDpc);
		/* Deallocate the DEH manager object */
		MEM_FreeObject(pDehMgr);
	}
	DBG_Trace(DBG_LEVEL1, "Exiting DEH_Destroy.\n");
	return (status);
}

/*
 *  ======== WMD_DEH_RegisterNotify ========
 *  purpose:
 *      Registers for DEH notifications.
 */
DSP_STATUS WMD_DEH_RegisterNotify(struct DEH_MGR *hDehMgr, UINT uEventMask,
				 UINT uNotifyType,
				 struct DSP_NOTIFICATION *hNotification)
{
	DSP_STATUS status = DSP_SOK;
	struct DEH_MGR *pDehMgr = (struct DEH_MGR *)hDehMgr;

	DBG_Trace(DBG_LEVEL1, "Entering WMD_DEH_RegisterNotify: 0x%x\n",
		 pDehMgr);

	if (MEM_IsValidHandle(pDehMgr, SIGNATURE)) {
		status = NTFY_Register(pDehMgr->hNtfy, hNotification,
			 uEventMask, uNotifyType);
	}
	DBG_Trace(DBG_LEVEL1, "Exiting WMD_DEH_RegisterNotify.\n");
	return (status);
}

#if ((defined DEBUG) || (defined DDSP_DEBUG_PRODUCT)) && GT_TRACE

/*
 *  ======== PackTraceBuffer ========
 *  Purpose:
 *      Removes extra nulls from the trace buffer returned from the DSP.
 *      Works even on buffers that already are packed (null removed); but has
 *      one bug in that case -- loses the last character (replaces with '\0').
 *      Continues through conversion for full set of nBytes input characters.
 *  Parameters:
 *    lpBuf:            Pointer to input/output buffer
 *    nBytes:           Number of characters in the buffer
 *    ulNumWords:       Number of DSP words in the buffer.  Indicates potential
 *                      number of extra carriage returns to generate.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EMEMORY:    Unable to allocate memory.
 *  Requires:
 *      lpBuf must be a fully allocated writable block of at least nBytes.
 *      There are no more than ulNumWords extra characters needed (the number of
 *      linefeeds minus the number of NULLS in the input buffer).
 */
static DSP_STATUS PackTraceBuffer(PSTR lpBuf, ULONG nBytes, ULONG ulNumWords)
{
	PSTR lpTmpBuf;
	PSTR lpBufStart;
	PSTR lpTmpStart;
	ULONG nCnt;
	CHAR thisChar;
	DSP_STATUS status = DSP_SOK;

	/* tmp workspace, 1 KB longer than input buf */
	lpTmpBuf = MEM_Calloc((nBytes + ulNumWords), MEM_PAGED);
	if (lpTmpBuf == NULL) {
		DBG_Trace(DBG_LEVEL7, "PackTrace buffer:OutofMemory \n");
		status = DSP_EMEMORY;
	}

	if (DSP_SUCCEEDED(status)) {
		lpBufStart = lpBuf;
		lpTmpStart = lpTmpBuf;
		for (nCnt = nBytes; nCnt > 0; nCnt--) {
			thisChar = *lpBuf++;
			switch (thisChar) {
			case '\0':	/* Skip null bytes */
				break;
			case '\n':	/* Convert \n to \r\n */
				/* NOTE: do not reverse order; Some OS */
				/* editors control doesn't understand "\n\r" */
				*lpTmpBuf++ = '\r';
				*lpTmpBuf++ = '\n';
				break;
			default:	/* Copy in the actual ascii byte */
				*lpTmpBuf++ = thisChar;
				break;
			}
		}
		*lpTmpBuf = '\0';    /* Make sure tmp buf is null terminated */
		/* Cut output down to input buf size */
		CSL_Strcpyn(lpBufStart, lpTmpStart, nBytes);
		/*Make sure output is null terminated */
		lpBufStart[nBytes - 1] = '\0';
		MEM_Free(lpTmpStart);
	}
	return (status);
}

#endif	/* #if ((defined DEBUG) || (defined DDSP_DEBUG_PRODUCT)) && GT_TRACE */

/*
 *  ======== WMD_DEH_Notify ========
 *  Purpose:
 *      DEH error notification function. Informs user about the error.
 */
VOID CDECL WMD_DEH_Notify(struct DEH_MGR *hDehMgr, ULONG ulEventMask,
			 DWORD dwErrInfo)
{
	struct DEH_MGR *pDehMgr = (struct DEH_MGR *)hDehMgr;
	struct WMD_DEV_CONTEXT *pDevContext;
	DSP_STATUS status = DSP_SOK;
	DWORD memPhysical = 0;
	SLST_t slst = SMALL_PAGE;
	ULONG HAL_MMU_MAX_TLB_COUNT = 31;
	DWORD extern faultAddr;
	DWORD extern dmmuEventMask;
	DWORD sizeTlb;
	struct CFG_HOSTRES resources;
	DWORD dummyVaAddr;
	HAL_STATUS halStatus;



	status = CFG_GetHostResources(
			(struct CFG_DEVNODE *)DRV_GetFirstDevExtension(),
			&resources);
	if (!DSP_SUCCEEDED(status))
		DBG_Trace(DBG_LEVEL7,
			 "**Failed to get Host Resources in MMU ISR **\n");

	DBG_Trace(DBG_LEVEL1, "Entering WMD_DEH_Notify: 0x%x, 0x%x\n", pDehMgr,
		 ulEventMask);
	if (MEM_IsValidHandle(pDehMgr, SIGNATURE)) {
		printk(KERN_INFO "WMD_DEH_Notify: ********** DEVICE EXCEPTION "
			"**********\n");
		switch (ulEventMask) {
		case DSP_SYSERROR:
			/* reset errInfo structure before use */
			pDehMgr->errInfo.dwErrMask = DSP_SYSERROR;
			pDehMgr->errInfo.dwVal1 = 0L;
			pDehMgr->errInfo.dwVal2 = 0L;
			pDehMgr->errInfo.dwVal3 = 0L;
			pDehMgr->errInfo.dwVal1 = dwErrInfo;
			printk(KERN_ERR "WMD_DEH_Notify: DSP_SYSERROR, errInfo "
				"= 0x%x\n", dwErrInfo);
			break;
		case DSP_MMUFAULT:
			/* MMU fault routine should have set err info
			 * structure */
			pDehMgr->errInfo.dwErrMask = DSP_MMUFAULT;
			printk(KERN_INFO "WMD_DEH_Notify: DSP_MMUFAULT, errInfo "
				"= 0x%x\n", dwErrInfo);
			printk(KERN_INFO "WMD_DEH_Notify: DSP_MMUFAULT, High "
				"Address = 0x%x\n", pDehMgr->errInfo.dwVal1);
			printk(KERN_INFO "WMD_DEH_Notify: DSP_MMUFAULT, Low "
				"Address = 0x%x\n", pDehMgr->errInfo.dwVal2);
			printk(KERN_INFO "WMD_DEH_Notify: DSP_MMUFAULT, fault "
				"address = 0x%x\n", faultAddr);
			dummyVaAddr = (ULONG)MEM_Calloc(sizeof(char) * 0x1000,
					MEM_PAGED);
			memPhysical = (ULONG)MEM_Calloc(sizeof(char) * 0x1000,
					MEM_PAGED);
			dummyVaAddr = PG_ALIGN_LOW((ULONG)dummyVaAddr,
					PG_SIZE_4K);
			memPhysical  = VirtToPhys(dummyVaAddr);
			DBG_Trace(DBG_LEVEL6, "WMD_DEH_Notify: DSP_MMUFAULT, "
				 "mem Physical= 0x%x\n", memPhysical);
			/* Reset the dynamic mmu index to fixed count if it
			 * exceeds 31. So that the dynmmuindex is always between
			 * the range of standard/fixed entries and 31.  */
			if (pDevContext->numTLBEntries >
			   HAL_MMU_MAX_TLB_COUNT) {
				pDevContext->numTLBEntries = pDevContext->
					fixedTLBEntries;
			}
			DBG_Trace(DBG_LEVEL6, "Adding TLB Entry %d: VA: 0x%x, "
				 "PA: 0x%x, Len: 0x%x\n", pDevContext->
				numTLBEntries, faultAddr, memPhysical, sizeTlb);
			if (DSP_SUCCEEDED(status)) {
				halStatus = HAL_MMU_TLBAdd(resources.dwDmmuBase,
					memPhysical, faultAddr,
					HAL_PAGE_SIZE_4KB, 1, &mapAttrs,
					HAL_SET, HAL_SET);
			}
			/* send an interrupt to DSP */
			HAL_MBOX_MsgWrite(resources.dwMboxBase, MBOX_ARM2DSP,
					 MBX_DEH_CLASS | MBX_DEH_EMMU);
			/* Clear MMU interrupt */
			HAL_MMU_EventAck(resources.dwDmmuBase,
					 HAL_MMU_TRANSLATION_FAULT);
			DBG_Trace(DBG_LEVEL6,
				 "***** PrintDspTraceBuffer: before\n");
			/*PrintDspTraceBuffer(hDehMgr);*/
			DBG_Trace(DBG_LEVEL6,
				 "***** PrintDspTraceBuffer: after \n");
			break;
		default:
			DBG_Trace(DBG_LEVEL6,
				 "WMD_DEH_Notify: Unknown Error, errInfo = "
				 "0x%x\n", dwErrInfo);
			break;
		}
		/* Signal DSP error/exception event. */
		NTFY_Notify(pDehMgr->hNtfy, ulEventMask);
	}
	DBG_Trace(DBG_LEVEL1, "Exiting WMD_DEH_Notify\n");

}

/*
 *  ======== WMD_DEH_GetInfo ========
 *  purpose:
 *      Retrieves error information.
 */
DSP_STATUS WMD_DEH_GetInfo(struct DEH_MGR *hDehMgr,
			  struct DSP_ERRORINFO *pErrInfo)
{
	DSP_STATUS status = DSP_SOK;
	struct DEH_MGR *pDehMgr = (struct DEH_MGR *)hDehMgr;

	DBC_Require(pDehMgr);
	DBC_Require(pErrInfo);

	DBG_Trace(DBG_LEVEL1, "Entering WMD_DEH_GetInfo: 0x%x\n", hDehMgr);

	if (MEM_IsValidHandle(pDehMgr, SIGNATURE)) {
		/* Copy DEH error info structure to PROC error info
		 * structure. */
		pErrInfo->dwErrMask = pDehMgr->errInfo.dwErrMask;
		pErrInfo->dwVal1 = pDehMgr->errInfo.dwVal1;
		pErrInfo->dwVal2 = pDehMgr->errInfo.dwVal2;
		pErrInfo->dwVal3 = pDehMgr->errInfo.dwVal3;
	}

	DBG_Trace(DBG_LEVEL1, "Exiting WMD_DEH_GetInfo\n");

	return (status);
}

