/*
 * dspbridge/src/pmgr/linux/common/wcd.c
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
 *  ======== wcd.c ========
 *  Description:
 *      Common WCD functions, also includes the wrapper
 *      functions called directly by the DeviceIOControl interface.
 *
 *  Public Functions:
 *      WCD_CallDevIOCtl
 *      WCD_Init
 *      WCD_InitComplete2
 *      WCD_Exit
 *      <MOD>WRAP_*
 *
 *! Revision History:
 *! ================
 *! 29-Apr-2004 hp  Call PROC_AutoStart only for DSP device
 *! 19-Apr-2004 sb  Aligned DMM definitions with Symbian
 *! 08-Mar-2004 sb  Added the Dynamic Memory Mapping APIs
 *! 03-Apr-2003 sb  Process environment pointer in PROCWRAP_Load
 *! 24-Feb-2003 swa PMGR Code review comments incorporated.
 *! 30-Jan-2002 ag  CMMWRAP_AllocBuf name changed to CMMWRAP_CallocBuf
 *! 15-Jan-2002 ag  Added actual bufSize param to STRMWRAP_Reclaim[issue].
 *! 14-Dec-2001 rr  ARGS_NODE_CONNECT maps the pAttr.
 *! 03-Oct-2001 rr  ARGS_NODE_ALLOCMSGBUF/FREEMSGBUF maps the pAttr.
 *! 10-Sep-2001 ag  Added CMD_CMM_GETHANDLE.
 *! 23-Apr-2001 jeh Pass pStatus to NODE_Terminate.
 *! 11-Apr-2001 jeh STRMWRAP_Reclaim embedded pointer is mapped and unmapped.
 *! 13-Feb-2001 kc: DSP/BIOS Bridge name updates.
 *! 06-Dec-2000 jeh WRAP_MAP2CALLER pointers in RegisterNotify calls.
 *! 05-Dec-2000 ag: Removed MAP2CALLER in NODEWRAP_FreeMsgBuf().
 *! 22-Nov-2000 kc: Added MGRWRAP_GetPerf_Data().
 *! 20-Nov-2000 jeh Added MSG_Init()/MSG_Exit(), IO_Init()/IO_Exit().
 *!		 WRAP pointers to handles for PROC_Attach, NODE_Allocate.
 *! 27-Oct-2000 jeh Added NODEWRAP_AllocMsgBuf, NODEWRAP_FreeMsgBuf. Removed
 *!		 NODEWRAP_GetMessageStream.
 *! 12-Oct-2000 ag: Added user CMM wrappers.
 *! 05-Oct-2000 rr: WcdInitComplete2 will fail even if one BRD or PROC
 *!		 AutoStart fails.
 *! 25-Sep-2000 rr: Updated to Version 0.9
 *! 13-Sep-2000 jeh Pass ARGS_NODE_CONNECT.pAttrs to NODE_Connect().
 *! 11-Aug-2000 rr: Part of node enabled.
 *! 31-Jul-2000 rr: UTIL_Wrap and MEM_Wrap added to RM.
 *! 27-Jul-2000 rr: PROCWRAP, NODEWRAP and STRMWRAP implemented.
 *!		 STRM and some NODE Wrappers are not implemented.
 *! 27-Jun-2000 rr: MGRWRAP fxns added.IFDEF to build for PM or DSP/BIOS Bridge
 *! 08-Feb-2000 rr  File name changed to wcd.c
 *! 03-Feb-2000 rr: Module initialization are done by OSAL init. GT Class
 *!		 changes for module init/exit fxns.
 *! 24-Jan-2000 rr: Merged with Scott's code.
 *! 21-Jan-1999 sg: Changed ARGS_CHNL_GETMODE field name from pdwMode to pMode.
 *! 17-Jan-2000 rr: BRD_GetStatus does WRAP_MAP2CALLER for state.
 *! 14-Dec-1999 ag: Removed _MAP2CALLER in CHNL_GetMgr().
 *! 13-Dec-1999 rr: BRDWRAP_GetSymbol, BRDWRAP_GetTrace uses WRAP_MAP2CALLER
 *!		 macros.BRDWRAP_Load maps and unmaps embedded pointers.
 *! 10-Dec-1999 ag: User CHNL bufs mapped in _AddIOReq & _GetIOCompletion.
 *! 09-Dec-1999 rr: BRDWRAP_Open and CHNLWRAP_GetMgr does not map
 *!		 pointer as there was a change in config.c
 *! 06-Dec-1999 rr: BRD_Read and Write Maps the buf pointers.
 *! 03-Dec-1999 rr: CHNLWRAP_GetMgr and BRDWRAP_Open maps  hDevNode pointer.
 *!		 WCD_InitComplete2 Included for BRD_AutoStart.
 *! 16-Nov-1999 ag: Map buf to process in CHNLWRAP_AllocBuffer().
 *!		 CHNL_GetMgr() Mapping Fix.
 *! 10-Nov-1999 ag: Removed unnecessary calls to WRAP_MAP2CALLER.
 *! 08-Nov-1999 kc: Added MEMRY & enabled BRD_IOCtl for tests.
 *! 29-Oct-1999 ag: Added CHNL.
 *! 29-Oct-1999 kc: Added trace statements; added ptr mapping; updated
 *!		 use of UTIL module API.
 *! 29-Oct-1999 rr: Wrapper functions does the Mapping of the Pointers.
 *!		 in WinCE all the explicit pointers will be converted
 *!		 by the OS during interprocess but not the embedded pointers.
 *! 16-Oct-1999 kc: Code review cleanup.
 *! 07-Oct-1999 kc: Added UTILWRAP_TestDll() to run PM test harness. See
 *!		 /src/doc/pmtest.doc for more detail.
 *! 09-Sep-1999 rr: After exactly two years(!). Adopted for WinCE. GT Enabled.
 *! 09-Sep-1997 gp: Created.
 */

/*  ----------------------------------- Host OS */
#include <host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <std.h>
#include <dbdefs.h>
#include <errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <dbc.h>
#include <gt.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <cfg.h>
#include <mem.h>
#include <ntfy.h>
#include <osal.h>
#include <util.h>

/*  ----------------------------------- Platform Manager */
#include <chnl.h>
#include <dev.h>
#include <drv.h>

#include <proc.h>
#include <strm.h>

/*  ----------------------------------- Resource Manager */
#include <disp.h>
#include <mgr.h>
#include <node.h>
#include <rmm.h>


/*  ----------------------------------- Others */
#include <msg.h>
#include <cmm.h>
#include <io.h>

/*  ----------------------------------- This */
#include <_dcd.h>
#include <dbdcd.h>

#ifndef RES_CLEANUP_DISABLE
#include <resourcecleanup.h>
#endif

/*  ----------------------------------- Defines, Data Structures, Typedefs */
#define MAX_TRACEBUFLEN 255
#define MAX_LOADARGS    16
#define MAX_NODES       64
#define MAX_STREAMS     16
#define MAX_BUFS	64

/* Following two macros should ideally have do{}while(0) */

#define cp_fm_usr(dest, src, status, elements)    \
    if (DSP_SUCCEEDED(status)) {\
	if ((src == NULL) || \
		   copy_from_user(dest, src, elements * sizeof(*(dest)))) {\
	   GT_1trace(WCD_debugMask, GT_7CLASS, \
		    "copy_from_user failed, src=0x%x\n", src);  \
	    status = DSP_EPOINTER ; \
	} \
    }

#define cp_to_usr(dest, src, status, elements)    \
    if (DSP_SUCCEEDED(status)) {\
	if ((dest == NULL) || \
		   copy_to_user(dest, src, elements * sizeof(*(src)))) { \
	   GT_1trace(WCD_debugMask, GT_7CLASS, \
		    "copy_to_user failed, dest=0x%x\n", dest); \
	    status = DSP_EPOINTER ;\
	} \
    }

/* Device IOCtl function pointer */
struct WCD_Cmd {
	DWORD(*fxn)(Trapped_Args *args);
	DWORD dwIndex;
} ;

/*  ----------------------------------- Globals */
struct GT_Mask WCD_debugMask = { 0, 0 };	/* Core VxD Mask */
ULONG WCD_cRefs = 0;

/*
 *  Function table.
 *  The order of these functions MUST be the same as the order of the command
 *  numbers defined in wcdioctl.h  This is how an IOCTL number in user mode
 *  turns into a function call in kernel mode.
 */
struct WCD_Cmd WCD_cmdTable[] = {
	/* MGR module */
	{MGRWRAP_EnumNode_Info, CMD_MGR_ENUMNODE_INFO_OFFSET},
	{MGRWRAP_EnumProc_Info, CMD_MGR_ENUMPROC_INFO_OFFSET},
	{MGRWRAP_RegisterObject, CMD_MGR_REGISTEROBJECT_OFFSET},
	{MGRWRAP_UnregisterObject, CMD_MGR_UNREGISTEROBJECT_OFFSET},
	{MGRWRAP_WaitForBridgeEvents, CMD_MGR_WAIT_OFFSET},
#ifndef RES_CLEANUP_DISABLE
	{MGRWRAP_GetProcessResourcesInfo, CMD_MGR_RESOUCES_OFFSET},
#endif
	/* PROC Module */
	{PROCWRAP_Attach, CMD_PROC_ATTACH_OFFSET},
	{PROCWRAP_Ctrl, CMD_PROC_CTRL_OFFSET},
	{PROCWRAP_Detach, CMD_PROC_DETACH_OFFSET},
	{PROCWRAP_EnumNode_Info, CMD_PROC_ENUMNODE_OFFSET},
	{PROCWRAP_EnumResources, CMD_PROC_ENUMRESOURCES_OFFSET},
	{PROCWRAP_GetState, CMD_PROC_GETSTATE_OFFSET},
	{PROCWRAP_GetTrace, CMD_PROC_GETTRACE_OFFSET},
	{PROCWRAP_Load, CMD_PROC_LOAD_OFFSET},
	{PROCWRAP_RegisterNotify, CMD_PROC_REGISTERNOTIFY_OFFSET},
	{PROCWRAP_Start, CMD_PROC_START_OFFSET},
	{PROCWRAP_ReserveMemory, CMD_PROC_RSVMEM_OFFSET},
	{PROCWRAP_UnReserveMemory, CMD_PROC_UNRSVMEM_OFFSET},
	{PROCWRAP_Map, CMD_PROC_MAPMEM_OFFSET},
	{PROCWRAP_UnMap, CMD_PROC_UNMAPMEM_OFFSET},
	{PROCWRAP_FlushMemory, CMD_PROC_FLUSHMEMORY_OFFSET},
	{PROCWRAP_Stop, CMD_PROC_STOP_OFFSET},
	{PROCWRAP_InvalidateMemory, CMD_PROC_INVALIDATEMEMORY_OFFSET},
	/* NODE Module */
	{NODEWRAP_Allocate, CMD_NODE_ALLOCATE_OFFSET},
	{NODEWRAP_AllocMsgBuf, CMD_NODE_ALLOCMSGBUF_OFFSET},
	{NODEWRAP_ChangePriority, CMD_NODE_CHANGEPRIORITY_OFFSET},
	{NODEWRAP_Connect, CMD_NODE_CONNECT_OFFSET},
	{NODEWRAP_Create, CMD_NODE_CREATE_OFFSET},
	{NODEWRAP_Delete, CMD_NODE_DELETE_OFFSET},
	{NODEWRAP_FreeMsgBuf, CMD_NODE_FREEMSGBUF_OFFSET},
	{NODEWRAP_GetAttr, CMD_NODE_GETATTR_OFFSET},
	{NODEWRAP_GetMessage, CMD_NODE_GETMESSAGE_OFFSET},
	{NODEWRAP_Pause, CMD_NODE_PAUSE_OFFSET},
	{NODEWRAP_PutMessage, CMD_NODE_PUTMESSAGE_OFFSET},
	{NODEWRAP_RegisterNotify, CMD_NODE_REGISTERNOTIFY_OFFSET},
	{NODEWRAP_Run, CMD_NODE_RUN_OFFSET},
	{NODEWRAP_Terminate, CMD_NODE_TERMINATE_OFFSET},
	{NODEWRAP_GetUUIDProps, CMD_NODE_GETUUIDPROPS_OFFSET},
	/* STRM wrapper functions */
	{STRMWRAP_AllocateBuffer, CMD_STRM_ALLOCATEBUFFER_OFFSET},
	{STRMWRAP_Close, CMD_STRM_CLOSE_OFFSET},
	{STRMWRAP_FreeBuffer, CMD_STRM_FREEBUFFER_OFFSET},
	{STRMWRAP_GetEventHandle, CMD_STRM_GETEVENTHANDLE_OFFSET},
	{STRMWRAP_GetInfo, CMD_STRM_GETINFO_OFFSET},
	{STRMWRAP_Idle, CMD_STRM_IDLE_OFFSET},
	{STRMWRAP_Issue, CMD_STRM_ISSUE_OFFSET},
	{STRMWRAP_Open, CMD_STRM_OPEN_OFFSET},
	{STRMWRAP_Reclaim, CMD_STRM_RECLAIM_OFFSET},
	{STRMWRAP_RegisterNotify, CMD_STRM_REGISTERNOTIFY_OFFSET},
	{STRMWRAP_Select, CMD_STRM_SELECT_OFFSET},
	/* CMM module */
	{CMMWRAP_CallocBuf, CMD_CMM_ALLOCBUF_OFFSET},
	{CMMWRAP_FreeBuf, CMD_CMM_FREEBUF_OFFSET},
	{CMMWRAP_GetHandle, CMD_CMM_GETHANDLE_OFFSET},
	{CMMWRAP_GetInfo, CMD_CMM_GETINFO_OFFSET},
	/* MEM module: */
	{MEMWRAP_Alloc, CMD_MEM_ALLOC_OFFSET},
	{MEMWRAP_Calloc, CMD_MEM_CALLOC_OFFSET},
	{MEMWRAP_Free, CMD_MEM_FREE_OFFSET},
	{MEMWRAP_PageLock, CMD_MEM_PAGELOCK_OFFSET},
	{MEMWRAP_PageUnlock, CMD_MEM_PAGEUNLOCK_OFFSET},
	/* UTIL module */
	{UTILWRAP_TestDll, CMD_UTIL_TESTDLL_OFFSET},  /* for PM test harness */
};

/*
 *  ======== WCD_CallDevIOCtl ========
 *  Purpose:
 *      Call the (wrapper) function for the corresponding WCD IOCTL.
 */
inline DSP_STATUS WCD_CallDevIOCtl(UINT cmd, Trapped_Args *args,
				    DWORD *pResult)
{
	if ((cmd < (sizeof(WCD_cmdTable) / sizeof(struct WCD_Cmd)))) {
		/* make the fxn call via the cmd table */
		*pResult = (*WCD_cmdTable[cmd].fxn) (args);
		return (DSP_SOK);
	} else {
		return (DSP_EINVALIDARG);
	}
}

/*
 *  ======== WCD_Exit ========
 */
VOID WCD_Exit()
{
	DBC_Require(WCD_cRefs > 0);
	WCD_cRefs--;
	GT_1trace(WCD_debugMask, GT_5CLASS,
		 "Entered WCD_Exit, ref count:  0x%x\n", WCD_cRefs);
	if (WCD_cRefs == 0) {
		/* Release all WCD modules initialized in WCD_Init(). */
		COD_Exit();
		DEV_Exit();
		CHNL_Exit();
		MSG_Exit();
		IO_Exit();
		STRM_Exit();
		NTFY_Exit();
		DISP_Exit();
		NODE_Exit();
		PROC_Exit();
		MGR_Exit();
		RMM_exit();
		DRV_Exit();
		OSAL_Exit();
	}
	DBC_Ensure(WCD_cRefs >= 0);
}

/*
 *  ======== WCD_Init ========
 *  Purpose:
 *      Module initialization is done by OSAL Init.
 */
BOOL WCD_Init()
{
	BOOL fInit = TRUE;
	BOOL fDRV, fDEV, fCOD, fOSAL, fCHNL, fMSG, fIO;
	BOOL fMGR, fPROC, fNODE, fDISP, fNTFY, fSTRM, fRMM;
#ifdef DEBUG
	/* runtime check of Device IOCtl array. */
	DWORD i;
	for (i = 1; i < (sizeof(WCD_cmdTable) / sizeof(struct WCD_Cmd)); i++)
		DBC_Assert(WCD_cmdTable[i - 1].dwIndex == i);

#endif
	if (WCD_cRefs == 0) {
		/* initialize all OSAL modules */
		fOSAL = OSAL_Init();
		/* initialize debugging module */
		DBC_Assert(!WCD_debugMask.flags);
		GT_create(&WCD_debugMask, "CD");    /* CD for class driver */
		/* initialize class driver and other modules */
		fDRV = DRV_Init();
		fMGR = MGR_Init();
		fPROC = PROC_Init();
		fNODE = NODE_Init();
		fDISP = DISP_Init();
		fNTFY = NTFY_Init();
		fSTRM = STRM_Init();
		fRMM = RMM_init();
		fCHNL = CHNL_Init();
		fMSG = MSG_Init();
		fIO = IO_Init();
		fDEV = DEV_Init();
		fCOD = COD_Init();
		fInit = fOSAL && fDRV && fDEV && fCHNL && fCOD && fMSG && fIO;
		fInit = fInit && fMGR && fPROC && fRMM;
		if (!fInit) {
			if (fOSAL)
				OSAL_Exit();

			if (fDRV)
				DRV_Exit();

			if (fMGR)
				MGR_Exit();

			if (fSTRM)
				STRM_Exit();

			if (fPROC)
				PROC_Exit();

			if (fNODE)
				NODE_Exit();

			if (fDISP)
				DISP_Exit();

			if (fNTFY)
				NTFY_Exit();

			if (fCHNL)
				CHNL_Exit();

			if (fMSG)
				MSG_Exit();

			if (fIO)
				IO_Exit();

			if (fDEV)
				DEV_Exit();

			if (fCOD)
				COD_Exit();

			if (fRMM)
				RMM_exit();

		}
	}
	if (fInit)
		WCD_cRefs++;

	GT_1trace(WCD_debugMask, GT_5CLASS,
		 "Entered WCD_Init, ref count: 0x%x\n",	WCD_cRefs);
	return (fInit);
}

/*
 *  ======== WCD_InitComplete2 ========
 *  Purpose:
 *      Perform any required WCD, and WMD initialization which
 *      cannot not be performed in WCD_Init() or DEV_StartDevice() due
 *      to the fact that some services are not yet
 *      completely initialized.
 *  Parameters:
 *  Returns:
 *      DSP_SOK:	Allow this device to load
 *      DSP_EFAIL:      Failure.
 *  Requires:
 *      WCD initialized.
 *  Ensures:
 */
DSP_STATUS WCD_InitComplete2()
{
	DSP_STATUS status = DSP_SOK;
	struct CFG_DEVNODE *DevNode;
	struct DEV_OBJECT *hDevObject;
	INT devType;

	DBC_Require(WCD_cRefs > 0);
	GT_0trace(WCD_debugMask, GT_ENTER, "Entered WCD_InitComplete\n");
	 /*  Walk the list of DevObjects, get each devnode, and attempting to
	 *  autostart the board. Note that this requires COF loading, which
	 *  requires KFILE.  */
	for (hDevObject = DEV_GetFirst(); hDevObject != NULL;
	     hDevObject = DEV_GetNext(hDevObject)) {
		if (!DSP_SUCCEEDED(DEV_GetDevNode(hDevObject, &DevNode)))
			continue;

		if (!DSP_SUCCEEDED(DEV_GetDevType(hDevObject, &devType)))
			continue;

		if ((devType == DSP_UNIT) || (devType == IVA_UNIT)) {
			if (DSP_FAILED(PROC_AutoStart(DevNode, hDevObject))) {
				GT_0trace(WCD_debugMask, GT_1CLASS,
					 "WCD_InitComplete2 Failed\n");
				status = DSP_EFAIL;
				/* break; */
			}
		} else
			GT_1trace(WCD_debugMask, GT_ENTER,
				 "Ignoring PROC_AutoStart "
				 "for Device Type = 0x%x \n", devType);
	}			/* End For Loop */
	GT_1trace(WCD_debugMask, GT_ENTER,
		 "Exiting WCD_InitComplete status 0x%x\n", status);
	return (status);
}

/*
 * ======== MGRWRAP_EnumNode_Info ========
 */
DWORD MGRWRAP_EnumNode_Info(Trapped_Args *args)
{
	BYTE *pNDBProps;
	UINT uNumNodes;
	DSP_STATUS status = DSP_SOK;
	UINT size = args->ARGS_MGR_ENUMNODE_INFO.uNDBPropsSize;

	GT_4trace(WCD_debugMask, GT_ENTER,
		 "MGR_EnumNodeInfo: entered args:\n0x%x"
		 " uNode: 0x%x\tpNDBProps: 0x%x\tuNDBPropsSize: "
		 "0x%x\tpuNumNodes\n", args->ARGS_MGR_ENUMNODE_INFO.uNode,
		 args->ARGS_MGR_ENUMNODE_INFO.pNDBProps,
		 args->ARGS_MGR_ENUMNODE_INFO.uNDBPropsSize,
		 args->ARGS_MGR_ENUMNODE_INFO.puNumNodes);
	pNDBProps = MEM_Alloc(size, MEM_NONPAGED);
	if (pNDBProps == NULL)
		status = DSP_EMEMORY;

	if (DSP_SUCCEEDED(status)) {
		status = MGR_EnumNodeInfo(args->ARGS_MGR_ENUMNODE_INFO.uNode,
					 (struct DSP_NDBPROPS *)pNDBProps,
					 size, &uNumNodes);
	}
	cp_to_usr(args->ARGS_MGR_ENUMNODE_INFO.pNDBProps, pNDBProps, status,
		 size);
	cp_to_usr(args->ARGS_MGR_ENUMNODE_INFO.puNumNodes, &uNumNodes, status,
		 1);
	if (pNDBProps)
		MEM_Free(pNDBProps);

	return status;
}

/*
 * ======== MGRWRAP_EnumProc_Info ========
 */
DWORD MGRWRAP_EnumProc_Info(Trapped_Args *args)
{
	BYTE *pProcessorInfo;
	UINT uNumProcs;
	DSP_STATUS status = DSP_SOK;
	UINT size = args->ARGS_MGR_ENUMPROC_INFO.uProcessorInfoSize;

	GT_4trace(WCD_debugMask, GT_ENTER,
		 "MGRWRAP_EnumProc_Info: entered args:\n"
		 "0x%x uProcessor: 0x%x\tpProcessorInfo: 0x%x\t"
		 "uProcessorInfoSize: 0x%x\tpuNumProcs \n",
		 args->ARGS_MGR_ENUMPROC_INFO.uProcessor,
		 args->ARGS_MGR_ENUMPROC_INFO.pProcessorInfo,
		 args->ARGS_MGR_ENUMPROC_INFO.uProcessorInfoSize,
		 args->ARGS_MGR_ENUMPROC_INFO.puNumProcs);
	pProcessorInfo = MEM_Alloc(size, MEM_NONPAGED);
	if (pProcessorInfo == NULL)
		status = DSP_EMEMORY;

	if (DSP_SUCCEEDED(status)) {
		status = MGR_EnumProcessorInfo(args->
				ARGS_MGR_ENUMPROC_INFO.uProcessor,
				(struct DSP_PROCESSORINFO *)pProcessorInfo,
				size, &uNumProcs);
	}
	cp_to_usr(args->ARGS_MGR_ENUMPROC_INFO.pProcessorInfo, pProcessorInfo,
		 status, size);
	cp_to_usr(args->ARGS_MGR_ENUMPROC_INFO.puNumProcs, &uNumProcs,
		 status, 1);
	if (pProcessorInfo)
		MEM_Free(pProcessorInfo);

	return status;
}

#define WRAP_MAP2CALLER(x) x
/*
 * ======== MGRWRAP_RegisterObject ========
 */
DWORD MGRWRAP_RegisterObject(Trapped_Args *args)
{
	GT_1trace(WCD_debugMask, GT_ENTER,
		 "MGRWRAP_RegisterObject: entered pg2hMsg "
		 "0x%x\n", args->ARGS_MGR_REGISTEROBJECT.pUuid);
	return (DCD_RegisterObject(WRAP_MAP2CALLER
	       (args->ARGS_MGR_REGISTEROBJECT.pUuid),
	       args->ARGS_MGR_REGISTEROBJECT.objType,
	       WRAP_MAP2CALLER(args->ARGS_MGR_REGISTEROBJECT.pszPathName)));
}

/*
 * ======== MGRWRAP_UnregisterObject ========
 */
DWORD MGRWRAP_UnregisterObject(Trapped_Args *args)
{
	GT_1trace(WCD_debugMask, GT_ENTER,
		 "MGRWRAP_UnregisterObject: entered pg2hMsg"
		 " 0x%x\n", args->ARGS_MGR_UNREGISTEROBJECT.pUuid);
	return (DCD_UnregisterObject
		(WRAP_MAP2CALLER(args->ARGS_MGR_UNREGISTEROBJECT.pUuid),
		 args->ARGS_MGR_UNREGISTEROBJECT.objType));
}

/*
 * ======== MGRWRAP_WaitForBridgeEvents ========
 */
DWORD MGRWRAP_WaitForBridgeEvents(Trapped_Args *args)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_NOTIFICATION *aNotifications[MAX_EVENTS];
	struct DSP_NOTIFICATION notifications[MAX_EVENTS];
	UINT uIndex, i;
	UINT uCount = args->ARGS_MGR_WAIT.uCount;

	GT_0trace(WCD_debugMask, GT_ENTER,
		 "MGRWRAP_WaitForBridgeEvents: entered\n");

	if (uCount > MAX_EVENTS)
		status = DSP_EINVALIDARG;

	/* get the array of pointers to user structures */
	cp_fm_usr(aNotifications, args->ARGS_MGR_WAIT.aNotifications,
		 status, uCount);
	/* get the events */
	for (i = 0; i < uCount; i++) {
		cp_fm_usr(&notifications[i], aNotifications[i], status, 1);
		if (DSP_SUCCEEDED(status)) {
			/* set the array of pointers to kernel structures*/
			aNotifications[i] = &notifications[i];
		}
	}
	if (DSP_SUCCEEDED(status)) {
		status = MGR_WaitForBridgeEvents(aNotifications, uCount,
			 &uIndex, args->ARGS_MGR_WAIT.uTimeout);
	}
	cp_to_usr(args->ARGS_MGR_WAIT.puIndex, &uIndex, status, 1);
	return (status);
}


#ifndef RES_CLEANUP_DISABLE
/*
 * ======== MGRWRAP_GetProcessResourceInfo ========
 */
DWORD MGRWRAP_GetProcessResourcesInfo(Trapped_Args *args)
{
    DSP_STATUS status = DSP_SOK;
    UINT uSize = 0;
    BYTE *pBuf = MEM_Alloc(8092, MEM_NONPAGED);
    UINT i = 0;
    status = DRV_ProcDisplayResInfo(pBuf, &uSize);
    GT_1trace(WCD_debugMask, GT_ENTER,
	     "MGRWRAP_GetProcessResourcesInfo:uSize=%d :\n", uSize);
    cp_to_usr(args->ARGS_PROC_GETTRACE.pBuf, pBuf, status, uSize);
    GT_0trace(WCD_debugMask, GT_ENTER, "\n***********"
	     "123MGRWRAP_GetProcessResourcesInfo:**************\n");
    GT_0trace(WCD_debugMask, GT_ENTER, "\n***********"
	     "456MGRWRAP_GetProcessResourcesInfo:**************\n");
    cp_to_usr(args->ARGS_PROC_GETTRACE.pSize, &uSize, status, 1);
    MEM_Free(pBuf);
    return (status);
}
#endif


/*
 * ======== PROCWRAP_Attach ========
 */
DWORD PROCWRAP_Attach(Trapped_Args *args)
{
	DSP_HPROCESSOR processor;
	DSP_STATUS status = DSP_SOK;
	struct DSP_PROCESSORATTRIN attrIn, *pAttrIn = NULL;

	GT_3trace(WCD_debugMask, GT_ENTER,
		 "PROCWRAP_Attach: entered args:\n" "0x%x"
		 " uProcessor: 0x%x\tpAttrIn: 0x%x\tphProcessor \n",
		 args->ARGS_PROC_ATTACH.uProcessor,
		 args->ARGS_PROC_ATTACH.pAttrIn,
		 args->ARGS_PROC_ATTACH.phProcessor);
	/* Optional argument */
	if (args->ARGS_PROC_ATTACH.pAttrIn) {
		cp_fm_usr(&attrIn, args->ARGS_PROC_ATTACH.pAttrIn, status, 1);
		if (DSP_SUCCEEDED(status))
			pAttrIn = &attrIn;

	}
	status = PROC_Attach(args->ARGS_PROC_ATTACH.uProcessor, pAttrIn,
			    &processor);
	cp_to_usr(args->ARGS_PROC_ATTACH.phProcessor, &processor, status, 1);
	return (status);
}

/*
 * ======== PROCWRAP_Ctrl ========
 */
DWORD PROCWRAP_Ctrl(Trapped_Args *args)
{
	ULONG cbDataSize, *pSize = (ULONG *)args->ARGS_PROC_CTRL.pArgs;
	BYTE *pArgs = NULL;
	DSP_STATUS status = DSP_SOK;

	GT_3trace(WCD_debugMask, GT_ENTER,
		 "PROCWRAP_Ctrl: entered args:\n 0x%x"
		 " uProcessor: 0x%x\tdwCmd: 0x%x\tpArgs \n",
		 args->ARGS_PROC_CTRL.hProcessor,
		 args->ARGS_PROC_CTRL.dwCmd,
		 args->ARGS_PROC_CTRL.pArgs);
	if (pSize) {
		if (get_user(cbDataSize, pSize))
			status = DSP_EFAIL;

		cbDataSize += sizeof(ULONG);
		if (DSP_SUCCEEDED(status)) {
			pArgs = MEM_Alloc(cbDataSize, MEM_NONPAGED);
			if (pArgs == NULL)
				status = DSP_EMEMORY;

		}
		cp_fm_usr(pArgs, args->ARGS_PROC_CTRL.pArgs, status,
			 cbDataSize);
	}
	if (DSP_SUCCEEDED(status)) {
		status = PROC_Ctrl(args->ARGS_PROC_CTRL.hProcessor,
				  args->ARGS_PROC_CTRL.dwCmd,
				  (struct DSP_CBDATA *)pArgs);
	}

	/* cp_to_usr(args->ARGS_PROC_CTRL.pArgs, pArgs, status, 1);*/
	if (pArgs)
		MEM_Free(pArgs);

	return (status);
}

/*
 * ======== PROCWRAP_Detach ========
 */
DWORD PROCWRAP_Detach(Trapped_Args *args)
{
	GT_1trace(WCD_debugMask, GT_ENTER,
		 "PROCWRAP_Detach: entered args\n0x%x "
		 "hProceesor \n", args->ARGS_PROC_DETACH.hProcessor);
	return (PROC_Detach(args->ARGS_PROC_DETACH.hProcessor));
}

/*
 * ======== PROCWRAP_EnumNode_Info ========
 */
DWORD PROCWRAP_EnumNode_Info(Trapped_Args *args)
{
	DSP_STATUS status;
	DSP_HNODE aNodeTab[MAX_NODES];
	UINT uNumNodes;
	UINT uAllocated;

	GT_5trace(WCD_debugMask, GT_ENTER,
		 "PROCWRAP_EnumNode_Info:entered args:\n0x"
		 "%xhProcessor:0x%x\taNodeTab:0x%x\tuNodeTabSize:"
		 "%0x%x\tpuNumNodes%\n0x%x puAllocated: \n",
		 args->ARGS_PROC_ENUMNODE_INFO.hProcessor,
		 args->ARGS_PROC_ENUMNODE_INFO.aNodeTab,
		 args->ARGS_PROC_ENUMNODE_INFO.uNodeTabSize,
		 args->ARGS_PROC_ENUMNODE_INFO.puNumNodes,
		 args->ARGS_PROC_ENUMNODE_INFO.puAllocated);
	DBC_Require(args->ARGS_PROC_ENUMNODE_INFO.uNodeTabSize <= MAX_NODES);
	status = PROC_EnumNodes(args->ARGS_PROC_ENUMNODE_INFO.hProcessor,
				aNodeTab,
				args->ARGS_PROC_ENUMNODE_INFO.uNodeTabSize,
				&uNumNodes, &uAllocated);
	cp_to_usr(args->ARGS_PROC_ENUMNODE_INFO.aNodeTab, aNodeTab, status,
		 uNumNodes);
	cp_to_usr(args->ARGS_PROC_ENUMNODE_INFO.puNumNodes, &uNumNodes,
		 status, 1);
	cp_to_usr(args->ARGS_PROC_ENUMNODE_INFO.puAllocated, &uAllocated,
		 status, 1);
	return (status);
}

/*
 * ======== PROCWRAP_FlushMemory ========
 */
DWORD PROCWRAP_FlushMemory(Trapped_Args *args)
{
	DSP_STATUS status;

	GT_0trace(WCD_debugMask, GT_ENTER, "PROCWRAP_FlushMemory: entered\n");

	status = PROC_FlushMemory(args->ARGS_PROC_FLUSHMEMORY.hProcessor,
				 args->ARGS_PROC_FLUSHMEMORY.pMpuAddr,
				 args->ARGS_PROC_FLUSHMEMORY.ulSize,
				 args->ARGS_PROC_FLUSHMEMORY.ulFlags);
	return (status);
}


/*
 * ======== PROCWRAP_InvalidateMemory ========
 */
DWORD PROCWRAP_InvalidateMemory(Trapped_Args *args)
{
	DSP_STATUS status;

	GT_0trace(WCD_debugMask, GT_ENTER,
		 "PROCWRAP_InvalidateMemory:entered\n");

	status = PROC_InvalidateMemory(
				  args->ARGS_PROC_INVALIDATEMEMORY.hProcessor,
				  args->ARGS_PROC_INVALIDATEMEMORY.pMpuAddr,
				  args->ARGS_PROC_INVALIDATEMEMORY.ulSize);
	return (status);
}


/*
 * ======== PROCWRAP_EnumResources ========
 */
DWORD PROCWRAP_EnumResources(Trapped_Args *args)
{
	GT_4trace(WCD_debugMask, GT_ENTER,
		 "PROCWRAP_EnumResources: entered args:\n"
		 "0x%x hProcessor: 0x%x\tuResourceMask: 0x%x\tpResourceInfo"
		 " 0x%x\tuResourceInfoSixe \n",
		 args->ARGS_PROC_ENUMRESOURCES.hProcessor,
		 args->ARGS_PROC_ENUMRESOURCES.uResourceType,
		 args->ARGS_PROC_ENUMRESOURCES.pResourceInfo,
		 args->ARGS_PROC_ENUMRESOURCES.uResourceInfoSize);
	return (PROC_GetResourceInfo(args->ARGS_PROC_ENUMRESOURCES.hProcessor,
			     args->ARGS_PROC_ENUMRESOURCES.uResourceType,
			     WRAP_MAP2CALLER(args->ARGS_PROC_ENUMRESOURCES.
					    pResourceInfo),
			     args->ARGS_PROC_ENUMRESOURCES.uResourceInfoSize));
}

/*
 * ======== PROCWRAP_GetState ========
 */
DWORD PROCWRAP_GetState(Trapped_Args *args)
{
	DSP_STATUS status;
	struct DSP_PROCESSORSTATE procStatus;
	GT_0trace(WCD_debugMask, GT_ENTER, "PROCWRAP_GetState: entered\n");
	status = PROC_GetState(args->ARGS_PROC_GETSTATE.hProcessor, &procStatus,
			      args->ARGS_PROC_GETSTATE.uStateInfoSize);
	cp_to_usr(args->ARGS_PROC_GETSTATE.pProcStatus, &procStatus, status, 1);
	return (status);

}

/*
 * ======== PROCWRAP_GetTrace ========
 */
DWORD PROCWRAP_GetTrace(Trapped_Args *args)
{
	DSP_STATUS status;
	BYTE *pBuf;

	GT_0trace(WCD_debugMask, GT_ENTER, "PROCWRAP_GetTrace: entered\n");

	DBC_Require(args->ARGS_PROC_GETTRACE.uMaxSize <= MAX_TRACEBUFLEN);

	pBuf = MEM_Calloc(args->ARGS_PROC_GETTRACE.uMaxSize, MEM_NONPAGED);
	if (pBuf != NULL) {
		status = PROC_GetTrace(args->ARGS_PROC_GETTRACE.hProcessor,
				      pBuf, args->ARGS_PROC_GETTRACE.uMaxSize);
	} else {
		status = DSP_EMEMORY;
	}
	cp_to_usr(args->ARGS_PROC_GETTRACE.pBuf, pBuf, status,
		 args->ARGS_PROC_GETTRACE.uMaxSize);
	if (pBuf)
		MEM_Free(pBuf);

	return (status);
}

/*
 * ======== PROCWRAP_Load ========
 */
DWORD PROCWRAP_Load(Trapped_Args *args)
{
	INT i, len;
	DSP_STATUS status = DSP_SOK;
	BYTE *temp;
	INT argc = args->ARGS_PROC_LOAD.iArgc;
	BYTE **argv, **envp = NULL;

	DBC_Require(argc > 0);
	DBC_Require(argc <= MAX_LOADARGS);

	argv = MEM_Alloc(argc * sizeof(BYTE *), MEM_NONPAGED);
	if (argv == NULL)
		status = DSP_EMEMORY;

	cp_fm_usr(argv, args->ARGS_PROC_LOAD.aArgv, status, argc);
	for (i = 0; DSP_SUCCEEDED(status) && (i < argc); i++) {
		if (argv[i] != NULL) {
			temp = argv[i];	/* User space pointer to argument */
			len = strlen_user(temp);
			/* Kernel space pointer to argument */
			argv[i] = MEM_Alloc(len, MEM_NONPAGED);
			if (argv[i] == NULL) {
				status = DSP_EMEMORY;
				break;
			}
			cp_fm_usr(argv[i], temp, status, len);
		}
	}
	/* TODO: validate this */
	if (args->ARGS_PROC_LOAD.aEnvp != NULL) {
		/* number of elements in the envp array including NULL */
		len = 0;
		do {
			len++;
			get_user(temp, args->ARGS_PROC_LOAD.aEnvp);
		} while (temp);
		envp = MEM_Alloc(len * sizeof(BYTE *), MEM_NONPAGED);
		if (envp == NULL)
			status = DSP_EMEMORY;

		cp_fm_usr(envp, args->ARGS_PROC_LOAD.aEnvp, status, len);
		for (i = 0; DSP_SUCCEEDED(status) && (envp[i] != NULL); i++) {
			temp = envp[i];	/* User space pointer to argument */
			len = strlen_user(temp);
			/* Kernel space pointer to argument */
			envp[i] = MEM_Alloc(len, MEM_NONPAGED);
			if (envp[i] == NULL) {
				status = DSP_EMEMORY;
				break;
			}
			cp_fm_usr(envp[i], temp, status, len);
		}
	}
	GT_5trace(WCD_debugMask, GT_ENTER,
		 "PROCWRAP_Load, hProcessor: 0x%x\n\tiArgc:"
		 "0x%x\n\taArgv: 0x%x\n\taArgv[0]: %s\n\taEnvp: 0x%0x\n",
		 args->ARGS_PROC_LOAD.hProcessor,
		 args->ARGS_PROC_LOAD.iArgc, args->ARGS_PROC_LOAD.aArgv,
		 argv[0], args->ARGS_PROC_LOAD.aEnvp);
	if (DSP_SUCCEEDED(status)) {
		status = PROC_Load(args->ARGS_PROC_LOAD.hProcessor,
				  args->ARGS_PROC_LOAD.iArgc,
				  (CONST CHAR **)argv, (CONST CHAR **)envp);
	}
	if (envp != NULL) {
		i = 0;
		while (envp[i] != NULL)
			MEM_Free(envp[i++]);

		MEM_Free(envp);
	}
	if (argv != NULL) {
		for (i = 0; i < argc; i++) {
			if (argv[i] != NULL)
				MEM_Free(argv[i]);

		}
		MEM_Free(argv);
	}
	return (status);
}

/*
 * ======== PROCWRAP_Map ========
 */
DWORD PROCWRAP_Map(Trapped_Args *args)
{
	DSP_STATUS status;
	PVOID pMapAddr;

	GT_0trace(WCD_debugMask, GT_ENTER, "PROCWRAP_Map: entered\n");
	status = PROC_Map(args->ARGS_PROC_MAPMEM.hProcessor,
			 args->ARGS_PROC_MAPMEM.pMpuAddr,
			 args->ARGS_PROC_MAPMEM.ulSize,
			 args->ARGS_PROC_MAPMEM.pReqAddr, &pMapAddr,
			 args->ARGS_PROC_MAPMEM.ulMapAttr);
	if (DSP_SUCCEEDED(status)) {
		if (put_user(pMapAddr, args->ARGS_PROC_MAPMEM.ppMapAddr))
			status = DSP_EINVALIDARG;

	}
	return (status);
}

/*
 * ======== PROCWRAP_RegisterNotify ========
 */
DWORD PROCWRAP_RegisterNotify(Trapped_Args *args)
{
	DSP_STATUS status;
	struct DSP_NOTIFICATION notification;

	GT_0trace(WCD_debugMask, GT_ENTER,
		 "PROCWRAP_RegisterNotify: entered\n");

	/* Initialize the notification data structure  */
	notification.psName = NULL;
	notification.handle = NULL;

	status = PROC_RegisterNotify(args->ARGS_PROC_REGISTER_NOTIFY.hProcessor,
				    args->ARGS_PROC_REGISTER_NOTIFY.uEventMask,
				    args->ARGS_PROC_REGISTER_NOTIFY.uNotifyType,
				    &notification);
	cp_to_usr(args->ARGS_PROC_REGISTER_NOTIFY.hNotification, &notification,
		 status, 1);
	return (status);
}

/*
 * ======== PROCWRAP_ReserveMemory ========
 */
DWORD PROCWRAP_ReserveMemory(Trapped_Args *args)
{
	DSP_STATUS status;
	PVOID pRsvAddr;

	GT_0trace(WCD_debugMask, GT_ENTER, "PROCWRAP_ReserveMemory: entered\n");
	status = PROC_ReserveMemory(args->ARGS_PROC_RSVMEM.hProcessor,
				   args->ARGS_PROC_RSVMEM.ulSize, &pRsvAddr);
	if (put_user(pRsvAddr, args->ARGS_PROC_RSVMEM.ppRsvAddr))
		status = DSP_EINVALIDARG;

	return (status);
}

/*
 * ======== PROCWRAP_Start ========
 */
DWORD PROCWRAP_Start(Trapped_Args *args)
{
	GT_0trace(WCD_debugMask, GT_ENTER, "PROCWRAP_Start: entered\n");
	return (PROC_Start(args->ARGS_PROC_START.hProcessor));
}

/*
 * ======== PROCWRAP_UnMap ========
 */
DWORD PROCWRAP_UnMap(Trapped_Args *args)
{
	DSP_STATUS status;

	GT_0trace(WCD_debugMask, GT_ENTER, "PROCWRAP_UnMap: entered\n");
	status = PROC_UnMap(args->ARGS_PROC_UNMAPMEM.hProcessor,
			   args->ARGS_PROC_UNMAPMEM.pMapAddr);
	return (status);
}

/*
 * ======== PROCWRAP_UnReserveMemory ========
 */
DWORD PROCWRAP_UnReserveMemory(Trapped_Args *args)
{
	DSP_STATUS status;

	GT_0trace(WCD_debugMask, GT_ENTER,
		 "PROCWRAP_UnReserveMemory: entered\n");
	status = PROC_UnReserveMemory(args->ARGS_PROC_UNRSVMEM.hProcessor,
				     args->ARGS_PROC_UNRSVMEM.pRsvAddr);
	return (status);
}

/*
 * ======== PROCWRAP_Stop ========
 */
DWORD
PROCWRAP_Stop(Trapped_Args *args)
{
	GT_0trace(WCD_debugMask, GT_ENTER, "PROCWRAP_Stop: entered\n");
	return (PROC_Stop(args->ARGS_PROC_STOP.hProcessor));
}

/*
 * ======== NODEWRAP_Allocate ========
 */
DWORD NODEWRAP_Allocate(Trapped_Args *args)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_UUID nodeId;
	ULONG cbDataSize;
	ULONG *pSize = (ULONG *)args->ARGS_NODE_ALLOCATE.pArgs;
	BYTE *pArgs = NULL;
	struct DSP_NODEATTRIN attrIn, *pAttrIn = NULL;
	struct NODE_OBJECT *hNode;

	GT_0trace(WCD_debugMask, GT_ENTER, "NODEWRAP_Allocate: entered\n");

	/* Optional argument */
	if (pSize) {
		if (get_user(cbDataSize, pSize))
			status = DSP_EFAIL;

		cbDataSize += sizeof(ULONG);
		if (DSP_SUCCEEDED(status)) {
			pArgs = MEM_Alloc(cbDataSize, MEM_NONPAGED);
			if (pArgs == NULL)
				status = DSP_EMEMORY;

		}
		cp_fm_usr(pArgs, args->ARGS_NODE_ALLOCATE.pArgs, status,
			 cbDataSize);
	}
	cp_fm_usr(&nodeId, args->ARGS_NODE_ALLOCATE.pNodeID, status, 1);
	/* Optional argument */
	if (args->ARGS_NODE_ALLOCATE.pAttrIn) {
		cp_fm_usr(&attrIn, args->ARGS_NODE_ALLOCATE.pAttrIn, status, 1);
		if (DSP_SUCCEEDED(status))
			pAttrIn = &attrIn;

	}
	if (DSP_SUCCEEDED(status)) {
		status = NODE_Allocate(args->ARGS_NODE_ALLOCATE.hProcessor,
				      &nodeId, (struct DSP_CBDATA *)pArgs,
				      pAttrIn, &hNode);
	}
	cp_to_usr(args->ARGS_NODE_ALLOCATE.phNode, &hNode, status, 1);
	if (pArgs)
		MEM_Free(pArgs);

	return (status);
}

/*
 *  ======== NODEWRAP_AllocMsgBuf ========
 */
DWORD NODEWRAP_AllocMsgBuf(Trapped_Args *args)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_BUFFERATTR *pAttr = NULL;
	struct DSP_BUFFERATTR attr;
	BYTE *pBuffer = NULL;

	if (args->ARGS_NODE_ALLOCMSGBUF.pAttr) {	/* Optional argument */
		cp_fm_usr(&attr, args->ARGS_NODE_ALLOCMSGBUF.pAttr, status, 1);
		if (DSP_SUCCEEDED(status))
			pAttr = &attr;

	}
	/* IN OUT argument */
	cp_fm_usr(&pBuffer, args->ARGS_NODE_ALLOCMSGBUF.pBuffer, status, 1);
	if (DSP_SUCCEEDED(status)) {
		status = NODE_AllocMsgBuf(args->ARGS_NODE_ALLOCMSGBUF.hNode,
					 args->ARGS_NODE_ALLOCMSGBUF.uSize,
					 pAttr, &pBuffer);
	}
	cp_to_usr(args->ARGS_NODE_ALLOCMSGBUF.pBuffer, &pBuffer, status, 1)
    return (status);
}

/*
 * ======== NODEWRAP_ChangePriority ========
 */
DWORD NODEWRAP_ChangePriority(Trapped_Args *args)
{
	GT_0trace(WCD_debugMask, GT_ENTER,
		 "NODEWRAP_ChangePriority: entered\n");

	return (NODE_ChangePriority(args->ARGS_NODE_CHANGEPRIORITY.hNode,
		args->ARGS_NODE_CHANGEPRIORITY.iPriority));
}

/*
 * ======== NODEWRAP_Connect ========
 */
DWORD NODEWRAP_Connect(Trapped_Args *args)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_STRMATTR attrs;
	struct DSP_STRMATTR *pAttrs = NULL;
	ULONG cbDataSize;
	ULONG *pSize = (ULONG *)args->ARGS_NODE_CONNECT.pConnParam;
	BYTE *pArgs = NULL;

	GT_0trace(WCD_debugMask, GT_ENTER, "NODEWRAP_Connect: entered\n");

	/* Optional argument */
	if (pSize) {
		if (get_user(cbDataSize, pSize))
			status = DSP_EFAIL;

		cbDataSize += sizeof(ULONG);
		if (DSP_SUCCEEDED(status)) {
			pArgs = MEM_Alloc(cbDataSize, MEM_NONPAGED);
			if (pArgs == NULL)
				status = DSP_EMEMORY;

		}
		cp_fm_usr(pArgs, args->ARGS_NODE_CONNECT.pConnParam, status,
			 cbDataSize);
	}
	if (args->ARGS_NODE_CONNECT.pAttrs) {	/* Optional argument */
		cp_fm_usr(&attrs, args->ARGS_NODE_CONNECT.pAttrs, status, 1);
		if (DSP_SUCCEEDED(status))
			pAttrs = &attrs;

	}
	if (DSP_SUCCEEDED(status)) {
		status = NODE_Connect(args->ARGS_NODE_CONNECT.hNode,
				     args->ARGS_NODE_CONNECT.uStream,
				     args->ARGS_NODE_CONNECT.hOtherNode,
				     args->ARGS_NODE_CONNECT.uOtherStream,
				     pAttrs, (struct DSP_CBDATA *)pArgs);
	}
	if (pArgs)
		MEM_Free(pArgs);

	return (status);
}

/*
 * ======== NODEWRAP_Create ========
 */
DWORD NODEWRAP_Create(Trapped_Args *args)
{
	GT_0trace(WCD_debugMask, GT_ENTER, "NODEWRAP_Create: entered\n");
	return (NODE_Create(args->ARGS_NODE_CREATE.hNode));
}

/*
 * ======== NODEWRAP_Delete ========
 */
DWORD NODEWRAP_Delete(Trapped_Args *args)
{
	GT_0trace(WCD_debugMask, GT_ENTER, "NODEWRAP_Delete: entered\n");
	return (NODE_Delete(args->ARGS_NODE_DELETE.hNode));
}

/*
 *  ======== NODEWRAP_FreeMsgBuf ========
 */
DWORD NODEWRAP_FreeMsgBuf(Trapped_Args *args)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_BUFFERATTR *pAttr = NULL;
	struct DSP_BUFFERATTR attr;
	if (args->ARGS_NODE_FREEMSGBUF.pAttr) {	/* Optional argument */
		cp_fm_usr(&attr, args->ARGS_NODE_FREEMSGBUF.pAttr, status, 1);
		if (DSP_SUCCEEDED(status))
			pAttr = &attr;

	}
	if (DSP_SUCCEEDED(status)) {
		status = NODE_FreeMsgBuf(args->ARGS_NODE_FREEMSGBUF.hNode,
					args->ARGS_NODE_FREEMSGBUF.pBuffer,
					pAttr);
	}

	return (status);
}

/*
 * ======== NODEWRAP_GetAttr ========
 */
DWORD NODEWRAP_GetAttr(Trapped_Args *args)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_NODEATTR attr;

	GT_0trace(WCD_debugMask, GT_ENTER, "NODEWRAP_GetAttr: entered\n");

	status = NODE_GetAttr(args->ARGS_NODE_GETATTR.hNode, &attr,
			     args->ARGS_NODE_GETATTR.uAttrSize);
	cp_to_usr(args->ARGS_NODE_GETATTR.pAttr, &attr, status, 1);

	return (status);
}

/*
 * ======== NODEWRAP_GetMessage ========
 */
DWORD NODEWRAP_GetMessage(Trapped_Args *args)
{
	DSP_STATUS status;
	struct DSP_MSG msg;

	GT_0trace(WCD_debugMask, GT_ENTER, "NODEWRAP_GetMessage: entered\n");

	status = NODE_GetMessage(args->ARGS_NODE_GETMESSAGE.hNode, &msg,
				args->ARGS_NODE_GETMESSAGE.uTimeout);

	cp_to_usr(args->ARGS_NODE_GETMESSAGE.pMessage, &msg, status, 1);

	return (status);
}

/*
 * ======== NODEWRAP_Pause ========
 */
DWORD NODEWRAP_Pause(Trapped_Args *args)
{
	GT_0trace(WCD_debugMask, GT_ENTER, "NODEWRAP_Pause: entered\n");

	return (NODE_Pause(args->ARGS_NODE_PAUSE.hNode));
}

/*
 * ======== NODEWRAP_PutMessage ========
 */
DWORD NODEWRAP_PutMessage(Trapped_Args *args)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_MSG msg;

	GT_0trace(WCD_debugMask, GT_ENTER, "NODEWRAP_PutMessage: entered\n");

	cp_fm_usr(&msg, args->ARGS_NODE_PUTMESSAGE.pMessage, status, 1);

	if (DSP_SUCCEEDED(status)) {
		status = NODE_PutMessage(args->ARGS_NODE_PUTMESSAGE.hNode, &msg,
					args->ARGS_NODE_PUTMESSAGE.uTimeout);
	}

	return (status);
}

/*
 * ======== NODEWRAP_RegisterNotify ========
 */
DWORD NODEWRAP_RegisterNotify(Trapped_Args *args)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_NOTIFICATION notification;

	GT_0trace(WCD_debugMask, GT_ENTER,
		 "NODEWRAP_RegisterNotify: entered\n");

	/* Initialize the notification data structure  */
	notification.psName = NULL;
	notification.handle = NULL;

	status = NODE_RegisterNotify(args->ARGS_NODE_REGISTERNOTIFY.hNode,
				    args->ARGS_NODE_REGISTERNOTIFY.uEventMask,
				    args->ARGS_NODE_REGISTERNOTIFY.uNotifyType,
				    &notification);
	cp_to_usr(args->ARGS_NODE_REGISTERNOTIFY.hNotification, &notification,
		 status, 1);
	return (status);
}

/*
 * ======== NODEWRAP_Run ========
 */
DWORD NODEWRAP_Run(Trapped_Args *args)
{
	GT_0trace(WCD_debugMask, GT_ENTER, "NODEWRAP_Run: entered\n");

	return (NODE_Run(args->ARGS_NODE_RUN.hNode));
}

/*
 * ======== NODEWRAP_Terminate ========
 */
DWORD NODEWRAP_Terminate(Trapped_Args *args)
{
	DSP_STATUS status;
	DSP_STATUS tempstatus;

	GT_0trace(WCD_debugMask, GT_ENTER, "NODEWRAP_Terminate: entered\n");

	status = NODE_Terminate(args->ARGS_NODE_TERMINATE.hNode, &tempstatus);

	cp_to_usr(args->ARGS_NODE_TERMINATE.pStatus, &tempstatus, status, 1);

	return (status);
}


/*
 * ======== NODEWRAP_GetUUIDProps ========
 */
DWORD NODEWRAP_GetUUIDProps(Trapped_Args *args)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_UUID nodeId;
	struct DSP_NDBPROPS    *pnodeProps = NULL;
	UINT    copied;

	GT_0trace(WCD_debugMask, GT_ENTER,
		 "NODEWRAP_GetUUIDPropste: entered\n");


	cp_fm_usr(&nodeId, args->ARGS_NODE_GETUUIDPROPS.pNodeID, status, 1);
	pnodeProps = MEM_Alloc(sizeof(struct DSP_NDBPROPS), MEM_NONPAGED);
	if (pnodeProps != NULL) {
		status = NODE_GetUUIDProps(args->
					  ARGS_NODE_GETUUIDPROPS.hProcessor,
					  &nodeId, pnodeProps);
		cp_to_usr(args->ARGS_NODE_GETUUIDPROPS.pNodeProps, pnodeProps,
			 status, 1);
	}	else
		status = DSP_EMEMORY;
	if (pnodeProps)
		MEM_Free(pnodeProps);
	return (status);
}

/*
 * ======== STRMWRAP_AllocateBuffer ========
 */
DWORD STRMWRAP_AllocateBuffer(Trapped_Args *args)
{
	DSP_STATUS status;
	BYTE **apBuffer = NULL;
	ULONG uNumBufs = args->ARGS_STRM_ALLOCATEBUFFER.uNumBufs;

	DBC_Require(uNumBufs <= MAX_BUFS);

	apBuffer = MEM_Alloc((uNumBufs * sizeof(BYTE *)), MEM_NONPAGED);

	status = STRM_AllocateBuffer(args->ARGS_STRM_ALLOCATEBUFFER.hStream,
				     args->ARGS_STRM_ALLOCATEBUFFER.uSize,
				     apBuffer, uNumBufs);
	cp_to_usr(args->ARGS_STRM_ALLOCATEBUFFER.apBuffer, apBuffer, status,
		 uNumBufs);
	if (apBuffer)
		MEM_Free(apBuffer);

	return (status);
}

/*
 * ======== STRMWRAP_Close ========
 */
DWORD STRMWRAP_Close(Trapped_Args *args)
{
	return (STRM_Close(args->ARGS_STRM_CLOSE.hStream));
}

/*
 * ======== STRMWRAP_FreeBuffer ========
 */
DWORD STRMWRAP_FreeBuffer(Trapped_Args *args)
{
	DSP_STATUS status = DSP_SOK;
	BYTE **apBuffer = NULL;
	ULONG uNumBufs = args->ARGS_STRM_FREEBUFFER.uNumBufs;

	DBC_Require(uNumBufs <= MAX_BUFS);

	apBuffer = MEM_Alloc((uNumBufs * sizeof(BYTE *)), MEM_NONPAGED);

	cp_fm_usr(apBuffer, args->ARGS_STRM_FREEBUFFER.apBuffer, status,
		 uNumBufs);

	if (DSP_SUCCEEDED(status)) {
		status = STRM_FreeBuffer(args->ARGS_STRM_FREEBUFFER.hStream,
					 apBuffer, uNumBufs);
	}
	cp_to_usr(args->ARGS_STRM_FREEBUFFER.apBuffer, apBuffer, status,
		 uNumBufs);
	if (apBuffer)
		MEM_Free(apBuffer);

	return (status);
}

/*
 * ======== STRMWRAP_GetEventHandle ========
 */
DWORD STRMWRAP_GetEventHandle(Trapped_Args *args)
{
	return (DSP_ENOTIMPL);
}

/*
 * ======== STRMWRAP_GetInfo ========
 */
DWORD STRMWRAP_GetInfo(Trapped_Args *args)
{
	DSP_STATUS status = DSP_SOK;
	struct STRM_INFO strmInfo;
	struct DSP_STREAMINFO user;
	struct DSP_STREAMINFO *temp;

	cp_fm_usr(&strmInfo, args->ARGS_STRM_GETINFO.pStreamInfo, status, 1);
	temp = strmInfo.pUser;

	strmInfo.pUser = &user;

	if (DSP_SUCCEEDED(status)) {
		status = STRM_GetInfo(args->ARGS_STRM_GETINFO.hStream,
			 &strmInfo, args->ARGS_STRM_GETINFO.uStreamInfoSize);
	}
	cp_to_usr(temp, strmInfo.pUser, status, 1);
	strmInfo.pUser = temp;
	cp_to_usr(args->ARGS_STRM_GETINFO.pStreamInfo, &strmInfo, status, 1);
	return (status);
}

/*
 * ======== STRMWRAP_Idle ========
 */
DWORD STRMWRAP_Idle(Trapped_Args *args)
{
	return (STRM_Idle(args->ARGS_STRM_IDLE.hStream,
			  args->ARGS_STRM_IDLE.bFlush));
}

/*
 * ======== STRMWRAP_Issue ========
 */
DWORD STRMWRAP_Issue(Trapped_Args *args)
{
		/* This is a user space pointer */
	return (STRM_Issue(args->ARGS_STRM_ISSUE.hStream,
		args->ARGS_STRM_ISSUE.pBuffer, args->ARGS_STRM_ISSUE.dwBytes,
		args->ARGS_STRM_ISSUE.dwBufSize, args->ARGS_STRM_ISSUE.dwArg));
}

/*
 * ======== STRMWRAP_Open ========
 */
DWORD STRMWRAP_Open(Trapped_Args *args)
{
	DSP_STATUS status = DSP_SOK;
	struct STRM_ATTR attr;
	struct STRM_OBJECT *pStrm;
	struct DSP_STREAMATTRIN strmAttrIn;

	cp_fm_usr(&attr, args->ARGS_STRM_OPEN.pAttrIn, status, 1);

	if (attr.pStreamAttrIn != NULL) {	/* Optional argument */
		cp_fm_usr(&strmAttrIn, attr.pStreamAttrIn, status, 1);
		if (DSP_SUCCEEDED(status))
			attr.pStreamAttrIn = &strmAttrIn;

	}
	status = STRM_Open(args->ARGS_STRM_OPEN.hNode,
			  args->ARGS_STRM_OPEN.uDirection,
			  args->ARGS_STRM_OPEN.uIndex, &attr, &pStrm);
	cp_to_usr(args->ARGS_STRM_OPEN.phStream, &pStrm, status, 1);
	return (status);
}

/*
 * ======== STRMWRAP_Reclaim ========
 */
DWORD STRMWRAP_Reclaim(Trapped_Args *args)
{
	DSP_STATUS status = DSP_SOK;
	BYTE *pBufPtr;
	ULONG ulBytes;
	DWORD dwArg;
	ULONG ulBufSize;

	status = STRM_Reclaim(args->ARGS_STRM_RECLAIM.hStream, &pBufPtr,
			     &ulBytes, &ulBufSize, &dwArg);
	cp_to_usr(args->ARGS_STRM_RECLAIM.pBufPtr, &pBufPtr, status, 1);
	cp_to_usr(args->ARGS_STRM_RECLAIM.pBytes, &ulBytes, status, 1);
	cp_to_usr(args->ARGS_STRM_RECLAIM.pdwArg, &dwArg, status, 1);

	if (args->ARGS_STRM_RECLAIM.pBufSize != NULL) {
		cp_to_usr(args->ARGS_STRM_RECLAIM.pBufSize, &ulBufSize,
			 status, 1);
	}

	return (status);
}

/*
 * ======== STRMWRAP_RegisterNotify ========
 */
DWORD STRMWRAP_RegisterNotify(Trapped_Args *args)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_NOTIFICATION notification;

	GT_0trace(WCD_debugMask, GT_ENTER,
		 "NODEWRAP_RegisterNotify: entered\n");

	/* Initialize the notification data structure  */
	notification.psName = NULL;
	notification.handle = NULL;

	status = STRM_RegisterNotify(args->ARGS_STRM_REGISTERNOTIFY.hStream,
				    args->ARGS_STRM_REGISTERNOTIFY.uEventMask,
				    args->ARGS_STRM_REGISTERNOTIFY.uNotifyType,
				    &notification);
	cp_to_usr(args->ARGS_STRM_REGISTERNOTIFY.hNotification, &notification,
		 status, 1);

	return (status);
}

/*
 * ======== STRMWRAP_Select ========
 */
DWORD STRMWRAP_Select(Trapped_Args *args)
{
	UINT mask;
	struct STRM_OBJECT *aStrmTab[MAX_STREAMS];
	DSP_STATUS status = DSP_SOK;

	DBC_Require(args->ARGS_STRM_SELECT.nStreams <= MAX_STREAMS);

	cp_fm_usr(aStrmTab, args->ARGS_STRM_SELECT.aStreamTab, status,
		 args->ARGS_STRM_SELECT.nStreams);
	if (DSP_SUCCEEDED(status)) {
		status = STRM_Select(aStrmTab, args->ARGS_STRM_SELECT.nStreams,
				    &mask, args->ARGS_STRM_SELECT.uTimeout);
	}
	cp_to_usr(args->ARGS_STRM_SELECT.pMask, &mask, status, 1);
	return (status);
}

/* CMM */

/*
 * ======== CMMWRAP_CallocBuf ========
 */
DWORD CMMWRAP_CallocBuf(Trapped_Args *args)
{
	/* This operation is done in kernel */
	return (DSP_ENOTIMPL);
}

/*
 * ======== CMMWRAP_FreeBuf ========
 */
DWORD CMMWRAP_FreeBuf(Trapped_Args *args)
{
	/* This operation is done in kernel */
	return (DSP_ENOTIMPL);
}

/*
 * ======== CMMWRAP_GetHandle ========
 */
DWORD CMMWRAP_GetHandle(Trapped_Args *args)
{
	DSP_STATUS status = DSP_SOK;
	struct CMM_OBJECT *hCmmMgr;

	status = CMM_GetHandle(args->ARGS_CMM_GETHANDLE.hProcessor, &hCmmMgr);

	cp_to_usr(args->ARGS_CMM_GETHANDLE.phCmmMgr, &hCmmMgr, status, 1);

	return (status);
}

/*
 * ======== CMMWRAP_GetInfo ========
 */
DWORD CMMWRAP_GetInfo(Trapped_Args *args)
{
	DSP_STATUS status = DSP_SOK;
	struct CMM_INFO cmmInfo;

	status = CMM_GetInfo(args->ARGS_CMM_GETINFO.hCmmMgr, &cmmInfo);

	cp_to_usr(args->ARGS_CMM_GETINFO.pCmmInfo, &cmmInfo, status, 1);

	return (status);
}

/*
 *  ======== MEMWRAP_Alloc ========
 */
DWORD MEMWRAP_Alloc(Trapped_Args *args)
{
	return (DSP_ENOTIMPL);

}

/*
 *  ======== MEMWRAP_Calloc ========
 */
DWORD MEMWRAP_Calloc(Trapped_Args *args)
{
	return (DSP_ENOTIMPL);
}

/*
 *  ======== MEMWRAP_Free ========
 */
DWORD MEMWRAP_Free(Trapped_Args *args)
{
	return (DSP_ENOTIMPL);
}

/*
 *  ======== MEMWRAP_PageLock ========
 */
DWORD MEMWRAP_PageLock(Trapped_Args *args)
{
	return (DSP_ENOTIMPL);
}

/*
 *  ======== MEMWRAP_PageUnlock ========
 */
DWORD MEMWRAP_PageUnlock(Trapped_Args *args)
{
	return (DSP_ENOTIMPL);
}

/*
 * ======== UTILWRAP_TestDll ========
 */
DWORD UTILWRAP_TestDll(Trapped_Args *args)
{
	return (DSP_ENOTIMPL);
}

