/*
 * dspbridge/mpu_driver/inc/_dcd.h
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
 *  ======== _dcd.h ========
 *  Description:
 *      Includes the wrapper functions called directly by the
 *      DeviceIOControl interface.
 *
 *  Public Functions:
 *      WCD_CallDevIOCtl
 *      WCD_Init
 *      WCD_InitComplete2
 *      WCD_Exit
 *      <MOD>WRAP_*
 *
 *  Notes:
 *      Compiled with CDECL calling convention.
 *
 *! Revision History:
 *! ================
 *! 19-Apr-2004 sb  Aligned DMM definitions with Symbian
 *! 08-Mar-2004 sb  Added the Dynamic Memory Mapping feature
 *! 30-Jan-2002 ag  Renamed CMMWRAP_AllocBuf to CMMWRAP_CallocBuf.
 *! 22-Nov-2000 kc: Added MGRWRAP_GetPerf_Data to acquire PERF stats.
 *! 27-Oct-2000 jeh Added NODEWRAP_AllocMsgBuf, NODEWRAP_FreeMsgBuf. Removed
 *!                 NODEWRAP_GetMessageStream.
 *! 10-Oct-2000 ag: Added user CMM wrappers.
 *! 04-Aug-2000 rr: MEMWRAP and UTIL_Wrap added.
 *! 27-Jul-2000 rr: NODEWRAP, STRMWRAP added.
 *! 27-Jun-2000 rr: MGRWRAP fxns added.IFDEF to build for PM or DSP/BIOS Bridge
 *! 03-Dec-1999 rr: WCD_InitComplete2 enabled for BRD_AutoStart.
 *! 09-Nov-1999 kc: Added MEMRY.
 *! 02-Nov-1999 ag: Added CHNL.
 *! 08-Oct-1999 rr: Utilwrap_Testdll fxn added
 *! 24-Sep-1999 rr: header changed from _wcd.h to _dcd.h
 *! 09-Sep-1997 gp: Created.
 */

#ifndef _WCD_
#define _WCD_

#include <wcdioctl.h>

/*
 *  ======== WCD_CallDevIOCtl ========
 *  Purpose:
 *      Call the (wrapper) function for the corresponding WCD IOCTL.
 *  Parameters:
 *      cmd:        IOCTL id, base 0.
 *      args:       Argument structure.
 *      pResult:
 *  Returns:
 *      DSP_SOK if command called; DSP_EINVALIDARG if command not in IOCTL
 *      table.
 *  Requires:
 *  Ensures:
 */

/* extern __inline DSP_STATUS WCD_CallDevIOCtl(unsigned int cmd, */
	extern DSP_STATUS WCD_CallDevIOCtl(unsigned int cmd,
					   Trapped_Args * args,
					   DWORD *pResult);

/*
 *  ======== WCD_Init ========
 *  Purpose:
 *      Initialize WCD modules, and export WCD services to WMD's.
 *      This procedure is called when the class driver is loaded.
 *  Parameters:
 *  Returns:
 *      TRUE if success; FALSE otherwise.
 *  Requires:
 *  Ensures:
 */
	extern BOOL WCD_Init();

/*
 *  ======== WCD_InitComplete2 ========
 *  Purpose:
 *      Perform any required WCD, and WMD initialization which
 *      cannot not be performed in WCD_Init() or DEV_StartDevice() due
 *      to the fact that some services are not yet
 *      completely initialized.
 *  Parameters:
 *  Returns:
 *      DSP_SOK:        Allow this device to load
 *      DSP_EFAIL:      Failure.
 *  Requires:
 *      WCD initialized.
 *  Ensures:
 */
	extern DSP_STATUS WCD_InitComplete2();

/*
 *  ======== WCD_Exit ========
 *  Purpose:
 *      Exit all modules initialized in WCD_Init().
 *      This procedure is called when the class driver is unloaded.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      WCD_Init() was previously called.
 *  Ensures:
 *      Resources acquired in WCD_Init() are freed.
 */
	extern VOID WCD_Exit();

/* MGR wrapper functions */
	extern DWORD MGRWRAP_EnumNode_Info(Trapped_Args * args);
	extern DWORD MGRWRAP_EnumProc_Info(Trapped_Args * args);
	extern DWORD MGRWRAP_RegisterObject(Trapped_Args * args);
	extern DWORD MGRWRAP_UnregisterObject(Trapped_Args * args);
	extern DWORD MGRWRAP_WaitForBridgeEvents(Trapped_Args * args);

#ifndef RES_CLEANUP_DISABLE
	extern DWORD MGRWRAP_GetProcessResourcesInfo(Trapped_Args * args);
#endif


/* CPRC (Processor) wrapper Functions */
	extern DWORD PROCWRAP_Attach(Trapped_Args * args);
	extern DWORD PROCWRAP_Ctrl(Trapped_Args * args);
	extern DWORD PROCWRAP_Detach(Trapped_Args * args);
	extern DWORD PROCWRAP_EnumNode_Info(Trapped_Args * args);
	extern DWORD PROCWRAP_EnumResources(Trapped_Args * args);
	extern DWORD PROCWRAP_GetState(Trapped_Args * args);
	extern DWORD PROCWRAP_GetTrace(Trapped_Args * args);
	extern DWORD PROCWRAP_Load(Trapped_Args * args);
	extern DWORD PROCWRAP_RegisterNotify(Trapped_Args * args);
	extern DWORD PROCWRAP_Start(Trapped_Args * args);
	extern DWORD PROCWRAP_ReserveMemory(Trapped_Args * args);
	extern DWORD PROCWRAP_UnReserveMemory(Trapped_Args * args);
	extern DWORD PROCWRAP_Map(Trapped_Args * args);
	extern DWORD PROCWRAP_UnMap(Trapped_Args * args);
	extern DWORD PROCWRAP_FlushMemory(Trapped_Args * args);
	extern DWORD PROCWRAP_Stop(Trapped_Args * args);
	extern DWORD PROCWRAP_InvalidateMemory(Trapped_Args * args);

/* NODE wrapper functions */
	extern DWORD NODEWRAP_Allocate(Trapped_Args * args);
	extern DWORD NODEWRAP_AllocMsgBuf(Trapped_Args * args);
	extern DWORD NODEWRAP_ChangePriority(Trapped_Args * args);
	extern DWORD NODEWRAP_Connect(Trapped_Args * args);
	extern DWORD NODEWRAP_Create(Trapped_Args * args);
	extern DWORD NODEWRAP_Delete(Trapped_Args * args);
	extern DWORD NODEWRAP_FreeMsgBuf(Trapped_Args * args);
	extern DWORD NODEWRAP_GetAttr(Trapped_Args * args);
	extern DWORD NODEWRAP_GetMessage(Trapped_Args * args);
	extern DWORD NODEWRAP_Pause(Trapped_Args * args);
	extern DWORD NODEWRAP_PutMessage(Trapped_Args * args);
	extern DWORD NODEWRAP_RegisterNotify(Trapped_Args * args);
	extern DWORD NODEWRAP_Run(Trapped_Args * args);
	extern DWORD NODEWRAP_Terminate(Trapped_Args * args);
	extern DWORD NODEWRAP_GetUUIDProps(Trapped_Args * args);

/* STRM wrapper functions */
	extern DWORD STRMWRAP_AllocateBuffer(Trapped_Args * args);
	extern DWORD STRMWRAP_Close(Trapped_Args * args);
	extern DWORD STRMWRAP_FreeBuffer(Trapped_Args * args);
	extern DWORD STRMWRAP_GetEventHandle(Trapped_Args * args);
	extern DWORD STRMWRAP_GetInfo(Trapped_Args * args);
	extern DWORD STRMWRAP_Idle(Trapped_Args * args);
	extern DWORD STRMWRAP_Issue(Trapped_Args * args);
	extern DWORD STRMWRAP_Open(Trapped_Args * args);
	extern DWORD STRMWRAP_Reclaim(Trapped_Args * args);
	extern DWORD STRMWRAP_RegisterNotify(Trapped_Args * args);
	extern DWORD STRMWRAP_Select(Trapped_Args * args);

	extern DWORD CMMWRAP_CallocBuf(Trapped_Args * args);
	extern DWORD CMMWRAP_FreeBuf(Trapped_Args * args);
	extern DWORD CMMWRAP_GetHandle(Trapped_Args * args);
	extern DWORD CMMWRAP_GetInfo(Trapped_Args * args);

	extern DWORD MEMWRAP_Alloc(Trapped_Args * args);
	extern DWORD MEMWRAP_Calloc(Trapped_Args * args);
	extern DWORD MEMWRAP_Free(Trapped_Args * args);
	extern DWORD MEMWRAP_PageLock(Trapped_Args * args);
	extern DWORD MEMWRAP_PageUnlock(Trapped_Args * args);

	extern DWORD UTILWRAP_TestDll(Trapped_Args * args);

#endif				/* _WCD_ */
