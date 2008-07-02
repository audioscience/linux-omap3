/*
 * dspbridge/mpu_driver/inc/resourcecleanup.h
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



#ifndef RES_CLEANUP_DISABLE

#include <nodepriv.h>
#include <drv.h>


extern DSP_STATUS DRV_DisplayProcContext(HANDLE hDrvObject);

extern DSP_STATUS DRV_GetProcCtxtList(HANDLE pPctxt, HANDLE hDrvObject);

extern DSP_STATUS DRV_InsertProcContext(HANDLE hDrvObject, HANDLE pPctxt);

extern DSP_STATUS DRV_RemoveAllDMMResElements(HANDLE pCtxt);

extern DSP_STATUS DRV_RemoveAllNodeResElements(HANDLE pCtxt);

extern DSP_STATUS DRV_ProcUpdatestate(HANDLE pCtxt,
				      GPP_PROC_RES_STATE resState);

extern DSP_STATUS DRV_ProcSetPID(HANDLE pCtxt, INT hProcess);

extern DSP_STATUS DRV_GetProcContext(HANDLE phProcess, HANDLE hDrvObject,
				     HANDLE pCtxt, DSP_HNODE hNode,
				     ULONG pMapAddr);

extern DSP_STATUS DRV_RemoveAllResources(HANDLE pPctxt);

extern DSP_STATUS DRV_RemoveProcContext(HANDLE hDRVObject, HANDLE hPCtxt,
					HANDLE hProcess);

extern DSP_STATUS DRV_GetNodeResElement(HANDLE hNode, HANDLE nodeRes,
					HANDLE pCtxt);

extern DSP_STATUS DRV_InsertNodeResElement(HANDLE hNode, HANDLE nodeRes,
					    HANDLE pCtxt);

extern DSP_STATUS DRV_ProcNodeUpdateHeapStatus(HANDLE nodeRes, BOOL status);

extern DSP_STATUS DRV_ProcNodeUpdateStreamStatus(HANDLE nodeRes, BOOL status);

extern DSP_STATUS DRV_RemoveNodeResElement(HANDLE nodeRes, BOOL status);

extern DSP_STATUS DRV_ProcNodeUpdateStatus(HANDLE nodeRes, BOOL status);

extern DSP_STATUS DRV_UpdateDMMResElement(HANDLE dmmRes, ULONG pMpuAddr,
					  UINT ulSize, ULONG pReqAddr,
					  ULONG ppMapAddr, HANDLE hProcesso);

extern DSP_STATUS DRV_InsertDMMResElement(HANDLE dmmRes, HANDLE pCtxt);

extern DSP_STATUS DRV_GetDMMResElement(UINT pMapAddr, HANDLE dmmRes,
				       HANDLE pCtxt);

extern DSP_STATUS DRV_RemoveDMMResElement(HANDLE dmmRes, HANDLE pCtxt);

extern DSP_STATUS DRV_ProcUpdateSTRMRes(UINT uNumBufs, HANDLE STRMRes,
					HANDLE pCtxt);

extern DSP_STATUS DRV_ProcInsertSTRMResElement(HANDLE hStrm, HANDLE STRMRes,
						HANDLE pPctxt);

extern DSP_STATUS DRV_GetSTRMResElement(HANDLE hStrm, HANDLE STRMRes,
					HANDLE pCtxt);

extern DSP_STATUS DRV_ProcRemoveSTRMResElement(HANDLE STRMRes, HANDLE pCtxt);

extern DSP_STATUS DRV_RemoveAllSTRMResElements(HANDLE pCtxt);

extern DSP_STATUS DRV_GetDSPHEAPResElement(ULONG pMapAddr, HANDLE hDSPHEAPRes,
					   HANDLE hPCtxt);

extern DSP_STATUS DRV_RemoveAllDSPHEAPResElements(HANDLE hPCtxt);

extern DSP_STATUS DRV_ProcFreeDSPHEAPRes(HANDLE hPCtxt);

extern DSP_STATUS DRV_UpdateDSPHEAPResElement(HANDLE hDSPHEAPRes,
					      ULONG pMpuAddr, ULONG ulSize,
					      ULONG pReqAddr, ULONG pMapAddr,
					      HANDLE hProcessor);

extern DSP_STATUS DRV_RemoveDSPHEAPResElement(HANDLE hDSPHEAPRes,
					      HANDLE hPCtxt);

extern DSP_STATUS DRV_InsertDSPHEAPResElement(HANDLE hDSPHEAPRes,
					      HANDLE hPCtxt);

extern DSP_STATUS DRV_ProcDisplayResInfo(BYTE *pBuf, UINT *pSize);

extern NODE_STATE NODE_GetState(HANDLE hNode);

#endif
