/*
 * dspbridge/mpu_driver/inc/wmdchnl.h
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
 *  ======== wmdchnl.h ========
 *  Description:
 *      Declares the upper edge channel class library functions required by
 *      all WMD / WCD driver interface tables.  These functions are implemented
 *      by every class of WMD channel library.
 *
 *  Public Functions:
 *
 *  Notes:
 *      The function comment headers reside with the function typedefs in wmd.h.
 *
 *! Revision History:
 *! ================
 *! 07-Jan-2002 ag  Added cBufSize to WMD_CHNL_AddIOReq().
 *! 13-Oct-2000 jeh Added dwArg parameter to WMD_CHNL_AddIOReq(), added
 *!                 WMD_CHNL_Idle and WMD_CHNL_RegisterNotify for DSPStream
 *!                 support.
 *! 11-Jul-1996 gp: Created.
 */

#ifndef WMDCHNL_
#define WMDCHNL_

	extern DSP_STATUS WMD_CHNL_Create(OUT struct CHNL_MGR **phChnlMgr,
					  struct DEV_OBJECT *hDevObject,
					  IN CONST struct CHNL_MGRATTRS
					  *pMgrAttrs);

	extern DSP_STATUS WMD_CHNL_Destroy(struct CHNL_MGR *hChnlMgr);

	extern DSP_STATUS WMD_CHNL_Open(OUT struct CHNL_OBJECT **phChnl,
					struct CHNL_MGR *hChnlMgr,
					CHNL_MODE uMode,
					ULONG uChnlId,
					CONST IN OPTIONAL struct CHNL_ATTRS
					*pAttrs);

	extern DSP_STATUS WMD_CHNL_Close(struct CHNL_OBJECT *hChnl);

	extern DSP_STATUS WMD_CHNL_AddIOReq(struct CHNL_OBJECT *hChnl,
					    PVOID pHostBuf,
					    ULONG cBytes, ULONG cBufSize,
					    OPTIONAL DWORD dwDspAddr,
					    DWORD dwArg);

	extern DSP_STATUS WMD_CHNL_GetIOC(struct CHNL_OBJECT *hChnl,
					  DWORD dwTimeOut,
					  OUT struct CHNL_IOC *pIOC);

	extern DSP_STATUS WMD_CHNL_CancelIO(struct CHNL_OBJECT *hChnl);

	extern DSP_STATUS WMD_CHNL_FlushIO(struct CHNL_OBJECT *hChnl,
					   DWORD dwTimeOut);

	extern DSP_STATUS WMD_CHNL_GetInfo(struct CHNL_OBJECT *hChnl,
					   OUT struct CHNL_INFO *pInfo);

	extern DSP_STATUS WMD_CHNL_GetMgrInfo(struct CHNL_MGR *hChnlMgr,
					      ULONG uChnlID,
					      OUT struct CHNL_MGRINFO
					      *pMgrInfo);

	extern DSP_STATUS WMD_CHNL_Idle(struct CHNL_OBJECT *hChnl,
					DWORD dwTimeOut, BOOL fFlush);

	extern DSP_STATUS WMD_CHNL_RegisterNotify(struct CHNL_OBJECT *hChnl,
						  UINT uEventMask,
						  UINT uNotifyType,
						  struct DSP_NOTIFICATION
						  *hNotification);

#endif				/* WMDCHNL_ */
