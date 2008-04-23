/*
 * dspbridge/inc/dmm.h
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
 *  ======== dmm.h ========
 *  Purpose:
 *      The Dynamic Memory Mapping(DMM) module manages the DSP Virtual address
 *      space that can be directly mapped to any MPU buffer or memory region
 *
 *  Public Functions:
 *
 *! Revision History:
 *! ================
 *! 20-Feb-2004 sb: Created.
 *!
 */

#ifndef DMM_
#define DMM_

#ifdef __cplusplus
extern "C" {
#endif

#include <dbdefs.h>

	struct DMM_OBJECT;
	/*typedef struct DMM_OBJECT *DMM_HMGR;*/

/* DMM attributes used in DMM_Create() */
	struct DMM_MGRATTRS {
		ULONG reserved;
	} ;

#if defined(OMAP_2430) || defined(OMAP_3430)
#define DMMPOOLSIZE      0x3000000
#endif

/*
 *  ======== DMM_GetHandle ========
 *  Purpose:
 *      Return the dynamic memory manager object for this device.
 *      This is typically called from the client process.
 */

	extern DSP_STATUS DMM_GetHandle(DSP_HPROCESSOR hProcessor,
					OUT struct DMM_OBJECT **phDmmMgr);

	extern DSP_STATUS DMM_ReserveMemory(struct DMM_OBJECT *hDmmMgr,
					    ULONG size,
					    ULONG *pRsvAddr);

	extern DSP_STATUS DMM_UnReserveMemory(struct DMM_OBJECT *hDmmMgr,
					      ULONG rsvAddr);

	extern DSP_STATUS DMM_MapMemory(struct DMM_OBJECT *hDmmMgr, ULONG addr,
					ULONG size);

	extern DSP_STATUS DMM_UnMapMemory(struct DMM_OBJECT *hDmmMgr,
					  ULONG addr,
					  ULONG *pSize);

	extern DSP_STATUS DMM_Destroy(struct DMM_OBJECT *hDmmMgr, BOOL bForce);

	extern DSP_STATUS DMM_Create(OUT struct DMM_OBJECT **phDmmMgr,
				     struct DEV_OBJECT *hDevObject,
				     IN CONST struct DMM_MGRATTRS *pMgrAttrs);

	extern BOOL DMM_Init();

	extern VOID DMM_Exit();

	extern DSP_STATUS DMM_ChunkAdd(struct DMM_OBJECT *hDmmMgr, ULONG addr,
				       ULONG size);

	extern DSP_STATUS DMM_Reset(struct DMM_OBJECT *hDmmMgr);

	extern DSP_STATUS DMM_BlockList(struct DMM_OBJECT *hDmmMgr,
					ULONG rsvAddr,
					IN OUT ULONG *count, ULONG *mapList);

#ifdef __cplusplus
}
#endif
#endif				/* DMM_ */
