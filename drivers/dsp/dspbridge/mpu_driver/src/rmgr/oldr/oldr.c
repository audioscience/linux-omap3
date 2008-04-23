/*
 * dspbridge/src/rmgr/linux/oldr/oldr.c
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
 *  ======== oldr.c ========
 *
 *  Description:
 *      DSP/BIOS Bridge static + overlay loader.
 *
 *  Public Functions:
 *      OLDR_Allocate
 *      OLDR_Create
 *      OLDR_Delete
 *      OLDR_Exit
 *      OLDR_Free
 *      OLDR_GetFxnAddr
 *      OLDR_Init
 *      OLDR_Load
 *      OLDR_Unload
 *
 *! Revision History
 *! ================
 *! 18-Feb-2003 vp	Code review updates.
 *! 18-Oct-2002 vp	Ported to Linux Platform.
 *! 18-Apr-2002 jeh     Call DBL functions through function table (for testing).
 *! 19-Oct-2001 jeh     Call DBL directly, instead of going through COD.
 *! 27-Aug-2001 jeh     Created.
 */

/*  ----------------------------------- Host OS */
#include <host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <std.h>
#include <dbdefs.h>
#include <errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <dbg_zones.h>
#include <dbc.h>
#include <gt.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <csl.h>
#include <cod.h>
#include <dev.h>
#include <mem.h>

/*  ----------------------------------- Resource Manager */
#include <rmm.h>
#include <uuidutil.h>
#include <dbldefs.h>
#include <dbl.h>

/*  ----------------------------------- This */
#include <oldr.h>

#define MAXNAMELEN          MAXUUIDLEN + 4	/* UUID + :<task phase> */
#define OLDR_SIGNATURE      0x52444c4F	/* "RDLO" */
#define OLDR_NODESIGNATURE  0x4e444c4F	/* "NDLO" */

/* Dummy OLDR_HNODE for non-overlay nodes. */
#define DUMMYNODE       0xffffffff

/*
 *  ======== OLDR_OBJECT ========
 *  Overlay loader object.
 */
struct OLDR_OBJECT {
	DWORD dwSignature;	/* For object validation */
	struct DEV_OBJECT *hDevObject;	/* Device object */
	DBL_Target dbl;		/* DBL loader */
	struct DBL_LibraryObj *lib;	/* Base image library */
	struct RMM_TargetObj *rmm;	/* Remote memory manager for DSP */
	DBL_WriteFxn pfnWrite;	/* Overlay function */
	DBL_AllocFxn pfnAlloc;	/* Remote allocation function */
	DBL_FreeFxn pfnFree;	/* Remote free function */
	DBL_Attrs dblAttrs;
	struct DBL_Fxns dblFxns;
};

/*
 *  ======== OLDR_NODEOBJECT ========
 *  Overlay node object.
 */
struct OLDR_NODEOBJECT {
	DWORD dwSignature;	/* For object validation */
	struct OLDR_OBJECT *pOldr;	/* Static loader object */
	PVOID pPrivRef;		/* Handle for DBL_WriteFxn */
	CHAR szUUID[MAXNAMELEN];	/* UUID */
	BOOL fOvly;		/* Overlay node? */
};

static struct DBL_Fxns dblFxns = {
	(DBL_CloseFxn) DBL_close,
	(DBL_CreateFxn) DBL_create,
	(DBL_DeleteFxn) DBL_delete,
	(DBL_ExitFxn) DBL_exit,
	(DBL_GetAttrsFxn) DBL_getAttrs,
	(DBL_GetAddrFxn) DBL_getAddr,
	(DBL_GetCAddrFxn) DBL_getCAddr,
	(DBL_GetSectFxn) DBL_getSect,
	(DBL_InitFxn) DBL_init,
	(DBL_LoadFxn) DBL_load,
	(DBL_LoadSectFxn) DBL_loadSect,
	(DBL_OpenFxn) DBL_open,
	(DBL_ReadSectFxn) DBL_readSect,
	(DBL_SetAttrsFxn) DBL_setAttrs,
	(DBL_UnloadFxn) DBL_unload,
	(DBL_UnloadSectFxn) DBL_unloadSect,
};

#if GT_TRACE
static struct GT_Mask OLDR_debugMask = { 0, 0 };	/* GT trace variable */
#endif
static ULONG cRefs = 0;		/* module reference count */

/*
 *  ======== OLDR_Allocate ========
 *  Purpose:
 *  	Allocates memory for overlay loader object.
 */
DSP_STATUS OLDR_Allocate(struct DLDR_OBJECT *hDldr, PVOID pPrivRef,
				IN CONST struct DCD_NODEPROPS *pNodeProps,
				OUT struct DLDR_NODEOBJECT **phDldrNode)
{
	struct OLDR_OBJECT *hOldr = (struct OLDR_OBJECT *)hDldr;
	struct OLDR_NODEOBJECT *pOldrNode = NULL;
	ULONG ulAddr;
	ULONG ulSize;
	struct DSP_UUID uuid;
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	DBC_Require(pNodeProps != NULL);
	DBC_Require(phDldrNode != NULL);
	DBC_Require(MEM_IsValidHandle(hOldr, OLDR_SIGNATURE));

	GT_4trace(OLDR_debugMask, GT_ENTER,
		 "OLDR_Allocate(0x%x, 0x%x, 0x%x, 0x%x)\n",
		 hDldr, pPrivRef, pNodeProps, phDldrNode);

	/* Initialize handle in case of failure */
	*phDldrNode = NULL;

	/* Allocate node object */
	MEM_AllocObject(pOldrNode, struct OLDR_NODEOBJECT, OLDR_NODESIGNATURE);

	if (pOldrNode == NULL) {
		GT_0trace(OLDR_debugMask, GT_6CLASS,
			 "OLDR_Allocate: Memory allocation"
			 " failed\n");
		status = DSP_EMEMORY;
	} else {
		pOldrNode->pOldr = hOldr;
		pOldrNode->pPrivRef = pPrivRef;

		/* Save node's UUID. */
		uuid = pNodeProps->ndbProps.uiNodeID;
		UUID_UuidToString(&uuid, pOldrNode->szUUID, MAXNAMELEN);

		/*
		 *  Determine if node is an overlay node. DBL_getSect will
		 *  return an error if node is not an overlay node.
		 */
		if (DSP_SUCCEEDED(hOldr->dblFxns.getSectFxn(hOldr->lib,
				 pOldrNode->szUUID, &ulAddr, &ulSize))) {
			/* Overlay node */
			pOldrNode->fOvly = TRUE;
		}

		*phDldrNode = (struct DLDR_NODEOBJECT *)pOldrNode;
	}

	/* Cleanup on failure */
	if (DSP_FAILED(status) && pOldrNode) {
		OLDR_Free((struct DLDR_NODEOBJECT *) pOldrNode);
	}

	DBC_Ensure(DSP_SUCCEEDED(status) &&
			(MEM_IsValidHandle(((struct OLDR_NODEOBJECT *)
			(*phDldrNode)), OLDR_NODESIGNATURE) ||
			(DSP_FAILED(status) && *phDldrNode == NULL)));

	return (status);
}

/*
 *  ======== OLDR_Create ========
 *  Purpose:
 *  	Creates overlay memory manager.
 */
DSP_STATUS OLDR_Create(OUT struct DLDR_OBJECT **phDldr,
		      struct DEV_OBJECT *hDevObject,
		      IN CONST struct DLDR_ATTRS *pAttrs)
{
	struct COD_MANAGER *hCodMgr;	/* COD manager */
	struct OLDR_OBJECT *pOldr = NULL;
	DBL_Attrs saveAttrs;
	DBL_Attrs newAttrs;
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	DBC_Require(phDldr != NULL);
	DBC_Require(hDevObject != NULL);
	DBC_Require(pAttrs != NULL);

	GT_3trace(OLDR_debugMask, GT_ENTER,
		 "OLDR_Create(0x%x, 0x%x, 0x%x)\n", phDldr,
		 hDevObject, pAttrs);

	/* Allocate loader object */
	MEM_AllocObject(pOldr, struct OLDR_OBJECT, OLDR_SIGNATURE);

	if (pOldr) {
		pOldr->hDevObject = hDevObject;
		(VOID) DEV_GetCodMgr(hDevObject, &hCodMgr);

		/*
		 *  Create Overlay memory manager
		 *         Set pOldr alloc, free, and write functions.
		 */
		status = RMM_create(&pOldr->rmm, NULL, 0);
		if (DSP_SUCCEEDED(status)) {
			pOldr->pfnWrite = (DBL_WriteFxn) pAttrs->pfnOvly;
			pOldr->pfnAlloc = (DBL_AllocFxn) RMM_alloc;
			pOldr->pfnFree = (DBL_FreeFxn) RMM_free;

			/* Set loader functions */
			pOldr->dblFxns = dblFxns;

			pOldr->dblFxns.initFxn();
			(VOID)COD_GetLoader(hCodMgr, &pOldr->dbl);
			(VOID)COD_GetBaseLib(hCodMgr, &pOldr->lib);

			/* set the alloc, and free functions for DBL */
			pOldr->dblFxns.getAttrsFxn(pOldr->dbl, &saveAttrs);

			newAttrs = saveAttrs;
			newAttrs.rmmHandle = pOldr->rmm;
			newAttrs.alloc = pOldr->pfnAlloc;
			newAttrs.free = pOldr->pfnFree;
			pOldr->dblFxns.setAttrsFxn(pOldr->dbl, &newAttrs);
			pOldr->dblAttrs = newAttrs;
			pOldr->dblAttrs.write = (DBL_WriteFxn) pAttrs->pfnOvly;
		}
	} else {
		GT_0trace(OLDR_debugMask, GT_6CLASS,
			  "OLDR_Create: Memory allocation "
			  "failed\n");
		status = DSP_EMEMORY;
	}

	if (DSP_SUCCEEDED(status)) {
		*phDldr = (struct DLDR_OBJECT *) pOldr;
	} else {
		if (pOldr) {
			OLDR_Delete((struct DLDR_OBJECT *) pOldr);
		}
		*phDldr = NULL;
	}

	DBC_Ensure(((DSP_SUCCEEDED(status)) &&
		 (MEM_IsValidHandle(((struct OLDR_OBJECT *)*phDldr),
		 OLDR_SIGNATURE)))  || ((DSP_FAILED(status)) &&
		 (*phDldr == NULL)));

	return (status);
}

/*
 *  ======== OLDR_Delete ========
 *  Purpose:
 *  	Frees resources allocated for the loader.
 */
VOID OLDR_Delete(struct DLDR_OBJECT *hDldr)
{
	struct OLDR_OBJECT *hOldr = (struct OLDR_OBJECT *) hDldr;

	DBC_Require(cRefs > 0);
	DBC_Require(MEM_IsValidHandle(hOldr, OLDR_SIGNATURE));

	GT_1trace(OLDR_debugMask, GT_ENTER, "OLDR_Delete(0x%x)\n", hDldr);

	if (hOldr->rmm) {
		RMM_delete(hOldr->rmm);
	}

	hOldr->dblFxns.exitFxn();
	MEM_FreeObject(hOldr);
}

/*
 *  ======== OLDR_Exit ========
 *  Discontinue usage of OLDR module.
 */
VOID OLDR_Exit()
{
	DBC_Require(cRefs > 0);

	cRefs--;

	GT_1trace(OLDR_debugMask, GT_5CLASS,
		 "Entered OLDR_Exit, ref count: 0x%x\n", cRefs);

	if (cRefs == 0) {
#if GT_TRACE
		OLDR_debugMask.flags = 0;
#endif
	}

	DBC_Ensure(cRefs >= 0);
}

/*
 *  ======== OLDR_Free ========
 *  Purpose:
 *  	Free the loader object.
 */
VOID OLDR_Free(struct DLDR_NODEOBJECT *hDldrNode)
{
	struct OLDR_NODEOBJECT *hOldrNode = (struct OLDR_NODEOBJECT *)hDldrNode;

	DBC_Require(cRefs > 0);
	DBC_Require(MEM_IsValidHandle(hOldrNode, OLDR_NODESIGNATURE));

	GT_1trace(OLDR_debugMask, GT_ENTER, "OLDR_Free(0x%x)\n", hDldrNode);

	MEM_FreeObject(hOldrNode);
}

/*
 *  ======== OLDR_GetFxnAddr ========
 *  Purpose:
 *  	Retrieves the DBL function addresses.
 */
DSP_STATUS OLDR_GetFxnAddr(struct DLDR_NODEOBJECT *hDldrNode, PSTR pstrFxn,
			  ULONG *pulAddr)
{
	struct DBL_Symbol *pSym;
	struct OLDR_NODEOBJECT *hOldrNode = (struct OLDR_NODEOBJECT *)hDldrNode;
	struct OLDR_OBJECT *hOldr;
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	DBC_Require(MEM_IsValidHandle(hOldrNode, OLDR_NODESIGNATURE));
	DBC_Require(pulAddr != NULL);
	DBC_Require(pstrFxn != NULL);

	GT_3trace(OLDR_debugMask, GT_ENTER, "OLDR_GetFxnAddr(0x%x, %s, 0x%x)\n",
		 hDldrNode, pstrFxn, pulAddr);

	hOldr = hOldrNode->pOldr;

	if (!hOldr->dblFxns.getAddrFxn(hOldr->lib, pstrFxn, &pSym)) {
		if (!hOldr->dblFxns.getCAddrFxn(hOldr->lib, pstrFxn, &pSym)) {
			GT_0trace(OLDR_debugMask, GT_7CLASS, "OLDR_GetFxnAddr: "
				 "Symbols not found\n");
			status = DSP_ESYMBOL;
		}
	}

	if (DSP_SUCCEEDED(status)) {
		*pulAddr = pSym->value;
	}

	return (status);
}

/*
 *  ======== OLDR_Init ========
 *  Initialize the OLDR module.
 */
BOOL OLDR_Init()
{
	BOOL fRetVal = TRUE;

	DBC_Require(cRefs >= 0);

	if (cRefs == 0) {
#if GT_TRACE
		DBC_Assert(!OLDR_debugMask.flags);
		GT_create(&OLDR_debugMask, "OL");	/* "OL" for OLdr */
#endif
	}

	if (fRetVal) {
		cRefs++;
	}

	GT_1trace(OLDR_debugMask, GT_5CLASS,
		 "OLDR_Init(), ref count:  0x%x\n", cRefs);

	DBC_Ensure((fRetVal && (cRefs > 0)) || (!fRetVal && (cRefs >= 0)));
	return (fRetVal);
}

/*
 *  ======== OLDR_Load ========
 *  Purpose:
 *  	Load a section on to the target memory.
 */
DSP_STATUS OLDR_Load(struct DLDR_NODEOBJECT *hDldrNode, DLDR_PHASE phase)
{
	struct OLDR_NODEOBJECT *hOldrNode = (struct OLDR_NODEOBJECT *)hDldrNode;
	struct OLDR_OBJECT *hOldr;
	DBL_Attrs attrs;
	CHAR szSectName[MAXNAMELEN];
	CHAR *pch;
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	DBC_Require(MEM_IsValidHandle(hOldrNode, OLDR_NODESIGNATURE));

	GT_2trace(OLDR_debugMask, GT_ENTER, "OLDR_Load(0x%x, 0x%x)\n",
		 hDldrNode, phase);

	hOldr = hOldrNode->pOldr;

	if (hOldrNode->fOvly) {
		/* Create section name from UUID, phase */
		CSL_Strcpyn(szSectName, hOldrNode->szUUID, MAXUUIDLEN);
		pch = szSectName + CSL_Strlen(szSectName);
		*pch++ = ':';
		CSL_NumToAscii(pch, phase);

		attrs = hOldr->dblAttrs;
		attrs.wHandle = hOldrNode->pPrivRef;
		status = hOldr->dblFxns.loadSectFxn(hOldr->lib, szSectName,
			 &attrs);
	}

	return (status);
}

/*
 *  ======== OLDR_Unload ========
 *  Purpose:
 *  	Unloads a section from the target memory.
 */
DSP_STATUS OLDR_Unload(struct DLDR_NODEOBJECT *hDldrNode, DLDR_PHASE phase)
{
	struct OLDR_NODEOBJECT *hOldrNode = (struct OLDR_NODEOBJECT *)hDldrNode;
	struct OLDR_OBJECT *hOldr;
	CHAR szSectName[MAXNAMELEN];
	CHAR *pch;
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	DBC_Require(MEM_IsValidHandle(hOldrNode, OLDR_NODESIGNATURE));

	GT_2trace(OLDR_debugMask, GT_ENTER, "OLDR_Unload(0x%x, 0x%x)\n",
		 hDldrNode, phase);

	hOldr = hOldrNode->pOldr;

	if (hOldrNode->fOvly) {
		/* Create section name from UUID, phase */
		CSL_Strcpyn(szSectName, hOldrNode->szUUID, MAXUUIDLEN);
		pch = szSectName + CSL_Strlen(szSectName);
		*pch++ = ':';
		CSL_NumToAscii(pch, phase);

		status = hOldr->dblFxns.unloadSectFxn(hOldr->lib, szSectName,
			 &hOldr->dblAttrs);
		DBC_Assert(DSP_SUCCEEDED(status));
	}

	return (status);
}

