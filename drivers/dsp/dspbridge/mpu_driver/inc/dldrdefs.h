/*
 * dspbridge/mpu_driver/inc/dldrdefs.h
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
 *  ======== dldrdefs.h ========
 *
 *  Description:
 *      DSP/BIOS Bridge loader interface. This is the interface shared by
 *  all loaders (eg, static loader and dynamic loader). This interface will
 *  be used by NODE.
 *
 *! Revision History
 *! ================
 *! 16-Sep-2002 map Updated with code review changes
 *! 24-Apr-2002 jeh     Added DLDR_WRITEFXN.
 *! 05-Nov-2001 jeh     Changed some function error return codes.
 *! 17-Sep-2001 jeh     Added function typedefs.
 *! 22-Aug-2001 jeh     Created.
 */

#ifndef DLDRDEFS_
#define DLDRDEFS_

#include <dbdcddef.h>
#include <dev.h>

#define DLDR_MAXPATHLENGTH       255

/* DLDR Objects: */
	struct DLDR_OBJECT;
	/*typedef struct DLDR_OBJECT *DLDR_HOBJECT;*/
	struct DLDR_NODEOBJECT;
	/*typedef struct DLDR_NODEOBJECT *DLDR_HNODE;*/

/*
 *  ======== DLDR_LOADTYPE ========
 *  Load types for a node. Must match values in node.h55.
 */
	typedef enum DLDR_LOADTYPE {
		DLDR_STATICLOAD,	/* Linked in base image, not overlay */
		DLDR_DYNAMICLOAD,	/* Dynamically loaded node */
		DLDR_OVLYLOAD	/* Linked in base image, overlay node */
	} DLDR_LOADTYPE;

/*
 *  ======== DLDR_OVLYFXN ========
 *  Causes code or data to be copied from load address to run address. This
 *  is the "COD_WRITEFXN" that gets passed to the DBL_Library and is used as
 *  the ZL write function.
 *
 *  Parameters:
 *      pPrivRef:       Handle to identify the node.
 *      ulDspRunAddr:   Run address of code or data.
 *      ulDspLoadAddr:  Load address of code or data.
 *      ulNumBytes:     Number of (GPP) bytes to copy.
 *      nMemSpace:      RMS_CODE or RMS_DATA.
 *  Returns:
 *      ulNumBytes:     Success.
 *      0:              Failure.
 *  Requires:
 *  Ensures:
 */
	typedef ULONG(CDECL * DLDR_OVLYFXN) (PVOID pPrivRef, ULONG ulDspRunAddr,
					     ULONG ulDspLoadAddr,
					     ULONG ulNumBytes, UINT nMemSpace);

/*
 *  ======== DLDR_WRITEFXN ========
 *  Write memory function. Used for dynamic load writes.
 *  Parameters:
 *      pPrivRef:       Handle to identify the node.
 *      ulDspAddr:      Address of code or data.
 *      pBuf:           Code or data to be written
 *      ulNumBytes:     Number of (GPP) bytes to write.
 *      nMemSpace:      DBL_DATA or DBL_CODE.
 *  Returns:
 *      ulNumBytes:     Success.
 *      0:              Failure.
 *  Requires:
 *  Ensures:
 */
	typedef ULONG(CDECL * DLDR_WRITEFXN) (PVOID pPrivRef,
					      ULONG ulDspAddr,
					      PVOID pBuf,
					      ULONG ulNumBytes, UINT nMemSpace);

/*
 *  ======== DLDR_ATTRS ========
 *  Attributes passed to DLDR_Create function.
 */
	struct DLDR_ATTRS {
		DLDR_OVLYFXN pfnOvly;
		DLDR_WRITEFXN pfnWrite;
		USHORT usDSPWordSize;
		USHORT usDSPMauSize;
	} ;

/*
 *  ======== DLDR_PHASE ========
 *  Indicates node create, delete, or execute phase function.
 */
	typedef enum {
		DLDR_CREATE,
		DLDR_DELETE,
		DLDR_EXECUTE,
		DLDR_NOPHASE
	} DLDR_PHASE;

/*
 *  Typedefs of loader functions imported from a DLL, or defined in a
 *  function table.
 */

/*
 *  ======== DLDR_Allocate ========
 *  Allocate resources to manage the loading of a node on the DSP.
 *
 *  Parameters:
 *      hDldr:          Handle of loader that will load the node.
 *      pPrivRef:       Handle to identify the node.
 *      pNodeProps:     Pointer to a DCD_NODEPROPS for the node.
 *      phDldrNode:     Location to store node handle on output. This handle
 *                      will be passed to DLDR_Load/DLDR_Unload.
 *      pfPhaseSplit:   pointer to boolean variable referenced in node.c
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EMEMORY:    Insufficient memory on GPP.
 *  Requires:
 *      DLDR_Init() called.
 *      Valid hDldr.
 *      pNodeProps != NULL.
 *      phDldrNode != NULL.
 *  Ensures:
 *      DSP_SOK:        IsValidNode(*phDldrNode).
 *      error:          *phDldrNode == NULL.
 */
	typedef DSP_STATUS(*DLDR_ALLOCATEFXN) (struct DLDR_OBJECT *hDldr,
					       PVOID pPrivRef,
					       IN CONST struct DCD_NODEPROPS *
					       pNodeProps,
					       OUT struct DLDR_NODEOBJECT
					       **phDldrNode,
					       OUT BOOL *pfPhaseSplit);

/*
 *  ======== DLDR_Create ========
 *  Create a loader object. This object handles the loading and unloading of
 *  create, delete, and execute phase functions of nodes on the DSP target.
 *
 *  Parameters:
 *      phDldr:         Location to store loader handle on output.
 *      hDevObject:     Device for this processor.
 *      pAttrs:         Loader attributes.
 *  Returns:
 *      DSP_SOK:                Success;
 *      DSP_EMEMORY:            Insufficient memory for requested resources.
 *  Requires:
 *      DLDR_Init() called.
 *      phDldr != NULL.
 *      hDevObject != NULL.
 *  pAttrs != NULL.
 *  Ensures:
 *      DSP_SOK:        Valid *phDldr.
 *      error:          *phDldr == NULL.
 */
	typedef DSP_STATUS(*DLDR_CREATEFXN) (OUT struct DLDR_OBJECT **phDldr,
					     struct DEV_OBJECT *hDevObject,
					     IN CONST struct DLDR_ATTRS
					     *pAttrs);

/*
 *  ======== DLDR_Delete ========
 *  Delete the DLDR loader.
 *
 *  Parameters:
 *      hDldr:          Node manager object.
 *  Returns:
 *  Requires:
 *      DLDR_Init() called.
 *      Valid hDldr.
 *  Ensures:
 *  hDldr invalid
 */
	typedef VOID(*DLDR_DELETEFXN) (struct DLDR_OBJECT *hDldr);

/*
 *  ======== DLDR_Exit ========
 *  Discontinue usage of DLDR module.
 *
 *  Parameters:
 *  Returns:
 *  Requires:
 *      DLDR_Init() successfully called before.
 *  Ensures:
 *      Any resources acquired in DLDR_Init() will be freed when last DLDR
 *      client calls DLDR_Exit().
 */
	typedef VOID(*DLDR_EXITFXN) ();

/*
 *  ======== DLDR_Free ========
 *  Free resources allocated in DLDR_Allocate.
 *
 *  Parameters:
 *      hDldrNode:      Handle returned from DLDR_Allocate().
 *  Returns:
 *  Requires:
 *      DLDR_Init() called.
 *      Valid hDldrNode.
 *  Ensures:
 */
	typedef VOID(*DLDR_FREEFXN) (struct DLDR_NODEOBJECT *hDldrNode);

/*
 *  ======== DLDR_GetFxnAddr ========
 *  Get address of create, delete, or execute phase function of a node on
 *  the DSP.
 *
 *  Parameters:
 *      hDldrNode:      Handle returned from DLDR_Allocate().
 *      pstrFxn:        Name of function.
 *      pulAddr:        Location to store function address.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_ESYMBOL:    Address of function not found.
 *  Requires:
 *      DLDR_Init() called.
 *      Valid hDldrNode.
 *      pulAddr != NULL;
 *      pstrFxn != NULL;
 *  Ensures:
 */
	typedef DSP_STATUS(*DLDR_GETFXNADDRFXN) (struct DLDR_NODEOBJECT
						 *hDldrNode,
						 PSTR pstrFxn, ULONG *pulAddr);

/*
 *  ======== DLDR_Init ========
 *  Initialize the DLDR module.
 *
 *  Parameters:
 *  Returns:
 *      TRUE if initialization succeeded, FALSE otherwise.
 *  Ensures:
 */
	typedef BOOL(*DLDR_INITFXN) ();

/*
 *  ======== DLDR_Load ========
 *  Load create, delete, or execute phase function of a node on the DSP.
 *
 *  Parameters:
 *      hDldrNode:      Handle returned from DLDR_Allocate().
 *      phase:          Type of function to load (create, delete, or execute).
 *  Returns:
 *      DSP_SOK:                Success.
 *      DSP_EMEMORY:            Insufficient memory on GPP.
 *      DSP_EOVERLAYMEMORY:     Can't overlay phase because overlay memory
 *                              is already in use.
 *      DSP_EDYNLOAD:           Failure in dynamic loader library.
 *      DSP_EFWRITE:            Failed to write phase's code or date to target.
 *  Requires:
 *      DLDR_Init() called.
 *      Valid hDldrNode.
 *  Ensures:
 */
	typedef DSP_STATUS(*DLDR_LOADFXN) (struct DLDR_NODEOBJECT *hDldrNode,
					   DLDR_PHASE phase);

/*
 *  ======== DLDR_Unload ========
 *  Unload create, delete, or execute phase function of a node on the DSP.
 *
 *  Parameters:
 *      hDldrNode:      Handle returned from DLDR_Allocate().
 *      phase:          Node function to unload (create, delete, or execute).
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EMEMORY:    Insufficient memory on GPP.
 *  Requires:
 *      DLDR_Init() called.
 *      Valid hDldrNode.
 *  Ensures:
 */
	typedef DSP_STATUS(*DLDR_UNLOADFXN) (struct DLDR_NODEOBJECT *hDldrNode,
					     DLDR_PHASE phase);

/*
 *  ======== DLDR_FXNS ========
 */
	struct DLDR_FXNS {
		DLDR_ALLOCATEFXN pfnAllocate;
		DLDR_CREATEFXN pfnCreate;
		DLDR_DELETEFXN pfnDelete;
		DLDR_EXITFXN pfnExit;
		DLDR_FREEFXN pfnFree;
		DLDR_GETFXNADDRFXN pfnGetFxnAddr;
		DLDR_INITFXN pfnInit;
		DLDR_LOADFXN pfnLoad;
		DLDR_UNLOADFXN pfnUnload;
	} ;

#endif				/* DLDRDEFS_ */
