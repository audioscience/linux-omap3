/*
 * dspbridge/inc/oldr.h
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
 *  ======== oldr.h ========
 *  Description:
 *      DSP/BIOS Bridge static/overlay loader interface.
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
 *  Notes:
 *
 *! Revision History
 *! ================
 *! 19-Nov-2001 jeh     Code review changes.
 *! 24-Aug-2001 jeh     Created.
 */

#ifndef OLDR_
#define OLDR_

#ifdef __cplusplus
extern "C" {
#endif

#include <dldrdefs.h>
#include <oldrdefs.h>

/*
 *  ======== OLDR_Allocate ========
 *  Purpose:
 *      Allocate resources to manage the loading of a node on the DSP.
 *  Parameters:
 *      hOldr:          Handle of loader that will load the node.
 *      pPrivRef:       Handle to identify the node.
 *      pNodeProps:     Pointer to a DCD_NODEPROPS for the node.
 *      phOldrNode:     Location to store node handle on output. This handle
 *                      will be passed to OLDR_Load/OLDR_Unload.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EMEMORY:    Insufficient memory on GPP.
 *  Requires:
 *      OLDR_Init() called.
 *      Valid hOldr.
 *      pNodeProps != NULL.
 *      phOldrNode != NULL.
 *  Ensures:
 *      DSP_SOK:        Valid *phOldrNode.
 *      error:          *phOldrNode == NULL.
 */
	extern DSP_STATUS OLDR_Allocate(struct DLDR_OBJECT *hOldr,
					PVOID pPrivRef,
					IN CONST struct DCD_NODEPROPS
					*pNodeProps,
					OUT DLDR_HNODE * phOldrNode);

/*
 *  ======== OLDR_Create ========
 *  Purpose:
 *      Create a loader object. This object handles the loading and unloading
 *      of create, delete, and execute phase functions of nodes on the DSP
 *      target.
 *  Parameters:
 *      phOldr:         Location to store loader handle on output.
 *      hDevObject:     Device for this processor.
 *      pAttrs:         Loader attributes.
 *  Returns:
 *      DSP_SOK:        Success;
 *      DSP_EMEMORY:    Insufficient memory for requested resources.
 *  Requires:
 *      OLDR_Init() called.
 *      phOldr != NULL.
 *      hDevObject != NULL.
 *      pAttrs != NULL.
 *  Ensures:
 *      DSP_SOK:        Valid *phOldr.
 *      error:          *phOldr == NULL.
 */
	extern DSP_STATUS OLDR_Create(OUT struct DLDR_OBJECT **phOldr,
				      struct DEV_OBJECT *hDevObject,
				      IN CONST struct DLDR_ATTRS *pAttrs);

/*
 *  ======== OLDR_Delete ========
 *  Purpose:
 *      Delete the OLDR loader.
 *  Parameters:
 *      hOldr:          Node manager object.
 *  Returns:
 *  Requires:
 *      OLDR_Init() called.
 *      Valid hOldr.
 *  Ensures:
 */
	extern VOID OLDR_Delete(struct DLDR_OBJECT *hOldr);

/*
 *  ======== OLDR_Exit ========
 *  Purpose:
 *      Discontinue usage of OLDR module.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      OLDR_Init() successfully called before.
 *  Ensures:
 *      Any resources acquired in OLDR_Init() will be freed when last OLDR
 *      client calls OLDR_Exit().
 */
	extern VOID OLDR_Exit();

/*
 *  ======== OLDR_Free ========
 *  Purpose:
 *      Free resource allocated in OLDR_Allocate().
 *  Parameters:
 *      hOldrNode:      Handle returned for OLDR_Allocate().
 *  Returns:
 *  Requires:
 *      OLDR_Init() called.
 *      Valid hOldrNode.
 *  Ensures:
 */
	extern VOID OLDR_Free(DLDR_HNODE hOldrNode);

/*
 *  ======== OLDR_GetFxnAddr ========
 *  Purpose:
 *      Get address of create, delete, or execute phase function of a node on
 *      the DSP.
 *  Parameters:
 *      hOldrNode:      Handle returned from OLDR_Allocate().
 *      pstrFxn:        Name of function.
 *      pulAddr:        Location to store function address.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_ESYMBOL:    Address of function not found.
 *  Requires:
 *      OLDR_Init() called.
 *      Valid hOldrNode.
 *      pulAddr != NULL.
 *      pstrFxn != NULL.
 *  Ensures:
 */
	extern DSP_STATUS OLDR_GetFxnAddr(DLDR_HNODE hOldrNode, PSTR pstrFxn,
					  ULONG *pulAddr);

/*
 *  ======== OLDR_Init ========
 *  Purpose:
 *      Initialize the OLDR module.
 *  Parameters:
 *  Returns:
 *      TRUE if initialization succeeded, FALSE otherwise.
 *  Requires:
 *  Ensures:
 */
	extern BOOL OLDR_Init();

/*
 *  ======== OLDR_Load ========
 *  Purpose:
 *      Load create, delete, or execute phase function of a node on the DSP.
 *  Parameters:
 *      hOldrNode:          Handle returned from OLDR_Allocate().
 *      phase:              Type of function to load (create, delete, or
 *                          execute).
 *  Returns:
 *      DSP_SOK:            Success.
 *      DSP_EMEMORY:        Insufficient memory on GPP.
 *      DSP_EOVERLAYMEMORY: Can't load because memory is already in use by
 *                          another overlay.
 *      DSP_EFWRITE:        Overlay function failed to copy sections on DSP.
 *  Requires:
 *      OLDR_Init() called.
 *      Valid hOldrNode.
 *  Ensures:
 */
	extern DSP_STATUS OLDR_Load(DLDR_HNODE hOldrNode, DLDR_PHASE phase);

/*
 *  ======== OLDR_Unload ========
 *  Purpose:
 *      Unload create, delete, or execute phase function of a node on the DSP.
 *  Parameters:
 *      hOldrNode:      Handle returned from OLDR_Allocate().
 *      phase:          Node function to unload (create, delete, or execute).
 *  Returns:
 *      DSP_SOK:        Success.
 *  Requires:
 *      OLDR_Init() called.
 *      Valid hOldrNode.
 *  Ensures:
 */
	extern DSP_STATUS OLDR_Unload(DLDR_HNODE hOldrNode, DLDR_PHASE phase);

#ifdef __cplusplus
}
#endif
#endif				/* OLDR_ */
