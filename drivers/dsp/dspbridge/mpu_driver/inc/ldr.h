/*
 * dspbridge/inc/ldr.h
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
 *  ======== ldr.h ========
 *  Purpose:
 *      Provide module loading services and symbol export services.
 *
 *  Public Functions:
 *      LDR_Exit
 *      LDR_FreeModule
 *      LDR_GetProcAddress
 *      LDR_Init
 *      LDR_LoadModule
 *
 *  Notes:
 *      This service is meant to be used by modules of the DSP/BIOS Bridge
 *       class driver.
 *
 *! Revision History:
 *! ================
 *! 22-Nov-1999 kc: Changes from code review.
 *! 12-Nov-1999 kc: Removed declaration of unused loader object.
 *! 29-Oct-1999 kc: Cleaned up for code review.
 *! 12-Jan-1998 cr: Cleaned up for code review.
 *! 04-Aug-1997 cr: Added explicit CDECL identifiers.
 *! 11-Nov-1996 cr: Cleaned up for code review.
 *! 16-May-1996 gp: Created.
 */

#ifndef LDR_
#define LDR_

#ifdef __cplusplus
extern "C" {
#endif

#include <dspapi.h>

/* Loader objects: */
	struct LDR_MODULE;
	/*typedef struct LDR_MODULE *LDR_HMODULE;*/

/*
 *  ======== LDR_Exit ========
 *  Purpose:
 *      Discontinue usage of module; free resources when reference count
 *      reaches 0.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      LDR initialized.
 *  Ensures:
 *      Resources used by module are freed when cRef reaches zero.
 */
	extern VOID LDR_Exit();

/*
 *  ======== LDR_FreeModule ========
 *  Purpose:
 *      Decrements the usage count on the given module, and frees it when it
 *      gets to zero.
 *  Parameters:
 *      hModule:        Handle to a DLL module, returned by LDR_LoadModule.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EHANDLE:    hModule was invalid.
 *      DSP_EFAIL:      General failure.
 *  Requires:
 *      - LDR initialized.
 *      - hModule is valid.
 *  Ensures:
 */
	extern DSP_STATUS LDR_FreeModule(IN struct LDR_MODULE *hModule);

/*
 *  ======== LDR_GetProcAddress ========
 *  Purpose:
 *      Retrieve the address of the given function name for a currently
 *      loaded module.
 *  Parameters:
 *      hModule:        Handle to the module.
 *      pstrFuncName:   Name of the function whose address is to be retrieved.
 *      ppAddr:         Ptr to a location to store the fxn address.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EFAIL:      Function name or module not found.
 *  Requires:
 *      - LDR initialized.
 *      - hModule is valid.
 *      - ppAddr != NULL.
 *      - pstrFuncName != NULL.
 *  Ensures:
 *      On DSP_SOK, ppAddr contains the address of the named function.
 */
	extern DSP_STATUS LDR_GetProcAddress(IN struct LDR_MODULE *hModule,
					     IN PSTR pstrFuncName,
					     OUT PVOID *ppAddr);

/*
 *  ======== LDR_Init ========
 *  Purpose:
 *      Initializes private state of LDR module.
 *  Parameters:
 *  Returns:
 *      TRUE if initialized; FALSE if error occured.
 *  Requires:
 *  Ensures:
 *      LDR initialized.
 */
	extern BOOL LDR_Init();

/*
 *  ======== LDR_LoadModule ========
 *  Purpose:
 *      Load a DLL module, and increment the usage count on that module.
 *      Does not perform further DLL loading of other modules to resolve
 *      external dependencies of the loaded DLL.
 *  Parameters:
 *      phModule:       Ptr to location to receive the module handle.
 *      pstrFileName:   String containing the path and filename to the DLL.
 *                      If the path is not specified, a default path is
 *                      is searched to find file.
 *  Returns:
 *      DSP_SOK:                Success.
 *      DSP_EMEMORY:            Insufficient memory to complete.
 *      LDR_E_FILEUNABLETOOPEN: Couldn't load the requested file.
 *  Requires:
 *      LDR initialized.
 *      phModule != NULL.
 *  Ensures:
 *      On DSP_SOK, the desired DLL is loaded and a valid module handle is
 *      returned.
 */
	extern DSP_STATUS LDR_LoadModule(OUT struct LDR_MODULE **phModule,
					 IN CONST PSTR pstrFileName);

#ifdef __cplusplus
}
#endif
#endif				/* LDR_ */
