/*
 * dspbridge/mpu_driver/inc/osal.h
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
 *  ======== osal.h ========
 *  Purpose:
 *      Provide loading and unloading of OSAL modules.
 *
 *  Public Functions:
 *      OSAL_Exit()
 *      OSAL_Init()
 *
 *! Revision History:
 *! ================
 *! 01-Feb-2000 kc: Created.
 */

#ifndef OSAL_
#define OSAL_

#ifdef __cplusplus
extern "C" {
#endif

#include <dspapi.h>

/*
 *  ======== OSAL_Exit ========
 *  Purpose:
 *      Discontinue usage of module; free resources when reference count
 *      reaches 0.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      OSAL initialized.
 *  Ensures:
 *      Resources used by module are freed when cRef reaches zero.
 */
	extern VOID OSAL_Exit();

/*
 *  ======== OSAL_Init ========
 *  Purpose:
 *      Initializes OSAL modules.
 *  Parameters:
 *  Returns:
 *      TRUE if all modules initialized; otherwise FALSE.
 *  Requires:
 *  Ensures:
 *      OSAL modules initialized.
 */
	extern BOOL OSAL_Init();

#ifdef __cplusplus
}
#endif
#endif				/* OSAL_ */
