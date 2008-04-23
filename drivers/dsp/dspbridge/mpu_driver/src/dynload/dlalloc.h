/*
 * dspbridge/src/dynload/dlalloc.h
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



#ifndef _DLALLOC_H
#define _DLALLOC_H

/*****************************************************************************
 *****************************************************************************
 *
 *                          DLALLOC.H
 *
 * A class used by the dynamic loader to allocate and deallocate target memory.
 *
 *****************************************************************************
 *****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include "dynamic_loader.h"

/* DL_alloc_t mimics DL memory allocation type */
/* Users can customize if desired */
	/*typedef Dynamic_Loader_Allocate DL_alloc_t;*/

/**************************************************************************
 * NOTE:    All methods take a 'thisptr' parameter that is a pointer to the
 *          environment for the DL Memory Allocator APIs.  See definition
 *			of  DL_alloc_t.
 *
 *************************************************************************/

/**************************************************************************
* DLalloc_Init
*
* Parameters:
*   None
*
* Effect:
*   Initialize the handlers for the Allocation Class
***************************************************************************/
	void DLalloc_init(struct Dynamic_Loader_Allocate *thisptr);

#ifdef __cplusplus
}
#endif
#endif /*_DLALLOC_H */
