/*
 * dspbridge/src/dynload/dlinit.h
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



#ifndef _DLINIT_H
#define _DLINIT_H

/**************************************************************************
 **************************************************************************
 *
 *                              DLINIT.H
 *
 * A class used by the dynamic loader to load data into a target.  This
 class provides the interface-specific functions needed to load data.
 *
 **************************************************************************
 **************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include "dynamic_loader.h"

/* DL_init_t remembers the start address of the dynamic module in the DL */
/* memory initializer class                                             */

	struct DL_init_t {
		struct Dynamic_Loader_Initialize init;
		LDR_ADDR start_address;
	};

/**************************************************************************
 * NOTE:    All methods take a 'thisptr' parameter that is a pointer to the
 *          environment for the DL Memory Initializer APIs.  See definition
   of    DL_init_t.
 *
 **************************************************************************/

/***************************************************************************
* DLinit_init
*
* Parameters:
*   none
*
* Effect:
*   Initialize the handlers for the memory initialization classes

***************************************************************************/
	void DLinit_init(struct DL_init_t *thisptr);

#ifdef __cplusplus
}
#endif
#endif				/* _DLINIT_H */
