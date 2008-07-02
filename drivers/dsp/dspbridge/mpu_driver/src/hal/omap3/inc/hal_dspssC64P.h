/*
 * dspbridge/src/hal/common/inc/arm11/hal_dspss.h
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
 *  ======== hal_dspss.h ========
 *  Description:
 *      DSP Subsystem API declarations
 *
 *! Revision History:
 *! ================
 *! 19-Apr-2004 sb: Removed redundant argument from HAL_DSPSS_IPIEndianismSet
 *!		    Moved endianness and element size to generic hal_defs.h
 *! 16 Feb 2003 sb: Initial version
 */
#ifndef __HAL_DSPSS_H
#define __HAL_DSPSS_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
* EXPORTED TYPES
* =============================================================================
*/
	typedef enum HAL_DSPSYSC_BootMode {
		HAL_DSPSYSC_DIRECTBOOT = 0x0,
		HAL_DSPSYSC_IDLEBOOT = 0x1,
		HAL_DSPSYSC_SELFLOOPBOOT = 0x2,
		HAL_DSPSYSC_USRBOOTSTRAP = 0x3,
		HAL_DSPSYSC_DEFAULTRESTORE = 0x4
	} HAL_DSPSYSC_BootMode_t;

#define HAL_DSP_IDLEBOOT_ADDR   0x007E0000

/* ============================================================================
* EXPORTED FUNCTIONS
* =============================================================================
*/
	extern HAL_STATUS HAL_DSPSS_BootModeSet(const UWORD32 baseAddress,
						HAL_DSPSYSC_BootMode_t bootMode,
						const UWORD32 bootAddress);

#ifdef __cplusplus
}
#endif
#endif				/* __HAL_DSPSS_H */
