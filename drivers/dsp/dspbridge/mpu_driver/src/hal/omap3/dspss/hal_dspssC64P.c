/*
 * dspbridge/src/hal/common/dspss/hal_dspss64P.c
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
 *  ======== hal_dspss64P.c ========
 *  Description:
 *      API definitions to configure DSP Subsystem modules like IPI
 *
 *! Revision History:
 *! ================
 *! 19 Apr 2004 sb: Implemented HAL_DSPSS_IPIEndianismSet
 *! 16 Feb 2003 sb: Initial version
 */

/* PROJECT SPECIFIC INCLUDE FILES */
#include <GlobalTypes.h>
#include <hal_defs.h>
#include <hal_dspssC64P.h>
#include <IVA2RegAcM.h>
#include <IPIAccInt.h>

/* HAL FUNCTIONS */
HAL_STATUS HAL_DSPSS_BootModeSet(const UWORD32 baseAddress,
		      HAL_DSPSYSC_BootMode_t bootMode,
		      const UWORD32 bootAddress)
{
	HAL_STATUS status = RET_OK;
	UWORD32 offset = SYSC_IVA2BOOTMOD_OFFSET;
	UWORD32 alignedBootAddr;

	/* if Boot mode it DIRECT BOOT, check that the bootAddress is aligned to
	 * atleast 1K :: TODO */
	WR_MEM_32_VOLATILE((baseAddress) + offset, bootMode);

	offset = SYSC_IVA2BOOTADDR_OFFSET;

	alignedBootAddr = bootAddress & SYSC_IVA2BOOTADDR_MASK;

	WR_MEM_32_VOLATILE((baseAddress) + offset, alignedBootAddr);

	return status;
}
