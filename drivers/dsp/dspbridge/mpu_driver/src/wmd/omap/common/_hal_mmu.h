/*
 * dspbridge/src/wmd/linux/omap/common/_hal_mmu.h
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
 *  ======== _hal_mmu.h ========
 *  Description:
 *      Defines lower edge MMU functions.
 *
 *! Revision History:
 *! ================
 *! 19-Feb-2003 vp: Code review updates.
 *! 18-Oct-2002 sb: Ported to Linux Platform
 *! 21-Sep-2001 kc: created.
 */

#ifndef _HAL_MMU_
#define _HAL_MMU_

#ifdef __cplusplus
extern "C" {
#endif

#include "_tiomap.h"
#if defined(OMAP_2430) || defined(OMAP_3430)

/* ======== HAL_DspMmuIntrClear ========
 * purpose:
 *  Clers MMU Interrupt register.
 */
VOID HAL_DspMmuIntrClear(IN struct WMD_DEV_CONTEXT *pDevContext,
			OUT struct DSP_ERRORINFO *pErrInfo);

/* ======== dspMmuTlbEntry ========
 * purose:
 *  Makes a DSP MMU entry
 */
extern VOID dspMmuTlbEntry(struct WMD_DEV_CONTEXT *pDevContext,
			  ULONG physical_address,
			  ULONG virtual_address, SLST_t slst_bit, AP_t ap_bits,
			  UCHAR locked_base_value, UCHAR current_entry,
			  PRESERVED_t p_bit);
#endif

#ifdef __cplusplus
}
#endif
#endif				/* _HAL_MMU_ */
