/*
 * dspbridge/src/wmd/linux/omap/common/mmu_fault.h
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
 *  ======== mmu_fault.h ========
 *  Description:
 *      Defines DSP MMU fault handling functions.
 *
 *! Revision History:
 *! ================
 *! 26-Dec-2004 hn: IVA MMU handlers.
 *! 10-Sep-2001 kc: created.
 */

#ifndef MMU_FAULT_
#define MMU_FAULT_

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== MMU_FaultDpc ========
 *  Purpose:
 *      Deferred procedure call to handle DSP MMU fault.
 */
	VOID MMU_FaultDpc(IN PVOID pRefData);

/*
 *  ======== MMU_FaultIsr ========
 *  Purpose:
 *      ISR to be triggered by a DSP MMU fault interrupt.
 */
	VOID MMU_FaultIsr(IN PVOID pRefData);


/* extern static DSP_STATUS */

/* PrintDspTraceBuffer(DEH_HMGR hDehMgr); */

	/*static BOOL MMU_CheckIfFault(struct WMD_DEV_CONTEXT  *pDevContext); */

#ifdef __cplusplus
}
#endif
#endif				/* MMU_FAULT_ */

