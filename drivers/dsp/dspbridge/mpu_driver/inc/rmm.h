/*
 * dspbridge/mpu_driver/inc/rmm.h
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
 *  ======== rmm.h ========
 *
 *  This memory manager provides general heap management and arbitrary
 *  alignment for any number of memory segments, and management of overlay
 *  memory.
 *
 *  Public functions:
 *      RMM_alloc
 *      RMM_create
 *      RMM_delete
 *      RMM_exit
 *      RMM_free
 *      RMM_init
 *
 *! Revision History
 *! ================
 *! 25-Jun-2002 jeh     Added RMM_Addr. Removed RMM_reserve, RMM_stat.
 *! 15-Oct-2001 jeh     Based on rm.h in gen tree.
 */

#ifndef RMM_
#define RMM_

/*
 *  ======== RMM_Addr ========
 *  DSP address + segid
 */
struct RMM_Addr {
	LgUns addr;
	Int segid;
} ;

/*
 *  ======== RMM_Segment ========
 *  Memory segment on the DSP available for remote allocations.
 */
struct RMM_Segment {
	LgUns base;		/* Base of the segment */
	LgUns length;		/* Size of the segment (target MAUs) */
	Int space;		/* Code or data */
	LgUns number;		/* Number of Allocated Blocks */
} ;

/*
 *  ======== RMM_Target ========
 */
struct RMM_TargetObj;
/*typedef struct RMM_TargetObj *RMM_Target;*/

/*
 *  ======== RMM_alloc ========
 *
 *  RMM_alloc is used to remotely allocate or reserve memory on the DSP.
 *
 *  Parameters:
 *      target          - Target returned from RMM_create().
 *      segid           - Memory segment to allocate from.
 *      size            - Size (target MAUS) to allocate.
 *      align           - alignment.
 *      dspAddr         - If reserve is FALSE, the location to store allocated
 *                        address on output, otherwise, the DSP address to
 *                        reserve.
 *      reserve         - If TRUE, reserve the memory specified by dspAddr.
 *  Returns:
 *      DSP_SOK:                Success.
 *      DSP_EMEMORY:            Memory allocation on GPP failed.
 *      DSP_EOVERLAYMEMORY:     Cannot "allocate" overlay memory because it's
 *                              already in use.
 *  Requires:
 *      RMM initialized.
 *      Valid target.
 *      dspAddr != NULL.
 *      size > 0
 *      reserve || target->numSegs > 0.
 *  Ensures:
 */
extern DSP_STATUS RMM_alloc(struct RMM_TargetObj *target, Uns segid, LgUns size,
			    Uns align, LgUns *dspAdr, Bool reserve);

/*
 *  ======== RMM_create ========
 *  Create a target object with memory segments for remote allocation. If
 *  segTab == NULL or numSegs == 0, memory can only be reserved through
 *  RMM_alloc().
 *
 *  Parameters:
 *      pTarget:        - Location to store target on output.
 *      segTab:         - Table of memory segments.
 *      numSegs:        - Number of memory segments.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EMEMORY:    Memory allocation failed.
 *  Requires:
 *      RMM initialized.
 *      pTarget != NULL.
 *      numSegs == 0 || segTab != NULL.
 *  Ensures:
 *      Success:        Valid *pTarget.
 *      Failure:        *pTarget == NULL.
 */
extern DSP_STATUS RMM_create(struct RMM_TargetObj **pTarget,
			     struct RMM_Segment segTab[], Uns numSegs);

/*
 *  ======== RMM_delete ========
 *  Delete target allocated in RMM_create().
 *
 *  Parameters:
 *      target          - Target returned from RMM_create().
 *  Returns:
 *  Requires:
 *      RMM initialized.
 *      Valid target.
 *  Ensures:
 */
extern Void RMM_delete(struct RMM_TargetObj *target);

/*
 *  ======== RMM_exit ========
 *  Exit the RMM module
 *
 *  Parameters:
 *  Returns:
 *  Requires:
 *      RMM_init successfully called.
 *  Ensures:
 */
extern Void RMM_exit(Void);

/*
 *  ======== RMM_free ========
 *  Free or unreserve memory allocated through RMM_alloc().
 *
 *  Parameters:
 *      target:         - Target returned from RMM_create().
 *      segid:          - Segment of memory to free.
 *      dspAddr:        - Address to free or unreserve.
 *      size:           - Size of memory to free or unreserve.
 *      reserved:       - TRUE if memory was reserved only, otherwise FALSE.
 *  Returns:
 *  Requires:
 *      RMM initialized.
 *      Valid target.
 *      reserved || segid < target->numSegs.
 *      reserve || [dspAddr, dspAddr + size] is a valid memory range.
 *  Ensures:
 */
extern Bool RMM_free(struct RMM_TargetObj *target, Uns segid, LgUns dspAddr,
		     LgUns size, Bool reserved);

/*
 *  ======== RMM_init ========
 *  Initialize the RMM module
 *
 *  Parameters:
 *  Returns:
 *      TRUE:   Success.
 *      FALSE:  Failure.
 *  Requires:
 *  Ensures:
 */
extern Bool RMM_init(Void);

/*
 *  ======== RMM_stat ========
 *  Obtain  memory segment status
 *
 *  Parameters:
 *      segid:       Segment ID of the dynamic loading segment.
 *      pMemStatBuf: Pointer to allocated buffer into which memory stats are
 *                   placed.
 *  Returns:
 *      TRUE:   Success.
 *      FALSE:  Failure.
 *  Requires:
 *      segid < target->numSegs
 *  Ensures:
 */
extern Bool RMM_stat(struct RMM_TargetObj *target, DSP_MEMTYPE segid,
		     struct DSP_MEMSTAT *pMemStatBuf);

#endif				/* RMM_ */
