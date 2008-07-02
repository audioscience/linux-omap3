/*
 * dspbridge/mpu_driver/inc/iodefs.h
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
 *  ======== iodefs.h ========
 *  Description:
 *      System-wide channel objects and constants.
 *
 *! Revision History:
 *! ================
 *! 07-Nov-2000 jeh     Created.
 */

#ifndef IODEFS_
#define IODEFS_

#ifdef __cplusplus
extern "C" {
#endif

#define IO_MAXIRQ   0xff	/* Arbitrarily large number. */

/* IO Objects: */
	struct IO_MGR;
	/*typedef struct IO_MGR *IO_HMGR; */

/* IO manager attributes: */
	struct IO_ATTRS {
		BYTE bIRQ;	/* Channel's I/O IRQ number. */
		BOOL fShared;	/* TRUE if the IRQ is shareable. */
		UINT uWordSize;	/* DSP Word size. */
		DWORD dwSMBase;	/* Physical base address of shared memory. */
		ULONG uSMLength;	/* Size (in bytes) of shared memory. */
	} ;

#ifdef __cplusplus
}
#endif
#endif				/* IODEFS_ */
