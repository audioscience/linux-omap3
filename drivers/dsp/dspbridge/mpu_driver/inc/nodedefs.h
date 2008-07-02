/*
 * dspbridge/mpu_driver/inc/nodedefs.h
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
 *  ======== nodedefs.h ========
 *  Description:
 *      Global NODE constants and types, shared by PROCESSOR, NODE, and DISP.
 *
 *! Revision History
 *! ================
 *! 23-Apr-2001 jeh     Removed NODE_MGRATTRS.
 *! 21-Sep-2000 jeh     Removed NODE_TYPE enum.
 *! 17-Jul-2000 jeh     Changed order of node types to match rms_sh.h.
 *! 20-Jun-2000 jeh     Created.
 */

#ifndef NODEDEFS_
#define NODEDEFS_

#ifdef __cplusplus
extern "C" {
#endif

#define NODE_SUSPENDEDPRI -1

/* NODE Objects: */
	struct NODE_MGR;
	/*typedef struct NODE_MGR *NODE_HMGR;*/
	struct NODE_OBJECT;
	/*typedef struct NODE_OBJECT *NODE_HOBJECT;*/

#ifdef __cplusplus
}
#endif
#endif				/* NODEDEFS_ */
