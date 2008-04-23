/*
 * dspbridge/src/osal/linux/regsup.h
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
 *  ======== regsup.h ========
 *
 *! Revision History
 *! ================
 */

#ifndef _REGSUP_H_
#define _REGSUP_H_

#define BRIDGE_MAX_NAME_SIZE                     MAXREGPATHLENGTH
#define BRIDGE_MAX_NUM_REG_ENTRIES               52

/*  Init function. MUST be called BEFORE any calls are  */
/*  made into this psuedo-registry!!!  Returns TRUE/FALSE for SUCCESS/ERROR  */
extern BOOL regsupInit(void);

/*  Release all registry support allocations.  */
extern void regsupExit(void);

/*
 *  ======== regsupDeleteValue ========
 */
extern DSP_STATUS regsupDeleteValue(IN CONST PSTR pstrSubkey,
				    IN CONST PSTR pstrValue);
/*  Get the value of the entry having the given name.  Returns DSP_SOK  */
/*  if an entry was found and the value retrieved.  Returns DSP_EFAIL
 *  otherwise.*/
extern DSP_STATUS regsupGetValue(char *valName, void *pBuf, DWORD *dataSize);

/*  Sets the value of the entry having the given name.  Returns DSP_SOK  */
/*  if an entry was found and the value set.  Returns DSP_EFAIL otherwise.  */
extern DSP_STATUS regsupSetValue(char *valName, void *pBuf, DWORD dataSize);

/*  Returns registry "values" and their "data" under a (sub)key. */
extern DSP_STATUS regsupEnumValue(IN DWORD dwIndex, IN CONST PSTR pstrKey,
			IN OUT PSTR pstrValue, IN OUT DWORD *pdwValueSize,
			IN OUT PSTR pstrData, IN OUT DWORD *pdwDataSize);

#endif

