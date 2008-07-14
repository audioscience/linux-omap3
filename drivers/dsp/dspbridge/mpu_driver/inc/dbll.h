/*
 * dspbridge/mpu_driver/inc/dbll.h
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
 *  ======== dbll.h ========
 *  DSP/BIOS Bridge Dynamic load library module interface. Function header
 *  comments are in the file dblldefs.h.
 *
 *! Revision History
 *! ================
 *! 31-Jul-2002 jeh     Removed function comments (now in dblldefs.h).
 *! 17-Apr-2002 jeh     Created based on zl.h.
 */

#ifndef DBLL_
#define DBLL_

#include <dbdefs.h>
#include <dblldefs.h>

	extern Void DBLL_close(struct DBLL_LibraryObj *lib);
	extern DSP_STATUS DBLL_create(struct DBLL_TarObj **pTarget,
				      struct DBLL_Attrs *pAttrs);
	extern Void DBLL_delete(struct DBLL_TarObj *target);
	extern Void DBLL_exit(Void);
	extern Bool DBLL_getAddr(struct DBLL_LibraryObj *lib, String name,
				 struct DBLL_Symbol **ppSym);
	extern Void DBLL_getAttrs(struct DBLL_TarObj *target,
				  struct DBLL_Attrs *pAttrs);
	extern Bool DBLL_getCAddr(struct DBLL_LibraryObj *lib, String name,
				  struct DBLL_Symbol **ppSym);
	extern DSP_STATUS DBLL_getSect(struct DBLL_LibraryObj *lib, String name,
				       LgUns *pAddr, LgUns *pSize);
	extern Bool DBLL_init(Void);
	extern DSP_STATUS DBLL_load(struct DBLL_LibraryObj *lib,
				    DBLL_Flags flags,
				    struct DBLL_Attrs *attrs, LgUns *pEntry);
	extern DSP_STATUS DBLL_loadSect(struct DBLL_LibraryObj *lib,
					String sectName,
					struct DBLL_Attrs *attrs);
	extern DSP_STATUS DBLL_open(struct DBLL_TarObj *target, String file,
				    DBLL_Flags flags,
				    struct DBLL_LibraryObj **pLib);
	extern DSP_STATUS DBLL_readSect(struct DBLL_LibraryObj *lib,
					String name,
					Char *pBuf, LgUns size);
	extern Void DBLL_setAttrs(struct DBLL_TarObj *target,
				  struct DBLL_Attrs *pAttrs);
	extern Void DBLL_unload(struct DBLL_LibraryObj *lib,
				struct DBLL_Attrs *attrs);
	extern DSP_STATUS DBLL_unloadSect(struct DBLL_LibraryObj *lib,
					  String sectName,
					  struct DBLL_Attrs *attrs);

#endif				/* DBLL_ */

