/*
 * dspbridge/mpu_driver/inc/gh.h
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
 *  ======== gh.h ========
 *
 *! Revision History
 *! ================
 */

#ifndef GH_
#define GH_

extern struct GH_THashTab *GH_create(MdUns maxBucket, MdUns valSize,
			    MdUns(*hash) (Ptr, MdUns), Bool(*match) (Ptr, Ptr),
			    Void(*delete) (Ptr));
extern Void GH_delete(struct GH_THashTab *hashTab);
extern Void GH_exit(Void);
extern Ptr GH_find(struct GH_THashTab *hashTab, Ptr key);
extern Void GH_init(Void);
extern Ptr GH_insert(struct GH_THashTab *hashTab, Ptr key, Ptr value);
#endif				/* GH_ */
