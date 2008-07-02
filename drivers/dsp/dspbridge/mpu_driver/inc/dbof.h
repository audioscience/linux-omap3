/*
 * dspbridge/mpu_driver/inc/dbof.h
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
 *  ======== dbof.h ========
 *  Description:
 *      Defines and typedefs for DSP/BIOS Bridge Object File Format (DBOF).
 *
 *! Revision History
 *! ================
 *! 12-Jul-2002 jeh     Added defines for DBOF_SectHdr page.
 *! 12-Oct-2001 jeh     Converted to std.h format.
 *! 07-Sep-2001 jeh     Added overlay support.
 *! 06-Jul-2001 jeh     Created.
 */

#ifndef DBOF_
#define DBOF_

#ifdef __cplusplus
extern "C" {
#endif

/* Enough to hold DCD section names: 32 digit ID + underscores */
#define DBOF_DCDSECTNAMELEN     40

/* Values for DBOF_SectHdr page field. */
#define         DBOF_PROGRAM    0
#define         DBOF_DATA       1
#define         DBOF_CINIT      2

/*
 *  ======== DBOF_FileHdr ========
 */
	struct DBOF_FileHdr {
		LgUns magic;	/* COFF magic number */
		LgUns entry;	/* Program entry point */
		MdUns numSymbols;	/* Number of bridge symbols */
		MdUns numDCDSects;	/* Number of DCD sections */
		MdUns numSects;	/* Number of sections to load */
		MdUns numOvlySects;	/* Number of overlay sections */
		LgUns symOffset;	/* Offset in file to symbols */
		LgUns dcdSectOffset;	/* Offset to DCD sections */
		LgUns loadSectOffset;	/* Offset to loadable sections */
		LgUns ovlySectOffset;	/* Offset to overlay data */
		MdUns version;	/* DBOF version number */
		MdUns resvd;	/* Reserved for future use */
	} ;

/*
 *  ======== DBOF_DCDSectHdr ========
 */
	struct DBOF_DCDSectHdr {
		LgUns size;	/* Sect size (target MAUs) */
		Char name[DBOF_DCDSECTNAMELEN];	/* DCD section name */
	} ;

/*
 *  ======== DBOF_OvlySectHdr ========
 */
	struct DBOF_OvlySectHdr {
		MdUns nameLen;	/* Length of section name */
		MdUns numCreateSects;	/* # of sects loaded for create phase */
		MdUns numDeleteSects;	/* # of sects loaded for delete phase */
		MdUns numExecuteSects; /* # of sects loaded for execute phase */

		/*
		 *  Number of sections where load/unload phase is not specified.
		 *  These sections will be loaded when create phase sects are
		 *  loaded, and unloaded when the delete phase is unloaded.
		 */
		MdUns numOtherSects;
		MdUns resvd;	/* Reserved for future use */
	};

/*
 *  ======== DBOF_OvlySectData ========
 */
	struct DBOF_OvlySectData {
		LgUns loadAddr;	/* Section load address */
		LgUns runAddr;	/* Section run address */
		LgUns size;	/* Section size (target MAUs) */
		MdUns page;	/* Memory page number */
		MdUns resvd;	/* Reserved */
	} ;

/*
 *  ======== DBOF_SectHdr ========
 */
	struct DBOF_SectHdr {
		LgUns addr;	/* Section address */
		LgUns size;	/* Section size (target MAUs) */
		MdUns page;	/* Page number */
		MdUns resvd;	/* Reserved for future use */
	} ;

/*
 *  ======== DBOF_SymbolHdr ========
 */
	struct DBOF_SymbolHdr {
		LgUns value;	/* Symbol value */
		MdUns nameLen;	/* Length of symbol name */
		MdUns resvd;	/* Reserved for future use */
	} ;

#ifdef __cplusplus
}
#endif
#endif				/* DBOF_ */

