/*
 * dspbridge/mpu_driver/inc/dblldefs.h
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
 *  ======== dblldefs.h ========
 *
 *! Revision History
 *! ================
 *! 08-Apr-2003 map	    Consolidated DBL into DBLL name
 *! 19-Mar-2002 jeh     Added DBL_Fxns type (to make it easier to switch
 *!                     between different loaders).
 *! 28-Sep-2001 jeh     Created from zl.h.
 */
#ifndef DBLLDEFS_
#define DBLLDEFS_

/*
 *  Bit masks for DBL_Flags.
 */
#define DBLL_NOLOAD   0x0	/* Don't load symbols, code, or data */
#define DBLL_SYMB     0x1	/* load symbols */
#define DBLL_CODE     0x2	/* load code */
#define DBLL_DATA     0x4	/* load data */
#define DBLL_DYNAMIC  0x8	/* dynamic load */
#define DBLL_BSS      0x20	/* Unitialized section */

#define DBLL_MAXPATHLENGTH       255

#ifndef _SIZE_T
typedef unsigned int size_t;
#define _SIZE_T
#endif

/*
 *  ======== DBLL_Target ========
 *
 */
struct DBLL_TarObj;
/*typedef struct DBLL_TarObj *DBLL_Target;*/

/*
 *  ======== DBLL_Flags ========
 *  Specifies whether to load code, data, or symbols
 */
typedef Int DBLL_Flags;

/*
 *  ======== DBLL_Library ========
 *
 */
struct DBLL_LibraryObj;
/*typedef struct DBLL_LibraryObj *DBLL_Library;*/

/*
 *  ======== DBLL_SectInfo ========
 *  For collecting info on overlay sections
 */
struct DBLL_SectInfo {
	const Char *name;	/* name of section */
	LgUns runAddr;		/* run address of section */
	LgUns loadAddr;		/* load address of section */
	LgUns size;		/* size of section (target MAUs) */
	DBLL_Flags type;	/* Code, data, or BSS */
} ;

/*
 *  ======== DBLL_Symbol ========
 *  (Needed for dynamic load library)
 */
struct DBLL_Symbol {
	LgUns value;
};

/*
 *  ======== DBLL_AllocFxn ========
 *  Allocate memory function.  Allocate or reserve (if reserved == TRUE)
 *  "size" bytes of memory from segment "space" and return the address in
 *  *dspAddr (or starting at *dspAddr if reserve == TRUE). Returns 0 on
 *  success, or an error code on failure.
 */
typedef Int(*DBLL_AllocFxn) (Void *hdl, Int space, LgUns size, Uns align,
			     LgUns *dspAddr, Int segId, Int req,
			     Bool reserved);

/*
 *  ======== DBLL_CloseFxn ========
 */
typedef Int(*DBLL_FCloseFxn) (Void *);

/*
 *  ======== DBLL_FreeFxn ========
 *  Free memory function.  Free, or unreserve (if reserved == TRUE) "size"
 *  bytes of memory from segment "space"
 */
typedef Bool(*DBLL_FreeFxn) (Void *hdl, LgUns addr, Int space, LgUns size,
			     Bool reserved);

/*
 *  ======== DBLL_FOpenFxn ========
 */
typedef Void *(*DBLL_FOpenFxn) (const Char *, const Char *);

/*
 *  ======== DBLL_LogWriteFxn ========
 *  Function to call when writing data from a section, to log the info.
 *  Can be NULL if no logging is required.
 */
typedef DSP_STATUS(*DBLL_LogWriteFxn)(Void *handle, struct DBLL_SectInfo *sect,
				       LgUns addr, LgUns nBytes);

/*
 *  ======== DBLL_ReadFxn ========
 */
typedef Int(*DBLL_ReadFxn) (Void *, size_t, size_t, Void *);

/*
 *  ======== DBLL_SeekFxn ========
 */
typedef Int(*DBLL_SeekFxn) (Void *, long, int);

/*
 *  ======== DBLL_SymLookup ========
 *  Symbol lookup function - Find the symbol name and return its value.
 *
 *  Parameters:
 *      handle          - Opaque handle
 *      pArg            - Opaque argument.
 *      name            - Name of symbol to lookup.
 *      sym             - Location to store address of symbol structure.
 *
 *  Returns:
 *      TRUE:           Success (symbol was found).
 *      FALSE:          Failed to find symbol.
 */
typedef Bool(*DBLL_SymLookup) (Void *handle, Void *pArg, Void *rmmHandle,
			       const Char *name, struct DBLL_Symbol **sym);

/*
 *  ======== DBLL_TellFxn ========
 */
typedef LgInt(*DBLL_TellFxn) (Void *);

/*
 *  ======== DBLL_WriteFxn ========
 *  Write memory function.  Write "n" HOST bytes of memory to segment "mtype"
 *  starting at address "dspAddr" from the buffer "buf".  The buffer is
 *  formatted as an array of words appropriate for the DSP.
 */
typedef Int(*DBLL_WriteFxn) (Void *hdl, LgUns dspAddr, Void *buf,
			     LgUns n, Int mtype);

/*
 *  ======== DBLL_Attrs ========
 */
struct DBLL_Attrs {
	DBLL_AllocFxn alloc;
	DBLL_FreeFxn free;
	Void *rmmHandle;	/* Handle to pass to alloc, free functions */
	DBLL_WriteFxn write;
	Void *wHandle;		/* Handle to pass to write, cinit function */
	Bool baseImage;
	DBLL_LogWriteFxn logWrite;
	Void *logWriteHandle;

	/* Symbol matching function and handle to pass to it */
	DBLL_SymLookup symLookup;
	Void *symHandle;
	Void *symArg;

	/*
	 *  These file manipulation functions should be compatible with the
	 *  "C" run time library functions of the same name.
	 */
	 Int(*fread) (Void *, size_t, size_t, Void *);
	 Int(*fseek) (Void *, long, int);
	 LgInt(*ftell) (Void *);
	 Int(*fclose) (Void *);
	Void *(*fopen) (const Char *, const Char *);
} ;

/*
 *  ======== DBLL_close ========
 *  Close library opened with DBLL_open.
 *  Parameters:
 *      lib             - Handle returned from DBLL_open().
 *  Returns:
 *  Requires:
 *      DBL initialized.
 *      Valid lib.
 *  Ensures:
 */
typedef Void(*DBLL_CloseFxn) (struct DBLL_LibraryObj *library);

/*
 *  ======== DBLL_create ========
 *  Create a target object, specifying the alloc, free, and write functions.
 *  Parameters:
 *      pTarget         - Location to store target handle on output.
 *      pAttrs          - Attributes.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EMEMORY:    Memory allocation failed.
 *  Requires:
 *      DBL initialized.
 *      pAttrs != NULL.
 *      pTarget != NULL;
 *  Ensures:
 *      Success:        *pTarget != NULL.
 *      Failure:        *pTarget == NULL.
 */
typedef DSP_STATUS(*DBLL_CreateFxn)(struct DBLL_TarObj **pTarget,
				    struct DBLL_Attrs *attrs);

/*
 *  ======== DBLL_delete ========
 *  Delete target object and free resources for any loaded libraries.
 *  Parameters:
 *      target          - Handle returned from DBLL_Create().
 *  Returns:
 *  Requires:
 *      DBL initialized.
 *      Valid target.
 *  Ensures:
 */
typedef Void(*DBLL_DeleteFxn) (struct DBLL_TarObj *target);

/*
 *  ======== DBLL_exit ========
 *  Discontinue use of DBL module.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      cRefs > 0.
 *  Ensures:
 *      cRefs >= 0.
 */
typedef Void(*DBLL_ExitFxn) (Void);

/*
 *  ======== DBLL_getAddr ========
 *  Get address of name in the specified library.
 *  Parameters:
 *      lib             - Handle returned from DBLL_open().
 *      name            - Name of symbol
 *      ppSym           - Location to store symbol address on output.
 *  Returns:
 *      TRUE:           Success.
 *      FALSE:          Symbol not found.
 *  Requires:
 *      DBL initialized.
 *      Valid library.
 *      name != NULL.
 *      ppSym != NULL.
 *  Ensures:
 */
typedef Bool(*DBLL_GetAddrFxn) (struct DBLL_LibraryObj *lib, String name,
				struct DBLL_Symbol **ppSym);

/*
 *  ======== DBLL_getAttrs ========
 *  Retrieve the attributes of the target.
 *  Parameters:
 *      target          - Handle returned from DBLL_Create().
 *      pAttrs          - Location to store attributes on output.
 *  Returns:
 *  Requires:
 *      DBL initialized.
 *      Valid target.
 *      pAttrs != NULL.
 *  Ensures:
 */
typedef Void(*DBLL_GetAttrsFxn) (struct DBLL_TarObj *target,
				 struct DBLL_Attrs *attrs);

/*
 *  ======== DBLL_getCAddr ========
 *  Get address of "C" name on the specified library.
 *  Parameters:
 *      lib             - Handle returned from DBLL_open().
 *      name            - Name of symbol
 *      ppSym           - Location to store symbol address on output.
 *  Returns:
 *      TRUE:           Success.
 *      FALSE:          Symbol not found.
 *  Requires:
 *      DBL initialized.
 *      Valid target.
 *      name != NULL.
 *      ppSym != NULL.
 *  Ensures:
 */
typedef Bool(*DBLL_GetCAddrFxn) (struct DBLL_LibraryObj *lib, String name,
				 struct DBLL_Symbol **ppSym);

/*
 *  ======== DBLL_getSect ========
 *  Get address and size of a named section.
 *  Parameters:
 *      lib             - Library handle returned from DBLL_open().
 *      name            - Name of section.
 *      pAddr           - Location to store section address on output.
 *      pSize           - Location to store section size on output.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_ENOSECT:    Section not found.
 *  Requires:
 *      DBL initialized.
 *      Valid lib.
 *      name != NULL.
 *      pAddr != NULL;
 *      pSize != NULL.
 *  Ensures:
 */
typedef DSP_STATUS(*DBLL_GetSectFxn) (struct DBLL_LibraryObj *lib, String name,
				      LgUns *addr, LgUns *size);

/*
 *  ======== DBLL_init ========
 *  Initialize DBL module.
 *  Parameters:
 *  Returns:
 *      TRUE:           Success.
 *      FALSE:          Failure.
 *  Requires:
 *      cRefs >= 0.
 *  Ensures:
 *      Success:        cRefs > 0.
 *      Failure:        cRefs >= 0.
 */
typedef Bool(*DBLL_InitFxn) (Void);

/*
 *  ======== DBLL_load ========
 *  Load library onto the target.
 *
 *  Parameters:
 *      lib             - Library handle returned from DBLL_open().
 *      flags           - Load code, data and/or symbols.
 *      attrs           - May contain alloc, free, and write function.
 *      pulEntry        - Location to store program entry on output.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EFREAD:     File read failed.
 *      DSP_EFWRITE:    Write to target failed.
 *      DSP_EDYNLOAD:   Failure in dynamic loader library.
 *  Requires:
 *      DBL initialized.
 *      Valid lib.
 *      pEntry != NULL.
 *  Ensures:
 */
typedef DSP_STATUS(*DBLL_LoadFxn) (struct DBLL_LibraryObj *lib,
				   DBLL_Flags flags,
				   struct DBLL_Attrs *attrs, LgUns *entry);

/*
 *  ======== DBLL_loadSect ========
 *  Load a named section from an library (for overlay support).
 *  Parameters:
 *      lib             - Handle returned from DBLL_open().
 *      sectName        - Name of section to load.
 *      attrs           - Contains write function and handle to pass to it.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_ENOSECT:    Section not found.
 *      DSP_EFWRITE:    Write function failed.
 *      DSP_ENOTIMPL:   Function not implemented.
 *  Requires:
 *      Valid lib.
 *      sectName != NULL.
 *      attrs != NULL.
 *      attrs->write != NULL.
 *  Ensures:
 */
typedef DSP_STATUS(*DBLL_LoadSectFxn) (struct DBLL_LibraryObj *lib,
				       Char *pszSectName,
				       struct DBLL_Attrs *attrs);

/*
 *  ======== DBLL_open ========
 *  DBLL_open() returns a library handle that can be used to load/unload
 *  the symbols/code/data via DBLL_load()/DBLL_unload().
 *  Parameters:
 *      target          - Handle returned from DBLL_create().
 *      file            - Name of file to open.
 *      flags           - If flags & DBLL_SYMB, load symbols.
 *      pLib            - Location to store library handle on output.
 *  Returns:
 *      DSP_SOK:            Success.
 *      DSP_EMEMORY:        Memory allocation failure.
 *      DSP_EFOPEN:         File open failure.
 *      DSP_EFREAD:         File read failure.
 *      DSP_ECORRUPTFILE:   Unable to determine target type.
 *  Requires:
 *      DBL initialized.
 *      Valid target.
 *      file != NULL.
 *      pLib != NULL.
 *      DBLL_Attrs fopen function non-NULL.
 *  Ensures:
 *      Success:        Valid *pLib.
 *      Failure:        *pLib == NULL.
 */
typedef DSP_STATUS(*DBLL_OpenFxn) (struct DBLL_TarObj *target, String file,
				   DBLL_Flags flags,
				   struct DBLL_LibraryObj **pLib);

/*
 *  ======== DBLL_readSect ========
 *  Read COFF section into a character buffer.
 *  Parameters:
 *      lib             - Library handle returned from DBLL_open().
 *      name            - Name of section.
 *      pBuf            - Buffer to write section contents into.
 *      size            - Buffer size
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_ENOSECT:    Named section does not exists.
 *  Requires:
 *      DBL initialized.
 *      Valid lib.
 *      name != NULL.
 *      pBuf != NULL.
 *      size != 0.
 *  Ensures:
 */
typedef DSP_STATUS(*DBLL_ReadSectFxn) (struct DBLL_LibraryObj *lib, String name,
				       String content, LgUns uContentSize);

/*
 *  ======== DBLL_setAttrs ========
 *  Set the attributes of the target.
 *  Parameters:
 *      target          - Handle returned from DBLL_create().
 *      pAttrs          - New attributes.
 *  Returns:
 *  Requires:
 *      DBL initialized.
 *      Valid target.
 *      pAttrs != NULL.
 *  Ensures:
 */
typedef Void(*DBLL_SetAttrsFxn)(struct DBLL_TarObj *target,
				struct DBLL_Attrs *attrs);

/*
 *  ======== DBLL_unload ========
 *  Unload library loaded with DBLL_load().
 *  Parameters:
 *      lib             - Handle returned from DBLL_open().
 *      attrs           - Contains free() function and handle to pass to it.
 *  Returns:
 *  Requires:
 *      DBL initialized.
 *      Valid lib.
 *  Ensures:
 */
typedef Void(*DBLL_UnloadFxn) (struct DBLL_LibraryObj *library,
			       struct DBLL_Attrs *attrs);

/*
 *  ======== DBLL_unloadSect ========
 *  Unload a named section from an library (for overlay support).
 *  Parameters:
 *      lib             - Handle returned from DBLL_open().
 *      sectName        - Name of section to load.
 *      attrs           - Contains free() function and handle to pass to it.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_ENOSECT:    Named section not found.
 *      DSP_ENOTIMPL
 *  Requires:
 *      DBL initialized.
 *      Valid lib.
 *      sectName != NULL.
 *  Ensures:
 */
typedef DSP_STATUS(*DBLL_UnloadSectFxn) (struct DBLL_LibraryObj *lib,
					 Char *pszSectName,
					 struct DBLL_Attrs *attrs);

struct DBLL_Fxns {
	DBLL_CloseFxn closeFxn;
	DBLL_CreateFxn createFxn;
	DBLL_DeleteFxn deleteFxn;
	DBLL_ExitFxn exitFxn;
	DBLL_GetAttrsFxn getAttrsFxn;
	DBLL_GetAddrFxn getAddrFxn;
	DBLL_GetCAddrFxn getCAddrFxn;
	DBLL_GetSectFxn getSectFxn;
	DBLL_InitFxn initFxn;
	DBLL_LoadFxn loadFxn;
	DBLL_LoadSectFxn loadSectFxn;
	DBLL_OpenFxn openFxn;
	DBLL_ReadSectFxn readSectFxn;
	DBLL_SetAttrsFxn setAttrsFxn;
	DBLL_UnloadFxn unloadFxn;
	DBLL_UnloadSectFxn unloadSectFxn;
} ;

#endif				/* DBLDEFS_ */
