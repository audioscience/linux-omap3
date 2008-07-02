/*
 * dspbridge/src/pmgr/linux/dbll/dbll.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software;  you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/*
 *  ======== dbll.c ========
 *
 *! Revision History
 *! ================
 *! 25-Apr-2030 map:    Fixed symbol redefinition bug + unload and return error
 *! 08-Apr-2003 map: 	Consolidated DBL with DBLL loader name
 *! 24-Mar-2003 map:    Updated findSymbol to support dllview update
 *! 23-Jan-2003 map:    Updated rmmAlloc to support memory granularity
 *! 21-Nov-2002 map:    Combine fopen and DLOAD_module_open to increase
 *!         performance on start.
 *! 04-Oct-2002 map:    Integrated new TIP dynamic loader w/ DOF api.
 *! 27-Sep-2002 map:    Changed handle passed to RemoteFree, instead of
 *!         RMM_free;  added GT_trace to rmmDealloc
 *! 20-Sep-2002 map:    Updated from Code Review
 *! 08-Aug-2002 jeh:    Updated to support overlays.
 *! 25-Jun-2002 jeh:    Pass RMM_Addr object to alloc function in rmmAlloc().
 *! 20-Mar-2002 jeh:    Created.
 */

/*  ----------------------------------- Host OS */
#include <host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <std.h>
#include <dbdefs.h>
#include <errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <gt.h>
#include <dbc.h>
#include <gh.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <csl.h>
#include <mem.h>

/* Dynamic loader library interface */
#include <dynamic_loader.h>
#include <getsection.h>

/*  ----------------------------------- This */
#include <dbll.h>
#include <rmm.h>

#define DBLL_TARGSIGNATURE      0x544c4c44	/* "TLLD" */
#define DBLL_LIBSIGNATURE       0x4c4c4c44	/* "LLLD" */

/* Number of buckets for symbol hash table */
#define MAXBUCKETS 211

/* Max buffer length */
#define MAXEXPR 128

#ifndef UINT32_C
#define UINT32_C(zzz) ((uint32_t)zzz)
#endif
#define DOFF_ALIGN(x) (((x) + 3) & ~UINT32_C(3))

/*
 *  ======== struct DBLL_TarObj* ========
 *  A target may have one or more libraries of symbols/code/data loaded
 *  onto it, where a library is simply the symbols/code/data contained
 *  in a DOFF file.
 */
/*
 *  ======== DBLL_TarObj ========
 */
struct DBLL_TarObj {
	LgUns dwSignature; 	/* For object validation */
	struct DBLL_Attrs attrs;
	struct DBLL_LibraryObj *head; 	/* List of all opened libraries */
} ;

/*
 *  The following 4 typedefs are "super classes" of the dynamic loader
 *  library types used in dynamic loader functions (dynamic_loader.h).
 */
/*
 *  ======== DBLLStream ========
 *  Contains Dynamic_Loader_Stream
 */
struct DBLLStream {
	struct Dynamic_Loader_Stream dlStream;
	struct DBLL_LibraryObj *lib;
} ;

/*
 *  ======== DBLLSymbol ========
 */
struct DBLLSymbol {
	struct Dynamic_Loader_Sym dlSymbol;
	struct DBLL_LibraryObj *lib;
} ;

/*
 *  ======== DBLLAlloc ========
 */
 struct DBLLAlloc {
	struct Dynamic_Loader_Allocate dlAlloc;
	struct DBLL_LibraryObj *lib;
} ;

/*
 *  ======== DBLLInit ========
 */
struct DBLLInit {
	struct Dynamic_Loader_Initialize dlInit;
	struct DBLL_LibraryObj *lib;
};

/*
 *  ======== DBLL_Library ========
 *  A library handle is returned by DBLL_Open() and is passed to DBLL_load()
 *  to load symbols/code/data, and to DBLL_unload(), to remove the
 *  symbols/code/data loaded by DBLL_load().
 */

/*
 *  ======== DBLL_LibraryObj ========
 */
 struct DBLL_LibraryObj {
	LgUns dwSignature; 	/* For object validation */
	struct DBLL_LibraryObj *next; 	/* Next library in target's list */
	struct DBLL_LibraryObj *prev; 	/* Previous in the list */
	struct DBLL_TarObj *pTarget; 	/* target for this library */

	/* Objects needed by dynamic loader */
	struct DBLLStream stream;
	struct DBLLSymbol symbol;
	struct DBLLAlloc allocate;
	struct DBLLInit init;
	DLOAD_mhandle mHandle;

	String fileName; 	/* COFF file name */
	Void *fp; 		/* Opaque file handle */
	LgUns entry; 		/* Entry point */
	DLOAD_mhandle desc; 	/* desc of DOFF file loaded */
	Uns openRef; 		/* Number of times opened */
	Uns loadRef; 		/* Number of times loaded */
	struct GH_THashTab *symTab; 	/* Hash table of symbols */
	ULONG ulPos;
} ;

/*
 *  ======== Symbol ========
 */
struct Symbol {
	struct DBLL_Symbol value;
	String name;
} ;
extern BOOL bSymbolsReloaded;

static Void dofClose(struct DBLL_LibraryObj *zlLib);
static DSP_STATUS dofOpen(struct DBLL_LibraryObj *zlLib);
static INT NoOp(struct Dynamic_Loader_Initialize *thisptr, void *bufr,
		LDR_ADDR locn, struct LDR_SECTION_INFO *info, unsigned bytsiz);

/*
 *  Functions called by dynamic loader
 *
 */
/* Dynamic_Loader_Stream */
static int readBuffer(struct Dynamic_Loader_Stream *this, void *buffer,
		     unsigned bufsize);
static int setFilePosn(struct Dynamic_Loader_Stream *this, unsigned int pos);
/* Dynamic_Loader_Sym */
static struct dynload_symbol *findSymbol(struct Dynamic_Loader_Sym *this,
					const char *name);
static struct dynload_symbol *addToSymbolTable(struct Dynamic_Loader_Sym *this,
					      const char *name,
					      unsigned moduleId);
static struct dynload_symbol *findInSymbolTable(struct Dynamic_Loader_Sym *this,
						const char *name,
						unsigned moduleid);
static void purgeSymbolTable(struct Dynamic_Loader_Sym *this,
			    unsigned moduleId);
static void *allocate(struct Dynamic_Loader_Sym *this, unsigned memsize);
static void deallocate(struct Dynamic_Loader_Sym *this, void *memPtr);
static void errorReport(struct Dynamic_Loader_Sym *this, const char *errstr,
			va_list args);
/* Dynamic_Loader_Allocate */
static int rmmAlloc(struct Dynamic_Loader_Allocate *this,
		   struct LDR_SECTION_INFO *info, unsigned align);
static void rmmDealloc(struct Dynamic_Loader_Allocate *this,
		      struct LDR_SECTION_INFO *info);

/* Dynamic_Loader_Initialize */
static int connect(struct Dynamic_Loader_Initialize *this);
static int readMem(struct Dynamic_Loader_Initialize *this, void *buf,
		  LDR_ADDR addr, struct LDR_SECTION_INFO *info,
		  unsigned nbytes);
static int writeMem(struct Dynamic_Loader_Initialize *this, void *buf,
		   LDR_ADDR addr, struct LDR_SECTION_INFO *info,
		   unsigned nbytes);
static int fillMem(struct Dynamic_Loader_Initialize *this, LDR_ADDR addr,
		   struct LDR_SECTION_INFO *info, unsigned nbytes,
		   unsigned val);
static int execute(struct Dynamic_Loader_Initialize *this, LDR_ADDR start);
static void release(struct Dynamic_Loader_Initialize *this);

/* symbol table hash functions */
static MdUns nameHash(Ptr name, MdUns maxBucket);
static Bool nameMatch(Ptr name, Ptr sp);
static Void symDelete(Ptr sp);

#if GT_TRACE
static struct GT_Mask DBLL_debugMask = { 0, 0 };     /* GT trace variable */
#endif

static LgUns cRefs = 0; 		/* module reference count */

/* Symbol Redefinition */
static Bool bRedefinedSymbol = FALSE;
static Bool bGblSearch = TRUE;

/*
 *  ======== DBLL_close ========
 */
Void DBLL_close(struct DBLL_LibraryObj *zlLib)
{
	struct DBLL_TarObj *zlTarget;

	DBC_Require(cRefs > 0);
	DBC_Require(MEM_IsValidHandle(zlLib, DBLL_LIBSIGNATURE));
	DBC_Require(zlLib->openRef > 0);
	zlTarget = zlLib->pTarget;
	GT_1trace(DBLL_debugMask, GT_ENTER, "DBLL_close: lib: 0x%x\n", zlLib);
	zlLib->openRef--;
	if (zlLib->openRef == 0) {
		/* Remove library from list */
		if (zlTarget->head == zlLib)
			zlTarget->head = zlLib->next;

		if (zlLib->prev)
			(zlLib->prev)->next = zlLib->next;

		if (zlLib->next)
			(zlLib->next)->prev = zlLib->prev;

		/* Free DOF resources */
		dofClose(zlLib);
		if (zlLib->fileName)
			MEM_Free(zlLib->fileName);

		/* remove symbols from symbol table */
		if (zlLib->symTab)
			GH_delete(zlLib->symTab);

		/* remove the library object itself */
		MEM_FreeObject(zlLib);
		zlLib = NULL;
	}
}

/*
 *  ======== DBLL_create ========
 */
DSP_STATUS DBLL_create(struct DBLL_TarObj **pTarget, struct DBLL_Attrs *pAttrs)
{
	struct DBLL_TarObj *pzlTarget;
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	DBC_Require(pAttrs != NULL);
	DBC_Require(pTarget != NULL);

	GT_2trace(DBLL_debugMask, GT_ENTER,
		  "DBLL_create: pTarget: 0x%x pAttrs: "
		  "0x%x\n", pTarget, pAttrs);
	/* Allocate DBL target object */
	MEM_AllocObject(pzlTarget, struct DBLL_TarObj, DBLL_TARGSIGNATURE);
	if (pTarget != NULL) {
		if (pzlTarget == NULL) {
			GT_0trace(DBLL_debugMask, GT_6CLASS,
				 "DBLL_create: Memory allocation"
				 " failed\n");
			*pTarget = NULL;
			status = DSP_EMEMORY;
		} else {
			pzlTarget->attrs = *pAttrs;
			*pTarget = (struct DBLL_TarObj *)pzlTarget;
		}
		DBC_Ensure((DSP_SUCCEEDED(status) &&
			  MEM_IsValidHandle(((struct DBLL_TarObj *)(*pTarget)),
			  DBLL_TARGSIGNATURE)) || (DSP_FAILED(status) &&
			  *pTarget == NULL));
	}

	return (status);
}

/*
 *  ======== DBLL_delete ========
 */
Void DBLL_delete(struct DBLL_TarObj *target)
{
	struct DBLL_TarObj *zlTarget = (struct DBLL_TarObj *)target;

	DBC_Require(cRefs > 0);
	DBC_Require(MEM_IsValidHandle(zlTarget, DBLL_TARGSIGNATURE));

	GT_1trace(DBLL_debugMask, GT_ENTER, "DBLL_delete: target: 0x%x\n",
		 target);

	if (zlTarget != NULL)
		MEM_FreeObject(zlTarget);

}

/*
 *  ======== DBLL_exit ========
 *  Discontinue usage of DBL module.
 */
Void DBLL_exit()
{
	DBC_Require(cRefs > 0);

	cRefs--;

	GT_1trace(DBLL_debugMask, GT_5CLASS, "DBLL_exit() ref count: 0x%x\n",
		  cRefs);

	if (cRefs == 0) {
		MEM_Exit();
		CSL_Exit();
		GH_exit();
#if GT_TRACE
		DBLL_debugMask.flags = 0;
#endif
	}

	DBC_Ensure(cRefs >= 0);
}

/*
 *  ======== DBLL_getAddr ========
 *  Get address of name in the specified library.
 */
Bool DBLL_getAddr(struct DBLL_LibraryObj *zlLib, String name,
		  struct DBLL_Symbol **ppSym)
{
	struct Symbol *sym;
	Bool status = FALSE;

	DBC_Require(cRefs > 0);
	DBC_Require(MEM_IsValidHandle(zlLib, DBLL_LIBSIGNATURE));
	DBC_Require(name != NULL);
	DBC_Require(ppSym != NULL);
	DBC_Require(zlLib->symTab != NULL);

	GT_3trace(DBLL_debugMask, GT_ENTER,
		 "DBLL_getAddr: lib: 0x%x name: %s pAddr:"
		 " 0x%x\n", zlLib, name, ppSym);
	sym = (struct Symbol *)GH_find(zlLib->symTab, name);
	if (sym != NULL) {
		*ppSym = &sym->value;
		status = TRUE;
	}
	return (status);
}

/*
 *  ======== DBLL_getAttrs ========
 *  Retrieve the attributes of the target.
 */
Void DBLL_getAttrs(struct DBLL_TarObj *target, struct DBLL_Attrs *pAttrs)
{
	struct DBLL_TarObj *zlTarget = (struct DBLL_TarObj *)target;

	DBC_Require(cRefs > 0);
	DBC_Require(MEM_IsValidHandle(zlTarget, DBLL_TARGSIGNATURE));
	DBC_Require(pAttrs != NULL);

	if ((pAttrs != NULL) && (zlTarget != NULL))
		*pAttrs = zlTarget->attrs;

}

/*
 *  ======== DBLL_getCAddr ========
 *  Get address of a "C" name in the specified library.
 */
Bool DBLL_getCAddr(struct DBLL_LibraryObj *zlLib, String name,
		   struct DBLL_Symbol **ppSym)
{
	struct Symbol *sym;
	Char cname[MAXEXPR + 1];
	Bool status = FALSE;

	DBC_Require(cRefs > 0);
	DBC_Require(MEM_IsValidHandle(zlLib, DBLL_LIBSIGNATURE));
	DBC_Require(ppSym != NULL);
	DBC_Require(zlLib->symTab != NULL);
	DBC_Require(name != NULL);

	cname[0] = '_';

	CSL_Strcpyn(cname + 1, name, sizeof(cname) - 2);
	cname[MAXEXPR] = '\0'; 	/* insure '\0' string termination */

	/* Check for C name, if not found */
	sym = (struct Symbol *)GH_find(zlLib->symTab, cname);

	if (sym != NULL) {
		*ppSym = &sym->value;
		status = TRUE;
	}

	return (status);
}

/*
 *  ======== DBLL_getSect ========
 *  Get the base address and size (in bytes) of a COFF section.
 */
DSP_STATUS DBLL_getSect(struct DBLL_LibraryObj *lib, String name, LgUns *pAddr,
			LgUns *pSize)
{
	Uns uByteSize;
	Bool fOpenedDoff = FALSE;
	const struct LDR_SECTION_INFO *sect = NULL;
	struct DBLL_LibraryObj *zlLib = (struct DBLL_LibraryObj *)lib;
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	DBC_Require(name != NULL);
	DBC_Require(pAddr != NULL);
	DBC_Require(pSize != NULL);
	DBC_Require(MEM_IsValidHandle(zlLib, DBLL_LIBSIGNATURE));

	GT_4trace(DBLL_debugMask, GT_ENTER,
		 "DBLL_getSect: lib: 0x%x name: %s pAddr:"
		 " 0x%x pSize: 0x%x\n", lib, name, pAddr, pSize);
	/* If DOFF file is not open, we open it. */
	if (zlLib != NULL) {
		if (zlLib->fp == NULL) {
			status = dofOpen(zlLib);
			if (DSP_SUCCEEDED(status))
				fOpenedDoff = TRUE;

		} else {
			(*(zlLib->pTarget->attrs.fseek))(zlLib->fp,
			 zlLib->ulPos, SEEK_SET);
		}
	}
	if (DSP_SUCCEEDED(status)) {
		uByteSize = 1;
		if (DLOAD_GetSectionInfo(zlLib->desc, name, &sect)) {
			*pAddr = sect->load_addr;
			*pSize = sect->size * uByteSize;
			/* Make sure size is even for good swap */
			if (*pSize % 2)
				(*pSize)++;

			/* Align size */
			*pSize = DOFF_ALIGN(*pSize);
		} else {
			status = DSP_ENOSECT;
		}
	}
	if (fOpenedDoff) {
		dofClose(zlLib);
		fOpenedDoff = FALSE;
	}

	return (status);
}

/*
 *  ======== DBLL_init ========
 */
Bool DBLL_init(Void)
{
	Bool retVal = TRUE;

	DBC_Require(cRefs >= 0);

	if (cRefs == 0) {
		DBC_Assert(!DBLL_debugMask.flags);
		GT_create(&DBLL_debugMask, "DL"); 	/* "DL" for dbDL */
		GH_init();
		CSL_Init();
		retVal = MEM_Init();
		if (!retVal)
			MEM_Exit();

	}

	if (retVal)
		cRefs++;


	GT_1trace(DBLL_debugMask, GT_5CLASS, "DBLL_init(), ref count:  0x%x\n",
		 cRefs);

	DBC_Ensure((retVal && (cRefs > 0)) || (!retVal && (cRefs >= 0)));

	return (retVal);
}

/*
 *  ======== DBLL_load ========
 */
DSP_STATUS DBLL_load(struct DBLL_LibraryObj *lib, DBLL_Flags flags,
		     struct DBLL_Attrs *attrs, LgUns *pEntry)
{
	struct DBLL_LibraryObj *zlLib = (struct DBLL_LibraryObj *)lib;
	struct DBLL_TarObj *dbzl;
	Bool gotSymbols = TRUE;
	Int err;
	DSP_STATUS status = DSP_SOK;
	BOOL fOpenedDoff = FALSE;
	DBC_Require(cRefs > 0);
	DBC_Require(MEM_IsValidHandle(zlLib, DBLL_LIBSIGNATURE));
	DBC_Require(pEntry != NULL);
	DBC_Require(attrs != NULL);

	GT_4trace(DBLL_debugMask, GT_ENTER,
		 "DBLL_load: lib: 0x%x flags: 0x%x pEntry:"
		 " 0x%x\n", lib, flags, attrs, pEntry);
	/*
	 *  Load if not already loaded.
	 */
	if (zlLib->loadRef == 0 || !(flags & DBLL_DYNAMIC)) {
		dbzl = zlLib->pTarget;
		dbzl->attrs = *attrs;
		/* Create a hash table for symbols if not already created */
		if (zlLib->symTab == NULL) {
			gotSymbols = FALSE;
			zlLib->symTab = GH_create(MAXBUCKETS,
						 sizeof(struct Symbol),
						 nameHash,
						 nameMatch, symDelete);
			if (zlLib->symTab == NULL)
				status = DSP_EMEMORY;

		}
		/*
		 *  Set up objects needed by the dynamic loader
		 */
		/* Stream */
		zlLib->stream.dlStream.read_buffer = readBuffer;
		zlLib->stream.dlStream.set_file_posn = setFilePosn;
		zlLib->stream.lib = zlLib;
		/* Symbol */
		zlLib->symbol.dlSymbol.Find_Matching_Symbol = findSymbol;
		if (gotSymbols) {
			zlLib->symbol.dlSymbol.Add_To_Symbol_Table =
							findInSymbolTable;
		} else {
			zlLib->symbol.dlSymbol.Add_To_Symbol_Table =
							addToSymbolTable;
		}
		zlLib->symbol.dlSymbol.Purge_Symbol_Table = purgeSymbolTable;
		zlLib->symbol.dlSymbol.Allocate = allocate;
		zlLib->symbol.dlSymbol.Deallocate = deallocate;
		zlLib->symbol.dlSymbol.Error_Report = errorReport;
		zlLib->symbol.lib = zlLib;
		/* Allocate */
		zlLib->allocate.dlAlloc.Allocate = rmmAlloc;
		zlLib->allocate.dlAlloc.Deallocate = rmmDealloc;
		zlLib->allocate.lib = zlLib;
		/* Init */
		zlLib->init.dlInit.connect = connect;
		zlLib->init.dlInit.readmem = readMem;
		zlLib->init.dlInit.writemem = writeMem;
		zlLib->init.dlInit.fillmem = fillMem;
		zlLib->init.dlInit.execute = execute;
		zlLib->init.dlInit.release = release;
		zlLib->init.lib = zlLib;
		/* If COFF file is not open, we open it. */
		if (zlLib->fp == NULL) {
			status = dofOpen(zlLib);
			if (DSP_SUCCEEDED(status))
				fOpenedDoff = TRUE;

		}
		if (DSP_SUCCEEDED(status)) {
			zlLib->ulPos = (*(zlLib->pTarget->attrs.ftell))
					(zlLib->fp);
			/* Reset file cursor */
			(*(zlLib->pTarget->attrs.fseek))(zlLib->fp, (long)0,
				 SEEK_SET);
			bSymbolsReloaded = TRUE;
			/* The 5th argument, DLOAD_INITBSS, tells the DLL
			 * module to zero-init all BSS sections.  In general,
			 * this is not necessary and also increases load time.
			 * We may want to make this configurable by the user */
			err = Dynamic_Load_Module(&zlLib->stream.dlStream,
			      &zlLib->symbol.dlSymbol, &zlLib->allocate.dlAlloc,
			      &zlLib->init.dlInit, DLOAD_INITBSS,
			      &zlLib->mHandle);

			if (err != 0) {
				GT_1trace(DBLL_debugMask, GT_6CLASS,
					 "DBLL_load: "
					 "Dynamic_Load_Module failed: 0x%lx\n",
					 err);
				status = DSP_EDYNLOAD;
			} else if (bRedefinedSymbol) {
				zlLib->loadRef++;
				DBLL_unload(zlLib, (struct DBLL_Attrs *) attrs);
				bRedefinedSymbol = FALSE;
				status = DSP_EDYNLOAD;
			} else {
				*pEntry = zlLib->entry;
			}
		}
	}
	if (DSP_SUCCEEDED(status))
		zlLib->loadRef++;

	/* Clean up DOFF resources */
	if (fOpenedDoff)
		dofClose(zlLib);

	DBC_Ensure(DSP_FAILED(status) || zlLib->loadRef > 0);
	return (status);
}

/*
 *  ======== DBLL_loadSect ========
 *  Not supported for COFF.
 */
DSP_STATUS DBLL_loadSect(struct DBLL_LibraryObj *zlLib, String sectName,
			struct DBLL_Attrs *attrs)
{
	DBC_Require(MEM_IsValidHandle(zlLib, DBLL_LIBSIGNATURE));

	return (DSP_ENOTIMPL);
}

/*
 *  ======== DBLL_open ========
 */
DSP_STATUS DBLL_open(struct DBLL_TarObj *target, String file, DBLL_Flags flags,
		    struct DBLL_LibraryObj **pLib)
{
	struct DBLL_TarObj *zlTarget = (struct DBLL_TarObj *)target;
	struct DBLL_LibraryObj *zlLib = NULL;
	Int err;
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	DBC_Require(MEM_IsValidHandle(zlTarget, DBLL_TARGSIGNATURE));
	DBC_Require(zlTarget->attrs.fopen != NULL);
	DBC_Require(file != NULL);
	DBC_Require(pLib != NULL);

	GT_3trace(DBLL_debugMask, GT_ENTER,
		 "DBLL_open: target: 0x%x file: %s pLib:"
		 " 0x%x\n", target, file, pLib);
	zlLib = zlTarget->head;
	while (zlLib != NULL) {
		if (CSL_Strcmp(zlLib->fileName, file) == 0) {
			/* Library is already opened */
			zlLib->openRef++;
			break;
		}
		zlLib = zlLib->next;
	}
	if (zlLib == NULL) {
		/* Allocate DBL library object */
		MEM_AllocObject(zlLib, struct DBLL_LibraryObj,
				DBLL_LIBSIGNATURE);
		if (zlLib == NULL) {
			GT_0trace(DBLL_debugMask, GT_6CLASS,
				 "DBLL_open: Memory allocation failed\n");
			status = DSP_EMEMORY;
		} else {
			zlLib->ulPos = 0;
			/* Increment ref count to allow close on failure
			 * later on */
			zlLib->openRef++;
			zlLib->pTarget = zlTarget;
			/* Keep a copy of the file name */
			zlLib->fileName = MEM_Calloc(CSL_Strlen(file) + 1,
							MEM_PAGED);
			if (zlLib->fileName == NULL) {
				GT_0trace(DBLL_debugMask, GT_6CLASS,
					 "DBLL_open: Memory "
					 "allocation failed\n");
				status = DSP_EMEMORY;
			} else {
				CSL_Strcpyn(zlLib->fileName, file,
					   CSL_Strlen(file) + 1);
			}
			zlLib->symTab = NULL;
		}
	}
	/*
	 *  Set up objects needed by the dynamic loader
	 */
	if (!DSP_SUCCEEDED(status))
		goto func_cont;

	/* Stream */
	zlLib->stream.dlStream.read_buffer = readBuffer;
	zlLib->stream.dlStream.set_file_posn = setFilePosn;
	zlLib->stream.lib = zlLib;
	/* Symbol */
	zlLib->symbol.dlSymbol.Add_To_Symbol_Table = addToSymbolTable;
	zlLib->symbol.dlSymbol.Find_Matching_Symbol = findSymbol;
	zlLib->symbol.dlSymbol.Purge_Symbol_Table = purgeSymbolTable;
	zlLib->symbol.dlSymbol.Allocate = allocate;
	zlLib->symbol.dlSymbol.Deallocate = deallocate;
	zlLib->symbol.dlSymbol.Error_Report = errorReport;
	zlLib->symbol.lib = zlLib;
	/* Allocate */
	zlLib->allocate.dlAlloc.Allocate = rmmAlloc;
	zlLib->allocate.dlAlloc.Deallocate = rmmDealloc;
	zlLib->allocate.lib = zlLib;
	/* Init */
	zlLib->init.dlInit.connect = connect;
	zlLib->init.dlInit.readmem = readMem;
	zlLib->init.dlInit.writemem = writeMem;
	zlLib->init.dlInit.fillmem = fillMem;
	zlLib->init.dlInit.execute = execute;
	zlLib->init.dlInit.release = release;
	zlLib->init.lib = zlLib;
	if (DSP_SUCCEEDED(status) && zlLib->fp == NULL)
		status = dofOpen(zlLib);

	zlLib->ulPos = (*(zlLib->pTarget->attrs.ftell)) (zlLib->fp);
	(*(zlLib->pTarget->attrs.fseek))(zlLib->fp, (long) 0, SEEK_SET);
	/* Create a hash table for symbols if flag is set */
	if (zlLib->symTab != NULL || !(flags & DBLL_SYMB))
		goto func_cont;

	zlLib->symTab = GH_create(MAXBUCKETS, sizeof(struct Symbol), nameHash,
				 nameMatch, symDelete);
	if (zlLib->symTab == NULL) {
		status = DSP_EMEMORY;
	} else {
		/* Do a fake load to get symbols - set write function to NoOp */
		zlLib->init.dlInit.writemem = NoOp;
#ifdef OPT_ELIMINATE_EXTRA_DLOAD
		err = Dynamic_Open_Module(&zlLib->stream.dlStream,
					&zlLib->symbol.dlSymbol,
					&zlLib->allocate.dlAlloc,
					&zlLib->init.dlInit, 0,
					&zlLib->mHandle);
#else
		err = Dynamic_Load_Module(&zlLib->stream.dlStream,
					 &zlLib->symbol.dlSymbol,
					 &zlLib->allocate.dlAlloc,
					 &zlLib->init.dlInit, 0,
					 &zlLib->mHandle);
#endif
		if (err != 0) {
			GT_1trace(DBLL_debugMask, GT_6CLASS, "DBLL_open: "
				 "Dynamic_Load_Module failed: 0x%lx\n", err);
			status = DSP_EDYNLOAD;
		} else {
			/* Now that we have the symbol table, we can unload */
			err = Dynamic_Unload_Module(zlLib->mHandle,
						   &zlLib->symbol.dlSymbol,
						   &zlLib->allocate.dlAlloc,
						   &zlLib->init.dlInit);
			if (err != 0) {
				GT_1trace(DBLL_debugMask, GT_6CLASS,
					"DBLL_open: "
					"Dynamic_Unload_Module failed: 0x%lx\n",
					err);
				status = DSP_EDYNLOAD;
			}
			zlLib->mHandle = NULL;
		}
	}
func_cont:
	if (DSP_SUCCEEDED(status)) {
		if (zlLib->openRef == 1) {
			/* First time opened - insert in list */
			if (zlTarget->head)
				(zlTarget->head)->prev = zlLib;

			zlLib->prev = NULL;
			zlLib->next = zlTarget->head;
			zlTarget->head = zlLib;
		}
		*pLib = (struct DBLL_LibraryObj *)zlLib;
	} else {
		*pLib = NULL;
		if (zlLib != NULL)
			DBLL_close((struct DBLL_LibraryObj *)zlLib);

	}
	DBC_Ensure((DSP_SUCCEEDED(status) && (zlLib->openRef > 0) &&
		  MEM_IsValidHandle(((struct DBLL_LibraryObj *)(*pLib)),
		  DBLL_LIBSIGNATURE)) || (DSP_FAILED(status) && *pLib == NULL));
	return (status);
}

/*
 *  ======== DBLL_readSect ========
 *  Get the content of a COFF section.
 */
DSP_STATUS DBLL_readSect(struct DBLL_LibraryObj *lib, String name,
			 Char *pContent, LgUns size)
{
	struct DBLL_LibraryObj *zlLib = (struct DBLL_LibraryObj *)lib;
	Bool fOpenedDoff = FALSE;
	Uns uByteSize; 		/* size of bytes */
	Uns ulSectSize; 		/* size of section */
	const struct LDR_SECTION_INFO *sect = NULL;
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	DBC_Require(MEM_IsValidHandle(zlLib, DBLL_LIBSIGNATURE));
	DBC_Require(name != NULL);
	DBC_Require(pContent != NULL);
	DBC_Require(size != 0);

	GT_4trace(DBLL_debugMask, GT_ENTER,
		 "DBLL_readSect: lib: 0x%x name: %s "
		 "pContent: 0x%x size: 0x%x\n", lib, name, pContent, size);
	/* If DOFF file is not open, we open it. */
	if (zlLib != NULL) {
		if (zlLib->fp == NULL) {
			status = dofOpen(zlLib);
			if (DSP_SUCCEEDED(status))
				fOpenedDoff = TRUE;

		} else {
			(*(zlLib->pTarget->attrs.fseek))(zlLib->fp,
				zlLib->ulPos, SEEK_SET);
		}
	}

	if (!DSP_SUCCEEDED(status))
		goto func_cont;

	uByteSize = 1;
	if (!DLOAD_GetSectionInfo(zlLib->desc, name, &sect)) {
		status = DSP_ENOSECT;
		goto func_cont;
	}
	/*
	 * Ensure the supplied buffer size is sufficient to store
	 * the section content to be read.
	 */
	ulSectSize = sect->size * uByteSize;
	/* Make sure size is even for good swap */
	if (ulSectSize % 2)
		ulSectSize++;

	/* Align size */
	ulSectSize = DOFF_ALIGN(ulSectSize);
	if (ulSectSize > size) {
		status = DSP_EFAIL;
	} else {
		if (!DLOAD_GetSection(zlLib->desc, sect, pContent))
			status = DSP_EFREAD;

	}
func_cont:
	if (fOpenedDoff) {
		dofClose(zlLib);
		fOpenedDoff = FALSE;
	}
	return (status);
}

/*
 *  ======== DBLL_setAttrs ========
 *  Set the attributes of the target.
 */
Void DBLL_setAttrs(struct DBLL_TarObj *target, struct DBLL_Attrs *pAttrs)
{
	struct DBLL_TarObj *zlTarget = (struct DBLL_TarObj *)target;
	DBC_Require(cRefs > 0);
	DBC_Require(MEM_IsValidHandle(zlTarget, DBLL_TARGSIGNATURE));
	DBC_Require(pAttrs != NULL);
	GT_2trace(DBLL_debugMask, GT_ENTER,
		 "DBLL_setAttrs: target: 0x%x pAttrs: "
		 "0x%x\n", target, pAttrs);
	if ((pAttrs != NULL) && (zlTarget != NULL))
		zlTarget->attrs = *pAttrs;

}

/*
 *  ======== DBLL_unload ========
 */
Void DBLL_unload(struct DBLL_LibraryObj *lib, struct DBLL_Attrs *attrs)
{
	struct DBLL_LibraryObj *zlLib = (struct DBLL_LibraryObj *)lib;
	Int err = 0;

	DBC_Require(cRefs > 0);
	DBC_Require(MEM_IsValidHandle(zlLib, DBLL_LIBSIGNATURE));
	DBC_Require(zlLib->loadRef > 0);
	GT_1trace(DBLL_debugMask, GT_ENTER, "DBLL_unload: lib: 0x%x\n", lib);
	zlLib->loadRef--;
	/* Unload only if reference count is 0 */
	if (zlLib->loadRef != 0)
		goto func_end;

	zlLib->pTarget->attrs = *attrs;
	if (zlLib != NULL) {
		if (zlLib->mHandle) {
			err = Dynamic_Unload_Module(zlLib->mHandle,
				&zlLib->symbol.dlSymbol,
				&zlLib->allocate.dlAlloc, &zlLib->init.dlInit);
			if (err != 0) {
				GT_1trace(DBLL_debugMask, GT_5CLASS,
					 "Dynamic_Unload_Module "
					 "failed: 0x%x\n", err);
			}
		}
		/* remove symbols from symbol table */
		if (zlLib->symTab != NULL) {
			GH_delete(zlLib->symTab);
			zlLib->symTab = NULL;
		}
		/* delete DOFF desc since it holds *lots* of host OS
		 * resources */
		dofClose(zlLib);
	}
func_end:
	DBC_Ensure(zlLib->loadRef >= 0);
}

/*
 *  ======== DBLL_unloadSect ========
 *  Not supported for COFF.
 */
DSP_STATUS DBLL_unloadSect(struct DBLL_LibraryObj *lib, String sectName,
			  struct DBLL_Attrs *attrs)
{
	struct DBLL_LibraryObj *zlLib = (struct DBLL_LibraryObj *)lib;

	DBC_Require(cRefs > 0);
	DBC_Require(MEM_IsValidHandle(zlLib, DBLL_LIBSIGNATURE));
	DBC_Require(sectName != NULL);
	GT_2trace(DBLL_debugMask, GT_ENTER,
		 "DBLL_unloadSect: lib: 0x%x sectName: "
		 "%s\n", lib, sectName);
	return (DSP_ENOTIMPL);
}

/*
 *  ======== dofClose ========
 */
static Void dofClose(struct DBLL_LibraryObj *zlLib)
{
	if (zlLib->desc) {
		DLOAD_module_close(zlLib->desc);
		zlLib->desc = NULL;
	}
	/* close file */
	if (zlLib->fp) {
		(zlLib->pTarget->attrs.fclose) (zlLib->fp);
		zlLib->fp = NULL;
	}
}

/*
 *  ======== dofOpen ========
 */
static DSP_STATUS dofOpen(struct DBLL_LibraryObj *zlLib)
{
	Void *open = *(zlLib->pTarget->attrs.fopen);
	DSP_STATUS status = DSP_SOK;

	/* First open the file for the dynamic loader, then open COF */
	zlLib->fp = (Void *)((Fxn)(open))(zlLib->fileName, "rb");

	/* Open DOFF module */
	if (zlLib->fp && zlLib->desc == NULL) {
		(*(zlLib->pTarget->attrs.fseek))(zlLib->fp, (long)0, SEEK_SET);
		zlLib->desc = DLOAD_module_open(&zlLib->stream.dlStream,
						&zlLib->symbol.dlSymbol);
		if (zlLib->desc == NULL) {
			(zlLib->pTarget->attrs.fclose)(zlLib->fp);
			zlLib->fp = NULL;
			status = DSP_EFOPEN;
		}
	} else {
		status = DSP_EFOPEN;
	}

	return (status);
}

/*
 *  ======== nameHash ========
 */
static MdUns nameHash(Ptr key, MdUns maxBucket)
{
	String name = (String) key;
	MdUns hash;

	DBC_Require(name != NULL);

	hash = 0;

	while (*name) {
		hash <<= 1;
		hash ^= *name++;
	}

	return (hash % maxBucket);
}

/*
 *  ======== nameMatch ========
 */
static Bool nameMatch(Ptr key, Ptr value)
{
	DBC_Require(key != NULL);
	DBC_Require(value != NULL);

	if ((key != NULL) && (value != NULL)) {
		if (CSL_Strcmp((String) key, ((struct Symbol *)value)->
		   name) == 0) {
			return TRUE;
		}
	}
	return FALSE;
}

/*
 *  ======== NoOp ========
 */
static int NoOp(struct Dynamic_Loader_Initialize *thisptr, void *bufr,
		LDR_ADDR locn, struct LDR_SECTION_INFO *info, unsigned bytsize)
{
	return (1);
}

/*
 *  ======== symDelete ========
 */
static Void symDelete(Ptr value)
{
	struct Symbol *sp = (struct Symbol *)value;

	MEM_Free(sp->name);
}

/*
 *  Dynamic Loader Functions
 */

/* Dynamic_Loader_Stream */
/*
 *  ======== readBuffer ========
 */
static int readBuffer(struct Dynamic_Loader_Stream *this, void *buffer,
		     unsigned bufsize)
{
	struct DBLLStream *pStream = (struct DBLLStream *)this;
	struct DBLL_LibraryObj *lib;
	int bytesRead = 0;

	DBC_Require(this != NULL);
	lib = pStream->lib;
	DBC_Require(MEM_IsValidHandle(lib, DBLL_LIBSIGNATURE));

	if (lib != NULL) {
		bytesRead = (*(lib->pTarget->attrs.fread))(buffer, 1, bufsize,
			    lib->fp);
	}
	return (bytesRead);
}

/*
 *  ======== setFilePosn ========
 */
static int setFilePosn(struct Dynamic_Loader_Stream *this, unsigned int pos)
{
	struct DBLLStream *pStream = (struct DBLLStream *)this;
	struct DBLL_LibraryObj *lib;
	int status = 0; 		/* Success */

	DBC_Require(this != NULL);
	lib = pStream->lib;
	DBC_Require(MEM_IsValidHandle(lib, DBLL_LIBSIGNATURE));

	if (lib != NULL) {
		status = (*(lib->pTarget->attrs.fseek))(lib->fp, (long)pos,
			 SEEK_SET);
	}

	return (status);
}

/* Dynamic_Loader_Sym */

/*
 *  ======== findSymbol ========
 */
static struct dynload_symbol *findSymbol(struct Dynamic_Loader_Sym *this,
					const char *name)
{
	struct DBLLSymbol *pSymbol = (struct DBLLSymbol *)this;
	struct DBLL_LibraryObj *lib;
	struct DBLL_Symbol *pSym = NULL;
	Bool status = FALSE; 	/* Symbol not found yet */

	DBC_Require(this != NULL);
	lib = pSymbol->lib;
	DBC_Require(MEM_IsValidHandle(lib, DBLL_LIBSIGNATURE));

	if (lib != NULL) {
		if (lib->pTarget->attrs.symLookup) {
		/* Check current lib + base lib + dep lib + persistent lib */
			status = (*(lib->pTarget->attrs.symLookup))
				 (lib->pTarget->attrs.symHandle,
				 lib->pTarget->attrs.symArg,
				 lib->pTarget->attrs.rmmHandle, name, &pSym);
		} else {
			/* Just check current lib for symbol */
			status = DBLL_getAddr((struct DBLL_LibraryObj *)lib,
				 (String)name, &pSym);
			if (!status) {
				status =
				   DBLL_getCAddr((struct DBLL_LibraryObj *)lib,
				   (String)name, &pSym);
			}
		}
	}

	if (!status && bGblSearch) {
		GT_1trace(DBLL_debugMask, GT_6CLASS,
			 "findSymbol: Symbol not found: %s\n", name);
	}

	DBC_Assert((status && (pSym != NULL)) || (!status && (pSym == NULL)));
	return ((struct dynload_symbol *) pSym);
}

/*
 *  ======== findInSymbolTable ========
 */
static struct dynload_symbol *findInSymbolTable(struct Dynamic_Loader_Sym *this,
						const char *name,
						unsigned moduleid)
{
	struct DBLLSymbol *pSymbol = (struct DBLLSymbol *)this;
	struct DBLL_LibraryObj *lib;
	struct Symbol *sym;

	DBC_Require(this != NULL);
	lib = pSymbol->lib;
	DBC_Require(MEM_IsValidHandle(lib, DBLL_LIBSIGNATURE));
	DBC_Require(lib->symTab != NULL);

	sym = (struct Symbol *)GH_find(lib->symTab, (char *) name);

	return ((struct dynload_symbol *)&sym->value);
}

/*
 *  ======== addToSymbolTable ========
 */
static struct dynload_symbol *addToSymbolTable(struct Dynamic_Loader_Sym *this,
					      const char *name,
					      unsigned moduleId)
{
	struct Symbol *symPtr = NULL;
	struct Symbol symbol;
	struct dynload_symbol *pSym = NULL;
	struct DBLLSymbol *pSymbol = (struct DBLLSymbol *)this;
	struct DBLL_LibraryObj *lib;
	struct dynload_symbol *retVal;

	DBC_Require(this != NULL);
	lib = pSymbol->lib;
	DBC_Require(MEM_IsValidHandle(lib, DBLL_LIBSIGNATURE));

	/* Check to see if symbol is already defined in symbol table */
	if (!(lib->pTarget->attrs.baseImage)) {
		bGblSearch = FALSE;
		pSym = findSymbol(this, name);
		bGblSearch = TRUE;
		if (pSym) {
			bRedefinedSymbol = TRUE;
			GT_1trace(DBLL_debugMask, GT_6CLASS,
				 "Symbol already defined in "
				 "symbol table: %s\n", name);
			return NULL;
		}
	}
	/* Allocate string to copy symbol name */
	symbol.name = (String)MEM_Calloc(CSL_Strlen((char *const)name) + 1,
							MEM_PAGED);
	if (symbol.name == NULL) {
		return NULL;
	}
	if (symbol.name != NULL) {
		/* Just copy name (value will be filled in by dynamic loader) */
		CSL_Strcpyn(symbol.name, (char *const)name,
			   CSL_Strlen((char *const)name) + 1);

		/* Add symbol to symbol table */
		symPtr = (struct Symbol *)GH_insert(lib->symTab, (Ptr)name,
			 (Ptr)&symbol);
		if (symPtr == NULL)
			MEM_Free(symbol.name);

	}
	if (symPtr != NULL)
		retVal = (struct dynload_symbol *)&symPtr->value;
	else
		retVal = NULL;

	return retVal;
}

/*
 *  ======== purgeSymbolTable ========
 */
static void purgeSymbolTable(struct Dynamic_Loader_Sym *this, unsigned moduleId)
{
	struct DBLLSymbol *pSymbol = (struct DBLLSymbol *)this;
	struct DBLL_LibraryObj *lib;

	DBC_Require(this != NULL);
	lib = pSymbol->lib;
	DBC_Require(MEM_IsValidHandle(lib, DBLL_LIBSIGNATURE));

	/* May not need to do anything */
}

/*
 *  ======== allocate ========
 */
static void *allocate(struct Dynamic_Loader_Sym *this, unsigned memsize)
{
	struct DBLLSymbol *pSymbol = (struct DBLLSymbol *)this;
	struct DBLL_LibraryObj *lib;
	void *buf;

	DBC_Require(this != NULL);
	lib = pSymbol->lib;
	DBC_Require(MEM_IsValidHandle(lib, DBLL_LIBSIGNATURE));

	buf = MEM_Calloc(memsize, MEM_PAGED);

	return (buf);
}

/*
 *  ======== deallocate ========
 */
static void deallocate(struct Dynamic_Loader_Sym *this, void *memPtr)
{
	struct DBLLSymbol *pSymbol = (struct DBLLSymbol *)this;
	struct DBLL_LibraryObj *lib;

	DBC_Require(this != NULL);
	lib = pSymbol->lib;
	DBC_Require(MEM_IsValidHandle(lib, DBLL_LIBSIGNATURE));

	MEM_Free(memPtr);
}

/*
 *  ======== errorReport ========
 */
static void errorReport(struct Dynamic_Loader_Sym *this, const char *errstr,
			va_list args)
{
	struct DBLLSymbol *pSymbol = (struct DBLLSymbol *)this;
	struct DBLL_LibraryObj *lib;
	CHAR tempBuf[MAXEXPR];

	DBC_Require(this != NULL);
	lib = pSymbol->lib;
	DBC_Require(MEM_IsValidHandle(lib, DBLL_LIBSIGNATURE));
	vsnprintf((CHAR *)tempBuf, MAXEXPR, (CHAR *)errstr, args);
	GT_1trace(DBLL_debugMask, GT_5CLASS, "%s\n", tempBuf);
}

/* Dynamic_Loader_Allocate */

/*
 *  ======== rmmAlloc ========
 */
static int rmmAlloc(struct Dynamic_Loader_Allocate *this,
		   struct LDR_SECTION_INFO *info, unsigned align)
{
	struct DBLLAlloc *pAlloc = (struct DBLLAlloc *)this;
	struct DBLL_LibraryObj *lib;
	DSP_STATUS status = DSP_SOK;
	Uns memType;
	struct RMM_Addr rmmAddr;
	INT retVal = TRUE;
	unsigned stype = DLOAD_SECTION_TYPE(info->type);
	CHAR *pToken = NULL;
	CHAR *szSecLastToken = NULL;
	CHAR *szLastToken = NULL;
	CHAR *szSectName = NULL;
	CHAR *pszCur;
	INT tokenLen = 0;
	INT segId = -1;
	INT req = -1;
	INT count = 0;
	ULONG allocSize = 0;

	DBC_Require(this != NULL);
	lib = pAlloc->lib;
	DBC_Require(MEM_IsValidHandle(lib, DBLL_LIBSIGNATURE));

	memType = (stype == DLOAD_TEXT) ? DBLL_CODE : (stype == DLOAD_BSS) ?
		   DBLL_BSS : DBLL_DATA;

	/* Attempt to extract the segment ID and requirement information from
	 the name of the section */
	tokenLen = CSL_Strlen((CHAR *)(info->name)) + 1;

	szSectName = MEM_Calloc(tokenLen, MEM_PAGED);
	szLastToken = MEM_Calloc(tokenLen, MEM_PAGED);
	szSecLastToken = MEM_Calloc(tokenLen, MEM_PAGED);

	if (szSectName == NULL || szSecLastToken == NULL ||
	   szLastToken == NULL) {
		status = DSP_EMEMORY;
		goto func_cont;
	}
	CSL_Strcpyn(szSectName, (CHAR *)(info->name), tokenLen);
	pToken = CSL_Strtokr(szSectName, ":", &pszCur);
	while (pToken) {
		CSL_Strcpyn(szSecLastToken, szLastToken,
			   CSL_Strlen(szLastToken) + 1);
		CSL_Strcpyn(szLastToken, pToken, CSL_Strlen(pToken) + 1);
		pToken = CSL_Strtokr(NULL, ":", &pszCur);
		count++; 	/* optimizes processing*/
	}
	/* If pToken is 0 or 1, and szSecLastToken is DYN_DARAM or DYN_SARAM,
	 or DYN_EXTERNAL, then mem granularity information is present
	 within the section name - only process if there are at least three
	 tokens within the section name (just a minor optimization)*/
	if (count >= 3)
		req = CSL_Atoi(szLastToken);

	if ((req == 0) || (req == 1)) {
		if (CSL_Strcmp(szSecLastToken, "DYN_DARAM") == 0) {
			segId = 0;
		} else {
			if (CSL_Strcmp(szSecLastToken, "DYN_SARAM") == 0) {
				segId = 1;
			} else {
				if (CSL_Strcmp(szSecLastToken,
				   "DYN_EXTERNAL") == 0) {
					segId = 2;
				}
			}
		}
		if (segId != -1) {
			GT_2trace(DBLL_debugMask, GT_5CLASS,
				 "Extracted values for memory"
				 " granularity req [%d] segId [%d]\n",
				 req, segId);
		}
	}
	MEM_Free(szSectName);
	szSectName = NULL;
	MEM_Free(szLastToken);
	szLastToken = NULL;
	MEM_Free(szSecLastToken);
	szSecLastToken = NULL;
func_cont:
	if (memType == DBLL_CODE)
		allocSize = info->size + GEM_L1P_PREFETCH_SIZE;
	else
		allocSize = info->size;
	/* TODO - ideally, we can pass the alignment requirement also
	 * from here */
	if (lib != NULL) {
		status = (lib->pTarget->attrs.alloc)(lib->pTarget->
			 attrs.rmmHandle, memType, allocSize, align,
			 (LgUns *)&rmmAddr, segId, req, FALSE);
	}
	if (DSP_FAILED(status)) {
		retVal = FALSE;
	} else {
		/* RMM gives word address. Need to convert to byte address */
		info->load_addr = rmmAddr.addr * DSPWORDSIZE;
		info->run_addr = info->load_addr;
		info->context = (Uns)rmmAddr.segid;
		GT_3trace(DBLL_debugMask, GT_5CLASS,
			 "Remote alloc: %s  base = 0x%lx len"
			 "= 0x%lx\n", info->name, info->load_addr / DSPWORDSIZE,
			 info->size / DSPWORDSIZE);
	}
	return (retVal);
}

/*
 *  ======== rmmDealloc ========
 */
static void rmmDealloc(struct Dynamic_Loader_Allocate *this,
		       struct LDR_SECTION_INFO *info)
{
	struct DBLLAlloc *pAlloc = (struct DBLLAlloc *)this;
	struct DBLL_LibraryObj *lib;
	Uns segid;
	DSP_STATUS status = DSP_SOK;
	unsigned stype = DLOAD_SECTION_TYPE(info->type);
	Uns memType;
	ULONG freeSize = 0;

	memType = (stype == DLOAD_TEXT) ? DBLL_CODE : (stype == DLOAD_BSS) ?
		  DBLL_BSS : DBLL_DATA;
	DBC_Require(this != NULL);
	lib = pAlloc->lib;
	DBC_Require(MEM_IsValidHandle(lib, DBLL_LIBSIGNATURE));
	/* segid was set by alloc function */
	segid = (Uns)info->context;
	if (memType == DBLL_CODE)
		freeSize = info->size + GEM_L1P_PREFETCH_SIZE;
	else
		freeSize = info->size;
	if (lib != NULL) {
		status = (lib->pTarget->attrs.free)(lib->pTarget->
			 attrs.symHandle, segid, info->load_addr / DSPWORDSIZE,
			 freeSize, FALSE);
	}
	if (DSP_SUCCEEDED(status)) {
		GT_2trace(DBLL_debugMask, GT_5CLASS,
			 "Remote dealloc: base = 0x%lx len ="
			 "0x%lx\n", info->load_addr / DSPWORDSIZE,
			 freeSize / DSPWORDSIZE);
	}
}

/* Dynamic_Loader_Initialize */
/*
 *  ======== connect ========
 */
static int connect(struct Dynamic_Loader_Initialize *this)
{
	return (TRUE);
}

/*
 *  ======== readMem ========
 *  This function does not need to be implemented.
 */
static int readMem(struct Dynamic_Loader_Initialize *this, void *buf,
		  LDR_ADDR addr, struct LDR_SECTION_INFO *info,
		  unsigned nbytes)
{
	struct DBLLInit *pInit = (struct DBLLInit *)this;
	struct DBLL_LibraryObj *lib;
	int bytesRead = 0;

	DBC_Require(this != NULL);
	lib = pInit->lib;
	DBC_Require(MEM_IsValidHandle(lib, DBLL_LIBSIGNATURE));
	/* Need WMD_BRD_Read function */
	return (bytesRead);
}

/*
 *  ======== writeMem ========
 */
static int writeMem(struct Dynamic_Loader_Initialize *this, void *buf,
		   LDR_ADDR addr, struct LDR_SECTION_INFO *info,
		   unsigned nBytes)
{
	struct DBLLInit *pInit = (struct DBLLInit *)this;
	struct DBLL_LibraryObj *lib;
	struct DBLL_SectInfo sectInfo;
	Uns memType;
	Bool retVal;

	DBC_Require(this != NULL);
	lib = pInit->lib;
	DBC_Require(MEM_IsValidHandle(lib, DBLL_LIBSIGNATURE));

	memType = (DLOAD_SECTION_TYPE(info->type) == DLOAD_TEXT) ? DBLL_CODE :
		  DBLL_DATA;
	if (lib != NULL) {
		retVal = (*lib->pTarget->attrs.write)(lib->pTarget->
			attrs.wHandle, addr, buf, nBytes, memType);
	}
	if (lib->pTarget->attrs.logWrite) {
		sectInfo.name = info->name;
		sectInfo.runAddr = info->run_addr;
		sectInfo.loadAddr = info->load_addr;
		sectInfo.size = info->size;
		sectInfo.type = memType;
		/* Pass the information about what we've written to
		 * another module */
		(*lib->pTarget->attrs.logWrite)(lib->pTarget->
			attrs.logWriteHandle, &sectInfo, addr, nBytes);
	}
	return (retVal);
}

/*
 *  ======== fillMem ========
 *  Fill nBytes of memory at a given address with a given value by
 *  writing from a buffer containing the given value.  Write in
 *  sets of MAXEXPR (128) bytes to avoid large stack buffer issues.
 */
static int fillMem(struct Dynamic_Loader_Initialize *this, LDR_ADDR addr,
		   struct LDR_SECTION_INFO *info, unsigned nBytes,
		   unsigned val)
{
	Bool retVal = TRUE;
	CHAR tempBuf[MAXEXPR];
	ULONG ulRemainBytes = 0;
	ULONG ulBytes = 0;
#ifdef OPT_USE_MEMSET
	CHAR *pBuf;
	struct DBLL_LibraryObj *lib;
	struct DBLLInit *pInit = (struct DBLLInit *)this;

	DBC_Require(this != NULL);
	lib = pInit->lib;
	pBuf = NULL;
	/* Pass the NULL pointer to writeMem to get the start address of Shared
	    memory. This is a trick to just get the start address, there is no
	    writing taking place with this Writemem
	*/
	if ((lib->pTarget->attrs.write) != NoOp)
		writeMem(this, &pBuf, addr, info, 0);
	if (pBuf)
		memset(pBuf, val, nBytes);
#else
	DBC_Require(this != NULL);
	ulRemainBytes = nBytes;
	/* Zero out buffer */
	memset(tempBuf, val, MAXEXPR);

	while ((ulRemainBytes > 0) && retVal) {
		ulBytes = ulRemainBytes > MAXEXPR ? MAXEXPR : ulRemainBytes;

		/* Call a function to fill memory */
		retVal = writeMem(this, tempBuf, addr, info, ulBytes);

		ulRemainBytes -= ulBytes;
		/* (BYTE *) addr += ulBytes; */
		addr = (LDR_ADDR)((BYTE *)addr + ulBytes);
	}
#endif
	return (retVal);
}

/*
 *  ======== execute ========
 */
static int execute(struct Dynamic_Loader_Initialize *this, LDR_ADDR start)
{
	struct DBLLInit *pInit = (struct DBLLInit *)this;
	struct DBLL_LibraryObj *lib;
	Bool retVal = TRUE;

	DBC_Require(this != NULL);
	lib = pInit->lib;
	DBC_Require(MEM_IsValidHandle(lib, DBLL_LIBSIGNATURE));
	/* Save entry point */
	if (lib != NULL)
		lib->entry = (ULONG)start;

	return (retVal);
}

/*
 *  ======== release ========
 */
static void release(struct Dynamic_Loader_Initialize *this)
{
}

