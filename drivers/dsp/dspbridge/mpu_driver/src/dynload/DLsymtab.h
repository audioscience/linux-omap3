/*
 * dspbridge/src/dynload/DLsymtab.h
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



#ifndef _DLSYMTAB_H
#define _DLSYMTAB_H

/*****************************************************************************
 *****************************************************************************
 *
 *                          DLSYMTAB.H
 *
 * A class used by the dynamic loader for symbol table support.
 *
 * This implementation uses a flat hash table, malloc/free, and printf
 *****************************************************************************
 *****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include "dynamic_loader.h"

#define LSYM_LOGTBLLEN 5

/* Symbol Table Entry */
struct symbol {
	struct symbol *link;	/* ptr to next symbol in the hash table */
	const char *name;	/* symbol name                          */
	unsigned versn;	/* id of the module in which this  symbol resides */
	unsigned length;	/* length of the symbol name            */
} ;

/* Symbol Table 'class' with support fields.*/
struct symtab {
	/* Symbol Table is a hash table of symbols size is based on hash fcn*/
	struct symbol *(mytable[(1 << (LSYM_LOGTBLLEN)) + 1]);
	int hash_key;	/* remember hash lookup index from last lookup  */
	unsigned last_length;	/* remember last length */
} ;

/* Customized DL Symbol Manager 'class' */
struct DL_sym_t {
	Dynamic_Loader_Sym sym;
	struct symtab a_symtab;
} ;

/*****************************************************************************
 * NOTE:    All methods take a 'thisptr' parameter that is a pointer to the
 *          environment for the DL Symbol Manager APIs.  See definition of
 *          DL_sym_t.
 *
 *****************************************************************************/

/*****************************************************************************
 * DLsym_init
 *
 * PARAMETERS :
 *  none
 *
 * EFFECT :
 *  Initialize symbol manager handlers
 *****************************************************************************/
void DLsym_init(DL_sym_t *thisptr);

#ifdef __cplusplus
}
#endif
#endif				/* _DLSYMTAB_H */
