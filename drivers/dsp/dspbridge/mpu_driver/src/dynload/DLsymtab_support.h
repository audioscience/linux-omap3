/*
 * dspbridge/src/dynload/DLsymtab_support.h
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



#ifndef _DLSYMTAB_SUPPORT_H
#define _DLSYMTAB_SUPPORT_H

/*****************************************************************************
 *****************************************************************************
 *
 *                          DLSYMTAB_SUPPORT.C
 *
 * Suuport functions used by the dynamic loader for the symbol table.
 *
 * This implementation uses a flat hash table, malloc/free, and printf
 *****************************************************************************
 *****************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

#include "dynamic_loader.h"
#include "DLsymtab.h"

typedef int Symbol_Action(struct symbol *thissym, void *arg);

void Init_Symbol_Table(symtab *a_stable, int llen);

struct symbol *Find_Matching_Symbol(symtab *a_table, const char *name);

void Add_To_Symbol_Table(symtab *a_stable, struct symbol *newsym,
				 const char *nname, unsigned versn);

void Iterate_Symbols(struct symbol **stable, Symbol_Action *action, void *arg);

int Purge_Symbol(struct symbol *thissym, void *arg);

#ifdef __cplusplus
}
#endif
#endif				/* _DLSYMTAB_SUPPORT_H */
