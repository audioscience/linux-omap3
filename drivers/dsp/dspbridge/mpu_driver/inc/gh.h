/*
 * dspbridge/inc/gh.h
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

/*typedef struct GH_THashTab *GH_HashTab;*/

#ifdef _LINT_

/* LINTLIBRARY */

/*
 *  ======== GH_create ========
 *  Create hash table with maxBucket entries in table (each entry is a list
 *  of elems that "hash" to the same entry).
 *
 *  valSize is the size of elems that are inserted into the hash table,
 *  hash() is the function that computes the index into the hash table given
 *  the elem and maxBucket, match() is a function that determines if an
 *  element in the table is the desired element being "looked up" (via
 *  GH_find()), and delete() is a callback function used to free up storage
 *  allocated for the elem by the client.
 *
 *  For example,
 *    typedef struct MyRec ...;
 *
 *    GH_create(NBUCKETS, sizeof(MyRec), hash, match, delete);
 *
 *    MdUns hash(Ptr key, MdUns nBuckets)
 *    {
 *	  return ( (MdUns)(key % nBuckets) );
 *    }
 *
 *    Bool match(Ptr key, Ptr value)
 *    {
 *	  MyRec *myPtr = value;
 *	  return (myPtr->key == key);
 *    }
 *
 *    Void delete(Ptr value)
 *    {
 *	  MyRec *myPtr = value;
 *	  free(myPtr->buf);
 *    }
 *
 *
 *  If delete == NULL then no callback is invoked.  Both hash and match must
 *  be valid non-NULL function pointers.
 */
/* ARGSUSED */
struct GH_THashTab*
GH_create(MdUns maxBucket, MdUns valSize,
	  MdUns(*hash) (Ptr, MdUns), Bool(*match) (Ptr, Ptr),
	  Void(*delete) (Ptr))
{
	return ((struct GH_THashTab *) NULL);
}

/*
 *  ======== GH_delete ========
 *  Delete the hash table and all its elems.
 */
/* ARGSUSED */
Void
GH_delete(struct GH_THashTab *hashTab)
{
}

/*
 *  ======== GH_exit ========
 */
/* ARGSUSED */
Void
GH_exit(Void)
{
}

/*
 *  ======== GH_find ========
 *  Lookup elem (identified via key) in the hash table hashTab.  Returns
 *  NULL if the lookup fails; i.e., if no element with the specified key
 *  exists in the hash table.
 *
 *  For example,
 *	if ((myPtr = (MyRec *)GH_find(hash, key)) != NULL) {
 *	    myPtr-> ...;
 *      }
 */
/* ARGSUSED */
Ptr
GH_find(struct GH_THashTab *hashTab, Ptr key)
{
	return ((Ptr) NULL);
}

/*
 *  ======== GH_init ========
 */
/* ARGSUSED */
Void
GH_init(Void)
{
}

/*
 *  ======== GH_insert ========
 *  Add elem (identified via key with value pointed to by value) to
 *  the hash table hashTab.
 *
 *  For example,
 *	MyRec rec;
 *	rec.key = key;
 *	  :
 *	GH_insert(hash, key, (Ptr)&rec)
 */
/* ARGSUSED */
Ptr
GH_insert(struct GH_THashTab *hashTab, Ptr key, Ptr value)
{
	return (NULL);
}

/*
 *  ======== GH_remove ========
 *  Remove elem (identified by key) from the hash table hashTab
 */
/* ARGSUSED */
Void
GH_remove(struct GH_THashTab *hashTab, Ptr key)
{
}

#else				/* !_LINT_ */

extern struct GH_THashTab *GH_create(MdUns maxBucket, MdUns valSize,
			    MdUns(*hash) (Ptr, MdUns), Bool(*match) (Ptr, Ptr),
			    Void(*delete) (Ptr));
extern Void GH_delete(struct GH_THashTab *hashTab);
extern Void GH_exit(Void);
extern Ptr GH_find(struct GH_THashTab *hashTab, Ptr key);
extern Void GH_init(Void);
extern Ptr GH_insert(struct GH_THashTab *hashTab, Ptr key, Ptr value);
extern Void GH_remove(struct GH_THashTab *hashTab, Ptr key);

#endif				/* _LINT_ */

#endif				/* GH_ */
