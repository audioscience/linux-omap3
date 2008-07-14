/*
 * dspbridge/src/gen/gh.c
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
 *  ======== gh.c ========
 */

#include <std.h>

#include <gs.h>

#include <gh.h>

/*typedef struct Elem Elem;*/
struct Elem {
	struct Elem *next;
	Byte data[1];
};

/*typedef Elem *ElemPtr;*/

struct GH_THashTab {
	MdUns maxBucket;
	MdUns valSize;
	struct Elem **buckets;
	 MdUns(*hash) (Ptr, MdUns);
	 Bool(*match) (Ptr, Ptr);
	 Void(*delete) (Ptr);
};

static Void nop(Ptr p);
static Int curInit;
static Void myfree(Ptr ptr, Int size);

/*
 *  ======== GH_create ========
 */

struct GH_THashTab *GH_create(MdUns maxBucket, MdUns valSize,
			      MdUns(*hash)(Ptr, MdUns), Bool(*match)(Ptr, Ptr),
			      Void(*delete)(Ptr))
{
	struct GH_THashTab *hashTab;
	MdUns i;
	hashTab = (struct GH_THashTab *)GS_alloc(sizeof(struct GH_THashTab));
	if (hashTab == NULL)
		return NULL;
	hashTab->maxBucket = maxBucket;
	hashTab->valSize = valSize;
	hashTab->hash = hash;
	hashTab->match = match;
	hashTab->delete = delete == NULL ? nop : delete;

	hashTab->buckets = (struct Elem **)
			   GS_alloc(sizeof(struct Elem *) * maxBucket);
	if (hashTab->buckets == NULL) {
		GH_delete(hashTab);
		return NULL;
	}

	for (i = 0; i < maxBucket; i++)
		hashTab->buckets[i] = NULL;

	return hashTab;
}

/*
 *  ======== GH_delete ========
 */
Void GH_delete(struct GH_THashTab *hashTab)
{
	struct Elem *elem, *next;
	MdUns i;

	if (hashTab != NULL) {
		if (hashTab->buckets != NULL) {
			for (i = 0; i < hashTab->maxBucket; i++) {
				for (elem = hashTab->buckets[i]; elem != NULL;
				    elem = next) {
					next = elem->next;
					(*hashTab->delete) (elem->data);
					myfree(elem, sizeof(struct Elem) - 1 +
					      hashTab->valSize);
				}
			}

			myfree(hashTab->buckets, sizeof(struct Elem *)
			      * hashTab->maxBucket);
		}

		myfree(hashTab, sizeof(struct GH_THashTab));
	}
}

/*
 *  ======== GH_exit ========
 */

Void GH_exit(Void)
{
	if (curInit-- == 1)
		GS_exit();

}

/*
 *  ======== GH_find ========
 */

Ptr GH_find(struct GH_THashTab *hashTab, Ptr key)
{
	struct Elem *elem;

	elem = hashTab->buckets[(*hashTab->hash)(key, hashTab->maxBucket)];

	for (; elem; elem = elem->next) {
		if ((*hashTab->match)(key, elem->data))
			return elem->data;
	}

	return NULL;
}

/*
 *  ======== GH_init ========
 */

Void GH_init(Void)
{
	if (curInit++ == 0)
		GS_init();
}

/*
 *  ======== GH_insert ========
 */

Ptr GH_insert(struct GH_THashTab *hashTab, Ptr key, Ptr value)
{
	struct Elem *elem;
	MdUns i;
	Char *src, *dst;

	elem = (struct Elem *)GS_alloc(sizeof(struct Elem) - 1 +
		hashTab->valSize);
	if (elem != NULL) {

		dst = (Char *)elem->data;
		src = (Char *)value;
		for (i = 0; i < hashTab->valSize; i++)
			*dst++ = *src++;

		i = (*hashTab->hash)(key, hashTab->maxBucket);
		elem->next = hashTab->buckets[i];
		hashTab->buckets[i] = elem;

		return elem->data;
	}

	return NULL;
}

/*
 *  ======== nop ========
 */
/* ARGSUSED */
static Void nop(Ptr p)
{
	p = p;			/* stifle compiler warning */
}

/*
 *  ======== myfree ========
 */
static Void
myfree(Ptr ptr, Int size)
{
	GS_free(ptr);
}
