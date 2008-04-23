/*
 * dspbridge/src/gen/gb.c
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
 *  ======== gb.c ========
 *  Description: Generic bitmap operations.
 *
 *! Revision History
 *! ================
 *! 24-Feb-2003 vp  Code review updates.
 *! 17-Dec-2002 map Fixed GB_minset(), GB_empty(), and GB_full(),
 *!                 to ensure only 'len' bits are considered in the map
 *! 18-Oct-2002 sb  Ported to Linux platform.
 *! 06-Dec-2001 jeh Fixed bug in GB_minclear().
 *!
 */

/*  ----------------------------------- DSP/BIOS Bridge */
#include <std.h>

/*  ----------------------------------- This */
#include <gs.h>
#include <gb.h>

#define LGSIZE  32		/* number of bits in a LgUns */

/*typedef struct GB_TMap GB_TMap;*/
typedef GB_BitNum GB_WordNum;

struct GB_TMap {
	GB_BitNum len;
	GB_WordNum wcnt;
	LgUns *words;
};

/*
 *  ======== GB_and ========
 *  purpose:
 *      Performs AND operation on the two bit maps provided.
 */

Void GB_and(struct GB_TMap *dst, struct GB_TMap *src)
{
	GB_WordNum i;

	for (i = 0; i < dst->wcnt; i++)
		dst->words[i] &= src->words[i];

}

/*
 *  ======== GB_clear ========
 *  purpose:
 *      Clears a bit in the bit map.
 */

Void GB_clear(struct GB_TMap *map, GB_BitNum bitn)
{
	LgUns mask;

	mask = 1L << (bitn % LGSIZE);
	map->words[bitn / LGSIZE] &= ~mask;
}

/*
 *  ======== GB_create ========
 *  purpose:
 *      Creates a bit map.
 */

struct GB_TMap *GB_create(GB_BitNum len)
{
	struct GB_TMap *map;
	GB_WordNum i;

	if ((map = (struct GB_TMap *)GS_alloc(sizeof(struct GB_TMap))) !=
	   NULL) {
		map->len = len;
		map->wcnt = len / LGSIZE + 1;
		map->words = (LgUns *)GS_alloc(map->wcnt * sizeof(LgUns));
		if (map->words != NULL) {
			for (i = 0; i < map->wcnt; i++) {
				map->words[i] = 0L;
			}
		} else {
			GS_frees(map, sizeof(struct GB_TMap));
			map = NULL;
		}
	}

	return (map);
}

/*
 *  ======== GB_delete ========
 *  purpose:
 *      Frees a bit map.
 */

Void GB_delete(struct GB_TMap *map)
{
	GS_frees(map->words, map->wcnt * sizeof(LgUns));
	GS_frees(map, sizeof(struct GB_TMap));
}

/*
 *  ======== GB_empty ========
 *  purpose:
 *      Returns TRUE if the bitmap is empty.
 */

Bool GB_empty(struct GB_TMap *map)
{
	LgUns mask = 0x0;
	GB_WordNum i;

	mask = ~mask >> (LGSIZE - (map->len % LGSIZE));

	for (i = 0; i < map->wcnt; i++) {
		if (i == (map->wcnt - 1)) {
			if (map->words[i] & mask) {
				return (FALSE);
			}
		} else {
			if (map->words[i] != 0L) {
				return (FALSE);
			}
		}
	}

	return (TRUE);
}

/*
 *  ======== GB_findandset ========
 *  purpose:
 *      Finds a free bit and sets it.
 */
GB_BitNum GB_findandset(struct GB_TMap *map)
{
	GB_BitNum bitn;

	bitn = GB_minclear(map);

	if (bitn != GB_NOBITS)
		GB_set(map, bitn);

	return (bitn);
}

/*
 *  ======== GB_full ========
 *  purpose:
 *      Returns TRUE if the bit map is full.
 */

Bool GB_full(struct GB_TMap *map)
{
	LgUns mask = 0x0;
	GB_WordNum i;

	mask = ~mask >> (LGSIZE - (map->len % LGSIZE));

	for (i = 0; i < map->wcnt; i++) {
		if (i == (map->wcnt - 1)) {
			if ((map->words[i] & mask) != mask) {
				return (FALSE);
			}
		} else {
			if (~map->words[i] != 0L) {
				return (FALSE);
			}
		}
	}

	return (TRUE);
}

/*
 *  ======== GB_init ========
 *  purpose:
 *      Initializes the GB module.
 */

Void GB_init(void)
{
	static Bool curInit = FALSE;

	if (curInit)
		return;

	curInit = TRUE;

	GS_init();
}

/*
 *  ======== GB_len ========
 *  purpose:
 *      Returns the length of the bit map.
 */

GB_BitNum GB_len(struct GB_TMap *map)
{
	return (map->len);
}

/*
 *  ======== GB_minclear ========
 *  purpose:
 *      returns the location of the first unset bit in the bit map.
 */
GB_BitNum GB_minclear(struct GB_TMap *map)
{
	GB_BitNum bitAcc = 0;
	GB_WordNum i;
	GB_BitNum bit;
	LgUns *word;

	for (word = map->words, i = 0; i < map->wcnt; word++, i++) {
		if (~*word) {
			for (bit = 0; bit < LGSIZE; bit++, bitAcc++) {
				if (bitAcc == map->len) {
					return (GB_NOBITS);
				}
				if (~*word & (1L << bit)) {
					return (i * LGSIZE + bit);
				}
			}
		} else {
			bitAcc += LGSIZE;
		}
	}

	return (GB_NOBITS);
}

/*
 *  ======== GB_minset ========
 *  purpose:
 *      Returns the first set bit location in the bit map.
 */
GB_BitNum GB_minset(struct GB_TMap *map)
{
	GB_BitNum bitAcc = 0;
	GB_WordNum i;
	GB_BitNum bit;
	LgUns *word;

	for (word = map->words, i = 0; i < map->wcnt; word++, i++) {
		if (*word) {
			for (bit = 0; bit < LGSIZE; bit++, bitAcc++) {
				if (bitAcc == map->len)
					return (GB_NOBITS);
				else if (*word & (1L << bit))
					return ((GB_BitNum) (i * LGSIZE + bit));
			}
		} else {
			bitAcc += LGSIZE;
		}
	}

	return (GB_NOBITS);
}

/*
 *  ======== GB_not ========
 *  purpose:
 *      Performs NOT operation on the bit map provided.
 */

Void GB_not(struct GB_TMap *dst)
{
	GB_WordNum i;

	for (i = 0; i < dst->wcnt; i++)
		dst->words[i] ^= ~0L;

}

/*
 *  ======== GB_or ========
 *  purpose:
 *      Perform OR operation on the two bit maps.
 */
Void GB_or(struct GB_TMap *dst, struct GB_TMap *src)
{
	GB_WordNum i;

	for (i = 0; i < dst->wcnt; i++)
		dst->words[i] |= src->words[i];

}

/*
 *  ======== GB_set ========
 *  purpose:
 *      Sets a bit in the bit map.
 */

Void GB_set(struct GB_TMap *map, GB_BitNum bitn)
{
	LgUns mask;

	mask = 1L << (bitn % LGSIZE);
	map->words[bitn / LGSIZE] |= mask;
}

/*
 *  ======== GB_test ========
 *  purpose:
 *      Returns TRUE if the bit is set in the specified location.
 */

Bool GB_test(struct GB_TMap *map, GB_BitNum bitn)
{
	LgUns mask;
	LgUns word;

	mask = 1L << (bitn % LGSIZE);
	word = map->words[bitn / LGSIZE];

	return (word & mask ? TRUE : FALSE);
}
