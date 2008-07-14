/*
 * dspbridge/src/gen/gt.c
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
 * ======== gt.c ========
 * Description: This module implements the trace mechanism for bridge.
 *
 *! Revision History
 *! ================
 *! 16-May-1997 dr	Changed GT_Config member names to conform to coding
 *!			standards.
 *! 23-Apr-1997 ge	Check for GT->TIDFXN for NULL before calling it.
 *! 03-Jan-1997	ge	Changed GT_Config structure member names to eliminate
 *!			preprocessor confusion with other macros.
 */

/*  ----------------------------------- DSP/BIOS Bridge */
#include <std.h>
#include <stdarg.h>

/*  ----------------------------------- This */
#include <gt.h>

#define GT_WILD	'*'

#define GT_CLEAR	'='
#define GT_ON		'+'
#define GT_OFF		'-'

typedef enum {
	GT_SEP,
	GT_FIRST,
	GT_SECOND,
	GT_OP,
	GT_DIGITS
} GT_State;

String GT_1format = "%s - %d: ";
String GT_2format = "%s - %d(%d): ";

SmBits *GT_tMask[GT_BOUND];

static Bool curInit = FALSE;
static String separator;
static SmBits tabMem[GT_BOUND][sizeof(SmBits) * GT_BOUND];

static Void error(String string);
static Void setMask(MdInt index1, MdInt index2, Char op, SmBits mask);

/*
 *  ======== _GT_create ========
 *  purpose:
 *      Creates GT mask.
 */
Void _GT_create(struct GT_Mask *mask, String modName)
{
	mask->modName = modName;
	mask->flags = &(GT_tMask[modName[0] - 'A'][modName[1] - 'A']);
}

/*
 *  ======== GT_init ========
 *  purpose:
 *      Initializes GT module.
 */
#ifdef GT_init
#undef GT_init
#endif
Void GT_init(void)
{
	register SmUns index1;
	register SmUns index2;

	if (!curInit) {
		curInit = TRUE;

		separator = " ,;/";

		for (index1 = 0; index1 < GT_BOUND; index1++) {
			GT_tMask[index1] = tabMem[index1];
			for (index2 = 0; index2 < GT_BOUND; index2++) {
				/* no tracing */
				GT_tMask[index1][index2] = 0x00;
			}
		}
	}
}

/*
 *  ======== _GT_set ========
 *  purpose:
 *      Sets the trace string format.
 */

Void _GT_set(String str)
{
	GT_State state;
	String sep;
	MdInt index1 = GT_BOUND;	/* indicates all values */
	MdInt index2 = GT_BOUND;	/* indicates all values */
	Char op = GT_CLEAR;
	Bool maskValid;
	MdInt digit;
	register SmBits mask = 0x0;	/* no tracing */

	if (str == NULL)
		return;

	maskValid = FALSE;
	state = GT_SEP;
	while (*str != NULL) {
		switch ((Int) state) {
		case (Int) GT_SEP:
			maskValid = FALSE;
			sep = separator;
			while (*sep != NULL) {
				if (*str == *sep) {
					str++;
					break;
				} else {
					sep++;
				}
			}
			if (*sep == NULL)
				state = GT_FIRST;

			break;
		case (Int) GT_FIRST:
			if (*str == GT_WILD) {
				/* indicates all values */
				index1 = GT_BOUND;
				/* indicates all values */
				index2 = GT_BOUND;
				state = GT_OP;
			} else {
				if (*str >= 'a')
					index1 = (MdInt) (*str - 'a');
				else
					index1 = (MdInt) (*str - 'A');
				if ((index1 >= 0) && (index1 < GT_BOUND))
					state = GT_SECOND;
				else
					state = GT_SEP;
			}
			str++;
			break;
		case (Int) GT_SECOND:
			if (*str == GT_WILD) {
				index2 = GT_BOUND;   /* indicates all values */
				state = GT_OP;
				str++;
			} else {
				if (*str >= 'a')
					index2 = (MdInt) (*str - 'a');
				else
					index2 = (MdInt) (*str - 'A');
				if ((index2 >= 0) && (index2 < GT_BOUND)) {
					state = GT_OP;
					str++;
				} else {
					state = GT_SEP;
				}
			}
			break;
		case (Int) GT_OP:
			op = *str;
			mask = 0x0;	/* no tracing */
			switch (op) {
			case (Int) GT_CLEAR:
				maskValid = TRUE;
			case (Int) GT_ON:
			case (Int) GT_OFF:
				state = GT_DIGITS;
				str++;
				break;
			default:
				state = GT_SEP;
				break;
			}
			break;
		case (Int) GT_DIGITS:
			digit = (MdInt) (*str - '0');
			if ((digit >= 0) && (digit <= 7)) {
				mask |= (0x01 << digit);
				maskValid = TRUE;
				str++;
			} else {
				if (maskValid == TRUE) {
					setMask(index1, index2, op, mask);
					maskValid = FALSE;
				}
				state = GT_SEP;
			}
			break;
		default:
			error("illegal trace mask");
			break;
		}
	}

	if (maskValid)
		setMask(index1, index2, op, mask);
}

/*
 *  ======== _GT_trace ========
 *  purpose:
 *      Prints the input string onto standard output
 */

Int _GT_trace(struct GT_Mask *mask, String format, ...)
{
	Int arg1, arg2, arg3, arg4, arg5, arg6;
	va_list va;

	va_start(va, format);

	arg1 = va_arg(va, Int);
	arg2 = va_arg(va, Int);
	arg3 = va_arg(va, Int);
	arg4 = va_arg(va, Int);
	arg5 = va_arg(va, Int);
	arg6 = va_arg(va, Int);

	va_end(va);
#ifdef DEBUG
	if (GT->PIDFXN == NULL) {
		(*GT->PRINTFXN)(GT_1format, mask->modName, GT->TIDFXN ?
		(*GT->TIDFXN)() : 0);
	} else {
		(*GT->PRINTFXN)(GT_2format, mask->modName, (*GT->PIDFXN)(),
		GT->TIDFXN ? (*GT->TIDFXN)() : 0);
	}
#endif
	(*GT->PRINTFXN)(format, arg1, arg2, arg3, arg4, arg5, arg6);

	return 0;
}

/*
 *  ======== error ========
 *  purpose:
 *      Prints errors onto the standard output.
 */
static Void error(String string)
{
	(*GT->PRINTFXN)("GT: %s", string);
}

/*
 *  ======== setmask ========
 *  purpose:
 *      Sets mask for the GT module.
 */

static Void setMask(MdInt index1, MdInt index2, Char op, SmBits mask)
{
	register MdInt index;

	if (index1 < GT_BOUND) {
		if (index2 < GT_BOUND) {
			switch (op) {
			case (Int) GT_CLEAR:
				GT_tMask[index1][index2] = mask;
				break;
			case (Int) GT_ON:
				GT_tMask[index1][index2] |= mask;
				break;
			case (Int) GT_OFF:
				GT_tMask[index1][index2] &= ~mask;
				break;
			default:
				error("illegal trace mask");
				break;
			}
		} else {
			for (index2--; index2 >= 0; index2--) {
				switch (op) {
				case (Int) GT_CLEAR:
					GT_tMask[index1][index2] = mask;
					break;
				case (Int) GT_ON:
					GT_tMask[index1][index2] |= mask;
					break;
				case (Int) GT_OFF:
					GT_tMask[index1][index2] &= ~mask;
					break;
				default:
					error("illegal trace mask");
					break;
				}
			}
		}
	} else {
		for (index1--; index1 >= 0; index1--) {
			if (index2 < GT_BOUND) {
				switch (op) {
				case (Int) GT_CLEAR:
					GT_tMask[index1][index2] = mask;
					break;
				case (Int) GT_ON:
					GT_tMask[index1][index2] |= mask;
					break;
				case (Int) GT_OFF:
					GT_tMask[index1][index2] &= ~mask;
					break;
				default:
					error("illegal trace mask");
					break;
				}
			} else {
				index = GT_BOUND;
				for (index--; index >= 0; index--) {
					switch (op) {
					case (Int) GT_CLEAR:
						GT_tMask[index1][index] = mask;
						break;
					case (Int) GT_ON:
						GT_tMask[index1][index] |= mask;
						break;
					case (Int) GT_OFF:
						GT_tMask[index1][index] &=
						    ~mask;
						break;
					default:
						error("illegal trace mask");
						break;
					}
				}
			}
		}
	}
}
