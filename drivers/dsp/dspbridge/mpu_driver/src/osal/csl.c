/*
 * dspbridge/src/osal/linux/csl.c
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
 *  ======== cslce.c ========
 *  Purpose:
 *      Provides platform independent C Standard library functions.
 *
 *  Public Functions:
 *      CSL_AnsiToWchar
 *      CSL_Atoi
 *      CSL_ByteSwap
 *      CSL_Exit
 *      CSL_Init
 *      CSL_NumToAscii
 *      CSL_Strcmp
 *      CSL_Strstr
 *      CSL_Strcpyn
 *      CSL_Strlen
 *      CSL_Strncat
 *      CSL_Strncmp
 *      CSL_Strtok
 *      CSL_Strtokr
 *      CSL_WcharToAnsi
 *      CSL_Wstrlen
 *
 *! Revision History:
 *! ================
 *! 07-Aug-2002 jeh: Added CSL_Strtokr().
 *! 21-Sep-2001 jeh: Added CSL_Strncmp(). Alphabetized functions.
 *! 22-Nov-2000 map: Added CSL_Atoi and CSL_Strtok
 *! 19-Nov-2000 kc: Added CSL_ByteSwap.
 *! 09-Nov-2000 kc: Added CSL_Strncat.
 *! 03-Feb-2000 rr: Module init/exit is handled by OSAL Init/Exit.GT Changes.
 *! 15-Dec-1999 ag: Removed incorrect assertion CSL_NumToAscii()
 *! 29-Oct-1999 kc: Added CSL_Wstrlen for UNICODE strings.
 *! 30-Sep-1999 ag: Removed DBC assertion (!CSL_DebugMask.flags) in
 *		  CSP_Init().
 *! 20-Sep-1999 ag: Added CSL_WcharToAnsi().
 *!		 Removed call to GT_set().
 *! 19-Jan-1998 cr: Code review cleanup.
 *! 29-Dec-1997 cr: Made platform independant, using MS CRT code, and
 *!		 combined csl32.c csl95.c and cslnt.c into csl.c.  Also
 *!		 changed CSL_lowercase to CSL_Uppercase.
 *! 21-Aug-1997 gp: Fix to CSL_strcpyn to initialize Source string, the NT way.
 *! 25-Jun-1997 cr: Created from csl95, added CSL_strcmp.
 */

/* ----------------------------------- Host OS */
#include <host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <std.h>
#include <dbdefs.h>

/*  ----------------------------------- Trace & Debug */
#include <dbc.h>
#include <dbg_zones.h>
#include <gt.h>

/*  ----------------------------------- This */
#include <csl.h>

/* Is character c in the string pstrDelim? */
#define IsDelimiter(c, pstrDelim) ((c != '\0') && \
				   (strchr(pstrDelim, c) != NULL))

/*  ----------------------------------- Globals */
#if GT_TRACE
static struct GT_Mask CSL_DebugMask = { 0, 0 };	/* GT trace var. */
#endif

#ifdef UNICODE
/*
 *  ======== CSL_AnsiToWchar ========
 *  Purpose:
 *  Convert an ansi string to a wide CHAR string.
 */
ULONG CSL_AnsiToWchar(OUT WCHAR *pwszDest, IN PSTR pstrSource, IN ULONG uSize)
{
	DWORD dwLength;
	DWORD i;

	if (!pstrSource)
		return (0);


	dwLength = CSL_Strlen(pstrSource);
	for (i = 0; (i < dwLength) && (i < (uSize - 1)); i++)
		pwszDest[i] = pstrSource[i];

	pwszDest[i] = '\0';

	return (i);
}
#endif

/*
 *  ======= CSL_Atoi =======
 *  Purpose:
 *      Convert a string to an integer
 */
INT CSL_Atoi(IN CONST CHAR *ptstrSrc)
{
	char *end_position;

	DBC_Require(ptstrSrc);

	return simple_strtol(ptstrSrc, &end_position, 10);
}

/*
 *  ======== CSL_ByteSwap ========
 *  Purpose:
 *      Swap bytes in buffer.
 */
VOID CSL_ByteSwap(IN PSTR pstrSrc, OUT PSTR pstrDest, IN ULONG ulBytes)
{
	int i, tmp;

	DBC_Require((ulBytes % 2) == 0);
	DBC_Require(pstrSrc && pstrDest);

	/* swap bytes. */
	for (i = 0; i < ulBytes / 2; i++) {
		tmp = 2 * i + 1;
		pstrDest[2 * i] = pstrSrc[tmp];
		pstrDest[tmp] = pstrSrc[2 * i];
	}
}

/*
 *  ======== CSL_Exit ========
 *  Purpose:
 *      Discontinue usage of the CSL module.
 */
void CSL_Exit()
{
	GT_0trace(CSL_DebugMask, GT_5CLASS, "CSL_Exit\n");
}

/*
 *  ======== CSL_Init ========
 *  Purpose:
 *      Initialize the CSL module's private state.
 */
BOOL CSL_Init()
{
	GT_create(&CSL_DebugMask, "CS");

	GT_0trace(CSL_DebugMask, GT_5CLASS, "CSL_Init\n");

	return (TRUE);
}

/*
 *  ======== CSL_NumToAscii ========
 *  Purpose:
 *      Convert a 1 or 2 digit number to a 2 digit string.
 */
VOID CSL_NumToAscii(OUT PSTR pstrNumber, DWORD dwNum)
{
	CHAR tens;

	DBC_Require(dwNum < 100);

	if (dwNum < 100) {
		tens = (CHAR) dwNum / 10;
		dwNum = dwNum % 10;

		if (tens) {
			pstrNumber[0] = tens + '0';
			pstrNumber[1] = (CHAR) dwNum + '0';
			pstrNumber[2] = '\0';
		} else {
			pstrNumber[0] = (CHAR) dwNum + '0';
			pstrNumber[1] = '\0';
		}
	} else {
		pstrNumber[0] = '\0';
	}
}

/*
 *  ======== CSL_Strcmp ========
 *  Purpose:
 *      Compare 2 ASCII strings.  Works the same was as stdio's strcmp.
 */
LONG CSL_Strcmp(IN CONST PSTR pstrStr1, IN CONST PSTR pstrStr2)
{
#ifdef LINUX
	return strcmp(pstrStr1, pstrStr2);
#else
	INT ret = 0;
	CONST CHAR *src = pstrStr1;
	CONST CHAR *dst = pstrStr2;

	DBC_Require(pstrStr1 != NULL);
	DBC_Require(pstrStr2 != NULL);

	while (!(ret = *(UCHAR *)src - *(UCHAR *)dst) && *dst)
		++src, ++dst;

	if (ret < 0)
		ret = -1;
	else if (ret > 0)
		ret = 1;

	return (ret);
#endif
}

/*
 *  ======== CSL_Strstr ========
 *  Purpose:
 *      Find substring in a stringn.
 *  Parameters:
 *      haystack:   Ptr to string1.
 *      needle:    Ptr to substring to catch.
 *  Returns:
 *      Ptr to first char matching the substring in the main string.
 *  Requires:
 *      CSL initialized.
 *      haystack is valid.
 *      needle is valid.
 *  Ensures:
 */

PSTR CSL_Strstr(IN CONST PSTR haystack, IN CONST PSTR needle)
{
#ifdef LINUX
	return (strstr(haystack, needle));
#else
	return NULL;
#endif
}

/*
 *  ======== CSL_Strcpyn ========
 *  Purpose:
 *      Safe strcpy function.
 */
PSTR CSL_Strcpyn(OUT PSTR pstrDest, IN CONST PSTR pstrSrc, DWORD cMax)
{
#ifdef LINUX
	return strncpy(pstrDest, pstrSrc, cMax);
#else
	CHAR *dest = pstrDest;
	CONST CHAR *source = pstrSrc;
	CHAR *start = dest;

	DBC_Require(pstrDest != NULL);
	DBC_Require(pstrSrc != NULL);

	while (cMax && (*dest++ = *source++)) {	/* copy string */
		cMax--;
	}

	if (cMax) {
		/* pad out with zeroes */
		while (--cMax)
			*dest++ = '\0';

	}
	return (start);
#endif
}

/*
 *  ======== CSL_Strlen ========
 *  Purpose:
 *      Determine the length of a null terminated ASCII string.
 */
DWORD CSL_Strlen(IN CONST PSTR pstrSrc)
{
	CONST CHAR *pStr = pstrSrc;

	DBC_Require(pstrSrc);

	while (*pStr++) ;

	return ((DWORD) (pStr - pstrSrc - 1));
}

/*
 *  ======== CSL_Strncat ========
 *  Purpose:
 *      Concatenate two strings together
 */
PSTR CSL_Strncat(IN PSTR pszDest, IN PSTR pszSrc, IN DWORD dwSize)
{

	DBC_Require(pszDest && pszSrc);

	return (strncat(pszDest, pszSrc, dwSize));
}

/*
 *  ======== CSL_Strncmp ========
 *  Purpose:
 *      Compare at most n characters of two ASCII strings.  Works the same
 *      way as stdio's strncmp.
 */
LONG CSL_Strncmp(IN CONST PSTR pstrStr1, IN CONST PSTR pstrStr2, DWORD n)
{
#ifdef LINUX
	return strncmp(pstrStr1, pstrStr2, n);
#else
	INT ret = 0;
	CONST CHAR *src = pstrStr1;
	CONST CHAR *dst = pstrStr2;
	ULONG i = 0;

	DBC_Require(pstrStr1 != NULL);
	DBC_Require(pstrStr2 != NULL);

	while (i < n && !(ret = *(UCHAR *) src - *(UCHAR *) dst)
	       && *src && *dst) {
		++src, ++dst;
		++i;
	}

	if (ret < 0)
		ret = -1;
	else if (ret > 0)
		ret = 1;

	return (ret);
#endif
}

#ifndef LINUX
/*
 *  ======= CSL_Strtok =======
 *  Purpose:
 *      Tokenize an input string based on the separators
 */
CHAR *CSL_Strtok(IN CHAR *ptstrSrc, IN CONST CHAR *szSeparators)
{

	/*
	 * Not implemented in Linux because strsep requires first argument to be
	 * CHAR **. This function is not called from Bridge.
	 */
	return (NULL);
	/*  return (strtok(ptstrSrc, szSeparators));  */
}
#endif

/*
 *  ======= CSL_Strtokr =======
 *  Purpose:
 *      Re-entrant version of strtok.
 */
CHAR *CSL_Strtokr(IN CHAR *pstrSrc, IN CONST CHAR *szSeparators,
		  OUT CHAR **ppstrLast)
{
	CHAR *pstrTemp;
	CHAR *pstrToken;

	DBC_Require(szSeparators != NULL);
	DBC_Require(ppstrLast != NULL);
	DBC_Require(pstrSrc != NULL || *ppstrLast != NULL);

	/*
	 *  Set string location to beginning (pstrSrc != NULL) or to the
	 *  beginning of the next token.
	 */
	pstrTemp = (pstrSrc != NULL) ? pstrSrc : *ppstrLast;
	if (*pstrTemp == '\0') {
		pstrToken = NULL;
	} else {
		pstrToken = pstrTemp;
		while (*pstrTemp != '\0' && !IsDelimiter(*pstrTemp,
		      szSeparators)) {
			pstrTemp++;
		}
		if (*pstrTemp != '\0') {
			while (IsDelimiter(*pstrTemp, szSeparators)) {
				/* TODO: Shouldn't we do this for
				 * only 1 char?? */
				*pstrTemp = '\0';
				pstrTemp++;
			}
		}

		/* Location in string for next call */
		*ppstrLast = pstrTemp;
	}

	return (pstrToken);
}

#ifdef UNICODE
/*
 *  ======== CSL_WcharToAnsi ========
 *  Purpose:
 *      UniCode to Ansi conversion.
 *  Note:
 *      uSize is # of chars in destination buffer.
 */
ULONG CSL_WcharToAnsi(OUT PSTR pstrDest, IN WCHAR *pwszSource, ULONG uSize)
{
	PSTR pstrTemp = pstrDest;
	ULONG uNumOfChars = 0;

	if (!pwszSource)
		return (0);

	while ((*pwszSource != TEXT('\0')) && (uSize-- > 0)) {
		*pstrTemp++ = (CHAR) * pwszSource++;
		uNumOfChars++;
	}
	*pstrTemp = '\0';

	return (uNumOfChars);
}

/*
 *  ======== CSL_Wstrlen ========
 *  Purpose:
 *      Determine the length of a null terminated UNICODE string.
 */
DWORD CSL_Wstrlen(IN CONST TCHAR *ptstrSrc)
{
	CONST TCHAR *ptstr = ptstrSrc;

	DBC_Require(ptstrSrc);

	while (*ptstr++) ;

	return ((DWORD) (ptstr - ptstrSrc - 1));
}
#endif
