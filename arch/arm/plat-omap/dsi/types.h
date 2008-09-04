/*
 * arch/arm/plat-omap/types.h
 *
 * Header file for the OMAP DSI low level driver file (dsi.c)
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

/*==== DECLARATION CONTROL =================================================*/

#ifndef OMAP_TYPES_H
#define OMAP_TYPES_H

/*==== INCLUDES ============================================================*/
/*TBD*/

/*==== CONSTS ==============================================================*/

/*TBD*/

/*==== TYPEDEFINES =========================================================*/

typedef unsigned char U8;
typedef signed char S8;
typedef char C8;
typedef U8 UWORD8;
typedef S8 WORD8;
typedef U8 BOOLEAN;

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

typedef unsigned short int U16;
typedef signed short int S16;
typedef U16 UWORD16;
typedef S16 WORD16;

typedef float F32;

/* #if 0
 * typedef unsigned int U32;
 * typedef signed int S32;
 * #endif */

typedef unsigned long U32;
typedef signed long S32;
typedef U32 UWORD32;
typedef S32 WORD32;

#ifndef NULL
#define NULL    ((void *)0)
#endif


/* This currently validates Sdu Handle by checking whether it is NULL or not.
If a SDU function is provided in future to validate SDU Handle, this
macro needs to be modified */
#define VALIDATE_SDU_HANDLE(SduHandle) ((SduHandle != (U32)NULL) ? TRUE : FALSE)


/*====== FUNCTION PROTOTYPES=================================================*/

#endif /* OMAP_TYPES_H */

