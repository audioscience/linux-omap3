/*
 * dspbridge/src/dynload/dlstream.h
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



#ifndef _DLSTREAM_H
#define _DLSTREAM_H

/*****************************************************************************
 *****************************************************************************
 *
 *                              DLSTREAM.H
 *
 * A class used by the dynamic loader for input of the module image
 *
 *****************************************************************************
 *****************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

#include "dynamic_loader.h"

/***************************************************************************
 * NOTE:    All methods take a 'thisptr' parameter that is a pointer to the
 *          environment for the DL Stream APIs.  See definition of
 *			DL_stream_t.
 **************************************************************************/

/* DL_stream_t extends the DL stream class with buffer management info  */
	struct DL_stream_t {
		struct Dynamic_Loader_Stream dstrm;
		unsigned char *mbase;
		uint32_t mcur;
	} ;

/***************************************************************************
 * DLstream_open
 *
 * PARAMETERS :
 *  image       Image to be loaded
 *  imagesize   Number of units to be loaded
 *
 * EFFECT :
 *  Set up the stream with the image to be loaded.  Initialize the pointer
 *  and size also.
 *
 **************************************************************************/
	int DLstream_open(struct DL_stream_t *thisptr, void *image);

/***************************************************************************
 * DLSTREAM_open
 *
 * PARAMETERS :
 *   None
 *
 * EFFECT :
 *  Do any necessary cleanup for the stream that was loaded.
 *
 *************************************************************************/
	void DLstream_close(struct DL_stream_t *thisptr);

/*************************************************************************
 * DLstream_init
 *
 * PARAMETERS :
 *   None
 *
 * EFFECT :
 *  Initialize the handlers for DL APIs
 *
 *************************************************************************/
	void DLstream_init(struct DL_stream_t *thisptr);

#ifdef __cplusplus
}
#endif
#endif				/* _DLSTREAM_H */
