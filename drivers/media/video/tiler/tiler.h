/*
 *  Copyright 2010 by Texas Instruments Incorporated.
 *
 */

/*
 * tiler.h
 *
 * TILER driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2009-2010 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef _TILER_H_
#define _TILER_H_

#define TILER_MEM_8BIT  0x60000000
#define TILER_MEM_16BIT 0x68000000
#define TILER_MEM_32BIT 0x70000000
#define TILER_MEM_PAGED 0x78000000
#define TILER_MEM_END   0x80000000

#define TILER_PAGESIZE 0x1000
#define TILER_WIDTH    256
#define TILER_HEIGHT   128
#define TILER_BLOCK_WIDTH  64
#define TILER_BLOCK_HEIGHT 64
#define TILER_LENGTH (TILER_WIDTH * TILER_HEIGHT * TILER_PAGESIZE)

#define TILER_DEVICE_PATH "/dev/tiler"
#define TILER_MAX_NUM_BLOCKS 16

extern int errno;

enum tiler_fmt {
        TILFMT_MIN     = -1,
        TILFMT_INVALID = -1,
        TILFMT_NONE    = 0,
        TILFMT_8BIT    = 1,
        TILFMT_16BIT   = 2,
        TILFMT_32BIT   = 3,
        TILFMT_PAGE    = 4,
        TILFMT_MAX     = 4
};

struct area {
        unsigned short width;
        unsigned short height;
};

struct tiler_block_info {
        enum tiler_fmt fmt;
        union {
                struct area area;
                unsigned long len;
        } dim;
        unsigned long stride;
        void *ptr;
        unsigned long ssptr;
};

struct tiler_buf_info {
        int num_blocks;
        struct tiler_block_info blocks[TILER_MAX_NUM_BLOCKS];
        int offset;
};

#define TILIOC_OPEN  _IOWR('z', 100, unsigned long)
#define TILIOC_GBUF  _IOWR('z', 101, unsigned long)
#define TILIOC_FBUF  _IOWR('z', 102, unsigned long)
#define TILIOC_CLOSE _IOWR('z', 103, unsigned long)
#define TILIOC_GSSP  _IOWR('z', 104, unsigned long)
#define TILIOC_MBUF  _IOWR('z', 105, unsigned long)
#define TILIOC_QBUF  _IOWR('z', 106, unsigned long)
#define TILIOC_RBUF  _IOWR('z', 109, unsigned long)
#define TILIOC_URBUF _IOWR('z', 110, unsigned long)
#define TILIOC_QUERY_BLK _IOWR('z', 111, unsigned long)

struct dmmViewOrientT {
        unsigned char dmm90Rotate;
        unsigned char dmmXInvert;
        unsigned char dmmYInvert;
};

int
tiler_alloc_buf(enum tiler_fmt fmt,
                unsigned long width,
                unsigned long height,
                void **sysptr);

int
tiler_free_buf(unsigned long sysptr);

unsigned long
tiler_reorient_addr(unsigned long tsptr, struct dmmViewOrientT orient);

unsigned long
tiler_get_natural_addr(void *sysPtr);

unsigned long
tiler_reorient_topleft(unsigned long tsptr,
                       struct dmmViewOrientT orient,
                       unsigned int validDataWidth,
                       unsigned int validDataHeight);

void
tiler_rotate_view(struct dmmViewOrientT *orient, unsigned long rotation);

unsigned long
tiler_stride(unsigned long tsptr);

void
tiler_packed_alloc_nv12_buf(int *count,
                            unsigned long width,
                            unsigned long height,
                            void **y_sysptr,
                            void **uv_sysptr,
                            void **y_allocptr,
                            void **uv_allocptr,
                            int aligned);

void
tiler_packed_alloc_buf(int *count,
                       enum tiler_fmt fmt,
                       unsigned long width,
                       unsigned long height,
                       void **sysptr,
                       void **allocptr,
                       int aligned);

#endif
/*
 *  @(#) ti.sdo.tiler.linux; 1, 0, 0,10; 4-1-2010 17:09:40; /db/atree/library/trees/linuxutils/linuxutils-g08x/src/
 */

