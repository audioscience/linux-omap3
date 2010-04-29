/*
 *  Copyright 2010 by Texas Instruments Incorporated.
 *
 */

/*
 * dmm_prv.h
 *
 * DMM driver support functions for TI OMAP processors.
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

#ifndef _DMM_PRV_H
#define _DMM_PRV_H

#include "dmm_2d_alloc.h"
#include "dmm_drv.h"
#include "dmm_reg.h"

#if 0
#define regdump(x, y) printk(KERN_NOTICE "%s()::%d:%s=(0x%08x)\n", \
                                __func__, __LINE__, x, (int)y);
#else
#define regdump(x, y)
#endif

#define DMM_BASE     0x4E000000
#define DMM_REVISION      0x000
#define DMM_HWINFO        0x004
#define LISA_HWINFO       0x008
#define DMM_SYSCONFIG     0x010
#define LISA_LOCK         0x01C
#define LISA_MAP__0       0x040
#define LISA_MAP__1       0x044
#define TILER_HWINFO      0x208
#define TILER_OR__0       0x220
#define TILER_OR__1       0x224
#define PAT_HWINFO        0x408
#define PAT_GEOMETRY      0x40C
#define PAT_CONFIG        0x410
#define PAT_VIEW__0       0x420
#define PAT_VIEW__1       0x424
#define PAT_VIEW_MAP__0   0x440
#define PAT_VIEW_MAP_BASE 0x460
#define PAT_IRQ_EOI       0x478
#define PAT_IRQSTATUS_RAW 0x480
#define PAT_IRQSTATUS     0x490
#define PAT_IRQENABLE_SET 0x4A0
#define PAT_IRQENABLE_CLR 0x4B0
#define PAT_STATUS__0     0x4C0
#define PAT_STATUS__1     0x4C4
#define PAT_STATUS__2     0x4C8
#define PAT_STATUS__3     0x4CC
#define PAT_DESCR__0      0x500
#define PAT_AREA__0       0x504
#define PAT_CTRL__0       0x508
#define PAT_DATA__0       0x50C
#define PEG_HWINFO        0x608
#define PEG_PRIO          0x620
#define PEG_PRIO_PAT      0x640

#define DMM__DMM    DMM__DMM
#define DMM_PAT_AREA_IRQ (0)

#define DMM_MNGD_PHYS_PAGES (16)

#define DMM_TILE_DIMM_X_MODE_8    (32)
#define DMM_TILE_DIMM_Y_MODE_8    (32)

#define DMM_TILE_DIMM_X_MODE_16    (32)
#define DMM_TILE_DIMM_Y_MODE_16    (16)

#define DMM_TILE_DIMM_X_MODE_32    (16)
#define DMM_TILE_DIMM_Y_MODE_32    (16)

#define DMM_PAGE_DIMM_X_MODE_8    (DMM_TILE_DIMM_X_MODE_8*2)
#define DMM_PAGE_DIMM_Y_MODE_8    (DMM_TILE_DIMM_Y_MODE_8*2)

#define DMM_PAGE_DIMM_X_MODE_16    (DMM_TILE_DIMM_X_MODE_16*2)
#define DMM_PAGE_DIMM_Y_MODE_16    (DMM_TILE_DIMM_Y_MODE_16*2)

#define DMM_PAGE_DIMM_X_MODE_32    (DMM_TILE_DIMM_X_MODE_32*2)
#define DMM_PAGE_DIMM_Y_MODE_32    (DMM_TILE_DIMM_Y_MODE_32*2)

#define DMM_HOR_X_ADDRSHIFT_8            (0)
#define DMM_HOR_X_ADDRMASK_8            (0x3FFF)
#define DMM_HOR_X_COOR_GET_8(x)\
        (((unsigned long)x >> DMM_HOR_X_ADDRSHIFT_8) & DMM_HOR_X_ADDRMASK_8)
#define DMM_HOR_X_PAGE_COOR_GET_8(x)\
                                (DMM_HOR_X_COOR_GET_8(x)/DMM_PAGE_DIMM_X_MODE_8)

#define DMM_HOR_Y_ADDRSHIFT_8            (14)
#define DMM_HOR_Y_ADDRMASK_8            (0x1FFF)
#define DMM_HOR_Y_COOR_GET_8(x)\
        (((unsigned long)x >> DMM_HOR_Y_ADDRSHIFT_8) & DMM_HOR_Y_ADDRMASK_8)
#define DMM_HOR_Y_PAGE_COOR_GET_8(x)\
                                (DMM_HOR_Y_COOR_GET_8(x)/DMM_PAGE_DIMM_Y_MODE_8)

#define DMM_HOR_X_ADDRSHIFT_16            (1)
#define DMM_HOR_X_ADDRMASK_16            (0x7FFE)
#define DMM_HOR_X_COOR_GET_16(x)        (((unsigned long)x >> \
                                DMM_HOR_X_ADDRSHIFT_16) & DMM_HOR_X_ADDRMASK_16)
#define DMM_HOR_X_PAGE_COOR_GET_16(x)    (DMM_HOR_X_COOR_GET_16(x) /           \
                                DMM_PAGE_DIMM_X_MODE_16)

#define DMM_HOR_Y_ADDRSHIFT_16            (15)
#define DMM_HOR_Y_ADDRMASK_16            (0xFFF)
#define DMM_HOR_Y_COOR_GET_16(x)        (((unsigned long)x >>                  \
                                DMM_HOR_Y_ADDRSHIFT_16) & DMM_HOR_Y_ADDRMASK_16)
#define DMM_HOR_Y_PAGE_COOR_GET_16(x)    (DMM_HOR_Y_COOR_GET_16(x) /           \
                                DMM_PAGE_DIMM_Y_MODE_16)

#define DMM_HOR_X_ADDRSHIFT_32            (2)
#define DMM_HOR_X_ADDRMASK_32            (0x7FFC)
#define DMM_HOR_X_COOR_GET_32(x)        (((unsigned long)x >>                  \
                                DMM_HOR_X_ADDRSHIFT_32) & DMM_HOR_X_ADDRMASK_32)
#define DMM_HOR_X_PAGE_COOR_GET_32(x)    (DMM_HOR_X_COOR_GET_32(x) /           \
                                DMM_PAGE_DIMM_X_MODE_32)

#define DMM_HOR_Y_ADDRSHIFT_32            (15)
#define DMM_HOR_Y_ADDRMASK_32            (0xFFF)
#define DMM_HOR_Y_COOR_GET_32(x)        (((unsigned long)x >>                  \
                                DMM_HOR_Y_ADDRSHIFT_32) & DMM_HOR_Y_ADDRMASK_32)
#define DMM_HOR_Y_PAGE_COOR_GET_32(x)    (DMM_HOR_Y_COOR_GET_32(x) /           \
                                DMM_PAGE_DIMM_Y_MODE_32)

/* :TODO: to be determined */

#define DMM_VER_X_ADDRSHIFT_8            (14)
#define DMM_VER_X_ADDRMASK_8            (0x1FFF)
#define DMM_VER_X_COOR_GET_8(x)\
        (((unsigned long)x >> DMM_VER_X_ADDRSHIFT_8) & DMM_VER_X_ADDRMASK_8)
#define DMM_VER_X_PAGE_COOR_GET_8(x)\
                                (DMM_VER_X_COOR_GET_8(x)/DMM_PAGE_DIMM_X_MODE_8)

#define DMM_VER_Y_ADDRSHIFT_8            (0)
#define DMM_VER_Y_ADDRMASK_8            (0x3FFF)
#define DMM_VER_Y_COOR_GET_8(x)\
        (((unsigned long)x >> DMM_VER_Y_ADDRSHIFT_8) & DMM_VER_Y_ADDRMASK_8)
#define DMM_VER_Y_PAGE_COOR_GET_8(x)\
                                (DMM_VER_Y_COOR_GET_8(x)/DMM_PAGE_DIMM_Y_MODE_8)

#define DMM_VER_X_ADDRSHIFT_16            (14)
#define DMM_VER_X_ADDRMASK_16            (0x1FFF)
#define DMM_VER_X_COOR_GET_16(x)        (((unsigned long)x >>                  \
                                DMM_VER_X_ADDRSHIFT_16) & DMM_VER_X_ADDRMASK_16)
#define DMM_VER_X_PAGE_COOR_GET_16(x)    (DMM_VER_X_COOR_GET_16(x) /           \
                                DMM_PAGE_DIMM_X_MODE_16)

#define DMM_VER_Y_ADDRSHIFT_16            (0)
#define DMM_VER_Y_ADDRMASK_16            (0x3FFF)
#define DMM_VER_Y_COOR_GET_16(x)        (((unsigned long)x >>                  \
                                DMM_VER_Y_ADDRSHIFT_16) & DMM_VER_Y_ADDRMASK_16)
#define DMM_VER_Y_PAGE_COOR_GET_16(x)    (DMM_VER_Y_COOR_GET_16(x) /           \
                                DMM_PAGE_DIMM_Y_MODE_16)

#define DMM_VER_X_ADDRSHIFT_32            (15)
#define DMM_VER_X_ADDRMASK_32            (0xFFF)
#define DMM_VER_X_COOR_GET_32(x)        (((unsigned long)x >>                  \
                                DMM_VER_X_ADDRSHIFT_32) & DMM_VER_X_ADDRMASK_32)
#define DMM_VER_X_PAGE_COOR_GET_32(x)    (DMM_VER_X_COOR_GET_32(x) /           \
                                DMM_PAGE_DIMM_X_MODE_32)

#define DMM_VER_Y_ADDRSHIFT_32            (0)
#define DMM_VER_Y_ADDRMASK_32            (0x7FFF)
#define DMM_VER_Y_COOR_GET_32(x)        (((unsigned long)x >>                  \
                                DMM_VER_Y_ADDRSHIFT_32) & DMM_VER_Y_ADDRMASK_32)
#define DMM_VER_Y_PAGE_COOR_GET_32(x)    (DMM_VER_Y_COOR_GET_32(x) /           \
                                DMM_PAGE_DIMM_Y_MODE_32)

#define DMM_TILER_ALIAS_BASE    (0x60000000)
#define DMM_ALIAS_VIEW_CLEAR    (~0xE0000000)

#define DMM_ACC_MODE_SHIFT  (27)
#define DMM_ACC_MODE_MASK   (3)
#define DMM_GET_ACC_MODE(x) ((enum dmmMemoryAccessT)\
                (((unsigned long)x &\
                (DMM_ACC_MODE_MASK<<DMM_ACC_MODE_SHIFT))>>DMM_ACC_MODE_SHIFT))

#define DMM_X_INVERT_SHIFT        (29)
#define DMM_GET_X_INVERTED(x)\
                ((((unsigned long)x & (1<<DMM_X_INVERT_SHIFT)) > 0) ? 1 : 0)

#define DMM_Y_INVERT_SHIFT        (30)
#define DMM_GET_Y_INVERTED(x)\
                ((((unsigned long)x & (1<<DMM_Y_INVERT_SHIFT)) > 0) ? 1 : 0)

#define DMM_ROTATION_SHIFT        (31)
#define DMM_GET_ROTATED(x)\
((((unsigned long)x & ((unsigned long)1<<DMM_ROTATION_SHIFT)) > 0) ? 1 : 0)

#define DMM_COMPOSE_TILER_ALIAS_PTR(x, accM)\
((void *)(DMM_TILER_ALIAS_BASE | \
                        (unsigned long)x | (accM << DMM_ACC_MODE_SHIFT)))
#define DMM_COMPOSE_TILER_PTR(x, rot, yInv, xInv, accM)\
((void *)((unsigned long)x | (rot << DMM_ROTATION_SHIFT) |\
(yInv << DMM_Y_INVERT_SHIFT) | (xInv << DMM_X_INVERT_SHIFT) |\
(accM << DMM_ACC_MODE_SHIFT)))

/** @struc dmmPhysPgLLT
* Structure defining Dmm physical memory pages managment linked list. */
struct dmmPhysPgLLT {
        struct dmmPhysPgLLT *nextPhysPg;
        struct dmmPhysPgLLT *prevPhysPg;
        unsigned long *physPgPtr;
        struct page *page_addr;
};

/** @struc dmmHwdCtxT
* Structure defining Dmm driver hardware context. */
struct dmmHwdCtxT {
        signed long dmmOpenInstances;
        struct dmmPATIrqEventsT patIrqEvnt0;
        struct dmmPATIrqEventsT patIrqEvnt1;
        struct dmmPATIrqEventsT patIrqEvnt2;
        struct dmmPATIrqEventsT patIrqEvnt3;
};

/** @struc dmmInstanceCtxT
* Structure defining Dmm driver context. */
struct dmmInstanceCtxT {
        struct dmmHwdCtxT dmmHwdCtx;
        struct MSP_Dmm_eventNotificationT dmmMspCtx;
        struct dmmTILERContCtxT dmmTilerCtx;
};

/** @struc dmmPatRefillManip
* Enumeration of possible area refill manipulations. */
enum dmmPatRefillManipT {
        DMMP_REMAP,
        DMMP_SWAP,
        DMMP_MOVE
};

#ifdef __DMM_DEBUG_BUILD__
#define DBG_OVERLAP_TEST(x) overlapping_test(x)

struct DMM_rStruct_t {
        unsigned long DMM_REVISION;
        unsigned long DMM_HWINFO;
        unsigned long DMM_LISA_HWINFO;
        unsigned long DMM_SYSCONFIG;
        unsigned long DMM_LISA_LOCK;
        unsigned char filler1[32];
        unsigned long DMM_LISA_MAP__0;
        unsigned long DMM_LISA_MAP__1;
        unsigned long DMM_LISA_MAP__2;
        unsigned long DMM_LISA_MAP__3;
        unsigned char filler2[440];
        unsigned long DMM_TILER_HWINFO;
        unsigned char filler3[20];
        unsigned long DMM_TILER_OR0;
        unsigned long DMM_TILER_OR1;
        unsigned char filler4[476];
        unsigned long DMM_PAT_HWINFO;
        unsigned long DMM_PAT_GEOMETRY;
        unsigned char filler5[16];
        unsigned long DMM_PAT_VIEW0;
        unsigned long DMM_PAT_VIEW1;
        unsigned char filler6[24];
        unsigned long DMM_PAT_VIEW_MAP__0;
        unsigned long DMM_PAT_VIEW_MAP__1;
        unsigned long DMM_PAT_VIEW_MAP__2;
        unsigned long DMM_PAT_VIEW_MAP__3;
        unsigned char filler7[16];
        unsigned long DMM_PAT_VIEW_MAP_BASE;
        unsigned char filler8[28];
        unsigned long DMM_PAT_STATUS__0;
        unsigned long DMM_PAT_STATUS__1;
        unsigned long DMM_PAT_STATUS__2;
        unsigned long DMM_PAT_STATUS__3;
        unsigned char filler9[48];
        unsigned long DMM_PAT_IRQSTATUS_RAW;
        unsigned char filler10[12];
        unsigned long DMM_PAT_IRQSTATUS;
        unsigned char filler11[12];
        unsigned long DMM_PAT_IRQENABLE_SET;
        unsigned char filler12[12];
        unsigned long DMM_PAT_IRQENABLE_CLR;
        unsigned char filler13[12];
        unsigned long DMM_PAT_DESCR__0;
        unsigned long DMM_PAT_AREA__0;
        unsigned long DMM_PAT_CTRL__0;
        unsigned long DMM_PAT_DATA__0;
        unsigned long DMM_PAT_DESCR__1;
        unsigned long DMM_PAT_AREA__1;
        unsigned long DMM_PAT_CTRL__1;
        unsigned long DMM_PAT_DATA__1;
        unsigned long DMM_PAT_DESCR__2;
        unsigned long DMM_PAT_AREA__2;
        unsigned long DMM_PAT_CTRL__2;
        unsigned long DMM_PAT_DATA__2;
        unsigned long DMM_PAT_DESCR__3;
        unsigned long DMM_PAT_AREA__3;
        unsigned long DMM_PAT_CTRL__3;
        unsigned long DMM_PAT_DATA__3;
        unsigned char filler14[200];
        unsigned long DMM_PEG_HWINFO;
        unsigned char filler15[20];
        unsigned long DMM_PEG_PRIO0;
        unsigned long DMM_PEG_PRIO1;
        unsigned char filler16[24];
        unsigned long DMM_PEG_PRIO_PAT;
};

#else

#define DBG_OVERLAP_TEST(x)

#endif /* __DMM_DEBUG_BUILD__ */

/* ========================================================================== */
/**
 *  dmm_tiler_container_map_area()
 *
 * @brief  Allocates a 2D TILER buffer - virtual 2D allocation,
 * descriptor and TILER system pointer updating.
 *
 * @param dmmTilerCtx - dmmTILERContCtxT* - [in] Tiler context instance.
 *
 * @param sizeWidth - unsigned short - [in] Width of buffer
 * (in container elements).
 *
 * @param sizeHeight - unsigned short - [in] Height of buffer (in container
 *  elements).
 *
 * @param contMod - dmmMemoryAccessT - [in] Container access mode - for element
 * sizes.
 *
 * @param allocedPtr - void ** - [out] The allocated buffer system pointer is
 * provided through this double pointer. If no buffer is available this pointer
 * is set to NULL.
 *
 * @param bufferMappedZone - dmmTILERContPageAreaT** - [out] Description of the
 * 2D area that is mapped in the TILER container is provided in this structure.
 *
 * @return errorCodeT
 *
 * @pre There is no pre conditions.
 *
 * @post There is no post conditions.
 *
 * @see errorCodeT, dmmMemoryAccessT, dmmTILERContPageAreaT for further detail.
 */
/* ========================================================================== */
enum errorCodeT dmm_tiler_container_map_area(
        struct dmmTILERContCtxT *dmmTilerCtx,
        unsigned long sizeWidth,
        unsigned long sizeHeight,
        enum dmmMemoryAccessT contMod,
        void **allocedPtr,
        struct dmmTILERContPageAreaT **bufferMappedZone);

/* ========================================================================== */
/**
 *  dmm_tiler_populate_pat_page_entry_data()
 *
 * @brief  Populates an aray with PAT page address entries.
 *
 * @param bfrSize - unsigned long - [in] Size of the buffer which will be used
 * to generate page entries for.
 *
 * @param pageEntries - unsigned long** - [out] The address to the allocated
 * page entry data array, aligned due to hardware specification.
 *
 * @param pageEntriesSpace - unsigned long** - [out] The address to the
 * allocated page entry data array for deallocation purposes.
 *
 * @param custmPagesPtr - void * - [in] Pointer to a custom created memory
 * pages list.
 *
 * @return errorCodeT
 *
 * @pre There is no pre conditions.
 *
 * @post There is no post conditions.
 *
 * @see errorCodeT for further detail.
 */
/* ========================================================================== */
enum errorCodeT dmm_tiler_populate_pat_page_entry_data(unsigned long bfrSize,
                unsigned long **pageEntries,
                unsigned long **pageEntriesSpace,
                void *custmPagesPtr);

/* ========================================================================== */
/**
 *  dmm_tiler_swap_pat_page_entry_data()
 *
 * @brief  Swaps entries in an aray with PAT page address entries.
 *
 * @param bfrSize - unsigned long - [in] Size of the buffer which will be used
 * to generate page entries for.
 *
 * @param pageEntries - unsigned long* - [in] The address to the allocated
 * page entry data array, aligned due to hardware specification.
 *
 * @param affectedArea - PATAreaT* - [in] Area that will be affected.
 *
 * @param destX - unsigned short - [in] Destination coordinate X.
 *
 * @param destY - unsigned short - [in] Destination coordinate Y.
 *
 * @param stride - unsigned short - [in] Stride of the area.
 *
 * @return errorCodeT
 *
 * @pre There is no pre conditions.
 *
 * @post There is no post conditions.
 *
 * @see errorCodeT for further detail.
 */
/* ========================================================================== */
enum errorCodeT dmm_tiler_swap_pat_page_entry_data(unsigned long numPages,
                unsigned long *pageEntries,
                struct PATAreaT *affectedArea,
                unsigned short destX,
                unsigned short destY,
                unsigned short stride);

/* ========================================================================== */
/**
 *  dmm_tiler_container_unmap_area()
 *
 * @brief  Frees a 2D virtual TILER container buffer. \
 *
 * @param dmmTilerCtx - dmmTILERContCtxT* - [in] Tiler context instance.
 *
 * @param bufferMappedZone - dmmTILERContPageAreaT* - [in] Description of the 2D
 * area that is mapped in the TILER container is provided in this structure.
 *
 * @return errorCodeT
 *
 * @pre There is no pre conditions.
 *
 * @post There is no post conditions.
 *
 * @see errorCodeT, dmmTILERContPageAreaT for further detail.
 */
/* ========================================================================== */
enum errorCodeT dmm_tiler_container_unmap_area(
        struct dmmTILERContCtxT *dmmTilerCtx,
        struct dmmTILERContPageAreaT *bufferMappedZone);

/* ========================================================================== */
/**
 *  dmm_tiler_get_area_from_sysptr()
 *
 * @brief  Gets a 2D area descriptor from the Tiler system pointer.
 *
 * @param dmmTilerCtx - dmmTILERContCtxT* - [in] Tiler context instance.
 *
 * @param sysPtr - void * - [in] Tiler system pointer to a 2D area.
 *
 * @return dmmTILERContPageAreaT* Pointer to the 2D area descriptor, NULL if
 * error is encountered during extraction.
 *
 * @pre There is no pre conditions.
 *
 * @post There is no post conditions.
 *
 * @see dmmTILERContPageAreaT for further detail.
 */
/* ========================================================================== */
struct dmmTILERContPageAreaT *dmm_tiler_get_area_from_sysptr(
        struct dmmTILERContCtxT *dmmTilerCtx, void *sysPtr);

/* ========================================================================== */
/**
 *  dmm_phys_page_rep_init()
 *
 * @brief  Initializes the physical memory page repository instance.
 *
 * @return errorCodeT
 *
 * @pre There is no pre conditions.
 *
 * @post There is no post conditions.
 *
 * @see errorCodeT for further detail.
 */
/* ========================================================================== */
enum errorCodeT dmm_phys_page_rep_init(void);

/* ========================================================================== */
/**
 *  dmmPhysPageRepDeinit()
 *
 * @brief  Releases all resources held by the physical memory page repository
 * instance.
 *
 * @return errorCodeT
 *
 * @pre There is no pre conditions.
 *
 * @post There is no post conditions.
 *
 * @see errorCodeT for further detail.
 */
/* ========================================================================== */
enum errorCodeT dmm_phys_page_rep_deinit(void);

/* ========================================================================== */
/**
 *  dmm_get_phys_pages()
 *
 * @brief  Return a pointer to a physical memory page and mark it as used.
 *
 * @return unsigned long* pointer to a physical memory page, NULL if error
 * occurs.
 *
 * @pre There is no pre conditions.
 *
 * @post There is no post conditions.
 *
 * @see
 */
/* ========================================================================== */
unsigned long *dmm_get_phys_page(void);

/* ========================================================================== */
/**
 *  dmm_free_phys_page()
 *
 * @brief  Frees a specified physical memory page.
 *
 * @param physPgPtr - unsigned long* - [in] The address of the allocated
 * physical page that should be freed.
 *
 * @return errorCodeT
 *
 * @pre There is no pre conditions.
 *
 * @post There is no post conditions.
 *
 * @see errorCodeT for further detail.
 */
/* ========================================================================== */
enum errorCodeT dmm_free_phys_page(unsigned long *physPgPtr);

/* ========================================================================== */
/**
 *  dmm_pat_phy2virt_mapping()
 *
 * @brief  Mapping physical memory pages to virtual containers.
 *
 * @param bufferMappedZone - dmmTILERContPageAreaT* - [in] The tiled buffer
 * descriptor.
 *
 * @param custmPagesPtr - void * - [in] Pointer to the custom supplied
 * physical pages.
 *
 * @return errorCodeT
 *
 * @pre There is no pre conditions.
 *
 * @post There is no post conditions.
 *
 * @see errorCodeT, dmmTILERContPageAreaT for further detail.
 */
/* ========================================================================== */
enum errorCodeT dmm_pat_phy2virt_mapping(
        struct dmmTILERContPageAreaT *bufferMappedZone,
        void *custmPagesPtr);

/* ========================================================================== */
/**
 *  dmm_pat_start_refill()
 *
 * @brief  Creates a PAT area descriptor and starts the refill.
 *
 * @param bufferMappedZone - dmmTILERContPageAreaT* - [in] The tiled buffer
 * descriptor.
 *
 * @return errorCodeT
 *
 * @pre There is no pre conditions.
 *
 * @post There is no post conditions.
 *
 * @see errorCodeT, dmmTILERContPageAreaT for further detail.
 */
/* ========================================================================== */
enum errorCodeT dmm_pat_start_refill(
        struct dmmTILERContPageAreaT *bufferMappedZone);

/* ========================================================================== */
/**
 *  overlapping_test()
 *
 * @brief  Performs an area overlap test for errors. Debug only.
 *
 * @param tlrCtx - dmmTILERContCtxT* - [in] TILER context structure.
 *
 * @return int: 1 if overlapping is detected.
 *
 * @pre There is no pre conditions.
 *
 * @post There is no post conditions.
 *
 * @see
 */
/* ========================================================================== */
void overlapping_test(struct dmmTILERContCtxT *dmmTilerCtx);

#endif /* _DMM_PRV_H */

/*
 *  @(#) ti.sdo.tiler.linux; 1, 0, 0,10; 4-1-2010 17:09:40; /db/atree/library/trees/linuxutils/linuxutils-g08x/src/
 */

