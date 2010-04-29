/*
 *  Copyright 2010 by Texas Instruments Incorporated.
 *
 */

/*
 * dmm_hl_drv.c
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

#include <linux/module.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include <linux/hardirq.h>
#include <linux/mutex.h>
#include "dmm_def.h"
#include "dmm_2d_alloc.h"
#include "dmm_prv.h"
#include "tiler.h"

/* ========================================================================== */
/**
 *  dmm_pat_start_refill()
 *
 * @param dmmInstanceCtxPtr - void * - [in] Tiler context instance.
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
        struct dmmTILERContPageAreaT *bufferMappedZone)
{
        struct PATDescrT areaDesc;

        areaDesc.area.x0 = bufferMappedZone->x0 + bufferMappedZone->xPageOfst;
        areaDesc.area.y0 = bufferMappedZone->y0 + bufferMappedZone->yPageOfst;
        areaDesc.area.x1 = bufferMappedZone->x0 + bufferMappedZone->xPageOfst +
                           bufferMappedZone->xPageCount - 1;
        areaDesc.area.y1 = bufferMappedZone->y0 + bufferMappedZone->yPageOfst +
                           bufferMappedZone->yPageCount - 1;

        areaDesc.ctrl.direction = 0;
        areaDesc.ctrl.initiator = 0;
        areaDesc.ctrl.lutID = 0;
        areaDesc.ctrl.start = 1;
        areaDesc.ctrl.sync = 0;

        areaDesc.nextPatEntry = NULL;
        areaDesc.data = (unsigned long)bufferMappedZone->dma_pa;

        return dmm_pat_area_refill(&areaDesc, 0, MANUAL, 0);
}

/* ========================================================================== */
/**
 *  dmm_pat_phy2virt_mapping()
 *
 * @brief  Mapping physical memory pages to virtual containers.
 *
 * @param bufferMappedZone - dmmTILERContPageAreaT* - [in] The tiled buffer
 *  descriptor.
 *
 * @param custmPagesPtr - void * - [in] Pointer to the custom supplied physical
 *  pages.
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
        void *custmPagesPtr)
{
        unsigned long bfrPages;
        enum errorCodeT eCode = DMM_NO_ERROR;

        bfrPages =
                (bufferMappedZone->xPageCount)*(bufferMappedZone->yPageCount);

        if (bfrPages == 0) {
                eCode = DMM_SYS_ERROR;
        } else {
                bufferMappedZone->dma_size = bfrPages*4+16;

                bufferMappedZone->dma_va =
                        dma_alloc_coherent(NULL, bufferMappedZone->dma_size,
                        &(bufferMappedZone->dma_pa), GFP_ATOMIC);
                if (!bufferMappedZone->dma_va)
                        return DMM_SYS_ERROR;

                memset(bufferMappedZone->dma_va,
                        0x0, bufferMappedZone->dma_size);
                bufferMappedZone->patPageEntries =
                        (unsigned long *)((((unsigned long)
                                bufferMappedZone->dma_va) + 15) & ~15);

                if (dmm_tiler_populate_pat_page_entry_data(bfrPages,
                                NULL,
                                NULL,
                                (void *)bufferMappedZone->patPageEntries
                                                          ) != DMM_NO_ERROR) {
                        eCode = DMM_SYS_ERROR;
                        return eCode;
                }

                if (custmPagesPtr != NULL)
                        bufferMappedZone->patCustomPages = 1;
                else
                        bufferMappedZone->patCustomPages = 0;
                eCode = dmm_pat_start_refill(bufferMappedZone);
        }

        return eCode;
}

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
enum errorCodeT dmm_tiler_populate_pat_page_entry_data(unsigned long numPages,
                unsigned long **pageEntries,
                unsigned long **pageEntriesSpace,
                void *custmPagesPtr)
{
        signed long iter;
        unsigned long *patAreaEntries = NULL;

        patAreaEntries = (unsigned long *)custmPagesPtr;

        for (iter = 0; iter < numPages; iter++) {
                patAreaEntries[iter] =
                                (unsigned long)dmm_get_phys_page();
                if (patAreaEntries[iter] == 0x0)
                        return DMM_SYS_ERROR;
        }

        return DMM_NO_ERROR;
}

/* ========================================================================== */
/**
 *  dmm_tiler_swap_pat_page_entry_data()
 *
 * @brief  Swaps entries in an aray with PAT page address entries.
 *
 * @param bfrSize - unsigned long - [in] Size of the buffer which will be used
 * to generate page entries for.
 *
 * @param pageEntries - unsigned long* - [in] The address to the allocated page
 * entry data array, aligned due to hardware specification.
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
                unsigned short stride)
{
        unsigned short row;
        unsigned short column;
        unsigned long entrySwap;

        unsigned short startX = affectedArea->x0;
        unsigned short startY = affectedArea->y0;

        unsigned short endX = affectedArea->x1 + 1;
        unsigned short enxY = affectedArea->y1 + 1;

        signed short ofstX = destX - startX;
        signed short ofstY = destY - startY;

        for (row = startY; row < enxY; row++) {
                for (column = startX; column < endX; column++) {
                        if (row*stride+column > numPages ||
                                (row+ofstY)*stride+(column+ofstX) > numPages)
                                return DMM_WRONG_PARAM;
                        entrySwap = pageEntries[row*stride+column];
                        pageEntries[row*stride+column] =
                                pageEntries[(row+ofstY)*stride+(column+ofstX)];
                        pageEntries[(row+ofstY)*stride+(column+ofstX)] =
                                entrySwap;
                }
        }

        return DMM_NO_ERROR;
}

/* ========================================================================== */
/**
 *  dmm_tiler_container_map_area()
 *
 * @brief  Allocates a 2D TILER buffer - virtual 2D allocation,
 * descriptor and TILER system pointer updating.
 *
 * @param dmmTilerCtx - dmmTILERContCtxT* - [in] Tiler context instance.
 *
 * @param sizeWidth - unsigned short - [in] Width of buffer (in container
 * elements).
 *
 * @param sizeHeight - unsigned short - [in] Height of buffer
 * (in container elements).
 *
 * @param contMod - dmmMemoryAccessT - [in] Container access mode - for element
 * sizes.
 *
 * @param allocedPtr - void ** - [out] The allocated buffer system pointer is
 * provided through this double pointer. If no buffer is available this pointer
 * is set to NULL.
 *
 * @param bufferMappedZone - dmmTILERContPageAreaT** - [out] Description of the
 * 2D
 * area that is mapped in the TILER container is provided in this structure.
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
        struct dmmTILERContPageAreaT **bufferMappedZone)
{
        struct dmmTILERContPageAreaT areaRequest;

        enum errorCodeT eCode = DMM_NO_ERROR;
        signed long pageDimmensionX = 0;
        signed long pageDimmensionY = 0;
        unsigned long    accessMode = 0;
        unsigned long addrShiftAlign = 0;
        unsigned short tiled_pages_per_ss_page = 0;

        switch (contMod) {
        case MODE_8_BIT:
                accessMode = 0;
                pageDimmensionX = DMM_PAGE_DIMM_X_MODE_8;
                pageDimmensionY = DMM_PAGE_DIMM_Y_MODE_8;
                addrShiftAlign = DMM_HOR_Y_ADDRSHIFT_8;
                tiled_pages_per_ss_page = DMM_4KiB_SIZE / pageDimmensionX;
                break;
        case MODE_16_BIT:
                accessMode = 1;
                pageDimmensionX = DMM_PAGE_DIMM_X_MODE_16;
                pageDimmensionY = DMM_PAGE_DIMM_Y_MODE_16;
                addrShiftAlign = DMM_HOR_Y_ADDRSHIFT_16;
                tiled_pages_per_ss_page = DMM_4KiB_SIZE / pageDimmensionX / 2;
                break;
        case MODE_32_BIT:
                accessMode = 2;
                pageDimmensionX = DMM_PAGE_DIMM_X_MODE_32;
                pageDimmensionY = DMM_PAGE_DIMM_Y_MODE_32;
                addrShiftAlign = DMM_HOR_Y_ADDRSHIFT_32;
                tiled_pages_per_ss_page = DMM_4KiB_SIZE / pageDimmensionX / 4;
                break;
        case MODE_PAGE:
                accessMode = 3;
                pageDimmensionX = 64;/*DMM_PAGE_DIMM_X_MODE_8;*/ /* 64 */
                pageDimmensionY = 64;/*DMM_PAGE_DIMM_Y_MODE_8;*/ /* 64 */
                /* ((width + 4096 - 1) / 4096) */
                sizeWidth = ((sizeWidth + DMM_4KiB_SIZE - 1)/DMM_4KiB_SIZE);
                tiled_pages_per_ss_page = 1;

                /* for 1D blocks larger than the container width, we need to
                   allocate multiple rows */
                if (sizeWidth > dmmTilerCtx->contSizeX) {
                        sizeHeight = (sizeWidth + dmmTilerCtx->contSizeX - 1) /
                                                        dmmTilerCtx->contSizeX;
                        sizeWidth = dmmTilerCtx->contSizeX;
                } else {
                        sizeHeight = 1;
                }

                sizeHeight *= pageDimmensionX;
                sizeWidth  *= pageDimmensionY;

                addrShiftAlign = DMM_HOR_Y_ADDRSHIFT_8; /* 14 */
                break;
        default:
                eCode = DMM_WRONG_PARAM;
                break;
        }

        areaRequest.x1 = (sizeWidth + pageDimmensionX - 1) /
                                                        pageDimmensionX - 1;
        areaRequest.y1 = (sizeHeight + pageDimmensionY - 1) /
                                                        pageDimmensionY - 1;

        /* fill out to page boundaries */
        if (0)
                printk(KERN_ERR "areaRequest(%u (was %u%%%u) by %u)\n",
                             (areaRequest.x1 + 64) & ~63, areaRequest.x1 + 1,
                             tiled_pages_per_ss_page, areaRequest.y1 + 1);

        /* since all containers are collapsed, we need to take the most
           conservative value for pages per SS page */
        tiled_pages_per_ss_page = 64;
        areaRequest.x1 = ((areaRequest.x1 + tiled_pages_per_ss_page) &
                          ~(tiled_pages_per_ss_page - 1)) - 1;

        if (areaRequest.x1 > dmmTilerCtx->contSizeX ||
                        areaRequest.y1 > dmmTilerCtx->contSizeY) {
                eCode = DMM_WRONG_PARAM;
        }

        if (eCode == DMM_NO_ERROR) {
                *bufferMappedZone = alloc_2d_area(dmmTilerCtx, &areaRequest);
                /* if we could not allocate, we set the return code */
                if (*bufferMappedZone == NULL)
                        eCode = DMM_SYS_ERROR;
                /* else
                        printk(KERN_ERR "=>%d-%d,%d-%d\n",
                                (*bufferMappedZone)->x0,
                                (*bufferMappedZone)->x1,
                                (*bufferMappedZone)->y0,
                                (*bufferMappedZone)->y1);*/
        }

        /* DBG_OVERLAP_TEST(dmmTilerCtx); */
        if (eCode == DMM_NO_ERROR) {
                if (accessMode == 0) {
                        *allocedPtr =
                                DMM_COMPOSE_TILER_ALIAS_PTR(
                                (((*bufferMappedZone)->x0 << 6) |
                                ((*bufferMappedZone)->y0 << 20)), accessMode);
                } else if (accessMode == 3) {
                        *allocedPtr =
                                DMM_COMPOSE_TILER_ALIAS_PTR(
                                (((*bufferMappedZone)->x0 |
                                ((*bufferMappedZone)->y0 << 8)) << 12),
                                accessMode);
                } else {
                        *allocedPtr =
                                DMM_COMPOSE_TILER_ALIAS_PTR(
                                (((*bufferMappedZone)->x0 << 7) |
                                ((*bufferMappedZone)->y0 << 20)), accessMode);
                }
        } else {
                *allocedPtr = NULL;
        }
        return eCode;
}

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
        struct dmmTILERContPageAreaT *bufferMappedZone)
{
        if (dealloc_2d_area(dmmTilerCtx, bufferMappedZone) != 1)
                return DMM_WRONG_PARAM;
        else
                return DMM_NO_ERROR;
}

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
        struct dmmTILERContCtxT *dmmTilerCtx, void *sysPtr) {
        unsigned long X;
        unsigned long Y;
        enum dmmMemoryAccessT accessModeM;
        struct dmmTILERContPageAreaT *found = NULL;

        accessModeM = DMM_GET_ACC_MODE(sysPtr);

        if (DMM_GET_ROTATED(sysPtr) == 0) {
                if (accessModeM == MODE_PAGE) {
                        X = ((long)sysPtr & 0x7FFFFFF) >> 12;
                        Y = X / 256;
                        X = X & 255;
                } else if (accessModeM == MODE_8_BIT) {
                        X = DMM_HOR_X_PAGE_COOR_GET_8(sysPtr);
                        Y = DMM_HOR_Y_PAGE_COOR_GET_8(sysPtr);
                } else if (accessModeM == MODE_16_BIT) {
                        X = DMM_HOR_X_PAGE_COOR_GET_16(sysPtr);
                        Y = DMM_HOR_Y_PAGE_COOR_GET_16(sysPtr);
                } else if (accessModeM == MODE_32_BIT) {
                        X = DMM_HOR_X_PAGE_COOR_GET_32(sysPtr);
                        Y = DMM_HOR_Y_PAGE_COOR_GET_32(sysPtr);
                }
        } else {
                if (accessModeM == MODE_PAGE) {
                        X = ((long)sysPtr & 0x7FFFFFF) >> 12;
                        Y = X / 256;
                        X = X & 255;
                } else if (accessModeM == MODE_8_BIT) {
                        X = DMM_VER_X_PAGE_COOR_GET_8(sysPtr);
                        Y = DMM_VER_Y_PAGE_COOR_GET_8(sysPtr);
                } else if (accessModeM == MODE_16_BIT) {
                        X = DMM_VER_X_PAGE_COOR_GET_16(sysPtr);
                        Y = DMM_VER_Y_PAGE_COOR_GET_16(sysPtr);
                } else if (accessModeM == MODE_32_BIT) {
                        X = DMM_VER_X_PAGE_COOR_GET_32(sysPtr);
                        Y = DMM_VER_Y_PAGE_COOR_GET_32(sysPtr);
                }
        }

        /* printk(KERN_ERR " ? %p => x=%ld,y=%ld\n", sysPtr, X, Y); */
        found = search_2d_area(dmmTilerCtx, X, Y, DMM_GET_X_INVERTED(sysPtr),
                                                DMM_GET_Y_INVERTED(sysPtr));
        if (found) {
                /*printk(KERN_ERR " >(x=%d-%d=%d+%d,y=%d-%d=%d+%d)\n",
                found->x0, found->x1, found->xPageCount,
                                                        found->xPageOfst,
                found->y0, found->y1, found->yPageCount,
                                                        found->yPageOfst);*/
        }
        return found;
}

/* ========================================================================== */
/**
 *  dmm_instance_init()
 *
 * @brief  Initializes the Tiler cotnext.
 *
 * @param dmmInstanceCtxPtr - void * - [in] Tiler context instance.
 *
 * @param contXSize - signed long - [in] Tiler container width.
 *
 * @param contYSize - signed long - [in] Tiler container height.
 *
 * @param hMSP - void * - [in] MSP handle related to this dmm_drv cotnext.
 *
 * @param usrAppData - void * - [in] Pointer to user specific data structure.
 *
 * @param usrCallback - MSP_usrCallback - [in] Pointer to callback supplied by
 * the user for notificiation events (interupts).
 *
 * @return int True if operation succeded.
 *
 * @pre There is no pre conditions.
 *
 * @post There is no post conditions.
 *
 * @see dmmTILERContCtxT for further detail.
 */
/* ========================================================================== */
int dmm_instance_init(void *dmmInstanceCtxPtr,
                      signed long contXSize,
                      signed long contYSize,
                      void *hMSP,
                      void *usrAppData)
{
        struct dmmTILERContCtxT *dmmTilerCtx =
                &((struct dmmInstanceCtxT *)dmmInstanceCtxPtr)->dmmTilerCtx;
        struct dmmHwdCtxT *dmmHwdCtx =
                &((struct dmmInstanceCtxT *)dmmInstanceCtxPtr)->dmmHwdCtx;

        if (contXSize > 256 || contYSize > 128)
                return 0;

        if (dmmHwdCtx->dmmOpenInstances == 0) {

                dmmTilerCtx->usdArList = NULL;

                dmmTilerCtx->tmpArSelect.plmntAr.x0 = 0;
                dmmTilerCtx->tmpArSelect.plmntAr.y0 = 0;
                dmmTilerCtx->tmpArSelect.plmntAr.x1 = 0;
                dmmTilerCtx->tmpArSelect.plmntAr.y1 = 0;
                dmmTilerCtx->tmpArSelect.anchrAr = NULL;

                dmmTilerCtx->contSizeX = contXSize;
                dmmTilerCtx->contSizeY = contYSize;
                mutex_init(&dmmTilerCtx->mtx);

                dmmHwdCtx->patIrqEvnt0.irqAreaSelect = 0;
                dmmHwdCtx->patIrqEvnt1.irqAreaSelect = 1;
                dmmHwdCtx->patIrqEvnt2.irqAreaSelect = 2;
                dmmHwdCtx->patIrqEvnt3.irqAreaSelect = 3;

                if (dmm_phys_page_rep_init() != DMM_NO_ERROR)
                        return 0;
        }

        dmmHwdCtx->dmmOpenInstances++;

        return 1;
}

/* ========================================================================== */
/**
 *  dmm_instance_deinit()
 *
 * @brief  Deinitializes the Tiler cotnext.
 *
 * @param dmmInstanceCtxPtr - void * - [in] Tiler context instance.
 *
 * @return int True if operation succeded.
 *
 * @pre There is no pre conditions.
 *
 * @post There is no post conditions.
 *
 * @see dmmTILERContCtxT for further detail.
 */
/* ========================================================================== */
int dmm_instance_deinit(void *dmmInstanceCtxPtr)
{
        struct dmmTILERContCtxT *dmmTilerCtx =
                &((struct dmmInstanceCtxT *)dmmInstanceCtxPtr)->dmmTilerCtx;
        struct dmmHwdCtxT *dmmHwdCtx =
                &((struct dmmInstanceCtxT *)dmmInstanceCtxPtr)->dmmHwdCtx;
        struct MSP_Dmm_eventNotificationT *dmmMspCtx =
                &((struct dmmInstanceCtxT *)dmmInstanceCtxPtr)->dmmMspCtx;
        dmmHwdCtx->dmmOpenInstances--;
        if (dmmHwdCtx->dmmOpenInstances < 0)
                return 0;

        if (dmmHwdCtx->dmmOpenInstances == 0) {
                while (dmmTilerCtx->usdArList != NULL) {
                        if (dmm_tiler_container_unmap_area(
                                        dmmTilerCtx,
                                        &(dmmTilerCtx->usdArList->pgAr)) !=
                                        DMM_NO_ERROR)
                                return 0;
                }

                dmmTilerCtx->usdArList = NULL;

                dmmTilerCtx->tmpArSelect.plmntAr.x0 = 0;
                dmmTilerCtx->tmpArSelect.plmntAr.y0 = 0;
                dmmTilerCtx->tmpArSelect.plmntAr.x1 = 0;
                dmmTilerCtx->tmpArSelect.plmntAr.y1 = 0;
                dmmTilerCtx->tmpArSelect.anchrAr = NULL;

                dmmTilerCtx->contSizeX = 0;
                dmmTilerCtx->contSizeY = 0;

                dmmMspCtx->hMSP = NULL;
                dmmMspCtx->usrAppData = NULL;
        }

        return 1;
}

/* ========================================================================== */
/**
 *  dmm_copy2tiler_alias_view()
 *
 * @brief  Auxiliary function for copying data to the Tiler alias view.
 *
 * @param destPtr - void * - [in] Destination pointer in Tiler alias view.
 *
 * @param srcPtr - void * - [in] Data source pointer.
 *
 * @param width - signed long - [in] Data width.
 *
 * @param height - signed long - [in] Data height.
 *
 * @param stride - signed long - [in] Data stride.
 *
 * @param accType - dmmMemoryAccessT - [in] Tiler memory view access type.
 *
 * @return errorCodeT error if event can't be signaled.
 *
 * @pre There is no pre conditions.
 *
 * @post There is no post conditions.
 *
 * @see dmmTILERContCtxT for further detail.
 */
/* ========================================================================== */
enum errorCodeT dmm_copy2tiler_alias_view(void *destPtr,
                void *srcPtr,
                signed long width,
                signed long height,
                signed long stride,
                enum dmmMemoryAccessT accType)
{
        signed long row;

        switch (accType) {
        case MODE_8_BIT: {
                unsigned char *destPtr8 = (unsigned char *)destPtr;
                unsigned char *srcPtr8 = (unsigned char *)srcPtr;
                for (row = 0; row < height; row++) {
                        memcpy(&destPtr8[row*DMM_TILER_CONT_WIDTH_8],
                               srcPtr8, width);
                        srcPtr8 += stride;
                }
        }
        break;
        case MODE_16_BIT: {
                unsigned short *destPtr16 = (unsigned short *)destPtr;
                unsigned short *srcPtr16 = (unsigned short *)srcPtr;
                for (row = 0; row < height; row++) {
                        memcpy(&destPtr16[row*DMM_TILER_CONT_WIDTH_16],
                               srcPtr16, width*2);
                        srcPtr16 += stride;
                }
        }
        break;
        case MODE_32_BIT: {
                unsigned long *destPtr32 = (unsigned long *)destPtr;
                unsigned long *srcPtr32 = (unsigned long *)srcPtr;
                for (row = 0; row < height; row++) {
                        memcpy(&destPtr32[row*DMM_TILER_CONT_WIDTH_32],
                               srcPtr32, width*4);
                        srcPtr32 += stride;
                }
        }
        break;
        case MODE_PAGE: {
                unsigned char *destPtr8 = (unsigned char *)destPtr;
                unsigned char *srcPtr8 = (unsigned char *)srcPtr;
                for (row = 0; row < stride*height; row++) {
                        memcpy(&destPtr8[row], srcPtr8, stride);
                        srcPtr8 += stride;
                }
        }
        break;
        default:
                return DMM_SYS_ERROR;
        }

        return DMM_NO_ERROR;
}

/* ========================================================================== */
/**
 *  dmm_copy_from_tiler_alias_view()
 *
 * @brief  Auxiliary function for copying data to the Tiler alias view.
 *
 * @param destPtr - void * - [in] Destination pointer in Tiler alias view.
 *
 * @param srcPtr - void * - [in] Data source pointer.
 *
 * @param width - signed long - [in] Data width.
 *
 * @param height - signed long - [in] Data height.
 *
 * @param stride - signed long - [in] Data stride.
 *
 * @param accType - dmmMemoryAccessT - [in] Tiler memory view access type.
 *
 * @return errorCodeT error if event can't be signaled.
 *
 * @pre There is no pre conditions.
 *
 * @post There is no post conditions.
 *
 * @see dmmTILERContCtxT for further detail.
 */
/* ========================================================================== */
enum errorCodeT dmm_copy_from_tiler_alias_view(void *destPtr,
                void *srcPtr,
                signed long width,
                signed long height,
                signed long stride,
                enum dmmMemoryAccessT accType)
{
        signed long row;

        switch (accType) {
        case MODE_8_BIT: {
                unsigned char *destPtr8 = (unsigned char *)destPtr;
                unsigned char *srcPtr8 = (unsigned char *)srcPtr;
                for (row = 0; row < height; row++) {
                        memcpy(destPtr8, &srcPtr8[row*DMM_TILER_CONT_WIDTH_8],
                               width);
                        destPtr8 += stride;
                }
        }
        break;
        case MODE_16_BIT: {
                unsigned short *destPtr16 = (unsigned short *)destPtr;
                unsigned short *srcPtr16 = (unsigned short *)srcPtr;
                for (row = 0; row < height; row++) {
                        memcpy(destPtr16,
                               &srcPtr16[row*DMM_TILER_CONT_WIDTH_16],
                               width*2);
                        destPtr16 += stride;
                }
        }
        break;
        case MODE_32_BIT: {
                unsigned long *destPtr32 = (unsigned long *)destPtr;
                unsigned long *srcPtr32 = (unsigned long *)srcPtr;
                for (row = 0; row < height; row++) {
                        memcpy(destPtr32,
                               &srcPtr32[row*DMM_TILER_CONT_WIDTH_32],
                               width*4);
                        destPtr32 += stride;
                }
        }
        break;
        case MODE_PAGE: {
                unsigned char *destPtr8 = (unsigned char *)destPtr;
                unsigned char *srcPtr8 = (unsigned char *)srcPtr;
                for (row = 0; row < stride*height; row++) {
                        memcpy(destPtr8, &srcPtr8[row], stride);
                        destPtr8 += stride;
                }
        }
        break;
        default:
                return DMM_SYS_ERROR;
        }

        return DMM_NO_ERROR;
}

/* ========================================================================== */
/**
 *  dmmVirtualBufferManipulations()
 *
 * @brief  Manipulates virtual buffers.
 *
 * @param dmmInstanceCtxPtr - void * - [in] Dmm context instance.
 *
 * @param sysPtr - void * - [in] Tiler system pointer to a 2D area.
 *
 * @param patOp - MSP_Dmm_Phy2VirtOpsT - [in] Refill operaion to perform.
 *
 * @param affectedArea - PATAreaT* - [in] Area that will be affected.
 *
 * @param destinationArea - PATAreaT* - [in] Destination coordinates.
 *
 * @return void * pointer to the area targeted by the operation.
 *
 * @pre There is no pre conditions.
 *
 * @post There is no post conditions.
 *
 * @see dmmTILERContPageAreaT for further detail.
 */
/* ========================================================================== */
void *dmm_virtual_buffer_manipulations(void *dmmInstanceCtxPtr,
                                       void *sysPtr,
                                       struct PATAreaT *affectedArea,
                                       struct PATAreaT *destinationArea)
{
        unsigned long bfrPages = 0x0;
        struct dmmTILERContPageAreaT *bufferMappedZone = NULL;
        enum errorCodeT eCode = DMM_NO_ERROR;
        struct dmmTILERContCtxT *dmmTilerCtx =
                &((struct dmmInstanceCtxT *)dmmInstanceCtxPtr)->dmmTilerCtx;

        enum dmmMemoryAccessT accessModeM = -1;
        struct dmmViewOrientT orient = {0};
        unsigned long addrAlignment = 0x0;
        unsigned long contWidth = 0x0;
        unsigned long contHeight = 0x0;

        bufferMappedZone = dmm_tiler_get_area_from_sysptr(dmmTilerCtx, sysPtr);
        bfrPages =
                (bufferMappedZone->xPageCount)*(bufferMappedZone->yPageCount);

        eCode = dmm_tiler_swap_pat_page_entry_data(bfrPages,
                        bufferMappedZone->patPageEntries, affectedArea,
                        destinationArea->x0, destinationArea->y0,
                        bufferMappedZone->xPageCount);

        eCode = dmm_pat_start_refill(bufferMappedZone);

        if (eCode != DMM_NO_ERROR)
                return NULL;

        accessModeM = DMM_GET_ACC_MODE(sysPtr);
        orient.dmm90Rotate = (unsigned char)DMM_GET_ROTATED(sysPtr);
        orient.dmmXInvert = (unsigned char)DMM_GET_X_INVERTED(sysPtr);
        orient.dmmYInvert = (unsigned char)DMM_GET_Y_INVERTED(sysPtr);

        switch (accessModeM) {
        case MODE_8_BIT:
        case MODE_PAGE:
                if (orient.dmm90Rotate) {
                        contWidth = DMM_TILER_CONT_HEIGHT_8;
                        contHeight = DMM_TILER_CONT_WIDTH_8;
                } else {
                        contWidth = DMM_TILER_CONT_WIDTH_8;
                        contHeight = DMM_TILER_CONT_HEIGHT_8;
                }
                addrAlignment = 0;
                break;
        case MODE_16_BIT:
                if (orient.dmm90Rotate) {
                        contWidth = DMM_TILER_CONT_HEIGHT_16;
                        contHeight = DMM_TILER_CONT_WIDTH_16;
                } else {
                        contWidth = DMM_TILER_CONT_WIDTH_16;
                        contHeight = DMM_TILER_CONT_HEIGHT_16;
                }
                addrAlignment = 1;
                break;
        case MODE_32_BIT:
                if (orient.dmm90Rotate) {
                        contWidth = DMM_TILER_CONT_HEIGHT_32;
                        contHeight = DMM_TILER_CONT_WIDTH_32;
                } else {
                        contWidth = DMM_TILER_CONT_WIDTH_32;
                        contHeight = DMM_TILER_CONT_HEIGHT_32;
                }
                addrAlignment = 2;
                break;
        }

        if (orient.dmm90Rotate) {
                sysPtr = (void *)((destinationArea->x0*contHeight +
                                destinationArea->y0) <<
                                                ((unsigned long)addrAlignment));
        } else {
                sysPtr = (void *)((destinationArea->y0*contWidth +
                                destinationArea->x0) <<
                                                ((unsigned long)addrAlignment));
        }
        sysPtr = DMM_COMPOSE_TILER_PTR(sysPtr, orient.dmm90Rotate,
                                orient.dmmYInvert,
                                                orient.dmmXInvert, accessModeM);

        return sysPtr;
}

/*
 *  @(#) ti.sdo.tiler.linux; 1, 0, 0,10; 4-1-2010 17:09:40; /db/atree/library/trees/linuxutils/linuxutils-g08x/src/
 */

