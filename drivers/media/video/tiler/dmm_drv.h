/*
 *  Copyright 2010 by Texas Instruments Incorporated.
 *
 */

/*
 * dmm_drv.h
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

#ifndef _DMM_DRV_H
#define _DMM_DRV_H

#include "dmm_def.h"

/* ========================================================================== */
/**
 *  dmm_pat_area_refill()
 *
 * @brief  Initiate a PAT area refill (or terminate an ongoing - consult
 * documentation).
 *
 * @param patDesc - PATDescrT* - [in] Pointer to a PAT area descriptor that's
 * needed to extract settings from for the refill procedure initation.
 *
 * @param dmmPatAreaSel - signed long - [in] Selects which PAT area will be
 *  configured for a area refill procedure.
 *
 * @param refillType - dmmPATRefillMethodT - [in] Selects the refill method -
 * manual or automatic.
 *
 * @param forcedRefill - int - [in] Selects if forced refill should be used
 * effectively terminating any ongoing area refills related to the selected
 * area.
 *
 * @return errorCodeT
 *
 * @pre If forced mode is not used, no refills should be ongoing for the
 * selected
 * area - error status returned if this occurs.
 *
 * @post If non valid data is provided for patDesc and the refill engines fail
 * to perform the request, an error status is returned.
 *
 * @see errorCodeT,  PATDescrT, dmmPATRefillMethodT, int for further detail.
 */
/* ========================================================================== */
enum errorCodeT dmm_pat_area_refill(struct PATDescrT *patDesc,
                                    signed long dmmPatAreaSel,
                                    enum dmmPATRefillMethodT refillType,
                                    int forcedRefill);

/* ========================================================================== */
/**
 *  dmm_pat_refill_area_status_get()
 *
 * @brief  Gets the status for the selected PAT area.
 *
 * @param dmmPatAreaSel - signed long - [in] Selects which PAT area status will
 *  be queried.
 *
 * @param areaStatus - dmmPATStatusT* - [out] Structure containing the PAT area
 * status that will be filled by dmmPatRefillAreaStatusGet().
 *
 * @return errorCodeT
 *
 * @pre There is no pre conditions.
 *
 * @post If the query fails the provided areaStatus structure is not updated at
 * all!
 *
 * @see errorCodeT, dmmPATStatusT for further detail.
 */
/* ========================================================================== */
enum errorCodeT dmm_pat_refill_area_status_get(signed long dmmPatAreaStatSel,
                struct dmmPATStatusT *areaStatus);

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
 * @param hMSP - MSP_HANDLE - [in] MSP handle related to this dmm_drv cotnext.
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
                      void *usrAppData);

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
int dmm_instance_deinit(void *dmmInstanceCtxPtr);

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
                enum dmmMemoryAccessT accType);

/* ========================================================================== */
/**
 *  dmm_virtual_buffer_manipulations()
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
                                       struct PATAreaT *destinationArea);

#endif /* _DMM_DRV_H */

/*
 *  @(#) ti.sdo.tiler.linux; 1, 0, 0,10; 4-1-2010 17:09:40; /db/atree/library/trees/linuxutils/linuxutils-g08x/src/
 */

