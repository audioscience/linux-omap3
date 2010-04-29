/*
 *  Copyright 2010 by Texas Instruments Incorporated.
 *
 */

/*
 * dmm_def.h
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

#ifndef _DMM_DEF_H
#define _DMM_DEF_H

#include "tiler.h"

#define DMM_4KiB_SIZE                   (4096)
#define DMM_TILER_CONT_WIDTH_8          (16384)
#define DMM_TILER_CONT_WIDTH_16         (16384)
#define DMM_TILER_CONT_WIDTH_32         (8192)
#define DMM_TILER_CONT_HEIGHT_8         (8192)
#define DMM_TILER_CONT_HEIGHT_16        (4096)
#define DMM_TILER_CONT_HEIGHT_32        (4096)

/** @struc MSP_Dmm_eventNotificationT
* Structure defining Dmm driver context for user event notification
* (user callback and application specific data pointer. */
struct MSP_Dmm_eventNotificationT {
        void *hMSP;
        void *usrAppData;
};

/** @enum errorCodeT
* Defining enumarated identifiers for general dmm driver errors. */
enum errorCodeT {
        DMM_NO_ERROR,
        DMM_WRONG_PARAM,
        DMM_HRDW_CONFIG_FAILED,
        DMM_HRDW_NOT_READY,
        DMM_SYS_ERROR
};

/** @enum dmmPATStatusErrT
* Defining enumarated identifiers for PAT area status error field. */
enum dmmPATStatusErrT {
        NO_ERROR                                = 0x0,
        INVALID_DESCR                   = 0x1,
        INVALID_DATA_PTR                = 0x2,
        UNEXP_AREA_UPDATE               = 0x4,
        UNEXP_CONTROL_UPDATE    = 0x8,
        UNEXP_DATA_UPDATE               = 0x10,
        UNEXP_ACCESS                    = 0x20
};

/** @struc dmmPATStatusT
* Structure defining PAT area status. */
struct dmmPATStatusT {
        enum dmmPATStatusErrT error;
        unsigned char ready;
        unsigned char validDescriptor;
        unsigned char engineRunning;
        unsigned char done;
        unsigned char linkedReconfig;
        unsigned char remainingLinesCounter;
};

/** @enum dmmMemSectionSizeT
* Defining enumarated identifiers for memory section sizes used by LISA. */
enum dmmMemSectionSizeT {
        SEC_16MiB,
        SEC_32MiB,
        SEC_64MiB,
        SEC_128MiB,
        SEC_256MiB,
        SEC_512MiB,
        SEC_1GiB,
        SEC_2GiB
};

/** @enum dmmMemSdrcIntlModeT
* Defining enumarated identifiers for memory section interleaving
* used by LISA.*/
enum dmmMemSdrcIntlModeT {
        SEC_INTL_NONE,
        SEC_INTL_128B,
        SEC_INTL_256B,
        SEC_INTL_512B
};

/** @enum dmmMemSectionMappingT
* Defining enumarated identifiers for memory section mapping used by LISA. */
enum dmmMemSectionMappingT {
        SEC_UNMAPPED,
        SEC_MAPPED_SDRC0,
        SEC_MAPPED_SDRC1,
        SEC_MAPPED_INTL
};

/** @struc dmmLISAConfigT
* Structure defining LISA memory map configuration. */
struct dmmLISAConfigT {
        signed long lisaMemMapIndx;
        unsigned long sysAddr;
        enum dmmMemSectionSizeT sysSize;
        enum dmmMemSdrcIntlModeT sdrcIntl;
        unsigned long sdrcAddrspc;
        enum dmmMemSectionMappingT sdrcMap;
        unsigned long sdrcAddr;
};

/** @struc dmmLISAConfigLstT
* Structure defining LISA memory map configuration linked list. */
struct dmmLISAConfigLstT {
        struct dmmLISAConfigLstT *nextConf;
        struct dmmLISAConfigT mapConf;
};

/** @enum dmmMemoryAccessT
* Defining enumarated identifiers for memory memory access types through TILER
* and PAT. */
enum dmmMemoryAccessT {
        MODE_8_BIT,
        MODE_16_BIT,
        MODE_32_BIT,
        MODE_PAGE
};

/** @enum dmmPATTranslationT
* Defining enumarated identifiers for possible PAT address translation
* schemes. */
enum dmmPATTranslationT {
        DIRECT,
        INDIRECT
};

/** @enum dmmPATEngineAccessT
* Defining enumarated identifiers for possible PAT engines memory access
* schemes. */
enum dmmPATEngineAccessT {
        NORMAL_MODE,
        DIRECT_LUT
};

/** @enum dmmPATRefillMethodT
* Defining enumarated identifiers for possible PAT area refill methods. */
enum dmmPATRefillMethodT {
        MANUAL,
        AUTO
};

/** @struc PATAreaT
* Structure defining PAT page-area register. */
struct PATAreaT {
        int x0:8;
        int y0:8;
        int x1:8;
        int y1:8;
};

/** @struc PATCtrlT
* Structure defining PAT control register. */
struct PATCtrlT {
        int start:4;
        int direction:4;
        int lutID:8;
        int sync:12;
        int initiator:4;
};

/** @struc PATDescrT
* Structure defining PAT area descriptor, needed for area refill procedures. */
struct PATDescrT {
        struct PATDescrT        *nextPatEntry;
        struct PATAreaT area;
        struct PATCtrlT ctrl;
        unsigned long   data;
};

/** @struc dmmPATIrqEventsT
* Structure defining PAT interrupt events. */
struct dmmPATIrqEventsT {
        unsigned char irqAreaSelect;
        unsigned char lutMiss;
        unsigned char updData;
        unsigned char updCtrl;
        unsigned char updArea;
        unsigned char invData;
        unsigned char invDsc;
        unsigned char fillLst;
        unsigned char fillDsc;
};

/** @struc dmmPatIrqConfigT
* Structure defining PAT interrupt configuration. */
struct dmmPatIrqConfigT {
        struct dmmPATIrqEventsT irqEvnts;
        int clrEvents;
};

/** @struc dmmPATIrqConfigLstT
* Structure defining PAT interrupt configuration linked list. */
struct dmmPATIrqConfigLstT {
        struct dmmPATIrqConfigLstT *nextConf;
        struct dmmPatIrqConfigT irqConf;
};

/** @struc dmmPATEngineConfigT
* Structure defining PAT engine configuration. */
struct dmmPATEngineConfigT {
        signed long dmmPatEngineSel;
        enum dmmPATEngineAccessT engineMode;
} ;

/** @struc dmmPATEngineConfigLstT
* Structure defining PAT engine configuration linked list. */
struct dmmPATEngineConfigLstT {
        struct dmmPATEngineConfigLstT *nextConf;
        struct dmmPATEngineConfigT engineConf;
};

/** @struc dmmPATViewConfigT
* Structure defining PAT view configuration. */
struct dmmPATViewConfigT {
        signed long initiatorId;
        signed long viewIndex;
};

/** @struc dmmPATViewConfigLstT
* Structure defining PAT alias view configuration linked list. */
struct dmmPATViewConfigLstT {
        struct dmmPATViewConfigLstT *nextConf;
        struct dmmPATViewConfigT aliasViewConf;
};

/** @struc dmmPATViewMapConfigT
* Structure defining PAT view map configuration. */
struct dmmPATViewMapConfigT {
        unsigned long patViewMapIndx;
        enum dmmMemoryAccessT memoryAccessMode;
        unsigned long contX;
        enum dmmPATTranslationT transType;
        unsigned long dmmPATViewBase;
};

/** @struc dmmPATViewMapConfigLstT
* Structure defining PAT view map configuration linked list. */
struct dmmPATViewMapConfigLstT {
        struct dmmPATViewMapConfigLstT *nextConf;
        struct dmmPATViewMapConfigT viewConf;
};

/** @struc dmmTILERConfigT
* Structure defining TILER alias view configuration. */
struct dmmTILERConfigT {
        signed long initiatorId;
        struct dmmViewOrientT orient;
};

/** @struc dmmTILERConfigLstT
* Structure defining TILER alias view configuration linked list. */
struct dmmTILERConfigLstT {
        struct dmmTILERConfigLstT *nextConf;
        struct dmmTILERConfigT aliasConf;
};

/** @struc dmmPEGConfigT
* Structure defining PEG priority configuration. */
struct dmmPEGConfigT {
        signed long initiatorId;
        unsigned long prio;
};

/** @struc dmmPEGConfigLstT
* Structure defining PEG priority configuration linked list. */
struct dmmPEGConfigLstT {
        struct dmmPEGConfigLstT *nextConf;
        struct dmmPEGConfigT prioConf;
};

/** @struc dmmPATStatusLstT
* Structure defining PAT area status linked list. */
struct dmmPATStatusLstT {
        struct dmmPATStatusLstT *nextConf;
        signed long areaSelect;
        struct dmmPATStatusT patAreaStatus;
};

/** @enum MSP_Dmm_Phy2VirtOpsT
* Defining all of the control commands related to physical to
* virtual transforms. */
/*enum MSP_Dmm_Phy2VirtOpsT
{
        DMM_P2V_SWAP
}*/

#endif /* _DMM_DEF_H */

/*
 *  @(#) ti.sdo.tiler.linux; 1, 0, 0,10; 4-1-2010 17:09:40; /db/atree/library/trees/linuxutils/linuxutils-g08x/src/
 */

