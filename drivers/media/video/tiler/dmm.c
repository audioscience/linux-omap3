/*
 *  Copyright 2010 by Texas Instruments Incorporated.
 *
 */

/*
 * dmm.c
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include "tiler.h"
#include "dmm_drv.h"
#include "dmm_prv.h"
#include "dmm_def.h"

#define DMM_MAJOR 0
#define DMM_MINOR 0

void *dmm_base;
static struct dmmInstanceCtxT *ctxptr;

struct node {
        struct tiler_buf_info *ptr;
        unsigned long reserved;
        struct node *nextnode;
};

static struct node *lsthd;
static int id;

static int dmm_open(struct inode *i, struct file *f);
static int dmm_release(struct inode *i, struct file *f);
static int dmm_ioctl(struct inode *i, struct file *f,
                     unsigned int c, unsigned long a);
static int dmm_mmap(struct file *f, struct vm_area_struct *v);
static void dmm_vma_open(struct vm_area_struct *vma);
static void dmm_vma_close(struct vm_area_struct *vma);
static int removenode(struct node *listhead, int offset);
static int tiler_destroy_buf_info_list(struct node *listhead);
static int addnode(struct node *listhead, struct tiler_buf_info *p);
static int createlist(struct node **listhead);
static int tiler_find_buf(unsigned long sysptr, struct tiler_block_info *blk);

static int
tiler_get_buf_info(struct node *listhead, struct tiler_buf_info **pp, int ofst);

static void pat_view_set(void);
static void pat_view_map_set(void);
static void pat_view_map_base_set(void);
static void tiler_or_set(void);

static int dmm_major;
static int dmm_minor;

static struct vm_operations_struct dmm_remap_vm_ops = {
        .open =  dmm_vma_open,
        .close = dmm_vma_close,
};

struct dmm_dev {
        struct cdev cdev;
};

static struct dmm_dev *dmm_device;
static struct class *dmmdev_class;
static const struct file_operations dmm_fops = {
        .open    = dmm_open,
        .ioctl   = dmm_ioctl,
        .release = dmm_release,
        .mmap    = dmm_mmap,
};

static struct platform_driver tiler_driver_ldm = {
        .driver = {
                .owner = THIS_MODULE,
                .name = "tiler",
        },
        .probe = NULL,
        .shutdown = NULL,
        .remove = NULL,
};

static int
__init dmm_init(void)
{
        dev_t dev  = 0;
        int retval = -1;
        int error = -1;
        struct device *device = NULL;
        void __iomem *reg = NULL;

        if (dmm_major) {
                dev = MKDEV(dmm_major, dmm_minor);
                retval = register_chrdev_region(dev, 1, "tiler");
        } else {
                retval = alloc_chrdev_region(&dev, dmm_minor, 1, "tiler");
                dmm_major = MAJOR(dev);
        }

        dmm_device = kmalloc(sizeof(struct dmm_dev), GFP_KERNEL);
        if (!dmm_device) {
                retval = -ENOMEM;
                unregister_chrdev_region(dev, 1);
                printk(KERN_ERR "kmalloc():failed\n");
                goto EXIT;
        }
        memset(dmm_device, 0x0, sizeof(struct dmm_dev));
        cdev_init(&dmm_device->cdev, &dmm_fops);
        dmm_device->cdev.owner = THIS_MODULE;
        dmm_device->cdev.ops   = &dmm_fops;

        retval = cdev_add(&dmm_device->cdev, dev, 1);
        if (retval)
                printk(KERN_ERR "cdev_add():failed\n");

        dmmdev_class = class_create(THIS_MODULE, "tiler");

        if (IS_ERR(dmmdev_class)) {
                printk(KERN_ERR "class_create():failed\n");
                goto EXIT;
        }

        device = device_create(dmmdev_class, NULL, dev, NULL, "tiler");
        if (device == NULL)
                printk(KERN_ERR "device_create() fail\n");

        retval = platform_driver_register(&tiler_driver_ldm);

        ctxptr = kmalloc(sizeof(struct dmmInstanceCtxT), GFP_KERNEL);
        if (!ctxptr)
                return -ENOMEM;
        memset(ctxptr, 0x0, sizeof(struct dmmInstanceCtxT));
        error = dmm_instance_init((void *)ctxptr, TILER_WIDTH,
                                  TILER_HEIGHT, NULL, NULL);
        if (error == 1) {
                retval = 0;
        } else {
                mutex_destroy(&(ctxptr->dmmTilerCtx.mtx));
                kfree(ctxptr);
                return retval;
        }

        dmm_base = ioremap(DMM_BASE, 0x1000);

        reg = (void __iomem *)(
                        (unsigned long)dmm_base | (unsigned long)LISA_MAP__0);
        regdump("LISA_MAP__0", __raw_readl(reg));

        reg = (void __iomem *)(
                        (unsigned long)dmm_base | (unsigned long)LISA_MAP__1);
        regdump("LISA_MAP__1", __raw_readl(reg));

        pat_view_set();
        pat_view_map_set();
        pat_view_map_base_set();
        tiler_or_set();

        /* create buffer info list */
        createlist(&lsthd);
        id = 0xda7a000;

EXIT:
        return retval;
}

static void
__exit dmm_exit(void)
{
        mutex_destroy(&(ctxptr->dmmTilerCtx.mtx));
        kfree(ctxptr);
        tiler_destroy_buf_info_list(lsthd);

        platform_driver_unregister(&tiler_driver_ldm);

        cdev_del(&dmm_device->cdev);
        kfree(dmm_device);

        device_destroy(dmmdev_class, MKDEV(dmm_major, dmm_minor));
        class_destroy(dmmdev_class);

        iounmap(dmm_base);
}

static int
dmm_open(struct inode *ip, struct file *filp)
{
        int retval = -1;
        retval = 0;
        return retval;
}

static int
dmm_release(struct inode *ip, struct file *filp)
{
        int retval = -1;
        retval = 0;
        return retval;
}

static int
dmm_ioctl(struct inode *ip, struct file *filp, unsigned int cmd,
                unsigned long arg)
{
        struct tiler_buf_info *bufinfo = NULL;
        void *ptr = NULL;
        int retval = -1;
        int error = -1;
        pgd_t *pgd = NULL;
        pmd_t *pmd = NULL;
        pte_t *ptep = NULL, pte = 0x0;
        unsigned long bytes = 0x1;
        int offset = 0x0;
        unsigned long ssptr = 0x0;

        struct tiler_buf_info buf_info = {0};
        struct tiler_block_info block_info = {0};

        switch (cmd) {
        case TILIOC_OPEN:
                retval = 0;
                break;
        case TILIOC_CLOSE:
                retval = 0;
                break;
        case TILIOC_GBUF:
                bytes = copy_from_user((void *)(&block_info),
                        (const void *)arg, sizeof(struct tiler_block_info));
                if (bytes != 0)
                        return retval;

                if (block_info.fmt == TILFMT_PAGE) {
                        error = tiler_alloc_buf(block_info.fmt,
                                                block_info.dim.len,
                                                1,
                                                &ptr);
                } else {
                        error = tiler_alloc_buf(block_info.fmt,
                                                block_info.dim.area.width,
                                                block_info.dim.area.height,
                                                &ptr);
                }

                if (error != 0)
                        return retval;

                block_info.ssptr = (unsigned long)ptr;
                bytes = copy_to_user((void *)arg, (const void *)(&block_info),
                                        sizeof(struct tiler_block_info));
                if (bytes != 0)
                        return retval;

                retval = 0;
                break;
        case TILIOC_FBUF:
                bytes = copy_from_user((void *)(&block_info),
                        (const void *)arg, sizeof(struct tiler_block_info));
                if (bytes != 0)
                        return retval;

                error = tiler_free_buf(block_info.ssptr);
                if (error == 0)
                        retval = 0;
                break;
        case TILIOC_GSSP:
                pgd = pgd_offset(current->mm, arg);
                if (!(pgd_none(*pgd) || pgd_bad(*pgd))) {
                        pmd = pmd_offset(pgd, arg);
                        if (!(pmd_none(*pmd) || pmd_bad(*pmd))) {
                                ptep = pte_offset_map(pmd, arg);
                                if (ptep) {
                                        pte = *ptep;
                                        if (pte_present(pte)) {
                                                ssptr = (pte & PAGE_MASK)
                                                        | (~PAGE_MASK & arg);
                                                retval = ssptr;
                                                break;
                                        }
                                }
                        }
                }
                retval = 0x0; /* va not in page table */
                break;
        case TILIOC_MBUF:
                retval = 0;
                break;
        case TILIOC_QBUF:
                bytes = copy_from_user((void *)(&buf_info),
                        (const void *)arg, sizeof(struct tiler_buf_info));
                if (bytes != 0)
                        return retval;

                offset = buf_info.offset;
                error = tiler_get_buf_info(lsthd, &bufinfo, offset);
                if (error != 0)
                        return retval;

                bytes = copy_to_user((void *)arg,
                        (const void *)bufinfo, sizeof(struct tiler_buf_info));
                if (bytes != 0)
                        return retval;

                retval = 0;
                break;
        case TILIOC_RBUF:
                bufinfo = kmalloc(sizeof(struct tiler_buf_info), GFP_KERNEL);
                if (!bufinfo)
                        return retval;
                memset(bufinfo, 0x0, sizeof(struct tiler_buf_info));
                bytes = copy_from_user((void *)bufinfo,
                        (const void *)arg, sizeof(struct tiler_buf_info));
                if (bytes != 0)
                        return retval;

                bufinfo->offset = id;
                id += 0x1000;

                bytes = copy_to_user((void *)arg,
                        (const void *)bufinfo, sizeof(struct tiler_buf_info));
                if (bytes != 0)
                        return retval;

                error = addnode(lsthd, bufinfo);
                if (error != 0)
                        return retval;
                retval = 0;
                break;
        case TILIOC_URBUF:
                bytes = copy_from_user((void *)(&buf_info),
                        (const void *)arg, sizeof(struct tiler_buf_info));
                if (bytes != 0)
                        return retval;

                offset = buf_info.offset;
                error = removenode(lsthd, offset);
                if (error != 0)
                        return retval;

                retval = 0;
                break;
        case TILIOC_QUERY_BLK:
                bytes = copy_from_user((void *)(&block_info),
                        (const void *)arg, sizeof(struct tiler_block_info));
                if (bytes != 0)
                        return retval;

                error = tiler_find_buf(block_info.ssptr, &block_info);
                if (error != 0)
                        return retval;

                bytes = copy_to_user((void *)arg,
                        (const void *)(&block_info),
                        sizeof(struct tiler_block_info));
                if (bytes != 0)
                        return retval;

                retval = 0;
                break;
        }
        return retval;
}

static int
dmm_mmap(struct file *filp, struct vm_area_struct *vma)
{
        int ret = -1;
        struct tiler_buf_info *b = NULL;
        int i = 0, j = 0, k = 0, m = 0, p = 0;
        int bpp = 1;

        vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

        ret = tiler_get_buf_info(lsthd, &b, vma->vm_pgoff << PAGE_SHIFT);
        if (ret != 0) {
                printk(KERN_ERR "%s::%s():%d: tiler_get_buf_info failed\n",
                                                __FILE__, __func__, __LINE__);
                return 0x0;
        }

        for (i = 0; i < b->num_blocks; i++) {
                if (b->blocks[i].fmt >= TILFMT_8BIT &&
                                        b->blocks[i].fmt <= TILFMT_32BIT) {
                        /* get line width */
                        bpp = (b->blocks[i].fmt == TILFMT_8BIT ? 1 :
                               b->blocks[i].fmt == TILFMT_16BIT ? 2 : 4);
                        p = (b->blocks[i].dim.area.width * bpp +
                                TILER_PAGESIZE - 1) & ~(TILER_PAGESIZE - 1);

                        for (j = 0; j < b->blocks[i].dim.area.height; j++) {
                                /* map each page of the line */
                                if (0)
                                        printk(KERN_NOTICE
                                        "%s::%s():%d: vm_start+%d = 0x%lx,"
                                        "blk[%d].ssptr+%d = 0x%lx, w=0x%x\n",
                                        __FILE__, __func__, __LINE__,
                                        k, vma->vm_start + k, i, m,
                                        (b->blocks[i].ssptr + m), p);
                                vma->vm_pgoff =
                                        (b->blocks[i].ssptr + m) >> PAGE_SHIFT;
                                if (remap_pfn_range(vma, vma->vm_start + k,
                                        (b->blocks[i].ssptr + m) >> PAGE_SHIFT,
                                        p, vma->vm_page_prot))
                                        return -EAGAIN;
                                k += p;
                                if (b->blocks[i].fmt == TILFMT_8BIT)
                                        m += 64*TILER_WIDTH;
                                else
                                        m += 2*64*TILER_WIDTH;
                        }
                        m = 0;
                } else if (b->blocks[i].fmt == TILFMT_PAGE) {
                        vma->vm_pgoff = (b->blocks[i].ssptr) >> PAGE_SHIFT;
                        p = (b->blocks[i].dim.len + TILER_PAGESIZE - 1) &
                                                        ~(TILER_PAGESIZE - 1);
                        if (0)
                                printk(KERN_NOTICE "%s::%s():%d:"
                                "vm_start = 0x%lx, blk[%d].ssptr = 0x%lx,"
                                "w=0x%x\n",
                                __FILE__, __func__, __LINE__,
                                vma->vm_start + k, i, (b->blocks[i].ssptr), p);
                        if (remap_pfn_range(vma, vma->vm_start + k,
                                (b->blocks[i].ssptr) >> PAGE_SHIFT, p,
                                vma->vm_page_prot))
                                return -EAGAIN;;
                        k += p;
                }
        }
        vma->vm_ops = &dmm_remap_vm_ops;
        dmm_vma_open(vma);
        return 0;
}

void
dmm_vma_open(struct vm_area_struct *vma)
{
        if (0)
                printk(KERN_NOTICE "dmm VMA open, virt %lx, phys %lx\n",
                              vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

void
dmm_vma_close(struct vm_area_struct *vma)
{
        /* printk(KERN_NOTICE "dmm VMA close.\n"); */
}

int
tiler_alloc_buf(enum tiler_fmt fmt,
                unsigned long width,
                unsigned long height,
                void **sysptr)
{
        enum errorCodeT eCode = DMM_NO_ERROR;
        struct dmmTILERContCtxT *dmmTilerCtx =
                        &((struct dmmInstanceCtxT *)ctxptr)->dmmTilerCtx;
        struct dmmTILERContPageAreaT *bufferMappedZone;
#ifdef USE_PAT
        void *custmPagesPtr = NULL;
#endif
        enum dmmMemoryAccessT contMod;

        if (fmt == TILFMT_8BIT)
                contMod = MODE_8_BIT;
        else if (fmt == TILFMT_16BIT)
                contMod = MODE_16_BIT;
        else if (fmt == TILFMT_32BIT)
                contMod = MODE_32_BIT;
        else if (fmt == TILFMT_PAGE)
                contMod = MODE_PAGE;
        else
                return DMM_SYS_ERROR;

        if (eCode == DMM_NO_ERROR) {
                eCode = dmm_tiler_container_map_area(dmmTilerCtx,
                                                     width,
                                                     height,
                                                     contMod,
                                                     sysptr,
                                                     &bufferMappedZone);
        }

        if (eCode == DMM_NO_ERROR) {
                bufferMappedZone->xPageOfst = 0;
                bufferMappedZone->yPageOfst = 0;
                bufferMappedZone->xPageCount =
                        bufferMappedZone->x1 - bufferMappedZone->x0 + 1;
                bufferMappedZone->yPageCount =
                        bufferMappedZone->y1 - bufferMappedZone->y0 + 1;
                /* printk(KERN_ERR "x(%u-%u=%u>%u) y(%u-%u=%u>%u)\n",
                  bufferMappedZone->x0, bufferMappedZone->x1,
                  bufferMappedZone->xPageCount, bufferMappedZone->xPageOfst,
                  bufferMappedZone->y0, bufferMappedZone->y1,
                  bufferMappedZone->yPageCount, bufferMappedZone->yPageOfst); */

#ifdef USE_PAT
                eCode = dmm_pat_phy2virt_mapping(bufferMappedZone,
                                                 custmPagesPtr);
#endif
        }

        if (eCode != DMM_NO_ERROR)
                *sysptr = NULL;

        return eCode;
}
EXPORT_SYMBOL(tiler_alloc_buf);

/* :TODO: Currently we do not track enough information from alloc to get back
   the actual width and height of the container, so we must make a guess.  We
   do not even have enough information to get the virtual stride of the buffer,
   which is the real reason for this ioctl */
int
tiler_find_buf(unsigned long sysptr, struct tiler_block_info *blk)
{
        enum errorCodeT eCode = DMM_NO_ERROR;
        struct dmmTILERContCtxT *dmmTilerCtx =
                        &((struct dmmInstanceCtxT *)ctxptr)->dmmTilerCtx;

        struct dmmTILERContPageAreaT *area;

        area = dmm_tiler_get_area_from_sysptr(dmmTilerCtx,
                (void *)(DMM_ALIAS_VIEW_CLEAR & sysptr));
        blk->ptr = NULL;
        if (area != NULL) {
                int accMode = DMM_GET_ACC_MODE(sysptr);
                blk->fmt = (accMode + 1);
                if (blk->fmt == TILFMT_PAGE) {
                        blk->dim.len = area->xPageCount *
                                        area->yPageCount * TILER_PAGESIZE;
                        blk->stride = 0;
                        blk->ssptr =
                                (unsigned long)
                                DMM_COMPOSE_TILER_ALIAS_PTR(
                                ((area->x0 | (area->y0 << 8)) << 12), accMode);
                } else {
                        blk->stride = blk->dim.area.width =
                                        area->xPageCount * TILER_BLOCK_WIDTH;
                        blk->dim.area.height =
                                        area->yPageCount * TILER_BLOCK_HEIGHT;
                        if (blk->fmt == TILFMT_8BIT) {
                                blk->ssptr =
                                        (unsigned long)
                                        DMM_COMPOSE_TILER_ALIAS_PTR(
                                        ((area->x0 << 6) | (area->y0 << 20)),
                                        accMode);
                        } else {
                                blk->ssptr =
                                        (unsigned long)
                                        DMM_COMPOSE_TILER_ALIAS_PTR(
                                        ((area->x0 << 7) | (area->y0 << 20)),
                                        accMode);
                                blk->stride <<= 1;
                                blk->dim.area.height >>= 1;
                                if (blk->fmt == TILFMT_32BIT)
                                        blk->dim.area.width >>= 1;
                        }
                        blk->stride = (blk->stride + TILER_PAGESIZE - 1) &
                                                        ~(TILER_PAGESIZE - 1);
                }
        } else {
                blk->fmt = TILFMT_INVALID;
                blk->dim.len = blk->stride = blk->ssptr = 0;
                eCode = DMM_WRONG_PARAM;
        }

        return eCode;
}

int
tiler_free_buf(unsigned long sysptr)
{
        enum errorCodeT eCode = DMM_NO_ERROR;
        struct dmmTILERContCtxT *dmmTilerCtx =
                        &((struct dmmInstanceCtxT *)ctxptr)->dmmTilerCtx;

        struct dmmTILERContPageAreaT *areaToFree;

        /* if (aliasViewPtr) {
                allocedPtr = (void *)((unsigned long)allocedPtr &
                                      DMM_ALIAS_VIEW_CLEAR);
        } */

        areaToFree = dmm_tiler_get_area_from_sysptr(dmmTilerCtx,
                        (void *)(DMM_ALIAS_VIEW_CLEAR & sysptr));
        if (areaToFree != NULL)
                eCode = dmm_tiler_container_unmap_area(dmmTilerCtx, areaToFree);
        else
                eCode = DMM_SYS_ERROR;

        return eCode;
}
EXPORT_SYMBOL(tiler_free_buf);

#define DMM_SHIFT_PER_X_8 0
#define DMM_SHIFT_PER_Y_8 0
#define DMM_SHIFT_PER_X_16 0
#define DMM_SHIFT_PER_Y_16 1
#define DMM_SHIFT_PER_X_32 1
#define DMM_SHIFT_PER_Y_32 1
#define DMM_SHIFT_PER_X_PAGE 6
#define DMM_SHIFT_PER_Y_PAGE 6

#define DMM_TILER_THE(NAME) (1 << DMM_TILER_##NAME##_BITS)
#define DMM_TILER_THE_(N, NAME) (1 << DMM_TILER_##NAME##_BITS_(N))

#define DMM_TILER_CONT_WIDTH_BITS  14
#define DMM_TILER_CONT_HEIGHT_BITS 13

#define DMM_SHIFT_PER_P_(N) (DMM_SHIFT_PER_X_##N + DMM_SHIFT_PER_Y_##N)

#define DMM_TILER_CONT_HEIGHT_BITS_(N) \
        (DMM_TILER_CONT_HEIGHT_BITS - DMM_SHIFT_PER_Y_##N)
#define DMM_TILER_CONT_WIDTH_BITS_(N) \
        (DMM_TILER_CONT_WIDTH_BITS - DMM_SHIFT_PER_X_##N)

#define DMM_TILER_MASK(bits) ((1 << (bits)) - 1)

#define DMM_TILER_GET_OFFSET_(N, var) \
        ((((unsigned long) var) & DMM_TILER_MASK(DMM_TILER_CONT_WIDTH_BITS + \
        DMM_TILER_CONT_HEIGHT_BITS)) >> DMM_SHIFT_PER_P_(N))

#define DMM_TILER_GET_0_X_(N, var) \
        (DMM_TILER_GET_OFFSET_(N, var) & \
        DMM_TILER_MASK(DMM_TILER_CONT_WIDTH_BITS_(N)))
#define DMM_TILER_GET_0_Y_(N, var) \
        (DMM_TILER_GET_OFFSET_(N, var) >> DMM_TILER_CONT_WIDTH_BITS_(N))
#define DMM_TILER_GET_90_X_(N, var) \
        (DMM_TILER_GET_OFFSET_(N, var) & \
        DMM_TILER_MASK(DMM_TILER_CONT_HEIGHT_BITS_(N)))
#define DMM_TILER_GET_90_Y_(N, var) \
        (DMM_TILER_GET_OFFSET_(N, var) >> DMM_TILER_CONT_HEIGHT_BITS_(N))

#define DMM_TILER_STRIDE_0_(N) \
        (DMM_TILER_THE(CONT_WIDTH) << DMM_SHIFT_PER_Y_##N)
#define DMM_TILER_STRIDE_90_(N) \
        (DMM_TILER_THE(CONT_HEIGHT) << DMM_SHIFT_PER_X_##N)

void tiler_get_natural_xy(unsigned long tsptr, unsigned long *x,
                          unsigned long *y)
{
        unsigned long x_bits, y_bits, offset;

        switch (DMM_GET_ACC_MODE(tsptr)) {
        case MODE_8_BIT:
                x_bits = DMM_TILER_CONT_WIDTH_BITS_(8);
                y_bits = DMM_TILER_CONT_HEIGHT_BITS_(8);
                offset = DMM_TILER_GET_OFFSET_(8, tsptr);
                break;
        case MODE_16_BIT:
                x_bits = DMM_TILER_CONT_WIDTH_BITS_(16);
                y_bits = DMM_TILER_CONT_HEIGHT_BITS_(16);
                offset = DMM_TILER_GET_OFFSET_(16, tsptr);
                break;
        case MODE_32_BIT:
                x_bits = DMM_TILER_CONT_WIDTH_BITS_(32);
                y_bits = DMM_TILER_CONT_HEIGHT_BITS_(32);
                offset = DMM_TILER_GET_OFFSET_(32, tsptr);
                break;
        case MODE_PAGE:
        default:
                x_bits = DMM_TILER_CONT_WIDTH_BITS_(PAGE);
                y_bits = DMM_TILER_CONT_HEIGHT_BITS_(PAGE);
                offset = DMM_TILER_GET_OFFSET_(PAGE, tsptr);
                break;
        }

        if (DMM_GET_ROTATED(tsptr)) {
                *x = offset >> y_bits;
                *y = offset & DMM_TILER_MASK(y_bits);
        } else {
                *x = offset & DMM_TILER_MASK(x_bits);
                *y = offset >> x_bits;
        }

        if (DMM_GET_X_INVERTED(tsptr))
                *x ^= DMM_TILER_MASK(x_bits);
        if (DMM_GET_Y_INVERTED(tsptr))
                *y ^= DMM_TILER_MASK(y_bits);
}

unsigned long tiler_get_address(struct dmmViewOrientT orient,
                                enum dmmMemoryAccessT accessModeM,
                                unsigned long x, unsigned long y)
{
        unsigned long x_bits, y_bits, tmp, x_mask, y_mask, alignment;

        switch (accessModeM) {
        case MODE_8_BIT:
                x_bits = DMM_TILER_CONT_WIDTH_BITS_(8);
                y_bits = DMM_TILER_CONT_HEIGHT_BITS_(8);
                alignment = DMM_SHIFT_PER_P_(8);
                break;
        case MODE_16_BIT:
                x_bits = DMM_TILER_CONT_WIDTH_BITS_(16);
                y_bits = DMM_TILER_CONT_HEIGHT_BITS_(16);
                alignment = DMM_SHIFT_PER_P_(16);
                break;
        case MODE_32_BIT:
                x_bits = DMM_TILER_CONT_WIDTH_BITS_(32);
                y_bits = DMM_TILER_CONT_HEIGHT_BITS_(32);
                alignment = DMM_SHIFT_PER_P_(32);
                break;
        case MODE_PAGE:
        default:
                x_bits = DMM_TILER_CONT_WIDTH_BITS_(PAGE);
                y_bits = DMM_TILER_CONT_HEIGHT_BITS_(PAGE);
                alignment = DMM_SHIFT_PER_P_(PAGE);
                break;
        }

        x_mask = DMM_TILER_MASK(x_bits);
        y_mask = DMM_TILER_MASK(y_bits);
        if (x < 0 || x > x_mask || y < 0 || y > y_mask)
                return 0;

        if (orient.dmmXInvert)
                x ^= x_mask;
        if (orient.dmmYInvert)
                y ^= y_mask;

        if (orient.dmm90Rotate)
                tmp = ((x << y_bits) + y);
        else
                tmp = ((y << x_bits) + x);

        return (unsigned long)
                DMM_COMPOSE_TILER_PTR((tmp << alignment),
                                      (orient.dmm90Rotate ? 1 : 0),
                                      (orient.dmmYInvert ? 1 : 0),
                                      (orient.dmmXInvert ? 1 : 0),
                                      accessModeM);
}

unsigned long
tiler_reorient_addr(unsigned long tsptr,
                struct dmmViewOrientT orient)
{
        unsigned long x, y;

        tiler_get_natural_xy(tsptr, &x, &y);
        return tiler_get_address(orient, DMM_GET_ACC_MODE(tsptr), x, y);
}
EXPORT_SYMBOL(tiler_reorient_addr);

unsigned long
tiler_get_natural_addr(void *sysPtr)
{
        return (unsigned long)sysPtr & DMM_ALIAS_VIEW_CLEAR;
}
EXPORT_SYMBOL(tiler_get_natural_addr);

unsigned long
tiler_reorient_topleft(unsigned long tsptr,
                        struct dmmViewOrientT orient,
                        unsigned int width,
                        unsigned int height)
{
        enum dmmMemoryAccessT accessModeM;
        unsigned long x, y;

        accessModeM = DMM_GET_ACC_MODE(tsptr);

        tiler_get_natural_xy(tsptr, &x, &y);

        if (DMM_GET_X_INVERTED(tsptr))
                x -= width - 1;
        if (DMM_GET_Y_INVERTED(tsptr))
                y -= height - 1;

        if (orient.dmmXInvert)
                x += width - 1;
        if (orient.dmmYInvert)
                y += height - 1;

        return tiler_get_address(orient, accessModeM, x, y);
}
EXPORT_SYMBOL(tiler_reorient_topleft);

unsigned long
tiler_stride(unsigned long tsptr)
{
        switch (DMM_GET_ACC_MODE(tsptr)) {
        case MODE_8_BIT:
                return DMM_GET_ROTATED(tsptr) ?
                        DMM_TILER_STRIDE_90_(8) : DMM_TILER_STRIDE_0_(8);
        case MODE_16_BIT:
                return DMM_GET_ROTATED(tsptr) ?
                        DMM_TILER_STRIDE_90_(16) : DMM_TILER_STRIDE_0_(16);
        case MODE_32_BIT:
                return DMM_GET_ROTATED(tsptr) ?
                        DMM_TILER_STRIDE_90_(32) : DMM_TILER_STRIDE_0_(32);
        default:
                return 0;
        }
}
EXPORT_SYMBOL(tiler_stride);

void
tiler_rotate_view(struct dmmViewOrientT *orient, unsigned long rotation)
{
        rotation = (rotation / 90) & 3;

        if (rotation & 2) {
                orient->dmmXInvert = !orient->dmmXInvert;
                orient->dmmYInvert = !orient->dmmYInvert;
        }

        if (rotation & 1) {
                if (orient->dmm90Rotate)
                        orient->dmmYInvert = !orient->dmmYInvert;
                else
                        orient->dmmXInvert = !orient->dmmXInvert;
                orient->dmm90Rotate = !orient->dmm90Rotate;
        }
}
EXPORT_SYMBOL(tiler_rotate_view);

#define ROUND_UP_2P(a, b) (((a) + (b) - 1) & ~((b) - 1))
#define DIVIDE_UP(a, b) (((a) + (b) - 1) / (b))
#define ROUND_UP(a, b) (DIVIDE_UP(a, b) * (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

void tiler_packed_alloc_buf(int *count,
                            enum tiler_fmt fmt,
                            unsigned long width,
                            unsigned long height,
                            void **sysptr,
                            void **allocptr,
                            int aligned)
{
        int til_width, bpp, bpt, buf_width, alloc_width, map_width;
        int buf_map_width, n_per_m, m_per_a, i = 0, m, n;

        /* Check input parameters for correctness */
        if (!width || !height || !sysptr || !allocptr || !count ||
            *count <= 0 || fmt < TILFMT_8BIT || fmt > TILFMT_32BIT) {
                if (count)
                        *count = 0;
                return;
        }

        /* tiler page width in pixels, bytes per pixel, tiler page in bytes */
        til_width = fmt == TILFMT_32BIT ? 32 : 64;
        bpp = 1 << (fmt - TILFMT_8BIT);
        bpt = til_width * bpp;

        /* width of buffer in tiled pages */
        buf_width = DIVIDE_UP(width, til_width);

        /* :TODO: for now tiler allocation width is 64-multiple */
        alloc_width = ROUND_UP_2P(buf_width, 64);
        map_width = TILER_PAGESIZE / bpt;

        /* ensure alignment if needed */
        buf_map_width = ROUND_UP_2P(buf_width, map_width);

        /* number of buffers in a map window */
        n_per_m = aligned ? 1 : (buf_map_width / buf_width);

        /* number of map windows per allocation */
        m_per_a = alloc_width / buf_map_width;

        printk(KERN_INFO "packing %d*%d buffers into an allocation\n",
               n_per_m, m_per_a);

        while (i < *count) {
                /* allocate required width of a frame to fit remaining
                   frames */
                int n_alloc, m_alloc, tiles, res;
                void *base;

                n_alloc = MIN(*count - i, m_per_a * n_per_m);
                m_alloc = DIVIDE_UP(n_alloc, n_per_m);
                tiles = ((m_alloc - 1) * map_width +
                             buf_width * (n_alloc - (m_alloc - 1) * m_per_a));

                res = tiler_alloc_buf(fmt, til_width * tiles, height,
                                      sysptr + i);
                if (res != DMM_NO_ERROR)
                        break;

                /* mark allocation */
                base = allocptr[i] = sysptr[i];
                i++;

                /* portion out remaining buffers */
                for (m = 0; m < m_per_a; m++, base += bpt * buf_map_width) {
                        for (n = 0; n < n_per_m; n++) {
                                /* first buffer is already allocated */
                                if (n + m == 0)
                                        continue;

                                /* stop if we are done */
                                if (i == *count)
                                        break;

                                /* set buffer address */
                                sysptr[i] = base + bpt * n * buf_width;
                                allocptr[i++] = NULL;
                        }
                }
        }

        /* mark how many buffers we allocated */
        *count = i;
}
EXPORT_SYMBOL(tiler_packed_alloc_buf);

static int layout_packed_nv12(char *offsets, int y_width, int uv_width,
                              void **buf, int blocks, int i,
                              void **y_sysptr, void **uv_sysptr,
                              void **y_allocptr, void **uv_allocptr)
{
        int j;
        for (j = 0; j < blocks; j++, offsets += 3) {
                int page_offset = (63 & (int) offsets[0])
                        + y_width * ((int) offsets[1])
                        + uv_width * (int) offsets[2];
                void *base = buf[offsets[0] >> 6] + 64 * page_offset;

                if (j & 1) {
                        /* convert 8-bit to 16-bit view */
                        /* this formula only works for even ys */
                        uv_sysptr[i] = base + (0x3FFF & (unsigned long) base)
                                + 0x8000000;
                        uv_allocptr[i] = page_offset ? NULL : uv_sysptr[i];
                        i++;
                } else {
                        y_sysptr[i] = base;
                        y_allocptr[i] = page_offset ? NULL : y_sysptr[i];
                }
        }
        return i;
}

void tiler_packed_alloc_nv12_buf(int *count,
                                 unsigned long width,
                                 unsigned long height,
                                 void **y_sysptr,
                                 void **uv_sysptr,
                                 void **y_allocptr,
                                 void **uv_allocptr,
                                 int aligned)
{
        /* optimized packing table */
        /* we read this table from beginning to end, and determine whether
           the optimization meets our requirement (e.g. allocating at least
           i buffers, with max w y-width, and alignment a. If not, we get
           to the next element.  Otherwise we do the allocation.  The table
           is constructed in such a way that if an interim tiler allocation
           fails, the next matching rule for the scenario will be able to
           use the buffers already allocated. */

#define MAX_BUFS_TO_PACK 3
        void *buf[MAX_BUFS_TO_PACK];
        int   n_buf, buf_w[MAX_BUFS_TO_PACK];

        char packing[] = {
                /* min(i), max(w), aligned, buffers to alloc */
                5, 16, 0, 2,
                        /* buffer widths in a + b * w(y) + c * w(uv) */
                        64, 0, 0,  64, 0, 0,
                                /* tiler-page offsets in
                                   a + b * w(y) + c * w(uv) */
                                0,   0, 0,  32,  0, 0,
                                16,  0, 0,  40,  0, 0,
                                64,  0, 0,  96,  0, 0,
                                80,  0, 0,  104, 0, 0,
                                112, 0, 0,  56,  0, 0,

                2, 16, 0, 1,
                        32, 0, 2,
                                0,   0, 0,  32,  0, 0,
                                0,   0, 2,  32,  0, 1,

                2, 20, 0, 1,
                        42, 1, 0,
                                0,   0, 0,  32,  0, 0,
                                42,  0, 0,  21,  0, 0,

                3, 24, 0, 2,
                        48, 0, 1,  32, 1, 0,
                                0,   0, 0,  64,  0, 0,
                                24,  0, 0,  76,  0, 0,
                                96,  0, 0,  48,  0, 0,

                4, 32, 0, 3,
                        48, 0, 1,  32, 1, 0,  32, 1, 0,
                                0,   0, 0,  32,  0, 0,
                                96,  0, 0,  48,  0, 0,
                                64,  0, 0,  128, 0, 0,
                                160, 0, 0,  144, 0, 0,

                /* this is needed for soft landing if prior allocation fails
                   after two buffers */
                2, 32, 1, 2,
                        32, 0, 1,  32, 0, 1,
                                0,  0, 0,  32, 0, 0,
                                64, 0, 0,  96, 0, 0,

                1, 32, 1, 1,
                        32, 0, 1,
                                0, 0, 0,  32, 0, 0,

                2, 64, 1, 3,
                        0, 1, 0,  32, 0, 1,  0, 1, 0,
                                0,   0, 0,  64, 0, 0,
                                128, 0, 0,  96, 0, 0,
                /* this is the basic NV12 allocation using 2 buffers */
                1, 0, 1, 2,
                        0, 1, 0,  0, 0, 1,
                                0, 0, 0,  64, 0, 0,
                0 };
        int y_width, uv_width, i = 0;

        /* Check input parameters for correctness */
        if (!width || !height || !y_sysptr || !y_allocptr || !count ||
            !uv_sysptr || !uv_allocptr || *count <= 0) {
                if (count)
                        *count = 0;
        }

        y_width = DIVIDE_UP(width, 64);
        uv_width = DIVIDE_UP(width >> 1, 64);

        while (i < *count) {
                int n_alloc = *count - i;
                char *p = packing;
                n_buf = 0;

                /* skip packings that do not apply */
                while (*p) {
                        /* see if this packing applies */
                        if (p[0] <= n_alloc &&
                            (!p[1] || p[1] >= y_width) &&
                            (!aligned || p[2])) {

                                /* allocate buffers */
                                while (n_buf < p[3]) {
                                        buf_w[n_buf] = p[4 + 3 * n_buf] +
                                                y_width * p[5 + 3 * n_buf] +
                                                uv_width * p[6 + 3 * n_buf];

                                        if (DMM_NO_ERROR != tiler_alloc_buf(
                                                TILFMT_8BIT, buf_w[n_buf] * 64,
                                                height, buf + n_buf))
                                                break;
                                        n_buf++;
                                }

                                /* if successfully allocated buffers */
                                if (n_buf >= p[3]) {
                                        i = layout_packed_nv12(p + 4 + 3 * p[3],
                                                               y_width,
                                                               uv_width,
                                                               buf, 2 * p[0], i,
                                                               y_sysptr,
                                                               uv_sysptr,
                                                               y_allocptr,
                                                               uv_allocptr);
                                        break;
                                }
                        }

                        p += 4 + 3 * p[3] + 6 * p[0];
                }

                /* if allocation failed free any outstanding buffers and stop */
                if (!*p) {
                        while (n_buf > 0)
                                tiler_free_buf((unsigned long)(buf[--n_buf]));
                        break;
                }
        }

        /* mark how many buffers we allocated */
        *count = i;
}
EXPORT_SYMBOL(tiler_packed_alloc_nv12_buf);

static int createlist(struct node **listhead)
{
        int error = -1;
        void *ret = NULL;

        *listhead = kmalloc(sizeof(struct node), GFP_KERNEL);
        if (*listhead == NULL) {
                printk(KERN_ERR "%s():%d: ERROR!\n", __func__, __LINE__);
                return error;
        }
        ret = memset(*listhead, 0x0, sizeof(struct node));
        if (ret != *listhead) {
                printk(KERN_ERR "%s():%d: ERROR!\n", __func__, __LINE__);
                return error;
        } else {
                /* printk(KERN_ERR "%s():%d: success!\n", __func__, __LINE__);*/
        }
        return 0;
}

static int addnode(struct node *listhead, struct tiler_buf_info *ptr)
{
        int error = -1;
        struct node *tmpnode = NULL;
        struct node *newnode = NULL;
        void *ret = NULL;

        /* assert(listhead != NULL); */
        newnode = kmalloc(sizeof(struct node), GFP_KERNEL);
        if (newnode == NULL) {
                printk(KERN_ERR "%s():%d: ERROR!\n", __func__, __LINE__);
                return error;
        }
        ret = memset(newnode, 0x0, sizeof(struct node));
        if (ret != newnode) {
                printk(KERN_ERR "%s():%d: ERROR!\n", __func__, __LINE__);
                return error;
        }
        newnode->ptr = ptr;
        tmpnode = listhead;

        while (tmpnode->nextnode != NULL)
                tmpnode = tmpnode->nextnode;
        tmpnode->nextnode = newnode;

        return 0;
}

static int
removenode(struct node *listhead, int offset)
{
        struct node *node = NULL;
        struct node *tmpnode = NULL;

        node = listhead;

        while (node->nextnode != NULL) {
                if (node->nextnode->ptr->offset == offset) {
                        tmpnode = node->nextnode;
                        node->nextnode = tmpnode->nextnode;
                        kfree(tmpnode->ptr);
                        kfree(tmpnode);
                        tmpnode = NULL;
                        return 0;
                }
                node = node->nextnode;
        }
        return -1;
}

static int
tiler_destroy_buf_info_list(struct node *listhead)
{
        struct node *tmpnode = NULL;
        struct node *node = NULL;

        node = listhead;

        while (node->nextnode != NULL) {
                tmpnode = node->nextnode;
                node->nextnode = tmpnode->nextnode;
                kfree(tmpnode);
                tmpnode = NULL;
        }
        kfree(listhead);
        return 0;
}

static int
tiler_get_buf_info(struct node *listhead, struct tiler_buf_info **pp, int offst)
{
        struct node *node = NULL;

        node = listhead;

        while (node->nextnode != NULL) {
                if (node->nextnode->ptr->offset == offst) {
                        *pp = node->nextnode->ptr;
                        return 0;
                }
                node = node->nextnode;
        }
        return -1;
}

static void tiler_or_set()
{
#if 0  /* crashes CCS, rvh 12/17/09 */
        void __iomem *reg = NULL;
        unsigned long reg_val = 0x0;
        unsigned long new_val = 0x0;
        unsigned long bit_field = 0x0;
        unsigned long field_pos = 0x0;

        /* set TILER_OR__0 register */
        reg = (void __iomem *)(
                        (unsigned long)dmm_base | (unsigned long)TILER_OR__0);
        reg_val = __raw_readl(reg);
        regdump("TILER_OR__0", reg_val);

        bit_field = BITFIELD(31, 0);
        field_pos = 0;
        new_val = (reg_val & (~(bit_field))) |
                        ((((unsigned long)0) << field_pos) & bit_field);
        __raw_writel(0x88888888, reg);

        reg = (void __iomem *)(
                        (unsigned long)dmm_base | (unsigned long)TILER_OR__0);
        reg_val = __raw_readl(reg);
        regdump("TILER_OR__0", reg_val);

        /* set TILER_OR__1 register */
        reg = (void __iomem *)(
                        (unsigned long)dmm_base | (unsigned long)TILER_OR__1);
        reg_val = __raw_readl(reg);
        regdump("TILER_OR__1", reg_val);

        bit_field = BITFIELD(31, 0);
        field_pos = 0;
        new_val = (reg_val & (~(bit_field))) |
                        ((((unsigned long)0) << field_pos) & bit_field);
        __raw_writel(0x88888888, reg);

        reg = (void __iomem *)(
                        (unsigned long)dmm_base | (unsigned long)TILER_OR__1);
        reg_val = __raw_readl(reg);
        regdump("TILER_OR__1", reg_val);
#endif
}

static void pat_view_set()
{
        void __iomem *reg = NULL;
        unsigned long reg_val = 0x0;
        unsigned long new_val = 0x0;
        unsigned long bit_field = 0x0;
        unsigned long field_pos = 0x0;

        /* set PAT_VIEW__0 register */
        reg = (void __iomem *)(
                        (unsigned long)dmm_base | (unsigned long)PAT_VIEW__0);
        reg_val = __raw_readl(reg);
        regdump("PAT_VIEW__0", reg_val);

        bit_field = BITFIELD(31, 0);
        field_pos = 0;
        new_val = (reg_val & (~(bit_field))) |
                        ((((unsigned long)0) << field_pos) & bit_field);
        __raw_writel(0x88888888, reg);


        reg = (void __iomem *)(
                        (unsigned long)dmm_base | (unsigned long)PAT_VIEW__0);
        reg_val = __raw_readl(reg);
        regdump("PAT_VIEW__0", reg_val);

        /* set PAT_VIEW__1 register */
        reg = (void __iomem *)(
                        (unsigned long)dmm_base | (unsigned long)PAT_VIEW__1);
        reg_val = __raw_readl(reg);
        regdump("PAT_VIEW__1", reg_val);

        bit_field = BITFIELD(31, 0);
        field_pos = 0;
        new_val = (reg_val & (~(bit_field))) |
                        ((((unsigned long)0) << field_pos) & bit_field);
        __raw_writel(0x88888888, reg);

        reg = (void __iomem *)(
                        (unsigned long)dmm_base | (unsigned long)PAT_VIEW__1);
        reg_val = __raw_readl(reg);
        regdump("PAT_VIEW__1", reg_val);
}

static void pat_view_map_set()
{
#ifdef USE_PAT
        void __iomem *reg = NULL;
        unsigned long reg_val = 0x0;
        unsigned long new_val = 0x0;
        unsigned long bit_field = 0x0;
        unsigned long field_pos = 0x0;

        /* set PAT_VIEW_MAP__0 register */
        reg = (void __iomem *)(
                (unsigned long)dmm_base | (unsigned long)PAT_VIEW_MAP__0);
        reg_val = __raw_readl(reg);
        regdump("PAT_VIEW_MAP__0", reg_val);

        bit_field = BITFIELD(31, 0);
        field_pos = 0;
        new_val = (reg_val & (~(bit_field))) |
                        ((((unsigned long)0) << field_pos) & bit_field);
        __raw_writel(0x80808080, reg);


        reg = (void __iomem *)(
                (unsigned long)dmm_base | (unsigned long)PAT_VIEW_MAP__0);
        reg_val = __raw_readl(reg);
        regdump("PAT_VIEW_MAP__0", reg_val);
#else
        unsigned long regval;
        void __iomem *reg;

        /* DMM_PAT_VIEW_MAP__0     0440    0504_0302 */
        reg = (void __iomem *)((unsigned long)dmm_base | (0x440ul));
        regval = __raw_readl(reg);
        __raw_writel(0x05040302, reg);

        /* DMM_PAT_VIEW_MAP__1     0444    0504_0302 */
        reg = (void __iomem *)((unsigned long)dmm_base | (0x444ul));
        regval = __raw_readl(reg);
        __raw_writel(0x05040302, reg);

        /* DMM_PAT_VIEW_MAP__2     0448    0504_0302 */
        reg = (void __iomem *)((unsigned long)dmm_base | (0x448ul));
        regval = __raw_readl(reg);
        __raw_writel(0x05040302, reg);

        /* DMM_PAT_VIEW_MAP__3     044C    0504_0302 */
        reg = (void __iomem *)((unsigned long)dmm_base | (0x44Cul));
        regval = __raw_readl(reg);
        __raw_writel(0x05040302, reg);
#endif
}

static void pat_view_map_base_set()
{
        void __iomem *reg = NULL;
        unsigned long reg_val = 0x0;
        unsigned long new_val = 0x0;
        unsigned long bit_field = 0x0;
        unsigned long field_pos = 0x0;

        /* set PAT_VIEW_MAP_BASE register */
        reg = (void __iomem *)(
                (unsigned long)dmm_base | (unsigned long)PAT_VIEW_MAP_BASE);
        reg_val = __raw_readl(reg);
        regdump("PAT_VIEW_MAP_BASE", reg_val);

        bit_field = BITFIELD(31, 0);
        field_pos = 0;
        new_val = (reg_val & (~(bit_field))) |
                        ((((unsigned long)0) << field_pos) & bit_field);
        __raw_writel(0x80000000, reg);


        reg = (void __iomem *)(
                (unsigned long)dmm_base | (unsigned long)PAT_VIEW_MAP_BASE);
        reg_val = __raw_readl(reg);
        regdump("PAT_VIEW_MAP_BASE", reg_val);
}

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("a0869738@ti.com");
module_init(dmm_init);
module_exit(dmm_exit);


/*
 *  @(#) ti.sdo.tiler.linux; 1, 0, 0,10; 4-1-2010 17:09:39; /db/atree/library/trees/linuxutils/linuxutils-g08x/src/
 */

