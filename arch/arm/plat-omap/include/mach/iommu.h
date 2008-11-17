/*
 * omap iommu: main structures
 *
 * Copyright (C) 2008 Nokia Corporation
 *
 * Written by Hiroshi DOYU <Hiroshi.DOYU@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MACH_IOMMU_H
#define __MACH_IOMMU_H

#include <linux/vmalloc.h>
#include <linux/scatterlist.h>

struct iotlb_entry {
	u32 da;
	u32 pa;
	unsigned int pgsz, prsvd, valid;
	union {
		u16 ap;
		struct {
			u32 endian, elsz, mixed;
		};
	};
};

struct iommu {
	char		*name;
	struct module	*owner;
	u32		*flags;
	struct clk	*clk;
	void __iomem	*regbase;
	unsigned int	regsize;
	struct device	*dev;

	unsigned int	refcount;
	struct mutex	iommu_lock;	/* global for this whole object */

	/*
	 * We don't change iopgd for a situation like pgd for a task,
	 * but share it globally for each iommu.
	 */
	u32		*iopgd;
	spinlock_t	page_table_lock; /* protect iopgd */

	int		nr_tlb_entries;
	int		irq;

	struct list_head	mmap;
	struct mutex		mmap_lock; /* protect mmap */

	int (*isr)(struct iommu *obj);

	void *ctx; /* iommu context: registres saved area */
};

struct cr_regs {
	union {
		struct {
			u16 cam_l;
			u16 cam_h;
		};
		u32 cam;
	};
	union {
		struct {
			u16 ram_l;
			u16 ram_h;
		};
		u32 ram;
	};
};

struct iotlb_lock {
	int base;
	int vict;
};

/* architecture specific functions */
struct iommu_functions {
	unsigned long	version;

	int (*enable)(struct iommu *obj);
	void (*disable)(struct iommu *obj);
	u32 (*fault_isr)(struct iommu *obj, u32 *ra);

	void (*tlb_read_cr)(struct iommu *obj, struct cr_regs *cr);
	void (*tlb_load_cr)(struct iommu *obj, struct cr_regs *cr);

	struct cr_regs *(*alloc_cr)(struct iommu *obj, struct iotlb_entry *e);
	int (*cr_valid)(struct cr_regs *cr);
	u32 (*cr_to_virt)(struct cr_regs *cr);
	void (*cr_to_e)(struct cr_regs *cr, struct iotlb_entry *e);
	ssize_t (*dump_cr)(struct iommu *obj, struct cr_regs *cr, char *buf);

	pgprot_t (*get_pte_attr)(struct iotlb_entry *e);

	void (*save_ctx)(struct iommu *obj);
	void (*restore_ctx)(struct iommu *obj);
	ssize_t (*dump_ctx)(struct iommu *obj, char *buf);
};

struct iommu_platform_data {
	const char	*name;
	int		nr_tlb_entries;
	struct clk	*clk;
	char		*clk_name;
	struct resource	*res;
	int		n_res;
	struct iommu_functions	*ops;
};

#include <mach/iommu2.h>

/*
 * utilities for super page(16MB, 1MB, 64KB and 4KB)
 */

#define iopgsz_max(bytes)			\
	(((bytes) >= SZ_16M) ? SZ_16M :		\
	 ((bytes) >= SZ_1M)  ? SZ_1M  :		\
	 ((bytes) >= SZ_64K) ? SZ_64K :		\
	 ((bytes) >= SZ_4K)  ? SZ_4K  :	0)

#define bytes_to_iopgsz(bytes)				\
	(((bytes) == SZ_16M) ? MMU_CAM_PGSZ_16M :	\
	 ((bytes) == SZ_1M)  ? MMU_CAM_PGSZ_1M  :	\
	 ((bytes) == SZ_64K) ? MMU_CAM_PGSZ_64K :	\
	 ((bytes) == SZ_4K)  ? MMU_CAM_PGSZ_4K  : -1)

#define iopgsz_to_bytes(iopgsz)				\
	(((iopgsz) == MMU_CAM_PGSZ_16M)	? SZ_16M :	\
	 ((iopgsz) == MMU_CAM_PGSZ_1M)	? SZ_1M  :	\
	 ((iopgsz) == MMU_CAM_PGSZ_64K)	? SZ_64K :	\
	 ((iopgsz) == MMU_CAM_PGSZ_4K)	? SZ_4K  : 0)

#define iopgsz_ok(bytes) (bytes_to_iopgsz(bytes) >= 0)

/*
 * global functions
 */
extern u32 iommu_arch_version(void);

extern int load_iotlb_entry(struct iommu *obj, struct iotlb_entry *e);
extern void flush_iotlb_page(struct iommu *obj, u32 da);
extern void flush_iotlb_range(struct iommu *obj, u32 start, u32 end);
extern void flush_iotlb_all(struct iommu *obj);

extern int iopgtable_store_entry(struct iommu *obj, struct iotlb_entry *e);
extern size_t iopgtable_clear_entry(struct iommu *obj, u32 iova);

extern struct iommu *iommu_get(const char *name);
extern void iommu_put(struct iommu *obj);

extern int iommu_enable(struct iommu *obj);
extern void iommu_disable(struct iommu *obj);
extern ssize_t iommu_dump_ctx(struct iommu *obj, char *buf);
extern void iotlb_cr_to_e(struct cr_regs *cr, struct iotlb_entry *e);
extern int iotlb_cr_valid(struct cr_regs *cr);
extern struct cr_regs *iotlb_alloc_cr(struct iommu *obj, struct iotlb_entry *e);
extern u32 iotlb_cr_to_virt(struct cr_regs *cr);
extern pgprot_t get_iopte_attr(struct iotlb_entry *e);
extern void iommu_fault_report(struct iommu *obj);
extern void iotlb_lock_get(struct iommu *obj, struct iotlb_lock *l);
extern void iotlb_lock_set(struct iommu *obj, struct iotlb_lock *l);
extern void iotlb_read_cr(struct iommu *obj, struct iotlb_lock *l,
			  struct cr_regs *cr);
extern void iotlb_load_cr(struct iommu *obj, struct cr_regs *cr);
extern ssize_t iotlb_dump_cr(struct iommu *obj, struct cr_regs *cr, char *buf);

extern int iommu_register_sysfs(struct iommu *obj);
extern void iommu_unregister_sysfs(struct iommu *obj);

extern int io_remap_area(struct vm_struct *area, unsigned long pfn);
extern void __iopgtable_follow_entry(struct iommu *obj, u32 da, u32 **ppgd,
				     u32 **ppte);

extern void iommu_save_ctx(struct iommu *obj);
extern void iommu_restore_ctx(struct iommu *obj);

#endif /* __MACH_IOMMU_H */
