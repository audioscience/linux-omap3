/*
 * omap iommu: sysfs for userland interface
 *
 * Copyright (C) 2008 Nokia Corporation
 *
 * Written by Hiroshi DOYU <Hiroshi.DOYU@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

#include <mach/iommu.h>
#include <mach/iovmm.h>

#include "iopgtable.h"

#define kobj_to_iommu(x)						\
	to_iommu((struct device *)container_of(x, struct device, kobj))

static ssize_t generic_attr_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct iommu *obj = to_iommu(dev);

	if (strcmp(attr->attr.name, "name") == 0)
		return sprintf(buf, "%s\n", obj->name);

	if (strcmp(attr->attr.name, "version") == 0)
		return sprintf(buf, "%08x\n", iommu_arch_version());

	if (strcmp(attr->attr.name, "nr_tlb_entries") == 0)
		return sprintf(buf, "%d\n", obj->nr_tlb_entries);

	WARN_ON(1);
	return 0;
}
static DEVICE_ATTR(name, S_IRUGO, generic_attr_show, NULL);
static DEVICE_ATTR(nr_tlb_entries, S_IRUGO, generic_attr_show, NULL);
static DEVICE_ATTR(version, S_IRUGO, generic_attr_show, NULL);

static ssize_t iotlb_attr_store(struct device *dev,
		struct device_attribute *attr, const char *buf,	size_t count)
{
	struct iommu *obj = to_iommu(dev);
	struct iotlb_entry e;
	struct cr_regs cr;
	u32 da;

	sscanf(buf, "%x %x", &cr.cam, &cr.ram); /* FIXME: Add OMAP1 support */
	dev_dbg(obj->dev, "c:%08x r:%08x\n", cr.cam, cr.ram);

	if (!cr.cam || !cr.ram)
		return -EINVAL;

	da = iotlb_cr_to_virt(&cr);

	clk_enable(obj->clk);

	flush_iotlb_page(obj, da);

	iotlb_cr_to_e(&cr, &e);
	load_iotlb_entry(obj, &e);

	clk_disable(obj->clk);

	return count;
}

static ssize_t iotlb_attr_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int i;
	struct iotlb_lock saved, l;
	struct iommu *obj = to_iommu(dev);
	char *p;

	clk_enable(obj->clk);

	iotlb_lock_get(obj, &saved);
	memcpy(&l, &saved, sizeof(saved));

	p = buf;
	p += sprintf(p, "%8s %8s\n", "cam:", "ram:");
	p += sprintf(p, "-----------------------------------------\n");

	for (i = 0; i < obj->nr_tlb_entries; i++) {
		struct cr_regs cr;

		l.vict = i;
		iotlb_read_cr(obj, &l, &cr);

		if (!iotlb_cr_valid(&cr))
			continue;

		p += iotlb_dump_cr(obj, &cr, p);
	}
	iotlb_lock_set(obj, &saved);
	clk_disable(obj->clk);

	return p - buf;
}
static DEVICE_ATTR(tlb, S_IRUGO | S_IWUSR, iotlb_attr_show, iotlb_attr_store);

static ssize_t iopgtable_attr_store(struct device *dev,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	struct iommu *obj = to_iommu(dev);
	struct iotlb_entry e;
	struct cr_regs cr;
	u32 da;
	int err;

	sscanf(buf, "%x %x", &cr.cam, &cr.ram);
	dev_dbg(obj->dev, "c:%08x r:%08x\n", cr.cam, cr.ram);

	if (!cr.cam || !cr.ram)
		return -EINVAL;

	da = iotlb_cr_to_virt(&cr);

	clk_enable(obj->clk);

	iotlb_cr_to_e(&cr, &e);
	err = iopgtable_store_entry(obj, &e);
	if (err)
		dev_err(obj->dev, "%s:\tfail to store cr\n", __func__);

	clk_disable(obj->clk);

	return count;
}

static ssize_t iopgtable_attr_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i;
	char *p = buf;
	struct iommu *obj = to_iommu(dev);
	u32 *iopgd;

	/*
	 * FIXME: analyze the content of attribute in detail
	 */

	p += sprintf(p, "L: %8s %8s\n", "da:", "pa:");
	p += sprintf(p, "-----------------------------------------\n");

	spin_lock(&obj->page_table_lock);

	iopgd = iopgd_offset(obj, 0);
	for (i = 0; i < PTRS_PER_IOPGD; i++, iopgd++) {
		int j;
		u32 *iopte;

		if (!*iopgd)
			continue;

		if (!((u32)*iopgd & IOPGD_TABLE)) {
			u32 da;

			da = i << IOPGD_SHIFT;
			p += sprintf(p, "1: %08x %08x\n", da, *iopgd);
			continue;
		}

		iopte = iopte_offset(iopgd, 0);
		dev_dbg(obj->dev, "%s:\tfound pte:%p\n", __func__, iopte);

		for (j = 0; j < PTRS_PER_IOPTE; j++, iopte++) {
			u32 da;

			if (!*iopte)
				continue;

			dev_dbg(obj->dev, "%s:\t[%03d] %p %08x\n", __func__,
				j, iopte, *iopte);

			da = (i << IOPGD_SHIFT) + (j << IOPTE_SHIFT);
			p += sprintf(p, "2: %08x %08x\n", da, *iopte);
		}
	}

	spin_unlock(&obj->page_table_lock);

	return p - buf;
}
static DEVICE_ATTR(pgtable, S_IRUGO | S_IWUSR,
		   iopgtable_attr_show, iopgtable_attr_store);

static ssize_t mmap_attr_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct iovm_struct *tmp;
	char *p = buf;
	struct iommu *obj = to_iommu(dev);
	int i = 0;

	/*
	 * FIXME:
	 * - trace physical memory address as well
	 * - analyze the content of flags
	 */

	p += sprintf(p, "%-3s %-8s %-8s %6s %8s\n",
		     "No", "start", "end", "size", "flags");
	p += sprintf(p, "-------------------------------------------------\n");

	list_for_each_entry(tmp, &obj->mmap, list) {
		size_t len;

		len = tmp->da_end - tmp->da_start;

		p += sprintf(p, "%3d %08x-%08x %6x %8x\n",
			     i, tmp->da_start, tmp->da_end, len, tmp->flags);
		i++;
	}

	return p - buf;
}
static DEVICE_ATTR(mmap, S_IRUGO, mmap_attr_show, NULL);

#ifdef DEBUG
static ssize_t regs_attr_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct iommu *obj = to_iommu(dev);

	return iommu_dump_ctx(obj, buf);
}
static DEVICE_ATTR(regs, S_IRUGO, regs_attr_show, NULL);
#endif

static struct attribute *iommu_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_nr_tlb_entries.attr,
	&dev_attr_version.attr,
	&dev_attr_tlb.attr,
	&dev_attr_pgtable.attr,
	&dev_attr_mmap.attr,
#ifdef DEBUG
	&dev_attr_regs.attr,
#endif
	NULL,
};

static struct attribute_group iommu_attr_grp = {
	.attrs	= iommu_attrs,
};

static ssize_t iommu_mem_read(struct kobject *kobj, struct bin_attribute *attr,
			      char *buf, loff_t offset, size_t count)
{
	struct iovm_struct *area;
	struct iommu *obj = kobj_to_iommu(kobj);

	mutex_lock(&obj->mmap_lock);

	area = find_iovm_area(obj, offset);
	if (IS_ERR(area)) {
		mutex_unlock(&obj->mmap_lock);
		return -EINVAL;
	}

	clk_enable(obj->clk);

	memcpy(buf, area->va, count);

	clk_disable(obj->clk);

	mutex_unlock(&obj->mmap_lock);
	return count;
}

static ssize_t iommu_mem_write(struct kobject *kobj, struct bin_attribute *attr,
			       char *buf, loff_t offset, size_t count)
{
	struct iovm_struct *area;
	struct iommu *obj = kobj_to_iommu(kobj);

	mutex_lock(&obj->mmap_lock);

	area = find_iovm_area(obj, offset);
	if (IS_ERR(area)) {
		mutex_unlock(&obj->mmap_lock);
		return -EINVAL;
	}

	clk_enable(obj->clk);

	memcpy(area->va, buf, count);

	clk_disable(obj->clk);

	mutex_unlock(&obj->mmap_lock);
	return count;
}

static int iommu_mem_mmap(struct kobject *kobj, struct bin_attribute *attr,
			  struct vm_area_struct *vma)
{
	struct iovm_struct *ia;
	struct iommu *obj = kobj_to_iommu(kobj);

	BUG_ON(!IS_ALIGNED(vma->vm_pgoff, PAGE_SIZE));

	mutex_lock(&obj->mmap_lock);

	ia = find_iovm_area(obj, vma->vm_pgoff);
	if (IS_ERR(ia)) {
		mutex_unlock(&obj->mmap_lock);
		return -EINVAL;
	}

	if ((vma->vm_end - vma->vm_start) != PAGE_SIZE) {
		mutex_unlock(&obj->mmap_lock);
		return -EINVAL;
	}

	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (io_remap_pfn_range(vma, vma->vm_start, (u32)ia->va >> PAGE_SHIFT,
			       PAGE_SIZE, vma->vm_page_prot)) {
		dev_err(obj->dev, "%s:\t\n", __func__);
		mutex_unlock(&obj->mmap_lock);
		return -EAGAIN;
	}
	/* FIXME: clock should be kept on during mapped? */

	mutex_unlock(&obj->mmap_lock);
	return 0;
}

/* FIXME: replace sysfs with more flexible interface(ex: char dev or dspfs) */
static struct bin_attribute iommu_attr_mem = {
	.attr	= {
		.name	= "mem",
		.owner	= THIS_MODULE,
		.mode	= S_IRUSR | S_IWUSR | S_IRGRP,
	},
	.read	= iommu_mem_read,
	.write	= iommu_mem_write,
	.mmap	= iommu_mem_mmap,
};

int iommu_register_sysfs(struct iommu *obj)
{
	int err;

	if (!obj || !obj->dev)
		return -EINVAL;

	err = sysfs_create_group(&obj->dev->kobj, &iommu_attr_grp);
	if (err)
		return err;

	err = sysfs_create_bin_file(&obj->dev->kobj, &iommu_attr_mem);
	if (err)
		sysfs_remove_group(&obj->dev->kobj, &iommu_attr_grp);

	return err;
}
EXPORT_SYMBOL_GPL(iommu_register_sysfs);

void iommu_unregister_sysfs(struct iommu *obj)
{
	WARN_ON(!obj || !obj->dev);

	sysfs_remove_bin_file(&obj->dev->kobj, &iommu_attr_mem);
	sysfs_remove_group(&obj->dev->kobj, &iommu_attr_grp);
}
EXPORT_SYMBOL_GPL(iommu_unregister_sysfs);

static int __init iommu_sysfs_init(void)
{
	return 0;
}
module_init(iommu_sysfs_init)

static void __exit iommu_sysfs_exit(void)
{
}
module_exit(iommu_sysfs_exit)

MODULE_DESCRIPTION("omap iommu: sysfs for userland interface");
MODULE_AUTHOR("Hiroshi DOYU <Hiroshi.DOYU@nokia.com>");
MODULE_LICENSE("GPL v2");
