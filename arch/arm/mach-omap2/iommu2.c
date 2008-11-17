/*
 * omap iommu: omap2 architecture specific functions
 *
 * Copyright (C) 2008 Nokia Corporation
 *
 * Written by Hiroshi DOYU <Hiroshi.DOYU@nokia.com>,
 *		Paul Mundt and Toshihiro Kobayashi
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/jiffies.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/stringify.h>
#include <linux/platform_device.h>

#include <mach/iommu.h>
#include <mach/iommu2.h>

/*
 * omap2 architecture specific register bit definitions
 */
#define IOMMU_ARCH_VERSION	0x00000011

/* SYSCONF */
#define MMU_SYS_IDLE_SHIFT	3
#define MMU_SYS_IDLE_FORCE	(0 << MMU_SYS_IDLE_SHIFT)
#define MMU_SYS_IDLE_NONE	(1 << MMU_SYS_IDLE_SHIFT)
#define MMU_SYS_IDLE_SMART	(2 << MMU_SYS_IDLE_SHIFT)
#define MMU_SYS_IDLE_MASK	(3 << MMU_SYS_IDLE_SHIFT)

#define MMU_SYS_SOFTRESET	(1 << 1)
#define MMU_SYS_AUTOIDLE	1

/* SYSSTATUS */
#define MMU_SYS_RESETDONE	1

/* IRQSTATUS & IRQENABLE */
#define MMU_IRQ_MULTIHITFAULT	(1 << 4)
#define MMU_IRQ_TABLEWALKFAULT	(1 << 3)
#define MMU_IRQ_EMUMISS		(1 << 2)
#define MMU_IRQ_TRANSLATIONFAULT	(1 << 1)
#define MMU_IRQ_TLBMISS		(1 << 0)
#define MMU_IRQ_MASK	\
	(MMU_IRQ_MULTIHITFAULT | MMU_IRQ_TABLEWALKFAULT | MMU_IRQ_EMUMISS | \
	 MMU_IRQ_TRANSLATIONFAULT)

/* MMU_CNTL */
#define MMU_CNTL_SHIFT		1
#define MMU_CNTL_MASK		(7 << MMU_CNTL_SHIFT)
#define MMU_CNTL_EML_TLB	(1 << 3)
#define MMU_CNTL_TWL_EN		(1 << 2)
#define MMU_CNTL_MMU_EN		(1 << 1)

#define get_cam_va_mask(pgsz)				\
	(((pgsz) == MMU_CAM_PGSZ_16M) ? 0xff000000 :	\
	 ((pgsz) == MMU_CAM_PGSZ_1M)  ? 0xfff00000 :	\
	 ((pgsz) == MMU_CAM_PGSZ_64K) ? 0xffff0000 :	\
	 ((pgsz) == MMU_CAM_PGSZ_4K)  ? 0xfffff000 : 0)

static int omap2_iommu_enable(struct iommu *obj)
{
	u32 l, pa;
	unsigned long timeout;

	if (!obj->iopgd || !IS_ALIGNED((u32)obj->iopgd,  SZ_16K))
		return -EINVAL;

	pa = virt_to_phys(obj->iopgd);
	if (!IS_ALIGNED(pa, SZ_16K))
		return -EINVAL;

	iommu_write_reg(obj, MMU_SYS_SOFTRESET, MMU_SYSCONFIG);

	timeout = jiffies + msecs_to_jiffies(20);
	do {
		l = iommu_read_reg(obj, MMU_SYSSTATUS);
		if (l & MMU_SYS_RESETDONE)
			break;
	} while (time_after(jiffies, timeout));

	if (!(l & MMU_SYS_RESETDONE)) {
		dev_err(obj->dev, "can't take MMU out of reset\n");
		return -ENODEV;
	}

	l = iommu_read_reg(obj, MMU_REVISION);
	dev_info(obj->dev, "%s: version %d.%d\n", obj->name,
		 (l >> 4) & 0xf, l & 0xf);

	l = iommu_read_reg(obj, MMU_SYSCONFIG);
	l &= ~MMU_SYS_IDLE_MASK;
	l |= MMU_SYS_IDLE_SMART;
	iommu_write_reg(obj, l, MMU_SYSCONFIG);

	iommu_write_reg(obj, MMU_IRQ_MASK, MMU_IRQENABLE);
	iommu_write_reg(obj, pa, MMU_TTB);

	l = iommu_read_reg(obj, MMU_CNTL);
	l &= ~MMU_CNTL_MASK;
	l |= MMU_CNTL_MMU_EN | MMU_CNTL_TWL_EN;
	iommu_write_reg(obj, l, MMU_CNTL);

	return 0;
}

static void omap2_iommu_disable(struct iommu *obj)
{
	u32 l = iommu_read_reg(obj, MMU_CNTL);

	l &= ~MMU_CNTL_MASK;
	iommu_write_reg(obj, l, MMU_CNTL);
	iommu_write_reg(obj, MMU_SYS_IDLE_FORCE, MMU_SYSCONFIG);

	dev_dbg(obj->dev, "%s is shutting down\n", obj->name);
}

static u32 omap2_iommu_fault_isr(struct iommu *obj, u32 *ra)
{
	int i;
	u32 stat, da;
	const char *err_msg[] =
		{"tlb miss", "translation fault", "emulation miss",
		 "table walk fault", "multi hit fault", };

	stat = iommu_read_reg(obj, MMU_IRQSTATUS);
	stat &= MMU_IRQ_MASK;
	if (!stat)
		return 0;

	da = iommu_read_reg(obj, MMU_FAULT_AD);
	*ra = da;

	dev_err(obj->dev, "%s:\tda:%08x ", __func__, da);

	for (i = 0; i < ARRAY_SIZE(err_msg); i++) {
		if (stat & (1 << i))
			pr_err("%s", err_msg[i]);
	}
	pr_err("\n");

	iommu_write_reg(obj, stat, MMU_IRQSTATUS);
	return stat;
}

static void omap2_tlb_read_cr(struct iommu *obj, struct cr_regs *cr)
{
	cr->cam = iommu_read_reg(obj, MMU_READ_CAM);
	cr->ram = iommu_read_reg(obj, MMU_READ_RAM);
}

static void omap2_tlb_load_cr(struct iommu *obj, struct cr_regs *cr)
{
	iommu_write_reg(obj, cr->cam | MMU_CAM_V, MMU_CAM);
	iommu_write_reg(obj, cr->ram, MMU_RAM);
}

static u32 omap2_cr_to_virt(struct cr_regs *cr)
{
	u32 page_size = cr->cam & MMU_CAM_PGSZ_MASK;
	u32 mask = get_cam_va_mask(cr->cam & page_size);

	return cr->cam & mask;
}

static struct cr_regs *omap2_alloc_cr(struct iommu *obj, struct iotlb_entry *e)
{
	struct cr_regs *cr;

	if (e->da & ~(get_cam_va_mask(e->pgsz))) {
		dev_err(obj->dev, "%s:\twrong alignment: %08x\n", __func__,
			e->da);
		return ERR_PTR(-EINVAL);
	}

	cr = kmalloc(sizeof(*cr), GFP_KERNEL);
	if (!cr)
		return ERR_PTR(-ENOMEM);

	cr->cam = (e->da & MMU_CAM_VATAG_MASK) | e->prsvd | e->pgsz;
	cr->ram = e->pa | e->endian | e->elsz;

	return cr;
}

static inline int omap2_cr_valid(struct cr_regs *cr)
{
	return cr->cam & MMU_CAM_V;
}

static pgprot_t omap2_get_pte_attr(struct iotlb_entry *e)
{
	u32 attr;

	attr = e->mixed << 5;
	attr |= e->endian;
	attr |= e->elsz >> 3;
	attr <<= ((e->pgsz & MMU_CAM_PGSZ_4K) ? 0 : 6);

	return attr;
}

static ssize_t omap2_dump_cr(struct iommu *obj, struct cr_regs *cr, char *buf)
{
	char *p = buf;

	/* FIXME: Need more detail analysis of cam/ram */
	p += sprintf(p, "%08x %08x\n", cr->cam, cr->ram);

	return p - buf;
}

#ifdef DEBUG
#define pr_reg(name)							\
	p += sprintf(p, "%20s: %08lx\n",				\
		     __stringify(name), iommu_read_reg(obj, MMU_##name));

static ssize_t omap2_iommu_dump_ctx(struct iommu *obj, char *buf)
{
	char *p = buf;

	pr_reg(REVISION);
	pr_reg(SYSCONFIG);
	pr_reg(SYSSTATUS);
	pr_reg(IRQSTATUS);
	pr_reg(IRQENABLE);
	pr_reg(WALKING_ST);
	pr_reg(CNTL);
	pr_reg(FAULT_AD);
	pr_reg(TTB);
	pr_reg(LOCK);
	pr_reg(LD_TLB);
	pr_reg(CAM);
	pr_reg(RAM);
	pr_reg(GFLUSH);
	pr_reg(FLUSH_ENTRY);
	pr_reg(READ_CAM);
	pr_reg(READ_RAM);
	pr_reg(EMU_FAULT_AD);

	return p - buf;
}
#else
#define omap2_iommu_dump_ctx NULL
#endif /* DEBUG */

static void omap2_iommu_save_ctx(struct iommu *obj)
{
	int i;

	for (i = 0; i < MMU_REG_SIZE; i += sizeof(u32)) {
		u32 val;

		val = iommu_read_reg(obj, i);
		*(u32 *)(obj->ctx + i) = val;

		dev_dbg(obj->dev, "%s\t[%02d] %08x\n", __func__, i, val);
	}
}

static void omap2_iommu_restore_ctx(struct iommu *obj)
{
	int i;

	for (i = 0; i < MMU_REG_SIZE; i += sizeof(u32)) {
		u32 val;

		val = *(u32 *)(obj->ctx + i);
		iommu_write_reg(obj, val, i);

		dev_dbg(obj->dev, "%s\t[%02d] %08x\n", __func__, i, val);
	}
}

static void omap2_cr_to_e(struct cr_regs *cr, struct iotlb_entry *e)
{
	e->da		= cr->cam & MMU_CAM_VATAG_MASK;
	e->pa		= cr->ram & MMU_RAM_PADDR_MASK;
	e->valid	= cr->cam & MMU_CAM_V;
	e->pgsz		= cr->cam & MMU_CAM_PGSZ_MASK;
	e->endian	= cr->ram & MMU_RAM_ENDIAN_MASK;
	e->elsz		= cr->ram & MMU_RAM_ELSZ_MASK;
	e->mixed	= cr->ram & MMU_RAM_MIXED;
}

static struct iommu_functions omap2_iommu_ops = {
	.version	= IOMMU_ARCH_VERSION,

	.enable		= omap2_iommu_enable,
	.disable	= omap2_iommu_disable,
	.fault_isr	= omap2_iommu_fault_isr,

	.tlb_read_cr	= omap2_tlb_read_cr,
	.tlb_load_cr	= omap2_tlb_load_cr,

	.cr_to_e	= omap2_cr_to_e,
	.cr_to_virt	= omap2_cr_to_virt,
	.alloc_cr	= omap2_alloc_cr,
	.cr_valid	= omap2_cr_valid,
	.dump_cr	= omap2_dump_cr,

	.get_pte_attr	= omap2_get_pte_attr,

	.save_ctx	= omap2_iommu_save_ctx,
	.restore_ctx	= omap2_iommu_restore_ctx,
	.dump_ctx	= omap2_iommu_dump_ctx,
};

#if defined(CONFIG_ARCH_OMAP3)

/* Camera ISP MMU */
#define OMAP2_MMU1_BASE		0x480bd400
#define OMAP2_MMU1_IRQ		24

/* IVA2.2 MMU */
#define OMAP2_MMU2_BASE		0x5d000000
#define OMAP2_MMU2_IRQ		28

static struct resource iommu1_res[] = { /* Camera ISP MMU */
	{
		.start		= OMAP2_MMU1_BASE,
		.end		= OMAP2_MMU1_BASE + MMU_REG_SIZE - 1,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP2_MMU1_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct resource iommu2_res[] = { /* IVA2.2 MMU */
	{
		.start		= OMAP2_MMU2_BASE,
		.end		= OMAP2_MMU2_BASE + MMU_REG_SIZE - 1,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP2_MMU2_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct iommu_platform_data omap2_iommu_platform_data[] = {
	{
		.name = "isp",
		.nr_tlb_entries = 8,
		.clk_name = "cam_ick",
		.res = iommu1_res,
		.n_res = ARRAY_SIZE(iommu1_res),
		.ops = &omap2_iommu_ops,
	},
	{
		.name = "iva2",
		.nr_tlb_entries = 32,
		.clk_name = "iva2_ck",
		.res = iommu2_res,
		.n_res = ARRAY_SIZE(iommu2_res),
		.ops = &omap2_iommu_ops,
	},
};
#else
static struct iommu_platform_data omap2_iommu_platform_data[];
#endif /* CONFIG_ARCH_OMAP3 */

static struct platform_device
	*omap2_iommu_pdev[ARRAY_SIZE(omap2_iommu_platform_data)];

static int __init omap2_iommu_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(omap2_iommu_platform_data); i++) {
		int err;
		struct platform_device *new;
		struct iommu_platform_data *p = &omap2_iommu_platform_data[i];

		new = platform_device_alloc("omap-iommu", i + 1);
		if (!new)
			continue;
		new->num_resources = p->n_res;
		new->resource = p->res;
		new->dev.platform_data = p;
		err = platform_device_add(new);
		if (err) {
			platform_device_put(new);
			continue;
		}

		p->clk = clk_get(NULL, p->clk_name);
		if (IS_ERR(p->clk)) {
			platform_device_del(new);
			platform_device_put(new);
			continue;
		}
		omap2_iommu_pdev[i] = new;
	}
	return 0;
}
module_init(omap2_iommu_init);

static void __exit omap2_iommu_exit(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(omap2_iommu_platform_data); i++) {
		int err;
		struct platform_device *pdev;
		struct iommu_platform_data *pdata;

		pdev = omap2_iommu_pdev[i];
		pdata = pdev->dev.platform_data;

		clk_put(pdata->clk);
		platform_device_del(pdev);
		platform_device_put(pdev);
	}
}
module_exit(omap2_iommu_exit);

MODULE_AUTHOR("Hiroshi DOYU, Paul Mundt and Toshihiro Kobayashi");
MODULE_DESCRIPTION("omap iommu: omap2 architecture specific functions");
MODULE_LICENSE("GPL v2");
