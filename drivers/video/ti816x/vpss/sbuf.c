/*
 * linux/drivers/video/ti816x/vpss/sbuf.c
 *
 * VPSS shared buffer(A8 and M3) driver for TI 816X
 *
 * Copyright (C) 2009 TI
 * Author: Yihe Hu <yihehu@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA	02111-1307, USA.
 */

#define VPSS_SUBMODULE_NAME "SHRBUF"

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <asm/setup.h>

#include "core.h"


/*shared buffer structure*/
struct sbuf_info {
	struct list_head alloc_list;
	u32 paddr;
	u32 pages;
	void *vaddr;
	void __iomem *base;

};
/*allocate buffer structure*/
struct sbuf_alloc {
	struct list_head list;
	u32 paddr;
	u32 pages;
};

static struct sbuf_info *sbinfo;
static DEFINE_MUTEX(sbuf_mutex);

/*create list node*/
static struct sbuf_alloc *sbuf_create_allocation(
					  u32 start,
					  u32 pages)
{
	struct sbuf_alloc *sba;
	struct sbuf_alloc *new;
	new = kzalloc(sizeof(*sba), GFP_KERNEL);
	if (!new)
		return NULL;

	new->paddr = start;
	new->pages = pages;


	list_for_each_entry(sba, &sbinfo->alloc_list, list) {
		if (sba->paddr > new->paddr)
			break;
	}

	list_add_tail(&new->list, &sba->list);
	return new;

}

/*free the allocation*/
static void sbuf_free_allocation(struct sbuf_alloc *sba)
{
	list_del(&sba->list);
	kfree(sba);
}


/*internal allocate buffer*/
static void *_vps_sbuf_alloc(u32 pages, u32 *paddr)
{
	struct sbuf_alloc *sballoc;
	u32 start, end;
	void *vaddr;

	start = sbinfo->paddr;

	list_for_each_entry(sballoc, &sbinfo->alloc_list, list) {
		end = sballoc->paddr;

		if (end - start >= pages << PAGE_SHIFT)
			goto found;
		start = sballoc->paddr + (sballoc->pages << PAGE_SHIFT);
	}
	end = sbinfo->paddr + (sbinfo->pages << PAGE_SHIFT);
found:
	if (end - start < pages << PAGE_SHIFT)
		return NULL;

	if (sbuf_create_allocation(start, pages) == NULL)
		return NULL;

	*paddr = start;

	vaddr = sbinfo->vaddr + (start - sbinfo->paddr);

	VPSSDBG("FOUND %x, end %x, map vir %p\n", start, end, vaddr);

	return vaddr;

}
/*allocate buffer*/
void *vps_sbuf_alloc(size_t size, u32 *paddr)
{
	void *vaddr;
	unsigned pages;

	pages = PAGE_ALIGN(size) >> PAGE_SHIFT;

	mutex_lock(&sbuf_mutex);

	vaddr = _vps_sbuf_alloc(pages, paddr);

	mutex_unlock(&sbuf_mutex);

	return vaddr;
}

/*free buffer*/
int vps_sbuf_free(u32 paddr, void *vaddr, size_t size)
{
	struct sbuf_alloc  *sba;
	u32 start, end;


	size = PAGE_ALIGN(size);

	mutex_lock(&sbuf_mutex);
	list_for_each_entry(sba, &sbinfo->alloc_list, list) {
		start = sba->paddr;
		end = sba->paddr + (sba->pages << PAGE_SHIFT);

		if (start >= paddr && (end <= paddr + size)) {
			sbuf_free_allocation(sba);
			VPSSDBG("free mem paddr %08x vaddr %p size %d\n",
				paddr, vaddr, size);

			break;
		}

	}
	mutex_unlock(&sbuf_mutex);
	return 0;
}

int __init vps_sbuf_init(void)
{
	int r = 0;
	int size;
	VPSSDBG("sbuf init\n");
	sbinfo = kzalloc(sizeof(struct sbuf_info), GFP_KERNEL);
	if (sbinfo == NULL) {
		VPSSERR("failed to allocate\n");
		r = -ENOMEM;
		goto exit;
	}
	size = CONFIG_TI816X_VPSS_SHARED_BUFFER_SIE * 1024;
	sbinfo->base = ioremap_nocache(CONFIG_TI816X_VPSS_SHARED_BUFFER_BASE,
				       size);

	if (sbinfo->base == NULL) {
		VPSSERR("failed to remap.\n");
		r = -ENODEV;
		goto exit;
	}

	sbinfo->vaddr = (void *)sbinfo->base;
	sbinfo->paddr = CONFIG_TI816X_VPSS_SHARED_BUFFER_BASE;
	sbinfo->pages = PAGE_ALIGN(size) >> PAGE_SHIFT;
	VPSSDBG("map %x to %p with size %x\n",
		sbinfo->paddr,
		sbinfo->base,
		sbinfo->pages << PAGE_SHIFT);

	INIT_LIST_HEAD(&sbinfo->alloc_list);

	return 0;
exit:
	vps_sbuf_deinit();
	return r;
}

int __exit vps_sbuf_deinit(void)
{
	struct sbuf_alloc *sba, *next;
	VPSSDBG("sbuf deinit\n");
	/*free any remaining buffer*/
	if (!list_empty(&sbinfo->alloc_list)) {
		list_for_each_entry_safe(sba,
					 next,
					 &sbinfo->alloc_list,
					 list)
			sbuf_free_allocation(sba);

	}
	if (sbinfo->base)
		iounmap(sbinfo->base);
	kfree(sbinfo);
	return 0;

}

