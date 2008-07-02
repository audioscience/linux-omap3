/*
 * dspbridge/src/pmgr/linux/common/dmm.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */


/*
 *  ======== dmm.c ========
 *  Purpose:
 *      The Dynamic Memory Manager (DMM) module manages the DSP Virtual address
 *      space that can be directly mapped to any MPU buffer or memory region
 *
 *  Public Functions:
 *      DMM_ChunkAdd
 *      DMM_Create
 *      DMM_Destroy
 *      DMM_Exit
 *      DMM_Init
 *      DMM_MapMemory
 *      DMM_Reset
 *      DMM_ReserveMemory
 *      DMM_UnMapMemory
 *      DMM_UnReserveMemory
 *
 *  Private Functions:
 *      AddRegion
 *      CreateRegion
 *      GetRegion
 *
 *  Notes:
 *      Region: Generic memory entitiy having a start address and a size
 *      Chunk:  Reserved region
 *      Block:  Mapped region
 *      Node:   Generic linked list node
 *
 *      First 3 fields of all structs (MapBlk, RsvChunk) must remain same
 *
 *! Revision History:
 *! ================
 *! 19-Apr-2004 sb: Integrated Alan's code review updates.
 *! 17-Mar-2004 ap: Fixed GetRegion for size=0 using tighter bound.
 *! 20-Feb-2004 sb: Created.
 *!
 */

/*  ----------------------------------- Host OS */
#include <host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <std.h>
#include <dbdefs.h>

/*  ----------------------------------- Trace & Debug */
#include <dbc.h>
#include <errbase.h>
#include <gt.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <list.h>
#include <mem.h>
#include <sync.h>

/*  ----------------------------------- Platform Manager */
#include <dev.h>
#include <proc.h>

/*  ----------------------------------- This */
#include <dmm.h>

/*  ----------------------------------- Defines, Data Structures, Typedefs */
/* Object signatures */
#define DMMSIGNATURE       0x004d4d44	/* "DMM"   (in reverse) */

/*
 * DMM Mgr
 */
struct DMM_OBJECT {
	DWORD dwSignature;	/* Used for object validation */
	/*
	 * Dmm Lock is used to serialize access mem manager for multi-threads.
	 */
	struct SYNC_CSOBJECT *hDmmLock;	/* Lock to access dmm mgr */
	struct LST_LIST *freeList;
	struct LST_LIST *usedList;
} ;

/*
 *  Notes:
 *
 *  Region: Generic memory entitiy having a start address and a size
 *  Chunk:  Reserved region
 *  Block:  Mapped region
 *  Node:   Generic linked list node
 *
 *  First 3 fields of all structs (MapBlk, RsvChunk) must remain same
 */

/* RsvChunk is used to reserve a contiguous chunk of
 * DSP virtual address space
 */
struct RsvChunk {
	struct LST_ELEM next;
	ULONG addr;
	ULONG size;
	struct LST_LIST *map;	/* List of mapped blocks in this region */
} ;

/* MapBlk is used to store info of a mapped block in a reserved region */
struct MapBlk {
	struct LST_ELEM next;
	ULONG addr;
	ULONG size;
} ;

/*  ----------------------------------- Globals */
#if GT_TRACE
static struct GT_Mask DMM_debugMask = { 0, 0 };	/* GT trace variable */
#endif

static ULONG cRefs = 0;		/* module reference count */

/*  ----------------------------------- Function Prototypes */
static struct LST_ELEM *CreateRegion(ULONG addr, ULONG size, ULONG elemSize);
static DSP_STATUS AddRegion(struct LST_LIST *list, ULONG addr, ULONG size,
			    ULONG elemSize, BOOL merge);
static struct LST_ELEM *GetRegion(struct LST_LIST *list, ULONG addr,
				 ULONG size);

/*
 *  ======== DMM_ChunkAdd ========
 *  Purpose:
 *      Add a chunk of virtually contiguous DSP/IVA address space to the
 *  freelist.
 */
DSP_STATUS DMM_ChunkAdd(struct DMM_OBJECT *hDmmMgr, ULONG addr, ULONG size)
{
	struct DMM_OBJECT *pDmmObj = (struct DMM_OBJECT *)hDmmMgr;
	DSP_STATUS status;

	GT_3trace(DMM_debugMask, GT_ENTER,
		 "Entered DMM_ChunkAdd () hDmmMgr %x, addr"
		 " %x, size %x\n", hDmmMgr, addr, size);
	SYNC_EnterCS(pDmmObj->hDmmLock);
	status = AddRegion(pDmmObj->freeList, addr, size,
		 sizeof(struct RsvChunk), TRUE);
	SYNC_LeaveCS(pDmmObj->hDmmLock);
	GT_1trace(DMM_debugMask, GT_ENTER, "Leaving DMM_ChunkAdd status %x\n",
		 status);
	return status;
}

/*
 *  ======== DMM_Create ========
 *  Purpose:
 *      Create a dynamic memory manager object.
 */
DSP_STATUS DMM_Create(OUT struct DMM_OBJECT **phDmmMgr,
		     struct DEV_OBJECT *hDevObject,
		     IN CONST struct DMM_MGRATTRS *pMgrAttrs)
{
	struct DMM_OBJECT *pDmmObject = NULL;
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	DBC_Require(phDmmMgr != NULL);

	GT_3trace(DMM_debugMask, GT_ENTER,
		 "DMM_Create: phDmmMgr: 0x%x hDevObject: "
		 "0x%x pMgrAttrs: 0x%x\n", phDmmMgr, hDevObject, pMgrAttrs);
	*phDmmMgr = NULL;
	/* create, zero, and tag a cmm mgr object */
	MEM_AllocObject(pDmmObject, struct DMM_OBJECT, DMMSIGNATURE);
	if (pDmmObject != NULL) {
		if (DSP_SUCCEEDED(status)) {
			/* create the free list */
			pDmmObject->freeList = LST_Create();
			if (pDmmObject->freeList == NULL) {
				GT_0trace(DMM_debugMask, GT_7CLASS,
					 "DMM_Create: LST_Create() "
					 "freeList failed\n");
				status = DSP_EMEMORY;
			}
		}
		if (DSP_SUCCEEDED(status)) {
			/* create the used list */
			pDmmObject->usedList = LST_Create() ;
			if (pDmmObject->usedList == NULL) {
				GT_0trace(DMM_debugMask, GT_7CLASS,
					 "DMM_Create: LST_Create() "
					 "usedList failed\n");
				status = DSP_EMEMORY;
			}
		}
		if (DSP_SUCCEEDED(status))
			status = SYNC_InitializeCS(&pDmmObject->hDmmLock);

		if (DSP_SUCCEEDED(status))
			*phDmmMgr = pDmmObject;
		else
			DMM_Destroy(pDmmObject, TRUE);

	} else {
		GT_0trace(DMM_debugMask, GT_6CLASS,
			 "DMM_Create: Object Allocation "
			 "Failure(DMM Object)\n");
		status = DSP_EMEMORY;
	}
	GT_4trace(DMM_debugMask, GT_ENTER,
		 "Leaving DMM_Create status %x pDmmObject"
		 "%x, freeList %x, usedList %x\n", status, pDmmObject,
		 pDmmObject->freeList, pDmmObject->usedList);

	return (status);
}

/*
 *  ======== DMM_Destroy ========
 *  Purpose:
 *      Release the communication memory manager resources.
 */
DSP_STATUS DMM_Destroy(struct DMM_OBJECT *hDmmMgr, BOOL bForce)
{
	struct DMM_OBJECT *pDmmObj = (struct DMM_OBJECT *)hDmmMgr;
	DSP_STATUS status = DSP_SOK;

	GT_2trace(DMM_debugMask, GT_ENTER,
		 "Entered DMM_Destroy () hDmmMgr %x, bForce"
		 "%x\n", hDmmMgr, bForce);
	DBC_Require(cRefs > 0);
	if (MEM_IsValidHandle(hDmmMgr, DMMSIGNATURE)) {
		/* If not force then fail if outstanding allocations exist */
		if (!bForce) {
			SYNC_EnterCS(pDmmObj->hDmmLock);
			/* Check for outstanding memory allocations */
			if (pDmmObj->usedList && !LST_IsEmpty(pDmmObj->
			   usedList)) {
				/* outstanding allocations */
				status = DSP_EFAIL;
			}
			SYNC_LeaveCS(pDmmObj->hDmmLock);
		}
		if (DSP_SUCCEEDED(status)) {
			/* Delete all DMM nodes in used & free lists
			 * DMM_Reset acquires DMM lock internally before
			 * operating on lists */
			status = DMM_Reset(hDmmMgr);
			SYNC_EnterCS(pDmmObj->hDmmLock);
			if (pDmmObj->freeList != NULL)
				LST_Delete(pDmmObj->freeList);

			if (pDmmObj->usedList != NULL)
				LST_Delete(pDmmObj->usedList);

			SYNC_LeaveCS(pDmmObj->hDmmLock);
		}
		if (DSP_SUCCEEDED(status)) {
			/* Delete CS & dmm mgr object */
			SYNC_DeleteCS(pDmmObj->hDmmLock);
			MEM_FreeObject(pDmmObj);
		}
	} else {
		status = DSP_EHANDLE;
	}
	GT_1trace(DMM_debugMask, GT_ENTER, "Leaving DMM_Destroy status %x\n",
		 status);
	return (status);
}

/*
 *  ======== DMM_Exit ========
 *  Purpose:
 *      Discontinue usage of module; free resources when reference count
 *      reaches 0.
 */
VOID DMM_Exit()
{
	DBC_Require(cRefs > 0);

	cRefs--;

	GT_1trace(DMM_debugMask, GT_ENTER,
		 "exiting DMM_Exit, ref count:0x%x\n", cRefs);
}

/*
 *  ======== DMM_GetHandle ========
 *  Purpose:
 *      Return the dynamic memory manager object for this device.
 *      This is typically called from the client process.
 */
DSP_STATUS DMM_GetHandle(DSP_HPROCESSOR hProcessor,
			OUT struct DMM_OBJECT **phDmmMgr)
{
	DSP_STATUS status = DSP_SOK;
	struct DEV_OBJECT *hDevObject;

	GT_2trace(DMM_debugMask, GT_ENTER,
		 "DMM_GetHandle: hProcessor %x, phDmmMgr"
		 "%x\n", hProcessor, phDmmMgr);
	DBC_Require(cRefs > 0);
	DBC_Require(phDmmMgr != NULL);
	if (hProcessor != NULL)
		status = PROC_GetDevObject(hProcessor, &hDevObject);
	else
		hDevObject = DEV_GetFirst();	/* default */

	if (DSP_SUCCEEDED(status))
		status = DEV_GetDmmMgr(hDevObject, phDmmMgr);

	GT_2trace(DMM_debugMask, GT_ENTER, "Leaving DMM_GetHandle status %x, "
		 "*phDmmMgr %x\n", status, phDmmMgr ? *phDmmMgr : 0);
	return (status);
}

/*
 *  ======== DMM_Init ========
 *  Purpose:
 *      Initializes private state of DMM module.
 */
BOOL DMM_Init()
{
	BOOL fRetval = TRUE;

	DBC_Require(cRefs >= 0);

	if (cRefs == 0) {
		/* Set the Trace mask */
		/*"DM" for Dymanic Memory Manager */
		GT_create(&DMM_debugMask, "DM");
	}

	if (fRetval)
		cRefs++;

	GT_1trace(DMM_debugMask, GT_ENTER,
		 "Entered DMM_Init, ref count:0x%x\n", cRefs);

	DBC_Ensure((fRetval && (cRefs > 0)) || (!fRetval && (cRefs >= 0)));

	return (fRetval);
}

/*
 *  ======== DMM_MapMemory ========
 *  Purpose:
 *      Add a mapping block to the reserved chunk. DMM assumes that this block
 *  will be mapped in the DSP/IVA's address space. DMM returns an error if a
 *  mapping overlaps another one. This function stores the info that will be
 *  required later while unmapping the block.
 */
DSP_STATUS DMM_MapMemory(struct DMM_OBJECT *hDmmMgr, ULONG addr, ULONG size)
{
	struct DMM_OBJECT *pDmmObj = (struct DMM_OBJECT *)hDmmMgr;
	struct RsvChunk *chunk;
	DSP_STATUS status = DSP_SOK;

	GT_3trace(DMM_debugMask, GT_ENTER,
		 "Entered DMM_MapMemory () hDmmMgr %x, "
		 "addr %x, size %x\n", hDmmMgr, addr, size);
	SYNC_EnterCS(pDmmObj->hDmmLock);
	/* Find the Reserved memory chunk containing the DSP block to
	 * be mapped */
	chunk = (struct RsvChunk *)GetRegion(pDmmObj->usedList, addr, size);

	if (chunk != NULL) {
		if (chunk->map == NULL) {
			chunk->map = LST_Create();
			if (chunk->map == NULL)
				status = DSP_EMEMORY;

		}
		if (DSP_SUCCEEDED(status)) {
			/* Add the block to be mapped to this chunk. */
			status = AddRegion(chunk->map, addr, size,
					  sizeof(struct MapBlk), FALSE);
		}
	} else {
		status = DSP_ENOTFOUND;
	}
	SYNC_LeaveCS(pDmmObj->hDmmLock);
	GT_3trace(DMM_debugMask, GT_ENTER,
		 "Leaving DMM_MapMemory status %x, chunk %x"
		 " map %x\n", status, chunk, chunk ? chunk->map : 0);
	return status;
}

/*
 *  ======== DMM_ReserveMemory ========
 *  Purpose:
 *      Reserve a chunk of virtually contiguous DSP/IVA address space.
 */
DSP_STATUS DMM_ReserveMemory(struct DMM_OBJECT *hDmmMgr, ULONG size,
			    ULONG *pRsvAddr)
{
	DSP_STATUS status = DSP_SOK;
	struct DMM_OBJECT *pDmmObj = (struct DMM_OBJECT *)hDmmMgr;
	struct RsvChunk *node;
	ULONG rsvAddr = 0;
	ULONG rsvSize = 0;

	GT_3trace(DMM_debugMask, GT_ENTER,
		 "Entered DMM_ReserveMemory () hDmmMgr %x, "
		 "size %x, pRsvAddr %x\n", hDmmMgr, size, pRsvAddr);
	SYNC_EnterCS(pDmmObj->hDmmLock);
	/* Try to get a DSP chunk from the free list */
	node = (struct RsvChunk *)GetRegion(pDmmObj->freeList, 0, size);

	if (node != NULL) {
		/* DSP chunk of given size is available. */
		rsvAddr = node->addr;
		 /* GetRegion will return first fit chunk. But we only use what
		 * is requested. So remove the remaining portion from the chunk
		 * and enqueue it back on the free list */
		if ((node->size - size) >= PG_SIZE_4K) {
			/* Adjust the freelist chunk's addr & size */
			node->addr += size;
			node->size -= size;
			/* Adjust our node's size */
			rsvSize = size;
		} else {
			rsvSize = node->size;
			LST_RemoveElem(pDmmObj->freeList,
				      (struct LST_ELEM *) node);
			MEM_Free(node);
		}
		status = AddRegion(pDmmObj->usedList, rsvAddr, rsvSize,
				  sizeof(struct RsvChunk), FALSE);
		if (DSP_SUCCEEDED(status)) {
			/* Return the chunk's starting address */
			*pRsvAddr = rsvAddr;
		}
	} else {
		/* DSP chunk of given size is not available */
		status = DSP_EMEMORY;
	}
	SYNC_LeaveCS(pDmmObj->hDmmLock);
	GT_3trace(DMM_debugMask, GT_ENTER,
		 "Leaving ReserveMemory status %x, rsvAddr"
		 " %x, rsvSize %x\n", status, rsvAddr, rsvSize);
	return status;
}

/*
 *  ======== DMM_Reset ========
 *  Purpose:
 *      Remove all the nodes in freeList and usedList.
 *      This function does not delete the lists themselves.
 */
DSP_STATUS DMM_Reset(struct DMM_OBJECT *hDmmMgr)
{
	struct MapBlk *memRegion;
	struct RsvChunk *chunk;
	DSP_STATUS status = DSP_SOK;
	struct DMM_OBJECT *pDmmObj = (struct DMM_OBJECT *)hDmmMgr;

	GT_1trace(DMM_debugMask, GT_ENTER, "Entered DMM_Reset () hDmmMgr %x\n",
		 hDmmMgr);
	SYNC_EnterCS(pDmmObj->hDmmLock);
	if (pDmmObj->freeList != NULL) {
		/* Free the nodes in freelist */
		while (!LST_IsEmpty(pDmmObj->freeList)) {
			chunk = (struct RsvChunk *)LST_GetHead(pDmmObj->
				freeList);
			MEM_Free(chunk);
		}
	}
	if (pDmmObj->usedList != NULL) {
		/* Free the nodes in usedlist */
		while (!LST_IsEmpty(pDmmObj->usedList)) {
			chunk = (struct RsvChunk *)LST_GetHead(pDmmObj->
				usedList);
			if (chunk == NULL)
				continue;

			if (chunk->map == NULL)
				continue;

			/* Free the mapped nodes */
			while (!LST_IsEmpty(chunk->map)) {
				memRegion = (struct MapBlk *)LST_GetHead
					    (chunk->map);
				MEM_Free(memRegion);
			}
			/* Delete mapping list */
			LST_Delete(chunk->map);
			MEM_Free(chunk);
		}
	}
	SYNC_LeaveCS(pDmmObj->hDmmLock);
	GT_1trace(DMM_debugMask, GT_ENTER, "Leaving DMM_Reset status %x\n",
		 status);
	return status;
}

/*
 *  ======== DMM_UnMapMemory ========
 *  Purpose:
 *      Remove the mapped block from the reserved chunk.
 */
DSP_STATUS DMM_UnMapMemory(struct DMM_OBJECT *hDmmMgr, ULONG addr, ULONG *pSize)
{
	struct DMM_OBJECT *pDmmObj = (struct DMM_OBJECT *)hDmmMgr;
	struct RsvChunk *chunk;
	DSP_STATUS status = DSP_SOK;
	struct MapBlk *mapBlk = NULL;

	GT_3trace(DMM_debugMask, GT_ENTER,
		 "Entered DMM_UnMapMemory () hDmmMgr %x, "
		 "addr %x, pSize %x\n", hDmmMgr, addr, pSize);
	SYNC_EnterCS(pDmmObj->hDmmLock);
	/* Find the chunk containing the mapped address */
	chunk = (struct RsvChunk *)GetRegion(pDmmObj->usedList, addr, 0);
	if (chunk) {
		/* Get the mapping block containing addr */
		mapBlk = (struct MapBlk *)GetRegion(chunk->map, addr, 0);
		/* addr must be the start address of the mapping block */
		if (mapBlk && (addr == mapBlk->addr)) {
			*pSize = mapBlk->size;
			LST_RemoveElem(chunk->map, (struct LST_ELEM *)mapBlk);
			MEM_Free(mapBlk);
		} else {
			status = DSP_ENOTFOUND;
		}
	} else {
		/* No chunk or map created */
		status = DSP_EFAIL;
	}
	SYNC_LeaveCS(pDmmObj->hDmmLock);
	GT_4trace(DMM_debugMask, GT_ENTER,
		 "Leaving DMM_UnMapMemory status %x, chunk"
		 " %x, mapBlk %x *pSize %x\n", status, chunk, mapBlk, *pSize);

	return status;
}

/*
 *  ======== DMM_UnReserveMemory ========
 *  Purpose:
 *      Free a chunk of reserved DSP/IVA address space.
 */
DSP_STATUS DMM_UnReserveMemory(struct DMM_OBJECT *hDmmMgr, ULONG rsvAddr)
{
	struct DMM_OBJECT *pDmmObj = (struct DMM_OBJECT *)hDmmMgr;
	struct RsvChunk *chunk;
	DSP_STATUS status = DSP_SOK;

	GT_2trace(DMM_debugMask, GT_ENTER,
		 "Entered DMM_UnReserveMemory () hDmmMgr "
		 "%x, rsvAddr %x\n", hDmmMgr, rsvAddr);

	SYNC_EnterCS(pDmmObj->hDmmLock);

	/* Find the chunk containing the reserved address */
	chunk = (struct RsvChunk *)GetRegion(pDmmObj->usedList, rsvAddr, 0);
	/* Address must match exactly */
	if (chunk && (rsvAddr == chunk->addr)) {
		/* map may not have been created */
		if (chunk->map) {
			/* Verify that there are no mapped blocks in
			 * this chunk */
			if (LST_IsEmpty(chunk->map))
				LST_Delete(chunk->map);
			else
				status = DSP_EFAIL;

		}
		if (DSP_SUCCEEDED(status)) {
			 /* Remove the chunk from the used list and
			 * aplace it in the free list */
			LST_RemoveElem(pDmmObj->usedList,
				      (struct LST_ELEM *)chunk);
			AddRegion(pDmmObj->freeList, chunk->addr, chunk->size,
				 sizeof(struct RsvChunk), TRUE);
			MEM_Free(chunk);
		}
	} else {
		status = DSP_ENOTFOUND;
	}
	SYNC_LeaveCS(pDmmObj->hDmmLock);
	GT_2trace(DMM_debugMask, GT_ENTER,
		 "Leaving DMM_UnReserveMemory status %x"
		 " chunk %x\n", status, chunk);
	return status;
}

/*
 *  ======== AddRegion ========
 *  Purpose:
 *      Add a new region to a linked list, optionally merging it with existing
 *  nodes. Also, set the address and size fields of the new node.
 */
static DSP_STATUS AddRegion(struct LST_LIST *list, ULONG addr, ULONG size,
			   ULONG elemSize, BOOL merge)
{
	BOOL mergePrev = FALSE;
	BOOL mergeNext = FALSE;
	struct MapBlk *newRegion = NULL;
	struct MapBlk *nextRegion = NULL;
	struct MapBlk *prevRegion = NULL;
	DSP_STATUS status = DSP_SOK;

	GT_5trace(DMM_debugMask, GT_ENTER,
		 "Entered AddRegion () list %x, addr %x, "
		 " size %x, elemSize %x, merge %x\n", list, addr, size,
		 elemSize, merge);
	if (!LST_IsEmpty(list)) {
		/* Find the node with the next higher address */
		nextRegion = (struct MapBlk *)LST_First(list);
		while (nextRegion && (addr > nextRegion->addr)) {
			prevRegion = nextRegion;
			nextRegion = (struct MapBlk *)
			LST_Next(list, (struct LST_ELEM *)nextRegion);
		}
		/* Verify that the new node does not overlap existing ones */
		if ((prevRegion && (addr < (prevRegion->addr +
		   prevRegion->size))) ||
		   (nextRegion && ((addr + size) > nextRegion->addr))) {
			status = DSP_EINVALIDARG;
		}
		 /* If the caller requested to merge adjacent nodes, check for
		 * address contiguity */
		if (merge) {
			if (prevRegion && (addr == (prevRegion->addr +
			   prevRegion->size))) {
				mergePrev = TRUE;
			}
			if (nextRegion && ((addr + size) == nextRegion->addr))
				mergeNext = TRUE;

		}
	}
	if (DSP_SUCCEEDED(status)) {
		/* Update existing nodes if merging */
		if (mergePrev && mergeNext) {
			prevRegion->size += size + nextRegion->size;
			LST_RemoveElem(list, (struct LST_ELEM *)nextRegion);
			MEM_Free(nextRegion);
		} else if (mergePrev) {
			prevRegion->size += size;
		} else if (mergeNext) {
			nextRegion->size += size;
			nextRegion->addr = addr;
		} else {
			/* No merge. Create new element */
			newRegion = (struct MapBlk *)CreateRegion(addr, size,
				    elemSize);
			if (newRegion != NULL) {
				if (nextRegion != NULL) {
					/* Insert before the next element */
					LST_InsertBefore(list,
						 (struct LST_ELEM *)newRegion,
						 (struct LST_ELEM *)nextRegion);
				} else {
					/* The List is empty, or there is no
					 * existing element with the next
					 *  higher address. Append the new node
					 *  to the end of the list */
					LST_PutTail(list,
						 (struct LST_ELEM *)newRegion);
				}
			} else {
				status = DSP_EMEMORY;
			}
		}
	}
	GT_4trace(DMM_debugMask, GT_ENTER,
		 "Leaving AddRegion status %x, mergePrev "
		 "%x, mergeNext %x, newRegion %x\n", status,
		 mergePrev, mergeNext, newRegion);
	return status;
}

/*
 *  ======== CreateRegion ========
 *  Purpose:
 *      Allocate a new region and set its address and size fields.
 */
static struct LST_ELEM *CreateRegion(ULONG addr, ULONG size, ULONG elemSize)
{
	struct MapBlk *newRegion;

	GT_3trace(DMM_debugMask, GT_ENTER,
		 "Entered CreateRegion () addr %x, size %x, "
		 " elemSize %x\n", addr, size, elemSize);

	newRegion = (struct MapBlk *)MEM_Calloc(elemSize, MEM_NONPAGED);
	if (newRegion != NULL) {
		/* Initialize the list element - next, prev and self pointers */
		LST_InitElem((struct LST_ELEM *) newRegion);
		/* Initialize the node/chunk attributes */
		newRegion->addr = addr;
		newRegion->size = size;
	}
	GT_1trace(DMM_debugMask, GT_ENTER,
		 "Leaving CreateRegion newRegion 0x%x\n", newRegion);
	return ((struct LST_ELEM *) newRegion);
}

/*
 *  ======== GetRegion ========
 *  Purpose:
 *      Returns a node containing the specified memory region if addr is nonzero
 *  Returns a node whose size is greater than the specified size if addr is zero
 */
static struct LST_ELEM *GetRegion(struct LST_LIST *list, ULONG addr,
				  ULONG size)
{
	struct MapBlk *currRegion = NULL;

	GT_3trace(DMM_debugMask, GT_ENTER,
		 "Entered GetRegion () list %x, addr %x, "
		 " size %x\n", list, addr, size);
	if (list == NULL)
		goto func_end;

	currRegion = (struct MapBlk *)LST_First(list);
	if (addr == 0) {
		/* If addr is NULL, find a node with sufficient size */
		while (currRegion) {
			if (size <= currRegion->size)
				break;

			currRegion = (struct MapBlk *)
				 LST_Next(list, (struct LST_ELEM *)currRegion);
		}
	} else if (size == 0) {
		while (currRegion) {
			/* find a node containing the address (if size is 0) */
			if ((addr >= currRegion->addr) &&
			   (addr < (currRegion->addr + currRegion->size))) {
				break;
			}
			currRegion = (struct MapBlk *)
				 LST_Next(list, (struct LST_ELEM *)currRegion);
		}
	} else {
		while (currRegion) {
			 /* Find a node containing the memory region specified
			 * by addr and size */
			if ((addr >= currRegion->addr) && ((addr + size) <=
			   (currRegion->addr + currRegion->size))) {
				break;
			}
			currRegion = (struct MapBlk *)
				LST_Next(list, (struct LST_ELEM *)currRegion);
		}
	}
func_end:
	GT_1trace(DMM_debugMask, GT_ENTER, "Leaving GetRegion currRegion %x\n",
		 currRegion);
	return ((struct LST_ELEM *)currRegion);
}

