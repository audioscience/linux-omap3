/*
 * dspbridge/src/wmd/linux/omap/common/tiomap_io.c
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
 *  ======== _tiomap_io.c ========
 *  Description:
 *      Implementation for the io read/write routines.
 *
 *! Revision History
 *! ================
 *! 16-Feb-2004 vp:  Fixed warning in WriteDspData function.
 *! 16-Apr-2003 vp:  Added support for TC word swap
 *! 26-Feb-2003 vp:  Fixed issue with EXT_BEG and EXT_END address.
 *! 24-Feb-2003 vp:  Ported to Linux platform
 *! 08-Oct-2002 rr:  Created.
 */

/*  ----------------------------------- DSP/BIOS Bridge */
#include <dbdefs.h>
#include <errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <dbc.h>
#include <dbg.h>
#include <dbg_zones.h>

/*  ----------------------------------- Platform Manager */
#include <dev.h>
#include <drv.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <mem.h>
#include <util.h>
#include <cfg.h>

/*  ----------------------------------- specific to this file */
#include "_tiomap.h"
#include "_tiomap_pwr.h"
#include "tiomap_io.h"

static ULONG ulExtBase = 0;
static ULONG ulExtEnd = 0;

#if !defined(OMAP_2430) && !defined(OMAP_3430)
static ULONG ulIvaExtBase = 0;
static ULONG ulIvaExtEnd = 0;
#endif

#if defined(OMAP_2430) || defined(OMAP_3430)
static ULONG ulShm0End = 0;
static ULONG ulDynExtBase = 0;
#endif

/*
 *  ======== ReadExtDspData ========
 *  purpose:
 *      Copies DSP external memory buffers to the host side buffers.
 */
DSP_STATUS ReadExtDspData(struct WMD_DEV_CONTEXT *hDevContext,
			 OUT BYTE *pbHostBuf, DWORD dwDSPAddr,
			 ULONG ulNumBytes, ULONG ulMemType)
{
	DSP_STATUS status = DSP_SOK;
	struct WMD_DEV_CONTEXT *pDevContext = hDevContext;
#if !defined(OMAP_2430) && !defined(OMAP_3430)
	INT remain = 0;
	INT i = 0;
	INT start = 0;
	ULONG bytes = 0;
	USHORT wTemp1;
	USHORT wTemp2;
	DWORD dwTemp;
	BYTE bTempByte1;
	BYTE bTempByte2;
	INT remainBytes;
	BYTE remainByte[4];
	DWORD myStart;
#endif
	ULONG offset;
	ULONG ulShmBaseVirt = 0;
	ULONG ulTLBBaseVirt = 0;
	ULONG ulShmOffsetVirt = 0;
	DWORD dwExtProgVirtMem;
	DWORD dwBaseAddr = pDevContext->dwDspExtBaseAddr;
	struct CFG_HOSTRES hostRes;

	DBG_Trace(DBG_ENTER, "ReadExtDspData, hDevContext: 0x%x\n\t\t"
		 "pbHostBuf: 0x%x\n\t\tdwDSPAddr:  0x%x\n\t\tulNumBytes:  "
		 "0x%x\n\t\t ulMemType:  0x%x\n", pDevContext, pbHostBuf,
		 dwDSPAddr, ulNumBytes, ulMemType);
	DBG_Trace(DBG_LEVEL1, "Reading from  DSP Trace section 0x%x \n",
		 dwDSPAddr);

	if (!dwBaseAddr) {
		/* Initialize ulExtBase and ulExtEnd */
		ulExtBase = 0;
		ulExtEnd = 0;
		/* Get SHM_BEG  EXT_BEG and EXT_END. */
		status = DEV_GetSymbol(pDevContext->hDevObject, SHMBASENAME,
					&ulShmBaseVirt);
		DBC_Assert(ulShmBaseVirt != 0);
		if (DSP_SUCCEEDED(status)) {
			status = DEV_GetSymbol(pDevContext->hDevObject,
				 EXTBASE, &ulExtBase);
		}
		DBC_Assert(ulExtBase != 0);
		if (DSP_SUCCEEDED(status)) {
			status = DEV_GetSymbol(pDevContext->hDevObject, EXTEND,
						&ulExtEnd);
		}
		DBC_Assert(ulExtEnd != 0);
		DBC_Assert(ulExtEnd > ulExtBase);
		if (ulExtEnd < ulExtBase) {
			status = DSP_EFAIL;
		}
#if !defined(OMAP_2430) && !defined(OMAP_3430)
		/* Convert word addresses to byte addresses */
		ulShmBaseVirt *= DSPWORDSIZE;
		ulExtBase *= DSPWORDSIZE;
		ulExtEnd *= DSPWORDSIZE;
#endif
		if (DSP_SUCCEEDED(status)) {
			/* ulTLBBaseVirt = pDevContext->aTLBEntry[0].ulDspVa;
			 * DBC_Assert(ulTLBBaseVirt <= ulShmBaseVirt);
			 * ulShmOffsetVirt = ulShmBaseVirt - */
			/* (ulTLBBaseVirt * DSPWORDSIZE);
			 * The code starts after EXT_BEG till EXT_END
			 * dwExtProgVirtMem = ulExtBase - ulShmBaseVirt;
			 * dwExtProgVirtMem += pDevContext->aTLBEntry[0].
			 * ulGppPa;
			 * dwExtProgVirtMem += ulShmOffsetVirt; */
			CFG_GetHostResources(
				(struct CFG_DEVNODE *)
				DRV_GetFirstDevExtension(),
				&hostRes);
			dwExtProgVirtMem = hostRes.dwMemBase[1];
			DBG_Trace(DBG_LEVEL1,
				 "***dwExtProgVirtMem in READEXTDSPDATA "
				 "is 0x%x \n", dwExtProgVirtMem);
			DBG_Trace(DBG_LEVEL1,
				 "***ulExtBase in READEXTDSPDATA is 0x%x\n",
				 ulExtBase);
			DBG_Trace(DBG_LEVEL1,
				 "***ulDynExtbase in READEXTDSPDATA is 0x%x\n",
				 ulDynExtBase);
			dwExtProgVirtMem += (ulExtBase - ulDynExtBase);
#if !defined(OMAP_2430) && !defined(OMAP_3430)
			/* if the ExtBase is not DWORD aligned, we align this
			 * * virtual address to a DWORD boundary */
			if (dwExtProgVirtMem % 4) {
				dwExtProgVirtMem -= 2;
				dwBaseAddr += 2;
			}
#endif
			pDevContext->dwDspExtBaseAddr =
				 (DWORD)MEM_LinearAddress((VOID *)
				 TO_VIRTUAL_UNCACHED(dwExtProgVirtMem),
				 ulExtEnd - ulExtBase);
			dwBaseAddr += pDevContext->dwDspExtBaseAddr;
			/* This dwDspExtBaseAddr will get cleared only when
			 * the board is stopped.  */
			if (!pDevContext->dwDspExtBaseAddr) {
				status = DSP_EFAIL;
				DBG_Trace(DBG_LEVEL7,
					 "ReadExtDspData: failed to Map the "
					 "program memory\n");
			}
		}
	}
	if (!dwBaseAddr || !ulExtBase || !ulExtEnd) {
		DBG_Trace(DBG_LEVEL7,
			 "Symbols missing for Ext Prog loading \n");
		status = DSP_EFAIL;
	}
	offset = dwDSPAddr - ulExtBase;
#if defined(OMAP_2430) || defined(OMAP_3430)
	memcpy(pbHostBuf, (BYTE *) dwBaseAddr + offset, ulNumBytes);
#else
	/*  If the starting address is not word aligned */
	start = (dwDSPAddr % 4);
	if (start) {
		myStart = dwDSPAddr - start;
		if (pDevContext->tcWordSwapOn) {
			remainBytes = (4 - start);
			if (remainBytes == 3) {
			     *pbHostBuf++ = *(BYTE *)(dwBaseAddr + myStart + 0);
			     *pbHostBuf++ = *(BYTE *)(dwBaseAddr + myStart + 3);
			     *pbHostBuf++ = *(BYTE *)(dwBaseAddr + myStart + 2);
			}
			if (remainBytes == 2) {
			     *pbHostBuf++ = *(BYTE *)(dwBaseAddr + myStart + 3);
			     *pbHostBuf++ = *(BYTE *)(dwBaseAddr + myStart + 2);
			}
			if (remainBytes == 1) {
			     *pbHostBuf++ = *(BYTE *)(dwBaseAddr + myStart + 2);
			}
		} else {
			for (i = start; i < 4; i++) {
			     *pbHostBuf++ = *(volatile BYTE *)(dwBaseAddr +
					myStart + 3 - i);
			}
		}
		bytes += (4 - start);
		offset += bytes;
	}
	for (i = 0; i < 4; i++) {
		remainByte[i] = 0x0;
	}
	/*  WMD has to make sure the size of ulNumBytes is word aligned ! */
	while (bytes < ulNumBytes) {
		/*  This is the terminating condition - i.e if there are less
		 *  than four bytes left.*/
		if ((remain = ulNumBytes - bytes) < 4) {
			if (pDevContext->tcWordSwapOn) {
				remainBytes = remain;
				if (remainBytes == 3) {
					*pbHostBuf++ = *(BYTE *)(dwBaseAddr +
							offset + 1);
					*pbHostBuf++ = *(BYTE *)(dwBaseAddr +
							offset + 0);
					*pbHostBuf++ = *(BYTE *)(dwBaseAddr +
							offset + 3);
				}
				if (remainBytes == 2) {
					*pbHostBuf++ = *(BYTE *)(dwBaseAddr +
							offset + 1);
					*pbHostBuf++ = *(BYTE *)(dwBaseAddr +
							offset + 0);
				}
				if (remainBytes == 1) {
					*pbHostBuf++ = *(BYTE *)(dwBaseAddr +
							offset + 1);
				}
			} else {
				for (i = 0; i < remain; i++) {
					*pbHostBuf++ = *(volatile BYTE *)
							(dwBaseAddr+offset+3-i);
				}
			}
			bytes += remain;
			break;
		}
		dwTemp = *(volatile DWORD *)(dwBaseAddr + offset);
		if (pDevContext->tcWordSwapOn) {
			/*  Get two words from the 32-bit dwTemp.*/
			wTemp1 = (USHORT) (dwTemp & 0xFFFF);
			wTemp2 = (USHORT) (dwTemp >> 16);
		} else {
			/*  Get two words from the 32-bit dwTemp. */
			wTemp2 = (USHORT) (dwTemp & 0xFFFF);
			wTemp1 = (USHORT) (dwTemp >> 16);
		}
		/*  Get first two bytes from wTemp1 */
		bTempByte2 = (BYTE) (wTemp1 & 0xFF);
		bTempByte1 = (BYTE) (wTemp1 >> 8);
		/*  first two bytes */
		*pbHostBuf++ = bTempByte1;
		*pbHostBuf++ = bTempByte2;
		/*  Get next two bytes from wTemp2 */
		bTempByte2 = (BYTE) (wTemp2 & 0xFF);
		bTempByte1 = (BYTE) (wTemp2 >> 8);
		/*  next two-bytes */
		*pbHostBuf++ = bTempByte1;
		*pbHostBuf++ = bTempByte2;
		offset += 4;
		bytes += 4;
	}
#endif
	return (status);
}

/*
 *  ======== WriteDspData ========
 *  purpose:
 *      Copies buffers to the DSP internal/external memory.
 */
DSP_STATUS WriteDspData(struct WMD_DEV_CONTEXT *hDevContext, IN BYTE *pbHostBuf,
			DWORD dwDSPAddr, ULONG ulNumBytes, ULONG ulMemType)
{
#if defined(OMAP_1710) || defined(OMAP_16xx) || defined(OMAP_1510)
	ULONG i;
	ULONG offset;
	USHORT wTemp;
	BYTE bTempByte1;
	BYTE bTempByte2;
#endif
	DWORD tempBaseaddr;
	DWORD offset;
	DWORD dwBaseAddr = hDevContext->dwDspBaseAddr;
	struct CFG_HOSTRES resources;
	DSP_STATUS status;
	DWORD base1, base2, base3;
	base1 = OMAP_DSP_MEM1_SIZE;
	base2 = OMAP_DSP_MEM2_BASE - OMAP_DSP_MEM1_BASE;
	base3 = OMAP_DSP_MEM3_BASE - OMAP_DSP_MEM1_BASE;
	DBG_Trace(DBG_ENTER, "Entered WriteDspData \n");


	status =  CFG_GetHostResources(
		 (struct CFG_DEVNODE *)DRV_GetFirstDevExtension(), &resources);

	offset = dwDSPAddr - hDevContext->dwDSPStartAdd;
	if (offset < base1) {
		dwBaseAddr = MEM_LinearAddress(resources.dwMemBase[2],
						resources.dwMemLength[2]);
	} else if (offset > base1 && offset < base2+OMAP_DSP_MEM2_SIZE) {
		dwBaseAddr = MEM_LinearAddress(resources.dwMemBase[3],
						resources.dwMemLength[3]);
		offset = offset - base2;
	} else if (offset >= base2+OMAP_DSP_MEM2_SIZE &&
		offset < base3 + OMAP_DSP_MEM3_SIZE) {
		dwBaseAddr = MEM_LinearAddress(resources.dwMemBase[4],
						resources.dwMemLength[4]);
		offset = offset - base3;
	} else{
		status = DSP_EFAIL;
		return(status);
	}
#if defined(OMAP_2430) || defined(OMAP_3430)
	memcpy((BYTE *) (dwBaseAddr+offset), pbHostBuf, ulNumBytes);
#else
	/* First check if DSP is in running state... */
	if (pDevContext->dwBrdState != BRD_RUNNING) {
		return (DSP_EWRONGSTATE);
	}
	offset = dwDSPAddr;
	DBG_Trace(DBG_LEVEL1, "Reading from  DSP Trace section 0x%x \n",
		 dwDSPAddr);
	/*  If address is ODD, write first byte */
	if (dwDSPAddr % 2) {
		offset--;
		bTempByte1 = *pbHostBuf++;
		wTemp = *((volatile WORD *)(dwBaseAddr + offset));
		wTemp &= 0xFF00;
		wTemp |= (USHORT) bTempByte1;
		*((volatile WORD *)(dwBaseAddr + offset)) = wTemp;
		offset += 2;
		ulNumBytes--;
	}
	/*  DSP is word accessible. Two bytes are written per DSP Address */
	for (i = 0; i < (ulNumBytes / 2); i++) {
		bTempByte1 = *pbHostBuf++;
		bTempByte2 = *pbHostBuf++;
		wTemp = (USHORT) bTempByte2;
		wTemp |= (USHORT) (bTempByte1 << 8);
		*((volatile WORD *)(dwBaseAddr + offset)) = wTemp;
		offset += 2;
	}
	/*  If number of bytes requested is ODD, write last byte */
	if (ulNumBytes % 2) {
		wTemp = *((volatile WORD *)(dwBaseAddr + offset));
		wTemp &= 0xFF;
		wTemp |= (USHORT)((*pbHostBuf) << 8);
		*((volatile WORD *)(dwBaseAddr + offset)) = wTemp;
	}
#endif
	return (status);
}

/*
 *  ======== WriteExtDspData ========
 *  purpose:
 *      Copies buffers to the external memory.
 *
 */
DSP_STATUS WriteExtDspData(struct WMD_DEV_CONTEXT *pDevContext,
			  IN BYTE *pbHostBuf, DWORD dwDSPAddr, ULONG ulNumBytes,
			  ULONG ulMemType, BOOL bDynamicLoad)
{
	DWORD dwBaseAddr = pDevContext->dwDspExtBaseAddr;
	DWORD dwOffset;
	BYTE bTempByte1, bTempByte2;
	BYTE remainByte[4];
	INT i;
	DSP_STATUS retVal = DSP_SOK;
	DWORD dwExtProgVirtMem;
	ULONG ulShmBaseVirt = 0;
	ULONG ulTLBBaseVirt = 0;
	ULONG ulShmOffsetVirt = 0;
	struct CFG_HOSTRES hostRes;
#if !defined(OMAP_2430) && !defined(OMAP_3430)
	INT remainBytes;
	DWORD myStart;
	ULONG bytes = 0;
	USHORT wTemp1, wTemp2;
	DWORD dwTemp;
	INT remain = 0;
	DWORD dw_mod_4 = 0;
#endif
	ULONG ulTraceSecBeg = 0;
	ULONG ulTraceSecEnd = 0;
	BOOL bTraceLoad = FALSE;
	bTempByte1 = 0x0;
	bTempByte2 = 0x0;
	DBG_Trace(DBG_ENTER, "Entered WriteExtDspData dwDSPAddr 0x%x "
		 "ulNumBytes 0x%x \n", dwDSPAddr, ulNumBytes);
	/* Check if it is a load to Trace section */
	retVal = DEV_GetSymbol(pDevContext->hDevObject, DSP_TRACESEC_BEG,
				&ulTraceSecBeg);
	if (DSP_SUCCEEDED(retVal)) {
		retVal = DEV_GetSymbol(pDevContext->hDevObject,
			 DSP_TRACESEC_END, &ulTraceSecEnd);
		if (DSP_SUCCEEDED(retVal)) {
			if ((dwDSPAddr <= ulTraceSecEnd) &&
			   (dwDSPAddr >= ulTraceSecBeg)) {
				DBG_Trace(DBG_LEVEL5, "Writing to DSP Trace "
					 "section 0x%x \n", dwDSPAddr);
				bTraceLoad = TRUE;
			}
		}
	}
	/* If dynamic, force remap/unmap */
	if ((bDynamicLoad || bTraceLoad) && dwBaseAddr) {
		dwBaseAddr = 0;
		MEM_UnmapLinearAddress((PVOID)pDevContext->dwDspExtBaseAddr);
		pDevContext->dwDspExtBaseAddr = 0x0;
	}
	if (!dwBaseAddr) {
		/* Get SHM_BEG  EXT_BEG and EXT_END. */
		retVal = DEV_GetSymbol(pDevContext->hDevObject, SHMBASENAME,
					&ulShmBaseVirt);
		DBC_Assert(ulShmBaseVirt != 0);
		if (bDynamicLoad) {
			if (DSP_SUCCEEDED(retVal)) {
				retVal = DEV_GetSymbol(pDevContext->hDevObject,
						      DYNEXTBASE, &ulExtBase);
			}
			DBC_Assert(ulExtBase != 0);
			if (DSP_SUCCEEDED(retVal)) {
				/* DR  OMAPS00013235 : DLModules array may be
				 * in EXTMEM. It is expected that DYNEXTMEM and
				 * EXTMEM are contiguous, so checking for the
				 * upper bound at EXTEND should be Ok. */
#if !defined(OMAP_2430) && !defined(OMAP_3430)
				retVal = DEV_GetSymbol(pDevContext->hDevObject,
					 DYNEXTEND, &ulExtEnd);
#else
				retVal = DEV_GetSymbol(pDevContext->hDevObject,
					 EXTEND, &ulExtEnd);
#endif
			}
		} else {
			if (DSP_SUCCEEDED(retVal)) {
				retVal = DEV_GetSymbol(pDevContext->hDevObject,
					 EXTBASE, &ulExtBase);
			}
			DBC_Assert(ulExtBase != 0);
			if (DSP_SUCCEEDED(retVal)) {
				retVal = DEV_GetSymbol(pDevContext->hDevObject,
					 EXTEND, &ulExtEnd);
			}
		}
		/* Trace buffer it right after the SHM SEG0, so set the
		 * 	base address to SHMBASE */
		if (bTraceLoad) {
			ulExtBase = ulShmBaseVirt;
		}
		DBC_Assert(ulExtEnd != 0);
		DBC_Assert(ulExtEnd > ulExtBase);
		if (ulExtEnd < ulExtBase) {
			retVal = DSP_EFAIL;
		}
#if !defined(OMAP_2430) && !defined(OMAP_3430)
		/* Convert word addresses to byte addresses */
		ulShmBaseVirt *= DSPWORDSIZE;
		ulExtBase *= DSPWORDSIZE;
		ulExtEnd *= DSPWORDSIZE;
#endif
		if (DSP_SUCCEEDED(retVal)) {
			ulTLBBaseVirt = pDevContext->aTLBEntry[0].ulDspVa *
					DSPWORDSIZE;
			DBC_Assert(ulTLBBaseVirt <= ulShmBaseVirt);
#if defined(OMAP_2430) || defined(OMAP_3430)
			if (DSP_SUCCEEDED(retVal)) {
				retVal = DEV_GetSymbol(pDevContext->hDevObject,
					 DSP_TRACESEC_END, &ulShm0End);
			}
			if (DSP_SUCCEEDED(retVal)) {
				retVal = DEV_GetSymbol(pDevContext->hDevObject,
					 DYNEXTBASE, &ulDynExtBase);
			}
			ulShmOffsetVirt = ulShmBaseVirt - ulTLBBaseVirt;
			if (bTraceLoad) {
				dwExtProgVirtMem = pDevContext->aTLBEntry[0].
						   ulGppVa;
			} else {
				CFG_GetHostResources(
					(struct CFG_DEVNODE *)
					DRV_GetFirstDevExtension(), &hostRes);
				dwExtProgVirtMem = hostRes.dwMemBase[1];
				dwExtProgVirtMem += (ulExtBase - ulDynExtBase);
			}
			DBG_Trace(DBG_LEVEL7, "WriteExtDspData: GPP VA "
				 "pointing to EXTMEMBASE 0x%x \n",
				 dwExtProgVirtMem);
#else
			ulShmOffsetVirt = ulShmBaseVirt - ulTLBBaseVirt;
			/* The code starts after EXT_BEG till EXT_END */
			dwExtProgVirtMem = ulExtBase - ulTLBBaseVirt;
			dwExtProgVirtMem += pDevContext->aTLBEntry[0].ulGppVa;
#endif
#if !defined(OMAP_2430) && !defined(OMAP_3430)
			/* if the ExtBase is not DWORD aligned, we align this
			 * virtual address to a DWORD boundary */
			if (dwExtProgVirtMem % 4) {
				dwExtProgVirtMem -= 2;
				dwBaseAddr += 2;
			}
#endif
			pDevContext->dwDspExtBaseAddr =
				(DWORD)MEM_LinearAddress((VOID *)
				TO_VIRTUAL_UNCACHED(dwExtProgVirtMem), ulExtEnd
				- ulExtBase);
			dwBaseAddr += pDevContext->dwDspExtBaseAddr;
			/* This dwDspExtBaseAddr will get cleared only when
			 * the board is stopped.  */
			if (!pDevContext->dwDspExtBaseAddr) {
				retVal = DSP_EFAIL;
				DBG_Trace(DBG_LEVEL7, "WriteExtDspData: failed "
					 "to Map the program memory\n");
			}
		}
	}
	if (!dwBaseAddr || !ulExtBase || !ulExtEnd) {
		DBG_Trace(DBG_LEVEL7, "Symbols missing for Ext Prog loading\n");
		retVal = DSP_EFAIL;
	}
	if (DSP_SUCCEEDED(retVal)) {
		for (i = 0; i < 4; i++) {
			remainByte[i] = 0x0;
		}
		dwOffset = dwDSPAddr - ulExtBase;
		/* Also make sure the dwDSPAddr is < ulExtEnd */
		if (dwDSPAddr > ulExtEnd || dwOffset > dwDSPAddr) {
			DBG_Trace(DBG_LEVEL7, "We can not load at this address "
				 "dwDSPAddr=0x%x, ulExt/DynBase=0x%x, "
				 "ulExtEnd=0x%x\n", dwDSPAddr, ulExtBase,
				 ulExtEnd);
			retVal = DSP_EFAIL;
		}
	}
#if defined(OMAP_2430) || defined(OMAP_3430)
	if (DSP_SUCCEEDED(retVal)) {
		memcpy((BYTE *) dwBaseAddr + dwOffset, pbHostBuf, ulNumBytes);
	}
#else
	/* Makesure that the following conditions are handled as well :
	 *   -   dwDSPAddr is not at odd location.
	 *   -   the dwOffset is not DWORD aligned.  */
	if (DSP_SUCCEEDED(retVal)) {
		dw_mod_4 = (dwOffset) % 4;
		if (dw_mod_4 > 0) {
			myStart = dwOffset - dw_mod_4;
			if (pDevContext->tcWordSwapOn) {
				remainBytes = (4 - dw_mod_4);
				if (remainBytes == 3) {
					*(BYTE *)(dwBaseAddr + myStart + 0) =
							*pbHostBuf++;
					*(BYTE *)(dwBaseAddr + myStart + 3) =
							*pbHostBuf++;
					*(BYTE *)(dwBaseAddr + myStart + 2) =
							*pbHostBuf++;
				}
				if (remainBytes == 2) {
					*(BYTE *)(dwBaseAddr + myStart + 3) =
							*pbHostBuf++;
					*(BYTE *)(dwBaseAddr + myStart + 2) =
							*pbHostBuf++;
				}
				if (remainBytes == 1) {
					*(BYTE *)(dwBaseAddr + myStart + 2) =
							*pbHostBuf++;
				}
			} else {
				for (i = dw_mod_4; i < 4; i++) {
					*(BYTE *)(dwBaseAddr + myStart + 3 - i)
							= *pbHostBuf++;
				}
			}
			bytes += (4 - dw_mod_4);
			dwOffset += bytes;
		}
	}
	for (i = 0; i < 4; i++) {
		remainByte[i] = 0x0;
	}
	while (bytes < ulNumBytes) {
		/*  This is the terminating condition - i.e if there are less
		 *  than four bytes left.*/
		if ((remain = ulNumBytes - bytes) < 4) {
			if (pDevContext->tcWordSwapOn) {
				remainBytes = remain;
				if (remainBytes == 3) {
					*(BYTE *)(dwBaseAddr + dwOffset + 1) =
							*pbHostBuf++;
					*(BYTE *)(dwBaseAddr + dwOffset + 0) =
							*pbHostBuf++;
					*(BYTE *)(dwBaseAddr + dwOffset + 3) =
							*pbHostBuf++;
				}
				if (remainBytes == 2) {
					*(BYTE *)(dwBaseAddr + dwOffset + 1) =
							*pbHostBuf++;
					*(BYTE *)(dwBaseAddr + dwOffset + 0) =
							*pbHostBuf++;
				}
				if (remainBytes == 1) {
					*(BYTE *)(dwBaseAddr + dwOffset + 1) =
							*pbHostBuf++;
				}
			} else {
				for (i = 0; i < remain; i++) {
					*(BYTE *)
					(dwBaseAddr + dwOffset + 3 - i) =
					*pbHostBuf++;
				}
			}
			bytes += remain;
			break;
		}
		/*  first two bytes */
		bTempByte1 = *pbHostBuf++;
		bTempByte2 = *pbHostBuf++;
		/*  swap two bytes and place in wTemp1 */
		wTemp1 = (USHORT) bTempByte2;
		wTemp1 |= (USHORT) (bTempByte1 << 8);
		/*  next two-bytes */
		bTempByte1 = *pbHostBuf++;
		bTempByte2 = *pbHostBuf++;
		/*  swap next two-bytes and place in wTemp2 */
		wTemp2 = (USHORT) bTempByte2;
		wTemp2 |= (USHORT) (bTempByte1 << 8);
		if (pDevContext->tcWordSwapOn) {
			/*  create 32 bit dwTemp */
			dwTemp = (DWORD) wTemp1;
			dwTemp |= (DWORD) (wTemp2 << 16);
		} else {
			/*  create 32 bit dwTemp with word swapped wTemp1
			 *  and wTemp2 */
			dwTemp = (DWORD) wTemp2;
			dwTemp |= (DWORD) (wTemp1 << 16);
		}
		/* This assertion should never fire as we take care of it */
		DBC_Assert(!((dwBaseAddr + dwOffset) % 4));
		*(volatile DWORD *)(dwBaseAddr + dwOffset) = dwTemp;
		dwOffset += 4;
		bytes += 4;
	}
#endif
	/* Unmap here to force remap for other Ext loads */
	if ((bDynamicLoad || bTraceLoad) && pDevContext->dwDspExtBaseAddr) {
		MEM_UnmapLinearAddress((PVOID) pDevContext->dwDspExtBaseAddr);
		pDevContext->dwDspExtBaseAddr = 0x0;
	}
	return (retVal);
}

