/*
 * dspbridge/mpu_driver/inc/_chnl_sm.h
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
 *  ======== _chnl_sm.h ========
 *  Description:
 *      Private header file defining channel manager and channel objects for
 *      a shared memory channel driver.
 *
 *  Public Functions:
 *      None.
 *
 *  Notes:
 *      Shared between the modules implementing the shared memory channel class
 *      library.
 *
 *! Revision History:
 *! ================
 *! 15-Oct-2002 kc  Removed legacy PERF code.
 *! 12-Jan-2002 ag  Removed unused gppReqIO & ddmaChnlId DDMA fields.
 *!                 Added zero-copy chnl descriptor array: zchnldesc.
 *! 21-Dec-2001 ag  Moved descPaGpp to private chnl obj from chnl descriptor.
 *! 20-May-2001 ag/jeh Removed fShmSyms field from CHNL_MGR.
 *! 04-Feb-2001 ag  DSP-DMA support added.
 *! 26-Oct-2000 jeh Added arg and resvd to SHM control structure. Added dwArg
 *!                 to CHNL_IRP.
 *! 16-Oct-2000 jeh Removed #ifdef DEBUG from around channel object's cIOCs
 *!                 field, added cIOReqs.
 *! 20-Jan-2000 ag: Incorporated code review comments.
 *! 05-Jan-2000 ag: Text format cleanup.
 *! 03-Nov-1999 ag: Added szEventName[] to CHNL object for name event support.
 *! 02-Nov-1999 ag: _SHM_BEG & _END Syms from COFF now used for IO and SM CLASS.
 *! 27-Oct-1999 jeh Define SHM structure to work for 16-bit targets.
 *! 25-May-1999 jg: Added target side symbol names for share memory buffer
 *! 03-Jan-1997 gp: Added fSharedIRQ field.
 *! 22-Oct-1996 gp: Made dwProcessID a handle.
 *! 09-Sep-1996 gp: Added dwProcessID field to CHNL_OBJECT.
 *! 13-Aug-1996 gp: Created.
 */

#ifndef _CHNL_SM_
#define _CHNL_SM_

#include <wcd.h>
#include <wmd.h>
#include <isr.h>
#include <dpc.h>

#include <list.h>
#include <ntfy.h>

/*
 *  These target side symbols define the beginning and ending addresses
 *  of shared memory buffer. They are defined in the *cfg.cmd file by
 *  cdb code.
 */
#define CHNL_SHARED_BUFFER_BASE_SYM "_SHM_BEG"
#define CHNL_SHARED_BUFFER_LIMIT_SYM "_SHM_END"
#define BRIDGEINIT_BIOSGPTIMER "_BRIDGEINIT_BIOSGPTIMER"
#define BRIDGEINIT_LOADMON_GPTIMER "_BRIDGEINIT_LOADMON_GPTIMER"

#ifndef _CHNL_WORDSIZE
#define _CHNL_WORDSIZE 4	/* default _CHNL_WORDSIZE is 2 bytes/word */
#endif

#ifdef OMAP_3430

#define MAXOPPS 16

struct oppTableEntry {
    Uns voltage;
    Uns frequency;
    Uns minFreq;
    Uns maxFreq;
} ;

struct oppStruct {
 /*  Uns currDspLoad; */
    Uns currOppPt;
    Uns numOppPts;
 /* Uns oppNotifyStatus; */
    struct oppTableEntry oppPoint[MAXOPPS];
} ;

/* Request to MPU */
struct oppRqstStruct {
    Uns rqstDspFreq;
    Uns rqstOppPt;
};

/* Info to MPU */
struct loadMonStruct {
    Uns currDspLoad;
    Uns currDspFreq;
    Uns predDspLoad;
    Uns predDspFreq;
};

#endif

	typedef DWORD SMWORD;

	typedef enum {
		SHM_CURROPP = 0,
		SHM_OPPINFO = 1,
		SHM_GETOPP = 2,		/* Get DSP requested OPP info */
	} SHM_DESCTYPE;

/* Structure in shared between DSP and PC for communication.*/
	struct SHM {
		DWORD dspFreeMask;	/* Written by DSP, read by PC. */
		DWORD hostFreeMask;	/* Written by PC, read by DSP */

		DWORD inputFull;	/* Input channel has unread data. */
		DWORD inputId;	/* Channel for which input is available. */
		DWORD inputSize;	/* Size of data block (in DSP words). */

		DWORD outputFull;	/* Output channel has unread data. */
		DWORD outputId;	/* Channel for which output is available. */
		DWORD outputSize;	/* Size of data block (in DSP words). */

		DWORD arg;	/* Arg for Issue/Reclaim (23 bits for 55x). */
		DWORD resvd;	/* Keep structure size even for 32-bit DSPs */

#ifdef OMAP_3430
		/* Operating Point structure */
		struct oppStruct  oppTableStruct;
		/* Operating Point Request structure */
		struct oppRqstStruct oppRequest;
		/* load monitor information structure*/
		struct loadMonStruct loadMonInfo;
		char dummy[184];             /* padding to 256 byte boundary */
		Uns shm_dbg_var[64];         /* shared memory debug variables */
#endif
	} ;

	/* Channel Manager: only one created per board: */
	struct CHNL_MGR {
		DWORD dwSignature;	/* Used for object validation */
		/* Function interface to WMD */
		struct WMD_DRV_INTERFACE *pIntfFxns;
		struct IO_MGR *hIOMgr;	/* IO manager */
		/* Device this board represents */
		struct DEV_OBJECT *hDevObject;

		/* These fields initialized in WMD_CHNL_Create():    */
		DWORD dwOutputMask; /* Host output channels w/ full buffers */
		DWORD dwLastOutput;	/* Last output channel fired from DPC */
		/* Critical section object handle */
		struct SYNC_CSOBJECT *hCSObj;
		ULONG uWordSize;	/* Size in bytes of DSP word */
		ULONG cChannels;	/* Total number of channels */
		ULONG cOpenChannels;	/* Total number of open channels */
		struct CHNL_OBJECT **apChannel;	/* Array of channels */
		DWORD dwType;	/* Type of channel class library */
		/* If no SHM syms, return for CHNL_Open */
		DSP_STATUS chnlOpenStatus;
	} ;

/*
 *  Channel: up to CHNL_MAXCHANNELS per board or if DSP-DMA supported then
 *     up to CHNL_MAXCHANNELS + CHNL_MAXDDMACHNLS per board.
 */
	struct CHNL_OBJECT {
		DWORD dwSignature;	/* Used for object validation */
		/* Pointer back to channel manager */
		struct CHNL_MGR *pChnlMgr;
		ULONG uId;	/* Channel id */
		DWORD dwState;	/* Current channel state */
		ULONG uMode;	/* Chnl mode and attributes */
		/* Chnl I/O completion event (user mode) */
		HANDLE hUserEvent;
		/* Abstract syncronization object */
		struct SYNC_OBJECT *hSyncEvent;
		/* Name of Sync event */
		char szEventName[SYNC_MAXNAMELENGTH + 1];
		HANDLE hProcess;	/* Process which created this channel */
		ULONG pCBArg;	/* Argument to use with callback */
		struct LST_LIST *pIORequests;	/* List of IOR's to driver */
		LONG cIOCs;	/* Number of IOC's in queue */
		LONG cIOReqs;	/* Number of IORequests in queue */
		LONG cChirps;	/* Initial number of free Irps */
		/* List of IOC's from driver */
		struct LST_LIST *pIOCompletions;
		struct LST_LIST *pFreeList;	/* List of free Irps */
		struct NTFY_OBJECT *hNtfy;
		ULONG cBytesMoved;	/* Total number of bytes transfered */

		/* For DSP-DMA */

		/* Type of chnl transport:CHNL_[PCPY][DDMA] */
		ULONG uChnlType;
	} ;

/* I/O Request/completion packet: */
	struct CHNL_IRP {
		struct LST_ELEM link;	/* Link to next CHIRP in queue. */
		/* Buffer to be filled/emptied. (User)   */
		BYTE *pHostUserBuf;
		/* Buffer to be filled/emptied. (System) */
		BYTE *pHostSysBuf;
		DWORD dwArg;	/* Issue/Reclaim argument.               */
		ULONG uDspAddr;	/* Transfer address on DSP side.         */
		ULONG cBytes;	/* Bytes transferred.                    */
		ULONG cBufSize;	/* Actual buffer size when allocated.    */
		DWORD status;	/* Status of IO completion.              */
	} ;

#endif				/* _CHNL_SM_ */
