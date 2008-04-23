/*
 * dspbridge/src/osal/linux/isr.c
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
 *  ======== isrce.c ========
 *  Purpose:
 *      Interrupt services.
 *
 *  Public Functions:
 *      ISR_Disable
 *      ISR_Exit
 *      ISR_GetStatus
 *      ISR_Init
 *      ISR_Install
 *      ISR_Restore
 *      ISR_SimulateInt
 *      ISR_Uninstall
 *
 *
 *! Revision History:
 *! ================
 *! 06-Feb-2003 kc: Renamed DSP_MAILBOX1 to ISR_MAILBOX1.
 *! 14-Mar-2002 rr: Added HELEN1_V1 flag while installing the interrupt.
 *! 05-Nov-2001 kc: Updated ISR_Install to support multiple HW interrupts.
 *! 27-Jul-2001 rr: Interrupt Id is based on x86 or ARM define.
 *! 24-Apr-2001 ag: Replaced nkintr.h with hal.h.
 *! 10-Oct-2000 rr: CeSetThreadPriority used instead of SetThreadPriority.
 *! 11-Aug-2000 ag: Removed #include <stdwin.h>
 *! 10-Aug-2000 rr: InterruptInitialize happens before the IST creation.
 *! 15-Feb-2000 rr: InterruptInitialize return value checked.
 *! 03-Feb-2000 rr: Module init/exit is handled by OSAL Init/Exit.GT Changes.
 *! 31-Jan-2000 rr: Changes after code review.Terminate thread,handle
 *!                 modified.ISR_UnInstall frees the ISR_Object only on
 *!                 Successful termination of the thread and the handle.
 *! 19-Jan-2000 rr: Code Cleaned up after code review.
 *! 06-Jan-2000 rr: Bus type included in the IRQ object. It is checked
 *!                 during the install and uninstall.
 *! 29-Dec-1999 rr: WaitForSingleObject removed during ISR_UnInstall
 *! 22-Nov-1999 rr: Event gets created before CardRequestIRQ
 *! 05-Nov-1999 rr: ISR_Install does not intialize the interrupt for PCMCIA
 *!                 For other bus type this will happen. IST function return
 *!                 value not checked as anyway we have to say InterruptDone
 *! 29-Oct-1999 rr: Hardware IST is created here.
 *! 25-Oct-1999 rr: New Isr design.
 *! 13-Sep-1999 a0216266: Stubbed from isrnt.c.
 */

/*  ----------------------------------- Host OS */
#include <host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <std.h>
#include <dbdefs.h>
#include <errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <dbc.h>
#include <dbg_zones.h>
#include <gp.h>
#include <gt.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <list.h>
#include <mem.h>
#include <perf.h>

/*  ----------------------------------- This */
#include <isr.h>

/*  ----------------------------------- Defines, Data Structures, Typedefs */
#define SIGNATURE       0x5f525349	/* "ISR_" (in reverse). */

/* The IRQ object, passed to our hardware and virtual interrupt routines: */
struct ISR_IRQ {
	DWORD dwSignature;	/* Used for object validation.   */
	PVOID pRefData;		/* Argument for client's ISR.    */
	ISR_PROC pfnISR;	/* Client's ISR.                 */

	DWORD dwIntrID;		/* hardware intertupt identifier */
#ifndef LINUX
	HANDLE hIntEvent;	/* Handle to Interrupt event */
	HANDLE hISThread;	/* Handle to thread which waits on interrupt */
	DWORD dwBusType;	/* Bus Type of this IRQ */
#endif
};

/*  ----------------------------------- Globals & Defines */
#if GT_TRACE
static struct GT_Mask ISR_DebugMask = { 0, 0 };	/* ISR Debug Mask */
#endif

HANDLE hEvent;
HANDLE hInterruptThread;

/*  ----------------------------------- Function Prototypes */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
static INT HardwareIST(int irq, void *hIRQ, struct pt_regs *pt_regs);
#else
static VOID HardwareIST(int irq, void *hIRQ, struct pt_regs *pt_regs);
#endif

#ifndef LINUX
/*
 *  ======== ISR_Disable ========
 *  Purpose:
 *      In CE, the client should use ISR_Uninstall to disable the interrupt.
 */
VOID ISR_Disable(OUT UINT *pFlags)
{
	DBC_Require(pFlags != NULL);

	GT_1trace(ISR_DebugMask, GT_ENTER,
		  "Entered ISR_Disable, *pFlags:  0x%x\n", *pFlags);
	/* Not implemented in CE */
	/*
	   Linux implementation requires change in this API.
	   To disable single ISR:
	   VOID ISR_Disable(IN ISR_HIRQ pIRQObject)
	   disable_irq (pIRQObject->dwIntrID) ;
	   To disable all ISRs:
	   VOID ISR_Disable(OUT UINT * pFlags, OUT spinlock_t * isr_lock)
	   spin_lock_irqsave (isr_lock, *pFlags) ;
	 */
}
#endif

/*
 *  ======== ISR_Exit ========
 *  Purpose:
 *      Discontinue usage of the ISR module.
 */
VOID ISR_Exit()
{
	GT_0trace(ISR_DebugMask, GT_ENTER, "Entered ISR_Exit\n");
}

#ifndef LINUX
/*
 *  ======== ISR_GetStatus ========
 *  Purpose:
 *      Get ISR status.
 */
DSP_STATUS ISR_GetStatus(struct ISR_IRQ *hIRQ, OUT DWORD *pdwFlags)
{
	DBC_Require(pdwFlags != NULL);

	GT_1trace(ISR_DebugMask, GT_ENTER,
		  "Entered ISR_GetStatus,hIRQ:0x%x\n", hIRQ);

	return (DSP_ENOTIMPL);
}
#endif

/*
 *  ======== ISR_Init ========
 *  Purpose:
 *      Initialize the ISR module's private state.
 */
BOOL ISR_Init()
{
	GT_create(&ISR_DebugMask, "IS");

	GT_0trace(ISR_DebugMask, GT_5CLASS, "Entered ISR_Init\n");

	return (TRUE);
}

/*
 *  ======== ISR_Install ========
 *  Purpose:
 *      Register an ISR for a given IRQ with the system's interrupt manager.
 */
DSP_STATUS ISR_Install(OUT struct ISR_IRQ **phIRQ,
		       IN CONST struct CFG_HOSTRES *pHostConfig,
		       ISR_PROC pfnISR, DWORD dwIntrType, PVOID pRefData)
{
	DSP_STATUS status = DSP_SOK;
	struct ISR_IRQ *pIRQObject = NULL;

	DBC_Require(pHostConfig);
	DBC_Require(pRefData != NULL);

	GT_5trace(ISR_DebugMask, GT_ENTER,
		  "Entered ISR_Install, args:" "\n\thIRQ:"
		  "0x%x\n\tpHostConfig: 0x%x\n\tpfnISR: 0x%x\n" "\tpRefData:"
		  "0x%x\n \tdwIntrType 0x%x\n", phIRQ, pHostConfig, pfnISR,
		  pRefData, dwIntrType);

	if (phIRQ != NULL) {
		*phIRQ = NULL;
		/*
		 *  Allocate an IRQ object to store information allowing our
		 *  interrupt handler to dispatch to the client's interrupt
		 *  routines.
		 */
		MEM_AllocObject(pIRQObject, struct ISR_IRQ, SIGNATURE);
		if (pIRQObject != NULL) {
			/* Fill out the Object: */
			pIRQObject->pRefData = pRefData;
			pIRQObject->pfnISR = pfnISR;
#ifndef LINUX
			pIRQObject->dwBusType = pHostConfig->dwBusType;
#endif
			/* Install different HW interrupts based on interrupt
			 * type. */
				switch (dwIntrType) {
				case ISR_MAILBOX1:
#if defined(OMAP_2430) || defined(OMAP_3430)
				pIRQObject->dwIntrID = INT_MAIL_MPU_IRQ;
#else
				pIRQObject->dwIntrID = INT_DSP_MAILBOX1;
#endif
				GT_0trace(ISR_DebugMask, GT_1CLASS,
					  "Setting intr id for "
					  "ISR_MAILBOX1\n");
				break;

				case ISR_MAILBOX2:
				pIRQObject->dwIntrID = MAIL_U3_MPU_IRQ;

				GT_0trace(ISR_DebugMask, GT_1CLASS,
					  "Setting intr id for "
					  "ISR_MAILBOX2\n");

				break;

				case DSP_MMUFAULT:
#if  defined(OMAP_2430) || defined(OMAP_3430)
				pIRQObject->dwIntrID = INT_DSP_MMU_IRQ;
#else
#ifdef OMAP_1710
				/* INT Level 1, interrupt 7 */
				pIRQObject->dwIntrID = INT_DSP_MMU_ABORT;
#else
				/* INT Level 2, interrupt 28 */
				pIRQObject->dwIntrID = INT_DSP_MMU;
#endif
#endif
				GT_0trace(ISR_DebugMask, GT_1CLASS,
					  "Setting intr id for "
					  "DSP_MMUFAULT\n");
				break;

				default:
				pIRQObject->dwIntrID = (DWORD) 0x00;
				GT_0trace(ISR_DebugMask, GT_1CLASS,
					  "Setting intr id to NULL\n");
				break;
			}

			if (pIRQObject->dwIntrID != 0) {
				status = request_irq(pIRQObject->dwIntrID,
						    &HardwareIST, 0,
						    "DspBridge", pIRQObject);
			} else {
				status = DSP_EINVALIDARG;
			}
		}
		if (DSP_SUCCEEDED(status)) {
			*phIRQ = pIRQObject;
			GT_1trace(ISR_DebugMask, GT_1CLASS,
				  "ISR:IRQ Object 0x%x \n",
				  pIRQObject);
		} else {
			MEM_FreeObject(pIRQObject);
		}
	}
	DBC_Ensure(DSP_SUCCEEDED(status) || (!phIRQ || *phIRQ == NULL));
	return (status);
}

#ifndef LINUX
/*
 *  ======== ISR_Restore ========
 *  Purpose:
 *      In CE, the client should use ISR_Install to restore the interrupt.
 */
VOID ISR_Restore(UINT saveFlags)
{
	GT_1trace(ISR_DebugMask, GT_ENTER,
		  "Entered ISR_Restore, saveFlags: 0x%x\n",
		  saveFlags);
	/*
	   Linux implementation requires change in this API.
	   To restore single ISR
	   VOID ISR_Restore(IN ISR_HIRQ pIRQObject)
	   enable_irq (pIRQObject->dwIntrID) ;
	   To restore all ISRs:
	   VOID ISR_Restore(UINT saveFlags, spinlock_t * isr_lock)
	   spin_unlock_irqrestore (isr_lock, saveFlags) ;
	 */
}

/*
 *  ======== ISR_SimulateInt ========
 *  Purpose:
 *      Simulate an interrupt.
 */
DSP_STATUS ISR_SimulateInt(struct ISR_IRQ *hIRQ)
{
	DSP_STATUS status = DSP_SOK;
	struct ISR_IRQ *pIRQObject = (struct ISR_IRQ *) hIRQ;

	DBC_Require(hIRQ > 0);
	GT_1trace(ISR_DebugMask, GT_ENTER,
		  "Entered ISR_SimulateInt, hIRQ: 0x%x\n",
		  hIRQ);
	if (MEM_IsValidHandle(hIRQ, SIGNATURE)) {
		/* Simulate the interrupt: */
		if (!SetEvent(pIRQObject->hIntEvent)) {
			status = DSP_EFAIL;
		}
	} else {
		status = DSP_EHANDLE;
	}
	return (status);
}
#endif

/*
 *  ======== ISR_Uninstall ========
 *  Purpose:
 *      Deregister the ISR previously installed by ISR_Install().
 *      if it fails we do not delete the IRQ object.
 */
DSP_STATUS ISR_Uninstall(struct ISR_IRQ *hIRQ)
{
	DSP_STATUS status = DSP_SOK;
	struct ISR_IRQ *pIRQObject = (struct ISR_IRQ *)hIRQ;

	DBC_Require(hIRQ > 0);

	GT_1trace(ISR_DebugMask, GT_ENTER,
		  "Entered ISR_Uninstall, hIRQ: 0x%x\n", hIRQ);

	if (MEM_IsValidHandle(hIRQ, SIGNATURE)) {
		/* Linux function to uninstall ISR */
		free_irq(pIRQObject->dwIntrID, pIRQObject);
		pIRQObject->dwIntrID = (DWORD) - 1;
	} else {
		status = DSP_EHANDLE;
	}

	/* Free our IRQ object: */
	if (DSP_SUCCEEDED(status)) {
		MEM_FreeObject(pIRQObject);
		pIRQObject = NULL;
	}

	DBC_Ensure((DSP_SUCCEEDED(status) && pIRQObject == NULL) ||
		   DSP_FAILED(status));

	return (status);
}

/*
 *  ======== HardwareIST ========
 *  Purpose:
 *      Linux calls this IRQ handler on interrupt.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)

static INT HardwareIST(int irq, void *hIRQ, struct pt_regs *pt_regs)
{
	struct ISR_IRQ *pIRQObject = (struct ISR_IRQ *)hIRQ;

	DBC_Require(irq == pIRQObject->dwIntrID);

	/* Call the function registered in ISR_Install */
	(*(pIRQObject->pfnISR)) (pIRQObject->pRefData);

	return IRQ_HANDLED;
}

#else

static VOID HardwareIST(int irq, void *hIRQ, struct pt_regs *pt_regs)
{
	struct ISR_IRQ *pIRQObject = (struct ISR_IRQ *)hIRQ;

	DBC_Require(irq == pIRQObject->dwIntrID);

	/* Call the function registered in ISR_Install */
	(*(pIRQObject->pfnISR))(pIRQObject->pRefData);
}

#endif

