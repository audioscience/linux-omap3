/*
 * dspbridge/src/hw/omap3/mbox/hw_mbox.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
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
 *  ======== hw_mbox.c ========
 *  Description:
 *      Mailbox messaging & configuration API definitions
 *
 *! Revision History:
 *! ================
 *! 16 Feb 2003 sb: Initial version
 */

/*
 * PROJECT SPECIFIC INCLUDE FILES
 */
#include <GlobalTypes.h>
#include "MLBRegAcM.h"
#include <hw_defs.h>
#include <hw_mbox.h>

/*
 * LOCAL TYPES AND DEFINITIONS
 */

/* width in bits of MBOX Id */
#define HW_MBOX_ID_WIDTH	   2

/*
 * FUNCTIONS
 */
struct MAILBOX_CONTEXT mboxsetting = {0, 0, 0};

/*
 *  ======== HW_MBOX_saveSettings ========
 *  purpose:
 *  	Saves the mailbox context
 */
HW_STATUS HW_MBOX_saveSettings(UWORD32    baseAddress)
{
	HW_STATUS status = RET_OK;

	mboxsetting.sysconfig = MLBMAILBOX_SYSCONFIGReadRegister32(baseAddress);
	/* Get current enable status */
	mboxsetting.irqEnable0 = MLBMAILBOX_IRQENABLE___0_3ReadRegister32
				 (baseAddress, HW_MBOX_U0_ARM);
	mboxsetting.irqEnable1 = MLBMAILBOX_IRQENABLE___0_3ReadRegister32
				 (baseAddress, HW_MBOX_U1_DSP1);
	return status;
}

/*
 *  ======== HW_MBOX_restoreSettings ========
 *  purpose:
 *  	Restores the mailbox context
 */
HW_STATUS HW_MBOX_restoreSettings(UWORD32    baseAddress)
{
	 HW_STATUS status = RET_OK;
	/* Restor IRQ enable status */
	MLBMAILBOX_IRQENABLE___0_3WriteRegister32(baseAddress, HW_MBOX_U0_ARM,
						 mboxsetting.irqEnable0);
	MLBMAILBOX_IRQENABLE___0_3WriteRegister32(baseAddress, HW_MBOX_U1_DSP1,
						 mboxsetting.irqEnable1);
	/* Restore Sysconfig register */
	MLBMAILBOX_SYSCONFIGWriteRegister32(baseAddress, mboxsetting.sysconfig);
	return status;
}

/*
 *  ======== HW_MBOX_MsgRead ========
 *  purpose:
 *  	Reads a UWORD32 from the sub module message box Specified. if there
 *  	are no messages in the mailbox then and error is returned.
 */
HW_STATUS HW_MBOX_MsgRead(
		      const UWORD32    baseAddress,
		      const HW_MBOX_Id_t   mailBoxId,
		      UWORD32 *const   pReadValue
		      )
{
    HW_STATUS status = RET_OK;

    /* Check input parameters */
    CHECK_INPUT_PARAM(baseAddress, 0, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
		      RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_PARAM(pReadValue, NULL, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
		      RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_RANGE_MIN0(mailBoxId,
			   HW_MBOX_ID_MAX,
			   RET_INVALID_ID,
			   RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

    /* Read 32-bit message in mail box */
    *pReadValue = MLBMAILBOX_MESSAGE___0_15ReadRegister32(baseAddress,
							 (UWORD32)mailBoxId);

    return status;
}

/*
 *  ======== HW_MBOX_MsgWrite ========
 *  purpose:
 *  	Writes a UWORD32 from the sub module message box Specified.
 */
HW_STATUS HW_MBOX_MsgWrite(const UWORD32 baseAddress,
			     const HW_MBOX_Id_t mailBoxId,
			     const UWORD32   writeValue)
{
    HW_STATUS status = RET_OK;

    /* Check input parameters */
    CHECK_INPUT_PARAM(baseAddress, 0, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
		      RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_RANGE_MIN0(mailBoxId,
			   HW_MBOX_ID_MAX,
			   RET_INVALID_ID,
			   RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

    /* Write 32-bit value to mailbox */
    MLBMAILBOX_MESSAGE___0_15WriteRegister32(baseAddress, (UWORD32)mailBoxId,
					    (UWORD32)writeValue);

    return status;
}

/*
 *  ======== HW_MBOX_IsFull ========
 *  purpose:
 *  	Reads the full status register for mailbox.
 */
HW_STATUS HW_MBOX_IsFull(
		      const UWORD32    baseAddress,
		      const HW_MBOX_Id_t   mailBoxId,
		      UWORD32  *const     pIsFull
		  )
{
    HW_STATUS status = RET_OK;
    UWORD32 fullStatus;

    /* Check input parameters */
    CHECK_INPUT_PARAM(baseAddress, 0, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
		      RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_PARAM(pIsFull,  NULL, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
		      RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_RANGE_MIN0(mailBoxId,
			   HW_MBOX_ID_MAX,
			   RET_INVALID_ID,
			   RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

    /* read the is full status parameter for Mailbox */
    fullStatus = MLBMAILBOX_FIFOSTATUS___0_15FifoFullMBmRead32(baseAddress,
							(UWORD32)mailBoxId);

    /* fill in return parameter */
    *pIsFull = (fullStatus & 0xFF);

    return status;
}

/*
 *  ======== HW_MBOX_NumMsgGet ========
 *  purpose:
 *  	Gets number of messages in a specified mailbox.
 */
HW_STATUS HW_MBOX_NumMsgGet(
		      const   UWORD32   baseAddress,
		      const   HW_MBOX_Id_t  mailBoxId,
		      UWORD32 *const    pNumMsg
		  )
{
    HW_STATUS status = RET_OK;

    /* Check input parameters */
    CHECK_INPUT_PARAM(baseAddress, 0, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
		      RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_PARAM(pNumMsg,  NULL, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
		      RES_INVALID_INPUT_PARAM);

    CHECK_INPUT_RANGE_MIN0(mailBoxId,
			   HW_MBOX_ID_MAX,
			   RET_INVALID_ID,
			   RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

    /* Get number of messages available for MailBox */
    *pNumMsg = MLBMAILBOX_MSGSTATUS___0_15NbOfMsgMBmRead32(baseAddress,
							  (UWORD32)mailBoxId);

    return status;
}

/*
 *  ======== HW_MBOX_EventEnable ========
 *  purpose:
 *  	Enables the specified IRQ.
 */
HW_STATUS HW_MBOX_EventEnable(
		      const UWORD32	     baseAddress,
		      const HW_MBOX_Id_t       mailBoxId,
		  const HW_MBOX_UserId_t   userId,
		  const UWORD32	     events
		      )
{
	HW_STATUS status = RET_OK;
	UWORD32      irqEnableReg;

	/* Check input parameters */
	CHECK_INPUT_PARAM(baseAddress, 0, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
			  RES_INVALID_INPUT_PARAM);
	CHECK_INPUT_RANGE_MIN0(
			 mailBoxId,
			 HW_MBOX_ID_MAX,
			 RET_INVALID_ID,
			 RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
	CHECK_INPUT_RANGE_MIN0(
			 enableIrq,
			 HW_MBOX_INT_MAX,
			 RET_INVALID_ID,
			 RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
	CHECK_INPUT_RANGE_MIN0(
			 userId,
			 HW_MBOX_USER_MAX,
			 RET_INVALID_ID,
			 RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

	/* Get current enable status */
	irqEnableReg = MLBMAILBOX_IRQENABLE___0_3ReadRegister32(baseAddress,
							     (UWORD32)userId);

	/* update enable value */
	irqEnableReg |= ((UWORD32)(events)) << (((UWORD32)(mailBoxId)) *
			HW_MBOX_ID_WIDTH);

	/* write new enable status */
	MLBMAILBOX_IRQENABLE___0_3WriteRegister32(baseAddress, (UWORD32)userId,
						 (UWORD32)irqEnableReg);

	mboxsetting.sysconfig = MLBMAILBOX_SYSCONFIGReadRegister32(baseAddress);
	/* Get current enable status */
	mboxsetting.irqEnable0 = MLBMAILBOX_IRQENABLE___0_3ReadRegister32
				(baseAddress, HW_MBOX_U0_ARM);
	mboxsetting.irqEnable1 = MLBMAILBOX_IRQENABLE___0_3ReadRegister32
				(baseAddress, HW_MBOX_U1_DSP1);
    return status;
}

/*
 *  ======== HW_MBOX_EventDisable ========
 *  purpose:
 *  	Disables the specified IRQ.
 */
HW_STATUS HW_MBOX_EventDisable(
		      const UWORD32	     baseAddress,
		      const HW_MBOX_Id_t       mailBoxId,
		  const HW_MBOX_UserId_t   userId,
		  const UWORD32	     events
		      )
{
    HW_STATUS status = RET_OK;
    UWORD32      irqDisableReg;

    /* Check input parameters */
    CHECK_INPUT_PARAM(baseAddress, 0, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
		      RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_RANGE_MIN0(
		     mailBoxId,
		     HW_MBOX_ID_MAX,
		     RET_INVALID_ID,
		     RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_RANGE_MIN0(
		     disableIrq,
		     HW_MBOX_INT_MAX,
		     RET_INVALID_ID,
		     RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_RANGE_MIN0(
		     userId,
		     HW_MBOX_USER_MAX,
		     RET_INVALID_ID,
		     RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

    /* Get current enable status */
    irqDisableReg = MLBMAILBOX_IRQENABLE___0_3ReadRegister32(baseAddress,
		    (UWORD32)userId);

    /* update enable value */
    irqDisableReg &= ~((UWORD32)(events)) << (((UWORD32)(mailBoxId)) *
		     HW_MBOX_ID_WIDTH);

    /* write new enable status */
    MLBMAILBOX_IRQENABLE___0_3WriteRegister32(baseAddress, (UWORD32)userId,
					     (UWORD32)irqDisableReg);

    return status;
}

/*
 *  ======== HW_MBOX_EventAck ========
 *  purpose:
 *  	Sets the status of the specified IRQ.
 */
HW_STATUS HW_MBOX_EventAck(
		      const UWORD32	  baseAddress,
		      const HW_MBOX_Id_t	mailBoxId,
		  const HW_MBOX_UserId_t    userId,
		  const UWORD32	      event
		     )
{
    HW_STATUS status = RET_OK;
    UWORD32      irqStatusReg;

    /* Check input parameters */
    CHECK_INPUT_PARAM(baseAddress,   0, RET_BAD_NULL_PARAM, RES_MBOX_BASE +
		      RES_INVALID_INPUT_PARAM);

    CHECK_INPUT_RANGE_MIN0(
		     irqStatus,
		     HW_MBOX_INT_MAX,
		     RET_INVALID_ID,
		     RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_RANGE_MIN0(
		     mailBoxId,
		     HW_MBOX_ID_MAX,
		     RET_INVALID_ID,
		     RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
    CHECK_INPUT_RANGE_MIN0(
		     userId,
		     HW_MBOX_USER_MAX,
		     RET_INVALID_ID,
		     RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);


    /* calculate status to write */
    irqStatusReg = ((UWORD32)event) << (((UWORD32)(mailBoxId)) *
		   HW_MBOX_ID_WIDTH);

    /* clear Irq Status for specified mailbox/User Id */
    MLBMAILBOX_IRQSTATUS___0_3WriteRegister32(baseAddress, (UWORD32)userId,
					     (UWORD32)irqStatusReg);

    return status;
}
