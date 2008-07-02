/*
 * dspbridge/src/hal/common/inc/hal_mbox.h
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
 *  ======== hal_mbox.h ========
 *  Description:
 *      HAL Mailbox API and types definitions
 *
 *! Revision History:
 *! ================
 *! 16 Feb 2003 sb: Initial version
 */
#ifndef __MBOX_H
#define __MBOX_H

/*
 * INCLUDE FILES (only if necessary)
 *
 */
#ifdef __cplusplus
extern "C"
{
#endif

/*
 * EXPORTED DEFINITIONS
 */

/*
* DEFINITION:
*
* DESCRIPTION:  Bitmasks for Mailbox interrupt sources
*/

#define HAL_MBOX_INT_NEW_MSG    0x1
#define HAL_MBOX_INT_NOT_FULL   0x2
#define HAL_MBOX_INT_ALL	0x3

/*
* DEFINITION:   HAL_MBOX_MAX_NUM_MESSAGES
*
* DESCRIPTION:  Maximum number of messages that mailbox can hald at a time.
*/

#define HAL_MBOX_MAX_NUM_MESSAGES   4

/*
 * EXPORTED TYPES
 */

/*
* TYPE:	 HAL_MBOX_Id_t
*
* DESCRIPTION:  Enumerated Type used to specify Mail Box Sub Module Id Number
*/
typedef enum HAL_MBOX_Id_label {
    HAL_MBOX_ID_0,
    HAL_MBOX_ID_1,
    HAL_MBOX_ID_2,
    HAL_MBOX_ID_3,
    HAL_MBOX_ID_4,
    HAL_MBOX_ID_5

} HAL_MBOX_Id_t, *pHAL_MBOX_Id_t;

/*
* TYPE:	 HAL_MBOX_UserId_t
*
* DESCRIPTION:  Enumerated Type used to specify Mail box User Id
*/
typedef enum HAL_MBOX_UserId_label {
    HAL_MBOX_U0_ARM,
    HAL_MBOX_U1_DSP1,
    HAL_MBOX_U2_DSP2,
    HAL_MBOX_U3_ARM

} HAL_MBOX_UserId_t, *pHAL_MBOX_UserId_t;

/*
* TYPE:	 MAILBOX_CONTEXT
*
* DESCRIPTION:  Mailbox context settings
*/
struct MAILBOX_CONTEXT {
	UWORD32 sysconfig;
	UWORD32 irqEnable0;
	UWORD32 irqEnable1;
};

/*
 * EXPORTED FUNCTIONS
 */

/*
* FUNCTION      : HAL_MBOX_MsgRead
*
* INPUTS:
*
*   Identifier  : baseAddress
*   Type	: const UWORD32
*   Description : Base Address of instance of Mailbox module
*
*   Identifier  : mailBoxId
*   Type	: const HAL_MBOX_Id_t
*   Description : Mail Box Sub module Id to read
*
* OUTPUTS:
*
*   Identifier  : pReadValue
*   Type	: UWORD32 *const
*   Description : Value read from MailBox
*
* RETURNS:
*
*   Type	: ReturnCode_t
*   Description : RET_OK	      No errors occured
*		 RET_BAD_NULL_PARAM  Address/ptr Paramater was set to 0/NULL
*		 RET_INVALID_ID      Invalid Id used
*		 RET_EMPTY	   Mailbox empty
*
* PURPOSE:      : this function reads a UWORD32 from the sub module message
*		 box Specified. if there are no messages in the mailbox
*		 then and error is returned.
*/
extern HAL_STATUS HAL_MBOX_MsgRead(
		      const UWORD32	 baseAddress,
		      const HAL_MBOX_Id_t   mailBoxId,
		      UWORD32 *const	pReadValue
		  );

/*
* FUNCTION      : HAL_MBOX_MsgWrite
*
* INPUTS:
*
*   Identifier  : baseAddress
*   Type	: const UWORD32
*   Description : Base Address of instance of Mailbox module
*
*   Identifier  : mailBoxId
*   Type	: const HAL_MBOX_Id_t
*   Description : Mail Box Sub module Id to write
*
*   Identifier  : writeValue
*   Type	: const UWORD32
*   Description : Value to write to MailBox
*
* RETURNS:
*
*   Type	: ReturnCode_t
*   Description : RET_OK	      No errors occured
*		 RET_BAD_NULL_PARAM  Address/pointer Paramater was set to 0/NULL
*		 RET_INVALID_ID      Invalid Id used
*
* PURPOSE:      : this function writes a UWORD32 from the sub module message
*		 box Specified.
*/
extern HAL_STATUS HAL_MBOX_MsgWrite(
		      const UWORD32	 baseAddress,
		      const HAL_MBOX_Id_t   mailBoxId,
		      const UWORD32	 writeValue
		  );

/*
* FUNCTION      : HAL_MBOX_IsFull
*
* INPUTS:
*
*   Identifier  : baseAddress
*   Type	: const UWORD32
*   Description : Base Address of instance of Mailbox module
*
*   Identifier  : mailBoxId
*   Type	: const HAL_MBOX_Id_t
*   Description : Mail Box Sub module Id to check
*
* OUTPUTS:
*
*   Identifier  : pIsFull
*   Type	: UWORD32 *const
*   Description : false means mail box not Full
*		 true means mailbox full.
*
* RETURNS:
*
*   Type	: ReturnCode_t
*   Description : RET_OK	      No errors occured
*		 RET_BAD_NULL_PARAM  Address/pointer Paramater was set to 0/NULL
*		 RET_INVALID_ID      Invalid Id used
*
* PURPOSE:      : this function reads the full status register for mailbox.
*/
extern HAL_STATUS HAL_MBOX_IsFull(
		      const UWORD32	 baseAddress,
		      const HAL_MBOX_Id_t   mailBoxId,
		      UWORD32 *const	pIsFull
		  );

/*
* FUNCTION      : HAL_MBOX_NumMsgGet
*
* INPUTS:
*
*   Identifier  : baseAddress
*   Type	: const UWORD32
*   Description : Base Address of instance of Mailbox module
*
*   Identifier  : mailBoxId
*   Type	: const HAL_MBOX_Id_t
*   Description : Mail Box Sub module Id to get num messages
*
* OUTPUTS:
*
*   Identifier  : pNumMsg
*   Type	: UWORD32 *const
*   Description : Number of messages in mailbox
*
* RETURNS:
*
*   Type	: ReturnCode_t
*   Description : RET_OK	      No errors occured
*		 RET_BAD_NULL_PARAM  Address/pointer Paramater was set to 0/NULL
*		 RET_INVALID_ID      Inavlid ID input at parameter
*
* PURPOSE:      : this function gets number of messages in a specified mailbox.
*/
extern HAL_STATUS HAL_MBOX_NumMsgGet(
		      const UWORD32	 baseAddress,
		      const HAL_MBOX_Id_t   mailBoxId,
		      UWORD32 *const	pNumMsg
		  );

/*
* FUNCTION      : HAL_MBOX_EventEnable
*
* INPUTS:
*
*   Identifier  : baseAddress
*   Type	: const UWORD32
*		 RET_BAD_NULL_PARAM  Address/pointer Paramater was set to 0/NULL
*
*   Identifier  : mailBoxId
*   Type	: const HAL_MBOX_Id_t
*   Description : Mail Box Sub module Id to enable
*
*   Identifier  : userId
*   Type	: const HAL_MBOX_UserId_t
*   Description : Mail box User Id to enable
*
*   Identifier  : enableIrq
*   Type	: const UWORD32
*   Description : Irq value to enable
*
* RETURNS:
*
*   Type	: ReturnCode_t
*   Description : RET_OK	      No errors occured
*		 RET_BAD_NULL_PARAM  A Pointer Paramater was set to NULL
*		 RET_INVALID_ID      Invalid Id used
*
* PURPOSE:      : this function enables the specified IRQ.
*/
extern HAL_STATUS HAL_MBOX_EventEnable(
		      const UWORD32	     baseAddress,
		      const HAL_MBOX_Id_t       mailBoxId,
		      const HAL_MBOX_UserId_t   userId,
		      const UWORD32	     events
		  );

/*
* FUNCTION      : HAL_MBOX_EventDisable
*
* INPUTS:
*
*   Identifier  : baseAddress
*   Type	: const UWORD32
*		 RET_BAD_NULL_PARAM  Address/pointer Paramater was set to 0/NULL
*
*   Identifier  : mailBoxId
*   Type	: const HAL_MBOX_Id_t
*   Description : Mail Box Sub module Id to disable
*
*   Identifier  : userId
*   Type	: const HAL_MBOX_UserId_t
*   Description : Mail box User Id to disable
*
*   Identifier  : enableIrq
*   Type	: const UWORD32
*   Description : Irq value to disable
*
* RETURNS:
*
*   Type	: ReturnCode_t
*   Description : RET_OK	      No errors occured
*		 RET_BAD_NULL_PARAM  A Pointer Paramater was set to NULL
*		 RET_INVALID_ID      Invalid Id used
*
* PURPOSE:      : this function disables the specified IRQ.
*/
extern HAL_STATUS HAL_MBOX_EventDisable(
		      const UWORD32	     baseAddress,
		      const HAL_MBOX_Id_t       mailBoxId,
		      const HAL_MBOX_UserId_t   userId,
		      const UWORD32	     events
		  );

/*
* FUNCTION      : HAL_MBOX_EventAck
*
* INPUTS:
*
*   Identifier  : baseAddress
*   Type	: const UWORD32
*   Description : Base Address of instance of Mailbox module
*
*   Identifier  : mailBoxId
*   Type	: const HAL_MBOX_Id_t
*   Description : Mail Box Sub module Id to set
*
*   Identifier  : userId
*   Type	: const HAL_MBOX_UserId_t
*   Description : Mail box User Id to set
*
*   Identifier  : irqStatus
*   Type	: const UWORD32
*   Description : The value to write IRQ status
*
* OUTPUTS:
*
* RETURNS:
*
*   Type	: ReturnCode_t
*   Description : RET_OK	      No errors occured
*		 RET_BAD_NULL_PARAM  Address Paramater was set to 0
*		 RET_INVALID_ID      Invalid Id used
*
* PURPOSE:      : this function sets the status of the specified IRQ.
*/
extern HAL_STATUS HAL_MBOX_EventAck(
		      const UWORD32	      baseAddress,
		      const HAL_MBOX_Id_t	mailBoxId,
		      const HAL_MBOX_UserId_t    userId,
		      const UWORD32	      event
		  );

/*
* FUNCTION      : HAL_MBOX_saveSettings
*
* INPUTS:
*
*   Identifier  : baseAddress
*   Type	: const UWORD32
*   Description : Base Address of instance of Mailbox module
*
*
* RETURNS:
*
*   Type	: ReturnCode_t
*   Description : RET_OK	      No errors occured
*		 RET_BAD_NULL_PARAM  Address/pointer Paramater was set to 0/NULL
*		 RET_INVALID_ID      Invalid Id used
*		 RET_EMPTY	   Mailbox empty
*
* PURPOSE:      : this function saves the context of mailbox
*/
extern HAL_STATUS HAL_MBOX_saveSettings(UWORD32    baseAddres);

/*
* FUNCTION      : HAL_MBOX_restoreSettings
*
* INPUTS:
*
*   Identifier  : baseAddress
*   Type	: const UWORD32
*   Description : Base Address of instance of Mailbox module
*
*
* RETURNS:
*
*   Type	: ReturnCode_t
*   Description : RET_OK	      No errors occured
*		 RET_BAD_NULL_PARAM  Address/pointer Paramater was set to 0/NULL
*		 RET_INVALID_ID      Invalid Id used
*		 RET_EMPTY	   Mailbox empty
*
* PURPOSE:      : this function restores the context of mailbox
*/
extern HAL_STATUS HAL_MBOX_restoreSettings(UWORD32    baseAddres);

#ifdef __cplusplus
}
#endif
#endif  /* __MBOX_H */
