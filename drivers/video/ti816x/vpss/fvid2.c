/*
 * linux/drivers/video/ti816x/vpss/fvid2.c
 *
 * VPSS FVID2 driver for TI 816X
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

#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/vps_proxyserver.h>
#include <linux/fvid2.h>
#include <linux/vps.h>

/* Syslink Module level headers */
#include <ti/syslink/Std.h>
#include <ti/ipc/MultiProc.h>
#include <ti/ipc/Notify.h>

#include "core.h"

#ifdef DEBUG
#define FVID2DBG(format, ...) \
	if (vpss_debug) \
		printk(KERN_INFO"VPSS_FVID2: " \
			format, ## __VA_ARGS__)

#define FVID2ERR(format, ...) \
	if (vpss_debug) \
		printk(KERN_ERR"VPSS_FVID2: " \
			format, ## __VA_ARGS__)
#else
#define FVID2DBG(format, ...)
#define FVID2ERR(format, ...)
#endif

/*3GRPX + 1 DISPCTRL*/
#define VPS_FVID2_NUM	  4

struct vps_fvid2_ctrl {
	bool					isused;
	u32					firm_ver;
	u32					fvid2handle;
	u32					rmprocid;
	u32					notifyno;
	u32					lineid;
	struct vps_psrvfvid2createparams	*fcrprms;
	u32					fcrprms_phy;
	struct vps_psrvfvid2deleteparams	*fdltprms;
	u32					fdltprms_phy;
	struct vps_psrvfvid2controlparams	*fctrlprms;
	u32					fctrlprms_phy;
	struct vps_psrvfvid2queueparams		*fqprms;
	u32					fqprms_phy;
	struct vps_psrvfvid2dequeueparams	*fdqprms;
	u32					fdqprms_phy;
	struct vps_psrvcallback			*cbprms;
	u32					cbprms_phy;
	struct vps_psrvcommandstruct		*cmdprms;
	u32					cmdprms_phy;
	struct vps_psrverrorcallback		*ecbprms;
	u32					ecbprms_phy;
};

static struct vps_fvid2_ctrl  *fvid2_ctrl[VPS_FVID2_NUM];

/*define the information used by the proxy running in M3*/
#define VPS_FVID2_RESERVED_NOTIFY	4
#define VPS_FVID2_M3_INIT_VALUE      (0xAAAAAAAA)
#define VPS_FVID2_PS_LINEID          0
#define CURRENT_VPS_FIRMWARE_VERSION        (0x01000111)

static void vps_callback(u16 procid,
		  u16 lineid,
		  u32 eventno,
		  void *arg,
		  u32 payload)
{
	struct vps_psrvcallback *appcb = (struct vps_psrvcallback *)payload;
	struct vps_fvid2_ctrl *fctrl = (struct vps_fvid2_ctrl *)arg;

	FVID2DBG("Recived No. %d event at line ID %d from processor %d\n",
		eventno, lineid, procid);
	/*perform all kinds of sanity check*/

	if (payload == 0) {
		FVID2ERR("Payload is empty.\n");
		return;
	}

	if (appcb->appcallback == NULL) {
		FVID2ERR("no callback registered\n");
		return;
	}

	if (arg == NULL) {
		FVID2ERR("empty callback arguments.\n");
		return ;
	}

	if (eventno != fctrl->notifyno) {
		FVID2ERR("received event id %d and expected id %d\n",
			eventno, fctrl->notifyno);
		return;
	}

	if (lineid != fctrl->lineid) {
		FVID2ERR("received lineid %d and expected lineid %d\n",
			lineid, fctrl->lineid);
	}

	if (procid != fctrl->rmprocid) {
		FVID2ERR("received from processor %d and  \
					expected from processor %d\n",
			procid, fctrl->rmprocid);
		return;
	}
	/*dispatch the callback*/
	if (appcb->cbtype == VPS_FVID2_IO_CALLBACK)
		appcb->appcallback(arg, appcb->appdata, NULL);

}

static struct vps_fvid2_ctrl *vps_get_fvid2_ctrl(void)
{
	int i;
	for (i = 0; i < VPS_FVID2_NUM; i++) {
		if (false == fvid2_ctrl[i]->isused) {
			fvid2_ctrl[i]->isused = true;
			return fvid2_ctrl[i];
		}
   }

	return NULL;
}

static int vps_check_fvid2_ctrl(void *handle)
{
	int i;
	for (i = 0; i < VPS_FVID2_NUM; i++) {
		if (((u32)handle == (u32)fvid2_ctrl[i]) &&
			(fvid2_ctrl[i]->isused == true))
			return 0;
	}

	return 1;
}
 void *vps_fvid2_create(u32 drvid,
			u32 instanceid,
			void *createargs,
			void *createstatusargs,
			struct fvid2_cbparams *cbparams)
{

	struct vps_fvid2_ctrl *fctrl = NULL;
	int status;
	int j = 0;
	fctrl = vps_get_fvid2_ctrl();
	if (fctrl == NULL)
		return NULL;
	/*assembel the create parameter structure*/
	fctrl->fcrprms->command = VPS_FVID2_CREATE;
	fctrl->fcrprms->hosttaskinstance = VPS_FVID2_TASK_TYPE_1,
	fctrl->fcrprms->createargs = createargs;
	fctrl->fcrprms->createstatusargs = createstatusargs;
	fctrl->fcrprms->drvid = drvid;
	fctrl->fcrprms->cbparams = cbparams;
	fctrl->fcrprms->instanceid = instanceid;
	fctrl->fcrprms->fvid2handle = (void *)VPS_FVID2_M3_INIT_VALUE;


	if (cbparams == NULL) {
		fctrl->fcrprms->ioreqcb = NULL;
		fctrl->fcrprms->errcb = NULL;
	} else {
		fctrl->fcrprms->ioreqcb =
			(struct vps_psrvcallback *)fctrl->cbprms_phy;
		fctrl->fcrprms->errcb =
			(struct vps_psrverrorcallback *)fctrl->ecbprms_phy;
	}

	fctrl->cmdprms->cmdtype = VPS_FVID2_CMDTYPE_SIMPLEX;
	fctrl->cmdprms->simplexcmdarg = (void *)fctrl->fcrprms_phy;
	/*set the event to M3*/
	status = Notify_sendEvent(fctrl->rmprocid,
				  fctrl->lineid,
				  VPS_FVID2_RESERVED_NOTIFY,
				  fctrl->cmdprms_phy,
				  1);

	if (status < 0) {
		FVID2ERR("send create event failed with status 0x%0x\n",
			  status);
		fctrl->isused = false;
		return NULL;
	}

	while ((fctrl->fcrprms->fvid2handle ==
	    (void *)VPS_FVID2_M3_INIT_VALUE)) {
		schedule();
		j++;
	}

	fctrl->notifyno = fctrl->fcrprms->syslnkntyno;
	fctrl->fvid2handle = (u32)fctrl->fcrprms->fvid2handle;
	FVID2DBG("Fvid2 handle 0x%08x with notifyno %d at %d\n",
		 (u32)fctrl->fvid2handle, fctrl->notifyno, j);

	if (fctrl->fvid2handle == 0) {
		fctrl->isused = false;
		return NULL;
	}
	/*register the callback if successfully*/
	if (cbparams != NULL) {
		status = Notify_registerEvent(fctrl->rmprocid,
				      fctrl->lineid,
				      fctrl->notifyno,
				      (Notify_FnNotifyCbck)vps_callback,
				      (void *)fctrl);

		if (status < 0) {
			FVID2ERR("register event status 0x%08x\n", status);
			fctrl->isused = false;
			return NULL;
		}
	}
	return (void *)fctrl;
}
EXPORT_SYMBOL(vps_fvid2_create);

int vps_fvid2_delete(void *handle, void *deleteargs)
{

	struct vps_fvid2_ctrl *fctrl = (struct vps_fvid2_ctrl *)handle;
	int status;
	int j = 0;

	FVID2DBG("enter delete.\n");
	if (vps_check_fvid2_ctrl(handle))
		return -EINVAL;

	fctrl->fdltprms->command = VPS_FVID2_DELETE;
	fctrl->fdltprms->fvid2handle = (void *)fctrl->fvid2handle;
	fctrl->fdltprms->deleteargs = deleteargs;
	fctrl->fdltprms->returnvalue = VPS_FVID2_M3_INIT_VALUE;


	fctrl->cmdprms->cmdtype = VPS_FVID2_CMDTYPE_SIMPLEX;
	fctrl->cmdprms->simplexcmdarg = (void *)fctrl->fdltprms_phy;

	/*send event to proxy in M3*/
	status = Notify_sendEvent(fctrl->rmprocid,
				  fctrl->lineid,
				  fctrl->notifyno,
				  fctrl->cmdprms_phy,
				  1);

	if (status < 0) {
		FVID2ERR("set delete event failed status 0x%08x\n", status);
		return -EINVAL;
	} else {

		while ((fctrl->fdltprms->returnvalue ==
			VPS_FVID2_M3_INIT_VALUE)) {

			schedule();
			j++;
		}

		FVID2DBG("delete event return %d at %d\n",
			 fctrl->fdltprms->returnvalue, j);

	}
	if (fctrl->fcrprms->cbparams != NULL) {
		status = Notify_unregisterEvent(
					fctrl->rmprocid,
					fctrl->lineid,
					fctrl->notifyno,
					(Notify_FnNotifyCbck)vps_callback,
					(void *)fctrl);

		if (status < 0)
			FVID2ERR("unregister Event status 0x%08x\n", status);
	}

	fctrl->isused = false;
	fctrl->fvid2handle = 0;
	return fctrl->fdltprms->returnvalue;

}
EXPORT_SYMBOL(vps_fvid2_delete);

int vps_fvid2_control(void *handle,
		      u32 cmd,
		      void *cmdargs,
		      void *cmdstatusargs)
{

	int j = 0;
	struct vps_fvid2_ctrl *fctrl = (struct vps_fvid2_ctrl *)handle;
	int status;
	if (vps_check_fvid2_ctrl(handle))
		return -EINVAL;

	FVID2DBG("send control with cmd 0x%08x\n", cmd);
	/*assembel the structure*/
	fctrl->fctrlprms->command = VPS_FVID2_CONTROL;
	fctrl->fctrlprms->fvid2handle = (void *)fctrl->fvid2handle;
	fctrl->fctrlprms->cmd = cmd;
	fctrl->fctrlprms->cmdargs = cmdargs;
	fctrl->fctrlprms->cmdstatusargs = cmdstatusargs;
	fctrl->fctrlprms->returnvalue = VPS_FVID2_M3_INIT_VALUE;

	fctrl->cmdprms->cmdtype = VPS_FVID2_CMDTYPE_SIMPLEX;
	fctrl->cmdprms->simplexcmdarg = (void *)fctrl->fctrlprms_phy;

	/*send the event*/
	status = Notify_sendEvent(fctrl->rmprocid,
				  fctrl->lineid,
				  fctrl->notifyno,
				  fctrl->cmdprms_phy,
				  1);

	if (status < 0) {
		FVID2ERR("send control with cmd 0x%08x status 0x%08x\n",
			  cmd, status);
	} else {
		while (fctrl->fctrlprms->returnvalue ==
			VPS_FVID2_M3_INIT_VALUE) {

			schedule();
			j++;
		}
	}

	FVID2DBG("control event return %d at %d.\n",
		fctrl->fctrlprms->returnvalue, j);

	return fctrl->fctrlprms->returnvalue;

}
EXPORT_SYMBOL(vps_fvid2_control);

int vps_fvid2_queue(void *handle,
				  struct fvid2_framelist *framelist,
				  u32 streamid)
{

	int j = 0;
	struct vps_fvid2_ctrl *fctrl = (struct vps_fvid2_ctrl *)handle;
	int status;
	if (vps_check_fvid2_ctrl(handle)) {
		FVID2ERR("Q handle 0x%08x\n", (u32)handle);
		return -EINVAL;
	}
	/*assemble the structure*/
	fctrl->fqprms->command = VPS_FVID2_QUEUE;
	fctrl->fqprms->fvid2handle = (void *)fctrl->fvid2handle;
	fctrl->fqprms->framelist = framelist;
	fctrl->fqprms->streamid = streamid;
	fctrl->fqprms->returnvalue = VPS_FVID2_M3_INIT_VALUE;

	fctrl->cmdprms->cmdtype = VPS_FVID2_CMDTYPE_SIMPLEX;
	fctrl->cmdprms->simplexcmdarg = (void *)fctrl->fqprms_phy;

	/* send event to proxy in M3*/
	status = Notify_sendEvent(fctrl->rmprocid,
				  fctrl->lineid,
				  fctrl->notifyno,
				  fctrl->cmdprms_phy,
				  1);

	if (status < 0) {
		FVID2ERR("send Q event status 0x%08x\n", status);
		return -EINVAL;
	} else {
		while (fctrl->fqprms->returnvalue ==
			VPS_FVID2_M3_INIT_VALUE) {

			schedule();
			j++;
		}
	}

	if (fctrl->fqprms->returnvalue == VPS_FVID2_M3_INIT_VALUE) {
		FVID2DBG("queue event timeout.\n");
	} else {
		FVID2DBG("queue event return %d at %d.\n",
			fctrl->fqprms->returnvalue, j);
	}
	return fctrl->fqprms->returnvalue;
}
EXPORT_SYMBOL(vps_fvid2_queue);

int vps_fvid2_dequeue(void *handle,
		      struct fvid2_framelist *framelist,
		      u32 stream_id,
		      u32 timeout)
{
	int j = 0;
	struct vps_fvid2_ctrl *fctrl = (struct vps_fvid2_ctrl *)handle;
	int status;
	if (vps_check_fvid2_ctrl(handle)) {
		FVID2ERR("DQ handle NULL.\n");
		return -EINVAL;
	}

	/*assembel the structure*/
	fctrl->fdqprms->command = VPS_FVID2_DEQUEUE;
	fctrl->fdqprms->framelist = framelist;
	fctrl->fdqprms->streamid = stream_id;
	fctrl->fdqprms->fvid2handle = (void *)fctrl->fvid2handle;
	fctrl->fdqprms->timeout = timeout;
	fctrl->fdqprms->returnvalue = VPS_FVID2_M3_INIT_VALUE;

	fctrl->cmdprms->cmdtype = VPS_FVID2_CMDTYPE_SIMPLEX;
	fctrl->cmdprms->simplexcmdarg = (void *)fctrl->fdqprms_phy;


	/* send event to proxy in M3*/
	status = Notify_sendEvent(fctrl->rmprocid,
				  fctrl->lineid,
				  fctrl->notifyno,
				  fctrl->cmdprms_phy,
				  1);
	if (status < 0) {
		FVID2ERR("send DQ event status 0x%08x\n", status);
		return -EINVAL;
	} else {
		while (fctrl->fdqprms->returnvalue ==
			VPS_FVID2_M3_INIT_VALUE) {

			schedule();
			j++;
		}
	}

	FVID2DBG("deqieie event return %d at %d\n",
		fctrl->fdqprms->returnvalue, j);

	return fctrl->fdqprms->returnvalue;
}
EXPORT_SYMBOL(vps_fvid2_dequeue);


static int get_firmware_version(int *version, u32 procid)
{
	struct vps_psrvcommandstruct  *cmdstruct;
	u32    cmdstruct_phy;
	struct vps_psrvgetstatusvercmdparams *verparams;
	u32 verparams_phy;
	int status;
	int r = -1;
	/*get the M3 version number*/
	cmdstruct = kzalloc(sizeof(struct vps_psrvcommandstruct),
			    GFP_KERNEL);

	BUG_ON(cmdstruct == NULL);
	cmdstruct_phy = virt_to_phys(cmdstruct);

	verparams = kzalloc(sizeof(struct vps_psrvgetstatusvercmdparams),
		GFP_KERNEL);
	BUG_ON(verparams == NULL);
	verparams_phy = virt_to_phys(verparams);
	/*init the structure*/
	cmdstruct->cmdtype = VPS_FVID2_CMDTYPE_SIMPLEX;
	cmdstruct->simplexcmdarg = (void *)verparams_phy;

	verparams->command = VPS_FVID2_GET_FIRMWARE_VERSION;
	verparams->returnvalue = VPS_FVID2_M3_INIT_VALUE;
	while (r != 0) {
		status = Notify_sendEvent(procid,
				 VPS_FVID2_PS_LINEID,
				 VPS_FVID2_RESERVED_NOTIFY,
				 cmdstruct_phy,
				 1);

		if (status < 0) {
			FVID2ERR("Get version status %#x.\n", status);
		} else {
			while (verparams->returnvalue ==
					VPS_FVID2_M3_INIT_VALUE)
				schedule();
		}

		r = verparams->returnvalue;
	}
	*version = verparams->version;
	/*release the memory*/
	kfree(verparams);
	kfree(cmdstruct);
	return r;
}

int vps_fvid2_init(void *args)
{
	int i;
	struct vps_fvid2_ctrl *fctrl;
	u32  procid;
	u32 version;
	FVID2DBG("FVID2 INIT\n");

	procid = MultiProc_getId("DSS");
	if (MultiProc_INVALIDID == procid) {
		FVID2ERR("failed to get the M3DSS processor ID.\n");
		return -EINVAL;
	}

	if (get_firmware_version(&version, procid) == 0) {
		if (version != CURRENT_VPS_FIRMWARE_VERSION) {
			if (version < CURRENT_VPS_FIRMWARE_VERSION) {
				FVID2ERR("firmware version is too old, \
					please update firmware version.\n");
				return -EINVAL;
			}

			if (version > CURRENT_VPS_FIRMWARE_VERSION)
				FVID2DBG("firmware version is newer, \
					may not working properly.\n");
		} else
			FVID2DBG("get firmware version 0x%08x.\n", version);
	}

	/*allocate the memory for various command structures*/
	for (i = 0; i < VPS_FVID2_NUM; i++) {
		fctrl = kzalloc(sizeof(struct vps_fvid2_ctrl), GFP_KERNEL);
		fvid2_ctrl[i] = fctrl;

		fctrl->fcrprms =
			kzalloc(sizeof(struct vps_psrvfvid2createparams),
				GFP_KERNEL);
		BUG_ON(fctrl->fcrprms == NULL);
		fctrl->fcrprms_phy = virt_to_phys(fctrl->fcrprms);

		fctrl->fdltprms =
			kzalloc(sizeof(struct vps_psrvfvid2deleteparams),
				GFP_KERNEL);
		BUG_ON(fctrl->fdltprms == NULL);
		fctrl->fdltprms_phy = virt_to_phys(fctrl->fdltprms);

		fctrl->fctrlprms =
			kzalloc(sizeof(struct vps_psrvfvid2controlparams),
				GFP_KERNEL);
		BUG_ON(fctrl->fctrlprms == NULL);
		fctrl->fctrlprms_phy = virt_to_phys(fctrl->fctrlprms);

		fctrl->fqprms =
			kzalloc(sizeof(struct vps_psrvfvid2queueparams),
				GFP_KERNEL);
		BUG_ON(fctrl->fqprms == NULL);
		fctrl->fqprms_phy = virt_to_phys(fctrl->fqprms);

		fctrl->fdqprms =
			kzalloc(sizeof(struct vps_psrvfvid2dequeueparams),
				GFP_KERNEL);
		BUG_ON(fctrl->fdqprms == NULL);
		fctrl->fdqprms_phy = virt_to_phys(fctrl->fdqprms);

		fctrl->cbprms =
			kzalloc(sizeof(struct vps_psrvcallback),
				GFP_KERNEL);
		BUG_ON(fctrl->cbprms == NULL);
		fctrl->cbprms_phy = virt_to_phys(fctrl->cbprms);

		fctrl->ecbprms =
			kzalloc(sizeof(struct vps_psrverrorcallback),
				GFP_KERNEL);
		BUG_ON(fctrl->ecbprms == NULL);
		fctrl->ecbprms_phy = virt_to_phys(fctrl->ecbprms);

		fctrl->cmdprms =
			kzalloc(sizeof(struct vps_psrvcommandstruct),
				GFP_KERNEL);
		BUG_ON(fctrl->cmdprms == NULL);
		fctrl->cmdprms_phy = virt_to_phys(fctrl->cmdprms);

		fctrl->rmprocid = procid;
		fctrl->lineid = VPS_FVID2_PS_LINEID;
		fctrl->firm_ver = version;

	}
	return 0;
}

void vps_fvid2_deinit(void *args)
{
	int i;
	struct vps_fvid2_ctrl *fctrl;
	FVID2DBG("Fvid2 deinit\n");

	for (i = 0; i <  VPS_FVID2_NUM; i++) {
		fctrl = fvid2_ctrl[i];

		if (fctrl) {

			kfree(fctrl->fcrprms);
			fctrl->fcrprms_phy = 0;

			kfree(fctrl->fdltprms);
			fctrl->fdltprms = 0;

			kfree(fctrl->fctrlprms);
			fctrl->fctrlprms_phy = 0;

			kfree(fctrl->fqprms);
			fctrl->fqprms_phy = 0;

			kfree(fctrl->fdqprms);
			fctrl->fdqprms_phy = 0;

			kfree(fctrl->cbprms);
			fctrl->cbprms_phy = 0;

			kfree(fctrl->ecbprms);
			fctrl->ecbprms_phy = 0;

			kfree(fctrl->cmdprms);
			fctrl->cmdprms_phy = 0;

			kfree(fctrl);
			fvid2_ctrl[i] = NULL;
		}
	}
}
