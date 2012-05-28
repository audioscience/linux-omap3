#ifndef __CPSW_LW_H__
#define __CPSW_LW_H__

#ifdef __KERNEL__
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/sockios.h>
#else
#define SIOCDEVPRIVATE	0x89F0	/* to 89FF */
#endif /* __KERNEL__ */

/* If the assert fails, compiler complains
   something like size of array `msg' is negative.
   Unlike linux BUILD_BUG_ON, this works outside function scope.
*/
#define compile_time_assert(cond, msg) \
    typedef char ASSERT_##msg[(cond) ? 1 : -1]

#define CPSW_LW_IOCTL (SIOCDEVPRIVATE+1)

#define CPSW_LW_DRVMSG_BASE 0x01
#define CPSW_LW_FWMSG_BASE 0x40

/* MSM = Message Status Mask */
#define CPSW_LW_MSM_FREE (1 << 0)
#define CPSW_LW_MSM_PREP (1 << 1)
#define CPSW_LW_MSM_POST (1 << 2)
#define CPSW_LW_MSM_PROC (1 << 3)
#define CPSW_LW_MSM_DONE (1 << 4)

enum cpsw_lw_drv_state {
	CPSW_LW_DRV_INVALID = 0,
	CPSW_LW_DRV_INIT,
	CPSW_LW_DRV_READY,
	CPSW_LW_DRV_STARTING,
	CPSW_LW_DRV_RUNNING,
	CPSW_LW_DRV_STOPPING,
	CPSW_LW_DRV_ERROR,
};

enum cpsw_lw_fw_state {
	CPSW_LW_FW_INVALID = 0,
	CPSW_LW_FW_INIT,
	CPSW_LW_FW_IDLE,
	CPSW_LW_FW_RUNNING,
	CPSW_LW_FW_ERROR,
};

enum cpsw_lw_drv_mode {
	CPSW_ASI_MODE_INVALID = 0,
	CPSW_ASI_MODE_NORMAL,
	CPSW_ASI_MODE_LW,
};

enum cpsw_drv_cmd {
	CPSW_LW_MSG_INVALID = 0,

	CPSW_LW_DRVMSG_GETVER = CPSW_LW_DRVMSG_BASE,
	CPSW_LW_DRVMSG_GETSTATUS,
	CPSW_LW_DRVMSG_SETNOTIFYPARAMS,
	CPSW_LW_DRVMSG_SETMODE,
	CPSW_LW_DRVMSG_GETMODE,
    CPSW_LW_DRVMSG_LAST,

    CPSW_LW_FWMSG_GETVER = CPSW_LW_FWMSG_BASE,
	CPSW_LW_FWMSG_GETSTATUS,
    CPSW_LW_FWMSG_PING,
    CPSW_LW_FWMSG_WAKE,
    CPSW_LW_FWMSG_LAST,
};

compile_time_assert(CPSW_LW_DRVMSG_LAST < CPSW_LW_FWMSG_BASE, error_in_enum_cpsw_drv_cmd);
compile_time_assert(CPSW_LW_FWMSG_LAST < (1 << 15)-1, cpsw_lw_msg_do_not_fit_in_signed_short_int);

struct cpsw_lw_shmem {
	uint32_t int_flags;

	int16_t slave_status;
	int16_t slave_errno;

	uint32_t h2s_msg_status;
	uint32_t h2s_msg_bufaddr;
	uint32_t s2h_msg_status;
	uint32_t s2h_msg_bufaddr;
};

struct cpsw_lw_mode_cmd {
	uint32_t mode_id;
};

struct cpsw_lw_verdata {
	uint32_t rel_ver;
};

struct cpsw_lw_statusdata {
	uint32_t status;
};

struct cpsw_lw_notifyparams {
	uint16_t proc_id;
	uint16_t line_id;
	uint32_t event_id;
};

union req_data {
    struct cpsw_lw_mode_cmd modecmd;
    struct cpsw_lw_notifyparams notify_params;
};

union res_data {
    struct cpsw_lw_verdata ver_info;
    struct cpsw_lw_statusdata status_info;
};

struct cpsw_lw_msg {
	uint32_t cmd;
	union req_data req;
	union res_data res;
	int32_t msg_errno;
};

#ifdef __KERNEL__
/* Forward declarations */
struct cpsw_lw_info;
struct cpsw_priv;

int cpsw_lw_usermsg(struct cpsw_lw_info *lw_info, struct cpsw_lw_msg *lw_msg);
void cpsw_lw_tx_timeout(struct cpsw_lw_info *lw_info);
netdev_tx_t cpsw_lw_xmit(struct cpsw_lw_info *lw_info,
				struct sk_buff *skb, void *data, size_t len);
enum cpsw_lw_drv_state cpsw_lw_status(struct cpsw_lw_info *lw_info);
enum cpsw_lw_drv_mode cpsw_lw_mode(struct cpsw_lw_info *lw_info);
int cpsw_lw_start(struct cpsw_lw_info *lw_info);
int cpsw_lw_stop(struct cpsw_lw_info *lw_info);
struct cpsw_lw_info* cpsw_lw_create(struct platform_device *pdev, struct net_device *ndev);
void cpsw_lw_destroy(struct net_device *ndev, struct cpsw_lw_info *lw_info);

#endif /* __KERNEL__ */
#endif /* __CPSW_LW_H__ */