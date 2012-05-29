
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/cpsw.h>

#include <syslink/notify.h>

#include <linux/cpsw_lw.h>
#include "davinci_cpdma.h"

#define LW_NAPI_POLL_WEIGHT 64
#define CPSW_LW_RELVER ((0x00 << 16) | (0x01 << 8))
#define CPSW_LW_MSG_POLL_PERIOD 50
#define CPSW_LW_MSG_TIMEOUT 1000

#define shmem_read(desc, fld)		__raw_readl(&(desc)->fld)
#define shmem_write(desc, fld, v)	__raw_writel((u32)(v), &(desc)->fld)

#define CPSW_LW_DEBUG	(NETIF_MSG_HW		| NETIF_MSG_WOL		| \
			 NETIF_MSG_DRV		| NETIF_MSG_LINK	| \
			 NETIF_MSG_IFUP		| NETIF_MSG_INTR	| \
			 NETIF_MSG_PROBE	| NETIF_MSG_TIMER	| \
			 NETIF_MSG_IFDOWN	| NETIF_MSG_RX_ERR	| \
			 NETIF_MSG_TX_ERR	| NETIF_MSG_TX_DONE	| \
			 NETIF_MSG_PKTDATA	| NETIF_MSG_TX_QUEUED	| \
			 NETIF_MSG_RX_STATUS)

#define msg(level, type, format, ...)				\
do {								\
	if (netif_msg_##type(lw_info) && net_ratelimit())		\
		dev_##level(&lw_info->ndev->dev, format, ## __VA_ARGS__);	\
} while (0)

struct cpsw_lw_info {
	spinlock_t lock;

	struct cpsw_lw_shmem __iomem *shmem;
	dma_addr_t shmem_hw_addr;

	u32 msg_enable;
	wait_queue_head_t wq;

	struct platform_device *pdev;
	struct net_device *ndev;

	struct cpdma_chan *tx_chan, *rx_chan;
	u32 tx_budget; /* in octets */
	u32 tx_timeout_count;

	enum cpsw_lw_drv_state status;
	enum cpsw_lw_drv_mode mode;

	u16 slave_proc_id;
	u16 slave_line_id;
	u32 slave_event_id;

	u16 host_proc_id;
	u16 host_line_id;
	u32 host_event_id;

	struct cpsw_lw_msg host_to_slave;
	struct cpsw_lw_msg slave_to_host;
	dma_addr_t h2s_msg_bufaddr;
	dma_addr_t s2h_msg_bufaddr;
};

static inline int cpsw_lw_prepmsg_slave(struct cpsw_lw_info *lw_info,
					enum cpsw_drv_cmd cmd, void *buf, size_t buf_size);
static inline int cpsw_lw_postmsg_slave(struct cpsw_lw_info *lw_info,
					bool wait_clear);
static inline int cpsw_lw_notify_slave(struct cpsw_lw_info *lw_info,
					bool wait_clear);

/* SysLink and NAPI Callbacks */

/* NOTE: Depending on the context in which cpsw_lw_notification_callback()
 * is called it may be possible (and advantageous) to merge it with 
 * cpsw_lw_poll() thus removing the need to use NAPI at all.
 */

/* No spin_lock(&lw_info->lock) inside this function because lw_info 
 * is only read and we make sure the relevant fields never change while 
 * the callback is registered.
 */
static void cpsw_lw_notification_callback(u16 procid, u16 lineid, u32 eventno, 
			void *arg, u32 payload)
{
	struct cpsw_lw_info *lw_info = arg;
	int num_rx, num_tx;

	BUG_ON(lw_info == NULL);

	if (unlikely(lw_info->status != CPSW_LW_DRV_RUNNING))
		return;

	/* NOTE: payload is currently unused. We could use it to send: 
	 * budget (3 bytes) + message available flags (1 byte); this way
	 * we wouldn't touch uncached memory unless necessary. */

	/* Warn if the TX channel is not done by now */
	if (unlikely(!cpdma_chan_isdone(lw_info->tx_chan)))
		msg(err, intr, "TX queue from the previous slot is still running\n");

	/* Restore the TX queue's budget */
	cpdma_chan_setbudget(lw_info->tx_chan, lw_info->tx_budget);
	/* Restart the outgoing packets queue */
	if (likely(netif_queue_stopped(lw_info->ndev)))
		netif_wake_queue(lw_info->ndev);

	/* Force DMA to restart in case of TX timeout, maybe unneeded because
	 * the following cpdma_chan_process() could take care of it.
	 * At some point we should try to remove this and test.
	 */
	if (unlikely(lw_info->tx_timeout_count)) {
		cpdma_chan_stop(lw_info->tx_chan);
		cpdma_chan_start(lw_info->tx_chan);
		lw_info->tx_timeout_count = 0;
	}

	/* Process all TX buffers. */
	num_tx = cpdma_chan_process(lw_info->tx_chan, INT_MAX);
	num_rx = cpdma_chan_process(lw_info->rx_chan, INT_MAX);

	if (likely(num_rx || num_tx))
		msg(dbg, intr, "poll %d rx, %d tx pkts\n", num_rx, num_tx);

	/* Wake up processes waiting on messaging events */
	if (unlikely(lw_info->shmem->s2h_msg_status == CPSW_LW_MSM_POST ||
		lw_info->shmem->h2s_msg_status == CPSW_LW_MSM_DONE))
		wake_up(&lw_info->wq);
}

void cpsw_lw_tx_timeout(struct cpsw_lw_info *lw_info)
{
	BUG_ON(lw_info == NULL);

	lw_info->tx_timeout_count++;
}

/* Messaging */

static inline int cpsw_lw_prepmsg_slave(struct cpsw_lw_info *lw_info,
					enum cpsw_drv_cmd cmd, void *buf, size_t buf_size)
{
	if (!lw_info)
		return -EINVAL;

	/* buf can be NULL IFF buf_size is also 0 */
	if ((!buf && buf_size != 0) || buf_size > sizeof(union req_data))
		return -EINVAL;

	spin_lock(&lw_info->lock);
	/* If a message is being processed bail out */
	if (shmem_read(lw_info->shmem, h2s_msg_status) != CPSW_LW_MSM_FREE) {
		spin_unlock(&lw_info->lock);
		return -EINPROGRESS;
	}
	shmem_write(lw_info->shmem, h2s_msg_status, CPSW_LW_MSM_PREP);
	spin_unlock(&lw_info->lock);

	memset(&lw_info->host_to_slave.req, 0, sizeof(struct cpsw_lw_msg));
	lw_info->host_to_slave.cmd = cmd;
	if (buf)
		memcpy(&lw_info->host_to_slave.req, buf, buf_size);
	lw_info->host_to_slave.msg_errno = -EBADE;
	return 0;
}

static inline int cpsw_lw_prepmsg_slave_fromptr(struct cpsw_lw_info *lw_info,
					struct cpsw_lw_msg *msg)
{
	if (!lw_info || !msg)
		return -EINVAL;

	spin_lock(&lw_info->lock);
	/* If a message is being processed bail out */
	if (shmem_read(lw_info->shmem, h2s_msg_status) != CPSW_LW_MSM_FREE) {
		spin_unlock(&lw_info->lock);
		return -EINPROGRESS;
	}
	shmem_write(lw_info->shmem, h2s_msg_status, CPSW_LW_MSM_PREP);
	memcpy(&lw_info->host_to_slave, msg, sizeof(struct cpsw_lw_msg));
	spin_unlock(&lw_info->lock);

	return 0;
}

static inline int cpsw_lw_postmsg_slave(struct cpsw_lw_info *lw_info, bool wait_clear)
{
	if (!lw_info)
		return -EINVAL;

	spin_lock(&lw_info->lock);	
	if (shmem_read(lw_info->shmem, h2s_msg_status) != CPSW_LW_MSM_PREP) {
		spin_unlock(&lw_info->lock);
		return -EINPROGRESS;
	}
	dma_sync_single_for_device(&lw_info->pdev->dev, lw_info->h2s_msg_bufaddr,
							sizeof(lw_info->host_to_slave), DMA_TO_DEVICE);
	shmem_write(lw_info->shmem, h2s_msg_status, CPSW_LW_MSM_POST);
	spin_unlock(&lw_info->lock);	

	return cpsw_lw_notify_slave(lw_info, true);
}

static inline int cpsw_lw_recvmsg_slave(struct cpsw_lw_info *lw_info,
									struct cpsw_lw_msg *dest_buf)
{
	if (!lw_info)
		return -EINVAL;

	spin_lock(&lw_info->lock);
	if (shmem_read(lw_info->shmem, h2s_msg_status) != CPSW_LW_MSM_DONE) {
		spin_unlock(&lw_info->lock);
		return -EINPROGRESS;
	}
	dma_sync_single_for_cpu(&lw_info->pdev->dev, lw_info->h2s_msg_bufaddr,
							sizeof(lw_info->host_to_slave), DMA_FROM_DEVICE);
	if (dest_buf) {
		memcpy(dest_buf, &lw_info->host_to_slave, sizeof(*dest_buf));
		shmem_write(lw_info->shmem, h2s_msg_status, CPSW_LW_MSM_FREE);
	}
	spin_unlock(&lw_info->lock);

	return 0;
}

static inline int cpsw_lw_freemsg_slave(struct cpsw_lw_info *lw_info)
{
	if (!lw_info)
		return -EINVAL;

	spin_lock(&lw_info->lock);	
	if (shmem_read(lw_info->shmem, h2s_msg_status) != CPSW_LW_MSM_DONE) {
		spin_unlock(&lw_info->lock);
		return -EINPROGRESS;
	}
	shmem_write(lw_info->shmem, h2s_msg_status, CPSW_LW_MSM_FREE);
	spin_unlock(&lw_info->lock);

	return 0;
}

static inline int cpsw_lw_resetmsg_slave(struct cpsw_lw_info *lw_info)
{
	if (!lw_info)
		return -EINVAL;

	spin_lock(&lw_info->lock);	
	shmem_write(lw_info->shmem, h2s_msg_status, CPSW_LW_MSM_FREE);
	spin_unlock(&lw_info->lock);

	return 0;
}

static inline int cpsw_lw_notify_slave(struct cpsw_lw_info *lw_info, bool wait_clear)
{
	int res;
	
	if (!lw_info)
		return -EINVAL;

	res = notify_send_event(lw_info->slave_proc_id,
				lw_info->slave_line_id,
				lw_info->slave_event_id,
				lw_info->shmem_hw_addr,
				wait_clear);
	return res;
}

static int cpsw_lw_waitres_wq_slave(struct cpsw_lw_info *lw_info,
									long poll_period,
									long poll_timeout)
{
	long res = 0;
	do {
		res = wait_event_interruptible_timeout(lw_info->wq,
					shmem_read(lw_info->shmem, h2s_msg_status) == CPSW_LW_MSM_DONE,
					usecs_to_jiffies(poll_period));
		if (res)
			break;
		poll_timeout -= poll_period;
	} while(poll_timeout > 0);

	if (res == -ERESTARTSYS)
		return -EINTR;
	if (poll_timeout <= 0)
		return -ETIMEDOUT; /* or -ETIME ? */
	return 0;
}

/*  */

int cpsw_lw_usermsg(struct cpsw_lw_info *lw_info, struct cpsw_lw_msg *msg)
{
	int status;

	if (msg->cmd >= CPSW_LW_FWMSG_BASE) {
		/* Send message to firmware */
		if ((status = cpsw_lw_prepmsg_slave_fromptr(lw_info, msg)))
			goto err;
		if ((status = cpsw_lw_postmsg_slave(lw_info, true)))
			goto err;
		if ((status = cpsw_lw_waitres_wq_slave(lw_info,
											CPSW_LW_MSG_POLL_PERIOD,
								 			CPSW_LW_MSG_TIMEOUT)))
			goto err;
		if ((status = cpsw_lw_recvmsg_slave(lw_info, msg)))
			goto err;
		return 0;
err:
		cpsw_lw_resetmsg_slave(lw_info);
		return status;
	}

	/* Driver message */
	switch (msg->cmd) {
		case CPSW_LW_DRVMSG_GETVER:
			msg->res.ver_info.rel_ver = CPSW_LW_RELVER;
			msg->msg_errno = 0;
			break;

		case CPSW_LW_DRVMSG_GETSTATUS:
			msg->res.status_info.status = lw_info->status;
			msg->msg_errno = 0;
			break;

		case CPSW_LW_DRVMSG_GETMODE:
			msg->res.mode_info.mode = lw_info->mode;
			msg->msg_errno = 0;
			break;

		case CPSW_LW_DRVMSG_SETNOTIFYPARAMS:
			spin_lock(&lw_info->lock);
			lw_info->slave_proc_id = msg->req.notify_params.proc_id;
			lw_info->slave_line_id = msg->req.notify_params.line_id;
			lw_info->slave_event_id = msg->req.notify_params.event_id;
			msg->msg_errno = 0;
			spin_unlock(&lw_info->lock);
			break;

		default:
			return -EOPNOTSUPP;
	}

	return 0;
}

/* Lifecycle related functions */

enum cpsw_lw_drv_mode cpsw_lw_mode(struct cpsw_lw_info *lw_info)
{
	int tmp;

	if (!lw_info)
		return -EINVAL;

	spin_lock(&lw_info->lock);
	tmp = lw_info->mode;
	spin_unlock(&lw_info->lock);
	return tmp;
}

enum cpsw_lw_drv_state cpsw_lw_status(struct cpsw_lw_info *lw_info)
{
	int tmp;

	if (!lw_info)
		return -EINVAL;

	spin_lock(&lw_info->lock);
	tmp = lw_info->status;
	spin_unlock(&lw_info->lock);
	return tmp;
}

int cpsw_lw_start(struct cpsw_lw_info *lw_info)
{
	int status;

	spin_lock(&lw_info->lock);
	if (lw_info->status != CPSW_LW_DRV_READY) {
		spin_unlock(&lw_info->lock);
		return -EBUSY;
	}
	lw_info->status = CPSW_LW_DRV_STARTING;
	spin_unlock(&lw_info->lock);

	/* Setup to receive notifications */
	status = notify_register_event(lw_info->host_proc_id,
				lw_info->host_line_id,
				lw_info->host_event_id,
				(notify_fn_notify_cbck)cpsw_lw_notification_callback,
				(void *)lw_info);
	if (status < 0)
		return status;

	status = cpsw_lw_prepmsg_slave(lw_info, CPSW_LW_FWMSG_GETVER, NULL, 0);
	if (status)
		goto err;
	status = cpsw_lw_postmsg_slave(lw_info, true);
	if (status)
		goto err;
	status = cpsw_lw_waitres_wq_slave(lw_info,
										CPSW_LW_MSG_POLL_PERIOD,
										CPSW_LW_MSG_TIMEOUT);
	if (status)
		goto err;
	status = cpsw_lw_freemsg_slave(lw_info);
	if (status)
		goto err;

	/* If major revision number do not match, report an error */
	if ((lw_info->host_to_slave.res.ver_info.rel_ver & 0xFF000000) != 
		(CPSW_LW_RELVER & 0xFF000000)) {
		dev_err(&lw_info->ndev->dev, "Driver ver 0x%08x and firmware ver 0x%08x are incompatible\n",
		CPSW_LW_RELVER, lw_info->host_to_slave.res.ver_info.rel_ver);
		status = -EOPNOTSUPP;
		goto err;
	}

	status = cpsw_lw_prepmsg_slave(lw_info, CPSW_LW_FWMSG_START, NULL, 0);
	if (status)
		goto err;
	status = cpsw_lw_postmsg_slave(lw_info, true);
	if (status)
		goto err;
	status = cpsw_lw_waitres_wq_slave(lw_info,
									CPSW_LW_MSG_POLL_PERIOD,
									CPSW_LW_MSG_TIMEOUT);
	if (status)
		goto err;
	status = cpsw_lw_freemsg_slave(lw_info);
	if (status)
		goto err;

	lw_info->status = CPSW_LW_DRV_RUNNING;
	lw_info->mode = CPSW_ASI_MODE_LW;
	return status;

err:
	cpsw_lw_resetmsg_slave(lw_info);
	cpsw_lw_prepmsg_slave(lw_info, CPSW_LW_FWMSG_STOP, NULL, 0);
	cpsw_lw_postmsg_slave(lw_info, true);
	cpsw_lw_waitres_wq_slave(lw_info, CPSW_LW_MSG_POLL_PERIOD,
											CPSW_LW_MSG_TIMEOUT);
	cpsw_lw_resetmsg_slave(lw_info);

	notify_unregister_event(lw_info->host_proc_id,
				lw_info->host_line_id,
				lw_info->host_event_id,
				(notify_fn_notify_cbck)cpsw_lw_notification_callback,
				(void *)lw_info);
	lw_info->status = CPSW_LW_DRV_READY;
	return status;
}

int cpsw_lw_stop(struct cpsw_lw_info *lw_info)
{
	int status;

	lw_info->status = CPSW_LW_DRV_STOPPING;
	status = cpsw_lw_prepmsg_slave(lw_info, CPSW_LW_FWMSG_STOP, NULL, 0);
	if (status)
		goto err;
	status = cpsw_lw_postmsg_slave(lw_info, true);
	if (status)
		goto err;
	status = cpsw_lw_waitres_wq_slave(lw_info, CPSW_LW_MSG_POLL_PERIOD,
											CPSW_LW_MSG_TIMEOUT);
	if (status)
		goto err;
	cpsw_lw_resetmsg_slave(lw_info);

err:
	if (status)
		dev_err(&lw_info->ndev->dev, "cpsw_lw_stop() failed to stop FW\n");

	status = notify_unregister_event(lw_info->host_proc_id,
				lw_info->host_line_id,
				lw_info->host_event_id,
				(notify_fn_notify_cbck)cpsw_lw_notification_callback,
				(void *)lw_info);
	if (status < 0)
		dev_err(&lw_info->ndev->dev, "cpsw_lw_stop() failed to unregister event callback\n");

	lw_info->status = CPSW_LW_DRV_READY;
	lw_info->mode = CPSW_ASI_MODE_NORMAL;
	return 0;
}

struct cpsw_lw_info* cpsw_lw_create(struct platform_device *pdev,
				struct net_device *ndev,
				struct cpdma_chan *tx_chan,
				struct cpdma_chan *rx_chan)
{
	struct cpsw_lw_info *lw_info;

	lw_info = (struct cpsw_lw_info *)kzalloc(sizeof(struct cpsw_lw_info), GFP_KERNEL);
	if (!lw_info) {
		dev_err(&ndev->dev, "failed to allocate struct cpsw_lw_info\n");
		goto err;
	}

	spin_lock_init(&lw_info->lock);
	init_waitqueue_head(&lw_info->wq);
	lw_info->ndev = ndev;
	lw_info->pdev = pdev;
	lw_info->tx_chan = tx_chan;
	lw_info->rx_chan = rx_chan;
	lw_info->tx_budget = INT_MAX;
	lw_info->mode = CPSW_ASI_MODE_NORMAL;
	lw_info->status = CPSW_LW_DRV_INIT;
	/* Defaults for usermode testing */
	lw_info->slave_proc_id = 3;
	lw_info->slave_line_id = 0;
	lw_info->slave_event_id = 7;
	lw_info->host_proc_id = 3;
	lw_info->host_line_id = 0;
	lw_info->host_event_id = 8;
	/* Allocate un-cached memory to communicate with the slave CPU */	
	lw_info->shmem = (struct cpsw_lw_shmem __force __iomem *)dma_alloc_coherent(&pdev->dev,
													sizeof(struct cpsw_lw_shmem), 
													&lw_info->shmem_hw_addr, GFP_KERNEL);
	if (!lw_info->shmem) {
		dev_err(&ndev->dev, "failed to allocate struct cpsw_lw_shmem\n");
		goto err;
	}

	lw_info->h2s_msg_bufaddr = dma_map_single(&pdev->dev, &lw_info->host_to_slave,
								sizeof(lw_info->host_to_slave), DMA_TO_DEVICE);

	if (dma_mapping_error(&ndev->dev, lw_info->h2s_msg_bufaddr)) {
		dev_err(&ndev->dev, "failed to map host2slave message buffer\n");
		goto err;
	}

	/* Regain ownership of the host to slave message buffer */
	dma_sync_single_for_cpu(&pdev->dev, lw_info->h2s_msg_bufaddr,
							sizeof(lw_info->host_to_slave), DMA_FROM_DEVICE);

	lw_info->s2h_msg_bufaddr = dma_map_single(&pdev->dev, &lw_info->slave_to_host,
								sizeof(lw_info->slave_to_host), DMA_FROM_DEVICE);

	if (dma_mapping_error(&ndev->dev, lw_info->s2h_msg_bufaddr)) {
		dev_err(&ndev->dev, "failed to map slave2host message buffer\n");
		goto free_h2s_mapping;
	}

	shmem_write(lw_info->shmem, slave_status, 0);
	shmem_write(lw_info->shmem, slave_errno, 0);
	shmem_write(lw_info->shmem, h2s_msg_status, CPSW_LW_MSM_FREE);
	shmem_write(lw_info->shmem, s2h_msg_status, 0);
	shmem_write(lw_info->shmem, h2s_msg_bufaddr, lw_info->h2s_msg_bufaddr);
	shmem_write(lw_info->shmem, s2h_msg_bufaddr, lw_info->s2h_msg_bufaddr);

	lw_info->msg_enable = CPSW_LW_DEBUG;
	lw_info->status = CPSW_LW_DRV_READY;
	return lw_info;

free_h2s_mapping:
	dma_unmap_single(&pdev->dev, lw_info->h2s_msg_bufaddr,
						sizeof(lw_info->host_to_slave), DMA_TO_DEVICE);

err:
	if (lw_info->shmem)
		dma_free_coherent(&pdev->dev, sizeof(struct cpsw_lw_shmem), 
							lw_info->shmem, lw_info->shmem_hw_addr);
	if (lw_info)
		kfree(lw_info);
	return NULL;
}

void cpsw_lw_destroy(struct net_device *ndev, struct cpsw_lw_info *lw_info)
{
	if (!lw_info)
		return;

	if (lw_info->h2s_msg_bufaddr)
		dma_unmap_single(&lw_info->pdev->dev, lw_info->h2s_msg_bufaddr,
						sizeof(lw_info->host_to_slave), DMA_TO_DEVICE);
	if (lw_info->s2h_msg_bufaddr)
		dma_unmap_single(&lw_info->pdev->dev, lw_info->s2h_msg_bufaddr,
						sizeof(lw_info->host_to_slave), DMA_TO_DEVICE);

	if (lw_info->shmem)
		dma_free_coherent(&lw_info->pdev->dev, sizeof(struct cpsw_lw_shmem),
						lw_info->shmem, lw_info->shmem_hw_addr);
	kfree(lw_info);
}
