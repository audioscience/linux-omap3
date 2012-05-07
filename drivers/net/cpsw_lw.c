
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/cpsw.h>

#include <syslink/notify.h>

#include "cpsw_lw.h"
#include "davinci_cpdma.h"

#define LW_NAPI_POLL_WEIGHT 64

#define shmem_read(desc, fld)		__raw_readl(&(desc)->fld)
#define shmem_write(desc, fld, v)	__raw_writel((u32)(v), &(desc)->fld)

struct cpsw_lw_shmem {
	u32 int_flags;

	s16 slave_state;
	s16 slave_errno;

	u32 h2s_msg_status;
	u32 h2s_msg_bufaddr;
	u32 s2h_msg_status;
	u32 s2h_msg_bufaddr;
};

struct cpsw_lw_info {
	spinlock_t lock;

	struct cpsw_lw_shmem __iomem *shmem;
	dma_addr_t shmem_hw_addr;

	wait_queue_head_t wq;

	struct net_device *ndev;
	struct napi_struct napi;

	struct cpdma_ctlr *cpdma_ctlr;
	struct cpdma_chan *tx_chan;
	struct cpdma_chan *outgoing_tx_chan;
	u32 tx_budget; /* in octets */

	s32 state;
	u32 req_mode;
	u32 cur_mode;

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

#define napi_to_lwinfo(napi) container_of(napi, struct cpsw_lw_info, napi)

static void cpsw_lw_tx_handler(void *token, int len, int status);
static void cpsw_lw_rx_handler(void *token, int len, int status);
static inline int cpsw_lw_prepmsg_slave(struct cpsw_lw_info *lw_info,
					enum cpsw_drv_cmd cmd, void *buf, size_t buf_size);
static inline int cpsw_lw_postmsg_slave(struct cpsw_lw_info *lw_info,
					bool wait_clear);
static inline int cpsw_lw_notify_slave(struct cpsw_lw_info *lw_info,
					bool wait_clear);

/* Callbacks */

static void cpsw_lw_notification_callback(u16 procid, u16 lineid, u32 eventno, 
			void *arg, u32 payload)
{
	struct cpsw_lw_info *lw_info = arg;

	BUG_ON(lw_info == NULL);
	
	// Start the next channel if the currently outgoing channel is done
	if (!cpdma_chan_desc_count(lw_info->outgoing_tx_chan))
		cpdma_chan_start(lw_info->tx_chan);

	// Schedule NAPI, will eventually call cpsw_lw_poll()
	napi_schedule(&lw_info->napi);

	if (lw_info->shmem->s2h_msg_status == CPSW_LW_MSM_POST ||
		lw_info->shmem->h2s_msg_status == CPSW_LW_MSM_DONE)
		wake_up(&lw_info->wq);
}

static void cpsw_lw_tx_handler(void *token, int len, int status)
{
	struct sk_buff		*skb = token;
	struct net_device	*ndev = skb->dev;
	struct cpsw_priv	*priv = netdev_priv(ndev);

	if (unlikely(netif_queue_stopped(ndev)))
		netif_start_queue(ndev);

	priv->stats.tx_packets++;
	priv->stats.tx_bytes += len;
	dev_kfree_skb_any(skb);
}

static void cpsw_lw_rx_handler(void *token, int len, int status)
{
	struct sk_buff		*skb = token;
	struct net_device	*ndev = skb->dev;
	struct cpsw_priv	*priv = netdev_priv(ndev);
	int			ret = 0;

	/* free and bail if we are shutting down */
	if (unlikely(!netif_running(ndev)) ||
			unlikely(!netif_carrier_ok(ndev))) {
		dev_kfree_skb_any(skb);
		return;
	}

	if (likely(status >= 0)) {
		skb_put(skb, len);
		skb->protocol = eth_type_trans(skb, ndev);
		netif_receive_skb(skb);
		priv->stats.rx_bytes += len;
		priv->stats.rx_packets++;
		skb = NULL;
	}


	if (unlikely(!netif_running(ndev))) {
		if (skb)
			dev_kfree_skb_any(skb);
		return;
	}

	if (likely(!skb)) {
		skb = netdev_alloc_skb_ip_align(ndev, priv->rx_packet_max);
		if (WARN_ON(!skb))
			return;

		ret = cpdma_chan_submit(priv->rxch, skb, skb->data,
				skb_tailroom(skb), 0, GFP_KERNEL);
	}

	WARN_ON(ret < 0);

}

static int cpsw_lw_poll(struct napi_struct *napi, int budget)
{
	struct cpsw_lw_info *lw_info = napi_to_lwinfo(napi);
	int num_tx, num_rx;

	struct cpdma_chan *next_tx_chan;
	unsigned long flags;

#if 0
	if (cpdma_chan_desc_count(lw_info->outgoing_tx_chan))
		; /* Log an error, the outgoing channel should be done by now */

	/* For speed, instead of creating and destroying a channel we should add a cpdma_chan_reset() function */
	next_tx_chan = cpdma_chan_create_budget(lw_info->cpdma_ctlr, tx_chan_num(0), cpsw_lw_tx_handler, lw_info->tx_budget);
	if (cpdma_chan_desc_count(lw_info->outgoing_tx_chan))
		; /* Log an error, the outgoing channel should be done by now */
	cpdma_chan_destroy(lw_info->outgoing_tx_chan);

	spin_lock_irqsave(&lw_info->lock, flags);
	lw_info->outgoing_tx_chan = lw_info->tx_chan;
	lw_info->tx_chan = next_tx_chan;
	spin_unlock_irqrestore(&lw_info->lock, flags);

	num_tx = cpdma_chan_process(priv->txch, 128);
	num_rx = cpdma_chan_process(priv->rxch, budget);

	if (num_rx || num_tx)
		msg(dbg, intr, "poll %d rx, %d tx pkts\n", num_rx, num_tx);
#endif
	if (num_rx < budget) {
		napi_complete(napi);
	}

	return num_rx;
}

static inline int cpsw_lw_prepmsg_slave(struct cpsw_lw_info *lw_info,
					enum cpsw_drv_cmd cmd, void *buf, size_t buf_size)
{
	if (!lw_info || !buf || buf_size > sizeof(union req_data))
		return -EINVAL;

	spin_lock(&lw_info->shmem_lock);
	/* If the message is being processed bail out */
	if (shmem_read(lw_info->shmem, h2s_msg_status) != CPSW_LW_MSM_FREE) {
		spin_unlock(&lw_info->shmem_lock);
		return -EINPROGRESS;
	}
	shmem_write(lw_info->shmem, h2s_msg_status, CPSW_LW_MSM_PREP);
	spin_unlock(&lw_info->shmem_lock);

	memset(&lw_info->host_to_slave.req, 0, sizeof(struct cpsw_lw_msg));
	lw_info->host_to_slave.cmd = cmd;
	memcpy(&lw_info->host_to_slave.req, buf, buf_size);
	return 0;
}

static inline int cpsw_lw_postmsg_slave(struct cpsw_lw_info *lw_info, bool wait_clear)
{
	if (!lw_info)
		return -EINVAL;

	spin_lock(&lw_info->shmem_lock);	
	if (shmem_read(lw_info->shmem, h2s_msg_status) != CPSW_LW_MSM_PREP) {
		spin_unlock(&lw_info->shmem_lock);
		return -EINPROGRESS;
	}
	shmem_write(lw_info->shmem, h2s_msg_status, CPSW_LW_MSM_POST);
	spin_unlock(&lw_info->shmem_lock);	

	dma_sync_single_for_device(&lw_info->ndev->dev, lw_info->h2s_msg_bufaddr,
							sizeof(lw_info->host_to_slave), DMA_TO_DEVICE);

	return cpsw_lw_notify_slave(lw_info, true);
}

static inline int cpsw_lw_recvmsg_slave(struct cpsw_lw_info *lw_info,
									struct cpsw_lw_msg *dest_buf)
{
	if (!lw_info)
		return -EINVAL;

	spin_lock(&lw_info->shmem_lock);	
	if (shmem_read(lw_info->shmem, h2s_msg_status) != CPSW_LW_MSM_DONE) {
		spin_unlock(&lw_info->shmem_lock);
		return -EINPROGRESS;
	}
	dma_sync_single_for_cpu(&lw_info->ndev->dev, lw_info->h2s_msg_bufaddr,
							sizeof(lw_info->host_to_slave), DMA_FROM_DEVICE);
	if (dest_buf) {
		memcpy(msg, &lw_info->host_to_slave, sizeof(*msg));
		shmem_write(lw_info->shmem, h2s_msg_status, CPSW_LW_MSM_FREE);
	}
	spin_unlock(&lw_info->shmem_lock);

	return 0;
}

static inline int cpsw_lw_freemsg_slave(struct cpsw_lw_info *lw_info)
{
	if (!lw_info)
		return -EINVAL;

	spin_lock(&lw_info->shmem_lock);	
	if (shmem_read(lw_info->shmem, h2s_msg_status) != CPSW_LW_MSM_DONE) {
		spin_unlock(&lw_info->shmem_lock);
		return -EINPROGRESS;
	}
	shmem_write(lw_info->shmem, h2s_msg_status, CPSW_LW_MSM_FREE);
	spin_unlock(&lw_info->shmem_lock);

	return 0;
}

static inline int cpsw_lw_notify_slave(struct cpsw_lw_info *lw_info, bool wait_clear)
{
	int res;
	
	if (!lw_info)
		return -EINVAL;
	
	spin_lock(&lw_info->lock);
	res = notify_send_event(lw_info->slave_proc_id,
				lw_info->slave_line_id,
				lw_info->slave_event_id,
				lw_info->shmem_hw_addr,
				wait_clear);
	spin_unlock(&lw_info->lock);
	return res;
}

/*  */

int cpsw_lw_status(struct cpsw_lw_info *lw_info)
{
	int tmp;

	if (!lw_info)
		return -EINVAL;

	spin_lock(&lw_info->lock);
	tmp = lw_info->cur_mode;
	spin_unlock(&lw_info->lock);
	return tmp;
}

/* Lifecycle related functions */

int cpsw_lw_start(struct cpsw_lw_info *lw_info)
{
	int status;

	spin_lock(&lw_info->lock);
	
	napi_enable(&lw_info->napi);

	if (lw_info->state != CPSW_LW_DRV_READY) {
		return -EBUSY;
	}

	/* Setup to receive notifications */
	status = notify_register_event(lw_info->host_proc_id,
				lw_info->host_line_id,
				lw_info->host_event_id,
				(notify_fn_notify_cbck)cpsw_lw_notification_callback,
				(void *)lw_info);
	if (status < 0)
		return status;

	cpsw_lw_prepmsg_slave(lw_info, CPSW_LW_FWMSG_GETVER, NULL, 0);
	status = cpsw_lw_postmsg_slave(lw_info, true);
#if 0
	cpsw_lw_waitres_wq_slave(lw_info, CPSW_LW_MSG_POLL_PERIOD, CPSW_LW_MSG_TIMEOUT);
	cpsw_lw_pollres_slave(lw_info, CPSW_LW_MSG_POLL_PERIOD, CPSW_LW_MSG_TIMEOUT);

	int retries = 4;
	do {
		long res = wait_event_interruptible_timeout(lw_info->wq,
					shmem_read(lw_info->shmem, h2s_msg_status) == CPSW_LW_MSM_DONE,
					usecs_to_jiffies(50));
		if (res) {
			status = res;
			break;
		}
	} while(retries--);

	if ()
		return -EINTR;
	if ()
		return -ETIMEDOUT; /* or -ETIME ? */
#endif
	spin_unlock(&lw_info->lock);

	return 0;
}

int cpsw_lw_stop(struct cpsw_lw_info *lw_info)
{
	int status;

	spin_lock(&lw_info->lock);
	status = notify_unregister_event(lw_info->host_proc_id,
				lw_info->host_line_id,
				lw_info->host_event_id,
				(notify_fn_notify_cbck)cpsw_lw_notification_callback,
				(void *)lw_info);
	if (status < 0)
		goto err;

	napi_disable(&lw_info->napi);

err:
	spin_unlock(&lw_info->lock);
	return 0;
}

struct cpsw_lw_info* cpsw_lw_create(struct net_device *ndev)
{
	struct cpsw_lw_info *lw_info;

	lw_info = (struct cpsw_lw_info *)kzalloc(sizeof(struct cpsw_lw_info), GFP_KERNEL);
	if (!lw_info) {
		dev_err(&ndev->dev, "failed to allocate struct cpsw_lw_info\n");
		goto err;
	}

	spin_lock_init(&lw_info->lock);
	spin_lock_init(&lw_info->shmem_lock);
	init_waitqueue_head(&lw_info->wq);

	lw_info->ndev = ndev;

	/* Allocate un-cached memory to communicate with the slave CPU */	
	lw_info->shmem = (struct cpsw_lw_shmem __force __iomem *)dma_alloc_coherent(&ndev->dev,
													sizeof(struct cpsw_lw_shmem), 
													&lw_info->shmem_hw_addr, GFP_KERNEL);
	if (!lw_info->shmem) {
		dev_err(&ndev->dev, "failed to allocate struct cpsw_lw_shmem\n");
		goto err;
	}

	lw_info->h2s_msg_bufaddr = dma_map_single(&ndev->dev, &lw_info->host_to_slave,
								sizeof(lw_info->host_to_slave), DMA_TO_DEVICE);

	if (dma_mapping_error(&ndev->dev, lw_info->h2s_msg_bufaddr)) {
		dev_err(&ndev->dev, "failed to map host2slave message buffer\n");
		goto err;
	}

	lw_info->s2h_msg_bufaddr = dma_map_single(&ndev->dev, &lw_info->slave_to_host,
								sizeof(lw_info->slave_to_host), DMA_FROM_DEVICE);

	if (dma_mapping_error(&ndev->dev, lw_info->s2h_msg_bufaddr)) {
		dev_err(&ndev->dev, "failed to map slave2host message buffer\n");
		goto free_h2s_mapping;
	}

	shmem_write(lw_info->shmem, slave_state, 0);
	shmem_write(lw_info->shmem, slave_errno, 0);
	shmem_write(lw_info->shmem, h2s_msg_status, 0);
	shmem_write(lw_info->shmem, s2h_msg_status, 0);
	shmem_write(lw_info->shmem, h2s_msg_bufaddr, lw_info->h2s_msg_bufaddr);
	shmem_write(lw_info->shmem, s2h_msg_bufaddr, lw_info->s2h_msg_bufaddr);

	netif_napi_add(ndev, &lw_info->napi, cpsw_lw_poll, LW_NAPI_POLL_WEIGHT);

	return lw_info;

free_h2s_mapping:
	dma_unmap_single(&ndev->dev, lw_info->h2s_msg_bufaddr,
						sizeof(lw_info->host_to_slave), DMA_TO_DEVICE);

err:
	if (lw_info->shmem)
		dma_free_coherent(&ndev->dev, sizeof(struct cpsw_lw_shmem), 
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
		dma_unmap_single(&ndev->dev, lw_info->h2s_msg_bufaddr,
						sizeof(lw_info->host_to_slave), DMA_TO_DEVICE);
	if (lw_info->s2h_msg_bufaddr)
		dma_unmap_single(&ndev->dev, lw_info->s2h_msg_bufaddr,
						sizeof(lw_info->host_to_slave), DMA_TO_DEVICE);

	if (lw_info->shmem)
		dma_free_coherent(&ndev->dev, sizeof(struct cpsw_lw_shmem),
						lw_info->shmem, lw_info->shmem_hw_addr);
	kfree(lw_info);
}
