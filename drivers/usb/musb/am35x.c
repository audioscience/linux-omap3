/*
 * Texas Instruments AM35x "glue layer"
 *
 * Copyright (c) 2010, by Texas Instruments
 *
 * Based on the DA8xx "glue layer" code.
 * Copyright (c) 2008-2009, MontaVista Software, Inc. <source@mvista.com>
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/init.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <plat/usb.h>

#include "musb_core.h"
#include "cppi41.h"
#include "cppi41_dma.h"

/*
 * AM35x specific definitions
 */
/* USB 2.0 OTG module registers */
#define USB_REVISION_REG	0x00
#define USB_CTRL_REG		0x04
#define USB_STAT_REG		0x08
#define USB_EMULATION_REG	0x0c
/* 0x10 Reserved */
#define USB_SRP_FIX_TIME_REG	0x18
#define EP_INTR_SRC_REG		0x20
#define EP_INTR_SRC_SET_REG	0x24
#define EP_INTR_SRC_CLEAR_REG	0x28
#define EP_INTR_MASK_REG	0x2c
#define EP_INTR_MASK_SET_REG	0x30
#define EP_INTR_MASK_CLEAR_REG	0x34
#define EP_INTR_SRC_MASKED_REG	0x38
#define CORE_INTR_SRC_REG	0x40
#define CORE_INTR_SRC_SET_REG	0x44
#define CORE_INTR_SRC_CLEAR_REG	0x48
#define CORE_INTR_MASK_REG	0x4c
#define CORE_INTR_MASK_SET_REG	0x50
#define CORE_INTR_MASK_CLEAR_REG 0x54
#define CORE_INTR_SRC_MASKED_REG 0x58
/* 0x5c Reserved */
#define USB_END_OF_INTR_REG	0x60

/* Control register bits */
#define AM35X_SOFT_RESET_MASK	1

/* USB interrupt register bits */
#define AM35X_INTR_USB_SHIFT	16
#define AM35X_INTR_USB_MASK	(0x1ff << AM35X_INTR_USB_SHIFT)
#define AM35X_INTR_DRVVBUS	0x100
#define AM35X_INTR_RX_SHIFT	16
#define AM35X_INTR_TX_SHIFT	0
#define AM35X_TX_EP_MASK	0xffff		/* EP0 + 15 Tx EPs */
#define AM35X_RX_EP_MASK	0xfffe		/* 15 Rx EPs */
#define AM35X_TX_INTR_MASK	(AM35X_TX_EP_MASK << AM35X_INTR_TX_SHIFT)
#define AM35X_RX_INTR_MASK	(AM35X_RX_EP_MASK << AM35X_INTR_RX_SHIFT)

/* CPPI 4.1 queue manager registers */
#define QMGR_PEND0_REG		0x4090
#define QMGR_PEND1_REG		0x4094
#define QMGR_PEND2_REG		0x4098

#define USB_MENTOR_CORE_OFFSET	0x400

#ifdef CONFIG_USB_TI_CPPI41_DMA
/*
 * CPPI 4.1 resources used for USB OTG controller module:
 *
 * USB   DMA  DMA  QMgr  Tx     Src
 *       Tx   Rx         QNum   Port
 * ---------------------------------
 * EP0   0    0    0     16,17  1
 * ---------------------------------
 * EP1   1    1    0     18,19  2
 * ---------------------------------
 * EP2   2    2    0     20,21  3
 * ---------------------------------
 * EP3   3    3    0     22,23  4
 * ---------------------------------
 */

static const u16 tx_comp_q[] = { 63, 64 };
static const u16 rx_comp_q[] = { 65, 66 };

const struct usb_cppi41_info usb_cppi41_info = {
	.dma_block	= 0,
	.ep_dma_ch	= { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 },
	.q_mgr		= 0,
	.num_tx_comp_q	= 2,
	.num_rx_comp_q	= 2,
	.tx_comp_q	= tx_comp_q,
	.rx_comp_q	= rx_comp_q
};

/* Fair scheduling */
u32 dma_sched_table[] = {
	0x81018000, 0x83038202, 0x85058404, 0x87078606,
	0x89098808, 0x8b0b8a0a, 0x8d0d8c0c, 0x00008e0e
};

/* DMA block configuration */
static const struct cppi41_tx_ch tx_ch_info[] = {
	[0] = {
		.port_num	= 1,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 32} , {0, 33} }
	},
	[1] = {
		.port_num	= 2,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 34} , {0, 35} }
	},
	[2] = {
		.port_num	= 3,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 36} , {0, 37} }
	},
	[3] = {
		.port_num	= 4,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 38} , {0, 39} }
	},
	[4] = {
		.port_num	= 5,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 40} , {0, 41} }
	},
	[5] = {
		.port_num	= 6,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 42} , {0, 43} }
	},
	[6] = {
		.port_num	= 7,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 44} , {0, 45} }
	},
	[7] = {
		.port_num	= 8,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 46} , {0, 47} }
	},
	[8] = {
		.port_num	= 9,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 48} , {0, 49} }
	},
	[9] = {
		.port_num	= 10,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 50} , {0, 51} }
	},
	[10] = {
		.port_num	= 11,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 52} , {0, 53} }
	},
	[11] = {
		.port_num	= 12,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 54} , {0, 55} }
	},
	[12] = {
		.port_num	= 13,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 56} , {0, 57} }
	},
	[13] = {
		.port_num	= 14,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 58} , {0, 59} }
	},
	[14] = {
		.port_num	= 15,
		.num_tx_queue	= 2,
		.tx_queue	= { {0, 60} , {0, 61} }
	}
};

struct cppi41_dma_block cppi41_dma_block[CPPI41_NUM_DMA_BLOCK] = {
	[0] = {
		.global_ctrl_base =
			IO_ADDRESS(OMAP34XX_HSUSB_OTG_BASE) + 0x1000,
		.ch_ctrl_stat_base =
			IO_ADDRESS(OMAP34XX_HSUSB_OTG_BASE) + 0x1800,
		.sched_ctrl_base =
			IO_ADDRESS(OMAP34XX_HSUSB_OTG_BASE) + 0x2000,
		.sched_table_base =
			IO_ADDRESS(OMAP34XX_HSUSB_OTG_BASE) + 0x2800,
		.num_tx_ch	= 15,
		.num_rx_ch	= 15,
		.tx_ch_info	= tx_ch_info
	}
};
EXPORT_SYMBOL(cppi41_dma_block);

/* Queues 0 to 66 are pre-assigned, others are spare */
static const u32 assigned_queues[] = { 0xffffffff, 0xffffffff, 0x7 };

/* Queue manager information */
struct cppi41_queue_mgr cppi41_queue_mgr[CPPI41_NUM_QUEUE_MGR] = {
	[0] = {
		.q_mgr_rgn_base =
			IO_ADDRESS(OMAP34XX_HSUSB_OTG_BASE) + 0x4000,
		.desc_mem_rgn_base =
			IO_ADDRESS(OMAP34XX_HSUSB_OTG_BASE) + 0x5000,
		.q_mgmt_rgn_base =
			IO_ADDRESS(OMAP34XX_HSUSB_OTG_BASE) + 0x6000,
		.q_stat_rgn_base =
			IO_ADDRESS(OMAP34XX_HSUSB_OTG_BASE) + 0x6800,

		.num_queue	= 96,
		.queue_types	= CPPI41_FREE_DESC_BUF_QUEUE |
					CPPI41_UNASSIGNED_QUEUE,
		.base_fdbq_num	= 0,
		.assigned	= assigned_queues
	}
};
EXPORT_SYMBOL(cppi41_queue_mgr);

int __init cppi41_init(struct musb *musb)
{
	u16 numch, blknum = usb_cppi41_info.dma_block, order;

	/* Initialize for Linking RAM region 0 alone */
	cppi41_queue_mgr_init(usb_cppi41_info.q_mgr, 0, 0x3fff);

	numch =  USB_CPPI41_NUM_CH * 2;
	order = get_count_order(numch);

	/* TODO: check two teardown desc per channel (5 or 7 ?)*/
	if (order < 5)
		order = 5;

	cppi41_dma_block_init(blknum, usb_cppi41_info.q_mgr, order,
			dma_sched_table, numch);
	return 0;
}
#endif /* CONFIG_USB_TI_CPPI41_DMA */

struct am35x_musb_glue {
	struct clk	*clk;
	struct clk	*fck;
	struct device	*dev;
};

static inline void phy_on(void)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(100);
	u32 devconf2;

	/*
	 * Start the on-chip PHY and its PLL.
	 */
	devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

	devconf2 &= ~(CONF2_RESET | CONF2_PHYPWRDN | CONF2_OTGPWRDN);
	devconf2 |= CONF2_PHY_PLLON;

	omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);

	DBG(1, "Waiting for PHY clock good...\n");
	while (!(omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2)
			& CONF2_PHYCLKGD)) {
		cpu_relax();

		if (time_after(jiffies, timeout)) {
			DBG(1, "musb PHY clock good timed out\n");
			break;
		}
	}
}

static inline void phy_off(void)
{
	u32 devconf2;

	/*
	 * Power down the on-chip PHY.
	 */
	devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

	devconf2 &= ~CONF2_PHY_PLLON;
	devconf2 |=  CONF2_PHYPWRDN | CONF2_OTGPWRDN;
	omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);
}

/*
 * am35x_musb_enable - enable interrupts
 */
static void am35x_musb_enable(struct musb *musb)
{
	void __iomem *reg_base = musb->ctrl_base;
	u32 epmask;

	/* Workaround: setup IRQs through both register sets. */
	epmask = ((musb->epmask & AM35X_TX_EP_MASK) << AM35X_INTR_TX_SHIFT) |
	       ((musb->epmask & AM35X_RX_EP_MASK) << AM35X_INTR_RX_SHIFT);

	musb_writel(reg_base, EP_INTR_MASK_SET_REG, epmask);
	musb_writel(reg_base, CORE_INTR_MASK_SET_REG, AM35X_INTR_USB_MASK);

	/* Force the DRVVBUS IRQ so we can start polling for ID change. */
	if (is_otg_enabled(musb))
		musb_writel(reg_base, CORE_INTR_SRC_SET_REG,
			    AM35X_INTR_DRVVBUS << AM35X_INTR_USB_SHIFT);
}

/*
 * am35x_musb_disable - disable HDRC and flush interrupts
 */
static void am35x_musb_disable(struct musb *musb)
{
	void __iomem *reg_base = musb->ctrl_base;

	musb_writel(reg_base, CORE_INTR_MASK_CLEAR_REG, AM35X_INTR_USB_MASK);
	musb_writel(reg_base, EP_INTR_MASK_CLEAR_REG,
			 AM35X_TX_INTR_MASK | AM35X_RX_INTR_MASK);
	musb_writeb(musb->mregs, MUSB_DEVCTL, 0);
	musb_writel(reg_base, USB_END_OF_INTR_REG, 0);
}

#ifdef CONFIG_USB_MUSB_HDRC_HCD
#define portstate(stmt)		stmt
#else
#define portstate(stmt)
#endif

static void am35x_set_vbus(struct musb *musb, int is_on)
{
	WARN_ON(is_on && is_peripheral_active(musb));
}

#define	POLL_SECONDS	2

static struct timer_list otg_workaround;
static u64 musb_dmamask = DMA_BIT_MASK(32);

static void otg_timer(unsigned long _musb)
{
	struct musb		*musb = (void *)_musb;
	void __iomem		*mregs = musb->mregs;
	u8			devctl;
	unsigned long		flags;

	/*
	 * We poll because AM35x's won't expose several OTG-critical
	 * status change events (from the transceiver) otherwise.
	 */
	devctl = musb_readb(mregs, MUSB_DEVCTL);
	DBG(7, "Poll devctl %02x (%s)\n", devctl, otg_state_string(musb));

	spin_lock_irqsave(&musb->lock, flags);
	switch (musb->xceiv->state) {
	case OTG_STATE_A_WAIT_BCON:
		devctl &= ~MUSB_DEVCTL_SESSION;
		musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

		devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		if (devctl & MUSB_DEVCTL_BDEVICE) {
			musb->xceiv->state = OTG_STATE_B_IDLE;
			MUSB_DEV_MODE(musb);
		} else {
			musb->xceiv->state = OTG_STATE_A_IDLE;
			MUSB_HST_MODE(musb);
		}
		break;
	case OTG_STATE_A_WAIT_VFALL:
		musb->xceiv->state = OTG_STATE_A_WAIT_VRISE;
		musb_writel(musb->ctrl_base, CORE_INTR_SRC_SET_REG,
			    MUSB_INTR_VBUSERROR << AM35X_INTR_USB_SHIFT);
		break;
	case OTG_STATE_B_IDLE:
		if (!is_peripheral_enabled(musb))
			break;

		devctl = musb_readb(mregs, MUSB_DEVCTL);
		if (devctl & MUSB_DEVCTL_BDEVICE)
			mod_timer(&otg_workaround, jiffies + POLL_SECONDS * HZ);
		else
			musb->xceiv->state = OTG_STATE_A_IDLE;
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&musb->lock, flags);
}

static void am35x_musb_try_idle(struct musb *musb, unsigned long timeout)
{
	static unsigned long last_timer;

	if (!is_otg_enabled(musb))
		return;

	if (timeout == 0)
		timeout = jiffies + msecs_to_jiffies(3);

	/* Never idle if active, or when VBUS timeout is not set as host */
	if (musb->is_active || (musb->a_wait_bcon == 0 &&
				musb->xceiv->state == OTG_STATE_A_WAIT_BCON)) {
		DBG(4, "%s active, deleting timer\n", otg_state_string(musb));
		del_timer(&otg_workaround);
		last_timer = jiffies;
		return;
	}

	if (time_after(last_timer, timeout) && timer_pending(&otg_workaround)) {
		DBG(4, "Longer idle timer already pending, ignoring...\n");
		return;
	}
	last_timer = timeout;

	DBG(4, "%s inactive, starting idle timer for %u ms\n",
	    otg_state_string(musb), jiffies_to_msecs(timeout - jiffies));
	mod_timer(&otg_workaround, timeout);
}

static irqreturn_t am35x_interrupt(int irq, void *hci)
{
	struct musb  *musb = hci;
	void __iomem *reg_base = musb->ctrl_base;
	unsigned long flags;
	irqreturn_t ret = IRQ_NONE;
	u32 pend1 = 0, pend2 = 0, tx, rx;
	u32 epintr, usbintr, lvl_intr;

	spin_lock_irqsave(&musb->lock, flags);

	/*
	 * CPPI 4.1 interrupts share the same IRQ and the EOI register but
	 * don't get reflected in the interrupt source/mask registers.
	 */
	if (is_cppi41_enabled()) {
		/*
		 * Check for the interrupts from Tx/Rx completion queues; they
		 * are level-triggered and will stay asserted until the queues
		 * are emptied.  We're using the queue pending register 0 as a
		 * substitute for the interrupt status register and reading it
		 * directly for speed.
		 */
		pend1 = musb_readl(reg_base, QMGR_PEND1_REG);
		pend2 = musb_readl(reg_base, QMGR_PEND2_REG);

		/* AM3517 uses 63,64,65 and 66 queues as completion queue */
		if ((pend1 & (1 << 31)) || (pend2 & (7 << 0))) {
			tx = (pend1 >> 31)  | ((pend2 & 1) ? (1 << 1) : 0);
			rx = (pend2 >> 1) & 0x3;

			DBG(4, "CPPI 4.1 IRQ: Tx %x, Rx %x\n", tx, rx);
			cppi41_completion(musb, rx, tx);
			ret = IRQ_HANDLED;
		}
	}
	/* Get endpoint interrupts */
	epintr = musb_readl(reg_base, EP_INTR_SRC_MASKED_REG);

	if (epintr) {
		musb_writel(reg_base, EP_INTR_SRC_CLEAR_REG, epintr);

		musb->int_rx =
			(epintr & AM35X_RX_INTR_MASK) >> AM35X_INTR_RX_SHIFT;
		musb->int_tx =
			(epintr & AM35X_TX_INTR_MASK) >> AM35X_INTR_TX_SHIFT;
	}

	/* Get usb core interrupts */
	usbintr = musb_readl(reg_base, CORE_INTR_SRC_MASKED_REG);
	if (!usbintr && !epintr)
		goto eoi;

	if (usbintr) {
		musb_writel(reg_base, CORE_INTR_SRC_CLEAR_REG, usbintr);

		musb->int_usb =
			(usbintr & AM35X_INTR_USB_MASK) >> AM35X_INTR_USB_SHIFT;
	}
	/*
	 * DRVVBUS IRQs are the only proxy we have (a very poor one!) for
	 * AM35x's missing ID change IRQ.  We need an ID change IRQ to
	 * switch appropriately between halves of the OTG state machine.
	 * Managing DEVCTL.SESSION per Mentor docs requires that we know its
	 * value but DEVCTL.BDEVICE is invalid without DEVCTL.SESSION set.
	 * Also, DRVVBUS pulses for SRP (but not at 5V) ...
	 */
	if (usbintr & (AM35X_INTR_DRVVBUS << AM35X_INTR_USB_SHIFT)) {
		int drvvbus = musb_readl(reg_base, USB_STAT_REG);
		void __iomem *mregs = musb->mregs;
		u8 devctl = musb_readb(mregs, MUSB_DEVCTL);
		int err;

		err = is_host_enabled(musb) && (musb->int_usb &
						MUSB_INTR_VBUSERROR);
		if (err) {
			/*
			 * The Mentor core doesn't debounce VBUS as needed
			 * to cope with device connect current spikes. This
			 * means it's not uncommon for bus-powered devices
			 * to get VBUS errors during enumeration.
			 *
			 * This is a workaround, but newer RTL from Mentor
			 * seems to allow a better one: "re"-starting sessions
			 * without waiting for VBUS to stop registering in
			 * devctl.
			 */
			musb->int_usb &= ~MUSB_INTR_VBUSERROR;
			musb->xceiv->state = OTG_STATE_A_WAIT_VFALL;
			mod_timer(&otg_workaround, jiffies + POLL_SECONDS * HZ);
			WARNING("VBUS error workaround (delay coming)\n");
		} else if (is_host_enabled(musb) && drvvbus) {
			MUSB_HST_MODE(musb);
			musb->xceiv->default_a = 1;
			musb->xceiv->state = OTG_STATE_A_WAIT_VRISE;
			portstate(musb->port1_status |= USB_PORT_STAT_POWER);
			del_timer(&otg_workaround);
		} else {
			musb->is_active = 0;
			MUSB_DEV_MODE(musb);
			musb->xceiv->default_a = 0;
			musb->xceiv->state = OTG_STATE_B_IDLE;
			portstate(musb->port1_status &= ~USB_PORT_STAT_POWER);
		}

		/* NOTE: this must complete power-on within 100 ms. */
		DBG(2, "VBUS %s (%s)%s, devctl %02x\n",
				drvvbus ? "on" : "off",
				otg_state_string(musb),
				err ? " ERROR" : "",
				devctl);
		ret = IRQ_HANDLED;
	}

	if (musb->int_tx || musb->int_rx || musb->int_usb)
		ret |= musb_interrupt(musb);

eoi:
	/* EOI needs to be written for the IRQ to be re-asserted. */
	if (ret == IRQ_HANDLED || epintr || usbintr) {
		/* clear level interrupt */
		lvl_intr = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
		lvl_intr |= AM35XX_USBOTGSS_INT_CLR;
		omap_ctrl_writel(lvl_intr, AM35XX_CONTROL_LVL_INTR_CLEAR);
		/* write EOI */
		musb_writel(reg_base, USB_END_OF_INTR_REG, 0);
	}

	/* Poll for ID change */
	if (is_otg_enabled(musb) && musb->xceiv->state == OTG_STATE_B_IDLE)
		mod_timer(&otg_workaround, jiffies + POLL_SECONDS * HZ);

	spin_unlock_irqrestore(&musb->lock, flags);

	return ret;
}

static int am35x_musb_set_mode(struct musb *musb, u8 musb_mode)
{
	u32 devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

	devconf2 &= ~CONF2_OTGMODE;
	switch (musb_mode) {
#ifdef	CONFIG_USB_MUSB_HDRC_HCD
	case MUSB_HOST:		/* Force VBUS valid, ID = 0 */
		devconf2 |= CONF2_FORCE_HOST;
		break;
#endif
#ifdef	CONFIG_USB_GADGET_MUSB_HDRC
	case MUSB_PERIPHERAL:	/* Force VBUS valid, ID = 1 */
		devconf2 |= CONF2_FORCE_DEVICE;
		break;
#endif
#ifdef	CONFIG_USB_MUSB_OTG
	case MUSB_OTG:		/* Don't override the VBUS/ID comparators */
		devconf2 |= CONF2_NO_OVERRIDE;
		break;
#endif
	default:
		DBG(2, "Trying to set unsupported mode %u\n", musb_mode);
	}

	omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);
	return 0;
}

static int am35x_musb_init(struct musb *musb, void *board_data)
{
	void __iomem *reg_base = musb->ctrl_base;
	u32 rev, lvl_intr, sw_reset;
	int status;

	musb->mregs += USB_MENTOR_CORE_OFFSET;

	/* Returns zero if e.g. not clocked */
	rev = musb_readl(reg_base, USB_REVISION_REG);
	if (!rev) {
		status = -ENODEV;
		goto exit1;
	}

	usb_nop_xceiv_register();
	musb->xceiv = otg_get_transceiver();
	if (!musb->xceiv) {
		status = -ENODEV;
		goto exit1;
	}

	if (is_host_enabled(musb))
		setup_timer(&otg_workaround, otg_timer, (unsigned long) musb);

	musb->board_set_vbus = am35x_set_vbus;

	/* Global reset */
	sw_reset = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);

	sw_reset |= AM35XX_USBOTGSS_SW_RST;
	omap_ctrl_writel(sw_reset, AM35XX_CONTROL_IP_SW_RESET);

	sw_reset &= ~AM35XX_USBOTGSS_SW_RST;
	omap_ctrl_writel(sw_reset, AM35XX_CONTROL_IP_SW_RESET);

	/* Reset the controller */
	musb_writel(reg_base, USB_CTRL_REG, AM35X_SOFT_RESET_MASK);

	/* Start the on-chip PHY and its PLL. */
	phy_on();

	msleep(5);

#ifdef CONFIG_USB_TI_CPPI41_DMA
	cppi41_init();
#endif

	musb->isr = am35x_interrupt;

	/* clear level interrupt */
	lvl_intr = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	lvl_intr |= AM35XX_USBOTGSS_INT_CLR;
	omap_ctrl_writel(lvl_intr, AM35XX_CONTROL_LVL_INTR_CLEAR);
	return 0;
exit1:
	return status;
}

static int am35x_musb_exit(struct musb *musb)
{
	if (is_host_enabled(musb))
		del_timer_sync(&otg_workaround);

	phy_off();

	otg_put_transceiver(musb->xceiv);
	usb_nop_xceiv_unregister();

#ifdef CONFIG_USB_TI_CPPI41_DMA
	cppi41_exit();
#endif

	clk_disable(musb->clock);

	clk_disable(musb->phy_clock);
	clk_put(musb->phy_clock);

	return 0;
}

#ifdef CONFIG_PM
void musb_platform_save_context(struct musb *musb,
	struct musb_context_registers *musb_context)
{
	phy_off();
}

void musb_platform_restore_context(struct musb *musb,
	struct musb_context_registers *musb_context)
{
	phy_on();
}
#endif

/* AM35x supports only 32bit read operation */
void musb_read_fifo(struct musb_hw_ep *hw_ep, u16 len, u8 *dst)
{
	void __iomem *fifo = hw_ep->fifo;
	u32		val;
	int		i;

	/* Read for 32bit-aligned destination address */
	if (likely((0x03 & (unsigned long) dst) == 0) && len >= 4) {
		readsl(fifo, dst, len >> 2);
		dst += len & ~0x03;
		len &= 0x03;
	}
	/*
	 * Now read the remaining 1 to 3 byte or complete length if
	 * unaligned address.
	 */
	if (len > 4) {
		for (i = 0; i < (len >> 2); i++) {
			*(u32 *) dst = musb_readl(fifo, 0);
			dst += 4;
		}
		len &= 0x03;
	}
	if (len > 0) {
		val = musb_readl(fifo, 0);
		memcpy(dst, &val, len);
	}
}

static struct musb_platform_ops am35x_musb_ops = {
	.init		= am35x_musb_init,
	.exit		= am35x_musb_exit,
	.enable		= am35x_musb_enable,
	.disable	= am35x_musb_disable,
	.try_idle	= am35x_musb_try_idle,
	.set_mode	= am35x_musb_set_mode,
};

static int __init am35x_musb_probe(struct platform_device *pdev)
{
	struct am35x_musb_glue		*am35x;
	struct platform_device		*musb;
	struct musb_hdrc_platform_data	*pdata = pdev->dev.platform_data;
	int				ret;

	am35x = kzalloc(sizeof(*am35x), GFP_KERNEL);
	if (!am35x) {
		dev_err(&pdev->dev, "unable to allocate memory\n");
		return -ENOMEM;
	}

	am35x->clk = clk_get(&pdev->dev, "ick");
	if (IS_ERR(am35x->clk)) {
		dev_err(&pdev->dev, "unable to get clock\n");
		ret = PTR_ERR(am35x->clk);
		goto err0;
	}

	am35x->fck = clk_get(&pdev->dev, "fck");
	if (IS_ERR(am35x->fck)) {
		dev_err(&pdev->dev, "unable to get fclock\n");
		ret = PTR_ERR(am35x->fck);
		goto err1;
	}

	musb = platform_device_alloc("musb_hdrc", -1);
	if (!musb) {
		dev_err(&pdev->dev, "failed to allocate musb device\n");
		ret = -ENOMEM;
		goto err2;
	}

	pdata->ops = &am35x_musb_ops;

	musb->dev.dma_mask = &musb_dmamask;
	musb->dev.coherent_dma_mask = musb_dmamask;

	platform_device_add_resources(musb, pdev->resource,
					pdev->num_resources);

	ret = platform_device_add_data(musb, pdata, sizeof(*pdata));
	if (ret) {
		dev_err(&pdev->dev, "failed to add platform_data\n");
		goto err3;
	}

	ret = clk_enable(am35x->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable clock\n");
		goto err3;
	}

	ret = clk_enable(am35x->fck);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable fclock\n");
		goto err4;
	}

	platform_set_drvdata(pdev, am35x);
	am35x->dev = &pdev->dev;

	ret = platform_device_add(musb);
	if (ret) {
		dev_err(&pdev->dev, "failed to register musb device\n");
		goto err5;
	}

	return 0;

err5:
	clk_disable(am35x->fck);
err4:
	clk_disable(am35x->clk);
err3:
	platform_device_put(musb);
err2:
	clk_put(am35x->fck);
err1:
	clk_put(am35x->clk);
err0:
	kfree(am35x);

	return ret;
}

static int __exit am35x_musb_remove(struct platform_device *pdev)
{
	struct am35x_musb_glue		*am35x = platform_get_drvdata(pdev);

	clk_disable(am35x->clk);
	clk_put(am35x->clk);
	clk_disable(am35x->fck);
	clk_put(am35x->fck);
	kfree(am35x);

	return 0;
}

static struct platform_driver am35x_musb_driver = {
	.remove		= __exit_p(am35x_musb_remove),
	.driver		= {
		.name	= "musb-am35x",
	},
};

MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_DESCRIPTION("AM35x MUSB Glue Layer");
MODULE_LICENSE("GPL v2");

static int __init am35x_glue_init(void)
{
	return platform_driver_probe(&am35x_musb_driver, am35x_musb_probe);
}
subsys_initcall(am35x_glue_init);

static void __exit am35x_glue_exit(void)
{
	platform_driver_unregister(&am35x_musb_driver);
}
module_exit(am35x_glue_exit);
