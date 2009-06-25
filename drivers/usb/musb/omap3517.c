/*
 * Texas Instruments OMAP3517 "glue layer"
 *
 * Copyright (c) 2008, MontaVista Software, Inc. <source@mvista.com>
 *
 * Based on the DaVinci "glue layer" code.
 * Copyright (C) 2005-2006 by Texas Instruments
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

#include <mach/cppi41.h>
/* get this from core files later */
#define OMAP3517_CONF0                  0x0580
#define OMAP3517_LVL_INTR_CLR		0x0594
#define OMAP3517_IP_SW_RST		0x0598
#define OMAP3517_IP_CLK_CTRL		0x059c
#define BASE_OMAP3517_CONF0		(OMAP343X_SCM_BASE + OMAP3517_CONF0)
#define BASE_OMAP3517_IP_CLK_CTRL	(OMAP343X_SCM_BASE + OMAP3517_IP_CLK_CTRL)
#define BASE_OMAP3517_LVL_INTR_CLR	(OMAP343X_SCM_BASE + OMAP3517_LVL_INTR_CLR)
#define BASE_OMAP3517_IP_SW_RST		(OMAP343X_SCM_BASE + OMAP3517_IP_SW_RST)

#include "musb_core.h"
#include "cppi41_dma.h"

/*
 * OMAP3517 specific definitions
 */

/* CPPI 4.1 queue manager registers */
#define QMGR_PEND0_REG		0x4090
#define QMGR_PEND1_REG		0x4094
#define QMGR_PEND2_REG		0x4098

/* USB 2.0 PHY Control */
#define CONF0_USB1PHYCLKMUX    0 /* CHECK */
#define CONF0_USB2PHYCLKMUX    0 /* CHECK */
#define CONF0_USB1SUSPENDM     0 /* CHECK*/
#define CONF0_PHY_GPIOMODE     (1 << 23)
#define CONF0_OTGMODE          (3 << 14)
#define CONF0_SESENDEN         (1 << 13)       /* Vsess_end comparator */
#define CONF0_VBDTCTEN         (1 << 12)       /* Vbus comparator */
#define CONF0_REFFREQ_24MHZ    (2 << 8)
#define CONF0_REFFREQ_26MHZ    (7 << 8)
#define CONF0_REFFREQ_13MHZ    (6 << 8)
#define CONF0_REFFREQ          (0xf << 8)
#define CONF0_PHYCLKGD         (1 << 7)
#define CONF0_VBUSSENSE        (1 << 6)
#define CONF0_PHY_PLLON        (1 << 5)        /* override PLL suspend */
#define CONF0_RESET            (1 << 4)
#define CONF0_PHYPWRDN         (1 << 3)
#define CONF0_OTGPWRDN         (1 << 2)
#define CONF0_DATPOL           (1 << 1)


#define OMAP3517_TX_EP_MASK	0xffff		/* EP0 + 15 Tx EPs */
#define OMAP3517_RX_EP_MASK	0xfffe		/* 15 Rx EPs */

#define OMAP3517_TX_INTR_MASK	(OMAP3517_TX_EP_MASK << USB_INTR_TX_SHIFT)
#define OMAP3517_RX_INTR_MASK	(OMAP3517_RX_EP_MASK << USB_INTR_RX_SHIFT)

#define A_WAIT_BCON_TIMEOUT	1100		/* in ms */

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

int __init cppi41_init(void)
{
	u16 numch, blknum = usb_cppi41_info.dma_block, order;

	/* Initialize for Linking RAM region 0 alone */
	cppi41_queue_mgr_init(usb_cppi41_info.q_mgr, 0, 0x3fff);

	numch =  USB_CPPI41_NUM_CH * 2;
	order = get_count_order(numch);

	if (order < 7)	/* CHECK: twp teardown desc per channel (5 in primus)*/
		order = 7;

	cppi41_dma_block_init(blknum, usb_cppi41_info.q_mgr, order,
			dma_sched_table, numch);
	return 0;
}

#endif /* CONFIG_USB_TI_CPPI41_DMA */

/*
 * REVISIT (PM): we should be able to keep the PHY in low power mode most
 * of the time (24 MHZ oscillator and PLL off, etc) by setting POWER.D0
 * and, when in host mode, autosuspending idle root ports... PHYPLLON
 * (overriding SUSPENDM?) then likely needs to stay off.
 */

static inline void phy_on(void)
{
	u32 cfgchip2;

	/*
	 * Start the on-chip PHY and its PLL.
	 */
	cfgchip2 = __raw_readl(IO_ADDRESS(BASE_OMAP3517_CONF0));

	/* AVV: can be removed later */
	cfgchip2 &= ~(0x0000FFFF);
	__raw_writel(cfgchip2, IO_ADDRESS(BASE_OMAP3517_CONF0));
	cfgchip2 = __raw_readl(IO_ADDRESS(BASE_OMAP3517_CONF0));

	/* Check whether USB0 PHY is already powered on */
	/* if (cfgchip2 & CONF0_PHY_PLLON)
		return;
	*/

	cfgchip2 &= ~(CONF0_RESET | CONF0_PHYPWRDN | CONF0_OTGPWRDN |
		      CONF0_OTGMODE | CONF0_REFFREQ | CONF0_PHY_GPIOMODE);
	cfgchip2 |= CONF0_SESENDEN | CONF0_VBDTCTEN | CONF0_PHY_PLLON |
		    CONF0_REFFREQ_13MHZ | CONF0_DATPOL;
	__raw_writel(cfgchip2, IO_ADDRESS(BASE_OMAP3517_CONF0));

	pr_info("Waiting for PHY clock good...(value 0x%x written in conf0)\n", cfgchip2);
	while (!(__raw_readl(IO_ADDRESS(BASE_OMAP3517_CONF0)) & CONF0_PHYCLKGD))
		cpu_relax();
}

static inline void phy_off(void)
{
	u32 cfgchip2;

	/*
	 * Power down the on-chip PHY.
	 */
	cfgchip2 = __raw_readl(IO_ADDRESS(BASE_OMAP3517_CONF0));

	/* Ensure that usb1.1 interface clk is not being sourced from usb0
	 * interface.  If so do not power down usb0 PHY
	 */
	if ((cfgchip2 & CONF0_USB1SUSPENDM) &&
		!(cfgchip2 & CONF0_USB1PHYCLKMUX)) {
		printk(KERN_WARNING
			"USB1 interface active - Cannot Power down USB0 PHY\n");
		return;
	}

	cfgchip2 &= ~CONF0_PHY_PLLON;
	cfgchip2 |=  CONF0_PHYPWRDN | CONF0_OTGPWRDN;
	__raw_writel(cfgchip2, IO_ADDRESS(BASE_OMAP3517_CONF0));
}

/*
 * Because we don't set CTRL.UINT, it's "important" to:
 *	- not read/write INTRUSB/INTRUSBE (except during
 *	  initial setup, as a workaround);
 *	- use INTSET/INTCLR instead.
 */

/**
 * musb_platform_enable - enable interrupts
 */
void musb_platform_enable(struct musb *musb)
{
	void __iomem *reg_base = musb->ctrl_base;
	u32 epmask, coremask;

	/* Workaround: setup IRQs through both register sets. */
	epmask = ((musb->epmask & OMAP3517_TX_EP_MASK) << USB_INTR_TX_SHIFT) |
	       ((musb->epmask & OMAP3517_RX_EP_MASK) << USB_INTR_RX_SHIFT);
	coremask = (0x01ff << USB_INTR_USB_SHIFT);

	musb_writel(reg_base, EP_INTR_MASK_SET_REG, epmask);
	musb_writel(reg_base, CORE_INTR_MASK_SET_REG, coremask);

	/* Force the DRVVBUS IRQ so we can start polling for ID change. */
	if (is_otg_enabled(musb))
		musb_writel(reg_base, CORE_INTR_SRC_SET_REG,
			    USB_INTR_DRVVBUS << USB_INTR_USB_SHIFT);
}

/**
 * musb_platform_disable - disable HDRC and flush interrupts
 */
void musb_platform_disable(struct musb *musb)
{
	void __iomem *reg_base = musb->ctrl_base;

	musb_writel(reg_base, CORE_INTR_MASK_CLEAR_REG, USB_INTR_USB_MASK);
	musb_writel(reg_base, EP_INTR_MASK_CLEAR_REG,
			 OMAP3517_TX_INTR_MASK | OMAP3517_RX_INTR_MASK);
	musb_writeb(musb->mregs, MUSB_DEVCTL, 0);
	musb_writel(reg_base, USB_END_OF_INTR_REG, 0);
}

/* REVISIT: it's not clear whether OMAP3517 can support full OTG.  */

static int vbus_state = -1;

#ifdef CONFIG_USB_MUSB_HDRC_HCD
#define portstate(stmt) 	stmt
#else
#define portstate(stmt)
#endif

static void omap3517_source_power(struct musb *musb, int is_on, int immediate)
{
	if (is_on)
		is_on = 1;

	if (vbus_state == is_on)
		return;
	vbus_state = is_on;		/* 0/1 vs "-1 == unknown/init" */
}

static void omap3517_set_vbus(struct musb *musb, int is_on)
{
	WARN_ON(is_on && is_peripheral_active(musb));
	omap3517_source_power(musb, is_on, 0);
}

#define	POLL_SECONDS	2

static struct timer_list otg_workaround;

static void otg_timer(unsigned long _musb)
{
	struct musb		*musb = (void *)_musb;
	void __iomem		*mregs = musb->mregs;
	u8			devctl;
	unsigned long		flags;

	/* We poll because DaVinci's won't expose several OTG-critical
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
		/*
		 * Wait till VBUS falls below SessionEnd (~0.2 V); the 1.3
		 * RTL seems to mis-handle session "start" otherwise (or in
		 * our case "recover"), in routine "VBUS was valid by the time
		 * VBUSERR got reported during enumeration" cases.
		 */
		if (devctl & MUSB_DEVCTL_VBUS) {
			mod_timer(&otg_workaround, jiffies + POLL_SECONDS * HZ);
			break;
		}
		musb->xceiv->state = OTG_STATE_A_WAIT_VRISE;
		musb_writel(musb->ctrl_base, CORE_INTR_SRC_SET_REG,
			    MUSB_INTR_VBUSERROR << USB_INTR_USB_SHIFT);
		break;
	case OTG_STATE_B_IDLE:
		if (!is_peripheral_enabled(musb))
			break;

		/*
		 * There's no ID-changed IRQ, so we have no good way to tell
		 * when to switch to the A-Default state machine (by setting
		 * the DEVCTL.SESSION flag).
		 *
		 * Workaround:  whenever we're in B_IDLE, try setting the
		 * session flag every few seconds.  If it works, ID was
		 * grounded and we're now in the A-Default state machine.
		 *
		 * NOTE: setting the session flag is _supposed_ to trigger
		 * SRP but clearly it doesn't.
		 */
		musb_writeb(mregs, MUSB_DEVCTL, devctl | MUSB_DEVCTL_SESSION);
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

static irqreturn_t omap3517_interrupt(int irq, void *hci)
{
	struct musb  *musb = hci;
	void __iomem *reg_base = musb->ctrl_base;
	unsigned long flags;
	irqreturn_t ret = IRQ_NONE;
	u32 pend1 = 0, pend2 = 0, tx, rx;
	u32 status1, status2, lvl_intr;

	spin_lock_irqsave(&musb->lock, flags);

	/*
	 * NOTE: OMAP3517 shadows the Mentor IRQs.  Don't manage them through
	 * the Mentor registers (except for setup), use the TI ones and EOI.
	 */

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
#if 1
		pend1 = musb_readl(reg_base, QMGR_PEND1_REG);
		pend2 = musb_readl(reg_base, QMGR_PEND2_REG);
#endif
		tx = (pend1 >> 31)  | ((pend2 & 1) ? (1 << 1) : 0);
		rx = (pend2 >> 1) & 0x3;

		if (tx || rx) {

			DBG(4, "CPPI 4.1 IRQ: Tx %x, Rx %x\n", tx, rx);
			cppi41_completion(musb, rx, tx);
			ret = IRQ_HANDLED;
		}
	}

	/* Acknowledge and handle non-CPPI interrupts */
	/* Get endpoint interrupts */
	status1 = musb_readl(reg_base, EP_INTR_SRC_MASKED_REG);

	if (status1) {
		musb_writel(reg_base, EP_INTR_SRC_CLEAR_REG, status1);
		//printk("USB EP IRQ %08x\n", status1);

		musb->int_rx = (status1 & OMAP3517_RX_INTR_MASK) >> USB_INTR_RX_SHIFT;
		musb->int_tx = (status1 & OMAP3517_TX_INTR_MASK) >> USB_INTR_TX_SHIFT;
	}

	/* Get usb core interrupts */
	status2 = musb_readl(reg_base, CORE_INTR_SRC_MASKED_REG);
	if (!status2 && !status1)
		goto eoi;

	if (status2) {
		musb_writel(reg_base, CORE_INTR_SRC_CLEAR_REG, status2);
		//printk("USB CORE IRQ %08x\n", status2);

		musb->int_usb = (status2 & USB_INTR_USB_MASK) >> USB_INTR_USB_SHIFT;
		/* musb->int_regs = regs; */
	}
	/*
	 * DRVVBUS IRQs are the only proxy we have (a very poor one!) for
	 * OMAP3517's missing ID change IRQ.  We need an ID change IRQ to
	 * switch appropriately between halves of the OTG state machine.
	 * Managing DEVCTL.SESSION per Mentor docs requires that we know its
	 * value but DEVCTL.BDEVICE is invalid without DEVCTL.SESSION set.
	 * Also, DRVVBUS pulses for SRP (but not at 5V) ...
	 */
	if (status2 & (USB_INTR_DRVVBUS << USB_INTR_USB_SHIFT)) {
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
			musb->is_active = 1;
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
		omap3517_source_power(musb, drvvbus, 0);
		DBG(2, "VBUS %s (%s)%s, devctl %02x\n",
				drvvbus ? "on" : "off",
				otg_state_string(musb),
				err ? " ERROR" : "",
				devctl);
		ret = IRQ_HANDLED;
	}

	if (musb->int_tx || musb->int_rx || musb->int_usb) {
	        irqreturn_t mret;

		mret = musb_interrupt(musb);
		if (mret == IRQ_HANDLED)
			ret = IRQ_HANDLED;
	}

	/* musb->int_regs = NULL; */

 eoi:
	/* EOI needs to be written for the IRQ to be re-asserted. */
	if (ret == IRQ_HANDLED || status1 || status2) {
		/* clear level interrupt */
		lvl_intr = __raw_readl(IO_ADDRESS(BASE_OMAP3517_LVL_INTR_CLR));
		lvl_intr |= (1 << 4);
 		__raw_writel(lvl_intr, IO_ADDRESS(BASE_OMAP3517_LVL_INTR_CLR));
		musb_writel(reg_base, USB_END_OF_INTR_REG, 0);
	}

	/* Poll for ID change */
	if (is_otg_enabled(musb) && musb->xceiv->state == OTG_STATE_B_IDLE)
		mod_timer(&otg_workaround, jiffies + POLL_SECONDS * HZ);

	spin_unlock_irqrestore(&musb->lock, flags);

	if (ret != IRQ_HANDLED) {
		if (status1 || status2)
			/*
			 * We sometimes get unhandled IRQs in the peripheral
			 * mode from EP0 and SOF...
			 */
			DBG(2, "Unhandled USB IRQ %08x-%08x\n",
					 status1, status2);
		else if (printk_ratelimit())
			/*
			 * We've seen series of spurious interrupts in the
			 * peripheral mode after USB reset and then after some
			 * time a real interrupt storm starting...
			 */
			DBG(2, "Spurious IRQ, CPPI 4.1 status %08x, %08x\n",
					 pend1, pend2);
	}
	return ret;
}

int musb_platform_set_mode(struct musb *musb, u8 musb_mode)
{
	/* TODO: implement this using CONF0 */
	WARNING("FIXME: %s not implemented\n", __func__);
	return -EIO;
}

int __init musb_platform_init(struct musb *musb)
{
	void __iomem *reg_base = musb->ctrl_base;
	struct clk              *otg_fck;
	u32 rev, lvl_intr, sw_reset;

	usb_nop_xceiv_register();

	musb->xceiv = otg_get_transceiver();
	if (!musb->xceiv)
		return -ENODEV;

	/* mentor is at offset of 0x400 in omap3517 */
	musb->mregs += USB_MENTOR_CORE_OFFSET;

	/* not required as clock is set in usb-musb.c file in arch */
	/* musb->clock = clk_get(NULL, "usbotg_vbusp_ck"); */
	if (IS_ERR(musb->clock))
		return PTR_ERR(musb->clock);

	if (clk_enable(musb->clock) < 0)
		return -ENODEV;

	otg_fck = clk_get(NULL, "usbotg_fck");
	clk_enable(otg_fck);

	/* Returns zero if e.g. not clocked */
	rev = musb_readl(reg_base, USB_REVISION_REG);
	if (!rev)
		return -ENODEV;

#ifdef CONFIG_USB_TI_CPPI41_DMA
	cppi41_init();
#endif

	if (is_host_enabled(musb))
		setup_timer(&otg_workaround, otg_timer, (unsigned long) musb);

	musb->board_set_vbus = omap3517_set_vbus;
	omap3517_source_power(musb, 0, 1);

#if 0 /* follow reset procedure */
	/* Reset the controller */
	musb_writel(reg_base, USB_CTRL_REG, USB_SOFT_RESET_MASK);

	/* wait till reset bit clears */
	while ((musb_readl(reg_base, USB_CTRL_REG) & 0x1))
		cpu_relax();

	/* clock disable */
	clk_disable(musb->clock);

	/* Start the on-chip PHY and its PLL. */
	phy_on();

	msleep(5);

	/* clock enable */
	clk_enable(musb->clock);

#else

	/* global reset */
	sw_reset = __raw_readl(IO_ADDRESS(BASE_OMAP3517_IP_SW_RST));

	sw_reset |= (1 << 0);
	__raw_writel(sw_reset, IO_ADDRESS(BASE_OMAP3517_IP_SW_RST));

	sw_reset &= ~(1 << 0);
	__raw_writel(sw_reset, IO_ADDRESS(BASE_OMAP3517_IP_SW_RST));

	/* Reset the controller */
	musb_writel(reg_base, USB_CTRL_REG, USB_SOFT_RESET_MASK);

#if 0	/* EOI toggle */
	musb_writel(reg_base, USB_END_OF_INTR_REG, 0);
	musb_writel(reg_base, USB_END_OF_INTR_REG, 1);
	musb_writel(reg_base, USB_END_OF_INTR_REG, 0);
#endif
	/* Start the on-chip PHY and its PLL. */
	phy_on();

	msleep(15);
#endif
	/* ip clk control wait till VBUSP clk disabled */
	printk("BASE_OMAP3517_IP_CLK_CTRL = 0x%x\n",
			__raw_readl(IO_ADDRESS(BASE_OMAP3517_IP_CLK_CTRL)));
	/* while ((__raw_readl(IO_ADDRESS(BASE_OMAP3517_IP_CLK_CTRL)) & 0x1))
		cpu_relax();
	*/

	/* __raw_writel(OMAP3517_KICK0_MAGIC, IO_ADDRESS(OMAP3517_KICK0)); */
	/* __raw_writel(OMAP3517_KICK1_MAGIC, IO_ADDRESS(OMAP3517_KICK1)); */
	/* NOTE: IRQs are in mixed mode, not bypass to pure MUSB */
	printk("OMAP3517 OTG revision %08x, PHY %03x, control %02x\n",
		 rev, __raw_readl(IO_ADDRESS(BASE_OMAP3517_CONF0)),
		 musb_readb(reg_base, USB_CTRL_REG));

	musb->a_wait_bcon = A_WAIT_BCON_TIMEOUT;
	musb->isr = omap3517_interrupt;
	/* update the HS EOF tiing */
	/* musb_writeb(musb->mregs, 0x7C, 0x40); */

	/* clear level interrupt */
#if 1
	lvl_intr = __raw_readl(IO_ADDRESS(BASE_OMAP3517_LVL_INTR_CLR));
	lvl_intr |= (1 << 4);
	__raw_writel(lvl_intr, IO_ADDRESS(BASE_OMAP3517_LVL_INTR_CLR));
#endif
	return 0;
}

int musb_platform_exit(struct musb *musb)
{
	if (is_host_enabled(musb))
		del_timer_sync(&otg_workaround);

	omap3517_source_power(musb, 0 /* off */, 1);

	/* Delay to avoid problems with module reload... */
	if (is_host_enabled(musb) && musb->xceiv->default_a) {
		int maxdelay = 30;
		u8 devctl, warn = 0;

		/*
		 * If there's no peripheral connected, this can take a
		 * long time to fall...
		 */
		do {
			devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
			if (!(devctl & MUSB_DEVCTL_VBUS))
				break;
			if ((devctl & MUSB_DEVCTL_VBUS) != warn) {
				warn = devctl & MUSB_DEVCTL_VBUS;
				DBG(1, "VBUS %d\n",
					warn >> MUSB_DEVCTL_VBUS_SHIFT);
			}
			msleep(1000);
			maxdelay--;
		} while (maxdelay > 0);

		/* In OTG mode, another host might be connected... */
		if (devctl & MUSB_DEVCTL_VBUS)
			DBG(1, "VBUS off timeout (devctl %02x)\n", devctl);
	}

	phy_off();

	usb_nop_xceiv_unregister();

	return 0;
}

void musb_platform_try_idle(struct musb *musb, unsigned long timeout)
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

