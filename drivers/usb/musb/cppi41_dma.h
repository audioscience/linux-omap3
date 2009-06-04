/*
 * Copyright (C) 2005-2006 by Texas Instruments
 * Copyright (c) 2008, MontaVista Software, Inc. <source@mvista.com>
 *
 * This program is free software; you can distribute it and/or modify it
 * under the terms of the GNU General Public License (Version 2) as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */

#ifndef _CPPI41_DMA_H_
#define _CPPI41_DMA_H_

/* USB 2.0 OTG module registers */
#define USB_REVISION_REG	0x00
#define USB_CTRL_REG		0x04
#define USB_STAT_REG		0x08
#define USB_EMULATION_REG	0x0c
/* 0x10 Reserved */
#define USB_AUTOREQ_REG		0x14
#define USB_SRP_FIX_TIME_REG	0x18
#define USB_TEARDOWN_REG	0x1c
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
#define MOP_SOP_INTR_ENABLE	0x64
/* 0x68-0x6c Reserved */
#define USB_TX_MODE_REG		0x70	/* Transparent, CDC, [Generic] RNDIS */
#define USB_RX_MODE_REG		0x74	/* Transparent, CDC, [Generic] RNDIS */
#define EP_COUNT_MODE_REG	0x78
#define USB_GENERIC_RNDIS_EP_SIZE_REG(n) (0x80 + (((n) - 1) << 2))

#define QUEUE_THRESHOLD_INTR_ENABLE_REG	0xc0
#define	QUEUE_63_THRESHOLD_REG	0xc4
#define QUEUE_63_THRESHOLD_INTR_CLEAR_REG 0xc8
#define	QUEUE_65_THRESHOLD_REG	0xd4
#define QUEUE_65_THRESHOLD_INTR_CLEAR_REG 0xd8

/* Control register bits */
#define USB_SOFT_RESET_MASK	1

/* Mode register bits */
#define USB_MODE_SHIFT(n)	((((n) - 1) << 1))
#define USB_MODE_MASK(n)	(3 << USB_MODE_SHIFT(n))
#define USB_TRANSPARENT_MODE	0
#define USB_RNDIS_MODE		1
#define USB_CDC_MODE		2
#define USB_GENERIC_RNDIS_MODE	3

/* AutoReq register bits */
#define USB_RX_AUTOREQ_SHIFT(n) (((n) - 1) << 1)
#define USB_RX_AUTOREQ_MASK(n)	(3 << USB_RX_AUTOREQ_SHIFT(n))
#define USB_NO_AUTOREQ		0
#define USB_AUTOREQ_ALL_BUT_EOP 1
#define USB_AUTOREQ_ALWAYS	3

/* Teardown register bits */
#define USB_TX_TDOWN_SHIFT(n)	(16 + (n))
#define USB_TX_TDOWN_MASK(n)	(1 << USB_TX_TDOWN_SHIFT(n))
#define USB_RX_TDOWN_SHIFT(n)	(n)
#define USB_RX_TDOWN_MASK(n)	(1 << USB_RX_TDOWN_SHIFT(n))

/* USB interrupt register bits */
#define USB_INTR_USB_SHIFT	16
#define USB_INTR_USB_MASK	(0x1ff << USB_INTR_USB_SHIFT) /* 8 Mentor */
					/* interrupts and DRVVBUS interrupt */
#define USB_INTR_DRVVBUS	0x100
#define USB_INTR_RX_SHIFT	16
#define USB_INTR_TX_SHIFT	0

#define USB_MENTOR_CORE_OFFSET	0x400

#define USB_CPPI41_NUM_CH	15

/**
 * struct usb_cppi41_info - CPPI 4.1 USB implementation details
 * @dma_block:	DMA block number
 * @ep_dma_ch:	DMA channel numbers used for EPs 1 .. Max_EP
 * @q_mgr:	queue manager number
 * @num_tx_comp_q: number of the Tx completion queues
 * @num_rx_comp_q: number of the Rx queues
 * @tx_comp_q:	pointer to the list of the Tx completion queue numbers
 * @rx_comp_q:	pointer to the list of the Rx queue numbers
 */
struct usb_cppi41_info {
	u8 dma_block;
	u8 ep_dma_ch[USB_CPPI41_NUM_CH];
	u8 q_mgr;
	u8 num_tx_comp_q;
	u8 num_rx_comp_q;
	const u16 *tx_comp_q;
	const u16 *rx_comp_q;
};

extern const struct usb_cppi41_info usb_cppi41_info;

/**
 * cppi41_completion - Tx/Rx completion queue interrupt handling hook
 * @musb:	the controller
 * @rx: 	bitmask having bit N set if Rx queue N is not empty
 * @tx: 	bitmask having bit N set if Tx completion queue N is not empty
 */
void cppi41_completion(struct musb *musb, u32 rx, u32 tx);

#endif	/* _CPPI41_DMA_H_ */
