/*
 * <plat/asp.h> - TI816X Audio Serial Port support
 * Based on <arch/arm/mach-davinci/include/mach/asp.h>
 */
#ifndef __ASM_ARCH_TI816X_ASP_H
#define __ASM_ARCH_TI816X_ASP_H

#include <mach/irqs.h>
#include <asm/hardware/edma.h>

/* Bases of ti816x register banks */
#define TI816X_ASP0_BASE	0x48038000
#define TI816X_ASP1_BASE	0x4803C000
#define TI816X_ASP2_BASE	0x48050000

/* EDMA channels of ti816x McASP0 */
#define	TI816X_DMA_MCASP0_AXEVT	8
#define	TI816X_DMA_MCASP0_AREVT	9

/* EDMA channels of ti816x McASP1 */
#define	TI816X_DMA_MCASP1_AXEVT	10
#define	TI816X_DMA_MCASP1_AREVT	11

/* EDMA channels of ti816x McASP2 */
#define	TI816X_DMA_MCASP2_AXEVT	12
#define	TI816X_DMA_MCASP2_AREVT	13

/* Interrupts */
#define TI816X_ASP0_RX_INT	TI816X_IRQ_MCASP0_RX
#define TI816X_ASP0_TX_INT	TI816X_IRQ_MCASP0_TX
#define TI816X_ASP1_RX_INT	TI816X_IRQ_MCASP1_RX
#define TI816X_ASP1_TX_INT	TI816X_IRQ_MCASP1_TX
#define TI816X_ASP2_RX_INT	TI816X_IRQ_MCASP2_RX
#define TI816X_ASP2_TX_INT	TI816X_IRQ_MCASP2_TX

struct snd_platform_data {
	u32 tx_dma_offset;
	u32 rx_dma_offset;
	enum dma_event_q eventq_no;	/* event queue number */
	unsigned int codec_fmt;
	/*
	 * Allowing this is more efficient and eliminates left and right swaps
	 * caused by underruns, but will swap the left and right channels
	 * when compared to previous behavior.
	 */
	unsigned enable_channel_combine:1;
	unsigned sram_size_playback;
	unsigned sram_size_capture;

	/* McASP specific fields */
	int tdm_slots;
	u8 op_mode;
	u8 num_serializer;
	u8 *serial_dir;
	u8 version;
	u8 txnumevt;
	u8 rxnumevt;
};

enum {
	MCASP_VERSION_1 = 0,	/* DM646x */
	MCASP_VERSION_2,	/* DA8xx/OMAPL1x */
	MCASP_VERSION_3,	/* TODO: VB__Verify the version for TI816x */
};

#define INACTIVE_MODE	0
#define TX_MODE		1
#define RX_MODE		2

#define DAVINCI_MCASP_IIS_MODE	0 /* Driver code needs to change modified */
#define DAVINCI_MCASP_DIT_MODE	1

void ti816x_register_mcasp(int id, struct snd_platform_data *pdata);

#endif /* __ASM_ARCH_TI816X_ASP_H */
