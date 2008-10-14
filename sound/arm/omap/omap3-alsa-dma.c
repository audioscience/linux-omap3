/*
 * sound/arm/omap/omap3-alsa-dma.c
 *
 * Common audio DMA handling for the OMAP processors
 *
 * Copyright (C) 20040-2007 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contibutors:
 * 	Jian Zhang
 *	Hari Nagalla
 *	Misael Lopez Cruz
 */

#include <linux/io.h>
#include <asm/arch/dma.h>
#include <asm/arch/mcbsp.h>

#include "omap-alsa-dma.h"
#include "omap-alsa-twl4030.h"

int twl4030_conf_data_interface(void)
{
	int ret = 0;
	int line = 0;
	int frame_length1 = OMAP_MCBSP_FRAMELEN_N(2);
	int word_length1 = OMAP_MCBSP_WORD_32;
	int frame_polarity = OMAP_MCBSP_FS_ACTIVE_LOW;
	int skip_alt = OMAP_MCBSP_SKIP_NONE;

	/* Check sample width */
	if (twl4030_local.bitspersample == AUDIO_SAMPLE_DATA_WIDTH_16) {
		if (twl4030_local.channels == STEREO_MODE) {
			frame_polarity = OMAP_MCBSP_FS_ACTIVE_HIGH;
		} else {
			/* Mono Mode */
			/* use 16 bits dma even though 32 bit width */
			word_length1 = OMAP_MCBSP_WORD_16;
		}
		/* 1 word */
		frame_length1 = OMAP_MCBSP_FRAMELEN_N(1);

	} else if (twl4030_local.bitspersample == AUDIO_SAMPLE_DATA_WIDTH_24) {
		if (twl4030_local.channels == MONO_MODE) {
			/* Mono Mode */
			/* use 32 bits dma and do doubleindex */
			skip_alt = OMAP_MCBSP_SKIP_SECOND;
		}
		/* 2 words */
		frame_length1 = OMAP_MCBSP_FRAMELEN_N(2);
	} else {
		printk(KERN_ERR "Unknown sample width %d\n",
			twl4030_local.bitspersample);
		return -EPERM;
	}

	/* Reset the McBSP registers so that we can
	 * configure it
	 */
	ret = omap2_mcbsp_reset(AUDIO_MCBSP);
	if (unlikely(ret)) {
		printk(KERN_ERR "conf_data Reset for MCBSP Failed[%d]\n", ret);
		/* Don't care about result */
		return ret;
	}

	/* Setup the new params */
	twl4030_mcbsp_settings.audio_mcbsp_tx_cfg_param.
						frame_length1 = frame_length1;
	twl4030_mcbsp_settings.audio_mcbsp_tx_cfg_param.
						word_length1 = word_length1;
	twl4030_mcbsp_settings.audio_mcbsp_tx_cfg_param.
						fs_polarity = frame_polarity;
	twl4030_mcbsp_settings.audio_mcbsp_tx_transfer_params.
						skip_alt = skip_alt;
	twl4030_mcbsp_settings.audio_mcbsp_tx_transfer_params.
						word_length1 = word_length1;

	twl4030_mcbsp_settings.audio_mcbsp_rx_cfg_param.
						frame_length1 = frame_length1;
	twl4030_mcbsp_settings.audio_mcbsp_rx_cfg_param.
						word_length1 = word_length1;
	twl4030_mcbsp_settings.audio_mcbsp_rx_cfg_param.
						fs_polarity = frame_polarity;
	twl4030_mcbsp_settings.audio_mcbsp_rx_transfer_params.
						skip_alt = skip_alt;
	twl4030_mcbsp_settings.audio_mcbsp_rx_transfer_params.
						word_length1 = word_length1;

#ifdef TWL_MASTER
	twl4030_mcbsp_settings.audio_mcbsp_srg_fsg_cfg.period = 0;
	twl4030_mcbsp_settings.audio_mcbsp_srg_fsg_cfg.pulse_width = 0;
	twl4030_mcbsp_settings.audio_mcbsp_srg_fsg_cfg.polarity = 0;
#else
	twl4030_mcbsp_settings.audio_mcbsp_srg_fsg_cfg.period =
				twl4030_local.bitspersample * 2 - 1;
	twl4030_mcbsp_settings.audio_mcbsp_srg_fsg_cfg.pulse_width =
				twl4030_local.bitspersample - 1;
	twl4030_mcbsp_settings.audio_mcbsp_srg_fsg_cfg.polarity = 1;
#endif
	twl4030_mcbsp_settings.audio_mcbsp_srg_fsg_cfg.sample_rate =
				twl4030_local.audio_samplerate;
	twl4030_mcbsp_settings.audio_mcbsp_srg_fsg_cfg.bits_per_sample =
				twl4030_local.bitspersample;

	ret = omap2_mcbsp_params_cfg(AUDIO_MCBSP,
#ifdef TWL_MASTER
			OMAP_MCBSP_SLAVE,
#else
			OMAP_MCBSP_MASTER,
#endif
			&(twl4030_mcbsp_settings.audio_mcbsp_rx_cfg_param),
			&(twl4030_mcbsp_settings.audio_mcbsp_tx_cfg_param),
			&(twl4030_mcbsp_settings.audio_mcbsp_srg_fsg_cfg));
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}
	ret = omap2_mcbsp_dma_recv_params(AUDIO_MCBSP,
		&(twl4030_mcbsp_settings.audio_mcbsp_rx_transfer_params));
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}
	ret = omap2_mcbsp_dma_recv_params(AUDIO_MCBSP,
		&(twl4030_mcbsp_settings.audio_mcbsp_tx_transfer_params));
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}

mcbsp_config_exit:
	if (unlikely(ret != 0))
		printk(KERN_ERR
		       "Unable to configure Mcbsp ret=%d @ line %d.", ret,
		       line);
	return ret;
}
EXPORT_SYMBOL(twl4030_conf_data_interface);


void omap_clear_alsa_sound_dma(struct audio_stream *s)
{
	/* Nothing to do for 3430 platform  */
}
EXPORT_SYMBOL(omap_clear_alsa_sound_dma);

int omap_request_alsa_sound_dma(int device_id, const char *device_name,
				void *data, int **channels)
{
	/* Nothing to do for 3430 platform */
	return 0;

}
EXPORT_SYMBOL(omap_request_alsa_sound_dma);

int omap_free_alsa_sound_dma(void *data, int **channels)
{
	/* Nothing to do for 3430 platform */
	return 0;
}
EXPORT_SYMBOL(omap_free_alsa_sound_dma);

int omap_init_alsa_sound_dma(int mode)
{
	int ret = 0;

	if (mode == SNDRV_PCM_STREAM_CAPTURE) {
		ret =
		    omap2_mcbsp_dma_recv_params(AUDIO_MCBSP,
				&(twl4030_mcbsp_settings.
				audio_mcbsp_rx_transfer_params));
		if (ret < 0) {
			printk(KERN_ERR "RECV params failed");
			goto transfer_exit;
		}
	} else {
		ret =
		    omap2_mcbsp_dma_trans_params(AUDIO_MCBSP,
				&(twl4030_mcbsp_settings.
				audio_mcbsp_tx_transfer_params));
		if (ret < 0) {
			printk(KERN_ERR "TRANS params failed");
			goto transfer_exit;
		}
	}
transfer_exit:
	return ret;
}
EXPORT_SYMBOL(omap_init_alsa_sound_dma);

/* Start DMA
 * Do the initial set of work to initialize all the channels as required.
 * We shall then initate a transfer
 */
int omap_start_alsa_sound_dma(struct audio_stream *s,
				dma_addr_t dma_ptr, u_int dma_size)
{
	int mode = s->stream_id;
	int ret = 0;

#ifdef TWL_DUMP_REGISTERS_MCBSP
	printk(KERN_INFO "TRANSFER");
	omap_mcbsp_dump_reg(AUDIO_MCBSP);
#endif
	if (mode == SNDRV_PCM_STREAM_CAPTURE) {
		/* Capture Path to be implemented */
		ret = omap2_mcbsp_receive_data(AUDIO_MCBSP, (void *)s,
					dma_ptr, dma_size);
	} else {
		ret = omap2_mcbsp_send_data(AUDIO_MCBSP, (void *)s,
					dma_ptr, dma_size);
	}

	return ret;
}
EXPORT_SYMBOL(omap_start_alsa_sound_dma);

int omap_stop_alsa_sound_dma(struct audio_stream *s)
{
	int mode = s->stream_id;
	int ret = 0;

	if (mode == SNDRV_PCM_STREAM_CAPTURE)
		ret = omap2_mcbsp_stop_datarx(AUDIO_MCBSP);
	else
		ret = omap2_mcbsp_stop_datatx(AUDIO_MCBSP);

	return ret;
}
EXPORT_SYMBOL(omap_stop_alsa_sound_dma);

static inline int element_size(int mcbsp_wordlen)
{
	if (mcbsp_wordlen == OMAP_MCBSP_WORD_32)
		return 4;
	if (mcbsp_wordlen == OMAP_MCBSP_WORD_16)
		return 2;
	/* We don't allow any other word lengths */
	return -1;
}

int omap_transfer_posn_alsa_sound_dma(struct audio_stream *s)
{
	int mode = s->stream_id;
	int ret = 0;
	int fi, ei;

	/* We always ask only one frame to transmit/recieve,
	 * variant is the element num
	 */
	if (mode == SNDRV_PCM_STREAM_CAPTURE) {
		ret = omap2_mcbsp_receiver_index(AUDIO_MCBSP,
						&ei, &fi);
		ret = ei * element_size(twl4030_mcbsp_settings.
			audio_mcbsp_rx_transfer_params.word_length1);
	} else {
		ret = omap2_mcbsp_transmitter_index(AUDIO_MCBSP,
						&ei, &fi);
		ret = ei * element_size(twl4030_mcbsp_settings.
			audio_mcbsp_tx_transfer_params.word_length1);
	}
	if (ret < 0)
		printk(KERN_ERR
		       "twl4030_transfer_posn: Unable to find index of "
		       "transfer\n");
	return ret;
}
EXPORT_SYMBOL(omap_transfer_posn_alsa_sound_dma);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Common DMA handling for Audio driver on OMAP3 processors");
MODULE_LICENSE("GPL");

