/*
 * sound/arm/omap/omap-audio-twl4030-mixer.c
 *
 * Codec driver for TWL4030 for OMAP processors
 *
 * Copyright (C) 2004-2007 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contributors:
 *	Nishanth Menon
 *	Jian Zhang
 *	Leonides Martinez
 *	Hari Nagalla
 *	Misael Lopez Cruz
 */

#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/errno.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <linux/uaccess.h>
#include <linux/io.h>

#include <asm/arch/hardware.h>
#include <asm/arch/dma.h>
#include <asm/mach-types.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/control.h>

#include <asm/arch/omap-alsa.h>
#include <linux/i2c/twl4030.h>
#if defined(CONFIG_ARCH_OMAP243X) || defined(CONFIG_ARCH_OMAP3430)
#include <asm/arch/mcbsp.h>
#include <asm/arch/clock.h>
#else
#error "Unsupported configuration"
#endif

#include "omap-alsa-twl4030.h"

/*
 *  ALSA mixer Callback 'info' for Stereo Playback Volume Controls
 */
static int pcm_playback_volume_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count			= 2;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= AUDIO_MAX_OUTPUT_VOLUME;

	return 0;
}

/*
 *  ALSA mixer Callback 'info' for Mono Playback Volume Controls
 */
static int pcm_mono_playback_volume_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= AUDIO_MAX_OUTPUT_VOLUME;

	return 0;
}

/*
 * Sidetone 'info'
 */
static int sidetone_volume_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= SIDETONE_MAX_GAIN;

	return 0;
}

/*
 * Alsa mixer interface function for getting the volume read from the DGC in a
 * 0 -100 alsa mixer format.
 */
static int pcm_playback_volume_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
		READ_LEFT_VOLUME(twl4030_local.master_play_vol); /* L */
	ucontrol->value.integer.value[1] =
		READ_RIGHT_VOLUME(twl4030_local.master_play_vol);/* R */

	return 0;
}

/*
 * Alsa mixer interface function for setting the master playback volume
 */
static int pcm_playback_volume_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;

	if ((READ_LEFT_VOLUME(twl4030_local.master_play_vol) !=
					ucontrol->value.integer.value[0]) |
	    (READ_RIGHT_VOLUME(twl4030_local.master_play_vol) !=
					ucontrol->value.integer.value[1])) {
		changed = twl4030_setvolume(OUTPUT_VOLUME,
				ucontrol->value.integer.value[0],
				ucontrol->value.integer.value[1]);
		if (!changed)
			changed = 1;
	}

	return changed;
}

/*
 * Headset Get Volume
 */
static int headset_playback_volume_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
		READ_LEFT_VOLUME(twl4030_local.headset_vol); /* L */
	ucontrol->value.integer.value[1] =
		READ_RIGHT_VOLUME(twl4030_local.headset_vol);/* R */

	return 0;
}

/*
 * Headset Set Volume
 */
static int headset_playback_volume_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;

	if ((READ_LEFT_VOLUME(twl4030_local.headset_vol) !=
					ucontrol->value.integer.value[0]) |
	    (READ_RIGHT_VOLUME(twl4030_local.headset_vol) !=
					ucontrol->value.integer.value[1])) {
		changed = twl4030_setvolume(OUTPUT_STEREO_HEADSET,
				ucontrol->value.integer.value[0],
				ucontrol->value.integer.value[1]);
		if (!changed)
			changed = 1;
	}

	return changed;
}

/*
 * Switch info
 */
static int pcm_switch_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type 			= SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= 1;

	return 0;
}

/*
 * Handsfree Switch Control
 */
static int handsfree_playback_switch_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	if (twl4030_local.output_src == OUTPUT_HANDS_FREE_CLASSD)
		ucontrol->value.integer.value[0] =
					((twl4030_local.handsfree_en) ? 1 : 0);
	else
		ucontrol->value.integer.value[0] = 0;

	return 0;
}

static int handsfree_playback_switch_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;

	if (ucontrol->value.integer.value[0] != (twl4030_local.handsfree_en)) {
		if (ucontrol->value.integer.value[0])
			twl4030_local.handsfree_en = 1;
		else
			twl4030_local.handsfree_en = 0;
		changed = 1;
	}

	return changed;
}

/* Controls to set the audio sample rate */
static int codec_audio_samplerate_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 8000;
	uinfo->value.integer.max	= 48000;

	return 0;
}

static int codec_audio_samplerate_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = twl4030_local.audio_samplerate;

	return 0;
}

static int codec_audio_samplerate_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;

	if (ucontrol->value.integer.value[0] !=
					(twl4030_local.audio_samplerate)) {
		twl4030_set_audio_samplerate(ucontrol->value.integer.value[0]);
		changed = 1;
	}

	return changed;
}

/* Controls to set the voice sample rate */
static int codec_voice_samplerate_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 8000;
	uinfo->value.integer.max	= 16000;

	return 0;
}

static int codec_voice_samplerate_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = twl4030_local.voice_samplerate;

	return 0;
}

static int codec_voice_samplerate_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;

	if (ucontrol->value.integer.value[0] !=
					(twl4030_local.voice_samplerate)) {
		twl4030_set_voice_samplerate(ucontrol->value.integer.value[0]);
		changed = 1;
	}

	return changed;
}


/*
 * Handset Earphone Control
 */
static int earphone_playback_volume_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
		READ_LEFT_VOLUME(twl4030_local.ear_vol); /* L  Mono */

	return 0;
}

static int earphone_playback_volume_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;

	if (READ_LEFT_VOLUME(twl4030_local.ear_vol) !=
				ucontrol->value.integer.value[0]) {
		changed = twl4030_setvolume(OUTPUT_MONO_EARPIECE,
				ucontrol->value.integer.value[0], 0);
		if (!changed)
			changed = 1;
	}

	return changed;
}

/*
 * Downlink Volume Control
 */
static int downlink_playback_volume_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
		READ_LEFT_VOLUME(twl4030_local.downlink_vol);

	return 0;
}

static int downlink_playback_volume_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;

	if (READ_LEFT_VOLUME(twl4030_local.downlink_vol) !=
				ucontrol->value.integer.value[0]) {
		changed = twl4030_setvolume(OUTPUT_DOWNLINK,
				ucontrol->value.integer.value[0], 0);
		if (!changed)
			changed = 1;
	}

	return changed;
}

/*
 * Sidetone Volume Control
 */
static int sidetone_playback_volume_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	u8 data = 0;
	int ret = 0;

	ret = audio_twl4030_read(REG_VSTPGA, &data);
	if (ret) {
		printk(KERN_ERR "Failed reading sidetone volume %d\n", ret);
		return ret;
	}
	ucontrol->value.integer.value[0] = (int) data;

	return 0;
}

static int sidetone_playback_volume_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;

	if (READ_LEFT_VOLUME(twl4030_local.sidetone_vol) !=
					ucontrol->value.integer.value[0]) {
		changed = twl4030_setvolume(OUTPUT_SIDETONE,
				ucontrol->value.integer.value[0], 0);
		if (!changed)
			changed = 1;
	}

	return changed;
}

/*
 * Sidetone Switch Control
 */
static int sidetone_playback_switch_get(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	u8 data = 0;
	int ret = 0;

	ret = audio_twl4030_read(REG_VSTPGA, &data);
	if (ret) {
		printk(KERN_ERR "Failed reading sidetone switch %d\n", ret);
		return ret;
	}
	ucontrol->value.integer.value[0] = (data ? 1 : 0);

	return 0;
}

static int sidetone_playback_switch_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	u8 data = 0;
	int ret = 0;

	ret = audio_twl4030_read(REG_VSTPGA, &data);
	if (ret) {
		printk(KERN_ERR "Failed reading sidetone switch %d\n", ret);
		return ret;
	}

	if (ucontrol->value.integer.value[0] != (data ? 1 : 0)) {
		if (ucontrol->value.integer.value[0]) {
			/* Enable sidetone */
			ret = audio_twl4030_write(REG_VSTPGA,
						twl4030_local.sidetone_vol);
			if (ret) {
				printk(KERN_ERR
					"Sidetone enable failed %d\n", ret);
				return ret;
			}
		} else {
			/* Disable sidetone = mute */
			ret = audio_twl4030_write(REG_VSTPGA, 0);
			if (ret) {
				printk(KERN_ERR
					"Sidetone disable failed %d\n", ret);
				return ret;
			}
		}
		ret = 1;
	}

	return ret;
}

/*
 * USB-Carkit Gain Control
 */
static int carkit_playback_volume_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
		READ_LEFT_VOLUME(twl4030_local.carkit_vol); /* L */
	ucontrol->value.integer.value[1] =
		READ_RIGHT_VOLUME(twl4030_local.carkit_vol);/* R */

	return 0;
}

static int carkit_playback_volume_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;

	if ((READ_LEFT_VOLUME(twl4030_local.carkit_vol) !=
					ucontrol->value.integer.value[0]) |
	    (READ_RIGHT_VOLUME(twl4030_local.carkit_vol) !=
					ucontrol->value.integer.value[1])) {
		changed = twl4030_setvolume(OUTPUT_CARKIT,
				ucontrol->value.integer.value[0],
				ucontrol->value.integer.value[1]);
		if (!changed)
			changed = 1;
	}

	return changed;
}

static int carkit_capture_volume_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
		READ_LEFT_VOLUME(twl4030_local.carkit_mic_vol); /* L  Mono */

	return 0;
}

static int carkit_capture_volume_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;

	if (READ_LEFT_VOLUME(twl4030_local.carkit_mic_vol) !=
				ucontrol->value.integer.value[0]) {
		changed = twl4030_setvolume(INPUT_CARKIT,
				ucontrol->value.integer.value[0], 0);
		if (!changed)
			changed = 1;
	}

	return changed;
}

/*
 *  Info callback for Stereo Capture Volume Controls
 */
static int pcm_capture_volume_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count			= 2;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= AUDIO_MAX_INPUT_VOLUME;

	return 0;
}

/*
 *  Info callback for Mono Capture Volume Controls
 */
static int mono_capture_volume_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type			= SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count			= 1;
	uinfo->value.integer.min	= 0;
	uinfo->value.integer.max	= AUDIO_MAX_INPUT_VOLUME;

	return 0;
}

/*
 * Master Capture Volume Control
 */
static int pcm_capture_volume_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
		READ_LEFT_VOLUME(twl4030_local.master_rec_vol); /* L */
	ucontrol->value.integer.value[1] =
		READ_RIGHT_VOLUME(twl4030_local.master_rec_vol); /* R */

	return 0;
}

static int pcm_capture_volume_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;

	if ((READ_LEFT_VOLUME(twl4030_local.master_rec_vol) !=
					ucontrol->value.integer.value[0]) |
	    (READ_RIGHT_VOLUME(twl4030_local.master_rec_vol) !=
					ucontrol->value.integer.value[1])) {
		changed =  twl4030_setvolume(INPUT_VOLUME,
				ucontrol->value.integer.value[0],
				ucontrol->value.integer.value[1]);
		if (!changed)
			changed = 1;
	}

	return changed;
}

/*
 * Headset Mic Control
 */
static int hset_mic_capture_switch_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	if (twl4030_local.input_src == INPUT_HEADSET_MIC)
		ucontrol->value.integer.value[0] =
				((twl4030_local.headset_mic_en) ? 1 : 0);
	else
		ucontrol->value.integer.value[0] = 0;

	return 0;
}

static int hset_mic_capture_switch_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;

	if (ucontrol->value.integer.value[0] !=
					(twl4030_local.headset_mic_en)) {
		if (ucontrol->value.integer.value[0])
			twl4030_local.headset_mic_en = 1;
		else
			twl4030_local.headset_mic_en = 0;
		changed = 1;
	}

	return changed;
}

static int hset_mic_capture_volume_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
		READ_LEFT_VOLUME(twl4030_local.headset_mic_vol); /* L */
	ucontrol->value.integer.value[1] =
		READ_RIGHT_VOLUME(twl4030_local.headset_mic_vol); /* R */

	return 0;
}

static int hset_mic_capture_volume_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;

	if ((READ_LEFT_VOLUME(twl4030_local.headset_mic_vol) !=
					ucontrol->value.integer.value[0]) |
	    (READ_RIGHT_VOLUME(twl4030_local.headset_mic_vol) !=
					ucontrol->value.integer.value[1])) {
		changed = twl4030_setvolume(INPUT_HEADSET_MIC,
				ucontrol->value.integer.value[0],
				ucontrol->value.integer.value[1]);
		if (!changed)
			changed = 1;
	}

	return changed;
}


/*
 * Main Mic Control (Mic 1)
 */
static int mic1_capture_volume_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
		READ_LEFT_VOLUME(twl4030_local.main_mic_vol); /* L */

	return 0;
}

static int mic1_capture_volume_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;

	if (READ_LEFT_VOLUME(twl4030_local.main_mic_vol) !=
					ucontrol->value.integer.value[0]) {
		changed =  twl4030_setvolume(INPUT_MAIN_MIC,
					ucontrol->value.integer.value[0], 0);
		if (!changed)
			changed = 1;
	}

	return changed;
}

static int mic1_capture_switch_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	if (twl4030_local.input_src == INPUT_MAIN_MIC)
		ucontrol->value.integer.value[0] =
					((twl4030_local.main_mic_en) ? 1 : 0);
	else
		ucontrol->value.integer.value[0] = 0;

	return 0;
}

static int mic1_capture_switch_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;

	if (ucontrol->value.integer.value[0] != (twl4030_local.main_mic_en)) {
		if (ucontrol->value.integer.value[0])
			twl4030_local.main_mic_en = 1;
		else
			twl4030_local.main_mic_en = 0;
		changed = 1;
	}

	return changed;
}


/*
 * Sub-Mic Control (Mic 2)
 */
static int mic2_capture_switch_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	if (twl4030_local.input_src == INPUT_SUB_MIC)
		ucontrol->value.integer.value[0] =
					((twl4030_local.sub_mic_en) ? 1 : 0);

	return 0;
}

static int mic2_capture_switch_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;

	if (ucontrol->value.integer.value[0] != (twl4030_local.sub_mic_en)) {
		if (ucontrol->value.integer.value[0])
			twl4030_local.sub_mic_en = 1;
		else
			twl4030_local.sub_mic_en = 0;
		changed = 1;
	}

	return changed;
}

static int mic2_capture_volume_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
		READ_RIGHT_VOLUME(twl4030_local.sub_mic_vol); /* R */

	return 0;
}

static int mic2_capture_volume_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;

	if (READ_RIGHT_VOLUME(twl4030_local.sub_mic_vol) !=
					ucontrol->value.integer.value[0]) {
		changed = twl4030_setvolume(INPUT_SUB_MIC, 0,
					ucontrol->value.integer.value[0]);
		if (!changed)
			changed = 1;
	}

	return changed;
}

/*
 * Auxiliary/FM Gain Control
 */
static int aux_capture_volume_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
		READ_LEFT_VOLUME(twl4030_local.aux_vol); /* L */
	ucontrol->value.integer.value[1] =
		READ_RIGHT_VOLUME(twl4030_local.aux_vol); /* R */

	return 0;
}

static int aux_capture_volume_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;

	if ((READ_LEFT_VOLUME(twl4030_local.aux_vol) !=
					ucontrol->value.integer.value[0]) |
	    (READ_RIGHT_VOLUME(twl4030_local.aux_vol) !=
					ucontrol->value.integer.value[1])) {
		changed = twl4030_setvolume(INPUT_AUX,
				ucontrol->value.integer.value[0],
				ucontrol->value.integer.value[1]);
		if (!changed)
			changed = 1;
	}

	return changed;
}

/* Output Source Selection */
static int snd_playback_source_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	static char *texts[4] = {"Stereo Headset",
				"Hands-free (Speakers)",
				"Mono Handset",
				"USB CarKit"};

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 4;
	if (uinfo->value.enumerated.item > 3)
		uinfo->value.enumerated.item = 0;
	strcpy(uinfo->value.enumerated.name,
		texts[uinfo->value.enumerated.item]);

	return 0;
}

static int snd_playback_source_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	if (twl4030_local.output_src == HEADSET_SOURCE)
		ucontrol->value.enumerated.item[0] = 0;
	else if (twl4030_local.output_src == HANDS_FREE_SOURCE)
		ucontrol->value.enumerated.item[0] = 1;
	else if (twl4030_local.output_src == EARPIECE_SOURCE)
		ucontrol->value.enumerated.item[0] = 2;
	else if (twl4030_local.output_src == CARKIT_SOURCE)
		ucontrol->value.enumerated.item[0] = 3;

	return 0;
}

static int snd_playback_source_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;
	int ret;

	if (ucontrol->value.enumerated.item[0] == 0)
		twl4030_local.output_src = HEADSET_SOURCE;
	else if (ucontrol->value.enumerated.item[0] == 1)
		twl4030_local.output_src = HANDS_FREE_SOURCE;
	else if (ucontrol->value.enumerated.item[0] == 2)
		twl4030_local.output_src = EARPIECE_SOURCE;
	else if (ucontrol->value.enumerated.item[0] == 3)
		twl4030_local.output_src = CARKIT_SOURCE;

	ret = twl4030_codec_tog_on();
	if (unlikely(ret)) {
		printk(KERN_ERR "Codec power tog failed!\n");
		return ret;
	}
	/* setup regs */
	ret = twl4030_select_source(twl4030_local.output_src);
	if (unlikely(ret)) {
		printk(KERN_ERR "Source selection failed!\n");
		return ret;
	}
	ret = twl4030_codec_tog_on();
	if (unlikely(ret)) {
		printk(KERN_ERR "Codec power tog failed!\n");
		return ret;
	}
	changed = 1;

	return changed;
}

/* Input Source Selection */
static int snd_capture_source_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	static char *texts[4] = {"Headset Mic",
				"Main Mic + Sub Mic",
				"Aux/FM",
				"USB CarKit"};

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 4;
	if (uinfo->value.enumerated.item > 3)
		uinfo->value.enumerated.item = 0;
	strcpy(uinfo->value.enumerated.name,
		texts[uinfo->value.enumerated.item]);

	return 0;
}

static int snd_capture_source_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	if (twl4030_local.input_src == HEADSET_MIC_SOURCE)
		ucontrol->value.enumerated.item[0] = 0;
	else if (twl4030_local.input_src == MAIN_SUB_MIC_SOURCE)
		ucontrol->value.enumerated.item[0] = 1;
	else if (twl4030_local.input_src == AUX_SOURCE)
		ucontrol->value.enumerated.item[0] = 2;
	else if (twl4030_local.input_src == CARKIT_MIC_SOURCE)
		ucontrol->value.enumerated.item[0] = 3;

	return 0;
}

static int snd_capture_source_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;
	int ret;

	if (ucontrol->value.enumerated.item[0] == 0)
		twl4030_local.input_src = HEADSET_MIC_SOURCE;
	else if (ucontrol->value.enumerated.item[0] == 1)
		twl4030_local.input_src = MAIN_SUB_MIC_SOURCE;
	else if (ucontrol->value.enumerated.item[0] == 2)
		twl4030_local.input_src = AUX_SOURCE;
	else if (ucontrol->value.enumerated.item[0] == 3)
		twl4030_local.input_src = CARKIT_MIC_SOURCE;

	ret = twl4030_codec_tog_on();
	if (unlikely(ret)) {
		printk(KERN_ERR "Codec power tog failed!\n");
		return ret;
	}
	/* setup regs */
	ret = twl4030_select_source(twl4030_local.input_src);
	if (unlikely(ret)) {
		printk(KERN_ERR "Source selection failed!\n");
		return ret;
	}
	ret = twl4030_codec_tog_on();
	if (unlikely(ret)) {
		printk(KERN_ERR "Codec power tog failed!\n");
		return ret;
	}
	changed = 1;

	return changed;
}

/* Output Source Selection */
static int snd_codec_mode_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	static char *texts[2] = {"Audio mode",
				"Voice mode"};

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 2;
	if (uinfo->value.enumerated.item > 1)
		uinfo->value.enumerated.item = 0;
	strcpy(uinfo->value.enumerated.name,
		texts[uinfo->value.enumerated.item]);

	return 0;
}

static int snd_codec_mode_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	if (twl4030_local.codec_mode == AUDIO_MODE)
		ucontrol->value.enumerated.item[0] = 0;
	else if (twl4030_local.codec_mode == VOICE_MODE)
		ucontrol->value.enumerated.item[0] = 1;

	return 0;
}

static int snd_codec_mode_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int changed = 0;
	int ret;

	if (ucontrol->value.enumerated.item[0] == 0)
		twl4030_local.codec_mode = AUDIO_MODE;
	else if (ucontrol->value.enumerated.item[0] == 1)
		twl4030_local.codec_mode = VOICE_MODE;

	ret = twl4030_codec_on();
	if (ret) {
		printk(KERN_ERR "Failed turning codec on %d\n", ret);
		return 0;
	}
	ret = twl4030_configure(twl4030_local.codec_mode);
	if (ret) {
		printk(KERN_ERR "Error changing codec mode %d\n", ret);
		changed = 0;
	} else {
		changed = 1;
	}

	return changed;
}


/* Controls Registered */

static struct snd_kcontrol_new twl4030_control[] __devinitdata = {
	/* Codec Mode selection */
	{
		.name   = "Codec mode",		/* Codec  mode */
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = snd_codec_mode_info,
		.get    = snd_codec_mode_get,
		.put    = snd_codec_mode_put,
	},
	/* Output Control*/
	{
		.name   = "TWL4030 Audio Sample Rate",
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = codec_audio_samplerate_info,
		.get    = codec_audio_samplerate_get,
		.put    = codec_audio_samplerate_put,
	},
	{
		.name   = "TWL4030 Voice Sample Rate",
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = codec_voice_samplerate_info,
		.get    = codec_voice_samplerate_get,
		.put    = codec_voice_samplerate_put,
	},
	{
		.name   = "Master Playback Volume",
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = pcm_playback_volume_info,
		.get    = pcm_playback_volume_get,
		.put    = pcm_playback_volume_put,
	},
	{
		.name   = "Handset Playback Volume",	/* Mono */
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = pcm_mono_playback_volume_info,
		.get    = earphone_playback_volume_get,
		.put    = earphone_playback_volume_put,
	},
	{
		.name   = "Hands-free Playback Switch",	/* ClassD */
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = pcm_switch_info,
		.get    = handsfree_playback_switch_get,
		.put    = handsfree_playback_switch_put,
	},
	{
		.name   = "Headset Playback Volume",	/* Line 1*/
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = pcm_playback_volume_info,
		.get    = headset_playback_volume_get,
		.put    = headset_playback_volume_put,
	},
	/* Downlink Gain */
	{
		.name   = "Downlink Playback Volume",	/* Downlink */
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = pcm_mono_playback_volume_info,
		.get    = downlink_playback_volume_get,
		.put    = downlink_playback_volume_put,
	},
	/* Sidetone Gain */
	{
		.name   = "Sidetone Playback Switch",
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = pcm_switch_info,
		.get    = sidetone_playback_switch_get,
		.put    = sidetone_playback_switch_put,
	},
	{
		.name   = "Sidetone Playback Volume",
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = sidetone_volume_info,
		.get    = sidetone_playback_volume_get,
		.put    = sidetone_playback_volume_put,
	},
	{
		.name   = "USB-Carkit Playback Volume",	/* USB Carkit Output */
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = pcm_playback_volume_info,
		.get    = carkit_playback_volume_get,
		.put    = carkit_playback_volume_put,
	},
	/* Input Control */
	{
		.name   = "Master Capture Volume",
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = pcm_capture_volume_info,
		.get    = pcm_capture_volume_get,
		.put    = pcm_capture_volume_put,
	},
	{
		.name   = "Mic Headset Capture Switch",	/* HS Mic */
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = pcm_switch_info,
		.get    = hset_mic_capture_switch_get,
		.put    = hset_mic_capture_switch_put,
	},
	{
		.name   = "Mic Headset Capture Volume",
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,	/* HS Mic */
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = pcm_capture_volume_info,
		.get    = hset_mic_capture_volume_get,
		.put    = hset_mic_capture_volume_put,
	},
	{
		.name   = "Mic Main Capture Switch",	/* Mic 1 */
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = pcm_switch_info,
		.get    = mic1_capture_switch_get,
		.put    = mic1_capture_switch_put,
	},
	{
		.name   = "Mic Main Capture Volume",	/* Mic 1 */
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = mono_capture_volume_info,
		.get    = mic1_capture_volume_get,
		.put    = mic1_capture_volume_put,
	},
	{
		.name   = "Mic Sub Capture Switch",	/* Mic 2 */
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = pcm_switch_info,
		.get    = mic2_capture_switch_get,
		.put    = mic2_capture_switch_put,
	},
	{
		.name   = "Mic Sub Capture Volume",	/* Mic 2 */
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = mono_capture_volume_info,
		.get    = mic2_capture_volume_get,
		.put    = mic2_capture_volume_put,
	},
	{
		.name   = "Aux/FM Capture Volume",	/* Aux/FM */
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = pcm_capture_volume_info,
		.get    = aux_capture_volume_get,
		.put    = aux_capture_volume_put,
	},
	{
		.name   = "USB-Carkit Capture Volume",	/* USB Carkit Input */
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = mono_capture_volume_info,
		.get    = carkit_capture_volume_get,
		.put    = carkit_capture_volume_put,
	},
	/* Input Source Selection*/
	{
		.name   = "Capture Source",	/* Input Source */
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = snd_capture_source_info,
		.get    = snd_capture_source_get,
		.put    = snd_capture_source_put,
	},
	/* Output Source Selection */
	{
		.name   = "Playback Source",	/* Output Source */
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index  = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info   = snd_playback_source_info,
		.get    = snd_playback_source_get,
		.put    = snd_playback_source_put,
	},
};

#ifdef CONFIG_PM
void snd_omap_suspend_mixer(void)
{
}

void snd_omap_resume_mixer(void)
{
}
#endif

void snd_omap_init_mixer(void)
{
	/* Nothing to do for 3430 platform */
}
EXPORT_SYMBOL(snd_omap_init_mixer);

int snd_omap_mixer(struct snd_card_omap_codec *chip)
{
	struct snd_card *card;
	int i = 0;
	int err = 0;

	card = chip->card;

	strcpy(card->mixername, MIXER_NAME);


	for (i = 0; i < ARRAY_SIZE(twl4030_control); i++) {
		err = snd_ctl_add(card,
				snd_ctl_new1(&twl4030_control[i], card));
		if (err < 0)
			return err;
	}
	return 0;
}
EXPORT_SYMBOL(snd_omap_mixer);

int snd_omap_mixer_shutdown(struct snd_card_omap_codec *chip)
{
	return 0;
}
EXPORT_SYMBOL(snd_omap_mixer_shutdown);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Codec audio driver for the TI TWL4030 codec.");
MODULE_LICENSE("GPL");
