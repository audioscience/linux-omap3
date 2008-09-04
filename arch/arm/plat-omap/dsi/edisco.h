/*
 * arch/arm/plat-omap/edisco.h
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

/*==== DECLARATION CONTROL =================================================*/
#ifndef OMAP_EDISCO_H
#define OMAP_EDISCO_H

/*==== DEFINES ============================================================*/
#define GPIO_NUM_EDISCO_1P2V	5 /* GPIO5 */

/* Prototype Functions */

void get_edisco_platform_config(T_EDISCO_DIS *edisco_dis);
void get_edisco_vm_timing(T_VM_TIMING *video_mode_timing, U16 image_size);
S32 enable_edisco_power(void);
S32 disable_edisco_power(void);

#endif /* OMAP_EDISCO_H */
