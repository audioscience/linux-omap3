#ifndef ARCH_ARM_MACH_OMAP2_CPUIDLE_34XX
#define ARCH_ARM_MACH_OMAP2_CPUIDLE_34XX

/*
 * OMAP2/3 powerdomain control
 *
 * Copyright (C) 2007-8 Texas Instruments, Inc.
 * Copyright (C) 2007-8 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

extern u32 *scratchpad_restore_addr;
extern u32 restore_pointer_address;
extern void omap_uart_save_ctx(int unum);
extern void omap_uart_restore_ctx(int unum);
extern void omap3_save_neon_context(void);
extern void omap3_restore_neon_context(void);
extern void omap3_save_per_context(void);
extern void omap3_restore_per_context(void);
extern void omap3_push_sram_functions(void);
extern void memory_logic_res_seting(void);
extern void omap3_restore_core_settings(void);
extern void enable_smartreflex(int srid);
extern void disable_smartreflex(int srid);
int perdomain_timer_pending(void);
int coredomain_timer_pending(void);
void context_restore_update(unsigned long domain_id);

extern unsigned long awake_time_end;

struct omap3_processor_cx {
	u8 valid;
	u8 type;
	u32 sleep_latency;
	u32 wakeup_latency;
	u32 mpu_state;
	u32 core_state;
	u32 threshold;
	u32 flags;
};
extern void serial8250_resume_devices(void);
extern int are_driver8250_uarts_active(int *);
extern int enable_off;
extern int enable_debug;
extern int lpr_enabled;
extern void omap2_gp_timer_program_next_event(unsigned long cycles);
extern void omap_sram_idle(void);
extern unsigned long omap_32k_sync_timer_read(void);
extern struct system_power_state target_state;

#define UART_TIME_OUT 6000 /* ms before cutting clock */

void omap_init_power_states(void);
int omap3_idle_init(void);

#define CORE_FCLK_MASK  0x3FF9E29 /* Mask of all functional clocks*/
					/*in core except UART*/
#define CORE3_FCLK_MASK 0x7 /* Mask of all functional clocks*/
#define USBHOST_FCLK_MASK       0x3 /* Mask of all functional clocks in USB*/
#define SGX_FCLK_MASK   0x2 /*Mask of all functional clocks in SGX*/

#define DSS_FCLK_MASK   0x7 /*Mask of all functional clocks in DSS*/
#define CAM_FCLK_MASK   0x1 /*Mask of all functional clocks in CAM*/
#define PER_FCLK_MASK   0x17FF /*Mask of all functional clocks in PER */
				/*except UART and GPIO*/
#define CORE1_ICLK_VALID        0x3FFFFFF9 /*Ignore SDRC_ICLK*/
#define CORE2_ICLK_VALID        0x1F
#define CORE3_ICLK_VALID        0x4
#define USBHOST_ICLK_VALID      0x1
#define SGX_ICLK_VALID  0x1

#define DSS_ICLK_VALID  0x1
#define CAM_ICLK_VALID  0x1
#define PER_ICLK_VALID 0x1
#define WKUP_ICLK_VALID 0x3E /* Ignore GPT1 ICLK as it is hanlded explicitly*/

#endif
