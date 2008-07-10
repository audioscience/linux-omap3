/*
 * linux/arch/arm/mach-omap2/pm_34xx.h
 *
 * OMAP3 Power Management Header
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Karthik Dasu <karthik-dp@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define S600M   600000000
#define S550M   550000000
#define S500M   500000000
#define S250M   250000000
#define S125M   125000000
#define S19M    19200000
#define S120M   120000000
#define S477M   476000000
#define S381M   381000000
#define S191M   190000000
#define S96M    96000000
#define S166M   166000000
#define S83M    83000000

#define PRCM_MPU_IRQ    0xB /* MPU IRQ 11 */

/* COntext memory */
u32 context_mem[128];

#define UART_TIME_OUT 6000 /* ms before cutting clock */

#define SCRATCHPAD_ROM  0x48002860
#define SCRATCHPAD      0x48002910
#define SCRATHPAD_ROM_OFFSET    0x19C
#define TABLE_ADDRESS_OFFSET    0x31
#define TABLE_VALUE_OFFSET      0x30
#define CONTROL_REG_VALUE_OFFSET        0x32

struct system_power_state target_state;

u32 *scratchpad_restore_addr;
u32 restore_pointer_address;

static void (*saved_idle)(void);
static void (*_omap_sram_idle)(u32 *addr, int save_state);
static void (*_omap_save_secure_sram)(u32 *addr);

extern unsigned long awake_time_end;

struct clk *p_vdd1_clk;
struct clk *p_vdd2_clk;

struct sram_mem {
	u32 i[32000];
};
struct sram_mem *sdram_mem;

struct res_handle  *memret1;
struct res_handle  *memret2;
struct res_handle  *logret1;
int res1_level = -1, res2_level = -1, res3_level = -1;

extern void omap_uart_save_ctx(void);
extern void omap_uart_restore_ctx(void);
extern void set_blank_interval(int fb_timeout_val);
extern void omap_gpio_save(void);
extern void omap_gpio_restore(void);
extern int omap3_pm_sysfs_init(void);
extern void enable_smartreflex(int srid);
extern void disable_smartreflex(int srid);
extern int enable_off;
