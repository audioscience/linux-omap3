/*
 * linux/arch/arm/mach-omap2/pm_34xx.c
 *
 * OMAP3 Power Management Routines
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Karthik Dasu <karthik-dp@ti.com>
 *
 * Copyright (C) 2006 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * Based on pm.c for omap1
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <linux/io.h>
#include <linux/irq.h>
#include <asm/atomic.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>
#include <asm/mach-types.h>
#include <linux/mm.h>
#include <asm/mmu.h>
#include <asm/tlbflush.h>

#include <asm/arch/irqs.h>
#include <asm/arch/clock.h>
#include <asm/arch/sram.h>
#include <asm/arch/pm.h>
#include <linux/tick.h>
#include <asm/arch/resource.h>
#include <asm/arch/prcm_34xx.h>
#ifdef CONFIG_OMAP34XX_OFFMODE
#include <asm/arch/io.h>
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

#include "prcm-regs.h"
#include "ti-compat.h"
#include "pm_34xx.h"

/* #define DEBUG_PM_34XX 1 */
#ifdef DEBUG_PM_34XX
#  define DPRINTK(fmt, args...) printk(KERN_ERR "%s: " fmt, __FUNCTION__ , \
								## args)
#else
#  define DPRINTK(fmt, args...)
#endif

int enable_off = 1;

#define IOPAD_WKUP 1

unsigned long awake_time_end;

static void restore_control_register(u32 val)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c1, c0, 0" : : "r" (val));
}

/* Function to restore the table entry that was modified for enabling MMU*/
static void restore_table_entry(void)
{
	u32 *scratchpad_address;
	u32 previous_value, control_reg_value;
	u32 *address;
	/* Get virtual address of SCRATCHPAD */
	scratchpad_address = (u32 *) io_p2v(SCRATCHPAD);
	/* Get address of entry that was modified */
	address = (u32 *) *(scratchpad_address + TABLE_ADDRESS_OFFSET);
	/* Get the previous value which needs to be restored */
	previous_value = *(scratchpad_address + TABLE_VALUE_OFFSET);
	/* Convert address to virtual address */
	address = __va(address);
	/* Restore table entry */
	*address = previous_value;
	/* Flush TLB */
	flush_tlb_all();
	control_reg_value = *(scratchpad_address + CONTROL_REG_VALUE_OFFSET);
	/* Restore control register*/
	/* This will enable caches and prediction */
	restore_control_register(control_reg_value);
}

static void (*_omap_sram_idle)(u32 *addr, int save_state);

void omap_sram_idle(void)
{
	/* Variable to tell what needs to be saved and restored
	 * in omap_sram_idle*/
	/* save_state = 0 => Nothing to save and restored */
	/* save_state = 1 => Only L1 and logic lost */
	/* save_state = 2 => Only L2 lost */
	/* save_state = 3 => L1, L2 and logic lost */
	int save_state = 0;
	if (!_omap_sram_idle)
		return;
	switch (target_state.mpu_state) {
	case PRCM_MPU_ACTIVE:
	case PRCM_MPU_INACTIVE:
	case PRCM_MPU_CSWR_L2RET:
		/* No need to save context */
		save_state = 0;
		break;
	case PRCM_MPU_OSWR_L2RET:
		/* L1 and Logic lost */
		save_state = 1;
		break;
	case PRCM_MPU_CSWR_L2OFF:
		/* Only L2 lost */
		save_state = 2;
		break;
	case PRCM_MPU_OSWR_L2OFF:
	case PRCM_MPU_OFF:
		/* L1, L2 and logic lost */
		save_state = 3;
		break;
	default:
		/* Invalid state */
		printk(KERN_ERR "Invalid mpu state in sram_idle\n");
		return;
	}
	_omap_sram_idle(context_mem, save_state);
	/* Restore table entry modified during MMU restoration */
	if ((PM_PREPWSTST_MPU & 0x3) == 0x0)
		restore_table_entry();
}

static int omap3_pm_prepare(void)
{
	int error = 0;
	/* We cannot sleep in idle until we have resumed */
	saved_idle = pm_idle;
	pm_idle = NULL;
	return error;
}

/* Save and retore global configuration in NEON domain */
void omap3_save_neon_context(void)
{
	/*Nothing to do here */
	return;
}

void omap3_restore_neon_context(void)
{
#ifdef CONFIG_VFP
	/* TODO this function is now non-static
	vfp_enable();*/
#endif
}

/* Save and restore global configuration in PER domain */
void omap3_save_per_context(void)
{
	/* Only GPIO save is done here */
	omap_gpio_save();
}

void omap3_restore_per_context(void)
{
	/* Only GPIO restore is done here */
	omap_gpio_restore();
}

void omap3_push_sram_functions()
{
	_omap_sram_idle = omap_sram_push(omap34xx_cpu_suspend,
				omap34xx_cpu_suspend_sz);
}

/* Configuration that is OS specific is done here */
void omap3_restore_core_settings(void)
{
	prcm_lock_iva_dpll(current_vdd1_opp);
	restore_sram_functions();
	omap3_push_sram_functions();
}

void memory_logic_res_seting(void)
{
	res1_level = resource_get_level(memret1);
	res2_level = resource_get_level(memret2);
	res3_level = resource_get_level(logret1);
	if (res3_level == LOGIC_RET) {
		if ((res1_level == MEMORY_RET) && (res2_level == MEMORY_RET))
			target_state.core_state = PRCM_CORE_CSWR_MEMRET;
		else if (res1_level == MEMORY_OFF && res2_level == MEMORY_RET)
			target_state.core_state = PRCM_CORE_CSWR_MEM1OFF;
		else if (res1_level == MEMORY_RET &&  res2_level == MEMORY_OFF)
			target_state.core_state = PRCM_CORE_CSWR_MEM2OFF;
		else
			target_state.core_state = PRCM_CORE_CSWR_MEMOFF;
	} else if (res3_level == LOGIC_OFF) {
		if ((res1_level && res2_level) == MEMORY_RET)
			target_state.core_state = PRCM_CORE_OSWR_MEMRET;
		else if (res1_level == MEMORY_OFF && res2_level == MEMORY_RET)
			target_state.core_state = PRCM_CORE_OSWR_MEM1OFF;
		else if (res1_level == MEMORY_RET &&  res2_level == MEMORY_OFF)
			target_state.core_state = PRCM_CORE_OSWR_MEM2OFF;
		else
			target_state.core_state = PRCM_CORE_OSWR_MEMOFF;
	} else
		DPRINTK("Current State not supported in Retention");

}

static int omap3_pm_suspend(void)
{
	int ret;
	u32 prcm_state, prepwstst;

	local_irq_disable();
	local_fiq_disable();

	PM_PREPWSTST_MPU = 0xFF;
	PM_PREPWSTST_CORE = 0xFF;
	PM_PREPWSTST_NEON = 0xFF;
	PM_PREPWSTST_PER = 0xFF;
#ifdef CONFIG_CORE_OFF
	if (enable_off)
		omap_uart_save_ctx();
#endif
#ifdef CONFIG_MPU_OFF
	/* On ES 2.0, if scrathpad is populated with valid
	* pointer, warm reset does not work
	* So populate scrathpad restore address only in
	* cpuidle and suspend calls
	*/
	*(scratchpad_restore_addr) = restore_pointer_address;
#endif

#ifdef CONFIG_OMAP34XX_OFFMODE
	context_restore_update(DOM_PER);
	context_restore_update(DOM_CORE1);
	prcm_state = PRCM_OFF;
#else
	prcm_state = PRCM_RET;
#endif

	target_state.iva2_state = prcm_state;
	target_state.gfx_state = prcm_state;
	target_state.dss_state = prcm_state;
	target_state.cam_state = prcm_state;
	target_state.per_state = prcm_state;
	target_state.usbhost_state = prcm_state;
	target_state.neon_state = prcm_state;

#ifdef CONFIG_MPU_OFF
	if (enable_off)
		target_state.mpu_state = PRCM_MPU_OFF;
	else
		target_state.mpu_state = PRCM_MPU_CSWR_L2RET;
#else
	target_state.mpu_state = PRCM_MPU_CSWR_L2RET;
#endif

	if (target_state.neon_state == PRCM_OFF)
		omap3_save_neon_context();

	if (target_state.per_state == PRCM_OFF)
		omap3_save_per_context();

#ifdef CONFIG_CORE_OFF
	if (enable_off)
		target_state.core_state = PRCM_CORE_OFF;
	else
		target_state.core_state = PRCM_CORE_CSWR_MEMRET;
#else
	target_state.core_state = PRCM_CORE_CSWR_MEMRET;
#endif
	if (target_state.core_state == PRCM_CORE_CSWR_MEMRET)
		memory_logic_res_seting();

	if (target_state.core_state >=  PRCM_CORE_OSWR_MEMRET) {
		prcm_save_core_context(target_state.core_state);
		omap_uart_save_ctx();
	}

	ret = prcm_set_chip_power_mode(&target_state, PRCM_WAKEUP_T2_KEYPAD |
	PRCM_WAKEUP_TOUCHSCREEN | PRCM_WAKEUP_UART);

#ifdef CONFIG_MPU_OFF
	*(scratchpad_restore_addr) = 0;
#endif
	if (target_state.neon_state == PRCM_OFF)
		omap3_restore_neon_context();
	PM_PREPWSTST_NEON = 0xFF;

	if (target_state.per_state == PRCM_OFF)
		omap3_restore_per_context();
	PM_PREPWSTST_PER = 0xFF;
#ifdef CONFIG_CORE_OFF
	if (enable_off) {
		omap3_restore_core_settings();
		omap_uart_restore_ctx();
	}
#else
	if (target_state.core_state >= PRCM_CORE_OSWR_MEMRET) {
#ifdef CONFIG_OMAP34XX_OFFMODE
	context_restore_update(DOM_CORE1);
#endif
		prcm_restore_core_context(target_state.core_state);
		omap_uart_restore_ctx();
	}

	if ((target_state.core_state > PRCM_CORE_CSWR_MEMRET) &&
			(target_state.core_state != PRCM_CORE_OSWR_MEMRET)) {
			restore_sram_functions();
			omap3_push_sram_functions();
		}
#endif
	local_fiq_enable();
	local_irq_enable();

	printk(KERN_INFO "\nSuspend/Resume Result: %s!! \n",
		(ret == PRCM_PASS) ? "SUCCESS" : "FAIL");

	prcm_state = target_state.mpu_state;
	prepwstst = PM_PREPWSTST_MPU & 0x3;
	printk(KERN_INFO "MPU state : TARGET-%4s PREPWST[0x%x]-%4s\n",
		(prcm_state == PRCM_MPU_OFF) ? "OFF" :
		(prcm_state >= PRCM_MPU_CSWR_L2RET) ? "RET" : "ON",
		PM_PREPWSTST_MPU,
		(prepwstst == 0x0) ? "OFF" :
		(prepwstst == 0x1) ? "RET" : "ON");

	prcm_state = target_state.core_state;
	prepwstst = PM_PREPWSTST_CORE & 0x3;
	printk(KERN_INFO "CORE state: TARGET-%4s PREPWST[0x%x]-%4s\n",
		(prcm_state == PRCM_CORE_OFF) ? "OFF" :
		(prcm_state >= PRCM_CORE_CSWR_MEMRET) ? "RET":"ON",
		PM_PREPWSTST_CORE,
		(prepwstst == 0x0) ? "OFF" :
		(prepwstst == 0x1) ? "RET" : "ON");

	PM_PREPWSTST_CORE = PM_PREPWSTST_CORE;
	PM_PREPWSTST_MPU = PM_PREPWSTST_MPU;
	return 0;
}

static int omap3_pm_enter(suspend_state_t state)
{
	int ret = 0;

	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = omap3_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int omap3_pm_valid(suspend_state_t state)
{
	int ret = 0;

	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = 1;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void omap3_pm_finish(void)
{
	pm_idle = saved_idle;
	return;
}

static struct platform_suspend_ops omap_pm_ops = {
	.valid		= omap3_pm_valid,
	.prepare	= omap3_pm_prepare,
	.enter		= omap3_pm_enter,
	.finish		= omap3_pm_finish,
};

/* PRCM Interrupt Handler */
irqreturn_t prcm_interrupt_handler (int irq, void *dev_id)
{
	u32 wkst_wkup = PM_WKST_WKUP;
	u32 wkst1_core = PM_WKST1_CORE;
	u32 wkst3_core = PM_WKST3_CORE;
	u32 wkst_usbhost = PM_WKST_USBHOST;
	u32 wkst_per = PM_WKST_PER;
	u32 errst_vc = PRM_VC_TIMEOUTERR_ST | PRM_VC_RAERR_ST |
							PRM_VC_SAERR_EN;
	u32 fclk = 0;
	u32 iclk = 0;

	if (PM_WKST_WKUP) {
#ifdef IOPAD_WKUP
		/* Resetting UART1 inactivity timeout, during IO_PAD wakeup */
		if (wkst_wkup & 0x100)
			awake_time_end = jiffies +
					 msecs_to_jiffies(UART_TIME_OUT);

#endif /* #ifdef IOPAD_WKUP */

		iclk = CM_ICLKEN_WKUP;
		fclk = CM_FCLKEN_WKUP;
		CM_ICLKEN_WKUP |= wkst_wkup;
		CM_FCLKEN_WKUP |= wkst_wkup;
		PM_WKST_WKUP = wkst_wkup;
		while (PM_WKST_WKUP);
		CM_ICLKEN_WKUP = iclk;
		CM_FCLKEN_WKUP = fclk;
	}
	if (PM_WKST1_CORE) {
		iclk = CM_ICLKEN1_CORE;
		fclk = CM_FCLKEN1_CORE;
		CM_ICLKEN1_CORE |= wkst1_core;
		CM_FCLKEN1_CORE |= wkst1_core;
		PM_WKST1_CORE = wkst1_core;
		while (PM_WKST1_CORE);
		CM_ICLKEN1_CORE = iclk;
		CM_FCLKEN1_CORE = fclk;
	}
	if (PM_WKST3_CORE) {
		iclk = CM_ICLKEN3_CORE;
		fclk = CM_FCLKEN3_CORE;
		CM_ICLKEN3_CORE |= wkst3_core;
		CM_FCLKEN3_CORE |= wkst3_core;
		PM_WKST3_CORE = wkst3_core;
		while (PM_WKST3_CORE);
		CM_ICLKEN3_CORE = iclk;
		CM_FCLKEN3_CORE = fclk;
	}
	if (PM_WKST_USBHOST) {

		iclk = CM_ICLKEN_USBHOST;
		fclk = CM_FCLKEN_USBHOST;
		CM_ICLKEN_USBHOST |= wkst_usbhost;
		CM_FCLKEN_USBHOST |= wkst_usbhost;
		PM_WKST_USBHOST = wkst_usbhost;
		while (PM_WKST_USBHOST);
		CM_ICLKEN_USBHOST = iclk;
		CM_FCLKEN_USBHOST = fclk;
	}
	if (PM_WKST_PER) {
		iclk = CM_ICLKEN_PER;
		fclk = CM_FCLKEN_PER;
		CM_ICLKEN_PER |= wkst_per;
		CM_FCLKEN_PER |= wkst_per;
		PM_WKST_PER = wkst_per;
		while (PM_WKST_PER);
		CM_ICLKEN_PER = iclk;
		CM_FCLKEN_PER = fclk;
	}

	if (!(wkst_wkup | wkst1_core | wkst3_core | wkst_usbhost | wkst_per)) {
		if (!(PRM_IRQSTATUS_MPU & errst_vc)) {
			printk(KERN_ERR "%x,%x,%x,%x\n", PRM_IRQSTATUS_MPU,
					wkst_wkup, wkst1_core, wkst_per);
			printk(KERN_ERR "Spurious PRCM interrupt\n");
		}
	}

#ifdef CONFIG_OMAP_VOLT_SR_BYPASS
	if (PRM_IRQSTATUS_MPU & PRM_VC_TIMEOUTERR_ST)
		printk(KERN_ERR "PRCM : Voltage Controller timeout\n");
	if (PRM_IRQSTATUS_MPU & PRM_VC_RAERR_ST)
		printk(KERN_ERR "PRCM : Voltage Controller register address"
							"acknowledge error\n");
	if (PRM_IRQSTATUS_MPU & PRM_VC_SAERR_ST)
		printk(KERN_ERR "PRCM : Voltage Controller slave address"
							"acknowledge error\n");
#endif /* #ifdef CONFIG_OMAP_VOLT_SR_BYPASS */

	if (PRM_IRQSTATUS_MPU) {
		PRM_IRQSTATUS_MPU |= 0x3;
		while (PRM_IRQSTATUS_MPU);
	}
	return IRQ_HANDLED;
}

int __init omap3_pm_init(void)
{
	int ret;
	printk(KERN_ERR "Power Management for TI OMAP.\n");

	omap3_push_sram_functions();
	suspend_set_ops(&omap_pm_ops);

	/* In case of cold boot, clear scratchpad */
	if (RM_RSTST_CORE & 0x1)
		clear_scratchpad_contents();
#ifdef CONFIG_MPU_OFF
	save_scratchpad_contents();
#endif
	prcm_init();
	if (!is_device_type_gp()) {
		sdram_mem = kmalloc(sizeof(struct sram_mem), GFP_KERNEL);
		if (!sdram_mem)
			printk(KERN_ERR "Memory allocation failed when"
				"allocating for secure sram context");
	}
	memret1 = (struct res_handle *)resource_get("corememresret1",
							"core_mem1ret");
	memret2 = (struct res_handle *)resource_get("corememresret2",
							"core_mem2ret");
	logret1 = (struct res_handle *)resource_get("corelogicret",
							"core_logicret");

	ret = request_irq(PRCM_MPU_IRQ, (irq_handler_t)prcm_interrupt_handler,
				IRQF_DISABLED, "prcm", NULL);
	if (ret) {
		printk(KERN_ERR "request_irq failed to register for 0x%x\n",
			PRCM_MPU_IRQ);
		return -1;
	}

	/* Adjust the system OPP based during bootup using
	* KConfig option
	*/
	if (p_vdd1_clk == NULL) {
		p_vdd1_clk = clk_get(NULL, "virt_vdd1_prcm_set");
		if (p_vdd1_clk == NULL) {
			printk(KERN_ERR "Unable to get the VDD1 clk\n");
			return -1;
		}
	}

	if (p_vdd2_clk == NULL) {
		p_vdd2_clk = clk_get(NULL, "virt_vdd2_prcm_set");
		if (p_vdd2_clk == NULL) {
			printk(KERN_ERR "Unable to get the VDD2 clk\n");
			return -1;
		}
	}

#ifdef CONFIG_MPU_OFF
	clear_scratchpad_contents();
	save_scratchpad_contents();
#endif
	PRM_IRQSTATUS_MPU = 0x3FFFFFD;
	PRM_IRQENABLE_MPU = 0x1;
#ifdef IOPAD_WKUP
	/* Enabling the IO_PAD PRCM interrupts */
	PRM_IRQENABLE_MPU |= 0x200;
#endif /* #ifdef IOPAD_WKUP */

#ifdef CONFIG_OMAP_VOLT_SR_BYPASS
	/* Enabling the VOLTAGE CONTROLLER PRCM interrupts */
	PRM_IRQENABLE_MPU |= PRM_VC_TIMEOUTERR_EN | PRM_VC_RAERR_EN|
				PRM_VC_SAERR_EN;
#endif /* #ifdef CONFIG_OMAP_VOLT_SR_BYPASS */
	/* omap3_pm_sysfs_init();*/
	return 0;
}

/* Clears the scratchpad contents in case of cold boot- called during bootup*/
void clear_scratchpad_contents(void)
{
	u32 max_offset = SCRATHPAD_ROM_OFFSET;
	u32 offset = 0;
	u32 v_addr = io_p2v(SCRATCHPAD_ROM);
	/* Check if it is a cold reboot */
	if (PRM_RSTST & 0x1) {
		for ( ; offset <= max_offset; offset += 0x4)
			__raw_writel(0x0, (v_addr + offset));
		PRM_RSTST |= 0x1;
	}
}

/* Populate the scratchpad structure with restore structure */
void save_scratchpad_contents(void)
{
	u32 *scratchpad_address;
	u32 *restore_address;
	u32 *sdram_context_address;
	/* Get virtual address of SCRATCHPAD */
	scratchpad_address = (u32 *) io_p2v(SCRATCHPAD);
	/* Get Restore pointer to jump to while waking up from OFF */
	restore_address = get_restore_pointer();
	/* Convert it to physical address */
	restore_address = (u32 *) io_v2p(restore_address);
	/* Get address where registers are saved in SDRAM */
	sdram_context_address = (u32 *) io_v2p(context_mem);
	/* Booting configuration pointer*/
	*(scratchpad_address++) = 0x0;
	/* Public restore pointer */
	/* On ES 2.0, if scrathpad is populated with valid
	* pointer, warm reset does not work
	* So populate scrathpad restore address only in
	* cpuidle and suspend calls
	*/
	scratchpad_restore_addr = scratchpad_address;
	restore_pointer_address = (u32) restore_address;
	*(scratchpad_address++) = restore_pointer_address;
	/* Secure ram restore pointer */
	if (is_device_type_gp())
		*(scratchpad_address++) = 0x0;
	else
		*(scratchpad_address++) = (u32) __pa(sdram_mem);
	/* SDRC Module semaphore */
	*(scratchpad_address++) = 0x0;
	/* PRCM Block Offset */
	*(scratchpad_address++) = 0x2C;
	/* SDRC Block Offset */
	*(scratchpad_address++) = 0x64;
	/* Empty */
	/* Offset 0x8*/
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = 0x0;
	/* Offset 0xC*/
	*(scratchpad_address++) = 0x0;
	/* Offset 0x10*/
	*(scratchpad_address++) = 0x0;
	/* Offset 0x14*/
	*(scratchpad_address++) = 0x0;
	/* Offset 0x18*/
	/* PRCM Block */
	*(scratchpad_address++) = PRM_CLKSRC_CTRL;
	*(scratchpad_address++) = PRM_CLKSEL;
	*(scratchpad_address++) = CM_CLKSEL_CORE;
	*(scratchpad_address++) = CM_CLKSEL_WKUP;
	*(scratchpad_address++) = CM_CLKEN_PLL;
	*(scratchpad_address++) = CM_AUTOIDLE_PLL;
	*(scratchpad_address++) = CM_CLKSEL1_PLL;
	*(scratchpad_address++) = CM_CLKSEL2_PLL;
	*(scratchpad_address++) = CM_CLKSEL3_PLL;
	*(scratchpad_address++) = CM_CLKEN_PLL_MPU;
	*(scratchpad_address++) = CM_AUTOIDLE_PLL_MPU;
	*(scratchpad_address++) = CM_CLKSEL1_PLL_MPU;
	*(scratchpad_address++) = CM_CLKSEL2_PLL_MPU;
	*(scratchpad_address++) = 0x0;
	/* SDRC Block */
	*(scratchpad_address++) = ((SDRC_CS_CFG & 0xFFFF) << 16) |
					(SDRC_SYS_CONFIG & 0xFFFF);
	*(scratchpad_address++) = ((SDRC_ERR_TYPE & 0xFFFF) << 16) |
					(SDRC_SHARING & 0xFFFF);
	*(scratchpad_address++) = SDRC_DLL_A_CTRL;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = SDRC_PWR;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = SDRC_MCFG_0;
	*(scratchpad_address++) = SDRC_MR0 & 0xFFFF;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = SDRC_ACTIM_CTRL_A_0;
	*(scratchpad_address++) = SDRC_ACTIM_CTRL_B_0;
	*(scratchpad_address++) = SDRC_RFR_CTRL_0;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = SDRC_MCFG_1;
	*(scratchpad_address++) = SDRC_MR1 & 0xFFFF;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = SDRC_ACTIM_CTRL_A_1;
	*(scratchpad_address++) = SDRC_ACTIM_CTRL_B_1;
	*(scratchpad_address++) = SDRC_RFR_CTRL_1;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = (u32) sdram_context_address;
}

__initcall(omap3_pm_init);
