/*
 * OMAP2/3 System Control Module register access
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Copyright (C) 2007 Nokia Corporation
 *
 * Written by Paul Walmsley
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#undef DEBUG

#include <linux/kernel.h>
#include <linux/io.h>

#include <asm/arch/common.h>
#include <asm/arch/control.h>
#include <asm/arch/sdrc.h>
#include "cm-regbits-34xx.h"
#include "prm-regbits-34xx.h"
#include "cm.h"
#include "prm.h"
#include "clock34xx.h"
#include "sdrc.h"

static void __iomem *omap2_ctrl_base;

#define OMAP_CTRL_REGADDR(reg)		(omap2_ctrl_base + (reg))

void __init omap2_set_globals_control(struct omap_globals *omap2_globals)
{
	omap2_ctrl_base = omap2_globals->ctrl;
}

void __iomem *omap_ctrl_base_get(void)
{
	return omap2_ctrl_base;
}

u8 omap_ctrl_readb(u16 offset)
{
	return __raw_readb(OMAP_CTRL_REGADDR(offset));
}

u16 omap_ctrl_readw(u16 offset)
{
	return __raw_readw(OMAP_CTRL_REGADDR(offset));
}

u32 omap_ctrl_readl(u16 offset)
{
	return __raw_readl(OMAP_CTRL_REGADDR(offset));
}

void omap_ctrl_writeb(u8 val, u16 offset)
{
	__raw_writeb(val, OMAP_CTRL_REGADDR(offset));
}

void omap_ctrl_writew(u16 val, u16 offset)
{
	__raw_writew(val, OMAP_CTRL_REGADDR(offset));
}

void omap_ctrl_writel(u32 val, u16 offset)
{
	__raw_writel(val, OMAP_CTRL_REGADDR(offset));
}

#ifdef CONFIG_ARCH_OMAP3

#define OMAP3430_PRM_RSTST \
		OMAP34XX_PRM_REGADDR(OMAP3430_GR_MOD, RM_RSTST)
/* Clears the scratchpad contents in case of cold boot-
 called during bootup*/
void omap3_clear_scratchpad_contents(void)
{
	u32 max_offset = OMAP343X_SCRATHPAD_ROM_OFFSET;
	u32 offset = 0;
	u32 v;
	u32 v_addr = io_p2v(OMAP343X_SCRATCHPAD_ROM);
	if (__raw_readl(OMAP3430_PRM_RSTST) & 0x1) {
		for ( ; offset <= max_offset; offset += 0x4)
			__raw_writel(0x0, (v_addr + offset));
		v = __raw_readl(OMAP3430_PRM_RSTST);
		v |= 0x1;
		__raw_writel(v, OMAP3430_PRM_RSTST);
	}
}

/* Populate the scratchpad structure with restore structure */
void omap3_save_scratchpad_contents(void)
{
	u32 *scratchpad_address;
	u32 *sdram_context_address;
	struct omap3_scratchpad scratchpad_contents;
	struct omap3_scratchpad_prcm_block prcm_block_contents;
	struct omap3_scratchpad_sdrc_block sdrc_block_contents;

	/* Populate the Scratchpad contents */
	scratchpad_contents.boot_config_ptr = 0x0;
	scratchpad_contents.public_restore_ptr =
			(u32)((u32 *)virt_to_phys(get_restore_pointer()));
	scratchpad_contents.secure_ram_restore_ptr = 0x0;
	scratchpad_contents.sdrc_module_semaphore = 0x0;
	scratchpad_contents.prcm_block_offset = 0x2C;
	scratchpad_contents.sdrc_block_offset = 0x64;

	/* Populate the PRCM block contents */
	prcm_block_contents.prm_clksrc_ctrl =
			__raw_readl(OMAP3430_PRM_CLKSRC_CTRL);
	prcm_block_contents.prm_clksel =
			__raw_readl(OMAP3430_PRM_CLKSEL);
	prcm_block_contents.cm_clksel_core =
			cm_read_mod_reg(CORE_MOD, CM_CLKSEL);
	prcm_block_contents.cm_clksel_wkup =
			cm_read_mod_reg(WKUP_MOD, CM_CLKSEL);
	prcm_block_contents.cm_clken_pll =
			cm_read_mod_reg(PLL_MOD, OMAP3430_CM_CLKEN_PLL);
	prcm_block_contents.cm_autoidle_pll =
			cm_read_mod_reg(PLL_MOD, OMAP3430_CM_AUTOIDLE_PLL);
	prcm_block_contents.cm_clksel1_pll =
			cm_read_mod_reg(PLL_MOD, OMAP3430_CM_CLKSEL1_PLL);
	prcm_block_contents.cm_clksel2_pll =
			cm_read_mod_reg(PLL_MOD, OMAP3430_CM_CLKSEL2_PLL);
	prcm_block_contents.cm_clksel3_pll =
			cm_read_mod_reg(PLL_MOD, OMAP3430_CM_CLKSEL3);
	prcm_block_contents.cm_clken_pll_mpu =
			cm_read_mod_reg(MPU_MOD, OMAP3430_CM_CLKEN_PLL);
	prcm_block_contents.cm_autoidle_pll_mpu =
			cm_read_mod_reg(MPU_MOD, OMAP3430_CM_AUTOIDLE_PLL);
	prcm_block_contents.cm_clksel1_pll_mpu =
			cm_read_mod_reg(MPU_MOD, OMAP3430_CM_CLKSEL1_PLL);
	prcm_block_contents.cm_clksel2_pll_mpu =
			cm_read_mod_reg(MPU_MOD, OMAP3430_CM_CLKSEL2_PLL);
	prcm_block_contents.prcm_block_size = 0x0;

	/* Populate the SDRC block contents */
	sdrc_block_contents.sdrc_sysconfig =
			(sdrc_read_reg(SDRC_SYSCONFIG) & 0xFFFF);
	sdrc_block_contents.sdrc_cs_cfg =
			(sdrc_read_reg(SDRC_CS_CFG) & 0xFFFF);
	sdrc_block_contents.sdrc_sharing =
			(sdrc_read_reg(SDRC_SHARING) & 0xFFFF);
	sdrc_block_contents.sdrc_err_type =
			(sdrc_read_reg(SDRC_ERR_TYPE) & 0xFFFF);
	sdrc_block_contents.sdrc_dll_a_ctrl = sdrc_read_reg(SDRC_DLLA_CTRL);
	sdrc_block_contents.sdrc_dll_b_ctrl = 0x0;
	sdrc_block_contents.sdrc_power = sdrc_read_reg(SDRC_POWER);
	sdrc_block_contents.sdrc_cs_0 = 0x0;
	sdrc_block_contents.sdrc_mcfg_0 = sdrc_read_reg(SDRC_MCFG_0);
	sdrc_block_contents.sdrc_mr_0 = (sdrc_read_reg(SDRC_MR_0) & 0xFFFF);
	sdrc_block_contents.sdrc_emr_1_0 = 0x0;
	sdrc_block_contents.sdrc_emr_2_0 = 0x0;
	sdrc_block_contents.sdrc_emr_3_0 = 0x0;
	sdrc_block_contents.sdrc_actim_ctrla_0 =
			sdrc_read_reg(SDRC_ACTIM_CTRL_A_0);
	sdrc_block_contents.sdrc_actim_ctrlb_0 =
			sdrc_read_reg(SDRC_ACTIM_CTRL_B_0);
	sdrc_block_contents.sdrc_rfr_ctrl_0 =
			sdrc_read_reg(SDRC_RFR_CTRL_0);
	sdrc_block_contents.sdrc_cs_1 = 0x0;
	sdrc_block_contents.sdrc_mcfg_1 = sdrc_read_reg(SDRC_MCFG_1);
	sdrc_block_contents.sdrc_mr_1 = sdrc_read_reg(SDRC_MR_1) & 0xFFFF;
	sdrc_block_contents.sdrc_emr_1_1 = 0x0;
	sdrc_block_contents.sdrc_emr_2_1 = 0x0;
	sdrc_block_contents.sdrc_emr_3_1 = 0x0;
	sdrc_block_contents.sdrc_actim_ctrla_1 =
			sdrc_read_reg(SDRC_ACTIM_CTRL_A_1);
	sdrc_block_contents.sdrc_actim_ctrlb_1 =
			sdrc_read_reg(SDRC_ACTIM_CTRL_B_1);
	sdrc_block_contents.sdrc_rfr_ctrl_1 =
			sdrc_read_reg(SDRC_RFR_CTRL_1);
	sdrc_block_contents.sdrc_dcdl_1_ctrl = 0x0;
	sdrc_block_contents.sdrc_dcdl_2_ctrl = 0x0;
	sdrc_block_contents.sdrc_flags = 0x0;
	sdrc_block_contents.sdrc_block_size = 0x0;
	sdrc_block_contents.sdrc_context_addr =
			(u32)((u32 *)io_v2p(context_mem));

	/* Copy all the contents to the scratchpad location*/
	scratchpad_address = (u32 *)io_p2v(OMAP343X_SCRATCHPAD);
	memcpy(scratchpad_address, &scratchpad_contents,
		 sizeof(scratchpad_contents));
	memcpy(scratchpad_address + scratchpad_contents.prcm_block_offset/4,
		 &prcm_block_contents, sizeof(prcm_block_contents));
	memcpy(scratchpad_address + scratchpad_contents.sdrc_block_offset/4,
		 &sdrc_block_contents, sizeof(sdrc_block_contents));
}

#endif /* CONFIG_ARCH_OMAP3 */
