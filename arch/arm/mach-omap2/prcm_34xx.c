/*
 * linux/arch/arm/mach-omap2/prcm_34xx.c
 *
 * OMAP 34xx Power Reset and Clock Management (PRCM) functions
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Karthik Dasu/Rajendra Nayak/Pavan Chinnabhandar
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
*/

#include <linux/module.h>
#include <linux/init.h>
#include <asm/arch/prcm_34xx.h>
#include <asm/io.h>
#include <asm/arch/gpmc.h>

#include "prcm-regs.h"
#include "ti-compat.h"

const u32 MAXLOOPCNT = 50;
const u32 MAXRETRIES = 5;

static int padconf_saved;

static unsigned int prcm_sleep_save[PRCM_SLEEP_SAVE_SIZE];
static struct int_controller_context intc_context;
static struct control_module_context control_ctx;
void init_wakeupconfig(void);
void uart_padconf_control(void);
void setup_board_wakeup_source(u32);

#define IOPAD_WKUP 1

/* Using 32K sync timer to generate delays in usecs */
#define OMAP_TIMER32K_SYNC_CR (OMAP3430_32KSYNCT_BASE + 0x10)
#define OMAP_TIMER32K_SYNC_CR_READ   (omap_readl(OMAP_TIMER32K_SYNC_CR))
#define OMAP_MAX_32K_CR 0xFFFFFFFF

#ifndef CONFIG_PM
#define omap_sram_idle() \
	{	\
		__asm__ __volatile__ ("wfi");	\
	}
#endif

/* Table to store domain registers */
struct domain_registers dom_reg[PRCM_NUM_DOMAINS] = {
	{
		/* IVA2 domain */
		{
			{0x1,		(u32 *)&CM_FCLKEN_IVA2},
			{0,		0},
			{0x1,		(u32 *)&CM_IDLEST_IVA2},
			{0,		0},
			{0,		0},
			{0,		0},
			{0x3,		(u32 *)&CM_CLKSTCTRL_IVA2},
			{0x1,		(u32 *)&CM_CLKSTST_IVA2},
			{0xFF0F0F,	(u32 *)&PM_PWSTCTRL_IVA2},
			{0x100FF7,	(u32 *)&PM_PWSTST_IVA2},
			{0xFF7, 	(u32 *)&PM_PREPWSTST_IVA2},
			{0,		0},
			{0x3F0F,	(u32 *)&RM_RSTST_IVA2},
		}
	},
	{
		/* MPU domain */
		{
			{0,		0},
			{0,		0},
			{0x1,		(u32 *)&CM_IDLEST_MPU},
			{0,		0},
			{0,		0},
			{0,		0},
			{0x3,		(u32 *)&CM_CLKSTCTRL_MPU},
			{0x1,		(u32 *)&CM_CLKSTST_MPU},
			{0x3010F,	(u32 *)&PM_PWSTCTRL_MPU},
			{0x1000C7,	(u32 *)&PM_PWSTST_MPU},
			{0xC7,		(u32 *)&PM_PREPWSTST_MPU},
			{0,		0},
			{0x80F,		(u32 *)&RM_RSTST_MPU},
		}
	},
	{
		/* CORE1 */
		{
			{0x43FFFE01,	(u32 *)&CM_FCLKEN1_CORE},
			{0x7FFFFED3, 	(u32 *)&CM_ICLKEN1_CORE},
			{0x7FFFFFF7, 	(u32 *)&CM_IDLEST1_CORE},
			{0x7FFFFED1,	(u32 *)&CM_AUTOIDLE1_CORE},
			{0x433FFE10,	(u32 *)&PM_WKEN1_CORE},
			{0x433FFE10,	(u32 *)&PM_WKST1_CORE},
			{0x0F,		(u32 *)&CM_CLKSTCTRL_CORE},
			{0x3,		(u32 *)&CM_CLKSTST_CORE},
			{0xF031F,	(u32 *)&PM_PWSTCTRL_CORE},
			{0x1000F7,	(u32 *)&PM_PWSTST_CORE},
			{0xF7,		(u32 *)&PM_PREPWSTST_CORE},
			{0xC0,		(u32 *)&CM_CLKSEL_CORE},
			{0x7,		(u32 *)&RM_RSTST_CORE},
		}
	},
	{
		/* CORE2 */
		{
			{0,		0},
			{0x1F,		(u32 *)&CM_ICLKEN2_CORE},
			{0x1F,		(u32 *)&CM_IDLEST2_CORE},
			{0x1F,		(u32 *)&CM_AUTOIDLE2_CORE},
			{0,		0},
			{0,		0},
			{0,		0},
			{0,		0},
			{0,		0},
			{0,		0},
			{0,		0},
			{0,		0},
			{0,		0},
		}
	},
	{
		/* SGX */
		{
			{0x2,		(u32 *)&CM_FCLKEN_SGX},
			{0x1,		(u32 *)&CM_ICLKEN_SGX},
			{0x1,		(u32 *)&CM_IDLEST_SGX},
			{0,		0},
			{0,		0},
			{0,		0},
			{0x3,		(u32 *)&CM_CLKSTCTRL_SGX},
			{0x1,		(u32 *)&CM_CLKSTST_SGX},
			{0x30107,	(u32 *)&PM_PWSTCTRL_SGX},
			{0x100003,	(u32 *)&PM_PWSTST_SGX},
			{0x3,		(u32 *)&PM_PREPWSTST_SGX},
			{0,		0},
			{0xF,		(u32 *)&RM_RSTST_SGX},
		}
	},
	{
		/* WKUP */
		{
			{0x2E9,		(u32 *)&CM_FCLKEN_WKUP},
			{0x23F,		(u32 *)&CM_ICLKEN_WKUP},
			{0x2FF,		(u32 *)&CM_IDLEST_WKUP},
			{0x23F,		(u32 *)&CM_AUTOIDLE_WKUP},
			{0x3CB,		(u32 *)&PM_WKEN_WKUP},
			{0x3CB,		(u32 *)&PM_WKST_WKUP},
			{0,		0},
			{0,		0},
			{0,		0},
			{0,		0},
			{0,		0},
			{0x7F,		(u32 *)&CM_CLKSEL_WKUP},
			{0,		0},
		}
	},
	{
		/* DSS */
		{
			{0x7,		(u32 *)&CM_FCLKEN_DSS},
			{0x1,		(u32 *)&CM_ICLKEN_DSS},
			{0x3,		(u32 *)&CM_IDLEST_DSS},
			{0x1,		(u32 *)&CM_AUTOIDLE_DSS},
			{0x1,		(u32 *)&PM_WKEN_DSS},
			{0,		0},
			{0x3,		(u32 *)&CM_CLKSTCTRL_DSS},
			{0x1,		(u32 *)&CM_CLKSTST_DSS},
			{0x30107,	(u32 *)&PM_PWSTCTRL_DSS},
			{0x100003,	(u32 *)&PM_PWSTST_DSS},
			{0x3,		(u32 *)&PM_PREPWSTST_DSS},
			{0,		0},
			{0xF,		(u32 *)&RM_RSTST_DSS},
		}
	},
	{
		/* CAM */
		{
			{0x3,		(u32 *)&CM_FCLKEN_CAM},
			{0x1,		(u32 *)&CM_ICLKEN_CAM},
			{0x1,		(u32 *)&CM_IDLEST_CAM},
			{0x1,		(u32 *)&CM_AUTOIDLE_CAM},
			{0,		0},
			{0,		0},
			{0x3,		(u32 *)&CM_CLKSTCTRL_CAM},
			{0x1,		(u32 *)&CM_CLKSTST_CAM},
			{0x30107,	(u32 *)&PM_PWSTCTRL_CAM},
			{0x100003,	(u32 *)&PM_PWSTST_CAM},
			{0x3,		(u32 *)&PM_PREPWSTST_CAM},
			{0,		0},
			{0xF,		(u32 *)&RM_RSTST_CAM},
		}
	},
	{
		/* PER */
		{
			{0x3FFFF,	(u32 *)&CM_FCLKEN_PER},
			{0x3FFFF,	(u32 *)&CM_ICLKEN_PER},
			{0x3FFFF,	(u32 *)&CM_IDLEST_PER},
			{0x3FFFF,	(u32 *)&CM_AUTOIDLE_PER},
			{0x3EFFF,	(u32 *)&PM_WKEN_PER},
			{0x3EFFF,	(u32 *)&PM_WKST_PER},
			{0x3,		(u32 *)&CM_CLKSTCTRL_PER},
			{0x1,		(u32 *)&CM_CLKSTST_PER},
			{0x30107,	(u32 *)&PM_PWSTCTRL_PER},
			{0x100003,	(u32 *)&PM_PWSTST_PER},
			{0x3,		(u32 *)&PM_PREPWSTST_PER},
			{0xFF,		(u32 *)&CM_CLKSEL_PER},
			{0xF,		(u32 *)&RM_RSTST_PER},
		}
	},
	{
		/* EMU */
		{
			{0,		0},
			{0,		0},
			{0,		0},
			{0,		0},
			{0,		0},
			{0,		0},
			{0x3,		(u32 *)&CM_CLKSTCTRL_EMU},
			{0x1,		(u32 *)&CM_CLKSTST_EMU},
			{0,		0},
			{0x100003,	(u32 *)&PM_PWSTST_EMU},
			{0,		0},
			{0,		0},
			{0x7,		(u32 *)&RM_RSTST_EMU},
		}
	},
	{
		/* NEON */
		{
			{0,		0},
			{0,		0},
			{0x1,		(u32 *)&CM_IDLEST_NEON},
			{0,		0},
			{0,		0},
			{0,		0},
			{0x3,		(u32 *)&CM_CLKSTCTRL_NEON},
			{0,		0},
			{0x7,		(u32 *)&PM_PWSTCTRL_NEON},
			{0x100003,	(u32 *)&PM_PWSTST_NEON},
			{0x3,		(u32 *)&PM_PREPWSTST_NEON},
			{0,		0},
			{0xF,		(u32 *)&RM_RSTST_NEON},
		}
	},
	{
		/* CORE3 */
		{
			{0x7,		(u32 *)&CM_FCLKEN3_CORE},
			{0x4,		(u32 *)&CM_ICLKEN3_CORE},
			{0x5,		(u32 *)&CM_IDLEST3_CORE},
			{0x4,		(u32 *)&CM_AUTOIDLE3_CORE},
			{0x4,		(u32 *)&PM_WKEN3_CORE},
			{0x4,		(u32 *)&PM_WKST3_CORE},
			{0,		0},
			{0,		0},
			{0,		0},
			{0,		0},
			{0,		0},
			{0,		0},
			{0,		0},
		}
	},
	{
		/* USBHOST */
		{
			{0x3,		(u32 *)&CM_FCLKEN_USBHOST},
			{0x1,		(u32 *)&CM_ICLKEN_USBHOST},
			{0x3,		(u32 *)&CM_IDLEST_USBHOST},
			{0x1,		(u32 *)&CM_AUTOIDLE_USBHOST},
			{0x1,		(u32 *)&PM_WKEN_USBHOST},
			{0x1,		(u32 *)&PM_WKST_USBHOST},
			{0x3,		(u32 *)&CM_CLKSTCTRL_USBHOST},
			{0x1,		(u32 *)&CM_CLKSTST_USBHOST},
			{0x30117,	(u32 *)&PM_PWSTCTRL_USBHOST},
			{0x100003, 	(u32 *)&PM_PWSTST_USBHOST},
			{0x3,		(u32 *)&PM_PREPWSTST_USBHOST},
			{0,		0},
			{0xF,		(u32 *)&RM_RSTST_USBHOST},
		}
	},
};

/* Table to store DPLL registers */
struct dpll_registers dpll_reg[NO_OF_DPLL] = {
	{
		/* DPLL1_MPU */
		{
			{0xFFFFFF0F,	(u32 *)&CM_CLKEN_PLL_MPU},
			{0x7,		(u32 *)&CM_AUTOIDLE_PLL_MPU},
			{0xFFF800FF,	(u32 *)&CM_CLKSEL1_PLL_MPU},
			{0,		(u32 *)&CM_IDLEST_PLL_MPU},
			{0xFFFFFFE0,	(u32 *)&CM_CLKSEL2_PLL_MPU},
			{0xFFFFFFE0,	(u32 *)&CM_CLKSEL2_PLL_MPU},
			{0,		0},
			{0,		0},
			{0,		0},
			{0,		0},
		}
	},
	{
		/* DPLL2_IVA2 */
		{
			{0xFFFFFF0F,	(u32 *)&CM_CLKEN_PLL_IVA2},
			{0x7,		(u32 *)&CM_AUTOIDLE_PLL_IVA2},
			{0xFFF800FF,	(u32 *)&CM_CLKSEL1_PLL_IVA2},
			{0,		(u32 *)&CM_IDLEST_PLL_IVA2},
			{0xFFFFFFE0,	(u32 *)&CM_CLKSEL2_PLL_IVA2},
			{0xFFFFFFE0,	(u32 *)&CM_CLKSEL2_PLL_IVA2},
			{0,		0},
			{0,		0},
			{0,		0},
			{0,		0},
		}
	},
	{
		/* DPLL3_CORE */
		{
			{0xFFFFFF0F,	(u32 *)&CM_CLKEN_PLL},
			{0x7,		(u32 *)&CM_AUTOIDLE_PLL},
			{0xF800FFFF,	(u32 *)&CM_CLKSEL1_PLL},
			{0,		(u32 *)&CM_IDLEST_CKGEN},
			{0x07FFFFFF,	(u32 *)&CM_CLKSEL1_PLL},
			{0x07FFFFFF,	(u32 *)&CM_CLKSEL1_PLL},
			{0xFFE0FFFF,	(u32 *)&CM_CLKSEL1_EMU},
			{0,		0},
			{0,		0},
			{0,		0},
		}
	},
	{
		/* DPLL4_PER */
		{
			{0xFF0FFFFF,	(u32 *)&CM_CLKEN_PLL},
			{0x38,		(u32 *)&CM_AUTOIDLE_PLL},
			{0xFFF800FF,	(u32 *)&CM_CLKSEL2_PLL},
			{0,		(u32 *)&CM_IDLEST_CKGEN},
			{0xFFFFFFE0,	(u32 *)&CM_CLKSEL3_PLL},
			{0xFFFFFFE0,	(u32 *)&CM_CLKSEL3_PLL},
			{0xFFFFE0FF,	(u32 *)&CM_CLKSEL_DSS},
			{0xFFFFFFE0,	(u32 *)&CM_CLKSEL_DSS},
			{0xFFFFFFE0,	(u32 *)&CM_CLKSEL_CAM},
			{0xE0FFFFFF,	(u32 *)&CM_CLKSEL1_EMU},
		}
	},
	{
		/* DPLL5_PER2 */
		{
			{0xFFFFFF0F,	(u32 *)&CM_CLKEN2_PLL},
			{0x7,		(u32 *)&CM_AUTOIDLE2_PLL},
			{0xFFF800FF,	(u32 *)&CM_CLKSEL4_PLL},
			{0,		(u32 *)&CM_IDLEST2_CKGEN},
			{0xFFFFFFE0,	(u32 *)&CM_CLKSEL5_PLL},
			{0,		0},
			{0,		0},
			{0,		0},
			{0,		0},
			{0,		0},
		}
	}
};

/* Table to store clksel registers for various clocks */
struct reg_def clksel_reg[PRCM_NO_OF_CLKS] = {
	{0x3, (u32 *)&CM_CLKSEL_CORE},	/* L3_ICLK */
	{0xC, (u32 *)&CM_CLKSEL_CORE},	/* L4_ICLK */
	{0x6, (u32 *)&CM_CLKSEL_WKUP},	/* RM_ICLK */
	{0x00, NULL},
	{0x38, (u32 *)&CM_CLKOUT_CTRL},	/* SYS_CLOCKOUT2 */
	{0x7, (u32 *)&CM_CLKSEL_SGX},	/* SGX_L3_FCLK */
	{0xF00, (u32 *)&CM_CLKSEL_CORE},	/* SSI */
	{0x180000, (u32 *)&CM_CLKSEL1_PLL_MPU},	/* DPLL1_FCLK */
	{0x180000, (u32 *)&CM_CLKSEL1_PLL_IVA2},	/* DPLL2_FCLK */
	{0x78, (u32 *)&CM_CLKSEL_WKUP}, 	/* USIM_FCLK */

};

struct sysconf_reg sysc_reg_core1[] = {
	{PRCM_SSI,      (u32 *)&PRCM_SSI_SYSCONFIG},
	{PRCM_SDRC,     (u32 *)&PRCM_SDRC_SYSCONFIG},
	{PRCM_SDMA,     (u32 *)&PRCM_SDMA_SYSCONFIG},
	{PRCM_HSOTG,    (u32 *)&PRCM_HSOTG_SYSCONFIG},
	{PRCM_OMAP_CTRL, (u32 *)&PRCM_OMAP_CTRL_SYSCONFIG},
	{PRCM_MBOXES,   (u32 *)&PRCM_MBOXES_SYSCONFIG},
	{PRCM_MCBSP1,   (u32 *)&PRCM_MCBSP1_SYSCONFIG},
	{PRCM_MCBSP5,   (u32 *)&PRCM_MCBSP5_SYSCONFIG},
	{PRCM_GPT10,    (u32 *)&PRCM_GPT10_SYSCONFIG},
	{PRCM_GPT11,    (u32 *)&PRCM_GPT11_SYSCONFIG},
	{PRCM_UART1,    (u32 *)&PRCM_UART1_SYSCONFIG},
	{PRCM_UART2,    (u32 *)&PRCM_UART2_SYSCONFIG},
	{PRCM_I2C1,     (u32 *)&PRCM_I2C1_SYSCONFIG},
	{PRCM_I2C2,     (u32 *)&PRCM_I2C2_SYSCONFIG},
	{PRCM_I2C3,     (u32 *)&PRCM_I2C3_SYSCONFIG},
	{PRCM_MCSPI1,   (u32 *)&PRCM_MCSPI1_SYSCONFIG},
	{PRCM_MCSPI2,   (u32 *)&PRCM_MCSPI2_SYSCONFIG},
	{PRCM_MCSPI3,   (u32 *)&PRCM_MCSPI3_SYSCONFIG},
	{PRCM_MCSPI4,   (u32 *)&PRCM_MCSPI4_SYSCONFIG},
	{PRCM_HDQ,      (u32 *)&PRCM_HDQ_SYSCONFIG},
	{PRCM_MMC1,     (u32 *)&PRCM_MMC1_SYSCONFIG},
	{PRCM_MMC2,     (u32 *)&PRCM_MMC2_SYSCONFIG},
	{PRCM_SMS,      (u32 *)&PRCM_SMS_SYSCONFIG},
	{PRCM_GPMC,     (u32 *)&PRCM_GPMC_SYSCONFIG},
	{PRCM_MPU_INTC, (u32 *)&PRCM_MPU_INTC_SYSCONFIG},
	{PRCM_MMC3,     (u32 *)&PRCM_MMC3_SYSCONFIG},
	{0x0,           0x0},
};

/* WKUP */
struct sysconf_reg sysc_reg_wkup[] = {
	{PRCM_GPIO1,    (u32 *)&PRCM_GPIO1_SYSCONFIG},
	{PRCM_GPT1,     (u32 *)&PRCM_GPT1_SYSCONFIG},
	{PRCM_GPT12,    (u32 *)&PRCM_GPT12_SYSCONFIG},
	{PRCM_WDT2,     (u32 *)&PRCM_WDT2_SYSCONFIG},
	{0x0,           0x0},
};

struct sysconf_reg sysc_reg_dss[] = {
	{PRCM_DSS,      (u32 *)&PRCM_DSS_SYSCONFIG},
	{PRCM_DISPC,    (u32 *)&PRCM_DISPC_SYSCONFIG},
	{PRCM_RFBI,     (u32 *)&PRCM_RFBI_SYSCONFIG},
	{0x0,           0x0},
};

/* CAM */
struct sysconf_reg sysc_reg_cam[] = {
	{PRCM_CAM,      (u32 *)&PRCM_CAM_SYSCONFIG},
	{PRCM_CSIA,     (u32 *)&PRCM_CSIA_SYSCONFIG},
	{PRCM_CSIB,     (u32 *)&PRCM_CSIB_SYSCONFIG},
	{PRCM_MMU,      (u32 *)&PRCM_MMU_SYSCONFIG},
	{0,             0x0},
};

/* PER */
struct sysconf_reg sysc_reg_per[] = {
	{PRCM_MCBSP2,   (u32 *)&PRCM_MCBSP2_SYSCONFIG},
	{PRCM_MCBSP3,   (u32 *)&PRCM_MCBSP3_SYSCONFIG},
	{PRCM_MCBSP4,   (u32 *)&PRCM_MCBSP4_SYSCONFIG},
	{PRCM_GPT2,     (u32 *)&PRCM_GPT2_SYSCONFIG},
	{PRCM_GPT3,     (u32 *)&PRCM_GPT3_SYSCONFIG},
	{PRCM_GPT4,     (u32 *)&PRCM_GPT4_SYSCONFIG},
	{PRCM_GPT5,     (u32 *)&PRCM_GPT5_SYSCONFIG},
	{PRCM_GPT6,     (u32 *)&PRCM_GPT6_SYSCONFIG},
	{PRCM_GPT7,     (u32 *)&PRCM_GPT7_SYSCONFIG},
	{PRCM_GPT8,     (u32 *)&PRCM_GPT8_SYSCONFIG},
	{PRCM_GPT9,     (u32 *)&PRCM_GPT9_SYSCONFIG},
	{PRCM_UART3,    (u32 *)&PRCM_UART3_SYSCONFIG},
	{PRCM_WDT3,     (u32 *)&PRCM_WDT3_SYSCONFIG},
	{PRCM_GPIO2,    (u32 *)&PRCM_GPIO2_SYSCONFIG},
	{PRCM_GPIO3,    (u32 *)&PRCM_GPIO3_SYSCONFIG},
	{PRCM_GPIO4,    (u32 *)&PRCM_GPIO4_SYSCONFIG},
	{PRCM_GPIO5,    (u32 *)&PRCM_GPIO5_SYSCONFIG},
	{PRCM_GPIO6,    (u32 *)&PRCM_GPIO6_SYSCONFIG},
	{0,             0x0},
};

/* Array containing possible divider values for all clocks
 which have dividers */
u32 div_arr[] = {
	/* L3 divs */
	1, 2, 0, 0, 0, 0,
	/* L4 divs */
	1, 2, 0, 0, 0, 0,
	/* RM divs */
	1, 2, 0, 0, 0, 0,
	/* USB L4 divs */
	0, 0, 0, 0, 0, 0,
	/* sys clkout2 divs */
	1, 2, 4, 8, 16, 0,
	/* sgx divs */
	3, 4, 6, 0, 0, 0,
	/* SSI divs */
	1, 2, 3, 4, 5, 6,
	/* dpll1 fclk divs */
	1, 2, 4, 0, 0, 0,
	/* dpll2 fclk divs */
	1, 2, 4, 0, 0, 0,
	/* cm usim divs */
	2, 4, 8, 10, 16, 20,
};

/* Possible values for Dpll dividers */
u32 div_dpll[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
u32 div_dpll3[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,\
		17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31 };

u32 dpll_mx_shift[NO_OF_DPLL][NO_DPLL_DIV] = {
	{0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
	{0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
	{0x1B, 0x1B, 0x10, 0x0, 0x0, 0x0},
	{0x0, 0x0, 0x8, 0x0, 0x0, 0x18},
	{0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
};

/* Offset values in registers for clocks which have dividers */
u32 clk_div_offset[PRCM_NO_OF_CLKS] = {
	/* L3_ICLK */
	0x0,
	/* L4_ICLK */
	0x2,
	/* RM_ICLK */
	0x1,
	/* USB_L4_ICLK */
	0x0,
	/* SYS_CLKOUT2 */
	0x3,
	/* SGX_L3_FCLK */
	0x0,
	/* SSI_CLK */
	0x8,
	/* DPLL1_FCLK */
	0x13,
	/* DPLL2_FCLK */
	0x13,
	/* SYS_USIM_CLK */
	0x3,
};

/* Tables having M,N,M2 and FreqSel values for different sys_clk speeds & OPPs*/
/* The tables are organized as follows: */
/* Rows : 1 - 12M, 2 - 13M, 3 - 19.2M, 4 - 26M, 5 - 38.4M */
/* Columns : 1 - OPP1, 2 - OPP2, 3 - OPP3, 4 - OPP4  5 - OPP5 */

/* MPU parameters */
struct dpll_param mpu_dpll_param[5][PRCM_NO_VDD1_OPPS] = {
	/* 12M values */
	/* OPP1(125 Mhz) and OPP2(250 Mhz)*/
	{{0x0FA, 0x05, 0x07, 0x04}, {0x0FA, 0x05, 0x07, 0x02},
	/* OPP3(500 Mhz) and OPP4(550 Mhz)*/
	{0x0FA, 0x05, 0x07, 0x01}, {0x113, 0x05, 0x07, 0x01},
	/* OPP5 (600 Mhz) */
	{0x12C, 0x05, 0x07, 0x01} },
	/* 13M values */
	/* OPP1(125 Mhz) and OPP2(250 Mhz)*/
	{{0x1F4, 0x0C, 0x03, 0x04}, {0x1F4, 0x0C, 0x03, 0x02},
	/* OPP3(500 Mhz) and OPP4(550 Mhz)*/
	{0x1F4, 0x0C, 0x03, 0x01}, {0x226, 0x0C, 0x03, 0x01},
	/* OPP5 (600 Mhz) */
	{0x258, 0x0C, 0x03, 0x01} },
	/* 19.2M values */
	/* OPP1(125 Mhz) and OPP2(250 Mhz)*/
	{{0x271, 0x17, 0x03, 0x04}, {0x271, 0x17, 0x03, 0x02},
	/* OPP3(500 Mhz) and OPP4(550 Mhz)*/
	{0x271, 0x17, 0x03, 0x01}, {0x191, 0x0D, 0x05, 0x01},
	/* OPP5 (600 Mhz) */
	{0x177, 0x0B, 0x06, 0x01} },
	/* 26M values */
	/* OPP1(125 Mhz) and OPP2(250 Mhz)*/
	{{0x0FA, 0x0C, 0x07, 0x04}, {0x0FA, 0x0C, 0x07, 0x02},
	/* OPP3(500 Mhz) and OPP4(550 Mhz)*/
	{0x0FA, 0x0C, 0x07, 0x01}, {0x113, 0x0C, 0x07, 0x01},
	/* OPP5 (600 Mhz) */
	{0x12C, 0x0C, 0x07, 0x01} },
	/* 38.4M values */
	/* OPP1(125 Mhz) and OPP2(250 Mhz)*/
	{{0x271, 0x2F, 0x03, 0x04}, {0x271, 0x2F, 0x03, 0x02},
	/* OPP3(500 Mhz) and OPP4(550 Mhz)*/
	{0x271, 0x2F, 0x03, 0x01}, {0x1BC, 0x1E, 0x04, 0x01},
	/* OPP5 (600 Mhz) */
	{0x177, 0x17, 0x06, 0x01} },
};


/* IVA parameters */
struct dpll_param iva_dpll_param[5][PRCM_NO_VDD1_OPPS] = {
	/* 12M values */
	/* OPP1(90 Mhz) and OPP2(180 Mhz)*/
	{{0x0B4, 0x05, 0x07, 0x04}, {0x0B4, 0x05, 0x07, 0x02},
	/* OPP3(360 Mhz) and OPP4(396 Mhz)*/
	 {0x0B4, 0x05, 0x07, 0x01}, {0x0C6, 0x05, 0x07, 0x01},
	/* OPP5 (430 Mhz) */
	 {0x0D7, 0x05, 0x07, 0x01} },
	/* 13M values */
	/* OPP1(90 Mhz) and OPP2(180 Mhz)*/
	{{0x168, 0x0C, 0x03, 0x04}, {0x168, 0x0C, 0x03, 0x02},
	/* OPP3(360 Mhz) and OPP4(400 Mhz)*/
	 {0x168, 0x0C, 0x03, 0x01}, {0x190, 0x0C, 0x03, 0x01},
	/* OPP5 (430 Mhz) */
	 {0x1AE, 0x0C, 0x03, 0x01} },
	/* 19.2M values */
	/* OPP1(90 Mhz) and OPP2(180 Mhz)*/
	{{0x0E1, 0x0B, 0x06, 0x04}, {0x0E1, 0x0B, 0x06, 0x02},
	/* OPP3(360 Mhz) and OPP4(396 Mhz)*/
	 {0x0E1, 0x0B, 0x06, 0x01}, {0x14A, 0x0F, 0x04, 0x01},
	/* OPP5 (430 Mhz) */
	 {0x203, 0x16, 0x03, 0x01} },
	/* 26M values */
	/* OPP1(90 Mhz) and OPP2(180 Mhz)*/
	{{0x0B4, 0x0C, 0x07, 0x04}, {0x0B4, 0x0C, 0x07, 0x02},
	/* OPP3(360 Mhz) and OPP4(396 Mhz)*/
	 {0x0B4, 0x0C, 0x07, 0x01}, {0x0C6, 0x0C, 0x07, 0x01},
	/* OPP5 (430 Mhz) */
	 {0x0D7, 0x0C, 0x07, 0x01} },
	/* 38.4M values */
	/* OPP1(90 Mhz) and OPP2(180 Mhz)*/
	{{0x0E1, 0x17, 0x06, 0x04}, {0x0E1, 0x17, 0x06, 0x02},
	/* OPP3(360 Mhz) and OPP4(396 Mhz)*/
	 {0x0E1, 0x17, 0x06, 0x01}, {0x14A, 0x1F, 0x04, 0x01},
	/* OPP5 (430 Mhz) */
	 {0x23B, 0x32, 0x01, 0x01} },
};

/* CORE parameters */
struct dpll_param core_dpll_param[5][PRCM_NO_VDD2_OPPS] = {
#ifdef CONFIG_OMAP3_CORE_166MHZ
	/* 12M values */
	/* OPP1(83 Mhz) and OPP2(166 Mhz) */
	{{0, 0, 0, 0}, {0x0A6, 0x05, 0x07, 0x02}, {0x0A6, 0x05, 0x07, 0x01} },
	/* 13M values */
	/* OPP1(83 Mhz) and OPP2(166 Mhz) */
	{{0, 0, 0, 0}, {0x14C, 0x0C, 0x03, 0x02}, {0x14C, 0x0C, 0x03, 0x01} },
	/* 19.2M values */
	/* OPP1(83 Mhz) and OPP2(166 Mhz) */
	{{0, 0, 0, 0}, {0x19F, 0x17, 0x03, 0x02}, {0x19F, 0x17, 0x03, 0x01} },
	/* 26M values */
	/* OPP1(83 Mhz) and OPP2(166 Mhz) */
	{{0, 0, 0, 0}, {0x0A6, 0x0C, 0x07, 0x02}, {0x0A6, 0x0C, 0x07, 0x01} },
	/* 38.4M values */
	/* OPP1(83 Mhz) and OPP2(166 Mhz) */
	{{0, 0, 0, 0}, {0x19F, 0x2F, 0x03, 0x02}, {0x19F, 0x2F, 0x03, 0x01} },

#elif defined(CONFIG_OMAP3_CORE_133MHZ)
	/* 12M values */
	/* OPP1(66 Mhz) and OPP2(133 Mhz) */
	{{0, 0, 0, 0}, {0x085, 0x05, 0x07, 0x02}, {0x085, 0x05, 0x07, 0x01} },
	/* 13M values */
	/* OPP1(66 Mhz) and OPP2(133 Mhz) */
	{{0, 0, 0, 0}, {0x10A, 0x0C, 0x03, 0x02}, {0x10A, 0x0C, 0x03, 0x01} },
	/* 19.2M values */
	/* OPP1(83 Mhz) and OPP2(133 Mhz) */
	{{0, 0, 0, 0}, {0x115, 0x13, 0x03, 0x02}, {0x115, 0x13, 0x03, 0x01} },
	/* 26M values */
	/* OPP1(66 Mhz) and OPP2(133 Mhz) */
	{{0, 0, 0, 0}, {0x085, 0x0C, 0x07, 0x02}, {0x085, 0x0C, 0x07, 0x01} },
	/* 38.4M values */
	/* OPP1(66 Mhz) and OPP2(133 Mhz) */
	{{0, 0, 0, 0}, {0x11C, 0x28, 0x03, 0x02}, {0x19F, 0x2F, 0x03, 0x01} },
#endif
};

struct dpll_param usb_dpll_param[5] = {
	/* 12M values */
	{0x3C, 0x05, 0x07, 0x01},
	/* 13M values */
	{0x78, 0x0C, 0x03, 0x01},
	/* 19.2M values  */
	{0x4B, 0x0B, 0x06, 0x01},
	/* 26M values */
	{0x3C, 0x0C, 0x07, 0x01},
	/* 38.4M values */
	{0x4B, 0x17, 0x06, 0x01},
};

u8 mpu_iva2_vdd1_volts [PRCM_NO_VDD1_OPPS] = {
	/* Vsel corresponding to 0.9V (OPP1), 1.00V (OPP2),
				1.20V (OPP3), 1.27V (OPP4), 1.35 (OPP5) */
	0x18, 0x20, 0x30, 0x36, 0x3C
};

u8 core_l3_vdd2_volts [PRCM_NO_VDD2_OPPS] = { /* only 3 OPPs */
	/* Vsel corresponding to 0.9V (OPP1), 1.00V (OPP2), 1.15 (OPP3) */
	0x18, 0x20, 0x2C
};
#if 0
u32 omap_prcm_get_reset_sources(void)
{
	omap3_clk_prepare_for_reboot();
	return PRM_RSTST & 0x7fb;
}
EXPORT_SYMBOL(omap_prcm_get_reset_sources);
#endif

void omap_udelay(u32 udelay)
{
	u32 counter_val, timeout_val;

	counter_val = OMAP_TIMER32K_SYNC_CR_READ;
	/* Since the 32 sync timer runs on a 32K clock,
	   the granularity of the delay achieved is around 30
	   us, hence divide the delay provided by user by 30 */
	timeout_val = counter_val + (udelay / 30);
	if (timeout_val < counter_val)
		/* There seems to be a overflow */
		/* take care of the overflow by waiting first for the
		   CR to reach the MAX value */
		while (OMAP_MAX_32K_CR > OMAP_TIMER32K_SYNC_CR_READ) ;
	/* wait for the specified delay */
	while (timeout_val > OMAP_TIMER32K_SYNC_CR_READ) ;
	return;
}

inline int loop_wait(u32 *lcnt, u32 *rcnt, u32 delay)
{
	(*lcnt)++;
	if (*lcnt > MAXLOOPCNT) {
		*lcnt = 0;
		if (*rcnt < MAXRETRIES)
			omap_udelay(delay);
		else
			return PRCM_FAIL;
		(*rcnt)++;
	}
	return PRCM_PASS;
}
#if 0
/* Resets clock rates and reboots the system. Only called from system.h */
void omap_prcm_arch_reset(char mode)
{
	omap3_clk_prepare_for_reboot();
	/* Assert global software reset */
	PRM_RSTCTRL |= 2;
}
#endif

static int check_device_status(u32 deviceid, u8 control)
{
	u8 curr_state;
	u32 loop_cnt = 0, retries_cnt = 0;
	int ret;

	if ((control != PRCM_ENABLE) && (control != PRCM_DISABLE))
		return PRCM_FAIL;

	curr_state = !control;
	while (curr_state != control) {
		ret = prcm_is_device_accessible(deviceid, &curr_state);
		if (ret != PRCM_PASS)
			return ret;
		ret = loop_wait(&loop_cnt, &retries_cnt, 100);
		if (ret != PRCM_PASS) {
			printk(KERN_INFO "Loop count exceeded in check "
				"device status for device:%u\n", deviceid);
			return ret;
		}
	}
	return PRCM_PASS;
}

static int get_dpll_mx(u32 dpll_id, u32 dpll_div, u32 dpll_mxsft)
{
	u32 valid;
	u32 *addr;

	addr = get_addr_pll(dpll_id, dpll_div + MX_ARRAY_OFFSET);
	valid = get_val_bits_pll(dpll_id, dpll_div + MX_ARRAY_OFFSET);
	return (*addr & ~valid) >> dpll_mxsft;
}

int get_dpll_m_n(u32 dpll_id, u32 *mult, u32 *div)
{
	u32 valid;
	u32 *addr;

	addr = get_addr_pll(dpll_id, REG_CLKSEL1_PLL);
	if (!addr)
		return PRCM_FAIL;

	valid = get_val_bits_pll(dpll_id, REG_CLKSEL1_PLL);
	if (dpll_id == DPLL3_CORE) {
		*mult = (*addr & ~valid) >> 16;
		*div = (*addr & ~DPLL3_N_MASK) >> 8;
	} else {
		*mult = (*addr & ~valid) >> 8;
		*div = (*addr & ~DPLL_N_MASK);
	}
	return PRCM_PASS;
}

static int is_dpll_locked(u32 dpll_id, int *result)
{
	u32 *addr;
	u32 dpll_enbit_mask, dpll_idlest_lock_bit;

	addr = get_addr_pll(dpll_id, REG_CLKEN_PLL);
	if (!addr)
		return PRCM_FAIL;

	dpll_enbit_mask = get_dpll_enbitmask(dpll_id);
	dpll_idlest_lock_bit = get_idlest_lock_bit(dpll_id);

	if ((*addr & (~dpll_enbit_mask)) == (~dpll_enbit_mask))
		*result = PRCM_TRUE;
	else
		*result = PRCM_FALSE;
	return PRCM_PASS;
}

static int check_accessibility(u32 deviceid, u8 clk_type)
{
	u32 valid = 0, enbit, type, domain;
	u32 *addr = 0;

	enbit = DEV_BIT_POS(deviceid);
	domain = DOMAIN_ID(deviceid);
	type = DEV_TYPE(deviceid);

	if (type == TARGET) {
		/* Skip devices without CM_IDLEST register bit support */
		addr = get_addr(domain, REG_IDLEST);
		valid = get_val_bits(domain, REG_IDLEST);
		if (!(addr) || !(valid & (1 << enbit)))
			return PRCM_PASS;
		/* Check if FCLK/ICLK is absent or ICLK is ON if present. */
		if (clk_type == ICLK) {
			addr = get_addr(domain, REG_FCLKEN);
			valid = get_val_bits(domain, REG_FCLKEN);
		} else if (clk_type == FCLK) {
			addr = get_addr(domain, REG_ICLKEN);
			valid = get_val_bits(domain, REG_ICLKEN);
		}
		if (!(addr) || !(valid & (1 << enbit))
		    || (*addr & (1 << enbit)))
			/* FCLK/ICLK present and is ON */
			return check_device_status(deviceid, PRCM_ENABLE);
	} else {		/* type = INITIATOR or INITIATOR+TARGET
				 * IDLEST bit cannot be polled,
				 * it only specifies the
				 * standby status
				 * Check if the ICLK is present and ON */
		if (clk_type == FCLK) {
			addr = get_addr(domain, REG_ICLKEN);
			valid = get_val_bits(domain, REG_ICLKEN);
		} else if (clk_type == ICLK) {
			addr = get_addr(domain, REG_FCLKEN);
			valid = get_val_bits(domain, REG_FCLKEN);
		}
		if (!(addr) || !(valid & (1 << enbit))
		    || (*addr & (1 << enbit))) {
			/* FCLK/ICLK present and is ON
			 * Wait for sometime for the clocks to stabilize*/
			omap_udelay(100);
			return PRCM_PASS;
		}
	}
	return PRCM_PASS;
}

static int calc_dpll_lock_delay(u32 dpll_id, u32 *delay)
{
	u32 f_ref, f_int, m, n;
	/* Calcluate an appropriate delay based on the below formula */
	/* This is a worst case formula which assumes there was a
	   M/N reprogramming.
	   2us + 350Fint_cycles (Fint_cycles = Fref/N+1)*/
	f_ref = prcm_get_system_clock_speed();
	if (f_ref == PRCM_FAIL) {
		printk(KERN_INFO "Unable to get system clock\n");
		return PRCM_FAIL;
	}

	if (get_dpll_m_n(dpll_id, &m, &n) == PRCM_FAIL) {
		printk(KERN_INFO "Failed to get the M and N values\n");
		return PRCM_FAIL;
	}

	f_int = f_ref / (n + 1);
	*delay = (2 + (350*1000)/f_int);
	return PRCM_PASS;
}

int prcm_clock_control(u32 deviceid, u8 clk_type, u8 control,
		       u8 checkaccessibility)
{
	u32 domain, omap, valid, enbit;
	u32 *addr;


	omap = OMAP(deviceid);
	enbit = DEV_BIT_POS(deviceid);
	domain = DOMAIN_ID(deviceid);

	if (cpu_is_omap3430() && !(omap & (AT_3430|AT_3430_ES2)))
		return PRCM_FAIL;

	switch (clk_type) {
	case ICLK:
		addr = get_addr(domain, REG_ICLKEN);
		valid = get_val_bits(domain, REG_ICLKEN);
		break;
	case FCLK:
		addr = get_addr(domain, REG_FCLKEN);
		valid = get_val_bits(domain, REG_FCLKEN);
		break;
	default:
		return PRCM_FAIL;
	}

	/* No functional/Interface Clk control for the device */
	if (!(addr) || !(valid & (1 << enbit)))
		return PRCM_FAIL;

	if (control == PRCM_ENABLE)
		*addr |= (1 << enbit);
	else if (control == PRCM_DISABLE)
		*addr &= ~(1 << enbit);

	if (checkaccessibility)
		return check_accessibility(deviceid, clk_type);

	return PRCM_PASS;
}

int prcm_is_device_accessible(u32 deviceid, u8 *result)
{
	u32 domain, omap, valid, enbit;
	u32 *addr;

	omap = OMAP(deviceid);
	enbit = DEV_BIT_POS(deviceid);
	domain = DOMAIN_ID(deviceid);

	if (cpu_is_omap3430() && !(omap & (AT_3430|AT_3430_ES2)))
		return PRCM_FAIL;

	addr = get_addr(domain, REG_IDLEST);
	valid = get_val_bits(domain, REG_IDLEST);

	if (!(addr) || !(valid & (1 << enbit)))
		return PRCM_FAIL;


	if (!(*addr & (1 << enbit))) {
		*result = PRCM_TRUE;
	} else {
		*result = PRCM_FALSE;
	}
	return PRCM_PASS;
}

int prcm_interface_clock_autoidle(u32 deviceid, u8 control)
{
	u32 domain, omap, valid, enbit;
	u32 *addr;

	omap = OMAP(deviceid);
	enbit = DEV_BIT_POS(deviceid);
	domain = DOMAIN_ID(deviceid);

	if (cpu_is_omap3430() && !(omap & (AT_3430|AT_3430_ES2)))
		return PRCM_FAIL;

	addr = get_addr(domain, REG_AUTOIDLE);
	valid = get_val_bits(domain, REG_AUTOIDLE);

	if (!(addr) || !(valid & (1 << enbit)))
		return PRCM_FAIL;

	if (control == PRCM_ENABLE)
		*addr |= (1 << enbit);
	else if (control == PRCM_DISABLE)
		*addr &= ~(1 << enbit);

	return PRCM_PASS;
}

int prcm_wakeup_event_control(u32 deviceid, u8 control)
{
	u32 domain, omap, valid, enbit;
	u32 *addr;

	omap = OMAP(deviceid);
	enbit = DEV_BIT_POS(deviceid);
	domain = DOMAIN_ID(deviceid);

	if (cpu_is_omap3430() && !(omap & (AT_3430|AT_3430_ES2)))
		return PRCM_FAIL;

	addr = get_addr(domain, REG_WKEN);
	valid = get_val_bits(domain, REG_WKEN);

	if (!(addr) || !(valid & (1 << enbit)))
		return PRCM_FAIL;

	if (control == PRCM_ENABLE)
		*addr |= (1 << enbit);
	else if (control == PRCM_DISABLE)
		*addr &= ~(1 << enbit);

	return PRCM_PASS;
}

int prcm_enable_dpll(u32 dpll_id)
{
	u32 dpll_idlest_lock_bit, dpll_enbit_mask, delay;
	u32 *addr, *addr_auto;
	u32 dpll_autoidle;
	int ret, enabled;
	u32 loop_cnt = 0, retries_cnt = 0;

	if (dpll_id > NO_OF_DPLL)
		return PRCM_FAIL;

	/* Currently, this API does not allow locking of core DPLL */
	/* Locking of core DPLL needs to be done without access to SDRAM */
	/* This can be done safely if execution is done from SRAM */
	if (dpll_id == DPLL3_CORE)
		return PRCM_FAIL;

	/* Store the DPLL autoidle */
	addr_auto = get_addr_pll(dpll_id, REG_AUTOIDLE_PLL);
	dpll_autoidle = *addr_auto;
	*addr_auto = 0x0;

	ret = is_dpll_locked(dpll_id, &enabled);
	if (ret != PRCM_PASS) {
		*addr_auto = dpll_autoidle;
		return ret;
	}
	if (enabled == PRCM_TRUE) {
		*addr_auto = dpll_autoidle;
		return PRCM_PASS;
	}

	addr = get_addr_pll(dpll_id, REG_CLKEN_PLL);

	if (!addr) {
		*addr_auto = dpll_autoidle;
		return PRCM_FAIL;
	}

	dpll_enbit_mask = get_dpll_enbitmask(dpll_id);
	dpll_idlest_lock_bit = get_idlest_lock_bit(dpll_id);

	*addr |= ~dpll_enbit_mask;	/* enable DPLL in lock mode */

	if (is_sil_rev_equal_to(OMAP3430_REV_ES1_0)) {
		/* WORKAROUND FOR SILICON ERRATA 1.56 */
		ret = calc_dpll_lock_delay(dpll_id, &delay);
		if (ret != PRCM_PASS) {
			*addr_auto = dpll_autoidle;
			return ret;
		}
		omap_udelay(delay);
	}

	ret = calc_dpll_lock_delay(dpll_id, &delay);
	if (ret != PRCM_PASS) {
		*addr_auto = dpll_autoidle;
		return ret;
	}
	while (!(*get_addr_pll(dpll_id, REG_IDLEST_PLL) &
						 dpll_idlest_lock_bit)) {
		/* wait for DPLL to lock */
		ret = loop_wait(&loop_cnt, &retries_cnt, delay/5);
		if (ret != PRCM_PASS) {
		printk(KERN_INFO "Loop count exceeded in"
				"prcm_enable_dpll for dpll:%u\n", dpll_id);
			*addr_auto = dpll_autoidle;
			return ret;
		}
	}
	/* Restore the autoidle for the DPLL back */
	*addr_auto = dpll_autoidle;
	return PRCM_PASS;
}

int prcm_configure_dpll(u32 dpll_id, u32 mult, u8 div, u8 freq_sel)
{
	u32 valid;
	u32 *addr, *addr_auto;
	u32 new_reg_val = 0x0;
	int ret, enabled, index;
	u32 sys_clkspeed, dpll_autoidle;

	if (dpll_id > NO_OF_DPLL)
		return PRCM_FAIL;

	if (is_sil_rev_equal_to(OMAP3430_REV_ES1_0)) {
		/* WORKAROUND FOR Limitation 2.5 */
		if (dpll_id == DPLL4_PER)
			return PRCM_FAIL;
	}

	/* Store the DPLL autoidle */
	addr_auto = get_addr_pll(dpll_id, REG_AUTOIDLE_PLL);
	dpll_autoidle = *addr_auto;
	*addr_auto = 0x0;

	/* DPLL M,N,FreqSel values should be changed only if the DPLL
	 * is in bypass mode. If it is not in bypass mode, return error */
	ret = is_dpll_locked(dpll_id, &enabled);

	if (enabled == PRCM_TRUE) {
		printk(KERN_INFO "Dpll enabled - m,n values cannot be"
								"changed\n");
		*addr_auto = dpll_autoidle;
		return PRCM_FAIL;
	}

	/* Configure M and N values */
	addr = get_addr_pll(dpll_id, REG_CLKSEL1_PLL);
	if (!addr)
		return PRCM_FAIL;
	if (dpll_id == DPLL5_PER2) {
		/* get the M/N/freqsel values */
		sys_clkspeed = prcm_get_system_clock_speed();
		switch (sys_clkspeed) {
		case (int)(12000):
			index = 0;
			break;
		case (int)(13000):
			index = 1;
			break;
		case (int)(19200):
			index = 2;
			break;
		case (int)(26000):
			index = 3;
			break;
		case (int)(38400):
			index = 4;
			break;
		default:
			return PRCM_FAIL;
		}
		mult = usb_dpll_param[index].dpll_m;
		div = usb_dpll_param[index].dpll_n;
		freq_sel = usb_dpll_param[index].dpll_freqsel;
	}
	valid = get_val_bits_pll(dpll_id, REG_CLKSEL1_PLL);
	if (dpll_id == DPLL3_CORE) {
		new_reg_val = *addr & valid & DPLL3_N_MASK;
		new_reg_val |= (mult << 16) | (div << 8);
		*addr = new_reg_val;
	} else {
		new_reg_val = *addr & valid & DPLL_N_MASK;
		new_reg_val |= (mult << 8) | div;
		*addr = new_reg_val;
	}

	/* Configure FreqSel values */
	addr = get_addr_pll(dpll_id, REG_CLKEN_PLL);
	if (!addr)
		return PRCM_FAIL;
	valid = get_val_bits_pll(dpll_id, REG_CLKEN_PLL);
	if (dpll_id == DPLL4_PER) {
		new_reg_val = *addr & valid;
		new_reg_val |= (freq_sel << 20);
		*addr = new_reg_val;
	} else {
		new_reg_val = *addr & valid;
		new_reg_val |= (freq_sel << 4);
		*addr = new_reg_val;
	}
	*addr_auto = dpll_autoidle;
	return PRCM_PASS;
}

int prcm_put_dpll_in_bypass(u32 dpll_id, u32 bypass_mode)
{
	u32 new_val;
	u32 *addr, *addr_auto;
	u32 dpll_autoidle;
	u32 loop_cnt = 0, retries_cnt = 0;
	int ret = PRCM_FAIL;

	if (dpll_id > NO_OF_DPLL)
		return ret;

	/* Currently, the API does not allow putting CORE dpll in bypass mode
	 * To safely put dpll in bypass mode, it is better to execute code
	 * from sram so that there is no access to sdram */
	if (dpll_id == DPLL3_CORE)
		return ret;

	addr = get_addr_pll(dpll_id, REG_CLKEN_PLL);
	if (!addr)
		return ret;

	/* This is needed if the condition in while loop returns true the */
							/*very first time*/
	ret = PRCM_PASS;

	/* Store the DPLL autoidle */
	addr_auto = get_addr_pll(dpll_id, REG_AUTOIDLE_PLL);
	dpll_autoidle = *addr_auto;
	*addr_auto = 0x0;

	if (dpll_id == DPLL1_MPU) {
		new_val = (*addr & DPLL_ENBIT_MASK) | LOW_POWER_BYPASS;
		*addr = new_val;
		while (*get_addr_pll(dpll_id, REG_IDLEST_PLL) & 0x1) {
			ret = loop_wait(&loop_cnt, &retries_cnt, 1000);
			if (ret != PRCM_PASS)
				break;
		}
	} else if (dpll_id == DPLL4_PER) {
		new_val = (*addr & DPLL4_ENBIT_MASK) | (LOW_POWER_STOP << 16);
		*addr = new_val;
		while (*get_addr_pll(dpll_id, REG_IDLEST_PLL) & 0x2) {
			ret = loop_wait(&loop_cnt, &retries_cnt, 1000);
			if (ret != PRCM_PASS)
				break;
		}
	} else {
		if ((dpll_id == DPLL5_PER2) && (bypass_mode != LOW_POWER_STOP))
			return ret;
		new_val = (*addr & DPLL_ENBIT_MASK) | bypass_mode;
		*addr = new_val;
		while (*get_addr_pll(dpll_id, REG_IDLEST_PLL) & 0x1) {
			ret = loop_wait(&loop_cnt, &retries_cnt, 1000);
			if (ret != PRCM_PASS)
				break;
		}
	}
	if (ret != PRCM_PASS)
		printk(KERN_INFO "Loop count exceeded in "
			"prcm_put_dpll_in_bypass for dpll:%u\n", dpll_id);

	/* Restore the autoidle for the DPLL back */
	*addr_auto = dpll_autoidle;
	return ret;
}

int prcm_dpll_clock_auto_control(u32 dpll_id, u32 control)
{
	u32 valid;
	u32 *addr;
	u32 setting;

	if (dpll_id > NO_OF_DPLL)
		return PRCM_FAIL;

	addr = get_addr_pll(dpll_id, REG_AUTOIDLE_PLL);
	if (!addr)
		return PRCM_FAIL;

	valid = get_val_bits_pll(dpll_id, REG_AUTOIDLE_PLL);

	if (dpll_id == DPLL4_PER)
		setting = 1 << 3;
	else if (dpll_id == DPLL3_CORE) {
		*addr &= ~CORE3_DPLL_MASK;
		switch (control) {
		case DPLL_AUTOIDLE_BYPASS:
			setting = 5;
			break;
		case DPLL_NO_AUTOIDLE:
		case DPLL_AUTOIDLE:
			setting = 1;
			break;
		default:
			return PRCM_FAIL;
			}
	} else
		setting = 1;

	switch (control) {
	case DPLL_AUTOIDLE:
		*addr |= setting;
		break;
	case DPLL_NO_AUTOIDLE:
		*addr &= ~setting;
		break;
	case DPLL_AUTOIDLE_BYPASS:
		*addr |= setting;
		break;
	default:
		return PRCM_FAIL;
	}
	return PRCM_PASS;
}

int prcm_get_dpll_mn_output(u32 dpll, u32 *mn_output)
{
	u32 dpll_idlest_lock_bit, clksel1_pll, dpll_id;
	u32 sys_clkspeed, core_clkspeed;
	int bypassclk_divider;
	u32 mult, div;
	u32 *addr;

	dpll_id = (dpll >> DPLL_NO_POS) & DPLL_NO_MASK;

	if (dpll_id > NO_OF_DPLL)
		return PRCM_FAIL;

	dpll_idlest_lock_bit = get_idlest_lock_bit(dpll_id);

	addr = get_addr_pll(dpll_id, REG_IDLEST_PLL);
	/* Get the sys clock speed */
	sys_clkspeed = prcm_get_system_clock_speed();

	if (*addr & dpll_idlest_lock_bit) {
		/* dpll locked */
		get_dpll_m_n(dpll_id, &mult, &div);
		*mn_output = ((sys_clkspeed * mult) / (div + 1));
	} else {
		/* dpll is in bypass mode */
		if ((dpll_id == DPLL3_CORE) || (dpll_id == DPLL4_PER)
						|| (dpll_id == DPLL5_PER2))
			* mn_output = sys_clkspeed;
		else {		/* DPLL1 and DPLL2
				 * Check if DPLL3 is in bypass */
			prcm_get_dpll_rate(PRCM_DPLL3_M2X2_CLK, &core_clkspeed);
			if (dpll_id == DPLL1_MPU)
				clksel1_pll = CM_CLKSEL1_PLL_MPU;
			else
				clksel1_pll = CM_CLKSEL1_PLL_IVA2;
			bypassclk_divider = (clksel1_pll >> 19) & 0x3;
			*mn_output = core_clkspeed / bypassclk_divider;
		}
	}
	return PRCM_PASS;
}

int prcm_get_dpll_rate(u32 dpll, u32 *output)
{
	u32 dpll_id, dpll_div, dpll_mxsft, id_type;
	u32 mx, omap;
	u32 mn_output;

	id_type = get_id_type(dpll);
	if (!(id_type & ID_DPLL_OP))
		return PRCM_FAIL;	/*Not dpll op */

	omap = OMAP(dpll);

	if (cpu_is_omap3430() && !(omap & (AT_3430|AT_3430_ES2)))
		return PRCM_FAIL;

	dpll_id = (dpll >> DPLL_NO_POS) & DPLL_NO_MASK;
	if (dpll_id > NO_OF_DPLL)
		return PRCM_FAIL;

	dpll_div = (dpll >> DPLL_DIV_POS) & DPLL_DIV_MASK;
	dpll_mxsft = dpll_mx_shift[dpll_id-1][dpll_div];
	mx = get_dpll_mx(dpll_id, dpll_div, dpll_mxsft);

	prcm_get_dpll_mn_output(dpll, &mn_output);

	/* Except for DPLL_M2 all clocks are (mn_output*2)/mx */
	if (dpll_div == DPLL_M2)
		*output = (mn_output) / mx;
	else
		*output = (2 * mn_output) / mx;
	return PRCM_PASS;
}

int prcm_configure_dpll_divider(u32 dpll, u32 setting)
{
	u32 dpll_id, valid, dpll_mxsft, omap, id_type;
	u32 dpll_div, new_val = 0x00000000;
	u32 *addr;

	id_type = get_id_type(dpll);
	if (!(id_type & ID_DPLL_OP))
		return PRCM_FAIL;	/*Not DPLL OP */

	omap = OMAP(dpll);

	if (cpu_is_omap3430() && !(omap & (AT_3430|AT_3430_ES2)))
		return PRCM_FAIL;

	dpll_id = (dpll >> DPLL_NO_POS) & DPLL_NO_MASK;
	if (dpll_id > NO_OF_DPLL)
		return PRCM_FAIL;

	if (is_sil_rev_equal_to(OMAP3430_REV_ES1_0)) {
		/* Workaround for limiation 2.5 */
		/* On ES 1.0, DPLL4 dividers cannot be changed */
		if (dpll_id == DPLL4_PER)
			return PRCM_FAIL;
	}

	dpll_div = (dpll >> DPLL_DIV_POS) & DPLL_DIV_MASK;
	dpll_mxsft = dpll_mx_shift[dpll_id-1][dpll_div];
	addr = get_addr_pll(dpll_id, dpll_div + MX_ARRAY_OFFSET);
	valid = get_val_bits_pll(dpll_id, dpll_div + MX_ARRAY_OFFSET);

	new_val = (*addr & valid) | (setting << dpll_mxsft);

	*addr = new_val;
	return PRCM_PASS;
}

int prcm_get_crystal_rate(void)
{
	u32 osc_clkspeed;

	osc_clkspeed = PRM_CLKSEL & 0x7;

	switch (osc_clkspeed) {
	case 0:
		return (int)(12000);	/*12MHz*/
	case 1:
		return (int)(13000);	/*13MHz*/
	case 2:
		return (int)(19200);	/*19.2MHz*/
	case 3:
		return (int)(26000);	/*26MHz*/
	case 4:
		return (int)(38400);	/*38.4MHz*/
	}

	return PRCM_FAIL;
}

int prcm_get_system_clock_speed(void)
{
	u32 osc_clkspeed, sys_clkdiv;

	osc_clkspeed = prcm_get_crystal_rate();
	sys_clkdiv = (PRM_CLKSRC_CTRL >> 6) & 0x3;
	if (osc_clkspeed == PRCM_FAIL)
		return PRCM_FAIL;
	else
		return osc_clkspeed / sys_clkdiv;
}

int prcm_select_system_clock_divider(u32 setting)
{
	u32 new_value = 0x00000000;

	new_value = (PRM_CLKSRC_CTRL & SYSCLK_DIV_MASK);
	new_value = new_value | (setting << 6);
	PRM_CLKSRC_CTRL = new_value;

	return PRCM_PASS;
}

int prcm_control_external_output_clock1(u32 control)
{
	u32 new_value = 0x00000000;

	new_value = (PRM_CLKOUT_CTRL & EXTCLK_OUTCTRL_MASK);
	new_value = new_value | (control << 7);
	PRM_CLKOUT_CTRL = new_value;

	return PRCM_PASS;
}

int prcm_control_external_output_clock2(u32 control)
{
	u32 new_value = 0x00000000;

	new_value = (CM_CLKOUT_CTRL & EXTCLK_OUTCTRL_MASK);
	new_value = new_value | (control << 7);
	CM_CLKOUT_CTRL = new_value;

	return PRCM_PASS;
}

int prcm_clksel_get_divider(u32 clk, u32 *div)
{
	u32 valid, offset, clk_id, omap, id_type;
	u32 *addr;

	id_type = get_id_type(clk);
	if (!(id_type & ID_CLK_DIV))
		return PRCM_FAIL;	/*No divider */

	omap = OMAP(clk);

	if (cpu_is_omap3430() && !(omap & (AT_3430|AT_3430_ES2)))
		return PRCM_FAIL;

	/* In case of 48m_fclk and 12m_fclk, the dividers are fixed */
	/* So this information is not stored in an array */
	if (clk == PRCM_48M_FCLK) {
		(*div) = 2;
		return PRCM_PASS;
	} else if (clk == PRCM_12M_FCLK) {
		(*div) = 4;
		return PRCM_PASS;
	}
	clk_id = (clk >> CLK_NO_POS) & CLK_NO_MASK;
	offset = clk_div_offset[clk_id-1];

	if (clk_id > PRCM_NO_OF_CLKS)
		return PRCM_FAIL;

	addr = clksel_reg[clk_id - 1].reg_addr;
	if (!addr)
		return PRCM_FAIL;
	valid = clksel_reg[clk_id - 1].valid_bits;

	*div = (*addr & valid) >> offset;
	/* For sys_clkout2, the divider is not same as value in register */
	/* Need to do bit shifting to get actual divider value */
	if (clk == PRCM_SYS_CLKOUT2)
		*div = (1 << (*div));
	if (clk == PRCM_USIM) {
		switch (*div) {
		case 0x3:
			*div = 2;
			break;
		case 0x5:
			*div = 8;
			break;
		case 0x6:
			*div = 10;
			break;
		case 0x7:
			*div = 4;
			break;
		case 0x9:
			*div = 16;
			break;
		case 0xA:
			*div = 20;
			break;
		default:
			break;
		}
	}
	if (clk == PRCM_SGX_FCLK) {
		switch (*div) {
		case 0x0:
			*div = 3;
			break;
		case 0x1:
			*div = 4;
			break;
		case 0x2:
			*div = 6;
			break;
		case 0x3:
			*div = 1;
			break;
		default:
			break;
		}
	}
	return PRCM_PASS;
}

int prcm_clksel_set_divider(u32 clk, u32 div)
{
	u32 valid, offset, clk_id, new_val, omap, id_type;
	u32 *addr;
	u32 divider = 0;
	u8 reenable_clk = PRCM_FALSE;
	u32 parent_id;

	id_type = get_id_type(clk);
	if (!(id_type & ID_CLK_DIV))
		return PRCM_FAIL;	/*No divider */

	omap = OMAP(clk);

	if (cpu_is_omap3430() && !(omap & (AT_3430|AT_3430_ES2)))
		return PRCM_FAIL;

	if (clk == PRCM_48M_FCLK)
		return PRCM_FAIL;
	else if (clk == PRCM_12M_FCLK)
		return PRCM_FAIL;
	if (clk == PRCM_USIM) {
		prcm_clk_get_source(PRCM_USIM, &parent_id);
		switch (div) {
		case 2:
			if (parent_id == PRCM_DPLL4_M2X2_CLK)
				div = 0x3;
			break;
		case 4:
			if (parent_id == PRCM_DPLL5_M2_CLK)
				div = 0x7;
				break;
		case 8:
			if (parent_id == PRCM_DPLL4_M2X2_CLK)
				div = 0x5;
			break;
		case 10:
			div = 0x6;
			break;
		case 16:
			div = 0x9;
			break;
		case 20:
			div = 0xA;
			break;
		default:
			break;
		}
	}
	if (clk == PRCM_SGX_FCLK) {
		switch (div) {
		case 3:
			div = 0x0;
			break;
		case 4:
			div = 0x1;
			break;
		case 6:
			div = 0x2;
			break;
		case 1:
			div = 0x3;
			break;
		default:
			break;
		}
	}
	clk_id = (clk >> CLK_NO_POS) & CLK_NO_MASK;
	offset = clk_div_offset[clk_id-1];

	if (clk_id > PRCM_NO_OF_CLKS)
		return PRCM_FAIL;

	addr = clksel_reg[clk_id - 1].reg_addr;
	if (!addr)
		return PRCM_FAIL;
	valid = clksel_reg[clk_id - 1].valid_bits;

	/* In case of sysclkout2, the value to be programmed in the register
	   is 0 for divider 1, 1 for divider 2, 2 for divider 4,
	   3 for divider 8 and 4 for divider 16 */
	if (clk == PRCM_SYS_CLKOUT2) {
		if (is_sil_rev_equal_to(OMAP3430_REV_ES1_0)) {
			/* WORKAROUND FOR SILICON ERRATA 1.37 */
			/* Disabling sysclkout2, bit 7 of CM_CLKOUT_CTRL */
			if (*addr&(1 << 7)) {
				*addr &= (~(1 << 7));
				reenable_clk = PRCM_TRUE;
			}
		}

		for (divider = 0; divider <= 4; divider++) {
			if (div & 0x1) {
				div = divider;
				break;
			}
			div >>= 1;
		}
	}
	new_val = (*addr & ~valid) | (div << offset);
	*addr = new_val;

	if (is_sil_rev_equal_to(OMAP3430_REV_ES1_0) && (clk == PRCM_SYS_CLKOUT2)
							 && (reenable_clk)) {
			/* WORKAROUND FOR SILCON ERRATA 1.37 */
			/* Enabling sysclkout2, bit 7 of CM_CLKOUT_CTRL */
			*addr |= (1 << 7);
	}
	return PRCM_PASS;
}

int prcm_get_processor_speed(u32 domainid, u32 *processor_speed)
{
	int ret = PRCM_PASS;
	switch (domainid) {
	case DOM_MPU:
		ret = prcm_get_dpll_rate(PRCM_DPLL1_M2X2_CLK, processor_speed);
		if (ret != PRCM_PASS)
			break;
		else
			/*There is a divider in mpu which makes the actual
			   processor speed half the dpll output */
			*processor_speed = *processor_speed / 2;
		break;
	case PRCM_IVA2:
		ret = prcm_get_dpll_rate(PRCM_DPLL2_M2X2_CLK, processor_speed);
		if (ret != PRCM_PASS)
			break;
		else
			/*There is a divider in iva which makes the actual
			   processor speed half the dpll output */
			*processor_speed = *processor_speed / 2;
		break;
	default:
		ret = PRCM_FAIL;
		break;
	}
	return ret;
}

int prcm_clksel_round_rate(u32 clk, u32 parent_rate, u32 target_rate,
			   u32 *newdiv)
{
	u32 omap, max_div = 6, div;
	u32 index, clk_id, test_rate, dpll_id;
	u32 id_type;
	u32 div_96M[] = {2, 4, 8, 10, 0, 0};
	u32 div_120M[] = {4, 8, 16, 20, 0, 0};
	u32 div_sys[] = {1, 2, 0, 0, 0, 0};
	u32 div_core[] = {3, 4, 6, 0, 0, 0};
	u32 *div_array = NULL;


	*newdiv = 0;
	omap = OMAP(clk);

	if (cpu_is_omap3430() && !(omap & (AT_3430|AT_3430_ES2)))
		return PRCM_FAIL;

	id_type = get_id_type(clk);

	if (is_sil_rev_equal_to(OMAP3430_REV_ES1_0)) {
		/* Workaround for limiation 2.5 */
		/* On ES 1.0, DPLL4 dividers cannot be changed */
		if (id_type & ID_DPLL_OP) {
			dpll_id = (clk >> DPLL_NO_POS) & DPLL_NO_MASK;
			if (dpll_id == DPLL4_PER)
				return PRCM_FAIL;
			/* For DPLL3, divider can only be changed */
						/*through OPP change */
			if (clk == PRCM_DPLL3_M3X2_CLK)
				return PRCM_FAIL;
		}
	}

	if (id_type & ID_DPLL_OP) {
		switch (clk) {
		case PRCM_DPLL3_M3X2_CLK:
			for (index = 0; index < 31; index++) {
				if (!div_dpll[index])
					return PRCM_FAIL;
				test_rate = parent_rate / div_dpll[index];
				if (test_rate <= target_rate) {
					*newdiv = div_dpll[index];
					return PRCM_PASS;
				}
			}
		case PRCM_DPLL5_M2_CLK:
		case PRCM_DPLL4_M2X2_CLK:
		case PRCM_DPLL4_M3X2_CLK:
		case PRCM_DSS:
		case PRCM_CAM:
		case PRCM_DPLL4_M6X2_CLK:
			for (index = 0; index < 16; index++) {
				if (!div_dpll[index]) {
					/* We have hit a NULL element
					 * which means dividers are exhausted */
					return PRCM_FAIL;
				}
				test_rate = parent_rate / div_dpll[index];
				if (test_rate <= target_rate) {
					*newdiv = div_dpll[index];
					return PRCM_PASS;
				}
			}
			break;
		default:
			/* No acceptable divider or divider
			cannot be changed on the fly */
			return PRCM_FAIL;
		}
	}

	if (!(id_type & ID_CLK_DIV))
		return PRCM_FAIL;	/* Clock rate cannot be changed */

	/* For 48M Fclk and 12M Fclk, the dividers are fixed.
	 *So we return the fixed dividers */
	if (clk == PRCM_48M_FCLK) {
		*newdiv = 2;
		return PRCM_PASS;
	}
	if (clk == PRCM_12M_FCLK) {
		*newdiv = 4;
		return PRCM_PASS;
	}
	if (clk == PRCM_USIM) {
		if (parent_rate == 96000000)
			div_array = div_96M;
		else if (parent_rate == 120000000)
			div_array = div_120M;
		else
			div_array = div_sys;
		for (index = 0; index < max_div; index++) {
			if (!(*div_array))
				return PRCM_FAIL;
			test_rate = parent_rate / *div_array;
			if (test_rate <= target_rate) {
				*newdiv = *div_array;
				return PRCM_PASS;
			}
			++div_array;
		}
		return PRCM_FAIL;
	}
	if (clk == PRCM_SGX_FCLK) {
		if (parent_rate == 96000000) {
			*newdiv = 1;
			return PRCM_PASS;
			}
		else {
			div_array = div_core;
			for (index = 0; index < max_div; index++) {
				if (!(*div_array))
					return PRCM_FAIL;
				test_rate = parent_rate / *div_array;
				if (test_rate <= target_rate) {
					*newdiv = *div_array;
					return PRCM_PASS;
				}
				++div_array;
			}
			return PRCM_FAIL;
		}
	}
	clk_id = clk & CLK_NO_MASK;
	if (clk_id > PRCM_NO_OF_CLKS)
		return PRCM_FAIL;

	for (index = 0; index < max_div; index++) {
		if (!div_arr[index]) {
			/* We have hit a NULL element which means
			dividers are exhausted */
			return PRCM_FAIL;
		}
		div = div_arr[(clk_id * max_div) + index];
		test_rate = parent_rate / div;
		if (test_rate <= target_rate) {
			*newdiv = div;
			return PRCM_PASS;
		}
	}
	return PRCM_FAIL;	/*No acceptable divider */
}

int prcm_clk_set_source(u32 clk_id, u32 parent_id)
{
	u32 valid, bit_pos, domain, bit_val, src_bit = 0, omap, id_type;
	u8 ret = PRCM_PASS, result = -1;

	u32 *addr;

	id_type = get_id_type(clk_id);
	if (!(id_type & ID_CLK_SRC))
		return PRCM_FAIL;	/*Rate cannot be selected */

	omap = OMAP(clk_id);

	if (cpu_is_omap3430() && !(omap & (AT_3430|AT_3430_ES2)))
		return PRCM_FAIL;

	switch (clk_id) {
	case PRCM_48M_FCLK:
		/* Check the parent for this clock */
		if (parent_id == PRCM_DPLL4_M2X2_CLK)
			src_bit = 0;
		else if (parent_id == PRCM_SYS_ALT_CLK)
			src_bit = 1;
		else
			return PRCM_FAIL;
		addr = (u32 *)&CM_CLKSEL1_PLL;
		bit_pos = 0x3;
		break;
	case PRCM_USIM:
		addr = (u32 *)&CM_CLKSEL_WKUP;
		valid = *addr & 0xFFFFFF87;
		if (parent_id == PRCM_SYS_CLK) {
			valid |= (0x1 << 3);
			*addr = valid;
			}
		else if (parent_id == PRCM_DPLL4_M2X2_CLK) {
			valid |= (0x3 << 3);
			*addr = valid;
			}
		else if (parent_id == PRCM_DPLL5_M2_CLK) {
			valid |= (0x7 << 3);
			*addr = valid;
		}
		return PRCM_PASS;
		break;
	case PRCM_SGX_FCLK:
		if (parent_id == PRCM_EXT_MCBSP_CLK)
			CM_CLKSEL_SGX = 0x3;
		else if (parent_id == PRCM_DPLL3_M2_CLK)
			CM_CLKSEL_SGX = 0x0; /*Default divider is 3 */
		return PRCM_PASS;
		break;
	case PRCM_96M_CLK:
		if (parent_id == PRCM_DPLL4_M2X2_CLK)
			src_bit = 0;
		else if (parent_id == PRCM_SYS_CLK)
			src_bit = 1;
		addr = (u32 *)&CM_CLKSEL1_PLL;
		bit_pos = 0x6;
		break;
	case PRCM_TVOUT:
		/* Check the parent for this clock */
		if (parent_id == PRCM_DPLL4_M3X2_CLK)
			src_bit = 0;
		else if (parent_id == PRCM_SYS_ALT_CLK)
			src_bit = 1;
		else
			return PRCM_FAIL;
		addr = (u32 *)&CM_CLKSEL1_PLL;
		bit_pos = 0x5;
		break;
	case PRCM_SYS_CLKOUT2:
		/* Check the parent for this clock */
		if (parent_id == PRCM_DPLL3_M2_CLK)
			src_bit = 0;
		else if (parent_id == PRCM_SYS_CLK)
			src_bit = 1;
		else if (parent_id == PRCM_DPLL4_M2X2_CLK)
			src_bit = 2;
		else if (parent_id == PRCM_TVOUT)
			src_bit = 3;
		else
			return PRCM_FAIL;
		addr = (u32 *)&CM_CLKOUT_CTRL;
		bit_pos = 0x0;
		valid = 0x3;
		*addr = (*addr & ~valid) | src_bit;
		/* Returning from here because we have already
		 *updated the register */
		return PRCM_PASS;
		break;
	case PRCM_MCBSP1:
	case PRCM_MCBSP2:
		ret = prcm_is_device_accessible(PRCM_OMAP_CTRL, &result);
		if (ret == PRCM_PASS) {
			if (result == PRCM_FALSE) {
				/*
				 * The device is not accessible.
				 * So enable the interface clock.
				 */
				prcm_clock_control(PRCM_OMAP_CTRL, ICLK,
						   PRCM_ENABLE, PRCM_TRUE);
			}
			bit_pos = CLK_SRC_BIT_POS(clk_id);
			addr = (u32 *)&OMAP2_CONTROL_DEVCONF0;

			if (parent_id == PRCM_DPLL4_M2X2_CLK)
				src_bit = 0;
			else if (parent_id == PRCM_EXT_MCBSP_CLK)
				src_bit = 1;
			else
				return PRCM_FAIL;
		} else
			return ret;
		break;
	case PRCM_MCBSP3:
	case PRCM_MCBSP4:
	case PRCM_MCBSP5:
		ret = prcm_is_device_accessible(PRCM_OMAP_CTRL, &result);
		if (ret == PRCM_PASS) {
			if (result == PRCM_FALSE) {
				/*
				 * The device is not accessible.
				 * So enable the interface clock.
				 */
				prcm_clock_control(PRCM_OMAP_CTRL, ICLK,
						   PRCM_ENABLE, PRCM_TRUE);
			}
			bit_pos = CLK_SRC_BIT_POS(clk_id);
			addr = (u32 *)&OMAP2_CONTROL_DEVCONF1;

			if (parent_id == PRCM_DPLL4_M2X2_CLK)
				src_bit = 0;
			else if (parent_id == PRCM_EXT_MCBSP_CLK)
				src_bit = 1;
			else
				return PRCM_FAIL;
		} else
			return PRCM_FAIL;
		break;
	case PRCM_GPT1:
	case PRCM_GPT2:
	case PRCM_GPT3:
	case PRCM_GPT4:
	case PRCM_GPT5:
	case PRCM_GPT6:
	case PRCM_GPT7:
	case PRCM_GPT8:
	case PRCM_GPT9:
	case PRCM_GPT10:
	case PRCM_GPT11:
		/* Setting clock source for GP timers. */
		if (parent_id == PRCM_SYS_32K_CLK)
			src_bit = 0;
		else if (parent_id == PRCM_SYS_CLK)
			src_bit = 1;
		else
			return PRCM_FAIL;
		bit_pos = CLK_SRC_BIT_POS(clk_id);
		domain = DOMAIN_ID(clk_id);
		addr = get_addr(domain, REG_CLK_SRC);
		if (!addr)
			return PRCM_FAIL;
		break;
	default:
		printk(KERN_INFO "Invalid clock ID in prcm_clk_set_source\n");
		return PRCM_FAIL;
	}
	bit_val = ((1 << bit_pos) & *addr) >> bit_pos;

	if (bit_val && !(src_bit))
		*addr &= (~(1 << bit_pos));
	else if (!(bit_val) && src_bit)
		*addr |= (1 << bit_pos);

	return PRCM_PASS;
}

int prcm_clk_get_source(u32 clk_id, u32 *parent_id)
{
	u32 valid, valid_val, bit_pos, domain, omap, id_type;
	u8 ret = PRCM_PASS, result = -1;
	u32 *addr;

	id_type = get_id_type(clk_id);
	if (!(id_type & ID_CLK_SRC))
		return PRCM_FAIL;	/*Rate cannot be selected */

	omap = OMAP(clk_id);

	if (cpu_is_omap3430() && !(omap & (AT_3430|AT_3430_ES2)))
		return PRCM_FAIL;

	switch (clk_id) {
	case PRCM_48M_FCLK:
		addr = (u32 *)&CM_CLKSEL1_PLL;
		bit_pos = 0x3;
		if (((*addr & (1 << bit_pos)) >> bit_pos) == 0)
			*parent_id = PRCM_DPLL4_M2X2_CLK;
		else
			*parent_id = PRCM_SYS_ALT_CLK;
		break;
	case PRCM_TVOUT:
		addr = (u32 *)&CM_CLKSEL1_PLL;
		bit_pos = 0x5;
		if (((*addr & (1 << bit_pos)) >> bit_pos) == 0)
			*parent_id = PRCM_DPLL4_M3X2_CLK;
		else
			*parent_id = PRCM_SYS_ALT_CLK;
		break;
	case PRCM_USIM:
		addr = (u32 *)&CM_CLKSEL_WKUP;
		bit_pos = 0x3;
		valid = 0x78;
		valid_val = (*addr & valid) >> bit_pos;
		if ((valid_val == 0x1) || (valid_val == 0x2))
			*parent_id = PRCM_SYS_CLK;
		else if ((valid_val >= 0x3) && (valid_val <= 0x6))
			*parent_id = PRCM_DPLL4_M2X2_CLK;
		else if ((valid_val >= 0x7) && (valid_val <= 0xA))
			*parent_id = PRCM_DPLL5_M2_CLK;
		break;
	case PRCM_SGX_FCLK:
		addr = (u32 *)&CM_CLKSEL_SGX;
		bit_pos = 0x0;
		valid = 0x7;
		valid_val = (*addr & valid) >> bit_pos;
		if ((valid_val >= 0x0) && (valid_val <= 0x2))
			*parent_id = PRCM_DPLL3_M2_CLK;
		else
			*parent_id = PRCM_EXT_MCBSP_CLK;
		break;
	case PRCM_96M_CLK:
		addr = (u32 *)&CM_CLKSEL1_PLL;
		bit_pos = 0x6;
		valid_val = *addr & (1<<bit_pos);
		if (valid_val == 0)
			*parent_id = PRCM_DPLL4_M2X2_CLK;
		else
			*parent_id = PRCM_SYS_CLK;
		break;
	case PRCM_SYS_CLKOUT2:
		addr = (u32 *)&CM_CLKOUT_CTRL;
		bit_pos = 0x0;
		valid = 0x3;
		valid_val = *addr & (valid << bit_pos);
		if (valid_val == 0)
			*parent_id = PRCM_DPLL3_M2_CLK;
		else if (valid_val == 1)
			*parent_id = PRCM_SYS_CLK;
		else if (valid_val == 2)
			*parent_id = PRCM_DPLL4_M2X2_CLK;
		else
			*parent_id = PRCM_TVOUT;
		break;
	case PRCM_MCBSP1:
	case PRCM_MCBSP2:
		ret = prcm_is_device_accessible(PRCM_OMAP_CTRL, &result);
		if (ret == PRCM_PASS) {
			if (result == PRCM_FALSE) {
				/*
				 * The device is not accessible.
				 * So enable the interface clock.
				 */
				prcm_clock_control(PRCM_OMAP_CTRL, ICLK,
						   PRCM_ENABLE, PRCM_TRUE);
			}

			else {
				addr = (u32 *)&OMAP2_CONTROL_DEVCONF0;
				bit_pos = CLK_SRC_BIT_POS(clk_id);

				if (((*addr & (1 << bit_pos)) >> bit_pos) == 0)
					*parent_id = PRCM_DPLL4_M2X2_CLK;
				else
					*parent_id = PRCM_EXT_MCBSP_CLK;
			}
		} else
			return PRCM_FAIL;
		break;
	case PRCM_MCBSP3:
	case PRCM_MCBSP4:
	case PRCM_MCBSP5:
		ret = prcm_is_device_accessible(PRCM_OMAP_CTRL, &result);
		if (ret == PRCM_PASS) {
			if (result == PRCM_FALSE) {
				/*
				 * The device is not accessible.
				 * So enable the interface clock.
				 */
				prcm_clock_control(PRCM_OMAP_CTRL, ICLK,
						PRCM_ENABLE, PRCM_TRUE);
			}

			else {
				addr = (u32 *)&OMAP2_CONTROL_DEVCONF1;
				bit_pos = CLK_SRC_BIT_POS(clk_id);

				if (((*addr & (1 << bit_pos)) >> bit_pos) == 0)
					*parent_id = PRCM_DPLL4_M2X2_CLK;
				else
					*parent_id = PRCM_EXT_MCBSP_CLK;
			}
		} else
			return PRCM_FAIL;
		break;
	case PRCM_GPT1:
	case PRCM_GPT2:
	case PRCM_GPT3:
	case PRCM_GPT4:
	case PRCM_GPT5:
	case PRCM_GPT6:
	case PRCM_GPT7:
	case PRCM_GPT8:
	case PRCM_GPT9:
	case PRCM_GPT10:
	case PRCM_GPT11:

		bit_pos = CLK_SRC_BIT_POS(clk_id);
		domain = DOMAIN_ID(clk_id);
		addr = get_addr(domain, REG_CLK_SRC);
		if (!addr)
			return PRCM_FAIL;

		if (((*addr & (1 << bit_pos)) >> bit_pos) == 0)
			*parent_id = PRCM_SYS_32K_CLK;
		else
			*parent_id = PRCM_SYS_CLK;

		break;

	default:
		printk(KERN_INFO "Invalid clock ID in prcm_clk_get_source\n");
		return PRCM_FAIL;
	}
	return PRCM_PASS;
}

int prcm_set_domain_power_configuration(u32 domainid, u8 idlemode,
					u8 standbymode, u8 autoidleenable)
{
	struct sysconf_reg *sysconf = NULL;
	u32 *addr, *fclken_addr, *iclken_addr;
	u32 index = 0, device_id = PRCM_SSI, i_mask = 0;
	u32 ret, loop_cnt = 0, retries_cnt = 0;
	u32     cm_fclken, cm_iclken;

	u32 valid_func_clk = 0, valid_int_clk = 0;
	u32 valid_idlest;
	u32 mask = 0;

	if (domainid > PRCM_NUM_DOMAINS)
		return PRCM_FAIL;

	switch (domainid) {
	case DOM_CORE1:
	case DOM_CORE2:
		/* Save the fclk/iclk registers */
		cm_fclken = CM_FCLKEN1_CORE;
		cm_iclken = CM_ICLKEN1_CORE;
		fclken_addr = (u32 *)&CM_FCLKEN1_CORE;
		iclken_addr = (u32 *)&CM_ICLKEN1_CORE;
		i_mask = CORE1_IMASK;
		sysconf = sysc_reg_core1;
		break;
	case DOM_WKUP:
		cm_fclken = CM_FCLKEN_WKUP;
		cm_iclken = CM_ICLKEN_WKUP;
		fclken_addr = (u32 *)&CM_FCLKEN_WKUP;
		iclken_addr = (u32 *)&CM_ICLKEN_WKUP;
		i_mask = WKUP_IMASK;
		sysconf = sysc_reg_wkup;
		break;
	case DOM_DSS:
		cm_fclken = CM_FCLKEN_DSS;
		cm_iclken = CM_ICLKEN_DSS;
		fclken_addr = (u32 *)&CM_FCLKEN_DSS;
		iclken_addr = (u32 *)&CM_ICLKEN_DSS;
		i_mask = DSS_IMASK;
		sysconf = sysc_reg_dss;
		break;
	case DOM_CAM:
		cm_fclken = CM_FCLKEN_CAM;
		cm_iclken = CM_ICLKEN_CAM;
		fclken_addr = (u32 *)&CM_FCLKEN_CAM;
		iclken_addr = (u32 *)&CM_ICLKEN_CAM;
		i_mask = CAM_IMASK;
		sysconf = sysc_reg_cam;
		break;
	case DOM_PER:
		cm_fclken = CM_FCLKEN_PER;
		cm_iclken = CM_ICLKEN_PER;
		fclken_addr = (u32 *)&CM_FCLKEN_PER;
		iclken_addr = (u32 *)&CM_ICLKEN_PER;
		i_mask = PER_IMASK;
		sysconf = sysc_reg_per;
		break;
	default:
		printk(KERN_INFO "Invalid domainid %d\n", domainid);
		return PRCM_FAIL;
	}
	/* enable Functional and interface clocks */
	valid_func_clk = get_val_bits(domainid, REG_FCLKEN);
	valid_int_clk  = get_val_bits(domainid, REG_ICLKEN);
	valid_idlest   = get_val_bits(domainid, REG_IDLEST);

	prcm_set_domain_functional_clocks(domainid,
		valid_func_clk);
	prcm_set_domain_interface_clocks(domainid,
		valid_int_clk);

	ret = prcm_get_devices_not_idle(domainid, &mask);
	if (ret == PRCM_FAIL) {
		printk(KERN_ERR"Error: Devices not idled in"
			" set_domain_power_configuration\n");
		return ret;
	}
	while ((mask | i_mask) != valid_idlest) {
		prcm_get_devices_not_idle(domainid, &mask);
		ret = loop_wait(&loop_cnt, &retries_cnt, 1000);
		if (ret != PRCM_PASS) {
			printk(KERN_INFO "Loop count exceeded "
			"in set_power_configuration for domain:%u"
			"mask=%x,imask=%x,valid_idlest=%x\n",
			domainid, mask, i_mask, valid_idlest);
			return ret;
		}
	}
	while (device_id != 0) {
		addr = sysconf[index].reg_addr;
		device_id = sysconf[index].device_id;

		/* The end if each array is marked by "0" entries.
		 * Break out of the loop is a NULL address is
		 * encountered.
		 */
		if (!addr)
			break;

		pr_debug("Domain is %d, Address is %p\n", domainid, addr);

		if (standbymode != PRCM_MIDLEMODE_DONTCARE) {
			 /* Setting the MIDLEMODE field */
			if (device_id == PRCM_SSI   || device_id == PRCM_SDMA ||
			device_id == PRCM_HSOTG || device_id == PRCM_DISPC ||
			device_id == PRCM_CAM   || device_id == PRCM_CSIA
			|| device_id == PRCM_CSIB
			) {
				pr_debug("MIDLEMODE SET\n");
				*addr |= (*addr & ~(PRCM_STANDBY_MASK)) |
				(standbymode << PRCM_STANDBY_OFF);
				pr_debug("TYPE %d *ADDR %x\n", INIT_TAR, *addr);
			}
		}
		if (idlemode != PRCM_SIDLEMODE_DONTCARE) {
			if (device_id == PRCM_SDRC && idlemode !=
							 PRCM_SMART_IDLE)
				printk("SDRC: Setting idle mode %u not"
						"supported\n", idlemode);


			/* Setting the SIDLEMODE field */
			if (device_id == PRCM_CAM || device_id == PRCM_HDQ ||
				device_id == PRCM_MPU_INTC || device_id ==
				PRCM_CSIA || device_id == PRCM_CSIB)
				pr_debug("SIDLEMODE cannot be set for %u\n",
								 device_id);
			else
				*addr |= (*addr & ~(PRCM_IDLE_MASK)) |
				(idlemode << PRCM_IDLE_OFF);

		}

		/* Setting the AUTOIDLE ENABLE field */
		if (device_id == PRCM_SDRC   || device_id == PRCM_MCBSP1 ||
		device_id == PRCM_MCBSP2 || device_id == PRCM_MCBSP3 ||
		device_id == PRCM_MCBSP4 || device_id == PRCM_MCBSP5) {
			pr_debug("AUTOIDLEMODE cannot set for %u\n", device_id);
		} else {
			*addr |= (*addr & ~(PRCM_AUTO_IDLE_MASK)) |
			(autoidleenable << PRCM_AUTO_IDLE_OFF);
		}

		index++;
	}

	/* Restore the fclk/iclk registers */
	*fclken_addr = cm_fclken;
	*iclken_addr = cm_iclken;

	return PRCM_PASS;
}

int prcm_do_frequency_scaling(u32 target_opp_id, u32 current_opp_id)
{
	u32 id_type, vdd;
	u32 curr_m, curr_n, tar_m, tar_n, tar_freqsel, tar_m2;
	int curr_opp_no, target_opp_no, index;
	u32 sys_clkspeed;
	unsigned long flags;
	u32 rfr_ctrl = 0, actim_ctrla = 0, actim_ctrlb = 0;

	if (target_opp_id == current_opp_id)
		return PRCM_PASS;

	id_type = get_other_id_type(target_opp_id);
	if (!(id_type & ID_OPP))
		return PRCM_FAIL;

	id_type = get_other_id_type(current_opp_id);
	if (!(id_type & ID_OPP))
		return PRCM_FAIL;

	vdd = get_vdd(target_opp_id);
	if (vdd != get_vdd(current_opp_id))
		return PRCM_FAIL;

	sys_clkspeed = prcm_get_system_clock_speed();
	switch (sys_clkspeed) {
	case (int)(12000):
		index = 0;
		break;
	case (int)(13000):
		index = 1;
		break;
	case (int)(19200):
		index = 2;
		break;
	case (int)(26000):
		index = 3;
		break;
	case (int)(38400):
		index = 4;
		break;
	default:
		return PRCM_FAIL;
	}

	if (vdd == PRCM_VDD1) {
		curr_opp_no = current_opp_id & OPP_NO_MASK;
		target_opp_no = target_opp_id & OPP_NO_MASK;
		curr_m = (mpu_dpll_param[index][curr_opp_no - 1].dpll_m);
		curr_n = (mpu_dpll_param[index][curr_opp_no - 1].dpll_n);
		tar_m = (mpu_dpll_param[index][target_opp_no - 1].dpll_m);
		tar_n = (mpu_dpll_param[index][target_opp_no - 1].dpll_n);

		if ((curr_m != tar_m) || (curr_n != tar_n)) {
			/* M/N need to be changed - so put DPLL in bypass */
			prcm_put_dpll_in_bypass(DPLL1_MPU, LOW_POWER_BYPASS);
			/* Reset M2 divider */
			prcm_configure_dpll_divider(PRCM_DPLL1_M2X2_CLK, 0x1);
			/* Set M,N,Freqsel values */
			tar_freqsel = (mpu_dpll_param[index]
				       [target_opp_no - 1].dpll_freqsel);
			prcm_configure_dpll
			    (DPLL1_MPU, tar_m, tar_n, tar_freqsel);
			/* Lock the DPLL */
			prcm_enable_dpll(DPLL1_MPU);
		}
		/* Configure M2 */
		tar_m2 = (mpu_dpll_param[index][target_opp_no - 1].dpll_m2);
		prcm_configure_dpll_divider(PRCM_DPLL1_M2X2_CLK, tar_m2);

		curr_m = (iva_dpll_param[index][curr_opp_no - 1].dpll_m);
		curr_n = (iva_dpll_param[index][curr_opp_no - 1].dpll_n);
		tar_m = (iva_dpll_param[index][target_opp_no - 1].dpll_m);
		tar_n = (iva_dpll_param[index][target_opp_no - 1].dpll_n);

		if ((curr_m != tar_m) || (curr_n != tar_n)) {
			/* M/N need to be changed - so put DPLL in bypass */
			prcm_put_dpll_in_bypass(DPLL2_IVA2, LOW_POWER_BYPASS);
			/* Reset M2 divider */
			prcm_configure_dpll_divider(PRCM_DPLL2_M2X2_CLK, 0x1);
			/* Set M,N,Freqsel values */
			tar_freqsel = (iva_dpll_param[index]
				       [target_opp_no - 1].dpll_freqsel);
			prcm_configure_dpll
			    (DPLL2_IVA2, tar_m, tar_n, tar_freqsel);
			/* Lock the DPLL */
			prcm_enable_dpll(DPLL2_IVA2);
		}
		/* Configure M2 */
		tar_m2 = (iva_dpll_param[index][target_opp_no - 1].dpll_m2);
		prcm_configure_dpll_divider(PRCM_DPLL2_M2X2_CLK, tar_m2);

		/* Todo: Find out if some recalibation needs to be done
		 * in the OS since MPU freq has been changed */
		}
	else if (vdd == PRCM_VDD2) {
		target_opp_no = target_opp_id & OPP_NO_MASK;
		tar_m = (core_dpll_param[index][target_opp_no - 1].dpll_m);
		tar_n = (core_dpll_param[index][target_opp_no - 1].dpll_n);
		tar_freqsel = (core_dpll_param[index]
					[target_opp_no - 1].dpll_freqsel);
		tar_m2 = (core_dpll_param[index][target_opp_no - 1].dpll_m2);
		/* This function is executed from SRAM */
		local_irq_save(flags);
		omap3_configure_core_dpll(rfr_ctrl, actim_ctrla,
							actim_ctrlb, tar_m2);
		local_irq_restore(flags);
	}
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== GET DOMAIN INTERFACE CLKS  ========================*/
/*============================================================================*/
/*= This function returns the CM_ICLKEN_<domain> register value in result for=*/
/*= the specified domain. The function returns PRCM_FAIL if the ICLKEN       =*/
/*= register is not available for the specified domain, else returns         =*/
/*= PRCM_PASS.                                                               =*/
/*============================================================================*/
int prcm_get_domain_interface_clocks(u32 domainid, u32 *result)
{
	u32 valid;
	u32 *addr;

	*result = 0x0;

	if (domainid > PRCM_NUM_DOMAINS)
		return PRCM_FAIL;

	addr = get_addr(domainid, REG_ICLKEN);
	valid = get_val_bits(domainid, REG_ICLKEN);

	if (!addr)
		return PRCM_FAIL;

	*result = *addr & valid;
	return PRCM_PASS;
}


/*============================================================================*/
/*======================== GET DOMAIN FUNCTIONAL CLKS  =======================*/
/*============================================================================*/
/*= This function returns the CM_FCLKEN_<domain> register in the result for  =*/
/*= the specified domain. The function returns PRCM_FAIL if the FCLKEN       =*/
/* register is not available for the specified domain, else returns          =*/
/*= PRCM_PASS.                                                               =*/
/*============================================================================*/
int prcm_get_domain_functional_clocks(u32 domainid, u32 *result)
{
	u32 valid;
	u32 *addr;

	*result = 0x0;

	if (domainid > PRCM_NUM_DOMAINS)
		return PRCM_FAIL;

	addr = get_addr(domainid, REG_FCLKEN);
	valid = get_val_bits(domainid, REG_FCLKEN);

	if (!addr)
		return PRCM_FAIL;

	*result = *addr & valid;
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== SET DOMAIN INTERFACE CLKS  ========================*/
/*============================================================================*/
/*= This function sets CM_ICLKEN_<domain> register for the specified domain  =*/
/*= with the mask specified in setmask. The function returns PRCM_FAIL if the=*/
/*= ICLKEN register is not available for the specified domain, else returns  =*/
/*= PRCM_PASS.                                                               =*/
/*============================================================================*/
int prcm_set_domain_interface_clocks(u32 domainid, u32 setmask)
{
	u32 valid;
	u32 *addr;

	if (domainid > PRCM_NUM_DOMAINS)
		return PRCM_FAIL;

	addr = get_addr(domainid, REG_ICLKEN);
	valid = get_val_bits(domainid, REG_ICLKEN);

	if (!addr)
		return PRCM_FAIL;

	*addr = setmask & valid;
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== SET DOMAIN FUNCTIONAL CLKS  =======================*/
/*============================================================================*/
/*= This function sets CM_FCLKEN_<domain> register for the specified domain  =*/
/*= with the mask specified in setmask. The function returns PRCM_FAIL if the=*/
/*=  FCLKEN register is not available for the specified domain, else returns =*/
/*= PRCM_PASS.                                                               =*/
/*============================================================================*/
int prcm_set_domain_functional_clocks(u32 domainid, u32 setmask)
{
	u32 valid;
	u32 *addr;

	if (domainid > PRCM_NUM_DOMAINS)
		return PRCM_FAIL;

	addr = get_addr(domainid, REG_FCLKEN);
	valid = get_val_bits(domainid, REG_FCLKEN);

	if (!addr)
		return PRCM_FAIL;

	*addr = setmask & valid;
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== CLOCK DOMAIN STATE  ===============================*/
/*============================================================================*/
/*= This function returns the state of the clock domain. It reads the        =*/
/*= CM_CLKSTST_<DOMAIN> register and returns the result as PRCM_ENABLE if    =*/
/*= the clock domain is active, else returns PRCM_DISABLE  		     =*/
/*= a wake up event.  If disabled, it masks the wake up event from the       =*/
/*= specified module.  This register will modify the PM_WKEN_<DOMAIN>        =*/
/*= register.  This function will return PRCM_FAIL if the parameters passed  =*/
/*= are invalid, otherwise it will return PRCM_PASS.  Valid parameters for   =*/
/*= control are PRCM_ENABLE and PRCM_DISABLE.                                =*/
/*============================================================================*/
int prcm_is_clock_domain_active(u32 domainid, u8 *result)
{
	u32 valid;
	u32 *addr;

	/* Core domain check is not supported */
	if ((domainid == DOM_CORE1) || (domainid == DOM_CORE2)) {
		printk(KERN_INFO "Currently prcm_is_clock_domain_active for "
			"the following domains (DOM_CORE1/DOM_CORE2) "
			"is not supported\n");
		return PRCM_FAIL;
	}

	addr = get_addr(domainid, REG_CLKSTST);
	valid = get_val_bits(domainid, REG_CLKSTST);

	if (!addr)
		return PRCM_FAIL;

	if (*addr & valid) {
		*result = PRCM_ENABLE;
		return PRCM_PASS;
	} else {
		*result = PRCM_DISABLE;
		return PRCM_PASS;
	}
}

/*============================================================================*/
/*======================== CLOCK DOMAIN STATUS ===============================*/
/*============================================================================*/
/* This function waits for the clock domain to transition to the desired state*/
/* It polls on the clock domain state and times out after a wait of ~500 micro*/
/* secs. It returns PRCM_PASS id the clock domain transitions to the desired  */
/* state within the timeout period, else return PRCM_FAIL		      */
/*============================================================================*/
static int prcm_check_clock_domain_status(u32 domainid, u8 desired_state)
{
	u8 curr_state;
	u32 loop_cnt = 0, retries_cnt = 0;
	int ret;

	/* Core domain check is not supported */
	if ((domainid == DOM_CORE1) || (domainid == DOM_CORE2) ||
						(domainid == DOM_MPU)) {
		printk(KERN_INFO "Prcm_check_clock_domain_status is not"
					"supported for the following domains"
					"(DOM_CORE1/DOM_CORE2/DOM_MPU)\n");
		return PRCM_FAIL;
	}

	if (domainid > PRCM_NUM_DOMAINS)
		return PRCM_FAIL;

	ret = prcm_is_clock_domain_active(domainid, &curr_state);
	if (ret != PRCM_PASS)
		return ret;

	while (curr_state != desired_state) {
		ret = prcm_is_clock_domain_active(domainid, &curr_state);
		if (ret != PRCM_PASS)
			return ret;
		ret = loop_wait(&loop_cnt, &retries_cnt, 100);
		if (ret != PRCM_PASS) {
			printk(KERN_INFO "Loop count exceeded in "
				"check_clock_domain_statusfor domain:%u\n",
								domainid);
			return ret;
		}
	}
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== SET CLOCK DOMAIN STATE ============================*/
/*============================================================================*/
/* This function sets the clock domain state to the 'new_state' specified. In */
/* software supervised mode it checks if all the pre-conditions for clock     */
/* domain transition are met. If check_accessibility is set to PRCM_TRUE the  */
/* function also waits fo the clock domain transition to complete             */
/*============================================================================*/
int prcm_set_clock_domain_state(u32 domainid, u8 new_state,
				u8 check_state)
{
	u32 *addr;
	u32 fclk_mask = 0, iclk_mask = 0, init_mask, dev_mask;
	u32 new_val, valid;

	if ((domainid == DOM_CORE1) || (domainid == DOM_CORE2)) {
		printk(KERN_INFO "Currently prcm_set_clock_domain_state "
			"for the following domains "
			"(DOM_CORE1/DOM_CORE2)is not supported\n");
		return PRCM_FAIL;
	}

	addr = get_addr(domainid, REG_CLKSTCTRL);
	valid = get_val_bits(domainid, REG_CLKSTCTRL);
	if (!addr) {
		printk(KERN_INFO "No ClkStCtrl for domain %d\n", domainid);
		return PRCM_FAIL;
	}

	/* It is not appropriate to pass check_state = TRUE for states
	 * PRCM_NO_AUTO and PRCM_HWSUP_AUTO.
	 * In case of PRCM_NO_AUTO, hardware control of clock domain is disabled
	 * In case of PRCM_HWSUP_AUTO, the clock domain will transition
	 * automatically when conditions are satisfied
	 */
	if (check_state == PRCM_TRUE) {
		if ((new_state == PRCM_NO_AUTO) || (new_state ==
							PRCM_HWSUP_AUTO)) {
			printk(KERN_INFO "Cannot wait till clock domain goes"
			"to specified state when target state is: %d\n",
								new_state);
			return PRCM_FAIL;
		}
	}
	/* Check preconditions for SWSUP sleep if check_state = TRUE */
	if (new_state == PRCM_SWSUP_SLEEP) {
		if (domainid == DOM_MPU)
			/* There is no SWSUPERVISED sleep for MPU*/
			return PRCM_FAIL;
		if (check_state == PRCM_TRUE) {
			/* Check if the pre-conditions for the clock domain*/
			/* transition are met */
			/* Check for fclk/iclk to be disabled */
			prcm_get_domain_functional_clocks(domainid, &fclk_mask);
			prcm_get_domain_interface_clocks(domainid, &iclk_mask);
			if (fclk_mask || iclk_mask) {
				printk(KERN_INFO "Pre condition for clock "
					"domain transition not met: Clocks"
					" enabled in the domain Fclk_mask:"
					" %d, Iclk_mask: %d,Domain: %d\n",
					fclk_mask, iclk_mask, domainid);
				return PRCM_FAIL;
			}
			if (domainid != DOM_NEON) {
			/* This check not needed for NEON Check if all*/
			/* Initiators are in standby, and all devices idle */
				prcm_get_initiators_not_standby(domainid,
								&init_mask);
				prcm_get_devices_not_idle(domainid, &dev_mask);
				if (init_mask || dev_mask) {
					printk(KERN_INFO "Pre condition for"
						"clock domain transition not"
						" met:init_mask:%x dev_mask:%x"
						"Domain: %d\n",
						init_mask, dev_mask, domainid);
					return PRCM_FAIL;
				}
			}
		}
	}
	/* Program Clkstctrl register */
	new_val = *addr & ~valid;
	new_val = new_val | new_state;
	*addr = new_val;

	/* NEON has no CLKSTST register */
	if ((check_state == PRCM_TRUE) && (domainid != DOM_NEON)) {
		/* Wait for the Clock domain to transition to the new state */
		if (new_state == PRCM_SWSUP_WKUP)
			return prcm_check_clock_domain_status(domainid,
							      PRCM_ENABLE);

		if (new_state == PRCM_SWSUP_SLEEP)
			return prcm_check_clock_domain_status(domainid,
							      PRCM_DISABLE);

	}
	return PRCM_PASS;
}

/*========================================================================*/
/*===================GET POWER DOMAIN STATE===============================*/
/*= This function returns the current state of a power domain. If the    =*/
/*= power domain is in the middle of a state transition, it waits for the=*/
/*= transition to complete.                                              =*/
/*========================================================================*/
int prcm_get_power_domain_state(u32 domainid, u8 *result)
{
	u32 *addr;
	u32 valid, loop_cnt = 0, retries_cnt = 0;
	int ret;

	/* Core domain check is not supported */
	if ((domainid == DOM_CORE1) || (domainid == DOM_CORE2)) {
		printk(KERN_INFO "Currently prcm_is_clock_domain_active for "
			"the following domains (DOM_CORE1/DOM_CORE2) "
			"is not supported\n");
		return PRCM_FAIL;
	}

	if (domainid > PRCM_NUM_DOMAINS)
		return PRCM_FAIL;

	addr = get_addr(domainid, REG_PWSTST);
	valid = get_val_bits(domainid, REG_PWSTST);

	if (!addr)
		return PRCM_FAIL;

	while (*addr & PWSTST_INTRANS_MASK) {
		ret = loop_wait(&loop_cnt, &retries_cnt, 100);
		if (ret != PRCM_PASS) {
			printk(KERN_INFO "Loop count exceeded in "
			"get_power_domain_state for domain:%u\n", domainid);
			return ret;
		}
	}

	*result = (u8) *addr & PWSTST_PWST_MASK;
	return PRCM_PASS;
}

/*========================================================================*/
/*===============GET PREVIOUS POWER DOMAIN STATE==========================*/
/*= This function returns the previous state of a power domain.          =*/
/*========================================================================*/
int prcm_get_pre_power_domain_state(u32 domainid, u8 *result)
{
	u32 *addr;
	u32 valid;

	if (domainid > PRCM_NUM_DOMAINS)
	return PRCM_FAIL;

	addr = get_addr(domainid, REG_PREPWSTST);
	valid = get_val_bits(domainid, REG_PREPWSTST);

	if (!addr)
	return PRCM_FAIL;

	*result = (u8) *addr & PWSTST_PWST_MASK;
	return PRCM_PASS;
}


/*============================================================================*/
/*======================== POWER DOMAIN STATUS ===============================*/
/*============================================================================*/
/* This function waits for the power domain to transition to the desired state*/
/* It polls on the power domain state and times out after a wait of ~500 micro*/
/* secs. It returns PRCM_PASS id the power domain transitions to the desired  */
/* state within the timeout period, else return PRCM_FAIL                     */
/*============================================================================*/
static int prcm_check_power_domain_status(u32 domainid, u8 desired_state)
{
	u8 curr_state;
	u32 loop_cnt = 0, retries_cnt = 0;
	int ret;

	if ((domainid == DOM_CORE1) || (domainid == DOM_CORE2)\
		|| (domainid == DOM_MPU)) {
		printk
		    (KERN_INFO "prcm_check_power_domain_status is not "
				"supported for the following domains"
				"(DOM_CORE1/DOM_CORE2/DOM_MPU\n");
		return PRCM_FAIL;
	}

	if (domainid > PRCM_NUM_DOMAINS)
		return PRCM_FAIL;

	if (prcm_get_power_domain_state(domainid, &curr_state))
		return PRCM_FAIL;

	while (curr_state != desired_state) {
		ret = prcm_get_power_domain_state(domainid, &curr_state);
		if (ret != PRCM_PASS)
			return ret;
		ret = loop_wait(&loop_cnt, &retries_cnt, 100);
		if (ret != PRCM_PASS) {
			printk(KERN_INFO "Loop count exceeded in "
			"check_power_domain_status for domain:%u\n", domainid);
			printk("curr_state = %x, desired_state = %x\n",
						curr_state, desired_state);
			return ret;
		}
	}
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== SET POWER DOMAIN STATE ============================*/
/*============================================================================*/
/* This function sets the power domain state to the 'new_state' specified. If */
/* mode is 'PRCM_AUTO', the clock domain is programmed in Hardware supervised */
/* mode and the function waits for the power domain to transition to the      */
/* desired state. If mode is 'PRCM_FORCE' the clock domain is programmed in   */
/* sofware supervised mode and the function does not wait for the power       */
/* domain transition to happen.                                               */
/*============================================================================*/
int prcm_set_power_domain_state(u32 domainid, u8 new_state, u8 mode)
{
	u32 *addr;
	u32 *rstst_addr;
	u32 new_val;
	int ret = PRCM_PASS;

	if ((domainid == DOM_CORE1) || (domainid == DOM_CORE2)) {
		printk
		    (KERN_INFO "Currently prcm_set_power_domain_state "
			"for the following domains (DOM_CORE1/DOM_CORE2) "
			"is not supported\n");
		return PRCM_FAIL;
	}

	addr = get_addr(domainid, REG_PWSTCTRL);
	if (!addr)
		return PRCM_FAIL;

	rstst_addr = get_addr(domainid, REG_RSTST);
	if (new_state == PRCM_ON) {
		if (rstst_addr != 0)
			*rstst_addr |= DOM_WKUP_RST;
	}
	if (mode == PRCM_AUTO) {
		/* Set the power domain state to new_state */
		new_val = *addr & ~PWSTST_PWST_MASK;
		new_val = new_val | new_state;
		*addr = new_val;
		ret =
		    prcm_set_clock_domain_state(domainid, PRCM_HWSUP_AUTO,
						PRCM_FALSE);
	} else if (mode == PRCM_FORCE) {
		if (domainid == DOM_MPU)
			return PRCM_FAIL; /*No force mode for MPU */

		new_val = *addr & ~PWSTST_PWST_MASK;
		new_val = new_val | new_state;
		*addr = new_val;
		if ((new_state == PRCM_OFF) || (new_state == PRCM_RET)) {
			ret =
			    prcm_set_clock_domain_state(domainid,
							PRCM_SWSUP_SLEEP,
							PRCM_TRUE);
			if (ret != PRCM_PASS)
				return ret;
		} else {
			ret =
			    prcm_set_clock_domain_state(domainid,
							PRCM_SWSUP_WKUP,
							PRCM_FALSE);
			if (ret != PRCM_PASS)
				return ret;
		}
		/* Wait for the power domain transition to complete */
		ret = prcm_check_power_domain_status(domainid, new_state);
	}
	return ret;
}

/*============================================================================*/
/*======================== DEVICES NOT IDLE  =================================*/
/*============================================================================*/
/*= This function returns a mask of devices that are not idle in the         =*/
/*= specified domain. It reads the CM_IDLEST_<DOMAIN> register to generate   =*/
/*= the mask. Each Bit in the mask which is set to 1, specifies the          =*/
/*= corresponding device is not idle. The function returns PRCM_FAIL if the  =*/
/*= specified device does not have a corresponding IDLEST register.          =*/
/*============================================================================*/
int prcm_get_devices_not_idle(u32 domainid, u32 *result)
{
	u32 valid;
	u32 *addr;

	*result = 0x0;

	if (domainid > PRCM_NUM_DOMAINS)
		return PRCM_FAIL;

	addr = get_addr(domainid, REG_IDLEST);
	valid = get_val_bits(domainid, REG_IDLEST);

	if (!addr)
		return PRCM_FAIL;

	*result = (~(*addr & valid) & valid);
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== INITIATORS NOT STANDBY  ===========================*/
/*============================================================================*/
/*= This function returns a mask of initiators that are not in standby mode. =*/
/*= Each bit in the mask which is set to 1, specifies that the Standby is not=*/
/*= asserted for the corresponding initiator. The function returns PRCM_FAIL =*/
/*= if the specified device does not have a corresponding IDLEST register.   =*/
/*============================================================================*/
int prcm_get_initiators_not_standby(u32 domainid, u32 *result)
{
	u32 valid;
	u32 *addr;

	*result = 0x0;

	if (domainid > PRCM_NUM_DOMAINS)
		return PRCM_FAIL;

	addr = get_addr(domainid, REG_IDLEST);
	valid = get_val_bits(domainid, REG_IDLEST);

	if (!addr)
		return PRCM_FAIL;

	if (prcm_get_devices_not_idle(domainid, result) != PRCM_PASS)
		return PRCM_FAIL;

	switch (domainid) {
	case DOM_IVA2:
		*result &= IVA2_IMASK;
		break;
	case DOM_MPU:
		*result &= MPU_IMASK;
		break;
	case DOM_CORE1:
		*result &= CORE1_IMASK;
		break;
	case DOM_CORE2:
		*result &= CORE2_IMASK;
		break;
	case DOM_SGX:
		*result &= SGX_IMASK;
		break;
	case DOM_USBHOST:
		*result &= USBHOST_IMASK;
		break;
	case DOM_WKUP:
		*result &= WKUP_IMASK;
		break;
	case DOM_DSS:
		*result &= DSS_IMASK;
		break;
	case DOM_CAM:
		*result &= CAM_IMASK;
		break;
	case DOM_PER:
		*result &= PER_IMASK;
		break;
	case DOM_NEON:
		*result &= NEON_IMASK;
		break;
	default:
		break;
	}
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== SET WAKE UP DEPENDENCY  ===========================*/
/*============================================================================*/
/*= This function sets the wake up dependency for a specified power          =*/
/*= domain by accessing the wake up dependendcy register.                    =*/
/*============================================================================*/
int prcm_set_wkup_dependency(u32 domainid, u32 wkup_dep)
{
	u32 *addr;

	switch (domainid) {
	case DOM_IVA2:
		addr = (u32 *)&PM_WKDEP_IVA2;
		*addr |= wkup_dep;
		break;
	case DOM_MPU:
		addr = (u32 *)&PM_WKDEP_MPU;
		*addr |= wkup_dep;
		break;
	case DOM_SGX:
		addr = (u32 *)&PM_WKDEP_SGX;
		*addr |= wkup_dep;
		break;
	case DOM_USBHOST:
		addr = (u32 *)&PM_WKDEP_USBHOST;
		*addr |= wkup_dep;
		break;
	case DOM_DSS:
		addr = (u32 *)&PM_WKDEP_DSS;
		*addr |= wkup_dep;
		break;
	case DOM_CAM:
		addr = (u32 *)&PM_WKDEP_CAM;
		*addr |= wkup_dep;
		break;
	case DOM_PER:
		addr = (u32 *)&PM_WKDEP_PER;
		*addr |= wkup_dep;
		break;
	case DOM_NEON:
		addr = (u32 *)&PM_WKDEP_NEON;
		*addr |= wkup_dep;
		break;
	default:
		printk(KERN_INFO "Wake up dependency does not exist for "
			 "the domain %d\n", domainid);
		return PRCM_FAIL;
	}
	return PRCM_PASS;
}
EXPORT_SYMBOL(prcm_set_wkup_dependency);

/*============================================================================*/
/*======================== CLEAR WAKE UP DEPENDENCY  =========================*/
/*============================================================================*/
/*= This function clears the wake up dependency for a specified power        =*/
/*= domain by accessing the wake up dependendcy register.                    =*/
/*============================================================================*/
int prcm_clear_wkup_dependency(u32 domainid, u32 wkup_dep)
{
	u32 *addr;

	switch (domainid) {
	case DOM_IVA2:
		addr = (u32 *)&PM_WKDEP_IVA2;
		*addr &= ~wkup_dep;
		break;
	case DOM_MPU:
		addr = (u32 *)&PM_WKDEP_MPU;
		*addr &= ~wkup_dep;
		break;
	case DOM_SGX:
		addr = (u32 *)&PM_WKDEP_SGX;
		*addr &= ~wkup_dep;
		break;
	case DOM_USBHOST:
		addr = (u32 *)&PM_WKDEP_USBHOST;
		*addr &= ~wkup_dep;
		break;
	case DOM_DSS:
		addr = (u32 *)&PM_WKDEP_DSS;
		*addr &= ~wkup_dep;
		break;
	case DOM_CAM:
		addr = (u32 *)&PM_WKDEP_CAM;
		*addr &= ~wkup_dep;
		break;
	case DOM_PER:
		addr = (u32 *)&PM_WKDEP_PER;
		*addr &= ~wkup_dep;
		break;
	case DOM_NEON:
		addr = (u32 *)&PM_WKDEP_NEON;
		*addr &= ~wkup_dep;
		break;
	default:
		printk(KERN_INFO "Cannot clear Wake up dependency,"
			"does not exist for the domain %d\n", domainid);
		return PRCM_FAIL;
}
	return PRCM_PASS;
}
EXPORT_SYMBOL(prcm_clear_wkup_dependency);

/*============================================================================*/
/*======================== SET SLEEP DEPENDENCY  =============================*/
/*============================================================================*/
/*= This function sets the sleep dependency for a specified power            =*/
/*= domain by accessing the sleep dependency register.                       =*/
/*============================================================================*/
int prcm_set_sleep_dependency(u32 domainid, u32 sleep_dep)
{
	u32 *addr;

	switch (domainid) {
	case DOM_SGX:
		addr = (u32 *)&CM_SLEEPDEP_SGX;
		*addr |= sleep_dep;
		break;
	case DOM_USBHOST:
		addr = (u32 *)&CM_SLEEPDEP_USBHOST;
		*addr |= sleep_dep;
		break;
	case DOM_DSS:
		addr = (u32 *)&CM_SLEEPDEP_DSS;
		*addr |= sleep_dep;
		break;
	case DOM_CAM:
		addr = (u32 *)&CM_SLEEPDEP_CAM;
		*addr |= sleep_dep;
		break;
	case DOM_PER:
		addr = (u32 *)&CM_SLEEPDEP_PER;
		*addr |= sleep_dep;
		break;
	default:
		printk(KERN_INFO "Sleep dependency does not exist for"
			 "the domain %d\n", domainid);
		return PRCM_FAIL;
	}
	return PRCM_PASS;
}
EXPORT_SYMBOL(prcm_set_sleep_dependency);

/*============================================================================*/
/*======================== CLEAR SLEEP DEPENDENCY  ===========================*/
/*============================================================================*/
/*= This function clears the sleep dependency for a specified power          =*/
/*= domain by accessing the sleep dependency register.                       =*/
/*============================================================================*/
int prcm_clear_sleep_dependency(u32 domainid, u32 sleep_dep)
{
	u32 *addr;

	switch (domainid) {
	case DOM_SGX:
		addr = (u32 *)&CM_SLEEPDEP_SGX;
		*addr &= ~sleep_dep;
		break;
	case DOM_USBHOST:
		addr = (u32 *)&CM_SLEEPDEP_USBHOST;
		*addr &= ~sleep_dep;
		break;
	case DOM_DSS:
		addr = (u32 *)&CM_SLEEPDEP_DSS;
		*addr &= ~sleep_dep;
		break;
	case DOM_CAM:
		addr = (u32 *)&CM_SLEEPDEP_CAM;
		*addr &= ~sleep_dep;
		break;
	case DOM_PER:
		addr = (u32 *)&CM_SLEEPDEP_PER;
		*addr &= ~sleep_dep;
		break;
	default:
		printk(KERN_INFO "Cannot clear Sleep dependency, does"
			 " not exist for this domain %d\n", domainid);
		return PRCM_FAIL;
	}
	return PRCM_PASS;
}
EXPORT_SYMBOL(prcm_clear_sleep_dependency);

/* This API is called by pm_init during bootup */
int prcm_init(void)
{
	u32 valid_bits;

	/* Clear sleep and wakeup dependencies for all domains */
	CM_SLEEPDEP_DSS = 0;
	PM_WKDEP_IVA2 = 0;
	PM_WKDEP_MPU = 0;
	PM_WKDEP_DSS = 0;
	PM_WKDEP_NEON = 0;
	CM_SLEEPDEP_CAM = 0;
	PM_WKDEP_CAM = 0;

#ifdef CONFIG_HW_SUP_TRANS
	PM_WKDEP_PER = 0x1;
	CM_SLEEPDEP_PER = 0x0;
	if (prcm_set_wkup_dependency(DOM_PER,
			PRCM_WKDEP_EN_MPU) != PRCM_PASS) {
		printk(KERN_INFO "Domain %d : wakeup dependency"
			" could not be set\n", DOM_PER);
		return -1;
	}
	if (prcm_set_sleep_dependency(DOM_PER,
			PRCM_SLEEPDEP_EN_MPU) != PRCM_PASS) {
		printk(KERN_INFO "Domain %d : sleep dependency"
			" could not be set\n", DOM_PER);
		return -1;
	}
#endif /* #ifdef CONFIG_HW_SUP_TRANS */
#ifndef CONFIG_HW_SUP_TRANS
	CM_SLEEPDEP_PER = 0;
	PM_WKDEP_PER = 0;
#endif /* #ifndef CONFIG_HW_SUP_TRANS */
	CM_SLEEPDEP_SGX = 0;
	CM_SLEEPDEP_USBHOST = 0;
	PM_WKDEP_SGX = 0;
	PM_WKDEP_USBHOST = 0;

	/* Enable interface clock autoidle for all modules */
	valid_bits = get_val_bits(DOM_CORE1, REG_AUTOIDLE);
	/* Disable Autoidle for HDQ, as it is not OCP compliant */
	CM_AUTOIDLE1_CORE = valid_bits & ~(1 << 22);
	valid_bits = get_val_bits(DOM_CORE2, REG_AUTOIDLE);
	CM_AUTOIDLE2_CORE = valid_bits;
	valid_bits = get_val_bits(DOM_WKUP, REG_AUTOIDLE);
	/* Disable Autoidle for GPT1 - errata 1.4 */
	CM_AUTOIDLE_WKUP = valid_bits & ~(0x1);
	valid_bits = get_val_bits(DOM_DSS, REG_AUTOIDLE);
	CM_AUTOIDLE_DSS = valid_bits;
	valid_bits = get_val_bits(DOM_CAM, REG_AUTOIDLE);
	CM_AUTOIDLE_CAM = valid_bits;
	valid_bits = get_val_bits(DOM_PER, REG_AUTOIDLE);
	CM_AUTOIDLE_PER = valid_bits;
	valid_bits = get_val_bits(DOM_CORE3, REG_AUTOIDLE);
	CM_AUTOIDLE3_CORE = valid_bits;
	valid_bits = get_val_bits(DOM_USBHOST, REG_AUTOIDLE);
	CM_AUTOIDLE_USBHOST = valid_bits;

	PM_MPUGRPSEL1_CORE = 0xC33FFE18;
	PM_IVA2GRPSEL1_CORE = 0x80000008;
	PM_MPUGRPSEL3_CORE = 0x4;
	PM_IVA2GRPSEL3_CORE = 0x0;
	PM_MPUGRPSEL_WKUP = 0x3CB;
	PM_IVA2GRPSEL_WKUP = 0x0;
	PM_MPUGRPSEL_PER = 0x3EFFF;
	PM_IVA2GRPSEL_PER = 0x0;
	PM_WKEN_WKUP = 0x3CB;
	PM_WKEN_DSS = 0x1;
	PM_WKEN_PER = 0x3EFFF;
	PM_WKEN1_CORE = 0xC33FFE18;

	CM_AUTOIDLE_PLL_MPU = 0x1;
	CM_AUTOIDLE_PLL_IVA2 = 0x1;
	CM_AUTOIDLE_PLL = 0x9;   /* always autoidle*/
	CM_AUTOIDLE2_PLL = 0x1;  /* always autoidle*/
	#if 0
	CONTROL_PADCONF_SYS_NIRQ &= 0xFFFFFFF8;
	CONTROL_PADCONF_SYS_NIRQ |= 0x4;
	GPIO1_SETIRQENABLE1 |= 0x1;
	GPIO1_SETWKUENA |= 0x1;
	GPIO1_FALLINGDETECT |= 0x1;
	#endif
	/* Enable OFF mode settings for SYS_NIRQ */
	CONTROL_PADCONF_SYS_NIRQ |= (IO_PAD_WAKEUPENABLE |
				IO_PAD_OFFPULLUDENABLE | IO_PAD_OFFOUTENABLE
				| IO_PAD_OFFENABLE | IO_PAD_INPUTENABLE |
				IO_PAD_PULLUDENABLE);

	/* Enable OFF mode settings for GPIO_2 */
	CONTROL_PADCONF_SYS_NRESWARM |= ((IO_PAD_WAKEUPENABLE |
				IO_PAD_OFFPULLUDENABLE | IO_PAD_OFFOUTENABLE
				| IO_PAD_OFFENABLE | IO_PAD_INPUTENABLE |
				IO_PAD_PULLUDENABLE) << IO_PAD_HIGH_SHIFT);

	/* Disable input for SYS_OFFMODE */
	CONTROL_PADCONF_SYS_OFF_MODE &= ~(IO_PAD_INPUTENABLE);
	/* Set the wakeup proc to 0, some issues seen in DVFS with this set */
	SDRC_PWR &= ~(1<<26);
	SDRC_PWR = 0x111210;
	SDRC_DLL_A_CTRL |= 0x26000000;
	CM_ICLKEN2_CORE |= 4;
	omap_writel(0, 0x480a0048);
	CM_ICLKEN2_CORE &= ~4;
#ifndef CONFIG_DISABLE_EMUDOMAIN_CONTROL
	CM_CLKSTCTRL_EMU = 0x3;
#endif
	/* To be enabled later when device drivers are tested with the
	 * configuration */

	prcm_set_domain_power_configuration(DOM_CORE1, PRCM_SIDLEMODE_DONTCARE,
					PRCM_MIDLEMODE_DONTCARE, PRCM_TRUE);
	prcm_set_domain_power_configuration(DOM_WKUP, PRCM_SIDLEMODE_DONTCARE,
					PRCM_MIDLEMODE_DONTCARE, PRCM_TRUE);
	#if 0
	prcm_set_domain_power_configuration(DOM_DSS, PRCM_SIDLEMODE_DONTCARE,
					PRCM_MIDLEMODE_DONTCARE, PRCM_TRUE);
	#endif
	prcm_set_domain_power_configuration(DOM_CAM, PRCM_SIDLEMODE_DONTCARE,
					PRCM_MIDLEMODE_DONTCARE, PRCM_TRUE);
	prcm_set_domain_power_configuration(DOM_PER, PRCM_SIDLEMODE_DONTCARE,
					PRCM_MIDLEMODE_DONTCARE, PRCM_TRUE);

	if (is_sil_rev_equal_to(OMAP3430_REV_ES1_0) ||
			is_sil_rev_equal_to(OMAP3430_REV_ES2_0)) {
		/* Due to Errata 1.27, IVA2 will not go to ret/off*/
		/* during bootup.We need to manually boot it to  */
		/* idle mode This is only required during bootup and not for*/
		/* subsequent transitions */
		/* Configure IVA to boot in idle mode */
		/* Clear reset of IVA2*/
		/* Disable all IVA clocks */
		CM_FCLKEN_IVA2 = 0x0;
		/* Reset IVA2 */
		RM_RSTCTRL_IVA2 = 0xFFFF;
		/* Enable IVA clocks */
		CM_FCLKEN_IVA2 = 0x1;
		/* Set boot mode to idle */
		CONTROL_IVA2_BOOTMOD = 0x1;
		/* unreset the DSP */
		RM_RSTCTRL_IVA2 = 0x0;
		/* Disable IVA clock */
		CM_FCLKEN_IVA2 = 0x0;
		/* Reset IVA2 */
		RM_RSTCTRL_IVA2 = 0xFFFF;
	}
#ifdef CONFIG_HW_SUP_TRANS
	/* MPU DPLL low power stop mode */
	CM_AUTOIDLE_PLL_MPU = LOW_POWER_STOP;
	/* MPU DPLL low power stop mode */
	CM_AUTOIDLE_PLL_MPU = LOW_POWER_STOP;

	/* L3, L4 ans D2D clock autogating */
	CM_CLKSTCTRL_CORE = (CLK_D2D_HW_SUP_ENABLE | CLK_L4_HW_SUP_ENABLE |
							CLK_L3_HW_SUP_ENABLE);
#endif /*#ifdef CONFIG_HW_SUP_TRANS */

#ifdef IOPAD_WKUP
	init_wakeupconfig();
#endif /* #ifdef IOPAD_WKUP */
	clear_domain_reset_status();
	return PRCM_PASS;
}


int prcm_set_mpu_domain_state(u32 mpu_dom_state)
{
	u32 id_type, value, ret;
	id_type = get_other_id_type(mpu_dom_state);
	if (!(id_type & ID_MPU_DOM_STATE))
		return PRCM_FAIL;

	if (mpu_dom_state > PRCM_MPU_OFF)
		return PRCM_FAIL;

	value = PM_PWSTCTRL_MPU;
	value &= ~PWSTST_PWST_MASK;

	switch (mpu_dom_state) {
	case PRCM_MPU_ACTIVE:
	case PRCM_MPU_INACTIVE:
		value |= 0x3;
		RM_RSTST_MPU |= DOM_WKUP_RST;
		break;
	case PRCM_MPU_CSWR_L2RET:
	case PRCM_MPU_CSWR_L2OFF:
	case PRCM_MPU_OSWR_L2RET:
	case PRCM_MPU_OSWR_L2OFF:
		value |= 0x1;
		break;
	case PRCM_MPU_OFF:
		value |= 0x0;
		break;
	default:
		return PRCM_FAIL;
	}
	PM_PWSTCTRL_MPU = value;
	if ((mpu_dom_state == PRCM_MPU_OSWR_L2RET) ||
			(mpu_dom_state == PRCM_MPU_OSWR_L2OFF))
		PM_PWSTCTRL_MPU &= ~(0x4);
	if ((mpu_dom_state == PRCM_MPU_CSWR_L2OFF) ||
			(mpu_dom_state == PRCM_MPU_OSWR_L2OFF))
		PM_PWSTCTRL_MPU &= ~(0x100);
	if (mpu_dom_state != PRCM_MPU_ACTIVE)
		ret = prcm_set_clock_domain_state(DOM_MPU, PRCM_HWSUP_AUTO,
								PRCM_FALSE);
	else
		ret = prcm_set_clock_domain_state(DOM_MPU, PRCM_NO_AUTO,
								PRCM_FALSE);
	return ret;
}

/* Change state of CORE power domain*/
int prcm_set_core_domain_state(u32 core_dom_state)
{
	u32 id_type, value;
	id_type = get_other_id_type(core_dom_state);
	if (!(id_type & ID_CORE_DOM_STATE))
		return PRCM_FAIL;

	value = PM_PWSTCTRL_CORE;
	value &= ~PWSTST_PWST_MASK;

	switch (core_dom_state) {
	case PRCM_CORE_ACTIVE:
	case PRCM_CORE_INACTIVE:
		value |= PRCM_CORE_PWRSTATEBIT1 | PRCM_CORE_PWRSTATEBIT2;
		RM_RSTST_CORE |= DOM_WKUP_RST;
		RM_RSTST_IVA2 |= COREDOM_WKUP_RST;
		RM_RSTST_MPU |= COREDOM_WKUP_RST;
		RM_RSTST_SGX |= COREDOM_WKUP_RST;
		RM_RSTST_DSS |= COREDOM_WKUP_RST;
		RM_RSTST_CAM |= COREDOM_WKUP_RST;
		RM_RSTST_PER |= COREDOM_WKUP_RST;
		break;

	case PRCM_CORE_CSWR_MEMRET:
		value |= (PRCM_CORE_MEMBIT | PRCM_CORE_LOGICBIT
					| PRCM_CORE_PWRSTATEBIT1);
		value &= ~PRCM_CORE_PWRSTATEBIT2;
		PRM_VOLTCTRL = PRM_VOLTCTRL_AUTO_RET;
#ifdef CONFIG_SYSOFFMODE
		PRM_VOLTCTRL &= ~PRM_VOLTCTRL_SEL_OFF;
#endif
		break;

	case PRCM_CORE_CSWR_MEM1OFF:
		value &= ~(PRCM_CORE_MEMBIT1 | PRCM_CORE_PWRSTATEBIT2);
		value |= PRCM_CORE_PWRSTATEBIT1 | PRCM_CORE_LOGICBIT;
		break;

	case PRCM_CORE_CSWR_MEM2OFF:
		value &= ~(PRCM_CORE_MEMBIT2 | PRCM_CORE_PWRSTATEBIT2);
		value |= PRCM_CORE_PWRSTATEBIT1 | PRCM_CORE_LOGICBIT;
		break;

	case PRCM_CORE_CSWR_MEMOFF:
		value &= ~(PRCM_CORE_MEMBIT | PRCM_CORE_PWRSTATEBIT2);
		value |= PRCM_CORE_PWRSTATEBIT1 | PRCM_CORE_LOGICBIT;
		break;

	case PRCM_CORE_OSWR_MEM1OFF:
		value |= PRCM_CORE_PWRSTATEBIT1;
		value &= ~(PRCM_CORE_MEMBIT1 | PRCM_CORE_LOGICBIT |
						PRCM_CORE_PWRSTATEBIT2);
		break;

	case PRCM_CORE_OSWR_MEMRET:
		value |= (PRCM_CORE_MEMBIT | PRCM_CORE_PWRSTATEBIT1);
		value &= ~(PRCM_CORE_LOGICBIT | PRCM_CORE_PWRSTATEBIT2);
		break;

	case PRCM_CORE_OSWR_MEM2OFF:
		value |= PRCM_CORE_PWRSTATEBIT1;
		value &= ~(PRCM_CORE_MEMBIT2 | PRCM_CORE_LOGICBIT |
					PRCM_CORE_PWRSTATEBIT2);
		break;

	case PRCM_CORE_OSWR_MEMOFF:
		value |= PRCM_CORE_PWRSTATEBIT1;
		value &= ~(PRCM_CORE_MEMBIT | PRCM_CORE_LOGICBIT  |
					PRCM_CORE_PWRSTATEBIT2);
		break;
	case PRCM_CORE_OFF:
		value |= PRCM_CORE_PWRSTATEOFF;
		PRM_VOLTCTRL = PRM_VOLTCTRL_AUTO_OFF;
#ifdef CONFIG_SYSOFFMODE
		PRM_VOLTCTRL |= PRM_VOLTCTRL_SEL_OFF;
#endif
		prcm_save_core_context(PRCM_CORE_OFF);
		break;

	default:
		return PRCM_FAIL;
	}
	PM_PWSTCTRL_CORE = value;
	/* Set DPLLs to go to low power mode automatically */
	CM_AUTOIDLE3_CORE = 0x4;
	/* This should be 0xF for es2 */
	/* However With 0xf CORE ret is prevented */
	CM_CLKSTCTRL_CORE = 0x3F;
	CM_CLKSTCTRL_CORE = 0x3F;
	return PRCM_PASS;
}

static void control_save_context(void)
{
	control_ctx.sysconfig = CONTROL_SYSCONFIG;
	control_ctx.devconf0 = OMAP2_CONTROL_DEVCONF0;
	control_ctx.mem_dftrw0 = CONTROL_MEM_DFTRW0;
	control_ctx.mem_dftrw1 = CONTROL_MEM_DFTRW1;
	control_ctx.msuspendmux_0 = CONTROL_MSUSPENDMUX_0;
	control_ctx.msuspendmux_1 = CONTROL_MSUSPENDMUX_1;
	control_ctx.msuspendmux_2 = CONTROL_MSUSPENDMUX_2;
	control_ctx.msuspendmux_3 = CONTROL_MSUSPENDMUX_3;
	control_ctx.msuspendmux_4 = CONTROL_MSUSPENDMUX_4;
	control_ctx.msuspendmux_5 = CONTROL_MSUSPENDMUX_5;
	control_ctx.sec_ctrl = CONTROL_SEC_CTRL;
	control_ctx.devconf1 = OMAP2_CONTROL_DEVCONF1;
	control_ctx.csirxfe = CONTROL_CSIRXFE;
	control_ctx.iva2_bootaddr = CONTROL_IVA2_BOOTADDR;
	control_ctx.iva2_bootmod = CONTROL_IVA2_BOOTMOD;
	control_ctx.debobs_0 = CONTROL_DEBOBS_0;
	control_ctx.debobs_1 = CONTROL_DEBOBS_1;
	control_ctx.debobs_2 = CONTROL_DEBOBS_2;
	control_ctx.debobs_3 = CONTROL_DEBOBS_3;
	control_ctx.debobs_4 = CONTROL_DEBOBS_4;
	control_ctx.debobs_5 = CONTROL_DEBOBS_5;
	control_ctx.debobs_6 = CONTROL_DEBOBS_6;
	control_ctx.debobs_7 = CONTROL_DEBOBS_7;
	control_ctx.debobs_8 = CONTROL_DEBOBS_8;
	control_ctx.prog_io0 = CONTROL_PROG_IO0;
	control_ctx.prog_io1 = CONTROL_PROG_IO1;
	control_ctx.dss_dpll_spreading = CONTROL_DSS_DPLL_SPREADING;
	control_ctx.core_dpll_spreading = CONTROL_CORE_DPLL_SPREADING;
	control_ctx.per_dpll_spreading = CONTROL_PER_DPLL_SPREADING;
	control_ctx.usbhost_dpll_spreading = CONTROL_USBHOST_DPLL_SPREADING;
	control_ctx.pbias_lite = OMAP2_CONTROL_PBIAS_1;
	control_ctx.temp_sensor = CONTROL_TEMP_SENSOR;
	control_ctx.sramldo4 = CONTROL_SRAMLDO4;
	control_ctx.sramldo5 = CONTROL_SRAMLDO5;
	control_ctx.csi = CONTROL_CSI;
}

static void control_restore_context(void)
{
	CONTROL_SYSCONFIG = control_ctx.sysconfig;
	OMAP2_CONTROL_DEVCONF0 = control_ctx.devconf0;
	CONTROL_MEM_DFTRW0 = control_ctx.mem_dftrw0;
	CONTROL_MEM_DFTRW1 = control_ctx.mem_dftrw1;
	CONTROL_MSUSPENDMUX_0 = control_ctx.msuspendmux_0;
	CONTROL_MSUSPENDMUX_1 = control_ctx.msuspendmux_1;
	CONTROL_MSUSPENDMUX_2 = control_ctx.msuspendmux_2;
	CONTROL_MSUSPENDMUX_3 = control_ctx.msuspendmux_3;
	CONTROL_MSUSPENDMUX_4 = control_ctx.msuspendmux_4;
	CONTROL_MSUSPENDMUX_5 = control_ctx.msuspendmux_5;
	CONTROL_SEC_CTRL = control_ctx.sec_ctrl;
	OMAP2_CONTROL_DEVCONF1 = control_ctx.devconf1;
	CONTROL_CSIRXFE = control_ctx.csirxfe;
	CONTROL_IVA2_BOOTADDR = control_ctx.iva2_bootaddr;
	CONTROL_IVA2_BOOTMOD = control_ctx.iva2_bootmod;
	CONTROL_DEBOBS_0 = control_ctx.debobs_0;
	CONTROL_DEBOBS_1 = control_ctx.debobs_1;
	CONTROL_DEBOBS_2 = control_ctx.debobs_2;
	CONTROL_DEBOBS_3 = control_ctx.debobs_3;
	CONTROL_DEBOBS_4 = control_ctx.debobs_4;
	CONTROL_DEBOBS_5 = control_ctx.debobs_5;
	CONTROL_DEBOBS_6 = control_ctx.debobs_6;
	CONTROL_DEBOBS_7 = control_ctx.debobs_7;
	CONTROL_DEBOBS_8 = control_ctx.debobs_8;
	CONTROL_PROG_IO0 = control_ctx.prog_io0;
	CONTROL_PROG_IO1 = control_ctx.prog_io1;
	CONTROL_DSS_DPLL_SPREADING = control_ctx.dss_dpll_spreading;
	CONTROL_CORE_DPLL_SPREADING = control_ctx.core_dpll_spreading;
	CONTROL_PER_DPLL_SPREADING = control_ctx.per_dpll_spreading;
	CONTROL_USBHOST_DPLL_SPREADING = control_ctx.usbhost_dpll_spreading;
	OMAP2_CONTROL_PBIAS_1 = control_ctx.pbias_lite;
	CONTROL_TEMP_SENSOR = control_ctx.temp_sensor;
	CONTROL_SRAMLDO4 = control_ctx.sramldo4;
	CONTROL_SRAMLDO5 = control_ctx.sramldo5;
	CONTROL_CSI = control_ctx.csi;
}

static void save_intc_context(void)
{
	int i = 0;
	intc_context.sysconfig = INTCPS_SYSCONFIG;
	intc_context.protection = INTCPS_PROTECTION;
	intc_context.idle = INTCPS_IDLE;
	intc_context.threshold = INTCPS_THRESHOLD;
	for (i = 0; i < 96; i++)
		intc_context.ilr[i] = IC_REG32_34XX(0x100 + 0x4*i);
	/* MIRs are saved and restore with other PRCM registers */
}

static void restore_intc_context(void)
{
	int i = 0;
	INTCPS_SYSCONFIG = intc_context.sysconfig;
	INTCPS_PROTECTION = intc_context.protection;
	INTCPS_IDLE = intc_context.idle;
	INTCPS_THRESHOLD = intc_context.threshold;
	for (i = 0; i < 96; i++)
		IC_REG32_34XX(0x100 + 0x4*i) = intc_context.ilr[i];
	/* MIRs are saved and restore with other PRCM registers */
}

void prcm_save_core_context(u32 target_core_state)
{
	if (target_core_state == PRCM_CORE_OFF) {
		if (!padconf_saved) {
			/* Start save of PADCONF registers */
			CONTROL_PADCONF_OFF |= 0x2;
			/* Wait till save is done */
			while (CONTROL_GENERAL_PURPOSE_STATUS & 0x1);
			padconf_saved = 1;
		}

		/* Save interrupt controller context */
		save_intc_context();
		/* Save gpmc context */
		gpmc_save_context();
		/* Save control module context */
		control_save_context();
	}

	if ((target_core_state  >= PRCM_CORE_OSWR_MEMRET) &&
				(target_core_state != PRCM_CORE_OFF)) {
		/* Save interrupt controller context */
		save_intc_context();
		/* Save gpmc context */
		gpmc_save_context();
	}
	/* Whenever we need to call rom code API to save secure ram,
	* save dma context also */
	if ((target_core_state  > PRCM_CORE_CSWR_MEMRET) &&
		(target_core_state != PRCM_CORE_OSWR_MEMRET)) {
		/* Save dma context */
		omap_dma_global_context_save();
	}
}

void prcm_restore_core_context(u32 target_core_state)
{
	u8 state;
	if (target_core_state == PRCM_CORE_OFF) {
		prcm_get_pre_power_domain_state(DOM_CORE1, &state);
		if (state == PRCM_OFF) {
			restore_intc_context();
			gpmc_restore_context();
			control_restore_context();
			padconf_saved = 0;
			uart_padconf_control();
			/* TODO prcm_set_domain_power_configuration(DOM_CORE1,
				PRCM_SIDLEMODE_DONTCARE,
				PRCM_MIDLEMODE_DONTCARE, PRCM_TRUE);*/
			/* Since PER is also handled along with CORE
			* enable autoidle for PER also
			*/
			/* TODO prcm_set_domain_power_configuration(DOM_PER,
				PRCM_SIDLEMODE_DONTCARE,
				PRCM_MIDLEMODE_DONTCARE, PRCM_TRUE);*/
			/* Lock DPLL5 */
			prcm_configure_dpll(DPLL5_PER2, -1, -1, -1);
			prcm_enable_dpll(DPLL5_PER2);
		}
	}

	 if ((target_core_state >= PRCM_CORE_OSWR_MEMRET) &&
		(target_core_state != PRCM_CORE_OFF)) {
			restore_intc_context();
			gpmc_restore_context();
		}
	/* Whenever we called rom code API, restore dma context also */
	if ((target_core_state  > PRCM_CORE_CSWR_MEMRET) &&
		(target_core_state != PRCM_CORE_OSWR_MEMRET)) {
		if (!is_device_type_gp()) {
			/* Clear irq status for DMA channels 0 and 1 */
			/* This needs to be done after calling after
			*saving/restoring secure ram context
			*/
			omap_clear_dma(0);
			omap_clear_dma(1);
		}
		omap_dma_global_context_restore();
	}
}

/* Save registers */
int prcm_save_registers(struct system_power_state *target_state)
{
	PRCM_SAVE(INTC_MIR_0);
	PRCM_SAVE(INTC_MIR_1);
	PRCM_SAVE(INTC_MIR_2);
	PRCM_SAVE(CONTROL_PADCONF_SYS_NIRQ);
	PRCM_SAVE(GPIO1_IRQENABLE1);
	PRCM_SAVE(GPIO1_WAKEUPENABLE);
	PRCM_SAVE(GPIO1_FALLINGDETECT);

	PRCM_SAVE(CM_CLKSEL2_PLL_IVA2);
	PRCM_SAVE(CM_SYSCONFIG);

	PRCM_SAVE(CM_CLKSEL_SGX);
	PRCM_SAVE(CM_CLKSEL_WKUP);
	PRCM_SAVE(CM_CLKSEL_DSS);
	PRCM_SAVE(CM_CLKSEL_CAM);
	PRCM_SAVE(CM_CLKSEL_PER);
	PRCM_SAVE(CM_CLKSEL1_EMU);
	PRCM_SAVE(CM_CLKSTCTRL_EMU);

	PRCM_SAVE(CM_AUTOIDLE2_PLL);
	PRCM_SAVE(CM_CLKSEL5_PLL);
	PRCM_SAVE(CM_POLCTRL);

	PRCM_SAVE(CM_FCLKEN_IVA2);
	PRCM_SAVE(CM_FCLKEN1_CORE);
	PRCM_SAVE(CM_FCLKEN3_CORE);
	PRCM_SAVE(CM_FCLKEN_SGX);
	PRCM_SAVE(CM_FCLKEN_WKUP);
	PRCM_SAVE(CM_FCLKEN_DSS);
	PRCM_SAVE(CM_FCLKEN_CAM);
	PRCM_SAVE(CM_FCLKEN_PER);

	PRCM_SAVE(CM_FCLKEN_USBHOST);
	PRCM_SAVE(CM_ICLKEN1_CORE);
	PRCM_SAVE(CM_ICLKEN2_CORE);
	PRCM_SAVE(CM_ICLKEN3_CORE);
	PRCM_SAVE(CM_ICLKEN_SGX);
	PRCM_SAVE(CM_ICLKEN_WKUP);
	PRCM_SAVE(CM_ICLKEN_DSS);
	PRCM_SAVE(CM_ICLKEN_CAM);
	PRCM_SAVE(CM_ICLKEN_PER);
	PRCM_SAVE(CM_ICLKEN_USBHOST);
	PRCM_SAVE(CM_AUTOIDLE_PLL_IVA2);
	PRCM_SAVE(CM_AUTOIDLE_PLL_MPU);
	PRCM_SAVE(CM_AUTOIDLE_PLL);

	PRCM_SAVE(CM_CLKSTCTRL_IVA2);
	PRCM_SAVE(CM_CLKSTCTRL_MPU);
	PRCM_SAVE(CM_CLKSTCTRL_CORE);
	PRCM_SAVE(CM_CLKSTCTRL_SGX);
	PRCM_SAVE(CM_CLKSTCTRL_DSS);
	PRCM_SAVE(CM_CLKSTCTRL_CAM);
	PRCM_SAVE(CM_CLKSTCTRL_PER);
	PRCM_SAVE(CM_CLKSTCTRL_NEON);
	PRCM_SAVE(CM_CLKSTCTRL_USBHOST);
	PRCM_SAVE(CM_AUTOIDLE1_CORE);
	PRCM_SAVE(CM_AUTOIDLE2_CORE);
	PRCM_SAVE(CM_AUTOIDLE3_CORE);
	PRCM_SAVE(CM_AUTOIDLE_WKUP);
	PRCM_SAVE(CM_AUTOIDLE_DSS);
	PRCM_SAVE(CM_AUTOIDLE_CAM);
	PRCM_SAVE(CM_AUTOIDLE_PER);
	PRCM_SAVE(CM_AUTOIDLE_USBHOST);
	PRCM_SAVE(CM_SLEEPDEP_SGX);
	PRCM_SAVE(CM_SLEEPDEP_DSS);
	PRCM_SAVE(CM_SLEEPDEP_CAM);
	PRCM_SAVE(CM_SLEEPDEP_PER);
	PRCM_SAVE(CM_SLEEPDEP_USBHOST);
	PRCM_SAVE(CM_CLKOUT_CTRL);
	PRCM_SAVE(PRM_CLKOUT_CTRL);

	PRCM_SAVE(PM_WKDEP_IVA2);
	PRCM_SAVE(PM_WKDEP_MPU);
	PRCM_SAVE(PM_WKDEP_SGX);
	PRCM_SAVE(PM_WKDEP_DSS);
	PRCM_SAVE(PM_WKDEP_CAM);
	PRCM_SAVE(PM_WKDEP_PER);
	PRCM_SAVE(PM_WKDEP_NEON);
	PRCM_SAVE(PM_WKDEP_USBHOST);
	PRCM_SAVE(PM_MPUGRPSEL1_CORE);
	PRCM_SAVE(PM_IVA2GRPSEL1_CORE);
	PRCM_SAVE(PM_IVA2GRPSEL3_CORE);
	PRCM_SAVE(PM_MPUGRPSEL3_CORE);
	PRCM_SAVE(PM_MPUGRPSEL_WKUP);
	PRCM_SAVE(PM_IVA2GRPSEL_WKUP);
	PRCM_SAVE(PM_MPUGRPSEL_PER);
	PRCM_SAVE(PM_IVA2GRPSEL_PER);
	PRCM_SAVE(PM_MPUGRPSEL_USBHOST);
	PRCM_SAVE(PM_IVA2GRPSEL_USBHOST);
	PRCM_SAVE(PM_WKEN_WKUP);
	return PRCM_PASS;
}

/* Restore registers */
int prcm_restore_registers(struct system_power_state *target_state)
{
	PRCM_RESTORE(INTC_MIR_0);
	PRCM_RESTORE(INTC_MIR_1);
	PRCM_RESTORE(INTC_MIR_2);
	PRCM_RESTORE(CONTROL_PADCONF_SYS_NIRQ);
	PRCM_RESTORE(GPIO1_IRQENABLE1);
	PRCM_RESTORE(GPIO1_WAKEUPENABLE);
	PRCM_RESTORE(GPIO1_FALLINGDETECT);

	PRCM_RESTORE(CM_CLKSEL2_PLL_IVA2);
	PRCM_RESTORE(CM_SYSCONFIG);

	PRCM_RESTORE(CM_CLKSEL_SGX);
	PRCM_RESTORE(CM_CLKSEL_WKUP);
	PRCM_RESTORE(CM_CLKSEL_DSS);
	PRCM_RESTORE(CM_CLKSEL_CAM);
	PRCM_RESTORE(CM_CLKSEL_PER);
	PRCM_RESTORE(CM_CLKSEL1_EMU);
	PRCM_RESTORE(CM_CLKSTCTRL_EMU);

	PRCM_RESTORE(CM_AUTOIDLE2_PLL);
	PRCM_RESTORE(CM_CLKSEL5_PLL);
	PRCM_RESTORE(CM_POLCTRL);

	PRCM_RESTORE(CM_FCLKEN_IVA2);
	PRCM_RESTORE(CM_FCLKEN1_CORE);
	PRCM_RESTORE(CM_FCLKEN3_CORE);
	PRCM_RESTORE(CM_FCLKEN_SGX);
	PRCM_RESTORE(CM_FCLKEN_WKUP);
	PRCM_RESTORE(CM_FCLKEN_DSS);
	PRCM_RESTORE(CM_FCLKEN_CAM);
	PRCM_RESTORE(CM_FCLKEN_PER);

	PRCM_RESTORE(CM_FCLKEN_USBHOST);
	PRCM_RESTORE(CM_ICLKEN1_CORE);
	PRCM_RESTORE(CM_ICLKEN2_CORE);
	PRCM_RESTORE(CM_ICLKEN3_CORE);
	PRCM_RESTORE(CM_ICLKEN_SGX);
	PRCM_RESTORE(CM_ICLKEN_WKUP);
	PRCM_RESTORE(CM_ICLKEN_DSS);
	PRCM_RESTORE(CM_ICLKEN_CAM);
	PRCM_RESTORE(CM_ICLKEN_PER);
	PRCM_RESTORE(CM_ICLKEN_USBHOST);
	PRCM_RESTORE(CM_AUTOIDLE_PLL_IVA2);
	PRCM_RESTORE(CM_AUTOIDLE_PLL_MPU);
	PRCM_RESTORE(CM_AUTOIDLE_PLL);

	PRCM_RESTORE(CM_CLKSTCTRL_IVA2);
	PRCM_RESTORE(CM_CLKSTCTRL_MPU);
	PRCM_RESTORE(CM_CLKSTCTRL_CORE);
	PRCM_RESTORE(CM_CLKSTCTRL_SGX);
	PRCM_RESTORE(CM_CLKSTCTRL_DSS);
	PRCM_RESTORE(CM_CLKSTCTRL_CAM);
	PRCM_RESTORE(CM_CLKSTCTRL_PER);
	PRCM_RESTORE(CM_CLKSTCTRL_NEON);
	PRCM_RESTORE(CM_CLKSTCTRL_USBHOST);
	PRCM_RESTORE(CM_AUTOIDLE1_CORE);
	PRCM_RESTORE(CM_AUTOIDLE2_CORE);
	PRCM_RESTORE(CM_AUTOIDLE3_CORE);
	PRCM_RESTORE(CM_AUTOIDLE_WKUP);
	PRCM_RESTORE(CM_AUTOIDLE_DSS);
	PRCM_RESTORE(CM_AUTOIDLE_CAM);
	PRCM_RESTORE(CM_AUTOIDLE_PER);
	PRCM_RESTORE(CM_AUTOIDLE_USBHOST);
	PRCM_RESTORE(CM_SLEEPDEP_SGX);
	PRCM_RESTORE(CM_SLEEPDEP_DSS);
	PRCM_RESTORE(CM_SLEEPDEP_CAM);
	PRCM_RESTORE(CM_SLEEPDEP_PER);
	PRCM_RESTORE(CM_SLEEPDEP_USBHOST);
	PRCM_RESTORE(CM_CLKOUT_CTRL);
	PRCM_RESTORE(PRM_CLKOUT_CTRL);

	PRCM_RESTORE(PM_WKDEP_IVA2);
	PRCM_RESTORE(PM_WKDEP_MPU);
	PRCM_RESTORE(PM_WKDEP_SGX);
	PRCM_RESTORE(PM_WKDEP_DSS);
	PRCM_RESTORE(PM_WKDEP_CAM);
	PRCM_RESTORE(PM_WKDEP_PER);
	PRCM_RESTORE(PM_WKDEP_NEON);
	PRCM_RESTORE(PM_WKDEP_USBHOST);
	PRCM_RESTORE(PM_MPUGRPSEL1_CORE);
	PRCM_RESTORE(PM_IVA2GRPSEL1_CORE);
	PRCM_RESTORE(PM_IVA2GRPSEL3_CORE);
	PRCM_RESTORE(PM_MPUGRPSEL3_CORE);
	PRCM_RESTORE(PM_MPUGRPSEL_WKUP);
	PRCM_RESTORE(PM_IVA2GRPSEL_WKUP);
	PRCM_RESTORE(PM_MPUGRPSEL_PER);
	PRCM_RESTORE(PM_IVA2GRPSEL_PER);
	PRCM_RESTORE(PM_MPUGRPSEL_USBHOST);
	PRCM_RESTORE(PM_IVA2GRPSEL_USBHOST);
	PRCM_RESTORE(PM_WKEN_WKUP);
	return PRCM_PASS;
}

int prcm_set_memory_resource_on_state(unsigned short state)
{
	u32 value;
	value = PM_PWSTCTRL_CORE;
	switch (state) {
	case MEMORY_ON:
		value |= PRCM_CORE_PWRSTATEBIT1 | PRCM_CORE_PWRSTATEBIT2;
		value |= PRCM_CORE_MEM1ONBITS | PRCM_CORE_MEM2ONBITS;
		break;
	case MEMORY_OFF:
		value |= PRCM_CORE_PWRSTATEBIT1 | PRCM_CORE_PWRSTATEBIT2;
		value &= ~(PRCM_CORE_MEM1ONBITS | PRCM_CORE_MEM2ONBITS);
		break;
	default:
		pr_debug("Resource State Not Supported");
		return -1;
	}
	PM_PWSTCTRL_CORE = value;
	return 0;
}

/* API called to lock iva dpll after wakeup from core off */
int prcm_lock_iva_dpll(u32 target_opp_id)
{
	u32 tar_m, tar_n, tar_freqsel, sys_clkspeed;
	int index, target_opp_no;
	target_opp_no = target_opp_id & OPP_NO_MASK;
	sys_clkspeed = prcm_get_system_clock_speed();
	switch (sys_clkspeed) {
	case (int)(12000):
		index = 0;
		break;
	case (int)(13000):
		index = 1;
		break;
	case (int)(19200):
		index = 2;
		break;
	case (int)(26000):
		index = 3;
		break;
	case (int)(38400):
		index = 4;
		break;
	default:
		return PRCM_FAIL;
	}
	tar_m = (iva_dpll_param[index][target_opp_no - 1].dpll_m);
	tar_n = (iva_dpll_param[index][target_opp_no - 1].dpll_n);
	/* M/N need to be changed - so put DPLL in bypass */
	prcm_put_dpll_in_bypass(DPLL2_IVA2, LOW_POWER_BYPASS);
	/* Reset M2 divider */
	prcm_configure_dpll_divider(PRCM_DPLL2_M2X2_CLK, 0x1);
	/* Set M,N,Freqsel values */
	tar_freqsel = (iva_dpll_param[index]
		[target_opp_no - 1].dpll_freqsel);
	prcm_configure_dpll
		(DPLL2_IVA2, tar_m, tar_n, tar_freqsel);
	/* Lock the DPLL */
	prcm_enable_dpll(DPLL2_IVA2);
	return PRCM_PASS;
}

int prcm_force_power_domain_state(u32 domain, u8 state)
{
	int ret;

	if ((domain == DOM_CORE1) || (domain == DOM_CORE2) ||
					(domain == DOM_MPU)) {
		printk
		(KERN_INFO "Currently prcm_force_power_domain_state is not "
			"supported for CORE and MPU domains\n");
			return PRCM_FAIL;
	}

	if (state == PRCM_ON) {
		ret = prcm_set_power_domain_state(domain, PRCM_ON, PRCM_FORCE);
		if (ret != PRCM_PASS)
			return ret;
	}

	if ((state == PRCM_RET) || (state == PRCM_OFF)) {
		/* Force all clocks in the power domain to be off */
		prcm_set_domain_interface_clocks(domain, 0x0);
		prcm_set_domain_functional_clocks(domain, 0x0);

		ret = prcm_set_power_domain_state(domain, state, PRCM_FORCE);
		if (ret != PRCM_PASS)
			return ret;
	}

	if (state == PRCM_INACTIVE) {
		/* Force all clocks in the power domain to be off */
		prcm_set_domain_interface_clocks(domain, 0x0);
		prcm_set_domain_functional_clocks(domain, 0x0);
		ret = prcm_set_clock_domain_state(domain, PRCM_HWSUP_AUTO,
								PRCM_FALSE);
		if (ret != PRCM_PASS)
			return ret;
	}
	return PRCM_PASS;
}

/* Check parameters for prcm_set_chip_power_mode() */
static int check_power_mode_parameters(struct system_power_state *state,
						u32 wakeup_source)
{
	u32 id_type, core_inactive_allowed = 1;
	id_type = get_other_id_type(state->mpu_state);
	if (!(id_type & ID_MPU_DOM_STATE)) {
		printk(KERN_INFO "Invalid parameter for mpu state\n");
		return PRCM_FAIL;
	}

	id_type = get_other_id_type(state->core_state);
	if (!(id_type & ID_CORE_DOM_STATE)) {
		printk(KERN_INFO "Invalid parameter for core state\n");
		return PRCM_FAIL;
	}

	if (state->mpu_state > PRCM_MPU_OFF) {
		printk(KERN_INFO "Unsupported state for mpu:%d\n",
							 state->mpu_state);
		return PRCM_FAIL;
	}

	if ((state->iva2_state > PRCM_RET) || (state->gfx_state > PRCM_RET) ||
		(state->dss_state > PRCM_RET) || (state->cam_state > PRCM_RET)
		|| (state->per_state > PRCM_RET) || (state->neon_state >
		PRCM_RET))
		core_inactive_allowed = 0;

	if ((!core_inactive_allowed) && (state->core_state >
							PRCM_CORE_ACTIVE)) {
		printk(KERN_INFO "Invalid combination of states: Core has to"
								"be active\n");
		return PRCM_FAIL;
	}

	if ((!(wakeup_source &  PRCM_WAKEUP_T2_KEYPAD)) && (!(wakeup_source &
						PRCM_WAKEUP_TOUCHSCREEN))) {
		printk(KERN_INFO "Invalid wakeup source\n");
		return PRCM_FAIL;
	}
	return PRCM_PASS;
}

/* Clear out any status which may gate sleep */
void clear_domain_reset_status(void)
{
	u32 tmp;

	RM_RSTST_IVA2  = RM_RSTST_IVA2;
	RM_RSTST_MPU  = RM_RSTST_MPU;
	RM_RSTST_CORE = RM_RSTST_CORE;
	RM_RSTST_SGX = RM_RSTST_SGX;
	RM_RSTST_DSS = RM_RSTST_DSS;
	RM_RSTST_CAM = RM_RSTST_CAM;
	RM_RSTST_PER = RM_RSTST_PER;
	RM_RSTST_EMU = RM_RSTST_EMU;
	PRM_RSTST  = PRM_RSTST;
	RM_RSTST_NEON = RM_RSTST_NEON;
	RM_RSTST_USBHOST = RM_RSTST_USBHOST;
	PRM_IRQSTATUS_IVA2 = PRM_IRQSTATUS_IVA2;
	PRM_IRQSTATUS_MPU = PRM_IRQSTATUS_MPU;
	PM_WKST1_CORE  = PM_WKST1_CORE;
	PM_WKST3_CORE = PM_WKST3_CORE;
	PM_WKST_WKUP = PM_WKST_WKUP;
	PM_WKST_PER  = PM_WKST_PER;
	PM_WKST_USBHOST = PM_WKST_USBHOST;
}

static int prcm_prepare_mpu_core_domains(struct system_power_state *state,
						u32 wakeup_source)
{
	int initiators = 0;
	if (state->mpu_state > PRCM_MPU_ACTIVE) {
		/*TODO omap3_save_secure_ram_context(state->core_state);*/
		prcm_set_mpu_domain_state(state->mpu_state);
		CM_AUTOIDLE_PLL_MPU = 0x1;
	}
	if (state->core_state > PRCM_CORE_ACTIVE) {
		/* Mask interrupts in MIR registers */
		/* Interrupts that are required to wakeup will be enabled in
		 * setup_board_wakeup_source function
		 */
		INTC_MIR_SET_0 = 0xffffffff;
		INTC_MIR_SET_1 = 0xffffffff;
		INTC_MIR_SET_2 = 0xffffffff;
		GPIO1_IRQSTATUS1 = 0xffffffff;
		GPIO1_IRQSTATUS2 = 0xffffffff;
		/* Make grpsel registers 0 */
		/* Required ones will be set in setup_board_wakeup_source() */
		PM_MPUGRPSEL1_CORE = 0xC33FFE18;
		PM_IVA2GRPSEL1_CORE = 0x80000008;
		PM_MPUGRPSEL_PER = 0x3EFFF;
		PM_MPUGRPSEL_WKUP = 0x0;
		PM_IVA2GRPSEL_WKUP = 0x0;
		PM_MPUGRPSEL_PER = 0x0;
		PM_IVA2GRPSEL_PER = 0x0;
		PM_MPUGRPSEL_USBHOST = 0x0;             /* es2 */
		PM_IVA2GRPSEL_USBHOST = 0x0;            /* es2 */
		setup_board_wakeup_source(wakeup_source);
		/* Disable functional clocks in core and wakeup domains*/
		/* Enable interface clock autoidle */
		CM_FCLKEN1_CORE = 0x0;
		CM_AUTOIDLE1_CORE = 0x7FFFFED1;
		CM_AUTOIDLE2_CORE = 0x1f;
		CM_FCLKEN_WKUP &= 0x9;
		CM_AUTOIDLE_WKUP = 0x23f;
		/* Enable SRonIdleReq in SDRC_PWR register */
		SDRC_PWR |= 0x40;
		/* Set DPLLs to go to low power mode automatically */
		CM_AUTOIDLE_PLL_IVA2 = 0x1;
		CM_AUTOIDLE_PLL = 0x9;
		PRM_CLKSRC_CTRL &= ~0x18;
		PRM_CLKSRC_CTRL |= 0x8; /* set sysclk to stop */
		/* Disable HSUSB ICLK explicitly */
		CM_ICLKEN1_CORE &= ~0x10;
		CM_FCLKEN3_CORE = 0x0;
		CM_AUTOIDLE3_CORE = 0x4;
		CM_AUTOIDLE2_PLL = 0x1;
		prcm_set_core_domain_state(state->core_state);
		prcm_get_initiators_not_standby(DOM_CORE1, &initiators);
		if (initiators) {
#ifdef CONFIG_OMAP_LL_DEBUG_UART3
			void enable_debug_uart_per(struct system_power_state
								*state);
			enable_debug_uart_per(state);
#endif
			printk(KERN_INFO "Initiators still active in core"
						"domain : %x\n", initiators);
			return PRCM_FAIL;
		}
		clear_domain_reset_status();
	}
	return PRCM_PASS;
}

static void prcm_restore_mpu_core_domains(struct system_power_state *state,
						u32 wakeup_source)
{
	if (state->mpu_state > PRCM_MPU_ACTIVE)
		prcm_set_mpu_domain_state(PRCM_MPU_ACTIVE);

	if (state->core_state > PRCM_CORE_ACTIVE) {
		prcm_set_core_domain_state(PRCM_CORE_ACTIVE);
		PRM_CLKSRC_CTRL &= ~0x18;
		/* Errata 1.4
		* if the timer device gets idled which is when we
		* are cutting the timer ICLK which is when we try
		* to put Core to RET.
		* Wait Period = 2 timer interface clock cycles +
		* 1 timer functional clock cycle
		* Interface clock = L4 clock. For the computation L4
		* clock is assumed at 50MHz (worst case).
		* Functional clock = 32KHz
		* Wait Period = 2*10^-6/50 + 1/32768 = 0.000030557 = 30.557uSec
		* Rounding off the delay value to a safer 50uSec
		*/
		/* Enabling 32ksync clock in case we are back from off */
		CM_ICLKEN_WKUP |= (1 << 2);
		omap_udelay(GPTIMER_WAIT_DELAY);
		CM_AUTOIDLE_WKUP &= ~(0x1);
	}
	/* Rest of registers are restored later in prcm_register_restore*/
}

int prcm_set_chip_power_mode(struct system_power_state *target_state,
						u32 wakeup_source)
{
	int ret;
	struct system_power_state current_state;
	u8 state;
	u32 prcm_id;

	/* Check that parameters are valid */
	ret = check_power_mode_parameters(target_state, wakeup_source);
	if (ret != PRCM_PASS)
		return ret;

	prcm_get_power_domain_state(DOM_IVA2, &state);
	current_state.iva2_state = state;
	prcm_get_power_domain_state(DOM_DSS, &state);
	current_state.dss_state = state;
	prcm_get_power_domain_state(DOM_CAM, &state);
	current_state.cam_state = state;
	prcm_get_power_domain_state(DOM_PER, &state);
	current_state.per_state = state;
	prcm_get_power_domain_state(DOM_NEON, &state);
	current_state.neon_state = state;
	prcm_get_power_domain_state(DOM_SGX, &state);
	current_state.gfx_state = state;
	prcm_get_power_domain_state(DOM_USBHOST, &state);
	current_state.usbhost_state = state;

	/* Reset previous power state registers for mpu and core*/
	PM_PREPWSTST_MPU = 0xFF;
	PM_PREPWSTST_CORE = 0xFF;


	prcm_save_registers(target_state);

	/* Force power domains to target state */
	/* IVA2 could be turned off by bridge code. Check that it is on*/
	if ((current_state.iva2_state != PRCM_OFF) &&
			(current_state.iva2_state != PRCM_RET)) {
		ret = prcm_force_power_domain_state(DOM_IVA2,
						target_state->iva2_state);
		if (ret != PRCM_PASS)
			goto restore;

	}
	if (current_state.gfx_state == PRCM_ON) {
		prcm_id = DOM_SGX;

#ifdef CONFIG_HW_SUP_TRANS
		/* Facilitating SWSUP RET, from HWSUP mode */
		prcm_set_clock_domain_state(prcm_id, PRCM_NO_AUTO, PRCM_FALSE);
		prcm_clear_sleep_dependency(prcm_id, PRCM_SLEEPDEP_EN_MPU);
		prcm_clear_wkup_dependency(prcm_id, PRCM_WKDEP_EN_MPU);

		prcm_set_power_domain_state(prcm_id, PRCM_ON, PRCM_FORCE);
#endif /* #ifdef CONFIG_HW_SUP_TRANS */

		ret = prcm_force_power_domain_state(prcm_id,
						target_state->gfx_state);
		if (ret != PRCM_PASS)
			goto restore;
	}
	if (current_state.dss_state == PRCM_ON) {
#ifdef CONFIG_HW_SUP_TRANS
		/* Facilitating SWSUP RET, from HWSUP mode */
		prcm_set_clock_domain_state(DOM_DSS, PRCM_NO_AUTO, PRCM_FALSE);
		prcm_clear_sleep_dependency(DOM_DSS, PRCM_SLEEPDEP_EN_MPU);
		prcm_clear_wkup_dependency(DOM_DSS, PRCM_WKDEP_EN_MPU);

		prcm_set_power_domain_state(DOM_DSS, PRCM_ON, PRCM_FORCE);
#endif /* #ifdef CONFIG_HW_SUP_TRANS */

		ret = prcm_force_power_domain_state(DOM_DSS,
						target_state->dss_state);
		if (ret != PRCM_PASS)
			goto restore;
	}

	if (current_state.cam_state == PRCM_ON) {
#ifdef CONFIG_HW_SUP_TRANS
		/* Facilitating SWSUP RET, from HWSUP mode */
		prcm_set_clock_domain_state(DOM_CAM, PRCM_NO_AUTO, PRCM_FALSE);
		prcm_clear_sleep_dependency(DOM_CAM, PRCM_SLEEPDEP_EN_MPU);
		prcm_clear_wkup_dependency(DOM_CAM, PRCM_WKDEP_EN_MPU);

		prcm_set_power_domain_state(DOM_CAM, PRCM_ON, PRCM_FORCE);
#endif /* #ifdef CONFIG_HW_SUP_TRANS */

		ret = prcm_force_power_domain_state(DOM_CAM,
						target_state->cam_state);
		if (ret != PRCM_PASS)
			goto restore;
	}

	if (current_state.per_state == PRCM_ON) {
#ifdef CONFIG_HW_SUP_TRANS
		/* Facilitating RET in SWSUP, from HWSUP mode */
		prcm_set_clock_domain_state(DOM_PER, PRCM_NO_AUTO, PRCM_FALSE);
		prcm_clear_sleep_dependency(DOM_PER, PRCM_SLEEPDEP_EN_MPU);
		prcm_clear_wkup_dependency(DOM_PER, PRCM_WKDEP_EN_MPU);

		prcm_set_power_domain_state(DOM_PER, PRCM_ON, PRCM_FORCE);
#endif /* #ifdef CONFIG_HW_SUP_TRANS */
		ret = prcm_force_power_domain_state(DOM_PER,
						target_state->per_state);
		if (ret != PRCM_PASS)
			goto restore;
	}

	if (current_state.neon_state == PRCM_ON) {
#ifdef CONFIG_HW_SUP_TRANS
		/* Facilitating SWSUP RET, from HWSUP mode */
		prcm_set_clock_domain_state(DOM_NEON, PRCM_NO_AUTO, PRCM_FALSE);
		prcm_set_power_domain_state(DOM_NEON, PRCM_ON, PRCM_FORCE);
#endif /* #ifdef CONFIG_HW_SUP_TRANS */

		ret = prcm_force_power_domain_state(DOM_NEON,
						target_state->neon_state);
		if (ret != PRCM_PASS)
			goto restore;
	}

	if (current_state.usbhost_state == PRCM_ON) {
#ifdef CONFIG_HW_SUP_TRANS
		/* Facilitating RET in SWSUP, from HWSUP mode */
		prcm_set_clock_domain_state(DOM_USBHOST, PRCM_NO_AUTO,
								PRCM_FALSE);
		prcm_clear_sleep_dependency(DOM_USBHOST, PRCM_SLEEPDEP_EN_MPU);
		prcm_clear_wkup_dependency(DOM_USBHOST, PRCM_WKDEP_EN_MPU);

		prcm_set_power_domain_state(DOM_USBHOST, PRCM_ON, PRCM_FORCE);
#endif /* #ifdef CONFIG_HW_SUP_TRANS */

		ret = prcm_force_power_domain_state(DOM_USBHOST,
						target_state->usbhost_state);
		if (ret != PRCM_PASS)
			goto restore;
	}
	ret = prcm_prepare_mpu_core_domains(target_state, wakeup_source);
#ifdef IOPAD_WKUP
	/* Enabling IO_PAD capabilities */
	PM_WKEN_WKUP |= 0x100;
#endif /* #ifdef IOPAD_WKUP */

	/* Jump to SRAM to execute idle */
	if (ret == PRCM_PASS)
		omap_sram_idle();

#ifdef IOPAD_WKUP
	/* Disabling IO_PAD capabilities */
	PM_WKEN_WKUP &= ~(0x100);
#endif /* #ifdef IOPAD_WKUP */
	prcm_restore_mpu_core_domains(target_state, wakeup_source);

restore:
	if (current_state.iva2_state == PRCM_ON)
		/* Force power domains back to original state */
		prcm_force_power_domain_state(DOM_IVA2,
				current_state.iva2_state);
	if (current_state.gfx_state == PRCM_ON)
		prcm_force_power_domain_state(DOM_SGX, current_state.gfx_state);
	if (current_state.usbhost_state == PRCM_ON)
		prcm_force_power_domain_state(DOM_USBHOST,
				current_state.usbhost_state);
	if (current_state.dss_state == PRCM_ON)
		prcm_force_power_domain_state(DOM_DSS, current_state.dss_state);
	if (current_state.cam_state == PRCM_ON)
		prcm_force_power_domain_state(DOM_CAM, current_state.cam_state);
	if (current_state.per_state == PRCM_ON)
		prcm_force_power_domain_state(DOM_PER, current_state.per_state);
	if (current_state.neon_state == PRCM_ON)
		prcm_force_power_domain_state(DOM_NEON,
				current_state.neon_state);
	prcm_restore_registers(target_state);
	if (target_state->core_state > PRCM_CORE_CSWR_MEMRET)
		prcm_restore_core_context(target_state->core_state);

#ifdef CONFIG_HW_SUP_TRANS
	/* TODO This breaks suspend the second time
	if (current_state.iva2_state == PRCM_ON)*/
		prcm_set_power_domain_state(DOM_IVA2, PRCM_ON, PRCM_AUTO);
	if (current_state.usbhost_state == PRCM_ON)
		prcm_set_power_domain_state(DOM_USBHOST, PRCM_ON, PRCM_AUTO);
	if (current_state.gfx_state == PRCM_ON)
		prcm_set_power_domain_state(DOM_SGX, PRCM_ON, PRCM_AUTO);
	if (current_state.dss_state == PRCM_ON)
		prcm_set_power_domain_state(DOM_DSS, PRCM_ON, PRCM_AUTO);
	if (current_state.cam_state == PRCM_ON)
		prcm_set_power_domain_state(DOM_CAM, PRCM_ON, PRCM_AUTO);
	if (current_state.per_state == PRCM_ON)
		prcm_set_power_domain_state(DOM_PER, PRCM_ON, PRCM_AUTO);
	if (current_state.neon_state == PRCM_ON)
		prcm_set_power_domain_state(DOM_NEON, PRCM_ON, PRCM_AUTO);
#endif /* #ifdef CONFIG_HW_SUP_TRANS */
	if (target_state->mpu_state > PRCM_MPU_INACTIVE) {
		prcm_get_pre_power_domain_state(DOM_MPU, &state);
		if (target_state->mpu_state == PRCM_MPU_OFF) {
			if (state != PRCM_OFF)
				ret = PRCM_FAIL;
		} else {
			if (state != PRCM_RET)
				ret = PRCM_FAIL;
		}
		if (ret == PRCM_FAIL) {
#ifdef DEBUG_SUSPEND
			printk(KERN_INFO "Mpu did not enter target state %d\n",
					target_state->mpu_state);
			printk(KERN_INFO "Pre pwstst : %x\n", PM_PREPWSTST_MPU);
#endif
			/* Reset previous power state registers and */
							/*return error */
			PM_PREPWSTST_MPU = 0xFF;
			PM_PREPWSTST_CORE = 0xFF;
			return PRCM_FAIL;
		}
	}
	if (target_state->core_state >= PRCM_CORE_CSWR_MEMRET) {
		prcm_get_pre_power_domain_state(DOM_CORE1, &state);
		if (target_state->core_state == PRCM_CORE_OFF) {
			if (state != PRCM_OFF)
				ret = PRCM_FAIL;
		} else {
			if (state != PRCM_RET)
				ret = PRCM_FAIL;
		}
		if (ret == PRCM_FAIL) {
#ifdef DEBUG_SUSPEND
			printk(KERN_INFO "Core did not enter"
					" target state: %x\n",
					target_state->core_state);
			printk(KERN_INFO "Pre pwstst mpu: %x,"
					"pre pwstst core: %x\n",
					PM_PREPWSTST_MPU, PM_PREPWSTST_CORE);
#endif
			/* Reset previous power state registers and return */
								/*error */
			PM_PREPWSTST_MPU = 0xFF;
			PM_PREPWSTST_CORE = 0xFF;
			return PRCM_FAIL;
		}
	}
#ifdef DEBUG_SUSPEND
	prcm_debug("\nRegister dump after restore\n\n");
#endif
	return PRCM_PASS;
}

#ifdef CONFIG_OMAP_LL_DEBUG_UART3
/* To print on a UART3 debug console need restore PER/UART */
void enable_debug_uart_per(struct system_power_state *state)
{
	void omap_uart_restore_ctx(int unum);
	prcm_restore_registers(state);
	prcm_set_power_domain_state(DOM_PER, PRCM_ON, PRCM_FORCE);
	prcm_force_power_domain_state(DOM_PER, PRCM_ON);
	CM_FCLKEN_PER = 0x800;
	CM_ICLKEN_PER = 0x800;
	if (state->per_state == PRCM_OFF)
	       omap_uart_restore_ctx(2);
}
#endif

