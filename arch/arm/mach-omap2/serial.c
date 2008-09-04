/*
 * arch/arm/mach-omap2/serial.c
 *
 * OMAP2 serial support.
 *
 * Copyright (C) 2005-2008 Nokia Corporation
 * Author: Paul Mundt <paul.mundt@nokia.com>
 *
 * Based off of arch/arm/mach-omap/omap1/serial.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/serial_8250.h>
#include <linux/serial_reg.h>
#include <linux/clk.h>

#include <linux/io.h>

#include <asm/arch/common.h>
#include <asm/arch/board.h>
#include <asm/arch/clock.h>
#include <asm/arch/control.h>

#include "prm.h"
#include "pm.h"

#define SERIAL_AWAKE_TIME 5

static struct clk *uart_ick[OMAP_MAX_NR_PORTS];
static struct clk *uart_fck[OMAP_MAX_NR_PORTS];
static struct timespec omap_serial_next_sleep;

#ifdef CONFIG_ARCH_OMAP24XX
static const u32 omap2_uart_wk_st[OMAP_MAX_NR_PORTS] = {
	OMAP2420_PRM_REGADDR(CORE_MOD, PM_WKST1),
	OMAP2420_PRM_REGADDR(CORE_MOD, PM_WKST1),
	OMAP2420_PRM_REGADDR(CORE_MOD, OMAP24XX_PM_WKST2)
};
static const u32 omap2_uart_wk_en[OMAP_MAX_NR_PORTS] = {
	OMAP2420_PRM_REGADDR(CORE_MOD, PM_WKEN1),
	OMAP2420_PRM_REGADDR(CORE_MOD, PM_WKEN1),
	OMAP2420_PRM_REGADDR(CORE_MOD, OMAP24XX_PM_WKEN2),
};
static const u32 omap2_uart_wk_bit[OMAP_MAX_NR_PORTS] = {
	OMAP24XX_ST_UART1, OMAP24XX_ST_UART2, OMAP24XX_ST_UART3
};
#endif

#ifdef CONFIG_ARCH_OMAP34XX
static const u32 omap3_uart_wk_st[OMAP_MAX_NR_PORTS] = {
	OMAP34XX_PRM_REGADDR(CORE_MOD, PM_WKST1),
	OMAP34XX_PRM_REGADDR(CORE_MOD, PM_WKST1),
	OMAP34XX_PRM_REGADDR(OMAP3430_PER_MOD, PM_WKST1)
};
static const u32 omap3_uart_wk_en[OMAP_MAX_NR_PORTS] = {
	OMAP34XX_PRM_REGADDR(CORE_MOD, PM_WKEN1),
	OMAP34XX_PRM_REGADDR(CORE_MOD, PM_WKEN1),
	OMAP34XX_PRM_REGADDR(OMAP3430_PER_MOD, PM_WKEN1)
};
static const u32 omap3_uart_wk_bit[OMAP_MAX_NR_PORTS] = {
	OMAP3430_ST_UART1, OMAP3430_ST_UART2, OMAP3430_ST_UART3
};
#endif

static const u32 *omap_uart_wk_st;
static const u32 *omap_uart_wk_en;
static const u32 *omap_uart_wk_bit;

/* UART padconfig registers, these may differ if non-default padconfig
   is used */
#define CONTROL_PADCONF_UART1_RX 0x182
#define CONTROL_PADCONF_UART2_RX 0x17A
#define CONTROL_PADCONF_UART3_RX 0x19E
#define PADCONF_WAKEUP_ST 0x8000

static const u32 omap34xx_uart_padconf[OMAP_MAX_NR_PORTS] = {
	CONTROL_PADCONF_UART1_RX,
	CONTROL_PADCONF_UART2_RX,
	CONTROL_PADCONF_UART3_RX
};

static struct plat_serial8250_port serial_platform_data[] = {
	{
		.membase	= (__force void __iomem *)IO_ADDRESS(OMAP_UART1_BASE),
		.mapbase	= (unsigned long)OMAP_UART1_BASE,
		.irq		= 72,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= OMAP24XX_BASE_BAUD * 16,
	}, {
		.membase	= (__force void __iomem *)IO_ADDRESS(OMAP_UART2_BASE),
		.mapbase	= (unsigned long)OMAP_UART2_BASE,
		.irq		= 73,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= OMAP24XX_BASE_BAUD * 16,
	}, {
		.membase	= (__force void __iomem *)IO_ADDRESS(OMAP_UART3_BASE),
		.mapbase	= (unsigned long)OMAP_UART3_BASE,
		.irq		= 74,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= OMAP24XX_BASE_BAUD * 16,
	}, {
		.flags		= 0
	}
};

static inline unsigned int serial_read_reg(struct plat_serial8250_port *up,
					   int offset)
{
	offset <<= up->regshift;
	return (unsigned int)__raw_readb(up->membase + offset);
}

static inline void serial_write_reg(struct plat_serial8250_port *p, int offset,
				    int value)
{
	offset <<= p->regshift;
	__raw_writeb(value, p->membase + offset);
}

/*
 * Internal UARTs need to be initialized for the 8250 autoconfig to work
 * properly. Note that the TX watermark initialization may not be needed
 * once the 8250.c watermark handling code is merged.
 */
static inline void __init omap_serial_reset(struct plat_serial8250_port *p)
{
	serial_write_reg(p, UART_OMAP_MDR1, 0x07);
	serial_write_reg(p, UART_OMAP_SCR, 0x08);
	serial_write_reg(p, UART_OMAP_MDR1, 0x00);
	serial_write_reg(p, UART_OMAP_SYSC, (0x02 << 3) | (1 << 2) | (1 << 0));
}

static void omap_serial_kick(void)
{
	getnstimeofday(&omap_serial_next_sleep);
	timespec_add_ns(&omap_serial_next_sleep, (s64)SERIAL_AWAKE_TIME *
		NSEC_PER_SEC);
}

void omap_serial_enable_clocks(int enable, int unum)
{
	if (uart_ick[unum] && uart_fck[unum]) {
		if (enable) {
			clk_enable(uart_ick[unum]);
			clk_enable(uart_fck[unum]);
		} else {
			clk_disable(uart_ick[unum]);
			clk_disable(uart_fck[unum]);
		}
	}
}

void omap_serial_fclk_mask(u32 *f1, u32 *f2)
{
	if (uart_ick[0])
		*f1 &= ~(1 << uart_fck[0]->enable_bit);
	if (uart_ick[1])
		*f1 &= ~(1 << uart_fck[1]->enable_bit);
	if (uart_ick[2])
		*f2 &= ~(1 << uart_fck[2]->enable_bit);
}

void omap_serial_check_wakeup(void)
{
	int i;


	for (i = 0; i < OMAP_MAX_NR_PORTS; i++) {
		if (!uart_ick[i])
			continue;

		if (cpu_is_omap34xx())
			if (omap_ctrl_readw(omap34xx_uart_padconf[i]) &
			    PADCONF_WAKEUP_ST) {
				omap_serial_kick();
				return;
			}

		if (__raw_readl(omap_uart_wk_st[i]) &
		    omap_uart_wk_bit[i]) {
			omap_serial_kick();
			return;
		}
	}
}

int omap_serial_can_sleep(void)
{
	int i;
	struct timespec t;

	struct plat_serial8250_port *p = serial_platform_data;

	getnstimeofday(&t);

	for (i = 0; i < OMAP_MAX_NR_PORTS; i++) {
		if (!uart_ick[i])
			continue;
		/* Check if we have data in the transmit buffer */
		if ((serial_read_reg(p + i, UART_LSR) &
			(UART_LSR_TEMT|UART_LSR_THRE))
			!= (UART_LSR_TEMT|UART_LSR_THRE)) {
				omap_serial_kick();
				return 0;
		}
	}

	if (timespec_compare(&t, &omap_serial_next_sleep) < 0)
		return 0;

	return 1;
}

void __init omap_serial_init(void)
{
	int i;
	const struct omap_uart_config *info;
	char name[16];

	/*
	 * Make sure the serial ports are muxed on at this point.
	 * You have to mux them off in device drivers later on
	 * if not needed.
	 */

	info = omap_get_config(OMAP_TAG_UART, struct omap_uart_config);

	if (info == NULL)
		return;

#ifdef CONFIG_ARCH_OMAP24XX
	if (cpu_is_omap242x()) {
		omap_uart_wk_st = omap2_uart_wk_st;
		omap_uart_wk_en = omap2_uart_wk_en;
		omap_uart_wk_bit = omap2_uart_wk_bit;
	}
#endif

#ifdef CONFIG_ARCH_OMAP34XX
	if (cpu_is_omap34xx()) {
		omap_uart_wk_st = omap3_uart_wk_st;
		omap_uart_wk_en = omap3_uart_wk_en;
		omap_uart_wk_bit = omap3_uart_wk_bit;
	}
#endif

	for (i = 0; i < OMAP_MAX_NR_PORTS; i++) {
		struct plat_serial8250_port *p = serial_platform_data + i;
		u32 v;

		if (!(info->enabled_uarts & (1 << i))) {
			p->membase = NULL;
			p->mapbase = 0;
			continue;
		}

		sprintf(name, "uart%d_ick", i+1);
		uart_ick[i] = clk_get(NULL, name);
		if (IS_ERR(uart_ick[i])) {
			printk(KERN_ERR "Could not get uart%d_ick\n", i+1);
			uart_ick[i] = NULL;
		} else
			clk_enable(uart_ick[i]);

		sprintf(name, "uart%d_fck", i+1);
		uart_fck[i] = clk_get(NULL, name);
		if (IS_ERR(uart_fck[i])) {
			printk(KERN_ERR "Could not get uart%d_fck\n", i+1);
			uart_fck[i] = NULL;
		} else
			clk_enable(uart_fck[i]);

		omap_serial_reset(p);

		v = __raw_readl(omap_uart_wk_en[i]);
		v |= omap_uart_wk_bit[i];
		__raw_writel(v, omap_uart_wk_en[i]);
	}

	omap_serial_kick();
}

static struct platform_device serial_device = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM,
	.dev			= {
		.platform_data	= serial_platform_data,
	},
};

static int __init omap_init(void)
{
	return platform_device_register(&serial_device);
}
arch_initcall(omap_init);
