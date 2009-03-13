/*
 * linux/arch/arm/plat-omap/dss/dss.c
 *
 * Copyright (C) 2008 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * Some code and ideas taken from drivers/video/omap/ driver
 * by Imre Deak.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DSS_SUBSYS_NAME "DSS"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <mach/display.h>
#include <mach/clock.h>
#include "dss.h"

#define DSS_BASE			0x48050000

#define DSS_SZ_REGS			SZ_512

struct dss_reg {
	u16 idx;
};

#define DSS_REG(idx)			((const struct dss_reg) { idx })

#define DSS_REVISION			DSS_REG(0x0000)
#define DSS_SYSCONFIG			DSS_REG(0x0010)
#define DSS_SYSSTATUS			DSS_REG(0x0014)
#define DSS_IRQSTATUS			DSS_REG(0x0018)
#define DSS_CONTROL			DSS_REG(0x0040)
#define DSS_SDI_CONTROL			DSS_REG(0x0044)
#define DSS_PLL_CONTROL			DSS_REG(0x0048)
#define DSS_SDI_STATUS			DSS_REG(0x005C)

#define REG_GET(idx, start, end) \
	FLD_GET(dss_read_reg(idx), start, end)

#define REG_FLD_MOD(idx, val, start, end) \
	dss_write_reg(idx, FLD_MOD(dss_read_reg(idx), val, start, end))

static struct {
	void __iomem    *base;

	struct clk      *dss_ick;
	struct clk	*dss1_fck;
	struct clk	*dss2_fck;
	struct clk      *dss_54m_fck;
	struct clk	*dss_96m_fck;

	unsigned	num_clks_enabled;
	struct platform_device *pdev;
	unsigned	ctx_id;
	u32		ctx[DSS_SZ_REGS / sizeof(u32)];
} dss;

static void dss_clk_enable_all_no_ctx(void);
static void dss_clk_disable_all_no_ctx(void);
static void dss_clk_enable_no_ctx(enum dss_clock clks);
static void dss_clk_disable_no_ctx(enum dss_clock clks);
static int _omap_dss_wait_reset(void);

static char *def_disp_name;
module_param_named(def_disp, def_disp_name, charp, 0);
MODULE_PARM_DESC(def_disp_name, "default display name");

#ifdef DEBUG
unsigned int dss_debug;
module_param_named(debug, dss_debug, bool, 0644);
#endif

static inline void dss_write_reg(const struct dss_reg idx, u32 val)
{
	__raw_writel(val, dss.base + idx.idx);
}

static inline u32 dss_read_reg(const struct dss_reg idx)
{
	return __raw_readl(dss.base + idx.idx);
}

#define SR(reg) \
	dss.ctx[(DSS_##reg).idx / sizeof(u32)] = dss_read_reg(DSS_##reg)
#define RR(reg) \
	dss_write_reg(DSS_##reg, dss.ctx[(DSS_##reg).idx / sizeof(u32)])

static void dss_save_context(void)
{
	if (cpu_is_omap24xx())
		return;

	SR(SYSCONFIG);
	SR(CONTROL);
	SR(SDI_CONTROL);
	SR(PLL_CONTROL);
}

static void dss_restore_context(void)
{
	RR(SYSCONFIG);
	RR(CONTROL);
	RR(SDI_CONTROL);
	RR(PLL_CONTROL);
}

#undef SR
#undef RR

static unsigned dss_get_ctx_id(void)
{
	struct omap_dss_platform_data *pdata = dss.pdev->dev.platform_data;

	if (!pdata->get_last_off_on_transaction_id)
		return 0;

	return pdata->get_last_off_on_transaction_id(&dss.pdev->dev);
}

static void save_all_ctx(void)
{
	DSSDBG("save context\n");

	dss_clk_enable_no_ctx(DSS_CLK_ICK | DSS_CLK_FCK1);

	dss_save_context();
	dispc_save_context();
#ifdef CONFIG_OMAP2_DSS_DSI
	dsi_save_context();
#endif

	dss_clk_disable_no_ctx(DSS_CLK_ICK | DSS_CLK_FCK1);
}

static void restore_all_ctx(void)
{
	DSSDBG("restore context\n");

	dss_clk_enable_all_no_ctx();

	if (_omap_dss_wait_reset())
		DSSERR("DSS not coming out of reset after sleep\n");

	dss_restore_context();
	dispc_restore_context();
#ifdef CONFIG_OMAP2_DSS_DSI
	dsi_restore_context();
#endif

	dss_clk_disable_all_no_ctx();
}

void dss_sdi_init(int datapairs)
{
	u32 l;

	BUG_ON(datapairs > 3 || datapairs < 1);

	l = dss_read_reg(DSS_SDI_CONTROL);
	l = FLD_MOD(l, 0xf, 19, 15);		/* SDI_PDIV */
	l = FLD_MOD(l, datapairs-1, 3, 2);	/* SDI_PRSEL */
	l = FLD_MOD(l, 2, 1, 0);		/* SDI_BWSEL */
	dss_write_reg(DSS_SDI_CONTROL, l);

	l = dss_read_reg(DSS_PLL_CONTROL);
	l = FLD_MOD(l, 0x7, 25, 22);	/* SDI_PLL_FREQSEL */
	l = FLD_MOD(l, 0xb, 16, 11);	/* SDI_PLL_REGN */
	l = FLD_MOD(l, 0xb4, 10, 1);	/* SDI_PLL_REGM */
	dss_write_reg(DSS_PLL_CONTROL, l);

	/* Reset SDI PLL */
	REG_FLD_MOD(DSS_PLL_CONTROL, 1, 18, 18); /* SDI_PLL_SYSRESET */
	udelay(1);	/* wait 2x PCLK */

	/* Lock SDI PLL */
	REG_FLD_MOD(DSS_PLL_CONTROL, 1, 28, 28); /* SDI_PLL_GOBIT */

	/* Waiting for PLL lock request to complete */
	while (dss_read_reg(DSS_SDI_STATUS) & (1 << 6))
		;

	/* Clearing PLL_GO bit */
	REG_FLD_MOD(DSS_PLL_CONTROL, 0, 28, 28);

	/* Waiting for PLL to lock */
	while (!(dss_read_reg(DSS_SDI_STATUS) & (1 << 5)))
		;

	dispc_lcd_enable_signal(1);

	/* Waiting for SDI reset to complete */
	while (!(dss_read_reg(DSS_SDI_STATUS) & (1 << 5)))
		;
}

ssize_t dss_print_clocks(char *buf, ssize_t size)
{
	ssize_t l = 0;
	int i;
	struct clk *clocks[5] = {
		dss.dss_ick,
		dss.dss1_fck,
		dss.dss2_fck,
		dss.dss_54m_fck,
		dss.dss_96m_fck
	};

	l += snprintf(buf + l, size - l, "- dss -\n");

	l += snprintf(buf + l, size - l, "internal clk count\t%u\n",
			dss.num_clks_enabled);

	for (i = 0; i < 5; i++) {
		if (!clocks[i])
			continue;
		l += snprintf(buf + l, size - l, "%-15s\t%lu\t%d\n",
				clocks[i]->name,
				clk_get_rate(clocks[i]),
				clocks[i]->usecount);
	}

	return l;
}

static int get_dss_clocks(void)
{
	const struct {
		struct clk **clock;
		char *omap2_name;
		char *omap3_name;
	} clocks[5] = {
		{ &dss.dss_ick, "dss_ick", "dss_ick" },	/* L3 & L4 ick */
		{ &dss.dss1_fck, "dss1_fck", "dss1_alwon_fck" },
		{ &dss.dss2_fck, "dss2_fck", "dss2_alwon_fck" },
		{ &dss.dss_54m_fck, "dss_54m_fck", "dss_tv_fck" },
		{ &dss.dss_96m_fck, NULL, "dss_96m_fck" },
	};

	int r = 0;
	int i;
	const int num_clocks = 5;

	for (i = 0; i < num_clocks; i++)
		*clocks[i].clock = NULL;

	for (i = 0; i < num_clocks; i++) {
		struct clk *clk;
		const char *clk_name;

		clk_name = cpu_is_omap34xx() ? clocks[i].omap3_name
			: clocks[i].omap2_name;

		if (!clk_name)
			continue;

		clk = clk_get(NULL, clk_name);

		if (IS_ERR(clk)) {
			DSSERR("can't get clock %s", clk_name);
			r = PTR_ERR(clk);
			goto err;
		}

		DSSDBG("clk %s, rate %ld\n",
				clk_name, clk_get_rate(clk));

		*clocks[i].clock = clk;
	}

	return 0;

err:
	for (i = 0; i < num_clocks; i++) {
		if (!IS_ERR(*clocks[i].clock))
			clk_put(*clocks[i].clock);
	}

	return r;
}

static void put_dss_clocks(void)
{
	if (dss.dss_96m_fck)
		clk_put(dss.dss_96m_fck);
	clk_put(dss.dss_54m_fck);
	clk_put(dss.dss1_fck);
	clk_put(dss.dss2_fck);
	clk_put(dss.dss_ick);
}

unsigned long dss_clk_get_rate(enum dss_clock clk)
{
	switch (clk) {
	case DSS_CLK_ICK:
		return clk_get_rate(dss.dss_ick);
	case DSS_CLK_FCK1:
		return clk_get_rate(dss.dss1_fck);
	case DSS_CLK_FCK2:
		return clk_get_rate(dss.dss2_fck);
	case DSS_CLK_54M:
		return clk_get_rate(dss.dss_54m_fck);
	case DSS_CLK_96M:
		return clk_get_rate(dss.dss_96m_fck);
	}

	BUG();
	return 0;
}

static unsigned count_clk_bits(enum dss_clock clks)
{
	unsigned num_clks = 0;

	if (clks & DSS_CLK_ICK)
		++num_clks;
	if (clks & DSS_CLK_FCK1)
		++num_clks;
	if (clks & DSS_CLK_FCK2)
		++num_clks;
	if (clks & DSS_CLK_54M)
		++num_clks;
	if (clks & DSS_CLK_96M)
		++num_clks;

	return num_clks;
}

static void dss_clk_enable_no_ctx(enum dss_clock clks)
{
	unsigned num_clks = count_clk_bits(clks);

	if (clks & DSS_CLK_ICK)
		clk_enable(dss.dss_ick);
	if (clks & DSS_CLK_FCK1)
		clk_enable(dss.dss1_fck);
	if (clks & DSS_CLK_FCK2)
		clk_enable(dss.dss2_fck);
	if (clks & DSS_CLK_54M)
		clk_enable(dss.dss_54m_fck);
	if (clks & DSS_CLK_96M)
		clk_enable(dss.dss_96m_fck);

	dss.num_clks_enabled += num_clks;
}

void dss_clk_enable(enum dss_clock clks)
{
	dss_clk_enable_no_ctx(clks);

	if (cpu_is_omap34xx()) {
		unsigned num_clks = count_clk_bits(clks);

		if (dss.num_clks_enabled == num_clks)
			restore_all_ctx();
	}
}

static void dss_clk_disable_no_ctx(enum dss_clock clks)
{
	unsigned num_clks = count_clk_bits(clks);

	if (clks & DSS_CLK_ICK)
		clk_disable(dss.dss_ick);
	if (clks & DSS_CLK_FCK1)
		clk_disable(dss.dss1_fck);
	if (clks & DSS_CLK_FCK2)
		clk_disable(dss.dss2_fck);
	if (clks & DSS_CLK_54M)
		clk_disable(dss.dss_54m_fck);
	if (clks & DSS_CLK_96M)
		clk_disable(dss.dss_96m_fck);

	dss.num_clks_enabled -= num_clks;
}

void dss_clk_disable(enum dss_clock clks)
{
	if (cpu_is_omap34xx()) {
		unsigned num_clks = count_clk_bits(clks);

		BUG_ON(dss.num_clks_enabled < num_clks);

		if (dss.num_clks_enabled == num_clks)
			save_all_ctx();
	}

	dss_clk_disable_no_ctx(clks);
}

static void dss_clk_enable_all_no_ctx(void)
{
	enum dss_clock clks;

	clks = DSS_CLK_ICK | DSS_CLK_FCK1 | DSS_CLK_FCK2 | DSS_CLK_54M;
	if (cpu_is_omap34xx())
		clks |= DSS_CLK_96M;
	dss_clk_enable_no_ctx(clks);
}

static void dss_clk_disable_all_no_ctx(void)
{
	enum dss_clock clks;

	clks = DSS_CLK_ICK | DSS_CLK_FCK1 | DSS_CLK_FCK2 | DSS_CLK_54M;
	if (cpu_is_omap34xx())
		clks |= DSS_CLK_96M;
	dss_clk_disable_no_ctx(clks);
}

static void dss_clk_disable_all(void)
{
	enum dss_clock clks;

	clks = DSS_CLK_ICK | DSS_CLK_FCK1 | DSS_CLK_FCK2 | DSS_CLK_54M;
	if (cpu_is_omap34xx())
		clks |= DSS_CLK_96M;
	dss_clk_disable(clks);
}

void dss_select_clk_source(int dsi, int dispc)
{
	u32 r;
	r = dss_read_reg(DSS_CONTROL);
	r = FLD_MOD(r, dsi, 1, 1);	/* DSI_CLK_SWITCH */
	r = FLD_MOD(r, dispc, 0, 0);	/* DISPC_CLK_SWITCH */
	dss_write_reg(DSS_CONTROL, r);
}

int dss_get_dsi_clk_source(void)
{
	return FLD_GET(dss_read_reg(DSS_CONTROL), 1, 1);
}

int dss_get_dispc_clk_source(void)
{
	return FLD_GET(dss_read_reg(DSS_CONTROL), 0, 0);
}

static irqreturn_t dss_irq_handler_omap2(int irq, void *arg)
{
	dispc_irq_handler();

	return IRQ_HANDLED;
}

static irqreturn_t dss_irq_handler_omap3(int irq, void *arg)
{
	u32 irqstatus;

	irqstatus = dss_read_reg(DSS_IRQSTATUS);

	if (irqstatus & (1<<0))	/* DISPC_IRQ */
		dispc_irq_handler();
#ifdef CONFIG_OMAP2_DSS_DSI
	if (irqstatus & (1<<1))	/* DSI_IRQ */
		dsi_irq_handler();
#endif

	/* Workaround suggested by Tony  for spurious interrupt warning */
	irqstatus = dss_read_reg(DSS_IRQSTATUS);

	return IRQ_HANDLED;
}

static int _omap_dss_wait_reset(void)
{
	unsigned timeout = 1000;

	while (REG_GET(DSS_SYSSTATUS, 0, 0) == 0) {
		udelay(1);
		if (!--timeout) {
			DSSERR("soft reset failed\n");
			return -ENODEV;
		}
	}

	return 0;
}

static int _omap_dss_reset(void)
{
	/* Soft reset */
	REG_FLD_MOD(DSS_SYSCONFIG, 1, 1, 1);
	return _omap_dss_wait_reset();
}

void dss_set_venc_output(enum omap_dss_venc_type type)
{
	int l = 0;

	if (type == OMAP_DSS_VENC_TYPE_COMPOSITE)
		l = 0;
	else if (type == OMAP_DSS_VENC_TYPE_SVIDEO)
		l = 1;
	else
		BUG();

	/* venc out selection. 0 = comp, 1 = svideo */
	REG_FLD_MOD(DSS_CONTROL, l, 6, 6);
}

void dss_set_dac_pwrdn_bgz(int enable)
{
	REG_FLD_MOD(DSS_CONTROL, enable, 5, 5);	/* DAC Power-Down Control */
}

int dss_init(void)
{
	int r;
	u32 rev;

	dss.base = ioremap(DSS_BASE, DSS_SZ_REGS);
	if (!dss.base) {
		DSSERR("can't ioremap DSS\n");
		r = -ENOMEM;
		goto fail0;
	}

	/* We need to wait here a bit, otherwise we sometimes start to get
	 * synclost errors. I believe we could wait for one framedone or
	 * perhaps vsync interrupt, but, because dispc is not initialized yet,
	 * we don't have access to the irq register.
	 */
	msleep(400);

	_omap_dss_reset();

	/* autoidle */
	REG_FLD_MOD(DSS_SYSCONFIG, 1, 0, 0);

	/* Select DPLL */
	REG_FLD_MOD(DSS_CONTROL, 0, 0, 0);

#ifdef CONFIG_OMAP2_DSS_VENC
	REG_FLD_MOD(DSS_CONTROL, 1, 4, 4);	/* venc dac demen */
	REG_FLD_MOD(DSS_CONTROL, 1, 3, 3);	/* venc clock 4x enable */
	REG_FLD_MOD(DSS_CONTROL, 0, 2, 2);	/* venc clock mode = normal */
#endif

	r = request_irq(INT_24XX_DSS_IRQ,
			cpu_is_omap24xx()
			? dss_irq_handler_omap2
			: dss_irq_handler_omap3,
			0, "OMAP DSS", NULL);

	if (r < 0) {
		DSSERR("omap2 dss: request_irq failed\n");
		goto fail1;
	}

	dss_save_context();

	rev = dss_read_reg(DSS_REVISION);
	printk(KERN_INFO "OMAP DSS rev %d.%d\n",
			FLD_GET(rev, 7, 4), FLD_GET(rev, 3, 0));

	return 0;

fail1:
	iounmap(dss.base);
fail0:
	return r;
}

void dss_exit(void)
{
	int c;

	free_irq(INT_24XX_DSS_IRQ, NULL);

	/* these should be removed at some point */
	c = dss.dss_ick->usecount;
	if (c > 0) {
		DSSERR("warning: dss_ick usecount %d, disabling\n", c);
		while (c-- > 0)
			clk_disable(dss.dss_ick);
	}

	c = dss.dss1_fck->usecount;
	if (c > 0) {
		DSSERR("warning: dss1_fck usecount %d, disabling\n", c);
		while (c-- > 0)
			clk_disable(dss.dss1_fck);
	}

	c = dss.dss2_fck->usecount;
	if (c > 0) {
		DSSERR("warning: dss2_fck usecount %d, disabling\n", c);
		while (c-- > 0)
			clk_disable(dss.dss2_fck);
	}

	c = dss.dss_54m_fck->usecount;
	if (c > 0) {
		DSSERR("warning: dss_54m_fck usecount %d, disabling\n", c);
		while (c-- > 0)
			clk_disable(dss.dss_54m_fck);
	}

	if (dss.dss_96m_fck) {
		c = dss.dss_96m_fck->usecount;
		if (c > 0) {
			DSSERR("warning: dss_96m_fck usecount %d, disabling\n",
					c);
			while (c-- > 0)
				clk_disable(dss.dss_96m_fck);
		}
	}

	put_dss_clocks();

	iounmap(dss.base);
}



static int omap_dss_probe(struct platform_device *pdev)
{
	struct omap_dss_platform_data *pdata = pdev->dev.platform_data;

	int r;

	dss.pdev = pdev;

	r = get_dss_clocks();
	if (r)
		goto fail0;

	dss_clk_enable_all_no_ctx();

	dss.ctx_id = dss_get_ctx_id();
	DSSDBG("initial ctx id %u\n", dss.ctx_id);

	r = dss_init();
	if (r) {
		DSSERR("Failed to initialize DSS\n");
		goto fail0;
	}

#ifdef CONFIG_OMAP2_DSS_RFBI
	r = rfbi_init();
	if (r) {
		DSSERR("Failed to initialize rfbi\n");
		goto fail0;
	}
#endif

	r = dpi_init();
	if (r) {
		DSSERR("Failed to initialize dpi\n");
		goto fail0;
	}

	r = dispc_init();
	if (r) {
		DSSERR("Failed to initialize dispc\n");
		goto fail0;
	}
#ifdef CONFIG_OMAP2_DSS_VENC
	r = venc_init();
	if (r) {
		DSSERR("Failed to initialize venc\n");
		goto fail0;
	}
#endif
	if (cpu_is_omap34xx()) {
#ifdef CONFIG_OMAP2_DSS_SDI
		r = sdi_init();
		if (r) {
			DSSERR("Failed to initialize SDI\n");
			goto fail0;
		}
#endif
#ifdef CONFIG_OMAP2_DSS_DSI
		r = dsi_init();
		if (r) {
			DSSERR("Failed to initialize DSI\n");
			goto fail0;
		}
#endif
	}

	initialize_displays(pdata);

	r = initialize_sysfs(&pdev->dev);
	if (r)
		goto fail0;

	initialize_overlays(def_disp_name);

	dss_clk_disable_all();

	return 0;

	/* XXX fail correctly */
fail0:
	return r;
}

static int omap_dss_remove(struct platform_device *pdev)
{
	uninitialize_sysfs(&pdev->dev);

#ifdef CONFIG_OMAP2_DSS_VENC
	venc_exit();
#endif
	dispc_exit();
	dpi_exit();
#ifdef CONFIG_OMAP2_DSS_RFBI
	rfbi_exit();
#endif
	if (cpu_is_omap34xx()) {
#ifdef CONFIG_OMAP2_DSS_DSI
		dsi_exit();
#endif
#ifdef CONFIG_OMAP2_DSS_SDI
		sdi_exit();
#endif
	}

	dss_exit();

	return 0;
}

static int omap_dss_suspend(struct platform_device *pdev, pm_message_t state)
{
	DSSDBG("suspend %d\n", state.event);

	return omap_dss_suspend_all_displays();
}

static int omap_dss_resume(struct platform_device *pdev)
{
	DSSDBG("resume\n");

	return omap_dss_resume_all_displays();
}

static struct platform_driver omap_dss_driver = {
	.probe          = omap_dss_probe,
	.remove         = omap_dss_remove,
	.suspend	= omap_dss_suspend,
	.resume		= omap_dss_resume,
	.driver         = {
		.name   = "omap-dss",
		.owner  = THIS_MODULE,
	},
};

static int __init omap_dss_init(void)
{
	return platform_driver_register(&omap_dss_driver);
}

static void __exit omap_dss_exit(void)
{
	platform_driver_unregister(&omap_dss_driver);
}

subsys_initcall(omap_dss_init);
module_exit(omap_dss_exit);


MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@nokia.com>");
MODULE_DESCRIPTION("OMAP2/3 Display Subsystem");
MODULE_LICENSE("GPL v2");

