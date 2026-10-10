// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTL8197F SoC clock driver
 *
 * Copyright (c) 2026 David Yang <mmyangfl@gmail.com>
 *
 * The RTL8197F derives the CPU clock from an on-chip PLL whose output
 * frequency is selected by a 4-bit hardware strap field and further
 * divided by a 2-bit divider, both readable from the hardware strap
 * register at 0x18000008.  Strap bit 14 bypasses the PLL and runs the
 * CPU directly from the fixed 200 MHz LX bus clock.
 *
 * This driver probes the reset values of those registers and exports
 * read-only clocks, plus the one thing on this SoC that is not a
 * divider: the WLAN clock/function gate the WiFi core cannot be touched
 * without.
 *
 * The provider is registered from of_clk_init() rather than from a platform
 * driver because its first consumers are: cpu@0, whose clock plat_time_init()
 * turns into mips_hpt_frequency, and the system timer, which timer_probe()
 * sets up in the same path.  Neither of them can wait for a driver to probe.
 */

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/spinlock.h>
#include <linux/slab.h>

#include <dt-bindings/clock/realtek-rtl819x-clks.h>

/* Hardware strap register, offsets relative to the mapped range */
#define RTL819X_REG_HW_STRAP			0x08

/* CPU clock bypass: run CPU from the LX bus clock instead of the PLL */
#define RTL819X_STRAP_CPU_BYPASS		BIT(14)
/* CPU PLL frequency selection, 4-bit index into rtl819x_pllsel_table[] */
#define RTL819X_STRAP_CPU_FREQ_SEL_SHIFT	15
#define RTL819X_STRAP_CPU_FREQ_SEL_WIDTH	4
/* CPU post-divider, 2-bit index into {1, 2, 4, 8} */
#define RTL819X_STRAP_CPU_DIV_SHIFT		19
#define RTL819X_STRAP_CPU_DIV_WIDTH		2

/* Fixed LX (bus) clock; extension timer and peripherals run from it */
#define RTL819X_LX_CLK_RATE			200000000
/* SPI NOR ("Sheipa") controller clock as programmed by the bootloader */
#define RTL819X_SPI_CLK_RATE			200000000

/*
 * Crystal oscillator select, in the same strap register: set means a
 * 40 MHz crystal, clear means 25 MHz.  The WLAN core has to be told
 * which one it is fed, so the rate is published as a clock.
 */
#define RTL819X_STRAP_XTAL_40MHZ		BIT(24)
#define RTL819X_XTAL_25MHZ			25000000
#define RTL819X_XTAL_40MHZ			40000000

/*
 * WLAN clock/function control.  The vendor enable sets the whole
 * five-bit field and tests bit0 for "clock running" (8192cd_osdep.c
 * "REG32(0xB8000064) |= 0x1F", 8192cd_util.h "if (!(REG32(0xB8000064)
 * & BIT(0)) panic)"), so this is not a single-bit gate and gets its own
 * clk_ops instead of clk-gate.
 */
#define RTL819X_REG_WLAN_CTRL			0x64
#define RTL819X_WLAN_CTRL_EN			GENMASK(4, 0)
#define RTL819X_WLAN_CTRL_STAT			BIT(0)

/*
 * Embedded switch core gate.  The vendor toggles this bit as part of the
 * switch core reset (rtl865x_asicCom.c "FullAndSemiReset()":
 * "REG32(SYS_CLK_MAG) &= ~CM_ACTIVE_SWCORE" ... then sets it back), so the
 * switch driver needs to gate the clock off and on again; it is a plain
 * single-bit gate, unlike the five-bit WLAN field above.
 */
#define RTL819X_REG_SYS_CLK_MAG			0x10
#define RTL819X_SWCORE_EN			BIT(11)

/*
 * CPU PLL output frequencies (MHz) indexed by strap bits 15..18,
 * from the vendor SDK cpu_clksel_table[].
 */
static const unsigned long rtl819x_pllsel_table[] = {
	[0]  = 450000000, [1]  = 500000000, [2]  = 550000000, [3]  = 600000000,
	[4]  = 650000000, [5]  = 700000000, [6]  = 750000000, [7]  = 800000000,
	[8]  = 850000000, [9]  = 900000000, [10] = 950000000, [11] = 1000000000,
	[12] = 1050000000, [13] = 1100000000, [14] = 1150000000, [15] = 1200000000,
};

/* CPU post-divider values indexed by strap bits 19..20 */
static const struct clk_div_table rtl819x_cpu_div_table[] = {
	{ .val = 0, .div = 1 },
	{ .val = 1, .div = 2 },
	{ .val = 2, .div = 4 },
	{ .val = 3, .div = 8 },
	{ .val = 0, .div = 0 },
};

struct rtl819x_pll_hw {
	struct clk_hw hw;
	void __iomem *base;
};

static inline struct rtl819x_pll_hw *to_rtl819x_pll_hw(struct clk_hw *hw)
{
	return container_of(hw, struct rtl819x_pll_hw, hw);
}

static unsigned long rtl819x_pll_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct rtl819x_pll_hw *pll = to_rtl819x_pll_hw(hw);
	unsigned int val;

	val = readl(pll->base + RTL819X_REG_HW_STRAP);
	val >>= RTL819X_STRAP_CPU_FREQ_SEL_SHIFT;
	val &= GENMASK(RTL819X_STRAP_CPU_FREQ_SEL_WIDTH - 1, 0);

	return rtl819x_pllsel_table[val];
}

static const struct clk_ops rtl819x_pll_ops = {
	.recalc_rate = rtl819x_pll_recalc_rate,
};

/*
 * The PLL takes no parent clock: its output is whatever the strap field selects
 * out of rtl819x_pllsel_table[], and recalc_rate() never looks at a parent rate.
 */
static struct clk_hw *rtl819x_register_pll(void __iomem *base,
					   const char *name)
{
	struct rtl819x_pll_hw *pll;
	struct clk_init_data init = {};

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &rtl819x_pll_ops;
	init.num_parents = 0;

	pll->base = base;
	pll->hw.init = &init;

	if (clk_hw_register(NULL, &pll->hw))
		return ERR_PTR(-EINVAL);
	return &pll->hw;
}

struct rtl819x_wlan_hw {
	struct clk_hw hw;
	void __iomem *base;
};

static inline struct rtl819x_wlan_hw *to_rtl819x_wlan_hw(struct clk_hw *hw)
{
	return container_of(hw, struct rtl819x_wlan_hw, hw);
}

static int rtl819x_wlan_enable(struct clk_hw *hw)
{
	struct rtl819x_wlan_hw *wlan = to_rtl819x_wlan_hw(hw);

	writel(readl(wlan->base + RTL819X_REG_WLAN_CTRL) |
	       RTL819X_WLAN_CTRL_EN, wlan->base + RTL819X_REG_WLAN_CTRL);
	return 0;
}

static void rtl819x_wlan_disable(struct clk_hw *hw)
{
	struct rtl819x_wlan_hw *wlan = to_rtl819x_wlan_hw(hw);

	writel(0, wlan->base + RTL819X_REG_WLAN_CTRL);
}

static int rtl819x_wlan_is_enabled(struct clk_hw *hw)
{
	struct rtl819x_wlan_hw *wlan = to_rtl819x_wlan_hw(hw);

	return readl(wlan->base + RTL819X_REG_WLAN_CTRL) &
	       RTL819X_WLAN_CTRL_STAT;
}

static const struct clk_ops rtl819x_wlan_ops = {
	.enable = rtl819x_wlan_enable,
	.disable = rtl819x_wlan_disable,
	.is_enabled = rtl819x_wlan_is_enabled,
};

static struct clk_hw *rtl819x_register_wlan_gate(void __iomem *base,
						 const char *name)
{
	struct rtl819x_wlan_hw *wlan;
	struct clk_init_data init = {};

	wlan = kzalloc(sizeof(*wlan), GFP_KERNEL);
	if (!wlan)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &rtl819x_wlan_ops;
	init.num_parents = 0;

	wlan->base = base;
	wlan->hw.init = &init;

	if (clk_hw_register(NULL, &wlan->hw))
		return ERR_PTR(-EINVAL);
	return &wlan->hw;
}

static DEFINE_SPINLOCK(rtl819x_clk_lock);

static void __init rtl819x_clk_setup(struct device_node *np)
{
	struct clk_hw_onecell_data *clk_data;
	struct clk_hw *lx_hw, *uart_hw, *spi_hw;
	struct clk_hw *pll_hw, *cpu_div_hw, *cpu_hw;
	struct clk_hw *wlan_hw, *xtal_hw, *swcore_hw;
	unsigned long xtal_rate;
	static const char *const cpu_parents[] = { "cpu_div", "lx" };
	static const char *cpu_div_parent = "pll";
	void __iomem *base;

	base = of_iomap(np, 0);
	if (!base) {
		pr_err("clk: cannot map the system controller registers\n");
		return;
	}

	/* Crystal rate, straight off the strap; the WLAN core needs it */
	xtal_rate = readl(base + RTL819X_REG_HW_STRAP) & RTL819X_STRAP_XTAL_40MHZ ?
		    RTL819X_XTAL_40MHZ : RTL819X_XTAL_25MHZ;
	xtal_hw = clk_hw_register_fixed_rate(NULL, "wifi_ref", NULL, 0,
					     xtal_rate);
	if (IS_ERR(xtal_hw))
		goto err_unmap;

	/* WLAN clock/function gate; without it the core registers read 0 */
	wlan_hw = rtl819x_register_wlan_gate(base, "wlan");
	if (IS_ERR(wlan_hw))
		goto err_unmap;

	/* Fixed LX bus clock: no runtime programmable divider was found */
	lx_hw = clk_hw_register_fixed_rate(NULL, "lx", NULL, 0,
					   RTL819X_LX_CLK_RATE);
	if (IS_ERR(lx_hw))
		goto err_unmap;

	/*
	 * Switch core gate, only meant to pulse the clock during the core
	 * reset.  CLK_IGNORE_UNUSED lets the framework leave the core clocked
	 * when no consumer holds a reference.
	 */
	swcore_hw = clk_hw_register_gate(NULL, "swcore", "lx",
					 CLK_IGNORE_UNUSED,
					 base + RTL819X_REG_SYS_CLK_MAG,
					 ffs(RTL819X_SWCORE_EN) - 1, 0,
					 &rtl819x_clk_lock);
	if (IS_ERR(swcore_hw))
		goto err_unmap;

	/* UART input clock: LX / 2 == 100 MHz in the vendor SDK setup */
	uart_hw = clk_hw_register_fixed_factor(NULL, "uart", "lx", 0, 1, 2);
	if (IS_ERR(uart_hw))
		goto err_unmap;

	/* SPI NOR controller clock as set up by the bootloader */
	spi_hw = clk_hw_register_fixed_rate(NULL, "spi", NULL, 0,
					    RTL819X_SPI_CLK_RATE);
	if (IS_ERR(spi_hw))
		goto err_unmap;

	pll_hw = rtl819x_register_pll(base, "pll");
	if (IS_ERR(pll_hw))
		goto err_unmap;

	cpu_div_hw = clk_hw_register_divider_table(NULL, "cpu_div",
						   cpu_div_parent, 0,
						   base + RTL819X_REG_HW_STRAP,
						   RTL819X_STRAP_CPU_DIV_SHIFT,
						   RTL819X_STRAP_CPU_DIV_WIDTH,
						   CLK_DIVIDER_READ_ONLY,
						   rtl819x_cpu_div_table, NULL);
	if (IS_ERR(cpu_div_hw))
		goto err_unmap;

	cpu_hw = clk_hw_register_mux(NULL, "cpu", cpu_parents, 2, 0,
				     base + RTL819X_REG_HW_STRAP,
				     ffs(RTL819X_STRAP_CPU_BYPASS) - 1,
				     1, CLK_MUX_READ_ONLY, NULL);
	if (IS_ERR(cpu_hw))
		goto err_unmap;

	clk_data = kzalloc(struct_size(clk_data, hws, RTL8197F_CLK_NR_CLKS),
			   GFP_KERNEL);
	if (!clk_data) {
		pr_err("clk: cannot allocate the clock table\n");
		goto err_unmap;
	}

	clk_data->num = RTL8197F_CLK_NR_CLKS;
	clk_data->hws[RTL8197F_CLK_CPU] = cpu_hw;
	clk_data->hws[RTL8197F_CLK_LX] = lx_hw;
	clk_data->hws[RTL8197F_CLK_UART] = uart_hw;
	clk_data->hws[RTL8197F_CLK_SPI] = spi_hw;
	clk_data->hws[RTL8197F_CLK_WLAN] = wlan_hw;
	clk_data->hws[RTL8197F_CLK_WIFI_REF] = xtal_hw;
	clk_data->hws[RTL8197F_CLK_SWCORE] = swcore_hw;

	if (of_clk_add_hw_provider(np, of_clk_hw_onecell_get, clk_data))
		pr_err("clk: failed to add the clock provider\n");
	return;

err_unmap:
	iounmap(base);
}

CLK_OF_DECLARE(rtl819x_clk, "realtek,rtl8197f-clock", rtl819x_clk_setup);
