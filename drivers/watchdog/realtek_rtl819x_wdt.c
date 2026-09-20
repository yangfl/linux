// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTL819x MIPS platform watchdog
 *
 * Watchdog timer found on the RTL8196E/8197D/8197F series of SoCs, sharing a
 * register block with the SoC's programmable timers. The block exposes a clock
 * divider register, prescaling the LX bus clock, and the watchdog control
 * register itself.
 *
 * Counter scaling and timeouts:
 * - The counter runs at lx_clk / PRESCALE ticks
 * - OVSEL: logarithmic selector, the counter expires after 2^(15 + OVSEL)
 *   ticks, with the 4-bit OVSEL index split over two bit fields
 * - Writing the disable key to the control register stops the counter;
 *   writing any other value in its place starts it
 *
 * System restart uses the same timer: enabling it with the shortest period
 * (OVSEL = 0, i.e. 2^15 prescaled ticks) resets the SoC, and the restart
 * handler returns without waiting for the reset to happen.
 */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>

/*
 * Register block at 0x18003118 (physical), containing the shared timer clock
 * divider (BSP_CDBR) and the watchdog control register (BSP_WDTCNR). Offsets
 * and magic values per Realtek SDK bspchip.h and bsp/timer.c.
 */
#define RTL819X_WDT_REG_PRESCALE	0x0
#define RTL819X_WDT_PRESCALE		GENMASK(31, 16)

#define RTL819X_WDT_REG_CTRL		0x4

/*
 * Writing 0xa5 to bits 31:24 disables the watchdog; any other value in these
 * bits (e.g. 0) enables it. See Realtek SDK bsp/timer.c: bsp_enable_watchdog()
 * writes 0x00240000, bsp_disable_watchdog() writes 0xa5240000.
 */
#define RTL819X_WDT_CTRL_DISABLE	GENMASK(31, 24)
#define RTL819X_WDT_DISABLE_KEY		0xa5000000

/*
 * Writing 1 clears the counter and reloads it, i.e. pings the watchdog. See
 * Realtek SDK bsp/timer.c watchdog_kick proc: sets bit 23 of the control
 * register (_WDTKICK_ in the SDK drivers).
 */
#define RTL819X_WDT_CTRL_KICK		BIT(23)

/*
 * Logarithmic timeout selector, stored as two 2-bit fields: the low bits at
 * bit 21 and the high bits at bit 17 of the control register (Realtek SDK
 * bsp/timer.c watchdog proc: oversel_l << 21 | oversel_h << 17). The combined
 * 4-bit index selects an expiry after 2^(15 + index) prescaled ticks.
 */
#define RTL819X_WDT_CTRL_OVSEL_L	GENMASK(22, 21)
#define RTL819X_WDT_CTRL_OVSEL_H	GENMASK(18, 17)

#define RTL819X_WDT_OVSEL_MAX		9
#define RTL819X_WDT_OVSEL_TICKS(index)	BIT(15 + (index))

/*
 * Per-SoC operational style. The register block is shared by the whole
 * RTL8196E/8197D/8197F family, but how the counter is started differs:
 *
 * - RTL8196E/8197D/8881A/8198C (old style): the counter is started by the
 *   bit 23 kick itself, e.g. rtl_gpio_98C.c brings the watchdog up with a
 *   plain "REG32(0xB800311c) |= 1 << 23" after the prescaler and OVSEL are
 *   programmed. Whether the hardware clears the kick bit by itself is not
 *   documented in the SDK; the vendor code always uses a read-modify-write
 *   OR (watchdog_kick proc semantics), and so does the driver.
 * - RTL8197F: bsp_enable_watchdog() starts the counter by writing the
 *   control register with the key bits clear (0x00240000); the bit 23 kick
 *   is only a reload, issued as a second write after the enable.
 */
struct rtl819x_wdt_soc {
	bool kick_starts_counter; /* old style: the bit 23 kick also starts it */
};

static const struct rtl819x_wdt_soc rtl8197f_wdt_soc = {
	.kick_starts_counter = false,
};

struct rtl819x_wdt {
	struct watchdog_device wdev;
	void __iomem *base;
	unsigned long tick_rate;
	unsigned int ovsel;	/* last period chosen for wdev.timeout */
	const struct rtl819x_wdt_soc *soc;
};

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout,
		 "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static u32 rtl819x_wdt_read(struct rtl819x_wdt *wdt, unsigned int reg)
{
	return readl(wdt->base + reg);
}

static void rtl819x_wdt_write(struct rtl819x_wdt *wdt, unsigned int reg, u32 val)
{
	writel(val, wdt->base + reg);
}

static unsigned int rtl819x_wdt_period_ticks(unsigned int ovsel)
{
	return RTL819X_WDT_OVSEL_TICKS(min(ovsel, RTL819X_WDT_OVSEL_MAX));
}

static void rtl819x_wdt_set_ovsel(struct rtl819x_wdt *wdt, unsigned int ovsel)
{
	u32 val;

	val = rtl819x_wdt_read(wdt, RTL819X_WDT_REG_CTRL);
	val &= ~(RTL819X_WDT_CTRL_DISABLE | RTL819X_WDT_CTRL_OVSEL_L |
		 RTL819X_WDT_CTRL_OVSEL_H);
	val |= FIELD_PREP(RTL819X_WDT_CTRL_OVSEL_L, ovsel & 0x3) |
	       FIELD_PREP(RTL819X_WDT_CTRL_OVSEL_H, (ovsel >> 2) & 0x3);

	if (wdt->soc->kick_starts_counter) {
		/*
		 * Old style: the bit 23 kick both reloads and runs the
		 * counter, a single write is enough.
		 */
		rtl819x_wdt_write(wdt, RTL819X_WDT_REG_CTRL,
				  val | RTL819X_WDT_CTRL_KICK);
	} else {
		/*
		 * Enabling with a non-zero value leaves the counter running;
		 * the kick write afterwards reloads it.
		 */
		rtl819x_wdt_write(wdt, RTL819X_WDT_REG_CTRL, val);
		rtl819x_wdt_write(wdt, RTL819X_WDT_REG_CTRL,
				  val | RTL819X_WDT_CTRL_KICK);
	}
}

static int rtl819x_wdt_ping(struct watchdog_device *wdev)
{
	struct rtl819x_wdt *wdt = watchdog_get_drvdata(wdev);

	rtl819x_wdt_write(wdt, RTL819X_WDT_REG_CTRL,
			  rtl819x_wdt_read(wdt, RTL819X_WDT_REG_CTRL) |
			  RTL819X_WDT_CTRL_KICK);

	return 0;
}

static int rtl819x_wdt_start(struct watchdog_device *wdev)
{
	struct rtl819x_wdt *wdt = watchdog_get_drvdata(wdev);

	/*
	 * Reuse the selector chosen by set_timeout(): re-deriving it from
	 * the (rounded-up) wdev->timeout would bump the exponent one step
	 * per start/set cycle.
	 */
	rtl819x_wdt_set_ovsel(wdt, wdt->ovsel);

	return 0;
}

static int rtl819x_wdt_stop(struct watchdog_device *wdev)
{
	struct rtl819x_wdt *wdt = watchdog_get_drvdata(wdev);
	u32 val;

	/*
	 * Kick the watchdog before disabling it: writing the disable key while
	 * the counter is about to expire may trigger an immediate reset (see
	 * Realtek SDK drivers/char/rtl_gpio_98C.c rtl_gpio_write() and
	 * bsp/timer.c bsp_disable_watchdog(), both kick before disabling).
	 */
	rtl819x_wdt_ping(wdev);

	val = rtl819x_wdt_read(wdt, RTL819X_WDT_REG_CTRL);
	val |= RTL819X_WDT_DISABLE_KEY;
	rtl819x_wdt_write(wdt, RTL819X_WDT_REG_CTRL, val);

	return 0;
}

static int rtl819x_wdt_set_timeout(struct watchdog_device *wdev, unsigned int timeout)
{
	struct rtl819x_wdt *wdt = watchdog_get_drvdata(wdev);
	unsigned int ovsel = 0;
	unsigned int ticks;

	ticks = timeout * wdt->tick_rate;
	while (ovsel < RTL819X_WDT_OVSEL_MAX &&
	       rtl819x_wdt_period_ticks(ovsel) < ticks)
		ovsel++;

	wdt->ovsel = ovsel;
	wdev->timeout = DIV_ROUND_UP(rtl819x_wdt_period_ticks(ovsel),
				     wdt->tick_rate);

	if (watchdog_active(wdev))
		rtl819x_wdt_set_ovsel(wdt, ovsel);

	return 0;
}

static int rtl819x_wdt_restart(struct watchdog_device *wdev, unsigned long action,
			       void *data)
{
	struct rtl819x_wdt *wdt = watchdog_get_drvdata(wdev);

	/*
	 * Enable the watchdog with the shortest period. The shared prescaler
	 * (BSP_CDBR) must keep a non-zero divider: writing 0 stops the counter
	 * entirely and the SoC never resets. This mirrors the SDK mach code,
	 * which leaves the prescaler programmed and writes 0 to the control
	 * register before waiting for the reset.
	 */
	rtl819x_wdt_write(wdt, RTL819X_WDT_REG_CTRL, 0);

	/*
	 * A short bounded wait for the counter to expire; the real
	 * guarantee is the caller spinning on the reset, not this delay.
	 */
	mdelay(3 * DIV_ROUND_UP(RTL819X_WDT_OVSEL_TICKS(0), wdt->tick_rate));

	return 0;
}

static const struct watchdog_info rtl819x_wdt_info = {
	.identity = "Realtek RTL819x watchdog timer",
	.options = WDIOF_KEEPALIVEPING |
		   WDIOF_MAGICCLOSE |
		   WDIOF_SETTIMEOUT,
};

static const struct watchdog_ops rtl819x_wdt_ops = {
	.owner = THIS_MODULE,
	.start = rtl819x_wdt_start,
	.stop = rtl819x_wdt_stop,
	.ping = rtl819x_wdt_ping,
	.set_timeout = rtl819x_wdt_set_timeout,
	.restart = rtl819x_wdt_restart,
};

static int rtl819x_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct rtl819x_wdt_soc *soc;
	struct watchdog_device *wdev;
	struct clk *clk;
	struct rtl819x_wdt *wdt;
	unsigned int prescale, ovsel, ticks;
	int ret;

	soc = of_device_get_match_data(dev);
	if (!soc)
		return dev_err_probe(dev, -EINVAL, "No compatible SoC data\n");

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->soc = soc;

	wdt->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(wdt->base))
		return PTR_ERR(wdt->base);

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "Failed to get clock\n");

	/*
	 * The pre-divider (CDBR) is shared with the timer block, which owns
	 * it and programs it for a ~200 kHz pre-divided input before this
	 * driver probes; reuse its setting. The prescaler field is 16 bits
	 * wide.
	 */
	prescale = FIELD_GET(RTL819X_WDT_PRESCALE,
			     rtl819x_wdt_read(wdt, RTL819X_WDT_REG_PRESCALE));
	if (!prescale) {
		prescale = DIV_ROUND_UP(clk_get_rate(clk),
					RTL819X_WDT_OVSEL_TICKS(0));
		if (prescale == 0 || prescale > FIELD_MAX(RTL819X_WDT_PRESCALE))
			return dev_err_probe(dev, -EINVAL,
					     "Unsupported clock rate\n");

		rtl819x_wdt_write(wdt, RTL819X_WDT_REG_PRESCALE,
				  FIELD_PREP(RTL819X_WDT_PRESCALE, prescale));
	}
	wdt->tick_rate = clk_get_rate(clk) / prescale;

	/*
	 * Kick before disabling: the bootloader may have left the counter
	 * close to expiry, and the disable key alone does not reload it.
	 */
	rtl819x_wdt_write(wdt, RTL819X_WDT_REG_CTRL,
			  rtl819x_wdt_read(wdt, RTL819X_WDT_REG_CTRL) |
			  RTL819X_WDT_CTRL_KICK);
	rtl819x_wdt_write(wdt, RTL819X_WDT_REG_CTRL, RTL819X_WDT_DISABLE_KEY);

	wdev = &wdt->wdev;
	wdev->info = &rtl819x_wdt_info;
	wdev->ops = &rtl819x_wdt_ops;
	wdev->parent = dev;

	/*
	 * With a ~164 ms base period (OVSEL = 0, see the header note
	 * above) the smallest selectable timeout is already whole
	 * seconds once rounded up.
	 */
	wdev->min_timeout = DIV_ROUND_UP(rtl819x_wdt_period_ticks(0),
					 wdt->tick_rate);
	wdev->max_hw_heartbeat_ms =
		DIV_ROUND_UP_ULL((u64) rtl819x_wdt_period_ticks(RTL819X_WDT_OVSEL_MAX) *
				 MSEC_PER_SEC, wdt->tick_rate);
	wdev->timeout = min(30U, wdev->max_hw_heartbeat_ms / 1000);

	watchdog_set_drvdata(wdev, wdt);
	watchdog_init_timeout(wdev, 0, dev);

	/* start() reuses the selector, so seed it for the initial timeout */
	ticks = wdev->timeout * wdt->tick_rate;
	for (ovsel = 0; ovsel < RTL819X_WDT_OVSEL_MAX &&
	     rtl819x_wdt_period_ticks(ovsel) < ticks; ovsel++)
		;
	wdt->ovsel = ovsel;

	watchdog_set_nowayout(wdev, nowayout);
	watchdog_stop_on_reboot(wdev);
	watchdog_set_restart_priority(wdev, 128);

	ret = devm_watchdog_register_device(dev, wdev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register watchdog\n");

	return 0;
}

static const struct of_device_id rtl819x_wdt_ids[] = {
	{ .compatible = "realtek,rtl8197f-wdt", .data = &rtl8197f_wdt_soc },
	{ }
};
MODULE_DEVICE_TABLE(of, rtl819x_wdt_ids);

static struct platform_driver rtl819x_wdt_driver = {
	.probe = rtl819x_wdt_probe,
	.driver = {
		.name = "realtek-rtl819x-watchdog",
		.of_match_table = rtl819x_wdt_ids,
	},
};
module_platform_driver(rtl819x_wdt_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Yang <mmyangfl@gmail.com>");
MODULE_DESCRIPTION("Realtek RTL819x watchdog timer driver");
