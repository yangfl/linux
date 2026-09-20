// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026 David Yang <mmyangfl@gmail.com>
 *
 * Realtek RTL819x SoC timer driver
 *
 * The RTL8197F core raises the CP0 Compare condition only as Cause.TI and
 * never asserts an IP line, so the r4k counter cannot drive the clock
 * event device.  Use the SoC timer block (TIMER0) instead; the r4k counter
 * keeps running as a clocksource, which is why Cause.DC must be cleared.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <asm/mipsregs.h>

#define TC0_DATA		0x00
#define TCCNR			0x10
#define TC0_EN			BIT(31)
#define TC0_MODE_TIMER		BIT(30)
#define TCIR			0x14
#define TC0_IE			BIT(31)
#define TC0_IP			BIT(29)
#define CDBR			0x18
#define CDBR_DIV		GENMASK(31, 16)

/*
 * The timer input is pre-divided by CDBR and the counter runs at the
 * pre-divided clock times 2^tc_data_offset (BSP_TCD_OFFSET), i.e. the
 * programmed period unit is a 2^-offset fraction of a pre-divided tick.
 * Vendor BSPs use 4 on RTL8197F/RTL8198C and 8 on
 * RTL8196E/RTL8197D/RTL8881A.
 */
struct rtl819x_timer_data {
	u8 tc_data_offset;
};

struct rtl819x_timer {
	void __iomem *base;
	u32 tick_hz;
	u8 tc_data_offset;
	struct clock_event_device clkevt;
};

static const struct rtl819x_timer_data rtl8197f_timer_data = {
	.tc_data_offset = 4,
};

static const struct of_device_id rtl819x_timer_of_match[] = {
	{ .compatible = "realtek,rtl8197f-timer", .data = &rtl8197f_timer_data },
	{ /* sentinel */ }
};

static int rtl819x_timer_shutdown(struct clock_event_device *dev)
{
	struct rtl819x_timer *timer = container_of(dev, struct rtl819x_timer,
						   clkevt);

	writel(readl(timer->base + TCCNR) & ~(TC0_EN | TC0_MODE_TIMER),
	       timer->base + TCCNR);
	writel(readl(timer->base + TCIR) & ~TC0_IE, timer->base + TCIR);

	return 0;
}

static int rtl819x_timer_set_oneshot(struct clock_event_device *dev)
{
	struct rtl819x_timer *timer = container_of(dev, struct rtl819x_timer,
						   clkevt);

	writel(readl(timer->base + TCIR) | TC0_IE, timer->base + TCIR);

	return 0;
}

static int rtl819x_timer_set_next_event(unsigned long cycles,
					struct clock_event_device *dev)
{
	struct rtl819x_timer *timer = container_of(dev, struct rtl819x_timer,
						   clkevt);

	writel(cycles << timer->tc_data_offset, timer->base + TC0_DATA);
	writel(readl(timer->base + TCCNR) | TC0_EN | TC0_MODE_TIMER,
	       timer->base + TCCNR);
	writel(readl(timer->base + TCIR) | TC0_IE, timer->base + TCIR);

	return 0;
}

static int rtl819x_timer_set_periodic(struct clock_event_device *dev)
{
	struct rtl819x_timer *timer = container_of(dev, struct rtl819x_timer,
						   clkevt);

	writel((timer->tick_hz / HZ) << timer->tc_data_offset,
	       timer->base + TC0_DATA);
	writel(readl(timer->base + TCCNR) | TC0_EN | TC0_MODE_TIMER,
	       timer->base + TCCNR);
	writel(readl(timer->base + TCIR) | TC0_IE, timer->base + TCIR);

	return 0;
}

static irqreturn_t rtl819x_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *dev = dev_id;
	struct rtl819x_timer *timer = container_of(dev, struct rtl819x_timer,
						   clkevt);

	/*
	 * Acknowledge the timer interrupt (write-one-to-clear) without
	 * touching the interrupt-enable bit in the same register.
	 */
	writel(TC0_IE | TC0_IP, timer->base + TCIR);

	dev->event_handler(dev);

	return IRQ_HANDLED;
}

static int __init rtl819x_timer_init(struct device_node *node)
{
	struct rtl819x_timer *timer;
	const struct rtl819x_timer_data *data;
	void __iomem *base;
	unsigned int div;
	struct clk *clk;
	int irq, ret;

	data = of_match_node(rtl819x_timer_of_match, node)->data;
	base = of_iomap(node, 0);
	if (!base) {
		pr_err("unable to map registers\n");
		return -ENXIO;
	}

	irq = of_irq_get(node, 0);
	if (irq <= 0) {
		pr_err("unable to get interrupt\n");
		ret = irq ?: -EINVAL;
		goto err_unmap;
	}

	timer = kzalloc(sizeof(*timer), GFP_KERNEL);
	if (!timer) {
		ret = -ENOMEM;
		goto err_unmap;
	}
	timer->base = base;
	timer->tc_data_offset = data->tc_data_offset;

	/*
	 * The r4k counter is the clocksource, so it must not be left
	 * disabled through Cause.DC.
	 */
	change_c0_cause(CAUSEF_DC, 0);

	clk = of_clk_get(node, 0);
	if (IS_ERR(clk)) {
		u32 rate;

		if (of_property_read_u32(node, "clock-frequency", &rate)) {
			pr_err("no clock or clock-frequency given\n");
			ret = PTR_ERR(clk);
			goto err_free;
		}
		timer->tick_hz = rate;
	} else {
		timer->tick_hz = clk_get_rate(clk);
		/*
		 * Keep the clock enabled for the lifetime of the timer: once
		 * the SoC clock controller grows gateable parents, the timer
		 * must not run on a disabled gate.
		 */
		if (clk_prepare_enable(clk)) {
			pr_err("unable to enable clock\n");
			clk_put(clk);
			ret = -EINVAL;
			goto err_free;
		}
	}

	/*
	 * Program the shared pre-divider.  This block owns CDBR: the
	 * clocksource probe runs before any platform driver (in particular
	 * the watchdog, which divides the same pre-divider output) and a
	 * bootloader-provided divider cannot be assumed to leave the
	 * watchdog a usefully long period, so program it for a fixed
	 * ~200 kHz pre-divided input.
	 */
	div = DIV_ROUND_UP(timer->tick_hz, 200000);
	writel(FIELD_PREP(CDBR_DIV, div), base + CDBR);
	timer->tick_hz = DIV_ROUND_UP(timer->tick_hz, div);

	timer->clkevt.name = node->name;
	timer->clkevt.features = CLOCK_EVT_FEAT_PERIODIC |
				 CLOCK_EVT_FEAT_ONESHOT;
	timer->clkevt.rating = 300;
	timer->clkevt.cpumask = cpumask_of(smp_processor_id());
	timer->clkevt.set_state_shutdown = rtl819x_timer_shutdown;
	timer->clkevt.set_state_periodic = rtl819x_timer_set_periodic;
	timer->clkevt.set_state_oneshot = rtl819x_timer_set_oneshot;
	timer->clkevt.set_next_event = rtl819x_timer_set_next_event;

	writel(readl(base + TCIR) & ~TC0_IE, base + TCIR);

	ret = request_irq(irq, rtl819x_timer_interrupt, IRQF_TIMER,
			  node->name, &timer->clkevt);
	if (ret) {
		pr_err("unable to request irq %d\n", irq);
		goto err_free;
	}

	clockevents_config_and_register(&timer->clkevt, timer->tick_hz,
					1 << timer->tc_data_offset,
					0xffff << timer->tc_data_offset);

	pr_info("timer at %pOF, %u Hz\n", node, timer->tick_hz);

	return 0;

err_free:
	kfree(timer);
err_unmap:
	iounmap(base);
	return ret;
}
TIMER_OF_DECLARE(realtek_rtl819x_timer, "realtek,rtl8197f-timer",
		 rtl819x_timer_init);
