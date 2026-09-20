// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTL819x (RTL8197F) SoC GPIO controller driver
 *
 * The RTL8197F provides two identical banks of 32 GPIOs, known in the
 * vendor documentation as ports ABCD (bank 0) and EFGH (bank 1).  Each
 * bank has its own register block and its own interrupt line into the
 * SoC interrupt controller (inputs 16 and 17 respectively).
 *
 * Each pin supports edge triggered interrupts only (rising, falling or
 * both edges).  Level triggered interrupts are not implemented in
 * hardware.
 *
 * There is no pinctrl hardware on this SoC: pin muxing is done by
 * writing to global system registers, which is out of the scope of this
 * driver.  Some pins are also muxed with the switch LED outputs.
 *
 * Copyright (C) 2019 Gaspare Bruno <gaspare@anlix.io>
 * Copyright (C) 2015 Alban Bedel <albeu@free.fr>
 * Copyright (C) David Yang <mmyangfl@gmail.com>
 */

#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/generic.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>

/* Register offsets within a bank */
#define RTL819X_GPIO_REG_CNR		0x00	/* pin control: 1 = alternate function */
#define RTL819X_GPIO_REG_DIR		0x08	/* direction: 1 = output */
#define RTL819X_GPIO_REG_DAT		0x0c	/* data in/out */
#define RTL819X_GPIO_REG_ISR		0x10	/* interrupt status, W1C */
#define RTL819X_GPIO_REG_IMR_LO		0x14	/* interrupt mode, pins 0-15 */
#define RTL819X_GPIO_REG_IMR_HI		0x18	/* interrupt mode, pins 16-31 */

#define RTL819X_GPIO_NR			32

/*
 * Interrupt mode is encoded as a 2 bit field per pin, spread over the
 * IMR_LO/IMR_HI registers:
 *   0 = disabled, 1 = falling edge, 2 = rising edge, 3 = both edges
 */
#define RTL819X_GPIO_IMR_NONE		0x0
#define RTL819X_GPIO_IMR_FALLING	0x1
#define RTL819X_GPIO_IMR_RISING		0x2
#define RTL819X_GPIO_IMR_BOTH		0x3

/*
 * Not gpio-realtek-otto.c: the Otto driver derives its register byte order
 * from the same flag that selects the port order (realtek_gpio_probe() picks
 * ioread32be plus a swapped port map unless GPIO_PORTS_REVERSED), so it has no
 * combination for little-endian registers in normal port order, and it defines
 * REALTEK_GPIO_REG_CNR without ever using it, so there is no place for the
 * "clear the pin-select bit when the pin is requested as GPIO" behaviour this
 * SoC needs.  Splitting those two axes changes behaviour for every Otto board,
 * which is a series of its own with that hardware in the loop.
 */

struct rtl819x_gpio_ctrl {
	struct gpio_generic_chip chip;
	void __iomem *base;
	u8 imr_mode[RTL819X_GPIO_NR];
};

static struct rtl819x_gpio_ctrl *
irq_data_to_rtl819x_gpio(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct gpio_generic_chip *gen_gc = to_gpio_generic_chip(gc);

	return container_of(gen_gc, struct rtl819x_gpio_ctrl, chip);
}

static void rtl819x_gpio_update_bits(struct rtl819x_gpio_ctrl *ctrl,
				     unsigned int reg, u32 mask, u32 bits)
{
	u32 val = readl(ctrl->base + reg);

	val &= ~mask;
	val |= bits & mask;
	writel(val, ctrl->base + reg);
}

/*
 * Clearing the CNR bit routes the pin to the GPIO controller.  The CNR
 * bit is set by default (or by the bootloader) for pins muxed to other
 * functions.
 */
static int rtl819x_gpio_request(struct gpio_chip *gc, unsigned int offset)
{
	struct gpio_generic_chip *gen_gc = to_gpio_generic_chip(gc);
	struct rtl819x_gpio_ctrl *ctrl =
		container_of(gen_gc, struct rtl819x_gpio_ctrl, chip);

	guard(gpio_generic_lock_irqsave)(&ctrl->chip);
	rtl819x_gpio_update_bits(ctrl, RTL819X_GPIO_REG_CNR,
				 BIT(offset), 0);

	return 0;
}

static void rtl819x_gpio_irq_ack(struct irq_data *data)
{
	struct rtl819x_gpio_ctrl *ctrl = irq_data_to_rtl819x_gpio(data);
	u32 mask = BIT(irqd_to_hwirq(data));

	writel(mask, ctrl->base + RTL819X_GPIO_REG_ISR);
}

static void rtl819x_gpio_irq_mask(struct irq_data *data)
{
	struct rtl819x_gpio_ctrl *ctrl = irq_data_to_rtl819x_gpio(data);
	unsigned int offset = irqd_to_hwirq(data);
	u32 mask = 0x3 << (2 * (offset % 16));

	scoped_guard(gpio_generic_lock_irqsave, &ctrl->chip)
		rtl819x_gpio_update_bits(ctrl,
			offset < 16 ? RTL819X_GPIO_REG_IMR_LO :
				      RTL819X_GPIO_REG_IMR_HI,
			mask, RTL819X_GPIO_IMR_NONE << (2 * (offset % 16)));

	gpiochip_disable_irq(&ctrl->chip.gc, offset);
}

static void rtl819x_gpio_irq_unmask(struct irq_data *data)
{
	struct rtl819x_gpio_ctrl *ctrl = irq_data_to_rtl819x_gpio(data);
	unsigned int offset = irqd_to_hwirq(data);
	u32 mask = 0x3 << (2 * (offset % 16));
	unsigned int reg;
	u32 mode;

	gpiochip_enable_irq(&ctrl->chip.gc, offset);

	/* Restore the mode stored by irq_set_type */
	reg = offset < 16 ? RTL819X_GPIO_REG_IMR_LO :
			    RTL819X_GPIO_REG_IMR_HI;
	mode = ctrl->imr_mode[offset];

	guard(gpio_generic_lock_irqsave)(&ctrl->chip);
	rtl819x_gpio_update_bits(ctrl, reg, mask, mode << (2 * (offset % 16)));
}

static int rtl819x_gpio_irq_set_type(struct irq_data *data,
				     unsigned int flow_type)
{
	struct rtl819x_gpio_ctrl *ctrl = irq_data_to_rtl819x_gpio(data);
	unsigned int offset = irqd_to_hwirq(data);
	unsigned int reg, mode;
	u32 mask;

	switch (flow_type) {
	case IRQ_TYPE_EDGE_RISING:
		mode = RTL819X_GPIO_IMR_RISING;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		mode = RTL819X_GPIO_IMR_FALLING;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		mode = RTL819X_GPIO_IMR_BOTH;
		break;
	default:
		/* Level triggered interrupts are not supported */
		return -EINVAL;
	}

	mask = 0x3 << (2 * (offset % 16));
	reg = offset < 16 ? RTL819X_GPIO_REG_IMR_LO :
			    RTL819X_GPIO_REG_IMR_HI;

	guard(gpio_generic_lock_irqsave)(&ctrl->chip);
	rtl819x_gpio_update_bits(ctrl, reg, mask, mode << (2 * (offset % 16)));
	ctrl->imr_mode[offset] = mode;

	return 0;
}

static const struct irq_chip rtl819x_gpio_irqchip = {
	.name = "gpio-rtl819x",
	.irq_ack = rtl819x_gpio_irq_ack,
	.irq_mask = rtl819x_gpio_irq_mask,
	.irq_unmask = rtl819x_gpio_irq_unmask,
	.irq_set_type = rtl819x_gpio_irq_set_type,
	.flags = IRQCHIP_IMMUTABLE | IRQCHIP_SET_TYPE_MASKED,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static void rtl819x_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	struct gpio_generic_chip *gen_gc = to_gpio_generic_chip(gc);
	struct rtl819x_gpio_ctrl *ctrl =
		container_of(gen_gc, struct rtl819x_gpio_ctrl, chip);
	unsigned long pending, enabled = 0;
	unsigned int irq;
	u32 status, imr_lo, imr_hi;

	chained_irq_enter(irqchip, desc);

	scoped_guard(gpio_generic_lock_irqsave, &ctrl->chip) {
		status = readl(ctrl->base + RTL819X_GPIO_REG_ISR);

		/*
		 * The status register latches edges of disabled pins too,
		 * so filter out the pins whose interrupt mode is off, and
		 * acknowledge them all.  IMR_LO/IMR_HI are two registers of
		 * 16 pins with a 2-bit mode each; keep them apart, shifting
		 * a merged value would wrap for pins >= 16.
		 */
		imr_lo = readl(ctrl->base + RTL819X_GPIO_REG_IMR_LO);
		imr_hi = readl(ctrl->base + RTL819X_GPIO_REG_IMR_HI);
		pending = status;
		writel(status, ctrl->base + RTL819X_GPIO_REG_ISR);
	}

	for (irq = 0; irq < gc->ngpio; irq++) {
		u32 imr = irq < 16 ? imr_lo >> (2 * irq)
				   : imr_hi >> (2 * (irq - 16));

		if (imr & 0x3)
			enabled |= BIT(irq);
	}

	pending &= enabled;

	for_each_set_bit(irq, &pending, gc->ngpio)
		generic_handle_domain_irq(gc->irq.domain, irq);

	chained_irq_exit(irqchip, desc);
}

static const struct of_device_id rtl819x_gpio_of_match[] = {
	{ .compatible = "realtek,rtl8197f-gpio" },
	{},
};
MODULE_DEVICE_TABLE(of, rtl819x_gpio_of_match);

static int rtl819x_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_generic_chip_config config;
	struct gpio_irq_chip *girq;
	struct rtl819x_gpio_ctrl *ctrl;
	int err;

	ctrl = devm_kzalloc(dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	ctrl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ctrl->base))
		return PTR_ERR(ctrl->base);

	config = (struct gpio_generic_chip_config) {
		.dev = dev,
		.sz = 4,
		.dat = ctrl->base + RTL819X_GPIO_REG_DAT,
		.dirout = ctrl->base + RTL819X_GPIO_REG_DIR,
	};

	err = gpio_generic_chip_init(&ctrl->chip, &config);
	if (err)
		return dev_err_probe(dev, err,
				     "failed to initialize generic GPIO chip\n");

	ctrl->chip.gc.ngpio = RTL819X_GPIO_NR;
	ctrl->chip.gc.request = rtl819x_gpio_request;

	if (device_property_read_bool(dev, "interrupt-controller")) {
		girq = &ctrl->chip.gc.irq;
		gpio_irq_chip_set_chip(girq, &rtl819x_gpio_irqchip);
		girq->parent_handler = rtl819x_gpio_irq_handler;
		girq->num_parents = 1;
		girq->parents = devm_kcalloc(dev, 1, sizeof(*girq->parents),
					     GFP_KERNEL);
		if (!girq->parents)
			return -ENOMEM;
		err = platform_get_irq(pdev, 0);
		if (err < 0)
			return err;
		girq->parents[0] = err;
		/*
		 * gpiolib forbids a default trigger on DT-probed chips
		 * (it WARNs and resets to NONE), so children must name
		 * their edge type explicitly; a child without one leaves
		 * its IMR bit clear and stays silent.
		 */
		girq->default_type = IRQ_TYPE_NONE;
		girq->handler = handle_edge_irq;
	}

	return devm_gpiochip_add_data(dev, &ctrl->chip.gc, ctrl);
}

static struct platform_driver rtl819x_gpio_driver = {
	.driver = {
		.name = "realtek-rtl819x-gpio",
		.of_match_table	= rtl819x_gpio_of_match,
	},
	.probe = rtl819x_gpio_probe,
};
module_platform_driver(rtl819x_gpio_driver);

MODULE_DESCRIPTION("Realtek RTL819x SoC GPIO controller driver");
MODULE_AUTHOR("David Yang <mmyangfl@gmail.com>");
MODULE_LICENSE("GPL");
