// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTL819x SoC interrupt controller driver.
 *
 * The RTL8197F and related little-endian Realtek MIPS router SoCs have a
 * 64-input secondary interrupt controller at 0x18003000, cascaded into the
 * MIPS CPU interrupt controller (mti,cpu-interrupt-controller).  The inputs
 * are split into two banks:
 *
 *   bank 0: inputs 0-31,  masked by GIMR,   reported by GISR
 *   bank 1: inputs 32-63, masked by GIMR2,  reported by GISR2
 *
 * Bank 1 is aggregated onto bank 0 input 31: whenever any unmasked bank 1
 * interrupt is pending, GISR bit 31 is set.  Input 31 has no routing of its
 * own and is not usable as a regular interrupt.
 *
 * Eight routing registers IRR0-IRR7 assign each input to one of the six
 * MIPS CPU interrupt lines IP2-IP7.  Each register covers eight inputs with
 * a 4-bit field per input; the field value is the CPU line number itself
 * (unlike the big-endian Otto RTL83xx controller, whose routing values are
 * biased by one) and 0 disconnects the input.
 *
 * Most inputs are routed to IP2 and handled through the single cascaded
 * parent listed in DT, but the RTL8197F wires a few high-priority sources
 * straight to dedicated CPU lines (IP4-IP7).  The driver therefore supports
 * up to five parent interrupts in DT; each parent handles the inputs whose
 * fixed routing targets its CPU line.
 *
 * Copyright (C) 2026 David Yang <mmyangfl@gmail.com>
 *
 * The geometry (number of banks and inputs) is described per compatible;
 * other members of the family, such as the single-bank RTL8196E, RTL8197D
 * and RTL8881A, only implement bank 0.
 */

#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>

/* Bank 0: global interrupt mask / status, inputs 0-31 */
#define RTL819X_GIMR			0x00
#define RTL819X_GISR			0x04
/* Bank 0 routing registers IRR0-IRR3, inputs 0-31 */
#define RTL819X_IRR0			0x08

/*
 * Bank 1: global interrupt mask / status, inputs 32-63.  Bank-relative
 * offsets, like GIMR/GISR above: bank_reg() adds the bank stride itself,
 * and GIMR2/GISR2 sit at +0x20/+0x24 in absolute terms (GIMR2 holds the
 * CPU SI timer mask, IRR4-IRR7 follow at +0x28..+0x34).
 */
#define RTL819X_GIMR2			0x00
#define RTL819X_GISR2			0x04
/* Bank 1 routing registers IRR4-IRR7, inputs 32-63 */
#define RTL819X_IRR4			0x28

#define RTL819X_IRR_BANK_STRIDE		0x20
#define RTL819X_MAX_INPUTS		64
#define RTL819X_NUM_BANK_INPUTS		32
#define RTL819X_INPUTS_PER_IRR		8

/*
 * Bank 0 input 31 aggregates bank 1.  Its GIMR bit must stay enabled for
 * the cascade to work, so it is permanently reserved and cannot be mapped.
 */
#define RTL819X_AGGREGATOR_INPUT	31

/* MIPS CPU interrupt lines the IRRs can target */
#define RTL819X_CPU_LINE_MIN		2
#define RTL819X_CPU_LINE_MAX		7
#define RTL819X_NUM_CPU_LINES		(RTL819X_CPU_LINE_MAX + 1)

/* Register access helpers, per bank */
#define bank_reg(ictl, bank, reg)	((ictl)->base + (reg) + \
					(bank) * RTL819X_IRR_BANK_STRIDE)

/**
 * struct rtl819x_ictl_data - per-compatible controller geometry
 * @num_inputs: total number of interrupt inputs across all banks
 * @num_banks: number of 32-input banks (1 or 2)
 *
 * The RTL8197F has two banks and 64 inputs; the RTL8196E, RTL8197D and
 * RTL8881A only implement bank 0 with 32 inputs and no aggregator input.
 */
struct rtl819x_ictl_data {
	u32				num_inputs;
	u32				num_banks;
};

/*
 * Not irq-realtek-rtl.c: that driver hard-wires big-endian accessors
 * (readl_be/writel_be throughout), 32 inputs with IRR0..3 in a single bank, and
 * the routing register holding parent_hwirq-1.  This SoC is little-endian with
 * 64 inputs over two banks at a 0x20 stride, inverted IRR order, and the
 * routing cell carrying the IP line, so the shared driver would need an
 * accessor indirection plus the same per-compatible table this file already
 * uses.  That rewrites the path every existing Otto machine takes, so it
 * belongs in a series with that hardware in the loop; until then the two
 * drivers stay separate and the difference is stated here rather than hidden.
 */

static const struct rtl819x_ictl_data rtl8197f_ictl_data = {
	.num_inputs			= 64,
	.num_banks			= 2,
};

static const struct of_device_id rtl819x_ictl_of_match[] = {
	{
		/* RTL8197F: two banks, 64 inputs */
		.compatible	= "realtek,rtl8197f-intc",
		.data		= &rtl8197f_ictl_data,
	},
	{ /* sentinel */ }
};

/**
 * struct rtl819x_ictl - driver private data
 * @data: per-compatible controller geometry
 * @base: remapped register block
 * @lock: protects read-modify-write cycles on the mask registers
 * @domain: IRQ domain covering all inputs
 * @line_mask: per CPU line, bitmask of bank inputs routed to it
 */
struct rtl819x_ictl {
	const struct rtl819x_ictl_data	*data;
	void __iomem			*base;
	raw_spinlock_t			lock;
	struct irq_domain		*domain;
	u32				line_mask[RTL819X_NUM_CPU_LINES][2];
};

struct rtl819x_output {
	struct rtl819x_ictl		*ictl;
	unsigned int			parent_irq;
	unsigned int			line;
};

/*
 * Fixed routing table, mirroring the RTL8197F vendor defaults:
 *
 *	input  9  UART0         -> IP2 (generic)
 *	input 12  USB OTG       -> IP2 (generic)
 *	input 13  USB host      -> IP2 (generic)
 *	input  7  TIMER0         -> IP2 (system tick)
 *	input 15  switch core   -> IP4
 *	input 16  GPIO A-D      -> IP2 (generic)
 *	input 17  GPIO E-H      -> IP2 (generic)
 *	input 21  PCIe          -> IP5
 *	input 29  WLAN MAC      -> IP6
 *	input 47  CPU SI timer  -> IP7
 *
 * The core never asserts the CP0 Compare interrupt on an IP line itself
 * (it only raises Cause.TI); the SoC controller observes it as input 47
 * ("CPU SI timer"), which is routed to IP7 and drives the r4k clock
 * event device through this controller.
 *
 * TODO: express the routing in DT with an interrupts-extended entry per
 * input instead, and drop this hard-coded table.
 */
static const u8 rtl819x_routing[RTL819X_MAX_INPUTS] = {
	[7]  = 2,
	[9]  = 2,
	[12] = 2,
	[13] = 2,
	[15] = 4,
	[16] = 2,
	[17] = 2,
	[21] = 5,
	[29] = 6,
	[47] = 7,
};

static void rtl819x_ictl_mask_bank(struct rtl819x_ictl *ictl,
				   unsigned int bank, unsigned int input,
				   bool mask)
{
	void __iomem *reg;
	u32 val;

	reg = bank_reg(ictl, bank, bank ? RTL819X_GIMR2 : RTL819X_GIMR);

	guard(raw_spinlock_irqsave)(&ictl->lock);
	val = readl(reg);
	if (mask)
		val &= ~BIT(input);
	else
		val |= BIT(input);
	writel(val, reg);
}

static void rtl819x_ictl_mask(struct irq_data *d)
{
	struct rtl819x_ictl *ictl = irq_data_get_irq_chip_data(d);
	unsigned int bank = d->hwirq / RTL819X_NUM_BANK_INPUTS;

	rtl819x_ictl_mask_bank(ictl, bank, d->hwirq % RTL819X_NUM_BANK_INPUTS,
			       true);
}

static void rtl819x_ictl_unmask(struct irq_data *d)
{
	struct rtl819x_ictl *ictl = irq_data_get_irq_chip_data(d);
	unsigned int bank = d->hwirq / RTL819X_NUM_BANK_INPUTS;

	rtl819x_ictl_mask_bank(ictl, bank, d->hwirq % RTL819X_NUM_BANK_INPUTS,
			       false);
}

static struct irq_chip rtl819x_ictl_chip = {
	.name			= "realtek-rtl819x-intc",
	.irq_mask		= rtl819x_ictl_mask,
	.irq_unmask		= rtl819x_ictl_unmask,
};

static int rtl819x_ictl_map(struct irq_domain *d, unsigned int virq,
			    irq_hw_number_t hwirq)
{
	struct rtl819x_ictl *ictl = d->host_data;

	if (hwirq >= ictl->data->num_inputs ||
	    (ictl->data->num_banks > 1 && hwirq == RTL819X_AGGREGATOR_INPUT))
		return -EINVAL;

	irq_set_chip_data(virq, ictl);
	irq_set_chip_and_handler(virq, &rtl819x_ictl_chip, handle_level_irq);

	return 0;
}

static const struct irq_domain_ops rtl819x_ictl_domain_ops = {
	.map	= rtl819x_ictl_map,
	.xlate	= irq_domain_xlate_twocell,
};

static bool rtl819x_ictl_dispatch_bank(struct rtl819x_ictl *ictl,
				       unsigned int bank, u32 mask)
{
	void __iomem *gimr = bank_reg(ictl, bank,
				      bank ? RTL819X_GIMR2 : RTL819X_GIMR);
	void __iomem *gisr = bank_reg(ictl, bank,
				      bank ? RTL819X_GISR2 : RTL819X_GISR);
	unsigned long pending;
	unsigned int input;

	pending = readl(gisr) & readl(gimr) & mask;

	for_each_set_bit(input, &pending, RTL819X_NUM_BANK_INPUTS)
		generic_handle_domain_irq(ictl->domain,
					  bank * RTL819X_NUM_BANK_INPUTS +
					  input);

	return pending;
}

static void rtl819x_ictl_dispatch(struct irq_desc *desc)
{
	struct rtl819x_output *output = irq_desc_get_handler_data(desc);
	struct rtl819x_ictl *ictl = output->ictl;
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long raw, pending;
	bool handled = false;

	chained_irq_enter(chip, desc);

	/*
	 * Bank 1 is always scanned directly: whether input 31 actually
	 * aggregates every unmasked bank 1 pending bit is undocumented,
	 * and the CPU line would stay asserted (spurious per tick) if a
	 * bank 1 source were relied on the aggregator that does not set
	 * it.  Pending is gated by GIMR2, so disconnected inputs are
	 * never seen here.
	 */
	if (ictl->data->num_banks > 1)
		handled |= rtl819x_ictl_dispatch_bank(ictl, 1,
						      ictl->line_mask[output->line][1]);

	raw = readl(bank_reg(ictl, 0, RTL819X_GISR)) &
	      readl(bank_reg(ictl, 0, RTL819X_GIMR));

	pending = raw & ictl->line_mask[output->line][0];
	if (pending) {
		rtl819x_ictl_dispatch_bank(ictl, 0, pending);
		handled = true;
	}

	if (!handled)
		spurious_interrupt();

	chained_irq_exit(chip, desc);
}

/*
 * Program IRR0-IRR7 with the fixed routing table.  Each IRR covers eight
 * inputs; the nibble for input i sits at bits (4 * i), i.e. the field for
 * the lowest input of a register is LSB-first (bspchip.h IRR_IPtoCPU()
 * places input a0..a3 in bits 3:0).
 */
static void rtl819x_ictl_setup_routing(struct rtl819x_ictl *ictl)
{
	unsigned int bank, reg, i, input;
	const struct rtl819x_ictl_data *data = ictl->data;

	for (bank = 0; bank < data->num_banks; bank++) {
		for (reg = 0; reg < RTL819X_NUM_BANK_INPUTS / RTL819X_INPUTS_PER_IRR; reg++) {
			void __iomem *irr;
			u32 val = 0;

			for (i = 0; i < RTL819X_INPUTS_PER_IRR; i++) {
				input = reg * RTL819X_INPUTS_PER_IRR + i;
				/*
				 * Unrouted bank 1 inputs default to the
				 * generic line, matching the dispatch
				 * side: they stay masked in GIMR2 until a
				 * domain consumer unmasks them.
				 */
				u8 route = rtl819x_routing[bank * RTL819X_NUM_BANK_INPUTS + input];

				if (!route && bank == 1)
					route = RTL819X_CPU_LINE_MIN;
				val |= route << (i % RTL819X_INPUTS_PER_IRR) * 4;
			}

			/*
			 * Unlike the mask/status registers, the bank 1 IRRs
			 * follow the bank 0 IRRs contiguously at their own
			 * absolute offsets (IRR4-7), not at +0x20.
			 */
			irr = ictl->base + (bank ? RTL819X_IRR4 : RTL819X_IRR0) +
			      reg * sizeof(u32);
			writel(val, irr);
		}
	}
}

static void __init rtl819x_ictl_setup_line_masks(struct rtl819x_ictl *ictl)
{
	unsigned int input, line;
	const struct rtl819x_ictl_data *data = ictl->data;

	/*
	 * Bank 1 inputs without an explicit route reach IP2 through the
	 * input 31 aggregator.
	 */
	for (input = 0; input < data->num_inputs; input++) {
		unsigned int bank = input / RTL819X_NUM_BANK_INPUTS;

		if (input == RTL819X_AGGREGATOR_INPUT)
			continue;

		line = rtl819x_routing[input];
		if (!line && bank == 1)
			line = RTL819X_CPU_LINE_MIN;

		ictl->line_mask[line][bank] |=
			BIT(input % RTL819X_NUM_BANK_INPUTS);
	}
}

static int __init rtl819x_ictl_setup_parents(struct device_node *node,
					     struct rtl819x_ictl *ictl)
{
	bool line_claimed[RTL819X_NUM_CPU_LINES] = { };
	struct rtl819x_output *outputs;
	unsigned int p, line, cnt;
	int parent_irq;

	cnt = of_irq_count(node);
	if (!cnt)
		return -EINVAL;

	outputs = kcalloc(cnt, sizeof(*outputs), GFP_KERNEL);
	if (!outputs)
		return -ENOMEM;

	for (p = 0; p < cnt; p++) {
		struct irq_data *parent_data;

		parent_irq = of_irq_get(node, p);
		if (parent_irq <= 0) {
			parent_irq = parent_irq ? parent_irq : -ENODEV;
			goto err_out;
		}

		parent_data = irq_get_irq_data(parent_irq);
		if (!parent_data) {
			parent_irq = -EINVAL;
			goto err_out;
		}

		line = irqd_to_hwirq(parent_data);
		if (line < RTL819X_CPU_LINE_MIN || line > RTL819X_CPU_LINE_MAX) {
			pr_err("%pOF: parent interrupt %d is not a CPU line %u-%u\n",
			       node, parent_irq, RTL819X_CPU_LINE_MIN,
			       RTL819X_CPU_LINE_MAX);
			parent_irq = -EINVAL;
			goto err_out;
		}

		if (line_claimed[line]) {
			pr_err("%pOF: parent interrupt %d claims CPU line %u twice\n",
			       node, parent_irq, line);
			parent_irq = -EINVAL;
			goto err_out;
		}

		if (!ictl->line_mask[line][0] && !ictl->line_mask[line][1])
			pr_warn("%pOF: no inputs routed to parent CPU line %u\n",
				node, line);

		line_claimed[line] = true;
		outputs[p].ictl = ictl;
		outputs[p].parent_irq = parent_irq;
		outputs[p].line = line;
		irq_set_chained_handler_and_data(parent_irq,
						 rtl819x_ictl_dispatch,
						 &outputs[p]);
	}

	/*
	 * Warn about inputs whose fixed routing targets a CPU line missing
	 * from DT so they are not silently lost.
	 */
	for (line = RTL819X_CPU_LINE_MIN; line <= RTL819X_CPU_LINE_MAX; line++) {
		if ((ictl->line_mask[line][0] || ictl->line_mask[line][1]) &&
		    !line_claimed[line])
			pr_warn("%pOF: inputs routed to CPU line %u but no matching parent\n",
				node, line);
	}

	return 0;

err_out:
	while (p--)
		irq_set_chained_handler_and_data(outputs[p].parent_irq,
						 NULL, NULL);
	kfree(outputs);
	return parent_irq;
}

static int __init rtl819x_ictl_of_init(struct device_node *node,
				       struct device_node *parent)
{
	struct rtl819x_ictl *ictl;
	const struct rtl819x_ictl_data *data;
	const struct of_device_id *match;
	unsigned int bank;
	int ret;

	match = of_match_node(rtl819x_ictl_of_match, node);
	if (!match || !match->data)
		return -EINVAL;
	data = match->data;

	ictl = kzalloc_obj(*ictl, GFP_KERNEL);
	if (!ictl)
		return -ENOMEM;

	ictl->data = data;
	raw_spin_lock_init(&ictl->lock);

	ictl->base = of_iomap(node, 0);
	if (!ictl->base) {
		pr_err("%pOF: unable to map registers\n", node);
		ret = -ENXIO;
		goto err_free;
	}

	/* Mask everything before touching the routing */
	for (bank = 0; bank < data->num_banks; bank++)
		writel(0, bank_reg(ictl, bank,
				   bank ? RTL819X_GIMR2 : RTL819X_GIMR));

	rtl819x_ictl_setup_routing(ictl);
	rtl819x_ictl_setup_line_masks(ictl);

	/* Keep the bank 1 aggregator enabled permanently */
	if (data->num_banks > 1)
		writel(BIT(RTL819X_AGGREGATOR_INPUT),
		       bank_reg(ictl, 0, RTL819X_GIMR));

	ictl->domain = irq_domain_create_linear(of_fwnode_handle(node),
						data->num_inputs,
						&rtl819x_ictl_domain_ops, ictl);
	if (!ictl->domain) {
		pr_err("%pOF: unable to add IRQ domain\n", node);
		ret = -ENOMEM;
		goto err_unmap;
	}

	ret = rtl819x_ictl_setup_parents(node, ictl);
	if (ret)
		goto err_domain;

	return 0;

err_domain:
	irq_domain_remove(ictl->domain);
err_unmap:
	iounmap(ictl->base);
err_free:
	kfree(ictl);
	return ret;
}
/*
 * Reserved for future members of the RTL819x family (e.g. the single-bank,
 * 32-input RTL8196E, RTL8197D and RTL8881A); their compatibles can be added
 * to rtl819x_ictl_of_match with per-SoC geometry once their routing tables
 * and input maps are handled.
 */
IRQCHIP_DECLARE(realtek_rtl819x_intc, "realtek,rtl8197f-intc",
		rtl819x_ictl_of_init);
