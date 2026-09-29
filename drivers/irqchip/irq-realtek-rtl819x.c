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
 * The interrupt specifiers are two cells, <input output>: the first cell is
 * the input number, the second one the index of the parent interrupt (in
 * the controller node's "interrupts" list) whose CPU line the input is
 * wired to.  As on the Otto controller, the routing nibble is written when
 * the input is mapped, and inputs never referenced from DT stay
 * disconnected (their IRR field is cleared at probe time).
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
#include <linux/slab.h>
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

struct rtl819x_output {
	struct fwnode_handle		*fwnode;
	struct irq_domain		*domain;
	struct rtl819x_ictl		*ictl;
	unsigned int			parent_irq;
	unsigned int			line;
	unsigned int			index;
	/* inputs routed to this output: bank 0 in the low half */
	u64				mask;
};

/**
 * struct rtl819x_ictl - driver private data
 * @data: per-compatible controller geometry
 * @base: remapped register block
 * @lock: protects read-modify-write cycles on the mask and routing
 *	registers
 * @num_outputs: number of entries in @outputs
 * @outputs: the controller's parent outputs; the chained handlers
 *	installed for them reference this array, so it lives and dies
 *	with the controller
 */
struct rtl819x_ictl {
	const struct rtl819x_ictl_data	*data;
	void __iomem			*base;
	raw_spinlock_t			lock;
	unsigned int			num_outputs;
	/* parent outputs, referenced by the chained handlers for as long
	 * as the controller lives; must stay the last member
	 */
	struct rtl819x_output		outputs[] __counted_by(num_outputs);
};

/* Write the routing nibble of one input; caller holds ictl->lock. */
static void rtl819x_write_irr(struct rtl819x_ictl *ictl, unsigned int hwirq,
			      unsigned int line)
{
	unsigned int bank = hwirq / RTL819X_NUM_BANK_INPUTS;
	void __iomem *irr;
	u32 val;

	irr = bank_reg(ictl, bank,
		       bank ? RTL819X_IRR4 : RTL819X_IRR0) +
	      (hwirq % RTL819X_NUM_BANK_INPUTS) / 8 * sizeof(u32);

	val = readl(irr);
	val &= ~(0xfu << (hwirq % 8) * 4);
	val |= line << (hwirq % 8) * 4;
	writel(val, irr);
}

static void rtl819x_ictl_mask_input(struct rtl819x_ictl *ictl,
				    unsigned int hwirq, bool mask)
{
	unsigned int bank = hwirq / RTL819X_NUM_BANK_INPUTS;
	void __iomem *reg;
	u32 val;

	reg = bank_reg(ictl, bank, bank ? RTL819X_GIMR2 : RTL819X_GIMR);

	guard(raw_spinlock_irqsave)(&ictl->lock);
	val = readl(reg);
	if (mask)
		val &= ~BIT(hwirq % RTL819X_NUM_BANK_INPUTS);
	else
		val |= BIT(hwirq % RTL819X_NUM_BANK_INPUTS);
	writel(val, reg);
}

static void rtl819x_ictl_mask(struct irq_data *d)
{
	rtl819x_ictl_mask_input(irq_data_get_irq_chip_data(d), d->hwirq, true);
}

static void rtl819x_ictl_unmask(struct irq_data *d)
{
	rtl819x_ictl_mask_input(irq_data_get_irq_chip_data(d), d->hwirq, false);
}

static struct irq_chip rtl819x_ictl_chip = {
	.name			= "realtek-rtl819x-intc",
	.irq_mask		= rtl819x_ictl_mask,
	.irq_unmask		= rtl819x_ictl_unmask,
};

static int rtl819x_ictl_map(struct irq_domain *d, unsigned int virq,
			    irq_hw_number_t hwirq)
{
	struct rtl819x_output *output = d->host_data;
	struct rtl819x_ictl *ictl = output->ictl;

	if (hwirq >= ictl->data->num_inputs ||
	    (ictl->data->num_banks > 1 && hwirq == RTL819X_AGGREGATOR_INPUT))
		return -EINVAL;

	irq_set_chip_data(virq, ictl);
	irq_set_chip_and_handler(virq, &rtl819x_ictl_chip, handle_level_irq);

	guard(raw_spinlock_irqsave)(&ictl->lock);
	output->mask |= BIT_ULL(hwirq);
	rtl819x_write_irr(ictl, hwirq, output->line);

	return 0;
}

static int rtl819x_ictl_select(struct irq_domain *d, struct irq_fwspec *fwspec,
			       enum irq_domain_bus_token bus_token)
{
	struct rtl819x_output *output = d->host_data;

	if (fwspec->fwnode != output->fwnode || fwspec->param_count != 2)
		return false;

	return fwspec->param[1] == output->index;
}

static const struct irq_domain_ops rtl819x_ictl_domain_ops = {
	.map	= rtl819x_ictl_map,
	.select	= rtl819x_ictl_select,
	.xlate	= irq_domain_xlate_onecell,
};

static bool rtl819x_ictl_dispatch_bank(struct rtl819x_output *output,
				       unsigned int bank, u32 mask)
{
	struct rtl819x_ictl *ictl = output->ictl;
	unsigned long pending;
	unsigned int input;

	pending = readl(bank_reg(ictl, bank,
				 bank ? RTL819X_GISR2 : RTL819X_GISR)) &
		  readl(bank_reg(ictl, bank,
				 bank ? RTL819X_GIMR2 : RTL819X_GIMR)) & mask;

	for_each_set_bit(input, &pending, RTL819X_NUM_BANK_INPUTS)
		generic_handle_domain_irq(output->domain,
					  bank * RTL819X_NUM_BANK_INPUTS +
					  input);

	return pending;
}

static void rtl819x_ictl_dispatch(struct irq_desc *desc)
{
	struct rtl819x_output *output = irq_desc_get_handler_data(desc);
	struct rtl819x_ictl *ictl = output->ictl;
	struct irq_chip *chip = irq_desc_get_chip(desc);
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
		handled |= rtl819x_ictl_dispatch_bank(output, 1,
						      output->mask >> 32);

	if (output->mask & GENMASK_ULL(31, 0))
		handled |= rtl819x_ictl_dispatch_bank(output, 0, output->mask);

	if (!handled)
		spurious_interrupt();

	chained_irq_exit(chip, desc);
}

static void rtl819x_ictl_clear_routing(struct rtl819x_ictl *ictl)
{
	unsigned int bank, reg;

	for (bank = 0; bank < ictl->data->num_banks; bank++)
		for (reg = 0; reg < RTL819X_NUM_BANK_INPUTS / 8; reg++)
			writel(0, bank_reg(ictl, bank,
					   bank ? RTL819X_IRR4 : RTL819X_IRR0) +
				   reg * sizeof(u32));
}

static int rtl819x_ictl_setup_parents(struct device_node *node,
				      struct rtl819x_ictl *ictl)
{
	bool line_claimed[RTL819X_NUM_CPU_LINES] = { };
	unsigned int p, cnt, line;
	int parent_irq, ret;

	cnt = of_irq_count(node);
	if (!cnt)
		return -EINVAL;
	ictl->num_outputs = cnt;

	for (p = 0; p < cnt; p++) {
		struct irq_data *parent_data;
		struct irq_domain *domain;

		parent_irq = of_irq_get(node, p);
		if (parent_irq <= 0) {
			ret = parent_irq ?: -ENODEV;
			goto err_out;
		}

		parent_data = irq_get_irq_data(parent_irq);
		if (!parent_data) {
			ret = -EINVAL;
			goto err_out;
		}

		line = irqd_to_hwirq(parent_data);
		if (line < RTL819X_CPU_LINE_MIN || line > RTL819X_CPU_LINE_MAX) {
			pr_err("%pOF: parent interrupt %d is not a CPU line %u-%u\n",
			       node, parent_irq, RTL819X_CPU_LINE_MIN,
			       RTL819X_CPU_LINE_MAX);
			ret = -EINVAL;
			goto err_out;
		}

		if (line_claimed[line]) {
			pr_err("%pOF: parent interrupt %d claims CPU line %u twice\n",
			       node, parent_irq, line);
			ret = -EINVAL;
			goto err_out;
		}
		line_claimed[line] = true;

		domain = irq_domain_create_linear(of_fwnode_handle(node),
						  ictl->data->num_inputs,
						  &rtl819x_ictl_domain_ops,
						  &ictl->outputs[p]);
		if (!domain) {
			ret = -ENOMEM;
			goto err_out;
		}

		ictl->outputs[p].domain = domain;
		ictl->outputs[p].fwnode = of_fwnode_handle(node);
		ictl->outputs[p].ictl = ictl;
		ictl->outputs[p].index = p;
		ictl->outputs[p].parent_irq = parent_irq;
		ictl->outputs[p].line = line;
		irq_set_chained_handler_and_data(parent_irq,
						 rtl819x_ictl_dispatch,
						 &ictl->outputs[p]);
	}

	return 0;

err_out:
	while (p--) {
		irq_set_chained_handler_and_data(ictl->outputs[p].parent_irq,
						 NULL, NULL);
		irq_domain_remove(ictl->outputs[p].domain);
	}

	return ret;
}

static int __init rtl819x_ictl_of_init(struct device_node *node,
				       struct device_node *parent)
{
	const struct rtl819x_ictl_data *data;
	const struct of_device_id *match;
	struct rtl819x_ictl *ictl;
	unsigned int bank;
	int ret;

	match = of_match_node(rtl819x_ictl_of_match, node);
	if (!match || !match->data)
		return -EINVAL;
	data = match->data;

	ictl = kzalloc(struct_size(ictl, outputs, of_irq_count(node)),
		       GFP_KERNEL);
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

	/* Mask everything and disconnect every input before serving DT */
	for (bank = 0; bank < data->num_banks; bank++)
		writel(0, bank_reg(ictl, bank,
				   bank ? RTL819X_GIMR2 : RTL819X_GIMR));
	rtl819x_ictl_clear_routing(ictl);

	/* Keep the bank 1 aggregator enabled permanently */
	if (data->num_banks > 1)
		writel(BIT(RTL819X_AGGREGATOR_INPUT),
		       bank_reg(ictl, 0, RTL819X_GIMR));

	ret = rtl819x_ictl_setup_parents(node, ictl);
	if (ret)
		goto err_unmap;

	return 0;

err_unmap:
	iounmap(ictl->base);
err_free:
	kfree(ictl);
	return ret;
}
/*
 * Reserved for future members of the RTL819x family (e.g. the single-bank,
 * 32-input RTL8196E, RTL8197D and RTL8881A); their compatibles can be added
 * to rtl819x_ictl_of_match with per-SoC geometry once their input maps are
 * handled.
 */
IRQCHIP_DECLARE(realtek_rtl819x_intc, "realtek,rtl8197f-intc",
		rtl819x_ictl_of_init);
