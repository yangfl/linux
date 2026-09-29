// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCI host controller driver for Realtek RTL819x router SoCs
 *
 * The RTL8197F and its siblings expose a single PCIe 1.1 root port used
 * for an on-board or mini-PCIe wireless card.  Root port configuration
 * space is memory-mapped directly, while configuration accesses to
 * downstream devices are routed through an indexed window selected by
 * the IPCFG register.  The PHY must be programmed over a private MDIO
 * bus before the link comes up; the reset bits live in the shared
 * system controller.
 *
 * Root port and endpoint interrupts are described by the interrupt-map
 * of the host bridge device node; the driver itself creates no
 * interrupt domain.
 *
 * Copyright (C) 2017 Weijie Gao <hackpascal@gmail.com>
 * Copyright (C) 2019 Gaspare Bruno <gaspare@anlix.io>
 * Copyright (C) 2026 David Yang <mmyangfl@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_pci.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/regmap.h>
#include <linux/resource_ext.h>

#include "../pci.h"

/* Root port configuration space (bus 0, device 0) */
#define RTL819X_PCIE_RC_CFG_SIZE		0x1000
#define RTL819X_PCIE_CFG_DEVCTL			0x78
#define RTL819X_PCIE_CFG_LINKDBG		0x728
#define RTL819X_PCIE_LINKDBG_LTSSM		GENMASK(4, 0)
#define RTL819X_PCIE_LTSSM_L0			0x11

/* Downstream device configuration window, indexed by IPCFG */
#define RTL819X_PCIE_EP_CFG_SIZE		0x1000

/* Root complex extended registers */
#define RTL819X_PCIE_RC_EXT_SIZE		0x100
#define RTL819X_PCIE_MDIO			0x00
#define RTL819X_PCIE_MDIO_WRITE			BIT(0)
#define RTL819X_PCIE_MDIO_REG			GENMASK(12, 8)
#define RTL819X_PCIE_MDIO_DATA			GENMASK(31, 16)
#define RTL819X_PCIE_PWRCR			0x08
#define RTL819X_PCIE_PWRCR_LTSSM_EN		BIT(0)
#define RTL819X_PCIE_PWRCR_PHY_RST		BIT(7)
#define RTL819X_PCIE_IPCFG			0x0c
#define RTL819X_PCIE_IPCFG_BUS			GENMASK(15, 8)
#define RTL819X_PCIE_IPCFG_DEVFN		GENMASK(7, 0)

/* System controller registers */
#define RTL819X_SYSC_HW_STRAP			0x08
#define RTL819X_SYSC_HW_STRAP_PCIE_CLK_40M	BIT(24)
#define RTL819X_SYSC_CLK_MANAGE			0x10
#define RTL819X_SYSC_CLK_MANAGE_PCIE_EN		(BIT(12) | BIT(13) | BIT(18))
#define RTL819X_SYSC_CLK_MANAGE_PCIE_ACTIVE	BIT(14)
#define RTL819X_SYSC_ENABLE_IP			0x50
#define RTL819X_SYSC_ENABLE_IP_PCIE_PERST	BIT(1)
#define RTL819X_SYSC_PCIE_PHY			0x100
#define RTL819X_SYSC_PCIE_PHY_MDIO_RST		BIT(3)
#define RTL819X_SYSC_PCIE_PHY_LOAD_DONE		BIT(1)
#define RTL819X_SYSC_PCIE_PHY_MDIO_EN		BIT(0)

#define RTL819X_PCIE_PERST_DELAY_MS		300
#define RTL819X_PCIE_LINK_TIMEOUT_MS		100
#define RTL819X_PCIE_LINK_RETRIES		10

/*
 * PERST# is not wired to the same register across the family: the RTL8197F
 * drives it from bit 1 of the ENABLE_IP register (offset 0x50), while the
 * RTL8196E / RTL8197D instead use bit 26 of the CLK_MANAGE register (offset
 * 0x10).  Keep the register offset and bit per compatible.
 */
struct rtl819x_pcie_data {
	u16 perst_reg;
	u8 perst_bit;
};

static const struct rtl819x_pcie_data rtl8197f_pcie_data = {
	.perst_reg	= RTL819X_SYSC_ENABLE_IP,
	.perst_bit	= RTL819X_SYSC_ENABLE_IP_PCIE_PERST,
	/*
	 * RTL8196E / RTL8197D, for future use:
	 *	.perst_reg	= RTL819X_SYSC_CLK_MANAGE,	0x10
	 *	.perst_bit	= BIT(26),
	 */
};

/**
 * struct rtl819x_pcie - RTL819x PCIe host controller
 * @dev: platform device
 * @data: per-SoC register offsets / bits (per-compatible data)
 * @rc_cfg: root port configuration space
 * @ep_cfg: downstream device configuration window
 * @rc_ext: root complex extended registers (MDIO, PWRCR, IPCFG)
 * @sysc: system controller with clock / reset / PERST bits
 */
struct rtl819x_pcie {
	struct device *dev;
	const struct rtl819x_pcie_data *data;
	void __iomem *rc_cfg;
	void __iomem *ep_cfg;
	void __iomem *rc_ext;
	struct regmap *sysc;
};

static bool rtl819x_pcie_link_ok(struct rtl819x_pcie *pcie);

static void __iomem *rtl819x_pcie_map_bus(struct pci_bus *bus,
					  unsigned int devfn, int where)
{
	struct rtl819x_pcie *pcie = bus->sysdata;
	u32 val;

	if (!rtl819x_pcie_link_ok(pcie))
		return NULL;

	if (bus->number == 0) {
		/* Root port configuration space is memory mapped directly */
		if (PCI_SLOT(devfn) != 0)
			return NULL;

		return pcie->rc_cfg + where;
	}

	/*
	 * Downstream configuration accesses go through an indexed window.
	 * Only the four devices behind an optional bridge are reachable.
	 */
	if (PCI_SLOT(devfn) > 3)
		return NULL;

	val = FIELD_PREP(RTL819X_PCIE_IPCFG_BUS, bus->number) |
	      FIELD_PREP(RTL819X_PCIE_IPCFG_DEVFN, devfn);
	writel_relaxed(val, pcie->rc_ext + RTL819X_PCIE_IPCFG);

	return pcie->ep_cfg + where;
}

static struct pci_ops rtl819x_pcie_ops = {
	.map_bus	= rtl819x_pcie_map_bus,
	.read		= pci_generic_config_read,
	.write		= pci_generic_config_write,
};

static void rtl819x_pcie_mdio_write(struct rtl819x_pcie *pcie, u32 reg, u32 data)
{
	u32 val;

	val = u32_encode_bits(data, RTL819X_PCIE_MDIO_DATA) |
	      u32_encode_bits(reg, RTL819X_PCIE_MDIO_REG) |
	      RTL819X_PCIE_MDIO_WRITE;
	writel_relaxed(val, pcie->rc_ext + RTL819X_PCIE_MDIO);
	usleep_range(2000, 3000);
}

static void rtl819x_pcie_mdio_reset(struct rtl819x_pcie *pcie)
{
	/* Release the private MDIO controller and load the PHY parameters */
	regmap_write(pcie->sysc, RTL819X_SYSC_PCIE_PHY,
		     RTL819X_SYSC_PCIE_PHY_MDIO_RST);
	regmap_write(pcie->sysc, RTL819X_SYSC_PCIE_PHY,
		     RTL819X_SYSC_PCIE_PHY_MDIO_RST |
		     RTL819X_SYSC_PCIE_PHY_MDIO_EN);
	regmap_write(pcie->sysc, RTL819X_SYSC_PCIE_PHY,
		     RTL819X_SYSC_PCIE_PHY_MDIO_RST |
		     RTL819X_SYSC_PCIE_PHY_LOAD_DONE |
		     RTL819X_SYSC_PCIE_PHY_MDIO_EN);
}

static void rtl819x_pcie_phy_reset(struct rtl819x_pcie *pcie)
{
	void __iomem *pwrcr = pcie->rc_ext + RTL819X_PCIE_PWRCR;

	writel_relaxed(RTL819X_PCIE_PWRCR_LTSSM_EN, pwrcr);
	writel_relaxed(RTL819X_PCIE_PWRCR_PHY_RST | RTL819X_PCIE_PWRCR_LTSSM_EN,
		       pwrcr);
}

/*
 * Program the PCIe PHY over the private MDIO bus.  The magic values come
 * from the vendor SDK (PCIeMDIOPHYParameterSetRLE0600); they depend on
 * the 25 MHz / 40 MHz reference clock selected by a hardware strap.
 */
static void rtl819x_pcie_mdio_phy_init(struct rtl819x_pcie *pcie)
{
	u32 val;
	int ret;

	ret = regmap_read(pcie->sysc, RTL819X_SYSC_HW_STRAP, &val);
	if (ret)
		dev_warn(pcie->dev, "failed to read hardware strap: %d\n", ret);

	if (!ret && (val & RTL819X_SYSC_HW_STRAP_PCIE_CLK_40M)) {
		rtl819x_pcie_mdio_write(pcie, 0x0f, 0x12f6);
		rtl819x_pcie_mdio_write(pcie, 0x00, 0x0071);
		rtl819x_pcie_mdio_write(pcie, 0x06, 0x1ac1);
	} else {
		rtl819x_pcie_mdio_write(pcie, 0x00, 0x0071);
		rtl819x_pcie_mdio_write(pcie, 0x06, 0x18c1);
	}
}

static void rtl819x_pcie_device_reset(struct rtl819x_pcie *pcie)
{
	/* PERST# is driven from the system controller */
	regmap_update_bits(pcie->sysc, pcie->data->perst_reg,
			   pcie->data->perst_bit, 0);
	msleep(RTL819X_PCIE_PERST_DELAY_MS);
	regmap_update_bits(pcie->sysc, pcie->data->perst_reg,
			   pcie->data->perst_bit,
			   pcie->data->perst_bit);
}

/* Single-shot check: LTSSM must have reached the L0 state */
static bool rtl819x_pcie_link_ok(struct rtl819x_pcie *pcie)
{
	u32 val = readl(pcie->rc_cfg + RTL819X_PCIE_CFG_LINKDBG);

	return FIELD_GET(RTL819X_PCIE_LINKDBG_LTSSM, val) ==
	       RTL819X_PCIE_LTSSM_L0;
}

static bool rtl819x_pcie_link_up(struct rtl819x_pcie *pcie)
{
	int i;

	for (i = 0; i < RTL819X_PCIE_LINK_RETRIES; i++) {
		if (rtl819x_pcie_link_ok(pcie))
			return true;

		msleep(RTL819X_PCIE_LINK_TIMEOUT_MS);
	}

	return false;
}

static void rtl819x_pcie_power_on(struct rtl819x_pcie *pcie)
{
	/* Turn on the PCIe IP clock and deassert its resets */
	regmap_update_bits(pcie->sysc, RTL819X_SYSC_CLK_MANAGE,
			   RTL819X_SYSC_CLK_MANAGE_PCIE_EN,
			   RTL819X_SYSC_CLK_MANAGE_PCIE_EN);
	regmap_update_bits(pcie->sysc, RTL819X_SYSC_CLK_MANAGE,
			   RTL819X_SYSC_CLK_MANAGE_PCIE_ACTIVE,
			   RTL819X_SYSC_CLK_MANAGE_PCIE_ACTIVE);
	usleep_range(10000, 20000);

	rtl819x_pcie_mdio_reset(pcie);
	rtl819x_pcie_phy_reset(pcie);
	rtl819x_pcie_mdio_phy_init(pcie);

	/* Reset the PHY again so the new parameters take effect */
	rtl819x_pcie_phy_reset(pcie);

	rtl819x_pcie_device_reset(pcie);
}

static void rtl819x_pcie_hw_init(struct rtl819x_pcie *pcie)
{
	void __iomem *addr;

	rtl819x_pcie_power_on(pcie);

	if (!rtl819x_pcie_link_up(pcie)) {
		dev_warn(pcie->dev, "PCIe link is down\n");
		/*
		 * Vendor fallback: gate the IP again so a half-trained
		 * link does not linger with its clock active.
		 */
		regmap_update_bits(pcie->sysc, RTL819X_SYSC_CLK_MANAGE,
				   RTL819X_SYSC_CLK_MANAGE_PCIE_ACTIVE, 0);
		return;
	}

	/* Enable bus mastering and the I/O / memory decoders on both ends */
	addr = pcie->rc_cfg + PCI_COMMAND;
	writel(PCI_COMMAND_MASTER | PCI_COMMAND_IO | PCI_COMMAND_MEMORY,
	       addr);

	addr = pcie->ep_cfg + PCI_COMMAND;
	writel(PCI_COMMAND_MASTER | PCI_COMMAND_IO | PCI_COMMAND_MEMORY,
	       addr);

	/* Cap the maximum payload size at 128 bytes */
	addr = pcie->rc_cfg + RTL819X_PCIE_CFG_DEVCTL;
	writel(readl(addr) & ~PCI_EXP_DEVCTL_PAYLOAD, addr);

	msleep(RTL819X_PCIE_LINK_TIMEOUT_MS);
}

static int rtl819x_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtl819x_pcie *pcie;
	struct pci_host_bridge *bridge;
	struct resource_entry *entry;
	struct resource *res;

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*pcie));
	if (!bridge)
		return -ENOMEM;

	pcie = pci_host_bridge_priv(bridge);
	pcie->dev = dev;

	pcie->data = of_device_get_match_data(dev);
	if (!pcie->data)
		return dev_err_probe(dev, -EINVAL,
				     "missing compatible match data\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return dev_err_probe(dev, -EINVAL,
				     "missing register resource 0\n");
	if (resource_size(res) != RTL819X_PCIE_RC_CFG_SIZE)
		return dev_err_probe(dev, -EINVAL,
				     "invalid root port config space size\n");
	pcie->rc_cfg = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->rc_cfg))
		return PTR_ERR(pcie->rc_cfg);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res)
		return dev_err_probe(dev, -EINVAL,
				     "missing register resource 1\n");
	if (resource_size(res) != RTL819X_PCIE_EP_CFG_SIZE)
		return dev_err_probe(dev, -EINVAL,
				     "invalid endpoint config space size\n");
	pcie->ep_cfg = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->ep_cfg))
		return PTR_ERR(pcie->ep_cfg);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res)
		return dev_err_probe(dev, -EINVAL,
				     "missing register resource 2\n");
	if (resource_size(res) != RTL819X_PCIE_RC_EXT_SIZE)
		return dev_err_probe(dev, -EINVAL,
				     "invalid extended register size\n");
	pcie->rc_ext = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->rc_ext))
		return PTR_ERR(pcie->rc_ext);

	pcie->sysc = syscon_regmap_lookup_by_phandle(dev->of_node, "syscon");
	if (IS_ERR(pcie->sysc))
		return dev_err_probe(dev, PTR_ERR(pcie->sysc),
				     "failed to get system controller\n");

	rtl819x_pcie_hw_init(pcie);

	/* The ranges property of the node must provide a memory window */
	entry = resource_list_first_type(&bridge->windows, IORESOURCE_MEM);
	if (!entry)
		return dev_err_probe(dev, -EINVAL,
				     "invalid or missing PCI ranges\n");

	bridge->sysdata = pcie;
	bridge->ops = &rtl819x_pcie_ops;

	return pci_host_probe(bridge);
}

static const struct of_device_id rtl819x_pcie_ids[] = {
	{ .compatible = "realtek,rtl8197f-pcie", .data = &rtl8197f_pcie_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rtl819x_pcie_ids);

static struct platform_driver rtl819x_pcie_driver = {
	.probe = rtl819x_pcie_probe,
	.driver = {
		.name = "realtek-rtl819x-pci",
		.of_match_table = rtl819x_pcie_ids,
	},
};
module_platform_driver(rtl819x_pcie_driver);

MODULE_DESCRIPTION("Realtek RTL819x PCIe host controller driver");
MODULE_LICENSE("GPL");
