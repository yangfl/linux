// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTL819x USB 2.0 PHY driver
 *
 * Copyright (C) 2026 David Yang <mmyangfl@gmail.com>
 *
 * The RTL8197F integrates two UTMI USB 2.0 PHYs.  Both PHYs are shared by
 * the on-chip EHCI and OHCI host controllers, and PHY 1 can alternatively
 * be routed to the DWC OTG device controller through the SIE control
 * register.  The PHYs themselves are powered and released from reset by
 * bits in the system controller, while their electrical parameters are
 * accessed indirectly through a data register in the USB analog block and
 * an address strobe register in the EHCI MMIO space.
 *
 * The register sequences below follow the vendor boot ROM / BSP code
 * (rtl819x boards/rtl8197F/bsp/dev_usb.c) and the Synopsys DWC OTG glue
 * in the vendor 3.10 tree.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/cleanup.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/*
 * System controller (realtek,rtl8197f-syscon, base 0x18000000).
 * Offsets and bit assignments per the vendor BSP (dev_usb.c, bspchip.h)
 * and the DWC OTG driver in the vendor kernel.
 */
#define RTL819X_SYSCON_STRAP			0x08
#define RTL819X_SYSCON_STRAP_XTAL_40M		BIT(24)	/* 1: 40 MHz xtal */
#define RTL819X_SYSCON_USB_CLKCR		0x14
#define RTL819X_SYSCON_USB_CLK_EN		BIT(3)	/* USB block clock */
#define RTL819X_SYSCON_USB_HOSTCR		0x160
#define RTL819X_SYSCON_USB_HOST_EN		BIT(0)	/* USB host enable */
#define RTL819X_SYSCON_USB_PHYCR		0x16c
#define RTL819X_SYSCON_USB_SIECR		0x180

/* PHYCR (0x16c) bits, per port: enable, reset (active high), activate */
#define RTL819X_USB_PHY_EN(port)		BIT(8 + 11 * (port))
#define RTL819X_USB_PHY_RESET(port)		BIT(9 + 11 * (port))
#define RTL819X_USB_PHY_ACTIVE(port)		BIT(10 + 11 * (port))

/* SIECR (0x180) bits */
#define RTL819X_USB_SIE_OTG_EN			BIT(0)	/* connect OTG MAC to PHY 1 */
#define RTL819X_USB_SIE_TWO_PORT		BIT(17)	/* 1: two-port mode */
#define RTL819X_USB_SIE_PORT1_HOST		BIT(18)	/* 0: OTG, 1: EHCI host */

/*
 * USB analog block (base 0x18140200).  Only the registers touched by the
 * vendor USB bring-up code are modelled.
 */
#define RTL819X_USB_ANA_UTMI_DATA		0x0c	/* indirect data */
#define RTL819X_USB_ANA_UTMI_CTRL		0x10	/* port force mode */

/*
 * UTMI_CTRL (0x10): force-mode bits per port (set the first, clear the
 * second) plus global suspend-drop workarounds from the vendor BSP
 * (dev_usb.c clears bits 9 and 25 of the register, one per port).
 */
#define RTL819X_USB_UTMI_FORCE(port)		BIT(7 + 16 * (port))
#define RTL819X_USB_UTMI_NOFORCE(port)		BIT(5 + 16 * (port))
#define RTL819X_USB_UTMI_SUSPEND_FIX		(BIT(9) | BIT(25))

/*
 * Indirect UTMI register access.  The data register lives in the analog
 * block, the address strobe register is a Synopsys vendor register at
 * EHCI MMIO offset 0xa4.  Strobes A/B select the address nibble halves;
 * bit 13 selects the port.
 */
#define RTL819X_USB_ACC_STROBE_A		0x2000
#define RTL819X_USB_ACC_STROBE_B		0x3000
#define RTL819X_USB_ACC_PORT_SHIFT		13
#define RTL819X_USB_ACC_ADDR_SHIFT		8
#define RTL819X_USB_ACC_DATA_MAGIC		0x00340000	/* vendor marker */

/* Indirect UTMI register addresses */
#define RTL819X_UTMI_REG_PAGE			0xf4
#define RTL819X_UTMI_PAGE0			0x9b
#define RTL819X_UTMI_PAGE1			0xbb

#define RTL819X_USB_PHY_NPORTS			2
#define RTL819X_USB_PHY_DELAY_US		10000

struct rtl819x_usb_phy {
	struct regmap		*syscon;
	void __iomem		*analog;
	void __iomem		*acc;	/* EHCI+0xa4 strobe register */
	bool			xtal_40m;
	/* serializes RMW on the two-port shared UTMI_CTRL register */
	struct mutex		utmi_lock;
};

struct rtl819x_usb_phy_port {
	struct rtl819x_usb_phy	*usbphy;
	unsigned int		id;
	struct phy		*phy;
};

static void rtl819x_usb_utmi_write(struct rtl819x_usb_phy *usbphy,
				   unsigned int port, u8 reg, u8 val)
{
	u8 lo = reg & 0x0f;
	u8 hi = (reg & 0xf0) >> 4;
	u32 port_bit = port << RTL819X_USB_ACC_PORT_SHIFT;
	int i;

	/*
	 * Sequence per the vendor SetUSBPhy(): put the value into the
	 * analog data register, then clock out the register address as two
	 * 4-bit nibbles, each framed by strobe B - A - B.
	 */
	writel(RTL819X_USB_ACC_DATA_MAGIC | (val << (8 * port)),
	       usbphy->analog + RTL819X_USB_ANA_UTMI_DATA);

	/*
	 * The vendor settles every strobe write for 10 ms (USB2_PHY_DELAY
	 * after each single write), not once per nibble.
	 */
	for (i = 0; i < 3; i++) {
		writel((i == 1 ? RTL819X_USB_ACC_STROBE_A :
				 RTL819X_USB_ACC_STROBE_B) |
		       port_bit | (lo << RTL819X_USB_ACC_ADDR_SHIFT),
		       usbphy->acc);
		fsleep(RTL819X_USB_PHY_DELAY_US);
	}

	for (i = 0; i < 3; i++) {
		writel((i == 1 ? RTL819X_USB_ACC_STROBE_A :
				 RTL819X_USB_ACC_STROBE_B) |
		       port_bit | (hi << RTL819X_USB_ACC_ADDR_SHIFT),
		       usbphy->acc);
		fsleep(RTL819X_USB_PHY_DELAY_US);
	}
}

/*
 * Electrical tuning values from the vendor BSP.  Separate tables exist for
 * the 25 MHz and 40 MHz crystal variants, selected at runtime by a boot
 * strap bit exactly as the vendor code does.  Values are written to the
 * page 0 / page 1 UTMI registers of both ports.
 */
static const u8 rtl819x_usb_tune_page0[][2] = {
	{ 0xe0, 0xe3 }, { 0xe1, 0x30 }, { 0xe2, 0xd5 },
	{ 0xe4, 0xc9 },	/* disconnect threshold */
	{ 0xe6, 0xc1 },
};

static const u8 rtl819x_usb_tune_page0_40m[][2] = {
	{ 0xe2, 0x33 },
	{ 0xe4, 0xc9 },
	{ 0xe6, 0xc1 },
};

static const u8 rtl819x_usb_tune_page1[][2] = {
	{ 0xe5, 0x11 }, { 0xe6, 0x06 }, { 0xe7, 0x66 },
};

static const u8 rtl819x_usb_tune_page1_40m[][2] = {
	{ 0xe6, 0x00 }, { 0xe7, 0x00 },
};

static void rtl819x_usb_phy_tune(struct rtl819x_usb_phy *usbphy,
				 unsigned int port)
{
	const u8 (*page0)[2];
	const u8 (*page1)[2];
	size_t n0, n1, i;

	if (usbphy->xtal_40m) {
		page0 = rtl819x_usb_tune_page0_40m;
		n0 = ARRAY_SIZE(rtl819x_usb_tune_page0_40m);
		page1 = rtl819x_usb_tune_page1_40m;
		n1 = ARRAY_SIZE(rtl819x_usb_tune_page1_40m);
	} else {
		page0 = rtl819x_usb_tune_page0;
		n0 = ARRAY_SIZE(rtl819x_usb_tune_page0);
		page1 = rtl819x_usb_tune_page1;
		n1 = ARRAY_SIZE(rtl819x_usb_tune_page1);
	}

	/* make sure we are on page 0 */
	rtl819x_usb_utmi_write(usbphy, port, RTL819X_UTMI_REG_PAGE,
			       RTL819X_UTMI_PAGE0);
	for (i = 0; i < n0; i++)
		rtl819x_usb_utmi_write(usbphy, port, page0[i][0], page0[i][1]);

	rtl819x_usb_utmi_write(usbphy, port, RTL819X_UTMI_REG_PAGE,
			       RTL819X_UTMI_PAGE1);
	for (i = 0; i < n1; i++)
		rtl819x_usb_utmi_write(usbphy, port, page1[i][0], page1[i][1]);

	rtl819x_usb_utmi_write(usbphy, port, RTL819X_UTMI_REG_PAGE,
			       RTL819X_UTMI_PAGE0);
}

static void rtl819x_usb_phy_port_init(struct rtl819x_usb_phy *usbphy,
				      unsigned int port)
{
	u32 val;

	/*
	 * UTMI_CTRL covers both ports; guard the read-modify-write so
	 * concurrent phy_init()s cannot clobber each other's field.
	 */
	guard(mutex)(&usbphy->utmi_lock);

	/*
	 * Reset this PHY before configuring it: enable it, hold reset
	 * (PHYCR, active high), release.  The same line is pulsed again after
	 * the parameters are written, which is what latches them, so every
	 * phy_init() - the configuration entry point - starts from a reset PHY
	 * rather than from whatever the boot code or a previous host left.
	 */
	regmap_update_bits(usbphy->syscon, RTL819X_SYSCON_USB_PHYCR,
			   RTL819X_USB_PHY_EN(port), RTL819X_USB_PHY_EN(port));
	regmap_update_bits(usbphy->syscon, RTL819X_SYSCON_USB_PHYCR,
			   RTL819X_USB_PHY_RESET(port),
			   RTL819X_USB_PHY_RESET(port));
	fsleep(RTL819X_USB_PHY_DELAY_US);
	regmap_update_bits(usbphy->syscon, RTL819X_SYSCON_USB_PHYCR,
			   RTL819X_USB_PHY_RESET(port), 0);

	/* leave the port in normal (non-forced) UTMI mode */
	val = readl(usbphy->analog + RTL819X_USB_ANA_UTMI_CTRL);
	val &= ~(RTL819X_USB_UTMI_SUSPEND_FIX | RTL819X_USB_UTMI_NOFORCE(port));
	val |= RTL819X_USB_UTMI_FORCE(port);
	writel(val, usbphy->analog + RTL819X_USB_ANA_UTMI_CTRL);

	/* enable the PHY and give it a reset pulse (reset is active high) */
	regmap_update_bits(usbphy->syscon, RTL819X_SYSCON_USB_PHYCR,
			   RTL819X_USB_PHY_EN(port), RTL819X_USB_PHY_EN(port));
	regmap_update_bits(usbphy->syscon, RTL819X_SYSCON_USB_PHYCR,
			   RTL819X_USB_PHY_RESET(port),
			   RTL819X_USB_PHY_RESET(port));
	fsleep(RTL819X_USB_PHY_DELAY_US);
	regmap_update_bits(usbphy->syscon, RTL819X_SYSCON_USB_PHYCR,
			   RTL819X_USB_PHY_RESET(port), 0);
	regmap_update_bits(usbphy->syscon, RTL819X_SYSCON_USB_PHYCR,
			   RTL819X_USB_PHY_ACTIVE(port),
			   RTL819X_USB_PHY_ACTIVE(port));
	fsleep(RTL819X_USB_PHY_DELAY_US);

	rtl819x_usb_phy_tune(usbphy, port);
}

static int rtl819x_usb_phy_init(struct phy *phy)
{
	struct rtl819x_usb_phy_port *port = phy_get_drvdata(phy);

	rtl819x_usb_phy_port_init(port->usbphy, port->id);

	return 0;
}

static int rtl819x_usb_phy_exit(struct phy *phy)
{
	struct rtl819x_usb_phy_port *port = phy_get_drvdata(phy);
	struct rtl819x_usb_phy *usbphy = port->usbphy;

	regmap_update_bits(usbphy->syscon, RTL819X_SYSCON_USB_PHYCR,
			   RTL819X_USB_PHY_RESET(port->id),
			   RTL819X_USB_PHY_RESET(port->id));
	regmap_update_bits(usbphy->syscon, RTL819X_SYSCON_USB_PHYCR,
			   RTL819X_USB_PHY_ACTIVE(port->id), 0);

	return 0;
}

static const struct phy_ops rtl819x_usb_phy_ops = {
	.init		= rtl819x_usb_phy_init,
	.exit		= rtl819x_usb_phy_exit,
	.owner		= THIS_MODULE,
};

static struct phy *rtl819x_usb_phy_xlate(struct device *dev,
					 const struct of_phandle_args *args)
{
	struct rtl819x_usb_phy_port *ports = dev_get_drvdata(dev);

	if (args->args_count != 1 || args->args[0] >= RTL819X_USB_PHY_NPORTS)
		return ERR_PTR(-EINVAL);

	return ports[args->args[0]].phy;
}

static int rtl819x_usb_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtl819x_usb_phy *usbphy;
	struct rtl819x_usb_phy_port *ports;
	struct resource *res;
	unsigned int port;
	u32 val;

	usbphy = devm_kzalloc(dev, sizeof(*usbphy), GFP_KERNEL);
	if (!usbphy)
		return -ENOMEM;

	ports = devm_kcalloc(dev, RTL819X_USB_PHY_NPORTS, sizeof(*ports),
			     GFP_KERNEL);
	if (!ports)
		return -ENOMEM;

	usbphy->syscon = syscon_regmap_lookup_by_phandle(dev->of_node,
							 "syscon");
	if (IS_ERR(usbphy->syscon))
		return dev_err_probe(dev, PTR_ERR(usbphy->syscon),
				     "missing syscon phandle\n");

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "analog");
	usbphy->analog = devm_ioremap_resource(dev, res);
	if (IS_ERR(usbphy->analog))
		return PTR_ERR(usbphy->analog);

	/*
	 * The address strobe register sits inside the EHCI controller MMIO
	 * window claimed by the generic EHCI driver, so map it without
	 * requesting it as an exclusive region.
	 */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "utmi-acc");
	if (!res)
		return dev_err_probe(dev, -EINVAL,
				     "missing utmi-acc register\n");
	usbphy->acc = devm_ioremap(dev, res->start, resource_size(res));
	if (!usbphy->acc)
		return -ENOMEM;

	/* one-time block level setup: clock, host mode routing */
	regmap_update_bits(usbphy->syscon, RTL819X_SYSCON_USB_CLKCR,
			   RTL819X_SYSCON_USB_CLK_EN,
			   RTL819X_SYSCON_USB_CLK_EN);
	regmap_update_bits(usbphy->syscon, RTL819X_SYSCON_USB_HOSTCR,
			   RTL819X_SYSCON_USB_HOST_EN,
			   RTL819X_SYSCON_USB_HOST_EN);

	/*
	 * Two-port mode and route port 1 to the EHCI host controller
	 * instead of the DWC OTG device controller.
	 */
	val = RTL819X_USB_SIE_TWO_PORT | RTL819X_USB_SIE_PORT1_HOST;
	regmap_update_bits(usbphy->syscon, RTL819X_SYSCON_USB_SIECR,
			   RTL819X_USB_SIE_OTG_EN | val, val);

	regmap_read(usbphy->syscon, RTL819X_SYSCON_STRAP, &val);
	usbphy->xtal_40m = !!(val & RTL819X_SYSCON_STRAP_XTAL_40M);


	mutex_init(&usbphy->utmi_lock);

	for (port = 0; port < RTL819X_USB_PHY_NPORTS; port++) {
		ports[port].usbphy = usbphy;
		ports[port].id = port;
		ports[port].phy = devm_phy_create(dev, dev->of_node,
						  &rtl819x_usb_phy_ops);
		if (IS_ERR(ports[port].phy))
			return PTR_ERR(ports[port].phy);

		phy_set_drvdata(ports[port].phy, &ports[port]);
	}

	dev_set_drvdata(dev, ports);

	return PTR_ERR_OR_ZERO(devm_of_phy_provider_register(dev,
							rtl819x_usb_phy_xlate));
}

static const struct of_device_id rtl819x_usb_phy_of_match[] = {
	{ .compatible = "realtek,rtl8197f-usbphy" },
	{ }
};
MODULE_DEVICE_TABLE(of, rtl819x_usb_phy_of_match);

static struct platform_driver rtl819x_usb_phy_driver = {
	.probe	= rtl819x_usb_phy_probe,
	.driver	= {
		.name		= "realtek-rtl819x-usb2phy",
		.of_match_table	= rtl819x_usb_phy_of_match,
	},
};
module_platform_driver(rtl819x_usb_phy_driver);

MODULE_AUTHOR("David Yang <mmyangfl@gmail.com>");
MODULE_DESCRIPTION("Realtek RTL819x USB 2.0 PHY driver");
MODULE_LICENSE("GPL");
