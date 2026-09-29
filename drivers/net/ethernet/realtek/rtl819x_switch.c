// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTL8197F embedded switch quirk driver.
 *
 * The RTL8197F switch block (physical 0x1b800000, vendor SWITCH_BASE,
 * registers seen in the datasheet as 0xBB80_4xxx) has no real driver
 * yet -- no DSA, no L2/VLAN/offload management; the bootloader leaves
 * it configured.  This driver only claims the register window and
 * performs the minimal, mandatory steps without which the CPU-port
 * NIC (rtl819x_eth) cannot move frames:
 *
 *  - pinmux quirk (vendor PIN_MUX_SEL2 at 0x18000808): mux the internal
 *    switch's P0MDC/P0MDIO pins to GPIO so the realtek-smi DSA driver
 *    can bit-bang the SMI lines PC2/PC3 to the external switch;
 *
 *  - the SIRR TRXRDY trigger (bit 0, 0xBB804204) that lets the
 *    CPU-interface DMA deliver frames; raised once here at the end of
 *    probe, so the Ethernet MAC never has to reach into this block.
 *
 *  - the probe tail performs a dummy read of the ACL table entry 0
 *    (0xBB0C0000), vendor workaround for "driver can't receive packet
 *    when LAN un-plug"; the switch is not reset between opens, so the
 *    poke is done once at probe rather than on every open.
 *
 * Register semantics are taken from the vendor SDK:
 *   rtk_openwrtSDK_v2.5 drivers/net/rtl819x/
 *     AsicDriver/rtl865xc_asicregs.h, rtl865x/rtl865x_asicCom.c
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>

#define RTL819X_SW_DRV_NAME	"rtl819x-switch"

/* Switch register window (SWITCH_BASE), byte offsets. */
#define RTL_SW_SIRR		0x4204	/* switch interrupt raw request */
#define RTL_SIRR_TRXRDY		BIT(0)	/* rx/tx descriptor processing trigger */
#define RTL_SW_ACL_ENTRY0	0xc0000	/* ACL table entry 0 */

/* PIN_MUX_SEL2 lives in the system controller, outside the switch window. */
#define RTL819X_SYSCTL_BASE	0x18000800
#define RTL819X_PIN_MUX_SEL2	0x08
#define RTL819X_PIN_MUX_SEL14	0x38
/*
 * P0MDC (bits 11:8) and P0MDIO (bits 7:4) both 0110 = GPIO function.  The
 * 4-bit fields may hold a different (LED or switch) function after boot;
 * always clear them before setting, like the vendor pin_enable() does.
 */
#define PIN_MUX_SEL2_SMI_MASK	GENMASK(11, 4)
#define PIN_MUX_SEL2_SMI_GPIO	0x660

/*
 * H2 (PIN_MUX_SEL14[31:28] = 2) is the reset line of the external
 * RTL8367RB; the vendor gpio.c muxes it explicitly because it has no
 * default GPIO entry.
 */
#define PIN_MUX_SEL14_H2_MASK	GENMASK(31, 28)
#define PIN_MUX_SEL14_H2_GPIO	2

static void __iomem *rtl819x_sw_base;

/*
 * Check which MAC the CPU interface feeds, from the CPU port's "ethernet"
 * phandle, and defer the probe until that MAC has registered its netdev.  The
 * two drivers still know nothing about each other: this is a device tree
 * reference, not a function call.
 */
static int rtl819x_sw_parse_conduit(struct device *dev)
{
	struct device_node *ports, *port = NULL, *eth = NULL;
	struct net_device *conduit;
	int ret = -EINVAL;

	ports = of_get_child_by_name(dev->of_node, "ports");
	if (!ports) {
		dev_err(dev, "missing ports node (no CPU port conduit)\n");
		return -EINVAL;
	}

	for_each_child_of_node(ports, port) {
		eth = of_parse_phandle(port, "ethernet", 0);
		if (eth)
			break;
	}

	if (!eth) {
		dev_err(dev, "no port references a conduit with \"ethernet\"\n");
		of_node_put(port);
		goto out_ports;
	}

	conduit = of_find_net_device_by_node(eth);
	of_node_put(eth);
	if (!conduit) {
		of_node_put(port);
		ret = -EPROBE_DEFER;
		goto out_ports;
	}
	put_device(&conduit->dev);

	of_node_put(port);
	ret = 0;
out_ports:
	of_node_put(ports);
	return ret;
}

static int rtl819x_sw_probe(struct platform_device *pdev)
{
	void __iomem *pinmux;
	void __iomem *swtable;
	int ret;

	rtl819x_sw_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rtl819x_sw_base))
		return PTR_ERR(rtl819x_sw_base);

	/*
	 * The CPU interface feeds one of the MACs of this SoC; wait until that
	 * MAC has registered its netdevice before claiming anything here.
	 */
	ret = rtl819x_sw_parse_conduit(&pdev->dev);
	if (ret)
		return ret;

	/*
	 * Board quirk: hand the internal switch's SMI pins to the GPIO
	 * controller so the external switch (RTL8367RB, realtek-smi DSA)
	 * can be managed.  Non-fatal if the window cannot be mapped --
	 * the internal PHYs keep their dedicated management pins then,
	 * only the external SMI bit-bang breaks.
	 */
	u32 val;

	pinmux = ioremap(RTL819X_SYSCTL_BASE, 0x10);
	if (!IS_ERR_OR_NULL(pinmux)) {
		val = readl(pinmux + RTL819X_PIN_MUX_SEL2);
		val = (val & ~PIN_MUX_SEL2_SMI_MASK) | PIN_MUX_SEL2_SMI_GPIO;
		writel(val, pinmux + RTL819X_PIN_MUX_SEL2);

		val = readl(pinmux + RTL819X_PIN_MUX_SEL14);
		val = (val & ~PIN_MUX_SEL14_H2_MASK) |
		      (PIN_MUX_SEL14_H2_GPIO << 28);
		writel(val, pinmux + RTL819X_PIN_MUX_SEL14);
		iounmap(pinmux);
	} else {
		dev_warn(&pdev->dev,
			 "cannot map PIN_MUX_SEL2, external SMI unusable\n");
	}

	/*
	 * vendor rtl865x_start() workaround, see file comment.  The ACL
	 * table window is a separate bus window (SWTABLE base, physical
	 * 0x1B0C0000), not an offset inside the switch register window.
	 * It is done once here rather than on every NIC open: the window
	 * would have to be ioremap()ed on a sleepable path, and the NIC
	 * must never take an ioremap inside tx_timeout().  The switch is
	 * not reset between opens, so one poke after probe suffices.
	 */
	swtable = ioremap(0x1b0c0000, 0x10);
	if (!IS_ERR_OR_NULL(swtable)) {
		readl(swtable);
		iounmap(swtable);
	} else {
		dev_warn(&pdev->dev, "cannot map ACL table window\n");
	}

	/*
	 * Hand the CPU interface to the descriptor engine: SIRR TRXRDY (bit 0)
	 * is what makes this core feed port 8 to the MAC.  Raised once here,
	 * like the boot code does at the end of its switch initialisation.
	 */
	writel(readl(rtl819x_sw_base + RTL_SW_SIRR) | RTL_SIRR_TRXRDY,
	       rtl819x_sw_base + RTL_SW_SIRR);

	return 0;
}

static const struct of_device_id rtl819x_sw_of_match[] = {
	{ .compatible = "realtek,rtl8197f-switch" },
	{ }
};
MODULE_DEVICE_TABLE(of, rtl819x_sw_of_match);

static struct platform_driver rtl819x_sw_driver = {
	.probe	= rtl819x_sw_probe,
	.driver	= {
		.name		= RTL819X_SW_DRV_NAME,
		.of_match_table	= rtl819x_sw_of_match,
	},
};
module_platform_driver(rtl819x_sw_driver);

MODULE_AUTHOR("David Yang <mmyangfl@gmail.com>");
MODULE_DESCRIPTION("Realtek RTL8197F embedded switch quirk driver");
MODULE_LICENSE("GPL");
