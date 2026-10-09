// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTL8197F embedded switch quirk driver.
 *
 * The RTL8197F switch block (physical 0x1b800000, vendor SWITCH_BASE,
 * registers seen in the datasheet as 0xBB80_4xxx) has no real driver
 * yet -- no DSA, no L2/VLAN/offload management.  This driver takes the
 * core back from the bootloader instead of inheriting its state: it
 * resets the core (SIRR FULL_RST plus a clock gate cycle, the vendor
 * FullAndSemiReset()) and then programs, from the SDK field definitions
 * only, the least it needs to move frames between the CPU port and the
 * external switch.  Inheriting the previous stage is not an option:
 * what the NIC then sees depends on whatever ran before it.
 *
 * The bring-up after the reset is
 *  - port 0, the RGMII link to the external RTL8367RB: interface type,
 *    RX/TX delay codes, forced 1G/full-duplex link, PHY interface on;
 *  - the CPU interface, which is a port of this core too (logical port
 *    8, six MAC ports plus three extended ones): PHY interface on and
 *    forwarding STP state; a reset leaves PCRP8 zero, which keeps every
 *    frame away from the host rings;
 *  - a pass-through VLAN setup: ingress VLAN filtering off, unknown
 *    unicast/multicast to the CPU, and one netif table entry for the
 *    PVID the ports come out of reset with (that table is reached
 *    through the SWTACR/SWTASR/SWTAA command port, not mapped).
 *
 * It also performs the two quirks that are not part of that sequence:
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

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
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
#define RTL_SIRR_FULL_RST	BIT(2)	/* full reset: registers and tables too */
#define RTL_SW_MACCR		0x4000	/* MAC configuration */
#define RTL_SW_PITCR		0x4100	/* port interface type control */
#define RTL_SW_PCRP(p)		(0x4104 + ((p) << 2))	/* port configuration */
#define RTL_SW_P0GMIICR		0x414c	/* port 0 RGMII/GMII/MII configuration */
#define RTL_SW_MSCR		0x4410	/* module (L2/L3/L4) switch control */
#define RTL_SW_FFCR		0x4428	/* frame forwarding configuration */
#define RTL_SW_TEACR		0x4400	/* aging / L2 hash control */
#define RTL_SW_SWTCCR0		0x4418	/* switch core control 0 */
#define RTL_SW_SWTCCR1		0x441c	/* switch core control 1 */
#define RTL_SW_PLITIMR		0x4420	/* port learning limit timer */
#define RTL_SW_VCR0		0x4a00	/* VLAN control 0 */
#define RTL_SW_VCR1		0x4a04	/* VLAN control 1 */
#define RTL_SW_PVID(p)		(0x4a08 + (((p) * 2) & ~3))	/* PVID word of port p */
/* One word holds two ports: the even one in the low half (regs.h:2661-2696). */
#define RTL_PVCR_PVID_EVEN		GENMASK(11, 0)
#define RTL_PVCR_PVID_ODD		GENMASK(27, 16)
#define RTL_SW_ACL_ENTRY0	0xc0000	/* ACL table entry 0 */

/*
 * Hardware table access (rtl865xc_asicregs.h:2846-2884,
 * rtl865x_asicBasic.c:83-156): the VLAN netif table is not a register file,
 * it is reached through an address/data command port.
 */
#define RTL_SW_SWTACR		0x4d00
#define  RTL_SWTACR_ACTION_START	BIT(0)
#define  RTL_SWTACR_ACTION_MASK		BIT(0)
#define  RTL_SWTACR_ACTION_DONE		0
#define  RTL_SWTACR_CMD_ADD		BIT(1)
#define RTL_SW_SWTASR		0x4d04
#define  RTL_SWTASR_TABSTS_MASK		GENMASK(1, 0)
#define  RTL_SWTASR_TABSTS_SUCCESS	0
#define RTL_SW_SWTAA		0x4d08
#define RTL_SW_TCR(w)		(0x4d20 + ((w) << 2))
#define RTL_SWTBL_BASE		0xbb000000u	/* REAL_SWTBL_BASE */
#define RTL_TBL_TYPE_VLAN	6		/* TYPE_VLAN_TABLE */
#define RTL_TBL_ENTRY_BYTES	32		/* asic table entry length */

/* VLAN netif table entry, word 0 (rtl865x_asicCom.h:228-243). */
#define RTL_VLAN_MEMBER		GENMASK(5, 0)		/* MAC ports 0-5 */
#define RTL_VLAN_EXT_MEMBER	GENMASK(8, 6)		/* extended ports 6-8 */
#define RTL_VLAN_EGRESS_UNTAG	GENMASK(14, 9)
#define RTL_VLAN_EXT_EGRESS_UNTAG	GENMASK(17, 15)
#define RTL_VLAN_FID		GENMASK(19, 18)
#define RTL_VLAN_HP		GENMASK(22, 20)		/* 8197F only */

/*
 * Port numbering on this chip: six MAC ports (0-5) plus three extended
 * ports, of which the CPU interface is extended port 2, i.e. logical port 8
 * (RTL865XC_PORT_NUMBER 9 in rtl865xc_asicregs.h:206, RTK_CPU_PORT in
 * rtl_nic.c:1418, PCRP8 at 0x4124 in rtl865xc_asicregs.h:1148).  The vendor
 * enables that port's PHY interface in its table init (com.c:1105-1108) and
 * puts every port from 0 to 8 into the forwarding STP state (L2.c:7771-7773).
 */
#define RTL_PORT_CPU		8
#define RTL_PORT_EXT_FIRST	6

/* MSCR / TEACR / VCR0 / SWTCR0 / FFCR fields. */
#define RTL_MSCR_EN_L2		BIT(0)
#define RTL_VCR0_EN_VLAN_IN_FILTER	GENMASK(8, 0)	/* per-port VLAN ingress filter */
#define RTL_SWTCCR0_EN_UK_VID_TO_CPU	BIT(15)
#define RTL_FFCR_UNK_MC_TO_CPU	BIT(0)
#define RTL_FFCR_UNK_UC_TO_CPU	BIT(1)

/* MACCR[13:12] selects the Lexra bus clock the MAC runs off (1 = 100 MHz). */
#define RTL_MACCR_SYSCLK	GENMASK(13, 12)
#define RTL_MACCR_SYSCLK_100M	FIELD_PREP(RTL_MACCR_SYSCLK, 1)

/* Port configuration fields, vendor rtl865xc_asicregs.h (bootcode copy
 * include/rtl8196x/asicregs.h:980-1059).
 */
#define RTL_PCRP_EXT_ID_5	(5 << 26)	/* external-PHY slot for port 0 */
#define RTL_PCRP_FORCE_MODE	BIT(25)	/* use the forced link fields below */
#define RTL_PCRP_FORCE_LINK	BIT(23)	/* report link up */
#define RTL_PCRP_SPEED_1G	(2 << 19)	/* forced speed: 1G */
#define RTL_PCRP_FORCE_DUPLEX	BIT(18)	/* forced: full duplex */
#define RTL_PCRP_MII_RXER	BIT(13)	/* drive the interface RXER */
#define RTL_PCRP_STP		GENMASK(5, 4)	/* spanning-tree state */
#define RTL_PCRP_STP_FORWARDING	FIELD_PREP(RTL_PCRP_STP, 3)
#define RTL_PCRP_MAC_RESET	BIT(3)	/* 0 = hold the port MAC in reset */
#define RTL_PCRP_PHY_IF		BIT(0)	/* enable the PHY interface */
#define RTL_PITCR_P0_TYPE	GENMASK(1, 0)	/* port 0 interface type */
#define RTL_PITCR_P0_RGMII	FIELD_PREP(RTL_PITCR_P0_TYPE, 1)	/* GMII/MII/RGMII */

/*
 * Port 0's RGMII link to the external switch (rtl865x_asicL2.c
 * "init_8197f_p0").  RX delay code 5 and TX delay off are what the vendor
 * programs for this pair; SEL_RGTXC picks the transmit clock path and
 * CFG_GMAC = LINK_RGMII selects RGMII at bits 24:23 (0 there).  Conf_done
 * releases the port from its configuration latch.
 */
#define RTL_P0GMIICR_RX_DELAY		GENMASK(2, 0)
#define RTL_P0GMIICR_RX_DELAY_5		FIELD_PREP(RTL_P0GMIICR_RX_DELAY, 5)
#define RTL_P0GMIICR_TX_DELAY		BIT(4)
#define RTL_P0GMIICR_SEL_RGTXC		GENMASK(19, 18)
#define RTL_P0GMIICR_SEL_RGTXC_3	FIELD_PREP(RTL_P0GMIICR_SEL_RGTXC, 3)
#define RTL_P0GMIICR_CFG_GMAC		GENMASK(24, 23)	/* 0 = RGMII link */
#define RTL_P0GMIICR_CONF_DONE		BIT(6)

/* PIN_MUX_SEL2 lives in the system controller, outside the switch window. */
#define RTL819X_SYSCTL_BASE	0x18000800
#define RTL819X_PIN_MUX_SEL2	0x08
#define RTL819X_SYSCTL_SIZE	0x40	/* PIN_MUX_SEL14 sits at 0x38 */
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
static struct clk *rtl819x_sw_clk;

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

/*
 * Hard reset of the embedded switch core.  The sequence and the delays are
 * the vendor's FullAndSemiReset(), which the bootcode also runs as the very
 * first thing in its switch bring-up (rtl865x_asicCom.c "FullAndSemiReset()",
 * called by boot/rtl8196x/swCore.c swCore_init()): assert the full-reset
 * request, then gate the core clock off and back on so the state machine and
 * its tables really drop.
 *
 * Everything the bootloader had configured is gone afterwards, which is the
 * point: the core is then set up by this driver rather than inherited, so the
 * state the NIC sees does not depend on what the previous stage left behind.
 */
static int rtl819x_sw_reset(struct device *dev)
{
	int ret;

	ret = clk_prepare_enable(rtl819x_sw_clk);
	if (ret)
		return dev_err_probe(dev, ret, "cannot enable switch core clock\n");

	writel(readl(rtl819x_sw_base + RTL_SW_SIRR) | RTL_SIRR_FULL_RST,
	       rtl819x_sw_base + RTL_SW_SIRR);
	mdelay(300);

	clk_disable_unprepare(rtl819x_sw_clk);
	mdelay(300);

	ret = clk_prepare_enable(rtl819x_sw_clk);
	if (ret) {
		dev_err(dev, "cannot re-enable switch core clock: %d\n", ret);
		return ret;
	}
	mdelay(50);

	return 0;
}

static u16 rtl819x_sw_port_pvid(void __iomem *base, unsigned int port)
{
	u32 val = readl(base + RTL_SW_PVID(port));

	if (port & 1)
		return FIELD_GET(RTL_PVCR_PVID_ODD, val);
	return FIELD_GET(RTL_PVCR_PVID_EVEN, val);
}

/*
 * Install a VLAN netif table entry.  The table is not memory mapped: an entry
 * is staged in the access registers, the address is written, and the command
 * port is rung and polled (rtl865x_asicBasic.c
 * "_rtl8651_asicTableAccessForward()" / "_rtl8651_addAsicEntry()").  The entry
 * index is the VLAN id, so this core's 4096-VLAN model is addressed directly
 * (RTL865XC_VLANTBL_SIZE in rtl865x_asicCom.h:24-26).
 */
static int rtl819x_sw_vlan_add(struct device *dev, u32 vid, u32 word0)
{
	void __iomem *base = rtl819x_sw_base;
	unsigned int spin;

	for (spin = 100000;
	     (readl(base + RTL_SW_SWTACR) & RTL_SWTACR_ACTION_MASK) !=
		     RTL_SWTACR_ACTION_DONE; spin--)
		if (!spin) {
			dev_err(dev, "table port busy, VLAN %u not written\n", vid);
			return -ETIMEDOUT;
		}

	writel(word0, base + RTL_SW_TCR(0));
	writel(0, base + RTL_SW_TCR(1));
	writel(0, base + RTL_SW_TCR(2));
	writel(RTL_SWTBL_BASE + (RTL_TBL_TYPE_VLAN << 16) +
	       vid * RTL_TBL_ENTRY_BYTES, base + RTL_SW_SWTAA);
	writel(RTL_SWTACR_ACTION_START | RTL_SWTACR_CMD_ADD,
	       base + RTL_SW_SWTACR);

	for (spin = 100000;
	     (readl(base + RTL_SW_SWTACR) & RTL_SWTACR_ACTION_MASK) !=
		     RTL_SWTACR_ACTION_DONE; spin--)
		if (!spin) {
			dev_err(dev, "table add never finished, VLAN %u\n", vid);
			return -ETIMEDOUT;
		}

	if ((readl(base + RTL_SW_SWTASR) & RTL_SWTASR_TABSTS_MASK) !=
	    RTL_SWTASR_TABSTS_SUCCESS) {
		dev_err(dev, "table add refused, VLAN %u status %08x\n", vid,
			readl(base + RTL_SW_SWTASR));
		return -EIO;
	}

	return 0;
}

/*
 * What a reset leaves wrong are the two ports that frame traffic has to cross.
 * Port 0 is the RGMII link to the external RTL8367RB (that chip's own CPU side
 * is its port 6): after a reset its interface type, delay settings and PHY
 * interface are cleared.  And the CPU interface is a *port* of this core too --
 * logical port 8, six MAC ports plus three extended ones -- and its PCRP8 comes
 * back zero, which leaves it out of the forwarding state, so nothing reaches
 * the host ring either.  Both are set the way the vendor's 8197F path sets them
 * (rtl865x_asicL2.c "init_8197f_p0", and the port bring-up plus the STP loop
 * over ports 0..8 in its initAsicL2).
 *
 * The VLAN side is then kept as close to a pass-through as this core allows:
 * the ingress VLAN filter stays off with every port admitting all frames, as
 * the vendor sets it in initAsicL2; a frame whose VLAN is not in the netif
 * table is handed to the CPU instead of dropped; and the one table entry
 * installed is for the PVID the port registers already carry after the reset,
 * with port 0 and the CPU port as untagged members.  Nothing here is copied
 * from another stage's running state: the PVID is read back from the reset
 * default and the entry is derived from it.
 */
static int rtl819x_sw_configure(struct device *dev)
{
	u32 ext_member, pvid, val, word0;
	int ret;

	/* Port 0 runs RGMII, not the UTP interface the reset selects. */
	val = readl(rtl819x_sw_base + RTL_SW_PITCR);
	writel((val & ~RTL_PITCR_P0_TYPE) | RTL_PITCR_P0_RGMII,
	       rtl819x_sw_base + RTL_SW_PITCR);

	/* Port 0 RGMII: RX delay code 5, no TX delay, RGMII clock path. */
	val = readl(rtl819x_sw_base + RTL_SW_P0GMIICR);
	val &= ~(RTL_P0GMIICR_RX_DELAY | RTL_P0GMIICR_TX_DELAY |
		 RTL_P0GMIICR_SEL_RGTXC | RTL_P0GMIICR_CFG_GMAC);
	val |= RTL_P0GMIICR_RX_DELAY_5 | RTL_P0GMIICR_SEL_RGTXC_3 |
	       RTL_P0GMIICR_CONF_DONE;
	writel(val, rtl819x_sw_base + RTL_SW_P0GMIICR);

	writel(RTL_PCRP_EXT_ID_5 | RTL_PCRP_FORCE_MODE | RTL_PCRP_FORCE_LINK |
	       RTL_PCRP_SPEED_1G | RTL_PCRP_FORCE_DUPLEX | RTL_PCRP_MII_RXER |
	       RTL_PCRP_PHY_IF | RTL_PCRP_MAC_RESET | RTL_PCRP_STP_FORWARDING,
	       rtl819x_sw_base + RTL_SW_PCRP(0));

	/*
	 * The CPU port keeps whatever external-PHY id the reset gives it (it
	 * names the port itself, not a MDIO address); only the three fields
	 * that decide whether frames may cross it are written.
	 */
	val = readl(rtl819x_sw_base + RTL_SW_PCRP(RTL_PORT_CPU));
	val &= ~RTL_PCRP_STP;
	writel(val | RTL_PCRP_PHY_IF | RTL_PCRP_MAC_RESET |
	       RTL_PCRP_STP_FORWARDING,
	       rtl819x_sw_base + RTL_SW_PCRP(RTL_PORT_CPU));

	writel(RTL_MSCR_EN_L2, rtl819x_sw_base + RTL_SW_MSCR);
	writel(readl(rtl819x_sw_base + RTL_SW_MACCR) | RTL_MACCR_SYSCLK_100M,
	       rtl819x_sw_base + RTL_SW_MACCR);

	writel(0, rtl819x_sw_base + RTL_SW_TEACR);
	writel(0, rtl819x_sw_base + RTL_SW_PLITIMR);
	writel(0, rtl819x_sw_base + RTL_SW_VCR0);
	writel(0, rtl819x_sw_base + RTL_SW_VCR1);

	writel(readl(rtl819x_sw_base + RTL_SW_SWTCCR0) |
	       RTL_SWTCCR0_EN_UK_VID_TO_CPU,
	       rtl819x_sw_base + RTL_SW_SWTCCR0);
	writel(0, rtl819x_sw_base + RTL_SW_SWTCCR1);

	/*
	 * Unlearnt unicast and multicast must reach the bridge.  This is an
	 * OR, not a replacement: FFCR[11:10] selects what the core does with
	 * IP multicast whose source is unmatched (0 = to the CPU, 1 = drop,
	 * 2 = flood) and comes out of reset as 1; overwriting the whole
	 * register would silently change that policy.
	 */
	val = readl(rtl819x_sw_base + RTL_SW_FFCR);
	writel(val | RTL_FFCR_UNK_UC_TO_CPU | RTL_FFCR_UNK_MC_TO_CPU,
	       rtl819x_sw_base + RTL_SW_FFCR);

	/*
	 * The netif table decides where a frame found by L2 lookup may go, so
	 * the PVID the ports were reset to needs an entry that names port 0 and
	 * the CPU port.  Read the id back rather than assuming it, and refuse to
	 * configure if the two ports disagree -- then there is no single VLAN
	 * that connects them and the entry below would be meaningless.
	 */
	pvid = FIELD_GET(RTL_PVCR_PVID_EVEN,
			 readl(rtl819x_sw_base + RTL_SW_PVID(0)));
	if (pvid != rtl819x_sw_port_pvid(rtl819x_sw_base, RTL_PORT_CPU)) {
		dev_err(dev, "port 0 PVID %u, CPU port PVID %u: no common VLAN\n",
			pvid, rtl819x_sw_port_pvid(rtl819x_sw_base, RTL_PORT_CPU));
		return -EINVAL;
	}

	/*
	 * Both ends untagged: the tag this link carries is the switch's own
	 * header appended by the DSA driver, not an 802.1Q tag, so the core
	 * must not insert one on egress in either direction.
	 */
	ext_member = BIT(RTL_PORT_CPU - RTL_PORT_EXT_FIRST);
	word0 = FIELD_PREP(RTL_VLAN_MEMBER, BIT(0)) |
		FIELD_PREP(RTL_VLAN_EXT_MEMBER, ext_member) |
		FIELD_PREP(RTL_VLAN_EGRESS_UNTAG, BIT(0)) |
		FIELD_PREP(RTL_VLAN_EXT_EGRESS_UNTAG, ext_member);
	ret = rtl819x_sw_vlan_add(dev, pvid, word0);
	if (ret)
		return ret;

	return 0;
}

static int rtl819x_sw_probe(struct platform_device *pdev)
{
	void __iomem *pinmux;
	void __iomem *swtable;
	u32 val;
	int ret;

	rtl819x_sw_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rtl819x_sw_base))
		return PTR_ERR(rtl819x_sw_base);

	rtl819x_sw_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(rtl819x_sw_clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(rtl819x_sw_clk),
				     "no switch core clock\n");

	/*
	 * Board quirk: hand the internal switch's SMI pins to the GPIO
	 * controller so the external switch (RTL8367RB, realtek-smi DSA)
	 * can be managed.  Non-fatal if the window cannot be mapped --
	 * the internal PHYs keep their dedicated management pins then,
	 * only the external SMI bit-bang breaks.
	 */
	pinmux = devm_ioremap(&pdev->dev, RTL819X_SYSCTL_BASE,
			      RTL819X_SYSCTL_SIZE);
	if (pinmux) {
		val = readl(pinmux + RTL819X_PIN_MUX_SEL2);
		val = (val & ~PIN_MUX_SEL2_SMI_MASK) | PIN_MUX_SEL2_SMI_GPIO;
		writel(val, pinmux + RTL819X_PIN_MUX_SEL2);

		val = readl(pinmux + RTL819X_PIN_MUX_SEL14);
		val = (val & ~PIN_MUX_SEL14_H2_MASK) |
		      (PIN_MUX_SEL14_H2_GPIO << 28);
		writel(val, pinmux + RTL819X_PIN_MUX_SEL14);
	} else {
		dev_warn(&pdev->dev,
			 "cannot map the system controller, external SMI unusable\n");
	}

	ret = rtl819x_sw_parse_conduit(&pdev->dev);
	if (ret)
		return ret;

	/*
	 * Take the core back from whatever the previous stage left behind,
	 * then configure it here.
	 */
	ret = rtl819x_sw_reset(&pdev->dev);
	if (ret)
		return ret;

	ret = rtl819x_sw_configure(&pdev->dev);
	if (ret)
		return ret;

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
	 * like the boot code does at the end of its switch initialisation
	 * (boot/rtl8196x/swCore.c:1509), because it is a property of the CPU
	 * port being wired up, not of the MAC being opened: the MAC has its
	 * own run/stop bits (CPUICR TXCMD/RXCMD, doorbell TXFD).  The vendor
	 * pairs its set in rtl865x_start() with a clear in rtl865x_down(), so
	 * a test build was booted that also cleared this bit every time the
	 * conduit MAC went down (SIRR read back 0x00000001 -> 0x00000000) and
	 * never re-armed it: traffic still passed.  Measured by cycling the
	 * MAC itself -- three "ip link set eth0 down/up" rounds, 20/20 ICMP
	 * each way, 4 MB copied each way with matching md5 -- so no per-open
	 * arm is needed here.
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
