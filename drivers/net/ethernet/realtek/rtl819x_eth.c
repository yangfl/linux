// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTL8197F Ethernet MAC / embedded switch CPU interface driver.
 *
 * The RTL8197F SoC contains a 5+1 port Ethernet switch.  The CPU talks to
 * the switch through a DMA engine (the "NIC" or "CPU interface" block,
 * physical 0x18010000) which fetches packet descriptors from memory rings.
 *
 * Scope of this driver:
 *  - single netdev (eth0) over the CPU port, NAPI rx/tx descriptor rings;
 *  - the embedded switch itself is owned by rtl819x_switch.c, a quirk
 *    driver that resets the core and performs the mandatory bring-up
 *    (register window, SMI pinmux, ACL workaround, TRXRDY trigger).
 *    This driver
 *    deliberately does not touch the switch L2 tables or offload engines;
 *  - no PHY management of its own: the MAC side of the CPU-port link is
 *    attached to phylink; boards wire the switch CPU port (RGMII on the
 *    RTL8197F) either as a fixed-link or through a phy-handle;
 *  - hardware L2/L3/L4 offload (NAPT, hw checksum path via the switch
 *    lookup engine) is not used: every Tx descriptor is sent in
 *    "direct Tx" mode (no HW lookup, no bridging), checksums are done in
 *    software.
 *
 * Register semantics are taken from the vendor SDK:
 *   rtk_openwrtSDK_v2.5 drivers/net/rtl819x/
 *     rtl819x_swNic.c, rtl819x_swNic.h
 *     AsicDriver/rtl865xc_asicregs.h
 *   RTL8197_3411D_2 rtl819x/boards/rtl8197F/bsp/bspchip.h
 * Vendor register names are kept in comments to ease cross-checking.
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/phylink.h>
#include <linux/skbuff.h>
#include <linux/if_vlan.h>
#include <net/page_pool/helpers.h>

#define RTL819X_DRV_NAME	"rtl819x-eth"

/* ------------------------------------------------------------------ */
/* Register block                                                      */
/* ------------------------------------------------------------------ */

/* CPU interface register offsets, from the "nic" reg window.
 * Vendor: CPU_IFACE_BASE = SYSTEM_BASE + 0x10000 = phys 0x18010000,
 * AsicDriver/rtl865xc_asicregs.h.
 */
#define RTL_CPUICR		0x00	/* CPU interface control */
#define RTL_CPURPDCR0		0x04	/* Rx pkthdr desc ring 0 base ptr */
#define RTL_CPUTPDCR0		0x20	/* Tx pkthdr desc ring 0 base ptr */
#define RTL_CPUIIMR		0x28	/* interrupt mask */
#define RTL_CPUIISR		0x2c	/* interrupt status (write 1 to clear) */
#define RTL_DMA_CR0		0x3c	/* DMA control 0 (FIFO watermarks) */
#define RTL_DMA_CR1		0x40	/* DMA control 1 (Tx ring0 tail offset) */
#define RTL_DMA_CR4		0xa0	/* DMA control 4 (Tx ring0 tail aware) */
#define RTL_CPUICR1		0xa4	/* CPU interface control 1 */

/* RTL_DMA_CR0 [15:8] = HiFifoMark, [7:0] = LowFifoMark.  Vendor
 * rtl865x_start() restores 0xA0/0xA0 for the 8197F because the defaults
 * (0x57) are clobbered when the bus burst size changes.  The upper half
 * of the register holds unrelated fields and is preserved by the
 * read-modify-write.
 */
#define RTL_DMA_CR0_HIFIFO	GENMASK(15, 8)
#define RTL_DMA_CR0_LOFIFO	GENMASK(7, 0)
#define RTL_DMA_CR0_HIFIFO_MARK	0xa0
#define RTL_DMA_CR0_LOFIFO_MARK	0xa0

/*
 * RTL_CPUICR1 [9:8] selects the descriptor format the DMA consumes:
 * 00 = classic 4-byte pkthdr pointers, 01 = the RTL8198C/8197F 6-word
 * descriptor ("shortcut"/LSO) format, which is what the vendor SDK
 * programs in re865x_probe() and what this driver's rings use.
 */
#define RTL_CPUICR1_PKTHDR	GENMASK(9, 8)
#define RTL_CPUICR1_PKTHDR_SHORTCUT	FIELD_PREP(RTL_CPUICR1_PKTHDR, 1)

/* RTL_CPUICR bits (rtl865xc_asicregs.h) */
#define RTL_TXCMD		BIT(31)		/* Tx DMA enable */
#define RTL_RXCMD		BIT(30)		/* Rx DMA enable */
#define RTL_BUSBURST_128W	(2U << 28)	/* Lexra bus burst = 128 bytes */
#define RTL_MBUF_2048B		(4U << 24)	/* buffer size indication */
#define RTL_TXFD		BIT(23)		/* kick Tx descriptor fetch */

/* RTL_CPUIIMR / RTL_CPUIISR bits (rtl865xc_asicregs.h).  The ISR is
 * write-one-to-clear.
 */
#define RTL_LINK_CHANGE_IE	BIT(31)
#define RTL_TX_DONE_IE1		BIT(10)
#define RTL_TX_DONE_IE0		BIT(9)
#define RTL_TX_ALL_DONE_IE3	BIT(13)
#define RTL_TX_ALL_DONE_IE2	BIT(12)
#define RTL_TX_ALL_DONE_IE1	BIT(2)
#define RTL_TX_ALL_DONE_IE0	BIT(1)
#define RTL_RX_DONE_IE(i)	BIT(3 + (i))
#define RTL_RX_DONE_IE_ALL	(0x3fU << 3)

/*
 * The vendor driver enables only these: RX_ERR/TX_ERR are never enabled,
 * and PKTHDR_RUNOUT is left off as well -- while descriptors are starved
 * the runout condition is level-asserted and the interrupt storms.
 */
#define RTL_IRQ_ALL		(RTL_RX_DONE_IE_ALL | \
				 RTL_TX_ALL_DONE_IE0 | RTL_TX_ALL_DONE_IE1 | \
				 RTL_TX_ALL_DONE_IE2 | RTL_TX_ALL_DONE_IE3 | \
				 RTL_TX_DONE_IE0 | RTL_TX_DONE_IE1 | \
				 RTL_LINK_CHANGE_IE)

/*
 * Only Rx ring 0 and Tx ring 0 are used by this driver.  The hardware
 * offers up to 6 Rx and 4 Tx rings (vendor NEW_NIC_MAX_*_DESC_RING); the
 * unused ring base registers are left untouched.
 */
/* ------------------------------------------------------------------ */
/* Descriptor format                                                   */
/* ------------------------------------------------------------------ */

/*
 * Descriptors are 6 words (24 bytes): opts1, buffer address, opts2..opts5.
 * Both rings use the same layout.  The hardware is little-endian.
 * Field positions follow rtl819x_swNic.h.
 */
struct rtl819x_desc {
	__le32	opts1;
	__le32	addr;
	__le32	opts2;
	__le32	opts3;
	__le32	opts4;
	__le32	opts5;
};

/* opts1 bits shared by Tx and Rx (rtl819x_swNic.h DescOwn etc.) */
#define RTL_DESC_OWN		BIT(0)	/* 1 = switch/DMA owned */
#define RTL_DESC_RING_END	BIT(1)	/* last descriptor of the ring */
#define RTL_DESC_LAST_FRAG	BIT(2)
#define RTL_DESC_FIRST_FRAG	BIT(3)

/* Rx opts1 [31:16]: buffer size handed to the DMA */
#define RTL_RD_SIZE_SHIFT	16
#define RTL_RD_SIZE_MASK	(0xffffU << RTL_RD_SIZE_SHIFT)

/* Rx opts2 [13:0]: received length including 4-byte FCS */
#define RTL_RD_LEN_MASK		0x3fff

/* Tx opts1 [22:6]: packet length (rtl819x_swNic.h TD_PHLEN) */
#define RTL_TD_PHLEN_SHIFT	6
/* Tx opts2 [31:15]: Tx data length (TD_M_LEN) */
#define RTL_TD_MLEN_SHIFT	15
/* Tx opts4 [30:24]: destination port mask (TD_DP), bit 6 = CPU port */
#define RTL_TD_DP_SHIFT		24
#define RTL_TD_DP_MASK		0x7fU

#define RTL_NUM_RX_DESC		256
#define RTL_NUM_TX_DESC		64
#define RTL_TX_WAKE_THRESH	(RTL_NUM_TX_DESC / 4)	/* free slots to wake */
#define RTL_RX_BUF_SIZE		2048	/* cluster size, see MBUF_2048B */

/*
 * The DMA never spans descriptors here: buffers are larger than the MTU,
 * so a packet always occupies exactly one descriptor.  Multi-descriptor
 * packets (FirstFrag/LastFrag) are dropped defensively.
 */
#define RTL_MIN_TX_LEN		60	/* pad; HW appends 4-byte FCS */

struct rtl819x_tx_info {
	struct sk_buff	*skb;
	DEFINE_DMA_UNMAP_ADDR(mapping);
	u32		map_len;	/* mapped length (padded, +4 FCS) */
};

struct rtl819x_priv {
	struct device		*dev;
	struct net_device	*ndev;
	struct napi_struct	napi;
	void __iomem		*base;
	int			irq;

	u32			tx_portmask;

	/* descriptor rings (coherent memory) */
	struct rtl819x_desc	*rx_ring;
	dma_addr_t		rx_ring_dma;
	/*
	 * Per-descriptor rx buffer slots.  The engine rewrites each
	 * descriptor address to point at the frame start inside the slot
	 * and, like the vendor SDK (which uses a fixed mbuf pool), derives
	 * the slot position from that address: handing it arbitrary slab
	 * addresses makes it write outside of the buffers, corrupting
	 * unrelated memory.  Slots are therefore order-0 pages, which are
	 * 4 KiB-aligned.  The engine does not require the *same* buffer for
	 * the lifetime of the ring, only the size/alignment: completed
	 * slots are handed to the stack zero-copy (build_skb() on the slot
	 * page) and the descriptor is refilled with a fresh page.  The
	 * engine only ever writes the first 2 KiB of the page (the opts1
	 * buffer size field); build_skb() is given PAGE_SIZE so
	 * skb_shared_info lands past it, out of the DMA's reach.  When a
	 * refill allocation fails the frame is copied out and the same slot
	 * is recycled.
	 */
	struct page		*rx_pages[RTL_NUM_RX_DESC];
	struct page_pool	*rx_pool;
	unsigned int		rx_dirty;	/* next desc to own-check */

	struct rtl819x_desc	*tx_ring;
	dma_addr_t		tx_ring_dma;
	struct rtl819x_tx_info	*tx_info;
	unsigned int		tx_curr;	/* next desc to fill */
	unsigned int		tx_dirty;	/* oldest not-yet-completed desc */

	spinlock_t		tx_lock;	/* tx_curr / queue handling */

	struct phylink		*phylink;
	struct phylink_config	phylink_config;
};

/* ------------------------------------------------------------------ */
/* Descriptor helpers                                                  */
/* ------------------------------------------------------------------ */

static inline u32 rtl_rr(struct rtl819x_priv *priv, unsigned int off)
{
	return readl(priv->base + off);
}

static inline void rtl_wr(struct rtl819x_priv *priv, unsigned int off,
			  u32 val)
{
	writel(val, priv->base + off);
}

static inline unsigned int rtl_next_idx(unsigned int idx, unsigned int size)
{
	return (idx + 1 == size) ? 0 : idx + 1;
}

static void rtl_kick_tx(struct rtl819x_priv *priv)
{
	/* notify the DMA that new Tx descriptors are available (TXFD) */
	rtl_wr(priv, RTL_CPUICR, rtl_rr(priv, RTL_CPUICR) | RTL_TXFD);
}

/*
 * Hot counters live in per-cpu storage: both writers (NAPI poll and
 * start_xmit) run with softirqs disabled, so this_cpu_ptr() plus the u64
 * sequence guard gives readers of ndo_get_stats64 a consistent 64-bit value on
 * this 32-bit part.
 */

static void rtl819x_count_rx(struct net_device *ndev, unsigned int len)
{
	struct pcpu_sw_netstats *stats = this_cpu_ptr(ndev->tstats);

	u64_stats_update_begin(&stats->syncp);
	u64_stats_add(&stats->rx_packets, 1);
	u64_stats_add(&stats->rx_bytes, len);
	u64_stats_update_end(&stats->syncp);
}

static void rtl819x_count_tx(struct net_device *ndev, unsigned int len)
{
	struct pcpu_sw_netstats *stats = this_cpu_ptr(ndev->tstats);

	u64_stats_update_begin(&stats->syncp);
	u64_stats_add(&stats->tx_packets, 1);
	u64_stats_add(&stats->tx_bytes, len);
	u64_stats_update_end(&stats->syncp);
}

/*
 * Back rx descriptor @idx with a fresh page slot and hand it to the engine.
 * Pages come from the rx page_pool: DMA-mapped once at pool allocation and
 * recycled when the consuming skb is freed (the skb is marked for recycle),
 * so refill does no cache-avoiding map/unmap work even under load.  Usable
 * from both open() (GFP_KERNEL) and NAPI poll (GFP_ATOMIC).
 */
static int rtl819x_rx_refill(struct rtl819x_priv *priv, unsigned int idx,
			     gfp_t gfp)
{
	struct page *page;

	page = page_pool_alloc_pages(priv->rx_pool, gfp);
	if (!page)
		return -ENOMEM;

	/* the pool synced the whole slot for device at allocation */
	priv->rx_pages[idx] = page;
	priv->rx_ring[idx].addr =
		cpu_to_le32(page_pool_get_dma_addr(page));
	/* buffer address visible before handing the descriptor to the engine */
	dma_wmb();
	priv->rx_ring[idx].opts1 =
		cpu_to_le32((RTL_RX_BUF_SIZE << RTL_RD_SIZE_SHIFT) |
			    RTL_DESC_OWN |
			    (idx == RTL_NUM_RX_DESC - 1 ?
				RTL_DESC_RING_END : 0));

	return 0;
}

/* ------------------------------------------------------------------ */
/* NAPI                                                                */
/* ------------------------------------------------------------------ */

static int rtl819x_poll(struct napi_struct *napi, int budget)
{
	struct rtl819x_priv *priv = container_of(napi, struct rtl819x_priv,
						 napi);
	struct net_device *ndev = priv->ndev;
	int work_done = 0;
	unsigned int dirty;

	/* ---- Rx ---- */
	/*
	 * Frames are claimed by scanning the descriptor ring; completion
	 * is judged by the OWN bit alone.  Gating the loop on the
	 * CPURPDCR0 write pointer would stall the interface whenever the
	 * engine's pointer rests on the descriptor that holds the last
	 * completed frame, and that frame is then lost together with its
	 * already-acknowledged interrupt.
	 */
	while (work_done < budget) {
		struct rtl819x_desc *desc = &priv->rx_ring[priv->rx_dirty];
		struct sk_buff *skb;
		struct page *new_page = NULL;
		unsigned int idx = priv->rx_dirty;
		dma_addr_t buf_dma =
			page_pool_get_dma_addr(priv->rx_pages[idx]);
		u32 opts1, opts2;
		unsigned int len, off;

		opts1 = le32_to_cpu(desc->opts1);
		if (opts1 & RTL_DESC_OWN)
			break;

		opts2 = le32_to_cpu(desc->opts2);
		len = (opts2 & RTL_RD_LEN_MASK) - 4; /* strip FCS */
		off = le32_to_cpu(desc->addr) - buf_dma;

		/*
		 * The DMA rewrites the descriptor address field (mdata) to
		 * point at the actual start of the received frame inside
		 * the slot page (vendor: r_skb->data = desc->mdata).  Frame
		 * start and length must both stay inside the 2 KiB slot;
		 * the vendor notes mdata can be an internal mbuf pointer
		 * when the CPU interface is misconfigured.  Written so that
		 * neither off nor len can wrap around.
		 */
		if (unlikely(!(opts1 & RTL_DESC_FIRST_FRAG) ||
			     !(opts1 & RTL_DESC_LAST_FRAG) ||
			     len < ETH_HLEN ||
			     off >= RTL_RX_BUF_SIZE ||
			     off + len > RTL_RX_BUF_SIZE - 2)) {
			netdev_err(ndev,
				   "bad rx frame %08x buf %08x len %u (o1=%08x o2=%08x)\n",
				   le32_to_cpu(desc->addr), buf_dma, len,
				   opts1, opts2);
			ndev->stats.rx_dropped++;
			goto rearm;
		}

		/*
		 * Zero-copy delivery: prepare a fresh slot page first, then
		 * build the skb directly on the completed page and hand it
		 * to the stack; the descriptor is refilled with the new
		 * page.  When no page is available, fall back to copying
		 * the frame out so the same slot can be recycled and the
		 * ring keeps running.
		 */
		new_page = page_pool_alloc_pages(priv->rx_pool,
						 GFP_ATOMIC | __GFP_NOWARN);
		skb = NULL;
		if (new_page)
			/*
			 * PAGE_SIZE so skb_shared_info lands past the
			 * engine's 2 KiB slot (see rx_pages above); a
			 * frag_size of 2048 would put it inside the frame
			 * area.
			 */
			skb = build_skb(page_address(priv->rx_pages[idx]),
					PAGE_SIZE);
		if (skb) {
			/*
			 * The consuming skb owns the completed slot page; with
			 * skb_mark_for_recycle() it returns to the pool on
			 * free, so no unmap is ever needed.  Only the frame
			 * range is synced for the CPU.
			 */
			dma_sync_single_for_cpu(priv->dev, buf_dma + off, len,
						DMA_FROM_DEVICE);
			priv->rx_pages[idx] = new_page;
			skb_mark_for_recycle(skb);

			skb_reserve(skb, off);	/* frame start inside slot */
			skb_put(skb, len);

			/*
			 * The engine is in PKT_HDR mode (CPUICR1 [9:8]=1, as
			 * the vendor SDK programs it): the DMA overwrites the
			 * first six bytes of the frame -- the destination MAC
			 * -- with a fixed header (56 aa a5 5a <metadata>).
			 * Restore the dst MAC; anything the DSA switch inserts
			 * after the source MAC (CPU tag, VLAN) stays in the
			 * frame untouched and is parsed by the tag driver
			 * above us.
			 */
			ether_addr_copy(&skb->data[0], ndev->dev_addr);
			skb->protocol = eth_type_trans(skb, ndev);
			rtl819x_count_rx(ndev, len);

			napi_gro_receive(napi, skb);
			work_done++;

			desc->addr = cpu_to_le32(page_pool_get_dma_addr(new_page));
			/* buffer address visible before handing the descriptor to the engine */
			dma_wmb();
			desc->opts1 =
				cpu_to_le32((RTL_RX_BUF_SIZE <<
					     RTL_RD_SIZE_SHIFT) |
					    RTL_DESC_OWN |
					    (idx == RTL_NUM_RX_DESC - 1 ?
						RTL_DESC_RING_END : 0));
			priv->rx_dirty = rtl_next_idx(idx, RTL_NUM_RX_DESC);
			continue;
		}
		if (new_page)
			page_pool_recycle_direct(priv->rx_pool, new_page);

		/* copy fallback: recycle the same slot */
		dma_sync_single_for_cpu(priv->dev, buf_dma + off, len,
					DMA_FROM_DEVICE);
		skb = netdev_alloc_skb(ndev, len);
		if (!skb) {
			ndev->stats.rx_dropped++;
			goto rearm;
		}
		skb_reserve(skb, 2);	/* IP header alignment */
		memcpy(skb_put(skb, len),
		       page_address(priv->rx_pages[idx]) + off, len);
		dma_sync_single_for_device(priv->dev, buf_dma + off, len,
					   DMA_FROM_DEVICE);

		ether_addr_copy(&skb->data[0], ndev->dev_addr);
		skb->protocol = eth_type_trans(skb, ndev);
		rtl819x_count_rx(ndev, len);

		napi_gro_receive(napi, skb);
		work_done++;

rearm:
		/*
		 * The same slot goes back to the engine.  The address must
		 * be restored explicitly: the engine rewrites the address
		 * field to the frame start it DMAed to, and handing the
		 * stale value back stops the Rx DMA after a few frames.
		 * No cache sync here: this path is
		 * also the bad-frame exit, where the CPU never touched the
		 * buffer.
		 */
		desc->addr = cpu_to_le32(buf_dma);
		dma_wmb();
		desc->opts1 = cpu_to_le32((RTL_RX_BUF_SIZE <<
					   RTL_RD_SIZE_SHIFT) |
					  RTL_DESC_OWN |
					  (idx == RTL_NUM_RX_DESC - 1 ?
						RTL_DESC_RING_END : 0));
		priv->rx_dirty = rtl_next_idx(idx, RTL_NUM_RX_DESC);
	}

	/* ---- Tx completion ---- */
	/*
	 * The tx ring indices are also reset by tx_timeout(); the lock keeps
	 * the completion walk here from racing the reset.
	 */
	spin_lock(&priv->tx_lock);

	dirty = priv->tx_dirty;
	while (dirty != priv->tx_curr) {
		struct rtl819x_tx_info *info = &priv->tx_info[dirty];
		struct rtl819x_desc *desc = &priv->tx_ring[dirty];

		if (le32_to_cpu(desc->opts1) & RTL_DESC_OWN)
			break;

		if (info->skb) {
			dma_unmap_single(priv->dev,
					 dma_unmap_addr(info, mapping),
					 info->map_len, DMA_TO_DEVICE);
			ndev->stats.tx_packets++;
			ndev->stats.tx_bytes += info->skb->len;
			dev_consume_skb_any(info->skb);
			info->skb = NULL;
		}
		dirty = rtl_next_idx(dirty, RTL_NUM_TX_DESC);
	}
	priv->tx_dirty = dirty;
	/*
	 * Wake the queue once enough slots drained, before the ring is empty
	 * (like mvneta's tx_wake_threshold): waking only on a free slot makes
	 * the queue stop/start oscillate under bursts.
	 */
	if (unlikely(netif_queue_stopped(ndev))) {
		unsigned int in_use = (priv->tx_curr - dirty +
				       RTL_NUM_TX_DESC) % RTL_NUM_TX_DESC;

		if (in_use <= RTL_NUM_TX_DESC - 1 - RTL_TX_WAKE_THRESH)
			netif_wake_queue(ndev);
	}

	spin_unlock(&priv->tx_lock);

	if (work_done < budget) {
		napi_complete_done(napi, work_done);
		/* re-enable interrupts */
		rtl_wr(priv, RTL_CPUIIMR, RTL_IRQ_ALL);
	}

	return work_done;
}

static irqreturn_t rtl819x_interrupt(int irq, void *dev_id)
{
	struct net_device *ndev = dev_id;
	struct rtl819x_priv *priv = netdev_priv(ndev);
	u32 status;

	status = rtl_rr(priv, RTL_CPUIISR) & RTL_IRQ_ALL;
	if (!status)
		return IRQ_NONE;

	rtl_wr(priv, RTL_CPUIISR, status);	/* write 1 to clear */

	if (likely(napi_schedule_prep(&priv->napi))) {
		/* mask interrupts while NAPI polls */
		rtl_wr(priv, RTL_CPUIIMR, 0);
		__napi_schedule(&priv->napi);
	}

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* Tx                                                                  */
/* ------------------------------------------------------------------ */

static netdev_tx_t rtl819x_start_xmit(struct sk_buff *skb,
				      struct net_device *ndev)
{
	struct rtl819x_priv *priv = netdev_priv(ndev);
	struct rtl819x_desc *desc;
	struct rtl819x_tx_info *info;
	dma_addr_t mapping;
	unsigned int curr, next;
	u32 len = skb->len;

	/* one descriptor per packet; not fragmentable */
	if (unlikely(skb->len > RTL_RX_BUF_SIZE - 4))
		goto drop;

	spin_lock(&priv->tx_lock);

	curr = priv->tx_curr;
	next = rtl_next_idx(curr, RTL_NUM_TX_DESC);
	if (unlikely(next == priv->tx_dirty)) {
		netif_stop_queue(ndev);
		spin_unlock(&priv->tx_lock);
		netdev_dbg(ndev, "tx ring full\n");
		return NETDEV_TX_BUSY;
	}

	/*
	 * Vendor pads short packets and adds 4 to the length because the
	 * hardware appends the FCS (rtl819x_swNic.c _New_swNic_send()).
	 */
	if (len < RTL_MIN_TX_LEN)
		len = RTL_MIN_TX_LEN + 4;
	else
		len += 4;

	/*
	 * The descriptor length covers the FCS, which the MAC appends on
	 * its own: zero-fill the extra bytes so the DMA never reads past
	 * the data the skb owns.  skb_pad() frees the skb on failure, so
	 * this path must return without touching the skb again.
	 */
	if (skb_pad(skb, len - skb->len)) {
		/* skb_pad() freed the skb; the ring slot we took stays
		 * free because tx_curr is advanced only below.
		 */
		spin_unlock(&priv->tx_lock);
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	mapping = dma_map_single(priv->dev, skb->data, len, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(priv->dev, mapping)))
		goto drop_unlock;

	info = &priv->tx_info[curr];
	info->skb = skb;
	info->map_len = len;
	dma_unmap_addr_set(info, mapping, mapping);

	desc = &priv->tx_ring[curr];
	desc->addr = cpu_to_le32(mapping);
	/*
	 * Direct Tx mode (HWLOOKUP=0, BRIDGING=0): the packet is sent as-is
	 * to the destination portlist in TD_DP.  No offload fields are set.
	 */
	desc->opts1 = cpu_to_le32(RTL_DESC_FIRST_FRAG | RTL_DESC_LAST_FRAG |
				  (len << RTL_TD_PHLEN_SHIFT));
	desc->opts2 = cpu_to_le32(len << RTL_TD_MLEN_SHIFT);
	desc->opts3 = 0;
	desc->opts4 = cpu_to_le32((priv->tx_portmask & RTL_TD_DP_MASK) <<
				  RTL_TD_DP_SHIFT);
	desc->opts5 = 0;

	/* ownership bit last, after all other fields are visible */
	dma_wmb();
	desc->opts1 |= cpu_to_le32(RTL_DESC_OWN);

	priv->tx_curr = next;

	rtl_kick_tx(priv);

	if (unlikely(next == priv->tx_dirty))
		netif_stop_queue(ndev);

	spin_unlock(&priv->tx_lock);
	return NETDEV_TX_OK;

drop_unlock:
	spin_unlock(&priv->tx_lock);
drop:
	ndev->stats.tx_dropped++;
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

/* ------------------------------------------------------------------ */
/* Open / close                                                        */
/* ------------------------------------------------------------------ */

static int rtl819x_init_rings(struct rtl819x_priv *priv)
{
	unsigned int i;

	priv->rx_ring = dma_alloc_coherent(priv->dev,
					   RTL_NUM_RX_DESC * sizeof(*priv->rx_ring),
					   &priv->rx_ring_dma, GFP_KERNEL);
	priv->tx_ring = dma_alloc_coherent(priv->dev,
					   RTL_NUM_TX_DESC * sizeof(*priv->tx_ring),
					   &priv->tx_ring_dma, GFP_KERNEL);
	priv->tx_info = kcalloc(RTL_NUM_TX_DESC,
				sizeof(*priv->tx_info), GFP_KERNEL);
	if (!priv->rx_ring || !priv->tx_ring || !priv->tx_info)
		return -ENOMEM;

	memset(priv->rx_ring, 0, RTL_NUM_RX_DESC * sizeof(*priv->rx_ring));
	memset(priv->tx_ring, 0, RTL_NUM_TX_DESC * sizeof(*priv->tx_ring));

	for (i = 0; i < RTL_NUM_RX_DESC; i++)
		if (rtl819x_rx_refill(priv, i, GFP_KERNEL))
			/* free_rings() cleans up the slots filled so far */
			return -ENOMEM;

	priv->rx_dirty = 0;
	priv->tx_curr = 0;
	priv->tx_dirty = 0;

	return 0;
}

static void rtl819x_free_rings(struct rtl819x_priv *priv)
{
	unsigned int i;

	for (i = 0; i < RTL_NUM_RX_DESC; i++) {
		if (priv->rx_pages[i]) {
			page_pool_put_full_page(priv->rx_pool,
						priv->rx_pages[i], false);
			priv->rx_pages[i] = NULL;
		}
	}

	for (i = 0; i < RTL_NUM_TX_DESC; i++) {
		struct rtl819x_tx_info *info = &priv->tx_info[i];

		if (info->skb) {
			dma_unmap_single(priv->dev,
					 dma_unmap_addr(info, mapping),
					 info->map_len, DMA_TO_DEVICE);
			dev_kfree_skb_any(info->skb);
			info->skb = NULL;
		}
	}

	if (priv->rx_ring)
		dma_free_coherent(priv->dev,
				  RTL_NUM_RX_DESC * sizeof(*priv->rx_ring),
				  priv->rx_ring, priv->rx_ring_dma);
	if (priv->tx_ring)
		dma_free_coherent(priv->dev,
				  RTL_NUM_TX_DESC * sizeof(*priv->tx_ring),
				  priv->tx_ring, priv->tx_ring_dma);
	kfree(priv->tx_info);
	priv->tx_info = NULL;
	priv->rx_ring = NULL;
	priv->tx_ring = NULL;
}

static int rtl819x_open(struct net_device *ndev)
{
	struct rtl819x_priv *priv = netdev_priv(ndev);
	int ret;

	/*
	 * Stop the CPU-interface DMA before touching the descriptor rings:
	 * the bootloader may have left the descriptors programmed and the
	 * DMA running.  The vendor driver never uses SOFTRST on this SoC
	 * ("re-initialize all descriptors" re-scans the still-registered
	 * ring base and DMAs over random memory); it just gates the
	 * interrupt, acknowledges pending status and clears the run bit.
	 */
	rtl_wr(priv, RTL_CPUIIMR, 0);
	rtl_wr(priv, RTL_CPUIISR, rtl_rr(priv, RTL_CPUIISR));
	rtl_wr(priv, RTL_CPUICR, 0);

	ret = rtl819x_init_rings(priv);
	if (ret)
		goto err_free;

	ret = request_irq(priv->irq, rtl819x_interrupt, 0,
			  RTL819X_DRV_NAME, ndev);
	if (ret)
		goto err_free;

	napi_enable(&priv->napi);
	netif_start_queue(ndev);

	/*
	 * The bootloader (its tftp) left the NIC engine running with
	 * classic 865xC pkthdr/mbuf rings.  CPUICR1 is only writable while
	 * TXCMD/RXCMD are off (vendor SDK probe clears them first), so stop
	 * the engine before touching ring descriptors or CPUICR1.
	 */
	rtl_wr(priv, RTL_CPUICR, 0);
	rtl_wr(priv, RTL_CPUIIMR, 0);
	rtl_wr(priv, RTL_CPUIISR, rtl_rr(priv, RTL_CPUIISR));

	/* descriptor ring base address (rings 1-5 stay off our radar) */
	rtl_wr(priv, RTL_CPURPDCR0, priv->rx_ring_dma);
	rtl_wr(priv, RTL_CPUTPDCR0, priv->tx_ring_dma);

	/*
	 * Vendor sets DMA_CR1 to (ring0 count - 1) * desc size and enables
	 * TX_RING0_TAIL_AWARE in DMA_CR4 so the DMA wraps ring 0 without a
	 * per-descriptor tail bit.
	 */
	rtl_wr(priv, RTL_DMA_CR1,
	       (RTL_NUM_TX_DESC - 1) * sizeof(*priv->tx_ring));
	rtl_wr(priv, RTL_DMA_CR4, BIT(0));	/* TX_RING0_TAIL_AWARE */

	/*
	 * CPUICR1: select the 6-word descriptor format (see the
	 * RTL_CPUICR1_PKTHDR comment).
	 */
	rtl_wr(priv, RTL_CPUICR1,
	       (rtl_rr(priv, RTL_CPUICR1) & ~RTL_CPUICR1_PKTHDR) |
	       RTL_CPUICR1_PKTHDR_SHORTCUT);

	/* clear any stale interrupt status, then start DMA */
	rtl_wr(priv, RTL_CPUIISR, rtl_rr(priv, RTL_CPUIISR));
	rtl_wr(priv, RTL_CPUICR,
	       RTL_TXCMD | RTL_RXCMD | RTL_BUSBURST_128W | RTL_MBUF_2048B);

	/*
	 * DMA_CR0 FIFO watermarks: the vendor values and the reason for
	 * the read-modify-write are at the RTL_DMA_CR0_* defines.  This
	 * must happen after the last CPUICR write: updating the burst
	 * size field resets HiFifoMark to 0x57.
	 */
	rtl_wr(priv, RTL_DMA_CR0,
	       (rtl_rr(priv, RTL_DMA_CR0) &
		~(RTL_DMA_CR0_LOFIFO | RTL_DMA_CR0_HIFIFO)) |
	       FIELD_PREP(RTL_DMA_CR0_LOFIFO, RTL_DMA_CR0_LOFIFO_MARK) |
	       FIELD_PREP(RTL_DMA_CR0_HIFIFO, RTL_DMA_CR0_HIFIFO_MARK));

	phylink_start(priv->phylink);

	rtl_wr(priv, RTL_CPUIIMR, RTL_IRQ_ALL);

	return 0;

err_free:
	netdev_err(ndev, "open failed with %d\n", ret);
	rtl819x_free_rings(priv);
	return ret;
}

static int rtl819x_stop(struct net_device *ndev)
{
	struct rtl819x_priv *priv = netdev_priv(ndev);

	netif_stop_queue(ndev);
	napi_disable(&priv->napi);
	phylink_stop(priv->phylink);

	rtl_wr(priv, RTL_CPUIIMR, 0);
	free_irq(priv->irq, ndev);

	/* vendor rtl865x_down() */
	rtl_wr(priv, RTL_CPUIISR, rtl_rr(priv, RTL_CPUIISR));
	rtl_wr(priv, RTL_CPUICR, 0);

	rtl819x_free_rings(priv);

	return 0;
}

/* ------------------------------------------------------------------ */
/* phylink / ethtool                                                   */
/* ------------------------------------------------------------------ */

/*
 * The NIC register window has no link/speed controls of its own: the CPU
 * port runs point-to-point between this DMA engine and the switch's CPU
 * port, whose far end (and RGMII delay configuration) is owned by the
 * switch side of the board topology.  mac_config/mac_link_up/down
 * therefore have nothing to program -- phylink is attached so the link
 * state reported by ethtool and the netdev carrier reflect the fixed
 * link (or PHY) described in the device tree.
 */
static void rtl819x_mac_config(struct phylink_config *config,
			       unsigned int mode,
			       const struct phylink_link_state *state)
{
}

static void rtl819x_mac_link_down(struct phylink_config *config,
				  unsigned int mode,
				  phy_interface_t interface)
{
}

static void rtl819x_mac_link_up(struct phylink_config *config,
				struct phy_device *phy,
				unsigned int mode,
				phy_interface_t interface,
				int speed, int duplex,
				bool tx_pause, bool rx_pause)
{
}

static const struct phylink_mac_ops rtl819x_phylink_mac_ops = {
	.mac_config		= rtl819x_mac_config,
	.mac_link_down		= rtl819x_mac_link_down,
	.mac_link_up		= rtl819x_mac_link_up,
};

static void rtl819x_get_drvinfo(struct net_device *ndev,
				struct ethtool_drvinfo *info)
{
	strscpy(info->driver, RTL819X_DRV_NAME, sizeof(info->driver));
	strscpy(info->bus_info, dev_name(ndev->dev.parent),
		sizeof(info->bus_info));
}

static int rtl819x_get_link_ksettings(struct net_device *ndev,
				      struct ethtool_link_ksettings *cmd)
{
	struct rtl819x_priv *priv = netdev_priv(ndev);

	return phylink_ethtool_ksettings_get(priv->phylink, cmd);
}

static int rtl819x_set_link_ksettings(struct net_device *ndev,
				      const struct ethtool_link_ksettings *cmd)
{
	struct rtl819x_priv *priv = netdev_priv(ndev);

	return phylink_ethtool_ksettings_set(priv->phylink, cmd);
}

static const struct ethtool_ops rtl819x_ethtool_ops = {
	.get_drvinfo		= rtl819x_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_link_ksettings	= rtl819x_get_link_ksettings,
	.set_link_ksettings	= rtl819x_set_link_ksettings,
};

/* ------------------------------------------------------------------ */
/* Probe                                                               */
/* ------------------------------------------------------------------ */

/*
 * The CPU interface stops delivering completions if a frame is lost on
 * the way; recover by resetting the descriptor rings in place, like the
 * vendor rtl865x_down()/rtl865x_start() pair.
 */
static void rtl819x_eth_tx_timeout(struct net_device *ndev, unsigned int txqueue)
{
	struct rtl819x_priv *priv = netdev_priv(ndev);
	unsigned int i;

	netdev_warn(ndev,
		    "tx timeout (queue %u): CPUICR=%08x CPUICR1=%08x IMR=%08x ISR=%08x tx_curr=%u tx_dirty=%d rx_dirty=%u\n",
		    txqueue, rtl_rr(priv, RTL_CPUICR), rtl_rr(priv, RTL_CPUICR1),
		    rtl_rr(priv, RTL_CPUIIMR), rtl_rr(priv, RTL_CPUIISR),
		    priv->tx_curr, priv->tx_dirty, priv->rx_dirty);

	spin_lock(&priv->tx_lock);

	/* stop the engine, ack pending status */
	rtl_wr(priv, RTL_CPUIIMR, 0);
	rtl_wr(priv, RTL_CPUICR, 0);
	rtl_wr(priv, RTL_CPUIISR, rtl_rr(priv, RTL_CPUIISR));

	/* re-init the descriptor rings in place (no allocation in atomic) */
	priv->tx_curr = 0;
	priv->tx_dirty = 0;
	for (i = 0; i < RTL_NUM_TX_DESC; i++) {
		if (priv->tx_info[i].skb) {
			dma_unmap_single(priv->dev,
					 dma_unmap_addr(&priv->tx_info[i], mapping),
					 priv->tx_info[i].map_len,
					 DMA_TO_DEVICE);
			dev_kfree_skb_any(priv->tx_info[i].skb);
			priv->tx_info[i].skb = NULL;
		}
		priv->tx_ring[i].opts1 = 0;
	}
	for (i = 0; i < RTL_NUM_RX_DESC; i++) {
		/*
		 * Slot pages already delivered to the stack were swapped out
		 * at poll time, so rx_pages[i] is always the page that
		 * belongs to this descriptor.  Re-arm from it; no page is
		 * shared with an outstanding skb here.
		 */
		priv->rx_ring[i].addr =
			cpu_to_le32(page_pool_get_dma_addr(priv->rx_pages[i]));
		priv->rx_ring[i].opts1 =
			cpu_to_le32((RTL_RX_BUF_SIZE << RTL_RD_SIZE_SHIFT) |
				    RTL_DESC_OWN |
				    (i == RTL_NUM_RX_DESC - 1 ?
					RTL_DESC_RING_END : 0));
	}
	priv->rx_dirty = 0;
	dma_wmb();

	/* reprogram the engine, then the vendor rtl865x_start() tail */
	rtl_wr(priv, RTL_CPURPDCR0, priv->rx_ring_dma);
	rtl_wr(priv, RTL_CPUTPDCR0, priv->tx_ring_dma);
	rtl_wr(priv, RTL_DMA_CR1,
	       (RTL_NUM_TX_DESC - 1) * sizeof(*priv->tx_ring));
	rtl_wr(priv, RTL_DMA_CR4, BIT(0));
	rtl_wr(priv, RTL_CPUICR1,
	       (rtl_rr(priv, RTL_CPUICR1) & ~RTL_CPUICR1_PKTHDR) |
	       RTL_CPUICR1_PKTHDR_SHORTCUT);
	rtl_wr(priv, RTL_CPUIISR, rtl_rr(priv, RTL_CPUIISR));
	rtl_wr(priv, RTL_CPUICR,
	       RTL_TXCMD | RTL_RXCMD | RTL_BUSBURST_128W | RTL_MBUF_2048B);
	/*
	 * DMA_CR0 FIFO watermarks must be programmed after the last CPUICR
	 * write: updating the burst size field resets HiFifoMark to 0x57.
	 */
	rtl_wr(priv, RTL_DMA_CR0,
	       (rtl_rr(priv, RTL_DMA_CR0) &
		~(RTL_DMA_CR0_LOFIFO | RTL_DMA_CR0_HIFIFO)) |
	       FIELD_PREP(RTL_DMA_CR0_LOFIFO, RTL_DMA_CR0_LOFIFO_MARK) |
	       FIELD_PREP(RTL_DMA_CR0_HIFIFO, RTL_DMA_CR0_HIFIFO_MARK));

	netif_wake_queue(ndev);
	rtl_wr(priv, RTL_CPUIIMR, RTL_IRQ_ALL);

	spin_unlock(&priv->tx_lock);
}

static const struct net_device_ops rtl819x_netdev_ops = {
	.ndo_open		= rtl819x_open,
	.ndo_stop		= rtl819x_stop,
	.ndo_start_xmit		= rtl819x_start_xmit,
	.ndo_get_stats64	= dev_get_tstats64,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_tx_timeout		= rtl819x_eth_tx_timeout,
	.ndo_validate_addr	= eth_validate_addr,
};

static int rtl819x_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	struct rtl819x_priv *priv;
	u8 addr[ETH_ALEN];
	int ret;

	ndev = devm_alloc_etherdev(&pdev->dev, sizeof(*priv));
	if (!ndev)
		return -ENOMEM;

	priv = netdev_priv(ndev);
	priv->ndev = ndev;
	priv->dev = &pdev->dev;
	spin_lock_init(&priv->tx_lock);
	SET_NETDEV_DEV(ndev, &pdev->dev);
	platform_set_drvdata(pdev, ndev);

	priv->base = devm_platform_ioremap_resource_byname(pdev, "nic");
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	/*
	 * The bootloader hands over with the CPU-interface DMA still running
	 * against its own descriptor rings.  Until open() reprograms them,
	 * every received frame is DMA'd into memory the kernel has likely
	 * reallocated by then, randomly corrupting data.  Gate the DMA and
	 * the interrupt as early as possible; open() reprograms and restarts.
	 */
	rtl_wr(priv, RTL_CPUIIMR, 0);
	rtl_wr(priv, RTL_CPUIISR, rtl_rr(priv, RTL_CPUIISR));
	rtl_wr(priv, RTL_CPUICR, 0);

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0)
		return priv->irq;

	ret = of_property_read_u32(pdev->dev.of_node, "realtek,tx-portmask",
				   &priv->tx_portmask);
	if (ret)
		priv->tx_portmask = 0x3f;	/* all switch ports */

	ret = of_get_mac_address(pdev->dev.of_node, addr);
	if (ret == -EPROBE_DEFER)
		return ret;
	if (ret == -ENODATA)
		ret = eth_platform_get_mac_address(&pdev->dev, addr);
	if (ret)
		eth_hw_addr_random(ndev);
	else
		eth_hw_addr_set(ndev, addr);

	ndev->netdev_ops = &rtl819x_netdev_ops;
	ndev->ethtool_ops = &rtl819x_ethtool_ops;

	/* No hardware offloads: the switch checksum/NAPT path is unused. */
	ndev->hw_features = 0;
	ndev->features |= NETIF_F_GRO;
	ndev->needed_headroom = 4;	/* vendor leaves room for switch tags */
	ndev->watchdog_timeo = 3 * HZ;

	/*
	 * RX slots are 2048-byte aligned pages and the engine's MBUF field
	 * caps the per-frame buffer at 2048 bytes: an ingress frame
	 * (header, tags and FCS included) must fit in one buffer, which
	 * bounds the MTU.
	 */
	ndev->max_mtu = RTL_RX_BUF_SIZE - ETH_HLEN - VLAN_HLEN - 4;

	/*
	 * The MAC talks RGMII to the switch CPU port; the delays are a
	 * property of the board wiring and are described in DT (plain
	 * "rgmii" or one of the rgmii-*-id variants).
	 */
	priv->phylink_config.dev = &ndev->dev;
	priv->phylink_config.type = PHYLINK_NETDEV;
	priv->phylink_config.mac_capabilities = MAC_ASYM_PAUSE |
						MAC_SYM_PAUSE |
						MAC_10 | MAC_100 |
						MAC_1000FD;
	phy_interface_set_rgmii(priv->phylink_config.supported_interfaces);

	ndev->tstats = netdev_alloc_pcpu_stats(struct pcpu_sw_netstats);
	if (!ndev->tstats)
		return -ENOMEM;

	netif_napi_add(ndev, &priv->napi, rtl819x_poll);

	/*
	 * Created after netif_napi_add() so the pool can bind the NAPI
	 * context (per-NAPI alloc cache).  The pool needs more pages than
	 * the ring: completed slots travel through in-flight skbs before
	 * recycling.
	 */
	priv->rx_pool = page_pool_create(&(struct page_pool_params) {
		.order		= 0,
		.flags		= PP_FLAG_DMA_MAP | PP_FLAG_DMA_SYNC_DEV,
		.pool_size	= RTL_NUM_RX_DESC * 2,
		.napi		= &priv->napi,
		.nid		= dev_to_node(&pdev->dev),
		.dev		= &pdev->dev,
		.dma_dir	= DMA_FROM_DEVICE,
		.max_len	= RTL_RX_BUF_SIZE,
	});
	if (IS_ERR(priv->rx_pool)) {
		ret = PTR_ERR(priv->rx_pool);
		priv->rx_pool = NULL;
		goto err_napi;
	}

	priv->phylink = phylink_create(&priv->phylink_config, pdev->dev.fwnode,
				       PHY_INTERFACE_MODE_NA,
				       &rtl819x_phylink_mac_ops);
	if (IS_ERR(priv->phylink)) {
		ret = PTR_ERR(priv->phylink);
		goto err_pool;
	}

	/*
	 * A fixed-link node or a phy-handle below the ethernet node is
	 * mandatory: without one there is nothing that reports the
	 * CPU-port link state.
	 */
	ret = phylink_of_phy_connect(priv->phylink, pdev->dev.of_node, 0);
	if (ret)
		goto err_phylink;

	ret = register_netdev(ndev);
	if (ret)
		goto err_phylink;

	netdev_info(ndev, "RTL8197F switch CPU port: %pM, irq %d, tx-portmask 0x%02x\n",
		    ndev->dev_addr, priv->irq, priv->tx_portmask);

	return 0;

err_phylink:
	phylink_disconnect_phy(priv->phylink);
	phylink_destroy(priv->phylink);
err_pool:
	page_pool_destroy(priv->rx_pool);
err_napi:
	netif_napi_del(&priv->napi);
	free_percpu(ndev->tstats);
	return ret;
}

static void rtl819x_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct rtl819x_priv *priv = netdev_priv(ndev);

	unregister_netdev(ndev);
	phylink_disconnect_phy(priv->phylink);
	phylink_destroy(priv->phylink);
	netif_napi_del(&priv->napi);
	page_pool_destroy(priv->rx_pool);
	free_percpu(ndev->tstats);
}

static const struct of_device_id rtl819x_of_match[] = {
	{ .compatible = "realtek,rtl8197f-eth" },
	{ }
};
MODULE_DEVICE_TABLE(of, rtl819x_of_match);

static struct platform_driver rtl819x_driver = {
	.probe	= rtl819x_probe,
	.remove	= rtl819x_remove,
	.driver	= {
		.name		= RTL819X_DRV_NAME,
		.of_match_table	= rtl819x_of_match,
	},
};
module_platform_driver(rtl819x_driver);

MODULE_AUTHOR("David Yang <mmyangfl@gmail.com>");
MODULE_DESCRIPTION("Realtek RTL8197F Ethernet MAC / switch CPU interface driver");
MODULE_LICENSE("GPL");
