// SPDX-License-Identifier: GPL-2.0-only
/*
 * RTL8192F-SoC descriptor rings, TX and RX reclaim.
 *
 * Data path of the RTL8197F WiFi core: the 88XX "new BD" dual-ring
 * model, not the rtlwifi PCI in-descriptor OWN model.
 *
 * TX: every frame is described by one 32-byte TXBD (4 x 8-byte
 * elements) plus a 40-byte TX info descriptor skb_push()ed into the
 * frame buffer, laid out in the vendor SetTxBufferDesc88XX() 3-segment
 * shape: TXBD element 0 carries the 40B desc, element 1 the 802.11
 * header, element 2 the payload, element 3 stays zero (no ICV/MIC).
 * The queue is kicked by writing the host producer index into the low 16
 * bits of the queue's TXBD_IDX register and reaped from the HW
 * consumer index in the high 16 bits - there is no OWN bit on the data
 * path.
 *
 * RX: 8-byte RXBDs point at buffers whose first 24 bytes are the RX
 * info descriptor, followed by the appended PHY status (RCR
 * APP_PHYST_RXFF) and the MPDU.  There is no OWN bit: the 8197F runs
 * the RXBD ready-check in its power-on "RXTAG" mode (REG_HCI_CTRL
 * 0x300 bit15 = 1, Hal88XXGen.c 5404-5425 / HalCfg.h
 * RXBD_READY_CHECK_METHOD), where the hardware stamps every RXBD it
 * fills with a rolling 13-bit tag in TOTALRXPKTSIZE.  The driver keeps
 * the expected next tag and treats a slot as ready when the field
 * matches it (or reads the 0xFFFF park value, the vendor "dirty fix"),
 * Hal88XXRxDesc.c 482-542.
 *
 * Vendor reference: RTL8197_3411D_2/rtl819x/linux-3.10/.../WlanHAL/
 * HalMac88XX/{halmac_tx_bd_nic.h,halmac_rx_bd_nic.h,halmac_reg2.h},
 * RTL88XX/Hal88XXTxDesc.c, RTL88XX/Hal88XXRxDesc.c,
 * RTL88XX/Hal88XXGen.c (ring startup 5748-5793).
 * Mainline reference: drivers/net/wireless/realtek/rtw88/{pci.c,tx.h}.
 */

#include "rtl8192f_soc.h"

/*
 * Interrupt mask bits (RTL8192F_HIMR_*) live in rtl8192f_soc.h so
 * core.c's irq handler and this file share one definition.  The group
 * 0/1 offsets (0xB0/0xB4, 0xB8/0xBC) and the bit assignments are the
 * 88XX ones, confirmed on this core by the interrupts that fire and
 * are cleared there.
 */

/* ------------------------------------------------------------------ */
/* Queue register / qsel tables                                        */
/* ------------------------------------------------------------------ */

static const struct rtl8192f_queue_regs {
	u16 desa;	/* TXBD ring base (DESA) register */
	u16 num;	/* ring length register */
	u16 idx;	/* producer/consumer index register */
} queue_regs_tbl[Q_NUM] = {
	[Q_MGNT] = { REG_MGQ_TXBD_DESA, REG_MGQ_TXBD_NUM, REG_MGQ_TXBD_IDX },
	[Q_BK]   = { REG_BKQ_TXBD_DESA, REG_BKQ_TXBD_NUM, REG_BKQ_TXBD_IDX },
	[Q_BE]   = { REG_BEQ_TXBD_DESA, REG_BEQ_TXBD_NUM, REG_BEQ_TXBD_IDX },
	[Q_VI]   = { REG_VIQ_TXBD_DESA, REG_VIQ_TXBD_NUM, REG_VIQ_TXBD_IDX },
	[Q_VO]   = { REG_VOQ_TXBD_DESA, REG_VOQ_TXBD_NUM, REG_VOQ_TXBD_IDX },
	[Q_HIGH] = { REG_HI0Q_TXBD_DESA, REG_HI0Q_TXBD_NUM, REG_HI0Q_TXBD_IDX },
	/* Q_BEACON: no DESA table entry here, the BCNQ base lives in
	 * REG_BCNQ_TXBD_DESA and the slot is not host-advanced.
	 */
	[Q_BEACON] = { 0, 0, 0 },
};

static const char *queue_name_tbl[Q_NUM] = {
	"MGNT", "BK", "BE", "VI", "VO", "HIGH", "BCN",
};

static void rtl8192f_rx_tasklet(struct tasklet_struct *t);

/* mac80211 AC (skb queue mapping) -> driver queue.  Anything outside
 * the four ACs (cab frames and other special queues when mac80211
 * numbers them >= IEEE80211_NUM_ACS) is group/special traffic and goes
 * to the HIGH ring, matching the multicast path in rtl8192f_tx()
 * .
 */
static u8 ac_to_queue(u16 ac)
{
	switch (ac) {
	case IEEE80211_AC_VO:
		return Q_VO;
	case IEEE80211_AC_VI:
		return Q_VI;
	case IEEE80211_AC_BK:
		return Q_BK;
	case IEEE80211_AC_BE:
		return Q_BE;
	default:
		return Q_HIGH;
	}
}

/* driver queue -> mac80211 AC (only valid for the five serviced rings;
 * the HIGH ring maps to BE, which is also the queue rtl8192f_tx()
 * stops when the HIGH ring fills).
 */
static u16 queue_to_ac(u8 qidx)
{
	switch (qidx) {
	case Q_VO:
		return IEEE80211_AC_VO;
	case Q_VI:
		return IEEE80211_AC_VI;
	case Q_BK:
		return IEEE80211_AC_BK;
	default:
		return IEEE80211_AC_BE;
	}
}

/* ------------------------------------------------------------------ */
/* TX info descriptor (40B) bit fields                                 */
/*                                                                     */
/* Same layout as mainline rtw88 tx.h 24-49 struct rtw_tx_desc w0-w5;  */
/* vendor HalHeader/HalComTXDesc.h TX_DW*.                             */
/* ------------------------------------------------------------------ */

#define TXD_W0_TXPKTSIZE		GENMASK(15, 0)
#define TXD_W0_OFFSET			GENMASK(23, 16)
#define TXD_W0_BMC			BIT(24)
/* DW0 bit26 is vendor TX_DW0_LS only on 8814A (HalComTXDesc.h 66-94);
 * on every other generation including the 8197F it is RSVD_26 and the
 * vendor never sets it (live mgmt DW0 = 0x00280159, bit26 = 0).
 */
#define TXD_W0_DISQSELSEQ		BIT(31)
#define TXD_W1_MACID			GENMASK(7, 0)
#define TXD_W1_QSEL			GENMASK(12, 8)
#define TXD_W1_RATE_ID			GENMASK(20, 16)
#define TXD_W1_PKT_OFFSET		GENMASK(28, 24)
/* TX_DW1 [23:22] names the algorithm the engine must apply (HalComTXDesc.h
 * 117/132 TX_DW1_SECTYPE, values 522-525).  There is no key index in the
 * descriptor: the engine finds the CAM entry from the frame's address and
 * key id, which is why set_key() stores the peer address with the key.
 */
#define TXD_W1_SEC_TYPE			GENMASK(23, 22)
#define  TXD_SEC_TYPE_AES		3
#define TXD_W2_G_ID			GENMASK(29, 24)
#define TXD_W2_AGG_EN			BIT(12)	/* vendor TX_DW2_AGG_EN_SH */
#define TXD_W2_AMPDU_DENSITY		GENMASK(22, 20)
#define TXD_W3_MAX_AGG_NUM		GENMASK(21, 17)
#define TXD_W3_USE_RATE			BIT(8)
#define TXD_W3_DISRTSFB			BIT(9)
#define TXD_W3_DISDATAFB		BIT(10)	/* no data fallback (beacons) */
#define TXD_W3_CTS2SELF			BIT(11)
#define TXD_W3_USE_RTS			BIT(12)
#define TXD_W4_DATARATE			GENMASK(6, 0)
#define TXD_W7_TXBUFF			GENMASK(15, 0)
#define TXD_W4_DATARATE_FB_LIMIT	GENMASK(12, 8)
#define TXD_W4_RTY_LMT_EN		BIT(17)	/* vendor TX_DW4_RTY_LMT_EN_SH */
#define TXD_W4_DATA_RT_LMT		GENMASK(23, 18)
#define TXD_W9_SW_SEQ			GENMASK(23, 12)
#define TXD_W4_RTSRATE			GENMASK(28, 24)
#define TXD_W5_DATA_SC			GENMASK(3, 0)	/* vendor TX_DW5_DATA_SC_SH 0 */
#define TXD_W5_DATA_SHORT		BIT(4)
#define TXD_W5_DATA_BW			GENMASK(6, 5)
#define TXD_W5_RTS_SC			GENMASK(16, 13)	/* vendor TX_DW5_RTS_SC_SH 13 */

/* W5 DATA_SC codes name the half the frame does *not* use as its center:
 * 1 with the extension channel above the primary, 2 below it, 3 when the
 * frame occupies both halves (vendor HalComTXDesc.h 533-534
 * TXDESC_DATASC_UPPER/LOWER, applied at 8192cd_tx.c 3435-3452 and
 * 3608-3620).  The encoding is the mirror image of the REG_DATA_SC value
 * hw.c programs, where below is 1.
 */
#define TXD_DATA_SC_ABOVE		1
#define TXD_DATA_SC_BELOW		2
#define TXD_DATA_SC_BOTH		3
#define TXD_DATA_BW_40			1	/* vendor HT_CHANNEL_WIDTH_20_40 */

/* RX info descriptor (24B) DW0/DW3 bit fields, 88XX/8197F layout
 * (halmac_rx_desc_nic.h 20-40/80: CRC32 DW0 bit14, ICVERR DW0 bit15).
 */
#define RXD_W0_PKT_LEN			GENMASK(13, 0)
#define RXD_W0_CRC32_ERR		BIT(14)
#define RXD_W0_ICV_ERR			BIT(15)
/* RX_DW0 [22:20] names the algorithm the engine applied and bit27 says the
 * frame was left to software (RX_DW0_SECURITY_SH / RX_DW0_SWDEC_SH,
 * HalComRXDesc.h 74/78).  Together they are how rtw88 rx.c 286 decides a
 * frame arrived decrypted.
 */
#define RXD_W0_SECURITY			GENMASK(22, 20)
#define RXD_W0_SWDEC			BIT(27)
#define RXD_W0_DRV_INFO_SIZE		GENMASK(19, 16)	/* in 8B units */
#define RXD_W0_SHIFT			GENMASK(25, 24)
/* DW3: RX_RATE is 7 bits (0-11 legacy linear, 12+ = MCS0+); there are
 * no SGI/BW fields in the 8197F RX info descriptor - the old W3
 * bits 8/9 reads hit BSSID_FIT/HTC garbage.  Bandwidth comes from the
 * PHY status instead (rate.c); short GI has no report field at all.
 */
/* DW2 bit28 marks a firmware-to-host C2H message (HalComRXDesc.h
 * 162/173, mask 0x1 shift 28); the vendor rx path branches on it in
 * 8192cd_rx.c 6877-6888 and never hands those bytes to the stack.
 */
#define RXD_W2_C2H_PKT			BIT(28)
#define RXD_W3_RX_RATE			GENMASK(6, 0)

/* Beacon TXBD element 0 DW0[31:16]: PSLEN in bits[30:16] (the low 15
 * bits of the psb_len halfword) and BCN_OWN in bit31 (bit15 of the
 * halfword), vendor HalHeader/HalComTXDesc.h TXBD_DW0_BCN_PSLEN /
 * TXBD_DW0_BCN_OWN (sh 16 / 31).
 */
#define TXBD_BCN_PSLEN			GENMASK(14, 0)
#define TXBD_BCN_OWN			BIT(15)

#define REG_FWHW_TXQ_CTRL		0x0420
/* EN_BCNQ_DL, vendor HalComBit.h BIT_En_bcnq_dl / 8192c_reg.h
 * EN_BCNQ_DL: "1: ptcl consider there is bcn pkt in txbuf;
 * 0: mask bcn pkt num in bcnq".  Cleared, the hardware never downloads
 * the beacon queue descriptor, so a beacon is never transmitted even
 * though the TXBD is armed.
 */
#define BIT_EN_BCNQ_DL			BIT(22)

/* ------------------------------------------------------------------ */
/* alloc / free                                                        */
/* ------------------------------------------------------------------ */

static struct rtl8192f_txbd *txbd_of(struct rtl8192f_tx_ring *ring, u32 idx)
{
	return ring->bd + idx;
}

static struct rtl8192f_rxbd *rxbd_of(struct rtl8192f_rx_ring *ring, u32 idx)
{
	struct rtl8192f_rxbd *bd = ring->bd;

	return bd + idx;
}

/* Fill the RX buffers: one skb per RXBD, whole buffer mapped once and
 * kept mapped for the slot's lifetime (refills only re-arm the BD).
 * The 24-byte RX info descriptor lands at the buffer start, so the
 * payload area starts right behind it - no gen-1 style skb_reserve()
 * shift .
 */
static int rtl8192f_rx_fill(struct rtl8192f_soc_priv *priv)
{
	struct rtl8192f_rx_ring *ring = &priv->rx_ring;
	struct device *dev = &priv->pdev->dev;
	u32 i;

	for (i = 0; i < RTL8192F_NUM_RX_DESC; i++) {
		struct rtl8192f_rx_desc_info *info = &ring->info[i];
		struct rtl8192f_rxbd *bd = rxbd_of(ring, i);
		struct sk_buff *skb;
		dma_addr_t dma;

		skb = dev_alloc_skb(RTL8192F_RX_BUF_SIZE + 32);
		if (!skb)
			return -ENOMEM;

		dma = dma_map_single(dev, skb->data, RTL8192F_RX_BUF_SIZE,
				     DMA_FROM_DEVICE);
		if (dma_mapping_error(dev, dma)) {
			dev_err(dev, "rx_fill dma map %u failed\n", i);
			dev_kfree_skb(skb);
			return -ENOMEM;
		}

		info->skb = skb;
		info->dma = dma;

		/* Park the 0xFFFF tag in TOTALRXPKTSIZE so a not-yet-
		 * DMA'd entry can never be mistaken for a zero-length
		 * packet (vendor tag method, Hal88XXRxDesc.c 421-429).
		 */
		memset(bd, 0, sizeof(*bd));
		bd->buf_size = cpu_to_le16(RTL8192F_RX_BUF_SIZE);
		bd->total_pkt_size = cpu_to_le16(0xffff);
		bd->dma = cpu_to_le32(dma);
	}
	return 0;
}

static int rtl8192f_bcn_alloc(struct rtl8192f_soc_priv *priv)
{
	struct device *dev = &priv->pdev->dev;
	struct rtl8192f_bcn_slot *bcn = &priv->bcn;
	struct rtl8192f_txbd *bd;

	/* One 128B-stride slot area so the hardware's fixed beacon
	 * slot offset (HalCfg.h TXBD_BEACON_OFFSET_V1 = 128 on the
	 * 8197F) lands on slot 0; additional MBSSID slots would follow
	 * at +128B.
	 */
	bcn->bd = dmam_alloc_coherent(dev, RTL8192F_BCN_SLOT_STRIDE,
				      &bcn->bd_dma, GFP_KERNEL);
	if (!bcn->bd) {
		dev_err(dev, "bcn bd alloc failed\n");
		return -ENOMEM;
	}

	bcn->txdesc = dmam_alloc_coherent(dev, RTL8192F_TX_INFO_DESC_SIZE,
					  &bcn->txdesc_dma, GFP_KERNEL);
	if (!bcn->txdesc) {
		dev_err(dev, "bcn txdesc alloc failed\n");
		return -ENOMEM;
	}

	/* Stable payload buffer, mapped for the whole lifetime: the hardware
	 * re-reads this address at every TBTT, so renewing the beacon is a
	 * memcpy plus one PSLEN/OWN write instead of re-pointing the slot at
	 * a new skb each period.
	 */
	bcn->frame = dmam_alloc_coherent(dev, RTL8192F_BEACON_MAX_LEN,
					 &bcn->frame_dma, GFP_KERNEL);
	if (!bcn->frame) {
		dev_err(dev, "bcn frame alloc failed\n");
		return -ENOMEM;
	}
	bcn->frame_len = 0;

	memset(bcn->bd, 0, RTL8192F_BCN_SLOT_STRIDE);

	/* Element 0 permanently describes the 40B TX info descriptor;
	 * only PSLEN and BCN_OWN (DW0 bits 30:16 / 31) change at
	 * upload time (vendor FillBeaconDesc88XX / SetBeaconDownload88XX,
	 * Hal88XXTxDesc.c 1909+ / 2248+).
	 */
	bd = bcn->bd;
	bd->ele[0].buf_size = cpu_to_le16(RTL8192F_TX_INFO_DESC_SIZE);
	bd->ele[0].dma = cpu_to_le32(bcn->txdesc_dma);

	return 0;
}

int rtl8192f_txrx_alloc(struct rtl8192f_soc_priv *priv)
{
	struct device *dev = &priv->pdev->dev;
	int ret;
	u32 q;

	BUILD_BUG_ON(sizeof(struct rtl8192f_txbd) != RTL8192F_TXBD_SIZE);
	BUILD_BUG_ON(sizeof(struct rtl8192f_rxbd) != RTL8192F_RXBD_SIZE);

	/* Set up first: every error path below runs rtl8192f_txrx_free(),
	 * whose tasklet_kill() needs the completion tasklet_setup()
	 * initialises.
	 */
	tasklet_setup(&priv->rx_tasklet, rtl8192f_rx_tasklet);

	for (q = 0; q < Q_NUM; q++) {
		struct rtl8192f_tx_ring *ring = &priv->tx_ring[q];

		ring->head = 0;
		ring->tail = 0;
		ring->name = queue_name_tbl[q];

		if (q == Q_BEACON)
			continue;	/* single slot, allocated below */

		switch (q) {
		case Q_MGNT:
			ring->qsel = RTL8192F_QSEL_MGMT;
			break;
		case Q_HIGH:
			ring->qsel = RTL8192F_QSEL_HIGH;
			break;
		default:
			ring->qsel = 0;	/* per-frame TID */
			break;
		}

		ring->bd = dmam_alloc_coherent(dev,
					       RTL8192F_NUM_TX_DESC * RTL8192F_TXBD_SIZE,
					       &ring->bd_dma, GFP_KERNEL);
		if (!ring->bd) {
			ret = -ENOMEM;
			goto err_free;
		}
		memset(ring->bd, 0,
		       RTL8192F_NUM_TX_DESC * RTL8192F_TXBD_SIZE);

		ring->info = kcalloc(RTL8192F_NUM_TX_DESC,
				     sizeof(*ring->info), GFP_KERNEL);
		if (!ring->info) {
			ret = -ENOMEM;
			goto err_free;
		}
	}

	priv->rx_ring.wp = 0;
	priv->rx_ring.rp = 0;
	priv->rx_ring.rxtag = 0;

	priv->rx_ring.bd = dmam_alloc_coherent(dev,
					       RTL8192F_NUM_RX_DESC * RTL8192F_RXBD_SIZE,
					       &priv->rx_ring.bd_dma, GFP_KERNEL);
	if (!priv->rx_ring.bd) {
		ret = -ENOMEM;
		goto err_free;
	}

	priv->rx_ring.info = kcalloc(RTL8192F_NUM_RX_DESC,
				     sizeof(*priv->rx_ring.info), GFP_KERNEL);
	if (!priv->rx_ring.info) {
		ret = -ENOMEM;
		goto err_free;
	}

	ret = rtl8192f_rx_fill(priv);
	if (ret)
		goto err_free;

	ret = rtl8192f_bcn_alloc(priv);
	if (ret)
		goto err_free;

	return 0;

err_free:
	rtl8192f_txrx_free(priv);
	return ret;
}

void rtl8192f_txrx_free(struct rtl8192f_soc_priv *priv)
{
	struct device *dev = &priv->pdev->dev;
	u32 q;

	tasklet_kill(&priv->rx_tasklet);

	for (q = 0; q < Q_NUM; q++) {
		struct rtl8192f_tx_ring *ring = &priv->tx_ring[q];

		if (ring->info) {
			u32 i;

			for (i = 0; i < RTL8192F_NUM_TX_DESC; i++) {
				if (ring->info[i].skb) {
					dma_unmap_single(dev,
							 ring->info[i].dma,
							 ring->info[i].len,
							 DMA_TO_DEVICE);
					dev_kfree_skb(ring->info[i].skb);
					ring->info[i].skb = NULL;
				}
			}
			kfree(ring->info);
			ring->info = NULL;
		}
		/* bd memory was allocated with dmam_*: freed by device */
	}

	/* the beacon slot (bd, txdesc, frame) is devres-managed coherent
	 * memory and no skb is retained any more, so nothing to release here.
	 */

	if (priv->rx_ring.info) {
		u32 i;

		for (i = 0; i < RTL8192F_NUM_RX_DESC; i++) {
			if (priv->rx_ring.info[i].skb) {
				dma_unmap_single(dev,
						 priv->rx_ring.info[i].dma,
						 RTL8192F_RX_BUF_SIZE,
						 DMA_FROM_DEVICE);
				dev_kfree_skb(priv->rx_ring.info[i].skb);
				priv->rx_ring.info[i].skb = NULL;
			}
		}
		kfree(priv->rx_ring.info);
		priv->rx_ring.info = NULL;
	}
}

/* ------------------------------------------------------------------ */
/* start / stop                                                        */
/* ------------------------------------------------------------------ */

/* Put the host's ring cursors back where REG_BD_RWPTR_CLR puts the
 * hardware's, and release the frames the old pass never completed.
 *
 * Both halves have to move together, which means this belongs on the
 * start side as well as the stop side: a mac80211 hardware restart runs
 * .start() without ever running .stop() (net/mac80211/util.c re-adds the
 * interfaces that "in HW restart ... exist already"), so a stop-only
 * reset leaves the host publishing a tail that the cleared engine reads
 * as the whole ring still being queued.  Measured on the board: after a
 * restart the index pair read 0/0 while five descriptors were published,
 * and the engine walked every stale slot up to the old tail - re-sending
 * frames whose buffers the reclaim had already handed back - before
 * reaching the new work.  Mainline rtw88 has the same pairing:
 * rtw_pci_reset_buf_desc() zeroes each ring's rp/wp in the function that
 * writes RTK_PCI_TXBD_RWPTR_CLR, and both rtw_pci_setup() and
 * rtw_pci_dma_release() call it.
 */
static void rtl8192f_trx_ring_reset(struct rtl8192f_soc_priv *priv)
{
	struct rtl8192f_tx_ring *rings = priv->tx_ring;
	unsigned long flags;
	int i;

	rtl8192f_write32(priv, REG_BD_RWPTR_CLR, 0xffffffff);

	spin_lock_irqsave(&priv->irq_lock, flags);
	for (i = 0; i < Q_NUM; i++) {
		struct rtl8192f_tx_ring *ring = &rings[i];
		/* head/tail are both ring-bounded cursors; mask instead
		 * of % so a cursor corruption cannot inflate cnt.
		 */
		u32 cnt = (ring->head - ring->tail) &
			  (RTL8192F_NUM_TX_DESC - 1);
		u32 d;

		for (d = 0; d < cnt; d++) {
			struct rtl8192f_tx_desc_info *info;
			u32 idx = (ring->tail + d) % RTL8192F_NUM_TX_DESC;

			info = &ring->info[idx];
			if (info->skb) {
				dma_unmap_single(&priv->pdev->dev,
						 info->dma, info->len,
						 DMA_TO_DEVICE);
				dev_kfree_skb(info->skb);
				info->skb = NULL;
				info->dma = 0;
				info->len = 0;
			}
		}
		ring->head = 0;
		ring->tail = 0;
	}
	priv->rx_ring.wp = 0;
	priv->rx_ring.rp = 0;
	spin_unlock_irqrestore(&priv->irq_lock, flags);
}

int rtl8192f_txrx_start(struct rtl8192f_soc_priv *priv)
{
	struct rtl8192f_rx_ring *rx = &priv->rx_ring;
	u16 rx_num;
	u32 q;

	/* Wait out any receive pass already in flight before the cursors move:
	 * the rx tasklet walks rx->wp and rx->rp without taking irq_lock, and a
	 * mac80211 hardware restart reaches .start() without ever running
	 * .stop(), which is the only other place that kills it.  hw_init has
	 * masked the interrupts by now, so nothing new can be scheduled behind
	 * this.
	 */
	tasklet_kill(&priv->rx_tasklet);

	/* Reset the hardware and the host cursors together before the
	 * rings are pointed at the hardware; see
	 * rtl8192f_trx_ring_reset() for why the host side has to be
	 * included here and not only on the stop path.
	 */
	rtl8192f_trx_ring_reset(priv);

	/* Ring length then base address, per queue (vendor startup
	 * Hal88XXGen.c 5748-5793 writes NUM first, DESA second;
	 * segment mode 1 = 4 elements, same value the vendor uses).
	 */
	for (q = 0; q < Q_NUM; q++) {
		const struct rtl8192f_queue_regs *qr = &queue_regs_tbl[q];
		struct rtl8192f_tx_ring *ring = &priv->tx_ring[q];

		if (q == Q_BEACON)
			continue;

		rtl8192f_write16(priv, qr->num,
				 (RTL8192F_NUM_TX_DESC & TXBD_NUM_MASK) |
				 TXBD_DESC_MODE_4SEG);
		rtl8192f_write32(priv, qr->desa, lower_32_bits(ring->bd_dma));
	}

	/* REG_RX_RXBD_NUM: RXBD count, BCNQ segment mode bits[14:13],
	 * 32-bit system (bit15 = 0).  Vendor reads/modifies/writes the
	 * same register three times (Hal88XXGen.c 5794-5810).
	 */
	rx_num = (RTL8192F_NUM_RX_DESC & RXBD_NUM_MASK) |
		 BCNQ_DESC_MODE_4SEG;
	rtl8192f_write16(priv, REG_RX_RXBD_NUM, rx_num);

	/* Vendor PrepareRXBD88XX resets the expected RXTAG to 0 on
	 * every ring (re)arm (Hal88XXRxDesc.c:175/:287) - the HW side
	 * tag counter restarts too (BD_RWPTR_CLR above re-arms the
	 * ring), so the host-side expectation must restart with it.
	 * Keeping the previous boot's tag made the first slot of the
	 * second start mismatch forever (no resync below) - the "up ->
	 * scan -> down -> up freezes" signature.
	 */
	rx->rxtag = 0;

	rtl8192f_write32(priv, REG_RXQ_RXBD_DESA, lower_32_bits(rx->bd_dma));

	/* Beacon single slot: BCNQ TXBD area base.  The slot itself is
	 * disarmed (BCN_OWN = 0) until rtl8192f_beacon_upload().
	 */
	rtl8192f_write32(priv, REG_BCNQ_TXBD_DESA,
			 lower_32_bits(priv->bcn.bd_dma));

	/* REG_TRXDMA_CTRL (0x10C, TXDMA_PQ_MAP) and the RX FIFO byte
	 * boundaries are programmed in rtl8192f_hw_init(), following
	 * vendor InitMAC88XX() (Hal88XXGen.c 4907-4933/4966-4973) -
	 * they are MAC-level configuration, not ring setup.
	 */

	rtl8192f_write32(priv, REG_RCR, RTL8192F_RCR_DEFAULT);

	/* HCI DMA control (Hal88XXGen.c 5404-5425, IS_EXIST_RTL8197FEM):
	 * RX/TX burst size 32B (LBUS default), keep BCNQ DMA stopped,
	 * keep BIT15 set - the 8197F power-on default is the RXTAG
	 * ready-check mode this driver's rx path implements.  Disable
	 * RX/TX DMA pre-fetch in 0x3DB.
	 */
	{
		/* Live vendor value for REG_HCI_CTRL (0x300), read back
		 * from a working board.  Both burst limits are at their
		 * top: HCI_MAX_RXDMA [30:28] and HCI_MAX_TXDMA [26:24]
		 * (halmac_bit2.h 18364, 18406) = 7 each; EN_SWENT_L23
		 * (bit17) and EN_HWEXT_L1 (bit16) are set, bit15 puts the
		 * RXBD ready check in RXTAG mode, HCIIO_PERSTB_SEL
		 * (bit31) is set, and HCI_RST_TRXDMA_INTF (bit20) - which
		 * mainline rtw88 pulses from its setup path
		 * (rtw_pci_dma_reset()) - is left clear here.  An earlier
		 * per-field RMW left the RX burst size at 0, which keeps
		 * the RX engine from fetching at all (no RXOK and not even
		 * RDU ever fires).
		 */
		rtl8192f_write32(priv, REG_HCI_CTRL, 0xf7038000);
	}
	rtl8192f_write8(priv, REG_HCI_CTRL2,
			rtl8192f_read8(priv, REG_HCI_CTRL2) |
			HCI2_DIS_RXDMA_PRE | HCI2_DIS_TXDMA_PRE);

	/* RX DMA engine fetch mode, live vendor value (REG_RXDMA_MODE
	 * 0x290 = 0x4 on a working 8197F); without it the RX engine
	 * stays completely silent (no RXOK, no RDU).
	 */
	rtl8192f_write8(priv, REG_RXDMA_MODE,
			rtl8192f_read8(priv, REG_RXDMA_MODE) | 0x04);

	/* Clear pending interrupts, then enable the ones we service in
	 * both groups (FOVW/TXFOVW live in HIMR1).
	 */
	rtl8192f_write32(priv, REG_HISR0, 0xffffffff);
	rtl8192f_write32(priv, REG_HISR1, 0xffffffff);
	rtl8192f_write32(priv, REG_HIMR0, RTL8192F_HIMR0_INT_MASK);
	rtl8192f_write32(priv, REG_HIMR1, RTL8192F_HIMR1_INT_MASK);

	/* InitMAC88XX 8197F block (Hal88XXGen.c 5420-5425, guarded by
	 * IS_EXIST_RTL8197FEM == IS_RTL8197F_SERIES, always true on the
	 * internal SoC): enable the core MIPS CPU clock (0x0008 |=
	 * BIT(14)) and arm the TXREQ clear-disable gate (0x0523 = 0x80,
	 * "Disable BT coextense").  Without these the in-core transmit
	 * scheduler never runs / TXREQ is never granted, which shows up
	 * as FIFO pages handed out but never handed back.
	 */
	rtl8192f_write32(priv, REG_SYS_CLK_CTRL,
			 rtl8192f_read32(priv, REG_SYS_CLK_CTRL) |
			 REG_CPU_CLK_EN);
	rtl8192f_write8(priv, REG_DIS_TXREQ_CLR, 0x80);

	/* Also unconditional in InitMAC88XX: slot time 0x09us (the port
	 * previously only wrote 0x51B on BSS_CHANGED_ERP_SLOT, which may
	 * never fire before the first scan) and the retry-limit register
	 * (vendor AP defaults 0x10/0x10; reset value is zero, which can
	 * stop the MAC from transmitting at all).
	 */
	rtl8192f_write8(priv, REG_SLOT_TIME, RTL8192F_SLOT_SHORT);
	rtl8192f_write16(priv, REG_RETRY_LIMIT, RTL8192F_RETRY_LIMIT_DEFAULT);

	/* All DMA enables move in here, after every ring base address and
	 * BD count is programmed (vendor order contract,
	 * 8192cd_osdep.c 3629-3636 "cannot be moved arbitrarily", then
	 * InitMAC88XX Hal88XXGen.c 5056-5059).  16-bit RMW: REG_PBP
	 * (0x104) sits behind the reserved halfword.
	 */
	rtl8192f_write16(priv, REG_CR,
			 rtl8192f_read16(priv, REG_CR) |
			 CR_HCI_TXDMA_EN | CR_HCI_RXDMA_EN |
			 CR_TXDMA_EN | CR_RXDMA_EN |
			 CR_MACTXEN | CR_MACRXEN);

	/* Free-page list (AUTO_LLT) with the DMA engine live - the LLT
	 * init must run after the CR enables (vendor InitMAC88XX order);
	 * before it, every TX fails with TXFF_PG_OVF/TXFF_PG_UDN in
	 * REG_TXDMA_STATUS and no frame ever reaches the air.
	 */
	if (rtl8192f_hw_fifo_init(priv))
		return -EIO;

	/* Load the core MIPS CPU firmware now that the TX FIFO page
	 * chain (LLT) is live - the image rides the beacon TXBD path
	 * into the TX FIFO and the on-chip DDMA copies it into
	 * IMEM/DMEM.  With the image running, REG_MCUFWDL reads back
	 * 0x00609428, the same value the factory firmware holds.
	 * What it buys is the in-core scheduler, the consumer of every
	 * H2C command and of the per-station rate tables; without it the
	 * descriptor data path still transmits, so a missing or failed
	 * image is not fatal.
	 */
	if (rtl8192f_fw_download(priv))
		dev_info(&priv->pdev->dev,
			 "fw: no image loaded by this bring-up, continuing\n");

	/* The core CPU start (firmware download / bootcmd) re-asserts
	 * AFE_PORT2_ISO, isolating the transmit analog path again: with
	 * only the power-on clear this register was back to 0x04800000
	 * once the interface went up and nothing reached the air.  Clear
	 * it here, after the CPU is running, and it stays clear for the
	 * rest of the session (0x70 read back as 0x00800000 while a
	 * radiating AP was up).
	 */
	rtl8192f_write32(priv, REG_SYS_CLKR_88XX,
			 rtl8192f_read32(priv, REG_SYS_CLKR_88XX) &
			 ~SYS_CLKR88_AFE_PORT2_ISO);

	priv->running = true;

	return 0;
}

void rtl8192f_txrx_stop(struct rtl8192f_soc_priv *priv)
{
	rtl8192f_write32(priv, REG_HIMR0, 0);
	rtl8192f_write32(priv, REG_HIMR1, 0);
	rtl8192f_write32(priv, REG_HISR0, 0xffffffff);
	rtl8192f_write32(priv, REG_HISR1, 0xffffffff);

	/* Stop RX/TX DMA engines (16-bit RMW) */
	rtl8192f_write16(priv, REG_CR,
			 rtl8192f_read16(priv, REG_CR) &
			 ~(CR_RXDMA_EN | CR_TXDMA_EN));

	tasklet_kill(&priv->rx_tasklet);

	/* Release SKBs not yet reaped and take both the hardware and the
	 * host cursors back to the head of the ring.
	 */
	rtl8192f_trx_ring_reset(priv);

	/* Disarm the beacon slot */
	spin_lock_bh(&priv->irq_lock);
	priv->bcn.bd->ele[0].psb_len &= cpu_to_le16(~TXBD_BCN_OWN);
	spin_unlock_bh(&priv->irq_lock);

	/*
	 * The stop path above may have dropped frames on rings whose
	 * mac80211 queues were stopped by rtl8192f_tx(); make sure
	 * everything is writable again before reporting the hw idle.
	 * mac80211 also stops all queues around ->stop() itself, so
	 * this is what un-stops them for the next start.
	 */
	ieee80211_wake_queues(priv->hw);

	priv->running = false;
}

/* Whether the security engine already took this frame apart: SWDEC says
 * it was left for software and SECURITY names the algorithm applied
 * (RX_DW0_SECURITY_SH / RX_DW0_SWDEC_SH, HalComRXDesc.h 74/78) - the
 * same pair of fields rtw88 rx.c 286 tests.  CAM_ALG_NONE is the value
 * a frame the engine did not touch reports.
 *
 * Neither test depends on how many addresses the header carries, and
 * the engine does decrypt four-address frames: the protected
 * four-address frames a client sources come up with a security type
 * other than NONE and without RXD_W0_ICV_ERR, so the AAD the engine
 * checks covers the fourth address.  Only receive - the transmit half
 * is not four-address aware (rtl8192f_set_key()), which is why a
 * four-address station has no CAM entry today and never reaches this
 * path in hardware.
 */
static bool rtl8192f_rx_hw_decrypted(__le32 rxdesc_w0)
{
	u32 w0 = le32_to_cpu(rxdesc_w0);

	if (w0 & RXD_W0_SWDEC)
		return false;

	return FIELD_GET(RXD_W0_SECURITY, w0) != CAM_ALG_NONE;
}

/* ------------------------------------------------------------------ */
/* xmit                                                                */
/* ------------------------------------------------------------------ */

/*
 * Fallback only: convert a mac80211 tx rate to the Realtek hardware
 * rate index (0-3 CCK 1/2/5.5/11M, 4-11 OFDM 6..54M, 12+ 11n MCS0+).
 * Our band table (tables_*.c) is declared with the CCK rates first
 * and OFDM rates second, in ascending bitrate order, so the mac80211
 * rate index matches the hardware index directly.  Normal operation
 * lets rtl8192f_tx_fill_rate() (rate.c) program the W4 DATARATE field.
 */
static u8 tx_rate_from_info(struct ieee80211_tx_rate *rate)
{
	if (rate->flags & IEEE80211_TX_RC_MCS) {
		/* 1SS 11n: hardware rate index is MCS index + 12 */
		return min_t(u8, rate->idx, 7) + 12;
	}
	if (rate->idx >= 0 && rate->idx < 12)
		return rate->idx;
	return 11;	/* 54M fallback */
}

/* Fill the 40B TX info descriptor (w0-w5, w7, w9; w6/w8 stay zero).
 * Field positions per mainline rtw88 tx.h 24-49 / vendor
 * HalComTXDesc.h.
 * @skb has already been skb_push()ed, so desc = skb->data and the
 * 802.11 header sits at desc + 40; @is_mgmt/@bmc were derived from it
 * by the caller before the push.
 */
static void rtl8192f_fill_tx_desc(struct rtl8192f_soc_priv *priv,
				  struct sk_buff *skb,
				  struct ieee80211_sta *sta,
				  u8 qsel, u8 macid, bool is_mgmt,
				  bool bmc)
{
	struct ieee80211_tx_info *tx_info = IEEE80211_SKB_CB(skb);
	__le32 *desc = (__le32 *)skb->data;
	const struct ieee80211_hdr *hdr = (const struct ieee80211_hdr *)
		(skb->data + RTL8192F_TX_INFO_DESC_SIZE);
	u32 len = skb->len - RTL8192F_TX_INFO_DESC_SIZE;
	u16 hdrlen = ieee80211_hdrlen(hdr->frame_control);
	u16 seq = le16_get_bits(hdr->seq_ctrl, IEEE80211_SCTL_SEQ);
	__le32 w0 = 0, w1 = 0, w2 = 0, w3 = 0, w4 = 0, w5 = 0, w7 = 0;
	u8 rate;

	/*
	 * The 40B info descriptor lives in the skb headroom, which
	 * carries whatever the slab allocator left there - only
	 * w0/w1/w3/w4/w5 are assigned below, so w2/w6/w7.hi/w8/w9
	 * would reach the hardware as garbage.  Those are behaviour
	 * bits: W2 AGG_EN/RAW/NULL_0/NULL_1/MOREFRAG (HalComTXDesc.h
	 * 140-170), W8 EN_HWSEQ and the NEXTHEADPAGE/TAILPAGE TX-FIFO
	 * page chain (HalComTXDesc.h 399-447), W9[23:12] the software
	 * sequence number.  A random page chain or aggregate flag makes
	 * the byte count the DMA hands over disagree with TXPKTSIZE -
	 * exactly PAYLOAD_UDN ("payload is under the packet length of
	 * Tx descriptor", 8192c_reg.h) and TXFOVW in REG_TXDMA_STATUS.
	 * Zero the whole descriptor before any field write: rtw88
	 * pci.c memsets its descriptor, the vendor FillTxDesc88XX_V1
	 * PlatformZeroMemory()s first (Hal88XXTxDesc.c 3570), and the
	 * beacon path in this very file memsets its descriptor - only
	 * the normal TX path missed it.
	 */
	memset(desc, 0, RTL8192F_TX_INFO_DESC_SIZE);

	/* W0: TXPKTSIZE = frame length including the 802.11 header (live
	 * mgmt DW0 = 0x159 = 345 with ELE1 24 + ELE2 321), OFFSET = 40
	 * (W0[23:16], rtw88 tx.h), BMC for group addressed frames.  No
	 * DW0 bit26: see the TXD_W0 comment above.
	 */
	w0 |= le32_encode_bits(len, TXD_W0_TXPKTSIZE);
	w0 |= le32_encode_bits(RTL8192F_TX_INFO_DESC_SIZE, TXD_W0_OFFSET);
	w0 |= le32_encode_bits(bmc, TXD_W0_BMC);

	/* W1: destination MACID (0 = default/broadcast station when
	 * mac80211 has no sta for this frame), queue select, and the
	 * rate-adaptation table id.  RATE_ID is a table selector, not a
	 * rate: the firmware looks the ladder up by it, so it has to carry
	 * what RA_MASK was sent with for this peer (see
	 * rtl8192f_ra_rate_id()), not the macid.  The macid comes from the
	 * peer's own slot (struct rtl8192f_sta) so several clients can be
	 * served at once: each one keeps its rate-adaptation entry and its
	 * TX-report owner instead of everyone sharing whichever peer
	 * authorised last.
	 */
	w1 |= le32_encode_bits(macid, TXD_W1_MACID);
	w1 |= le32_encode_bits(qsel, TXD_W1_QSEL);
	/* mac80211 leaves an hardware-key frame unencrypted and without an
	 * IV: the engine inserts the IV, encrypts, and appends the MIC.
	 * info->control.hw_key is only set for a key this driver accepted in
	 * set_key(), i.e. pairwise CCMP.
	 */
	if (tx_info->control.hw_key &&
	    tx_info->control.hw_key->cipher == WLAN_CIPHER_SUITE_CCMP)
		w1 |= le32_encode_bits(TXD_SEC_TYPE_AES, TXD_W1_SEC_TYPE);
	w1 |= le32_encode_bits(rtl8192f_ra_rate_id(sta), TXD_W1_RATE_ID);

	/* W7[15:0] TXBUFF: the frame length the FIFO consumer reads
	 * (vendor FillTxDesc88XX TX_DESC_TXBUFF = frLen, Hal88XXTxDesc.c
	 * 1199; every vendor TX path fills it - beacon 2006, rsvd page
	 * 2150, shortcut 2582).  Left zero the core firmware consumes
	 * the descriptor and returns the page but transmits nothing.
	 */
	/* TXBUFF = TXPKTSIZE - hdrlen, the payload length without the
	 * 802.11 header - the same length TXBD element 2 describes.
	 */
	w7 = le32_encode_bits(len - hdrlen, TXD_W7_TXBUFF);

	/* W2[29:24] GROUP_ID 0x3f = "no MU group" (vendor live mgmt
	 * descriptors carry 3f000000 in DW2; zero means "MU group 0",
	 * which has no members and is never scheduled).
	 */
	w2 |= le32_encode_bits(0x3f, TXD_W2_G_ID);

	/* Fixed-rate transmit; the actual rate comes from rate.c into
	 * W3 (USE_RATE) / W4 (DATARATE, same bit positions the gen-2
	 * layout used, so rate.c needs no change on the TX side).
	 * Fallback if rate.c could not derive a rate (no rate control
	 * result, e.g. mgmt frames or no sta): program the last usable
	 * legacy rate so the frame still goes out.
	 */
	w3 |= TXD_W3_USE_RATE;
	/* Allow rate fallback across the full table (mainline rtw88
	 * sets DATARATE_FB_LIMIT 0x1f on the old-datarate chips).
	 */
	w4 |= le32_encode_bits(0x1f, TXD_W4_DATARATE_FB_LIMIT);

	/* Per-frame data retry limit.  Without RTY_LMT_EN the MAC applies
	 * the global short/long limits (REG_RETRY_LIMIT, 0x10/0x10 here),
	 * so a group addressed frame - which no station will ever ACK -
	 * burns ten retries on air every time.  The vendor overrides the
	 * limit per descriptor instead: broadcast/multicast gets
	 * RTY_LMT_EN with a limit of 0 ("No retry for broadcast",
	 * Hal88XXTxDesc.c 3689-3694), management frames get 6
	 * (8192cd_tx.c 3775-3788).  Unicast data keeps the global limits.
	 */
	if (bmc) {
		w4 |= TXD_W4_RTY_LMT_EN;
		w4 |= le32_encode_bits(0, TXD_W4_DATA_RT_LMT);
	} else if (is_mgmt) {
		w4 |= TXD_W4_RTY_LMT_EN;
		w4 |= le32_encode_bits(6, TXD_W4_DATA_RT_LMT);
	}

	rtl8192f_tx_fill_rate(priv, desc, tx_info, sta);
	rate = le32_get_bits(desc[4], TXD_W4_DATARATE);
	if (!rate)
		rate = tx_rate_from_info(&tx_info->control.rates[0]);
	w4 |= le32_encode_bits(rate, TXD_W4_DATARATE);

	/*
	 * RTS/CTS: mac80211 asks for protection with control.use_rts
	 * (RTS) or control.use_cts_prot (CTS-to-self).  RTS rate: CCK
	 * 11M (hw index 3) for OFDM/MCS data rates, the data rate
	 * itself for legacy CCK, as the vendor driver does when
	 * protection is required.
	 */
	if (tx_info->control.use_rts) {
		u8 rts_rate;

		/* OFDM/MCS data -> RTS at CCK 11M; legacy CCK data
		 * keeps its own rate for RTS.
		 */
		if (rate >= 4)
			rts_rate = 3;
		else
			rts_rate = rate;
		w3 |= TXD_W3_USE_RTS;
		w4 |= le32_encode_bits(rts_rate, TXD_W4_RTSRATE);
	} else if (tx_info->control.use_cts_prot) {
		/* CTS-to-self: DW3 bit11 (vendor Hal88XXTxDesc.c
		 * 1136-1137; only writing the RTS rate left the
		 * protection mechanism inactive).
		 */
		w3 |= TXD_W3_CTS2SELF;
		w4 |= le32_encode_bits(3, TXD_W4_RTSRATE);
	}

	/* W5 DATA_SHORT is one bit serving two meanings: short preamble
	 * on CCK rates, short GI on HT rates (vendor TX_DW5_DATA_SHORT,
	 * rtw88 RTW_TX_DESC_W5_DATA_SHORT).
	 */
	if (!is_mgmt && rate < 4 &&
	    (tx_info->control.rates[0].flags &
	     IEEE80211_TX_RC_USE_SHORT_PREAMBLE))
		w5 |= TXD_W5_DATA_SHORT;
	else if (!is_mgmt && rate >= 0x0c && rate <= 0x13 &&
		 (tx_info->control.rates[0].flags & IEEE80211_TX_RC_SHORT_GI))
		w5 |= TXD_W5_DATA_SHORT;

	/* DATA_BW/DATA_SC say how wide this frame is.  The analyser moved
	 * there or to 20 MHz in rtl8192f_set_bandwidth(), but the PHY takes
	 * the width per descriptor, so a frame that does not name it is
	 * transmitted 20 MHz wide no matter what the synthesizer is doing.
	 * A 40 MHz frame uses both halves; a narrower frame inside a 40 MHz
	 * BSS has to name which half is its primary, and so does the RTS in
	 * front of it (vendor 8192cd_tx.c 3435-3437 for the 40 MHz case,
	 * 3443-3452 and 3608-3620 for the narrower ones).  Rate control only
	 * offers 40 MHz groups once the peer negotiated that width, so a
	 * 20 MHz client on a 40 MHz BSS lands in the second arm.
	 */
	if (tx_info->control.rates[0].flags & IEEE80211_TX_RC_40_MHZ_WIDTH)
		w5 |= le32_encode_bits(TXD_DATA_BW_40, TXD_W5_DATA_BW) |
		      le32_encode_bits(TXD_DATA_SC_BOTH, TXD_W5_DATA_SC);
	else if (priv->cur_chan_type == NL80211_CHAN_HT40MINUS)
		w5 |= le32_encode_bits(TXD_DATA_SC_BELOW, TXD_W5_DATA_SC) |
		      le32_encode_bits(TXD_DATA_SC_BELOW, TXD_W5_RTS_SC);
	else if (priv->cur_chan_type == NL80211_CHAN_HT40PLUS)
		w5 |= le32_encode_bits(TXD_DATA_SC_ABOVE, TXD_W5_DATA_SC) |
		      le32_encode_bits(TXD_DATA_SC_ABOVE, TXD_W5_RTS_SC);

	/* W2 AGG_EN is the switch that makes the core collect consecutive
	 * descriptors of this macid/queue into one A-MPDU instead of
	 * handing each one its own ACK.  mac80211 sets TX_CTL_AMPDU only
	 * once the peer has an operational AddBA agreement for this TID,
	 * which is the same gate the vendor applies (ADDBA_ready[TID],
	 * 8192cd_tx.c 3468-3474).  MAX_AGG_NUM counts pairs of MPDUs: the
	 * vendor derives pstat->maxAggNum from the peer's A-MPDU length
	 * factor - 5/10/21/42 for factor 0..3 (8192cd_sme.c 24468-24485) -
	 * and writes half of it here (8192cd_tx.c 3520).  AMPDU_DENSITY
	 * follows the vendor's encrypted-frame value 7 (3501-3503);
	 * otherwise the peer's negotiated minimum spacing.
	 */
	if (sta && (tx_info->flags & IEEE80211_TX_CTL_AMPDU)) {
		static const u8 max_agg_num[] = { 2, 5, 10, 21 };
		u8 factor = sta->deflink.ht_cap.ampdu_factor;
		u8 density = ieee80211_has_protected(hdr->frame_control) ? 7 :
			sta->deflink.ht_cap.ampdu_density;

		w2 |= TXD_W2_AGG_EN | le32_encode_bits(density,
						      TXD_W2_AMPDU_DENSITY);
		w3 |= le32_encode_bits(max_agg_num[min_t(u8, factor, 3)],
				       TXD_W3_MAX_AGG_NUM);
	}

	desc[0] = w0;
	desc[1] = w1;
	desc[2] = w2;
	desc[3] = w3;
	desc[4] = w4;
	desc[5] = w5;
	desc[7] = w7;
	/* W9 SW_SEQ: the 12-bit sequence number taken from the frame
	 * header, W8 EN_HWSEQ left 0.  Both references agree on this
	 * branch: vendor FillTxDesc88XX_V1 3740 writes SW_SEQ because
	 * CFG_HAL_HW_SEQ is 0 for 8197F (WLAN_HAL_HW_SEQ sits inside the
	 * commented-out WLAN_HAL_HW_TX_SHORTCUT_HDR_CONV, 8192cd_cfg.h
	 * 1605-1610), and mainline rtw88 tx.c 81-83 does the same.
	 */
	desc[9] = cpu_to_le32(le32_encode_bits(seq, TXD_W9_SW_SEQ));

	/*
	 * No TXDESC checksum: on the 8197F the W7[15:0] field is the
	 * TXBUFF timestamp/frLen (FillTxDesc88XX_V1 3733, read back as
	 * a buffer size at 2919-2925), and halmac's
	 * halmac_fill_txdesc_check_sum_88xx / SET_TX_DESC_CHK_EN have
	 * zero callers on the live 8197F path - the rtw88-style
	 * checksum is a gen-2 PCIe convention.  Writing one clobbers
	 * functional W7 bits; leaving 0 matches the vendor beacon path
	 * (Hal88XXTxDesc.c 3188-3190, "// no use ?").
	 */
}

static void rtl8192f_tx_kick(struct rtl8192f_soc_priv *priv, u8 qidx)
{
	const struct rtl8192f_queue_regs *qr = &queue_regs_tbl[qidx];
	struct rtl8192f_tx_ring *ring = &priv->tx_ring[qidx];

	/* Kick = publish the host producer index in the low 16 bits of
	 * the queue's TXBD_IDX register (vendor Hal88XXTxDesc.c
	 * 1389-1390 writes the index with a 16-bit store; mainline
	 * rtw88 pci.c 796 the same).
	 */
	wmb();	/* TXBD + info desc visible before the index update */
	rtl8192f_write16(priv, qr->idx, ring->head & TXBD_NUM_MASK);
}

netdev_tx_t rtl8192f_tx(struct ieee80211_hw *hw, struct sk_buff *skb,
			struct ieee80211_sta *sta)
{
	struct rtl8192f_soc_priv *priv = hw->priv;
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
	struct rtl8192f_tx_ring *ring;
	struct rtl8192f_tx_desc_info *info;
	struct rtl8192f_txbd *bd;
	u32 head, next;
	u32 len;
	dma_addr_t dma;
	u8 qidx;
	u8 qsel;
	u16 ac;
	u16 psb;
	u16 hdrlen;
	bool is_data;
	bool is_mgmt;
	bool bmc;
	unsigned long flags;

	if (!priv->running)
		goto drop;

	is_data = ieee80211_is_data(hdr->frame_control);
	is_mgmt = !is_data || ieee80211_is_mgmt(hdr->frame_control);
	bmc = is_multicast_ether_addr(hdr->addr1);

	/*
	 * Queue selection with the 88XX qsel encoding (0..15 = TID,
	 * 16 = BCN, 17 = HIGH, 18 = MGMT; mainline rtw88 tx.h 56-77):
	 * management and control frames go on the MGMT ring (qsel 18),
	 * group addressed data on the HIGH ring (qsel 17), everything
	 * else follows the AC mac80211 put in the skb queue mapping,
	 * with the skb TID as qsel.  (EAPOL is data and keeps its AC
	 * queue, matching the vendor driver.)
	 */
	if (is_mgmt) {
		qidx = Q_MGNT;
		qsel = priv->tx_ring[Q_MGNT].qsel;
	} else if (bmc) {
		qidx = Q_HIGH;
		qsel = priv->tx_ring[Q_HIGH].qsel;
	} else {
		qidx = ac_to_queue(skb_get_queue_mapping(skb));
		qsel = skb->priority & 0x0f;	/* 88XX: TID0..15 */
	}
	ac = queue_to_ac(qidx);
	ring = &priv->tx_ring[qidx];

	spin_lock_irqsave(&priv->irq_lock, flags);

	head = ring->head;
	next = (head + 1) % RTL8192F_NUM_TX_DESC;
	if (next == ring->tail) {
		/* The queue-done (DOK) bits are consumed by the in-core
		 * scheduler and must not be host-enabled; doing so kills the
		 * transmitter.  So reclamation normally rides on RXOK.  A burst
		 * toward a peer that answers with nothing the host can see -
		 * ACK/CTS are not DMA'd - generates no such interrupt: the ring
		 * would stay full, the AC stay stopped and the link die silently.
		 * Reclaim from the hardware consumer index before giving up the
		 * slot; tx_isr takes the lock itself, so drop it around the call
		 * and reload the cursors afterwards.
		 */
		spin_unlock_irqrestore(&priv->irq_lock, flags);
		rtl8192f_tx_isr(priv);
		spin_lock_irqsave(&priv->irq_lock, flags);

		head = ring->head;
		next = (head + 1) % RTL8192F_NUM_TX_DESC;
	}
	if (next == ring->tail) {
		/* Ring full: stop the mac80211 queue under the same
		 * lock rtl8192f_tx_isr() wakes it from - stopping
		 * outside the lock lets a concurrent wake land before
		 * the stop and leave the AC stuck forever.  Drop this frame.
		 */
		if (is_data)
			ieee80211_stop_queue(hw, ac);
		spin_unlock_irqrestore(&priv->irq_lock, flags);
		goto drop;
	}

	/*
	 * Vendor 3-segment TXBD (SetTxBufferDesc88XX, Hal88XXTxDesc.c
	 * 1401-1625; the merged desc+header+payload variant only exists
	 * behind the commented-out MERGE_TXDESC_HEADER_PAYLOAD,
	 * 8192cd_cfg.h 1554): element 0 = the 40B info descriptor,
	 * element 1 = the 802.11 header, element 2 = the payload, and
	 * element 3 stays zero (no ICV/MIC on this open path).  The core
	 * firmware walks the segments, so the header/payload boundary
	 * must match the vendor's exactly (live mgmt: ELE1 = 24B,
	 * ELE2 = 321B).
	 */
	len = skb->len;
	skb_push(skb, RTL8192F_TX_INFO_DESC_SIZE);
	hdrlen = ieee80211_hdrlen(hdr->frame_control);

	/*
	 * REG_TXDMA_STATUS is write-1-to-clear and the ISR only clears
	 * it on an overflow/error interrupt - reading it here without a
	 * preceding clear reports the previous frame's stale bits
	 * (vendor ISR does "r = read; write(r)" in its TXFOVW/TXERR
	 * branches, 8192cd_osdep.c 1765-1795).  Clear first, then
	 * snapshot after the kick so the readout belongs to this frame.
	 */
	rtl8192f_write32(priv, REG_TXDMA_STATUS,
			 rtl8192f_read32(priv, REG_TXDMA_STATUS));

	rtl8192f_fill_tx_desc(priv, skb, sta, qsel,
			      sta ? ((struct rtl8192f_sta *)sta->drv_priv)->macid :
				    0,
			      is_mgmt, bmc);

	dma = dma_map_single(&priv->pdev->dev, skb->data, skb->len,
			     DMA_TO_DEVICE);
	if (dma_mapping_error(&priv->pdev->dev, dma)) {
		spin_unlock_irqrestore(&priv->irq_lock, flags);
		goto drop;
	}

	info = &ring->info[head];
	info->skb = skb;
	info->dma = dma;
	info->len = skb->len;
	/* Keep the rate the descriptor was programmed with: the tx done
	 * path has to echo it back, or rate control never sees which
	 * rate was actually tried and pins the link at the basic rate.
	 */
	info->rate = le32_get_bits(((__le32 *)skb->data)[4],
				   TXD_W4_DATARATE);
	info->sgi = !!le32_get_bits(((__le32 *)skb->data)[5],
				    TXD_W5_DATA_SHORT);

	/*
	 * PSB = page count of (40 + frame) with the 8197F's fixed 256B
	 * page size (Hal88XXTxDesc.c 1653-1669, PBP_PSTX_SIZE = 256;
	 * NOT the PCIe 128 that rtw88 uses).
	 */
	psb = ((RTL8192F_TX_INFO_DESC_SIZE + len - 1) /
	       RTL8192F_TX_PAGE_SIZE) + 1;

	bd = txbd_of(ring, head);
	memset(bd, 0, RTL8192F_TXBD_SIZE);
	bd->ele[0].buf_size = cpu_to_le16(RTL8192F_TX_INFO_DESC_SIZE);
	bd->ele[0].psb_len = cpu_to_le16(psb);
	bd->ele[0].dma = cpu_to_le32(dma);
	bd->ele[1].buf_size = cpu_to_le16(hdrlen);
	bd->ele[1].dma = cpu_to_le32(dma + RTL8192F_TX_INFO_DESC_SIZE);
	bd->ele[2].buf_size = cpu_to_le16(len - hdrlen);
	bd->ele[2].dma = cpu_to_le32(dma + RTL8192F_TX_INFO_DESC_SIZE +
				     hdrlen);
	/* ele[3] stays zero: no ICV/MIC segment (open path); hardware
	 * encryption gather would go here .
	 */

	ring->head = next;

	rtl8192f_tx_kick(priv, qidx);

	/*
	 * Stop the mac80211 queue for this AC when the ring is nearly
	 * full (under irq_lock, see above); rtl8192f_tx_isr() wakes it
	 * back up after reaping.
	 */
	if (is_data && (next + 2) % RTL8192F_NUM_TX_DESC == ring->tail) {
		ieee80211_stop_queue(hw, ac);
		/* tx_isr may have run between the space check and the
		 * stop; re-test so a reap that already freed space
		 * re-opens the queue immediately.
		 */
		if ((next + 2) % RTL8192F_NUM_TX_DESC != ring->tail)
			ieee80211_wake_queue(hw, ac);
	}

	spin_unlock_irqrestore(&priv->irq_lock, flags);

	return NETDEV_TX_OK;

drop:
	/* This skb came in through mac80211's data path, so it may be the
	 * head of a fragment list that mac80211 still owns a reference to;
	 * ieee80211_free_txskb() is the only release that handles that (a
	 * plain kfree of an skb mac80211 still counts on is a use-after-free
	 * waiting for its window).
	 */
	ieee80211_free_txskb(hw, skb);
	return NETDEV_TX_OK;
}

/* ------------------------------------------------------------------ */
/* TX done                                                             */
/* ------------------------------------------------------------------ */

/* Turn the descriptor's TX_RATE encoding back into the rate report mac80211
 * hands to rate control: bits 5:0 carry the linear table value (the legacy
 * entries sit at the same positions in the band table, 0x0c = MCS0) and bit 6
 * is the short-guard-interval request.  count = 1 is the best this path can
 * say - the core firmware retries inside the MAC and does not report how
 * often.
 */
static void rtl8192f_tx_report_rate(struct ieee80211_tx_info *txi, u8 rate,
				    bool sgi)
{
	struct ieee80211_tx_rate *r = &txi->status.rates[0];
	u8 value = rate & 0x3f;

	if (value >= 0x0c && value <= 0x13) {
		r->flags = IEEE80211_TX_RC_MCS;
		if (sgi)
			r->flags |= IEEE80211_TX_RC_SHORT_GI;
		r->idx = value - 0x0c;
	} else {
		r->idx = value;
	}
	r->count = 1;
}

void rtl8192f_tx_isr(struct rtl8192f_soc_priv *priv)
{
	u32 q;

	for (q = 0; q < Q_NUM; q++) {
		struct rtl8192f_tx_ring *ring = &priv->tx_ring[q];
		const struct rtl8192f_queue_regs *qr;
		unsigned long flags;
		u32 cur_rp;
		u32 count;

		if (q == Q_BEACON)
			continue;	/* not a host-advanced ring */

		qr = &queue_regs_tbl[q];

		spin_lock_irqsave(&priv->irq_lock, flags);

		/* Nothing outstanding on this queue: skip the register read.
		 * The handler calls this on every interrupt, so the idle case
		 * has to be the cheap one.
		 */
		if (ring->head == ring->tail) {
			spin_unlock_irqrestore(&priv->irq_lock, flags);
			continue;
		}

		/* HW consumer index: high 16 bits of the TXBD_IDX
		 * register (vendor GetTxQueueHWIdx88XX,
		 * Hal88XXTxDesc.c 2476-2489).  Read under the lock -
		 * tx() moves head under it and the stop/wake pairing
		 * below needs the same critical section.
		 */
		cur_rp = (rtl8192f_read32(priv, qr->idx) >> 16) &
			 TXBD_NUM_MASK;
		/* TXBD_NUM_MASK is 12 bits but the ring is only 128
		 * entries; a stale or resync-skewed hardware index can
		 * land anywhere in 0..4095.  Without this guard the
		 * (cur_rp - tail) arithmetic below unrolls up to 127
		 * bogus completions per queue inside hardirq, running
		 * ring->tail past the end of info[].
		 */
		if (cur_rp >= RTL8192F_NUM_TX_DESC) {
			if (net_ratelimit())
				dev_warn(&priv->pdev->dev,
					 "tx q%u: bogus HW rp %u >= %u, skipping\n",
					 q, cur_rp,
					 RTL8192F_NUM_TX_DESC);
			spin_unlock_irqrestore(&priv->irq_lock, flags);
			continue;
		}
		if (cur_rp == ring->tail) {
			spin_unlock_irqrestore(&priv->irq_lock, flags);
			continue;
		}
		count = (cur_rp - ring->tail) &
			(RTL8192F_NUM_TX_DESC - 1);
		/* Only what this driver actually published may be reaped.
		 * The HW consumer index can sit ahead of ring->head after a
		 * stop/start cycle (txrx_stop() resets the cursors while the
		 * slots keep their last dma/len), and unwinding that gap would
		 * dma_unmap buffers the ring no longer owns - a stale address
		 * unmap performs cache maintenance on memory that may belong to
		 * someone else by now - and move ring->tail past the producer.
		 */
		count = min_t(u32, count,
			      (ring->head - ring->tail) &
			      (RTL8192F_NUM_TX_DESC - 1));

		while (count--) {
			struct rtl8192f_tx_desc_info *info;
			struct sk_buff *skb;

			info = &ring->info[ring->tail];
			skb = info->skb;
			info->skb = NULL;
			/* A slot without an skb was never published (or was
			 * already recycled); its dma/len must not be unmapped a
			 * second time.
			 */
			if (skb) {
				dma_unmap_single(&priv->pdev->dev, info->dma,
						 info->len, DMA_TO_DEVICE);
				info->dma = 0;
				info->len = 0;
			}

			ring->tail = (ring->tail + 1) %
				     RTL8192F_NUM_TX_DESC;

			if (skb) {
				struct ieee80211_tx_info *txi =
					IEEE80211_SKB_CB(skb);

				/* tx() pushed the 40B info descriptor in front
				 * of the 802.11 header; mac80211 reads the
				 * header from skb->data when it looks the
				 * station up and when it hands the rate report
				 * to rate control, so the descriptor has to go
				 * first (rtw88 does the same in
				 * rtw_pci_tx_done).  Leaving it attached made
				 * every lookup miss: debugfs rc_stats kept
				 * #success/#attempts at zero and the link never
				 * left the initial rate.
				 */
				skb_pull(skb, RTL8192F_TX_INFO_DESC_SIZE);

				ieee80211_tx_info_clear_status(txi);
				rtl8192f_tx_report_rate(txi, info->rate,
							info->sgi);
				/* The scheduler consuming the descriptor is
				 * the only completion report this path has, so it is what gets turned
				 * into an ACK (same shape as adm8211's descriptor-done test, and what
				 * rtlwifi's pci tx done path does).  Claiming nothing instead would
				 * make every MLME frame - authentication and association responses
				 * among them - time out on the cfg80211/hostapd side as "peer did not
				 * acknowledge", which drops the station before it is ever authorised.
				 *
				 * This is a limit of the chip, not a shortcut: the 8197F writes no
				 * per-frame result back anywhere.  The vendor reaps by index only
				 * (8192cd_tx.c:14520-14525 recycles up to the HW write pointer from
				 * Hal88XXTxDesc.c:2477 without consulting any status field), and the
				 * closest thing to a "did the peer ACK" signal is a per-STA aggregate
				 * the firmware reports in answer to H2C_AP_REQ_TXREP (0x43) as C2H
				 * TX_RATE with RPT_TXOK / RPT_TXFAIL (8192cd_sme.c:7616-7634, decode
				 * 8192cd_util.c:8345-8374), used for liveness (DetectSTAExistance88XX:
				 * 300 straight failures, or 30 over 3 s, drops the station) rather
				 * than per-frame status.  That request is not sent here and no report
				 * arrives on any route, so the reap has nothing better than the
				 * descriptor index.
				 */
				if (txi->flags & IEEE80211_TX_CTL_NO_ACK)
					/* Broadcast and multicast frames are
					 * acknowledged by nobody by design, so
					 * claiming an ACK for them asserts
					 * something the chip cannot show;
					 * rtw88's tx done path makes the same
					 * split (pci.c 1003-1007).
					 */
					txi->flags |=
						IEEE80211_TX_STAT_NOACK_TRANSMITTED;
				else
					txi->flags |= IEEE80211_TX_STAT_ACK;
				/* Once a TX block-ack agreement exists,
				 * mac80211 and minstrel_ht both throw away a
				 * report that does not say it covers an
				 * aggregate (status.c 883-885,
				 * rc80211_minstrel_ht.c 1299-1301), so the
				 * rate control never sees a single frame and
				 * the link stays on its initial rate.  Each
				 * reap is one MPDU, so that is the aggregate
				 * length reported.
				 */
				if (txi->flags & IEEE80211_TX_CTL_AMPDU) {
					txi->flags |= IEEE80211_TX_STAT_AMPDU;
					txi->status.ampdu_len = 1;
					txi->status.ampdu_ack_len = 1;
				}
				ieee80211_tx_status_irqsafe(priv->hw, skb);
			}
		}

		/* Wake the mac80211 queue inside the lock - same critical
		 * section rtl8192f_tx() stops it from.  Every
		 * data-carrying ring is woken, HIGH included (group
		 * addressed frames ride it and its stop/wake both map
		 * to AC BE).
		 */
		if (q != Q_MGNT)
			ieee80211_wake_queue(priv->hw, queue_to_ac(q));

		spin_unlock_irqrestore(&priv->irq_lock, flags);
	}
}

/* ------------------------------------------------------------------ */
/* RX                                                                  */
/* ------------------------------------------------------------------ */

/* Hand processed RXBDs back to the hardware.  ring->wp is the arm
 * cursor (next entry to release); processing advances ring->rp past
 * every consumed entry and this loop re-arms from wp up to rp, so an
 * allocation failure just leaves the cursor behind and is retried on
 * the next pass.  Re-arming means: (re)map a fresh buffer, sync it for
 * device access (the MIPS bus is not DMA-coherent), park 0xFFFF
 * in TOTALRXPKTSIZE (never matches the rolling RXTAG, so an entry the
 * HW has not filled cannot be mistaken for a ready one), then release
 * the entries by writing the host read pointer to the low 16 bits of
 * REG_RXQ_RXBD_IDX.
 */
static void rtl8192f_rx_refill(struct rtl8192f_soc_priv *priv)
{
	struct rtl8192f_rx_ring *ring = &priv->rx_ring;
	struct device *dev = &priv->pdev->dev;

	while (ring->wp != ring->rp) {
		struct rtl8192f_rx_desc_info *info = &ring->info[ring->wp];
		struct rtl8192f_rxbd *bd = rxbd_of(ring, ring->wp);

		/* Software cursors only; guard against a future bug
		 * driving either past the array.
		 */
		if (WARN_ON_ONCE(ring->wp >= RTL8192F_NUM_RX_DESC ||
				 ring->rp >= RTL8192F_NUM_RX_DESC))
			return;

		/* info->skb was consumed (unmapped) by the rx tasklet;
		 * install the fresh buffer - the old mapping is never
		 * silently overwritten .
		 */
		if (!info->skb) {
			struct sk_buff *nskb;
			dma_addr_t dma;

			nskb = dev_alloc_skb(RTL8192F_RX_BUF_SIZE + 32);
			if (!nskb) {
				/* Breaking here silently is
				 * self-sustaining: with the ring empty the
				 * hardware stops fetching, so no new
				 * RXOK/RDU arrives to retry this refill.
				 * The warning is the only signal.
				 */
				if (net_ratelimit())
					dev_warn(&priv->pdev->dev,
						 "rx refill: alloc failed\n");
				break;
			}

			dma = dma_map_single(dev, nskb->data,
					     RTL8192F_RX_BUF_SIZE,
					     DMA_FROM_DEVICE);
			if (dma_mapping_error(dev, dma)) {
				dev_kfree_skb(nskb);
				if (net_ratelimit())
					dev_warn(&priv->pdev->dev,
						 "rx refill: map failed\n");
				break;
			}
			info->skb = nskb;
			info->dma = dma;
			bd->dma = cpu_to_le32(dma);
		}

		dma_sync_single_for_device(dev, info->dma,
					   RTL8192F_RX_BUF_SIZE,
					   DMA_FROM_DEVICE);

		/* Vendor runtime refill (UpdateRXBDInfo88XX with bInit
		 * _FALSE, 8192cd_rx.c:7290/:7360) rewrites ONLY
		 * RXBuffSize + the DMA address - never TOTALRXPKTSIZE.
		 * The park (0xffff) exists solely in the bInit path
		 * (Hal88XXRxDesc.c:420-424).  Rewriting TOTAL at runtime
		 * races the DMA's own stamp: both live in RXBD dword0,
		 * and when the park lands after the DMA's tag the slot
		 * reads 0xffff forever - rp stalls, no RP publish, and
		 * HISR0.RDU latches in a self-sustaining storm (the
		 * low-frequency fw-mode freezes).
		 */
		bd->buf_size = cpu_to_le16(RTL8192F_RX_BUF_SIZE);
		/* size + dma visible before the index bump */
		wmb();

		if (++ring->wp >= RTL8192F_NUM_RX_DESC)
			ring->wp = 0;
	}

	rtl8192f_write16(priv, REG_RXQ_RXBD_IDX,
			 ring->wp & TXBD_NUM_MASK);
}

static void rtl8192f_rx_tasklet(struct tasklet_struct *t)
{
	struct rtl8192f_soc_priv *priv =
		container_of(t, struct rtl8192f_soc_priv, rx_tasklet);
	struct rtl8192f_rx_ring *ring = &priv->rx_ring;
	struct ieee80211_hw *hw = priv->hw;
	struct device *dev = &priv->pdev->dev;
	u32 bd_idx, hw_wp;
	int quota = RTL8192F_NUM_RX_DESC;

	for (;;) {
		struct rtl8192f_rx_desc_info *info;
		struct ieee80211_rx_status rx_status = { 0 };
		struct rtl8192f_rxbd *bd;
		struct sk_buff *skb;
		void *rxinfo;
		void *physt;
		u32 total, fs, ls, pkt_len, drv, shift, off;

		/* HW write pointer: high 16 bits of REG_RXQ_RXBD_IDX (0x3B4),
		 * the producer/consumer scheme the vendor uses as the very
		 * loop bound it uses here (8192cd_rx.c:6670
		 * `while (cur_host_idx != cur_q->hw_idx)`).  It is re-read
		 * each round like the vendor's refresh
		 * (8192cd_rx.c:7390 -> UpdateRXBDHWIdx88XX) so frames that
		 * landed while this pass ran are picked up without waiting
		 * for the next interrupt.
		 *
		 * The 12-bit-vs-128 skew guard keeps a bogus pointer from
		 * driving rp past the RXBD array.
		 */
		bd_idx = rtl8192f_read32(priv, REG_RXQ_RXBD_IDX);
		hw_wp = (bd_idx >> 16) & TXBD_NUM_MASK;
		if (hw_wp >= RTL8192F_NUM_RX_DESC) {
			if (net_ratelimit())
				dev_warn(&priv->pdev->dev,
					 "rx: bogus HW wp %u >= %u\n",
					 hw_wp, RTL8192F_NUM_RX_DESC);
			break;
		}
		if (ring->rp == hw_wp || quota-- <= 0)
			break;

		bd = rxbd_of(ring, ring->rp);
		total = le16_to_cpu(bd->total_pkt_size);
		fs = (le16_to_cpu(bd->buf_size) & 0x8000) ? 1 : 0;
		ls = (le16_to_cpu(bd->buf_size) & 0x4000) ? 1 : 0;

		/* RXBD readiness is the index difference computed above,
		 * exactly as the vendor models it: 8192cd_rx.c:6670
		 * `while (cur_host_idx != cur_q->hw_idx)` with hw_idx =
		 * read16(REG_RXQ_RXBD_IDX + 2) (UpdateRXBDHWIdx88XX,
		 * Hal88XXRxDesc.c:1081).  The RXTAG stamp is only a
		 * completion hint: QueryRxDesc88XX_V1 polls it
		 * (Hal88XXRxDesc.c:536-577) and when the poll expires it
		 * adopts the observed stamp and processes the descriptor
		 * anyway - the failure return is commented out at :594.
		 * Gating on the stamp here instead parks the ring
		 * silently: rp never passes the slot, the RXDMA has no
		 * buffer to release into and HISR0.RDU storms.  Measured:
		 * slot rp carried a stamp exactly one
		 * ring lap (128) behind the expected tag, pending stayed 0
		 * for 40 s at a time, and mac80211 counted rx_duplicates
		 * while every ARP exchange timed out.
		 */
		if (total != 0xffff && total != ring->rxtag)
			ring->rxtag = total & 0x1fff;

		/* Expected tag for the next slot (13-bit HW counter,
		 * RXBD_RXTAG_MASK; the vendor advances after every
		 * consumed entry, Hal88XXRxDesc.c 539-542).
		 */
		ring->rxtag = (ring->rxtag + 1) & 0x1fff;

		info = &ring->info[ring->rp];
		skb = info->skb;
		ring->rp = (ring->rp + 1) % RTL8192F_NUM_RX_DESC;

		if (!skb)
			continue;

		dma_sync_single_for_cpu(dev, info->dma,
					RTL8192F_RX_BUF_SIZE,
					DMA_FROM_DEVICE);

		rxinfo = skb->data;
		pkt_len = le32_get_bits(((__le32 *)rxinfo)[0],
					RXD_W0_PKT_LEN);

		if (!fs || !ls) {
			dma_unmap_single(dev, info->dma,
					 RTL8192F_RX_BUF_SIZE,
					 DMA_FROM_DEVICE);
			dev_kfree_skb(skb);
			info->skb = NULL;
			continue;
		}

		drv = le32_get_bits(((__le32 *)rxinfo)[0],
				    RXD_W0_DRV_INFO_SIZE) * 8;
		shift = le32_get_bits(((__le32 *)rxinfo)[0],
				      RXD_W0_SHIFT);
		off = RTL8192F_RX_INFO_DESC_SIZE + drv + shift;

		/* Validate the descriptor's own numbers before a single byte
		 * of payload is read - including by the C2H handler and the
		 * diagnostics below.  PKT_LEN is 14 bits (up to 16383) while
		 * the DMA buffer is RTL8192F_RX_BUF_SIZE, and a descriptor the
		 * hardware never actually wrote carries allocator garbage.
		 * rtw88 does the same check first (pci.c 1083).
		 */
		if (pkt_len < 4 || off + pkt_len > RTL8192F_RX_BUF_SIZE) {
			dma_unmap_single(dev, info->dma,
					 RTL8192F_RX_BUF_SIZE,
					 DMA_FROM_DEVICE);
			dev_kfree_skb(skb);
			info->skb = NULL;
			continue;
		}

		/* Firmware-to-host messages ride the same ring as received
		 * frames; their length field does not include an FCS (unlike
		 * data frames, which we strip 4 bytes of).  Consume the bytes
		 * here and recycle, so a C2H packet is never delivered to
		 * mac80211 as if it were network traffic.
		 */
		if (le32_get_bits(((__le32 *)rxinfo)[2], RXD_W2_C2H_PKT)) {
			rtl8192f_c2h_handle(priv, rxinfo + off, pkt_len);
			dma_unmap_single(dev, info->dma,
					 RTL8192F_RX_BUF_SIZE,
					 DMA_FROM_DEVICE);
			dev_kfree_skb(skb);
			info->skb = NULL;
			continue;
		}

		/* Bad frames are dropped here: CRC/ICV errors and
		 * gather fragments (FS/LS both set expected; gather is
		 * not implemented).
		 */
		if (((__le32 *)rxinfo)[0] &
		    (cpu_to_le32(RXD_W0_CRC32_ERR | RXD_W0_ICV_ERR))) {
			dma_unmap_single(dev, info->dma,
					 RTL8192F_RX_BUF_SIZE,
					 DMA_FROM_DEVICE);
			dev_kfree_skb(skb);
			info->skb = NULL;
			continue;
		}

		/* Vendor drops frames whose content cannot even hold the
		 * FC + Duration (8192cd_rx.c:6819-6828 rx_reuse) - never
		 * forward suspicious bytes to the stack.
		 */
		if (pkt_len - 4 <
		    (u32)ieee80211_hdrlen(get_unaligned_le16(rxinfo + off))) {
			dma_unmap_single(dev, info->dma,
					 RTL8192F_RX_BUF_SIZE,
					 DMA_FROM_DEVICE);
			dev_kfree_skb(skb);
			info->skb = NULL;
			continue;
		}

		/* The mapping is fully consumed below - hand the buffer
		 * to the stack and let the refill install a fresh one
		 * .
		 */
		dma_unmap_single(dev, info->dma, RTL8192F_RX_BUF_SIZE,
				 DMA_FROM_DEVICE);

		/* RCR_APP_FCS keeps the FCS in the received frame
		 * (PKT_LEN includes it); trim the 4 bytes the vendor
		 * also trims (8192cd_rx.c: pktlen = PKT_LEN - _CRCLNG_).
		 */
		pkt_len -= 4;

		if (pkt_len == 0) {
			dev_kfree_skb(skb);
			info->skb = NULL;
			continue;
		}

		/* PHY status segment sits behind the 24B RX info desc,
		 * payload behind it (rate.c parses both from these
		 * pointers, 88XX layout).
		 */
		physt = rxinfo + RTL8192F_RX_INFO_DESC_SIZE;
		rtl8192f_rx_fill_status(priv, rxinfo, physt, &rx_status);

		/* The skb still carries the whole DMA buffer at its
		 * data pointer and skb->len == 0, so moving the data
		 * pointer past the info/PHY-status block is just a
		 * headroom adjustment.
		 */
		skb_reserve(skb, off);
		skb_put(skb, pkt_len);

		/* A frame the engine decrypted keeps its IV in the header,
		 * which mac80211 parses to find the key and check the replay
		 * counter, so only the flag tells the stack to skip software
		 * decryption (net/mac80211/rx.c ieee80211_rx_h_decrypt()).
		 */
		if (rtl8192f_rx_hw_decrypted(((__le32 *)rxinfo)[0]))
			rx_status.flag |= RX_FLAG_DECRYPTED;
		if (!hw->conf.chandef.chan) {
			/* No channel configured - an rx_status with
			 * freq = 0 is rejected by mac80211.
			 */
			dev_kfree_skb(skb);
			info->skb = NULL;
			continue;
		}
		rx_status.freq = hw->conf.chandef.chan->center_freq;
		rx_status.band = hw->conf.chandef.chan->band;

		/* mac80211 reads the status from skb->cb - copy the
		 * filled status there (rtw88 pci.c does the same before
		 * ieee80211_rx_napi; without this the stack parsed
		 * uninitialized cb contents and silently dropped every
		 * frame).
		 */
		memcpy(IEEE80211_SKB_RXCB(skb), &rx_status, sizeof(rx_status));
		ieee80211_rx_irqsafe(hw, skb);
		info->skb = NULL;
	}

	rtl8192f_rx_refill(priv);
}

/* Called from the hard IRQ (core.c): only acknowledge progress and
 * hand the buffer processing to the tasklet - no allocation or DMA
 * mapping in interrupt context .  TODO: NAPI is the
 * long-term direction (mainline rtw88 pci.c uses it).
 */
void rtl8192f_rx_isr(struct rtl8192f_soc_priv *priv)
{
	tasklet_schedule(&priv->rx_tasklet);
}

/* ------------------------------------------------------------------ */
/* TX queue stall watch                                                */
/* ------------------------------------------------------------------ */

/* The vendor watches the same condition from its periodic maintenance
 * path: check_hangup() (8192cd_sme.c 4037, called at :8262 and :8454)
 * walks the queues and only then declares a hang - and what it does about
 * one is a full driver reset with FAST_RECOVERY (backup_sta() /
 * restore_backup_sta()), not an engine poke.  That code is live in the
 * shipped build because CHECK_HANGUP comes from CONFIG_PCI_HCI, which
 * config.mk:27 turns on.
 *
 * Here the signal is the pair the completion path already reads: the index we
 * published and the engine's consumer index.  A ring whose two halves differ
 * while the engine's value stops moving is not waiting for the medium -
 * retries over the air still consume descriptors - it is not being served at
 * all, and a hang is only declared when no ring is keeping up, the same rule
 * the vendor applies.  The recovery is mac80211's restart rather than a
 * hand-rolled DMA reset: this core has no per-engine reset to reach (the HAL
 * only wires CheckHangHandler, Hal8197FGen.c:299, and its own hangup path is a
 * full driver reset), and a restart re-runs the same bring-up an ifup does.
 */
#define RTL8192F_TX_WATCH_PERIOD	(2 * HZ)
/* Five passes without progress.  A confirmed hang on this core held its index
 * for well over five minutes with the queues still published, so the window
 * only has to stay clear of the coincidence where the engine consumes exactly
 * ring-size * k descriptors between two samples; the vendor waits 60 s
 * (PENDING_PERIOD, 8192cd.h:3427).
 */
#define RTL8192F_TX_WATCH_STALLS	5
/* Console heartbeat period, in watch passes (2 s each). */
#define RTL8192F_TX_WATCH_BEATS		30

/* Channel loading over a window, in microseconds.
 *
 * The BBP times CCA-busy itself and needs no transmit permission to do it,
 * which is what makes the reading usable beside a halted transmitter: it
 * answers "did the scheduler stop because the air was taken" without asking
 * the scheduler.  The arm sequence is the vendor's N-series branch
 * (start_bbp_ch_load(), read_bbp_ch_load(), 8192cd_hw.c 29699-29738), which
 * their channel-utilisation timer runs continuously in AP mode.
 */
static unsigned int rtl8192f_channel_busy_us(struct rtl8192f_soc_priv *priv,
					     unsigned int window_us,
					     bool *valid)
{
	u32 v;

	rtl8192f_write16(priv, REG_CH_LOAD_PERIOD, window_us / 4);
	rtl8192f_write32(priv, REG_CH_LOAD_CTRL, 0);
	rtl8192f_write32(priv, REG_CH_LOAD_CTRL, BIT(8));
	rtl8192f_write32(priv, REG_CH_LOAD_CTRL, BIT(8) | BIT(0));
	msleep(window_us / 1000 + 20);

	v = rtl8192f_read32(priv, REG_CH_LOAD_STATUS);
	*valid = v & BIT(16);
	if (!*valid)
		return 0;

	return (rtl8192f_read32(priv, REG_CH_LOAD_COUNT) & GENMASK(15, 0)) * 4;
}

/* Report the state a halt was found in.
 *
 * The watch above can only say that the engine stopped taking descriptors.
 * Why is not reachable from anything this driver books, and this core offers
 * no narrower way back: the HAL binds only CheckHangHandler for it
 * (Hal8197FGen.c:299) and the vendor's own hangup recovery is a full driver
 * close/open with the stations copied out first (8192cd_sme.c:4716).  On a
 * live halt, clearing TXPAUSE, re-publishing a ring's write pointer and
 * toggling REG_CR's MACTXEN were each tried and none moved the consumer
 * index, while leaving the station behind did not either - so the recovery
 * stays the restart below and this function is the record taken while the
 * halt is still on the board.  Three readings separate what is left:
 *
 *  - channel loading.  A halt with the air measured busy is a scheduler
 *    waiting for a TXOP, which the vendor calls an adaptivity hangup and
 *    answers by changing channel (8192cd_sme.c:4573-4586).  Under bulk
 *    traffic on a loaded 2.4 GHz channel a normal window measures 10-60%
 *    busy, so a full window stands out against that baseline by itself.
 *  - the firmware mailbox.  Asking it for a transmit report is a question
 *    whose answer never arrives on this chip (fw.c, H2C_88XX_AP_REQ_TXREP),
 *    so it perturbs nothing; a command that is not taken means the scheduler
 *    stopped along with the rings rather than merely gating them.
 *  - the descriptors.  The stalled entry is printed next to the last entry
 *    the engine really consumed, because a field only the stalled one carries
 *    is the only way this side can implicate what we published.  Nothing
 *    here decodes either dump; the comparison is the point.
 *
 * The register line is everything that could hold the transmitter without
 * raising an error latch: TXPAUSE, through which the vendor pauses and
 * resumes queues (8192cd_sme.c:4583), REG_CR's enable group, the TX and RX
 * DMA latches, both interrupt status groups, and the RX ring's index pair.
 * The ring line prints all six TX rings raw: the upper half of each is the
 * engine's consumer index, the lower half the one we published, so which
 * rings are behind and by how much is readable without arithmetic.
 */
/* Name the 802.11 frame a published slot carries.
 *
 * Which frame class is jammed is part of what a halt means - one stubborn
 * Action frame is a different fault from the queue having nothing it is
 * allowed to send - and the raw descriptor bytes cannot say it.  The frame
 * starts RTL8192F_TX_INFO_DESC_SIZE past skb->data, which is how the tx path
 * lays the skb out (rtl8192f_tx_fill_desc()).
 *
 * The kernel's IEEE80211_STYPE_* are subtype fields without the type bits, so
 * each case here has to carry its IEEE80211_FTYPE_* - IEEE80211_STYPE_RTS and
 * IEEE80211_STYPE_AUTH are both 0x00b0 and only the type separates an RTS from
 * an Authentication frame.  Anything unnamed still prints its raw fc.
 */
static const char *rtl8192f_frame_name(u16 fc)
{
	switch (fc & IEEE80211_FCTL_TYPE) {
	case IEEE80211_STYPE_ASSOC_REQ:
		return "assoc-req";
	case IEEE80211_STYPE_ASSOC_RESP:
		return "assoc-resp";
	case IEEE80211_STYPE_REASSOC_REQ:
		return "reassoc-req";
	case IEEE80211_STYPE_REASSOC_RESP:
		return "reassoc-resp";
	case IEEE80211_STYPE_PROBE_REQ:
		return "probe-req";
	case IEEE80211_STYPE_PROBE_RESP:
		return "probe-resp";
	case IEEE80211_STYPE_DISASSOC:
		return "disassoc";
	case IEEE80211_STYPE_AUTH:
		return "auth";
	case IEEE80211_STYPE_DEAUTH:
		return "deauth";
	case IEEE80211_STYPE_ACTION:
		return "action";
	case IEEE80211_FTYPE_CTL | IEEE80211_STYPE_RTS:
		return "rts";
	case IEEE80211_FTYPE_CTL | IEEE80211_STYPE_CTS:
		return "cts";
	case IEEE80211_FTYPE_CTL | IEEE80211_STYPE_BACK:
		return "block-ack";
	case IEEE80211_FTYPE_CTL | IEEE80211_STYPE_BACK_REQ:
		return "block-ack-req";
	case IEEE80211_FTYPE_DATA | IEEE80211_STYPE_DATA:
		return "data";
	case IEEE80211_FTYPE_DATA | IEEE80211_STYPE_QOS_DATA:
		return "qos-data";
	case IEEE80211_FTYPE_DATA | IEEE80211_STYPE_NULLFUNC:
		return "null";
	case IEEE80211_FTYPE_DATA | IEEE80211_STYPE_QOS_NULLFUNC:
		return "qos-null";
	default:
		return "unnamed";
	}
}

/* The TX FIFO page ledger, one "name=free/owned" pair per priority queue.
 * This is the reading that separates "the engine cannot fetch because it has
 * no pages left" from "the engine has pages and still will not fetch": the
 * first is a credit problem in the path that returns them, the second is not.
 * Printed on every heartbeat as well, because a halt-time number only means
 * something against a same-build baseline taken while the ring was served.
 */
static void rtl8192f_tx_page_ledger(struct rtl8192f_soc_priv *priv, char *buf,
				    size_t size)
{
	static const char *const pq_name[REG_FIFOPAGE_NUM] = {
		"hpq", "lpq", "npq", "epq", "pubq",
	};
	size_t off = 0;
	int i;

	for (i = 0; i < REG_FIFOPAGE_NUM; i++) {
		u32 v = rtl8192f_read32(priv, REG_FIFOPAGE_INFO(i));

		off += scnprintf(buf + off, size - off, " %s=%lu/%lu",
				 pq_name[i], FIELD_GET(PQ_AVAL_PG, v),
				 FIELD_GET(PQ_PG_CNT, v));
	}
}

/* The other half of the same question: the page ledger says how many pages a
 * queue has, these say what the hardware last wrote back about releasing them.
 * txq is FWHW_TXQ_CTRL - bit20 is the gate that lets a boundary write set the
 * free tail, bit22 the beacon-download gate the firmware download parks; both
 * are read-modify-written from the beacon path, so the halt and the heartbeat
 * record what they actually hold.  free_tail is the CPU management queue's
 * report of how far the in-core scheduler has finished with the buffer, and
 * bcn_head is the beacon header page that bit22 lets the hardware rewrite.
 * req is the TXREQ clear-disable gate (0x0523) and clk the in-core CPU clock
 * bit (0x0008 bit14); rtl8192f_txrx_start() re-arms both on every open, so a
 * ring that is published and never fetched while req has lost its 0x80, or
 * while clk lacks bit14, names whoever switched the gate off behind us.
 */
static void rtl8192f_tx_tail_state(struct rtl8192f_soc_priv *priv, char *buf,
				   size_t size)
{
	int n;

	n = scnprintf(buf, size, " txq=%#010x hci=%#010x ftail=%#06lx bcn=%#08x",
		      rtl8192f_read32(priv, REG_FWHW_TXQ_CTRL),
		      rtl8192f_read32(priv, REG_HCI_CTRL),
		      FIELD_GET(FW_FREE_TAIL,
				rtl8192f_read32(priv, REG_CPU_MGQ_INFO)),
		      rtl8192f_read16(priv, REG_FIFOPAGE_CTRL_2));
	scnprintf(buf + n, size - n,
		  " req=%#06x clk=%#010x txdma=%#010x sleep=%#010x rel=%#010x metfr=%#06x",
		  rtl8192f_read8(priv, REG_DIS_TXREQ_CLR),
		  rtl8192f_read32(priv, REG_SYS_CLK_CTRL),
		  rtl8192f_read32(priv, REG_TXDMA_OFFSET_CHK),
		  rtl8192f_read32(priv, REG_MACID_SLEEP),
		  rtl8192f_read32(priv, REG_MACID_RELEASE0),
		  rtl8192f_read8(priv, REG_HMETFR));
}

static void rtl8192f_tx_halt_report(struct rtl8192f_soc_priv *priv, int stalled,
				    u32 slot)
{
	struct rtl8192f_tx_ring *ring = &priv->tx_ring[stalled];
	struct device *dev = &priv->pdev->dev;
	struct rtl8192f_txbd stuck_bd, last_bd;
	u8 stuck_desc[RTL8192F_TX_INFO_DESC_SIZE];
	unsigned long flags;
	unsigned int busy_us, window_us = 50000;
	unsigned int last = (ring->tail + RTL8192F_NUM_TX_DESC - 1) %
			    RTL8192F_NUM_TX_DESC;
	bool have_desc = false, slot_ok, busy_valid;
	char line[160];
	char pages[80];
	char pend[80];
	char tail[160];
	size_t poff = 0;
	int off, ret, n;
	u32 i;
	u8 q;

	slot_ok = slot < RTL8192F_NUM_TX_DESC;

	off = scnprintf(line, sizeof(line), "tx halt rings:");
	for (q = 0; q < Q_NUM; q++) {
		if (q == Q_BEACON)
			continue;
		off += scnprintf(line + off, sizeof(line) - off, " %s=0x%08x",
				 priv->tx_ring[q].name,
				 rtl8192f_read32(priv, queue_regs_tbl[q].idx));
		poff += scnprintf(pend + poff, sizeof(pend) - poff, " %s=%u",
				  priv->tx_ring[q].name,
				  (priv->tx_ring[q].head - priv->tx_ring[q].tail) &
				  (RTL8192F_NUM_TX_DESC - 1));
	}
	dev_err(dev, "%s\n", line);

	dev_err(dev, "tx halt: pause=0x%02x cr=0x%04x txdma=0x%x rxdma=0x%x\n",
		rtl8192f_read8(priv, REG_TXPAUSE),
		rtl8192f_read16(priv, REG_CR),
		rtl8192f_read32(priv, REG_TXDMA_STATUS),
		rtl8192f_read32(priv, REG_RXDMA_STATUS));
	dev_err(dev, "tx halt: hisr=0x%08x/0x%08x fwisr=0x%x rxidx=0x%08x\n",
		rtl8192f_read32(priv, REG_HISR0),
		rtl8192f_read32(priv, REG_HISR1),
		rtl8192f_read32(priv, REG_FWISR),
		rtl8192f_read32(priv, REG_RXQ_RXBD_IDX));
	rtl8192f_tx_page_ledger(priv, pages, sizeof(pages));
	dev_err(dev, "tx halt pages:%s\n", pages);
	dev_err(dev, "tx halt pending:%s\n", pend);
	rtl8192f_tx_tail_state(priv, tail, sizeof(tail));
	dev_err(dev, "tx halt tail:%s\n", tail);

	/* Two more questions the line above cannot answer.
	 *
	 * Interrupt masks: our own header records that the per-AC TX-done
	 * bits have to stay out of HIMR0 because the in-core scheduler
	 * consumes them, and the ISR's RDU workaround reaches that register
	 * with a read-modify-write that a concurrent mask update could undo.
	 * So name the TX-done bits here instead of making a reader decode 0xfc.
	 *
	 * Queue configuration: ring lengths, the queue map and the beacon
	 * engine are what the TX path needs intact to make progress; this is
	 * what they hold at the moment the fetch stopped.  The values we
	 * program live in rtl8192f_txrx_start() and hw.c.
	 */
	dev_err(dev, "tx halt irq: himr0=0x%08x himr1=0x%08x%s\n",
		rtl8192f_read32(priv, REG_HIMR0),
		rtl8192f_read32(priv, REG_HIMR1),
		(rtl8192f_read32(priv, REG_HIMR0) & RTL8192F_TX_DONE_MASK) ?
			" (TX-DONE BITS ENABLED)" : "");
	dev_err(dev,
		"tx halt ringlen: mgnt=0x%04x rx=0x%04x vo=0x%04x vi=0x%04x be=0x%04x bk=0x%04x hi=0x%04x\n",
		rtl8192f_read16(priv, REG_MGQ_TXBD_NUM),
		rtl8192f_read16(priv, REG_RX_RXBD_NUM),
		rtl8192f_read16(priv, REG_VOQ_TXBD_NUM),
		rtl8192f_read16(priv, REG_VIQ_TXBD_NUM),
		rtl8192f_read16(priv, REG_BEQ_TXBD_NUM),
		rtl8192f_read16(priv, REG_BKQ_TXBD_NUM),
		rtl8192f_read16(priv, REG_HI0Q_TXBD_NUM));
	dev_err(dev,
		"tx halt queue: map=0x%04x rwpclr=0x%08x retry=0x%02x bcn=0x%08x tbtt=0x%08x\n",
		rtl8192f_read16(priv, REG_TRXDMA_CTRL),
		rtl8192f_read32(priv, REG_BD_RWPTR_CLR),
		rtl8192f_read8(priv, REG_RETRY_LIMIT),
		rtl8192f_read32(priv, REG_BCN_CTRL),
		rtl8192f_read32(priv, REG_MBSSID_BCN_SPACE));

	/* Copy the descriptor bytes under the lock the reaper uses, and print
	 * them after it: the liveness question below sleeps, and the skb this
	 * slot holds is only guaranteed to stay put while the reaper is out.
	 */
	if (slot_ok) {
		spin_lock_irqsave(&priv->irq_lock, flags);
		stuck_bd = *txbd_of(ring, slot);
		last_bd = *txbd_of(ring, last);
		if (ring->info[slot].skb) {
			memcpy(stuck_desc, ring->info[slot].skb->data,
			       sizeof(stuck_desc));
			have_desc = true;
		}
		spin_unlock_irqrestore(&priv->irq_lock, flags);
	}

	busy_us = rtl8192f_channel_busy_us(priv, window_us, &busy_valid);
	ret = rtl8192f_fw_alive(priv);
	dev_err(dev,
		"tx halt: channel busy %u us of %u us (%s), firmware %s the request\n",
		busy_us, window_us, busy_valid ? "count valid" : "no count",
		ret ? "ignores" : "took");

	if (!slot_ok) {
		dev_err(dev, "tx halt: slot %u we wait on is outside the ring\n",
			slot);
		return;
	}

	if (have_desc) {
		u32 w1 = get_unaligned_le32(stuck_desc + 4);
		u32 w4 = get_unaligned_le32(stuck_desc + 16);

		dev_err(dev, "tx halt: stalled slot %u txbd %*ph desc %*ph\n",
			slot, (int)sizeof(stuck_bd), &stuck_bd,
			(unsigned int)sizeof(stuck_desc), stuck_desc);
		/* Who the held frame is aimed at, read out of the descriptor the
		 * driver wrote: the parks have covered unicast and group, both
		 * security settings and both aggregation states, so the next
		 * question is whether they all share a macid or a programmed
		 * rate.
		 */
		dev_err(dev, "tx halt: stuck frame macid=%lu qsel=%lu rate=%lu\n",
			FIELD_GET(TXD_W1_MACID, w1),
			FIELD_GET(TXD_W1_QSEL, w1),
			FIELD_GET(TXD_W4_DATARATE, w4));
	} else {
		dev_err(dev,
			"tx halt: stalled slot %u txbd %*ph, no skb in it\n",
			slot, (int)sizeof(stuck_bd), &stuck_bd);
	}
	dev_err(dev,
		"tx halt: h2c metfr=0x%02x boxes 0x%08x 0x%08x 0x%08x 0x%08x\n",
		rtl8192f_read8(priv, REG_HMETFR),
		rtl8192f_read32(priv, REG_HMEBOX(0)),
		rtl8192f_read32(priv, REG_HMEBOX(1)),
		rtl8192f_read32(priv, REG_HMEBOX(2)),
		rtl8192f_read32(priv, REG_HMEBOX(3)));
	dev_err(dev, "tx halt: last consumed slot %u txbd %*ph\n", last,
		(int)sizeof(last_bd), &last_bd);

	/* The reaper is what stopped, so start the run at the oldest
	 * descriptor we have no completion for and name the whole unpublished
	 * run rather than one slot.  Four is enough to tell "one stubborn frame" from "everything
	 * on this queue"; the skbs are stable because the reaper needs the
	 * same lock this runs under.
	 */
	i = slot;
	for (n = 0; n < 4 && i != (ring->head & (RTL8192F_NUM_TX_DESC - 1));
	     n++) {
		struct sk_buff *skb = ring->info[i].skb;
		u16 fc;

		if (!skb || skb->len <= RTL8192F_TX_INFO_DESC_SIZE + 1) {
			dev_err(dev, "tx halt: slot %u has no frame to name\n",
				i);
			break;
		}

		fc = get_unaligned_le16(skb->data +
					RTL8192F_TX_INFO_DESC_SIZE);
		dev_err(dev, "tx halt: slot %u holds %s (fc 0x%04x, %u B)\n",
			i, rtl8192f_frame_name(fc), fc,
			skb->len - RTL8192F_TX_INFO_DESC_SIZE);
		i = (i + 1) & (RTL8192F_NUM_TX_DESC - 1);
	}
}

/* Name the latched AXI causes instead of printing a mask nobody can decode
 * off a log line.  Bit order is halmac_bit2.h:32851-32856 (CS[5:0]).
 */
static void rtl8192f_axi_causes_str(u32 causes, char *out, size_t len)
{
	static const struct {
		u32 bit;
		const char *name;
	} names[] = {
		{ BIT(0), "slverr-r" },
		{ BIT(1), "slverr-w" },
		{ BIT(2), "decerr-r" },
		{ BIT(3), "decerr-w" },
		{ BIT(4), "txdma-timeout" },
		{ BIT(5), "rxdma-timeout" },
	};
	size_t n = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		if (!(causes & names[i].bit))
			continue;
		n += scnprintf(out + n, len - n, "%s%s", n ? "," : "",
			       names[i].name);
	}
	if (!n)
		scnprintf(out, len, "none");
}

/* Minimum spacing between two recovery restarts.
 *
 * A restart that does not cure the wedge leaves the detector armed again the
 * moment .start() has finished, and the same ring was re-declared 17.9 s after
 * the pass that gave up on it.  Unbounded, that cadence is a restart every
 * twenty seconds, which never lets an association live long enough here to
 * carry traffic.
 */
#define RTL8192F_TX_RECOVER_GAP		(60 * HZ)

/* Recover a halted transmit path.
 *
 * The engine consumes host-published descriptors again only after the core has
 * been re-initialised: measured at eleven separate halts, the mailbox refill,
 * the three H2C commands the AP path normally leaves unsent, REG_TX_HANG_CTRL's
 * search-queue and OQT-bypass bits, a same-value TXBD_IDX write, a hardware
 * read-pointer write, the ring re-arm (NUM then DESA) and a full ring bring-up
 * each left the read pointer exactly where it was.  Clearing the halted
 * queue's own read pointer does make it fetch again, but the engine then walks
 * slots the host already reclaimed, so that is not a usable recovery.
 *
 * Waiting is no better: of 12 declarations that were watched for 30 s instead
 * of restarted, 11 still had the ring unconsumed at the expiry.  Restart is
 * therefore the action, bounded to one per RTL8192F_TX_RECOVER_GAP.
 *
 * Returns whether the restart happened.  A deferred one must not end the watch:
 * only a restart re-arms this work through .start(), so the caller has to
 * reschedule it itself.
 */
static bool rtl8192f_tx_recover(struct rtl8192f_soc_priv *priv, int q,
				unsigned int pending)
{
	if (priv->tx_recoveries &&
	    time_before(jiffies,
			priv->tx_last_recover + RTL8192F_TX_RECOVER_GAP)) {
		dev_err(&priv->pdev->dev,
			"tx stall on %s: %u published, restart deferred, %u recoveries done\n",
			priv->tx_ring[q].name, pending, priv->tx_recoveries);
		return false;
	}

	priv->tx_last_recover = jiffies;
	priv->tx_recoveries++;
	/* The verdict is measured on our own reclaim progress, so it has to be
	 * re-armed together with the rings: .start() resets the cursors these
	 * arrays are compared against.
	 */
	memset(priv->tx_stall, 0, sizeof(priv->tx_stall));
	memset(priv->tx_last_tail, 0, sizeof(priv->tx_last_tail));
	/* Queues the restart work; it calls back into stop()/start(), which
	 * cancels and re-arms this work, so nothing is rescheduled here.
	 */
	ieee80211_restart_hw(priv->hw);
	return true;
}

void rtl8192f_tx_watch(struct work_struct *work)
{
	struct rtl8192f_soc_priv *priv =
		container_of(to_delayed_work(work), struct rtl8192f_soc_priv,
			     tx_watch);
	unsigned long flags;
	char causes_str[48];
	char pages[80];
	char tail[160];
	char pend[80];
	size_t pend_off;
	bool armed = false, beat;
	int stalled = -1;
	int armed_q = -1;
	bool progress = false;
	u32 cs, axi_time, causes;
	u32 be_idx, hisr0, hisr1;
	u32 stalled_pending = 0, stalled_rp = 0, stalled_wp = 0;
	u32 stalled_slot = 0;
	u32 armed_stall = 0, armed_pend = 0, armed_rp = 0, armed_wp = 0;
	u32 q;

	if (!READ_ONCE(priv->running))
		return;

	/* Once any ring is being counted as stalled this tick is worth
	 * bracketing: if the board dies inside a wifi MMIO read, the "pre"
	 * line is the last thing the console ever gets, and that is the only
	 * observation from this side that separates "the bus stopped answering"
	 * from "the transmitter just stopped making progress".
	 */
	for (q = 0; q < Q_NUM; q++)
		if (priv->tx_stall[q])
			armed = true;

	/* The same bracket is worth it every beat, not only when a ring is
	 * flagged: measured here is a whole-SoC stop that raises no halt at all,
	 * takes the console with it, and is only reported afterwards by the
	 * boot code as a watchdog reset.  The last line before such a death is
	 * the whole measurement, so the reads that could be at fault have to
	 * sit between a printed "pre" and a printed answer.
	 */
	priv->tx_ticks++;
	beat = !(priv->tx_ticks % RTL8192F_TX_WATCH_BEATS);
	if (beat)
		dev_warn(&priv->pdev->dev, "tx watch: beat tick=%u\n",
			 priv->tx_ticks);

	if (beat) {
		be_idx = rtl8192f_read32(priv, queue_regs_tbl[Q_BE].idx);
		/* Read only, never written back: HISR1 is write-1-to-clear and
		 * the ISR has its own handling for these bits, so clearing one
		 * here would swallow an event.  A read still answers the
		 * question - a FIFO overflow that latched and stayed is exactly
		 * the unhandled case.
		 */
		hisr1 = rtl8192f_read32(priv, REG_HISR1);
		/* HISR0 is different: the ISR only writes back the bits it
		 * claims (RTL8192F_HIMR0_INT_MASK), so every other latch in
		 * this word is never cleared by anyone and the word is a
		 * history of the whole boot rather than a reading of the
		 * moment.  Clearing exactly the bits no host path consumes is
		 * what makes the halt-time value an event - it then says "this
		 * latched since the last heartbeat", which is the only window
		 * comparable with the ring state.
		 *
		 * The TX-done bits are not among them: the ISR decides whether
		 * to reap from them (core.c), so clearing one between two
		 * interrupts would drop a reclaim and could itself stop the
		 * ring - the failure this work is looking for.
		 */
		hisr0 = rtl8192f_read32(priv, REG_HISR0);
		rtl8192f_write32(priv, REG_HISR0,
				 hisr0 & ~(RTL8192F_HIMR0_INT_MASK |
					   RTL8192F_TX_DONE_MASK));
		rtl8192f_tx_page_ledger(priv, pages, sizeof(pages));
		rtl8192f_tx_tail_state(priv, tail, sizeof(tail));
	}

	if (armed)
		dev_emerg(&priv->pdev->dev, "tx watch: pre\n");

	/* The 8197F latches its AXI bus exceptions here: CS bits[5:0] are
	 * SLVERR and DECERR on read and write plus the TXDMA and RXDMA
	 * timeouts, and 0x1354 holds the two timeout values the master uses
	 * to produce them (halmac_bit2.h:32849-32885).  The vendor driver
	 * never reads either, so this is the only place that asks the block
	 * whether it saw a bus fault rather than merely stopped.
	 */
	cs = rtl8192f_read32(priv, REG_AXI_EXCEPT_CS);
	axi_time = rtl8192f_read32(priv, REG_AXI_EXCEPT_TIME);
	causes = cs & AXI_CAUSE_MASK;
	if (armed || causes) {
		rtl8192f_axi_causes_str(causes, causes_str, sizeof(causes_str));
		dev_err(&priv->pdev->dev,
			"tx watch: axi cs=0x%08x time=0x%08x causes=%s\n",
			cs, axi_time, causes_str);
	}

	spin_lock_irqsave(&priv->irq_lock, flags);
	pend_off = 0;
	for (q = 0; q < Q_NUM; q++) {
		const struct rtl8192f_queue_regs *qr = &queue_regs_tbl[q];
		struct rtl8192f_tx_ring *ring = &priv->tx_ring[q];
		u32 idx, rp;

		if (q == Q_BEACON)
			continue;	/* not a host-advanced ring */

		/* What each ring still owes the engine, counted from our own
		 * pointers rather than from the index pair: the FIFO page
		 * ledger alone cannot tell queued work from a credit that was
		 * never returned, and only the second case can be the halt.
		 */
		if (beat)
			pend_off += scnprintf(pend + pend_off,
					      sizeof(pend) - pend_off, " %s=%u",
					      ring->name,
					      (ring->head - ring->tail) &
					      (RTL8192F_NUM_TX_DESC - 1));

		if (ring->head == ring->tail) {
			priv->tx_stall[q] = 0;
			continue;
		}

		/*
		 * Liveness is our own reclaim advancing, not the hardware index
		 * pair agreeing: ring->tail only moves when the engine reports
		 * TX-done for a descriptor, so a ring with work outstanding whose
		 * tail has not moved for RTL8192F_TX_WATCH_STALLS passes has not
		 * had a single completion in that window.
		 *
		 * The vendor's rule (8192cd_sme.c:4148) compares the two halves of
		 * the index register - ((idx >> 16) ^ idx) & 0xffff == 0 means the
		 * engine caught up - and that equality cannot be trusted here after
		 * a re-arm: measured on this board the read-pointer half stays 0
		 * while the driver has already reaped the whole ring (tick 120 of
		 * #276 reads be=0x0000004c with every ring empty), and in another
		 * run the pair read 0/0 for seven minutes with five descriptors
		 * outstanding, so the equality held and no declaration was ever
		 * made.  The pair is still read, for the report.
		 */
		idx = rtl8192f_read32(priv, qr->idx);
		rp = (idx >> 16) & TXBD_NUM_MASK;
		if (ring->tail == priv->tx_last_tail[q]) {
			if (priv->tx_stall[q] < RTL8192F_TX_WATCH_STALLS)
				priv->tx_stall[q]++;
		} else {
			progress = true;
			priv->tx_stall[q] = 0;
		}
		priv->tx_last_tail[q] = ring->tail;
		priv->tx_last_rp[q] = rp;

		if (priv->tx_stall[q] >= RTL8192F_TX_WATCH_STALLS) {
			stalled = q;
			stalled_pending = (ring->head - ring->tail) &
					  (RTL8192F_NUM_TX_DESC - 1);
			stalled_rp = rp;
			stalled_wp = idx & TXBD_NUM_MASK;
			stalled_slot = ring->tail;
		} else if (priv->tx_stall[q]) {
			armed_q = q;
			armed_stall = priv->tx_stall[q];
			armed_pend = (ring->head - ring->tail) &
				     (RTL8192F_NUM_TX_DESC - 1);
			armed_rp = rp;
			armed_wp = idx & TXBD_NUM_MASK;
		}
	}

	if (progress) {
		/* A reclaim on any ring means the engine is completing frames, so
		 * nothing is hung - the escape the vendor uses (8192cd_sme.c:4184),
		 * decided by our own reaping instead of the index pair, because the
		 * pair is measurably stale after a re-arm.  A ring that is frozen on
		 * its own while others drain is a station that stopped taking data,
		 * not a hung transmitter.
		 */
		memset(priv->tx_stall, 0, sizeof(priv->tx_stall));
		stalled = -1;
	}
	spin_unlock_irqrestore(&priv->irq_lock, flags);

	if (armed_q >= 0) {
		/* The passes before a declaration are the ones no dump ever
		 * covered: they say whether the ring's pages ran out and stayed
		 * out (a queue that cannot be given a page) or whether pages were
		 * available the whole time (a fetch that stopped being asked), and
		 * they carry the index pair next to the reclaim count that decided
		 * the verdict, so a later reading can tell whether the hardware
		 * half was stale at the time.
		 */
		rtl8192f_tx_page_ledger(priv, pages, sizeof(pages));
		rtl8192f_tx_tail_state(priv, tail, sizeof(tail));
		dev_warn(&priv->pdev->dev,
			 "tx watch: armed %s stall=%u pend=%u rp=%u wp=%u pages:%s%s\n",
			 priv->tx_ring[armed_q].name, armed_stall, armed_pend,
			 armed_rp, armed_wp, pages, tail);
	}

	if (armed)
		dev_emerg(&priv->pdev->dev, "tx watch: post\n");

	/* Channel loading, sampled on every pass and accumulated until the
	 * next heartbeat.  The halt report reads this itself, but by then the
	 * engine has already been stopped for RTL8192F_TX_WATCH_STALLS passes
	 * and the peer has usually given up, so that single number says what
	 * the air was like after the damage; sampled per pass it says what the
	 * air was like while the ring was still being served.
	 */
	{
		bool load_valid;
		u32 busy = rtl8192f_channel_busy_us(priv, 60000, &load_valid);

		if (load_valid) {
			priv->tx_busy_samples++;
			if (busy > priv->tx_busy_max_us)
				priv->tx_busy_max_us = busy;
		}
	}

	if (beat) {
		dev_warn(&priv->pdev->dev,
			 "tx watch alive: tick=%u be=0x%08x cr=0x%04x hisr1=0x%08x busy=%u/%u\n",
			 priv->tx_ticks, be_idx,
			 rtl8192f_read16(priv, REG_CR), hisr1,
			 priv->tx_busy_max_us, priv->tx_busy_samples);
		/* Split so neither line has to be abbreviated: the ledger is
		 * the comparison partner of the halt report's own page line,
		 * and hisr0 here is "latched since the previous heartbeat".
		 */
		dev_warn(&priv->pdev->dev,
			 "tx watch pages:%s hisr0=0x%08x\n", pages, hisr0);
		dev_warn(&priv->pdev->dev, "tx watch pend:%s%s\n", pend, tail);
		priv->tx_busy_max_us = 0;
		priv->tx_busy_samples = 0;
	}

	if (stalled < 0) {
		schedule_delayed_work(&priv->tx_watch,
				      RTL8192F_TX_WATCH_PERIOD);
		return;
	}

	dev_err(&priv->pdev->dev,
		"tx stall on %s: %u published, no completion for %u passes (rp=%u wp=%u)\n",
		priv->tx_ring[stalled].name, stalled_pending,
		RTL8192F_TX_WATCH_STALLS, stalled_rp, stalled_wp);
	rtl8192f_tx_halt_report(priv, stalled, stalled_slot);
	if (rtl8192f_tx_recover(priv, stalled, stalled_pending))
		return;
	schedule_delayed_work(&priv->tx_watch, RTL8192F_TX_WATCH_PERIOD);
}

void rtl8192f_tx_watch_start(struct rtl8192f_soc_priv *priv)
{
	char causes_str[48];
	u32 cs = rtl8192f_read32(priv, REG_AXI_EXCEPT_CS);
	u32 time = rtl8192f_read32(priv, REG_AXI_EXCEPT_TIME);
	u32 txto = FIELD_GET(GENMASK(11, 0), time);
	u32 rxto = FIELD_GET(GENMASK(23, 12), time);
	u32 rec = FIELD_GET(GENMASK(31, 24), time);

	memset(priv->tx_stall, 0, sizeof(priv->tx_stall));
	memset(priv->tx_last_tail, 0, sizeof(priv->tx_last_tail));
	memset(priv->tx_last_rp, 0, sizeof(priv->tx_last_rp));
	/* Baseline for the AXI probe, and the answer to "is the timeout
	 * mechanism armed at all on this part": TIME bits[11:0] and [23:12]
	 * are the TXDMA/RXDMA timeout values, bits[31:24] the recovery time.
	 * The vendor tree writes none of them, so whatever the boot code left
	 * is what the silicon runs on.
	 */
	rtl8192f_axi_causes_str(cs & AXI_CAUSE_MASK, causes_str,
				sizeof(causes_str));
	dev_info(&priv->pdev->dev,
		 "axi: cs=0x%08x (%s) time=0x%08x txto=%u rxto=%u rec=%u\n", cs,
		 causes_str, time, txto, rxto, rec);
	/* The healthy reference for the lines the halt report prints, so a
	 * stall can be read as a difference rather than as bare hex.
	 */
	dev_info(&priv->pdev->dev,
		 "tx cfg: himr0=0x%08x map=0x%04x be=0x%04x retry=0x%02x bcn=0x%08x tbtt=0x%08x\n",
		 rtl8192f_read32(priv, REG_HIMR0),
		 rtl8192f_read16(priv, REG_TRXDMA_CTRL),
		 rtl8192f_read16(priv, REG_BEQ_TXBD_NUM),
		 rtl8192f_read8(priv, REG_RETRY_LIMIT),
		 rtl8192f_read32(priv, REG_BCN_CTRL),
		 rtl8192f_read32(priv, REG_MBSSID_BCN_SPACE));
	schedule_delayed_work(&priv->tx_watch, RTL8192F_TX_WATCH_PERIOD);
}

void rtl8192f_tx_watch_stop(struct rtl8192f_soc_priv *priv)
{
	cancel_delayed_work_sync(&priv->tx_watch);
}

/* ------------------------------------------------------------------ */
/* Beacon upload (moved from hw.c, rewritten for the 88XX model)       */
/*                                                                     */
/* The beacon does not ride a host-advanced ring: the BCNQ TXBD holds  */
/* a resident single slot (fixed 128B stride per VIF, HalCfg.h         */
/* TXBD_BEACON_OFFSET_V1), the hardware polls it every TBTT.  Upload   */
/* = refresh element 1 (payload), the PSLEN field (TXBD DW0 bits       */
/* [30:16]) and flip BCN_OWN (DW0 bit31); a NULL skb disarms the slot  */
/* by clearing BCN_OWN (vendor SetBeaconDownload88XX, Hal88XXTxDesc.c  */
/* 2248-2274).  The 40B TX info descriptor for the slot lives in its   */
/* own coherent buffer (element 0 points at it permanently).           */
/* ------------------------------------------------------------------ */

/* Vendor default beacon rate: OFDM 6M (FillBeaconDesc88XX writes
 * DATARATE 4 unless mib->beacon_rate says otherwise), basic rate 4 in
 * our band table.  TODO: switch to 1M CCK if 6M proves too hot for
 * long-range clients.
 */
#define RTL8192F_BCN_HW_RATE	0x04

/* Patch the TIM element of the resident beacon in place.
 *
 * This deliberately does not touch the TXBD, its PSLEN or BCN_OWN: the core
 * re-reads this buffer at every TBTT, and a renewal that gates the slot while
 * it is fetching is what previously cost beacons (see
 * rtl8192f_beacon_upload()).  Only a same-length change can be applied this
 * way, which is the usual case - the TIM bitmap width only moves when the
 * highest buffered AID crosses a byte boundary - so anything else is refused
 * and left to the next template renewal.
 */
bool rtl8192f_beacon_tim_update(struct rtl8192f_soc_priv *priv,
				const struct sk_buff *skb, u16 tim_off,
				u16 tim_len)
{
	struct rtl8192f_bcn_slot *bcn = &priv->bcn;
	bool updated = false;

	if (!tim_off || !tim_len || skb->len != READ_ONCE(bcn->frame_len))
		return false;

	spin_lock_bh(&priv->irq_lock);
	if (tim_off + tim_len <= bcn->frame_len) {
		memcpy(bcn->frame + tim_off, skb->data + tim_off, tim_len);
		updated = true;
	}
	spin_unlock_bh(&priv->irq_lock);

	return updated;
}

void rtl8192f_beacon_upload(struct rtl8192f_soc_priv *priv,
			    struct sk_buff *skb)
{
	struct rtl8192f_bcn_slot *bcn = &priv->bcn;
	struct rtl8192f_txbd *bd = bcn->bd;
	__le32 *desc = bcn->txdesc;
	u32 len;
	u16 pslen;
	u16 seq;

	if (!skb) {
		/* stop beacon: clear BCN_OWN (HW stops sending after
		 * the current TBTT) and gate the beacon function.
		 */
		dev_dbg(&priv->pdev->dev, "beacon off\n");
		spin_lock_bh(&priv->irq_lock);
		bd->ele[0].psb_len &= cpu_to_le16(~TXBD_BCN_OWN);
		/* the disarm must be visible before we return */
		wmb();
		spin_unlock_bh(&priv->irq_lock);
		rtl8192f_write8(priv, REG_BCN_CTRL,
				rtl8192f_read8(priv, REG_BCN_CTRL) &
				~BIT(3));
		rtl8192f_write32(priv, REG_FWHW_TXQ_CTRL,
				 rtl8192f_read32(priv, REG_FWHW_TXQ_CTRL) &
				 ~BIT_EN_BCNQ_DL);
		return;
	}

	len = skb->len;
	if (len > RTL8192F_BEACON_MAX_LEN) {
		dev_warn(&priv->pdev->dev,
			 "beacon template %u B exceeds the %u B slot, dropped\n",
			 len, RTL8192F_BEACON_MAX_LEN);
		dev_kfree_skb_any(skb);
		return;
	}

	/* Vendor's update ordering (TBTT handler: 8192cd_osdep.c 1177 clears
	 * BCN_OWN before rebuilding the beacon; 8192cd_sme.c 12289-12290
	 * writes the new buffer and only then sets BCN_OWN again).  The gate
	 * has to come first because the core re-reads this very slot at every
	 * TBTT: a renewal that lands while it is fetching emits one beacon
	 * whose element lengths do not match the body it reads.
	 */
	rtl8192f_write32(priv, REG_FWHW_TXQ_CTRL,
			 rtl8192f_read32(priv, REG_FWHW_TXQ_CTRL) &
			 ~BIT_EN_BCNQ_DL);
	spin_lock_bh(&priv->irq_lock);
	bd->ele[0].psb_len &= cpu_to_le16(~TXBD_BCN_OWN);
	spin_unlock_bh(&priv->irq_lock);

	/* A download is one TXBD + one descriptor + the payload read; 100 us
	 * is more than that takes, so after this nothing is in flight.  It
	 * stays outside the lock because the core is entitled to a TBTT here:
	 * at worst one beacon is skipped, which the TXBCN0OK counter shows.
	 * Every caller (bss_info_changed, remove_interface) is process
	 * context under the wiphy mutex, so this may sleep.
	 */
	usleep_range(100, 200);

	/* The payload goes into the resident, permanently mapped buffer; the
	 * hardware keeps reading that same address at every TBTT, so no
	 * previous buffer is freed under the DMA engine.
	 */
	memcpy(bcn->frame, skb->data, len);
	dev_kfree_skb_any(skb);

	pslen = ((RTL8192F_TX_INFO_DESC_SIZE + len - 1) /
		 RTL8192F_TX_PAGE_SIZE) + 1;

	spin_lock_bh(&priv->irq_lock);

	/* 40B TX info descriptor: broadcast frame, beacon qsel 16,
	 * fixed basic rate (vendor FillBeaconDesc88XX: BMC, OFFSET 40,
	 * TXPKTSIZE, USERATE, DATARATE).
	 */
	memset(desc, 0, RTL8192F_TX_INFO_DESC_SIZE);
	desc[0] = cpu_to_le32(le32_encode_bits(len, TXD_W0_TXPKTSIZE) |
			      le32_encode_bits(RTL8192F_TX_INFO_DESC_SIZE,
					       TXD_W0_OFFSET) |
			      TXD_W0_BMC);
	desc[1] = cpu_to_le32(le32_encode_bits(0, TXD_W1_MACID) |
			      le32_encode_bits(RTL8192F_QSEL_BEACON,
					       TXD_W1_QSEL));
	/* DISDATAFB: the beacon must not fall back to a lower rate
	 * (vendor Hal88XXTxDesc.c 1945-1946).
	 */
	desc[2] = cpu_to_le32(le32_encode_bits(0x3f, TXD_W2_G_ID));
	desc[3] = cpu_to_le32(TXD_W3_USE_RATE | TXD_W3_DISDATAFB);
	desc[4] = cpu_to_le32(le32_encode_bits(RTL8192F_BCN_HW_RATE,
					       TXD_W4_DATARATE) |
		le32_encode_bits(0x1f, TXD_W4_DATARATE_FB_LIMIT));
	/* W5: the beacon is a 20 MHz frame, but in a 40 MHz BSS it still has
	 * to name its primary half (vendor FillBeaconDesc88XX_V1,
	 * Hal88XXTxDesc.c 3138-3145).
	 */
	if (priv->cur_chan_type == NL80211_CHAN_HT40MINUS)
		desc[5] = cpu_to_le32(le32_encode_bits(TXD_DATA_SC_BELOW, TXD_W5_DATA_SC));
	else if (priv->cur_chan_type == NL80211_CHAN_HT40PLUS)
		desc[5] = cpu_to_le32(le32_encode_bits(TXD_DATA_SC_ABOVE, TXD_W5_DATA_SC));
	/* W7 TXBUFF (see rtl8192f_fill_tx_desc; vendor uses len-24) */
	desc[7] = cpu_to_le32(len - 24);
	/* W9 SW_SEQ from the template's own header, as the vendor beacon
	 * path does (FillBEACONTxDesc_88XX, Hal88XXTxDesc.c 3132).
	 */
	seq = le16_get_bits(((const struct ieee80211_hdr *)bcn->frame)->seq_ctrl,
			    IEEE80211_SCTL_SEQ);
	desc[9] = cpu_to_le32(le32_encode_bits(seq, TXD_W9_SW_SEQ));

	/* Element 1: the payload.  Elements 2/3 stay zero. */
	bd->ele[1].buf_size = cpu_to_le16(len);
	bd->ele[1].dma = cpu_to_le32(lower_32_bits(bcn->frame_dma));

	/* PSLEN (TXBD DW0 bits 30:16) then arm the slot: BCN_OWN
	 * (DW0 bit31).  wmb() orders the payload/desc writes ahead of the
	 * OWN write that lets the hardware fetch.
	 */
	bd->ele[0].psb_len = cpu_to_le16(pslen);
	/* PSLEN + payload visible before OWN arms the slot */
	wmb();
	bd->ele[0].psb_len |= cpu_to_le16(TXBD_BCN_OWN);
	bcn->frame_len = len;

	spin_unlock_bh(&priv->irq_lock);

	/*
	 * Re-arm the beacon function in case beacon_get(NULL) was used
	 * to stop transmission.  hw_init() already wrote REG_BCNQ_BDNY
	 * = 0.  BCNQ_BDNY lives at 0x424 on the 88XX map and TBTT /
	 * BCNDMATIM are covered by the vendor MAC_REG init table.
	 * TODO: confirm the BDNY value under the 88XX single-slot model.
	 */
	rtl8192f_write8(priv, REG_BCN_CTRL,
			rtl8192f_read8(priv, REG_BCN_CTRL) | BIT(3));
	rtl8192f_write32(priv, REG_FWHW_TXQ_CTRL,
			 rtl8192f_read32(priv, REG_FWHW_TXQ_CTRL) |
			 BIT_EN_BCNQ_DL);

	dev_dbg(&priv->pdev->dev, "beacon uploaded, %u bytes\n", len);
}
