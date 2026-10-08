// SPDX-License-Identifier: GPL-2.0-only
/*
 * RTL8192F-SoC mac80211 driver - platform glue

 * WiFi core of the Realtek RTL8197F router SoC (integrated 802.11bgn
 * 1T1R, RTL8192F gen-2.5 family, PCI-style descriptor DMA on the SoC bus).
 */

#include "rtl8192f_soc.h"

static void rtl8192f_set_myaddr(struct rtl8192f_soc_priv *priv,
				const u8 *addr);
static void rtl8192f_set_bssid(struct rtl8192f_soc_priv *priv, const u8 *addr);

/* Register macros (REG_MACID/BSSIDR/RRSR/SLOT_TIME, EDCA layout) live
 * in rtl8192f_soc.h - single source of truth .
 */

/* mac80211 AC number (0=BK, 1=BE, 2=VI, 3=VO) -> EDCA parameter register.
 * Matches vendor ordering: EDCA_BK_PARA 0x50C, BE 0x508, VI 0x504,
 * VO 0x500.
 */
static const u16 rtl8192f_edca_reg[4] = {
	REG_EDCA_BK_PARAM, REG_EDCA_BE_PARAM,
	REG_EDCA_VI_PARAM, REG_EDCA_VO_PARAM,
};

static struct ieee80211_channel rtl8192f_channels_24g[] = {
	{ .center_freq = 2412, .hw_value = 1,  .max_power = 20 },
	{ .center_freq = 2417, .hw_value = 2,  .max_power = 20 },
	{ .center_freq = 2422, .hw_value = 3,  .max_power = 20 },
	{ .center_freq = 2427, .hw_value = 4,  .max_power = 20 },
	{ .center_freq = 2432, .hw_value = 5,  .max_power = 20 },
	{ .center_freq = 2437, .hw_value = 6,  .max_power = 20 },
	{ .center_freq = 2442, .hw_value = 7,  .max_power = 20 },
	{ .center_freq = 2447, .hw_value = 8,  .max_power = 20 },
	{ .center_freq = 2452, .hw_value = 9,  .max_power = 20 },
	{ .center_freq = 2457, .hw_value = 10, .max_power = 20 },
	{ .center_freq = 2462, .hw_value = 11, .max_power = 20 },
	{ .center_freq = 2467, .hw_value = 12, .max_power = 20 },
	{ .center_freq = 2472, .hw_value = 13, .max_power = 20 },
	{ .center_freq = 2484, .hw_value = 14, .max_power = 20 },
};

static struct ieee80211_rate rtl8192f_rates[] = {
	{ .bitrate = 10,  .hw_value = 0x00, },
	{ .bitrate = 20,  .hw_value = 0x01, },
	{ .bitrate = 55,  .hw_value = 0x02, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 110, .hw_value = 0x03, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 60,  .hw_value = 0x04, },
	{ .bitrate = 90,  .hw_value = 0x05, },
	{ .bitrate = 120, .hw_value = 0x06, },
	{ .bitrate = 180, .hw_value = 0x07, },
	{ .bitrate = 240, .hw_value = 0x08, },
	{ .bitrate = 360, .hw_value = 0x09, },
	{ .bitrate = 480, .hw_value = 0x0a, },
	{ .bitrate = 540, .hw_value = 0x0b, },
};

static struct ieee80211_supported_band rtl8192f_band_2g = {
	.band		= NL80211_BAND_2GHZ,
	.channels	= rtl8192f_channels_24g,
	.n_channels	= ARRAY_SIZE(rtl8192f_channels_24g),
	.bitrates	= rtl8192f_rates,
	.n_bitrates	= ARRAY_SIZE(rtl8192f_rates),
	.ht_cap		= {
		.ht_supported = true,
		.cap = IEEE80211_HT_CAP_SGI_20 |
		       IEEE80211_HT_CAP_SGI_40 |
		       IEEE80211_HT_CAP_SUP_WIDTH_20_40 |
		       IEEE80211_HT_CAP_MAX_AMSDU |
		       (1 << IEEE80211_HT_CAP_RX_STBC_SHIFT),
		.ampdu_factor = IEEE80211_HT_MAX_AMPDU_16K,
		.ampdu_density = IEEE80211_HT_MPDU_DENSITY_NONE,
		.mcs = {
			.rx_mask = { 0xff, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
			.tx_params = IEEE80211_HT_MCS_TX_DEFINED,
		},
	},
};

static int rtl8192f_config(struct ieee80211_hw *hw, int radio_idx, u32 changed)
{
	struct rtl8192f_soc_priv *priv = hw->priv;
	int ret = 0;

	mutex_lock(&priv->conf_mutex);

	if (changed & IEEE80211_CONF_CHANGE_CHANNEL) {
		struct ieee80211_channel *ch = hw->conf.chandef.chan;

		if (priv->running)
			ret = rtl8192f_set_channel(priv, ch->hw_value,
						   cfg80211_get_chandef_type(&hw->conf.chandef));
	}

	mutex_unlock(&priv->conf_mutex);
	return ret;
}

static int rtl8192f_start(struct ieee80211_hw *hw)
{
	struct rtl8192f_soc_priv *priv = hw->priv;
	int ret;

	mutex_lock(&priv->conf_mutex);

	/* mac80211 opens the hardware through .start() every time, so the
	 * bring-up has to happen here rather than only at probe.  Reusing the
	 * probe-time state is not enough: after `ip link set wlan0 down/up`
	 * an AP ends up with every MAC register identical to the working
	 * first start (BCN_CTRL 0x550=0x105c, TBTT 0x554=0x640064,
	 * FWHW_TXQ_CTRL bit22, beacon slot armed) and yet sources no frame at
	 * all - a monitor capture sees 0 frames from our own TA, while the
	 * same build with hw_power_on() run again gives ~4 beacons per 8 s.
	 * Re-running only hw_init() does not help, and neither does a bare
	 * REG_CR = 0 enable edge; the FSMCO power-up sequence
	 * (96fsoc_power_on + cardemu_to_act) is what restarts the in-core
	 * scheduler that feeds the beacon queue.
	 */
	ret = rtl8192f_hw_power_on(priv);
	if (ret)
		goto out;
	ret = rtl8192f_hw_init(priv);
	if (ret)
		goto out;

	ret = rtl8192f_txrx_start(priv);
	if (ret)
		goto out;

	/* enable rx of data frames, allow MAC to run */
	rtl8192f_write32(priv, REG_RCR, RTL8192F_RCR_DEFAULT);
	/* REG_CR (0x100) is a 16-bit register with REG_PBP (0x104) right
	 * behind the reserved halfword - always 16-bit RMW here.
	 */
	rtl8192f_write16(priv, REG_CR,
			 rtl8192f_read16(priv, REG_CR) |
			 CR_TXDMA_EN | CR_RXDMA_EN);

	/* unmask rx/tx interrupts; detail set lives in txrx start */
	priv->running = true;

	if (priv->cur_channel)
		rtl8192f_set_channel(priv, priv->cur_channel,
				     priv->cur_chan_type);

	ieee80211_wake_queues(hw);

	rtl8192f_tx_watch_start(priv);

out:
	mutex_unlock(&priv->conf_mutex);
	return ret;
}

static void rtl8192f_stop(struct ieee80211_hw *hw, bool suspend)
{
	struct rtl8192f_soc_priv *priv = hw->priv;

	mutex_lock(&priv->conf_mutex);

	rtl8192f_tx_watch_stop(priv);

	ieee80211_stop_queues(hw);
	rtl8192f_write16(priv, REG_CR,
			 rtl8192f_read16(priv, REG_CR) &
			 ~(CR_TXDMA_EN | CR_RXDMA_EN));
	rtl8192f_write32(priv, REG_RCR, 0);
	rtl8192f_txrx_stop(priv);
	priv->running = false;

	mutex_unlock(&priv->conf_mutex);
}

static u64 rtl8192f_get_tsf(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	return 0;
}

int rtl8192f_add_interface(struct ieee80211_hw *hw,
			   struct ieee80211_vif *vif)
{
	struct rtl8192f_soc_priv *priv = hw->priv;

	if (priv->vif)
		return -EOPNOTSUPP;

	switch (vif->type) {
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_AP:
		break;
	default:
		return -EOPNOTSUPP;
	}

	priv->vif = vif;

	/* NETYPE (REG_CR bits[17:16], byte 0x102) - vendor InitMAC88XX
	 * sets it to the operating mode (Hal88XXGen.c 5234); the MAC's
	 * frame-exchange/NAV scheduler keys off it.  The 16-bit CR RMWs
	 * in power_on/txrx_start never touch this byte.
	 */
	rtl8192f_write8(priv, REG_MSR,
			vif->type == NL80211_IFTYPE_AP ?
			MSR_AP : MSR_INFRA);

	/* Our own address goes to REG_MACID (vendor HW_VAR_MAC_ADDR
	 * writes CurrentAddress there) - it is the address the MAC puts
	 * into the TA field of every frame it sources.  Without it the
	 * air shows frames with a TA that is not the netdev address, and
	 * no AP ever answers.
	 */
	rtl8192f_set_myaddr(priv, vif->addr);

	/* Single-BSS AP: the BSSID is our own address (vendor
	 * HW_VAR_BSSID with hwaddr at 8192cd_hw.c:15346).
	 */
	if (vif->type == NL80211_IFTYPE_AP)
		rtl8192f_set_bssid(priv, vif->addr);

	/* STA bring-up, vendor form: RTL_W8(0x422, RTL_R8(0x422) ^ BIT(6))
	 * (8192cd_hw.c 15640).  Byte 0x422 bit6 is bit22 of the FWHW_TXQ_CTRL
	 * dword at 0x420, i.e. EN_BCNQ_DL: a client must not let the beacon
	 * queue download frames.  The rest of the vendor's client-mode block
	 * (TXPAUSE 0x522 wrapped around the TSFTR pair, DIS_TSF_UPDATE_N,
	 * RCR &= ~RCR_CBSSID; 8192cd_sme.c 29977-30010) belongs to the
	 * TSF-sync sequence, not to interface creation, and is still
	 * unported - it must not be reduced to a one-shot write here.
	 */
	if (vif->type == NL80211_IFTYPE_STATION)
		rtl8192f_write8(priv, 0x422,
				rtl8192f_read8(priv, 0x422) ^ BIT(6));

	return 0;
}

void rtl8192f_remove_interface(struct ieee80211_hw *hw,
			       struct ieee80211_vif *vif)
{
	struct rtl8192f_soc_priv *priv = hw->priv;

	/* Take the beacon slot down before the VIF goes away: the hardware
	 * re-reads the resident slot at every TBTT, so it has to be told to
	 * stop owning it (rtl8192f_txrx_stop() does the same on an ifdown).
	 */
	rtl8192f_beacon_upload(priv, NULL);

	priv->vif = NULL;

	/* Back to NO_LINK so a later interface does not inherit the
	 * network type (vendor HW_VAR_MEDIA_STATUS on ifdown).
	 */
	rtl8192f_write8(priv, REG_MSR, MSR_NOLINK);
}

/* Program our own (BSS) address into REG_MACID, the 6-byte TA the MAC
 * sources frames with (vendor HW_VAR_ETHER_ADDR writes exactly
 * REG_MACID + 0..5, Hal88XXGen.c 264-268).
 *
 * There is no per-macid address table here: the 0x610-0x62f block is
 * MACID (0x610) / BSSID (0x618) / MAR (0x620), not an array - the vendor
 * SDK programs a peer's macid only in the TX descriptor and never
 * touches this block for it (no MACID_CFG H2C caller exists in the
 * 8197F SDK).  Writing 0x610 + 8*macid for a client puts the client's
 * address into REG_BSSID (macid 1) and REG_MAR (macid 2), which kills
 * the BSS's own-address matching: measured, no unicast is then
 * exchanged in either direction while beacons and broadcast keep
 * flowing.
 */
static void rtl8192f_set_myaddr(struct rtl8192f_soc_priv *priv,
				const u8 *addr)
{
	rtl8192f_write32(priv, REG_MACID,
			 addr[0] | (addr[1] << 8) |
			 (addr[2] << 16) | (addr[3] << 24));
	rtl8192f_write16(priv, REG_MACID + 4,
			 addr[4] | (addr[5] << 8));
}

/* REG_BSSIDR (0x618): the BSSID the WMAC matches against for its own-BSS
 * work (vendor HW_VAR_BSSID writes dot11OperationEntry.hwaddr here at
 * interface bring-up, 8192cd_hw.c:15346), and the working vendor AP reads
 * 0x618 back equal to 0x610.  mac80211 only flags BSS_CHANGED_BSSID when a
 * user actually sets one, so an AP interface would otherwise leave this at
 * the reset value 0 while BCN_CTRL keeps the BSSID-fit filter enabled.
 */
static void rtl8192f_set_bssid(struct rtl8192f_soc_priv *priv, const u8 *addr)
{
	rtl8192f_write32(priv, REG_BSSIDR,
			 addr[0] | (addr[1] << 8) |
			 (addr[2] << 16) | (addr[3] << 24));
	rtl8192f_write16(priv, REG_BSSIDR + 4,
			 addr[4] | (addr[5] << 8));
}

int rtl8192f_sta_state(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		       struct ieee80211_sta *sta,
		       enum ieee80211_sta_state old_state,
		       enum ieee80211_sta_state new_state)
{
	struct rtl8192f_soc_priv *priv = hw->priv;
	struct rtl8192f_sta *rsta = (struct rtl8192f_sta *)sta->drv_priv;
	int macid;

	if (new_state == IEEE80211_STA_AUTHORIZED &&
	    old_state < IEEE80211_STA_AUTHORIZED) {
		/* STA mode: the AP peer uses the fixed hardware default
		 * macid 1; AP mode clients get a driver-managed macid.
		 * The macid travels in the TX descriptor only - the
		 * hardware has no per-macid address register to program
		 * (see rtl8192f_set_myaddr).
		 */
		if (vif->type == NL80211_IFTYPE_STATION) {
			rsta->macid = 1;
			return 0;
		}

		/* A restart re-adds the station without ever taking it down
		 * (ieee80211_reconfig_stations() walks the state machine up from
		 * NOTEXIST), so a station that already owns a MACID keeps it and
		 * only re-publishes its rate table; allocating a second one here
		 * strands the first for the life of the pool.
		 */
		if (rsta->macid) {
			macid = rsta->macid;
		} else {
			macid = rtl8192f_alloc_macid(priv);
			if (macid < 1 || macid > 31)
				return -ENOSPC;
		}

		rsta->macid = macid;
		rtl8192f_send_ra_mask(priv, macid, sta);
		return 0;
	}

	/* mac80211 walks a departing station down one state at a time
	 * (AUTHORIZED -> ASSOC -> AUTH -> NONE -> NOTEXIST, see
	 * __sta_info_destroy_part2), so the MACID must be returned on the
	 * first step out of AUTHORIZED; waiting for new_state == NONE never
	 * fires.  A deauthorised station is re-authorised later with a fresh
	 * allocation, which is what the RA mask wants anyway.
	 */
	if (old_state == IEEE80211_STA_AUTHORIZED &&
	    new_state < IEEE80211_STA_AUTHORIZED) {
		/* Only MACIDs this driver handed out are returned; the
		 * client-mode macid 1 was never allocated from the pool.
		 */
		if (vif->type != NL80211_IFTYPE_STATION && rsta->macid) {
			rtl8192f_free_macid(priv, rsta->macid);
			rsta->macid = 0;
		}
	}

	return 0;
}

int rtl8192f_conf_tx(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		     unsigned int link_id, u16 queue,
		     const struct ieee80211_tx_queue_params *params)
{
	struct rtl8192f_soc_priv *priv = hw->priv;
	u32 val;

	if (queue >= 4)
		return -EINVAL;

	mutex_lock(&priv->conf_mutex);

	/* the register stores ECW exponents, mac80211 gives raw CW values */
	val = EDCA_VAL(params->aifs,
		       ilog2(params->cw_min + 1), ilog2(params->cw_max + 1),
		       params->txop);
	rtl8192f_write32(priv, rtl8192f_edca_reg[queue], val);

	mutex_unlock(&priv->conf_mutex);
	return 0;
}

void rtl8192f_sw_scan_start(struct ieee80211_hw *hw,
			    struct ieee80211_vif *vif,
			    const u8 *mac_addr)
{
	struct rtl8192f_soc_priv *priv = hw->priv;

	mutex_lock(&priv->conf_mutex);

	priv->scanning = true;

	/* Keep accepting beacons / probe responses during the scan.  Also
	 * pass all addresses so we do not drop the associated AP's
	 * beacons on a foreign channel.
	 */
	rtl8192f_write32(priv, REG_RCR, RTL8192F_RCR_DEFAULT);

	mutex_unlock(&priv->conf_mutex);
}

void rtl8192f_sw_scan_complete(struct ieee80211_hw *hw,
			       struct ieee80211_vif *vif)
{
	struct rtl8192f_soc_priv *priv = hw->priv;

	mutex_lock(&priv->conf_mutex);
	priv->scanning = false;

	rtl8192f_write32(priv, REG_RCR, RTL8192F_RCR_DEFAULT);

	mutex_unlock(&priv->conf_mutex);
}

int rtl8192f_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
		     struct ieee80211_vif *vif, struct ieee80211_sta *sta,
		     struct ieee80211_key_conf *key)
{
	/* No hardware crypto in this stage: return 1 so mac80211 keeps the
	 * key in software (per mac80211 "Hardware crypto acceleration" doc,
	 * verified in net/mac80211/key.c: ret == 1 selects the software
	 * path without an error message).
	 */
	if (cmd == DISABLE_KEY)
		return 0;

	return 1;
}

/* Beacon scheduling and the ATIM TX gate (vendor HW_VAR_MAC_CONFIG, AP
 * branch: Hal88XXGen.c 432/457-464/484-490).  The PHYDM MP image leaves
 * these at their reset values; only this step brings the MAC to what the
 * working vendor AP reads back (0x558 = 0x003c010a, 0x550 byte = 0x5c,
 * 0x528 bit19 clear, 0x540 = 0x80040004, 0x510 = 0x281c320c).
 * BIT_DIS_PKT_TX_ATIM in particular prohibits data TX around every TBTT,
 * and we had it set.  TBTT_PROHIBIT keeps bit31: the vendor writes before
 * the core firmware boots, we run after it set that bit.
 */
static void rtl8192f_ap_beacon_config(struct rtl8192f_soc_priv *priv)
{
	u32 val = rtl8192f_read32(priv, REG_TBTT_PROHIBIT);

	rtl8192f_write32(priv, REG_TBTT_PROHIBIT,
			 (val & BIT(31)) | RTL8192F_TBTT_PROHIBIT_AP);
	rtl8192f_write8(priv, REG_DRVERLYINT, 10);
	rtl8192f_write8(priv, REG_BCNDMATIM, 1);
	rtl8192f_write16(priv, REG_ATIMWND, 0x3c);
	rtl8192f_write16(priv, REG_BCNTCFG, RTL8192F_BCNTCFG_AP);
	rtl8192f_clrbits32(priv, REG_PKT_LIFETIME_CTRL, BIT_DIS_PKT_TX_ATIM);
	rtl8192f_write8(priv, REG_BCN_CTRL,
			BIT_BCN_DIS_TSF_UDT | BIT_BCN_EN_FUNCTION |
			BIT_BCN_DIS_TXBCN_RPT | BIT_BCN_DIS_RX_BSSID_FIT);
}

void rtl8192f_bss_info_changed(struct ieee80211_hw *hw,
			       struct ieee80211_vif *vif,
			       struct ieee80211_bss_conf *info,
			       u64 changed)
{
	struct rtl8192f_soc_priv *priv = hw->priv;

	mutex_lock(&priv->conf_mutex);

	if (changed & BSS_CHANGED_BSSID && is_valid_ether_addr(info->bssid)) {
		/* vendor 8192c_reg.h BSSIDR (0x618), 6 bytes */
		rtl8192f_write32(priv, REG_BSSIDR,
				 info->bssid[0] | (info->bssid[1] << 8) |
				 (info->bssid[2] << 16) |
				 (info->bssid[3] << 24));
		rtl8192f_write16(priv, REG_BSSIDR + 4,
				 info->bssid[4] | (info->bssid[5] << 8));
	}

	if (changed & BSS_CHANGED_ASSOC) {
		/* AAP stays on in station mode: the auth/assoc exchange
		 * happens before vif->cfg.assoc is set, and the AP's
		 * auth/assoc responses are unicast - filtering on our
		 * own address only makes them unreachable.  The vendor
		 * V2 baseline keeps AAP set unconditionally
		 * (Hal88XXGen.c 5248-5250, RCR 0xF400430F).
		 */
		u32 rcr = RTL8192F_RCR_DEFAULT;

		if (!priv->scanning)
			rtl8192f_write32(priv, REG_RCR, rcr);

		dev_dbg(&priv->pdev->dev, "%sassociated\n",
			vif->cfg.assoc ? "" : "de");
	}

	if (changed & BSS_CHANGED_ERP_SLOT)
		/* vendor 8192c_reg.h SLOT_TIME (MAC 0x51B): short 0x09us,
		 * long 0x14us
		 */
		rtl8192f_write8(priv, REG_SLOT_TIME,
				info->use_short_slot ?
				RTL8192F_SLOT_SHORT : RTL8192F_SLOT_LONG);

	/* No RRSR (0x440) write here on purpose.  mac80211's basic_rates
	 * bitmap is not the response-rate set: the vendor programs RRSR to
	 * "all rates" in HW_VAR_... (Hal88XXGen.c 5284-5285) and the
	 * working AP ends up at the MP-image value 0x15d; writing
	 * basic_rates overwrote that with 0xf (read back from the shipped driver), which is
	 * a divergence without a vendor counterpart.
	 */

	if (vif->type == NL80211_IFTYPE_AP) {
		struct sk_buff *skb = NULL;
		bool arm = false;

		if (changed & BSS_CHANGED_BEACON_ENABLED) {
			if (info->enable_beacon) {
				rtl8192f_ap_beacon_config(priv);
				arm = true;
			} else {
				/* also clears BCN_OWN and EN_BCNQ_DL */
				rtl8192f_beacon_upload(priv, NULL);
			}
		}

		/* The slot keeps pointing at one resident buffer, so renewing
		 * the beacon is just a fresh copy: mac80211 notifies
		 * BSS_CHANGED_BEACON every time hostapd renews the template
		 * (net/mac80211/cfg.c ieee80211_assign_beacon), which is also
		 * how the RSN IE and the privacy bit arrive - they are
		 * installed only after the BSS has started.  This replaced a
		 * 10 Hz refresh that re-mapped a new skb every pass and freed
		 * the previous one while an in-flight TBTT fetch could still
		 * read it; the refresh bought nothing measurable (content
		 * updates come through the notification above) and the
		 * driver now touches the slot only when the template changes.
		 */
		if (info->enable_beacon &&
		    (arm || (changed & BSS_CHANGED_BEACON)))
			skb = ieee80211_beacon_get(hw, vif, 0);

		if (skb)
			rtl8192f_beacon_upload(priv, skb);
	}

	mutex_unlock(&priv->conf_mutex);
}

/*
 * AP link power save is not implemented here and there is no .set_tim.
 *
 * mac80211 only computes a station's TIM bit for the driver through
 * set_tim, and that bit flips whenever the station has frames queued -
 * including for an awake station, because the buffered-TID set it uses
 * covers the driver's TX queues.  Re-uploading the resident beacon slot
 * on each such flip gates BCN_OWN away from the hardware while it is
 * fetching a beacon: measured on this board with 4 MB pushed over the
 * association, that produced 25 "beacon TX error" interrupts in a few
 * tens of seconds and dropped the peer's RX to the basic rate with 75 %
 * packet loss, where without set_tim the same run reported none.
 *
 * Doing it right means answering PS-POLL (or u-APSD) as well, which
 * mac80211 offers no host-side path for - the vendor does both from the
 * in-core scheduler, and that firmware is not part of this driver.  So
 * the TIM stays as mac80211 last built it into the template, and a dozing
 * client is served when it next transmits.
 */

static void rtl8192f_op_tx(struct ieee80211_hw *hw,
			   struct ieee80211_tx_control *control,
			   struct sk_buff *skb)
{
	rtl8192f_tx(hw, skb, control->sta);
}

/* mac80211 requires configure_filter.  The RX filter itself is not
 * programmed yet, so report back only the flags the engine already
 * passes through (no per-BSSID filtering) and keep beacon/probe
 * reception open for scanning.
 */
static void rtl8192f_configure_filter(struct ieee80211_hw *hw,
				      unsigned int changed_flags,
				      unsigned int *total_flags,
				      u64 multicast)
{
	struct rtl8192f_soc_priv *priv = hw->priv;

	*total_flags &= FIF_ALLMULTI | FIF_BCN_PRBRESP_PROMISC;
	priv->filter_flags = *total_flags;
}

/*
 * Block-ack sessions are the scheduler's business: it hears the ADDBA
 * exchange on the air and starts grouping that TID's frames by itself, so
 * there is nothing for the host to set up.  mac80211 still needs the
 * callback, because without one it opens no session at all, and TX_START
 * reporting IMMEDIATE is how mainline's other Realtek drivers express the
 * same thing (rtw88 rtw_ops_ampdu_action()).
 */
static int rtl8192f_ampdu_action(struct ieee80211_hw *hw,
				 struct ieee80211_vif *vif,
				 struct ieee80211_ampdu_params *params)
{
	switch (params->action) {
	case IEEE80211_AMPDU_TX_START:
		return IEEE80211_AMPDU_TX_START_IMMEDIATE;
	case IEEE80211_AMPDU_TX_STOP_CONT:
	case IEEE80211_AMPDU_TX_STOP_FLUSH:
	case IEEE80211_AMPDU_TX_STOP_FLUSH_CONT:
		ieee80211_stop_tx_ba_cb_irqsafe(vif, params->sta->addr,
						params->tid);
		break;
	case IEEE80211_AMPDU_TX_OPERATIONAL:
	case IEEE80211_AMPDU_RX_START:
	case IEEE80211_AMPDU_RX_STOP:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct ieee80211_ops rtl8192f_ops = {
	.tx			= rtl8192f_op_tx,
	.start			= rtl8192f_start,
	.stop			= rtl8192f_stop,
	.add_interface		= rtl8192f_add_interface,
	.remove_interface	= rtl8192f_remove_interface,
	.config			= rtl8192f_config,
	.bss_info_changed	= rtl8192f_bss_info_changed,
	.sta_state		= rtl8192f_sta_state,
	.conf_tx		= rtl8192f_conf_tx,
	.ampdu_action		= rtl8192f_ampdu_action,
	.set_key		= rtl8192f_set_key,
	.sw_scan_start		= rtl8192f_sw_scan_start,
	.sw_scan_complete	= rtl8192f_sw_scan_complete,
	.get_tsf		= rtl8192f_get_tsf,
	.configure_filter	= rtl8192f_configure_filter,
	.wake_tx_queue		= ieee80211_handle_wake_tx_queue,
	.add_chanctx		= ieee80211_emulate_add_chanctx,
	.remove_chanctx		= ieee80211_emulate_remove_chanctx,
	.change_chanctx		= ieee80211_emulate_change_chanctx,
};

irqreturn_t rtl8192f_interrupt(int irq, void *dev_id)
{
	struct rtl8192f_soc_priv *priv = dev_id;
	static u32 rdu_burst;
	u32 hisr0, hisr1;

	/* Two interrupt groups: 0xB0/0xB4 carry RXOK/RDU and the per-
	 * queue TX done bits, 0xB8/0xBC the RX/TX FIFO overflow bits
	 * (halmac_bit2.h 8197F section).  Both status
	 * registers are write-1-to-clear and both must be read/cleared
	 * on every interrupt - leaving HISR1 uncleared turns any of its
	 * enabled events into an interrupt storm.
	 */
	/* The vendor 8197F ISR unconditionally read+write-clears HSISR
	 * (0x5C) on every interrupt (Hal88XXIsr.c 115-124): latched
	 * HSISR bits - GPIO9 hints and, on this SoC, the AXI-bus
	 * exception hint - keep the shared line asserted with no
	 * driver-visible source in HISR0/HISR1, which both hangs the
	 * shared handler and masks spurious detection.  No HSIMR bit is
	 * ever enabled (the cardemu GPIO9 steps are not ported), so
	 * write back exactly the latched bits to clear them.
	 */
	rtl8192f_write32(priv, REG_HSISR, rtl8192f_read32(priv, REG_HSISR));

	hisr0 = rtl8192f_read32(priv, REG_HISR0);
	hisr1 = rtl8192f_read32(priv, REG_HISR1);

	if (!hisr0 && !hisr1)
		return IRQ_NONE;

	/* Vendor InterruptRecognized88XX (Hal88XXIsr.c 92-101) masks the
	 * status word by its own IntMask before writing it back: only the
	 * bits the host claimed may be cleared.  HISR0's DOK bits belong to
	 * the in-core scheduler, so clear group 1 first (HISR1_IND must not
	 * re-latch behind us) and group 0 only where we own the bit.
	 */
	rtl8192f_write32(priv, REG_HISR1, hisr1);
	rtl8192f_write32(priv, REG_HISR0, hisr0 & RTL8192F_HIMR0_INT_MASK);

	/* A beacon slot the scheduler could not put on air is the one
	 * AP-side TX failure this chip reports to the host.
	 */
	if (hisr0 & RTL8192F_HIMR_TXBCN0ERR)
		dev_warn_ratelimited(&priv->pdev->dev,
				     "beacon TX error on mbssid 0\n");

	/* RDU re-latches faster than its W1C clears when the RX hardware
	 * finds no descriptor available (fw-mode RX semantics), so the
	 * line stays asserted and the ISR re-enters endlessly - long
	 * enough to starve timers and the console.  Break the loop: mask
	 * RDU once a burst of RDU-only entries is seen; the rx tasklet's
	 * refill re-arms it.
	 */
	if (hisr0 == RTL8192F_HIMR_RDU) {
		if (rdu_burst == 10000) {
			/* Write the intended mask, do not read-modify-write
			 * it: a read here can land while txrx_start() is
			 * installing its own mask, and writing the stale word
			 * back would put the bits we deliberately keep clear -
			 * above all the per-AC TX-done bits the in-core
			 * scheduler consumes - back into the live register.
			 */
			u32 himr0 = RTL8192F_HIMR0_INT_MASK &
				    ~RTL8192F_HIMR_RDU;

			rtl8192f_write32(priv, REG_HIMR0, himr0);
			dev_err_ratelimited(&priv->pdev->dev,
					    "RDU storm - masking RDU (himr0 0x%08x)\n",
					    himr0);
		}
	} else {
		rdu_burst = 0;
	}

	if (hisr0 & (RTL8192F_HIMR_RXOK | RTL8192F_HIMR_RDU)) {
		/* RDU (RX descriptor unavailable): schedule the rx
		 * tasklet as well - its refill loop re-arms whatever
		 * descriptors are missing .
		 */
		rtl8192f_rx_isr(priv);
	}
	if (hisr1 & RTL8192F_HIMR1_INT_MASK) {
		/* FOVW/TXFOVW: overflow means frames were dropped by
		 * the hardware; kick the rx tasklet so the rings drain
		 * (and the tx side self-heals on the next reap).
		 * TXFOVW/RXERR/TXERR are the branches where the vendor
		 * ISR also read+write-clears REG_TXDMA_STATUS (0x210,
		 * 8192cd_osdep.c 1765-1795 + Hal88XXGen.c 725-728) -
		 * it is W1C too, and leaving it latched keeps the error
		 * source asserted.
		 */
		if (hisr1 & (RTL8192F_HIMR1_TXFOVW |
			     RTL8192F_HIMR1_RXERR |
			     RTL8192F_HIMR1_TXERR)) {
			u32 st = rtl8192f_read32(priv, REG_TXDMA_STATUS);

			rtl8192f_write32(priv, REG_TXDMA_STATUS, st);
		}
		/* The RX side has its own latch.  The vendor ISR clears
		 * REG_RXDMA_STATUS (read + write-back of what was read)
		 * whenever RX fifo overflow or RXERR fired
		 * (8192cd_osdep.c 1997-2002); without it the latched bits
		 * keep the RX error source asserted, which is the same
		 * class of stuck-error interrupt the RDU masking above
		 * deals with.
		 */
		if (hisr1 & (RTL8192F_HIMR1_FOVW |
			     RTL8192F_HIMR1_RXERR)) {
			u32 st = rtl8192f_read32(priv, REG_RXDMA_STATUS);

			if (st)
				rtl8192f_write32(priv, REG_RXDMA_STATUS, st);
		}
		rtl8192f_rx_isr(priv);
	}
	/* TX completion: all DOK bits incl. HIGHDOK (bit 7) - group
	 * addressed frames go out on Q_HIGH and without it the HIGH ring tail
	 * never advances.  The bits are not host-enabled (the in-core
	 * scheduler owns them; enabling them kills the transmitter), so this
	 * dispatch runs on the coattails of the host-owned interrupts.  A
	 * producer that really runs out of slots reclaims on the spot in
	 * rtl8192f_tx().
	 */
	if (hisr0 & RTL8192F_TX_DONE_MASK)
		rtl8192f_tx_isr(priv);

	return IRQ_HANDLED;
}

static int rtl8192f_soc_probe(struct platform_device *pdev)
{
	struct ieee80211_hw *hw;
	struct rtl8192f_soc_priv *priv;
	int ret;
	int i;

	hw = ieee80211_alloc_hw(sizeof(*priv), &rtl8192f_ops);
	if (!hw) {
		dev_err(&pdev->dev, "ieee80211_alloc_hw failed\n");
		return -ENOMEM;
	}

	priv = hw->priv;
	priv->hw = hw;
	priv->pdev = pdev;
	SET_IEEE80211_DEV(hw, &pdev->dev);
	platform_set_drvdata(pdev, hw);

	spin_lock_init(&priv->irq_lock);
	mutex_init(&priv->h2c_mutex);
	mutex_init(&priv->conf_mutex);
	INIT_DELAYED_WORK(&priv->tx_watch, rtl8192f_tx_watch);

	priv->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->regs)) {
		ret = PTR_ERR(priv->regs);
		goto err_free_hw;
	}

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0) {
		ret = priv->irq;
		goto err_free_hw;
	}

	/* SoC-side WiFi clocks: the WLAN clock/function gate, and the
	 * crystal reference the core's analog front end has to be told
	 * about.  Both come from the system-controller clock driver.
	 */
	priv->clk_core = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(priv->clk_core))
		return dev_err_probe(&pdev->dev, PTR_ERR(priv->clk_core),
				     "cannot get WLAN clock\n");
	priv->clk_ref = devm_clk_get(&pdev->dev, "ref");
	if (IS_ERR(priv->clk_ref))
		return dev_err_probe(&pdev->dev, PTR_ERR(priv->clk_ref),
				     "cannot get WiFi reference clock\n");

	ret = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		goto err_free_hw;

	/* The WLAN clock/function gate must be open before ANY WiFi-core
	 * MMIO access - the EFuse controller lives at REG_EFUSE_CTRL in
	 * that window, so opening it has to happen here rather than
	 * inside hw_power_on().  Open it, give the core clocks the same
	 * settle delay that sequence used to apply, then read the EFuse.
	 */
	ret = rtl8192f_wlan_clk_enable(priv);
	if (ret) {
		dev_err(&pdev->dev, "WLAN clock enable failed %d\n", ret);
		goto err_free_hw;
	}
	mdelay(10);

	/* Unconditional WLAN platform reset, once here: the EFuse read below is
	 * the first real access into the core, and a core left wedged by a
	 * previous boot's storm-freeze would deadlock the probe bus on it.
	 * Everything configured after this point may assume a reset core.
	 */
	rtl8192f_platform_reset(priv);

	ret = rtl8192f_efuse_read(priv);
	if (ret) {
		dev_err(&pdev->dev, "rtl8192f_efuse_read failed %d\n", ret);
		goto err_free_hw;
	}

	ret = rtl8192f_read_calib(priv);
	if (ret == -EPROBE_DEFER)
		goto err_free_hw;

	/* Address precedence: a DT property (mac-address / local-mac-address,
	 * which a bootloader or packaging layer can fill in), then an nvmem
	 * cell ("mac-address" from the factory MIB partition in flash), then
	 * the first field of the board efuse record, then the on-die eFuse,
	 * then a random locally administered address.
	 */
	ret = of_get_mac_address(pdev->dev.of_node, priv->mac_addr);
	if (ret == -EPROBE_DEFER)
		goto err_free_hw;
	if (ret) {
		ret = of_get_mac_address_nvmem(pdev->dev.of_node,
					       priv->mac_addr);
		if (ret == -EPROBE_DEFER)
			goto err_free_hw;
	}
	if (ret && priv->has_calib &&
	    is_valid_ether_addr(priv->calib + RTL8192F_CAL_MAC)) {
		ether_addr_copy(priv->mac_addr,
				priv->calib + RTL8192F_CAL_MAC);
		ret = 0;
	}
	if (ret) {
		ret = rtl8192f_efuse_get_macaddr(priv);
		/* -ENOENT means no valid MAC in efuse; take the random address
		 * fallback below rather than failing probe.
		 */
		if (ret && ret != -ENOENT)
			goto err_free_hw;
	}

	if (!is_valid_ether_addr(priv->mac_addr)) {
		/* Loud on purpose: without a DT/nvmem address and with a blank
		 * efuse the interface comes up with a different address on every
		 * boot, which looks like an unstable network rather than a
		 * missing MAC source.
		 */
		eth_random_addr(priv->mac_addr);
		dev_warn(&pdev->dev,
			 "no MAC from DT or efuse, using random address %pM\n",
			 priv->mac_addr);
	}
	SET_IEEE80211_PERM_ADDR(hw, priv->mac_addr);

	/* only the four AC data queues are exposed to
	 * mac80211 - conf_tx()/ac_to_queue() only handle 0..3, so
	 * declaring 6 let mac80211 put cab/P2P-device frames on queues
	 * this driver stops/wakes by a different number.  Beacon, HIGH
	 * (group addressed) and mgmt frames are routed internally.
	 */
	hw->queues = 4;
	/* rx_fill_status() reports RSSI in dBm from the JAGUAR2 phy
	 * status; without SIGNAL_DBM mac80211 marks every BSS signal
	 * invalid and iw scan shows no signal line.
	 */
	ieee80211_hw_set(hw, SIGNAL_DBM);
	/* The scheduler groups the frames waiting in a TID's queue into an
	 * A-MPDU on its own once a block-ack session is up - there is no
	 * per-session register for the host to program.  mac80211 still
	 * refuses to start a session unless AMPDU_AGGREGATION is declared
	 * *and* ieee80211_ops.ampdu_action exists
	 * (___ieee80211_start_tx_ba_session() returns -EINVAL on either
	 * count), so without the pair below every exchange is one MPDU plus
	 * one ACK.
	 */
	ieee80211_hw_set(hw, AMPDU_AGGREGATION);
	/* rtl8192f_tx() pushes the 40-byte TX info descriptor in front
	 * of the frame; reserve headroom for it plus cache alignment
	 * .
	 */
	hw->extra_tx_headroom = ALIGN(RTL8192F_TX_INFO_DESC_SIZE, 8);
	hw->max_listen_interval = 1;
	hw->vif_data_size = 0;
	hw->sta_data_size = sizeof(struct rtl8192f_sta);
	hw->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION) |
				     BIT(NL80211_IFTYPE_AP);

	for (i = 0; i < NUM_NL80211_BANDS; i++)
		if (i == NL80211_BAND_2GHZ)
			hw->wiphy->bands[i] = &rtl8192f_band_2g;

	ret = rtl8192f_hw_power_on(priv);
	if (ret) {
		dev_err(&pdev->dev, "rtl8192f_hw_power_on failed %d\n", ret);
		goto err_free_hw;
	}

	ret = rtl8192f_hw_init(priv);
	if (ret) {
		dev_err(&pdev->dev, "rtl8192f_hw_init failed %d\n", ret);
		goto err_power_off;
	}

	/* No RAM image here: the 8197F WiFi core is a MIPS CPU, so the
	 * rtlwifi 8051 memcpy download flow does not apply.  The core's own
	 * scheduler image is pushed by rtl8192f_fw_download() once the rings
	 * and TX FIFO pages exist.
	 */
	ret = rtl8192f_txrx_alloc(priv);
	if (ret) {
		dev_err(&pdev->dev, "rtl8192f_txrx_alloc failed %d\n", ret);
		goto err_power_off;
	}

	ret = request_irq(priv->irq, rtl8192f_interrupt, IRQF_SHARED,
			  "rtl8192f-soc", priv);
	if (ret)
		dev_err(&pdev->dev, "request_irq failed %d\n", ret);
	if (ret)
		goto err_txrx_free;

	priv->hw_ready = true;

	ret = ieee80211_register_hw(hw);
	if (ret) {
		dev_err(&pdev->dev, "ieee80211_register_hw failed %d\n", ret);
		goto err_irq;
	}

	dev_info(&pdev->dev,
		 "RTL8197F WiFi (RTL8192F-SoC) registered, MAC %pM, irq %d\n",
		 priv->mac_addr, priv->irq);

	return 0;

err_irq:
	free_irq(priv->irq, priv);
	priv->hw_ready = false;
err_txrx_free:
	rtl8192f_txrx_free(priv);
err_power_off:
	rtl8192f_hw_power_off(priv);
err_free_hw:
	ieee80211_free_hw(hw);
	return ret;
}

static void rtl8192f_soc_remove(struct platform_device *pdev)
{
	struct ieee80211_hw *hw = platform_get_drvdata(pdev);
	struct rtl8192f_soc_priv *priv = hw->priv;

	if (priv->hw_ready) {
		ieee80211_unregister_hw(hw);
		free_irq(priv->irq, priv);
	}
	rtl8192f_txrx_free(priv);
	rtl8192f_hw_power_off(priv);
	ieee80211_free_hw(hw);
}

static const struct of_device_id rtl8192f_soc_of_match[] = {
	{ .compatible = "realtek,rtl8197f-wifi" },
	{ }
};
MODULE_DEVICE_TABLE(of, rtl8192f_soc_of_match);

static struct platform_driver rtl8192f_soc_driver = {
	.probe		= rtl8192f_soc_probe,
	.remove		= rtl8192f_soc_remove,
	.driver		= {
		.name		= "rtl8192f-soc",
		.of_match_table	= rtl8192f_soc_of_match,
	},
};
module_platform_driver(rtl8192f_soc_driver);

MODULE_AUTHOR("David Yang <mmyangfl@gmail.com>");
MODULE_DESCRIPTION("Realtek RTL8192F-SoC (RTL8197F integrated) wireless driver");
MODULE_LICENSE("GPL");
