// SPDX-License-Identifier: GPL-2.0-only
/*
 * RTL8192F-SoC mac80211 driver - rate encoding, rx status, MACID.

 * This file implements:
 * - the mapping from mac80211 tx rate indices to the Realtek
 * descriptor TX_RATE field (rtl8192f_tx_fill_rate and friends),
 * - the mac80211 rx_status fill from the RX descriptor and the
 * prepended PHY status (rtl8192f_rx_fill_status),
 * - driver-side MACID allocation (rtl8192f_alloc_macid/free).

 * Rate encoding conclusions (RTL8192C lineage descriptor rate tables):

 * The descriptor TX_RATE field uses the same linear table as the
 * vendor RXD RxMcs field (8192cd_hw.h enum RX_RxMcsMask 0x03F, DWORD3
 * bits 5:0) and rtlwifi's enum DESC_RATE (rtlwifi/wifi.h):

 * 0x00-0x03  CCK 1/2/5.5/11M      (DSSS, 0x02/0x03 long preamble)
 * 0x04-0x0B  OFDM 6/9/12/18/24/36/48/54M
 * 0x0C-0x13  HT MCS0-7 (1SS), 0x0C + mcs

 * The vendor "HT_RATE_ID = 0x80" (8192cd_hw.h) is a *software-only*
 * rate identifier space (used in the rate adaptive table and
 * pfrinfo->rx_rate), NOT the TXD TX_RATE field value; the hardware
 * field is the linear 0x0C+mcs encoding, confirmed by rtlwifi
 * rtl8192ee struct tx_desc (txrate:6 + shortgi:1 in dword5) and
 * rtlwifi/wifi.h DESC_RATEMCS0 = 0x0c.  This matches core.c's
 * rtl8192f_rates hw_value for the legacy entries (0x00-0x0B).

 * PHY status layout (RCR_APP_PHYST_RXFF prepends it before the MPDU):
 * the 8197F reports the phydm JAGUAR2 format (phydm_hwconfig.c,
 * phydm_GetRxPhyStatusType0/Type1) - NOT the gen-1 8192C layout the
 * earlier revision of this file assumed:

 * Type 0 (CCK, PHY_STATUS_RPT_JAGUAR2_TYPE0):
 *  byte  0     page_num
 *  byte  1     pwdb
 *  byte  2     gain:6, rsvd:1, trsw:1
 *  byte  4     rxsc:4, agc_table:4
 *  byte  6     channel
 *  byte 12     signal_quality
 *  byte 13     vga:5, lna_l:3
 *  byte 14     bb_power:6, rsvd:1, lna_h:1
 *
 * Type 1 (OFDM/HT, PHY_STATUS_RPT_JAGUAR2_TYPE1):
 *  byte  0     page_num
 *  byte  1-4   pwdb[4]  (per-path, dB domain, report = pwdb - 110)
 *  byte  5     l_rxsc:4, ht_rxsc:4
 *  byte  6     channel
 */

#include "rtl8192f_soc.h"

/*
 * TX descriptor rate fields.  The 88XX TX info descriptor carries
 * USE_RATE in W3 bit8 and DATARATE in W4 bits 6:0 (mainline rtw88
 * tx.h 41/46, RTW_TX_DESC_W3_USE_RATE / RTW_TX_DESC_W4_DATARATE) -
 * the same bit positions the gen-2 layout used, so rtl8192f_tx_fill_
 * rate() works unchanged on the w-array of the 40B descriptor.
 * Duplicated here as local macros because txrx.c keeps its TXD bit
 * definitions file-private.
 */
#define RATED_TXD_USE_RATE	BIT(8)		/* W3: fixed rate mode */
#define RATED_TXD_RATE		GENMASK(6, 0)	/* W4 DATARATE */

/*
 * RX info descriptor (24B at the start of the RX DMA buffer) DW3 rate
 * field, 88XX/8197F layout: RX_RATE is bits 6:0 (7 bits,
 * halmac_rx_desc_nic.h 80 GET_RX_DESC_RX_RATE).  Values 0-11 are the
 * linear legacy table, 12+ are HT (mcs = rate - 12).  There are no
 * SGI/BW fields in the 8197F RX info descriptor - the previous W3
 * bits 8/9 reads hit BSSID_FIT/HTC; SGI/BW would
 * have to come from the PHY status (TODO, 20 MHz/no-SGI assumed).
 */
#define RATED_RX_RATE		GENMASK(6, 0)	/* DW3: linear rate */

/*
 * Map a mac80211 band bitrates[] index to the descriptor rate value.
 * core.c declares rtl8192f_rates with hw_value equal to the index and
 * identical encoding (0x00-0x03 CCK, 0x04-0x0B OFDM), so this is a
 * validated identity map.
 */
u32 rtl8192f_legacy_rate_to_hw(int idx, bool cck)
{
	if (idx < 0 || idx > 11)
		return cck ? 0x00 : 0x0b;	/* 1M / 54M fallback */

	return (u32)idx;
}

/*
 * Map an HT MCS index to the dword5 rate field payload: bits 5:0 hold
 * the linear rate 0x0C + mcs only.  Short GI is *not* part of that
 * value: DATARATE is TX_DW4_DATARATE_MSK 0x7f wide (HalComTXDesc.h),
 * so a bit-6 flag here would simply produce an out-of-range rate index
 * (0x4c..0x53) that the in-core scheduler cannot use - which is what
 * the first port did and why HT unicast never reached the air.  The GI
 * request lives in W5 DATA_SHORT, set by rtl8192f_tx().  See the file
 * header for why this is 0x0C+mcs and not the vendor software
 * HT_RATE_ID 0x80+mcs.
 */
u8 rtl8192f_mcs_to_hw(u8 mcs)
{
	return 0x0c + min_t(u8, mcs, 7);
}

void rtl8192f_tx_fill_rate(struct rtl8192f_soc_priv *priv, __le32 *desc,
			   struct ieee80211_tx_info *info,
			   struct ieee80211_sta *sta)
{
	struct ieee80211_tx_rate *rate = &info->control.rates[0];
	u32 d3, d4;
	u32 hw;

	/*
	 * Frames without rate control information (management / no-RC
	 * frames keep the default rates[0].idx == 0) are sent at a
	 * basic rate: index 0 is 1M CCK in core.c's band table, which
	 * is the desired mgmt basic rate on 2.4GHz (rtlwifi's mgmt
	 * path likewise uses DESC_RATE1M).  Explicitly map unknown
	 * indices to 1M as well.
	 */
	if (rate->flags & IEEE80211_TX_RC_MCS) {
		bool ht_ok = sta && sta->deflink.ht_cap.ht_supported;

		if (!ht_ok || rate->idx < 0 || rate->idx > 7)
			hw = 0x0b;	/* 54M fallback */
		else
			hw = rtl8192f_mcs_to_hw((u8)rate->idx);
	} else if (rate->idx < 0 || rate->idx > 11) {
		hw = 0x00;	/* 1M fallback */
	} else if (rate->idx < 4) {
		hw = rtl8192f_legacy_rate_to_hw(rate->idx, true);
	} else {
		hw = rtl8192f_legacy_rate_to_hw(rate->idx, false);
	}

	/* dword3: fixed rate (USE_RATE), dword4: the rate index alone */
	d3 = le32_to_cpu(desc[3]);
	d3 |= RATED_TXD_USE_RATE;
	desc[3] = cpu_to_le32(d3);

	d4 = le32_to_cpu(desc[4]);
	d4 &= ~RATED_TXD_RATE;
	d4 |= hw & RATED_TXD_RATE;
	desc[4] = cpu_to_le32(d4);
}

/* desc points at the 24-byte 88XX RX info descriptor at the start of
 * the RX DMA buffer (HalHeader/HalComRXDesc.h layout); phy_status at
 * the appended PHY status right behind it (RXINFO 24B + drv_info
 * offset, the position 8192cd_rx.c hands to translate_rssi_sq).
 */
/*
 * CCK RSSI lookup for the old-AGC report (0xa9c bit17 = 0 after the
 * A-cut workaround): RSSI = base[lna_l] + diff_para - 2 * vga, vendor
 * odm_CCKRSSI_8197F() (phydm_rtl8197f.c 31+).  diff_para is +7 unless
 * the board combines external TRSW/LNA/PA with PackageType 1; the
 * default branch is used here.
 */
static const s8 rtl8192f_cck_rssi_base[8] = {
	2, -2, -12, -18, -26, -32, -42, -52
};

/* Width an HT frame was received at.  The RX info descriptor has no
 * bandwidth bit, but the type-1 PHY status names the half the frame arrived
 * on (byte 5's ht_rxsc nibble) next to the analyser's own rf_mode (byte 15
 * [5:4]): 20 MHz while the analyser is narrow or the frame sits in a single
 * half, 40 MHz when it spans both.  Legacy OFDM has no 40 MHz form, so only
 * HT frames need this.  Vendor phydm_hwconfig.c 3270-3289 with
 * PHY_STATUS_RPT_JAGUAR2_TYPE1.
 */
static enum rate_info_bw rtl8192f_rx_bw(const u8 *p)
{
	u8 ht_rxsc = p[5] >> 4;

	if (((p[15] >> 4) & 0x3) == 0 || ht_rxsc == 1 || ht_rxsc == 2)
		return RATE_INFO_BW_20;

	return RATE_INFO_BW_40;
}

void rtl8192f_rx_fill_status(struct rtl8192f_soc_priv *priv, __le32 *desc,
			     void *phy_status,
			     struct ieee80211_rx_status *status)
{
	u32 w3 = le32_to_cpu(desc[3]);
	u32 rate = w3 & RATED_RX_RATE;
	u8 *p = phy_status;

	status->band = NL80211_BAND_2GHZ;
	/* status->freq is filled by the caller from the chandef */

	if (rate >= 0x0c) {
		/* HT: the linear rate field continues at 0x0C + mcs,
		 * vendor 8192cd_rx.c 6916-6926 (rx_rate = HT_RATE_ID +
		 * (RX_RATE - 12)); there is no separate HT flag in the
		 * 88XX W3.
		 */
		u8 mcs = rate > 0x0c ? rate - 0x0c : 0;

		/* Clamp to what mac80211 accepts (rx.c WARNs >76 and
		 * drops); garbage PHY status under the mask ROM can put
		 * anything in the 7-bit field.
		 */
		if (mcs > 76)
			mcs = 76;

		status->encoding = RX_ENC_HT;
		status->rate_idx = mcs;
		status->bw = rtl8192f_rx_bw(p);
		/* Was falling through into the legacy tail, which
		 * relabelled every HT frame LEGACY with
		 * rate_idx = rate - 0x04 (up to 123) - one mac80211
		 * WARN_ON per received frame, flooding the console
		 * from softirq (the "console dies, system lives"
		 * phenomenon).  HT frames end here.
		 */
		return;
	} else if (rate < 0x04) {
		/* CCK: JAGUAR2 type 0 report, old-AGC RSSI from
		 * lna_l/vga (vendor phydm_GetRxPhyStatusType0 with
		 * odm_CCKRSSI_8197F).
		 */
		u8 lna_l = (p[13] >> 5) & 0x7;
		u8 vga = p[13] & 0x1f;

		status->encoding = RX_ENC_LEGACY;
		status->rate_idx = rate;
		status->bw = RATE_INFO_BW_20;

		status->signal = (s8)(rtl8192f_cck_rssi_base[lna_l] + 7 -
				      2 * vga);
		return;
	}

	/* Legacy OFDM: JAGUAR2 type 1 report, per-path pwdb at bytes 1-4;
	 * single receive path (path A), vendor phydm_GetRxPhyStatusType1
	 * (rx_path_pwr_db = pwdb - 110).  No short-GI report exists for this
	 * core, so the 20 MHz rate stays the reported rate.
	 */
	status->encoding = RX_ENC_LEGACY;
	/* hw_value == array index in the band table (core.c), and the
	 * vendor treats the linear field as a direct index too
	 * (8192cd_util.h:1229) - no 0x04 subtraction.  Clamp against
	 * the untrusted 7-bit field anyway.
	 */
	status->rate_idx = min_t(u8, rate, 11);
	status->bw = RATE_INFO_BW_20;	/* OFDM has no 40 MHz form */

	status->signal = (s8)p[1] - 110;
}

/* ------------------------------------------------------------------ */
/* Rate-adaptation table id                                           */
/* ------------------------------------------------------------------ */

/*
 * The id that names the firmware's rate ladder for a peer: RA_MASK
 * payload byte 1 bits[4:0] and TX descriptor W1 RATE_ID bits[20:16] both
 * carry it, and they have to agree.  It is a table selector, not a rate.
 *
 * The SDK has two mappings for it and this mask ROM follows the older one,
 * MRateIdxToARFRId88XX() (Hal88XXFirmware.c, table at 1497-1540), not
 * phydm_rate_id_mapping() (phydm_rainfo.c 1811-1918): sending the
 * phydm answer for an HT 1T1R peer - 3, which under the ARFR table
 * is RATR_INX_WIRELESS_NB - stopped AP-to-client data being
 * delivered entirely (the AP took nine ARP requests and answered
 * none), while id 1 works in the same setup.
 * This radio is 1T1R on 2.4GHz only, so three entries can apply: 1 for an
 * HT peer (NGB), 6 for an 11g peer (GB) and 8 for a CCK-only one (B).
 * Six is also what the vendor's live management descriptors carry, which
 * is why frames with no peer use it.
 */
u8 rtl8192f_ra_rate_id(const struct ieee80211_sta *sta)
{
	if (!sta)
		return 6;
	if (sta->deflink.ht_cap.ht_supported)
		return 1;
	if (sta->deflink.supp_rates[NL80211_BAND_2GHZ] & GENMASK(11, 4))
		return 6;
	return 8;
}

/* ------------------------------------------------------------------ */
/* MACID allocation (0 = broadcast/multicast, reserved)                */
/* ------------------------------------------------------------------ */

int rtl8192f_alloc_macid(struct rtl8192f_soc_priv *priv)
{
	unsigned long flags;
	int macid;

	spin_lock_irqsave(&priv->irq_lock, flags);

	macid = find_next_zero_bit(priv->macid_map, 32, 1);
	if (macid >= 32) {
		spin_unlock_irqrestore(&priv->irq_lock, flags);
		return -ENOENT;	/* table full */
	}
	__set_bit(macid, priv->macid_map);

	spin_unlock_irqrestore(&priv->irq_lock, flags);

	/* TODO: program REG_MACID (0x0610 + 8 * macid) and the MACID
	 * CAM (HMAVR) entry for this peer.
	 */
	return macid;
}

void rtl8192f_free_macid(struct rtl8192f_soc_priv *priv, u8 macid)
{
	unsigned long flags;

	if (macid == 0 || macid >= 32)
		return;

	spin_lock_irqsave(&priv->irq_lock, flags);
	__clear_bit(macid, priv->macid_map);
	spin_unlock_irqrestore(&priv->irq_lock, flags);
}
