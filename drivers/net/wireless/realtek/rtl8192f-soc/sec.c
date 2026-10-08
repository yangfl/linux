// SPDX-License-Identifier: GPL-2.0-only
/*
 * RTL8192F-SoC mac80211 driver - hardware security engine
 *
 * The 8197F MAC holds up to 64 security CAM entries, each of 8 dwords, and
 * the only way to any of them is the indirect command/data pair at
 * 0x670/0x674 (write) and 0x678 (read).  An entry carries the peer address,
 * the algorithm, the key id and 16 bytes of key material; once the engine is
 * enabled the MAC looks an entry up by the frame's address and key id, adds
 * the IV and the MIC on the way out and removes them on the way in, which is
 * why the TX descriptor only has to name the algorithm.
 */

#include <linux/unaligned.h>

#include "rtl8192f_soc.h"

/* The polling request bit stays set while the engine works on the dword;
 * the vendor bounds the wait at 10000 reads (CAMFindUsable88XX(),
 * Hal88XXGen.c 78-90).
 */
#define CAM_POLL_LIMIT		10000

static int rtl8192f_cam_poll(struct rtl8192f_soc_priv *priv)
{
	int retry = CAM_POLL_LIMIT;

	while (rtl8192f_read32(priv, REG_CAM_CMD) & CAM_CMD_POLLING) {
		if (!--retry)
			return -ETIMEDOUT;
		udelay(1);
	}

	return 0;
}

/* One dword write: data first, then the command word that names the dword
 * and asks for the transfer (CAMProgramEntry88XX(), Hal88XXGen.c 206-226).
 */
static int rtl8192f_cam_write(struct rtl8192f_soc_priv *priv, u16 dword,
			      u32 content)
{
	rtl8192f_write32(priv, REG_CAM_WRITE, content);
	rtl8192f_write32(priv, REG_CAM_CMD,
			 dword | CAM_CMD_POLLING | CAM_CMD_WRITE_ENABLE);

	return rtl8192f_cam_poll(priv);
}

/* Mark an entry free.  A zero in dword 0 clears its valid bit, which is the
 * test CAMFindUsable88XX() uses, and the vendor's empty routine covers all
 * 8 dwords of the entry rather than the 6 that carry content
 * (CAMEmptyEntry88XX(), Hal88XXGen.c 46-63).
 */
static void rtl8192f_cam_empty(struct rtl8192f_soc_priv *priv, u8 entry)
{
	u16 base = entry * CAM_DWORDS_PER_ENTRY;
	int i;

	for (i = 0; i < CAM_DWORDS_PER_ENTRY; i++)
		if (rtl8192f_cam_write(priv, base + i, 0))
			dev_err_ratelimited(&priv->pdev->dev,
					    "CAM %u dword %d write timed out\n",
					    entry, i);
}

/* Install a key.  Dwords go in from the top so the one carrying the address
 * bytes 0-1 and the valid/config word lands last - an entry is never
 * half-visible to the engine (CAMProgramEntry88XX() walks
 * CAM_CONTENT_USABLE_COUNT-1 down to 0, Hal88XXGen.c 204-233).
 */
static int rtl8192f_cam_program(struct rtl8192f_soc_priv *priv, u8 entry,
				const u8 *addr, const struct ieee80211_key_conf *key,
				u16 config)
{
	u16 base = entry * CAM_DWORDS_PER_ENTRY;
	u32 content;
	int i;

	for (i = CAM_DWORDS_IN_USE - 1; i >= 0; i--) {
		if (i == 0) {
			content = FIELD_PREP(GENMASK(31, 24), addr[1]) |
				  FIELD_PREP(GENMASK(23, 16), addr[0]) |
				  config;
		} else if (i == 1) {
			content = get_unaligned_le32(addr + 2);
		} else {
			/* dwords 2..5 carry the 16 key bytes; a shorter key
			 * leaves the remaining bytes zero, which is what the
			 * vendor's zero-padded key128 buffer produces for
			 * WEP (CamAddOneEntry() passes that buffer straight
			 * through, 8192cd_hw.c 19271).
			 */
			const u8 *p = key->key + (i - 2) * 4;

			if ((i - 2) * 4 >= key->keylen) {
				content = 0;
			} else if (key->keylen - (i - 2) * 4 >= 4) {
				content = get_unaligned_le32(p);
			} else {
				u8 tail[4] = { 0 };

				memcpy(tail, p, key->keylen - (i - 2) * 4);
				content = get_unaligned_le32(tail);
			}
		}

		if (rtl8192f_cam_write(priv, base + i, content)) {
			dev_err(&priv->pdev->dev,
				"CAM %u dword %d write timed out\n", entry, i);
			rtl8192f_cam_empty(priv, entry);
			return -ETIMEDOUT;
		}
	}

	return 0;
}

void rtl8192f_sec_enable(struct rtl8192f_soc_priv *priv)
{
	u32 val;

	/* REG_CR is a 16-bit register here (see the CR RMWs in core.c), and
	 * bit9 is inside that half.
	 */
	val = rtl8192f_read16(priv, REG_CR);
	if (!(val & CR_MAC_SEC_EN))
		rtl8192f_write16(priv, REG_CR, val | CR_MAC_SEC_EN);

	/* TXENC | RXDEC | NOSKMC | CHK_KEYID, the set the vendor programs for
	 * the hardware path (CamAddOneEntry() -> HW_VAR_SECURITY_CONFIG,
	 * 8192cd_hw.c 19273-19283).  NOSKMC keeps multicast frames out of the
	 * CAM search: this driver leaves group keys in software, exactly as
	 * the shipped firmware does (set_gkey_to_cam = 0,
	 * 8192cd_security.c 621).
	 */
	val = rtl8192f_read32(priv, REG_SECCFG);
	rtl8192f_write32(priv, REG_SECCFG,
			 val | SEC_CFG_TXENC | SEC_CFG_RXDEC |
			 SEC_CFG_NOSKMC | SEC_CFG_CHK_KEYID);
}

/* The entry this key already owns, or -1 when it has none.
 *
 * Without this, a re-upload (mac80211 does one after every hardware restart)
 * would take a fresh slot each time and leave the previous one programmed and
 * unaccounted for: the pairwise range is CAM_ENTRIES - CAM_GROUP_ENTRIES
 * entries wide, so a client that survived that many stalls would end up with
 * its key handled in software while the orphaned entries still decrypt for
 * addresses that no longer exist.
 */
static int rtl8192f_cam_slot(struct rtl8192f_soc_priv *priv,
			     const struct ieee80211_key_conf *key)
{
	int i;

	for (i = CAM_GROUP_ENTRIES; i < CAM_ENTRIES; i++)
		if (priv->cam_owner[i] == key)
			return i;

	return -1;
}

/* Group keys stay in software, so only a pairwise CCMP key is installed in
 * hardware; anything else returns 1 and mac80211 keeps handling it itself
 * (return 1 for a key is the documented "software crypto for this key"
 * answer - net/mac80211.h "Hardware crypto acceleration").
 */
int rtl8192f_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
		     struct ieee80211_vif *vif, struct ieee80211_sta *sta,
		     struct ieee80211_key_conf *key)
{
	struct rtl8192f_soc_priv *priv = hw->priv;
	u16 config;
	int entry;
	int ret = 0;

	mutex_lock(&priv->conf_mutex);

	if (cmd == DISABLE_KEY) {
		/* Only free a slot this key really owns: key->hw_key_idx
		 * survives a disable, so after an entry has been handed to
		 * another key the number alone can point at that other key.
		 */
		if (key->hw_key_idx < CAM_ENTRIES &&
		    priv->cam_owner[key->hw_key_idx] == key &&
		    test_and_clear_bit(key->hw_key_idx, priv->cam_map)) {
			rtl8192f_cam_empty(priv, key->hw_key_idx);
			priv->cam_owner[key->hw_key_idx] = NULL;
			dev_dbg(&priv->pdev->dev, "CAM %u freed\n",
				key->hw_key_idx);
		}
		goto out;
	}

	if (!sta || key->cipher != WLAN_CIPHER_SUITE_CCMP ||
	    key->keylen != 16) {
		ret = 1;
		goto out;
	}

	if (((struct rtl8192f_sta *)sta->drv_priv)->four_addr) {
		/* The engine cannot be made to encrypt for a four-address
		 * station: with the key in the CAM such a station passes nothing in
		 * either direction, while the same station in three-address form works
		 * (measured).  Receive is not the problem - the descriptor reports
		 * four-address frames decrypted with no ICV error, so the AAD the
		 * engine checks covers the extra address - and the failure is
		 * transmit-only.  There is no knob left for the host: the
		 * header-geometry field of this descriptor family, DW3[4:0]
		 * ("unit: 2 bytes", Hal88XXTxDesc.c 959, 1209) was tried with the
		 * vendor's own encoding and changed nothing on either leg, and the
		 * reference build never writes that field outside the MAC-V2
		 * hardware-shortcut branch.  So the key stays in software for these
		 * stations.  Note that mac80211 cannot move a key that was already
		 * handed over (ieee80211_set_sta_4addr() only calls the driver), so a
		 * station put into four-address form after its key was installed
		 * keeps that key and stays broken until it re-associates.
		 */
		ret = 1;
		goto out;
	}

	entry = rtl8192f_cam_slot(priv, key);
	if (entry < 0)
		entry = find_next_zero_bit(priv->cam_map, CAM_ENTRIES,
					   CAM_GROUP_ENTRIES);
	if (entry >= CAM_ENTRIES) {
		/* Out of pairwise slots: fall back to software for this peer
		 * rather than evicting another peer's key.
		 */
		ret = 1;
		goto out;
	}

	config = CAM_CFG_VALID |
		 (CAM_ALG_CCMP << CAM_CFG_ALG_SHIFT) |
		 FIELD_PREP(CAM_CFG_KEYID_MASK, key->keyidx);

	ret = rtl8192f_cam_program(priv, entry, sta->addr, key, config);
	if (ret)
		goto out;

	set_bit(entry, priv->cam_map);
	priv->cam_owner[entry] = key;
	key->hw_key_idx = entry;

	/* The engine writes the CCMP header into the frame and appends the MIC,
	 * so mac80211 has to leave room for both without filling them in
	 * itself.  Same reason the vendor adds the IV length to the header
	 * segment whenever a frame is hardware-encrypted: "CCMP: 1) iv is CCMP
	 * header, 2) mic for hw encrypt" (SetTxBufferDesc88XX(),
	 * Hal88XXTxDesc.c 1452-1460).
	 */
	key->flags |= IEEE80211_KEY_FLAG_PUT_IV_SPACE |
		      IEEE80211_KEY_FLAG_RESERVE_TAILROOM;
	rtl8192f_sec_enable(priv);
	dev_dbg(&priv->pdev->dev, "CCMP key for %pM -> CAM %u (0x%04x)\n",
		sta->addr, entry, config);

out:
	mutex_unlock(&priv->conf_mutex);
	return ret;
}
