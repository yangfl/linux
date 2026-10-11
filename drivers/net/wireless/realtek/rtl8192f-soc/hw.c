// SPDX-License-Identifier: GPL-2.0-only
/*
 * RTL8192F-SoC (RTL8197F integrated WiFi) - power, MAC/BB/RF init,
 * channel switching and EFuse access.

 * Sequences are ported from:
 * - Realtek vendor driver rtl8192cd (RTL8197_3411D_2 SDK), 8192cd_hw.c /
 * 8192cd_osdep.c: SoC system-controller gating for the 8197F WiFi core,
 * gen-2.5 EFuse read algorithm.
 * - phydm rtl8197f: phydm_hal_api8197f.c RF/BB access and channel switch.
 * - rtl8xxxu 8192f.c: WiFi-core (gen-2.5) power-up state machine.
 */

#include <linux/etherdevice.h>

#include <linux/module.h>
#include <linux/nvmem-consumer.h>

#include "rtl8192f_soc.h"

/* Register/bit macros live in rtl8192f_soc.h - single source of
 * truth.
 */

/* ------------------------------------------------------------------ */
/* RF (HSSI) access - path A only, RTL8197F is 1T1R                    */
/* phydm_hal_api8197f.c: offset_writeRF[2] = {0x840, 0x844},           */
/* offset_readRF[2] = {0x2800, 0x2c00}                                 */
/* ------------------------------------------------------------------ */

#define RTL8192F_RF_WRITE_A	RTL8192F_HSSI_R1	/* 0x840 */
#define RTL8192F_RF_WRITE_B	RTL8192F_HSSI_R2	/* 0x844 (unused, 1T1R) */
#define RTL8192F_RF_READ_A	0x2800			/* + (reg << 2) */
#define RTL8192F_RF_READ_B	0x2c00			/* (unused, 1T1R) */

/* RF register 0x18 (RF6052_REG_MODE_AG): band + central channel */
#define RF_REG_MODE_AG		0x18
#define RF_MODE_AG_BAND_MASK	(BIT(18) | BIT(17))	/* 2.4G band select */

/* ------------------------------------------------------------------ */
/* EFuse geometry macros (EFUSE_MAX_* / EFUSE_MAP_LEN /                */
/* EFUSE_MAP_MAC_OFFSET) live in rtl8192f_soc.h.                       */
/* ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */

static int rtl8192f_poll_bit(struct rtl8192f_soc_priv *priv, u16 addr,
			     u32 mask, u32 expect, u32 loops)
{
	u32 val;

	do {
		val = rtl8192f_read32(priv, addr);
		if ((val & mask) == expect)
			return 0;
		udelay(10);
	} while (--loops);

	dev_err(&priv->pdev->dev,
		"poll_bit timeout: addr 0x%03x val 0x%x (mask 0x%x expect 0x%x)\n",
		addr, val, mask, expect);
	return -EBUSY;
}

/* ------------------------------------------------------------------ */
/* Power on / off                                                      */
/* ------------------------------------------------------------------ */

/* Core power-up follows the vendor 8197F sequences exactly:
 * - WlanHAL/RTL88XX/RTL8197F/Hal8197FPwrSeqCmd.h
 *     RTL8197F_TRANS_96FSOC_POWER_ON (PMC handshake, 0x02/0x03 = 0xff)
 *     RTL8197F_TRANS_CARDEMU_TO_ACT  (ALL_MSK steps only; the USB/SDIO
 *     LDO and GPIO-wakeup steps do not apply on the SoC bus)
 * - InitPON8197F (Hal8197FGen.c 155-177): RSV_CTRL all-unlock first,
 *     then AFE_CTRL5 (0x94) REF_SEL per the SoC XTAL frequency.
 * The previous rtl8xxxu 8192f USB hybrid never performed the PMC
 * handshake and left RSV_CTRL/0x02/0x03 unprogrammed .
 */

/* The crystal the SoC straps to the core arrives as the "ref" clock
 * (the system controller reads the same strap bit the vendor does:
 * 1 = 40 MHz, 0 = 25 MHz).  REF_SEL: 1 = 25 MHz, 0 = 40 MHz
 * (InitPON8197F).
 */
#define REG_AFE_CTRL5		0x0094
#define AFE5_REF_SEL_SHIFT	25
#define AFE5_REF_SEL_MASK	GENMASK(28, 25)

static int rtl8192f_pmc_ready(struct rtl8192f_soc_priv *priv, u16 addr,
			      u32 mask, u32 expect, u32 loops)
{
	u32 val;

	do {
		val = rtl8192f_read8(priv, addr);
		if ((val & mask) == expect)
			return 0;
		udelay(10);
	} while (--loops);

	dev_err(&priv->pdev->dev,
		"pmc_ready timeout: addr 0x%03x val 0x%x (mask 0x%x expect 0x%x)\n",
		addr, val, mask, expect);
	return -ETIMEDOUT;
}

static int rtl8192f_96fsoc_power_on(struct rtl8192f_soc_priv *priv)
{
	int ret;

	/* 0xa0[26:24] = 0 to prepare for checking HPON PMC */
	rtl8192f_write8(priv, 0xa3,
			rtl8192f_read8(priv, 0xa3) & ~(BIT(0) | BIT(1) | BIT(2)));

	/* HPON PMC ready: 0xa0[7:4] == 4'b0010, 0xa0[9:8] == 2'b01 */
	ret = rtl8192f_pmc_ready(priv, 0xa0, 0xf0, BIT(5), 10000);
	if (ret)
		return ret;
	ret = rtl8192f_pmc_ready(priv, 0xa1, 0x3, BIT(1), 10000);
	if (ret)
		return ret;

	/* 0x04[8] = 1: enable MAC PMC (bit 8 = byte at FSMCO+1) */
	rtl8192f_write8(priv, REG_APS_FSMCO + 1,
			rtl8192f_read8(priv, REG_APS_FSMCO + 1) | BIT(0));
	/* 0x04[16] = 1: release WLON reset (bit 16 = byte at FSMCO+2) */
	rtl8192f_write8(priv, REG_APS_FSMCO + 2,
			rtl8192f_read8(priv, REG_APS_FSMCO + 2) | BIT(0));
	/* MAC PMC ready when 0x04[8] self-clears to 0 */
	ret = rtl8192f_pmc_ready(priv, REG_APS_FSMCO + 1, BIT(0), 0, 10000);
	if (ret)
		return ret;

	/* 0xa0[26:24] = 4 for the second PMC check */
	rtl8192f_write8(priv, 0xa3,
			(rtl8192f_read8(priv, 0xa3) &
			 ~(BIT(0) | BIT(1) | BIT(2))) | BIT(2));

	/* HPON PMC settled: 0xa0[7:0] == 0, 0xa0[9:8] == 2'b01 */
	ret = rtl8192f_pmc_ready(priv, 0xa0, 0xff, 0, 10000);
	if (ret)
		return ret;
	ret = rtl8192f_pmc_ready(priv, 0xa1, 0x3, BIT(0), 10000);
	if (ret)
		return ret;

	/* 0x02[7:0] = 0xff, 0x03[7:0] = 0xff: enable all functions */
	rtl8192f_write8(priv, REG_SYS_ISO_CTRL, 0xff);
	rtl8192f_write8(priv, REG_SYS_ISO_CTRL + 1, 0xff);

	return 0;
}

/* CARDEMU_TO_ACT, interface-independent steps */
static int rtl8192f_cardemu_to_act(struct rtl8192f_soc_priv *priv)
{
	int ret;

	/* disable SW LPS 0x04[10] and WLSUS_EN 0x04[12:11] */
	rtl8192f_write16(priv, REG_APS_FSMCO,
			 rtl8192f_read16(priv, REG_APS_FSMCO) &
			 ~(APS_FSMCO_SW_LPS | APS_FSMCO_HW_SUSPEND |
			   APS_FSMCO_PCIE));

	/* wait till 0x04[17] = 1 (power ready) */
	ret = rtl8192f_poll_bit(priv, REG_APS_FSMCO, BIT(17), BIT(17), 10000);
	if (ret)
		return ret;

	/* release WLON reset 0x04[16] = 1 (bit 16 = byte at FSMCO+2) */
	rtl8192f_write8(priv, REG_APS_FSMCO + 2,
			rtl8192f_read8(priv, REG_APS_FSMCO + 2) | BIT(0));

	/* disable HWPDN 0x04[15] = 0, disable WL suspend 0x04[12:11] = 0 */
	rtl8192f_write16(priv, REG_APS_FSMCO,
			 rtl8192f_read16(priv, REG_APS_FSMCO) &
			 ~(APS_FSMCO_HW_POWERDOWN | APS_FSMCO_HW_SUSPEND |
			   APS_FSMCO_PCIE));

	/* 0x04[8] = 1 then poll until it clears (MAC on, self-clearing) */
	rtl8192f_write8(priv, REG_APS_FSMCO + 1,
			rtl8192f_read8(priv, REG_APS_FSMCO + 1) | BIT(0));
	ret = rtl8192f_pmc_ready(priv, REG_APS_FSMCO + 1, BIT(0), 0, 10000);
	if (ret)
		return ret;

	/* 0x73[2] = 1 */
	rtl8192f_write8(priv, 0x73,
			rtl8192f_read8(priv, 0x73) | BIT(2));

	/* The 0x49/0x63/0x62/0x58/0x5A GPIO9 + HSIMR steps of
	 * RTL8197F_TRANS_CARDEMU_TO_ACT are deliberately NOT ported: on
	 * the 8197F SoC the vendor never runs that table - it is
	 * commented out of rtl8197F_card_enable_flow
	 * (Hal8197FPwrSeqCmd.c 52-59) and 96FSOC_POWER_ON carries none
	 * of it (Hal8197FPwrSeqCmd.h 169-182).  Enabling the HSIMR bits
	 * without ever clearing REG_HSISR (0x5C) in the ISR latches an
	 * interrupt group nothing services.
	 */

	return 0;
}

/*
 * WLAN platform reset: 0x1080 bit 16 is the platform-level reset line of this
 * core, and the vendor pulses it in ConfigBeforeDLFW ("WLAN platform reset
 * before download fw, or 97F will have platform err INT").  Drop it, raise it
 * again, and let the core settle, so each bring-up starts from a reset core
 * rather than from whatever the previous stage - boot code or a storm-frozen
 * MAC - left behind.  Called from the two configuration entry points: probe
 * (before the first EFuse access) and hw_power_on().
 */
void rtl8192f_platform_reset(struct rtl8192f_soc_priv *priv)
{
	rtl8192f_write32(priv, 0x1080,
			 rtl8192f_read32(priv, 0x1080) & ~BIT(16));
	rtl8192f_write32(priv, 0x1080,
			 rtl8192f_read32(priv, 0x1080) | BIT(16));
	mdelay(10);
}

int rtl8192f_hw_power_on(struct rtl8192f_soc_priv *priv)
{
	u32 afe5;
	int ret;

	/* Bring the core up from a reset state: this is the configuration
	 * entry point, called by probe and by .start().
	 */
	rtl8192f_platform_reset(priv);

	/* InitPON8197F: unlock all register access windows first - the
	 * RSV_CTRL reset default locks the 0x00-0x16/0x20 area and the
	 * 0x02/0x03 writes below would be dropped silently.
	 */
	rtl8192f_write8(priv, REG_RSV_CTRL, 0x00);

	/* The 88xx SYS_CLKR/ISO register is 0x70, not the 92c 0x08 that
	 * rtl8192f_efuse_power_on() touches below, and bit26 of it
	 * (vendor HalComBit.h BIT_AFE_PORT2_ISO) isolates the transmit
	 * analog path.  It comes up set out of the bootcode; while set,
	 * every transmission completes in the MAC - HISR0 TXBCN0OK
	 * latches - and nothing reaches the air.  The shipped driver runs
	 * a radiating AP with this bit clear.
	 */
	rtl8192f_write32(priv, REG_SYS_CLKR_88XX,
			 rtl8192f_read32(priv, REG_SYS_CLKR_88XX) &
			 ~SYS_CLKR88_AFE_PORT2_ISO);

	/* REF_SEL from the crystal the SoC straps to the core */
	afe5 = rtl8192f_read32(priv, REG_AFE_CTRL5) & ~AFE5_REF_SEL_MASK;
	if (clk_get_rate(priv->clk_ref) >= 40000000)
		/* 40 MHz crystal: REF_SEL = 0 */
		rtl8192f_write32(priv, REG_AFE_CTRL5, afe5);
	else
		/* 25 MHz crystal: REF_SEL = 1 */
		rtl8192f_write32(priv, REG_AFE_CTRL5,
				 afe5 | (1U << AFE5_REF_SEL_SHIFT));

	ret = rtl8192f_96fsoc_power_on(priv);
	if (ret)
		return ret;

	ret = rtl8192f_cardemu_to_act(priv);
	if (ret)
		return ret;

	rtl8192f_write16(priv, REG_CR, 0);

	/* Only the protocol/scheduler blocks come up here.  Every DMA
	 * enable (HCI_TXDMA/HCI_RXDMA/TXDMA/RXDMA/MACTXEN/MACRXEN) is
	 * deferred to rtl8192f_txrx_start(): the vendor sequence programs
	 * all ring base addresses (PrepareRXBD88XX/PrepareTXBD88XX,
	 * 8192cd_osdep.c 3629-3636, "it cannot be moved arbitrarily")
	 * and the BD counts BEFORE opening CR (InitMAC88XX, Hal88XXGen.c
	 * 5056-5059).  Turning the DMA engines on in probe - before
	 * REG_RXQ_RXBD_DESA (0x338) / REG_RX_RXBD_NUM (0x382) /
	 * REG_HCI_CTRL (0x300) are ever written - lets RXDMA fetch
	 * descriptors from an unprogrammed (reset-default) base address
	 * as soon as the BB produces frames and the RX FIFO has space.
	 */
	rtl8192f_write16(priv, REG_CR, CR_PROTOCOL_EN | CR_SCHEDULE_EN);

	return 0;
}

void rtl8192f_hw_power_off(struct rtl8192f_soc_priv *priv)
{
	/* ACT_TO_CARDEMU (Hal8197FPwrSeqCmd.h), not a bare HWPDN set:
	 * turn the RF off (0x1F = 0), release WLON reset, then turn the
	 * MAC off via the HW state machine 0x04[9] and poll it clear.
	 */
	rtl8192f_write8(priv, REG_RF_CTRL, 0);

	rtl8192f_write16(priv, REG_APS_FSMCO,
			 rtl8192f_read16(priv, REG_APS_FSMCO) |
			 APS_FSMCO_WLON_RESET);

	rtl8192f_write16(priv, REG_APS_FSMCO,
			 rtl8192f_read16(priv, REG_APS_FSMCO) |
			 APS_FSMCO_MAC_OFF);
	rtl8192f_poll_bit(priv, REG_APS_FSMCO, APS_FSMCO_MAC_OFF, 0, 10000);

	rtl8192f_write16(priv, REG_CR, 0);

	rtl8192f_wlan_clk_disable(priv);
}

/* Open the SoC-side WLAN clock/function gate before touching the core
 * MMIO window, and close it again on the way down.  This has to be
 * callable from probe *before* any core register access (the EFuse read
 * walks the WiFi-core window at REG_EFUSE_CTRL), so it is decoupled
 * from rtl8192f_hw_power_on().
 */
int rtl8192f_wlan_clk_enable(struct rtl8192f_soc_priv *priv)
{
	return clk_prepare_enable(priv->clk_core);
}

void rtl8192f_wlan_clk_disable(struct rtl8192f_soc_priv *priv)
{
	clk_disable_unprepare(priv->clk_core);
}

/* ------------------------------------------------------------------ */
/* MAC / BB / RF init                                                  */
/* ------------------------------------------------------------------ */

/* The vendor init tables carry no zero-address terminator: the first
 * MAC_REG row is {0x000, 0x92} and the last RadioA row is {0x00,
 * 0x35df5}, so sentinel loops skipped whole tables .
 * Iterate over the array length instead.  Address 0xffe rows are the
 * vendor 50 ms settling-delay markers (phydm_regconfig8197f.c
 * odm_ConfigRFReg_8197F()); in the RF table they are stored as 0xfe
 * (the register-value struct keeps an 8-bit address).
 */
static void rtl8192f_apply_mac_table(struct rtl8192f_soc_priv *priv,
				     const struct rtl8192f_reg8val *table,
				     size_t len)
{
	size_t i;

	for (i = 0; i < len; i++)
		rtl8192f_write8(priv, table[i].addr, table[i].val);
}

static void rtl8192f_apply_bb_table(struct rtl8192f_soc_priv *priv,
				    const struct rtl8192f_reg32val *table,
				    size_t len)
{
	size_t i;

	for (i = 0; i < len; i++)
		rtl8192f_write_bb_reg(priv, table[i].addr, table[i].val);
}

/* phydm-style mask RMW on a BB register (ODM_SetBBReg semantics). */
static void rtl8192f_rmw_bb(struct rtl8192f_soc_priv *priv, u16 addr,
			    u32 mask, u32 val)
{
	u32 reg = rtl8192f_read_bb_reg(priv, addr);

	rtl8192f_write_bb_reg(priv, addr, (reg & ~mask) | (val & mask));
}

/* Per-rate transmit power: the BB TxAGC codewords, one byte per rate,
 * 0 being the lowest step of the PA ladder and 0x3f the highest.
 *
 * The vendor composes these on every channel switch - SwChnl() calls
 * SetTxPowerLevel() (8192cd_hw.c 7461, which branches for 8197F at :5449)
 * and that runs PHYSetOFDMTxPower88XX_N() / PHYSetCCKTxPower88XX_N() - from
 * a per-channel level kept in the board's flash MIB plus a per-rate ramp
 * taken from this core's power-gain (PG) table, every sum clamped by
 * POWER_RANGE_CHECK() (8192cd_cfg.h 321).  Where the board carries no level
 * for a channel both functions fall back to a flat default instead: 0x28
 * for the OFDM and HT groups, 0x24 for the CCK fields (Hal88XXPhyCfg.c
 * 4326, 4677).  The two paths are the same code with a different level, so
 * both are implemented here and the level comes from nvmem cells.
 *
 * The BB init table cannot supply any of this: it carries zeros at
 * 0xe00..0xe1c (halhwimg8197f_bb.c 5230-5235) precisely because the vendor
 * overwrites the group at run time, so a driver that leaves the table value
 * alone drives every frame from the bottom of the ladder.
 */

/* The PG ramp in codeword steps, high byte to low byte of each register,
 * rebased against the reference rate of its group the way the vendor
 * rebases the PG table after reading it back (8192cd_hw.c 13009-13035:
 * OFDM against 54M, MCS0-7 against MCS7, MCS8-15 against MCS15, CCK against
 * 11M).  Source: WlanHAL/Data/8197F/PHY_REG_PG_8197Fmp.txt.
 */
static const s8 rtl8192f_pg_ofdm[8] = {
	2, 3, 3, 3,		/* 0xe00: 18, 12, 9, 6 Mbps */
	0, 1, 1, 2,		/* 0xe04: 54, 48, 36, 24 Mbps */
};

static const s8 rtl8192f_pg_ht[16] = {
	2, 3, 3, 4,		/* 0xe10: MCS3, MCS2, MCS1, MCS0 */
	0, 1, 1, 2,		/* 0xe14: MCS7..MCS4 */
	2, 3, 3, 4,		/* 0xe18: MCS11..MCS8 */
	0, 1, 1, 2,		/* 0xe1c: MCS15..MCS12 */
};

/* Every CCK rate carries the same PG codeword (20), so the CCK ramp is flat
 * and only the per-channel level moves it.
 */

#define RTL8192F_TXPWR_OFDM_DEFAULT	0x28
#define RTL8192F_TXPWR_CCK_DEFAULT	0x24

/* POWER_RANGE_CHECK() */
static u8 rtl8192f_txpwr_clamp(int val)
{
	if (val > 0x3f)
		return 0x3f;
	if (val < 0)
		return 0;
	return val;
}

static u8 rtl8192f_txpwr_step(u8 level, s8 ramp)
{
	return rtl8192f_txpwr_clamp(level + ramp);
}

/* COUNT_SIGN_OFFSET() (8192cd_cfg.h 322): the MIB stores one signed 4-bit
 * step per path in each byte, low nibble path A and high nibble path B, so
 * 0x0e means -2 and 0x01 means +1.
 */
static u8 rtl8192f_txpwr_diff(u8 level, u8 packed, bool path_b)
{
	u8 oft = path_b ? packed >> 4 : packed & 0x0f;

	if (oft & 0x08)
		return rtl8192f_txpwr_clamp(level - (0x10 - oft));
	return rtl8192f_txpwr_clamp(level + oft);
}

/* Four clamped codewords for one TXAGC register, high byte first. */
static u32 rtl8192f_txagc(u8 level, const s8 *ramp)
{
	return (u32)rtl8192f_txpwr_step(level, ramp[0]) << 24 |
	       (u32)rtl8192f_txpwr_step(level, ramp[1]) << 16 |
	       (u32)rtl8192f_txpwr_step(level, ramp[2]) << 8 |
	       rtl8192f_txpwr_step(level, ramp[3]);
}

/* The same codeword in all four rate slots, for a group with a flat ramp. */
static u32 rtl8192f_txagc_flat(u8 level)
{
	return level * 0x01010101;
}

/* The board's factory record, taken as one block from the MIB partition in
 * flash.  Absent or short is not an error: the composition below then runs
 * entirely on the vendor's fallback levels.
 */
int rtl8192f_read_calib(struct rtl8192f_soc_priv *priv)
{
	struct device *dev = &priv->pdev->dev;
	struct nvmem_cell *cell;
	void *buf;
	size_t len;

	cell = nvmem_cell_get(dev, "efuse");
	if (IS_ERR(cell)) {
		if (PTR_ERR(cell) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_info(dev, "no board efuse record, using the default TX power level\n");
		return 0;
	}

	buf = nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);
	if (IS_ERR(buf))
		return 0;

	if (len < RTL8192F_CAL_LEN) {
		dev_warn(dev, "board efuse record is %zu bytes, need %d, ignoring it\n",
			 len, RTL8192F_CAL_LEN);
	} else {
		memcpy(priv->calib, buf, RTL8192F_CAL_LEN);
		priv->has_calib = true;
	}
	kfree(buf);

	return 0;
}

static u8 rtl8192f_calib_get(const struct rtl8192f_soc_priv *priv, size_t off)
{
	return priv->has_calib ? priv->calib[off] : 0;
}

static void rtl8192f_set_tx_power(struct rtl8192f_soc_priv *priv, u32 channel,
				  bool wide)
{
	unsigned int idx = channel - 1;
	const s8 *ofdm_ramp = rtl8192f_pg_ofdm;
	const s8 *ht_ramp = rtl8192f_pg_ht;
	u8 level, ofdm, ht, ht2s, cck_a, cck_b;

	/* OFDM and HT share one stored level: the HT40 column of the record.
	 * The OFDM group and, at 20 MHz only, the HT group are that level
	 * shifted by their own per-channel diff nibble, and MCS8-15 shift the
	 * HT result once more (Hal88XXPhyCfg.c 4490, 4516, 4521, 4540).  This
	 * core has a single transmitter, so only path A is programmed -
	 * MCS8-15 still mirror the single-stream groups.
	 */
	level = rtl8192f_calib_get(priv, RTL8192F_CAL_PWR_HT40_A + idx);
	if (!level)
		level = RTL8192F_TXPWR_OFDM_DEFAULT;
	ofdm = rtl8192f_txpwr_diff(level,
				   rtl8192f_calib_get(priv,
						      RTL8192F_CAL_DIFF_OFDM + idx),
				   false);
	ht = level;
	if (!wide)
		ht = rtl8192f_txpwr_diff(ht,
					 rtl8192f_calib_get(priv,
							    RTL8192F_CAL_DIFF_HT20 + idx),
					 false);
	ht2s = rtl8192f_txpwr_diff(ht,
				   rtl8192f_calib_get(priv,
						      RTL8192F_CAL_DIFF_HT40_2S + idx),
				   false);

	rtl8192f_write_bb_reg(priv, 0xe00, rtl8192f_txagc(ofdm, ofdm_ramp));
	rtl8192f_write_bb_reg(priv, 0xe04, rtl8192f_txagc(ofdm, ofdm_ramp + 4));
	rtl8192f_write_bb_reg(priv, 0xe10, rtl8192f_txagc(ht, ht_ramp));
	rtl8192f_write_bb_reg(priv, 0xe14, rtl8192f_txagc(ht, ht_ramp + 4));
	rtl8192f_write_bb_reg(priv, 0xe18, rtl8192f_txagc(ht2s, ht_ramp + 8));
	rtl8192f_write_bb_reg(priv, 0xe1c, rtl8192f_txagc(ht2s, ht_ramp + 12));

	/* CCK has a flat ramp, so only the two per-channel levels matter
	 * here.  A zero entry falls back the same way it does above, but
	 * per path: the vendor tests A and B together (Hal88XXPhyCfg.c
	 * 4666) yet takes each level on its own.
	 */
	cck_a = rtl8192f_calib_get(priv, RTL8192F_CAL_PWR_CCK_A + idx);
	if (!cck_a)
		cck_a = RTL8192F_TXPWR_CCK_DEFAULT;
	cck_b = rtl8192f_calib_get(priv, RTL8192F_CAL_PWR_CCK_B + idx);
	if (!cck_b)
		cck_b = RTL8192F_TXPWR_CCK_DEFAULT;

	/* CCK 1M shares 0xe08 with the MCS32 slot (bits [15:8]); the other
	 * fields keep whatever the BB table left there, as on the shipped
	 * AP.  0x86c carries path A's 11/5.5/2M plus path B's 11M, and
	 * 0x838[31:8] the rest of path B - the vendor writes both groups
	 * unconditionally even on this single-transmitter core.
	 */
	rtl8192f_rmw_bb(priv, 0xe08, 0x0000ff00, cck_a << 8);
	rtl8192f_rmw_bb(priv, 0x838, 0xffffff00,
			rtl8192f_txagc_flat(cck_b) & 0xffffff00);
	rtl8192f_write_bb_reg(priv, 0x86c,
			      (rtl8192f_txagc_flat(cck_a) & 0xffffff00) |
			      cck_b);

	/* The levels written here come out of the factory (PG) ramp in the
	 * board record, which is what the shipped device is calibrated from
	 * too; the shipped device sits a few codewords above the PG ramp on
	 * the slower rates because the vendor then applies its own regulatory
	 * limit table (WlanHAL/Data/8197F/TXPWR_LMT_8197Fmp.txt, real dBm per
	 * channel and rate section) on top.  Those limits are not ported: in
	 * this kernel the per-channel regulatory ceiling is cfg80211's job,
	 * and the levels written here stay inside the factory per-channel
	 * value the board was calibrated to.
	 */
}

/* Cut-A silicon workaround, phydm_phypara_a_cut() verbatim
 * (phydm/rtl8197f/phydm_rtl8197f.c 93-111).  The vendor runs it right
 * after the BB PHY_REG table download only when CutVersion == A
 * (phydm_hwconfig.c 2536); the call site gates on the SYS_CFG cut
 * version accordingly.
 */
static void rtl8192f_phypara_a_cut(struct rtl8192f_soc_priv *priv)
{
	rtl8192f_rmw_bb(priv, 0x97c, 0xff000000, 0x20);
	rtl8192f_rmw_bb(priv, 0xa9c, BIT(17), 0);
	rtl8192f_rmw_bb(priv, 0xa0c, 0xff0000, 0x7e);
	rtl8192f_rmw_bb(priv, 0xa0c, 0xff00, 0x0);
	rtl8192f_rmw_bb(priv, 0xa84, 0xff0000, 0x1f);

	/* path A/B r_rxhp_tx / r_rxhp_t2r */
	rtl8192f_rmw_bb(priv, 0x824, BIT(14), BIT(14));
	rtl8192f_rmw_bb(priv, 0x824, BIT(17), BIT(17));
	rtl8192f_rmw_bb(priv, 0x82c, BIT(14), BIT(14));
	rtl8192f_rmw_bb(priv, 0x82c, BIT(17), BIT(17));

	rtl8192f_rmw_bb(priv, 0xc5c, BIT(2), BIT(2));
	rtl8192f_rmw_bb(priv, 0xc5c, BIT(5), BIT(5));
	rtl8192f_rmw_bb(priv, 0xc5c, BIT(8), BIT(8));
	rtl8192f_rmw_bb(priv, 0xc5c, BIT(11), BIT(11));
	rtl8192f_rmw_bb(priv, 0xc5c, BIT(14), BIT(14));
	rtl8192f_rmw_bb(priv, 0xce0, BIT(2), BIT(2));
	rtl8192f_rmw_bb(priv, 0xc7c, BIT(24), BIT(24));
	rtl8192f_rmw_bb(priv, 0xc7c, BIT(27), BIT(27));
}

static void rtl8192f_apply_rf_table(struct rtl8192f_soc_priv *priv,
				    const struct rtl8192f_rfregval *table,
				    size_t len)
{
	size_t i;

	for (i = 0; i < len; i++) {
		if (table[i].addr == 0xfe) {
			mdelay(50);
			continue;
		}
		rtl8192f_phy_set_rf_reg(priv, table[i].addr, table[i].val);
		udelay(1);	/* vendor: 1 us between RF writes */
	}
}

/* TX FIFO page allocation + auto-LLT init, vendor
 * InitLLT_Table88XX_V1() (Hal88XXGen.c 4461+, 8197F branch).  Without
 * it no TX queue owns any FIFO pages and the first transmitted frame
 * raises TX FIFO overflow (TXFOVW).
 */
#define REG_RQPN_CTRL_2		0x022c	/* trigger byte at +3 */
#define REG_AUTO_LLT		0x0208
#define REG_FWHW_TXQ_CTRL	0x0420
#define BIT_EN_WR_FREE_TAIL	BIT(20)
#define REG_BCNQ1_BDNY		0x0456
#define REG_MGQ_BDNY		0x047a
#define REG_WMAC_LBK_BUF_HD	0x0478
#define SECOND_BCN_PAGE_OFFSET	5

int rtl8192f_hw_fifo_init(struct rtl8192f_soc_priv *priv)
{
	static const u16 page_cnt[5] = { 0x0e, 0x20, 0x29, 0x04, 0x9a };
	u16 bndy = 0xf6;	/* = sum(page_cnt) + 1 */
	u32 val32;
	u8 val8;
	int i;

	for (i = 0; i < 5; i++)
		rtl8192f_write16(priv, REG_FIFOPAGE_INFO(i), page_cnt[i]);

	/* latch the page numbers into the queue configuration */
	rtl8192f_write8(priv, REG_RQPN_CTRL_2 + 3, 0x80);

	val32 = rtl8192f_read32(priv, REG_FWHW_TXQ_CTRL);
	rtl8192f_write32(priv, REG_FWHW_TXQ_CTRL,
			 val32 | BIT_EN_WR_FREE_TAIL);

	rtl8192f_write16(priv, REG_BCNQ_BDNY, bndy);
	rtl8192f_write16(priv, REG_BCNQ1_BDNY, bndy + SECOND_BCN_PAGE_OFFSET);
	rtl8192f_write16(priv, REG_MGQ_BDNY, bndy);
	/* BCNQ / BCNQ1 head pages (0x204 word pair) */
	rtl8192f_write16(priv, 0x0204, bndy);
	rtl8192f_write16(priv, 0x0206, bndy + SECOND_BCN_PAGE_OFFSET);
	rtl8192f_write16(priv, REG_WMAC_LBK_BUF_HD, bndy);

	rtl8192f_write32(priv, REG_FWHW_TXQ_CTRL,
			 rtl8192f_read32(priv, REG_FWHW_TXQ_CTRL) &
			 ~BIT_EN_WR_FREE_TAIL);

	/* hardware auto-links the page list; bit0 self-clears on done */
	rtl8192f_write8(priv, REG_AUTO_LLT,
			rtl8192f_read8(priv, REG_AUTO_LLT) | BIT(0));
	{
		int retry = 1000;

		do {
			val8 = rtl8192f_read8(priv, REG_AUTO_LLT);
			if (!(val8 & BIT(0)))
				break;
			udelay(10);
		} while (--retry);

		if (!retry) {
			dev_err(&priv->pdev->dev, "auto LLT init timeout\n");
			return -ETIMEDOUT;
		}
	}

	return 0;
}

int rtl8192f_hw_init(struct rtl8192f_soc_priv *priv)
{
	u32 rcr;

	/* Enable BB and RF (vendor 8192cd_hw.c 8197F init: SYS_FUNC_EN
	 * gets BIT0|BIT1|BIT13 - bit13 DIO_RF routes the RF interface
	 * over DIO on the 8197F).
	 */
	rtl8192f_write16(priv, REG_SYS_FUNC_EN,
			 rtl8192f_read16(priv, REG_SYS_FUNC_EN) |
			 SYS_FUNC_BBRSTB | SYS_FUNC_BB_GLB_RSTN |
			 SYS_FUNC_DIO_RF);

	/* RF path power, 8197F-specific three steps (8192cd_hw.c
	 * 8644-8666): reset then enable BIT_RF1_SDMRSTB/BIT_RF1_RSTB/
	 * BIT_RF1_EN of REG_AFE_CTRL4 (0x78) bits 26:24, enable
	 * BIT_POW_REGU_P0/P1 (0x20 bit2/bit10), then the RF_CTRL go.
	 */
	rtl8192f_write8(priv, REG_RF_CTRL,
			rtl8192f_read8(priv, REG_RF_CTRL) &
			~(RF_ENABLE | RF_RSTB | RF_SDMRSTB));
	rtl8192f_write32(priv, 0x78,
			 rtl8192f_read32(priv, 0x78) &
			 ~(BIT(24) | BIT(25) | BIT(26)));
	rtl8192f_write32(priv, 0x78,
			 rtl8192f_read32(priv, 0x78) |
			 BIT(24) | BIT(25) | BIT(26));
	rtl8192f_write32(priv, 0x20,
			 rtl8192f_read32(priv, 0x20) | BIT(2) | BIT(10));

	rtl8192f_write8(priv, REG_RF_CTRL, RF_ENABLE | RF_RSTB | RF_SDMRSTB);

	/* To fix MAC loopback mode fail. */
	rtl8192f_write8(priv, REG_LDOHCI12_CTRL, 0x0f);

	/* RFE type 0 GPIO muxing (8192cd_hw.c 12583-12589; the MIB
	 * rfe_type default is 0, vendor runs these on every 8197F
	 * without an explicit RFE setting).
	 */
	rtl8192f_write_bb_reg(priv, 0x4c,
			      rtl8192f_read_bb_reg(priv, 0x4c) & ~BIT(24));
	rtl8192f_write_bb_reg(priv, 0x64,
			      rtl8192f_read_bb_reg(priv, 0x64) & ~BIT(28));
	rtl8192f_write_bb_reg(priv, 0x40,
			      rtl8192f_read_bb_reg(priv, 0x40) & ~0x0f000000);

	/* config_phydm_parameter_8197f_init(ODM_PRE_SETTING): OFDM/CCK
	 * baseband blocks stay gated off while the tables load
	 * (phydm_hal_api8197f.c 1000-1018, call site 8192cd_hw.c
	 * 13705).
	 */
	rtl8192f_rmw_bb(priv, 0x800, BIT(24) | BIT(25), 0);

	rtl8192f_apply_mac_table(priv, rtl8192f_soc_mac_init_table,
				 RTL8192F_MAC_TABLE_LEN);

	rtl8192f_apply_bb_table(priv, rtl8192f_soc_phy_init_table,
				RTL8192F_PHY_TABLE_LEN);
	/* Cut detection: SYS_CFG[15:12] is the vendor cut index
	 * (8192cd_hw.c 12428: ODM_CMNINFO_CUT_VER), where 1 is the B-cut, and
	 * the vendor runs the A-cut workaround block only for CutVersion == A.
	 */
	priv->cut_version = (rtl8192f_read32(priv, REG_SYS_CFG) >> 12) & 0xf;
	if (priv->cut_version == 0)
		rtl8192f_phypara_a_cut(priv);

	rtl8192f_apply_bb_table(priv, rtl8192f_soc_agc_table,
				RTL8192F_AGC_TABLE_LEN);
	rtl8192f_apply_rf_table(priv, rtl8192f_soc_radioa_table,
				RTL8192F_RF_TABLE_LEN);

	/* config_phydm_parameter_8197f_init(ODM_POST_SETTING): enable
	 * the OFDM and CCK blocks.  Without this the baseband never
	 * produces received frames (no RXOK and no RDU ever fire).
	 */
	rtl8192f_rmw_bb(priv, 0x800, BIT(24) | BIT(25), BIT(24) | BIT(25));
	/* RX FIFO byte boundaries and TX DMA queue mapping, vendor
	 * InitMAC88XX() (Hal88XXGen.c 4907-4933, IS_EXIST_EMBEDDED
	 * branch).  The RX FIFO area boundaries are 8197F bring-up
	 * essentials: without them the RX DMA engine has no FIFO space;
	 * BIT_RXSHFT_EN keeps
	 * the delivered payload 4-byte aligned, BIT_RXDMA_ARBBW_EN is
	 * the RX DMA arbitration burst width.
	 */
	rtl8192f_write32(priv, REG_TRXFF_BNDY,
			 (rtl8192f_read32(priv, REG_TRXFF_BNDY) & 0xffff) |
			 (MAC_RXFF_SIZE << 16));
	rtl8192f_write32(priv, REG_RXFF_BNDY,
			 (rtl8192f_read32(priv, REG_RXFF_BNDY) & 0xfffc0000) |
			 MAC_RXFF_SIZE);
	rtl8192f_write32(priv, REG_TRXDMA_CTRL,
			 TRX_DMA_QUEUE_MAP_PARA | BIT_RXSHFT_EN |
			 BIT_RXDMA_ARBBW_EN);

	/* RX driver info: 4 x 8-byte units appended per packet,
	 * APP_MH_SHIFT set, WMAC_ENSHIFT cleared (Hal88XXGen.c
	 * 4966-4973).  The RX path reads the per-packet echo of this
	 * size from the RX descriptor, so no parser constant to keep in
	 * sync.
	 */
	rtl8192f_write8(priv, REG_RX_DRVINFO_SZ,
			(rtl8192f_read8(priv, REG_RX_DRVINFO_SZ) & 0xf0) |
			RX_DRVINFO_SZ_UNIT);
	rtl8192f_write8(priv, REG_RX_DRVINFO_SZ,
			(rtl8192f_read8(priv, REG_RX_DRVINFO_SZ) |
			 RX_DRVINFO_APP_MH_SHIFT) & ~RX_DRVINFO_ENSHIFT);

	/* 0x60C/0x60D/0x60F are BYTE registers (HalComReg.h 1135-1137).
	 * RX_PKT_LIMIT is already set to the vendor value 0x18 by the
	 * MAC init table above (tables_mac.c 0x60c=0x18) and
	 * RX_DRVINFO_SZ is programmed with write8 further up, so no
	 * manual write here at all: the previous 16-bit write to
	 * "0x60E" cleared 0x60F (RX_DRVINFO_SZ) and clobbered 0x60D
	 */

	/* MAC sequence / beacon queue boundary.  BCNQ_BDNY is 0x424 on
	 * the 88XX map; TBTT (MBSSID_BCN_SPACE 0x554)
	 * and BCNDMATIM (0x559) are covered by the vendor MAC_REG init
	 * table above, so no manual write here.
	 */
	rtl8192f_write8(priv, REG_HWSEQ_CTRL, 0xff);

	/* rtl8192f_fifo_init() (AUTO_LLT / free-page list) moved to
	 * rtl8192f_txrx_start() behind the CR DMA enables: with the DMA
	 * engine still gated the LLT chain never gets built and every
	 * later TX fails with TXFF_PG_OVF/TXFF_PG_UDN in REG_TXDMA_STATUS
	 * (vendor InitMAC88XX enables CR before InitLLT_TableHandler).
	 */

	/* Initial WMAC filter settings: accept unicast to us, multicast,
	 * broadcast and management frames; append PHY status and security
	 * fields to the RX descriptor.
	 */
	rcr = RCR_AAP | RCR_APM | RCR_AM | RCR_AB |
	      RCR_APP_FCS | RCR_APP_MIC | RCR_APP_ICV |
	      RCR_APP_PHYST_RXFF;
	rtl8192f_write32(priv, REG_RCR, rcr);

	/* Default EDCA parameters (rtl8xxxu_init_common(), match vendor
	 * Win7 defaults)
	 */
	rtl8192f_write32(priv, REG_EDCA_BE_PARAM, 0x005ea42b);
	rtl8192f_write32(priv, REG_EDCA_BK_PARAM, 0x0000a44f);
	rtl8192f_write32(priv, REG_EDCA_VI_PARAM, 0x005ea324);
	rtl8192f_write32(priv, REG_EDCA_VO_PARAM, 0x002fa226);

	/* The vendor ORs four bits here; only the two queue-counter
	 * updates apply to this part, and BIT_CFEND_FORMAT (bit 9) is not among
	 * them.
	 */
	rtl8192f_setbits32(priv, REG_TCR, BIT_TCR_UPD_HGQMD |
			   BIT_TCR_UPD_TIMIE | BIT(0) |
			   BIT(12) | BIT(13));

	/* Accept only PS-Poll among control frames (vendor MAC_V2 branch,
	 * Hal88XXGen.c 524 `W16(REG_RXFLTMAP1, BIT_CTRLFLT10EN)`; the MP
	 * image leaves 0x6A2 = 0xffff, which DMAs every ACK/CTS/Block-Ack
	 * on the channel into the RX ring as 14-32 byte junk entries).
	 */
	rtl8192f_write16(priv, REG_RXFLTMAP1, BIT_RXFLT_PSPOLL);

	/* Rate fall-back set 1: the MP image value we just applied
	 * (0x003ff010 / 0x40000000) belongs to the 8814A branch of
	 * HW_VAR_MAC_CONFIG; 8197F takes the else branch and writes
	 * 0x00000015 / 0x003ff000 (Hal88XXGen.c 374-375), which is what
	 * the working vendor AP reads back at 0x44C/0x450.
	 */
	rtl8192f_write32(priv, REG_ARFR1, 0x00000015);
	rtl8192f_write32(priv, REG_ARFR1 + 4, 0x003ff000);

	/* clear pending interrupt status, mask all until start */
	rtl8192f_write32(priv, REG_HISR0, 0xffffffff);
	rtl8192f_write32(priv, REG_HISR1, 0xffffffff);
	rtl8192f_write32(priv, REG_HIMR0, 0);
	rtl8192f_write32(priv, REG_HIMR1, 0);

	/* AP power-on calibration: LCK + one-shot IQK (vendor
	 * phydm_iqk_8197f.c runs IQK + LCK at init).
	 */
	rtl8192f_iqk(priv);
	/* Channel 1's level until mac80211 programs the real one: the
	 * factory array is per channel, and leaving the group at the BB
	 * table's zero would drive every frame from the bottom of the PA
	 * ladder in the window before the first config().
	 */
	rtl8192f_set_tx_power(priv, priv->cur_channel ?: 1, false);

	priv->hw_ready = true;

	return 0;
}

/* ------------------------------------------------------------------ */
/* TX power                                                            */

/* ------------------------------------------------------------------ */
/* Dynamic initial gain                                                */
/* ------------------------------------------------------------------ */
/* Count the false alarms of one period.  The vendor holds both counter pages
 * and the CCK counters before reading, then pulses the resets and drops the
 * holds (phydm_dig.c 2959-3030); without that the next period would report a
 * running total instead of its own.  The OFDM bins are the six the vendor
 * sums into Cnt_Ofdm_fail - the low half of 0xda0 is a busy-channel count and
 * stays out of it (phydm_dig.c 2715-2718) - and the CCK side is the 16-bit FA
 * counter split across 0xa5c (low byte) and 0xa58 (high byte), assembled as at
 * phydm_dig.c 2747-2753.
 */
static u32 rtl8192f_dig_fa_count(struct rtl8192f_soc_priv *priv)
{
	u32 val, ofdm, cck;

	rtl8192f_rmw_bb(priv, REG_FA_OFDM_HOLD, BIT(31), BIT(31));
	rtl8192f_rmw_bb(priv, REG_FA_OFDM_RESET2, BIT(31), BIT(31));
	rtl8192f_rmw_bb(priv, REG_FA_CCK_CTRL, BIT(12), BIT(12));
	rtl8192f_rmw_bb(priv, REG_FA_CCK_CTRL, BIT(14), BIT(14));

	val = rtl8192f_read32(priv, REG_FA_OFDM_SYNC);
	ofdm = FIELD_GET(GENMASK(15, 0), val) + FIELD_GET(GENMASK(31, 16), val);

	val = rtl8192f_read32(priv, REG_FA_OFDM_CCA);
	ofdm += FIELD_GET(GENMASK(31, 16), val);

	val = rtl8192f_read32(priv, REG_FA_OFDM_HDR);
	ofdm += FIELD_GET(GENMASK(15, 0), val) + FIELD_GET(GENMASK(31, 16), val);

	val = rtl8192f_read32(priv, REG_FA_OFDM_MCS);
	ofdm += FIELD_GET(GENMASK(15, 0), val);

	cck = FIELD_GET(GENMASK(7, 0), rtl8192f_read32(priv, REG_FA_CCK_LOW)) +
	      (FIELD_GET(GENMASK(31, 24), rtl8192f_read32(priv, REG_FA_CCK_HIGH))
	       << 8);

	rtl8192f_rmw_bb(priv, REG_FA_OFDM_RESET, BIT(31), BIT(31));
	rtl8192f_rmw_bb(priv, REG_FA_OFDM_RESET, BIT(31), 0);
	rtl8192f_rmw_bb(priv, REG_FA_OFDM_RESET2, BIT(27), BIT(27));
	rtl8192f_rmw_bb(priv, REG_FA_OFDM_RESET2, BIT(27), 0);
	rtl8192f_rmw_bb(priv, REG_FA_OFDM_HOLD, BIT(31), 0);
	rtl8192f_rmw_bb(priv, REG_FA_OFDM_RESET2, BIT(31), 0);
	rtl8192f_rmw_bb(priv, REG_FA_CCK_CTRL, BIT(13) | BIT(12), 0);
	rtl8192f_rmw_bb(priv, REG_FA_CCK_CTRL, BIT(13) | BIT(12), BIT(13));
	rtl8192f_rmw_bb(priv, REG_FA_CCK_CTRL, BIT(15) | BIT(14), 0);
	rtl8192f_rmw_bb(priv, REG_FA_CCK_CTRL, BIT(15) | BIT(14), BIT(15));

	return ofdm + cck;
}

/* One DIG step: trade the initial gain against the measured false-alarm
 * rate.  Above the middle threshold the gain code goes up, below the low
 * threshold it comes back down, and the code stays inside the vendor's
 * 1T1R 2.4G window.  The direction of the arithmetic is the SDK's
 * (phydm_dig.c 2598-2605), not a re-derivation: 0xc50 counts the other
 * way from what the name suggests, and the shipped radio is the
 * reference for which way is "less sensitive".
 *
 * Left out on purpose, because each needs state this host path does not
 * have: the RSSI-driven narrowing of the bounds, the TDMA dual-gain
 * machine, the CCK packet-detect/CS-ratio ladder (0xa08[19:16],
 * 0xaa8[20:16]) and the AGC table switching.  Their absence is why a
 * quiet channel still sits at the initial value: only the false-alarm
 * term drives the loop here.
 */
void rtl8192f_dig_work(struct work_struct *work)
{
	struct rtl8192f_soc_priv *priv =
		container_of(to_delayed_work(work), struct rtl8192f_soc_priv,
			     dig_work);
	u32 fa;
	u8 igi;

	if (!priv->hw_ready)
		return;

	fa = rtl8192f_dig_fa_count(priv);
	igi = priv->dig_igi;

	if (fa > DIG_FA_TH_HIGH)
		igi += DIG_FA_STEP_HIGH;
	else if (fa > DIG_FA_TH_MID)
		igi += DIG_FA_STEP_MID;
	else if (fa < DIG_FA_TH_LOW)
		igi -= DIG_FA_STEP_LOW;

	igi = clamp_t(u8, igi, DIG_IGI_MIN, DIG_IGI_MAX);
	if (igi != priv->dig_igi) {
		priv->dig_igi = igi;
		rtl8192f_rmw_bb(priv, REG_INITIAL_GAIN, 0xff, igi);
	}

	schedule_delayed_work(&priv->dig_work,
			      msecs_to_jiffies(DIG_PERIOD_MS));
}

/* Seed the state from the value the BB init table left in 0xc50 - the same
 * place the vendor's DIG initialisation reads - and clear the counters so the
 * first sample covers one period only.
 */
void rtl8192f_dig_start(struct rtl8192f_soc_priv *priv)
{
	priv->dig_igi = rtl8192f_read32(priv, REG_INITIAL_GAIN) & 0xff;
	rtl8192f_dig_fa_count(priv);
	schedule_delayed_work(&priv->dig_work,
			      msecs_to_jiffies(DIG_PERIOD_MS));
}

void rtl8192f_dig_stop(struct rtl8192f_soc_priv *priv)
{
	cancel_delayed_work_sync(&priv->dig_work);
}

/* ------------------------------------------------------------------ */
/* Channel switching                                                   */
/* ------------------------------------------------------------------ */

/* Secondary channel index for REG_DATA_SC, per get_txsc_92e()
 * (8192cd_hw.c 18487-18499): 1 when the extension channel sits below the
 * primary, 2 above, 0 at 20 MHz.
 */
#define RTL8192F_DATA_SC_BELOW		1
#define RTL8192F_DATA_SC_ABOVE		2

/* The two receive-path helpers that config_phydm_switch_bandwidth_8197f()
 * runs after the width writes: phydm_RxDfirParByBw_8197f()
 * (phydm_hal_api8197f.c 198-217) and phydm_CcaParByBw_8197f() (same file
 * 29-186).  They are bandwidth-dependent, so a 40 MHz switch that stops
 * at the width registers leaves the receive filter and the CCA threshold
 * configured for 20 MHz.
 *
 * Only the RFEType 0 branch of the CCA helper is ported: this core's
 * board record says RFEType = 0, and the other branches key off
 * external PA/LNA variants this core does not have.  Inside that branch
 * the vendor tests RXAntStatus with two arms that write the same value,
 * so the test is not ported either.  The cut detected at init is B
 * (rtl8192f_hw_init()), so the A-cut arm, which never touches 0xaa8,
 * does not apply here.
 *
 * The same helper also pins 0xe24[21:20] = 2 (TAP_UPD) and 0xc10[29:28]
 * = 2 (down-sampling factor) on every switch; the BB init table already
 * holds both fields at 2 (0x00200000 and 0x22000000), so re-writing them
 * here would be a no-op.
 */
static void rtl8192f_bw_rx_path(struct rtl8192f_soc_priv *priv, bool wide)
{
	u32 c3c;

	/* RX DFIR tap set: bit8 selects the BW20/BW10/BW5 bank with the
	 * coefficient 0xa3 in byte0, the 40 MHz bank is bit8 clear with 0x3.
	 */
	rtl8192f_rmw_bb(priv, REG_RX_DFIR_MOD, BIT(8), wide ? 0 : BIT(8));
	rtl8192f_rmw_bb(priv, REG_RX_DFIR_MOD, GENMASK(7, 0),
			wide ? 0x3 : 0xa3);

	/* Energy-detect CCA threshold for the width actually in use:
	 * 0xaa8[20:16] = 0xf at 20 MHz, 0x12 at 40 MHz.
	 */
	rtl8192f_rmw_bb(priv, REG_EDCCA_LEVEL, GENMASK(20, 16),
			wide ? FIELD_PREP(GENMASK(20, 16), 0x12) :
			       FIELD_PREP(GENMASK(20, 16), 0xf));

	/* 0xc3c: rebuild bits[8:3] the way the vendor branch does - keep
	 * bits[8:6], drop bits[5:3] and set bit4 (both bandwidth arms write
	 * the same 2 << 3 here).
	 */
	c3c = rtl8192f_read_bb_reg(priv, REG_CCA_CTRL);
	rtl8192f_rmw_bb(priv, REG_CCA_CTRL, GENMASK(8, 3),
			(c3c & GENMASK(8, 6)) | BIT(4));
}

/* Primary-CCA side select: phydm_api_primary_cca() (phydm_rtl8197f.c
 * 185-215), which SwChnl() runs at the end of every channel switch on
 * this chip - 1 when the extension channel is above the primary, 2
 * below, 0 at 20 MHz (8192cd_hw.c 7570-7586).  The same call parks the
 * PD threshold: naming a single side band clears 0xc84[31:28], and the
 * saved value only comes back when the select returns to the both-halves
 * setting.
 *
 * Both fields sit at their BB-table values unless something writes them:
 * 0xc6c = 0x35 (select 0) and 0xc84 = 0x32000000 (threshold 3).  That
 * is the 20 MHz configuration, and a core left in it while the rest of
 * the PHY is in 40 MHz mode asserts no carrier detect at all, so the AP
 * receives nothing.
 */
static void rtl8192f_bw_primary_cca(struct rtl8192f_soc_priv *priv,
				    enum nl80211_channel_type chan_type)
{
	u32 sel = FIELD_GET(PRIMARY_CCA_SEL,
			    rtl8192f_read_bb_reg(priv, REG_PRIMARY_CCA));
	u32 next = PRIMARY_CCA_DSC;

	if (chan_type == NL80211_CHAN_HT40MINUS)
		next = PRIMARY_CCA_USC;
	else if (chan_type == NL80211_CHAN_HT40PLUS)
		next = PRIMARY_CCA_LSC;

	if (sel == PRIMARY_CCA_DSC || sel == 3) {
		u32 pd = rtl8192f_read_bb_reg(priv, REG_PD_THRESHOLD);

		priv->pd_threshold = (u8)FIELD_GET(PD_THRESHOLD, pd);
	}

	rtl8192f_rmw_bb(priv, REG_PRIMARY_CCA, PRIMARY_CCA_SEL,
			FIELD_PREP(PRIMARY_CCA_SEL, next));

	if (next == PRIMARY_CCA_DSC || next == 3)
		rtl8192f_rmw_bb(priv, REG_PD_THRESHOLD, PD_THRESHOLD,
				FIELD_PREP(PD_THRESHOLD, priv->pd_threshold));
	else
		rtl8192f_rmw_bb(priv, REG_PD_THRESHOLD, PD_THRESHOLD, 0);
}

/* Program the 20/40 MHz path.  This is SwBWMode88XX_97F()
 * (Hal8197FPhyCfg.c 78-102) plus the BB half of
 * config_phydm_switch_bandwidth_8197f() (phydm_hal_api8197f.c 889-961);
 * the RF half (0x18[11:10]) is folded into the channel write below so the
 * synthesizer and the width change in one HSSI transaction.
 *
 * The DAC clock field (0x800[14:12] = 5) and the ADC buffer clock
 * (0xca4[27:26] = 2) carry the same value on both widths in the vendor's
 * 20 MHz and 40 MHz branches, so writing them unconditionally is not a
 * change of behaviour; the ADC clock (0xbb0[18:16]) is only touched on the
 * way back to 20 MHz because the vendor's 40 MHz branch leaves it to the BB
 * init table.
 */
static void rtl8192f_set_bandwidth(struct rtl8192f_soc_priv *priv,
				   enum nl80211_channel_type chan_type)
{
	bool wide = chan_type == NL80211_CHAN_HT40PLUS ||
		    chan_type == NL80211_CHAN_HT40MINUS;
	u8 sc = 0;

	if (wide)
		sc = chan_type == NL80211_CHAN_HT40MINUS ?
			RTL8192F_DATA_SC_BELOW : RTL8192F_DATA_SC_ABOVE;

	rtl8192f_write8(priv, REG_DATA_SC,
			(rtl8192f_read8(priv, REG_DATA_SC) & 0xf0) | sc);

	rtl8192f_rmw_bb(priv, REG_SMALL_BANDWIDTH, 0xc0000000, 0);
	rtl8192f_rmw_bb(priv, REG_RF_MODE, BIT(0), wide ? BIT(0) : 0);
	rtl8192f_rmw_bb(priv, REG_RF_MODE_PAGE9, BIT(0), wide ? BIT(0) : 0);
	rtl8192f_rmw_bb(priv, REG_RF_MODE, 0x7000, 5 << 12);
	rtl8192f_rmw_bb(priv, REG_ANTDIV_PARA1, 0xc000000, 2 << 26);

	if (wide) {
		/* CCK lives in the lower 20 MHz half when the extension
		 * channel is above the primary (the vendor's
		 * primary_ch_idx == 1 test).
		 */
		rtl8192f_rmw_bb(priv, REG_CCK_ANTDIV_PARA1, BIT(4),
				chan_type == NL80211_CHAN_HT40MINUS ? BIT(4) : 0);
	} else {
		rtl8192f_rmw_bb(priv, REG_RXCK_RFMOD, 0x70000, 4 << 16);
	}

	/* A width change moves the analyser's passband, so the vendor runs
	 * the same LCK/IQK as a channel switch does; the caller does it right
	 * after this.
	 */
}

/* vendor phydm_hal_api8197f.c config_phydm_switch_channel_8197f():
 * RF reg 0x18 carries both band select (bits 17:18, cleared for 2.4G)
 * and the channel number the synthesizer is set to in bits 7:0 - the
 * argument there is named central_ch, and 8192cd_hw.c SwChnl() feeds it
 * get_center_channel(), which is the primary +-2 at 40 MHz.  There is no
 * N-divider LUT for 2.4G, so the primary only says which 20 MHz half
 * legacy frames live in; writing it to the synthesizer tunes the LO
 * 10 MHz off the channel we are actually on.
 */
int rtl8192f_set_channel(struct rtl8192f_soc_priv *priv, u32 channel,
			 enum nl80211_channel_type chan_type)
{
	bool wide = chan_type == NL80211_CHAN_HT40PLUS ||
		    chan_type == NL80211_CHAN_HT40MINUS;
	u32 center = channel;
	u32 rf18;

	if (!channel || channel > 14)
		return -EINVAL;

	if (wide)
		center = chan_type == NL80211_CHAN_HT40MINUS ? channel - 2 :
								channel + 2;

	rtl8192f_set_bandwidth(priv, chan_type);

	rf18 = rtl8192f_phy_get_rf_reg(priv, RF_REG_MODE_AG);

	/* switch to 2.4G band and program central channel */
	rf18 &= ~(RF_MODE_AG_BAND_MASK | 0xff);
	rf18 |= center;

	/* RF bandwidth, 0x18[11:10]: both bits set for 20 MHz, only bit11
	 * for 40 MHz (phydm_hal_api8197f.c 933, 958-960).
	 */
	rf18 &= ~(BIT(11) | BIT(10));
	rf18 |= wide ? BIT(11) : (BIT(11) | BIT(10));

	rtl8192f_phy_set_rf_reg(priv, RF_REG_MODE_AG, rf18);

	/* The vendor's switch helper finishes the bandwidth change with the
	 * receive filter and the CCA threshold for the new width, after the
	 * RF bandwidth bit has gone out (config_phydm_switch_bandwidth_8197f()
	 * 1020-1023).
	 */
	rtl8192f_bw_rx_path(priv, wide);

	/* The vendor recalibrates and re-programs the PA ladder on every
	 * channel switch, not only at bring-up: SwChnl() calls
	 * PHY_IQCalibrate() (8192cd_hw.c 7523, 7530 - skipped only in
	 * monitor mode) and then SetTxPowerLevel() (:7461).  IQK tunes the
	 * modulator's I/Q balance for the frequency the synthesizer is now
	 * on, so a channel change without it leaves the previous channel's
	 * correction in place.
	 */
	rtl8192f_iqk(priv);
	rtl8192f_set_tx_power(priv, channel, wide);

	/* Last, as in SwChnl(): the select names the half the analyser should
	 * be listening on once the rest of the width switch is in place.
	 */
	rtl8192f_bw_primary_cca(priv, chan_type);

	priv->cur_channel = channel;
	priv->cur_chan_type = chan_type;

	return 0;
}

/* ------------------------------------------------------------------ */
/* EFuse                                                               */
/* ------------------------------------------------------------------ */

/* Read one physical EFuse byte through the gen-2 EFuse controller
 * (MAC space 0x30..0x33).  Vendor 8192cd_hw.c ReadEFuseByte():
 * write address into 0x31/0x32[1:0], load the read command 0x72 into
 * 0x33, poll 0x33[7] for ready, data appears in 0x30[7:0].
 */
static int rtl8192f_efuse_read_byte(struct rtl8192f_soc_priv *priv,
				    u16 offset, u8 *val)
{
	u8 tmp;
	u32 retry = 10000;

	tmp = rtl8192f_read8(priv, REG_EFUSE_CTRL + 1);
	rtl8192f_write8(priv, REG_EFUSE_CTRL + 1, (u8)(offset & 0xff));

	tmp = rtl8192f_read8(priv, REG_EFUSE_CTRL + 2);
	rtl8192f_write8(priv, REG_EFUSE_CTRL + 2,
			(tmp & 0xfc) | (u8)((offset >> 8) & 0x03));

	/* read command: EF_FLAG=0 (read), EF_RDT, ready-bit polled below */
	rtl8192f_write8(priv, REG_EFUSE_CTRL + 3, 0x72);

	while (retry--) {
		if (rtl8192f_read8(priv, REG_EFUSE_CTRL + 3) & 0x80)
			break;
		udelay(1);
	}
	if (!retry)
		return -ETIMEDOUT;

	/* settle delay suggested by vendor (read errors at high temp) */
	udelay(50);

	*val = rtl8192f_read8(priv, REG_EFUSE_CTRL);

	return 0;
}

/* Bring the EFuse controller clock/power up.  Vendor 8192cd_hw.c
 * efuse_PowerSwitch(), read direction.
 */
static void rtl8192f_efuse_power_on(struct rtl8192f_soc_priv *priv)
{
	u16 val16;

	/* Vendor efuse_PowerSwitch(): RSV_CTRL (0x1C) bit1 locks the
	 * 0x00/0x02 register area - clear it around each write there
	 * (these writes run before hw_power_on()).
	 */

	/* 1.2V power from VDDON: SYS_ISO_CTRL[15] PWC_EV12V */
	val16 = rtl8192f_read16(priv, REG_SYS_ISO_CTRL);
	if (!(val16 & SYS_ISO_PWC_EV12V)) {
		rtl8192f_write8(priv, REG_RSV_CTRL,
				rtl8192f_read8(priv, REG_RSV_CTRL) & ~BIT(1));
		rtl8192f_write16(priv, REG_SYS_ISO_CTRL,
				 val16 | SYS_ISO_PWC_EV12V);
		rtl8192f_write8(priv, REG_RSV_CTRL,
				rtl8192f_read8(priv, REG_RSV_CTRL) | BIT(1));
	}

	/* enable ELDR (EFuse loader) clock: SYS_FUNC_EN[12] */
	val16 = rtl8192f_read16(priv, REG_SYS_FUNC_EN);
	if (!(val16 & SYS_FUNC_ELDR)) {
		rtl8192f_write8(priv, REG_RSV_CTRL,
				rtl8192f_read8(priv, REG_RSV_CTRL) | BIT(1));
		rtl8192f_write16(priv, REG_SYS_FUNC_EN,
				 val16 | SYS_FUNC_ELDR);
		rtl8192f_write8(priv, REG_RSV_CTRL,
				rtl8192f_read8(priv, REG_RSV_CTRL) | BIT(1));
	}

	/* gate + 8M clock from ANA: SYS_CLKR[5], SYS_CLKR[1] */
	val16 = rtl8192f_read16(priv, REG_SYS_CLKR);
	if (!(val16 & (SYS_CLK_LOADER_CLK_EN | SYS_CLK_ANA8M)))
		rtl8192f_write16(priv, REG_SYS_CLKR,
				 val16 | SYS_CLK_LOADER_CLK_EN |
				 SYS_CLK_ANA8M);
}

int rtl8192f_efuse_read(struct rtl8192f_soc_priv *priv)
{
	u16 word[EFUSE_MAX_SECTION][EFUSE_MAX_WORD_UNIT];
	u16 addr = 0, i, j;
	u8 hdr, offset, wren;
	bool ext_hdr;
	int ret;

	memset(priv->efuse_map, 0xff, sizeof(priv->efuse_map));
	memset(word, 0xff, sizeof(word));

	rtl8192f_efuse_power_on(priv);

	/* empty EFuse: first physical byte stays 0xff */
	ret = rtl8192f_efuse_read_byte(priv, addr, &hdr);
	if (ret)
		return ret;
	if (hdr == 0xff)
		return 0;
	addr++;

	/* Walk the PG packet stream.  Header forms (vendor 8192cd_hw.c
	 * ReadEFuse()):
	 * - plain 1-byte header:  offset[7:4] | wren[3:0]
	 * - gen-2.5 extended 2-byte header, detected by the 5-bit PG
	 *   field (bits 4:0) == 0x0F: first byte bits 7:5 = offset
	 *   bits [2:0], second byte bits 7:4 = offset bits [6:3] and
	 *   bits 3:0 = wren.  (Vendor gates this form behind a
	 *   bSupportExtendHdr chip flag; gen-2.5 parts always use it.)
	 * In both cases a wren bit of 1 marks the corresponding word
	 * pair as skipped/bad - data bytes exist only for wren bits
	 * that are 0.
	 */
	while (hdr != 0xff && addr < EFUSE_REAL_CONTENT_LEN) {
		ext_hdr = (hdr & 0x1f) == 0x0f;

		if (ext_hdr) {
			u8 hdr2;

			ret = rtl8192f_efuse_read_byte(priv, addr++, &hdr2);
			if (ret)
				return ret;
			if ((hdr2 & 0x0f) == 0x0f) {
				/* "end of used area" marker: vendor
				 * consumes the second header byte plus
				 * one trailing byte before leaving the
				 * scan loop (ReadEFuse() ext-hdr
				 * branch).
				 */
				if (addr < EFUSE_REAL_CONTENT_LEN) {
					u8 pad;

					ret = rtl8192f_efuse_read_byte(priv,
								       addr++,
								       &pad);
					if (ret)
						return ret;
					if (pad != 0xff &&
					    addr < EFUSE_REAL_CONTENT_LEN)
						addr++;
				}
				break;
			}
			offset = ((hdr2 & 0xf0) >> 1) | (hdr & 0xe0) >> 5;
			wren = hdr2 & 0x0f;
		} else {
			offset = (hdr >> 4) & 0x0f;
			wren = hdr & 0x0f;
		}

		if (offset < EFUSE_MAX_SECTION) {
			for (i = 0; i < EFUSE_MAX_WORD_UNIT; i++) {
				u8 lo, hi;

				/* wren bit 0 => the word pair is
				 * present in the stream; bit 1 =>
				 * skipped (vendor: read only when
				 * !(wren & 0x01)).
				 */
				if (!(wren & 0x01) &&
				    addr < EFUSE_REAL_CONTENT_LEN) {
					ret = rtl8192f_efuse_read_byte(priv,
								       addr++,
								       &lo);
					if (ret)
						return ret;
					if (addr >= EFUSE_REAL_CONTENT_LEN)
						break;
					ret = rtl8192f_efuse_read_byte(priv,
								       addr++,
								       &hi);
					if (ret)
						return ret;
					word[offset][i] = lo |
						((u16)hi << 8);
					if (addr >= EFUSE_REAL_CONTENT_LEN)
						break;
				}
				wren >>= 1;
			}
		}

		if (addr >= EFUSE_REAL_CONTENT_LEN)
			break;
		ret = rtl8192f_efuse_read_byte(priv, addr, &hdr);
		if (ret)
			return ret;
		addr++;
	}

	/* collect sections into the logical map */
	for (i = 0; i < EFUSE_MAX_SECTION; i++) {
		for (j = 0; j < EFUSE_MAX_WORD_UNIT; j++) {
			u16 off = (i << 3) + (j << 1);

			if (off + 1 >= EFUSE_MAP_LEN)
				break;
			priv->efuse_map[off] = word[i][j] & 0xff;
			priv->efuse_map[off + 1] = word[i][j] >> 8;
		}
	}

	return 0;
}

int rtl8192f_efuse_get_macaddr(struct rtl8192f_soc_priv *priv)
{
	const u8 *mac = &priv->efuse_map[EFUSE_MAP_MAC_OFFSET];

	if (is_zero_ether_addr(mac) || is_multicast_ether_addr(mac))
		return -ENOENT;

	ether_addr_copy(priv->mac_addr, mac);

	return 0;
}

/* ------------------------------------------------------------------ */
/* BB / RF register access                                             */
/* ------------------------------------------------------------------ */

/* BB registers are memory mapped directly at MAC space 0x800+; the
 * init tables carry the full address including the 0x800 base, so no
 * offset is added here (same convention as phydm ODM_SetBBReg).
 */
void rtl8192f_write_bb_reg(struct rtl8192f_soc_priv *priv, u16 addr, u32 val)
{
	rtl8192f_write32(priv, addr, val);
}

u32 rtl8192f_read_bb_reg(struct rtl8192f_soc_priv *priv, u16 addr)
{
	return rtl8192f_read32(priv, addr);
}

/* RF write: push data[19:0] | addr[27:20] into the HSSI write port.
 * vendor phydm_hal_api8197f.c config_phydm_write_rf_reg_8197f():
 * DataAndAddr = (RegAddr << 20) | (Data & 0xfffff), written to
 * 0x840 (path A).  Single-path part, path B (0x844) unused.
 */
void rtl8192f_phy_set_rf_reg(struct rtl8192f_soc_priv *priv, u8 addr,
			     u32 val)
{
	u32 data_and_addr;

	addr &= 0xff;
	val &= RFREG_MASK;

	data_and_addr = ((u32)addr << 20) | val;
	rtl8192f_write32(priv, RTL8192F_RF_WRITE_A, data_and_addr);
}

/* RF read: the gen-2.5 SI interface exposes a direct read window at
 * BB space 0x2800 + (rf_addr << 2) for path A (0x2c00 path B, unused).
 * vendor phydm_hal_api8197f.c config_phydm_read_rf_reg_8197f().
 */
u32 rtl8192f_phy_get_rf_reg(struct rtl8192f_soc_priv *priv, u8 addr)
{
	u16 reg_addr;

	addr &= 0xff;
	reg_addr = RTL8192F_RF_READ_A + ((u16)addr << 2);

	return rtl8192f_read_bb_reg(priv, reg_addr) & RFREG_MASK;
}
