// SPDX-License-Identifier: GPL-2.0-only
/*
 * RTL8192F-SoC calibration: LC calibration and TX/RX IQ calibration.

 * Ported from the vendor PHYDM 8197F IQK code:
 * phydm/rtl8197f/phydm_iqk_8197f.c  (path A only, 1T1R)

 * Simplifications vs the vendor flow (all noted inline):
 * - single IQK pass, no 3-round simularity comparison / final candidate
 * - path A only (RTL8192F-SoC is 1T1R)
 * - no DPK off/on around IQK, no DPK tracking
 * - no thermal-meter triggered re-calibration (init-time one-shot IQK)
 * - external PA/LNA and cut-version A branches dropped (SoC internal PA)
 * - no continuous-TX / MP-mode guards
 */

#include "rtl8192f_soc.h"

/* ------------------------------------------------------------------ */
/* Vendor register aliases (8812_reg.h / 8192cd_phyreg.h values)       */
/* ------------------------------------------------------------------ */

#define IQK_MODE_REG		0xe28	/* rFPGA0_IQK, mask 0xffffff00 */
#define IQK_MODE_MASK		0xffffff00
#define IQK_MODE_ENTER		0x808000
#define IQK_MODE_LEAVE		0x000000

#define REG_TX_IQK_TONE_A	0xe30	/* rTx_IQK_Tone_A */
#define REG_RX_IQK_TONE_A	0xe34	/* rRx_IQK_Tone_A */
#define REG_TX_IQK_PI_A		0xe38	/* rTx_IQK_PI_A */
#define REG_RX_IQK_PI_A		0xe3c	/* rRx_IQK_PI_A */
#define REG_TX_IQK		0xe40	/* rTx_IQK */
#define REG_RX_IQK		0xe44	/* rRx_IQK */
#define REG_IQK_AGC_PTS		0xe48	/* rIQK_AGC_Pts (one-shot) */
#define REG_IQK_AGC_RSP		0xe4c	/* rIQK_AGC_Rsp */

#define REG_TX_PWR_BEFORE_A	0xe94	/* rTx_Power_Before_IQK_A */
#define REG_TX_PWR_AFTER_A	0xe9c	/* rTx_Power_After_IQK_A */
#define REG_RX_PWR_BEFORE_A2	0xea4	/* rRx_Power_Before_IQK_A_2 */
#define REG_RX_PWR_AFTER_A2	0xeac	/* rRx_Power_After_IQK_A_2 */

#define IQK_DELAY_MS		10	/* IQK_DELAY_TIME_97F, AP branch */

/* MAC registers touched during IQK (vendor IQK_MAC_REG array) */
#define MAC_REG_TXPAUSE		0x0522
#define MAC_REG_BCN_CTRL	0x0550
#define MAC_REG_BCN_CTRL_1	0x0551
#define MAC_REG_GPIO_MUXCFG	0x0040

#define IQK_RETRY_COUNT		2	/* vendor ODM_AP retryCount */

struct rtl8192f_iqk_backup {
	u32	bb_d94;
	u32	bb_e70;
	u32	bb_a04;
	u32	bb_c04;
	u32	bb_c08;
	u32	bb_874;
	u32	bb_c50;
	u8	mac_522;
	u8	mac_550;
	u8	mac_551;
	u8	mac_040;
	u32	rf_55;
};

static void rtl8192f_iqk_mode(struct rtl8192f_soc_priv *priv, bool enter)
{
	rtl8192f_write_bb_reg(priv, IQK_MODE_REG,
			      (rtl8192f_read_bb_reg(priv, IQK_MODE_REG) &
			       ~IQK_MODE_MASK) |
			      (enter ? IQK_MODE_ENTER : IQK_MODE_LEAVE));
}

/* read-modify-write helper: the RF reg API takes no mask */
static void rtl8192f_rf_rmw(struct rtl8192f_soc_priv *priv,
			    u8 addr, u32 mask, u32 val)
{
	u32 v = rtl8192f_phy_get_rf_reg(priv, addr);

	rtl8192f_phy_set_rf_reg(priv, addr, (v & ~mask) | (val & mask));
}

/* ------------------------------------------------------------------ */
/* LC calibration - vendor phy_LCCalibrate_8197F() lines 1933-1991     */
/* ------------------------------------------------------------------ */

static void rtl8192f_lck(struct rtl8192f_soc_priv *priv)
{
	u8 tmp;
	u32 lc_cal;
	int cnt;

	dev_dbg(&priv->pdev->dev, "LCK start\n");

	/* vendor line 1957: check continuous TX vs packet TX */
	tmp = rtl8192f_read8(priv, 0xd03);
	if ((tmp & 0x70) != 0)
		/* disable all continuous TX */
		rtl8192f_write8(priv, 0xd03, tmp & 0x8f);
	else
		/* block all queues */
		rtl8192f_write8(priv, MAC_REG_TXPAUSE, 0xff);

	/* vendor line 1966: backup RF0x18 (RF_CHNLBW) */
	lc_cal = rtl8192f_phy_get_rf_reg(priv, 0x18);

	/* vendor line 1969: start LCK */
	rtl8192f_rf_rmw(priv, 0x18, RFREG_MASK, lc_cal | 0x08000);

	/* vendor lines 1971-1976: wait for bit15 to clear */
	for (cnt = 0; cnt < 100; cnt++) {
		if (!(rtl8192f_phy_get_rf_reg(priv, 0x18) & 0x8000))
			break;
		usleep_range(10000, 11000);
	}

	/* vendor line 1979: recover channel number */
	rtl8192f_rf_rmw(priv, 0x18, RFREG_MASK, lc_cal);

	/* vendor lines 1983-1989: restore original situation */
	if ((tmp & 0x70) != 0)
		rtl8192f_write8(priv, 0xd03, tmp);
	else
		rtl8192f_write8(priv, MAC_REG_TXPAUSE, 0x00);

	dev_dbg(&priv->pdev->dev, "LCK done after %d polls\n", cnt);
}

/* ------------------------------------------------------------------ */
/* TX IQK, path A - vendor phy_PathA_IQK_8197F() lines 338-432         */
/* returns true when TX IQK succeeded                                  */
/* ------------------------------------------------------------------ */

static bool rtl8192f_tx_iqk_path_a(struct rtl8192f_soc_priv *priv)
{
	u32 reg_eac, reg_e94, reg_e9c;
	bool ok;

	/* vendor line 364: leave IQK mode */
	rtl8192f_iqk_mode(priv, false);

	/* vendor lines 370-373: internal PA branch (non ExtPA, non cut A) */
	rtl8192f_rf_rmw(priv, 0xdf, 0x00800, 0x1);
	rtl8192f_rf_rmw(priv, 0x56, 0x003ff, 0xe8);

	/* vendor line 380: enter IQK mode */
	rtl8192f_iqk_mode(priv, true);

	/* vendor lines 383-386: path A tone settings */
	rtl8192f_write_bb_reg(priv, REG_TX_IQK_TONE_A, 0x18008c0c);
	rtl8192f_write_bb_reg(priv, REG_RX_IQK_TONE_A, 0x38008c0c);
	rtl8192f_write_bb_reg(priv, 0xe50, 0x38008c0c);	/* rTx_IQK_Tone_B */
	rtl8192f_write_bb_reg(priv, 0xe54, 0x38008c0c);	/* rRx_IQK_Tone_B */

	/* vendor line 394: TX PI (non cut A value) */
	rtl8192f_write_bb_reg(priv, REG_TX_IQK_PI_A, 0x8214000f);
	/* vendor line 396 */
	rtl8192f_write_bb_reg(priv, REG_RX_IQK_PI_A, 0x28140000);

	/* vendor line 399: LO calibration setting (rIQK_AGC_Rsp).  The
	 * vendor TXIQK writes 0x00e62911; with 0x00862911 the one-shot returns
	 * all-zero measured powers.
	 */
	rtl8192f_write_bb_reg(priv, REG_IQK_AGC_RSP, 0x00e62911);

	/* vendor lines 402-403: one shot, path A LOK & IQK */
	rtl8192f_write_bb_reg(priv, REG_IQK_AGC_PTS, 0xf9005800);
	rtl8192f_write_bb_reg(priv, REG_IQK_AGC_PTS, 0xf8005800);

	usleep_range(IQK_DELAY_MS * 1000, IQK_DELAY_MS * 1000 + 1000);

	/* vendor lines 409-411: read results */
	reg_eac = rtl8192f_read_bb_reg(priv, REG_RX_PWR_AFTER_A2);
	reg_e94 = rtl8192f_read_bb_reg(priv, REG_TX_PWR_BEFORE_A);
	reg_e9c = rtl8192f_read_bb_reg(priv, REG_TX_PWR_AFTER_A);
	/* vendor lines 420-422: reload 0xdf and CCK_IND off */
	rtl8192f_iqk_mode(priv, false);
	rtl8192f_rf_rmw(priv, 0x0, BIT(14), 0x0);
	rtl8192f_rf_rmw(priv, 0xdf, 0x00800, 0x0);

	/* vendor lines 424-427: success check */
	ok = !(reg_eac & BIT(28)) &&
	     ((reg_e94 & 0x03ff0000) >> 16) != 0x142 &&
	     ((reg_e9c & 0x03ff0000) >> 16) != 0x42;
	return ok;
}

/* ------------------------------------------------------------------ */
/* RX IQK, path A - vendor phy_PathA_RxIQK_97F() lines 434-615         */
/* returns 0x01 TX OK, 0x03 TX+RX OK (vendor bit convention)           */
/* ------------------------------------------------------------------ */

static u8 rtl8192f_rx_iqk_path_a(struct rtl8192f_soc_priv *priv)
{
	u32 reg_eac, reg_e94, reg_e9c, reg_ea4, u4tmp;
	u8 result = 0x00;

	/* vendor line 464: leave IQK mode */
	rtl8192f_iqk_mode(priv, false);

	/* ============== step 1: measure TX IMR (vendor lines 459-529) ==== */

	/* vendor line 476: enter IQK mode */
	rtl8192f_iqk_mode(priv, true);

	/* vendor lines 479-486: path A IQK setting */
	rtl8192f_write_bb_reg(priv, REG_TX_IQK_TONE_A, 0x18008c0c);
	rtl8192f_write_bb_reg(priv, REG_RX_IQK_TONE_A, 0x38008c0c);
	rtl8192f_write_bb_reg(priv, 0xe50, 0x38008c0c);
	rtl8192f_write_bb_reg(priv, 0xe54, 0x38008c0c);
	rtl8192f_write_bb_reg(priv, REG_TX_IQK_PI_A, 0x82160000);
	rtl8192f_write_bb_reg(priv, REG_RX_IQK_PI_A, 0x28160000);
	rtl8192f_write_bb_reg(priv, REG_TX_IQK, 0x01007c00);
	rtl8192f_write_bb_reg(priv, REG_RX_IQK, 0x01004800);

	/* vendor line 489 */
	rtl8192f_write_bb_reg(priv, REG_IQK_AGC_RSP, 0x0046a911);

	/* vendor lines 492-493: one shot */
	rtl8192f_write_bb_reg(priv, REG_IQK_AGC_PTS, 0xf9005800);
	rtl8192f_write_bb_reg(priv, REG_IQK_AGC_PTS, 0xf8005800);

	usleep_range(IQK_DELAY_MS * 1000, IQK_DELAY_MS * 1000 + 1000);

	/* vendor lines 500-502 */
	reg_eac = rtl8192f_read_bb_reg(priv, REG_RX_PWR_AFTER_A2);
	reg_e94 = rtl8192f_read_bb_reg(priv, REG_TX_PWR_BEFORE_A);
	reg_e9c = rtl8192f_read_bb_reg(priv, REG_TX_PWR_AFTER_A);
	/* vendor lines 515-525: if TX not OK, ignore RX */
	if ((reg_eac & BIT(28)) ||
	    ((reg_e94 & 0x03ff0000) >> 16) == 0x142 ||
	    ((reg_e9c & 0x03ff0000) >> 16) == 0x42) {
		rtl8192f_iqk_mode(priv, false);
		return result;
	}
	result |= 0x01;

	/* vendor line 527-528: feed back TXIMR */
	u4tmp = 0x80007c00 | (reg_e94 & 0x3ff0000) |
		((reg_e9c & 0x3ff0000) >> 16);
	rtl8192f_write_bb_reg(priv, REG_TX_IQK, u4tmp);

	/* ============== step 2: RX IQK (vendor lines 532-611) ============ */

	/* vendor line 537: leave IQK mode */
	rtl8192f_iqk_mode(priv, false);

	/* vendor lines 555-556 (non cut A) */
	rtl8192f_rf_rmw(priv, 0xdf, 0x00800, 0x1);
	rtl8192f_rf_rmw(priv, 0x56, 0x003e0, 0x3);

	/* vendor line 559: enter IQK mode */
	rtl8192f_iqk_mode(priv, true);

	/* vendor line 562 */
	rtl8192f_write_bb_reg(priv, REG_RX_IQK, 0x01004800);

	/* vendor lines 565-574 (non cut A value for RX PI) */
	rtl8192f_write_bb_reg(priv, REG_TX_IQK_TONE_A, 0x38008c0c);
	rtl8192f_write_bb_reg(priv, REG_RX_IQK_TONE_A, 0x18008c0c);
	rtl8192f_write_bb_reg(priv, 0xe50, 0x38008c0c);
	rtl8192f_write_bb_reg(priv, 0xe54, 0x38008c0c);
	rtl8192f_write_bb_reg(priv, REG_TX_IQK_PI_A, 0x82170000);
	rtl8192f_write_bb_reg(priv, REG_RX_IQK_PI_A, 0x28170000);

	/* vendor line 581 */
	rtl8192f_write_bb_reg(priv, REG_IQK_AGC_RSP, 0x0046a8d1);

	/* vendor lines 584-585: one shot */
	rtl8192f_write_bb_reg(priv, REG_IQK_AGC_PTS, 0xf9005800);
	rtl8192f_write_bb_reg(priv, REG_IQK_AGC_PTS, 0xf8005800);

	usleep_range(IQK_DELAY_MS * 1000, IQK_DELAY_MS * 1000 + 1000);

	/* vendor lines 591-592 */
	reg_eac = rtl8192f_read_bb_reg(priv, REG_RX_PWR_AFTER_A2);
	reg_ea4 = rtl8192f_read_bb_reg(priv, REG_RX_PWR_BEFORE_A2);
	/* vendor lines 600-603: PA/PAD controlled by 0x0 */
	rtl8192f_iqk_mode(priv, false);
	rtl8192f_rf_rmw(priv, 0xdf, 0x00800, 0x0);

	/* vendor lines 605-608: success check */
	if (!(reg_eac & BIT(27)) &&
	    ((reg_ea4 & 0x03ff0000) >> 16) != 0x132 &&
	    ((reg_eac & 0x03ff0000) >> 16) != 0x36)
		result |= 0x02;

	return result;
}

/* ------------------------------------------------------------------ */
/* Write the calibrated IQK matrix - vendor                            */
/* _PHY_PathAFillIQKMatrix_97F(), phydm_iqk_8197f.c 908-1001           */
/* ------------------------------------------------------------------ */

static void rtl8192f_fill_iqk_matrix_path_a(struct rtl8192f_soc_priv *priv,
					    bool tx_ok, bool rx_ok,
					    u32 tx_x, u32 tx_y,
					    u32 rx_x, u32 rx_y)
{
	u32 oldval, x, tx0_a, reg, v;
	s32 y, tx0_c;

	if (!tx_ok)
		return;

	/* Live vendor _PHY_PathAFillIQKMatrix_97F (phydm_iqk_8197f.c
	 * 963-985).  The rOFDM0_XATxIQImbalance matrix form is #if 0'd right
	 * above it (934-962): the 97F writes the TX IQK result into the
	 * IQ-control registers instead - 0xe30[31:22] for the I branch,
	 * 0xe20[9:0] for the Q branch (the vendor's two masked writes,
	 * 0x3C0 then 0x3F, are the same ten bits) - scaling by 0x100>>8, i.e.
	 * by one, and copying the old 0xc80[31:22] field back over
	 * 0xc80[9:0] so the legacy matrix path stays neutral.  Writing the
	 * #if 0 branch instead, as this file did until now, means every TX
	 * IQK result landed in registers the 97F never reads.
	 */
	v = rtl8192f_read_bb_reg(priv, 0xc80);
	oldval = (v >> 22) & 0x3ff;
	rtl8192f_write_bb_reg(priv, 0xc80, (v & ~0x3ff) | oldval);

	x = tx_x;
	if (x & 0x200)
		x |= 0xfffffc00;	/* sign-extend the 10-bit result */
	tx0_a = (x * 0x100) >> 8;
	v = rtl8192f_read_bb_reg(priv, 0xe30);
	rtl8192f_write_bb_reg(priv, 0xe30,
			      (v & ~0x3ff00000) | ((tx0_a & 0x3ff) << 22));

	y = (s32)tx_y;
	if (y & 0x200)
		y |= (s32)0xfffffc00;
	tx0_c = (y * 0x100) >> 8;
	v = rtl8192f_read_bb_reg(priv, 0xe20);
	rtl8192f_write_bb_reg(priv, 0xe20, (v & ~0x3ff) | (tx0_c & 0x3ff));

	if (!rx_ok)
		return;

	/* RX IQK matrix, with the vendor AP-branch clamp on RX X. */
	reg = rx_x;
	if (abs((s32)reg - 0x100) >= 16)
		reg = 0x100;
	rtl8192f_write_bb_reg(priv, 0xc14,	/* rOFDM0_XARxIQImbalance */
			      (rtl8192f_read_bb_reg(priv, 0xc14) & ~0x3ff) |
			      (reg & 0x3ff));
	rtl8192f_write_bb_reg(priv, 0xc14,
			      (rtl8192f_read_bb_reg(priv, 0xc14) & ~0xfc00) |
			      ((rx_y & 0x3f) << 10));
	rtl8192f_write_bb_reg(priv, 0xca0,	/* rOFDM0_RxIQExtAnta */
			      (rtl8192f_read_bb_reg(priv, 0xca0) &
			       ~0xf0000000) |
			      (((rx_y >> 6) & 0xf) << 28));
}

/* ------------------------------------------------------------------ */
/* Full calibration sequence                                           */
/* ------------------------------------------------------------------ */

/**
 * rtl8192f_iqk - run LC calibration and a one-shot TX/RX IQK on path A
 * @priv: driver private data
 *
 * Simplified init-time calibration: a single IQK pass is run and the
 * result is applied immediately (no multi-round simularity voting, no
 * thermal re-calibration, no DPK, path B omitted for 1T1R).
 */
void rtl8192f_iqk(struct rtl8192f_soc_priv *priv)
{
	struct rtl8192f_iqk_backup bak;
	bool tx_ok = false, rx_ok = false;
	u32 tx_x = 0x100, tx_y = 0, rx_x = 0x100, rx_y = 0;
	u32 val;
	int i;

	/* LCK first, as the vendor brings up LCK before IQK */
	rtl8192f_lck(priv);

	dev_dbg(&priv->pdev->dev, "IQK start (path A, single pass)\n");

	/* Save registers we will modify (vendor _PHY_SaveADDARegisters_97F
	 * / _PHY_SaveMACRegisters_97F, lines 1130-1192, reduced set)
	 */
	bak.bb_d94 = rtl8192f_read_bb_reg(priv, 0xd94);
	bak.bb_e70 = rtl8192f_read_bb_reg(priv, 0xe70);
	bak.bb_a04 = rtl8192f_read_bb_reg(priv, 0xa04);
	bak.bb_c04 = rtl8192f_read_bb_reg(priv, 0xc04);
	bak.bb_c08 = rtl8192f_read_bb_reg(priv, 0xc08);
	bak.bb_874 = rtl8192f_read_bb_reg(priv, 0x874);
	bak.bb_c50 = rtl8192f_read_bb_reg(priv, 0xc50);
	bak.mac_522 = rtl8192f_read8(priv, MAC_REG_TXPAUSE);
	bak.mac_550 = rtl8192f_read8(priv, MAC_REG_BCN_CTRL);
	bak.mac_551 = rtl8192f_read8(priv, MAC_REG_BCN_CTRL_1);
	bak.mac_040 = rtl8192f_read8(priv, MAC_REG_GPIO_MUXCFG);
	bak.rf_55 = rtl8192f_phy_get_rf_reg(priv, 0x55);

	/* vendor _PHY_PathADDAOn_97F(), lines 1285-1286: ADDA on */
	rtl8192f_write_bb_reg(priv, 0xd94, (bak.bb_d94 & ~0x00ff0000) | 0x00ff0000);
	rtl8192f_write_bb_reg(priv, 0xe70, 0x00400040);

	/* vendor phy_IQCalibrate_8197F() lines 1670-1671: DAC gain low */
	rtl8192f_rf_rmw(priv, 0x55, 0x000e0, 0x0);

	/* vendor lines 1706-1709: BB settings for IQK */
	rtl8192f_write_bb_reg(priv, 0xa04,	/* rCCK0_AFESetting */
			      (bak.bb_a04 & ~0x0f000000) | 0x0f000000);
	rtl8192f_write_bb_reg(priv, 0xc04, 0x6f005403);	/* rOFDM0_TRxPathEnable */
	rtl8192f_write_bb_reg(priv, 0xc08, 0x000804e4);	/* rOFDM0_TRMuxPar */
	rtl8192f_write_bb_reg(priv, 0x874, 0x04203400);	/* rFPGA0_XCD_RFInterfaceSW */

	/* vendor _PHY_MACSettingCalibration_97F(), line 1336 */
	rtl8192f_write8(priv, 0x520 + 2, 0xff);

	/* vendor lines 1736-1738: initial IQK trigger settings */
	rtl8192f_iqk_mode(priv, true);
	rtl8192f_write_bb_reg(priv, REG_TX_IQK, 0x01007c00);
	rtl8192f_write_bb_reg(priv, REG_RX_IQK, 0x01004800);

	/* vendor lines 1743-1767: path A TXIQK with retry */
	for (i = 0; i < IQK_RETRY_COUNT && !tx_ok; i++) {
		tx_ok = rtl8192f_tx_iqk_path_a(priv);
		if (tx_ok) {
			tx_x = (rtl8192f_read_bb_reg(priv, REG_TX_PWR_BEFORE_A) &
				0x3ff0000) >> 16;
			tx_y = (rtl8192f_read_bb_reg(priv, REG_TX_PWR_AFTER_A) &
				0x3ff0000) >> 16;
		}
	}
	/* vendor lines 1772-1791: path A RXIQK with retry */
	for (i = 0; i < IQK_RETRY_COUNT; i++) {
		val = rtl8192f_rx_iqk_path_a(priv);
		if (val == 0x03) {
			rx_ok = true;
			rx_x = (rtl8192f_read_bb_reg(priv, REG_RX_PWR_BEFORE_A2) &
				0x3ff0000) >> 16;
			rx_y = (rtl8192f_read_bb_reg(priv, REG_RX_PWR_AFTER_A2) &
				0x3ff0000) >> 16;
			break;
		}
	}
	/* vendor line 1875: back to BB mode */
	rtl8192f_iqk_mode(priv, false);

	/* Restore everything saved above (vendor reload path, lines
	 * 1877-1928, reduced set).  Restore RX initial gain via the
	 * vendor 0xc50 trick: write 0x50 then the old value.
	 */
	rtl8192f_write_bb_reg(priv, 0xd94, bak.bb_d94);
	rtl8192f_write_bb_reg(priv, 0xe70, bak.bb_e70);
	rtl8192f_write_bb_reg(priv, 0xa04, bak.bb_a04);
	rtl8192f_write_bb_reg(priv, 0xc04, bak.bb_c04);
	rtl8192f_write_bb_reg(priv, 0xc08, bak.bb_c08);
	rtl8192f_write_bb_reg(priv, 0x874, bak.bb_874);
	rtl8192f_write8(priv, MAC_REG_TXPAUSE, bak.mac_522);
	rtl8192f_write8(priv, MAC_REG_BCN_CTRL, bak.mac_550);
	rtl8192f_write8(priv, MAC_REG_BCN_CTRL_1, bak.mac_551);
	rtl8192f_write8(priv, MAC_REG_GPIO_MUXCFG, bak.mac_040);

	rtl8192f_write_bb_reg(priv, 0xc50, (bak.bb_c50 & ~0xff) | 0x50);
	rtl8192f_write_bb_reg(priv, 0xc50, bak.bb_c50);

	/* vendor lines 1914-1916: reload DAC gain (K-free) */
	rtl8192f_rf_rmw(priv, 0x55, RFREG_MASK, bak.rf_55);

	/* Apply the calibration results (vendor lines 2264-2270 path A).
	 * Defaults X=0x100 / Y=0 match the vendor fail-safe values.
	 */
	rtl8192f_fill_iqk_matrix_path_a(priv, tx_ok, rx_ok,
					tx_x, tx_y, rx_x, rx_y);
}
