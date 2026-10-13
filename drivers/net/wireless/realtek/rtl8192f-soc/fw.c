// SPDX-License-Identifier: GPL-2.0-only
/*
 * RTL8192F-SoC (RTL8197F) core MIPS CPU firmware download.
 *
 * The "8197F WiFi core" contains a MIPS CPU of its own whose firmware
 * runs the in-core transmit scheduler / frame-exchange engine.  On a
 * cold SoC the CPU sits in reset (REG_SYS_FUNC_EN bit10 = 0,
 * REG_MCUFWDL bit15 FW_INIT_RDY = 0) and then nothing consumes the
 * H2C mailbox, so the per-station rate tables (RA_MASK) cannot be
 * programmed; the descriptor-driven data path itself keeps working
 * without the image.  Loading it mirrors the vendor
 * InitMIPSFirmware88XX() sequence (WlanHAL/RTL88XX/
 * Hal88XXFirmware.c: ConfigBeforeDLFW / DLtoTXBUFandDDMA88XX /
 * checkFWSizeAndDLFW88XX / RestoreAfterDLFW):
 *
 *   host memory --BCN TXBD--> TXPKTBUF --DDMA ch0--> IMEM / DMEM
 *
 * The chunk travels over the beacon TXBD path (40B TX info descriptor
 * + payload landed at beacon header page 0 of the TX FIFO), then the
 * on-chip DDMA channel 0 copies it from the TX buffer OCP window
 * (OCPBASE_TXBUF + 40) into the core's IMEM/DMEM.  Sections are
 * checksummed by the DDMA itself (DDMA_CHKSUM_EN, fail flag in
 * REG_DDMA_CH0CTRL bit27); each section ends with 8 checksum dummy
 * bytes accounted for in the header sizes.
 *
 * The image itself is not part of the kernel tree: it is redistributable
 * only under Realtek's proprietary licence, so it is requested from
 * userspace as RTL8192F_FW_NAME below (linux-firmware).
 */
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/unaligned.h>

#include "rtl8192f_soc.h"

/* Requested from userspace at interface start (.start() can sleep).  The
 * name is where linux-firmware installs the image.
 */
#define RTL8192F_FW_NAME		"rtlwifi/rtl8197ffw.bin"
MODULE_FIRMWARE(RTL8192F_FW_NAME);

/* TX info descriptor fields (same layout as txrx.c; the beacon-slot
 * descriptor is reused as the firmware chunk carrier).
 */
#define FW_TXD_W0_TXPKTSIZE	GENMASK(15, 0)
#define FW_TXD_W0_OFFSET	GENMASK(23, 16)
#define FW_TXD_W0_BMC		BIT(24)
#define FW_TXD_W1_QSEL		GENMASK(12, 8)
#define FW_TXD_W3_USE_RATE	BIT(8)
#define FW_TXD_W2_G_ID		GENMASK(29, 24)
#define FW_TXD_W3_DISDATAFB	BIT(10)

#define REG_FWHW_TXQ_CTRL	0x0420

/* TXBD beacon-slot DW0 [30:16] PSLEN / bit31 OWN (txrx.c TXBD_BCN_*). */
#define FW_TXBD_BCN_OWN		BIT(15)

/* RTL88XX_MIPS_FW_HDR (WlanHAL/RTL88XX/Hal88XXFirmware.h), all LE. */
#define FW_HDR_SIZE		64
#define FW_CHKSUM_DUMMY_SZ	8
#define FW_MAX_CHUNK		0x8000	/* MIPS_MAX_FWBLOCK_DL_SIZE */

#define FW_SIG_8197F		0x8197

/* REG_MCUFWDL (0x80) */
#define MCUFWDL_EN		BIT(0)
#define IMEM_DL_RDY		BIT(3)
#define IMEM_CHKSUM_FAIL	BIT(4)
#define DMEM_DL_RDY		BIT(5)
#define DMEM_CHKSUM_FAIL	BIT(6)
#define FW_DW_RDY		BIT(14)
#define FW_INIT_RDY		BIT(15)
#define CPU_CLK_SEL_40M		BIT(12)	/* CPU_CLK_SEL = 1: core 40MHz */

/* REG_SYS_FUNC_EN (0x02) */
#define FEN_CPUEN		BIT(10)

/* REG_CR (0x100) */
#define CR_ENSWBCN		BIT(8)

/* REG_BCN_CTRL (0x550) */
#define BCN_EN_BCN_FUNCTION	BIT(3)
#define BCN_DIS_TSF_UDT		BIT(4)

/* REG_CPU_DMEM_CON (0x1080): bit16 = WLAN platform reset (97F: pulse
 * it before the download or the core raises a platform error int).
 */
#define REG_CPU_DMEM_CON	0x1080
#define CPU_DMEM_CON_RST	BIT(16)

/* DDMA channel 0 (HalCfg.h / HalComReg.h).  Source and destination are
 * OCP-space addresses, not host bus addresses.
 */
#define REG_DDMA_CH0SA		0x1200
#define REG_DDMA_CH0DA		0x1204
#define REG_DDMA_CH0CTRL	0x1208

#define DDMA_CH_OWN		BIT(31)
#define DDMA_CHKSUM_FAIL	BIT(27)
#define DDMA_CH_CHKSUM_CNT	BIT(24)
#define DDMA_CHKSUM_EN		BIT(29)
#define DDMA_LEN_MASK		0x0001ffff

#define OCPBASE_IMEM		0x00000000
#define OCPBASE_DMEM		0x00200000
#define OCPBASE_TXBUF		0x18780000

/* Beacon download trigger (DownloadRsvdPage88XX): REG_RX_RXBD_NUM+1
 * bit4 is a self-clearing "fetch the armed beacon slot" kick.
 */
#define REG_BCN_DL_TRIGGER	0x0383
#define BCN_DL_TRIGGER		BIT(4)

/* Fill the resident BCN TXBD slot with one firmware chunk and wait
 * for the TXDMA to swallow it into the TX FIFO (vendor
 * SigninBeaconTXBD88XX + DownloadRsvdPage88XX).  Desc fields follow
 * FillBeaconDesc88XX: BMC, OFFSET 40, TXPKTSIZE, QSEL_BEACON,
 * USERATE|DISDATAFB; the rate fields stay 0 (the frame never goes on
 * the air, only the page content matters).
 */
static int fw_dl_chunk(struct rtl8192f_soc_priv *priv,
		       const u8 *buf, u32 len, u32 ocp_dst)
{
	struct rtl8192f_bcn_slot *bcn = &priv->bcn;
	struct rtl8192f_txbd *bd = bcn->bd;
	__le32 *desc = bcn->txdesc;
	dma_addr_t dma;
	u16 pslen;
	unsigned int i;

	dma = dma_map_single(&priv->pdev->dev, (void *)buf, len,
			     DMA_TO_DEVICE);
	if (dma_mapping_error(&priv->pdev->dev, dma))
		return -EIO;

	pslen = (RTL8192F_TX_INFO_DESC_SIZE + len +
		 RTL8192F_TX_PAGE_SIZE - 1) / RTL8192F_TX_PAGE_SIZE;

	memset(desc, 0, RTL8192F_TX_INFO_DESC_SIZE);
	desc[0] = cpu_to_le32(le32_encode_bits(len,
					       FW_TXD_W0_TXPKTSIZE) |
			      le32_encode_bits(RTL8192F_TX_INFO_DESC_SIZE,
					       FW_TXD_W0_OFFSET) |
			      FW_TXD_W0_BMC);
	desc[1] = cpu_to_le32(le32_encode_bits(RTL8192F_QSEL_BEACON,
					       FW_TXD_W1_QSEL));
	desc[2] = cpu_to_le32(le32_encode_bits(0x3f, FW_TXD_W2_G_ID));
	desc[3] = cpu_to_le32(FW_TXD_W3_USE_RATE | FW_TXD_W3_DISDATAFB);
	/* W7 TXBUFF = chunk length (vendor FillBeaconDesc88XX 2006) */
	desc[7] = cpu_to_le32(len);

	spin_lock_bh(&priv->irq_lock);
	bd->ele[1].buf_size = cpu_to_le16(len);
	bd->ele[1].dma = cpu_to_le32(dma);
	bd->ele[0].psb_len = cpu_to_le16(pslen);
	/* desc + payload must be visible before OWN arms the slot */
	wmb();
	bd->ele[0].psb_len |= cpu_to_le16(FW_TXBD_BCN_OWN);
	spin_unlock_bh(&priv->irq_lock);

	/* Kick and wait for the self-clearing trigger (byte register -
	 * vendor polls 100 x 10us; the HW clears bit4 once the slot is
	 * consumed).
	 */
	rtl8192f_write8(priv, REG_BCN_DL_TRIGGER,
			rtl8192f_read8(priv, REG_BCN_DL_TRIGGER) |
			BCN_DL_TRIGGER);
	for (i = 0; i < 100; i++) {
		if (!(rtl8192f_read8(priv, REG_BCN_DL_TRIGGER) &
		      BCN_DL_TRIGGER))
			break;
		usleep_range(10, 20);
	}
	if (i == 100) {
		dev_err(&priv->pdev->dev,
			"fw: beacon download trigger timeout (len=%u)\n", len);
		dma_unmap_single(&priv->pdev->dev, dma, len, DMA_TO_DEVICE);
		return -ETIMEDOUT;
	}

	/* DDMA: TXBUF+40 (past the info descriptor) -> IMEM/DMEM. */
	for (i = 0; i < 1000; i++) {
		if (!(rtl8192f_read32(priv, REG_DDMA_CH0CTRL) & DDMA_CH_OWN))
			break;
		usleep_range(1000, 2000);
	}
	if (i == 1000) {
		dev_err(&priv->pdev->dev, "fw: ddma busy timeout\n");
		dma_unmap_single(&priv->pdev->dev, dma, len, DMA_TO_DEVICE);
		return -ETIMEDOUT;
	}

	rtl8192f_write32(priv, REG_DDMA_CH0SA, OCPBASE_TXBUF +
			 RTL8192F_TX_INFO_DESC_SIZE);
	rtl8192f_write32(priv, REG_DDMA_CH0DA, ocp_dst);
	rtl8192f_write32(priv, REG_DDMA_CH0CTRL,
			 DDMA_CHKSUM_EN | DDMA_CH_CHKSUM_CNT | DDMA_CH_OWN |
			 (len & DDMA_LEN_MASK));

	for (i = 0; i < 1000; i++) {
		if (!(rtl8192f_read32(priv, REG_DDMA_CH0CTRL) & DDMA_CH_OWN))
			break;
		usleep_range(1000, 2000);
	}
	dma_unmap_single(&priv->pdev->dev, dma, len, DMA_TO_DEVICE);
	if (i == 1000) {
		dev_err(&priv->pdev->dev, "fw: ddma transfer timeout\n");
		return -ETIMEDOUT;
	}
	/* Note: the CHECKSUM_FAIL bit is a per-transfer verdict - it
	 * also sets for every chunk that does not end on the section's
	 * trailing 8 checksum dummy bytes, so it must only be sampled
	 * at section end (vendor DLImageCheckSum, called once after
	 * checkFWSizeAndDLFW88XX consumed the whole section).
	 */
	return 0;
}

/* Re-apply only the bits the download window moved.
 *
 * Writing back the value read *before* the download reverts whatever the
 * in-core scheduler changed in the meantime.  That is not hypothetical:
 * REG_CR's bit10 (BIT_32K_CAL_TMR_EN, halmac_bit2.h:13128) differs between two
 * stall dumps on this board (0x00ff and 0x04ff) and no host path writes it, so
 * the core does.  REG_CR also sits one halfword in front of REG_PBP (0x104), so
 * it must never be touched as a dword - see the 16-bit RMW note in core.c.
 */
static void fw_set_clear_bits32(struct rtl8192f_soc_priv *priv, u16 addr,
				u32 set, u32 clear)
{
	rtl8192f_write32(priv, addr,
			 (rtl8192f_read32(priv, addr) & ~clear) | set);
}

/* One IMEM/DMEM section, chunked; after each section the MCUFWDL
 * status bits are set per vendor (DMEM_DL_RDY / IMEM_DL_RDY).
 */
static int fw_dl_section(struct rtl8192f_soc_priv *priv,
			 const u8 *buf, u32 len, u32 base, bool is_imem)
{
	u32 offset = 0;
	u8 rdy = is_imem ? IMEM_DL_RDY : DMEM_DL_RDY;
	u8 fail = is_imem ? IMEM_CHKSUM_FAIL : DMEM_CHKSUM_FAIL;
	u8 *stage = NULL;
	int ret = 0;

	/* request_firmware() hands out the image as a vmalloc mapping, and
	 * the DMA API refuses vmalloc addresses here, so put the section in
	 * the linear map once instead of per chunk.
	 */
	if (is_vmalloc_addr(buf)) {
		stage = kmalloc(len, GFP_KERNEL);
		if (!stage)
			return -ENOMEM;
		memcpy(stage, buf, len);
		buf = stage;
	}

	while (offset < len) {
		u32 chunk = min(len - offset, (u32)FW_MAX_CHUNK);

		ret = fw_dl_chunk(priv, buf + offset, chunk, base + offset);
		if (ret)
			goto out;

		offset += chunk;
		/* Vendor delays 10ms between blocks. */
		mdelay(10);
	}

	/* Section-end checksum verdict (vendor DLImageCheckSum). */
	if (rtl8192f_read32(priv, REG_DDMA_CH0CTRL) & DDMA_CHKSUM_FAIL) {
		dev_err(&priv->pdev->dev,
			"fw: section checksum fail (base=0x%08x, chksum=0x%04x)\n",
			base, rtl8192f_read16(priv, 0x12f0));
		rtl8192f_write8(priv, REG_MCUFWDL,
				rtl8192f_read8(priv, REG_MCUFWDL) | fail);
		ret = -EILSEQ;
		goto out;
	}

	rtl8192f_write8(priv, REG_MCUFWDL,
			(rtl8192f_read8(priv, REG_MCUFWDL) & ~fail) | rdy);
out:
	kfree(stage);
	return ret;
}

int rtl8192f_fw_download(struct rtl8192f_soc_priv *priv)
{
	const struct firmware *fw;
	u32 saved[5];
	const u8 *dmem, *imem;
	u32 dmem_sz, imem_sz, total;
	u16 cfg_sz;
	u32 iram_sz;
	int ret;

	/* The download runs on every bring-up.  .start() pulses the platform reset
	 * first (rtl8192f_platform_reset()), so the core is not live here and
	 * ConfigBeforeDLFW is safe to re-run; skipping it would drop state that
	 * only the download path establishes.  Measured: REG_CR's
	 * BIT_32K_CAL_TMR_EN - no host path writes it, so the core does - read
	 * 0x00ff at both halts taken after a restart by a build that skipped the
	 * download (1125, 2197 s), and 0x04ff (0x06ff once a pairwise key was
	 * installed) at all ten halts taken by builds that re-download, six of those
	 * taken after a restart.  The vendor matches the re-download shape: it
	 * downloads on every init while load_fw is set (8192cd_hw.c:15696) and
	 * clears bFWReady at the entry of its download routine rather than
	 * treating an earlier download as a reason to skip
	 * (Hal88XXFirmware.c:488).
	 *
	 * Unlike the vendor, fw_ready is deliberately left alone on the
	 * failure path below: a bring-up whose download fails still runs the
	 * image the core already holds, because nothing here clears its
	 * instruction memory.  Measured: with the firmware file moved away and
	 * the interface bounced, H2C commands were still acknowledged and a
	 * 1 MB read came back over the air.  The flag describes the core, not
	 * this bring-up.
	 */
	ret = request_firmware(&fw, RTL8192F_FW_NAME, &priv->pdev->dev);
	if (ret) {
		dev_info(&priv->pdev->dev,
			 "fw: %s not available (%d); this bring-up loaded no image\n",
			 RTL8192F_FW_NAME, ret);
		return ret;
	}

	total = fw->size;
	if (total <= FW_HDR_SIZE) {
		dev_err(&priv->pdev->dev, "fw: image too small (%u)\n", total);
		ret = -EINVAL;
		goto out;
	}
	if (get_unaligned_le16(fw->data) != FW_SIG_8197F) {
		dev_err(&priv->pdev->dev, "fw: bad signature 0x%04x\n",
			get_unaligned_le16(fw->data));
		ret = -EINVAL;
		goto out;
	}

	cfg_sz = get_unaligned_le16(fw->data + 36);
	iram_sz = get_unaligned_le32(fw->data + 48);
	dmem_sz = cfg_sz + FW_CHKSUM_DUMMY_SZ;
	imem_sz = iram_sz + FW_CHKSUM_DUMMY_SZ;

	if (total != FW_HDR_SIZE + dmem_sz + imem_sz) {
		dev_err(&priv->pdev->dev,
			"fw: size mismatch img=%u hdr+%u+%u=%u\n",
			total, dmem_sz, imem_sz,
			FW_HDR_SIZE + dmem_sz + imem_sz);
		ret = -EINVAL;
		goto out;
	}

	dmem = fw->data + FW_HDR_SIZE;
	imem = dmem + dmem_sz;

	/* vendor 8192cd_hw.c:15033 writes 0x7d4=0x90 unconditionally in
	 * init, BEFORE the firmware download takes over the register: once
	 * the core runs, host writes to byte0 are ignored (reads stick at
	 * 0x5080a000).  Vendor self-note at Hal88XXGen.c:5372 calls this
	 * "set ps detect enable".
	 */
	rtl8192f_write8(priv, 0x7d4, 0x90);

	/* ---- ConfigBeforeDLFW: park the beacon engine, reset the
	 * core CPU, put 0x80 into download mode.
	 */
	saved[0] = rtl8192f_read16(priv, REG_CR);
	rtl8192f_write16(priv, REG_CR, saved[0] | CR_ENSWBCN);

	saved[1] = rtl8192f_read32(priv, REG_BCN_CTRL);
	rtl8192f_write32(priv, REG_BCN_CTRL,
			 (saved[1] | BCN_DIS_TSF_UDT) & ~BCN_EN_BCN_FUNCTION);

	saved[2] = rtl8192f_read32(priv, REG_FWHW_TXQ_CTRL);
	rtl8192f_write32(priv, REG_FWHW_TXQ_CTRL, saved[2] & ~BIT(22));

	/* Beacon header page = 0 (tx free tail must not rewrite it -
	 * FWHW_TXQ_CTRL bit22 above - while we own the page).
	 */
	saved[3] = rtl8192f_read32(priv, 0x0204);
	rtl8192f_write16(priv, 0x0204, 0);

	saved[4] = rtl8192f_read16(priv, REG_SYS_FUNC_EN);
	rtl8192f_write16(priv, REG_SYS_FUNC_EN, saved[4] & ~FEN_CPUEN);

	/* HCI_MIX_CFG: multi-tag off. */
	rtl8192f_write8(priv, 0x03fc, 0);

	/* Vendor ConfigBeforeDLFW -> HW_VAR_ENABLE_BEACON_DMA (8197F):
	 * the beacon fetch engine must be running for the TXBD slot to
	 * be consumed.  txrx_start left STOP_BCNQ set.
	 */
	/* Clear the status/mod fields, arm the download enable and
	 * select the 40MHz core clock (8197F; vendor ConfigBeforeDLFW).
	 */
	rtl8192f_write32(priv, REG_MCUFWDL,
			 (rtl8192f_read32(priv, REG_MCUFWDL) &
			  ~(u32)0x7ff &
			  ~(u32)(FW_DW_RDY | FW_INIT_RDY) &
			  ~(u32)0x3000) |
			 MCUFWDL_EN | CPU_CLK_SEL_40M);

	/* 97F: WLAN platform reset pulse before the download. */
	rtl8192f_write32(priv, REG_CPU_DMEM_CON,
			 rtl8192f_read32(priv, REG_CPU_DMEM_CON) &
			 ~CPU_DMEM_CON_RST);
	rtl8192f_write32(priv, REG_CPU_DMEM_CON,
			 rtl8192f_read32(priv, REG_CPU_DMEM_CON) |
			 CPU_DMEM_CON_RST);

	/* ---- Download DMEM then IMEM. */
	ret = fw_dl_section(priv, dmem, dmem_sz, OCPBASE_DMEM, false);
	if (!ret)
		ret = fw_dl_section(priv, imem, imem_sz, OCPBASE_IMEM, true);

	/* ---- RestoreAfterDLFW. */
	spin_lock_bh(&priv->irq_lock);
	priv->bcn.bd->ele[0].psb_len &= cpu_to_le16(~FW_TXBD_BCN_OWN);
	spin_unlock_bh(&priv->irq_lock);

	rtl8192f_write16(priv, 0x0204, saved[3] & 0xffff);
	rtl8192f_write16(priv, REG_CR,
			 (rtl8192f_read16(priv, REG_CR) & ~CR_ENSWBCN) |
			 (saved[0] & CR_ENSWBCN));
	fw_set_clear_bits32(priv, REG_BCN_CTRL,
			    saved[1] & BCN_EN_BCN_FUNCTION, BCN_DIS_TSF_UDT);
	if (saved[2] & BIT(22))
		rtl8192f_setbits32(priv, REG_FWHW_TXQ_CTRL, BIT(22));
	rtl8192f_write16(priv, REG_SYS_FUNC_EN,
			 rtl8192f_read16(priv, REG_SYS_FUNC_EN) |
			 (saved[4] & FEN_CPUEN));
	/* The MCUFWDL_EN clear used to live only on the success path: a
	 * chunk timeout or checksum failure left the core in "download
	 * mode" permanently, and every later txrx_start inherited that
	 * half-started state.  Clear it unconditionally so a failed
	 * download really does fall back to the mask ROM.
	 */
	rtl8192f_write8(priv, REG_MCUFWDL,
			rtl8192f_read8(priv, REG_MCUFWDL) & ~MCUFWDL_EN);

	if (ret)
		goto out;

	/* Let the core run and wait for its init-ready flag. */
	rtl8192f_write16(priv, REG_SYS_FUNC_EN,
			 rtl8192f_read16(priv, REG_SYS_FUNC_EN) | FEN_CPUEN);
	msleep(100);

	if (!(rtl8192f_read32(priv, REG_MCUFWDL) & FW_INIT_RDY)) {
		dev_err(&priv->pdev->dev,
			"fw: MIPS CPU not running (0x80=0x%08x)\n",
			rtl8192f_read32(priv, REG_MCUFWDL));
		ret = -ETIMEDOUT;
		goto out;
	}

	priv->fw_ready = true;
	dev_info(&priv->pdev->dev,
		 "fw: MIPS CPU running, version 0x%04x.%02x %02d-%02d %02d:%02d\n",
		 get_unaligned_le16(fw->data + 4),
		 fw->data[6], fw->data[16],
		 fw->data[17], fw->data[18], fw->data[19]);
	ret = 0;
out:
	release_firmware(fw);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Host -> core firmware command mailbox (H2C)                         */
/* ------------------------------------------------------------------ */
/*
 * The in-core MIPS scheduler owns one control block per macid (rate
 * set id, bandwidth, SGI, power state, drop flag).  The vendor fills
 * that block in with the RA_MASK command on every rate-mask update:
 * UpdateHalRAMask88XX() builds a 7-byte payload and hands it to
 * FillH2CCmd88XX(H2C_88XX_RA_MASK, 7, ..)
 * (Hal88XXFirmware.c 1531-1562), and that handler is bound for our
 * chip (Hal8197FGen.c:341).  Without it the firmware has never been
 * told anything about the macid our unicast descriptors name.
 *
 * Four boxes at 0x1D0..0x1DC, each with a 4-byte extension at
 * 0x1F0..0x1FC.  A command of 4..7 bytes is split as payload[0..2] into
 * the box after its ID byte and payload[3..6] into the extension,
 * extension first (FillH2CCmd88XX 2470-2525).
 *
 * REG_HMETFR (0x1CC) bit n is documented as "the firmware still holds
 * box n" and the vendor waits for it to drop (CheckFwReadLastH2C88XX,
 * Hal88XXFirmware.c 2356-2370).  Measured on this board with traffic
 * running: writing all four boxes leaves 0x0f and the bits then fall back
 * on their own (0x04 after one command, 0x01 in a quiet beat, 0x00 in
 * others), so the firmware does read them.  What persists is the box
 * *content* - 0x1d0 still held 0x40810140, an old RA_MASK, long after it
 * was taken - so content says nothing about delivery; this driver gates on
 * FWISR instead.
 */
#define H2C_88XX_RA_MASK	0x40
/* The 88XX family also has H2C_88XX_AP_REQ_TXREP (0x43, vendor
 * Hal88XXFirmware.h 209), answered by C2H id C2H_88XX_TX_RATE (4) with the
 * per-MACID success/failure totals (8192cd_sme.c:7616-7634, decode
 * 8192cd_util.c:8345-8374).  It is not sent here for its answer, and the
 * verdict was re-made with a valid predicate: three requests on the board
 * were each acknowledged (REG_FWISR FS_H2CCMD) and moved nothing - the
 * RX-side C2H count stayed 0, REG_C2HEVT (0x1A0..0x1AF, the vendor's other
 * C2H window, HalComReg.h:279) read back all zero, and neither
 * FS_TRXRPT_INT nor FS_C2H_W_READY asserted in REG_FWISR.  Every route a
 * report could arrive on is empty, so there is no aggregate to decode.
 * What is useful is exactly that dead end: a command nobody consumes is a
 * question with a harmless content, so its acknowledgment serves as the
 * liveness test (rtl8192f_fw_alive).
 */
#define H2C_88XX_AP_REQ_TXREP	0x43
/*
 * Did the firmware take the command we just published?
 *
 * The vendor waits on the REG_HMETFR box-full bit
 * (CheckFwReadLastH2C88XX, Hal88XXFirmware.c 2356-2370) because on its
 * chips the scheduler clears it after reading the box.  That bit moves here
 * too, but it is not a usable gate for us: a box is reused before the flag
 * of the previous command has necessarily fallen, so waiting for "the bit I
 * just set went away" would race our own rotation.  REG_FWISR's FS_H2CCMD
 * bit, by contrast, re-asserts on every box write and stays clear while
 * nothing is written, which is what this loop polls.
 *
 * Budget is the vendor's 1200 polls of 10 us (FillH2CCmd88XX
 * Hal88XXFirmware.c 2446-2468).
 */
static int h2c_cmd_taken(struct rtl8192f_soc_priv *priv)
{
	int i;

	for (i = 0; i < 1200; i++) {
		if (rtl8192f_read32(priv, REG_FWISR) & FWISR_H2CCMD) {
			rtl8192f_write32(priv, REG_FWISR, FWISR_H2CCMD);
			return 0;
		}
		usleep_range(10, 20);
	}

	return -ETIMEDOUT;
}

int rtl8192f_h2c_send(struct rtl8192f_soc_priv *priv, u8 id,
		      const u8 *payload, u8 len)
{
	u8 msg[4] = { id }, ext[4] = { 0 };
	u32 boxv, extv;
	u8 box;
	int ret;

	if (len > 7)
		return -E2BIG;
	if (!priv->fw_ready)
		return -ENODEV;

	memcpy(msg + 1, payload, min_t(u8, len, 3));
	if (len > 3)
		memcpy(ext, payload + 3, len - 3);
	boxv = msg[0] | (msg[1] << 8) | (msg[2] << 16) | (msg[3] << 24);
	extv = ext[0] | (ext[1] << 8) | (ext[2] << 16) | (ext[3] << 24);

	mutex_lock(&priv->h2c_mutex);

	/* Clear the acknowledgment before publishing, so the poll below
	 * cannot be satisfied by an earlier command's indication.
	 */
	rtl8192f_write32(priv, REG_FWISR, FWISR_H2CCMD);

	box = priv->h2c_box;
	priv->h2c_box = (box + 1) & 3;

	/* Extension dword first, then the box dword, both written in full from
	 * zero-padded buffers - that is what mainline rtw88 does (fw.c
	 * 496-497) and what the vendor's endian-free path does byte by byte
	 * (Hal88XXFirmware.c 2484-2493, 2512-2521).  Skipping a byte is not
	 * harmless: the firmware sees a stale byte in the box and never
	 * consumes the command.
	 */
	rtl8192f_write32(priv, REG_HMEBOX_EXT(box), extv);
	rtl8192f_write32(priv, REG_HMEBOX(box), boxv);

	ret = h2c_cmd_taken(priv);
	if (ret)
		dev_dbg(&priv->pdev->dev,
			"h2c: id 0x%02x not acknowledged (box %u)\n", id, box);

	mutex_unlock(&priv->h2c_mutex);
	return ret;
}

/* Ask the in-core scheduler whether it is still reading its mailbox.
 *
 * REG_HMETFR cannot answer this on the 8197F (its box-full bits never drop,
 * above), so the only take-it signal this part produces is FS_H2CCMD, which
 * h2c_cmd_taken() polls.  The command chosen is the transmit-report request:
 * its answer never reaches the host on this chip (see the note above), so
 * asking cannot change what the caller is trying to observe.
 *
 * Returns 0 when the firmware took the command, -ETIMEDOUT when it did not,
 * -ENODEV when no firmware is running.  Sleeps up to the vendor's 12 ms poll
 * budget, so only call it from process context.
 */
int rtl8192f_fw_alive(struct rtl8192f_soc_priv *priv)
{
	u8 macid = 0;

	if (!priv->fw_ready)
		return -ENODEV;

	return rtl8192f_h2c_send(priv, H2C_88XX_AP_REQ_TXREP, &macid,
				 sizeof(macid));
}

void rtl8192f_send_ra_mask(struct rtl8192f_soc_priv *priv, u8 macid,
			   const struct ieee80211_sta *sta)
{
	bool wide = sta->deflink.bandwidth >= IEEE80211_STA_RX_BW_40;
	u16 sgi_cap = wide ? IEEE80211_HT_CAP_SGI_40 : IEEE80211_HT_CAP_SGI_20;
	u8 cmd[7];
	u32 ratr;
	int ret;

	/* The scheduler that reads this table is the firmware; with no
	 * image loaded there is nobody to take the command, and host-side
	 * rate control still programs the descriptor, so say nothing and
	 * carry on.
	 */
	if (!priv->fw_ready)
		return;

	/* Legacy part: our 2G bitrate table is ordered exactly like the
	 * rate bitmap (1,2,5.5,11,6,9,12,18,24,36,48,54 Mbit/s), so
	 * mac80211's supported-rate bitmap transfers without remapping.
	 * HT: MCS0..7 in bits 12..19.  Then the 1T1R NGB limit the vendor
	 * applies for this radio configuration (Hal88XXFirmware.c 1488).
	 */
	ratr = sta->deflink.supp_rates[NL80211_BAND_2GHZ] & GENMASK(11, 0);
	if (sta->deflink.ht_cap.ht_supported)
		ratr |= (u32)sta->deflink.ht_cap.mcs.rx_mask[0] << 12;
	ratr &= 0x000ff0ff;

	cmd[0] = macid;
	/* Rate-adaptation table id plus the short-GI flag, exactly as the
	 * vendor packs it (H2CCommand[1] = pstat->ratr_idx | (bShortGI ?
	 * 0x80 : 0), Hal88XXFirmware.c 1552).  ratr_idx is a table
	 * selector - phydm_rate_id_mapping() picks it from mode/streams/
	 * bandwidth - so it must be the same value the TX descriptor's
	 * RATE_ID field carries, or the scheduler looks up a ladder that
	 * does not describe this peer's band width.  The GI bit is the one
	 * belonging to that width, not always the 20 MHz one
	 * (Get_RA_ShortGI88XX, :1098-1111).
	 */
	cmd[1] = rtl8192f_ra_rate_id(sta) |
		 ((sta->deflink.ht_cap.cap & cpu_to_le16(sgi_cap)) ?
		  BIT(7) : 0);
	/* Band width in bits[1:0], with the vendor's own codes (0 = 20 MHz,
	 * 1 = 40 MHz) and DisableTXPowerTraining, as the vendor sets them
	 * (:1553-1555).  sta->deflink.bandwidth is the peer's width already
	 * capped by this BSS's chandef, which is the pair of tests the
	 * vendor applies (is_40m_bw && pstat->tx_bw, :1278-1289).
	 */
	cmd[2] = (wide ? 1 : 0) | BIT(6);
	put_unaligned_le32(ratr, &cmd[3]);

	ret = rtl8192f_h2c_send(priv, H2C_88XX_RA_MASK, cmd, sizeof(cmd));
	if (ret == -ETIMEDOUT)
		dev_warn(&priv->pdev->dev, "h2c: RA_MASK for macid %u timed out\n",
			 macid);
}

void rtl8192f_c2h_handle(struct rtl8192f_soc_priv *priv, const u8 *pkt,
			 u32 len)
{
	priv->c2h_cnt++;

	if (!priv->c2h_first_len) {
		priv->c2h_first_id = pkt[0];
		priv->c2h_first_len = len;
	}
}
