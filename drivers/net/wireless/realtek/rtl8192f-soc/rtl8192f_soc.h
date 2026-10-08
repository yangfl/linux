/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * RTL8192F-SoC (RTL8197F integrated 802.11bgn 1T1R) mac80211 driver

 * The WiFi core embedded in the RTL8197F router SoC is a member of the
 * RTL8192F (gen-2.5) family wired directly to the SoC bus.  Its DMA
 * engine uses the 88XX "new BD" dual-ring model (32-byte TXBD + 40-byte
 * TX info descriptor, 8-byte RXBD + 24-byte RX info descriptor in the
 * buffer), not the rtlwifi PCI in-descriptor OWN model.  Register maps
 * and init tables match the USB RTL8192F sibling supported by rtl8xxxu;
 * the bus and data path do not.
 */

#ifndef __RTL8192F_SOC_H
#define __RTL8192F_SOC_H

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/ieee80211.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <net/mac80211.h>

/* ------------------------------------------------------------------ */
/* MAC register offsets (RTL8192C lineage, extended for gen-2.5)       */
/* ------------------------------------------------------------------ */

/*
 * MAC register offsets.  Single source of truth for the whole driver
 * : hw.c and core.c must not carry
 * local copies.  Offsets/names follow vendor 8192c_reg.h unless
 * noted otherwise.
 */
#define REG_SYS_ISO_CTRL		0x0000
#define REG_AFE_MISC			0x0010
#define REG_SYS_FUNC_EN			0x0002
#define REG_APS_FSMCO			0x0004
#define REG_SYS_CLKR			0x0008
/* 88xx family naming (vendor HalComReg.h REG_SYS_CLKR / REG_SDIO_CTRL);
 * the 92c-style 0x0008 above is a different register on this core.
 */
#define REG_SYS_CLKR_88XX		0x0070
#define REG_RSV_CTRL			0x001C
#define REG_LDOA15_CTRL			0x0020
#define  LDOA15_ENABLE			BIT(0)
#define REG_RF_CTRL			0x001F
#define REG_LDOHCI12_CTRL		0x0022
#define REG_EFUSE_CTRL			0x0030	/* EFUSE_CTRL 0x030 */
#define REG_EFUSE_TEST			0x0034
#define REG_LEDCFG			0x004C
#define REG_LDO_SW_CTRL			0x007C	/* rtl8xxxu regs.h: LDO
						 * settings; 0x7c[31]=1 "LDO
						 * has max output capability"
						 * (rtl8xxxu 8192f.c:1655).
						 * Vendor 8192c_reg.h carries
						 * no name for 0x7C.
						 */
#define REG_SYS_CLK_CTRL		0x0008
/* 0x0008[14] (BIT_CPU_CLK_EN, halmac_bit2.h 826): clock enable for the
 * in-core MIPS CPU that runs the transmit scheduler on the 8197F.
 */
#define REG_CPU_CLK_EN			BIT(14)
#define REG_MCUFWDL			0x0080
#define REG_CR				0x0100
/* Network type sits in REG_CR bits[17:16] - i.e. byte 0x102, which the
 * 16-bit CR RMWs used elsewhere never cover (Hal88XXGen.c 5234,
 * BIT_SHIFT_NETYPE0 = 16, MASK = 0x3; MSR values HalComBit.h 48).
 */
#define REG_MSR				0x0102
#define MSR_NOLINK			0x00
#define MSR_INFRA			0x01
#define MSR_ADHOC			0x02
#define MSR_AP				0x03
/* REG_PBP (0x0104) deliberately absent: on the 8197F the TX page size
 * is fixed in hardware, vendor does not write it (Hal88XXGen.c puts
 * both PBP writes inside #if 0, "TX page file already fixed in HW.
 * No need to set register. Including old IC 8814A, 8197F").  The PSB
 * divisor for the TXBD page count is 256 (HalCfg.h PBP_PSTX_SIZE).
 */
/* REG_TRXDMA_CTRL (0x010C, 88XX TXDMA_PQ_MAP): vendor InitMAC88XX()
 * programs it on the 8197F with the queue map, RX shift enable and RX
 * DMA arbitration burst width (Hal88XXGen.c 4927-4933).
 */
#define REG_TRXDMA_CTRL		0x010C
/* TRX_DMA_QUEUE_MAP_PARA: VOQ/BEQ normal, VIQ/BKQ low, MGQ extra, HIQ
 * high (HalCfg.h 560-594, default txbuf_merge off).
 */
#define TRX_DMA_QUEUE_MAP_PARA	0xc660
#define BIT_RXDMA_ARBBW_EN	BIT(0)	/* HalComBit.h 8668-8669 */
#define BIT_RXSHFT_EN		BIT(1)

/* RX FIFO byte boundaries, vendor InitMAC88XX() (Hal88XXGen.c
 * 4907-4918): the RX FIFO area of the 16K TX/RX FIFO must be told
 * where it starts (TRXFF_BNDY high half) and ends (RXFF_BNDY low
 * half); both get MAC_RXFF_SIZE.  Without them the RX DMA engine has
 * no FIFO space and stays silent.
 */
#define REG_TRXFF_BNDY		0x0114
#define REG_RXFF_BNDY		0x011C
#define MAC_RXFF_SIZE		0x3e7f	/* HalCfg.h 552: 16K - 384 C2H - 1 */

/* Interrupt registers, 88XX/8197F layout
 * (vendor WlanHAL/HalMac88XX/halmac_reg2.h 422-425 and halmac_bit2.h
 * 10855-10890/11147-11160).  Group 0 (0xB0/0xB4) carries RXOK, RDU and
 * the per-queue TX done bits; RX/TX FIFO overflow (FOVW/TXFOVW) live in
 * group 1 (0xB8/0xBC).  Both status registers are write-1-to-clear.
 */
#define REG_HIMR0		0x00B0	/* interrupt mask group 0 */
#define REG_HISR0		0x00B4	/* interrupt status 0 (W1C) */
#define REG_HIMR1		0x00B8	/* interrupt mask group 1 */
#define REG_HISR1		0x00BC	/* interrupt status 1 (W1C) */
/* Vendor 8197F is MAC_V2 and its ISR also services groups 2/3
 * (Hal88XXIsr.c:101-107); this driver enables neither group.
 */
#define REG_TXDMA_STATUS	0x0210	/* TXDMA error latches (W1C):
					 * BIT(12) DSC_CHKSUM_FAIL,
					 * BIT(13) PAYLOAD_OVF,
					 * BIT(14) PAYLOAD_UDN ("payload
					 * is under the packet length of
					 * Tx descriptor", 8192c_reg.h),
					 * BIT(15) PAYLOAD_CHKSUM_ERR
					 * (halmac_bit2.h 8197F section).
					 * The vendor ISR read+write-
					 * clears this in the TXFOVW and
					 * TXERR branches; the read must
					 * be preceded by a clear or it
					 * reports stale bits.
					 */
#define REG_RXDMA_STATUS	0x0288	/* RXDMA error latches (W1C), the
					 * counterpart of REG_TXDMA_STATUS:
					 * RXFOVW/RXERR set these and the
					 * vendor ISR read+write-clears the
					 * register (8192cd_osdep.c 1997-
					 * 2002, REG_RXDMA_STATUS offset from
					 * halmac_bit2.h 12191).  Leaving it
					 * latched keeps the RX error source
					 * asserted, the same failure mode the
					 * RDU masking above works around.
					 */
/* HSISR mask (GPIO9/AXI-exception group, halmac_reg2.h 1789); the
 * matching status register is REG_HSISR below.
 */
#define REG_HSIMR		0x0058
/* HSISR status (W1C).  The vendor 8197F ISR always read+write-clears it
 * (Hal88XXIsr.c 115-124); a latched, never-cleared HSISR bit keeps the
 * aggregated line asserted with no driver-visible source.
 */
#define REG_HSISR		0x005C
/* Firmware interrupt status (W1C).  FS_H2CCMD_INT (bit4) is asserted each
 * time the in-core scheduler takes an H2C command off a mailbox;
 * FS_TRXRPT_INT (bit7) is its TX-report indication.  Bit names from
 * halmac_bit2.h's REG_FWISR block for HALMAC_8197F_SUPPORT.
 */
#define REG_FWISR		0x0134
#define FWISR_H2CCMD		BIT(4)
#define FWISR_TRXRPT		BIT(7)
/* Host-to-firmware mailbox: four boxes at 0x1D0 (each with a 4-byte
 * extension at 0x1F0) and one status byte at 0x1CC.  The vendor reads bit n
 * of the status byte as "the firmware still holds box n" and waits for it to
 * drop before reusing the box (CheckFwReadLastH2C88XX, Hal88XXFirmware.c
 * 2356-2370); when it never drops it clears the boxes with 0x00020101 -
 * first the last one, then all four (FillH2CCmd88XX, Hal88XXFirmware.c
 * 2446-2468).
 */
#define REG_HMETFR			0x01CC
#define REG_HMEBOX(n)			(0x01D0 + (n) * 4)
#define REG_HMEBOX_EXT(n)		(0x01F0 + (n) * 4)
/* AXI bus exception cause / timestamp (8197F only, halmac_reg2.h
 * 2683-2684, bits halmac_bit2.h 32849-32885: DECERR/SLVERR R/W and
 * AXI_RXDMA_TIMEOUT).  Vendor never reads them; a non-zero value here
 * names the bus master and address that wedged the SoC.
 */
#define REG_TXDMA_OFFSET_CHK		0x020C
#define REG_RXDMA_STATUS		0x0288
#define REG_RXDMA_MODE		0x0290
#define REG_SYS_CFG			0x00F0

/* ------------------------------------------------------------------ */
/* 88XX "new BD" ring registers (0x300 block).  All offsets per vendor */
/* WlanHAL/HalMac88XX/halmac_reg2.h 897-936 (8197F included); the  */
/* kick/reap protocol matches mainline rtw88 pci.h/pci.c.              */
/* ------------------------------------------------------------------ */

/* TXBD ring base addresses (DESA), one per hardware queue */
#define REG_BCNQ_TXBD_DESA		0x0308
#define REG_MGQ_TXBD_DESA		0x0310
#define REG_VOQ_TXBD_DESA		0x0318
#define REG_VIQ_TXBD_DESA		0x0320
#define REG_BEQ_TXBD_DESA		0x0328
#define REG_BKQ_TXBD_DESA		0x0330
#define REG_RXQ_RXBD_DESA		0x0338
#define REG_HI0Q_TXBD_DESA		0x0340

/* Ring length registers: bits[11:0] = number of entries,
 * bits[13:12] = segment mode (0=2, 1=4, 2=8 elements per BD; vendor
 * uses 1 everywhere, HalCfg.h TX_*_TXBD_MODE_SEL).  REG_RX_RXBD_NUM
 * differs: bits[14:13] = BCNQ segment mode, bit[12] = HCI_BCNQ_FLAG,
 * bit[15] = SYS_32_64 (halmac_bit2.h 18704-18769).
 */
#define REG_MGQ_TXBD_NUM		0x0380
#define REG_RX_RXBD_NUM			0x0382
#define REG_VOQ_TXBD_NUM		0x0384
#define REG_VIQ_TXBD_NUM		0x0386
#define REG_BEQ_TXBD_NUM		0x0388
#define REG_BKQ_TXBD_NUM		0x038A
#define REG_HI0Q_TXBD_NUM		0x038C

/* TXBD_NUM / RX_RXBD_NUM bit fields (halmac_bit2.h 18710-18769) */
#define TXBD_NUM_MASK			GENMASK(11, 0)
#define TXBD_DESC_MODE_4SEG		BIT(12)		/* TX queues */
#define RXBD_NUM_MASK			GENMASK(11, 0)
#define BCNQ_DESC_MODE_4SEG		BIT(13)		/* bits[14:13] */
#define RXBD_SYS_32_64			BIT(15)		/* 0 = 32-bit */

/* Write 0xffffffff to reset all host + HW ring indices (same as
 * mainline rtw88 pci.c RTK_PCI_TXBD_RWPTR_CLR; vendor 8192cd_hw.c
 * also writes 0xffffffff at 0x39C).
 */
#define REG_BD_RWPTR_CLR		0x039C

/* HCI DMA control (0x300) and pre-fetch control (0x3DB),
 * vendor halmac_reg2.h 883/991.  Programming per Hal88XXGen.c
 * 5404-5425 (IS_EXIST_RTL8197FEM): burst sizes, keep BIT15 set (the
 * 8197F hardware default "RXTAG" RXBD ready-check mode) and BIT14
 * (stop BCNQ DMA); 0x3DB bits 6/7 disable RX/TX pre-fetch.
 */
#define REG_HCI_CTRL			0x0300
#define REG_HCI_CTRL2			0x03DB
#define HCI_MAX_RXDMA_SHIFT		28	/* bits[30:28], 0=16B..7=2048B */
#define HCI_MAX_RXDMA_MASK		GENMASK(30, 28)
#define HCI_MAX_TXDMA_SHIFT		24	/* bits[26:24] */
#define HCI_MAX_TXDMA_MASK		GENMASK(26, 24)
#define HCI_BURST_SIZE_32B		1	/* LBUS burst: 32 bytes */
#define HCI_STOP_BCNQ			BIT(14)
#define HCI_RXTAG_MODE			BIT(15)	/* 1 = RXBD ready check by
						 * HW RXTAG counter (8197F
						 * power-on default)
						 */
#define HCI2_DIS_RXDMA_PRE		BIT(6)
#define HCI2_DIS_TXDMA_PRE		BIT(7)

/* Producer/consumer pointer registers: low 16 bits = host write
 * (producer, we write with write16), high 16 bits = HW consumer
 * (TX done / RX fill).  Vendor Hal88XXTxDesc.c 2476-2489
 * GetTxQueueHWIdx88XX() reads bits[27:16] (HalComBit.h
 * BIT_SHIFT_QUEUE_HW_IDX 16); mainline rtw88 pci.h TRX_BD_IDX_MASK /
 * TRX_BD_HW_IDX_MASK.
 */
#define REG_VOQ_TXBD_IDX		0x03A0
#define REG_VIQ_TXBD_IDX		0x03A4
#define REG_BEQ_TXBD_IDX		0x03A8
#define REG_BKQ_TXBD_IDX		0x03AC
#define REG_MGQ_TXBD_IDX		0x03B0
#define REG_RXQ_RXBD_IDX		0x03B4
#define REG_HI0Q_TXBD_IDX		0x03B8
#define REG_RRSR			0x0440	/* Response Rate Set */
#define REG_ARFR1			0x044C	/* rate fall-back set 1, 64 bit */
/* Channel width: the secondary-channel index, and the BB/RF bits that
 * select 20 vs 40 MHz.  Names follow the vendor's phydm register table
 * (phydm_regdefine11n.h) so a reader can match the write sequences.
 */
#define REG_DATA_SC			0x0483	/* [3:0] secondary channel */
#define REG_HWSEQ_CTRL			0x0423
#define REG_EDCA_VO_PARAM		0x0500
#define REG_EDCA_VI_PARAM		0x0504
#define REG_EDCA_BE_PARAM		0x0508
#define REG_EDCA_BK_PARAM		0x050C
#define REG_SLOT_TIME			0x051B	/* slot time in us (byte) */
#define REG_TXPAUSE			0x0522
/* TXREQ clear-disable gate (HalComReg.h 940): 0x80 = "Disable BT
 * coextense" - without it the MAC believes the air interface is
 * arbitrated away and never grants TXREQ (pages never drain).
 */
#define REG_DIS_TXREQ_CLR		0x0523
/* Channel loading measurement: the BBP times CCA-busy itself over a window
 * the host programs in 4 us units, arms it through 0x890 (reset, then arm,
 * then start) and leaves the count in 0x8d0 with 0x8b4 bit16 as the valid
 * flag.  The vendor's N-series branch of start_bbp_ch_load()/read_bbp_ch_load()
 * covers this chip (8192cd_hw.c 29699-29738) and its channel-utilisation timer
 * runs it continuously, so the counter needs no transmit permission to work -
 * which is what makes it usable next to a halted transmitter.
 */
#define REG_CH_LOAD_CTRL		0x0890	/* bit8 arms, bit0 starts */
#define REG_CH_LOAD_PERIOD		0x0894	/* u16, window in 4 us units */
#define REG_CH_LOAD_STATUS		0x08b4	/* bit16: count is valid */
#define REG_CH_LOAD_COUNT		0x08d0	/* bits 15:0, 4 us units */

/* Short[15:8] / long[7:0] retry limits (HalComReg.h 697); zero limits
 * can keep the MAC from transmitting at all.  Vendor AP defaults
 * (HalCfg.h 626-630, non-CONFIG_RTL8672): 0x10/0x10.
 */
#define REG_RETRY_LIMIT			0x042A
#define RTL8192F_RETRY_LIMIT_DEFAULT	0x1010
#define REG_BCN_CTRL			0x0550
#define REG_MBID_NUM			0x0552
/* Beacon registers on the 88XX map (halmac_reg2.h 1182/1571-1573):
 * BCNQ_BDNY is 0x424, NOT the rtlwifi 8192ee 0x558.  0x554 is
 * MBSSID_BCN_SPACE (TBTT period) and 0x558/0x559 are DRVERLYINT /
 * BCNDMATIM (the vendor MAC_REG init table writes 0x559 = 2).
 */
#define REG_BCNQ_BDNY			0x0424
#define REG_MBSSID_BCN_SPACE		0x0554
#define REG_DRVERLYINT			0x0558
#define REG_BCNDMATIM			0x0559
#define REG_ATIMWND			0x055A	/* 16 bit (HalComReg.h 976) */
#define REG_TBTT_PROHIBIT		0x0540	/* HalComReg.h 946 */
#define REG_BCNTCFG			0x0510	/* HalComReg.h 931 */
#define REG_PKT_LIFETIME_CTRL		0x0528	/* HalComReg.h 944 */
/* Vendor HW_VAR_MAC_CONFIG, AP branch (Hal88XXGen.c 432/457-462/484-490):
 * TBTT_PROHIBIT = 0x40004, DRVERLYINT = 10 (BEACON_ERALY_INIT_TIME),
 * BCNDMATIM = 1, ATIMWND = 0x3C, BCNTCFG = 0x3213, and PKT_LIFETIME_CTRL
 * bit19 cleared (`HAL_RTL_W32(REG_PKT_LIFETIME_CTRL, .. & ~BIT(19))`,
 * :464; the bit is BIT_DIS_PKT_TX_ATIM, halmac_bit2.h REG_PKT_LIFETIME_CTRL).
 */
#define RTL8192F_TBTT_PROHIBIT_AP	0x00040004
#define RTL8192F_BCNTCFG_AP		0x3213
#define BIT_DIS_PKT_TX_ATIM		BIT(19)
/* REG_BCN_CTRL bits (HalComBit.h 16406-16409).  The vendor AP value is
 * DIS_TSF_UDT|EN_BCN_FUNCTION|EN_TXBCN_RPT|DIS_RX_BSSID_FIT = 0x5c, which
 * is what the working vendor AP reads back at 0x550.
 */
#define BIT_BCN_DIS_TXBCN_RPT		BIT(2)
#define BIT_BCN_EN_FUNCTION		BIT(3)
#define BIT_BCN_DIS_TSF_UDT		BIT(4)
#define BIT_BCN_DIS_RX_BSSID_FIT	BIT(6)
#define REG_TCR				0x0604
/* mactx updates DTIM count / group bit / moreData in the resident beacon
 * (HalComBit.h 17559-17560, written by Hal88XXGen.c 5281).
 */
#define BIT_TCR_UPD_HGQMD		BIT(4)
#define BIT_TCR_UPD_TIMIE		BIT(5)
#define REG_RCR				0x0608
/* Frame-type accept maps (HalComReg.h 1237-1239 + the MAC_V2 branch of
 * Hal88XXGen.c 519-526): management 0x6A0, control 0x6A2, data 0x6A4.
 * The vendor enables exactly PS-Poll (BIT_CTRLFLT10EN) in the control
 * map; leaving it wide open DMAs ACK/CTS/Block-Ack frames into the RX
 * ring as 14-32 byte junk descriptors.
 */
#define REG_RXFLTMAP0			0x06A0	/* management */
#define REG_RXFLTMAP1			0x06A2	/* control */
#define REG_RXFLTMAP2			0x06A4	/* data */
#define BIT_RXFLT_PSPOLL		BIT(10)
/* BB side of the 20/40 MHz choice (phydm_regdefine11n.h 43/78/87/112/136):
 * 0x800 bit0 and 0x900 bit0 are the "RF mode" pair, 0x800[14:12] the DAC
 * clock, 0xca4[27:26] the ADC buffer clock, 0xa00 bit4 the CCK primary
 * channel, and 0xce4[31:30] the small-bandwidth selector (0 = 20/40 MHz).
 */
#define REG_RF_MODE			0x0800
#define REG_RF_MODE_PAGE9		0x0900
#define REG_RX_DFIR_MOD			0x0948	/* ODM_REG_RX_DFIR_MOD_97F;
						 * receive DFIR tap bank
						 */
#define REG_CCK_ANTDIV_PARA1		0x0A00
#define REG_EDCCA_LEVEL			0x0AA8	/* [20:16] energy-detect CCA
						 * threshold, literal in
						 * phydm_CcaParByBw_8197f()
						 */
#define REG_CCA_CTRL			0x0C3C	/* bits[8:3] CCA setup, same
						 * helper
						 */
/* Which half of a 40 MHz channel the receiver puts carrier detect on, and
 * the PD (preamble detect) threshold that goes with it.  Neither register is
 * named in the vendor's phydm code - phydm_api_primary_cca()
 * (phydm_rtl8197f.c 185-215) writes both by number.
 */
#define REG_PRIMARY_CCA			0x0C6C	/* [8:7] side-band select */
#define  PRIMARY_CCA_SEL		GENMASK(8, 7)
#define  PRIMARY_CCA_DSC		0	/* both halves (default) */
#define  PRIMARY_CCA_LSC		1	/* primary on the lower half */
#define  PRIMARY_CCA_USC		2	/* primary on the upper half */
#define REG_PD_THRESHOLD		0x0C84	/* [31:28] preamble detect level */
#define  PD_THRESHOLD			GENMASK(31, 28)
#define REG_RXCK_RFMOD			0x0BB0
#define REG_ANTDIV_PARA1			0x0CA4
#define REG_SMALL_BANDWIDTH		0x0CE4
/* Byte registers at 0x60C-0x60F (HalComReg.h 1135-1137, halmac_reg2.h
 * 1809-1811): RX_PKT_LIMIT 0x60C, RX_DLK_TIME 0x60D, and RX_DRVINFO_SZ
 * 0x60F.  A wider write here spills into the neighbours - the previous
 * 16-bit write to "0x60E" zeroed 0x60F and corrupted 0x60D.
 */
#define REG_RX_PKT_LIMIT		0x060C
#define REG_RX_DLK_TIME			0x060D
/* REG_RX_DRVINFO_SZ (0x60F): bits[3:0] = driver info size in 8-byte
 * units appended to each RX packet, bit6 = APP_MH_SHIFT (payload
 * 4-byte alignment), bit5 = WMAC_ENSHIFT (must stay clear, vendor
 * InitMAC88XX() Hal88XXGen.c 4966-4973).  halmac_bit2.h
 * 25923/25943/25986-25988, halmac_reg2.h 1811.
 */
#define REG_RX_DRVINFO_SZ		0x060F
#define RX_DRVINFO_SZ_UNIT		4	/* 4 x 8B per RX packet */
#define RX_DRVINFO_APP_MH_SHIFT	BIT(6)
#define RX_DRVINFO_ENSHIFT		BIT(5)
#define REG_MACID			0x0610	/* MACID[0] addr, 8B each */
#define REG_BSSIDR			0x0618	/* BSSID, 6 bytes */

/* Multi-BSSID address CAM (HalComReg.h 1141-1142, bits HalComBit.h
 * 17719-17726).  One entry per extra BSSID the MAC has to recognise as its
 * own: dword 1 carries address bytes 0..3, dword 2 bytes 4..5 with the entry
 * index, the valid bit and the write/poll flags.  InitMBIDCAM88XX() writes
 * dword 1 first, then dword 2 with both flags set (Hal88XXGen.c 6185-6204),
 * and StopMBIDCAM88XX() clears an entry by writing the same command with the
 * valid bit low and a zero address (Hal88XXGen.c 6876-6890).
 */
#define REG_MBIDCAM_1			0x0628
#define REG_MBIDCAM_2			0x062C
#define  MBIDCAM_POLL			BIT(31)
#define  MBIDCAM_WRITE			BIT(30)
#define  MBIDCAM_INDEX			GENMASK(28, 24)
#define  MBIDCAM_VALID			BIT(23)

/* Security CAM (HalComReg.h 1207-1211).  The entry RAM is not memory
 * mapped: each of the 64 entries is 8 dwords and is reached one dword at
 * a time through the write/data and read/data pair.  The command word is
 * dword address (entry * 8 + dword) plus the polling request bit and, for
 * a write, the write-enable bit; the polling bit stays set while the
 * engine is busy, so it has to be observed clear before the next dword
 * (CAMProgramEntry88XX / CAMFindUsable88XX, Hal88XXGen.c 46-98).
 */
#define REG_CAM_CMD			0x0670
#define REG_CAM_WRITE			0x0674
#define  CAM_CMD_POLLING		BIT(31)	/* BIT_SECCAM_POLLING */
#define  CAM_CMD_WRITE_ENABLE		BIT(16)	/* BIT_SECCAM_WE */
#define  CAM_DWORDS_PER_ENTRY		8	/* HAL_CAM_CONTENT_COUNT */
/* HAL_CAM_CONTENT_USABLE_COUNT: two address dwords plus four key dwords. */
#define  CAM_DWORDS_IN_USE		6
/* total_cam_entry from 8192cd_osdep.c 3746-3749, the else branch that the
 * 8197F takes.
 */
#define  CAM_ENTRIES			64
/* Entries 0..3 stay reserved for group keys: with the engine's default-key
 * search a multicast frame's key id indexes the entry directly (rtw88
 * sec.c rtw_sec_get_free_cam() makes the same choice for the same reason),
 * so pairwise keys are allocated from CAM_GROUP_ENTRIES upwards.
 */
#define  CAM_GROUP_ENTRIES		4
/* DW0 low half: valid bit plus the algorithm and key id, exactly the
 * combination CamAddOneEntry() builds (8192cd_hw.c 19245, with the
 * algorithm passed already shifted by 2 - 8192cd_net80211.c 1064) and the
 * same bit the vendor tests to decide whether an entry is free
 * (CAMFindUsable88XX, Hal88XXGen.c 92-95).
 */
#define  CAM_CFG_VALID			BIT(15)
#define  CAM_CFG_ALG_SHIFT		2
#define  CAM_CFG_KEYID_MASK		0x3
/* DOT11_ENC_ALGO (8192cd_security.h 116-124) */
#define  CAM_ALG_NONE			0
#define  CAM_ALG_CCMP			4

/* Security engine configuration (REG_SECCFG, HalComReg.h 1211).  The
 * vendor programs, for the hardware-crypto path, TXENC | RXDEC | NOSKMC |
 * CHK_KEYID (CamAddOneEntry() -> HW_VAR_SECURITY_CONFIG, 8192cd_hw.c
 * 19271-19283 with SCO_* from Wlan_TypeDef.h 40-50 mapped through
 * HalComBit.h 18224-18230).  NOSKMC is what keeps a multicast frame from
 * being decrypted by a CAM entry: this driver leaves group keys in
 * software, like the vendor build does (set_gkey_to_cam = 0,
 * 8192cd_security.c 621).
 */
#define REG_SECCFG			0x0680
#define  SEC_CFG_TXENC			BIT(2)
#define  SEC_CFG_RXDEC			BIT(3)
#define  SEC_CFG_NOSKMC			BIT(5)
#define  SEC_CFG_CHK_KEYID		BIT(8)
/* REG_CR bit9, BIT_MAC_SEC_EN (HalComBit.h 8563): the MAC security engine
 * has to be enabled for the CAM to be consulted at all;
 * CAMProgramEntry88XX sets it as part of installing an entry
 * (Hal88XXGen.c 229-232).
 */
#define  CR_MAC_SEC_EN			BIT(9)

/* MACID Setting area / BSSIDR / RRSR / SLOT_TIME provenance:
 * vendor 8192c_reg.h (RTL8197_3411D_2 rtl8192cd).
 */

/* EDCA register value layout (Realtek gen-2): bits 31:16 TXOP limit in us,
 * bits 15:8 ECWmax, bits 7:4 ECWmin, bits 3:0 AIFSN
 */
#define EDCA_VAL(aifs, ecwmin, ecwmax, txop) \
	(((txop) << 16) | ((ecwmax) << 8) | ((ecwmin) << 4) | (aifs))

/* SLOT_TIME (MAC 0x51B) values in us */
#define RTL8192F_SLOT_SHORT	0x09
#define RTL8192F_SLOT_LONG	0x14

/* RCR bits (vendor WlanHAL/HalHeader/HalComBit.h 17611-17659, 88XX
 * layout - NOT the gen-1 8192c_reg.h positions).
 */
#define RCR_AAP				BIT(0)
#define RCR_APM				BIT(1)
#define RCR_AM				BIT(2)
#define RCR_AB				BIT(3)
#define RCR_ACRC32			BIT(8)	/* pass FCS-error frames
						 * (rtw88 FIF_FCSFAIL)
						 */
#define RCR_AICV			BIT(9)
/* Accept management frames: gen-2 gates beacons/probe responses on this
 * bit (the vendor sets 0x2000 in RCR).
 */
#define RCR_AMF				BIT(13)
#define RCR_ADF				BIT(11)
#define RCR_APP_PHYST_RXFF		BIT(28)
#define RCR_APP_ICV			BIT(29)
#define RCR_APP_MIC			BIT(30)
#define RCR_APP_FCS			BIT(31)	/* keep the FCS in the
						 * received frame
						 */
/* Recognise the MBIDCAM addresses as own-BSS traffic (BIT_ENMBID,
 * HalComBit.h 17636).  The vendor always rewrites RCR as clear-then-set so
 * the MAC samples the CAM again (InitMBIDCAM88XX, Hal88XXGen.c 6201-6203).
 */
#define RCR_ENMBID			BIT(24)

/* Default RX filter for all modes (hw_init, start, scan, assoc):
 * accept unicast/multicast/broadcast, keep the FCS in the frame
 * (the rx path trims it), append PHY status and crypto fields.
 * ACRC32 matches the vendor 88XX baseline (Hal88XXGen.c 5248-5250,
 * MAC-V2 which is what 8197F is): it lets FCS-bad frames reach the
 * driver so the drop is visible and countable.  ADF must NOT be
 * set: the vendor removes it deliberately ("setting BIT_ADF will
 * also accept BA and BAR", Hal88XXGen.c 505-507) and on this core
 * ADF stops beacon reception entirely.
 */
#define RTL8192F_RCR_DEFAULT		(RCR_AAP | RCR_APM | RCR_AM | \
					 RCR_AB | RCR_AMF | RCR_ACRC32 | \
					 RCR_APP_FCS | RCR_APP_MIC | \
					 RCR_APP_ICV | RCR_APP_PHYST_RXFF)

/* CR (0x0100) bits - vendor 8192c_reg.h 1087-1094 names, one set for
 * the whole driver.
 */
#define CR_HCI_TXDMA_EN			BIT(0)	/* HCI to TXDMA interface */
#define CR_HCI_RXDMA_EN			BIT(1)	/* HCI to RXDMA interface */
#define CR_TXDMA_EN			BIT(2)	/* TXDMA function */
#define CR_RXDMA_EN			BIT(3)	/* RXDMA function */
#define CR_PROTOCOL_EN			BIT(4)	/* protocol block */
#define CR_SCHEDULE_EN			BIT(5)	/* scheduler */
#define CR_MACTXEN			BIT(6)	/* MAC transmitter */
#define CR_MACRXEN			BIT(7)	/* MAC receiver */

/*
 * Interrupt mask bits, group 0 (REG_HIMR0 0xB0 / REG_HISR0 0xB4).
 * Bit layout per vendor halmac_bit2.h 10864-10881 (8197F section):
 * RXOK=0, RDU=1, VODOK=2, VIDOK=3, BEDOK=4, BKDOK=5, MGTDOK=6,
 * HIGHDOK=7.  Shared by core.c (irq handler) and txrx.c (start/stop +
 * per-ring DOK map).
 */
#define RTL8192F_HIMR_RXOK		BIT(0)	/* RX DMA OK */
#define RTL8192F_HIMR_RDU		BIT(1)	/* RX descriptor unavailable */
#define RTL8192F_HIMR_VODOK		BIT(2)
#define RTL8192F_HIMR_VIDOK		BIT(3)
#define RTL8192F_HIMR_BEDOK		BIT(4)
#define RTL8192F_HIMR_BKDOK		BIT(5)
#define RTL8192F_HIMR_MGTDOK		BIT(6)
#define RTL8192F_HIMR_HIGHDOK		BIT(7)
#define RTL8192F_HIMR_HISR1_IND		BIT(11)	/* HISR1 has latched bits */
#define RTL8192F_HIMR_BCNDMAINT0	BIT(20)	/* beacon TXBD request */
#define RTL8192F_HIMR_TXBCN0ERR		BIT(26)	/* beacon TX failed */
#define RTL8192F_HIMR_TXBCN0OK		BIT(25)	/* beacon TX done */

/* Group 1 (REG_HIMR1 0xB8 / REG_HISR1 0xBC), halmac_bit2.h 11147-11160:
 * FIFO overflow indicators live here, not in group 0.
 */
#define RTL8192F_HIMR1_FOVW		BIT(8)	/* RX fifo overflow */
#define RTL8192F_HIMR1_TXFOVW		BIT(9)	/* TX fifo overflow */
#define RTL8192F_HIMR1_RXERR		BIT(10)	/* RX error (0x210 latch) */
#define RTL8192F_HIMR1_TXERR		BIT(11)	/* TX error (0x210 latch) */

/* TX queue done (DOK) bits the irq handler serves.  HIGHDOK is bit 7:
 * group addressed frames go out on Q_HIGH and its ring is reaped by
 * the same tx done path.
 */
#define RTL8192F_TX_DONE_MASK		(RTL8192F_HIMR_VODOK |	\
					 RTL8192F_HIMR_VIDOK |	\
					 RTL8192F_HIMR_BEDOK |	\
					 RTL8192F_HIMR_BKDOK |	\
					 RTL8192F_HIMR_MGTDOK |	\
					 RTL8192F_HIMR_HIGHDOK)

/* Host-owned HIMR0 set, vendor EnableIMR88XX/IntMask[0] (Hal88XXGen.c
 * 5479: BIT_RXOK | BIT_HISR1_IND_INT | BIT_RDU, plus the AP-mode beacon
 * bits at 5489-5491).  The per-queue DOK bits are NOT here on purpose:
 * on this SoC the in-core MIPS scheduler consumes them, and the host
 * enabling them - which write-clears them - silences the transmitter.
 */
#define RTL8192F_HIMR0_INT_MASK		(RTL8192F_HIMR_RXOK |	\
					 RTL8192F_HIMR_RDU |	\
					 RTL8192F_HIMR_HISR1_IND | \
					 RTL8192F_HIMR_BCNDMAINT0 | \
					 RTL8192F_HIMR_TXBCN0OK | \
					 RTL8192F_HIMR_TXBCN0ERR)
#define RTL8192F_HIMR1_INT_MASK		(RTL8192F_HIMR1_FOVW |	\
					 RTL8192F_HIMR1_TXFOVW |	\
					 RTL8192F_HIMR1_RXERR |	\
					 RTL8192F_HIMR1_TXERR)

/* TXDMA_STATUS error bits that carry the reason a frame was refused
 * (see REG_TXDMA_STATUS above); masked against the raw read when the
 * TX path takes its per-frame forensic snapshot.
 */
#define RTL8192F_TXDMA_ERR_MASK		0x0000f000

/* SYS_ISO_CTRL bits (vendor 8192c_reg.h) */
#define SYS_ISO_ANALOG_IPS		BIT(5)	/* PW_ISO_ANO_IPS: isolate
						 * analog IPs from digital
						 */
#define SYS_ISO_PWC_EV12V		BIT(15)	/* 1.2V power from VDDON */

/* SYS_FUNC_EN bits (vendor 8192c_reg.h; BIT13 DIO_RF per
 * 8192cd_hw.c 8197F init block, "val16 | BIT(13) | BIT(0) | BIT(1)")
 */
#define SYS_FUNC_BBRSTB			BIT(0)
#define SYS_FUNC_BB_GLB_RSTN		BIT(1)
#define SYS_FUNC_CPU_EN			BIT(10)	/* MCU (+1 byte bit2) */
#define SYS_FUNC_ELDR			BIT(12)	/* EFuse loader clock */
#define SYS_FUNC_DIO_RF			BIT(13)	/* RF over DIO (8197F) */
#define SYS_FUNC_MREGEN			BIT(15)

/* APS_FSMCO bits (vendor 8192c_reg.h) */
#define APS_FSMCO_MAC_ENABLE		BIT(8)	/* MAC_ON, self-clearing */
#define APS_FSMCO_MAC_OFF		BIT(9)
#define APS_FSMCO_SW_LPS		BIT(10)
#define APS_FSMCO_HW_SUSPEND		BIT(11)
#define APS_FSMCO_PCIE			BIT(12)
#define APS_FSMCO_HW_POWERDOWN		BIT(15)
#define APS_FSMCO_WLON_RESET		BIT(16)

/* SYS_CLKR bits (vendor 8192c_reg.h) */
#define SYS_CLK_ANA8M			BIT(1)
#define SYS_CLK_LOADER_CLK_EN		BIT(5)	/* EFuse loader clock gate */
#define SYS_CLK_MAC_CLK_ENABLE		BIT(11)

/* REG_SYS_CLKR_88XX (0x70) bits, vendor HalComBit.h */
#define SYS_CLKR88_AFE_PORT2_ISO		BIT(26)	/* isolates the TX path */

/* AFE_MISC bits */
#define AFE_MISC_WL_XTAL_CTRL		BIT(6)	/* WL XTAL control */
#define AFE_MISC_SWB_TELDO		BIT(18)	/* SWR OCP enable (rtl8xxxu
						 * 8192f.c)
						 */

/* RF_CTRL (0x1f) bits */
#define RF_ENABLE			BIT(0)
#define RF_RSTB				BIT(1)
#define RF_SDMRSTB			BIT(2)

/* ------------------------------------------------------------------ */
/* Queue / ring layout - 88XX "new BD" dual-ring model                 */
/*                                                                     */
/* Every TX queue is a pair of rings: a TXBD ring of 32-byte buffer    */
/* descriptors (4 x 8-byte elements, addresses live here, no OWN       */
/* kick) plus a 40-byte TX info descriptor embedded in the payload     */
/* buffer (mainline style, rtw88 pci.c rtw_pci_tx_write_data()).  Kick */
/* and reclaim are done through the *_TXBD_IDX register producer /     */
/* consumer indices.  The beacon queue is a resident single-slot BD    */
/* gated by the BCN_OWN bit, not a host-advanced ring.                 */
/* ------------------------------------------------------------------ */

enum rtl8192f_queue {
	Q_MGNT = 0,
	Q_BK,
	Q_BE,
	Q_VI,
	Q_VO,
	Q_HIGH,		/* multicast / group addressed */
	Q_BEACON,	/* single-slot BCNQ, no host idx ring */
	Q_NUM
};

/*
 * Hardware queue select values for the TX info descriptor W1 QSEL
 * field (88XX encoding, vendor Hal88XXTxDesc.c queue mapping; mainline
 * rtw88 tx.h 56-77 enum rtw_tx_desc_queue_select).  Data AC frames use
 * the TID from the skb (0..15) directly.
 */
#define RTL8192F_QSEL_BEACON		16
#define RTL8192F_QSEL_HIGH		17
#define RTL8192F_QSEL_MGMT		18

/* Geometry :
 * TXBD element 8B x 4 = 32B (HalCfg.h TXBD_SEG_32_64_SEL=0 +
 * TXBD_ELE_NUM=4, Hal88XXTxDesc.h 88-101), TX info desc 40B
 * (SIZE_TXDESC_88XX, Hal88XXTxDesc.h 20-23; the 8197F uses 40, not
 * the V1 48), RXBD 8B (halmac_rx_bd_nic.h), RX info desc 24B at the
 * start of the DMA buffer (SIZE_RXDESC_88XX, Hal88XXRxDesc.h 20).
 */
#define RTL8192F_TXBD_ELE_NUM		4
#define RTL8192F_TXBD_ELE_SIZE		8
#define RTL8192F_TXBD_SIZE		(RTL8192F_TXBD_ELE_NUM * \
					 RTL8192F_TXBD_ELE_SIZE)
#define RTL8192F_TX_INFO_DESC_SIZE	40
#define RTL8192F_RXBD_SIZE		8
#define RTL8192F_RX_INFO_DESC_SIZE	24

#define RTL8192F_NUM_TX_DESC		128
#define RTL8192F_NUM_RX_DESC		128
#define RTL8192F_RX_BUF_SIZE		8192

/* PSB page size: pages of 256B on the 8197F (HalCfg.h PBP_PSTX_SIZE
 * 256 for the 92E/8881A/8197F series; NOT the 128 of PCIe parts that
 * rtw88 uses).  PSB = ceil((40 + frame len) / 256),
 * Hal88XXTxDesc.c 1653-1669.
 */
#define RTL8192F_TX_PAGE_SIZE		256

/* Beacon: resident single slot per VIF at a fixed 128-byte stride in
 * the BCNQ TXBD area ("HW Fixed value", HalCfg.h TXBD_BEACON_OFFSET_V1
 * = 128 for the 8197F).  One slot for the single supported VIF; MBSSID
 * slots would follow at +128B each.
 */
#define RTL8192F_BCN_SLOT_STRIDE	128
#define RTL8192F_NUM_BCN_SLOT		1
/* Beacon payload buffer: 2.4 GHz beacon (24B header + fixed IEs + vendor/RSN
 * IEs + TIM) stays well under this; an oversized template is dropped with a
 * warning rather than corrupting the resident slot.
 */
#define RTL8192F_BEACON_MAX_LEN		1024

/* TXBD element: DW0 = TX_BUFF_SIZE0[15:0] + PSB[23:16] (beacon variant
 * bits[30:16] = PSLEN, bit31 = BCN_OWN), DW1 = DMA low 32.
 * halmac_tx_bd_nic.h 15-20; mainline rtw88 pci.h
 * struct rtw_pci_tx_buffer_desc.
 */
struct rtl8192f_txbd_ele {
	__le16	buf_size;
	__le16	psb_len;
	__le32	dma;
} __packed;

/* TXBD: 4 elements x 8B = 32B.  The full 4-segment layout is kept so
 * hardware-encryption gather segments (ELE[2]/ELE[3]) can be added
 * later without moving the ring .
 */
struct rtl8192f_txbd {
	struct rtl8192f_txbd_ele ele[RTL8192F_TXBD_ELE_NUM];
} __packed;

/* RXBD element: DW0 = RXBUFFSIZE[13:0] + LS[14] + FS[15] +
 * TOTALRXPKTSIZE[28:16] + RXFAIL[31], DW1 = DMA low 32.
 * halmac_rx_bd_nic.h 7-13; mainline rtw88 pci.h
 * struct rtw_pci_rx_buffer_desc.  There is no OWN bit; a filled entry
 * is recognised by TOTALRXPKTSIZE != tag.
 */
struct rtl8192f_rxbd {
	__le16	buf_size;
	__le16	total_pkt_size;
	__le32	dma;
} __packed;

struct rtl8192f_tx_ring {
	struct rtl8192f_txbd	*bd;	/* coherent TXBD array, 32B/entry */
	dma_addr_t		bd_dma;
	struct rtl8192f_tx_desc_info *info;
	u32	head;		/* next TXBD to fill (host producer) */
	u32	tail;		/* next TXBD to reap (HW consumer) */
	u8	qsel;		/* fixed qsel (mgmt/high), 0 = per-skb TID */
	const char *name;
};

/* Number of hardware MACIDs (peer slots).  MACID 0 is the client-mode
 * default and never handed out by rtl8192f_alloc_macid().
 */
#define RTL8192F_NUM_MACID		32

/* BSSID slots this core can hold.  The MBIDCAM index field is five bits
 * and the vendor's clear loop covers eight; four is what this core's
 * CAM indices and the factory configuration both provide.
 */
#define RTL8192F_NUM_BSSID		4

/* Per-interface driver state (hw->vif_data_size): which MBIDCAM slot this
 * BSSID's address lives in, so removing the interface can free it again.
 */
struct rtl8192f_vif {
	u8 mbid;
	bool mbid_set;
};

/* Per-peer driver state (hw->sta_data_size): the MACID slot this peer's
 * frames carry, which is also the rate-adaptation entry the firmware was
 * given for it.  It cannot live in priv: with one slot there, a second
 * client overwrote it and the first client's teardown failed its "is this
 * my station" test, leaking the MACID.
 */
struct rtl8192f_sta {
	u8 macid;
};

/* Beacon single slot: one TXBD (128B stride area) + one 40B TX info
 * descriptor + the beacon payload, all coherent, allocated once at probe.
 *
 * The payload buffer is stable and permanently DMA-mapped: the hardware
 * re-reads it at every TBTT, so a content change is just a memcpy plus the
 * BSS_CHANGED_BEACON notification mac80211 sends whenever hostapd renews the
 * template (net/mac80211/cfg.c:1480 ieee80211_assign_beacon).  Re-mapping a
 * fresh skb per period was both a use-after-free window (the previous buffer
 * was freed while an in-flight fetch could still read it) and a race against
 * the TBTT.
 *
 * It was introduced because a one-shot upload appeared to leave a stale
 * template on air; the hardware counter says the schedule was never the
 * problem (HISR0 TXBCN0OK runs at 10.0/s with TXBCN0ERR 0 before
 * and after an interface restart), so what this change buys is the
 * removal of the free-while-DMA-may-still-fetch window and of a 10 Hz work
 * item - not a beacon-rate fix.
 */
struct rtl8192f_bcn_slot {
	struct rtl8192f_txbd	*bd;	/* inside the BCNQ TXBD area */
	dma_addr_t		bd_dma;
	void			*txdesc;	/* 40B TX info desc */
	dma_addr_t		txdesc_dma;
	void			*frame;		/* RTL8192F_BEACON_MAX_LEN payload */
	dma_addr_t		frame_dma;
	u32			frame_len;	/* bytes currently armed */
};

struct rtl8192f_rx_ring {
	struct rtl8192f_rxbd	*bd;
	dma_addr_t		bd_dma;	struct rtl8192f_rx_desc_info *info;
	u32	wp;		/* next RXBD to fill (host producer) */
	u32	rp;		/* next RXBD to process */
	/* Expected HW RXTAG counter value (RXBD_READY_CHECK_METHOD: the
	 * hardware stamps every RXBD it fills with a rolling 13-bit tag;
	 * a slot is ready when its TOTALRXPKTSIZE field equals this,
	 * vendor Hal88XXRxDesc.c 482-542).
	 */
	u16	rxtag;
};

struct rtl8192f_tx_desc_info {
	struct sk_buff *skb;
	DEFINE_DMA_UNMAP_ADDR(dma);
	DEFINE_DMA_UNMAP_LEN(len);
	u8	rate;		/* descriptor TX_RATE encoding, for the
				 * mac80211 rate-control report
				 */
	bool	sgi;		/* W5 DATA_SHORT on an HT rate: short GI */
};

struct rtl8192f_rx_desc_info {
	struct sk_buff *skb;
	DEFINE_DMA_UNMAP_ADDR(dma);
};

/* ------------------------------------------------------------------ */
/* Init table entry formats (ported from vendor PHYDM halhwimg files)  */
/* ------------------------------------------------------------------ */

struct rtl8192f_reg32val {
	u16 addr;
	u32 val;
};

struct rtl8192f_reg8val {
	u16 addr;
	u8 val;
};

struct rtl8192f_rfregval {
	u8 addr;
	u32 val;
};

/* Init tables (exported by tables_*.c).  The vendor arrays carry no
 * zero-address terminator (first MAC row is {0x000, 0x92}, last RadioA
 * row is {0x00, 0x35df5}), so consumers iterate over these explicit
 * lengths .
 */
#define RTL8192F_MAC_TABLE_LEN		144
#define RTL8192F_PHY_TABLE_LEN		464
#define RTL8192F_AGC_TABLE_LEN		132
#define RTL8192F_RF_TABLE_LEN		72

extern const struct rtl8192f_reg8val rtl8192f_soc_mac_init_table[RTL8192F_MAC_TABLE_LEN];
extern const struct rtl8192f_reg32val rtl8192f_soc_phy_init_table[RTL8192F_PHY_TABLE_LEN];
extern const struct rtl8192f_reg32val rtl8192f_soc_agc_table[RTL8192F_AGC_TABLE_LEN];
extern const struct rtl8192f_rfregval rtl8192f_soc_radioa_table[RTL8192F_RF_TABLE_LEN];

/* ------------------------------------------------------------------ */
/* Private state                                                       */
/* ------------------------------------------------------------------ */

/* Board efuse record: the factory block this WiFi core is calibrated
 * from, taken whole out of the flash MIB partition as a single nvmem
 * cell and indexed here, at the offsets the vendor's own record layout
 * uses (struct hw_wlan_setting, users/boa/.../rtk_tx_calibration.c
 * 43-148).  Power entries are one TXAGC codeword per 2.4 GHz channel,
 * channel 1 first; 0 means the board stores no level for that channel,
 * which is also how the vendor tests it (Hal88XXPhyCfg.c 4372, 4666).
 * The diff bytes pack two 4-bit signed steps each, low nibble path A
 * and high nibble path B (COUNT_SIGN_OFFSET(), 8192cd_cfg.h 322).
 */
#define RTL8192F_CAL_MAC			0
#define RTL8192F_CAL_PWR_CCK_A			48
#define RTL8192F_CAL_PWR_CCK_B			62
#define RTL8192F_CAL_PWR_HT40_A			76
#define RTL8192F_CAL_PWR_HT40_B			90
#define RTL8192F_CAL_DIFF_HT40_2S		104
#define RTL8192F_CAL_DIFF_HT20			118
#define RTL8192F_CAL_DIFF_OFDM			132
#define RTL8192F_CAL_REG_DOMAIN			146
#define RTL8192F_CAL_RFE_TYPE			147
#define RTL8192F_CAL_XTAL			149
#define RTL8192F_CAL_THERMAL			152
#define RTL8192F_NUM_CHANNEL			14
#define RTL8192F_CAL_LEN			163

struct rtl8192f_soc_priv {
	struct ieee80211_hw *hw;
	struct platform_device *pdev;
	void __iomem *regs;
	int irq;

	/* SoC-side WiFi clocks: the WLAN clock/function gate and the
	 * crystal reference behind the core's REF_SEL choice.
	 */
	struct clk *clk_core;
	struct clk *clk_ref;

	spinlock_t irq_lock;	/* HISR ack / ring indices */
	struct mutex conf_mutex;

	/* Host -> core firmware command mailbox (H2C): which box is next
	 * and a lock so two senders cannot interleave inside one box.
	 * Sleepable: the sender waits for the firmware to empty the box.
	 */
	struct mutex h2c_mutex;
	u8 h2c_box;

	struct rtl8192f_tx_ring tx_ring[Q_NUM];
	struct rtl8192f_rx_ring rx_ring;
	bool fw_ready;	/* core MIPS firmware loaded and running */
	struct rtl8192f_bcn_slot bcn;

	/* RX DMA buffer processing runs in a tasklet; the hard IRQ only
	 * advances the HW write pointer check and schedules it.  NAPI is
	 * the long-term direction, TODO.
	 */
	struct tasklet_struct rx_tasklet;

	/* TX queue stall watch (rtl8192f_tx_watch): the consumer index the
	 * hardware published for each host-advanced ring, and how many
	 * passes in a row it has not moved while that ring still holds
	 * descriptors.  Written only by the work, read there too.
	 */
	struct delayed_work tx_watch;
	u32 tx_last_rp[Q_NUM];
	u8 tx_stall[Q_NUM];
	u32 tx_recoveries;
	unsigned long tx_last_recover;

	bool hw_ready;
	bool running;
	u32 cur_channel;
	enum nl80211_channel_type cur_chan_type;
	u8 dig_igi;
	struct delayed_work dig_work;

	/* PD threshold saved before a 40 MHz switch cleared it, so the value
	 * the table (or a later AGC change) installed comes back on the way to
	 * 20 MHz - the vendor keeps the same single byte (phydm's
	 * original_pd_threshold).
	 */
	u8 pd_threshold;

	/* Security CAM bookkeeping (sec.c): which entries this driver holds.
	 * The hardware cannot tell an entry of ours from one left by an
	 * earlier boot, and a re-key only reaches the driver again when the
	 * old key is deleted, so the allocation map has to live here.
	 */
	DECLARE_BITMAP(cam_map, CAM_ENTRIES);
	/* Which key owns each CAM entry.  mac80211 hands the very same
	 * ieee80211_key_conf back to set_key after a hardware restart
	 * (ieee80211_reconfig() -> ieee80211_reenable_keys() ->
	 * drv_set_key, net/mac80211/util.c:2264, net/mac80211/key.c:974)
	 * without disabling it first, so the driver has to recognise its own
	 * slot again; hw_key_idx cannot carry that on its own, because once
	 * this key's slot was freed it can already belong to another key.
	 */
	struct ieee80211_key_conf *cam_owner[CAM_ENTRIES];

	/* Interface state.  One interface owns the beacon slot and the
	 * REG_MACID/REG_BSSID pair - the MAC sources every frame with that
	 * address - and any further BSS only gets an MBIDCAM entry, because
	 * this core has a single resident beacon slot (see
	 * RTL8192F_NUM_BCN_SLOT).
	 */
	struct ieee80211_vif *bss_vif;
	DECLARE_BITMAP(bssid_map, RTL8192F_NUM_BSSID);
	bool scanning;
	DECLARE_BITMAP(macid_map, RTL8192F_NUM_MACID);

	/* Firmware-to-host messages, counted where they are dropped (see
	 * rtl8192f_c2h_handle).  Nothing decodes them: the one per-peer TX
	 * totals message this family has (C2H id 4) only exists as the answer
	 * to H2C AP_REQ_TXREP (0x43), which is not requested here (fw.c, note
	 * above h2c_cmd_taken), so there is no aggregate to store.
	 * c2h_first_id/len record what did arrive, if anything.
	 */
	unsigned long c2h_cnt;
	u8 c2h_first_id;
	u32 c2h_first_len;

	u8 mac_addr[ETH_ALEN];
	u8 cut_version;	/* SYS_CFG[15:12], vendor ODM_CMNINFO_CUT_VER */

	u8 calib[RTL8192F_CAL_LEN];
	bool has_calib;

	/* flags reported to mac80211 by configure_filter() */
	u32 filter_flags;

	/* EFuse decoded logical map.  512 bytes: the gen-2.5 logical map
	 * is 64 sections x 4 word units x 2 bytes = 512 .
	 * Vendor 8192cd_hw.c ReadEFuse(): priv->EfuseMapLen.
	 */
	u8 efuse_map[512];

};

/* ------------------------------------------------------------------ */
/* I/O helpers                                                         */
/* ------------------------------------------------------------------ */

static inline u32 rtl8192f_read32(struct rtl8192f_soc_priv *priv, u16 addr)
{
	return readl(priv->regs + addr);
}

static inline void rtl8192f_write32(struct rtl8192f_soc_priv *priv,
				    u16 addr, u32 val)
{
	writel(val, priv->regs + addr);
}

static inline u8 rtl8192f_read8(struct rtl8192f_soc_priv *priv, u16 addr)
{
	return readb(priv->regs + addr);
}

static inline void rtl8192f_write8(struct rtl8192f_soc_priv *priv,
				   u16 addr, u8 val)
{
	writeb(val, priv->regs + addr);
}

static inline u16 rtl8192f_read16(struct rtl8192f_soc_priv *priv, u16 addr)
{
	return readw(priv->regs + addr);
}

static inline void rtl8192f_write16(struct rtl8192f_soc_priv *priv,
				    u16 addr, u16 val)
{
	writew(val, priv->regs + addr);
}

static inline void rtl8192f_setbits32(struct rtl8192f_soc_priv *priv,
				      u16 addr, u32 mask)
{
	rtl8192f_write32(priv, addr, rtl8192f_read32(priv, addr) | mask);
}

static inline void rtl8192f_clrbits32(struct rtl8192f_soc_priv *priv,
				      u16 addr, u32 mask)
{
	rtl8192f_write32(priv, addr, rtl8192f_read32(priv, addr) & ~mask);
}

/* BB/PHY access: BB registers live at 0x800+ (via MAC PHY IF), RF via
 * HSSI read/write registers.  Standard Realtek gen-2 indirect layout.
 */
#define RTL8192F_BB_BASE		0x0800
#define RTL8192F_HSSI_R1		0x0840	/* rFPGA0_XA_HSSI_PARM1 */
#define RTL8192F_HSSI_R2		0x0844	/* rFPGA0_XA_HSSI_PARM2 */
#define RTL8192F_HSSI_READ_TRIGGER	BIT(8)

void rtl8192f_phy_set_rf_reg(struct rtl8192f_soc_priv *priv, u8 addr, u32 val);
u32 rtl8192f_phy_get_rf_reg(struct rtl8192f_soc_priv *priv, u8 addr);
void rtl8192f_write_bb_reg(struct rtl8192f_soc_priv *priv, u16 addr, u32 val);
u32 rtl8192f_read_bb_reg(struct rtl8192f_soc_priv *priv, u16 addr);

/* RF register masks */
#define RFREG_MASK			0xfffff

/* ------------------------------------------------------------------ */
/* SoC-side WiFi clocks                                                */
/*                                                                     */
/* The core MMIO window (priv->regs) is only reachable after the       */
/* SoC-side WLAN clock/function gate is opened, and its analog front   */
/* end has to be told which crystal it is fed.  Both are provided by   */
/* the system-controller clock driver: "core" is the gate (vendor      */
/* 8192cd_osdep.c sets 0xB8000064 |= 0x1F on and = 0x00 off, and       */
/* 8192cd_util.h treats bit0 as "may access WiFi registers"), "ref"    */
/* carries the 25/40 MHz strap so REF_SEL can be derived from it.      */
/* ------------------------------------------------------------------ */

int rtl8192f_wlan_clk_enable(struct rtl8192f_soc_priv *priv);
void rtl8192f_platform_reset(struct rtl8192f_soc_priv *priv);
void rtl8192f_wlan_clk_disable(struct rtl8192f_soc_priv *priv);

/* ------------------------------------------------------------------ */
/* EFuse geometry (gen-2.5)                                            */
/* vendor rtl8192cd_hw.c ReadEFuse(): the physical EFuse content is a  */
/* stream of PG packets (1 or 2 byte header + up to 4 word-pairs);     */
/* packets are decoded into a logical byte map where section i / word  */
/* j lands at map[(i << 3) + (j << 1)].                                */
/* ------------------------------------------------------------------ */

#define EFUSE_MAX_WORD_UNIT	4
#define EFUSE_MAX_SECTION	64
#define EFUSE_REAL_CONTENT_LEN	512	/* physical efuse bytes */
#define EFUSE_MAP_LEN		512	/* logical map bytes */

/* MAC address inside the decoded logical map.  The vendor has no 8197F
 * branch for it (8192cd_eeprom.c:509 uses 0x2A for the crystal cap), so
 * a board whose efuse holds no MAC address falls back to a random
 * locally administered one rather than failing probe.
 */
#define EFUSE_MAP_MAC_OFFSET	0x2a

/* ------------------------------------------------------------------ */
/* core.c                                                              */
/* ------------------------------------------------------------------ */

irqreturn_t rtl8192f_interrupt(int irq, void *dev_id);
int rtl8192f_add_interface(struct ieee80211_hw *hw,
			   struct ieee80211_vif *vif);
void rtl8192f_remove_interface(struct ieee80211_hw *hw,
			       struct ieee80211_vif *vif);
int rtl8192f_sta_state(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		       struct ieee80211_sta *sta,
		       enum ieee80211_sta_state old_state,
		       enum ieee80211_sta_state new_state);
int rtl8192f_conf_tx(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		     unsigned int link_id, u16 queue,
		     const struct ieee80211_tx_queue_params *params);
void rtl8192f_sw_scan_start(struct ieee80211_hw *hw,
			    struct ieee80211_vif *vif, const u8 *mac_addr);
void rtl8192f_sw_scan_complete(struct ieee80211_hw *hw,
			       struct ieee80211_vif *vif);
/* sec.c: the hardware security CAM, reached only through the indirect
 * command/data pair.  rtl8192f_sec_enable() also switches the engine on in
 * REG_CR / REG_SECCFG, because the bring-up tables leave both at their
 * reset values and .start() re-runs them after any key was installed.
 */
int rtl8192f_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
		     struct ieee80211_vif *vif, struct ieee80211_sta *sta,
		     struct ieee80211_key_conf *key);
void rtl8192f_sec_enable(struct rtl8192f_soc_priv *priv);
void rtl8192f_bss_info_changed(struct ieee80211_hw *hw,
			       struct ieee80211_vif *vif,
			       struct ieee80211_bss_conf *info,
			       u64 changed);

/* ------------------------------------------------------------------ */
/* hw.c - power, init, channel, efuse                                  */
/* ------------------------------------------------------------------ */

/* rtl8192f_beacon_upload moved to txrx.c (88XX beacon single slot).
 * The core MIPS CPU firmware download lives in fw.c (vendor
 * InitMIPSFirmware88XX): TXBD/BCN path + on-chip DDMA into IMEM/DMEM.
 * Without it the core CPU never leaves reset and the TX drain stalls
 * with FIFO pages never returned.
 */

int rtl8192f_hw_power_on(struct rtl8192f_soc_priv *priv);
/* Dynamic initial gain (DIG).  The 8197F executes phydm's 11N branch, so the
 * counters and the hold/reset bits are the 11N set: addresses from
 * phydm_regdefine11n.h 101-183, the read bin layout from phydm_dig.c
 * 2700-2753 and the hold/reset order from phydm_dig.c 2959-3030.
 */
#define REG_FA_CCK_CTRL			0x0A2C	/* CCK FA/CCA hold + reset */
#define REG_FA_CCK_HIGH			0x0A58	/* CCK FA counter, bits 15:8 */
#define REG_FA_CCK_LOW			0x0A5C	/* CCK FA counter, bits 7:0 */
#define REG_FA_OFDM_HOLD		0x0C00	/* page C hold, bit31 */
#define REG_FA_OFDM_RESET		0x0C0C	/* page C reset, bit31 */
#define REG_INITIAL_GAIN		0x0C50	/* byte0 = initial gain code */
#define REG_FA_OFDM_SYNC		0x0CF0	/* fast sync / server board search */
#define REG_FA_OFDM_RESET2		0x0D00	/* page D hold bit31, reset bit27 */
#define REG_FA_OFDM_CCA			0x0DA0	/* OFDM CCA (low) / parity fail */
#define REG_FA_OFDM_HDR			0x0DA4	/* rate illegal / crc8 fail */
#define REG_FA_OFDM_MCS			0x0DA8	/* mcs fail (low half) */

/* Bounds and ladder for this radio as the vendor sets them: 1T1R takes
 * DM_DIG_MAX_AP-6 = 0x38 (phydm_dig.c 1083-1086), the 8197F lower bound is
 * 0x1e (phydm_dig.c 1090-1091), the AP-side FA thresholds are 0x100/0x200/0x300
 * (phydm_dig.c 116-118) and the step triple is 2/1/2 (phydm_dig.c 931-934).
 */
#define DIG_IGI_MAX			0x38
#define DIG_IGI_MIN			0x1e
#define DIG_FA_STEP_HIGH		2
#define DIG_FA_STEP_MID			1
#define DIG_FA_STEP_LOW			2
#define DIG_FA_TH_LOW			0x100
#define DIG_FA_TH_MID			0x200
#define DIG_FA_TH_HIGH			0x300
#define DIG_PERIOD_MS			2000

int rtl8192f_hw_init(struct rtl8192f_soc_priv *priv);
void rtl8192f_dig_work(struct work_struct *work);
void rtl8192f_dig_start(struct rtl8192f_soc_priv *priv);
void rtl8192f_dig_stop(struct rtl8192f_soc_priv *priv);
int rtl8192f_hw_fifo_init(struct rtl8192f_soc_priv *priv);
int rtl8192f_fw_download(struct rtl8192f_soc_priv *priv);
int rtl8192f_h2c_send(struct rtl8192f_soc_priv *priv, u8 id,
		      const u8 *payload, u8 len);
int rtl8192f_fw_alive(struct rtl8192f_soc_priv *priv);
void rtl8192f_c2h_handle(struct rtl8192f_soc_priv *priv, const u8 *pkt,
			 u32 len);
void rtl8192f_send_ra_mask(struct rtl8192f_soc_priv *priv, u8 macid,
			   const struct ieee80211_sta *sta);
u8 rtl8192f_ra_rate_id(const struct ieee80211_sta *sta);
void rtl8192f_hw_power_off(struct rtl8192f_soc_priv *priv);
int rtl8192f_set_channel(struct rtl8192f_soc_priv *priv, u32 channel,
			 enum nl80211_channel_type chan_type);
int rtl8192f_read_calib(struct rtl8192f_soc_priv *priv);
int rtl8192f_efuse_read(struct rtl8192f_soc_priv *priv);
int rtl8192f_efuse_get_macaddr(struct rtl8192f_soc_priv *priv);

/* ------------------------------------------------------------------ */
/* rate.c - rate helpers                                               */
/* ------------------------------------------------------------------ */

void rtl8192f_tx_fill_rate(struct rtl8192f_soc_priv *priv, __le32 *desc,
			   struct ieee80211_tx_info *info,
			   struct ieee80211_sta *sta);
/* desc points at the 24-byte 88XX RX info descriptor at the start of
 * the RX DMA buffer, phy_status at the appended PHY status right after
 * it (both interpreted per the 88XX layout, see rate.c).
 */
void rtl8192f_rx_fill_status(struct rtl8192f_soc_priv *priv, __le32 *desc,
			     void *phy_status,
			     struct ieee80211_rx_status *status);
u32 rtl8192f_legacy_rate_to_hw(int idx, bool cck);
u8 rtl8192f_mcs_to_hw(u8 mcs);
int rtl8192f_alloc_macid(struct rtl8192f_soc_priv *priv);
void rtl8192f_free_macid(struct rtl8192f_soc_priv *priv, u8 macid);

/* ------------------------------------------------------------------ */
/* txrx.c - descriptor rings, xmit, rx reclaim                         */
/* ------------------------------------------------------------------ */

int rtl8192f_txrx_alloc(struct rtl8192f_soc_priv *priv);
void rtl8192f_txrx_free(struct rtl8192f_soc_priv *priv);
int rtl8192f_txrx_start(struct rtl8192f_soc_priv *priv);
void rtl8192f_txrx_stop(struct rtl8192f_soc_priv *priv);
void rtl8192f_tx_watch(struct work_struct *work);
void rtl8192f_tx_watch_start(struct rtl8192f_soc_priv *priv);
void rtl8192f_tx_watch_stop(struct rtl8192f_soc_priv *priv);
netdev_tx_t rtl8192f_tx(struct ieee80211_hw *hw, struct sk_buff *skb,
			struct ieee80211_sta *sta);
void rtl8192f_tx_isr(struct rtl8192f_soc_priv *priv);
void rtl8192f_rx_isr(struct rtl8192f_soc_priv *priv);
void rtl8192f_beacon_upload(struct rtl8192f_soc_priv *priv,
			    struct sk_buff *skb);

/* ------------------------------------------------------------------ */
/* calib.c - IQK / calibration                                         */
/* ------------------------------------------------------------------ */

void rtl8192f_iqk(struct rtl8192f_soc_priv *priv);

#endif /* __RTL8192F_SOC_H */
