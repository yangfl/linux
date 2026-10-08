// SPDX-License-Identifier: GPL-2.0-only
/*
 * RTL8197F (RTL8192F-SoC) init tables, ported row-by-row from the
 * vendor PHYDM image files (rtl8192cd/phydm/rtl8197f/, MP image v37):
 *   halhwimg8197f_mac.c  Array_MP_8197F_MAC_REG
 *   halhwimg8197f_bb.c   Array_MP_8197F_AGC_TAB / Array_MP_8197F_PHY_REG
 *   halhwimg8197f_rf.c   Array_MP_8197F_RadioA
 *
 * The vendor arrays are PHYDM conditional-lookup format: rows whose
 * address has bit31 set are IF/ELSE/ENDIF condition markers and rows
 * with bit30 set are negative-condition evaluators (they select the
 * board package / cut variant).  For the RTL8197F SoC target (package
 * type 0, cut A, internal PA - CheckPositive() never matches) the
 * vendor apply loops end up taking the ELSE branch of every block, so
 * the rows below are exactly the ELSE-branch content in original table
 * order.  Rows with address 0xffe (RF table) are the vendor 50 ms
 * settling-delay markers (phydm_regconfig8197f.c
 * odm_ConfigRFReg_8197F()); the apply code turns them into delays.
 */
#include "rtl8192f_soc.h"

/* RadioA: RF register writes; 0xffe rows are 50 ms delay markers. */
const struct rtl8192f_rfregval rtl8192f_soc_radioa_table[] = {
	{ 0x00, 0x00030000 },
	{ 0x18, 0x00000c07 },
	{ 0xca, 0x00002000 },
	{ 0x81, 0x0000fc00 },
	{ 0x82, 0x000003c0 },
	{ 0x84, 0x00000005 },
	{ 0x86, 0x000933a5 },
	{ 0x87, 0x00000000 },
	{ 0x88, 0x0007b030 },
	{ 0x8e, 0x00064540 },
	{ 0x8f, 0x000a8000 },
	{ 0xdf, 0x000600 },
	{ 0x51, 0x00000c1c },
	{ 0x52, 0x0009a007 },
	{ 0x53, 0x00018c47 },
	{ 0x54, 0x00090034 },
	{ 0x55, 0x00082000 },
	{ 0x56, 0x0000880f },
	{ 0x57, 0x0007f000 },
	{ 0x58, 0x000603f0 },
	{ 0x5a, 0x00050000 },
	{ 0x5b, 0x00000006 },
	{ 0x5c, 0x00000015 },
	{ 0x65, 0x00020000 },
	{ 0x6e, 0x00038319 },
	{ 0xdf, 0x000600 },
	{ 0xef, 0x00000100 },
	{ 0x33, 0x000000a5 },
	{ 0x33, 0x000040a8 },
	{ 0x33, 0x000080e8 },
	{ 0x33, 0x0000c4e6 },
	{ 0x33, 0x00010de5 },
	{ 0x33, 0x00014de8 },
	{ 0x33, 0x00018deb },
	{ 0x33, 0x0001cdee },
	{ 0x33, 0x00020df1 },
	{ 0x33, 0x00024df4 },
	{ 0x33, 0x00028df7 },
	{ 0xef, 0x00000000 },
	{ 0xef, 0x00020000 },
	{ 0x33, 0x00030026 },
	{ 0x33, 0x00038026 },
	{ 0x33, 0x00070026 },
	{ 0x33, 0x00078026 },
	{ 0xef, 0x00000000 },
	{ 0xef, 0x00020000 },
	{ 0x33, 0x00030021 },
	{ 0xef, 0x00000000 },
	{ 0xc9, 0x00000600 },
	{ 0xb0, 0x0ffbc3 },
	{ 0xb1, 0x033b87 },
	{ 0xb2, 0x00033762 },
	{ 0xb4, 0x0140f0 },
	{ 0xb5, 0x00014000 },
	{ 0xb6, 0x00012425 },
	{ 0xb7, 0x010e8c },
	{ 0xb8, 0x030fff },
	{ 0xb9, 0x000c0008 },
	{ 0xba, 0x00040005 },
	{ 0xc2, 0x00002c01 },
	{ 0xc3, 0x0000000b },
	{ 0xc4, 0x00081e2f },
	{ 0xc5, 0x0005c28f },
	{ 0xc6, 0x000000a0 },
	{ 0xc9, 0x00000600 },
	{ 0xca, 0x00002000 },
	{ 0xfe, 0x00000000 },
	{ 0x18, 0x00008c07 },
	{ 0xfe, 0x00000000 },
	{ 0xfe, 0x00000000 },
	{ 0xfe, 0x00000000 },
	{ 0x00, 0x00035df5 },
};

static_assert(ARRAY_SIZE(rtl8192f_soc_radioa_table) ==
		      RTL8192F_RF_TABLE_LEN,
	      "RF table rows out of sync with RTL8192F_RF_TABLE_LEN");
