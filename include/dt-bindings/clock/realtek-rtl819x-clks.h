/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
/*
 * Device Tree clock indices for the Realtek RTL819x SoC family.
 *
 * Copyright (c) 2026 David Yang <mmyangfl@gmail.com>
 */

#ifndef _DT_BINDINGS_CLOCK_REALTEK_RTL819X_CLKS_H_
#define _DT_BINDINGS_CLOCK_REALTEK_RTL819X_CLKS_H_

#define RTL8197F_CLK_CPU	0
#define RTL8197F_CLK_LX		1
#define RTL8197F_CLK_UART	2
#define RTL8197F_CLK_SPI	3
/* WLAN core clock/function gate in the system controller */
#define RTL8197F_CLK_WLAN	4
/* Crystal reference fed to the WLAN core (25 or 40 MHz by strap) */
#define RTL8197F_CLK_WIFI_REF	5

#define RTL8197F_CLK_NR_CLKS	6

#endif /* _DT_BINDINGS_CLOCK_REALTEK_RTL819X_CLKS_H_ */
