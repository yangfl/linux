// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 David Yang <mmyangfl@gmail.com>
 *
 * Realtek RTL819x (RTL8197F) machine support for the generic MIPS platform.
 */

#include <asm/machine.h>

static const struct of_device_id rtl819x_of_match[] __initconst = {
	{ .compatible = "realtek,rtl8197f-soc" },
	{}
};

MIPS_MACHINE(rtl819x) = {
	.matches = rtl819x_of_match,
};
