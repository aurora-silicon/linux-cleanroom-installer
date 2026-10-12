/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_DCP_M4_PROFILE_H
#define APPLE_DCP_M4_PROFILE_H

#include <linux/types.h>

/*
 * Per-board internal-DCP profile. The protocol, property, frame and KMS layers
 * are shared; only these firmware-ABI and panel facts differ between the
 * J713 (T8132, macOS 26 firmware) and J514S (T6030, firmware 14.7.0) owners.
 */
struct dcp_m4_profile {
	const char *name;
	bool fw14;			/* J514S firmware 14.7 RPC family */
	u32 panel_width, panel_height;
	u32 boot_stride;		/* D101: inherited framebuffer stride */
	u32 swap_size;			/* A407 input */
	u32 swap_surface;		/* plane-0 surface */
	u32 swap_dva;			/* plane-0 surface DVA */
	u32 swap_nullable;		/* ten nullable surface bytes */
	u32 swap_tail;			/* final two bytes */
	u32 swap_nits;			/* binary64 brightness */
	u32 disabled_color;		/* background with no surface */
	u16 done_method;		/* swap completion callback */
	u32 done_size;
	u16 open_method;		/* first client open */
	u32 brightness_max, brightness_default;
	u32 width_mm, height_mm;
};

static const struct dcp_m4_profile dcp_m4_profile_j713 = {
	.name = "J713",
	.panel_width = 2560, .panel_height = 1664, .boot_stride = 10240,
	.swap_size = 0x1bd8, .swap_surface = 0x588, .swap_dva = 0xe38,
	.swap_nullable = 0x1bcb, .swap_tail = 0x1bd6, .swap_nits = 0x35e,
	.disabled_color = 0xff402060,
	.done_method = 590, .done_size = 0x730, .open_method = 457,
	.brightness_max = 500, .brightness_default = 120,
	.width_mm = 0, .height_mm = 0,
};

static const struct dcp_m4_profile dcp_m4_profile_j514s = {
	.name = "J514S",
	.fw14 = true,
	.panel_width = 3024, .panel_height = 1964, .boot_stride = 12096,
	.swap_size = 0x1b58, .swap_surface = 0x508, .swap_dva = 0xdb8,
	.swap_nullable = 0x1b4b, .swap_tail = 0x1b56, .swap_nits = 0x35c,
	.disabled_color = 0xff000000,
	.done_method = 589, .done_size = 0x6f0, .open_method = 455,
	.brightness_max = 500, .brightness_default = 140,
	.width_mm = 302, .height_mm = 196,
};

/* One internal DCP per machine; selected once at probe, before any session. */
static const struct dcp_m4_profile *dcp_m4_pf = &dcp_m4_profile_j713;

#endif
