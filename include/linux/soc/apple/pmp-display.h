/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef _LINUX_SOC_APPLE_PMP_DISPLAY_H
#define _LINUX_SOC_APPLE_PMP_DISPLAY_H

#include <linux/types.h>

struct device;
struct apple_pmp_display;
bool apple_pmp_display_retained_board(void);

/* Private bridge for the existing Rust PMP owner; no second mailbox owner. */
struct apple_pmp_display *apple_pmp_display_register(struct device *dev,
		const void *cookie, int (*send)(const void *cookie, u64 word));
bool apple_pmp_display_receive(struct apple_pmp_display *display, u8 endpoint, u64 word);
void apple_pmp_display_fail(struct apple_pmp_display *display, int error);
/* Internal: caller holds the bound supplier device lock. */
struct apple_pmp_display *apple_pmp_display_bound_owner(struct device *dev);

/* J713-only retained vote, serialized and held until reboot. */
int apple_pmp_j713_display_prepare(struct device *consumer); /* During consumer probe only. */
int apple_pmp_j713_display_get(struct device *consumer);
int apple_pmp_j713_display_check(struct device *consumer);

/* Same untagged request stream, independent retained AGX consumer link/vote. */
int apple_pmp_j713_gpu_prepare(struct device *consumer); /* During probe only. */
int apple_pmp_j713_gpu_get(struct device *consumer);
int apple_pmp_j713_gpu_check(struct device *consumer);

/*
 * J514S (T6030) display power. pre_run seeds the PTD DISP|ANS request before
 * the PMP CPU runs (called by pmp-core.rs; a no-op on other boards). prepare
 * holds the display PMGR chain and requires the bound PMP; get/check require
 * the PMP to have acknowledged the DISP request.
 */
int apple_pmp_display_pre_run(struct device *dev);
int apple_pmp_j514s_display_prepare(struct device *consumer);
int apple_pmp_j514s_display_get(struct device *consumer);
int apple_pmp_j514s_display_check(struct device *consumer);
int apple_pmp_j514s_gpu_prepare(struct device *consumer); /* During probe only. */
int apple_pmp_j514s_gpu_get(struct device *consumer);
int apple_pmp_j514s_gpu_check(struct device *consumer);

#endif
