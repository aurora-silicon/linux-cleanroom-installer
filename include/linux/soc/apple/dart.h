/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef __LINUX_SOC_APPLE_DART_H__
#define __LINUX_SOC_APPLE_DART_H__

#include <linux/kconfig.h>
#include <linux/errno.h>
#include <linux/types.h>

struct device;

/* J713 internal DCP only: requires validated, borrowed SID23 tables. */
int apple_dart_dcp_check(struct device *dev);
/* A borrowed RTKit payload must be fully writable, not inherited firmware TEXT. */
int apple_dart_dcp_check_buffer(struct device *dev, dma_addr_t iova, size_t size);
/* Retained CPU/DVA allocation: even failed publication pins pages until reboot. */
void *apple_dart_dcp_alloc(struct device *dev, size_t size, dma_addr_t *dva);

/* Qualified noncoherent J713 SID23/SID16 pair; no CPU access after publication. */
int apple_dart_dcp_shared_alloc(struct device *dev, struct device *piodma,
				size_t size, u64 *phys, u64 *dva);
int apple_dart_dcp_shared_map(struct device *dev, struct device *piodma,
			      u64 phys, u64 dva, size_t size);

/* Private retained SID0 scanout, not the firmware's capped D451 allocation class. */
void *apple_dart_dcp_scanout_alloc(struct device *dev, size_t size, dma_addr_t *dva);

/*
 * The tunneled PCIe IOMMU shares the port clock. Stop issuing commands
 * before that clock is gated, and allow them again once it is back.
 */
#if IS_ENABLED(CONFIG_APPLE_DART)
int apple_dart_quiesce_commands(struct device *dev);
int apple_dart_resume_commands(struct device *dev);
int apple_dart_save_tunnel_state(struct device *dev);
int apple_dart_restore_tunnel_state(struct device *dev);
#else
static inline int apple_dart_quiesce_commands(struct device *dev) { return 0; }
static inline int apple_dart_resume_commands(struct device *dev) { return 0; }
static inline int apple_dart_save_tunnel_state(struct device *dev) { return -EOPNOTSUPP; }
static inline int apple_dart_restore_tunnel_state(struct device *dev) { return -EOPNOTSUPP; }
#endif

#endif
