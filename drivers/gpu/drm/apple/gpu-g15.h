/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * J514S (T6030, G15S) GPU: state shared by the stage-1 firmware owner
 * (gpu-g15-core.c) and the stage-2 compute/DRM backend (gpu-g15-backend.c).
 */
#ifndef APPLE_GPU_G15_H
#define APPLE_GPU_G15_H

#include <linux/atomic.h>
#include <linux/ioport.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/workqueue.h>

struct apple_rtkit;
struct g15_backend;

#define G15_PAGE		0x4000ULL
#define G15_PA_MASK		0x3ffffffc000ULL

/* Firmware upper half: root[0..1] are firmware-owned, root[2] the host window. */
#define G15_UPPER		0xfffffc0000000000ULL
#define G15_HOST		0xfffffc2000000000ULL
#define G15_HOST_SIZE		0x1000000000ULL

/* UAT leaf attributes (contract section 3). */
#define G15_PTE_FW		0x00c000000000044bULL	/* firmware-only shared RW data */
#define G15_PTE_SHARED		0x00e000000000040bULL	/* FW+GPU shared auxiliary */
#define G15_PTE_MMIO		0x00c0000000000447ULL	/* firmware Device RW */
#define G15_PTE_TABLE		3ULL

#define G15_PPL_MAGIC		0x4b1d000000000002ULL

enum g15_region { G15_TTBS, G15_TABLES, G15_HANDOFF, G15_SHARED_L2, G15_TEXT, G15_DATA, G15_REGIONS };

/* Canonical object VAs (contract 4.1). */
#define G15_ROOT		0xfffffc204062ff40ULL
#define G15_REGION_A		0xfffffc2040670000ULL
#define G15_RUNTIME		0xfffffc20406b7b4dULL
#define G15_HWDATA		0xfffffc20406fb5fcULL
#define G15_PAIR		0xfffffc2040747f00ULL
#define G15_SMALL		0xfffffc204078bff0ULL
#define G15_C0			0xfffffc2070003000ULL
#define G15_C1			0xfffffc2070008000ULL
#define G15_C3			0xfffffc2070010000ULL
#define G15_GLOBALS		0xfffffc20407cc204ULL
#define G15_POWER		0xfffffc204081370cULL
#define G15_CONTROL		0xfffffc20003bbc30ULL
#define G15_FLAGS		0xfffffc2040857fc0ULL
#define G15_CH_STATE(i)		(0xfffffc2040003fd0ULL + (i) * 0x44000ULL)
#define G15_CH_RING(i)		(0xfffffc2000002800ULL + (i) * 0x44000ULL)
#define G15_DEVCTL_STATE	0xfffffc2040333fd0ULL
#define G15_DEVCTL_RING		0xfffffc2000330800ULL
#define G15_EVENT_STATE		0xfffffc2040377fd0ULL
#define G15_EVENT_RING		0xfffffc20403b8800ULL
#define G15_LOG_STATE		0xfffffc20403ffee0ULL
#define G15_LOG_RING		0xfffffc2040443000ULL
#define G15_TRACE_STATE		0xfffffc20404d7fd0ULL
#define G15_TRACE_RING		0xfffffc2040519000ULL
#define G15_STATS_STATE		0xfffffc2040563fd0ULL
#define G15_STATS_RING		0xfffffc20405a4000ULL
#define G15_FWCTL_STATE		0xfffffc20405ebfd0ULL
#define G15_FWCTL_RING		0xfffffc2000376c00ULL
#define G15_TIMESTAMPS		0xfffffc2ffc000000ULL

/* One host-window 16 KiB page: either owned DMA backing or a device page. */
struct g15_page {
	u64 va;
	void *cpu;	/* NULL for device (MMIO) pages */
	dma_addr_t pa;
};

#define G15_MAX_PAGES	2048
#define G15_MAX_L3	128

struct g15_gpu {
	struct device *dev;
	struct mutex lock;
	void __iomem *asc, *sgx_id, *sgx_open, *sgx_core, *sgx_status;
	void __iomem *ttbs, *tables, *handoff, *shared_l2;
	struct resource region[G15_REGIONS];
	u64 text_va, data_va;
	struct apple_rtkit *rtkit;
	void __iomem *crash;
	u64 crash_pa;
	u32 crash_size;
	struct work_struct drain;
	/* Owned UAT tables: context0 lower root and host-window L3 tables. */
	void *lower;
	dma_addr_t lower_pa;
	struct { u32 index; void *cpu; dma_addr_t pa; } l3[G15_MAX_L3];
	unsigned int l3_count;
	struct g15_page *pages;
	unsigned int page_count;
	u32 devctl_producer;
	u32 version, counts, cores, control;
	/* Receive-ring observations for the backend (event channel). */
	atomic_t compute_events, stage_events, event_errors, channel_errors;
	u64 event_ns[3];	/* last firing time per event bit (TA, FS, compute) */
	/* Last firmware error event (type 0 fault, 4 timeout): copied before event_errors++. */
	u32 err_type, err_raw[14];
	s32 err_slot;
	struct g15_backend *be;
	int error;
	bool started, voted, ready;
};

/* Core services (gpu-g15-core.c). Host-window edits run under gpu->lock. */
int g15_map_page(struct g15_gpu *gpu, u64 va, u64 pa, u64 attr);
int g15_unmap_page(struct g15_gpu *gpu, u64 va);
int g15_write(struct g15_gpu *gpu, u64 va, const void *src, size_t len);
u32 g15_r32(struct g15_gpu *gpu, u64 va);
int g15_w32(struct g15_gpu *gpu, u64 va, u32 v);
int g15_ppl_lock(struct g15_gpu *gpu);
void g15_ppl_unlock(struct g15_gpu *gpu);
void g15_tlbi_all(void);
void g15_drain_rings(struct g15_gpu *gpu);
void g15_dump_events(struct g15_gpu *gpu, u32 count);
void g15_fail(struct g15_gpu *gpu, int error);
int g15_devctl_record(struct g15_gpu *gpu, const u8 *record);

/* Stage-2 backend (gpu-g15-backend.c). */
int g15_backend_start(struct g15_gpu *gpu);
void g15_backend_kick(struct g15_gpu *gpu);

#endif
