// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * J514S (T6030, G15S) GPU stage 2: compute backend for the shared Asahi-UAPI
 * frontend (gpu-m4-drm.c).
 *
 * Cleanroom implementation of the written stage-2 contract: client VMs on the
 * persistent lower tree (gpu-m4-drm-vm-tree.c) bound to exclusive ASIDs with
 * the shared kernel upper root, one persistent compute graph in the
 * firmware/client apertures, the 0x893 work image and 0x2b8 microsequence,
 * channel-2 submission and retirement on both stamps, the work queue, the
 * channel consumer and the compute event. Prototype limits: compute only
 * (render is refused), one outstanding command globally, explicit sync only,
 * native (non-imported) BOs only. Everything firmware-visible that a failed
 * command may still reference is retained until reboot.
 */
#include <linux/bitmap.h>
#include <linux/delay.h>
#include <linux/dma-fence.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>
#include <linux/dma-buf.h>
#include <linux/dma-resv.h>
#include <linux/hrtimer.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <asm/arch_timer.h>
#include <asm/cacheflush.h>
#include <drm/drm_gem.h>

#include "gpu-g15.h"
#include "gpu-m4-drm.h"
#include "gpu-m4-drm-memory.h"
#include "gpu-m4-drm-scheduler.h"
#include "gpu-m4-drm-validation.h"
#include "gpu-m4-drm-vm-tree.h"

/* Firmware VA allocator window and the client native aperture (contract 2/3). */
#define G15_FW_BASE		0xfffffc2d00000000ULL
#define G15_FW_END		0xfffffc2e00000000ULL
#define G15_CLIENT_BASE		0x7000000000ULL
#define G15_CLIENT_END		0x8000000000ULL
#define G15_TS_PAGES		(0x4000000ULL / G15_PAGE)	/* 64 MiB arena */

/* Fixed client placement, identical in every VM (placement addendum Q2). */
#define G15_POOL_GVA(k)		(0x7100000000ULL + (k) * 0x404000ULL)
#define G15_FREELIST_GVA	0x7105858000ULL
#define G15_POOL_TABLE_PAGE	0x7105c5c000ULL
#define G15_POOL_TABLE_OFF	0x3000
#define G15_PREEMPT_GVA		0x7105c64000ULL
#define G15_WORK_PAGE		0x7300000000ULL
#define G15_WORK_OFF		0x3760
/* Compute command slots (scheduling contract 2): work page and preemption strides. */
#define G15_COMPUTE_SLOTS	16
#define G15_CWORK_STRIDE	0x4000ULL
#define G15_PREEMPT_STRIDE	0x10000ULL

/* Context-0 lower work alias: owned lower tree, GPU RW NC, non-global (Q3). */
#define G15_PTE_LOWER		0x00c0000000000c8bULL

#define G15_POOLS		22
#define G15_POOL_SIZE		0x400000ULL
#define G15_FREELIST_SEED	8192
#define G15_QUEUE_CAP		0x500
#define G15_STAMP_SEED		0xc1000000U
#define G15_UUID		0xc10000U
#define G15_EVENT_SLOT		2
#define G15_COMPUTE_CHANNEL	2
#define G15_COMPUTE_DOORBELL	0x0083000000000002ULL
#define G15_WORK_SIZE		0x893
#define G15_SEQ_SIZE		0x2b8
#define G15_MAX_ATTACHMENTS	16
#define G15_VM_TABLES		4096

/* Render (render contract + render placement addendum). */
#define G15_TA_CHANNEL		0
#define G15_FS_CHANNEL		1
#define G15_TA_DOORBELL		0x0083000000000000ULL
#define G15_FS_DOORBELL		0x0083000000000001ULL
#define G15_TA_SEED		0x7a000000U
#define G15_FS_SEED		0x3d000000U
#define G15_TA_UUID		0x7a0000U
#define G15_FS_UUID		0x3d0000U
#define G15_TA_WORK_SIZE	0x92b
#define G15_FS_WORK_SIZE	0xc73
#define G15_TA_SEQ_SIZE		0x2d8
#define G15_FS_SEQ_SIZE		0x310
#define G15_BARRIER_SIZE	0x3e
#define G15_PB_PAGES		8192
#define G15_PB_BLOCKS		2048
#define G15_PB_HEAP_GVA		0x7800000000ULL
#define G15_PB_HEAP_SIZE	0x10000000ULL
#define G15_PB_PAGE_LIST_GVA	0x1074000000ULL
#define G15_PB_BLOCK_LIST_GVA	0x100e0000000ULL
#define G15_UMA_GVA(e, k)	(0x10080000000ULL + (e) * 0x2450000ULL + (k) * 0x408000ULL)
#define G15_UMA_FREELIST_GVA(e)	(0x10082040000ULL + (e) * 0x2450000ULL)
#define G15_UMA_TABLE_PAGE(e)	(0x10082448000ULL + (e) * 0x2450000ULL)
#define G15_AUX_GVA		0x100848a0000ULL
#define G15_PRE0_PAGE		0x1006080000ULL
#define G15_PRE1_PAGE		0x1006098000ULL
#define G15_PRE2_PAGE		0x10060b0000ULL
#define G15_HEAP_META_PAGE	0x1006140000ULL
#define G15_SCENE_USER_GVA	0x1006158000ULL
#define G15_FS_SCRATCH_GVA	0x1006178000ULL
/* TPC/tilemap at their maximum single-layer size in the donor growth window. */
#define G15_TPC_GVA		0x7400000000ULL
#define G15_TPC_SIZE		0x1000000ULL
#define G15_TILEMAP_GVA		0x7402000000ULL
#define G15_TILEMAP_SIZE	0x800000ULL
/*
 * Render pass slots (scheduling contract 2, placement addendum strides): a
 * prefix of consecutive renders of one submission is published together,
 * member j in slot j. Slot 0 keeps the maximum TPC/tilemap; later slots hold
 * any single-layer panel-sized pass, and a larger member ends the prefix.
 */
#define G15_RENDER_SLOTS	16
#define G15_TPC_SLOT_SIZE	0x200000ULL
#define G15_TILEMAP_SLOT_SIZE	0x100000ULL
#define G15_TPC_SLOT_STRIDE	0x4000000ULL
#define G15_PASS_STRIDE		0x118000ULL	/* preemption, heap, scene user, FS scratch */
/*
 * Two-cluster vertex clustering (clustering contract 2): a clustered pass
 * names one GPU-cached allocation of 64 tilemap segments and four metadata
 * regions, no lower alias, after the ordinary TPC/tilemap slots. A pool of
 * twice the pass slots lets a retired pass's buffer be zeroed while the next
 * prefix runs. A larger eligible geometry than a buffer holds uses the
 * ordinary encoding.
 */
#define G15_CT_GVA		0x7440000000ULL
#define G15_CT_STRIDE		0x1000000ULL
#define G15_CT_SIZE		0x400000ULL
#define G15_CT_BUFS		(2 * G15_RENDER_SLOTS)
#define G15_CLUSTER_PB_PAGES	0x3c
#define G15_AUX_STRIDE		0x98000ULL
#define G15_WORK_STRIDE		0x20000ULL	/* TA/FS lower and client work pages */
#define G15_FS_WORK_PAGE	0x7200000000ULL
#define G15_FS_WORK_OFF		0x3380
#define G15_TA_WORK_PAGE	0x7200010000ULL
#define G15_TA_WORK_OFF		0x36c0
/* Client leaf words (render placement addendum R1). */
#define G15_LEAF_NC		0x00c0000000000c8bULL
#define G15_LEAF_CACHED		0x00c0000000000c83ULL
#define G15_LEAF_PB_LIST	0x00e0000000000c03ULL
#define G15_MAX_CLIENT		320

static bool wb_sweep;
module_param(wb_sweep, bool, 0644);
MODULE_PARM_DESC(wb_sweep, "Clean+invalidate every WRITEBACK BO of the VM to PoC around each command (non-coherent fallback; default off: hardware-coherent like the donor)");

static unsigned int compute_slots = 16;
module_param(compute_slots, uint, 0444);
MODULE_PARM_DESC(compute_slots, "Compute commands of one submission published together, 1..16");

static unsigned int render_slots = 16;
module_param(render_slots, uint, 0444);
MODULE_PARM_DESC(render_slots, "Render passes of one submission published together, 1..16");

static bool mixed_prefix = true;
module_param(mixed_prefix, bool, 0644);
MODULE_PARM_DESC(mixed_prefix, "Publish computes and renders of one submission in one ordered prefix (up to compute_slots + render_slots), cross-engine order kept by firmware barriers; 0: one engine per prefix");

static bool vertex_clustering = true;
module_param(vertex_clustering, bool, 0644);
MODULE_PARM_DESC(vertex_clustering, "Tile eligible passes (one layer, one sample, 32x32 utiles) on both clusters; storage is provisioned only if set at load");

static bool ct_async = true;
module_param(ct_async, bool, 0644);
MODULE_PARM_DESC(ct_async, "Zero a retired pass's clustering buffer in the background while the next prefix runs; 0: zero it when it is next claimed");

static unsigned int max_inflight = 2;
module_param(max_inflight, uint, 0644);
MODULE_PARM_DESC(max_inflight, "Prefixes of one VM published at once, 1..2: the next is appended to the running queues and starts when the previous one's work ends; 1: publish only after full retirement");

static bool slot_rotate = true;
module_param(slot_rotate, bool, 0644);
MODULE_PARM_DESC(slot_rotate, "Give each prefix's computes the slots after the previous prefix's, so slots are scrubbed in the background while the next prefix runs; 0: always from slot 0");

static bool pass_full_reset = true;
module_param(pass_full_reset, bool, 0644);
MODULE_PARM_DESC(pass_full_reset, "Also zero a pass slot's auxiliary, heap-metadata and scene-user buffers on every use (per-use reset contract)");

static bool early_tiling = true;
module_param(early_tiling, bool, 0444);
MODULE_PARM_DESC(early_tiling, "Advertise fragment-only render dependencies: a flagged render's tiling may overlap the previous pass's fragments within a prefix");

static unsigned int deadline_ms = 5000;
module_param(deadline_ms, uint, 0644);
MODULE_PARM_DESC(deadline_ms, "Host deadline for a published prefix; firmware progress timeouts may need longer");

static bool gpu_recovery = true;
module_param(gpu_recovery, bool, 0644);
MODULE_PARM_DESC(gpu_recovery, "After a firmware fault/timeout event, acknowledge the firmware halt, fail the affected submission and continue; 0 latches the GPU failed until reboot");

static bool hang_preempt = true;
module_param(hang_preempt, bool, 0644);
MODULE_PARM_DESC(hang_preempt, "At the host deadline of a compute/FS prefix, ask the firmware to preempt (Globals +0x1c/+0x24 with DEVCTL 0x13) so a non-yielding job enters firmware recovery");

static unsigned int compute_fault_mask = 1;
module_param(compute_fault_mask, uint, 0644);
MODULE_PARM_DESC(compute_fault_mask, "Compute Start +0x30: 1 (donor) masks the context's MMU faults (unmapped stores are absorbed), 0 lets them fault");

static unsigned int repoll_us = 15;
module_param(repoll_us, uint, 0644);
MODULE_PARM_DESC(repoll_us, "Retry a prefix ready behind the running ones, held only by a channel's unconsumed record, after this many us, 5..100 (0: at the next completion check)");

/*
 * Firmware cache policy (cache contract 1): the firmware's MAIR makes
 * AttrIndex 2 inner non-cacheable for the ASC, so every firmware step that
 * reads our command objects through it misses its caches. The objects the
 * firmware itself cleans and invalidates around each use are mapped with
 * AttrIndex 0 instead, keeping their access class: microsequences (encoded
 * length at completion), compute/TA/FS work images (0x880/0x920/0xc60 at
 * admission and completion; each image ends inside the last line of that
 * span, so every byte is in a maintained line), barriers (0x30 of the
 * one-line 0x3e record) and InitBM (0x20). Queue-info, the scheduling
 * contexts and the queues' job lists are firmware-private after the host
 * initializes them, before the firmware first sees their addresses; so is
 * each queue's scratch (queue-info +0x18), the firmware's own per-queue
 * working storage, which the common backend allocates firmware-private
 * cached: it becomes firmware-only (GPU inaccessible) as well.
 * Host-polled or host-updated cells (stamps, queue state, thresholds,
 * counters, notifiers, timestamp cells) and GPU-written storage keep
 * AttrIndex 2. All of these objects are retained until reboot, so no cached
 * mapping is ever torn down.
 */
static bool fw_cached = true;
module_param(fw_cached, bool, 0444);
MODULE_PARM_DESC(fw_cached, "Map microsequences, work images, barriers/InitBM, queue-info, queue scratch, scheduling contexts and job lists ASC-cached (firmware AttrIndex 0); 0: AttrIndex 2 (inner non-cacheable)");

static unsigned int resnap_us = 100;
module_param(resnap_us, uint, 0644);
MODULE_PARM_DESC(resnap_us, "Completion re-check interval while a command is published, 25..1000 us (EP0x20 events wake earlier)");

static const struct drm_asahi_params_global g15_params = {
	.features = DRM_ASAHI_FEATURE_VM_STATUS | DRM_ASAHI_FEATURE_SCHEDULED_QUEUES |
		DRM_ASAHI_FEATURE_COMPUTE_WIDE_VISIBILITY,
	.gpu_generation = 15,
	.gpu_variant = 'S',
	.gpu_revision = 0x11,
	.chip_id = 0x6030,
	.num_dies = 1,
	.num_clusters_total = 2,
	.num_cores_per_cluster = 10,
	/* core_masks and max_frequency_khz: per machine, set at registration. */
	.vm_start = 0x4000,
	.vm_end = 0x2ffffff8000ULL,
	.vm_kernel_min_size = 0x20000000,
	.max_commands_per_submission = 64,
	.max_attachments = 16,
	.command_timestamp_frequency_hz = 1000000000,
	.usc_generation = 3,
	.gpu_hal_generation = DRM_ASAHI_GPU_HAL_LEGACY,
};

/* ----------------------------------------------------------- buffers --- */

/*
 * One owned allocation of 16 KiB DMA pages (NC for this non-coherent device),
 * optionally aliased at a canonical firmware VA (va) and/or a fixed lower GPU
 * VA shared by every client VM (gva). va/gva name the payload, which starts
 * off bytes into the first page in every alias. Retained until reboot.
 */
struct g15_buf {
	u64 va, gva, off, size;
	u64 client_attrs;	/* client DRIVER leaf (0: GPU-only NC RW) */
	u32 count;
	void **cpu;		/* NULL entries: GPU-only pages without a CPU view */
	u64 *pa;
};

struct g15_slot {
	struct g15_buf work, seq, preempt, tail, its, uts, scratch[5];
	struct g15_buf barrier;	/* mixed prefix: wait on the previous render's FS */
	u32 state;		/* G15_CT_* scrub state of its mutable storage (be->ct_lock) */
};

struct g15_graph {
	struct g15_buf legacy, pools[G15_POOLS], freelist, pool_table, manager, counter;
	struct g15_buf sched_ctx, notifier, threshold, job_list;
	struct g15_buf qinfo, qstate, ring, qscratch, user_stamp, fw_stamp;
	struct g15_slot slot[G15_COMPUTE_SLOTS];
	u32 slots;
	u32 cursor;				/* next slot to try */
};

/* One render pass slot (slot 0 only in this version). */
/*
 * A clustering buffer is all zero while CLEAN (a compute slot: its runtime
 * storage scrubbed). A prefix claims it (USED; dirty: the span its pass may
 * write); after the prefix retires it is DIRTY until the scrub worker
 * (CLEARING) or a claim with no CLEAN one left resets it. States change under
 * be->ct_lock.
 */
enum { G15_CT_CLEAN, G15_CT_USED, G15_CT_DIRTY, G15_CT_CLEARING, G15_CT_STALE };

struct g15_ct {
	struct g15_buf b;
	u64 dirty;
	u32 state;
};

struct g15_pass {
	struct g15_buf aux, pre[3], tpc, tilemap, heap, scene_user, fs_scratch;
	struct g15_buf ta_work, fs_work, ta_seq, fs_seq, ta_scratch, scene;
	struct g15_buf its, uts, own_barrier, prev_barrier, fs_dep;
};

struct g15_render {
	struct g15_buf pools[2][8], freelist[2], pool_table[2], uma_mgr[2], uma_counter[2];
	struct g15_buf sched_ctx, notifier, threshold, job_list;
	struct g15_buf block_ctl, pb_counter, mgr_misc, scene_list, pb_mgr, legacy_empty;
	struct g15_buf pb_pages, pb_blocks, pb_heap, initbm;
	struct g15_buf qinfo[2], qstate[2], ring[2], qscratch[2], user_stamp[2], fw_stamp[2];
	struct g15_pass pass[G15_RENDER_SLOTS];
	u32 slots;
	u32 pre_records;	/* preemption records per buffer: 2, 3 with clustering */
	struct g15_ct ct[G15_CT_BUFS];
	u32 ct_count;
	u32 pass_busy;		/* pass slots held by published or prepared prefixes */
	u32 pass_cursor;	/* next small pass slot (1..slots-1) to try */
	bool reinit;		/* re-initialize allocators + InitBM before the next render */
	u64 draw;		/* cumulative host draw ordinal of the last publication */
	bool registered, c1_written;
};

struct g15_job;

/*
 * One published prefix (scheduling contract 2; two-prefix contract 4): its
 * job and commands, its own receipts (final targets, exclusive ring ends and
 * the ring positions before it, channel producers after its records) and the
 * storage it holds until its own retirement.
 */
struct g15_flight {
	struct g15_job *job;
	u32 count, computes, renders;
	u32 target, head, c_start;
	u32 ta_target, fs_target, ta_head, fs_head, ta_start, fs_start;
	u32 rec[3];
	u32 slot[G15_COMPUTE_SLOTS], slot_count;	/* compute slot of compute member j */
	u32 pass[G15_RENDER_SLOTS], pass_mask;		/* pass slot of render member j */
	struct g15_ct *ct[G15_RENDER_SLOTS];
	u32 ct_count;
	bool rotating;
	u64 pub_ns;
	unsigned long published_at;
};

#define G15_FLIGHTS	2

struct g15_backend {
	struct g15_gpu *gpu;
	struct gpu_m4_drm *drm;
	/* Engine, graph contents, ASIDs, allocators, PPL use. Never taken in an
	 * RTKit callback; nests outside gpu->lock.
	 */
	struct mutex lock;
	u64 fw_next;
	void *lower_l1[64];
	struct { u64 base; u64 *cpu; } lower_l2[16];
	u32 lower_l2_count;
	struct g15_graph g;
	struct g15_render r;
	struct g15_buf *client[G15_MAX_CLIENT];	/* mapped into every VM */
	u32 client_count;
	DECLARE_BITMAP(asids, 64);
	DECLARE_BITMAP(ts_pages, G15_TS_PAGES);
	/*
	 * Engine: published prefixes in order (fl, oldest first); a later one
	 * only for the same VM, appended to the running queues. Ready jobs are
	 * taken by queue priority, then the least recently served queue (turn),
	 * at prefix boundaries. pending holds jobs with unpublished commands.
	 */
	struct list_head pending, retained;
	struct g15_flight fl[G15_FLIGHTS];
	u32 fl_first, fl_count;
	u32 tail_prev, tail_ct, tail_fs;	/* last member of the newest prefix */
	u64 turn;
	u64 ordinal;		/* host command ordinal n of the last publication */
	/* Newest published receipts (the tail); event counts at its publication. */
	u32 target, head, events;
	u32 ta_target, fs_target, ta_head, fs_head, stage_seen;
	u32 producer[3];	/* host channels 0 TA, 1 FS, 2 compute */
	unsigned long deadline;	/* of the oldest published prefix */
	u64 st_overlaps;	/* prefixes published while another ran */
	/*
	 * Statistics (be->lock): commands per engine (0 compute, 1 render);
	 * prefixes and publish-to-retire-observed time per prefix kind (0
	 * compute, 1 render, 2 mixed); host gap from a retirement to the next
	 * publication while work was ready.
	 */
	u64 pub_ns, idle_from_ns, starve_from_ns, prep_from_ns;
	u64 st_starved_ns, st_prep_ns;	/* idle with nothing ready; prefix preparation */
	/*
	 * Internal stage timestamps of retired prefixes (24 MHz ticks): summed
	 * TA, FS and compute durations, and the span from the earliest start
	 * to the latest end of each prefix.
	 */
	u64 st_ta_ticks, st_fs_ticks, st_cdm_ticks, st_span_ticks;
	u64 st_count[3], st_prefixes[3], st_busy_ns[3], st_max_ns[3];	/* [2]: mixed commands */
	u64 st_gap_ns, st_gaps, st_yields;
	u64 st_first_ns[3], st_last_ns[3];	/* publish to first / last stage event */
	u64 st_jobs, st_job_cmds, st_job_max;	/* retired jobs and their command counts */
	u64 st_early;				/* early-tiled render members */
	u64 st_clustered;			/* clustered render members */
	u64 st_ct_sync;				/* clustering buffers zeroed at claim */
	u64 st_slot_sync;			/* compute slots scrubbed at claim */
	bool chan_wait;		/* a ready prefix waits only for a record's consumption */
	spinlock_t ct_lock;
	struct work_struct ct_clear;
	/*
	 * ct_clear runs here: preparation (drm_sched's reclaim workqueue)
	 * flushes it when every slot or buffer is still being reset.
	 */
	struct workqueue_struct *clear_wq;
	struct dentry *debugfs;
	bool queue_registered;
	int failed;
	int errors_seen;	/* gpu->event_errors already handled by recovery */
	u32 preempt_global;	/* Globals offset of the armed preemption divisor, or 0 */
	bool fs_resync;		/* next render waits on the reached FS stamp */
	u64 st_recoveries, st_hang_kills;
	struct delayed_work poll;
	struct hrtimer resnap;		/* high-resolution poll fallback */
	atomic_t evented;		/* an EP0x20 drain kicked the poll */
	struct work_struct reap;
	struct list_head dead_vms;
	spinlock_t dead_lock;
	/* Retirement fences: one context per queue, signaled in queue order. */
	spinlock_t fence_lock;
};

static void *g15_page_cpu(const struct g15_buf *b, u64 off)
{
	return (u8 *)b->cpu[off / G15_PAGE] + (off & (G15_PAGE - 1));
}

/* Payload-relative byte writes. */
static void g15_buf_write(const struct g15_buf *b, u64 off, const void *src, size_t len)
{
	off += b->off;
	while (len) {
		size_t n = min_t(size_t, len, G15_PAGE - (off & (G15_PAGE - 1)));

		memcpy(g15_page_cpu(b, off), src, n);
		off += n;
		src = (const u8 *)src + n;
		len -= n;
	}
}

static void g15_buf_fill(const struct g15_buf *b, u64 off, u8 value, size_t len)
{
	off += b->off;
	while (len) {
		size_t n = min_t(size_t, len, G15_PAGE - (off & (G15_PAGE - 1)));

		memset(g15_page_cpu(b, off), value, n);
		off += n;
		len -= n;
	}
}

static void g15_buf_w32(const struct g15_buf *b, u64 off, u32 v)
{
	u8 x[4];

	put_unaligned_le32(v, x);
	g15_buf_write(b, off, x, 4);
}

static void g15_buf_w64(const struct g15_buf *b, u64 off, u64 v)
{
	u8 x[8];

	put_unaligned_le64(v, x);
	g15_buf_write(b, off, x, 8);
}

/* Aligned 32-bit observation of a firmware/GPU-written cell. */
static u32 g15_buf_r32(const struct g15_buf *b, u64 off)
{
	return READ_ONCE(*(u32 *)g15_page_cpu(b, b->off + off));
}

/* Context-0 lower tree: owned coherent tables under gpu->lower. */
static int g15_lower_map(struct g15_backend *be, u64 gva, u64 pa)
{
	struct g15_gpu *gpu = be->gpu;
	u32 i0 = (gva >> 36) & 63, i1 = (gva >> 25) & 2047, i2 = (gva >> 14) & 2047;
	u64 base = gva & ~((1ULL << 25) - 1);
	u64 *root = gpu->lower, *l1, *l2 = NULL;
	dma_addr_t tpa;

	if (!be->lower_l1[i0]) {
		l1 = dma_alloc_coherent(gpu->dev, G15_PAGE, &tpa, GFP_KERNEL);
		if (!l1)
			return -ENOMEM;
		be->lower_l1[i0] = l1;
		wmb();
		WRITE_ONCE(root[i0], tpa | G15_PTE_TABLE);
	}
	l1 = be->lower_l1[i0];
	for (u32 i = 0; i < be->lower_l2_count; i++)
		if (be->lower_l2[i].base == base)
			l2 = be->lower_l2[i].cpu;
	if (!l2) {
		if (be->lower_l2_count == ARRAY_SIZE(be->lower_l2))
			return -ENOSPC;
		l2 = dma_alloc_coherent(gpu->dev, G15_PAGE, &tpa, GFP_KERNEL);
		if (!l2)
			return -ENOMEM;
		be->lower_l2[be->lower_l2_count].base = base;
		be->lower_l2[be->lower_l2_count++].cpu = l2;
		wmb();
		WRITE_ONCE(l1[i1], tpa | G15_PTE_TABLE);
	}
	if (l2[i2])
		return -EEXIST;
	WRITE_ONCE(l2[i2], pa | G15_PTE_LOWER);
	return 0;
}

/*
 * Allocate a size-byte payload at off into owned zeroed 16 KiB pages.
 * fw_attr != 0 maps a canonical firmware alias; gpage names the fixed client
 * page VA (mapped into each VM later; lower: also aliased in context 0).
 * be->lock held.
 */
static int __g15_buf_alloc(struct g15_backend *be, struct g15_buf *b, u64 size, u64 fw_attr,
			   u64 gpage, u64 off, bool lower, bool nomap)
{
	struct g15_gpu *gpu = be->gpu;
	int ret = 0;

	b->off = off;
	b->size = ALIGN(off + size, G15_PAGE);
	b->count = b->size / G15_PAGE;
	b->cpu = kvcalloc(b->count, sizeof(*b->cpu), GFP_KERNEL);
	b->pa = kvcalloc(b->count, sizeof(*b->pa), GFP_KERNEL);
	if (!b->cpu || !b->pa)
		return -ENOMEM;
	for (u32 i = 0; i < b->count; i++) {
		dma_addr_t pa;

		if (nomap) {
			/* GPU-only: zero, push to PoC, never touched by the CPU again. */
			struct page *page = alloc_page(GFP_KERNEL | __GFP_ZERO);
			unsigned long va;

			if (!page)
				return -ENOMEM;
			va = (unsigned long)page_address(page);
			dcache_clean_poc(va, va + G15_PAGE);
			dcache_inval_poc(va, va + G15_PAGE);
			b->pa[i] = page_to_phys(page);
			pa = b->pa[i];
		} else {
			b->cpu[i] = dma_alloc_coherent(gpu->dev, G15_PAGE, &pa, GFP_KERNEL);
			if (!b->cpu[i])
				return -ENOMEM;
			b->pa[i] = pa;
		}
		if (!pa || (pa & (G15_PAGE - 1)) || (pa & ~G15_PA_MASK))
			return -EPERM;
	}
	if (fw_attr) {
		if (be->fw_next + b->size + G15_PAGE > G15_FW_END)
			return -ENOSPC;
		mutex_lock(&gpu->lock);
		for (u32 i = 0; !ret && i < b->count; i++)
			ret = g15_map_page(gpu, be->fw_next + i * G15_PAGE, b->pa[i], fw_attr);
		mutex_unlock(&gpu->lock);
		if (ret)
			return ret;
		b->va = be->fw_next + off;
		be->fw_next += b->size + G15_PAGE;	/* unmapped guard page */
	}
	if (gpage) {
		if ((gpage & (G15_PAGE - 1)) || gpage + b->size > g15_params.vm_end)
			return -EINVAL;
		b->gva = gpage + off;
		for (u32 i = 0; lower && !ret && i < b->count; i++)
			ret = g15_lower_map(be, gpage + i * G15_PAGE, b->pa[i]);
		if (!ret && be->client_count == G15_MAX_CLIENT)
			ret = -ENOSPC;
		if (!ret)
			be->client[be->client_count++] = b;
	}
	return ret;
}

static int g15_buf_alloc(struct g15_backend *be, struct g15_buf *b, u64 size, u64 fw_attr,
			 u64 gpage, u64 off, bool lower)
{
	return __g15_buf_alloc(be, b, size, fw_attr, gpage, off, lower, false);
}

/* ------------------------------------------------------ compute graph --- */

/* Leaf classes of firmware-maintained command objects (see fw_cached). */
static u64 g15_cmd_fw(void)
{
	return fw_cached ? G15_PTE_FW_CACHED : G15_PTE_FW;
}

static u64 g15_cmd_shared(void)
{
	return fw_cached ? G15_PTE_SHARED_CACHED : G15_PTE_SHARED;
}

/* Queue scratch: firmware-private cached, or the previous shared index-2 class. */
static u64 g15_queue_scratch(void)
{
	return fw_cached ? G15_PTE_FW_CACHED : G15_PTE_SHARED;
}

static int g15_graph_alloc(struct g15_backend *be)
{
	struct g15_graph *g = &be->g;
	struct g15_slot *s;
	const u64 sh = G15_PTE_SHARED, fw = G15_PTE_FW;
	const u64 cfw = g15_cmd_fw(), csh = g15_cmd_shared();
	int ret;

	ret = g15_buf_alloc(be, &g->legacy, 0x4000, sh, 0, 0, false);
	for (u32 i = 0; !ret && i < G15_POOLS; i++)
		ret = g15_buf_alloc(be, &g->pools[i], G15_POOL_SIZE, sh, G15_POOL_GVA(i), 0, false);
	ret = ret ?: g15_buf_alloc(be, &g->freelist, 0x400000, sh, G15_FREELIST_GVA, 0, false);
	ret = ret ?: g15_buf_alloc(be, &g->pool_table, 0x1000, sh, G15_POOL_TABLE_PAGE,
				   G15_POOL_TABLE_OFF, false);
	ret = ret ?: g15_buf_alloc(be, &g->manager, 0x84, sh, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &g->counter, 0x40, sh, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &g->sched_ctx, 0x40, cfw, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &g->notifier, 0x108, fw, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &g->threshold, 4, fw, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &g->job_list, 0x18, cfw, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &g->qinfo, 0xb4, cfw, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &g->qstate, 0x60, fw, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &g->ring, G15_QUEUE_CAP * 8, fw, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &g->qscratch, 0x40000, g15_queue_scratch(), 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &g->user_stamp, 4, sh, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &g->fw_stamp, 4, sh, 0, 0, false);
	/* Distinct mutable command slots; shared queue, stamps and allocator. */
	g->slots = clamp(compute_slots, 1U, (u32)G15_COMPUTE_SLOTS);
	for (u32 k = 0; !ret && k < g->slots; k++) {
		s = &g->slot[k];
		ret = g15_buf_alloc(be, &s->its, 16, sh, 0, 0, false);
		ret = ret ?: g15_buf_alloc(be, &s->uts, 16, fw, 0, 0, false);
		ret = ret ?: g15_buf_alloc(be, &s->preempt, 0x10000, sh,
					   G15_PREEMPT_GVA + k * G15_PREEMPT_STRIDE, 0, false);
		ret = ret ?: g15_buf_alloc(be, &s->work, G15_WORK_SIZE, csh,
					   G15_WORK_PAGE + k * G15_CWORK_STRIDE, G15_WORK_OFF, true);
		ret = ret ?: g15_buf_alloc(be, &s->tail, 0x10000, sh, 0, 0, false);
		ret = ret ?: g15_buf_alloc(be, &s->seq, 0x1000, cfw, 0, 0, false);
		for (u32 i = 0; !ret && i < 5; i++)
			ret = g15_buf_alloc(be, &s->scratch[i], 0x10000, sh, 0, 0, false);
	}
	return ret;
}

/*
 * Per-use reset of a retired compute slot's runtime storage (scheduling
 * contract 2, per-use reset contract a): timestamp cells, tail and sequence
 * scratch zeroed, preemption poisoned with 0xcc.
 */
static void g15_slot_scrub(struct g15_slot *s)
{
	g15_buf_fill(&s->its, 0, 0, 16);
	g15_buf_fill(&s->tail, 0, 0, 0x10000);
	for (u32 k = 0; k < 5; k++)
		g15_buf_fill(&s->scratch[k], 0, 0, 0x10000);
	g15_buf_fill(&s->preempt, 0, 0xcc, 0x10000);
}

/* Initial contents (contract section 3); firmware owns them afterwards. */
static void g15_graph_init(struct g15_backend *be)
{
	struct g15_graph *g = &be->g;
	u32 entry = 0;

	/* Freelist: the first eight 4 MiB pools in 0x1000 units. */
	for (u32 k = 0; k < 8; k++)
		for (u64 off = 0; off < G15_POOL_SIZE; off += 0x1000)
			g15_buf_w64(&g->freelist, 8ULL * entry++, g->pools[k].gva + off);
	for (u32 k = 0; k < G15_POOLS; k++)
		g15_buf_w64(&g->pool_table, 0x40ULL * k, g->pools[k].gva | BIT_ULL(62));
	g15_buf_w32(&g->manager, 0x00, 3);
	g15_buf_w32(&g->manager, 0x08, 2);
	g15_buf_w32(&g->manager, 0x0c, 1);
	g15_buf_w64(&g->manager, 0x14, g->freelist.gva);
	g15_buf_w32(&g->manager, 0x1c, 0x80000);
	g15_buf_w32(&g->manager, 0x24, G15_FREELIST_SEED);
	g15_buf_w32(&g->manager, 0x2c, G15_FREELIST_SEED);
	g15_buf_w64(&g->manager, 0x34, g->pool_table.gva);
	g15_buf_w32(&g->manager, 0x44, 0x40);
	g15_buf_w64(&g->manager, 0x48, g->counter.va);
	g15_buf_w32(&g->counter, 0, 1);

	g15_buf_fill(&g->sched_ctx, 0x00, 0xff, 2);
	g15_buf_fill(&g->sched_ctx, 0x05, 0x01, 1);
	g15_buf_fill(&g->sched_ctx, 0x1e, 0xff, 1);
	g15_buf_fill(&g->sched_ctx, 0x23, 0x02, 1);
	g15_buf_w64(&g->notifier, 0x00, g->threshold.va);
	g15_buf_w32(&g->notifier, 0x10, 0x50);
	g15_buf_fill(&g->notifier, 0xa8, 0xff, 8);
	g15_buf_w64(&g->job_list, 0x08, g->job_list.va);
	g15_buf_w32(&g->threshold, 0, 1);

	g15_buf_w64(&g->qinfo, 0x00, g->qstate.va);
	g15_buf_w64(&g->qinfo, 0x08, g->ring.va);
	g15_buf_w64(&g->qinfo, 0x10, g->job_list.va);
	g15_buf_w64(&g->qinfo, 0x18, g->qscratch.va);
	g15_buf_w32(&g->qinfo, 0x2c, 0xffffffff);
	g15_buf_w64(&g->qinfo, 0x38, 0xffffffffffff0000ULL);
	g15_buf_w32(&g->qinfo, 0x40, 1);
	g15_buf_w32(&g->qinfo, 0x48, 1);
	g15_buf_w32(&g->qinfo, 0x4c, 0xffffffff);
	g15_buf_w32(&g->qinfo, 0x50, G15_UUID);
	g15_buf_w64(&g->qinfo, 0xa4, g->sched_ctx.va);
	g15_buf_w64(&g->qinfo, 0xac, 4);
	g15_buf_w32(&g->qstate, 0x50, G15_QUEUE_CAP);

	g15_buf_w32(&g->user_stamp, 0, G15_STAMP_SEED);
	g15_buf_w32(&g->fw_stamp, 0, 0);
	for (u32 k = 0; k < g->slots; k++) {
		g15_slot_scrub(&g->slot[k]);
		g->slot[k].state = G15_CT_CLEAN;
	}
	wmb();
}

/* ------------------------------------------------------- render graph --- */

static u64 g15_compact(u64 gva)
{
	return gva - 0x1000000000ULL;
}

static int g15_render_alloc(struct g15_backend *be)
{
	struct g15_render *r = &be->r;
	const u64 sh = G15_PTE_SHARED, fw = G15_PTE_FW;
	const u64 cfw = g15_cmd_fw(), csh = g15_cmd_shared();
	int ret = 0;

	for (u32 e = 0; !ret && e < 2; e++) {
		for (u32 k = 0; !ret && k < 8; k++)
			ret = __g15_buf_alloc(be, &r->pools[e][k], G15_POOL_SIZE, sh,
					      G15_UMA_GVA(e, k), 0, false, true);
		ret = ret ?: g15_buf_alloc(be, &r->freelist[e], 0x400000, sh,
					   G15_UMA_FREELIST_GVA(e), 0, false);
		ret = ret ?: g15_buf_alloc(be, &r->pool_table[e], 0x1000, sh,
					   G15_UMA_TABLE_PAGE(e), 0x3000, false);
		ret = ret ?: g15_buf_alloc(be, &r->uma_mgr[e], 0x84, sh, 0, 0, false);
		ret = ret ?: g15_buf_alloc(be, &r->uma_counter[e], 0x40, sh, 0, 0, false);
		ret = ret ?: g15_buf_alloc(be, &r->qinfo[e], 0xb4, cfw, 0, 0, false);
		ret = ret ?: g15_buf_alloc(be, &r->qstate[e], 0x60, fw, 0, 0, false);
		ret = ret ?: g15_buf_alloc(be, &r->ring[e], G15_QUEUE_CAP * 8, fw, 0, 0, false);
		ret = ret ?: g15_buf_alloc(be, &r->qscratch[e], 0x40000, g15_queue_scratch(), 0, 0,
					   false);
		ret = ret ?: g15_buf_alloc(be, &r->user_stamp[e], 4, sh, 0, 0, false);
		ret = ret ?: g15_buf_alloc(be, &r->fw_stamp[e], 4, sh, 0, 0, false);
	}
	ret = ret ?: g15_buf_alloc(be, &r->sched_ctx, 0x40, cfw, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &r->notifier, 0x108, fw, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &r->threshold, 4, fw, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &r->job_list, 0x18, cfw, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &r->block_ctl, 0x40, fw, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &r->pb_counter, 0x40, fw, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &r->mgr_misc, 0x40, fw, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &r->scene_list, 0xc0, sh, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &r->pb_mgr, 0xac, fw, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &r->legacy_empty, 0x40, sh, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &r->initbm, 0x20, cfw, 0, 0, false);
	ret = ret ?: g15_buf_alloc(be, &r->pb_pages, 0x28000, sh, G15_PB_PAGE_LIST_GVA, 0, false);
	ret = ret ?: g15_buf_alloc(be, &r->pb_blocks, 0x8000, sh, G15_PB_BLOCK_LIST_GVA, 0, false);
	ret = ret ?: __g15_buf_alloc(be, &r->pb_heap, G15_PB_HEAP_SIZE, sh, G15_PB_HEAP_GVA, 0,
				     false, true);
	r->slots = clamp(render_slots, 1U, (u32)G15_RENDER_SLOTS);
	/*
	 * Clustered tiling needs three records in each preemption buffer (two
	 * per-cluster plus an aggregate), each buffer then spanning two pages
	 * from the same payload offset (clustering contract 5).
	 */
	r->pre_records = vertex_clustering ? 3 : 2;
	for (u32 i = 0; !ret && i < r->slots; i++) {
		struct g15_pass *p = &r->pass[i];
		u64 ps = i * G15_PASS_STRIDE, ws = i * G15_WORK_STRIDE;
		u64 tpc = i ? G15_TPC_SLOT_SIZE : G15_TPC_SIZE;
		u64 tilemap = i ? G15_TILEMAP_SLOT_SIZE : G15_TILEMAP_SIZE;

		ret = g15_buf_alloc(be, &p->aux, 0x20000, sh, G15_AUX_GVA + i * G15_AUX_STRIDE,
				    0, false);
		ret = ret ?: g15_buf_alloc(be, &p->pre[0], r->pre_records * 0x540, sh,
					   G15_PRE0_PAGE + ps, 0x3580, false);
		ret = ret ?: g15_buf_alloc(be, &p->pre[1], r->pre_records * 0x280, sh,
					   G15_PRE1_PAGE + ps, 0x3b00, false);
		ret = ret ?: g15_buf_alloc(be, &p->pre[2], r->pre_records * 0x40, sh,
					   G15_PRE2_PAGE + ps, 0x3f80, false);
		ret = ret ?: g15_buf_alloc(be, &p->tpc, tpc, sh,
					   G15_TPC_GVA + i * G15_TPC_SLOT_STRIDE, 0, false);
		ret = ret ?: g15_buf_alloc(be, &p->tilemap, tilemap, sh,
					   G15_TILEMAP_GVA + i * G15_TPC_SLOT_STRIDE, 0, false);
		ret = ret ?: g15_buf_alloc(be, &p->heap, 0x1000, sh, G15_HEAP_META_PAGE + ps, 0x3000,
					   false);
		ret = ret ?: g15_buf_alloc(be, &p->scene_user, 0x10000, sh, G15_SCENE_USER_GVA + ps,
					   0, false);
		ret = ret ?: g15_buf_alloc(be, &p->fs_scratch, 0x10000, sh, G15_FS_SCRATCH_GVA + ps,
					   0, false);
		ret = ret ?: g15_buf_alloc(be, &p->fs_work, G15_FS_WORK_SIZE, csh,
					   G15_FS_WORK_PAGE + ws, G15_FS_WORK_OFF, true);
		ret = ret ?: g15_buf_alloc(be, &p->ta_work, G15_TA_WORK_SIZE, csh,
					   G15_TA_WORK_PAGE + ws, G15_TA_WORK_OFF, true);
		ret = ret ?: g15_buf_alloc(be, &p->ta_seq, 0x1000, cfw, 0, 0, false);
		ret = ret ?: g15_buf_alloc(be, &p->fs_seq, 0x1000, cfw, 0, 0, false);
		ret = ret ?: g15_buf_alloc(be, &p->ta_scratch, 0x10000, sh, 0, 0, false);
		ret = ret ?: g15_buf_alloc(be, &p->scene, 0x80, sh, 0, 0, false);
		/* Internal TA start/end, FS start/end; legacy user cells likewise. */
		ret = ret ?: g15_buf_alloc(be, &p->its, 32, sh, 0, 0, false);
		ret = ret ?: g15_buf_alloc(be, &p->uts, 32, fw, 0, 0, false);
		ret = ret ?: g15_buf_alloc(be, &p->own_barrier, G15_BARRIER_SIZE, cfw, 0, 0, false);
		ret = ret ?: g15_buf_alloc(be, &p->prev_barrier, G15_BARRIER_SIZE, cfw, 0, 0, false);
		ret = ret ?: g15_buf_alloc(be, &p->fs_dep, G15_BARRIER_SIZE, cfw, 0, 0, false);
		if (ret)
			break;
		/* Client leaf classes (GPU-only NC is the default). */
		for (u32 k = 0; k < 3; k++)
			p->pre[k].client_attrs = G15_LEAF_CACHED;
		p->tpc.client_attrs = p->tilemap.client_attrs = p->heap.client_attrs = G15_LEAF_CACHED;
		p->scene_user.client_attrs = p->fs_scratch.client_attrs = G15_LEAF_CACHED;
	}
	if (ret)
		return ret;
	r->pb_heap.client_attrs = G15_LEAF_CACHED;
	r->pb_pages.client_attrs = r->pb_blocks.client_attrs = G15_LEAF_PB_LIST;
	return 0;
}

/*
 * Allocator state: UMA pools/freelists/managers and the parameter buffer
 * lists, manager and counters. Initial contents; firmware owns them once
 * published. Rewritten (zeroed first) only after a recovered render abort,
 * followed by a fresh InitBM.
 */
static void g15_render_pools_init(struct g15_backend *be)
{
	struct g15_render *r = &be->r;

	for (u32 e = 0; e < 2; e++) {
		u32 entry = 0;

		g15_buf_fill(&r->uma_mgr[e], 0, 0, 0x84);
		g15_buf_fill(&r->uma_counter[e], 0, 0, 0x40);
		for (u32 k = 0; k < 8; k++)
			for (u64 off = 0; off < G15_POOL_SIZE; off += 0x1000)
				g15_buf_w64(&r->freelist[e], 8ULL * entry++, r->pools[e][k].gva + off);
		for (u32 k = 0; k < 8; k++)
			g15_buf_w64(&r->pool_table[e], 0x40ULL * k, r->pools[e][k].gva | BIT_ULL(62));
		g15_buf_w32(&r->uma_mgr[e], 0x00, e + 1);
		g15_buf_w32(&r->uma_mgr[e], 0x08, e);
		g15_buf_w32(&r->uma_mgr[e], 0x0c, 1);
		g15_buf_w64(&r->uma_mgr[e], 0x14, r->freelist[e].gva);
		g15_buf_w32(&r->uma_mgr[e], 0x1c, 0x80000);
		g15_buf_w32(&r->uma_mgr[e], 0x24, G15_FREELIST_SEED);
		g15_buf_w32(&r->uma_mgr[e], 0x2c, G15_FREELIST_SEED);
		g15_buf_w64(&r->uma_mgr[e], 0x34, r->pool_table[e].gva);
		g15_buf_w32(&r->uma_mgr[e], 0x44, 0x40);
		g15_buf_w64(&r->uma_mgr[e], 0x48, r->uma_counter[e].va);
		g15_buf_w32(&r->uma_counter[e], 0, 1);
	}

	/* PB: 8192 pages of 0x8000 in 2048 blocks of 0x20000 (contract 3). */
	g15_buf_fill(&r->pb_mgr, 0, 0, 0xac);
	g15_buf_fill(&r->block_ctl, 0, 0, 0x40);
	g15_buf_fill(&r->mgr_misc, 0, 0, 0x40);
	g15_buf_fill(&r->pb_counter, 0, 0, 0x40);
	g15_buf_fill(&r->scene_list, 0, 0, 0xc0);
	for (u32 i = 0; i < G15_PB_PAGES; i++)
		g15_buf_w32(&r->pb_pages, 4ULL * i,
			    (u32)(g15_compact(r->pb_heap.gva + i * 0x8000ULL) / 0x8000));
	for (u32 i = 0; i < G15_PB_BLOCKS; i++)
		g15_buf_w32(&r->pb_blocks, 8ULL * i,
			    (u32)(g15_compact(r->pb_heap.gva + i * 0x20000ULL) / 0x8000));
	g15_buf_w64(&r->pb_mgr, 0x1c, r->pb_pages.gva);
	g15_buf_w32(&r->pb_mgr, 0x24, 0x28000);
	g15_buf_w32(&r->pb_mgr, 0x28, G15_PB_PAGES);
	g15_buf_w32(&r->pb_mgr, 0x2c, 0x26cd);
	g15_buf_w32(&r->pb_mgr, 0x30, G15_PB_BLOCKS);
	g15_buf_w64(&r->pb_mgr, 0x38, r->pb_blocks.gva);
	g15_buf_w64(&r->pb_mgr, 0x40, r->block_ctl.va);
	g15_buf_w32(&r->pb_mgr, 0x48, G15_PB_PAGES - 1);
	g15_buf_w32(&r->pb_mgr, 0x4c, 0x20000);
	g15_buf_w64(&r->pb_mgr, 0x58, r->pb_counter.va);
	g15_buf_w32(&r->pb_mgr, 0x70, G15_PB_PAGES);
	g15_buf_w32(&r->pb_mgr, 0x74, G15_PB_PAGES);
	g15_buf_w32(&r->block_ctl, 0, G15_PB_BLOCKS);
	g15_buf_w32(&r->block_ctl, 4, G15_PB_BLOCKS);
	g15_buf_w32(&r->mgr_misc, 0x20, 1);
	g15_buf_w32(&r->pb_counter, 0, 1);
	wmb();
}

static void g15_render_init(struct g15_backend *be)
{
	struct g15_render *r = &be->r;
	static const u32 uuid[2] = { G15_TA_UUID, G15_FS_UUID };
	static const u32 seed[2] = { G15_TA_SEED, G15_FS_SEED };

	for (u32 e = 0; e < 2; e++) {
		g15_buf_w64(&r->qinfo[e], 0x00, r->qstate[e].va);
		g15_buf_w64(&r->qinfo[e], 0x08, r->ring[e].va);
		g15_buf_w64(&r->qinfo[e], 0x10, r->job_list.va);
		g15_buf_w64(&r->qinfo[e], 0x18, r->qscratch[e].va);
		g15_buf_w32(&r->qinfo[e], 0x2c, 0xffffffff);
		g15_buf_w64(&r->qinfo[e], 0x38, 0xffffffffffff0000ULL);
		g15_buf_w32(&r->qinfo[e], 0x40, 1);
		g15_buf_w32(&r->qinfo[e], 0x48, 1);
		g15_buf_w32(&r->qinfo[e], 0x4c, 0xffffffff);
		g15_buf_w32(&r->qinfo[e], 0x50, uuid[e]);
		g15_buf_w64(&r->qinfo[e], 0xa4, r->sched_ctx.va);
		g15_buf_w64(&r->qinfo[e], 0xac, 4);
		g15_buf_w32(&r->qstate[e], 0x50, G15_QUEUE_CAP);
		g15_buf_fill(&r->ring[e], 0, 0xcc, G15_QUEUE_CAP * 8);
		g15_buf_w32(&r->user_stamp[e], 0, seed[e]);
		g15_buf_w32(&r->fw_stamp[e], 0, seed[e]);
	}
	g15_buf_fill(&r->sched_ctx, 0x00, 0xff, 2);
	g15_buf_fill(&r->sched_ctx, 0x05, 0x01, 1);
	g15_buf_fill(&r->sched_ctx, 0x1e, 0xff, 1);
	g15_buf_fill(&r->sched_ctx, 0x23, 0x02, 1);
	g15_buf_w64(&r->notifier, 0x00, r->threshold.va);
	g15_buf_w32(&r->notifier, 0x10, 0x50);
	g15_buf_fill(&r->notifier, 0xa8, 0xff, 8);
	g15_buf_w64(&r->job_list, 0x08, r->job_list.va);
	g15_buf_w32(&r->threshold, 0, 2);
	g15_render_pools_init(be);
}

/* ---------------------------------------------------------- fences --- */

static const char *g15_fence_driver(struct dma_fence *fence)
{
	return "apple-g15";
}

static const char *g15_fence_timeline(struct dma_fence *fence)
{
	return "g15-compute";
}

static const struct dma_fence_ops g15_fence_ops = {
	.get_driver_name = g15_fence_driver,
	.get_timeline_name = g15_fence_timeline,
};

/* ---------------------------------------------------- VMs and objects --- */

struct g15_vm {
	struct kref ref;
	struct g15_backend *be;
	struct gpu_m4_drm_vm *frontend;
	struct gpu_m4_vm_tree *tree;
	struct mutex objects_lock;
	struct list_head objects;
	struct list_head dead;
	u32 asid;
	bool installed;
};

/* One pinned native GEM inside one VM; tree extents hold references. */
struct g15_obj {
	struct kref ref;
	struct list_head link;
	struct g15_vm *vm;
	struct drm_gem_object *gem;
	struct gpu_m4_drm_bo_pin *pin;	/* native shmem */
	struct sg_table *sgt;		/* imported PRIME: our attachment's mapping */
	u64 *pa, size;
};

static void g15_obj_unpin(struct g15_obj *o)
{
	if (o->sgt) {
		dma_buf_unmap_attachment_unlocked(o->gem->import_attach, o->sgt,
						  DMA_BIDIRECTIONAL);
		drm_gem_object_put(o->gem);
	} else if (!IS_ERR_OR_NULL(o->pin)) {
		gpu_m4_drm_bo_pin_put(o->pin);
	}
}

/*
 * PRIME import (e.g. a DCP scanout buffer): map the attachment for this
 * device and take its 16 KiB-aligned page inventory. The mapping and a GEM
 * reference are held while any extent names the object.
 */
static int g15_obj_import(struct g15_obj *o, struct drm_gem_object *gem)
{
	struct scatterlist *sg;
	u64 page = 0, pages = o->size / G15_PAGE;
	u32 i;

	if (!gem->import_attach)
		return -EPERM;
	o->sgt = dma_buf_map_attachment_unlocked(gem->import_attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(o->sgt)) {
		int ret = PTR_ERR(o->sgt);

		o->sgt = NULL;
		return ret;
	}
	drm_gem_object_get(gem);
	o->gem = gem;
	for_each_sgtable_dma_sg(o->sgt, sg, i) {
		dma_addr_t a = sg_dma_address(sg);
		u64 len = sg_dma_len(sg);

		if ((a | len) & (G15_PAGE - 1) || (a & ~G15_PA_MASK))
			return -EINVAL;
		for (u64 off = 0; off < len && page < pages; off += G15_PAGE)
			o->pa[page++] = a + off;
	}
	return page == pages ? 0 : -EINVAL;
}

static void g15_obj_release(struct kref *ref)
{
	struct g15_obj *o = container_of(ref, struct g15_obj, ref);

	mutex_lock(&o->vm->objects_lock);
	list_del(&o->link);
	mutex_unlock(&o->vm->objects_lock);
	g15_obj_unpin(o);
	kvfree(o->pa);
	kfree(o);
}

static void g15_obj_put(struct g15_obj *o)
{
	if (o)
		kref_put(&o->ref, g15_obj_release);
}

static int g15_tree_get(void *cookie, void *object)
{
	kref_get(&((struct g15_obj *)object)->ref);
	return 0;
}

static void g15_tree_put(void *cookie, void *object)
{
	g15_obj_put(object);
}

/* Settled by the retirement path once the last guard drops. */
static void g15_tree_deferred(void *cookie)
{
}

static struct g15_obj *g15_obj_find(struct g15_vm *vm, struct drm_gem_object *gem)
{
	struct g15_obj *o;

	list_for_each_entry(o, &vm->objects, link)
		if (o->gem == gem && kref_get_unless_zero(&o->ref))
			return o;
	return NULL;
}

/* Find or pin gem for this VM; returns a staging reference. */
static struct g15_obj *g15_obj_get(struct g15_vm *vm, struct drm_gem_object *gem)
{
	struct g15_obj *o, *other;
	int ret;

	mutex_lock(&vm->objects_lock);
	o = g15_obj_find(vm, gem);
	mutex_unlock(&vm->objects_lock);
	if (o)
		return o;
	if (!gpu_m4_drm_aligned(gem->size) ||
	    (!drm_gem_is_imported(gem) && !gpu_m4_drm_bo_belongs_to_vm(gem, vm->frontend)))
		return ERR_PTR(-EPERM);
	o = kzalloc(sizeof(*o), GFP_KERNEL);
	if (!o)
		return ERR_PTR(-ENOMEM);
	o->size = gem->size;
	o->pa = kvcalloc(o->size / G15_PAGE, sizeof(*o->pa), GFP_KERNEL);
	if (!o->pa) {
		kfree(o);
		return ERR_PTR(-ENOMEM);
	}
	if (drm_gem_is_imported(gem)) {
		ret = g15_obj_import(o, gem);
	} else {
		ret = dma_resv_lock(gem->resv, NULL);
		if (!ret) {
			o->pin = gpu_m4_drm_bo_pin_locked(gem);
			dma_resv_unlock(gem->resv);
			ret = IS_ERR(o->pin) ? PTR_ERR(o->pin) : 0;
		}
		for (u64 i = 0; !ret && i < o->size / G15_PAGE; i++)
			ret = gpu_m4_drm_bo_pin_page(o->pin, i * G15_PAGE, &o->pa[i]);
	}
	if (ret) {
		g15_obj_unpin(o);
		kvfree(o->pa);
		kfree(o);
		return ERR_PTR(ret);
	}
	kref_init(&o->ref);
	o->vm = vm;
	o->gem = gem;
	mutex_lock(&vm->objects_lock);
	other = g15_obj_find(vm, gem);
	if (!other)
		list_add_tail(&o->link, &vm->objects);
	mutex_unlock(&vm->objects_lock);
	if (other) {
		g15_obj_unpin(o);
		kvfree(o->pa);
		kfree(o);
		return other;
	}
	return o;
}

static void g15_tlbi_asid(u32 asid)
{
	asm volatile(".arch armv8.4-a\n\tdsb ishst\n\ttlbi aside1os, %0\n\tdsb sy\n\tisb"
		     : : "r" ((u64)asid << 48) : "memory");
}

/* Publish this VM's TTBAT pair under the PPL handoff (be->lock held). */
static int g15_vm_install(struct g15_vm *vm)
{
	struct g15_backend *be = vm->be;
	struct g15_gpu *gpu = be->gpu;
	u64 lower = gpu_m4_vm_tree_root(vm->tree) | 1 | (u64)vm->asid << 48;
	u64 upper = gpu->region[G15_TABLES].start | 1 | (u64)vm->asid << 48;
	int ret;

	ret = gpu_m4_vm_tree_install_begin(vm->tree, vm->asid);
	if (ret)
		return ret;
	ret = g15_ppl_lock(gpu);
	if (ret) {
		gpu_m4_vm_tree_evicted(vm->tree);
		return ret;
	}
	writeq(lower, gpu->ttbs + 16 * vm->asid);
	writeq(upper, gpu->ttbs + 16 * vm->asid + 8);
	g15_ppl_unlock(gpu);
	g15_tlbi_asid(vm->asid);
	vm->installed = true;
	return 0;
}

/* Clear only the exact pair this VM still owns; false: retain everything. */
static bool g15_vm_uninstall(struct g15_vm *vm)
{
	struct g15_backend *be = vm->be;
	struct g15_gpu *gpu = be->gpu;
	u64 lower = gpu_m4_vm_tree_root(vm->tree) | 1 | (u64)vm->asid << 48;

	if (!vm->installed)
		return true;
	if (g15_ppl_lock(gpu))
		return false;
	if (readq(gpu->ttbs + 16 * vm->asid) == lower) {
		writeq(0, gpu->ttbs + 16 * vm->asid);
		writeq(0, gpu->ttbs + 16 * vm->asid + 8);
	}
	g15_ppl_unlock(gpu);
	g15_tlbi_asid(vm->asid);
	gpu_m4_vm_tree_evicted(vm->tree);
	vm->installed = false;
	return true;
}

static void g15_vm_free(struct g15_vm *vm)
{
	struct g15_backend *be = vm->be;
	bool retain;

	mutex_lock(&be->lock);
	retain = be->failed || !g15_vm_uninstall(vm);
	mutex_unlock(&be->lock);
	/* A refused destroy (guarded/faulted) or failed GPU keeps the tree. */
	if (retain || gpu_m4_vm_tree_destroy(vm->tree)) {
		dev_warn(be->gpu->dev, "retaining VM context %u\n", vm->asid);
		return;
	}
	mutex_lock(&be->lock);
	clear_bit(vm->asid, be->asids);
	mutex_unlock(&be->lock);
	gpu_m4_drm_vm_put(vm->frontend);
	kfree(vm);
}

static void g15_reap_work(struct work_struct *work)
{
	struct g15_backend *be = container_of(work, struct g15_backend, reap);
	struct g15_vm *vm;

	for (;;) {
		spin_lock(&be->dead_lock);
		vm = list_first_entry_or_null(&be->dead_vms, struct g15_vm, dead);
		if (vm)
			list_del(&vm->dead);
		spin_unlock(&be->dead_lock);
		if (!vm)
			break;
		g15_vm_free(vm);
	}
}

static void g15_vm_release(struct kref *ref)
{
	struct g15_vm *vm = container_of(ref, struct g15_vm, ref);
	struct g15_backend *be = vm->be;

	spin_lock(&be->dead_lock);
	list_add_tail(&vm->dead, &be->dead_vms);
	spin_unlock(&be->dead_lock);
	queue_work(system_unbound_wq, &be->reap);
}

static void g15_vm_put(struct g15_vm *vm)
{
	if (vm)
		kref_put(&vm->ref, g15_vm_release);
}

/* The graph's client objects as DRIVER extents of this VM. */
static int g15_vm_map_graph(struct g15_vm *vm)
{
	struct gpu_m4_vm_tree_op *ops;
	int ret;

	ops = kcalloc(vm->be->client_count, sizeof(*ops), GFP_KERNEL);
	if (!ops)
		return -ENOMEM;
	for (u32 i = 0; i < vm->be->client_count; i++) {
		struct g15_buf *b = vm->be->client[i];

		ops[i] = (struct gpu_m4_vm_tree_op) {
			.va = b->gva - b->off, .size = b->size, .object_size = b->size, .pa = b->pa,
			.access = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE,
			.flags = GPU_M4_VM_TREE_DRIVER, .identity = b,
			.cpu_policy = GPU_M4_DRM_CPU_NC, .attrs = b->client_attrs,
		};
	}
	ret = gpu_m4_vm_tree_bind(vm->tree, ops, vm->be->client_count);
	kfree(ops);
	return ret;
}

/* --------------------------------------------------- timestamp objects --- */

struct g15_ts {
	struct kref ref;
	struct g15_backend *be;
	struct gpu_m4_drm_bo_pin *pin;
	u64 va;
	u32 first, count;
};

static void g15_ts_release(struct kref *ref)
{
	struct g15_ts *t = container_of(ref, struct g15_ts, ref);
	struct g15_backend *be = t->be;
	struct g15_gpu *gpu = be->gpu;

	mutex_lock(&be->lock);
	if (be->failed) {
		/* A failed command may still name this alias: retain it. */
		mutex_unlock(&be->lock);
		return;
	}
	mutex_lock(&gpu->lock);
	for (u32 i = 0; i < t->count; i++)
		g15_unmap_page(gpu, t->va + i * G15_PAGE);
	mutex_unlock(&gpu->lock);
	g15_tlbi_all();
	bitmap_clear(be->ts_pages, t->first, t->count);
	mutex_unlock(&be->lock);
	gpu_m4_drm_bo_pin_put(t->pin);
	kfree(t);
}

static void g15_ts_put(struct g15_ts *t)
{
	if (t)
		kref_put(&t->ref, g15_ts_release);
}

/* ------------------------------------------------------- queues/jobs --- */

struct g15_queue {
	struct kref ref;
	struct g15_backend *be;
	struct g15_vm *vm;
	struct gpu_m4_drm_sched_queue *sched;
	u64 usc_base;
	u32 priority;
	u64 turn;			/* be->turn of its last publication (be->lock) */
	u64 fence_context, seqno;	/* seqno under be->lock */
};

/* Compute: ts 0/1 start/end. Render: 0/1 vertex start/end, 2/3 fragment. */
struct g15_cmd {
	bool render;
	struct drm_asahi_cmd_compute c;
	struct drm_asahi_cmd_render r;
	struct drm_asahi_attachment att[G15_MAX_ATTACHMENTS];
	u32 att_count;
	struct g15_ts *ts[4];
	u64 ts_va[4];
};

struct g15_job {
	struct kref ref;
	struct list_head link;
	struct g15_queue *q;
	struct g15_vm *vm;
	struct g15_cmd *cmds;
	u32 count, next;		/* next: first command not yet retired */
	u32 sent;			/* first command not yet published */
	struct dma_fence *fence;
	bool guard, signaled;
};

static void g15_queue_release(struct kref *ref)
{
	struct g15_queue *q = container_of(ref, struct g15_queue, ref);

	g15_vm_put(q->vm);
	kfree(q);
}

static void g15_queue_put(struct g15_queue *q)
{
	if (q)
		kref_put(&q->ref, g15_queue_release);
}

static void g15_guard_put(struct g15_job *job)
{
	struct gpu_m4_vm_tree *tree = job->vm->tree;

	if (!job->guard)
		return;
	job->guard = false;
	gpu_m4_vm_tree_guard_put(tree);
	/* GEM/VM closes deferred by this guard (no locks held here). */
	if (!gpu_m4_vm_tree_guards(tree) && gpu_m4_vm_tree_deferred_count(tree))
		gpu_m4_vm_tree_settle_deferred(tree);
}

static void g15_job_release(struct kref *ref)
{
	struct g15_job *job = container_of(ref, struct g15_job, ref);

	g15_guard_put(job);
	for (u32 i = 0; i < job->count; i++) {
		for (u32 t = 0; t < 4; t++)
			g15_ts_put(job->cmds[i].ts[t]);
	}
	kvfree(job->cmds);
	dma_fence_put(job->fence);
	g15_vm_put(job->vm);
	g15_queue_put(job->q);
	kfree(job);
}

static void g15_job_put(struct g15_job *job)
{
	if (job)
		kref_put(&job->ref, g15_job_release);
}

/* ------------------------------------------------------------ engine --- */

/*
 * Conservative WB policy for this non-dma-coherent device: every WRITEBACK
 * BO bound in the VM is cleaned and invalidated to PoC before the command is
 * published and again after its retirement (be->lock held).
 */
static void g15_wb_sweep(struct g15_vm *vm)
{
	struct g15_obj *o;

	if (!READ_ONCE(wb_sweep))
		return;
	mutex_lock(&vm->objects_lock);
	list_for_each_entry(o, &vm->objects, link) {
		if (gpu_m4_drm_bo_policy(o->gem) != GPU_M4_DRM_CPU_WRITEBACK)
			continue;
		for (u64 i = 0; i < o->size / G15_PAGE; i++) {
			unsigned long va = (unsigned long)phys_to_virt(o->pa[i]);

			dcache_clean_poc(va, va + G15_PAGE);
			dcache_inval_poc(va, va + G15_PAGE);
		}
	}
	mutex_unlock(&vm->objects_lock);
	dsb(sy);
}

static u32 g15_stamp(u64 n)
{
	u64 seed = G15_STAMP_SEED / 0x100;

	return (((seed - 1 + n % 0xffffff) % 0xffffff) + 1) * 0x100;
}

/* REMOVE-DEBUG: up to 64 bytes of a mapped client range, through the linear map. */
static void g15_dump_client(struct g15_backend *be, struct g15_vm *vm, const char *what,
			    u64 va, u32 len)
{
	struct gpu_m4_drm_vm_leaf leaf = {};
	u8 buf[64];
	unsigned long cpu;
	int ret;

	len = min_t(u32, len, sizeof(buf));
	len = min_t(u64, len, G15_PAGE - (va & (G15_PAGE - 1)));
	ret = gpu_m4_vm_tree_page(vm->tree, va & ~(G15_PAGE - 1), DRM_ASAHI_BIND_READ, &leaf);
	if (ret) {
		dev_info(be->gpu->dev, "%s %#llx: no leaf (%d)\n", what, va, ret);
		return;
	}
	cpu = (unsigned long)phys_to_virt(leaf.pa) + (va & (G15_PAGE - 1));
	dcache_inval_poc(cpu & ~63UL, ALIGN(cpu + len, 64));
	memcpy(buf, (void *)cpu, len);
	dev_info(be->gpu->dev, "%s %#llx (pa %#llx): %*ph\n", what, va, leaf.pa, len, buf);
}

/* Post one 0x18-byte host channel record (contract 6.3, render R8). */
static int g15_channel_post(struct g15_backend *be, u32 ch, u64 queue, u32 kind, u32 head,
			    u8 event, bool new_queue)
{
	struct g15_gpu *gpu = be->gpu;
	u32 producer = be->producer[ch];
	u8 rec[0x18] = {};
	int ret;

	if (g15_r32(gpu, G15_CH_STATE(ch)) != producer ||
	    g15_r32(gpu, G15_CH_STATE(ch) + 0x20) != producer)
		return -EBUSY;
	put_unaligned_le64(queue, rec + 0x8);
	put_unaligned_le32(kind, rec + 0x10);
	put_unaligned_le16(head, rec + 0x14);
	rec[0x16] = event;
	rec[0x17] = new_queue;
	ret = g15_write(gpu, G15_CH_RING(ch) + producer * 0x18ULL, rec, sizeof(rec));
	if (ret)
		return ret;
	wmb();
	producer = (producer + 1) % 256;
	g15_w32(gpu, G15_CH_STATE(ch) + 0x20, producer);
	wmb();
	be->producer[ch] = producer;
	return 0;
}

static void g15_stat_publish(struct g15_backend *be, struct g15_flight *f)
{
	be->pub_ns = ktime_get_ns();
	f->pub_ns = be->pub_ns;
	be->st_prep_ns += be->pub_ns - be->prep_from_ns;
	if (be->idle_from_ns) {
		be->st_gap_ns += be->pub_ns - be->idle_from_ns;
		be->st_gaps++;
		be->idle_from_ns = 0;
	} else if (be->starve_from_ns) {
		be->st_starved_ns += be->pub_ns - be->starve_from_ns;
	}
	be->starve_from_ns = 0;
}

/* Add one stage interval read from internal timestamp cells (start, end). */
static void g15_stat_stage(const struct g15_buf *b, u64 off, u64 *sum, u64 *lo, u64 *hi)
{
	u64 start = (u64)g15_buf_r32(b, off + 4) << 32 | g15_buf_r32(b, off);
	u64 end = (u64)g15_buf_r32(b, off + 12) << 32 | g15_buf_r32(b, off + 8);

	if (!start || end < start)
		return;
	*sum += end - start;
	*lo = min(*lo, start);
	*hi = max(*hi, end);
}

/*
 * A retired prefix: publication to retirement (including any wait behind its
 * predecessor), event times, internal stage times of its members.
 */
static void g15_stat_retire(struct g15_backend *be, struct g15_flight *f)
{
	u64 now = ktime_get_ns(), d = now - f->pub_ns, first = U64_MAX, last = 0;
	u32 kind = f->computes && f->renders ? 2 : !!f->renders;
	u64 lo = U64_MAX, hi = 0;

	for (u32 k = 0; k < f->renders; k++) {
		g15_stat_stage(&be->r.pass[f->pass[k]].its, 0, &be->st_ta_ticks, &lo, &hi);
		g15_stat_stage(&be->r.pass[f->pass[k]].its, 0x10, &be->st_fs_ticks, &lo, &hi);
	}
	for (u32 k = 0; k < f->computes; k++)
		g15_stat_stage(&be->g.slot[f->slot[k]].its, 0, &be->st_cdm_ticks, &lo, &hi);
	if (hi > lo)
		be->st_span_ticks += hi - lo;

	for (u32 b = 0; b < 3; b++) {
		u64 t = READ_ONCE(be->gpu->event_ns[b]);

		if (b == 2 ? !f->computes : !f->renders)
			continue;
		first = min(first, t);
		last = max(last, t);
	}
	if (first > f->pub_ns && last <= now) {
		be->st_first_ns[kind] += first - f->pub_ns;
		be->st_last_ns[kind] += last - f->pub_ns;
	}
	be->st_count[0] += f->computes;
	be->st_count[1] += f->renders;
	if (kind == 2)
		be->st_count[2] += f->count;
	be->st_prefixes[kind]++;
	be->st_busy_ns[kind] += d;
	be->st_max_ns[kind] = max(be->st_max_ns[kind], d);
}

/*
 * Encode compute command cmd as ordinal n into its unpublished slot s: work
 * image and microsequence (contracts 4, 5), slot storage reset.
 */
static int g15_encode_compute(struct g15_backend *be, struct g15_slot *s,
			      const struct g15_cmd *cmd, u32 ctx, u64 n)
{
	struct g15_graph *g = &be->g;
	const struct drm_asahi_cmd_compute *c = &cmd->c;
	u64 W = s->work.va, Q = g->qinfo.va, H = G15_HWDATA, pre = s->preempt.gva;
	u32 T = g15_stamp(n);
	u8 *w, *q;
	static const u32 sel[22] = {
		0x1a510, 0x1a420, 0x1a4d0, 0x1a4d8, 0x1a4e0, 0x1a4e8, 0x1a440, 0x1a458,
		0x12091, 0x101d9, 0x1a089, 0x1a091, 0x1a059, 0x1a061, 0x1a0b9, 0x1a0c1,
		0x101d1, 0xd479, 0x1a0e9, 0x107a1, 0xa599, 0xd411,
	};
	u64 val[22] = {
		pre, c->cdm_ctrl_stream_base, pre + 0x1480, pre + 0x1488, pre + 0x1490,
		pre + 0x1498, 0x154024201ULL, 0x10c08860, 0x1c, 0x1c, 0, 0, 0, 0, 0, 0,
		0, 0, 8, 0xff0000, 0x13200400020ULL, 0x200000001ULL,
	};

	w = kzalloc(G15_WORK_SIZE, GFP_KERNEL);
	q = kzalloc(G15_SEQ_SIZE, GFP_KERNEL);
	if (!w || !q) {
		kfree(w);
		kfree(q);
		return -ENOMEM;
	}

	/* Work image (contract 4). */
	put_unaligned_le32(3, w + 0x0);
	put_unaligned_le64((u32)(n - 1), w + 0x4);
	put_unaligned_le32(ctx, w + 0x10);
	put_unaligned_le64(g->notifier.va, w + 0x14);
	for (u32 r = 0; r < 22; r++) {
		put_unaligned_le32(sel[r], w + 0x20 + 12 * r);
		put_unaligned_le64(val[r], w + 0x24 + 12 * r);
	}
	put_unaligned_le64(s->work.gva + 0x20, w + 0x720);
	put_unaligned_le16(22, w + 0x728);
	put_unaligned_le16(22 * 12, w + 0x72a);
	put_unaligned_le64(s->seq.va, w + 0x760);
	put_unaligned_le32(G15_SEQ_SIZE, w + 0x768);
	put_unaligned_le64(pre, w + 0x794);
	put_unaligned_le64(c->cdm_ctrl_stream_end - 4, w + 0x79c);
	put_unaligned_le64(0x154024201ULL, w + 0x7c4);
	put_unaligned_le32(0xffffffff, w + 0x7e0);
	put_unaligned_le64(g->user_stamp.va, w + 0x7e8);
	put_unaligned_le64(g->fw_stamp.va, w + 0x7f0);
	put_unaligned_le32(T, w + 0x7f8);
	put_unaligned_le32(G15_EVENT_SLOT, w + 0x7fc);
	put_unaligned_le32(1, w + 0x804);
	put_unaligned_le32(G15_UUID, w + 0x808);
	put_unaligned_le32((u32)(n - 1), w + 0x80c);
	put_unaligned_le64(s->its.va, w + 0x818);
	put_unaligned_le64(s->its.va + 8, w + 0x820);
	put_unaligned_le64(cmd->ts_va[0], w + 0x828);
	put_unaligned_le64(cmd->ts_va[1], w + 0x830);
	put_unaligned_le64(g->manager.va, w + 0x83e);
	w[0x846] = 1;
	put_unaligned_le64(0xad0000, w + 0x847);
	put_unaligned_le64(0x1600000, w + 0x84f);
	put_unaligned_le64(s->tail.va, w + 0x857);
	w[0x85f] = 1;

	/* Microsequence (contract 5): start. */
	put_unaligned_le32(0x0b, q + 0x0);
	put_unaligned_le64(W + 0x20, q + 0x14);
	put_unaligned_le64(H + 0x3740, q + 0x1c);
	put_unaligned_le64(Q, q + 0x24);
	put_unaligned_le32(ctx, q + 0x2c);
	put_unaligned_le32(READ_ONCE(compute_fault_mask) ? 1 : 0, q + 0x30);
	put_unaligned_le32(0, q + 0x34);
	put_unaligned_le64(W + 0x76c, q + 0x44);
	put_unaligned_le32(G15_UUID, q + 0x50);
	for (u32 a = 0; a < cmd->att_count; a++) {
		u64 base = cmd->att[a].pointer & ~127ULL;
		u64 end = ALIGN(cmd->att[a].pointer + cmd->att[a].size, 128);

		put_unaligned_le64(base, q + 0x54 + 16 * a);
		put_unaligned_le32((end - base) / 128, q + 0x54 + 16 * a + 8);
		put_unaligned_le16(0x17, q + 0x54 + 16 * a + 0xc);
		put_unaligned_le16(1, q + 0x54 + 16 * a + 0xe);
	}
	put_unaligned_le32(cmd->att_count, q + 0x154);
	put_unaligned_le64(g->manager.va, q + 0x15c);
	put_unaligned_le64(s->scratch[0].va, q + 0x174);
	put_unaligned_le64(s->scratch[1].va, q + 0x184);
	put_unaligned_le64(s->scratch[2].va, q + 0x18c);
	put_unaligned_le64(s->scratch[3].va, q + 0x194);
	put_unaligned_le64(s->scratch[4].va, q + 0x19c);
	put_unaligned_le64(W + 0x880, q + 0x1a4);
	put_unaligned_le64(g->notifier.va + 0xa8, q + 0x1b4);
	/* Start and end timestamp records. */
	for (u32 t = 0; t < 2; t++) {
		u8 *r = q + (t ? 0x1fc : 0x1bc);

		put_unaligned_le32(t ? 3 : 0x80000003, r + 0x0);
		put_unaligned_le64(W + 0x810, r + 0x4);
		put_unaligned_le64(W + 0x818, r + 0xc);
		put_unaligned_le64(W + (t ? 0x820 : 0x818), r + 0x14);
		put_unaligned_le64(Q, r + 0x1c);
		/* Both records name the user pair base when any output is held. */
		put_unaligned_le64(cmd->ts_va[0] || cmd->ts_va[1] ? W + 0x828 : 0, r + 0x24);
		put_unaligned_le64(W + 0x868, r + 0x2c);
		put_unaligned_le32(G15_UUID, r + 0x34);
	}
	put_unaligned_le32(1, q + 0x1f8);
	/* Finalize and END. */
	put_unaligned_le32(0x0c, q + 0x238);
	put_unaligned_le64(H + 0x3740, q + 0x238 + 0x4);
	put_unaligned_le64(Q, q + 0x238 + 0xc);
	put_unaligned_le32(ctx, q + 0x238 + 0x14);
	put_unaligned_le64(W + 0x76c, q + 0x238 + 0x18);
	put_unaligned_le32(G15_UUID, q + 0x238 + 0x24);
	put_unaligned_le64(g->fw_stamp.va, q + 0x238 + 0x28);
	put_unaligned_le32(T, q + 0x238 + 0x30);
	put_unaligned_le64(W + 0x83e, q + 0x238 + 0x58);
	put_unaligned_le32((u32)-0x238, q + 0x238 + 0x60);
	q[0x238 + 0x64] = cmd->att_count ? 1 : 0;
	put_unaligned_le64(W + 0x880, q + 0x238 + 0x65);
	put_unaligned_le64(W + 0x860, q + 0x238 + 0x6d);
	put_unaligned_le32(0x40000002, q + 0x2b4);

	/* Rewrite the slot's images (its runtime storage is already scrubbed). */
	g15_buf_fill(&s->work, 0, 0, G15_WORK_SIZE);
	g15_buf_fill(&s->seq, 0, 0, 0x1000);
	g15_buf_write(&s->work, 0, w, G15_WORK_SIZE);
	g15_buf_write(&s->seq, 0, q, G15_SEQ_SIZE);
	kfree(w);
	kfree(q);
	return 0;
}

/* ------------------------------------------------------- render work --- */

static u32 g15_stamp_seed(u32 seed, u64 n)
{
	u64 base = seed / 0x100;

	return (((base - 1 + n % 0xffffff) % 0xffffff) + 1) * 0x100;
}

/* Geometry vocabulary (render contract 4), accelerator count 2. */
struct g15_geom {
	u32 tx, ty, bx, by, sx, sy, pixels, screen, blocks, tiles;
	u32 region_stride, tpc_stride, isp_blocks, utile_config, layer_mode;
	u32 tilebuffer_control, tile_mode, isp_control;
	u64 E, x_blocks, y_blocks, tilemap_bytes, tpc_bytes, layer_metadata, native_tiles;
};

static void g15_geometry(const struct drm_asahi_cmd_render *u, u64 heap, struct g15_geom *g)
{
	u32 W = u->width_px, H = u->height_px, L = u->layers;
	bool empty = u->flags & DRM_ASAHI_RENDER_PROCESS_EMPTY_TILES;
	u32 log2s = ilog2(u->samples);

	g->tx = DIV_ROUND_UP(W, 32);
	g->ty = DIV_ROUND_UP(H, 32);
	g->bx = round_up(DIV_ROUND_UP(g->tx, 4), 4);
	g->by = round_up(DIV_ROUND_UP(g->ty, 4), 4);
	g->sx = g->bx * (u->utile_width_px == 16 ? 2 : 1);
	g->sy = g->by * (u->utile_height_px == 16 ? 2 : 1);
	g->E = (4ULL * g->sx) * (4ULL * g->sy);
	g->pixels = (W - 1) | (H - 1) << 16;
	g->screen = (g->tx - 1) | (g->ty - 1) << 12;
	g->x_blocks = (u64)g->bx << 10 | (u64)g->bx << 18 | 3ULL * g->bx;
	g->y_blocks = (u64)g->by << 18 | (u64)g->by << 10 | 3ULL * g->by;
	g->blocks = g->bx * g->by;
	g->tiles = g->tx * g->ty;
	g->region_stride = 5 * g->E / 64;
	g->tpc_stride = g->E / 8;
	g->isp_blocks = ((g->sx & 0x3fd) << 16) | (g->sy & 0x3fd);
	g->utile_config = (u->utile_width_px == 32 ? 0x2000 : 0x1000) |
		(u->utile_height_px == 32 ? 0x8000 : 0x4000) | log2s;
	g->tilemap_bytes = round_up(5 * g->E * L, 0x1000);
	g->tpc_bytes = 2 * g->E * 8 * L;
	g->layer_metadata = heap + (L > 1 ? 0x200 : 0);
	g->layer_mode = L == 1 ? 0x8000 : 0xc000 | (empty ? 0x2000 : 0x1000) | (L - 1);
	g->tilebuffer_control = DIV_ROUND_UP(u->sample_size_B * u->utile_width_px *
					     u->utile_height_px * u->samples, 2048) |
		log2s << 17;
	g->tile_mode = 0x280 | (L > 1) | (empty ? 1 << 16 : 0);
	g->isp_control = 0xc000 | (u->flags & BIT(18));
	g->native_tiles = 221201ULL | (u64)(L > 1) << 32 | (u64)(g->tx - 1) << 44 |
		(u64)(g->ty - 1) << 53 | 137438953472ULL |
		(u64)(g->utile_config & 0xf000) << 28;
}

/*
 * Clustered tiling layout from base (clustering contract 2): 64 segments of
 * 5E bytes, then M2 (0xc00), M3 (0xa00), M4 ((Q+1) x 512 for Q = pages of
 * the owned PB / 0x1800, rounded up) and the 128-aligned 0x80-byte M1.
 */
struct g15_cluster {
	u64 ct, m1, m2, m3, m4, bytes;
};

static void g15_cluster_layout(const struct g15_geom *geo, u64 base, struct g15_cluster *c)
{
	u64 q = DIV_ROUND_UP(DIV_ROUND_UP(G15_PB_HEAP_SIZE, 0x8000), 0x1800);

	c->ct = base;
	c->m2 = base + ALIGN(64 * 5 * geo->E, 32);
	c->m3 = c->m2 + 0xc00;
	c->m4 = c->m3 + 0xa00;
	c->m1 = base + ALIGN(c->m4 - base + (q + 1) * 8 * 64, 128);
	c->bytes = c->m1 + 0x80 - base;
}

/* Program address and resource word (render addendum R6). */
static u64 g15_prog_addr(u64 usc_base, const struct drm_asahi_bg_eot *p)
{
	return usc_base + (p->usc & ~63U);
}

static u64 g15_prog_rsrc(const struct drm_asahi_cmd_render *u, const struct drm_asahi_bg_eot *p,
			 u32 hi)
{
	return u->flags & DRM_ASAHI_RENDER_RSRC_SPEC_HI ? (u64)hi << 32 | p->rsrc_spec :
		p->rsrc_spec;
}

static void g15_regs(u8 *dst, const u32 *sel, const u64 *val, u32 count)
{
	for (u32 i = 0; i < count; i++) {
		put_unaligned_le32(sel[i], dst + 12 * i);
		put_unaligned_le64(val[i], dst + 12 * i + 4);
	}
}

/* 0x3c timestamp record (render contract 7). */
static void g15_ts_record(u8 *rec, bool end, u64 W, u32 B, u64 Q, bool requested, u32 uuid)
{
	put_unaligned_le32(end ? 3 : 0x80000003, rec + 0x0);
	put_unaligned_le64(W + B, rec + 0x4);
	put_unaligned_le64(W + B + 8, rec + 0xc);
	put_unaligned_le64(W + B + (end ? 0x10 : 8), rec + 0x14);
	put_unaligned_le64(Q, rec + 0x1c);
	put_unaligned_le64(requested ? W + B + 0x18 : 0, rec + 0x24);
	put_unaligned_le64(W + B + 0x58, rec + 0x2c);
	put_unaligned_le32(uuid, rec + 0x34);
}

/* Shared work tail (TA +0x8a8, FS +0xbf0); mirrors the compute tail. */
static void g15_work_tail(u8 *t, u64 its_start, u64 its_end, u64 user_start, u64 user_end,
			  u64 uma_mgr, u64 scratch)
{
	put_unaligned_le64(its_start, t + 0x8);
	put_unaligned_le64(its_end, t + 0x10);
	put_unaligned_le64(user_start, t + 0x18);
	put_unaligned_le64(user_end, t + 0x20);
	put_unaligned_le64(uma_mgr, t + 0x2e);
	t[0x36] = 1;
	put_unaligned_le64(0xad0000, t + 0x37);
	put_unaligned_le64(0x1600000, t + 0x3f);
	put_unaligned_le64(scratch, t + 0x47);
	t[0x4f] = 1;
}

static void g15_barrier(const struct g15_buf *b, u64 stamp, u32 wait, u32 event, u32 own,
			u32 uuid, u32 type)
{
	u8 x[G15_BARRIER_SIZE] = {};

	put_unaligned_le32(4, x + 0x0);
	put_unaligned_le64(stamp, x + 0x4);
	put_unaligned_le64(stamp, x + 0xc);
	put_unaligned_le32(wait, x + 0x14);
	put_unaligned_le32(event, x + 0x18);
	put_unaligned_le32(own, x + 0x1c);
	put_unaligned_le32(uuid, x + 0x20);
	put_unaligned_le32(type, x + 0x2a);
	g15_buf_write(b, 0, x, sizeof(x));
}

static void g15_encode_ta(struct g15_backend *be, struct g15_pass *p, const struct g15_cmd *cmd,
			  u32 ctx, u64 usc, u32 Tta, const struct g15_geom *geo,
			  const struct g15_cluster *cl, u8 *w, u8 *q)
{
	struct g15_render *r = &be->r;
	const struct drm_asahi_cmd_render *u = &cmd->r;
	u64 W = p->ta_work.va, U = p->ta_seq.va, Q = r->qinfo[0].va, H = G15_HWDATA;
	u64 heap = p->heap.gva, su = p->scene_user.gva;
	bool ts = cmd->ts_va[0] || cmd->ts_va[1];
	static const u32 sel[72] = {
		0x1748, 0x10141, 0x1c039, 0x1c9c8, 0x1c041, 0x1c9d0, 0x1c0a1, 0x1c031,
		0x1c9c0, 0x1c051, 0x1c061, 0x10149, 0x10139, 0x10111, 0x1c9b0, 0x10119,
		0x1c9b8, 0x1c958, 0x1c950, 0x1c930, 0x1c880, 0x1c898, 0x1c948, 0x1c888,
		0x1c890, 0x1c918, 0x1c079, 0x1c9d8, 0x1c089, 0x1c9e0, 0x16c41, 0x1ca40,
		0x1c9a8, 0x1c920, 0x10151, 0x1c199, 0x1c1a1, 0x1c1a9, 0x1c1b1, 0x1c1b9,
		0x1c8f8, 0x1c0b1, 0x1c850, 0x10131, 0x10121, 0x10129, 0x101b9, 0x1c069,
		0x1c071, 0x1c081, 0x1c0a9, 0x10171, 0x10169, 0x12099, 0x101e1, 0x1c9e8,
		0x1a099, 0x1a0a1, 0x1a069, 0x1a071, 0x1a0c9, 0x1a0d1, 0x101c9, 0xd471,
		0x1a0f1, 0x10799, 0xa5a1, 0xd419, 0x1c830, 0x1ca30, 0x16c39, 0x1c910,
	};
	u64 val[72] = {
		1, 0x200, g15_compact(p->tilemap.gva), g15_compact(p->tilemap.gva), 0, 0,
		g15_compact(p->tpc.gva), g15_compact(heap) | BIT_ULL(63),
		g15_compact(heap) | BIT_ULL(63), 0x003a0012006b0003ULL, 1, geo->utile_config,
		u->ppp_multisamplectl, g15_compact(p->pre[0].gva), g15_compact(p->pre[0].gva),
		g15_compact(p->pre[1].gva),
		g15_compact(p->pre[1].gva), 1, g15_compact(p->pre[2].gva) | BIT_ULL(50), 0,
		g15_compact(u->vdm_ctrl_stream_base), 0, 0, 0,
		1, 0, g15_compact(geo->layer_metadata), g15_compact(geo->layer_metadata), 0, 0,
		0, 0,
		G15_PB_PAGES, 0, 0, 0, 0, 0, 0, 0,
		0x8860, geo->region_stride, geo->region_stride, 0x88, 2 | (u->ppp_ctrl & 0x201),
		geo->pixels, geo->screen, geo->x_blocks,
		geo->y_blocks, geo->blocks, geo->tpc_stride, 0x100, geo->layer_mode, 0x1c, 0x1c, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		8, 0xff0000, 0x7f00400020ULL, 0x200000001ULL, 0, g15_compact(su + 0x40),
		g15_compact(su + 0x40),
		0xa000000000ULL | ((r->scene_list.va + 4) & 0xfffffffffULL) | 1,
	};

	(void)usc;
	if (cl) {
		/* The ten clustered TA values (clustering contract 3), compact form. */
		val[4] = val[5] = g15_compact(cl->ct);
		val[22] = g15_compact(cl->m2);
		val[23] = g15_compact(cl->m3);
		val[24] = 0x200440;
		val[25] = 4;
		val[30] = val[31] = g15_compact(cl->m4);
		val[32] = G15_CLUSTER_PB_PAGES;
		val[33] = g15_compact(cl->m1);
	}
	/* Work image (render contract 5). */
	put_unaligned_le32(0, w + 0x0);
	put_unaligned_le64(1, w + 0x4);
	put_unaligned_le32(ctx, w + 0xc);
	put_unaligned_le64(r->notifier.va, w + 0x14);
	put_unaligned_le64(r->pb_mgr.va, w + 0x24);
	put_unaligned_le64(p->scene.va, w + 0x2c);
	put_unaligned_le64(r->legacy_empty.va, w + 0x34);
	g15_regs(w + 0x40, sel, val, 72);
	put_unaligned_le64(p->ta_work.gva + 0x40, w + 0x740);
	put_unaligned_le16(72, w + 0x748);
	put_unaligned_le16(72 * 12, w + 0x74a);
	put_unaligned_le64(p->tpc.gva, w + 0x760);
	put_unaligned_le64(geo->tpc_bytes, w + 0x768);
	put_unaligned_le64(U, w + 0x770);
	put_unaligned_le32(G15_TA_SEQ_SIZE, w + 0x778);
	put_unaligned_le32(1, w + 0x77c);
	put_unaligned_le32(Tta, w + 0x780);
	put_unaligned_le64(g15_compact(p->pre[0].gva), w + 0x7a8);
	put_unaligned_le32(0x67, w + 0x840);
	put_unaligned_le32(0xffffffff, w + 0x848);
	put_unaligned_le32(1, w + 0x854);
	put_unaligned_le64(r->user_stamp[0].va, w + 0x864);
	put_unaligned_le64(r->fw_stamp[0].va, w + 0x86c);
	put_unaligned_le32(Tta, w + 0x874);
	put_unaligned_le32(1, w + 0x880);
	put_unaligned_le32(G15_TA_UUID, w + 0x884);
	g15_work_tail(w + 0x8a8, p->its.va, p->its.va + 8, cmd->ts_va[0], cmd->ts_va[1],
		      r->uma_mgr[0].va, p->ta_scratch.va);

	/* TA microsequence (render contract 7). */
	put_unaligned_le32(5, q + 0x0);
	put_unaligned_le64(W + 0x40, q + 0x14);
	put_unaligned_le64(r->pb_mgr.va, q + 0x1c);
	put_unaligned_le64(p->scene.va, q + 0x24);
	put_unaligned_le64(H + 0x1884, q + 0x2c);
	put_unaligned_le64(Q, q + 0x34);
	put_unaligned_le64(W + 0x84c, q + 0x3c);
	put_unaligned_le32(ctx, q + 0x44);
	put_unaligned_le64(W + 0x784, q + 0x64);
	put_unaligned_le64(W + 0x890, q + 0x6c);
	put_unaligned_le64((u64)G15_TA_UUID << 32, q + 0x78);
	put_unaligned_le64(r->uma_mgr[0].va, q + 0x18c);
	put_unaligned_le64(p->ta_scratch.va, q + 0x1a4);
	put_unaligned_le64(1, q + 0x1b4);
	put_unaligned_le64(r->notifier.va + 0xa8, q + 0x1bc);
	g15_ts_record(q + 0x1cc, false, W, 0x8a8, Q, ts, G15_TA_UUID);
	put_unaligned_le32(1, q + 0x208);
	g15_ts_record(q + 0x20c, true, W, 0x8a8, Q, ts, G15_TA_UUID);
	put_unaligned_le32(6, q + 0x248);
	put_unaligned_le64(p->scene.va, q + 0x248 + 0x4);
	put_unaligned_le64(r->pb_mgr.va, q + 0x248 + 0xc);
	put_unaligned_le64(H + 0x1884, q + 0x248 + 0x14);
	put_unaligned_le64(Q, q + 0x248 + 0x1c);
	put_unaligned_le64(W + 0x84c, q + 0x248 + 0x24);
	put_unaligned_le32(ctx, q + 0x248 + 0x2c);
	put_unaligned_le64(W + 0x784, q + 0x248 + 0x34);
	put_unaligned_le32(G15_TA_UUID, q + 0x248 + 0x40);
	put_unaligned_le64(r->fw_stamp[0].va, q + 0x248 + 0x44);
	put_unaligned_le32(Tta, q + 0x248 + 0x4c);
	put_unaligned_le64(U + 0x18c, q + 0x248 + 0x74);
	put_unaligned_le32((u32)-0x248, q + 0x248 + 0x7c);
	put_unaligned_le64(W + 0x8f8, q + 0x248 + 0x84);
	put_unaligned_le32(0x40000002, q + 0x2d4);
}

static void g15_encode_fs(struct g15_backend *be, struct g15_pass *p, const struct g15_cmd *cmd,
			  u32 ctx, u64 usc, u64 n, u32 Tfs, const struct g15_geom *geo, u8 *w, u8 *q)
{
	struct g15_render *r = &be->r;
	const struct drm_asahi_cmd_render *u = &cmd->r;
	u64 W = p->fs_work.va, U = p->fs_seq.va, Q = r->qinfo[1].va, H = G15_HWDATA;
	u64 heap = p->heap.gva, su = p->scene_user.gva;
	bool ts = cmd->ts_va[2] || cmd->ts_va[3];
	u8 *P = w + 0x790;
	static const u32 sel[79] = {
		0x1739, 0x10009, 0x15379, 0x15381, 0x15369, 0x15371, 0x15131, 0x15139,
		0x100a1, 0x15069, 0x15071, 0x16058, 0x10019, 0x100b1, 0x16030, 0x100d9,
		0x10791, 0x16098, 0x15109, 0x15101, 0x15021, 0x15211, 0x15049, 0x10051,
		0x15321, 0x15301, 0x15309, 0x15311, 0x15319, 0x15349, 0x15351, 0x15329,
		0x15331, 0x15339, 0x15341, 0x15231, 0x15221, 0x15239, 0x15229, 0x15401,
		0x15421, 0x15409, 0x15429, 0x153c1, 0x15411, 0x153c9, 0x15431, 0x153d1,
		0x15419, 0x153d9, 0x15439, 0x16429, 0x16060, 0x16431, 0x10039, 0x16020,
		0x16451, 0x15359, 0x100b8, 0x16461, 0x16090, 0x120a1, 0x101e9, 0x160a8,
		0x16068, 0x160b8, 0x1a0a9, 0x1a0b1, 0x1a079, 0x1a081, 0x1a0d9, 0x1a0e1,
		0x101c1, 0xd469, 0x1a0f9, 0x1c838, 0x1ca28, 0xa5a9, 0xd429,
	};
	u64 val[79] = {
		1, geo->utile_config, g15_prog_rsrc(u, &u->eot, u->bg_eot_rsrc_spec_hi),
		g15_prog_addr(usc, &u->eot), g15_prog_rsrc(u, &u->bg, u->bg_rsrc_spec_hi),
		g15_prog_addr(usc, &u->bg), u->isp_merge_upper_x, u->isp_merge_upper_y,
		0, 0, 0, 0, u->ppp_multisamplectl, geo->isp_blocks, geo->isp_blocks, geo->screen,
		0xff0200, heap, u->isp_scissor_base, u->isp_dbias_base, geo->isp_control,
		(u64)u->height_px << 32 | u->width_px, 0x100000, geo->tilebuffer_control,
		u->isp_zls_pixels, u->isp_bgobjdepth, u->isp_bgobjvals, u->isp_oclqry_base & ~15ULL,
		u->zls_ctrl, 0x04040404, 0, u->depth.base,
		u->depth.base, u->stencil.base, u->stencil.base, 0, 0, 0, 0, u->depth.stride,
		u->depth.stride, u->stencil.stride, u->stencil.stride, u->depth.comp_base,
		u->depth.comp_stride, u->depth.comp_base, u->depth.comp_stride,
		u->stencil.comp_base,
		u->stencil.comp_stride, u->stencil.comp_base, u->stencil.comp_stride,
		p->tilemap.gva, geo->layer_metadata, (4ULL * geo->region_stride) << 24,
		geo->tile_mode, 0,
		0, 0, 0x8860, p->aux.gva, p->aux.gva, 0x1c, 0x1c, 0,
		geo->native_tiles, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 8, 0, g15_compact(su + 0x40) & ~15ULL, 0x6000400020ULL, 0x200000001ULL,
	};
	u64 pbg_r = g15_prog_rsrc(u, &u->partial_bg, u->bg_partial_rsrc_spec_hi);
	u64 pbg_a = g15_prog_addr(usc, &u->partial_bg);
	u64 peot_a = g15_prog_addr(usc, &u->partial_eot);

	/* Work image (render contract 6). */
	put_unaligned_le32(1, w + 0x0);
	put_unaligned_le64(0, w + 0x4);
	put_unaligned_le32(ctx, w + 0xc);
	put_unaligned_le64(U, w + 0x14);
	put_unaligned_le32(G15_FS_SEQ_SIZE, w + 0x1c);
	put_unaligned_le64(r->notifier.va, w + 0x20);
	put_unaligned_le64(r->pb_mgr.va, w + 0x28);
	put_unaligned_le64(p->scene.va, w + 0x30);
	put_unaligned_le64(r->legacy_empty.va, w + 0x38);
	put_unaligned_le64(p->tilemap.gva, w + 0x40);
	put_unaligned_le64(0x88, w + 0x48);
	put_unaligned_le32(1, w + 0x50);
	put_unaligned_le16(geo->y_blocks >> 18, w + 0x54);
	put_unaligned_le16(geo->x_blocks >> 18, w + 0x56);
	put_unaligned_le32(u->isp_merge_upper_x, w + 0x68);
	put_unaligned_le32(u->isp_merge_upper_y, w + 0x6c);
	put_unaligned_le64(geo->tiles, w + 0x78);
	g15_regs(w + 0x80, sel, val, 79);
	put_unaligned_le64(p->fs_work.gva + 0x80, w + 0x780);
	put_unaligned_le16(79, w + 0x788);
	put_unaligned_le16(79 * 12, w + 0x78a);
	/* Parameters / firmware mirrors at +0x790. */
	put_unaligned_le64(u->isp_dbias_base, P + 0x0);
	put_unaligned_le64(u->isp_scissor_base, P + 0x10);
	put_unaligned_le64(u->isp_oclqry_base, P + 0x20);
	put_unaligned_le64(pbg_r, P + 0x168);
	put_unaligned_le64(pbg_a, P + 0x170);
	put_unaligned_le64(pbg_r, P + 0x198);
	put_unaligned_le64(pbg_a, P + 0x1a0);
	put_unaligned_le64(u->zls_ctrl, P + 0x1a8);
	put_unaligned_le32(0x04040404, P + 0x1b0);
	put_unaligned_le64(u->depth.base, P + 0x1b8);
	put_unaligned_le64(u->depth.stride, P + 0x1c0);
	put_unaligned_le64(u->depth.comp_stride, P + 0x1c8);
	put_unaligned_le64(u->depth.base, P + 0x1d0);
	put_unaligned_le64(u->depth.base, P + 0x1d8);
	put_unaligned_le64(u->depth.comp_base, P + 0x1e0);
	put_unaligned_le64(u->stencil.base, P + 0x1e8);
	put_unaligned_le64(u->stencil.stride, P + 0x1f0);
	put_unaligned_le64(u->stencil.comp_stride, P + 0x1f8);
	put_unaligned_le64(u->stencil.base, P + 0x200);
	put_unaligned_le64(u->stencil.base, P + 0x208);
	put_unaligned_le64(u->stencil.comp_base, P + 0x210);
	put_unaligned_le32(geo->tilebuffer_control, P + 0x228);
	put_unaligned_le32(geo->isp_control, P + 0x230);
	put_unaligned_le32(u->width_px, P + 0x238);
	put_unaligned_le32(u->height_px, P + 0x23c);
	put_unaligned_le32(0x100000, P + 0x240);
	put_unaligned_le32(geo->tile_mode, P + 0x248);
	put_unaligned_le32(u->partial_eot.rsrc_spec, P + 0x264);
	put_unaligned_le64(peot_a, P + 0x26c);
	put_unaligned_le32(u->partial_eot.rsrc_spec, P + 0x284);
	put_unaligned_le64(peot_a, P + 0x28c);
	put_unaligned_le32(u->isp_bgobjdepth, P + 0x298);
	put_unaligned_le32(u->isp_bgobjvals, P + 0x29c);
	put_unaligned_le32(u->sample_size_B, P + 0x2a0);
	put_unaligned_le64(u->isp_zls_pixels, P + 0x2c0);
	put_unaligned_le32(0x67, w + 0xb88);
	put_unaligned_le32(0xffffffff, w + 0xb90);
	put_unaligned_le32(!!(u->flags & DRM_ASAHI_RENDER_PROCESS_EMPTY_TILES), w + 0xb94);
	put_unaligned_le32(1, w + 0xb98);
	put_unaligned_le64(r->user_stamp[1].va, w + 0xbac);
	put_unaligned_le64(r->fw_stamp[1].va, w + 0xbb4);
	put_unaligned_le32(Tfs, w + 0xbbc);
	put_unaligned_le32(1, w + 0xbc0);
	put_unaligned_le32(1, w + 0xbc8);
	put_unaligned_le32(G15_FS_UUID, w + 0xbcc);
	put_unaligned_le32(1, w + 0xbe8);
	g15_work_tail(w + 0xbf0, p->its.va + 0x10, p->its.va + 0x18, cmd->ts_va[2],
		      cmd->ts_va[3], r->uma_mgr[1].va, p->fs_scratch.gva);

	/* FS microsequence (render contract 7). */
	put_unaligned_le32(7, q + 0x0);
	put_unaligned_le64(W + 0x80, q + 0x14);
	put_unaligned_le64(p->scene.va, q + 0x1c);
	put_unaligned_le64(H + 0x24c8, q + 0x24);
	put_unaligned_le64(W + 0xb70, q + 0x2c);
	put_unaligned_le64(W + 0xb74, q + 0x34);
	put_unaligned_le64(W + 0xba0, q + 0x3c);
	put_unaligned_le64(Q, q + 0x44);
	put_unaligned_le64(W, q + 0x4c);
	put_unaligned_le32(ctx, q + 0x54);
	put_unaligned_le32(1, q + 0x58);
	put_unaligned_le64((u32)(n - 1), q + 0x68);
	put_unaligned_le64(W + 0xa58, q + 0x74);
	put_unaligned_le64(W + 0xbd8, q + 0x7c);
	put_unaligned_le32(G15_FS_UUID, q + 0x98);
	put_unaligned_le64(r->uma_mgr[1].va, q + 0x1a4);
	put_unaligned_le64(p->fs_scratch.gva, q + 0x1bc);
	put_unaligned_le64(r->notifier.va + 0xa8, q + 0x1d4);
	g15_ts_record(q + 0x1dc, false, W, 0xbf0, Q, ts, G15_FS_UUID);
	put_unaligned_le32(1, q + 0x218);
	g15_ts_record(q + 0x21c, true, W, 0xbf0, Q, ts, G15_FS_UUID);
	put_unaligned_le32(8, q + 0x258);
	put_unaligned_le32(G15_FS_UUID, q + 0x258 + 0x4);
	put_unaligned_le64(r->fw_stamp[1].va, q + 0x258 + 0xc);
	put_unaligned_le32(Tfs, q + 0x258 + 0x14);
	put_unaligned_le64(p->scene.va, q + 0x258 + 0x1c);
	put_unaligned_le64(r->pb_mgr.va, q + 0x258 + 0x24);
	put_unaligned_le64(1, q + 0x258 + 0x2c);
	put_unaligned_le64(H + 0x24c8, q + 0x258 + 0x34);
	put_unaligned_le64(W + 0xba0, q + 0x258 + 0x3c);
	put_unaligned_le64(W + 0xb70, q + 0x258 + 0x44);
	put_unaligned_le64(Q, q + 0x258 + 0x4c);
	put_unaligned_le64(W, q + 0x258 + 0x54);
	put_unaligned_le64(ctx, q + 0x258 + 0x5c);
	put_unaligned_le64(W + 0xa58, q + 0x258 + 0x64);
	put_unaligned_le64(U + 0x1a4, q + 0x258 + 0x9c);
	put_unaligned_le32((u32)-0x258, q + 0x258 + 0xa4);
	put_unaligned_le32(1, q + 0x258 + 0xa8);
	put_unaligned_le64(W + 0xc40, q + 0x258 + 0xac);
	put_unaligned_le32(0x40000002, q + 0x30c);
}

/*
 * Reset retired storage outside be->lock: compute slots' runtime storage and
 * clustering buffers (their written span).
 */
static void g15_ct_clear_work(struct work_struct *work)
{
	struct g15_backend *be = container_of(work, struct g15_backend, ct_clear);
	struct g15_graph *g = &be->g;
	struct g15_render *r = &be->r;

	for (;;) {
		struct g15_slot *s = NULL;

		spin_lock(&be->ct_lock);
		for (u32 k = 0; !s && k < g->slots; k++)
			if (g->slot[k].state == G15_CT_DIRTY)
				s = &g->slot[k];
		if (s)
			s->state = G15_CT_CLEARING;
		spin_unlock(&be->ct_lock);
		if (!s)
			break;
		g15_slot_scrub(s);
		wmb();	/* scrubbed before the slot can be claimed */
		spin_lock(&be->ct_lock);
		s->state = G15_CT_CLEAN;
		spin_unlock(&be->ct_lock);
	}
	for (;;) {
		struct g15_ct *c = NULL;

		spin_lock(&be->ct_lock);
		for (u32 k = 0; !c && k < r->ct_count; k++)
			if (r->ct[k].state == G15_CT_DIRTY)
				c = &r->ct[k];
		if (c)
			c->state = G15_CT_CLEARING;
		spin_unlock(&be->ct_lock);
		if (!c)
			return;
		g15_buf_fill(&c->b, 0, 0, c->dirty);
		wmb();	/* zeroes complete before the buffer can be claimed */
		spin_lock(&be->ct_lock);
		c->dirty = 0;
		c->state = G15_CT_CLEAN;
		spin_unlock(&be->ct_lock);
	}
}

/*
 * A zeroed clustering buffer of at least bytes for a pass of the prefix
 * being prepared (be->lock held), or NULL: none provisioned or too small.
 */
static struct g15_ct *g15_ct_claim(struct g15_backend *be, struct g15_flight *f, u64 bytes)
{
	struct g15_render *r = &be->r;
	struct g15_ct *c = NULL;
	u64 clear = 0;

	if (!r->ct_count || bytes > r->ct[0].b.size - r->ct[0].b.off)
		return NULL;
	for (u32 attempt = 0; !c && attempt < 2; attempt++) {
		spin_lock(&be->ct_lock);
		for (u32 k = 0; !c && k < r->ct_count; k++)
			if (r->ct[k].state == G15_CT_CLEAN)
				c = &r->ct[k];
		/* None clean: take a retired one and zero it here. */
		for (u32 k = 0; !c && k < r->ct_count; k++)
			if (r->ct[k].state == G15_CT_DIRTY)
				c = &r->ct[k];
		if (c) {
			clear = c->state == G15_CT_DIRTY ? c->dirty : 0;
			c->state = G15_CT_USED;
		}
		spin_unlock(&be->ct_lock);
		if (!c)
			flush_work(&be->ct_clear);	/* the rest are being cleared */
	}
	if (!c)
		return NULL;
	if (clear) {
		g15_buf_fill(&c->b, 0, 0, clear);
		be->st_ct_sync++;
	}
	c->dirty = bytes;
	f->ct[f->ct_count++] = c;
	return c;
}

/*
 * A scrubbed compute slot for compute member j of the prefix f being prepared
 * (be->lock held): the first clean slot from the cursor, which follows the
 * previous prefix's slots, so they can be scrubbed while this one runs; with
 * none clean, a retired slot is scrubbed here. Slots of published prefixes
 * are never taken. Without rotation member j takes slot j when it is free.
 */
static struct g15_slot *g15_slot_claim(struct g15_backend *be, struct g15_flight *f, u32 j,
				       u32 *index)
{
	struct g15_graph *g = &be->g;
	struct g15_slot *s = NULL;
	bool scrub = false;
	u32 k = 0;

	if (!j)
		f->rotating = READ_ONCE(slot_rotate);
	if (!f->rotating && j < g->slots) {
		spin_lock(&be->ct_lock);
		while (g->slot[j].state == G15_CT_CLEARING) {
			spin_unlock(&be->ct_lock);
			flush_work(&be->ct_clear);
			spin_lock(&be->ct_lock);
		}
		if (g->slot[j].state != G15_CT_USED) {
			k = j;
			s = &g->slot[k];
			scrub = s->state != G15_CT_CLEAN;
			s->state = G15_CT_USED;
		}
		spin_unlock(&be->ct_lock);
	}
	for (u32 attempt = 0; !s && attempt < 2; attempt++) {
		bool clearing = false;

		spin_lock(&be->ct_lock);
		for (u32 i = 0; !s && i < g->slots; i++) {
			k = (g->cursor + i) % g->slots;
			if (g->slot[k].state == G15_CT_CLEAN)
				s = &g->slot[k];
		}
		for (u32 i = 0; !s && i < g->slots; i++) {
			k = (g->cursor + i) % g->slots;
			if (g->slot[k].state == G15_CT_DIRTY || g->slot[k].state == G15_CT_STALE)
				s = &g->slot[k];
			clearing |= g->slot[k].state == G15_CT_CLEARING;
		}
		if (s) {
			scrub = s->state != G15_CT_CLEAN;
			s->state = G15_CT_USED;
		}
		spin_unlock(&be->ct_lock);
		if (!s && !clearing)
			break;		/* all held by published prefixes */
		if (!s)
			flush_work(&be->ct_clear);	/* the rest are being scrubbed */
	}
	if (!s)
		return NULL;
	if (scrub) {
		g15_slot_scrub(s);
		be->st_slot_sync++;
	}
	g->cursor = (k + 1) % g->slots;
	f->slot[f->slot_count++] = k;
	*index = k;
	return s;
}

/* Compute slots a new prefix can still claim: not held by a published one. */
static u32 g15_slots_free(struct g15_backend *be)
{
	u32 n = 0;

	spin_lock(&be->ct_lock);
	for (u32 k = 0; k < be->g.slots; k++)
		n += be->g.slot[k].state != G15_CT_USED;
	spin_unlock(&be->ct_lock);
	return n;
}

/*
 * A pass slot for a render of the given geometry, not held and not already
 * picked: the next small slot after the previous prefix's, else slot 0
 * (which holds any admitted geometry); -1 if none is free.
 */
static int g15_pass_pick(struct g15_render *r, const struct g15_geom *geo, u32 picked)
{
	u32 busy = r->pass_busy | picked;

	for (u32 j = 0; r->slots > 1 && j < r->slots - 1; j++) {
		u32 k = 1 + (r->pass_cursor + j) % (r->slots - 1);

		if (!(busy & BIT(k)) && geo->tpc_bytes <= r->pass[k].tpc.size &&
		    geo->tilemap_bytes <= r->pass[k].tilemap.size)
			return k;
	}
	if (!(busy & BIT(0)) && geo->tpc_bytes <= r->pass[0].tpc.size &&
	    geo->tilemap_bytes <= r->pass[0].tilemap.size)
		return 0;
	return -1;
}

/*
 * The storage of a retired (or never published) prefix: compute slots and
 * clustering buffers are reset in the background while later prefixes run;
 * its pass slots become free again.
 */
static void g15_flight_release(struct g15_backend *be, struct g15_flight *f)
{
	struct g15_graph *g = &be->g;

	be->r.pass_busy &= ~f->pass_mask;
	f->pass_mask = 0;
	if (!f->ct_count && !f->slot_count)
		return;
	spin_lock(&be->ct_lock);
	for (u32 k = 0; k < f->ct_count; k++)
		f->ct[k]->state = G15_CT_DIRTY;
	/* Without rotation the next claim of a slot scrubs it (STALE). */
	for (u32 k = 0; k < f->slot_count; k++)
		g->slot[f->slot[k]].state = f->rotating ? G15_CT_DIRTY : G15_CT_STALE;
	spin_unlock(&be->ct_lock);
	f->ct_count = 0;
	f->slot_count = 0;
	if (READ_ONCE(ct_async))
		queue_work(be->clear_wq, &be->ct_clear);
}

/*
 * A prefix under preparation (scheduling contract 2): members in API order,
 * each in a claimed compute or pass slot. prev is the engine of the previous
 * member (0 none, 1 compute, 2 render), of this prefix or else the last
 * member of the newest published one; last_ct/last_fs its compute/FS target.
 */
struct g15_prefix {
	u32 computes, renders, prev;
	u32 c_head, ta_head, fs_head;
	u64 n, draw;		/* last compute ordinal / last draw ordinal */
	u32 last_ct, last_fs;
};

/*
 * Compute member: encode into its slot and append to the CDM ring. After a
 * render of the same prefix it first waits on that render's FS (scheduling
 * contract 2, render -> compute): a distinct barrier entry ahead of the work.
 */
static int g15_prefix_compute(struct g15_backend *be, struct g15_job *job,
			      const struct g15_cmd *cmd, struct g15_prefix *pf,
			      struct g15_flight *f)
{
	struct g15_graph *g = &be->g;
	const struct drm_asahi_cmd_compute *c = &cmd->c;
	u64 n = pf->n + 1;
	u32 T = g15_stamp(n), index;
	struct g15_slot *s = g15_slot_claim(be, f, pf->computes, &index);
	int ret;

	if (!s)
		return -EBUSY;	/* nothing published yet */
	ret = g15_encode_compute(be, s, cmd, job->vm->asid, n);
	if (ret)
		return ret;	/* nothing published yet */
	if (pf->prev == 2) {
		g15_barrier(&s->barrier, be->r.fw_stamp[1].va, pf->last_fs, 1, T, G15_UUID, 1);
		g15_buf_w64(&g->ring, 8ULL * pf->c_head, s->barrier.va);
		pf->c_head = (pf->c_head + 1) % G15_QUEUE_CAP;
	}
	g15_buf_w64(&g->ring, 8ULL * pf->c_head, s->work.va);
	pf->c_head = (pf->c_head + 1) % G15_QUEUE_CAP;
	if (n <= 4) {
		dev_info(be->gpu->dev, "compute n=%llu slot %u ctx=%u cdm %#llx..%#llx flags=%#x att=%u ts=%#llx/%#llx\n",
			 n, index, job->vm->asid, c->cdm_ctrl_stream_base,
			 c->cdm_ctrl_stream_end, c->flags, cmd->att_count, cmd->ts_va[0],
			 cmd->ts_va[1]);
		g15_dump_client(be, job->vm, "  cdm", c->cdm_ctrl_stream_base,
				c->cdm_ctrl_stream_end - c->cdm_ctrl_stream_base);
	}
	pf->computes++;
	pf->n = n;
	pf->last_ct = T;
	pf->prev = 1;
	return 0;
}

/*
 * Render member: TA and FS work, sequences, scene and barriers in its pass
 * slot, appended to the TA and FS rings. Its TA waits on the previous FS
 * (full order), on the previous TA (early tiling: a flagged later render
 * after a render, its FS then also waiting on the previous FS), or on the
 * previous compute of the prefix; its FS always waits on its own TA.
 */
static void g15_prefix_render(struct g15_backend *be, struct g15_job *job,
			      const struct g15_cmd *cmd, struct g15_prefix *pf,
			      struct g15_flight *f, u8 *ta, u8 *fs, u8 *tas, u8 *fss)
{
	struct g15_render *r = &be->r;
	const struct drm_asahi_cmd_render *u = &cmd->r;
	struct g15_pass *p = &r->pass[f->pass[pf->renders]];
	u64 n = pf->draw + 1, usc = job->q->usc_base;
	u32 ctx = job->vm->asid;
	u32 Tta = g15_stamp_seed(G15_TA_SEED, n), Tfs = g15_stamp_seed(G15_FS_SEED, n);
	/*
	 * Early tiling (early-tiling addendum): a later render member after a
	 * render of the same prefix that asks for a fragment-only dependency
	 * and no start timestamps lets its TA wait for the previous TA; its FS
	 * still waits for the previous FS, then its own TA.
	 */
	bool early = pf->computes + pf->renders && pf->prev == 2 && READ_ONCE(early_tiling) &&
		(u->flags & DRM_ASAHI_RENDER_FRAGMENT_DEPENDENCY) &&
		!cmd->ts_va[0] && !cmd->ts_va[2];
	struct g15_ct *ct = NULL;
	struct g15_cluster cl;
	struct g15_geom geo;
	bool cluster;
	u8 scene[0x80];

	g15_geometry(u, p->heap.gva, &geo);
	/*
	 * Clustered tiling (clustering contract 1): one layer, one sample,
	 * 32x32 utiles, unless the submission asked for single-cluster vertex
	 * work, and a zeroed buffer holds this geometry.
	 */
	cluster = READ_ONCE(vertex_clustering) && u->layers == 1 && geo.utile_config == 0xa000 &&
		!(u->flags & DRM_ASAHI_RENDER_NO_VERTEX_CLUSTERING);
	if (cluster) {
		g15_cluster_layout(&geo, 0, &cl);
		ct = g15_ct_claim(be, f, cl.bytes);
		cluster = ct;
	}
	if (cluster)
		g15_cluster_layout(&geo, ct->b.gva, &cl);
	memset(ta, 0, G15_TA_WORK_SIZE);
	memset(fs, 0, G15_FS_WORK_SIZE);
	memset(tas, 0, G15_TA_SEQ_SIZE);
	memset(fss, 0, G15_FS_SEQ_SIZE);
	g15_encode_ta(be, p, cmd, ctx, usc, Tta, &geo, cluster ? &cl : NULL, ta, tas);
	g15_encode_fs(be, p, cmd, ctx, usc, n, Tfs, &geo, fs, fss);

	/* The slot is unpublished: its previous prefix fully retired. */
	g15_buf_fill(&p->ta_work, 0, 0, G15_TA_WORK_SIZE);
	g15_buf_fill(&p->fs_work, 0, 0, G15_FS_WORK_SIZE);
	g15_buf_fill(&p->ta_seq, 0, 0, 0x1000);
	g15_buf_fill(&p->fs_seq, 0, 0, 0x1000);
	g15_buf_fill(&p->ta_scratch, 0, 0, 0x10000);
	g15_buf_fill(&p->fs_scratch, 0, 0, 0x10000);
	g15_buf_fill(&p->its, 0, 0, 32);
	g15_buf_fill(&p->tpc, 0, 0, geo.tpc_bytes);
	g15_buf_fill(&p->tilemap, 0, 0, geo.tilemap_bytes);
	/*
	 * The rest of the slot's firmware/GPU-written storage, as the per-use
	 * reset contract (b) requires: auxiliary, heap metadata (logical span)
	 * and scene-user, all zeroed.
	 */
	if (READ_ONCE(pass_full_reset)) {
		g15_buf_fill(&p->aux, 0, 0, 0x20000);
		g15_buf_fill(&p->heap, 0, 0, 0x1000);
		g15_buf_fill(&p->scene_user, 0, 0, 0x10000);
	}
	/* Preemption save areas start zeroed on every use (clustering contract 5). */
	g15_buf_fill(&p->pre[0], 0, 0, r->pre_records * 0x540);
	g15_buf_fill(&p->pre[1], 0, 0, r->pre_records * 0x280);
	g15_buf_fill(&p->pre[2], 0, 0, r->pre_records * 0x40);
	if (cluster)
		be->st_clustered++;	/* its buffer is zero: cleared since its last use */
	g15_buf_write(&p->ta_work, 0, ta, G15_TA_WORK_SIZE);
	g15_buf_write(&p->fs_work, 0, fs, G15_FS_WORK_SIZE);
	g15_buf_write(&p->ta_seq, 0, tas, G15_TA_SEQ_SIZE);
	g15_buf_write(&p->fs_seq, 0, fss, G15_FS_SEQ_SIZE);
	memset(scene, 0, sizeof(scene));
	put_unaligned_le64(r->scene_list.va + 4 * (n % 48), scene + 0x0);
	put_unaligned_le64(r->scene_list.va + 4 * (n % 48), scene + 0x8);
	put_unaligned_le64(g15_compact(p->scene_user.gva + 0x40), scene + 0x28);
	put_unaligned_le32(g15_compact(p->scene_user.gva + 0x900), scene + 0x30);
	put_unaligned_le64(r->mgr_misc.va, scene + 0x40);
	g15_buf_write(&p->scene, 0, scene, sizeof(scene));
	if (pf->prev == 1) {
		/* Compute -> render: the TA waits on the previous compute. */
		g15_barrier(&p->prev_barrier, be->g.fw_stamp.va, pf->last_ct, G15_EVENT_SLOT, Tta,
			    0x3d420, 1);
	} else if (early) {
		g15_barrier(&p->prev_barrier, r->fw_stamp[0].va,
			    g15_stamp_seed(G15_TA_SEED, n - 1), 0, Tta, 0x3d420, 1);
		g15_barrier(&p->fs_dep, r->fw_stamp[1].va, pf->last_fs, 1, Tfs, 0x3d420, 1);
		be->st_early++;
	} else {
		u32 prev_fs = pf->prev ? pf->last_fs :
			n == 1 ? G15_FS_SEED : g15_stamp_seed(G15_FS_SEED, n - 1);

		/* After a recovered failure the previous FS may never have written. */
		if (!pf->prev && be->fs_resync)
			prev_fs = g15_buf_r32(&r->fw_stamp[1], 0);
		g15_barrier(&p->prev_barrier, r->fw_stamp[1].va, prev_fs, 1, Tta, 0x3d420, 1);
	}
	g15_barrier(&p->own_barrier, r->fw_stamp[0].va, Tta, 0, Tfs, G15_FS_UUID, 0);
	if (n == 1 || (!pf->renders && r->reinit)) {
		u8 bm[0x20] = {};

		put_unaligned_le32(6, bm + 0x0);
		put_unaligned_le32(ctx, bm + 0x4);
		put_unaligned_le32(G15_PB_BLOCKS, bm + 0x10);
		put_unaligned_le64(r->pb_mgr.va, bm + 0x14);
		put_unaligned_le32(Tta, bm + 0x1c);
		g15_buf_write(&r->initbm, 0, bm, sizeof(bm));
		g15_buf_w64(&r->ring[0], 8ULL * pf->ta_head, r->initbm.va);
		pf->ta_head = (pf->ta_head + 1) % G15_QUEUE_CAP;
	}
	/*
	 * Rings: TA [InitBM,] predecessor wait, TA; FS [prior-FS wait if
	 * early,] own-TA wait, FS.
	 */
	g15_buf_w64(&r->ring[0], 8ULL * pf->ta_head, p->prev_barrier.va);
	pf->ta_head = (pf->ta_head + 1) % G15_QUEUE_CAP;
	g15_buf_w64(&r->ring[0], 8ULL * pf->ta_head, p->ta_work.va);
	pf->ta_head = (pf->ta_head + 1) % G15_QUEUE_CAP;
	if (early) {
		g15_buf_w64(&r->ring[1], 8ULL * pf->fs_head, p->fs_dep.va);
		pf->fs_head = (pf->fs_head + 1) % G15_QUEUE_CAP;
	}
	g15_buf_w64(&r->ring[1], 8ULL * pf->fs_head, p->own_barrier.va);
	pf->fs_head = (pf->fs_head + 1) % G15_QUEUE_CAP;
	g15_buf_w64(&r->ring[1], 8ULL * pf->fs_head, p->fs_work.va);
	pf->fs_head = (pf->fs_head + 1) % G15_QUEUE_CAP;
	if (n <= 4) {
		dev_info(be->gpu->dev, "render n=%llu slot %u ctx=%u %ux%u L%u s%u utile %ux%u ss%u flags=%#x vdm %#llx ta/fs head %u/%u tpc %#llx tilemap %#llx\n",
			 n, f->pass[pf->renders], ctx, u->width_px, u->height_px, u->layers, u->samples,
			 u->utile_width_px, u->utile_height_px, u->sample_size_B,
			 u->flags, u->vdm_ctrl_stream_base, pf->ta_head, pf->fs_head,
			 geo.tpc_bytes, geo.tilemap_bytes);
		g15_dump_client(be, job->vm, "  vdm", u->vdm_ctrl_stream_base, 64);
	}
	pf->renders++;
	pf->draw = n;
	pf->last_fs = Tfs;
	pf->prev = 2;
}

/*
 * Publish the next prefix of submission job, from its first unpublished
 * command (be->lock held). Members: consecutive commands of one engine, or
 * with mixed_prefix of both engines in API order, each in a free compute or
 * pass slot. All images and ring entries precede one WRITE per participating
 * queue and the kicks (compute, FS, TA); host thresholds and counters name
 * the final ordinals.
 *
 * With prefixes already published (same VM only; two-prefix contract), the
 * new one is appended behind them on the running queues: it takes only slots
 * they do not hold, its first member waits on their last member like any
 * member on its predecessor, and its SubmitQueue records extend the queues'
 * horizons (new_queue 0). Returns -EAGAIN when it cannot be appended yet
 * (no free slot, or a channel record still unconsumed).
 */
static int g15_publish(struct g15_backend *be, struct g15_job *job)
{
	struct g15_gpu *gpu = be->gpu;
	struct g15_graph *g = &be->g;
	struct g15_render *r = &be->r;
	struct g15_flight *f = &be->fl[(be->fl_first + be->fl_count) % G15_FLIGHTS];
	bool overlap = be->fl_count, mixed = READ_ONCE(mixed_prefix);
	struct g15_prefix pf = { .n = be->ordinal, .draw = r->draw };
	u32 i = job->sent, ctx = job->vm->asid, computes = 0, renders = 0, end, free_c;
	bool render0 = job->cmds[i].render;
	u8 *ta = NULL, *fs = NULL, *tas = NULL, *fss = NULL;
	struct g15_geom geo;
	int ret = 0;

	be->prep_from_ns = ktime_get_ns();
	memset(f, 0, sizeof(*f));
	free_c = g15_slots_free(be);
	for (end = i; end < job->count; end++) {
		const struct g15_cmd *cmd = &job->cmds[end];
		int k;

		if (cmd->render != render0 && !mixed)
			break;
		if (!cmd->render) {
			if (computes == free_c)
				break;
			computes++;
			continue;
		}
		g15_geometry(&cmd->r, 0, &geo);
		k = g15_pass_pick(r, &geo, f->pass_mask);
		if (k < 0)
			break;
		f->pass_mask |= BIT(k);
		f->pass[renders++] = k;
	}
	if (end == i)
		return overlap ? -EAGAIN : -E2BIG;	/* idle: every slot is free */
	/*
	 * Participating channels: every earlier record consumed. Behind a
	 * running prefix the firmware takes its record within microseconds:
	 * retry soon rather than at that prefix's retirement.
	 */
	for (u32 ch = 0; ch < 3; ch++) {
		if (ch == G15_COMPUTE_CHANNEL ? !computes : !renders)
			continue;
		if (g15_r32(gpu, G15_CH_STATE(ch)) != be->producer[ch] ||
		    g15_r32(gpu, G15_CH_STATE(ch) + 0x20) != be->producer[ch]) {
			be->chan_wait = overlap;
			return overlap ? -EAGAIN : -EBUSY;
		}
	}
	r->pass_busy |= f->pass_mask;
	if (renders) {
		r->pass_cursor = (f->pass[renders - 1] ?: r->pass_cursor) % max(r->slots - 1, 1U);
		ta = kzalloc(G15_TA_WORK_SIZE, GFP_KERNEL);
		fs = kzalloc(G15_FS_WORK_SIZE, GFP_KERNEL);
		tas = kzalloc(G15_TA_SEQ_SIZE, GFP_KERNEL);
		fss = kzalloc(G15_FS_SEQ_SIZE, GFP_KERNEL);
		ret = ta && fs && tas && fss ? 0 : -ENOMEM;
		if (ret)
			goto out;
		/* After a recovered render abort the allocator state is not trusted. */
		if (r->reinit) {
			g15_render_pools_init(be);
			dev_info(gpu->dev, "render allocators re-initialized; InitBM at draw %llu\n",
				 r->draw + 1);
		}
	}
	if (overlap) {
		pf.prev = be->tail_prev;
		pf.last_ct = be->tail_ct;
		pf.last_fs = be->tail_fs;
	}
	pf.c_head = g15_buf_r32(&g->qstate, 0x40) % G15_QUEUE_CAP;
	pf.ta_head = g15_buf_r32(&r->qstate[0], 0x40) % G15_QUEUE_CAP;
	pf.fs_head = g15_buf_r32(&r->qstate[1], 0x40) % G15_QUEUE_CAP;
	f->c_start = pf.c_head;
	f->ta_start = pf.ta_head;
	f->fs_start = pf.fs_head;
	for (u32 k = i; k < end; k++) {
		const struct g15_cmd *cmd = &job->cmds[k];

		if (cmd->render) {
			g15_prefix_render(be, job, cmd, &pf, f, ta, fs, tas, fss);
			continue;
		}
		ret = g15_prefix_compute(be, job, cmd, &pf, f);
		if (ret)
			goto out;	/* nothing published yet */
	}
	/* Host thresholds/counters after the whole prefix (contract 2 table). */
	if (computes) {
		g15_buf_w32(&g->threshold, 0, (u32)pf.n);
		g15_buf_w32(&g->counter, 0, (u32)pf.n);
		g15_buf_w32(&g->notifier, 0x24, ctx);
	}
	if (renders) {
		g15_buf_w32(&r->threshold, 0, 2 * (u32)pf.draw);
		g15_buf_w32(&r->pb_counter, 0, (u32)pf.draw);
		g15_buf_w32(&r->notifier, 0x24, ctx);
		if (!r->c1_written) {
			u8 c1[16] = {};

			put_unaligned_le32(0x07400000, c1 + 0x0);
			put_unaligned_le32(G15_PB_PAGES, c1 + 0x4);
			ret = g15_write(gpu, G15_C1, c1, sizeof(c1));
			if (ret)
				goto out;
			r->c1_written = true;
		}
	}
	wmb();
	if (computes)
		g15_buf_w32(&g->qstate, 0x40, pf.c_head);
	if (renders) {
		g15_buf_w32(&r->qstate[0], 0x40, pf.ta_head);
		g15_buf_w32(&r->qstate[1], 0x40, pf.fs_head);
	}
	wmb();
	g15_wb_sweep(job->vm);

	f->job = job;
	f->count = end - i;
	f->computes = computes;
	f->renders = renders;
	if (computes) {
		be->ordinal = pf.n;
		f->target = be->target = pf.last_ct;
		f->head = be->head = pf.c_head;
	}
	if (renders) {
		f->ta_target = be->ta_target = g15_stamp_seed(G15_TA_SEED, pf.draw);
		f->fs_target = be->fs_target = pf.last_fs;
		f->ta_head = be->ta_head = pf.ta_head;
		f->fs_head = be->fs_head = pf.fs_head;
		r->draw = pf.draw;
		be->fs_resync = false;
		r->reinit = false;
	}
	be->tail_prev = pf.prev;
	be->tail_ct = pf.last_ct;
	be->tail_fs = pf.last_fs;
	be->events = atomic_read(&gpu->compute_events);
	be->stage_seen = atomic_read(&gpu->stage_events);
	f->published_at = jiffies;
	g15_stat_publish(be, f);
	if (overlap)
		be->st_overlaps++;
	else
		be->deadline = jiffies + msecs_to_jiffies(max(READ_ONCE(deadline_ms), 100U));
	job->sent = end;
	be->fl_count++;
	/* Compute on channel 2, then FS on channel 1, then TA on channel 0 (R8). */
	if (computes) {
		ret = g15_channel_post(be, G15_COMPUTE_CHANNEL, g->qinfo.va, 2, pf.c_head,
				       G15_EVENT_SLOT, !be->queue_registered);
		if (!ret) {
			be->queue_registered = true;
			ret = apple_rtkit_send_message(gpu->rtkit, 0x21, G15_COMPUTE_DOORBELL,
						       NULL, false);
		}
	}
	if (renders && !ret) {
		ret = g15_channel_post(be, G15_FS_CHANNEL, r->qinfo[1].va, 1, pf.fs_head, 1,
				       !r->registered);
		if (!ret)
			ret = apple_rtkit_send_message(gpu->rtkit, 0x21, G15_FS_DOORBELL, NULL,
						       false);
		if (!ret)
			ret = g15_channel_post(be, G15_TA_CHANNEL, r->qinfo[0].va, 0, pf.ta_head, 0,
					       !r->registered);
		if (!ret)
			ret = apple_rtkit_send_message(gpu->rtkit, 0x21, G15_TA_DOORBELL, NULL,
						       false);
		r->registered = true;
	}
	for (u32 ch = 0; ch < 3; ch++)
		f->rec[ch] = be->producer[ch];
	dev_dbg(gpu->dev, "prefix ctx=%u computes %u (n=%llu head %u) renders %u (draw %llu ta/fs head %u/%u) behind %u: %d\n",
		ctx, computes, pf.n, pf.c_head, renders, pf.draw, pf.ta_head, pf.fs_head,
		be->fl_count - 1, ret);
	/* A queue WRITE already advanced: any later failure is uncertain. */
	ret = ret ? -EIO : 0;
out:
	if (ret && ret != -EIO)
		g15_flight_release(be, f);	/* never published */
	kfree(ta);
	kfree(fs);
	kfree(tas);
	kfree(fss);
	return ret;
}

static struct g15_flight *g15_oldest(struct g15_backend *be)
{
	return be->fl_count ? &be->fl[be->fl_first] : NULL;
}

/*
 * Engine receipts (placement addendum Q6), be->lock held: busy is bit 0 of
 * C020 | C120; fault is the D8C0 status under fault-bank selectors 0 and 1
 * (D800), with the saved selector restored. 64-bit accesses.
 */
static void g15_engine_state(struct g15_backend *be, u64 *busy, u64 *fault)
{
	void __iomem *r = be->gpu->sgx_status;
	u64 sel, f[2], a[2];

	*busy = (readq(r + 0x20) | readq(r + 0x120)) & 1;
	sel = readq(r + 0x1800);
	for (u32 bank = 0; bank < 2; bank++) {
		writeq(bank, r + 0x1800);
		f[bank] = readq(r + 0x18c0);
		a[bank] = readq(r + 0x18c8) << 6;
	}
	writeq(sel, r + 0x1800);
	*fault = f[0] | f[1];
	for (u32 bank = 0; bank < 2 && *fault; bank++) {
		struct gpu_m4_drm_vm_leaf leaf = {};
		struct g15_job *job = be->fl_count ? g15_oldest(be)->job : NULL;
		int ret = -ENODEV;

		if (!f[bank])
			continue;
		if (job && a[bank] < G15_UPPER)
			ret = gpu_m4_vm_tree_page(job->vm->tree, a[bank] & ~(G15_PAGE - 1),
						  DRM_ASAHI_BIND_READ, &leaf);
		dev_info(be->gpu->dev, "fault bank %u: status %#llx va %#llx (%s %s level %llu unit %#llx ctx %llu) tree=%d pa=%#llx access=%u\n",
			 bank, f[bank], a[bank], (f[bank] >> 4) & 1 ? "read" : "write",
			 ((f[bank] >> 1) & 7) ? "reason!=unmapped" : "unmapped",
			 (f[bank] >> 7) & 3, (f[bank] >> 9) & 0xff, (f[bank] >> 17) & 0x3f,
			 ret, leaf.pa, leaf.access);
	}
}

static void g15_report(struct g15_backend *be, const char *what)
{
	struct g15_gpu *gpu = be->gpu;
	struct g15_graph *g = &be->g;
	u64 ch = G15_CH_STATE(G15_COMPUTE_CHANNEL), busy, fault;

	struct g15_flight *f = g15_oldest(be);

	g15_engine_state(be, &busy, &fault);
	dev_info(gpu->dev, "%s: ctx=%u prefixes=%u n=%llu target=%#x user=%#x fw=%#x done=%u read=%u write=%u head=%u ch=%u/%u/%u events=%d/%u errors=%d ready=%#x/%u c020=%#llx c120=%#llx busy=%llu fault=%#llx %ums\n",
		 what, f ? f->job->vm->asid : 0, be->fl_count, be->ordinal, be->target,
		 g15_buf_r32(&g->user_stamp, 0),
		 g15_buf_r32(&g->fw_stamp, 0), g15_buf_r32(&g->qstate, 0),
		 g15_buf_r32(&g->qstate, 0x30), g15_buf_r32(&g->qstate, 0x40), be->head,
		 g15_r32(gpu, ch), g15_r32(gpu, ch + 0x10), g15_r32(gpu, ch + 0x20),
		 atomic_read(&gpu->compute_events), be->events,
		 atomic_read(&gpu->event_errors), g15_r32(gpu, G15_RUNTIME + 0x314),
		 g15_r32(gpu, G15_FLAGS + 4), readq(gpu->sgx_status + 0x20),
		 readq(gpu->sgx_status + 0x120), busy, fault,
		 f ? jiffies_to_msecs(jiffies - f->published_at) : 0);
	if (f && f->renders) {
		struct g15_render *r = &be->r;

		dev_info(gpu->dev, "  render: draw=%llu ta %#x/%#x/%#x done=%u read=%u write=%u head=%u ch0=%u/%u; fs %#x/%#x/%#x done=%u read=%u write=%u head=%u ch1=%u/%u; stage events=%d/%u pb counter=%u block ctl=%u/%u\n",
			 r->draw, be->ta_target, g15_buf_r32(&r->user_stamp[0], 0),
			 g15_buf_r32(&r->fw_stamp[0], 0), g15_buf_r32(&r->qstate[0], 0),
			 g15_buf_r32(&r->qstate[0], 0x30), g15_buf_r32(&r->qstate[0], 0x40),
			 be->ta_head, g15_r32(gpu, G15_CH_STATE(0)), g15_r32(gpu, G15_CH_STATE(0) + 0x20),
			 be->fs_target, g15_buf_r32(&r->user_stamp[1], 0),
			 g15_buf_r32(&r->fw_stamp[1], 0), g15_buf_r32(&r->qstate[1], 0),
			 g15_buf_r32(&r->qstate[1], 0x30), g15_buf_r32(&r->qstate[1], 0x40),
			 be->fs_head, g15_r32(gpu, G15_CH_STATE(1)), g15_r32(gpu, G15_CH_STATE(1) + 0x20),
			 atomic_read(&gpu->stage_events), be->stage_seen,
			 g15_buf_r32(&r->pb_counter, 0), g15_buf_r32(&r->block_ctl, 0),
			 g15_buf_r32(&r->block_ctl, 4));
	}
}

/* Stamp cell obs reached own target, bounded by the newest published target tail. */
static bool g15_stamp_reached(u32 obs, u32 own, u32 tail)
{
	return obs && !(obs & 0xff) && obs - own <= tail - own;
}

/* Cursor c of a ring of m entries passed end h of entries from b, within tail t. */
static bool g15_ring_passed(u32 c, u32 b, u32 h, u32 t, u32 m)
{
	u32 dc = (c + m - b) % m, dh = (h + m - b) % m, dt = (t + m - b) % m;

	return dh <= dc && dc <= dt;
}

/*
 * The oldest published prefix while later ones may run (two-prefix contract
 * 4): its own stamp targets reached within the published interval, DONE past
 * its saved ring ends, its channel records consumed. Engine idle is not
 * required; faults still are checked.
 */
static int g15_flight_retired(struct g15_backend *be, struct g15_flight *f)
{
	struct g15_gpu *gpu = be->gpu;
	struct g15_graph *g = &be->g;
	struct g15_render *r = &be->r;
	u64 busy, fault;

	if (f->computes &&
	    (!g15_stamp_reached(g15_buf_r32(&g->user_stamp, 0), f->target, be->target) ||
	     !g15_stamp_reached(g15_buf_r32(&g->fw_stamp, 0), f->target, be->target) ||
	     !g15_ring_passed(g15_buf_r32(&g->qstate, 0), f->c_start, f->head, be->head,
			      G15_QUEUE_CAP) ||
	     !g15_ring_passed(g15_r32(gpu, G15_CH_STATE(G15_COMPUTE_CHANNEL)),
			      (f->rec[2] + 255) % 256, f->rec[2], be->producer[2], 256)))
		return 0;
	if (f->renders &&
	    (!g15_stamp_reached(g15_buf_r32(&r->user_stamp[0], 0), f->ta_target, be->ta_target) ||
	     !g15_stamp_reached(g15_buf_r32(&r->fw_stamp[0], 0), f->ta_target, be->ta_target) ||
	     !g15_stamp_reached(g15_buf_r32(&r->user_stamp[1], 0), f->fs_target, be->fs_target) ||
	     !g15_stamp_reached(g15_buf_r32(&r->fw_stamp[1], 0), f->fs_target, be->fs_target) ||
	     !g15_ring_passed(g15_buf_r32(&r->qstate[0], 0), f->ta_start, f->ta_head, be->ta_head,
			      G15_QUEUE_CAP) ||
	     !g15_ring_passed(g15_buf_r32(&r->qstate[1], 0), f->fs_start, f->fs_head, be->fs_head,
			      G15_QUEUE_CAP) ||
	     !g15_ring_passed(g15_r32(gpu, G15_CH_STATE(G15_TA_CHANNEL)),
			      (f->rec[0] + 255) % 256, f->rec[0], be->producer[0], 256) ||
	     !g15_ring_passed(g15_r32(gpu, G15_CH_STATE(G15_FS_CHANNEL)),
			      (f->rec[1] + 255) % 256, f->rec[1], be->producer[1], 256)))
		return 0;
	g15_engine_state(be, &busy, &fault);
	return fault ? -EIO : 1;
}

/*
 * Retirement of the oldest published prefix: 1 retired, 0 not yet, -EIO
 * fault. The last published one needs every receipt exactly (contract 6.5):
 * final stamps, DONE/READ at the tail heads, consumed pipes, new firing bits
 * and idle engines.
 */
static int g15_retired(struct g15_backend *be)
{
	struct g15_gpu *gpu = be->gpu;
	struct g15_graph *g = &be->g;
	struct g15_render *r = &be->r;
	struct g15_flight *f = g15_oldest(be);
	u64 busy, fault;

	rmb();
	if (be->fl_count > 1)
		return g15_flight_retired(be, f);
	/* Render R8 / contract 8: both stamp pairs, both queues, both pipes. */
	if (f->renders &&
	    (g15_buf_r32(&r->user_stamp[0], 0) != be->ta_target ||
	     g15_buf_r32(&r->fw_stamp[0], 0) != be->ta_target ||
	     g15_buf_r32(&r->user_stamp[1], 0) != be->fs_target ||
	     g15_buf_r32(&r->fw_stamp[1], 0) != be->fs_target ||
	     g15_buf_r32(&r->qstate[0], 0) != be->ta_head ||
	     g15_buf_r32(&r->qstate[0], 0x30) != be->ta_head ||
	     g15_buf_r32(&r->qstate[1], 0) != be->fs_head ||
	     g15_buf_r32(&r->qstate[1], 0x30) != be->fs_head ||
	     g15_r32(gpu, G15_CH_STATE(G15_TA_CHANNEL)) != be->producer[G15_TA_CHANNEL] ||
	     g15_r32(gpu, G15_CH_STATE(G15_FS_CHANNEL)) != be->producer[G15_FS_CHANNEL]))
		return 0;
	if (f->computes &&
	    (g15_buf_r32(&g->user_stamp, 0) != be->target ||
	     g15_buf_r32(&g->fw_stamp, 0) != be->target ||
	     g15_buf_r32(&g->qstate, 0) != be->head ||
	     g15_buf_r32(&g->qstate, 0x30) != be->head ||
	     g15_r32(gpu, G15_CH_STATE(G15_COMPUTE_CHANNEL)) != be->producer[G15_COMPUTE_CHANNEL] ||
	     atomic_read(&gpu->compute_events) == be->events))
		return 0;
	/* New firing bits: at least one per participating stage (1, 2 or 3). */
	if (f->renders &&
	    atomic_read(&gpu->stage_events) - be->stage_seen < (f->computes ? 3 : 2))
		return 0;
	g15_engine_state(be, &busy, &fault);
	if (fault)
		return -EIO;
	return busy ? 0 : 1;
}

static void g15_signal(struct g15_job *job, int error)
{
	if (job->signaled)
		return;
	job->signaled = true;
	if (error)
		dma_fence_set_error(job->fence, error);
	dma_fence_signal(job->fence);
}

/*
 * Next ready job: highest queue priority, then the queue served least
 * recently, then submission order. A queue keeps its order: a job paused at a
 * command boundary is requeued at the head, ahead of its queue's later jobs.
 */
static struct g15_job *g15_pick(struct g15_backend *be)
{
	struct g15_job *job, *best = NULL;

	list_for_each_entry(job, &be->pending, link)
		if (!best || job->q->priority > best->q->priority ||
		    (job->q->priority == best->q->priority && job->q->turn < best->q->turn))
			best = job;
	return best;
}

/*
 * The engine failed with published prefixes: their jobs fail (each once) and,
 * with every owner the firmware may still reference, are retained.
 */
static void g15_fail_flights(struct g15_backend *be, int error)
{
	for (u32 k = 0; k < be->fl_count; k++) {
		struct g15_job *job = be->fl[(be->fl_first + k) % G15_FLIGHTS].job;

		gpu_m4_drm_vm_error(job->vm->frontend, error);
		g15_signal(job, error);
		if (list_empty(&job->link))
			list_add_tail(&job->link, &be->retained);
	}
	be->fl_count = 0;
}

/*
 * Publish further prefixes (be->lock held): with nothing published any ready
 * job; behind published prefixes only the next job of the same VM, up to
 * max_inflight, and not while recovery state is pending. Jobs whose first
 * publication failed before any visible record complete with an error; done
 * collects jobs whose engine reference the caller drops after unlocking.
 */
static void g15_engine_next(struct g15_backend *be, struct list_head *done)
{
	struct g15_job *job;
	int ret;

	be->chan_wait = false;
	while (!be->failed && !list_empty(&be->pending)) {
		struct g15_flight *tail = be->fl_count ?
			&be->fl[(be->fl_first + be->fl_count - 1) % G15_FLIGHTS] : NULL;

		job = g15_pick(be);
		if (tail && (be->fl_count >= clamp(READ_ONCE(max_inflight), 1U, (u32)G15_FLIGHTS) ||
			     job->vm != tail->job->vm || be->r.reinit || be->fs_resync ||
			     be->preempt_global))
			break;
		if (tail && tail->job != job && !list_empty(&tail->job->link))
			be->st_yields++;	/* another queue's turn at a prefix boundary */
		job->q->turn = ++be->turn;
		ret = g15_publish(be, job);
		if (!ret) {
			if (job->sent == job->count)
				list_del_init(&job->link);	/* fully published */
			continue;
		}
		if (ret == -EAGAIN)
			break;		/* storage or channel still held: on a later retirement */
		list_del_init(&job->link);
		if (ret == -EIO || job->sent) {
			/* Uncertain, or earlier commands ran: retain, stop the engine. */
			be->failed = -EIO;
			g15_fail(be->gpu, -EIO);
			g15_fail_flights(be, -EIO);
			gpu_m4_drm_vm_error(job->vm->frontend, -EIO);
			if (list_empty(&job->link))
				list_add_tail(&job->link, &be->retained);
			g15_signal(job, -EIO);
			break;
		}
		gpu_m4_drm_vm_error(job->vm->frontend, ret);
		g15_signal(job, ret);
		list_add_tail(&job->link, done);
	}
	if (be->failed) {
		while (!list_empty(&be->pending)) {
			job = list_first_entry(&be->pending, struct g15_job, link);
			list_del_init(&job->link);
			/* A partly published job already ran commands: its owners stay retained. */
			gpu_m4_drm_vm_error(job->vm->frontend, -EIO);
			g15_signal(job, -EIO);
			list_add_tail(&job->link, job->sent ? &be->retained : done);
		}
	}
}

/*
 * Firmware recovery handshake (same-boot recovery contract 3, 6), offsets in
 * the Control object (initdata root +0xb0): pending records (u32 info, u32
 * wait) at +0x40a0, u64 halt counter +0x4580, halted +0x4590 (firmware sets
 * 1, host clears), resume +0x45a0 (host sets 1, firmware clears).
 */
#define G15_CTL_PENDING		0x40a0
#define G15_CTL_HALT_COUNT	0x4580
#define G15_CTL_HALTED		0x4590
#define G15_CTL_RESUME		0x45a0
#define G15_CTL_FLAG		0x45b0
#define G15_HW_FAULT_INFO	0x8900	/* HardwareData AGXFaultInfo */

static void g15_recover_snapshot(struct g15_backend *be, const char *phase)
{
	struct g15_gpu *gpu = be->gpu;
	const u64 c = G15_CONTROL, f = G15_HWDATA + G15_HW_FAULT_INFO;
	u32 shown = 0;

	dev_info(gpu->dev, "recovery %s: halt_count=%#x halted=%u resume=%u flag=%#x faultinfo status=%u do_not_halt=%u did_not_halt=%u total=%u\n",
		 phase, g15_r32(gpu, c + G15_CTL_HALT_COUNT), g15_r32(gpu, c + G15_CTL_HALTED),
		 g15_r32(gpu, c + G15_CTL_RESUME), g15_r32(gpu, c + G15_CTL_FLAG),
		 g15_r32(gpu, f), g15_r32(gpu, f + 4), g15_r32(gpu, f + 8), g15_r32(gpu, f + 0x18));
	for (u32 i = 0; i < 128 && shown < 16; i++) {
		u32 info = g15_r32(gpu, c + G15_CTL_PENDING + 8 * i);

		if (!(info & 1))
			continue;
		shown++;
		dev_info(gpu->dev, "  pending[%u]: info %#x slot %u wait %#x\n", i, info,
			 (info >> 4) & 0x7f, g15_r32(gpu, c + G15_CTL_PENDING + 8 * i + 4));
	}
}

/*
 * Hang trigger (preemption addendum A, D): a nonzero periodic divisor at
 * Globals +0x1c (compute) or +0x24 (fragment), imported by DEVCTL 0x13, makes
 * the firmware request context switches; a job that does not yield is then
 * killed through its own recovery task (halt, then our resume handshake).
 */
static int g15_preempt_set(struct g15_backend *be, u32 offset, u32 divisor)
{
	struct g15_gpu *gpu = be->gpu;
	u8 rec[0x38] = {};
	int ret;

	put_unaligned_le32(0x13, rec);
	mutex_lock(&gpu->lock);
	g15_w32(gpu, G15_GLOBALS + offset, divisor);
	wmb();
	ret = g15_devctl_record(gpu, rec);
	mutex_unlock(&gpu->lock);
	dev_info(gpu->dev, "preemption divisor Globals+%#x=%u: %d\n", offset, divisor, ret);
	return ret;
}

static void g15_preempt_disarm(struct g15_backend *be)
{
	if (be->preempt_global) {
		g15_preempt_set(be, be->preempt_global, 0);
		be->preempt_global = 0;
	}
}

/* Firmware DONE/READ reached the newest published ends, pipes consumed, engines idle. */
static bool g15_prefix_drained(struct g15_backend *be)
{
	struct g15_gpu *gpu = be->gpu;
	u64 busy, fault;

	{
		struct g15_render *r = &be->r;

		if (g15_buf_r32(&r->qstate[0], 0) != be->ta_head ||
		    g15_buf_r32(&r->qstate[0], 0x30) != be->ta_head ||
		    g15_buf_r32(&r->qstate[1], 0) != be->fs_head ||
		    g15_buf_r32(&r->qstate[1], 0x30) != be->fs_head ||
		    g15_r32(gpu, G15_CH_STATE(G15_TA_CHANNEL)) != be->producer[G15_TA_CHANNEL] ||
		    g15_r32(gpu, G15_CH_STATE(G15_FS_CHANNEL)) != be->producer[G15_FS_CHANNEL])
			return false;
	}
	if (g15_buf_r32(&be->g.qstate, 0) != be->head ||
	    g15_buf_r32(&be->g.qstate, 0x30) != be->head ||
	    g15_r32(gpu, G15_CH_STATE(G15_COMPUTE_CHANNEL)) != be->producer[G15_COMPUTE_CHANNEL])
		return false;
	g15_engine_state(be, &busy, &fault);
	return !busy;
}

/*
 * be->lock held, a published prefix active, a firmware error event seen (or a
 * receipt fault). Returns the error for the prefix and sets *recovered when
 * the firmware halted, consumed the host's resume and moved past the prefix;
 * otherwise the caller latches the GPU failed (host-only timeout, no halt,
 * RTKit failure, or a queue the firmware did not drain).
 */
static int g15_recover(struct g15_backend *be, bool *recovered)
{
	struct g15_gpu *gpu = be->gpu;
	const u64 c = G15_CONTROL;
	int err, t;
	u64 busy, fault;

	*recovered = false;
	smp_rmb();
	g15_engine_state(be, &busy, &fault);
	err = fault || gpu->err_type == 0 ? -EIO : -ETIMEDOUT;
	dev_warn(gpu->dev, "GPU %s: event type %u slot %d raw %*ph; recovering\n",
		 err == -EIO ? "fault" : "timeout", gpu->err_type, gpu->err_slot,
		 (int)sizeof(gpu->err_raw), gpu->err_raw);
	g15_recover_snapshot(be, "error");
	/* The event precedes the halt publication: wait for the real halt. */
	for (t = 0; t < 200 && g15_r32(gpu, c + G15_CTL_HALTED) != 1; t++) {
		mutex_lock(&gpu->lock);
		g15_drain_rings(gpu);
		mutex_unlock(&gpu->lock);
		usleep_range(1000, 1500);
	}
	if (g15_r32(gpu, c + G15_CTL_HALTED) != 1 || READ_ONCE(gpu->error)) {
		dev_err(gpu->dev, "recovery: firmware did not halt (%d ms)\n", t);
		return err;
	}
	g15_recover_snapshot(be, "halted");
	/* Acknowledge: halted = 0, then resume = 1, ordered; nothing else. */
	g15_w32(gpu, c + G15_CTL_HALTED, 0);
	wmb();
	g15_w32(gpu, c + G15_CTL_RESUME, 1);
	wmb();
	for (t = 0; t < 500 && g15_r32(gpu, c + G15_CTL_RESUME); t++)
		usleep_range(1000, 1500);
	if (g15_r32(gpu, c + G15_CTL_RESUME) || READ_ONCE(gpu->error)) {
		dev_err(gpu->dev, "recovery: resume not consumed (%d ms)\n", t);
		return err;
	}
	/* Firmware clears resume before finishing: wait for the queue to drain. */
	for (t = 0; t < 1000 && !g15_prefix_drained(be); t++) {
		mutex_lock(&gpu->lock);
		g15_drain_rings(gpu);
		mutex_unlock(&gpu->lock);
		usleep_range(1000, 1500);
	}
	g15_recover_snapshot(be, "resumed");
	g15_report(be, "recovery");
	if (!g15_prefix_drained(be) || g15_r32(gpu, c + G15_CTL_HALTED) || READ_ONCE(gpu->error)) {
		dev_err(gpu->dev, "recovery: failed prefix not drained (%d ms)\n", t);
		return err;
	}
	*recovered = true;
	return err;
}

static enum hrtimer_restart g15_resnap(struct hrtimer *t)
{
	struct g15_backend *be = container_of(t, struct g15_backend, resnap);

	mod_delayed_work(system_highpri_wq, &be->poll, 0);
	return HRTIMER_NORESTART;
}

/*
 * Re-check a published command soon: hrtimer, with a one-jiffy backup;
 * sooner while a ready prefix waits for a channel record's consumption.
 */
static void g15_rearm(struct g15_backend *be)
{
	u32 us = clamp(READ_ONCE(resnap_us), 25U, 1000U);

	if (be->chan_wait && READ_ONCE(repoll_us))
		us = clamp(READ_ONCE(repoll_us), 5U, 100U);

	hrtimer_start(&be->resnap, us * NSEC_PER_USEC, HRTIMER_MODE_REL);
	mod_delayed_work(system_highpri_wq, &be->poll, 1);
}

/*
 * The oldest published prefix retired (be->lock held): its storage is
 * released, its job completes when this was its last published prefix and
 * nothing of it is left to publish. A hang-killed job fails and drops its
 * unpublished commands.
 */
static void g15_flight_done(struct g15_backend *be, struct g15_flight *f, int error,
			    struct list_head *done)
{
	struct g15_job *job = f->job;

	if (!error)
		g15_stat_retire(be, f);
	g15_flight_release(be, f);	/* after the stats read its timestamp cells */
	be->fl_first = (be->fl_first + 1) % G15_FLIGHTS;
	be->fl_count--;
	if (!be->fl_count)
		be->idle_from_ns = ktime_get_ns();	/* drained: a gap until the next publication */
	/* The next prefix starts executing about now. */
	be->deadline = jiffies + msecs_to_jiffies(max(READ_ONCE(deadline_ms), 100U));
	job->next += f->count;
	if (error) {
		gpu_m4_drm_vm_error(job->vm->frontend, error);
		g15_signal(job, error);
		if (!list_empty(&job->link)) {
			list_del_init(&job->link);	/* drop its unpublished commands */
			job->count = job->sent;
		}
	}
	if (job->next < job->sent || job->sent < job->count)
		return;		/* more of it published or still to publish */
	if (!error) {
		be->st_jobs++;
		be->st_job_cmds += job->count;
		be->st_job_max = max_t(u64, be->st_job_max, job->count);
		dma_rmb();
		g15_wb_sweep(job->vm);
	}
	g15_guard_put(job);
	g15_signal(job, 0);
	list_add_tail(&job->link, done);
}

static void g15_poll_work(struct work_struct *work)
{
	struct g15_backend *be = container_of(to_delayed_work(work), struct g15_backend, poll);
	struct g15_gpu *gpu = be->gpu;
	struct g15_job *job, *tmp;
	struct g15_flight *f;
	LIST_HEAD(done);
	bool again = false;
	int ret = 0;

	mutex_lock(&be->lock);
	if (!be->fl_count || be->failed)
		goto out;
	/* After an event, a few short trailing polls catch the final receipts. */
	for (u32 tail = atomic_xchg(&be->evented, 0) ? 4 : 0; ; tail--) {
		mutex_lock(&gpu->lock);
		g15_drain_rings(gpu);
		mutex_unlock(&gpu->lock);
		ret = g15_retired(be);
		while (ret > 0) {
			f = g15_oldest(be);
			if (be->ordinal <= 2 && be->r.draw <= 2) {
				g15_report(be, "prefix retired");
				mutex_lock(&gpu->lock);
				g15_dump_events(gpu, 2);
				mutex_unlock(&gpu->lock);
			}
			if (be->preempt_global) {
				/*
				 * Retired only after the hang preemption: the firmware
				 * ended it (stamps written, work abandoned). Report the
				 * submission failed and its client's VM lost, like a hang
				 * reset.
				 */
				g15_preempt_disarm(be);
				be->st_hang_kills++;
				if (f->renders)
					be->r.reinit = true;	/* aborted scene: PB state untrusted */
				dev_warn(gpu->dev, "hung submission of ctx %u killed by firmware preemption\n",
					 f->job->vm->asid);
				g15_flight_done(be, f, -ETIMEDOUT, &done);
			} else {
				g15_flight_done(be, f, 0, &done);
			}
			ret = be->fl_count ? g15_retired(be) : 0;
		}
		if (ret || !tail || !be->fl_count)
			break;
		udelay(5);
	}
	if (!be->fl_count)
		goto next;
	f = g15_oldest(be);
	if (!ret && !READ_ONCE(gpu->error) &&
	    atomic_read(&gpu->event_errors) == be->errors_seen &&
	    time_after(jiffies, be->deadline) && !be->preempt_global &&
	    READ_ONCE(hang_preempt) && READ_ONCE(gpu_recovery)) {
		/* Host deadline, no firmware event: let the firmware kill it. */
		g15_report(be, "hung, requesting preemption");
		/* The compute divisor while the oldest prefix's computes are not done. */
		be->preempt_global = f->computes &&
			!g15_stamp_reached(g15_buf_r32(&be->g.fw_stamp, 0), f->target, be->target) ?
			0x1c : f->renders ? 0x24 : 0x1c;
		if (g15_preempt_set(be, be->preempt_global, 1))
			be->deadline = jiffies;	/* fail on the next pass */
		else
			be->deadline = jiffies + msecs_to_jiffies(3000);
		again = true;
	} else if (ret < 0 || READ_ONCE(gpu->error) ||
		   atomic_read(&gpu->event_errors) != be->errors_seen ||
		   time_after(jiffies, be->deadline)) {
		bool recovered = false, render = false;
		int err;

		g15_report(be, "prefix FAILED");
		mutex_lock(&gpu->lock);
		g15_dump_events(gpu, 4);
		mutex_unlock(&gpu->lock);
		for (u32 k = 0; k < be->fl_count; k++)
			render |= be->fl[(be->fl_first + k) % G15_FLIGHTS].renders;
		/*
		 * A firmware error event, or a receipt fault (the fault banks can
		 * be seen before the firmware's event is drained), with a live
		 * transport: recovery waits for the firmware's own halt. Every
		 * published prefix fails with it.
		 */
		if (READ_ONCE(gpu_recovery) && !READ_ONCE(gpu->error) &&
		    (atomic_read(&gpu->event_errors) != be->errors_seen || ret == -EIO)) {
			err = g15_recover(be, &recovered);
			g15_preempt_disarm(be);
			if (recovered) {
				be->errors_seen = atomic_read(&gpu->event_errors);
				be->st_recoveries++;
				if (render) {
					be->fs_resync = true;
					be->r.reinit = true;
				}
				dev_warn(gpu->dev, "recovered: ctx %u submission failed with %d\n",
					 f->job->vm->asid, err);
				/* The failing client's VM is lost; other clients continue. */
				while (be->fl_count)
					g15_flight_done(be, g15_oldest(be), err, &done);
				goto next;
			}
		}
		g15_preempt_disarm(be);
		be->failed = ret ?: READ_ONCE(gpu->error) ?: -ETIMEDOUT;
		g15_fail(gpu, be->failed);
		/* Published and uncertain: never release their owners. */
		g15_fail_flights(be, be->failed);
	} else {
		again = true;
	}
next:
	g15_engine_next(be, &done);
	if (!be->fl_count && be->idle_from_ns) {
		/* Idle with nothing ready is not a gap: the clients are late. */
		be->starve_from_ns = be->idle_from_ns;
		be->idle_from_ns = 0;
	}
	again |= !!be->fl_count;
out:
	if (again)
		g15_rearm(be);
	mutex_unlock(&be->lock);
	list_for_each_entry_safe(job, tmp, &done, link) {
		list_del_init(&job->link);
		g15_job_put(job);
	}
}

static int g15_stats_show(struct seq_file *m, void *unused)
{
	struct g15_backend *be = m->private;
	static const char * const name[3] = { "compute", "render", "mixed" };

	mutex_lock(&be->lock);
	for (u32 e = 0; e < 3; e++)
		seq_printf(m, "%s: count %llu prefixes %llu busy_us %llu max_us %llu first_us %llu last_us %llu\n",
			   name[e], be->st_count[e],
			   be->st_prefixes[e], div_u64(be->st_busy_ns[e], 1000),
			   div_u64(be->st_max_ns[e], 1000), div_u64(be->st_first_ns[e], 1000),
			   div_u64(be->st_last_ns[e], 1000));
	seq_printf(m, "gaps %llu gap_us %llu yields %llu overlaps %llu\n", be->st_gaps,
		   div_u64(be->st_gap_ns, 1000), be->st_yields, be->st_overlaps);
	seq_printf(m, "jobs %llu job_cmds %llu job_max %llu early %llu recoveries %llu hang_kills %llu clustered %llu\n",
		   be->st_jobs, be->st_job_cmds, be->st_job_max, be->st_early, be->st_recoveries,
		   be->st_hang_kills, be->st_clustered);
	seq_printf(m, "ct_sync %llu slot_sync %llu starved_us %llu prep_us %llu\n", be->st_ct_sync,
		   be->st_slot_sync, div_u64(be->st_starved_ns, 1000), div_u64(be->st_prep_ns, 1000));
	seq_printf(m, "stage: ta_us %llu fs_us %llu cdm_us %llu span_us %llu\n",
		   div_u64(be->st_ta_ticks, 24), div_u64(be->st_fs_ticks, 24),
		   div_u64(be->st_cdm_ticks, 24), div_u64(be->st_span_ticks, 24));
	mutex_unlock(&be->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(g15_stats);

void g15_backend_kick(struct g15_gpu *gpu)
{
	struct g15_backend *be = READ_ONCE(gpu->be);

	if (be) {
		atomic_set(&be->evented, 1);
		mod_delayed_work(system_highpri_wq, &be->poll, 0);
	}
}

/* ------------------------------------------------------- scheduler ops --- */
static int g15_cover(struct g15_vm *vm, u64 start, u64 end, u32 access);

static bool stream_gate = true;
module_param(stream_gate, bool, 0644);
MODULE_PARM_DESC(stream_gate, "Reject CDM/VDM streams outside the qualified f568 HoneyKrisp encodings before publication");

/*
 * Client reads through the VM's own translations and the linear map, with
 * the last page translation and the last proven program page cached for one
 * parse (a submission's commands usually share their stream pages).
 */
struct g15_reader {
	struct g15_vm *vm;
	u64 page, prog_page;	/* ~0: none */
	unsigned long cpu;	/* linear-map address of page */
};

static void g15_reader_init(struct g15_reader *rd, struct g15_vm *vm)
{
	rd->vm = vm;
	rd->page = rd->prog_page = ~0ULL;
}

/* Clean+invalidate (never plain invalidate): CPU-written lines may be dirty. */
static int g15_reader_read(struct g15_reader *rd, u64 va, void *dst, u32 len)
{
	while (len) {
		u64 page = va & ~(G15_PAGE - 1);
		u32 n = min_t(u64, len, G15_PAGE - (va & (G15_PAGE - 1)));
		unsigned long cpu;

		if (page != rd->page) {
			struct gpu_m4_drm_vm_leaf leaf = {};

			if (gpu_m4_vm_tree_page(rd->vm->tree, page, DRM_ASAHI_BIND_READ, &leaf))
				return -EFAULT;
			rd->page = page;
			rd->cpu = (unsigned long)phys_to_virt(leaf.pa);
		}
		cpu = rd->cpu + (va & (G15_PAGE - 1));
		dcache_clean_poc(cpu & ~63UL, ALIGN(cpu + n, 64));
		dcache_inval_poc(cpu & ~63UL, ALIGN(cpu + n, 64));
		memcpy(dst, (void *)cpu, n);
		va += n;
		dst = (u8 *)dst + n;
		len -= n;
	}
	return 0;
}

static int g15_client_read(struct g15_vm *vm, u64 va, void *dst, u32 len)
{
	struct g15_reader rd;

	g15_reader_init(&rd, vm);
	return g15_reader_read(&rd, va, dst, len);
}

static int g15_gate_reject(struct g15_vm *vm, const char *what, u64 va, const u32 *w)
{
	dev_warn_ratelimited(vm->be->gpu->dev,
			     "rejecting %s record at %#llx: %08x %08x %08x %08x\n",
			     what, va, w[0], w[1], w[2], w[3]);
	return -EINVAL;
}

/* A launch's program: inside the queue's 4 GiB USC window, first block readable. */
static int g15_gate_program(struct g15_reader *rd, u64 usc, u64 prog)
{
	int ret;

	if (prog < usc || prog - usc >= 0x100000000ULL || (prog & 63))
		return -EINVAL;
	/* 64-aligned: the block never crosses its 16 KiB page. */
	if ((prog & ~(G15_PAGE - 1)) == rd->prog_page)
		return 0;
	ret = g15_cover(rd->vm, prog, prog + 64, DRM_ASAHI_BIND_READ);
	if (!ret)
		rd->prog_page = prog & ~(G15_PAGE - 1);
	return ret;
}

/* Stream bytes [va, va + n) through a window of whole client reads. */
struct g15_window {
	u64 va;
	u32 len;
	u8 b[256];
};

static const u32 *g15_window_get(struct g15_window *win, struct g15_reader *rd, u64 va, u32 n)
{
	u32 len;

	if (win->len && va >= win->va && va + n <= win->va + win->len)
		return (const u32 *)(win->b + (va - win->va));
	/* Up to the window size, not past the page unless the record crosses it. */
	len = max_t(u32, n, min_t(u64, sizeof(win->b), G15_PAGE - (va & (G15_PAGE - 1))));
	win->len = 0;
	if (g15_reader_read(rd, va, win->b, len))
		return NULL;
	win->va = va;
	win->len = len;
	return (const u32 *)win->b;
}

/*
 * CDM (stream admission addendum 1): walk the stream from its base through
 * no-return links until TERMINATE, admitting only the f568 HK G15S records:
 * direct launch (40), indirect launch (36/24), barriers (4), link (8).
 * Bounded; a stream longer than the bound is admitted unchecked past it.
 */
static int g15_gate_cdm(struct g15_reader *rd, u64 usc, u64 va)
{
	struct g15_vm *vm = rd->vm;
	struct g15_window window, *win = &window;
	u32 links = 0, records = 0;
	int ret = 0;

	if (!READ_ONCE(stream_gate))
		return 0;
	win->len = 0;
	while (records++ < 8192) {
		const u32 *w = g15_window_get(win, rd, va, 4);
		u64 prog, ind;

		if (!w) {
			ret = -EFAULT;
			break;
		}
		switch (w[0]) {
		case 0x60000160: case 0x600215fe: case 0x60001560: case 0x600201fe:
			va += 4;
			continue;
		case 0x40000000:
			goto done;
		}
		if ((w[0] & 0xffff0000) == 0x20000000) {
			w = g15_window_get(win, rd, va, 8);
			if (!w) {
				ret = -EFAULT;
				break;
			}
			if (++links > 64)
				goto done;
			va = ((u64)(w[0] & 0xffff) << 32) | w[1];
			if (va & 3) {
				ret = g15_gate_reject(vm, "CDM link", va, w);
				break;
			}
			continue;
		}
		if (w[0] == 0x00080000) {
			w = g15_window_get(win, rd, va, 40);
			if (!w) {
				ret = -EFAULT;
				break;
			}
			prog = ((u64)(w[1] >> 16) << 32) | ((u64)w[2] << 6);
			if ((w[1] & 0xffff) || w[3] != 0x40 || (w[2] >> 26) != ((w[1] >> 16) & 0x3f)) {
				ret = g15_gate_reject(vm, "CDM direct launch", va, w);
				break;
			}
			ret = g15_gate_program(rd, usc, prog);
			if (ret) {
				if (ret == -EINVAL)
					ret = g15_gate_reject(vm, "CDM program", va, w);
				break;
			}
			va += 40;
			continue;
		}
		if ((w[0] & 0xff7fffff) == 0x08080000 || (w[0] & 0xff7fffff) == 0x10080000) {
			bool sizes = (w[0] & 0xff7fffff) == 0x08080000;

			w = g15_window_get(win, rd, va, sizes ? 36 : 24);
			if (!w) {
				ret = -EFAULT;
				break;
			}
			prog = ((u64)(w[1] >> 22) << 38) | ((u64)w[2] << 6);
			ind = ((u64)(w[4] & 0xffff) << 32) | w[5];
			if (w[3] != (0x40000000 | ((prog >> 40) & 0xff)) || (w[4] >> 16) || (ind & 3) ||
			    (sizes && (!w[6] || !w[7] || !w[8] ||
				       w[6] > 0x7ff || w[7] > 0x7ff || w[8] > 0x7ff))) {
				ret = g15_gate_reject(vm, "CDM indirect launch", va, w);
				break;
			}
			ret = g15_gate_program(rd, usc, prog) ?:
			      g15_cover(vm, ind, ind + 12, DRM_ASAHI_BIND_READ);
			if (ret) {
				if (ret == -EINVAL)
					ret = g15_gate_reject(vm, "CDM indirect", va, w);
				break;
			}
			va += sizes ? 36 : 24;
			continue;
		}
		ret = g15_gate_reject(vm, "CDM", va, w);
		break;
	}
done:
	return ret;
}

/* VDM (addendum 2): a normal HK root opens with exactly the G15 barrier. */
static int g15_gate_vdm(struct g15_vm *vm, u64 va)
{
	u32 w[4] = {};

	if (!READ_ONCE(stream_gate))
		return 0;
	if (g15_client_read(vm, va, w, 4))
		return -EFAULT;
	return w[0] == 0x21000960 ? 0 : g15_gate_reject(vm, "VDM root", va, w);
}


static int g15_cover(struct g15_vm *vm, u64 start, u64 end, u32 access)
{
	u64 first = start & ~(G15_PAGE - 1), last = ALIGN(end, G15_PAGE);

	if (end <= start || last < end)
		return -EINVAL;
	return gpu_m4_vm_tree_covers(vm->tree, first, last - first, access, NULL);
}

/* Resolve and hold one requested timestamp output (8 bytes in the arena). */
static int g15_parse_ts(const struct gpu_m4_drm_submit *submit, const struct drm_asahi_timestamp *t,
			struct g15_cmd *cmd, u32 index)
{
	struct gpu_m4_drm_timestamp r;
	struct g15_ts *obj;
	int ret;

	if (!t->handle)
		return 0;
	ret = submit->lookup_timestamp(submit->lookup_cookie, t->handle, t->offset, &r);
	if (ret)
		return ret;
	obj = r.backend;
	if (!obj || r.offset + 8 > (u64)obj->count * G15_PAGE)
		return -EINVAL;
	kref_get(&obj->ref);
	cmd->ts[index] = obj;
	cmd->ts_va[index] = obj->va + r.offset;
	return 0;
}

/* Normalized attachment ranges, 128-byte granular, actually mapped. */
static int g15_parse_att(struct g15_job *job, const struct drm_asahi_attachment *att, u32 count,
			 u32 access)
{
	for (u32 a = 0; a < count; a++) {
		u64 p = att[a].pointer, end;
		int ret;

		if (check_add_overflow(p, att[a].size, &end) || end > U64_MAX - 127 ||
		    (ALIGN(end, 128) - (p & ~127ULL)) / 128 > U32_MAX)
			return -EINVAL;
		ret = g15_cover(job->vm, p & ~127ULL, ALIGN(end, 128), access);
		if (ret)
			return ret;
	}
	return 0;
}

static bool g15_compact_ok(u64 gva)
{
	return gva >= 0x1000000000ULL && gva < 0x9000000000ULL;
}

/* Serial non-clustered render admission (render contract 1, R6). */
static int g15_parse_render(struct g15_job *job, struct g15_cmd *cmd,
			    const struct gpu_m4_drm_submit *submit)
{
	const struct drm_asahi_cmd_render *u = &cmd->r;
	const u32 allowed = DRM_ASAHI_RENDER_PROCESS_EMPTY_TILES |
		DRM_ASAHI_RENDER_NO_VERTEX_CLUSTERING | DRM_ASAHI_RENDER_RSRC_SPEC_HI |
		DRM_ASAHI_RENDER_DBIAS_IS_INT |
		(early_tiling ? DRM_ASAHI_RENDER_FRAGMENT_DEPENDENCY : 0);
	const struct drm_asahi_bg_eot *progs[4] = { &u->bg, &u->eot, &u->partial_bg, &u->partial_eot };
	u64 usc = job->q->usc_base;
	struct g15_geom geo;
	int ret;

	if ((u->flags & ~allowed) || !u->width_px || u->width_px > 16384 || !u->height_px ||
	    u->height_px > 16384 || !u->layers || u->layers > 2048 ||
	    (u->samples != 1 && u->samples != 2 && u->samples != 4) ||
	    (u->utile_width_px != 16 && u->utile_width_px != 32) ||
	    (u->utile_height_px != 16 && u->utile_height_px != 32) || (u->sample_size_B & 7) ||
	    (u32)u->sample_size_B * u->utile_width_px * u->utile_height_px * u->samples > 32768 ||
	    u->sampler_heap || u->sampler_count || u->vertex_helper.binary ||
	    u->vertex_helper.cfg || u->vertex_helper.data || u->fragment_helper.binary ||
	    u->fragment_helper.cfg || u->fragment_helper.data || (u->ppp_ctrl & ~0x203U))
		return -EINVAL;
	if (u->stencil.comp_base || u->stencil.comp_stride ||
	    (u->depth.comp_base && (u->layers != 1 || u->samples != 1)))
		return -EOPNOTSUPP;
	/* VDM: compact-addressable and readable; scissor: 8-aligned readable. */
	if ((u->vdm_ctrl_stream_base & 3) || !g15_compact_ok(u->vdm_ctrl_stream_base) ||
	    !u->isp_scissor_base || (u->isp_scissor_base & 7))
		return -EINVAL;
	ret = g15_cover(job->vm, u->vdm_ctrl_stream_base, u->vdm_ctrl_stream_base + 4,
			DRM_ASAHI_BIND_READ) ?:
	      g15_gate_vdm(job->vm, u->vdm_ctrl_stream_base) ?:
	      g15_cover(job->vm, u->isp_scissor_base, u->isp_scissor_base + 8,
			DRM_ASAHI_BIND_READ);
	if (!ret && u->isp_dbias_base)
		ret = g15_cover(job->vm, u->isp_dbias_base, u->isp_dbias_base + 8,
				DRM_ASAHI_BIND_READ);
	if (!ret && u->isp_oclqry_base)
		ret = g15_cover(job->vm, u->isp_oclqry_base & ~15ULL,
				(u->isp_oclqry_base & ~15ULL) + 8, DRM_ASAHI_BIND_WRITE);
	for (u32 i = 0; !ret && i < 4; i++) {
		u64 a;

		if (!progs[i]->usc)
			continue;
		if (check_add_overflow(usc, (u64)(progs[i]->usc & ~63U), &a))
			return -EINVAL;
		ret = g15_cover(job->vm, a, a + 64, DRM_ASAHI_BIND_READ);
	}
	if (ret)
		return ret;
	g15_geometry(u, 0, &geo);
	if (geo.tpc_bytes > G15_TPC_SIZE || geo.tilemap_bytes > G15_TILEMAP_SIZE)
		return -E2BIG;
	return g15_parse_ts(submit, &u->ts_vtx.start, cmd, 0) ?:
	       g15_parse_ts(submit, &u->ts_vtx.end, cmd, 1) ?:
	       g15_parse_ts(submit, &u->ts_frag.start, cmd, 2) ?:
	       g15_parse_ts(submit, &u->ts_frag.end, cmd, 3);
}

static int g15_parse(struct g15_job *job, const struct gpu_m4_drm_submit *submit)
{
	const u8 *bytes = submit->cmdbuf;
	struct drm_asahi_attachment att[G15_MAX_ATTACHMENTS];
	u32 off = 0, att_count = 0;
	struct g15_reader rd;
	int ret;

	g15_reader_init(&rd, job->vm);
	job->count = 0;
	while (off < submit->size) {
		struct drm_asahi_cmd_header h;

		memcpy(&h, bytes + off, sizeof(h));
		off += sizeof(h);
		if (h.cmd_type == DRM_ASAHI_SET_COMPUTE_ATTACHMENTS) {
			att_count = h.size / sizeof(att[0]);
			if (att_count > G15_MAX_ATTACHMENTS)
				return -EINVAL;
			memcpy(att, bytes + off, h.size);
		} else if (h.cmd_type == DRM_ASAHI_SET_VERTEX_ATTACHMENTS ||
			   h.cmd_type == DRM_ASAHI_SET_FRAGMENT_ATTACHMENTS) {
			/* Validated, never serialized on G15 (render contract 1). */
			struct drm_asahi_attachment ra[G15_MAX_ATTACHMENTS];
			u32 count = h.size / sizeof(ra[0]);

			if (count > G15_MAX_ATTACHMENTS)
				return -EINVAL;
			memcpy(ra, bytes + off, h.size);
			ret = g15_parse_att(job, ra, count, DRM_ASAHI_BIND_READ);
			if (ret)
				return ret;
		} else if (h.cmd_type == DRM_ASAHI_CMD_RENDER) {
			struct g15_cmd *cmd = &job->cmds[job->count++];

			cmd->render = true;
			ret = gpu_m4_drm_payload(&cmd->r, sizeof(cmd->r), bytes + off, h.size) ?:
			      g15_parse_render(job, cmd, submit);
			if (ret)
				return ret;
		} else if (h.cmd_type == DRM_ASAHI_CMD_COMPUTE) {
			struct g15_cmd *cmd = &job->cmds[job->count++];
			const struct drm_asahi_cmd_compute *c = &cmd->c;

			ret = gpu_m4_drm_payload(&cmd->c, sizeof(cmd->c), bytes + off, h.size);
			if (ret)
				return ret;
			if ((c->flags & ~DRM_ASAHI_COMPUTE_WIDE_VISIBILITY) || c->sampler_count ||
			    c->sampler_heap || c->helper.binary || c->helper.cfg || c->helper.data ||
			    (c->cdm_ctrl_stream_base & 3) || (c->cdm_ctrl_stream_end & 3) ||
			    c->cdm_ctrl_stream_end <= c->cdm_ctrl_stream_base)
				return -EINVAL;
			ret = g15_cover(job->vm, c->cdm_ctrl_stream_base, c->cdm_ctrl_stream_end,
					DRM_ASAHI_BIND_READ) ?:
			      g15_gate_cdm(&rd, job->q->usc_base, c->cdm_ctrl_stream_base) ?:
			      g15_parse_att(job, att, att_count, DRM_ASAHI_BIND_WRITE) ?:
			      g15_parse_ts(submit, &c->ts.start, cmd, 0) ?:
			      g15_parse_ts(submit, &c->ts.end, cmd, 1);
			if (ret)
				return ret;
			memcpy(cmd->att, att, sizeof(att[0]) * att_count);
			cmd->att_count = att_count;
		}
		off += h.size;
	}
	return job->count ? 0 : -EINVAL;
}

static void *g15_prepare(void *owner, const struct gpu_m4_drm_submit *submit)
{
	struct g15_queue *q = owner;
	struct g15_job *job;
	int ret;

	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job)
		return ERR_PTR(-ENOMEM);
	kref_init(&job->ref);	/* scheduler cookie */
	INIT_LIST_HEAD(&job->link);
	kref_get(&q->ref);
	job->q = q;
	kref_get(&q->vm->ref);
	job->vm = q->vm;
	job->cmds = kvcalloc(g15_params.max_commands_per_submission, sizeof(*job->cmds),
			     GFP_KERNEL);
	ret = job->cmds ? 0 : -ENOMEM;
	if (!ret)
		ret = gpu_m4_vm_tree_guard_get(job->vm->tree);
	if (!ret) {
		job->guard = true;
		ret = g15_parse(job, submit);
	}
	if (ret) {
		g15_job_put(job);
		return ERR_PTR(ret);
	}
	return job;
}

/* Explicit sync: no reservation fences are published or awaited. */
static void g15_publish_finished(void *owner, void *job, struct dma_fence *finished)
{
}

static struct dma_fence *g15_next_dependency(void *owner, void *job)
{
	return NULL;
}

static struct dma_fence *g15_run(void *owner, void *cookie)
{
	struct g15_queue *q = owner;
	struct g15_backend *be = q->be;
	struct g15_job *job = cookie;
	struct dma_fence *fence;
	LIST_HEAD(done);
	struct g15_job *j, *tmp;

	fence = kzalloc(sizeof(*fence), GFP_KERNEL);
	if (!fence)
		return ERR_PTR(-ENOMEM);
	mutex_lock(&be->lock);
	if (be->failed) {
		mutex_unlock(&be->lock);
		kfree(fence);
		return ERR_PTR(-EIO);
	}
	dma_fence_init(fence, &g15_fence_ops, &be->fence_lock, q->fence_context, ++q->seqno);
	job->fence = dma_fence_get(fence);
	kref_get(&job->ref);	/* engine */
	list_add_tail(&job->link, &be->pending);
	g15_engine_next(be, &done);
	if (be->fl_count)
		g15_rearm(be);
	mutex_unlock(&be->lock);
	list_for_each_entry_safe(j, tmp, &done, link) {
		list_del_init(&j->link);
		g15_job_put(j);
	}
	return fence;
}

static void g15_fault(void *owner, void *cookie, int error)
{
	struct g15_job *job = cookie;

	gpu_m4_drm_vm_error(job->vm->frontend, error);
}

static int g15_dependency_error(void *owner, void *cookie, int error)
{
	struct g15_job *job = cookie;

	/* Another client's failure does not end this job. */
	return gpu_m4_drm_vm_status(job->vm->frontend) ? error : 0;
}

static enum drm_gpu_sched_stat g15_timedout(void *owner, void *cookie,
					    struct drm_sched_job *sched_job)
{
	/* The engine's own 2 s deadline owns failure; late delivery is not a hang. */
	return DRM_GPU_SCHED_STAT_NO_HANG;
}

static void g15_release(void *owner, void *cookie, bool uncertain)
{
	g15_job_put(cookie);
}

static void g15_closed(void *owner)
{
	g15_queue_put(owner);
}

static const struct gpu_m4_drm_sched_ops g15_sched_ops = {
	.prepare = g15_prepare,
	.publish_finished = g15_publish_finished,
	.next_dependency = g15_next_dependency,
	.run = g15_run,
	.fault = g15_fault,
	.fail = g15_fault,
	.dependency_error = g15_dependency_error,
	.timedout = g15_timedout,
	.release = g15_release,
	.closed = g15_closed,
};

/* --------------------------------------------------------- frontend ops --- */

static int g15_session_get(void *owner)
{
	struct g15_backend *be = owner;

	return READ_ONCE(be->gpu->ready) ? 0 : -ENODEV;
}

static void g15_session_put(void *owner)
{
}

/*
 * GET_TIME in the command timestamp timebase: the firmware writes requested
 * timestamps as nanoseconds of the CPU counter (CNTPCT), so scale CNTPCT by
 * its actual frequency (1 GHz on this kernel), not the GPU's 24 MHz clock.
 */
static int g15_read_time(void *owner, u64 *timestamp_ns)
{
	u32 frequency = arch_timer_get_cntfrq();

	if (!frequency)
		return -EIO;
	*timestamp_ns = mul_u64_u32_div(__arch_counter_get_cntpct(), NSEC_PER_SEC, frequency);
	return 0;
}

static int g15_health(void *owner)
{
	struct g15_backend *be = owner;

	return READ_ONCE(be->failed) ?: READ_ONCE(be->gpu->error);
}

static void *g15_vm_create(void *owner, struct gpu_m4_drm_vm *frontend,
			   u64 kernel_start, u64 kernel_end)
{
	struct g15_backend *be = owner;
	struct gpu_m4_vm_tree_config config = {
		.ops = &gpu_m4_vm_tree_installed_ops,
		.get = g15_tree_get, .put = g15_tree_put,
		.low = g15_params.vm_start, .high = g15_params.vm_end,
		.kernel_start = kernel_start, .kernel_end = kernel_end,
		.reserved = {
			{ G15_CLIENT_BASE, G15_CLIENT_END },
			{ 0x10080000000ULL, 0x10100000000ULL },
		},
		.max_tables = G15_VM_TABLES,
		.authority = true, .deferred = g15_tree_deferred,
	};
	struct g15_vm *vm;
	u32 asid;
	int ret;

	if (READ_ONCE(be->failed))
		return ERR_PTR(-EIO);
	vm = kzalloc(sizeof(*vm), GFP_KERNEL);
	if (!vm)
		return ERR_PTR(-ENOMEM);
	kref_init(&vm->ref);
	vm->be = be;
	mutex_init(&vm->objects_lock);
	INIT_LIST_HEAD(&vm->objects);
	config.cookie = vm;
	vm->tree = gpu_m4_vm_tree_create(&config);
	if (IS_ERR(vm->tree)) {
		ret = PTR_ERR(vm->tree);
		kfree(vm);
		return ERR_PTR(ret);
	}
	ret = g15_vm_map_graph(vm);
	mutex_lock(&be->lock);
	asid = find_next_zero_bit(be->asids, 64, 1);
	if (!ret && asid >= 64)
		ret = -ENOSPC;
	if (!ret) {
		set_bit(asid, be->asids);
		vm->asid = asid;
		ret = g15_vm_install(vm);
		if (ret)
			clear_bit(asid, be->asids);
	}
	mutex_unlock(&be->lock);
	if (ret) {
		if (gpu_m4_vm_tree_destroy(vm->tree))
			return ERR_PTR(ret);	/* retained */
		kfree(vm);
		return ERR_PTR(ret);
	}
	gpu_m4_drm_vm_get(frontend);
	vm->frontend = frontend;
	dev_info(be->gpu->dev, "VM context %u root %#llx\n", vm->asid,
		 gpu_m4_vm_tree_root(vm->tree));
	return vm;
}

static int g15_vm_destroy(void *owner, void *cookie)
{
	struct g15_vm *vm = cookie;

	if (gpu_m4_vm_tree_guards(vm->tree))
		return -EBUSY;
	gpu_m4_vm_tree_close(vm->tree);
	g15_vm_put(vm);
	return 0;
}

static void g15_vm_close(void *owner, void *cookie)
{
	struct g15_vm *vm = cookie;

	gpu_m4_vm_tree_close(vm->tree);
	g15_vm_put(vm);
}

static int g15_vm_bind(void *owner, void *cookie, const struct gpu_m4_drm_bind *binds, u32 count)
{
	struct g15_vm *vm = cookie;
	struct gpu_m4_vm_tree_op *ops;
	struct g15_obj **staged;
	int ret = 0;

	if (READ_ONCE(vm->be->failed))
		return -EIO;
	ops = kcalloc(count, sizeof(*ops), GFP_KERNEL);
	staged = kcalloc(count, sizeof(*staged), GFP_KERNEL);
	if (!ops || !staged) {
		ret = -ENOMEM;
		goto out;
	}
	for (u32 i = 0; i < count; i++) {
		const struct drm_asahi_gem_bind_op *op = &binds[i].op;
		struct g15_obj *o;

		ops[i] = (struct gpu_m4_vm_tree_op) { .va = op->addr, .size = op->range };
		if (op->flags & DRM_ASAHI_BIND_UNBIND) {
			ops[i].flags = GPU_M4_VM_TREE_UNBIND;
			continue;
		}
		o = g15_obj_get(vm, binds[i].object);
		if (IS_ERR(o)) {
			ret = PTR_ERR(o);
			goto out;
		}
		staged[i] = o;
		ops[i] = (struct gpu_m4_vm_tree_op) {
			.va = op->addr, .size = op->range, .offset = op->offset,
			.object_size = o->size, .object = o, .pa = o->pa,
			.access = op->flags & (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE),
			.flags = op->flags & DRM_ASAHI_BIND_SINGLE_PAGE ?
				GPU_M4_VM_TREE_SINGLE_PAGE : 0,
			.identity = o->gem, .cpu_policy = gpu_m4_drm_bo_policy(o->gem),
		};
	}
	ret = gpu_m4_vm_tree_bind(vm->tree, ops, count);
out:
	for (u32 i = 0; staged && i < count; i++)
		g15_obj_put(staged[i]);
	kfree(staged);
	kfree(ops);
	if (ret == -EIO)
		gpu_m4_drm_vm_error(vm->frontend, ret);
	return ret;
}

static void g15_bo_close(void *owner, void *cookie, struct drm_gem_object *gem)
{
	struct g15_vm *vm = cookie;
	struct g15_obj *o;

	mutex_lock(&vm->objects_lock);
	o = g15_obj_find(vm, gem);
	mutex_unlock(&vm->objects_lock);
	if (!o)
		return;
	if (gpu_m4_vm_tree_remove_object(vm->tree, o))
		gpu_m4_drm_vm_error(vm->frontend, -EIO);
	g15_obj_put(o);
}

static void *g15_object_bind(void *owner, void *vm, struct drm_gem_object *gem,
			     u64 offset, u64 size)
{
	struct g15_backend *be = owner;
	struct g15_gpu *gpu = be->gpu;
	u32 count = size / G15_PAGE;
	struct g15_ts *t;
	unsigned long first;
	int ret;

	if (!size || !gpu_m4_drm_aligned(offset) || !gpu_m4_drm_aligned(size) ||
	    offset >= gem->size || size > gem->size - offset)
		return ERR_PTR(-EINVAL);
	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (!t)
		return ERR_PTR(-ENOMEM);
	kref_init(&t->ref);
	t->be = be;
	ret = dma_resv_lock(gem->resv, NULL);
	if (ret)
		goto free;
	t->pin = gpu_m4_drm_bo_pin_locked(gem);
	dma_resv_unlock(gem->resv);
	if (IS_ERR(t->pin)) {
		ret = PTR_ERR(t->pin);
		goto free;
	}
	mutex_lock(&be->lock);
	first = bitmap_find_next_zero_area(be->ts_pages, G15_TS_PAGES, 0, count, 0);
	ret = be->failed ?: first >= G15_TS_PAGES ? -ENOSPC : 0;
	if (!ret) {
		t->first = first;
		t->count = count;
		t->va = G15_TIMESTAMPS + first * G15_PAGE;
		mutex_lock(&gpu->lock);
		for (u32 i = 0; !ret && i < count; i++) {
			u64 pa;

			ret = gpu_m4_drm_bo_pin_page(t->pin, offset + i * G15_PAGE, &pa) ?:
			      g15_map_page(gpu, t->va + i * G15_PAGE, pa, G15_PTE_SHARED);
			if (ret)
				while (i--)
					g15_unmap_page(gpu, t->va + i * G15_PAGE);
		}
		mutex_unlock(&gpu->lock);
		if (!ret)
			bitmap_set(be->ts_pages, first, count);
	}
	mutex_unlock(&be->lock);
	if (!ret)
		return t;
	gpu_m4_drm_bo_pin_put(t->pin);
free:
	kfree(t);
	return ERR_PTR(ret);
}

static void g15_object_unbind(void *owner, void *vm, void *object)
{
	g15_ts_put(object);
}

static void *g15_queue_create(void *owner, void *cookie, u32 priority, u64 usc_base)
{
	struct g15_backend *be = owner;
	struct g15_vm *vm = cookie;
	struct gpu_m4_drm_sched_config config = {
		.dev = be->gpu->dev, .ops = &g15_sched_ops, .priority = priority,
		.client_id = (u64)(uintptr_t)vm, .timeout = msecs_to_jiffies(3000),
		.credit_limit = 2,	/* the next job waits in the backend, ready */
	};
	struct g15_queue *q;

	if (READ_ONCE(be->failed))
		return ERR_PTR(-EIO);
	q = kzalloc(sizeof(*q), GFP_KERNEL);
	if (!q)
		return ERR_PTR(-ENOMEM);
	kref_init(&q->ref);		/* frontend handle */
	q->be = be;
	q->usc_base = usc_base;
	q->priority = priority;
	q->fence_context = dma_fence_context_alloc(1);
	kref_get(&vm->ref);
	q->vm = vm;
	config.owner = q;
	kref_get(&q->ref);		/* scheduler callbacks until closed() */
	q->sched = gpu_m4_drm_sched_queue_create(&config);
	if (IS_ERR(q->sched)) {
		void *err = q->sched;

		q->sched = NULL;
		kref_put(&q->ref, g15_queue_release);
		g15_queue_put(q);
		return err;
	}
	return q;
}

static int g15_queue_destroy(void *owner, void *cookie)
{
	struct g15_queue *q = cookie;
	int ret = gpu_m4_drm_sched_queue_try_close(q->sched);

	if (ret)
		return ret;
	gpu_m4_drm_sched_queue_put(q->sched);
	g15_queue_put(q);
	return 0;
}

static void g15_queue_close(void *owner, void *cookie)
{
	struct g15_queue *q = cookie;

	gpu_m4_drm_sched_queue_close(q->sched);
	gpu_m4_drm_sched_queue_put(q->sched);
	g15_queue_put(q);
}

static struct dma_fence *g15_submit(void *owner, void *cookie,
				    const struct gpu_m4_drm_submit *submit)
{
	struct g15_queue *q = cookie;

	return gpu_m4_drm_sched_submit(q->sched, submit);
}

static const struct gpu_m4_drm_ops g15_drm_ops = {
	.session_get = g15_session_get,
	.session_put = g15_session_put,
	.read_time = g15_read_time,
	.health = g15_health,
	.vm_create = g15_vm_create,
	.vm_destroy = g15_vm_destroy,
	.vm_close = g15_vm_close,
	.vm_bind = g15_vm_bind,
	.bo_close = g15_bo_close,
	.object_bind = g15_object_bind,
	.object_unbind = g15_object_unbind,
	.queue_create = g15_queue_create,
	.queue_destroy = g15_queue_destroy,
	.queue_close = g15_queue_close,
	.submit = g15_submit,
};

/* ------------------------------------------------------------- start --- */

int g15_backend_start(struct g15_gpu *gpu)
{
	struct drm_asahi_params_global params;
	struct g15_backend *be;
	struct gpu_m4_drm *drm;
	int ret;

	if (!gpu->ready || gpu->be)
		return -EINVAL;
	for (u32 i = 0; i < 3; i++)
		if (g15_r32(gpu, G15_CH_STATE(i)) || g15_r32(gpu, G15_CH_STATE(i) + 0x20))
			return -EPROTO;	/* TA/FS/compute channels must start empty */
	be = devm_kzalloc(gpu->dev, sizeof(*be), GFP_KERNEL);
	if (!be)
		return -ENOMEM;
	be->clear_wq = alloc_workqueue("apple-gpu-g15-clear", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!be->clear_wq)
		return -ENOMEM;
	be->gpu = gpu;
	mutex_init(&be->lock);
	INIT_LIST_HEAD(&be->pending);
	INIT_LIST_HEAD(&be->retained);
	INIT_LIST_HEAD(&be->dead_vms);
	spin_lock_init(&be->dead_lock);
	spin_lock_init(&be->fence_lock);
	INIT_DELAYED_WORK(&be->poll, g15_poll_work);
	hrtimer_setup(&be->resnap, g15_resnap, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	be->debugfs = debugfs_create_dir("apple-gpu-g15", NULL);
	debugfs_create_file("stats", 0444, be->debugfs, be, &g15_stats_fops);
	INIT_WORK(&be->reap, g15_reap_work);
	spin_lock_init(&be->ct_lock);
	INIT_WORK(&be->ct_clear, g15_ct_clear_work);
	be->fw_next = G15_FW_BASE;
	mutex_lock(&be->lock);
	ret = g15_graph_alloc(be) ?: g15_render_alloc(be);
	/* Mixed-prefix compute barriers, after the render graph's firmware VAs. */
	for (u32 k = 0; !ret && k < be->g.slots; k++)
		ret = g15_buf_alloc(be, &be->g.slot[k].barrier, G15_BARRIER_SIZE, g15_cmd_fw(), 0, 0,
				    false);
	/* Clustering storage: GPU-cached client view and shared upper alias. */
	for (u32 k = 0; !ret && be->r.pre_records == 3 && k < 2 * be->r.slots; k++) {
		struct g15_buf *ct = &be->r.ct[k].b;

		ret = g15_buf_alloc(be, ct, G15_CT_SIZE, G15_PTE_SHARED,
				    G15_CT_GVA + k * G15_CT_STRIDE, 0, false);
		ct->client_attrs = G15_LEAF_CACHED;
		be->r.ct_count = k + 1;
	}
	if (!ret) {
		g15_graph_init(be);
		g15_render_init(be);
	}
	mutex_unlock(&be->lock);
	if (ret) {
		dev_err(gpu->dev, "compute graph allocation failed: %d (retained)\n", ret);
		return ret;
	}
	/* New firmware and context-0 translations before any reference. */
	g15_tlbi_all();
	dev_info(gpu->dev, "compute graph: fw %#llx..%#llx work %#llx/%#llx pools %#llx freelist %#llx table %#llx\n",
		 G15_FW_BASE, be->fw_next, be->g.slot[0].work.va, be->g.slot[0].work.gva,
		 be->g.pools[0].gva, be->g.freelist.gva, be->g.pool_table.gva);
	params = g15_params;
	/* This machine's binning and top DVFS state, as stage 1 measured them. */
	params.core_masks[0] = gpu->core_masks[0];
	params.core_masks[1] = gpu->core_masks[1];
	params.max_frequency_khz = gpu->max_khz;
	if (early_tiling)
		params.features |= DRM_ASAHI_FEATURE_FRAGMENT_DEPENDENCY;
	drm = gpu_m4_drm_create_private(gpu->dev, be, &params, &g15_drm_ops, 0);
	if (IS_ERR(drm))
		return PTR_ERR(drm);
	be->drm = drm;
	WRITE_ONCE(gpu->be, be);
	/* Mesa allocates WRITEBACK BOs; see g15_wb_sweep(). */
	ret = gpu_m4_drm_register_ready(drm, true);
	if (ret)
		return ret;
	dev_info(gpu->dev, "G15S compute render node registered\n");
	return 0;
}
MODULE_IMPORT_NS("DMA_BUF");
