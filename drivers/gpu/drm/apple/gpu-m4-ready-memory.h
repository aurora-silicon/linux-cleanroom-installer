/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_READY_MEMORY_H
#define APPLE_GPU_M4_READY_MEMORY_H

#include <linux/dma-direct.h>
#include <linux/dma-map-ops.h>
#include <linux/soc/apple/pmp-temps.h>
#include "gpu-m4-bootstrap.h"
#include "gpu-m4-pstate.h"
#include "gpu-m4-drm-provider-upper.h"

static bool ready_enable;
module_param(ready_enable, bool, 0400);
MODULE_PARM_DESC(ready_enable, "Permit fresh watchdog-guarded J713 initdata/NOP trial");
static bool policy_enable;
module_param(policy_enable, bool, 0400);
MODULE_PARM_DESC(policy_enable, "Permit separately watchdog-guarded J713 policy continuation");
static bool execution_enable;
module_param(execution_enable, bool, 0400);
MODULE_PARM_DESC(execution_enable, "Stage retained first-execution backing before RUN; no jobs");
static bool runtime_enable;
module_param(runtime_enable, bool, 0400);
MODULE_PARM_DESC(runtime_enable, "Stage native command arena and the ordinary retained M4 runtime before RUN");
/* Per-VM GPU ASIDs: each ordinary user VM's first queue takes a dedicated
 * user context 2..63, so a VM's stable tree root is installed once instead
 * of on every switch between VMs. Off: every VM shares ASID 1.
 */
static bool user_asids = true;
module_param(user_asids, bool, 0400);
MODULE_PARM_DESC(user_asids, "Give each ordinary user VM its own GPU ASID 2..63 (default on; off: all share ASID 1)");
/* Two packets in flight: a lane cap of two, two banks of 16 UMA slots per
 * engine class and a successor admitted beside the head when its bank,
 * context, ring and pipe allow (a global-tail firmware barrier orders it).
 * Default on (r80: desktop, games and CTS subsets without faults); off:
 * serial, byte-identical to the one-credit path.
 */
static bool inflight2 = true;
module_param(inflight2, bool, 0400);
MODULE_PARM_DESC(inflight2, "Two ordinary packets in flight (default 1; 0: one at a time)");
/* inflight2 widening (r66 admission rules): a successor of another queue
 * behind a head still publishing its window, and any (mixed/compute) window
 * behind its own queue. r67: a SuperTuxKart serial mixed window timed out
 * with these; off keeps the r65 rules (whole window published, same queue
 * only render-only behind a render tail).
 */
/* Packets in flight with inflight2 (A/B switch, read per admission): 2 is
 * the working runtime's pair. Up to four, each in its own host UMA slot
 * bank: with three or more the runtime splits each class domain into four
 * banks of 8 slots instead of two of 16, at load. Every admitted packet
 * still publishes its whole window first and its first command waits on
 * the newest in-flight packet's last command (a chain), so a run of small
 * submissions keeps the firmware fed instead of turning over two at a time
 * behind each host receipt.
 */
static unsigned int inflight_jobs = 4;
module_param(inflight_jobs, uint, 0644);
MODULE_PARM_DESC(inflight_jobs, "inflight2: packets in flight, 2..4 (default 4; banks of 8 slots when the load value is >2)");
static bool inflight2_wide;
module_param(inflight2_wide, bool, 0400);
MODULE_PARM_DESC(inflight2_wide, "inflight2: also overlap partial windows and same-queue mixed/compute windows (default 0)");
static bool compute_enable;
module_param(compute_enable, bool, 0400);
MODULE_PARM_DESC(compute_enable, "Stage the pinned J713 compute fixture; separately gated submission");
/* REMOVE-DEBUG: isolated post-RUN alias consumer qualification, never enabled
 * by default. Each mode needs a fresh session and the full shader predicate.
 */
static unsigned int upper_alias_trial;
module_param(upper_alias_trial, uint, 0400);
MODULE_PARM_DESC(upper_alias_trial, "Fresh first-store alias trial: 0 off, 1 existing child, 2 new parent");
static unsigned int pstate_max = GPU_M4_PSTATE_MAX;
module_param(pstate_max, uint, 0400);
MODULE_PARM_DESC(pstate_max, "Configured GPU maximum performance state S, 1..9 (9 = 1470 MHz)");
/* Firmware Ktrace class mask, Control+0, imported once at initdata: bit n
 * enables tag class 0x100*n. Every enabled record is written synchronously
 * by the scheduling, completion and notifier paths that emit it (mutex,
 * 0x48-byte ring store, DSB, producer publish, and a spin when the ring is
 * full), roughly 26 per render and 12 per compute. Native publishes 0 unless
 * kdebug tracing is active; 0x33 (classes 0,1,4,5) is the diagnostic set.
 */
static unsigned int ktrace_mask;
module_param(ktrace_mask, uint, 0400);
MODULE_PARM_DESC(ktrace_mask, "Firmware Ktrace class mask published at initdata (default 0 as native; 0x33 diagnostic)");

/* Retained session queue records mapped ASC-cached (AttrIndex 0, FW RW,
 * PA | 0x00c0000000000443), as native maps its submit-pipe record arrays,
 * work-ring storage and queue-info; index 2 makes the firmware CPU read them
 * inner non-cacheable. Bit 0: the twelve submit-pipe record arrays; bit 1:
 * the three work rings (the firmware cleans and invalidates newly advertised
 * records and ring entries before it reads them; the pipe/ring producer,
 * consumer and CFI cells are separate allocations and stay index 2). Bit 2:
 * the three queue-info records, which the firmware scheduler reads on every
 * probe; the consumer (+0x20..+0x28) and busy (+0x80) cells the host polls
 * reach memory (they equal the index-2 ring-state consumer at idle). Bit 3:
 * the tiler/fragment/compute activity records every Start/Finalize updates
 * (host-written only at initdata). Bit 4: the scheduling context and the
 * queues' notifier list heads (firmware-written after registration, which
 * synchronizes the context; the host only reads its registration bytes).
 * All of these allocations live until reboot, so no cached-mapping teardown
 * is ever needed. Read once before the bootstrap tables are built (boot
 * only); 0 restores index 2 everywhere.
 */
static unsigned int queue_cached = 31;
module_param(queue_cached, uint, 0400);
MODULE_PARM_DESC(queue_cached, "Map session queue records ASC-cached, as native: 1 pipe records, 2 work rings, 4 queue-info, 8 engine activity, 16 context/notifier lists (boot only, default 31)");

struct m4_ready_buffer {
	void *cpu;
	dma_addr_t dma;
	u64 pa, va;
	u32 size;
};

struct m4_command_arena;

enum m4_ready_table { M4_UPPER_LEAF, M4_LOWER_MIDDLE, M4_LOWER_LEAF, M4_READY_TABLES };

struct m4_gpu_ready {
	struct m4_gpu_hello *hello;
	struct m4_gpu_exec_memory *execution;
	/* Native pre-RUN command backing, distinct from every diagnostic DMA BO. */
	struct m4_command_arena *arena;
	/* Separately gated real private-provider root owner; retained to reboot. */
	struct m4_gpu_provider_runtime *provider_runtime;
	struct gpu_m4_drm_upper *upper;
	/* Exclusive fresh TS qualification owner; never coexists with a trial. */
	struct m4_provider_ts_upper *timestamp_upper;
	struct gpu_m4_drm_upper_table upper_extra[32];
	u32 upper_extra_count;
	struct gpu_m4_initdata_input input;
	/* Configured S, host ceilings and REMOVE-DEBUG state samples. */
	struct m4_gpu_pstate pstate;
	struct m4_ready_buffer buffer[GPU_M4_OBJECT_COUNT + 24];
	struct page *table[M4_READY_TABLES];
	u8 expected[M4_READY_TABLES + 2][0x4000]; /* plus shared and lower root */
	void __iomem *generation, *loader;
	u64 next_va;
	struct delayed_work drain_work;
	unsigned long full_health_at; /* jiffies; periodic full diagnostic scan */
	/* Live READY health pass only (m4_ready_health()): the M4_READY_FRESH_*
	 * pages it invalidated before its own first read, and the subset its
	 * upper health may read without invalidating again (m4_upper_table()).
	 */
	u32 health_fresh, health_reuse;
	/* Consecutive drain-worker passes deferred behind the held owner mutex. */
	u32 drain_deferrals;
	/* REMOVE-DEBUG: drain worker mutex waits/deferrals, observational only. */
	u64 drain_waits, drain_wait_ns, drain_wait_max, drain_deferred;
	atomic_t notifications;
	u32 log_records, trace_records, event_records, statistics_records;
	u32 trace_dropped_records, policy_generation;
	u64 trace_lost_records;
	/* REMOVE-DEBUG: first qualified fault-bank sample (see
	 * m4_ready_fault_latch()) and how many fault reports were logged.
	 */
	u8 fault_sample[0x20];
	bool fault_sampled;
	u32 fault_reports;
	/* REMOVE-DEBUG: last observed FirmwareData recovery phase/generation. */
	u32 recovery_phase;
	u64 recovery_generation;
	int error;
	/* queue_cached, sampled once before the bootstrap tables are built. */
	u32 queue_cached;
	bool prepared, published, applications, submitted, complete;
	bool policy_attempted, policy_complete;
};

/* Firmware access class of the session queue record selected by bit. */
static enum gpu_m4_access m4_ready_queue_access(const struct m4_gpu_ready *r, u32 bit)
{
	return r->queue_cached & bit ? GPU_M4_FW_CACHED_RW : GPU_M4_FW_RW;
}

static u64 m4_ready_queue_attrs(const struct m4_gpu_ready *r, u32 bit)
{
	return r->queue_cached & bit ? 0x00c0000000000443ULL : 0x00c000000000044bULL;
}

/* queue_cached bit 3: the per-engine activity records of works. */
static bool m4_ready_object_cached(const struct m4_gpu_ready *r, u32 id)
{
	return (r->queue_cached & 8) && (id == GPU_M4_ACTIVITY_TILER ||
		id == GPU_M4_ACTIVITY_FRAGMENT || id == GPU_M4_ACTIVITY_COMPUTE);
}

static int m4_exec_memory_prepare(struct m4_gpu_hello *h);
static void m4_exec_memory_clean(struct m4_gpu_hello *h);
static bool m4_exec_memory_check(struct m4_gpu_hello *h, bool full);
static bool m4_exec_memory_valid(struct m4_gpu_hello *h);
static int m4_user_prerun_prepare_locked(struct m4_gpu_hello *h);
static int m4_user_upper_init_locked(struct m4_gpu_hello *h);
static bool m4_arena_memory_check(struct m4_gpu_hello *h, bool full);

/* Retained WB pages a live READY health pass invalidates before it first
 * reads them: the reserved root (h->tables), the reserved middle (h->shared)
 * and the READY upper leaf, which the upper owner also names as its root,
 * middle and inherited AP child.
 */
enum {
	M4_READY_FRESH_ROOT = BIT(0),
	M4_READY_FRESH_MIDDLE = BIT(1),
	M4_READY_FRESH_LEAF = BIT(2),
};

/* REMOVE-DEBUG: cost components of every live READY health pass
 * (m4_ready_health(), any holder), observational only, under the GPU mutex.
 * A failing pass stops accounting at its failing component.
 */
enum m4_ready_cost {
	M4_READY_COST_FIXED,	/* callback/RTKit/ASC/pstate, root page and word */
	M4_READY_COST_TABLES,	/* fixed READY, shared and lower root pages */
	M4_READY_COST_UPPER,	/* extra AP pages and the dynamic upper graph */
	M4_READY_COST_EDGES,	/* bootstrap and lower edges */
	M4_READY_COST_EXEC,	/* execution client tables and edges */
	M4_READY_COST_ARENA,	/* command arena path, guards and cells */
	M4_READY_COST_PROVIDER,	/* TTBAT[0/1] and the provider owner health */
	M4_READY_COST_TTBAT,	/* TTBAT slots 2..127 */
	M4_READY_COSTS,
};

static struct {
	u64 passes, ns[M4_READY_COSTS];
	/* Upper-owner page invalidations a pass's earlier one made redundant. */
	u64 reused;
} m4_ready_health_cost;

static u64 m4_ready_cost(enum m4_ready_cost component, u64 start)
{
	u64 now = ktime_get_ns();

	m4_ready_health_cost.ns[component] += now - start;
	return now;
}

/* full=true is the fault/diagnostic verifier (every backing oracle and the
 * checked PA index); full=false is the per-publication/poll live graph health
 * over once-admitted lease inventories.
 */
static bool m4_upper_memory_valid(struct m4_gpu_ready *r, bool full)
{
	bool valid;

	/* Fresh pages belong to exactly one live pass's upper health. */
	r->health_reuse = full ? 0 : r->health_fresh;
	r->health_fresh = 0;
	if (!r->upper) {
		r->health_reuse = 0;
		return !r->upper_extra_count;
	}
	lockdep_assert_held(&r->hello->gpu->lock);
	for (u32 i = 0; i < r->upper_extra_count; i++) {
		void *cpu = r->upper_extra[i].cpu;

		/* Registered retained AP tables only; no PA-to-pointer conversion.
		 * Publication cleaned these pages before making them reachable.
		 */
		dcache_inval_poc((unsigned long)cpu, (unsigned long)cpu + 0x4000);
	}
	/*
	 * The live upper health reads the reserved root, the reserved middle and
	 * the READY upper leaf through m4_upper_table(), which invalidates each
	 * page before the owner's word checks. This same pass invalidated each of
	 * them moments ago, right before its own first read (root word, byte
	 * compare against the expected image), and stores to none of them: the
	 * health is read-only, under the GPU mutex, and the AP never leaves dirty
	 * bytes in a live table page. Those reused pages are therefore read as
	 * fresh as the bootstrap/execution edge walks later in this pass already
	 * read them, without another invalidation. Every word is still compared,
	 * every other page keeps its own invalidation, and firmware stores are
	 * observed by the next pass's invalidation exactly as before. The full
	 * verifier always invalidates again.
	 */
	valid = !(full ? gpu_m4_drm_upper_verify_locked(r->upper) :
		  gpu_m4_drm_upper_health_locked(r->upper));
	r->health_reuse = 0;
	return valid;
}

static int m4_ready_take_va(struct m4_gpu_ready *r, u32 size, u64 *va)
{
	u64 span = ALIGN(size, 0x4000) + 0x4000; /* unmapped guard after every object */

	if (r->next_va >= GPU_M4_HOST_START + GPU_M4_BOOTSTRAP_SPAN ||
	    span > GPU_M4_HOST_START + GPU_M4_BOOTSTRAP_SPAN - r->next_va)
		return -ENOSPC;
	*va = r->next_va;
	r->next_va += span;
	return 0;
}

static int m4_ready_buffer(struct m4_gpu_hello *h, unsigned int id, u32 size,
			   enum gpu_m4_access access)
{
	struct device *dev = h->gpu->dev;
	struct m4_gpu_ready *r = h->ready;
	struct m4_ready_buffer *b = &r->buffer[id];
	u64 end;
	int ret;

	b->size = ALIGN(size, 0x4000);
	b->cpu = dma_alloc_coherent(dev, b->size, &b->dma, GFP_KERNEL);
	if (!b->cpu)
		return -ENOMEM;
	/* The returned Normal-NC pointer is the ONLY CPU data view. Never obtain
	 * or touch a WB direct-map alias. All allocations remain pinned to reboot.
	 */
	b->pa = dma_to_phys(dev, b->dma);
	end = b->pa + b->size - 1;
	if (((unsigned long)b->cpu & 0x3fff) || (b->pa & ~GPU_M4_PA_MASK) ||
	    end < b->pa || end >= BIT_ULL(42) ||
	    phys_to_dma(dev, b->pa) != b->dma ||
	    phys_to_dma(dev, end) != b->dma + b->size - 1 ||
	    dma_to_phys(dev, b->dma + b->size - 1) != end ||
	    region_intersects(b->pa, b->size, IORESOURCE_SYSTEM_RAM,
			      IORES_DESC_NONE) != REGION_INTERSECTS)
		return -EINVAL;
	for (u32 offset = 0; offset < b->size; offset += 0x4000)
		if (!pfn_valid(PHYS_PFN(b->pa + offset)) ||
		    phys_to_dma(dev, b->pa + offset) != b->dma + offset ||
		    dma_to_phys(dev, b->dma + offset) != b->pa + offset)
			return -EINVAL;
	memset(b->cpu, 0, b->size);
	ret = m4_ready_take_va(r, b->size, &b->va);
	if (ret)
		return ret;
	return gpu_m4_bootstrap_map(r->expected[M4_UPPER_LEAF], b->va, b->pa,
				    b->size, access, false);
}

static int m4_ready_mmio(struct m4_gpu_hello *h, u64 va, u64 pa, u32 size,
			 enum gpu_m4_access access)
{
	if (region_intersects(pa, size, IORESOURCE_SYSTEM_RAM, IORES_DESC_NONE) != REGION_DISJOINT)
		return -EINVAL;
	for (unsigned int i = 0; i < 6; i++)
		if (pa <= h->region[i].end && h->region[i].start < pa + size)
			return -EINVAL;
	return gpu_m4_bootstrap_map(h->ready->expected[M4_UPPER_LEAF], va, pa, size, access, false);
}

static int m4_ready_prepare(struct m4_gpu_hello *h)
{
	struct device *dev = h->gpu->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct resource *sgx, generation;
	struct m4_gpu_ready *r = h->ready;
	struct gpu_m4_initdata_input *in = &r->input;
	u64 pa, va;
	u32 size, offset;
	int ret;

	if (!pstate_max || pstate_max > GPU_M4_PSTATE_MAX)
		return -EINVAL;
	/* Do not silently enter a custom coherent pool, IOMMU or arch DMA path. */
	if (dev_is_dma_coherent(dev) || device_iommu_mapped(dev) || get_dma_ops(dev))
		return -EOPNOTSUPP;
#ifdef CONFIG_DMA_DECLARE_COHERENT
	if (dev->dma_mem)
		return -EOPNOTSUPP;
#endif
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(42));
	if (ret)
		return ret;
	r->next_va = GPU_M4_HOST_START;
	r->queue_cached = READ_ONCE(queue_cached) & 31;
	for (unsigned int i = 0; i < GPU_M4_OBJECT_COUNT; i++) {
		ret = m4_ready_buffer(h, i, gpu_m4_object_size[i],
			 i == GPU_M4_UMA || i == GPU_M4_PARAMETER ? GPU_M4_SHARED_RW :
			 m4_ready_object_cached(r, i) ? GPU_M4_FW_CACHED_RW : GPU_M4_FW_RW);
		if (ret)
			return ret;
		in->object[i] = r->buffer[i].cpu;
		in->va[i] = r->buffer[i].va;
	}
	for (unsigned int i = 0; i < 24; i++) {
		unsigned int id = GPU_M4_OBJECT_COUNT + i;

		ret = m4_ready_buffer(h, id, i & 1 ? 0x1800 : 0x30,
			i & 1 ? m4_ready_queue_access(r, 1) : GPU_M4_FW_RW);
		if (ret)
			return ret;
		if (i & 1)
			in->channel_ring[i / 2] = r->buffer[id].va;
		else
			in->channel_state[i / 2] = r->buffer[id].va;
	}
	in->lower_uma = GPU_M4_BOOTSTRAP_LOWER;
	in->lower_parameter = GPU_M4_BOOTSTRAP_LOWER + 0x8000;
	ret = gpu_m4_bootstrap_map(r->expected[M4_LOWER_LEAF], in->lower_uma,
			 r->buffer[GPU_M4_UMA].pa, 0x4000, GPU_M4_GPU_RW, true);
	if (ret)
		return ret;
	ret = gpu_m4_bootstrap_map(r->expected[M4_LOWER_LEAF], in->lower_parameter,
			 r->buffer[GPU_M4_PARAMETER].pa, 0x4000, GPU_M4_GPU_RW, true);
	if (ret)
		return ret;
	for (unsigned int slot = 0; slot < GPU_M4_LOADER_COUNT; slot++) {
		const struct gpu_m4_loader *entry = &gpu_m4_loader[slot];

		if (!entry->length)
			continue;
		ret = m4_ready_take_va(r, (entry->pa & 0x3fff) + entry->length, &va);
		if (ret)
			return ret;
		in->loader_va[slot] = va + (entry->pa & 0x3fff);
		for (unsigned int bank = 0; bank < (slot == 3 ? 2 : 1); bank++) {
			ret = gpu_m4_loader_span(slot, bank, &pa, &size, &offset);
			if (ret)
				return ret;
			ret = m4_ready_mmio(h, va + offset, pa, size, entry->access);
			if (ret)
				return ret;
		}
	}
	ret = m4_ready_take_va(r, 0x8000, &in->scratch);
	if (ret)
		return ret;
	ret = m4_ready_mmio(h, in->scratch, 0x300d60000ULL, 0x8000, GPU_M4_FW_RW);
	if (ret)
		return ret;
	for (unsigned int i = 0; i < M4_READY_TABLES; i++) {
		r->table[i] = alloc_page(GFP_KERNEL | __GFP_ZERO);
		if (!r->table[i])
			return -ENOMEM;
		if (page_to_phys(r->table[i]) & ~GPU_M4_PA_MASK)
			return -EINVAL;
	}
	put_unaligned_le64(page_to_phys(r->table[M4_UPPER_LEAF]) | 3,
			   r->expected[M4_READY_TABLES]);
	put_unaligned_le64(page_to_phys(r->table[M4_LOWER_MIDDLE]) | 3,
			   r->expected[M4_READY_TABLES + 1] + ((in->lower_uma >> 36) & 63) * 8);
	put_unaligned_le64(page_to_phys(r->table[M4_LOWER_LEAF]) | 3,
			   r->expected[M4_LOWER_MIDDLE] + ((in->lower_uma >> 25) & 2047) * 8);
	/* One S for the Main tables, Globals, PowerPerformance and health bounds. */
	r->pstate.configured = pstate_max;
	r->pstate.user = GPU_M4_PSTATE_MAX;
	gpu_m4_thermal_init(&r->pstate.thermal);
	in->ktrace_mask = ktrace_mask & 0x3ff;
	ret = gpu_m4_encode_initdata(in, r->pstate.configured);
	if (ret)
		return ret;
	gpu_m4_pstate_startup(&r->pstate, in->object[GPU_M4_GLOBALS]);
	/* Read-only PMP die records; no mapping or sample keeps the S1 ceiling. */
	r->pstate.temperature = devm_ioremap(dev, PMP_TEMP_SRAM_BASE, PMP_TEMP_SRAM_SIZE);
	if (!r->pstate.temperature)
		dev_warn(dev, "GPU pstate: PMP temperature records unmapped; ceiling stays S1\n");
	/* Optional governor input; a no-op while compiled out. */
	ret = gpu_m4_thermal_governor_register();
	if (ret)
		return ret;
	if (execution_enable) {
		ret = m4_exec_memory_prepare(h);
		if (ret)
			return ret;
	}
	/* CPU claim only for the owned host-generation register, never external IO. */
	sgx = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sgx");
	if (!sgx || sgx->start != 0x300000000ULL || resource_size(sgx) != 0x4000000)
		return -EINVAL;
	generation = *sgx;
	generation.start += 0xd60000;
	generation.end = generation.start + 0x3fff;
	r->generation = devm_ioremap_resource(dev, &generation);
	if (IS_ERR(r->generation))
		return PTR_ERR(r->generation);
	r->loader = devm_ioremap_wc(dev, h->region[M4_DATA].start + 0x1332f0,
				    GPU_M4_LOADER_COUNT * GPU_M4_LOADER_STRIDE);
	if (!r->loader)
		return -ENOMEM;
	r->prepared = true;
	dev_info(dev, "GPU READY private bootstrap: 55 NC buffers, S%u, VA end=%#llx; nothing published\n",
		 r->pstate.configured, r->next_va);
	return 0;
}

/* Called only after the stopped-owner graph and TTBAT have been rechecked. */
static void m4_ready_publish_tables(struct m4_gpu_hello *h)
{
	struct m4_gpu_ready *r = h->ready;
	const unsigned int order[] = { M4_UPPER_LEAF, M4_LOWER_LEAF, M4_LOWER_MIDDLE };

	r->published = true;
	if (r->execution)
		m4_exec_memory_clean(h);
	for (unsigned int i = 0; i < ARRAY_SIZE(order); i++) {
		unsigned int id = order[i];
		void *p = page_address(r->table[id]);

		memcpy(p, r->expected[id], 0x4000);
		dcache_clean_poc((unsigned long)p, (unsigned long)p + 0x4000);
		dma_wmb();
	}
	memcpy(page_address(h->lower), r->expected[M4_READY_TABLES + 1], 0x4000);
	dcache_clean_poc((unsigned long)page_address(h->lower),
			 (unsigned long)page_address(h->lower) + 0x4000);
	dma_wmb();
	memcpy(h->shared, r->expected[M4_READY_TABLES], 0x4000);
	dcache_clean_poc((unsigned long)h->shared, (unsigned long)h->shared + 0x4000);
	dma_wmb();
}

static int m4_ready_edges_valid(struct m4_gpu_hello *h)
{
	struct m4_gpu_ready *r = h->ready;
	struct gpu_m4_bootstrap_view upper = {
		.root = h->tables, .middle = h->shared,
		.leaf = page_address(r->table[M4_UPPER_LEAF]),
		.middle_pa = h->region[M4_SHARED].start,
		.leaf_pa = page_to_phys(r->table[M4_UPPER_LEAF]),
		.running = h->gpu->firmware_start_attempted,
	};
	struct gpu_m4_bootstrap_view lower = {
		.root = page_address(h->lower),
		.middle = page_address(r->table[M4_LOWER_MIDDLE]),
		.leaf = page_address(r->table[M4_LOWER_LEAF]),
		.middle_pa = page_to_phys(r->table[M4_LOWER_MIDDLE]),
		.leaf_pa = page_to_phys(r->table[M4_LOWER_LEAF]),
		.lower = true,
	};
	int ret;

	for (unsigned int i = 0; i < ARRAY_SIZE(r->buffer); i++) {
		struct m4_ready_buffer *b = &r->buffer[i];
		u64 attrs = i == GPU_M4_UMA || i == GPU_M4_PARAMETER ?
			0x00e000000000040bULL : i >= GPU_M4_OBJECT_COUNT &&
			((i - GPU_M4_OBJECT_COUNT) & 1) ? m4_ready_queue_attrs(r, 1) :
			m4_ready_object_cached(r, i) ? 0x00c0000000000443ULL :
			0x00c000000000044bULL;
		u32 bytes = i < GPU_M4_OBJECT_COUNT ? gpu_m4_object_size[i] :
			((i - GPU_M4_OBJECT_COUNT) & 1 ? 0x1800 : 0x30);

		if (!b->cpu || b->size != ALIGN(bytes, 0x4000))
			return -EINVAL;
		if (i < GPU_M4_OBJECT_COUNT &&
		    (r->input.object[i] != b->cpu || r->input.va[i] != b->va))
			return -EPROTO;
		if (i >= GPU_M4_OBJECT_COUNT) {
			unsigned int channel = (i - GPU_M4_OBJECT_COUNT) / 2;

			if (((i - GPU_M4_OBJECT_COUNT) & 1 ? r->input.channel_ring[channel] :
			     r->input.channel_state[channel]) != b->va)
				return -EPROTO;
		}
		ret = gpu_m4_bootstrap_check(&upper, b->va, b->pa, b->size, attrs, true);
		if (ret)
			return ret;
	}
	ret = gpu_m4_bootstrap_check(&lower, r->input.lower_uma,
			r->buffer[GPU_M4_UMA].pa, 0x4000, 0x00c0000000000c8bULL, true);
	if (ret)
		return ret;
	ret = gpu_m4_bootstrap_check(&lower, r->input.lower_parameter,
			r->buffer[GPU_M4_PARAMETER].pa, 0x4000, 0x00c0000000000c8bULL, true);
	if (ret)
		return ret;
	for (unsigned int slot = 0; slot < GPU_M4_LOADER_COUNT; slot++) {
		const struct gpu_m4_loader *entry = &gpu_m4_loader[slot];
		u64 attrs, pa, va;
		u32 bytes, offset;

		if (!entry->length) {
			if (r->input.loader_va[slot])
				return -EPROTO;
			continue;
		}
		attrs = slot == 36 ? 0x00200000000004c7ULL :
			(slot == 10 || slot == 14 || slot == 15 ?
			 0x0080000000000447ULL : 0x00c0000000000447ULL);
		va = r->input.loader_va[slot];
		if ((va & 0x3fff) != (entry->pa & 0x3fff))
			return -EPROTO;
		va -= entry->pa & 0x3fff;
		for (unsigned int bank = 0; bank < (slot == 3 ? 2 : 1); bank++) {
			ret = gpu_m4_loader_span(slot, bank, &pa, &bytes, &offset);
			if (ret)
				return ret;
			ret = gpu_m4_bootstrap_check(&upper, va + offset, pa, bytes,
					attrs, slot != 3 || bank == 1);
			if (ret)
				return ret;
		}
	}
	return gpu_m4_bootstrap_check(&upper, r->input.scratch, 0x300d60000ULL,
				      0x8000, 0x00c000000000044bULL, true);
}

static bool m4_ready_tables_check(struct m4_gpu_hello *h, bool full)
{
	struct m4_gpu_ready *r = h->ready;
	u64 at = full ? 0 : ktime_get_ns();
	int edges;

	/* REMOVE-DEBUG: one-shot attribution of the exact failing sub-check. */
	if (!r->published) {
		dev_warn_once(h->gpu->dev, "REMOVE-DEBUG GPU READY tables full=%u: unpublished (ready-memory:tables)\n", full);
		return false;
	}
	for (unsigned int i = 0; i < M4_READY_TABLES + 2; i++) {
		void *p = i < M4_READY_TABLES ? page_address(r->table[i]) :
			  i == M4_READY_TABLES ? h->shared : page_address(h->lower);

		dcache_inval_poc((unsigned long)p, (unsigned long)p + 0x4000);
		/* The live pass's first read of the shared middle and upper leaf. */
		if (!full && (i == M4_READY_TABLES || i == M4_UPPER_LEAF))
			r->health_fresh |= i == M4_UPPER_LEAF ? M4_READY_FRESH_LEAF :
				M4_READY_FRESH_MIDDLE;
		if (memcmp(p, r->expected[i], 0x4000)) {
			dev_warn_once(h->gpu->dev, "REMOVE-DEBUG GPU READY tables full=%u: table %u bytes differ (ready-memory:memcmp)\n",
				      full, i);
			return false;
		}
	}
	if (!full)
		at = m4_ready_cost(M4_READY_COST_TABLES, at);
	if (!m4_upper_memory_valid(r, full)) {
		dev_warn_once(h->gpu->dev, "REMOVE-DEBUG GPU READY tables full=%u: dynamic upper (ready-memory:upper, see GPU upper line)\n",
			      full);
		return false;
	}
	if (!full)
		at = m4_ready_cost(M4_READY_COST_UPPER, at);
	edges = m4_ready_edges_valid(h);
	if (edges) {
		dev_warn_once(h->gpu->dev, "REMOVE-DEBUG GPU READY tables full=%u: bootstrap edges %d (ready-memory:edges)\n",
			      full, edges);
		return false;
	}
	if (!full)
		at = m4_ready_cost(M4_READY_COST_EDGES, at);
	if (r->execution && !m4_exec_memory_check(h, full)) {
		dev_warn_once(h->gpu->dev, "REMOVE-DEBUG GPU READY tables full=%u: execution memory (ready-memory:exec)\n", full);
		return false;
	}
	if (!full)
		at = m4_ready_cost(M4_READY_COST_EXEC, at);
	if (!m4_arena_memory_check(h, full)) {
		dev_warn_once(h->gpu->dev, "REMOVE-DEBUG GPU READY tables full=%u: command arena (ready-memory:arena)\n", full);
		return false;
	}
	if (!full)
		m4_ready_cost(M4_READY_COST_ARENA, at);
	return true;
}

/* Complete qualification/diagnostic check, including upper backing oracles
 * and a fresh re-proof of every immutable execution/arena allocation fact.
 */
static bool m4_ready_tables_valid(struct m4_gpu_hello *h)
{
	return m4_ready_tables_check(h, true);
}

/* Per-publication/poll check: identical fixed READY table bytes and edges,
 * every live execution client table, arena path/guard, cell identity and lease
 * borrower; the dynamic upper graph uses its live revision health. Immutable
 * allocation admission (DMA/PFN/RAM round trips, sealed fixture graph, pool
 * and arena exclusions, arena uniqueness) is proven once after the final
 * preparation/import and re-proven only by m4_ready_tables_valid(). Only the
 * live pass m4_ready_health() runs it: that pass owns health_fresh.
 */
static bool m4_ready_tables_live(struct m4_gpu_hello *h)
{
	return m4_ready_tables_check(h, false);
}

#include "gpu-m4-exec-memory.h"
#include "gpu-m4-ready-arena.h"

#endif
