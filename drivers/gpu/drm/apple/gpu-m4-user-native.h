/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_USER_NATIVE_H
#define APPLE_GPU_M4_USER_NATIVE_H

#include "gpu-m4-render-native.h"
#include "gpu-m4-compute-packet.h"
#include "gpu-m4-compute-reuse.h"
#include "gpu-m4-user-budget.h"

/* Acceptance storage only. Actual job/cache/lane/binding/UMA/PB retirement is
 * the existing provider/runtime's authority; none is inferred from allocation.
 */
#define M4_USER_NATIVE_LIMIT M4_ORDINARY_NATIVE_MEMORY_LIMIT
#define M4_USER_NATIVE_BO_MAX (32U * GPU_M4_DRM_JOB_MAX_COMMANDS)
#define M4_USER_NATIVE_MAP_MAX (24U * GPU_M4_DRM_JOB_MAX_COMMANDS)
#define M4_USER_NATIVE_EXTRA_MAX (12U * GPU_M4_DRM_JOB_MAX_COMMANDS + 1U)
#define M4_USER_NATIVE_RW (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE)
/* Aggregate idle (displaced, unborrowed, evictable) unit cache, by the bytes
 * a cut releases: cache policy, never firmware ownership. Above it the oldest
 * idle units are evicted even without allocation pressure. Pinned owners
 * (in flight, a global slot's current last owner) are never counted here.
 */
#define M4_USER_NATIVE_IDLE_LIMIT (2048ULL << 20)

/* Each retained command has a >=32MiB pool. The 192 work cells (two per
 * render owner) are the shared finite arena's own structural bound and
 * pressure; the aggregate byte limit is only a host safety bound above it.
 * Keep a warmed bundle's fixed-VA work leases and embedded descriptors
 * intact; a fence alone cannot return/reassign them.
 */

/* Historical observations are not authority: reuse also authenticates the
 * strongly held job's actual successful CORE/cache state. Root writes these
 * observations only from the existing per-command full retirement predicate.
 */
struct m4_user_native_receipt {
	u16 type;
	u32 uma_slot, parameter_id;
	u64 submitted, completed;
	union {
		struct gpu_m4_exec_retirement compute;
		struct gpu_m4_render_stage_retirement render[2];
	};
	/* Captured only after actual full paired retirement. Historical CORE
	 * ownership authenticates these exact dependency/operation observations:
	 * [0] the own TA -> FS edge, [1] a strict TA prior-command edge.
	 */
	struct gpu_m4_render_retirement_dependency render_dependency[3];
	struct gpu_m4_render_retirement_copy render_copies[GPU_M4_DRM_RENDER_TIMESTAMPS];
	u32 render_dependency_count, render_copy_count;
};

struct m4_user_native_history {
	struct list_head garbage;
	/* Settlement (independent last owners): after this COMPLETE history's
	 * receipts were proven once through its actual job, the job is released
	 * (identity comparisons then use the history itself: a freed job's
	 * address may be reused by a new job) and the owner stage stops holding
	 * the units' BOs: each
	 * unit then retains only its own backing, its receipt values here and
	 * the stage's notifier. settle: queued for the reaper (holds one ref).
	 */
	struct list_head settle;
	bool settled, stage_held;
	struct m4_gpu_provider_runtime *runtime;
	struct gpu_m4_drm_provider_job *job;
	struct gpu_m4_drm_provider_version *version; /* Borrowed through job. */
	const struct gpu_m4_drm_vm_owner *snapshot; /* Borrowed through job. */
	/* Immutable membership only, established by receipt_prepare's complete
	 * pin/page/exclusion/cache/resv admission. Protection/retirement remain
	 * live core checks through this exact retained job on EVERY use.
	 */
	struct gpu_m4_drm_cache_job *admitted_cache;
	const struct gpu_m4_drm_mixed_batch *admitted_batch;
	void *admitted_runtime_job;
	const void *admitted_commands, *admitted_command_bytes;
	u32 admitted_command_size;
	u32 admitted_bo_count, admitted_native_count, admitted_extra_count;
	/* Selected-inventory proof of THIS pending job, established once by
	 * receipt_prepare after every stage BO and the notifier passed the actual
	 * job's access check. Nonzero only for this exact job's admitted tuple;
	 * each proven BO carries the same id. Never a phase/protection grant.
	 */
	u64 access_proof;
	/* Receipt proof of this COMPLETE (immutable) history: every receipt
	 * passed its full predicate (paired render retirement and its owner
	 * callback, compute retired prefixes) once, against its owner stage's
	 * immutable natives and this exact retained job/version/snapshot. The
	 * job's CORE/cache phase is still asked live on every check.
	 */
	bool receipts_proven;
	struct m4_user_native_stage *replacement, *release_stage, *owner_stage;
	u64 binding_epoch, charge, render_reset_mask;
	u8 binding_generation;
	u32 asid; /* The user context the job's work named (a value, not a lease). */
	u64 bind_sequence; /* Its bind's p->user_bind_sequence: cache age only. */
	/* inflight2: its first command's global-tail barrier (values, no ref):
	 * the predecessor FW stamp it waited on and that event; 0 = none.
	 */
	u64 cross_stamp_fw;
	u32 cross_event;
	u32 count, spare_refs, last_mask;
	refcount_t refs;
	struct m4_user_native_receipt command[GPU_M4_DRM_JOB_MAX_COMMANDS];
	/* Cold-collector scratch, valid only inside one locked scan: unit edges
	 * (history + displacement) from units selected for the cut, and whether
	 * this history's owner stage was examined/blocked as a dying component.
	 */
	u32 cold_edges;
	bool cold_checked, cold_blocked;
	bool complete;
};

struct m4_user_native_bo {
	struct list_head garbage;
	struct m4_gpu_provider_runtime *runtime;
	struct gpu_m4_drm_provider_vm *vm;
	struct drm_gem_object *gem;
	struct gpu_m4_drm_provider_object *object;
	struct gpu_m4_drm_bo_pin *pin; /* Borrowed from this held accounting owner. */
	struct iosys_map map;
	struct gpu_m4_drm_provider_private_lease *claim;
	struct gpu_m4_drm_provider_private_gem mapping;
	struct gpu_m4_drm_upper_backing backing;
	struct gpu_m4_drm_upper_lease *upper;
	struct m4_command_cell *cell;
	struct m4_user_native_unit *unit;
	u64 va, fw_va, size, coverage, charge;
	/* Id of the latest selected-inventory proof that admitted this BO. */
	/* Compute firmware-alias walk memo: every page's upper translation and
	 * the trailing guard matched with walk_attrs at upper owner revision
	 * walk_revision (any lease/table mutation advances it; pages are pinned).
	 */
	/* bo_check memo: the full check passed against this exact version
	 * (pointer and generation); the object's live gate still runs.
	 */
	const struct gpu_m4_drm_provider_version *checked_version;
	u64 checked_generation;
	u64 walk_revision, walk_attrs;
	u64 access_proof;
	/* REMOVE-DEBUG scratch of one locked holder census (its stamp). */
	u64 census;
	refcount_t refs;
	bool mapped, borrowed_map;
	/* Cold cell wrapper whose cell CREDIT already returned while retired
	 * receipt stages still held the wrapper (cell_return_locked): b->unit
	 * stays its tombstone and the wrapper keeps its own claim, object and
	 * seed upper refs until those stages drain; user_cell_owner[] may name a
	 * new owner of the same cell.
	 */
	bool cell_released;
};

struct m4_user_notifier {
	struct m4_user_native_bo *bo;
	/* threshold[] is also the published threshold tail; published_target[]
	 * the newest reachable work's target stamp per engine on this notifier.
	 */
	u64 sequence[3], threshold[3], stage_counter, charge;
	u32 published_target[3];
	/* The queue VM's user context, written once into every record's +0x24
	 * and named by every work published on this page.
	 */
	u32 asid;
	refcount_t refs;
};

struct m4_user_compute_native {
	struct m4_user_native_bo *private[GPU_M4_COMPUTE_PRIVATE_COUNT], *counter;
};

struct m4_user_render_native {
	struct gpu_m4_render_native_owner owner;
	struct gpu_m4_exec_geometry geometry;
	struct gpu_m4_render_pb_layout pb;
	struct m4_user_native_bo *work[2];
	struct m4_user_native_bo *resource[GPU_M4_RENDER_ENCODED_RESOURCES];
	struct m4_user_native_bo *parameter_heap;
	struct m4_user_native_bo *firmware[GPU_M4_RENDER_FIRST_FW_COUNT];
	struct m4_user_native_bo *gpu[GPU_M4_RENDER_FIRST_GPU_COUNT];
	struct m4_user_native_bo *manager, *tpc;
};

struct m4_user_native_command {
	u16 type;
	struct gpu_m4_render_geometry render_key;
	union {
		struct m4_user_compute_native compute;
		struct m4_user_render_native render;
	};
};

/* A native command's backing is independent of its latest public batch
 * ordinal. Published units remain discoverable while upper baseline owners
 * retain their data; the queue handle cannot silently orphan that inventory.
 */
struct m4_user_native_unit {
	struct list_head retained;
	/* Its queue's units (q->units), exactly while on user_native_units:
	 * warm selection walks only its own queue's units.
	 */
	struct list_head queue_link;
	struct m4_user_queue *queue;
	struct m4_user_native_command command;
	struct m4_user_native_bo *bo[32];
	u32 bo_count, history_index, displacement_index;
	struct m4_user_native_history *history, *displacement;
	struct m4_user_native_unit *displacer;
	struct m4_user_native_stage *accepted;
	u64 charge;
	refcount_t refs;
	bool published, idle;
	/* Explicit cold transition of a closed queue's wholly displaced unit.
	 * Still published: the record stays as a tombstone so the raw b->unit and
	 * user_cell_owner[] pointers remain valid and its cells stay occupied, but
	 * it owns no history/job and is never selected again. cold_mark,
	 * cold_proven, cold_base, cold_warm, cold_idle (an open queue's idle
	 * cache member), cold_trim (selected for its own cache's budget) and
	 * cold_need (in the history component such a selection requires) are
	 * scratch for one locked collector scan only.
	 */
	bool cold, cold_mark, cold_proven, cold_base, cold_warm, cold_idle;
	bool cold_trim, cold_need;
	/* This COMPLETED unit's typed mapped descriptors are members of its VM's
	 * persistent native baseline (future child images). Not a use, borrower,
	 * retirement or release authority; cleared only by an actual removal or
	 * the provider's VM-close detach.
	 */
	bool baseline;
	/* S3c: these descriptors are DRIVER extents of the VM's authority tree
	 * (the hardware mapping). Cleared only by their removal (TLBI), also
	 * after the VM closed.
	 */
	bool tree;
};

/* REMOVE-DEBUG experiment: PB heap multiplier for NEW render units (1 = the
 * geometry policy max(4 MiB, ceil(tiles/128) rounded to 8 blocks)). Read at
 * unit construction and fit checks only; a retained unit keeps its heap.
 */
static unsigned int pb_scale = 4;
module_param(pb_scale, uint, 0644);
MODULE_PARM_DESC(pb_scale, "PB heap multiplier for new render units of a queue that partially rendered at that size (1..16, default 4)");
/* 1: pb_scale only after a partial render (pb_grow_area); 0: always. */
static bool pb_grow_on_partial = true;
module_param(pb_grow_on_partial, bool, 0644);
/* Declare each render's PB at its own geometry need (pb_scale included)
 * instead of the whole unit heap, which unit_highwater sizes to the largest
 * render seen: the firmware InitBuffer before every TA initializes every
 * declared block (~0.33 us per 128 KiB), so a tiny pass on a large reused
 * unit paid for the full-screen heap.
 */
static bool pb_fit = true;
module_param(pb_fit, bool, 0644);
MODULE_PARM_DESC(pb_fit, "Declare each render's PB at its geometry need rather than the unit's whole heap (default 1)");

static int m4_user_render_pb_layout(struct gpu_m4_render_pb_layout *pb,
	const struct gpu_m4_render_geometry *key, const struct m4_user_queue *q)
{
	u32 scale = clamp_t(u32, READ_ONCE(pb_scale), 1, 16);
	int ret = gpu_m4_render_pb_layout(pb, key);

	/* The device-wide hint (any client's partial render at this size or
	 * smaller): a new process need not partially render once more.
	 */
	if (READ_ONCE(pb_grow_on_partial) &&
	    (!q || !q->runtime || !READ_ONCE(q->runtime->user_pb_grow_area) ||
	     (u64)key->width * key->height < READ_ONCE(q->runtime->user_pb_grow_area)))
		scale = 1;
	if (ret || scale == 1)
		return ret;
	return gpu_m4_render_pb_capacity(pb, pb->heap_size * scale);
}

/* Displaced-owner capacity reuse (no exact geometry match): a new render
 * unit's geometry-dependent backing (tilemap, TPC, parameter heap and its
 * lists) is built to the device's high-water need of its layer class, so a
 * later larger render reuses an idle unit instead of constructing another
 * (a new process starts at the sizes earlier ones needed). The encoder recomputes each command's geometry and
 * adopts the unit's actual heap capacity.
 */
static bool unit_highwater = true;
module_param(unit_highwater, bool, 0644);
MODULE_PARM_DESC(unit_highwater, "Build new render units to the largest render need seen (default 1)");

static void m4_user_render_highwater_note(struct m4_user_queue *q, bool layered,
	const struct gpu_m4_exec_geometry *geometry, const struct gpu_m4_render_pb_layout *pb)
{
	u64 *hw = q->runtime->user_render_highwater[layered];
	u64 need[3] = { geometry->tilemap_size, geometry->tpc_size, pb->heap_size };

	for (u32 i = 0; i < 3; i++)
		if (need[i] > READ_ONCE(hw[i]))
			WRITE_ONCE(hw[i], need[i]);
}

static int m4_user_render_highwater_apply(const struct m4_user_queue *q, bool layered,
	struct gpu_m4_exec_geometry *geometry, struct gpu_m4_render_pb_layout *pb)
{
	const u64 *hw = q->runtime->user_render_highwater[layered];
	u64 heap = READ_ONCE(hw[2]);

	if (!READ_ONCE(unit_highwater))
		return 0;
	if (READ_ONCE(hw[0]) > geometry->tilemap_size)
		geometry->tilemap_size = READ_ONCE(hw[0]);
	if (READ_ONCE(hw[1]) > geometry->tpc_size)
		geometry->tpc_size = READ_ONCE(hw[1]);
	return heap > pb->heap_size ? gpu_m4_render_pb_capacity(pb, heap) : 0;
}

struct m4_user_native_stage {
	struct list_head retained, idle;
	struct m4_user_queue *queue;
	struct gpu_m4_drm_provider_version *source;
	struct m4_user_notifier *notifier;
	struct m4_user_native_bo *bo[M4_USER_NATIVE_BO_MAX];
	struct gpu_m4_drm_provider_private_gem native[M4_USER_NATIVE_MAP_MAX];
	struct gpu_m4_drm_provider_stage_extra extra[M4_USER_NATIVE_EXTRA_MAX];
	struct m4_user_native_command command[GPU_M4_DRM_JOB_MAX_COMMANDS];
	struct m4_user_native_unit *unit[GPU_M4_DRM_JOB_MAX_COMMANDS], *building;
	struct m4_user_native_history *reuse_history[GPU_M4_DRM_JOB_MAX_COMMANDS];
	struct m4_user_native_history *reuse_displacement[GPU_M4_DRM_JOB_MAX_COMMANDS];
	u32 reuse_index[GPU_M4_DRM_JOB_MAX_COMMANDS], displacement_index[GPU_M4_DRM_JOB_MAX_COMMANDS];
	u64 reused_commands;
	/* Warm units moved to another slot of this stage's bank (dirtyroom
	 * idle-owner retargeting): retarget_slot[i] is command i's destination,
	 * a slot with a retired current owner (kept until this use retires).
	 */
	u64 retargeted;
	u32 retarget_slot[GPU_M4_DRM_JOB_MAX_COMMANDS];
	struct m4_user_native_history *history, *pending;
	struct m4_user_native_stage *idle_replacement;
	u64 next_va, charge, allocation_bytes;
	u32 count, bo_count, native_count, extra_count;
	u32 bank; /* host UMA slot bank its warm units and leases must lie in */
	bool bank_staging; /* counted in runtime user_bank_staging[bank] */
	refcount_t refs;
	bool exposed, displaced, reused, cached, cache_ref;
	bool history_owned;
	/* Its job's stage reference is gone (the job's cache/snapshot pin
	 * borrowers with it), and its units' BO references were dropped after
	 * settlement (bo[] entries of unit BOs are then NULL).
	 */
	bool job_gone, bos_dropped;
	bool deferred_materialization;
	/* The latest construction refusal came from a native BACKING quota:
	 * the runtime native charge or the GEM factory's native quota, recorded
	 * at that admission. Provider pin/claim/upper metadata capacity and host
	 * allocation refusals leave it clear and keep their own errno.
	 */
	bool backing_refused;
};

static void m4_user_queue_put_locked(struct m4_user_queue *q);
static void m4_user_native_stage_put(void *owner, void *stage_owner);
static bool m4_user_native_reused(const struct m4_user_native_stage *s, u32 index);
static int m4_user_native_slot_owner_locked(struct m4_gpu_provider_runtime *p,
	u16 type, u32 slot, struct m4_user_native_unit **owner);

/* Qualified UMA lease per command type: compute 1..32, paired render 33..64
 * with PB ID slot - 33. Event/scheduler IDs are never slot numbers.
 */
static bool m4_user_native_slot_valid(u16 type, u32 slot)
{
	if (type == DRM_ASAHI_CMD_COMPUTE)
		return slot >= 1 && slot <= GPU_M4_COMPUTE_PACKET_SLOT_MAX;
	return type == DRM_ASAHI_CMD_RENDER && slot >= GPU_M4_RENDER_UMA_SLOT_FIRST &&
		slot - GPU_M4_RENDER_UMA_SLOT_FIRST < GPU_M4_RENDER_UMA_SLOT_COUNT;
}

static_assert(GPU_M4_COMPUTE_PACKET_SLOT_MAX < GPU_M4_RENDER_UMA_SLOT_FIRST &&
	      GPU_M4_RENDER_UMA_SLOT_FIRST + GPU_M4_RENDER_UMA_SLOT_COUNT == M4_USER_SLOT_TABLE,
	      "per-slot owner table does not cover the encoded slot domains");

static_assert(M4_USER_BANK_COUNT <= M4_USER_BANK_MAX &&
	      M4_USER_BANK_COUNT * M4_USER_BANK_SIZE <= GPU_M4_COMPUTE_PACKET_SLOT_MAX &&
	      M4_USER_BANK_COUNT * M4_USER_BANK_SIZE <= GPU_M4_RENDER_UMA_SLOT_COUNT &&
	      M4_USER_INFLIGHT2_BANKS <= M4_USER_BANK_MAX &&
	      M4_USER_INFLIGHT2_BANKS * M4_USER_INFLIGHT2_BANK_SIZE <= GPU_M4_COMPUTE_PACKET_SLOT_MAX &&
	      M4_USER_INFLIGHT2_BANKS * M4_USER_INFLIGHT2_BANK_SIZE <= GPU_M4_RENDER_UMA_SLOT_COUNT,
	      "host UMA slot banks exceed the encoded slot domains");

/* The runtime's bank geometry, sampled once at m4_user_init(): one bank of
 * M4_USER_BANK_SIZE, or (inflight2) M4_USER_INFLIGHT2_BANKS of
 * M4_USER_INFLIGHT2_BANK_SIZE. An unset owner keeps the single bank.
 */
static u32 m4_user_bank_count(const struct m4_gpu_provider_runtime *p)
{
	return p->user_bank_count ?: M4_USER_BANK_COUNT;
}

static u32 m4_user_bank_size(const struct m4_gpu_provider_runtime *p)
{
	return p->user_bank_size ?: M4_USER_BANK_SIZE;
}

/* Only the first P positions of each bank, P the largest per-class window
 * (2P firmware-last owners per type instead of the whole domain): a child
 * never needs more distinct slots of a type than it has commands.
 */
static bool bank_window_slots = true;
module_param(bank_window_slots, bool, 0644);
MODULE_PARM_DESC(bank_window_slots, "Use only the first window-size positions of each slot bank (default 1)");

static u32 m4_user_bank_slots(const struct m4_gpu_provider_runtime *p)
{
	u32 size = m4_user_bank_size(p), used = READ_ONCE(p->user_bank_used);

	return READ_ONCE(bank_window_slots) && used && used < size ? used : size;
}

/* A valid slot inside host bank b of its type's domain. */
static bool m4_user_native_bank_slot(const struct m4_gpu_provider_runtime *p, u16 type,
	u32 bank, u32 slot)
{
	u32 first = (type == DRM_ASAHI_CMD_RENDER ? GPU_M4_RENDER_UMA_SLOT_FIRST : 1) +
		bank * m4_user_bank_size(p);

	return bank < m4_user_bank_count(p) && m4_user_native_slot_valid(type, slot) &&
		slot >= first && slot - first < m4_user_bank_slots(p);
}

/* inflight2: the bank a new stage leases from, chosen once under the GPU
 * mutex before any warm unit is taken (a warm unit keeps its slot, so it
 * serves only its own bank). The queue's own bank holds its warm units;
 * while another accepted or in-flight job holds it, any bank nobody holds
 * lets this job overlap; with none free the queue's bank only waits (the
 * lane admits a successor only in a bank no in-flight job holds).
 */
static bool bank_alternate = true;
module_param(bank_alternate, bool, 0644);
MODULE_PARM_DESC(bank_alternate, "inflight2: with every bank held, a new stage takes the bank its queue's newest job does not hold (default 1)");

static u32 m4_user_bank_choose_locked(const struct m4_gpu_provider_runtime *p,
	const struct m4_user_queue *q)
{
	u32 count = m4_user_bank_count(p), want = q->bank < count ? q->bank : 0;
	u32 busy = p->user_bank_busy, flight = p->user_bank_flight;

	if (count <= 1)
		return want;
	for (u32 b = 0; b < count; b++)
		if (p->user_bank_staging[b])
			busy |= BIT(b);
	busy |= flight;
	/* Nobody's: the queue's own bank (its warm units), else any. */
	if (!(busy & BIT(want)))
		return want;
	for (u32 b = 0; b < count; b++)
		if (!(busy & BIT(b)))
			return b;
	/* Shared only with other queues' jobs not in flight: this job can
	 * still overlap whatever executes now (and they each other's).
	 */
	for (u32 i = 0, b = want; i < count; i++, b = (b + 1) % count)
		if (!(flight & BIT(b)) && !q->bank_holders[b])
			return b;
	/* Every bank held: not the bank of this queue's newest job (want) but
	 * the next one, whose older job retires first, so the lane can admit
	 * this one behind the newest (alternating banks keeps two in flight).
	 * Warm units of the other bank follow by retargeting.
	 */
	return READ_ONCE(bank_alternate) ? (want + 1) % count : want;
}

/* A choosing stage's bank reservation ends at acceptance or its release. */
static void m4_user_bank_unstage_locked(struct m4_gpu_provider_runtime *p,
	struct m4_user_native_stage *s)
{
	if (!s->bank_staging)
		return;
	s->bank_staging = false;
	if (!WARN_ON(s->bank >= M4_USER_BANK_MAX || !p->user_bank_staging[s->bank]))
		p->user_bank_staging[s->bank]--;
}

/* PB ID leased together with a valid render UMA slot; compute owns none. */
static u32 m4_user_native_slot_parameter(u16 type, u32 slot)
{
	return type == DRM_ASAHI_CMD_RENDER ? slot - GPU_M4_RENDER_UMA_SLOT_FIRST : 0;
}

static int m4_user_native_charge_locked(struct m4_gpu_provider_runtime *p, u64 size)
{
	lockdep_assert_held(&p->hello->gpu->lock);
	if (!size || size > M4_USER_NATIVE_LIMIT || p->user_native_bytes > M4_USER_NATIVE_LIMIT - size)
		return -ENOMEM;
	p->user_native_bytes += size;
	return 0;
}

/* Allocate the finite receipt workspace at acceptance, never protected arm.
 * The old job ref is later queued outside frontend/provider/reservation locks.
 */
static struct m4_user_native_history *m4_user_native_history_create(
	struct m4_gpu_provider_runtime *p, u32 count)
{
	struct mutex *lock = &p->hello->gpu->lock;
	struct m4_user_native_history *h;
	int ret;

	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	ret = m4_user_native_charge_locked(p, sizeof(*h));
	mutex_unlock(lock);
	if (ret)
		return ERR_PTR(ret);
	h = kvzalloc(sizeof(*h), GFP_KERNEL);
	if (!h) {
		mutex_lock(lock);
		p->user_native_bytes -= sizeof(*h);
		mutex_unlock(lock);
		return ERR_PTR(-ENOMEM);
	}
	INIT_LIST_HEAD(&h->garbage);
	INIT_LIST_HEAD(&h->settle);
	h->runtime = p;
	h->count = count;
	h->charge = sizeof(*h);
	refcount_set(&h->refs, 1);
	return h;
}

static void m4_user_native_history_put_locked(struct m4_user_native_history *h)
{
	struct m4_gpu_provider_runtime *p;

	if (!h)
		return;
	p = h->runtime;
	lockdep_assert_held(&p->hello->gpu->lock);
	if (!refcount_dec_and_test(&h->refs))
		return;
	if (WARN_ON(!list_empty(&h->garbage)))
		return;
	list_add_tail(&h->garbage, &p->user_native_history_garbage);
	schedule_work(&p->user_reap);
}

/* The actual job of a history as an identity only. A settled history's job
 * is released and its address may be reused by a new job: the history's own
 * (live) address stands in, never equal to any live job, never dereferenced.
 */
static struct gpu_m4_drm_provider_job *m4_user_native_history_job(
	const struct m4_user_native_history *h)
{
	return !h ? NULL : h->job ?: (struct gpu_m4_drm_provider_job *)h;
}

static void m4_user_native_bo_put_locked(struct m4_user_native_bo *b)
{
	struct m4_gpu_provider_runtime *p = b->runtime;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!refcount_dec_and_test(&b->refs))
		return;
	if (b->claim)
		gpu_m4_drm_provider_private_put_locked(b->claim);
	if (b->object)
		gpu_m4_drm_provider_object_put_locked(b->object);
	list_add_tail(&b->garbage, &p->user_native_garbage);
	schedule_work(&p->user_reap);
}

/* Exactly the typed mapped descriptors this unit contributes to a stage's
 * result.native (b->mapping of each claimed BO, cell wrappers included).
 */
static u32 m4_user_native_unit_maps(const struct m4_user_native_unit *u,
	struct gpu_m4_drm_provider_private_gem *maps)
{
	u32 n = 0;

	for (u32 i = 0; i < u->bo_count && i < ARRAY_SIZE(u->bo); i++)
		if (u->bo[i]->va)
			maps[n++] = u->bo[i]->mapping;
	return n;
}

/* Remove a member unit from FUTURE baseline membership only. Older baselines
 * held by composed job versions keep their mappings, objects and claims until
 * normal reaping, or until such a finished version's image is released once a
 * later binding replaced it as the resident root (provider-owned; its receipt
 * keeps the version record). Nothing physical, no residency or claim changes
 * here. Prior detachment is accepted ONLY when the held runtime VM record is
 * closed under this same GPU mutex (its close/destroy detached membership). An
 * open VM's missing member or an empty inventory of a member is a failure (the
 * flag stays). A provider fault is not closure. Removal never grants cell reuse.
 * Fallible: the caller decides before any irreversible change.
 */
static int m4_user_native_baseline_remove_locked(struct m4_user_native_unit *u)
{
	struct gpu_m4_drm_provider_private_gem maps[ARRAY_SIZE(u->bo)];
	struct gpu_m4_drm_provider_vm *vm;
	u64 revision;
	u32 count;
	int ret = -EAGAIN;

	if (!u->baseline && !u->tree)
		return 0;
	lockdep_assert_held(&u->queue->runtime->hello->gpu->lock);
	/* S3c: the hardware mapping first, with the tree's full removal
	 * sequence; a closed VM's tree still exists and still maps them.
	 */
	if (u->tree) {
		vm = u->queue->vm->provider_vm;
		count = m4_user_native_unit_maps(u, maps);
		ret = count ? gpu_m4_drm_provider_vm_native_unmap_locked(vm, maps, count) : -ESTALE;
		if (ret)
			return ret;
		u->tree = false;
		ret = -EAGAIN;
		if (!u->baseline)
			return 0;
	}
	if (u->queue->vm->closed) {
		u->baseline = false;
		return 0;
	}
	vm = u->queue->vm->provider_vm;
	count = m4_user_native_unit_maps(u, maps);
	if (!count)
		return -ESTALE;
	/* The provider mutex is held: a re-read revision cannot race; the retry
	 * only follows the API's revision protocol.
	 */
	for (u32 attempt = 0; ret == -EAGAIN && attempt < 2; attempt++)
		ret = gpu_m4_drm_provider_vm_baseline_remove_locked(vm,
			gpu_m4_drm_provider_vm_baseline_revision_locked(vm), maps, count, &revision);
	if (!ret) {
		u->baseline = false;
		/* REMOVE-DEBUG until S3c: the shadow tree drops the same DRIVER
		 * extents (clear, clean, TLBI, DSB SY); the baseline decided.
		 */
		gpu_m4_drm_provider_vm_tree_native_remove_locked(vm, maps, count);
	}
	return ret;
}

/* Install a NEW unit's complete BO/claim inventory once, after construction
 * (an incomplete unit never becomes a member). Atomic in the provider with
 * revision recheck; any failure leaves the previous baseline and leaves this
 * unit working through the per-child append path. Warm reuse never re-adds.
 */
static int m4_user_native_baseline_add(struct m4_user_native_unit *u)
{
	struct gpu_m4_drm_provider_private_gem maps[ARRAY_SIZE(u->bo)];
	struct m4_gpu_provider_runtime *p = u->queue->runtime;
	struct mutex *lock = &p->hello->gpu->lock;
	struct gpu_m4_drm_provider_vm *vm = u->queue->vm->provider_vm;
	u64 revision;
	u32 count;
	int ret = -EAGAIN;

	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	/* S3c: an authority VM's jobs translate through its tree only: the
	 * unit's maps must be its DRIVER extents, or this unit cannot run.
	 */
	if (gpu_m4_drm_provider_vm_authority(vm) && !u->tree && !p->error) {
		count = m4_user_native_unit_maps(u, maps);
		ret = count ? gpu_m4_drm_provider_vm_native_map_locked(vm, maps, count) : 0;
		if (ret) {
			mutex_unlock(lock);
			return ret;
		}
		u->tree = !!count;
		ret = -EAGAIN;
	}
	count = u->baseline || p->error ? 0 : m4_user_native_unit_maps(u, maps);
	for (u32 attempt = 0; count && ret == -EAGAIN && attempt < 2; attempt++)
		ret = gpu_m4_drm_provider_vm_baseline_add_locked(vm,
			gpu_m4_drm_provider_vm_baseline_revision_locked(vm), maps, count, &revision);
	if (count && !ret) {
		u->baseline = true;
		/* REMOVE-DEBUG until S3c: the unit's client-VA BOs become DRIVER
		 * extents of the shadow tree, mirroring the committed baseline.
		 */
		gpu_m4_drm_provider_vm_tree_native_add_locked(vm, maps, count);
	}
	mutex_unlock(lock);
	return 0;
}

/* Final disposal of an unpublished unit (or a drained rollback tombstone):
 * returns its software cell credits and drops every remaining unit ref.
 */
/* u stops being any slot's latest committed user (cold cut, rollback
 * tombstone, free). A cold unit is never a current owner anyway; this keeps
 * the table from naming freed memory. O(slots), only on these transitions.
 */
static void m4_user_native_slot_forget_locked(struct m4_gpu_provider_runtime *p,
	const struct m4_user_native_unit *u)
{
	for (u32 slot = 0; slot < M4_USER_SLOT_TABLE; slot++)
		if (p->user_slot_unit[slot] == u)
			p->user_slot_unit[slot] = NULL;
}

static void m4_user_native_unit_free_locked(struct m4_gpu_provider_runtime *p,
	struct m4_user_native_unit *u)
{
	/* An arena reservation can precede a failed BO factory. The immutable
	 * unit, not an eventually-created BO, owns that unpublished reservation.
	 */
	for (u32 c = 0; c < M4_ARENA_CELLS; c++)
		if (p->user_cell_owner[c] == u)
			p->user_cell_owner[c] = NULL;
	m4_user_native_slot_forget_locked(p, u);
	for (u32 i = 0; i < u->bo_count; i++) {
		struct m4_user_native_bo *b = u->bo[i];

		if (b->upper)
			gpu_m4_drm_upper_put_locked(b->upper);
		m4_user_native_bo_put_locked(b);
	}
	m4_user_native_history_put_locked(u->history);
	m4_user_native_history_put_locked(u->displacement);
	m4_user_queue_put_locked(u->queue);
	p->user_native_bytes -= u->charge;
	kfree(u);
}

/* Is this claimed wrapper still held by anyone but the unit (a stage), or is
 * its claim no longer this VM's exact registration? Captured versions or an
 * older baseline still holding the identical claim do not hold the cell: the
 * next same-VM owner of the cell adopts that exact claim (see claim_locked).
 */
static bool m4_user_native_claim_held_locked(struct m4_gpu_provider_runtime *p,
	struct m4_user_native_bo *b)
{
	int ret;

	if (!b->claim)
		return false;
	if (refcount_read(&b->refs) != 1)
		return true;
	ret = gpu_m4_drm_provider_private_sole_locked(p->provider, b->vm, b->claim,
		b->va, b->size, 0, GPU_M4_DRM_PAGE_SIZE);
	return ret && ret != -EBUSY;
}

/* Drop a rollback tombstone's non-cell BOs once it no longer is a member
 * (their claims need no software credit; holders drain them normally).
 */
static void m4_user_native_rollback_trim_locked(struct m4_user_native_unit *u)
{
	u32 kept = 0;

	for (u32 i = 0; i < u->bo_count; i++) {
		struct m4_user_native_bo *b = u->bo[i];

		if (b->cell) {
			u->bo[kept++] = b;
			continue;
		}
		if (b->upper)
			gpu_m4_drm_upper_put_locked(b->upper);
		m4_user_native_bo_put_locked(b);
	}
	for (u32 i = kept; i < u->bo_count; i++)
		u->bo[i] = NULL;
	u->bo_count = kept;
}

/* A rolled-back unit whose baseline removal failed, or whose cell claim is
 * still held (e.g. another same-VM queue composed the baseline it joined),
 * must not free its cells: the next unit would hit -EADDRINUSE on the fixed
 * cell VA. Keep it as an unpublished cold rollback tombstone, the strong
 * owner of its cell wrappers and user_cell_owner[] credits, on the unit list.
 * The deferred reaper retries removal, then uses the cell-return seam (no
 * borrower, wrapper drained, exact sole claim after provider reaping), and
 * finally frees it. No CORE receipt is involved or fabricated.
 */
static bool m4_user_native_rollback_defer_locked(struct m4_gpu_provider_runtime *p,
	struct m4_user_native_unit *u)
{
	bool held = !!m4_user_native_baseline_remove_locked(u);
	u32 kept = 0;

	for (u32 i = 0; !held && i < u->bo_count; i++)
		held = u->bo[i]->cell && m4_user_native_claim_held_locked(p, u->bo[i]);
	if (!held)
		return false;
	u->cold = true;
	m4_user_native_slot_forget_locked(p, u);
	if (!u->baseline && !u->tree) {
		/* Keep only held cell wrappers; return any drained credit now. */
		m4_user_native_rollback_trim_locked(u);
		for (u32 i = 0; i < u->bo_count; i++) {
			struct m4_user_native_bo *b = u->bo[i];

			if (m4_user_native_claim_held_locked(p, b)) {
				u->bo[kept++] = b;
				continue;
			}
			for (u32 c = 0; c < M4_ARENA_CELLS; c++)
				if (b->cell == &p->ready->arena->cell[c] && p->user_cell_owner[c] == u)
					p->user_cell_owner[c] = NULL;
			if (b->upper)
				gpu_m4_drm_upper_put_locked(b->upper);
			m4_user_native_bo_put_locked(b);
		}
		for (u32 i = kept; i < u->bo_count; i++)
			u->bo[i] = NULL;
		u->bo_count = kept;
	}
	/* Unbound reserved credits never had a claim. */
	for (u32 c = 0; c < M4_ARENA_CELLS; c++) {
		bool bound = false;

		if (p->user_cell_owner[c] != u)
			continue;
		for (u32 i = 0; i < u->bo_count; i++)
			bound |= u->bo[i]->cell == &p->ready->arena->cell[c];
		if (!bound)
			p->user_cell_owner[c] = NULL;
	}
	refcount_set(&u->refs, 1); /* the unit list's rollback ownership */
	list_add_tail(&u->retained, &p->user_native_units);
	list_add_tail(&u->queue_link, &u->queue->units);
	schedule_work(&p->user_reap);
	return true;
}

static void m4_user_native_unit_put_locked(struct m4_user_native_unit *u)
{
	struct m4_gpu_provider_runtime *p;

	if (!u || !refcount_dec_and_test(&u->refs))
		return;
	p = u->queue->runtime;
	lockdep_assert_held(&p->hello->gpu->lock);
	/* The published baseline is never removed by a job/error/queue close. */
	if (WARN_ON(u->published || !list_empty(&u->retained))) {
		refcount_set(&u->refs, 1);
		return;
	}
	/* A constructed unit of a stage rolled back before publication may be a
	 * member, and its cell claims may still be held by compositions of the
	 * baseline it joined. Then it stays a deferred rollback owner. A cold
	 * unit here is only a test-qualified shutdown of an already drained one.
	 */
	if (!u->cold && p->ready->arena && m4_user_native_rollback_defer_locked(p, u))
		return;
	m4_user_native_unit_free_locked(p, u);
}

static int m4_user_native_unit_create(struct m4_user_native_stage *s, u32 index)
{
	struct m4_gpu_provider_runtime *p = s->queue->runtime;
	struct mutex *lock = &p->hello->gpu->lock;
	struct m4_user_native_unit *u;
	int ret;

	mutex_lock(lock);
	ret = m4_user_native_charge_locked(p, sizeof(*u));
	mutex_unlock(lock);
	if (ret)
		return ret;
	u = kzalloc(sizeof(*u), GFP_KERNEL);
	if (!u) {
		mutex_lock(lock); p->user_native_bytes -= sizeof(*u); mutex_unlock(lock);
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&u->retained);
	INIT_LIST_HEAD(&u->queue_link);
	u->queue = s->queue;
	u->accepted = s;
	u->charge = sizeof(*u);
	refcount_set(&u->refs, 1); /* This stage's membership. */
	mutex_lock(lock);
	if (!refcount_read(&s->queue->refs) || refcount_read(&s->queue->refs) == U32_MAX) {
		p->user_native_bytes -= u->charge;
		ret = -EOVERFLOW;
	} else {
		refcount_inc(&s->queue->refs);
		s->unit[index] = u;
		s->building = u;
	}
	mutex_unlock(lock);
	if (ret)
		kfree(u);
	return ret;
}

static int m4_user_native_backing_get(void *owner)
{
	struct m4_user_native_bo *b = owner;

	lockdep_assert_held(&b->runtime->hello->gpu->lock);
	if (!refcount_read(&b->refs) || refcount_read(&b->refs) == U32_MAX)
		return -EOVERFLOW;
	refcount_inc(&b->refs);
	return 0;
}

static void m4_user_native_backing_put(void *owner)
{
	struct m4_user_native_bo *b = owner;
	struct mutex *lock = &b->runtime->hello->gpu->lock;

	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	m4_user_native_bo_put_locked(b);
	mutex_unlock(lock);
}

static int m4_user_native_backing_page(void *owner, u64 offset, u64 *pa)
{
	struct m4_user_native_bo *b = owner;

	lockdep_assert_held(&b->runtime->hello->gpu->lock);
	if (!refcount_read(&b->refs) || !b->object || !pa || offset >= b->size ||
	    !gpu_m4_drm_aligned(offset))
		return -EPERM;
	return gpu_m4_drm_provider_object_page_locked(b->object, offset, pa);
}

static int m4_user_native_backing_check(void *owner, const void *identity, u64 offset, u64 pa)
{
	struct m4_user_native_bo *b = owner;
	u64 actual;
	int ret;

	if (identity != b->gem || !b->pin || gpu_m4_drm_bo_pin_object(b->pin) != b->gem ||
	    gpu_m4_drm_bo_policy(b->gem) != GPU_M4_DRM_CPU_NC)
		return -EPERM;
	/* The provider object's admitted pin inventory is the page oracle: it
	 * proved the same RAM/firmware/table/MMIO exclusions once for this held
	 * pin and rechecks the live session/error gate on every call.
	 */
	ret = m4_user_native_backing_page(b, offset, &actual);
	if (ret || actual != pa)
		return ret ?: -ESTALE;
	return 0;
}

/* REMOVE-DEBUG: selected-inventory proof statistics, observational only. */
static struct {
	u64 proofs, proof_bos, arm_reused, arm_full;
	u64 receipt_proofs, receipt_reused;
	u64 render_reused, render_full; /* render owner per-BO checks */
} m4_user_native_proof_stats;
static u64 m4_user_native_proof_seq;

/* The pending history whose selected-inventory proof still names exactly this
 * job's admitted tuple: same stage, runtime queue/job, version, snapshot,
 * batch/command records and bytes, and BO/map/extra membership counts.
 */
static struct m4_user_native_history *m4_user_native_access_proof_locked(
	struct m4_gpu_provider_runtime *p, struct gpu_m4_drm_provider_job *job)
{
	struct gpu_m4_drm_provider_job_view view = { };
	struct m4_user_native_stage *s;
	struct m4_user_native_history *h;

	gpu_m4_drm_provider_job_view(job, &view);
	s = view.stage_owner;
	h = s ? s->pending : NULL;
	if (!h || !h->access_proof || h->job != job || h->owner_stage != s || h->complete ||
	    !s->queue || s->queue->runtime != p || view.runtime_queue != s->queue ||
	    view.runtime_job != h->admitted_runtime_job || view.version != h->version ||
	    view.snapshot != h->snapshot || !view.batch || view.batch != h->admitted_batch ||
	    view.batch->commands != h->admitted_commands ||
	    view.command_bytes != h->admitted_command_bytes ||
	    view.command_size != h->admitted_command_size ||
	    h->admitted_bo_count != s->bo_count ||
	    h->admitted_native_count != s->native_count ||
	    h->admitted_extra_count != s->extra_count)
		return NULL;
	return h;
}

/* Current strong STARTING job (the newest admitted lane owner), never a cached
 * protected flag/job pointer in the persistent upper owner. The root gate
 * independently proves exact lane/item.
 * A BO admitted by THIS job's selected-inventory proof reuses its immutable
 * object/coverage/cache/resv membership; the current lane, latched session
 * error and the actual job's PUBLISHED/protected phase are checked live.
 */
static int m4_user_native_bo_arm_locked(struct m4_user_native_bo *b,
	struct gpu_m4_drm_provider_job *job)
{
	struct m4_gpu_provider_runtime *p = b->runtime;
	struct m4_user_native_history *h;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!job || job != p->user_starting.job || !b->object || !refcount_read(&b->refs))
		return -EPERM;
	h = b->access_proof ? m4_user_native_access_proof_locked(p, job) : NULL;
	if (h && b->access_proof == h->access_proof) {
		if (p->error || p->ready->error)
			return p->error ?: p->ready->error;
		m4_user_native_proof_stats.arm_reused++;
		return gpu_m4_drm_provider_job_check_execution_locked(job, p->provider, true);
	}
	m4_user_native_proof_stats.arm_full++;
	return gpu_m4_drm_provider_job_check_access_locked(job, p->provider,
		b->object, 0, b->coverage, M4_USER_NATIVE_RW, false, NULL);
}

static int m4_user_native_backing_ready(void *owner, u64 offset, u64 size,
	enum gpu_m4_drm_upper_class class)
{
	struct m4_user_native_bo *b = owner;

	if (offset || size != b->size ||
	    class != (b->va ? GPU_M4_DRM_UPPER_SHARED : GPU_M4_DRM_UPPER_FW_ONLY))
		return -EPERM;
	return m4_user_native_bo_arm_locked(b, b->runtime->user_starting.job);
}

/* Settlement A/B switch: a complete history is proven once, then releases
 * its job and its stage's references to the units' BOs (see history).
 */
static bool native_settle = true;
module_param(native_settle, bool, 0644);
MODULE_PARM_DESC(native_settle, "Release a retired window's job and stage BO references once its receipts are proven (default 1)");

static void m4_user_native_settle(struct m4_gpu_provider_runtime *p);

/* Collector ONLY: never frontend/provider/reservation serialized. Native BO
 * put queued the accounting owner; provider_reap follows after NC unmap/GEM put.
 */
static void m4_user_native_reap(struct m4_gpu_provider_runtime *p)
{
	struct mutex *lock = &p->hello->gpu->lock;
	struct m4_user_native_bo *b;
	struct m4_user_native_history *h;
	int ret;

	lockdep_assert_not_held(lock);
	m4_user_native_settle(p);
	for (;;) {
		mutex_lock(lock);
		if (list_empty(&p->user_native_history_garbage)) {
			mutex_unlock(lock);
			break;
		}
		h = list_first_entry(&p->user_native_history_garbage,
			struct m4_user_native_history, garbage);
		list_del_init(&h->garbage);
		mutex_unlock(lock);
		ret = h->job ? gpu_m4_drm_provider_job_put(h->job) : 0;
		if (ret) {
			/* Unexpected uncertainty remains reachable; no busy retry loop. */
			mutex_lock(lock);
			list_add_tail(&h->garbage, &p->user_native_history_garbage);
			mutex_unlock(lock);
			break;
		}
		m4_user_native_stage_put(p, h->replacement);
		m4_user_native_stage_put(p, h->release_stage);
		for (u32 i = 0; i < h->spare_refs; i++)
			m4_user_native_stage_put(p, h->owner_stage);
		for (u32 i = 0; i < 3; i++)
			if (h->last_mask & (1U << i))
				m4_user_native_stage_put(p, h->owner_stage);
		if (h->stage_held)
			m4_user_native_stage_put(p, h->owner_stage);
		mutex_lock(lock);
		p->user_native_bytes -= h->charge;
		mutex_unlock(lock);
		kvfree(h);
	}
	for (;;) {
		mutex_lock(lock);
		if (list_empty(&p->user_native_garbage)) {
			mutex_unlock(lock);
			break;
		}
		b = list_first_entry(&p->user_native_garbage, struct m4_user_native_bo, garbage);
		list_del_init(&b->garbage);
		mutex_unlock(lock);
		if (b->mapped && !b->borrowed_map)
			drm_gem_vunmap(b->gem, &b->map);
		if (b->gem)
			drm_gem_object_put(b->gem);
		mutex_lock(lock);
		p->user_native_bytes -= b->charge;
		mutex_unlock(lock);
		kfree(b);
	}
}

/* Search the actual provider's held guarded intervals, then acquire the exact
 * claim under the same mutex. Fixed arena cells are never silently relocated.
 */
static int m4_user_native_claim_locked(struct m4_user_native_stage *s,
	struct m4_user_native_bo *b, u64 alignment, u64 fixed_va)
{
	u64 va = fixed_va;
	struct gpu_m4_drm_provider_private_lease *lease;
	int ret;

	if (!fixed_va) {
		ret = gpu_m4_drm_provider_private_find_locked(b->vm, s->next_va, b->size,
			alignment, 0, GPU_M4_DRM_PAGE_SIZE, &va);
		if (ret)
			return ret;
	}
	lease = gpu_m4_drm_provider_private_reserve_locked(b->vm, va, b->size,
		0, GPU_M4_DRM_PAGE_SIZE);
	/* A fixed arena cell VA is exclusively this unit's (its cell credit). A
	 * same-VM claim still registered there belongs only to a drained former
	 * wrapper of the same cell, kept by immutable captured versions (e.g. the
	 * resident root) that map the identical cell object: share that exact
	 * reservation instead of waiting for a future root to drop it.
	 */
	if (fixed_va && IS_ERR(lease) && PTR_ERR(lease) == -EADDRINUSE)
		lease = gpu_m4_drm_provider_private_adopt_locked(b->vm, va, b->size,
			0, GPU_M4_DRM_PAGE_SIZE);
	if (IS_ERR(lease))
		return PTR_ERR(lease);
	b->claim = lease;
	b->va = va;
	b->mapping = (struct gpu_m4_drm_provider_private_gem) {
		.object = b->object, .lease = lease, .va = va,
		.size = b->size, .access = M4_USER_NATIVE_RW,
	};
	if (!fixed_va)
		s->next_va = va + b->size + GPU_M4_DRM_PAGE_SIZE;
	return 0;
}

/* Queue creation (under the frontend VM bind lock), synchronous submit staging
 * (under the logical queue's submission gate) and a dependency-ready
 * execution window all run WITHOUT the frontend mutex, so every one uses the
 * locking factory: its owned charge/VM/DRM reservation is a short frontend
 * section and shmem construction runs outside it. No GPU/reservation mutex
 * spans the factory or pin/vmap. The returned record transfers even on error,
 * so rollback never loses owners.
 */
static bool native_rezero;
module_param(native_rezero, bool, 0644);
MODULE_PARM_DESC(native_rezero, "Zero each new native backing again through its NC mapping (default 0: shmem pages are already zero)");

static int m4_user_native_bo_create(struct m4_user_queue *q, u64 size,
	struct m4_command_cell *cell, bool deferred, struct m4_user_native_bo **out,
	bool *backing_refused)
{
	struct m4_gpu_provider_runtime *p = q->runtime;
	struct mutex *lock = &p->hello->gpu->lock;
	struct m4_user_native_bo *b;
	u64 charge;
	int ret;

	lockdep_assert_not_held(lock);
	if (!out || !size || size > M4_USER_NATIVE_LIMIT ||
	    size > U64_MAX - GPU_M4_DRM_PAGE_SIZE + 1)
		return -EINVAL;
	size = ALIGN(size, GPU_M4_DRM_PAGE_SIZE);
	charge = sizeof(*b) + (cell ? 0 : size);
	mutex_lock(lock);
	ret = p->error ?: (!p->ordinary || q->closed || q->vm->closed ? -ESHUTDOWN : 0);
	if (!ret) {
		/* Its only refusal is the runtime native quota. */
		ret = m4_user_native_charge_locked(p, charge);
		*backing_refused = !!ret;
	}
	mutex_unlock(lock);
	if (ret)
		return ret;
	b = kzalloc(sizeof(*b), GFP_KERNEL);
	if (!b) {
		mutex_lock(lock);
		p->user_native_bytes -= charge;
		mutex_unlock(lock);
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&b->garbage);
	b->runtime = p;
	b->vm = q->vm->provider_vm;
	b->size = b->coverage = size;
	b->charge = charge;
	b->cell = cell;
	refcount_set(&b->refs, 1);
	*out = b;
	if (cell) {
		if (size != GPU_M4_DRM_PAGE_SIZE)
			return -EINVAL;
		b->gem = cell->gem;
		drm_gem_object_get(b->gem);
	} else {
		(void)deferred; /* both entrances are outside the frontend mutex */
		b->gem = gpu_m4_drm_bo_create_native(p->frontend, size,
			GPU_M4_DRM_CPU_NC, q->vm->frontend_vm);
		if (IS_ERR(b->gem)) {
			ret = PTR_ERR(b->gem);
			b->gem = NULL;
			/* The factory's -ENOSPC is exactly its native quota; shmem
			 * construction failure is host memory (-ENOMEM).
			 */
			*backing_refused = ret == -ENOSPC;
			return ret;
		}
	}
	b->object = gpu_m4_drm_provider_object_pin_deferred(b->vm, b->gem);
	if (IS_ERR(b->object)) {
		ret = PTR_ERR(b->object);
		b->object = NULL;
		return ret;
	}
	b->pin = gpu_m4_drm_provider_object_native(b->object);
	if (!b->pin || gpu_m4_drm_bo_pin_object(b->pin) != b->gem ||
	    gpu_m4_drm_bo_pin_size(b->pin) != size ||
	    gpu_m4_drm_bo_policy(b->gem) != GPU_M4_DRM_CPU_NC)
		return -EPERM;
	if (cell) {
		b->map = cell->map;
		b->mapped = b->borrowed_map = true;
	} else {
		ret = drm_gem_vmap(b->gem, &b->map); /* pin prepared NC before first view */
		if (ret)
			return ret;
		b->mapped = true;
	}
	if (iosys_map_is_null(&b->map) || b->map.is_iomem ||
	    ((unsigned long)b->map.vaddr & (GPU_M4_DRM_PAGE_SIZE - 1)))
		return -EPERM;
	/* A fresh shmem backing is already zero (its pages are cleared at
	 * allocation) and the pin's NC preparation cleaned them to PoC before
	 * this first NC view; rewriting it through the NC mapping only costs
	 * construction time (native_rezero restores it).
	 */
	if (!cell && READ_ONCE(native_rezero)) {
		memset(b->map.vaddr, 0, size);
		dma_wmb();
	}
	b->backing = (struct gpu_m4_drm_upper_backing) {
		.owner = b, .identity = b->gem, .size = size, .policy = GPU_M4_DRM_CPU_NC,
		.get_locked = m4_user_native_backing_get, .put = m4_user_native_backing_put,
		.page_locked = m4_user_native_backing_page, .check_locked = m4_user_native_backing_check,
		.ready_locked = m4_user_native_backing_ready,
	};
	return 0;
}

static int m4_user_native_upper_locked(struct m4_user_native_bo *b)
{
	struct gpu_m4_drm_upper_lease *lease;
	int ret;

	if (b->cell) {
		ret = gpu_m4_drm_upper_reference_locked(b->cell->upper, &b->cell->backing,
			GPU_M4_DRM_UPPER_NORMAL, GPU_M4_DRM_UPPER_SHARED);
		if (ret)
			return ret;
		b->backing = b->cell->backing;
		lease = b->cell->upper;
	} else {
		lease = gpu_m4_drm_upper_accept_locked(b->runtime->ready->upper, &b->backing,
			GPU_M4_DRM_UPPER_NORMAL,
			b->va ? GPU_M4_DRM_UPPER_SHARED : GPU_M4_DRM_UPPER_FW_ONLY);
		if (IS_ERR(lease))
			return PTR_ERR(lease);
	}
	b->upper = lease;
	b->fw_va = gpu_m4_drm_upper_va_locked(lease);
	return b->fw_va ? 0 : -ESTALE;
}

/* The ONE stage extra of an unmapped native BO, identical for its fresh
 * construction and every warm whole-unit reuse: exact RW8 only for a pure
 * submitted counter; otherwise the whole declared firmware-metadata coverage.
 */
static struct gpu_m4_drm_provider_stage_extra m4_user_native_extra(const struct m4_user_native_bo *b)
{
	return (struct gpu_m4_drm_provider_stage_extra) {
		.object = b->object, .size = b->coverage, .access = M4_USER_NATIVE_RW,
		.purpose = b->coverage == 8 ? GPU_M4_DRM_PROVIDER_STAGE_SUBMITTED_COUNTER :
			GPU_M4_DRM_PROVIDER_STAGE_FIRMWARE_METADATA,
	};
}

/* Prebuilt private pools: a queue whose task name recently built N fresh
 * units gets N unclaimed 32 MiB pools built in the background while the
 * client loads, so its first frames do not allocate, clear and clean them.
 * Bounded by the recorded count, prebuild_max and native headroom; freed at
 * queue close.
 */
static u64 m4_user_native_bytes_free_locked(struct m4_gpu_provider_runtime *p);
static u64 m4_user_native_prebuilt; /* REMOVE-DEBUG: pools prebuilt */
static unsigned int prebuild_max = 96;
module_param(prebuild_max, uint, 0644);
MODULE_PARM_DESC(prebuild_max, "Prebuilt 32 MiB unit pools per new queue, from its task's previous high-water (0 = off)");

static struct m4_user_native_bo *m4_user_native_spare_take(struct m4_user_queue *q, u64 size)
{
	struct mutex *lock = &q->runtime->hello->gpu->lock;
	struct m4_user_native_bo *b, *found = NULL;
	struct m4_user_queue *from[2] = { q, NULL };

	mutex_lock(lock);
	/* Its own spares, then its VM's prebuilding sibling's (same VM). */
	if (q->vm->prebuilder && q->vm->prebuilder != q && !q->vm->prebuilder->closed)
		from[1] = q->vm->prebuilder;
	for (u32 i = 0; !found && i < ARRAY_SIZE(from) && from[i]; i++)
		list_for_each_entry(b, &from[i]->spare, garbage)
			if (b->size == size && b->vm == q->vm->provider_vm) {
				found = b;
				list_del_init(&b->garbage);
				from[i]->spares--;
				break;
			}
	mutex_unlock(lock);
	return found;
}

static void m4_user_native_spares_free_locked(struct m4_user_queue *q)
{
	lockdep_assert_held(&q->runtime->hello->gpu->lock);
	while (!list_empty(&q->spare)) {
		struct m4_user_native_bo *b = list_first_entry(&q->spare,
			struct m4_user_native_bo, garbage);

		list_del_init(&b->garbage);
		q->spares--;
		m4_user_native_bo_put_locked(b);
	}
}

static void m4_user_native_prebuild_record_locked(struct m4_user_queue *q)
{
	struct m4_gpu_provider_runtime *p = q->runtime;
	u32 i, slot = ARRAY_SIZE(p->user_prebuild);

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!q->need_pool && !q->need_heap)
		return;
	for (i = 0; i < ARRAY_SIZE(p->user_prebuild); i++)
		if (!strncmp(p->user_prebuild[i].comm, q->comm, sizeof(p->user_prebuild[i].comm)))
			slot = i;
	if (slot == ARRAY_SIZE(p->user_prebuild)) {
		slot = p->user_prebuild_next++ % ARRAY_SIZE(p->user_prebuild);
		memcpy(p->user_prebuild[slot].comm, q->comm, sizeof(p->user_prebuild[slot].comm));
		p->user_prebuild[slot].comm[sizeof(p->user_prebuild[slot].comm) - 1] = 0;
	}
	/* Never shrink below the last need at once: a queue served from
	 * spares still asks for the same backings.
	 */
	p->user_prebuild[slot].units = q->need_pool > p->user_prebuild[slot].units / 2 ?
		q->need_pool : p->user_prebuild[slot].units / 2;
	p->user_prebuild[slot].render = q->need_heap > p->user_prebuild[slot].render / 2 ?
		q->need_heap : p->user_prebuild[slot].render / 2;
}

static void m4_user_native_prebuild_start_locked(struct m4_user_queue *q)
{
	struct m4_gpu_provider_runtime *p = q->runtime;
	u32 want = 0, heaps = 0;

	lockdep_assert_held(&p->hello->gpu->lock);
	for (u32 i = 0; i < ARRAY_SIZE(p->user_prebuild); i++)
		if (!strncmp(p->user_prebuild[i].comm, q->comm, sizeof(p->user_prebuild[i].comm))) {
			want = p->user_prebuild[i].units;
			heaps = p->user_prebuild[i].render;
		}
	/* One prebuilding queue per VM; its siblings take from its spares. */
	if (q->vm->prebuilder && !q->vm->prebuilder->closed)
		return;
	q->prebuild_want = want < READ_ONCE(prebuild_max) ? want : READ_ONCE(prebuild_max);
	q->prebuild_heaps = heaps < READ_ONCE(prebuild_max) ? heaps : READ_ONCE(prebuild_max);
	if (!q->prebuild_want || q->closed || refcount_read(&q->refs) == U32_MAX)
		return;
	q->vm->prebuilder = q;
	refcount_inc(&q->refs);
	if (!queue_work(system_unbound_wq, &q->prebuild))
		m4_user_queue_put_locked(q);
}

static void m4_user_native_prebuild_work(struct work_struct *work)
{
	struct m4_user_queue *q = container_of(work, struct m4_user_queue, prebuild);
	struct m4_gpu_provider_runtime *p = q->runtime;
	struct mutex *lock = &p->hello->gpu->lock;

	/* Pools first (every unit), then render parameter heaps at the device
	 * high-water size new render units are built to.
	 */
	u32 built = 0, heaps = 0;
	u64 started __attribute__((unused)) = ktime_get_ns(), free_bytes = 0;
	int last __attribute__((unused)) = 0;

	for (;;) {
		struct m4_user_native_bo *b = NULL;
		bool refused = false, stop;
		u64 size = GPU_M4_EXEC_POOL_SIZE;
		int ret;

		mutex_lock(lock);
		if (built >= q->prebuild_want) {
			size = READ_ONCE(p->user_render_highwater[0][2]);
			if (heaps >= q->prebuild_heaps || !size || !READ_ONCE(unit_highwater))
				size = 0;
		}
		free_bytes = m4_user_native_bytes_free_locked(p);
		stop = !size || q->closed || p->error ||
			free_bytes < M4_ORDINARY_NATIVE_MEMORY_LIMIT / 4 + size;
		mutex_unlock(lock);
		if (stop)
			break;
		if (size == GPU_M4_EXEC_POOL_SIZE)
			built++;
		else
			heaps++;
		ret = m4_user_native_bo_create(q, size, NULL, true, &b, &refused);
		mutex_lock(lock);
		if (!ret && !q->closed) {
			list_add_tail(&b->garbage, &q->spare);
			q->spares++;
			m4_user_native_prebuilt++;
			b = NULL;
		}
		if (b)
			m4_user_native_bo_put_locked(b);
		mutex_unlock(lock);
		last = ret;
		if (ret)
			break;
	}
#ifdef MODULE
	/* REMOVE-DEBUG: what the prebuild delivered and why it stopped. */
	pr_info_ratelimited("GPU prebuild: comm=%s pools=%u/%u heaps=%u/%u free=%llu MiB error=%d %llu ms\n",
		q->comm, built, q->prebuild_want, heaps, q->prebuild_heaps, free_bytes >> 20, last,
		(ktime_get_ns() - started) / 1000000);
#endif
	mutex_lock(lock);
	m4_user_queue_put_locked(q);
	mutex_unlock(lock);
}

static int m4_user_native_add(struct m4_user_native_stage *s, u64 size,
	u64 alignment, bool mapped, bool firmware, bool counter, bool work,
	struct m4_user_native_bo **out)
{
	struct m4_gpu_provider_runtime *p = s->queue->runtime;
	struct mutex *lock = &p->hello->gpu->lock;
	struct m4_command_cell *cell = NULL;
	struct m4_user_native_bo *b = NULL;
	const char *capacity_phase = "claim";
	int ret;

	if (s->bo_count == M4_USER_NATIVE_BO_MAX ||
	    (mapped && s->native_count == M4_USER_NATIVE_MAP_MAX) ||
	    (!mapped && s->extra_count == M4_USER_NATIVE_EXTRA_MAX) ||
	    (s->building && s->building->bo_count == ARRAY_SIZE(s->building->bo)))
		return -E2BIG;
	if (work) {
		/* Use one of the building unit's credits reserved atomically for
		 * the whole batch; never claim an extra free cell here.
		 */
		mutex_lock(lock);
		if (!p->ready->arena || !p->ready->arena->imported || !s->building) {
			mutex_unlock(lock);
			return -EPERM;
		}
		for (u32 i = 0; !cell && i < M4_ARENA_CELLS; i++) {
			bool bound = false;

			if (p->user_cell_owner[i] != s->building)
				continue;
			for (u32 k = 0; k < s->building->bo_count; k++)
				bound |= s->building->bo[k]->cell == &p->ready->arena->cell[i];
			if (!bound)
				cell = &p->ready->arena->cell[i];
		}
		mutex_unlock(lock);
		if (!cell) {
			pr_warn("GPU native capacity: no reserved cell commands=%u bo_count=%u\n",
				s->count, s->bo_count);
			return -ENOSPC;
		}
	}
	s->backing_refused = false;
	if (!cell && size >= GPU_M4_RENDER_PARAMETER_MIN) {
		b = m4_user_native_spare_take(s->queue, ALIGN(size, GPU_M4_DRM_PAGE_SIZE));
		mutex_lock(lock);
		if (ALIGN(size, GPU_M4_DRM_PAGE_SIZE) == GPU_M4_EXEC_POOL_SIZE)
			s->queue->need_pool++;
		else
			s->queue->need_heap++;
		s->queue->spare_miss += !b;
		mutex_unlock(lock);
	}
	ret = b ? 0 : m4_user_native_bo_create(s->queue, size, cell, s->deferred_materialization,
		&b, &s->backing_refused);
	if (b) {
		s->bo[s->bo_count++] = b;
		s->allocation_bytes += b->charge;
		*out = b;
		if (s->building) {
			mutex_lock(lock);
			b->unit = s->building;
			refcount_inc(&b->refs);
			s->building->bo[s->building->bo_count++] = b;
			mutex_unlock(lock);
		}
	}
	if (ret) {
		if (ret == -ENOSPC || ret == -ENOMEM) {
			/* REMOVE-DEBUG: the constructor includes runtime charging and
			 * pin/map setup, not just the GEM factory. Sample both budgets
			 * together; a failed cold stage is released after this return.
			 */
			mutex_lock(lock);
			pr_warn("GPU native capacity: constructor size=%llu gem_bytes=%llu runtime_bytes=%llu stage_bytes=%llu commands=%u bo_count=%u error=%d\n",
				size, gpu_m4_drm_private_bytes(p->frontend),
				p->user_native_bytes, s->allocation_bytes, s->count,
				s->bo_count, ret);
			mutex_unlock(lock);
		}
		return ret;
	}
	if (counter)
		b->coverage = 8;
	mutex_lock(lock);
	if (mapped)
		ret = m4_user_native_claim_locked(s, b, alignment, cell ? cell->va : 0);
	if (!ret && firmware) {
		capacity_phase = "upper";
		ret = m4_user_native_upper_locked(b);
	}
	if (!ret && mapped)
		s->native[s->native_count++] = b->mapping;
	if (!ret && !mapped)
		s->extra[s->extra_count++] = m4_user_native_extra(b);
	/* REMOVE-DEBUG: only diagnose the existing rejected owner. */
	if (ret == -ENOSPC || ret == -ENOMEM)
		pr_warn("GPU native capacity: phase=%s size=%llu mapped=%u firmware=%u bo_count=%u native_count=%u extra_count=%u next_va=%#llx\n",
			capacity_phase, size, mapped, firmware, s->bo_count,
			s->native_count, s->extra_count, s->next_va);
	mutex_unlock(lock);
	return ret;
}

static int m4_user_native_bo_check_locked(struct m4_user_native_bo *b,
	struct gpu_m4_drm_provider *provider, struct gpu_m4_drm_provider_version *version)
{
	u64 pa, generation;
	int ret;

	if (!b || b->runtime->provider != provider || !refcount_read(&b->refs) ||
	    !b->mapped || !b->gem || !b->object || !b->pin ||
	    !b->size || !gpu_m4_drm_aligned(b->size) ||
	    !b->coverage || b->coverage > b->size ||
	    iosys_map_is_null(&b->map) || b->map.is_iomem ||
	    ((unsigned long)b->map.vaddr & (GPU_M4_DRM_PAGE_SIZE - 1)) ||
	    gpu_m4_drm_provider_object_identity(b->object) != b->gem ||
	    gpu_m4_drm_provider_object_native(b->object) != b->pin ||
	    gpu_m4_drm_bo_pin_object(b->pin) != b->gem ||
	    gpu_m4_drm_bo_pin_size(b->pin) != b->size ||
	    (!!b->upper != !!b->fw_va) || (!!b->claim != !!b->va) ||
	    gpu_m4_drm_bo_policy(b->gem) != GPU_M4_DRM_CPU_NC)
		return M4_USER_WHY(-ESTALE, "native bo provider=%d refs=%u mapped=%d size=%#llx coverage=%#llx upper=%d fw=%#llx claim=%d va=%#llx",
			b && b->runtime->provider == provider, b ? refcount_read(&b->refs) : 0,
			b && b->mapped, M4_WHY64(b ? b->size : 0), M4_WHY64(b ? b->coverage : 0),
			b && b->upper, M4_WHY64(b ? b->fw_va : 0), b && b->claim, M4_WHY64(b ? b->va : 0));
	generation = gpu_m4_drm_provider_version_generation(version);
	if (b->checked_version == version && b->checked_generation == generation && generation) {
		/* Immutable BO identity/claim/manifest proven against this same
		 * version: only the object's live session/admission gate.
		 */
		ret = gpu_m4_drm_provider_object_page_locked(b->object, 0, &pa);
		return M4_USER_WHY(ret, "native bo gate va=%#llx", M4_WHY64(b->va));
	}
	b->checked_version = NULL;
	ret = gpu_m4_drm_provider_object_origin_locked(provider, version, b->object);
	if (!ret && b->va && (b->mapping.object != b->object || b->mapping.lease != b->claim ||
	    b->mapping.va != b->va || b->mapping.offset || b->mapping.size != b->size ||
	    b->mapping.access != M4_USER_NATIVE_RW))
		ret = M4_USER_WHY(-ESTALE, "native bo %#llx mapping va=%#llx offset=%#llx size=%#llx access=%u",
			M4_WHY64(b->va), M4_WHY64(b->mapping.va), M4_WHY64(b->mapping.offset),
			M4_WHY64(b->mapping.size), b->mapping.access);
	if (!ret && b->va)
		ret = gpu_m4_drm_provider_private_check_locked(b->claim, b->vm, b->va, b->size);
	/* Admitted native inventory plus live gate; no repeated PA oracle. A
	 * mapped BO's installed pages are one sequential run (O(manifest blocks)
	 * lookups), memoized per exact span of the immutable manifest shared by
	 * later jobs; its first page's live gate runs every call.
	 */
	if (!ret && b->va)
		ret = gpu_m4_drm_provider_version_object_locked(version, b->va, b->object,
			b->gem, 0, b->size, M4_USER_NATIVE_RW, true);
	for (u64 offset = 0; !ret && !b->va && offset < b->size; offset += GPU_M4_DRM_PAGE_SIZE)
		ret = gpu_m4_drm_provider_object_page_locked(b->object, offset, &pa);
	if (!ret && b->cell) {
		ret = gpu_m4_drm_provider_object_page_locked(b->object, 0, &pa);
		if (!ret && (b->size != GPU_M4_DRM_PAGE_SIZE || pa != b->cell->pa ||
		    b->va != b->cell->va || b->map.vaddr != b->cell->map.vaddr ||
		    b->upper != b->cell->upper || b->fw_va != b->cell->fw_va))
			ret = M4_USER_WHY(-ESTALE, "native bo cell %#llx pa=%#llx/%#llx va=%#llx/%#llx upper=%d",
				M4_WHY64(b->fw_va), M4_WHY64(pa), M4_WHY64(b->cell->pa), M4_WHY64(b->va),
				M4_WHY64(b->cell->va), b->upper == b->cell->upper);
	}
	if (!ret && b->upper && gpu_m4_drm_upper_va_locked(b->upper) != b->fw_va)
		ret = M4_USER_WHY(-ESTALE, "native bo alias %#llx lease %#llx", M4_WHY64(b->fw_va),
			M4_WHY64(gpu_m4_drm_upper_va_locked(b->upper)));
	if (!ret) {
		b->checked_version = version;
		b->checked_generation = generation;
	}
	return M4_USER_WHY(ret, "native bo va=%#llx fw=%#llx size=%#llx", M4_WHY64(b->va),
		M4_WHY64(b->fw_va), M4_WHY64(b->size));
}

static struct m4_user_notifier *m4_user_notifier_create(struct m4_user_queue *q)
{
	struct m4_gpu_provider_runtime *p = q->runtime;
	struct mutex *lock = &p->hello->gpu->lock;
	struct m4_user_notifier *n;
	bool backing_refused;
	int ret;

	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	ret = m4_user_native_charge_locked(p, sizeof(*n));
	/* Its records name the VM's context: hold it while they exist. */
	if (!ret) {
		ret = m4_user_asid_get_locked(p, q->vm->asid);
		if (ret)
			p->user_native_bytes -= sizeof(*n);
	}
	mutex_unlock(lock);
	if (ret)
		return ERR_PTR(ret);
	n = kzalloc(sizeof(*n), GFP_KERNEL);
	if (!n) {
		mutex_lock(lock);
		p->user_native_bytes -= sizeof(*n);
		m4_user_asid_put_locked(p, q->vm->asid);
		mutex_unlock(lock);
		return ERR_PTR(-ENOMEM);
	}
	n->charge = sizeof(*n);
	n->asid = q->vm->asid;
	refcount_set(&n->refs, 1);
	/* Queue creation has no backpressure: any refusal keeps its errno. */
	ret = m4_user_native_bo_create(q, GPU_M4_DRM_PAGE_SIZE, NULL, false, &n->bo,
		&backing_refused);
	if (!ret) {
		mutex_lock(lock);
		ret = m4_user_native_upper_locked(n->bo);
		mutex_unlock(lock);
	}
	if (ret) {
		if (n->bo) {
			mutex_lock(lock);
			if (n->bo->upper)
				gpu_m4_drm_upper_put_locked(n->bo->upper);
			m4_user_native_bo_put_locked(n->bo);
			mutex_unlock(lock);
		}
		mutex_lock(lock);
		p->user_native_bytes -= n->charge;
		m4_user_asid_put_locked(p, n->asid);
		mutex_unlock(lock);
		kfree(n);
		return ERR_PTR(ret);
	}
	/* Logical queues get their OWN NC record/stamps, each naming the VM's
	 * user context once for the notifier's lifetime. Physical queues/
	 * context/list remain session owners.
	 */
	for (u32 i = 0; i < 3; i++) {
		u8 *record = n->bo->map.vaddr + i * 0x200;

		put_unaligned_le64(n->bo->fw_va + 0x600 + i * 8, record);
		put_unaligned_le32(0x50, record + 0x10);
		put_unaligned_le32(n->asid, record + 0x24);
		memset(record + 0xa8, 0xff, 8);
		put_unaligned_le32(1, record + 0x114);
	}
	dma_wmb();
	return n;
}

static int m4_user_notifier_get_locked(struct m4_user_notifier *n)
{
	if (!n || !refcount_read(&n->refs) || refcount_read(&n->refs) == U32_MAX)
		return -EOVERFLOW;
	refcount_inc(&n->refs);
	return 0;
}

static void m4_user_notifier_put_locked(struct m4_user_notifier *n)
{
	struct m4_gpu_provider_runtime *p;

	if (!n || !refcount_dec_and_test(&n->refs))
		return;
	p = n->bo->runtime;
	if (n->bo->upper)
		gpu_m4_drm_upper_put_locked(n->bo->upper);
	m4_user_native_bo_put_locked(n->bo);
	p->user_native_bytes -= n->charge;
	m4_user_asid_put_locked(p, n->asid);
	kfree(n); /* No reservation/frontend/native final drop. */
}

static int m4_user_native_initial_range(void *cookie, u64 va, u64 size, bool write)
{
	struct m4_user_native_stage *s = cookie;

	if (!write || !size)
		return -EINVAL;
	for (u32 i = 0; i < s->bo_count; i++) {
		struct m4_user_native_bo *b = s->bo[i];

		if (b->va && va >= b->va && va - b->va < b->size && size <= b->size - (va - b->va))
			return gpu_m4_drm_provider_private_check_locked(b->claim, b->vm, va, size);
	}
	return -EACCES;
}

static int m4_user_native_compute_allocate(struct m4_user_native_stage *s, u32 index)
{
	static const u64 size[GPU_M4_COMPUTE_PRIVATE_COUNT] = {
		0x4000, 0x4000, 0x4000, 0x40000, 0x20000, 0x4000, GPU_M4_EXEC_POOL_SIZE,
	};
	struct m4_user_compute_native *c = &s->command[index].compute;
	struct gpu_m4_exec_vm vm = { .start = GPU_M4_DRM_UAT_PRIVATE_START,
		.end = GPU_M4_DRM_UAT_PRIVATE_END, .cookie = s, .check = m4_user_native_initial_range };
	struct mutex *lock = &s->queue->runtime->hello->gpu->lock;
	int ret;

	for (u32 i = 0; i < GPU_M4_COMPUTE_PRIVATE_COUNT; i++) {
		ret = m4_user_native_add(s, size[i], GPU_M4_DRM_PAGE_SIZE, true,
			i == GPU_M4_COMPUTE_PRIVATE_WORK || i == GPU_M4_COMPUTE_PRIVATE_AUX,
			false, i == GPU_M4_COMPUTE_PRIVATE_WORK, &c->private[i]);
		if (ret)
			return ret;
	}
	/* Combined firmware-only counter/wait page, staged as ONE whole-page NC
	 * firmware-metadata extra (RW, 0x4000) before cache/resv construction:
	 * the submitted counter stays at [0,8) and a strict prior-command wait
	 * record lives at [GPU_M4_COMPUTE_PACKET_WAIT_OFFSET, +0x40). The narrow
	 * RW8 SUBMITTED_COUNTER purpose could not cover that record.
	 */
	ret = m4_user_native_add(s, GPU_M4_DRM_PAGE_SIZE, GPU_M4_DRM_PAGE_SIZE,
		false, true, false, false, &c->counter);
	if (ret)
		return ret;
	mutex_lock(lock);
	ret = gpu_m4_exec_uma_pool(c->private[GPU_M4_COMPUTE_PRIVATE_POOL_STATE]->map.vaddr,
		c->private[GPU_M4_COMPUTE_PRIVATE_PAGE_LIST]->map.vaddr, &vm,
		c->private[GPU_M4_COMPUTE_PRIVATE_POOL]->va);
	mutex_unlock(lock);
	dma_wmb();
	return ret;
}

/* GPU backing is a complete typed private mapping. Firmware backing retains
 * the same actual native object and its canonical lease, not a PA callback.
 * Logical firmware subrecords are inside that full-page cache owner.
 */
static struct gpu_m4_render_native_backing m4_user_native_gpu(
	struct m4_user_native_bo *b)
{
	return (struct gpu_m4_render_native_backing) {
		.span = { b->va, b->size }, .kind = GPU_M4_RENDER_NATIVE_GEM, .gem = b->mapping,
	};
}

static struct gpu_m4_render_native_backing m4_user_native_fw(
	struct m4_user_native_bo *b, u64 offset, u64 size)
{
	return (struct gpu_m4_render_native_backing) {
		.span = { b->fw_va + offset, size }, .kind = GPU_M4_RENDER_NATIVE_CACHE,
		.cache = { .object = b->object, .upper_lease = b->upper, .size = b->size,
			.firmware_va = b->fw_va, .access = M4_USER_NATIVE_RW },
	};
}

static int m4_user_native_render_allocate(struct m4_user_native_stage *s, u32 index,
	const struct gpu_m4_drm_job_render *command)
{
	static const u32 resources[] = {
		GPU_M4_RENDER_TILEMAP, GPU_M4_RENDER_TPC, GPU_M4_RENDER_SCENE_METADATA,
		GPU_M4_RENDER_SCENE_LIST, GPU_M4_RENDER_PREEMPT_0, GPU_M4_RENDER_PREEMPT_1,
		GPU_M4_RENDER_PREEMPT_2, GPU_M4_RENDER_SCRATCH, GPU_M4_RENDER_FRAGMENT_AUXILIARY,
	};
	struct m4_user_render_native *n = &s->command[index].render;
	const struct drm_asahi_cmd_render *r = &command->uapi;
	struct gpu_m4_render_geometry geometry = { r->width_px, r->height_px, r->layers,
		r->samples, r->utile_width_px, r->utile_height_px, r->sample_size_B };
	struct gpu_m4_render_native_owner *o = &n->owner;
	struct gpu_m4_exec_vm vm = { .start = GPU_M4_DRM_UAT_PRIVATE_START,
		.end = GPU_M4_DRM_UAT_PRIVATE_END, .cookie = s, .check = m4_user_native_initial_range };
	struct m4_user_native_bo *b;
	struct mutex *lock = &s->queue->runtime->hello->gpu->lock;
	u64 resource_size[ARRAY_SIZE(resources)], gpu_size[GPU_M4_RENDER_FIRST_GPU_COUNT];
	int ret;

	s->command[index].render_key = geometry;
	ret = gpu_m4_render_geometry(&n->geometry, &geometry);
	if (!ret)
		ret = m4_user_render_pb_layout(&n->pb, &geometry, s->queue);
	if (!ret) {
		m4_user_render_highwater_note(s->queue, r->layers > 1, &n->geometry, &n->pb);
		ret = m4_user_render_highwater_apply(s->queue, r->layers > 1, &n->geometry, &n->pb);
	}
	if (ret)
		return ret;
	resource_size[0] = n->geometry.tilemap_size;
	resource_size[1] = n->geometry.tpc_size;
	resource_size[2] = resource_size[3] = 0x4000;
	resource_size[4] = resource_size[5] = 0x10000;
	resource_size[6] = resource_size[7] = 0x4000;
	resource_size[8] = 0x10000;
	for (u32 i = 0; i < ARRAY_SIZE(resources); i++) {
		u32 role = resources[i];
		bool fw = role == GPU_M4_RENDER_TPC || role == GPU_M4_RENDER_SCRATCH ||
			role == GPU_M4_RENDER_FRAGMENT_AUXILIARY;

		ret = m4_user_native_add(s, resource_size[i], GPU_M4_DRM_PAGE_SIZE,
			true, fw, false, false, &n->resource[role]);
		if (ret)
			return ret;
		o->resource[role] = m4_user_native_gpu(n->resource[role]);
	}
	/* Layer metadata is a subrecord of the real scene metadata allocation;
	 * the native encoder validates this alias specially, not as another BO.
	 */
	n->resource[GPU_M4_RENDER_LAYER_METADATA] = n->resource[GPU_M4_RENDER_SCENE_METADATA];
	o->resource[GPU_M4_RENDER_LAYER_METADATA] = o->resource[GPU_M4_RENDER_SCENE_METADATA];
	o->resource[GPU_M4_RENDER_LAYER_METADATA].span = (struct gpu_m4_render_span) {
		n->resource[GPU_M4_RENDER_SCENE_METADATA]->va + (r->layers > 1 ? 0x200 : 0), 0x100,
	};
	n->tpc = n->resource[GPU_M4_RENDER_TPC];
	o->tpc_firmware = m4_user_native_fw(n->tpc, 0, n->tpc->size);
	ret = m4_user_native_add(s, n->pb.heap_size, 0x20000, true, false, false, false,
		&n->parameter_heap);
	if (ret)
		return ret;
	o->parameter_heap = m4_user_native_gpu(n->parameter_heap);
	gpu_size[GPU_M4_RENDER_FIRST_PRIVATE_POOL_GPU] = GPU_M4_EXEC_POOL_SIZE;
	gpu_size[GPU_M4_RENDER_FIRST_POOL_STATE_GPU] = GPU_M4_EXEC_POOL_STATE_SIZE;
	gpu_size[GPU_M4_RENDER_FIRST_UMA_LIST_GPU] = GPU_M4_EXEC_POOL_PAGE_LIST_SIZE;
	gpu_size[GPU_M4_RENDER_FIRST_PB_LIST_GPU] = n->pb.page_list_size;
	gpu_size[GPU_M4_RENDER_FIRST_UMA_GPU] = 0x4000;
	gpu_size[GPU_M4_RENDER_FIRST_TILER_WORK_GPU] = 0x4000;
	gpu_size[GPU_M4_RENDER_FIRST_FRAGMENT_WORK_GPU] = 0x4000;
	for (u32 i = 0; i < GPU_M4_RENDER_FIRST_GPU_COUNT; i++) {
		bool work = i == GPU_M4_RENDER_FIRST_TILER_WORK_GPU ||
			i == GPU_M4_RENDER_FIRST_FRAGMENT_WORK_GPU;
		bool fw = work || i == GPU_M4_RENDER_FIRST_PB_LIST_GPU || i == GPU_M4_RENDER_FIRST_UMA_GPU;

		ret = m4_user_native_add(s, gpu_size[i], GPU_M4_DRM_PAGE_SIZE, true, fw,
			false, work, &n->gpu[i]);
		if (ret)
			return ret;
		o->gpu[i] = m4_user_native_gpu(n->gpu[i]);
	}
	n->work[0] = n->gpu[GPU_M4_RENDER_FIRST_TILER_WORK_GPU];
	n->work[1] = n->gpu[GPU_M4_RENDER_FIRST_FRAGMENT_WORK_GPU];
#define M4_NATIVE_FW(role, bytes) do { \
	ret = m4_user_native_add(s, (bytes), GPU_M4_DRM_PAGE_SIZE, false, true, false, false, \
		&n->firmware[(role)]); \
	if (ret) return ret; \
	o->firmware[(role)] = m4_user_native_fw(n->firmware[(role)], 0, (bytes)); \
} while (0)
	M4_NATIVE_FW(GPU_M4_RENDER_FIRST_SCENE_FW, 0x4000);
	M4_NATIVE_FW(GPU_M4_RENDER_FIRST_PB_BLOCKS_FW, n->pb.block_list_size);
	M4_NATIVE_FW(GPU_M4_RENDER_FIRST_PB_RING_FW, 8);
	M4_NATIVE_FW(GPU_M4_RENDER_FIRST_PB_COUNTER_FW, 0x40);
	M4_NATIVE_FW(GPU_M4_RENDER_FIRST_TILER_SEQUENCE_FW, 0x400);
	M4_NATIVE_FW(GPU_M4_RENDER_FIRST_FRAGMENT_SEQUENCE_FW, 0x800);
#undef M4_NATIVE_FW
#define M4_NATIVE_ALIAS(role, bo, logical) do { \
	n->firmware[(role)] = (bo); \
	o->firmware[(role)] = m4_user_native_fw((bo), 0, (logical)); \
} while (0)
	M4_NATIVE_ALIAS(GPU_M4_RENDER_FIRST_SCRATCH_FW, n->resource[GPU_M4_RENDER_SCRATCH], 0x4000);
	M4_NATIVE_ALIAS(GPU_M4_RENDER_FIRST_UMA_FW, n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU], 0x4000);
	M4_NATIVE_ALIAS(GPU_M4_RENDER_FIRST_PB_LIST_FW, n->gpu[GPU_M4_RENDER_FIRST_PB_LIST_GPU], n->pb.page_list_size);
	M4_NATIVE_ALIAS(GPU_M4_RENDER_FIRST_TILER_WORK_FW, n->work[0], 0x4000);
	M4_NATIVE_ALIAS(GPU_M4_RENDER_FIRST_FRAGMENT_WORK_FW, n->work[1], 0x4000);
#undef M4_NATIVE_ALIAS
	ret = m4_user_native_add(s, 0x4000, GPU_M4_DRM_PAGE_SIZE, false, true, false, false, &n->manager);
	if (ret)
		return ret;
	o->manager = m4_user_native_fw(n->manager, 0, GPU_M4_RENDER_PARAMETER_MANAGER_SIZE);
	b = n->firmware[GPU_M4_RENDER_FIRST_SCENE_FW];
	o->init_buffer = m4_user_native_fw(b, 0x300, GPU_M4_RENDER_INIT_BUFFER_SIZE);
	o->fragment_dependency = m4_user_native_fw(b, 0x340, GPU_M4_EXEC_DEPENDENCY_SIZE);
	o->stage_owner = s;
	o->vm = s->queue->vm->provider_vm;
	o->render.context = o->render.asid = s->queue->vm->asid;
	o->render.parameter_manager_fw = n->manager->fw_va;
	put_unaligned_le64(n->resource[GPU_M4_RENDER_SCRATCH]->va, b->map.vaddr);
	put_unaligned_le64(n->resource[GPU_M4_RENDER_SCRATCH]->fw_va, b->map.vaddr + 8);
	put_unaligned_le64(n->resource[GPU_M4_RENDER_SCENE_LIST]->va - GPU_M4_RENDER_RANGE1_START,
		b->map.vaddr + 0x28);
	put_unaligned_le64(b->fw_va + 0x200, b->map.vaddr + 0x40);
	/* Slot/PB descriptor, physical READY queues/activity and notifier binding
	 * are supplied only by the root's actual publication reservation. Fresh PB
	 * allocator bytes also wait for that real ID; no dummy ID is encoded here.
	 */
	mutex_lock(lock);
	ret = gpu_m4_exec_uma_pool(n->gpu[GPU_M4_RENDER_FIRST_POOL_STATE_GPU]->map.vaddr,
		n->gpu[GPU_M4_RENDER_FIRST_UMA_LIST_GPU]->map.vaddr, &vm,
		n->gpu[GPU_M4_RENDER_FIRST_PRIVATE_POOL_GPU]->va);
	mutex_unlock(lock);
	dma_wmb();
	return ret;
}

/* The exact whole-BO RW cache row of b's pin and RW reservation row of its GEM. */
static bool m4_user_native_cache_row(const struct gpu_m4_drm_cache_access *a,
	const struct m4_user_native_bo *b)
{
	return a->pin == b->pin && a->offset == 0 && a->size >= b->coverage &&
		a->size <= b->size && a->access == M4_USER_NATIVE_RW;
}

static bool m4_user_native_resv_row(const struct gpu_m4_drm_resv_access *r,
	const struct m4_user_native_bo *b)
{
	return r->object == b->gem && r->access == M4_USER_NATIVE_RW;
}

static int m4_user_native_snapshot_bo_locked(struct m4_user_native_bo *b,
	struct gpu_m4_drm_provider *provider, struct gpu_m4_drm_provider_version *version,
	const struct gpu_m4_drm_vm_owner *snapshot)
{
	const struct gpu_m4_drm_cache_access *cache;
	const struct gpu_m4_drm_resv_access *resv;
	u32 nc, nr, row;
	bool cached = false, reserved = false;
	int ret = m4_user_native_bo_check_locked(b, provider, version);

	if (ret)
		return ret;
	cache = gpu_m4_drm_vm_owner_cache_access(snapshot, &nc);
	resv = gpu_m4_drm_vm_owner_resv_access(snapshot, &nr);
	if (!cache || !resv || nc > GPU_M4_DRM_MAX_BINDS || nr > GPU_M4_DRM_MAX_BINDS)
		return M4_USER_WHY(-ESTALE, "native snapshot cache=%d/%u resv=%d/%u", !!cache, nc, !!resv, nr);
	/* The snapshot's own row of this GEM first (its identity order); a
	 * refusal there takes the complete scans, so the answer is theirs.
	 */
	row = gpu_m4_drm_vm_owner_find(snapshot, b->gem);
	if (row < nc)
		cached = m4_user_native_cache_row(&cache[row], b);
	if (row < nr)
		reserved = m4_user_native_resv_row(&resv[row], b);
	for (u32 i = 0; !cached && i < nc; i++)
		cached = m4_user_native_cache_row(&cache[i], b);
	for (u32 i = 0; !reserved && i < nr; i++)
		reserved = m4_user_native_resv_row(&resv[i], b);
	return cached && reserved ? 0 : M4_USER_WHY(-EACCES, "native snapshot bo %#llx cached=%d reserved=%d",
		M4_WHY64(b->fw_va ? b->fw_va : b->va), cached, reserved);
}

static int m4_user_native_admission_check_locked(struct m4_user_native_stage *s,
	struct m4_user_native_history *h, struct gpu_m4_drm_provider_version *version,
	const struct gpu_m4_drm_vm_owner *snapshot)
{
	struct m4_gpu_provider_runtime *p = s->queue->runtime;
	struct gpu_m4_drm_provider_job_view view = { };
	struct gpu_m4_drm_cache_job *cache = NULL;
	bool retired = h == s->history;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (p->error || !h->job || !h->admitted_cache || h->runtime != p ||
	    h->owner_stage != s || h->count != s->count || h->complete != retired ||
	    (!retired && (h != s->pending || m4_user_inflight_slot_locked(p, h->job) < 0)) ||
	    h->version != version || h->snapshot != snapshot ||
	    h->admitted_bo_count != s->bo_count ||
	    h->admitted_native_count != s->native_count ||
	    h->admitted_extra_count != s->extra_count)
		return M4_USER_WHY(p->error ?: -ESTALE, "native admission retired=%d error=%d job=%d cache=%d owner=%d count=%u/%u complete=%d pending=%d current=%d version=%d snapshot=%d bo=%u/%u native=%u/%u extra=%u/%u",
			retired, p->error, !!h->job, !!h->admitted_cache, h->owner_stage == s, h->count,
			s->count, h->complete, h == s->pending, m4_user_inflight_slot_locked(p, h->job) >= 0,
			h->version == version, h->snapshot == snapshot, h->admitted_bo_count,
			s->bo_count, h->admitted_native_count, s->native_count,
			h->admitted_extra_count, s->extra_count);
	gpu_m4_drm_provider_job_view(h->job, &view);
	if (view.stage_owner != s || view.runtime_queue != s->queue ||
	    view.runtime_job != h->admitted_runtime_job ||
	    view.version != version || view.snapshot != snapshot || !view.batch ||
	    !view.batch->commands || view.batch->count != s->count ||
	    view.batch->vm_snapshot != snapshot || view.batch != h->admitted_batch ||
	    view.batch->commands != h->admitted_commands ||
	    view.command_bytes != h->admitted_command_bytes ||
	    view.command_size != h->admitted_command_size)
		return M4_USER_WHY(-ESTALE, "native admission view retired=%d stage=%d queue=%d job=%d version=%d snapshot=%d batch=%d/%u commands=%d bytes=%d size=%u/%u",
			retired, view.stage_owner == s, view.runtime_queue == s->queue,
			view.runtime_job == h->admitted_runtime_job, view.version == version,
			view.snapshot == snapshot, view.batch == h->admitted_batch,
			view.batch ? view.batch->count : 0,
			view.batch && view.batch->commands == h->admitted_commands,
			view.command_bytes == h->admitted_command_bytes, view.command_size,
			h->admitted_command_size);
	/* This asks the real core for its current cache/resv/fence phase. An old
	 * admission cannot turn an error, unprotected job or unfinished CORE into
	 * successful execution/retirement. The immutable remaining memberships
	 * were checked against this SAME cache and snapshot before the memo.
	 */
	ret = gpu_m4_drm_provider_job_check_access_locked(h->job, p->provider,
		s->notifier->bo->object, 0, s->notifier->bo->coverage,
		M4_USER_NATIVE_RW, retired, &cache);
	return M4_USER_WHY(ret ?: (cache == h->admitted_cache ? 0 : -ESTALE),
		"native admission access retired=%d ret=%d cache=%d", retired, ret,
		cache == h->admitted_cache);
}

static int m4_user_native_command_check_locked(struct m4_user_native_stage *s, u32 index,
	u16 type, struct gpu_m4_drm_provider *provider, struct gpu_m4_drm_provider_version *version,
	const struct gpu_m4_drm_vm_owner *snapshot)
{
	struct m4_user_native_history *h;
	int ret;

	if (!s || !s->queue || !refcount_read(&s->refs) || !s->count ||
	    s->count > GPU_M4_DRM_JOB_MAX_COMMANDS || index >= s->count ||
	    s->command[index].type != type || s->queue->runtime->provider != provider ||
	    !version || !snapshot || !s->notifier || s->notifier != s->queue->notifier ||
	    !refcount_read(&s->notifier->refs) || s->bo_count > M4_USER_NATIVE_BO_MAX)
		return M4_USER_WHY(-ESTALE, "native command %u type=%u/%u stage=%d refs=%u count=%u provider=%d notifier=%d/%d bo=%u",
			index, s && index < s->count ? s->command[index].type : 0xffff, type, !!s,
			s ? refcount_read(&s->refs) : 0, s ? s->count : 0,
			s && s->queue && s->queue->runtime->provider == provider,
			s && s->notifier, s && s->queue && s->notifier == s->queue->notifier,
			s ? s->bo_count : 0);
	lockdep_assert_held(&s->queue->runtime->hello->gpu->lock);
	h = s->pending && s->pending->version == version && s->pending->snapshot == snapshot ?
		s->pending : s->history && s->history->version == version &&
		s->history->snapshot == snapshot ? s->history : NULL;
	if (h && h->admitted_cache)
		return m4_user_native_admission_check_locked(s, h, version, snapshot);
	ret = gpu_m4_drm_provider_version_check_origin_locked(provider, s->queue->vm->provider_vm, version);
	if (!ret)
		ret = m4_user_native_snapshot_bo_locked(s->notifier->bo, provider, version, snapshot);
	for (u32 i = 0; !ret && i < s->bo_count; i++)
		ret = m4_user_native_snapshot_bo_locked(s->bo[i], provider, version, snapshot);
	return ret;
}

static int m4_user_native_compute_check_locked(struct m4_user_native_stage *s, u32 index,
	struct gpu_m4_drm_provider *provider, struct gpu_m4_drm_provider_version *version,
	const struct gpu_m4_drm_vm_owner *snapshot)
{
	return m4_user_native_command_check_locked(s, index, DRM_ASAHI_CMD_COMPUTE, provider, version, snapshot);
}

static int m4_user_native_render_check_locked(struct m4_user_native_stage *s, u32 index,
	struct gpu_m4_drm_provider *provider, struct gpu_m4_drm_provider_version *version,
	const struct gpu_m4_drm_vm_owner *snapshot)
{
	return m4_user_native_command_check_locked(s, index, DRM_ASAHI_CMD_RENDER, provider, version, snapshot);
}

static const struct m4_user_compute_native *m4_user_native_compute(
	const struct m4_user_native_stage *s, u32 index)
{
	return s && index < s->count && s->command[index].type == DRM_ASAHI_CMD_COMPUTE ?
		&s->command[index].compute : NULL;
}

static const struct m4_user_render_native *m4_user_native_render(
	const struct m4_user_native_stage *s, u32 index)
{
	return s && index < s->count && s->command[index].type == DRM_ASAHI_CMD_RENDER ?
		&s->command[index].render : NULL;
}

static int m4_user_native_stage_get_locked(struct m4_user_native_stage *s)
{
	if (!s || !refcount_read(&s->refs) || refcount_read(&s->refs) == U32_MAX)
		return -EOVERFLOW;
	refcount_inc(&s->refs);
	return 0;
}

/* Protected publication entrance, BEFORE the first root/upper/producer write.
 * The runtime records each command's real hardware observations into pending,
 * then the actual successful CORE callback alone completes it. A job ref owns
 * its version/snapshot; no opaque provider fields are dereferenced here.
 */
static int m4_user_native_receipt_prepare_locked(struct m4_user_native_stage *s,
	struct gpu_m4_drm_provider_job *job)
{
	struct gpu_m4_drm_provider_job_view view = { };
	struct m4_gpu_provider_runtime *p;
	struct m4_user_native_history *h;
	struct gpu_m4_drm_cache_job *cache = NULL;
	u32 refs, last_mask = 0;
	u64 proof;
	int ret;

	if (!s || !s->queue || !job || !s->pending)
		return -EINVAL;
	p = s->queue->runtime;
	h = s->pending;
	lockdep_assert_held(&p->hello->gpu->lock);
	/* Withdraw any earlier proof first; it is re-established only below. */
	h->access_proof = 0;
	gpu_m4_drm_provider_job_view(job, &view);
	if (m4_user_inflight_slot_locked(p, job) < 0 || view.stage_owner != s ||
	    view.runtime_queue != s->queue || !view.batch ||
	    !view.batch->commands || view.batch->count != s->count ||
	    h->count != s->count || h->complete ||
	    (h->job && (h->job != job || h->version != view.version || h->snapshot != view.snapshot)))
		return -ESTALE;
	ret = m4_user_native_command_check_locked(s, 0, s->command[0].type,
		p->provider, view.version, view.snapshot);
	for (u32 i = 0; !ret && i < s->count; i++) {
		struct m4_user_native_unit *u = s->unit[i];

		if (!u || u->queue != s->queue || u->accepted != s || u->idle ||
		    !refcount_read(&u->refs) || refcount_read(&u->refs) == U32_MAX ||
		    u->command.type != s->command[i].type ||
		    (m4_user_native_reused(s, i) &&
		     (!s->reuse_history[i] || !s->reuse_displacement[i]))) {
			ret = -ESTALE;
			break;
		}
		if (view.batch->commands[i].type != s->command[i].type)
			ret = -ESTALE;
		else if (s->command[i].type == DRM_ASAHI_CMD_COMPUTE)
			last_mask |= 1U << 2;
		else if (s->command[i].type == DRM_ASAHI_CMD_RENDER)
			last_mask |= 3U;
		else
			ret = -EOPNOTSUPP;
	}
	/* One complete selected-inventory proof for this exact protected job:
	 * each distinct BO (stage BOs and the queue notifier) is checked once.
	 */
	proof = ++m4_user_native_proof_seq;
	for (u32 i = 0; !ret && i <= s->bo_count; i++) {
		struct m4_user_native_bo *b = i == s->bo_count ? s->notifier->bo : s->bo[i];

		if (b->access_proof == proof)
			continue; /* Repeated occurrence of an already proven BO. */
		ret = gpu_m4_drm_provider_job_check_access_locked(job, p->provider,
			b->object, 0, b->coverage, M4_USER_NATIVE_RW, false, &cache);
		if (!ret) {
			b->access_proof = proof;
			m4_user_native_proof_stats.proof_bos++;
		}
	}
	if (!ret && h->job) {
		if (cache != h->admitted_cache)
			return -ESTALE;
		h->access_proof = proof;
		m4_user_native_proof_stats.proofs++;
	}
	if (ret || h->job)
		return ret;
	/* Only the actual engine-last references belong to this batch. Command
	 * backing and its history are retained independently, not by a stage cache.
	 */
	refs = 0;
	for (u32 i = 0; i < 3; i++)
		refs += !!(last_mask & (1U << i));
	if (!refcount_read(&s->refs) || refcount_read(&s->refs) > U32_MAX - refs)
		return -EOVERFLOW;
	ret = gpu_m4_drm_provider_job_get_locked(job);
	if (ret)
		return ret;
	for (u32 i = 0; i < refs; i++)
		refcount_inc(&s->refs);
	h->owner_stage = s;
	h->spare_refs = 0;
	h->last_mask = last_mask;
	h->job = job;
	h->version = view.version;
	h->snapshot = view.snapshot;
	h->admitted_cache = cache;
	h->admitted_batch = view.batch;
	h->admitted_runtime_job = view.runtime_job;
	h->admitted_commands = view.batch->commands;
	h->admitted_command_bytes = view.command_bytes;
	h->admitted_command_size = view.command_size;
	h->admitted_bo_count = s->bo_count;
	h->admitted_native_count = s->native_count;
	h->admitted_extra_count = s->extra_count;
	h->access_proof = proof;
	m4_user_native_proof_stats.proofs++;
	for (u32 i = 0; i < s->count; i++)
		h->command[i].type = s->command[i].type;
	return 0;
}

/* No fallible validation after actual core fence-ready. The existing lane
 * callback is the sole caller. All workspace/refs were obtained prepublication.
 */
static void m4_user_native_receipt_complete_locked(struct m4_user_native_stage *s)
{
	struct m4_user_native_history *h = s->pending;
	struct m4_gpu_provider_runtime *p = s->queue->runtime;
	struct m4_user_native_unit *slot_prior[GPU_M4_DRM_JOB_MAX_COMMANDS] = { };

	lockdep_assert_held(&s->queue->runtime->hello->gpu->lock);
	if (WARN_ON(!h || !h->job || h->complete || refcount_read(&h->refs) != 1))
		return;
	/* Per-slot prior owners BEFORE this receipt adopts its units: compute and
	 * render displacement follow the actual leased UMA slot, never an engine
	 * tail. Commands of one child lease distinct slots.
	 */
	for (u32 i = 0; i < s->count && i < ARRAY_SIZE(slot_prior); i++) {
		u16 type = s->command[i].type;
		int ret;

		if (type != DRM_ASAHI_CMD_COMPUTE && type != DRM_ASAHI_CMD_RENDER)
			continue;
		for (u32 k = 0; k < i; k++)
			WARN_ON(s->command[k].type == type &&
				h->command[k].uma_slot == h->command[i].uma_slot);
		ret = m4_user_native_slot_owner_locked(p, type, h->command[i].uma_slot,
			&slot_prior[i]);
		WARN_ON(ret && ret != -ENOENT);
	}
	h->complete = true;
	s->history = h;
	s->pending = NULL;
	s->history_owned = false; /* Unit refs, not this stage, own immutable h. */
	/* Commit all new uses first. Earlier commands in this very batch can then
	 * be released by distinct later same-slot commands under this ONE actual
	 * successful CORE/cache receipt. No pending receipt enters the idle set.
	 */
	for (u32 i = 0; i < s->count; i++) {
		struct m4_user_native_unit *u = s->unit[i];

		if (WARN_ON(!u || u->accepted != s || !u->published))
			continue;
		refcount_inc(&h->refs);
		m4_user_native_history_put_locked(u->history);
		m4_user_native_history_put_locked(u->displacement);
		u->history = h; u->history_index = i;
		u->displacement = NULL; u->displacer = NULL;
		u->accepted = NULL; u->idle = false;
	}
	for (u32 i = 0; i < s->count; i++) {
		struct m4_user_native_unit *u = s->unit[i];
		u32 first = s->command[i].type == DRM_ASAHI_CMD_COMPUTE ? 2 : 0;
		u32 engines = first == 2 ? 1 : 2;
		/* The engine tails below stay chronological pointers only. */
		struct m4_user_native_unit *prior = i < ARRAY_SIZE(slot_prior) ? slot_prior[i] : NULL;

		if (WARN_ON(!u))
			continue;
		if (prior && prior != u) {
			refcount_inc(&h->refs);
			m4_user_native_history_put_locked(prior->displacement);
			prior->displacement = h; prior->displacement_index = i;
			prior->displacer = u;
			prior->idle = !prior->accepted;
		}
		for (u32 e = first; e < first + engines; e++)
			p->user_unit_last[e] = u;
		if (h->command[i].uma_slot < M4_USER_SLOT_TABLE)
			p->user_slot_unit[h->command[i].uma_slot] = u;
	}
	/* Explicit acceptance origin refs survive all packet writes and are
	 * relinquished ONLY after their new use is fully CORE/cache retired.
	 */
	for (u32 i = 0; i < s->count; i++) {
		m4_user_native_history_put_locked(s->reuse_history[i]);
		m4_user_native_history_put_locked(s->reuse_displacement[i]);
		s->reuse_history[i] = s->reuse_displacement[i] = NULL;
	}
	/* Settlement waits for the success callback's bookkeeping (reaper). */
	if (READ_ONCE(native_settle) && list_empty(&h->settle)) {
		refcount_inc(&h->refs);
		list_add_tail(&h->settle, &p->user_native_settle);
		schedule_work(&p->user_reap);
	}
	m4_user_native_history_put_locked(h); /* Transfer pending's ref to units. */
}

/* Transfer one preheld engine-last ref after actual CORE success. This is
 * infallible bookkeeping, not a new reference acquisition or retirement test.
 * Root assigns its exact last-owner pointer immediately after this transfer.
 */
static void m4_user_native_stage_last_locked(struct m4_user_native_stage *s, u32 engine)
{
	struct m4_user_native_history *h = s->history;

	lockdep_assert_held(&s->queue->runtime->hello->gpu->lock);
	if (WARN_ON(!h || !h->complete || engine >= 3 || !(h->last_mask & (1U << engine))))
		return;
	h->last_mask &= ~(1U << engine);
}

struct m4_user_native_render_history_sample {
	struct m4_user_native_stage *stage;
	struct m4_user_native_history *history;
	u32 index;
	struct gpu_m4_render_pair_retirement pair;
};

/* Authenticate the retained paired receipt through the REAL successful
 * CORE/cache job, not through a copied completed bit or an invented TA stamp.
 */
static int m4_user_native_render_history_owner_locked(void *cookie,
	const struct gpu_m4_render_pair_retirement *pair)
{
	struct m4_user_native_render_history_sample *sample = cookie;
	struct m4_user_native_stage *s = sample->stage;
	struct m4_user_native_history *h = sample->history;
	const struct m4_user_native_receipt *r = &h->command[sample->index];
	const struct m4_user_render_native *n = m4_user_native_render(s, sample->index);
	struct gpu_m4_drm_provider_job_view view = { };
	const struct gpu_m4_drm_job_render *command;
	u32 count = 0;
	bool retired;
	int ret;

	/* Phase comes from exact held receipt membership, never a caller flag.
	 * Pending receipts belong to earlier hardware-retired commands of the
	 * current protected batch; historical receipts require real CORE success.
	 */
	retired = h == s->history;
	if ((!retired && (h != s->pending || h->complete ||
	    m4_user_inflight_slot_locked(s->queue->runtime, h->job) < 0)) ||
	    (retired && !h->complete) ||
	    h->owner_stage != s || h->runtime != s->queue->runtime || h->count != s->count ||
	    !refcount_read(&s->refs) || pair != &sample->pair || !n ||
	    pair->dependencies != r->render_dependency ||
	    !r->render_dependency_count || r->render_dependency_count > 3 ||
	    pair->dependency_count != r->render_dependency_count ||
	    pair->dependency_capacity != ARRAY_SIZE(r->render_dependency) ||
	    pair->copies != r->render_copies || pair->copy_count != r->render_copy_count ||
	    pair->copy_capacity != GPU_M4_DRM_RENDER_TIMESTAMPS ||
	    r->render_dependency[0].record_fw != n->owner.fragment_dependency.span.va ||
	    (r->render_dependency_count >= 2 &&
	     (r->render_dependency[1].record_fw !=
		n->firmware[GPU_M4_RENDER_FIRST_SCENE_FW]->fw_va + 0x380 ||
	      r->render_dependency[1].engine ||
	      /* In-window prior render (early tiling: its TA), a mixed
	       * window's prior compute, or the first command's global-tail
	       * barrier on its recorded predecessor's page.
	       */
	      ((r->render_dependency[1].stamp_fw != s->notifier->bo->fw_va + 0x630 ||
		r->render_dependency[1].event != 0x71) &&
	       (!sample->index || r->render_dependency_count != 2 ||
		r->render_dependency[1].stamp_fw != s->notifier->bo->fw_va + 0x640 ||
		r->render_dependency[1].event != 15) &&
	       ((!sample->index && h->cross_stamp_fw != s->notifier->bo->fw_va + 0x630) ||
		r->render_dependency_count != 3 ||
		r->render_dependency[1].stamp_fw != s->notifier->bo->fw_va + 0x620 ||
		r->render_dependency[1].event != 0x70) &&
	       (sample->index || !h->cross_stamp_fw ||
		r->render_dependency[1].stamp_fw != h->cross_stamp_fw ||
		r->render_dependency[1].event != h->cross_event)))) ||
	    /* An early-tiling render's FS strict wait on the previous FS. */
	    /* (The first command: early tiling across the window boundary behind
	     * its own queue's previous render, the global tail.)
	     */
	    (r->render_dependency_count == 3 &&
	     ((!sample->index && h->cross_stamp_fw != s->notifier->bo->fw_va + 0x630) ||
	      r->render_dependency[1].event != 0x70 ||
	      r->render_dependency[2].record_fw !=
		n->firmware[GPU_M4_RENDER_FIRST_SCENE_FW]->fw_va + 0x440 ||
	      r->render_dependency[2].engine != 1 || r->render_dependency[2].event != 0x71 ||
	      r->render_dependency[2].stamp_fw != s->notifier->bo->fw_va + 0x630)) ||
	    pair->uma_descriptor_fw != n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU]->fw_va ||
	    pair->uma_submitted_fw != n->firmware[GPU_M4_RENDER_FIRST_SCENE_FW]->fw_va + 0x500 ||
	    pair->stage[0].expected_generation != h->binding_generation ||
	    pair->stage[1].expected_generation != h->binding_generation ||
	    pair->stage[0].notifier_fw != s->notifier->bo->fw_va ||
	    pair->stage[1].notifier_fw != s->notifier->bo->fw_va)
		return -ESTALE;
	gpu_m4_drm_provider_job_view(h->job, &view);
	if (view.stage_owner != s || view.runtime_queue != s->queue ||
	    view.version != h->version || view.snapshot != h->snapshot || !view.batch ||
	    !view.batch->commands || view.batch->count != h->count || sample->index >= view.batch->count ||
	    view.batch->commands[sample->index].type != DRM_ASAHI_CMD_RENDER)
		return -ESTALE;
	ret = gpu_m4_drm_provider_job_check_access_locked(h->job, s->queue->runtime->provider,
		s->notifier->bo->object, 0, s->notifier->bo->coverage, M4_USER_NATIVE_RW, retired, NULL);
	if (ret)
		return ret;
	command = &view.batch->commands[sample->index].render;
	for (u32 i = 0; i < GPU_M4_DRM_RENDER_TIMESTAMPS; i++) {
		const struct gpu_m4_drm_job_timestamp *t = &command->timestamps[i];
		const struct gpu_m4_render_retirement_copy *copy;

		if (!t->lease)
			continue;
		if (count >= r->render_copy_count)
			return -ESTALE;
		copy = &r->render_copies[count++];
		if (copy->destination_fw != t->firmware_va || copy->engine != (i < 2 ? 0 : 1) ||
		    copy->end != (i == 1 || i >= 3))
			return -ESTALE;
	}
	return count == r->render_copy_count ? 0 : -ESTALE;
}

static int m4_user_native_render_receipt_pair_locked(struct m4_user_native_stage *s,
	struct m4_user_native_history *history, u32 index,
	const struct gpu_m4_render_stage_retirement observed[2])
{
	const struct m4_user_native_receipt *r;
	const struct m4_user_render_native *n = m4_user_native_render(s, index);
	struct m4_user_native_render_history_sample sample = {
		.stage = s, .history = history, .index = index,
	};

	if (!history || (history != s->history && history != s->pending) ||
	    history->count != s->count || index >= history->count)
		return -ESTALE;
	r = &history->command[index];
	if (!n || r->submitted < 2 || (r->submitted & 1) || r->submitted != r->completed ||
	    r->render_copy_count > GPU_M4_DRM_RENDER_TIMESTAMPS ||
	    !r->render_dependency_count || r->render_dependency_count > 3)
		return -ESTALE;
	memcpy(sample.pair.stage, observed ?: r->render, sizeof(sample.pair.stage));
	sample.pair.uma_descriptor_fw = n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU]->fw_va;
	sample.pair.uma_submitted_fw = n->firmware[GPU_M4_RENDER_FIRST_SCENE_FW]->fw_va + 0x500;
	sample.pair.expected_uma_slot = sample.pair.uma_slot = r->uma_slot;
	sample.pair.uma_submitted_before = r->submitted - 2;
	sample.pair.uma_expected_submitted = sample.pair.uma_submitted = r->submitted;
	sample.pair.uma_completed_high_before = sample.pair.uma_completed_high_after = r->completed >> 32;
	sample.pair.uma_completed_low = r->completed;
	sample.pair.copies = r->render_copies;
	sample.pair.copy_count = r->render_copy_count;
	sample.pair.copy_capacity = GPU_M4_DRM_RENDER_TIMESTAMPS;
	sample.pair.dependencies = r->render_dependency;
	sample.pair.dependency_count = r->render_dependency_count;
	sample.pair.dependency_capacity = ARRAY_SIZE(r->render_dependency);
	sample.pair.init_buffer = true;
	/* Mandatory callback proves actual committed CORE/cache provenance. */
	sample.pair.rtkit_healthy = sample.pair.run_healthy = sample.pair.control_healthy = true;
	sample.pair.binding_healthy = sample.pair.fault_drain_complete = true;
	return gpu_m4_render_pair_hardware_retired(&sample.pair, &sample,
		m4_user_native_render_history_owner_locked);
}

static int m4_user_native_history_check_locked(struct m4_user_native_stage *s)
{
	struct m4_user_native_history *h = s->history;
	struct gpu_m4_drm_provider_job_view view = { };
	struct m4_gpu_provider_runtime *p = s->queue->runtime;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	/* Settled: every receipt passed its full proof through the actual
	 * successful job before that job was released; the values are immutable.
	 */
	if (h && h->settled)
		return M4_USER_WHY(p->error ?: (h->complete && h->receipts_proven && !h->job &&
			h->runtime == p && h->owner_stage == s && h->count == s->count &&
			s->history == h ? 0 : -ESTALE), "native settled history count=%u", h->count);
	if (!h || !h->complete || !h->job || h->runtime != p || h->owner_stage != s ||
	    h->count != s->count || !h->binding_epoch || h->asid != s->queue->vm->asid ||
	    h->binding_generation != gpu_m4_drm_provider_binding_wire_generation(h->binding_epoch))
		return M4_USER_WHY(-ESTALE, "native history=%d complete=%d job=%d runtime=%d owner=%d count=%u/%u epoch=%llu wire=%u",
			!!h, h && h->complete, h && h->job, h && h->runtime == p, h && h->owner_stage == s,
			h ? h->count : 0, s->count, M4_WHY64(h ? h->binding_epoch : 0),
			h ? h->binding_generation : 0);
	gpu_m4_drm_provider_job_view(h->job, &view);
	if (view.stage_owner != s || view.runtime_queue != s->queue ||
	    view.version != h->version || view.snapshot != h->snapshot ||
	    !view.batch || !view.batch->commands || view.batch->count != h->count)
		return M4_USER_WHY(-ESTALE, "native history view stage=%d queue=%d version=%d snapshot=%d batch=%u/%u",
			view.stage_owner == s, view.runtime_queue == s->queue, view.version == h->version,
			view.snapshot == h->snapshot, view.batch ? view.batch->count : 0, h->count);
	/* With an admission memo this is the job's live CORE/cache phase check:
	 * the very job_check_access each paired owner callback below repeats.
	 */
	ret = m4_user_native_command_check_locked(s, 0, s->command[0].type,
		p->provider, h->version, h->snapshot);
	if (!ret && h->receipts_proven && h->admitted_cache) {
		m4_user_native_proof_stats.receipt_reused++;
		return 0;
	}
	for (u32 i = 0; !ret && i < h->count; i++) {
		const struct m4_user_native_receipt *r = &h->command[i];

		if (r->type != s->command[i].type || r->type != view.batch->commands[i].type ||
		    !r->submitted || r->submitted != r->completed)
			return M4_USER_WHY(-ESTALE, "native history receipt %u type=%u/%u/%u submitted=%llu completed=%llu",
				i, r->type, s->command[i].type, view.batch->commands[i].type,
				M4_WHY64(r->submitted), M4_WHY64(r->completed));
		if (r->type == DRM_ASAHI_CMD_COMPUTE) {
			if (!m4_user_native_slot_valid(r->type, r->uma_slot) || r->parameter_id ||
			    r->compute.uma_submitted != r->submitted || r->compute.uma_completed != r->completed)
				return M4_USER_WHY(-ESTALE, "native history receipt %u slot=%u parameter=%u uma=%llu/%llu",
					i, r->uma_slot, r->parameter_id, M4_WHY64(r->compute.uma_submitted),
					M4_WHY64(r->compute.uma_completed));
			ret = gpu_m4_compute_reuse_retired_prefixes(&r->compute);
		} else if (r->type == DRM_ASAHI_CMD_RENDER) {
			if (!m4_user_native_slot_valid(r->type, r->uma_slot) ||
			    r->parameter_id != m4_user_native_slot_parameter(r->type, r->uma_slot) ||
			    (r->submitted & 1))
				return M4_USER_WHY(-ESTALE, "native history receipt %u render slot=%u parameter=%u submitted=%llu",
					i, r->uma_slot, r->parameter_id, M4_WHY64(r->submitted));
			ret = m4_user_native_render_receipt_pair_locked(s, h, i, NULL);
		} else {
			return -EOPNOTSUPP;
		}
	}
	for (u32 i = 0; !ret && !h->admitted_cache && i <= s->bo_count; i++) {
		struct m4_user_native_bo *b = i == s->bo_count ? s->notifier->bo : s->bo[i];

		ret = gpu_m4_drm_provider_job_check_access_locked(h->job, p->provider,
			b->object, 0, b->coverage, M4_USER_NATIVE_RW, true, NULL);
	}
	if (!ret && h->admitted_cache) {
		h->receipts_proven = true;
		m4_user_native_proof_stats.receipt_proofs++;
	}
	return M4_USER_WHY(ret, "native history proof count=%u memo=%d", h->count, h->receipts_proven);
}

/* REMOVE-DEBUG: settled histories, refused settlements, stages whose unit BO
 * references were dropped.
 */
static u64 m4_user_native_settle_stats[3];

/* After settlement AND the job's own stage reference (the last borrower of
 * the units' pins through its cache/snapshot) is gone, the stage stops
 * holding the units' BOs: each BO then lives exactly as long as its unit.
 * Stage-only BOs (no unit) stay with the stage.
 */
static void m4_user_native_stage_drop_bos_locked(struct m4_user_native_stage *s)
{
	lockdep_assert_held(&s->queue->runtime->hello->gpu->lock);
	if (s->bos_dropped || !s->job_gone || !s->history || !s->history->settled || s->pending)
		return;
	for (u32 i = 0; i < s->bo_count; i++) {
		struct m4_user_native_bo *b = s->bo[i];

		if (!b || !b->unit)
			continue;
		s->bo[i] = NULL;
		m4_user_native_bo_put_locked(b);
	}
	s->bos_dropped = true;
	m4_user_native_settle_stats[2]++;
}

/* Settle each queued complete history whose receipts pass their full proof
 * through the still held actual job; a refused one keeps its job (the
 * conservative whole-window retention).
 */
static void m4_user_native_settle(struct m4_gpu_provider_runtime *p)
{
	struct mutex *lock = &p->hello->gpu->lock;

	lockdep_assert_not_held(lock);
	for (;;) {
		struct gpu_m4_drm_provider_job *job = NULL;
		struct m4_user_native_history *h;
		struct m4_user_native_stage *s;

		mutex_lock(lock);
		if (list_empty(&p->user_native_settle)) {
			mutex_unlock(lock);
			break;
		}
		h = list_first_entry(&p->user_native_settle, struct m4_user_native_history, settle);
		list_del_init(&h->settle);
		s = h->owner_stage;
		if (READ_ONCE(native_settle) && !p->error && h->complete && h->job && !h->settled &&
		    s && s->history == h && !s->pending && refcount_read(&s->refs) &&
		    refcount_read(&s->refs) != U32_MAX &&
		    m4_user_inflight_slot_locked(p, h->job) < 0 &&
		    !m4_user_native_history_check_locked(s) && h->receipts_proven) {
			refcount_inc(&s->refs);
			h->stage_held = true;
			job = h->job;
			h->job = NULL;
			h->version = NULL;
			h->snapshot = NULL;
			h->admitted_cache = NULL;
			h->settled = true;
			m4_user_native_settle_stats[0]++;
			m4_user_native_stage_drop_bos_locked(s);
		} else {
			m4_user_native_settle_stats[1]++;
		}
		m4_user_native_history_put_locked(h); /* the queue's reference */
		mutex_unlock(lock);
		if (job)
			WARN_ON(gpu_m4_drm_provider_job_put(job));
	}
}


/* A whole retained bundle has one actual CORE receipt, but each allocator is
 * indexed by its own command. Neither a copied observation nor command zero
 * stands in for the selected command's successful receipt.
 */
static int m4_user_native_history_command_locked(struct m4_user_native_stage *s,
	u32 index, u16 type, const struct m4_user_native_receipt **receipt)
{
	int ret;

	if (!s || !receipt || index >= s->count || s->command[index].type != type)
		return M4_USER_WHY(-ESTALE, "native history command %u/%u type=%u/%u", index,
			s ? s->count : 0, s && index < s->count ? s->command[index].type : 0xffff, type);
	ret = m4_user_native_history_check_locked(s);
	if (ret)
		return M4_USER_WHY(ret, "native history command %u", index);
	*receipt = &s->history->command[index];
	return 0;
}

static bool m4_user_native_reused(const struct m4_user_native_stage *s, u32 index)
{
	return s && index < s->count && !!(s->reused_commands & (1ULL << index));
}

/* Historical displacement releases one exact completed use, not the unit's
 * next accepted use. Both histories retain their actual CORE/cache jobs and
 * original arrays/ordinals. The current hardware predecessor is checked by
 * the publisher independently and need not equal this historical witness.
 */
static int m4_user_native_reuse_history_locked(struct m4_user_native_stage *s,
	u32 index, struct m4_user_native_stage **origin, u32 *old_index,
	struct m4_user_native_stage **displacer, u32 *displacer_index)
{
	struct m4_user_native_unit *u;
	struct m4_user_native_history *a, *b;
	const struct m4_user_native_receipt *ra, *rb;
	struct m4_user_native_stage *sa, *sb;
	u32 ia, ib;
	int ret;

	if (!s || !origin || !old_index || !displacer || !displacer_index ||
	    !m4_user_native_reused(s, index) || !(u = s->unit[index]) ||
	    !(a = s->reuse_history[index]) || !(b = s->reuse_displacement[index]))
		return M4_USER_WHY(-ESTALE, "native reuse %u reused=%d unit=%d origin=%d displacement=%d",
			index, m4_user_native_reused(s, index), s && index < s->count && s->unit[index],
			s && index < s->count && s->reuse_history[index],
			s && index < s->count && s->reuse_displacement[index]);
	lockdep_assert_held(&s->queue->runtime->hello->gpu->lock);
	ia = s->reuse_index[index]; ib = s->displacement_index[index];
	sa = a->owner_stage; sb = b->owner_stage;
	if (!sa || !sb || !refcount_read(&a->refs) || !refcount_read(&b->refs) ||
	    !refcount_read(&u->refs) || !u->published || u->accepted != s || u->idle ||
	    u->queue != s->queue || u->history != a || u->history_index != ia ||
	    u->displacement != b || u->displacement_index != ib ||
	    !u->displacer || u->displacer == u || ia >= sa->count || ib >= sb->count ||
	    sa->unit[ia] != u || sb->unit[ib] != u->displacer ||
	    sa->history != a || sb->history != b || sa->queue != s->queue ||
	    !sb->queue || sb->queue->runtime != s->queue->runtime ||
	    sa->command[ia].type != s->command[index].type ||
	    sb->command[ib].type != s->command[index].type ||
	    (a == b && ib <= ia))
		return M4_USER_WHY(-ESTALE, "native reuse %u unit refs=%u published=%d accepted=%d idle=%d queue=%d history=%d/%u/%u displacement=%d/%u/%u displacer=%d stages=%d,%d units=%d,%d same=%d",
			index, refcount_read(&u->refs), u->published, u->accepted == s, u->idle,
			u->queue == s->queue, u->history == a, u->history_index, ia,
			u->displacement == b, u->displacement_index, ib,
			u->displacer && u->displacer != u, sa && sa->history == a, sb && sb->history == b,
			sa && ia < sa->count && sa->unit[ia] == u,
			sb && ib < sb->count && sb->unit[ib] == u->displacer, a == b);
	for (u32 e = 0; e < 3; e++)
		if (s->queue->runtime->user_unit_last[e] == u)
			return M4_USER_WHY(-ESTALE, "native reuse %u unit is engine %u last", index, e);
	ret = m4_user_native_history_command_locked(sa, ia, s->command[index].type, &ra);
	if (!ret)
		ret = m4_user_native_history_command_locked(sb, ib, s->command[index].type, &rb);
	if (ret)
		return M4_USER_WHY(ret, "native reuse %u histories A=%u B=%u", index, ia, ib);
	if (ra->uma_slot != rb->uma_slot || ra->parameter_id != rb->parameter_id ||
	    !m4_user_native_slot_valid(s->command[index].type, ra->uma_slot) ||
	    ra->parameter_id != m4_user_native_slot_parameter(s->command[index].type, ra->uma_slot))
		return M4_USER_WHY(-ESTALE, "native reuse %u slot A=%u/%u B=%u/%u", index, ra->uma_slot,
			ra->parameter_id, rb->uma_slot, rb->parameter_id);
	*origin = sa; *old_index = ia; *displacer = sb; *displacer_index = ib;
	return 0;
}

/* Warm per-pass backing only, after the caller's actual lane/binding/upper
 * gates and before either producer. The old CORE receipts and new protected
 * cache are checked here too; a copied displaced/completed flag is not enough.
 * Every fallible check precedes the first clear. Full allocator/UMA/notifier
 * storage, PB backing and Mesa resources are deliberately not reset here.
 */
static int m4_user_native_render_reset_locked(struct m4_user_native_stage *s,
	u32 index, struct gpu_m4_drm_provider_job *job)
{
	static const u32 roles[] = {
		GPU_M4_RENDER_TILEMAP, GPU_M4_RENDER_TPC, GPU_M4_RENDER_SCENE_METADATA,
		GPU_M4_RENDER_LAYER_METADATA, GPU_M4_RENDER_SCENE_LIST,
		GPU_M4_RENDER_PREEMPT_0, GPU_M4_RENDER_PREEMPT_1, GPU_M4_RENDER_PREEMPT_2,
		GPU_M4_RENDER_SCRATCH, GPU_M4_RENDER_FRAGMENT_AUXILIARY,
	};
	struct m4_user_native_bo *reset[ARRAY_SIZE(roles)];
	struct gpu_m4_drm_provider_job_view view = { };
	struct m4_gpu_provider_runtime *p;
	struct m4_user_native_stage *origin, *displacer;
	const struct m4_user_render_native *n;
	u32 count = 0, old_index, displacer_index;
	u64 bit;
	int ret;

	if (!s || !s->queue || !job || !s->count ||
	    s->count > GPU_M4_DRM_JOB_MAX_COMMANDS || index >= s->count ||
	    s->command[index].type != DRM_ASAHI_CMD_RENDER)
		return -ESTALE;
	p = s->queue->runtime;
	lockdep_assert_held(&p->hello->gpu->lock);
	if (p->error)
		return p->error;
	/* A reused unit is never a current slot owner: it was displaced by a
	 * distinct, fully retired same-slot replacement (checked below).
	 */
	if (!m4_user_native_reused(s, index) || !s->pending ||
	    s->pending->job != job || m4_user_inflight_slot_locked(p, job) < 0)
		return -ESTALE;
	bit = 1ULL << index;
	if (s->pending->render_reset_mask & bit)
		return -EALREADY;
	/* B releases A's immutable old use. The caller independently validated
	 * the actual current C slot/PB predecessor; B need not remain current or
	 * share A's client VM. Never inspect B's mutable old backing here.
	 */
	ret = m4_user_native_reuse_history_locked(s, index, &origin, &old_index,
		&displacer, &displacer_index);
	if (ret)
		return ret;
	gpu_m4_drm_provider_job_view(job, &view);
	if (view.stage_owner != s || view.runtime_queue != s->queue || !view.runtime_job ||
	    view.version != s->pending->version || view.snapshot != s->pending->snapshot ||
	    !view.batch || !view.batch->commands || view.batch->count != s->count ||
	    view.batch->commands[index].type != DRM_ASAHI_CMD_RENDER)
		return -ESTALE;
	ret = m4_user_native_render_check_locked(s, index, p->provider, view.version, view.snapshot);
	if (ret)
		return ret;
	n = &s->command[index].render;
	if (n->resource[GPU_M4_RENDER_LAYER_METADATA] != n->resource[GPU_M4_RENDER_SCENE_METADATA] ||
	    n->tpc != n->resource[GPU_M4_RENDER_TPC])
		return -ESTALE;
	for (u32 i = 0; i < ARRAY_SIZE(roles); i++) {
		struct m4_user_native_bo *b = n->resource[roles[i]];
		const struct gpu_m4_render_native_backing *owner = &n->owner.resource[roles[i]];
		bool found = false, duplicate = false;

		if (!b || !b->va || b->coverage != b->size || !b->mapped ||
		    iosys_map_is_null(&b->map) || b->map.is_iomem ||
		    ((unsigned long)b->map.vaddr & (GPU_M4_DRM_PAGE_SIZE - 1)) ||
		    gpu_m4_drm_bo_policy(b->gem) != GPU_M4_DRM_CPU_NC ||
		    owner->kind != GPU_M4_RENDER_NATIVE_GEM || owner->gem.object != b->object ||
		    owner->gem.lease != b->claim || owner->gem.va != b->va || owner->gem.offset ||
		    owner->gem.size != b->size || owner->gem.access != M4_USER_NATIVE_RW ||
		    b == s->notifier->bo || b == n->parameter_heap || b == n->manager)
			return -ESTALE;
		for (u32 k = 0; k < s->bo_count; k++)
			found |= s->bo[k] == b;
		if (!found)
			return -ESTALE;
		for (u32 k = 0; k < GPU_M4_RENDER_FIRST_GPU_COUNT; k++)
			if (n->gpu[k] && n->gpu[k]->gem == b->gem)
				return -ESTALE;
		for (u32 k = 0; k < GPU_M4_RENDER_FIRST_FW_COUNT; k++)
			if (k != GPU_M4_RENDER_FIRST_SCRATCH_FW && n->firmware[k] &&
			    n->firmware[k]->gem == b->gem)
				return -ESTALE;
		ret = gpu_m4_drm_provider_job_check_access_locked(job, p->provider,
			b->object, 0, b->size, M4_USER_NATIVE_RW, false, NULL);
		if (ret)
			return ret;
		for (u32 k = 0; k < count; k++)
			duplicate |= reset[k] == b;
		if (!duplicate)
			reset[count++] = b;
	}
	for (u32 i = 0; i < count; i++)
		memset(reset[i]->map.vaddr, 0, reset[i]->size);
	s->pending->render_reset_mask |= bit;
	dma_wmb();
	return 0;
}

/* Unit eligibility was committed by receipt_complete from the actual serial
 * packet retirements. Displacing a batch drops no unit metadata or backing.
 */
static void m4_user_native_stage_cache_locked(struct m4_user_native_stage *s,
	struct m4_user_native_stage *replacement)
{
	lockdep_assert_held(&s->queue->runtime->hello->gpu->lock);
	if (WARN_ON(!replacement || replacement == s || !replacement->history ||
	    !replacement->history->complete))
		return;
	s->displaced = true;
}

static void m4_user_native_prebuild_record_locked(struct m4_user_queue *q);
static void m4_user_native_spares_free_locked(struct m4_user_queue *q);

static void m4_user_native_queue_close_locked(struct m4_user_queue *q)
{
	lockdep_assert_held(&q->runtime->hello->gpu->lock);
#ifdef MODULE
	/* REMOVE-DEBUG: per-queue construction versus prebuilt spares. */
	if (q->constructed)
		pr_info_ratelimited("GPU queue close: comm=%s constructed=%u render=%u need_pool=%u need_heap=%u miss=%u prebuild_want=%u heaps=%u spares_left=%u\n",
			q->comm, q->constructed, q->constructed_render, q->need_pool,
			q->need_heap, q->spare_miss, q->prebuild_want, q->prebuild_heaps, q->spares);
#endif
	m4_user_native_prebuild_record_locked(q);
	m4_user_native_spares_free_locked(q);
	if (q->vm->prebuilder == q)
		q->vm->prebuilder = NULL;
	/* Each published unit owns its queue/VM and discoverable inventory.
	 * Closing this handle prohibits selection, not reclaim or orphaning.
	 * Close alone asserts no completion: already accepted work drains
	 * normally. Only schedule the deferred cold scan; the units may become
	 * cold later, after a displacement/root replacement, which reschedules.
	 */
	schedule_work(&q->runtime->user_reap);
}

static void m4_user_native_stage_exposed_locked(struct m4_user_native_stage *s)
{
	lockdep_assert_held(&s->queue->runtime->hello->gpu->lock);
	for (u32 i = 0; i < s->count; i++) {
		struct m4_user_native_unit *u = s->unit[i];

		if (WARN_ON(!u || u->accepted != s))
			continue;
		if (!u->published) {
			refcount_inc(&u->refs); /* Persistent discoverable backing baseline. */
			u->published = true;
			list_add_tail(&u->retained, &s->queue->runtime->user_native_units);
			list_add_tail(&u->queue_link, &u->queue->units);
		}
	}
	s->exposed = true; /* BEFORE first possible root/upper/work producer store. */
}

/* Infallible bookkeeping ONLY from the runtime's actual post-core-success
 * same-slot replacement/displacement callback. A fence on this owner, an error
 * fence, unrelated slot, closed VM, or copied boolean is NOT that authority.
 */
static void m4_user_native_stage_displaced_locked(struct m4_user_native_stage *s)
{
	struct m4_gpu_provider_runtime *p = s->queue->runtime;

	lockdep_assert_held(&p->hello->gpu->lock);
	s->displaced = true;
}

/* Teardown of a stage whose last reference dropped. Its source version may
 * still be protected/working, or its pending history unconsumed: then it is
 * retained (one synthetic ref, on user_native_retained) and every user_reap
 * retries here, so its BO/cell-wrapper/unit refs can never leak. Returns true
 * when the caller frees s outside the lock.
 */
static bool m4_user_native_stage_release_locked(struct m4_gpu_provider_runtime *p,
	struct m4_user_native_stage *s)
{
	lockdep_assert_held(&p->hello->gpu->lock);
	if ((s->exposed && s->pending) || gpu_m4_drm_provider_version_put_locked(s->source)) {
		refcount_set(&s->refs, 1);
		if (list_empty(&s->retained))
			list_add_tail(&s->retained, &p->user_native_retained);
		return false;
	}
	list_del_init(&s->retained);
	for (u32 i = 0; i < s->bo_count; i++) {
		if (!s->bo[i]) /* dropped after settlement */
			continue;
		if (!s->bo[i]->unit && s->bo[i]->upper)
			gpu_m4_drm_upper_put_locked(s->bo[i]->upper);
		m4_user_native_bo_put_locked(s->bo[i]);
	}
	for (u32 i = 0; i < s->count; i++) {
		struct m4_user_native_unit *u = s->unit[i];

		if (u && u->accepted == s) {
			/* No possible exposure: returning exactly the previously displaced
			 * use is safe, without manufacturing a new retirement receipt.
			 */
			u->accepted = NULL;
			u->idle = u->history && u->displacement;
		}
		m4_user_native_history_put_locked(s->reuse_history[i]);
		m4_user_native_history_put_locked(s->reuse_displacement[i]);
		m4_user_native_unit_put_locked(u);
	}
	m4_user_native_history_put_locked(s->pending);
	if (s->history_owned)
		m4_user_native_history_put_locked(s->history);
	m4_user_notifier_put_locked(s->notifier);
	m4_user_queue_put_locked(s->queue);
	p->user_native_bytes -= s->charge;
	/* A disposed borrower can have held the last unexplained reuse edge or
	 * accepted claim on a closed queue's cold component: rescan, deferred.
	 */
	schedule_work(&p->user_reap);
	return true;
}

/* The core job's own stage reference (its last pin borrower is gone). */
static void m4_user_native_stage_job_put(void *owner, void *stage_owner)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_user_native_stage *s = stage_owner;

	if (!s)
		return;
	mutex_lock(&p->hello->gpu->lock);
	s->job_gone = true;
	m4_user_native_stage_drop_bos_locked(s);
	mutex_unlock(&p->hello->gpu->lock);
	m4_user_native_stage_put(owner, stage_owner);
}

static void m4_user_native_stage_put(void *owner, void *stage_owner)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_user_native_stage *s = stage_owner;
	struct mutex *lock = &p->hello->gpu->lock;
	bool release = false;

	if (!s)
		return;
	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	m4_user_bank_unstage_locked(p, s); /* never accepted: its choice ends */
	if (!WARN_ON(s->queue->runtime != p) && refcount_dec_and_test(&s->refs))
		release = m4_user_native_stage_release_locked(p, s);
	mutex_unlock(lock);
	if (release)
		kvfree(s);
}


/* REMOVE-DEBUG: unit cache effectiveness, observational only, updated
 * under the GPU mutex and exported through runtime_perf. Rejections count
 * candidate units of the SAME queue and command type that a selection
 * passed over (a unit can count once per selection).
 */
enum m4_user_native_reject {
	M4_USER_NATIVE_REJECT_BUSY,	/* accepted, or not yet displaced */
	M4_USER_NATIVE_REJECT_CURRENT,	/* current firmware-last owner */
	M4_USER_NATIVE_REJECT_GEOMETRY,	/* no exact key and too small/other class */
	M4_USER_NATIVE_REJECT_SLOT,	/* compute slot already leased in child */
	M4_USER_NATIVE_REJECT_COLD,	/* evicted tombstone */
	M4_USER_NATIVE_REJECT_NONE,	/* no unit of this queue/type at all */
	M4_USER_NATIVE_REJECTS,
};
/* Evictions: closed queues, under pressure (cells/bytes/demand), or only
 * for the idle cache budgets, with the open-queue bytes they release.
 * Construction: deferred backpressure waits (-EAGAIN, materializer retries
 * after reclaim) and refusals returned to an eager caller; every NEW unit's
 * backing construction attempt (render/compute allocate, unlocked factory
 * work and locked claim/upper admissions together), its completions, its
 * failures (the stage then rolls back), total and maximum wall time.
 */
static struct {
	u64 units_new, reused_exact, reused_fit, retargeted, evicted_closed, evicted_pressure;
	u64 evicted_cache, evicted_bytes, construct_waits, construct_refusals;
	u64 construct_attempts, construct_done, construct_rollbacks, construct_ns;
	u64 construct_max;
	/* Cold cuts: firmware aliases released or refused (kept, latched or not
	 * permitted), cuts that released any, and fully drained tombstones freed.
	 */
	u64 upper_released, upper_release_refused, cold_released, cold_freed;
	/* Retained stages finally released by a user_reap retry. */
	u64 stages_retried;
	/* REMOVE-DEBUG: consecutive cell-credit deferrals (reset on success),
	 * consecutive byte deferrals/backing refusals (reset once the estimate
	 * fits), and constructions started below the estimated byte headroom
	 * because no eviction could make room (the real charges then decided).
	 */
	u32 cell_defers, byte_defers;
	u64 byte_proceeds;
	/* Cold cell wrappers whose credit returned while retired receipts'
	 * stages still held them (cell_return_locked).
	 */
	u64 cells_released_held;
	/* Witness edges moved forward, and cell shortages waited for because
	 * in-flight or still-borrowed units will return cells (not refusals).
	 */
	u64 witness_advances, busy_waits;
	/* Housekeeping collections run, deferred (batched into a later pass)
	 * and their time under the provider mutex.
	 */
	u64 collect_runs, collect_deferred, collect_ns, collect_max;
	/* Fresh units placed on a closed client's owned slot instead of the
	 * lowest free one.
	 */
	u64 slot_steered;
	u64 reject[M4_USER_NATIVE_REJECTS];
} m4_user_native_unit_stats;

static ssize_t m4_user_native_unit_stats_show(char *buf, ssize_t size)
{
	ssize_t start = size;

	size += sysfs_emit_at(buf, size,
		" units_prebuilt=%llu units_new=%llu units_reused_exact=%llu units_reused_fit=%llu units_retargeted=%llu units_evicted_closed=%llu units_evicted_pressure=%llu units_evicted_cache=%llu units_evicted_bytes=%llu construct_waits=%llu construct_refusals=%llu reuse_reject_busy=%llu reuse_reject_current=%llu reuse_reject_geometry=%llu reuse_reject_slot=%llu reuse_reject_cold=%llu reuse_reject_none=%llu",
		m4_user_native_prebuilt,
		m4_user_native_unit_stats.units_new, m4_user_native_unit_stats.reused_exact,
		m4_user_native_unit_stats.reused_fit, m4_user_native_unit_stats.retargeted,
		m4_user_native_unit_stats.evicted_closed,
		m4_user_native_unit_stats.evicted_pressure, m4_user_native_unit_stats.evicted_cache,
		m4_user_native_unit_stats.evicted_bytes, m4_user_native_unit_stats.construct_waits,
		m4_user_native_unit_stats.construct_refusals,
		m4_user_native_unit_stats.reject[M4_USER_NATIVE_REJECT_BUSY],
		m4_user_native_unit_stats.reject[M4_USER_NATIVE_REJECT_CURRENT],
		m4_user_native_unit_stats.reject[M4_USER_NATIVE_REJECT_GEOMETRY],
		m4_user_native_unit_stats.reject[M4_USER_NATIVE_REJECT_SLOT],
		m4_user_native_unit_stats.reject[M4_USER_NATIVE_REJECT_COLD],
		m4_user_native_unit_stats.reject[M4_USER_NATIVE_REJECT_NONE]);
	size += sysfs_emit_at(buf, size,
		" construct_attempts=%llu construct_done=%llu construct_rollbacks=%llu construct_ns=%llu construct_max=%llu",
		m4_user_native_unit_stats.construct_attempts,
		m4_user_native_unit_stats.construct_done,
		m4_user_native_unit_stats.construct_rollbacks,
		m4_user_native_unit_stats.construct_ns, m4_user_native_unit_stats.construct_max);
	size += sysfs_emit_at(buf, size,
		" cold_upper_released=%llu cold_upper_refused=%llu cold_units_released=%llu cold_units_freed=%llu stages_retried=%llu cell_defers=%u byte_defers=%u byte_proceeds=%llu cells_released_held=%llu",
		m4_user_native_unit_stats.upper_released,
		m4_user_native_unit_stats.upper_release_refused,
		m4_user_native_unit_stats.cold_released, m4_user_native_unit_stats.cold_freed,
		m4_user_native_unit_stats.stages_retried, m4_user_native_unit_stats.cell_defers,
		m4_user_native_unit_stats.byte_defers, m4_user_native_unit_stats.byte_proceeds,
		m4_user_native_unit_stats.cells_released_held);
	size += sysfs_emit_at(buf, size, " witness_advances=%llu busy_waits=%llu",
		m4_user_native_unit_stats.witness_advances, m4_user_native_unit_stats.busy_waits);
	size += sysfs_emit_at(buf, size,
		" collect_runs=%llu collect_deferred=%llu collect_ns=%llu collect_max=%llu slot_steered=%llu",
		m4_user_native_unit_stats.collect_runs, m4_user_native_unit_stats.collect_deferred,
		m4_user_native_unit_stats.collect_ns, m4_user_native_unit_stats.collect_max,
		m4_user_native_unit_stats.slot_steered);
	return size - start;
}

/* user_reap: retry every retained stage (see stage_release_locked). */
static void m4_user_native_retained_reap(struct m4_gpu_provider_runtime *p)
{
	struct mutex *lock = &p->hello->gpu->lock;
	struct m4_user_native_stage *s, *next;
	struct list_head freed;

	lockdep_assert_not_held(lock);
	INIT_LIST_HEAD(&freed);
	mutex_lock(lock);
	list_for_each_entry_safe(s, next, &p->user_native_retained, retained) {
		if (refcount_read(&s->refs) != 1 || s->queue->runtime != p)
			continue;
		if (m4_user_native_stage_release_locked(p, s)) {
			m4_user_native_unit_stats.stages_retried++;
			list_add_tail(&s->retained, &freed);
		}
	}
	mutex_unlock(lock);
	list_for_each_entry_safe(s, next, &freed, retained)
		kvfree(s);
}

/* Exact cache key: preferred, no slack. */
static bool m4_user_native_unit_matches(const struct m4_user_native_unit *u,
	const struct gpu_m4_drm_job_command *command)
{
	if (u->command.type != command->type)
		return false;
	if (command->type == DRM_ASAHI_CMD_COMPUTE)
		return true;
	if (command->type == DRM_ASAHI_CMD_RENDER) {
		const struct drm_asahi_cmd_render *r = &command->render.uapi;
		struct gpu_m4_render_geometry key = { r->width_px, r->height_px, r->layers,
			r->samples, r->utile_width_px, r->utile_height_px, r->sample_size_B };

		return !memcmp(&key, &u->command.render_key, sizeof(key));
	}
	return false;
}

/* Exact geometry equality is cache policy, not hardware authority. The
 * render encoder recomputes geometry and PB layout from each COMMAND and only
 * requires every geometry-dependent backing (tilemap, TPC, parameter heap and
 * the heap-capacity-derived PB lists) to be at least that large, adopting the
 * unit's actual larger heap capacity for the PB manager/InitBuffer rebuild.
 * Samples, utile size and sample stride only scale those sizes (and the
 * on-chip tile buffer the command encodes); the PB layout follows the screen
 * tiles. So a displaced render unit of the same layer class whose retained
 * sizes cover this command fits without rebuilding anything: its owned
 * scene/layer metadata subrecord layout depends only on that class (the
 * encoder rejects any other). Width/height may shrink or stay within the
 * retained capacity. A unit never grows: its BO/claim identities, baseline
 * membership and firmware-visible upper leases are immutable once built.
 */
static bool m4_user_native_unit_fits(const struct m4_user_native_unit *u,
	const struct gpu_m4_drm_job_command *command,
	const struct gpu_m4_exec_geometry *geometry, const struct gpu_m4_render_pb_layout *pb)
{
	const struct gpu_m4_render_geometry *k = &u->command.render_key;
	const struct drm_asahi_cmd_render *r = &command->render.uapi;
	const struct m4_user_render_native *n = &u->command.render;

	return u->command.type == DRM_ASAHI_CMD_RENDER && command->type == DRM_ASAHI_CMD_RENDER &&
		geometry && pb && (k->layers > 1) == (r->layers > 1) &&
		n->geometry.tilemap_size >= geometry->tilemap_size &&
		n->geometry.tpc_size >= geometry->tpc_size &&
		n->pb.heap_size >= pb->heap_size &&
		n->parameter_heap && n->parameter_heap->size >= n->pb.heap_size &&
		n->tpc && n->tpc->size >= geometry->tpc_size &&
		n->resource[GPU_M4_RENDER_TILEMAP] &&
		n->resource[GPU_M4_RENDER_TILEMAP]->size >= geometry->tilemap_size;
}

/* A render command's required geometry-dependent sizes (pure; no lock). */
static bool m4_user_native_render_need(const struct gpu_m4_drm_job_command *command,
	struct gpu_m4_exec_geometry *geometry, struct gpu_m4_render_pb_layout *pb,
	const struct m4_user_queue *q)
{
	const struct drm_asahi_cmd_render *r = &command->render.uapi;
	struct gpu_m4_render_geometry key = { r->width_px, r->height_px, r->layers,
		r->samples, r->utile_width_px, r->utile_height_px, r->sample_size_B };

	return command->type == DRM_ASAHI_CMD_RENDER && !gpu_m4_render_geometry(geometry, &key) &&
		!m4_user_render_pb_layout(pb, &key, q);
}

/* Original slot (render: and PB ID) of an already selected warm command of
 * this stage, from its own completed use.
 */
static u32 m4_user_native_reused_slot(const struct m4_user_native_stage *s, u32 index)
{
	const struct m4_user_native_history *h;
	u16 type;

	if (!m4_user_native_reused(s, index))
		return 0;
	if (s->retargeted & (1ULL << index))
		return s->retarget_slot[index];
	type = s->command[index].type;
	h = s->reuse_history[index];
	if ((type != DRM_ASAHI_CMD_COMPUTE && type != DRM_ASAHI_CMD_RENDER) ||
	    !h || !h->complete || s->reuse_index[index] >= h->count ||
	    h->command[s->reuse_index[index]].type != type)
		return 0;
	return h->command[s->reuse_index[index]].uma_slot;
}

/* Compute (1..32) and render (33..64) slot domains are disjoint. */
static bool m4_user_native_slot_taken_locked(const struct m4_user_native_stage *s,
	u32 index, u32 slot)
{
	for (u32 i = 0; i < index && i < s->count; i++)
		if (m4_user_native_reused_slot(s, i) == slot)
			return true;
	return false;
}

/* Actual current owner of one compute or render UMA slot (render: with its
 * paired PB ID), derived from the retained units themselves rather than an
 * engine-global pointer: the unique published, non-cold, unborrowed unit
 * whose latest COMPLETED (CORE-receipted) use names that slot and that no
 * later completed use of the same slot displaced. A unit of a still-protected
 * job is never an owner here; its use becomes one only in receipt_complete.
 * Its own history keeps job/stage/BO alive, and the cold collector never cuts
 * an undisplaced unit. -ENOENT: slot never used.
 */
static int m4_user_native_slot_owner_locked(struct m4_gpu_provider_runtime *p,
	u16 type, u32 slot, struct m4_user_native_unit **owner)
{
	struct m4_user_native_unit *u;
	const struct m4_user_native_history *h;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!owner || !m4_user_native_slot_valid(type, slot))
		return -EINVAL;
	*owner = NULL;
	/* Every other unit that ever used this slot was displaced by a later
	 * committed use (or is cold), so only the table's latest committed user
	 * can satisfy the owner predicate: O(1), no walk of the retained units.
	 */
	u = slot < M4_USER_SLOT_TABLE ? p->user_slot_unit[slot] : NULL;
	if (!u)
		return -ENOENT;
	h = u->history;
	if (u->cold || !u->published || u->displacement || u->accepted || !h ||
	    !h->complete || u->command.type != type ||
	    u->history_index >= h->count ||
	    h->command[u->history_index].type != type ||
	    h->command[u->history_index].uma_slot != slot)
		return -ENOENT;
	*owner = u;
	return 0;
}

/* Some unit's latest completed use in s is still the current owner of its
 * slot: s keeps firmware-last owner state and is never a cold component.
 */
static bool m4_user_native_stage_current_locked(const struct m4_user_native_stage *s)
{
	for (u32 i = 0; i < s->count; i++) {
		const struct m4_user_native_unit *u = s->unit[i];

		if (u && !u->cold && u->published && !u->displacement && u->history &&
		    u->history == s->history && u->history_index == i)
			return true;
	}
	return false;
}

/* Select one whole backing unit, not its original batch shape/ordinal. All
 * references and vector capacity are checked BEFORE any acceptance transfer.
 * Immutable A/B histories remain independently held through rollback or new
 * CORE completion; neither is assumed to still be the global current owner.
 */
static bool m4_user_native_retargeted(const struct m4_user_native_stage *s, u32 index)
{
	return s && index < GPU_M4_DRM_JOB_MAX_COMMANDS && (s->retargeted & (1ULL << index));
}

/* Idle-owner retargeting (A/B switch). */
static bool slot_retarget = true;
module_param(slot_retarget, bool, 0644);
/* Warm units retarget into exited clients' last-owner slots before keeping
 * their own (slot_closed_retarget): reuse then displaces those owners.
 */
static bool slot_closed_retarget = true;
module_param(slot_closed_retarget, bool, 0644);
MODULE_PARM_DESC(slot_closed_retarget, "Retarget warm units into exited clients' last-owner slots first (default 1)");
MODULE_PARM_DESC(slot_retarget, "Reuse a displaced idle unit in another slot of the stage's bank (default 1)");

/* A destination for moving a warm unit: a slot of the stage's bank no
 * earlier command of the stage uses, whose current owner is a retired,
 * unborrowed unit (it stays retained until the moved unit retires there).
 * This queue's own owners first. 0 when none.
 */
static u32 m4_user_native_retarget_slot_locked(struct m4_user_native_stage *s, u32 index,
	u16 type)
{
	struct m4_gpu_provider_runtime *p = s->queue->runtime;
	u32 first = (type == DRM_ASAHI_CMD_RENDER ? GPU_M4_RENDER_UMA_SLOT_FIRST : 1) +
		s->bank * m4_user_bank_size(p);
	u32 other = 0, own = 0;

	for (u32 n = 0; n < m4_user_bank_slots(p); n++) {
		u32 slot = first + n;
		struct m4_user_native_unit *c;
		bool used = false;

		for (u32 i = 0; i < index && i < s->count && !used; i++)
			used = s->command[i].type == type && m4_user_native_reused_slot(s, i) == slot;
		if (used || m4_user_native_slot_owner_locked(p, type, slot, &c) || !c->history ||
		    !c->history->complete || c->accepted || c->cold)
			continue;
		/* An exited client's last owner first: its displacement frees
		 * that client's retained windows.
		 */
		if (c->queue && c->queue->closed && READ_ONCE(slot_closed_retarget))
			return slot;
		if (c->queue == s->queue && !own)
			own = slot;
		if (!other && c->queue != s->queue)
			other = slot;
	}
	return own ?: other;
}

static int m4_user_native_unit_take(struct m4_user_native_stage *s, u32 index,
	const struct gpu_m4_drm_job_command *command)
{
	struct m4_gpu_provider_runtime *p = s->queue->runtime;
	struct mutex *lock = &p->hello->gpu->lock;
	struct m4_user_native_unit *u;
	struct gpu_m4_exec_geometry geometry;
	struct gpu_m4_render_pb_layout pb;
	u32 passes = 1, total, retarget_slot = 0;
	bool seen = false, retarget_first = false;
	u32 why[M4_USER_NATIVE_REJECTS] = { }, units = 0;
	int ret = 0;

	/* Render: an exact key first, then any displaced unit whose retained
	 * geometry/PB capacity covers this command (see unit_fits). Last, a
	 * displaced unit whose own slot is unusable here moves to another slot
	 * of this bank (retarget pass).
	 */
	if (m4_user_native_render_need(command, &geometry, &pb, s->queue)) {
		passes = 2;
		m4_user_render_highwater_note(s->queue, command->render.uapi.layers > 1,
			&geometry, &pb);
	}
	total = passes + !!READ_ONCE(slot_retarget);
	mutex_lock(lock);
	if (p->error || s->queue->closed || s->queue->vm->closed) {
		ret = p->error ?: -ESHUTDOWN;
		goto out;
	}
	/* An exited client's last owner in this bank: retarget a warm unit
	 * there first (whatever its own slot), so reuse displaces it.
	 */
	if (total > passes && READ_ONCE(slot_closed_retarget)) {
		struct m4_user_native_unit *c;

		retarget_slot = m4_user_native_retarget_slot_locked(s, index, command->type);
		retarget_first = retarget_slot &&
			!m4_user_native_slot_owner_locked(p, command->type, retarget_slot, &c) &&
			c->queue && c->queue->closed;
		if (!retarget_first)
			retarget_slot = 0;
	}
	for (u32 step = 0; step < total && !s->unit[index]; step++)
	list_for_each_entry(u, &s->queue->units, queue_link) {
		/* With retarget_first the retarget pass (normally last) runs first. */
		u32 pass = !retarget_first ? step : step ? step - 1 : passes;
		struct m4_user_native_history *a = u->history, *b = u->displacement;
		struct m4_user_native_stage *sa, *sb;
		const struct m4_user_native_receipt *ra, *rb;
		u32 nb = 0, nm = 0, ne = 0;
		bool current_owner = false, last = step + 1 == total, retarget = pass >= passes;

		if (u->queue != s->queue || u->command.type != command->type)
			continue;
		seen = true;
		units += last;
		/* A cold tombstone owns no history: reject it before following
		 * any old command/history/displacer pointer.
		 */
		if (u->cold) {
			m4_user_native_unit_stats.reject[M4_USER_NATIVE_REJECT_COLD] += last; why[M4_USER_NATIVE_REJECT_COLD] += last;
			continue;
		}
		for (u32 e = 0; e < 3; e++)
			current_owner |= p->user_unit_last[e] == u;
		if (current_owner) {
			m4_user_native_unit_stats.reject[M4_USER_NATIVE_REJECT_CURRENT] += last; why[M4_USER_NATIVE_REJECT_CURRENT] += last;
			continue;
		}
		if (!u->published || !u->idle || u->accepted) {
			m4_user_native_unit_stats.reject[M4_USER_NATIVE_REJECT_BUSY] += last; why[M4_USER_NATIVE_REJECT_BUSY] += last;
			continue;
		}
		if ((pass && command->type == DRM_ASAHI_CMD_RENDER) ?
		    !m4_user_native_unit_fits(u, command, &geometry, &pb) :
		    !m4_user_native_unit_matches(u, command)) {
			m4_user_native_unit_stats.reject[M4_USER_NATIVE_REJECT_GEOMETRY] += last; why[M4_USER_NATIVE_REJECT_GEOMETRY] += last;
			continue;
		}
		if (!a || !b || !u->displacer || u->displacer == u ||
		    !(sa = a->owner_stage) || !(sb = b->owner_stage) ||
		    sa->queue != s->queue || !sb->queue || sb->queue->runtime != p ||
		    sa->history != a || sb->history != b ||
		    u->history_index >= sa->count || u->displacement_index >= sb->count ||
		    sa->unit[u->history_index] != u ||
		    sb->unit[u->displacement_index] != u->displacer ||
		    (a == b && u->displacement_index <= u->history_index)) {
			ret = -ESTALE;
			goto out;
		}
		ret = m4_user_native_history_command_locked(sa, u->history_index, command->type, &ra);
		if (!ret)
			ret = m4_user_native_history_command_locked(sb, u->displacement_index, command->type, &rb);
		if (ret)
			goto out;
		if (ra->uma_slot != rb->uma_slot || ra->parameter_id != rb->parameter_id ||
		    !m4_user_native_slot_valid(command->type, ra->uma_slot) ||
		    ra->parameter_id != m4_user_native_slot_parameter(command->type, ra->uma_slot) ||
		    !u->bo_count || u->bo_count > ARRAY_SIZE(u->bo)) {
			ret = -ESTALE;
			goto out;
		}
		/* A warm unit keeps its ORIGINAL slot (render: and PB ID) when
		 * that slot is in this bank and no other command of the child
		 * leases it; otherwise only the retarget pass moves it to a
		 * destination slot of this bank with a retired current owner.
		 */
		if (!m4_user_native_bank_slot(p, command->type, s->bank, ra->uma_slot) ||
		    m4_user_native_slot_taken_locked(s, index, ra->uma_slot)) {
			if (!retarget) {
				m4_user_native_unit_stats.reject[M4_USER_NATIVE_REJECT_SLOT] += last; why[M4_USER_NATIVE_REJECT_SLOT] += last;
				continue;
			}
			if (!retarget_slot)
				retarget_slot = m4_user_native_retarget_slot_locked(s, index,
					command->type);
			if (!retarget_slot) {
				m4_user_native_unit_stats.reject[M4_USER_NATIVE_REJECT_SLOT] += last; why[M4_USER_NATIVE_REJECT_SLOT] += last;
				continue;
			}
		} else if (retarget && !retarget_first) {
			continue; /* An earlier pass already rejected it otherwise. */
		}
		if (!refcount_read(&u->refs) || refcount_read(&u->refs) == U32_MAX ||
		    !refcount_read(&a->refs) || !refcount_read(&b->refs) ||
		    refcount_read(&a->refs) > U32_MAX - (a == b ? 2 : 1) ||
		    refcount_read(&b->refs) == U32_MAX) {
			ret = -EOVERFLOW;
			goto out;
		}
		for (u32 k = 0; k < u->bo_count; k++) {
			struct m4_user_native_bo *bo = u->bo[k];
			bool found = false;

			if (!bo || bo->unit != u || bo->runtime != p ||
			    bo->vm != s->queue->vm->provider_vm ||
			    !refcount_read(&bo->refs) || refcount_read(&bo->refs) == U32_MAX) {
				ret = -ESTALE;
				goto out;
			}
			for (u32 j = 0; j < s->bo_count; j++)
				found |= s->bo[j] == bo;
			if (!found) {
				nb++;
				nm += !!bo->va;
				ne += !bo->va;
			}
		}
		if (nb > M4_USER_NATIVE_BO_MAX - s->bo_count ||
		    nm > M4_USER_NATIVE_MAP_MAX - s->native_count ||
		    ne > M4_USER_NATIVE_EXTRA_MAX - s->extra_count) {
			ret = -E2BIG;
			goto out;
		}
		/* Explicit sync: no private native reservation fence orders this
		 * reuse after the unit's previous use. Selection alone does: only an
		 * idle, displaced unit, whose last use and displacing use both
		 * completed their full CORE receipt (retirement-ready under this
		 * mutex, before its fence is signalled outside it), is taken.
		 */
		WARN_ON(!u->published || !u->idle || u->accepted || !u->displacer ||
			u->displacement != b || u->history != a || !a->complete ||
			!b->complete || (!a->job && !a->settled) || (!b->job && !b->settled));
		refcount_inc(&u->refs);
		refcount_inc(&a->refs);
		refcount_inc(&b->refs);
		u->accepted = s;
		u->idle = false;
		s->unit[index] = u;
		s->reuse_history[index] = a;
		s->reuse_displacement[index] = b;
		s->reuse_index[index] = u->history_index;
		s->displacement_index[index] = u->displacement_index;
		s->reused_commands |= 1ULL << index;
		s->command[index] = u->command;
		/* This is a NEW admission, not a retargeted old batch memo. All
		 * immutable BO/claim identities stay unchanged, the membership
		 * cookie belongs to this new job's captured stage.
		 */
		if (command->type == DRM_ASAHI_CMD_RENDER)
			s->command[index].render.owner.stage_owner = s;
		for (u32 k = 0; k < u->bo_count; k++) {
			struct m4_user_native_bo *bo = u->bo[k];
			bool found = false;

			for (u32 j = 0; j < s->bo_count; j++)
				found |= s->bo[j] == bo;
			if (found)
				continue;
			refcount_inc(&bo->refs);
			s->bo[s->bo_count++] = bo;
			if (bo->va)
				s->native[s->native_count++] = bo->mapping;
			else /* Same coverage/purpose as its fresh construction. */
				s->extra[s->extra_count++] = m4_user_native_extra(bo);
		}
		if (retarget) {
			s->retargeted |= 1ULL << index;
			s->retarget_slot[index] = retarget_slot;
			m4_user_native_unit_stats.retargeted++;
		} else if (pass) {
			m4_user_native_unit_stats.reused_fit++;
		} else {
			m4_user_native_unit_stats.reused_exact++;
		}
		break; /* Exclusive accepted identity prevents selecting it twice. */
	}
	if (!seen && !s->unit[index])
		m4_user_native_unit_stats.reject[M4_USER_NATIVE_REJECT_NONE]++;
	(void)units; (void)why;
#ifdef CONFIG_DEBUG_FS
	/* REMOVE-DEBUG: why this command constructs (type, its units, rejects). */
	if (!s->unit[index])
		gpu_m4_trace(GPU_M4_TRACE_PHASE, 16, (u64)(uintptr_t)s->queue,
			((u64)command->type << 32) | units,
			((u64)why[M4_USER_NATIVE_REJECT_BUSY] << 48) |
			((u64)why[M4_USER_NATIVE_REJECT_CURRENT] << 32) |
			((u64)why[M4_USER_NATIVE_REJECT_GEOMETRY] << 16) |
			why[M4_USER_NATIVE_REJECT_SLOT],
			((u64)why[M4_USER_NATIVE_REJECT_COLD] << 32) |
			(command->type == DRM_ASAHI_CMD_RENDER ?
			 ((u64)command->render.uapi.width_px << 16 | command->render.uapi.height_px) : 0));
#endif
out:
	mutex_unlock(lock);
	return ret;
}

/* Closed-queue cold component reclamation.
 *
 * A published unit closes the cycle unit -> history -> actual job -> stage ->
 * unit (plus stage -> BO). Queue close cannot break it and unit_put refuses a
 * published final drop. This deferred collector performs one explicit checked
 * reference cut for a CLOSED queue's wholly displaced, unborrowed units, using
 * only existing successful CORE receipts. It never unmaps, remaps or returns a
 * cell, resets allocator bytes or writes firmware. Actual release is the
 * normal history -> job -> stage/version/object/BO reapers, outside the lock.
 */

/* Keep this many arena cells free or pending return (two concurrent render
 * windows). Below it, or below a waiting materializer's demand, eligible idle
 * units of OPEN queues are evicted too: a global cache, not per-client bands.
 */
#define M4_USER_NATIVE_CELL_LOW_WATER (4U * GPU_M4_DRM_PROVIDER_WINDOW_MAX_PER_CLASS)
/* Backing headroom kept in both native backing budgets (runtime native and
 * GEM factory): one full execution window of new units. Below it (or below a
 * waiting materializer's byte demand) idle open-queue units are trimmed too.
 */
#define M4_USER_NATIVE_BYTE_LOW_WATER \
	((u64)GPU_M4_DRM_PROVIDER_WINDOW_MAX_PER_CLASS * M4_ORDINARY_NATIVE_UNIT_BYTES)

/* Structural candidate only, no authority: the same lineage selection uses.
 * A current owner/render stage or accepted borrower never cuts. Queue state
 * is the CALLER's selection: closed queues always, open ones under pressure.
 */
static bool m4_user_native_cold_candidate_locked(struct m4_gpu_provider_runtime *p,
	struct m4_user_native_unit *u)
{
	struct m4_user_native_history *a = u->history, *b = u->displacement;
	struct m4_user_native_stage *sa, *sb;

	if (u->cold || !u->published || u->accepted || !u->queue || u->queue->runtime != p ||
	    !refcount_read(&u->refs) || !u->bo_count ||
	    u->bo_count > ARRAY_SIZE(u->bo) || !a || !b || !a->complete || !b->complete ||
	    a->runtime != p || b->runtime != p ||
	    !refcount_read(&a->refs) || !refcount_read(&b->refs) ||
	    !u->displacer || u->displacer == u ||
	    !(sa = a->owner_stage) || !(sb = b->owner_stage) ||
	    sa->history != a || sb->history != b || sa->queue != u->queue ||
	    u->history_index >= sa->count || u->displacement_index >= sb->count ||
	    sa->unit[u->history_index] != u || sb->unit[u->displacement_index] != u->displacer ||
	    sa->command[u->history_index].type != u->command.type ||
	    sb->command[u->displacement_index].type != u->command.type ||
	    (a == b && u->displacement_index <= u->history_index))
		return false;
	for (u32 e = 0; e < 3; e++)
		if (p->user_unit_last[e] == u || p->user_last[e] == sa)
			return false;
	/* A settled window no longer holds its units' backing: a current
	 * stage-mate pins only its own.
	 */
	return a->settled || !m4_user_native_stage_current_locked(sa);
}

/* A history about to lose its LAST reference. Its owner stage must be a
 * completed, root-free component with no current or accepted unit (even one
 * that moved on: conservative first cut), and both the actual job origin
 * version and the stage source must be nonresident, unpending, unprotected
 * and idle. Only the real root replacement removes residency, never this.
 */
static int m4_user_native_cold_history_locked(struct m4_gpu_provider_runtime *p,
	struct m4_user_native_history *h)
{
	struct m4_user_native_stage *s = h->owner_stage;
	struct gpu_m4_drm_provider_vm *vm;
	int ret;

	if (!h->complete || (!h->job && !h->settled) || h->runtime != p || !s || s->history != h ||
	    s->pending || s->history_owned || !refcount_read(&s->refs) || !s->queue ||
	    s->queue->runtime != p || !s->queue->vm || h->count != s->count)
		return -ESTALE;
	if (m4_user_native_stage_current_locked(s))
		return -EBUSY;
	for (u32 e = 0; e < 3; e++)
		if (p->user_last[e] == s)
			return -EBUSY;
	for (u32 i = 0; i < s->count; i++) {
		struct m4_user_native_unit *u = s->unit[i];

		if (!u || u->accepted)
			return -EBUSY;
		for (u32 e = 0; e < 3; e++)
			if (p->user_unit_last[e] == u)
				return -EBUSY;
	}
	vm = s->queue->vm->provider_vm;
	/* A settled history holds no job version any more. */
	ret = h->settled ? 0 :
		gpu_m4_drm_provider_version_quiescent_locked(p->provider, vm, h->version);
	if (!ret && s->source != h->version)
		ret = gpu_m4_drm_provider_version_quiescent_locked(p->provider, vm, s->source);
	return ret;
}

/* Authority: A's and B's exact selected commands through the real retained
 * CORE/cache phase and receipt checks (render: the saved paired path), the
 * same actual slot and paired PB ID. Never a fence, !uncertain or live
 * shared stamps.
 */
static int m4_user_native_cold_proof_locked(struct m4_user_native_unit *u)
{
	const struct m4_user_native_receipt *ra, *rb;
	u16 type = u->command.type;
	int ret;

	ret = m4_user_native_history_command_locked(u->history->owner_stage,
		u->history_index, type, &ra);
	if (!ret)
		ret = m4_user_native_history_command_locked(u->displacement->owner_stage,
			u->displacement_index, type, &rb);
	if (ret)
		return ret;
	if (ra->uma_slot != rb->uma_slot || ra->parameter_id != rb->parameter_id ||
	    !m4_user_native_slot_valid(type, ra->uma_slot) ||
	    ra->parameter_id != m4_user_native_slot_parameter(type, ra->uma_slot))
		return -ESTALE;
	return 0;
}

/* Every reference to h is one of the marked edges being detached. Counting
 * EDGES, so a same-batch A == B contributes two. Any unit, reuse, pending or
 * history-owned reference outside the marked set keeps h (and its job/stage).
 */
static bool m4_user_native_cold_dying(const struct m4_user_native_history *h)
{
	return h->cold_edges == refcount_read(&h->refs);
}

/* Recount the marked edges, then unmark any unit whose own latest history
 * would survive or whose dying history/witness was refused. Marks only ever
 * decrease, so this converges. Surviving foreign witnesses are permitted:
 * only the cold unit's reference to such a live B is dropped.
 */
static void m4_user_native_cold_settle_locked(struct m4_gpu_provider_runtime *p)
{
	struct m4_user_native_unit *u;
	bool changed;

	do {
		changed = false;
		list_for_each_entry(u, &p->user_native_units, retained) {
			if (u->history)
				u->history->cold_edges = 0;
			if (u->displacement)
				u->displacement->cold_edges = 0;
		}
		list_for_each_entry(u, &p->user_native_units, retained) {
			if (!u->cold_mark)
				continue;
			u->history->cold_edges++;
			u->displacement->cold_edges++;
		}
		list_for_each_entry(u, &p->user_native_units, retained) {
			struct m4_user_native_history *a = u->history, *b = u->displacement;

			/* A settled window does not hold the unit's backing: cutting it
			 * pays even while its history survives through stage-mates.
			 */
			if (u->cold_mark && ((!m4_user_native_cold_dying(a) && !a->settled) ||
			    a->cold_blocked ||
			    (m4_user_native_cold_dying(b) && b->cold_blocked))) {
				u->cold_mark = false;
				changed = true;
			}
		}
	} while (changed);
}

/* Release a cold unit's own firmware alias (never an arena cell's seed
 * lease) under the cut's proven authority: its own and a distinct same-slot
 * replacement's actual CORE retirement, not a current slot owner, not in
 * flight, no surviving history holder or batch-mate, healthy session. The
 * upper owner additionally requires the only reference and an unfaulted owner;
 * its failure is latched there (provider fault) and the lease is retained.
 */
static int m4_user_native_upper_release_locked(struct m4_user_native_bo *b)
{
	struct m4_gpu_provider_runtime *p = b->runtime;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (p->error || p->ready->error || b->cell || !b->upper)
		return -EPERM;
	ret = gpu_m4_drm_upper_release_locked(b->upper, &b->backing);
	if (ret)
		return ret;
	b->upper = NULL;
	b->fw_va = 0;
	return 0;
}

/* Infallible cut of one checked cold/evicted unit. The tombstone keeps its
 * identity, queue ref, published membership and, only until the pending-cell
 * return below, each arena cell wrapper (claim/BO wrapper and its
 * unit-acquired seed upper ref). Every other BO loses only this unit's ref
 * (stage, version and snapshot refs still protect it until normal
 * destruction); an upper-leased one first has its firmware alias released
 * (cleared, invalidated, VA freed, backing ref dropped at reap). A refused or
 * failed release keeps that BO and its visible lease on the tombstone. A
 * dropped BO no longer names the unit: its lease ref, if any, was released,
 * so the stage destructor never puts a lease it does not own.
 */
static void m4_user_native_cold_cut_locked(struct m4_user_native_unit *u)
{
	struct m4_user_native_history *a = u->history, *b = u->displacement;
	struct m4_gpu_provider_runtime *p = u->queue->runtime;
	u32 kept = 0;
	bool released = false;

	u->history = u->displacement = NULL;
	u->displacer = NULL;
	u->idle = false;
	u->cold = true;
	m4_user_native_slot_forget_locked(p, u);
	/* Old allocator/native owner pointers are no longer this unit's. */
	u->command = (struct m4_user_native_command) {
		.type = u->command.type, .render_key = u->command.render_key,
	};
	for (u32 i = 0; i < u->bo_count; i++) {
		struct m4_user_native_bo *bo = u->bo[i];

		if (bo->upper && !bo->cell) {
			if (m4_user_native_upper_release_locked(bo)) {
				m4_user_native_unit_stats.upper_release_refused++;
				u->bo[kept++] = bo;
				continue;
			}
			m4_user_native_unit_stats.upper_released++;
			released = true;
		}
		if (bo->cell) {
			u->bo[kept++] = bo;
			continue;
		}
		bo->unit = NULL;
		m4_user_native_bo_put_locked(bo);
	}
	for (u32 i = kept; i < u->bo_count; i++)
		u->bo[i] = NULL;
	u->bo_count = kept;
	if (released) {
		m4_user_native_unit_stats.cold_released++;
		schedule_work(&p->user_reap); /* upper reap: VA claims, backing refs */
	}
	/* Exactly the owned edges. h->job is released only by the reaper. */
	m4_user_native_history_put_locked(a);
	m4_user_native_history_put_locked(b);
}

static u32 m4_user_native_unit_cells(const struct m4_user_native_unit *u)
{
	u32 cells = 0;

	for (u32 i = 0; i < u->bo_count; i++)
		cells += !!u->bo[i]->cell;
	return cells;
}

/* Runtime-charged bytes a cut would let the normal reapers release: the
 * unaliased GPU-only BOs. Cell wrappers and upper-leased BOs stay with the
 * tombstone. Other stage/version holders may delay, never cancel, release.
 */
static u64 m4_user_native_unit_bytes(const struct m4_user_native_unit *u)
{
	u64 bytes = 0;

	for (u32 i = 0; i < u->bo_count; i++)
		if (!u->bo[i]->cell && !u->bo[i]->upper)
			bytes += u->bo[i]->charge;
	return bytes;
}

/* Smaller headroom of the two native BACKING budgets: the runtime native
 * charge and the frontend's native GEM factory. The factory refuses
 * construction at its own aggregate, which also counts GEMs that outlived
 * their runtime charge: an evicted unit's backing stays until every composed
 * version whose baseline named it released its image (once finished and no
 * longer resident) or is gone, and every stage that selected it (a retained
 * receipt's) is released. Both limits are the host safety bound only (see
 * M4_ORDINARY_NATIVE_MEMORY_LIMIT), never cache policy. Provider metadata is
 * another budget: it never charges backing, so freed pool payload is no provider
 * refund, and its refusals take the provider's own capacity path (graph
 * cache drop, local capacity retry), never backing-sized eviction.
 */
static u64 m4_user_native_bytes_free_locked(struct m4_gpu_provider_runtime *p)
{
	u64 native = p->user_native_bytes, factory = gpu_m4_drm_private_bytes(p->frontend);

	native = native < M4_ORDINARY_NATIVE_MEMORY_LIMIT ?
		M4_ORDINARY_NATIVE_MEMORY_LIMIT - native : 0;
	factory = factory < M4_ORDINARY_NATIVE_MEMORY_LIMIT ?
		M4_ORDINARY_NATIVE_MEMORY_LIMIT - factory : 0;
	return min(native, factory);
}

/* Why a cold/evicted unit's arena cell wrapper cannot return yet. */
enum m4_user_native_cell_state {
	M4_USER_NATIVE_CELL_READY,
	M4_USER_NATIVE_CELL_MEMBER,	/* still a baseline member (removal retried) */
	M4_USER_NATIVE_CELL_UNIT,	/* not cold, accepted, history or credit */
	M4_USER_NATIVE_CELL_HOLDERS,	/* another BO ref: a retired receipt's stage */
	M4_USER_NATIVE_CELL_IDENTITY,	/* wrapper no longer the exact arena cell */
	M4_USER_NATIVE_CELL_CLAIM,	/* claim not this VM's exact registration */
	M4_USER_NATIVE_CELL_STATES,
};

static u32 m4_user_native_cell_pending_locked(struct m4_gpu_provider_runtime *p,
	struct m4_user_native_unit *u, u32 c);

/* Free cells, and cells still owned by a cold/evicted unit that the next
 * cell return can actually give back: drained (ready), held only by retired
 * receipts' stages (credit returned early) or awaiting a retried baseline
 * removal. A cell whose wrapper no longer matches its arena cell or claim
 * never returns by waiting, so it counts as neither.
 */
static u32 m4_user_native_cells_free_locked(struct m4_gpu_provider_runtime *p, u32 *pending)
{
	u32 free = 0, state;

	*pending = 0;
	for (u32 i = 0; i < M4_ARENA_CELLS; i++) {
		struct m4_user_native_unit *u = p->user_cell_owner[i];

		free += !u;
		if (!u || !u->cold)
			continue;
		state = m4_user_native_cell_pending_locked(p, u, i);
		/* Only credits that can actually return. MEMBER waits on a baseline
		 * removal that may be refused indefinitely (e.g. a VM-local error
		 * before closure), so it must not hide a real shortage.
		 */
		*pending += state == M4_USER_NATIVE_CELL_READY ||
			state == M4_USER_NATIVE_CELL_HOLDERS;
	}
	return free;
}

/* Cells of units still in flight or borrowed by an accepted, unpublished
 * stage: their receipt (or rollback) makes them idle and collectible, so a
 * shortage they cause resolves without any new client work.
 */
static u32 m4_user_native_cells_busy_locked(struct m4_gpu_provider_runtime *p)
{
	u32 busy = 0;

	for (u32 i = 0; i < M4_ARENA_CELLS; i++) {
		struct m4_user_native_unit *u = p->user_cell_owner[i];

		busy += u && !u->cold && (u->accepted || !u->history || !u->history->complete);
	}
	return busy;
}

/* The marked set's closure, owner validation and CORE authority, iterated to
 * a fixed point. Marks only ever decrease; no state outside scratch changes.
 */
static void m4_user_native_cold_resolve_locked(struct m4_gpu_provider_runtime *p)
{
	struct m4_user_native_unit *u;
	bool changed;

	do {
		changed = false;
		m4_user_native_cold_settle_locked(p);
		list_for_each_entry(u, &p->user_native_units, retained) {
			struct m4_user_native_history *h[2] = { u->history, u->displacement };

			for (u32 k = 0; u->cold_mark && k < 2; k++) {
				if (!m4_user_native_cold_dying(h[k]) || h[k]->cold_checked)
					continue;
				h[k]->cold_checked = true;
				if (m4_user_native_cold_history_locked(p, h[k])) {
					h[k]->cold_blocked = true;
					changed = true;
				}
			}
		}
		if (changed)
			continue;
		/* Expensive authority only for an otherwise collectible component. */
		list_for_each_entry(u, &p->user_native_units, retained) {
			if (!u->cold_mark || u->cold_proven)
				continue;
			if (m4_user_native_cold_proof_locked(u)) {
				u->cold_mark = false;
				changed = true;
			} else {
				u->cold_proven = true;
			}
		}
	} while (changed);
}

/* Keep this many most recently used idle units per open queue and command
 * type out of low-water trimming (one A/B pair's next reuse plus one), and
 * additionally the newest idle unit of each slot that queue/type uses: a warm
 * unit returns only to its ORIGINAL slot, so every slot of a multi-command
 * window needs its own next reuse. This is cache priority for proactive
 * trimming only, never ownership: firmware last-owner retention is per
 * GLOBAL slot (any queue) and that current owner is never a candidate at
 * all. The idle cache budgets and a real allocation demand evict warm units
 * oldest-first like any other idle unit, so this set cannot grow retention
 * with queues x slots.
 */
#define M4_USER_NATIVE_WARM_KEEP 2U

/* Completed-use slot of a cold-base candidate (which has a history). */
static u32 m4_user_native_unit_slot(const struct m4_user_native_unit *u)
{
	return u->history_index < u->history->count ?
		u->history->command[u->history_index].uma_slot : 0;
}

/* Idle cache budget of each open queue's render and of its compute cache, by
 * the bytes a cut releases; all caches together stay within
 * M4_USER_NATIVE_IDLE_LIMIT. Above either, the oldest idle units go first.
 */
#define M4_USER_NATIVE_CACHE_LIMIT (1280ULL << 20)

/* u (a pressure candidate) is among its queue/type's WARM_KEEP newest idle
 * candidates by completed-use bind sequence, or the newest of its slot.
 */
static bool m4_user_native_warm_locked(struct m4_gpu_provider_runtime *p,
	const struct m4_user_native_unit *u)
{
	const struct m4_user_native_unit *v;
	u32 newer = 0, newer_slot = 0, slot = m4_user_native_unit_slot(u);

	list_for_each_entry(v, &p->user_native_units, retained)
		if (v != u && v->cold_base && v->queue == u->queue &&
		    v->command.type == u->command.type &&
		    (v->history->bind_sequence > u->history->bind_sequence ||
		     (v->history->bind_sequence == u->history->bind_sequence && v > u))) {
			newer++;
			newer_slot += m4_user_native_unit_slot(v) == slot;
		}
	return newer < M4_USER_NATIVE_WARM_KEEP || !newer_slot;
}

/* v and u share one idle cache: the same open queue and command type. */
static bool m4_user_native_same_cache(const struct m4_user_native_unit *v,
	const struct m4_user_native_unit *u)
{
	return v->queue == u->queue && v->command.type == u->command.type;
}

/* LRU-ish: the structurally eligible, unmarked open-queue unit (of cache's
 * idle cache, or of any when NULL) whose latest completed use has the OLDEST
 * bind sequence (monotonic per job bind, across all contexts). O(units) per
 * pick; picks stop at the budget.
 */
static struct m4_user_native_unit *m4_user_native_cold_oldest_locked(
	struct m4_gpu_provider_runtime *p, const struct m4_user_native_unit *cache)
{
	struct m4_user_native_unit *u, *best = NULL;

	list_for_each_entry(u, &p->user_native_units, retained)
		if (u->cold_base && !u->cold_mark &&
		    (!cache || m4_user_native_same_cache(u, cache)) &&
		    (!best || u->history->bind_sequence < best->history->bind_sequence))
			best = u;
	return best;
}

/* Still-unmarked idle bytes of u's cache (all idle candidates, warm ones
 * included: the budget counts what the cache actually retains).
 */
static u64 m4_user_native_cache_bytes_locked(struct m4_gpu_provider_runtime *p,
	const struct m4_user_native_unit *u)
{
	const struct m4_user_native_unit *v;
	u64 bytes = 0;

	list_for_each_entry(v, &p->user_native_units, retained)
		if (v->cold_idle && !v->cold_mark && m4_user_native_same_cache(v, u))
			bytes += m4_user_native_unit_bytes(v);
	return bytes;
}

/* Idle bytes above the aggregate budget (all caches together), and the sum
 * of every cache's own excess (each cache counted once, at its first unit on
 * the list). Deficits even without allocation pressure.
 */
static void m4_user_native_cache_excess_locked(struct m4_gpu_provider_runtime *p,
	u64 *idle, u64 *own)
{
	struct m4_user_native_unit *u, *v;
	u64 total = 0, bytes;

	*own = 0;
	list_for_each_entry(u, &p->user_native_units, retained) {
		bool first = true;

		if (!u->cold_idle)
			continue;
		total += m4_user_native_unit_bytes(u);
		list_for_each_entry(v, &p->user_native_units, retained) {
			if (v == u)
				break;
			first &= !(v->cold_idle && m4_user_native_same_cache(v, u));
		}
		if (!first)
			continue;
		bytes = m4_user_native_cache_bytes_locked(p, u);
		*own += bytes > M4_USER_NATIVE_CACHE_LIMIT ? bytes - M4_USER_NATIVE_CACHE_LIMIT : 0;
	}
	*idle = total > M4_USER_NATIVE_IDLE_LIMIT ? total - M4_USER_NATIVE_IDLE_LIMIT : 0;
}

/* A cache above its own budget gives up its own oldest selectable units
 * first, starting with the history components an earlier round's closure
 * required (cold_need); the aggregate and pressure deficits then take the
 * globally oldest.
 */
static void m4_user_native_cache_trim_locked(struct m4_gpu_provider_runtime *p,
	u32 *cells, u64 *bytes)
{
	struct m4_user_native_unit *u, *oldest;
	bool trimmed;

	list_for_each_entry(u, &p->user_native_units, retained) {
		if (!u->cold_need || !u->cold_base || u->cold_mark)
			continue;
		u->cold_mark = u->cold_trim = true;
		*cells += m4_user_native_unit_cells(u);
		*bytes += m4_user_native_unit_bytes(u);
	}
	do {
		trimmed = false;
		list_for_each_entry(u, &p->user_native_units, retained) {
			if (!u->cold_base || u->cold_mark ||
			    m4_user_native_cache_bytes_locked(p, u) <= M4_USER_NATIVE_CACHE_LIMIT)
				continue;
			oldest = m4_user_native_cold_oldest_locked(p, u);
			oldest->cold_mark = oldest->cold_trim = true;
			*cells += m4_user_native_unit_cells(oldest);
			*bytes += m4_user_native_unit_bytes(oldest);
			trimmed = true;
			break;
		}
	} while (trimmed);
}

/* u holds v's own (A) history, which must die for v's cut: closure keeps v
 * unless u is cut too. v's displacement (B) may survive as a witness.
 */
static bool m4_user_native_holds_history(const struct m4_user_native_unit *u,
	const struct m4_user_native_unit *v)
{
	return v->history && (u->history == v->history || u->displacement == v->history);
}

/* After closure: does some open queue/type idle cache still exceed its own
 * budget by its remaining (unmarked) idle bytes? Bytes cut from another cache
 * never pay for it. A unit its trim selected but closure unmarked needs the
 * rest of its history component: flag every unit holding its own history,
 * transitively, for the next round's trim. Whether such a unit is ever
 * cut stays the closure's, CORE's and baseline removal's decision.
 */
static bool m4_user_native_cache_over_locked(struct m4_gpu_provider_runtime *p)
{
	struct m4_user_native_unit *u, *v;
	bool over = false, changed;

	list_for_each_entry(u, &p->user_native_units, retained) {
		if (!u->cold_idle ||
		    m4_user_native_cache_bytes_locked(p, u) <= M4_USER_NATIVE_CACHE_LIMIT)
			continue;
		over = true;
		u->cold_need |= u->cold_trim && !u->cold_mark;
	}
	do {
		changed = false;
		list_for_each_entry(u, &p->user_native_units, retained) {
			if (u->cold_need || u->cold)
				continue;
			list_for_each_entry(v, &p->user_native_units, retained)
				if (v->cold_need && m4_user_native_holds_history(u, v)) {
					u->cold_need = changed = true;
					break;
				}
		}
	} while (changed);
	return over;
}

/* One bounded scan under the provider mutex: structural marks, component
 * closure, dying-owner validation, then CORE receipt authentication, all
 * BEFORE the first change. Fail-stop sessions retain everything.
 *
 * Closed queues are always candidates. Idle units of OPEN queues are added
 * by an internal selection, oldest use first, when free plus pending-return
 * cells fall below max(cell low water, cell demand), when the native backing
 * headroom (runtime native budget, GEM factory) falls below max(byte low
 * water, byte demand), or when the idle caches exceed their budgets (an
 * over-budget cache first gives up its own oldest units). The selection
 * starts with just enough cells/bytes and widens (doubling, then all) only
 * when the checked closure keeps too few. Selection ends only when the cell
 * and byte deficits are met AND every queue/type cache's remaining idle bytes
 * fit its own budget, or once the bounded search is exhausted: an over-budget
 * cache whose trimmed units closure kept widens to their history component,
 * never accepting another cache's cut as payment. An evicted open-queue unit becomes
 * cold exactly like a closed one; its pools are released by the normal
 * reapers and its live queue simply builds a different unit later. In-flight
 * units, every global slot's current owner and the component of a current
 * stage-mate are never candidates. No caller "safe" flag exists.
 */
/*
 * Witness advance. An idle unit's displacement B only proves that a later,
 * completed use of its slot by a distinct unit exists. When that displacer d
 * was itself displaced by a completed later use B' of the same slot, B' is an
 * equally valid witness (same slot, same parameter ID, later than B). Move
 * the edge forward, toward the stage of the slot's current owner, which is
 * pinned anyway. Otherwise an unevictable unit (say a batch-mate of a closed
 * client's current slot owner) keeps its old B alive, B keeps every unit
 * whose latest use is B (their A edges cannot die), those keep their own B,
 * and so on: one pinned stage cascades forward through all later uses of
 * every slot until the whole arena is held by idle, unevictable units and a
 * new client can never build a unit (-ENOSPC, then DEVICE_LOST).
 *
 * A closed client's units are never reused, so theirs move on every
 * collection: its exit then releases its stages instead of retaining them
 * (and their pages) behind a cascade. Open clients' warm units keep the exact
 * A/B pairs their displacements left (moving those costs every later reuse
 * proof) and move only under a real cell deficit (see cold_collect).
 * Only idle, unaccepted, published units move; a borrowed (accepted) one
 * keeps the exact pair its reuse captured. Never drops a last history
 * reference here: an old B that loses its final edge dies only through the
 * collector's own dying-component checks.
 */
static void m4_user_native_witness_advance_locked(struct m4_gpu_provider_runtime *p,
	bool open)
{
	struct m4_user_native_unit *u, *d;

	lockdep_assert_held(&p->hello->gpu->lock);
	list_for_each_entry(u, &p->user_native_units, retained) {
		if (!open && !u->queue->closed)
			continue;
		for (u32 step = 0; step < M4_ARENA_CELLS; step++) {
			struct m4_user_native_history *a = u->history, *b = u->displacement, *nb;
			const struct m4_user_native_receipt *ra, *rn;

			d = u->displacer;
			if (u->cold || !u->published || u->accepted || !a || !b || !d || d == u ||
			    d->cold || d->accepted || !a->complete || !b->complete)
				break;
			nb = d->displacement;
			if (!nb || nb == b || nb == a || !nb->complete || !d->displacer ||
			    d->displacer == u || d->displacer == d ||
			    refcount_read(&b->refs) <= 1 ||
			    u->history_index >= a->count || d->displacement_index >= nb->count ||
			    !nb->owner_stage || nb->owner_stage->history != nb ||
			    nb->owner_stage->unit[d->displacement_index] != d->displacer)
				break;
			ra = &a->command[u->history_index];
			rn = &nb->command[d->displacement_index];
			if (ra->type != u->command.type || rn->type != ra->type ||
			    rn->uma_slot != ra->uma_slot || rn->parameter_id != ra->parameter_id)
				break;
			refcount_inc(&nb->refs);
			u->displacement = nb;
			u->displacement_index = d->displacement_index;
			u->displacer = d->displacer;
			m4_user_native_history_put_locked(b);
			m4_user_native_unit_stats.witness_advances++;
		}
	}
}

static u32 m4_user_native_cold_collect_locked(struct m4_gpu_provider_runtime *p,
	u32 cell_demand, u64 byte_demand)
{
	struct m4_user_native_unit *u;
	u32 count = 0, cell_deficit = 0, cell_budget, pending, free;
	u64 byte_deficit = 0, idle_deficit, cache_deficit, byte_budget, bytes, target;
	bool warm = false, pressure;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (p->error || !p->ordinary || !p->provider)
		return 0;
	free = m4_user_native_cells_free_locked(p, &pending);
	cell_budget = max(cell_demand, M4_USER_NATIVE_CELL_LOW_WATER);
	if (free + pending < cell_budget)
		cell_deficit = cell_budget - free - pending;
	/* Closed clients' witnesses always move; open ones only under a real
	 * cell shortage, so warm A/B reuse pairs stay as displacement left them.
	 */
	m4_user_native_witness_advance_locked(p, cell_deficit != 0);
	bytes = m4_user_native_bytes_free_locked(p);
	target = max(byte_demand, M4_USER_NATIVE_BYTE_LOW_WATER);
	if (bytes < target)
		byte_deficit = target - bytes;
	pressure = cell_deficit || byte_deficit;
	/* The idle cache: structural candidates of open queues. Its budgets
	 * bound it even without pressure; the cut pays the larger deficit.
	 */
	list_for_each_entry(u, &p->user_native_units, retained) {
		u->cold_idle = m4_user_native_cold_candidate_locked(p, u) && !u->queue->closed;
		u->cold_mark = u->cold_need = false;
	}
	/* The aggregate idle excess is paid by any cache's oldest units; each
	 * cache's own excess only by that cache (and its required components).
	 */
	m4_user_native_cache_excess_locked(p, &idle_deficit, &cache_deficit);
	byte_deficit = max(byte_deficit, max(idle_deficit, cache_deficit));
	cell_budget = cell_deficit;
	byte_budget = byte_deficit;
	for (int round = 0;; round++) {
		u32 selected_cells = 0, offered_cells = 0;
		u64 selected_bytes = 0, offered_bytes = 0;
		bool unselected = false, warm_held = false, over = false;

		list_for_each_entry(u, &p->user_native_units, retained) {
			bool base = m4_user_native_cold_candidate_locked(p, u);

			u->cold_mark = base && u->queue->closed;
			u->cold_base = base && !u->queue->closed && (cell_deficit || byte_deficit);
			u->cold_trim = false;
			u->cold_proven = false;
			if (u->history)
				u->history->cold_checked = u->history->cold_blocked = false;
			if (u->displacement)
				u->displacement->cold_checked = u->displacement->cold_blocked = false;
		}
		/* Warm set: an open queue's most recently used idle units of each
		 * command type are its next A/B reuses. Evicting them only forces
		 * a fresh unit next frame (thrash), so low-water trimming never
		 * takes them; only a real allocation demand's or an over-budget
		 * cache's final round may.
		 */
		if (!warm)
			list_for_each_entry(u, &p->user_native_units, retained)
				if (u->cold_base && m4_user_native_warm_locked(p, u))
					u->cold_warm = true;
		list_for_each_entry(u, &p->user_native_units, retained) {
			warm_held |= u->cold_warm;
			u->cold_base &= !u->cold_warm;
			u->cold_warm = false;
		}
		if (cache_deficit)
			m4_user_native_cache_trim_locked(p, &selected_cells, &selected_bytes);
		while (selected_cells < cell_budget || selected_bytes < byte_budget) {
			u = m4_user_native_cold_oldest_locked(p, NULL);
			if (!u)
				break;
			u->cold_mark = true;
			selected_cells += m4_user_native_unit_cells(u);
			selected_bytes += m4_user_native_unit_bytes(u);
		}
		list_for_each_entry(u, &p->user_native_units, retained)
			unselected |= u->cold_base && !u->cold_mark;
		m4_user_native_cold_resolve_locked(p);
		if (!cell_deficit && !byte_deficit)
			break;
		list_for_each_entry(u, &p->user_native_units, retained)
			if (u->cold_mark && !u->queue->closed) {
				offered_cells += m4_user_native_unit_cells(u);
				offered_bytes += m4_user_native_unit_bytes(u);
			}
		/* Remaining idle bytes after closure, per queue/type cache. */
		if (cache_deficit)
			over = m4_user_native_cache_over_locked(p);
		if (offered_cells >= cell_deficit && offered_bytes >= byte_deficit && !over)
			break;
		if (!unselected || round >= 3) {
			/* Only a real allocation demand or an over-budget idle cache
			 * may then take warm units, again starting from just its
			 * shortfall, oldest first.
			 */
			if (warm || !(cell_demand || byte_demand || idle_deficit || cache_deficit) ||
			    !warm_held)
				break;
			warm = true;
			round = -1;
			cell_budget = cell_deficit;
			byte_budget = byte_deficit;
			continue;
		}
		/* Only a shortfall of the scalar deficits widens the global oldest
		 * selection; an over-budget cache alone widens its own components.
		 */
		if (offered_cells < cell_deficit || offered_bytes < byte_deficit) {
			cell_budget = round == 2 ? U32_MAX : max(cell_budget * 2, 1U);
			byte_budget = round == 2 ? U64_MAX : max(byte_budget * 2, (u64)1);
		}
	}
	/* Step 1 of every eviction: leave FUTURE baseline membership, while old
	 * composed versions keep their mappings/backing until normal reaping.
	 * Fallible, so before the cut: a unit that cannot leave stays uncut (and
	 * a member), and the reduced set is re-closed. A unit that left but is
	 * then unmarked merely uses the per-child append path from now on.
	 */
	for (bool failed = true; failed;) {
		failed = false;
		list_for_each_entry(u, &p->user_native_units, retained)
			if (u->cold_mark && m4_user_native_baseline_remove_locked(u)) {
				u->cold_mark = false;
				failed = true;
			}
		if (failed)
			m4_user_native_cold_resolve_locked(p);
	}
	/* No fallible decision remains. Clear each dying history's BORROWED
	 * owner-stage pointer before any put can queue that history for release.
	 */
	list_for_each_entry(u, &p->user_native_units, retained) {
		struct m4_user_native_history *h[2] = { u->history, u->displacement };

		for (u32 k = 0; u->cold_mark && k < 2; k++)
			if (m4_user_native_cold_dying(h[k]) && h[k]->owner_stage->history == h[k])
				h[k]->owner_stage->history = NULL;
	}
	list_for_each_entry(u, &p->user_native_units, retained) {
		if (!u->cold_mark)
			continue;
		u->cold_mark = false;
		if (u->queue->closed) {
			m4_user_native_unit_stats.evicted_closed++;
		} else {
			if (pressure)
				m4_user_native_unit_stats.evicted_pressure++;
			else
				m4_user_native_unit_stats.evicted_cache++;
			m4_user_native_unit_stats.evicted_bytes += m4_user_native_unit_bytes(u);
		}
		m4_user_native_cold_cut_locked(u);
		count++;
	}
	return count;
}

/* REMOVE-DEBUG: live published units by retention state, for runtime_perf.
 * In flight: borrowed by an accepted stage (a fresh unit joins the list at
 * its exposure). Current: latest completed and undisplaced use, the global
 * slot's firmware-last owner. Idle: displaced cache; "evictable" is its
 * open-queue subset not held by a current stage-mate's component. Cold:
 * evicted tombstones (cells pending return, firmware-visible upper backing).
 * Bytes are the units' own BO charges; runtime and factory are the two
 * native budgets' current use (their gap is backing whose runtime wrapper
 * is gone but which older composed versions still hold).
 */
enum m4_user_native_state {
	M4_USER_NATIVE_INFLIGHT,
	M4_USER_NATIVE_CURRENT,
	M4_USER_NATIVE_IDLE,
	M4_USER_NATIVE_COLD,
	M4_USER_NATIVE_STATES,
};

static const char *const m4_user_native_cell_reason[M4_USER_NATIVE_CELL_STATES] = {
	"ready", "member", "unit", "holders", "identity", "claim",
};

/* REMOVE-DEBUG: native FACTORY backing by holder class. A live user wrapper
 * owned by a non-cold unit (UNIT), by a cold tombstone (COLD: cells pending,
 * refused firmware aliases), or only by stages: completed receipts' stage
 * inventories, retained stages, queue notifiers (STAGE). Then the provider's
 * classes for objects no live wrapper holds (gpu_m4_drm_provider_holder),
 * and wrappers queued for the outside-lock reaper. "pins" is the remainder of
 * the factory's own count: the permanent work-cell arena (3 MiB), and GEMs
 * only snapshot/cache pins (retained jobs) or unreaped provider objects hold.
 */
enum m4_user_native_factory_holder {
	M4_USER_NATIVE_FACTORY_UNIT,
	M4_USER_NATIVE_FACTORY_COLD,
	M4_USER_NATIVE_FACTORY_STAGE,
	M4_USER_NATIVE_FACTORY_REAP,
	M4_USER_NATIVE_FACTORY_HOLDERS,
};

struct m4_user_native_census {
	u64 units[M4_USER_NATIVE_STATES], bytes[M4_USER_NATIVE_STATES];
	u64 evictable, evictable_bytes, runtime_bytes, factory_bytes;
	/* Live (non-cold) units of CLOSED queues, and those still a current
	 * slot owner: what an exited client retains.
	 */
	u64 closed_units, closed_bytes, closed_current;
	/* Arena cells: free, owned by a cold unit (pending return) and why. */
	u32 cells_free, cells_pending, cells_reason[M4_USER_NATIVE_CELL_STATES];
	/* Cold wrappers whose credit returned while stages still hold them. */
	u32 cells_held;
	u64 stamp, factory[M4_USER_NATIVE_FACTORY_HOLDERS];
	struct gpu_m4_drm_provider_holders holders;
};

static void m4_user_native_cells_census_locked(struct m4_gpu_provider_runtime *p,
	struct m4_user_native_census *c);

/* Each live wrapper once: its factory bytes in its first (strongest) class,
 * and its object marked as held by the runtime for the provider's census.
 */
static void m4_user_native_census_bo(struct m4_user_native_census *c,
	struct m4_user_native_bo *b, u32 holder)
{
	if (!b || b->census == c->stamp)
		return;
	b->census = c->stamp;
	if (b->object)
		gpu_m4_drm_provider_object_census_locked(b->object, c->stamp);
	if (!b->cell)
		c->factory[holder] += b->size;
}

static void m4_user_native_census_stage(struct m4_user_native_census *c,
	const struct m4_user_native_stage *s)
{
	if (!s)
		return;
	for (u32 i = 0; i < s->bo_count; i++)
		m4_user_native_census_bo(c, s->bo[i], M4_USER_NATIVE_FACTORY_STAGE);
	if (s->notifier)
		m4_user_native_census_bo(c, s->notifier->bo, M4_USER_NATIVE_FACTORY_STAGE);
}

static void m4_user_native_census_locked(struct m4_gpu_provider_runtime *p,
	struct m4_user_native_census *c)
{
	struct m4_user_native_stage *s;
	struct m4_user_native_unit *u;
	struct m4_user_native_bo *b;

	lockdep_assert_held(&p->hello->gpu->lock);
	*c = (struct m4_user_native_census) {
		.runtime_bytes = p->user_native_bytes,
		.factory_bytes = gpu_m4_drm_private_bytes(p->frontend),
		.stamp = gpu_m4_drm_provider_census_begin_locked(p->provider),
	};
	list_for_each_entry(u, &p->user_native_units, retained) {
		u32 state;
		u64 bytes = 0;

		for (u32 i = 0; i < u->bo_count; i++) {
			bytes += u->bo[i]->charge;
			c->cells_held += u->cold && u->bo[i]->cell_released;
			m4_user_native_census_bo(c, u->bo[i], u->cold ?
				M4_USER_NATIVE_FACTORY_COLD : M4_USER_NATIVE_FACTORY_UNIT);
		}
		if (u->cold)
			state = M4_USER_NATIVE_COLD;
		else if (u->accepted || !u->history || !u->history->complete)
			state = M4_USER_NATIVE_INFLIGHT;
		else if (!u->displacement)
			state = M4_USER_NATIVE_CURRENT;
		else
			state = M4_USER_NATIVE_IDLE;
		c->units[state]++;
		c->bytes[state] += bytes;
		if (state != M4_USER_NATIVE_COLD && u->queue && u->queue->closed) {
			c->closed_units++;
			c->closed_bytes += bytes;
			c->closed_current += state == M4_USER_NATIVE_CURRENT;
		}
		if (state == M4_USER_NATIVE_IDLE && !u->queue->closed &&
		    m4_user_native_cold_candidate_locked(p, u)) {
			c->evictable++;
			c->evictable_bytes += m4_user_native_unit_bytes(u);
		}
	}
	/* Stage-only wrappers: every stage a unit's history, displacement or
	 * acceptance still names (a receipt's job holds its stage), the global
	 * engine-last stages and retained stages.
	 */
	list_for_each_entry(u, &p->user_native_units, retained) {
		m4_user_native_census_stage(c, u->accepted);
		if (u->history)
			m4_user_native_census_stage(c, u->history->owner_stage);
		if (u->displacement)
			m4_user_native_census_stage(c, u->displacement->owner_stage);
	}
	for (u32 e = 0; e < 3; e++)
		m4_user_native_census_stage(c, p->user_last[e]);
	list_for_each_entry(s, &p->user_native_retained, retained)
		m4_user_native_census_stage(c, s);
	list_for_each_entry(b, &p->user_native_garbage, garbage)
		if (!b->cell)
			c->factory[M4_USER_NATIVE_FACTORY_REAP] += b->size;
	gpu_m4_drm_provider_holders_locked(p->provider, c->stamp, &c->holders);
	m4_user_native_cells_census_locked(p, c);
}

static ssize_t m4_user_native_census_show(char *buf, ssize_t size,
	const struct m4_user_native_census *c)
{
	const struct gpu_m4_drm_provider_holders *h = &c->holders;
	ssize_t start = size;
	u64 attributed = 0;

	for (u32 i = 0; i < M4_USER_NATIVE_FACTORY_HOLDERS; i++)
		attributed += c->factory[i];
	for (u32 i = GPU_M4_DRM_PROVIDER_HOLDER_BASELINE; i < GPU_M4_DRM_PROVIDER_HOLDERS; i++)
		attributed += h->bytes[i];
	size += sysfs_emit_at(buf, size,
		" native_units_inflight=%llu native_units_current=%llu native_units_idle=%llu native_units_cold=%llu native_bytes_inflight=%llu native_bytes_current=%llu native_bytes_idle=%llu native_bytes_cold=%llu native_idle_evictable=%llu native_idle_evictable_bytes=%llu native_runtime_bytes=%llu native_factory_bytes=%llu",
		c->units[M4_USER_NATIVE_INFLIGHT], c->units[M4_USER_NATIVE_CURRENT],
		c->units[M4_USER_NATIVE_IDLE], c->units[M4_USER_NATIVE_COLD],
		c->bytes[M4_USER_NATIVE_INFLIGHT], c->bytes[M4_USER_NATIVE_CURRENT],
		c->bytes[M4_USER_NATIVE_IDLE], c->bytes[M4_USER_NATIVE_COLD],
		c->evictable, c->evictable_bytes, c->runtime_bytes, c->factory_bytes);
	size += sysfs_emit_at(buf, size,
		" native_closed_units=%llu native_closed_bytes=%llu native_closed_current=%llu",
		c->closed_units, c->closed_bytes, c->closed_current);
	size += sysfs_emit_at(buf, size,
		" cells_free=%u cells_pending=%u cells_pending_member=%u cells_pending_unit=%u cells_pending_holders=%u cells_pending_identity=%u cells_pending_claim=%u cells_pending_ready=%u cells_held_wrappers=%u",
		c->cells_free, c->cells_pending, c->cells_reason[1], c->cells_reason[2],
		c->cells_reason[3], c->cells_reason[4], c->cells_reason[5], c->cells_reason[0],
		c->cells_held);
	/* REMOVE-DEBUG: the factory count by holder (see the census). */
	size += sysfs_emit_at(buf, size,
		" factory_unit=%llu factory_cold=%llu factory_stage=%llu factory_reap=%llu factory_baseline=%llu factory_composed=%llu factory_finished=%llu factory_version=%llu factory_other=%llu factory_garbage=%llu factory_pins=%llu",
		c->factory[M4_USER_NATIVE_FACTORY_UNIT], c->factory[M4_USER_NATIVE_FACTORY_COLD],
		c->factory[M4_USER_NATIVE_FACTORY_STAGE], c->factory[M4_USER_NATIVE_FACTORY_REAP],
		h->bytes[GPU_M4_DRM_PROVIDER_HOLDER_BASELINE],
		h->bytes[GPU_M4_DRM_PROVIDER_HOLDER_COMPOSED],
		h->bytes[GPU_M4_DRM_PROVIDER_HOLDER_FINISHED],
		h->bytes[GPU_M4_DRM_PROVIDER_HOLDER_VERSION],
		h->bytes[GPU_M4_DRM_PROVIDER_HOLDER_OTHER],
		h->bytes[GPU_M4_DRM_PROVIDER_HOLDER_GARBAGE],
		c->factory_bytes > attributed ? c->factory_bytes - attributed : 0);
	size += sysfs_emit_at(buf, size,
		" versions_composed=%u versions_finished=%u versions_retired=%u images_released=%llu image_objects=%llu image_failures=%llu",
		h->composed, h->finished, h->retired, h->images_released, h->image_objects,
		h->image_failures);
	return size - start;
}

/* Housekeeping cadence. user_reap runs after every job release, i.e. once
 * per frame per client; a full collection walks every retained unit (and,
 * right after a large client exits, its many still-pinned closed units)
 * under the provider mutex the submit/window path needs. Without a real
 * shortage nothing here is urgent, so such passes are batched.
 */
#define M4_USER_NATIVE_COLLECT_MS 50U

/* Deferred user_reap worker only, never the firmware worker or a success
 * callback. Rescheduled after close, displacement and borrower disposal.
 * Runs at once under cell or byte low water; otherwise at most once per
 * M4_USER_NATIVE_COLLECT_MS, a skipped pass being re-kicked by user_collect
 * (closed clients' units are still cut within that interval, and every
 * demand path collects inline regardless).
 */
static void m4_user_native_cold_why_locked(struct m4_gpu_provider_runtime *p);
static bool cold_why_once;
module_param(cold_why_once, bool, 0644);
MODULE_PARM_DESC(cold_why_once, "REMOVE-DEBUG: report once why idle units are not collected");

static void m4_user_native_cold_collect(struct m4_gpu_provider_runtime *p)
{
	struct mutex *lock = &p->hello->gpu->lock;
	unsigned long now, due;
	u32 pending, free;
	u64 start, ns;

	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	now = jiffies;
	due = p->user_collect_last + msecs_to_jiffies(M4_USER_NATIVE_COLLECT_MS);
	if (p->user_collect_last && time_before(now, due)) {
		free = m4_user_native_cells_free_locked(p, &pending);
		if (free + pending >= M4_USER_NATIVE_CELL_LOW_WATER &&
		    m4_user_native_bytes_free_locked(p) >= M4_USER_NATIVE_BYTE_LOW_WATER) {
			if (!p->user_collect_deferred) {
				p->user_collect_deferred = true;
				schedule_delayed_work(&p->user_collect, due - now);
			}
			m4_user_native_unit_stats.collect_deferred++;
			mutex_unlock(lock);
			return;
		}
	}
	p->user_collect_deferred = false;
	p->user_collect_last = now ?: 1;
	/* REMOVE-DEBUG: one-shot report of why idle candidates are kept. */
	if (READ_ONCE(cold_why_once)) {
		WRITE_ONCE(cold_why_once, false);
		m4_user_native_cold_why_locked(p);
	}
	start = ktime_get_ns();
	m4_user_native_cold_collect_locked(p, 0, 0);
	ns = ktime_get_ns() - start;
	m4_user_native_unit_stats.collect_runs++;
	m4_user_native_unit_stats.collect_ns += ns;
	m4_user_native_unit_stats.collect_max = max(m4_user_native_unit_stats.collect_max, ns);
	mutex_unlock(lock);
}

/* Pending-cell return of one cold/evicted unit's arena cell wrapper. It must
 * still match the permanent arena cell exactly. READY: every old holder of the
 * WRAPPER drained, the unit's wrapper ref is the ONLY BO ref (no stage/status/
 * reuse holder, so clearing b->unit cannot make a stage put an extra upper
 * ref). HOLDERS: only retired receipts' stages still hold the wrapper; its
 * CREDIT may return early (cell_return_locked). Its claim must still be this
 * VM's exact registration, but immutable captured versions (e.g. the resident
 * root, released only by a later binding) may keep holding it, and so may the
 * held wrapper itself: they map the identical cell object at the identical
 * VA, so the next same-VM owner adopts that exact claim and other VMs reserve
 * their own. Waiting for them would make the next root, which needs a free
 * cell, a precondition of returning cells. Read-only.
 */
static u32 m4_user_native_cell_state_locked(struct m4_gpu_provider_runtime *p,
	struct m4_user_native_unit *u, struct m4_user_native_bo *b, u32 *index)
{
	struct m4_command_arena *a = p->ready->arena;
	struct m4_command_cell *c = NULL;
	int ret;
	u64 pa;

	for (u32 i = 0; i < M4_ARENA_CELLS && b->cell; i++)
		if (b->cell == &a->cell[i]) {
			c = b->cell;
			*index = i;
			break;
		}
	if (!c)
		return M4_USER_NATIVE_CELL_IDENTITY;
	if (u->baseline || u->tree)
		return M4_USER_NATIVE_CELL_MEMBER;
	if (!u->cold || u->accepted || u->history || u->displacement || b->unit != u ||
	    (!b->cell_released && p->user_cell_owner[*index] != u))
		return M4_USER_NATIVE_CELL_UNIT;
	if (b->runtime != p || b->gem != c->gem || !b->object || !b->claim ||
	    !b->mapped || !b->borrowed_map || b->map.vaddr != c->map.vaddr ||
	    b->size != GPU_M4_DRM_PAGE_SIZE || b->va != c->va || b->fw_va != c->fw_va ||
	    (b->upper ? b->upper != c->upper : u->published) ||
	    gpu_m4_drm_provider_object_page_locked(b->object, 0, &pa) || pa != c->pa)
		return M4_USER_NATIVE_CELL_IDENTITY;
	ret = gpu_m4_drm_provider_private_sole_locked(p->provider, b->vm, b->claim,
		c->va, GPU_M4_DRM_PAGE_SIZE, 0, GPU_M4_DRM_PAGE_SIZE);
	if (ret && ret != -EBUSY)
		return M4_USER_NATIVE_CELL_CLAIM;
	return refcount_read(&b->refs) != 1 ? M4_USER_NATIVE_CELL_HOLDERS :
		M4_USER_NATIVE_CELL_READY;
}

/* Why cell c, owned by cold unit u, has not returned (unbound: UNIT). */
static u32 m4_user_native_cell_pending_locked(struct m4_gpu_provider_runtime *p,
	struct m4_user_native_unit *u, u32 c)
{
	u32 index;

	for (u32 i = 0; i < u->bo_count; i++)
		if (u->bo[i]->cell == &p->ready->arena->cell[c])
			return m4_user_native_cell_state_locked(p, u, u->bo[i], &index);
	return M4_USER_NATIVE_CELL_UNIT;
}

/* REMOVE-DEBUG: free and pending cells, the latter by blocking reason. */
static void m4_user_native_cells_census_locked(struct m4_gpu_provider_runtime *p,
	struct m4_user_native_census *c)
{
	if (!p->ready->arena || !p->ready->arena->imported)
		return;
	for (u32 i = 0; i < M4_ARENA_CELLS; i++) {
		struct m4_user_native_unit *u = p->user_cell_owner[i];

		if (!u) {
			c->cells_free++;
		} else if (u->cold) {
			c->cells_pending++;
			c->cells_reason[m4_user_native_cell_pending_locked(p, u, i)]++;
		}
	}
}

/* REMOVE-DEBUG: which stages still hold a pending cell wrapper b beside its
 * cold unit, and through which unit edge each stays alive: a receipt's job
 * holds its stage, so a completed history kept as some unit's latest use (A)
 * or as its displacement witness (B) keeps every BO that stage selected,
 * including those of units cut since. Also retained and engine-last stages.
 * At most four holders.
 */
static void m4_user_native_holders_report_locked(struct m4_gpu_provider_runtime *p,
	const struct m4_user_native_bo *b)
{
	struct m4_user_native_stage *s;
	struct m4_user_native_unit *v;
	u32 printed = 0, found = 0;

	list_for_each_entry(v, &p->user_native_units, retained) {
		struct m4_user_native_history *h[2] = { v->history, v->displacement };

		for (u32 k = 0; k < 2; k++) {
			bool held = false;

			if (!h[k] || !(s = h[k]->owner_stage))
				continue;
			for (u32 i = 0; i < s->bo_count; i++)
				held |= s->bo[i] == b;
			if (!held)
				continue;
			found++;
			if (printed++ >= 4)
				continue;
			pr_warn("REMOVE-DEBUG GPU native cell holder stage=%p history=%p complete=%u proven=%u history_refs=%u via unit=%p edge=%s unit_cold=%u unit_idle=%u unit_current=%u unit_accepted=%u queue_closed=%u\n",
				s, h[k], h[k]->complete, h[k]->receipts_proven, refcount_read(&h[k]->refs),
				v, k ? "B" : "A", v->cold, v->idle, !v->cold && !v->displacement,
				!!v->accepted, v->queue->closed);
		}
	}
	list_for_each_entry(s, &p->user_native_retained, retained)
		for (u32 i = 0; i < s->bo_count; i++)
			if (s->bo[i] == b && found++ < 8)
				pr_warn("REMOVE-DEBUG GPU native cell holder retained stage=%p exposed=%u pending=%u\n",
					s, s->exposed, !!s->pending);
	for (u32 e = 0; e < 3; e++)
		for (u32 i = 0; p->user_last[e] && i < p->user_last[e]->bo_count; i++)
			if (p->user_last[e]->bo[i] == b && found++ < 8)
				pr_warn("REMOVE-DEBUG GPU native cell holder engine %u last stage=%p\n",
					e, p->user_last[e]);
	/* Unit edges may name one stage twice; refs beyond the unit's own and
	 * these are held by stages no unit edge reaches (jobs awaiting reap).
	 */
	pr_warn("REMOVE-DEBUG GPU native cell wrapper=%p refs=%u reached_holders=%u released=%u\n",
		b, refcount_read(&b->refs), found, b->cell_released);
}

/* REMOVE-DEBUG: why idle candidates (any queue) are not collectible. Marks
 * every candidate, resolves exactly like the collector, classifies each one
 * the closure unmarked, then clears the scratch marks again.
 */
static u32 m4_user_native_why_history_locked(struct m4_gpu_provider_runtime *p,
	struct m4_user_native_history *h)
{
	struct m4_user_native_stage *s = h->owner_stage;
	struct gpu_m4_drm_provider_vm *vm;

	if (!h->complete || (!h->job && !h->settled) || h->runtime != p || !s || s->history != h ||
	    s->pending || s->history_owned || !refcount_read(&s->refs) || !s->queue ||
	    s->queue->runtime != p || !s->queue->vm || h->count != s->count)
		return 0;
	if (m4_user_native_stage_current_locked(s))
		return 1;
	for (u32 e = 0; e < 3; e++)
		if (p->user_last[e] == s)
			return 2;
	for (u32 i = 0; i < s->count; i++) {
		if (!s->unit[i] || s->unit[i]->accepted)
			return 3;
		for (u32 e = 0; e < 3; e++)
			if (p->user_unit_last[e] == s->unit[i])
				return 4;
	}
	vm = s->queue->vm->provider_vm;
	if (!h->settled && gpu_m4_drm_provider_version_quiescent_locked(p->provider, vm, h->version))
		return 5;
	if (s->source != h->version &&
	    gpu_m4_drm_provider_version_quiescent_locked(p->provider, vm, s->source))
		return 6;
	return 7;
}

static void m4_user_native_cold_why_locked(struct m4_gpu_provider_runtime *p)
{
	struct m4_user_native_unit *u, *v;
	u32 cand = 0, kept = 0, a_live = 0, a_blocked = 0, b_blocked = 0, unproven = 0;
	u32 cause[8] = { }, holder[6] = { }, closed = 0;

	list_for_each_entry(u, &p->user_native_units, retained) {
		u->cold_trim = m4_user_native_cold_candidate_locked(p, u);
		u->cold_mark = u->cold_trim;
		u->cold_proven = false;
		cand += u->cold_trim;
		closed += u->cold_trim && u->queue->closed;
		if (u->history)
			u->history->cold_checked = u->history->cold_blocked = false;
		if (u->displacement)
			u->displacement->cold_checked = u->displacement->cold_blocked = false;
	}
	m4_user_native_cold_resolve_locked(p);
	list_for_each_entry(u, &p->user_native_units, retained) {
		struct m4_user_native_history *a = u->history, *b = u->displacement;
		u32 edges = 0;

		if (!u->cold_trim || u->cold_mark)
			continue;
		kept++;
		if (a->cold_blocked) {
			a_blocked++;
			cause[m4_user_native_why_history_locked(p, a)]++;
		} else if (!m4_user_native_cold_dying(a)) {
			a_live++;
			list_for_each_entry(v, &p->user_native_units, retained) {
				u32 n = (v->history == a) + (v->displacement == a);

				if (!n || v->cold_mark)
					continue;
				edges += n;
				if (v->cold)
					holder[0]++;
				else if (v->accepted || !v->history || !v->history->complete)
					holder[1]++;
				else if (!v->displacement)
					holder[2]++;
				else if (!v->cold_trim)
					holder[3]++;
				else
					holder[4]++;
			}
			holder[5] += refcount_read(&a->refs) > a->cold_edges + edges;
		} else if (b && m4_user_native_cold_dying(b) && b->cold_blocked) {
			b_blocked++;
			cause[m4_user_native_why_history_locked(p, b)]++;
		} else {
			unproven++;
		}
	}
	list_for_each_entry(u, &p->user_native_units, retained)
		u->cold_mark = u->cold_trim = u->cold_proven = false;
	pr_warn("REMOVE-DEBUG GPU native cold why: candidates=%u closed=%u kept=%u a_live=%u a_blocked=%u b_blocked=%u unproven=%u cause stale=%u current=%u last=%u accepted=%u unit_last=%u version=%u source=%u none=%u holders cold=%u inflight=%u current=%u noncand=%u cascade=%u extra_refs=%u\n",
		cand, closed, kept, a_live, a_blocked, b_blocked, unproven,
		cause[0], cause[1], cause[2], cause[3], cause[4], cause[5], cause[6], cause[7],
		holder[0], holder[1], holder[2], holder[3], holder[4], holder[5]);
}

/* REMOVE-DEBUG: after repeated cell deferrals, who owns every cell (by unit
 * state), then each cold pending owner (at most eight), why its cells have not
 * returned, with its wrapper's holders, and the wrappers whose credit already
 * returned while retired receipts' stages still hold them.
 */
static void m4_user_native_cells_report_locked(struct m4_gpu_provider_runtime *p, u32 defers)
{
	struct m4_user_native_unit *u;
	u32 owners = 0, state[M4_USER_NATIVE_STATES] = { }, candidates = 0, held = 0;

	for (u32 i = 0; i < M4_ARENA_CELLS; i++) {
		u = p->user_cell_owner[i];
		if (!u)
			continue;
		if (u->cold)
			state[M4_USER_NATIVE_COLD]++;
		else if (u->accepted || !u->history || !u->history->complete)
			state[M4_USER_NATIVE_INFLIGHT]++;
		else if (!u->displacement)
			state[M4_USER_NATIVE_CURRENT]++;
		else
			state[M4_USER_NATIVE_IDLE]++;
		candidates += !u->cold && m4_user_native_cold_candidate_locked(p, u);
	}
	list_for_each_entry(u, &p->user_native_units, retained)
		for (u32 k = 0; u->cold && k < u->bo_count; k++)
			held += u->bo[k]->cell_released;
	pr_warn("REMOVE-DEBUG GPU native cells defers=%u owned inflight=%u current=%u idle=%u idle_candidate=%u cold=%u held_wrappers=%u released_held=%llu\n",
		defers, state[M4_USER_NATIVE_INFLIGHT], state[M4_USER_NATIVE_CURRENT],
		state[M4_USER_NATIVE_IDLE], candidates, state[M4_USER_NATIVE_COLD], held,
		m4_user_native_unit_stats.cells_released_held);
	for (u32 i = 0; i < M4_ARENA_CELLS && owners < 8; i++) {
		struct m4_user_native_bo *b = NULL;
		bool first = true;

		u = p->user_cell_owner[i];
		if (!u || !u->cold)
			continue;
		for (u32 j = 0; j < i; j++)
			first &= p->user_cell_owner[j] != u;
		if (!first)
			continue;
		owners++;
		for (u32 k = 0; k < u->bo_count; k++)
			if (u->bo[k]->cell == &p->ready->arena->cell[i])
				b = u->bo[k];
		pr_warn("REMOVE-DEBUG GPU native pending cell %u owner=%p defers=%u reason=%s published=%u baseline=%u unit_refs=%u bo_count=%u wrapper_refs=%u claim_sole=%d queue_closed=%u\n",
			i, u, defers, m4_user_native_cell_reason[m4_user_native_cell_pending_locked(p, u, i)],
			u->published, u->baseline, refcount_read(&u->refs), u->bo_count,
			b ? refcount_read(&b->refs) : 0,
			b && b->claim ? gpu_m4_drm_provider_private_sole_locked(p->provider, b->vm,
				b->claim, b->va, b->size, 0, GPU_M4_DRM_PAGE_SIZE) : -ENOENT,
			u->queue->closed);
		if (b && refcount_read(&b->refs) != 1)
			m4_user_native_holders_report_locked(p, b);
	}
	owners = 0;
	list_for_each_entry(u, &p->user_native_units, retained)
		for (u32 k = 0; u->cold && k < u->bo_count && owners < 4; k++)
			if (u->bo[k]->cell_released) {
				owners++;
				m4_user_native_holders_report_locked(p, u->bo[k]);
			}
}

/* REMOVE-DEBUG: after repeated BYTE deferrals or refusals, the native factory
 * count by holder class (see the census).
 */
static void m4_user_native_factory_report_locked(struct m4_gpu_provider_runtime *p, u32 defers)
{
	struct m4_user_native_census c;
	const struct gpu_m4_drm_provider_holders *h = &c.holders;

	m4_user_native_census_locked(p, &c);
	pr_warn("REMOVE-DEBUG GPU native factory defers=%u factory=%llu runtime=%llu unit=%llu cold=%llu stage=%llu reap=%llu baseline=%llu composed=%llu finished=%llu version=%llu other=%llu garbage=%llu versions_composed=%u finished=%u retired=%u images_released=%llu image_failures=%llu cells_pending_holders=%u\n",
		defers, c.factory_bytes, c.runtime_bytes, c.factory[M4_USER_NATIVE_FACTORY_UNIT],
		c.factory[M4_USER_NATIVE_FACTORY_COLD], c.factory[M4_USER_NATIVE_FACTORY_STAGE],
		c.factory[M4_USER_NATIVE_FACTORY_REAP], h->bytes[GPU_M4_DRM_PROVIDER_HOLDER_BASELINE],
		h->bytes[GPU_M4_DRM_PROVIDER_HOLDER_COMPOSED],
		h->bytes[GPU_M4_DRM_PROVIDER_HOLDER_FINISHED],
		h->bytes[GPU_M4_DRM_PROVIDER_HOLDER_VERSION], h->bytes[GPU_M4_DRM_PROVIDER_HOLDER_OTHER],
		h->bytes[GPU_M4_DRM_PROVIDER_HOLDER_GARBAGE], h->composed, h->finished, h->retired,
		h->images_released, h->image_failures, c.cells_reason[M4_USER_NATIVE_CELL_HOLDERS]);
}

/* Does anything besides the unit list still name this cold unit: a stage or
 * history membership ref, a cell credit, a global slot's last owner, or a
 * not-yet-cut unit it displaced?
 */
static bool m4_user_native_unit_named_locked(struct m4_gpu_provider_runtime *p,
	const struct m4_user_native_unit *u)
{
	const struct m4_user_native_unit *v;

	if (refcount_read(&u->refs) != 1)
		return true;
	for (u32 c = 0; c < M4_ARENA_CELLS; c++)
		if (p->user_cell_owner[c] == u)
			return true;
	for (u32 e = 0; e < 3; e++)
		if (p->user_unit_last[e] == u)
			return true;
	list_for_each_entry(v, &p->user_native_units, retained)
		if (v->displacer == u)
			return true;
	return false;
}

/* Locked detach scan, run by user_reap AFTER the provider reaper. For each
 * drained cell wrapper, in one transaction: compact u->bo[], clear b->unit
 * (no stage holds b), drop exactly the unit-acquired seed upper ref, then the
 * unit's wrapper ref (its claim/object follow their normal disposal), then
 * free user_cell_owner[] (unless already returned). The arena GEM, pin, NC
 * view, PA, leaves, seed and payload are untouched; a new unit rebuilds
 * contents under its own protected publication. The tombstone itself stays
 * (other raw b->unit). Each cell of a render pair returns independently.
 *
 * A wrapper still held only by older stages returns its CREDIT now and stays
 * held (cell_released). Those stages belong to retired receipts: the cut
 * proved this unit's own and its distinct same-slot displacer's actual CORE
 * retirement, it is neither accepted nor any slot's current owner, and no
 * stage that selected it can still be in flight (selection marks it accepted
 * until its own receipt or rollback). A receipt only reads such a stage's
 * immutable records (BO identities and firmware VAs), never the cell's
 * payload, so they need the wrapper record, not the cell. Kept alive only by
 * another idle unit's (or a current stage-mate's) receipt, such a stage
 * would otherwise park the cell indefinitely. The next same-VM owner adopts
 * the identical claim, as for a captured version; the wrapper's own claim,
 * object and seed upper refs drop when its holders do.
 */
static u32 m4_user_native_cell_return_locked(struct m4_gpu_provider_runtime *p)
{
	struct m4_user_native_unit *u, *next;
	u32 count = 0;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (p->error || !p->ordinary || !p->provider || !p->ready->arena ||
	    !p->ready->arena->imported)
		return 0;
	list_for_each_entry_safe(u, next, &p->user_native_units, retained) {
		u32 kept = 0, index;

		if (!u->cold)
			continue;
		/* Retried deferred removal of a rollback owner; still a member: no
		 * cell of it may return (the current baseline holds its claims).
		 */
		if (u->baseline || u->tree) {
			if (m4_user_native_baseline_remove_locked(u))
				continue;
			if (!u->published)
				m4_user_native_rollback_trim_locked(u);
		}
		for (u32 i = 0; i < u->bo_count; i++) {
			struct m4_user_native_bo *b = u->bo[i];
			u32 state = b->cell ? m4_user_native_cell_state_locked(p, u, b, &index) :
				M4_USER_NATIVE_CELL_UNIT;

			if (state == M4_USER_NATIVE_CELL_HOLDERS && !b->cell_released) {
				p->user_cell_owner[index] = NULL;
				b->cell_released = true;
				m4_user_native_unit_stats.cells_released_held++;
			}
			if (state != M4_USER_NATIVE_CELL_READY) {
				u->bo[kept++] = b;
				continue;
			}
			if (!b->cell_released)
				p->user_cell_owner[index] = NULL;
			b->unit = NULL;
			if (b->upper)
				gpu_m4_drm_upper_put_locked(b->upper);
			m4_user_native_bo_put_locked(b);
			count++;
		}
		for (u32 i = kept; i < u->bo_count; i++)
			u->bo[i] = NULL;
		u->bo_count = kept;
		/* A drained rollback owner (never published) is finally freed. */
		if (!u->published && !u->bo_count && !u->baseline && !u->tree) {
			list_del_init(&u->retained);
			list_del_init(&u->queue_link);
			m4_user_native_unit_free_locked(p, u);
			continue;
		}
		/* So is a drained published tombstone (every firmware alias
		 * released, every cell returned) that nothing names any more.
		 */
		if (u->published && !u->bo_count && !u->baseline && !u->tree &&
		    !m4_user_native_unit_named_locked(p, u)) {
			list_del_init(&u->retained);
			list_del_init(&u->queue_link);
			m4_user_native_unit_stats.cold_freed++;
			m4_user_native_unit_free_locked(p, u);
		}
	}
	return count;
}

/* user_reap only, after gpu_m4_drm_provider_reap(): returns the number of
 * detached wrappers whose normal outside-lock disposal is now queued.
 */
static u32 m4_user_native_cell_return(struct m4_gpu_provider_runtime *p)
{
	struct mutex *lock = &p->hello->gpu->lock;
	u32 count;

	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	count = m4_user_native_cell_return_locked(p);
	mutex_unlock(lock);
	return count;
}

static u32 m4_user_native_command_cells(u16 type)
{
	return type == DRM_ASAHI_CMD_RENDER ? 2 : type == DRM_ASAHI_CMD_COMPUTE ? 1 : 0;
}

/* Conservative backing of one NEW unit for this command: the allocator pool
 * and small owners, plus a render's geometry-sized tilemap/TPC and its whole
 * PB heap (4 MiB minimum, larger targets more). Admission and eviction
 * sizing only; the real charges at construction remain the authority.
 */
static u64 m4_user_native_command_bytes(const struct gpu_m4_drm_job_command *command,
	const struct m4_user_queue *q)
{
	struct gpu_m4_exec_geometry geometry;
	struct gpu_m4_render_pb_layout pb;

	if (!m4_user_native_render_need(command, &geometry, &pb, q))
		return M4_ORDINARY_NATIVE_UNIT_BYTES;
	return M4_ORDINARY_NATIVE_UNIT_BYTES + pb.heap_size + geometry.tilemap_size +
		geometry.tpc_size;
}

/* Bytes of every NEW (not warm-reused) unit of commands [first, count). */
static u64 m4_user_native_new_bytes(const struct m4_user_native_stage *s,
	const struct gpu_m4_drm_job_command *commands, u32 first)
{
	u64 bytes = 0;

	for (u32 i = first; i < s->count; i++)
		if (!m4_user_native_reused(s, i))
			bytes += m4_user_native_command_bytes(&commands[i], s->queue);
	return bytes;
}

/* Whole-batch cell credits, atomically BEFORE constructing any new unit: each
 * NEW unit (not a warm whole-unit reuse) is assigned all of its cells (one
 * compute work, distinct TA+FS render work) in this one locked transaction,
 * so concurrent materializers never hold half a render pair. The unpublished
 * unit destructor is the rollback. A shortage first runs the checked eviction
 * and an immediate drained-cell return; anything still pending returns via
 * the reaper. That is backpressure: -EAGAIN for a deferred window child (its
 * materializer retries), -ENOSPC only for an eager synchronous batch. The
 * byte check uses the native backing headroom, the GEM factory included, so
 * idle units are evicted BEFORE any construction the factory would refuse.
 * Bytes are an estimate against the host safety bound, never cache policy: a
 * deferred child waits for them only when this very pass evicted units, whose
 * pools the reapers now return. Otherwise no wait can make room (backing that
 * finished images or retained receipts still hold returns at their own
 * release, not by this child's retry), so it builds now and the real charges
 * decide; a refusal then fails this child alone (capacity_backpressure) and
 * never parks its queue behind a deferral that cannot end.
 */
static int m4_user_native_cells_reserve(struct m4_user_native_stage *s,
	const struct gpu_m4_drm_job_command *commands)
{
	struct m4_gpu_provider_runtime *p = s->queue->runtime;
	struct mutex *lock = &p->hello->gpu->lock;
	u32 need = 0, free, pending, next = 0, evicted = 0, defers;
	u64 need_bytes, bytes;
	bool byte_wait, wait;
	int ret = 0;

	for (u32 i = 0; i < s->count; i++)
		if (s->unit[i] && !m4_user_native_reused(s, i))
			need += m4_user_native_command_cells(s->command[i].type);
	if (!need)
		return 0;
	need_bytes = m4_user_native_new_bytes(s, commands, 0);
	mutex_lock(lock);
	if (!p->ready->arena || !p->ready->arena->imported) {
		mutex_unlock(lock);
		return -EPERM;
	}
	free = m4_user_native_cells_free_locked(p, &pending);
	bytes = m4_user_native_bytes_free_locked(p);
	if (free < need || bytes < need_bytes) {
		/* Checked eviction now; its pools/cells return after the reaper. */
		evicted = m4_user_native_cold_collect_locked(p, need, need_bytes);
		m4_user_native_cell_return_locked(p);
		free = m4_user_native_cells_free_locked(p, &pending);
	}
	/* A deferred child waits for byte headroom the reapers are returning
	 * instead of building a partial window doomed to fail. With nothing
	 * evicted it, like an eager batch, tries the real charges, which remain
	 * the authority (the estimate is conservative).
	 */
	byte_wait = s->deferred_materialization && bytes < need_bytes && evicted;
	if (bytes < need_bytes && !byte_wait && free >= need)
		m4_user_native_unit_stats.byte_proceeds++;
	/* Cells, like bytes, are waited for only while a return is under way:
	 * cells this pass evicted or still pending a return the reaper can make.
	 * Otherwise no wait can make room, so this child fails alone (-ENOSPC:
	 * its materializer's bounded reclaim-then-rebuild passes, then only its
	 * public batch's fence) instead of parking its queue behind it.
	 */
	wait = free < need ? s->deferred_materialization && (evicted || pending) : byte_wait;
	/* Cells held by in-flight or borrowed units come back with their
	 * receipt or rollback: capacity backpressure, never a refusal.
	 */
	if (!wait && free < need && s->deferred_materialization &&
	    m4_user_native_cells_busy_locked(p)) {
		wait = true;
		m4_user_native_unit_stats.busy_waits++;
	}
	if (free < need || byte_wait) {
		schedule_work(&p->user_reap);
		ret = wait ? -EAGAIN : -ENOSPC;
		if (ret == -EAGAIN)
			m4_user_native_unit_stats.construct_waits++;
		else
			m4_user_native_unit_stats.construct_refusals++;
		defers = free < need ? ++m4_user_native_unit_stats.cell_defers :
			++m4_user_native_unit_stats.byte_defers;
		/* REMOVE-DEBUG: a repeating deferral reports its first, eighth and
		 * every 256th occurrence, naming its blockers then (cell owners
		 * and their holders, or the factory holders), so the reports are
		 * never drowned by the deferrals themselves.
		 */
		if (defers == 1 || defers == 8 || !(defers % 256)) {
			pr_warn("GPU native capacity: arena cells=%u free=%u pending=%u need=%u bytes_free=%llu need_bytes=%llu commands=%u deferred=%u evicted=%u defers=%u\n",
				M4_ARENA_CELLS, free, pending, need, bytes, need_bytes, s->count,
				s->deferred_materialization, evicted, defers);
			if (free < need) {
				m4_user_native_cells_report_locked(p, defers);
				m4_user_native_cold_why_locked(p);
			}
			else
				m4_user_native_factory_report_locked(p, defers);
		}
		mutex_unlock(lock);
		return ret;
	}
	m4_user_native_unit_stats.cell_defers = 0;
	if (bytes >= need_bytes)
		m4_user_native_unit_stats.byte_defers = 0;
	for (u32 i = 0; i < s->count; i++) {
		u32 cells;

		if (!s->unit[i] || m4_user_native_reused(s, i))
			continue;
		cells = m4_user_native_command_cells(s->command[i].type);
		for (; cells && next < M4_ARENA_CELLS; next++)
			if (!p->user_cell_owner[next]) {
				p->user_cell_owner[next] = s->unit[i];
				cells--;
			}
	}
	mutex_unlock(lock);
	return 0;
}

/* A real charge refused construction of command index's unit. Only a refusal
 * by a native BACKING quota (the runtime native charge or the GEM factory's
 * native quota), as recorded at that very admission, is backing pressure:
 * run the checked backing eviction for every remaining new unit now (its
 * pools return through the reapers, which this schedules). A deferred window
 * child then waits (-EAGAIN, bounded by its materializer, which rebuilds
 * after the reapers ran), but only when that eviction cut a unit: with
 * nothing left to evict, waiting cannot make room, so the original refusal
 * fails this child alone (the materializer's bounded reclaim-then-rebuild
 * passes, then only its public batch's fence). An eager synchronous batch,
 * whose caller holds the frontend mutex and so cannot run the reapers here,
 * also reports the original error. Any other refusal (provider pin metadata,
 * private claim, firmware upper capacity, host memory) keeps its original
 * errno for the provider's local capacity retry, which reaps and drops
 * optional graph caches first; evicting backing would refund none of it.
 * The headroom left after this stage's own partial charges never decides
 * which budget refused: the geometry estimate only sizes the eviction (and
 * the pre-construction reserve decision in cells_reserve).
 */
static int m4_user_native_capacity_backpressure(struct m4_user_native_stage *s, u32 index,
	const struct gpu_m4_drm_job_command *commands, int error)
{
	struct m4_gpu_provider_runtime *p = s->queue->runtime;
	struct mutex *lock = &p->hello->gpu->lock;
	u64 need = m4_user_native_new_bytes(s, commands, index);
	int ret = error;

	mutex_lock(lock);
	if (s->backing_refused &&
	    m4_user_native_cold_collect_locked(p, 0, need) && s->deferred_materialization)
		ret = -EAGAIN;
	/* REMOVE-DEBUG: repeated backing refusals name the factory holders. */
	if (s->backing_refused) {
		u32 defers = ++m4_user_native_unit_stats.byte_defers;

		if (defers == 1 || defers == 8 || !(defers % 256))
			m4_user_native_factory_report_locked(p, defers);
	}
	schedule_work(&p->user_reap);
	if (ret == -EAGAIN)
		m4_user_native_unit_stats.construct_waits++;
	else
		m4_user_native_unit_stats.construct_refusals++;
	mutex_unlock(lock);
	return ret;
}

/* REMOVE-DEBUG: one new unit's backing construction attempt. */
static void m4_user_native_construct_stats(struct m4_gpu_provider_runtime *p, u64 started,
	int ret)
{
	struct mutex *lock = &p->hello->gpu->lock;
	u64 ns = ktime_get_ns() - started;

	mutex_lock(lock);
	m4_user_native_unit_stats.construct_attempts++;
	if (ret) {
		m4_user_native_unit_stats.construct_rollbacks++;
	} else {
		m4_user_native_unit_stats.construct_done++;
		/* Units actually built, never reservation-time records. */
		m4_user_native_unit_stats.units_new++;
	}
	m4_user_native_unit_stats.construct_ns += ns;
	m4_user_native_unit_stats.construct_max = max(m4_user_native_unit_stats.construct_max, ns);
	mutex_unlock(lock);
}

static int m4_user_native_stage(void *owner, void *queue,
	const struct gpu_m4_drm_provider_stage_view *view,
	struct gpu_m4_drm_provider_stage_result *result)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_user_queue *q = queue;
	struct mutex *lock = &p->hello->gpu->lock;
	struct m4_user_native_stage *s;
	int ret;

	lockdep_assert_not_held(lock);
#ifdef CONFIG_DEBUG_FS
	gpu_m4_trace(GPU_M4_TRACE_PHASE, 10, (u64)(uintptr_t)q, 0, 0, 0);
#endif
	if (!q || q->runtime != p || !view || !result || view->runtime_queue != q ||
	    view->vm != q->vm->provider_vm || !view->batch || !view->batch->commands ||
	    !view->batch->count || view->batch->count > GPU_M4_DRM_JOB_MAX_COMMANDS)
		return -EINVAL;
	mutex_lock(lock);
	ret = p->error ?: (!p->ordinary || q->closed || q->vm->closed || !q->notifier ? -ESHUTDOWN : 0);
	if (!ret)
		ret = gpu_m4_drm_provider_version_check_origin_locked(p->provider, view->vm, view->source);
	mutex_unlock(lock);
	if (ret)
		return ret;
	mutex_lock(lock);
	ret = m4_user_native_charge_locked(p, sizeof(*s));
	mutex_unlock(lock);
	if (ret)
		return ret;
	s = kvzalloc(sizeof(*s), GFP_KERNEL);
	if (!s) {
		mutex_lock(lock);p->user_native_bytes -= sizeof(*s);mutex_unlock(lock);
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&s->retained);
	INIT_LIST_HEAD(&s->idle);
	s->queue = q;
	s->source = view->source;
	s->notifier = q->notifier;
	s->charge = sizeof(*s);
	s->count = view->batch->count;
	s->deferred_materialization = view->deferred_materialization;
	s->next_va = GPU_M4_BOOTSTRAP_LOWER + GPU_M4_BOOTSTRAP_SPAN;
	refcount_set(&s->refs, 1);
	mutex_lock(lock);
	s->bank = m4_user_bank_choose_locked(p, q);
	if (p->user_inflight2) {
		s->bank_staging = true;
		p->user_bank_staging[s->bank]++;
	}
	/* Check every bounded ref BEFORE incrementing; then no rollback can lose
	 * a partially retained source/queue/notifier on a later overflow.
	 */
	ret = (!refcount_read(&q->refs) || refcount_read(&q->refs) == U32_MAX ||
		!refcount_read(&s->notifier->refs) || refcount_read(&s->notifier->refs) == U32_MAX) ?
		-EOVERFLOW : gpu_m4_drm_provider_version_get_locked(s->source);
	if (!ret) {
		refcount_inc(&q->refs);
		refcount_inc(&s->notifier->refs);
	}
	mutex_unlock(lock);
	if (ret) {
		/* No native owner transferred yet. Source/queue refs taken above are
		 * handled explicitly; result never claims an unheld source baseline.
		 */
		mutex_lock(lock);p->user_native_bytes -= s->charge;mutex_unlock(lock);
		kvfree(s);
		return ret;
	}
#ifdef CONFIG_DEBUG_FS
	gpu_m4_trace(GPU_M4_TRACE_PHASE, 11, (u64)(uintptr_t)q, 0, 0, 0);
#endif
	result->owner = s;
	s->pending = m4_user_native_history_create(p, s->count);
	if (IS_ERR(s->pending)) {
		ret = PTR_ERR(s->pending);
		s->pending = NULL;
		return ret;
	}
	s->extra[s->extra_count++] = (struct gpu_m4_drm_provider_stage_extra) {
		.object = s->notifier->bo->object, .size = s->notifier->bo->size,
		.access = M4_USER_NATIVE_RW, .purpose = GPU_M4_DRM_PROVIDER_STAGE_FIRMWARE_METADATA,
	};
	/* Warm whole-unit reuse first, then every NEW unit and its exact cell
	 * credits are reserved together before any partial construction.
	 */
	for (u32 i = 0; i < s->count; i++) {
		s->command[i].type = view->batch->commands[i].type;
		ret = m4_user_native_unit_take(s, i, &view->batch->commands[i]);
		if (ret)
			return ret;
	}
#ifdef CONFIG_DEBUG_FS
	gpu_m4_trace(GPU_M4_TRACE_PHASE, 12, (u64)(uintptr_t)q, 0, 0, 0);
#endif
	for (u32 i = 0; i < s->count; i++) {
		if (s->unit[i])
			continue;
		ret = m4_user_native_unit_create(s, i);
		if (ret)
			return ret;
	}
	s->building = NULL;
#ifdef CONFIG_DEBUG_FS
	gpu_m4_trace(GPU_M4_TRACE_PHASE, 13, (u64)(uintptr_t)q, 0, 0, 0);
#endif
	ret = m4_user_native_cells_reserve(s, view->batch->commands);
	if (ret)
		return ret; /* owner transfers; unpublished units drop their credits. */
#ifdef CONFIG_DEBUG_FS
	gpu_m4_trace(GPU_M4_TRACE_PHASE, 15, (u64)(uintptr_t)q, 0, 0, 0);
#endif
	for (u32 i = 0; i < s->count; i++) {
		const struct gpu_m4_drm_job_command *c = &view->batch->commands[i];
		u64 started;

		if (m4_user_native_reused(s, i))
			continue;
		started = ktime_get_ns();
		s->building = s->unit[i];
		if (c->type == DRM_ASAHI_CMD_COMPUTE)
			ret = m4_user_native_compute_allocate(s, i);
		else if (c->type == DRM_ASAHI_CMD_RENDER)
			ret = m4_user_native_render_allocate(s, i, &c->render);
		else
			ret = -EOPNOTSUPP;
		m4_user_native_construct_stats(p, started, ret);
		if (!ret) {
			mutex_lock(lock);
			s->queue->constructed++;
			s->queue->constructed_render += c->type == DRM_ASAHI_CMD_RENDER;
			mutex_unlock(lock);
		}
		if (ret == -ENOSPC || ret == -ENOMEM)
			ret = m4_user_native_capacity_backpressure(s, i,
				view->batch->commands, ret);
		if (ret)
			return ret; /* owner transfers on failure; existing core rolls back. */
		s->unit[i]->command = s->command[i];
		/* Persistent backing records have no old admission self-cookie.
		 * Every assembly supplies its own current stage membership above.
		 */
		if (c->type == DRM_ASAHI_CMD_RENDER)
			s->unit[i]->command.render.owner.stage_owner = NULL;
		s->building = NULL;
		/* Complete inventory: join the VM's persistent native baseline (and,
		 * S3c, its authority tree, which must succeed).
		 */
		ret = m4_user_native_baseline_add(s->unit[i]);
		if (ret)
			return ret; /* owner transfers; the unit's rollback unmaps. */
	}
#ifdef CONFIG_DEBUG_FS
	gpu_m4_trace(GPU_M4_TRACE_PHASE, 14, (u64)(uintptr_t)q, 0, 0, 0);
#endif
	result->native = s->native;
	result->count = s->native_count;
	result->extra = s->extra;
	result->extra_count = s->extra_count;
	return 0;
}

#endif
