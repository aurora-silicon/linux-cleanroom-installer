/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_PROVIDER_RUNTIME_H
#define APPLE_GPU_M4_PROVIDER_RUNTIME_H

#include <linux/refcount.h>
#include <linux/iosys-map.h>
#include <linux/firmware.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/sizes.h>
#include <drm/drm_gem.h>
#include "gpu-m4-drm-validation.h"
#include "gpu-m4-drm-provider-vm.h"
#include "gpu-m4-drm-memory.h"
#include "gpu-m4-wb-fixture.h"
#include "gpu-m4-compute-packet.h"
#include "gpu-m4-compute-timestamps.h"
#include "gpu-m4-drm-provider-timestamp-alias.h"
#include "gpu-m4-drm-provider-lane.h"
#include "gpu-m4-user-budget.h"

/* REMOVE-DEBUG: private, idle ASID-1 root qualification. A separate gate stages
 * a complete NC manifest using three NEW native code PAs, not the r6 backing
 * manifest. No DRM node, command producer, reuse, teardown or shader
 * witness is provided. Every returned owner/session baseline stays until reboot.
 * Included after store-runtime, with the real READY/power owner definitions.
 */
static bool provider_root_trial;
module_param(provider_root_trial, bool, 0400);
MODULE_PARM_DESC(provider_root_trial, "Permit private idle captured ASID-1 root trial only");
static bool provider_compute_trial;
module_param(provider_compute_trial, bool, 0400);
MODULE_PARM_DESC(provider_compute_trial, "Stage retained NC V0/V1 and allow a separately guarded first-work trial");
static bool provider_writeback_trial;
module_param(provider_writeback_trial, bool, 0400);
MODULE_PARM_DESC(provider_writeback_trial, "Stage private W0 WB output with exact future-V1 cache protection before idle roots");
static bool provider_writeback_input_trial;
module_param(provider_writeback_input_trial, bool, 0400);
MODULE_PARM_DESC(provider_writeback_input_trial, "Use the pinned W1 load/XOR/store profile with native WB input and output");
static unsigned int provider_writeback_challenge;
module_param(provider_writeback_challenge, uint, 0400);
MODULE_PARM_DESC(provider_writeback_challenge, "Immutable W1 CPU challenge 0 or 1, each on a fresh one-shot session");
static bool provider_timestamp_trial;
module_param(provider_timestamp_trial, bool, 0400);
MODULE_PARM_DESC(provider_timestamp_trial, "Use one sealed private NC compute job and canonical firmware timestamp pair");
static bool provider_timestamp_writeback_trial;
module_param(provider_timestamp_writeback_trial, bool, 0400);
MODULE_PARM_DESC(provider_timestamp_writeback_trial, "Qualify sole-CPU-WB firmware timestamp storage through the real private job/cache path");

#define M4_PROVIDER_KERNEL_START 0x8000000000ULL
#define M4_PROVIDER_KERNEL_END 0x8020000000ULL

/* The ordinary provider's finite budget bounds only what the provider itself
 * allocates: per-VM pin bookkeeping (record and PA inventory), retained
 * versions, snapshots, manifests, construction caches and tables. Pinned GEM
 * backing is the factory's (shmem for public BOs, the native budget for
 * private command backing) and is never charged here, so a buffer shared by
 * the compositor and its clients is one buffer, not one per importing VM.
 * A refusal of this budget is local to its request, job or VM, never a
 * provider fault.
 * The per-VM leaf ceiling is an independent VA/table-size bound (4 GiB of
 * mapped pages per VM ledger), not derived from the byte budget.
 * Private qualification profiles keep their original smaller bounds.
 */
#define M4_ORDINARY_PROVIDER_MEMORY_LIMIT SZ_4G
#define M4_ORDINARY_PROVIDER_MAX_LEAVES ((u32)(SZ_4G / GPU_M4_DRM_PAGE_SIZE) - 4U)
#define M4_PROVIDER_MEMORY_LIMIT 0x1000000ULL
#define M4_PROVIDER_EXTERNAL_COUNT 10U
#define M4_PROVIDER_CODE_COUNT 3U
#define M4_PROVIDER_EXTERNAL_METADATA_LIMIT 0x20000ULL

struct m4_gpu_provider_runtime;
struct m4_gpu_provider_work;
struct m4_provider_external {
	struct m4_gpu_provider_runtime *runtime;
	struct gpu_m4_drm_provider_private_lease *claim;
	struct m4_ready_buffer *source, allocation, *pool;
	u64 *pa, va, size;
	u32 pages, access;
	refcount_t refs;
};

struct m4_provider_code {
	struct drm_gem_object *gem;
	struct gpu_m4_drm_provider_object *object;
	struct iosys_map map;
	struct m4_ready_buffer source;
	u8 *selected_preimage;
	u64 va, pa;
	bool prepared, mapped, uploaded;
};

struct m4_provider_writeback_output {
	struct drm_gem_object *gem;
	struct gpu_m4_drm_provider_object *object;
	struct iosys_map map;
	u8 *preimage;
	u64 va, size, pa;
	bool mapped, initialized;
};

struct m4_provider_ts_storage {
	struct drm_gem_object *gem;
	struct gpu_m4_drm_provider_object *object;
	struct iosys_map map;
	struct gpu_m4_drm_provider_timestamp_cookie *cookie;
	enum gpu_m4_drm_cpu_policy policy;
	u8 *preimage;
	u64 size, pa, sentinel[2];
	bool prepared, mapped, initialized;
};

/* One retained first-work payload, not another provider job state machine.
 * The actual job/lane owns all acceptance, protection and completion authority.
 */
struct m4_provider_ts_job {
	struct m4_gpu_provider_runtime *runtime;
	struct gpu_m4_drm_provider_job *job;
	struct m4_gpu_provider_work *work;
	struct gpu_m4_drm_provider_lane_item item;
	struct gpu_m4_drm_timestamp_alias_token *token;
	struct gpu_m4_compute_ts_plan ts_plan;
	u8 ts_scratch[GPU_M4_EXEC_COMPUTE_SIZE];
	int error;
	bool completed, released, job_held;
};

static const u32 m4_provider_code_ids[M4_PROVIDER_CODE_COUNT] = {
	M4_EXEC_USC_WINDOW, M4_EXEC_CONTAINER, M4_EXEC_IOTO,
};
static const u64 m4_provider_code_va[M4_PROVIDER_CODE_COUNT] = {
	M4_EXEC_USC_BASE, M4_EXEC_CONTAINER_GPU, M4_EXEC_IOTO_GPU,
};
static const struct {
	u64 va, size;
	u32 id, access;
} m4_provider_external_role[M4_PROVIDER_EXTERNAL_COUNT] = {
	{ GPU_M4_BOOTSTRAP_LOWER, 0x4000, GPU_M4_UMA,
	  DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE },
	{ M4_EXEC_WORK_GPU, 0x4000, M4_EXEC_WORK,
	  DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE },
	{ M4_EXEC_CDM_GPU, 0x4000, M4_EXEC_CDM, DRM_ASAHI_BIND_READ },
	{ M4_EXEC_PREEMPT_GPU, 0x4000, M4_EXEC_PREEMPT,
	  DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE },
	{ M4_EXEC_AUX_GPU, 0x4000, M4_EXEC_AUX,
	  DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE },
	{ M4_EXEC_OUTPUT_GPU, 0x4000, M4_EXEC_OUTPUT,
	  DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE },
	{ M4_EXEC_CONTEXT_STORE_GPU, 0x40000, M4_EXEC_CONTEXT_STORE,
	  DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE },
	{ M4_EXEC_POOL_STATE_GPU, 0x20000, M4_EXEC_POOL_STATE,
	  DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE },
	{ M4_EXEC_PAGE_LIST_GPU, 0x4000, M4_EXEC_PAGE_LIST,
	  DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE },
	{ M4_EXEC_POOL_GPU, 0x2000000, 0,
	  DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE },
};

/* Fixed J713 boot-profile physical admission index for m4_provider_page_allowed.
 *
 * Built ONCE (m4_provider_page_index_build_locked) after HELLO regions, READY
 * buffers/tables and execution memory are allocated; every source below is
 * retained unchanged to reboot. It replaces only the per-page resource-tree
 * walk and the immutable exclusion loops with O(log n) lookups; accept/reject
 * semantics are identical to the retained scan (m4_provider_page_scan).
 *
 * Not indexed, still checked live on every call: the session/error gate,
 * pfn_valid/PageReserved, the append-only r->upper_extra AP tables and the
 * execution pool's own immutable index. Dynamically allocated AP tables and
 * upper claims stay in the checked PA ownership index of the upper owner.
 *
 * RAM topology is a boot snapshot of the SystemRAM resource union. If RAM
 * hotplug or any iomem topology change is ever supported, this index must be
 * explicitly rebuilt (or ram_built cleared to force the live region walk); it
 * is never silently revalidated.
 */
#define M4_PROVIDER_EXCLUSION_MAX 256U
#define M4_PROVIDER_RAM_MAX 64U

struct m4_provider_exclusion {
	u64 start, end; /* [start, end) */
	/* Exact retained allocation an external role may exempt, or NULL. */
	const struct m4_ready_buffer *owner;
};

struct m4_provider_ram_range {
	u64 start, last; /* inclusive, merged SystemRAM resource union */
	bool verified; /* whole range proved REGION_INTERSECTS at build */
};

struct m4_provider_page_index {
	struct m4_provider_exclusion exclusion[M4_PROVIDER_EXCLUSION_MAX];
	u64 reach[M4_PROVIDER_EXCLUSION_MAX]; /* prefix maximum of exclusion[].end */
	struct m4_provider_ram_range ram[M4_PROVIDER_RAM_MAX];
	u32 exclusion_count, ram_count;
	bool exclusions_built, ram_built;
};

/* One ordinary lane owner admitted to start: its exact lane item and core
 * job. A borrowed identity pair, never a ref, phase proof or lane authority.
 */
struct m4_user_inflight {
	struct gpu_m4_drm_provider_lane_item *item;
	struct gpu_m4_drm_provider_job *job;
};

#define M4_USER_INFLIGHT_MAX GPU_M4_DRM_LANE_MAX_INFLIGHT

/* UMA slot numbers: compute 1..32 and render 33..64 share one index space
 * (checked against the encoded domains in gpu-m4-user-native.h).
 */
#define M4_USER_SLOT_TABLE 65U

/* Host UMA slot banks. Two in-flight jobs never share a slot or render PB ID
 * (slot - 33); a bank is busy while any accepted (prepared) or in-flight job
 * holds it, never because of a retired last owner. Bank b, ordinal r < size:
 * compute slot 1 + b * size + r, render slot 33 + b * size + r. The encoded
 * domains admit 2 x 16 or 4 x 8; until a second job can be admitted only
 * bank 0 exists and it spans each whole 32-slot domain (lowest free slot).
 */
#define M4_USER_BANK_MAX 4U
#define M4_USER_BANK_COUNT 1U
#define M4_USER_BANK_SIZE 32U
/* Two packets in flight (inflight2): the working runtime's two disjoint
 * banks of 16 slots per engine class (compute 1+16b+r, render 33+16b+r,
 * PB ID slot - 33).
 */
#define M4_USER_INFLIGHT2_BANKS 2U
#define M4_USER_INFLIGHT2_BANK_SIZE 16U
/* Three or four packets in flight: four banks of 8 per class. */
#define M4_USER_INFLIGHT4_BANKS 4U
#define M4_USER_INFLIGHT4_BANK_SIZE 8U

/* Admission vetoes of a successor beside the in-flight head, in the order
 * they are tested (the first one stops admission, head-of-line).
 */
enum m4_user_veto {
	M4_USER_VETO_DISABLED,	/* inflight2=0: serial lane */
	M4_USER_VETO_STATE,	/* unresolvable/unprepared owner */
	M4_USER_VETO_PUBLISHING, /* an in-flight job still publishes its window */
	M4_USER_VETO_QUEUE,	/* same logical queue (same-queue overlap off) */
	M4_USER_VETO_BANK,	/* its UMA slot bank is held in flight */
	M4_USER_VETO_ASID,	/* not co-resident on its own bound context */
	/* A render first command behind a compute tail: their target counters
	 * are independent, and a render's prior edge must name an older target.
	 */
	M4_USER_VETO_CLASS,
	M4_USER_VETO_RING,	/* work-ring credit (< 128 vs all four consumers) */
	M4_USER_VETO_PIPE,	/* submit-pipe READ/CFI capacity */
	M4_USER_VETO_DEPTH,	/* inflight_jobs packets already in flight */
	M4_USER_VETOES,
};

/* REMOVE-DEBUG observational inflight2 counters, under the GPU mutex. */
struct m4_user_inflight2_stats {
	u64 veto[M4_USER_VETOES];
	/* Successors admitted beside a head: of the same VM on another queue,
	 * of another VM, of the same queue; and first commands carrying the
	 * global-tail barrier on a render (0x71) or compute (15) predecessor.
	 */
	u64 admitted, admitted_vm, admitted_foreign, admitted_queue;
	u64 barrier[2];
	/* BARRIER_NONE compute-only jobs published with no global-tail wait,
	 * and candidates kept back behind one.
	 */
	u64 none, veto_none;
	/* Time with two jobs in flight (successor admission to the first of
	 * the pair leaving), and accepted jobs per bank.
	 */
	u64 overlap_ns, overlap_max, bank_jobs[M4_USER_BANK_MAX];
	/* At each head retirement, what stood behind it: a successor already
	 * in flight (same queue / other), a READY candidate not admitted
	 * (by its latest veto; [M4_USER_VETOES] = never asked), or nothing
	 * READY, then by the retiring job's own queue (GPU_M4_DRM_QUEUE_NEXT_*).
	 */
	u64 retire_paired_queue, retire_paired_other, retire_ready, retire_idle;
	u64 retire_veto[M4_USER_VETOES + 1];
	u64 retire_next[8]; /* GPU_M4_DRM_QUEUE_NEXTS */
	/* Successors admitted while the head still published its window. */
	u64 admitted_partial;
};

struct m4_user_vm;

/* Ordinary user contexts: TTBAT entries 1..63 (0 is the firmware's own).
 * ASID 1 is shared by every VM without a dedicated context; its root is
 * swapped between them as before. 2..63 are dedicated, each to one VM from
 * its first queue until its last lease is gone.
 */
#define M4_USER_ASIDS 64U
#define M4_USER_ASID_SHARED 1U

/* One user context: binding authority, leases and the installed binding,
 * mirrored from the provider core's per-ASID binding under the GPU mutex.
 */
struct m4_user_asid {
	/* The VM this dedicated context belongs to; NULL when free or shared.
	 * Only its own jobs may bind here, so a dedicated root changes only
	 * between that VM's own graphs.
	 */
	const struct m4_user_vm *owner;
	/* Resident graph and the TTBAT lower word installed for it (root PA |
	 * valid | asid << 48); 0 when unbound. The upper word is always zero.
	 */
	struct gpu_m4_drm_provider_version *resident;
	u64 low, generation;
	/* The VM's own lease and one per notifier, whose records name this
	 * context. Retained native owners (stages, last-owner units, idle
	 * caches) hold their queue, which holds both. At zero a dedicated
	 * context is unbound (both words cleared, ASIDE1OS) and freed.
	 */
	u32 leases;
	/* A failed release left an unknown TTBAT or core state: never reused. */
	bool quarantined;
	/* REMOVE-DEBUG: successful binds here: resident, adopted, installed. */
	u64 binds[3];
};

struct m4_user_render_alloc;

struct m4_gpu_provider_runtime {
	struct m4_gpu_hello *hello;
	struct m4_gpu_ready *ready;
	struct m4_gpu_exec_memory *execution;
	struct gpu_m4_drm *frontend;
	struct gpu_m4_drm_vm *frontend_vm;
	struct gpu_m4_drm_provider *provider;
	struct gpu_m4_drm_provider_vm *vm;
	struct gpu_m4_drm_provider_version *version[2], *resident;
	struct gpu_m4_drm_vm_owner *snapshot;
	struct gpu_m4_drm_provider_version *base, *common;
	struct gpu_m4_drm_vm_owner *base_snapshot, *work_snapshot;
	struct gpu_m4_drm_provider_version *work_version;
	struct m4_provider_external external[M4_PROVIDER_EXTERNAL_COUNT];
	struct m4_provider_code code[M4_PROVIDER_CODE_COUNT];
	struct m4_provider_writeback_output wb_output;
	struct m4_provider_writeback_output wb_input;
	struct gpu_m4_drm_provider_private_lease *wb_input_claim;
	const struct firmware *wb_firmware;
	struct gpu_m4_wb_fixture wb_fixture;
	u32 writeback_challenge;
	struct m4_gpu_provider_work *work, *staged_work;
	struct m4_provider_ts_storage timestamp;
	struct gpu_m4_drm_timestamp_aliases *ts_aliases;
	struct gpu_m4_drm_provider_lane *ts_lane;
	struct gpu_m4_drm_provider_queue *ts_queue;
	struct m4_provider_ts_job *ts_job;
	struct dma_fence *ts_finished;
	struct work_struct ts_reap_work;
	struct drm_asahi_params_global params;
	refcount_t ts_queue_refs;
	struct gpu_m4_drm_vm_leaf context[4];
	const void *context_identity;
	u64 context_pa[4], root[2], resident_low, resident_high, generation;
	u64 started, finished;
	u64 external_bytes;
	refcount_t session_refs;
	/* Ordinary clients use the same READY/provider and hardware lane, not the
	 * fixed V0/V1 diagnostic manifests below. Final puts run from user_reap.
	 */
	const struct gpu_m4_drm_provider_job_runtime *user_runtime;
	struct list_head user_vm_garbage, user_queue_garbage, user_handle_put;
	struct work_struct user_reap;
	/* Housekeeping cold collection (no cell/byte shortage) runs at most once
	 * per M4_USER_NATIVE_COLLECT_MS from user_reap; a skipped pass re-kicks
	 * user_reap from this delayed work. Demand paths collect inline.
	 */
	struct delayed_work user_collect;
	unsigned long user_collect_last;
	bool user_collect_deferred;
	struct gpu_m4_drm_provider_lane *user_lane;
	/* Lane order mirror: [0] is the lane head. user_starting is the newest
	 * admitted owner, the only one that may still publish uppers, aliases
	 * or its start; it leaves with its in-flight entry.
	 */
	struct m4_user_inflight user_inflight[M4_USER_INFLIGHT_MAX], user_starting;
	/* Ordinary user contexts; [0] is unused. user_binding_revision
	 * advances on every change of any of them (installation, adoption,
	 * release), so a remembered health pass never outlives one.
	 */
	struct m4_user_asid user_asid[M4_USER_ASIDS];
	u64 user_binding_revision;
	/* Every successful ordinary bind, any kind: a job's age for the native
	 * caches (binding generations no longer advance per job).
	 */
	u64 user_bind_sequence;
	/* Last live READY health pass inside one ordinary lane-worker hold: its
	 * hold token, upper owner revision and resident bindings (see
	 * m4_provider_ready_health()). 0 = none.
	 */
	u64 health_hold, health_revision, health_generation, health_binding;
	const struct gpu_m4_drm_provider_version *health_resident;
	/* Lane-worker hold of the current service snapshot (0 = none) and the
	 * upper owner revision of the last upper graph pass at publication
	 * (m4_user_service_locked(), m4_user_upper_health_locked()).
	 */
	u64 service_hold, upper_health_revision;
	/* ktime of the last complete service snapshot (poll_health_us). */
	u64 service_ns;
	/* Accepted and in-flight holders per bank; bit b of busy = holders[b]. */
	u32 user_bank_holders[M4_USER_BANK_MAX], user_bank_busy;
	/* inflight2 sampled once at m4_user_init(): the lane cap is two, the
	 * banks are M4_USER_INFLIGHT2_BANKS x M4_USER_INFLIGHT2_BANK_SIZE and a
	 * successor may be admitted beside the head. Otherwise one bank spans
	 * each whole domain and every successor is vetoed.
	 */
	bool user_inflight2, user_inflight2_wide;
	u32 user_inflight_max; /* packet cap fixed at init: bank count, credits */
	u32 user_bank_count, user_bank_size;
	/* Positions used per bank: the largest per-class window of any queue
	 * created so far (0: the whole bank).
	 */
	u32 user_bank_used;
	/* Fresh units built by the latest closed queue of each task name (a
	 * small LRU table): the next queue of that name prebuilds its pools.
	 */
	struct {
		char comm[16];
		u32 units, render;
	} user_prebuild[8];
	u32 user_prebuild_next;
	/* Largest render tilemap, TPC and parameter heap any command needed,
	 * per layer class (single, layered), device-wide.
	 */
	u64 user_render_highwater[2][3];
	/* Stages that chose a bank and are still materializing (not yet
	 * accepted): a concurrent build must not pick the same bank.
	 */
	u32 user_bank_staging[M4_USER_BANK_MAX];
	/* Banks of the admitted in-flight jobs (user_inflight[]). */
	u32 user_bank_flight;
	/* The last reachable wire record per engine of any job, a copy that
	 * outlives its job (inflight2): a successor published behind a head's
	 * earlier commands sees the head's later records as its own physical
	 * successors after the head retired.
	 */
	u64 user_wire_last[3][16]; /* struct m4_user_engine (gpu-m4-user-wire.h) */
	u64 user_overlap_start; /* REMOVE-DEBUG: ns of the current pair, 0 = none */
	struct m4_user_inflight2_stats user_inflight2_stats;
	/* Global per-engine publication order of reachable wire records. */
	u64 user_wire_sequence[3];
	struct list_head user_native_garbage, user_native_history_garbage;
	struct list_head user_native_settle; /* complete histories to settle */
	struct list_head user_native_retained;
	struct list_head user_native_units;
	struct list_head user_job_garbage;
	u64 user_native_bytes;
	u64 user_pb_grow_area; /* PB growth hint: smallest partially rendered area */
	struct m4_user_native_unit *user_cell_owner[M4_ORDINARY_WORK_CELLS];
	/* Next never-used notifier record slot in the record cells. */
	u32 user_record_next;
	struct m4_user_native_unit *user_unit_last[3];
	/* Per UMA slot (compute 1..32, render 33..64): the unit of its latest
	 * committed use (receipt_complete), cleared when that unit is cut cold
	 * or freed. Only this unit can be the slot's current owner, so the owner
	 * lookup is O(1) instead of a walk of every retained unit.
	 */
	struct m4_user_native_unit *user_slot_unit[M4_USER_SLOT_TABLE];
	/* render_shared: the retained render allocator holding each render
	 * UMA ID / PB ID (slot), and slots released by a quiescent one (their
	 * PB descriptor slice is a saved, released descriptor, not zero) or
	 * quarantined (an allocator released without catch-up proof).
	 */
	struct m4_user_render_alloc *user_slot_alloc[M4_USER_SLOT_TABLE];
	DECLARE_BITMAP(user_slot_released, M4_USER_SLOT_TABLE);
	DECLARE_BITMAP(user_slot_quarantined, M4_USER_SLOT_TABLE);
	struct m4_user_native_stage *user_last[3];
	struct m4_user_probe *user_probe;
	u64 user_client_id;
	u32 user_target[3], user_physical_used;
	bool user_stage_cells; /* stage_cells, sampled at m4_user_init() */
	/* Per held event: the newest target whose producer was made reachable
	 * (m4_user_wire_publish_locked()), bounding its runtime-wide stage cell.
	 */
	u32 user_stage_exposed[3];
	bool user_render_stage_waits; /* render_stage_waits, likewise */
	bool ordinary, prerun_staging;
	int error, handoff_get, handoff_put, tlbi;
	bool attempted, prepared, complete, selected, v0_extra, writing;
	bool compute_manifest, writeback_manifest, writeback_input_manifest, manifest_ready;
	bool timestamp_manifest, ts_attempted;
	/* Immutable boot-profile page admission index; gpu->lock serializes. */
	struct m4_provider_page_index page_index;
};

struct m4_user_queue;
struct m4_user_notifier;
static const struct gpu_m4_drm_ops m4_user_frontend_ops;
static int m4_user_health_locked(struct m4_gpu_provider_runtime *p);
static int m4_user_live_health_locked(struct m4_gpu_provider_runtime *p);
static int m4_user_binding_check_locked(struct m4_gpu_provider_runtime *p);
static void m4_user_init(struct m4_gpu_provider_runtime *p);
static struct m4_user_notifier *m4_user_notifier_create(struct m4_user_queue *q);
static void m4_user_notifier_put_locked(struct m4_user_notifier *n);
static void m4_user_native_queue_close_locked(struct m4_user_queue *q);
static void m4_user_collect_native(struct m4_gpu_provider_runtime *p);
static int m4_user_runtime_prepare(struct m4_gpu_provider_runtime *p);

static void m4_provider_fault_locked(struct m4_gpu_provider_runtime *p, int error);
static int m4_provider_native_coverage(struct m4_gpu_provider_runtime *p);
static int m4_provider_work_validate_locked(struct m4_gpu_provider_runtime *p);
static int m4_provider_work_health(struct m4_gpu_provider_runtime *p);
/* Implemented by the actual retained fixture/work owner, included afterwards. */
static int m4_provider_wb_prebind_stage(struct m4_gpu_provider_runtime *p);
static int m4_provider_wb_prebind_protect_locked(struct m4_gpu_provider_runtime *p);
static void m4_provider_wb_prebind_fault(struct m4_gpu_provider_runtime *p, int error);
/* Actual TS table/work adapters are included after this runtime and work.h. */
static int m4_provider_ts_upper_stage_locked(struct m4_gpu_provider_runtime *p);
static int m4_provider_ts_owner_check_locked(void *owner,
	struct gpu_m4_drm_provider *provider, struct gpu_m4_drm_upper *upper,
	struct gpu_m4_drm *frontend, struct mutex *lock);
static int m4_provider_ts_work_accept_locked(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_job *job, struct m4_gpu_provider_work **work);
static int m4_provider_ts_work_prepare_locked(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_job *job, struct m4_gpu_provider_work *work);
static int m4_provider_ts_work_publish_locked(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_job *job, struct m4_gpu_provider_work *work);
static int m4_provider_ts_work_retired_locked(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_job *job, struct m4_gpu_provider_work *work);
static int m4_provider_ts_work_health_locked(struct m4_gpu_provider_runtime *p);
static void m4_provider_ts_work_complete_locked(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_job *job, struct m4_gpu_provider_work *work);
static void m4_provider_ts_work_fault_locked(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_job *job, struct m4_gpu_provider_work *work, int error);
static void m4_provider_ts_work_release_locked(struct m4_gpu_provider_runtime *p,
	struct m4_gpu_provider_work *work);

/* The probe already permanently retains its power/module/device baseline and
 * suppresses bind/unbind. These additional references retain that EXACT owner,
 * not an unrelated device cookie. No final qualifier-baseline put exists.
 */
static int m4_provider_session_get(void *cookie)
{
	struct m4_gpu_provider_runtime *p = cookie;
	struct m4_gpu_hello *h = p->hello;
	struct m4_gpu_power *gpu = h->gpu;
	bool admitted;
	int ret = 0;

	lockdep_assert_not_held(&gpu->lock);
	mutex_lock(&gpu->lock);
	/* Lifetime acquisition is separate from work admission. The same private
	 * frontend must pin the command arena before RUN, then be retained through
	 * READY and eventual public registration. No fake READY/policy state.
	 */
	admitted = h->ready && h->complete && h->ready->complete &&
		h->ready->policy_complete && gpu->firmware_start_attempted;
	if (!admitted && p->ordinary && p->prerun_staging)
		admitted = !h->published && !gpu->firmware_start_attempted &&
			gpu->stopped_verified && !(readl(gpu->asc + 0x44) & BIT(4));
	if (gpu->hello != h || h->ready != p->ready || !h->ready ||
	    h->ready->execution != p->execution || h->ready->provider_runtime != p ||
	    !p->attempted || p->error || gpu->error || h->error || h->ready->error ||
	    !gpu->voted || !admitted ||
	    !refcount_inc_not_zero(&p->session_refs)) {
		ret = -EIO;
	} else {
		__module_get(THIS_MODULE);
		get_device(gpu->dev);
		pm_runtime_get_noresume(gpu->dev);
	}
	mutex_unlock(&gpu->lock);
	return ret;
}

static void m4_provider_session_put(void *cookie)
{
	struct m4_gpu_provider_runtime *p = cookie;
	struct m4_gpu_power *gpu = p->hello->gpu;
	bool drop;

	lockdep_assert_not_held(&gpu->lock);
	mutex_lock(&gpu->lock);
	/* Refuse consuming the permanent qualifier baseline even on API misuse. */
	drop = refcount_dec_not_one(&p->session_refs);
	mutex_unlock(&gpu->lock);
	if (drop) {
		pm_runtime_put_noidle(gpu->dev);
		put_device(gpu->dev);
		module_put(THIS_MODULE);
	}
}

static int m4_provider_validate_lock(void *cookie, struct mutex *lock)
{
	struct m4_gpu_provider_runtime *p = cookie;

	return lock == &p->hello->gpu->lock &&
		p->hello->gpu->hello == p->hello &&
		p->hello->ready->provider_runtime == p ? 0 : -EPERM;
}

static bool m4_provider_overlap(u64 pa, u64 first, u64 size)
{
	return size && (first > U64_MAX - size ||
		(pa < first + size && first < pa + GPU_M4_DRM_PAGE_SIZE));
}

static bool m4_provider_buffer_overlap(u64 pa, const struct m4_ready_buffer *b)
{
	return m4_provider_overlap(pa, b->pa, b->size);
}

/* Session part of the page oracle: exact live HELLO/READY/execution owner and
 * no latched runtime error. Cheap and evaluated on every call, including for
 * pages whose immutable legitimacy was admitted earlier.
 */
static int m4_provider_page_gate(struct m4_gpu_provider_runtime *p)
{
	struct m4_gpu_hello *h = p->hello;
	struct m4_gpu_ready *r = h->ready;
	struct m4_gpu_exec_memory *e = r->execution;

	lockdep_assert_held(&h->gpu->lock);
	if (h->gpu->hello != h || r != p->ready || e != p->execution ||
	    p->error || !e || r->upper_extra_count > ARRAY_SIZE(r->upper_extra))
		return -EPERM;
	return 0;
}

/* Retained reference scan of the immutable exclusions. Used before the index is
 * built (or if it could not be built) and as the differential test oracle.
 * Upper extras and the execution pool are NOT part of this scan.
 */
static bool m4_provider_page_scan(struct m4_gpu_provider_runtime *p, u64 pa,
				  const struct m4_ready_buffer *allowed)
{
	struct m4_gpu_hello *h = p->hello;
	struct m4_gpu_ready *r = h->ready;
	struct m4_gpu_exec_memory *e = r->execution;
	struct platform_device *pdev = to_platform_device(h->gpu->dev);
	struct resource *res;

	for (u32 i = 0; i < ARRAY_SIZE(h->region); i++)
		if (m4_provider_overlap(pa, h->region[i].start, resource_size(&h->region[i])))
			return true;
	/* Reject ALL platform MMIO resources, including enclosing unmapped ranges. */
	for (u32 i = 0; (res = platform_get_resource(pdev, IORESOURCE_MEM, i)); i++)
		if (m4_provider_overlap(pa, res->start, resource_size(res)))
			return true;
	if (m4_provider_overlap(pa, page_to_phys(h->lower), PAGE_SIZE))
		return true;
	for (u32 i = 0; i < ARRAY_SIZE(r->table); i++)
		if (m4_provider_overlap(pa, page_to_phys(r->table[i]), PAGE_SIZE))
			return true;
	for (u32 i = 0; i < ARRAY_SIZE(r->buffer); i++)
		if (&r->buffer[i] != allowed && m4_provider_buffer_overlap(pa, &r->buffer[i]))
			return true;
	if (m4_provider_buffer_overlap(pa, &e->context) ||
	    m4_provider_buffer_overlap(pa, &e->zero_stamp) ||
	    m4_provider_buffer_overlap(pa, &e->notification))
		return true;
	for (u32 engine = 0; engine < ARRAY_SIZE(e->queue); engine++)
		for (u32 i = 0; i < ARRAY_SIZE(e->queue[engine]); i++)
			/* Session-role validation admits its exact retained queue
			 * owner, never a client PA range. Native admission passes NULL.
			 */
			if (&e->queue[engine][i] != allowed &&
			    m4_provider_buffer_overlap(pa, &e->queue[engine][i]))
				return true;
	for (u32 i = 0; i < ARRAY_SIZE(e->buffer); i++)
		if (&e->buffer[i] != allowed && m4_provider_buffer_overlap(pa, &e->buffer[i]))
			return true;
	for (u32 i = 0; i < ARRAY_SIZE(e->page); i++)
		if (m4_provider_overlap(pa, page_to_phys(e->page[i]), PAGE_SIZE))
			return true;
	return false;
}

static bool m4_provider_exclusion_add(struct m4_provider_page_index *x, u64 first,
				      u64 size, const struct m4_ready_buffer *owner)
{
	struct m4_provider_exclusion *n;

	/* Exactly m4_provider_overlap(): empty never overlaps, wrap overlaps all. */
	if (!size)
		return true;
	if (x->exclusion_count >= ARRAY_SIZE(x->exclusion))
		return false;
	n = &x->exclusion[x->exclusion_count++];
	*n = first > U64_MAX - size ?
		(struct m4_provider_exclusion) { 0, U64_MAX, owner } :
		(struct m4_provider_exclusion) { first, first + size, owner };
	return true;
}

static int m4_provider_exclusion_compare(const void *a, const void *b)
{
	u64 x = ((const struct m4_provider_exclusion *)a)->start;
	u64 y = ((const struct m4_provider_exclusion *)b)->start;

	return (x > y) - (x < y);
}

static int m4_provider_ram_compare(const void *a, const void *b)
{
	u64 x = ((const struct m4_provider_ram_range *)a)->start;
	u64 y = ((const struct m4_provider_ram_range *)b)->start;

	return (x > y) - (x < y);
}

static int m4_provider_ram_collect(struct resource *res, void *cookie)
{
	struct m4_provider_page_index *x = cookie;

	if (x->ram_count >= ARRAY_SIZE(x->ram) || res->end < res->start)
		return -ENOSPC;
	x->ram[x->ram_count++] = (struct m4_provider_ram_range) {
		.start = res->start, .last = res->end,
	};
	return 0;
}

/* Snapshot the union of all SystemRAM resources (top-level and descendants).
 * A page wholly inside one merged range whose entire span the actual
 * region_intersects() classified REGION_INTERSECTS is REGION_INTERSECTS too:
 * every top-level resource it meets is RAM or RAM-covered over that span. A
 * page disjoint from the union is REGION_DISJOINT. Boundary pages and
 * unverified ranges keep the live walk.
 */
static void m4_provider_ram_build(struct m4_provider_page_index *x)
{
	u32 kept = 0;

	x->ram_built = false;
	x->ram_count = 0;
	if (walk_iomem_res_desc(IORES_DESC_NONE, IORESOURCE_SYSTEM_RAM, 0, U64_MAX,
				x, m4_provider_ram_collect) == -ENOSPC) {
		x->ram_count = 0;
		return;
	}
	sort(x->ram, x->ram_count, sizeof(x->ram[0]), m4_provider_ram_compare, NULL);
	for (u32 i = 0; i < x->ram_count; i++) {
		struct m4_provider_ram_range *last = kept ? &x->ram[kept - 1] : NULL;

		if (last && (last->last == U64_MAX || x->ram[i].start <= last->last + 1)) {
			last->last = max(last->last, x->ram[i].last);
			continue;
		}
		x->ram[kept++] = x->ram[i];
	}
	x->ram_count = kept;
	for (u32 i = 0; i < x->ram_count; i++) {
		struct m4_provider_ram_range *range = &x->ram[i];

		range->verified = range->last - range->start < U64_MAX &&
			range->last - range->start < SIZE_MAX &&
			region_intersects(range->start, range->last - range->start + 1,
					  IORESOURCE_SYSTEM_RAM, IORES_DESC_NONE) ==
			REGION_INTERSECTS;
	}
	x->ram_built = true;
}

static bool m4_provider_page_ram(const struct m4_provider_page_index *x, u64 pa)
{
	const u64 last = pa + GPU_M4_DRM_PAGE_SIZE - 1;
	const struct m4_provider_ram_range *range;
	u32 low = 0, high = x->ram_count;

	if (!x->ram_built)
		goto live;
	while (low < high) {
		u32 middle = low + (high - low) / 2;

		if (x->ram[middle].start <= last)
			low = middle + 1;
		else
			high = middle;
	}
	/* Merged ranges are disjoint and sorted: only ram[low - 1] can meet it. */
	if (!low || x->ram[low - 1].last < pa)
		return false; /* No SystemRAM resource intersects: REGION_DISJOINT. */
	range = &x->ram[low - 1];
	if (range->verified && range->start <= pa && range->last >= last)
		return true;
live:
	return region_intersects(pa, GPU_M4_DRM_PAGE_SIZE, IORESOURCE_SYSTEM_RAM,
				 IORES_DESC_NONE) == REGION_INTERSECTS;
}

/* True when any exclusion other than the exactly allowed owner overlaps the
 * page. exclusion[] is sorted by start; reach[] is its prefix maximum end, so
 * the backward walk stops at the first prefix that cannot reach this page and
 * an accepted page costs one binary search.
 */
static bool m4_provider_exclusion_hit(const struct m4_provider_page_index *x, u64 pa,
				      const struct m4_ready_buffer *allowed)
{
	u32 low = 0, high = x->exclusion_count;

	while (low < high) {
		u32 middle = low + (high - low) / 2;

		if (x->exclusion[middle].start < pa + GPU_M4_DRM_PAGE_SIZE)
			low = middle + 1;
		else
			high = middle;
	}
	for (u32 i = low; i-- && x->reach[i] > pa;) {
		const struct m4_provider_exclusion *n = &x->exclusion[i];

		if (n->end > pa && (!n->owner || n->owner != allowed))
			return true;
	}
	return false;
}

/* Build once from the retained allocations, under the hardware mutex. Never
 * fails the session: an over-capacity profile keeps the reference scan.
 */
static void m4_provider_page_index_build_locked(struct m4_gpu_provider_runtime *p)
{
	struct m4_provider_page_index *x = &p->page_index;
	struct m4_gpu_hello *h = p->hello;
	struct m4_gpu_ready *r = h->ready;
	struct m4_gpu_exec_memory *e = r->execution;
	struct platform_device *pdev = to_platform_device(h->gpu->dev);
	struct resource *res;
	bool ok = true;

	lockdep_assert_held(&h->gpu->lock);
	x->exclusions_built = false;
	x->exclusion_count = 0;
	m4_provider_ram_build(x);
	if (r != p->ready || !e || e != p->execution)
		return;
	for (u32 i = 0; i < ARRAY_SIZE(h->region); i++)
		ok &= m4_provider_exclusion_add(x, h->region[i].start,
						resource_size(&h->region[i]), NULL);
	for (u32 i = 0; ok && (res = platform_get_resource(pdev, IORESOURCE_MEM, i)); i++)
		ok &= m4_provider_exclusion_add(x, res->start, resource_size(res), NULL);
	ok &= m4_provider_exclusion_add(x, page_to_phys(h->lower), PAGE_SIZE, NULL);
	for (u32 i = 0; i < ARRAY_SIZE(r->table); i++)
		ok &= m4_provider_exclusion_add(x, page_to_phys(r->table[i]), PAGE_SIZE, NULL);
	for (u32 i = 0; i < ARRAY_SIZE(r->buffer); i++)
		ok &= m4_provider_exclusion_add(x, r->buffer[i].pa, r->buffer[i].size,
						&r->buffer[i]);
	/* Common execution records are never exemptible, even by identity. */
	ok &= m4_provider_exclusion_add(x, e->context.pa, e->context.size, NULL);
	ok &= m4_provider_exclusion_add(x, e->zero_stamp.pa, e->zero_stamp.size, NULL);
	ok &= m4_provider_exclusion_add(x, e->notification.pa, e->notification.size, NULL);
	for (u32 engine = 0; engine < ARRAY_SIZE(e->queue); engine++)
		for (u32 i = 0; i < ARRAY_SIZE(e->queue[engine]); i++)
			ok &= m4_provider_exclusion_add(x, e->queue[engine][i].pa,
				e->queue[engine][i].size, &e->queue[engine][i]);
	for (u32 i = 0; i < ARRAY_SIZE(e->buffer); i++)
		ok &= m4_provider_exclusion_add(x, e->buffer[i].pa, e->buffer[i].size,
						&e->buffer[i]);
	for (u32 i = 0; i < ARRAY_SIZE(e->page); i++)
		ok &= m4_provider_exclusion_add(x, page_to_phys(e->page[i]), PAGE_SIZE, NULL);
	if (!ok) {
		x->exclusion_count = 0;
		return;
	}
	sort(x->exclusion, x->exclusion_count, sizeof(x->exclusion[0]),
	     m4_provider_exclusion_compare, NULL);
	for (u32 i = 0; i < x->exclusion_count; i++)
		x->reach[i] = i ? max(x->reach[i - 1], x->exclusion[i].end) :
			x->exclusion[i].end;
	x->exclusions_built = true;
}

/* External admission exempts one actual allocation ONLY after its exact role,
 * owner and DMA roundtrip have been checked. Native/context admission exempts
 * none. Existing firmware/table/bootstrap/MMIO exclusions remain independent.
 * Immutable RAM/exclusion facts use the boot-profile index when built; the
 * session gate, page flags, upper extras and pool index remain live.
 */
static int m4_provider_page_allowed(struct m4_gpu_provider_runtime *p, u64 pa,
				    const struct m4_ready_buffer *allowed)
{
	const struct m4_provider_page_index *x = &p->page_index;
	struct m4_gpu_ready *r = p->hello->ready;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (m4_provider_page_gate(p) || !pa ||
	    !gpu_m4_drm_aligned(pa) || pa >= BIT_ULL(42) ||
	    !pfn_valid(PHYS_PFN(pa)) || PageReserved(pfn_to_page(PHYS_PFN(pa))) ||
	    !m4_provider_page_ram(x, pa))
		return -EPERM;
	if (x->exclusions_built ? m4_provider_exclusion_hit(x, pa, allowed) :
	    m4_provider_page_scan(p, pa, allowed))
		return -EPERM;
	for (u32 i = 0; i < r->upper_extra_count; i++)
		if (m4_provider_overlap(pa, r->upper_extra[i].pa, PAGE_SIZE))
			return -EPERM;
	if (m4_exec_pool_overlap(r->execution, pa, GPU_M4_DRM_PAGE_SIZE, allowed))
		return -EPERM;
	return 0;
}

/* Provider admitted-page gate: the native pin's every PA already passed
 * m4_provider_validate_page at first admission and that pin still holds the
 * pages. Recheck only the ordinary oracle's live non-PA conditions. Fixture
 * manifests keep their role-specific full oracle.
 */
static int m4_provider_admitted_page(void *cookie, struct gpu_m4_drm_vm *vm,
				     const void *identity, u64 offset)
{
	struct m4_gpu_provider_runtime *p = cookie;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!p->ordinary)
		return 1;
	if (!identity || !gpu_m4_drm_aligned(offset) ||
	    !gpu_m4_drm_vm_belongs_to_frontend(vm, p->frontend))
		return -EPERM;
	return m4_provider_page_gate(p);
}

static int m4_provider_validate_page(void *cookie, struct gpu_m4_drm_vm *vm,
				     const void *identity, u64 offset, u64 pa)
{
	struct m4_gpu_provider_runtime *p = cookie;
	u32 index;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (p->ordinary) {
		/* The provider's context/native owner already proves identity, exact
		 * immutable PA inventory and policy. This oracle adds the independent
		 * session/foreign-firmware/table/MMIO exclusions, not a fixture role.
		 */
		if (!identity || !gpu_m4_drm_aligned(offset) ||
		    !gpu_m4_drm_vm_belongs_to_frontend(vm, p->frontend))
			return -EPERM;
		return m4_provider_page_allowed(p, pa, NULL);
	}
	if (!vm || vm != p->frontend_vm || !identity || !gpu_m4_drm_aligned(offset))
		return -EPERM;
	ret = m4_provider_page_allowed(p, pa, NULL);
	if (ret)
		return ret;
	if (p->timestamp_manifest && identity == p->timestamp.gem) {
		struct m4_provider_ts_storage *t = &p->timestamp;

		/* Named factory admission precedes the real pin. Upper timestamps do
		 * not use the lower context fallback or acquire a client mapping.
		 */
		if (!p->compute_manifest || p->writeback_manifest || offset ||
		    !t->prepared || !t->mapped || !t->initialized || !t->preimage ||
		    t->size != GPU_M4_DRM_PAGE_SIZE || t->gem->size != t->size ||
		    iosys_map_is_null(&t->map) || t->map.is_iomem ||
		    !gpu_m4_drm_bo_is_native(t->gem) ||
		    !gpu_m4_drm_bo_belongs_to_frontend(t->gem, p->frontend) ||
		    !gpu_m4_drm_bo_belongs_to_vm(t->gem, vm) ||
		    gpu_m4_drm_bo_policy(t->gem) != t->policy ||
		    !gpu_m4_compute_packet_separate(t->preimage, t->size, t->map.vaddr, t->size))
			return -EPERM;
		if (t->pa && t->pa != pa)
			return -ESTALE;
		for (u32 i = 0; i < ARRAY_SIZE(p->code); i++)
			if (p->code[i].pa == pa)
				return -EPERM;
		for (u32 i = 0; i < ARRAY_SIZE(p->context_pa); i++)
			if (p->context_pa[i] == pa)
				return -EPERM;
		if (p->wb_output.pa == pa || p->wb_input.pa == pa)
			return -EPERM;
		t->pa = pa;
		return 0;
	}
	for (u32 role = 0; p->writeback_manifest &&
	     role < 1U + p->writeback_input_manifest; role++) {
		struct m4_provider_writeback_output *w = role ? &p->wb_input : &p->wb_output;
		struct m4_provider_writeback_output *other = role ? &p->wb_output : &p->wb_input;

		if (identity != w->gem)
			continue;

		/* Metadata only, including after cache prepare. Never sample payload. */
		if (!p->compute_manifest || offset || !w->mapped || !w->initialized ||
		    !w->preimage || iosys_map_is_null(&w->map) || w->map.is_iomem ||
		    w->va != (role ? GPU_M4_WB_INPUT_VA : M4_EXEC_OUTPUT_GPU) ||
		    w->size != GPU_M4_DRM_PAGE_SIZE ||
		    !gpu_m4_drm_bo_is_native(w->gem) ||
		    !gpu_m4_drm_bo_belongs_to_vm(w->gem, vm) || w->gem->size != w->size ||
		    gpu_m4_drm_bo_policy(w->gem) != GPU_M4_DRM_CPU_WRITEBACK)
			return -EPERM;
		if (w->pa && w->pa != pa)
			return -ESTALE;
		if (p->timestamp_manifest && p->timestamp.pa == pa)
			return -EPERM;
		if (p->writeback_input_manifest && other->pa == pa)
			return -EPERM;
		for (u32 i = 0; i < ARRAY_SIZE(p->code); i++)
			if (p->code[i].pa == pa)
				return -EPERM;
		for (u32 i = 0; i < ARRAY_SIZE(p->context_pa); i++)
			if (p->context_pa[i] == pa)
				return -EPERM;
		w->pa = pa;
		return 0;
	}
	for (u32 i = 0; p->compute_manifest && i < ARRAY_SIZE(p->code); i++) {
		struct m4_provider_code *c = &p->code[i];

		if (identity != c->gem)
			continue;
		if (offset || !c->prepared || !c->uploaded ||
		    c->va != m4_provider_code_va[i] ||
		    !gpu_m4_drm_bo_is_native(c->gem) ||
		    !gpu_m4_drm_bo_belongs_to_vm(c->gem, vm) ||
		    c->gem->size != GPU_M4_DRM_PAGE_SIZE ||
		    gpu_m4_drm_bo_policy(c->gem) != GPU_M4_DRM_CPU_NC)
			return -EPERM;
		if (c->pa && c->pa != pa)
			return -ESTALE;
		if (p->timestamp_manifest && p->timestamp.pa == pa)
			return -EPERM;
		if (p->writeback_manifest && p->wb_output.pa == pa)
			return -EPERM;
		if (p->writeback_input_manifest && p->wb_input.pa == pa)
			return -EPERM;
		for (u32 j = 0; j < ARRAY_SIZE(p->code); j++)
			if (j != i && p->code[j].pa == pa)
				return -EPERM;
		for (u32 j = 0; j < ARRAY_SIZE(p->context_pa); j++)
			if (p->context_pa[j] == pa)
				return -EPERM;
		c->pa = pa;
		return 0;
	}
	if (offset >= GPU_M4_DRM_UAT_CONTEXT_SIZE)
		return -EPERM;
	index = offset / GPU_M4_DRM_PAGE_SIZE;
	if ((p->context_identity && p->context_identity != identity) ||
	    (p->context_pa[index] && p->context_pa[index] != pa))
		return -ESTALE;
	for (u32 i = 0; i < ARRAY_SIZE(p->context_pa); i++)
		if (i != index && p->context_pa[i] == pa)
			return -EPERM;
	for (u32 i = 0; i < ARRAY_SIZE(p->code); i++)
		if (p->code[i].pa == pa)
			return -EPERM;
	if (p->writeback_manifest && p->wb_output.pa == pa)
		return -EPERM;
	if (p->writeback_input_manifest && p->wb_input.pa == pa)
		return -EPERM;
	if (p->timestamp_manifest && p->timestamp.pa == pa)
		return -EPERM;
	p->context_identity = identity;
	p->context_pa[index] = pa;
	return 0;
}

static bool m4_provider_same_buffer(const struct m4_ready_buffer *a,
				    const struct m4_ready_buffer *b)
{
	return a->cpu == b->cpu && a->dma == b->dma && a->pa == b->pa &&
		a->va == b->va && a->size == b->size;
}

static int m4_provider_external_valid(struct m4_provider_external *x)
{
	struct m4_gpu_provider_runtime *p = x->runtime;
	struct m4_gpu_ready *r = p->hello->ready;
	u32 role;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!p->compute_manifest || p->error || r != p->ready ||
	    r->execution != p->execution || p->hello->gpu->hello != p->hello ||
	    r->provider_runtime != p || !p->hello->gpu->voted ||
	    !p->hello->gpu->firmware_start_attempted ||
	    !p->frontend_vm || !p->vm || !x->source || !x->pa || !x->pages ||
	    !refcount_read(&x->refs) || !refcount_read(&p->session_refs) ||
	    gpu_m4_drm_provider_private_check_locked(x->claim, p->vm, x->va, x->size))
		return -EPERM;
	for (role = 0; role < ARRAY_SIZE(p->external); role++)
		if (x == &p->external[role])
			break;
	if (role == ARRAY_SIZE(p->external) ||
	    x->va != m4_provider_external_role[role].va ||
	    x->size != m4_provider_external_role[role].size ||
	    x->access != m4_provider_external_role[role].access ||
	    x->pages != x->size / GPU_M4_DRM_PAGE_SIZE ||
	    x->source != (role == 0 ? &r->buffer[GPU_M4_UMA] :
		role == ARRAY_SIZE(p->external) - 1 ? r->execution->pool :
		&r->execution->buffer[m4_provider_external_role[role].id]) ||
	    !!x->pool != (role == ARRAY_SIZE(p->external) - 1))
		return -EPERM;
	return 0;
}

/* get/page run under the actual shared owner lock. These references retain
 * the permanent real session baseline, not fabricated GEM or device cookies.
 * Each external's explicit claim/allocation baseline is retained until reboot.
 */
static int m4_provider_external_get(void *cookie, struct gpu_m4_drm_vm *vm,
				    const void *identity)
{
	struct m4_provider_external *x = cookie;
	int ret = m4_provider_external_valid(x);

	if (ret || vm != x->runtime->frontend_vm || identity != x->source)
		return ret ? ret : -EPERM;
	ret = gpu_m4_drm_provider_private_get_locked(x->claim);
	if (ret)
		return ret;
	refcount_inc(&x->refs);
	refcount_inc(&x->runtime->session_refs);
	return 0;
}

static void m4_provider_external_put(void *cookie)
{
	struct m4_provider_external *x = cookie;
	struct mutex *lock = &x->runtime->hello->gpu->lock;

	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	/* Never consume the first-qualifier baseline or release DMA here. */
	if (refcount_dec_not_one(&x->refs)) {
		gpu_m4_drm_provider_private_put_locked(x->claim);
		if (!refcount_dec_not_one(&x->runtime->session_refs))
			m4_provider_fault_locked(x->runtime, -EPROTO);
	}
	mutex_unlock(lock);
}

static int m4_provider_external_page(void *cookie, u64 offset, u64 *pa)
{
	struct m4_provider_external *x = cookie;
	struct m4_gpu_provider_runtime *p = x->runtime;
	struct m4_ready_buffer *b;
	const struct m4_ready_buffer *expected;
	u32 index;
	int ret = m4_provider_external_valid(x);

	if (ret || !pa || !gpu_m4_drm_aligned(offset) || offset >= x->size)
		return ret ? ret : -EINVAL;
	index = offset / GPU_M4_DRM_PAGE_SIZE;
	b = x->pool ? &x->source[index] : x->source;
	expected = x->pool ? &x->pool[index] : &x->allocation;
	if (!m4_provider_same_buffer(b, expected) ||
	    !m4_exec_backing_valid(p->hello, b, b->size, !!b->va) ||
	    x->pa[index] != b->pa + (x->pool ? 0 : offset))
		return -ESTALE;
	ret = m4_provider_page_allowed(p, x->pa[index], b);
	if (ret)
		return ret;
	for (u32 i = 0; i < ARRAY_SIZE(p->context_pa); i++)
		if (x->pa[index] == p->context_pa[i])
			return -EPERM;
	for (u32 i = 0; i < ARRAY_SIZE(p->code); i++)
		if (x->pa[index] == p->code[i].pa)
			return -EPERM;
	if (p->writeback_manifest && x->pa[index] == p->wb_output.pa)
		return -EPERM;
	if (p->writeback_input_manifest && x->pa[index] == p->wb_input.pa)
		return -EPERM;
	if (p->timestamp_manifest && x->pa[index] == p->timestamp.pa)
		return -EPERM;
	*pa = x->pa[index];
	return 0;
}

static int m4_provider_external_stage_locked(struct m4_gpu_provider_runtime *p)
{
	struct m4_gpu_exec_memory *e = p->hello->ready->execution;

	lockdep_assert_held(&p->hello->gpu->lock);
	for (u32 i = 0; i < ARRAY_SIZE(p->external); i++) {
		struct m4_provider_external *x = &p->external[i];
		u64 checked;

		x->runtime = p;
		x->va = m4_provider_external_role[i].va;
		x->size = m4_provider_external_role[i].size;
		x->pages = x->size / GPU_M4_DRM_PAGE_SIZE;
		x->access = m4_provider_external_role[i].access;
		x->source = i == 0 ? &p->hello->ready->buffer[GPU_M4_UMA] :
			i == ARRAY_SIZE(p->external) - 1 ? e->pool :
			&e->buffer[m4_provider_external_role[i].id];
		x->allocation = *x->source;
		refcount_set(&x->refs, 1);
		/* The claim includes ONLY payload plus trailing guard. UMA's separate
		 * parameter page is never leased or imported into either candidate.
		 */
		x->claim = gpu_m4_drm_provider_private_reserve_locked(p->vm,
			x->va, x->size, 0, GPU_M4_DRM_PAGE_SIZE);
		if (IS_ERR(x->claim)) {
			int ret = PTR_ERR(x->claim);

			x->claim = NULL;
			return ret;
		}
		if ((u64)x->pages * sizeof(*x->pa) >
		    M4_PROVIDER_EXTERNAL_METADATA_LIMIT - p->external_bytes)
			return -ENOSPC;
		p->external_bytes += (u64)x->pages * sizeof(*x->pa);
		x->pa = kvcalloc(x->pages, sizeof(*x->pa), GFP_KERNEL);
		if (!x->pa)
			return -ENOMEM;
		if (i == ARRAY_SIZE(p->external) - 1) {
			if ((u64)x->pages * sizeof(*x->pool) >
			    M4_PROVIDER_EXTERNAL_METADATA_LIMIT - p->external_bytes)
				return -ENOSPC;
			p->external_bytes += (u64)x->pages * sizeof(*x->pool);
			x->pool = kvcalloc(x->pages, sizeof(*x->pool), GFP_KERNEL);
			if (!x->pool)
				return -ENOMEM;
			memcpy(x->pool, e->pool, x->pages * sizeof(*x->pool));
		} else if (x->allocation.size != x->size) {
			return -EPROTO;
		}
		for (u32 j = 0; j < x->pages; j++)
			x->pa[j] = x->pool ? x->pool[j].pa : x->allocation.pa +
				(u64)j * GPU_M4_DRM_PAGE_SIZE;
		for (u32 j = 0; j < x->pages; j++) {
			int ret = m4_provider_external_page(x, (u64)j * GPU_M4_DRM_PAGE_SIZE, &checked);

			if (ret)
				return ret;
		}
	}
	return 0;
}

/* The W1 tooling blob is a separate permanently retained owner. Its fixed
 * parser grants no mapping/cache authority. Preserve and independently compare
 * the original NC USC/Ioto/CDM, never rewrite those static source descriptors.
 */
static int m4_provider_wb_load(struct m4_gpu_provider_runtime *p)
{
	static const u32 resource[] = {
		GPU_M4_WB_USC_WINDOW, GPU_M4_WB_PROGRAM, GPU_M4_WB_CDM,
	};
	static const u32 buffer[] = { M4_EXEC_USC_WINDOW, M4_EXEC_IOTO, M4_EXEC_CDM };
	const struct firmware *firmware = NULL;
	struct gpu_m4_wb_fixture fixture;
	struct mutex *lock = &p->hello->gpu->lock;
	int ret;

	lockdep_assert_not_held(lock);
	if (!p->writeback_input_manifest || !p->writeback_manifest ||
	    !p->compute_manifest || p->writeback_challenge >= GPU_M4_WB_CHALLENGE_COUNT ||
	    p->wb_firmware)
		return -EPROTO;
	ret = request_firmware(&firmware, "apple/m4-j713-wb.bin", p->hello->gpu->dev);
	mutex_lock(lock);
	p->wb_firmware = firmware; /* Retain even a later parse/staging failure. */
	mutex_unlock(lock);
	if (ret)
		return ret;
	ret = gpu_m4_wb_fixture_parse(firmware->data, firmware->size,
		m4_store_hash, NULL, &fixture);
	mutex_lock(lock);
	if (!ret)
		ret = m4_store_fresh(p->hello);
	for (u32 i = 0; !ret && i < ARRAY_SIZE(resource); i++) {
		const struct gpu_m4_store_resource *src = &fixture.resource[resource[i]];
		struct m4_ready_buffer *original = &p->execution->buffer[buffer[i]];

		if (original->size != GPU_M4_DRM_PAGE_SIZE ||
		    src->size > original->size || memcmp(original->cpu, src->data, src->size) ||
		    memchr_inv(original->cpu + src->size, 0, original->size - src->size))
			ret = -ESTALE;
	}
	if (!ret)
		p->wb_fixture = fixture;
	mutex_unlock(lock);
	return ret;
}

/* Read only the independently retained compiler/source bytes, never WB data.
 * The full blob was parsed/authenticated once before native allocations.
 */
static int m4_provider_wb_profile(struct m4_gpu_provider_runtime *p)
{
	static const u32 resource[] = {
		GPU_M4_WB_USC_WINDOW, GPU_M4_WB_CONTAINER, GPU_M4_WB_PROGRAM,
	};
	struct gpu_m4_wb_fixture *f = &p->wb_fixture;
	static const u32 offset[] = { 512, 1600, 1728, 1792, 18176, 34560 };
	static const u32 size[] = { 1088, 90, 48, 16384, 16384, 16384 };

	if (!p->writeback_input_manifest || !p->writeback_manifest || !p->wb_firmware ||
	    !p->wb_firmware->data || p->wb_firmware->size != GPU_M4_WB_BUNDLE_SIZE ||
	    p->writeback_challenge >= GPU_M4_WB_CHALLENGE_COUNT ||
	    f->main_size != 70 || f->nr_gprs != 8 || f->staging_halfs || f->compiler_d ||
	    f->usc_base != M4_EXEC_USC_BASE || f->shader_entry != M4_EXEC_CONTAINER_GPU + 0x3c0 ||
	    f->input_va != GPU_M4_WB_INPUT_VA || f->output_va != M4_EXEC_OUTPUT_GPU ||
	    f->cdm_end != M4_EXEC_CDM_GPU + 48 ||
	    f->challenge[0].input != 0x13579bdf || f->challenge[0].expected_output != 0x7e7ce22a ||
	    f->challenge[0].sentinel != 0xdeadbeef || f->challenge[1].input != 0x2468ace1 ||
	    f->challenge[1].expected_output != 0x4943d514 || f->challenge[1].sentinel != 0x0badc0de)
		return -EPROTO;
	for (u32 i = 0; i < GPU_M4_WB_RESOURCE_COUNT; i++)
		if (f->resource[i].data != p->wb_firmware->data + offset[i] ||
		    f->resource[i].size != size[i])
			return -ESTALE;
	for (u32 i = 0; i < ARRAY_SIZE(p->code); i++) {
		struct m4_provider_code *c = &p->code[i];
		const struct gpu_m4_store_resource *src = &f->resource[resource[i]];

		if (!gpu_m4_compute_packet_separate(c->selected_preimage, GPU_M4_DRM_PAGE_SIZE,
		    c->map.vaddr, GPU_M4_DRM_PAGE_SIZE) ||
		    !gpu_m4_compute_packet_separate(c->selected_preimage, GPU_M4_DRM_PAGE_SIZE,
		    p->wb_output.map.vaddr, GPU_M4_DRM_PAGE_SIZE) ||
		    !gpu_m4_compute_packet_separate(c->selected_preimage, GPU_M4_DRM_PAGE_SIZE,
		    p->wb_input.map.vaddr, GPU_M4_DRM_PAGE_SIZE) ||
		    !src->data || src->gpu_va != c->va ||
		    src->size != (i == 0 ? GPU_M4_DRM_PAGE_SIZE : i == 1 ? 1088 : 90) ||
		    memcmp(c->selected_preimage, src->data, src->size) ||
		    memchr_inv(c->selected_preimage + src->size, 0,
			GPU_M4_DRM_PAGE_SIZE - src->size))
			return -ESTALE;
	}
	return 0;
}

static int m4_provider_code_stage(struct m4_gpu_provider_runtime *p)
{
	struct gpu_m4_drm_bind binds[M4_PROVIDER_CODE_COUNT] = { 0 };
	struct mutex *lock = &p->hello->gpu->lock;
	int ret;

	lockdep_assert_not_held(lock);
	/* Sources stay immutable under this claimed one-shot session. Legacy store
	 * and alias gates reject provider presence; source facts are checked again.
	 */
	for (u32 i = 0; i < ARRAY_SIZE(p->code); i++) {
		struct m4_provider_code *c = &p->code[i];
		struct drm_gem_object *gem;

		mutex_lock(lock);
		ret = m4_store_fresh(p->hello);
		c->source = p->hello->ready->execution->buffer[m4_provider_code_ids[i]];
		c->va = m4_provider_code_va[i];
		mutex_unlock(lock);
		if (ret || c->source.size != GPU_M4_DRM_PAGE_SIZE)
			return ret ? ret : -EPROTO;
		if (p->writeback_input_manifest) {
			const u32 resource[] = {
				GPU_M4_WB_USC_WINDOW, GPU_M4_WB_CONTAINER, GPU_M4_WB_PROGRAM,
			};
			const struct gpu_m4_store_resource *src = &p->wb_fixture.resource[resource[i]];
			u8 *preimage = kvcalloc(1, GPU_M4_DRM_PAGE_SIZE, GFP_KERNEL);

			mutex_lock(lock);
			c->selected_preimage = preimage;
			mutex_unlock(lock);
			if (!preimage)
				return -ENOMEM;
			memcpy(preimage, src->data, src->size);
		}
		gem = gpu_m4_drm_bo_create_native(p->frontend, GPU_M4_DRM_PAGE_SIZE,
			GPU_M4_DRM_CPU_NC, p->frontend_vm);
		if (IS_ERR(gem))
			return PTR_ERR(gem);
		mutex_lock(lock);
		c->gem = gem; /* Retain returned owner even on subsequent failure. */
		mutex_unlock(lock);
		if (!gpu_m4_drm_bo_is_native(gem) ||
		    !gpu_m4_drm_bo_belongs_to_vm(gem, p->frontend_vm) ||
		    gpu_m4_drm_bo_policy(gem) != GPU_M4_DRM_CPU_NC || gem->size != GPU_M4_DRM_PAGE_SIZE)
			return -EPERM;
		ret = gpu_m4_drm_bo_prepare(gem); /* BEFORE first NC view exists. */
		if (ret)
			return ret;
		c->prepared = true;
		ret = drm_gem_vmap(gem, &c->map);
		if (ret)
			return ret;
		c->mapped = true;
		if (iosys_map_is_null(&c->map) || c->map.is_iomem)
			return -EPERM;
		memcpy(c->map.vaddr, p->writeback_input_manifest ? c->selected_preimage :
			c->source.cpu, GPU_M4_DRM_PAGE_SIZE);
		if (memcmp(c->map.vaddr, p->writeback_input_manifest ? c->selected_preimage :
		    c->source.cpu, GPU_M4_DRM_PAGE_SIZE))
			return -EIO;
		dma_wmb(); /* No WB clean or WB CPU access after this NC upload. */
		mutex_lock(lock);
		ret = m4_store_fresh(p->hello);
		if (!ret && !m4_provider_same_buffer(&c->source,
		    &p->hello->ready->execution->buffer[m4_provider_code_ids[i]]))
			ret = -ESTALE;
		if (!ret)
			c->uploaded = true;
		mutex_unlock(lock);
		if (ret)
			return ret;
		binds[i].object = gem;
		binds[i].op.addr = c->va;
		binds[i].op.range = GPU_M4_DRM_PAGE_SIZE;
		binds[i].op.flags = DRM_ASAHI_BIND_READ;
	}
	/* These private-factory GEMs have no userspace file/handle namespace.
	 * Use the typed native entrance, not the numeric-handle ioctl adapter.
	 */
	ret = gpu_m4_drm_provider_vm_bind_native(p->vm, binds, ARRAY_SIZE(binds));
	if (ret)
		dev_info(p->hello->gpu->dev, "GPU native code bind staging failed: %d (no root exposed)\n", ret);
	for (u32 i = 0; !ret && i < ARRAY_SIZE(p->code); i++) {
		struct gpu_m4_drm_provider_object *object =
			gpu_m4_drm_provider_object_pin(p->vm, p->code[i].gem);

		if (IS_ERR(object))
			return PTR_ERR(object);
		mutex_lock(lock);
		p->code[i].object = object;
		mutex_unlock(lock);
	}
	return ret;
}

/* Fresh WB view only. Initialization and its full preimage precede any cache
 * owner preparation. This returned GEM/map/pin baseline is never dropped here,
 * even when a later detached build or root publication fails.
 */
static int m4_provider_writeback_object_stage(struct m4_gpu_provider_runtime *p,
	struct m4_provider_writeback_output *w, u64 va,
	const struct gpu_m4_store_resource *source, u32 initial)
{
	struct mutex *lock = &p->hello->gpu->lock;
	struct gpu_m4_drm_provider_object *object;
	struct drm_gem_object *gem;
	struct iosys_map map = { 0 };
	u8 *preimage;
	int ret;

	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	w->va = va;
	w->size = GPU_M4_DRM_PAGE_SIZE;
	mutex_unlock(lock);
	gem = gpu_m4_drm_bo_create_native(p->frontend, w->size,
		GPU_M4_DRM_CPU_WRITEBACK, p->frontend_vm);
	if (IS_ERR(gem))
		return PTR_ERR(gem);
	mutex_lock(lock);
	w->gem = gem;
	mutex_unlock(lock);
	if (!gpu_m4_drm_bo_is_native(gem) ||
	    !gpu_m4_drm_bo_belongs_to_vm(gem, p->frontend_vm) ||
	    gpu_m4_drm_bo_policy(gem) != GPU_M4_DRM_CPU_WRITEBACK || gem->size != w->size)
		return -EPERM;
	preimage = kvcalloc(1, w->size, GFP_KERNEL);
	mutex_lock(lock);
	w->preimage = preimage;
	mutex_unlock(lock);
	if (!preimage)
		return -ENOMEM;
	/* bo_prepare() is NC-only. Actual WB cache preparation belongs to fixture. */
	ret = drm_gem_vmap(gem, &map);
	if (ret)
		return ret;
	mutex_lock(lock);
	w->map = map;
	w->mapped = true;
	mutex_unlock(lock);
	if (iosys_map_is_null(&map) || map.is_iomem)
		return -EPERM;
	if (source)
		memcpy(map.vaddr, source->data, w->size);
	else
		memset(map.vaddr, 0, w->size);
	put_unaligned_le32(initial, map.vaddr);
	memcpy(preimage, map.vaddr, w->size);
	mutex_lock(lock);
	w->initialized = true;
	mutex_unlock(lock);
	object = gpu_m4_drm_provider_object_pin(p->vm, gem);
	if (IS_ERR(object))
		return PTR_ERR(object);
	mutex_lock(lock);
	w->object = object;
	mutex_unlock(lock);
	return 0;
}

static int m4_provider_writeback_stage(struct m4_gpu_provider_runtime *p)
{
	const struct gpu_m4_wb_challenge *c;
	int ret;

	if (!p->writeback_input_manifest)
		return m4_provider_writeback_object_stage(p, &p->wb_output,
			M4_EXEC_OUTPUT_GPU, NULL, GPU_M4_STORE_SENTINEL);
	c = &p->wb_fixture.challenge[p->writeback_challenge];
	ret = m4_provider_writeback_object_stage(p, &p->wb_output, M4_EXEC_OUTPUT_GPU,
		&p->wb_fixture.resource[GPU_M4_WB_OUTPUT], c->sentinel);
	if (!ret)
		ret = m4_provider_writeback_object_stage(p, &p->wb_input, GPU_M4_WB_INPUT_VA,
			&p->wb_fixture.resource[GPU_M4_WB_INPUT], c->input);
	return ret;
}

static struct gpu_m4_drm_provider_timestamp_config
m4_provider_ts_timestamp_config(struct m4_gpu_provider_runtime *p)
{
	return (struct gpu_m4_drm_provider_timestamp_config) {
		.frontend = p->frontend, .owner = p, .owner_lock = &p->hello->gpu->lock,
		.session_get = m4_provider_session_get, .session_put = m4_provider_session_put,
	};
}

/* This is a genuinely global native BO: NULL private_vm, one sole CPU
 * view, no lower VM_BIND and no fabricated GEM handle. Every partial owner is
 * installed before the next fallible operation and remains reachable.
 */
static int m4_provider_timestamp_stage(struct m4_gpu_provider_runtime *p)
{
	struct m4_provider_ts_storage *t = &p->timestamp;
	struct mutex *lock = &p->hello->gpu->lock;
	struct gpu_m4_drm_provider_timestamp_config config;
	struct gpu_m4_drm_provider_timestamp_cookie *cookie;
	struct gpu_m4_drm_provider_object *object;
	struct drm_gem_object *gem;
	struct iosys_map map = { 0 };
	u8 *preimage;
	int ret;

	lockdep_assert_not_held(lock);
	if (!p->timestamp_manifest || !p->compute_manifest || p->writeback_manifest || t->gem)
		return -EPROTO;
	gem = gpu_m4_drm_bo_create_native(p->frontend, GPU_M4_DRM_PAGE_SIZE,
		t->policy, NULL);
	if (IS_ERR(gem))
		return PTR_ERR(gem);
	mutex_lock(lock);
	t->gem = gem;
	t->size = GPU_M4_DRM_PAGE_SIZE;
	t->sentinel[0] = 0xdeadbeefcafef00dULL;
	t->sentinel[1] = 0x0badc0de5a5a5a5aULL;
	mutex_unlock(lock);
	/* NC backing preparation is not WB cache maintenance. The latter is
	 * performed by the actual captured job core before publication/retirement.
	 */
	if (t->policy == GPU_M4_DRM_CPU_NC) {
		ret = gpu_m4_drm_bo_prepare(gem);
		if (ret)
			return ret;
		mutex_lock(lock);
		t->prepared = true;
		mutex_unlock(lock);
	}
	preimage = kvcalloc(1, t->size, GFP_KERNEL);
	mutex_lock(lock);
	t->preimage = preimage;
	mutex_unlock(lock);
	if (!preimage)
		return -ENOMEM;
	ret = drm_gem_vmap(gem, &map);
	if (ret)
		return ret;
	mutex_lock(lock);
	t->map = map;
	t->mapped = true;
	mutex_unlock(lock);
	if (iosys_map_is_null(&map) || map.is_iomem ||
	    !gpu_m4_compute_packet_separate(preimage, t->size, map.vaddr, t->size))
		return -EPERM;
	memset(map.vaddr, 0, t->size);
	put_unaligned_le64(t->sentinel[0], map.vaddr + 8);
	put_unaligned_le64(t->sentinel[1], map.vaddr + 16);
	memcpy(preimage, map.vaddr, t->size);
	dma_wmb();
	mutex_lock(lock);
	t->initialized = true;
	t->prepared = true; /* Factory/preimage ready, not cache-prepared authority. */
	mutex_unlock(lock);
	object = gpu_m4_drm_provider_object_pin(p->vm, gem);
	if (IS_ERR(object))
		return PTR_ERR(object);
	mutex_lock(lock);
	t->object = object;
	mutex_unlock(lock);
	config = m4_provider_ts_timestamp_config(p);
	cookie = gpu_m4_drm_provider_timestamp_bind(&config, gem, 0, t->size);
	if (IS_ERR(cookie))
		return PTR_ERR(cookie);
	mutex_lock(lock);
	t->cookie = cookie;
	mutex_unlock(lock);
	if (gpu_m4_drm_provider_timestamp_policy(cookie) != t->policy ||
	    gpu_m4_drm_bo_policy(gem) != t->policy)
		return -EPERM;
	return 0;
}

static void *m4_provider_vm_create(void *cookie, struct gpu_m4_drm_vm *frontend,
				   u64 kernel_start, u64 kernel_end)
{
	struct m4_gpu_provider_runtime *p = cookie;
	struct gpu_m4_drm_provider_vm *vm;
	struct mutex *lock = &p->hello->gpu->lock;

	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	if (p->error || p->frontend_vm || !p->provider) {
		mutex_unlock(lock);
		return ERR_PTR(-EBUSY);
	}
	/* Extra frontend reference survives even a partially failed VM factory. */
	gpu_m4_drm_vm_get(frontend);
	p->frontend_vm = frontend;
	mutex_unlock(lock);
	vm = gpu_m4_drm_provider_vm_create(p->provider, frontend, kernel_start, kernel_end);
	mutex_lock(lock);
	if (!IS_ERR(vm))
		p->vm = vm;
	mutex_unlock(lock);
	return vm;
}

static int m4_provider_vm_destroy(void *cookie, void *vm)
{
	/* No qualified teardown exists, even for a root-only completed trial. */
	return -EBUSY;
}

static void m4_provider_vm_close(void *cookie, void *vm)
{
	struct m4_gpu_provider_runtime *p = cookie;
	struct mutex *lock = &p->hello->gpu->lock;

	mutex_lock(lock);
	if (!p->error)
		p->error = -ESHUTDOWN;
	if (p->provider)
		gpu_m4_drm_provider_fault_locked(p->provider, p->error);
	p->hello->ready->error = p->error;
	mutex_unlock(lock);
}

static bool m4_provider_leaf_equal(const struct gpu_m4_drm_vm_leaf *leaf,
	u64 va, u64 pa, const void *identity, u64 offset, u32 access)
{
	return leaf->va == va && leaf->pa == pa && leaf->identity == identity &&
		leaf->object_offset == offset && leaf->access == access &&
		leaf->cpu_policy == GPU_M4_DRM_CPU_NC;
}

static int m4_provider_fixture_page(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_version *v, u64 va, u32 access)
{
	struct gpu_m4_drm_vm_leaf leaf;
	u64 pa;
	int ret = gpu_m4_drm_provider_version_page_locked(v, va, access, &leaf);

	if (ret)
		return ret;
	if (va >= GPU_M4_DRM_UAT_CONTEXT_START &&
	    va - GPU_M4_DRM_UAT_CONTEXT_START < GPU_M4_DRM_UAT_CONTEXT_SIZE) {
		u64 offset = va - GPU_M4_DRM_UAT_CONTEXT_START;
		u32 index = offset / GPU_M4_DRM_PAGE_SIZE;

		ret = m4_provider_validate_page(p, p->frontend_vm, p->context_identity,
			offset, p->context_pa[index]);
		return ret ? ret : m4_provider_leaf_equal(&leaf, va, p->context_pa[index],
			p->context_identity, offset, DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE) ?
			0 : -ESTALE;
	}
	if (p->writeback_manifest && (va == M4_EXEC_OUTPUT_GPU ||
	    (p->writeback_input_manifest && va == GPU_M4_WB_INPUT_VA))) {
		bool input = va == GPU_M4_WB_INPUT_VA;
		struct m4_provider_writeback_output *w = input ? &p->wb_input : &p->wb_output;
		u64 original_pa;

		/* The displaced NC allocation/guard claim remain separately retained.
		 * Only the selected leaf changes; both graphs name this actual WB GEM.
		 */
		ret = input ? gpu_m4_drm_provider_private_check_locked(p->wb_input_claim,
			p->vm, GPU_M4_WB_INPUT_VA, GPU_M4_DRM_PAGE_SIZE) :
			m4_provider_external_page(&p->external[5], 0, &original_pa);
		if (ret)
			return ret;
		if (!w->gem || !w->object ||
		    gpu_m4_drm_provider_object_identity(w->object) != w->gem)
			return -ESTALE;
		ret = gpu_m4_drm_provider_object_page_locked(w->object, 0, &pa);
		return ret ? ret : pa == w->pa && (input || pa != original_pa) &&
			leaf.va == w->va && leaf.pa == pa && leaf.identity == w->gem &&
			!leaf.object_offset && leaf.cpu_policy == GPU_M4_DRM_CPU_WRITEBACK &&
			leaf.access == (DRM_ASAHI_BIND_READ | (input ? 0 : DRM_ASAHI_BIND_WRITE)) ?
			0 : -ESTALE;
	}
	for (u32 i = 0; i < ARRAY_SIZE(p->external); i++) {
		struct m4_provider_external *x = &p->external[i];
		u64 offset;

		if (va < x->va || va - x->va >= x->size)
			continue;
		offset = va - x->va;
		ret = m4_provider_external_page(x, offset, &pa);
		return ret ? ret : m4_provider_leaf_equal(&leaf, va, pa, x->source,
			offset, x->access) ? 0 : -ESTALE;
	}
	for (u32 i = 0; i < ARRAY_SIZE(p->code); i++) {
		struct m4_provider_code *c = &p->code[i];

		if (va != c->va)
			continue;
		if (!c->gem || !c->object || !c->mapped || !c->uploaded ||
		    c->va != m4_provider_code_va[i] ||
		    !m4_provider_same_buffer(&c->source,
			&p->hello->ready->execution->buffer[m4_provider_code_ids[i]]) ||
		    c->map.is_iomem || iosys_map_is_null(&c->map) ||
		    gpu_m4_drm_provider_object_identity(c->object) != c->gem ||
		    memcmp(c->map.vaddr, p->writeback_input_manifest ? c->selected_preimage :
			c->source.cpu, GPU_M4_DRM_PAGE_SIZE))
			return -ESTALE;
		ret = gpu_m4_drm_provider_object_page_locked(c->object, 0, &pa);
		return ret ? ret : pa == c->pa && m4_provider_leaf_equal(&leaf, va,
			pa, c->gem, 0, DRM_ASAHI_BIND_READ) ? 0 : -ESTALE;
	}
	return -EPERM;
}

/* TTBAT lower word of user context asid: the graph's ASID-free root
 * descriptor (root PA | valid) with the ASID in bits 63:48. The upper word of
 * a user context is always zero on this SoC (contract Part II).
 */
static u64 m4_provider_ttbat_low(u64 root, u32 asid)
{
	return root ? root | (u64)asid << 48 : 0;
}

/* The ASID-free root descriptor of a TTBAT lower word installed for asid; 0
 * stays 0 (unbound) and a word naming any other ASID has no valid root.
 */
static u64 m4_provider_ttbat_root(u64 low, u32 asid)
{
	if (!low)
		return 0;
	return low >> 48 == asid ? low & ((1ULL << 48) - 1) : U64_MAX;
}

static int m4_provider_graph(struct m4_gpu_provider_runtime *p, u32 index)
{
	struct gpu_m4_drm_provider_version *v = p->version[index];
	u64 low, high;
	int ret = gpu_m4_drm_provider_version_refresh_locked(v);

	if (!ret)
		ret = gpu_m4_drm_provider_version_roots_locked(v, &low, &high);
	if (!ret && (m4_provider_ttbat_low(low, 1) != p->root[index] || high ||
	    !(low & GPU_M4_PA_MASK) || (low & ~GPU_M4_PA_MASK) != 1))
		ret = -EPROTO;
	for (u32 i = 0; !ret && i < ARRAY_SIZE(p->context); i++) {
		struct gpu_m4_drm_vm_leaf leaf;

		ret = gpu_m4_drm_provider_version_page_locked(v,
			GPU_M4_DRM_UAT_CONTEXT_START + (u64)i * GPU_M4_DRM_PAGE_SIZE,
			DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE, &leaf);
		if (!ret && (leaf.va != p->context[i].va || leaf.pa != p->context[i].pa ||
		    leaf.identity != p->context[i].identity ||
		    leaf.object_offset != p->context[i].object_offset ||
		    leaf.access != p->context[i].access || leaf.cpu_policy != p->context[i].cpu_policy))
			ret = -ESTALE;
	}
	if (!ret && p->compute_manifest) {
		struct gpu_m4_drm_vm_leaf unused;

		if (!p->manifest_ready)
			return -EPROTO;
		for (u32 i = 0; !ret && i < ARRAY_SIZE(p->external); i++) {
			struct m4_provider_external *x = &p->external[i];

			for (u32 j = 0; !ret && j < x->pages; j++)
				ret = m4_provider_fixture_page(p, v,
					x->va + (u64)j * GPU_M4_DRM_PAGE_SIZE, x->access);
			if (!ret && gpu_m4_drm_provider_version_page_locked(v,
			    x->va + x->size, DRM_ASAHI_BIND_READ, &unused) != -ENOENT)
				ret = -EPROTO;
		}
		for (u32 i = 0; !ret && i < ARRAY_SIZE(p->code); i++) {
			ret = m4_provider_fixture_page(p, v, p->code[i].va, DRM_ASAHI_BIND_READ);
			if (!ret && gpu_m4_drm_provider_version_page_locked(v,
			    p->code[i].va + GPU_M4_DRM_PAGE_SIZE,
			    DRM_ASAHI_BIND_READ, &unused) != -ENOENT)
				ret = -EPROTO;
		}
		for (u32 i = 2; !ret && i <= 3; i++)
			if (gpu_m4_drm_provider_version_page_locked(v,
			    GPU_M4_BOOTSTRAP_LOWER + (u64)i * GPU_M4_DRM_PAGE_SIZE,
			    DRM_ASAHI_BIND_READ, &unused) != -ENOENT)
				ret = -EPROTO;
		if (!ret && p->writeback_input_manifest)
			ret = m4_provider_fixture_page(p, v, GPU_M4_WB_INPUT_VA, DRM_ASAHI_BIND_READ);
		if (!ret && p->writeback_input_manifest &&
		    (gpu_m4_drm_provider_version_page_locked(v, GPU_M4_WB_INPUT_VA -
		     GPU_M4_DRM_PAGE_SIZE, DRM_ASAHI_BIND_READ, &unused) != -ENOENT ||
		     gpu_m4_drm_provider_version_page_locked(v, GPU_M4_WB_INPUT_VA +
		     GPU_M4_DRM_PAGE_SIZE, DRM_ASAHI_BIND_READ, &unused) != -ENOENT))
			ret = -EPROTO;
	}
	return ret;
}

/* Independent of mutable work/output/notifier contents. The source code pages
 * are immutable RO preimages, while original DMA table/alias health stays exact.
 * This does not publish work or qualify the new native physical code backing.
 */
static int m4_provider_manifest_valid(struct m4_gpu_provider_runtime *p)
{
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!p->compute_manifest || !p->manifest_ready || p->error ||
	    p->hello->ready != p->ready || p->ready->execution != p->execution ||
	    !p->provider || gpu_m4_drm_provider_error_locked(p->provider) ||
	    !m4_exec_memory_valid(p->hello))
		return -EPROTO;
	if (p->writeback_input_manifest) {
		ret = m4_provider_wb_profile(p);
		if (ret)
			return ret;
	}
	ret = m4_provider_native_coverage(p);
	if (ret)
		return ret;
	ret = m4_provider_graph(p, 0);
	return ret ? ret : m4_provider_graph(p, 1);
}

/* Pure encoder oracle: EXACT captured resident V1, never execution.vm/static
 * graph or a later active VM lookup. Owner proof refreshes full graphs before
 * preparation/publication; this bounded walk checks the immutable leaf ledger.
 */
static int __maybe_unused m4_provider_range(void *cookie, u64 va, u64 size, bool write)
{
	struct m4_gpu_provider_runtime *p = cookie;
	u32 access = DRM_ASAHI_BIND_READ | (write ? DRM_ASAHI_BIND_WRITE : 0);
	u64 end;

	if (!p)
		return -EPERM;
	lockdep_assert_held(&p->hello->gpu->lock);
	if (!p->compute_manifest || !p->manifest_ready || p->error || !p->selected ||
	    gpu_m4_drm_provider_error_locked(p->provider) ||
	    p->generation != 2 || p->resident != p->version[1] ||
	    gpu_m4_drm_provider_resident_locked(p->provider) != p->version[1] ||
	    gpu_m4_drm_provider_binding_generation_locked(p->provider) != 2 ||
	    readq(p->hello->gpu->ttbs + 0x10) != p->root[1] ||
	    readq(p->hello->gpu->ttbs + 0x18) || !size ||
	    va < GPU_M4_DRM_PAGE_SIZE || va >= BIT_ULL(42) || size > BIT_ULL(42) - va)
		return -EPERM;
	end = va + size;
	for (u64 page = va & ~(GPU_M4_DRM_PAGE_SIZE - 1); page < end;
	     page += GPU_M4_DRM_PAGE_SIZE) {
		int ret = m4_provider_fixture_page(p, p->version[1], page, access);

		if (ret)
			return ret;
	}
	return 0;
}

static void m4_provider_fault_locked(struct m4_gpu_provider_runtime *p, int error)
{
	lockdep_assert_held(&p->hello->gpu->lock);
	if (!p->error)
		p->error = error < 0 ? error : -EIO;
	p->complete = false;
	p->hello->ready->error = p->error;
	if (p->provider)
		gpu_m4_drm_provider_fault_locked(p->provider, p->error);
	if (p->frontend_vm)
		gpu_m4_drm_vm_error(p->frontend_vm, p->error);
}

static int m4_provider_health(struct m4_gpu_hello *h)
{
	struct m4_gpu_provider_runtime *p = h->ready->provider_runtime;
	int ret = 0;

	lockdep_assert_held(&h->gpu->lock);
	if (!p || p->hello != h || h->ready != p->ready ||
	    h->ready->execution != p->execution || !p->attempted ||
	    p->error || !refcount_read(&p->session_refs))
		return p && p->error ? p->error : -EPROTO;
	if (p->provider)
		ret = gpu_m4_drm_provider_error_locked(p->provider);
	if (!ret && p->ordinary)
		return m4_user_health_locked(p);
	if (!ret && p->prepared && p->compute_manifest)
		ret = m4_provider_manifest_valid(p);
	else if (!ret && p->prepared) {
		ret = m4_provider_graph(p, 0);
		if (!ret)
			ret = m4_provider_graph(p, 1);
	}
	if (!ret && p->work)
		ret = m4_provider_work_validate_locked(p);
	if (!ret && p->ts_job && p->ts_job->work)
		ret = m4_provider_ts_work_health_locked(p);
	if (ret)
		m4_provider_fault_locked(p, ret);
	return ret;
}

/* Whether the owner's TTBAT image of slots 2..127 is valid at all, from host
 * owner state alone. The ordinary owner's image is its user_asid[] table,
 * which must mirror the provider core's per-ASID bindings exactly; the fixed
 * diagnostic owner binds only ASID 1 (slots 2/3). Constant while the GPU
 * mutex is held and the owner state unchanged, so one READY health pass
 * decides it once.
 */
static bool m4_provider_bound_image(struct m4_gpu_hello *h)
{
	struct m4_gpu_provider_runtime *p = h->ready->provider_runtime;

	lockdep_assert_held(&h->gpu->lock);
	if (!p || p->hello != h || p->error || p->writing)
		return false;
	if (p->ordinary)
		return p->provider && !gpu_m4_drm_provider_error_locked(p->provider) &&
			!p->selected && !p->resident && !p->generation &&
			!m4_user_binding_check_locked(p);
	if (!p->selected)
		return !p->generation && !p->resident &&
			(!p->provider || (!gpu_m4_drm_provider_resident_locked(p->provider) &&
			 !gpu_m4_drm_provider_binding_generation_locked(p->provider)));
	return p->provider && p->resident && p->generation && p->generation <= 2 &&
		p->resident == p->version[p->generation - 1] && !p->resident_high &&
		p->resident_low == p->root[p->generation - 1] &&
		!gpu_m4_drm_provider_error_locked(p->provider) &&
		gpu_m4_drm_provider_resident_locked(p->provider) == p->resident &&
		gpu_m4_drm_provider_binding_generation_locked(p->provider) == p->generation;
}

/* The word TTBAT slot index (2..127) must hold in a valid image: user
 * context index / 2's lower word, or its upper word, always zero.
 */
static u64 m4_provider_bound_word(struct m4_gpu_hello *h, u32 index)
{
	struct m4_gpu_provider_runtime *p = h->ready->provider_runtime;

	if (p->ordinary)
		return index & 1 ? 0 : p->user_asid[index / 2].low;
	if (!p->selected)
		return 0;
	return index == 2 ? p->resident_low : index == 3 ? p->resident_high : 0;
}

static int m4_provider_engines_idle(struct m4_gpu_hello *h)
{
	struct m4_gpu_ready *r = h->ready;
	struct m4_gpu_exec_memory *e = r->execution;
	struct m4_gpu_provider_runtime *p = r->provider_runtime;

	if (!e->barrier_complete || !m4_exec_context_registered(e))
		return -EPROTO;
	for (u32 engine = 0; engine < 3; engine++) {
		struct m4_ready_buffer *q = e->queue[engine];
		void *state = q[M4_EXEC_QUEUE_STATE].cpu, *info = q[M4_EXEC_QUEUE_INFO].cpu;
		void *pipe = r->buffer[GPU_M4_OBJECT_COUNT + engine * 2].cpu;
		u32 expected = engine ? 0 : 1; /* The already retired queue-0 registration. */
		u32 pipe_expected = expected;

		/* Ordinary jobs reuse the retained queues. At the single global lane's
		 * bind boundary, all consumers must match the actual producer, including
		 * after ring wrap. Never reset them to first-job positions.
		 */
		if (p && p->ordinary) {
			expected = m4_ready_read32(state + 0x40);
			pipe_expected = m4_ready_read32(pipe + 0x20);
		}
		if (expected >= 256 || pipe_expected >= 256)
			return -EPROTO;
		if (m4_ready_read32(state + 0x30) != expected ||
		    m4_ready_read32(state + 0x40) != expected ||
		    m4_ready_read32(state + 0x50) != 256 || m4_ready_read32(info + 0x80) ||
		    m4_ready_read32(pipe) != pipe_expected ||
		    m4_ready_read32(pipe + 0x10) != pipe_expected ||
		    m4_ready_read32(pipe + 0x20) != pipe_expected)
			return -EPROTO;
		for (u32 i = 0; i < 3; i++)
			if (m4_ready_read32(info + 0x20 + i * 4) != expected)
				return -EPROTO;
	}
	return 0;
}

/* Fixed live observations ONLY inside handoff; no graph walk/cache refresh,
 * and no engine-idle requirement: other contexts' work may run. Every user
 * TTBAT slot holds the bound image, except context asid (when nonzero), which
 * must hold exactly old_low/old_high.
 */
static int m4_provider_fixed_live(struct m4_gpu_hello *h, u32 asid, u64 old_low, u64 old_high)
{
	struct m4_gpu_power *gpu = h->gpu;
	struct m4_gpu_ready *r = h->ready;
	struct m4_gpu_store *s = r->execution->store;

	if (gpu->error || h->error || r->error || atomic_read(&h->callback_error) ||
	    !apple_rtkit_is_running(h->rtkit) || apple_rtkit_is_crashed(h->rtkit) ||
	    !(readl(gpu->asc + 0x44) & BIT(4)) ||
	    m4_pstate_sample(h, M4_PSTATE_FIXED) > r->pstate.configured ||
	    m4_ready_read32(r->input.object[GPU_M4_FIRMWARE_DATA] + 0x4fb0) ||
	    m4_store_control(h) || s->attempted || s->binding_attempted || s->bound ||
	    s->published || s->upper_attempted ||
	    readq(gpu->ttbs) != (page_to_phys(h->lower) | 1) ||
	    readq(gpu->ttbs + 8) != (h->region[M4_TABLES].start | 1))
		return -EPROTO;
	for (u32 i = 2; i < 128; i++) {
		u64 expected = i / 2 != asid ? m4_provider_bound_word(h, i) :
			i & 1 ? old_high : old_low;

		if (readq(gpu->ttbs + i * 8) != expected)
			return -ESTALE;
	}
	return 0;
}

/* The fixed observations with every engine idle (fixtures, ASID 1). */
static int m4_provider_fixed(struct m4_gpu_hello *h, u64 old_low, u64 old_high)
{
	return m4_provider_engines_idle(h) ? -EPROTO :
		m4_provider_fixed_live(h, 1, old_low, old_high);
}

/* Completes the stores to user context asid's TTBAT entry, then invalidates
 * that ASID's non-global translations: ASIDE1OS operand (u64)asid << 48,
 * never a TTBAT word, root PA or slot number.
 */
static void m4_provider_asid_sync(u32 asid)
{
	asm volatile("dsb osh\n\t.arch armv8.4-a\n\ttlbi aside1os, %0\n\tdsb sy"
		: : "r" ((u64)asid << 48) : "memory");
}

/* REMOVE-DEBUG: READY health of the ordinary lane worker's holds: live passes,
 * same-hold reuses and reuses refused by their bounded live observations
 * (m4_provider_ready_health()), under the GPU mutex.
 */
static struct {
	u64 live, reused, live_ns, refused;
} m4_provider_health_stats;

/* Drop the same-hold READY health pass: any host store to state it covers. */
static void m4_provider_health_void(struct m4_gpu_provider_runtime *p)
{
	if (p)
		p->health_hold = 0;
}

/* What a same-hold reuse still observes live, every time: latched owner,
 * READY and callback errors, RTKit, ASC RUN, the actual SGX state against
 * the configured S after any due thermal service (m4_pstate_sample()), the
 * bootstrap TTBAT[0/1] pair, every bound context's core binding and the
 * actual TTBAT pair of each bound context. The GPU mutex freezes host owner
 * state, not firmware, MMIO or the thermal sampling clock.
 */
static int m4_provider_ready_live(struct m4_gpu_provider_runtime *p)
{
	struct m4_gpu_power *gpu = p->hello->gpu;
	int ret = m4_user_live_health_locked(p);

	if (!ret)
		ret = m4_user_binding_check_locked(p);
	if (!ret && !m4_provider_bound_image(p->hello))
		ret = -EPROTO;
	for (u32 asid = 1; !ret && asid < M4_USER_ASIDS; asid++)
		if (p->user_asid[asid].low &&
		    (readq(gpu->ttbs + 16 * asid) != p->user_asid[asid].low ||
		     readq(gpu->ttbs + 16 * asid + 8)))
			ret = -EPROTO;
	return ret;
}

/* READY health (m4_ready_health()) for the provider owner. The ordinary lane
 * worker holds this mutex continuously from its lock to its unlock, its hold
 * token constant and odd meanwhile (gpu_m4_drm_provider_lane_hold_locked()).
 * One live pass therefore serves every later start/bind/publish/poll check of
 * that SAME hold for its graph and inventory proofs (fixed READY, execution
 * and upper tables, edges, arena, resident refreshes, unbound TTBAT slots)
 * while each host-owned input it covered is unchanged: the upper owner
 * revision (every lease, table and expected-ledger store, see
 * gpu_m4_drm_upper_revision_locked()) and the resident bindings (TTBAT image,
 * generations: user_binding_revision); upper commits and binding stores also
 * void it explicitly.
 * Every reuse still makes the bounded live observations of
 * m4_provider_ready_live(), so publication never runs on a stale RUN, SGX,
 * thermal or TTBAT sample. Any failure there voids the reuse and is returned
 * as is: the complete live pass then runs only for its attribution and
 * faults, never replacing that error by success or remembering a pass. Never
 * reused across an unlock; every other holder (TDR service, READY drain and
 * its once-per-second full scan, store/exec/fixture paths) runs live.
 */
static int m4_provider_ready_health(struct m4_gpu_hello *h)
{
	struct m4_gpu_provider_runtime *p = h->ready->provider_runtime;
	u64 hold = 0, revision = 0, start;
	int ret, live = 0;

	if (p && p->ordinary && p->user_lane) {
		hold = gpu_m4_drm_provider_lane_hold_locked(p->user_lane);
		revision = gpu_m4_drm_upper_revision_locked(h->ready->upper);
	}
	if (hold && p->health_hold == hold && p->health_revision == revision &&
	    p->health_generation == p->generation && p->health_resident == p->resident &&
	    p->health_binding == p->user_binding_revision) {
		live = m4_provider_ready_live(p);
		if (!live) {
			m4_provider_health_stats.reused++;
			return 0;
		}
		m4_provider_health_stats.refused++;
	}
	m4_provider_health_void(p);
	start = hold ? ktime_get_ns() : 0;
	ret = m4_ready_health(h);
	if (!hold)
		return ret;
	m4_provider_health_stats.live++;
	m4_provider_health_stats.live_ns += ktime_get_ns() - start;
	/* A refused reuse's live failure stands, whatever the pass found. */
	if (live)
		return live;
	if (!ret) {
		p->health_hold = hold;
		p->health_revision = revision;
		p->health_generation = p->generation;
		p->health_resident = p->resident;
		p->health_binding = p->user_binding_revision;
	}
	return ret;
}

static int m4_provider_idle(struct m4_gpu_hello *h)
{
	int ret = m4_ready_drain(h);

	if (!ret)
		ret = m4_provider_ready_health(h);
	if (!ret)
		ret = m4_store_control(h);
	/* m4_ready_health() just ran the retained fixture's live check (every
	 * client table byte, READY edges, selected alias) over its once-admitted
	 * allocation inventory. Only its presence is required here; a second
	 * pass would re-read the same tables in this locked scope.
	 */
	if (!ret && !h->ready->execution)
		ret = -EPROTO;
	if (!ret && !h->ready->provider_runtime->ordinary)
		ret = m4_store_fresh(h);
	if (!ret)
		ret = m4_provider_engines_idle(h);
	return ret;
}

/* The fixed observations a binding store into asid (0: none) requires: the
 * exact table, context asid holding old_low/old_high. The fixed diagnostic
 * owner also requires every engine idle. The ordinary owner's authority is
 * per ASID instead (that binding's quiescence, checked by the core): another
 * context's work may run meanwhile (contract Part II, "Idle gates").
 */
static int m4_provider_fixed_at(struct m4_gpu_hello *h, u32 asid, u64 old_low, u64 old_high)
{
	struct m4_gpu_provider_runtime *p = h->ready->provider_runtime;

	if ((!p || !p->ordinary) && m4_provider_engines_idle(h))
		return -EPROTO;
	return m4_provider_fixed_live(h, asid, old_low, old_high);
}

/* Installs version's root into user context asid: TTBAT words low (root PA |
 * valid | asid << 48) and high (zero). Core begin pins the old and new
 * graphs; under handoff both words are stored and read back; after its
 * release the stores are ordered, ASIDE1OS invalidates exactly that ASID and
 * DSB SY completes it; only then does the core complete the binding and
 * advance that ASID's generation. The fixed diagnostic owner binds only
 * ASID 1 and mirrors it in resident/resident_low/generation; the ordinary
 * owner mirrors it in user_asid[asid]. Any failure is fail-stop for the
 * caller's fault latch.
 */
static int m4_provider_install_binding(struct m4_gpu_provider_runtime *p, u32 asid,
	struct gpu_m4_drm_provider_version *version, u64 low, u64 high)
{
	struct m4_gpu_hello *h = p->hello;
	struct m4_user_asid *a = &p->user_asid[asid];
	u64 old_low = p->ordinary ? a->low : p->resident_low;
	u64 old_high = p->ordinary ? 0 : p->resident_high;
	u64 observed_low, observed_high;
	int ret, release;

	if (asid < 1 || asid >= M4_USER_ASIDS || (!p->ordinary && asid != 1) ||
	    m4_provider_ttbat_root(low, asid) == U64_MAX || !(low >> 48) || high)
		return -EINVAL;
	/* Ordinary: the complete READY pass (memoized per lane hold), no
	 * global engine-idle gate; begin refuses a working resident here.
	 */
	ret = p->ordinary ? m4_provider_ready_health(h) : m4_provider_idle(h);
	if (!ret)
		ret = m4_provider_fixed_at(h, asid, old_low, old_high);
	if (!ret && !p->ordinary &&
	    gpu_m4_drm_provider_binding_generation_locked(p->provider) >= U8_MAX)
		ret = -EOVERFLOW;
	if (!ret)
		ret = gpu_m4_drm_provider_asid_bind_begin_locked(p->provider, asid, version,
			m4_provider_ttbat_root(old_low, asid), old_high);
	if (ret)
		return ret;
	p->handoff_get = m4_hello_lock(h);
	if (p->handoff_get)
		return p->handoff_get;
	ret = m4_provider_fixed_at(h, asid, old_low, old_high);
	if (!ret) {
		p->writing = true;
		m4_provider_health_void(p);
		writeq(low, h->gpu->ttbs + 16 * asid);
		writeq(high, h->gpu->ttbs + 16 * asid + 8);
		if (readq(h->gpu->ttbs + 16 * asid) != low ||
		    readq(h->gpu->ttbs + 16 * asid + 8) != high)
			ret = -EIO;
	}
	m4_hello_unlock(h);
	release = readb(h->handoff + 0x10) ? -EIO : 0;
	p->handoff_put = release;
	if (!ret)
		ret = release;
	if (ret)
		return ret;
	m4_provider_asid_sync(asid);
	p->tlbi = 0;
	observed_low = readq(h->gpu->ttbs + 16 * asid);
	observed_high = readq(h->gpu->ttbs + 16 * asid + 8);
	/* The core refreshes both exact old/new graphs again at completion before
	 * releasing the former resident. Fixed fixture manifests are not authority
	 * for an ordinary captured user version. An ordinary refresh consumes the
	 * immutable graph's construction proof and rereads only its live root page:
	 * this binding changed the selected graph, not any table contents. Live
	 * TTBAT readback, binding epoch and fault-boundary rescans remain.
	 */
	ret = p->ordinary ? gpu_m4_drm_provider_version_refresh_locked(version) :
		m4_provider_graph(p, 0);
	if (!ret && !p->ordinary)
		ret = m4_provider_graph(p, 1);
	if (!ret)
		ret = m4_provider_fixed_at(h, asid, low, high);
	if (!ret && (observed_low != low || observed_high != high))
		ret = -EIO;
	if (!ret)
		ret = gpu_m4_drm_provider_asid_bind_complete_locked(p->provider, asid, version,
			m4_provider_ttbat_root(observed_low, asid), observed_high);
	if (ret)
		return ret;
	p->writing = false;
	if (p->ordinary) {
		a->resident = version;
		a->low = observed_low;
		a->generation = gpu_m4_drm_provider_asid_generation_locked(p->provider, asid);
		p->user_binding_revision++;
		return 0; /* Publication runs its own READY pass next. */
	}
	p->resident = version;
	p->resident_low = observed_low;
	p->resident_high = observed_high;
	p->generation = gpu_m4_drm_provider_binding_generation_locked(p->provider);
	p->selected = true;
	return m4_provider_idle(h);
}

static int m4_provider_bind(struct m4_gpu_provider_runtime *p, u32 index)
{
	return m4_provider_install_binding(p, 1, p->version[index], p->root[index], 0);
}

/* The installed TTBAT pair of user context asid, read live. */
static int m4_provider_installed(struct m4_gpu_provider_runtime *p, u32 asid, u64 low)
{
	return low && readq(p->hello->gpu->ttbs + 16 * asid) == low &&
		!readq(p->hello->gpu->ttbs + 16 * asid + 8) ? 0 : -EIO;
}

/*
 * Exact unchanged-image adoption in user context asid: candidate's separate
 * UAT wrapper names the very same retained root/node image and complete
 * user/private tuples as that context's resident (checked by the core). No
 * table or translation changes, so there is no TTBAT store, handoff, TLBI,
 * engine-idle check or binding generation change: work holding the same
 * actual binding shares one generation (co-residency contract section 1).
 * The installed pair is read live; the core then moves the resident owner in
 * one step, uncertainty pinned first. Any failure is handled by the caller's
 * fault latch.
 */
static int m4_provider_adopt_binding(struct m4_gpu_provider_runtime *p, u32 asid,
	struct gpu_m4_drm_provider_version *version, u64 low, u64 high)
{
	struct m4_user_asid *a = &p->user_asid[asid];
	int ret;

	if (!p->ordinary || asid < 1 || asid >= M4_USER_ASIDS || !a->resident ||
	    !a->low || low != a->low || high)
		return -ESTALE;
	ret = m4_provider_installed(p, asid, low);
	if (ret)
		return ret;
	m4_provider_health_void(p); /* Resident owner transfer begins. */
	ret = gpu_m4_drm_provider_asid_adopt_locked(p->provider, asid, version,
		m4_provider_ttbat_root(low, asid), high);
	if (ret)
		return ret;
	a->resident = version;
	p->user_binding_revision++;
	return gpu_m4_drm_provider_asid_generation_locked(p->provider, asid) == a->generation ?
		0 : -EPROTO;
}

/* REMOVE-DEBUG: successful ordinary binds by kind, under the GPU mutex:
 * [0] candidate already resident, [1] same-image adoption, [2] root install.
 */
static u64 m4_user_bind_stats[3];

/* The actual job owns candidate, not a later lookup of the client's active VM.
 * Called only by the exclusive global publication lane before begin_work.
 * Binding authority: vm leases asid and is its dedicated owner, or asid is
 * the shared ASID 1, so a dedicated context's root only ever changes between
 * graphs of the one VM whose notifiers and retained owners name it.
 */
static int __maybe_unused m4_user_bind_locked(struct m4_gpu_provider_runtime *p, u32 asid,
	const struct m4_user_vm *vm, struct gpu_m4_drm_provider_version *candidate,
	u64 *epoch, u8 *wire)
{
	struct m4_user_asid *a;
	u64 low, high;
	u32 kind = 0;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!p->ordinary || !candidate || !epoch || !wire || !vm || p->error)
		return p->error ?: -EINVAL;
	a = &p->user_asid[asid < M4_USER_ASIDS ? asid : 0];
	if (!asid || asid >= M4_USER_ASIDS || !a->leases || a->quarantined ||
	    a->owner != (asid == M4_USER_ASID_SHARED ? NULL : vm)) {
		ret = -EPERM;
	} else if (a->resident == candidate) {
		/* Resident fast path: only the context's installed pair, live. */
		ret = m4_provider_installed(p, asid, a->low);
	} else {
		ret = gpu_m4_drm_provider_version_roots_locked(candidate, &low, &high);
		low = m4_provider_ttbat_low(low, asid);
		/* Identical physical image: transfer the resident owner only. A
		 * different root or any changed path takes the actual rebind.
		 */
		kind = 2;
		if (!ret && a->resident &&
		    gpu_m4_drm_provider_version_same_image_locked(p->provider, a->resident,
								  candidate)) {
			kind = 1;
			ret = m4_provider_adopt_binding(p, asid, candidate, low, high);
		} else if (!ret) {
			ret = m4_provider_install_binding(p, asid, candidate, low, high);
		}
	}
	if (!ret)
		ret = gpu_m4_drm_provider_asid_binding_locked(p->provider, asid, candidate,
			epoch, wire);
	if (!ret && *epoch != a->generation)
		ret = -ESTALE;
	if (!ret) {
		m4_user_bind_stats[kind]++;
		a->binds[kind]++;
	}
	if (ret)
		m4_provider_fault_locked(p, ret);
	return ret;
}

/* Expensive factories run outside the hardware mutex. Their callbacks use the
 * exact same hardware mutex, never a substitute provider lock or reservation.
 */
static int m4_provider_stage(struct m4_gpu_provider_runtime *p)
{
	static const struct gpu_m4_drm_ops ops = {
		.session_get = m4_provider_session_get, .session_put = m4_provider_session_put,
		.vm_create = m4_provider_vm_create, .vm_destroy = m4_provider_vm_destroy,
		.vm_close = m4_provider_vm_close,
	};
	struct m4_gpu_power *gpu = p->hello->gpu;
	struct gpu_m4_drm_provider_config config = {
		.session = p, .get = m4_provider_session_get, .put = m4_provider_session_put,
		.lock = &gpu->lock, .validate_lock = m4_provider_validate_lock,
		.validate_page = m4_provider_validate_page,
		.admitted_page = m4_provider_admitted_page,
		/* Parked early first windows drop their children on an error. */
		.error_wake_locked = gpu_m4_drm_provider_job_error_wake_locked,
		/* Ordinary binds commit logical mappings; acceptance captures their
		 * exact snapshot and native staging builds the final job graph.
		 * Detached qualification/bootstrap paths remain eagerly materialized.
		 */
		.defer_user_materialization = p->ordinary,
		.vm_start = 0x4000, .vm_end = (1ULL << 42) - 0x8000,
		.memory_limit = p->ordinary ? M4_ORDINARY_PROVIDER_MEMORY_LIMIT : M4_PROVIDER_MEMORY_LIMIT,
		/* A BO close or baseline removal at a full budget may replace its
		 * owner within 1/16 of the limit outstanding provider-wide.
		 */
		.removal_reserve = (p->ordinary ? M4_ORDINARY_PROVIDER_MEMORY_LIMIT :
			M4_PROVIDER_MEMORY_LIMIT) / 16,
		.max_leaves = p->ordinary ? M4_ORDINARY_PROVIDER_MAX_LEAVES :
			(p->compute_manifest ? 2082 + p->writeback_input_manifest : 16),
		.max_tables = p->ordinary ? GPU_M4_DRM_UAT_MAX_TABLES : 16,
	};
	/* Factory validation metadata, and the ordinary node's GET_PARAMS: never
	 * used to select/change a frequency or claim a measured maximum.
	 */
	struct drm_asahi_params_global params = {
		.gpu_generation = 16, .gpu_variant = 'G', .gpu_revision = 0x10,
		.chip_id = 0x8132, .num_dies = 1, .num_clusters_total = 1,
		.num_cores_per_cluster = 10, .core_masks = { gpu->cores },
		.max_frequency_khz = 1470000, .vm_start = 0x4000, .vm_end = (1ULL << 42) - 0x8000,
		.vm_kernel_min_size = 0x20000000,
		.max_commands_per_submission = p->ordinary ? GPU_M4_DRM_JOB_MAX_COMMANDS : 1,
		.max_attachments = 16, .command_timestamp_frequency_hz = 1000000000,
		.usc_generation = 3, .gpu_hal_generation = DRM_ASAHI_GPU_HAL_200,
		/* Ordinary Mesa node only; qualification paths keep the bare ABI. */
		.features = p->ordinary ? GPU_M4_DRM_FEATURES : 0,
	};
	struct gpu_m4_drm_provider *provider;
	struct gpu_m4_drm *frontend;
	struct gpu_m4_drm_vm *vm;
	int ret;

	lockdep_assert_not_held(&gpu->lock);
	if (p->ordinary)
		m4_user_init(p);
	if (p->writeback_input_manifest) {
		ret = m4_provider_wb_load(p);
		if (ret)
			return ret;
	}
	/* HELLO/READY/execution exclusions and SystemRAM are final before any
	 * provider exists; build the immutable page index once, before the first
	 * oracle call. The session gate and dynamic upper extras stay live.
	 */
	mutex_lock(&gpu->lock);
	m4_provider_page_index_build_locked(p);
	mutex_unlock(&gpu->lock);
	provider = gpu_m4_drm_provider_create(&config);
	if (IS_ERR(provider))
		return PTR_ERR(provider);
	mutex_lock(&gpu->lock);
	p->provider = provider;
	p->params = params;
	mutex_unlock(&gpu->lock);
	frontend = gpu_m4_drm_create_private(gpu->dev, p, &params,
		p->ordinary ? &m4_user_frontend_ops : &ops,
		p->ordinary ? M4_ORDINARY_NATIVE_MEMORY_LIMIT : p->compute_manifest ? (M4_PROVIDER_CODE_COUNT + p->writeback_manifest +
		p->writeback_input_manifest + p->timestamp_manifest) *
		GPU_M4_DRM_PAGE_SIZE : 0);
	if (IS_ERR(frontend))
		return PTR_ERR(frontend);
	mutex_lock(&gpu->lock);
	p->frontend = frontend;
	mutex_unlock(&gpu->lock);
	if (p->ordinary)
		return 0;
	vm = gpu_m4_drm_vm_create_private(frontend, M4_PROVIDER_KERNEL_START, M4_PROVIDER_KERNEL_END);
	if (IS_ERR(vm))
		return PTR_ERR(vm);
	if (vm != p->frontend_vm || gpu_m4_drm_vm_backend(vm) != p->vm)
		return -EPROTO;
	ret = p->compute_manifest ? m4_provider_code_stage(p) : 0;
	if (!ret && p->timestamp_manifest)
		ret = m4_provider_timestamp_stage(p);
	return ret || !p->writeback_manifest ? ret : m4_provider_writeback_stage(p);
}

static int m4_provider_native_coverage(struct m4_gpu_provider_runtime *p)
{
	const struct gpu_m4_drm_cache_access *cache;
	const struct gpu_m4_drm_resv_access *resv;
	u32 nc, nr, expected = M4_PROVIDER_CODE_COUNT + p->writeback_manifest +
		p->writeback_input_manifest;

	if (!p->work_snapshot || !p->work_version || !p->version[1])
		return -EPROTO;
	cache = gpu_m4_drm_vm_owner_cache_access(p->work_snapshot, &nc);
	resv = gpu_m4_drm_vm_owner_resv_access(p->work_snapshot, &nr);
	if (!cache || !resv || nc != expected || nr != expected ||
	    gpu_m4_drm_vm_owner_vm(p->work_snapshot) != p->frontend_vm ||
	    p->work_version != p->version[1] ||
	    !gpu_m4_drm_provider_version_native_covered_locked(p->version[1], p->work_snapshot))
		return -EPROTO;
	for (u32 i = 0; i < ARRAY_SIZE(p->code); i++) {
		bool found_cache = false, found_resv = false;

		if (!p->code[i].gem || !p->code[i].object ||
		    !gpu_m4_drm_provider_object_native(p->code[i].object))
			return -EPROTO;
		for (u32 j = 0; j < nc; j++)
			if (cache[j].pin == gpu_m4_drm_provider_object_native(p->code[i].object) &&
			    gpu_m4_drm_bo_pin_object(cache[j].pin) == p->code[i].gem &&
			    !cache[j].offset && cache[j].size == GPU_M4_DRM_PAGE_SIZE &&
			    cache[j].access == DRM_ASAHI_BIND_READ)
				found_cache = true;
		for (u32 j = 0; j < nr; j++)
			if (resv[j].object == p->code[i].gem && resv[j].access == DRM_ASAHI_BIND_READ)
				found_resv = true;
		if (!found_cache || !found_resv)
			return -EPROTO;
	}
	for (u32 role = 0; p->writeback_manifest &&
	     role < 1U + p->writeback_input_manifest; role++) {
		struct m4_provider_writeback_output *w = role ? &p->wb_input : &p->wb_output;
		bool found_cache = false, found_resv = false;
		u32 access = DRM_ASAHI_BIND_READ | (role ? 0 : DRM_ASAHI_BIND_WRITE);

		if (!w->gem || !w->object ||
		    gpu_m4_drm_provider_object_identity(w->object) != w->gem ||
		    !gpu_m4_drm_provider_object_native(w->object))
			return -EPROTO;
		for (u32 j = 0; j < nc; j++)
			if (cache[j].pin == gpu_m4_drm_provider_object_native(w->object) &&
			    gpu_m4_drm_bo_pin_object(cache[j].pin) == w->gem &&
			    !cache[j].offset && cache[j].size == GPU_M4_DRM_PAGE_SIZE &&
			    cache[j].access == access)
				found_cache = true;
		for (u32 j = 0; j < nr; j++)
			if (resv[j].object == w->gem && resv[j].access == access)
				found_resv = true;
		if (!found_cache || !found_resv)
			return -EPROTO;
	}
	return 0;
}

static int m4_provider_prepare_locked(struct m4_gpu_provider_runtime *p)
{
	struct m4_gpu_hello *h = p->hello;
	int ret = m4_provider_idle(h);

	lockdep_assert_held(&h->gpu->lock);
	if (!ret && p->compute_manifest) {
		struct gpu_m4_drm_uat_private external[M4_PROVIDER_EXTERNAL_COUNT];
		u32 count = 0;

		ret = gpu_m4_drm_provider_vm_capture_locked(p->vm, &p->base, &p->base_snapshot);
		if (!ret && (gpu_m4_drm_vm_owner_vm(p->base_snapshot) != p->frontend_vm ||
		    gpu_m4_drm_vm_owner_leaf_count(p->base_snapshot) != M4_PROVIDER_CODE_COUNT))
			ret = -EPROTO;
		if (!ret)
			ret = m4_provider_external_stage_locked(p);
		if (!ret && p->writeback_input_manifest) {
			p->wb_input_claim = gpu_m4_drm_provider_private_reserve_locked(p->vm,
				GPU_M4_WB_INPUT_VA, GPU_M4_DRM_PAGE_SIZE,
				GPU_M4_DRM_PAGE_SIZE, GPU_M4_DRM_PAGE_SIZE);
			if (IS_ERR(p->wb_input_claim)) {
				ret = PTR_ERR(p->wb_input_claim);
				p->wb_input_claim = NULL;
			}
		}
		if (ret)
			dev_info(h->gpu->dev, "GPU detached manifest base/external staging failed: %d (no root exposed)\n", ret);
		for (u32 i = 0; !ret && i < ARRAY_SIZE(external); i++) {
			struct m4_provider_external *x = &p->external[i];

			if (p->writeback_manifest && i == 5)
				continue; /* Retained original NC owner, not selected WB leaf. */
			external[count++] = (struct gpu_m4_drm_uat_private) {
				.identity = x->source, .cookie = x,
				.get = m4_provider_external_get, .put = m4_provider_external_put,
				.page = m4_provider_external_page, .va = x->va, .size = x->size,
				.access = x->access, .cpu_policy = GPU_M4_DRM_CPU_NC,
			};
		}
		if (!ret) {
			p->common = gpu_m4_drm_provider_version_extend_private_locked(
				p->base, external, count);
			if (IS_ERR(p->common)) {
				ret = PTR_ERR(p->common);
				p->common = NULL;
				dev_info(h->gpu->dev,
					 "GPU detached V0 manifest build failed: %d (no root exposed)\n", ret);
			}
		}
		if (!ret && p->writeback_manifest) {
			struct gpu_m4_drm_provider_private_gem data[2] = {
				{ .object = p->wb_output.object, .lease = p->external[5].claim,
				  .va = M4_EXEC_OUTPUT_GPU, .size = GPU_M4_DRM_PAGE_SIZE,
				  .access = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE },
				{ .object = p->wb_input.object, .lease = p->wb_input_claim,
				  .va = GPU_M4_WB_INPUT_VA, .size = GPU_M4_DRM_PAGE_SIZE,
				  .access = DRM_ASAHI_BIND_READ },
			};

			p->version[0] = gpu_m4_drm_provider_version_extend_private_gem_locked(
				p->common, data, 1U + p->writeback_input_manifest);
			if (IS_ERR(p->version[0])) {
				ret = PTR_ERR(p->version[0]);
				p->version[0] = NULL;
			}
		} else if (!ret)
			p->version[0] = p->common; /* Same returned baseline, no extra put. */
		if (!ret) {
			struct gpu_m4_drm_provider_version *captured;

			ret = gpu_m4_drm_provider_version_capture_locked(p->version[0],
				&captured, &p->snapshot);
			if (!ret && captured != p->version[0])
				ret = -EPROTO;
		}
	} else if (!ret)
		ret = gpu_m4_drm_provider_vm_capture_locked(p->vm, &p->version[0], &p->snapshot);
	if (!ret && (gpu_m4_drm_vm_owner_vm(p->snapshot) != p->frontend_vm ||
	    gpu_m4_drm_vm_owner_leaf_count(p->snapshot) != (p->compute_manifest ? M4_PROVIDER_CODE_COUNT : 0)))
		ret = -EPROTO;
	if (!ret) {
		ret = gpu_m4_drm_provider_version_get_locked(p->version[0]);
		p->v0_extra = !ret;
	}
	if (!ret) {
		p->version[1] = gpu_m4_drm_provider_version_extend_private_locked(p->version[0], NULL, 0);
		if (IS_ERR(p->version[1])) {
			ret = PTR_ERR(p->version[1]);
			p->version[1] = NULL;
		}
	}
	for (u32 i = 0; !ret && i < 2; i++) {
		u64 high;

		ret = gpu_m4_drm_provider_version_roots_locked(p->version[i], &p->root[i], &high);
		if (!ret && high)
			ret = -EPROTO;
		/* The fixed diagnostic manifests always install into ASID 1. */
		p->root[i] = m4_provider_ttbat_low(p->root[i], 1);
	}
	if (!ret && (p->root[0] == p->root[1] ||
	    !gpu_m4_drm_provider_version_same_user_locked(p->version[0], p->version[1])))
		ret = -EPROTO;
	if (!ret && p->compute_manifest) {
		ret = gpu_m4_drm_provider_version_capture_locked(p->version[1],
			&p->work_version, &p->work_snapshot);
		if (!ret)
			ret = m4_provider_native_coverage(p);
		if (!ret)
			p->manifest_ready = true;
	}
	/* Expected manifest comes from the retained allocation oracle, not a live
	 * candidate walk. Every table/zero is additionally checked by UAT refresh.
	 */
	for (u32 i = 0; !ret && i < ARRAY_SIZE(p->context); i++) {
		if (!p->context_identity || !p->context_pa[i]) {
			ret = -EPROTO;
			break;
		}
		p->context[i] = (struct gpu_m4_drm_vm_leaf){
			.va = GPU_M4_DRM_UAT_CONTEXT_START + (u64)i * GPU_M4_DRM_PAGE_SIZE,
			.pa = p->context_pa[i], .object_offset = (u64)i * GPU_M4_DRM_PAGE_SIZE,
			.identity = p->context_identity, .cpu_policy = GPU_M4_DRM_CPU_NC,
			.access = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE,
		};
	}
	if (ret)
		return ret;
	p->prepared = true;
	if (p->compute_manifest) {
		ret = m4_provider_manifest_valid(p);
		if (ret)
			return ret;
	}
	return 0;
}

static int m4_provider_run_locked(struct m4_gpu_provider_runtime *p)
{
	struct m4_gpu_hello *h = p->hello;
	u64 stable, deadline;
	int ret;

	lockdep_assert_held(&h->gpu->lock);
	if (!p->prepared || (p->writeback_manifest && !p->staged_work))
		return -EPROTO;
	ret = m4_provider_bind(p, 0);
	if (!ret && p->generation != 1)
		ret = -EPROTO;
	if (!ret)
		ret = m4_provider_bind(p, 1);
	if (!ret && p->generation != 2)
		ret = -EPROTO;
	if (ret)
		return ret;
	stable = ktime_get_ns();
	deadline = stable + 2 * NSEC_PER_SEC;
	do {
		u64 now;

		ret = m4_provider_idle(h);
		if (ret)
			return ret;
		now = ktime_get_ns();
		if (now >= deadline)
			return -ETIMEDOUT;
		if (now - stable >= 50 * NSEC_PER_MSEC) {
			p->complete = true;
			return 0;
		}
		usleep_range(1000, 2000);
	} while (ktime_get_ns() < deadline);
	return -ETIMEDOUT;
}

static int m4_provider_ts_lane_check_locked(void *owner,
	struct gpu_m4_drm_provider_job *job)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_provider_ts_job *j = p->ts_job;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!j || j->runtime != p || j->job != job || j->released)
		return -EPERM;
	return gpu_m4_drm_provider_lane_check_active_locked(p->ts_lane,
		p->provider, &j->item, job);
}

static int m4_provider_ts_queue_get_locked(void *owner, void *queue)
{
	struct m4_gpu_provider_runtime *p = owner;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (queue != p || !p->timestamp_manifest || p->error ||
	    !refcount_inc_not_zero(&p->ts_queue_refs))
		return -EPERM;
	/* The actual provider/queue references retain p's permanent READY session.
	 * This private queue counter never consumes that qualifier baseline.
	 */
	return 0;
}

static void m4_provider_ts_queue_put_locked(void *owner, void *queue)
{
	struct m4_gpu_provider_runtime *p = owner;

	lockdep_assert_held(&p->hello->gpu->lock);
	WARN_ON(queue != p || !refcount_dec_not_one(&p->ts_queue_refs));
}

static int m4_provider_ts_timestamp_hold_locked(void *owner,
	struct gpu_m4_drm_provider_version *version,
	struct gpu_m4_drm_provider_object *object,
	const struct gpu_m4_drm_timestamp *borrowed, void **alias, u64 *firmware_va)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct gpu_m4_drm_timestamp_alias_request *request;
	u64 va;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!alias || !firmware_va)
		return -EINVAL;
	if (!p->timestamp_manifest || version != p->version[1] || !borrowed ||
	    borrowed->backend != p->timestamp.cookie || borrowed->object != p->timestamp.gem ||
	    borrowed->offset != borrowed->object_offset ||
	    (borrowed->object_offset != 8 && borrowed->object_offset != 16))
		return -EPERM;
	ret = gpu_m4_drm_timestamp_alias_hold_locked(p->ts_aliases, version, object,
		borrowed, &request, &va);
	if (!ret) {
		*alias = request;
		*firmware_va = va;
	}
	return ret;
}

static void m4_provider_ts_timestamp_put_locked(void *owner, void *alias)
{
	struct m4_gpu_provider_runtime *p = owner;

	lockdep_assert_held(&p->hello->gpu->lock);
	gpu_m4_drm_timestamp_alias_request_put_locked(alias);
}

/* Detached encoder holds retain the SAME captured request, never a new alias
 * or job reference. Cache/snapshot protection is independently checked later
 * by token arm, after the actual job enters PUBLISHED.
 */
static int m4_provider_ts_hold(void *cookie,
	const struct gpu_m4_compute_ts_destination *destination, void **held)
{
	struct m4_provider_ts_job *j = cookie;
	struct m4_gpu_provider_runtime *p;
	struct gpu_m4_drm_provider_job_timestamp_view view;
	u32 count;
	int ret;

	if (!j || !destination || !held)
		return -EINVAL;
	p = j->runtime;
	lockdep_assert_held(&p->hello->gpu->lock);
	if (p->ts_job != j || j->released || !destination->object)
		return -EPERM;
	ret = gpu_m4_drm_provider_job_timestamp_count_locked(j->job, &count);
	if (!ret && count != 2)
		ret = -EPROTO;
	for (u32 i = 0; !ret && i < count; i++) {
		ret = gpu_m4_drm_provider_job_timestamp_at_locked(j->job, i, &view);
		if (ret)
			return ret;
		if (view.runtime_alias != destination->lease)
			continue;
		if (view.object != destination->object || view.gem != p->timestamp.gem ||
		    view.object_offset != destination->object_offset ||
		    view.firmware_va != destination->firmware_va || view.command_index ||
		    view.timestamp_index != i || view.object_offset != 8 + (u64)i * 8)
			return -EPERM;
		ret = gpu_m4_drm_timestamp_alias_request_get_locked(p->ts_aliases,
			view.runtime_alias);
		if (!ret)
			*held = view.runtime_alias;
		return ret;
	}
	return ret ? ret : -ENOENT;
}

static void m4_provider_ts_put(void *cookie, void *held)
{
	struct m4_provider_ts_job *j = cookie;

	lockdep_assert_held(&j->runtime->hello->gpu->lock);
	gpu_m4_drm_timestamp_alias_request_put_locked(held);
}

static int m4_provider_ts_job_check_locked(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_job *job, bool accepting)
{
	struct gpu_m4_drm_provider_job_view view = { 0 };
	struct gpu_m4_drm_provider_job_timestamp_view ts[2];
	const struct gpu_m4_drm_cache_access *cache;
	const struct gpu_m4_drm_resv_access *resv;
	const struct gpu_m4_drm_job_compute *compute;
	u32 count, nc, nr, code_cache = 0, code_resv = 0;
	bool timestamp_cache = false, timestamp_resv = false;
	u64 pa;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!p->timestamp_manifest || p->writeback_manifest || p->work ||
	    !p->complete || !p->selected || p->generation != 2 ||
	    p->resident != p->version[1])
		return -EPERM;
	if (!p->timestamp.cookie || !p->timestamp.gem ||
	    gpu_m4_drm_bo_policy(p->timestamp.gem) != p->timestamp.policy ||
	    gpu_m4_drm_provider_timestamp_policy(p->timestamp.cookie) != p->timestamp.policy)
		return -EPERM;
	ret = accepting ? gpu_m4_drm_provider_job_check_accepting_locked(job, p->provider) :
		gpu_m4_drm_provider_job_check_owner_locked(job, p->provider);
	if (ret)
		return ret;
	gpu_m4_drm_provider_job_view(job, &view);
	if (view.version != p->version[1] || !view.snapshot || !view.batch ||
	    !view.batch->commands || view.batch->count != 1 ||
	    view.batch->vm_snapshot != view.snapshot || view.runtime_queue != p ||
	    view.stage_owner || (accepting ? !!view.runtime_job : view.runtime_job != p->ts_job) ||
	    gpu_m4_drm_vm_owner_vm(view.snapshot) != p->frontend_vm ||
	    gpu_m4_drm_vm_owner_generation(view.snapshot) !=
		gpu_m4_drm_provider_version_generation(view.version) ||
	    !gpu_m4_drm_provider_version_native_covered_locked(view.version, view.snapshot) ||
	    view.batch->commands[0].type != DRM_ASAHI_CMD_COMPUTE)
		return -EPERM;
	compute = &view.batch->commands[0].compute;
	if (compute->flags || compute->cdm_base != M4_EXEC_CDM_GPU ||
	    compute->cdm_end != M4_EXEC_CDM_GPU + 48 || compute->usc_base != M4_EXEC_USC_BASE ||
	    compute->attachment_count != 1 || compute->attachments[0].va != M4_EXEC_OUTPUT_GPU ||
	    compute->attachments[0].size != 128 ||
	    compute->vdm_barrier != DRM_ASAHI_BARRIER_NONE ||
	    compute->cdm_barrier != DRM_ASAHI_BARRIER_NONE)
		return -EOPNOTSUPP;
	ret = gpu_m4_drm_provider_job_timestamp_count_locked(job, &count);
	if (!ret && count != 2)
		ret = -EPROTO;
	for (u32 i = 0; !ret && i < 2; i++) {
		ret = gpu_m4_drm_provider_job_timestamp_at_locked(job, i, &ts[i]);
		if (!ret && (!ts[i].runtime_alias || !ts[i].object || ts[i].gem != p->timestamp.gem ||
		    ts[i].command_index || ts[i].timestamp_index != i ||
		    ts[i].object_offset != 8 + (u64)i * 8))
			ret = -EPERM;
		if (!ret)
			ret = gpu_m4_drm_provider_object_origin_locked(p->provider, view.version,
				ts[i].object);
		if (!ret)
			ret = gpu_m4_drm_provider_object_page_locked(ts[i].object, 0, &pa);
		if (!ret && pa != p->timestamp.pa)
			ret = -ESTALE;
	}
	if (ret)
		return ret;
	if (ts[0].object != ts[1].object || ts[0].runtime_alias == ts[1].runtime_alias ||
	    ts[0].firmware_va > U64_MAX - 8 || ts[1].firmware_va != ts[0].firmware_va + 8)
		return -EPROTO;
	cache = gpu_m4_drm_vm_owner_cache_access(view.snapshot, &nc);
	resv = gpu_m4_drm_vm_owner_resv_access(view.snapshot, &nr);
	if (!cache || !resv || nc != 4 || nr != 4)
		return -EPROTO;
	for (u32 n = 0; n < 4; n++) {
		for (u32 i = 0; i < ARRAY_SIZE(p->code); i++) {
			if (cache[n].pin == gpu_m4_drm_provider_object_native(p->code[i].object) &&
			    gpu_m4_drm_bo_pin_object(cache[n].pin) == p->code[i].gem &&
			    !cache[n].offset && cache[n].size == GPU_M4_DRM_PAGE_SIZE &&
			    cache[n].access == DRM_ASAHI_BIND_READ)
				code_cache |= BIT(i);
			if (resv[n].object == p->code[i].gem && resv[n].access == DRM_ASAHI_BIND_READ)
				code_resv |= BIT(i);
		}
		/* Current request pin, never the historical canonical anchor's pin. */
		if (cache[n].pin == gpu_m4_drm_provider_object_native(ts[0].object) &&
		    gpu_m4_drm_bo_pin_object(cache[n].pin) == p->timestamp.gem &&
		    cache[n].offset == 8 && cache[n].size == 16 && cache[n].access == DRM_ASAHI_BIND_WRITE)
			timestamp_cache = true;
		if (resv[n].object == p->timestamp.gem && resv[n].access == DRM_ASAHI_BIND_WRITE)
			timestamp_resv = true;
	}
	return code_cache == 7 && code_resv == 7 && timestamp_cache && timestamp_resv ? 0 : -EACCES;
}

static void m4_provider_ts_reap(void *owner)
{
	struct m4_gpu_provider_runtime *p = owner;

	lockdep_assert_not_held(&p->hello->gpu->lock);
	gpu_m4_drm_timestamp_alias_reap(p->ts_aliases);
	gpu_m4_drm_provider_reap(p->provider);
}

static void m4_provider_ts_reap_work(struct work_struct *work)
{
	struct m4_gpu_provider_runtime *p = container_of(work,
		struct m4_gpu_provider_runtime, ts_reap_work);

	m4_provider_ts_reap(p);
}

static void m4_provider_ts_success_locked(void *owner,
	struct gpu_m4_drm_provider_lane_item *item, struct gpu_m4_drm_provider_job *job)
{
	struct m4_provider_ts_job *j = owner;
	struct m4_gpu_provider_runtime *p = j->runtime;
	struct gpu_m4_drm_timestamp_alias_token *token = j->token;

	lockdep_assert_held(&p->hello->gpu->lock);
	/* The actual lane has already committed error-free job retirement. No
	 * validation, timestamp read or fallible success stage belongs here.
	 */
	if (WARN_ON(item != &j->item || job != j->job || p->ts_job != j || !token))
		return;
	j->token = NULL;
	gpu_m4_drm_timestamp_alias_token_complete_locked(token);
	gpu_m4_compute_ts_put(&j->ts_plan);
	j->completed = true;
	m4_provider_ts_work_complete_locked(p, job, j->work);
	schedule_work(&p->ts_reap_work);
}

static int m4_provider_ts_accept_locked(void *owner, void *queue,
	struct gpu_m4_drm_provider_job *job, void **runtime_job)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_provider_ts_job *j;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (queue != p || !runtime_job || p->ts_job)
		return -EBUSY;
	ret = m4_provider_ts_job_check_locked(p, job, true);
	if (ret)
		return ret;
	j = kvzalloc(sizeof(*j), GFP_KERNEL);
	if (!j)
		return -ENOMEM;
	j->runtime = p;
	j->job = job; /* borrowed ONLY until actual post-accept enqueue */
	p->ts_job = j; /* Permanent diagnostic payload, including partial errors. */
	ret = m4_provider_ts_work_accept_locked(p, job, &j->work);
	if (!ret)
		ret = gpu_m4_drm_provider_lane_item_init_locked(p->ts_lane, &j->item, job, p);
	if (!ret)
		ret = gpu_m4_drm_provider_lane_item_set_retired_locked(&j->item, j,
			m4_provider_ts_success_locked);
	/* LAST fallible step. No successfully prepared token can be abandoned
	 * while the job is still borrowed by synchronous acceptance.
	 */
	if (!ret)
		ret = gpu_m4_drm_timestamp_alias_token_prepare_locked(p->ts_aliases, job, &j->token);
	if (ret) {
		if (j->item.lane)
			WARN_ON(gpu_m4_drm_provider_lane_item_fini_locked(&j->item));
		if (j->work)
			m4_provider_ts_work_release_locked(p, j->work);
		j->error = ret;
		j->released = true;
		j->job = NULL;
		return ret;
	}
	*runtime_job = j;
	return 0;
}

static void m4_provider_ts_release_locked(void *owner, void *runtime_job)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_provider_ts_job *j = runtime_job;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (WARN_ON(j != p->ts_job || j->runtime != p || j->job_held))
		return; /* Permanent/uncertain owners are never disposed here. */
	if (j->token) {
		ret = gpu_m4_drm_timestamp_alias_token_cancel_locked(j->token);
		if (WARN_ON(ret))
			return;
		j->token = NULL;
	}
	if (j->item.lane && WARN_ON(gpu_m4_drm_provider_lane_item_fini_locked(&j->item)))
		return;
	gpu_m4_compute_ts_put(&j->ts_plan);
	if (j->work)
		m4_provider_ts_work_release_locked(p, j->work);
	j->released = true;
	j->job = NULL;
}

static void m4_provider_ts_enqueue_locked(void *owner,
	struct gpu_m4_drm_provider_job *job)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_provider_ts_job *j = p->ts_job;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	/* Only post-accept enqueue owns a transferable job reference. Keep an
	 * independent qualifier baseline before the lane consumes that reference;
	 * debug work and canonical evidence may outlive the scheduler and token.
	 */
	if (WARN_ON(!j || j->job != job || j->released || j->job_held))
		return;
	ret = gpu_m4_drm_provider_job_get_locked(job);
	if (WARN_ON(ret)) {
		gpu_m4_drm_provider_job_fault_locked(job, ret);
		return; /* The transferred owner is retained, never guessed away. */
	}
	j->job_held = true;
	gpu_m4_drm_provider_lane_enqueue_locked(&j->item);
}

static void m4_provider_ts_close_locked(void *owner, void *queue)
{
	struct m4_gpu_provider_runtime *p = owner;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (WARN_ON(queue != p))
		return;
	gpu_m4_drm_provider_lane_close_queue_locked(p->ts_lane, queue);
}

static int m4_provider_ts_prepare_locked(void *owner,
	struct gpu_m4_drm_provider_job *job)
{
	struct m4_gpu_provider_runtime *p = owner;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	ret = m4_provider_ts_lane_check_locked(p, job);
	if (!ret)
		ret = m4_provider_ts_job_check_locked(p, job, false);
	if (!ret)
		ret = m4_provider_ts_work_prepare_locked(p, job, p->ts_job->work);
	return ret;
}

static int m4_provider_ts_publish_locked(void *owner,
	struct gpu_m4_drm_provider_job *job)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_provider_ts_job *j = p->ts_job;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	ret = m4_provider_ts_lane_check_locked(p, job);
	if (!ret)
		ret = m4_provider_ts_job_check_locked(p, job, false);
	if (!ret)
		ret = gpu_m4_drm_timestamp_alias_token_arm_locked(j->token, job);
	/* Arm checks the actual PUBLISHED/cache/resv/lane owner. Publication marks
	 * uncertainty before its first upper write. No failure unarms or releases.
	 */
	if (!ret)
		ret = gpu_m4_drm_timestamp_alias_token_publish_locked(j->token, true);
	if (!ret)
		ret = gpu_m4_drm_timestamp_alias_token_check_locked(j->token);
	if (!ret)
		ret = m4_provider_ts_work_publish_locked(p, job, j->work);
	return ret;
}

static int m4_provider_ts_retired_locked(void *owner,
	struct gpu_m4_drm_provider_job *job)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_provider_ts_job *j = p->ts_job;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	ret = m4_provider_ts_lane_check_locked(p, job);
	if (!ret)
		ret = m4_provider_ts_job_check_locked(p, job, false);
	if (ret)
		return ret;
	ret = m4_provider_ts_work_retired_locked(p, job, j->work);
	if (ret != 1)
		return ret;
	/* Every fallible alias/translation/lane check precedes the core's actual
	 * cache/version retirement and success-ready fence commit.
	 */
	ret = gpu_m4_drm_timestamp_alias_token_check_locked(j->token);
	return ret ? ret : 1;
}

static void m4_provider_ts_fault_locked(void *owner,
	struct gpu_m4_drm_provider_job *job, int error)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_provider_ts_job *j = p->ts_job;

	lockdep_assert_held(&p->hello->gpu->lock);
	error = error < 0 ? error : -EIO;
	if (j && j->job == job) {
		if (!j->error)
			j->error = error;
		if (j->token)
			gpu_m4_drm_timestamp_alias_token_fault_locked(j->token, error);
		if (j->work)
			m4_provider_ts_work_fault_locked(p, job, j->work, error);
	}
	gpu_m4_drm_provider_lane_fault_locked(p->ts_lane, error);
	m4_provider_fault_locked(p, error);
}

static enum drm_gpu_sched_stat m4_provider_ts_timedout(void *owner,
	struct gpu_m4_drm_provider_job *job, struct drm_sched_job *sched_job)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct mutex *lock = &p->hello->gpu->lock;
	bool completed;

	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	completed = p->ts_job && p->ts_job->job == job && p->ts_job->completed;
	if (!completed)
		gpu_m4_drm_provider_job_fault_locked(job, -ETIMEDOUT);
	mutex_unlock(lock);
	if (completed)
		return DRM_GPU_SCHED_STAT_NO_HANG;
	/* A terminal qualifier fault is not a reset/retry. Commit and signal the
	 * actual error fence outside the GPU lock, then stop the scheduler with
	 * its guilty job so its timeout worker can dispose the scheduler owner.
	 * Lane/token/qualifier references retain all uncertain backing separately.
	 */
	WARN_ON(gpu_m4_drm_provider_job_signal(job));
	drm_sched_stop(sched_job->sched, sched_job);
	return DRM_GPU_SCHED_STAT_ENODEV;
}

static const struct gpu_m4_drm_provider_job_runtime m4_provider_ts_ops = {
	.queue_get_locked = m4_provider_ts_queue_get_locked,
	.queue_put_locked = m4_provider_ts_queue_put_locked,
	.timestamp_hold_locked = m4_provider_ts_timestamp_hold_locked,
	.timestamp_put_locked = m4_provider_ts_timestamp_put_locked,
	.accept_locked = m4_provider_ts_accept_locked,
	.release_locked = m4_provider_ts_release_locked,
	.enqueue_locked = m4_provider_ts_enqueue_locked,
	.close_locked = m4_provider_ts_close_locked,
	.prepare_locked = m4_provider_ts_prepare_locked,
	.publish_locked = m4_provider_ts_publish_locked,
	.retired_locked = m4_provider_ts_retired_locked,
	.fault_locked = m4_provider_ts_fault_locked,
	.timedout = m4_provider_ts_timedout,
	.reap = m4_provider_ts_reap,
};

static int m4_provider_ts_lookup(void *cookie, u32 handle, u32 offset,
	struct gpu_m4_drm_timestamp *timestamp)
{
	struct m4_gpu_provider_runtime *p = cookie;
	struct gpu_m4_drm_provider_timestamp_config config = m4_provider_ts_timestamp_config(p);
	int ret;

	lockdep_assert_not_held(&p->hello->gpu->lock);
	/* One private cookie-table index, NOT an invented GEM/VM_BIND handle.
	 * The global BIND_OBJECT owner is actual and permanently retained. Lookup
	 * is synchronous; neither this callback nor numeric index survives decode.
	 */
	if (!timestamp || handle != 1 || (offset != 8 && offset != 16) ||
	    !p->timestamp_manifest || !p->timestamp.cookie)
		return -EINVAL;
	if (!gpu_m4_drm_provider_timestamp_get(p->timestamp.cookie))
		return -EPERM;
	ret = gpu_m4_drm_provider_timestamp_matches(p->timestamp.cookie, &config) ?
		gpu_m4_drm_provider_timestamp_slice(p->timestamp.cookie, offset, timestamp) : -EPERM;
	gpu_m4_drm_provider_timestamp_put(p->timestamp.cookie);
	return ret;
}

static int m4_provider_ts_submit(struct m4_gpu_provider_runtime *p)
{
	struct gpu_m4_drm_timestamp_alias_config aliases;
	struct gpu_m4_drm_provider_queue_config queue;
	struct gpu_m4_drm_timestamp_aliases *registry;
	struct gpu_m4_drm_provider_lane *lane;
	struct gpu_m4_drm_provider_queue *q;
	struct drm_asahi_cmd_header attachment_header = {
		.cmd_type = DRM_ASAHI_SET_COMPUTE_ATTACHMENTS,
		.size = sizeof(struct drm_asahi_attachment),
		.vdm_barrier = DRM_ASAHI_BARRIER_NONE, .cdm_barrier = DRM_ASAHI_BARRIER_NONE,
	};
	struct drm_asahi_attachment attachment = { .pointer = M4_EXEC_OUTPUT_GPU, .size = 4 };
	struct drm_asahi_cmd_header compute_header = {
		.cmd_type = DRM_ASAHI_CMD_COMPUTE, .size = sizeof(struct drm_asahi_cmd_compute),
		.vdm_barrier = DRM_ASAHI_BARRIER_NONE, .cdm_barrier = DRM_ASAHI_BARRIER_NONE,
	};
	struct drm_asahi_cmd_compute compute = {
		.cdm_ctrl_stream_base = M4_EXEC_CDM_GPU,
		.cdm_ctrl_stream_end = M4_EXEC_CDM_GPU + 48,
		.ts = { .start = { .handle = 1, .offset = 8 }, .end = { .handle = 1, .offset = 16 } },
	};
	u8 bytes[2 * sizeof(struct drm_asahi_cmd_header) + sizeof(attachment) + sizeof(compute)];
	struct gpu_m4_drm_submit submit = {
		.cmdbuf = bytes, .size = sizeof(bytes), .lookup_cookie = p,
		.lookup_timestamp = m4_provider_ts_lookup,
	};
	struct dma_fence *finished;
	struct mutex *lock;
	u32 at = 0;
	int ret;

	if (!p || !p->hello || !p->hello->gpu)
		return -EINVAL;
	lock = &p->hello->gpu->lock;
	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	if (!p->timestamp_manifest || !p->compute_manifest || p->writeback_manifest ||
	    !p->complete || !p->selected || p->generation != 2 ||
	    p->resident != p->version[1] || p->work || p->staged_work ||
	    p->ts_attempted || p->ts_job || !p->timestamp.initialized || p->error) {
		mutex_unlock(lock);
		return -EPERM;
	}
	p->ts_attempted = true; /* No retry after any partial construction. */
	refcount_set(&p->ts_queue_refs, 1);
	INIT_WORK(&p->ts_reap_work, m4_provider_ts_reap_work);
	ret = m4_provider_ts_upper_stage_locked(p);
	mutex_unlock(lock);
	if (ret)
		goto fault;
	aliases = (struct gpu_m4_drm_timestamp_alias_config) {
		.provider = p->provider, .upper = p->ready->upper,
		.timestamp = m4_provider_ts_timestamp_config(p), .owner = p,
		.owner_check_locked = m4_provider_ts_owner_check_locked,
		.lane_check_locked = m4_provider_ts_lane_check_locked,
		.memory_limit = 0x10000, .max_entries = 1, .max_pages = 1,
		.max_requests = 2, .max_tokens = 1,
	};
	registry = gpu_m4_drm_timestamp_alias_create(&aliases);
	if (IS_ERR(registry)) {
		ret = PTR_ERR(registry);
		goto fault;
	}
	mutex_lock(lock);
	p->ts_aliases = registry;
	mutex_unlock(lock);
	lane = gpu_m4_drm_provider_lane_create(p->provider, 1, 2000, 1);
	if (IS_ERR(lane)) {
		ret = PTR_ERR(lane);
		goto fault;
	}
	mutex_lock(lock);
	p->ts_lane = lane;
	mutex_unlock(lock);
	queue = (struct gpu_m4_drm_provider_queue_config) {
		.dev = p->hello->gpu->dev, .vm = p->vm, .params = &p->params,
		.kernel_start = M4_PROVIDER_KERNEL_START, .kernel_end = M4_PROVIDER_KERNEL_END,
		.usc_base = M4_EXEC_USC_BASE, .client_id = 1,
		.priority = DRM_ASAHI_PRIORITY_MEDIUM, .timeout = msecs_to_jiffies(2000),
		.runtime_owner = p, .runtime_queue = p, .runtime = &m4_provider_ts_ops,
	};
	q = gpu_m4_drm_provider_queue_create_private(&queue, p->version[1]);
	if (IS_ERR(q)) {
		ret = PTR_ERR(q);
		goto fault;
	}
	mutex_lock(lock);
	p->ts_queue = q;
	mutex_unlock(lock);
	/* Copy explicit real UAPI records without compiler-inserted inter-record
	 * padding. No original command/container/Ioto/preimage bytes are replaced.
	 */
	memcpy(bytes + at, &attachment_header, sizeof(attachment_header));
	at += sizeof(attachment_header);
	memcpy(bytes + at, &attachment, sizeof(attachment));
	at += sizeof(attachment);
	memcpy(bytes + at, &compute_header, sizeof(compute_header));
	at += sizeof(compute_header);
	memcpy(bytes + at, &compute, sizeof(compute));
	finished = gpu_m4_drm_provider_queue_submit(q, &submit);
	if (IS_ERR(finished)) {
		ret = PTR_ERR(finished);
		goto fault;
	}
	if (!finished) {
		ret = -EPROTO;
		goto fault;
	}
	mutex_lock(lock);
	p->ts_finished = finished; /* Real core/scheduler finished-fence owner. */
	mutex_unlock(lock);
	return 0;
fault:
	mutex_lock(lock);
	m4_provider_fault_locked(p, ret);
	mutex_unlock(lock);
	/* No queue close/destroy, token cancel, root rollback or backing release:
	 * every successfully constructed owner remains reachable for diagnosis.
	 */
	return ret;
}

static ssize_t provider_root_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_provider_runtime *p;
	ssize_t size;
	int prebind_error = 0;

	if (!gpu)
		return -ENODEV;
	mutex_lock(&gpu->lock);
	p = gpu->hello && gpu->hello->ready ? gpu->hello->ready->provider_runtime : NULL;
	if (p && p->selected && !p->error) {
		int ret = p->ts_job && p->ts_job->work ? m4_provider_ts_work_health_locked(p) :
			p->work ? m4_provider_work_health(p) : m4_provider_idle(p->hello);

		if (ret)
			m4_provider_fault_locked(p, ret);
	}
	if (!p)
		size = sysfs_emit(buf, "attempted=0 prepared=0 complete=0 error=0 selected=0 generation=0 root0=0 root1=0 handoff=0,0 tlbi=0 owners=0\n");
	else
		size = sysfs_emit(buf, "attempted=%u prepared=%u complete=%u error=%d selected=%u generation=%llu root0=%#llx root1=%#llx handoff=%d,%d tlbi=%d owners=%u\n",
			p->attempted, p->prepared, p->complete, p->error, p->selected,
			p->generation, p->root[0], p->root[1], p->handoff_get, p->handoff_put,
			p->tlbi, !!p->version[0] + !!p->version[1]);
	if (p && p->writeback_manifest && !p->work)
		prebind_error = p->error;
	mutex_unlock(&gpu->lock);
	if (prebind_error)
		m4_provider_wb_prebind_fault(p, prebind_error);
	return size;
}
static DEVICE_ATTR_RO(provider_root_state);

static ssize_t provider_root_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_provider_runtime *p;
	struct m4_gpu_hello *h;
	int ret;

	if (!provider_root_trial || !gpu || !sysfs_streq(buf, "1") ||
	    (provider_timestamp_trial && (!provider_compute_trial || provider_writeback_trial)) ||
	    (provider_timestamp_writeback_trial && !provider_timestamp_trial) ||
	    (provider_writeback_trial && !provider_compute_trial) ||
	    (provider_writeback_input_trial && (!provider_writeback_trial || !provider_compute_trial)) ||
	    provider_writeback_challenge >= GPU_M4_WB_CHALLENGE_COUNT ||
	    (provider_writeback_challenge && !provider_writeback_input_trial))
		return -EINVAL;
	mutex_lock(&gpu->lock);
	h = gpu->hello;
	if (!h || !h->ready || !h->ready->execution || !h->ready->execution->store ||
	    h->ready->provider_runtime || !h->complete || !h->ready->complete ||
	    !h->ready->policy_complete || !h->ready->execution->barrier_complete ||
	    !h->ready->execution->store->prepared || h->ready->execution->store->error ||
	    h->ready->execution->error ||
	    upper_alias_trial || h->ready->execution->store->attempted ||
	    h->ready->execution->store->upper_attempted || (provider_compute_trial && !compute_enable)) {
		mutex_unlock(&gpu->lock);
		return -EBUSY;
	}
	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p) {
		h->ready->error = -ENOMEM;
		mutex_unlock(&gpu->lock);
		return -ENOMEM;
	}
	p->hello = h;
	p->ready = h->ready;
	p->execution = h->ready->execution;
	p->compute_manifest = provider_compute_trial;
	p->writeback_manifest = provider_writeback_trial;
	p->writeback_input_manifest = provider_writeback_input_trial;
	p->writeback_challenge = provider_writeback_challenge;
	p->timestamp_manifest = provider_timestamp_trial;
	p->timestamp.policy = provider_timestamp_writeback_trial ?
		GPU_M4_DRM_CPU_WRITEBACK : GPU_M4_DRM_CPU_NC;
	p->attempted = true;
	p->started = ktime_get_ns();
	p->handoff_get = p->handoff_put = p->tlbi = -EINPROGRESS;
	refcount_set(&p->session_refs, 1);
	__module_get(THIS_MODULE);
	get_device(gpu->dev);
	pm_runtime_get_noresume(gpu->dev);
	h->ready->provider_runtime = p;
	ret = m4_provider_idle(h);
	if (!ret)
		ret = m4_provider_fixed(h, 0, 0);
	mutex_unlock(&gpu->lock);
	if (!ret)
		ret = m4_provider_stage(p);
	mutex_lock(&gpu->lock);
	if (!ret)
		ret = m4_provider_prepare_locked(p);
	if (!ret && p->writeback_manifest) {
		/* Real resv/fence capture outside GPU; no root exposed yet. The NC
		 * path keeps its original uninterrupted prepare/bind serialization.
		 */
		mutex_unlock(&gpu->lock);
		ret = m4_provider_wb_prebind_stage(p);
		mutex_lock(&gpu->lock);
		if (!ret)
			ret = m4_provider_wb_prebind_protect_locked(p);
	}
	if (!ret)
		ret = m4_provider_run_locked(p);
	if (ret) {
		m4_provider_fault_locked(p, ret);
		ret = p->error;
	}
	p->finished = ktime_get_ns();
	dev_info(dev, "GPU private root-only trial complete=%u error=%d generation=%llu owners=%u; no engine work, owners retained\n",
		p->complete, ret, p->generation, !!p->version[0] + !!p->version[1]);
	mutex_unlock(&gpu->lock);
	if (ret && p->writeback_manifest)
		m4_provider_wb_prebind_fault(p, ret);
	return ret ? ret : (ssize_t)count;
}
static DEVICE_ATTR_WO(provider_root);

#endif
