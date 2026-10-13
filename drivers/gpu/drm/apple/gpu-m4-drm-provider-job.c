// SPDX-License-Identifier: GPL-2.0-only
/* Private Mesa acceptance -> real retained runtime -> scheduler retirement. */
#include <linux/debugfs.h>
#include <linux/dma-fence.h>
#include <linux/dma-fence-unwrap.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/refcount.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <drm/drm_gem.h>
#include <asm/arch_timer.h>
#include "gpu-m4-drm-provider-job.h"
#include "gpu-m4-drm-vm-tree.h"

struct gpu_m4_drm_provider_queue {
	struct kref ref;
	struct gpu_m4_drm_provider *provider;
	struct gpu_m4_drm_provider_vm *vm;
	/* Immutable kernel-private authority; NULL is the ordinary user profile. */
	struct gpu_m4_drm_provider_version *private_candidate;
	struct gpu_m4_drm_sched_queue *scheduler;
	struct drm_asahi_params_global params;
	struct gpu_m4_drm_provider_job_runtime runtime;
	void *runtime_owner, *runtime_queue;
	u64 kernel_start, kernel_end, usc_base;
	/* Running aggregates in run (= queue) order, one per scheduler credit,
	 * woken on close. Registered while its materializer ref is held;
	 * removed at terminalization.
	 */
	struct gpu_m4_drm_provider_job *aggregate[GPU_M4_DRM_SCHED_MAX_CREDITS];
	/* The next job materializing its first window before run, woken on
	 * close. Registered while its early materializer ref is held.
	 */
	struct gpu_m4_drm_provider_job *early;
	u32 execution_window, credit_limit;
	/* REMOVE-DEBUG: accepted public jobs the scheduler has not run yet;
	 * the latest public success signal and submit entry (pacing).
	 */
	atomic_t waiting;
	atomic64_t signal_ns, submit_ns;
	bool closed, vm_held, runtime_held;
	/* S3c: the queue's VM is a tree authority (sampled at create): its jobs
	 * take the TREE version and a guard, never a capture (see tree path).
	 */
	bool tree;
};

/* The fence itself may outlive every job/queue/backing reference. */
struct gpu_m4_drm_provider_fence {
	struct dma_fence base;
	spinlock_t lock; /* serializes ready/error and dma-fence callbacks */
	bool ready;
	int error;
};

enum gpu_m4_drm_provider_job_state {
	GPU_M4_PROVIDER_PREPARED,
	GPU_M4_PROVIDER_READY,
	GPU_M4_PROVIDER_PUBLISHED,
	GPU_M4_PROVIDER_RETIRED,
	GPU_M4_PROVIDER_FAILED,
	GPU_M4_PROVIDER_FAULTED,
	GPU_M4_PROVIDER_CANCELED,
	GPU_M4_PROVIDER_AGGREGATE,
	GPU_M4_PROVIDER_AGGREGATE_DONE,
	GPU_M4_PROVIDER_AGGREGATE_FAILED,
	GPU_M4_PROVIDER_AGGREGATE_FAULTED,
};

struct gpu_m4_drm_provider_job {
	refcount_t refs; /* serialized by provider mutex */
	struct gpu_m4_drm_provider_queue *queue;
	struct gpu_m4_drm_provider_version *version;
	/* Borrowed ONLY during decode, before batch.vm_snapshot is installed. */
	struct gpu_m4_drm_vm_owner *decode_owner;
	struct gpu_m4_drm_mixed_config decode;
	struct gpu_m4_drm_mixed_batch batch;
	struct gpu_m4_drm_job_command *commands;
	struct gpu_m4_drm_cache_job *cache;
	struct gpu_m4_drm_resv_job *resv;
	struct gpu_m4_drm_provider_fence *retirement;
	struct dma_fence *finished;
	struct gpu_m4_drm_provider_timestamp *deferred_timestamps;
	struct gpu_m4_drm_provider_object **stage_extra_objects;
	u32 stage_extra_count;
	/* Explicit sync only (!implicit): which snapshot identities its cache job
	 * owns (pins borrowed from the retained snapshot). cache_rows: this job's
	 * stage native/extra rows then the timestamp rows, dropped at cache
	 * creation; timestamp_rows: the public capture's (children read their
	 * parent's). Never the whole-VM inventory.
	 */
	struct gpu_m4_drm_cache_access *cache_rows, *timestamp_rows;
	u32 cache_row_count, timestamp_row_count;
	void *bytes, *runtime_job, *stage_owner;
	u32 size;
	/* The public scheduler owns only the cheap parent. Each real child has
	 * independent cache/snapshot/resv/version/native/retirement state and holds
	 * the original decoded metadata and timestamp leases through this ref.
	 */
	struct gpu_m4_drm_provider_job *window_parent, *active_window;
	/* window_prefetch: the following child, built and enqueued behind the
	 * still executing active one (lane FIFO order). Promoted to active when
	 * the active child completes; finalization drops it like active.
	 */
	struct gpu_m4_drm_provider_job *next_window;
	/* Parent-only CPU materializer. Workqueue non-reentrancy serializes every
	 * unlocked materialization field; the provider mutex guards the rest.
	 * window_work_ref is the ONE parent ref owned from run until finalization.
	 */
	struct work_struct window_work, window_put_work;
	/* Native capacity backpressure (-EAGAIN from stage_native): a bounded
	 * delayed rerun of window_work while the runtime reaper returns cells.
	 */
	struct delayed_work window_retry;
	u32 window_retries;
	/* Host capacity refusal of an unpublished child (provider charge or
	 * allocator): bounded reclaim-then-rebuild passes before this public
	 * batch alone fails. Never a provider or other-job fault.
	 */
	u32 window_capacity_retries;
	/* Capacity backpressure split: nonzero caps this batch's remaining
	 * children at that many commands (halved per refused build, >= 1).
	 */
	u32 window_limit;
	struct dma_fence_cb window_dependency_cb;
	struct dma_fence *window_dependency;
	/* REMOVE-DEBUG timing: early first-window completion, and run (every
	 * scheduler dependency signaled), carried to the first child.
	 */
	u64 window_built_ns, window_ready_ns;
	u32 command_first, command_total, window_cursor;
	/* Coverage memo of timestamp_batch_locked(): the exact held version and
	 * snapshot pair whose complete native_covered_locked() proof succeeded.
	 * Both are immutable while held; EVERY reassignment of either clears it
	 * first, so a reused address can never inherit the proof.
	 */
	struct gpu_m4_drm_provider_version *covered_version;
	const struct gpu_m4_drm_vm_owner *covered_snapshot;
	/* Every held timestamp view of this job in scan order, built by one
	 * complete scan against the covered version/snapshot pair and dropped
	 * with it (uncover). Leases, commands and the batch are immutable.
	 */
	struct gpu_m4_drm_provider_job_timestamp_view *ts_memo;
	u32 ts_memo_count, ts_memo_capacity;
	bool ts_memo_valid;
	enum gpu_m4_drm_provider_job_state state;
	/* implicit_sync sampled once by the public capture, inherited by every
	 * child: false never prepares, publishes or waits on a dma_resv.
	 */
	bool implicit;
	bool accepted, protected, staged, aggregate, window_work_ref, window_complete;
	bool window_enqueued, window_armed;
	/* Parent: its final child entered the lane FIFO (a later running batch
	 * of the queue may now materialize its first child).
	 */
	bool window_final;
	bool waiting_counted; /* REMOVE-DEBUG: counted in queue->waiting */
	/* REMOVE-DEBUG pacing: its submit entry; parent: final child's retirement. */
	u64 submit_ns, final_ns;
	/* Early first window (provider mutex): window_early while registered
	 * against early_max (never run, materializer ref held); window_evict
	 * asks it to drop its unsealed child; window_released is the
	 * scheduler's release of a never-run job; window_early_used marks a
	 * first window built before run (aggregate and that child).
	 */
	bool window_early, window_evict, window_released, window_early_used;
	/* S3c tree path: job->version is the VM's TREE version; guarded holds
	 * one VM guard from acceptance until successful retirement (before the
	 * fence signals). A published job that never retired keeps it.
	 */
	bool tree, guarded;
};

struct gpu_m4_drm_provider_timestamp {
	struct gpu_m4_drm_provider_job *job;
	struct gpu_m4_drm_provider_object *object;
	void *alias;
	u64 object_offset, firmware_va;
	struct gpu_m4_drm_provider_timestamp *next;
};

static struct mutex *gpu_m4_provider_job_lock(struct gpu_m4_drm_provider_job *job)
{
	return gpu_m4_drm_provider_lock(job->queue->provider);
}

static void gpu_m4_provider_window_work(struct work_struct *work);

/* REMOVE-DEBUG: observational submit-path CPU cost, never authority. */
static atomic64_t gpu_m4_provider_stat[6]; /* prepare n/ns/max, window n/ns/max */

static void gpu_m4_provider_stat_add(u32 base, u64 start)
{
	u64 d = ktime_get_ns() - start;

	atomic64_inc(&gpu_m4_provider_stat[base]);
	atomic64_add(d, &gpu_m4_provider_stat[base + 1]);
	if (d > (u64)atomic64_read(&gpu_m4_provider_stat[base + 2]))
		atomic64_set(&gpu_m4_provider_stat[base + 2], d);
}

void gpu_m4_drm_provider_job_stats(u64 out[6])
{
	for (u32 i = 0; i < 6; i++)
		out[i] = atomic64_read(&gpu_m4_provider_stat[i]);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_stats);

/* REMOVE-DEBUG: proof reuse and submit-path mutex waits, never authority. */
static atomic64_t gpu_m4_provider_proof_stat[GPU_M4_DRM_PROVIDER_PROOF_STATS];

static void gpu_m4_provider_proof_add(enum gpu_m4_drm_provider_proof_stat stat, u64 value)
{
	atomic64_add(value, &gpu_m4_provider_proof_stat[stat]);
}

static void gpu_m4_provider_proof_max(enum gpu_m4_drm_provider_proof_stat stat, u64 value)
{
	if (value > (u64)atomic64_read(&gpu_m4_provider_proof_stat[stat]))
		atomic64_set(&gpu_m4_provider_proof_stat[stat], value);
}

void gpu_m4_drm_provider_job_proof_stats(u64 out[GPU_M4_DRM_PROVIDER_PROOF_STATS])
{
	for (u32 i = 0; i < GPU_M4_DRM_PROVIDER_PROOF_STATS; i++)
		out[i] = atomic64_read(&gpu_m4_provider_proof_stat[i]);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_proof_stats);

/* REMOVE-DEBUG: early first windows and dependency-ready latency. */
static atomic64_t gpu_m4_provider_early_stat[GPU_M4_DRM_PROVIDER_EARLY_STATS];
static atomic64_t gpu_m4_provider_ready_stat[2][GPU_M4_DRM_PROVIDER_READY_PHASES][3];

static void gpu_m4_provider_early_add(enum gpu_m4_drm_provider_early_stat stat, u64 value)
{
	atomic64_add(value, &gpu_m4_provider_early_stat[stat]);
}

static void gpu_m4_provider_ready_add(bool early, enum gpu_m4_drm_provider_ready_phase phase,
	u64 ready)
{
	atomic64_t *stat = gpu_m4_provider_ready_stat[early][phase];
	u64 d = ktime_get_ns() - ready;

	atomic64_inc(&stat[0]);
	atomic64_add(d, &stat[1]);
	if (d > (u64)atomic64_read(&stat[2]))
		atomic64_set(&stat[2], d);
}

/* A first window's dependency-ready (run) to its lane activation and its
 * publication; any later start of the same job is not measured again.
 */
static void gpu_m4_provider_ready_mark(struct gpu_m4_drm_provider_job *job,
	enum gpu_m4_drm_provider_ready_phase phase)
{
	if (job->window_ready_ns)
		gpu_m4_provider_ready_add(job->window_early_used, phase, job->window_ready_ns);
	if (phase == GPU_M4_DRM_PROVIDER_READY_PUBLISH)
		job->window_ready_ns = 0;
}

void gpu_m4_drm_provider_job_early_stats(u64 early[GPU_M4_DRM_PROVIDER_EARLY_STATS],
	u64 ready[2][GPU_M4_DRM_PROVIDER_READY_PHASES][3])
{
	for (u32 i = 0; i < GPU_M4_DRM_PROVIDER_EARLY_STATS; i++)
		early[i] = atomic64_read(&gpu_m4_provider_early_stat[i]);
	for (u32 k = 0; k < 2; k++)
		for (u32 i = 0; i < GPU_M4_DRM_PROVIDER_READY_PHASES; i++)
			for (u32 j = 0; j < 3; j++)
				ready[k][i][j] = atomic64_read(&gpu_m4_provider_ready_stat[k][i][j]);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_early_stats);

/* REMOVE-DEBUG (plan S0): submit sub-phase CPU cost (count/ns/max) of the
 * ordinary prepare and of each window child's build, plus the row counts
 * the per-job path carries: the captured VM inventory (what implicit sync
 * reserves) and the job's cache rows. Observational, never authority.
 * Read: /sys/module/gpu_m4_drm_provider_job/parameters/submit_phases.
 */
enum gpu_m4_provider_phase {
	GPU_M4_PROVIDER_PHASE_CAPTURE,		/* VM snapshot capture (in decode) */
	GPU_M4_PROVIDER_PHASE_DECODE,		/* copy + decode, capture included */
	GPU_M4_PROVIDER_PHASE_TS_JOIN,
	GPU_M4_PROVIDER_PHASE_STAGE,
	GPU_M4_PROVIDER_PHASE_CACHE,
	GPU_M4_PROVIDER_PHASE_ACCEPT,
	GPU_M4_PROVIDER_PHASE_RESV,		/* implicit_sync=1 only */
	GPU_M4_PROVIDER_PHASE_WINDOW_CLONE,
	GPU_M4_PROVIDER_PHASE_WINDOW_STAGE,
	GPU_M4_PROVIDER_PHASE_WINDOW_CACHE,
	GPU_M4_PROVIDER_PHASE_WINDOW_ACCEPT,
	GPU_M4_PROVIDER_PHASE_WINDOW_SEAL,
	GPU_M4_PROVIDER_PHASE_VM_ROWS,		/* values, not ns */
	GPU_M4_PROVIDER_PHASE_CACHE_ROWS,
	/* Public jobs whose captured VM holds exported/imported dma-bufs (any
	 * access, and writable): count = such jobs, total/max = those rows.
	 */
	GPU_M4_PROVIDER_PHASE_SHARED_ROWS,
	GPU_M4_PROVIDER_PHASE_SHARED_WRITE_ROWS,
	GPU_M4_PROVIDER_PHASE_IMPLICIT_JOBS,	/* jobs prepared with implicit_sync */
	GPU_M4_PROVIDER_PHASES,
};

/* REMOVE-DEBUG (S3c): jobs accepted on each path, and tree-path guard drops
 * at retirement versus kept by quarantine. Read: vm_paths.
 */
static atomic64_t gpu_m4_provider_path_stat[4]; /* tree, version, guard_retired, guard_kept */

static int __maybe_unused gpu_m4_provider_paths_get(char *buffer, const struct kernel_param *kp)
{
	(void)kp;
	return scnprintf(buffer, PAGE_SIZE, "tree=%lld version=%lld guard_retired=%lld guard_kept=%lld\n",
		(long long)atomic64_read(&gpu_m4_provider_path_stat[0]),
		(long long)atomic64_read(&gpu_m4_provider_path_stat[1]),
		(long long)atomic64_read(&gpu_m4_provider_path_stat[2]),
		(long long)atomic64_read(&gpu_m4_provider_path_stat[3]));
}

/* Exported through debugfs (gpu_m4_drm_provider_job/vm_paths), never a
 * module parameter: live counters are not driver configuration.
 */

/* S3c: the job's VM guard after its actual successful retirement. */
static void gpu_m4_provider_guard_retired(struct gpu_m4_drm_provider_job *job)
{
	if (!job->guarded)
		return;
	job->guarded = false;
	gpu_m4_drm_provider_vm_guard_put(job->queue->vm);
	atomic64_inc(&gpu_m4_provider_path_stat[2]);
}

/*
 * S3c, at the final free: a guard still held was never published (prepare,
 * stage or accept failure, cancel, an aggregate parent that failed) and
 * drops; a published job that never retired is quarantined and keeps it, so
 * a deferred GEM/VM-close removal never runs under work of unknown state.
 */
static void gpu_m4_provider_guard_release(struct gpu_m4_drm_provider_job *job)
{
	if (!job->guarded)
		return;
	job->guarded = false;
	if (job->protected && job->state != GPU_M4_PROVIDER_RETIRED) {
		atomic64_inc(&gpu_m4_provider_path_stat[3]);
		return;
	}
	gpu_m4_drm_provider_vm_guard_put(job->queue->vm);
}

static const char *const gpu_m4_provider_phase_name[GPU_M4_PROVIDER_PHASES] = {
	"capture", "decode", "ts_join", "stage", "cache", "accept", "resv",
	"window_clone", "window_stage", "window_cache", "window_accept", "window_seal",
	"vm_rows", "cache_rows", "shared_rows", "shared_write_rows", "implicit_jobs",
};

static atomic64_t gpu_m4_provider_phase_stat[GPU_M4_PROVIDER_PHASES][3];

static void gpu_m4_provider_phase_value(enum gpu_m4_provider_phase phase, u64 value)
{
	atomic64_t *stat = gpu_m4_provider_phase_stat[phase];

	atomic64_inc(&stat[0]);
	atomic64_add(value, &stat[1]);
	if (value > (u64)atomic64_read(&stat[2]))
		atomic64_set(&stat[2], value);
}

/* REMOVE-DEBUG: the captured inventory's size, and its exported/imported
 * dma-bufs. Under explicit sync no kernel fence reaches their reservations:
 * a cross-device consumer relies on userspace's sync-file bridge. Pointer
 * reads of the immutable captured rows only; never authority.
 */
static void gpu_m4_provider_census(const struct gpu_m4_drm_vm_owner *snapshot)
{
	const struct gpu_m4_drm_resv_access *rows;
	u32 count, shared = 0, written = 0;

	rows = gpu_m4_drm_vm_owner_resv_access(snapshot, &count);
	gpu_m4_provider_phase_value(GPU_M4_PROVIDER_PHASE_VM_ROWS, count);
	for (u32 i = 0; rows && i < count; i++) {
		struct drm_gem_object *gem = rows[i].object;

		if (!gem || (!READ_ONCE(gem->dma_buf) && !drm_gem_is_imported(gem)))
			continue;
		shared++;
		written += !!(rows[i].access & DRM_ASAHI_BIND_WRITE);
	}
	if (shared)
		gpu_m4_provider_phase_value(GPU_M4_PROVIDER_PHASE_SHARED_ROWS, shared);
	if (written)
		gpu_m4_provider_phase_value(GPU_M4_PROVIDER_PHASE_SHARED_WRITE_ROWS, written);
}

/* Charge [*start, now) to phase and restart the clock there. */
static void gpu_m4_provider_phase_mark(enum gpu_m4_provider_phase phase, u64 *start)
{
	u64 now = ktime_get_ns();

	gpu_m4_provider_phase_value(phase, now - *start);
	*start = now;
}

static int __maybe_unused gpu_m4_provider_phase_get(char *buffer, const struct kernel_param *kp)
{
	int size = 0;

	(void)kp;
	for (u32 i = 0; i < GPU_M4_PROVIDER_PHASES; i++)
		size += scnprintf(buffer + size, PAGE_SIZE - size, "%s%s=%lld/%lld/%lld",
			i ? " " : "", gpu_m4_provider_phase_name[i],
			(long long)atomic64_read(&gpu_m4_provider_phase_stat[i][0]),
			(long long)atomic64_read(&gpu_m4_provider_phase_stat[i][1]),
			(long long)atomic64_read(&gpu_m4_provider_phase_stat[i][2]));
	size += scnprintf(buffer + size, PAGE_SIZE - size, "\n");
	return size;
}

/* debugfs gpu_m4_drm_provider_job/submit_phases: name=count/total/max. */

/* Transitional (plan S1): explicit synchronization only, as the working
 * driver. 0: userspace orders shared buffers with syncobjs and sync files
 * (imported into a dma-buf for any implicit consumer, e.g. KMS prepare_fb);
 * no submission reserves, fences or waits on any dma_resv, and each job's
 * cache job carries only its native/extra and timestamp rows. 1: the old
 * whole-VM implicit reservation path (one-line rollback for a consumer
 * without a sync-file bridge). Writable at runtime: sampled once when a
 * public submission is prepared, so a switch takes effect at the next job
 * boundary; that batch and every one of its children keep the mode they
 * were accepted with.
 */
static bool implicit_sync;
module_param(implicit_sync, bool, 0644);
MODULE_PARM_DESC(implicit_sync, "Transitional: 1 = also synchronize implicitly on every VM BO reservation (old path); takes effect at the next submission");

/* Early first windows (see begin_dependencies). Each registered aggregate
 * holds its materializer ref and native cells/units while its dependencies
 * may wait on ANY other job, so their number is bounded globally (one per
 * queue follows from scheduling credit one). 0 restores the lazy build.
 */
#define GPU_M4_PROVIDER_EARLY_MAX 4U
/* Window prefetch (A/B switch): while a child executes, build the next child
 * of the same public batch and enqueue it right behind it in the lane FIFO,
 * so materialization overlaps execution. 0 builds after completion.
 */
static bool window_prefetch = true;
module_param(window_prefetch, bool, 0644);
MODULE_PARM_DESC(window_prefetch, "Build and enqueue a batch's next child window while the current one executes (default 1)");
/* REMOVE-DEBUG: prefetched children enqueued, prefetches refused. */
static atomic64_t gpu_m4_provider_prefetch_stat[2];
/* Prefetch kick (A/B): rerun the materializer right after a child is enqueued
 * so its successor builds while it executes. 0: only later wakes prefetch.
 */
static bool window_prefetch_kick = true;
#ifdef MODULE
module_param(window_prefetch_kick, bool, 0644);
MODULE_PARM_DESC(window_prefetch_kick, "Prefetch the next child window right after enqueueing one (default 1)");
#endif

static unsigned int early_max = 2;
module_param(early_max, uint, 0644);
MODULE_PARM_DESC(early_max, "Public batches materializing their first window before dependencies (0 = lazy, at most 4)");
/* Leaf lock, nested inside the provider mutex. Membership: a never-run
 * aggregate (PREPARED) holding its early materializer ref.
 */
static DEFINE_SPINLOCK(gpu_m4_provider_early_lock);
static struct gpu_m4_drm_provider_job *gpu_m4_provider_early[GPU_M4_PROVIDER_EARLY_MAX];
/* 1 + ktime of the latest window capacity backpressure; 0 = none. No new
 * early window starts within the quiet period after one.
 */
static atomic64_t gpu_m4_provider_early_pressure;
#define GPU_M4_PROVIDER_EARLY_QUIET_NS (100ULL * 1000 * 1000)

static bool gpu_m4_provider_early_register_locked(struct gpu_m4_drm_provider_job *job)
{
	u32 limit = READ_ONCE(early_max), held = 0, slot = GPU_M4_PROVIDER_EARLY_MAX;

	lockdep_assert_held(gpu_m4_drm_provider_lock(job->queue->provider));
	if (job->window_early)
		return false;
	spin_lock(&gpu_m4_provider_early_lock);
	for (u32 i = 0; i < GPU_M4_PROVIDER_EARLY_MAX; i++) {
		if (gpu_m4_provider_early[i])
			held++;
		else if (slot == GPU_M4_PROVIDER_EARLY_MAX)
			slot = i;
	}
	if (held < limit && slot < GPU_M4_PROVIDER_EARLY_MAX) {
		gpu_m4_provider_early[slot] = job;
		job->window_early = true;
	}
	spin_unlock(&gpu_m4_provider_early_lock);
	return job->window_early;
}

/* Run, a drop to the lazy build or finalization: no longer blocked here. */
static void gpu_m4_provider_early_unregister_locked(struct gpu_m4_drm_provider_job *job)
{
	lockdep_assert_held(gpu_m4_drm_provider_lock(job->queue->provider));
	if (job->queue->early == job)
		job->queue->early = NULL;
	if (!job->window_early)
		return;
	spin_lock(&gpu_m4_provider_early_lock);
	for (u32 i = 0; i < GPU_M4_PROVIDER_EARLY_MAX; i++)
		if (gpu_m4_provider_early[i] == job)
			gpu_m4_provider_early[i] = NULL;
	spin_unlock(&gpu_m4_provider_early_lock);
	job->window_early = false;
	job->window_evict = false;
}

/* A dependency-ready window was refused native capacity. Ask every OTHER
 * registered aggregate of this provider to drop its unsealed child (cells,
 * units and charges return through the ordinary unpublished abort and the
 * reaper) and open a quiet period without new early windows. Those batches
 * then build lazily once dependency-ready, exactly as without this path,
 * so a pre-built child can never hold capacity its own dependency needs.
 */
static void gpu_m4_provider_early_evict(struct gpu_m4_drm_provider_job *parent)
{
	struct mutex *lock = gpu_m4_provider_job_lock(parent);

	atomic64_set(&gpu_m4_provider_early_pressure, ktime_get_ns() + 1);
	mutex_lock(lock);
	spin_lock(&gpu_m4_provider_early_lock);
	for (u32 i = 0; i < GPU_M4_PROVIDER_EARLY_MAX; i++) {
		struct gpu_m4_drm_provider_job *job = gpu_m4_provider_early[i];

		if (!job || job == parent || job->window_evict ||
		    job->queue->provider != parent->queue->provider)
			continue;
		job->window_evict = true;
		queue_work(system_unbound_wq, &job->window_work);
	}
	spin_unlock(&gpu_m4_provider_early_lock);
	mutex_unlock(lock);
}

/* A provider fault or VM-local error parks no timer or callback for a built
 * early child: wake each registered materializer of that scope so it drops
 * the child (abort, signal and puts outside every lock, never success) and
 * keeps only its ref for run or release. Registration retains each parent.
 */
void gpu_m4_drm_provider_job_error_wake_locked(struct gpu_m4_drm_provider *provider,
	struct gpu_m4_drm_provider_vm *vm)
{
	lockdep_assert_held(gpu_m4_drm_provider_lock(provider));
	spin_lock(&gpu_m4_provider_early_lock);
	for (u32 i = 0; i < GPU_M4_PROVIDER_EARLY_MAX; i++) {
		struct gpu_m4_drm_provider_job *job = gpu_m4_provider_early[i];

		if (job && job->queue->provider == provider && (!vm || job->queue->vm == vm))
			queue_work(system_unbound_wq, &job->window_work);
	}
	spin_unlock(&gpu_m4_provider_early_lock);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_error_wake_locked);

/* Before every reassignment of job->version or job->batch.vm_snapshot. */
static void gpu_m4_provider_job_uncover(struct gpu_m4_drm_provider_job *job)
{
	job->covered_version = NULL;
	job->covered_snapshot = NULL;
	job->ts_memo_valid = false;
}

static void gpu_m4_provider_window_put_work(struct work_struct *work);
static void gpu_m4_provider_window_retry(struct work_struct *work);

/* Native cell credits are returned by the runtime's deferred reaper only after
 * old holders drain, or by in-flight/borrowed units at their receipt. The
 * runtime answers -EAGAIN only while such a return is under way, so a
 * dependency-ready child waits for it without a bound instead of failing its
 * public batch: capacity is never fatal while progress is possible. Closure,
 * provider failure (a hung holder ends in the lane's fail-stop) and timeout
 * are rechecked on every rerun. After this many passes the interval grows.
 */
/* One jiffy (4 ms at HZ=250): a return under way completes within one
 * reaper pass; a longer sleep only idles the GPU (r99: 14-20 ms gaps).
 */
#define GPU_M4_PROVIDER_WINDOW_RETRY_MS 1U
#define GPU_M4_PROVIDER_WINDOW_RETRY_SLOW_MS 100U
#define GPU_M4_PROVIDER_WINDOW_RETRIES 2500U
/* A refused charge first reclaims through the runtime's mandatory collector
 * (cold native units, provider garbage, construction caches under pressure).
 * Half a second of such passes; a refusal that persists is genuine.
 */
#define GPU_M4_PROVIDER_CAPACITY_RETRIES 125U

static const char *gpu_m4_provider_fence_driver(struct dma_fence *fence)
{
	(void)fence;
	return "apple-m4-private";
}

static const char *gpu_m4_provider_fence_timeline(struct dma_fence *fence)
{
	(void)fence;
	return "qualified-retirement";
}

static void gpu_m4_provider_fence_release(struct dma_fence *fence)
{
	/* base is the first member; honor dma-fence's RCU readers on final put. */
	dma_fence_free(fence);
	/* RCU freeing is core-owned. No module callback remains after release;
	 * external resv/syncobj refs, not the provider, own this ops lifetime.
	 */
	module_put(THIS_MODULE);
}

static const struct dma_fence_ops gpu_m4_provider_fence_ops = {
	.get_driver_name = gpu_m4_provider_fence_driver,
	.get_timeline_name = gpu_m4_provider_fence_timeline,
	.release = gpu_m4_provider_fence_release,
};

static void gpu_m4_provider_complete_locked(struct gpu_m4_drm_provider_job *job, int error)
{
	struct gpu_m4_drm_provider_fence *fence = job->retirement;
	unsigned long flags;

	lockdep_assert_held(gpu_m4_provider_job_lock(job));
	/* An admitted public job that fails for ANY reason (local capacity or
	 * allocation, closed queue, VM-local stop, shared-domain failure)
	 * latches that error on its own VM before its fence can signal: Mesa
	 * must report DEVICE_LOST, never success for work that did not run.
	 * An internal child window reaches userspace only through its public
	 * batch, which latches when it fails; an early child dropped for a
	 * later rebuild fails nothing. A failed domain also latches children.
	 */
	if (error && (!job->window_parent ||
		      gpu_m4_drm_provider_error_locked(job->queue->provider))) {
		/* REMOVE-DEBUG: a public job failing on a healthy provider. */
		if (!gpu_m4_drm_provider_error_locked(job->queue->provider) && !fence->ready)
			pr_warn_ratelimited("GPU public job failed locally: error=%d state=%u\n",
				error, job->state);
		gpu_m4_drm_provider_vm_quarantine_locked(job->queue->vm, error);
	}
	spin_lock_irqsave(&fence->lock, flags);
	if (!fence->ready) {
		fence->error = error;
		fence->ready = true;
	}
	spin_unlock_irqrestore(&fence->lock, flags);
	/* Unpublished child failures and protected faults have no successful native
	 * receipt callback. Their actual error schedules aggregate disposal; success
	 * waits for window_complete AFTER post-CORE runtime bookkeeping instead.
	 * Only the attached child may wake its parent: the attachment's child ref
	 * retains the child, whose parent ref retains the work item.
	 */
	if (error && job->window_parent && (job->window_parent->active_window == job ||
					     job->window_parent->next_window == job))
		queue_work(system_unbound_wq, &job->window_parent->window_work);
}

/* REMOVE-DEBUG client pacing aggregates: [count, ns, max]. */
static atomic64_t gpu_m4_provider_pacing[GPU_M4_DRM_PROVIDER_PACINGS][3];

static void gpu_m4_provider_pacing_add(enum gpu_m4_drm_provider_pacing stat, u64 ns)
{
	atomic64_t *s = gpu_m4_provider_pacing[stat];

	atomic64_inc(&s[0]);
	atomic64_add(ns, &s[1]);
	if (ns > (u64)atomic64_read(&s[2]))
		atomic64_set(&s[2], ns);
}

void gpu_m4_drm_provider_job_pacing_stats(u64 out[GPU_M4_DRM_PROVIDER_PACINGS][3])
{
	for (u32 i = 0; i < GPU_M4_DRM_PROVIDER_PACINGS; i++)
		for (u32 k = 0; k < 3; k++)
			out[i][k] = atomic64_read(&gpu_m4_provider_pacing[i][k]);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_pacing_stats);

static int gpu_m4_provider_signal_one(struct gpu_m4_drm_provider_job *job, bool *now)
{
	struct gpu_m4_drm_provider_fence *fence = job->retirement;
	unsigned long flags;
	int ret = 0;

	*now = false;
	spin_lock_irqsave(&fence->lock, flags);
	if (!fence->ready) {
		ret = -EAGAIN;
	} else if (!dma_fence_is_signaled(&fence->base)) {
		if (fence->error)
			dma_fence_set_error(&fence->base, fence->error);
		dma_fence_signal_locked(&fence->base);
		*now = !fence->error;
	}
	spin_unlock_irqrestore(&fence->lock, flags);
	return ret;
}

/* After job's own signal: a final child completed its batch at its receipt
 * (two credits), so the public fence signals with it, not one materializer
 * hop later (the parent is held by this child's ref; its finalization later
 * finds it signalled). REMOVE-DEBUG pacing of a public success signal.
 */
static void gpu_m4_provider_signal_window(struct gpu_m4_drm_provider_job *job, int ret, bool now)
{
	struct gpu_m4_drm_provider_job *parent = job->window_parent, *pub = parent ?: job;
	u64 t;

	if (!ret && parent && parent->queue->credit_limit > 1 && parent->retirement->ready)
		WARN_ON(gpu_m4_provider_signal_one(parent, &now));
	else if (parent)
		now = false;
	if (!now)
		return;
	t = ktime_get_ns();
#ifdef CONFIG_DEBUG_FS
	gpu_m4_trace(GPU_M4_TRACE_SIGNAL, 0, (u64)(uintptr_t)pub->queue->runtime_queue,
		(u64)(uintptr_t)pub, 0, 0);
#endif
	atomic64_set(&pub->queue->signal_ns, t);
	if (pub->final_ns && t >= pub->final_ns)
		gpu_m4_provider_pacing_add(GPU_M4_DRM_PROVIDER_PACING_RETIRE_SIGNAL, t - pub->final_ns);
}

int gpu_m4_drm_provider_job_signal(struct gpu_m4_drm_provider_job *job)
{
	bool now;
	int ret;

	if (!job)
		return -EINVAL;
	lockdep_assert_not_held(gpu_m4_provider_job_lock(job));
	ret = gpu_m4_provider_signal_one(job, &now);
	gpu_m4_provider_signal_window(job, ret, now);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_signal);

static int gpu_m4_provider_retirement_init(struct gpu_m4_drm_provider_job *job)
{
	job->retirement = kzalloc(sizeof(*job->retirement), GFP_KERNEL);
	if (!job->retirement)
		return -ENOMEM;
	/* NULL for built-in: try_module_get succeeds and module_put is a
	 * no-op. A modular fence independently pins its callback text.
	 */
	if (!try_module_get(THIS_MODULE)) {
		kfree(job->retirement);
		job->retirement = NULL;
		return -ENODEV;
	}
	spin_lock_init(&job->retirement->lock);
	dma_fence_init(&job->retirement->base, &gpu_m4_provider_fence_ops,
		&job->retirement->lock, dma_fence_context_alloc(1), 1);
	return 0;
}

void gpu_m4_drm_provider_job_view(struct gpu_m4_drm_provider_job *job,
				struct gpu_m4_drm_provider_job_view *view)
{
	if (!job || !view)
		return;
	*view = (struct gpu_m4_drm_provider_job_view) {
		.version = job->version, .snapshot = job->batch.vm_snapshot,
		.batch = &job->batch, .command_bytes = job->bytes,
		.command_size = job->size, .runtime_queue = job->queue->runtime_queue,
		.runtime_job = job->runtime_job, .stage_owner = job->stage_owner,
		.window_parent = job->window_parent, .command_first = job->command_first,
		.command_total = job->command_total ?: job->batch.count, .aggregate = job->aggregate,
	};
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_view);

static int gpu_m4_provider_timestamp_batch_locked(struct gpu_m4_drm_provider_job *job)
{
	int ret;

	if (IS_ERR_OR_NULL(job) || IS_ERR_OR_NULL(job->queue))
		return -EINVAL;
	lockdep_assert_held(gpu_m4_provider_job_lock(job));
	if (!refcount_read(&job->refs) || IS_ERR_OR_NULL(job->version) ||
	    IS_ERR_OR_NULL(job->batch.vm_snapshot) || !job->commands ||
	    job->batch.commands != job->commands || !job->batch.count ||
	    job->batch.count > GPU_M4_DRM_JOB_MAX_COMMANDS ||
	    job->batch.count > job->queue->params.max_commands_per_submission)
		return -EINVAL;
	ret = gpu_m4_drm_provider_error_locked(job->queue->provider);
	if (ret)
		return gpu_m4_drm_job_status(ret);
	if (gpu_m4_drm_vm_owner_generation(job->batch.vm_snapshot) !=
	    gpu_m4_drm_provider_version_generation(job->version))
		return -ESTALE;
	/* Also establishes exact frontend VM identity and preserves the final
	 * snapshot's coverage of every captured native version access. Timestamp
	 * extras are then checked independently below; they need not be user VAs.
	 * The complete proof over the live immutable arrays runs once per held
	 * version/snapshot pair; the memo names exactly that pair (see above).
	 */
	if (job->covered_version == job->version &&
	    job->covered_snapshot == job->batch.vm_snapshot) {
		gpu_m4_provider_proof_add(GPU_M4_DRM_PROVIDER_PROOF_COVERED_HITS, 1);
		return 0;
	}
	gpu_m4_provider_job_uncover(job);
	gpu_m4_provider_proof_add(GPU_M4_DRM_PROVIDER_PROOF_COVERED_PROOFS, 1);
	if (!gpu_m4_drm_provider_version_native_covered_locked(job->version,
							      job->batch.vm_snapshot))
		return -EACCES;
	job->covered_version = job->version;
	job->covered_snapshot = job->batch.vm_snapshot;
	return 0;
}

bool gpu_m4_drm_provider_job_native_covered_locked(struct gpu_m4_drm_provider_job *job,
	struct gpu_m4_drm_provider_version *version, const struct gpu_m4_drm_vm_owner *snapshot)
{
	if (IS_ERR_OR_NULL(job) || IS_ERR_OR_NULL(job->queue) || IS_ERR_OR_NULL(version) ||
	    IS_ERR_OR_NULL(snapshot))
		return false;
	lockdep_assert_held(gpu_m4_provider_job_lock(job));
	if (version != job->version || snapshot != job->batch.vm_snapshot)
		return gpu_m4_drm_provider_version_native_covered_locked(version, snapshot);
	if (job->covered_version == version && job->covered_snapshot == snapshot) {
		gpu_m4_provider_proof_add(GPU_M4_DRM_PROVIDER_PROOF_COVERED_HITS, 1);
		return true;
	}
	gpu_m4_provider_job_uncover(job);
	gpu_m4_provider_proof_add(GPU_M4_DRM_PROVIDER_PROOF_COVERED_PROOFS, 1);
	if (!gpu_m4_drm_provider_version_native_covered_locked(version, snapshot))
		return false;
	job->covered_version = version;
	job->covered_snapshot = snapshot;
	return true;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_native_covered_locked);

/* Only caller-owned decoded command storage is inspected. The runtime's opaque
 * alias is compared as an identity, never cast or dereferenced by these APIs.
 */
static int gpu_m4_provider_timestamp_view_locked(struct gpu_m4_drm_provider_job *job,
	const struct gpu_m4_drm_job_timestamp *ts, u32 command, u32 index,
	struct gpu_m4_drm_provider_job_timestamp_view *view)
{
	struct gpu_m4_drm_provider_timestamp *lease = ts->lease;
	struct gpu_m4_drm_provider_job *origin = job->window_parent ?: job;
	struct gpu_m4_drm_bo_pin *pin;
	int ret;

	if (!lease)
		return (!ts->backend && !ts->object && !ts->offset && !ts->object_offset &&
			!ts->firmware_va) ? -ENOENT : -EINVAL;
	if (job->window_parent) {
		const struct gpu_m4_drm_job_command *original;
		const struct gpu_m4_drm_job_timestamp *captured;
		u32 slots;

		if (!origin->aggregate || !refcount_read(&origin->refs) ||
		    command >= job->batch.count || job->command_first >= origin->batch.count ||
		    command >= origin->batch.count - job->command_first ||
		    origin->batch.commands != origin->commands)
			return -ESTALE;
		original = &origin->commands[job->command_first + command];
		if (original->type != job->commands[command].type)
			return -ESTALE;
		captured = original->type == DRM_ASAHI_CMD_COMPUTE ?
			original->compute.timestamps : original->render.timestamps;
		slots = original->type == DRM_ASAHI_CMD_COMPUTE ?
			GPU_M4_DRM_JOB_TIMESTAMPS : GPU_M4_DRM_RENDER_TIMESTAMPS;
		if (index >= slots || ts != (original->type == DRM_ASAHI_CMD_COMPUTE ?
		    &job->commands[command].compute.timestamps[index] :
		    &job->commands[command].render.timestamps[index]) ||
		    memcmp(ts, &captured[index], sizeof(*ts)))
			return -ESTALE;
	}
	if (IS_ERR(lease) || lease->job != origin || IS_ERR_OR_NULL(lease->object) ||
	    IS_ERR_OR_NULL(lease->alias) || !ts->backend || !ts->object ||
	    ts->object_offset != lease->object_offset || ts->firmware_va != lease->firmware_va ||
	    (ts->object_offset & 7) || !gpu_m4_drm_job_timestamp_range(ts->firmware_va))
		return -EINVAL;
	ret = gpu_m4_drm_provider_object_origin_locked(job->queue->provider,
						     job->version, lease->object);
	if (ret)
		return gpu_m4_drm_job_status(ret);
	pin = gpu_m4_drm_provider_object_native(lease->object);
	if (IS_ERR_OR_NULL(pin) || gpu_m4_drm_provider_object_identity(lease->object) != ts->object ||
	    gpu_m4_drm_bo_pin_object(pin) != ts->object ||
	    ts->object_offset >= gpu_m4_drm_bo_pin_size(pin) ||
	    gpu_m4_drm_bo_pin_size(pin) - ts->object_offset < 8)
		return -EPERM;
	*view = (struct gpu_m4_drm_provider_job_timestamp_view) {
		.runtime_alias = lease->alias, .object = lease->object, .gem = ts->object,
		.object_offset = ts->object_offset, .firmware_va = ts->firmware_va,
		.command_index = command, .timestamp_index = index,
		.original_command_index = job->command_first + command,
	};
	return 0;
}

static int gpu_m4_provider_timestamp_slots(const struct gpu_m4_drm_job_command *command,
	const struct gpu_m4_drm_job_timestamp **slots, u32 *count)
{
	if (command->type == DRM_ASAHI_CMD_COMPUTE) {
		*slots = command->compute.timestamps;
		*count = GPU_M4_DRM_JOB_TIMESTAMPS;
	} else if (command->type == DRM_ASAHI_CMD_RENDER) {
		*slots = command->render.timestamps;
		*count = GPU_M4_DRM_RENDER_TIMESTAMPS;
	} else {
		return -EINVAL;
	}
	return 0;
}

/* Kernel only: host fixtures exercise the read-only complete scans. */
#ifdef MODULE
static bool timestamp_view_memo = true;
module_param(timestamp_view_memo, bool, 0644);
MODULE_PARM_DESC(timestamp_view_memo, "Reuse a job's complete timestamp view scan for the same covered version (default 1)");
#endif

#ifdef MODULE
static int gpu_m4_provider_timestamp_memo_build(struct gpu_m4_drm_provider_job *job,
	u32 selected, struct gpu_m4_drm_provider_job_timestamp_view *view);
#endif

static int gpu_m4_provider_timestamp_scan_locked(struct gpu_m4_drm_provider_job *job,
	u32 selected, struct gpu_m4_drm_provider_job_timestamp_view *view, u32 *count)
{
	struct gpu_m4_drm_provider_job_timestamp_view result = { 0 };
	u32 held = 0;
	int ret = gpu_m4_provider_timestamp_batch_locked(job);

	if (ret)
		return ret;
	/* batch_locked just re-proved (or kept) the covered pair the memo
	 * was built against.
	 */
#ifdef MODULE
	if (!job->ts_memo_valid && READ_ONCE(timestamp_view_memo) && (view || count)) {
		struct gpu_m4_drm_provider_job_timestamp_view first;

		ret = gpu_m4_provider_timestamp_memo_build(job, 0, &first);
		if (ret && ret != -ENOENT && ret != -ENOBUFS)
			return ret;
	}
	if (job->ts_memo_valid && READ_ONCE(timestamp_view_memo)) {
		if (view && selected >= job->ts_memo_count)
			return -ENOENT;
		if (view)
			*view = job->ts_memo[selected];
		if (count)
			*count = job->ts_memo_count;
		return 0;
	}
#endif
	for (u32 i = 0; i < job->batch.count; i++) {
		const struct gpu_m4_drm_job_timestamp *slots;
		u32 n;

		ret = gpu_m4_provider_timestamp_slots(&job->batch.commands[i], &slots, &n);
		if (ret)
			return ret;
		for (u32 j = 0; j < n; j++) {
			struct gpu_m4_drm_provider_job_timestamp_view one;

			ret = gpu_m4_provider_timestamp_view_locked(job, &slots[j], i, j, &one);
			if (ret == -ENOENT && !slots[j].lease)
				continue;
			if (ret)
				return ret;
			if (held++ == selected) {
				result = one;
				/* A selected view needs no later slot (count does). */
				if (view && !count)
					goto found;
			}
		}
	}
found:
	if (view && selected >= held)
		return -ENOENT;
	if (view)
		*view = result;
	if (count)
		*count = held;
	return 0;
}

#ifdef MODULE
/* One complete scan into the memo (the caller proved the batch); selected's
 * view returned.
 */
static int gpu_m4_provider_timestamp_memo_build(struct gpu_m4_drm_provider_job *job,
	u32 selected, struct gpu_m4_drm_provider_job_timestamp_view *view)
{
	u32 capacity = 0, held = 0;
	int ret;

	for (u32 i = 0; i < job->batch.count; i++) {
		const struct gpu_m4_drm_job_timestamp *slots;
		u32 n;

		ret = gpu_m4_provider_timestamp_slots(&job->batch.commands[i], &slots, &n);
		if (ret)
			return ret;
		capacity += n;
	}
	job->ts_memo_valid = false;
	if (capacity > job->ts_memo_capacity) {
		struct gpu_m4_drm_provider_job_timestamp_view *memo =
			kcalloc(capacity, sizeof(*memo), GFP_KERNEL);

		if (!memo)
			return -ENOBUFS; /* the caller takes its complete scan */
		kfree(job->ts_memo);
		job->ts_memo = memo;
		job->ts_memo_capacity = capacity;
	}
	for (u32 i = 0; i < job->batch.count; i++) {
		const struct gpu_m4_drm_job_timestamp *slots;
		u32 n;

		ret = gpu_m4_provider_timestamp_slots(&job->batch.commands[i], &slots, &n);
		if (ret)
			return ret;
		for (u32 k = 0; k < n; k++) {
			ret = gpu_m4_provider_timestamp_view_locked(job, &slots[k], i, k,
				&job->ts_memo[held]);
			if (ret == -ENOENT && !slots[k].lease)
				continue;
			if (ret)
				return ret;
			held++;
		}
	}
	job->ts_memo_count = held;
	job->ts_memo_valid = true;
	if (selected >= held)
		return -ENOENT;
	*view = job->ts_memo[selected];
	return 0;
}
#endif

int gpu_m4_drm_provider_job_timestamp_count_locked(struct gpu_m4_drm_provider_job *job,
	u32 *count)
{
	if (!count)
		return -EINVAL;
	return gpu_m4_provider_timestamp_scan_locked(job, 0, NULL, count);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_timestamp_count_locked);

int gpu_m4_drm_provider_job_timestamp_at_locked(struct gpu_m4_drm_provider_job *job,
	u32 index, struct gpu_m4_drm_provider_job_timestamp_view *view)
{
	if (!view)
		return -EINVAL;
	return gpu_m4_provider_timestamp_scan_locked(job, index, view, NULL);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_timestamp_at_locked);

static int gpu_m4_provider_timestamp_covered_locked(struct gpu_m4_drm_provider_job *job,
	const struct gpu_m4_drm_provider_job_timestamp_view *view, bool retired)
{
	const struct gpu_m4_drm_cache_access *cache;
	const struct gpu_m4_drm_resv_access *resv;
	struct gpu_m4_drm_bo_pin *pin = gpu_m4_drm_provider_object_native(view->object);
	const u32 allowed = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE;
	u32 count, resv_count = 0;
	bool cache_found = false, resv_found = !job->resv;
	int ret;

	/* An explicit-sync job (no reservation owner) needs no reservation row:
	 * the snapshot and cache rows below remain the destination's authority.
	 */
	cache = gpu_m4_drm_vm_owner_cache_access(job->batch.vm_snapshot, &count);
	resv = job->resv ? gpu_m4_drm_vm_owner_resv_access(job->batch.vm_snapshot, &resv_count) : NULL;
	if (!cache || !count || count > GPU_M4_DRM_MAX_BINDS ||
	    (job->resv && (!resv || !resv_count || resv_count > GPU_M4_DRM_MAX_BINDS)))
		return -EINVAL;
	for (u32 i = 0; i < count; i++) {
		const struct gpu_m4_drm_cache_access *a = &cache[i];

		if (a->pin != pin || !a->access || (a->access & ~allowed) ||
		    !(a->access & DRM_ASAHI_BIND_WRITE) || !a->size ||
		    a->offset >= gpu_m4_drm_bo_pin_size(pin) ||
		    a->size > gpu_m4_drm_bo_pin_size(pin) - a->offset ||
		    view->object_offset < a->offset)
			continue;
		if (view->object_offset - a->offset < a->size &&
		    a->size - (view->object_offset - a->offset) >= 8)
			cache_found = true;
	}
	for (u32 i = 0; !resv_found && i < resv_count; i++)
		if (resv[i].object == view->gem && resv[i].access &&
		    !(resv[i].access & ~allowed) && (resv[i].access & DRM_ASAHI_BIND_WRITE))
			resv_found = true;
	if (!cache_found || !resv_found)
		return -EACCES;
	ret = gpu_m4_drm_cache_job_check_access(job->cache, pin, view->object_offset,
					      8, DRM_ASAHI_BIND_WRITE, retired);
	return gpu_m4_drm_job_status(ret);
}

int gpu_m4_drm_provider_job_timestamp_check_locked(struct gpu_m4_drm_provider_job *job,
	void *runtime_alias, bool retired)
{
	bool found = false;
	int ret;

	if (IS_ERR_OR_NULL(runtime_alias))
		return -EINVAL;
	ret = gpu_m4_provider_timestamp_batch_locked(job);
	if (ret)
		return ret;
	/* Closing an accepted queue/VM is not cancellation of its published work.
	 * These owned software states, not current frontend openness, are authority.
	 */
	if (!job->accepted || !job->protected || IS_ERR_OR_NULL(job->runtime_job) ||
	    IS_ERR_OR_NULL(job->cache) || IS_ERR(job->resv) ||
	    IS_ERR_OR_NULL(job->finished) || IS_ERR_OR_NULL(job->retirement) ||
	    job->state != (retired ? GPU_M4_PROVIDER_RETIRED : GPU_M4_PROVIDER_PUBLISHED) ||
	    job->retirement->error || job->retirement->ready != retired)
		return -EINVAL;
	for (u32 i = 0; i < job->batch.count; i++) {
		const struct gpu_m4_drm_job_timestamp *slots;
		u32 n;

		ret = gpu_m4_provider_timestamp_slots(&job->batch.commands[i], &slots, &n);
		if (ret)
			return ret;
		for (u32 j = 0; j < n; j++) {
			struct gpu_m4_drm_provider_job_timestamp_view view;

			ret = gpu_m4_provider_timestamp_view_locked(job, &slots[j], i, j, &view);
			if (ret == -ENOENT && !slots[j].lease)
				continue;
			if (ret)
				return ret;
			if (view.runtime_alias != runtime_alias)
				continue;
			/* The same canonical alias may have multiple held slot references.
			 * Every matching eight-byte destination must satisfy this phase.
			 */
			ret = gpu_m4_provider_timestamp_covered_locked(job, &view, retired);
			if (ret)
				return ret;
			found = true;
		}
	}
	return found ? 0 : -ENOENT;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_timestamp_check_locked);

int gpu_m4_drm_provider_job_check_owner_locked(struct gpu_m4_drm_provider_job *job,
	struct gpu_m4_drm_provider *expected_provider)
{
	int ret;

	if (IS_ERR_OR_NULL(job) || IS_ERR_OR_NULL(expected_provider))
		return -EINVAL;
	lockdep_assert_held(gpu_m4_drm_provider_lock(expected_provider));
	if (IS_ERR_OR_NULL(job->queue))
		return -EINVAL;
	if (job->queue->provider != expected_provider)
		return -EPERM;
	if (!refcount_read(&job->refs) || !kref_read(&job->queue->ref))
		return -ENOENT;
	ret = gpu_m4_drm_provider_error_locked(expected_provider);
	return gpu_m4_drm_job_status(ret);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_check_owner_locked);

int gpu_m4_drm_provider_job_check_execution_locked(struct gpu_m4_drm_provider_job *job,
	struct gpu_m4_drm_provider *expected_provider, bool published)
{
	int ret = gpu_m4_drm_provider_job_check_owner_locked(job, expected_provider);

	if (ret)
		return ret;
	/* The lane's timeout observer must inspect the actual core phase, not
	 * infer readiness from !uncertain (which also admits failed jobs).
	 * Accepted captures keep this identity after their namespace closes.
	 */
	if (!job->accepted || IS_ERR_OR_NULL(job->runtime_job) ||
	    IS_ERR_OR_NULL(job->cache) || IS_ERR(job->resv) ||
	    IS_ERR_OR_NULL(job->finished) || IS_ERR_OR_NULL(job->retirement) ||
	    job->state != (published ? GPU_M4_PROVIDER_PUBLISHED : GPU_M4_PROVIDER_READY) ||
	    job->protected != published || job->retirement->ready || job->retirement->error)
		return -ESTALE;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_check_execution_locked);

int gpu_m4_drm_provider_job_check_accepting_locked(struct gpu_m4_drm_provider_job *job,
	struct gpu_m4_drm_provider *expected_provider)
{
	int ret = gpu_m4_drm_provider_job_check_owner_locked(job, expected_provider);

	if (ret)
		return ret;
	if (job->aggregate)
		return -EOPNOTSUPP;
	if (job->state != GPU_M4_PROVIDER_PREPARED || job->accepted || job->protected ||
	    IS_ERR_OR_NULL(job->cache) || job->finished || job->resv || job->runtime_job)
		return -EBUSY;
	if (job->queue->closed)
		return -ECANCELED;
	return gpu_m4_provider_timestamp_batch_locked(job);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_check_accepting_locked);

/* One snapshot cache row covering [offset, offset + size) of this exact pin
 * with at least access; one reservation row of the GEM with at least access.
 */
static bool gpu_m4_provider_cache_covers(const struct gpu_m4_drm_cache_access *a,
	const struct gpu_m4_drm_bo_pin *pin, u64 bound, u64 offset, u64 size, u32 access)
{
	const u32 allowed = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE;

	if (a->pin != pin || !a->access || (a->access & ~allowed) || !a->size ||
	    a->offset >= bound || a->size > bound - a->offset ||
	    (a->access & access) != access || offset < a->offset)
		return false;
	return offset - a->offset < a->size && size <= a->size - (offset - a->offset);
}

static bool gpu_m4_provider_resv_covers(const struct gpu_m4_drm_resv_access *r,
	const struct drm_gem_object *gem, u32 access)
{
	const u32 allowed = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE;

	return r->object == gem && r->access && !(r->access & ~allowed) &&
		(r->access & access) == access;
}

int gpu_m4_drm_provider_job_check_access_locked(struct gpu_m4_drm_provider_job *job,
	struct gpu_m4_drm_provider *expected_provider,
	struct gpu_m4_drm_provider_object *object, u64 offset, u64 size, u32 access,
	bool retired, struct gpu_m4_drm_cache_job **cache_out)
{
	const u32 allowed = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE;
	const struct gpu_m4_drm_cache_access *cache;
	const struct gpu_m4_drm_resv_access *resv;
	struct gpu_m4_drm_bo_pin *pin;
	struct drm_gem_object *gem;
	u64 bound;
	u32 count, resv_count = 0, row;
	bool cache_found = false, resv_found;
	int ret;

	if (IS_ERR_OR_NULL(object) || !size || !access || (access & ~allowed))
		return -EINVAL;
	ret = gpu_m4_drm_provider_job_check_owner_locked(job, expected_provider);
	if (ret)
		return ret;
	ret = gpu_m4_provider_timestamp_batch_locked(job);
	if (ret)
		return ret;
	/* These are core-owned phases. In particular, runtime.publish enters after
	 * protection and before GPU producers; a retained error fence, !uncertain
	 * or cache visibility by itself cannot manufacture successful retirement.
	 */
	if (!job->accepted || !job->protected || IS_ERR_OR_NULL(job->runtime_job) ||
	    IS_ERR_OR_NULL(job->cache) || IS_ERR(job->resv) ||
	    IS_ERR_OR_NULL(job->finished) || IS_ERR_OR_NULL(job->retirement) ||
	    job->state != (retired ? GPU_M4_PROVIDER_RETIRED : GPU_M4_PROVIDER_PUBLISHED) ||
	    job->retirement->error || job->retirement->ready != retired)
		return -EINVAL;
	/* Membership must precede dereferencing a supplied opaque object. This
	 * also admits a held file-global BO absent from the lower user ledger.
	 */
	ret = gpu_m4_drm_provider_object_origin_locked(expected_provider, job->version, object);
	if (ret)
		return gpu_m4_drm_job_status(ret);
	pin = gpu_m4_drm_provider_object_native(object);
	gem = gpu_m4_drm_provider_object_identity(object);
	if (IS_ERR_OR_NULL(pin) || IS_ERR_OR_NULL(gem) || gpu_m4_drm_bo_pin_object(pin) != gem)
		return -EPERM;
	bound = gpu_m4_drm_bo_pin_size(pin);
	if (offset >= bound || size > bound - offset)
		return -EINVAL;
	/* An explicit-sync job owns no reservation set: no reservation row. */
	resv_found = !job->resv;
	cache = gpu_m4_drm_vm_owner_cache_access(job->batch.vm_snapshot, &count);
	resv = job->resv ? gpu_m4_drm_vm_owner_resv_access(job->batch.vm_snapshot, &resv_count) : NULL;
	if (!cache || !count || count > GPU_M4_DRM_MAX_BINDS ||
	    (job->resv && (!resv || !resv_count || resv_count > GPU_M4_DRM_MAX_BINDS)))
		return -EINVAL;
	/* The snapshot's own row of this native object first (O(log n) over its
	 * immutable identity order); any refusal there takes the complete scans,
	 * so the answer is exactly theirs.
	 */
	row = gpu_m4_drm_vm_owner_find(job->batch.vm_snapshot, gem);
	if (row < count)
		cache_found = gpu_m4_provider_cache_covers(&cache[row], pin, bound, offset, size,
							   access);
	if (row < resv_count)
		resv_found = gpu_m4_provider_resv_covers(&resv[row], gem, access);
	/* REMOVE-DEBUG: identity-row answers versus complete scans. */
	gpu_m4_provider_proof_add(cache_found && resv_found ?
		GPU_M4_DRM_PROVIDER_PROOF_ACCESS_INDEXED : GPU_M4_DRM_PROVIDER_PROOF_ACCESS_SCANNED, 1);
	for (u32 i = 0; !cache_found && i < count; i++)
		cache_found = gpu_m4_provider_cache_covers(&cache[i], pin, bound, offset, size,
							   access);
	/* The core prepared this reservation set from this same immutable snapshot
	 * and installed job->finished before accepting. Do not look up mutable live
	 * dma_resv fences or lock reservations inside a runtime publication check.
	 */
	for (u32 i = 0; !resv_found && i < resv_count; i++)
		resv_found = gpu_m4_provider_resv_covers(&resv[i], gem, access);
	/* S3c: a user GEM of a tree-path job has no row (its snapshot names no
	 * user BO): its live tree mapping with this access is the proof, and the
	 * coherent cache phase below the CPU-visibility one, as for explicit
	 * sync's row-less user GEMs.
	 */
	if (!cache_found && job->tree && !job->resv &&
	    gpu_m4_drm_provider_vm_object_mapped(job->queue->vm, object, offset, size, access))
		cache_found = true;
	if (!cache_found || !resv_found)
		return -EACCES;
	ret = gpu_m4_drm_cache_job_check_access(job->cache, pin, offset, size, access, retired);
	/* Explicit sync: a user-VM GEM (e.g. a WB output read back after
	 * retirement) has no row of its own. Its exact pin, span and access were
	 * just proven from this job's retained snapshot; an admitted pin is GPU
	 * coherent, so the job's cache phase (its ordering barriers) is the whole
	 * remaining CPU-visibility proof.
	 */
	if (ret && !job->resv)
		ret = gpu_m4_drm_cache_job_check_coherent(job->cache, pin, retired);
	if (ret)
		return gpu_m4_drm_job_status(ret);
	if (cache_out)
		*cache_out = job->cache;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_check_access_locked);

int gpu_m4_drm_provider_job_get_locked(struct gpu_m4_drm_provider_job *job)
{
	if (!job)
		return -EINVAL;
	lockdep_assert_held(gpu_m4_provider_job_lock(job));
	return refcount_inc_not_zero(&job->refs) ? 0 : -ENOENT;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_get_locked);

bool gpu_m4_drm_provider_job_uncertain_locked(struct gpu_m4_drm_provider_job *job)
{
	if (!job)
		return true;
	lockdep_assert_held(gpu_m4_provider_job_lock(job));
	/* The aggregate has no hardware payload. An uncertain child keeps its own
	 * transferred ref and therefore this parent; follow only the attached child.
	 */
	if (job->aggregate)
		return (job->active_window &&
			gpu_m4_drm_provider_job_uncertain_locked(job->active_window)) ||
		       (job->next_window &&
			gpu_m4_drm_provider_job_uncertain_locked(job->next_window));
	return job->protected && job->state != GPU_M4_PROVIDER_RETIRED;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_uncertain_locked);

int gpu_m4_drm_provider_job_adopt_version_locked(struct gpu_m4_drm_provider_job *job,
	struct gpu_m4_drm_provider_version *candidate)
{
	struct gpu_m4_drm_provider_version *old;
	u64 low, high;
	int ret;

	if (!job || IS_ERR_OR_NULL(candidate))
		return -EINVAL;
	lockdep_assert_held(gpu_m4_provider_job_lock(job));
	if (job->aggregate)
		return -EOPNOTSUPP;
	if (job->protected || (job->state != GPU_M4_PROVIDER_PREPARED &&
			       job->state != GPU_M4_PROVIDER_READY))
		return -EBUSY;
	ret = gpu_m4_drm_provider_error_locked(job->queue->provider);
	if (ret)
		return ret;
	if (job->queue->closed)
		return -ECANCELED;
	old = job->version;
	if (!old || !job->batch.vm_snapshot ||
	    gpu_m4_drm_provider_version_generation(candidate) !=
		gpu_m4_drm_vm_owner_generation(job->batch.vm_snapshot) ||
	    !gpu_m4_drm_provider_version_same_user_locked(candidate, old))
		return -EINVAL;
	/* The cache set was captured before runtime acceptance; reservations use
	 * that SAME immutable snapshot immediately afterward. Matching the user
	 * ledger alone cannot admit a new private native BO, a larger range or
	 * stronger access after preparation. Root-only rebuilding may reuse the
	 * already covered set, but this path never joins/replaces that snapshot.
	 * resv is intentionally not required here: accept runs before its capture.
	 */
	if (IS_ERR_OR_NULL(job->cache) ||
	    !gpu_m4_drm_provider_version_native_covered_locked(candidate,
							      job->batch.vm_snapshot))
		return -EACCES;
	/* Recheck full graph ownership, not just matching generation numbers. */
	ret = gpu_m4_drm_provider_version_roots_locked(candidate, &low, &high);
	if (ret)
		return ret;
	gpu_m4_provider_job_uncover(job);
	job->version = candidate;
	ret = gpu_m4_drm_provider_version_put_locked(old);
	if (ret)
		job->version = old; /* failed put retained old; candidate stays caller-owned */
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_adopt_version_locked);

static int gpu_m4_provider_snapshot_inner(void *cookie, void **snapshot);

/* REMOVE-DEBUG: capture timing only. */
static int gpu_m4_provider_snapshot(void *cookie, void **snapshot)
{
	u64 start = ktime_get_ns();
	int ret = gpu_m4_provider_snapshot_inner(cookie, snapshot);

	/* REMOVE-DEBUG: which decode step refused for capacity. */
	if (ret == -ENOSPC)
		pr_warn_ratelimited("GPU public prepare capacity: snapshot tree=%d\n",
			((struct gpu_m4_drm_provider_job *)cookie)->tree);
	gpu_m4_provider_phase_mark(GPU_M4_PROVIDER_PHASE_CAPTURE, &start);
	return ret;
}

static int gpu_m4_provider_snapshot_inner(void *cookie, void **snapshot)
{
	struct gpu_m4_drm_provider_job *job = cookie;
	struct gpu_m4_drm_vm_owner *owner = NULL;
	struct mutex *lock = gpu_m4_provider_job_lock(job);
	u64 start;
	int ret;

	/* S3c tree path: no capture. The VM's TREE version, a job-local wrapper
	 * of its empty snapshot and one O(1) VM guard; never the provider mutex.
	 * Named ranges are validated against the live tree (check_span).
	 */
	if (job->tree) {
		ret = gpu_m4_drm_provider_vm_guard_get(job->queue->vm);
		if (ret)
			return ret;
		ret = gpu_m4_drm_provider_vm_tree_capture(job->queue->vm, &job->version, &owner);
		if (ret) {
			gpu_m4_drm_provider_vm_guard_put(job->queue->vm);
			return ret;
		}
		job->guarded = true;
		atomic64_inc(&gpu_m4_provider_path_stat[0]);
		*snapshot = owner;
		return 0;
	}
	atomic64_inc(&gpu_m4_provider_path_stat[1]);
	/* An ordinary submit never waits for the shared mutex (held across a
	 * whole lane start/packet build) to recapture the unchanged active
	 * version: the identical capture through its published ticket. Every
	 * other case, including the first capture of each version, is locked.
	 */
	if (!job->queue->private_candidate) {
		ret = gpu_m4_drm_provider_vm_capture(job->queue->vm, &job->version, &owner);
		if (ret != -EAGAIN) {
			if (!ret) {
				gpu_m4_provider_proof_add(GPU_M4_DRM_PROVIDER_PROOF_CAPTURE_LOCKLESS, 1);
				*snapshot = owner;
			}
			return ret; /* a failure holds nothing */
		}
	}
	start = ktime_get_ns();
	mutex_lock(lock);
	/* REMOVE-DEBUG: the submit path's wait for the shared mutex. */
	start = ktime_get_ns() - start;
	gpu_m4_provider_proof_add(GPU_M4_DRM_PROVIDER_PROOF_CAPTURE_LOCKED, 1);
	gpu_m4_provider_proof_add(GPU_M4_DRM_PROVIDER_PROOF_CAPTURE_WAIT_NS, start);
	gpu_m4_provider_proof_max(GPU_M4_DRM_PROVIDER_PROOF_CAPTURE_WAIT_MAX, start);
	if (job->queue->private_candidate) {
		ret = job->queue->closed ? -ECANCELED :
			gpu_m4_drm_provider_version_check_origin_locked(job->queue->provider,
				job->queue->vm, job->queue->private_candidate);
		if (!ret)
			ret = gpu_m4_drm_provider_version_capture_locked(job->queue->private_candidate,
				&job->version, &owner);
		if (!ret && job->version != job->queue->private_candidate)
			ret = -ESTALE;
		if (!ret && gpu_m4_drm_vm_owner_generation(owner) !=
		    gpu_m4_drm_provider_version_generation(job->version))
			ret = -ESTALE;
		if (!ret && !gpu_m4_drm_provider_version_native_covered_locked(job->version, owner))
			ret = -EACCES;
		if (!ret)
			job->decode_owner = owner;
	} else {
		ret = gpu_m4_drm_provider_vm_capture_locked(job->queue->vm, &job->version, &owner);
	}
	mutex_unlock(lock);
	if (!ret) {
		*snapshot = owner;
	} else if (owner) {
		/* Even accepting rejection drops cache/snapshot pin borrowers BEFORE
		 * the exact version baseline; native finalizers stay outside locks.
		 */
		WARN_ON(gpu_m4_drm_vm_owner_release(owner));
		mutex_lock(lock);
		WARN_ON(gpu_m4_drm_provider_version_put_locked(job->version));
		gpu_m4_provider_job_uncover(job);
		job->version = NULL;
		mutex_unlock(lock);
	}
	return ret;
}

static void gpu_m4_provider_snapshot_put(void *cookie, void *snapshot)
{
	struct gpu_m4_drm_provider_job *job = cookie;
	struct mutex *lock = gpu_m4_provider_job_lock(job);
	struct gpu_m4_drm_provider_timestamp *timestamp;

	/* Keep provider object charges alive through the snapshot's final pin
	 * puts. Concurrent provider_reap must not uncharge a still pinned GEM.
	 */
	WARN_ON(gpu_m4_drm_vm_owner_release(snapshot));
	if (job->decode_owner == snapshot)
		job->decode_owner = NULL;
	while ((timestamp = job->deferred_timestamps)) {
		job->deferred_timestamps = timestamp->next;
		mutex_lock(lock);
		gpu_m4_drm_provider_object_put_locked(timestamp->object);
		mutex_unlock(lock);
		kfree(timestamp);
	}
}

static int gpu_m4_provider_private_decode_locked(struct gpu_m4_drm_provider_job *job,
	void *snapshot)
{
	struct gpu_m4_drm_provider_queue *queue = job->queue;

	lockdep_assert_held(gpu_m4_provider_job_lock(job));
	if (!queue->private_candidate || snapshot != job->decode_owner ||
	    !snapshot || job->version != queue->private_candidate)
		return -ESTALE;
	if (queue->closed)
		return -ECANCELED;
	return gpu_m4_drm_provider_version_check_origin_locked(queue->provider,
		queue->vm, job->version);
}

static int gpu_m4_provider_check_span(void *cookie, void *snapshot,
				      u64 va, u64 size, bool write)
{
	struct gpu_m4_drm_provider_job *job = cookie;
	struct mutex *lock;
	int ret;

	/* S3c: named-range validation by PTE walk of the live tree. */
	if (job->tree)
		return gpu_m4_drm_provider_vm_covers(job->queue->vm, va, size,
			write ? DRM_ASAHI_BIND_WRITE : DRM_ASAHI_BIND_READ, NULL);
	if (job->queue->private_candidate) {
		lock = gpu_m4_provider_job_lock(job);
		mutex_lock(lock);
		ret = gpu_m4_provider_private_decode_locked(job, snapshot);
		if (!ret)
			ret = gpu_m4_drm_provider_version_range_locked(job->version, va, size,
				write ? DRM_ASAHI_BIND_WRITE : DRM_ASAHI_BIND_READ);
		mutex_unlock(lock);
		return ret;
	}
	return gpu_m4_drm_vm_owner_range(snapshot, va, size,
		write ? DRM_ASAHI_BIND_WRITE : DRM_ASAHI_BIND_READ);
}

static int gpu_m4_provider_timestamp_hold(void *cookie, void *snapshot,
					const struct gpu_m4_drm_timestamp *borrowed,
					struct gpu_m4_drm_job_timestamp *owned)
{
	struct gpu_m4_drm_provider_job *job = cookie;
	struct gpu_m4_drm_provider_queue *queue = job->queue;
	struct gpu_m4_drm_provider_timestamp *timestamp;
	struct gpu_m4_drm_bo_pin *pin;
	struct mutex *lock = gpu_m4_provider_job_lock(job);
	u64 va = 0;
	int ret;

	timestamp = kzalloc(sizeof(*timestamp), GFP_KERNEL);
	if (!timestamp)
		return -ENOMEM;
	if (queue->private_candidate) {
		mutex_lock(lock);
		ret = gpu_m4_provider_private_decode_locked(job, snapshot);
		mutex_unlock(lock);
		if (ret)
			goto free;
	}
	/* No provider lock across shmem pinning/GEM reservation acquisition. */
	timestamp->object = gpu_m4_drm_provider_object_pin(queue->vm, borrowed->object);
	if (IS_ERR(timestamp->object)) {
		ret = PTR_ERR(timestamp->object);
		if (ret == -ENOSPC) /* REMOVE-DEBUG */
			pr_warn_ratelimited("GPU public prepare capacity: timestamp pin\n");
		goto free;
	}
	pin = gpu_m4_drm_provider_object_native(timestamp->object);
	if (!pin || gpu_m4_drm_provider_object_identity(timestamp->object) != borrowed->object ||
	    borrowed->object_offset >= gpu_m4_drm_bo_pin_size(pin) ||
	    gpu_m4_drm_bo_pin_size(pin) - borrowed->object_offset < 8) {
		ret = -EINVAL;
		goto put;
	}
	mutex_lock(lock);
	ret = gpu_m4_drm_provider_error_locked(queue->provider);
	if (!ret && queue->private_candidate)
		ret = gpu_m4_provider_private_decode_locked(job, snapshot);
	if (!ret && !queue->closed)
		ret = gpu_m4_drm_job_status(queue->runtime.timestamp_hold_locked(
			queue->runtime_owner, job->version, timestamp->object,
			borrowed, &timestamp->alias, &va));
	else if (!ret)
		ret = -ECANCELED;
	if (!ret && (!timestamp->alias || !gpu_m4_drm_job_timestamp_range(va)))
		ret = -EIO;
	if (ret && timestamp->alias) {
		queue->runtime.timestamp_put_locked(queue->runtime_owner, timestamp->alias);
		timestamp->alias = NULL;
	}
	mutex_unlock(lock);
	if (ret == -ENOSPC) /* REMOVE-DEBUG */
		pr_warn_ratelimited("GPU public prepare capacity: timestamp hold offset=%llu\n",
			(unsigned long long)borrowed->object_offset);
	if (ret)
		goto put;
	timestamp->job = job;
	timestamp->object_offset = borrowed->object_offset;
	timestamp->firmware_va = va;
	*owned = (struct gpu_m4_drm_job_timestamp) {
		.lease = timestamp, .backend = borrowed->backend,
		.object = borrowed->object, .offset = borrowed->offset,
		.object_offset = borrowed->object_offset, .firmware_va = va,
	};
	return 0;
put:
	mutex_lock(lock);
	gpu_m4_drm_provider_object_put_locked(timestamp->object);
	mutex_unlock(lock);
	queue->runtime.reap(queue->runtime_owner);
free:
	kfree(timestamp);
	return ret;
}

static void gpu_m4_provider_timestamp_put(void *cookie, struct gpu_m4_drm_job_timestamp *owned)
{
	struct gpu_m4_drm_provider_job *job = cookie;
	struct gpu_m4_drm_provider_queue *queue = job->queue;
	struct gpu_m4_drm_provider_timestamp *timestamp = owned->lease;
	struct mutex *lock = gpu_m4_provider_job_lock(job);

	if (job->window_parent) {
		/* The original public capture owns this lease exactly once. The child
		 * holds its parent, and drops only its copied slot and own snapshot.
		 */
		WARN_ON(timestamp->job != job->window_parent);
		return;
	}
	mutex_lock(lock);
	queue->runtime.timestamp_put_locked(queue->runtime_owner, timestamp->alias);
	mutex_unlock(lock);
	/* mixed_put drops aliases before the VM owner. Its native pin borrowers
	 * remain backed AND charged until the following snapshot_put callback.
	 */
	timestamp->next = job->deferred_timestamps;
	job->deferred_timestamps = timestamp;
}

static const struct gpu_m4_drm_job_ops gpu_m4_provider_decode_ops = {
	.snapshot_vm = gpu_m4_provider_snapshot,
	.check_span = gpu_m4_provider_check_span,
	.put_vm = gpu_m4_provider_snapshot_put,
	.hold_timestamp = gpu_m4_provider_timestamp_hold,
	.put_timestamp = gpu_m4_provider_timestamp_put,
};

/* Stop at a permission/backing/offset boundary, including SINGLE_PAGE's
 * repeated backing offset. Only the immutable captured ledger is consulted.
 */
static int gpu_m4_provider_resolve_span(void *cookie, void *snapshot,
				       enum gpu_m4_drm_render_span_role role,
				       u64 va, u64 minimum, bool write,
				       struct gpu_m4_drm_render_span *span)
{
	struct gpu_m4_drm_provider_job *job = cookie;
	struct gpu_m4_drm_vm_leaf first, next;
	u64 offset = va & (GPU_M4_DRM_PAGE_SIZE - 1), size, page;
	u32 access = write ? DRM_ASAHI_BIND_WRITE : DRM_ASAHI_BIND_READ;
	int ret;

	/* A private external leaf identity is an allocator, not a GEM. This
	 * qualifier has no render resolver and never casts its private identity.
	 */
	if (job->queue->private_candidate)
		return -EOPNOTSUPP;
	(void)role;
	/* S3c: the run at va in the live tree (same object, contiguous offsets,
	 * same access), its first page checked like a named range.
	 */
	if (job->tree) {
		struct gpu_m4_vm_tree_span run;

		ret = gpu_m4_drm_provider_vm_covers(job->queue->vm, va, 1, access, &run);
		if (ret == -ENOSPC) /* REMOVE-DEBUG */
			pr_warn_ratelimited("GPU public prepare capacity: span va=%#llx role=%d\n",
				va, role);
		if (ret)
			return ret;
		if (run.size < minimum)
			return -EACCES;
		*span = (struct gpu_m4_drm_render_span) {
			.va = va, .size = run.size, .object = (struct drm_gem_object *)run.identity,
			.object_offset = run.offset,
			.writable = !!(run.access & DRM_ASAHI_BIND_WRITE),
		};
		return 0;
	}
	ret = gpu_m4_drm_vm_owner_page(snapshot, va, access, &first);
	if (ret)
		return ret;
	size = GPU_M4_DRM_PAGE_SIZE - offset;
	page = first.va + GPU_M4_DRM_PAGE_SIZE;
	for (u32 i = 1; i < gpu_m4_drm_vm_owner_leaf_count(snapshot);
	     i++, page += GPU_M4_DRM_PAGE_SIZE) {
		ret = gpu_m4_drm_vm_owner_page(snapshot, page, access, &next);
		if (ret || next.identity != first.identity || next.access != first.access ||
		    next.object_offset != first.object_offset + i * GPU_M4_DRM_PAGE_SIZE)
			break;
		size += GPU_M4_DRM_PAGE_SIZE;
	}
	if (size < minimum)
		return -EACCES;
	*span = (struct gpu_m4_drm_render_span) {
		.va = va, .size = size, .object = (struct drm_gem_object *)first.identity,
		.object_offset = first.object_offset + offset,
		.writable = !!(first.access & DRM_ASAHI_BIND_WRITE),
	};
	return 0;
}

static void gpu_m4_provider_job_free(struct gpu_m4_drm_provider_job *job)
{
	struct gpu_m4_drm_provider_queue *queue = job->queue;
	struct gpu_m4_drm_provider_job *parent = job->window_parent;
	struct mutex *lock = gpu_m4_provider_job_lock(job);

	/* Registration holds a materializer ref: never freed while counted. */
	WARN_ON(job->window_early);
	gpu_m4_provider_guard_release(job);
	/* Even accepting rollback unlocks every ww reservation BEFORE pin puts. */
	gpu_m4_drm_resv_release(job->resv);
	if (job->cache)
		WARN_ON(gpu_m4_drm_cache_job_release(job->cache));
	mutex_lock(lock);
	if (job->runtime_job)
		queue->runtime.release_locked(queue->runtime_owner, job->runtime_job);
	mutex_unlock(lock);
	gpu_m4_drm_mixed_put(&job->decode, &job->batch);
	/* Stage cleanup may acquire reservations or the provider mutex. Its
	 * native baselines remain retained by the version or core extra refs until
	 * all snapshot/cache pin borrowers have gone; uncertain jobs never tear down.
	 */
	if (job->stage_owner)
		queue->runtime.stage_put(queue->runtime_owner, job->stage_owner);
	/* Version and extra object baselines own every borrowed native pin and
	 * charge. Drop them only AFTER both cache and VM snapshot final puts.
	 */
	mutex_lock(lock);
	for (u32 i = 0; i < job->stage_extra_count; i++)
		gpu_m4_drm_provider_object_put_locked(job->stage_extra_objects[i]);
	if (job->version)
		WARN_ON(gpu_m4_drm_provider_version_put_locked(job->version));
	mutex_unlock(lock);
	queue->runtime.reap(queue->runtime_owner);
	dma_fence_put(job->finished);
	if (job->retirement)
		dma_fence_put(&job->retirement->base);
	kvfree(job->commands); /* mixed_put clears batch, not this allocation owner */
	if (!parent)
		kfree(job->bytes);
	kfree(job->stage_extra_objects);
	kfree(job->ts_memo);
	kfree(job->cache_rows);
	kfree(job->timestamp_rows);
	kfree(job);
	gpu_m4_drm_provider_queue_put(queue);
	if (parent)
		WARN_ON(gpu_m4_drm_provider_job_put(parent));
}

int gpu_m4_drm_provider_job_put(struct gpu_m4_drm_provider_job *job)
{
	struct mutex *lock;
	bool last;

	if (!job)
		return 0;
	lock = gpu_m4_provider_job_lock(job);
	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	if (refcount_read(&job->refs) == 1 && gpu_m4_drm_provider_job_uncertain_locked(job)) {
		mutex_unlock(lock);
		return -EBUSY;
	}
	last = refcount_dec_and_test(&job->refs);
	mutex_unlock(lock);
	if (last)
		gpu_m4_provider_job_free(job);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_put);

static int gpu_m4_provider_join_timestamps(struct gpu_m4_drm_provider_job *job)
{
	struct gpu_m4_drm_cache_access *extra;
	struct gpu_m4_drm_vm_owner *joined, *source = job->batch.vm_snapshot;
	u32 count = 0;

	extra = kcalloc(job->batch.count * GPU_M4_DRM_RENDER_TIMESTAMPS,
			 sizeof(*extra), GFP_KERNEL);
	if (!extra)
		return -ENOMEM;
	for (u32 i = 0; i < job->batch.count; i++) {
		struct gpu_m4_drm_job_command *command = &job->batch.commands[i];
		struct gpu_m4_drm_job_timestamp *ts = command->type == DRM_ASAHI_CMD_COMPUTE ?
			command->compute.timestamps : command->render.timestamps;
		u32 n = command->type == DRM_ASAHI_CMD_COMPUTE ?
			GPU_M4_DRM_JOB_TIMESTAMPS : GPU_M4_DRM_RENDER_TIMESTAMPS;

		for (u32 j = 0; j < n; j++) {
			struct gpu_m4_drm_provider_timestamp *lease = ts[j].lease;

			if (!lease)
				continue;
			extra[count++] = (struct gpu_m4_drm_cache_access) {
				.pin = gpu_m4_drm_provider_object_native(lease->object),
				.offset = ts[j].object_offset, .size = 8,
				.access = DRM_ASAHI_BIND_WRITE,
			};
		}
	}
	if (!count) {
		kfree(extra);
		return 0;
	}
	/* Exact copied version/generation plus extras; NEVER reread active VM. */
	joined = gpu_m4_drm_vm_owner_clone_with_extras(source, extra, count);
	if (IS_ERR(joined)) {
		kfree(extra);
		return PTR_ERR(joined);
	}
	gpu_m4_provider_job_uncover(job);
	job->batch.vm_snapshot = joined;
	WARN_ON(gpu_m4_drm_vm_owner_release(source));
	/* Explicit sync: these exact timestamp rows (their pins now retained by
	 * the joined snapshot) are the only non-native rows of every cache job.
	 */
	if (job->implicit) {
		kfree(extra);
	} else {
		job->timestamp_rows = extra;
		job->timestamp_row_count = count;
	}
	return 0;
}

/* The job's cache job. Implicit: every snapshot row (the whole captured VM
 * plus extras), as before. Explicit: only the snapshot rows of this job's
 * stage native/extra objects and of the public capture's timestamp objects,
 * each exactly as the implicit set carries it (one unioned row per identity),
 * so every access check of those objects answers as before. User WB GEMs are
 * mapped through the GPU's shared coherent class (admitted pins never take a
 * payload sweep), so whole-VM rows only added pin traffic; the ordering
 * barriers of prepare/retire belong to the cache job itself, with or without
 * rows.
 */
static int gpu_m4_provider_cache_create(struct gpu_m4_drm_provider_job *job)
{
	const struct gpu_m4_drm_provider_job *origin = job->window_parent ?: job;
	const struct gpu_m4_drm_cache_access *all, *wanted, *use = NULL;
	struct gpu_m4_drm_cache_access *rows = NULL;
	struct gpu_m4_drm_cache_job *cache;
	u32 count, n, selected = 0;
	u8 *seen = NULL;
	int ret = 0;

	all = gpu_m4_drm_vm_owner_cache_access(job->batch.vm_snapshot, &count);
	if (job->implicit) {
		use = all;
		selected = count;
		goto create;
	}
	wanted = job->cache_rows ?: origin->timestamp_rows;
	n = job->cache_rows ? job->cache_row_count : origin->timestamp_row_count;
	if (!n)
		goto create;
	if (!all || !count || count > GPU_M4_DRM_MAX_BINDS) {
		ret = -EINVAL;
		goto out;
	}
	rows = kcalloc(n, sizeof(*rows), GFP_KERNEL);
	seen = kcalloc(count, sizeof(*seen), GFP_KERNEL);
	if (!rows || !seen) {
		ret = -ENOMEM;
		goto out;
	}
	for (u32 i = 0; i < n; i++) {
		u32 row = gpu_m4_drm_vm_owner_find(job->batch.vm_snapshot,
			gpu_m4_drm_bo_pin_object(wanted[i].pin));

		/* Every explicit row was joined into this very snapshot. */
		if (row >= count) {
			ret = -ESTALE;
			goto out;
		}
		if (!seen[row]) {
			seen[row] = 1;
			rows[selected++] = all[row];
		}
	}
	use = rows;
create:
	cache = gpu_m4_drm_cache_job_create(use, selected);
	if (IS_ERR(cache)) {
		ret = PTR_ERR(cache);
		goto out;
	}
	gpu_m4_provider_phase_value(GPU_M4_PROVIDER_PHASE_CACHE_ROWS, selected);
	job->cache = cache;
out:
	kfree(rows);
	kfree(seen);
	kfree(job->cache_rows);
	job->cache_rows = NULL;
	job->cache_row_count = 0;
	return ret;
}

/* Separate from post-cache adopt: this is the ONLY snapshot-growth entrance. */
static int gpu_m4_provider_stage_check_locked(struct gpu_m4_drm_provider_job *job,
	struct gpu_m4_drm_provider_version *source, const struct gpu_m4_drm_vm_owner *snapshot)
{
	int ret;

	lockdep_assert_held(gpu_m4_provider_job_lock(job));
	if (job->state != GPU_M4_PROVIDER_PREPARED || job->accepted || job->protected ||
	    job->cache || job->resv || job->runtime_job || job->staged || job->stage_owner ||
	    job->version != source || job->batch.vm_snapshot != snapshot)
		return -EBUSY;
	ret = gpu_m4_drm_provider_error_locked(job->queue->provider);
	if (ret)
		return ret;
	if (job->queue->closed)
		return -ECANCELED;
	if (!gpu_m4_drm_provider_vm_open_locked(job->queue->vm))
		return -ENOENT;
	return 0;
}

/* Alias/fresh allocator state belongs to the actual stage/runtime owner. This
 * entrance authenticates native origin and complete NC backing: exact RW8 for
 * submitted counters, whole declared pages for firmware-only metadata.
 */
static int gpu_m4_provider_stage_extras_locked(struct gpu_m4_drm_provider_job *job,
	struct gpu_m4_drm_provider_version *source, const struct gpu_m4_drm_vm_owner *snapshot,
	const struct gpu_m4_drm_provider_stage_result *result,
	struct gpu_m4_drm_cache_access *access,
	struct gpu_m4_drm_provider_object **objects, u32 *held)
{
	const struct gpu_m4_drm_cache_access *previous;
	u32 previous_count;
	int ret;

	lockdep_assert_held(gpu_m4_provider_job_lock(job));
	previous = gpu_m4_drm_vm_owner_cache_access(snapshot, &previous_count);
	for (u32 i = 0; i < result->extra_count; i++) {
		const struct gpu_m4_drm_provider_stage_extra *e = &result->extra[i];
		struct gpu_m4_drm_bo_pin *pin;
		struct drm_gem_object *gem;
		u64 bound, pa, retained;

		if (!gpu_m4_drm_aligned(e->offset))
			return -EINVAL;
		if (e->purpose == GPU_M4_DRM_PROVIDER_STAGE_SUBMITTED_COUNTER) {
			if (e->size != 8 ||
			    e->access != (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE))
				return -EINVAL;
			retained = GPU_M4_DRM_PAGE_SIZE;
		} else if (e->purpose == GPU_M4_DRM_PROVIDER_STAGE_FIRMWARE_METADATA) {
			if (!e->size || !gpu_m4_drm_aligned(e->size) ||
			    (e->access != DRM_ASAHI_BIND_READ &&
			     e->access != (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE)))
				return -EINVAL;
			retained = e->size;
		} else {
			return -EINVAL;
		}
		ret = gpu_m4_drm_provider_object_origin_locked(job->queue->provider, source, e->object);
		if (ret)
			return gpu_m4_drm_job_status(ret);
		pin = gpu_m4_drm_provider_object_native(e->object);
		gem = gpu_m4_drm_provider_object_identity(e->object);
		if (IS_ERR_OR_NULL(pin) || IS_ERR_OR_NULL(gem) ||
		    gpu_m4_drm_bo_pin_object(pin) != gem || gpu_m4_drm_bo_policy(gem) != GPU_M4_DRM_CPU_NC)
			return -EPERM;
		bound = gpu_m4_drm_bo_pin_size(pin);
		if (e->offset >= bound || retained > bound - e->offset)
			return -EINVAL;
		/* Separate whole-BO identity: never widen a user/timestamp or mapped
		 * work/AUX/pool access, nor alias another extra's backing.
		 */
		for (u32 j = 0; j < previous_count; j++)
			if (gpu_m4_drm_bo_pin_object(previous[j].pin) == gem)
				return -EADDRINUSE;
		for (u32 j = 0; j < result->count; j++)
			if (gpu_m4_drm_provider_object_identity(result->native[j].object) == gem)
				return -EADDRINUSE;
		for (u32 j = 0; j < *held; j++)
			if (gpu_m4_drm_provider_object_identity(objects[j]) == gem)
				return -EADDRINUSE;
		ret = gpu_m4_drm_provider_object_get_locked(e->object);
		if (ret)
			return gpu_m4_drm_job_status(ret);
		objects[(*held)++] = e->object;
		for (u64 offset = 0; offset < retained; offset += GPU_M4_DRM_PAGE_SIZE) {
			ret = gpu_m4_drm_provider_object_page_locked(e->object, e->offset + offset, &pa);
			if (ret)
				return gpu_m4_drm_job_status(ret);
			if (!pa || !gpu_m4_drm_aligned(pa) || pa >= (1ULL << 42))
				return -EPERM;
		}
		access[i] = (struct gpu_m4_drm_cache_access) {
			.pin = pin, .offset = e->offset, .size = e->size, .access = e->access,
		};
	}
	return 0;
}

/*
 * S3c tree path of the native stage. The runtime selects and builds its
 * units exactly as for the version path (a new unit's maps become the VM
 * tree's DRIVER extents at construction); the job then only checks that
 * every selected map IS such an extent of its tree and joins the selected
 * and extra rows into its (empty) snapshot for the cache job. No compose,
 * extend, graph construction, coverage proof or selected-coverage proof.
 */
static int gpu_m4_provider_stage_tree(struct gpu_m4_drm_provider_job *job)
{
	struct gpu_m4_drm_provider_queue *queue = job->queue;
	struct gpu_m4_drm_provider_version *source = job->version;
	struct gpu_m4_drm_vm_owner *snapshot = job->batch.vm_snapshot, *joined = NULL;
	struct gpu_m4_drm_provider_stage_result result = { 0 };
	struct gpu_m4_drm_provider_stage_view view = {
		.vm = queue->vm, .source = source, .snapshot = snapshot,
		.batch = &job->batch, .command_bytes = job->bytes,
		.command_size = job->size, .runtime_queue = queue->runtime_queue,
		.command_first = job->command_first, .command_total = job->command_total ?: job->batch.count,
		.deferred_materialization = !!job->window_parent,
	};
	struct gpu_m4_drm_cache_access *extra = NULL;
	const struct gpu_m4_drm_cache_access *previous;
	struct gpu_m4_drm_provider_object **objects = NULL;
	struct mutex *lock = gpu_m4_provider_job_lock(job);
	const struct gpu_m4_drm_provider_job *origin = job->window_parent ?: job;
	u32 total, existing, held = 0, spare;
	int ret;

	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	ret = gpu_m4_provider_stage_check_locked(job, source, snapshot);
	mutex_unlock(lock);
	if (ret)
		return ret;
	ret = gpu_m4_drm_job_status(queue->runtime.stage_native(queue->runtime_owner,
		queue->runtime_queue, &view, &result));
	if (ret == -ENOSPC)
		pr_warn("GPU native stage capacity: allocation commands=%u\n", job->batch.count);
	if (ret)
		goto out;
	if (result.count > GPU_M4_DRM_MAX_BINDS ||
	    result.extra_count > GPU_M4_DRM_PROVIDER_STAGE_MAX_EXTRAS ||
	    result.count > GPU_M4_DRM_MAX_BINDS - result.extra_count ||
	    (result.count && (IS_ERR_OR_NULL(result.native) || !result.owner)) ||
	    (!result.count && result.native) ||
	    (result.extra_count && (IS_ERR_OR_NULL(result.extra) || !result.owner)) ||
	    (!result.extra_count && result.extra)) {
		ret = -EINVAL;
		goto out;
	}
	/* The selected maps are exactly the tree's DRIVER extents (the
	 * hardware mapping this job will translate through).
	 */
	if (result.count && !gpu_m4_drm_provider_vm_native_mapped(queue->vm, result.native,
								  result.count)) {
		ret = -ESTALE;
		goto out;
	}
	total = result.count + result.extra_count;
	spare = origin->timestamp_row_count;
	if (total) {
		previous = gpu_m4_drm_vm_owner_cache_access(snapshot, &existing);
		if ((existing && !previous) || existing > GPU_M4_DRM_MAX_BINDS - total) {
			ret = -E2BIG;
			goto out;
		}
		extra = kcalloc(total + spare, sizeof(*extra), GFP_KERNEL);
		if (!extra) {
			ret = -ENOMEM;
			goto out;
		}
		if (result.extra_count) {
			objects = kcalloc(result.extra_count, sizeof(*objects), GFP_KERNEL);
			if (!objects) {
				ret = -ENOMEM;
				goto out;
			}
		}
		mutex_lock(lock);
		ret = gpu_m4_provider_stage_check_locked(job, source, snapshot);
		for (u32 i = 0; !ret && i < result.count; i++) {
			const struct gpu_m4_drm_provider_private_gem *m = &result.native[i];

			extra[i] = (struct gpu_m4_drm_cache_access) {
				.pin = gpu_m4_drm_provider_object_native(m->object),
				.offset = m->offset, .size = m->size, .access = m->access,
			};
		}
		if (!ret && result.extra_count)
			ret = gpu_m4_provider_stage_extras_locked(job, source, snapshot, &result,
				extra + result.count, objects, &held);
		mutex_unlock(lock);
		if (ret)
			goto out;
		joined = gpu_m4_drm_vm_owner_clone_with_extras(snapshot, extra, total);
		if (IS_ERR(joined)) {
			ret = PTR_ERR(joined);
			joined = NULL;
			goto out;
		}
	}
	mutex_lock(lock);
	ret = gpu_m4_provider_stage_check_locked(job, source, snapshot);
	if (!ret) {
		gpu_m4_provider_job_uncover(job);
		if (joined) {
			job->batch.vm_snapshot = joined;
			joined = NULL;
		}
		/* An empty TREE need set: coverage is trivially this pair. */
		job->covered_version = job->version;
		job->covered_snapshot = job->batch.vm_snapshot;
		job->stage_extra_objects = objects;
		job->stage_extra_count = held;
		objects = NULL;
		held = 0;
		job->stage_owner = result.owner;
		result.owner = NULL;
		job->staged = true;
		if (extra) {
			if (spare)
				memcpy(extra + total, origin->timestamp_rows, spare * sizeof(*extra));
			job->cache_rows = extra;
			job->cache_row_count = total + spare;
			extra = NULL;
		}
	}
	mutex_unlock(lock);
	if (!ret && job->batch.vm_snapshot != snapshot)
		WARN_ON(gpu_m4_drm_vm_owner_release(snapshot));
out:
	if (joined)
		WARN_ON(gpu_m4_drm_vm_owner_release(joined));
	if (held) {
		mutex_lock(lock);
		for (u32 i = 0; i < held; i++)
			gpu_m4_drm_provider_object_put_locked(objects[i]);
		mutex_unlock(lock);
	}
	if (result.owner)
		queue->runtime.stage_put(queue->runtime_owner, result.owner);
	kfree(extra);
	kfree(objects);
	return ret;
}

static int gpu_m4_provider_stage_native(struct gpu_m4_drm_provider_job *job)
{
	struct gpu_m4_drm_provider_queue *queue = job->queue;
	struct gpu_m4_drm_provider_version *source = job->version, *candidate = NULL;
	struct gpu_m4_drm_vm_owner *snapshot = job->batch.vm_snapshot, *joined = NULL;
	struct gpu_m4_drm_provider_stage_result result = { 0 };
	struct gpu_m4_drm_provider_stage_view view = {
		.vm = queue->vm, .source = source, .snapshot = snapshot,
		.batch = &job->batch, .command_bytes = job->bytes,
		.command_size = job->size, .runtime_queue = queue->runtime_queue,
		.command_first = job->command_first, .command_total = job->command_total ?: job->batch.count,
		.deferred_materialization = !!job->window_parent,
	};
	struct gpu_m4_drm_cache_access *extra = NULL;
	const struct gpu_m4_drm_cache_access *previous;
	struct gpu_m4_drm_provider_object **objects = NULL;
	struct gpu_m4_drm_provider_baseline *baseline = NULL;
	struct mutex *lock = gpu_m4_provider_job_lock(job);
	const struct gpu_m4_drm_provider_job *origin = job->window_parent ?: job;
	bool composed = false;
	u64 low, high;
	u32 total, existing, held = 0, spare = 0;
	int ret;

	if (!queue->runtime.stage_native)
		return 0; /* Exact existing acceptance/cache path. */
	if (job->tree)
		return gpu_m4_provider_stage_tree(job);
	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	ret = gpu_m4_provider_stage_check_locked(job, source, snapshot);
	mutex_unlock(lock);
	if (ret)
		return ret;
	ret = gpu_m4_drm_job_status(queue->runtime.stage_native(queue->runtime_owner,
		queue->runtime_queue, &view, &result));
	if (ret == -ENOSPC)
		pr_warn("GPU native stage capacity: allocation commands=%u\n", job->batch.count);
	if (ret)
		goto out;
	if (result.count > GPU_M4_DRM_MAX_BINDS ||
	    result.extra_count > GPU_M4_DRM_PROVIDER_STAGE_MAX_EXTRAS ||
	    result.count > GPU_M4_DRM_MAX_BINDS - result.extra_count ||
	    (result.count && (IS_ERR_OR_NULL(result.native) || !result.owner)) ||
	    (!result.count && result.native) ||
	    (result.extra_count && (IS_ERR_OR_NULL(result.extra) || !result.owner)) ||
	    (!result.extra_count && result.extra)) {
		ret = -EINVAL;
		goto out;
	}
	/* Exact membership resolution against the VM's held native baseline:
	 * when every selected unit is already mapped there, the child composes
	 * its held user capture with that baseline (no per-child extension).
	 * Otherwise (a unit not yet installed) today's per-child append stays.
	 */
	if (result.count) {
		mutex_lock(lock);
		baseline = gpu_m4_drm_provider_vm_baseline_get_locked(queue->vm);
		composed = baseline && gpu_m4_drm_provider_baseline_contains_locked(baseline,
			result.native, result.count);
		mutex_unlock(lock);
	}
	/* Build the captured source's one complete graph outside the provider
	 * mutex (held through job->version). The locked extension below then only
	 * derives the changed branches; the no-extension ensure finds it built.
	 * A composed child needs no source graph: it shares/derives its own.
	 */
	if (!composed) {
		ret = gpu_m4_drm_provider_version_ensure_graph(source, NULL);
		if (ret)
			goto out;
	}
	total = result.count + result.extra_count;
	if (total) {
		/* Bound clone/cache/resv capacity before any count-controlled allocation.
		 * The runtime's stage owner charges its finite acceptance metadata budget.
		 */
		previous = gpu_m4_drm_vm_owner_cache_access(snapshot, &existing);
		if ((existing && !previous) ||
		    existing > GPU_M4_DRM_MAX_BINDS - total) {
			ret = -E2BIG;
			goto out;
		}
		/* Explicit sync: room for the timestamp rows after the stage's own,
		 * so this array becomes the job's complete cache row set.
		 */
		spare = job->implicit ? 0 : origin->timestamp_row_count;
		extra = kcalloc(total + spare, sizeof(*extra), GFP_KERNEL);
		if (!extra) {
			ret = -ENOMEM;
			goto out;
		}
		if (result.extra_count) {
			objects = kcalloc(result.extra_count, sizeof(*objects), GFP_KERNEL);
			if (!objects) {
				ret = -ENOMEM;
				goto out;
			}
		}
		mutex_lock(lock);
		ret = gpu_m4_provider_stage_check_locked(job, source, snapshot);
		if (!ret && composed) {
			/* Unchanged held user map + exactly the held baseline. Its
			 * snapshot carries no baseline member access: only the
			 * selected accesses joined below enter cache/resv.
			 */
			candidate = gpu_m4_drm_provider_version_compose_locked(source, baseline);
			if (IS_ERR(candidate)) {
				ret = PTR_ERR(candidate);
				candidate = NULL;
				composed = false;
				if (ret == -ESTALE)
					ret = 0; /* e.g. per-child-extended source: append */
			}
		}
		if (!ret && result.count && !composed) {
			/* Authoritative construction copies the entire old private/context
			 * manifest and real native/claim references before appending.
			 * same_user alone could not establish private-prefix preservation.
			 */
			candidate = gpu_m4_drm_provider_version_extend_private_gem_locked(
				source, result.native, result.count);
			if (IS_ERR(candidate)) {
				ret = PTR_ERR(candidate);
				if (ret == -ENOSPC)
					pr_warn("GPU native stage capacity: extend-private maps=%u extra=%u\n",
						result.count, result.extra_count);
				candidate = NULL;
			}
		}
		if (!ret)
			for (u32 i = 0; i < result.count; i++) {
				const struct gpu_m4_drm_provider_private_gem *m = &result.native[i];

				/* Actual validated native owners, never callback-supplied pins/PAs. */
				extra[i] = (struct gpu_m4_drm_cache_access) {
					.pin = gpu_m4_drm_provider_object_native(m->object),
					.offset = m->offset, .size = m->size, .access = m->access,
				};
			}
		if (!ret && result.extra_count)
			ret = gpu_m4_provider_stage_extras_locked(job, source, snapshot, &result,
				extra + result.count, objects, &held);
		mutex_unlock(lock);
		if (ret)
			goto out;
		/* Preserve EVERY old timestamp-only extra, not a bare candidate capture.
		 * Allocation/clone rollback and pin puts occur outside GPU/resv locks.
		 */
		joined = gpu_m4_drm_vm_owner_clone_with_extras(snapshot, extra, total);
		if (IS_ERR(joined)) {
			ret = PTR_ERR(joined);
			joined = NULL;
			goto out;
		}
	}
	/* A composed child's image is materialized once per user map + baseline
	 * revision; this child gets a use wrapper of it (or a detached delta)
	 * outside the provider mutex.
	 */
	if (composed) {
		ret = gpu_m4_drm_provider_version_ensure_graph(candidate, NULL);
		if (ret)
			goto out;
	}
	mutex_lock(lock);
	ret = gpu_m4_provider_stage_check_locked(job, source, snapshot);
	if (!ret && (gpu_m4_drm_provider_version_generation(candidate ? candidate : source) !=
			gpu_m4_drm_vm_owner_generation(joined ? joined : snapshot) ||
	    (candidate && !gpu_m4_drm_provider_version_same_user_locked(candidate, source)) ||
	    gpu_m4_drm_vm_owner_vm(joined ? joined : snapshot) != gpu_m4_drm_vm_owner_vm(snapshot)))
		ret = -EINVAL;
	/* Ordinary acceptance may capture only a logical snapshot. Construct the
	 * final job graph here, including the extras-only/no-extension case, before
	 * granting roots, cache coverage or runtime acceptance authority.
	 */
	if (!ret)
		ret = gpu_m4_drm_provider_version_ensure_graph_locked(candidate ? candidate : source);
	if (!ret)
		ret = gpu_m4_drm_provider_version_roots_locked(candidate ? candidate : source,
							     &low, &high);
	if (!ret && !gpu_m4_drm_provider_version_native_covered_locked(
			candidate ? candidate : source, joined ? joined : snapshot))
		ret = -EACCES;
	/* Composed: every selected range is an exact member of the held
	 * baseline and covered by this child's actual joined cache/resv rows.
	 */
	if (!ret && composed && !gpu_m4_drm_provider_version_selected_covered_locked(
			candidate, result.native, result.count, joined ? joined : snapshot))
		ret = -EACCES;
	if (!ret && candidate)
		ret = gpu_m4_drm_provider_version_put_locked(source);
	if (!ret) {
		/* No fallible operation remains in this serialized transfer. Source
		 * snapshot pin borrowers remain charged by the copied candidate owners
		 * and the unchanged timestamp leases until the outside-lock drop.
		 */
		gpu_m4_provider_job_uncover(job);
		if (candidate) {
			job->version = candidate;
			candidate = NULL;
		}
		if (joined) {
			job->batch.vm_snapshot = joined;
			joined = NULL;
		}
		/* The complete coverage proof above is of exactly this pair. */
		job->covered_version = job->version;
		job->covered_snapshot = job->batch.vm_snapshot;
		job->stage_extra_objects = objects;
		job->stage_extra_count = held;
		objects = NULL;
		held = 0;
		job->stage_owner = result.owner;
		result.owner = NULL;
		job->staged = true;
		if (!job->implicit && extra) {
			if (spare)
				memcpy(extra + total, origin->timestamp_rows,
				       spare * sizeof(*extra));
			job->cache_rows = extra;
			job->cache_row_count = total + spare;
			extra = NULL;
		}
	}
	mutex_unlock(lock);
	if (!ret && job->batch.vm_snapshot != snapshot)
		WARN_ON(gpu_m4_drm_vm_owner_release(snapshot));
out:
	/* Never drop the candidate's accounting baseline before clone pin puts. */
	if (joined)
		WARN_ON(gpu_m4_drm_vm_owner_release(joined));
	if (candidate || baseline) {
		mutex_lock(lock);
		if (candidate)
			WARN_ON(gpu_m4_drm_provider_version_put_locked(candidate));
		/* The composed version holds its own baseline reference. */
		gpu_m4_drm_provider_baseline_release_locked(baseline);
		mutex_unlock(lock);
	}
	if (held) {
		mutex_lock(lock);
		for (u32 i = 0; i < held; i++)
			gpu_m4_drm_provider_object_put_locked(objects[i]);
		mutex_unlock(lock);
	}
	if (result.owner)
		queue->runtime.stage_put(queue->runtime_owner, result.owner);
	kfree(extra);
	kfree(objects);
	return ret;
}

static int gpu_m4_provider_private_stream(struct gpu_m4_drm_provider_queue *queue,
	const void *bytes, u32 size)
{
	struct drm_asahi_cmd_header h;
	u32 computes = 0;
	int ret;

	ret = gpu_m4_drm_stream_validate(&queue->params, bytes, size);
	if (ret)
		return ret;
	for (u32 offset = 0; offset < size; offset += sizeof(h) + h.size) {
		memcpy(&h, bytes + offset, sizeof(h));
		if (h.cmd_type == DRM_ASAHI_CMD_COMPUTE)
			computes++;
		else if (h.cmd_type != DRM_ASAHI_SET_COMPUTE_ATTACHMENTS)
			return -EOPNOTSUPP;
	}
	return computes == 1 ? 0 : -EINVAL;
}

static void *gpu_m4_provider_prepare_inner(void *cookie, const struct gpu_m4_drm_submit *submit);

static void *gpu_m4_provider_prepare(void *cookie, const struct gpu_m4_drm_submit *submit)
{
	u64 start = ktime_get_ns();
	struct gpu_m4_drm_provider_job *job = gpu_m4_provider_prepare_inner(cookie, submit);

	gpu_m4_provider_stat_add(0, start);
	if (!IS_ERR_OR_NULL(job))
		job->submit_ns = start; /* REMOVE-DEBUG pacing */
	return job;
}

static void *gpu_m4_provider_prepare_inner(void *cookie, const struct gpu_m4_drm_submit *submit)
{
	struct gpu_m4_drm_provider_queue *queue = cookie;
	struct gpu_m4_drm_provider_job *job;
	struct gpu_m4_drm_job_command *commands = NULL, *scratch = NULL;
	struct gpu_m4_drm_submit copied = *submit;
	const struct gpu_m4_drm_resv_access *resv;
	struct mutex *lock = gpu_m4_drm_provider_lock(queue->provider);
	const char *phase = "allocation";
	u32 count, capacity = queue->params.max_commands_per_submission;
	u64 clock = ktime_get_ns(); /* REMOVE-DEBUG sub-phase timing */
	int ret;

	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job)
		return ERR_PTR(-ENOMEM);
	refcount_set(&job->refs, 1); /* scheduler-cookie ref */
	job->queue = queue;
	gpu_m4_drm_provider_queue_get(queue);
	job->state = GPU_M4_PROVIDER_PREPARED;
	/* The sealed private qualification profile keeps its whole-VM inventory:
	 * its single probe proves user WB input/output through those rows.
	 */
	job->implicit = READ_ONCE(implicit_sync) || queue->private_candidate;
	/* S3c: tree-path jobs are explicit-sync only (no captured inventory to
	 * reserve); implicit_sync stays the version path's rollback.
	 */
	job->tree = queue->tree;
	if (job->tree)
		job->implicit = false;
	INIT_WORK(&job->window_work, gpu_m4_provider_window_work);
	INIT_WORK(&job->window_put_work, gpu_m4_provider_window_put_work);
	INIT_DELAYED_WORK(&job->window_retry, gpu_m4_provider_window_retry);
	job->size = submit->size;
	job->bytes = kmemdup(submit->cmdbuf, submit->size, GFP_KERNEL);
	ret = gpu_m4_provider_retirement_init(job);
	if (ret)
		goto fail;
	commands = kvcalloc(capacity, sizeof(*commands), GFP_KERNEL);
	job->commands = commands;
	scratch = kvcalloc(capacity, sizeof(*scratch), GFP_KERNEL);
	if (!job->bytes || !job->retirement || !commands || !scratch) {
		ret = -ENOMEM;
		goto fail;
	}
	if (queue->private_candidate) {
		ret = gpu_m4_provider_private_stream(queue, job->bytes, job->size);
		if (ret)
			goto fail;
	}
	phase = "decode";
	job->decode = (struct gpu_m4_drm_mixed_config) {
		.job = { .params = &queue->params, .kernel_start = queue->kernel_start,
			.kernel_end = queue->kernel_end, .usc_base = queue->usc_base,
			.cookie = job, .ops = &gpu_m4_provider_decode_ops },
		.resolve_span = gpu_m4_provider_resolve_span,
	};
	copied.cmdbuf = job->bytes;
	ret = gpu_m4_drm_mixed_decode(&job->batch, commands, scratch, capacity,
				    &job->decode, &copied);
	job->decode_owner = NULL; /* Never authorize a later snapshot/span lookup. */
	if (ret)
		goto fail;
	gpu_m4_provider_phase_mark(GPU_M4_PROVIDER_PHASE_DECODE, &clock);
	/* REMOVE-DEBUG (S0/S1): the captured VM inventory implicit sync
	 * reserves, and the shared dma-bufs explicit sync leaves to userspace.
	 */
	if (!job->tree)
		gpu_m4_provider_census(job->batch.vm_snapshot);
	if (job->implicit)
		gpu_m4_provider_phase_value(GPU_M4_PROVIDER_PHASE_IMPLICIT_JOBS, 1);
	phase = "timestamp-join";
	ret = gpu_m4_provider_join_timestamps(job);
	if (ret)
		goto fail;
	gpu_m4_provider_phase_mark(GPU_M4_PROVIDER_PHASE_TS_JOIN, &clock);
	/* Every ordinary batch materializes through the bounded CPU worker: its
	 * native capacity pressure becomes backpressure/retry instead of an
	 * ioctl-time failure under the scheduler queue mutex.
	 */
	if (queue->execution_window) {
		/* Cheap immutable public capture only: the original decoded commands,
		 * joined snapshot/version, timestamp leases and common reservation
		 * inputs. Each bounded child later owns real native/cache/runtime state.
		 */
		job->aggregate = true;
		kvfree(scratch);
		scratch = NULL;
		if (!job->implicit)
			return job; /* explicit inputs only: nothing reserved */
		phase = "aggregate-reservation";
		resv = gpu_m4_drm_vm_owner_resv_access(job->batch.vm_snapshot, &count);
		job->resv = gpu_m4_drm_resv_prepare(resv, count);
		if (IS_ERR(job->resv)) {
			ret = PTR_ERR(job->resv);
			job->resv = NULL;
			goto fail;
		}
		gpu_m4_provider_phase_mark(GPU_M4_PROVIDER_PHASE_RESV, &clock);
		return job;
	}
	phase = "native-stage";
	ret = gpu_m4_provider_stage_native(job);
	if (ret)
		goto fail;
	gpu_m4_provider_phase_mark(GPU_M4_PROVIDER_PHASE_STAGE, &clock);
	phase = "cache";
	ret = gpu_m4_provider_cache_create(job);
	if (ret)
		goto fail;
	gpu_m4_provider_phase_mark(GPU_M4_PROVIDER_PHASE_CACHE, &clock);
	phase = "runtime-accept";
	mutex_lock(lock);
	ret = gpu_m4_drm_provider_error_locked(queue->provider);
	if (!ret && !queue->closed)
		ret = gpu_m4_drm_job_status(queue->runtime.accept_locked(queue->runtime_owner,
			queue->runtime_queue, job, &job->runtime_job));
	else if (!ret)
		ret = -ECANCELED;
	if (!ret && !job->runtime_job)
		ret = -EIO;
	mutex_unlock(lock);
	if (ret)
		goto fail;
	kvfree(scratch);
	scratch = NULL;
	gpu_m4_provider_phase_mark(GPU_M4_PROVIDER_PHASE_ACCEPT, &clock);
	if (!job->implicit)
		return job; /* explicit inputs only: nothing reserved */
	phase = "reservation";
	resv = gpu_m4_drm_vm_owner_resv_access(job->batch.vm_snapshot, &count);
	/* LAST fallible step: hold these ww locks until immediate finished install. */
	job->resv = gpu_m4_drm_resv_prepare(resv, count);
	if (IS_ERR(job->resv)) {
		ret = PTR_ERR(job->resv);
		job->resv = NULL;
		goto fail;
	}
	gpu_m4_provider_phase_mark(GPU_M4_PROVIDER_PHASE_RESV, &clock);
	return job;
fail:
	/* REMOVE-DEBUG: identify ordinary desktop capacity rejection without
	 * changing publication, fault state or any allocation ceiling.
	 */
	if (ret == -ENOSPC)
		pr_warn("GPU public prepare capacity: phase=%s commands=%u bytes=%u error=%d\n",
			phase, job->batch.count, job->size, ret);
	/* An ioctl-local refusal; job_free's collector request reclaims. */
	if (gpu_m4_drm_provider_capacity_error(ret))
		gpu_m4_provider_proof_add(GPU_M4_DRM_PROVIDER_PROOF_CAPACITY_REFUSALS, 1);
	gpu_m4_provider_job_free(job);
	kvfree(scratch);
	return ERR_PTR(ret);
}

static void gpu_m4_provider_publish_finished(void *cookie, void *owner, struct dma_fence *finished)
{
	struct gpu_m4_drm_provider_job *job = owner;

	(void)cookie;
	job->finished = dma_fence_get(finished);
	/* Explicit sync: a plain store. No BO reservation ever receives this
	 * fence; output syncobjs (and sync files userspace imports into shared
	 * dma-bufs) carry it to every consumer.
	 */
	if (job->implicit)
		gpu_m4_drm_resv_publish_finished(job->resv, finished); /* unlocks ALL ww */
	job->accepted = true;
	/* REMOVE-DEBUG: accepted, not run yet (inflight2 diagnostics). */
	if (!job->waiting_counted) {
		job->waiting_counted = true;
		atomic_inc(&job->queue->waiting);
	}
}

/* REMOVE-DEBUG: the job leaves the accepted-not-run set (run or release). */
static void gpu_m4_provider_waiting_done(struct gpu_m4_drm_provider_job *job)
{
	if (job->waiting_counted) {
		job->waiting_counted = false;
		atomic_dec(&job->queue->waiting);
	}
}

/* REMOVE-DEBUG: run of an accepted job: it stops waiting; submit -> run. */
static void gpu_m4_provider_pacing_run(struct gpu_m4_drm_provider_job *job)
{
	gpu_m4_provider_waiting_done(job);
#ifdef CONFIG_DEBUG_FS
	gpu_m4_trace(GPU_M4_TRACE_RUN, 0, (u64)(uintptr_t)job->queue->runtime_queue,
		(u64)(uintptr_t)job, 0, 0);
#endif
	if (job->submit_ns)
		gpu_m4_provider_pacing_add(GPU_M4_DRM_PROVIDER_PACING_SUBMIT_RUN,
			ktime_get_ns() - job->submit_ns);
}


/* Explicit sync: the scheduler's input syncobj fences are the only inputs. */
static struct dma_fence *gpu_m4_provider_next_dependency(void *cookie, void *owner)
{
	struct gpu_m4_drm_provider_job *job = owner;

	(void)cookie;
	if (!job->implicit)
		return NULL;
	return gpu_m4_drm_resv_next_dependency(job->resv);
}

/* Running aggregates of one queue, in run order (the scheduler entity runs a
 * queue's jobs in submission order): at most one per scheduler credit.
 */
static void gpu_m4_provider_aggregate_add_locked(struct gpu_m4_drm_provider_queue *queue,
	struct gpu_m4_drm_provider_job *job)
{
	for (u32 i = 0; i < (u32)GPU_M4_DRM_SCHED_MAX_CREDITS; i++) {
		if (!queue->aggregate[i]) {
			queue->aggregate[i] = job;
			return;
		}
	}
	WARN_ON(1); /* more running batches than scheduler credits */
}

/* Wake the running batches after parent (or every one with parent NULL). */
static void gpu_m4_provider_aggregate_wake_locked(struct gpu_m4_drm_provider_queue *queue,
	struct gpu_m4_drm_provider_job *parent)
{
	bool after = !parent;

	for (u32 i = 0; i < (u32)GPU_M4_DRM_SCHED_MAX_CREDITS && queue->aggregate[i]; i++) {
		struct gpu_m4_drm_provider_job *job = queue->aggregate[i];

		if (after && job->window_work_ref)
			queue_work(system_unbound_wq, &job->window_work);
		after |= job == parent;
	}
}

static void gpu_m4_provider_aggregate_unlink_locked(struct gpu_m4_drm_provider_queue *queue,
	struct gpu_m4_drm_provider_job *parent)
{
	u32 n = 0;

	for (u32 i = 0; i < (u32)GPU_M4_DRM_SCHED_MAX_CREDITS; i++)
		if (queue->aggregate[i] && queue->aggregate[i] != parent)
			queue->aggregate[n++] = queue->aggregate[i];
	for (u32 i = n; i < (u32)GPU_M4_DRM_SCHED_MAX_CREDITS; i++)
		queue->aggregate[i] = NULL;
}

static void gpu_m4_provider_aggregate_remove_locked(struct gpu_m4_drm_provider_queue *queue,
	struct gpu_m4_drm_provider_job *parent)
{
	gpu_m4_provider_aggregate_unlink_locked(queue, parent);
	/* A batch materializing behind the finished one may proceed now. */
	gpu_m4_provider_aggregate_wake_locked(queue, NULL);
}

/* Same-queue order of running batches: parent materializes its FIRST child
 * only once every earlier running batch of its queue enqueued its final
 * child, so the lane's per-queue order sees all of them in queue order. With
 * one scheduler credit there is never an earlier running batch.
 */
static bool gpu_m4_provider_aggregate_turn_locked(struct gpu_m4_drm_provider_job *parent)
{
	struct gpu_m4_drm_provider_queue *queue = parent->queue;

	for (u32 i = 0; i < (u32)GPU_M4_DRM_SCHED_MAX_CREDITS && queue->aggregate[i]; i++) {
		if (queue->aggregate[i] == parent)
			return true;
		if (!queue->aggregate[i]->window_final)
			return false;
	}
	return true;
}

int gpu_m4_drm_provider_job_queue_next_locked(struct gpu_m4_drm_provider_job *child)
{
	struct gpu_m4_drm_provider_job *parent;
	struct gpu_m4_drm_provider_queue *queue;
	int next = GPU_M4_DRM_QUEUE_NEXT_NONE;

	if (IS_ERR_OR_NULL(child) || IS_ERR_OR_NULL(child->queue))
		return -EINVAL;
	queue = child->queue;
	lockdep_assert_held(gpu_m4_drm_provider_lock(queue->provider));
	parent = child->window_parent;
	if (parent && child->command_first + child->batch.count < parent->batch.count)
		return GPU_M4_DRM_QUEUE_NEXT_OWN;
	for (u32 i = 0; i < (u32)GPU_M4_DRM_SCHED_MAX_CREDITS && queue->aggregate[i]; i++) {
		struct gpu_m4_drm_provider_job *job = queue->aggregate[i];

		if (job == parent)
			continue;
		if (job->active_window && job->active_window->window_enqueued)
			return GPU_M4_DRM_QUEUE_NEXT_ENQUEUED;
		next = !job->active_window && !gpu_m4_provider_aggregate_turn_locked(job) ?
			GPU_M4_DRM_QUEUE_NEXT_GATED : GPU_M4_DRM_QUEUE_NEXT_BUILDING;
	}
	if (next == GPU_M4_DRM_QUEUE_NEXT_NONE && atomic_read(&queue->waiting))
		next = GPU_M4_DRM_QUEUE_NEXT_WAITING;
	return next;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_queue_next_locked);

/* Early first window. The scheduler names this queue's next job (every
 * earlier job of the queue completed) while its explicit and implicit inputs
 * may still be pending. Materializing the first child (native stage, units
 * and cells, graph and compose, cache set and runtime acceptance) reads only
 * the immutable submit capture and publishes nothing; its reservation seal,
 * own dependencies and lane enqueue still follow run, i.e. every scheduler
 * dependency. The materializer ref taken here moves to run or ends at
 * release. Bounded by early_max and no new one starts in a quiet period.
 */
static void gpu_m4_provider_begin_dependencies(void *cookie, void *owner)
{
	struct gpu_m4_drm_provider_queue *queue = cookie;
	struct gpu_m4_drm_provider_job *job = owner;
	struct mutex *lock = gpu_m4_provider_job_lock(job);
	u64 pressure = atomic64_read(&gpu_m4_provider_early_pressure);
	bool skipped;

	if (!job->aggregate || !READ_ONCE(early_max))
		return;
	mutex_lock(lock);
	/* With two credits the next batch gets its credit while the previous
	 * one runs: it may build its first window early once every running
	 * batch of the queue enqueued its final child (queue order).
	 */
	if (job->state != GPU_M4_PROVIDER_PREPARED || !job->accepted || job->window_work_ref ||
	    queue->closed || !gpu_m4_provider_aggregate_turn_locked(job) || queue->early ||
	    gpu_m4_drm_provider_error_locked(queue->provider) ||
	    gpu_m4_drm_provider_vm_error_locked(queue->vm)) {
		mutex_unlock(lock);
		return;
	}
	skipped = (pressure && ktime_get_ns() - (pressure - 1) < GPU_M4_PROVIDER_EARLY_QUIET_NS) ||
		!gpu_m4_provider_early_register_locked(job);
	if (!skipped && gpu_m4_drm_provider_job_get_locked(job)) {
		gpu_m4_provider_early_unregister_locked(job);
		skipped = true;
	}
	if (!skipped) {
		job->window_work_ref = true;
		queue->early = job;
		queue_work(system_unbound_wq, &job->window_work);
	}
	mutex_unlock(lock);
	if (skipped)
		gpu_m4_provider_early_add(GPU_M4_DRM_PROVIDER_EARLY_SKIPPED, 1);
}


/* Run of an aggregate: dependency-ready. True when an early materializer
 * already holds window_work_ref; the batch then stops counting against
 * early_max, and a child still being built attaches when its build ends.
 */
static bool gpu_m4_provider_window_ready_locked(struct gpu_m4_drm_provider_job *job)
{
	atomic64_t *max = &gpu_m4_provider_early_stat[GPU_M4_DRM_PROVIDER_EARLY_HIDDEN_MAX];
	u64 hidden;

	job->window_ready_ns = ktime_get_ns(); /* REMOVE-DEBUG */
	if (!job->window_work_ref)
		return false;
	if (job->active_window) {
		hidden = job->window_ready_ns - job->window_built_ns;
		job->window_early_used = true;
		gpu_m4_provider_early_add(GPU_M4_DRM_PROVIDER_EARLY_USED, 1);
		gpu_m4_provider_early_add(GPU_M4_DRM_PROVIDER_EARLY_HIDDEN_NS, hidden);
		if (hidden > (u64)atomic64_read(max))
			atomic64_set(max, hidden);
	}
	gpu_m4_provider_early_unregister_locked(job);
	return true;
}

static struct dma_fence *gpu_m4_provider_run(void *cookie, void *owner)
{
	struct gpu_m4_drm_provider_queue *queue = cookie;
	struct gpu_m4_drm_provider_job *job = owner;
	struct mutex *lock = gpu_m4_provider_job_lock(job);
	struct dma_fence *fence;
	int ret;

	mutex_lock(lock);
	gpu_m4_provider_pacing_run(job); /* REMOVE-DEBUG */
	ret = gpu_m4_drm_provider_error_locked(queue->provider);
	if (!ret && (queue->closed || !job->accepted || job->state != GPU_M4_PROVIDER_PREPARED))
		ret = -ECANCELED;
	/* An admitted job that never runs still fails its owner's VM first. */
	if (ret)
		gpu_m4_drm_provider_vm_quarantine_locked(queue->vm, ret);
	if (ret) {
		fence = ERR_PTR(ret); /* guaranteed NO hardware publication */
	} else if (job->aggregate) {
		/* CPU materializer, not a second scheduler: every real child still
		 * enters the global lane. The work owns this ref until finalization;
		 * an early first window's materializer already holds it.
		 */
		job->state = GPU_M4_PROVIDER_AGGREGATE;
		if (!gpu_m4_provider_window_ready_locked(job)) {
			gpu_m4_drm_provider_job_get_locked(job);
			job->window_work_ref = true;
		}
		gpu_m4_provider_aggregate_add_locked(queue, job);
		fence = dma_fence_get(&job->retirement->base);
		queue_work(system_unbound_wq, &job->window_work);
	} else {
		job->state = GPU_M4_PROVIDER_READY;
		gpu_m4_drm_provider_job_get_locked(job); /* transferred runtime ref */
		fence = dma_fence_get(&job->retirement->base);
		queue->runtime.enqueue_locked(queue->runtime_owner, job);
	}
	mutex_unlock(lock);
	return fence;
}

void gpu_m4_drm_provider_job_fault_locked(struct gpu_m4_drm_provider_job *job, int error)
{
	struct gpu_m4_drm_provider_queue *queue;
	struct gpu_m4_drm_vm *vm;
	bool first;

	if (!job)
		return;
	lockdep_assert_held(gpu_m4_provider_job_lock(job));
	if (job->state == GPU_M4_PROVIDER_RETIRED || job->state == GPU_M4_PROVIDER_CANCELED)
		return;
	error = error < 0 ? error : -EIO;
	if (job->aggregate) {
		/* No native runtime payload: the actual child already latched any
		 * global fault. Fail only this public completion; the materializer
		 * detaches and disposes outside locks when it next runs.
		 */
		if (!job->retirement->ready) {
			job->state = GPU_M4_PROVIDER_AGGREGATE_FAULTED;
			gpu_m4_provider_complete_locked(job, error);
		}
		if (job->window_work_ref)
			queue_work(system_unbound_wq, &job->window_work);
		return;
	}
	queue = job->queue;
	vm = gpu_m4_drm_vm_owner_vm(job->batch.vm_snapshot);
	gpu_m4_drm_provider_fault_locked(queue->provider, error);
	gpu_m4_drm_vm_error(vm, error); /* BEFORE any outside-lock error signal */
	first = job->state != GPU_M4_PROVIDER_FAULTED;
	job->state = GPU_M4_PROVIDER_FAULTED;
	if (first)
		queue->runtime.fault_locked(queue->runtime_owner, job, error);
	gpu_m4_provider_complete_locked(job, error);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_fault_locked);

int gpu_m4_drm_provider_job_start_locked(struct gpu_m4_drm_provider_job *job)
{
	struct gpu_m4_drm_provider_queue *queue;
	u64 low, high;
	int ret;

	if (!job)
		return -EINVAL;
	lockdep_assert_held(gpu_m4_provider_job_lock(job));
	if (job->state != GPU_M4_PROVIDER_READY)
		return -EINVAL;
	queue = job->queue;
	gpu_m4_provider_ready_mark(job, GPU_M4_DRM_PROVIDER_READY_START);
	ret = gpu_m4_drm_provider_error_locked(queue->provider);
	if (!ret && queue->closed)
		ret = -ECANCELED;
	/* A VM-local refusal (sticky VM error) stops this READY job BEFORE any
	 * cache preparation or protection: it completes definitely unpublished
	 * with that error below, never through the provider fault hook. Its
	 * already published predecessors keep their exact owners and retire
	 * through the actual hardware/cache predicate.
	 */
	if (!ret)
		ret = gpu_m4_drm_provider_vm_error_locked(queue->vm);
	if (!ret)
		ret = gpu_m4_drm_provider_version_roots_locked(job->version, &low, &high);
#ifdef CONFIG_DEBUG_FS
	gpu_m4_trace(GPU_M4_TRACE_PHASE, 30, 0, 0, 0, 0);
#endif
	if (!ret) {
		ret = gpu_m4_drm_cache_job_prepare(job->cache); /* after ALL deps */
		if (ret) {
			/* Lost cache/backing qualification is terminal even before
			 * this job writes hardware; older running owners stay held.
			 */
			gpu_m4_drm_provider_job_fault_locked(job, ret);
			return ret;
		}
	}
#ifdef CONFIG_DEBUG_FS
	gpu_m4_trace(GPU_M4_TRACE_PHASE, 31, 0, 0, 0, 0);
#endif
	if (!ret)
		ret = gpu_m4_drm_job_status(
			queue->runtime.prepare_locked(queue->runtime_owner, job));
#ifdef CONFIG_DEBUG_FS
	gpu_m4_trace(GPU_M4_TRACE_PHASE, 32, 0, 0, 0, 0);
#endif
	if (ret) {
		job->state = GPU_M4_PROVIDER_FAILED;
		gpu_m4_provider_complete_locked(job, ret);
		return ret; /* no protected owners, no HW-visible write */
	}
	/* Protect EVERY owner before entering an operation that may write roots. */
	ret = gpu_m4_drm_cache_job_publish(job->cache);
	if (ret) {
		job->state = GPU_M4_PROVIDER_FAILED;
		gpu_m4_provider_complete_locked(job, ret);
		return ret;
	}
	job->protected = true;
#ifdef CONFIG_DEBUG_FS
	gpu_m4_trace(GPU_M4_TRACE_PHASE, 33, 0, 0, 0, 0);
#endif
	/* S3c: the tree path's snapshot names no user BO: nothing to publish. */
	ret = job->tree ? 0 : gpu_m4_drm_vm_owner_publish(job->batch.vm_snapshot);
	if (!ret) {
		job->state = GPU_M4_PROVIDER_PUBLISHED;
		ret = gpu_m4_drm_job_status(
			queue->runtime.publish_locked(queue->runtime_owner, job));
	}
	if (ret)
		gpu_m4_drm_provider_job_fault_locked(job, ret);
	else
		gpu_m4_provider_ready_mark(job, GPU_M4_DRM_PROVIDER_READY_PUBLISH);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_start_locked);

int gpu_m4_drm_provider_job_retire_locked(struct gpu_m4_drm_provider_job *job)
{
	struct gpu_m4_drm_provider_queue *queue;
	int ret;

	if (!job)
		return -EINVAL;
	lockdep_assert_held(gpu_m4_provider_job_lock(job));
	if (job->state != GPU_M4_PROVIDER_PUBLISHED)
		return -EINVAL;
	queue = job->queue;
	ret = gpu_m4_drm_provider_error_locked(queue->provider);
	if (ret)
		goto fault;
	ret = queue->runtime.retired_locked(queue->runtime_owner, job);
	if (!ret)
		return -EAGAIN;
	if (ret != 1) {
		ret = ret < 0 ? ret : -EIO;
		goto fault;
	}
	/* GPU-written WB pages become CPU-visible BEFORE real fence success. */
	ret = gpu_m4_drm_cache_job_retire(job->cache);
	if (!ret && !job->tree)
		ret = gpu_m4_drm_vm_owner_retire(job->batch.vm_snapshot);
	/* Tree path: only the TREE version's O(1) work count (ASID quiescence
	 * for a VM switch on the shared ASID 1).
	 */
	if (!ret)
		ret = gpu_m4_drm_provider_version_retire_locked(job->version);
	if (ret)
		goto fault;
	job->state = GPU_M4_PROVIDER_RETIRED;
	/* S3c: the VM guard drops on success BEFORE the fence signals; a
	 * deferred GEM/VM-close removal may follow (provider worker).
	 */
	gpu_m4_provider_guard_retired(job);
	gpu_m4_provider_complete_locked(job, 0);
	return 0;
fault:
	gpu_m4_drm_provider_job_fault_locked(job, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_retire_locked);

int gpu_m4_drm_provider_job_cancel_locked(struct gpu_m4_drm_provider_job *job)
{
	if (!job)
		return -EINVAL;
	lockdep_assert_held(gpu_m4_provider_job_lock(job));
	if (job->protected)
		return -EBUSY;
	if (job->state != GPU_M4_PROVIDER_READY)
		return -EINVAL;
	job->state = GPU_M4_PROVIDER_CANCELED;
	gpu_m4_provider_complete_locked(job, -ECANCELED);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_cancel_locked);

/* Bounded native execution. The aggregate parent is the public scheduler job:
 * it owns the original decoded commands, joined snapshot/version, timestamp
 * leases and common reservation inputs, and never publishes hardware work.
 * Children are real provider jobs, materialized one at a time from that same
 * capture and executed by the existing global lane.
 */
static void gpu_m4_provider_window_wake(struct dma_fence *fence, struct dma_fence_cb *cb)
{
	struct gpu_m4_drm_provider_job *parent =
		container_of(cb, struct gpu_m4_drm_provider_job, window_dependency_cb);

	(void)fence;
	/* Finalization removes an armed callback before dropping window_work_ref. */
	queue_work(system_unbound_wq, &parent->window_work);
}

static int gpu_m4_provider_dependency_status(struct dma_fence *fence)
{
	struct dma_fence_unwrap cursor;
	struct dma_fence *leaf;
	int status = dma_fence_get_status(fence);

	if (status <= 0)
		return status;
	/* Containers do not copy contained errors; complete the iterator. */
	dma_fence_unwrap_for_each(leaf, &cursor, fence) {
		int error = dma_fence_get_status(leaf);

		if (error < 0 && status > 0)
			status = error;
	}
	return status;
}

static int gpu_m4_provider_window_seal(struct gpu_m4_drm_provider_job *parent,
	struct gpu_m4_drm_provider_job *child);

/* Outside every lock. Clone ONLY the original capture: no handle lookup,
 * current VM snapshot or redecode. Returns a runtime-accepted child; sealed,
 * it holds the parent's public finished fence (implicit: its private
 * reservations also contain it). An early first window (!seal, parent not
 * yet run) is sealed by the materializer only after run.
 */
/* The next child's command count from first: up to the queue's execution
 * window, ending before a command whose class (render or compute; others
 * count as compute) already has GPU_M4_DRM_PROVIDER_WINDOW_MAX_PER_CLASS
 * members in it. Pure function of the immutable public batch.
 */
static u32 gpu_m4_provider_window_count(const struct gpu_m4_drm_provider_job *parent, u32 first)
{
	u32 count = 0, per_class[2] = { };
	u32 limit = parent->window_limit ?: parent->queue->execution_window;

	while (first + count < parent->batch.count && count < limit) {
		u32 class = parent->batch.commands[first + count].type == DRM_ASAHI_CMD_RENDER;

		if (per_class[class] >= GPU_M4_DRM_PROVIDER_WINDOW_MAX_PER_CLASS)
			break;
		per_class[class]++;
		count++;
	}
	return count;
}

static struct gpu_m4_drm_provider_job *gpu_m4_provider_window_build(
	struct gpu_m4_drm_provider_job *parent, u32 first, u32 count, bool seal)
{
	struct gpu_m4_drm_provider_queue *queue = parent->queue;
	struct mutex *lock = gpu_m4_provider_job_lock(parent);
	struct gpu_m4_drm_provider_job *child;
	struct gpu_m4_drm_vm_owner *snapshot;
	u64 clock = ktime_get_ns(); /* REMOVE-DEBUG sub-phase timing */
	int ret;

	child = kzalloc(sizeof(*child), GFP_KERNEL);
	if (!child)
		return ERR_PTR(-ENOMEM);
	refcount_set(&child->refs, 1); /* parent's attachment ref */
	child->queue = queue;
	gpu_m4_drm_provider_queue_get(queue);
	child->state = GPU_M4_PROVIDER_PREPARED;
	child->implicit = parent->implicit;
	child->tree = parent->tree;
	/* S3c: each child executes, so it holds its own guard until retired. */
	if (child->tree) {
		ret = gpu_m4_drm_provider_vm_guard_get(queue->vm);
		if (ret) {
			gpu_m4_provider_job_free(child);
			return ERR_PTR(ret);
		}
		child->guarded = true;
	}
	INIT_WORK(&child->window_work, gpu_m4_provider_window_work); /* never queued */
	INIT_WORK(&child->window_put_work, gpu_m4_provider_window_put_work);
	INIT_DELAYED_WORK(&child->window_retry, gpu_m4_provider_window_retry); /* never queued */
	mutex_lock(lock);
	ret = gpu_m4_drm_provider_job_get_locked(parent);
	if (!ret) {
		child->window_parent = parent;
		ret = gpu_m4_drm_provider_version_get_locked(parent->version);
		if (!ret)
			child->version = parent->version;
	}
	mutex_unlock(lock);
	if (ret)
		goto fail;
	/* Borrowed through the parent ref; job_free never frees a child's bytes. */
	child->bytes = parent->bytes;
	child->size = parent->size;
	child->command_first = first;
	child->command_total = parent->batch.count;
	ret = gpu_m4_provider_retirement_init(child);
	if (ret)
		goto fail;
	child->commands = kvcalloc(count, sizeof(*child->commands), GFP_KERNEL);
	if (!child->commands) {
		ret = -ENOMEM;
		goto fail;
	}
	/* Timestamp slots are copies of parent-owned leases; put_timestamp skips
	 * them for a child. Original barrier ordinals are retained unchanged.
	 */
	memcpy(child->commands, parent->commands + first, count * sizeof(*child->commands));
	child->decode = parent->decode;
	child->decode.job.cookie = child;
	snapshot = gpu_m4_drm_vm_owner_clone_with_extras(parent->batch.vm_snapshot, NULL, 0);
	if (IS_ERR(snapshot)) {
		ret = PTR_ERR(snapshot);
		goto fail;
	}
	child->batch = (struct gpu_m4_drm_mixed_batch) {
		.vm_snapshot = snapshot, .commands = child->commands, .count = count,
	};
	gpu_m4_provider_phase_mark(GPU_M4_PROVIDER_PHASE_WINDOW_CLONE, &clock);
	ret = gpu_m4_provider_stage_native(child);
	if (ret)
		goto fail;
	gpu_m4_provider_phase_mark(GPU_M4_PROVIDER_PHASE_WINDOW_STAGE, &clock);
	ret = gpu_m4_provider_cache_create(child);
	if (ret)
		goto fail;
	gpu_m4_provider_phase_mark(GPU_M4_PROVIDER_PHASE_WINDOW_CACHE, &clock);
	mutex_lock(lock);
	ret = gpu_m4_drm_provider_error_locked(queue->provider);
	/* An early build may see run arrive meanwhile; a released one never. */
	if (!ret && (queue->closed || parent->window_released ||
	    (parent->state != GPU_M4_PROVIDER_AGGREGATE &&
	     (seal || parent->state != GPU_M4_PROVIDER_PREPARED))))
		ret = -ECANCELED;
	if (!ret)
		ret = gpu_m4_drm_job_status(queue->runtime.accept_locked(queue->runtime_owner,
			queue->runtime_queue, child, &child->runtime_job));
	if (!ret && !child->runtime_job)
		ret = -EIO;
	mutex_unlock(lock);
	if (ret)
		goto fail;
	gpu_m4_provider_phase_mark(GPU_M4_PROVIDER_PHASE_WINDOW_ACCEPT, &clock);
	if (seal) {
		ret = gpu_m4_provider_window_seal(parent, child);
		if (ret)
			goto fail;
		gpu_m4_provider_phase_mark(GPU_M4_PROVIDER_PHASE_WINDOW_SEAL, &clock);
	}
	return child;
fail:
	if (ret == -ENOSPC || ret == -ENOMEM)
		pr_warn("GPU execution window capacity: first=%u commands=%u total=%u error=%d\n",
			first, count, parent->batch.count, ret);
	gpu_m4_provider_job_free(child);
	return ERR_PTR(ret);
}

/* The accepted child's LAST step, outside every lock. Explicit sync: it only
 * takes the parent's public finished fence; nothing is reserved, and reuse of
 * its private native units is ordered by unit selection (only idle, displaced
 * units whose last use completed its full receipt), not by a BO fence.
 * Implicit (old path): inherited reservations, after which its own retirement
 * fence guards its private native leaders. An early first window stays
 * unsealed until its parent ran, so no fence of a batch whose dependencies
 * are pending is ever captured by a later user of a reused unit, and
 * dropping it signals no orphan there.
 */
static int gpu_m4_provider_window_seal(struct gpu_m4_drm_provider_job *parent,
	struct gpu_m4_drm_provider_job *child)
{
	struct mutex *lock = gpu_m4_provider_job_lock(parent);
	const struct gpu_m4_drm_resv_access *resv;
	struct gpu_m4_drm_resv_job *owner = NULL;
	u32 n;

	if (child->implicit) {
		resv = gpu_m4_drm_vm_owner_resv_access(child->batch.vm_snapshot, &n);
		owner = gpu_m4_drm_resv_prepare_inherited(parent->resv, resv, n);
		if (IS_ERR(owner))
			return PTR_ERR(owner);
		/* Public fence stays on common reservations only. New kernel-private
		 * leaders get this child's real retirement fence; unlocks every ww lock.
		 */
		gpu_m4_drm_resv_publish_inherited(owner, &child->retirement->base);
	}
	mutex_lock(lock);
	child->resv = owner;
	child->finished = dma_fence_get(parent->finished);
	child->accepted = true;
	mutex_unlock(lock);
	return 0;
}

/* 0 when every captured child input completed successfully, -EINPROGRESS
 * while one callback is armed. Only the non-reentrant work touches these
 * fields; no lock is held and no wait blocks a worker or command deadline.
 */
static int gpu_m4_provider_window_dependencies(struct gpu_m4_drm_provider_job *parent,
	struct gpu_m4_drm_provider_job *child)
{
	struct dma_fence *fence;
	int ret;

	for (;;) {
		fence = parent->window_dependency;
		if (fence && parent->window_armed) {
			if (!dma_fence_is_signaled(fence))
				return -EINPROGRESS;
			dma_fence_remove_callback(fence, &parent->window_dependency_cb);
			parent->window_armed = false;
		}
		if (fence) {
			if (!dma_fence_add_callback(fence, &parent->window_dependency_cb,
						    gpu_m4_provider_window_wake)) {
				parent->window_armed = true;
				return -EINPROGRESS;
			}
			ret = gpu_m4_provider_dependency_status(fence);
			parent->window_dependency = NULL;
			dma_fence_put(fence);
			if (ret < 0)
				return ret;
			if (!ret)
				return -EIO;
		}
		/* Explicit sync: the parent's scheduler inputs were the only ones. */
		fence = child->implicit ? gpu_m4_drm_resv_next_dependency(child->resv) : NULL;
		if (IS_ERR(fence))
			return PTR_ERR(fence);
		if (!fence)
			return 0;
		parent->window_dependency = fence;
	}
}

static int gpu_m4_provider_window_enqueue_locked(struct gpu_m4_drm_provider_job *parent,
	struct gpu_m4_drm_provider_job *child)
{
	struct gpu_m4_drm_provider_queue *queue = parent->queue;
	int ret;

	lockdep_assert_held(gpu_m4_provider_job_lock(parent));
	ret = gpu_m4_drm_provider_error_locked(queue->provider);
	if (!ret)
		ret = gpu_m4_drm_provider_vm_error_locked(queue->vm);
	if (ret)
		return gpu_m4_drm_job_status(ret);
	if (queue->closed || parent->state != GPU_M4_PROVIDER_AGGREGATE ||
	    parent->retirement->ready)
		return -ECANCELED;
	if ((parent->active_window != child && parent->next_window != child) ||
	    child->window_parent != parent ||
	    !child->accepted || child->window_enqueued || child->state != GPU_M4_PROVIDER_PREPARED)
		return -ESTALE;
	/* Exactly like run(): one transferred ref, READY, global lane FIFO. */
	child->state = GPU_M4_PROVIDER_READY;
	gpu_m4_drm_provider_job_get_locked(child);
	child->window_enqueued = true;
	if (child->command_first + child->batch.count == parent->batch.count) {
		/* Its last child is in the lane FIFO: a later running batch of this
		 * queue may now materialize behind it, in queue order.
		 */
		parent->window_final = true;
		gpu_m4_provider_aggregate_wake_locked(queue, parent);
	}
	/* REMOVE-DEBUG: only the first window measures dependency-ready. */
	if (parent->window_ready_ns) {
		gpu_m4_provider_ready_add(parent->window_early_used,
			GPU_M4_DRM_PROVIDER_READY_ENQUEUE, parent->window_ready_ns);
		child->window_ready_ns = parent->window_ready_ns;
		child->window_early_used = parent->window_early_used;
		parent->window_ready_ns = 0;
	}
	queue->runtime.enqueue_locked(queue->runtime_owner, child);
	return 0;
}

/* Complete the aggregate once, detach its child (breaking the ref cycle) and
 * drop every materializer ref outside locks. A child uncertain owner keeps its
 * own lane ref and therefore this parent; nothing here faults the runtime.
 */
/* Detached, accepted, never-enqueued child owned by the caller. Commit and
 * signal the local error (never success or cache retirement) before the
 * ordinary disposal: implicit, its private reservations already hold its
 * retirement fence, so a later user of those native units cannot wait on an
 * orphan fence.
 */
static void gpu_m4_provider_window_abort(struct gpu_m4_drm_provider_job *child, int error)
{
	struct mutex *lock = gpu_m4_provider_job_lock(child);

	mutex_lock(lock);
	if (!WARN_ON(child->window_enqueued || child->protected ||
		     child->window_parent->active_window == child ||
		     child->window_parent->next_window == child ||
		     child->state != GPU_M4_PROVIDER_PREPARED)) {
		child->state = GPU_M4_PROVIDER_CANCELED;
		gpu_m4_provider_complete_locked(child, error < 0 ? error : -ECANCELED);
	}
	mutex_unlock(lock);
	WARN_ON(gpu_m4_drm_provider_job_signal(child));
	WARN_ON(gpu_m4_drm_provider_job_put(child));
}

/* Final materializer-ref owner: no producer may queue window_work after
 * terminalization, and an already queued rerun is drained before the put.
 */
static void gpu_m4_provider_window_put_work(struct work_struct *work)
{
	struct gpu_m4_drm_provider_job *parent =
		container_of(work, struct gpu_m4_drm_provider_job, window_put_work);

	/* A pending capacity retry would queue window_work: drain it first. */
	cancel_delayed_work_sync(&parent->window_retry);
	cancel_work_sync(&parent->window_work);
	WARN_ON(gpu_m4_drm_provider_job_put(parent));
}

/* Holds no ref: window_put_work drains this before the parent's final put. */
static void gpu_m4_provider_window_retry(struct work_struct *work)
{
	struct gpu_m4_drm_provider_job *parent = container_of(to_delayed_work(work),
		struct gpu_m4_drm_provider_job, window_retry);

	queue_work(system_unbound_wq, &parent->window_work);
}

static void gpu_m4_provider_window_finish(struct gpu_m4_drm_provider_job *parent,
	int error, bool faulted)
{
	struct mutex *lock = gpu_m4_provider_job_lock(parent);
	struct gpu_m4_drm_provider_job *child, *next;
	bool enqueued = false, next_enqueued = false;

	mutex_lock(lock);
	if (!parent->retirement->ready) {
		parent->state = !error ? GPU_M4_PROVIDER_AGGREGATE_DONE : faulted ?
			GPU_M4_PROVIDER_AGGREGATE_FAULTED : GPU_M4_PROVIDER_AGGREGATE_FAILED;
		/* S3c: every child retired (each dropped its own guard): the
		 * public batch's guard drops before its fence signals. A failed
		 * batch's published child keeps its own guard if quarantined.
		 */
		if (!error)
			gpu_m4_provider_guard_retired(parent);
		gpu_m4_provider_complete_locked(parent, error);
	}
	child = parent->active_window;
	if (child)
		enqueued = child->window_enqueued;
	parent->active_window = NULL;
	next = parent->next_window;
	if (next)
		next_enqueued = next->window_enqueued;
	parent->next_window = NULL;
	parent->window_work_ref = false;
	gpu_m4_provider_aggregate_remove_locked(parent->queue, parent);
	gpu_m4_provider_early_unregister_locked(parent);
	mutex_unlock(lock);
	if (parent->window_dependency) {
		if (parent->window_armed)
			dma_fence_remove_callback(parent->window_dependency,
						  &parent->window_dependency_cb);
		parent->window_armed = false;
		dma_fence_put(parent->window_dependency);
		parent->window_dependency = NULL;
	}
	if (child && enqueued)
		WARN_ON(gpu_m4_drm_provider_job_put(child) && !faulted);
	else if (child)
		gpu_m4_provider_window_abort(child, error);
	/* A prefetched child already in the lane FIFO keeps its own lane ref
	 * (it fails at start on a failed VM/provider); an unenqueued one is
	 * dropped unpublished.
	 */
	if (next && next_enqueued)
		WARN_ON(gpu_m4_drm_provider_job_put(next) && !faulted);
	else if (next)
		gpu_m4_provider_window_abort(next, error);
	WARN_ON(gpu_m4_drm_provider_job_signal(parent));
	/* Transfer window_work_ref: this may run inside window_work itself. */
	queue_work(system_unbound_wq, &parent->window_put_work);
}

/* Build the child starting at first while the active one executes and
 * enqueue it behind it. Any refusal (capacity, closure, failure) leaves the
 * ordinary build after the active child's completion to retry; nothing is
 * published here and a built but unenqueued child is dropped unpublished.
 */
static void gpu_m4_provider_window_prefetch(struct gpu_m4_drm_provider_job *parent, u32 first)
{
	struct mutex *lock = gpu_m4_provider_job_lock(parent);
	struct gpu_m4_drm_provider_job *child;
	u64 start = ktime_get_ns();
	int ret;

	child = gpu_m4_provider_window_build(parent, first,
		gpu_m4_provider_window_count(parent, first), true);
	gpu_m4_provider_stat_add(3, start);
	if (IS_ERR(child)) {
		atomic64_inc(&gpu_m4_provider_prefetch_stat[1]);
		return;
	}
	mutex_lock(lock);
	ret = parent->retirement->ready || parent->next_window || !parent->active_window ||
	      !parent->active_window->window_enqueued ||
	      parent->window_cursor + parent->active_window->batch.count != first ? -ECANCELED : 0;
	if (!ret)
		parent->next_window = child;
	/* Explicit sync: the parent's scheduler inputs were its only inputs. */
	if (!ret && child->implicit)
		ret = -EOPNOTSUPP;
	if (!ret)
		ret = gpu_m4_provider_window_enqueue_locked(parent, child);
	if (ret && parent->next_window == child)
		parent->next_window = NULL;
	mutex_unlock(lock);
	if (ret) {
		atomic64_inc(&gpu_m4_provider_prefetch_stat[1]);
		gpu_m4_provider_window_abort(child, ret);
		return;
	}
	atomic64_inc(&gpu_m4_provider_prefetch_stat[0]);
}

/* Never-run phase of an early materializer (begin_dependencies). It builds
 * the first window once, unsealed, and parks it on active_window. Closure,
 * provider or VM failure and another window's capacity eviction drop that
 * child unpublished and degrade this batch to the lazy build at run; only
 * the scheduler's release of a never-run job finalizes here. A refused build
 * never retries early: run's ordinary bounded retry owns capacity waits.
 */
static void gpu_m4_provider_window_early(struct gpu_m4_drm_provider_job *parent)
{
	struct gpu_m4_drm_provider_queue *queue = parent->queue;
	struct mutex *lock = gpu_m4_provider_job_lock(parent);
	struct gpu_m4_drm_provider_job *child = NULL;
	enum gpu_m4_drm_provider_early_stat stat = GPU_M4_DRM_PROVIDER_EARLY_CANCELED;
	u64 start;
	u32 count;
	int ret;

	mutex_lock(lock);
	if (!parent->window_work_ref || parent->state != GPU_M4_PROVIDER_PREPARED) {
		mutex_unlock(lock); /* run queued the ordinary materializer */
		return;
	}
	if (parent->window_released) {
		if (parent->window_early || parent->active_window)
			gpu_m4_provider_early_add(GPU_M4_DRM_PROVIDER_EARLY_CANCELED, 1);
		mutex_unlock(lock);
		gpu_m4_provider_window_finish(parent, -ECANCELED, false);
		return;
	}
	ret = gpu_m4_drm_job_status(gpu_m4_drm_provider_error_locked(queue->provider));
	if (!ret && queue->closed)
		ret = -ECANCELED;
	if (!ret)
		ret = gpu_m4_drm_provider_vm_error_locked(queue->vm);
	if (!ret && parent->window_evict) {
		ret = -ENOSPC;
		stat = GPU_M4_DRM_PROVIDER_EARLY_EVICTED;
	}
	if (ret || !parent->window_early || parent->active_window) {
		/* Degrade: keep only the materializer ref for run or release. */
		if (ret && parent->window_early) {
			child = parent->active_window;
			parent->active_window = NULL;
			gpu_m4_provider_early_unregister_locked(parent);
			gpu_m4_provider_early_add(stat, 1);
		}
		mutex_unlock(lock);
		if (child)
			gpu_m4_provider_window_abort(child, ret);
		return;
	}
	count = gpu_m4_provider_window_count(parent, 0);
	mutex_unlock(lock);
	start = ktime_get_ns();
	child = gpu_m4_provider_window_build(parent, 0, count, false);
	gpu_m4_provider_stat_add(3, start);
	mutex_lock(lock);
	if (IS_ERR(child)) {
		/* A capacity refusal also opens the quiet period: the lazy
		 * build at run (with its bounded backoff) owns the wait.
		 */
		ret = PTR_ERR(child);
		if (ret == -EAGAIN || gpu_m4_drm_provider_capacity_error(ret)) {
			atomic64_set(&gpu_m4_provider_early_pressure, ktime_get_ns() + 1);
			stat = GPU_M4_DRM_PROVIDER_EARLY_FALLBACKS;
		}
		if (parent->state == GPU_M4_PROVIDER_PREPARED)
			gpu_m4_provider_early_unregister_locked(parent);
		mutex_unlock(lock);
		gpu_m4_provider_early_add(stat, 1);
		return;
	}
	/* Attach while still never run (and not dropped meanwhile), or to a
	 * batch that ran during the build: its queued materializer seals it.
	 */
	ret = parent->retirement->ready || parent->window_released || queue->closed ? -ECANCELED : 0;
	if (!ret && parent->state == GPU_M4_PROVIDER_PREPARED && parent->window_evict)
		ret = -ENOSPC;
	if (!ret && (parent->state == GPU_M4_PROVIDER_PREPARED ||
		     parent->state == GPU_M4_PROVIDER_AGGREGATE) && !parent->active_window) {
		parent->active_window = child;
		parent->window_built_ns = ktime_get_ns();
		gpu_m4_provider_early_add(GPU_M4_DRM_PROVIDER_EARLY_BUILT, 1);
		if (parent->state == GPU_M4_PROVIDER_AGGREGATE) {
			parent->window_early_used = true;
			gpu_m4_provider_early_add(GPU_M4_DRM_PROVIDER_EARLY_LATE, 1);
			gpu_m4_provider_early_add(GPU_M4_DRM_PROVIDER_EARLY_LATE_NS,
				parent->window_built_ns - parent->window_ready_ns);
		}
		mutex_unlock(lock);
		return;
	}
	if (parent->state == GPU_M4_PROVIDER_PREPARED) {
		gpu_m4_provider_early_unregister_locked(parent);
		gpu_m4_provider_early_add(ret == -ENOSPC ? GPU_M4_DRM_PROVIDER_EARLY_EVICTED :
			GPU_M4_DRM_PROVIDER_EARLY_CANCELED, 1);
	}
	mutex_unlock(lock);
	gpu_m4_provider_window_abort(child, ret ?: -ECANCELED);
}

static void gpu_m4_provider_window_work(struct work_struct *work)
{
	struct gpu_m4_drm_provider_job *parent =
		container_of(work, struct gpu_m4_drm_provider_job, window_work);
	struct gpu_m4_drm_provider_queue *queue = parent->queue;
	struct mutex *lock = gpu_m4_provider_job_lock(parent);
	struct gpu_m4_drm_provider_job *child, *drop = NULL;
	bool done, faulted = false;
	u32 first, count;
	int ret = 0;

#ifdef CONFIG_DEBUG_FS
	gpu_m4_trace(GPU_M4_TRACE_PHASE, 0, (u64)(uintptr_t)queue->runtime_queue,
		(u64)(uintptr_t)parent, 0, 0);
#endif
	mutex_lock(lock);
	if (!parent->window_work_ref) {
		mutex_unlock(lock); /* spurious wakeup after finalization */
		return;
	}
	if (parent->state == GPU_M4_PROVIDER_PREPARED) {
		mutex_unlock(lock); /* never run: early first window only */
		gpu_m4_provider_window_early(parent);
		return;
	}
	if (parent->retirement->ready) {
		/* Already failed by the scheduler timeout path, or (two credits)
		 * completed at its final child's receipt.
		 */
		int error = parent->state == GPU_M4_PROVIDER_AGGREGATE_DONE ? 0 :
			parent->retirement->error ?: -EIO;

		mutex_unlock(lock);
		gpu_m4_provider_window_finish(parent, error, false);
		return;
	}
again:
	child = parent->active_window;
	if (child && child->window_enqueued) {
		if (child->state == GPU_M4_PROVIDER_RETIRED && child->window_complete) {
			/* Actual CORE/cache retirement plus root receipt bookkeeping. */
			parent->window_cursor += child->batch.count;
			parent->active_window = NULL;
			if (drop) {
				mutex_unlock(lock);
				WARN_ON(gpu_m4_drm_provider_job_put(drop));
				mutex_lock(lock);
			}
			drop = child;
			/* The prefetched successor is already in the lane FIFO. */
			if (parent->next_window) {
				parent->active_window = parent->next_window;
				parent->next_window = NULL;
				goto again;
			}
		} else if (child->retirement->ready) {
			ret = child->retirement->error ?: -EIO;
			faulted = child->state == GPU_M4_PROVIDER_FAULTED;
		} else if (READ_ONCE(window_prefetch) && !parent->next_window &&
			   parent->window_cursor + child->batch.count < parent->batch.count &&
			   !queue->closed && !gpu_m4_drm_provider_error_locked(queue->provider) &&
			   !gpu_m4_drm_provider_vm_error_locked(queue->vm)) {
			/* Still executing: build and enqueue the next child now. */
			first = parent->window_cursor + child->batch.count;
			mutex_unlock(lock);
			if (drop)
				WARN_ON(gpu_m4_drm_provider_job_put(drop));
			gpu_m4_provider_window_prefetch(parent, first);
			return;
		} else {
			mutex_unlock(lock); /* still executing on the global lane */
			if (drop)
				WARN_ON(gpu_m4_drm_provider_job_put(drop));
			return;
		}
		child = NULL;
	}
	if (!ret)
		ret = gpu_m4_drm_job_status(gpu_m4_drm_provider_error_locked(queue->provider));
	done = !ret && !child && parent->window_cursor == parent->batch.count;
	/* An unpublished remainder is canceled locally; a published child above
	 * still retired normally. NEW allocation requires an OPEN namespace, and
	 * a VM-local refusal ends the remainder with that VM's own error.
	 */
	if (!ret && !done && queue->closed)
		ret = -ECANCELED;
	if (!ret && !done)
		ret = gpu_m4_drm_provider_vm_error_locked(queue->vm);
	first = parent->window_cursor;
	count = gpu_m4_provider_window_count(parent, first);
	if (!ret && !done && !child && !first && !gpu_m4_provider_aggregate_turn_locked(parent)) {
		/* An earlier running batch of this queue still materializes: its
		 * final child's enqueue (or its end) wakes this one.
		 */
		mutex_unlock(lock);
		if (drop)
			WARN_ON(gpu_m4_drm_provider_job_put(drop));
		return;
	}
	mutex_unlock(lock);
	if (drop)
		WARN_ON(gpu_m4_drm_provider_job_put(drop));
	if (ret || done) {
		gpu_m4_provider_window_finish(parent, ret, faulted);
		return;
	}
	/* An early first window is sealed only now, after run. Refused, it is
	 * dropped unpublished and the ordinary bounded build below takes over.
	 */
	if (child && !child->accepted) {
		ret = gpu_m4_provider_window_seal(parent, child);
		if (ret) {
			mutex_lock(lock);
			parent->active_window = NULL;
			parent->window_early_used = false;
			mutex_unlock(lock);
			gpu_m4_provider_window_abort(child, ret);
			gpu_m4_provider_early_add(GPU_M4_DRM_PROVIDER_EARLY_FALLBACKS, 1);
			child = NULL;
			ret = 0;
		}
	}
	if (!child) {
		u64 start = ktime_get_ns();

#ifdef CONFIG_DEBUG_FS
		gpu_m4_trace(GPU_M4_TRACE_PHASE, 1, (u64)(uintptr_t)queue->runtime_queue,
			(u64)(uintptr_t)parent, first, count);
#endif
		child = gpu_m4_provider_window_build(parent, first, count, true);
#ifdef CONFIG_DEBUG_FS
		gpu_m4_trace(GPU_M4_TRACE_PHASE, 2, (u64)(uintptr_t)queue->runtime_queue,
			(u64)(uintptr_t)parent, first, IS_ERR(child) ? PTR_ERR(child) : 0);
#endif
		gpu_m4_provider_stat_add(3, start);
		/* A refused multi-command build splits: the next attempt builds
		 * half as many commands (after one reclaim pass for a capacity
		 * refusal, after two waits for cell/byte returns), so a window
		 * that cannot fit never fails while a smaller one could.
		 */
		if (IS_ERR(child) && count > 1 &&
		    ((gpu_m4_drm_provider_capacity_error(PTR_ERR(child)) &&
		      parent->window_capacity_retries >= 1) ||
		     (PTR_ERR(child) == -EAGAIN && parent->window_retries >= 2))) {
			parent->window_limit = count / 2;
			parent->window_retries = 0;
			parent->window_capacity_retries = 0;
			gpu_m4_provider_proof_add(GPU_M4_DRM_PROVIDER_PROOF_CAPACITY_SPLITS, 1);
			gpu_m4_provider_early_evict(parent);
			queue->runtime.reap(queue->runtime_owner);
			queue_work(system_unbound_wq, &parent->window_work);
			return;
		}
		if (IS_ERR(child) && PTR_ERR(child) == -EAGAIN) {
			/* Native capacity backpressure: the failed build rolled back
			 * every partial unit/cell credit. Rerun later; the top of this
			 * work rechecks timeout, provider failure and queue closure.
			 * Blocked early windows give their capacity back first.
			 */
			bool slow = parent->window_retries >= GPU_M4_PROVIDER_WINDOW_RETRIES;

			if (!slow)
				parent->window_retries++;
			gpu_m4_provider_early_evict(parent);
			queue_delayed_work(system_unbound_wq, &parent->window_retry,
				msecs_to_jiffies(slow ? GPU_M4_PROVIDER_WINDOW_RETRY_SLOW_MS :
						 GPU_M4_PROVIDER_WINDOW_RETRY_MS));
			return;
		}
		if (IS_ERR(child) && gpu_m4_drm_provider_capacity_error(PTR_ERR(child)) &&
		    parent->window_capacity_retries < GPU_M4_PROVIDER_CAPACITY_RETRIES) {
			/* The refused build rolled back every partial owner and charge
			 * and published nothing. Reclaim, then rebuild the same window;
			 * the rerun rechecks timeout, provider failure and closure.
			 */
			parent->window_capacity_retries++;
			gpu_m4_provider_proof_add(GPU_M4_DRM_PROVIDER_PROOF_CAPACITY_RETRIES, 1);
			gpu_m4_provider_early_evict(parent);
			queue->runtime.reap(queue->runtime_owner);
			queue_delayed_work(system_unbound_wq, &parent->window_retry,
				msecs_to_jiffies(GPU_M4_PROVIDER_WINDOW_RETRY_MS));
			return;
		}
		if (IS_ERR(child)) {
			ret = PTR_ERR(child);
			/* Persistent refusal: only this public batch's fence fails. */
			if (gpu_m4_drm_provider_capacity_error(ret))
				gpu_m4_provider_proof_add(GPU_M4_DRM_PROVIDER_PROOF_CAPACITY_REFUSALS, 1);
			gpu_m4_provider_window_finish(parent, ret, false);
			return;
		}
		parent->window_retries = 0;
		parent->window_capacity_retries = 0;
		mutex_lock(lock);
		ret = parent->retirement->ready ? -ECANCELED : 0;
		if (!ret)
			parent->active_window = child;
		mutex_unlock(lock);
		if (ret) {
			gpu_m4_provider_window_abort(child, ret);
			gpu_m4_provider_window_finish(parent, ret, false);
			return;
		}
	}
	ret = gpu_m4_provider_window_dependencies(parent, child);
	if (ret == -EINPROGRESS)
		return;
	if (!ret) {
		mutex_lock(lock);
		ret = gpu_m4_provider_window_enqueue_locked(parent, child);
		/* Prefetch the following child now, while this one waits and
		 * executes, instead of only at a later wake (its completion).
		 */
		if (!ret && READ_ONCE(window_prefetch_kick) && READ_ONCE(window_prefetch) &&
		    parent->active_window == child &&
		    parent->window_cursor + child->batch.count < parent->batch.count)
			queue_work(system_unbound_wq, &parent->window_work);
		mutex_unlock(lock);
	}
#ifdef CONFIG_DEBUG_FS
	gpu_m4_trace(GPU_M4_TRACE_PHASE, 3, (u64)(uintptr_t)queue->runtime_queue,
		(u64)(uintptr_t)parent, ret, 0);
#endif
	if (ret)
		gpu_m4_provider_window_finish(parent, ret, false);
}

struct gpu_m4_drm_provider_job *gpu_m4_drm_provider_job_active_window_locked(
	struct gpu_m4_drm_provider_job *parent)
{
	struct gpu_m4_drm_provider_job *child;
	int ret;

	if (IS_ERR_OR_NULL(parent) || IS_ERR_OR_NULL(parent->queue) || !parent->aggregate)
		return ERR_PTR(-EINVAL);
	lockdep_assert_held(gpu_m4_provider_job_lock(parent));
	ret = gpu_m4_drm_provider_error_locked(parent->queue->provider);
	if (ret)
		return ERR_PTR(gpu_m4_drm_job_status(ret));
	/* Local completion/cancellation is drained by the outside worker. */
	if (parent->state == GPU_M4_PROVIDER_AGGREGATE_DONE ||
	    parent->state == GPU_M4_PROVIDER_AGGREGATE_FAILED)
		return NULL;
	if (parent->state != GPU_M4_PROVIDER_AGGREGATE || parent->retirement->ready)
		return ERR_PTR(parent->retirement->error ?: -EIO);
	child = parent->active_window;
	/* Materialization/dependency/completion intervals are CPU-side and healthy. */
	if (!child || !child->window_enqueued || child->window_complete)
		return NULL;
	return child;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_active_window_locked);

int gpu_m4_drm_provider_job_window_complete_locked(struct gpu_m4_drm_provider_job *child)
{
	struct gpu_m4_drm_provider_job *parent;

	if (!child)
		return -EINVAL;
	lockdep_assert_held(gpu_m4_provider_job_lock(child));
	parent = child->window_parent;
	if (!parent)
		return 0;
	if ((parent->active_window != child && parent->next_window != child) ||
	    !child->window_enqueued || child->window_complete ||
	    child->state != GPU_M4_PROVIDER_RETIRED || !child->retirement->ready ||
	    child->retirement->error)
		return -ESTALE;
	child->window_complete = true;
	if (child->command_first + child->batch.count == parent->batch.count) {
		parent->final_ns = ktime_get_ns(); /* REMOVE-DEBUG pacing */
		/* Two credits (inflight2): the final child's actual CORE/cache
		 * receipt completes its public batch here, so the fence signals
		 * with the child's drain; the materializer only finalizes. The
		 * guard drops first, as in window_finish().
		 */
		if (parent->queue->credit_limit > 1 && parent->state == GPU_M4_PROVIDER_AGGREGATE &&
		    !parent->retirement->ready) {
			parent->state = GPU_M4_PROVIDER_AGGREGATE_DONE;
			gpu_m4_provider_guard_retired(parent);
			gpu_m4_provider_complete_locked(parent, 0);
			/* Its credit returns with the signal: the next batch may
			 * run before this one's finalization; it no longer orders
			 * anything (its final child is done, so no later batch
			 * waits on its turn: no wake).
			 */
			gpu_m4_provider_aggregate_unlink_locked(parent->queue, parent);
		}
	}
	queue_work(system_unbound_wq, &parent->window_work);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_job_window_complete_locked);

static void gpu_m4_provider_fault(void *cookie, void *owner, int error)
{
	struct gpu_m4_drm_provider_job *job = owner;
	struct mutex *lock = gpu_m4_provider_job_lock(job);

	(void)cookie;
	mutex_lock(lock);
	gpu_m4_drm_provider_job_fault_locked(job, error);
	mutex_unlock(lock);
}

/* The helper refused to run an admitted job (input error, cancellation). */
static void gpu_m4_provider_fail(void *cookie, void *owner, int error)
{
	struct gpu_m4_drm_provider_job *job = owner;
	struct mutex *lock = gpu_m4_provider_job_lock(job);

	(void)cookie;
	mutex_lock(lock);
	/* REMOVE-DEBUG: why a job the helper refused latched its VM. */
	pr_warn_ratelimited("GPU job refused at run: error=%d state=%u\n", error, job->state);
	gpu_m4_drm_provider_vm_quarantine_locked(job->queue->vm, error);
	mutex_unlock(lock);
}

/*
 * A signaled input/implicit dependency failed. Per-client isolation: another
 * VM's, file's or a foreign producer's failure never ends this job (nor
 * latches its VM); it runs on whatever that producer left. This VM's own
 * failed job latched the VM before its fence signaled, so a latched VM (or a
 * failed shared domain) is what marks a failure as ours: refuse then.
 */
static int gpu_m4_provider_dependency_error(void *cookie, void *owner, int error)
{
	struct gpu_m4_drm_provider_job *job = owner;
	struct gpu_m4_drm_provider_queue *queue = job->queue;
	struct mutex *lock = gpu_m4_provider_job_lock(job);
	int ret;

	(void)cookie;
	mutex_lock(lock);
	ret = gpu_m4_drm_provider_error_locked(queue->provider) ?:
		gpu_m4_drm_provider_vm_status_locked(queue->vm);
	mutex_unlock(lock);
	/* REMOVE-DEBUG */
	pr_warn_ratelimited("GPU job dependency failed: error=%d own=%d\n", error, ret);
	return ret ? error : 0;
}

static enum drm_gpu_sched_stat gpu_m4_provider_timedout(void *cookie, void *owner,
						      struct drm_sched_job *sched_job)
{
	struct gpu_m4_drm_provider_queue *queue = cookie;

	return queue->runtime.timedout(queue->runtime_owner, owner, sched_job);
}

static void gpu_m4_provider_release(void *cookie, void *owner, bool uncertain)
{
	struct gpu_m4_drm_provider_job *job = owner;
	struct mutex *lock = gpu_m4_provider_job_lock(job);

	(void)cookie;
	mutex_lock(lock);
	gpu_m4_provider_waiting_done(job); /* REMOVE-DEBUG */
	if (uncertain && job->protected && job->state != GPU_M4_PROVIDER_RETIRED)
		gpu_m4_drm_provider_job_fault_locked(job, -EIO);
	/* Never run (close, dependency error): an early materializer still
	 * holding its ref aborts any unsealed child and finalizes.
	 */
	if (job->aggregate && job->state == GPU_M4_PROVIDER_PREPARED && job->window_work_ref) {
		job->window_released = true;
		queue_work(system_unbound_wq, &job->window_work);
	}
	mutex_unlock(lock);
	/* Runtime still holds its own ref to READY/published/ambiguous jobs. */
	WARN_ON(gpu_m4_drm_provider_job_put(job));
}

static void gpu_m4_provider_closed(void *cookie)
{
	gpu_m4_drm_provider_queue_put(cookie); /* scheduler-owned queue ref */
}

static const struct gpu_m4_drm_sched_ops gpu_m4_provider_sched_ops = {
	.prepare = gpu_m4_provider_prepare, .publish_finished = gpu_m4_provider_publish_finished,
	.next_dependency = gpu_m4_provider_next_dependency,
	.begin_dependencies = gpu_m4_provider_begin_dependencies, .run = gpu_m4_provider_run,
	.fault = gpu_m4_provider_fault, .fail = gpu_m4_provider_fail,
	.dependency_error = gpu_m4_provider_dependency_error,
	.timedout = gpu_m4_provider_timedout,
	.release = gpu_m4_provider_release, .closed = gpu_m4_provider_closed,
};

static void gpu_m4_provider_queue_free(struct kref *ref)
{
	struct gpu_m4_drm_provider_queue *queue =
		container_of(ref, struct gpu_m4_drm_provider_queue, ref);
	struct gpu_m4_drm_provider *provider = queue->provider;
	struct mutex *lock = gpu_m4_drm_provider_lock(provider);

	if (queue->scheduler)
		gpu_m4_drm_sched_queue_put(queue->scheduler);
	mutex_lock(lock);
	if (queue->runtime_held)
		queue->runtime.queue_put_locked(queue->runtime_owner, queue->runtime_queue);
	if (queue->private_candidate)
		WARN_ON(gpu_m4_drm_provider_version_put_locked(queue->private_candidate));
	if (queue->vm_held)
		gpu_m4_drm_provider_vm_put_locked(queue->vm);
	mutex_unlock(lock);
	queue->runtime.reap(queue->runtime_owner);
	kfree(queue);
	WARN_ON(gpu_m4_drm_provider_put(provider));
}

void gpu_m4_drm_provider_queue_get(struct gpu_m4_drm_provider_queue *queue)
{
	kref_get(&queue->ref);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_queue_get);

void gpu_m4_drm_provider_queue_put(struct gpu_m4_drm_provider_queue *queue)
{
	if (queue)
		kref_put(&queue->ref, gpu_m4_provider_queue_free);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_queue_put);

static struct gpu_m4_drm_provider_queue *gpu_m4_provider_queue_create(
	const struct gpu_m4_drm_provider_queue_config *config,
	struct gpu_m4_drm_provider_version *candidate)
{
	const struct gpu_m4_drm_provider_job_runtime *ops;
	struct gpu_m4_drm_provider_queue *queue;
	struct gpu_m4_drm_sched_config scheduler;
	struct mutex *lock;
	int ret;

	if (!config || !config->dev || !config->vm || !config->params ||
	    !config->runtime || !config->runtime_owner || !config->runtime_queue)
		return ERR_PTR(-EINVAL);
	ops = config->runtime;
	if (!ops->queue_get_locked || !ops->queue_put_locked || !ops->timestamp_hold_locked ||
	    !ops->timestamp_put_locked || !ops->accept_locked || !ops->release_locked ||
	    !ops->enqueue_locked || !ops->close_locked || !ops->prepare_locked ||
	    !ops->publish_locked || !ops->retired_locked || !ops->fault_locked ||
	    !ops->timedout || !ops->reap)
		return ERR_PTR(-EOPNOTSUPP);
	if (!!ops->stage_native != !!ops->stage_put)
		return ERR_PTR(-EINVAL);
	if (candidate && (IS_ERR(candidate) || ops->stage_native || ops->stage_put ||
	    config->params->max_commands_per_submission != 1))
		return ERR_PTR(-EINVAL);
	if (!config->params->max_commands_per_submission ||
	    config->params->max_commands_per_submission > GPU_M4_DRM_JOB_MAX_COMMANDS)
		return ERR_PTR(-EINVAL);
	/* Windows need the native stage and the ordinary active-VM capture. */
	if (config->execution_window > GPU_M4_DRM_PROVIDER_WINDOW_MAX_COMMANDS ||
	    (config->execution_window && (candidate || !ops->stage_native)))
		return ERR_PTR(-EINVAL);
	/* Two credits only with bounded windows: the aggregate machinery keeps
	 * same-queue batches in order.
	 */
	if (config->credit_limit > (u32)GPU_M4_DRM_SCHED_MAX_CREDITS ||
	    (config->credit_limit > 1 && !config->execution_window))
		return ERR_PTR(-EINVAL);
	queue = kzalloc(sizeof(*queue), GFP_KERNEL);
	if (!queue)
		return ERR_PTR(-ENOMEM);
	kref_init(&queue->ref);
	queue->vm = config->vm;
	queue->provider = gpu_m4_drm_provider_vm_provider(config->vm);
	queue->runtime = *ops;
	queue->runtime_owner = config->runtime_owner;
	queue->runtime_queue = config->runtime_queue;
	queue->params = *config->params;
	queue->kernel_start = config->kernel_start;
	queue->kernel_end = config->kernel_end;
	queue->usc_base = config->usc_base;
	queue->execution_window = config->execution_window;
	queue->credit_limit = config->credit_limit;
	ret = gpu_m4_drm_provider_get(queue->provider);
	if (ret) {
		kfree(queue);
		return ERR_PTR(ret);
	}
	lock = gpu_m4_drm_provider_lock(queue->provider);
	mutex_lock(lock);
	ret = candidate ? gpu_m4_drm_provider_version_check_origin_locked(
		queue->provider, queue->vm, candidate) : 0;
	if (!ret && candidate) {
		ret = gpu_m4_drm_provider_version_get_locked(candidate);
		if (!ret)
			queue->private_candidate = candidate;
	}
	if (!ret)
		ret = gpu_m4_drm_provider_vm_get_locked(queue->vm);
	if (!ret) {
		queue->tree = !candidate && gpu_m4_drm_provider_vm_authority(queue->vm);
		queue->vm_held = true;
		ret = gpu_m4_drm_job_status(
			ops->queue_get_locked(queue->runtime_owner, queue->runtime_queue));
		queue->runtime_held = !ret;
	}
	mutex_unlock(lock);
	if (ret)
		goto fail;
	scheduler = (struct gpu_m4_drm_sched_config) {
		.dev = config->dev, .owner = queue, .ops = &gpu_m4_provider_sched_ops,
		.priority = config->priority, .client_id = config->client_id,
		.timeout = config->timeout, .credit_limit = config->credit_limit,
	};
	gpu_m4_drm_provider_queue_get(queue); /* initialized scheduler callback owner */
	queue->scheduler = gpu_m4_drm_sched_queue_create(&scheduler);
	if (IS_ERR(queue->scheduler)) {
		ret = PTR_ERR(queue->scheduler);
		queue->scheduler = NULL;
		gpu_m4_drm_provider_queue_put(queue);
		goto fail;
	}
	return queue;
fail:
	gpu_m4_drm_provider_queue_put(queue);
	return ERR_PTR(ret);
}

struct gpu_m4_drm_provider_queue *gpu_m4_drm_provider_queue_create(
	const struct gpu_m4_drm_provider_queue_config *config)
{
	return gpu_m4_provider_queue_create(config, NULL);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_queue_create);

struct gpu_m4_drm_provider_queue *gpu_m4_drm_provider_queue_create_private(
	const struct gpu_m4_drm_provider_queue_config *config,
	struct gpu_m4_drm_provider_version *candidate)
{
	if (IS_ERR_OR_NULL(candidate))
		return ERR_PTR(-EINVAL);
	return gpu_m4_provider_queue_create(config, candidate);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_queue_create_private);

static void gpu_m4_provider_close_runtime(struct gpu_m4_drm_provider_queue *queue)
{
	struct mutex *lock = gpu_m4_drm_provider_lock(queue->provider);

	mutex_lock(lock);
	if (!queue->closed) {
		queue->closed = true;
		queue->runtime.close_locked(queue->runtime_owner, queue->runtime_queue);
		/* A child parked on a private dependency is a PRIVATE lane entry the
		 * lane never revisits: wake its materializer to cancel the remainder.
		 */
		for (u32 i = 0; i < (u32)GPU_M4_DRM_SCHED_MAX_CREDITS; i++)
			if (queue->aggregate[i] && queue->aggregate[i]->window_work_ref)
				queue_work(system_unbound_wq, &queue->aggregate[i]->window_work);
		/* Likewise a never-run early first window: drop its child now. */
		if (queue->early && queue->early->window_work_ref)
			queue_work(system_unbound_wq, &queue->early->window_work);
	}
	mutex_unlock(lock);
}

void gpu_m4_drm_provider_queue_close(struct gpu_m4_drm_provider_queue *queue)
{
	/* Scheduler takes queue mutex -> provider in run; never invert that order. */
	gpu_m4_drm_sched_queue_close(queue->scheduler);
	gpu_m4_provider_close_runtime(queue);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_queue_close);

int gpu_m4_drm_provider_queue_try_close(struct gpu_m4_drm_provider_queue *queue)
{
	int ret = gpu_m4_drm_sched_queue_try_close(queue->scheduler);

	if (!ret)
		gpu_m4_provider_close_runtime(queue);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_queue_try_close);

/* REMOVE-DEBUG pacing: client time since the queue's latest success signal,
 * or a submit while its previous job is still unsignalled.
 */
static void gpu_m4_provider_pacing_submit(struct gpu_m4_drm_provider_queue *queue)
{
	u64 now = ktime_get_ns(), signal = atomic64_read(&queue->signal_ns);

	if (signal && signal > (u64)atomic64_read(&queue->submit_ns) && now >= signal)
		gpu_m4_provider_pacing_add(GPU_M4_DRM_PROVIDER_PACING_CLIENT, now - signal);
	else
		gpu_m4_provider_pacing_add(GPU_M4_DRM_PROVIDER_PACING_AHEAD, 0);
	atomic64_set(&queue->submit_ns, now);
}

struct dma_fence *gpu_m4_drm_provider_queue_submit(struct gpu_m4_drm_provider_queue *queue,
						 const struct gpu_m4_drm_submit *submit)
{
	gpu_m4_provider_pacing_submit(queue);
#ifdef CONFIG_DEBUG_FS
	gpu_m4_trace(GPU_M4_TRACE_SUBMIT, 0, (u64)(uintptr_t)queue->runtime_queue,
		current->tgid, 0, 0);
#endif
	return gpu_m4_drm_sched_submit(queue->scheduler, submit);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_queue_submit);

#ifdef CONFIG_DEBUG_FS
/* REMOVE-DEBUG timeline ring (see gpu_m4_trace()). Lossy by design: the
 * newest GPU_M4_TRACE_RECORDS records survive; a reader skips torn slots.
 */
#define GPU_M4_TRACE_RECORDS 65536U
struct gpu_m4_trace_rec {
	u64 seq, time, a, b, c, d;
	u32 kind, arg;
};
bool gpu_m4_trace_enabled;
EXPORT_SYMBOL_GPL(gpu_m4_trace_enabled);
static struct gpu_m4_trace_rec *gpu_m4_trace_ring;
static atomic64_t gpu_m4_trace_head;
static struct dentry *gpu_m4_provider_debugfs;

void gpu_m4_trace_add(u32 kind, u32 arg, u64 a, u64 b, u64 c, u64 d)
{
	struct gpu_m4_trace_rec *r;
	u64 n;

	if (!gpu_m4_trace_ring)
		return;
	n = atomic64_inc_return(&gpu_m4_trace_head);
	r = &gpu_m4_trace_ring[(n - 1) % GPU_M4_TRACE_RECORDS];
	WRITE_ONCE(r->seq, 0);
	smp_wmb();
	r->time = __arch_counter_get_cntpct();
	r->kind = kind;
	r->arg = arg;
	r->a = a;
	r->b = b;
	r->c = c;
	r->d = d;
	smp_wmb();
	WRITE_ONCE(r->seq, n);
}
EXPORT_SYMBOL_GPL(gpu_m4_trace_add);

static int gpu_m4_trace_show(struct seq_file *m, void *unused)
{
	u64 head = atomic64_read(&gpu_m4_trace_head), first;
	static const char *const name[GPU_M4_TRACE_KINDS] = {
		"submit", "run", "activate", "started", "publish", "retire", "done", "signal",
		"render", "compute", "phase", "fwring",
	};

	seq_printf(m, "# cntpct=%llu cntfrq=%u boottime_ns=%llu monotonic_ns=%llu head=%llu\n",
		__arch_counter_get_cntpct(), arch_timer_get_cntfrq(), ktime_get_boottime_ns(),
		ktime_get_ns(), head);
	first = head > GPU_M4_TRACE_RECORDS ? head - GPU_M4_TRACE_RECORDS : 0;
	for (u64 n = first + 1; gpu_m4_trace_ring && n <= head; n++) {
		const struct gpu_m4_trace_rec *r = &gpu_m4_trace_ring[(n - 1) % GPU_M4_TRACE_RECORDS];
		struct gpu_m4_trace_rec copy;

		if (READ_ONCE(r->seq) != n)
			continue;
		smp_rmb();
		copy = *r;
		smp_rmb();
		if (READ_ONCE(r->seq) != n || copy.kind >= GPU_M4_TRACE_KINDS)
			continue;
		seq_printf(m, "%llu %s %u %llx %llx %llx %llx\n", copy.time, name[copy.kind],
			copy.arg, copy.a, copy.b, copy.c, copy.d);
	}
	return 0;
}

static int gpu_m4_trace_open(struct inode *inode, struct file *file)
{
	return single_open_size(file, gpu_m4_trace_show, NULL,
		(size_t)GPU_M4_TRACE_RECORDS * 96 + PAGE_SIZE);
}

static ssize_t gpu_m4_trace_write(struct file *file, const char __user *ubuf,
	size_t count, loff_t *ppos)
{
	bool on;
	int ret = kstrtobool_from_user(ubuf, count, &on);

	if (ret)
		return ret;
	if (on) {
		WRITE_ONCE(gpu_m4_trace_enabled, false);
		synchronize_rcu();
		if (gpu_m4_trace_ring)
			memset(gpu_m4_trace_ring, 0,
			       sizeof(*gpu_m4_trace_ring) * GPU_M4_TRACE_RECORDS);
		atomic64_set(&gpu_m4_trace_head, 0);
	}
	WRITE_ONCE(gpu_m4_trace_enabled, on && gpu_m4_trace_ring);
	return count;
}

static const struct file_operations gpu_m4_trace_fops = {
	.owner = THIS_MODULE,
	.open = gpu_m4_trace_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = gpu_m4_trace_write,
};

static int gpu_m4_provider_stat_show(struct seq_file *m, void *unused)
{
	char *buffer = kzalloc(PAGE_SIZE, GFP_KERNEL);
	int (*get)(char *, const struct kernel_param *) = m->private;

	if (!buffer)
		return -ENOMEM;
	get(buffer, NULL);
	seq_puts(m, buffer);
	kfree(buffer);
	return 0;
}

static int gpu_m4_provider_stat_open(struct inode *inode, struct file *file)
{
	return single_open(file, gpu_m4_provider_stat_show, inode->i_private);
}

static const struct file_operations gpu_m4_provider_stat_fops = {
	.owner = THIS_MODULE,
	.open = gpu_m4_provider_stat_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init gpu_m4_provider_job_init(void)
{
	gpu_m4_trace_ring = vzalloc(sizeof(*gpu_m4_trace_ring) * GPU_M4_TRACE_RECORDS);
	gpu_m4_provider_debugfs = debugfs_create_dir(KBUILD_MODNAME, NULL);
	debugfs_create_file("trace", 0600, gpu_m4_provider_debugfs, NULL, &gpu_m4_trace_fops);
	debugfs_create_file("submit_phases", 0400, gpu_m4_provider_debugfs,
		gpu_m4_provider_phase_get, &gpu_m4_provider_stat_fops);
	debugfs_create_file("vm_paths", 0400, gpu_m4_provider_debugfs,
		gpu_m4_provider_paths_get, &gpu_m4_provider_stat_fops);
	return 0;
}

static void __exit gpu_m4_provider_job_exit(void)
{
	WRITE_ONCE(gpu_m4_trace_enabled, false);
	debugfs_remove_recursive(gpu_m4_provider_debugfs);
	synchronize_rcu();
	vfree(gpu_m4_trace_ring);
}
module_init(gpu_m4_provider_job_init);
module_exit(gpu_m4_provider_job_exit);
#endif

MODULE_DESCRIPTION("Private retained M4 Mesa scheduler/runtime job ownership");
MODULE_LICENSE("GPL");
