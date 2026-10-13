/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_DRM_PROVIDER_JOB_H
#define APPLE_GPU_M4_DRM_PROVIDER_JOB_H

#include "gpu-m4-drm-provider-vm.h"
#include "gpu-m4-drm-render-job.h"
#include "gpu-m4-drm-scheduler.h"

struct gpu_m4_drm_provider_queue;
struct gpu_m4_drm_provider_job;
/* A bounded child window: at most 32 commands and at most 16 of each class
 * (render, compute), the working publisher's packet bounds (one host UMA
 * slot bank of 16 per class).
 */
#define GPU_M4_DRM_PROVIDER_WINDOW_MAX_COMMANDS 32U
#define GPU_M4_DRM_PROVIDER_WINDOW_MAX_PER_CLASS 16U

/* Borrowed immutable acceptance view, valid only during stage_native(). No
 * job pointer/handle lookup escapes; copied commands are never redecoded
 * against staged private entries or a subsequently active user VM version.
 */
struct gpu_m4_drm_provider_stage_view {
	struct gpu_m4_drm_provider_vm *vm;
	struct gpu_m4_drm_provider_version *source;
	const struct gpu_m4_drm_vm_owner *snapshot; /* Already timestamp-joined. */
	const struct gpu_m4_drm_mixed_batch *batch;
	const void *command_bytes;
	u32 command_size;
	void *runtime_queue;
	/* Internal CPU materialization after the original submit's inputs. No
	 * frontend mutex is held; factory calls must use their locking entrance.
	 * Barrier indices remain the original submit's ordinals, not this slice.
	 */
	u32 command_first, command_total;
	bool deferred_materialization;
};

/* Initial cache/reservation-only purpose. The submitted u64 is separate from
 * the embedded descriptor's packed completion counter. No client mapping,
 * private VA claim, context-0 mirror, raw pin/PA or caller policy is supplied.
 */
enum gpu_m4_drm_provider_stage_extra_purpose {
	GPU_M4_DRM_PROVIDER_STAGE_SUBMITTED_COUNTER = 1,
	/* Native NC firmware-only scene/sequence/notifier/PB metadata. The runtime
	 * proves its canonical upper lease; this core captures the complete owned
	 * page span, never a dummy client UAT mapping.
	 */
	GPU_M4_DRM_PROVIDER_STAGE_FIRMWARE_METADATA = 2,
};
#define GPU_M4_DRM_PROVIDER_STAGE_MAX_EXTRAS (GPU_M4_DRM_JOB_MAX_COMMANDS * 16U)
struct gpu_m4_drm_provider_stage_extra {
	struct gpu_m4_drm_provider_object *object;
	u64 offset, size; /* Counter RW8 or page-aligned complete metadata span. */
	u32 access; /* Counter READ|WRITE; metadata READ or READ|WRITE. */
	enum gpu_m4_drm_provider_stage_extra_purpose purpose;
};

/* owner strongly owns every native object, private claim and runtime lease.
 * Entries are immutable until stage_put; combined count+extra_count is bounded
 * by MAX_BINDS and the runtime's charged finite per-job metadata/native budget.
 * Extras additionally obey MAX_EXTRAS. Each counter has separate native NC
 * backing, a retained complete page, fresh/displaced allocator state and an
 * actual ordinary FW-only upper lease owned/proved by the runtime. An owner or
 * upper reference alone never establishes cache/reservation coverage.
 * count=0/native=NULL retains the captured version/roots/generation; extras
 * still join its new unpublished job snapshot before cache/resv construction.
 * With both counts zero, owner may hold detached runtime leases or be NULL.
 * The adapter, not the hook, constructs any candidate through the typed builder.
 * Core-owned extra object refs outlive all cache/snapshot pin borrowers; the
 * runtime separately retains firmware-reachable allocator baselines on success
 * or uncertainty. No returned count controls disposal of the opaque owner.
 */
struct gpu_m4_drm_provider_stage_result {
	void *owner;
	const struct gpu_m4_drm_provider_private_gem *native;
	u32 count;
	const struct gpu_m4_drm_provider_stage_extra *extra;
	u32 extra_count;
};

/* This is a PRIVATE adapter for the retained READY owner, not a registration
 * gate. It advertises neither render support nor WB coherency. Every hardware
 * runtime hook is mandatory: a decoder, consumer movement or a dummy fence cannot
 * substitute for a qualified publisher and full retirement predicate.
 *
 * *_locked hooks hold the ONE provider mutex. They never acquire a GEM resv,
 * wait for a dependency, signal a fence, or release a final native pin. The
 * root runtime retains actual queue/work/upper alias/event/notifier/UMA owners;
 * their final backing release is deferred to reap(), outside all locks.
 */
struct gpu_m4_drm_provider_job_runtime {
	int (*queue_get_locked)(void *owner, void *queue);
	void (*queue_put_locked)(void *owner, void *queue);
	/* Optional PAIRED hooks. Run synchronously outside provider/GEM-resv locks
	 * after timestamp capture, before cache/resv creation. Native allocation,
	 * pinning and initialization of NEW exclusively owned backing may run here;
	 * never overwrite/clean earlier dependency-owned WB storage. Brief provider
	 * locking for already pinned claims is allowed. No roots/upper leaves/rings,
	 * dependency waits, fence signals, recursive queue submit/close or asynchronous
	 * borrowed view retention. The scheduler's queue mutex remains held, as does
	 * the frontend mutex on actual public submit. That path must use the native
	 * factory's explicitly frontend-locked entrance, never its locking wrapper;
	 * off-frontend private callers retain their ordinary factory entrance.
	 * Every non-NULL returned owner transfers even on error/malformed output and
	 * is disposed by stage_put outside provider/resv locks. Positive status fails.
	 * Acceptance rollback may still hold frontend/queue locks during stage_put;
	 * it must not recursively acquire those locks either.
	 * Successful owners survive cache/accept/resv rollback and are retained with
	 * ambiguous published jobs; release_locked only borrows this stage owner.
	 */
	int (*stage_native)(void *owner, void *queue,
			    const struct gpu_m4_drm_provider_stage_view *view,
			    struct gpu_m4_drm_provider_stage_result *result);
	void (*stage_put)(void *owner, void *stage_owner);
	/* Use the staged exact native object; retain the borrowed object-bind
	 * backend and a real, non-overlapping dedicated FW timestamp alias. No
	 * current handle lookup or fabricated client-VA substitution is allowed.
	 * Success returns one owned alias; failure returns no alias or writes.
	 */
	int (*timestamp_hold_locked)(void *owner,
				     struct gpu_m4_drm_provider_version *version,
				     struct gpu_m4_drm_provider_object *object,
				     const struct gpu_m4_drm_timestamp *borrowed,
				     void **alias, u64 *firmware_va);
	void (*timestamp_put_locked)(void *owner, void *alias);
	/* Allocate/retain complete PRIVATE work and queue leases. Called during
	 * synchronous acceptance, before reservations are locked. Validate the
	 * decoded command types against the actual qualified backend: reject an
	 * unsupported render/compute encoder instead of pretending it ran.
	 * No table/root/leaf/ring publication. Failure returns no runtime job.
	 * The job pointer is borrowed here; never retain it in asynchronous work
	 * or take a job ref until enqueue transfers its initialized owned ref.
	 */
	int (*accept_locked)(void *owner, void *queue,
			     struct gpu_m4_drm_provider_job *job, void **runtime_job);
	void (*release_locked)(void *owner, void *runtime_job);
	/* Infallibly consume ONE job ref after ALL explicit/implicit deps. Append
	 * to the provider's GLOBAL ready FIFO and schedule independent work. Do
	 * not start here, while the scheduler's queue mutex remains held. Per-
	 * queue scheduling credit is not the global lane. An occupied lane must
	 * queue the ready job, not return -EBUSY. Never impose acceptance-order
	 * dependencies: an earlier job may wait for a future point supplied by a
	 * later ready job on another queue.
	 */
	void (*enqueue_locked)(void *owner, struct gpu_m4_drm_provider_job *job);
	/* Cancel only unpublished pending jobs via job_cancel_locked(). Arrange
	 * signal()/put() from independent work AFTER unlocking, without awaiting
	 * future inputs. Running/ambiguous jobs retain their runtime-owned refs.
	 */
	void (*close_locked)(void *owner, void *queue);
	/* Private work preparation AFTER deps/cache maintenance. No HW-visible
	 * write. The runtime has already acquired the exclusive global lane.
	 */
	int (*prepare_locked)(void *owner, struct gpu_m4_drm_provider_job *job);
	/* Job/cache are protected BEFORE entry. Bind the exact captured version
	 * (owned roots/handoff/readback/completed TLBI), then version_begin_work
	 * BEFORE threshold/ring publication and kick. Zero means an actual work
	 * lease exists; ANY error is potentially published and retained. No
	 * rollback/reset/release follows from that error.
	 */
	int (*publish_locked)(void *owner, struct gpu_m4_drm_provider_job *job);
	/* 1 ONLY for the FULL engine-dependent retirement predicate: owned
	 * selected stamps, queue/pipe prefixes, health/binding and final UMA.
	 * Compute requires nonzero internal start/end with end > start. Render
	 * requires its paired TA/FS proof, NOT every optional empty-stage timing
	 * cell. All requested command stages/copies/dependencies must be done.
	 * 0 is pending; negative is a retained terminal fault. This must have a
	 * real qualified implementation, not a constant-success stub.
	 */
	int (*retired_locked)(void *owner, struct gpu_m4_drm_provider_job *job);
	/* Latch the whole runtime failed, stop new work, retain every potentially
	 * reachable queue/alias/work owner and capture bounded evidence. VM and
	 * provider error are already latched; no error-fence signal occurs here.
	 */
	void (*fault_locked)(void *owner, struct gpu_m4_drm_provider_job *job, int error);
	/* Actual scheduler timeout disposal/recovery, outside provider locks.
	 * Follow sched_ops.timedout: ENODEV alone does not dispose the guilty job.
	 */
	enum drm_gpu_sched_stat (*timedout)(void *owner,
					  struct gpu_m4_drm_provider_job *job,
					  struct drm_sched_job *sched_job);
	/* Sole native/provider collection entrance, outside provider/resv locks.
	 * Acceptance/queue rollback can still hold frontend or scheduler queue
	 * serialization: defer collection to independent work in that case, retaining
	 * the actual runtime/provider owner until the collector runs. Collect queued
	 * provider garbage as well as runtime leases; core never bypasses this hook.
	 */
	void (*reap)(void *owner);
};

struct gpu_m4_drm_provider_queue_config {
	struct device *dev;
	struct gpu_m4_drm_provider_vm *vm;
	const struct drm_asahi_params_global *params;
	u64 kernel_start, kernel_end, usc_base, client_id;
	u32 priority;
	long timeout;
	/* Ordinary-only bounded native execution. Zero preserves the existing
	 * eager qualification path; 1..8 materializes one real child at a time.
	 */
	u32 execution_window;
	/* Scheduler credits of this queue: 0/1 one public batch run at a time,
	 * 2 (two packets in flight) lets the next batch run, and materialize
	 * its first child, while the previous one still executes.
	 */
	u32 credit_limit;
	void *runtime_owner, *runtime_queue;
	const struct gpu_m4_drm_provider_job_runtime *runtime;
};

/* One caller ref. The scheduler and each retained/quarantined job hold their
 * own queue ref, so frontend close cannot free an uncertain runtime queue.
 * Create initializes all owner/ref state BEFORE creating the scheduler.
 */
struct gpu_m4_drm_provider_queue *gpu_m4_drm_provider_queue_create(
	const struct gpu_m4_drm_provider_queue_config *config);
/* Sealed diagnostic entrance, NEVER a userspace flag/mutable queue switch.
 * Retain this exact registered candidate BEFORE creating scheduler callbacks;
 * every accepting job obtains its own ref and fresh cache/resv snapshot clone.
 * Translation checks use only that owned version's typed UAT pages. The normal
 * constructor retains active-VM/user-ledger semantics with no private fallback.
 * This profile requires max_commands_per_submission == 1, rejects render and
 * disables stage_native/stage_put. Runtime accept still must prove the concrete
 * qualified compute encoder/owner: this entrance grants no hardware authority.
 */
struct gpu_m4_drm_provider_queue *gpu_m4_drm_provider_queue_create_private(
	const struct gpu_m4_drm_provider_queue_config *config,
	struct gpu_m4_drm_provider_version *candidate);
void gpu_m4_drm_provider_queue_get(struct gpu_m4_drm_provider_queue *queue);
/* Final queue puts take the provider lock and reap backing: no resv/provider
 * lock may be held. Close is mandatory before the caller's final queue put.
 */
void gpu_m4_drm_provider_queue_put(struct gpu_m4_drm_provider_queue *queue);
void gpu_m4_drm_provider_queue_close(struct gpu_m4_drm_provider_queue *queue);
int gpu_m4_drm_provider_queue_try_close(struct gpu_m4_drm_provider_queue *queue);
struct dma_fence *gpu_m4_drm_provider_queue_submit(
	struct gpu_m4_drm_provider_queue *queue, const struct gpu_m4_drm_submit *submit);

/* Immutable accepted bytes and exact captured owners, borrowed through the
 * caller's job ref. No asynchronous frontend/file/handle callback survives.
 */
struct gpu_m4_drm_provider_job_view {
	struct gpu_m4_drm_provider_version *version;
	struct gpu_m4_drm_vm_owner *snapshot;
	const struct gpu_m4_drm_mixed_batch *batch;
	const void *command_bytes;
	u32 command_size;
	void *runtime_queue, *runtime_job, *stage_owner;
	struct gpu_m4_drm_provider_job *window_parent;
	u32 command_first, command_total;
	bool aggregate;
};

void gpu_m4_drm_provider_job_view(struct gpu_m4_drm_provider_job *job,
				struct gpu_m4_drm_provider_job_view *view);
/* Borrowed through the exact held aggregate parent. NULL means healthy CPU
 * materialization/dependency wait/completion, or a local (non-GPU) aggregate
 * failure still being drained by the outside worker: never a PUBLISHED or
 * retirement authority. ERR_PTR reports a nonparent, provider failure or a
 * faulted aggregate. The returned enqueued child retains its original parent.
 */
struct gpu_m4_drm_provider_job *gpu_m4_drm_provider_job_active_window_locked(
	struct gpu_m4_drm_provider_job *parent);
/* Root calls AFTER its real post-CORE native receipt/last-owner bookkeeping.
 * No-op for ordinary nonwindow jobs. Advances only the exact successfully
 * retired child and schedules allocation/signaling outside every GPU/resv lock.
 */
int gpu_m4_drm_provider_job_window_complete_locked(struct gpu_m4_drm_provider_job *child);
/* REMOVE-DEBUG: prepare count/ns/max, window-build count/ns/max. */
void gpu_m4_drm_provider_job_stats(u64 out[6]);
/* REMOVE-DEBUG: proof reuse, submit-path mutex waits and job-local capacity
 * outcomes, never authority.
 */
enum gpu_m4_drm_provider_proof_stat {
	GPU_M4_DRM_PROVIDER_PROOF_COVERED_PROOFS,	/* complete coverage proofs */
	GPU_M4_DRM_PROVIDER_PROOF_COVERED_HITS,		/* held-pair memo reuse */
	GPU_M4_DRM_PROVIDER_PROOF_CAPTURE_LOCKLESS,	/* snapshot without the mutex */
	GPU_M4_DRM_PROVIDER_PROOF_CAPTURE_LOCKED,	/* snapshot under the mutex */
	GPU_M4_DRM_PROVIDER_PROOF_CAPTURE_WAIT_NS,	/* its mutex wait, total/max */
	GPU_M4_DRM_PROVIDER_PROOF_CAPTURE_WAIT_MAX,
	GPU_M4_DRM_PROVIDER_PROOF_CAPACITY_REFUSALS,	/* public jobs failed locally */
	GPU_M4_DRM_PROVIDER_PROOF_CAPACITY_RETRIES,	/* reclaim-then-rebuild passes */
	GPU_M4_DRM_PROVIDER_PROOF_ACCESS_INDEXED,	/* access rows found by identity */
	GPU_M4_DRM_PROVIDER_PROOF_ACCESS_SCANNED,	/* ... or by the complete scans */
	GPU_M4_DRM_PROVIDER_PROOF_CAPACITY_SPLITS,	/* refused windows rebuilt smaller */
	GPU_M4_DRM_PROVIDER_PROOF_STATS,
};
void gpu_m4_drm_provider_job_proof_stats(u64 out[GPU_M4_DRM_PROVIDER_PROOF_STATS]);
/* REMOVE-DEBUG: first execution windows materialized before their public
 * batch's dependencies, never authority.
 */
enum gpu_m4_drm_provider_early_stat {
	GPU_M4_DRM_PROVIDER_EARLY_BUILT,	/* first windows materialized before run */
	GPU_M4_DRM_PROVIDER_EARLY_USED,		/* ...already held at dependency-ready */
	GPU_M4_DRM_PROVIDER_EARLY_LATE,		/* ...still building at dependency-ready */
	GPU_M4_DRM_PROVIDER_EARLY_LATE_NS,	/* that remaining build time, total */
	GPU_M4_DRM_PROVIDER_EARLY_HIDDEN_NS,	/* build complete -> dependency-ready */
	GPU_M4_DRM_PROVIDER_EARLY_HIDDEN_MAX,
	GPU_M4_DRM_PROVIDER_EARLY_SKIPPED,	/* not attempted: bound or quiet period */
	GPU_M4_DRM_PROVIDER_EARLY_FALLBACKS,	/* refused or unsealed: built lazily */
	GPU_M4_DRM_PROVIDER_EARLY_EVICTED,	/* dropped for another window's capacity */
	GPU_M4_DRM_PROVIDER_EARLY_CANCELED,	/* close/failure/release while pre-built */
	GPU_M4_DRM_PROVIDER_EARLY_STATS,
};
/* Dependency-ready (run) to the first window's lane enqueue, its lane
 * activation (start entry) and its publication (successful start), for
 * [lazy, early] first windows: count, total ns, max ns.
 */
enum gpu_m4_drm_provider_ready_phase {
	GPU_M4_DRM_PROVIDER_READY_ENQUEUE,
	GPU_M4_DRM_PROVIDER_READY_START,
	GPU_M4_DRM_PROVIDER_READY_PUBLISH,
	GPU_M4_DRM_PROVIDER_READY_PHASES,
};
/* Provider error_wake_locked hook: the provider mutex is held and a provider
 * fault (vm NULL) or vm's VM-local error is latched. Queue the materializer of
 * every registered early first window in that scope; it drops its unsealed
 * child unpublished outside every lock. Queues work only.
 */
void gpu_m4_drm_provider_job_error_wake_locked(struct gpu_m4_drm_provider *provider,
	struct gpu_m4_drm_provider_vm *vm);
void gpu_m4_drm_provider_job_early_stats(u64 early[GPU_M4_DRM_PROVIDER_EARLY_STATS],
	u64 ready[2][GPU_M4_DRM_PROVIDER_READY_PHASES][3]);
/* REMOVE-DEBUG (two packets in flight): what the queue of a retiring child
 * has behind it, read under the provider mutex. Never an authority.
 */
enum gpu_m4_drm_queue_next {
	GPU_M4_DRM_QUEUE_NEXT_NONE,	/* nothing accepted behind it */
	GPU_M4_DRM_QUEUE_NEXT_WAITING,	/* accepted, not run: inputs or credit */
	GPU_M4_DRM_QUEUE_NEXT_OWN,	/* its own batch has another window */
	GPU_M4_DRM_QUEUE_NEXT_GATED,	/* a later running batch awaits its turn */
	GPU_M4_DRM_QUEUE_NEXT_BUILDING,	/* a later running batch materializes */
	GPU_M4_DRM_QUEUE_NEXT_ENQUEUED,	/* a later batch's child is in the lane */
	GPU_M4_DRM_QUEUE_NEXTS,
};
int gpu_m4_drm_provider_job_queue_next_locked(struct gpu_m4_drm_provider_job *child);
/* REMOVE-DEBUG client pacing, per public job of a queue: [count, ns, max].
 * RETIRE_SIGNAL: final child's CORE retirement -> public fence signal.
 * CLIENT: public fence signal -> the queue's next submit entry (client time).
 * SUBMIT_RUN: submit entry -> scheduler run (inputs and credit).
 * AHEAD counts submits entering while the queue's previous job was unsignalled
 * (in its ns slot: none).
 */
enum gpu_m4_drm_provider_pacing {
	GPU_M4_DRM_PROVIDER_PACING_RETIRE_SIGNAL,
	GPU_M4_DRM_PROVIDER_PACING_CLIENT,
	GPU_M4_DRM_PROVIDER_PACING_SUBMIT_RUN,
	GPU_M4_DRM_PROVIDER_PACING_AHEAD,
	GPU_M4_DRM_PROVIDER_PACINGS,
};
void gpu_m4_drm_provider_job_pacing_stats(u64 out[GPU_M4_DRM_PROVIDER_PACINGS][3]);

/* REMOVE-DEBUG host/GPU timeline (debugfs gpu_m4_drm_provider_job/trace,
 * write 1 to clear and enable, 0 to stop). Times are raw CNTPCT; GPU stage
 * stamps are 24 MHz ticks of the same counter. Observational only.
 */
enum gpu_m4_trace_kind {
	GPU_M4_TRACE_SUBMIT,		/* a=queue b=tgid */
	GPU_M4_TRACE_RUN,		/* a=queue b=job c=commands */
	GPU_M4_TRACE_ACTIVATE,		/* a=queue b=item c=commands d=inflight slot */
	GPU_M4_TRACE_STARTED,		/* a=queue b=item c=ret */
	GPU_M4_TRACE_PUBLISH,		/* a=queue b=item c=index d=type */
	GPU_M4_TRACE_RETIRE,		/* a=queue b=item c=index */
	GPU_M4_TRACE_DONE,		/* a=queue b=item c=ret */
	GPU_M4_TRACE_SIGNAL,		/* a=queue b=job */
	GPU_M4_TRACE_RENDER,		/* a=ta0 b=ta1 c=fs0 d=fs1, arg=w|h<<16 */
	GPU_M4_TRACE_COMPUTE,		/* a=start b=end */
	GPU_M4_TRACE_PHASE,		/* a=queue b=item c=phase d=ns */
	GPU_M4_TRACE_FWRING,		/* arg=channel<<28|tag a=+4 b=+0xc c=+0x14 d=+0x30|+0x1c<<32 */
	GPU_M4_TRACE_KINDS,
};
/* Call sites stay inside #ifdef CONFIG_DEBUG_FS (host selftests build none). */
#ifdef CONFIG_DEBUG_FS
extern bool gpu_m4_trace_enabled;
void gpu_m4_trace_add(u32 kind, u32 arg, u64 a, u64 b, u64 c, u64 d);
static inline void gpu_m4_trace(u32 kind, u32 arg, u64 a, u64 b, u64 c, u64 d)
{
	if (READ_ONCE(gpu_m4_trace_enabled))
		gpu_m4_trace_add(kind, arg, a, b, c, d);
}
#endif
/* Borrowed exact held timestamp identities, not a current handle lookup. The
 * public runtime cannot cast the private decoder lease. Enumeration preserves
 * mixed command order and skips absent timestamp slots. object_offset is the
 * ABSOLUTE eight-byte GEM slice (bind offset already included).
 */
struct gpu_m4_drm_provider_job_timestamp_view {
	void *runtime_alias;
	struct gpu_m4_drm_provider_object *object;
	struct drm_gem_object *gem;
	u64 object_offset, firmware_va;
	u32 command_index, timestamp_index;
	u32 original_command_index;
};
/* Caller retains a live job reference. All three helpers hold the ONE provider
 * mutex and perform no ref mutation,
 * payload/cache operation, hardware publication or fence signaling. Outputs
 * remain unchanged on error. Count/at are also available before publication for
 * detached token staging. Retained accepted owners remain valid after queue/VM
 * close; latched provider failure is never converted into a successful proof.
 */
int gpu_m4_drm_provider_job_timestamp_count_locked(struct gpu_m4_drm_provider_job *job,
	u32 *count);
int gpu_m4_drm_provider_job_timestamp_at_locked(struct gpu_m4_drm_provider_job *job,
	u32 index, struct gpu_m4_drm_provider_job_timestamp_view *view);
/* Exact runtime_alias membership plus final immutable WRITE cache/resv coverage
 * of eight bytes. false requires accepted+protected PUBLISHED/cache-published;
 * true requires RETIRED/cache-retired and committed error-free completion.
 * This does NOT validate the runtime's exclusive lane, translation/alias graph
 * or full hardware retirement predicate: its token owner must do that too.
 */
int gpu_m4_drm_provider_job_timestamp_check_locked(struct gpu_m4_drm_provider_job *job,
	void *runtime_alias, bool retired);
/* Caller retains this actual job and holds expected_provider's mutex. Check
 * immutable actual queue/provider identity and live job/queue refs without
 * looking up active VM/queue state or mutating refs. Closed accepted work keeps
 * this identity; a latched provider error is returned, never hidden by closure.
 */
int gpu_m4_drm_provider_job_check_owner_locked(struct gpu_m4_drm_provider_job *job,
	struct gpu_m4_drm_provider *expected_provider);
/* Read-only exact accepted READY (!published) or protected PUBLISHED phase.
 * Used with the lane's independent registered-item/deadline proof to identify
 * a false whole-submit scheduler timeout. Does not grant publication, cache
 * retirement or completion, and does not require an accepted VM to stay open.
 */
int gpu_m4_drm_provider_job_check_execution_locked(struct gpu_m4_drm_provider_job *job,
	struct gpu_m4_drm_provider *expected_provider, bool published);
/* Exact synchronous runtime.accept entrance, NOT !job_uncertain(): captured
 * source/cache exist while state is PREPARED, before acceptance, protection,
 * runtime-job transfer and finished/resv publication. Only read-only checks.
 * No current-active VM capture or new timestamp/native pin is performed.
 */
int gpu_m4_drm_provider_job_check_accepting_locked(struct gpu_m4_drm_provider_job *job,
	struct gpu_m4_drm_provider *expected_provider);
/* Caller retains the actual job and object and holds expected_provider's one
 * mutex. Prove the object's healthy native origin in this captured version,
 * exact byte-range/access coverage in the final immutable snapshot/cache and
 * the job's finished-reservation contract. No lower-ledger membership, current
 * VM openness, timestamp request or CPU policy substitution is required.
 * false is the accepted+protected PUBLISHED/cache-published runtime publication
 * entrance; true requires actual RETIRED/cache-retired, error-free committed
 * completion. This does NOT prove exclusive lane ownership, completed binding,
 * translation aliases or hardware retirement: the runtime must prove those.
 * Optional cache_out receives this job's exact opaque cache only on success.
 * It is borrowed through the held job reference and checked phase, never a
 * separately owned cache or authority to perform cache operations. Apart from
 * that successful output assignment, no refs, cache/reservations, payload or
 * fence state changes occur; output remains unchanged on every failure.
 */
int gpu_m4_drm_provider_job_check_access_locked(struct gpu_m4_drm_provider_job *job,
	struct gpu_m4_drm_provider *expected_provider,
	struct gpu_m4_drm_provider_object *object, u64 offset, u64 size, u32 access,
	bool retired, struct gpu_m4_drm_cache_job **cache_out);
/* Complete native coverage (gpu_m4_drm_provider_version_native_covered_locked)
 * of this job's OWN held version/snapshot pair, through the job's exact pair
 * coverage memo: proved once over the live immutable arrays, then reused for
 * exactly that pair (the same memo as access checks). Any other pair runs
 * the complete comparison and is never remembered. Caller holds the lock.
 */
bool gpu_m4_drm_provider_job_native_covered_locked(struct gpu_m4_drm_provider_job *job,
	struct gpu_m4_drm_provider_version *version, const struct gpu_m4_drm_vm_owner *snapshot);
int gpu_m4_drm_provider_job_get_locked(struct gpu_m4_drm_provider_job *job);
/* Exact final-put retention gate, not authority to clear or reclaim backing. */
bool gpu_m4_drm_provider_job_uncertain_locked(struct gpu_m4_drm_provider_job *job);
/* Adopt an owned private-extended version only before publication. Require
 * exact retained VM/user-generation lineage, unchanged captured user leaves,
 * and a complete owned UAT graph. Every candidate native BO/range/permission
 * must be covered by the unchanged snapshot used to create the job's cache
 * and reservation sets. A previously uncaptured private WB BO is -EACCES;
 * capture it BEFORE job preparation, never join it in runtime acceptance.
 * NEVER reread current VM_BIND state. Failure
 * leaves both job and caller's candidate ownership unchanged; success transfers
 * that one candidate ref into the job before dropping the old version ref.
 * The original immutable batch/cache/resv snapshot is not replaced.
 */
int gpu_m4_drm_provider_job_adopt_version_locked(struct gpu_m4_drm_provider_job *job,
	struct gpu_m4_drm_provider_version *candidate);
/* Never under provider/resv locks. Final put refuses published uncertainty.
 * Runtime enqueue consumes one ref; keep it while pending/running/quarantined.
 */
int gpu_m4_drm_provider_job_put(struct gpu_m4_drm_provider_job *job);

/* Root-owned independent lane/drain worker calls these under provider lock.
 * start requires the exclusive GLOBAL lane; callback publishes actual roots
 * and work. A failed unpublished start has a ready error fence, not success.
 * retire returns -EAGAIN while the mandatory full predicate is incomplete.
 * No transition signals under the provider mutex.
 */
int gpu_m4_drm_provider_job_start_locked(struct gpu_m4_drm_provider_job *job);
int gpu_m4_drm_provider_job_retire_locked(struct gpu_m4_drm_provider_job *job);
int gpu_m4_drm_provider_job_cancel_locked(struct gpu_m4_drm_provider_job *job);
void gpu_m4_drm_provider_job_fault_locked(struct gpu_m4_drm_provider_job *job, int error);
/* Hold a job ref and drop provider/resv locks first. Success/error signals only
 * a completion already committed above; there is no public "signal success"
 * operation. WB invalidation and VM/version retirement precede success.
 */
int gpu_m4_drm_provider_job_signal(struct gpu_m4_drm_provider_job *job);

#endif
