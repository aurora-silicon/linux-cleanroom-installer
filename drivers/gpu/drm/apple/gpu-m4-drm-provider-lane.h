/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_DRM_PROVIDER_LANE_H
#define APPLE_GPU_M4_DRM_PROVIDER_LANE_H

#include <linux/bits.h>
#include <linux/list.h>
#include "gpu-m4-drm-provider-job.h"

struct gpu_m4_drm_provider_lane;
struct gpu_m4_drm_provider_lane_item;

/* Observational host times, never a retirement/cache/fence authority.
 * Command mode keeps a contiguous published horizon: [0, published_count)
 * are reachable and [0, retired_count) carry full hardware receipts. The head
 * is retired_count; its budget starts at head_ns = max(its own publication,
 * its predecessor's actual receipt). command_ns keeps each actual publication
 * time separately; publishing ahead never extends the head's deadline.
 */
struct gpu_m4_drm_provider_lane_timing {
	u64 ready_ns, activated_ns, published_ns, deadline_ns, retired_ns;
	u32 published_index, retired_index;
	bool published, retired;
	u32 published_count, retired_count;
	u64 head_ns;
	u64 command_ns[GPU_M4_DRM_JOB_MAX_COMMANDS];
};

/* Embedded in each retained runtime job, allocated during fallible acceptance.
 * The scheduler's run callback transfers a job reference without allocating.
 * Only the lane worker may consume that reference. A quarantined entry remains
 * reachable until an independently qualified shutdown, not an error fence.
 */
struct gpu_m4_drm_provider_lane_item {
	struct list_head owner_link, ready_link, completion_link;
	struct gpu_m4_drm_provider_lane *lane;
	struct gpu_m4_drm_provider_job *job;
	void *queue;
	/* Optional PRIVATE-configured bookkeeping. Caller retains the strongly
	 * owned runtime payload with this item, including quarantine. No ref is
	 * acquired or disposed by this generic lane callback configuration.
	 */
	void *retired_owner;
	void (*retired_locked)(void *owner, struct gpu_m4_drm_provider_lane_item *item,
			       struct gpu_m4_drm_provider_job *job);
	u32 state;
	/* Host scheduling level, 0 (lowest) .. GPU_M4_DRM_LANE_PRIORITIES - 1;
	 * zero unless set while PRIVATE. Orders only the READY FIFO.
	 */
	u32 priority;
	u32 command_count; /* zero preserves the original single-job lane mode */
	u32 command_depth; /* bounded published-unreceipted commands, >= 1 */
	struct gpu_m4_drm_provider_lane_timing timing;
	/* REMOVE-DEBUG: GPU_M4_DRM_LANE_OVERLAP_* observed while READY behind a
	 * retiring head; metadata only, never an admission input.
	 */
	u32 overlap;
	bool closed;
};

/* REMOVE-DEBUG overlap classes of a READY candidate behind the ACTIVE head.
 * SEEN marks the observation; QUEUE implies VM. Neither implies admission.
 */
#define GPU_M4_DRM_LANE_OVERLAP_SEEN BIT(0)
#define GPU_M4_DRM_LANE_OVERLAP_QUEUE BIT(1)
#define GPU_M4_DRM_LANE_OVERLAP_VM BIT(2)

/* Lane levels: DRM_ASAHI_PRIORITY_LOW .. REALTIME map to 0..3 unchanged. */
#define GPU_M4_DRM_LANE_PRIORITIES 4U

/* Hard representation bound of the in-flight set: [0] is the head, the only
 * item serviced, timed and retired; [1..] are admitted successors, in
 * admission (= publication) order.
 */
#define GPU_M4_DRM_LANE_MAX_INFLIGHT 4U

/* Optional runtime admission policy, set once before any item exists. Without
 * one a lane never admits beyond its head. admit_locked runs only while
 * another item is in flight, before the candidate leaves the READY FIFO; any
 * nonzero result keeps it (and everything behind it) queued for a later pass.
 * classify_locked (REMOVE-DEBUG) returns GPU_M4_DRM_LANE_OVERLAP_VM bits for
 * a READY candidate behind a successfully retiring head. Both are read-only
 * under the provider mutex: no hardware, ref, wait or signal.
 */
struct gpu_m4_drm_provider_lane_policy {
	int (*admit_locked)(void *owner, struct gpu_m4_drm_provider_lane_item *head,
			    struct gpu_m4_drm_provider_lane_item *candidate);
	u32 (*classify_locked)(void *owner, struct gpu_m4_drm_provider_lane_item *head,
			       struct gpu_m4_drm_provider_lane_item *candidate);
	/* Lane cap chosen by the runtime (bounded by
	 * GPU_M4_DRM_LANE_MAX_INFLIGHT); 0 keeps the max_inflight parameter.
	 */
	u32 max_inflight;
	/* A promoted successor keeps the budget of its head command's actual
	 * publication (2 s from publication, never renewed when it becomes the
	 * head). false keeps max(publication, promotion).
	 */
	bool publication_deadline;
};

struct gpu_m4_drm_provider_lane *gpu_m4_drm_provider_lane_create(
	struct gpu_m4_drm_provider *provider, u32 max_items, u32 timeout_ms,
	u32 poll_ms);
/* No provider lock or reservation locks: cancels independent work outside
 * them. Refuses every prepared, queued, running or quarantined runtime owner.
 */
int gpu_m4_drm_provider_lane_destroy(struct gpu_m4_drm_provider_lane *lane);
/* Outside the provider mutex, before any item is registered. The lane cap
 * becomes policy->max_inflight, or without one the max_inflight module
 * parameter (default 1), bounded by GPU_M4_DRM_LANE_MAX_INFLIGHT; owner is
 * retained by the caller for the lane's lifetime.
 */
int gpu_m4_drm_provider_lane_set_policy(struct gpu_m4_drm_provider_lane *lane,
	void *owner, const struct gpu_m4_drm_provider_lane_policy *policy);
u32 gpu_m4_drm_provider_lane_max_inflight_locked(struct gpu_m4_drm_provider_lane *lane);
/* REMOVE-DEBUG: the READY FIFO's first item (the next admission candidate),
 * or NULL. Read-only identity for diagnostics, never an admission input.
 */
struct gpu_m4_drm_provider_lane_item *gpu_m4_drm_provider_lane_ready_first_locked(
	struct gpu_m4_drm_provider_lane *lane);

/* All *_locked interfaces use the provider's one retained owner mutex. */
int gpu_m4_drm_provider_lane_item_init_locked(
	struct gpu_m4_drm_provider_lane *lane,
	struct gpu_m4_drm_provider_lane_item *item,
	struct gpu_m4_drm_provider_job *job, void *queue);
int gpu_m4_drm_provider_lane_item_fini_locked(
	struct gpu_m4_drm_provider_lane_item *item);
/* Read-only exact registered ACTIVE (in-flight: head or admitted successor)
 * item/job/provider proof under the expected provider's mutex. No opaque lane
 * cast, mutable frontend lookup, ref/cache change or hardware operation.
 * Accepted closed queues may retire normally; provider/lane failures do not
 * grant current-job readiness.
 */
int gpu_m4_drm_provider_lane_check_active_locked(
	struct gpu_m4_drm_provider_lane *lane, struct gpu_m4_drm_provider *expected_provider,
	struct gpu_m4_drm_provider_lane_item *item, struct gpu_m4_drm_provider_job *job);
/* Service a whole-submit scheduler timeout through the SAME active retirement
 * path as the worker. Zero means real CORE retirement just completed, the
 * serial command is still within its own actual publication deadline, or the
 * exact READY item is waiting in the owned software FIFO. -ETIMEDOUT means a
 * fresh complete retirement predicate is still pending at/after its deadline.
 * The original non-serial lane mode has no false-timeout exemption. An
 * admitted successor is NO_HANG while the head is in flight: only the head's
 * deadline counts, and the successor's own service retires or faults that
 * head on its behalf. Other owner/state/health errors propagate. No pending
 * poll renews either clock.
 * Actual successful CORE retirement invokes the existing callback once and
 * queues its transferred reference for the worker's OUTSIDE-lock signal/put.
 * This never starts a new lane item or signals/drops refs inline. Published
 * accepted close does not revoke ACTIVE retirement; an exact closed READY
 * item is canceled locally and queued for its ordinary error-fence drain.
 * Caller retains the exact runtime/job owner.
 */
int gpu_m4_drm_provider_lane_service_timeout_locked(
	struct gpu_m4_drm_provider_lane *lane, struct gpu_m4_drm_provider *expected_provider,
	struct gpu_m4_drm_provider_lane_item *item, struct gpu_m4_drm_provider_job *job);
/* Optional command execution clock, configured once while PRIVATE. Runtime
 * supplies the exact accepted batch count (depth defaults to one: serial) and
 * optionally a bounded publication depth. No preparation budget or scheduler
 * recheck interval is changed. Only the actual publisher calls published
 * AFTER reachable producer publication, with exactly the next contiguous
 * index while fewer than depth commands are unreceipted; only the complete
 * hardware predicate of the current HEAD calls retired. Both require the same
 * ACTIVE protected job. Polling, ring consumption and a pending stamp must
 * never call these hooks.
 */
int gpu_m4_drm_provider_lane_item_set_commands_locked(
	struct gpu_m4_drm_provider_lane_item *item, u32 count);
int gpu_m4_drm_provider_lane_item_set_depth_locked(
	struct gpu_m4_drm_provider_lane_item *item, u32 depth);
int gpu_m4_drm_provider_lane_command_published_locked(
	struct gpu_m4_drm_provider_lane_item *item,
	struct gpu_m4_drm_provider_job *job, u32 index);
int gpu_m4_drm_provider_lane_command_retired_locked(
	struct gpu_m4_drm_provider_lane_item *item,
	struct gpu_m4_drm_provider_job *job, u32 index);
/* Exact registered item/job diagnostic copy remains available after a fault.
 * Never grants a healthy owner proof; no pointers or payload are returned.
 */
int gpu_m4_drm_provider_lane_item_timing_locked(
	struct gpu_m4_drm_provider_lane_item *item,
	struct gpu_m4_drm_provider_job *job,
	struct gpu_m4_drm_provider_lane_timing *timing);
/* Optional paired owner/callback, set at most once while registered PRIVATE,
 * before enqueue. NULL/NULL explicitly leaves the normal callback-free path.
 * owner is already strongly retained by the runtime payload with this item.
 * Callback runs ONLY after job_retire_locked returns zero, while the exact item
 * remains ACTIVE and before clearing it or starting another job. Success fence
 * is already committed: this is infallible bookkeeping, not a validation gate.
 * It may detach only that job's token and queue held refs for outside-lock reap.
 * NO final ref puts, signal/wait, hardware write or recursive queue/lane close.
 * Error/cancel/quarantine never notify, and retain all uncertain token owners.
 */
int gpu_m4_drm_provider_lane_item_set_retired_locked(
	struct gpu_m4_drm_provider_lane_item *item, void *owner,
	void (*retired_locked)(void *owner, struct gpu_m4_drm_provider_lane_item *item,
			       struct gpu_m4_drm_provider_job *job));
/* Optional host scheduling level of a still PRIVATE item (default 0). The
 * global lane is where the logical queues' packets meet: each released
 * packet (a bounded child window of a larger submit) returns its turn, and
 * the next READY item is the oldest one of the highest level. Strict
 * precedence between levels, FIFO within one; it never preempts an ACTIVE
 * item, reorders one queue's own items or changes any firmware field.
 */
int gpu_m4_drm_provider_lane_item_set_priority_locked(
	struct gpu_m4_drm_provider_lane_item *item, u32 priority);
/* Infallibly consumes the scheduler run callback's ONE transferred job ref.
 * Entries are ordered by priority, then dependency readiness, NEVER ioctl
 * acceptance order.
 * Enqueue schedules independent work; it never publishes or signals inline.
 * The worker signals already completed items, outside the mutex, BEFORE it
 * activates the next ready item: no fence waits for a successor's start.
 */
void gpu_m4_drm_provider_lane_enqueue_locked(
	struct gpu_m4_drm_provider_lane_item *item);
void gpu_m4_drm_provider_lane_close_queue_locked(
	struct gpu_m4_drm_provider_lane *lane, void *queue);
/* IRQ-safe exact firmware notification entrance (EP 0x20 shared-ring
 * activity). Only records a sequence and queues immediate lane work: no mutex,
 * ring drain, page walk, fence signal or put. A coalesced wake hint, never a
 * completion. The caller retains the lane for every possible callback.
 */
void gpu_m4_drm_provider_lane_notify(struct gpu_m4_drm_provider_lane *lane);
/* REMOVE-DEBUG: observational latency aggregates, never authority. */
struct gpu_m4_drm_provider_lane_stats {
	u64 jobs, queue_ns, queue_max, start_ns, start_max;
	u64 commands, command_ns, command_max, gaps, gap_ns, gap_max;
	u64 polls, pending_polls, timer_wakes, notify_wakes;
	u64 ahead; /* commands published while a predecessor was outstanding */
	/* Worker mutex: wait to acquire it and every hold. A start hold is the
	 * one in which an item was activated (its whole start and first window).
	 */
	u64 waits, wait_ns, wait_max, holds, hold_ns, hold_max;
	u64 start_holds, start_hold_ns, start_hold_max;
	/* Activations whose item was READY while the previous head was ACTIVE,
	 * by class against that head: same queue, same VM (other queue) and a
	 * different VM. Never an admission or retirement input.
	 */
	u64 overlap, overlap_queue, overlap_vm, overlap_foreign;
	u64 promotions; /* in-flight successors that became the head */
	u64 successor_services; /* head services right after a successor's start */
};
void gpu_m4_drm_provider_lane_stats_locked(struct gpu_m4_drm_provider_lane *lane,
	struct gpu_m4_drm_provider_lane_stats *out);
/* Nonzero only inside the worker's own continuous mutex hold (start, service
 * and their callbacks), constant for that whole hold and never reused by a
 * later one; 0 for every other holder (TDR service, ioctls, other workers).
 * Equal nonzero values prove no unlock happened in between. An identity for
 * callers' same-hold memos, never a lane, phase or health authority.
 */
u64 gpu_m4_drm_provider_lane_hold_locked(struct gpu_m4_drm_provider_lane *lane);
/* The lane's latched failure (or -ESHUTDOWN once closing), else 0: a scalar
 * publication gate input, no membership walk or hardware read.
 */
int gpu_m4_drm_provider_lane_error_locked(struct gpu_m4_drm_provider_lane *lane);
/* Stop new publication. The worker latches errors, signals outside the mutex
 * and preserves uncertain active backing. No reset or teardown is implied.
 */
void gpu_m4_drm_provider_lane_fault_locked(
	struct gpu_m4_drm_provider_lane *lane, int error);

#endif
