/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_DRM_SCHEDULER_H
#define APPLE_GPU_M4_DRM_SCHEDULER_H

#include <drm/gpu_scheduler.h>
#include "gpu-m4-drm.h"

struct gpu_m4_drm_sched_queue;

/*
 * This helper owns software scheduling, not RTKit, UAT or GPU recovery. It is
 * not a provider and registering it alone must never expose a render node.
 * Every operation is required. The provider/owner remains alive until closed.
 */
struct gpu_m4_drm_sched_ops {
	/*
	 * Last fallible accepting step, before job_arm. No GPU publication.
	 * Snapshot and retain queue/VM/GEM/timestamp mappings synchronously; the
	 * lookup callback cannot outlive this call. Command/input storage is owned
	 * by the helper through release, but every backing/mapping ref is YOURS.
	 * Reserve BO fence slots here; reservation locks may remain held until
	 * publish_finished, which follows immediately without a fallible step.
	 * ERR_PTR must leave no provider refs, locks or firmware-visible writes.
	 */
	void *(*prepare)(void *owner, const struct gpu_m4_drm_submit *submit);
	/*
	 * Infallibly install the borrowed actual scheduler FINISHED fence into
	 * every accessed BO reservation, then release prepare's reservation locks.
	 * This runs before enqueue and before returning the fence to the frontend.
	 */
	void (*publish_finished)(void *owner, void *job, struct dma_fence *finished);
	/*
	 * Enumerate OWNED exact implicit dma_resv snapshots captured in prepare
	 * under the same locks as reserve/publish_finished. NULL ends enumeration;
	 * ERR_PTR aborts before GPU publication. Never reread current reservations.
	 * Called after explicit input submissions/execution, outside all helper
	 * mutexes/spinlocks and the frontend lock. No allocation or waiting here.
	 * Retain the provider's snapshots through release (the helper owns its
	 * returned refs). Close cancels unpublished jobs without awaiting them.
	 */
	struct dma_fence *(*next_dependency)(void *owner, void *job);
	/*
	 * Optional. Called at most once per armed job, when it holds this queue's
	 * scheduling credit (every earlier job of the queue has completed) and its
	 * dependency resolution begins: explicit inputs may still be unsubmitted
	 * and implicit ones unsignaled. Outside all helper mutexes/spinlocks. Only
	 * CPU-side preparation that publishes nothing may start here; run still
	 * follows only after every dependency. Never wait or fail the job; close or
	 * a dependency error may still release it without run.
	 */
	void (*begin_dependencies)(void *owner, void *job);
	/*
	 * Nonblocking hardware publication, serialized against queue close. Return
	 * an OWNED real retirement fence, signaled only after qualified GPU/cache
	 * completion. ERR_PTR guarantees NO GPU publication. NULL is an error,
	 * not success; it is treated as potentially published and retained.
	 * Never wait for inputs/hardware or call queue_close under this callback.
	 * Perform qualified WB cleaning only now, AFTER every explicit/implicit
	 * input completes; successful retirement includes required CPU visibility.
	 */
	struct dma_fence *(*run)(void *owner, void *job);
	/* Latch VM error before a helper-detected uncertain error is signaled. */
	void (*fault)(void *owner, void *job, int error);
	/*
	 * Optional. An armed job the helper refuses to run (an input/implicit
	 * dependency error, an unsubmitted input, cancellation): latch the
	 * owner's VM error before drm_sched signals that error. Nothing was
	 * published, so this is never a provider fault. Called from run_job
	 * under the helper queue mutex; no wait.
	 */
	void (*fail)(void *owner, void *job, int error);
	/*
	 * Optional policy for a signaled explicit/implicit dependency that
	 * carries an error. Return 0 to run the job anyway (the failure is not
	 * this owner's: per-client isolation), or a negative errno to refuse it
	 * (then fail() latches before the signal). Without it every dependency
	 * error refuses the job. No wait; may be called under the queue mutex.
	 */
	int (*dependency_error)(void *owner, void *job, int error);
	/*
	 * Qualified bounded recovery/fail-stop is provider duty. It must latch VM
	 * error BEFORE error-fence signaling and retain uncertain GPU-visible
	 * backing. No generic reset of shared RTKit is authorized by this helper.
	 * Follow the local drm_sched timeout/stop/start ownership contract: the
	 * guilty job has been removed from pending_list on entry. Return RESET
	 * only after real recovery, NO_HANG only for a proven false timeout.
	 * ENODEV alone does not dispose of the removed job or signal its fence.
	 * Calls are outside helper mutexes. Do not free sched_job directly.
	 */
	enum drm_gpu_sched_stat (*timedout)(void *owner, void *job,
					   struct drm_sched_job *sched_job);
	/*
	 * Last helper use of the provider cookie. If uncertain, transfer/retain
	 * all firmware-visible refs before returning; cookie cleanup is NOT
	 * evidence that hardware stopped. Called without helper mutexes.
	 */
	void (*release)(void *owner, void *job, bool uncertain);
	/* All jobs, observer work, entity and scheduler are now gone. */
	void (*closed)(void *owner);
};

/* At most this many credits per queue (two packets in flight). */
enum { GPU_M4_DRM_SCHED_MAX_CREDITS = 2 };

struct gpu_m4_drm_sched_config {
	struct device *dev;
	void *owner;
	const struct gpu_m4_drm_sched_ops *ops;
	/* DRM_ASAHI_PRIORITY_*: LOW/MEDIUM/HIGH/REALTIME select the Low/Normal/
	 * High/Kernel entity level (num_rqs covers all four). The caller checked
	 * the privilege; no firmware priority is implied.
	 */
	u32 priority;
	u64 client_id;
	long timeout; /* positive finite jiffies, provider recovery is mandatory */
	/* Scheduler credits (jobs run, i.e. handed to the provider, at once):
	 * 0 or 1 serial, 2 when the provider runs two packets in flight.
	 */
	u32 credit_limit;
};

/* Create returns one owned queue ref. No hardware/node registration occurs. */
struct gpu_m4_drm_sched_queue *
gpu_m4_drm_sched_queue_create(const struct gpu_m4_drm_sched_config *config);
void gpu_m4_drm_sched_queue_get(struct gpu_m4_drm_sched_queue *queue);
/* Call close before dropping the last caller-owned ref. */
void gpu_m4_drm_sched_queue_put(struct gpu_m4_drm_sched_queue *queue);
/* Idempotent, nonblocking: cancel unpublished jobs, retain published jobs. */
void gpu_m4_drm_sched_queue_close(struct gpu_m4_drm_sched_queue *queue);
/* Explicit destroy refuses busy queues; success closes but does not put. */
int gpu_m4_drm_sched_queue_try_close(struct gpu_m4_drm_sched_queue *queue);

/*
 * ERR_PTR means unaccepted and no GPU publication; success is an owned actual
 * scheduler finished fence. Acceptance never waits for submission/execution of
 * future points. Caller holds a queue ref throughout this call.
 */
struct dma_fence *
gpu_m4_drm_sched_submit(struct gpu_m4_drm_sched_queue *queue,
			const struct gpu_m4_drm_submit *submit);

#endif
