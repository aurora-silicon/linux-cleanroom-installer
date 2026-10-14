// SPDX-License-Identifier: GPL-2.0-only
/* Software dependency ownership for the strictly gated retained GPU provider. */
#include <linux/device.h>
#include <linux/dma-fence-unwrap.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <drm/drm_syncobj.h>
#include "gpu-m4-drm-scheduler.h"
#include "gpu-m4-drm-validation.h"

struct gpu_m4_drm_sched_gate {
	struct dma_fence fence;
	spinlock_t lock;
};

/*
 * Ordered terminal of one armed job when the queue runs several credits: the
 * scheduler FINISHED fences of one entity share a context, so a later job's
 * must never signal before an earlier one's (dma-fence ordering, dma_resv
 * replacement), success or error. This fence, handed to drm_sched as the
 * job's hardware fence, signals once the previous job's FINISHED fence has
 * signaled AND (if the job ran) the provider's retirement fence has; it
 * carries only the provider's (or the refusal's) error, never the previous
 * job's. Each pending callback owns one fence reference.
 */
struct gpu_m4_drm_sched_terminal {
	struct dma_fence fence;
	spinlock_t lock;
	struct dma_fence_cb previous_cb, hardware_cb;
	struct dma_fence *hardware; /* owned, put at release */
	atomic_t pending;
	int error;
};

struct gpu_m4_drm_sched_job;

struct gpu_m4_drm_sched_input {
	struct drm_syncobj_callback callback;
	struct gpu_m4_drm_sched_job *job;
	struct dma_fence *resolved;
	u32 index;
};

struct gpu_m4_drm_sched_queue {
	struct kref ref;
	struct mutex lock; /* acceptance, run publication, close, job list */
	struct drm_gpu_scheduler scheduler;
	struct drm_sched_entity entity;
	struct list_head jobs;
	struct work_struct close_work;
	wait_queue_head_t idle;
	atomic_t job_count;
	struct device *dev;
	struct gpu_m4_drm_sched_ops ops;
	void *owner;
	char *name;
	u64 client_id;
	struct dma_fence *last_finished; /* newest armed job's FINISHED (credits > 1) */
	u32 credit_limit;
	bool closed, drained;
};

struct gpu_m4_drm_sched_job {
	struct drm_sched_job base;
	struct kref ref;
	struct gpu_m4_drm_sched_queue *queue;
	struct list_head node;
	/* Serializes observer ADD against REMOVE/reuse/storage destruction. */
	struct mutex preparation;
	spinlock_t lock; /* resolved inputs, remaining, canceled */
	struct work_struct ready_work;
	struct gpu_m4_drm_sched_gate *gate;
	/* Credit two only: the previous armed job's FINISHED fence (owned) and
	 * this job's preallocated ordered terminal, consumed by run_job.
	 */
	struct dma_fence *previous;
	struct gpu_m4_drm_sched_terminal *terminal;
	struct gpu_m4_drm_submit submit;
	struct gpu_m4_drm_dependency *inputs; /* immutable captured snapshots */
	struct gpu_m4_drm_sched_input *dependencies;
	struct dma_fence *implicit;
	void *cookie;
	u32 remaining, next_dependency;
	int dependency_error;
	bool initialized, tracked, close_seen, canceled, published, uncertain;
	bool dependencies_begun; /* only the scheduler's serialized prepare_job */
};

static const char *gpu_m4_drm_gate_driver(struct dma_fence *fence)
{
	return "apple-m4-submission";
}

static const char *gpu_m4_drm_gate_timeline(struct dma_fence *fence)
{
	return "input-submission-known";
}

static void gpu_m4_drm_gate_release(struct dma_fence *fence)
{
	kfree(container_of(fence, struct gpu_m4_drm_sched_gate, fence));
}

static const struct dma_fence_ops gpu_m4_drm_gate_ops = {
	.get_driver_name = gpu_m4_drm_gate_driver,
	.get_timeline_name = gpu_m4_drm_gate_timeline,
	.release = gpu_m4_drm_gate_release,
};

static const char *gpu_m4_drm_terminal_timeline(struct dma_fence *fence)
{
	return "queue-ordered-finish";
}

static void gpu_m4_drm_terminal_release(struct dma_fence *fence)
{
	struct gpu_m4_drm_sched_terminal *t =
		container_of(fence, struct gpu_m4_drm_sched_terminal, fence);

	dma_fence_put(t->hardware);
	kfree(t);
}

static const struct dma_fence_ops gpu_m4_drm_terminal_ops = {
	.get_driver_name = gpu_m4_drm_gate_driver,
	.get_timeline_name = gpu_m4_drm_terminal_timeline,
	.release = gpu_m4_drm_terminal_release,
};

static struct gpu_m4_drm_sched_terminal *gpu_m4_drm_terminal_create(void)
{
	struct gpu_m4_drm_sched_terminal *t = kzalloc(sizeof(*t), GFP_KERNEL);

	if (!t)
		return NULL;
	spin_lock_init(&t->lock);
	dma_fence_init(&t->fence, &gpu_m4_drm_terminal_ops, &t->lock,
		       dma_fence_context_alloc(1), 1);
	return t;
}

static void gpu_m4_drm_terminal_arrive(struct gpu_m4_drm_sched_terminal *t)
{
	if (atomic_dec_and_test(&t->pending)) {
		if (t->error)
			dma_fence_set_error(&t->fence, t->error);
		dma_fence_signal(&t->fence);
	}
	dma_fence_put(&t->fence); /* this arrival's reference */
}

static void gpu_m4_drm_terminal_previous(struct dma_fence *f, struct dma_fence_cb *cb)
{
	(void)f; /* the previous job's own error is never this job's */
	gpu_m4_drm_terminal_arrive(container_of(cb, struct gpu_m4_drm_sched_terminal,
		previous_cb));
}

static void gpu_m4_drm_terminal_hardware(struct dma_fence *f, struct dma_fence_cb *cb)
{
	struct gpu_m4_drm_sched_terminal *t =
		container_of(cb, struct gpu_m4_drm_sched_terminal, hardware_cb);

	if (f->error)
		t->error = f->error;
	gpu_m4_drm_terminal_arrive(t);
}

/* Consumes hardware (an owned provider fence, or ERR_PTR) into t, whose one
 * returned reference is drm_sched's hardware fence for the job.
 */
static struct dma_fence *gpu_m4_drm_terminal_arm(struct gpu_m4_drm_sched_terminal *t,
	struct dma_fence *previous, struct dma_fence *hardware)
{
	bool ran = !IS_ERR(hardware);

	if (!ran)
		t->error = PTR_ERR(hardware);
	else
		t->hardware = hardware;
	atomic_set(&t->pending, 1 + ran);
	dma_fence_get(&t->fence);
	if (dma_fence_add_callback(previous, &t->previous_cb, gpu_m4_drm_terminal_previous))
		gpu_m4_drm_terminal_previous(previous, &t->previous_cb);
	if (ran) {
		dma_fence_get(&t->fence);
		if (dma_fence_add_callback(hardware, &t->hardware_cb, gpu_m4_drm_terminal_hardware))
			gpu_m4_drm_terminal_hardware(hardware, &t->hardware_cb);
	}
	return &t->fence; /* the creation reference */
}

static struct gpu_m4_drm_sched_gate *gpu_m4_drm_gate_create(void)
{
	struct gpu_m4_drm_sched_gate *gate;

	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate)
		return NULL;
	spin_lock_init(&gate->lock);
	dma_fence_init(&gate->fence, &gpu_m4_drm_gate_ops, &gate->lock,
		       dma_fence_context_alloc(1), 1);
	return gate;
}

static void gpu_m4_drm_gate_signal(struct gpu_m4_drm_sched_gate *gate, int error)
{
	unsigned long flags;

	spin_lock_irqsave(&gate->lock, flags);
	if (!dma_fence_is_signaled_locked(&gate->fence)) {
		if (error)
			dma_fence_set_error(&gate->fence, error);
		dma_fence_signal_locked(&gate->fence);
	}
	spin_unlock_irqrestore(&gate->lock, flags);
}

static void gpu_m4_drm_sched_queue_free(struct kref *ref)
{
	struct gpu_m4_drm_sched_queue *queue;

	queue = container_of(ref, struct gpu_m4_drm_sched_queue, ref);
	/* A caller must close its owner ref; never tear down live GPU state here. */
	if (WARN_ON(!queue->drained))
		return;
	dma_fence_put(queue->last_finished);
	put_device(queue->dev);
	kfree(queue->name);
	kfree(queue);
}

void gpu_m4_drm_sched_queue_get(struct gpu_m4_drm_sched_queue *queue)
{
	kref_get(&queue->ref);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_sched_queue_get);

void gpu_m4_drm_sched_queue_put(struct gpu_m4_drm_sched_queue *queue)
{
	kref_put(&queue->ref, gpu_m4_drm_sched_queue_free);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_sched_queue_put);

static void gpu_m4_drm_sched_job_free(struct kref *ref)
{
	struct gpu_m4_drm_sched_job *job;
	struct gpu_m4_drm_sched_queue *queue;
	u32 i;

	job = container_of(ref, struct gpu_m4_drm_sched_job, ref);
	queue = job->queue;
	/* Unlist first: a job whose count reached zero must not stay visible to
	 * the close worker's scan while its (possibly long) release runs. The
	 * idle count still drops only after every release step below.
	 */
	if (job->tracked) {
		mutex_lock(&queue->lock);
		list_del_init(&job->node);
		mutex_unlock(&queue->lock);
	}
	if (job->cookie)
		queue->ops.release(queue->owner, job->cookie, job->uncertain);
	for (i = 0; i < job->submit.input_count; i++) {
		dma_fence_put(job->dependencies[i].resolved);
		dma_fence_put(job->inputs[i].fence);
		if (job->inputs[i].object)
			drm_syncobj_put(job->inputs[i].object);
	}
	dma_fence_put(job->implicit);
	dma_fence_put(job->previous);
	if (job->terminal)
		dma_fence_put(&job->terminal->fence); /* never armed */
	dma_fence_put(job->gate ? &job->gate->fence : NULL);
	kfree(job->dependencies);
	kfree(job->inputs);
	kfree(job->submit.cmdbuf);
	if (job->tracked && atomic_dec_and_test(&queue->job_count))
		wake_up_all(&queue->idle);
	kfree(job);
	gpu_m4_drm_sched_queue_put(queue);
}

static void gpu_m4_drm_sched_job_put(struct gpu_m4_drm_sched_job *job)
{
	kref_put(&job->ref, gpu_m4_drm_sched_job_free);
}

/* No sleep/mutation/remove under the syncobj lock, only refs and queued work. */
static void gpu_m4_drm_sched_input_ready(struct drm_syncobj_callback *callback,
				       struct dma_fence *fence)
{
	struct gpu_m4_drm_sched_input *input;
	struct gpu_m4_drm_sched_job *job;
	unsigned long flags;

	input = container_of(callback, struct gpu_m4_drm_sched_input, callback);
	job = input->job;
	spin_lock_irqsave(&job->lock, flags);
	if (!job->canceled && !input->resolved) {
		input->resolved = dma_fence_get(fence);
		job->remaining--;
		/* A successful queue owns this ref through run OR cancel_work_sync. */
		kref_get(&job->ref);
		if (!queue_work(system_unbound_wq, &job->ready_work))
			gpu_m4_drm_sched_job_put(job);
	}
	spin_unlock_irqrestore(&job->lock, flags);
}

static void gpu_m4_drm_sched_ready_work(struct work_struct *work)
{
	struct gpu_m4_drm_sched_job *job;
	unsigned long flags;
	bool ready, canceled;
	u32 i;

	job = container_of(work, struct gpu_m4_drm_sched_job, ready_work);
	mutex_lock(&job->preparation);
	/* Registration has returned before any observer can be removed. */
	for (i = 0; i < job->submit.input_count; i++) {
		spin_lock_irqsave(&job->lock, flags);
		ready = job->dependencies[i].resolved != NULL;
		spin_unlock_irqrestore(&job->lock, flags);
		if (ready)
			drm_syncobj_remove_callback(&job->dependencies[i].callback);
	}
	spin_lock_irqsave(&job->lock, flags);
	ready = !job->remaining;
	canceled = job->canceled;
	spin_unlock_irqrestore(&job->lock, flags);
	mutex_unlock(&job->preparation);
	if (ready || canceled)
		gpu_m4_drm_gate_signal(job->gate, canceled ? -ECANCELED : 0);
	gpu_m4_drm_sched_job_put(job);
}

static void gpu_m4_drm_sched_cancel_inputs(struct gpu_m4_drm_sched_job *job)
{
	unsigned long flags;
	u32 i;

	mutex_lock(&job->preparation);
	spin_lock_irqsave(&job->lock, flags);
	job->canceled = true;
	spin_unlock_irqrestore(&job->lock, flags);
	for (i = 0; i < job->submit.input_count; i++)
		drm_syncobj_remove_callback(&job->dependencies[i].callback);
	mutex_unlock(&job->preparation);
	/* Callback removal has quiesced every producer before work cancellation. */
	if (cancel_work_sync(&job->ready_work))
		gpu_m4_drm_sched_job_put(job);
	if (job->gate)
		gpu_m4_drm_gate_signal(job->gate, -ECANCELED);
}

static int gpu_m4_drm_sched_dependency_status(struct dma_fence *fence)
{
	struct dma_fence_unwrap cursor;
	struct dma_fence *leaf;
	int status = dma_fence_get_status(fence);

	if (status <= 0)
		return status;
	/* Chain containers do not copy their contained hardware fence errors. */
	dma_fence_unwrap_for_each(leaf, &cursor, fence) {
		int error = dma_fence_get_status(leaf);

		if (error < 0 && status > 0)
			status = error;
		/* Complete the iterator even on error: it owns container refs. */
	}
	return status;
}

/*
 * A signaled dependency carried an error. Without a provider policy the job
 * is refused with it. With one, the provider decides: 0 means the failure is
 * not this owner's (another VM, file or a foreign/sync_file producer) and the
 * job runs on whatever that producer left; negative refuses it. Called
 * outside the job spinlock; the caller may hold the queue mutex.
 */
static int gpu_m4_drm_sched_dependency_failed(struct gpu_m4_drm_sched_job *job, int error)
{
	struct gpu_m4_drm_sched_queue *queue = job->queue;

	if (!queue->ops.dependency_error)
		return error;
	return queue->ops.dependency_error(queue->owner, job->cookie, error);
}

/*
 * drm_sched never waits for a fence of the job's own entity (its scheduled or
 * finished context): the entity runs its jobs in order and, with several credits,
 * an earlier job of this queue may still be executing when a later one that
 * names it is prepared. The provider orders the later packet's first command
 * behind its queue's published window, so such a still-pending input is
 * satisfied by that order. It is never a dependency error, and returning it
 * again would only make drm_sched skip it again.
 */
static bool gpu_m4_drm_sched_own_pending(struct gpu_m4_drm_sched_job *job,
	struct dma_fence *fence)
{
	u64 context = job->queue->entity.fence_context;

	return fence && !dma_fence_is_signaled(fence) &&
		(fence->context == context || fence->context == context + 1);
}

static struct dma_fence *
gpu_m4_drm_sched_prepare_job(struct drm_sched_job *base,
			     struct drm_sched_entity *entity)
{
	struct gpu_m4_drm_sched_job *job;
	struct dma_fence *fence = NULL;
	unsigned long flags;
	bool implicit = false;
	u32 i;
	int status;

	job = container_of(base, struct gpu_m4_drm_sched_job, base);
	/* drm_sched resolves dependencies only for an entity's next job that
	 * already holds scheduling credit: with credit one, every earlier job
	 * of this queue has completed (with more, every earlier one already
	 * ran, the newest may still execute). The provider may begin unpublished CPU
	 * preparation now, outside every helper lock; run still waits below.
	 */
	if (!job->dependencies_begun) {
		bool begin;

		job->dependencies_begun = true;
		spin_lock_irqsave(&job->lock, flags);
		begin = !job->canceled && !job->dependency_error;
		spin_unlock_irqrestore(&job->lock, flags);
		if (begin && job->queue->ops.begin_dependencies)
			job->queue->ops.begin_dependencies(job->queue->owner, job->cookie);
	}
	spin_lock_irqsave(&job->lock, flags);
	if (job->canceled || job->dependency_error)
		goto out;
	if (job->remaining) {
		/* PRIVATE gate: knowing submissions is not finished rendering. */
		fence = dma_fence_get(&job->gate->fence);
		goto out;
	}
	while (job->next_dependency < job->submit.input_count) {
		fence = job->dependencies[job->next_dependency++].resolved;
		if (!dma_fence_is_signaled(fence) && !gpu_m4_drm_sched_own_pending(job, fence)) {
			fence = dma_fence_get(fence);
			goto out;
		}
	}
	fence = NULL;
	implicit = true;
out:
	spin_unlock_irqrestore(&job->lock, flags);
	if (!implicit)
		return fence;
	/* Scheduler owns this job until entity_idle; provider calls never hold
	 * the input spinlock, queue/frontend mutex or observer preparation lock.
	 */
	for (i = 0; i < job->submit.input_count; i++) {
		struct dma_fence *input = job->dependencies[i].resolved;

		if (gpu_m4_drm_sched_own_pending(job, input))
			continue;
		status = gpu_m4_drm_sched_dependency_status(input);
		if (status < 0)
			status = gpu_m4_drm_sched_dependency_failed(job, status);
		else if (!status)
			return dma_fence_get(input); /* not yet signaled: wait again */
		else
			status = 0;
		if (status) {
			job->dependency_error = status;
			return NULL;
		}
	}
	for (;;) {
		if (job->implicit) {
			status = gpu_m4_drm_sched_own_pending(job, job->implicit) ? 1 :
				gpu_m4_drm_sched_dependency_status(job->implicit);
			if (!status)
				return dma_fence_get(job->implicit);
			dma_fence_put(job->implicit);
			job->implicit = NULL;
			if (status < 0)
				status = gpu_m4_drm_sched_dependency_failed(job, status);
			else
				status = 0;
			if (status) {
				job->dependency_error = status;
				return NULL;
			}
		}
		fence = job->queue->ops.next_dependency(job->queue->owner, job->cookie);
		if (IS_ERR(fence)) {
			job->dependency_error = PTR_ERR(fence);
			return NULL;
		}
		if (!fence)
			return NULL;
		job->implicit = fence; /* provider returned an owned exact snapshot */
	}
}

static struct dma_fence *gpu_m4_drm_sched_run_job(struct drm_sched_job *base)
{
	struct gpu_m4_drm_sched_job *job;
	struct gpu_m4_drm_sched_queue *queue;
	struct dma_fence *fence;
	unsigned long flags;
	int error = 0;
	u32 i;

	job = container_of(base, struct gpu_m4_drm_sched_job, base);
	queue = job->queue;
	mutex_lock(&queue->lock);
	spin_lock_irqsave(&job->lock, flags);
	if (job->canceled || queue->closed)
		error = -ECANCELED;
	else if (job->dependency_error)
		error = job->dependency_error;
	else if (job->remaining)
		error = -EIO;
	spin_unlock_irqrestore(&job->lock, flags);
	/* prepare_job already accepted (or refused) every explicit input's
	 * error; only an unsignaled input here is impossible and refused.
	 */
	for (i = 0; !error && i < job->submit.input_count; i++) {
		struct dma_fence *input = job->dependencies[i].resolved;
		int status;

		if (gpu_m4_drm_sched_own_pending(job, input))
			continue;
		status = gpu_m4_drm_sched_dependency_status(input);
		if (!status)
			error = -EIO;
		else if (status < 0)
			error = gpu_m4_drm_sched_dependency_failed(job, status);
	}
	if (error) {
		/* REMOVE-DEBUG: which refusal. */
		pr_warn_ratelimited("GPU sched refuses job: error=%d canceled=%d closed=%d dependency=%d remaining=%u\n",
			error, job->canceled, queue->closed, job->dependency_error, job->remaining);
		/* Admitted, never published: its VM still learns the failure. */
		if (queue->ops.fail)
			queue->ops.fail(queue->owner, job->cookie, error);
		fence = ERR_PTR(error);
		goto out;
	}
	fence = queue->ops.run(queue->owner, job->cookie);
	job->published = !IS_ERR(fence);
	if (!fence) {
		/* drm_sched treats NULL as SUCCESS: never permit that false result. */
		job->uncertain = true;
		queue->ops.fault(queue->owner, job->cookie, -EIO);
		fence = ERR_PTR(-EIO);
	}
out:
	/* Several credits: FINISHED stays in entity order, success or error. */
	if (job->terminal && job->previous) {
		struct gpu_m4_drm_sched_terminal *t = job->terminal;

		job->terminal = NULL;
		fence = gpu_m4_drm_terminal_arm(t, job->previous, fence);
	}
	mutex_unlock(&queue->lock);
	return fence;
}

static enum drm_gpu_sched_stat
gpu_m4_drm_sched_timedout_job(struct drm_sched_job *base)
{
	struct gpu_m4_drm_sched_job *job;

	job = container_of(base, struct gpu_m4_drm_sched_job, base);
	return job->queue->ops.timedout(job->queue->owner, job->cookie, base);
}

static void gpu_m4_drm_sched_free_job(struct drm_sched_job *base)
{
	struct gpu_m4_drm_sched_job *job;

	job = container_of(base, struct gpu_m4_drm_sched_job, base);
	job->uncertain |= job->published &&
		dma_fence_get_status(&base->s_fence->finished) < 0;
	gpu_m4_drm_sched_cancel_inputs(job);
	drm_sched_job_cleanup(base);
	job->initialized = false;
	gpu_m4_drm_sched_job_put(job);
}

static const struct drm_sched_backend_ops gpu_m4_drm_scheduler_ops = {
	.prepare_job = gpu_m4_drm_sched_prepare_job,
	.run_job = gpu_m4_drm_sched_run_job,
	.timedout_job = gpu_m4_drm_sched_timedout_job,
	.free_job = gpu_m4_drm_sched_free_job,
};

static void gpu_m4_drm_sched_close_work(struct work_struct *work)
{
	struct gpu_m4_drm_sched_queue *queue;
	struct gpu_m4_drm_sched_job *job, *found;

	queue = container_of(work, struct gpu_m4_drm_sched_queue, close_work);
	for (;;) {
		found = NULL;
		mutex_lock(&queue->lock);
		list_for_each_entry(job, &queue->jobs, node) {
			if (job->close_seen)
				continue;
			job->close_seen = true;
			/* A job whose final put already began is being freed by
			 * that put (it unlists itself under this lock): never
			 * revive it. Its inputs need no cancellation here.
			 */
			if (!kref_get_unless_zero(&job->ref))
				continue;
			found = job;
			break;
		}
		mutex_unlock(&queue->lock);
		if (!found)
			break;
		gpu_m4_drm_sched_cancel_inputs(found);
		gpu_m4_drm_sched_job_put(found);
	}
	/* No external-dependency xarray: unexecuted jobs can be killed now. */
	drm_sched_entity_fini(&queue->entity);
	/* Published jobs retain their true hardware fence/provider pins. */
	wait_event(queue->idle, !atomic_read(&queue->job_count));
	drm_sched_fini(&queue->scheduler);
	queue->ops.closed(queue->owner);
	queue->drained = true;
	gpu_m4_drm_sched_queue_put(queue); /* work-owned ref */
}

static void gpu_m4_drm_sched_close_locked(struct gpu_m4_drm_sched_queue *queue)
{
	struct gpu_m4_drm_sched_job *job;
	unsigned long flags;

	lockdep_assert_held(&queue->lock);
	if (queue->closed)
		return;
	queue->closed = true;
	list_for_each_entry(job, &queue->jobs, node) {
		spin_lock_irqsave(&job->lock, flags);
		job->canceled = true;
		spin_unlock_irqrestore(&job->lock, flags);
	}
	gpu_m4_drm_sched_queue_get(queue);
	queue_work(system_unbound_wq, &queue->close_work);
}

void gpu_m4_drm_sched_queue_close(struct gpu_m4_drm_sched_queue *queue)
{
	mutex_lock(&queue->lock);
	gpu_m4_drm_sched_close_locked(queue);
	mutex_unlock(&queue->lock);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_sched_queue_close);

int gpu_m4_drm_sched_queue_try_close(struct gpu_m4_drm_sched_queue *queue)
{
	int ret = 0;

	mutex_lock(&queue->lock);
	if (atomic_read(&queue->job_count))
		ret = -EBUSY;
	else
		gpu_m4_drm_sched_close_locked(queue);
	mutex_unlock(&queue->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_sched_queue_try_close);

struct gpu_m4_drm_sched_queue *
gpu_m4_drm_sched_queue_create(const struct gpu_m4_drm_sched_config *config)
{
	struct gpu_m4_drm_sched_queue *queue;
	struct drm_gpu_scheduler *scheduler;
	struct drm_sched_init_args args = {
		.ops = &gpu_m4_drm_scheduler_ops,
		.num_rqs = DRM_SCHED_PRIORITY_COUNT,
		.credit_limit = 1,
	};
	enum drm_sched_priority priority;
	int ret;

	if (!config || !config->dev || !config->ops ||
	    !config->ops->prepare || !config->ops->publish_finished ||
	    !config->ops->next_dependency ||
	    !config->ops->run || !config->ops->timedout ||
	    !config->ops->fault || !config->ops->release || !config->ops->closed)
		return ERR_PTR(-EOPNOTSUPP);
	if (config->timeout <= 0 || config->timeout == MAX_SCHEDULE_TIMEOUT ||
	    config->credit_limit > (u32)GPU_M4_DRM_SCHED_QUEUE_CREDITS)
		return ERR_PTR(-EINVAL);
	args.credit_limit = config->credit_limit ?: 1;
	/* Host entity levels only: the shared firmware queues and their fields
	 * are identical for every priority (no firmware priority/preemption).
	 */
	switch (config->priority) {
	case DRM_ASAHI_PRIORITY_LOW:
		priority = DRM_SCHED_PRIORITY_LOW;
		break;
	case DRM_ASAHI_PRIORITY_MEDIUM:
		priority = DRM_SCHED_PRIORITY_NORMAL;
		break;
	case DRM_ASAHI_PRIORITY_HIGH:
		priority = DRM_SCHED_PRIORITY_HIGH;
		break;
	case DRM_ASAHI_PRIORITY_REALTIME:
		priority = DRM_SCHED_PRIORITY_KERNEL;
		break;
	default:
		return ERR_PTR(-EOPNOTSUPP);
	}
	queue = kzalloc(sizeof(*queue), GFP_KERNEL);
	if (!queue)
		return ERR_PTR(-ENOMEM);
	queue->name = kasprintf(GFP_KERNEL, "apple-m4-%llu", config->client_id);
	if (!queue->name) {
		kfree(queue);
		return ERR_PTR(-ENOMEM);
	}
	kref_init(&queue->ref);
	mutex_init(&queue->lock);
	INIT_LIST_HEAD(&queue->jobs);
	INIT_WORK(&queue->close_work, gpu_m4_drm_sched_close_work);
	init_waitqueue_head(&queue->idle);
	atomic_set(&queue->job_count, 0);
	queue->dev = get_device(config->dev);
	queue->owner = config->owner;
	queue->ops = *config->ops;
	queue->client_id = config->client_id;
	queue->credit_limit = args.credit_limit;
	args.dev = config->dev;
	args.name = queue->name;
	args.timeout = config->timeout;
	ret = drm_sched_init(&queue->scheduler, &args);
	if (ret)
		goto fail;
	scheduler = &queue->scheduler;
	ret = drm_sched_entity_init(&queue->entity, priority, &scheduler, 1, NULL);
	if (ret) {
		drm_sched_fini(&queue->scheduler);
		goto fail;
	}
	return queue;
fail:
	put_device(queue->dev);
	kfree(queue->name);
	kfree(queue);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_sched_queue_create);

static int gpu_m4_drm_sched_copy_inputs(struct gpu_m4_drm_sched_job *job,
				      const struct gpu_m4_drm_submit *submit)
{
	u32 i;

	job->inputs = kcalloc(submit->input_count, sizeof(*job->inputs), GFP_KERNEL);
	job->dependencies = kcalloc(submit->input_count,
				    sizeof(*job->dependencies), GFP_KERNEL);
	if (submit->input_count && (!job->inputs || !job->dependencies))
		return -ENOMEM;
	for (i = 0; i < submit->input_count; i++) {
		struct gpu_m4_drm_sched_input *input = &job->dependencies[i];
		const struct gpu_m4_drm_dependency *captured = &submit->inputs[i];

		/* Initialize ALL callback-used state before registering ANY input. */
		drm_syncobj_callback_init(&input->callback);
		input->job = job;
		input->index = i;
		if (!captured->object || IS_ERR(captured->fence))
			return -EINVAL;
		drm_syncobj_get(captured->object);
		job->inputs[i] = *captured;
		job->inputs[i].fence = dma_fence_get(captured->fence);
		input->resolved = dma_fence_get(captured->fence);
		job->submit.input_count++;
		if (!captured->fence)
			job->remaining++;
	}
	job->submit.inputs = job->inputs;
	return 0;
}

struct dma_fence *
gpu_m4_drm_sched_submit(struct gpu_m4_drm_sched_queue *queue,
			const struct gpu_m4_drm_submit *submit)
{
	struct gpu_m4_drm_sched_job *job;
	struct dma_fence *finished;
	u32 i;
	int ret;

	if (!submit || !submit->cmdbuf || !submit->size ||
	    submit->size > GPU_M4_DRM_MAX_CMDBUF ||
	    submit->input_count > GPU_M4_DRM_MAX_SYNCS ||
	    (submit->input_count && !submit->inputs))
		return ERR_PTR(-EINVAL);
	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job)
		return ERR_PTR(-ENOMEM);
	kref_init(&job->ref);
	mutex_init(&job->preparation);
	spin_lock_init(&job->lock);
	INIT_WORK(&job->ready_work, gpu_m4_drm_sched_ready_work);
	INIT_LIST_HEAD(&job->node);
	job->queue = queue;
	gpu_m4_drm_sched_queue_get(queue);
	job->gate = gpu_m4_drm_gate_create();
	if (queue->credit_limit > 1)
		job->terminal = gpu_m4_drm_terminal_create();
	job->submit.cmdbuf = kmemdup(submit->cmdbuf, submit->size, GFP_KERNEL);
	if (!job->gate || !job->submit.cmdbuf || (queue->credit_limit > 1 && !job->terminal)) {
		ret = -ENOMEM;
		goto fail;
	}
	job->submit.size = submit->size;
	job->submit.lookup_cookie = submit->lookup_cookie;
	job->submit.lookup_timestamp = submit->lookup_timestamp;
	ret = gpu_m4_drm_sched_copy_inputs(job, submit);
	if (ret)
		goto fail;
	mutex_lock(&queue->lock);
	if (queue->closed) {
		ret = -ECANCELED;
		goto fail_locked;
	}
	ret = drm_sched_job_init(&job->base, &queue->entity, 1, queue->owner,
				 queue->client_id);
	if (ret)
		goto fail_locked;
	job->initialized = true;
	mutex_lock(&job->preparation);
	for (i = 0; i < job->submit.input_count; i++) {
		if (job->inputs[i].fence)
			continue; /* never reread a captured binary snapshot */
		ret = drm_syncobj_add_callback(job->inputs[i].object,
					       &job->dependencies[i].callback,
					       job->inputs[i].point,
					       gpu_m4_drm_sched_input_ready);
		if (ret)
			break;
	}
	mutex_unlock(&job->preparation);
	if (ret)
		goto fail_locked;
	/* All allocations/observer adds precede the provider's accepting step. */
	job->cookie = queue->ops.prepare(queue->owner, &job->submit);
	if (IS_ERR_OR_NULL(job->cookie)) {
		ret = job->cookie ? PTR_ERR(job->cookie) : -EIO;
		job->cookie = NULL;
		goto fail_locked;
	}
	/* POINT OF NO RETURN: every armed job is pushed, no fallible operation. */
	drm_sched_job_arm(&job->base);
	finished = dma_fence_get(&job->base.s_fence->finished);
	set_bit(DRM_SCHED_FENCE_DONT_PIPELINE, &finished->flags);
	if (job->terminal) {
		/* The previous armed job of this entity, in arm (= run) order. */
		job->previous = queue->last_finished;
		queue->last_finished = dma_fence_get(finished);
	}
	queue->ops.publish_finished(queue->owner, job->cookie, finished);
	job->submit.lookup_cookie = NULL;
	job->submit.lookup_timestamp = NULL;
	job->tracked = true;
	list_add_tail(&job->node, &queue->jobs);
	atomic_inc(&queue->job_count);
	drm_sched_entity_push_job(&job->base);
	mutex_unlock(&queue->lock);
	return finished;
fail_locked:
	mutex_unlock(&queue->lock);
fail:
	gpu_m4_drm_sched_cancel_inputs(job);
	if (job->initialized)
		drm_sched_job_cleanup(&job->base);
	gpu_m4_drm_sched_job_put(job);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_sched_submit);

MODULE_DESCRIPTION("Apple M4 retained-provider DRM scheduling helper");
MODULE_LICENSE("GPL");
