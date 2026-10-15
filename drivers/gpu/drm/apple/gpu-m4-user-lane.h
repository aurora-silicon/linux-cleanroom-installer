/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_USER_LANE_H
#define APPLE_GPU_M4_USER_LANE_H

/* Ordinary scheduler glue, included after the root-owned runtime, logical
 * queue and m4_user_job types. The existing lane is the ONLY independent
 * worker: it starts/polls the real job core, signals outside the owner mutex,
 * and retains the transferred ref for every protected uncertain item.
 */
static void m4_user_job_retired_locked(struct m4_gpu_provider_runtime *p,
	struct m4_user_job *j);

/* Exact admitted in-flight slot of an ordinary core job, or -1. Membership
 * only: never a phase proof or lane authority.
 */
static int m4_user_inflight_slot_locked(const struct m4_gpu_provider_runtime *p,
	const struct gpu_m4_drm_provider_job *job)
{
	for (u32 i = 0; job && i < M4_USER_INFLIGHT_MAX; i++)
		if (p->user_inflight[i].job == job)
			return i;
	return -1;
}

static bool m4_user_inflight_item_locked(const struct m4_gpu_provider_runtime *p,
	const struct gpu_m4_drm_provider_lane_item *item, const struct gpu_m4_drm_provider_job *job)
{
	int slot = m4_user_inflight_slot_locked(p, job);

	return item && slot >= 0 && p->user_inflight[slot].item == item;
}

/* REMOVE-DEBUG: the ordinary lane's active-work deadline (default 2 s), so a
 * hang experiment can watch whether firmware starts its own recovery before
 * the host fails all work. Load time only; the lane bounds it to 60 s.
 */
static unsigned int lane_timeout_ms = 2000;
module_param(lane_timeout_ms, uint, 0400);
MODULE_PARM_DESC(lane_timeout_ms, "DEBUG: ordinary active-work deadline in ms (100..60000, default 2000)");

static struct m4_user_job *m4_user_lane_job_locked(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_job *job);
static void m4_user_fault_report(void *owner);
static int m4_user_recovery(void *owner, u32 step, struct m4_ready_recovery *rec);

static int m4_user_inflight2_admit_locked(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_lane_item *candidate);
static void m4_user_inflight2_observe_locked(struct m4_gpu_provider_runtime *p,
	struct m4_user_job *j);

/* The lane asks only while another item is in flight. Without inflight2
 * ordinary execution stays serial: every successor is vetoed whatever the
 * lane cap. With it, m4_user_inflight2_admit_locked() decides.
 */
static int m4_user_lane_admit_locked(void *owner,
	struct gpu_m4_drm_provider_lane_item *head, struct gpu_m4_drm_provider_lane_item *candidate)
{
	struct m4_gpu_provider_runtime *p = owner;

	(void)head;
	if (!p->user_inflight2) {
		p->user_inflight2_stats.veto[M4_USER_VETO_DISABLED]++;
		return -EBUSY;
	}
	return m4_user_inflight2_admit_locked(p, candidate);
}

/* REMOVE-DEBUG: same-VM class of a READY candidate behind the retiring head
 * (other queue). Read-only; an unresolvable owner is simply unclassified.
 */
static u32 m4_user_lane_classify_locked(void *owner,
	struct gpu_m4_drm_provider_lane_item *head, struct gpu_m4_drm_provider_lane_item *candidate)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_user_job *a = m4_user_lane_job_locked(p, head->job);
	struct m4_user_job *b = m4_user_lane_job_locked(p, candidate->job);

	if (IS_ERR(a) || IS_ERR(b) || !a->queue->vm || a->queue->vm != b->queue->vm)
		return 0;
	return GPU_M4_DRM_LANE_OVERLAP_VM;
}

static const struct gpu_m4_drm_provider_lane_policy m4_user_lane_policy = {
	.admit_locked = m4_user_lane_admit_locked,
	.classify_locked = m4_user_lane_classify_locked,
};

/* The banks of the admitted in-flight jobs (inflight2 bank choice). */
static void m4_user_inflight_banks_locked(struct m4_gpu_provider_runtime *p)
{
	p->user_bank_flight = 0;
	for (u32 i = 0; i < M4_USER_INFLIGHT_MAX; i++) {
		const struct m4_user_inflight *o = &p->user_inflight[i];
		const struct m4_user_job *j;

		if (!o->item || !o->job)
			continue;
		j = container_of(o->item, struct m4_user_job, item);
		if (j->bank_held && j->bank < M4_USER_BANK_MAX)
			p->user_bank_flight |= BIT(j->bank);
	}
}

/* Remove one admitted owner, keeping lane order; it stops being the starter. */
static void m4_user_inflight_remove_locked(struct m4_gpu_provider_runtime *p, u32 slot)
{
	/* REMOVE-DEBUG: a pair in flight ends with the first of its two. */
	if (p->user_overlap_start) {
		u64 d = ktime_get_ns() - p->user_overlap_start;

		p->user_inflight2_stats.overlap_ns += d;
		p->user_inflight2_stats.overlap_max = max(p->user_inflight2_stats.overlap_max, d);
		p->user_overlap_start = 0;
	}
	if (p->user_starting.item == p->user_inflight[slot].item &&
	    p->user_starting.job == p->user_inflight[slot].job)
		p->user_starting = (struct m4_user_inflight) { };
	for (u32 i = slot + 1; i < M4_USER_INFLIGHT_MAX; i++)
		p->user_inflight[i - 1] = p->user_inflight[i];
	p->user_inflight[M4_USER_INFLIGHT_MAX - 1] = (struct m4_user_inflight) { };
	m4_user_inflight_banks_locked(p);
}

/* Pre-RUN initialization, outside GPU/frontend/reservation serialization.
 * The lane retains the provider. No firmware state or frontend registration.
 */
static int m4_user_lane_init(struct m4_gpu_provider_runtime *p)
{
	struct gpu_m4_drm_provider_lane_policy policy;
	struct gpu_m4_drm_provider_lane *lane;
	int ret;

	if (!p || !p->ordinary || !p->provider)
		return -EINVAL;
	lockdep_assert_not_held(&p->hello->gpu->lock);
	if (p->user_lane)
		return -EALREADY;
	lane = gpu_m4_drm_provider_lane_create(p->provider, 4096,
		clamp(lane_timeout_ms, 100U, 60000U), 1);
	if (IS_ERR(lane))
		return PTR_ERR(lane);
	policy = m4_user_lane_policy;
	if (p->user_inflight2) {
		/* Two packets in flight; a successor's 2 s run from its publication. */
		policy.max_inflight = GPU_M4_DRM_LANE_MAX_INFLIGHT;
		policy.publication_deadline = true;
	}
	ret = gpu_m4_drm_provider_lane_set_policy(lane, p, &policy);
	if (ret) {
		WARN_ON(gpu_m4_drm_provider_lane_destroy(lane));
		return ret;
	}
	p->user_lane = lane;
	/* Never destroyed while the pinned READY/session owner can receive. */
	p->hello->fault_report_owner = p;
	p->hello->fault_report = m4_user_fault_report;
	p->hello->recovery = m4_user_recovery;
	smp_store_release(&p->hello->notify_lane, lane);
	return 0;
}

/* A view is borrowed through the caller's actual core ref, not a mutable
 * frontend lookup. This identity check does not grant publication/retirement.
 */
static struct m4_user_job *m4_user_lane_job_locked(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_job *job)
{
	struct gpu_m4_drm_provider_job_view view = { 0 };
	struct m4_user_job *j;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!job || !p->ordinary || !p->user_lane)
		return ERR_PTR(-EINVAL);
	gpu_m4_drm_provider_job_view(job, &view);
	j = view.runtime_job;
	if (!j || j->runtime != p || j->core_job != job || !j->queue ||
	    j->queue->runtime != p || view.runtime_queue != j->queue ||
	    j->item.lane != p->user_lane)
		return ERR_PTR(-EPROTO);
	return j;
}

/* Only the lane's post-CORE-success callback calls this. Bookkeeping retains
 * any allocator/last-owner receipt BEFORE the next ready job can start. No
 * final put, signal, wait or hardware operation belongs to this callback.
 */
static void m4_user_lane_retired_locked(void *owner,
	struct gpu_m4_drm_provider_lane_item *item, struct gpu_m4_drm_provider_job *job)
{
	struct m4_user_job *j = owner;
	struct m4_gpu_provider_runtime *p = j->runtime;

	lockdep_assert_held(&p->hello->gpu->lock);
	/* Retirement is head-only: the runtime head mirrors the lane head. */
	if (WARN_ON(item != &j->item || j->core_job != job ||
	    p->user_inflight[0].item != item || p->user_inflight[0].job != job || j->retired))
		return;
	j->retired = true;
	m4_user_inflight2_observe_locked(p, j); /* REMOVE-DEBUG */
	m4_user_job_retired_locked(p, j);
	m4_user_inflight_remove_locked(p, 0);
}

static int m4_user_lane_accept_locked(struct m4_gpu_provider_runtime *p,
	struct m4_user_job *j, struct gpu_m4_drm_provider_job *job, struct m4_user_queue *q)
{
	struct gpu_m4_drm_provider_job_view view = { 0 };
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!p->ordinary || !p->user_lane || !j || !job || !q || q->runtime != p ||
	    q->closed || j->item.lane || j->prepared || j->published || j->retired)
		return -EINVAL;
	ret = gpu_m4_drm_provider_job_check_accepting_locked(job, p->provider);
	if (ret)
		return ret;
	gpu_m4_drm_provider_job_view(job, &view);
	if (view.runtime_queue != q || view.runtime_job)
		return -EPERM;
	j->runtime = p;
	j->queue = q;
	j->core_job = job; /* Borrowed until enqueue transfers its actual job ref. */
	ret = gpu_m4_drm_provider_lane_item_init_locked(p->user_lane, &j->item, job, q);
	/* The queue's host level orders its packets in the shared lane. */
	if (!ret)
		ret = gpu_m4_drm_provider_lane_item_set_priority_locked(&j->item, q->priority);
	if (!ret)
		ret = gpu_m4_drm_provider_lane_item_set_retired_locked(&j->item, j,
			m4_user_lane_retired_locked);
	if (ret) {
		if (j->item.lane)
			WARN_ON(gpu_m4_drm_provider_lane_item_fini_locked(&j->item));
		j->core_job = NULL;
	}
	return ret;
}

static int m4_user_lane_check_locked(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_job *job)
{
	struct m4_user_job *j = m4_user_lane_job_locked(p, job);

	if (IS_ERR(j))
		return PTR_ERR(j);
	if (!m4_user_inflight_item_locked(p, &j->item, job))
		return M4_USER_WHY(-ESTALE, "lane check in-flight slot=%d starting=%d",
			m4_user_inflight_slot_locked(p, job), p->user_starting.job == job);
	return M4_USER_WHY(gpu_m4_drm_provider_lane_check_active_locked(p->user_lane, p->provider,
		&j->item, job), "lane active state=%u", j->item.state);
}

/* Called only after the real packet publisher has made this exact command's
 * producer reachable, with an explicit ordinal. A partial/error publication
 * never starts or renews an execution interval; an early-published successor
 * never renews the current head's budget.
 */
static int m4_user_lane_command_published_locked(struct m4_user_job *j, u32 index)
{
	int ret = m4_user_lane_check_locked(j->runtime, j->core_job);

	if (ret)
		return ret;
	if (index >= j->command_count || index >= ARRAY_SIZE(j->cmd) ||
	    !j->cmd[index].published || j->cmd[index].receipt || index + 1 != j->published_count)
		return M4_USER_WHY_LANE(j, -ESTALE, "publish runtime", index);
#ifdef CONFIG_DEBUG_FS
	gpu_m4_trace(GPU_M4_TRACE_PUBLISH, j->cmd[index].engine_mask,
		(u64)(uintptr_t)j->queue, (u64)(uintptr_t)&j->item, index, j->command_count);
#endif
	return M4_USER_WHY_LANE(j, gpu_m4_drm_provider_lane_command_published_locked(&j->item,
		j->core_job, index), "publish", index);
}

/* The caller has just returned zero from the exact head command's complete
 * actual hardware predicate, not from ring consumption, health or a pending
 * stamp. This records that receipt without retiring the batch's cache or
 * fence; an already published successor becomes the head only now.
 */
static int m4_user_lane_command_retired_locked(struct m4_user_job *j, u32 index)
{
	int ret = m4_user_lane_check_locked(j->runtime, j->core_job);

	if (ret)
		return ret;
	if (index >= j->command_count || index >= ARRAY_SIZE(j->cmd) ||
	    !j->cmd[index].published || j->cmd[index].receipt || index != j->completed_count)
		return M4_USER_WHY_LANE(j, -ESTALE, "retire runtime", index);
#ifdef CONFIG_DEBUG_FS
	gpu_m4_trace(GPU_M4_TRACE_RETIRE, 0, (u64)(uintptr_t)j->queue, (u64)(uintptr_t)&j->item,
		index, 0);
#endif
	return M4_USER_WHY_LANE(j, gpu_m4_drm_provider_lane_command_retired_locked(&j->item,
		j->core_job, index), "retire", index);
}

static int m4_user_lane_acquire_locked(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_job *job)
{
	struct m4_user_job *j = m4_user_lane_job_locked(p, job), *old;
	bool stale[M4_USER_INFLIGHT_MAX] = { };
	u32 slot;
	int ret;

	if (IS_ERR(j))
		return PTR_ERR(j);
	ret = gpu_m4_drm_provider_lane_check_active_locked(p->user_lane, p->provider,
		&j->item, job);
	if (ret)
		return ret;
	if (m4_user_inflight_item_locked(p, &j->item, job))
		return 0;
	for (u32 i = 0; i < M4_USER_INFLIGHT_MAX; i++) {
		struct m4_user_inflight *o = &p->user_inflight[i];

		if (!o->item && !o->job)
			continue;
		old = m4_user_lane_job_locked(p, o->job);
		if (IS_ERR(old) || o->item != &old->item)
			return -EBUSY;
		/* A co-owner the lane itself still holds in flight. */
		if (!gpu_m4_drm_provider_lane_check_active_locked(p->user_lane, p->provider,
				o->item, o->job))
			continue;
		/* The core starts another ready job BEFORE outside-lock completion
		 * puts. Replace only a definitely unpublished previous failure.
		 * Its transferred ref is still held by the lane's completion list.
		 */
		if (old->published || gpu_m4_drm_provider_job_uncertain_locked(old->core_job))
			return -EBUSY;
		stale[i] = true;
	}
	for (u32 i = M4_USER_INFLIGHT_MAX; i--; )
		if (stale[i])
			m4_user_inflight_remove_locked(p, i);
	for (slot = 0; slot < M4_USER_INFLIGHT_MAX && p->user_inflight[slot].job; slot++)
		;
	if (slot == M4_USER_INFLIGHT_MAX)
		return -EBUSY;
	p->user_inflight[slot] = (struct m4_user_inflight) { &j->item, job };
	p->user_starting = p->user_inflight[slot];
	m4_user_inflight_banks_locked(p);
	if (slot)
		p->user_overlap_start = ktime_get_ns() ?: 1; /* REMOVE-DEBUG */
	return 0;
}

/* Called by actual core release, not an error-fence disposal shortcut. The
 * root defers all backing disposal to its existing outside-lock collector.
 */
static int m4_user_lane_release_locked(struct m4_gpu_provider_runtime *p,
	struct m4_user_job *j)
{
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!j || j->runtime != p)
		return -EINVAL;
	if ((j->published && !j->retired) ||
	    (j->core_job && gpu_m4_drm_provider_job_uncertain_locked(j->core_job)))
		return -EBUSY;
	if (j->item.lane && !j->core_job)
		return -EINVAL;
	if (j->item.lane) {
		ret = gpu_m4_drm_provider_lane_item_fini_locked(&j->item);
		if (ret)
			return ret;
	}
	for (u32 i = M4_USER_INFLIGHT_MAX; i--; )
		if (p->user_inflight[i].item == &j->item && p->user_inflight[i].job == j->core_job)
			m4_user_inflight_remove_locked(p, i);
	j->core_job = NULL;
	return 0;
}

static void m4_user_lane_enqueue_locked(void *owner, struct gpu_m4_drm_provider_job *job)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_user_job *j = m4_user_lane_job_locked(p, job);

	/* Exactly ONE initialized ref was transferred by provider_run. The item
	 * was registered during acceptance; no extra qualifier baseline/get.
	 */
	if (WARN_ON(IS_ERR(j)))
		return; /* Internal invariant failure retains, never drops, that ref. */
	gpu_m4_drm_provider_lane_enqueue_locked(&j->item);
}

static void m4_user_lane_close_locked(void *owner, void *queue)
{
	struct m4_gpu_provider_runtime *p = owner;

	lockdep_assert_held(&p->hello->gpu->lock);
	gpu_m4_drm_provider_lane_close_queue_locked(p->user_lane, queue);
}

static void m4_user_lane_fault_locked(void *owner, struct gpu_m4_drm_provider_job *job,
	int error)
{
	struct m4_gpu_provider_runtime *p = owner;

	lockdep_assert_held(&p->hello->gpu->lock);
	(void)job;
	error = error < 0 ? error : -EIO;
	if (!p->error)
		p->error = error;
	gpu_m4_drm_provider_lane_fault_locked(p->user_lane, error);
}

static enum drm_gpu_sched_stat m4_user_lane_timedout(void *owner,
	struct gpu_m4_drm_provider_job *job, struct drm_sched_job *sched_job)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct mutex *lock = &p->hello->gpu->lock;
	struct m4_user_job *j;
	int ret;

	struct gpu_m4_drm_provider_job_view view = { 0 };
	struct gpu_m4_drm_provider_job *active = job;

	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	gpu_m4_drm_provider_job_view(job, &view);
	if (view.aggregate) {
		/* The public job is the CPU aggregate. Service its exact attached
		 * real child; NULL is healthy materialization/dependency/completion.
		 */
		active = gpu_m4_drm_provider_job_active_window_locked(job);
		ret = IS_ERR(active) ? PTR_ERR(active) : 0;
		if (IS_ERR_OR_NULL(active))
			active = NULL;
	}
	if (active) {
		j = m4_user_lane_job_locked(p, active);
		ret = IS_ERR(j) ? PTR_ERR(j) : j->retired ? 0 :
			gpu_m4_drm_provider_lane_service_timeout_locked(p->user_lane,
				p->provider, &j->item, active);
		if (ret)
			gpu_m4_drm_provider_job_fault_locked(active, ret);
	}
	/* An aggregate failure completes only the public fence; the actual child
	 * fault above, not the payload-free parent, owns global fail-stop.
	 */
	if (ret && active != job)
		gpu_m4_drm_provider_job_fault_locked(job, ret);
	mutex_unlock(lock);
	/* DRM rearms/reinserts this exact pending scheduler job on NO_HANG.
	 * Nothing here renews a command clock or signals an unfinished batch.
	 * The lane still enforces each actual command's two-second deadline.
	 */
	if (!ret)
		return DRM_GPU_SCHED_STAT_NO_HANG;
	/* No reset/retry: signal the committed error outside GPU/resv locks and
	 * stop with the guilty job, enabling actual scheduler timeout disposal.
	 * The lane retains every protected uncertain item independently.
	 */
	WARN_ON(gpu_m4_drm_provider_job_signal(job));
	drm_sched_stop(sched_job->sched, sched_job);
	return DRM_GPU_SCHED_STAT_ENODEV;
}

#endif
