// SPDX-License-Identifier: GPL-2.0-only
/* Dependency-ready global lane and independent full-retirement drain. */
#include <linux/atomic.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "gpu-m4-drm-provider-lane.h"

/* Production stays serial. A larger value only takes effect on a lane with a
 * runtime admission policy, which may still veto every successor.
 */
static unsigned int max_inflight = 1;
module_param(max_inflight, uint, 0400);
MODULE_PARM_DESC(max_inflight, "Ordinary lane jobs in flight (1 = serial, at most 2)");

enum gpu_m4_lane_state {
	GPU_M4_LANE_PRIVATE,
	GPU_M4_LANE_READY,
	GPU_M4_LANE_ACTIVE,
	GPU_M4_LANE_DONE,
	GPU_M4_LANE_QUARANTINED,
};

struct gpu_m4_drm_provider_lane {
	struct gpu_m4_drm_provider *provider;
	struct delayed_work work;
	/* One-shot completion fallback while ACTIVE; only queues immediate work. */
	struct hrtimer poll_timer;
	/* Short trailing polls left before the fallback interval (reset on
	 * progress or a newer notification).
	 */
	u32 trailing;
	int last_seq;
	atomic_t notify_seq;
	atomic64_t timer_wakes;
	struct gpu_m4_drm_provider_lane_stats stats; /* REMOVE-DEBUG, under mutex */
	struct list_head owners, ready, completed;
	/* Admission order: [0] is the head, the only serviced/timed/retired
	 * item; [1] is a successor, promoted only after [0] completes. deadline
	 * is always the head's.
	 */
	struct gpu_m4_drm_provider_lane_item *inflight[GPU_M4_DRM_LANE_MAX_INFLIGHT];
	struct gpu_m4_drm_provider_lane_policy policy;
	void *policy_owner;
	u64 deadline;
	/* Worker hold token: advanced when the worker takes the mutex and again
	 * before it drops it, so it is odd exactly while the worker itself holds
	 * it. An unchanged odd token proves one continuous worker hold.
	 */
	u64 hold;
	u32 inflight_count, max_inflight;
	u32 count, max_items, timeout_ms, poll_ms;
	int error;
	bool closing;
};

static struct mutex *gpu_m4_lane_lock(struct gpu_m4_drm_provider_lane *lane)
{
	return gpu_m4_drm_provider_lock(lane->provider);
}

static int gpu_m4_lane_item_member_locked(struct gpu_m4_drm_provider_lane *lane,
	struct gpu_m4_drm_provider_lane_item *wanted)
{
	struct gpu_m4_drm_provider_lane_item *item;
	u32 visited = 0;
	bool found = false;

	lockdep_assert_held(gpu_m4_lane_lock(lane));
	if (!lane->max_items || lane->max_items > 4096 || lane->count > lane->max_items)
		return -EPROTO;
	list_for_each_entry(item, &lane->owners, owner_link) {
		if (++visited > lane->max_items)
			return -EPROTO;
		if (item == wanted)
			found = true;
	}
	if (visited != lane->count)
		return -EPROTO;
	return found ? 0 : -ENOENT;
}

/* Exact in-flight slot of an item, or -1. Pointer comparison only. */
static int gpu_m4_lane_inflight_slot(const struct gpu_m4_drm_provider_lane *lane,
	const struct gpu_m4_drm_provider_lane_item *item)
{
	for (u32 i = 0; i < lane->inflight_count && i < GPU_M4_DRM_LANE_MAX_INFLIGHT; i++)
		if (lane->inflight[i] == item)
			return i;
	return -1;
}

int gpu_m4_drm_provider_lane_check_active_locked(
	struct gpu_m4_drm_provider_lane *lane, struct gpu_m4_drm_provider *expected_provider,
	struct gpu_m4_drm_provider_lane_item *item, struct gpu_m4_drm_provider_job *job)
{
	int ret;

	if (IS_ERR_OR_NULL(lane) || IS_ERR_OR_NULL(expected_provider) ||
	    IS_ERR_OR_NULL(item) || IS_ERR_OR_NULL(job))
		return -EINVAL;
	lockdep_assert_held(gpu_m4_drm_provider_lock(expected_provider));
	if (lane->provider != expected_provider)
		return -EPERM;
	/* Do not dereference a caller-supplied item until actual owned membership.
	 * The matched item's transferred job reference retains the exact job.
	 */
	ret = gpu_m4_lane_item_member_locked(lane, item);
	if (ret)
		return ret;
	if (gpu_m4_lane_inflight_slot(lane, item) < 0 || item->lane != lane || item->job != job ||
	    item->state != GPU_M4_LANE_ACTIVE)
		return -ESTALE;
	ret = gpu_m4_drm_provider_job_check_owner_locked(job, expected_provider);
	if (ret)
		return ret;
	if (lane->error)
		return lane->error < 0 ? lane->error : -EIO;
	return lane->closing ? -ESHUTDOWN : 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_check_active_locked);

static int gpu_m4_lane_timeout_check_locked(
	struct gpu_m4_drm_provider_lane *lane, struct gpu_m4_drm_provider *expected_provider,
	struct gpu_m4_drm_provider_lane_item *item, struct gpu_m4_drm_provider_job *job)
{
	const struct gpu_m4_drm_provider_lane_timing *t;
	struct gpu_m4_drm_provider_lane_item *queued, *head;
	u64 now, budget;
	u32 visited = 0;
	bool found = false;
	int ret, slot;

	if (IS_ERR_OR_NULL(lane) || IS_ERR_OR_NULL(expected_provider) ||
	    IS_ERR_OR_NULL(item) || IS_ERR_OR_NULL(job))
		return -EINVAL;
	lockdep_assert_held(gpu_m4_drm_provider_lock(expected_provider));
	if (lane->provider != expected_provider)
		return -EPERM;
	ret = gpu_m4_lane_item_member_locked(lane, item);
	if (ret)
		return ret;
	if (item->lane != lane || item->job != job)
		return -ESTALE;
	ret = gpu_m4_drm_provider_job_check_owner_locked(job, expected_provider);
	if (ret)
		return ret;
	if (lane->error)
		return lane->error < 0 ? lane->error : -EIO;
	if (lane->closing)
		return -ESHUTDOWN;
	if (item->state != GPU_M4_LANE_READY && item->state != GPU_M4_LANE_ACTIVE)
		return -ESTALE;
	if (item->command_count > GPU_M4_DRM_JOB_MAX_COMMANDS)
		return -EPROTO;
	t = &item->timing;
	slot = gpu_m4_lane_inflight_slot(lane, item);
	if (item->state == GPU_M4_LANE_READY) {
		if (slot >= 0)
			return -ESTALE;
		if (t->activated_ns || t->published_ns || t->deadline_ns || t->retired_ns ||
		    t->published_index || t->retired_index || t->published || t->retired ||
		    t->published_count || t->retired_count || t->head_ns)
			return -EPROTO;
		/* A lane-state number or !uncertain is not actual queue membership.
		 * Inspect the bounded retained FIFO before granting this exemption.
		 */
		list_for_each_entry(queued, &lane->ready, ready_link) {
			if (++visited > lane->count || visited > lane->max_items ||
			    queued->lane != lane || queued->state != GPU_M4_LANE_READY || !queued->job)
				return -EPROTO;
			if (queued == item)
				found = true;
		}
		if (!found)
			return -ESTALE;
		ret = gpu_m4_drm_provider_job_check_execution_locked(job, expected_provider, false);
		if (ret)
			return ret;
		if (item->closed)
			return -ECANCELED;
		return item->command_count ? 0 : -ETIMEDOUT;
	}
	if (slot > 0) {
		/* An admitted successor executes only behind the head. While the
		 * head is in flight, ITS deadline alone decides (NO_HANG here); a
		 * head fault or timeout faults this successor through the lane.
		 */
		head = lane->inflight[0];
		if (item->state != GPU_M4_LANE_ACTIVE || !list_empty(&item->ready_link) ||
		    !head || head->lane != lane || head->state != GPU_M4_LANE_ACTIVE ||
		    !list_empty(&head->ready_link))
			return -EPROTO;
		return gpu_m4_drm_provider_job_check_execution_locked(job, expected_provider, true);
	}
	if (!item->command_count)
		return -ETIMEDOUT; /* Preserve the original single-job timeout policy. */
	if (item->state != GPU_M4_LANE_ACTIVE || slot ||
	    !list_empty(&item->ready_link))
		return -ESTALE;
	ret = gpu_m4_drm_provider_job_check_execution_locked(job, expected_provider, true);
	if (ret)
		return ret;
	budget = (u64)lane->timeout_ms * NSEC_PER_MSEC;
	/* Only the unreceipted HEAD's own budget decides a timeout. */
	if (!budget || lane->timeout_ms > 60000 || !t->published || t->retired ||
	    !item->command_depth || !t->published_count ||
	    t->published_count > item->command_count ||
	    t->retired_count >= t->published_count ||
	    t->published_count - t->retired_count > item->command_depth ||
	    t->published_index != t->published_count - 1 ||
	    t->published_ns < t->activated_ns || t->head_ns < t->activated_ns ||
	    t->head_ns < t->command_ns[t->retired_count] ||
	    t->head_ns > ~0ULL - budget || t->deadline_ns != t->head_ns + budget ||
	    lane->deadline != t->deadline_ns ||
	    (t->retired_count ? (t->retired_index != t->retired_count - 1 ||
				t->retired_ns < t->activated_ns || t->retired_ns > t->head_ns) :
			       (t->retired_index || t->retired_ns)))
		return -EPROTO;
	now = ktime_get_ns();
	if (now < t->head_ns)
		return -EPROTO;
	return now < t->deadline_ns ? 0 : -ETIMEDOUT;
}

int gpu_m4_drm_provider_lane_item_set_commands_locked(
	struct gpu_m4_drm_provider_lane_item *item, u32 count)
{
	struct gpu_m4_drm_provider_lane *lane;
	int ret;

	if (IS_ERR_OR_NULL(item) || IS_ERR_OR_NULL(item->lane) ||
	    !count || count > GPU_M4_DRM_JOB_MAX_COMMANDS)
		return -EINVAL;
	lane = item->lane;
	lockdep_assert_held(gpu_m4_lane_lock(lane));
	ret = gpu_m4_lane_item_member_locked(lane, item);
	if (ret)
		return ret;
	if (item->state != GPU_M4_LANE_PRIVATE || item->command_count)
		return -EBUSY;
	ret = gpu_m4_drm_provider_job_check_owner_locked(item->job, lane->provider);
	if (ret)
		return ret;
	item->command_count = count;
	item->command_depth = 1;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_item_set_commands_locked);

/* Bounded publication depth, while PRIVATE and after the exact command count.
 * It never exceeds the count or one bounded execution window.
 */
int gpu_m4_drm_provider_lane_item_set_depth_locked(
	struct gpu_m4_drm_provider_lane_item *item, u32 depth)
{
	struct gpu_m4_drm_provider_lane *lane;
	int ret;

	if (IS_ERR_OR_NULL(item) || IS_ERR_OR_NULL(item->lane) || !depth ||
	    depth > GPU_M4_DRM_PROVIDER_WINDOW_MAX_COMMANDS)
		return -EINVAL;
	lane = item->lane;
	lockdep_assert_held(gpu_m4_lane_lock(lane));
	ret = gpu_m4_lane_item_member_locked(lane, item);
	if (ret)
		return ret;
	if (item->state != GPU_M4_LANE_PRIVATE || !item->command_count ||
	    item->command_depth != 1 || depth > item->command_count)
		return -EBUSY;
	ret = gpu_m4_drm_provider_job_check_owner_locked(item->job, lane->provider);
	if (ret)
		return ret;
	item->command_depth = depth;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_item_set_depth_locked);

static int gpu_m4_lane_command_check_locked(struct gpu_m4_drm_provider_lane_item *item,
	struct gpu_m4_drm_provider_job *job, u32 index)
{
	struct gpu_m4_drm_provider_lane *lane;
	int ret;

	if (IS_ERR_OR_NULL(item) || IS_ERR_OR_NULL(item->lane))
		return -EINVAL;
	lane = item->lane;
	ret = gpu_m4_drm_provider_lane_check_active_locked(lane, lane->provider, item, job);
	if (ret)
		return ret;
	if (!item->command_count || index >= item->command_count ||
	    !gpu_m4_drm_provider_job_uncertain_locked(job))
		return -ESTALE;
	return 0;
}

int gpu_m4_drm_provider_lane_command_published_locked(
	struct gpu_m4_drm_provider_lane_item *item,
	struct gpu_m4_drm_provider_job *job, u32 index)
{
	struct gpu_m4_drm_provider_lane_timing *t;
	struct gpu_m4_drm_provider_lane_stats *s;
	u64 now, budget;
	int ret = gpu_m4_lane_command_check_locked(item, job, index);

	if (ret)
		return ret;
	t = &item->timing;
	if (!item->command_depth || index != t->published_count ||
	    t->published_count >= item->command_count ||
	    t->retired_count > t->published_count ||
	    t->published_count - t->retired_count >= item->command_depth ||
	    t->published != !!t->published_count)
		return -ESTALE;
	now = ktime_get_ns();
	budget = (u64)item->lane->timeout_ms * NSEC_PER_MSEC;
	if (now > ~0ULL - budget)
		return -EOVERFLOW;
	s = &item->lane->stats;
	if (!t->published_count) {
		s->start_ns += now - t->activated_ns;
		s->start_max = max(s->start_max, now - t->activated_ns);
	} else if (t->published_count == t->retired_count) {
		s->gaps++; s->gap_ns += now - t->retired_ns;
		s->gap_max = max(s->gap_max, now - t->retired_ns);
	} else {
		s->ahead++;
	}
	t->command_ns[index] = now;
	if (t->published_count == t->retired_count) {
		/* Nothing outstanding: this command is the head from now on. An
		 * in-flight successor's clock becomes the lane's only on promotion.
		 */
		t->head_ns = now;
		t->deadline_ns = now + budget;
		if (item->lane->inflight[0] == item)
			item->lane->deadline = t->deadline_ns;
	}
	t->published_count++;
	t->published = true;
	t->published_index = index;
	t->published_ns = now;
	t->retired = false;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_command_published_locked);

int gpu_m4_drm_provider_lane_command_retired_locked(
	struct gpu_m4_drm_provider_lane_item *item,
	struct gpu_m4_drm_provider_job *job, u32 index)
{
	struct gpu_m4_drm_provider_lane_timing *t;
	struct gpu_m4_drm_provider_lane_stats *s;
	u64 now, budget;
	int ret = gpu_m4_lane_command_check_locked(item, job, index);

	if (ret)
		return ret;
	t = &item->timing;
	/* Only the lane head carries a receipt; a successor waits behind it. */
	if (item->lane->inflight[0] != item || !t->published || t->retired ||
	    index != t->retired_count || index >= t->published_count ||
	    t->published_count > item->command_count)
		return -ESTALE;
	now = ktime_get_ns();
	budget = (u64)item->lane->timeout_ms * NSEC_PER_MSEC;
	if (now < t->head_ns || now > ~0ULL - budget)
		return -EPROTO;
	s = &item->lane->stats;
	s->commands++; s->command_ns += now - t->head_ns;
	s->command_max = max(s->command_max, now - t->head_ns);
	t->retired_index = index;
	t->retired_ns = now;
	t->retired_count++;
	t->retired = t->retired_count == t->published_count;
	if (!t->retired) {
		/* The already published successor executes only after this genuine
		 * ordered receipt. With a publication-deadline policy its budget is
		 * its own publication's (never renewed, like a promoted job's);
		 * otherwise it starts now, never at the predecessor's.
		 */
		t->head_ns = item->lane->policy.publication_deadline ?
			t->command_ns[t->retired_count] :
			max(t->command_ns[t->retired_count], now);
		t->deadline_ns = t->head_ns + budget;
		item->lane->deadline = t->deadline_ns;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_command_retired_locked);

int gpu_m4_drm_provider_lane_item_timing_locked(
	struct gpu_m4_drm_provider_lane_item *item,
	struct gpu_m4_drm_provider_job *job,
	struct gpu_m4_drm_provider_lane_timing *timing)
{
	struct gpu_m4_drm_provider_lane *lane;
	int ret;

	if (IS_ERR_OR_NULL(item) || IS_ERR_OR_NULL(item->lane) ||
	    IS_ERR_OR_NULL(job) || !timing)
		return -EINVAL;
	lane = item->lane;
	lockdep_assert_held(gpu_m4_lane_lock(lane));
	ret = gpu_m4_lane_item_member_locked(lane, item);
	if (ret)
		return ret;
	if (item->job != job)
		return -ESTALE;
	*timing = item->timing;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_item_timing_locked);

int gpu_m4_drm_provider_lane_item_set_retired_locked(
	struct gpu_m4_drm_provider_lane_item *item, void *owner,
	void (*retired_locked)(void *owner, struct gpu_m4_drm_provider_lane_item *item,
			       struct gpu_m4_drm_provider_job *job))
{
	struct gpu_m4_drm_provider_lane *lane;
	int ret;

	if (IS_ERR_OR_NULL(item) || IS_ERR_OR_NULL(item->lane) ||
	    (owner && IS_ERR(owner)) || !!owner != !!retired_locked)
		return -EINVAL;
	lane = item->lane;
	lockdep_assert_held(gpu_m4_lane_lock(lane));
	ret = gpu_m4_lane_item_member_locked(lane, item);
	if (ret)
		return ret;
	if (item->state != GPU_M4_LANE_PRIVATE || item->retired_owner || item->retired_locked ||
	    !list_empty(&item->ready_link) || !list_empty(&item->completion_link))
		return -EBUSY;
	ret = gpu_m4_drm_provider_job_check_owner_locked(item->job, lane->provider);
	if (ret)
		return ret;
	if (lane->error)
		return lane->error < 0 ? lane->error : -EIO;
	if (lane->closing || item->closed)
		return -ECANCELED;
	item->retired_owner = owner;
	item->retired_locked = retired_locked;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_item_set_retired_locked);

/* Completion latency: the lane worker runs on the high-priority pool (a
 * quick wake after a firmware notification or the poll timer), and the poll
 * that stands in for a missing notification is lane_poll_us.
 */
static bool lane_highpri = true;
module_param(lane_highpri, bool, 0644);
MODULE_PARM_DESC(lane_highpri, "Run the lane worker on the high-priority workqueue (default 1)");
static unsigned int lane_poll_us = 100;
module_param(lane_poll_us, uint, 0644);
MODULE_PARM_DESC(lane_poll_us, "Lane progress poll while work is pending, in microseconds (default 100)");

static unsigned int lane_trailing_polls = 4;
module_param(lane_trailing_polls, uint, 0644);
MODULE_PARM_DESC(lane_trailing_polls, "Polls 20 us apart after progress before the lane_poll_us fallback (default 4)");

static struct workqueue_struct *gpu_m4_lane_wq(void)
{
	return READ_ONCE(lane_highpri) ? system_highpri_wq : system_unbound_wq;
}

static void gpu_m4_lane_schedule(struct gpu_m4_drm_provider_lane *lane, u32 delay)
{
	mod_delayed_work(gpu_m4_lane_wq(), &lane->work, msecs_to_jiffies(delay));
}

/* Timer expiry is only a request to inspect progress, never completion. */
static enum hrtimer_restart gpu_m4_lane_poll_timer(struct hrtimer *timer)
{
	struct gpu_m4_drm_provider_lane *lane =
		container_of(timer, struct gpu_m4_drm_provider_lane, poll_timer);

	/* A destroyed lane's stop state is visible to timer/notify entrances. */
	atomic64_inc(&lane->timer_wakes);
	if (!READ_ONCE(lane->closing))
		mod_delayed_work(gpu_m4_lane_wq(), &lane->work, 0);
	return HRTIMER_NORESTART;
}

void gpu_m4_drm_provider_lane_stats_locked(struct gpu_m4_drm_provider_lane *lane,
	struct gpu_m4_drm_provider_lane_stats *out)
{
	lockdep_assert_held(gpu_m4_lane_lock(lane));
	*out = lane->stats;
	out->timer_wakes = atomic64_read(&lane->timer_wakes);
	out->notify_wakes = atomic_read(&lane->notify_seq);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_stats_locked);

void gpu_m4_drm_provider_lane_notify(struct gpu_m4_drm_provider_lane *lane)
{
	if (!lane || READ_ONCE(lane->closing))
		return;
	/* Coalesced firmware ring activity: an immediate wake always wins. */
	atomic_inc(&lane->notify_seq);
	mod_delayed_work(gpu_m4_lane_wq(), &lane->work, 0);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_notify);

static void gpu_m4_lane_complete_locked(struct gpu_m4_drm_provider_lane_item *item,
				       struct list_head *completed)
{
	lockdep_assert_held(gpu_m4_lane_lock(item->lane));
	list_del_init(&item->ready_link);
	item->state = gpu_m4_drm_provider_job_uncertain_locked(item->job) ?
		GPU_M4_LANE_QUARANTINED : GPU_M4_LANE_DONE;
	list_add_tail(&item->completion_link, completed);
}

/* Remove the completed head. After a success an admitted successor becomes
 * the new head: its budget starts at max(its own head command's publication,
 * now), never at a publication made while it waited behind the predecessor.
 * Without a successor the last deadline is left as it was.
 */
static struct gpu_m4_drm_provider_lane_item *gpu_m4_lane_pop_head_locked(
	struct gpu_m4_drm_provider_lane *lane, bool promote)
{
	struct gpu_m4_drm_provider_lane_item *head = lane->inflight[0], *next;
	struct gpu_m4_drm_provider_lane_timing *t;
	u64 now, budget;

	lockdep_assert_held(gpu_m4_lane_lock(lane));
	for (u32 i = 1; i < GPU_M4_DRM_LANE_MAX_INFLIGHT; i++)
		lane->inflight[i - 1] = lane->inflight[i];
	lane->inflight[GPU_M4_DRM_LANE_MAX_INFLIGHT - 1] = NULL;
	lane->inflight_count--;
	next = lane->inflight[0];
	if (!next || !promote)
		return head;
	lane->stats.promotions++;
	t = &next->timing;
	now = ktime_get_ns();
	budget = (u64)lane->timeout_ms * NSEC_PER_MSEC;
	if (!next->command_count) {
		t->deadline_ns = now + budget;
	} else if (t->published && t->retired_count < t->published_count &&
		   t->retired_count < GPU_M4_DRM_JOB_MAX_COMMANDS) {
		/* Publication deadline: the successor's clock started when its
		 * head command became reachable, waiting behind its strict
		 * firmware predecessor included; promotion never renews it.
		 */
		t->head_ns = lane->policy.publication_deadline ?
			t->command_ns[t->retired_count] :
			max(t->command_ns[t->retired_count], now);
		t->deadline_ns = t->head_ns + budget;
	}
	lane->deadline = t->deadline_ns;
	return head;
}

/* REMOVE-DEBUG: classify the READY FIFO's first candidate against the head
 * that is retiring now. The next activation counts it; metadata only.
 */
static void gpu_m4_lane_observe_locked(struct gpu_m4_drm_provider_lane *lane,
	struct gpu_m4_drm_provider_lane_item *head)
{
	struct gpu_m4_drm_provider_lane_item *next;

	if (list_empty(&lane->ready))
		return;
	next = list_first_entry(&lane->ready, struct gpu_m4_drm_provider_lane_item, ready_link);
	next->overlap = GPU_M4_DRM_LANE_OVERLAP_SEEN;
	if (next->queue == head->queue)
		next->overlap |= GPU_M4_DRM_LANE_OVERLAP_QUEUE | GPU_M4_DRM_LANE_OVERLAP_VM;
	else if (lane->policy.classify_locked)
		next->overlap |= lane->policy.classify_locked(lane->policy_owner, head, next) &
			GPU_M4_DRM_LANE_OVERLAP_VM;
}

/* The worker and TDR observer share the real CORE retirement authority. A slow
 * first validation that crosses a deadline must take a fresh deadline-side
 * predicate; it must not mistake an already completed command for a hang.
 * A successor may be published inside retire_locked: use its NEW deadline.
 * Only the head is serviced. Its failure completes every in-flight item, the
 * head first, each error-signalled exactly once by the drain.
 */
static int gpu_m4_lane_service_active_locked(struct gpu_m4_drm_provider_lane *lane)
{
	struct gpu_m4_drm_provider_lane_item *item;
	int ret;

	lockdep_assert_held(gpu_m4_lane_lock(lane));
	if (!lane->error)
		lane->error = gpu_m4_drm_provider_error_locked(lane->provider);
	if (!lane->inflight[0])
		return 0;
	item = lane->inflight[0];
	if (lane->error) {
		gpu_m4_drm_provider_job_fault_locked(item->job, lane->error);
		ret = lane->error;
	} else {
		ret = gpu_m4_drm_provider_lane_check_active_locked(lane, lane->provider, item, item->job);
		if (!ret)
			ret = gpu_m4_drm_provider_job_check_execution_locked(item->job, lane->provider, true);
		if (ret) {
			gpu_m4_drm_provider_job_fault_locked(item->job, ret);
		} else {
			ret = gpu_m4_drm_provider_job_retire_locked(item->job);
			lane->stats.polls++;
			lane->stats.pending_polls += ret == -EAGAIN;
			if (ret == -EAGAIN && lane->deadline && ktime_get_ns() >= lane->deadline) {
				ret = gpu_m4_drm_provider_job_retire_locked(item->job);
				if (ret == -EAGAIN && lane->deadline && ktime_get_ns() >= lane->deadline) {
					ret = -ETIMEDOUT;
					gpu_m4_drm_provider_job_fault_locked(item->job, ret);
				}
			}
		}
	}
	if (ret != -EAGAIN) {
		/* Success is already committed, while the exact item remains ACTIVE.
		 * This callback only transfers retained completion bookkeeping.
		 */
#ifdef CONFIG_DEBUG_FS
		gpu_m4_trace(GPU_M4_TRACE_DONE, 0, (u64)(uintptr_t)item->queue,
			(u64)(uintptr_t)item, ret, 0);
#endif
		if (!ret)
			gpu_m4_lane_observe_locked(lane, item);
		if (!ret && item->retired_locked)
			item->retired_locked(item->retired_owner, item, item->job);
		gpu_m4_lane_pop_head_locked(lane, !ret);
		if (ret && !lane->error)
			lane->error = ret;
		gpu_m4_lane_complete_locked(item, &lane->completed);
		/* Fault-all: a successor published behind a failed or timed-out
		 * head is uncertain too. Quarantine it in admission order.
		 */
		while (ret && lane->inflight[0]) {
			item = gpu_m4_lane_pop_head_locked(lane, false);
			gpu_m4_drm_provider_job_fault_locked(item->job, lane->error);
			gpu_m4_lane_complete_locked(item, &lane->completed);
		}
	}
	return ret;
}

int gpu_m4_drm_provider_lane_service_timeout_locked(
	struct gpu_m4_drm_provider_lane *lane, struct gpu_m4_drm_provider *expected_provider,
	struct gpu_m4_drm_provider_lane_item *item, struct gpu_m4_drm_provider_job *job)
{
	bool requested_active;
	int ret = gpu_m4_lane_timeout_check_locked(lane, expected_provider, item, job);

	if (ret == -ECANCELED) {
		/* The classifier proved exact FIFO membership and accepted READY,
		 * unprotected CORE ownership. Preserve the worker's local close path:
		 * an error fence, never success/retirement or global GPU failure.
		 */
		ret = gpu_m4_drm_provider_job_cancel_locked(job);
		if (ret)
			return ret;
		gpu_m4_lane_complete_locked(item, &lane->completed);
		gpu_m4_lane_schedule(lane, 0);
		return 0;
	}
	if (ret && ret != -ETIMEDOUT)
		return ret;
	/* The original non-command qualifier and an already latched failure must
	 * not acquire a healthy-progress exemption from this serial service API.
	 */
	if (!item->command_count || lane->error ||
	    gpu_m4_drm_provider_error_locked(expected_provider))
		return ret ?: (lane->error ?: gpu_m4_drm_provider_error_locked(expected_provider));
	/* A successor's TDR services the head on its behalf: only the head can
	 * retire, and its failure is the successor's too.
	 */
	requested_active = lane->inflight[0] == item;
	ret = gpu_m4_lane_service_active_locked(lane);
	/* The worker alone starts another item and drains shared completions. A
	 * queued transferred reference survives this return and all signal reentry.
	 */
	gpu_m4_lane_schedule(lane, 0);
	if (ret && ret != -EAGAIN)
		return ret;
	if (!ret && requested_active)
		return 0; /* Real CORE/cache success and its post-success callback. */
	return gpu_m4_lane_timeout_check_locked(lane, expected_provider, item, job);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_service_timeout_locked);

/* Outside the mutex: signal every completed item and drop its transferred
 * ref. dma-fence callbacks may acquire the provider lock or release other
 * queues; the transferred runtime reference protects each job here.
 */
static void gpu_m4_lane_drain_completed(struct gpu_m4_drm_provider_lane *lane)
{
	struct gpu_m4_drm_provider_lane_item *item;
	struct mutex *lock = gpu_m4_lane_lock(lane);

	for (;;) {
		struct gpu_m4_drm_provider_job *job;
		bool retained;

		mutex_lock(lock);
		if (list_empty(&lane->completed)) {
			mutex_unlock(lock);
			break;
		}
		item = list_first_entry(&lane->completed,
				       struct gpu_m4_drm_provider_lane_item, completion_link);
		list_del_init(&item->completion_link);
		job = item->job;
		retained = item->state == GPU_M4_LANE_QUARANTINED;
		if (!retained)
			item->job = NULL;
		mutex_unlock(lock);
		WARN_ON(gpu_m4_drm_provider_job_signal(job));
		if (!retained)
			WARN_ON(gpu_m4_drm_provider_job_put(job));
		/* A final put can free the embedded item. Do not touch it afterward.
		 * Uncertain items retain their transferred ref on lane->owners.
		 */
	}
}

/* The worker's ONLY mutex acquisition and release: they also advance the hold
 * token (see gpu_m4_drm_provider_lane_hold_locked()). REMOVE-DEBUG: the
 * worker's own mutex wait and hold times, observational only.
 */
static u64 gpu_m4_lane_lock_timed(struct gpu_m4_drm_provider_lane *lane)
{
	u64 start = ktime_get_ns(), now;

	mutex_lock(gpu_m4_lane_lock(lane));
	lane->hold++;
	now = ktime_get_ns();
	lane->stats.waits++;
	lane->stats.wait_ns += now - start;
	lane->stats.wait_max = max(lane->stats.wait_max, now - start);
	return now;
}

static void gpu_m4_lane_unlock_timed(struct gpu_m4_drm_provider_lane *lane, u64 locked,
	bool started)
{
	u64 held = ktime_get_ns() - locked;

	lane->stats.holds++;
	lane->stats.hold_ns += held;
	lane->stats.hold_max = max(lane->stats.hold_max, held);
	if (started) {
		lane->stats.start_holds++;
		lane->stats.start_hold_ns += held;
		lane->stats.start_hold_max = max(lane->stats.start_hold_max, held);
	}
	lane->hold++;
	mutex_unlock(gpu_m4_lane_lock(lane));
}

u64 gpu_m4_drm_provider_lane_hold_locked(struct gpu_m4_drm_provider_lane *lane)
{
	if (!lane)
		return 0;
	lockdep_assert_held(gpu_m4_lane_lock(lane));
	return lane->hold & 1 ? lane->hold : 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_hold_locked);

int gpu_m4_drm_provider_lane_error_locked(struct gpu_m4_drm_provider_lane *lane)
{
	if (!lane)
		return -EINVAL;
	lockdep_assert_held(gpu_m4_lane_lock(lane));
	if (lane->error)
		return lane->error < 0 ? lane->error : -EIO;
	return lane->closing ? -ESHUTDOWN : 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_error_locked);

/* Per-queue order: a successor may start only once every earlier accepted
 * item of its queue has left PRIVATE/READY (activated or completed). Bounded
 * by the owner count; closed items never block.
 */
static bool gpu_m4_lane_queue_ordered_locked(struct gpu_m4_drm_provider_lane *lane,
	struct gpu_m4_drm_provider_lane_item *candidate)
{
	struct gpu_m4_drm_provider_lane_item *item;
	u32 visited = 0;

	list_for_each_entry(item, &lane->owners, owner_link) {
		if (item == candidate)
			return true;
		if (++visited > lane->count)
			return false;
		if (item->queue == candidate->queue && !item->closed &&
		    (item->state == GPU_M4_LANE_PRIVATE || item->state == GPU_M4_LANE_READY))
			return false;
	}
	return false;
}

/* Admission beyond the head: the runtime policy plus the per-queue order
 * gate. The head itself keeps the plain dependency-ready FIFO.
 */
static bool gpu_m4_lane_admit_locked(struct gpu_m4_drm_provider_lane *lane,
	struct gpu_m4_drm_provider_lane_item *candidate)
{
	if (!lane->inflight_count)
		return true;
	if (!lane->policy.admit_locked || !gpu_m4_lane_queue_ordered_locked(lane, candidate))
		return false;
	return !lane->policy.admit_locked(lane->policy_owner, lane->inflight[0], candidate);
}

/* Observable lane progress: completed jobs plus each in-flight item's
 * published and retired command counts.
 */
static u64 gpu_m4_lane_progress(const struct gpu_m4_drm_provider_lane *lane)
{
	u64 token = lane->stats.jobs << 16;

	for (u32 i = 0; i < GPU_M4_DRM_LANE_MAX_INFLIGHT; i++)
		if (lane->inflight[i])
			token += lane->inflight[i]->timing.published_count +
				lane->inflight[i]->timing.retired_count;
	return token;
}

static void gpu_m4_lane_work(struct work_struct *work)
{
	struct gpu_m4_drm_provider_lane *lane = container_of(to_delayed_work(work),
		struct gpu_m4_drm_provider_lane, work);
	struct gpu_m4_drm_provider_lane_item *item;
	int seq = atomic_read(&lane->notify_seq);
	bool pending, started = false;
	u64 locked, progress;
	u32 poll_us;
	int ret;

	locked = gpu_m4_lane_lock_timed(lane);
	progress = gpu_m4_lane_progress(lane);
	/* A real command receipt can publish its successor inside retire_locked.
	 * Resnapshot that successor immediately, within a bounded budget, rather
	 * than waiting for another fallback interval.
	 */
	for (u32 budget = 0; budget < 8; budget++) {
		struct gpu_m4_drm_provider_lane_item *head = lane->inflight[0];
		u32 published = head ? head->timing.published_count : 0;
		u32 retired = head ? head->timing.retired_count : 0;

		gpu_m4_lane_service_active_locked(lane);
		/* Completed-prefix/head progress counts even when the furthest
		 * published index and its time did not change. A promoted
		 * successor is a new head: service it in the same pass.
		 */
		if (!head || !lane->inflight[0] ||
		    (lane->inflight[0] == head && head->timing.published_count == published &&
		     head->timing.retired_count == retired))
			break;
	}
	for (;;) {
		/* A completed item's fence never waits for a successor's activation
		 * and packet build, which hold this mutex for a whole start: neither
		 * a retired head nor an item this loop just cancelled or failed
		 * locally. Completed items are already final: before selecting ANY
		 * next item, signal them outside the mutex (exactly the drain
		 * below), retake it and re-evaluate every condition. No item is
		 * held across the unlock; the next one is selected afresh.
		 */
		if (lane->inflight_count < lane->max_inflight && !list_empty(&lane->ready) &&
		    !list_empty(&lane->completed)) {
			gpu_m4_lane_unlock_timed(lane, locked, started);
			gpu_m4_lane_drain_completed(lane);
			locked = gpu_m4_lane_lock_timed(lane);
			started = false;
			continue;
		}
		if (lane->inflight_count >= lane->max_inflight || list_empty(&lane->ready))
			break;
		item = list_first_entry(&lane->ready,
				       struct gpu_m4_drm_provider_lane_item, ready_link);
		/* Head-of-line: a vetoed successor and everything behind it waits
		 * for a later pass. Failures and cancellations never wait.
		 */
		if (!lane->error && !item->closed && !lane->closing &&
		    !gpu_m4_lane_admit_locked(lane, item))
			break;
		list_del_init(&item->ready_link);
		if (lane->error) {
			gpu_m4_drm_provider_job_fault_locked(item->job, lane->error);
			ret = lane->error;
		} else if (item->closed || lane->closing) {
			ret = gpu_m4_drm_provider_job_cancel_locked(item->job);
		} else {
			u32 slot = lane->inflight_count;
			u64 deadline;

			/* Establish exclusive lane ownership BEFORE publisher callbacks,
			 * including a fault callback which can mark this lane failed.
			 */
			lane->inflight[slot] = item;
			lane->inflight_count++;
			item->state = GPU_M4_LANE_ACTIVE;
			item->timing.activated_ns = ktime_get_ns();
			lane->stats.jobs++;
			started = true;
			if (item->timing.ready_ns) {
				u64 d = item->timing.activated_ns - item->timing.ready_ns;

				lane->stats.queue_ns += d;
				lane->stats.queue_max = max(lane->stats.queue_max, d);
			}
			/* REMOVE-DEBUG: it was READY while its predecessor was ACTIVE. */
			if (item->overlap & GPU_M4_DRM_LANE_OVERLAP_SEEN) {
				lane->stats.overlap++;
				if (item->overlap & GPU_M4_DRM_LANE_OVERLAP_QUEUE)
					lane->stats.overlap_queue++;
				else if (item->overlap & GPU_M4_DRM_LANE_OVERLAP_VM)
					lane->stats.overlap_vm++;
				else
					lane->stats.overlap_foreign++;
			}
			/* A successor's provisional clock never replaces the head's. */
			deadline = item->command_count ? 0 : item->timing.activated_ns +
				(u64)lane->timeout_ms * NSEC_PER_MSEC;
			if (!slot)
				lane->deadline = deadline;
			item->timing.deadline_ns = deadline;
#ifdef CONFIG_DEBUG_FS
			gpu_m4_trace(GPU_M4_TRACE_ACTIVATE, 0, (u64)(uintptr_t)item->queue,
				(u64)(uintptr_t)item, item->command_count, slot);
#endif
			ret = gpu_m4_drm_provider_job_start_locked(item->job);
#ifdef CONFIG_DEBUG_FS
			gpu_m4_trace(GPU_M4_TRACE_STARTED, 0, (u64)(uintptr_t)item->queue,
				(u64)(uintptr_t)item, ret, 0);
#endif
			if (!ret && item->command_count && !item->timing.published) {
				ret = -EPROTO;
				gpu_m4_drm_provider_job_fault_locked(item->job, ret);
			}
			if (!ret && slot) {
				/* A successor's start held the mutex for a whole
				 * publication: observe the head again now, so its
				 * completion meanwhile is not left for the next pass
				 * (its signal precedes any further activation).
				 */
				lane->stats.successor_services++;
				gpu_m4_lane_service_active_locked(lane);
			}
			if (!ret)
				continue; /* Admit another while the cap allows. */
			lane->inflight[slot] = NULL;
			lane->inflight_count--;
		}
		/* A definite unpublished failure is local. A protected failure or
		 * latched provider failure stops all subsequent hardware publication.
		 */
		if (!lane->error)
			lane->error = gpu_m4_drm_provider_error_locked(lane->provider);
		if (gpu_m4_drm_provider_job_uncertain_locked(item->job) && !lane->error)
			lane->error = ret < 0 ? ret : -EIO;
		gpu_m4_lane_complete_locked(item, &lane->completed);
	}
	pending = lane->inflight_count || !list_empty(&lane->ready);
	/* Working-driver cadence: after progress (or a newer notification) a few
	 * short trailing polls, then the fallback interval.
	 */
	if (started || gpu_m4_lane_progress(lane) != progress || lane->last_seq != seq)
		lane->trailing = READ_ONCE(lane_trailing_polls);
	lane->last_seq = seq;
	poll_us = clamp_t(u32, READ_ONCE(lane_poll_us), 10, 10000);
	if (lane->trailing) {
		lane->trailing--;
		poll_us = poll_us < 20 ? poll_us : 20;
	}
	gpu_m4_lane_unlock_timed(lane, locked, started);
	gpu_m4_lane_drain_completed(lane);
	/* Notification racing the snapshot wins; otherwise one-shot 100 us. */
	if (READ_ONCE(lane->closing))
		pending = false; /* destruction drains; never self-rearm */
	if (pending && atomic_read(&lane->notify_seq) != seq)
		gpu_m4_lane_schedule(lane, 0);
	else if (pending)
		hrtimer_start(&lane->poll_timer, ns_to_ktime((u64)poll_us * NSEC_PER_USEC),
			      HRTIMER_MODE_REL);
}

struct gpu_m4_drm_provider_lane *gpu_m4_drm_provider_lane_create(
	struct gpu_m4_drm_provider *provider, u32 max_items, u32 timeout_ms,
	u32 poll_ms)
{
	struct gpu_m4_drm_provider_lane *lane;
	int ret;

	if (!provider || !max_items || max_items > 4096 || !timeout_ms ||
	    timeout_ms > 60000 || !poll_ms || poll_ms > timeout_ms)
		return ERR_PTR(-EINVAL);
	lane = kzalloc_obj(*lane);
	if (!lane)
		return ERR_PTR(-ENOMEM);
	ret = gpu_m4_drm_provider_get(provider);
	if (ret) {
		kfree(lane);
		return ERR_PTR(ret);
	}
	lane->provider = provider;
	lane->max_inflight = 1;
	lane->max_items = max_items;
	lane->timeout_ms = timeout_ms;
	lane->poll_ms = poll_ms;
	INIT_LIST_HEAD(&lane->owners);
	INIT_LIST_HEAD(&lane->ready);
	INIT_LIST_HEAD(&lane->completed);
	INIT_DELAYED_WORK(&lane->work, gpu_m4_lane_work);
	hrtimer_setup(&lane->poll_timer, gpu_m4_lane_poll_timer, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL);
	atomic_set(&lane->notify_seq, 0);
	return lane;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_create);

int gpu_m4_drm_provider_lane_destroy(struct gpu_m4_drm_provider_lane *lane)
{
	struct mutex *lock;
	int ret;

	if (!lane)
		return -EINVAL;
	lock = gpu_m4_lane_lock(lane);
	lockdep_assert_not_held(lock);
	if (current_work() == &lane->work.work)
		return -EDEADLK;
	mutex_lock(lock);
	if (lane->count || lane->inflight_count || !list_empty(&lane->owners) ||
	    !list_empty(&lane->ready) || !list_empty(&lane->completed)) {
		mutex_unlock(lock);
		return -EBUSY;
	}
	WRITE_ONCE(lane->closing, true);
	mutex_unlock(lock);
	hrtimer_cancel(&lane->poll_timer);
	cancel_delayed_work_sync(&lane->work);
	hrtimer_cancel(&lane->poll_timer);
	cancel_delayed_work_sync(&lane->work);
	ret = gpu_m4_drm_provider_put(lane->provider);
	if (ret)
		return ret;
	kfree(lane);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_destroy);

int gpu_m4_drm_provider_lane_set_policy(struct gpu_m4_drm_provider_lane *lane,
	void *owner, const struct gpu_m4_drm_provider_lane_policy *policy)
{
	struct mutex *lock;
	int ret = 0;

	if (!lane || !owner || IS_ERR(owner) || !policy || !policy->admit_locked)
		return -EINVAL;
	lock = gpu_m4_lane_lock(lane);
	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	if (lane->count || lane->inflight_count || lane->policy_owner || lane->closing) {
		ret = -EBUSY;
	} else {
		lane->policy = *policy;
		lane->policy_owner = owner;
		lane->max_inflight = clamp_t(u32, policy->max_inflight ?: max_inflight, 1,
					     GPU_M4_DRM_LANE_MAX_INFLIGHT);
	}
	mutex_unlock(lock);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_set_policy);

u32 gpu_m4_drm_provider_lane_max_inflight_locked(struct gpu_m4_drm_provider_lane *lane)
{
	lockdep_assert_held(gpu_m4_lane_lock(lane));
	return lane->max_inflight;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_max_inflight_locked);

struct gpu_m4_drm_provider_lane_item *gpu_m4_drm_provider_lane_ready_first_locked(
	struct gpu_m4_drm_provider_lane *lane)
{
	lockdep_assert_held(gpu_m4_lane_lock(lane));
	return list_empty(&lane->ready) ? NULL :
		list_first_entry(&lane->ready, struct gpu_m4_drm_provider_lane_item, ready_link);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_ready_first_locked);

int gpu_m4_drm_provider_lane_item_init_locked(
	struct gpu_m4_drm_provider_lane *lane,
	struct gpu_m4_drm_provider_lane_item *item,
	struct gpu_m4_drm_provider_job *job, void *queue)
{
	if (!lane || !item || !job || !queue)
		return -EINVAL;
	lockdep_assert_held(gpu_m4_lane_lock(lane));
	if (lane->closing || lane->error ||
	    gpu_m4_drm_provider_error_locked(lane->provider))
		return lane->error ? lane->error : -ENODEV;
	if (lane->count == lane->max_items)
		return -ENOSPC;
	*item = (struct gpu_m4_drm_provider_lane_item) {
		.lane = lane, .job = job, .queue = queue, .state = GPU_M4_LANE_PRIVATE,
	};
	INIT_LIST_HEAD(&item->owner_link);
	INIT_LIST_HEAD(&item->ready_link);
	INIT_LIST_HEAD(&item->completion_link);
	list_add_tail(&item->owner_link, &lane->owners);
	lane->count++;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_item_init_locked);

int gpu_m4_drm_provider_lane_item_fini_locked(
	struct gpu_m4_drm_provider_lane_item *item)
{
	struct gpu_m4_drm_provider_lane *lane;

	if (!item || !item->lane)
		return -EINVAL;
	lane = item->lane;
	lockdep_assert_held(gpu_m4_lane_lock(lane));
	if ((item->state != GPU_M4_LANE_PRIVATE && item->state != GPU_M4_LANE_DONE) ||
	    (item->state == GPU_M4_LANE_DONE && item->job) ||
	    !list_empty(&item->ready_link) || !list_empty(&item->completion_link))
		return -EBUSY;
	list_del_init(&item->owner_link);
	lane->count--;
	item->lane = NULL;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_item_fini_locked);

int gpu_m4_drm_provider_lane_item_set_priority_locked(
	struct gpu_m4_drm_provider_lane_item *item, u32 priority)
{
	if (!item || !item->lane || priority >= GPU_M4_DRM_LANE_PRIORITIES)
		return -EINVAL;
	lockdep_assert_held(gpu_m4_lane_lock(item->lane));
	if (item->state != GPU_M4_LANE_PRIVATE)
		return -EBUSY;
	item->priority = priority;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_item_set_priority_locked);

void gpu_m4_drm_provider_lane_enqueue_locked(
	struct gpu_m4_drm_provider_lane_item *item)
{
	struct gpu_m4_drm_provider_lane *lane = item->lane;
	struct gpu_m4_drm_provider_lane_item *next;
	struct list_head *at = &lane->ready;

	lockdep_assert_held(gpu_m4_lane_lock(lane));
	if (WARN_ON(item->state != GPU_M4_LANE_PRIVATE || !item->job))
		return;
	item->state = GPU_M4_LANE_READY;
	item->timing.ready_ns = ktime_get_ns();
	/* Before the first READY item of a lower level (else at the tail): one
	 * queue's items share a level, so its own order is preserved.
	 */
	list_for_each_entry(next, &lane->ready, ready_link) {
		if (next->priority < item->priority) {
			at = &next->ready_link;
			break;
		}
	}
	list_add_tail(&item->ready_link, at);
	gpu_m4_lane_schedule(lane, 0);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_enqueue_locked);

void gpu_m4_drm_provider_lane_close_queue_locked(
	struct gpu_m4_drm_provider_lane *lane, void *queue)
{
	struct gpu_m4_drm_provider_lane_item *item;

	lockdep_assert_held(gpu_m4_lane_lock(lane));
	list_for_each_entry(item, &lane->owners, owner_link)
		if (item->queue == queue)
			item->closed = true;
	/* Active work must retire normally. PRIVATE entries own no transferred
	 * runtime ref; scheduler close will cancel their future observers.
	 */
	gpu_m4_lane_schedule(lane, 0);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_close_queue_locked);

void gpu_m4_drm_provider_lane_fault_locked(
	struct gpu_m4_drm_provider_lane *lane, int error)
{
	lockdep_assert_held(gpu_m4_lane_lock(lane));
	if (!lane->error)
		lane->error = error < 0 ? error : -EIO;
	gpu_m4_drm_provider_fault_locked(lane->provider, lane->error);
	gpu_m4_lane_schedule(lane, 0);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_lane_fault_locked);

MODULE_DESCRIPTION("Private Apple M4 dependency-ready hardware lane and drain");
MODULE_LICENSE("GPL");
