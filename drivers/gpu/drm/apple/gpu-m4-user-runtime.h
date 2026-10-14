/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_USER_RUNTIME_H
#define APPLE_GPU_M4_USER_RUNTIME_H

/* One actual provider job/lane owns a public batch. Internal command
 * serialization keeps user cache ownership protected until the WHOLE batch
 * retires; no intermediate command can signal or expose WB data to the CPU.
 */
static int m4_user_render_packet_publish_locked(struct m4_user_job *j, u32 index, bool defer);
static int m4_user_render_packet_close_locked(struct m4_user_job *j);
static int m4_user_render_packet_retired_locked(struct m4_user_job *j, u32 index);

/* Stable, charged per-command callback storage. Historical observations are
 * borrowed from the native owner's real CORE receipts, never their authority.
 */
struct m4_user_compute_reuse {
	struct m4_user_job *job;
	u32 index;
	struct gpu_m4_compute_reuse_owner owner[2];
	struct gpu_m4_compute_reuse_input input;
	struct gpu_m4_compute_reuse_binding binding;
};

/* A bank is held from acceptance until CORE retirement or an unpublished
 * release: accepted (prepared) and in-flight jobs alike, never a retired
 * last owner. Acquisition never waits: two accepted jobs may share a bank
 * (they lease the same slots only one after the other); the lane admits a
 * successor only in a bank no in-flight job holds (inflight2), so jobs in
 * flight together never share a slot or PB ID. An early first window (the
 * core's begin_dependencies) may hold its bank while its batch's
 * dependencies wait: it never blocks a head, which always starts alone.
 */
static int m4_user_bank_acquire_locked(struct m4_gpu_provider_runtime *p, struct m4_user_job *j)
{
	u32 bank = j->native_stage->bank;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (j->bank_held || bank >= m4_user_bank_count(p) || p->user_bank_holders[bank] == U32_MAX)
		return -EINVAL;
	j->bank = bank;
	j->bank_held = true;
	p->user_bank_holders[bank]++;
	p->user_bank_busy |= BIT(bank);
	j->queue->bank = bank;
	j->queue->bank_holders[bank]++;
	m4_user_bank_unstage_locked(p, j->native_stage);
	p->user_inflight2_stats.bank_jobs[bank]++; /* REMOVE-DEBUG */
	return 0;
}

static void m4_user_bank_release_locked(struct m4_gpu_provider_runtime *p, struct m4_user_job *j)
{
	lockdep_assert_held(&p->hello->gpu->lock);
	if (!j->bank_held ||
	    WARN_ON(j->bank >= m4_user_bank_count(p) || !p->user_bank_holders[j->bank]))
		return;
	j->bank_held = false;
	if (!--p->user_bank_holders[j->bank])
		p->user_bank_busy &= ~BIT(j->bank);
	if (!WARN_ON(!j->queue->bank_holders[j->bank]))
		j->queue->bank_holders[j->bank]--;
}

/* Drop the global-tail predecessor's notifier: at this job's actual CORE
 * retirement, or its release when it never published (a published uncertain
 * job keeps it in quarantine).
 */
static void m4_user_cross_put_locked(struct m4_user_job *j)
{
	struct m4_user_notifier *n = j->cross.notifier;

	j->cross = (struct m4_user_cross) { };
	m4_user_notifier_put_locked(n);
}

static int m4_user_timestamp_owner_locked(void *owner,
	struct gpu_m4_drm_provider *provider, struct gpu_m4_drm_upper *upper,
	struct gpu_m4_drm *frontend, struct mutex *lock)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_gpu_hello *h = p->hello;

	lockdep_assert_held(&h->gpu->lock);
	/* Scalar ownership only: the upper inventory calls this recursively. */
	return p->ordinary && !p->error && p->complete && h->ready == p->ready &&
		h->gpu->hello == h && p->ready->provider_runtime == p &&
		p->execution == p->ready->execution && lock == &h->gpu->lock &&
		provider == p->provider && frontend == p->frontend && upper == p->ready->upper &&
		!h->gpu->error && !h->error && !p->ready->error &&
		refcount_read(&p->session_refs) ? 0 : -ESTALE;
}

static int m4_user_timestamp_lane_locked(void *owner,
	struct gpu_m4_drm_provider_job *job)
{
	return m4_user_lane_check_locked(owner, job);
}

static int m4_user_timestamp_hold_locked(void *owner,
	struct gpu_m4_drm_provider_version *version,
	struct gpu_m4_drm_provider_object *object,
	const struct gpu_m4_drm_timestamp *borrowed, void **alias, u64 *firmware_va)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct gpu_m4_drm_timestamp_alias_request *request;
	u64 va;
	int ret;

	if (!alias || !firmware_va || !p->ts_aliases)
		return -EINVAL;
	ret = gpu_m4_drm_timestamp_alias_hold_locked(p->ts_aliases, version, object,
		borrowed, &request, &va);
	if (!ret) {
		*alias = request;
		*firmware_va = va;
	}
	return ret;
}

static void m4_user_timestamp_put_locked(void *owner, void *alias)
{
	struct m4_gpu_provider_runtime *p = owner;

	lockdep_assert_held(&p->hello->gpu->lock);
	gpu_m4_drm_timestamp_alias_request_put_locked(alias);
}

static void m4_user_job_images_put_locked(struct m4_user_job *j)
{
	for (u32 i = 0; j->compute && i < j->command_count; i++)
		gpu_m4_compute_ts_put(&j->compute[i].timestamps);
}

static void m4_user_release_locked(void *owner, void *runtime_job)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_user_job *j = runtime_job;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (WARN_ON(!j || j->runtime != p))
		return;
	ret = m4_user_lane_release_locked(p, j);
	if (WARN_ON(ret))
		return; /* No release from an error fence or an uncertain core state. */
	m4_user_bank_release_locked(p, j);
	m4_user_cross_put_locked(j);
	if (j->token) {
		ret = gpu_m4_drm_timestamp_alias_token_cancel_locked(j->token);
		if (WARN_ON(ret))
			return;
		j->token = NULL;
	}
	m4_user_job_images_put_locked(j);
	list_add_tail(&j->garbage, &p->user_job_garbage);
	schedule_work(&p->user_reap);
}

static void m4_user_collect_native(struct m4_gpu_provider_runtime *p)
{
	struct mutex *lock = &p->hello->gpu->lock;
	struct m4_user_job *j;

	lockdep_assert_not_held(lock);
	/* Checked closed-queue cold cut first (under the mutex, deferred work):
	 * its released histories are then disposed by the reapers below. Any
	 * runtime/stage/BO garbage they enqueue reschedules this worker.
	 * Retained stages (source version then protected/working) retry their
	 * release here, so their BO and cell-wrapper refs cannot pin cells.
	 */
	m4_user_native_cold_collect(p);
	m4_user_native_retained_reap(p);
	for (;;) {
		mutex_lock(lock);
		if (list_empty(&p->user_job_garbage)) {
			mutex_unlock(lock);
			break;
		}
		j = list_first_entry(&p->user_job_garbage, struct m4_user_job, garbage);
		list_del_init(&j->garbage);
		mutex_unlock(lock);
		/* A disposal carrier holds only transferred displaced[] refs; the
		 * retired job cleared its own entries, so none is put twice.
		 */
		for (u32 i = 0; i < 3; i++)
			m4_user_native_stage_put(p, j->displaced[i]);
		m4_user_native_stage_put(p, j->native_stage);
		for (u32 i = 0; j->render && i < j->command_count; i++) {
			kvfree(j->render[i].pb_pages);
			kvfree(j->render[i].pb_blocks);
		}
		kvfree(j->compute);
		kvfree(j->compute_reservation);
		kvfree(j->compute_reuse);
		kvfree(j->render);
		kvfree(j->render_reservation);
		kvfree(j->session);
		mutex_lock(lock);
		p->user_native_bytes -= j->charge;
		/* An untransferred carrier owns no stage ref (displaced[] empty). */
		if (j->disposal)
			p->user_native_bytes -= j->disposal->charge;
		mutex_unlock(lock);
		kvfree(j->disposal);
		kvfree(j);
	}
	/* Canonical backing puts precede the last native NC CPU view/GEM put. */
	if (p->ready->upper)
		gpu_m4_drm_upper_reap(p->ready->upper);
	m4_user_native_reap(p);
}

/* Publication depth of a homogeneous render child: its whole bounded window.
 * Every render owns a distinct UMA slot/PB ID, so no outstanding owner shares
 * a manager, scene or allocator with a successor.
 */
#define M4_USER_RENDER_DEPTH GPU_M4_DRM_PROVIDER_WINDOW_MAX_COMMANDS

/* Render packet batching (A/B switch, read once per publication scope):
 * consecutive renders published in one scope form packets of at most
 * render_batch_max members (all ring entries, one pipe record per engine, one
 * FS then one TA kick; flush requests on the packet's last render only).
 * 0 restores one packet per render.
 *
 * A deferred member is invisible to firmware until its packet closes, and
 * each render takes the host roughly 0.05-0.15 ms to build. Holding a whole
 * window back delays its first render by the build of all the others, an idle
 * bubble whenever the GPU drains the previous work first; native publication
 * exposes each changed ring horizon at once. One packet per render instead
 * pays the completion flush between every pair of renders. Pairs keep the
 * first render of a window visible after one more build and halve the flushes.
 */
static bool render_batch = true;
module_param(render_batch, bool, 0644);
MODULE_PARM_DESC(render_batch, "Batch consecutive renders into shared firmware packets (default 1)");
static unsigned int render_batch_max = 2;
module_param(render_batch_max, uint, 0644);
MODULE_PARM_DESC(render_batch_max, "Renders per batched firmware packet (default 2, at most a window)");

/* Service health cadence (A/B switch for hardware rollback, read per call).
 * 0: one service snapshot per lane-worker pass (m4_user_service_locked()),
 * a scalar gate before each publication (m4_user_publish_gate_locked()) and
 * the upper graph pass only at a changed upper revision; the complete READY
 * health stays at fault/TDR boundaries, binding installation and the READY
 * owner's periodic diagnostic. 1 restores the per-job READY/upper/token
 * passes on start, publication and every poll.
 */
static bool health_per_job;
module_param(health_per_job, bool, 0644);
MODULE_PARM_DESC(health_per_job, "Per-job READY/upper/token health on start, publish and retire (default 0: one service snapshot per lane-worker pass)");

/* Mixed render/compute children (A/B switch, read once per acceptance): 1
 * publishes the whole bounded window ahead, each command after the first
 * strictly waiting on its immediate predecessor's final firmware stamp
 * (render: FS 0x71 +0x630; compute: 15 +0x640) on the queue's notifier, as
 * the working publisher's mixed packets do. 0 restores serial publication
 * (one command reachable at a time) for mixed children.
 */
static bool mixed_window = true;
module_param(mixed_window, bool, 0644);
module_param_named(barrier_relax, gpu_m4_barrier_relax, uint, 0644);
MODULE_PARM_DESC(barrier_relax, "Experiment: opcode-4 +0x3c=0 on waits behind an untimed command (bit0 compute, bit1 TA behind compute, bit2 early-tiling FS, bit3 TA behind render; default 15)");

/* Publication depth of a compute-only child (read once per acceptance):
 * commands reachable ahead of their receipts, each strictly waiting on its
 * immediate predecessor's FW stamp (as in a mixed window). 2 is the r65
 * rule. Depth 2 left every third dependent compute behind a host receipt
 * and a fresh firmware pickup (~250 us per pair); the whole bounded window,
 * as render-only and mixed children already publish, keeps the firmware
 * queue fed. Each command still has its own UMA slot and strict prior edge.
 */
static unsigned int compute_depth = 32;
module_param(compute_depth, uint, 0644);
MODULE_PARM_DESC(compute_depth, "Publication depth of a compute-only window (2..32, default 32 = the whole window)");

/* inflight2 behind a same-queue window of any class mix (A/B switch, read per
 * admission): the next child window of a queue may start beside its
 * predecessor whatever their classes; its first command strictly waits on the
 * predecessor's final stamp (render FS 0x71 or compute 15, cross-allocator
 * targets unordered). 0 keeps the r65 rule (render-only behind a render tail).
 */
static bool inflight2_mixed = true;

/* Fresh-unit slot choice (A/B switch): 1 prefers a slot whose last owner is
 * this queue's own unit not used by the child, then a closed client's slot,
 * then the lowest free one. 0: closed client's slot, then lowest.
 */
static bool slot_own_first = true;
module_param(slot_own_first, bool, 0644);
module_param_named(ta_alloc, gpu_m4_render_alloc_word[0], ullong, 0644);
module_param_named(fs_alloc, gpu_m4_render_alloc_word[1], ullong, 0644);
module_param_named(ta_minmax, gpu_m4_render_alloc_minmax[0], ullong, 0644);
module_param_named(fs_minmax, gpu_m4_render_alloc_minmax[1], ullong, 0644);
module_param(inflight2_mixed, bool, 0644);
MODULE_PARM_DESC(inflight2_mixed, "inflight2: admit any same-queue window class mix behind its predecessor (default 1)");
MODULE_PARM_DESC(mixed_window, "Publish a mixed render/compute window ahead with strict cross-class edges (default 1; 0 = serial)");

/* First UMA slot and slot count of a command type's domain inside host bank
 * b. The single bank is the whole domain; inflight2 has two of 16.
 */
static u32 m4_user_slot_first(const struct m4_gpu_provider_runtime *p, u16 type, u32 bank)
{
	return (type == DRM_ASAHI_CMD_RENDER ? GPU_M4_RENDER_UMA_SLOT_FIRST : 1) +
		bank * m4_user_bank_size(p);
}

static u32 m4_user_slot_count(const struct m4_gpu_provider_runtime *p, u16 type)
{
	(void)type;
	return m4_user_bank_slots(p);
}

/* A fresh unit's slot within bank: the first free one (not leased by this
 * child) whose current owner belongs to a CLOSED client, else the lowest
 * free one. A closed client's last owners are released only by a distinct,
 * fully retired later use of the same slot (the firmware may retain its
 * UMA/PB state until then); steering fresh units there makes later work
 * release them instead of landing on slots that live clients would reuse
 * anyway. O(bank size) through the per-slot table, no unit walk. Returns
 * the slot count when none is free.
 */
/* Exited clients' last owners first (always, slot_closed_first): every
 * displacement lets their whole retained windows be reclaimed, while the
 * queue's own last owners are displaced by its warm reuse anyway.
 */
static bool slot_closed_first = true;
module_param(slot_closed_first, bool, 0644);
MODULE_PARM_DESC(slot_closed_first, "Steer fresh units to exited clients' last-owner slots first (default 1)");

static u32 m4_user_slot_steer_locked(struct m4_gpu_provider_runtime *p, u16 type,
	u32 bank, u32 mask, const struct m4_user_queue *queue)
{
	u32 count = m4_user_slot_count(p, type), first = m4_user_slot_first(p, type, bank);
	u32 lowest = count, closed = count, own = count;
	bool closed_first = READ_ONCE(slot_closed_first);
	/* Under native backing pressure a closed client's last owners are the
	 * only reclaimable backing left: displace them first.
	 */
	bool own_first = READ_ONCE(slot_own_first) &&
		m4_user_native_bytes_free_locked(p) >= M4_ORDINARY_NATIVE_MEMORY_LIMIT / 4;

	for (u32 n = 0; n < count; n++) {
		struct m4_user_native_unit *owner;

		if (mask & BIT(n))
			continue;
		if (lowest == count)
			lowest = n;
		if (m4_user_native_slot_owner_locked(p, type, first + n, &owner))
			continue;
		if (owner->queue && owner->queue->closed) {
			if (closed == count)
				closed = n;
			if (closed_first || !own_first)
				break;
			continue;
		}
		/* The queue's own last owner of a slot this child does not use:
		 * displacing it makes that unit reusable (A/B in one slot)
		 * instead of leaving it last owner and constructing yet another.
		 */
		if (own_first && owner->queue == queue && own == count) {
			own = n;
			if (!closed_first)
				break;
		}
	}
	if (closed != count && (closed_first || own == count)) {
		if (closed != lowest)
			m4_user_native_unit_stats.slot_steered++;
		return closed;
	}
	return own != count ? own : lowest;
}

/* Lease each command's UMA slot at acceptance (an unpublished, definite
 * failure point): a warm unit keeps its ORIGINAL slot (render: and its PB ID,
 * slot - 33), fresh units take the lowest slots of their type's domain that no
 * other command of this child holds. Commands of one child never share a slot,
 * so no outstanding predecessor's slot is overwritten and no intra-child
 * replacement receipt is needed. Within the bounded execution window a
 * compute-only child publishes two deep, a render-only child (and, with
 * mixed_window, a mixed child) the whole window; otherwise serial.
 */
static int m4_user_command_slots_locked(struct m4_user_job *j)
{
	struct m4_user_native_stage *s = j->native_stage;
	u32 used[2] = { };
	bool compute_only = true, render_only = true;

	if (!s || s->count != j->command_count || j->command_count > ARRAY_SIZE(j->cmd))
		return -ESTALE;
	for (u32 i = 0; i < j->command_count; i++) {
		u16 type = s->command[i].type;
		u32 slot, bit;

		j->cmd[i].uma_slot = 0;
		compute_only &= type == DRM_ASAHI_CMD_COMPUTE;
		render_only &= type == DRM_ASAHI_CMD_RENDER;
		if ((type != DRM_ASAHI_CMD_COMPUTE && type != DRM_ASAHI_CMD_RENDER) ||
		    !m4_user_native_reused(s, i))
			continue;
		slot = m4_user_native_reused_slot(s, i);
		if (!m4_user_native_bank_slot(j->runtime, type, j->bank, slot))
			return -ESTALE;
		bit = BIT(slot - m4_user_slot_first(j->runtime, type, j->bank));
		if (used[type == DRM_ASAHI_CMD_RENDER] & bit)
			return -ESTALE;
		used[type == DRM_ASAHI_CMD_RENDER] |= bit;
		j->cmd[i].uma_slot = slot;
	}
	for (u32 i = 0; i < j->command_count; i++) {
		u16 type = s->command[i].type;
		u32 *mask = &used[type == DRM_ASAHI_CMD_RENDER], n = 0;

		if ((type != DRM_ASAHI_CMD_COMPUTE && type != DRM_ASAHI_CMD_RENDER) ||
		    j->cmd[i].uma_slot)
			continue;
		n = m4_user_slot_steer_locked(j->runtime, type, j->bank, *mask, j->queue);
		if (n >= m4_user_slot_count(j->runtime, type))
			return -E2BIG;
		*mask |= BIT(n);
		j->cmd[i].uma_slot = m4_user_slot_first(j->runtime, type, j->bank) + n;
	}
	j->depth = 1;
	j->mixed = false;
	if (j->command_count <= GPU_M4_DRM_PROVIDER_WINDOW_MAX_COMMANDS && compute_only) {
		u32 d = READ_ONCE(compute_depth);

		d = d < 2 ? 2 : d > M4_USER_RENDER_DEPTH ? M4_USER_RENDER_DEPTH : d;
		j->depth = min_t(u32, d, j->command_count);
	} else if (j->command_count <= GPU_M4_DRM_PROVIDER_WINDOW_MAX_COMMANDS && render_only) {
		j->depth = min_t(u32, M4_USER_RENDER_DEPTH, j->command_count);
	} else if (j->command_count <= GPU_M4_DRM_PROVIDER_WINDOW_MAX_COMMANDS &&
		 READ_ONCE(mixed_window)) {
		j->depth = min_t(u32, M4_USER_RENDER_DEPTH, j->command_count);
		j->mixed = j->depth > 1;
	}
	return 0;
}

static int m4_user_accept_locked(void *owner, void *queue,
	struct gpu_m4_drm_provider_job *job, void **runtime_job)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_user_queue *q = queue;
	struct gpu_m4_drm_provider_job_view view = { };
	struct m4_user_job *j;
	u64 charge;
	u32 count;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!runtime_job || !q || q->runtime != p || !q->notifier || !p->complete)
		return -EINVAL;
	/* Every record of the queue's notifier names its VM's context. */
	if (!gpu_m4_exec_user_asid(q->notifier->asid) || q->notifier->asid != q->vm->asid)
		return -ESTALE;
	ret = gpu_m4_drm_provider_job_check_accepting_locked(job, p->provider);
	if (ret)
		return ret;
	gpu_m4_drm_provider_job_view(job, &view);
	if (view.runtime_queue != q || view.runtime_job || !view.batch ||
	    !view.batch->count || view.batch->count > GPU_M4_DRM_JOB_MAX_COMMANDS ||
	    !view.stage_owner)
		return -ESTALE;
	charge = sizeof(*j) + (u64)view.batch->count *
		(sizeof(*j->compute) + sizeof(*j->compute_reservation) + sizeof(*j->compute_reuse) +
		 sizeof(*j->render) + sizeof(*j->render_reservation) +
		 5 * sizeof(*j->session));
	ret = m4_user_native_charge_locked(p, charge);
	if (ret)
		return ret;
	j = kvzalloc(sizeof(*j), GFP_KERNEL);
	if (!j) {
		p->user_native_bytes -= charge;
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&j->garbage);
	j->runtime = p;
	j->queue = q;
	j->asid = q->notifier->asid;
	j->charge = charge;
	j->command_count = view.batch->count;
	j->compute = kvcalloc(j->command_count, sizeof(*j->compute), GFP_KERNEL);
	j->compute_reservation = kvcalloc(j->command_count, sizeof(*j->compute_reservation), GFP_KERNEL);
	j->compute_reuse = kvcalloc(j->command_count, sizeof(*j->compute_reuse), GFP_KERNEL);
	j->render = kvcalloc(j->command_count, sizeof(*j->render), GFP_KERNEL);
	j->render_reservation = kvcalloc(j->command_count, sizeof(*j->render_reservation), GFP_KERNEL);
	j->session = kvcalloc(j->command_count * 5, sizeof(*j->session), GFP_KERNEL);
	if (!j->compute || !j->compute_reservation || !j->compute_reuse || !j->render ||
	    !j->render_reservation || !j->session) {
		ret = -ENOMEM;
		goto fail;
	}
	/* Preallocate, before any publication, the carrier that later hands the
	 * displaced[] disposal refs to the reaper: the success callback cannot
	 * allocate. Separately charged; refunded only when it is really freed.
	 */
	ret = m4_user_native_charge_locked(p, sizeof(*j->disposal));
	if (ret)
		goto fail;
	j->disposal = kvzalloc(sizeof(*j->disposal), GFP_KERNEL);
	if (!j->disposal) {
		p->user_native_bytes -= sizeof(*j->disposal);
		ret = -ENOMEM;
		goto fail;
	}
	INIT_LIST_HEAD(&j->disposal->garbage);
	j->disposal->runtime = p;
	j->disposal->charge = sizeof(*j->disposal);
	ret = m4_user_native_stage_get_locked(view.stage_owner);
	if (ret)
		goto fail;
	j->native_stage = view.stage_owner;
	for (u32 i = 0; i < j->command_count; i++) {
		const struct m4_user_render_native *n;
		u64 bytes;

		if (view.batch->commands[i].type != DRM_ASAHI_CMD_RENDER)
			continue;
		n = m4_user_native_render(j->native_stage, i);
		if (!n) {
			ret = -ESTALE;
			goto fail;
		}
		bytes = (u64)n->pb.page_list_size + n->pb.block_list_size;
		ret = m4_user_native_charge_locked(p, bytes);
		if (ret)
			goto fail;
		j->charge += bytes;
		j->render[i].pb_pages_capacity = n->pb.page_list_size;
		j->render[i].pb_blocks_capacity = n->pb.block_list_size;
		j->render[i].pb_pages = kvzalloc(n->pb.page_list_size, GFP_KERNEL);
		j->render[i].pb_blocks = kvzalloc(n->pb.block_list_size, GFP_KERNEL);
		if (!j->render[i].pb_pages || !j->render[i].pb_blocks) {
			ret = -ENOMEM;
			goto fail;
		}
	}
	ret = m4_user_bank_acquire_locked(p, j);
	if (!ret)
		ret = m4_user_command_slots_locked(j);
	if (!ret)
		ret = m4_user_lane_accept_locked(p, j, job, q);
	if (!ret)
		ret = gpu_m4_drm_provider_lane_item_set_commands_locked(&j->item,
			j->command_count);
	if (!ret)
		ret = gpu_m4_drm_provider_lane_item_set_depth_locked(&j->item, j->depth);
	if (!ret)
		ret = gpu_m4_drm_provider_job_timestamp_count_locked(job, &count);
	/* Last fallible step: no successfully prepared token loses its borrowed
	 * synchronous job owner on an ensuing acceptance error.
	 */
	if (!ret && count)
		ret = gpu_m4_drm_timestamp_alias_token_prepare_locked(p->ts_aliases, job, &j->token);
	if (!ret) {
		*runtime_job = j;
		return 0;
	}
fail:
	if (j->item.lane)
		WARN_ON(m4_user_lane_release_locked(p, j));
	m4_user_bank_release_locked(p, j);
	list_add_tail(&j->garbage, &p->user_job_garbage);
	schedule_work(&p->user_reap);
	return ret;
}

static int m4_user_compute_reuse_owner_locked(struct m4_user_native_stage *stage,
	u32 index, bool current_command, struct gpu_m4_compute_reuse_owner *owner)
{
	const struct m4_user_compute_native *n = m4_user_native_compute(stage, index);
	const struct m4_user_native_history *h = current_command ? stage->pending : stage->history;
	const struct m4_user_native_receipt *receipt;
	struct gpu_m4_drm_provider_job_view view = { };
	int ret;

	if (!n || !h || (!h->job && !h->settled) || index >= h->count || h->count != stage->count)
		return M4_USER_WHY(-ESTALE, "reuse owner %u native=%d history=%d job=%d count=%u/%u current=%d",
			index, !!n, !!h, h && (h->job || h->settled), h ? h->count : 0, stage->count, current_command);
	if (!current_command) {
		ret = m4_user_native_history_command_locked(stage, index, DRM_ASAHI_CMD_COMPUTE, &receipt);
	} else {
		gpu_m4_drm_provider_job_view(h->job, &view);
		receipt = &h->command[index];
		if (h->complete ||
		    m4_user_inflight_slot_locked(stage->queue->runtime, h->job) < 0 ||
		    h->owner_stage != stage || view.stage_owner != stage || view.runtime_queue != stage->queue ||
		    view.version != h->version || view.snapshot != h->snapshot ||
		    receipt->type != DRM_ASAHI_CMD_COMPUTE ||
		    !m4_user_native_slot_valid(DRM_ASAHI_CMD_COMPUTE, receipt->uma_slot) ||
		    receipt->parameter_id || !receipt->submitted || receipt->submitted != receipt->completed ||
		    receipt->compute.uma_submitted != receipt->submitted ||
		    receipt->compute.uma_completed != receipt->completed)
			return M4_USER_WHY(-ESTALE, "reuse owner %u current: complete=%d job=%d stage=%d/%d queue=%d version=%d snapshot=%d type=%u slot=%u parameter=%u submitted=%llu completed=%llu uma=%llu/%llu",
				index, h->complete, m4_user_inflight_slot_locked(stage->queue->runtime, h->job) >= 0,
				h->owner_stage == stage, view.stage_owner == stage,
				view.runtime_queue == stage->queue, view.version == h->version,
				view.snapshot == h->snapshot, receipt->type, receipt->uma_slot,
				receipt->parameter_id, M4_WHY64(receipt->submitted),
				M4_WHY64(receipt->completed), M4_WHY64(receipt->compute.uma_submitted),
				M4_WHY64(receipt->compute.uma_completed));
		ret = gpu_m4_drm_provider_job_check_access_locked(h->job, stage->queue->runtime->provider,
			n->counter->object, 0, n->counter->coverage, M4_USER_NATIVE_RW, false, NULL);
		if (!ret)
			ret = gpu_m4_compute_reuse_retired_prefixes(&receipt->compute);
		if (!ret)
			ret = m4_user_native_compute_check_locked(stage, index,
				stage->queue->runtime->provider, h->version, h->snapshot);
	}
	if (ret)
		return M4_USER_WHY(ret, "reuse owner %u current=%d", index, current_command);
	*owner = (struct gpu_m4_compute_reuse_owner) {
		.identity = n, .vm_lease = stage->queue->vm,
		.work = n->private[GPU_M4_COMPUTE_PRIVATE_WORK]->map.vaddr,
		.work_fw = n->private[GPU_M4_COMPUTE_PRIVATE_WORK]->fw_va,
		.work_gpu = n->private[GPU_M4_COMPUTE_PRIVATE_WORK]->va,
		.counter_fw = n->counter->fw_va,
		.pool_state_gpu = n->private[GPU_M4_COMPUTE_PRIVATE_POOL_STATE]->va,
		.page_list_gpu = n->private[GPU_M4_COMPUTE_PRIVATE_PAGE_LIST]->va,
		.pool_gpu = n->private[GPU_M4_COMPUTE_PRIVATE_POOL]->va,
		.submitted = receipt->submitted, .uma_slot = receipt->uma_slot,
		.asid = h->asid, .binding_generation = h->binding_generation,
		.retirement = receipt->compute,
	};
	return 0;
}

/* The same-slot predecessor C of warm unit A is the actual CURRENT owner of
 * the command's leased (A's original) compute slot. Commands of one child
 * lease distinct slots, so C is never a command of this still-protected job:
 * it always carries its own successful CORE/cache receipt. C need not be the
 * newest compute command, nor belong to A's queue/VM.
 */
static int m4_user_compute_reuse_predecessor_locked(struct m4_user_job *j, u32 index,
	struct m4_user_native_stage **stage, u32 *command, bool *current_command)
{
	struct m4_user_native_stage *a = j->native_stage, *b;
	struct m4_user_native_unit *c;
	int ret;

	if (!a || index >= a->count || index >= ARRAY_SIZE(j->cmd) ||
	    index != j->published_count || j->cmd[index].published ||
	    !a->pending || a->pending->job != j->core_job || a->pending->complete ||
	    a->pending->count != j->command_count)
		return M4_USER_WHY(-ESTALE, "predecessor %u stage=%d count=%u published=%u/%d pending=%d job=%d complete=%d commands=%u/%u",
			index, !!a, a ? a->count : 0, j->published_count,
			index < ARRAY_SIZE(j->cmd) && j->cmd[index].published, a && a->pending,
			a && a->pending && a->pending->job == j->core_job,
			a && a->pending && a->pending->complete,
			a && a->pending ? a->pending->count : 0, j->command_count);
	ret = m4_user_native_slot_owner_locked(j->runtime, DRM_ASAHI_CMD_COMPUTE,
		j->cmd[index].uma_slot, &c);
	if (ret)
		return M4_USER_WHY(ret == -ENOENT ? -ESTALE : ret, "predecessor %u slot %u owner %d",
			index, j->cmd[index].uma_slot, ret);
	b = c->history->owner_stage;
	/* The held owner is the authority (never the firmware-owned table): a
	 * published, undisplaced, unborrowed unit of its own completed use.
	 */
	if (!b || b == a || !b->queue || b->queue->runtime != j->runtime || !refcount_read(&b->refs) ||
	    b->history != c->history || c->history_index >= b->count || b->unit[c->history_index] != c ||
	    !refcount_read(&c->refs) || !c->published || c->displacement || c->accepted || c->cold)
		return M4_USER_WHY(-ESTALE, "predecessor %u slot %u: stage=%d self=%d queue=%d refs=%u history=%d index=%u/%u unit=%d unit_refs=%u published=%d displaced=%d accepted=%d cold=%d",
			index, j->cmd[index].uma_slot, !!b, b == a, b && b->queue,
			b ? refcount_read(&b->refs) : 0, b && b->history == c->history,
			c->history_index, b ? b->count : 0,
			b && c->history_index < b->count && b->unit[c->history_index] == c,
			refcount_read(&c->refs), c->published, !!c->displacement, !!c->accepted, c->cold);
	ret = m4_user_native_history_check_locked(b);
	if (ret)
		return M4_USER_WHY(ret, "predecessor %u slot %u history", index, j->cmd[index].uma_slot);
	*stage = b; *command = c->history_index; *current_command = false;
	return 0;
}

static bool m4_user_compute_reuse_owner_same(const struct gpu_m4_compute_reuse_owner *a,
	const struct gpu_m4_compute_reuse_owner *b)
{
	return a->identity == b->identity && a->vm_lease == b->vm_lease &&
		a->work == b->work && a->work_fw == b->work_fw && a->work_gpu == b->work_gpu &&
		a->counter_fw == b->counter_fw && a->pool_state_gpu == b->pool_state_gpu &&
		a->page_list_gpu == b->page_list_gpu && a->pool_gpu == b->pool_gpu &&
		a->submitted == b->submitted && a->uma_slot == b->uma_slot && a->asid == b->asid &&
		a->binding_generation == b->binding_generation &&
		!memcmp(&a->retirement, &b->retirement, sizeof(a->retirement));
}

/* REMOVE-DEBUG: one firmware-owned global UMA hardware record (0x20 bytes at
 * slot * 0x20), decoded for diagnostics only. The table is never written by
 * the host and never owner authority: firmware packs a bound descriptor into
 * it and drains it back into that descriptor.
 */
struct m4_user_uma_record {
	u64 pool;
	u32 capacity, cursor, second, empty, pages;
};

static struct m4_user_uma_record m4_user_why_uma_record(struct m4_gpu_provider_runtime *p, u32 slot)
{
	const void *r = p->ready->buffer[GPU_M4_UMA].cpu + slot * 0x20;
	u64 w0 = m4_store_read64(r), w1 = m4_store_read64(r + 8);

	return (struct m4_user_uma_record) {
		.pool = (w0 & (BIT_ULL(41) - 1)) << 7, .capacity = w0 >> 41,
		.cursor = (w1 >> 5) & (BIT(22) - 1), .second = (w1 >> 33) & (BIT(22) - 1),
		.empty = (w1 >> 61) & 1, .pages = m4_store_read64(r + 0x10) & (BIT(22) - 1),
	};
}

/* Historical A/B CORE receipts and the current same-slot owner C all have
 * their own provenance. A-private live state must still equal A's saved
 * receipt; C is authenticated through ITS actual retained job/receipt and its
 * own live private descriptor/allocator state. Shared queue/notifier counters
 * describe later commands (possibly this job's own outstanding ones), never
 * historical A or C, so they are not required to revert to either.
 */
static int m4_user_compute_reuse_check_locked(void *cookie,
	const struct gpu_m4_compute_reuse_input *input,
	const struct gpu_m4_compute_reuse_binding *binding, u64 epoch, u8 wire,
	const void *output, const void *scratch)
{
	struct m4_user_compute_reuse *reuse = cookie;
	struct m4_user_job *j = reuse->job;
	struct m4_gpu_provider_runtime *p = j->runtime;
	struct m4_user_native_stage *a = j->native_stage, *origin, *b, *previous;
	struct m4_user_native_history *ha, *hp;
	const struct m4_user_compute_native *na, *nb;
	struct gpu_m4_drm_provider_job_view view = { };
	struct gpu_m4_compute_reuse_owner expected[2];
	const struct gpu_m4_exec_retirement *receipt;
	u64 completed;
	u32 index = reuse->index, prior_index, old_index, released_index, slot;
	bool current_command;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!a || !j->compute_reuse || index >= j->command_count || index >= ARRAY_SIZE(j->cmd) ||
	    index != j->published_count || j->cmd[index].published ||
	    j->command_count != a->count || reuse != &j->compute_reuse[index] ||
	    input != &reuse->input || binding != &reuse->binding ||
	    input->owner != &reuse->owner[0] || input->replacement != &reuse->owner[1] ||
	    !m4_user_native_reused(a, index) || a->cached || !a->pending || a->pending->job != j->core_job ||
	    a->queue != j->queue ||
	    p->user_last[0] == a || p->user_last[1] == a ||
	    !(slot = j->cmd[index].uma_slot) || input->destination_slot != slot ||
	    j->compute_reservation[index].reuse != input || j->compute_reservation[index].reuse_binding != binding ||
	    input->rebuilt_work != j->compute[index].base.work || input->cookie != reuse ||
	    binding->cookie != reuse || binding->check != m4_user_compute_reuse_check_locked ||
	    output != j->compute[index].work || scratch != j->compute[index].ts_scratch ||
	    epoch != j->binding_epoch || wire != j->binding_generation ||
	    binding->job != j->core_job || binding->vm != j->queue->vm->provider_vm)
		return M4_USER_WHY(-ESTALE, "reuse check %u/%u published=%u stage=%d reused=%d cached=%d pending=%d queue=%d last=%d,%d slot=%u/%u epoch=%llu/%llu wire=%u/%u job=%d",
			index, j->command_count, j->published_count, !!a,
			m4_user_native_reused(a, index), a && a->cached,
			a && a->pending && a->pending->job == j->core_job, a && a->queue == j->queue,
			p->user_last[0] == a, p->user_last[1] == a,
			index < ARRAY_SIZE(j->cmd) ? j->cmd[index].uma_slot : 0, input->destination_slot,
			M4_WHY64(epoch), M4_WHY64(j->binding_epoch), wire, j->binding_generation,
			binding->job == j->core_job);
	ret = m4_user_native_reuse_history_locked(a, index, &origin, &old_index,
		&b, &released_index);
	if (ret)
		return M4_USER_WHY(ret, "reuse check %u history", index);
	ha = origin->history;
	if (m4_user_native_history_job(ha) == j->core_job ||
	    m4_user_native_history_job(b->history) == j->core_job ||
	    binding->owner_retired_job != m4_user_native_history_job(ha) ||
	    binding->owner_receipt != &ha->command[old_index])
		return M4_USER_WHY(-ESTALE, "reuse check %u origin: own=%d displacer_own=%d job=%d receipt=%d",
			index, m4_user_native_history_job(ha) == j->core_job,
			m4_user_native_history_job(b->history) == j->core_job,
			binding->owner_retired_job == m4_user_native_history_job(ha),
			binding->owner_receipt == &ha->command[old_index]);
	ret = m4_user_compute_reuse_predecessor_locked(j, index, &previous, &prior_index,
		&current_command);
	if (ret)
		return M4_USER_WHY(ret, "reuse check %u predecessor", index);
	hp = previous->history;
	if (current_command || binding->replacement_current_command ||
	    binding->replacement_retired_job != m4_user_native_history_job(hp) ||
	    binding->replacement_receipt != &hp->command[prior_index] ||
	    input->last_owner != m4_user_native_compute(previous, prior_index))
		return M4_USER_WHY(-ESTALE, "reuse check %u predecessor %u changed: current=%d/%d job=%d receipt=%d identity=%d",
			index, prior_index, current_command, binding->replacement_current_command,
			binding->replacement_retired_job == m4_user_native_history_job(hp),
			binding->replacement_receipt == &hp->command[prior_index],
			input->last_owner == m4_user_native_compute(previous, prior_index));
	/* C can belong to a different (even closed but retained) client queue.
	 * Its own notifier/pin/cache/alias remain authority after A selects a
	 * different root; never authenticate C through A's current snapshot.
	 */
	if (!previous->notifier || !refcount_read(&previous->notifier->refs) ||
	    !previous->notifier->bo)
		return M4_USER_WHY(-ESTALE, "reuse check %u predecessor notifier=%d refs=%u bo=%d", index,
			!!previous->notifier, previous->notifier ? refcount_read(&previous->notifier->refs) : 0,
			previous->notifier && previous->notifier->bo);
	ret = hp->settled ? 0 :
		m4_user_native_bo_check_locked(previous->notifier->bo, p->provider, hp->version);
	if (!ret && !hp->settled)
		ret = gpu_m4_drm_provider_job_check_access_locked(hp->job, p->provider,
			previous->notifier->bo->object, 0, previous->notifier->bo->coverage,
			M4_USER_NATIVE_RW, true, NULL);
	if (!ret)
		ret = gpu_m4_drm_upper_check_lease_locked(previous->notifier->bo->upper,
			&previous->notifier->bo->backing, GPU_M4_DRM_UPPER_NORMAL,
			GPU_M4_DRM_UPPER_FW_ONLY, previous->notifier->bo->fw_va);
	if (ret)
		return M4_USER_WHY(ret, "reuse check %u predecessor notifier lease", index);
	gpu_m4_drm_provider_job_view(j->core_job, &view);
	if (binding->version != view.version || binding->snapshot != view.snapshot)
		return M4_USER_WHY(-ESTALE, "reuse check %u version=%d snapshot=%d", index,
			binding->version == view.version, binding->snapshot == view.snapshot);
	ret = m4_user_compute_check_locked(&j->compute[index]);
	if (!ret)
		ret = m4_user_compute_reuse_owner_locked(origin, old_index, false, &expected[0]);
	if (!ret)
		ret = m4_user_compute_reuse_owner_locked(previous, prior_index, false, &expected[1]);
	if (ret)
		return M4_USER_WHY(ret, "reuse check %u owners A=%u C=%u", index, old_index, prior_index);
	/* A returns to its ORIGINAL slot unless retargeted; C owns the
	 * destination slot either way.
	 */
	if (!m4_user_compute_reuse_owner_same(input->owner, &expected[0]) ||
	    !m4_user_compute_reuse_owner_same(input->replacement, &expected[1]) ||
	    (expected[0].uma_slot != slot && !m4_user_native_retargeted(a, index)) ||
	    expected[1].uma_slot != slot)
		return M4_USER_WHY(-ESTALE, "reuse check %u owner same A=%d C=%d slot=%u A=%u C=%u submitted A=%llu/%llu C=%llu/%llu",
			index, m4_user_compute_reuse_owner_same(input->owner, &expected[0]),
			m4_user_compute_reuse_owner_same(input->replacement, &expected[1]), slot,
			expected[0].uma_slot, expected[1].uma_slot, M4_WHY64(input->owner->submitted),
			M4_WHY64(expected[0].submitted), M4_WHY64(input->replacement->submitted),
			M4_WHY64(expected[1].submitted));
	na = m4_user_native_compute(a, index); nb = m4_user_native_compute(previous, prior_index);
	ret = m4_store_packed64(expected[0].work + 0xe54, &completed);
	if (ret)
		return M4_USER_WHY(ret, "reuse check %u A completed", index);
	/* The global UMA table is firmware-owned allocator state, never an
	 * owner name: C is the held slot owner. Its own descriptor still names C
	 * (+0 owner, +8 slot) and its own allocator completed its receipted use.
	 */
	if (completed != expected[0].submitted || m4_store_read64(na->counter->map.vaddr) != completed ||
	    *((const u8 *)expected[0].work + 0x88b) != ha->binding_generation ||
	    m4_store_read64(expected[1].work + 0xe00) != expected[1].work_fw ||
	    m4_ready_read32(expected[1].work + 0xe08) != slot ||
	    *((const u8 *)expected[1].work + 0x88b) != hp->binding_generation)
		return M4_USER_WHY(-ESTALE, "reuse check %u slot %u: A completed=%llu counter=%llu want %llu gen=%u/%u; C owner=%#llx want %#llx slot=%u gen=%u/%u",
			index, slot, M4_WHY64(completed), M4_WHY64(m4_store_read64(na->counter->map.vaddr)),
			M4_WHY64(expected[0].submitted), *((const u8 *)expected[0].work + 0x88b),
			ha->binding_generation, M4_WHY64(m4_store_read64(expected[1].work + 0xe00)),
			M4_WHY64(expected[1].work_fw), m4_ready_read32(expected[1].work + 0xe08),
			*((const u8 *)expected[1].work + 0x88b), hp->binding_generation);
	ret = m4_store_packed64(expected[1].work + 0xe54, &completed);
	if (ret)
		return M4_USER_WHY(ret, "reuse check %u C completed", index);
	/* C's own live private allocator still equals its CORE-receipted use. */
	receipt = &hp->command[prior_index].compute;
	if (m4_store_read64(nb->counter->map.vaddr) != expected[1].submitted ||
	    completed != expected[1].submitted ||
	    receipt->uma_submitted != expected[1].submitted ||
	    receipt->uma_completed != expected[1].submitted ||
	    m4_store_read64(nb->counter->map.vaddr + GPU_M4_EXEC_COMPUTE_STAMPS) !=
		receipt->timestamp_start ||
	    m4_store_read64(nb->counter->map.vaddr + GPU_M4_EXEC_COMPUTE_STAMPS + 8) !=
		receipt->timestamp_end)
		return M4_USER_WHY(-ESTALE, "reuse check %u C %u: counter=%llu completed=%llu uma=%llu/%llu want %llu; time=%#llx,%#llx receipt %#llx,%#llx",
			index, prior_index, M4_WHY64(m4_store_read64(nb->counter->map.vaddr)),
			M4_WHY64(completed), M4_WHY64(receipt->uma_submitted),
			M4_WHY64(receipt->uma_completed), M4_WHY64(expected[1].submitted),
			M4_WHY64(m4_store_read64(nb->counter->map.vaddr + GPU_M4_EXEC_COMPUTE_STAMPS)),
			M4_WHY64(m4_store_read64(nb->counter->map.vaddr + GPU_M4_EXEC_COMPUTE_STAMPS + 8)),
			M4_WHY64(receipt->timestamp_start), M4_WHY64(receipt->timestamp_end));
	return M4_USER_WHY(gpu_m4_compute_reuse_retired_prefixes(receipt), "reuse check %u C prefixes", index);
}

static int m4_user_compute_reuse_prepare_locked(struct m4_user_job *j, u32 index,
	struct m4_user_compute_reservation *r)
{
	struct m4_user_native_stage *a = j->native_stage, *origin, *b, *previous;
	struct m4_user_native_history *hp;
	struct m4_user_compute_reuse *reuse;
	struct gpu_m4_drm_provider_job_view view = { };
	u32 prior_index, old_index, released_index;
	bool current_command;
	int ret;

	if (!m4_user_native_reused(a, index))
		return 0;
	if (index >= j->command_count || !j->compute_reuse)
		return M4_USER_WHY(-ESTALE, "reuse prepare %u/%u storage=%d", index, j->command_count,
			!!j->compute_reuse);
	ret = m4_user_native_reuse_history_locked(a, index, &origin, &old_index,
		&b, &released_index);
	/* C: the destination slot's actual current owner, A's own slot or
	 * (idle-owner retarget) another slot of this bank. A's displacement in
	 * its old slot is the native layer's receipt proof (B retired there).
	 */
	if (!ret)
		ret = m4_user_compute_reuse_predecessor_locked(j, index, &previous, &prior_index, &current_command);
	if (ret)
		return M4_USER_WHY(ret, "reuse prepare %u slot %u", index, j->cmd[index].uma_slot);
	hp = current_command ? previous->pending : previous->history;
	reuse = &j->compute_reuse[index];
	memset(reuse, 0, sizeof(*reuse));
	reuse->job = j; reuse->index = index;
	ret = m4_user_compute_reuse_owner_locked(origin, old_index, false, &reuse->owner[0]);
	if (!ret)
		ret = m4_user_compute_reuse_owner_locked(previous, prior_index, current_command, &reuse->owner[1]);
	if (ret)
		return M4_USER_WHY(ret, "reuse prepare %u owners A=%u C=%u", index, old_index, prior_index);
	if (reuse->owner[0].submitted == U64_MAX)
		return M4_USER_WHY(-EOVERFLOW, "reuse prepare %u submitted", index);
	gpu_m4_drm_provider_job_view(j->core_job, &view);
	reuse->input = (struct gpu_m4_compute_reuse_input) {
		.owner = &reuse->owner[0], .replacement = &reuse->owner[1], .last_owner = reuse->owner[1].identity,
		.rebuilt_work = j->compute[index].base.work,
		.destination_slot = j->cmd[index].uma_slot, .cookie = reuse,
	};
	reuse->binding = (struct gpu_m4_compute_reuse_binding) {
		.job = j->core_job, .vm = j->queue->vm->provider_vm, .version = view.version,
		.snapshot = view.snapshot,
		.owner_retired_job = m4_user_native_history_job(origin->history),
		.replacement_retired_job = m4_user_native_history_job(hp),
		.owner_receipt = &origin->history->command[old_index],
		.replacement_receipt = &hp->command[prior_index], .cookie = reuse,
		.check = m4_user_compute_reuse_check_locked,
		.replacement_current_command = current_command,
	};
	r->reuse = &reuse->input; r->reuse_binding = &reuse->binding;
	r->chronology.submitted_before = r->chronology.completed_before = reuse->owner[0].submitted;
	r->chronology.submitted = reuse->owner[0].submitted + 1;
	return 0;
}

/* Strict prior-command edge: inside a bounded compute child, every command
 * after the first waits on its immediately preceding command's FIRMWARE
 * compute stamp (+0x640, event 15) through a record in its own counter page.
 * Queue order alone is not an execution dependency.
 */
static bool m4_user_compute_strict_prior(const struct m4_user_job *j, u32 index)
{
	return j->depth > 1 && index && index < j->command_count;
}

static int m4_user_compute_prior_expected(struct m4_user_job *j, u32 index,
	struct gpu_m4_compute_packet_dependency *prior)
{
	const struct m4_user_compute_native *n = m4_user_native_compute(j->native_stage, index);

	memset(prior, 0, sizeof(*prior));
	/* inflight2: the first command waits on the global in-flight tail, the
	 * predecessor job's last command, on THAT job's notifier page (held).
	 */
	if (!index && j->cross.notifier) {
		if (!n || !n->counter)
			return M4_USER_WHY(-ESTALE, "prior 0 cross native=%d counter=%d", !!n,
				n && n->counter);
		*prior = (struct gpu_m4_compute_packet_dependency) {
			.owner = j->cross.notifier,
			.firmware = { n->counter->fw_va + GPU_M4_COMPUTE_PACKET_WAIT_OFFSET,
				GPU_M4_EXEC_DEPENDENCY_SIZE },
			.stamp_fw = j->cross.stamp_fw, .target = j->cross.target,
			.event = j->cross.event, .strict = true,
			.relaxed = j->cross.untimed && (gpu_m4_barrier_relax & 1),
		};
		return 0;
	}
	if (!m4_user_compute_strict_prior(j, index))
		return 0;
	/* Mixed window: behind a render, wait on its final FS firmware stamp. */
	if (j->mixed && j->cmd[index - 1].engine_mask == 3) {
		if (!n || !n->counter || !j->cmd[index - 1].published ||
		    !j->cmd[index - 1].final_target || !j->queue->notifier ||
		    !j->queue->notifier->bo)
			return M4_USER_WHY(-ESTALE, "prior %u behind render native=%d counter=%d published=%d engines=%#x",
				index, !!n, n && n->counter, j->cmd[index - 1].published,
				j->cmd[index - 1].engine_mask);
		*prior = (struct gpu_m4_compute_packet_dependency) {
			.owner = &j->cmd[index - 1],
			.firmware = { n->counter->fw_va + GPU_M4_COMPUTE_PACKET_WAIT_OFFSET,
				GPU_M4_EXEC_DEPENDENCY_SIZE },
			.stamp_fw = j->queue->notifier->bo->fw_va + 0x630,
			.target = j->cmd[index - 1].final_target,
			.event = 0x71, .strict = true, .relaxed = m4_user_prior_relax(j, index, 1),
		};
		return 0;
	}
	if (!n || !n->counter || !m4_user_native_compute(j->native_stage, index - 1) ||
	    !j->cmd[index - 1].published || !j->queue->notifier || !j->queue->notifier->bo)
		return M4_USER_WHY(-ESTALE, "prior %u native=%d counter=%d predecessor=%d published=%d notifier=%d",
			index, !!n, n && n->counter, !!m4_user_native_compute(j->native_stage, index - 1),
			j->cmd[index - 1].published, j->queue->notifier && j->queue->notifier->bo);
	*prior = (struct gpu_m4_compute_packet_dependency) {
		.owner = &j->compute[index - 1],
		.firmware = { n->counter->fw_va + GPU_M4_COMPUTE_PACKET_WAIT_OFFSET,
			GPU_M4_EXEC_DEPENDENCY_SIZE },
		.stamp_fw = j->queue->notifier->bo->fw_va + 0x640,
		.target = j->compute_reservation[index - 1].chronology.target_stamp,
		.event = 15, .strict = true, .relaxed = m4_user_prior_relax(j, index, 1),
	};
	return 0;
}

static bool m4_user_compute_prior_same(const struct gpu_m4_compute_packet_dependency *a,
	const struct gpu_m4_compute_packet_dependency *b)
{
	return a->owner == b->owner && a->firmware.va == b->firmware.va &&
		a->firmware.size == b->firmware.size && a->stamp_fw == b->stamp_fw &&
		a->target == b->target && a->event == b->event && a->strict == b->strict &&
		a->relaxed == b->relaxed;
}

/* Exact reservation of one command, either the next unpublished one being
 * prepared or an already reachable one. A reachable command's notifier
 * threshold, global target and sequence tail belong to the LAST reachable
 * command of this job (its published horizon), never to every earlier one.
 */
static int m4_user_compute_reservation_check_locked(struct m4_user_job *j,
	u32 index, const struct m4_user_compute_reservation *r)
{
	struct m4_gpu_provider_runtime *p = j->runtime;
	struct m4_user_notifier *n = j->queue->notifier;
	const struct m4_user_compute_native *native;
	const struct gpu_m4_compute_packet_chronology *c, *tail;
	struct gpu_m4_compute_packet_dependency prior;
	const struct m4_user_command *cmd;
	struct m4_ready_buffer *q;
	u64 threshold;
	u32 last;
	bool published;
	int ret;

	if (!j->compute_reservation || index >= j->command_count || index >= ARRAY_SIZE(j->cmd) ||
	    r != &j->compute_reservation[index] || !n || !refcount_read(&n->refs) ||
	    r->notifier != n->bo || j->native_stage->notifier != n || r->completion_record ||
	    (!!r->reuse != !!r->reuse_binding))
		return M4_USER_WHY(-ESTALE, "reservation %u/%u array=%d notifier=%d/%d/%d record=%u reuse=%d/%d",
			index, j->command_count, j->compute_reservation && r == &j->compute_reservation[index],
			n && refcount_read(&n->refs), n && r->notifier == n->bo,
			j->native_stage->notifier == n, r->completion_record, !!r->reuse, !!r->reuse_binding);
	cmd = &j->cmd[index];
	published = cmd->published;
	if ((published ? index >= j->published_count : index != j->published_count) ||
	    (cmd->receipt && !published) || j->published_count > j->command_count ||
	    j->completed_count > j->published_count || !j->depth ||
	    j->published_count - j->completed_count > j->depth ||
	    !m4_user_native_slot_valid(DRM_ASAHI_CMD_COMPUTE, cmd->uma_slot))
		return M4_USER_WHY(-ESTALE, "reservation %u published=%d receipt=%d window=%u/%u/%u depth=%u slot=%u",
			index, published, cmd->receipt, j->completed_count, j->published_count,
			j->command_count, j->depth, cmd->uma_slot);
	if (m4_user_native_reused(j->native_stage, index)) {
		struct m4_user_native_stage *origin, *displacer;
		const struct m4_user_native_receipt *receipt;
		u32 old_index, released_index;

		ret = m4_user_native_reuse_history_locked(j->native_stage, index,
			&origin, &old_index, &displacer, &released_index);
		if (!ret)
			ret = m4_user_native_history_command_locked(origin, old_index,
				DRM_ASAHI_CMD_COMPUTE, &receipt);
		if (ret)
			return M4_USER_WHY(ret, "reservation %u reuse history", index);
		if (!j->compute_reuse ||
		    r->reuse != &j->compute_reuse[index].input ||
		    r->reuse_binding != &j->compute_reuse[index].binding ||
		    receipt->submitted == U64_MAX ||
		    (receipt->uma_slot != cmd->uma_slot &&
		     !m4_user_native_retargeted(j->native_stage, index)) ||
		    r->chronology.submitted_before != receipt->submitted ||
		    r->chronology.completed_before != receipt->submitted ||
		    r->chronology.submitted != receipt->submitted + 1)
			return M4_USER_WHY(-ESTALE, "reservation %u warm: reuse=%d slot=%u/%u submitted %llu/%llu/%llu origin %llu",
				index, j->compute_reuse && r->reuse == &j->compute_reuse[index].input &&
				r->reuse_binding == &j->compute_reuse[index].binding,
				receipt->uma_slot, cmd->uma_slot, M4_WHY64(r->chronology.submitted_before),
				M4_WHY64(r->chronology.completed_before), M4_WHY64(r->chronology.submitted),
				M4_WHY64(receipt->submitted));
	} else if (r->reuse || r->reuse_binding || r->chronology.submitted_before ||
		   r->chronology.completed_before || r->chronology.submitted != 1) {
		return M4_USER_WHY(-ESTALE, "reservation %u fresh: reuse=%d/%d submitted %llu/%llu/%llu",
			index, !!r->reuse, !!r->reuse_binding, M4_WHY64(r->chronology.submitted_before),
			M4_WHY64(r->chronology.completed_before), M4_WHY64(r->chronology.submitted));
	}
	ret = m4_user_lane_check_locked(p, j->core_job);
	if (!ret)
		ret = m4_user_compute_prior_expected(j, index, &prior);
	if (ret)
		return M4_USER_WHY(ret, "reservation %u lane/prior", index);
	native = m4_user_native_compute(j->native_stage, index);
	if (!native || !native->counter)
		return M4_USER_WHY(-ESTALE, "reservation %u native=%d", index, !!native);
	q = p->execution->queue[2];
	c = &r->chronology;
	tail = c;
	last = index;
	if (published) {
		u32 between = 0;

		/* The newest reachable compute of this job: a mixed window's
		 * later renders own no compute sequence or target.
		 */
		last = j->published_count - 1;
		while (j->mixed && last > index && !m4_user_native_compute(j->native_stage, last))
			last--;
		if (!m4_user_native_compute(j->native_stage, last) || !j->cmd[last].published)
			return M4_USER_WHY(-ESTALE, "reservation %u tail %u native=%d published=%d", index,
				last, !!m4_user_native_compute(j->native_stage, last), j->cmd[last].published);
		for (u32 i = index + 1; i <= last; i++)
			between += !!m4_user_native_compute(j->native_stage, i);
		tail = &j->compute_reservation[last].chronology;
		/* One sequence and one 0x100 target per reachable compute command. */
		if (tail->threshold_to_publish - c->threshold_to_publish != between ||
		    tail->target_stamp - c->target_stamp != between * 0x100U)
			return M4_USER_WHY(-ESTALE, "reservation %u tail %u threshold %llu/%llu target %#x/%#x",
				index, last, M4_WHY64(c->threshold_to_publish),
				M4_WHY64(tail->threshold_to_publish), c->target_stamp, tail->target_stamp);
	}
	threshold = published ? tail->threshold_to_publish : c->threshold_before;
	if (c->binding_generation != j->binding_generation || c->uma_slot != cmd->uma_slot ||
	    c->outstanding >= j->depth || c->outstanding > index ||
	    c->threshold_before != c->event_sequence ||
	    c->threshold_to_publish != c->event_sequence + 1 ||
	    /* Reachable: this job's tail, or a later same-queue job's. */
	    (published ? !m4_user_wire_notifier_tail(j, n->sequence[2], threshold, 1) :
		n->sequence[2] != threshold) ||
	    n->threshold[2] != n->sequence[2] ||
	    m4_store_read64(n->bo->map.vaddr + 0x610) != n->sequence[2] ||
	    r->queue_fw != q[M4_EXEC_QUEUE_INFO].va ||
	    r->stats_fw != p->ready->buffer[GPU_M4_ACTIVITY_COMPUTE].va ||
	    c->target_stamp != c->global_previous_target + 0x100 ||
	    (published ? !m4_user_wire_target_tail(j, 2, tail->target_stamp) :
		p->user_target[2] != c->global_previous_target) ||
	    !m4_user_compute_prior_same(&r->prior, &prior))
		return M4_USER_WHY(-ESTALE, "reservation %u tail %u: gen=%llu/%u slot=%u/%u outstanding=%u depth=%u seq=%llu thr=%llu/%llu want %llu notifier=%llu/%llu/%llu queue=%d stats=%d target=%#x prev=%#x global=%#x prior=%d",
			index, last, M4_WHY64(c->binding_generation), j->binding_generation, c->uma_slot,
			cmd->uma_slot, c->outstanding, j->depth, M4_WHY64(c->event_sequence),
			M4_WHY64(c->threshold_before), M4_WHY64(c->threshold_to_publish), M4_WHY64(threshold),
			M4_WHY64(n->sequence[2]), M4_WHY64(n->threshold[2]),
			M4_WHY64(m4_store_read64(n->bo->map.vaddr + 0x610)),
			r->queue_fw == q[M4_EXEC_QUEUE_INFO].va,
			r->stats_fw == p->ready->buffer[GPU_M4_ACTIVITY_COMPUTE].va, c->target_stamp,
			c->global_previous_target, p->user_target[2],
			m4_user_compute_prior_same(&r->prior, &prior));
	return 0;
}

/* Published, unreceipted compute commands of this job: its notifier's
 * compute sequences still outstanding (a mixed window's renders own none).
 */
static u32 m4_user_outstanding_compute(const struct m4_user_job *j)
{
	u32 count = 0;

	if (!j->mixed)
		return j->published_count - j->completed_count;
	for (u32 i = j->completed_count; i < j->published_count && i < ARRAY_SIZE(j->cmd); i++)
		count += j->cmd[i].engine_mask == BIT(2);
	return count;
}

static int m4_user_compute_packet_publish_locked(struct m4_user_job *j, u32 index)
{
	struct m4_gpu_provider_runtime *p = j->runtime;
	struct m4_user_notifier *n = j->queue->notifier;
	const struct m4_user_compute_native *native = m4_user_native_compute(j->native_stage, index);
	struct m4_user_compute_reservation *r = &j->compute_reservation[index];
	struct m4_user_compute_packet *packet = &j->compute[index];
	struct m4_user_command *cmd = &j->cmd[index];
	struct m4_ready_buffer *q = p->execution->queue[2];
	void *pipe = p->ready->buffer[GPU_M4_OBJECT_COUNT + 4].cpu;
	const struct gpu_m4_compute_packet_publication *pub;
	u64 entry, entries[2];
	int ret;

	if (!native || index >= ARRAY_SIZE(j->cmd) || index != j->published_count ||
	    cmd->published || cmd->receipt || cmd->engine_mask ||
	    !m4_user_native_slot_valid(DRM_ASAHI_CMD_COMPUTE, cmd->uma_slot))
		return M4_USER_WHY(-ESTALE, "compute publish %u native=%d published=%u/%d receipt=%d engines=%#x slot=%u",
			index, !!native, j->published_count, cmd->published, cmd->receipt,
			cmd->engine_mask, cmd->uma_slot);
	if (n->sequence[2] >= U32_MAX || p->user_target[2] >= 0xfffffe00U)
		return M4_USER_WHY(-EOVERFLOW, "compute publish %u sequence=%llu target=%#x", index,
			M4_WHY64(n->sequence[2]), p->user_target[2]);
	*r = (struct m4_user_compute_reservation) {
		.notifier = n->bo, .queue_fw = q[M4_EXEC_QUEUE_INFO].va,
		.stats_fw = p->ready->buffer[GPU_M4_ACTIVITY_COMPUTE].va,
		.timestamp = __arch_counter_get_cntpct(),
		.chronology = {
			.binding_generation = j->binding_generation, .asid = j->asid,
			.uma_slot = cmd->uma_slot,
			.event_sequence = n->sequence[2], .threshold_before = n->threshold[2],
			.threshold_to_publish = n->sequence[2] + 1, .submitted = 1,
			.global_previous_target = p->user_target[2],
			.target_stamp = p->user_target[2] + 0x100,
			/* Live observation: an outstanding predecessor is NOT complete. */
			.selected_stamp_before = m4_ready_read32(n->bo->map.vaddr + 0x640),
			.queue_read = m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x30),
			.queue_write = m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x40),
			.pipe_read = m4_ready_read32(pipe), .pipe_cfi = m4_ready_read32(pipe + 0x10),
			.pipe_write = m4_ready_read32(pipe + 0x20),
			.outstanding = m4_user_outstanding_compute(j),
			.new_queue = !(p->user_physical_used & BIT(2)),
		},
	};
	for (u32 i = 0; i < 3; i++)
		r->chronology.queue_consumer[i] = m4_ready_read32(q[M4_EXEC_QUEUE_INFO].cpu + 0x20 + 4 * i);
	ret = m4_user_compute_prior_expected(j, index, &r->prior);
	if (!ret)
		ret = m4_user_compute_reuse_prepare_locked(j, index, r);
	if (ret)
		return M4_USER_WHY(ret, "compute publish %u prior/reuse warm=%d slot=%u", index,
			m4_user_native_reused(j->native_stage, index), cmd->uma_slot);
	if (m4_store_read64(native->counter->map.vaddr) != r->chronology.submitted_before)
		return M4_USER_WHY(-EBUSY, "compute publish %u counter=%llu want %llu", index,
			M4_WHY64(m4_store_read64(native->counter->map.vaddr)),
			M4_WHY64(r->chronology.submitted_before));
	ret = m4_user_compute_prepare_locked(j, index, r, packet);
	if (ret)
		return M4_USER_WHY(ret, "compute publish %u prepare warm=%d slot=%u", index,
			m4_user_native_reused(j->native_stage, index), cmd->uma_slot);
	pub = &packet->bytes.publication;
	entry = native->private[GPU_M4_COMPUTE_PRIVATE_WORK]->fw_va;
	if (pub->queue_entries != (gpu_m4_compute_packet_has_prior(&r->prior) ? 2U : 1U))
		return M4_USER_WHY(-ESTALE, "compute publish %u entries=%u prior=%d", index,
			pub->queue_entries, gpu_m4_compute_packet_has_prior(&r->prior));
	for (u32 i = 0; i < pub->queue_entries; i++)
		entries[i] = get_unaligned_le64(pub->ring_entries + i * 8);
	if (entries[pub->queue_entries - 1] != entry ||
	    (pub->queue_entries == 2 && entries[0] != r->prior.firmware.va))
		return M4_USER_WHY(-ESTALE, "compute publish %u ring %#llx,%#llx want work %#llx wait %#llx",
			index, M4_WHY64(entries[0]), M4_WHY64(entries[pub->queue_entries - 1]),
			M4_WHY64(entry), M4_WHY64(r->prior.firmware.va));
	ret = m4_user_wire_reserve_locked(j, index, 2, entries, pub->queue_entries);
	if (!ret && (cmd->engine_mask != BIT(2) ||
	    pub->queue_before != cmd->wire[2].queue_before ||
	    pub->queue_next != cmd->wire[2].queue_next ||
	    pub->pipe_before != cmd->wire[2].pipe_before ||
	    pub->pipe_next != cmd->wire[2].pipe_next || pub->uma_slot != cmd->uma_slot))
		ret = M4_USER_WHY(-ESTALE, "compute publish %u packet/wire engines=%#x queue %u-%u/%u-%u pipe %u-%u/%u-%u slot=%u/%u",
			index, cmd->engine_mask, pub->queue_before, pub->queue_next,
			cmd->wire[2].queue_before, cmd->wire[2].queue_next, pub->pipe_before,
			pub->pipe_next, cmd->wire[2].pipe_before, cmd->wire[2].pipe_next,
			pub->uma_slot, cmd->uma_slot);
	if (!ret)
		ret = m4_user_wire_check_locked(j, index, 2);
	/* Shared metadata index 0 (work +0x818) and the per-work FW/USER stamp
	 * cells of the one held notifier serve every chained command.
	 */
	if (!ret && get_unaligned_le32(packet->bytes.work + 0x818))
		ret = M4_USER_WHY(-ESTALE, "compute publish %u metadata index %u", index,
			get_unaligned_le32(packet->bytes.work + 0x818));
	/* After protection, BEFORE any write: this actual job's cache/resv
	 * covers the wait subrange of the combined counter/wait page.
	 */
	if (!ret && pub->queue_entries == 2)
		ret = native->counter->coverage <
			GPU_M4_COMPUTE_PACKET_WAIT_OFFSET + GPU_M4_EXEC_DEPENDENCY_SIZE ? -ESTALE :
			gpu_m4_drm_provider_job_check_access_locked(j->core_job, p->provider,
				native->counter->object,
				native->counter->backing.offset + GPU_M4_COMPUTE_PACKET_WAIT_OFFSET,
				GPU_M4_EXEC_DEPENDENCY_SIZE, M4_USER_NATIVE_RW, false, NULL);
	if (ret)
		return M4_USER_WHY(ret, "compute publish %u wire/wait coverage=%#llx", index,
			M4_WHY64(native->counter->coverage));
	/* Actual command anchor, not the pure encoder's temporary owner value. */
	put_unaligned_le64(entry, packet->bytes.work + 0xe00);
	memcpy(native->private[GPU_M4_COMPUTE_PRIVATE_WORK]->map.vaddr,
		packet->bytes.work, sizeof(packet->bytes.work));
	/* The strict wait record is owned, covered NC firmware memory of THIS
	 * command, complete before any producer can name it. The host never
	 * rewrites it until this unit's real CORE retirement and same-slot
	 * displacement admit a new use.
	 */
	if (pub->queue_entries == 2)
		memcpy(native->counter->map.vaddr + GPU_M4_COMPUTE_PACKET_WAIT_OFFSET,
			packet->bytes.prior_dependency, GPU_M4_EXEC_DEPENDENCY_SIZE);
	m4_user_native_stage_exposed_locked(j->native_stage);
	j->published = true;
	cmd->published = true;
	cmd->final_target = r->chronology.target_stamp;
	j->published_count++;
	/* Submitted/threshold precede BOTH producer levels. The descriptor and
	 * slot travel only in the work (+0x86a, descriptor +8): the global UMA
	 * table is firmware-owned and never written by the host. The threshold
	 * names this newest reserved sequence; used stamps are kept.
	 */
	put_unaligned_le64(r->chronology.submitted, native->counter->map.vaddr);
	put_unaligned_le64(r->chronology.threshold_to_publish, n->bo->map.vaddr + 0x610);
	n->sequence[2] = n->threshold[2] = r->chronology.threshold_to_publish;
	n->published_target[2] = r->chronology.target_stamp;
	p->user_target[2] = r->chronology.target_stamp;
	j->all_engines |= BIT(2);
	dma_wmb();
	return m4_user_wire_publish_locked(j, index);
}

/* Full per-command hardware predicate, tolerant of published successors:
 * consumption THROUGH the command's saved prefix within the owned physical
 * horizon, shared stamps reached within the notifier's published tail (each
 * half exact when it has no successor), and exact own
 * UMA slot/descriptor/submitted/completed/timestamp state. Never a CORE
 * receipt: cache/version retirement waits for the whole child.
 */
static int m4_user_compute_packet_retired_locked(struct m4_user_job *j, u32 index)
{
	struct m4_gpu_provider_runtime *p = j->runtime;
	const struct m4_user_compute_native *n = m4_user_native_compute(j->native_stage, index);
	const struct m4_user_compute_reservation *r = &j->compute_reservation[index];
	struct m4_user_native_history *history = j->native_stage->pending;
	struct gpu_m4_exec_retirement observed = { };
	struct m4_ready_buffer *queue = p->execution->queue[2];
	void *pipe = p->ready->buffer[GPU_M4_OBJECT_COUNT + 4].cpu;
	const struct m4_user_engine *horizon, *own;
	void *work, *notifier;
	u64 completed;
	u32 slot;
	int ret = m4_user_compute_reservation_check_locked(j, index, r);

	if (!ret && (!history || history->job != j->core_job || history->complete ||
	    index >= history->count || history->command[index].type != DRM_ASAHI_CMD_COMPUTE ||
	    index != j->completed_count || !j->cmd[index].published || j->cmd[index].receipt))
		ret = M4_USER_WHY(-ESTALE, "compute retire %u pending=%d job=%d complete=%d count=%u type=%u completed=%u published=%d receipt=%d",
			index, !!history, history && history->job == j->core_job,
			history && history->complete, history ? history->count : 0,
			history && index < history->count ? history->command[index].type : 0xffff,
			j->completed_count, j->cmd[index].published, j->cmd[index].receipt);
	if (!ret)
		ret = m4_user_wire_horizon_locked(j, 2, &horizon);
	if (ret)
		return M4_USER_WHY(ret, "compute retire %u reservation/horizon", index);
	own = &j->cmd[index].wire[2];
	slot = j->cmd[index].uma_slot;
	work = n->private[GPU_M4_COMPUTE_PRIVATE_WORK]->map.vaddr;
	notifier = j->queue->notifier->bo->map.vaddr;
	/* Own descriptor (+0 owner, +8 slot); the global table is not consulted. */
	if (m4_store_read64(work + 0xe00) != n->private[GPU_M4_COMPUTE_PRIVATE_WORK]->fw_va ||
	    m4_ready_read32(work + 0xe08) != slot ||
	    READ_ONCE(*(u8 *)(work + 0x88b)) != j->binding_generation)
		return M4_USER_WHY(-ESTALE, "compute retire %u slot %u: owner=%#llx want %#llx work slot=%u gen=%u/%u",
			index, slot, M4_WHY64(m4_store_read64(work + 0xe00)),
			M4_WHY64(n->private[GPU_M4_COMPUTE_PRIVATE_WORK]->fw_va),
			m4_ready_read32(work + 0xe08), READ_ONCE(*(u8 *)(work + 0x88b)),
			j->binding_generation);
	ret = m4_store_packed64(work + 0xe54, &completed);
	if (ret)
		return M4_USER_WHY(ret, "compute retire %u completed", index);
	observed = (struct gpu_m4_exec_retirement) {
		.expected_stamp = r->chronology.target_stamp,
		.firmware_stamp = m4_ready_read32(notifier + 0x640),
		.user_stamp = m4_ready_read32(notifier + 0x648),
		.next = own->queue_next,
		.queue_read = m4_ready_read32(queue[M4_EXEC_QUEUE_STATE].cpu + 0x30),
		.queue_write = m4_ready_read32(queue[M4_EXEC_QUEUE_STATE].cpu + 0x40),
		.queue_busy = m4_ready_read32(queue[M4_EXEC_QUEUE_INFO].cpu + 0x80),
		.pipe_read = m4_ready_read32(pipe), .pipe_cfi = m4_ready_read32(pipe + 0x10),
		.pipe_write = m4_ready_read32(pipe + 0x20), .pipe_next = own->pipe_next,
		.pipe_capacity = 256, .uma_submitted = m4_store_read64(n->counter->map.vaddr),
		.uma_completed = completed,
		.timestamp_start = m4_store_read64(n->counter->map.vaddr + GPU_M4_EXEC_COMPUTE_STAMPS),
		.timestamp_end = m4_store_read64(n->counter->map.vaddr + GPU_M4_EXEC_COMPUTE_STAMPS + 8),
		.ignore_busy = m4_user_wire_overlapped(j),
	};
	for (u32 i = 0; i < 3; i++)
		observed.queue_consumer[i] = m4_ready_read32(queue[M4_EXEC_QUEUE_INFO].cpu + 0x20 + i * 4);
	if (observed.uma_submitted != r->chronology.submitted)
		return M4_USER_WHY(-ESTALE, "compute retire %u submitted=%llu want %llu", index,
			M4_WHY64(observed.uma_submitted), M4_WHY64(r->chronology.submitted));
	/* Physical: the ledger's last reachable compute record. Notifier: this
	 * notifier's newest published compute target.
	 */
	ret = gpu_m4_compute_horizon_retired(&observed, horizon->queue_next, horizon->pipe_next,
		j->queue->notifier->published_target[2]);
	/* REMOVE-DEBUG: the pure predicate's inputs, against its horizon. */
	(void)M4_USER_WHY(ret, "compute horizon %u seq %llu/%llu queue %u/%u/%u-%u r=%u c=%u,%u,%u busy=%u pipe %u/%u-%u r=%u cfi=%u stamp %#x,%#x want %#x tail %#x uma %llu/%llu time %#llx,%#llx",
		index, M4_WHY64(own->sequence), M4_WHY64(horizon->sequence), observed.queue_write,
		own->queue_before, observed.next, horizon->queue_next,
		observed.queue_read, observed.queue_consumer[0], observed.queue_consumer[1],
		observed.queue_consumer[2], observed.queue_busy, observed.pipe_write,
		observed.pipe_next, horizon->pipe_next, observed.pipe_read, observed.pipe_cfi,
		observed.firmware_stamp, observed.user_stamp, observed.expected_stamp,
		j->queue->notifier->published_target[2],
		M4_WHY64(observed.uma_submitted), M4_WHY64(observed.uma_completed),
		M4_WHY64(observed.timestamp_start), M4_WHY64(observed.timestamp_end));
	/* Raw observations, including the horizon they were taken within. */
#ifdef CONFIG_DEBUG_FS
	if (!ret)
		gpu_m4_trace(GPU_M4_TRACE_COMPUTE, index, observed.timestamp_start,
			observed.timestamp_end, (u64)(uintptr_t)j->queue, 0);
#endif
	if (!ret)
		history->command[index] = (struct m4_user_native_receipt) {
			.type = DRM_ASAHI_CMD_COMPUTE, .uma_slot = slot,
			.submitted = observed.uma_submitted, .completed = observed.uma_completed,
			.compute = observed,
		};
	return ret;
}

/* REMOVE-DEBUG: cumulative lane-start phase timing (ns), observational only,
 * under the GPU mutex. Exported through runtime_perf.
 */
enum m4_user_start_phase {
	M4_USER_START_IDLE, M4_USER_START_RECEIPT, M4_USER_START_BIND,
	M4_USER_START_ARM, M4_USER_START_TOKEN, M4_USER_START_BEGIN,
	M4_USER_START_PACKET, M4_USER_START_PHASES,
};
static struct {
	u64 count, ns, max;
} m4_user_start_perf[M4_USER_START_PHASES];

/* REMOVE-DEBUG: window publication ahead of an unreceipted predecessor
 * ([0] compute, [1] render) and the largest outstanding command count.
 */
static struct {
	u64 ahead[2], outstanding_max;
} m4_user_window_stats;

/* REMOVE-DEBUG Stage 0: of the activations the lane counted as READY behind
 * an ACTIVE head, whether same_image(resident, candidate) held at this job's
 * own bind (resident or same-image adoption) or it needed a root install.
 * The resident then is exactly the predecessor's retired image.
 */
static struct {
	u64 same_image, rebind;
} m4_user_overlap_stats;

static u64 m4_user_start_account(enum m4_user_start_phase phase, u64 start)
{
	u64 now = ktime_get_ns(), delta = now - start;

#ifdef CONFIG_DEBUG_FS
	gpu_m4_trace(GPU_M4_TRACE_PHASE, 20 + phase, 0, 0, 0, delta);
#endif
	m4_user_start_perf[phase].count++;
	m4_user_start_perf[phase].ns += delta;
	if (delta > m4_user_start_perf[phase].max)
		m4_user_start_perf[phase].max = delta;
	return now;
}

/* REMOVE-DEBUG: starts admitted while some engine was not idle. The retired
 * global gate, observed only (soak), never refused.
 */
static u64 m4_user_idle_soak_busy;

/* REMOVE-DEBUG: service health cadence (health_per_job=0), observational
 * only, under the GPU mutex: service snapshots taken (and their time) and
 * reused inside one lane-worker hold, scalar publication gates, upper graph
 * passes at publication run at a changed upper revision or skipped at an
 * unchanged one, and READY drain-worker notification passes that made the
 * bounded live service observation instead of the complete live READY pass.
 */
static struct {
	u64 snapshots, reused, snapshot_ns, gates, upper_runs, upper_skips, notify_light, fresh;
} m4_user_service_stats;

/* Latched failures only, no MMIO: owner, READY and callback errors, RTKit
 * running/not crashed (host flags), the provider core's and the lane's
 * latched errors. The working runtime's "health latch" before publication.
 */
static int m4_user_latched_locked(struct m4_gpu_provider_runtime *p)
{
	struct m4_gpu_hello *h = p->hello;
	int ret = p->error ?: h->ready->error;

	if (!ret && (atomic_read(&h->callback_error) || !apple_rtkit_is_running(h->rtkit) ||
		     apple_rtkit_is_crashed(h->rtkit)))
		ret = -EIO;
	if (!ret && p->provider)
		ret = gpu_m4_drm_provider_error_locked(p->provider);
	if (!ret && p->user_lane)
		ret = gpu_m4_drm_provider_lane_error_locked(p->user_lane);
	return ret;
}

/* Retirement progress (a command receipt) voids the hold's snapshot: the
 * next service of the same hold resnapshots, as the completion worker does.
 */
static void m4_user_service_void(struct m4_gpu_provider_runtime *p)
{
	p->service_hold = 0;
}

/* The lane-worker service snapshot: drained READY rings (fault, unexpected
 * event and recovery statistics records latch here), the bounded live
 * session (latched errors, RTKit, ASC RUN, the actual SGX state within the
 * configured S after any due thermal ceiling service, TTBAT[0/1]), Control
 * READY and the execution owner. No table, graph or TTBAT slot walk: those
 * are host-written and verified where the host changes them (binding
 * install/release, upper commit/release), at fault/TDR boundaries and by the
 * READY owner's periodic full scan. With health_per_job=0 it is taken once
 * per lane-worker hold (gpu_m4_drm_provider_lane_hold_locked()) and reused
 * by every later start/poll of that hold, each reuse rechecking the latched
 * failures, until retirement progress voids it. Any other holder (TDR
 * service, ioctls) and health_per_job=1 take it on every call. A failure is
 * never remembered.
 */
static int m4_user_service_locked(struct m4_gpu_provider_runtime *p)
{
	struct m4_gpu_hello *h = p->hello;
	u64 hold = 0, start;
	int ret;

	if (!READ_ONCE(health_per_job) && p->user_lane)
		hold = gpu_m4_drm_provider_lane_hold_locked(p->user_lane);
	if (hold && p->service_hold == hold) {
		ret = m4_user_latched_locked(p);
		if (!ret) {
			m4_user_service_stats.reused++;
			return 0;
		}
		p->service_hold = 0;
		return ret;
	}
	p->service_hold = 0;
	start = ktime_get_ns();
	ret = m4_ready_drain(h);
	if (!ret)
		ret = m4_user_live_health_locked(p);
	if (!ret)
		ret = m4_store_control(h);
	if (!ret && !h->ready->execution)
		ret = -EPROTO;
	m4_user_service_stats.snapshots++;
	m4_user_service_stats.snapshot_ns += ktime_get_ns() - start;
	if (!ret) {
		p->service_hold = hold;
		p->service_ns = start;
	}
	return ret;
}

/* A still-pending poll inside the lane worker re-takes the live MMIO
 * snapshot (RUN, TTBAT, SGX state, thermal service) at most once per
 * poll_health_us; between snapshots it checks only the latched failures
 * (callback/RTKit/lane/provider errors). Any observed progress, every start
 * and every receipt still take the complete snapshot first. 0: every poll.
 */
static unsigned int poll_health_us = 1000;
#ifdef MODULE
module_param(poll_health_us, uint, 0644);
MODULE_PARM_DESC(poll_health_us, "Live snapshot cadence of still-pending lane polls in us (default 1000; 0 = every poll)");
#endif

static bool m4_user_poll_fresh(const struct m4_gpu_provider_runtime *p)
{
	u32 us = READ_ONCE(poll_health_us);

	return us && p->service_ns && ktime_get_ns() - p->service_ns < (u64)us * 1000;
}

/* Scalar gate before each publication scope (health_per_job=0): the latched
 * failures above, then the actual SGX state against the configured S through
 * the actual sampler, which first services a due thermal sample and publishes
 * a changed ceiling (gpu-m4-ready-pstate.h): a lowered ceiling still reaches
 * Globals before this work. One register read; no READY, table or TTBAT pass.
 */
static int m4_user_publish_gate_locked(struct m4_gpu_provider_runtime *p)
{
	int ret = m4_user_latched_locked(p);

	m4_user_service_stats.gates++;
	if (!ret && m4_pstate_sample(p->hello, M4_PSTATE_POLL) > p->hello->ready->pstate.configured)
		ret = -EIO;
	return ret;
}

/* The upper graph/lease pass before a publication arms and publishes its
 * leases: health_per_job=0 runs it only when the upper owner revision moved
 * since the last successful pass here (a lease, table or ledger store: commit,
 * release, new publication). An unchanged published lease's own proof is
 * still rechecked by gpu_m4_drm_upper_publish_locked() against the owner's
 * last checked revision and live root, falling back to its own slot walk.
 */
static int m4_user_upper_health_locked(struct m4_gpu_provider_runtime *p)
{
	struct gpu_m4_drm_upper *upper = p->ready->upper;
	u64 revision = gpu_m4_drm_upper_revision_locked(upper);
	int ret;

	if (!READ_ONCE(health_per_job) && revision && p->upper_health_revision == revision) {
		m4_user_service_stats.upper_skips++;
		return 0;
	}
	p->upper_health_revision = 0;
	ret = gpu_m4_drm_upper_health_locked(upper);
	m4_user_service_stats.upper_runs++;
	if (!ret)
		p->upper_health_revision = revision;
	return ret;
}

/* READY drain-worker notification passes of the ordinary runtime with
 * health_per_job=0: the bounded live service only (the lane worker snapshots
 * its own service); the complete pass keeps its periodic cadence.
 */
static bool m4_ready_notify_light(struct m4_gpu_hello *h)
{
	struct m4_gpu_provider_runtime *p = h->ready->provider_runtime;

	if (READ_ONCE(health_per_job) || !p || !p->ordinary || !p->complete)
		return false;
	m4_user_service_stats.notify_light++;
	return true;
}

/* Start admission under the lane: the service snapshot (drained READY rings,
 * the bounded live session, Control READY and the execution owner), then
 * per-ASID authority: the job's VM holds its user context and that context
 * alone has no pending installation or working resident. Other contexts need
 * not be idle (contract Part II); a binding installation runs the complete
 * READY pass, publication its scalar gate. Engine idle is observed only.
 */
/* inflight2 co-residency: j's VM is a tree authority whose one stable TREE
 * version (j's version) is already the installed resident of j's own user
 * context. Its jobs then share that binding and generation with no TTBAT
 * write, TLBI or rebind, whatever else runs: the same VM always co-resides;
 * another VM co-resides only on its own bound context. A shared ASID 1 that
 * names another VM's root (or any image change) is never co-resident.
 */
static bool m4_user_coresident_locked(struct m4_gpu_provider_runtime *p, struct m4_user_job *j)
{
	struct gpu_m4_drm_provider_job_view view = { };
	const struct m4_user_asid *a;

	if (!p->user_inflight2 || !j->asid || j->asid >= M4_USER_ASIDS ||
	    j->asid != j->queue->vm->asid ||
	    !gpu_m4_drm_provider_vm_authority(j->queue->vm->provider_vm))
		return false;
	a = &p->user_asid[j->asid];
	gpu_m4_drm_provider_job_view(j->core_job, &view);
	return view.version && a->resident == view.version && a->low && a->leases &&
		!a->quarantined && a->owner == (j->asid == M4_USER_ASID_SHARED ? NULL : j->queue->vm);
}

static int m4_user_admit_locked(struct m4_gpu_provider_runtime *p, struct m4_user_job *j)
{
	const struct m4_user_asid *a;
	int ret = m4_user_service_locked(p);

	if (ret)
		return ret;
	if (!j->asid || j->asid >= M4_USER_ASIDS || j->asid != j->queue->vm->asid)
		return -ESTALE;
	a = &p->user_asid[j->asid];
	if (!a->leases || a->quarantined ||
	    a->owner != (j->asid == M4_USER_ASID_SHARED ? NULL : j->queue->vm))
		return -EPERM;
	ret = gpu_m4_drm_provider_asid_quiescent_locked(p->provider, j->asid);
	/* A working resident is this VM's own stable root: co-reside. */
	if (ret == -EBUSY && m4_user_coresident_locked(p, j))
		ret = 0;
	if (ret)
		return ret;
	if (m4_provider_engines_idle(p->hello) && !m4_user_wire_overlapped(j)) {
		m4_user_idle_soak_busy++;
		dev_warn_once(p->hello->gpu->dev, "REMOVE-DEBUG GPU user start: an engine is not idle (soak only, asid=%u)\n",
			      j->asid);
	}
	return 0;
}

/* inflight2 global-tail barrier: name the last command of the newest job
 * admitted ahead of j (the lane admits j only once every in-flight job
 * published its whole window, in admission order) and hold its notifier.
 * Each admitted job's first command waits on its own predecessor's tail, so
 * the newest one's last command completes after every earlier in-flight
 * command. A head has no predecessor in flight.
 */
static int m4_user_cross_locked(struct m4_gpu_provider_runtime *p, struct m4_user_job *j)
{
	struct gpu_m4_drm_provider_job_view view = { };
	struct m4_user_job *a = NULL;
	struct m4_user_notifier *n;
	u32 last;
	int ret;

	if (j->cross.notifier || j->published)
		return -ESTALE;
	if (!p->user_inflight2)
		return 0;
	for (u32 i = 0; i < M4_USER_INFLIGHT_MAX; i++) {
		const struct m4_user_inflight *o = &p->user_inflight[i];

		if (!o->item || !o->job || o->job == j->core_job)
			continue;
		/* user_inflight[] is in admission order: the last one wins. */
		a = m4_user_lane_job_locked(p, o->job);
		if (IS_ERR(a))
			return PTR_ERR(a);
	}
	if (!a)
		return 0;
	n = a->queue->notifier;
	last = a->published_count - 1;
	/* Its newest reachable command: the whole window, or (another queue)
	 * the published prefix of one still publishing.
	 */
	if (!a->published || !a->published_count || a->published_count > a->command_count ||
	    (a->published_count != a->command_count && a->queue == j->queue) ||
	    a->wire_open || last >= ARRAY_SIZE(a->cmd) || !a->cmd[last].published ||
	    !n || !n->bo || (a->cmd[last].engine_mask != 3 && a->cmd[last].engine_mask != BIT(2)))
		return M4_USER_WHY(-ESTALE, "cross predecessor published=%u/%u open=%u engines=%#x",
			a->published_count, a->command_count, a->wire_open,
			a->published_count && a->published_count <= ARRAY_SIZE(a->cmd) ?
			a->cmd[a->published_count - 1].engine_mask : 0);
	gpu_m4_drm_provider_job_view(j->core_job, &view);
	/* Admission never puts a render first command behind a compute tail
	 * (inflight2_mixed: its TA waits on the compute stamp).
	 */
	if (!view.batch || !view.batch->count ||
	    (!READ_ONCE(inflight2_mixed) && view.batch->commands[0].type == DRM_ASAHI_CMD_RENDER &&
	     a->cmd[last].engine_mask != 3))
		return M4_USER_WHY(-EPROTO, "cross render behind compute tail");
	ret = m4_user_notifier_get_locked(n);
	if (ret)
		return ret;
	if (a->cmd[last].engine_mask == 3) {
		struct gpu_m4_drm_provider_job_view av = { };

		gpu_m4_drm_provider_job_view(a->core_job, &av);
		j->cross = (struct m4_user_cross) {
			.untimed = m4_user_prior_relax(a, last + 1, 15),
			.notifier = n, .stamp_fw = n->bo->fw_va + 0x630, .event = 0x71, .engine = 1,
			.target = a->render_reservation[last].chronology.target_stamp,
			.same_packet = a->queue == j->queue && view.window_parent &&
				av.window_parent == view.window_parent &&
				a->published_count == a->command_count &&
				av.command_first + a->command_count == view.command_first,
		};
	}
	else
		j->cross = (struct m4_user_cross) {
			.untimed = m4_user_prior_relax(a, last + 1, 15),
			.notifier = n, .stamp_fw = n->bo->fw_va + 0x640, .event = 15, .engine = 2,
			.target = a->compute_reservation[last].chronology.target_stamp,
		};
	p->user_inflight2_stats.barrier[j->cross.event == 15]++; /* REMOVE-DEBUG */
	return 0;
}

static int m4_user_prepare_locked(void *owner, struct gpu_m4_drm_provider_job *job)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_user_job *j;
	int ret = m4_user_lane_acquire_locked(p, job);
	u64 start;

	if (ret)
		return ret;
	j = m4_user_lane_job_locked(p, job);
	if (IS_ERR(j))
		return PTR_ERR(j);
	start = ktime_get_ns();
	ret = m4_user_admit_locked(p, j);
	if (!ret)
		ret = m4_user_cross_locked(p, j);
	m4_user_start_account(M4_USER_START_IDLE, start);
	if (!ret)
		j->prepared = true;
	return ret;
}

/* inflight2 lane admission of READY candidate beside the in-flight job(s),
 * read-only under the GPU mutex. Each rule in order; the first veto stops
 * (the lane keeps the candidate and everything behind it for a later pass):
 * - every in-flight job published its whole window, so the global tail its
 *   first command will wait on is final (a packet publishes whole);
 * - another logical queue (behind the published prefix of a window still
 *   publishing when that cannot interleave its later commands), or the same
 *   one behind its whole published window (the lane already ordered it
 *   after every earlier item of its queue);
 * - its UMA slot bank, held since acceptance, is no in-flight job's bank;
 * - it co-resides on its own context (m4_user_coresident_locked());
 * - a render first command never waits on a compute tail (independent
 *   target counters; a compute first command may wait on either class);
 * - its whole window fits the work rings (fewer than 128 outstanding
 *   against all four consumers, the publication window rules) and the
 *   submit pipes (READ/CFI room) of every engine it uses, behind every
 *   in-flight job's reachable records.
 */
static int m4_user_inflight2_admit_locked(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_lane_item *candidate)
{
	struct m4_user_inflight2_stats *st = &p->user_inflight2_stats;
	struct gpu_m4_drm_provider_job_view view = { };
	struct m4_user_job *j = m4_user_lane_job_locked(p, candidate->job), *a;
	u32 entries[3] = { }, records[3] = { }, veto = M4_USER_VETO_STATE;
	bool same_vm = false, compute_tail = false, same_queue = false, render_only = true;
	bool partial = false;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (IS_ERR(j) || &j->item != candidate || !j->bank_held || j->prepared || j->published)
		goto vetoed;
	gpu_m4_drm_provider_job_view(j->core_job, &view);
	if (!view.batch || view.batch->count != j->command_count)
		goto vetoed;
	{
		u32 cap = clamp_t(u32, READ_ONCE(inflight_jobs), 2, p->user_inflight_max), n = 0;

		for (u32 i = 0; i < M4_USER_INFLIGHT_MAX; i++)
			n += p->user_inflight[i].item && p->user_inflight[i].job &&
				p->user_inflight[i].job != j->core_job;
		if (n >= cap) {
			veto = M4_USER_VETO_DEPTH;
			goto vetoed;
		}
	}
	for (u32 i = 0; i < M4_USER_INFLIGHT_MAX; i++) {
		const struct m4_user_inflight *o = &p->user_inflight[i];

		if (!o->item && !o->job)
			continue;
		a = m4_user_lane_job_locked(p, o->job);
		if (IS_ERR(a) || o->item != &a->item || a == j) {
			veto = M4_USER_VETO_STATE;
			goto vetoed;
		}
		if (!a->published || !a->published_count || a->wire_open) {
			veto = M4_USER_VETO_PUBLISHING;
			goto vetoed;
		}
		if (a->published_count != a->command_count) {
			struct gpu_m4_drm_provider_job_view av = { };
			u32 rest = 0, mine = 0;

			/* Still publishing: never behind its own queue (its later
			 * commands come first). Another queue may follow its
			 * published prefix: a serial (depth 1) window carries no
			 * in-window edge or contiguous target run, so its later
			 * commands may land behind ours; a deeper one only when
			 * we touch none of the engine classes it still publishes.
			 */
			gpu_m4_drm_provider_job_view(a->core_job, &av);
			if (!p->user_inflight2_wide || a->queue == j->queue || !av.batch ||
			    av.batch->count != a->command_count) {
				veto = M4_USER_VETO_PUBLISHING;
				goto vetoed;
			}
			for (u32 k = a->published_count; k < a->command_count; k++)
				rest |= BIT(av.batch->commands[k].type == DRM_ASAHI_CMD_RENDER);
			for (u32 k = 0; k < j->command_count; k++)
				mine |= BIT(view.batch->commands[k].type == DRM_ASAHI_CMD_RENDER);
			if (a->depth > 1 && (rest & mine)) {
				veto = M4_USER_VETO_PUBLISHING;
				goto vetoed;
			}
			partial = true;
		}
		/* Same queue (same notifier page), behind its whole published
		 * window: its first command waits on that tail; the page's
		 * thresholds, targets and stage counter advance per command
		 * exactly as for a published window successor. A compute
		 * command on a page with earlier compute uses needs the page's
		 * compute stamp written once: until then a new compute sequence
		 * is admitted only while every earlier use is an outstanding
		 * command of the SAME job.
		 */
		if (a->queue == j->queue) {
			for (u32 k = 0; k < j->command_count; k++)
				render_only &= view.batch->commands[k].type == DRM_ASAHI_CMD_RENDER;
			/* inflight2_wide=0 (r65): only a render-only window
			 * behind a render tail.
			 */
			if ((!p->user_inflight2_wide && !READ_ONCE(inflight2_mixed) &&
			     (!render_only || a->cmd[a->published_count - 1].engine_mask != 3)) ||
			    (!render_only && j->queue->notifier->sequence[2] &&
			     !m4_ready_read32(j->queue->notifier->bo->map.vaddr + 0x640))) {
				veto = M4_USER_VETO_QUEUE;
				goto vetoed;
			}
			same_queue = true;
		}
		if (!a->bank_held || a->bank == j->bank) {
			veto = M4_USER_VETO_BANK;
			goto vetoed;
		}
		same_vm |= a->queue->vm == j->queue->vm;
		compute_tail |= a->cmd[a->published_count - 1].engine_mask == BIT(2);
	}
	if (!m4_user_coresident_locked(p, j)) {
		veto = M4_USER_VETO_ASID;
		goto vetoed;
	}
	if (compute_tail && view.batch->commands[0].type == DRM_ASAHI_CMD_RENDER &&
	    !READ_ONCE(inflight2_mixed)) {
		veto = M4_USER_VETO_CLASS;
		goto vetoed;
	}
	/* Worst case of its whole window: a render's TA [barrier, InitBuffer,
	 * TA] and FS [early-tiling prior-FS wait, own-TA wait, FS], a
	 * compute's [barrier, work]; one submit-pipe record per command and
	 * engine.
	 */
	for (u32 i = 0; i < j->command_count; i++) {
		if (view.batch->commands[i].type == DRM_ASAHI_CMD_RENDER) {
			entries[0] += 3; entries[1] += 3;
			records[0]++; records[1]++;
		} else {
			entries[2] += 2;
			records[2]++;
		}
	}
	for (u32 e = 0; e < 3; e++) {
		if (!entries[e])
			continue;
		ret = m4_user_wire_admit_locked(j, e, entries[e], records[e]);
		if (ret) {
			veto = ret == -EMLINK ? M4_USER_VETO_PIPE : M4_USER_VETO_RING;
			goto vetoed;
		}
	}
	st->admitted++;
	st->admitted_partial += partial;
	if (same_queue)
		st->admitted_queue++;
	else if (same_vm)
		st->admitted_vm++;
	else
		st->admitted_foreign++;
	return 0;
vetoed:
	st->veto[veto]++;
	if (!IS_ERR(j))
		j->veto = veto + 1;
	return -EBUSY;
}

static_assert(GPU_M4_DRM_QUEUE_NEXTS <=
	      ARRAY_SIZE(((struct m4_user_inflight2_stats *)0)->retire_next), "queue next stats");

/* REMOVE-DEBUG: what stood behind head j at its retirement (inflight2). */
static void m4_user_inflight2_observe_locked(struct m4_gpu_provider_runtime *p,
	struct m4_user_job *j)
{
	struct m4_user_inflight2_stats *st = &p->user_inflight2_stats;
	struct gpu_m4_drm_provider_lane_item *next;
	struct m4_user_job *k;
	int queue;

	if (!p->user_inflight2)
		return;
	for (u32 i = 1; i < M4_USER_INFLIGHT_MAX; i++) {
		if (!p->user_inflight[i].job)
			continue;
		k = m4_user_lane_job_locked(p, p->user_inflight[i].job);
		if (!IS_ERR(k) && k->queue == j->queue)
			st->retire_paired_queue++;
		else
			st->retire_paired_other++;
		return;
	}
	next = gpu_m4_drm_provider_lane_ready_first_locked(p->user_lane);
	if (next) {
		k = m4_user_lane_job_locked(p, next->job);
		st->retire_ready++;
		st->retire_veto[IS_ERR(k) || !k->veto ? M4_USER_VETOES : k->veto - 1]++;
		return;
	}
	st->retire_idle++;
	queue = gpu_m4_drm_provider_job_queue_next_locked(j->core_job);
	if (queue >= 0 && queue < GPU_M4_DRM_QUEUE_NEXTS)
		st->retire_next[queue]++;
}

/* Publish the next contiguous commands of the protected child up to its
 * bounded depth: never beyond the child count, the reserved ring credits or
 * the per-command slot leases. A serial (depth one) child publishes exactly
 * its head. A deeper child is homogeneous or (mixed_window) mixed; each
 * command carries its strict prior-command edge. A compute command is its own packet. With render_batch
 * consecutive renders published in this scope form packets of at most
 * render_batch_max members (ring entries of every member, one pipe record and
 * one kick per engine, flush requests on its last render); otherwise each
 * render is its own packet. Any failure leaves every
 * already installed owner retained and reachable (an open packet is closed
 * through its last installed member); nothing later is published.
 */
static int m4_user_publish_window_locked(struct m4_user_job *j)
{
	struct gpu_m4_drm_provider_job_view view = { };
	bool health = false, batch = READ_ONCE(render_batch);
	u32 batch_max = clamp_t(u32, READ_ONCE(render_batch_max), 1, M4_USER_RENDER_DEPTH);
	int ret;

	gpu_m4_drm_provider_job_view(j->core_job, &view);
	if (!view.batch || view.batch->count != j->command_count ||
	    j->command_count > ARRAY_SIZE(j->cmd) || view.stage_owner != j->native_stage ||
	    !j->depth || j->completed_count > j->published_count ||
	    j->published_count > j->command_count ||
	    j->published_count - j->completed_count > j->depth || j->wire_open)
		return M4_USER_WHY(-ESTALE, "window batch=%u/%u stage=%d window=%u/%u depth=%u open=%u",
			view.batch ? view.batch->count : 0, j->command_count,
			view.stage_owner == j->native_stage, j->completed_count, j->published_count,
			j->depth, j->wire_open);
	while (j->published_count < j->command_count &&
	       j->published_count - j->completed_count < j->depth) {
		u32 index = j->published_count;
		u16 type = view.batch->commands[index].type;
		/* Another member follows in this scope: keep the packet open. */
		bool defer = batch && j->depth > 1 && type == DRM_ASAHI_CMD_RENDER &&
			index + 1 < j->command_count && index + 1 - j->completed_count < j->depth &&
			view.batch->commands[index + 1].type == DRM_ASAHI_CMD_RENDER &&
			j->wire_open + 1 < batch_max;

		if (j->cmd[index].published || j->cmd[index].receipt ||
		    (j->depth != 1 && !j->mixed && type != view.batch->commands[0].type))
			return M4_USER_WHY(-ESTALE, "window %u published=%d receipt=%d type=%u first=%u depth=%u",
				index, j->cmd[index].published, j->cmd[index].receipt, type,
				view.batch->commands[0].type, j->depth);
		j->command_index = j->completed_count;
		/* One gate per locked publication scope: the scalar latches and
		 * SGX/thermal service (health_per_job=0) or the live graph/session
		 * pass (or this lane-worker hold's, m4_provider_ready_health()).
		 * Nested packet owners recheck exact binding/core phase.
		 */
		if (!health) {
			ret = READ_ONCE(health_per_job) ?
				m4_provider_ready_health(j->runtime->hello) :
				m4_user_publish_gate_locked(j->runtime);
			if (ret)
				return M4_USER_WHY(ret, "window %u READY health", index);
			health = true;
		}
		if (type == DRM_ASAHI_CMD_COMPUTE && !j->wire_open)
			ret = m4_user_compute_packet_publish_locked(j, index);
		else if (type == DRM_ASAHI_CMD_COMPUTE)
			ret = -EPROTO; /* Never behind an open render packet. */
		else
			ret = m4_user_render_packet_publish_locked(j, index, defer);
		if (!ret && (!j->cmd[index].published || j->published_count != index + 1 ||
		    (!defer && j->wire_open)))
			ret = -EPROTO;
		if (!ret)
			ret = m4_user_lane_command_published_locked(j, index);
		if (ret) {
			/* Installed members of an open packet become reachable now:
			 * a retained owner is never left unpublished on the wire.
			 */
			if (j->wire_open) {
				int close = m4_user_render_packet_close_locked(j);

				if (close)
					(void)M4_USER_WHY(close, "window %u close open packet", index);
			}
			return M4_USER_WHY(ret, "window publish %u type=%u window=%u/%u", index, type,
				j->completed_count, j->published_count);
		}
		/* REMOVE-DEBUG: publication ahead of an unreceipted predecessor. */
		if (index > j->completed_count) {
			m4_user_window_stats.ahead[type == DRM_ASAHI_CMD_RENDER]++;
			m4_user_window_stats.outstanding_max = max(m4_user_window_stats.outstanding_max,
				(u64)(j->published_count - j->completed_count));
		}
	}
	/* Unreachable: the last member of a scope never defers. */
	return j->wire_open ? M4_USER_WHY(m4_user_render_packet_close_locked(j) ?: -EPROTO,
		"window open packet %u", j->wire_open) : 0;
}

static int m4_user_publish_locked(void *owner, struct gpu_m4_drm_provider_job *job)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_user_job *j = m4_user_lane_job_locked(p, job);
	struct gpu_m4_drm_provider_job_view view = { };
	u64 start, installs = m4_user_bind_stats[2];
	int ret;

	if (IS_ERR(j))
		return PTR_ERR(j);
	m4_user_why_begin(job, "publish");
	ret = m4_user_lane_check_locked(p, job);
	gpu_m4_drm_provider_job_view(job, &view);
	start = ktime_get_ns();
	/* Establishes this exact protected job's selected-inventory proof once. */
	if (!ret)
		ret = m4_user_native_receipt_prepare_locked(j->native_stage, job);
	start = m4_user_start_account(M4_USER_START_RECEIPT, start);
	if (!ret)
		ret = m4_user_bind_locked(p, j->asid, j->queue->vm, view.version, &j->binding_epoch,
			&j->binding_generation);
	if (!ret) {
		j->native_stage->pending->binding_epoch = j->binding_epoch;
		j->native_stage->pending->binding_generation = j->binding_generation;
		j->native_stage->pending->asid = j->asid;
		j->native_stage->pending->bind_sequence = ++p->user_bind_sequence;
		j->native_stage->pending->cross_stamp_fw = j->cross.notifier ? j->cross.stamp_fw : 0;
		j->native_stage->pending->cross_event = j->cross.notifier ? j->cross.event : 0;
		if (j->item.overlap & GPU_M4_DRM_LANE_OVERLAP_SEEN) {
			if (m4_user_bind_stats[2] != installs)
				m4_user_overlap_stats.rebind++;
			else
				m4_user_overlap_stats.same_image++;
		}
	}
	start = m4_user_start_account(M4_USER_START_BIND, start);
	/* Any failure after the core's protection gate retains this whole stage,
	 * including a partially published canonical alias or root transaction.
	 */
	m4_user_native_stage_exposed_locked(j->native_stage);
	/* One live upper graph/lease pass for this publication (health_per_job=0:
	 * only at a changed upper revision); an unchanged published lease then
	 * reuses its publication proof at this revision, while each BO still
	 * passes this job's live lane/phase/session gate.
	 */
	if (!ret)
		ret = m4_user_upper_health_locked(p);
	if (!ret)
		ret = m4_user_native_bo_arm_locked(j->queue->notifier->bo, job);
	if (!ret)
		ret = gpu_m4_drm_upper_publish_locked(j->queue->notifier->bo->upper, false);
	for (u32 i = 0; !ret && i < j->native_stage->bo_count; i++) {
		struct m4_user_native_bo *b = j->native_stage->bo[i];

		if (!b->upper || b->cell)
			continue; /* Pre-RUN arena alias is already visible, independently proved. */
		ret = m4_user_native_bo_arm_locked(b, job);
		if (!ret)
			ret = gpu_m4_drm_upper_publish_locked(b->upper, false);
	}
	start = m4_user_start_account(M4_USER_START_ARM, start);
	if (!ret && j->token)
		ret = gpu_m4_drm_timestamp_alias_token_arm_locked(j->token, job);
	if (!ret && j->token)
		ret = gpu_m4_drm_timestamp_alias_token_publish_locked(j->token, false);
	start = m4_user_start_account(M4_USER_START_TOKEN, start);
	if (!ret)
		ret = gpu_m4_drm_provider_version_begin_work_locked(view.version);
	start = m4_user_start_account(M4_USER_START_BEGIN, start);
	if (!ret)
		ret = m4_user_publish_window_locked(j);
	m4_user_start_account(M4_USER_START_PACKET, start);
	/* REMOVE-DEBUG: a failed start's trail (a local failure has no fault
	 * report of its own).
	 */
#ifdef CONFIG_DEBUG_FS
	if (ret && m4_user_why.job == job && printk_ratelimit()) {
		u32 count = m4_user_why_count(), first = count > M4_USER_WHY_DEPTH ?
			count - M4_USER_WHY_DEPTH : 0;

		for (u32 i = first; i < count; i++) {
			const struct m4_user_why_entry *e = &m4_user_why.entry[i % M4_USER_WHY_DEPTH];

			dev_warn(p->hello->gpu->dev, "GPU ordinary start why %s[%u/%u]: %d at %s:%d %s\n",
				m4_user_why.call, i + 1, count, e->error, e->file, e->line, e->text);
		}
	}
#endif
	return ret;
}

/* Bounded live state for a PENDING poll: latched callback/RTKit crash,
 * RUN and recovery registers, and the bound handoff roots. Full table/upper/
 * client graph validation runs only before a real receipt is recorded.
 */
static int m4_user_live_health_locked(struct m4_gpu_provider_runtime *p)
{
	struct m4_gpu_hello *h = p->hello;
	struct m4_gpu_power *gpu = h->gpu;

	if (p->error || h->ready->error || atomic_read(&h->callback_error) ||
	    !apple_rtkit_is_running(h->rtkit) || apple_rtkit_is_crashed(h->rtkit) ||
	    !(readl(gpu->asc + 0x44) & BIT(4)) || m4_pstate_sample(h, M4_PSTATE_POLL) > h->ready->pstate.configured)
		return -EIO;
	if (readq(gpu->ttbs) != (page_to_phys(h->lower) | 1) ||
	    readq(gpu->ttbs + 8) != (h->region[M4_TABLES].start | 1))
		return -EPROTO;
	return 0;
}

/* Quick observation of the current command's OWN firmware completion stamp
 * (compute +0x640, paired render fragment +0x630) against its reserved
 * target. true only proves "not yet": never a receipt, retirement or signal.
 * Anything unexpected falls through to the complete predicate.
 */
static bool m4_user_command_pending_locked(struct m4_user_job *j,
	const struct gpu_m4_drm_provider_job_view *view)
{
	const void *notifier;
	u32 index = j->completed_count, target, stamp;

	if (index >= j->command_count || index >= ARRAY_SIZE(j->cmd) || !j->cmd[index].published ||
	    j->cmd[index].receipt || !view->batch || index >= view->batch->count ||
	    !j->queue->notifier || !j->queue->notifier->bo)
		return false;
	notifier = j->queue->notifier->bo->map.vaddr;
	if (view->batch->commands[index].type == DRM_ASAHI_CMD_COMPUTE) {
		if (!j->compute_reservation)
			return false;
		target = j->compute_reservation[index].chronology.target_stamp;
		stamp = m4_ready_read32(notifier + 0x640);
	} else {
		if (!j->render_reservation)
			return false;
		target = j->render_reservation[index].chronology.target_stamp;
		stamp = m4_ready_read32(notifier + 0x630);
	}
	return target && (s32)(stamp - target) < 0;
}

/* Drain the completed prefix of the published window. Each full per-command
 * hardware receipt advances the head and may append the next command into
 * its own lease; only the whole child's receipt returns success to the real
 * CORE, which alone retires cache/version and authorizes the fence. Bounded
 * by the child count: every iteration requires a new genuine receipt.
 */
static int m4_user_retired_locked(void *owner, struct gpu_m4_drm_provider_job *job)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_user_job *j = m4_user_lane_job_locked(p, job);
	struct gpu_m4_drm_provider_job_view view = { };
	bool service;
	int ret;

	if (IS_ERR(j))
		return PTR_ERR(j);
	m4_user_why_begin(job, "retire");
	/* health_per_job=0 inside the lane worker's own hold: its service
	 * snapshot covers every poll and receipt. Any other holder (the TDR
	 * service: a fault boundary) and health_per_job=1 keep the per-poll
	 * live health and the complete READY/token pass before a receipt.
	 */
	service = !READ_ONCE(health_per_job) && p->user_lane &&
		gpu_m4_drm_provider_lane_hold_locked(p->user_lane);
	ret = m4_user_lane_check_locked(p, job);
	if (!ret && !service)
		ret = m4_ready_drain(p->hello);
	if (ret)
		return M4_USER_WHY(ret, "retire lane/drain");
	gpu_m4_drm_provider_job_view(job, &view);
	if (!view.batch || view.batch->count != j->command_count)
		return M4_USER_WHY(-ESTALE, "retire batch=%u/%u", view.batch ? view.batch->count : 0,
			j->command_count);
	for (;;) {
		u32 head = j->completed_count;

		if (head >= j->command_count || head >= j->published_count)
			return M4_USER_WHY(-ESTALE, "retire head %u/%u published=%u", head,
				j->command_count, j->published_count);
		/* A recent snapshot covers a poll that observes no progress. */
		if (service && m4_user_command_pending_locked(j, &view) && m4_user_poll_fresh(p)) {
			ret = m4_user_latched_locked(p);
			if (ret)
				return M4_USER_WHY(ret, "retire %u latched", head);
			m4_user_service_stats.fresh++;
			return 0;
		}
		ret = service ? m4_user_service_locked(p) : m4_user_live_health_locked(p);
		if (ret)
			return M4_USER_WHY(ret, "retire %u live health", head);
		if (m4_user_command_pending_locked(j, &view))
			return 0;
		/* Possible completion outside the worker's service: full graph
		 * health and token authority first (one live pass per lane-worker
		 * hold, m4_provider_ready_health()).
		 */
		if (!service) {
			ret = m4_provider_ready_health(p->hello);
			if (!ret && j->token)
				ret = gpu_m4_drm_timestamp_alias_token_check_locked(j->token);
			if (ret)
				return M4_USER_WHY(ret, "retire %u health/token=%d", head, !!j->token);
		}
		j->command_index = head;
		ret = view.batch->commands[head].type == DRM_ASAHI_CMD_COMPUTE ?
			m4_user_compute_packet_retired_locked(j, head) :
			m4_user_render_packet_retired_locked(j, head);
		if (ret == -EAGAIN)
			return 0;
		if (ret)
			return M4_USER_WHY(ret, "retire %u predicate type=%u", head,
				view.batch->commands[head].type);
		ret = m4_user_lane_command_retired_locked(j, head);
		if (ret)
			return M4_USER_WHY(ret, "retire %u lane", head);
		j->cmd[head].receipt = true;
		j->completed_count++;
		m4_user_service_void(p); /* Progress: the next service resnapshots. */
		if (j->completed_count == j->command_count)
			return 1; /* Core performs cache/version retirement, then lane notification. */
		j->command_index = j->completed_count;
		ret = m4_user_publish_window_locked(j);
		if (ret)
			return M4_USER_WHY(ret, "retire %u refill window=%u/%u", head, j->completed_count,
				j->published_count);
	}
}

static void m4_user_job_retired_locked(struct m4_gpu_provider_runtime *p, struct m4_user_job *j)
{
	lockdep_assert_held(&p->hello->gpu->lock);
	if (j->token) {
		gpu_m4_drm_timestamp_alias_token_complete_locked(j->token);
		j->token = NULL;
	}
	m4_user_job_images_put_locked(j);
	m4_user_native_receipt_complete_locked(j->native_stage);
	m4_user_bank_release_locked(p, j);
	m4_user_cross_put_locked(j); /* Its successor-held predecessor page. */
	/* Independent last-owner refs outlive this fence. A fully retired distinct
	 * replacement in the SAME fixed hardware slot displaces the predecessor.
	 */
	for (u32 i = 0; i < 3; i++) {
		if (!(j->all_engines & BIT(i)) || p->user_last[i] == j->native_stage)
			continue;
		j->displaced[i] = p->user_last[i];
		m4_user_native_stage_last_locked(j->native_stage, i);
		p->user_last[i] = j->native_stage;
	}
	for (u32 i = 0; i < 3; i++) {
		bool still_last = false;

		if (!j->displaced[i])
			continue;
		for (u32 e = 0; e < 3; e++)
			still_last |= p->user_last[e] == j->displaced[i];
		if (!still_last)
			m4_user_native_stage_cache_locked(j->displaced[i], j->native_stage);
	}
	/* j->displaced[] are disposal refs, not witness refs: a unit that still
	 * needs a predecessor's proof holds that history (so its job and stage)
	 * itself. With receipt, user_last[] and cache bookkeeping complete, move
	 * them to the carrier preallocated at acceptance for the existing
	 * outside-lock reaper, instead of retaining older (possibly closed)
	 * stages until this possibly long-current job dies. No final put here;
	 * cleared entries are never put again by m4_user_collect_native().
	 */
	if (j->disposal) {
		u32 moved = 0;

		for (u32 i = 0; i < 3; i++) {
			j->disposal->displaced[i] = j->displaced[i];
			moved += !!j->displaced[i];
			j->displaced[i] = NULL;
		}
		if (moved) {
			list_add_tail(&j->disposal->garbage, &p->user_job_garbage);
			j->disposal = NULL;
		}
	}
	/* Also the deferred cold scan: a displacement can complete a component. */
	schedule_work(&p->user_reap);
	/* Receipt/last-owner/cache bookkeeping above is complete. An execution
	 * window child now lets its aggregate materialize the next window, outside
	 * every lock; ordinary jobs are a no-op.
	 */
	WARN_ON(gpu_m4_drm_provider_job_window_complete_locked(j->core_job));
}

/* REMOVE-DEBUG fault attribution (READY hook, owner mutex held): every
 * leased user context with its VM, and each in-flight lane job's context,
 * client and window position. Read-only.
 */
static void m4_user_fault_report(void *owner)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct device *dev = p->hello->gpu->dev;

	lockdep_assert_held(&p->hello->gpu->lock);
	for (u32 asid = 0; asid < M4_USER_ASIDS; asid++) {
		const struct m4_user_asid *a = &p->user_asid[asid];

		if (!a->leases && !a->owner && !a->low && !a->quarantined)
			continue;
		dev_err(dev, "GPU fault ctx asid=%u vm=%px leases=%u low=%#llx generation=%llu resident=%d quarantined=%d\n",
			asid, a->owner, a->leases, a->low, a->generation, !!a->resident, a->quarantined);
	}
	for (u32 i = 0; i < M4_USER_INFLIGHT_MAX; i++) {
		struct m4_user_job *j;

		if (!p->user_inflight[i].job)
			continue;
		j = m4_user_lane_job_locked(p, p->user_inflight[i].job);
		if (IS_ERR_OR_NULL(j) || !j->queue || !j->queue->vm) {
			dev_err(dev, "GPU fault inflight[%u] unresolved\n", i);
			continue;
		}
		dev_err(dev, "GPU fault inflight[%u] asid=%u vm=%px client=%llu command=%u/%u published=%u completed=%u bank=%u\n",
			i, j->queue->vm->asid, j->queue->vm, (unsigned long long)j->queue->client_id,
			j->command_index, j->command_count, j->published_count, j->completed_count,
			j->bank);
	}
}

static void m4_user_fault_locked(void *owner, struct gpu_m4_drm_provider_job *job, int error)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_user_job *j = m4_user_lane_job_locked(p, job);

	if (!p->error && !IS_ERR(j)) {
		struct gpu_m4_drm_provider_job_view view = { };
		struct gpu_m4_drm_provider_lane_timing timing = { };
		const struct m4_user_command *head = NULL;
		u16 type = 0;

		gpu_m4_drm_provider_job_view(job, &view);
		if (view.batch && view.batch->commands && j->command_index < view.batch->count &&
		    j->command_index < ARRAY_SIZE(j->cmd)) {
			type = view.batch->commands[j->command_index].type;
			head = &j->cmd[j->command_index];
		}
		dev_err(p->hello->gpu->dev,
			"GPU ordinary job fault: command=%u/%u type=%u published=%u window=%u/%u depth=%u engines=%x slot=%u error=%d binding=%llu/%u\n",
			j->command_index, j->command_count, type, head && head->published,
			j->completed_count, j->published_count, j->depth,
			head ? head->engine_mask : 0, head ? head->uma_slot : 0, error,
			(unsigned long long)j->binding_epoch, j->binding_generation);
		if (!gpu_m4_drm_provider_lane_item_timing_locked(&j->item, job, &timing))
			dev_err(p->hello->gpu->dev,
				"GPU ordinary lane ns: activated=%llu head=%llu published=%llu deadline=%llu retired=%llu command=%u last_retired=%u horizon=%u/%u valid=%u/%u\n",
				(unsigned long long)timing.activated_ns,
				(unsigned long long)timing.head_ns,
				(unsigned long long)timing.published_ns,
				(unsigned long long)timing.deadline_ns,
				(unsigned long long)timing.retired_ns,
				timing.published_index, timing.retired_index,
				timing.retired_count, timing.published_count,
				timing.published, timing.retired_ns != 0 || timing.retired);
		/* REMOVE-DEBUG: the child within its batch, per-command slot/warm/
		 * wire map (warm: whether the same-slot predecessor of a prepared
		 * reuse proof shares this job's VM), then the exact failing
		 * sub-checks of this job's latest publish/retire call.
		 */
		dev_err(p->hello->gpu->dev,
			"GPU ordinary child: first=%u total=%u window=%d token=%d native_bytes=%llu\n",
			view.command_first, view.command_total, !!view.window_parent, !!j->token,
			(unsigned long long)p->user_native_bytes);
		for (u32 i = 0; i < j->command_count && i < ARRAY_SIZE(j->cmd); i++) {
			const struct m4_user_compute_reuse *reuse = j->compute_reuse ? &j->compute_reuse[i] : NULL;
			struct m4_user_uma_record uma = { };
			u32 timestamps = 0;

			if (j->cmd[i].uma_slot && j->cmd[i].uma_slot < 0x4000 / 0x20)
				uma = m4_user_why_uma_record(p, j->cmd[i].uma_slot);

			for (u32 k = 0; view.batch && view.batch->commands && i < view.batch->count &&
			     view.batch->commands[i].type == DRM_ASAHI_CMD_COMPUTE &&
			     k < GPU_M4_DRM_JOB_TIMESTAMPS; k++)
				timestamps += !!view.batch->commands[i].compute.timestamps[k].lease;
			if (view.batch && view.batch->commands && i < view.batch->count &&
			    view.batch->commands[i].type == DRM_ASAHI_CMD_COMPUTE)
				dev_err(p->hello->gpu->dev,
					"GPU ordinary compute %u: cdm=%#llx..%#llx length=%#llx usc=%#llx attachments=%u flags=%#x\n",
					i, (unsigned long long)view.batch->commands[i].compute.cdm_base,
					(unsigned long long)view.batch->commands[i].compute.cdm_end,
					(unsigned long long)(view.batch->commands[i].compute.cdm_end -
						view.batch->commands[i].compute.cdm_base),
					(unsigned long long)view.batch->commands[i].compute.usc_base,
					view.batch->commands[i].compute.attachment_count,
					view.batch->commands[i].compute.flags);

			dev_err(p->hello->gpu->dev,
				"GPU ordinary command %u: type=%u slot=%u warm=%d published=%d receipt=%d engines=%x queue=%u-%u pipe=%u-%u predecessor_vm=%d timestamps=%u uma pool=%#llx capacity=%u cursors=%u,%u empty=%u pages=%u\n",
				i, view.batch && view.batch->commands && i < view.batch->count ?
				view.batch->commands[i].type : 0xffff, j->cmd[i].uma_slot,
				m4_user_native_reused(j->native_stage, i), j->cmd[i].published,
				j->cmd[i].receipt, j->cmd[i].engine_mask, j->cmd[i].wire[2].queue_before,
				j->cmd[i].wire[2].queue_next, j->cmd[i].wire[2].pipe_before,
				j->cmd[i].wire[2].pipe_next, !reuse || !reuse->owner[1].vm_lease ? -1 :
				reuse->owner[1].vm_lease == j->queue->vm, timestamps,
				(unsigned long long)uma.pool, uma.capacity, uma.cursor, uma.second,
				uma.empty, uma.pages);
		}
		if (m4_user_why.job == job) {
			u32 count = m4_user_why_count(), first = count > M4_USER_WHY_DEPTH ?
				count - M4_USER_WHY_DEPTH : 0;

			for (u32 i = first; i < count; i++) {
				const struct m4_user_why_entry *e = &m4_user_why.entry[i % M4_USER_WHY_DEPTH];

				dev_err(p->hello->gpu->dev, "GPU ordinary why %s[%u/%u]: %d at %s:%d %s\n",
					m4_user_why.call, i + 1, count, e->error, e->file, e->line, e->text);
			}
		} else {
			dev_err(p->hello->gpu->dev, "GPU ordinary why: no trail for this job\n");
		}
		/* The head, then each published successor of a render window. */
		for (u32 i = j->command_index; type == DRM_ASAHI_CMD_RENDER &&
		     i < j->published_count && i < ARRAY_SIZE(j->cmd); i++)
			m4_user_render_fault_snapshot_locked(j, i);
	}
	if (!IS_ERR(j) && j->token)
		gpu_m4_drm_timestamp_alias_token_fault_locked(j->token, error);
	m4_user_lane_fault_locked(owner, job, error);
	m4_provider_fault_locked(p, error);
}

static void m4_user_enqueue_locked(void *owner, struct gpu_m4_drm_provider_job *job)
{
	m4_user_lane_enqueue_locked(owner, job);
	m4_user_probe_enqueue_locked(owner, job);
}

static const struct gpu_m4_drm_provider_job_runtime m4_user_runtime_ops = {
	.queue_get_locked = m4_user_queue_get_locked,
	.queue_put_locked = m4_user_runtime_queue_put_locked,
	.stage_native = m4_user_native_stage, .stage_put = m4_user_native_stage_job_put,
	.timestamp_hold_locked = m4_user_timestamp_hold_locked,
	.timestamp_put_locked = m4_user_timestamp_put_locked,
	.accept_locked = m4_user_accept_locked, .release_locked = m4_user_release_locked,
	.enqueue_locked = m4_user_enqueue_locked, .close_locked = m4_user_lane_close_locked,
	.prepare_locked = m4_user_prepare_locked, .publish_locked = m4_user_publish_locked,
	.retired_locked = m4_user_retired_locked, .fault_locked = m4_user_fault_locked,
	.timedout = m4_user_lane_timedout, .reap = m4_user_reap,
};

static int m4_user_runtime_prepare(struct m4_gpu_provider_runtime *p)
{
	int ret = m4_user_lane_init(p);

	if (!ret)
		p->user_runtime = &m4_user_runtime_ops;
	return ret;
}

/* After real READY/policy/queue registration. Creates only ordinary lifetime
 * machinery, never a public device or a fabricated hardware qualification.
 */
static int m4_user_runtime_activate(struct m4_gpu_provider_runtime *p)
{
	struct gpu_m4_drm_timestamp_alias_config config;
	struct gpu_m4_drm_timestamp_aliases *aliases;
	struct mutex *lock = &p->hello->gpu->lock;
	u32 armed = 1;
	int ret;

	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	ret = p->error ?: (!p->ordinary || !p->prepared || p->complete || p->ts_aliases ||
		!p->ready->arena || !p->ready->arena->imported ||
		!p->ready->complete || !p->ready->policy_complete ||
		!p->execution->barrier_complete || !p->user_lane ? -EPERM : 0);
	if (!ret)
		ret = m4_provider_idle(p->hello);
	if (!ret) {
		p->complete = true;
		p->user_physical_used = BIT(0); /* Actual retired physical registration. */
		armed = gpu_m4_drm_provider_lane_max_inflight_locked(p->user_lane);
	}
	mutex_unlock(lock);
	if (ret)
		return ret;
	config = (struct gpu_m4_drm_timestamp_alias_config) {
		.provider = p->provider, .upper = p->ready->upper,
		.timestamp = m4_provider_ts_timestamp_config(p), .owner = p,
		.owner_check_locked = m4_user_timestamp_owner_locked,
		.lane_check_locked = m4_user_timestamp_lane_locked,
		.memory_limit = SZ_1M, .max_entries = 256, .max_pages = 4096,
		.max_requests = 4096, .max_tokens = 64, .max_armed = armed,
	};
	aliases = gpu_m4_drm_timestamp_alias_create(&config);
	if (IS_ERR(aliases)) {
		mutex_lock(lock);
		m4_provider_fault_locked(p, PTR_ERR(aliases));
		mutex_unlock(lock);
		return PTR_ERR(aliases);
	}
	mutex_lock(lock);
	p->ts_aliases = aliases;
	mutex_unlock(lock);
	return 0;
}

static ssize_t runtime_start_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_provider_runtime *p;
	int ret;

	if (!sysfs_streq(buf, "1") || !runtime_enable)
		return -EINVAL;
	mutex_lock(&gpu->lock);
	p = gpu->hello && gpu->hello->ready ? gpu->hello->ready->provider_runtime : NULL;
	mutex_unlock(&gpu->lock);
	if (!p)
		return -ENODEV;
	ret = m4_user_runtime_activate(p);
	return ret ?: count;
}
static DEVICE_ATTR_WO(runtime_start);

static ssize_t runtime_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_provider_runtime *p;
	u32 leases = 0, pages = 0;
	ssize_t size;

	mutex_lock(&gpu->lock);
	p = gpu->hello && gpu->hello->ready ? gpu->hello->ready->provider_runtime : NULL;
	if (!p || !p->ordinary) {
		size = sysfs_emit(buf, "prepared=0 active=0 error=0 arena=0 bytes=0 queues=0 binding=0 command=0\n");
	} else {
		gpu_m4_drm_upper_usage_locked(p->ready->upper, &leases, &pages);
		size = sysfs_emit(buf,
			"prepared=%u active=%u error=%d arena=%u bytes=%llu queues=%u binding=%llu command=%u upper_leases=%u upper_pages=%u\n",
			p->prepared, p->complete, p->error,
			p->ready->arena && p->ready->arena->imported,
			p->user_native_bytes, p->user_physical_used, p->user_binding_revision,
			p->user_inflight[0].item ? container_of(p->user_inflight[0].item,
				struct m4_user_job, item)->command_index : 0, leases, pages);
	}
	mutex_unlock(&gpu->lock);
	return size;
}
static DEVICE_ATTR_RO(runtime_state);

/* REMOVE-DEBUG: cumulative ordinary latency aggregates (ns), observational. */
/* REMOVE-DEBUG inflight2: successors admitted beside a head (same VM other
 * queue, other VM, same queue), global-tail barriers on a render/compute
 * predecessor, time with two in flight, vetoes by first failing rule (per
 * admission attempt) and accepted jobs per bank. Own frame: runtime_perf_show
 * is at its stack bound.
 */
static noinline ssize_t m4_user_inflight2_show(struct m4_gpu_power *gpu, char *buf, ssize_t size)
{
	static const char *const veto_name[M4_USER_VETOES] = {
		"disabled", "state", "publishing", "queue", "bank", "asid", "class", "ring", "pipe",
		"depth",
	};
	static const char *const next_name[GPU_M4_DRM_QUEUE_NEXTS] = {
		"none", "waiting", "own", "gated", "building", "enqueued",
	};
	struct m4_user_inflight2_stats st = { };
	struct m4_gpu_provider_runtime *p;
	ssize_t n;

	mutex_lock(&gpu->lock);
	p = gpu->hello && gpu->hello->ready ? gpu->hello->ready->provider_runtime : NULL;
	if (p)
		st = p->user_inflight2_stats;
	mutex_unlock(&gpu->lock);
	n = sysfs_emit_at(buf, size,
		" inflight2_admitted=%llu inflight2_vm=%llu inflight2_foreign=%llu inflight2_queue=%llu inflight2_barrier_render=%llu inflight2_barrier_compute=%llu inflight2_overlap_ns=%llu inflight2_overlap_max=%llu",
		st.admitted, st.admitted_vm, st.admitted_foreign, st.admitted_queue,
		st.barrier[0], st.barrier[1], st.overlap_ns, st.overlap_max);
	for (u32 i = 0; i < M4_USER_VETOES; i++)
		n += sysfs_emit_at(buf, size + n, " inflight2_veto_%s=%llu", veto_name[i],
			st.veto[i]);
	for (u32 i = 0; i < M4_USER_INFLIGHT2_BANKS; i++)
		n += sysfs_emit_at(buf, size + n, " bank%u_jobs=%llu", i, st.bank_jobs[i]);
	/* Per head retirement: paired (successor already in flight), a READY
	 * candidate by its latest veto, or nothing READY by the retiring job's
	 * own queue: none accepted, waiting (inputs/credit), own next window,
	 * a later batch gated / materializing / enqueued.
	 */
	n += sysfs_emit_at(buf, size + n,
		" retire_paired_queue=%llu retire_paired_other=%llu retire_ready=%llu retire_idle=%llu inflight2_admitted_partial=%llu",
		st.retire_paired_queue, st.retire_paired_other, st.retire_ready, st.retire_idle,
		st.admitted_partial);
	for (u32 i = 0; i <= M4_USER_VETOES; i++)
		n += sysfs_emit_at(buf, size + n, " retire_veto_%s=%llu",
			i < M4_USER_VETOES ? veto_name[i] : "unasked", st.retire_veto[i]);
	for (u32 i = 0; i < GPU_M4_DRM_QUEUE_NEXTS; i++)
		n += sysfs_emit_at(buf, size + n, " retire_next_%s=%llu", next_name[i],
			st.retire_next[i]);
	return n;
}

/* REMOVE-DEBUG client pacing per public job: the final child's receipt to
 * the public fence signal, that signal to the queue's next submit entry
 * (client time), submit entry to scheduler run, and submits entering while
 * the previous job was still unsignalled. Own frame (stack bound).
 */
static noinline ssize_t m4_user_pacing_show(char *buf, ssize_t size)
{
	static const char *const name[GPU_M4_DRM_PROVIDER_PACINGS] = {
		"retire_signal", "client", "submit_run", "ahead",
	};
	u64 pacing[GPU_M4_DRM_PROVIDER_PACINGS][3];
	ssize_t n = 0;

	gpu_m4_drm_provider_job_pacing_stats(pacing);
	for (u32 i = 0; i < GPU_M4_DRM_PROVIDER_PACINGS; i++)
		n += sysfs_emit_at(buf, size + n, " pacing_%s=%llu pacing_%s_ns=%llu pacing_%s_max=%llu",
			name[i], pacing[i][0], name[i], pacing[i][1], name[i], pacing[i][2]);
	return n;
}

static ssize_t runtime_perf_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_provider_runtime *p;
	struct gpu_m4_drm_provider_lane_stats s = { 0 };
	static const char *const phase[M4_USER_START_PHASES] = {
		"idle", "receipt", "bind", "arm", "token", "begin", "packet",
	};
	u64 j[6], perf[M4_USER_START_PHASES][3], proof[8], upper_walks = 0, upper_reused = 0;
	u64 health[4 + M4_READY_COSTS + 2];
	u64 lease_walks = 0, lease_reused = 0, ready[4] = { 0 };
	u64 job_proof[GPU_M4_DRM_PROVIDER_PROOF_STATS];
	u64 early[GPU_M4_DRM_PROVIDER_EARLY_STATS];
	u64 ready_phase[2][GPU_M4_DRM_PROVIDER_READY_PHASES][3];
	static const char *const ready_name[GPU_M4_DRM_PROVIDER_READY_PHASES] = {
		"enqueue", "start", "publish",
	};
	u64 window[3], overlap[2], bind[3];
	u64 uat[GPU_M4_DRM_UAT_STATS];
	struct gpu_m4_drm_upper_choose_stats choose = { };
	u64 upper_released = 0, upper_release_failures = 0;
	/* REMOVE-DEBUG census: heap, it would exceed this frame's budget. */
	struct m4_user_native_census *census = kzalloc(sizeof(*census), GFP_KERNEL);
	ssize_t size;

	if (!census)
		return -ENOMEM;

	gpu_m4_drm_provider_job_stats(j);
	gpu_m4_drm_provider_job_proof_stats(job_proof);
	gpu_m4_drm_provider_job_early_stats(early, ready_phase);
	gpu_m4_drm_uat_stats(uat);
	mutex_lock(&gpu->lock);
	p = gpu->hello && gpu->hello->ready ? gpu->hello->ready->provider_runtime : NULL;
	if (p && p->user_lane)
		gpu_m4_drm_provider_lane_stats_locked(p->user_lane, &s);
	if (p && p->ordinary)
		m4_user_native_census_locked(p, census);
	if (p && p->ready && p->ready->upper) {
		gpu_m4_drm_upper_publish_stats_locked(p->ready->upper, &upper_walks, &upper_reused);
		gpu_m4_drm_upper_lease_stats_locked(p->ready->upper, &lease_walks, &lease_reused);
		gpu_m4_drm_upper_choose_stats_locked(p->ready->upper, &choose);
		gpu_m4_drm_upper_release_stats_locked(p->ready->upper, &upper_released,
			&upper_release_failures);
	}
	if (p && p->ready) {
		ready[0] = p->ready->drain_waits;
		ready[1] = p->ready->drain_wait_ns;
		ready[2] = p->ready->drain_wait_max;
		ready[3] = READ_ONCE(p->ready->drain_deferred);
	}
	for (u32 i = 0; i < M4_USER_START_PHASES; i++) {
		perf[i][0] = m4_user_start_perf[i].count;
		perf[i][1] = m4_user_start_perf[i].ns;
		perf[i][2] = m4_user_start_perf[i].max;
	}
	proof[0] = m4_user_native_proof_stats.proofs;
	proof[1] = m4_user_native_proof_stats.proof_bos;
	proof[2] = m4_user_native_proof_stats.arm_reused;
	proof[3] = m4_user_native_proof_stats.arm_full;
	proof[4] = m4_user_native_proof_stats.receipt_proofs;
	proof[5] = m4_user_native_proof_stats.receipt_reused;
	proof[6] = m4_user_native_proof_stats.render_reused;
	proof[7] = m4_user_native_proof_stats.render_full;
	health[0] = m4_provider_health_stats.live;
	health[1] = m4_provider_health_stats.reused;
	health[2] = m4_provider_health_stats.live_ns;
	health[3] = m4_provider_health_stats.refused;
	for (u32 i = 0; i < M4_READY_COSTS; i++)
		health[4 + i] = m4_ready_health_cost.ns[i];
	health[4 + M4_READY_COSTS] = m4_ready_health_cost.passes;
	health[5 + M4_READY_COSTS] = m4_ready_health_cost.reused;
	window[0] = m4_user_window_stats.ahead[0];
	window[1] = m4_user_window_stats.ahead[1];
	window[2] = m4_user_window_stats.outstanding_max;
	overlap[0] = m4_user_overlap_stats.same_image;
	overlap[1] = m4_user_overlap_stats.rebind;
	for (u32 i = 0; i < 3; i++)
		bind[i] = m4_user_bind_stats[i];
	mutex_unlock(&gpu->lock);
	size = sysfs_emit(buf,
		"jobs=%llu queue_ns=%llu queue_max=%llu start_ns=%llu start_max=%llu commands=%llu command_ns=%llu command_max=%llu gaps=%llu gap_ns=%llu gap_max=%llu polls=%llu pending=%llu timer_wakes=%llu notify_wakes=%llu prepares=%llu prepare_ns=%llu prepare_max=%llu windows=%llu window_ns=%llu window_max=%llu ahead=%llu",
		s.jobs, s.queue_ns, s.queue_max, s.start_ns, s.start_max, s.commands,
		s.command_ns, s.command_max, s.gaps, s.gap_ns, s.gap_max, s.polls,
		s.pending_polls, s.timer_wakes, s.notify_wakes, j[0], j[1], j[2], j[3], j[4], j[5],
		s.ahead);
	for (u32 i = 0; i < M4_USER_START_PHASES; i++)
		size += sysfs_emit_at(buf, size, " start_%s=%llu start_%s_ns=%llu start_%s_max=%llu",
			phase[i], perf[i][0], phase[i], perf[i][1], phase[i], perf[i][2]);
	size += sysfs_emit_at(buf, size,
		" ahead_compute=%llu ahead_render=%llu outstanding_max=%llu",
		window[0], window[1], window[2]);
	/* Stage 0 two-in-flight opportunity: activations READY behind an ACTIVE
	 * head by class (same queue / same VM / different VM), whether the
	 * candidate's image matched the resident at its bind, successor
	 * promotions, and all binds by kind (resident / adopted / installed).
	 */
	size += sysfs_emit_at(buf, size,
		" overlap=%llu overlap_queue=%llu overlap_vm=%llu overlap_foreign=%llu overlap_same_image=%llu overlap_rebind=%llu promotions=%llu bind_resident=%llu bind_adopt=%llu bind_install=%llu",
		s.overlap, s.overlap_queue, s.overlap_vm, s.overlap_foreign, overlap[0], overlap[1],
		s.promotions, bind[0], bind[1], bind[2]);
	size += sysfs_emit_at(buf, size, " successor_services=%llu", s.successor_services);
	size += sysfs_emit_at(buf, size, " compute_wide_flagged=%llu compute_wide_coalesced=%llu",
		m4_user_compute_wide_stats.flagged, m4_user_compute_wide_stats.coalesced);
	size += m4_user_pacing_show(buf, size);
	size += m4_user_inflight2_show(gpu, buf, size);
	size += m4_user_native_unit_stats_show(buf, size);
	/* Live units and native bytes by retention state, both native budgets. */
	size += m4_user_native_census_show(buf, size, census);
	size += sysfs_emit_at(buf, size,
		" inventory_proofs=%llu inventory_bos=%llu arm_reused=%llu arm_full=%llu upper_publish_walks=%llu upper_publish_reused=%llu",
		proof[0], proof[1], proof[2], proof[3], upper_walks, upper_reused);
	/* Cached proofs: held-lease path walks/reuse, coverage proofs/reuse,
	 * complete-history receipt proofs/reuse.
	 */
	size += sysfs_emit_at(buf, size,
		" upper_lease_walks=%llu upper_lease_reused=%llu covered_proofs=%llu covered_reused=%llu receipt_proofs=%llu receipt_reused=%llu",
		lease_walks, lease_reused, job_proof[GPU_M4_DRM_PROVIDER_PROOF_COVERED_PROOFS],
		job_proof[GPU_M4_DRM_PROVIDER_PROOF_COVERED_HITS], proof[4], proof[5]);
	/* Same-hold READY health: live passes (and their time) versus reuses
	 * inside one lane-worker hold; render owner BO checks reusing the job's
	 * selected-inventory proof versus complete ones; job access checks
	 * answered by the snapshot's identity row versus the complete scans.
	 */
	size += sysfs_emit_at(buf, size,
		" ready_health_live=%llu ready_health_reused=%llu ready_health_ns=%llu render_bo_reused=%llu render_bo_full=%llu access_indexed=%llu access_scanned=%llu",
		health[0], health[1], health[2], proof[6], proof[7],
		job_proof[GPU_M4_DRM_PROVIDER_PROOF_ACCESS_INDEXED],
		job_proof[GPU_M4_DRM_PROVIDER_PROOF_ACCESS_SCANNED]);
	/* Same-hold reuses refused by their live RUN/SGX/thermal/TTBAT/binding
	 * observations; live READY health pass cost by component (every holder's
	 * passes, so a superset of ready_health_live): fixed/root, READY tables,
	 * upper, edges, execution, arena, provider (with TTBAT[0/1]), TTBAT slots
	 * 2..127, and upper page invalidations an earlier one in the same pass
	 * made redundant.
	 */
	size += sysfs_emit_at(buf, size,
		" ready_health_refused=%llu ready_health_passes=%llu ready_health_fixed_ns=%llu ready_health_tables_ns=%llu ready_health_upper_ns=%llu ready_health_edges_ns=%llu ready_health_exec_ns=%llu ready_health_arena_ns=%llu ready_health_provider_ns=%llu ready_health_ttbat_ns=%llu ready_health_inval_reused=%llu",
		health[3], health[4 + M4_READY_COSTS], health[4 + M4_READY_COST_FIXED],
		health[4 + M4_READY_COST_TABLES], health[4 + M4_READY_COST_UPPER],
		health[4 + M4_READY_COST_EDGES], health[4 + M4_READY_COST_EXEC],
		health[4 + M4_READY_COST_ARENA], health[4 + M4_READY_COST_PROVIDER],
		health[4 + M4_READY_COST_TTBAT], health[5 + M4_READY_COSTS]);
	/* Service health cadence (health_per_job): lane-worker service snapshots
	 * taken (and their time) and reused in one hold, scalar publication
	 * gates, upper graph passes run/skipped at publication, and READY
	 * drain-worker notification passes with the bounded live service only.
	 * Unlocked single-word reads of observational counters (stack bound).
	 */
	size += sysfs_emit_at(buf, size,
		" health_per_job=%d service_snapshots=%llu service_reused=%llu service_ns=%llu publish_gates=%llu upper_health_runs=%llu upper_health_skips=%llu ready_notify_light=%llu poll_fresh=%llu",
		READ_ONCE(health_per_job), READ_ONCE(m4_user_service_stats.snapshots),
		READ_ONCE(m4_user_service_stats.reused), READ_ONCE(m4_user_service_stats.snapshot_ns),
		READ_ONCE(m4_user_service_stats.gates), READ_ONCE(m4_user_service_stats.upper_runs),
		READ_ONCE(m4_user_service_stats.upper_skips),
		READ_ONCE(m4_user_service_stats.notify_light),
		READ_ONCE(m4_user_service_stats.fresh));
	/* Mutex contention: the lane worker's waits and holds (a start hold is
	 * one job's activation through its first window), the READY drain
	 * worker's waits and deferrals, and the submit snapshot's capture path.
	 */
	size += sysfs_emit_at(buf, size,
		" lane_waits=%llu lane_wait_ns=%llu lane_wait_max=%llu lane_holds=%llu lane_hold_ns=%llu lane_hold_max=%llu start_holds=%llu start_hold_ns=%llu start_hold_max=%llu",
		s.waits, s.wait_ns, s.wait_max, s.holds, s.hold_ns, s.hold_max,
		s.start_holds, s.start_hold_ns, s.start_hold_max);
	size += sysfs_emit_at(buf, size,
		" ready_waits=%llu ready_wait_ns=%llu ready_wait_max=%llu ready_deferred=%llu capture_lockless=%llu capture_locked=%llu capture_wait_ns=%llu capture_wait_max=%llu",
		ready[0], ready[1], ready[2], ready[3],
		job_proof[GPU_M4_DRM_PROVIDER_PROOF_CAPTURE_LOCKLESS],
		job_proof[GPU_M4_DRM_PROVIDER_PROOF_CAPTURE_LOCKED],
		job_proof[GPU_M4_DRM_PROVIDER_PROOF_CAPTURE_WAIT_NS],
		job_proof[GPU_M4_DRM_PROVIDER_PROOF_CAPTURE_WAIT_MAX]);
	/* Host capacity: public jobs refused locally (ioctl error or that batch's
	 * error fence, never a provider fault) and reclaim-then-rebuild passes.
	 */
	size += sysfs_emit_at(buf, size, " capacity_refusals=%llu capacity_retries=%llu capacity_splits=%llu",
		job_proof[GPU_M4_DRM_PROVIDER_PROOF_CAPACITY_REFUSALS],
		job_proof[GPU_M4_DRM_PROVIDER_PROOF_CAPACITY_RETRIES],
		job_proof[GPU_M4_DRM_PROVIDER_PROOF_CAPACITY_SPLITS]);
	/* First windows materialized before dependencies: built, held at run,
	 * still building at run (and that remaining time), build-complete to
	 * dependency-ready (the hidden latency), not attempted, built lazily
	 * after a refusal, evicted for another window's capacity, and dropped
	 * by close/failure/release.
	 */
	size += sysfs_emit_at(buf, size,
		" early_built=%llu early_used=%llu early_late=%llu early_late_ns=%llu early_hidden_ns=%llu early_hidden_max=%llu early_skipped=%llu early_fallbacks=%llu early_evicted=%llu early_canceled=%llu",
		early[GPU_M4_DRM_PROVIDER_EARLY_BUILT], early[GPU_M4_DRM_PROVIDER_EARLY_USED],
		early[GPU_M4_DRM_PROVIDER_EARLY_LATE], early[GPU_M4_DRM_PROVIDER_EARLY_LATE_NS],
		early[GPU_M4_DRM_PROVIDER_EARLY_HIDDEN_NS], early[GPU_M4_DRM_PROVIDER_EARLY_HIDDEN_MAX],
		early[GPU_M4_DRM_PROVIDER_EARLY_SKIPPED], early[GPU_M4_DRM_PROVIDER_EARLY_FALLBACKS],
		early[GPU_M4_DRM_PROVIDER_EARLY_EVICTED], early[GPU_M4_DRM_PROVIDER_EARLY_CANCELED]);
	/* Dependency-ready to the first window's lane enqueue, activation and
	 * publication, for lazily and early built first windows.
	 */
	for (u32 k = 0; k < 2; k++)
		for (u32 i = 0; i < GPU_M4_DRM_PROVIDER_READY_PHASES; i++)
			size += sysfs_emit_at(buf, size,
				" ready_%s_%s=%llu ready_%s_%s_ns=%llu ready_%s_%s_max=%llu",
				ready_name[i], k ? "early" : "lazy", ready_phase[k][i][0],
				ready_name[i], k ? "early" : "lazy", ready_phase[k][i][1],
				ready_name[i], k ? "early" : "lazy", ready_phase[k][i][2]);
	/* Firmware upper placement: calls, refusals, wall time, scanned span in
	 * pages, live child slot reads and reservation/segment steps.
	 */
	size += sysfs_emit_at(buf, size,
		" upper_choose_calls=%llu upper_choose_failures=%llu upper_choose_ns=%llu upper_choose_pages=%llu upper_choose_reads=%llu upper_choose_steps=%llu",
		choose.calls, choose.failures, choose.ns, choose.pages, choose.reads, choose.steps);
	/* Released firmware aliases and latched release failures. */
	size += sysfs_emit_at(buf, size, " upper_released=%llu upper_release_failures=%llu",
		upper_released, upper_release_failures);
	/* Translation work: per-page layered lookups vs O(run) walks and pages,
	 * and cross-job span memo hits/misses, of the hits those answered by a
	 * block below the graph's top (a sibling's or the base's proof) and by
	 * a key inherited from a rebase/flatten base; keys new blocks inherited
	 * versus dropped because the delta touched their pages. Divide by jobs
	 * for per-job cost.
	 */
	size += sysfs_emit_at(buf, size,
		" uat_page_lookups=%llu uat_runs=%llu uat_run_lookups=%llu uat_run_pages=%llu span_hits=%llu span_misses=%llu",
		uat[GPU_M4_DRM_UAT_STAT_PAGE_LOOKUPS], uat[GPU_M4_DRM_UAT_STAT_RUNS],
		uat[GPU_M4_DRM_UAT_STAT_LOOKUPS], uat[GPU_M4_DRM_UAT_STAT_PAGES],
		uat[GPU_M4_DRM_UAT_STAT_SPAN_HITS], uat[GPU_M4_DRM_UAT_STAT_SPAN_MISSES]);
	size += sysfs_emit_at(buf, size,
		" span_below_hits=%llu span_inherited_hits=%llu span_inherited=%llu span_dropped=%llu",
		uat[GPU_M4_DRM_UAT_STAT_SPAN_BELOW_HITS], uat[GPU_M4_DRM_UAT_STAT_SPAN_INHERITED_HITS],
		uat[GPU_M4_DRM_UAT_STAT_SPAN_INHERITED], uat[GPU_M4_DRM_UAT_STAT_SPAN_DROPPED]);
	/* Span memo churn and miss attribution: keys remembered and evicted;
	 * misses never remembered (or lost by the shadow), evicted from this
	 * chain (by reuse distance: the memo size that would have kept them),
	 * remembered only on a block outside the chain (another manifest or
	 * VM), NAMED keys present for another generation; summed chain depth;
	 * then hits and misses per site (object, native, strict, named,
	 * backing, range, other). Divide by jobs for per-job figures.
	 */
	size += sysfs_emit_at(buf, size,
		" span_inserts=%llu span_evictions=%llu span_miss_absent=%llu span_miss_evicted=%llu span_reuse_512=%llu span_reuse_1k=%llu span_reuse_4k=%llu span_reuse_far=%llu span_miss_elsewhere=%llu span_miss_stale_named=%llu span_depth=%llu",
		uat[GPU_M4_DRM_UAT_STAT_SPAN_INSERTS], uat[GPU_M4_DRM_UAT_STAT_SPAN_EVICTIONS],
		uat[GPU_M4_DRM_UAT_STAT_SPAN_MISS_ABSENT], uat[GPU_M4_DRM_UAT_STAT_SPAN_MISS_EVICTED],
		uat[GPU_M4_DRM_UAT_STAT_SPAN_REUSE_512], uat[GPU_M4_DRM_UAT_STAT_SPAN_REUSE_1K],
		uat[GPU_M4_DRM_UAT_STAT_SPAN_REUSE_4K], uat[GPU_M4_DRM_UAT_STAT_SPAN_REUSE_FAR],
		uat[GPU_M4_DRM_UAT_STAT_SPAN_MISS_ELSEWHERE],
		uat[GPU_M4_DRM_UAT_STAT_SPAN_MISS_STALE_NAMED], uat[GPU_M4_DRM_UAT_STAT_SPAN_DEPTH]);
	for (u32 i = 0; i < GPU_M4_DRM_UAT_SITES; i++) {
		static const char *const site[GPU_M4_DRM_UAT_SITES] = {
			"object", "native", "strict", "named", "backing", "range", "other",
		};

		size += sysfs_emit_at(buf, size, " span_%s_hits=%llu span_%s_misses=%llu",
			site[i], uat[GPU_M4_DRM_UAT_STAT_SPAN_SITE_HITS + i],
			site[i], uat[GPU_M4_DRM_UAT_STAT_SPAN_SITE_MISSES + i]);
	}
	/* Graph construction: constructs by kind and rebase wall time; manifest
	 * entries comparison-sorted versus placed by chunk merges; table pages
	 * built versus shared; rebase private leaves kept as base runs versus
	 * resolved again; manifest chunks built versus shared by rebases; rebase
	 * leaf changes, entries visited or written, of which user-diff steps
	 * (a new user generation). Divide by jobs or rebases for per-unit cost.
	 */
	size += sysfs_emit_at(buf, size,
		" uat_constructs=%llu uat_fulls=%llu uat_appends=%llu uat_shares=%llu uat_rebases=%llu uat_rebase_ns=%llu uat_sorted=%llu uat_merged=%llu uat_tables_built=%llu uat_tables_shared=%llu uat_private_kept=%llu uat_private_resolved=%llu",
		uat[GPU_M4_DRM_UAT_STAT_CONSTRUCTS], uat[GPU_M4_DRM_UAT_STAT_FULLS],
		uat[GPU_M4_DRM_UAT_STAT_APPENDS], uat[GPU_M4_DRM_UAT_STAT_SHARES],
		uat[GPU_M4_DRM_UAT_STAT_REBASES], uat[GPU_M4_DRM_UAT_STAT_REBASE_NS],
		uat[GPU_M4_DRM_UAT_STAT_SORTED], uat[GPU_M4_DRM_UAT_STAT_MERGED],
		uat[GPU_M4_DRM_UAT_STAT_TABLES_BUILT], uat[GPU_M4_DRM_UAT_STAT_TABLES_SHARED],
		uat[GPU_M4_DRM_UAT_STAT_PRIVATE_KEPT], uat[GPU_M4_DRM_UAT_STAT_PRIVATE_RESOLVED]);
	size += sysfs_emit_at(buf, size,
		" uat_chunks_built=%llu uat_chunks_shared=%llu uat_rebase_delta=%llu uat_rebase_touched=%llu uat_user_diffed=%llu\n",
		uat[GPU_M4_DRM_UAT_STAT_CHUNKS_BUILT], uat[GPU_M4_DRM_UAT_STAT_CHUNKS_SHARED],
		uat[GPU_M4_DRM_UAT_STAT_REBASE_DELTA], uat[GPU_M4_DRM_UAT_STAT_REBASE_TOUCHED],
		uat[GPU_M4_DRM_UAT_STAT_USER_DIFFED]);
	kfree(census);
	return size;
}
static DEVICE_ATTR_RO(runtime_perf);

/* REMOVE-DEBUG: render packets and per-render timing (ns), cumulative since
 * load or the last write (any write resets). Each time is count/total/max:
 * ta, fs (stage durations), ta_fs (TA end -> FS start) or overlap (FS start
 * before TA end), gpu (TA start -> FS end), chain (previous packet member's
 * FS end -> this TA start), cross (the previous packet's last FS end -> a
 * packet's first TA start), pickup (packet publication -> its first TA
 * start: the publication gap), observe (FS end -> host receipt: the final
 * retirement) and retire (publication -> host receipt). Host samples are
 * CNTPCT converted from CNTFRQ (cntfrq=) to the GPU's 24 MHz ticks; renders
 * whose GPU cells are zero/unordered count as invalid, those not ordered
 * against the host samples as unrelated: early (GPU start before
 * publication; early_max_ns the largest lead), late (receipt before GPU end;
 * late_max_ns), no CNTFRQ, or far (beyond 10 s).
 */
static ssize_t runtime_render_timing_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	static const char *const name[M4_USER_RENDER_TIMES] = {
		"ta", "fs", "ta_fs", "overlap", "gpu", "pickup", "chain", "observe", "retire",
		"cross",
	};
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	typeof(m4_user_render_timing) t;
	ssize_t size;

	mutex_lock(&gpu->lock);
	t = m4_user_render_timing;
	mutex_unlock(&gpu->lock);
	size = sysfs_emit(buf, "render_batch=%d cntfrq=%u renders=%llu invalid=%llu unrelated=%llu early=%llu early_max_ns=%llu late=%llu late_max_ns=%llu no_frequency=%llu far=%llu packet_renders=%llu",
		READ_ONCE(render_batch), arch_timer_get_cntfrq(), t.renders, t.invalid,
		t.unrelated, t.early, t.early_max, t.late, t.late_max, t.no_frequency, t.far,
		t.packet_renders);
	for (u32 i = 0; i < M4_USER_RENDER_PACKET_SIZES; i++)
		size += sysfs_emit_at(buf, size, " packets_%u%s=%llu", i + 1,
			i + 1 == M4_USER_RENDER_PACKET_SIZES ? "plus" : "", t.packets[i]);
	for (u32 i = 0; i < M4_USER_RENDER_TIMES; i++)
		size += sysfs_emit_at(buf, size, " %s=%llu %s_ns=%llu %s_max=%llu", name[i],
			t.count[i], name[i], t.ns[i], name[i], t.max[i]);
	size += sysfs_emit_at(buf, size, "\n");
	return size;
}

static ssize_t runtime_render_timing_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);

	mutex_lock(&gpu->lock);
	memset(&m4_user_render_timing, 0, sizeof(m4_user_render_timing));
	mutex_unlock(&gpu->lock);
	return count;
}
static DEVICE_ATTR_RW(runtime_render_timing);

/* REMOVE-DEBUG: user contexts. Totals: dedicated acquisitions, VMs on the
 * shared ASID 1, released and quarantined dedicated contexts. Then each
 * context that is leased or ever bound: owner kind (dedicated/shared/free),
 * leases, generation, whether installed, and its binds by kind (already
 * resident, same-image adoption, root installation).
 */
static ssize_t runtime_asid_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_provider_runtime *p;
	ssize_t size;

	mutex_lock(&gpu->lock);
	p = gpu->hello && gpu->hello->ready ? gpu->hello->ready->provider_runtime : NULL;
	size = sysfs_emit(buf, "asid_allocs=%llu asid_fallback_shared=%llu asid_releases=%llu asid_quarantined=%llu idle_soak_busy=%llu\n",
		m4_user_asid_stats.allocs, m4_user_asid_stats.fallback,
		m4_user_asid_stats.releases, m4_user_asid_stats.quarantined,
		m4_user_idle_soak_busy);
	for (u32 asid = 1; p && p->ordinary && asid < M4_USER_ASIDS; asid++) {
		const struct m4_user_asid *a = &p->user_asid[asid];

		if (!a->leases && !a->binds[0] && !a->binds[1] && !a->binds[2] && !a->quarantined)
			continue;
		if (size > PAGE_SIZE - 160)
			break;
		size += sysfs_emit_at(buf, size,
			"asid=%u owner=%s leases=%u generation=%llu bound=%u quarantined=%u bind_resident=%llu bind_adopt=%llu bind_install=%llu\n",
			asid, a->owner ? "dedicated" : asid == M4_USER_ASID_SHARED ? "shared" : "free",
			a->leases, a->generation, !!a->low, a->quarantined, a->binds[0], a->binds[1],
			a->binds[2]);
	}
	mutex_unlock(&gpu->lock);
	return size;
}
static DEVICE_ATTR_RO(runtime_asid);

/* REMOVE-DEBUG: explicit watchdog-supervised qualification of the ordinary
 * provider. These controls never accept caller-supplied executable bytes.
 */
static struct m4_gpu_provider_runtime *m4_user_runtime_from_device(struct device *dev)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_provider_runtime *p;

	mutex_lock(&gpu->lock);
	p = gpu->hello && gpu->hello->ready ? gpu->hello->ready->provider_runtime : NULL;
	mutex_unlock(&gpu->lock);
	return p;
}

static ssize_t runtime_probe_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct m4_gpu_provider_runtime *p = m4_user_runtime_from_device(dev);
	int ret;

	if (!runtime_enable || !sysfs_streq(buf, "1"))
		return -EINVAL;
	if (!p)
		return -ENODEV;
	ret = m4_user_probe_start(p);
	return ret ?: count;
}
static DEVICE_ATTR_WO(runtime_probe);

static ssize_t runtime_probe_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct m4_gpu_provider_runtime *p = m4_user_runtime_from_device(dev);
	struct m4_user_probe *probe;
	ssize_t size;
	int ret;

	if (!p)
		return sysfs_emit(buf, "attempted=0 complete=0 error=0\n");
	ret = m4_user_probe_finish(p);
	mutex_lock(&p->hello->gpu->lock);
	probe = p->user_probe;
	if (!probe)
		size = sysfs_emit(buf, "attempted=0 complete=0 error=0\n");
	else
		size = sysfs_emit(buf,
			"attempted=1 submitted=%u complete=%u error=%d pending=%u output=%#x start_ns=%llu end_ns=%llu binding=%llu\n",
			probe->submitted, probe->complete, probe->error ?: (ret == -EAGAIN ? 0 : ret),
			ret == -EAGAIN, probe->observed_output, probe->timestamp_ns[0],
			probe->timestamp_ns[1], probe->binding_epoch);
	mutex_unlock(&p->hello->gpu->lock);
	return size;
}
static DEVICE_ATTR_RO(runtime_probe_state);

static ssize_t runtime_register_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct m4_gpu_provider_runtime *p = m4_user_runtime_from_device(dev);
	int ret;

	if (!runtime_enable || !sysfs_streq(buf, "1"))
		return -EINVAL;
	if (!p)
		return -ENODEV;
	ret = m4_user_probe_finish(p);
	if (!ret)
		ret = gpu_m4_drm_register_ready(p->frontend, true);
	return ret ?: count;
}
static DEVICE_ATTR_WO(runtime_register);

#endif
