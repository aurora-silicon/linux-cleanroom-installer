/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_PROVIDER_USERS_H
#define APPLE_GPU_M4_PROVIDER_USERS_H

/* Frontend adapters for the one retained READY runtime. The provider owns VM,
 * pin, version and job phases; these small records own only frontend handles
 * and logical queue configuration. No firmware queue is recreated on open.
 */
struct m4_user_vm {
	struct list_head garbage;
	struct m4_gpu_provider_runtime *runtime;
	struct gpu_m4_drm_provider_vm *provider_vm;
	struct gpu_m4_drm_vm *frontend_vm;
	u64 kernel_start, kernel_end;
	/* The user TTBAT context every queue, notifier record and job of this
	 * VM names: 0 until its first queue, then fixed for the VM's lifetime
	 * (m4_user_asid_acquire_locked()).
	 */
	u32 asid;
	/* The one open queue of this VM holding prebuilt spares (its sibling
	 * queues take from it): a task's extra idle queues build none.
	 */
	struct m4_user_queue *prebuilder;
	refcount_t refs;
	bool closed;
};

static void m4_user_native_prebuild_work(struct work_struct *work);
static void m4_user_native_prebuild_start_locked(struct m4_user_queue *q);
static void m4_user_render_alloc_release_locked(struct m4_user_queue *q);

struct m4_user_queue {
	struct list_head garbage, handle_put, native_idle, units;
	struct m4_gpu_provider_runtime *runtime;
	struct m4_user_vm *vm;
	struct gpu_m4_drm_provider_queue *provider_queue;
	struct m4_user_notifier *notifier;
	/* render_shared: this queue's retained render allocator, created by its
	 * first render stage and released with the queue.
	 */
	struct m4_user_render_alloc *render_alloc;
	u64 usc_base, client_id, native_idle_bytes;
	u32 priority; /* DRM_ASAHI_PRIORITY_*, privilege checked by the frontend */
	u32 execution_window; /* commands per child window, fixed at creation */
	/* The UMA slot bank of its latest accepted job: its warm units lie
	 * there, so its next stage prefers it (inflight2); and its accepted or
	 * in-flight jobs per bank.
	 */
	u32 bank, bank_holders[M4_USER_BANK_MAX];
	/* Prebuilt unclaimed 32 MiB private pools (on b->garbage links) for this
	 * queue's fresh units, built ahead from the high-water of the latest
	 * closed queue of the same task name; constructed counts fresh units.
	 */
	struct list_head spare;
	u32 spares, prebuild_want, prebuild_heaps, constructed, constructed_render;
	/* Large backings constructed units asked for (unit pools, other
	 * parameter-heap-sized ones) and how many found no spare.
	 */
	u32 need_pool, need_heap, spare_miss;
	char comm[TASK_COMM_LEN];
	struct work_struct prebuild;
	refcount_t refs;
	bool closed;
};

/* These two external claims refer to the real retained READY DMA allocations,
 * not GEM/cache stand-ins. UAT versions own the guarded claims independently
 * of this frontend handle, so closing a file cannot invalidate queued work.
 */
struct m4_user_global {
	struct m4_gpu_provider_runtime *runtime;
	struct gpu_m4_drm_provider_vm *vm;
	struct gpu_m4_drm_vm *frontend;
	struct gpu_m4_drm_provider_private_lease *claim;
	struct m4_ready_buffer *source, allocation;
	u64 va;
	u32 id;
	refcount_t refs;
};

static int m4_user_global_valid_locked(struct m4_user_global *g)
{
	struct m4_gpu_provider_runtime *p = g->runtime;
	struct m4_gpu_ready *r = p->ready;
	struct m4_ready_buffer *b = g->source;
	u64 va;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!p->ordinary || p->error || p->hello->ready != r ||
	    (g->id != GPU_M4_UMA && g->id != GPU_M4_PARAMETER) ||
	    b != &r->buffer[g->id] || !refcount_read(&g->refs) ||
	    !m4_provider_same_buffer(b, &g->allocation) ||
	    b->size != GPU_M4_DRM_PAGE_SIZE || !b->cpu ||
	    !m4_exec_backing_valid(p->hello, b, b->size, true))
		return p->error ?: -ESTALE;
	va = g->id == GPU_M4_UMA ? r->input.lower_uma : r->input.lower_parameter;
	if (g->va != va)
		return -ESTALE;
	ret = gpu_m4_drm_provider_private_check_locked(g->claim, g->vm, va, b->size);
	return ret ?: m4_provider_page_allowed(p, b->pa, b);
}

static int m4_user_global_get(void *cookie, struct gpu_m4_drm_vm *frontend,
			      const void *identity)
{
	struct m4_user_global *g = cookie;
	int ret = m4_user_global_valid_locked(g);

	if (!ret && (identity != g->source || frontend != g->frontend))
		ret = -EPERM;
	if (!ret)
		refcount_inc(&g->refs);
	return ret;
}

static void m4_user_global_put(void *cookie)
{
	struct m4_user_global *g = cookie;
	struct mutex *lock = &g->runtime->hello->gpu->lock;
	bool last;

	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	last = refcount_dec_and_test(&g->refs);
	mutex_unlock(lock);
	if (last)
		kfree(g);
}

static int m4_user_global_page(void *cookie, u64 offset, u64 *pa)
{
	struct m4_user_global *g = cookie;
	int ret = m4_user_global_valid_locked(g);

	if (!ret && (!pa || offset))
		ret = -EINVAL;
	if (!ret)
		*pa = g->allocation.pa;
	return ret;
}

static int m4_user_vm_globals(struct m4_user_vm *vm)
{
	static const u32 ids[] = { GPU_M4_UMA, GPU_M4_PARAMETER };
	struct m4_gpu_provider_runtime *p = vm->runtime;
	struct mutex *lock = &p->hello->gpu->lock;
	struct m4_user_global *g[ARRAY_SIZE(ids)] = { };
	struct gpu_m4_drm_provider_private_lease *claims[ARRAY_SIZE(ids)] = { };
	struct gpu_m4_drm_uat_private mapping[ARRAY_SIZE(ids)] = { };
	int ret = 0;

	lockdep_assert_not_held(lock);
	for (u32 i = 0; i < ARRAY_SIZE(g); i++) {
		g[i] = kzalloc(sizeof(*g[i]), GFP_KERNEL);
		if (!g[i]) {
			ret = -ENOMEM;
			goto out;
		}
		g[i]->runtime = p;
		refcount_set(&g[i]->refs, 1);
	}
	mutex_lock(lock);
	for (u32 i = 0; !ret && i < ARRAY_SIZE(g); i++) {
		struct m4_user_global *x = g[i];

		x->vm = vm->provider_vm;
		x->frontend = vm->frontend_vm;
		x->id = ids[i];
		x->source = &p->ready->buffer[ids[i]];
		x->allocation = *x->source;
		x->va = ids[i] == GPU_M4_UMA ? p->ready->input.lower_uma :
			p->ready->input.lower_parameter;
		x->claim = gpu_m4_drm_provider_private_reserve_locked(x->vm, x->va,
			GPU_M4_DRM_PAGE_SIZE, 0, GPU_M4_DRM_PAGE_SIZE);
		if (IS_ERR(x->claim)) {
			ret = PTR_ERR(x->claim);
			x->claim = NULL;
			break;
		}
		claims[i] = x->claim;
		mapping[i] = (struct gpu_m4_drm_uat_private) {
			.identity = x->source, .cookie = x,
			.get = m4_user_global_get, .put = m4_user_global_put,
			.page = m4_user_global_page, .va = x->va,
			.size = GPU_M4_DRM_PAGE_SIZE,
			.access = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE,
			.cpu_policy = GPU_M4_DRM_CPU_NC,
		};
	}
	if (!ret)
		ret = gpu_m4_drm_provider_vm_initialize_private_locked(vm->provider_vm,
			mapping, claims, ARRAY_SIZE(mapping));
	for (u32 i = 0; i < ARRAY_SIZE(claims); i++)
		if (claims[i])
			gpu_m4_drm_provider_private_put_locked(claims[i]);
	mutex_unlock(lock);
out:
	for (u32 i = 0; i < ARRAY_SIZE(g); i++)
		if (g[i])
			m4_user_global_put(g[i]);
	return ret;
}

static void m4_user_vm_put_locked(struct m4_user_vm *vm)
{
	struct m4_gpu_provider_runtime *p = vm->runtime;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (refcount_dec_and_test(&vm->refs)) {
		list_add_tail(&vm->garbage, &p->user_vm_garbage);
		schedule_work(&p->user_reap);
	}
}

static void m4_user_queue_put_locked(struct m4_user_queue *q)
{
	struct m4_gpu_provider_runtime *p = q->runtime;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (refcount_dec_and_test(&q->refs)) {
		list_add_tail(&q->garbage, &p->user_queue_garbage);
		schedule_work(&p->user_reap);
	}
}

/* REMOVE-DEBUG: user context leases under the GPU mutex: dedicated
 * acquisitions, VMs that fell back to the shared ASID 1, released (unbound
 * and freed) dedicated contexts, and failed releases left quarantined.
 */
static struct {
	u64 allocs, fallback, releases, quarantined;
} m4_user_asid_stats;

/* A VM's first queue binds it to a user context for its whole lifetime: the
 * lowest free dedicated ASID 2..63, else the shared ASID 1; always the shared
 * one unless user_asids is set. The VM itself holds that lease until its
 * final put. A free dedicated context is unbound: either never installed or
 * cleared by its release.
 */
static int m4_user_asid_acquire_locked(struct m4_gpu_provider_runtime *p, struct m4_user_vm *vm)
{
	u32 asid = M4_USER_ASID_SHARED;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (vm->asid)
		return 0;
	for (u32 i = M4_USER_ASID_SHARED + 1; user_asids && i < M4_USER_ASIDS; i++) {
		const struct m4_user_asid *a = &p->user_asid[i];

		if (!a->leases && !a->owner && !a->quarantined && !a->resident && !a->low) {
			asid = i;
			break;
		}
	}
	if (p->user_asid[asid].leases == U32_MAX)
		return -EOVERFLOW;
	if (asid == M4_USER_ASID_SHARED) {
		if (user_asids)
			m4_user_asid_stats.fallback++;
	} else {
		p->user_asid[asid].owner = vm;
		m4_user_asid_stats.allocs++;
	}
	p->user_asid[asid].leases++;
	vm->asid = asid;
	/* REMOVE-DEBUG: fault attribution, once per VM's first queue. */
	dev_info(p->hello->gpu->dev, "GPU VM asid=%u vm=%px pid=%d comm=%s\n",
		 asid, vm, task_tgid_nr(current), current->comm);
	/* S3c: an authority VM's tree invalidates this ASID on removals. */
	gpu_m4_drm_provider_vm_set_asid_locked(vm->provider_vm, asid);
	return 0;
}

/* One more holder (a notifier) of a context its VM already leases. */
static int m4_user_asid_get_locked(struct m4_gpu_provider_runtime *p, u32 asid)
{
	struct m4_user_asid *a;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!asid || asid >= M4_USER_ASIDS)
		return -EINVAL;
	a = &p->user_asid[asid];
	if (!a->leases || a->quarantined)
		return -ESTALE;
	if (a->leases == U32_MAX)
		return -EOVERFLOW;
	a->leases++;
	return 0;
}

/* Final healthy destruction of a released dedicated context's only VM: no
 * VM, notifier, queue, retained native owner or job names it any more. Under
 * handoff both TTBAT words are cleared and read back; after its release the
 * stores are ordered, ASIDE1OS invalidates exactly that ASID and DSB SY
 * completes it; only then does the core drop the resident (its generation is
 * kept, so the next installation here advances it). A context never
 * installed has nothing to clear.
 */
static int m4_user_asid_unbind_locked(struct m4_gpu_provider_runtime *p, u32 asid)
{
	struct m4_user_asid *a = &p->user_asid[asid];
	void __iomem *ttbs = p->hello->gpu->ttbs + 16 * asid;
	int ret, release;

	if (!a->resident && !a->low)
		return 0;
	ret = p->error ?: m4_user_live_health_locked(p);
	if (!ret)
		ret = m4_user_binding_check_locked(p);
	if (!ret)
		ret = m4_provider_fixed_live(p->hello, asid, a->low, 0);
	if (ret)
		return ret;
	ret = m4_hello_lock(p->hello);
	if (ret)
		return ret;
	ret = m4_provider_fixed_live(p->hello, asid, a->low, 0);
	if (!ret) {
		p->writing = true;
		m4_provider_health_void(p);
		writeq(0, ttbs);
		writeq(0, ttbs + 8);
		if (readq(ttbs) || readq(ttbs + 8))
			ret = -EIO;
	}
	m4_hello_unlock(p->hello);
	release = readb(p->hello->handoff + 0x10) ? -EIO : 0;
	if (!ret)
		ret = release;
	if (ret)
		return ret;
	m4_provider_asid_sync(asid);
	ret = gpu_m4_drm_provider_asid_unbind_complete_locked(p->provider, asid, readq(ttbs),
		readq(ttbs + 8));
	if (ret)
		return ret;
	a->resident = NULL;
	a->low = 0;
	p->user_binding_revision++;
	p->writing = false;
	return 0;
}

/* Drops one lease. The last one frees a dedicated context after clearing it;
 * the shared ASID 1 keeps its resident root, as before. A release that fails
 * leaves the context quarantined, never reused: before any store its state is
 * unchanged and consistent, after one the runtime is faulted.
 */
static void m4_user_asid_put_locked(struct m4_gpu_provider_runtime *p, u32 asid)
{
	struct m4_user_asid *a;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!asid || asid >= M4_USER_ASIDS)
		return; /* A VM that never had a queue holds none. */
	a = &p->user_asid[asid];
	if (WARN_ON(!a->leases) || --a->leases || asid == M4_USER_ASID_SHARED)
		return;
	a->owner = NULL;
	ret = m4_user_asid_unbind_locked(p, asid);
	if (!ret) {
		m4_user_asid_stats.releases++;
		return;
	}
	a->quarantined = true;
	m4_user_asid_stats.quarantined++;
	dev_warn(p->hello->gpu->dev, "GPU user context %u release failed (%d); quarantined\n",
		 asid, ret);
	if (p->writing)
		m4_provider_fault_locked(p, ret);
}

static int __maybe_unused m4_user_queue_get_locked(void *owner, void *queue)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_user_queue *q = queue;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!q || q->runtime != p || !p->ordinary || p->error || q->closed ||
	    !refcount_inc_not_zero(&q->refs))
		return p->error ?: -ESHUTDOWN;
	return 0;
}

static void __maybe_unused m4_user_runtime_queue_put_locked(void *owner, void *queue)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_user_queue *q = queue;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (WARN_ON(!q || q->runtime != p))
		return;
	m4_user_queue_put_locked(q);
}

/* Also callable from the job core's outside-lock collector. Queue/core/VM puts
 * may recurse through this entrance; only independent work performs them.
 */
static void m4_user_reap(void *owner)
{
	struct m4_gpu_provider_runtime *p = owner;

	schedule_work(&p->user_reap);
}

static u32 m4_user_native_cell_return(struct m4_gpu_provider_runtime *p);

/* A deferred housekeeping collection is due: run it from user_reap. */
static void m4_user_collect_work(struct work_struct *work)
{
	struct m4_gpu_provider_runtime *p = container_of(to_delayed_work(work),
		struct m4_gpu_provider_runtime, user_collect);

	schedule_work(&p->user_reap);
}

static void m4_user_reap_work(struct work_struct *work)
{
	struct m4_gpu_provider_runtime *p = container_of(work,
		struct m4_gpu_provider_runtime, user_reap);
	struct mutex *lock = &p->hello->gpu->lock;
	struct m4_user_queue *q;
	struct m4_user_vm *vm;

	for (;;) {
		mutex_lock(lock);
		if (!list_empty(&p->user_handle_put)) {
			q = list_first_entry(&p->user_handle_put, struct m4_user_queue, handle_put);
			list_del_init(&q->handle_put);
			mutex_unlock(lock);
			gpu_m4_drm_provider_queue_put(q->provider_queue);
			continue;
		}
		if (!list_empty(&p->user_queue_garbage)) {
			q = list_first_entry(&p->user_queue_garbage, struct m4_user_queue, garbage);
			list_del_init(&q->garbage);
			/* No stage or unit of the queue remains. */
			m4_user_render_alloc_release_locked(q);
			m4_user_notifier_put_locked(q->notifier);
			m4_user_vm_put_locked(q->vm);
			mutex_unlock(lock);
			kfree(q);
			continue;
		}
		if (!list_empty(&p->user_vm_garbage)) {
			vm = list_first_entry(&p->user_vm_garbage, struct m4_user_vm, garbage);
			list_del_init(&vm->garbage);
			/* Every queue, hence every retained owner, is already gone. */
			m4_user_asid_put_locked(p, vm->asid);
			mutex_unlock(lock);
			gpu_m4_drm_vm_put(vm->frontend_vm);
			kfree(vm);
			continue;
		}
		mutex_unlock(lock);
		break;
	}
	if (p->ts_aliases)
		gpu_m4_drm_timestamp_alias_reap(p->ts_aliases);
	/* Checked closed-queue cut and cell-pressure eviction under the GPU
	 * mutex, then the outside-lock job/history/stage/BO reapers, then the
	 * provider reaper, which can release a captured version's last hold on an
	 * old cell address claim. Only after that can the locked pending-cell
	 * scan detach drained wrappers; their normal disposal runs as ONE bounded
	 * follow-up pass. Any further put reschedules this work; a genuinely busy
	 * holder never spins here.
	 */
	m4_user_collect_native(p);
	gpu_m4_drm_provider_reap(p->provider);
	if (m4_user_native_cell_return(p)) {
		m4_user_collect_native(p);
		gpu_m4_drm_provider_reap(p->provider);
	}
}

static int m4_user_read_time(void *owner, u64 *timestamp_ns)
{
	u32 frequency = arch_timer_get_cntfrq();

	if (!timestamp_ns || !frequency)
		return -EINVAL;
	*timestamp_ns = mul_u64_u32_div(__arch_counter_get_cntpct(), NSEC_PER_SEC, frequency);
	return 0;
}

/* VM_STATUS report of a failed shared execution domain: sticky latched
 * failures (runtime, READY, firmware callback, crashed RTKit, provider core)
 * read locklessly under the frontend mutex. There is no recovery, so a
 * failure never clears. Not a publication gate.
 */
static int m4_user_health(void *owner)
{
	struct m4_gpu_provider_runtime *p = owner;
	int ret = READ_ONCE(p->error) ?: READ_ONCE(p->ready->error);

	if (!ret && (atomic_read(&p->hello->callback_error) ||
		     apple_rtkit_is_crashed(p->hello->rtkit)))
		ret = -EIO;
	return ret ?: gpu_m4_drm_provider_error(READ_ONCE(p->provider));
}

static void m4_user_init(struct m4_gpu_provider_runtime *p)
{
	INIT_LIST_HEAD(&p->user_vm_garbage);
	INIT_LIST_HEAD(&p->user_queue_garbage);
	INIT_LIST_HEAD(&p->user_handle_put);
	INIT_LIST_HEAD(&p->user_native_garbage);
	INIT_LIST_HEAD(&p->user_native_history_garbage);
	INIT_LIST_HEAD(&p->user_native_settle);
	INIT_LIST_HEAD(&p->user_native_retained);
	INIT_LIST_HEAD(&p->user_native_units);
	INIT_LIST_HEAD(&p->user_job_garbage);
	INIT_WORK(&p->user_reap, m4_user_reap_work);
	INIT_DELAYED_WORK(&p->user_collect, m4_user_collect_work);
	p->user_stage_cells = stage_cells;
	p->user_render_stage_waits = stage_cells && render_stage_waits;
	p->user_inflight2 = inflight2;
	p->user_inflight2_wide = inflight2 && inflight2_wide;
	p->user_inflight_max = !inflight2 ? 1 :
		clamp_t(u32, READ_ONCE(inflight_jobs), 2, M4_USER_INFLIGHT_MAX);
	p->user_bank_count = !inflight2 ? M4_USER_BANK_COUNT :
		p->user_inflight_max > 2 ? M4_USER_INFLIGHT4_BANKS : M4_USER_INFLIGHT2_BANKS;
	p->user_bank_size = !inflight2 ? M4_USER_BANK_SIZE :
		p->user_inflight_max > 2 ? M4_USER_INFLIGHT4_BANK_SIZE : M4_USER_INFLIGHT2_BANK_SIZE;
}

/* Scheduler credits per new queue: batches the scheduler hands to the
 * provider at once, each materializing its windows while earlier ones
 * execute, at least the packet cap. Above it, the next batches are built
 * and wait accepted in the lane FIFO while the cap's packets run, so a
 * completion is refilled without a run/materialize hop. 0: the packet cap.
 */
static unsigned int queue_credits = 8;
module_param(queue_credits, uint, 0644);
MODULE_PARM_DESC(queue_credits, "Scheduler credits per new queue, packet cap..16 (default 8; 0: the packet cap)");

static u32 m4_user_queue_credits(const struct m4_gpu_provider_runtime *p)
{
	u32 credits = READ_ONCE(queue_credits);

	if (!p->user_inflight2)
		return 0;
	return credits ? clamp_t(u32, credits, p->user_inflight_max,
				 GPU_M4_DRM_SCHED_PROVIDER_MAX_CREDITS) : p->user_inflight_max;
}

static void *m4_user_vm_create(void *owner, struct gpu_m4_drm_vm *frontend,
			       u64 kernel_start, u64 kernel_end)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_user_vm *vm;
	int ret;

	vm = kzalloc(sizeof(*vm), GFP_KERNEL);
	if (!vm)
		return ERR_PTR(-ENOMEM);
	vm->runtime = p;
	vm->frontend_vm = frontend;
	vm->kernel_start = kernel_start;
	vm->kernel_end = kernel_end;
	refcount_set(&vm->refs, 1);
	INIT_LIST_HEAD(&vm->garbage);
	/* Actual provider context/pins retain this frontend independently. This
	 * handle record has a separate ref for surviving logical queue callbacks.
	 */
	gpu_m4_drm_vm_get(frontend);
	vm->provider_vm = gpu_m4_drm_provider_vm_create_deferred(p->provider, frontend,
		kernel_start, kernel_end);
	if (IS_ERR(vm->provider_vm)) {
		int ret = PTR_ERR(vm->provider_vm);

		gpu_m4_drm_vm_put(frontend);
		kfree(vm);
		m4_user_reap(p);
		return ERR_PTR(ret);
	}
	ret = m4_user_vm_globals(vm);
	if (ret) {
		mutex_lock(&p->hello->gpu->lock);
		gpu_m4_drm_provider_vm_close_locked(vm->provider_vm);
		vm->closed = true;
		m4_user_vm_put_locked(vm);
		mutex_unlock(&p->hello->gpu->lock);
		m4_user_reap(p);
		return ERR_PTR(ret);
	}
	m4_user_reap(p);
	return vm;
}

static int m4_user_vm_destroy(void *owner, void *backend)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_user_vm *vm = backend;
	struct mutex *lock = &p->hello->gpu->lock;
	int ret;

	mutex_lock(lock);
	ret = gpu_m4_drm_provider_vm_destroy_locked(vm->provider_vm);
	if (!ret) {
		vm->closed = true;
		m4_user_vm_put_locked(vm);
	}
	mutex_unlock(lock);
	if (!ret)
		m4_user_reap(p);
	return ret;
}

static void m4_user_vm_close(void *owner, void *backend)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_user_vm *vm = backend;
	struct mutex *lock = &p->hello->gpu->lock;

	mutex_lock(lock);
	if (!vm->closed) {
		vm->closed = true;
		gpu_m4_drm_provider_vm_close_locked(vm->provider_vm);
		m4_user_vm_put_locked(vm);
	}
	mutex_unlock(lock);
	m4_user_reap(p);
}

static int m4_user_vm_bind(void *owner, void *backend,
			   const struct gpu_m4_drm_bind *binds, u32 count)
{
	struct m4_user_vm *vm = backend;
	int ret = gpu_m4_drm_provider_vm_bind_deferred(vm->provider_vm, binds, count);

	m4_user_reap(owner);
	return ret;
}

static void m4_user_bo_close(void *owner, void *backend, struct drm_gem_object *object)
{
	struct m4_user_vm *vm = backend;

	gpu_m4_drm_provider_vm_bo_close(vm->provider_vm, object);
	m4_user_reap(owner);
}

static void *m4_user_object_bind(void *owner, void *vm,
			       struct drm_gem_object *object, u64 offset, u64 size)
{
	struct gpu_m4_drm_provider_timestamp_config config = m4_provider_ts_timestamp_config(owner);

	/* vm==NULL is the ordinary unchanged-Mesa file-global timestamp binding. */
	return gpu_m4_drm_provider_timestamp_bind(&config, object, offset, size);
}

static void m4_user_object_unbind(void *owner, void *vm, void *object)
{
	gpu_m4_drm_provider_timestamp_put(object);
}

/* Child window size for queues created from now on (A/B switch): a public
 * submit is materialized as children of at most this many commands (and at
 * most 16 per class), each published whole. 8 restores the earlier bound.
 */
static unsigned int window_commands = 8;
module_param(window_commands, uint, 0644);
MODULE_PARM_DESC(window_commands, "Commands per bounded child window for new queues (1..32, default 8; at most 16 per class)");

static void *m4_user_queue_create(void *owner, void *backend, u32 priority, u64 usc_base)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_user_vm *vm = backend;
	struct mutex *lock = &p->hello->gpu->lock;
	struct m4_user_queue *q;
	struct gpu_m4_drm_provider_queue_config config;
	int ret;

	if (!p->user_runtime)
		return ERR_PTR(-EOPNOTSUPP);
	q = kzalloc(sizeof(*q), GFP_KERNEL);
	if (!q)
		return ERR_PTR(-ENOMEM);
	q->runtime = p;
	q->vm = vm;
	q->usc_base = usc_base;
	q->priority = priority;
	refcount_set(&q->refs, 1);
	INIT_LIST_HEAD(&q->garbage);
	INIT_LIST_HEAD(&q->handle_put);
	INIT_LIST_HEAD(&q->native_idle);
	INIT_LIST_HEAD(&q->spare);
	get_task_comm(q->comm, current);
	INIT_WORK(&q->prebuild, m4_user_native_prebuild_work);
	INIT_LIST_HEAD(&q->units);
	mutex_lock(lock);
	ret = p->error ?: (vm->closed ? -ESHUTDOWN : 0);
	if (!ret && p->user_client_id == U64_MAX)
		ret = -EOVERFLOW;
	/* Before the notifier, whose records name the VM's context. */
	if (!ret)
		ret = m4_user_asid_acquire_locked(p, vm);
	if (!ret) {
		u32 window = READ_ONCE(window_commands);

		q->client_id = ++p->user_client_id;
		refcount_inc(&vm->refs);
		window = window && window <= GPU_M4_DRM_PROVIDER_WINDOW_MAX_COMMANDS ?
			window : GPU_M4_DRM_PROVIDER_WINDOW_MAX_COMMANDS;
		/* Commands of one class in a child each lease a distinct slot of
		 * its one bank: banks of 8 (more than two packets in flight)
		 * bound the whole child window to the bank.
		 */
		q->execution_window = window;
		if (p->user_bank_size && p->user_bank_size < GPU_M4_DRM_PROVIDER_WINDOW_MAX_PER_CLASS)
			q->execution_window = window = min_t(u32, window, p->user_bank_size);
		window = min_t(u32, window, GPU_M4_DRM_PROVIDER_WINDOW_MAX_PER_CLASS);
		if (window > p->user_bank_used)
			p->user_bank_used = window;
	}
	mutex_unlock(lock);
	if (ret) {
		kfree(q);
		return ERR_PTR(ret);
	}
	q->notifier = m4_user_notifier_create(q);
	if (IS_ERR(q->notifier)) {
		ret = PTR_ERR(q->notifier);
		q->notifier = NULL;
		mutex_lock(lock);
		m4_user_queue_put_locked(q);
		mutex_unlock(lock);
		return ERR_PTR(ret);
	}
	config = (struct gpu_m4_drm_provider_queue_config) {
		.dev = p->hello->gpu->dev, .vm = vm->provider_vm, .params = &p->params,
		.kernel_start = vm->kernel_start, .kernel_end = vm->kernel_end,
		.usc_base = usc_base, .client_id = q->client_id, .priority = priority,
		/* Periodic whole-submit hang check, independent of each actually
		 * published command's two-second lane clock. The timeout callback
		 * consults that exact lane owner before faulting a progressing batch.
		 */
		.timeout = msecs_to_jiffies(30000), .runtime_owner = p,
		/* Bounded native execution: a larger public submit is materialized
		 * one real child of at most window_commands commands (16 per
		 * class) at a time. Public UAPI acceptance remains
		 * max_commands_per_submission.
		 */
		.execution_window = q->execution_window,
		/* Packets in flight: the next batches of this queue may run
		 * (and materialize) while earlier ones execute.
		 */
		.credit_limit = m4_user_queue_credits(p),
		.runtime_queue = q, .runtime = p->user_runtime,
	};
	q->provider_queue = gpu_m4_drm_provider_queue_create(&config);
	if (IS_ERR(q->provider_queue)) {
		ret = PTR_ERR(q->provider_queue);
		mutex_lock(lock);
		m4_user_queue_put_locked(q);
		mutex_unlock(lock);
		return ERR_PTR(ret);
	}
	mutex_lock(lock);
	m4_user_native_prebuild_start_locked(q);
	mutex_unlock(lock);
	return q;
}

static void m4_user_queue_drop_handle(struct m4_user_queue *q)
{
	struct m4_gpu_provider_runtime *p = q->runtime;
	struct mutex *lock = &p->hello->gpu->lock;

	mutex_lock(lock);
	if (!q->closed) {
		q->closed = true;
		/* Schedules the deferred cold scan; it asserts no completion. */
		m4_user_native_queue_close_locked(q);
		/* Hold the handle record until the independent core queue put, even
		 * if scheduler teardown immediately drops its runtime reference.
		 */
		list_add_tail(&q->handle_put, &p->user_handle_put);
		m4_user_queue_put_locked(q);
	}
	mutex_unlock(lock);
	m4_user_reap(p);
}

static int m4_user_queue_destroy(void *owner, void *backend)
{
	struct m4_user_queue *q = backend;
	int ret = gpu_m4_drm_provider_queue_try_close(q->provider_queue);

	if (!ret)
		m4_user_queue_drop_handle(q);
	return ret;
}

static void m4_user_queue_close(void *owner, void *backend)
{
	struct m4_user_queue *q = backend;

	gpu_m4_drm_provider_queue_close(q->provider_queue);
	m4_user_queue_drop_handle(q);
}

static struct dma_fence *m4_user_submit(void *owner, void *backend,
				     const struct gpu_m4_drm_submit *submit)
{
	struct m4_user_queue *q = backend;

	return gpu_m4_drm_provider_queue_submit(q->provider_queue, submit);
}

static const struct gpu_m4_drm_ops m4_user_frontend_ops = {
	.session_get = m4_provider_session_get, .session_put = m4_provider_session_put,
	.read_time = m4_user_read_time, .health = m4_user_health,
	.vm_create = m4_user_vm_create,
	.vm_destroy = m4_user_vm_destroy, .vm_close = m4_user_vm_close,
	.vm_bind = m4_user_vm_bind, .bo_close = m4_user_bo_close,
	.object_bind = m4_user_object_bind, .object_unbind = m4_user_object_unbind,
	.queue_create = m4_user_queue_create, .queue_destroy = m4_user_queue_destroy,
	.queue_close = m4_user_queue_close, .submit = m4_user_submit,
};

/* Bounded live identity check of every user context for nested protected
 * packet consumers: the provider core's per-ASID resident and generation
 * equal the mirrored ones, a context is installed exactly while it has a
 * resident, and only dedicated contexts with leases have an owner. No
 * registered-inventory walk: m4_user_asid_health_locked() proves one
 * context's resident that way. Complete graph health is still performed by
 * the outer publication/poll entrance.
 */
static int m4_user_binding_check_locked(struct m4_gpu_provider_runtime *p)
{
	lockdep_assert_held(&p->hello->gpu->lock);
	if (!p->ordinary || !p->provider || !p->frontend || p->error)
		return p->error ?: -EPROTO;
	/* The ordinary owner never uses the fixed manifests' ASID-1 mirror. */
	if (p->selected || p->resident || p->generation)
		return -ESTALE;
	for (u32 asid = 1; asid < M4_USER_ASIDS; asid++) {
		const struct m4_user_asid *a = &p->user_asid[asid];

		if (gpu_m4_drm_provider_asid_resident_locked(p->provider, asid) != a->resident ||
		    gpu_m4_drm_provider_asid_generation_locked(p->provider, asid) != a->generation ||
		    !a->resident != !a->low || (a->resident && !a->generation) ||
		    (a->owner && (asid == M4_USER_ASID_SHARED || !a->leases)))
			return -ESTALE;
	}
	return 0;
}

/* One user context's exact binding through the core's registered inventory
 * (no pending installation, resident identity, generation), its refreshed
 * root page and its installed TTBAT word. An unbound context has none.
 */
static int m4_user_asid_health_locked(struct m4_gpu_provider_runtime *p, u32 asid)
{
	const struct m4_user_asid *a = &p->user_asid[asid];
	u64 epoch, low, high;
	u8 wire;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!a->resident)
		return 0;
	ret = gpu_m4_drm_provider_asid_binding_locked(p->provider, asid, a->resident,
		&epoch, &wire);
	if (!ret && epoch != a->generation)
		ret = -ESTALE;
	if (ret) {
		/* REMOVE-DEBUG: one-shot attribution, failure path only. */
		dev_warn_once(p->hello->gpu->dev, "REMOVE-DEBUG GPU user health binding %d asid=%u generation=%llu (provider-users:binding)\n",
			      ret, asid, a->generation);
		return ret;
	}
	/* Refresh rescans the whole manifest once per binding/fault revision and
	 * otherwise checks owned tables and the live root page; the roots come
	 * from the checked getter, not a second complete graph verification.
	 */
	ret = gpu_m4_drm_provider_version_refresh_locked(a->resident);
	if (ret) {
		dev_warn_once(p->hello->gpu->dev, "REMOVE-DEBUG GPU user health resident refresh %d asid=%u (provider-users:refresh)\n",
			      ret, asid);
		return ret;
	}
	ret = gpu_m4_drm_provider_version_roots_locked(a->resident, &low, &high);
	if (!ret && (m4_provider_ttbat_low(low, asid) != a->low || high))
		ret = -ESTALE;
	if (ret)
		dev_warn_once(p->hello->gpu->dev, "REMOVE-DEBUG GPU user health resident roots %d asid=%u (provider-users:roots)\n",
			      ret, asid);
	return ret;
}

static int m4_user_health_locked(struct m4_gpu_provider_runtime *p)
{
	int ret = m4_user_binding_check_locked(p);

	/* Binding epochs here and the callers' TTBAT readbacks are live identity. */
	if (ret) {
		/* REMOVE-DEBUG: one-shot attribution, failure path only. */
		dev_warn_once(p->hello->gpu->dev, "REMOVE-DEBUG GPU user health contexts %d p_error=%d (provider-users:contexts)\n",
			      ret, p->error);
		return ret;
	}
	for (u32 asid = 1; !ret && asid < M4_USER_ASIDS; asid++)
		ret = m4_user_asid_health_locked(p, asid);
	return ret;
}

static int m4_user_upper_gate(void *cookie, u32 gates, bool trial)
{
	struct m4_gpu_hello *h = cookie;
	struct m4_gpu_provider_runtime *p = h->ready->provider_runtime;
	int ret;

	lockdep_assert_held(&h->gpu->lock);
	if (!p || !p->ordinary || trial || !gates ||
	    (gates & ~(GPU_M4_DRM_UPPER_EXISTING_CHILD | GPU_M4_DRM_UPPER_NEW_PARENT |
		       GPU_M4_DRM_UPPER_TIMESTAMP_GATE)) ||
	    !p->complete || !h->ready->complete || !h->ready->policy_complete ||
	    !p->user_lane || !p->user_starting.item || !p->user_starting.job)
		return -EPERM;
	/* Only the starting owner publishes new firmware aliases. */
	ret = gpu_m4_drm_provider_lane_check_active_locked(p->user_lane, p->provider,
		p->user_starting.item, p->user_starting.job);
	/* Fixed observations only: upper repeats this while holding handoff. The
	 * real job/canonical backing callbacks separately check cache protection.
	 */
	return ret ?: m4_provider_fixed_at(h, 0, 0, 0);
}

static int m4_user_upper_seed(void *cookie,
	const struct gpu_m4_drm_upper_backing *backing, u64 va,
	enum gpu_m4_drm_upper_class class)
{
	struct m4_gpu_hello *h = cookie;
	struct m4_gpu_provider_runtime *p = h->ready->provider_runtime;
	int ret;

	lockdep_assert_held(&h->gpu->lock);
	if (!p || !p->ordinary || !p->prepared || p->error ||
	    h->gpu->firmware_start_attempted || !h->ready->published)
		return -EPERM;
	ret = m4_gpu_stopped(h->gpu);
	return ret ?: m4_arena_seed_locked(h, backing, va, class);
}

static int m4_user_upper_handoff_get(void *cookie, u32 ceiling_ms)
{
	return ceiling_ms == 100 ? m4_hello_lock(cookie) : -EINVAL;
}

static int m4_user_upper_handoff_put(void *cookie)
{
	struct m4_gpu_hello *h = cookie;

	m4_hello_unlock(h);
	return readb(h->handoff + 0x10) ? -EIO : 0;
}

static int m4_user_upper_clean(void *cookie, void *cpu, u32 size)
{
	struct m4_gpu_hello *h = cookie;
	struct gpu_m4_drm_upper_table child;
	bool owned = m4_upper_contains(h->shared, cpu, size);

	lockdep_assert_held(&h->gpu->lock);
	/* Enumerate only the allocator's retained AP views, never follow a PA from
	 * firmware memory. The lookup authenticates each page's actual owner.
	 */
	for (u32 i = 0; !owned && i < 2048; i++) {
		if (!gpu_m4_drm_upper_table_locked(h->ready->upper, i, &child))
			owned = m4_upper_contains(child.cpu, cpu, size);
	}
	if (!owned)
		return -EPERM;
	dcache_clean_poc((unsigned long)cpu, (unsigned long)cpu + size);
	return 0;
}

/* Publication completion only: every published slot was zero before, and
 * ASIDE1OS does not remove the global leaves a release clears.
 */
static int m4_user_upper_asid0_sync(void *cookie)
{
	struct m4_gpu_hello *h = cookie;

	lockdep_assert_held(&h->gpu->lock);
	asm volatile("dsb osh\n\t.arch armv8.4-a\n\ttlbi aside1os, %0\n\tdsb sy"
		: : "r" (0ULL) : "memory");
	return 0;
}

/* One outer-shareable by-VA invalidation, ASID 0, of a 16 KiB page. Unlike
 * ASIDE1OS it also removes global final-level entries (these upper leaves
 * have nG clear). Operand: (VA >> 12) & 0xffffffffffc, ASID bits zero.
 */
static void m4_user_upper_tlbi_vae1os(u64 operand)
{
	asm volatile(".arch armv8.4-a\n\ttlbi vae1os, %0" : : "r" (operand) : "memory");
}

static void m4_user_upper_dsb_osh(void)
{
	asm volatile("dsb osh" : : : "memory");
}

static void m4_user_upper_dsb_sy(void)
{
	asm volatile("dsb sy" : : : "memory");
}

/* Release invalidation: the cleared, cleaned leaf stores complete (DSB OSH),
 * then one VAE1OS per released page, then DSB SY: on return no PE or the
 * firmware's shared TLB holds a translation of [va, va + size).
 */
static int m4_user_upper_asid0_range_sync(void *cookie, u64 va, u64 size)
{
	struct m4_gpu_hello *h = cookie;

	lockdep_assert_held(&h->gpu->lock);
	if (!size || ((va | size) & (GPU_M4_DRM_PAGE_SIZE - 1)) ||
	    va < GPU_M4_DRM_UPPER_START || va >= GPU_M4_DRM_UPPER_END ||
	    size > GPU_M4_DRM_UPPER_END - va)
		return -EINVAL;
	m4_user_upper_dsb_osh();
	for (u64 offset = 0; offset < size; offset += GPU_M4_DRM_PAGE_SIZE)
		m4_user_upper_tlbi_vae1os(((va + offset) >> 12) & 0xffffffffffcULL);
	m4_user_upper_dsb_sy();
	return 0;
}

static int m4_user_upper_commit(void *cookie,
	const struct gpu_m4_drm_upper_update *updates, u32 count)
{
	struct m4_gpu_hello *h = cookie;
	struct m4_gpu_ready *r = h->ready;
	struct gpu_m4_drm_upper_table child;
	u32 parents = 0;

	lockdep_assert_held(&h->gpu->lock);
	if (!updates || !count)
		return -EINVAL;
	/* Validate the entire bounded zero-slot transition before adopting it.
	 * Other AP children stay under the actual upper owner's strict inventory;
	 * only the original READY child also has a bootstrap expected image.
	 */
	for (u32 i = 0; i < count; i++) {
		const struct gpu_m4_drm_upper_update *u = &updates[i];

		if (u->index >= 2048 ||
		    get_unaligned_le64(u->table_cpu + u->index * 8) != u->value)
			return -EPROTO;
		if (u->parent) {
			if (u->table_cpu != h->shared || u->table_pa != h->region[M4_SHARED].start ||
			    get_unaligned_le64(r->expected[M4_READY_TABLES] + u->index * 8) ||
			    gpu_m4_drm_upper_table_locked(r->upper, u->index, &child) ||
			    child.owner != r->upper || child.class != GPU_M4_DRM_UPPER_INHERITED_AP ||
			    u->value != (child.pa | 3))
				return -EPROTO;
			parents++;
		} else if (u->table_cpu == page_address(r->table[M4_UPPER_LEAF]) &&
			   (u->table_pa != page_to_phys(r->table[M4_UPPER_LEAF]) ||
			    get_unaligned_le64(r->expected[M4_UPPER_LEAF] + u->index * 8))) {
			return -EPROTO;
		}
	}
	if (parents > ARRAY_SIZE(r->upper_extra) - r->upper_extra_count)
		return -ENOSPC;
	m4_provider_health_void(r->provider_runtime);
	for (u32 i = 0; i < count; i++) {
		const struct gpu_m4_drm_upper_update *u = &updates[i];

		if (u->parent) {
			gpu_m4_drm_upper_table_locked(r->upper, u->index, &child);
			put_unaligned_le64(u->value, r->expected[M4_READY_TABLES] + u->index * 8);
			r->upper_extra[r->upper_extra_count++] = child;
		} else if (u->table_cpu == page_address(r->table[M4_UPPER_LEAF])) {
			put_unaligned_le64(u->value, r->expected[M4_UPPER_LEAF] + u->index * 8);
		}
	}
	return 0;
}

/* Adopt exactly these released leaf slots: zero, live and cleared, in a
 * retained AP view of the upper owner (never a parent, foreign or unknown
 * table). The READY bootstrap child additionally drops the released PTE from
 * its own expected image, which must still hold exactly that value.
 */
static int m4_user_upper_release_commit(void *cookie,
	const struct gpu_m4_drm_upper_update *updates, u32 count)
{
	struct m4_gpu_hello *h = cookie;
	struct m4_gpu_ready *r = h->ready;
	void *leaf = page_address(r->table[M4_UPPER_LEAF]), *checked = NULL;
	struct gpu_m4_drm_upper_table child;

	lockdep_assert_held(&h->gpu->lock);
	if (!updates || !count)
		return -EINVAL;
	for (u32 i = 0; i < count; i++) {
		const struct gpu_m4_drm_upper_update *u = &updates[i];
		bool owned = u->table_cpu == checked;

		if (u->parent || u->index >= 2048 || u->value || !u->previous ||
		    get_unaligned_le64(u->table_cpu + u->index * 8))
			return -EPROTO;
		if (u->table_cpu == leaf) {
			if (u->table_pa != page_to_phys(r->table[M4_UPPER_LEAF]) ||
			    get_unaligned_le64(r->expected[M4_UPPER_LEAF] + u->index * 8) != u->previous)
				return -EPROTO;
			continue;
		}
		for (u32 m = 0; !owned && m < 2048; m++)
			owned = !gpu_m4_drm_upper_table_locked(r->upper, m, &child) &&
				child.cpu == u->table_cpu && child.pa == u->table_pa;
		if (!owned)
			return -EPROTO;
		checked = u->table_cpu;
	}
	m4_provider_health_void(r->provider_runtime);
	for (u32 i = 0; i < count; i++)
		if (updates[i].table_cpu == leaf)
			put_unaligned_le64(0, r->expected[M4_UPPER_LEAF] + updates[i].index * 8);
	return 0;
}

static void m4_user_upper_fault(void *cookie, int error)
{
	struct m4_gpu_hello *h = cookie;

	m4_provider_fault_locked(h->ready->provider_runtime, error);
}

/* The native upper owner is created BEFORE RUN, unlike the old diagnostic
 * publisher. Its cold zero snapshot is not a post-HELLO firmware-root image.
 * Classify ONLY the observed J713/25G83 firmware root transitions using this session's
 * retained FDT pagetables reservation. No firmware table is written/adopted.
 * AP root[2], middle, leaves and guards still have their independent exact
 * owner checks. This is a read-only manifest, not a live firmware fence.
 */
static int m4_user_upper_foreign_root(void *cookie, u32 index, u64 initial, u64 observed)
{
	struct m4_gpu_hello *h = cookie;
	u64 tables = h->region[M4_TABLES].start, expected;

	lockdep_assert_held(&h->gpu->lock);
	if (h->gpu->hello != h || !h->ready || !h->ready->published ||
	    resource_size(&h->region[M4_TABLES]) != 0x40000 ||
	    !tables || (tables & ~GPU_M4_PA_MASK))
		return -EPERM;
	if (!h->gpu->firmware_start_attempted || index > 1 || initial)
		return -EPROTO;
	/* Ordinary-r2 proves both roots are zero in the stopped session. RUN
	 * installs the documented page-1 firmware link as well as page 4.
	 * Classify only those exact retained reservation-relative words; the
	 * generic owner remembers acceptance and rejects a rollback to zero.
	 */
	expected = index ? (tables + 0x10000) | 0x00200000000004cbULL :
		(tables + GPU_M4_DRM_PAGE_SIZE) | 0x403ULL;
	return observed == expected ? 0 : -EPROTO;
}

static int m4_user_upper_init_locked(struct m4_gpu_hello *h)
{
	static const struct gpu_m4_drm_upper_ops ops = {
		.session_get_locked = m4_upper_session_get, .session_put = m4_upper_session_put,
		.table_locked = m4_upper_table, .gate_locked = m4_user_upper_gate,
		.foreign_root_locked = m4_user_upper_foreign_root,
		.seed_locked = m4_user_upper_seed,
		.handoff_get_locked = m4_user_upper_handoff_get,
		.handoff_put_locked = m4_user_upper_handoff_put,
		.clean_locked = m4_user_upper_clean, .barrier_locked = m4_upper_barrier,
		.asid0_tlbi_locked = m4_user_upper_asid0_sync, .commit_locked = m4_user_upper_commit,
		/* Release invalidates each released page by VA (global leaves). */
		.asid0_range_tlbi_locked = m4_user_upper_asid0_range_sync,
		.release_commit_locked = m4_user_upper_release_commit,
		.shutdown_locked = m4_upper_shutdown, .fault_locked = m4_user_upper_fault,
	};
	struct m4_gpu_ready *r = h->ready;
	struct gpu_m4_drm_upper_table child = {
		.cpu = page_address(r->table[M4_UPPER_LEAF]),
		.pa = page_to_phys(r->table[M4_UPPER_LEAF]),
		.owner = h, .class = GPU_M4_DRM_UPPER_INHERITED_AP,
	};
	struct gpu_m4_drm_upper_config config = {
		.lock = &h->gpu->lock, .session = h, .ops = &ops,
		.root = { .cpu = h->tables, .pa = h->region[M4_TABLES].start,
			.owner = h, .class = GPU_M4_DRM_UPPER_RESERVED_ROOT },
		.middle = { .cpu = h->shared, .pa = h->region[M4_SHARED].start,
			.owner = h, .class = GPU_M4_DRM_UPPER_RESERVED_MIDDLE },
		.children = &child, .child_count = 1, .max_children = 32,
		/* Every lease spans >= 1 page: bounding leases by pages loses nothing
		 * and avoids exhaustion by many small per-job leases under CTS churn.
		 */
		.max_pages = 16384, .max_leases = 16384, .scan_pages = 65536,
		.normal_min_va = r->next_va,
	};

	dev_info(h->gpu->dev,
		 "GPU ordinary stopped firmware roots: root0=%#llx root1=%#llx; AP arena cells=%u\n",
		 get_unaligned_le64(h->tables), get_unaligned_le64(h->tables + 8),
		 M4_ARENA_CELLS);
	return m4_arena_import_locked(h, &config);
}

/* Called from the one stopped HELLO owner with gpu->lock held. Claim first,
 * then drop that lock for real GEM/reservation factories; every subsequent
 * publication still repeats the original stopped/root/handoff checks.
 */
static int m4_user_prerun_prepare_locked(struct m4_gpu_hello *h)
{
	struct m4_gpu_power *gpu = h->gpu;
	struct m4_gpu_provider_runtime *p;
	int ret;

	lockdep_assert_held(&gpu->lock);
	if (!runtime_enable || !h->ready || !h->ready->prepared ||
	    !h->ready->execution || !h->ready->execution->store ||
	    h->ready->provider_runtime || h->published ||
	    provider_root_trial || upper_alias_trial || !execution_enable ||
	    !gpu->stopped_verified || gpu->firmware_start_attempted)
		return -EINVAL;
	ret = m4_gpu_stopped(gpu);
	if (ret)
		return ret;
	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->hello = h;
	p->ready = h->ready;
	p->execution = h->ready->execution;
	p->ordinary = p->prerun_staging = p->attempted = true;
	p->started = ktime_get_ns();
	p->handoff_get = p->handoff_put = p->tlbi = -EINPROGRESS;
	refcount_set(&p->session_refs, 1);
	__module_get(THIS_MODULE);
	get_device(gpu->dev);
	pm_runtime_get_noresume(gpu->dev);
	h->ready->provider_runtime = p;
	mutex_unlock(&gpu->lock);
	ret = m4_provider_stage(p);
	if (!ret)
		ret = m4_arena_prepare(h, p->frontend);
	if (!ret)
		ret = m4_user_runtime_prepare(p);
	mutex_lock(&gpu->lock);
	/* An incomplete owner cannot race a second sysfs startup: its claim is
	 * already installed. Nevertheless recheck exact identity and stopped state
	 * after the unlocked allocation interval, before any possibly visible write.
	 */
	if (!ret && (gpu->hello != h || h->ready != p->ready ||
	    h->ready->provider_runtime != p || h->published ||
	    gpu->firmware_start_attempted || p->error || gpu->error || h->error))
		ret = -ESTALE;
	if (!ret)
		ret = m4_gpu_stopped(gpu);
	if (!ret)
		ret = m4_hello_graph(h, false, false);
	p->prerun_staging = false;
	if (ret)
		m4_provider_fault_locked(p, ret);
	else
		p->prepared = true;
	return ret;
}

#endif
