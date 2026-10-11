/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_PROVIDER_TS_UPPER_H
#define APPLE_GPU_M4_PROVIDER_TS_UPPER_H

/* REMOVE-DEBUG: one fresh native timestamp page with one NC or WB CPU view.
 * CPU WB never changes the qualified firmware NC leaf permission bits.
 * This is an actual READY
 * table owner, not authority borrowed from the ordinary notifier trial. It
 * retains all reachable storage until reboot, including failed publication.
 */
struct m4_provider_ts_upper {
	struct m4_gpu_provider_runtime *runtime;
	struct gpu_m4_drm_upper_table child;
	u64 base, parent_before, parent_after, leaf_before, leaf_after;
	u32 middle, index;
	int handoff_get, handoff_put, tlbi, error;
	bool committed;
};

static int m4_provider_ts_owner_check_locked(void *owner,
	struct gpu_m4_drm_provider *provider, struct gpu_m4_drm_upper *upper,
	struct gpu_m4_drm *frontend, struct mutex *lock)
{
	struct m4_gpu_provider_runtime *p = owner;
	struct m4_gpu_hello *h = p->hello;
	struct m4_gpu_ready *r = p->ready;

	lockdep_assert_held(&h->gpu->lock);
	/* Scalar identity only: this callback also runs inside upper inventory. */
	return lock == &h->gpu->lock && h->gpu->hello == h && h->ready == r &&
		r->provider_runtime == p && r->execution == p->execution &&
		p->timestamp_manifest && !p->writeback_manifest && !p->error &&
		provider == p->provider && frontend == p->frontend && upper == r->upper &&
		r->timestamp_upper && r->timestamp_upper->runtime == p &&
		!r->timestamp_upper->error && !r->error && !h->error && !h->gpu->error &&
		refcount_read(&p->session_refs) ? 0 : -EPERM;
}

static int m4_provider_ts_upper_gate(void *cookie, u32 gates, bool trial)
{
	struct m4_gpu_hello *h = cookie;
	struct m4_gpu_ready *r = h->ready;
	struct m4_gpu_provider_runtime *p = r->provider_runtime;
	struct m4_provider_ts_upper *t = r->timestamp_upper;
	struct gpu_m4_drm_provider_job_timestamp_view view;
	u32 count;
	int ret;

	lockdep_assert_held(&h->gpu->lock);
	if (!trial || gates != (GPU_M4_DRM_UPPER_TIMESTAMP_GATE |
	    GPU_M4_DRM_UPPER_NEW_PARENT) || !t || t->runtime != p || t->committed ||
	    t->error || !p || !p->timestamp_manifest || !p->ts_job ||
	    !p->ts_job->job || !p->ts_job->work || p->ts_job->work->begun ||
	    p->work || !p->complete || !p->selected || p->generation != 2 ||
	    p->resident != p->version[1] || upper_alias_trial ||
	    !r->complete || !r->policy_complete || !r->execution->barrier_complete)
		return -EPERM;
	ret = m4_provider_ts_owner_check_locked(p, p->provider, r->upper,
		p->frontend, &h->gpu->lock);
	if (!ret)
		ret = gpu_m4_drm_provider_lane_check_active_locked(p->ts_lane, p->provider,
			&p->ts_job->item, p->ts_job->job);
	if (!ret)
		ret = gpu_m4_drm_provider_job_timestamp_count_locked(p->ts_job->job, &count);
	if (!ret && count != 2)
		ret = -EPROTO;
	for (u32 i = 0; !ret && i < count; i++) {
		ret = gpu_m4_drm_provider_job_timestamp_at_locked(p->ts_job->job, i, &view);
		if (!ret && (view.gem != p->timestamp.gem ||
		    !view.object || view.object_offset != 8 + i * 8 ||
		    view.firmware_va != GPU_M4_DRM_UPPER_TIMESTAMP + 8 + i * 8))
			ret = -EPERM;
		if (!ret)
			ret = gpu_m4_drm_provider_job_timestamp_check_locked(p->ts_job->job,
				view.runtime_alias, false);
	}
	/* Repeated under the bounded firmware handoff: fixed live observations,
	 * no graph refresh, dependency wait or allocation while firmware is locked.
	 */
	if (!ret)
		ret = m4_provider_fixed(h, p->root[1], 0);
	return ret;
}

static int m4_provider_ts_upper_handoff_get(void *cookie, u32 ceiling_ms)
{
	struct m4_gpu_hello *h = cookie;
	struct m4_provider_ts_upper *t = h->ready->timestamp_upper;

	if (!t || ceiling_ms != 100)
		return -EINVAL;
	t->handoff_get = m4_hello_lock(h);
	return t->handoff_get;
}

static int m4_provider_ts_upper_handoff_put(void *cookie)
{
	struct m4_gpu_hello *h = cookie;
	struct m4_provider_ts_upper *t = h->ready->timestamp_upper;

	m4_hello_unlock(h);
	t->handoff_put = readb(h->handoff + 0x10) ? -EIO : 0;
	return t->handoff_put;
}

static int m4_provider_ts_upper_child(struct m4_gpu_hello *h,
	struct gpu_m4_drm_upper_table *child)
{
	/* Resolve the checked allocator's retained view, never a live PA cast. */
	return gpu_m4_drm_upper_table_locked(h->ready->upper,
		(GPU_M4_DRM_UPPER_TIMESTAMP >> 25) & 2047, child);
}

static int m4_provider_ts_upper_clean(void *cookie, void *cpu, u32 size)
{
	struct m4_gpu_hello *h = cookie;
	struct gpu_m4_drm_upper_table child;
	int ret;

	lockdep_assert_held(&h->gpu->lock);
	if (!m4_upper_contains(h->shared, cpu, size)) {
		ret = m4_provider_ts_upper_child(h, &child);
		if (ret || child.owner != h->ready->upper ||
		    child.class != GPU_M4_DRM_UPPER_INHERITED_AP ||
		    !m4_upper_contains(child.cpu, cpu, size))
			return ret ?: -EPERM;
	}
	dcache_clean_poc((unsigned long)cpu, (unsigned long)cpu + size);
	return 0;
}

static int m4_provider_ts_upper_asid0_sync(void *cookie)
{
	struct m4_gpu_hello *h = cookie;

	lockdep_assert_held(&h->gpu->lock);
	asm volatile("dsb osh\n\t.arch armv8.4-a\n\ttlbi aside1os, %0\n\tdsb sy"
		: : "r" (0ULL) : "memory");
	h->ready->timestamp_upper->tlbi = 0;
	return 0;
}

static int m4_provider_ts_upper_commit(void *cookie,
	const struct gpu_m4_drm_upper_update *updates, u32 count)
{
	struct m4_gpu_hello *h = cookie;
	struct m4_gpu_ready *r = h->ready;
	struct m4_provider_ts_upper *t = r->timestamp_upper;
	struct m4_gpu_provider_runtime *p = r->provider_runtime;
	struct gpu_m4_drm_upper_table child;
	u32 parents = 0, leaves = 0;
	int ret;

	lockdep_assert_held(&h->gpu->lock);
	if (!t || t->committed || !updates || count != 2 || t->tlbi ||
	    r->upper_extra_count || !p->timestamp.initialized ||
	    p->timestamp.size != GPU_M4_DRM_PAGE_SIZE)
		return -EPROTO;
	ret = m4_provider_ts_upper_child(h, &child);
	if (ret)
		return ret;
	if (child.owner != r->upper || child.class != GPU_M4_DRM_UPPER_INHERITED_AP ||
	    !child.cpu || !child.pa)
		return -EPERM;
	for (u32 i = 0; i < count; i++) {
		const struct gpu_m4_drm_upper_update *u = &updates[i];

		if (u->parent) {
			if (u->table_cpu != h->shared || u->table_pa != h->region[M4_SHARED].start ||
			    u->index != ((GPU_M4_DRM_UPPER_TIMESTAMP >> 25) & 2047) ||
			    u->value != (child.pa | 3) ||
			    get_unaligned_le64(r->expected[M4_READY_TABLES] + u->index * 8) ||
			    get_unaligned_le64(h->shared + u->index * 8) != u->value)
				return -EPROTO;
			parents++;
		} else {
			if (u->table_cpu != child.cpu || u->table_pa != child.pa || u->index ||
			    u->value != (p->timestamp.pa | GPU_M4_DRM_UPPER_FW_ATTRS) ||
			    get_unaligned_le64(child.cpu) != u->value ||
			    memchr_inv(child.cpu + 8, 0, GPU_M4_DRM_PAGE_SIZE - 8))
				return -EPROTO;
			leaves++;
		}
	}
	if (parents != 1 || leaves != 1)
		return -EPROTO;
	/* Adopt ONLY the individually checked zero parent and one leaf. Keeping
	 * the entire remaining child zero also proves the trailing guard.
	 */
	t->child = child;
	t->base = GPU_M4_DRM_UPPER_TIMESTAMP;
	t->middle = (t->base >> 25) & 2047;
	t->index = 0;
	t->parent_after = child.pa | 3;
	t->leaf_after = p->timestamp.pa | GPU_M4_DRM_UPPER_FW_ATTRS;
	put_unaligned_le64(t->parent_after, r->expected[M4_READY_TABLES] + t->middle * 8);
	r->upper_extra[r->upper_extra_count++] = child;
	t->committed = true;
	return 0;
}

static void m4_provider_ts_upper_fault(void *cookie, int error)
{
	struct m4_gpu_hello *h = cookie;

	if (h->ready->timestamp_upper && !h->ready->timestamp_upper->error)
		h->ready->timestamp_upper->error = error < 0 ? error : -EIO;
	m4_provider_fault_locked(h->ready->provider_runtime, error);
	h->error = error < 0 ? error : -EIO;
	h->gpu->error = h->error;
}

static int m4_provider_ts_upper_stage_locked(struct m4_gpu_provider_runtime *p)
{
	static const struct gpu_m4_drm_upper_ops ops = {
		.session_get_locked = m4_upper_session_get, .session_put = m4_upper_session_put,
		.table_locked = m4_upper_table, .gate_locked = m4_provider_ts_upper_gate,
		.handoff_get_locked = m4_provider_ts_upper_handoff_get,
		.handoff_put_locked = m4_provider_ts_upper_handoff_put,
		.clean_locked = m4_provider_ts_upper_clean, .barrier_locked = m4_upper_barrier,
		.asid0_tlbi_locked = m4_provider_ts_upper_asid0_sync,
		.commit_locked = m4_provider_ts_upper_commit,
		.shutdown_locked = m4_upper_shutdown, .fault_locked = m4_provider_ts_upper_fault,
	};
	struct m4_gpu_hello *h = p->hello;
	struct m4_gpu_ready *r = p->ready;
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
		.children = &child, .child_count = 1, .max_children = 2,
		.max_pages = 1, .max_leases = 1, .scan_pages = 2, .normal_min_va = r->next_va,
	};
	struct m4_provider_ts_upper *t;
	int ret;

	lockdep_assert_held(&h->gpu->lock);
	if (!p->timestamp_manifest || !p->complete || !p->selected || p->generation != 2 ||
	    p->resident != p->version[1] || !p->timestamp.initialized || p->work ||
	    r->upper || r->timestamp_upper || r->upper_extra_count || upper_alias_trial ||
	    r->execution->store->upper_attempted || r->execution->notification_fw)
		return -EPERM;
	ret = m4_provider_idle(h);
	if (ret)
		return ret;
	/* The checked allocator must encounter a fresh TS parent at its interval
	 * start; do not let scan fallback broaden this hardware qualification.
	 */
	if (get_unaligned_le64(h->shared + ((GPU_M4_DRM_UPPER_TIMESTAMP >> 25) & 2047) * 8) ||
	    get_unaligned_le64(r->expected[M4_READY_TABLES] +
		((GPU_M4_DRM_UPPER_TIMESTAMP >> 25) & 2047) * 8))
		return -EEXIST;
	t = kzalloc_obj(*t);
	if (!t)
		return -ENOMEM;
	t->runtime = p;
	t->handoff_get = t->handoff_put = t->tlbi = -EINPROGRESS;
	r->timestamp_upper = t;
	r->upper = gpu_m4_drm_upper_create_locked(&config);
	if (IS_ERR(r->upper)) {
		ret = PTR_ERR(r->upper);
		r->upper = NULL;
		t->error = ret;
		return ret; /* Detached owner remains reachable for diagnostics. */
	}
	return 0;
}

#endif
