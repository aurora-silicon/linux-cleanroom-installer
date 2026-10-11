/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_UPPER_RUNTIME_H
#define APPLE_GPU_M4_UPPER_RUNTIME_H

/* REMOVE-DEBUG: one fresh notifier alias consumer, not a general publisher.
 * Actual retained owners, exclusion and invalidation are used here; no passing
 * test authorizes translation replacement, unmapping or firmware shutdown.
 */
static int m4_store_fresh(struct m4_gpu_hello *h);
static int m4_store_control(struct m4_gpu_hello *h);

static struct m4_gpu_store *m4_upper_store(struct m4_gpu_hello *h)
{
	return h->ready->execution->store;
}

static int m4_upper_session_get(void *cookie)
{
	struct m4_gpu_hello *h = cookie;

	lockdep_assert_held(&h->gpu->lock);
	get_device(h->gpu->dev);
	__module_get(THIS_MODULE);
	return 0;
}

static void m4_upper_session_put(void *cookie)
{
	struct m4_gpu_hello *h = cookie;

	lockdep_assert_not_held(&h->gpu->lock);
	put_device(h->gpu->dev);
	module_put(THIS_MODULE);
}

static int m4_upper_table(void *cookie, const struct gpu_m4_drm_upper_table *table)
{
	struct m4_gpu_hello *h = cookie;
	struct m4_gpu_ready *r = h->ready;
	bool owned = table->owner == h &&
		((table->class == GPU_M4_DRM_UPPER_RESERVED_ROOT &&
		  table->cpu == h->tables && table->pa == h->region[M4_TABLES].start) ||
		 (table->class == GPU_M4_DRM_UPPER_RESERVED_MIDDLE &&
		  table->cpu == h->shared && table->pa == h->region[M4_SHARED].start) ||
		 (table->class == GPU_M4_DRM_UPPER_INHERITED_AP &&
		  table->cpu == page_address(r->table[M4_UPPER_LEAF]) &&
		  table->pa == page_to_phys(r->table[M4_UPPER_LEAF])));
	u32 fresh = table->class == GPU_M4_DRM_UPPER_RESERVED_ROOT ? M4_READY_FRESH_ROOT :
		table->class == GPU_M4_DRM_UPPER_RESERVED_MIDDLE ? M4_READY_FRESH_MIDDLE :
		M4_READY_FRESH_LEAF;

	lockdep_assert_held(&h->gpu->lock);
	if (!owned)
		return -EPERM;
	/* Inside one live READY health pass's read-only upper health only: this
	 * exact page was invalidated by that pass before its first read and not
	 * stored to since (m4_upper_memory_valid()). Never a store/clean path.
	 */
	if (r->health_reuse & fresh) {
		m4_ready_health_cost.reused++;
		return 0;
	}
	/* The AP never leaves unpublished dirty bytes in an admitted live page.
	 * This also obtains current neighboring words before a cacheline clean.
	 */
	dcache_inval_poc((unsigned long)table->cpu,
		(unsigned long)table->cpu + 0x4000);
	return 0;
}

static int m4_upper_gate(void *cookie, u32 gates, bool trial)
{
	struct m4_gpu_hello *h = cookie;
	struct m4_gpu_ready *r = h->ready;
	struct m4_gpu_exec_memory *e = r->execution;
	struct m4_gpu_store *s = e->store;
	u32 selected = upper_alias_trial == 1 ? GPU_M4_DRM_UPPER_EXISTING_CHILD :
		GPU_M4_DRM_UPPER_NEW_PARENT;
	int ret;

	lockdep_assert_held(&h->gpu->lock);
	if (!trial || upper_alias_trial < 1 || upper_alias_trial > 2 || gates != selected ||
	    !s->upper_attempted || s->upper_published || e->notification_fw ||
	    !s->prepared || s->bound || s->binding_attempted || s->published ||
	    s->error || r->error || h->error || h->gpu->error || !r->complete ||
	    !r->policy_complete || !e->barrier_complete)
		return -EPERM;
	/* No pre-existing recovery/control transition may acquire live table-write
	 * authority. Check again under handoff, before the very first visible slot.
	 */
	if (m4_ready_read32(r->input.object[GPU_M4_FIRMWARE_DATA] + 0x4fb0))
		return -EPROTO;
	ret = m4_store_control(h);
	if (ret)
		return ret;
	ret = m4_ready_health(h);
	if (!ret)
		ret = m4_store_fresh(h);
	if (!ret && m4_ready_read32(r->input.object[GPU_M4_FIRMWARE_DATA] + 0x4fb0))
		ret = -EPROTO;
	return ret;
}

static int m4_upper_handoff_get(void *cookie, u32 ceiling_ms)
{
	struct m4_gpu_hello *h = cookie;
	struct m4_gpu_store *s = m4_upper_store(h);

	if (ceiling_ms != 100)
		return -EINVAL;
	s->upper_handoff_get = m4_hello_lock(h);
	return s->upper_handoff_get;
}

static int m4_upper_handoff_put(void *cookie)
{
	struct m4_gpu_hello *h = cookie;
	struct m4_gpu_store *s = m4_upper_store(h);

	m4_hello_unlock(h);
	s->upper_handoff_put = readb(h->handoff + 0x10) ? -EIO : 0;
	return s->upper_handoff_put;
}

static bool m4_upper_contains(void *base, void *cpu, u32 size)
{
	unsigned long first = (unsigned long)base, address = (unsigned long)cpu;

	return base && cpu && size && address >= first && address - first < 0x4000 &&
		size <= 0x4000 - (address - first);
}

static int m4_upper_clean(void *cookie, void *cpu, u32 size)
{
	struct m4_gpu_hello *h = cookie;
	struct m4_gpu_store *s = m4_upper_store(h);

	if (!m4_upper_contains(h->shared, cpu, size) &&
	    !m4_upper_contains(s->upper_child.cpu, cpu, size))
		return -EPERM;
	dcache_clean_poc((unsigned long)cpu, (unsigned long)cpu + size);
	return 0;
}

static int m4_upper_barrier(void *cookie)
{
	struct m4_gpu_hello *h = cookie;

	lockdep_assert_held(&h->gpu->lock);
	dsb(osh);
	return 0;
}

static int m4_upper_asid0_sync(void *cookie)
{
	struct m4_gpu_hello *h = cookie;

	lockdep_assert_held(&h->gpu->lock);
	/* ASID zero, not the previously bound ASID-1 operand, a PA or a VA. */
	asm volatile("dsb osh\n\t.arch armv8.4-a\n\ttlbi aside1os, %0\n\tdsb sy"
		: : "r" (0ULL) : "memory");
	m4_upper_store(h)->upper_tlbi = 0;
	return 0;
}

static int m4_upper_commit(void *cookie,
	const struct gpu_m4_drm_upper_update *updates, u32 count)
{
	struct m4_gpu_hello *h = cookie;
	struct m4_gpu_ready *r = h->ready;
	struct m4_gpu_exec_memory *e = r->execution;
	struct m4_gpu_store *s = e->store;
	u32 expected = s->upper_new_child ? 2 : 1;
	u32 parents = 0, leaves = 0;

	if (count != expected || !updates)
		return -EPROTO;
	for (u32 i = 0; i < count; i++) {
		const struct gpu_m4_drm_upper_update *u = &updates[i];

		if (u->parent) {
			if (!s->upper_new_child || u->table_cpu != h->shared ||
			    u->table_pa != h->region[M4_SHARED].start ||
			    u->index != s->upper_middle || u->index >= 2046 ||
			    u->value != (s->upper_child.pa | 3) ||
			    get_unaligned_le64(r->expected[M4_READY_TABLES] + u->index * 8) ||
			    get_unaligned_le64(h->shared + u->index * 8) != u->value)
				return -EPROTO;
			parents++;
		} else {
			if (u->table_cpu != s->upper_child.cpu || u->table_pa != s->upper_child.pa ||
			    u->index != s->upper_index || u->index >= 2048 ||
			    u->value != (e->notification.pa | GPU_M4_DRM_UPPER_FW_ATTRS) ||
			    get_unaligned_le64(u->table_cpu + u->index * 8) != u->value ||
			    (!s->upper_new_child &&
			     get_unaligned_le64(r->expected[M4_UPPER_LEAF] + u->index * 8)))
				return -EPROTO;
			leaves++;
		}
	}
	if (parents != s->upper_new_child || leaves != 1 ||
	    (s->upper_new_child && r->upper_extra_count >= ARRAY_SIZE(r->upper_extra)))
		return -EPROTO;
	/* Authorize ONLY these independently checked zero-slot transitions. Never
	 * copy a live table wholesale into a passing expected shadow.
	 */
	for (u32 i = 0; i < count; i++) {
		const struct gpu_m4_drm_upper_update *u = &updates[i];

		if (u->parent)
			put_unaligned_le64(u->value, r->expected[M4_READY_TABLES] + u->index * 8);
		else if (!s->upper_new_child)
			put_unaligned_le64(u->value, r->expected[M4_UPPER_LEAF] + u->index * 8);
	}
	if (s->upper_new_child)
		r->upper_extra[r->upper_extra_count++] = s->upper_child;
	return 0;
}

static int m4_upper_shutdown(void *cookie)
{
	/* No permanent firmware halt is qualified. Even success retains aliases. */
	return -EOPNOTSUPP;
}

static void m4_upper_fault(void *cookie, int error)
{
	struct m4_gpu_hello *h = cookie;

	m4_upper_store(h)->upper_error = error;
	h->ready->error = error;
	h->error = error;
	h->gpu->error = error;
}

static int m4_upper_backing_get(void *cookie)
{
	struct m4_gpu_hello *h = cookie;
	struct m4_gpu_store *s = m4_upper_store(h);

	lockdep_assert_held(&h->gpu->lock);
	if (s->upper_refs == U32_MAX)
		return -EOVERFLOW;
	s->upper_refs++;
	return m4_upper_session_get(h);
}

static void m4_upper_backing_put(void *cookie)
{
	struct m4_gpu_hello *h = cookie;
	struct m4_gpu_store *s = m4_upper_store(h);

	lockdep_assert_not_held(&h->gpu->lock);
	mutex_lock(&h->gpu->lock);
	WARN_ON(!s->upper_refs);
	if (s->upper_refs)
		s->upper_refs--;
	mutex_unlock(&h->gpu->lock);
	m4_upper_session_put(h);
}

static int m4_upper_backing_page(void *cookie, u64 offset, u64 *pa)
{
	struct m4_gpu_hello *h = cookie;
	struct m4_ready_buffer *b = &h->ready->execution->notification;

	lockdep_assert_held(&h->gpu->lock);
	if (!pa || !m4_exec_backing_valid(h, b, GPU_M4_EXEC_NOTIFY_SIZE, true) ||
	    (offset & 0x3fff) || offset >= b->size)
		return -EPERM;
	*pa = b->pa + offset;
	return 0;
}

static int m4_upper_backing_check(void *cookie, const void *identity, u64 offset, u64 pa)
{
	struct m4_gpu_hello *h = cookie;
	u64 actual;
	int ret = m4_upper_backing_page(cookie, offset, &actual);

	if (ret)
		return ret;
	return identity == &h->ready->execution->notification && pa == actual ? 0 : -EPERM;
}

static int m4_upper_backing_ready(void *cookie, u64 offset, u64 size,
	enum gpu_m4_drm_upper_class class)
{
	struct m4_gpu_hello *h = cookie;
	struct m4_ready_buffer *b = &h->ready->execution->notification;
	u64 pa;
	int ret = m4_upper_backing_page(cookie, offset, &pa);

	if (ret || offset || size != b->size || class != GPU_M4_DRM_UPPER_FW_ONLY)
		return ret ?: -EPERM;
	dma_wmb(); /* Retain the sole NC data view; never touch a WB alias. */
	return 0;
}

static int m4_upper_trial(struct m4_gpu_hello *h)
{
	static const struct gpu_m4_drm_upper_ops ops = {
		.session_get_locked = m4_upper_session_get, .session_put = m4_upper_session_put,
		.table_locked = m4_upper_table, .gate_locked = m4_upper_gate,
		.handoff_get_locked = m4_upper_handoff_get, .handoff_put_locked = m4_upper_handoff_put,
		.clean_locked = m4_upper_clean, .barrier_locked = m4_upper_barrier,
		.asid0_tlbi_locked = m4_upper_asid0_sync, .commit_locked = m4_upper_commit,
		.shutdown_locked = m4_upper_shutdown, .fault_locked = m4_upper_fault,
	};
	struct m4_gpu_ready *r = h->ready;
	struct m4_gpu_exec_memory *e = r->execution;
	struct m4_gpu_store *s = e->store;
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
		.max_pages = 1, .max_leases = 1, .scan_pages = 4096,
		.normal_min_va = r->next_va,
	};
	struct gpu_m4_drm_upper_backing backing = {
		.owner = h, .identity = &e->notification, .size = e->notification.size,
		.policy = GPU_M4_DRM_CPU_NC, .get_locked = m4_upper_backing_get,
		.put = m4_upper_backing_put, .page_locked = m4_upper_backing_page,
		.check_locked = m4_upper_backing_check, .ready_locked = m4_upper_backing_ready,
	};
	int ret;

	if (upper_alias_trial < 1 || upper_alias_trial > 2 || s->upper_attempted ||
	    r->upper || e->notification_fw)
		return -EINVAL;
	ret = m4_store_fresh(h);
	if (ret)
		return ret;
	s->upper_attempted = true;
	s->upper_mode = upper_alias_trial;
	s->upper_started = ktime_get_ns();
	s->upper_original = e->notification.va;
	s->upper_handoff_get = s->upper_handoff_put = s->upper_tlbi = -EINPROGRESS;
	r->upper = gpu_m4_drm_upper_create_locked(&config);
	if (IS_ERR(r->upper)) {
		ret = PTR_ERR(r->upper);
		r->upper = NULL;
		goto failed;
	}
	s->upper_lease = gpu_m4_drm_upper_accept_placed_locked(r->upper, &backing,
		GPU_M4_DRM_UPPER_NORMAL, GPU_M4_DRM_UPPER_FW_ONLY, 0,
		upper_alias_trial == 1 ? GPU_M4_DRM_UPPER_EXISTING_ONLY : GPU_M4_DRM_UPPER_NEW_ONLY);
	if (IS_ERR(s->upper_lease)) {
		ret = PTR_ERR(s->upper_lease);
		s->upper_lease = NULL;
		goto failed;
	}
	s->upper_selected = gpu_m4_drm_upper_va_locked(s->upper_lease);
	s->upper_middle = (s->upper_selected >> 25) & 2047;
	s->upper_index = (s->upper_selected >> 14) & 2047;
	ret = gpu_m4_drm_upper_table_locked(r->upper, s->upper_middle, &s->upper_child);
	if (ret)
		goto failed;
	s->upper_parent_before = get_unaligned_le64(h->shared + s->upper_middle * 8);
	s->upper_leaf_before = get_unaligned_le64(s->upper_child.cpu + s->upper_index * 8);
	s->upper_new_child = !s->upper_parent_before;
	memcpy(s->upper_child_before, s->upper_child.cpu, 0x4000);
	if (s->upper_selected == s->upper_original || s->upper_leaf_before ||
	    s->upper_new_child != (upper_alias_trial == 2) ||
	    (s->upper_new_child && memchr_inv(s->upper_child_before, 0, 0x4000))) {
		ret = -EPROTO;
		goto failed;
	}
	ret = gpu_m4_drm_upper_publish_locked(s->upper_lease, true);
	if (ret)
		goto failed;
	s->upper_published = true;
	s->upper_parent_after = get_unaligned_le64(h->shared + s->upper_middle * 8);
	s->upper_leaf_after = get_unaligned_le64(s->upper_child.cpu + s->upper_index * 8);
	/* Change only three unused absolute pointers, not the whole record image. */
	for (u32 engine = 0; engine < 3; engine++)
		put_unaligned_le64(s->upper_selected + 0x600 + engine * 8,
			e->notification.cpu + engine * 0x200);
	dma_wmb();
	e->notification_fw = s->upper_selected;
	ret = m4_ready_health(h);
	if (!ret)
		ret = m4_store_fresh(h);
	s->upper_finished = ktime_get_ns();
	if (!ret)
		return 0;
failed:
	if (s->upper_child.cpu) {
		dcache_inval_poc((unsigned long)h->shared, (unsigned long)h->shared + 0x4000);
		s->upper_parent_after = get_unaligned_le64(h->shared + s->upper_middle * 8);
		/* A linked child was cleaned before parent publication. A detached
		 * newly allocated child may still contain AP-only dirty zero bytes;
		 * retain that private preimage without invalidating it away.
		 */
		if (s->upper_parent_after || !s->upper_new_child)
			dcache_inval_poc((unsigned long)s->upper_child.cpu,
				(unsigned long)s->upper_child.cpu + 0x4000);
		s->upper_leaf_after = get_unaligned_le64(s->upper_child.cpu + s->upper_index * 8);
	}
	s->upper_error = ret;
	s->upper_finished = ktime_get_ns();
	return ret; /* Every original/new owner remains reachable through reboot. */
}

static ssize_t upper_alias_state_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_store *s;
	ssize_t size;

	mutex_lock(&gpu->lock);
	s = gpu->hello && gpu->hello->ready && gpu->hello->ready->execution ?
		gpu->hello->ready->execution->store : NULL;
	if (!s || !s->upper_attempted)
		size = sysfs_emit(buf, "attempted=0 mode=%u\n", upper_alias_trial);
	else
		size = sysfs_emit(buf, "attempted=1 mode=%u published=%u consumed=%u error=%d original=%#llx selected=%#llx parent=%#llx,%#llx leaf=%#llx,%#llx new_child=%u handoff=%d,%d tlbi=%d refs=%u duration_ns=%llu\n",
			s->upper_mode, s->upper_published, s->upper_consumed, s->upper_error,
			s->upper_original, s->upper_selected, s->upper_parent_before, s->upper_parent_after,
			s->upper_leaf_before, s->upper_leaf_after, s->upper_new_child,
			s->upper_handoff_get, s->upper_handoff_put, s->upper_tlbi, s->upper_refs,
			s->upper_finished - s->upper_started);
	mutex_unlock(&gpu->lock);
	return size;
}
static DEVICE_ATTR_RO(upper_alias_state);

#endif
