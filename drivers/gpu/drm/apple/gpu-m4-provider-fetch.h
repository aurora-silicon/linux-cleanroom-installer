/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_PROVIDER_FETCH_H
#define APPLE_GPU_M4_PROVIDER_FETCH_H

#include <linux/compiler.h>
#include <linux/byteorder/generic.h>
#include "gpu-m4-drm-provider-job.h"
#include "gpu-m4-drm-provider-lane.h"
#include "gpu-m4-drm-provider-upper.h"

/* Existing READY context-0 child only; these are the checked current topology,
 * not permission to allocate parents or mirror arbitrary user VM mappings.
 */
#define GPU_M4_FETCH_START 0x7100000000ULL
#define GPU_M4_FETCH_END 0x7102000000ULL
#define GPU_M4_FETCH_LEAVES 2048U
#define GPU_M4_FETCH_ROOT_INDEX 7U
#define GPU_M4_FETCH_MIDDLE_INDEX 128U
#define GPU_M4_FETCH_LOWER_ATTRS 0x00c0000000000c8bULL
#define GPU_M4_FETCH_MAX_PAGES 16U /* Software transaction budget, not HW qualification. */
#define GPU_M4_FETCH_MAX_CLAIMS 64U

enum gpu_m4_fetch_state {
	GPU_M4_FETCH_UNUSED,
	GPU_M4_FETCH_DETACHED,
	GPU_M4_FETCH_UNCERTAIN,
	GPU_M4_FETCH_COMMITTED,
	GPU_M4_FETCH_FAILED_RETAINED,
	GPU_M4_FETCH_CANCELLED,
};

struct gpu_m4_fetch;
struct gpu_m4_fetch_claim;

/* All three table views are actual retained AP-owned WB pages. Unknown/recovered
 * PAs must never be mapped or CPU-dereferenced. Their owner check is mandatory.
 */
struct gpu_m4_fetch_table {
	__le64 *cpu;
	u64 pa;
	void *owner;
};

struct gpu_m4_fetch_request {
	struct gpu_m4_drm_provider_vm *vm;
	struct gpu_m4_drm_provider_version *source;
	struct gpu_m4_drm_provider_private_gem work;
	struct gpu_m4_drm_upper *upper;
	struct gpu_m4_drm_upper_lease *upper_lease;
};

/* These are retained actual objects, not opaque casts, handles or booleans
 * claiming execution. Adapter job_locked proves exact cache association and
 * protected job state before this helper uses the cache/lane APIs.
 */
struct gpu_m4_fetch_job {
	struct gpu_m4_drm_provider_job *job;
	struct gpu_m4_drm_provider_version *old_version, *version;
	struct gpu_m4_drm_vm_owner *snapshot;
	struct gpu_m4_drm_cache_job *cache;
	struct gpu_m4_drm_provider_lane *lane;
	struct gpu_m4_drm_provider_lane_item *item;
	void *stage_owner;
	u64 binding_generation;
};

struct gpu_m4_fetch_update {
	u32 index;
	u64 value;
};

struct gpu_m4_fetch_ops {
	/* One READY session baseline retains every actual table/global ledger owner
	 * until reboot. init is outside handoff, under provider mutex. No final
	 * session destructor, live reclaim or uncertain owner release exists here.
	 */
	int (*session_get_locked)(void *session);
	int (*table_locked)(void *session, const struct gpu_m4_fetch_table *table);
	/* Fresh cache-current WB views, no repair/adoption of observed contents. */
	int (*refresh_locked)(void *session, const struct gpu_m4_fetch_table tables[3]);
	/* Real sole native NC view, immutable pin and private guard claim, upper
	 * lease and exact allocator ownership. Exclude tables/MMIO/foreign backing.
	 * At acceptance upper may be detached; at publication alias_locked below
	 * requires its actual canonical shared-RW published walk.
	 */
	int (*backing_locked)(void *session, const struct gpu_m4_fetch_claim *claim);
	/* Exact canonical upper lease reference, never a numeric VA lookup. Success
	 * transfers one reference; error transfers none. Put queues disposal only,
	 * never finalizes native backing under the provider lock. The session baseline
	 * independently keeps the sole upper owner alive.
	 */
	int (*alias_get_locked)(void *session, const struct gpu_m4_fetch_claim *claim);
	int (*alias_put_locked)(void *session, const struct gpu_m4_fetch_claim *claim);
	/* Full original READY + old/candidate graphs, actual protected current job,
	 * exact cache/resv/finished ownership, completed dependencies, real exclusive
	 * lane and actual completed binding. Called ONLY outside handoff. Do not
	 * substitute a root value whitelist or attachment-only inventory.
	 */
	int (*job_locked)(void *session, const struct gpu_m4_fetch_claim *claim,
			  const struct gpu_m4_fetch_job *job);
	int (*handoff_get_locked)(void *session, u32 ceiling_ms);
	/* Fixed scalar health, selected actual TTBAT pair and context-0 root word;
	 * no graph/cache walk, allocation, waits or recursively acquired lock.
	 */
	int (*fixed_locked)(void *session, const struct gpu_m4_fetch_claim *claim,
			    const struct gpu_m4_fetch_job *job);
	/* Write EXACTLY one naturally aligned owned normal-memory LE64 leaf using
	 * a single-copy store (WRITE_ONCE). Error may follow the store, so uncertainty
	 * is committed BEFORE the first call. Never write parents/guards/neighbors.
	 */
	int (*store_locked)(void *session, __le64 *cell, __le64 value);
	int (*clean_locked)(void *session, __le64 *first, u32 bytes);
	int (*barrier_locked)(void *session);
	/* Must release exclusion and verify actual AP lock release, even on error. */
	int (*handoff_put_locked)(void *session);
	/* Exact dsb osh; tlbi aside1os, operand0; dsb sy. ASID1 bind is independent. */
	int (*asid0_sync_locked)(void *session, u64 operand);
	/* Adopt ONLY checked selected additions into READY's existing mirror. No
	 * arbitrary full-page memcpy/adoption, producers or other hardware writes.
	 * Fallible partial adoption is terminal and all owners remain retained.
	 */
	int (*commit_locked)(void *session, const struct gpu_m4_fetch_update *updates, u32 count);
	void (*fault_locked)(void *session, int error);
};

struct gpu_m4_fetch_config {
	struct gpu_m4_drm_provider *provider;
	void *session;
	const struct gpu_m4_fetch_ops *ops;
	struct gpu_m4_fetch_table table[3]; /* Root, middle, leaf. */
	const __le64 *expected[3]; /* Exact existing READY committed inventory. */
	u32 max_pages, max_claims;
};

/* Caller heap owns these large retained objects, never kernel-stack scratch.
 * Claims and session baseline survive every possible visible publication until
 * reboot. A job receipt or error fence never authorizes clear/unmap/free here.
 */
struct gpu_m4_fetch {
	struct gpu_m4_fetch_config config;
	__le64 expected[3][GPU_M4_FETCH_LEAVES];
	struct gpu_m4_fetch_claim *claims[GPU_M4_FETCH_MAX_CLAIMS];
	u32 claim_count;
	int error;
	bool initialized;
};

struct gpu_m4_fetch_claim {
	struct gpu_m4_fetch *owner;
	struct gpu_m4_fetch_request request;
	struct gpu_m4_fetch_job job;
	struct gpu_m4_drm_bo_pin *pin; /* Borrowed from retained actual object. */
	u64 pa[GPU_M4_FETCH_MAX_PAGES], value[GPU_M4_FETCH_MAX_PAGES];
	u32 first, pages;
	enum gpu_m4_fetch_state state;
	int error;
	bool vm_ref, source_ref, object_ref, private_ref, alias_ref;
	bool old_ref, version_ref, snapshot_ref, job_ref;
};

static inline int gpu_m4_fetch_status(int ret)
{
	return ret > 0 ? -EPROTO : ret;
}

static inline bool gpu_m4_fetch_reserved_guard(u32 index)
{
	return index == 1 || index == 3 || index == 65;
}

static inline bool gpu_m4_fetch_disjoint(const void *a, u64 asize,
		const void *b, u64 bsize)
{
	u64 first = (unsigned long)a, second = (unsigned long)b;

	return first && second && asize && bsize && asize <= ~0ULL - first &&
		bsize <= ~0ULL - second &&
		(first + asize <= second || second + bsize <= first);
}

/* Borrow only one registered global claim and its immutable copied geometry.
 * A caller-provided pointer/count or a mutated run is never table authority.
 */
static inline int gpu_m4_fetch_claim_check(struct gpu_m4_fetch_claim *c)
{
	struct gpu_m4_fetch *f = c->owner;
	unsigned int i;
	bool found = false;

	if (!f || !f->initialized || !f->config.max_claims ||
	    f->claim_count > f->config.max_claims || f->claim_count > GPU_M4_FETCH_MAX_CLAIMS ||
	    !c->pages || c->pages > f->config.max_pages || c->pages > GPU_M4_FETCH_MAX_PAGES ||
	    c->first >= GPU_M4_FETCH_LEAVES || c->pages >= GPU_M4_FETCH_LEAVES - c->first ||
	    c->request.work.va != GPU_M4_FETCH_START + (u64)c->first * GPU_M4_DRM_PAGE_SIZE ||
	    c->request.work.size != (u64)c->pages * GPU_M4_DRM_PAGE_SIZE ||
	    c->request.work.access != (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE) ||
	    !c->vm_ref || !c->source_ref || !c->object_ref || !c->private_ref || !c->alias_ref)
		return -ESTALE;
	for (i = 0; i < f->claim_count; i++)
		if (f->claims[i] == c)
			found = true;
	if (!found)
		return -ENOENT;
	for (i = 0; i <= c->pages; i++)
		if (gpu_m4_fetch_reserved_guard(c->first + i) ||
		    (i < c->pages && c->value[i] != (c->pa[i] | GPU_M4_FETCH_LOWER_ATTRS)))
			return -ESTALE;
	return 0;
}

static inline int gpu_m4_fetch_fail(struct gpu_m4_fetch *f,
		struct gpu_m4_fetch_claim *c, int ret)
{
	ret = gpu_m4_fetch_status(ret);
	if (!ret)
		ret = -EPROTO;
	if (!f->error)
		f->error = ret;
	if (c) {
		c->error = ret;
		c->state = GPU_M4_FETCH_FAILED_RETAINED;
	}
	f->config.ops->fault_locked(f->config.session, ret);
	return ret;
}

/* Complete graph comparison. Only one optional pending claim may contribute
 * precomputed leaves; the expected mirror stays old until final adoption.
 * Parent and all neighboring/guard bytes remain exact throughout.
 */
static inline int gpu_m4_fetch_graph(struct gpu_m4_fetch *f,
		const struct gpu_m4_fetch_claim *pending, bool refresh)
{
	unsigned int table, i;
	int ret;

	if (refresh) {
		ret = gpu_m4_fetch_status(f->config.ops->refresh_locked(f->config.session,
			f->config.table));
		if (ret)
			return ret;
	}
	for (table = 0; table < 3; table++)
		for (i = 0; i < GPU_M4_FETCH_LEAVES; i++) {
			u64 expected = le64_to_cpu(f->expected[table][i]);

			if (table == 2 && pending && i >= pending->first &&
			    i - pending->first < pending->pages)
				expected = pending->value[i - pending->first];
			if (le64_to_cpu(READ_ONCE(f->config.table[table].cpu[i])) != expected)
				return -ESTALE;
		}
	return 0;
}

static inline int gpu_m4_fetch_init_locked(struct gpu_m4_fetch *f,
		const struct gpu_m4_fetch_config *config)
{
	unsigned int i;
	int ret;

	if (!f || !config || !config->provider || !config->session || !config->ops ||
	    !config->max_pages || config->max_pages > GPU_M4_FETCH_MAX_PAGES ||
	    !config->max_claims || config->max_claims > GPU_M4_FETCH_MAX_CLAIMS ||
	    f->initialized || f->claim_count || f->error ||
	    !gpu_m4_fetch_disjoint(f, sizeof(*f), config, sizeof(*config)))
		return -EINVAL;
	lockdep_assert_held(gpu_m4_drm_provider_lock(config->provider));
#define M4_FETCH_REQUIRED(op) if (!config->ops->op) return -EINVAL
	M4_FETCH_REQUIRED(session_get_locked); M4_FETCH_REQUIRED(table_locked);
	M4_FETCH_REQUIRED(refresh_locked); M4_FETCH_REQUIRED(backing_locked);
	M4_FETCH_REQUIRED(alias_get_locked); M4_FETCH_REQUIRED(alias_put_locked);
	M4_FETCH_REQUIRED(job_locked); M4_FETCH_REQUIRED(handoff_get_locked);
	M4_FETCH_REQUIRED(fixed_locked); M4_FETCH_REQUIRED(store_locked);
	M4_FETCH_REQUIRED(clean_locked); M4_FETCH_REQUIRED(barrier_locked);
	M4_FETCH_REQUIRED(handoff_put_locked); M4_FETCH_REQUIRED(asid0_sync_locked);
	M4_FETCH_REQUIRED(commit_locked); M4_FETCH_REQUIRED(fault_locked);
#undef M4_FETCH_REQUIRED
	for (i = 0; i < 3; i++) {
		const struct gpu_m4_fetch_table *t = &config->table[i];

		if (!t->owner || !t->cpu || !config->expected[i] ||
		    !gpu_m4_fetch_disjoint(f, sizeof(*f), t->cpu, GPU_M4_DRM_PAGE_SIZE) ||
		    !gpu_m4_fetch_disjoint(f, sizeof(*f), config->expected[i], GPU_M4_DRM_PAGE_SIZE) ||
		    ((unsigned long)t->cpu & (GPU_M4_DRM_PAGE_SIZE - 1)) ||
		    !t->pa || (t->pa & ~GPU_M4_PA_MASK))
			return -EINVAL;
		for (u32 j = 0; j < i; j++)
			if (config->table[j].cpu == t->cpu || config->table[j].pa == t->pa)
				return -EINVAL;
		ret = gpu_m4_fetch_status(config->ops->table_locked(config->session, t));
		if (ret)
			return ret;
	}
	if (le64_to_cpu(config->expected[0][GPU_M4_FETCH_ROOT_INDEX]) != (config->table[1].pa | 3) ||
	    le64_to_cpu(config->expected[1][GPU_M4_FETCH_MIDDLE_INDEX]) != (config->table[2].pa | 3) ||
	    !le64_to_cpu(config->expected[2][0]) || !le64_to_cpu(config->expected[2][2]) ||
	    !le64_to_cpu(config->expected[2][64]) || config->expected[2][1] ||
	    config->expected[2][3] || config->expected[2][65])
		return -ESTALE;
	ret = gpu_m4_fetch_status(config->ops->session_get_locked(config->session));
	if (ret)
		return ret;
	/* init owns a strong session baseline even if the first fresh check fails. */
	f->config = *config;
	for (i = 0; i < 3; i++)
		memcpy(f->expected[i], config->expected[i], GPU_M4_DRM_PAGE_SIZE);
	f->initialized = true;
	ret = gpu_m4_fetch_graph(f, NULL, true);
	return ret ? gpu_m4_fetch_fail(f, NULL, ret) : 0;
}

/* Before any finalizer, remove only a never-published detached global claim.
 * Provider puts queue garbage; caller separately reaps outside ALL locks.
 * Caller still owns the claim storage, and must never reuse published storage.
 */
static inline int gpu_m4_fetch_cancel_locked(struct gpu_m4_fetch_claim *c)
{
	struct gpu_m4_fetch *f;
	unsigned int i;

	if (!c || !c->owner || c->state != GPU_M4_FETCH_DETACHED)
		return -EBUSY;
	f = c->owner;
	lockdep_assert_held(gpu_m4_drm_provider_lock(f->config.provider));
	for (i = 0; i < f->claim_count && f->claims[i] != c; i++)
		;
	if (i == f->claim_count)
		return -ENOENT;
	f->claims[i] = f->claims[--f->claim_count];
	if (c->alias_ref) {
		int ret = f->config.ops->alias_put_locked(f->config.session, c);

		if (ret) {
			f->claims[f->claim_count++] = c;
			return gpu_m4_fetch_fail(f, c, ret);
		}
		c->alias_ref = false;
	}
	if (c->source_ref) {
		int ret = gpu_m4_drm_provider_version_put_locked(c->request.source);

		if (ret) {
			f->claims[f->claim_count++] = c;
			return gpu_m4_fetch_fail(f, c, ret);
		}
		c->source_ref = false;
	}
	if (c->object_ref) {
		gpu_m4_drm_provider_object_put_locked(c->request.work.object);
		c->object_ref = false;
	}
	if (c->private_ref) {
		gpu_m4_drm_provider_private_put_locked(c->request.work.lease);
		c->private_ref = false;
	}
	if (c->vm_ref) {
		gpu_m4_drm_provider_vm_put_locked(c->request.vm);
		c->vm_ref = false;
	}
	c->state = GPU_M4_FETCH_CANCELLED;
	return 0;
}

static inline int gpu_m4_fetch_backing(struct gpu_m4_fetch_claim *c)
{
	struct gpu_m4_fetch *f = c->owner;
	const struct gpu_m4_drm_provider_private_gem *g = &c->request.work;
	unsigned int i;
	int ret;

	ret = gpu_m4_fetch_status(f->config.ops->backing_locked(f->config.session, c));
	if (ret)
		return ret;
	ret = gpu_m4_drm_provider_private_check_locked(g->lease, c->request.vm, g->va, g->size);
	if (ret)
		return gpu_m4_fetch_status(ret);
	ret = gpu_m4_drm_provider_object_origin_locked(f->config.provider, c->request.source, g->object);
	if (ret)
		return gpu_m4_fetch_status(ret);
	if (c->pin != gpu_m4_drm_provider_object_native(g->object) ||
	    gpu_m4_drm_bo_pin_object(c->pin) != gpu_m4_drm_provider_object_identity(g->object) ||
	    gpu_m4_drm_bo_policy(gpu_m4_drm_bo_pin_object(c->pin)) != GPU_M4_DRM_CPU_NC ||
	    g->offset > gpu_m4_drm_bo_pin_size(c->pin) ||
	    g->size > gpu_m4_drm_bo_pin_size(c->pin) - g->offset)
		return -EPERM;
	for (i = 0; i < c->pages; i++) {
		u64 pa;

		ret = gpu_m4_drm_provider_object_page_locked(g->object,
			g->offset + (u64)i * GPU_M4_DRM_PAGE_SIZE, &pa);
		if (ret)
			return gpu_m4_fetch_status(ret);
		if (!pa || (pa & ~GPU_M4_PA_MASK) || pa != c->pa[i])
			return -ESTALE;
		for (u32 j = 0; j < i; j++)
			if (pa == c->pa[j])
				return -EOPNOTSUPP; /* Command pages are not SINGLE_PAGE repetitions. */
	}
	return 0;
}

/* Explicit stage-selected VA, no auto-search or private mapping append here.
 * Acceptance is detached, before core clone/cache/resv creation. Caller supplies
 * zeroed heap claim storage. Every successful ref is recorded before failure;
 * rollback can dispose only this never-published owner through cancel_locked.
 */
static inline int gpu_m4_fetch_accept_locked(struct gpu_m4_fetch *f,
		struct gpu_m4_fetch_claim *c, const struct gpu_m4_fetch_request *request)
{
	const struct gpu_m4_drm_provider_private_gem *g;
	u64 end;
	unsigned int i;
	int ret;

	if (!f || !f->initialized || !c || !request || c->state != GPU_M4_FETCH_UNUSED ||
	    c->owner || c->vm_ref || c->source_ref || c->object_ref || c->private_ref || c->alias_ref ||
	    !request->vm || !request->source || !request->work.object || !request->work.lease ||
	    !request->upper || !request->upper_lease ||
	    !gpu_m4_fetch_disjoint(f, sizeof(*f), c, sizeof(*c)) ||
	    !gpu_m4_fetch_disjoint(c, sizeof(*c), request, sizeof(*request)))
		return -EINVAL;
	lockdep_assert_held(gpu_m4_drm_provider_lock(f->config.provider));
	if (f->error)
		return f->error;
	if (f->claim_count >= f->config.max_claims)
		return -ENOSPC;
	for (i = 0; i < 3; i++)
		if (!gpu_m4_fetch_disjoint(c, sizeof(*c), f->config.table[i].cpu, GPU_M4_DRM_PAGE_SIZE) ||
		    !gpu_m4_fetch_disjoint(c, sizeof(*c), f->config.expected[i], GPU_M4_DRM_PAGE_SIZE))
			return -EINVAL;
	g = &request->work;
	if (!g->size || ((g->va | g->offset | g->size) & (GPU_M4_DRM_PAGE_SIZE - 1)) ||
	    g->size / GPU_M4_DRM_PAGE_SIZE > f->config.max_pages ||
	    g->access != (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE) ||
	    g->va < GPU_M4_FETCH_START || g->va >= GPU_M4_FETCH_END ||
	    g->size > GPU_M4_FETCH_END - g->va - GPU_M4_DRM_PAGE_SIZE ||
	    g->size > ~0ULL - g->offset ||
	    gpu_m4_drm_provider_vm_provider(request->vm) != f->config.provider)
		return -EINVAL;
	ret = gpu_m4_fetch_graph(f, NULL, true);
	if (ret)
		return gpu_m4_fetch_fail(f, NULL, ret);
	end = (g->va - GPU_M4_FETCH_START + g->size) / GPU_M4_DRM_PAGE_SIZE;
	for (i = (g->va - GPU_M4_FETCH_START) / GPU_M4_DRM_PAGE_SIZE; i <= end; i++) {
		if (gpu_m4_fetch_reserved_guard(i) || f->expected[2][i] ||
		    READ_ONCE(f->config.table[2].cpu[i]))
			return -EEXIST;
		for (u32 j = 0; j < f->claim_count; j++)
			if (i >= f->claims[j]->first && i <= f->claims[j]->first + f->claims[j]->pages)
				return -EEXIST;
	}
	ret = gpu_m4_drm_provider_version_check_origin_locked(f->config.provider,
		request->vm, request->source);
	if (ret)
		return gpu_m4_fetch_status(ret);
	c->owner = f; c->request = *request;
	c->first = (g->va - GPU_M4_FETCH_START) / GPU_M4_DRM_PAGE_SIZE;
	c->pages = g->size / GPU_M4_DRM_PAGE_SIZE;
	c->state = GPU_M4_FETCH_DETACHED;
	f->claims[f->claim_count++] = c;
	ret = gpu_m4_drm_provider_vm_get_locked(request->vm);
	if (ret)
		goto cancel;
	c->vm_ref = true;
	ret = gpu_m4_drm_provider_version_get_locked(request->source);
	if (ret)
		goto cancel;
	c->source_ref = true;
	ret = gpu_m4_drm_provider_object_get_locked(g->object);
	if (ret)
		goto cancel;
	c->object_ref = true;
	ret = gpu_m4_drm_provider_private_get_locked(g->lease);
	if (ret)
		goto cancel;
	c->private_ref = true;
	ret = gpu_m4_fetch_status(f->config.ops->alias_get_locked(f->config.session, c));
	if (ret)
		goto cancel;
	c->alias_ref = true;
	c->pin = gpu_m4_drm_provider_object_native(g->object);
	for (i = 0; i < c->pages; i++) {
		ret = gpu_m4_drm_provider_object_page_locked(g->object,
			g->offset + (u64)i * GPU_M4_DRM_PAGE_SIZE, &c->pa[i]);
		if (ret)
			goto cancel;
		c->value[i] = c->pa[i] | GPU_M4_FETCH_LOWER_ATTRS;
	}
	ret = gpu_m4_fetch_backing(c);
	if (!ret)
		for (i = 0; i + 1 < f->claim_count; i++)
			for (u32 own = 0; own < c->pages; own++)
				for (u32 prior = 0; prior < f->claims[i]->pages; prior++)
					if (c->pa[own] == f->claims[i]->pa[prior])
						ret = -EEXIST;
	if (!ret)
		return 0;
cancel:
	ret = gpu_m4_fetch_status(ret);
	{
		int rollback = gpu_m4_fetch_cancel_locked(c);

		return rollback ? rollback : ret;
	}
}

/* Actual current protected-job aliases/cache proof. No hardware writes. Full
 * READY/captured graphs are the adapter's separate outside-handoff callback.
 */
static inline int gpu_m4_fetch_job_check(struct gpu_m4_fetch_claim *c,
		const struct gpu_m4_fetch_job *job)
{
	struct gpu_m4_fetch *f = c->owner;
	const struct gpu_m4_drm_provider_private_gem *g = &c->request.work;
	struct gpu_m4_drm_provider_job_view view = { 0 };
	struct gpu_m4_drm_cache_job *checked_cache = NULL;
	u64 upper;
	unsigned int i;
	int ret;

	if (!job || !job->job || !job->old_version || !job->version || !job->snapshot ||
	    !job->cache || !job->lane || !job->item || !job->stage_owner ||
	    !job->binding_generation || job->binding_generation > 255)
		return -EINVAL;
	ret = gpu_m4_drm_provider_job_check_owner_locked(job->job, f->config.provider);
	if (ret)
		return gpu_m4_fetch_status(ret);
	gpu_m4_drm_provider_job_view(job->job, &view);
	if (view.version != job->version || view.snapshot != job->snapshot ||
	    view.stage_owner != job->stage_owner || !view.runtime_job ||
	    !gpu_m4_drm_provider_version_native_covered_locked(job->version, job->snapshot) ||
	    !gpu_m4_drm_provider_version_same_user_locked(c->request.source, job->version) ||
	    gpu_m4_drm_provider_resident_locked(f->config.provider) != job->version ||
	    gpu_m4_drm_provider_binding_generation_locked(f->config.provider) != job->binding_generation)
		return -ESTALE;
	ret = gpu_m4_fetch_status(f->config.ops->job_locked(f->config.session, c, job));
	if (ret)
		return ret;
	ret = gpu_m4_drm_provider_job_check_access_locked(job->job, f->config.provider,
		g->object, g->offset, g->size, g->access, false, &checked_cache);
	if (ret)
		return gpu_m4_fetch_status(ret);
	if (checked_cache != job->cache)
		return -ESTALE;
	ret = gpu_m4_drm_provider_lane_check_active_locked(job->lane, f->config.provider,
		job->item, job->job);
	if (ret)
		return gpu_m4_fetch_status(ret);
	ret = gpu_m4_drm_cache_job_check_access(job->cache, c->pin, g->offset, g->size, g->access, false);
	if (ret)
		return gpu_m4_fetch_status(ret);
	ret = gpu_m4_fetch_backing(c);
	if (ret)
		return ret;
	upper = gpu_m4_drm_upper_va_locked(c->request.upper_lease);
	if (!upper || upper < GPU_M4_DRM_UPPER_START || upper >= GPU_M4_DRM_UPPER_TIMESTAMP ||
	    g->size > GPU_M4_DRM_UPPER_TIMESTAMP - upper)
		return -EINVAL;
	for (i = 0; i < c->pages; i++) {
		struct gpu_m4_drm_vm_leaf leaf;
		u64 va = g->va + (u64)i * GPU_M4_DRM_PAGE_SIZE, pa, attrs;

		ret = gpu_m4_drm_provider_version_page_locked(job->version, va, g->access, &leaf);
		if (ret)
			return gpu_m4_fetch_status(ret);
		if (leaf.pa != c->pa[i] || leaf.va != va ||
		    leaf.identity != gpu_m4_drm_bo_pin_object(c->pin) ||
		    leaf.object_offset != g->offset + (u64)i * GPU_M4_DRM_PAGE_SIZE ||
		    leaf.access != g->access || leaf.cpu_policy != GPU_M4_DRM_CPU_NC)
			return -ESTALE;
		ret = gpu_m4_drm_upper_walk_locked(c->request.upper,
			upper + (u64)i * GPU_M4_DRM_PAGE_SIZE, &pa, &attrs);
		if (ret)
			return gpu_m4_fetch_status(ret);
		if (pa != c->pa[i] || attrs != GPU_M4_DRM_UPPER_SHARED_ATTRS)
			return -ESTALE;
	}
	return 0;
}

/* Fixed, bounded reads only while firmware is excluded. No cache refresh,
 * graph recursion, allocation/dependency waits or native-oracle callback.
 */
static inline int gpu_m4_fetch_fixed(struct gpu_m4_fetch_claim *c)
{
	struct gpu_m4_fetch *f = c->owner;
	u32 i;
	int ret = gpu_m4_fetch_status(f->config.ops->fixed_locked(f->config.session, c, &c->job));

	if (ret)
		return ret;
	if (le64_to_cpu(READ_ONCE(f->config.table[0].cpu[GPU_M4_FETCH_ROOT_INDEX])) !=
	    (f->config.table[1].pa | 3) ||
	    le64_to_cpu(READ_ONCE(f->config.table[1].cpu[GPU_M4_FETCH_MIDDLE_INDEX])) !=
	    (f->config.table[2].pa | 3))
		return -ESTALE;
	for (i = 0; i <= c->pages; i++)
		if (READ_ONCE(f->config.table[2].cpu[c->first + i]) ||
		    f->expected[2][c->first + i])
			return -EEXIST;
	for (i = 0; i < 3; i++) {
		static const u32 guard[] = { 1, 3, 65 };

		if (READ_ONCE(f->config.table[2].cpu[guard[i]]))
			return -ESTALE;
	}
	return 0;
}

/* First bounded live transaction. The candidate client root must already be
 * actually selected/completed; this helper never binds roots or advances bind
 * generation. No producer can expose the command until COMMITTED. Any failed
 * publication is terminal and retains all successfully acquired refs/storage.
 */
static inline int gpu_m4_fetch_publish_locked(struct gpu_m4_fetch_claim *c,
		const struct gpu_m4_fetch_job *job)
{
	struct gpu_m4_fetch_update updates[GPU_M4_FETCH_MAX_PAGES];
	struct gpu_m4_fetch *f;
	unsigned int i;
	int ret, release;

	if (!c || !c->owner || c->state != GPU_M4_FETCH_DETACHED)
		return -EINVAL;
	f = c->owner;
	lockdep_assert_held(gpu_m4_drm_provider_lock(f->config.provider));
	if (f->error)
		return f->error;
	ret = gpu_m4_fetch_claim_check(c);
	if (!ret)
		ret = gpu_m4_fetch_graph(f, NULL, true);
	if (!ret)
		ret = gpu_m4_fetch_job_check(c, job);
	if (ret)
		return gpu_m4_fetch_fail(f, c, ret);
	c->job = *job;
	/* Keep partial acquisitions on every subsequent error, including before
	 * a store. Final job/snapshot releases would acquire other locks; no
	 * speculative rollback or late frees belong in this transaction.
	 */
	ret = gpu_m4_drm_provider_version_get_locked(job->old_version);
	if (ret)
		goto fail;
	c->old_ref = true;
	ret = gpu_m4_drm_provider_version_get_locked(job->version);
	if (ret)
		goto fail;
	c->version_ref = true;
	ret = gpu_m4_drm_vm_owner_get(job->snapshot);
	if (ret)
		goto fail;
	c->snapshot_ref = true;
	ret = gpu_m4_drm_provider_job_get_locked(job->job);
	if (ret)
		goto fail;
	c->job_ref = true;
	for (i = 0; i < c->pages; i++)
		updates[i] = (struct gpu_m4_fetch_update){ c->first + i, c->value[i] };
	ret = gpu_m4_fetch_status(f->config.ops->handoff_get_locked(f->config.session, 100));
	if (ret)
		goto fail;
	ret = gpu_m4_fetch_fixed(c);
	if (!ret) {
		c->state = GPU_M4_FETCH_UNCERTAIN;
		for (i = 0; i < c->pages; i++) {
			ret = gpu_m4_fetch_status(f->config.ops->store_locked(f->config.session,
				&f->config.table[2].cpu[c->first + i], cpu_to_le64(c->value[i])));
			if (ret)
				break;
		}
	}
	if (!ret)
		ret = gpu_m4_fetch_status(f->config.ops->clean_locked(f->config.session,
			&f->config.table[2].cpu[c->first], c->pages * sizeof(__le64)));
	if (!ret)
		ret = gpu_m4_fetch_status(f->config.ops->barrier_locked(f->config.session));
	if (!ret)
		for (i = 0; i < c->pages; i++)
			if (le64_to_cpu(READ_ONCE(f->config.table[2].cpu[c->first + i])) != c->value[i]) {
				ret = -EIO;
				break;
			}
	release = gpu_m4_fetch_status(f->config.ops->handoff_put_locked(f->config.session));
	if (!ret)
		ret = release;
	if (ret)
		goto fail;
	ret = gpu_m4_fetch_status(f->config.ops->asid0_sync_locked(f->config.session, 0));
	if (!ret)
		ret = gpu_m4_fetch_graph(f, c, true);
	if (!ret)
		ret = gpu_m4_fetch_job_check(c, job);
	if (!ret)
		ret = gpu_m4_fetch_status(f->config.ops->commit_locked(f->config.session, updates, c->pages));
	if (ret)
		goto fail;
	/* Only precomputed selected leaves become the authoritative mirror. */
	for (i = 0; i < c->pages; i++)
		f->expected[2][c->first + i] = cpu_to_le64(c->value[i]);
	c->state = GPU_M4_FETCH_COMMITTED;
	return 0;
fail:
	return gpu_m4_fetch_fail(f, c, ret);
}

/* Independent health for all committed lower aliases, including the original
 * full parent/leaf inventory. Does not silently admit observed drift or revoke
 * published references. Pure detached claims still reserve their zero guards.
 */
static inline int gpu_m4_fetch_verify_locked(struct gpu_m4_fetch *f)
{
	unsigned int i, j;
	int ret;

	if (!f || !f->initialized)
		return -EINVAL;
	lockdep_assert_held(gpu_m4_drm_provider_lock(f->config.provider));
	if (f->error)
		return f->error;
	ret = gpu_m4_fetch_graph(f, NULL, true);
	if (ret)
		return ret;
	for (i = 0; i < f->claim_count; i++) {
		struct gpu_m4_fetch_claim *c = f->claims[i];

		ret = gpu_m4_fetch_claim_check(c);
		if (!ret)
			ret = gpu_m4_fetch_backing(c);
		if (ret)
			return ret;
		if (c->state != GPU_M4_FETCH_DETACHED && c->state != GPU_M4_FETCH_COMMITTED)
			return -EPROTO;
		for (j = 0; j <= c->pages; j++) {
			u64 expected = j < c->pages && c->state == GPU_M4_FETCH_COMMITTED ? c->value[j] : 0;

			if (le64_to_cpu(f->expected[2][c->first + j]) != expected)
				return -ESTALE;
		}
	}
	return 0;
}

#endif
