/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_DRM_VM_LEDGER_H
#define APPLE_GPU_M4_DRM_VM_LEDGER_H

#include <linux/slab.h>

#include "gpu-m4-drm.h"
#include "gpu-m4-drm-validation.h"
#include "gpu-m4-format.h"

/*
 * Dynamic VA ownership, separate from the intentionally restrictive sealed
 * first-store UAT helper. No PTE write, cache maintenance, TLBI or pin/ref
 * operation occurs here. The provider retains each identity/backing while any
 * active/planned alias or in-flight job names it, including uncertain jobs.
 */
struct gpu_m4_drm_vm_leaf {
	u64 va, pa, object_offset;
	const void *identity; /* stable local GEM identity, not a transient pin */
	u32 access;
	enum gpu_m4_drm_cpu_policy cpu_policy;
};

struct gpu_m4_drm_vm_backing {
	const void *identity;
	const u64 *pa; /* immutable PA array from retained, pinned local shmem */
	u64 size;
	enum gpu_m4_drm_cpu_policy cpu_policy;
};

struct gpu_m4_drm_vm_change {
	struct drm_asahi_gem_bind_op op;
	const struct gpu_m4_drm_vm_backing *backing; /* NULL for exact unbind */
	/* Default false for public/raw changes. ONLY the provider's explicit
	 * native entrance sets this after retaining/checking the actual native
	 * GEM pin. No numeric handle is synthesized or stored in the ledger.
	 */
	bool internal_native;
};

struct gpu_m4_drm_vm_ledger {
	struct gpu_m4_drm_vm_leaf *leaf;
	u32 count, capacity;
	u64 low, high, kernel_start, kernel_end;
	void *owner;
	/* Mandatory: prove pinned owner/offset/PA and reject firmware/table/MMIO. */
	int (*validate_page)(void *owner, const void *identity, u64 offset, u64 pa);
};

struct gpu_m4_drm_vm_access_page {
	const void *identity;
	u64 pa, object_offset;
	u32 access;
	enum gpu_m4_drm_cpu_policy cpu_policy;
};

struct gpu_m4_drm_vm_access_object {
	const void *identity;
	u64 offset, size; /* conservative bounding span, including mapped holes */
	u32 access;
	enum gpu_m4_drm_cpu_policy cpu_policy;
};

static inline u64 gpu_m4_drm_vm_leaf_attrs(u32 access)
{
	if (access == DRM_ASAHI_BIND_READ)
		return 0x0080000000000c8bULL;
	if (access == DRM_ASAHI_BIND_WRITE)
		return 0x00a0000000000c8bULL;
	if (access == (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE))
		return 0x00c0000000000c8bULL;
	return 0;
}

static inline bool gpu_m4_drm_vm_ledger_range(const struct gpu_m4_drm_vm_ledger *l,
					    u64 start, u64 size)
{
	return gpu_m4_drm_aligned(start) && gpu_m4_drm_aligned(size) &&
		gpu_m4_drm_range(start, size, l->low, l->high) &&
		(start >= l->kernel_end || start + size <= l->kernel_start);
}

static inline int gpu_m4_drm_vm_ledger_shape(const struct gpu_m4_drm_vm_ledger *l)
{
	if (!l || !l->leaf || !l->capacity || l->count > l->capacity ||
	    !l->validate_page || l->low < GPU_M4_DRM_PAGE_SIZE ||
	    l->low >= l->high || l->high > (1ULL << 42) ||
	    !gpu_m4_drm_aligned(l->low) || !gpu_m4_drm_aligned(l->high) ||
	    !gpu_m4_drm_aligned(l->kernel_start) ||
	    !gpu_m4_drm_aligned(l->kernel_end) || l->kernel_start < l->low ||
	    l->kernel_end > l->high || l->kernel_start >= l->kernel_end)
		return -EINVAL;
	return 0;
}

/* These invariants also apply to retained backing removed by a replacement. */
static inline int gpu_m4_drm_vm_leaf_alias_validate(
	const struct gpu_m4_drm_vm_leaf *old, const struct gpu_m4_drm_vm_leaf *p)
{
	if (old->pa == p->pa && (old->identity != p->identity ||
	    old->object_offset != p->object_offset || old->cpu_policy != p->cpu_policy))
		return -EPERM;
	if (old->identity == p->identity &&
	    (old->cpu_policy != p->cpu_policy ||
	     (old->object_offset == p->object_offset && old->pa != p->pa)))
		return -EPROTO;
	return 0;
}

/* Static leaf shape only: no ownership oracle and no alias comparison. */
static inline int gpu_m4_drm_vm_leaf_shape(const struct gpu_m4_drm_vm_ledger *l,
					const struct gpu_m4_drm_vm_leaf *p)
{
	if (!p->identity || !gpu_m4_drm_vm_ledger_range(l, p->va, GPU_M4_DRM_PAGE_SIZE) ||
	    !gpu_m4_drm_aligned(p->object_offset) ||
	    p->object_offset > ~0ULL - GPU_M4_DRM_PAGE_SIZE || !p->pa ||
	    !gpu_m4_drm_aligned(p->pa) || (p->pa & ~GPU_M4_PA_MASK) ||
	    !gpu_m4_drm_vm_leaf_attrs(p->access) ||
	    (p->cpu_policy != GPU_M4_DRM_CPU_NC &&
	     p->cpu_policy != GPU_M4_DRM_CPU_WRITEBACK))
		return -EINVAL;
	return 0;
}

/* Every repeated PA must name the SAME pinned object page and CPU policy. */
static inline int gpu_m4_drm_vm_leaf_validate(const struct gpu_m4_drm_vm_ledger *l,
					   const struct gpu_m4_drm_vm_leaf *p,
					   u32 prior)
{
	if (gpu_m4_drm_vm_leaf_shape(l, p))
		return -EINVAL;
	for (u32 i = 0; i < prior; i++) {
		const struct gpu_m4_drm_vm_leaf *old = &l->leaf[i];
		int ret;

		if (old->va == p->va)
			return -EEXIST;
		ret = gpu_m4_drm_vm_leaf_alias_validate(old, p);
		if (ret)
			return ret;
	}
	return l->validate_page(l->owner, p->identity, p->object_offset, p->pa);
}

/* Private indices never grant ownership: every build still checks every leaf
 * with the exact retained page oracle. Sorting pointers preserves ledger order,
 * including callers' detached plans and immutable job snapshots.
 */
struct gpu_m4_drm_vm_leaf_index {
	const struct gpu_m4_drm_vm_leaf **va, **pa, **identity;
	u32 count;
};

enum gpu_m4_drm_vm_leaf_key {
	GPU_M4_DRM_VM_KEY_VA,
	GPU_M4_DRM_VM_KEY_PA,
	GPU_M4_DRM_VM_KEY_IDENTITY,
};

static inline bool gpu_m4_drm_vm_leaf_less(const struct gpu_m4_drm_vm_leaf *a,
	const struct gpu_m4_drm_vm_leaf *b, enum gpu_m4_drm_vm_leaf_key key)
{
	if (key == GPU_M4_DRM_VM_KEY_VA)
		return a->va < b->va;
	if (key == GPU_M4_DRM_VM_KEY_PA)
		return a->pa < b->pa;
	if (a->identity != b->identity)
		return (unsigned long)a->identity < (unsigned long)b->identity;
	return a->object_offset < b->object_offset;
}

/* Iterative heapsort, O(N log N), with no recursion or ledger mutation. */
static inline void gpu_m4_drm_vm_leaf_sift(const struct gpu_m4_drm_vm_leaf **a,
	u32 count, u32 root, enum gpu_m4_drm_vm_leaf_key key)
{
	while (root < count / 2) {
		u32 child = root * 2 + 1;
		const struct gpu_m4_drm_vm_leaf *tmp;

		if (child + 1 < count && gpu_m4_drm_vm_leaf_less(a[child], a[child + 1], key))
			child++;
		if (!gpu_m4_drm_vm_leaf_less(a[root], a[child], key))
			return;
		tmp = a[root];
		a[root] = a[child];
		a[child] = tmp;
		root = child;
	}
}

static inline void gpu_m4_drm_vm_leaf_sort(const struct gpu_m4_drm_vm_leaf **a,
	u32 count, enum gpu_m4_drm_vm_leaf_key key)
{
	if (count < 2)
		return;
	for (u32 parent = count / 2; parent;)
		gpu_m4_drm_vm_leaf_sift(a, count, --parent, key);
	for (u32 end = count - 1; end; end--) {
		const struct gpu_m4_drm_vm_leaf *tmp = a[0];

		a[0] = a[end];
		a[end] = tmp;
		gpu_m4_drm_vm_leaf_sift(a, end, 0, key);
	}
}

static inline int gpu_m4_drm_vm_leaf_index_build(const struct gpu_m4_drm_vm_ledger *l,
	struct gpu_m4_drm_vm_leaf_index *index,
	const struct gpu_m4_drm_vm_leaf **storage, u32 capacity)
{
	int ret = gpu_m4_drm_vm_ledger_shape(l);

	if (ret)
		return ret;
	if (!storage || capacity < l->count)
		return -ENOSPC;
	*index = (struct gpu_m4_drm_vm_leaf_index) {
		.va = storage, .pa = storage + capacity,
		.identity = storage + capacity + capacity, .count = l->count,
	};
	for (u32 i = 0; i < l->count; i++) {
		ret = gpu_m4_drm_vm_leaf_validate(l, &l->leaf[i], 0);
		if (ret)
			return ret;
		index->va[i] = index->pa[i] = index->identity[i] = &l->leaf[i];
	}
	gpu_m4_drm_vm_leaf_sort(index->va, l->count, GPU_M4_DRM_VM_KEY_VA);
	for (u32 i = 1; i < l->count; i++)
		if (index->va[i - 1]->va == index->va[i]->va)
			return -EEXIST;
	gpu_m4_drm_vm_leaf_sort(index->pa, l->count, GPU_M4_DRM_VM_KEY_PA);
	for (u32 i = 1; i < l->count; i++) {
		ret = gpu_m4_drm_vm_leaf_alias_validate(index->pa[i - 1], index->pa[i]);
		if (ret)
			return ret;
	}
	gpu_m4_drm_vm_leaf_sort(index->identity, l->count, GPU_M4_DRM_VM_KEY_IDENTITY);
	for (u32 i = 1; i < l->count; i++) {
		ret = gpu_m4_drm_vm_leaf_alias_validate(index->identity[i - 1], index->identity[i]);
		if (ret)
			return ret;
	}
	return 0;
}

static inline u32 gpu_m4_drm_vm_leaf_lower_bound(const struct gpu_m4_drm_vm_leaf **a,
	u32 count, const struct gpu_m4_drm_vm_leaf *p, enum gpu_m4_drm_vm_leaf_key key)
{
	u32 first = 0, end = count;

	while (first < end) {
		u32 middle = first + (end - first) / 2;

		if (gpu_m4_drm_vm_leaf_less(a[middle], p, key))
			first = middle + 1;
		else
			end = middle;
	}
	return first;
}

/* The old manifest remains authoritative even for leaves being replaced. */
static inline int gpu_m4_drm_vm_leaf_index_alias(const struct gpu_m4_drm_vm_leaf_index *index,
	const struct gpu_m4_drm_vm_leaf *p)
{
	struct gpu_m4_drm_vm_leaf first = *p;
	u32 at;
	int ret;

	at = gpu_m4_drm_vm_leaf_lower_bound(index->pa, index->count, p, GPU_M4_DRM_VM_KEY_PA);
	if (at < index->count && index->pa[at]->pa == p->pa) {
		ret = gpu_m4_drm_vm_leaf_alias_validate(index->pa[at], p);
		if (ret)
			return ret;
	}
	first.object_offset = 0;
	at = gpu_m4_drm_vm_leaf_lower_bound(index->identity, index->count, &first,
		GPU_M4_DRM_VM_KEY_IDENTITY);
	if (at < index->count && index->identity[at]->identity == p->identity &&
	    index->identity[at]->cpu_policy != p->cpu_policy)
		return -EPROTO;
	at = gpu_m4_drm_vm_leaf_lower_bound(index->identity, index->count, p,
		GPU_M4_DRM_VM_KEY_IDENTITY);
	if (at < index->count && index->identity[at]->identity == p->identity &&
	    index->identity[at]->object_offset == p->object_offset)
		return gpu_m4_drm_vm_leaf_alias_validate(index->identity[at], p);
	return 0;
}

static inline int gpu_m4_drm_vm_ledger_validate(const struct gpu_m4_drm_vm_ledger *l)
{
	struct gpu_m4_drm_vm_leaf_index index;
	const struct gpu_m4_drm_vm_leaf **storage;
	int ret = gpu_m4_drm_vm_ledger_shape(l);

	if (ret || !l->count)
		return ret;
	storage = kvcalloc(l->count, 3 * sizeof(*storage), GFP_KERNEL);
	if (!storage)
		return -ENOMEM;
	ret = gpu_m4_drm_vm_leaf_index_build(l, &index, storage, l->count);
	kvfree(storage);
	return ret;
}

/*
 * Retained VA/PA/identity indexes of ONE immutable, already validated leaf
 * array. Entries are leaf positions, not pointers, so an exact copy of the
 * leaves keeps the index valid. The index is created by the full validation
 * below, or derived for a detached candidate from a validated source plus the
 * proven delta. A retained index is never edited: a mutation builds its own
 * candidate index, and an unchanged captured owner keeps its index for its
 * whole refcounted lifetime. Possessing an index grants no page ownership;
 * every newly introduced page still passes the exact retained page oracle.
 */
struct gpu_m4_drm_vm_ledger_index {
	u32 *va, *pa, *identity;
	u32 count;
};

static inline void gpu_m4_drm_vm_ledger_index_free(struct gpu_m4_drm_vm_ledger_index *index)
{
	kvfree(index->va);
	index->va = index->pa = index->identity = NULL;
	index->count = 0;
}

static inline int gpu_m4_drm_vm_ledger_index_alloc(struct gpu_m4_drm_vm_ledger_index *index,
						   u32 count)
{
	u32 *storage = NULL;

	if (count) {
		storage = kvcalloc(count, 3 * sizeof(*storage), GFP_KERNEL);
		if (!storage)
			return -ENOMEM;
	}
	index->va = storage;
	index->pa = storage ? storage + count : NULL;
	index->identity = storage ? storage + 2 * (u64)count : NULL;
	index->count = count;
	return 0;
}

/* Structural pairing only. Validity is the authority of its creator. */
static inline bool gpu_m4_drm_vm_ledger_index_valid(const struct gpu_m4_drm_vm_ledger *l,
	const struct gpu_m4_drm_vm_ledger_index *index)
{
	return l && index && index->count == l->count &&
		(!l->count || (l->leaf && index->va && index->pa && index->identity));
}

static inline int gpu_m4_drm_vm_ledger_index_copy(struct gpu_m4_drm_vm_ledger_index *out,
	const struct gpu_m4_drm_vm_ledger_index *index)
{
	int ret = gpu_m4_drm_vm_ledger_index_alloc(out, index->count);

	if (!ret && index->count)
		memcpy(out->va, index->va, 3 * sizeof(*out->va) * (u64)index->count);
	return ret;
}

/* Position heapsort: the exact comparisons and swaps of leaf_sort above, so
 * a positional build visits the same adjacent pairs as ledger_validate().
 */
static inline void gpu_m4_drm_vm_pos_sift(const struct gpu_m4_drm_vm_leaf *leaf, u32 *a,
	u32 count, u32 root, enum gpu_m4_drm_vm_leaf_key key)
{
	while (root < count / 2) {
		u32 child = root * 2 + 1, tmp;

		if (child + 1 < count &&
		    gpu_m4_drm_vm_leaf_less(&leaf[a[child]], &leaf[a[child + 1]], key))
			child++;
		if (!gpu_m4_drm_vm_leaf_less(&leaf[a[root]], &leaf[a[child]], key))
			return;
		tmp = a[root];
		a[root] = a[child];
		a[child] = tmp;
		root = child;
	}
}

static inline void gpu_m4_drm_vm_pos_sort(const struct gpu_m4_drm_vm_leaf *leaf, u32 *a,
	u32 count, enum gpu_m4_drm_vm_leaf_key key)
{
	if (count < 2)
		return;
	for (u32 parent = count / 2; parent;)
		gpu_m4_drm_vm_pos_sift(leaf, a, count, --parent, key);
	for (u32 end = count - 1; end; end--) {
		u32 tmp = a[0];

		a[0] = a[end];
		a[end] = tmp;
		gpu_m4_drm_vm_pos_sift(leaf, a, end, 0, key);
	}
}

static inline u32 gpu_m4_drm_vm_pos_lower_bound(const struct gpu_m4_drm_vm_leaf *leaf,
	const u32 *a, u32 count, const struct gpu_m4_drm_vm_leaf *p, enum gpu_m4_drm_vm_leaf_key key)
{
	u32 first = 0, end = count;

	while (first < end) {
		u32 middle = first + (end - first) / 2;

		if (gpu_m4_drm_vm_leaf_less(&leaf[a[middle]], p, key))
			first = middle + 1;
		else
			end = middle;
	}
	return first;
}

/* Sort identity-initialized positions and prove the pairwise alias contract. */
static inline int gpu_m4_drm_vm_ledger_index_sort(const struct gpu_m4_drm_vm_leaf *leaf,
	struct gpu_m4_drm_vm_ledger_index *index)
{
	u32 count = index->count;
	int ret;

	gpu_m4_drm_vm_pos_sort(leaf, index->va, count, GPU_M4_DRM_VM_KEY_VA);
	for (u32 i = 1; i < count; i++)
		if (leaf[index->va[i - 1]].va == leaf[index->va[i]].va)
			return -EEXIST;
	gpu_m4_drm_vm_pos_sort(leaf, index->pa, count, GPU_M4_DRM_VM_KEY_PA);
	for (u32 i = 1; i < count; i++) {
		ret = gpu_m4_drm_vm_leaf_alias_validate(&leaf[index->pa[i - 1]], &leaf[index->pa[i]]);
		if (ret)
			return ret;
	}
	gpu_m4_drm_vm_pos_sort(leaf, index->identity, count, GPU_M4_DRM_VM_KEY_IDENTITY);
	for (u32 i = 1; i < count; i++) {
		ret = gpu_m4_drm_vm_leaf_alias_validate(&leaf[index->identity[i - 1]],
							&leaf[index->identity[i]]);
		if (ret)
			return ret;
	}
	return 0;
}

/* Exactly ledger_validate(): same order of shape, oracle and alias results,
 * but the proven indexes are retained instead of discarded.
 */
static inline int gpu_m4_drm_vm_ledger_index_create(const struct gpu_m4_drm_vm_ledger *l,
	struct gpu_m4_drm_vm_ledger_index *index)
{
	int ret = gpu_m4_drm_vm_ledger_shape(l);

	*index = (struct gpu_m4_drm_vm_ledger_index) { 0 };
	if (ret)
		return ret;
	ret = gpu_m4_drm_vm_ledger_index_alloc(index, l->count);
	if (ret)
		return ret;
	for (u32 i = 0; i < l->count; i++) {
		ret = gpu_m4_drm_vm_leaf_validate(l, &l->leaf[i], 0);
		if (ret)
			goto fail;
		index->va[i] = index->pa[i] = index->identity[i] = i;
	}
	ret = gpu_m4_drm_vm_ledger_index_sort(l->leaf, index);
	if (!ret)
		return 0;
fail:
	gpu_m4_drm_vm_ledger_index_free(index);
	return ret;
}

/* Positional twin of leaf_index_alias(): one consistent group member suffices. */
static inline int gpu_m4_drm_vm_ledger_index_alias(const struct gpu_m4_drm_vm_leaf *leaf,
	const struct gpu_m4_drm_vm_ledger_index *index, const struct gpu_m4_drm_vm_leaf *p)
{
	struct gpu_m4_drm_vm_leaf first = *p;
	const struct gpu_m4_drm_vm_leaf *q;
	u32 at;
	int ret;

	at = gpu_m4_drm_vm_pos_lower_bound(leaf, index->pa, index->count, p, GPU_M4_DRM_VM_KEY_PA);
	if (at < index->count && leaf[index->pa[at]].pa == p->pa) {
		ret = gpu_m4_drm_vm_leaf_alias_validate(&leaf[index->pa[at]], p);
		if (ret)
			return ret;
	}
	first.object_offset = 0;
	at = gpu_m4_drm_vm_pos_lower_bound(leaf, index->identity, index->count, &first,
		GPU_M4_DRM_VM_KEY_IDENTITY);
	q = at < index->count ? &leaf[index->identity[at]] : NULL;
	if (q && q->identity == p->identity && q->cpu_policy != p->cpu_policy)
		return -EPROTO;
	at = gpu_m4_drm_vm_pos_lower_bound(leaf, index->identity, index->count, p,
		GPU_M4_DRM_VM_KEY_IDENTITY);
	q = at < index->count ? &leaf[index->identity[at]] : NULL;
	if (q && q->identity == p->identity && q->object_offset == p->object_offset)
		return gpu_m4_drm_vm_leaf_alias_validate(q, p);
	return 0;
}

/* Exact VA lookup through a retained index, or U32_MAX. */
static inline u32 gpu_m4_drm_vm_ledger_index_find(const struct gpu_m4_drm_vm_leaf *leaf,
	const struct gpu_m4_drm_vm_ledger_index *index, u64 va)
{
	struct gpu_m4_drm_vm_leaf key = { .va = va };
	u32 at = gpu_m4_drm_vm_pos_lower_bound(leaf, index->va, index->count, &key,
		GPU_M4_DRM_VM_KEY_VA);

	return at < index->count && leaf[index->va[at]].va == va ? index->va[at] : ~0U;
}

/* Reverse object->range lookup: does any retained leaf name identity? */
static inline bool gpu_m4_drm_vm_ledger_index_names(const struct gpu_m4_drm_vm_leaf *leaf,
	const struct gpu_m4_drm_vm_ledger_index *index, const void *identity)
{
	struct gpu_m4_drm_vm_leaf key = { .identity = identity };
	u32 at = gpu_m4_drm_vm_pos_lower_bound(leaf, index->identity, index->count, &key,
		GPU_M4_DRM_VM_KEY_IDENTITY);

	return at < index->count && leaf[index->identity[at]].identity == identity;
}

/* Distinct L1 (VA >> 36) and L2 (VA >> 25) prefixes of a VA-sorted list. */
static inline void gpu_m4_drm_vm_ledger_prefixes(const struct gpu_m4_drm_vm_leaf *leaf,
	const u32 *va, u32 count, u32 *l1, u32 *l2)
{
	u64 last1 = ~0ULL, last2 = ~0ULL;

	*l1 = *l2 = 0;
	for (u32 i = 0; i < count; i++) {
		u64 address = leaf[va[i]].va;

		if (address >> 36 != last1) {
			last1 = address >> 36;
			(*l1)++;
		}
		if (address >> 25 != last2) {
			last2 = address >> 25;
			(*l2)++;
		}
	}
}

/* Sorted distinct identities named by an indexed ledger (for inventory pruning). */
static inline int gpu_m4_drm_vm_ledger_identities(const struct gpu_m4_drm_vm_leaf *leaf,
	const struct gpu_m4_drm_vm_ledger_index *index, const void ***out, u32 *count)
{
	const void **identity;
	u32 n = 0;

	*out = NULL;
	*count = 0;
	for (u32 i = 0; i < index->count; i++)
		if (!i || leaf[index->identity[i]].identity != leaf[index->identity[i - 1]].identity)
			n++;
	if (!n)
		return 0;
	identity = kvcalloc(n, sizeof(*identity), GFP_KERNEL);
	if (!identity)
		return -ENOMEM;
	n = 0;
	for (u32 i = 0; i < index->count; i++)
		if (!i || leaf[index->identity[i]].identity != leaf[index->identity[i - 1]].identity)
			identity[n++] = leaf[index->identity[i]].identity;
	*out = identity;
	*count = n;
	return 0;
}

static inline bool gpu_m4_drm_vm_private_storage(const struct gpu_m4_drm_vm_ledger *l,
					       const void *storage, u64 bytes)
{
	unsigned long first = (unsigned long)l->leaf;
	unsigned long second = (unsigned long)storage;
	u64 first_bytes = (u64)l->capacity * sizeof(*l->leaf);

	return storage && bytes && first_bytes <= ~0UL - first && bytes <= ~0UL - second &&
		(first >= second + bytes || second >= first + first_bytes);
}

/*
 * Command attachments are optional hints, not exhaustive accessibility. Capture
 * every mapped physical page, union permissions across RO/RW/SINGLE_PAGE aliases
 * and deduplicate the same owned page. Output storage is private scratch; the
 * provider must acquire/retain the corresponding BO and mapping/job references
 * synchronously before dropping its VM lock. No asynchronous handle lookup.
 */
static inline int gpu_m4_drm_vm_snapshot_pages(const struct gpu_m4_drm_vm_ledger *l,
					     struct gpu_m4_drm_vm_access_page *out,
					     u32 capacity, u32 *count)
{
	u32 used = 0;
	int ret;

	if (!out || !capacity || !count)
		return -EINVAL;
	*count = 0;
	ret = gpu_m4_drm_vm_ledger_validate(l);
	if (ret)
		return ret;
	if (!gpu_m4_drm_vm_private_storage(l, out, (u64)capacity * sizeof(*out)))
		return -EINVAL;
	for (u32 i = 0; i < l->count; i++) {
		const struct gpu_m4_drm_vm_leaf *p = &l->leaf[i];
		u32 j;

		for (j = 0; j < used; j++)
			if (out[j].identity == p->identity && out[j].object_offset == p->object_offset)
				break;
		if (j < used) {
			out[j].access |= p->access;
			continue;
		}
		if (used == capacity)
			return -ENOSPC;
		out[used++] = (struct gpu_m4_drm_vm_access_page) {
			.identity = p->identity, .pa = p->pa,
			.object_offset = p->object_offset, .access = p->access,
			.cpu_policy = p->cpu_policy,
		};
	}
	*count = used;
	return 0;
}

/* One implicit-reservation/cache owner per BO, with unioned GPU permissions. */
static inline int gpu_m4_drm_vm_snapshot_objects(const struct gpu_m4_drm_vm_ledger *l,
					       struct gpu_m4_drm_vm_access_object *out,
					       u32 capacity, u32 *count)
{
	u32 used = 0;
	int ret;

	if (!out || !capacity || !count)
		return -EINVAL;
	*count = 0;
	ret = gpu_m4_drm_vm_ledger_validate(l);
	if (ret)
		return ret;
	if (!gpu_m4_drm_vm_private_storage(l, out, (u64)capacity * sizeof(*out)))
		return -EINVAL;
	for (u32 i = 0; i < l->count; i++) {
		const struct gpu_m4_drm_vm_leaf *p = &l->leaf[i];
		u64 end = p->object_offset + GPU_M4_DRM_PAGE_SIZE;
		u32 j;

		for (j = 0; j < used; j++)
			if (out[j].identity == p->identity)
				break;
		if (j < used) {
			u64 first = out[j].offset < p->object_offset ? out[j].offset : p->object_offset;
			u64 last = out[j].offset + out[j].size;

			out[j].offset = first;
			out[j].size = (last > end ? last : end) - first;
			out[j].access |= p->access;
			continue;
		}
		if (used == capacity)
			return -ENOSPC;
		out[used++] = (struct gpu_m4_drm_vm_access_object) {
			.identity = p->identity, .offset = p->object_offset,
			.size = GPU_M4_DRM_PAGE_SIZE, .access = p->access,
			.cpu_policy = p->cpu_policy,
		};
	}
	*count = used;
	return 0;
}

static inline int gpu_m4_drm_vm_ledger_bind_check(const struct gpu_m4_drm_vm_ledger *l,
	const struct gpu_m4_drm_vm_change *c, u32 access, u64 pages)
{
	struct gpu_m4_drm_vm_leaf_index index;
	const struct gpu_m4_drm_vm_leaf **storage;
	u32 capacity = l->count ? l->count : 1;
	int ret;

	storage = kvcalloc(capacity, 3 * sizeof(*storage), GFP_KERNEL);
	if (!storage)
		return -ENOMEM;
	ret = gpu_m4_drm_vm_leaf_index_build(l, &index, storage, capacity);
	if (ret)
		goto out;
	for (u64 i = 0; i < pages; i++) {
		u64 offset = c->op.offset + (c->op.flags & DRM_ASAHI_BIND_SINGLE_PAGE ?
			0 : i * GPU_M4_DRM_PAGE_SIZE);
		struct gpu_m4_drm_vm_leaf p = {
			.va = c->op.addr + i * GPU_M4_DRM_PAGE_SIZE,
			.pa = c->backing->pa[offset / GPU_M4_DRM_PAGE_SIZE],
			.object_offset = offset, .identity = c->backing->identity,
			.access = access, .cpu_policy = c->backing->cpu_policy,
		};
		u32 at;

		if (c->internal_native) {
			at = gpu_m4_drm_vm_leaf_lower_bound(index.va, index.count, &p,
				GPU_M4_DRM_VM_KEY_VA);
			if (at < index.count && index.va[at]->va == p.va) {
				ret = -EEXIST;
				goto out;
			}
		}
		ret = gpu_m4_drm_vm_leaf_validate(l, &p, 0);
		if (!ret)
			ret = gpu_m4_drm_vm_leaf_index_alias(&index, &p);
		if (ret)
			goto out;
	}
out:
	kvfree(storage);
	return ret;
}

static inline int gpu_m4_drm_vm_ledger_change(struct gpu_m4_drm_vm_ledger *l,
					    const struct gpu_m4_drm_vm_change *c)
{
	const struct drm_asahi_gem_bind_op *op = &c->op;
	u32 access = op->flags & (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE);
	u32 allowed = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE |
		DRM_ASAHI_BIND_UNBIND | DRM_ASAHI_BIND_SINGLE_PAGE;
	u64 pages;
	int ret;

	if ((op->flags & ~allowed) || !gpu_m4_drm_vm_ledger_range(l, op->addr, op->range) ||
	    !gpu_m4_drm_aligned(op->offset) ||
	    (c->internal_native && (op->handle ||
	     (op->flags & (DRM_ASAHI_BIND_UNBIND | DRM_ASAHI_BIND_SINGLE_PAGE)))))
		return -EINVAL;
	pages = op->range / GPU_M4_DRM_PAGE_SIZE;
	if (op->flags & DRM_ASAHI_BIND_UNBIND) {
		u32 found = 0, keep = 0;

		if (op->flags != DRM_ASAHI_BIND_UNBIND || op->handle || op->offset || c->backing)
			return -EINVAL;
		/* Exact leaf removal: an unbound hole is an error, never a free hint. */
		for (u32 i = 0; i < l->count; i++)
			if (l->leaf[i].va >= op->addr && l->leaf[i].va < op->addr + op->range)
				found++;
		if (found != pages)
			return -ENOENT;
		for (u32 i = 0; i < l->count; i++)
			if (l->leaf[i].va < op->addr || l->leaf[i].va >= op->addr + op->range)
				l->leaf[keep++] = l->leaf[i];
		l->count = keep;
		return 0;
	}
	if ((!c->internal_native && !op->handle) || !access ||
	    !c->backing || !c->backing->identity ||
	    !c->backing->pa || !gpu_m4_drm_aligned(c->backing->size) ||
	    op->offset >= c->backing->size ||
	    (op->flags & DRM_ASAHI_BIND_SINGLE_PAGE ? GPU_M4_DRM_PAGE_SIZE : op->range) >
		c->backing->size - op->offset ||
	    (c->backing->cpu_policy != GPU_M4_DRM_CPU_NC &&
	     c->backing->cpu_policy != GPU_M4_DRM_CPU_WRITEBACK))
		return -EINVAL;
	/* Preflight the whole new span against the entire retained old manifest,
	 * before dropping replaced leaves. Binary alias lookups replace the prior
	 * pages-times-old-leaves scan; the detached result is checked once below.
	 */
	ret = gpu_m4_drm_vm_ledger_bind_check(l, c, access, pages);
	if (ret)
		return ret;
	if (!c->internal_native) {
		u32 keep = 0;

		/* Only this detached software candidate is modified. */
		for (u32 i = 0; i < l->count; i++)
			if (l->leaf[i].va < op->addr || l->leaf[i].va >= op->addr + op->range)
				l->leaf[keep++] = l->leaf[i];
		l->count = keep;
	}
	for (u64 i = 0; i < pages; i++) {
		u64 offset = op->offset + (op->flags & DRM_ASAHI_BIND_SINGLE_PAGE ?
			0 : i * GPU_M4_DRM_PAGE_SIZE);
		struct gpu_m4_drm_vm_leaf p = {
			.va = op->addr + i * GPU_M4_DRM_PAGE_SIZE,
			.pa = c->backing->pa[offset / GPU_M4_DRM_PAGE_SIZE],
			.object_offset = offset,
			.identity = c->backing->identity,
			.access = access,
			.cpu_policy = c->backing->cpu_policy,
		};
		if (l->count == l->capacity)
			return -ENOSPC;
		l->leaf[l->count++] = p;
	}
	/* Includes aliases within the new span, not just old-versus-new aliases.
	 * SINGLE_PAGE and RO/RW shadows remain legal only for one exact owned page.
	 */
	return gpu_m4_drm_vm_ledger_validate(l);
}

/* Count only newly occupied VAs, respecting earlier operations in this batch.
 * Actual backing/permission checks and staged public replacement remain in
 * ledger_change. The caller allocates the peak software manifest before
 * planning, so overlapping/repeated public binds do not consume extra quota.
 */
static inline int gpu_m4_drm_vm_ledger_plan_capacity_indexed(
	const struct gpu_m4_drm_vm_ledger *active, const struct gpu_m4_drm_vm_ledger_index *index,
	const struct gpu_m4_drm_vm_change *change, u32 count, u32 limit, u64 *capacity)
{
	const u32 *old_va = NULL;
	u32 *sorted = NULL;
	u32 used, peak;
	int ret = gpu_m4_drm_vm_ledger_shape(active);

	if (ret)
		return ret;
	if (!capacity || !change || !count || count > GPU_M4_DRM_MAX_BINDS || active->count > limit)
		return -EINVAL;
	/* Presence only: do not validate backing here. A retained source VA index
	 * replaces the temporary sort; without one, sort private positions once.
	 */
	if (active->count && gpu_m4_drm_vm_ledger_index_valid(active, index)) {
		old_va = index->va;
	} else if (active->count) {
		sorted = kvcalloc(active->count, sizeof(*sorted), GFP_KERNEL);
		if (!sorted)
			return -ENOMEM;
		for (u32 i = 0; i < active->count; i++)
			sorted[i] = i;
		gpu_m4_drm_vm_pos_sort(active->leaf, sorted, active->count, GPU_M4_DRM_VM_KEY_VA);
		old_va = sorted;
	}
	used = peak = active->count;
	for (u32 i = 0; i < count; i++) {
		const struct drm_asahi_gem_bind_op *op = &change[i].op;

		if (!gpu_m4_drm_vm_ledger_range(active, op->addr, op->range)) {
			ret = -EINVAL;
			goto out;
		}
		for (u64 offset = 0; offset < op->range; offset += GPU_M4_DRM_PAGE_SIZE) {
			u64 va = op->addr + offset;
			struct gpu_m4_drm_vm_leaf key = { .va = va };
			u32 at;
			bool present = false;

			for (u32 j = i; j; j--) {
				const struct drm_asahi_gem_bind_op *prior = &change[j - 1].op;

				if (va >= prior->addr && va - prior->addr < prior->range) {
					present = !(prior->flags & DRM_ASAHI_BIND_UNBIND);
					goto counted;
				}
			}
			at = gpu_m4_drm_vm_pos_lower_bound(active->leaf, old_va, active->count,
				&key, GPU_M4_DRM_VM_KEY_VA);
			present = at < active->count && active->leaf[old_va[at]].va == va;
		counted:
			if (op->flags & DRM_ASAHI_BIND_UNBIND) {
				if (!present) {
					ret = -ENOENT;
					goto out;
				}
				used--;
			} else if (!present) {
				if (used == limit) {
					ret = -ENOSPC;
					goto out;
				}
				used++;
				if (used > peak)
					peak = used;
			}
		}
	}
	*capacity = peak;
	ret = 0;
out:
	kvfree(sorted);
	return ret;
}

static inline int gpu_m4_drm_vm_ledger_plan_capacity(
	const struct gpu_m4_drm_vm_ledger *active,
	const struct gpu_m4_drm_vm_change *change, u32 count, u32 limit, u64 *capacity)
{
	return gpu_m4_drm_vm_ledger_plan_capacity_indexed(active, NULL, change, count,
							  limit, capacity);
}

/*
 * Stage the whole batch in distinct caller-owned storage. Failure may modify
 * the scratch plan, NEVER the active ledger or firmware-visible bytes. On
 * success the provider can plan child-before-parent PTE/cache publication and
 * ASID TLBI, then commit its ledger only after the actual mapping transaction.
 * All validation callbacks are read-only, non-publishing ownership checks.
 */
static inline int gpu_m4_drm_vm_ledger_plan(const struct gpu_m4_drm_vm_ledger *active,
					  struct gpu_m4_drm_vm_ledger *plan,
					  const struct gpu_m4_drm_vm_change *change,
					  u32 count)
{
	struct gpu_m4_drm_vm_leaf *storage;
	unsigned long first, second;
	u64 first_bytes, second_bytes;
	u32 capacity;
	int ret;

	if (!plan || !plan->leaf || plan == active || !change || !count ||
	    count > GPU_M4_DRM_MAX_BINDS)
		return -EINVAL;
	ret = gpu_m4_drm_vm_ledger_validate(active);
	if (ret)
		return ret;
	if (plan->leaf == active->leaf || plan->capacity < active->count)
		return -ENOSPC;
	/* Distinct pointers are not sufficient: scratch may overlap active+1. */
	first = (unsigned long)active->leaf;
	second = (unsigned long)plan->leaf;
	first_bytes = (u64)active->capacity * sizeof(*active->leaf);
	second_bytes = (u64)plan->capacity * sizeof(*plan->leaf);
	if (first_bytes > ~0UL - first || second_bytes > ~0UL - second ||
	    (first < second + second_bytes && second < first + first_bytes))
		return -EINVAL;
	storage = plan->leaf;
	capacity = plan->capacity;
	*plan = *active;
	plan->leaf = storage;
	plan->capacity = capacity;
	memcpy(plan->leaf, active->leaf, active->count * sizeof(*plan->leaf));
	for (u32 i = 0; i < count; i++) {
		ret = gpu_m4_drm_vm_ledger_change(plan, &change[i]);
		if (ret)
			return ret;
	}
	return 0;
}


/*
 * Changed-range planning against a retained, validated source index.
 *
 * plan_delta() is pure: no page oracle, no lock requirement, and no write to
 * the source or its index. It proves a STRICT SUPERSET of the sequential
 * reference contract (ledger_plan): every generated leaf is checked against
 * the whole retained source, including leaves this batch later replaces or
 * unbinds, and against every other generated leaf of the batch, including
 * leaves a later operation supersedes. Since a valid source is pairwise
 * consistent, that proves every intermediate and final state. Presence,
 * exact-unbind, native exclusivity and peak capacity follow the sequential
 * order exactly. Result order equals ledger_plan(): surviving source leaves
 * in source order, then surviving batch leaves in batch order. Indexes are
 * merged once, not rebuilt per operation.
 *
 * Anything the superset cannot prove (malformed input, a real conflict, an
 * alias whose only conflicting partner was removed by an earlier operation,
 * a pathological batch) returns GPU_M4_DRM_VM_PLAN_EXACT: the caller then runs
 * the unchanged sequential reference, which alone decides the outcome and
 * error code. -ENOMEM is final. On success, plan_admit() must still pass every
 * generated page through the exact page oracle before the plan is used.
 */
enum { GPU_M4_DRM_VM_PLAN_EXACT = 1 };

static inline bool gpu_m4_drm_vm_change_removes(const struct gpu_m4_drm_vm_change *c)
{
	return (c->op.flags & DRM_ASAHI_BIND_UNBIND) || !c->internal_native;
}

/* Static checks of ledger_change(), independent of the planned state. */
static inline bool gpu_m4_drm_vm_change_shape(const struct gpu_m4_drm_vm_ledger *l,
	const struct gpu_m4_drm_vm_change *c)
{
	const struct drm_asahi_gem_bind_op *op = &c->op;
	u32 access = op->flags & (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE);
	u32 allowed = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE |
		DRM_ASAHI_BIND_UNBIND | DRM_ASAHI_BIND_SINGLE_PAGE;

	if ((op->flags & ~allowed) || !gpu_m4_drm_vm_ledger_range(l, op->addr, op->range) ||
	    !gpu_m4_drm_aligned(op->offset) ||
	    (c->internal_native && (op->handle ||
	     (op->flags & (DRM_ASAHI_BIND_UNBIND | DRM_ASAHI_BIND_SINGLE_PAGE)))))
		return false;
	if (op->flags & DRM_ASAHI_BIND_UNBIND)
		return op->flags == DRM_ASAHI_BIND_UNBIND && !op->handle && !op->offset &&
			!c->backing;
	return !((!c->internal_native && !op->handle) || !access ||
		 !c->backing || !c->backing->identity ||
		 !c->backing->pa || !gpu_m4_drm_aligned(c->backing->size) ||
		 op->offset >= c->backing->size ||
		 (op->flags & DRM_ASAHI_BIND_SINGLE_PAGE ? GPU_M4_DRM_PAGE_SIZE : op->range) >
			c->backing->size - op->offset ||
		 (c->backing->cpu_policy != GPU_M4_DRM_CPU_NC &&
		  c->backing->cpu_policy != GPU_M4_DRM_CPU_WRITEBACK));
}

/* Exactly the leaf ledger_change() appends for page i of a shaped bind. */
static inline struct gpu_m4_drm_vm_leaf gpu_m4_drm_vm_change_leaf(
	const struct gpu_m4_drm_vm_change *c, u64 i)
{
	const struct drm_asahi_gem_bind_op *op = &c->op;
	u64 offset = op->offset + (op->flags & DRM_ASAHI_BIND_SINGLE_PAGE ?
		0 : i * GPU_M4_DRM_PAGE_SIZE);

	return (struct gpu_m4_drm_vm_leaf) {
		.va = op->addr + i * GPU_M4_DRM_PAGE_SIZE,
		.pa = c->backing->pa[offset / GPU_M4_DRM_PAGE_SIZE],
		.object_offset = offset, .identity = c->backing->identity,
		.access = op->flags & (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE),
		.cpu_policy = c->backing->cpu_policy,
	};
}

/* Presence seen by operation k: the last earlier covering operation decides,
 * otherwise the retained source. The same rule as plan_capacity().
 */
static inline bool gpu_m4_drm_vm_plan_present(const struct gpu_m4_drm_vm_ledger *active,
	const struct gpu_m4_drm_vm_ledger_index *index, const struct gpu_m4_drm_vm_change *change,
	u32 k, u64 va)
{
	for (u32 j = k; j; j--) {
		const struct drm_asahi_gem_bind_op *prior = &change[j - 1].op;

		if (va >= prior->addr && va - prior->addr < prior->range)
			return !(prior->flags & DRM_ASAHI_BIND_UNBIND);
	}
	return gpu_m4_drm_vm_ledger_index_find(active->leaf, index, va) != ~0U;
}

/* Merge the retained order (minus removed leaves) with sorted batch survivors. */
static inline void gpu_m4_drm_vm_plan_merge(const struct gpu_m4_drm_vm_leaf *leaf,
	const u32 *old, u32 old_count, const bool *gone, const u32 *map,
	const u32 *fresh, u32 fresh_count, const bool *dead, const u32 *fresh_map,
	u32 *out, enum gpu_m4_drm_vm_leaf_key key)
{
	u32 i = 0, j = 0, at = 0;

	for (;;) {
		while (i < old_count && gone[old[i]])
			i++;
		while (j < fresh_count && dead[fresh[j]])
			j++;
		if (i == old_count && j == fresh_count)
			return;
		if (j == fresh_count || (i < old_count &&
		    !gpu_m4_drm_vm_leaf_less(&leaf[fresh_map[fresh[j]]], &leaf[map[old[i]]], key)))
			out[at++] = map[old[i++]];
		else
			out[at++] = fresh_map[fresh[j++]];
	}
}

static inline int gpu_m4_drm_vm_ledger_plan_delta(const struct gpu_m4_drm_vm_ledger *active,
	const struct gpu_m4_drm_vm_ledger_index *index, struct gpu_m4_drm_vm_ledger *plan,
	struct gpu_m4_drm_vm_ledger_index *plan_index, const struct gpu_m4_drm_vm_change *change,
	u32 count, u32 *retained)
{
	struct gpu_m4_drm_vm_leaf *fresh = NULL, *storage;
	u32 *first = NULL, *order = NULL, *map = NULL, *fresh_map = NULL;
	u32 *by_pa, *by_identity, *by_va;
	bool *gone = NULL, *dead = NULL;
	unsigned long a, b;
	u64 total = 0, live, a_bytes, b_bytes;
	u32 n, made = 0, kept = 0, alive = 0, capacity;
	int ret;

	if (plan_index)
		*plan_index = (struct gpu_m4_drm_vm_ledger_index) { 0 };
	if (!active || !plan || !plan->leaf || !plan_index || !retained || plan == active ||
	    !change || !count || count > GPU_M4_DRM_MAX_BINDS ||
	    gpu_m4_drm_vm_ledger_shape(active) ||
	    !gpu_m4_drm_vm_ledger_index_valid(active, index) ||
	    plan->leaf == active->leaf || plan->capacity < active->count)
		return GPU_M4_DRM_VM_PLAN_EXACT;
	a = (unsigned long)active->leaf;
	b = (unsigned long)plan->leaf;
	a_bytes = (u64)active->capacity * sizeof(*active->leaf);
	b_bytes = (u64)plan->capacity * sizeof(*plan->leaf);
	if (a_bytes > ~0UL - a || b_bytes > ~0UL - b || (a < b + b_bytes && b < a + a_bytes))
		return GPU_M4_DRM_VM_PLAN_EXACT;
	n = active->count;
	for (u32 k = 0; k < count; k++) {
		if (!gpu_m4_drm_vm_change_shape(active, &change[k]))
			return GPU_M4_DRM_VM_PLAN_EXACT;
		if (change[k].op.flags & DRM_ASAHI_BIND_UNBIND)
			continue;
		total += change[k].op.range / GPU_M4_DRM_PAGE_SIZE;
		/* Repeated rewrites of one range are left to the exact reference. */
		if (total > 2 * (u64)plan->capacity || total > ~0U / 4)
			return GPU_M4_DRM_VM_PLAN_EXACT;
	}
	ret = -ENOMEM;
	fresh = kvcalloc(total ? total : 1, sizeof(*fresh), GFP_KERNEL);
	first = kvcalloc((u64)count + 1, sizeof(*first), GFP_KERNEL);
	order = kvcalloc(total ? 3 * total : 1, sizeof(*order), GFP_KERNEL);
	fresh_map = kvcalloc(total ? total : 1, sizeof(*fresh_map), GFP_KERNEL);
	dead = kvcalloc(total ? total : 1, sizeof(*dead), GFP_KERNEL);
	gone = kvcalloc(n ? n : 1, sizeof(*gone), GFP_KERNEL);
	map = kvcalloc(n ? n : 1, sizeof(*map), GFP_KERNEL);
	if (!fresh || !first || !order || !fresh_map || !dead || !gone || !map)
		goto out;
	ret = GPU_M4_DRM_VM_PLAN_EXACT;
	for (u32 k = 0; k < count; k++) {
		const struct gpu_m4_drm_vm_change *c = &change[k];

		first[k] = made;
		if (c->op.flags & DRM_ASAHI_BIND_UNBIND)
			continue;
		for (u64 i = 0; i < c->op.range / GPU_M4_DRM_PAGE_SIZE; i++) {
			fresh[made] = gpu_m4_drm_vm_change_leaf(c, i);
			if (gpu_m4_drm_vm_leaf_shape(active, &fresh[made]))
				goto out;
			made++;
		}
	}
	first[count] = made;

	/* Sequential presence: exact unbind, native exclusivity and peak quota. */
	live = n;
	for (u32 k = 0; k < count; k++) {
		const struct drm_asahi_gem_bind_op *op = &change[k].op;
		u64 pages = op->range / GPU_M4_DRM_PAGE_SIZE, present = 0;

		for (u64 i = 0; i < pages; i++)
			present += gpu_m4_drm_vm_plan_present(active, index, change, k,
				op->addr + i * GPU_M4_DRM_PAGE_SIZE);
		if (op->flags & DRM_ASAHI_BIND_UNBIND) {
			if (present != pages)
				goto out;
			live -= pages;
			continue;
		}
		if (change[k].internal_native && present)
			goto out;
		live = live - present + pages;
		if (live > plan->capacity)
			goto out;
	}

	/* Superset alias proof. SINGLE_PAGE repeats one identical page. */
	for (u32 k = 0; k < count; k++) {
		u32 end = first[k + 1];

		if ((change[k].op.flags & DRM_ASAHI_BIND_SINGLE_PAGE) && end > first[k])
			end = first[k] + 1;
		for (u32 i = first[k]; i < end; i++)
			if (gpu_m4_drm_vm_ledger_index_alias(active->leaf, index, &fresh[i]))
				goto out;
	}
	by_pa = order;
	by_identity = order + made;
	by_va = order + 2 * (u64)made;
	for (u32 i = 0; i < made; i++)
		by_pa[i] = by_identity[i] = by_va[i] = i;
	gpu_m4_drm_vm_pos_sort(fresh, by_pa, made, GPU_M4_DRM_VM_KEY_PA);
	for (u32 i = 1; i < made; i++)
		if (gpu_m4_drm_vm_leaf_alias_validate(&fresh[by_pa[i - 1]], &fresh[by_pa[i]]))
			goto out;
	gpu_m4_drm_vm_pos_sort(fresh, by_identity, made, GPU_M4_DRM_VM_KEY_IDENTITY);
	for (u32 i = 1; i < made; i++)
		if (gpu_m4_drm_vm_leaf_alias_validate(&fresh[by_identity[i - 1]],
						      &fresh[by_identity[i]]))
			goto out;

	/* Stable compaction plus appends == source survivors, then batch survivors. */
	for (u32 k = 0; k < count; k++) {
		const struct drm_asahi_gem_bind_op *op = &change[k].op;
		struct gpu_m4_drm_vm_leaf key = { .va = op->addr };

		if (!gpu_m4_drm_vm_change_removes(&change[k]))
			continue;
		for (u32 at = gpu_m4_drm_vm_pos_lower_bound(active->leaf, index->va, n, &key,
			GPU_M4_DRM_VM_KEY_VA);
		     at < n && active->leaf[index->va[at]].va - op->addr < op->range; at++)
			gone[index->va[at]] = true;
		for (u32 j = 0; j < k; j++) {
			const struct drm_asahi_gem_bind_op *prior = &change[j].op;
			u64 low, high;

			if (prior->flags & DRM_ASAHI_BIND_UNBIND)
				continue;
			low = prior->addr > op->addr ? prior->addr : op->addr;
			high = prior->addr + prior->range < op->addr + op->range ?
				prior->addr + prior->range : op->addr + op->range;
			for (u64 va = low; va < high; va += GPU_M4_DRM_PAGE_SIZE)
				dead[first[j] + (va - prior->addr) / GPU_M4_DRM_PAGE_SIZE] = true;
		}
	}
	for (u32 i = 0; i < n; i++)
		if (!gone[i])
			map[i] = kept++;
	for (u32 i = 0; i < made; i++)
		if (!dead[i])
			fresh_map[i] = kept + alive++;
	if ((u64)kept + alive != live)
		goto out;
	ret = gpu_m4_drm_vm_ledger_index_alloc(plan_index, live);
	if (ret)
		goto out;
	storage = plan->leaf;
	capacity = plan->capacity;
	*plan = *active;
	plan->leaf = storage;
	plan->capacity = capacity;
	plan->count = live;
	for (u32 i = 0; i < n; i++)
		if (!gone[i])
			plan->leaf[map[i]] = active->leaf[i];
	for (u32 i = 0; i < made; i++)
		if (!dead[i])
			plan->leaf[fresh_map[i]] = fresh[i];
	gpu_m4_drm_vm_pos_sort(fresh, by_va, made, GPU_M4_DRM_VM_KEY_VA);
	gpu_m4_drm_vm_plan_merge(plan->leaf, index->va, n, gone, map, by_va, made, dead,
				 fresh_map, plan_index->va, GPU_M4_DRM_VM_KEY_VA);
	gpu_m4_drm_vm_plan_merge(plan->leaf, index->pa, n, gone, map, by_pa, made, dead,
				 fresh_map, plan_index->pa, GPU_M4_DRM_VM_KEY_PA);
	gpu_m4_drm_vm_plan_merge(plan->leaf, index->identity, n, gone, map, by_identity, made,
				 dead, fresh_map, plan_index->identity, GPU_M4_DRM_VM_KEY_IDENTITY);
	ret = 0;
	for (u32 i = 1; i < plan->count; i++)
		if (plan->leaf[plan_index->va[i - 1]].va == plan->leaf[plan_index->va[i]].va)
			ret = GPU_M4_DRM_VM_PLAN_EXACT;
	if (ret)
		gpu_m4_drm_vm_ledger_index_free(plan_index);
	else
		*retained = kept;
out:
	kvfree(map);
	kvfree(gone);
	kvfree(dead);
	kvfree(fresh_map);
	kvfree(order);
	kvfree(first);
	kvfree(fresh);
	return ret;
}

/* Exact page oracle for EVERY generated leaf, including leaves a later batch
 * operation supersedes. A SINGLE_PAGE operation names one page repeatedly.
 * Separate from plan_delta() so the oracle runs under its owner's lock.
 */
static inline int gpu_m4_drm_vm_ledger_plan_admit(const struct gpu_m4_drm_vm_ledger *l,
	const struct gpu_m4_drm_vm_change *change, u32 count)
{
	if (!l || !l->validate_page || !change)
		return -EINVAL;
	for (u32 k = 0; k < count; k++) {
		const struct gpu_m4_drm_vm_change *c = &change[k];

		if (c->op.flags & DRM_ASAHI_BIND_UNBIND)
			continue;
		for (u64 i = 0; i < c->op.range / GPU_M4_DRM_PAGE_SIZE; i++) {
			struct gpu_m4_drm_vm_leaf p = gpu_m4_drm_vm_change_leaf(c, i);
			int ret = l->validate_page(l->owner, p.identity, p.object_offset, p.pa);

			if (ret)
				return ret;
			if (c->op.flags & DRM_ASAHI_BIND_SINGLE_PAGE)
				break;
		}
	}
	return 0;
}

/* Delta plan with the exact sequential reference as the sole fallback. */
static inline int gpu_m4_drm_vm_ledger_plan_indexed(const struct gpu_m4_drm_vm_ledger *active,
	const struct gpu_m4_drm_vm_ledger_index *index, struct gpu_m4_drm_vm_ledger *plan,
	struct gpu_m4_drm_vm_ledger_index *plan_index, const struct gpu_m4_drm_vm_change *change,
	u32 count)
{
	u32 retained;
	int ret;

	if (!plan_index)
		return -EINVAL;
	ret = gpu_m4_drm_vm_ledger_plan_delta(active, index, plan, plan_index, change, count,
					      &retained);
	if (ret < 0)
		return ret;
	if (!ret) {
		ret = gpu_m4_drm_vm_ledger_plan_admit(plan, change, count);
		if (!ret)
			return 0;
		gpu_m4_drm_vm_ledger_index_free(plan_index);
	}
	ret = gpu_m4_drm_vm_ledger_plan(active, plan, change, count);
	if (!ret)
		ret = gpu_m4_drm_vm_ledger_index_create(plan, plan_index);
	return ret;
}

/*
 * Reverse object->range removal: drop every leaf naming identity. A subset of
 * a validated ledger needs no new alias proof; positions are re-based in one
 * pass, never re-sorted. The source and its index are unchanged.
 */
static inline int gpu_m4_drm_vm_ledger_filter_identity(const struct gpu_m4_drm_vm_ledger *active,
	const struct gpu_m4_drm_vm_ledger_index *index, const void *identity,
	struct gpu_m4_drm_vm_ledger *plan, struct gpu_m4_drm_vm_ledger_index *plan_index)
{
	struct gpu_m4_drm_vm_leaf *storage;
	u32 *map, *source[3], *target[3];
	u32 kept = 0, capacity;
	int ret;

	if (plan_index)
		*plan_index = (struct gpu_m4_drm_vm_ledger_index) { 0 };
	if (!active || !plan || !plan_index || gpu_m4_drm_vm_ledger_shape(active) ||
	    !gpu_m4_drm_vm_ledger_index_valid(active, index) || !plan->leaf ||
	    plan->capacity < active->count || !gpu_m4_drm_vm_private_storage(active,
		plan->leaf, (u64)plan->capacity * sizeof(*plan->leaf)))
		return -EINVAL;
	map = kvcalloc(active->count ? active->count : 1, sizeof(*map), GFP_KERNEL);
	if (!map)
		return -ENOMEM;
	for (u32 i = 0; i < active->count; i++)
		map[i] = active->leaf[i].identity == identity ? ~0U : kept++;
	ret = gpu_m4_drm_vm_ledger_index_alloc(plan_index, kept);
	if (ret)
		goto out;
	storage = plan->leaf;
	capacity = plan->capacity;
	*plan = *active;
	plan->leaf = storage;
	plan->capacity = capacity;
	plan->count = kept;
	for (u32 i = 0; i < active->count; i++)
		if (map[i] != ~0U)
			plan->leaf[map[i]] = active->leaf[i];
	source[0] = index->va;
	source[1] = index->pa;
	source[2] = index->identity;
	target[0] = plan_index->va;
	target[1] = plan_index->pa;
	target[2] = plan_index->identity;
	for (u32 key = 0; kept && key < 3; key++)
		for (u32 i = 0, at = 0; i < active->count; i++)
			if (map[source[key][i]] != ~0U)
				target[key][at++] = map[source[key][i]];
out:
	kvfree(map);
	return ret;
}

#endif
