// SPDX-License-Identifier: GPL-2.0-only
/* Retained software snapshots only: no UAT publication or render registration. */
#include <linux/err.h>
#include <linux/export.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/refcount.h>
#include <linux/slab.h>

#include "gpu-m4-drm-vm-owner.h"

/*
 * Immutable cache/resv inventory: one entry per native object, each holding
 * one pin reference, with a conservative unioned span/access. Shared by every
 * capture and clone that does not add extras; never written once published.
 */
struct gpu_m4_drm_vm_owner_set {
	refcount_t refs;
	struct gpu_m4_drm_cache_access *access;
	struct gpu_m4_drm_resv_access *resv;
	u32 *order;		/* entry positions sorted by native object identity */
	u32 count, capacity;
};

/*
 * Immutable validated mapping: copied VA leaves, their retained VA/PA/identity
 * indexes, the creator's identity->pin inventory proving every leaf, the
 * frontend VM and the captured generation. Shared by all captures and clones.
 */
struct gpu_m4_drm_vm_owner_map {
	refcount_t refs;
	struct gpu_m4_drm_vm *vm;
	struct gpu_m4_drm_vm_ledger ledger;
	struct gpu_m4_drm_vm_ledger_index index;
	struct gpu_m4_drm_vm_owner_set *pins;
	u64 generation;
};

/*
 * Job-local wrapper. Publication/retirement belong ONLY to this wrapper and
 * its explicit get() holders; immutable blocks are shared, mutable lifecycle
 * never is. The remaining fields are borrowed views of map/set.
 */
struct gpu_m4_drm_vm_owner {
	refcount_t refs;
	struct gpu_m4_drm_vm_owner_map *map;
	struct gpu_m4_drm_vm_owner_set *set;
	struct gpu_m4_drm_vm *vm;
	struct gpu_m4_drm_vm_ledger ledger;
	struct gpu_m4_drm_cache_access *access;
	struct gpu_m4_drm_resv_access *resv;
	u64 generation;
	u32 count, capacity;
	bool published, retired;
};

static void gpu_m4_drm_vm_owner_sift(u32 *a, u32 count, u32 root,
	bool (*less)(const void *context, u32 x, u32 y), const void *context)
{
	while (root < count / 2) {
		u32 child = root * 2 + 1, tmp;

		if (child + 1 < count && less(context, a[child], a[child + 1]))
			child++;
		if (!less(context, a[root], a[child]))
			return;
		tmp = a[root];
		a[root] = a[child];
		a[child] = tmp;
		root = child;
	}
}

/* Bounded, non-recursive heapsort of small scratch position arrays. */
static void gpu_m4_drm_vm_owner_sort(u32 *a, u32 count,
	bool (*less)(const void *context, u32 x, u32 y), const void *context)
{
	if (count < 2)
		return;
	for (u32 parent = count / 2; parent;)
		gpu_m4_drm_vm_owner_sift(a, count, --parent, less, context);
	for (u32 end = count - 1; end; end--) {
		u32 tmp = a[0];

		a[0] = a[end];
		a[end] = tmp;
		gpu_m4_drm_vm_owner_sift(a, end, 0, less, context);
	}
}

static bool gpu_m4_drm_vm_owner_first_less(const void *context, u32 x, u32 y)
{
	const u32 *first = context;

	return first[x] < first[y];
}

static bool gpu_m4_drm_vm_owner_extra_less(const void *context, u32 x, u32 y)
{
	const void *const *identity = context;

	if (identity[x] != identity[y])
		return (unsigned long)identity[x] < (unsigned long)identity[y];
	return x < y;
}

static struct gpu_m4_drm_vm_owner_set *gpu_m4_drm_vm_owner_set_alloc(u32 capacity)
{
	struct gpu_m4_drm_vm_owner_set *set = kzalloc(sizeof(*set), GFP_KERNEL);

	if (!set)
		return NULL;
	refcount_set(&set->refs, 1);
	capacity = max(capacity, 1U);
	set->access = kvcalloc(capacity, sizeof(*set->access), GFP_KERNEL);
	set->resv = kvcalloc(capacity, sizeof(*set->resv), GFP_KERNEL);
	set->order = kvcalloc(capacity, sizeof(*set->order), GFP_KERNEL);
	if (!set->access || !set->resv || !set->order) {
		kvfree(set->access);
		kvfree(set->resv);
		kvfree(set->order);
		kfree(set);
		return NULL;
	}
	set->capacity = capacity;
	return set;
}

static void gpu_m4_drm_vm_owner_set_put(struct gpu_m4_drm_vm_owner_set *set)
{
	if (!set || !refcount_dec_and_test(&set->refs))
		return;
	/* A final pin put may lock its resv. Caller has dropped ALL ww locks. */
	for (u32 i = 0; i < set->count; i++)
		gpu_m4_drm_bo_pin_put(set->access[i].pin);
	kvfree(set->access);
	kvfree(set->resv);
	kvfree(set->order);
	kfree(set);
}

/* Identity -> entry by binary search over the immutable sorted order. */
static u32 gpu_m4_drm_vm_owner_set_find(const struct gpu_m4_drm_vm_owner_set *set,
					const void *identity)
{
	u32 first = 0, end = set->count;

	while (first < end) {
		u32 middle = first + (end - first) / 2;
		const void *object = set->resv[set->order[middle]].object;

		if (object == identity)
			return set->order[middle];
		if ((unsigned long)object < (unsigned long)identity)
			first = middle + 1;
		else
			end = middle;
	}
	return U32_MAX;
}

static void gpu_m4_drm_vm_owner_map_put(struct gpu_m4_drm_vm_owner_map *map)
{
	if (!map || !refcount_dec_and_test(&map->refs))
		return;
	gpu_m4_drm_vm_owner_set_put(map->pins);
	if (map->vm)
		gpu_m4_drm_vm_put(map->vm);
	kvfree(map->ledger.leaf);
	gpu_m4_drm_vm_ledger_index_free(&map->index);
	kfree(map);
}

/* This oracle is backed by snapshot-owned pins, not a mutable provider cookie. */
static int gpu_m4_drm_vm_owner_page_valid(void *cookie, const void *identity,
					u64 offset, u64 pa)
{
	struct gpu_m4_drm_vm_owner_map *map = cookie;
	u32 i = gpu_m4_drm_vm_owner_set_find(map->pins, identity);
	u64 actual;
	int ret;

	if (i == U32_MAX)
		return -ENOENT;
	ret = gpu_m4_drm_bo_pin_page(map->pins->access[i].pin, offset, &actual);
	return ret ? ret : actual == pa ? 0 : -EPERM;
}

/* Every check of one extra access that is independent of earlier joins. */
static int gpu_m4_drm_vm_owner_extra_check(const struct gpu_m4_drm_cache_access *a)
{
	struct gpu_m4_drm_bo_pin *pin = a->pin;
	struct drm_gem_object *object;
	enum gpu_m4_drm_cpu_policy policy;
	u64 length;

	if (IS_ERR_OR_NULL(pin))
		return pin ? PTR_ERR(pin) : -ENOENT;
	object = gpu_m4_drm_bo_pin_object(pin);
	length = gpu_m4_drm_bo_pin_size(pin);
	if (!object)
		return -EINVAL;
	policy = gpu_m4_drm_bo_policy(object);
	if (policy != GPU_M4_DRM_CPU_NC && policy != GPU_M4_DRM_CPU_WRITEBACK)
		return -EOPNOTSUPP;
	if (!gpu_m4_drm_aligned(length) || !gpu_m4_drm_vm_leaf_attrs(a->access) ||
	    !a->size || a->offset >= length || a->size > length - a->offset)
		return -EINVAL;
	return gpu_m4_drm_bo_pin_range(pin, a->offset, a->size);
}

/*
 * Join one checked access into an entry. The SAME actual pin needs only the
 * bounds/access union: its pages were proven when the entry was admitted.
 * Different pin acquisitions for one identity must still name precisely the
 * same backing. Never maintain one pin while hardware uses another.
 */
static int gpu_m4_drm_vm_owner_join(struct gpu_m4_drm_cache_access *a,
				    struct gpu_m4_drm_resv_access *r,
				    const struct gpu_m4_drm_cache_access *x)
{
	u64 end = x->offset + x->size;

	if (x->pin != a->pin) {
		int ret;

		if (gpu_m4_drm_bo_pin_size(x->pin) != gpu_m4_drm_bo_pin_size(a->pin))
			return -EPROTO;
		ret = gpu_m4_drm_bo_pin_same_range(a->pin, x->pin, x->offset, x->size);
		if (ret)
			return ret;
	}
	end = max(end, a->offset + a->size);
	a->offset = min(x->offset, a->offset);
	a->size = end - a->offset;
	a->access |= x->access;
	r->access |= x->access;
	return 0;
}

/*
 * One pass over the retained identity index: each identity group (sorted by
 * object offset) yields exactly one lookup, its bounding span and unioned
 * access, and the snapshot-owned proof of every leaf PA. Entries keep the
 * first-appearance order of the ledger, as the per-leaf construction did.
 */
static struct gpu_m4_drm_vm_owner_set *gpu_m4_drm_vm_owner_set_leaves(
	const struct gpu_m4_drm_vm_owner_map *map, void *cookie,
	struct gpu_m4_drm_bo_pin *(*lookup)(void *cookie, const void *identity))
{
	const struct gpu_m4_drm_vm_leaf *leaf = map->ledger.leaf;
	const u32 *by_identity = map->index.identity;
	struct gpu_m4_drm_vm_owner_set *set;
	u32 *start = NULL, *first = NULL, *rank = NULL;
	u32 n = map->ledger.count, groups = 0, g;
	int ret = -ENOMEM;

	for (u32 i = 0; i < n; i++)
		if (!i || leaf[by_identity[i]].identity != leaf[by_identity[i - 1]].identity)
			groups++;
	set = gpu_m4_drm_vm_owner_set_alloc(groups);
	start = kvcalloc((u64)groups + 1, sizeof(*start), GFP_KERNEL);
	first = kvcalloc(max(groups, 1U), sizeof(*first), GFP_KERNEL);
	rank = kvcalloc(max(groups, 1U), sizeof(*rank), GFP_KERNEL);
	if (!set || !start || !first || !rank)
		goto fail;
	for (u32 i = 0, at = 0; i < n; i++) {
		if (!i || leaf[by_identity[i]].identity != leaf[by_identity[i - 1]].identity) {
			start[at] = i;
			first[at++] = by_identity[i];
		} else if (by_identity[i] < first[at - 1]) {
			first[at - 1] = by_identity[i];
		}
	}
	start[groups] = n;
	for (g = 0; g < groups; g++)
		rank[g] = g;
	gpu_m4_drm_vm_owner_sort(rank, groups, gpu_m4_drm_vm_owner_first_less, first);
	for (u32 r = 0; r < groups; r++) {
		const struct gpu_m4_drm_vm_leaf *head, *last;
		struct gpu_m4_drm_bo_pin *pin;
		struct drm_gem_object *object;
		enum gpu_m4_drm_cpu_policy policy;
		u64 length;

		g = rank[r];
		head = &leaf[by_identity[start[g]]];
		last = &leaf[by_identity[start[g + 1] - 1]];
		pin = lookup(cookie, head->identity);
		if (IS_ERR_OR_NULL(pin)) {
			ret = pin ? PTR_ERR(pin) : -ENOENT;
			goto fail;
		}
		object = gpu_m4_drm_bo_pin_object(pin);
		if (object != head->identity || gpu_m4_drm_bo_policy(object) != head->cpu_policy) {
			ret = -EPROTO;
			goto fail;
		}
		policy = gpu_m4_drm_bo_policy(object);
		length = gpu_m4_drm_bo_pin_size(pin);
		if (policy != GPU_M4_DRM_CPU_NC && policy != GPU_M4_DRM_CPU_WRITEBACK) {
			ret = -EOPNOTSUPP;
			goto fail;
		}
		if (!gpu_m4_drm_aligned(length)) {
			ret = -EINVAL;
			goto fail;
		}
		gpu_m4_drm_bo_pin_get(pin);
		set->access[r] = (struct gpu_m4_drm_cache_access) {
			.pin = pin, .offset = head->object_offset,
			.size = last->object_offset + GPU_M4_DRM_PAGE_SIZE - head->object_offset,
		};
		set->resv[r] = (struct gpu_m4_drm_resv_access) { .object = object };
		set->count++;
		for (u32 i = start[g]; i < start[g + 1]; i++) {
			const struct gpu_m4_drm_vm_leaf *p = &leaf[by_identity[i]];
			u64 actual;

			if (!gpu_m4_drm_vm_leaf_attrs(p->access) || p->object_offset >= length ||
			    GPU_M4_DRM_PAGE_SIZE > length - p->object_offset) {
				ret = -EINVAL;
				goto fail;
			}
			ret = gpu_m4_drm_bo_pin_page(pin, p->object_offset, &actual);
			if (!ret && actual != p->pa)
				ret = -EPERM;
			if (ret)
				goto fail;
			set->access[r].access |= p->access;
			set->resv[r].access |= p->access;
		}
		set->order[g] = r;
	}
	kvfree(rank);
	kvfree(first);
	kvfree(start);
	return set;
fail:
	kvfree(rank);
	kvfree(first);
	kvfree(start);
	gpu_m4_drm_vm_owner_set_put(set);
	return ERR_PTR(ret);
}

/*
 * New immutable set = source inventory (its pins re-referenced, its timestamp-
 * only extras preserved) plus the extras merged by identity. Existing entries
 * keep their position; new identities append in first-appearance order. The
 * source set is never modified. O((B + E) log E), never per-leaf.
 */
static struct gpu_m4_drm_vm_owner_set *gpu_m4_drm_vm_owner_set_extend(
	const struct gpu_m4_drm_vm_owner_set *source,
	const struct gpu_m4_drm_cache_access *extra, u32 extra_count)
{
	struct gpu_m4_drm_vm_owner_set *set;
	const void **identity = NULL;
	u32 *sorted = NULL, *start = NULL, *head = NULL, *fresh = NULL, *entry = NULL;
	u32 groups = 0, created = 0, base = source->count, at = 0, i = 0;
	int ret;

	for (u32 x = 0; x < extra_count; x++) {
		ret = gpu_m4_drm_vm_owner_extra_check(&extra[x]);
		if (ret)
			return ERR_PTR(ret);
	}
	ret = -ENOMEM;
	set = gpu_m4_drm_vm_owner_set_alloc(base + extra_count);
	identity = kvcalloc(extra_count, sizeof(*identity), GFP_KERNEL);
	sorted = kvcalloc(extra_count, sizeof(*sorted), GFP_KERNEL);
	start = kvcalloc((u64)extra_count + 1, sizeof(*start), GFP_KERNEL);
	head = kvcalloc(extra_count, sizeof(*head), GFP_KERNEL);
	fresh = kvcalloc(extra_count, sizeof(*fresh), GFP_KERNEL);
	entry = kvcalloc(extra_count, sizeof(*entry), GFP_KERNEL);
	if (!set || !identity || !sorted || !start || !head || !fresh || !entry)
		goto fail;
	for (u32 x = 0; x < base; x++) {
		set->access[x] = source->access[x];
		set->resv[x] = source->resv[x];
		gpu_m4_drm_bo_pin_get(set->access[x].pin);
		set->count++;
	}
	for (u32 x = 0; x < extra_count; x++) {
		identity[x] = gpu_m4_drm_bo_pin_object(extra[x].pin);
		sorted[x] = x;
	}
	gpu_m4_drm_vm_owner_sort(sorted, extra_count, gpu_m4_drm_vm_owner_extra_less, identity);
	for (u32 x = 0; x < extra_count; x++)
		if (!x || identity[sorted[x]] != identity[sorted[x - 1]])
			start[groups++] = x;
	start[groups] = extra_count;
	for (u32 g = 0; g < groups; g++) {
		head[g] = sorted[start[g]];
		entry[g] = gpu_m4_drm_vm_owner_set_find(source, identity[head[g]]);
		if (entry[g] == U32_MAX)
			fresh[created++] = g;
	}
	gpu_m4_drm_vm_owner_sort(fresh, created, gpu_m4_drm_vm_owner_first_less, head);
	for (u32 r = 0; r < created; r++) {
		const struct gpu_m4_drm_cache_access *x = &extra[head[fresh[r]]];
		u32 e = set->count;

		gpu_m4_drm_bo_pin_get(x->pin);
		set->access[e] = *x;
		set->resv[e] = (struct gpu_m4_drm_resv_access) {
			.object = gpu_m4_drm_bo_pin_object(x->pin), .access = x->access,
		};
		set->count++;
		entry[fresh[r]] = e;
	}
	for (u32 g = 0; g < groups; g++)
		for (u32 x = start[g] + (entry[g] >= base); x < start[g + 1]; x++) {
			ret = gpu_m4_drm_vm_owner_join(&set->access[entry[g]], &set->resv[entry[g]],
						       &extra[sorted[x]]);
			if (ret)
				goto fail;
		}
	/* Identity order: merge the source order with new groups (already sorted). */
	for (u32 g = 0; g < groups; g++) {
		if (entry[g] < base)
			continue;
		while (i < base && (unsigned long)source->resv[source->order[i]].object <
		       (unsigned long)identity[head[g]])
			set->order[at++] = source->order[i++];
		set->order[at++] = entry[g];
	}
	while (i < base)
		set->order[at++] = source->order[i++];
	ret = 0;
fail:
	kvfree(entry);
	kvfree(fresh);
	kvfree(head);
	kvfree(start);
	kvfree(sorted);
	kvfree(identity);
	if (!ret)
		return set;
	gpu_m4_drm_vm_owner_set_put(set);
	return ERR_PTR(ret);
}

static void gpu_m4_drm_vm_owner_views(struct gpu_m4_drm_vm_owner *owner)
{
	owner->vm = owner->map->vm;
	owner->ledger = owner->map->ledger;
	owner->generation = owner->map->generation;
	owner->access = owner->set->access;
	owner->resv = owner->set->resv;
	owner->count = owner->set->count;
	owner->capacity = owner->set->capacity;
}

struct gpu_m4_drm_vm_owner *gpu_m4_drm_vm_owner_create_indexed(
	struct gpu_m4_drm_vm *vm, struct mutex *vm_lock,
	const struct gpu_m4_drm_vm_ledger *active,
	const struct gpu_m4_drm_vm_ledger_index *index, u64 generation,
	void *cookie,
	struct gpu_m4_drm_bo_pin *(*lookup)(void *cookie, const void *identity),
	const struct gpu_m4_drm_cache_access *extra, u32 extra_count)
{
	struct gpu_m4_drm_vm_ledger_index proven = { 0 };
	struct gpu_m4_drm_vm_owner *owner = NULL;
	struct gpu_m4_drm_vm_owner_map *map = NULL;
	struct gpu_m4_drm_vm_owner_set *set;
	int ret;

	if (!vm || !vm_lock || !active || !lookup || (extra_count && !extra))
		return ERR_PTR(-EINVAL);
	lockdep_assert_held(vm_lock);
	/* A retained index is its source owner's validated authority. Without
	 * one, prove the source now with its own oracle, as ledger_validate().
	 */
	if (index) {
		ret = gpu_m4_drm_vm_ledger_shape(active);
		if (!ret && !gpu_m4_drm_vm_ledger_index_valid(active, index))
			ret = -EINVAL;
	} else {
		ret = gpu_m4_drm_vm_ledger_index_create(active, &proven);
	}
	if (ret)
		return ERR_PTR(ret);
	if (extra_count > U32_MAX - active->count) {
		ret = -E2BIG;
		goto fail;
	}
	ret = -ENOMEM;
	owner = kzalloc(sizeof(*owner), GFP_KERNEL);
	if (!owner)
		goto fail;
	refcount_set(&owner->refs, 1);
	map = kzalloc(sizeof(*map), GFP_KERNEL);
	if (!map)
		goto fail;
	refcount_set(&map->refs, 1);
	map->ledger = *active;
	map->ledger.capacity = max(active->count, 1U);
	map->ledger.leaf = kvcalloc(map->ledger.capacity, sizeof(*active->leaf), GFP_KERNEL);
	if (!map->ledger.leaf)
		goto fail;
	memcpy(map->ledger.leaf, active->leaf, array_size(active->count, sizeof(*active->leaf)));
	if (index) {
		ret = gpu_m4_drm_vm_ledger_index_copy(&map->index, index);
		if (ret)
			goto fail;
	} else {
		map->index = proven;
		proven = (struct gpu_m4_drm_vm_ledger_index) { 0 };
	}
	map->ledger.owner = map;
	map->ledger.validate_page = gpu_m4_drm_vm_owner_page_valid;
	map->generation = generation;
	map->vm = vm;
	gpu_m4_drm_vm_get(vm);
	map->pins = gpu_m4_drm_vm_owner_set_leaves(map, cookie, lookup);
	if (IS_ERR(map->pins)) {
		ret = PTR_ERR(map->pins);
		map->pins = NULL;
		goto fail;
	}
	/* The map's oracle inventory is the creator's complete set, as before:
	 * leaf identities resolve to the same pins with or without its extras.
	 */
	if (extra_count) {
		set = gpu_m4_drm_vm_owner_set_extend(map->pins, extra, extra_count);
		if (IS_ERR(set)) {
			ret = PTR_ERR(set);
			goto fail;
		}
		gpu_m4_drm_vm_owner_set_put(map->pins);
		map->pins = set;
	}
	set = map->pins;
	refcount_inc(&set->refs);
	owner->map = map;
	owner->set = set;
	gpu_m4_drm_vm_owner_views(owner);
	return owner;
fail:
	gpu_m4_drm_vm_ledger_index_free(&proven);
	gpu_m4_drm_vm_owner_map_put(map);
	kfree(owner);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_owner_create_indexed);

struct gpu_m4_drm_vm_owner *gpu_m4_drm_vm_owner_create(
	struct gpu_m4_drm_vm *vm, struct mutex *vm_lock,
	const struct gpu_m4_drm_vm_ledger *active, u64 generation,
	void *cookie,
	struct gpu_m4_drm_bo_pin *(*lookup)(void *cookie, const void *identity),
	const struct gpu_m4_drm_cache_access *extra, u32 extra_count)
{
	return gpu_m4_drm_vm_owner_create_indexed(vm, vm_lock, active, NULL, generation,
						  cookie, lookup, extra, extra_count);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_owner_create);

struct gpu_m4_drm_vm_owner *gpu_m4_drm_vm_owner_clone_with_extras(
	const struct gpu_m4_drm_vm_owner *source,
	const struct gpu_m4_drm_cache_access *extra, u32 extra_count)
{
	struct gpu_m4_drm_vm_owner *owner;
	struct gpu_m4_drm_vm_owner_set *set;

	if (!source || source->published || source->retired || (extra_count && !extra))
		return ERR_PTR(-EINVAL);
	if (extra_count > U32_MAX - source->count)
		return ERR_PTR(-E2BIG);
	owner = kzalloc(sizeof(*owner), GFP_KERNEL);
	if (!owner)
		return ERR_PTR(-ENOMEM);
	refcount_set(&owner->refs, 1);
	/* Immutable VA leaves, indexes and pin inventory are shared; a zero-
	 * extra clone is only this job-local wrapper. Extras get a new set.
	 */
	if (extra_count) {
		set = gpu_m4_drm_vm_owner_set_extend(source->set, extra, extra_count);
		if (IS_ERR(set)) {
			kfree(owner);
			return ERR_PTR(PTR_ERR(set));
		}
	} else {
		refcount_inc(&source->set->refs);
		set = source->set;
	}
	refcount_inc(&source->map->refs);
	owner->map = source->map;
	owner->set = set;
	gpu_m4_drm_vm_owner_views(owner);
	return owner;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_owner_clone_with_extras);

u64 gpu_m4_drm_vm_owner_bytes(const struct gpu_m4_drm_vm_owner *owner, bool map)
{
	u64 bytes;

	if (!owner)
		return 0;
	bytes = sizeof(*owner) + sizeof(*owner->set) + (u64)owner->set->capacity *
		(sizeof(*owner->set->access) + sizeof(*owner->set->resv) + sizeof(*owner->set->order));
	if (map)
		bytes += sizeof(*owner->map) +
			(u64)owner->map->ledger.capacity * sizeof(*owner->map->ledger.leaf) +
			(u64)owner->map->index.count * 3 * sizeof(*owner->map->index.va);
	return bytes;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_owner_bytes);

u64 gpu_m4_drm_vm_owner_generation(const struct gpu_m4_drm_vm_owner *owner)
{
	return owner ? owner->generation : 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_owner_generation);

int gpu_m4_drm_vm_owner_get(struct gpu_m4_drm_vm_owner *owner)
{
	if (!owner)
		return -EINVAL;
	refcount_inc(&owner->refs);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_owner_get);

int gpu_m4_drm_vm_owner_get_unpublished(struct gpu_m4_drm_vm_owner *owner)
{
	if (!owner)
		return -EINVAL;
	if (owner->published || owner->retired)
		return -EBUSY;
	return gpu_m4_drm_vm_owner_get(owner);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_owner_get_unpublished);

u32 gpu_m4_drm_vm_owner_leaf_count(const struct gpu_m4_drm_vm_owner *owner)
{
	return owner ? owner->ledger.count : 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_owner_leaf_count);

int gpu_m4_drm_vm_owner_leaf(const struct gpu_m4_drm_vm_owner *owner,
			   u32 index, struct gpu_m4_drm_vm_leaf *leaf)
{
	if (!owner || !leaf)
		return -EINVAL;
	if (index >= owner->ledger.count)
		return -ENOENT;
	*leaf = owner->ledger.leaf[index];
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_owner_leaf);

struct gpu_m4_drm_vm *gpu_m4_drm_vm_owner_vm(const struct gpu_m4_drm_vm_owner *owner)
{
	return owner ? owner->vm : NULL;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_owner_vm);

/* O(log N) through the retained VA index, never a whole-ledger scan. */
int gpu_m4_drm_vm_owner_page(const struct gpu_m4_drm_vm_owner *owner,
			   u64 va, u32 access, struct gpu_m4_drm_vm_leaf *leaf)
{
	u64 page = va & ~(GPU_M4_DRM_PAGE_SIZE - 1);
	const struct gpu_m4_drm_vm_leaf *p;
	u32 at;

	if (!owner || !leaf || !gpu_m4_drm_vm_leaf_attrs(access))
		return -EINVAL;
	at = gpu_m4_drm_vm_ledger_index_find(owner->ledger.leaf, &owner->map->index, page);
	if (at == U32_MAX)
		return -ENOENT;
	p = &owner->ledger.leaf[at];
	if ((p->access & access) != access)
		return -EACCES;
	*leaf = *p;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_owner_page);

/* One lookup, then consecutive VA-sorted entries: exactly the per-page result. */
int gpu_m4_drm_vm_owner_range(const struct gpu_m4_drm_vm_owner *owner,
			    u64 va, u64 size, u32 access)
{
	const struct gpu_m4_drm_vm_ledger_index *index;
	const struct gpu_m4_drm_vm_leaf *leaf;
	struct gpu_m4_drm_vm_leaf key;
	u64 end, page;
	u32 at;

	if (!owner || !gpu_m4_drm_vm_leaf_attrs(access) ||
	    !gpu_m4_drm_range(va, size, owner->ledger.low, owner->ledger.high))
		return -EINVAL;
	index = &owner->map->index;
	leaf = owner->ledger.leaf;
	end = va + size;
	page = va & ~(GPU_M4_DRM_PAGE_SIZE - 1);
	key = (struct gpu_m4_drm_vm_leaf) { .va = page };
	at = gpu_m4_drm_vm_pos_lower_bound(leaf, index->va, index->count, &key,
					   GPU_M4_DRM_VM_KEY_VA);
	for (; page < end; page += GPU_M4_DRM_PAGE_SIZE, at++) {
		if (at >= index->count || leaf[index->va[at]].va != page)
			return -ENOENT;
		if ((leaf[index->va[at]].access & access) != access)
			return -EACCES;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_owner_range);

const struct gpu_m4_drm_cache_access *gpu_m4_drm_vm_owner_cache_access(
	const struct gpu_m4_drm_vm_owner *owner, u32 *count)
{
	if (!count)
		return NULL;
	*count = owner ? owner->count : 0;
	return owner ? owner->access : NULL;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_owner_cache_access);

const struct gpu_m4_drm_resv_access *gpu_m4_drm_vm_owner_resv_access(
	const struct gpu_m4_drm_vm_owner *owner, u32 *count)
{
	if (!count)
		return NULL;
	*count = owner ? owner->count : 0;
	return owner ? owner->resv : NULL;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_owner_resv_access);

u32 gpu_m4_drm_vm_owner_find(const struct gpu_m4_drm_vm_owner *owner, const void *identity)
{
	u32 row;

	if (!owner || !owner->set || !identity)
		return U32_MAX;
	row = gpu_m4_drm_vm_owner_set_find(owner->set, identity);
	return row < owner->count ? row : U32_MAX;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_owner_find);

int gpu_m4_drm_vm_owner_publish(struct gpu_m4_drm_vm_owner *owner)
{
	if (!owner || owner->published || owner->retired)
		return -EINVAL;
	owner->published = true;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_owner_publish);

int gpu_m4_drm_vm_owner_retire(struct gpu_m4_drm_vm_owner *owner)
{
	if (!owner || !owner->published || owner->retired)
		return -EINVAL;
	owner->retired = true;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_owner_retire);

int gpu_m4_drm_vm_owner_release(struct gpu_m4_drm_vm_owner *owner)
{
	if (!owner)
		return 0;
	if (owner->published && !owner->retired)
		return -EBUSY;
	if (!refcount_dec_and_test(&owner->refs))
		return 0;
	/* Shared blocks drop their pins/VM only with their last wrapper. A final
	 * pin put may lock its resv: the caller has dropped ALL ww locks.
	 */
	gpu_m4_drm_vm_owner_set_put(owner->set);
	gpu_m4_drm_vm_owner_map_put(owner->map);
	kfree(owner);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_owner_release);

MODULE_DESCRIPTION("Retained M4 whole-VM BO and permission snapshots");
MODULE_LICENSE("GPL");
