/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_READY_ARENA_H
#define APPLE_GPU_M4_READY_ARENA_H

#include <linux/build_bug.h>
#include <drm/drm_gem_shmem_helper.h>
#include "gpu-m4-drm-job.h"
#include "gpu-m4-drm-memory.h"
#include "gpu-m4-drm-validation.h"
#include "gpu-m4-user-budget.h"

/* Finite session capacity also covers retained native allocator owners, not
 * only one M=64 all-render batch plus the last compute/TA/FS.
 * These are session baselines, not a second job/cell-retirement lifecycle.
 */
#define M4_ARENA_MAX_COMMANDS GPU_M4_DRM_JOB_MAX_COMMANDS
#define M4_ARENA_CELLS M4_ORDINARY_WORK_CELLS
static_assert(M4_ARENA_CELLS >= 2U * M4_ARENA_MAX_COMMANDS + 3U);
#define M4_ARENA_LOWER_ATTRS 0x00c0000000000c8bULL

struct m4_command_cell {
	struct m4_command_arena *arena;
	struct drm_gem_object *gem;
	struct gpu_m4_drm_bo_pin *pin;
	struct iosys_map map;
	struct gpu_m4_drm_upper_backing backing;
	struct gpu_m4_drm_upper_lease *upper;
	u64 pa, va, fw_va;
	u32 baseline_refs;
	bool mapped, initialized;
};

struct m4_command_arena {
	struct m4_gpu_hello *hello;
	struct gpu_m4_drm *frontend;
	struct m4_command_cell cell[M4_ARENA_CELLS];
	/* Immutable before/final images retain the complete bootstrap prefix.
	 * Live upper allocation may extend only zero slots AFTER upper_end.
	 */
	u8 before[2][GPU_M4_DRM_PAGE_SIZE], sealed[2][GPU_M4_DRM_PAGE_SIZE];
	u64 upper_start, upper_end;
	u32 allocated, lower_first;
	/* Immutable inventory proof over the sealed/imported arena: before/sealed
	 * images, per-cell backing descriptors, every cell PA against the READY
	 * physical exclusions and pairwise PA/GEM/CPU uniqueness. Keyed to the
	 * exclusion set it covered (READY upper_extra only grows). Cell ownership,
	 * lease borrowers, live table paths/guards and pin identity are NEVER part
	 * of it; those are rechecked at every use.
	 */
	u32 proved_extra;
	int error;
	bool prepared, imported, proved;
	/* Cell upper class, fixed at prepare (cell_cached experiment). */
	enum gpu_m4_drm_upper_class cell_class;
};

/* Read once at arena preparation: map the retained command cells ASC-cached
 * (AttrIndex 0, FW+GPU RW), as native maps its command images and
 * microsequences; the firmware cleans and invalidates their synchronized
 * spans at admission and completion. Cells live until ASC stop, so no
 * cached-mapping teardown is ever needed. 0 restores index 2 (boot only).
 */
static bool cell_cached = true;
module_param(cell_cached, bool, 0444);
MODULE_PARM_DESC(cell_cached, "Map command cells ASC-cached, as native (boot only, default 1)");

static inline u64 m4_arena_cell_attrs(const struct m4_command_arena *a)
{
	return gpu_m4_drm_upper_class_attrs(a->cell_class);
}

static bool m4_ready_page_overlap(u64 pa, u64 base, u64 size)
{
	return size && (base > U64_MAX - size ||
		(pa < base + size && base < pa + GPU_M4_DRM_PAGE_SIZE));
}

/* Independent physical exclusions shared by pre-RUN native arena admission
 * and the ordinary captured provider. An external role may exempt ONLY its
 * exact independently retained DMA allocation. Never read a data payload or
 * convert an observed descriptor PA into a CPU pointer here.
 */
static int m4_ready_page_allowed_locked(struct m4_gpu_hello *h, u64 pa,
	const struct m4_ready_buffer *allowed)
{
	struct m4_gpu_ready *r = h->ready;
	struct m4_gpu_exec_memory *e = r->execution;
	struct platform_device *pdev = to_platform_device(h->gpu->dev);
	struct resource *res;

	lockdep_assert_held(&h->gpu->lock);
	if (h->gpu->hello != h || !pa || !gpu_m4_drm_aligned(pa) ||
	    pa >= BIT_ULL(42) || !pfn_valid(PHYS_PFN(pa)) ||
	    PageReserved(pfn_to_page(PHYS_PFN(pa))) ||
	    r->upper_extra_count > ARRAY_SIZE(r->upper_extra) ||
	    region_intersects(pa, GPU_M4_DRM_PAGE_SIZE, IORESOURCE_SYSTEM_RAM,
			      IORES_DESC_NONE) != REGION_INTERSECTS)
		return -EPERM;
	for (u32 i = 0; i < ARRAY_SIZE(h->region); i++)
		if (m4_ready_page_overlap(pa, h->region[i].start, resource_size(&h->region[i])))
			return -EPERM;
	for (u32 i = 0; (res = platform_get_resource(pdev, IORESOURCE_MEM, i)); i++)
		if (m4_ready_page_overlap(pa, res->start, resource_size(res)))
			return -EPERM;
	if (h->lower && m4_ready_page_overlap(pa, page_to_phys(h->lower), PAGE_SIZE))
		return -EPERM;
	for (u32 i = 0; i < ARRAY_SIZE(r->table); i++)
		if (r->table[i] && m4_ready_page_overlap(pa, page_to_phys(r->table[i]), PAGE_SIZE))
			return -EPERM;
	for (u32 i = 0; i < r->upper_extra_count; i++)
		if (m4_ready_page_overlap(pa, r->upper_extra[i].pa, PAGE_SIZE))
			return -EPERM;
	for (u32 i = 0; i < ARRAY_SIZE(r->buffer); i++)
		if (&r->buffer[i] != allowed &&
		    m4_ready_page_overlap(pa, r->buffer[i].pa, r->buffer[i].size))
			return -EPERM;
	if (!e)
		return 0;
	if (m4_ready_page_overlap(pa, e->context.pa, e->context.size) ||
	    m4_ready_page_overlap(pa, e->zero_stamp.pa, e->zero_stamp.size) ||
	    m4_ready_page_overlap(pa, e->notification.pa, e->notification.size))
		return -EPERM;
	for (u32 engine = 0; engine < ARRAY_SIZE(e->queue); engine++)
		for (u32 i = 0; i < ARRAY_SIZE(e->queue[engine]); i++)
			if (m4_ready_page_overlap(pa, e->queue[engine][i].pa, e->queue[engine][i].size))
				return -EPERM;
	for (u32 i = 0; i < ARRAY_SIZE(e->buffer); i++)
		if (&e->buffer[i] != allowed &&
		    m4_ready_page_overlap(pa, e->buffer[i].pa, e->buffer[i].size))
			return -EPERM;
	if (m4_exec_pool_overlap(e, pa, GPU_M4_DRM_PAGE_SIZE, allowed))
		return -EPERM;
	for (u32 i = 0; i < ARRAY_SIZE(e->page); i++)
		if (e->page[i] && m4_ready_page_overlap(pa, page_to_phys(e->page[i]), PAGE_SIZE))
			return -EPERM;
	return 0;
}

static bool m4_arena_proved_locked(const struct m4_command_arena *a)
{
	return a->proved && a->proved_extra == a->hello->ready->upper_extra_count;
}

/* Selected cell identity: session/arena/error state, the retained native GEM,
 * pin, NC view and policy, and the pin's actual page. The cell PA's physical
 * exclusions are part of the inventory proof once it exists.
 */
static int m4_arena_cell_valid_locked(struct m4_command_cell *c)
{
	struct m4_command_arena *a = c->arena;
	struct m4_gpu_hello *h = a->hello;
	u64 pa;

	lockdep_assert_held(&h->gpu->lock);
	if (h->gpu->hello != h || h->ready->arena != a || a->error ||
	    h->gpu->error || h->error || h->ready->error ||
	    !a->frontend || !c->baseline_refs || !c->gem || !c->pin ||
	    !c->mapped || !c->initialized || iosys_map_is_null(&c->map) || c->map.is_iomem ||
	    ((unsigned long)c->map.vaddr & (GPU_M4_DRM_PAGE_SIZE - 1)) ||
	    !gpu_m4_drm_bo_is_native(c->gem) ||
	    !gpu_m4_drm_bo_belongs_to_frontend(c->gem, a->frontend) ||
	    c->gem->size != GPU_M4_DRM_PAGE_SIZE ||
	    gpu_m4_drm_bo_policy(c->gem) != GPU_M4_DRM_CPU_NC ||
	    gpu_m4_drm_bo_pin_object(c->pin) != c->gem ||
	    gpu_m4_drm_bo_pin_size(c->pin) != GPU_M4_DRM_PAGE_SIZE ||
	    gpu_m4_drm_bo_pin_page(c->pin, 0, &pa) || pa != c->pa)
		return -EPERM;
	return m4_arena_proved_locked(a) ? 0 : m4_ready_page_allowed_locked(h, pa, NULL);
}

static int m4_arena_backing_get(void *owner)
{
	struct m4_command_cell *c = owner;
	int ret = m4_arena_cell_valid_locked(c);

	if (ret)
		return ret;
	if (c->baseline_refs == U32_MAX)
		return -EOVERFLOW;
	c->baseline_refs++;
	return 0;
}

static void m4_arena_backing_put(void *owner)
{
	struct m4_command_cell *c = owner;
	struct mutex *lock = &c->arena->hello->gpu->lock;

	/* Upper rollback/final release runs outside provider/reservation locks.
	 * The READY native GEM/pin/NC view baseline is NEVER dropped by this put.
	 */
	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	if (WARN_ON(c->baseline_refs <= 1))
		goto out;
	c->baseline_refs--;
out:
	mutex_unlock(lock);
}

static int m4_arena_backing_page(void *owner, u64 offset, u64 *pa)
{
	struct m4_command_cell *c = owner;
	int ret = m4_arena_cell_valid_locked(c);

	if (ret)
		return ret;
	if (offset || !pa)
		return -EINVAL;
	*pa = c->pa;
	return 0;
}

static int m4_arena_backing_check(void *owner, const void *identity, u64 offset, u64 pa)
{
	struct m4_command_cell *c = owner;
	int ret = m4_arena_cell_valid_locked(c);

	return ret ?: (identity == c->gem && !offset && pa == c->pa ? 0 : -EPERM);
}

static int m4_arena_backing_ready(void *owner, u64 offset, u64 size,
	enum gpu_m4_drm_upper_class class)
{
	struct m4_command_cell *c = owner;
	int ret = m4_arena_cell_valid_locked(c);

	if (offset || size != GPU_M4_DRM_PAGE_SIZE || class != c->arena->cell_class)
		return -EINVAL;

	/* Already-visible pre-RUN aliases are referenced, not republished. Native
	 * NC policy is NOT current-job cache/dependency/lane authority. The actual
	 * runtime must prove that separately before writing command contents.
	 */
	return ret ?: -EOPNOTSUPP;
}

static bool m4_arena_backing_equal(const struct m4_command_cell *c,
	const struct gpu_m4_drm_upper_backing *b)
{
	return b && b->owner == c && b->identity == c->gem && !b->offset &&
		b->size == GPU_M4_DRM_PAGE_SIZE && b->policy == GPU_M4_DRM_CPU_NC &&
		b->get_locked == m4_arena_backing_get && b->put == m4_arena_backing_put &&
		b->page_locked == m4_arena_backing_page && b->check_locked == m4_arena_backing_check &&
		b->ready_locked == m4_arena_backing_ready;
}

static bool m4_arena_parent_image(const void *table, u32 slots, u32 index, u64 value)
{
	for (u32 i = 0; i < slots; i++)
		if (get_unaligned_le64(table + i * 8) != (i == index ? value : 0))
			return false;
	return true;
}

/* Root holds the stopped HELLO claim while this function drops GPU lock for
 * frontend/reservation factories. Partial owners are rooted in r->arena BEFORE
 * allocation and remain there on every error, including unpublished failures.
 */
static int m4_arena_prepare(struct m4_gpu_hello *h, struct gpu_m4_drm *frontend)
{
	struct mutex *lock = &h->gpu->lock;
	struct m4_gpu_ready *r = h->ready;
	struct m4_command_arena *a;
	u64 upper_end;
	u32 first;
	int ret = 0;

	lockdep_assert_not_held(lock);
	if (!frontend)
		return -EINVAL;
	a = kvzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;
	mutex_lock(lock);
	if (!runtime_enable || h->gpu->hello != h || !r->prepared || r->arena ||
	    r->published || h->gpu->firmware_start_attempted || !h->gpu->stopped_verified ||
	    h->gpu->error || h->error || r->error) {
		ret = -EPERM;
		mutex_unlock(lock);
		kvfree(a); /* No allocation/backing was yet attached or exposed. */
		return ret;
	}
	a->hello = h;
	a->frontend = frontend; /* Same permanently retained pre-RUN frontend. */
	a->cell_class = READ_ONCE(cell_cached) ? GPU_M4_DRM_UPPER_SHARED_CACHED :
		GPU_M4_DRM_UPPER_SHARED;
	a->upper_start = r->next_va;
	memcpy(a->before[0], r->expected[M4_LOWER_LEAF], GPU_M4_DRM_PAGE_SIZE);
	memcpy(a->before[1], r->expected[M4_UPPER_LEAF], GPU_M4_DRM_PAGE_SIZE);
	r->arena = a;
	mutex_unlock(lock);
	for (u32 i = 0; i < M4_ARENA_CELLS; i++) {
		struct m4_command_cell *c = &a->cell[i];
		struct drm_gem_object *gem;

		c->arena = a;
		gem = gpu_m4_drm_bo_create_native(frontend, GPU_M4_DRM_PAGE_SIZE,
			GPU_M4_DRM_CPU_NC, NULL);
		if (IS_ERR(gem)) {
			ret = PTR_ERR(gem);
			break;
		}
		/* Exclusive stopped staging owns these fields. No health or producer
		 * accepts the arena until prepared=true under the GPU mutex.
		 */
		c->gem = gem;
		a->allocated++;
		if (!gpu_m4_drm_bo_is_native(gem) || gem->size != GPU_M4_DRM_PAGE_SIZE ||
		    !gpu_m4_drm_bo_belongs_to_frontend(gem, frontend) ||
		    gpu_m4_drm_bo_policy(gem) != GPU_M4_DRM_CPU_NC) {
			ret = -EPERM;
			break;
		}
		ret = dma_resv_lock_interruptible(gem->resv, NULL);
		if (ret)
			break;
		c->pin = gpu_m4_drm_bo_pin_locked(gem);
		if (IS_ERR_OR_NULL(c->pin)) {
			ret = c->pin ? PTR_ERR(c->pin) : -ENOMEM;
			c->pin = NULL;
		} else {
			/* pin_locked performed initial NC preparation BEFORE this sole
			 * NC shmem view. No direct-map/WB payload access follows it.
			 */
			ret = drm_gem_shmem_vmap_locked(to_drm_gem_shmem_obj(gem), &c->map);
			if (!ret)
				c->mapped = true;
		}
		dma_resv_unlock(gem->resv);
		if (ret)
			break;
		if (iosys_map_is_null(&c->map) || c->map.is_iomem ||
		    ((unsigned long)c->map.vaddr & (GPU_M4_DRM_PAGE_SIZE - 1))) {
			ret = -EPERM;
			break;
		}
		memset(c->map.vaddr, 0, GPU_M4_DRM_PAGE_SIZE);
		dma_wmb();
		c->initialized = true;
		c->baseline_refs = 1;
		ret = gpu_m4_drm_bo_pin_page(c->pin, 0, &c->pa);
		if (ret)
			break;
		c->backing = (struct gpu_m4_drm_upper_backing) {
			.owner = c, .identity = gem, .size = GPU_M4_DRM_PAGE_SIZE,
			.policy = GPU_M4_DRM_CPU_NC, .get_locked = m4_arena_backing_get,
			.put = m4_arena_backing_put, .page_locked = m4_arena_backing_page,
			.check_locked = m4_arena_backing_check, .ready_locked = m4_arena_backing_ready,
		};
		mutex_lock(lock);
		ret = m4_arena_cell_valid_locked(c);
		for (u32 j = 0; !ret && j < i; j++)
			if (a->cell[j].pa == c->pa || a->cell[j].gem == c->gem ||
			    a->cell[j].map.vaddr == c->map.vaddr)
				ret = -EEXIST;
		mutex_unlock(lock);
		if (ret)
			break;
	}
	mutex_lock(lock);
	if (ret)
		goto fault;
	if (h->gpu->hello != h || h->ready != r || r->arena != a ||
	    r->published || h->gpu->firmware_start_attempted || !h->gpu->stopped_verified ||
	    h->gpu->error || h->error || r->error || r->next_va != a->upper_start ||
	    memcmp(a->before[0], r->expected[M4_LOWER_LEAF], GPU_M4_DRM_PAGE_SIZE) ||
	    memcmp(a->before[1], r->expected[M4_UPPER_LEAF], GPU_M4_DRM_PAGE_SIZE)) {
		ret = -ESTALE;
		goto fault;
	}
	/* The live AP children are still detached/zero. Preserve every declared
	 * old leaf, plus literal UMA/parameter/fixture edges and permanent guards.
	 */
	for (u32 i = 0; i < M4_READY_TABLES; i++)
		if (!r->table[i] || memchr_inv(page_address(r->table[i]), 0, GPU_M4_DRM_PAGE_SIZE)) {
			ret = -EPROTO;
			goto fault;
		}
	if (!m4_arena_parent_image(r->expected[M4_READY_TABLES], 2048, 0,
		page_to_phys(r->table[M4_UPPER_LEAF]) | 3) ||
	    !m4_arena_parent_image(r->expected[M4_READY_TABLES + 1], 2048, 7,
		page_to_phys(r->table[M4_LOWER_MIDDLE]) | 3) ||
	    !m4_arena_parent_image(r->expected[M4_LOWER_MIDDLE], 2048, 128,
		page_to_phys(r->table[M4_LOWER_LEAF]) | 3)) {
		ret = -EPROTO;
		goto fault;
	}
	if (get_unaligned_le64(a->before[0]) != (r->buffer[GPU_M4_UMA].pa | M4_ARENA_LOWER_ATTRS) ||
	    get_unaligned_le64(a->before[0] + 16) !=
		(r->buffer[GPU_M4_PARAMETER].pa | M4_ARENA_LOWER_ATTRS) ||
	    get_unaligned_le64(a->before[0] + 8) || get_unaligned_le64(a->before[0] + 24) ||
	    get_unaligned_le64(a->before[0] + 65 * 8) ||
	    (r->execution && get_unaligned_le64(a->before[0] + 64 * 8) !=
		(r->execution->buffer[M4_EXEC_WORK].pa | M4_ARENA_LOWER_ATTRS))) {
		ret = -EPROTO;
		goto fault;
	}
	/* One sealed global lower ledger. Find a complete zero payload+guard run,
	 * never infer free slots from a sample index or a bitmap alone.
	 */
	for (first = 0; first + 2 * M4_ARENA_CELLS <= 2048; first++) {
		if (first <= 1 || (first <= 3 && first + 2 * M4_ARENA_CELLS > 3) ||
		    (first <= 65 && first + 2 * M4_ARENA_CELLS > 65))
			continue;
		if (!memchr_inv(a->before[0] + first * 8, 0, 2 * M4_ARENA_CELLS * 8))
			break;
	}
	upper_end = GPU_M4_HOST_START + GPU_M4_BOOTSTRAP_SPAN;
	if (first + 2 * M4_ARENA_CELLS > 2048 || !gpu_m4_drm_aligned(a->upper_start) ||
	    a->upper_start < GPU_M4_HOST_START || a->upper_start >= upper_end ||
	    2ULL * M4_ARENA_CELLS * GPU_M4_DRM_PAGE_SIZE > upper_end - a->upper_start ||
	    memchr_inv(a->before[1] + ((a->upper_start - GPU_M4_HOST_START) >> 14) * 8,
		0, 2 * M4_ARENA_CELLS * 8)) {
		ret = -ENOSPC;
		goto fault;
	}
	a->lower_first = first;
	a->upper_end = a->upper_start + 2ULL * M4_ARENA_CELLS * GPU_M4_DRM_PAGE_SIZE;
	memcpy(a->sealed, a->before, sizeof(a->sealed));
	/* Every native owner and every complete zero extent is now checked. Only
	 * private expected images change; existing child-to-parent publication is
	 * the sole writer/cleaner of live bootstrap tables.
	 */
	for (u32 i = 0; i < M4_ARENA_CELLS; i++) {
		struct m4_command_cell *c = &a->cell[i];

		c->va = GPU_M4_BOOTSTRAP_LOWER + (u64)(first + 2 * i) * GPU_M4_DRM_PAGE_SIZE;
		c->fw_va = a->upper_start + (u64)2 * i * GPU_M4_DRM_PAGE_SIZE;
		put_unaligned_le64(c->pa | M4_ARENA_LOWER_ATTRS, a->sealed[0] + (first + 2 * i) * 8);
		put_unaligned_le64(c->pa | m4_arena_cell_attrs(a),
			a->sealed[1] + ((c->fw_va - GPU_M4_HOST_START) >> 14) * 8);
	}
	memcpy(r->expected[M4_LOWER_LEAF], a->sealed[0], GPU_M4_DRM_PAGE_SIZE);
	memcpy(r->expected[M4_UPPER_LEAF], a->sealed[1], GPU_M4_DRM_PAGE_SIZE);
	r->next_va = a->upper_end;
	a->prepared = true;
	mutex_unlock(lock);
	return 0;
fault:
	a->error = ret < 0 ? ret : -EIO;
	mutex_unlock(lock);
	return a->error;
}

static int m4_arena_seed_locked(struct m4_gpu_hello *h,
	const struct gpu_m4_drm_upper_backing *backing, u64 va,
	enum gpu_m4_drm_upper_class class)
{
	struct m4_command_arena *a = h->ready->arena;

	lockdep_assert_held(&h->gpu->lock);
	if (!a || !a->prepared || a->error || a->hello != h || !h->ready->published ||
	    h->gpu->firmware_start_attempted || !h->gpu->stopped_verified ||
	    class != a->cell_class)
		return -EPERM;
	/* Match an actual retained owner first, without dereferencing foreign b->owner. */
	for (u32 i = 0; i < M4_ARENA_CELLS; i++) {
		struct m4_command_cell *c = &a->cell[i];

		if (c->fw_va == va && m4_arena_backing_equal(c, backing))
			return m4_arena_cell_valid_locked(c);
	}
	return -EPERM;
}

static int m4_arena_import_locked(struct m4_gpu_hello *h,
	const struct gpu_m4_drm_upper_config *config)
{
	struct m4_gpu_ready *r = h->ready;
	struct m4_command_arena *a = r->arena;
	struct gpu_m4_drm_upper *upper;
	int ret;

	lockdep_assert_held(&h->gpu->lock);
	if (!config || !a || !a->prepared || a->error || a->imported || r->upper ||
	    !r->published || h->gpu->firmware_start_attempted || !h->gpu->stopped_verified ||
	    config->lock != &h->gpu->lock || config->session != h ||
	    config->normal_min_va != a->upper_end || r->next_va != a->upper_end ||
	    config->root.cpu != h->tables || config->root.pa != h->region[M4_TABLES].start ||
	    config->root.owner != h || config->root.class != GPU_M4_DRM_UPPER_RESERVED_ROOT ||
	    config->middle.cpu != h->shared || config->middle.pa != h->region[M4_SHARED].start ||
	    config->middle.owner != h || config->middle.class != GPU_M4_DRM_UPPER_RESERVED_MIDDLE ||
	    config->child_count != 1 || !config->children ||
	    config->children[0].cpu != page_address(r->table[M4_UPPER_LEAF]) ||
	    config->children[0].pa != page_to_phys(r->table[M4_UPPER_LEAF]) ||
	    config->children[0].owner != h ||
	    config->children[0].class != GPU_M4_DRM_UPPER_INHERITED_AP ||
	    config->max_pages < M4_ARENA_CELLS || config->max_leases < M4_ARENA_CELLS)
		return -EPERM;
	upper = gpu_m4_drm_upper_create_locked(config);
	if (IS_ERR(upper)) {
		ret = PTR_ERR(upper);
		goto fault;
	}
	r->upper = upper; /* Retain actual upper/session even if a later import fails. */
	for (u32 i = 0; i < M4_ARENA_CELLS; i++) {
		struct m4_command_cell *c = &a->cell[i];
		struct gpu_m4_drm_upper_lease *lease;

		lease = gpu_m4_drm_upper_import_seed_locked(upper, &c->backing,
			c->fw_va, a->cell_class);
		if (IS_ERR(lease)) {
			ret = PTR_ERR(lease);
			goto fault;
		}
		c->upper = lease;
	}
	a->imported = true;
	return 0;
fault:
	a->error = ret < 0 ? ret : -EIO;
	return a->error;
}

/* Immutable arena inventory, after the structural checks in the caller. The
 * before/sealed images, backing descriptors, cell PAs and native GEM/NC
 * identities never change after sealing; ordinary packets and work-cell
 * ownership transitions do not touch them. Proven once, re-proven when the
 * READY exclusion set grows, and by every full diagnostic.
 */
static bool m4_arena_inventory_locked(struct m4_gpu_hello *h)
{
	struct m4_command_arena *a = h->ready->arena;
	u32 upper_first = (a->upper_start - GPU_M4_HOST_START) >> 14;

	for (u32 i = 0; i < 2048; i++) {
		bool lower_payload = i >= a->lower_first && i < a->lower_first + 2 * M4_ARENA_CELLS &&
			!((i - a->lower_first) & 1);
		bool upper_payload = i >= upper_first && i < upper_first + 2 * M4_ARENA_CELLS &&
			!((i - upper_first) & 1);

		if ((!lower_payload && get_unaligned_le64(a->sealed[0] + i * 8) !=
		     get_unaligned_le64(a->before[0] + i * 8)) ||
		    (!upper_payload && get_unaligned_le64(a->sealed[1] + i * 8) !=
		     get_unaligned_le64(a->before[1] + i * 8)))
			return false;
	}
	for (u32 i = 0; i < M4_ARENA_CELLS; i++) {
		struct m4_command_cell *c = &a->cell[i];
		u32 lower = a->lower_first + 2 * i, upper = upper_first + 2 * i;

		if (!m4_arena_backing_equal(c, &c->backing) ||
		    get_unaligned_le64(a->before[0] + lower * 8) ||
		    get_unaligned_le64(a->before[0] + (lower + 1) * 8) ||
		    get_unaligned_le64(a->before[1] + upper * 8) ||
		    get_unaligned_le64(a->before[1] + (upper + 1) * 8) ||
		    get_unaligned_le64(a->sealed[0] + lower * 8) != (c->pa | M4_ARENA_LOWER_ATTRS) ||
		    get_unaligned_le64(a->sealed[1] + upper * 8) !=
			(c->pa | m4_arena_cell_attrs(a)) ||
		    m4_ready_page_allowed_locked(h, c->pa, NULL))
			return false;
		for (u32 j = 0; j < i; j++)
			if (a->cell[j].pa == c->pa || a->cell[j].gem == c->gem ||
			    a->cell[j].map.vaddr == c->map.vaddr)
				return false;
	}
	return true;
}

/* full=false: per-publication/poll arena health. Structure, the expected
 * lower image and upper prefix against the sealed images, and per cell its
 * selected identity, layout, live canonical lower/upper path and guard, and
 * current lease/borrowers. full=true also re-proves the immutable inventory;
 * a failed re-proof withdraws the cached one.
 */
static bool m4_arena_memory_check(struct m4_gpu_hello *h, bool full)
{
	struct m4_gpu_ready *r = h->ready;
	struct m4_command_arena *a = r->arena;
	u32 upper_first;

	lockdep_assert_held(&h->gpu->lock);
	if (!a)
		return !runtime_enable;
	if (a->hello != h || !a->prepared || !a->imported || a->error || !r->upper ||
	    a->allocated != M4_ARENA_CELLS || a->lower_first > 2048 - 2 * M4_ARENA_CELLS ||
	    a->lower_first <= 1 ||
	    (a->lower_first <= 3 && a->lower_first + 2 * M4_ARENA_CELLS > 3) ||
	    (a->lower_first <= 65 && a->lower_first + 2 * M4_ARENA_CELLS > 65) ||
	    !gpu_m4_drm_aligned(a->upper_start) || a->upper_start < GPU_M4_HOST_START ||
	    a->upper_start >= GPU_M4_HOST_START + GPU_M4_BOOTSTRAP_SPAN ||
	    a->upper_end != a->upper_start + 2ULL * M4_ARENA_CELLS * GPU_M4_DRM_PAGE_SIZE ||
	    a->upper_end > GPU_M4_HOST_START + GPU_M4_BOOTSTRAP_SPAN ||
	    memcmp(r->expected[M4_LOWER_LEAF], a->sealed[0], GPU_M4_DRM_PAGE_SIZE))
		return false;
	/* Dynamic canonical aliases after the sealed floor may extend this AP
	 * child. They may not replace any prefix byte, including arena guards.
	 */
	if (memcmp(r->expected[M4_UPPER_LEAF], a->sealed[1],
		  ((a->upper_end - GPU_M4_HOST_START) >> 14) * 8))
		return false;
	if (full || !m4_arena_proved_locked(a)) {
		a->proved = false;
		if (!m4_arena_inventory_locked(h))
			return false;
		a->proved_extra = r->upper_extra_count;
		a->proved = true;
	}
	upper_first = (a->upper_start - GPU_M4_HOST_START) >> 14;
	for (u32 i = 0; i < M4_ARENA_CELLS; i++) {
		struct m4_command_cell *c = &a->cell[i];
		u32 lower = a->lower_first + 2 * i, upper = upper_first + 2 * i;

		if (c->arena != a || m4_arena_cell_valid_locked(c) ||
		    !c->upper || c->baseline_refs < 2 ||
		    gpu_m4_drm_upper_va_locked(c->upper) != c->fw_va ||
		    c->va != GPU_M4_BOOTSTRAP_LOWER + (u64)lower * GPU_M4_DRM_PAGE_SIZE ||
		    c->fw_va != a->upper_start + (u64)2 * i * GPU_M4_DRM_PAGE_SIZE ||
		    get_unaligned_le64(page_address(r->table[M4_LOWER_LEAF]) + lower * 8) !=
			(c->pa | M4_ARENA_LOWER_ATTRS) ||
		    get_unaligned_le64(page_address(r->table[M4_LOWER_LEAF]) + (lower + 1) * 8) ||
		    get_unaligned_le64(page_address(r->table[M4_UPPER_LEAF]) + upper * 8) !=
			(c->pa | m4_arena_cell_attrs(a)) ||
		    get_unaligned_le64(page_address(r->table[M4_UPPER_LEAF]) + (upper + 1) * 8))
			return false;
	}
	/* No command-payload read here, before or after current-job protection. */
	return true;
}

#endif
