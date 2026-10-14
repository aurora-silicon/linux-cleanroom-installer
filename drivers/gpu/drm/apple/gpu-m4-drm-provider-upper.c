// SPDX-License-Identifier: GPL-2.0-only
#include <linux/err.h>
#include <linux/interval_tree_generic.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/timekeeping.h>

#include "gpu-m4-drm-provider-upper.h"

#define UPPER_PAGE 0x4000U
#define UPPER_ENTRIES 2048U
#define UPPER_PA_MASK 0x3ffffffc000ULL
#define UPPER_ROOT_LIVE 0x00200000000004cbULL
#define UPPER_PA_TOMB 1ULL /* never a valid aligned PA */

/* Reserved VA interval of one u->leases member, inclusive: its pages and
 * trailing guard, exactly [va, va + size + UPPER_PAGE - 1]. An interval tree
 * node (subtree_last augments the start-ordered rbtree), so membership change
 * is O(log n), the free space between reservations coalesces by construction
 * and overlapping reservations (only ever seeds) stay exact.
 */
struct upper_extent {
	struct rb_node rb;
	u64 start, last, subtree_last;
};

#define UPPER_EXTENT_START(x) ((x)->start)
#define UPPER_EXTENT_LAST(x) ((x)->last)
INTERVAL_TREE_DEFINE(struct upper_extent, rb, u64, subtree_last,
		     UPPER_EXTENT_START, UPPER_EXTENT_LAST, static, upper_extent)

struct upper_child {
	struct gpu_m4_drm_upper_table table;
	struct page *page;
	u64 *expected;
	u32 middle;
	bool linked;
	/* Its 2048 slots equalled expected[] at the latest graph pass and no
	 * owner store changed it since (upper_changed() clears it).
	 */
	bool verified;
};

/* 1: every graph pass compares every child slot; 0: an owned child table
 * unchanged since its last complete comparison is skipped (root, middle,
 * parent links and inherited children are always compared).
 */
/* 1: a published lease's recheck reuses the graph pass already made at the
 * current owner revision.
 */
static bool upper_graph_memo = true;
#ifdef MODULE
module_param(upper_graph_memo, bool, 0644);
MODULE_PARM_DESC(upper_graph_memo, "Published-lease rechecks reuse a graph pass at the same owner revision (default 1)");
#endif

static bool upper_graph_full;
#ifdef MODULE
module_param(upper_graph_full, bool, 0644);
MODULE_PARM_DESC(upper_graph_full, "Compare every upper child table slot on each graph pass (default 0: changed children only)");
#endif

struct gpu_m4_drm_upper_lease {
	struct list_head link;
	struct gpu_m4_drm_upper *upper;
	struct gpu_m4_drm_upper_backing backing;
	u64 *pa, va, root_value, attrs;
	struct gpu_m4_drm_upper_update *updates;
	u32 pages, refs, update_count, update_capacity;
	/* Reservation in upper->reserved exactly while a u->leases member. */
	struct upper_extent reserved;
	/* Captured prefix of pa[] owned in the checked PA index, until reap. */
	u32 indexed;
	enum gpu_m4_drm_upper_window window;
	enum gpu_m4_drm_upper_class class;
	enum gpu_m4_drm_upper_placement placement;
	u32 allocation_gates;
	/* Publication proof of this held, published lease: the owner revision and
	 * root[2] at which its complete owned PTE/guard path (against the held
	 * immutable pin/PA inventory, class, VA and span) last checked out. Any
	 * lease/table/expected-ledger/index mutation advances the revision and
	 * voids it. Never an access, protection or cache grant.
	 */
	u64 proof_revision, proof_root;
	bool held, charged, visible, published, garbage, seeded;
};

/* One checked PA claim. owner is the retained lease; NULL names an owned or
 * inherited translation table (root, middle or child).
 */
struct upper_pa_slot {
	u64 pa;
	const void *owner;
};

struct gpu_m4_drm_upper {
	struct gpu_m4_drm_upper_config config;
	struct gpu_m4_drm_upper_ops ops;
	struct upper_child *child;
	struct list_head leases, garbage;
	u64 root_snapshot[64];
	u64 foreign_root_transitioned;
	u64 middle_snapshot[UPPER_ENTRIES];
	/* Checked PA index under config.lock: every table PA and every captured
	 * lease page, including unpublished garbage until it is actually reaped.
	 * Open addressing, at most half full; UPPER_PA_TOMB marks a removal.
	 */
	struct upper_pa_slot *index;
	u32 index_size, index_live, index_used;
	/* Any lease/table/expected-ledger mutation advances revision. A health
	 * pass may omit per-lease slot rechecks ONLY for the exact revision and
	 * root[2] word whose complete owned slot/guard pass already succeeded;
	 * the live table graph itself is still compared on every pass.
	 */
	u64 revision, checked_revision, checked_root;
	/* The revision of the latest successful graph pass (published-lease
	 * reuse skips another pass at the same revision).
	 */
	u64 graph_revision;
	/* upper_root_recent memo: ktime and revision of the latest successful
	 * live root/middle check.
	 */
	u64 root_checked_ns, root_checked_revision;
	/* Allocator indexes, exact mirrors of the state above, never authority.
	 * middle_child[m] is 1 + the u->child[] index linked to middle m (0:
	 * none); children are never removed or moved. reserved holds every
	 * u->leases member's reservation (lease pages and guard), so placement
	 * visits reservations in VA order instead of walking every lease for
	 * every page, and a lease leaving u->leases frees its VA in O(log n).
	 */
	u16 middle_child[UPPER_ENTRIES];
	struct rb_root_cached reserved;
	u64 full_scans, slot_scans; /* diagnostic statistics only */
	/* REMOVE-DEBUG: placement cost: calls, refusals, ns, scanned span in
	 * pages, live child slot reads and extent/segment steps.
	 */
	struct gpu_m4_drm_upper_choose_stats choose;
	/* REMOVE-DEBUG: published-lease reuse: full path walks vs proof reuse,
	 * for publication and for held-lease checks.
	 */
	u64 publish_walks, publish_reused, lease_walks, lease_reused;
	/* REMOVE-DEBUG: released leases and latched release failures. */
	u64 released, release_failures;
	/* REMOVE-DEBUG: last failing sub-check line and one-shot report mask. */
	u32 why_line, reported;
	u32 children, pages, leases_count, reapers;
	int error;
	bool shutdown, dead;
};

static int upper_fault(struct gpu_m4_drm_upper *u, int error);

/* REMOVE-DEBUG: record the exact failing upper sub-check (source line) so a
 * health/verify failure can be attributed once, only on the failure path.
 */
#define UPPER_WHY(owner, err) ({ \
	__typeof__(err) __why = (err); \
	/* Record only a failure, and keep the innermost/first recorded site. */ \
	if (__why && !(owner)->why_line) \
		(owner)->why_line = __LINE__; \
	__why; })

static void upper_report(struct gpu_m4_drm_upper *u, u32 kind, const char *what,
			 int error, const struct gpu_m4_drm_upper_lease *l, u64 indexed)
{
	if (u->reported & kind)
		return;
	u->reported |= kind;
	pr_warn("REMOVE-DEBUG GPU upper %s failed %d at gpu-m4-drm-provider-upper.c:%u lease_va=%#llx pages=%u indexed=%u refs=%u visible=%u published=%u seeded=%u rev=%llu checked=%llu index_live=%u index_used=%u claims=%llu children=%u leases=%u pages=%u error=%d\n",
		what, error, u->why_line, l ? l->va : 0, l ? l->pages : 0,
		l ? l->indexed : 0, l ? l->refs : 0, l ? l->visible : 0,
		l ? l->published : 0, l ? l->seeded : 0, u->revision,
		u->checked_revision, u->index_live, u->index_used, indexed,
		u->children, u->leases_count, u->pages, u->error);
}

static u32 upper_index_hash(const struct gpu_m4_drm_upper *u, u64 pa)
{
	return (u32)(((pa >> 14) * 0x9e3779b97f4a7c15ULL) >> 32) & (u->index_size - 1);
}

static struct upper_pa_slot *upper_index_slot(struct gpu_m4_drm_upper *u, u64 pa)
{
	u32 i, mask = u->index_size - 1;

	if (!u->index_size || pa == UPPER_PA_TOMB || !pa)
		return NULL;
	/* Load is at most one half, so an empty terminator always exists. */
	for (i = upper_index_hash(u, pa);; i = (i + 1) & mask) {
		struct upper_pa_slot *s = &u->index[i];

		if (!s->pa)
			return NULL;
		if (s->pa == pa)
			return s;
	}
}

/* Caller reserved capacity and proved pa absent. */
static void upper_index_put(struct gpu_m4_drm_upper *u, u64 pa, const void *owner)
{
	u32 i, mask = u->index_size - 1;

	for (i = upper_index_hash(u, pa);; i = (i + 1) & mask) {
		struct upper_pa_slot *s = &u->index[i];

		if (s->pa && s->pa != UPPER_PA_TOMB)
			continue;
		if (!s->pa)
			u->index_used++;
		s->pa = pa;
		s->owner = owner;
		u->index_live++;
		return;
	}
}

static int upper_index_reserve(struct gpu_m4_drm_upper *u, u32 extra)
{
	struct upper_pa_slot *old = u->index, *table;
	u32 old_size = u->index_size, size = 64, i;
	u64 need = (u64)u->index_live + extra;

	if ((u64)u->index_used + extra <= u->index_size / 2)
		return 0;
	while (size / 2 < need) {
		if (size >= (1U << 30))
			return -ENOSPC;
		size <<= 1;
	}
	table = kvcalloc(size, sizeof(*table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;
	u->index = table;
	u->index_size = size;
	u->index_live = u->index_used = 0;
	for (i = 0; i < old_size; i++)
		if (old[i].pa && old[i].pa != UPPER_PA_TOMB)
			upper_index_put(u, old[i].pa, old[i].owner);
	kvfree(old);
	return 0;
}

static void upper_index_forget(struct gpu_m4_drm_upper_lease *l)
{
	struct gpu_m4_drm_upper *u = l->upper;
	u32 i;

	for (i = 0; i < l->indexed; i++) {
		struct upper_pa_slot *s = upper_index_slot(u, l->pa[i]);

		if (s && s->owner == l) {
			s->pa = UPPER_PA_TOMB;
			s->owner = NULL;
			u->index_live--;
		}
	}
	l->indexed = 0;
	u->revision++;
}

static int upper_errno(int error)
{
	return error < 0 ? error : -EIO;
}

static u64 upper_read(const void *cpu, u32 index)
{
	return le64_to_cpu(READ_ONCE(((const __le64 *)cpu)[index]));
}

static void upper_write(void *cpu, u32 index, u64 value)
{
	WRITE_ONCE(((__le64 *)cpu)[index], cpu_to_le64(value));
}

static bool upper_pa(u64 pa)
{
	return pa && !(pa & ~UPPER_PA_MASK);
}

static int upper_table(struct gpu_m4_drm_upper *u,
		       const struct gpu_m4_drm_upper_table *t)
{
	if (!t->cpu || ((unsigned long)t->cpu & (UPPER_PAGE - 1)) ||
	    !upper_pa(t->pa) || !t->owner)
		return -EINVAL;
	return u->ops.table_locked(u->config.session, t);
}

static struct upper_child *upper_find(struct gpu_m4_drm_upper *u, u32 middle)
{
	u32 slot = middle < UPPER_ENTRIES ? u->middle_child[middle] : 0;

	return slot ? &u->child[slot - 1] : NULL;
}

/* l joins u->leases: reserve its pages and trailing guard. */
static void upper_reserve(struct gpu_m4_drm_upper *u, struct gpu_m4_drm_upper_lease *l)
{
	l->reserved.start = l->va;
	l->reserved.last = l->va + l->backing.size + UPPER_PAGE - 1;
	upper_extent_insert(&l->reserved, &u->reserved);
}

/* l leaves u->leases: its whole reservation, guard included, is free again. */
static void upper_unreserve(struct gpu_m4_drm_upper *u, struct gpu_m4_drm_upper_lease *l)
{
	upper_extent_remove(&l->reserved, &u->reserved);
}

/* Live root/shared-l2, not allocator state, is authoritative. Unknown parents
 * are rejected before dereferencing any child. Preserve foreign root bytes.
 */
static int upper_root(struct gpu_m4_drm_upper *u)
{
	u64 root = upper_read(u->config.root.cpu, 2);
	u32 i;
	int ret;

	ret = upper_table(u, &u->config.root);
	if (ret)
		return UPPER_WHY(u, ret);
	ret = upper_table(u, &u->config.middle);
	if (ret)
		return UPPER_WHY(u, ret);
	if (root != (u->config.middle.pa | 3) &&
	    root != (u->config.middle.pa | UPPER_ROOT_LIVE))
		return UPPER_WHY(u, -EPROTO);
	for (i = 0; i < 64; i++) {
		u64 observed;

		if (i == 2)
			continue;
		observed = upper_read(u->config.root.cpu, i);
		if (observed == u->root_snapshot[i]) {
			/* A classified firmware link may not silently roll back to its
			 * cold image. Preserve the original immutable snapshot.
			 */
			if (u->foreign_root_transitioned & (1ULL << i))
				return UPPER_WHY(u, -EPROTO);
			continue;
		}
		if (u->ops.foreign_root_locked) {
			ret = u->ops.foreign_root_locked(u->config.session, i,
				u->root_snapshot[i], observed);
			if (ret)
				return UPPER_WHY(u, upper_errno(ret));
			if (!(u->foreign_root_transitioned & (1ULL << i)))
				u->revision++;
			u->foreign_root_transitioned |= 1ULL << i;
		} else {
			return UPPER_WHY(u, -EPROTO);
		}
	}
	return 0;
}

/* Per-lease rechecks (publication-proof reuse, owned path checks, alias
 * walks) re-read the live root and middle (two table invalidations) once per
 * call. Within upper_root_reuse_us of a successful live check at the same
 * owner revision one check stands for all of them, as for the leases of one
 * window publication; whole-graph passes always re-read. 0 disables.
 */
#ifdef MODULE
static bool upper_member_flag = true;
#else
static bool upper_member_flag;
#endif
static unsigned int upper_root_reuse_us = 200;
#ifdef MODULE
module_param(upper_root_reuse_us, uint, 0644);
MODULE_PARM_DESC(upper_root_reuse_us, "Reuse a live upper root check this many microseconds for per-lease checks (0 = never)");
#endif

static int upper_root_recent(struct gpu_m4_drm_upper *u)
{
	u64 now = ktime_get_ns();
	int ret;

	if (upper_root_reuse_us && u->root_checked_ns &&
	    u->root_checked_revision == u->revision &&
	    now - u->root_checked_ns < (u64)upper_root_reuse_us * 1000)
		return 0;
	ret = upper_root(u);
	if (!ret) {
		u->root_checked_ns = now;
		u->root_checked_revision = u->revision;
	}
	return ret;
}

static int upper_graph_pass(struct gpu_m4_drm_upper *u, bool changed_only)
{
	u32 i, j;
	int ret = upper_root(u);

	if (ret)
		return UPPER_WHY(u, ret);
	for (i = 0; i < UPPER_ENTRIES; i++) {
		u64 parent = upper_read(u->config.middle.cpu, i);
		struct upper_child *child = upper_find(u, i);

		if (parent != u->middle_snapshot[i])
			return UPPER_WHY(u, -EPROTO);
		if (!parent) {
			if (child && child->linked)
				return UPPER_WHY(u, -EPROTO);
			continue;
		}
		if (!child || parent != (child->table.pa | 3) ||
		    !child->linked)
			return UPPER_WHY(u, -EPROTO);
		if (child->page) {
			/* This owner allocated and retains the real normal-RAM WB page.
			 * The inherited/no-map oracle must not invent a new CPU view.
			 */
			ret = child->table.cpu == page_address(child->page) &&
				child->table.pa == page_to_phys(child->page) ? 0 : -EPROTO;
		} else {
			ret = upper_table(u, &child->table);
		}
		if (ret)
			return UPPER_WHY(u, ret);
		/* Only this owner's own allocated table pages are trusted
		 * between its stores; inherited (foreign-writable) children are
		 * compared on every pass.
		 */
		if (changed_only && child->page && child->verified && !READ_ONCE(upper_graph_full))
			continue;
		for (j = 0; j < UPPER_ENTRIES; j++)
			if (upper_read(child->table.cpu, j) != child->expected[j])
				return UPPER_WHY(u, -EPROTO);
		child->verified = true;
	}
	return 0;
}

/* Every pass re-proves root, middle, parent links and every inherited
 * child (foreign occupancy wins); owned children unchanged since their last
 * complete comparison are skipped unless upper_graph_full.
 */
static int upper_graph(struct gpu_m4_drm_upper *u)
{
	return upper_graph_pass(u, true);
}

static bool upper_backing_valid(const struct gpu_m4_drm_upper_backing *b)
{
	return b && b->owner && b->identity && b->get_locked && b->put &&
		b->page_locked && b->check_locked && b->ready_locked && b->size &&
		!(b->offset & (UPPER_PAGE - 1)) && !(b->size & (UPPER_PAGE - 1)) &&
		b->size <= U64_MAX - b->offset &&
		(b->policy == GPU_M4_DRM_CPU_NC || b->policy == GPU_M4_DRM_CPU_WRITEBACK);
}

static bool upper_same(const struct gpu_m4_drm_upper_backing *a,
		       const struct gpu_m4_drm_upper_backing *b)
{
	return a->owner == b->owner && a->identity == b->identity &&
		a->offset == b->offset && a->size == b->size && a->policy == b->policy &&
		a->get_locked == b->get_locked && a->put == b->put &&
		a->page_locked == b->page_locked && a->check_locked == b->check_locked &&
		a->ready_locked == b->ready_locked;
}

/* Immutable page inventory of one retained lease. capture=true is admission:
 * resolve every page, prove RAM/table exclusion and native identity, then claim
 * its PA in the checked index, so uniqueness against every retained lease,
 * garbage and table is one lookup, not an all-pairs scan. The held backing ref
 * prevents migration/remap/free; capture is NOT repeated for a held lease.
 * capture=false is the diagnostic/fault recheck of the same oracles against
 * the captured inventory and its index ownership.
 */
static int upper_inventory(struct gpu_m4_drm_upper_lease *l, bool capture)
{
	struct gpu_m4_drm_upper *u = l->upper;
	u32 i;
	int ret;

	if (capture) {
		if (l->indexed)
			return UPPER_WHY(u, -EPROTO);
		ret = upper_index_reserve(u, l->pages);
		if (ret)
			return UPPER_WHY(u, ret);
	} else if (l->indexed != l->pages) {
		return UPPER_WHY(u, -EPROTO);
	}
	for (i = 0; i < l->pages; i++) {
		u64 pa, offset = l->backing.offset + (u64)i * UPPER_PAGE;
		const struct upper_pa_slot *claim;

		ret = l->backing.page_locked(l->backing.owner, offset, &pa);
		if (ret)
			return UPPER_WHY(u, ret);
		if (!upper_pa(pa) || !pfn_valid(PHYS_PFN(pa)) ||
		    region_intersects(pa, UPPER_PAGE, IORESOURCE_SYSTEM_RAM,
				      IORES_DESC_NONE) != REGION_INTERSECTS ||
		    pa == u->config.root.pa || pa == u->config.middle.pa)
			return UPPER_WHY(u, -EPERM);
		ret = l->backing.check_locked(l->backing.owner, l->backing.identity, offset, pa);
		if (ret)
			return UPPER_WHY(u, ret);
		claim = upper_index_slot(u, pa);
		if (claim && !claim->owner)
			return UPPER_WHY(u, -EPERM); /* translation table */
		if (capture) {
			if (claim)
				return UPPER_WHY(u, -EEXIST); /* another or this same lease */
			l->pa[i] = pa;
			upper_index_put(u, pa, l);
			l->indexed = i + 1;
			continue;
		}
		if (!claim)
			return UPPER_WHY(u, -EPROTO);
		if (claim->owner != l)
			return UPPER_WHY(u, -EEXIST);
		if (pa != l->pa[i])
			return UPPER_WHY(u, -EPROTO);
	}
	if (capture)
		u->revision++;
	return 0;
}

static bool upper_placement(struct gpu_m4_drm_upper *u, u64 va,
			    enum gpu_m4_drm_upper_placement placement)
{
	u32 middle = (va >> 25) & 2047;
	struct upper_child *child = upper_find(u, middle);
	u64 parent = upper_read(u->config.middle.cpu, middle);

	if (!child)
		return !parent && placement != GPU_M4_DRM_UPPER_EXISTING_ONLY;
	if (child->table.class != GPU_M4_DRM_UPPER_INHERITED_AP)
		return false;
	if ((placement == GPU_M4_DRM_UPPER_EXISTING_ONLY && !parent) ||
	    (placement == GPU_M4_DRM_UPPER_NEW_ONLY && parent))
		return false;
	return !parent || parent == (child->table.pa | 3);
}

/* First fit of the lease's pages plus its trailing guard page from first,
 * below the window end, within the first config.scan_pages pages (every page
 * counts, unsuitable parents included). A page is usable exactly when no
 * u->leases member reserves it (pages and guard), its middle admits the
 * placement and its live child slot, if any, is zero: the original per-page
 * predicate and the same lowest VA. The reservation tree yields reservations
 * meeting the scan in start order, placement is decided once per middle
 * segment, and a slot is read live only where a run can still complete
 * before the next reservation or the scan limit; a child-less middle has no
 * slot to read.
 */
static int upper_choose(struct gpu_m4_drm_upper_lease *l, u64 first)
{
	struct gpu_m4_drm_upper *u = l->upper;
	u64 end = l->window == GPU_M4_DRM_UPPER_TS ? GPU_M4_DRM_UPPER_END :
		GPU_M4_DRM_UPPER_TIMESTAMP;
	u64 need = (u64)l->pages + 1, run = 0, va = first, limit, scan;
	u64 started = ktime_get_ns();
	struct upper_extent *x = NULL;
	u32 i;

	u->choose.calls++;
	scan = (end - first) / UPPER_PAGE;
	if (scan > u->config.scan_pages)
		scan = u->config.scan_pages;
	limit = first + scan * UPPER_PAGE;
	if (limit > first)
		x = upper_extent_iter_first(&u->reserved, first, limit - 1);
	while (va < limit) {
		u64 stop = x && x->start < limit ? x->start : limit;
		u64 next = (va | (UPPER_PAGE * (u64)UPPER_ENTRIES - 1)) + 1, pages;
		struct upper_child *c;

		u->choose.steps++;
		/* Start order; a reservation wholly below va no longer matters. */
		if (x && x->last < va) {
			x = upper_extent_iter_next(x, first, limit - 1);
			continue;
		}
		if (x && x->start <= va) {
			run = 0;
			va = x->last + 1;
			x = upper_extent_iter_next(x, first, limit - 1);
			continue;
		}
		/* [va, next): unreserved, one middle, below the scan limit. */
		if (next > stop)
			next = stop;
		pages = (next - va) / UPPER_PAGE;
		if (!upper_placement(u, va, l->placement)) {
			run = 0;
			va = next;
			continue;
		}
		c = upper_find(u, (va >> 25) & 2047);
		if (!c) {
			if (run + pages >= need) {
				va += (need - run - 1) * UPPER_PAGE;
				goto found;
			}
			run += pages;
			va = next;
			continue;
		}
		/* A reservation or the limit follows: no run can complete here. */
		if (next == stop && run + pages < need) {
			run = 0;
			va = next;
			continue;
		}
		for (; va < next; va += UPPER_PAGE) {
			u->choose.reads++;
			if (upper_read(c->table.cpu, (va >> 14) & 2047))
				run = 0;
			else if (++run == need)
				goto found;
		}
	}
	u->choose.pages += scan;
	u->choose.failures++;
	u->choose.ns += ktime_get_ns() - started;
	/* REMOVE-DEBUG: preserve the scan limit and report its actual use. */
	pr_warn("GPU upper capacity: scan=%u limit=%u need_pages=%u used_pages=%u leases=%u children=%u first=%#llx end=%#llx\n",
		(u32)scan, u->config.scan_pages, l->pages, u->pages,
		u->leases_count, u->children, first, end);
	return -ENOSPC;
found:
	/* va is the run's last page: the trailing guard. */
	u->choose.pages += (va - first) / UPPER_PAGE + 1;
	l->va = va - (u64)l->pages * UPPER_PAGE;
	for (i = 0; i <= l->pages; i++) {
		u32 middle = ((l->va + (u64)i * UPPER_PAGE) >> 25) & 2047;

		l->allocation_gates |= upper_read(u->config.middle.cpu, middle) ?
			GPU_M4_DRM_UPPER_EXISTING_CHILD : GPU_M4_DRM_UPPER_NEW_PARENT;
	}
	u->choose.ns += ktime_get_ns() - started;
	return 0;
}

static int upper_child_allocate(struct gpu_m4_drm_upper *u, u32 middle)
{
	struct upper_child *c;
	struct page *page;
	u64 pa;

	if (upper_find(u, middle))
		return 0;
	if (u->children == u->config.max_children) {
		pr_warn("GPU upper capacity: children=%u limit=%u middle=%u\n",
			u->children, u->config.max_children, middle);
		return -ENOSPC;
	}
	if (upper_read(u->config.middle.cpu, middle))
		return -EEXIST;
	if (upper_index_reserve(u, 1))
		return -ENOMEM;
	page = alloc_page(GFP_KERNEL | __GFP_ZERO);
	if (!page)
		return -ENOMEM;
	pa = page_to_phys(page);
	if (!upper_pa(pa) || !pfn_valid(PHYS_PFN(pa)) ||
	    region_intersects(pa, UPPER_PAGE, IORESOURCE_SYSTEM_RAM,
			      IORES_DESC_NONE) != REGION_INTERSECTS ||
	    pa == u->config.root.pa || pa == u->config.middle.pa)
		goto bad;
	/* Every retained table and captured lease page, including garbage. */
	if (upper_index_slot(u, pa))
		goto bad;
	c = &u->child[u->children++];
	*c = (struct upper_child) {
		.table = { .cpu = page_address(page), .pa = pa, .owner = u,
			.class = GPU_M4_DRM_UPPER_INHERITED_AP },
		.page = page, .middle = middle,
	};
	c->expected = kcalloc(UPPER_ENTRIES, sizeof(*c->expected), GFP_KERNEL);
	if (!c->expected) {
		u->children--;
		__free_page(page);
		return -ENOMEM;
	}
	u->middle_child[middle] = u->children;
	upper_index_put(u, pa, NULL);
	u->revision++;
	return 0;
bad:
	__free_page(page);
	return -EPERM;
}

struct gpu_m4_drm_upper *gpu_m4_drm_upper_create_locked(
	const struct gpu_m4_drm_upper_config *config)
{
	struct gpu_m4_drm_upper *u;
	const struct gpu_m4_drm_upper_ops *ops;
	u32 i, j;
	int ret;

	if (!config || !config->lock || !config->session || !config->ops ||
	    !config->max_children || config->max_children > UPPER_ENTRIES ||
	    config->child_count > config->max_children ||
	    (config->child_count && !config->children) ||
	    !config->max_pages || config->max_pages > (1U << 20) ||
	    !config->max_leases || config->max_leases > config->max_pages ||
	    !config->scan_pages || config->scan_pages > UPPER_ENTRIES * UPPER_ENTRIES ||
	    config->normal_min_va < GPU_M4_DRM_UPPER_START ||
	    config->normal_min_va > GPU_M4_DRM_UPPER_TIMESTAMP ||
	    (config->normal_min_va & (UPPER_PAGE - 1)) ||
	    config->root.class != GPU_M4_DRM_UPPER_RESERVED_ROOT ||
	    config->middle.class != GPU_M4_DRM_UPPER_RESERVED_MIDDLE ||
	    config->root.pa == config->middle.pa || PAGE_SIZE != UPPER_PAGE)
		return ERR_PTR(-EINVAL);
	lockdep_assert_held(config->lock);
	ops = config->ops;
	if (!ops->session_get_locked || !ops->session_put || !ops->table_locked ||
	    !ops->gate_locked || !ops->handoff_get_locked || !ops->handoff_put_locked ||
	    !ops->clean_locked || !ops->barrier_locked || !ops->asid0_tlbi_locked ||
	    !ops->commit_locked || !ops->shutdown_locked || !ops->fault_locked)
		return ERR_PTR(-EINVAL);
	u = kzalloc_obj(*u);
	if (!u)
		return ERR_PTR(-ENOMEM);
	u->child = kcalloc(config->max_children, sizeof(*u->child), GFP_KERNEL);
	if (!u->child) {
		kfree(u);
		return ERR_PTR(-ENOMEM);
	}
	u->config = *config;
	u->config.children = NULL;
	u->ops = *ops;
	INIT_LIST_HEAD(&u->leases);
	INIT_LIST_HEAD(&u->garbage);
	u->reserved = RB_ROOT_CACHED;
	ret = upper_table(u, &config->root);
	if (!ret)
		ret = upper_table(u, &config->middle);
	if (ret)
		goto failed;
	for (i = 0; i < 64; i++)
		u->root_snapshot[i] = upper_read(config->root.cpu, i);
	for (i = 0; i < UPPER_ENTRIES; i++)
		u->middle_snapshot[i] = upper_read(config->middle.cpu, i);
	for (i = 0; i < config->child_count; i++) {
		const struct gpu_m4_drm_upper_table *t = &config->children[i];

		if ((t->class != GPU_M4_DRM_UPPER_INHERITED_AP &&
		     t->class != GPU_M4_DRM_UPPER_INHERITED_FOREIGN) ||
		    t->pa == config->root.pa || t->pa == config->middle.pa) {
			ret = -EINVAL;
			goto failed;
		}
		ret = upper_table(u, t);
		if (ret)
			goto failed;
		for (j = 0; j < i; j++)
			if (t->pa == u->child[j].table.pa || t->cpu == u->child[j].table.cpu) {
				ret = -EEXIST;
				goto failed;
			}
		u->child[u->children] = (struct upper_child) { .table = *t, .middle = U32_MAX };
		u->child[u->children].expected = kcalloc(UPPER_ENTRIES, sizeof(u64), GFP_KERNEL);
		if (!u->child[u->children].expected) {
			ret = -ENOMEM;
			goto failed;
		}
		for (j = 0; j < UPPER_ENTRIES; j++)
			u->child[u->children].expected[j] = upper_read(t->cpu, j);
		for (j = 0; j < UPPER_ENTRIES; j++)
			if (u->middle_snapshot[j] == (t->pa | 3)) {
				if (u->child[u->children].middle != U32_MAX) {
					ret = -EEXIST;
					/* Include this expected allocation in cleanup. */
					u->children++;
					goto failed;
				}
				u->child[u->children].middle = j;
				u->child[u->children].linked = true;
				u->middle_child[j] = u->children + 1;
			}
		u->children++;
	}
	ret = upper_graph(u);
	if (ret)
		goto failed;
	for (i = 0; i < u->children; i++)
		if (u->child[i].middle == U32_MAX) {
			ret = -EPROTO;
			goto failed;
		}
	/* Root, middle and inherited children are already pairwise distinct. */
	ret = upper_index_reserve(u, 2 + u->children);
	if (ret)
		goto failed;
	upper_index_put(u, config->root.pa, NULL);
	upper_index_put(u, config->middle.pa, NULL);
	for (i = 0; i < u->children; i++)
		upper_index_put(u, u->child[i].table.pa, NULL);
	u->revision = 1;
	ret = ops->session_get_locked(config->session);
	if (ret)
		goto failed;
	return u;
failed:
	for (i = 0; i < u->children; i++)
		kfree(u->child[i].expected);
	kvfree(u->index);
	kfree(u->child);
	kfree(u);
	return ERR_PTR(upper_errno(ret));
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_create_locked);

struct gpu_m4_drm_upper_lease *gpu_m4_drm_upper_accept_placed_locked(
	struct gpu_m4_drm_upper *u, const struct gpu_m4_drm_upper_backing *backing,
	enum gpu_m4_drm_upper_window window, enum gpu_m4_drm_upper_class class,
	u64 minimum_va, enum gpu_m4_drm_upper_placement placement)
{
	struct gpu_m4_drm_upper_lease *l;
	u64 first, end;
	u32 middle, last;
	int ret;

	if (!u || !upper_backing_valid(backing) ||
	    (window != GPU_M4_DRM_UPPER_NORMAL && window != GPU_M4_DRM_UPPER_TS) ||
	    class > GPU_M4_DRM_UPPER_SHARED_CACHED ||
	    (window == GPU_M4_DRM_UPPER_TS && class != GPU_M4_DRM_UPPER_FW_ONLY) ||
	    (placement != GPU_M4_DRM_UPPER_ANY_CHILD &&
	     placement != GPU_M4_DRM_UPPER_EXISTING_ONLY && placement != GPU_M4_DRM_UPPER_NEW_ONLY))
		return ERR_PTR(-EINVAL);
	lockdep_assert_held(u->config.lock);
	if (u->dead || u->shutdown || u->error)
		return ERR_PTR(u->error ?: -ENODEV);
	first = window == GPU_M4_DRM_UPPER_TS ? GPU_M4_DRM_UPPER_TIMESTAMP : u->config.normal_min_va;
	end = window == GPU_M4_DRM_UPPER_TS ? GPU_M4_DRM_UPPER_END : GPU_M4_DRM_UPPER_TIMESTAMP;
	if (minimum_va) {
		if (minimum_va < first || minimum_va > end || (minimum_va & (UPPER_PAGE - 1)))
			return ERR_PTR(-EINVAL);
		first = minimum_va;
	}
	if (backing->size > end - first || UPPER_PAGE > end - first - backing->size)
		return ERR_PTR(-ENOSPC);
	ret = upper_graph(u);
	if (ret)
		return ERR_PTR(upper_errno(ret));
	list_for_each_entry(l, &u->leases, link)
		if (upper_same(&l->backing, backing) && l->window == window && l->class == class) {
			if ((l->va < first && !(l->seeded && !minimum_va)) ||
			    (placement == GPU_M4_DRM_UPPER_EXISTING_ONLY &&
			     l->allocation_gates != GPU_M4_DRM_UPPER_EXISTING_CHILD) ||
			    (placement == GPU_M4_DRM_UPPER_NEW_ONLY &&
			     l->allocation_gates != GPU_M4_DRM_UPPER_NEW_PARENT))
				return ERR_PTR(-EEXIST);
			ret = gpu_m4_drm_upper_reference_locked(l, backing, window, class);
			return ret ? ERR_PTR(upper_errno(ret)) : l;
		}
	if (backing->size / UPPER_PAGE > u->config.max_pages - u->pages ||
	    u->leases_count == u->config.max_leases) {
		pr_warn("GPU upper capacity: need_pages=%llu used_pages=%u max_pages=%u leases=%u max_leases=%u\n",
			backing->size / UPPER_PAGE, u->pages, u->config.max_pages,
			u->leases_count, u->config.max_leases);
		return ERR_PTR(-ENOSPC);
	}
	l = kzalloc_obj(*l);
	if (!l)
		return ERR_PTR(-ENOMEM);
	l->upper = u;
	l->backing = *backing;
	l->pages = backing->size / UPPER_PAGE;
	l->window = window;
	l->class = class;
	l->placement = placement;
	l->attrs = gpu_m4_drm_upper_class_attrs(class);
	l->pa = kcalloc(l->pages, sizeof(*l->pa), GFP_KERNEL);
	l->update_capacity = l->pages + l->pages / UPPER_ENTRIES + 2;
	l->updates = kcalloc(l->update_capacity, sizeof(*l->updates), GFP_KERNEL);
	if (!l->pa || !l->updates) {
		ret = -ENOMEM;
		goto failed;
	}
	ret = backing->get_locked(backing->owner);
	if (ret)
		goto failed;
	l->held = true;
	/* Failed owned acceptance remains charged until its deferred native put;
	 * repeated failed ioctls cannot accumulate an unbounded garbage pin list.
	 */
	l->charged = true;
	u->pages += l->pages;
	u->leases_count++;
	ret = upper_inventory(l, true);
	if (ret)
		goto failed;
	ret = upper_choose(l, first);
	if (ret)
		goto failed;
	middle = (l->va >> 25) & 2047;
	last = ((l->va + backing->size - 1) >> 25) & 2047;
	for (; middle <= last; middle++) {
		ret = upper_child_allocate(u, middle);
		if (ret)
			goto failed;
	}
	/* An allocator must never have returned a backing page as a new table:
	 * the index refused it. Recheck the admitted oracles once at this commit.
	 */
	ret = upper_inventory(l, false);
	if (ret)
		goto failed;
	l->root_value = upper_read(u->config.root.cpu, 2);
	l->refs = 1;
	list_add_tail(&l->link, &u->leases);
	upper_reserve(u, l);
	u->revision++;
	return l;
failed:
	if (l->held) {
		/* Its captured prefix stays claimed until actual reap. */
		l->garbage = true;
		list_add_tail(&l->link, &u->garbage);
		u->revision++;
	} else {
		kfree(l->updates);
		kfree(l->pa);
		kfree(l);
	}
	return ERR_PTR(upper_errno(ret));
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_accept_placed_locked);

struct gpu_m4_drm_upper_lease *gpu_m4_drm_upper_accept_locked(
	struct gpu_m4_drm_upper *u, const struct gpu_m4_drm_upper_backing *backing,
	enum gpu_m4_drm_upper_window window, enum gpu_m4_drm_upper_class class)
{
	return gpu_m4_drm_upper_accept_placed_locked(u, backing, window, class, 0,
		GPU_M4_DRM_UPPER_ANY_CHILD);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_accept_locked);

static bool upper_root_value(struct gpu_m4_drm_upper_lease *l)
{
	struct gpu_m4_drm_upper *u = l->upper;
	u64 root = upper_read(u->config.root.cpu, 2);

	return root == l->root_value ||
		(l->seeded && l->root_value == (u->config.middle.pa | 3) &&
		 root == (u->config.middle.pa | UPPER_ROOT_LIVE));
}

static int upper_guard(struct gpu_m4_drm_upper_lease *l)
{
	u64 guard = l->va + l->backing.size;
	struct upper_child *c = upper_find(l->upper, (guard >> 25) & 2047);

	if (c && (c->table.class != GPU_M4_DRM_UPPER_INHERITED_AP ||
		  upper_read(c->table.cpu, (guard >> 14) & 2047)))
		return UPPER_WHY(l->upper, -EEXIST);
	return 0;
}

/* Selected owned PTE/guard checks of one held lease against its admitted
 * inventory. The caller has checked the complete graph under the same owner
 * mutex. The immutable page inventory was captured once at admission and is
 * NOT reconstructed here; that remains the full diagnostic verifier's job.
 */
static int upper_slots_check(struct gpu_m4_drm_upper_lease *l)
{
	struct gpu_m4_drm_upper *u = l->upper;
	u32 i;

	if (!upper_root_value(l))
		return UPPER_WHY(u, -EPROTO);
	for (i = 0; i < l->pages; i++) {
		u64 va = l->va + (u64)i * UPPER_PAGE;
		struct upper_child *c = upper_find(u, (va >> 25) & 2047);
		u64 pte;

		if (!c || c->table.class != GPU_M4_DRM_UPPER_INHERITED_AP)
			return UPPER_WHY(u, -EPERM);
		pte = upper_read(c->table.cpu, (va >> 14) & 2047);
		if (pte != (l->visible ? l->pa[i] | l->attrs : 0))
			return UPPER_WHY(u, -EEXIST);
	}
	return UPPER_WHY(u, upper_guard(l));
}

static int upper_slots(struct gpu_m4_drm_upper_lease *l)
{
	int ret = upper_graph(l->upper);

	if (!ret)
		l->upper->graph_revision = l->upper->revision;
	return ret ?: upper_slots_check(l);
}

/* An already published lease's recheck: one graph pass per owner revision
 * (any lease/table/ledger mutation advances it), then this lease's own
 * slots and guard.
 */
static int upper_slots_reuse(struct gpu_m4_drm_upper_lease *l)
{
	struct gpu_m4_drm_upper *u = l->upper;

	if (!upper_graph_memo || !u->graph_revision || u->graph_revision != u->revision)
		return upper_slots(l);
	return upper_slots_check(l);
}

static int upper_child_owned(struct gpu_m4_drm_upper *u, struct upper_child *c)
{
	if (c->page)
		return UPPER_WHY(u, c->table.cpu == page_address(c->page) &&
			c->table.pa == page_to_phys(c->page) ? 0 : -EPROTO);
	return UPPER_WHY(u, upper_table(u, &c->table));
}

/* Exact owned translation path of selected pages of one published lease:
 * root words (including root[2]), its parent word, the owned child identity
 * and leaf, all against the held admitted pin PA/attributes and the expected
 * ledger. No other lease, leaf or backing oracle is consulted.
 */
static int upper_path(struct gpu_m4_drm_upper_lease *l, u32 first, u32 count)
{
	struct gpu_m4_drm_upper *u = l->upper;
	struct upper_child *checked = NULL;
	u32 i;
	int ret = upper_root_recent(u);

	if (ret)
		return UPPER_WHY(u, ret);
	if (!upper_root_value(l))
		return UPPER_WHY(u, -EPROTO);
	for (i = first; i < first + count; i++) {
		u64 va = l->va + (u64)i * UPPER_PAGE, parent, pte;
		u32 middle = (va >> 25) & 2047, index = (va >> 14) & 2047;
		struct upper_child *c = upper_find(u, middle);

		if (!c || c->table.class != GPU_M4_DRM_UPPER_INHERITED_AP)
			return UPPER_WHY(u, -EPERM);
		parent = upper_read(u->config.middle.cpu, middle);
		if (!c->linked || parent != u->middle_snapshot[middle] ||
		    parent != (c->table.pa | 3))
			return UPPER_WHY(u, -EPROTO);
		if (c != checked) {
			ret = upper_child_owned(u, c);
			if (ret)
				return UPPER_WHY(u, ret);
			checked = c;
		}
		pte = upper_read(c->table.cpu, index);
		if (pte != c->expected[index] || pte != (l->pa[i] | l->attrs))
			return UPPER_WHY(u, -EPROTO);
	}
	return 0;
}

struct gpu_m4_drm_upper_lease *gpu_m4_drm_upper_import_seed_locked(
	struct gpu_m4_drm_upper *u, const struct gpu_m4_drm_upper_backing *backing,
	u64 va, enum gpu_m4_drm_upper_class class)
{
	struct gpu_m4_drm_upper_lease *l, *other;
	u64 end;
	int ret;

	if (!u || !upper_backing_valid(backing) || backing->policy != GPU_M4_DRM_CPU_NC ||
	    class > GPU_M4_DRM_UPPER_SHARED_CACHED ||
	    va < GPU_M4_DRM_UPPER_START || (va & (UPPER_PAGE - 1)))
		return ERR_PTR(-EINVAL);
	lockdep_assert_held(u->config.lock);
	if (!u->ops.seed_locked)
		return ERR_PTR(-EOPNOTSUPP);
	if (u->dead || u->shutdown || u->error)
		return ERR_PTR(u->error ?: -ENODEV);
	end = u->config.normal_min_va;
	if (va >= end || backing->size > end - va || UPPER_PAGE > end - va - backing->size ||
	    upper_read(u->config.root.cpu, 2) != (u->config.middle.pa | 3))
		return ERR_PTR(-EPERM);
	ret = u->ops.seed_locked(u->config.session, backing, va, class);
	if (!ret)
		ret = upper_graph(u);
	if (ret)
		return ERR_PTR(upper_errno(ret));
	list_for_each_entry(other, &u->leases, link)
		if (upper_same(&other->backing, backing)) {
			if (!other->seeded || other->va != va || other->class != class)
				return ERR_PTR(-EEXIST);
			ret = gpu_m4_drm_upper_reference_locked(other, backing, GPU_M4_DRM_UPPER_NORMAL, class);
			return ret ? ERR_PTR(upper_errno(ret)) : other;
		}
	if (backing->size / UPPER_PAGE > u->config.max_pages - u->pages ||
	    u->leases_count == u->config.max_leases)
		return ERR_PTR(-ENOSPC);
	l = kzalloc_obj(*l);
	if (!l)
		return ERR_PTR(-ENOMEM);
	l->upper = u;
	l->backing = *backing;
	l->pages = backing->size / UPPER_PAGE;
	l->window = GPU_M4_DRM_UPPER_NORMAL;
	l->class = class;
	l->placement = GPU_M4_DRM_UPPER_EXISTING_ONLY;
	l->allocation_gates = GPU_M4_DRM_UPPER_EXISTING_CHILD;
	l->attrs = gpu_m4_drm_upper_class_attrs(class);
	l->va = va;
	l->root_value = u->config.middle.pa | 3;
	l->seeded = l->visible = l->published = true;
	l->pa = kcalloc(l->pages, sizeof(*l->pa), GFP_KERNEL);
	if (!l->pa) {
		ret = -ENOMEM;
		goto failed;
	}
	/* READY already retains the actual backing. Finish every fallible graph,
	 * native-page and authority check BEFORE obtaining this additional ref;
	 * rollback then frees only private metadata, never a final owner under lock.
	 */
	ret = upper_inventory(l, true);
	if (!ret)
		ret = upper_slots(l);
	if (!ret) {
		struct upper_child *guard = upper_find(u, ((va + backing->size) >> 25) & 2047);

		if (!guard || !guard->linked || guard->table.class != GPU_M4_DRM_UPPER_INHERITED_AP)
			ret = -EPERM;
	}
	if (!ret)
		ret = u->ops.seed_locked(u->config.session, backing, va, class);
	if (!ret)
		ret = backing->get_locked(backing->owner);
	if (ret)
		goto failed;
	l->held = l->charged = true;
	l->refs = 1;
	u->pages += l->pages;
	u->leases_count++;
	list_add_tail(&l->link, &u->leases);
	upper_reserve(u, l);
	u->revision++;
	return l;
failed:
	if (l->pa)
		upper_index_forget(l);
	kfree(l->pa);
	kfree(l);
	return ERR_PTR(upper_errno(ret));
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_import_seed_locked);

static int upper_trial_slots(struct gpu_m4_drm_upper_lease *l)
{
	u32 i;

	/* Include the guard: a trial cannot cross into an unwanted parent class.
	 * This is allocation/publication placement, not an exact-ref rewrite.
	 */
	for (i = 0; i <= l->pages; i++)
		if (!upper_placement(l->upper, l->va + (u64)i * UPPER_PAGE, l->placement))
			return -EEXIST;
	return 0;
}

int gpu_m4_drm_upper_reference_locked(struct gpu_m4_drm_upper_lease *l,
	const struct gpu_m4_drm_upper_backing *backing,
	enum gpu_m4_drm_upper_window window, enum gpu_m4_drm_upper_class class)
{
	int ret;

	if (!l || !upper_backing_valid(backing))
		return -EINVAL;
	lockdep_assert_held(l->upper->config.lock);
	if (l->upper->error || l->upper->shutdown || l->garbage ||
	    !upper_same(&l->backing, backing) || l->window != window || l->class != class)
		return -EPERM;
	if (l->refs == U32_MAX)
		return -EOVERFLOW;
	ret = upper_slots(l);
	if (!ret)
		l->refs++;
	return ret && l->visible ? upper_fault(l->upper, ret) : ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_reference_locked);

static bool upper_publication_proved(struct gpu_m4_drm_upper_lease *l);

int gpu_m4_drm_upper_check_lease_locked(struct gpu_m4_drm_upper_lease *l,
	const struct gpu_m4_drm_upper_backing *backing,
	enum gpu_m4_drm_upper_window window, enum gpu_m4_drm_upper_class class, u64 va)
{
	struct gpu_m4_drm_upper_lease *owned;
	bool member = false;
	int ret;

	if (!l || !upper_backing_valid(backing))
		return -EINVAL;
	lockdep_assert_held(l->upper->config.lock);
	if (l->upper->dead || l->upper->shutdown || l->upper->error)
		return l->upper->error ?: -ENODEV;
	/* The publication proof of this held published lease, exactly as a
	 * publication reuses it: its complete owned PTE/guard path at the
	 * CURRENT revision/root[2], and the owner's latest live health pass (the
	 * whole graph against the expected ledger) at that same revision. Every
	 * lease/table/ledger/membership change advances the revision. Identity,
	 * VA and class are still compared on every call.
	 */
	if (l->refs && l->held && l->published && l->visible && !l->garbage && l->va == va &&
	    upper_same(&l->backing, backing) && l->window == window && l->class == class &&
	    upper_publication_proved(l)) {
		l->upper->lease_reused++;
		return 0;
	}
	l->upper->lease_walks++;
	/* A lease leaves the owner's list only together with garbage (or at
	 * shutdown, refused above): kernel builds trust that invariant; host
	 * fixtures probe forged leases through the complete membership walk.
	 */
	if (upper_member_flag)
		member = !l->garbage;
	else
		list_for_each_entry(owned, &l->upper->leases, link)
			if (owned == l)
				member = true;
	if (!member || !l->refs || !l->held || !l->published || !l->visible ||
	    l->garbage || l->va != va ||
	    !upper_same(&l->backing, backing) || l->window != window || l->class != class)
		return -EPERM;
	/* Admitted held inventory: its exact owned path and guard, not the
	 * whole graph or every lease's PA inventory per backing.
	 */
	ret = upper_path(l, 0, l->pages);
	return ret ?: upper_guard(l);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_check_lease_locked);

u64 gpu_m4_drm_upper_va_locked(struct gpu_m4_drm_upper_lease *l)
{
	if (!l)
		return 0;
	lockdep_assert_held(l->upper->config.lock);
	return l->garbage ? 0 : l->va;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_va_locked);

static int upper_fault(struct gpu_m4_drm_upper *u, int error)
{
	if (!u->error) {
		u->error = error < 0 ? error : -EIO;
		u->revision++;
		u->ops.fault_locked(u->config.session, u->error);
	}
	return u->error;
}

static int upper_changed(struct gpu_m4_drm_upper_lease *l, void *cpu,
			 u64 pa, u32 index, u64 value, bool parent)
{
	struct gpu_m4_drm_upper *u = l->upper;
	struct upper_child *c = NULL;
	u64 previous;
	int ret;

	if (l->update_count >= l->update_capacity)
		return -EOVERFLOW;
	if (!parent)
		for (u32 i = 0; i < u->children && !c; i++)
			if (u->child[i].table.pa == pa)
				c = &u->child[i];
	/* The expected ledger's value being replaced (a cleared slot's PTE). */
	previous = parent ? u->middle_snapshot[index] : c ? c->expected[index] : 0;
	/* All owners were retained during acceptance. Pin uncertainty BEFORE the
	 * first potentially visible store, even if its later readback fails.
	 */
	l->visible = true;
	u->revision++;
	if (c)
		c->verified = false;
	upper_write(cpu, index, value);
	ret = u->ops.clean_locked(u->config.session, (u8 *)cpu + index * 8, 8);
	if (ret)
		return ret;
	ret = u->ops.barrier_locked(u->config.session);
	if (ret)
		return ret;
	if (upper_read(cpu, index) != value)
		return -EIO;
	ret = upper_root(u);
	if (ret)
		return ret;
	if (parent)
		u->middle_snapshot[index] = value;
	else if (c)
		c->expected[index] = value;
	else
		return -EPROTO;
	l->updates[l->update_count++] = (struct gpu_m4_drm_upper_update) {
		.table_cpu = cpu, .table_pa = pa, .index = index, .value = value,
		.previous = previous, .parent = parent,
	};
	return 0;
}

static void upper_publication_prove(struct gpu_m4_drm_upper_lease *l)
{
	l->proof_revision = l->upper->revision;
	l->proof_root = upper_read(l->upper->config.root.cpu, 2);
}

/* An unchanged, already published ordinary NC lease: its publication proof is
 * at the CURRENT owner revision and root[2], and the latest live health pass
 * (complete live graph bytes plus every lease's owned PTE/guard) succeeded at
 * that same revision/root. The live root words are rechecked here. Timestamp
 * window aliases and WB backings always take the full path.
 */
static bool upper_publication_proved(struct gpu_m4_drm_upper_lease *l)
{
	struct gpu_m4_drm_upper *u = l->upper;
	u64 root;

	if (!l->proof_revision || !l->published || !l->visible || !l->held ||
	    l->window != GPU_M4_DRM_UPPER_NORMAL || l->backing.policy != GPU_M4_DRM_CPU_NC ||
	    upper_root_recent(u) || !upper_root_value(l))
		return false;
	root = upper_read(u->config.root.cpu, 2);
	return l->proof_revision == u->revision && l->proof_root == root &&
		u->checked_revision == u->revision && u->checked_root == root;
}

int gpu_m4_drm_upper_publish_locked(struct gpu_m4_drm_upper_lease *l, bool trial)
{
	struct gpu_m4_drm_upper *u;
	u32 i, gates;
	int ret, release;

	if (!l)
		return -EINVAL;
	u = l->upper;
	lockdep_assert_held(u->config.lock);
	if (!l->refs || l->garbage || u->shutdown || u->error)
		return u->error ?: -ENODEV;
	if (l->published) {
		/* Reusing an existing timestamp alias still needs this job's WB
		 * cache owner/protection before a new firmware write can reach it.
		 * Every reuse ends in the backing's current-job readiness check.
		 */
		if (upper_publication_proved(l)) {
			u->publish_reused++;
		} else {
			ret = upper_slots_reuse(l);
			if (ret)
				return upper_fault(u, ret);
			upper_publication_prove(l);
			u->publish_walks++;
		}
		return l->backing.ready_locked(l->backing.owner, l->backing.offset,
			l->backing.size, l->class); /* Never rewrite. */
	}
	if (l->visible)
		return upper_fault(u, -EPROTO);
	/* Construction transition: recheck this lease's admitted oracles once.
	 * Its PA uniqueness is the index claim made at admission.
	 */
	ret = upper_slots(l);
	if (!ret)
		ret = upper_inventory(l, false);
	if (ret)
		return ret;
	ret = upper_trial_slots(l);
	if (ret)
		return ret;
	gates = l->window == GPU_M4_DRM_UPPER_TS ? GPU_M4_DRM_UPPER_TIMESTAMP_GATE : 0;
	for (i = 0; i < l->pages; i++) {
		struct upper_child *c = upper_find(u, ((l->va + (u64)i * UPPER_PAGE) >> 25) & 2047);

		gates |= c->linked ? GPU_M4_DRM_UPPER_EXISTING_CHILD : GPU_M4_DRM_UPPER_NEW_PARENT;
	}
	ret = u->ops.gate_locked(u->config.session, gates, trial);
	if (ret)
		return ret;
	ret = l->backing.ready_locked(l->backing.owner, l->backing.offset, l->backing.size, l->class);
	if (ret)
		return ret;
	ret = u->ops.handoff_get_locked(u->config.session, 100);
	if (ret)
		return upper_fault(u, ret); /* Failed lock transition is not a retry. */
	ret = u->ops.gate_locked(u->config.session, gates, trial);
	if (!ret)
		ret = upper_slots(l);
	if (!ret)
		ret = upper_trial_slots(l);
	if (ret)
		goto unlock;
	for (i = 0; i < l->pages; i++) {
		u64 va = l->va + (u64)i * UPPER_PAGE;
		u32 middle = (va >> 25) & 2047;
		struct upper_child *c = upper_find(u, middle);

		if (!c->linked) {
			/* Detached child must be wholly zero and cleaned BEFORE parent. */
			if (memchr_inv(c->table.cpu, 0, UPPER_PAGE)) {
				ret = -EPROTO;
				goto unlock;
			}
			ret = u->ops.clean_locked(u->config.session, c->table.cpu, UPPER_PAGE);
			if (!ret)
				ret = u->ops.barrier_locked(u->config.session);
			if (ret)
				goto unlock;
			ret = upper_changed(l, u->config.middle.cpu, u->config.middle.pa,
				middle, c->table.pa | 3, true);
			if (ret)
				goto unlock;
			c->linked = true;
		}
		ret = upper_changed(l, c->table.cpu, c->table.pa, (va >> 14) & 2047,
			l->pa[i] | l->attrs, false);
		if (ret)
			goto unlock;
	}
	ret = upper_slots(l);
unlock:
	release = u->ops.handoff_put_locked(u->config.session);
	if (ret || release)
		return upper_fault(u, ret ?: release);
	ret = u->ops.asid0_tlbi_locked(u->config.session);
	if (ret)
		return upper_fault(u, ret);
	ret = upper_slots(l);
	if (!ret)
		ret = u->ops.commit_locked(u->config.session, l->updates, l->update_count);
	if (ret)
		return upper_fault(u, ret);
	l->published = true;
	u->revision++;
	upper_publication_prove(l);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_publish_locked);

static int upper_release_fault(struct gpu_m4_drm_upper *u, int error)
{
	u->release_failures++;
	return upper_fault(u, error);
}

int gpu_m4_drm_upper_release_locked(struct gpu_m4_drm_upper_lease *l,
	const struct gpu_m4_drm_upper_backing *backing)
{
	struct gpu_m4_drm_upper *u;
	int ret, release;
	u32 i;

	if (!l || !upper_backing_valid(backing))
		return -EINVAL;
	u = l->upper;
	lockdep_assert_held(u->config.lock);
	if (u->dead || u->shutdown || u->error)
		return u->error ?: -ENODEV;
	/* The leaves are global (nG clear): only a by-VA invalidation removes
	 * them. Without one, refuse before any store; never ASID-only.
	 */
	if (!u->ops.release_commit_locked || !u->ops.asid0_range_tlbi_locked)
		return -EOPNOTSUPP;
	/* Session-lifetime seeds and timestamp aliases are never released here;
	 * another holder (refs) still names this VA.
	 */
	if (l->garbage || l->seeded || !l->held || l->window != GPU_M4_DRM_UPPER_NORMAL ||
	    !upper_same(&l->backing, backing))
		return -EPERM;
	if (l->refs != 1 || !l->published || !l->visible)
		return -EBUSY;
	/* The complete owned path and guard are still exactly as published. */
	ret = upper_slots(l);
	if (ret)
		return upper_release_fault(u, ret);
	ret = u->ops.handoff_get_locked(u->config.session, 100);
	if (ret)
		return upper_release_fault(u, ret); /* Failed lock transition is not a retry. */
	/* The publication updates were committed; reuse their bounded record. */
	l->update_count = 0;
	for (i = 0; !ret && i < l->pages; i++) {
		u64 va = l->va + (u64)i * UPPER_PAGE;
		struct upper_child *c = upper_find(u, (va >> 25) & 2047);

		ret = upper_changed(l, c->table.cpu, c->table.pa, (va >> 14) & 2047, 0, false);
	}
	/* Every live table equals the ledger, these slots now zero included. */
	if (!ret)
		ret = upper_graph(u);
	release = u->ops.handoff_put_locked(u->config.session);
	if (ret || release)
		return upper_release_fault(u, ret ?: release);
	/* Every released page, after the cleaned stores; complete on success. */
	ret = u->ops.asid0_range_tlbi_locked(u->config.session, l->va, l->backing.size);
	if (!ret)
		ret = upper_graph(u);
	if (!ret)
		ret = u->ops.release_commit_locked(u->config.session, l->updates, l->update_count);
	if (ret)
		return upper_release_fault(u, ret);
	/* No translation remains: the VA and guard are free now; the PA claims
	 * and the backing reference drop at reap, outside every lock.
	 */
	l->refs = 0;
	l->visible = l->published = false;
	l->proof_revision = 0;
	list_del(&l->link);
	upper_unreserve(u, l);
	l->garbage = true;
	list_add_tail(&l->link, &u->garbage);
	u->revision++;
	u->released++;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_release_locked);

void gpu_m4_drm_upper_release_stats_locked(struct gpu_m4_drm_upper *u, u64 *released,
	u64 *failures)
{
	*released = *failures = 0;
	if (!u)
		return;
	lockdep_assert_held(u->config.lock);
	*released = u->released;
	*failures = u->release_failures;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_release_stats_locked);

void gpu_m4_drm_upper_usage_locked(struct gpu_m4_drm_upper *u, u32 *leases, u32 *pages)
{
	*leases = *pages = 0;
	if (!u)
		return;
	lockdep_assert_held(u->config.lock);
	*leases = u->leases_count;
	*pages = u->pages;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_usage_locked);

u64 gpu_m4_drm_upper_revision_locked(struct gpu_m4_drm_upper *u)
{
	if (!u)
		return 0;
	lockdep_assert_held(u->config.lock);
	return u->revision;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_revision_locked);

void gpu_m4_drm_upper_publish_stats_locked(struct gpu_m4_drm_upper *u, u64 *walks, u64 *reused)
{
	*walks = *reused = 0;
	if (!u)
		return;
	lockdep_assert_held(u->config.lock);
	*walks = u->publish_walks;
	*reused = u->publish_reused;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_publish_stats_locked);

void gpu_m4_drm_upper_lease_stats_locked(struct gpu_m4_drm_upper *u, u64 *walks, u64 *reused)
{
	*walks = *reused = 0;
	if (!u)
		return;
	lockdep_assert_held(u->config.lock);
	*walks = u->lease_walks;
	*reused = u->lease_reused;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_lease_stats_locked);

void gpu_m4_drm_upper_choose_stats_locked(struct gpu_m4_drm_upper *u,
	struct gpu_m4_drm_upper_choose_stats *stats)
{
	*stats = (struct gpu_m4_drm_upper_choose_stats) { 0 };
	if (!u)
		return;
	lockdep_assert_held(u->config.lock);
	*stats = u->choose;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_choose_stats_locked);

int gpu_m4_drm_upper_put_locked(struct gpu_m4_drm_upper_lease *l)
{
	if (!l)
		return -EINVAL;
	lockdep_assert_held(l->upper->config.lock);
	if (!l->refs || l->garbage)
		return -EINVAL;
	if (--l->refs || l->visible)
		return 0; /* Possibly visible baseline remains until qualified shutdown. */
	list_del(&l->link);
	upper_unreserve(l->upper, l);
	l->garbage = true;
	list_add_tail(&l->link, &l->upper->garbage);
	l->upper->revision++;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_put_locked);

int gpu_m4_drm_upper_verify_locked(struct gpu_m4_drm_upper *u)
{
	struct gpu_m4_drm_upper_lease *l, *failed = NULL;
	u64 indexed = 0;
	int ret;

	if (!u)
		return -EINVAL;
	lockdep_assert_held(u->config.lock);
	if (u->dead)
		return -ENODEV;
	u->checked_revision = 0;
	u->why_line = 0;
	ret = upper_graph(u);
	if (ret)
		goto report;
	list_for_each_entry(l, &u->leases, link) {
		ret = upper_slots_check(l);
		if (!ret)
			ret = upper_inventory(l, false);
		if (ret) {
			failed = l;
			goto report;
		}
		indexed += l->indexed;
	}
	/* Every lease page maps to itself; exact claim count then proves the
	 * index holds no stale or duplicate PA claim (garbage until reaped).
	 */
	list_for_each_entry(l, &u->garbage, link)
		indexed += l->indexed;
	if (u->index_live != indexed + 2 + u->children) {
		ret = UPPER_WHY(u, -EPROTO);
		goto report;
	}
	u->full_scans++;
	/* Backing oracles remain mandatory. Check the whole graph again after
	 * them, including paths outside the last lease, without caching a grant
	 * across this call or changing any individual publication entrance.
	 */
	ret = upper_graph(u);
	if (!ret) {
		u->checked_revision = u->revision;
		u->checked_root = upper_read(u->config.root.cpu, 2);
		return 0;
	}
report:
	upper_report(u, 1, "verify", ret, failed, indexed);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_verify_locked);

int gpu_m4_drm_upper_health_locked(struct gpu_m4_drm_upper *u)
{
	struct gpu_m4_drm_upper_lease *l;
	u64 root;
	int ret;

	if (!u)
		return -EINVAL;
	lockdep_assert_held(u->config.lock);
	/* Live authority: a latched owner failure or shutdown is never healthy,
	 * and never installs a cached success (unlike the full diagnostic).
	 */
	if (u->error)
		return u->error;
	if (u->dead || u->shutdown)
		return -ENODEV;
	u->why_line = 0;
	ret = upper_graph(u);
	if (ret) {
		u->checked_revision = 0;
		upper_report(u, 2, "health graph", ret, NULL, 0);
		return ret;
	}
	root = upper_read(u->config.root.cpu, 2);
	if (u->checked_revision == u->revision && u->checked_root == root)
		return 0;
	list_for_each_entry(l, &u->leases, link) {
		ret = upper_slots_check(l);
		if (ret) {
			u->checked_revision = 0;
			upper_report(u, 4, "health slots", ret, l, 0);
			return ret;
		}
	}
	u->slot_scans++;
	u->checked_revision = u->revision;
	u->checked_root = root;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_health_locked);

int gpu_m4_drm_upper_walk_locked(struct gpu_m4_drm_upper *u, u64 va,
			       u64 *pa, u64 *attrs)
{
	struct gpu_m4_drm_upper_lease *l;
	int ret;

	if (!u || !pa || !attrs || va < GPU_M4_DRM_UPPER_START || va >= GPU_M4_DRM_UPPER_END)
		return -EINVAL;
	lockdep_assert_held(u->config.lock);
	/* Live authority, unlike the full diagnostic verifier: a latched owner
	 * failure or shutdown is never reported as healthy or translated.
	 */
	if (u->error)
		return u->error;
	if (u->dead || u->shutdown)
		return -ENODEV;
	/* Every u->leases member's reservation is in the interval index: a
	 * single published owner of va is found in O(log n). Several candidates
	 * (overlapping seeds) keep the list order below.
	 */
	{
		struct gpu_m4_drm_upper_lease *found = NULL;
		struct upper_extent *e;
		u32 matches = 0;

		for (e = upper_extent_iter_first(&u->reserved, va, va); e;
		     e = upper_extent_iter_next(e, va, va)) {
			struct gpu_m4_drm_upper_lease *c =
				container_of(e, struct gpu_m4_drm_upper_lease, reserved);

			if (c->published && va >= c->va && va - c->va < c->backing.size) {
				found = c;
				matches++;
			}
		}
		/* No member reserves va: no published lease translates it (the
		 * list walk below could only find the same members).
		 */
		if (!matches && !upper_extent_iter_first(&u->reserved, va, va)) {
			ret = upper_root(u);
			return ret ?: -ENOENT;
		}
		if (matches == 1) {
			u64 offset = va - found->va;

			ret = upper_path(found, offset / UPPER_PAGE, 1);
			if (!ret)
				ret = upper_guard(found);
			if (ret)
				return ret;
			*pa = found->pa[offset / UPPER_PAGE] | (offset & (UPPER_PAGE - 1));
			*attrs = found->attrs;
			return 0;
		}
	}
	list_for_each_entry(l, &u->leases, link)
		if (l->published && va >= l->va && va - l->va < l->backing.size) {
			u64 offset = va - l->va;

			/* Only this page's owned root/parent/leaf and the lease's
			 * trailing guard, not every lease or the global graph.
			 */
			ret = upper_path(l, offset / UPPER_PAGE, 1);
			if (!ret)
				ret = upper_guard(l);
			if (ret)
				return ret;
			*pa = l->pa[offset / UPPER_PAGE] | (offset & (UPPER_PAGE - 1));
			*attrs = l->attrs;
			return 0;
		}
	ret = upper_root(u);
	return ret ?: -ENOENT;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_walk_locked);

int gpu_m4_drm_upper_table_locked(struct gpu_m4_drm_upper *u, u32 middle,
				struct gpu_m4_drm_upper_table *table)
{
	struct upper_child *c;
	int ret;

	if (!u || !table || middle >= UPPER_ENTRIES)
		return -EINVAL;
	lockdep_assert_held(u->config.lock);
	if (u->dead)
		return -ENODEV;
	c = upper_find(u, middle);
	if (!c)
		return -ENOENT;
	if (c->table.class != GPU_M4_DRM_UPPER_INHERITED_AP)
		return -EPERM;
	if (c->page) {
		ret = c->table.cpu == page_address(c->page) &&
			c->table.pa == page_to_phys(c->page) && upper_pa(c->table.pa) ? 0 : -EPROTO;
	} else {
		ret = upper_table(u, &c->table);
	}
	if (ret)
		return ret;
	*table = c->table;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_table_locked);

int gpu_m4_drm_upper_error_locked(struct gpu_m4_drm_upper *u)
{
	lockdep_assert_held(u->config.lock);
	return u->error;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_error_locked);

int gpu_m4_drm_upper_shutdown_locked(struct gpu_m4_drm_upper *u)
{
	int ret;

	if (!u)
		return -EINVAL;
	lockdep_assert_held(u->config.lock);
	if (u->dead || u->shutdown)
		return -EALREADY;
	ret = u->ops.shutdown_locked(u->config.session);
	if (!ret) {
		u->shutdown = true;
		u->revision++;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_shutdown_locked);

static void upper_free_lease(struct gpu_m4_drm_upper_lease *l)
{
	if (l->held)
		l->backing.put(l->backing.owner);
	kfree(l->updates);
	kfree(l->pa);
	kfree(l);
}

void gpu_m4_drm_upper_reap(struct gpu_m4_drm_upper *u)
{
	struct gpu_m4_drm_upper_lease *l;

	if (!u)
		return;
	mutex_lock(u->config.lock);
	u->reapers++;
	while (!list_empty(&u->garbage)) {
		u32 pages;
		bool charged;

		l = list_first_entry(&u->garbage, struct gpu_m4_drm_upper_lease, link);
		list_del(&l->link);
		/* Actual reap releases this lease's PA claims under the owner lock. */
		upper_index_forget(l);
		pages = l->pages;
		charged = l->charged;
		mutex_unlock(u->config.lock);
		upper_free_lease(l);
		mutex_lock(u->config.lock);
		if (charged) {
			u->pages -= pages;
			u->leases_count--;
		}
	}
	u->reapers--;
	mutex_unlock(u->config.lock);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_reap);

int gpu_m4_drm_upper_destroy(struct gpu_m4_drm_upper *u)
{
	struct gpu_m4_drm_upper_lease *l;
	u32 i;

	if (!u)
		return -EINVAL;
	mutex_lock(u->config.lock);
	if (u->dead || u->reapers) {
		mutex_unlock(u->config.lock);
		return -EBUSY;
	}
	list_for_each_entry(l, &u->leases, link)
		if (l->refs || (l->visible && !u->shutdown)) {
			mutex_unlock(u->config.lock);
			return -EBUSY;
		}
	u->dead = true;
	while (!list_empty(&u->leases)) {
		l = list_first_entry(&u->leases, struct gpu_m4_drm_upper_lease, link);
		list_del(&l->link);
		list_add_tail(&l->link, &u->garbage);
	}
	mutex_unlock(u->config.lock);
	/* No other owner API may run after successful destroy begins. */
	while (!list_empty(&u->garbage)) {
		l = list_first_entry(&u->garbage, struct gpu_m4_drm_upper_lease, link);
		list_del(&l->link);
		upper_free_lease(l);
	}
	for (i = 0; i < u->children; i++) {
		if (u->child[i].page)
			__free_page(u->child[i].page);
		kfree(u->child[i].expected);
	}
	u->ops.session_put(u->config.session);
	kvfree(u->index);
	kfree(u->child);
	kfree(u);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_upper_destroy);

MODULE_DESCRIPTION("Retained Apple M4 firmware upper alias owner");
MODULE_LICENSE("GPL");
