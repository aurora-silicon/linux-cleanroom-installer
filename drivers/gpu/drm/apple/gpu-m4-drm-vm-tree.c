// SPDX-License-Identifier: GPL-2.0-only
/*
 * Persistent per-VM lower page-table tree (see gpu-m4-drm-vm-tree.h).
 *
 * S3a/S3b: the provider builds one per VM as a shadow of the immutable version
 * path (user and driver mappings); it is never installed, so its TLBI
 * callbacks issue nothing.
 *
 * S3c (config.authority): the VM's installed tree; real TLBI while resident,
 * guards, deferred GEM/VM close and named-range lookups.
 */
#include <linux/atomic.h>
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/interval_tree_generic.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/rbtree.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/string.h>
#include <asm/barrier.h>
#include <asm/cacheflush.h>

#include "gpu-m4-drm-vm-tree.h"

/* One table page; root and level 1 keep their children by slot. */
struct gpu_m4_vm_table {
	struct page *page;
	void *cpu;
	u64 pa, base;
	struct gpu_m4_vm_table **child;	/* root: 64, level 1: 2048, level 2: none */
	struct gpu_m4_vm_table *parent;
	struct gpu_m4_vm_table *staged;	/* transaction list of new tables */
	u32 slot, used;			/* slot in parent; valid words (level 2) */
	u8 level;
	bool fresh;			/* allocated by the open transaction */
};

/* A mapped object: exactly one held reference while it has extents. */
struct gpu_m4_vm_object {
	struct rb_node node;
	void *object;
	struct list_head extents, link;	/* link: spare pool or deferred put */
	u32 count;
};

/* One mapping [start, last] (inclusive, page granular). */
struct gpu_m4_vm_extent {
	struct rb_node rb;
	u64 start, last, subtree_last;
	struct gpu_m4_vm_object *owner;	/* NULL: a driver extent without object */
	const void *identity;		/* backing identity reported by lookups */
	u64 offset;
	u64 attrs;			/* DRIVER leaf attribute override, 0: from access */
	u32 access, flags, cpu_policy;
	struct list_head link;		/* owner->extents, spare pool or removal list */
	struct list_head deferred;	/* tree->deferred while CLOSED */
};

#define GPU_M4_VM_EXTENT_START(e) ((e)->start)
#define GPU_M4_VM_EXTENT_LAST(e) ((e)->last)
INTERVAL_TREE_DEFINE(struct gpu_m4_vm_extent, rb, u64, subtree_last, GPU_M4_VM_EXTENT_START,
		     GPU_M4_VM_EXTENT_LAST, static __maybe_unused, gpu_m4_vm_extent)

#define GPU_M4_VM_TREE_CHANGES 64U

struct gpu_m4_vm_tree {
	struct mutex lock;
	/* Residency and ASID (S3c); nested inside lock, taken alone by the
	 * binder under the GPU mutex. Held across a removal's TLBI decision and
	 * instructions, so an installation cannot interleave with them.
	 */
	struct mutex binding_lock;
	struct gpu_m4_vm_tree_config config;
	struct rb_root_cached extents;
	struct rb_root objects;
	struct gpu_m4_vm_table *root;	/* stable for the tree's lifetime */
	u64 leaves, driver_leaves;	/* valid leaf words, of which DRIVER */
	u32 tables, asid;
	bool desynced, faulted;
	/* S3c: accepted-job guards, CLOSED extents waiting for them, objects a
	 * faulted tree keeps (I2), residency (binding_lock) and VM close.
	 */
	atomic_t guards;
	/* Advanced under lock before every possible leaf/extent change: an
	 * unchanged value proves the leaves did not change since it was read.
	 */
	u64 revision;
	/* The span each recent revision changed (change_lock; any VA when not
	 * a bind): a proof at revision r stays valid while every later one
	 * is logged here and misses its span.
	 */
	spinlock_t change_lock;
	struct gpu_m4_vm_tree_change {
		u64 revision, first, last;
	} changes[GPU_M4_VM_TREE_CHANGES];
	struct list_head deferred;
	struct list_head retained;
	bool resident, closed, deferred_pending;
};

/* Preacquired resources of one batch. */
struct gpu_m4_vm_tree_txn {
	struct gpu_m4_vm_table *staged;
	struct list_head extents, objects;
	u32 tables;
};

/* One removal: the cleared span, its removed extents and dead objects. */
struct gpu_m4_vm_removal {
	u64 first, last, pages;
	struct list_head extents, dead;
};

/* REMOVE-DEBUG: shadow-tree mode (0 off until the S3c cutover, 1 build,
 * 2 build and compare) and cumulative statistics.
 */
static unsigned int gpu_m4_vm_tree_mode_param;
module_param_named(mode, gpu_m4_vm_tree_mode_param, uint, 0644);
MODULE_PARM_DESC(mode, "REMOVE-DEBUG shadow per-VM page-table tree, never installed: 0 off, 1 mirror VM_BIND and the driver mappings (new VMs), 2 also compare the complete image with the composed version after every change, composition and binding");

enum {
	GPU_M4_VM_TREE_STAT_TREES,
	GPU_M4_VM_TREE_STAT_BATCHES,
	GPU_M4_VM_TREE_STAT_OPS,
	GPU_M4_VM_TREE_STAT_MAPPED,
	GPU_M4_VM_TREE_STAT_UNMAPPED,
	GPU_M4_VM_TREE_STAT_TABLES,
	GPU_M4_VM_TREE_STAT_TLBI_VA,
	GPU_M4_VM_TREE_STAT_TLBI_RANGE,
	GPU_M4_VM_TREE_STAT_TLBI_ASID,
	GPU_M4_VM_TREE_STAT_REFUSED,
	GPU_M4_VM_TREE_STAT_DESYNCS,
	GPU_M4_VM_TREE_STAT_COMPARES,
	GPU_M4_VM_TREE_STAT_MISMATCHES,
	GPU_M4_VM_TREE_STAT_FAULTS,
	GPU_M4_VM_TREE_STAT_DRIVER_ADDED,	/* DRIVER extents created */
	GPU_M4_VM_TREE_STAT_DRIVER_REMOVED,	/* DRIVER extents removed */
	GPU_M4_VM_TREE_STAT_ORACLE_COMPARES,	/* composed-image compares */
	GPU_M4_VM_TREE_STAT_ORACLE_MISMATCHES,
	GPU_M4_VM_TREE_STAT_ORACLE_SKIPPED,	/* expected image not current */
	GPU_M4_VM_TREE_STAT_RAM_REFUSED,	/* table page failed RAM admission */
	/* S3c authority */
	GPU_M4_VM_TREE_STAT_TLBI_SKIPPED,	/* removal runs of a non-resident tree */
	GPU_M4_VM_TREE_STAT_INSTALLS,		/* install_begin */
	GPU_M4_VM_TREE_STAT_EVICTIONS,
	GPU_M4_VM_TREE_STAT_GUARDS,		/* guards taken */
	GPU_M4_VM_TREE_STAT_GUARD_WAITS,	/* GEM/VM closes deferred by a guard */
	GPU_M4_VM_TREE_STAT_DEFERRED_CLOSES,	/* extents marked CLOSED */
	GPU_M4_VM_TREE_STAT_DEFERRED_REMOVED,	/* CLOSED extents removed */
	GPU_M4_VM_TREE_STAT_COVERS,		/* named-range validations */
	GPU_M4_VM_TREE_STAT_COVERS_REFUSED,
	GPU_M4_VM_TREE_STAT_RETAINED,		/* references a faulted tree kept */
	GPU_M4_VM_TREE_STATS,
};

static const char *const gpu_m4_vm_tree_stat_name[GPU_M4_VM_TREE_STATS] = {
	"trees", "batches", "ops", "mapped", "unmapped", "tables", "tlbi_va", "tlbi_range",
	"tlbi_asid", "refused", "desyncs", "compares", "mismatches", "faults", "driver_added",
	"driver_removed", "oracle_compares", "oracle_mismatches", "oracle_skipped", "ram_refused",
	"tlbi_skipped", "installs", "evictions", "guards", "guard_waits", "deferred_closes",
	"deferred_removed", "covers", "covers_refused", "retained",
};

static atomic64_t gpu_m4_vm_tree_stat[GPU_M4_VM_TREE_STATS];

static void gpu_m4_vm_tree_count(u32 stat, u64 value)
{
	atomic64_add(value, &gpu_m4_vm_tree_stat[stat]);
}

static int __maybe_unused gpu_m4_vm_tree_stats_get(char *buffer, const struct kernel_param *kp __always_unused)
{
	int at = 0;

	for (u32 i = 0; i < GPU_M4_VM_TREE_STATS; i++)
		at += scnprintf(buffer + at, PAGE_SIZE - at, "%s%s=%lld", i ? " " : "",
				gpu_m4_vm_tree_stat_name[i],
				(long long)atomic64_read(&gpu_m4_vm_tree_stat[i]));
	at += scnprintf(buffer + at, PAGE_SIZE - at, "\n");
	return at;
}

/* REMOVE-DEBUG: debugfs gpu_m4_drm_vm_tree/stats (live counters are not module parameters). */

unsigned int gpu_m4_vm_tree_mode(void)
{
	return READ_ONCE(gpu_m4_vm_tree_mode_param);
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_mode);

static void gpu_m4_vm_tree_shadow_clean(void *cookie __always_unused, void *start, size_t bytes)
{
	dcache_clean_poc((unsigned long)start, (unsigned long)start + bytes);
}

static void gpu_m4_vm_tree_shadow_dsb_osh(void *cookie __always_unused)
{
	dsb(osh);
}

static void gpu_m4_vm_tree_shadow_dsb_sy(void *cookie __always_unused)
{
	dsb(sy);
}

/* Never installed: there is no translation of this tree to invalidate. */
static void gpu_m4_vm_tree_shadow_tlbi(void *cookie __always_unused, u64 operand __always_unused)
{
}

const struct gpu_m4_vm_tree_ops gpu_m4_vm_tree_shadow_ops = {
	.clean = gpu_m4_vm_tree_shadow_clean,
	.dsb_osh = gpu_m4_vm_tree_shadow_dsb_osh,
	.dsb_sy = gpu_m4_vm_tree_shadow_dsb_sy,
	.tlbi_va = gpu_m4_vm_tree_shadow_tlbi,
	.tlbi_range = gpu_m4_vm_tree_shadow_tlbi,
	.tlbi_asid = gpu_m4_vm_tree_shadow_tlbi,
};
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_shadow_ops);

/*
 * Installed (S3c) maintenance: the tree's root is in TTBAT. Operands are the
 * VM's user ASID with the answers' encodings (gpu-m4-drm-uat-format.h):
 * VAE1OS for one 16 KiB page, RVAE1OS (TG 16 KiB, TTL 0) for a run,
 * ASIDE1OS only for teardown. The host tests define GPU_M4_VM_TREE_TLBI to
 * record the instruction and operand.
 */
#ifndef GPU_M4_VM_TREE_TLBI
#define GPU_M4_VM_TREE_TLBI(insn, operand) \
	asm volatile(".arch armv8.4-a\n\ttlbi " insn ", %0" : : "r" (operand) : "memory")
#endif

static void gpu_m4_vm_tree_installed_tlbi_va(void *cookie __always_unused, u64 operand)
{
	GPU_M4_VM_TREE_TLBI("vae1os", operand);
}

static void gpu_m4_vm_tree_installed_tlbi_range(void *cookie __always_unused, u64 operand)
{
	GPU_M4_VM_TREE_TLBI("rvae1os", operand);
}

static void gpu_m4_vm_tree_installed_tlbi_asid(void *cookie __always_unused, u64 operand)
{
	GPU_M4_VM_TREE_TLBI("aside1os", operand);
}

const struct gpu_m4_vm_tree_ops gpu_m4_vm_tree_installed_ops = {
	.clean = gpu_m4_vm_tree_shadow_clean,
	.dsb_osh = gpu_m4_vm_tree_shadow_dsb_osh,
	.dsb_sy = gpu_m4_vm_tree_shadow_dsb_sy,
	.tlbi_va = gpu_m4_vm_tree_installed_tlbi_va,
	.tlbi_range = gpu_m4_vm_tree_installed_tlbi_range,
	.tlbi_asid = gpu_m4_vm_tree_installed_tlbi_asid,
};
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_installed_ops);

/* Table words are little-endian u64 on this little-endian-only driver. */
static u64 gpu_m4_vm_word(const struct gpu_m4_vm_table *t, u32 slot)
{
	return READ_ONCE(((const u64 *)t->cpu)[slot]);
}

static void gpu_m4_vm_word_set(struct gpu_m4_vm_table *t, u32 slot, u64 value)
{
	WRITE_ONCE(((u64 *)t->cpu)[slot], value);
}

static void gpu_m4_vm_clean_words(struct gpu_m4_vm_tree *tree, struct gpu_m4_vm_table *t,
				  u32 first, u32 last)
{
	tree->config.ops->clean(tree->config.cookie, (u64 *)t->cpu + first,
				(size_t)(last - first + 1) * sizeof(u64));
}

/*
 * The RAM admission gpu-m4-drm-uat.c makes for its own table pages: the whole
 * 16 KiB page is System RAM inside the PA mask and every CPU page of it has a
 * valid, unreserved PFN. Only new tables are admitted (rarely), so the live
 * resource query is used without the builder's boot-topology snapshot.
 */
static int gpu_m4_vm_tree_ram(u64 pa)
{
	if (!pa || (pa & ~GPU_M4_PA_MASK) || !gpu_m4_drm_aligned(pa) ||
	    PAGE_SIZE > GPU_M4_DRM_PAGE_SIZE ||
	    region_intersects(pa, GPU_M4_DRM_PAGE_SIZE, IORESOURCE_SYSTEM_RAM,
			      IORES_DESC_NONE) != REGION_INTERSECTS)
		return -EPERM;
	for (u64 offset = 0; offset < GPU_M4_DRM_PAGE_SIZE; offset += PAGE_SIZE) {
		unsigned long pfn = (pa + offset) >> PAGE_SHIFT;

		if (!pfn_valid(pfn) || PageReserved(pfn_to_page(pfn)))
			return -EPERM;
	}
	return 0;
}

/* A new zero table page that passed RAM admission, or an ERR_PTR. */
static struct gpu_m4_vm_table *gpu_m4_vm_table_alloc(u8 level, u64 base)
{
	struct gpu_m4_vm_table *t = kzalloc(sizeof(*t), GFP_KERNEL);
	int ret = -ENOMEM;

	if (!t)
		return ERR_PTR(-ENOMEM);
	if (level < 2) {
		t->child = kvcalloc(level ? GPU_M4_UAT_ENTRIES : GPU_M4_UAT_ROOT_ENTRIES,
				    sizeof(*t->child), GFP_KERNEL);
		if (!t->child)
			goto fail;
	}
	t->page = alloc_pages(GFP_KERNEL | __GFP_ZERO, get_order(GPU_M4_DRM_PAGE_SIZE));
	if (!t->page)
		goto fail;
	t->cpu = page_address(t->page);
	t->pa = page_to_phys(t->page);
	ret = gpu_m4_vm_tree_ram(t->pa);
	if (ret)
		gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_RAM_REFUSED, 1);
	else if (!t->cpu || ((unsigned long)t->cpu & (GPU_M4_DRM_PAGE_SIZE - 1)) ||
		 memchr_inv(t->cpu, 0, GPU_M4_DRM_PAGE_SIZE))
		ret = -EPROTO;
	if (ret) {
		__free_pages(t->page, get_order(GPU_M4_DRM_PAGE_SIZE));
		goto fail;
	}
	t->level = level;
	t->base = base;
	return t;
fail:
	kvfree(t->child);
	kfree(t);
	return ERR_PTR(ret);
}

static void gpu_m4_vm_table_free(struct gpu_m4_vm_table *t)
{
	__free_pages(t->page, get_order(GPU_M4_DRM_PAGE_SIZE));
	kvfree(t->child);
	kfree(t);
}

static struct gpu_m4_vm_table *gpu_m4_vm_tree_l2(const struct gpu_m4_vm_tree *tree, u64 va)
{
	const struct gpu_m4_vm_table *l1 = tree->root->child[gpu_m4_uat_fmt_slot(va, 0)];

	return l1 ? l1->child[gpu_m4_uat_fmt_slot(va, 1)] : NULL;
}

static struct gpu_m4_vm_object *gpu_m4_vm_tree_find(const struct gpu_m4_vm_tree *tree,
						    const void *object)
{
	struct rb_node *node = tree->objects.rb_node;

	while (node) {
		struct gpu_m4_vm_object *o = rb_entry(node, struct gpu_m4_vm_object, node);

		if ((unsigned long)object < (unsigned long)o->object)
			node = node->rb_left;
		else if ((unsigned long)object > (unsigned long)o->object)
			node = node->rb_right;
		else
			return o;
	}
	return NULL;
}

static void gpu_m4_vm_tree_link_object(struct gpu_m4_vm_tree *tree, struct gpu_m4_vm_object *o)
{
	struct rb_node **link = &tree->objects.rb_node, *parent = NULL;

	while (*link) {
		struct gpu_m4_vm_object *at = rb_entry(*link, struct gpu_m4_vm_object, node);

		parent = *link;
		link = (unsigned long)o->object < (unsigned long)at->object ?
			&parent->rb_left : &parent->rb_right;
	}
	rb_link_node(&o->node, parent, link);
	rb_insert_color(&o->node, &tree->objects);
}

static bool gpu_m4_vm_overlap(u64 start, u64 end, u64 first, u64 limit)
{
	return first < limit && start < limit && first < end;
}

/* Static shape of one op; nothing is acquired. */
static int gpu_m4_vm_tree_check(const struct gpu_m4_vm_tree *tree,
				const struct gpu_m4_vm_tree_op *op, bool driver)
{
	const struct gpu_m4_vm_tree_config *c = &tree->config;
	u32 known = GPU_M4_VM_TREE_UNBIND | GPU_M4_VM_TREE_SINGLE_PAGE | GPU_M4_VM_TREE_DRIVER;
	bool single = op->flags & GPU_M4_VM_TREE_SINGLE_PAGE;
	u64 end, pages, first;

	if ((op->flags & ~known) || !!(op->flags & GPU_M4_VM_TREE_DRIVER) != driver ||
	    !op->size || !gpu_m4_drm_aligned(op->va) || !gpu_m4_drm_aligned(op->size) ||
	    op->va > U64_MAX - op->size)
		return -EINVAL;
	end = op->va + op->size;
	if (op->va < c->low || end > c->high)
		return -EINVAL;
	if (!driver && (gpu_m4_vm_overlap(op->va, end, c->kernel_start, c->kernel_end) ||
			gpu_m4_vm_overlap(op->va, end, c->reserved[0][0], c->reserved[0][1]) ||
			gpu_m4_vm_overlap(op->va, end, c->reserved[1][0], c->reserved[1][1])))
		return -EADDRINUSE;
	if (op->attrs && (!driver || (op->attrs & GPU_M4_PA_MASK) || !(op->attrs & 1)))
		return -EINVAL;
	if (op->flags & GPU_M4_VM_TREE_UNBIND)
		return op->object || op->pa || op->offset || single ? -EINVAL : 0;
	if (!gpu_m4_drm_vm_leaf_attrs(op->access) || !op->pa || (!op->object && !driver) ||
	    !gpu_m4_drm_aligned(op->offset) || !gpu_m4_drm_aligned(op->object_size) ||
	    op->offset >= op->object_size ||
	    (single ? GPU_M4_DRM_PAGE_SIZE : op->size) > op->object_size - op->offset)
		return -EINVAL;
	pages = single ? 1 : op->size / GPU_M4_DRM_PAGE_SIZE;
	first = op->offset / GPU_M4_DRM_PAGE_SIZE;
	for (u64 i = 0; i < pages; i++) {
		u64 pa = op->pa[first + i];

		if (!pa || !gpu_m4_drm_aligned(pa) || (pa & ~GPU_M4_PA_MASK))
			return -EINVAL;
	}
	return 0;
}

/* Existing child, else a new zero table linked in software only. */
static struct gpu_m4_vm_table *gpu_m4_vm_tree_ensure(struct gpu_m4_vm_tree *tree,
	struct gpu_m4_vm_tree_txn *txn, struct gpu_m4_vm_table *parent, u8 level, u64 base,
	int *error)
{
	u32 slot = gpu_m4_uat_fmt_slot(base, level - 1);
	struct gpu_m4_vm_table *t = parent->child[slot];

	if (t)
		return t;
	if (tree->tables + txn->tables >= tree->config.max_tables) {
		*error = -ENOSPC;
		return NULL;
	}
	t = gpu_m4_vm_table_alloc(level, base);
	if (IS_ERR(t)) {
		*error = PTR_ERR(t);
		return NULL;
	}
	t->parent = parent;
	t->slot = slot;
	t->fresh = true;
	t->staged = txn->staged;
	txn->staged = t;
	txn->tables++;
	parent->child[slot] = t;
	return t;
}

static int gpu_m4_vm_tree_tables_for(struct gpu_m4_vm_tree *tree, struct gpu_m4_vm_tree_txn *txn,
				     u64 va, u64 size)
{
	u64 last = (va + size - 1) >> gpu_m4_uat_fmt_shift(1);
	int error = 0;

	for (u64 block = va >> gpu_m4_uat_fmt_shift(1); block <= last; block++) {
		u64 base = block << gpu_m4_uat_fmt_shift(1);
		struct gpu_m4_vm_table *l1;

		l1 = gpu_m4_vm_tree_ensure(tree, txn, tree->root, 1, gpu_m4_uat_fmt_base(base, 1),
					   &error);
		if (!l1 || !gpu_m4_vm_tree_ensure(tree, txn, l1, 2, base, &error))
			return error;
	}
	return 0;
}

static void gpu_m4_vm_tree_pools_free(struct gpu_m4_vm_tree_txn *txn)
{
	while (!list_empty(&txn->extents)) {
		struct gpu_m4_vm_extent *e = list_first_entry(&txn->extents,
			struct gpu_m4_vm_extent, link);

		list_del(&e->link);
		kfree(e);
	}
	while (!list_empty(&txn->objects)) {
		struct gpu_m4_vm_object *o = list_first_entry(&txn->objects,
			struct gpu_m4_vm_object, link);

		list_del(&o->link);
		kfree(o);
	}
}

/* Undo a prepare: no page was published, so unlink and free (newest first). */
static void gpu_m4_vm_tree_unstage(struct gpu_m4_vm_tree_txn *txn)
{
	while (txn->staged) {
		struct gpu_m4_vm_table *t = txn->staged;

		txn->staged = t->staged;
		t->parent->child[t->slot] = NULL;
		gpu_m4_vm_table_free(t);
	}
	txn->tables = 0;
}

static bool gpu_m4_vm_tree_takes_ref(const struct gpu_m4_vm_tree_op *op)
{
	return !(op->flags & GPU_M4_VM_TREE_UNBIND) && op->object;
}

/*
 * Acquire everything the whole batch can need: tables for every inserted
 * page, one extent per insertion plus one per op for a split, one object
 * record per insertion and one object reference per insertion. On failure
 * nothing visible changed and nothing is held.
 */
static int gpu_m4_vm_tree_prepare(struct gpu_m4_vm_tree *tree, const struct gpu_m4_vm_tree_op *op,
				  u32 count, struct gpu_m4_vm_tree_txn *txn)
{
	bool driver = op[0].flags & GPU_M4_VM_TREE_DRIVER;
	u32 got;
	int ret = 0;

	for (u32 i = 0; i < count; i++) {
		u64 last = op[i].va + op[i].size - 1;
		struct gpu_m4_vm_extent *e;

		ret = gpu_m4_vm_tree_check(tree, &op[i], driver);
		if (ret)
			return ret;
		/* User operations never touch a driver mapping (I4), and driver
		 * operations never touch a user mapping.
		 */
		for (e = gpu_m4_vm_extent_iter_first(&tree->extents, op[i].va, last);
		     e; e = gpu_m4_vm_extent_iter_next(e, op[i].va, last))
			if (!(e->flags & GPU_M4_VM_TREE_DRIVER) != !driver)
				return -EBUSY;
	}
	for (u32 i = 0; i < count && !ret; i++)
		if (!(op[i].flags & GPU_M4_VM_TREE_UNBIND))
			ret = gpu_m4_vm_tree_tables_for(tree, txn, op[i].va, op[i].size);
	for (u32 i = 0; i < count && !ret; i++) {
		u32 extents = op[i].flags & GPU_M4_VM_TREE_UNBIND ? 1 : 2;

		for (u32 j = 0; j < extents && !ret; j++) {
			struct gpu_m4_vm_extent *e = kzalloc(sizeof(*e), GFP_KERNEL);

			if (e)
				list_add_tail(&e->link, &txn->extents);
			else
				ret = -ENOMEM;
		}
		if (!ret && gpu_m4_vm_tree_takes_ref(&op[i])) {
			struct gpu_m4_vm_object *o = kzalloc(sizeof(*o), GFP_KERNEL);

			if (o)
				list_add_tail(&o->link, &txn->objects);
			else
				ret = -ENOMEM;
		}
	}
	for (got = 0; got < count && !ret; got++)
		if (gpu_m4_vm_tree_takes_ref(&op[got]))
			ret = tree->config.get(tree->config.cookie, op[got].object);
	if (!ret)
		return 0;
	/* got - 1 is the failed op when a get failed; put the earlier ones. */
	for (u32 i = 0; i + 1 < got; i++)
		if (gpu_m4_vm_tree_takes_ref(&op[i]))
			tree->config.put(tree->config.cookie, op[i].object);
	gpu_m4_vm_tree_pools_free(txn);
	gpu_m4_vm_tree_unstage(txn);
	return ret;
}

/*
 * Publish the new (zero) tables: clean them, then their parent words (leaf
 * tables first, so no parent ever names an uncleaned child), then DSB OSH.
 */
static void gpu_m4_vm_tree_publish(struct gpu_m4_vm_tree *tree, struct gpu_m4_vm_tree_txn *txn)
{
	struct gpu_m4_vm_table *t;

	if (!txn->staged)
		return;
	for (u8 level = 2; level >= 1; level--) {
		for (t = txn->staged; t; t = t->staged)
			if (t->level == level)
				tree->config.ops->clean(tree->config.cookie, t->cpu,
							GPU_M4_DRM_PAGE_SIZE);
		for (t = txn->staged; t; t = t->staged) {
			if (t->level != level)
				continue;
			gpu_m4_vm_word_set(t->parent, t->slot, gpu_m4_uat_fmt_table_pte(t->pa));
			if (!t->parent->fresh)
				gpu_m4_vm_clean_words(tree, t->parent, t->slot, t->slot);
		}
	}
	tree->config.ops->dsb_osh(tree->config.cookie);
	for (t = txn->staged; t; t = t->staged)
		t->fresh = false;
	tree->tables += txn->tables;
	gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_TABLES, txn->tables);
	txn->staged = NULL;
	txn->tables = 0;
}

static void gpu_m4_vm_tree_fault(struct gpu_m4_vm_tree *tree)
{
	if (!tree->faulted)
		gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_FAULTS, 1);
	tree->faulted = true;
}

/* Clear the valid leaves of [first, last] and clean each changed span. */
static void gpu_m4_vm_tree_clear(struct gpu_m4_vm_tree *tree, u64 first, u64 last, u32 flags,
				 struct gpu_m4_vm_removal *r)
{
	u64 va = first;

	while (va <= last) {
		u64 block_last = gpu_m4_uat_fmt_base(va, 2) + (1ULL << gpu_m4_uat_fmt_shift(1)) - 1;
		u64 end = min(block_last, last);
		struct gpu_m4_vm_table *t = gpu_m4_vm_tree_l2(tree, va);
		u32 from = gpu_m4_uat_fmt_slot(va, 2), to = gpu_m4_uat_fmt_slot(end, 2);

		if (!t) {
			gpu_m4_vm_tree_fault(tree);
		} else {
			for (u32 slot = from; slot <= to; slot++) {
				if (!gpu_m4_vm_word(t, slot)) {
					gpu_m4_vm_tree_fault(tree);
					continue;
				}
				gpu_m4_vm_word_set(t, slot, 0);
				t->used--;
				tree->leaves--;
				if (flags & GPU_M4_VM_TREE_DRIVER)
					tree->driver_leaves--;
				r->pages++;
			}
			gpu_m4_vm_clean_words(tree, t, from, to);
		}
		if (end == last)
			break;
		va = end + 1;
	}
	r->first = min(r->first, first);
	r->last = max(r->last, last);
	gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_UNMAPPED, (last - first + 1) / GPU_M4_DRM_PAGE_SIZE);
}

/*
 * Whether a removal must invalidate: a shadow tree always records its TLBI
 * (no-ops); an authority tree only while resident in its ASID (S3c). Returns
 * with binding_lock held for an authority tree: the decision, the TLBI and
 * the completing DSB SY form one section an installation cannot enter, so a
 * root installed after a "not resident" decision can only ever walk leaves
 * already cleared, cleaned and ordered by the preceding DSB OSH.
 */
static bool gpu_m4_vm_tree_tlbi_begin(struct gpu_m4_vm_tree *tree)
{
	if (!tree->config.authority)
		return true;
	mutex_lock(&tree->binding_lock);
	if (tree->resident && tree->asid)
		return true;
	gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_TLBI_SKIPPED, 1);
	return false;
}

static void gpu_m4_vm_tree_tlbi_end(struct gpu_m4_vm_tree *tree)
{
	if (tree->config.authority)
		mutex_unlock(&tree->binding_lock);
}

/* Invalidate [va, va + pages) on the VM's ASID: VAE1OS for one page, else
 * RVAE1OS runs (16 KiB granule), each covering (NUM + 1) << (5 * SCALE + 1).
 */
static void gpu_m4_vm_tree_tlbi(struct gpu_m4_vm_tree *tree, u64 va, u64 pages)
{
	const struct gpu_m4_vm_tree_ops *ops = tree->config.ops;
	u32 scale = 0;

	while (pages) {
		u64 num, covered;

		if (pages == 1) {
			ops->tlbi_va(tree->config.cookie, gpu_m4_uat_fmt_tlbi_va(tree->asid, va));
			gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_TLBI_VA, 1);
			return;
		}
		if (pages >= GPU_M4_UAT_TLBI_RANGE_MAX_PAGES) {
			ops->tlbi_range(tree->config.cookie,
					gpu_m4_uat_fmt_tlbi_range(tree->asid, va, 3, 31));
			gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_TLBI_RANGE, 1);
			va += GPU_M4_UAT_TLBI_RANGE_MAX_PAGES * GPU_M4_DRM_PAGE_SIZE;
			pages -= GPU_M4_UAT_TLBI_RANGE_MAX_PAGES;
			continue;
		}
		if (scale > 3) {
			/* Unreachable: below the maximum, scales 0..3 leave at most one page. */
			gpu_m4_vm_tree_fault(tree);
			return;
		}
		num = (pages >> (5 * scale + 1)) & 31;
		if (num) {
			ops->tlbi_range(tree->config.cookie,
					gpu_m4_uat_fmt_tlbi_range(tree->asid, va, scale, num - 1));
			gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_TLBI_RANGE, 1);
			covered = gpu_m4_uat_fmt_tlbi_range_pages(scale, num - 1);
			va += covered * GPU_M4_DRM_PAGE_SIZE;
			pages -= covered;
		}
		scale++;
	}
}

static void gpu_m4_vm_tree_removal_init(struct gpu_m4_vm_removal *r)
{
	r->first = U64_MAX;
	r->last = 0;
	r->pages = 0;
	INIT_LIST_HEAD(&r->extents);
	INIT_LIST_HEAD(&r->dead);
}

/* Detach extent e from its owner; a now unmapped owner waits in r->dead. */
static void gpu_m4_vm_tree_disown(struct gpu_m4_vm_tree *tree, struct gpu_m4_vm_extent *e,
				  struct gpu_m4_vm_removal *r)
{
	struct gpu_m4_vm_object *o = e->owner;

	list_del(&e->link);
	list_add_tail(&e->link, &r->extents);
	if (e->flags & GPU_M4_VM_TREE_CLOSED) {
		list_del_init(&e->deferred);
		gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_DEFERRED_REMOVED, 1);
	}
	if (e->flags & GPU_M4_VM_TREE_DRIVER)
		gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_DRIVER_REMOVED, 1);
	if (!o)
		return;
	e->owner = NULL;
	if (--o->count)
		return;
	rb_erase(&o->node, &tree->objects);
	list_add_tail(&o->link, &r->dead);
}

/* After the TLBI and DSB SY: free removed extents, drop dead references. */
static void gpu_m4_vm_tree_release(struct gpu_m4_vm_tree *tree, struct gpu_m4_vm_removal *r)
{
	while (!list_empty(&r->extents)) {
		struct gpu_m4_vm_extent *e = list_first_entry(&r->extents,
			struct gpu_m4_vm_extent, link);

		list_del(&e->link);
		kfree(e);
	}
	while (!list_empty(&r->dead)) {
		struct gpu_m4_vm_object *o = list_first_entry(&r->dead,
			struct gpu_m4_vm_object, link);

		list_del(&o->link);
		/* I2: an authority tree whose edits found an inconsistency cannot
		 * prove the removed range was translated by nothing else: it keeps
		 * the reference (and the tree its pages) for good.
		 */
		if (tree->faulted && tree->config.authority) {
			list_add_tail(&o->link, &tree->retained);
			gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_RETAINED, 1);
			continue;
		}
		tree->config.put(tree->config.cookie, o->object);
		kfree(o);
	}
}

static struct gpu_m4_vm_extent *gpu_m4_vm_tree_spare(struct gpu_m4_vm_tree_txn *txn)
{
	struct gpu_m4_vm_extent *e;

	if (list_empty(&txn->extents))
		return NULL;
	e = list_first_entry(&txn->extents, struct gpu_m4_vm_extent, link);
	list_del(&e->link);
	return e;
}

static u64 gpu_m4_vm_extent_advance(const struct gpu_m4_vm_extent *e, u64 va)
{
	return e->flags & GPU_M4_VM_TREE_SINGLE_PAGE ? 0 : va - e->start;
}

/* Remove [s, l] from extent e: clear, then trim, split or drop it. */
static void gpu_m4_vm_tree_cut(struct gpu_m4_vm_tree *tree, struct gpu_m4_vm_tree_txn *txn,
			       struct gpu_m4_vm_extent *e, u64 s, u64 l,
			       struct gpu_m4_vm_removal *r)
{
	u64 first = max(e->start, s), last = min(e->last, l);
	struct gpu_m4_vm_extent *tail;

	gpu_m4_vm_tree_clear(tree, first, last, e->flags, r);
	gpu_m4_vm_extent_remove(e, &tree->extents);
	if (e->last > l && e->start < s) {
		/* At most one extent per op strictly contains the range. */
		tail = gpu_m4_vm_tree_spare(txn);
		if (!tail) {
			gpu_m4_vm_tree_fault(tree);
			gpu_m4_vm_tree_clear(tree, l + 1, e->last, e->flags, r);
			e->last = s - 1;
		} else {
			*tail = (struct gpu_m4_vm_extent) {
				.start = l + 1, .last = e->last, .owner = e->owner,
				.identity = e->identity,
				.offset = e->offset + gpu_m4_vm_extent_advance(e, l + 1),
				.access = e->access, .flags = e->flags, .cpu_policy = e->cpu_policy,
				.attrs = e->attrs,
			};
			INIT_LIST_HEAD(&tail->link);
			INIT_LIST_HEAD(&tail->deferred);
			if (tail->flags & GPU_M4_VM_TREE_CLOSED)
				list_add_tail(&tail->deferred, &tree->deferred);
			if (tail->flags & GPU_M4_VM_TREE_DRIVER)
				gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_DRIVER_ADDED, 1);
			if (tail->owner) {
				list_add_tail(&tail->link, &tail->owner->extents);
				tail->owner->count++;
			}
			gpu_m4_vm_extent_insert(tail, &tree->extents);
			e->last = s - 1;
		}
		gpu_m4_vm_extent_insert(e, &tree->extents);
	} else if (e->start < s) {
		e->last = s - 1;
		gpu_m4_vm_extent_insert(e, &tree->extents);
	} else if (e->last > l) {
		e->offset += gpu_m4_vm_extent_advance(e, l + 1);
		e->start = l + 1;
		gpu_m4_vm_extent_insert(e, &tree->extents);
	} else {
		gpu_m4_vm_tree_disown(tree, e, r);
	}
}

/* Write the leaves of a new extent into zero words; no TLBI. */
static void gpu_m4_vm_tree_insert(struct gpu_m4_vm_tree *tree, struct gpu_m4_vm_tree_txn *txn,
				  const struct gpu_m4_vm_tree_op *op)
{
	struct gpu_m4_vm_extent *e = gpu_m4_vm_tree_spare(txn);
	bool single = op->flags & GPU_M4_VM_TREE_SINGLE_PAGE, surplus = false;
	u64 last = op->va + op->size - 1, first = op->offset / GPU_M4_DRM_PAGE_SIZE, i = 0;
	u64 va = op->va;

	if (!e) {
		gpu_m4_vm_tree_fault(tree);
		if (op->object)
			tree->config.put(tree->config.cookie, op->object);
		return;
	}
	*e = (struct gpu_m4_vm_extent) {
		.start = op->va, .last = last, .offset = op->offset, .access = op->access,
		.flags = op->flags & (GPU_M4_VM_TREE_SINGLE_PAGE | GPU_M4_VM_TREE_DRIVER),
		.identity = op->identity ?: op->object, .cpu_policy = op->cpu_policy,
		.attrs = op->flags & GPU_M4_VM_TREE_DRIVER ? op->attrs : 0,
	};
	INIT_LIST_HEAD(&e->link);
	INIT_LIST_HEAD(&e->deferred);
	if (op->object) {
		struct gpu_m4_vm_object *o = gpu_m4_vm_tree_find(tree, op->object);

		if (o) {
			surplus = true;
		} else {
			/* Prepared one record per referencing insertion. */
			o = list_first_entry(&txn->objects, struct gpu_m4_vm_object, link);
			list_del(&o->link);
			*o = (struct gpu_m4_vm_object) { .object = op->object };
			INIT_LIST_HEAD(&o->extents);
			INIT_LIST_HEAD(&o->link);
			gpu_m4_vm_tree_link_object(tree, o);
		}
		list_add_tail(&e->link, &o->extents);
		o->count++;
		e->owner = o;
	}
	gpu_m4_vm_extent_insert(e, &tree->extents);
	while (va <= last) {
		u64 block_last = gpu_m4_uat_fmt_base(va, 2) + (1ULL << gpu_m4_uat_fmt_shift(1)) - 1;
		u64 end = min(block_last, last);
		struct gpu_m4_vm_table *t = gpu_m4_vm_tree_l2(tree, va);
		u32 from = gpu_m4_uat_fmt_slot(va, 2), to = gpu_m4_uat_fmt_slot(end, 2);

		if (!t) {
			gpu_m4_vm_tree_fault(tree);
			i += to - from + 1;
		}
		for (u32 slot = from; t && slot <= to; slot++, i++) {
			u64 pa = op->pa[first + (single ? 0 : i)];

			if (gpu_m4_vm_word(t, slot)) {
				gpu_m4_vm_tree_fault(tree);
				continue;
			}
			gpu_m4_vm_word_set(t, slot, e->attrs ? pa | e->attrs :
					   gpu_m4_uat_fmt_leaf_pte(pa, op->access));
			t->used++;
			tree->leaves++;
			if (op->flags & GPU_M4_VM_TREE_DRIVER)
				tree->driver_leaves++;
		}
		if (t)
			gpu_m4_vm_clean_words(tree, t, from, to);
		if (end == last)
			break;
		va = end + 1;
	}
	tree->config.ops->dsb_osh(tree->config.cookie);
	gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_MAPPED, op->size / GPU_M4_DRM_PAGE_SIZE);
	if (op->flags & GPU_M4_VM_TREE_DRIVER)
		gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_DRIVER_ADDED, 1);
	/* The prepared reference of an already held object is not needed. */
	if (surplus)
		tree->config.put(tree->config.cookie, op->object);
}

/* One op: remove the range (full removal sequence), then insert. */
static void gpu_m4_vm_tree_apply(struct gpu_m4_vm_tree *tree, struct gpu_m4_vm_tree_txn *txn,
				 const struct gpu_m4_vm_tree_op *op)
{
	u64 last = op->va + op->size - 1;
	struct gpu_m4_vm_removal r;
	struct gpu_m4_vm_extent *e;

	gpu_m4_vm_tree_removal_init(&r);
	while ((e = gpu_m4_vm_extent_iter_first(&tree->extents, op->va, last)))
		gpu_m4_vm_tree_cut(tree, txn, e, op->va, last, &r);
	if (r.pages) {
		tree->config.ops->dsb_osh(tree->config.cookie);
		if (gpu_m4_vm_tree_tlbi_begin(tree))
			gpu_m4_vm_tree_tlbi(tree, r.first,
					    (r.last - r.first) / GPU_M4_DRM_PAGE_SIZE + 1);
		tree->config.ops->dsb_sy(tree->config.cookie);
		gpu_m4_vm_tree_tlbi_end(tree);
	}
	gpu_m4_vm_tree_release(tree, &r);
	if (!(op->flags & GPU_M4_VM_TREE_UNBIND))
		gpu_m4_vm_tree_insert(tree, txn, op);
	gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_OPS, 1);
}

/* Removal list r was cleared and cleaned: barrier, invalidate, release. */
static void gpu_m4_vm_tree_settle(struct gpu_m4_vm_tree *tree, struct gpu_m4_vm_removal *r,
				  bool whole_asid)
{
	const struct gpu_m4_vm_tree_ops *ops = tree->config.ops;
	struct gpu_m4_vm_extent *e;

	if (r->pages || whole_asid) {
		ops->dsb_osh(tree->config.cookie);
		if (whole_asid && tree->config.authority) {
			/* Teardown of an evicted authority tree: the binder already
			 * invalidated its ASID; one more ASIDE1OS (when it ever had
			 * one) costs nothing and needs no residency decision.
			 */
			mutex_lock(&tree->binding_lock);
			if (tree->asid) {
				ops->tlbi_asid(tree->config.cookie,
					       gpu_m4_uat_fmt_tlbi_asid(tree->asid));
				gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_TLBI_ASID, 1);
			}
		} else if (!gpu_m4_vm_tree_tlbi_begin(tree)) {
			/* Not resident: nothing of this root can be cached. */
		} else if (whole_asid) {
			ops->tlbi_asid(tree->config.cookie, gpu_m4_uat_fmt_tlbi_asid(tree->asid));
			gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_TLBI_ASID, 1);
		} else {
			list_for_each_entry(e, &r->extents, link)
				gpu_m4_vm_tree_tlbi(tree, e->start,
					(e->last - e->start) / GPU_M4_DRM_PAGE_SIZE + 1);
		}
		ops->dsb_sy(tree->config.cookie);
		gpu_m4_vm_tree_tlbi_end(tree);
	}
	gpu_m4_vm_tree_release(tree, r);
}

/* Clear every extent, the driver ones included. */
static void gpu_m4_vm_tree_clear_all(struct gpu_m4_vm_tree *tree, bool whole_asid)
{
	struct gpu_m4_vm_removal r;
	struct gpu_m4_vm_extent *e;

	gpu_m4_vm_tree_removal_init(&r);
	while ((e = gpu_m4_vm_extent_iter_first(&tree->extents, 0, U64_MAX))) {
		gpu_m4_vm_tree_clear(tree, e->start, e->last, e->flags, &r);
		gpu_m4_vm_extent_remove(e, &tree->extents);
		gpu_m4_vm_tree_disown(tree, e, &r);
	}
	gpu_m4_vm_tree_settle(tree, &r, whole_asid);
}

struct gpu_m4_vm_tree *gpu_m4_vm_tree_create(const struct gpu_m4_vm_tree_config *config)
{
	const struct gpu_m4_vm_tree_ops *ops = config ? config->ops : NULL;
	struct gpu_m4_vm_tree *tree;

	if (!ops || !ops->clean || !ops->dsb_osh || !ops->dsb_sy || !ops->tlbi_va ||
	    !ops->tlbi_range || !ops->tlbi_asid || !config->get || !config->put ||
	    !config->max_tables || config->low >= config->high ||
	    config->high > GPU_M4_UAT_VA_LIMIT || !gpu_m4_drm_aligned(config->low) ||
	    !gpu_m4_drm_aligned(config->high))
		return ERR_PTR(-EINVAL);
	tree = kzalloc(sizeof(*tree), GFP_KERNEL);
	if (!tree)
		return ERR_PTR(-ENOMEM);
	tree->root = gpu_m4_vm_table_alloc(0, 0);
	if (IS_ERR(tree->root)) {
		int ret = PTR_ERR(tree->root);

		kfree(tree);
		return ERR_PTR(ret);
	}
	mutex_init(&tree->lock);
	spin_lock_init(&tree->change_lock);
	mutex_init(&tree->binding_lock);
	tree->config = *config;
	tree->extents = RB_ROOT_CACHED;
	tree->objects = RB_ROOT;
	tree->tables = 1;
	atomic_set(&tree->guards, 0);
	INIT_LIST_HEAD(&tree->deferred);
	INIT_LIST_HEAD(&tree->retained);
	/* The root is zero; clean it once before anything can name it. */
	ops->clean(config->cookie, tree->root->cpu, GPU_M4_DRM_PAGE_SIZE);
	ops->dsb_osh(config->cookie);
	gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_TREES, 1);
	gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_TABLES, 1);
	return tree;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_create);

/* Advance the revision before any leaf/extent change of [first, last]
 * (tree->lock held).
 */
static void gpu_m4_vm_tree_changing(struct gpu_m4_vm_tree *tree, u64 first, u64 last)
{
	u64 rev = tree->revision + 1;
	struct gpu_m4_vm_tree_change *c = &tree->changes[rev % GPU_M4_VM_TREE_CHANGES];

	lockdep_assert_held(&tree->lock);
	spin_lock(&tree->change_lock);
	c->revision = rev;
	c->first = first;
	c->last = last;
	WRITE_ONCE(tree->revision, rev);
	spin_unlock(&tree->change_lock);
}

int gpu_m4_vm_tree_destroy(struct gpu_m4_vm_tree *tree)
{
	struct gpu_m4_vm_table *root;
	bool resident;

	if (IS_ERR_OR_NULL(tree))
		return 0;
	mutex_lock(&tree->lock);
	gpu_m4_vm_tree_changing(tree, 0, U64_MAX); /* before any leaf change */
	mutex_lock(&tree->binding_lock);
	resident = tree->resident;
	mutex_unlock(&tree->binding_lock);
	/* I3: the owner destroys only an unguarded tree its binder already
	 * evicted (TTBAT entry replaced or cleared, ASID invalidated). Anything
	 * else keeps every page and reference for good.
	 */
	if (tree->config.authority && (WARN_ON(atomic_read(&tree->guards)) || WARN_ON(resident)))
		gpu_m4_vm_tree_fault(tree);
	if (!tree->faulted)
		gpu_m4_vm_tree_clear_all(tree, true);
	root = tree->root;
	if (tree->faulted) {
		/* I2: a tree whose edits went wrong keeps its pages. */
		mutex_unlock(&tree->lock);
		return -EBUSY;
	}
	for (u32 i = 0; i < GPU_M4_UAT_ROOT_ENTRIES; i++) {
		struct gpu_m4_vm_table *l1 = root->child[i];

		if (!l1)
			continue;
		for (u32 j = 0; j < GPU_M4_UAT_ENTRIES; j++)
			if (l1->child[j])
				gpu_m4_vm_table_free(l1->child[j]);
		gpu_m4_vm_table_free(l1);
	}
	gpu_m4_vm_table_free(root);
	mutex_unlock(&tree->lock);
	kfree(tree);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_destroy);

int gpu_m4_vm_tree_bind(struct gpu_m4_vm_tree *tree, const struct gpu_m4_vm_tree_op *op,
			u32 count)
{
	struct gpu_m4_vm_tree_txn txn = { 0 };
	u64 first = U64_MAX, last = 0;
	int ret;

	if (IS_ERR_OR_NULL(tree) || !op || !count)
		return -EINVAL;
	INIT_LIST_HEAD(&txn.extents);
	INIT_LIST_HEAD(&txn.objects);
	for (u32 i = 0; i < count; i++) {
		first = min(first, op[i].va);
		last = max(last, op[i].va + (op[i].size ? op[i].size - 1 : 0));
	}
	mutex_lock(&tree->lock);
	gpu_m4_vm_tree_changing(tree, first, last); /* before any leaf change */
	ret = tree->desynced ? -ESTALE : tree->faulted ? -EIO :
		tree->closed && !(op[0].flags & GPU_M4_VM_TREE_DRIVER) ? -ENOENT :
		gpu_m4_vm_tree_prepare(tree, op, count, &txn);
	if (ret) {
		gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_REFUSED, 1);
		mutex_unlock(&tree->lock);
		return ret;
	}
	gpu_m4_vm_tree_publish(tree, &txn);
	for (u32 i = 0; i < count; i++)
		gpu_m4_vm_tree_apply(tree, &txn, &op[i]);
	gpu_m4_vm_tree_pools_free(&txn);
	gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_BATCHES, 1);
	ret = tree->faulted ? -EIO : 0;
	mutex_unlock(&tree->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_bind);

/*
 * Mark a user extent CLOSED: still mapped and holding its reference, removed
 * by settle_deferred once no accepted job holds a guard. Returns whether it
 * was newly marked.
 */
static bool gpu_m4_vm_tree_close_extent(struct gpu_m4_vm_tree *tree, struct gpu_m4_vm_extent *e)
{
	if (e->flags & (GPU_M4_VM_TREE_DRIVER | GPU_M4_VM_TREE_CLOSED))
		return false;
	e->flags |= GPU_M4_VM_TREE_CLOSED;
	list_add_tail(&e->deferred, &tree->deferred);
	gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_DEFERRED_CLOSES, 1);
	return true;
}

/* Remove every CLOSED extent when no guard exists (tree->lock held). */
static int gpu_m4_vm_tree_settle_deferred_locked(struct gpu_m4_vm_tree *tree)
{
	struct gpu_m4_vm_removal r;
	int removed = 0;

	if (atomic_read(&tree->guards))
		return 0;
	gpu_m4_vm_tree_removal_init(&r);
	while (!list_empty(&tree->deferred)) {
		struct gpu_m4_vm_extent *e = list_first_entry(&tree->deferred,
			struct gpu_m4_vm_extent, deferred);

		gpu_m4_vm_tree_clear(tree, e->start, e->last, e->flags, &r);
		gpu_m4_vm_extent_remove(e, &tree->extents);
		gpu_m4_vm_tree_disown(tree, e, &r);
		removed++;
	}
	WRITE_ONCE(tree->deferred_pending, false);
	gpu_m4_vm_tree_settle(tree, &r, false);
	return removed;
}

/*
 * After marking: publish the pending flag, then look at the guards again. A
 * concurrent last guard_put either sees the flag (and asks for the deferred
 * removal) or dropped to zero before this re-read (removed here). Both is
 * harmless: the second finds nothing.
 */
static void gpu_m4_vm_tree_defer_locked(struct gpu_m4_vm_tree *tree)
{
	WRITE_ONCE(tree->deferred_pending, true);
	smp_mb();
	if (atomic_read(&tree->guards))
		gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_GUARD_WAITS, 1);
	else
		gpu_m4_vm_tree_settle_deferred_locked(tree);
}

int gpu_m4_vm_tree_remove_object(struct gpu_m4_vm_tree *tree, const void *object)
{
	struct gpu_m4_vm_removal r;
	struct gpu_m4_vm_object *o;
	int ret;

	if (IS_ERR_OR_NULL(tree) || !object)
		return -EINVAL;
	gpu_m4_vm_tree_removal_init(&r);
	mutex_lock(&tree->lock);
	gpu_m4_vm_tree_changing(tree, 0, U64_MAX); /* before any leaf change */
	o = gpu_m4_vm_tree_find(tree, object);
	if (o && tree->config.authority && !tree->faulted) {
		struct gpu_m4_vm_extent *e;
		bool marked = false;

		/* GEM close (S3c): CLOSED until no accepted job is guarded. */
		list_for_each_entry(e, &o->extents, link)
			marked |= gpu_m4_vm_tree_close_extent(tree, e);
		if (marked)
			gpu_m4_vm_tree_defer_locked(tree);
	} else if (o) {
		struct gpu_m4_vm_extent *e, *next;

		/* Detach every user extent first; the list is the owner's. */
		list_for_each_entry_safe(e, next, &o->extents, link) {
			if (e->flags & GPU_M4_VM_TREE_DRIVER)
				continue;
			gpu_m4_vm_tree_clear(tree, e->start, e->last, e->flags, &r);
			gpu_m4_vm_extent_remove(e, &tree->extents);
			gpu_m4_vm_tree_disown(tree, e, &r);
		}
		gpu_m4_vm_tree_settle(tree, &r, false);
	}
	ret = tree->faulted ? -EIO : 0;
	mutex_unlock(&tree->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_remove_object);

void gpu_m4_vm_tree_desync(struct gpu_m4_vm_tree *tree)
{
	if (IS_ERR_OR_NULL(tree))
		return;
	mutex_lock(&tree->lock);
	gpu_m4_vm_tree_changing(tree, 0, U64_MAX); /* before any leaf change */
	if (!tree->desynced)
		gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_DESYNCS, 1);
	tree->desynced = true;
	gpu_m4_vm_tree_clear_all(tree, false);
	mutex_unlock(&tree->lock);
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_desync);

bool gpu_m4_vm_tree_desynced(struct gpu_m4_vm_tree *tree)
{
	bool desynced;

	if (IS_ERR_OR_NULL(tree))
		return true;
	mutex_lock(&tree->lock);
	desynced = tree->desynced;
	mutex_unlock(&tree->lock);
	return desynced;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_desynced);

bool gpu_m4_vm_tree_holds(struct gpu_m4_vm_tree *tree, const void *object)
{
	bool holds;

	if (IS_ERR_OR_NULL(tree))
		return false;
	mutex_lock(&tree->lock);
	holds = gpu_m4_vm_tree_find(tree, object);
	mutex_unlock(&tree->lock);
	return holds;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_holds);

u64 gpu_m4_vm_tree_root(struct gpu_m4_vm_tree *tree)
{
	return IS_ERR_OR_NULL(tree) ? 0 : tree->root->pa;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_root);

void gpu_m4_vm_tree_set_asid(struct gpu_m4_vm_tree *tree, u32 asid)
{
	if (IS_ERR_OR_NULL(tree))
		return;
	mutex_lock(&tree->lock);
	mutex_lock(&tree->binding_lock);
	tree->asid = asid;
	mutex_unlock(&tree->binding_lock);
	mutex_unlock(&tree->lock);
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_set_asid);

int gpu_m4_vm_tree_install_begin(struct gpu_m4_vm_tree *tree, u32 asid)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(tree) || !asid || asid >= GPU_M4_UAT_ROOT_ENTRIES)
		return -EINVAL;
	mutex_lock(&tree->binding_lock);
	/* A VM's ASID is fixed for its lifetime (dedicated or the shared 1). */
	if (tree->asid && tree->asid != asid) {
		ret = -EINVAL;
	} else {
		tree->asid = asid;
		tree->resident = true;
	}
	mutex_unlock(&tree->binding_lock);
	if (!ret)
		gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_INSTALLS, 1);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_install_begin);

void gpu_m4_vm_tree_evicted(struct gpu_m4_vm_tree *tree)
{
	if (IS_ERR_OR_NULL(tree))
		return;
	mutex_lock(&tree->binding_lock);
	if (tree->resident)
		gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_EVICTIONS, 1);
	tree->resident = false;
	mutex_unlock(&tree->binding_lock);
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_evicted);

bool gpu_m4_vm_tree_resident(struct gpu_m4_vm_tree *tree, u32 *asid)
{
	bool resident;

	if (IS_ERR_OR_NULL(tree))
		return false;
	mutex_lock(&tree->binding_lock);
	resident = tree->resident;
	if (asid)
		*asid = tree->asid;
	mutex_unlock(&tree->binding_lock);
	return resident;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_resident);

int gpu_m4_vm_tree_healthy(struct gpu_m4_vm_tree *tree)
{
	int ret;

	if (IS_ERR_OR_NULL(tree))
		return -EINVAL;
	mutex_lock(&tree->lock);
	ret = tree->desynced ? -ESTALE : tree->faulted ? -EIO : 0;
	mutex_unlock(&tree->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_healthy);

int gpu_m4_vm_tree_guard_get(struct gpu_m4_vm_tree *tree)
{
	if (IS_ERR_OR_NULL(tree))
		return -EINVAL;
	if (READ_ONCE(tree->closed))
		return -ESHUTDOWN;
	atomic_inc(&tree->guards);
	/* Pairs with close(): it publishes closed, then reads the guards. */
	smp_mb__after_atomic();
	if (READ_ONCE(tree->closed) || READ_ONCE(tree->faulted)) {
		int ret = READ_ONCE(tree->closed) ? -ESHUTDOWN : -EIO;

		gpu_m4_vm_tree_guard_put(tree);
		return ret;
	}
	gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_GUARDS, 1);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_guard_get);

void gpu_m4_vm_tree_guard_put(struct gpu_m4_vm_tree *tree)
{
	if (IS_ERR_OR_NULL(tree) || WARN_ON(!atomic_read(&tree->guards)))
		return;
	/* atomic_dec_and_test is fully ordered; pairs with defer_locked(). */
	if (atomic_dec_and_test(&tree->guards) && READ_ONCE(tree->deferred_pending) &&
	    tree->config.deferred)
		tree->config.deferred(tree->config.cookie);
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_guard_put);

u32 gpu_m4_vm_tree_guards(struct gpu_m4_vm_tree *tree)
{
	return IS_ERR_OR_NULL(tree) ? 0 : atomic_read(&tree->guards);
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_guards);

int gpu_m4_vm_tree_close(struct gpu_m4_vm_tree *tree)
{
	struct gpu_m4_vm_extent *e;
	bool marked = false;
	int ret;

	if (IS_ERR_OR_NULL(tree))
		return -EINVAL;
	mutex_lock(&tree->lock);
	gpu_m4_vm_tree_changing(tree, 0, U64_MAX); /* before any leaf change */
	if (!tree->closed) {
		WRITE_ONCE(tree->closed, true);
		smp_mb(); /* closed before the guards are read (guard_get) */
		for (e = gpu_m4_vm_extent_iter_first(&tree->extents, 0, U64_MAX); e;
		     e = gpu_m4_vm_extent_iter_next(e, 0, U64_MAX))
			marked |= gpu_m4_vm_tree_close_extent(tree, e);
		if (marked && !tree->faulted)
			gpu_m4_vm_tree_defer_locked(tree);
	}
	ret = tree->faulted ? -EIO : 0;
	mutex_unlock(&tree->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_close);

int gpu_m4_vm_tree_settle_deferred(struct gpu_m4_vm_tree *tree)
{
	int ret;

	if (IS_ERR_OR_NULL(tree))
		return -EINVAL;
	mutex_lock(&tree->lock);
	if (!list_empty(&tree->deferred)) /* else nothing to remove */
		gpu_m4_vm_tree_changing(tree, 0, U64_MAX);
	ret = tree->faulted ? -EIO : gpu_m4_vm_tree_settle_deferred_locked(tree);
	mutex_unlock(&tree->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_settle_deferred);

u32 gpu_m4_vm_tree_deferred_count(struct gpu_m4_vm_tree *tree)
{
	struct gpu_m4_vm_extent *e;
	u32 n = 0;

	if (IS_ERR_OR_NULL(tree))
		return 0;
	mutex_lock(&tree->lock);
	list_for_each_entry(e, &tree->deferred, deferred)
		n++;
	mutex_unlock(&tree->lock);
	return n;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_deferred_count);

/* The installed leaf of page under extent e, checked word against extent. */
static int gpu_m4_vm_tree_leaf(const struct gpu_m4_vm_tree *tree,
			       const struct gpu_m4_vm_extent *e, u64 page,
			       struct gpu_m4_drm_vm_leaf *leaf)
{
	const struct gpu_m4_vm_table *t = gpu_m4_vm_tree_l2(tree, page);
	u64 word = t ? gpu_m4_vm_word(t, gpu_m4_uat_fmt_slot(page, 2)) : 0;
	u64 pa = word & GPU_M4_PA_MASK;

	if (!word || word != (e->attrs ? pa | e->attrs : gpu_m4_uat_fmt_leaf_pte(pa, e->access)))
		return -EPROTO;
	*leaf = (struct gpu_m4_drm_vm_leaf) {
		.va = page, .pa = pa, .identity = e->identity,
		.object_offset = e->offset + gpu_m4_vm_extent_advance(e, page),
		.access = e->access, .cpu_policy = e->cpu_policy,
	};
	return 0;
}

/*
 * Visit every page of [va, va + size) in VA order through the extents that
 * cover it, checking each installed leaf; fn (optional) sees it. user: only
 * live user extents count (DRIVER or CLOSED pages are holes). A missing page
 * is found by the visited count. tree->lock held.
 */
static int gpu_m4_vm_tree_visit(struct gpu_m4_vm_tree *tree, u64 va, u64 size, u32 access,
				bool user, gpu_m4_vm_tree_leaf_fn fn, void *cookie)
{
	u64 first = va & ~(GPU_M4_DRM_PAGE_SIZE - 1), last, cursor, visited = 0;
	struct gpu_m4_vm_extent *e;

	if (tree->desynced || tree->faulted)
		return tree->desynced ? -ESTALE : -EIO;
	if (!size || !gpu_m4_drm_vm_leaf_attrs(access) || va > U64_MAX - size ||
	    va < tree->config.low || va + size > tree->config.high)
		return -EINVAL;
	last = (va + size - 1) & ~(GPU_M4_DRM_PAGE_SIZE - 1);
	cursor = first;
	for (e = gpu_m4_vm_extent_iter_first(&tree->extents, first, last); e && cursor <= last;
	     e = gpu_m4_vm_extent_iter_next(e, first, last)) {
		u64 end = min(e->last & ~(GPU_M4_DRM_PAGE_SIZE - 1), last);

		if (e->start > cursor ||
		    (user && (e->flags & (GPU_M4_VM_TREE_DRIVER | GPU_M4_VM_TREE_CLOSED))))
			return -ENOENT;
		if ((e->access & access) != access)
			return -EACCES;
		for (u64 page = cursor; page <= end; page += GPU_M4_DRM_PAGE_SIZE) {
			struct gpu_m4_drm_vm_leaf leaf;
			int ret = gpu_m4_vm_tree_leaf(tree, e, page, &leaf);

			if (!ret && fn)
				ret = fn(cookie, page, &leaf);
			if (ret)
				return ret;
			visited++;
		}
		if (end == last) {
			cursor = last + GPU_M4_DRM_PAGE_SIZE;
			break;
		}
		cursor = end + GPU_M4_DRM_PAGE_SIZE;
	}
	/* Holes by the visited-page count (working covers_range). */
	return visited == (last - first) / GPU_M4_DRM_PAGE_SIZE + 1 ? 0 : -ENOENT;
}

/* The run at va: same object, identity and access at contiguous offsets. */
static void gpu_m4_vm_tree_run(struct gpu_m4_vm_tree *tree, u64 va,
			       struct gpu_m4_vm_tree_span *span)
{
	u64 page = va & ~(GPU_M4_DRM_PAGE_SIZE - 1), within = va - page, next;
	struct gpu_m4_vm_extent *e = gpu_m4_vm_extent_iter_first(&tree->extents, page, page);
	u64 offset = e->offset + gpu_m4_vm_extent_advance(e, page);

	*span = (struct gpu_m4_vm_tree_span) {
		.va = va, .offset = offset + within, .object = e->owner ? e->owner->object : NULL,
		.identity = e->identity, .access = e->access, .cpu_policy = e->cpu_policy,
	};
	/* A SINGLE_PAGE extent repeats one page: the run ends with it. */
	if (e->flags & GPU_M4_VM_TREE_SINGLE_PAGE) {
		span->size = GPU_M4_DRM_PAGE_SIZE - within;
		return;
	}
	next = e->last + 1;
	span->size = next - va;
	while (next < tree->config.high) {
		struct gpu_m4_vm_extent *n = gpu_m4_vm_extent_iter_first(&tree->extents, next, next);

		if (!n || n->start != next || (n->flags & (GPU_M4_VM_TREE_SINGLE_PAGE |
		    GPU_M4_VM_TREE_DRIVER | GPU_M4_VM_TREE_CLOSED)) ||
		    n->identity != span->identity || n->access != span->access ||
		    n->owner != e->owner || n->offset != offset + (next - page))
			break;
		span->size += n->last + 1 - next;
		next = n->last + 1;
	}
}

int gpu_m4_vm_tree_covers(struct gpu_m4_vm_tree *tree, u64 va, u64 size, u32 access,
			  struct gpu_m4_vm_tree_span *span)
{
	const struct gpu_m4_vm_tree_config *c;
	int ret;

	if (IS_ERR_OR_NULL(tree))
		return -EINVAL;
	c = &tree->config;
	mutex_lock(&tree->lock);
	/* Named ranges are user VAs: never the kernel or a driver range. */
	if (size && va <= U64_MAX - size &&
	    (gpu_m4_vm_overlap(va, va + size, c->kernel_start, c->kernel_end) ||
	     gpu_m4_vm_overlap(va, va + size, c->reserved[0][0], c->reserved[0][1]) ||
	     gpu_m4_vm_overlap(va, va + size, c->reserved[1][0], c->reserved[1][1])))
		ret = -ENOENT;
	else
		ret = gpu_m4_vm_tree_visit(tree, va, size, access, true, NULL, NULL);
	if (!ret && span)
		gpu_m4_vm_tree_run(tree, va, span);
	mutex_unlock(&tree->lock);
	gpu_m4_vm_tree_count(ret ? GPU_M4_VM_TREE_STAT_COVERS_REFUSED : GPU_M4_VM_TREE_STAT_COVERS, 1);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_covers);

static int gpu_m4_vm_tree_first_leaf(void *cookie, u64 page __always_unused,
				     const struct gpu_m4_drm_vm_leaf *leaf)
{
	*(struct gpu_m4_drm_vm_leaf *)cookie = *leaf;
	return 0;
}

int gpu_m4_vm_tree_page(struct gpu_m4_vm_tree *tree, u64 va, u32 access,
			struct gpu_m4_drm_vm_leaf *leaf)
{
	int ret;

	if (IS_ERR_OR_NULL(tree) || !leaf)
		return -EINVAL;
	mutex_lock(&tree->lock);
	ret = gpu_m4_vm_tree_visit(tree, va & ~(GPU_M4_DRM_PAGE_SIZE - 1), GPU_M4_DRM_PAGE_SIZE,
				   access, false, gpu_m4_vm_tree_first_leaf, leaf);
	mutex_unlock(&tree->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_page);

u64 gpu_m4_vm_tree_revision(struct gpu_m4_vm_tree *tree)
{
	return tree ? READ_ONCE(tree->revision) : 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_revision);

bool gpu_m4_vm_tree_unchanged(struct gpu_m4_vm_tree *tree, u64 since, u64 va, u64 size)
{
	u64 last = va + (size ? size - 1 : 0), now;
	bool ok = true;

	/* A fault may clear beyond the logged span (cut without a spare). */
	if (!tree || !since || READ_ONCE(tree->faulted) || READ_ONCE(tree->desynced))
		return false;
	spin_lock(&tree->change_lock);
	now = tree->revision;
	if (since > now || now - since >= GPU_M4_VM_TREE_CHANGES)
		ok = since == now;
	else
		for (u64 r = since + 1; ok && r <= now; r++) {
			const struct gpu_m4_vm_tree_change *c =
				&tree->changes[r % GPU_M4_VM_TREE_CHANGES];

			ok = c->revision == r && (c->last < va || c->first > last);
		}
	spin_unlock(&tree->change_lock);
	return ok;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_unchanged);

int gpu_m4_vm_tree_walk(struct gpu_m4_vm_tree *tree, u64 va, u64 size, u32 access,
			gpu_m4_vm_tree_leaf_fn fn, void *cookie)
{
	int ret;

	if (IS_ERR_OR_NULL(tree))
		return -EINVAL;
	mutex_lock(&tree->lock);
	ret = gpu_m4_vm_tree_visit(tree, va, size, access, false, fn, cookie);
	mutex_unlock(&tree->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_walk);

bool gpu_m4_vm_tree_maps(struct gpu_m4_vm_tree *tree, u64 va, u64 size, const void *object,
			 u64 offset, u32 access, bool driver)
{
	struct gpu_m4_vm_extent *e;
	bool maps;

	if (IS_ERR_OR_NULL(tree) || !size || va > U64_MAX - size)
		return false;
	mutex_lock(&tree->lock);
	e = gpu_m4_vm_extent_iter_first(&tree->extents, va, va + size - 1);
	maps = !tree->desynced && !tree->faulted && e && e->start == va &&
		e->last == va + size - 1 && e->owner && e->owner->object == object &&
		e->offset == offset && e->access == access &&
		!(e->flags & (GPU_M4_VM_TREE_SINGLE_PAGE | GPU_M4_VM_TREE_CLOSED)) &&
		!(e->flags & GPU_M4_VM_TREE_DRIVER) == !driver;
	mutex_unlock(&tree->lock);
	return maps;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_maps);

bool gpu_m4_vm_tree_object_access(struct gpu_m4_vm_tree *tree, const void *object, u64 offset,
				  u64 size, u32 access)
{
	struct gpu_m4_vm_object *o;
	bool covered = false;

	if (IS_ERR_OR_NULL(tree) || !object || !size || offset > U64_MAX - size ||
	    !gpu_m4_drm_vm_leaf_attrs(access))
		return false;
	mutex_lock(&tree->lock);
	o = tree->desynced || tree->faulted ? NULL : gpu_m4_vm_tree_find(tree, object);
	for (u64 page = offset & ~(GPU_M4_DRM_PAGE_SIZE - 1); o && page < offset + size;
	     page += GPU_M4_DRM_PAGE_SIZE) {
		struct gpu_m4_vm_extent *e;

		covered = false;
		list_for_each_entry(e, &o->extents, link) {
			u64 bytes = e->flags & GPU_M4_VM_TREE_SINGLE_PAGE ? GPU_M4_DRM_PAGE_SIZE :
				e->last - e->start + 1;

			if (!(e->flags & GPU_M4_VM_TREE_CLOSED) && (e->access & access) == access &&
			    page >= e->offset && page - e->offset < bytes) {
				covered = true;
				break;
			}
		}
		if (!covered)
			break;
	}
	mutex_unlock(&tree->lock);
	return covered;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_object_access);

/* Every parent word names exactly its child table; leaf counts match. */
static int gpu_m4_vm_tree_structure(const struct gpu_m4_vm_tree *tree, u64 *bad_va)
{
	const struct gpu_m4_vm_table *root = tree->root;
	struct gpu_m4_vm_extent *e;
	u64 words = 0, mapped = 0;

	for (u32 i = 0; i < GPU_M4_UAT_ENTRIES; i++) {
		const struct gpu_m4_vm_table *l1 = i < GPU_M4_UAT_ROOT_ENTRIES ? root->child[i] : NULL;

		*bad_va = (u64)i << gpu_m4_uat_fmt_shift(0);
		if (gpu_m4_vm_word(root, i) != (l1 ? gpu_m4_uat_fmt_table_pte(l1->pa) : 0))
			return -EPROTO;
		for (u32 j = 0; l1 && j < GPU_M4_UAT_ENTRIES; j++) {
			const struct gpu_m4_vm_table *l2 = l1->child[j];
			u32 used = 0;

			*bad_va = l1->base | ((u64)j << gpu_m4_uat_fmt_shift(1));
			if (gpu_m4_vm_word(l1, j) != (l2 ? gpu_m4_uat_fmt_table_pte(l2->pa) : 0) ||
			    (l2 && (l2->base != *bad_va || l2->parent != l1)))
				return -EPROTO;
			for (u32 k = 0; l2 && k < GPU_M4_UAT_ENTRIES; k++)
				used += !!gpu_m4_vm_word(l2, k);
			if (l2 && used != l2->used)
				return -EPROTO;
			words += used;
		}
	}
	*bad_va = 0;
	if (words != tree->leaves)
		return -EPROTO;
	/* The extents describe exactly the valid leaves. */
	for (e = gpu_m4_vm_extent_iter_first((struct rb_root_cached *)&tree->extents, 0, U64_MAX);
	     e; e = gpu_m4_vm_extent_iter_next(e, 0, U64_MAX)) {
		for (u64 va = e->start; va < e->last; va += GPU_M4_DRM_PAGE_SIZE) {
			const struct gpu_m4_vm_table *t = gpu_m4_vm_tree_l2(tree, va);

			*bad_va = va;
			if (!t || !gpu_m4_vm_word(t, gpu_m4_uat_fmt_slot(va, 2)))
				return -EPROTO;
			mapped++;
		}
	}
	*bad_va = 0;
	return mapped == tree->leaves ? 0 : -EPROTO;
}

int gpu_m4_vm_tree_compare(struct gpu_m4_vm_tree *tree, const struct gpu_m4_drm_vm_leaf *leaf,
			   u32 count, u64 *bad_va)
{
	u64 bad = 0;
	int ret;

	if (IS_ERR_OR_NULL(tree) || (count && !leaf))
		return -EINVAL;
	mutex_lock(&tree->lock);
	if (tree->desynced) {
		mutex_unlock(&tree->lock);
		return -ESTALE;
	}
	ret = gpu_m4_vm_tree_structure(tree, &bad);
	if (!ret && tree->leaves - tree->driver_leaves != count)
		ret = -EPROTO;
	for (u32 i = 0; !ret && i < count; i++) {
		const struct gpu_m4_vm_table *t = leaf[i].va < GPU_M4_UAT_VA_LIMIT ?
			gpu_m4_vm_tree_l2(tree, leaf[i].va) : NULL;

		if (!t || !gpu_m4_drm_vm_leaf_attrs(leaf[i].access) ||
		    gpu_m4_vm_word(t, gpu_m4_uat_fmt_slot(leaf[i].va, 2)) !=
		    gpu_m4_uat_fmt_leaf_pte(leaf[i].pa, leaf[i].access)) {
			bad = leaf[i].va;
			ret = -EPROTO;
		}
	}
	gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_COMPARES, 1);
	if (ret)
		gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_MISMATCHES, 1);
	mutex_unlock(&tree->lock);
	if (bad_va)
		*bad_va = bad;
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_compare);

static int gpu_m4_vm_tree_word_order(const void *first, const void *second)
{
	const struct gpu_m4_vm_tree_word *a = first, *b = second;

	return a->va < b->va ? -1 : a->va > b->va;
}

int gpu_m4_vm_tree_compare_words(struct gpu_m4_vm_tree *tree, struct gpu_m4_vm_tree_word *word,
				 u32 count, u64 *bad_va)
{
	u64 bad = 0, drivers = 0;
	int ret;

	if (IS_ERR_OR_NULL(tree) || (count && !word))
		return -EINVAL;
	sort(word, count, sizeof(*word), gpu_m4_vm_tree_word_order, NULL);
	mutex_lock(&tree->lock);
	if (tree->desynced) {
		mutex_unlock(&tree->lock);
		return -ESTALE;
	}
	ret = gpu_m4_vm_tree_structure(tree, &bad);
	for (u32 i = 0; !ret && i < count; i++) {
		const struct gpu_m4_vm_tree_word *w = &word[i];
		const struct gpu_m4_vm_table *t = w->va < GPU_M4_UAT_VA_LIMIT ?
			gpu_m4_vm_tree_l2(tree, w->va) : NULL;
		struct gpu_m4_vm_extent *e = gpu_m4_vm_extent_iter_first(&tree->extents, w->va, w->va);

		if ((i && w->va == word[i - 1].va) || !gpu_m4_drm_aligned(w->va) || !t || !e ||
		    gpu_m4_vm_word(t, gpu_m4_uat_fmt_slot(w->va, 2)) != w->pte ||
		    !(e->flags & GPU_M4_VM_TREE_DRIVER) != !w->driver) {
			bad = w->va;
			ret = -EPROTO;
		}
		drivers += w->driver;
	}
	if (!ret && (tree->leaves != count || drivers != tree->driver_leaves))
		ret = -EPROTO;
	gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_ORACLE_COMPARES, 1);
	if (ret)
		gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_ORACLE_MISMATCHES, 1);
	mutex_unlock(&tree->lock);
	if (bad_va)
		*bad_va = bad;
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_compare_words);

void gpu_m4_vm_tree_oracle_skipped(void)
{
	gpu_m4_vm_tree_count(GPU_M4_VM_TREE_STAT_ORACLE_SKIPPED, 1);
}
EXPORT_SYMBOL_GPL(gpu_m4_vm_tree_oracle_skipped);

MODULE_DESCRIPTION("Apple M4 GPU persistent per-VM page-table tree");
#ifdef CONFIG_DEBUG_FS
static struct dentry *gpu_m4_drm_vm_tree_debugfs;

static int gpu_m4_drm_vm_tree_stats_show(struct seq_file *m, void *unused)
{
	char *buffer = kzalloc(PAGE_SIZE, GFP_KERNEL);

	if (!buffer)
		return -ENOMEM;
	gpu_m4_vm_tree_stats_get(buffer, NULL);
	seq_puts(m, buffer);
	kfree(buffer);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(gpu_m4_drm_vm_tree_stats);

static int __init gpu_m4_drm_vm_tree_debugfs_init(void)
{
	gpu_m4_drm_vm_tree_debugfs = debugfs_create_dir(KBUILD_MODNAME, NULL);
	debugfs_create_file("stats", 0400, gpu_m4_drm_vm_tree_debugfs, NULL, &gpu_m4_drm_vm_tree_stats_fops);
	return 0;
}

static void __exit gpu_m4_drm_vm_tree_debugfs_exit(void)
{
	debugfs_remove_recursive(gpu_m4_drm_vm_tree_debugfs);
}
module_init(gpu_m4_drm_vm_tree_debugfs_init);
module_exit(gpu_m4_drm_vm_tree_debugfs_exit);
#endif

MODULE_LICENSE("GPL");
