// SPDX-License-Identifier: GPL-2.0-only
/* Detached retained lower versions. No live root/table mutation or registration. */
#include <linux/atomic.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/refcount.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/timekeeping.h>
#include <asm/cacheflush.h>

#include "gpu-m4-drm-uat.h"
#include "gpu-m4-drm-uat-format.h"

/* Bounded expected-manifest chain; a deeper candidate is flattened once. */
#define GPU_M4_UAT_MAX_DEPTH 8U
/* Canonical orders of a manifest block: VA, PA and identity/offset. */
#define GPU_M4_UAT_ORDERS 3U

/*
 * One immutable table page. Its creating graph allocates, fills, cleans and
 * proves it exactly once while detached; afterwards it is never written and
 * may be named by any number of derived graphs, each holding a strong ref.
 * Children are named by stable inventory slot + 1 (0: none): a derived graph
 * keeps every shared node at its original slot and replaces a copied node in
 * place, so a shared parent's child index stays exact in every graph naming
 * it. used is the nonzero-entry count proven by the creating graph's check.
 *
 * proven is the creating graph's construction proof: set exactly once, after
 * that graph's word-by-word check of this page succeeded, and never set by any
 * later scan. It lasts as long as this allocation/image/child index does; a
 * changed mapping always gets a fresh node with its own proof, never a re-blessed
 * old one. Binding consumes it instead of re-reading the page.
 */
struct gpu_m4_drm_uat_node {
	refcount_t refs;
	u32 used;
	struct page *allocation;
	void *cpu;
	u64 pa, base;
	u16 *child;	/* level 0: 64 entries, level 1: 2048; none at level 2 */
	u8 level;
	bool proven;
};

/*
 * One immutable piece of one canonical manifest order: count consecutive
 * leaves of it. A construction allocates and fills it exactly once; it is
 * never written afterwards and is shared, by strong reference, by every
 * segment holding exactly these leaves there. A chunk fits one 4 KiB slab
 * object; every chunk of an order but its last holds at least CHUNK_MIN.
 */
#define GPU_M4_UAT_CHUNK 96U
#define GPU_M4_UAT_CHUNK_MIN 24U

struct gpu_m4_drm_uat_chunk {
	refcount_t refs;
	u32 count;
	struct gpu_m4_drm_vm_leaf leaf[];
};

/*
 * One immutable run of consecutive chunks of one order, holding a reference
 * on each, shared by strong reference by every block whose order contains it.
 * Every segment of an order but its last holds at least SEGMENT_MIN chunks,
 * so an order of n leaves has O(n / (CHUNK * SEGMENT)) segments: a rebase
 * names an untouched region by one segment reference, and within a reached
 * segment each untouched chunk by one chunk reference.
 */
#define GPU_M4_UAT_SEGMENT 32U
#define GPU_M4_UAT_SEGMENT_MIN 8U

struct gpu_m4_drm_uat_segment {
	refcount_t refs;
	u32 count;
	struct gpu_m4_drm_uat_chunk *chunk[];
};

/* One canonical order of a block: nonempty segments of nonempty chunks. */
struct gpu_m4_drm_uat_seq {
	struct gpu_m4_drm_uat_segment **segment;
	u32 segments;
};

/* A position in one order; segment == segments is its end. */
struct gpu_m4_drm_uat_cursor {
	const struct gpu_m4_drm_uat_seq *seq;
	u32 segment, chunk, at;
};

/*
 * One private mapping a block's leaves were resolved from, by its exact
 * descriptor shape: the block's leaves over [va, va + size) are exactly that
 * mapping's pages, and every private leaf of the block lies in exactly one
 * run. A rebase identifies a kept mapping by its run, never by its pages.
 */
struct gpu_m4_drm_uat_run {
	const void *identity;
	u64 va, offset, size;
	u32 access;
	enum gpu_m4_drm_cpu_policy cpu_policy;
};

/*
 * Immutable expected-manifest block: its leaves in three canonical orders,
 * VA, PA and identity/offset (VA breaks ties, see gpu_m4_drm_uat_before()),
 * each a list of shared immutable segments of chunks, plus the VA-ordered
 * runs of its private mappings. A graph's complete manifest is its block and
 * every block below; VAs are disjoint across blocks and the union proved the
 * full builder's alias contract when the top block was added. A rebase copies
 * only the chunks (and segments) its delta touches and names every other
 * segment and chunk of the base block.
 */
struct gpu_m4_drm_uat_layer {
	refcount_t refs;
	struct gpu_m4_drm_uat_layer *below;
	/* Indexed by enum gpu_m4_drm_vm_leaf_key. */
	struct gpu_m4_drm_uat_seq seq[GPU_M4_UAT_ORDERS];
	struct gpu_m4_drm_uat_run *run;
	u32 count, depth, runs;
	/* This block's entries in the shared span memo (see the memo): their
	 * list (entry index + 1, 0: none) and number. Each is a caller span fact
	 * proven once against a constructed graph whose chain names this block,
	 * every page of its span a leaf of this block or of one below it
	 * (gpu_m4_drm_uat_span_home()). That sub-chain is immutable and
	 * VA-disjoint from every block above it in any chain, so the entry holds
	 * for every graph naming the block; the block's release removes them all
	 * first. Memo lock guarded.
	 */
	u32 span_head, span_count;
	/* REMOVE-DEBUG: never-reused name for the miss attribution shadow. */
	u64 span_serial;
};

/* REMOVE-DEBUG: translation walk statistics, observational only (updated
 * under the provider lock; an occasionally torn diagnostic read is harmless).
 * Construction statistics are atomic: construct() runs without that lock.
 */
static u64 gpu_m4_uat_stat[GPU_M4_DRM_UAT_STATS];
static atomic64_t gpu_m4_uat_build_stat[GPU_M4_DRM_UAT_STATS];

static void gpu_m4_drm_uat_build_stat(u32 stat, u64 value)
{
	atomic64_add(value, &gpu_m4_uat_build_stat[stat]);
}

/* Per-graph inventory record of one owned (fresh) or shared table node. */
struct gpu_m4_drm_uat_table {
	struct page *allocation;
	void *cpu;
	u64 pa, base;
	struct gpu_m4_drm_uat_node *node;
	/* Construction only: the shared node a fresh copy replaces in place. */
	struct gpu_m4_drm_uat_node *replaces;
	u8 level;
	bool fresh;
};

struct gpu_m4_drm_uat {
	refcount_t refs;
	struct gpu_m4_drm_vm_owner *snapshot;
	struct gpu_m4_drm_uat_private *private;
	/* Complete expected manifest (held chain) and, until construction, the
	 * fresh leaves this graph adds to it.
	 */
	struct gpu_m4_drm_uat_layer *manifest;
	struct gpu_m4_drm_vm_leaf *leaf;
	struct gpu_m4_drm_uat_table *table;
	/* private[from, private_count) are the mappings this graph adds. */
	u32 private_count, held, count, fresh, user_count, from;
	u32 tables, table_limit, table_capacity, owned;
	/* Checked at construction: every fresh page was proven once against the
	 * exact manifest while still detached (shared pages by their creators),
	 * and the immutable ASID-free root descriptor (root PA | valid) recorded;
	 * the binder adds the ASID field of the TTBAT word it installs it in.
	 * proof_manifest is the exact
	 * (possibly flattened) manifest that proof was made against.
	 */
	u64 checked_low;
	const struct gpu_m4_drm_uat_layer *proof_manifest;
	/* Cross-generation rebase, prepare->construct only: the held construction
	 * base's single manifest block and the borrowed target user view (valid
	 * while the caller holds the target version).
	 */
	struct gpu_m4_drm_uat_layer *rebase;
	struct gpu_m4_drm_uat_user user;
	/* keep[i]: private[i] is exactly a run of the base block, so its leaves
	 * are the base's (not resolved). user_same: base and target name the
	 * same user generation of the VM, so the user leaves are the base's and
	 * the borrowed view is not consulted.
	 */
	bool *keep;
	bool user_same;
	/* A construction-cache image: shared nodes and manifest only, no snapshot,
	 * owner holds or residency; never bindable, only a rebase base.
	 */
	struct gpu_m4_drm_vm *image_vm;
	u64 image_generation;
	bool image;
	/* A use wrapper sharing an exact constructed image: nothing to build. */
	bool share;
	bool prepared, cleaned, published, uncertain, resident, retired;
};

/* Fixed boot RAM topology snapshot: each System RAM resource whose WHOLE span
 * already proved REGION_INTERSECTS admits any interior page without walking
 * the resource tree again (a subrange of a conflict-free span is conflict
 * free). Boundary/unverified pages and overflow use the live query. A RAM
 * hotplug/topology change would require an explicit rebuild.
 */
#define GPU_M4_UAT_RAM_RANGES 64
static struct { u64 start, end; } gpu_m4_uat_ram_range[GPU_M4_UAT_RAM_RANGES];
static u32 gpu_m4_uat_ram_count;
static bool gpu_m4_uat_ram_built;
static DEFINE_MUTEX(gpu_m4_uat_ram_lock);

static int gpu_m4_uat_ram_collect(struct resource *res, void *arg)
{
	u32 *count = arg;

	if (*count == GPU_M4_UAT_RAM_RANGES)
		return -ENOSPC;
	if (res->end > res->start &&
	    region_intersects(res->start, resource_size(res), IORESOURCE_SYSTEM_RAM,
			      IORES_DESC_NONE) == REGION_INTERSECTS) {
		gpu_m4_uat_ram_range[*count].start = res->start;
		gpu_m4_uat_ram_range[*count].end = res->end;
		(*count)++;
	}
	return 0;
}

static bool gpu_m4_drm_uat_ram_fast(u64 pa)
{
	u64 last = pa + GPU_M4_DRM_PAGE_SIZE - 1;

	if (!smp_load_acquire(&gpu_m4_uat_ram_built)) {
		mutex_lock(&gpu_m4_uat_ram_lock);
		if (!gpu_m4_uat_ram_built) {
			u32 count = 0;

			if (walk_iomem_res_desc(IORES_DESC_NONE, IORESOURCE_SYSTEM_RAM, 0,
						U64_MAX, &count, gpu_m4_uat_ram_collect) == -ENOSPC)
				count = 0; /* overflow: always use the live query */
			gpu_m4_uat_ram_count = count;
			smp_store_release(&gpu_m4_uat_ram_built, true);
		}
		mutex_unlock(&gpu_m4_uat_ram_lock);
	}
	for (u32 i = 0; i < gpu_m4_uat_ram_count; i++)
		if (pa >= gpu_m4_uat_ram_range[i].start && last <= gpu_m4_uat_ram_range[i].end)
			return true;
	return false;
}

static int gpu_m4_drm_uat_ram(u64 pa)
{
	if (!pa || (pa & ~GPU_M4_PA_MASK) || PAGE_SIZE > GPU_M4_DRM_PAGE_SIZE)
		return -EPERM;
	if (!gpu_m4_drm_uat_ram_fast(pa) &&
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

static bool gpu_m4_drm_uat_reserved(u64 va)
{
	return (va >= GPU_M4_DRM_UAT_CONTEXT_START &&
		va < GPU_M4_DRM_UAT_CONTEXT_START + GPU_M4_DRM_UAT_CONTEXT_SIZE) ||
		(va >= GPU_M4_DRM_UAT_PRIVATE_START && va < GPU_M4_DRM_UAT_PRIVATE_END);
}

static int gpu_m4_drm_uat_private_shape(const struct gpu_m4_drm_uat_private *p)
{
	if (!p->identity || !p->cookie || !p->get || !p->put || !p->page || !p->size ||
	    !gpu_m4_drm_aligned(p->va) || !gpu_m4_drm_aligned(p->offset) ||
	    !gpu_m4_drm_aligned(p->size) || p->size > ~0ULL - p->offset ||
	    !gpu_m4_drm_vm_leaf_attrs(p->access) ||
	    (p->cpu_policy != GPU_M4_DRM_CPU_NC && p->cpu_policy != GPU_M4_DRM_CPU_WRITEBACK))
		return -EINVAL;
	if (p->va == GPU_M4_DRM_UAT_CONTEXT_START)
		return p->size == GPU_M4_DRM_UAT_CONTEXT_SIZE && !p->offset &&
		       p->access == (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE) ? 0 : -EINVAL;
	return gpu_m4_drm_range(p->va, p->size, GPU_M4_DRM_UAT_PRIVATE_START,
				GPU_M4_DRM_UAT_PRIVATE_END) ? 0 : -EINVAL;
}

static bool gpu_m4_drm_uat_private_same(const struct gpu_m4_drm_uat_private *a,
					const struct gpu_m4_drm_uat_private *b)
{
	return a->identity == b->identity && a->cookie == b->cookie && a->get == b->get &&
		a->put == b->put && a->page == b->page && a->va == b->va &&
		a->offset == b->offset && a->size == b->size && a->access == b->access &&
		a->cpu_policy == b->cpu_policy;
}

static int gpu_m4_drm_uat_leaf_add(struct gpu_m4_drm_uat *v,
				  const struct gpu_m4_drm_vm_leaf *p)
{
	int ret = gpu_m4_drm_uat_ram(p->pa);

	if (ret)
		return ret;
	if (!p->identity || !gpu_m4_drm_aligned(p->va) ||
	    !gpu_m4_drm_range(p->va, GPU_M4_DRM_PAGE_SIZE, GPU_M4_DRM_PAGE_SIZE, 1ULL << 42) ||
	    !gpu_m4_drm_vm_leaf_attrs(p->access))
		return -EINVAL;
	v->leaf[v->fresh++] = *p;
	v->count++;
	return 0;
}

static int gpu_m4_drm_uat_leaf_compare(const void *first, const void *second)
{
	const struct gpu_m4_drm_vm_leaf *a = first, *b = second;

	return a->va < b->va ? -1 : a->va > b->va;
}

static int gpu_m4_drm_uat_pa_compare(const void *first, const void *second)
{
	u64 a = *(const u64 *)first, b = *(const u64 *)second;

	return a < b ? -1 : a > b;
}

/*
 * Canonical block orders: PA or (identity, offset), VA breaking ties. VAs are
 * distinct within a valid block, so each is a strict total order and an order
 * updated from a base block's by the sorted delta holds exactly the sequence
 * sorted from scratch. Lookups compare the key alone, which this refinement
 * keeps sorted.
 */
static bool gpu_m4_drm_uat_before(const struct gpu_m4_drm_vm_leaf *a,
	const struct gpu_m4_drm_vm_leaf *b, enum gpu_m4_drm_vm_leaf_key key)
{
	if (gpu_m4_drm_vm_leaf_less(a, b, key))
		return true;
	return !gpu_m4_drm_vm_leaf_less(b, a, key) && a->va < b->va;
}

static bool gpu_m4_drm_uat_leaf_same(const struct gpu_m4_drm_vm_leaf *a,
				     const struct gpu_m4_drm_vm_leaf *b)
{
	return a->va == b->va && a->pa == b->pa && a->object_offset == b->object_offset &&
		a->identity == b->identity && a->access == b->access &&
		a->cpu_policy == b->cpu_policy;
}

/* Canonical pointer orders for sort(), VA breaking ties. */
static int gpu_m4_drm_uat_pa_order(const void *first, const void *second)
{
	const struct gpu_m4_drm_vm_leaf *a = *(const struct gpu_m4_drm_vm_leaf *const *)first;
	const struct gpu_m4_drm_vm_leaf *b = *(const struct gpu_m4_drm_vm_leaf *const *)second;

	return gpu_m4_drm_uat_before(a, b, GPU_M4_DRM_VM_KEY_PA) ? -1 :
		gpu_m4_drm_uat_before(b, a, GPU_M4_DRM_VM_KEY_PA);
}

static int gpu_m4_drm_uat_id_order(const void *first, const void *second)
{
	const struct gpu_m4_drm_vm_leaf *a = *(const struct gpu_m4_drm_vm_leaf *const *)first;
	const struct gpu_m4_drm_vm_leaf *b = *(const struct gpu_m4_drm_vm_leaf *const *)second;

	return gpu_m4_drm_uat_before(a, b, GPU_M4_DRM_VM_KEY_IDENTITY) ? -1 :
		gpu_m4_drm_uat_before(b, a, GPU_M4_DRM_VM_KEY_IDENTITY);
}

static int gpu_m4_drm_uat_run_compare(const void *first, const void *second)
{
	const struct gpu_m4_drm_uat_run *a = first, *b = second;

	return a->va < b->va ? -1 : a->va > b->va;
}

/* Canonical PA and identity views of n VA-ordered leaves. */
static void gpu_m4_drm_uat_views(const struct gpu_m4_drm_vm_leaf *leaf, u32 n,
	const struct gpu_m4_drm_vm_leaf **pa, const struct gpu_m4_drm_vm_leaf **id)
{
	for (u32 i = 0; i < n; i++)
		pa[i] = id[i] = &leaf[i];
	sort(pa, n, sizeof(*pa), gpu_m4_drm_uat_pa_order, NULL);
	sort(id, n, sizeof(*id), gpu_m4_drm_uat_id_order, NULL);
	gpu_m4_drm_uat_build_stat(GPU_M4_DRM_UAT_STAT_SORTED, 2 * (u64)n);
}

/* The builder's identity rule over a canonical identity view: an identity
 * names one CPU policy and an identity+offset one PA.
 */
static int gpu_m4_drm_uat_ids_check(const struct gpu_m4_drm_vm_leaf *const *id, u32 n)
{
	for (u32 i = 1; i < n; i++) {
		const struct gpu_m4_drm_vm_leaf *a = id[i - 1], *b = id[i];

		if (a->identity == b->identity && (a->cpu_policy != b->cpu_policy ||
		    (a->object_offset == b->object_offset && a->pa != b->pa)))
			return -EPROTO;
	}
	return 0;
}

/* The builder's PA rule over a canonical PA view: a repeated PA names the
 * same object page and policy.
 */
static int gpu_m4_drm_uat_pas_check(const struct gpu_m4_drm_vm_leaf *const *pa, u32 n)
{
	for (u32 i = 1; i < n; i++) {
		const struct gpu_m4_drm_vm_leaf *a = pa[i - 1], *b = pa[i];

		if (a->pa == b->pa && (a->identity != b->identity ||
		    a->object_offset != b->object_offset || a->cpu_policy != b->cpu_policy))
			return -EPERM;
	}
	return 0;
}

static void gpu_m4_drm_uat_chunk_put(struct gpu_m4_drm_uat_chunk *c)
{
	if (c && refcount_dec_and_test(&c->refs))
		kfree(c);
}

/* A fresh chunk of count leaves, copied directly or through pointers. */
static struct gpu_m4_drm_uat_chunk *gpu_m4_drm_uat_chunk_new(
	const struct gpu_m4_drm_vm_leaf *leaf, const struct gpu_m4_drm_vm_leaf *const *from,
	u32 count)
{
	struct gpu_m4_drm_uat_chunk *c;

	c = kmalloc(sizeof(*c) + array_size(count, sizeof(*c->leaf)), GFP_KERNEL);
	if (!c)
		return NULL;
	refcount_set(&c->refs, 1);
	c->count = count;
	if (from)
		for (u32 i = 0; i < count; i++)
			c->leaf[i] = *from[i];
	else
		memcpy(c->leaf, leaf, array_size(count, sizeof(*c->leaf)));
	gpu_m4_drm_uat_build_stat(GPU_M4_DRM_UAT_STAT_CHUNKS_BUILT, 1);
	return c;
}

static void gpu_m4_drm_uat_segment_put(struct gpu_m4_drm_uat_segment *g)
{
	if (!g || !refcount_dec_and_test(&g->refs))
		return;
	for (u32 i = 0; i < g->count; i++)
		gpu_m4_drm_uat_chunk_put(g->chunk[i]);
	kfree(g);
}

/* An empty segment with room for count chunks. */
static struct gpu_m4_drm_uat_segment *gpu_m4_drm_uat_segment_alloc(u32 count)
{
	struct gpu_m4_drm_uat_segment *g;

	g = kmalloc(sizeof(*g) + array_size(count, sizeof(*g->chunk)), GFP_KERNEL);
	if (!g)
		return NULL;
	refcount_set(&g->refs, 1);
	g->count = 0;
	return g;
}

static const struct gpu_m4_drm_vm_leaf *gpu_m4_drm_uat_segment_last(
	const struct gpu_m4_drm_uat_segment *g)
{
	const struct gpu_m4_drm_uat_chunk *c = g->chunk[g->count - 1];

	return &c->leaf[c->count - 1];
}

/* Drop an order's segment references; its pointer array lives in its block. */
static void gpu_m4_drm_uat_seq_put(struct gpu_m4_drm_uat_seq *s)
{
	for (u32 i = 0; i < s->segments; i++)
		gpu_m4_drm_uat_segment_put(s->segment[i]);
	s->segments = 0;
}

/* Parts of an even cut of n into parts of at most max (each at least max / 2
 * when there are several).
 */
static u32 gpu_m4_drm_uat_pieces(u32 n, u32 max)
{
	return n / max + !!(n % max);
}

/* Segments of an order of n leaves cut from scratch. */
static u32 gpu_m4_drm_uat_order_segments(u32 n)
{
	return gpu_m4_drm_uat_pieces(gpu_m4_drm_uat_pieces(n, GPU_M4_UAT_CHUNK),
				     GPU_M4_UAT_SEGMENT);
}

/*
 * One order of a new block from its count leaves in that order (directly or
 * through pointers): even chunks, evenly grouped into segments appended to s
 * (sized by order_segments()). On failure what was built stays in s.
 */
static int gpu_m4_drm_uat_seq_build(struct gpu_m4_drm_uat_seq *s,
	const struct gpu_m4_drm_vm_leaf *leaf, const struct gpu_m4_drm_vm_leaf *const *from,
	u32 count)
{
	u32 chunks = gpu_m4_drm_uat_pieces(count, GPU_M4_UAT_CHUNK);
	u32 segments = gpu_m4_drm_uat_pieces(chunks, GPU_M4_UAT_SEGMENT);

	for (u32 j = 0; j < segments; j++) {
		u32 first = (u32)((u64)chunks * j / segments);
		u32 end = (u32)((u64)chunks * (j + 1) / segments);
		struct gpu_m4_drm_uat_segment *g = gpu_m4_drm_uat_segment_alloc(end - first);

		if (!g)
			return -ENOMEM;
		s->segment[s->segments++] = g;
		for (u32 c = first; c < end; c++) {
			u32 at = (u32)((u64)count * c / chunks);
			u32 to = (u32)((u64)count * (c + 1) / chunks);
			struct gpu_m4_drm_uat_chunk *k = gpu_m4_drm_uat_chunk_new(
				from ? NULL : leaf + at, from ? from + at : NULL, to - at);

			if (!k)
				return -ENOMEM;
			g->chunk[g->count++] = k;
		}
	}
	return 0;
}

static bool gpu_m4_drm_uat_same_key(const struct gpu_m4_drm_vm_leaf *a,
	const struct gpu_m4_drm_vm_leaf *b, enum gpu_m4_drm_vm_leaf_key key)
{
	return !gpu_m4_drm_uat_before(a, b, key) && !gpu_m4_drm_uat_before(b, a, key);
}

/* Cut count pending leaves into even chunks at chunk[*chunks] (capacity cap). */
static int gpu_m4_drm_uat_chunk_cut(struct gpu_m4_drm_uat_chunk **chunk, u32 *chunks, u32 cap,
	const struct gpu_m4_drm_vm_leaf *leaf, const struct gpu_m4_drm_vm_leaf *const *from,
	u32 count)
{
	u32 pieces = gpu_m4_drm_uat_pieces(count, GPU_M4_UAT_CHUNK), at = 0;

	if (pieces > cap - *chunks)
		return -EPROTO;
	for (u32 i = 0; i < pieces; i++) {
		u32 end = (u32)((u64)count * (i + 1) / pieces);
		struct gpu_m4_drm_uat_chunk *c = gpu_m4_drm_uat_chunk_new(from ? NULL : leaf + at,
			from ? from + at : NULL, end - at);

		if (!c)
			return -ENOMEM;
		chunk[(*chunks)++] = c;
		at = end;
	}
	return 0;
}

/*
 * Group count chunks (the caller's references) into even segments at
 * segment[*segments] (capacity cap). Adopts all of them or, on failure,
 * none: every segment is allocated before any is filled.
 */
static int gpu_m4_drm_uat_segment_cut(struct gpu_m4_drm_uat_segment **segment, u32 *segments,
	u32 cap, struct gpu_m4_drm_uat_chunk *const *chunk, u32 count)
{
	u32 pieces = gpu_m4_drm_uat_pieces(count, GPU_M4_UAT_SEGMENT), at = 0;

	if (pieces > cap - *segments)
		return -EPROTO;
	for (u32 i = 0; i < pieces; i++) {
		u32 end = (u32)((u64)count * (i + 1) / pieces);

		segment[*segments + i] = gpu_m4_drm_uat_segment_alloc(end - at);
		if (!segment[*segments + i]) {
			while (i)
				kfree(segment[*segments + --i]);
			return -ENOMEM;
		}
		at = end;
	}
	at = 0;
	for (u32 i = 0; i < pieces; i++) {
		struct gpu_m4_drm_uat_segment *g = segment[(*segments)++];
		u32 end = (u32)((u64)count * (i + 1) / pieces);

		memcpy(g->chunk, chunk + at, array_size(end - at, sizeof(*chunk)));
		g->count = end - at;
		at = end;
	}
	return 0;
}

/*
 * Merge chunk k with its operations del[*d, de) and ins[*i, ie) into
 * pending[*m...] (capacity cap): deletions must name its leaves exactly and
 * no insertion one of its kept leaves (else -EPROTO).
 */
static int gpu_m4_drm_uat_chunk_merge(const struct gpu_m4_drm_uat_chunk *k,
	enum gpu_m4_drm_vm_leaf_key key, const struct gpu_m4_drm_vm_leaf *const *del, u32 *d,
	u32 de, const struct gpu_m4_drm_vm_leaf *const *ins, u32 *i, u32 ie,
	struct gpu_m4_drm_vm_leaf *pending, u32 *m, u32 cap)
{
	if (k->count > GPU_M4_UAT_CHUNK || (u64)*m + k->count + (ie - *i) > cap)
		return -EPROTO;
	for (u32 e = 0; e < k->count; e++) {
		const struct gpu_m4_drm_vm_leaf *x = &k->leaf[e];

		while (*i < ie && gpu_m4_drm_uat_before(ins[*i], x, key))
			pending[(*m)++] = *ins[(*i)++];
		if (*d < de && gpu_m4_drm_uat_same_key(del[*d], x, key)) {
			if (!gpu_m4_drm_uat_leaf_same(del[(*d)++], x))
				return -EPROTO;
			continue;
		}
		if ((*d < de && gpu_m4_drm_uat_before(del[*d], x, key)) ||
		    (*i < ie && gpu_m4_drm_uat_same_key(ins[*i], x, key)))
			return -EPROTO;
		pending[(*m)++] = *x;
	}
	while (*i < ie)
		pending[(*m)++] = *ins[(*i)++];
	return *d == de ? 0 : -EPROTO;
}

/*
 * The segment of old, at or after from, that the earlier of two next
 * operations falls into (the first whose last leaf is not before it; the
 * final segment takes all beyond); old->segments when there is none.
 */
static u32 gpu_m4_drm_uat_seq_next(const struct gpu_m4_drm_uat_seq *old, u32 from,
	const struct gpu_m4_drm_vm_leaf *del, const struct gpu_m4_drm_vm_leaf *ins,
	enum gpu_m4_drm_vm_leaf_key key)
{
	const struct gpu_m4_drm_vm_leaf *next = !del ||
		(ins && gpu_m4_drm_uat_before(ins, del, key)) ? ins : del;
	u32 first = from, end = old->segments - 1;

	if (!next)
		return old->segments;
	while (first < end) {
		u32 middle = first + (end - first) / 2;

		const struct gpu_m4_drm_vm_leaf *last =
			gpu_m4_drm_uat_segment_last(old->segment[middle]);

		if (gpu_m4_drm_uat_before(last, next, key))
			first = middle + 1;
		else
			end = middle;
	}
	return first;
}

/* Rebuilt-chunk room a seq_update() needs for ni insertions. */
static u64 gpu_m4_drm_uat_update_chunks(u32 ni)
{
	return 2ULL * GPU_M4_UAT_SEGMENT + ni + 4;
}

/* Segment room a seq_update() of old needs for ni insertions. */
static u64 gpu_m4_drm_uat_update_segments(const struct gpu_m4_drm_uat_seq *old, u32 ni)
{
	return 6ULL * old->segments + ni + 4;
}

/*
 * Copy-on-write successor of one order: old minus del plus ins, both sorted
 * in this canonical order, every deletion naming a leaf of old exactly and no
 * insertion an existing one (else -EPROTO), into segment[] (room for
 * update_segments()). A segment no operation reaches is shared by one
 * reference. In a reached one each chunk without an operation is shared,
 * each touched chunk merged with its operations and cut again (carried on
 * into its successor while below CHUNK_MIN), and the rebuilt run of chunks
 * (chunk[], room for update_chunks()) grouped into segments again (carried
 * on while below SEGMENT_MIN). pending holds CHUNK + CHUNK_MIN + ni leaves.
 * O(segments + reached segments * SEGMENT + touched chunks * CHUNK +
 * operations): never a pass over the chunks or leaves. On failure no
 * reference is left in segment[].
 */
static int gpu_m4_drm_uat_seq_update(struct gpu_m4_drm_uat_segment **segment, u32 *segments,
	const struct gpu_m4_drm_uat_seq *old, enum gpu_m4_drm_vm_leaf_key key,
	const struct gpu_m4_drm_vm_leaf *const *del, u32 nd,
	const struct gpu_m4_drm_vm_leaf *const *ins, u32 ni,
	struct gpu_m4_drm_uat_chunk **chunk, struct gpu_m4_drm_vm_leaf *pending, u64 *touched)
{
	u32 cap = (u32)gpu_m4_drm_uat_update_segments(old, ni);
	u32 room = (u32)gpu_m4_drm_uat_update_chunks(ni);
	u32 pend = GPU_M4_UAT_CHUNK + GPU_M4_UAT_CHUNK_MIN + ni;
	u32 shared = 0, d = 0, i = 0, m = 0, x = 0;
	int ret = 0;

	*segments = 0;
	for (u32 g = 0; !ret && g < old->segments; g++) {
		const struct gpu_m4_drm_uat_segment *seg;
		bool last;

		if (!m && !x) {
			/* Share every segment before the next operation's. */
			u32 to = gpu_m4_drm_uat_seq_next(old, g, d < nd ? del[d] : NULL,
							 i < ni ? ins[i] : NULL, key);

			for (; g < to; g++) {
				refcount_inc(&old->segment[g]->refs);
				segment[(*segments)++] = old->segment[g];
				shared += old->segment[g]->count;
			}
			if (g == old->segments)
				break;
		}
		seg = old->segment[g];
		last = g + 1 == old->segments;
		for (u32 c = 0; !ret && c < seg->count; c++) {
			struct gpu_m4_drm_uat_chunk *k = seg->chunk[c];
			const struct gpu_m4_drm_vm_leaf *end = &k->leaf[k->count - 1];
			bool final = last && c + 1 == seg->count;
			u32 de = d, ie = i;

			/* Its operations: those up to its last leaf, all after it. */
			while (de < nd && (final || !gpu_m4_drm_uat_before(end, del[de], key)))
				de++;
			while (ie < ni && (final || !gpu_m4_drm_uat_before(end, ins[ie], key)))
				ie++;
			if (!m && de == d && ie == i) {
				ret = x < room ? 0 : -EPROTO;
				if (!ret) {
					refcount_inc(&k->refs);
					chunk[x++] = k;
					shared++;
				}
			} else {
				ret = gpu_m4_drm_uat_chunk_merge(k, key, del, &d, de, ins, &i, ie,
								 pending, &m, pend);
				if (!ret && (m >= GPU_M4_UAT_CHUNK_MIN || final)) {
					ret = gpu_m4_drm_uat_chunk_cut(chunk, &x, room, pending,
								       NULL, m);
					*touched += m;
					gpu_m4_drm_uat_build_stat(GPU_M4_DRM_UAT_STAT_MERGED, m);
					m = 0;
				}
			}
			/* A full segment of rebuilt chunks needs no later neighbour. */
			if (!ret && x >= 2 * GPU_M4_UAT_SEGMENT) {
				ret = gpu_m4_drm_uat_segment_cut(segment, segments, cap, chunk,
								 GPU_M4_UAT_SEGMENT);
				if (!ret) {
					x -= GPU_M4_UAT_SEGMENT;
					memmove(chunk, chunk + GPU_M4_UAT_SEGMENT,
						array_size(x, sizeof(*chunk)));
				}
			}
		}
		if (ret || m || (x < GPU_M4_UAT_SEGMENT_MIN && !last))
			continue;
		ret = gpu_m4_drm_uat_segment_cut(segment, segments, cap, chunk, x);
		if (!ret)
			x = 0;
	}
	if (!ret && !old->segments) {
		ret = gpu_m4_drm_uat_chunk_cut(chunk, &x, room, NULL, ins, ni);
		if (!ret)
			ret = gpu_m4_drm_uat_segment_cut(segment, segments, cap, chunk, x);
		if (!ret)
			x = 0;
		*touched += ni;
		i = ni;
	}
	if (!ret && (d != nd || i != ni || m || x))
		ret = -EPROTO;
	if (ret) {
		for (u32 c = 0; c < x; c++)
			gpu_m4_drm_uat_chunk_put(chunk[c]);
		for (u32 s = 0; s < *segments; s++)
			gpu_m4_drm_uat_segment_put(segment[s]);
		*segments = 0;
		return ret;
	}
	gpu_m4_drm_uat_build_stat(GPU_M4_DRM_UAT_STAT_CHUNKS_SHARED, shared);
	return 0;
}

/*
 * First position of order s whose key is not below probe's (the key alone,
 * which the canonical refinement keeps sorted): one search over the
 * segments' last leaves, one over the chunks of that segment, one within the
 * chunk. O(log n).
 */
static struct gpu_m4_drm_uat_cursor gpu_m4_drm_uat_seq_bound(const struct gpu_m4_drm_uat_seq *s,
	const struct gpu_m4_drm_vm_leaf *probe, enum gpu_m4_drm_vm_leaf_key key)
{
	struct gpu_m4_drm_uat_cursor c = { .seq = s };
	const struct gpu_m4_drm_uat_segment *g;
	const struct gpu_m4_drm_uat_chunk *k;
	u32 first = 0, end = s->segments;

	while (first < end) {
		u32 middle = first + (end - first) / 2;

		const struct gpu_m4_drm_vm_leaf *last =
			gpu_m4_drm_uat_segment_last(s->segment[middle]);

		if (gpu_m4_drm_vm_leaf_less(last, probe, key))
			first = middle + 1;
		else
			end = middle;
	}
	c.segment = first;
	if (first == s->segments)
		return c;
	g = s->segment[first];
	first = 0;
	end = g->count;
	while (first < end) {
		u32 middle = first + (end - first) / 2;

		k = g->chunk[middle];
		if (gpu_m4_drm_vm_leaf_less(&k->leaf[k->count - 1], probe, key))
			first = middle + 1;
		else
			end = middle;
	}
	c.chunk = first;
	k = g->chunk[first];
	first = 0;
	end = k->count;
	while (first < end) {
		u32 middle = first + (end - first) / 2;

		if (gpu_m4_drm_vm_leaf_less(&k->leaf[middle], probe, key))
			first = middle + 1;
		else
			end = middle;
	}
	c.at = first;
	return c;
}

static const struct gpu_m4_drm_vm_leaf *gpu_m4_drm_uat_cursor_leaf(
	const struct gpu_m4_drm_uat_cursor *c)
{
	return c->segment < c->seq->segments ?
		&c->seq->segment[c->segment]->chunk[c->chunk]->leaf[c->at] : NULL;
}

static void gpu_m4_drm_uat_cursor_next(struct gpu_m4_drm_uat_cursor *c)
{
	const struct gpu_m4_drm_uat_segment *g = c->seq->segment[c->segment];

	if (++c->at < g->chunk[c->chunk]->count)
		return;
	c->at = 0;
	if (++c->chunk < g->count)
		return;
	c->chunk = 0;
	c->segment++;
}

/* First leaf of order s whose key equals probe's, or NULL. */
static const struct gpu_m4_drm_vm_leaf *gpu_m4_drm_uat_seq_find(
	const struct gpu_m4_drm_uat_seq *s, const struct gpu_m4_drm_vm_leaf *probe,
	enum gpu_m4_drm_vm_leaf_key key)
{
	struct gpu_m4_drm_uat_cursor c = gpu_m4_drm_uat_seq_bound(s, probe, key);
	const struct gpu_m4_drm_vm_leaf *p = gpu_m4_drm_uat_cursor_leaf(&c);

	return p && !gpu_m4_drm_vm_leaf_less(probe, p, key) ? p : NULL;
}

static void gpu_m4_drm_uat_node_put(struct gpu_m4_drm_uat_node *n)
{
	if (!n || !refcount_dec_and_test(&n->refs))
		return;
	__free_pages(n->allocation, get_order(GPU_M4_DRM_PAGE_SIZE));
	kvfree(n->child);
	kfree(n);
}

/*
 * One block's metadata in one allocation: the block, its runs and the
 * segment pointer arrays of its three orders (room for count[k] segments
 * each); segments and chunks are separate shared allocations.
 */
static struct gpu_m4_drm_uat_layer *gpu_m4_drm_uat_layer_alloc(u32 runs, const u32 *count)
{
	u64 segments = (u64)count[0] + count[1] + count[2];
	struct gpu_m4_drm_uat_segment **segment;
	struct gpu_m4_drm_uat_layer *l;

	l = kvzalloc(sizeof(*l) + array_size(runs, sizeof(*l->run)) +
		     array_size(segments, sizeof(*segment)), GFP_KERNEL);
	if (!l)
		return NULL;
	l->run = runs ? (struct gpu_m4_drm_uat_run *)(l + 1) : NULL;
	segment = (struct gpu_m4_drm_uat_segment **)((struct gpu_m4_drm_uat_run *)(l + 1) + runs);
	for (u32 k = 0; k < GPU_M4_UAT_ORDERS; k++) {
		l->seq[k].segment = segment;
		segment += count[k];
	}
	return l;
}

static void gpu_m4_drm_uat_span_purge(struct gpu_m4_drm_uat_layer *l);

static void gpu_m4_drm_uat_layer_free(struct gpu_m4_drm_uat_layer *l)
{
	/* No graph names l any more: its memo entries go before it does. The
	 * count is memo-locked state that CLOCK eviction changes concurrently,
	 * so always purge; the locked purge of an empty list is harmless.
	 */
	gpu_m4_drm_uat_span_purge(l);
	for (u32 k = 0; k < GPU_M4_UAT_ORDERS; k++)
		gpu_m4_drm_uat_seq_put(&l->seq[k]);
	kvfree(l);
}

static void gpu_m4_drm_uat_layer_put(struct gpu_m4_drm_uat_layer *l)
{
	while (l && refcount_dec_and_test(&l->refs)) {
		struct gpu_m4_drm_uat_layer *below = l->below;

		gpu_m4_drm_uat_layer_free(l);
		l = below;
	}
}

/* A new block's three orders: its VA-ordered leaves and canonical views. */
static int gpu_m4_drm_uat_layer_fill(struct gpu_m4_drm_uat_layer *l,
	const struct gpu_m4_drm_vm_leaf *leaf, const struct gpu_m4_drm_vm_leaf *const *pa,
	const struct gpu_m4_drm_vm_leaf *const *id, u32 n)
{
	int ret = gpu_m4_drm_uat_seq_build(&l->seq[GPU_M4_DRM_VM_KEY_VA], leaf, NULL, n);

	if (!ret)
		ret = gpu_m4_drm_uat_seq_build(&l->seq[GPU_M4_DRM_VM_KEY_PA], NULL, pa, n);
	if (!ret)
		ret = gpu_m4_drm_uat_seq_build(&l->seq[GPU_M4_DRM_VM_KEY_IDENTITY], NULL, id, n);
	l->count = n;
	return ret;
}

/* A new block's runs (allocated with it): the private mappings it adds, by
 * shape, in VA order.
 */
static void gpu_m4_drm_uat_layer_runs(struct gpu_m4_drm_uat_layer *l,
	const struct gpu_m4_drm_uat_private *p, u32 count)
{
	for (u32 i = 0; i < count; i++)
		l->run[i] = (struct gpu_m4_drm_uat_run) {
			.identity = p[i].identity, .va = p[i].va, .offset = p[i].offset,
			.size = p[i].size, .access = p[i].access, .cpu_policy = p[i].cpu_policy,
		};
	l->runs = count;
	sort(l->run, count, sizeof(*l->run), gpu_m4_drm_uat_run_compare, NULL);
}

/* The run of block l with exactly p's shape, or -ENOENT. */
static int gpu_m4_drm_uat_run_find(const struct gpu_m4_drm_uat_layer *l,
				   const struct gpu_m4_drm_uat_private *p)
{
	u32 first = 0, end = l->runs;
	const struct gpu_m4_drm_uat_run *r;

	while (first < end) {
		u32 middle = first + (end - first) / 2;

		if (l->run[middle].va < p->va)
			first = middle + 1;
		else
			end = middle;
	}
	if (first == l->runs)
		return -ENOENT;
	r = &l->run[first];
	return r->va == p->va && r->size == p->size && r->identity == p->identity &&
		r->offset == p->offset && r->access == p->access &&
		r->cpu_policy == p->cpu_policy ? (int)first : -ENOENT;
}

static const struct gpu_m4_drm_vm_leaf *gpu_m4_drm_uat_layer_va(
	const struct gpu_m4_drm_uat_layer *l, u64 va)
{
	struct gpu_m4_drm_vm_leaf probe = { .va = va };

	for (; l; l = l->below) {
		const struct gpu_m4_drm_vm_leaf *p = gpu_m4_drm_uat_seq_find(
			&l->seq[GPU_M4_DRM_VM_KEY_VA], &probe, GPU_M4_DRM_VM_KEY_VA);

		if (p)
			return p;
	}
	return NULL;
}

static const struct gpu_m4_drm_vm_leaf *gpu_m4_drm_uat_layer_pa(
	const struct gpu_m4_drm_uat_layer *l, u64 pa)
{
	struct gpu_m4_drm_vm_leaf probe = { .pa = pa };

	for (; l; l = l->below) {
		const struct gpu_m4_drm_vm_leaf *p = gpu_m4_drm_uat_seq_find(
			&l->seq[GPU_M4_DRM_VM_KEY_PA], &probe, GPU_M4_DRM_VM_KEY_PA);

		if (p)
			return p;
	}
	return NULL;
}

/* The builder's identity rule against an already consistent manifest: one
 * member of the identity (and identity+offset) group represents all of it.
 */
static int gpu_m4_drm_uat_layer_identity(const struct gpu_m4_drm_uat_layer *l,
					 const struct gpu_m4_drm_vm_leaf *p)
{
	struct gpu_m4_drm_vm_leaf first = { .identity = p->identity };

	for (; l; l = l->below) {
		const struct gpu_m4_drm_uat_seq *s = &l->seq[GPU_M4_DRM_VM_KEY_IDENTITY];
		struct gpu_m4_drm_uat_cursor c = gpu_m4_drm_uat_seq_bound(s, &first,
			GPU_M4_DRM_VM_KEY_IDENTITY);
		const struct gpu_m4_drm_vm_leaf *q = gpu_m4_drm_uat_cursor_leaf(&c);

		if (!q || q->identity != p->identity)
			continue;
		if (q->cpu_policy != p->cpu_policy)
			return -EPROTO;
		q = gpu_m4_drm_uat_seq_find(s, p, GPU_M4_DRM_VM_KEY_IDENTITY);
		if (q && q->pa != p->pa)
			return -EPROTO;
	}
	return 0;
}

/* The builder's PA rule: a repeated PA names the same object page/policy. */
static int gpu_m4_drm_uat_layer_alias(const struct gpu_m4_drm_uat_layer *l,
				      const struct gpu_m4_drm_vm_leaf *p)
{
	const struct gpu_m4_drm_vm_leaf *q = gpu_m4_drm_uat_layer_pa(l, p->pa);

	if (q && (q->identity != p->identity || q->object_offset != p->object_offset ||
		  q->cpu_policy != p->cpu_policy))
		return -EPERM;
	return 0;
}

/* ---- Span memo: placement, lookup and inheritance (see the block) ---- */

/*
 * The span memo, shared by every block of every VM: GPU_M4_UAT_SPAN_CACHE
 * entries, each naming its block by pointer. A block's entries are removed
 * before the block is freed (gpu_m4_drm_uat_span_purge()), so the pointer
 * never dangles or names a later block, and an entry answers only lookups
 * whose chain names that exact block. Fixed size: its memory never grows
 * with any client's blocks or keys (a block itself costs two words), so it
 * stays outside the per-version charge. Reuse distances of thousands of
 * keys (many render units cycling) fit; an open-addressed (block, key)
 * index, at most half full, keeps lookups O(1). Replacement is CLOCK: an
 * entry hit since the hand last passed it is spared once, so keys proven
 * once per job never flush the stable set every job reuses. Replacement
 * only forgets: it never affects an answer's validity. The memo lock is
 * innermost: lookups and remembers take it under the provider lock,
 * construction (carry) and block release alone.
 */
#define GPU_M4_UAT_SPAN_CACHE 8192U
#define GPU_M4_UAT_SPAN_SLOTS (2 * GPU_M4_UAT_SPAN_CACHE)

struct gpu_m4_drm_uat_span_entry {
	struct gpu_m4_drm_uat_span_key key;
	struct gpu_m4_drm_uat_layer *block;	/* NULL: free */
	u16 prev, next;		/* the block's list or the free list (entry + 1) */
	bool referenced;	/* hit since the hand last passed it */
	bool inherited;		/* REMOVE-DEBUG: carried from a base block */
};

static DEFINE_MUTEX(gpu_m4_uat_span_lock);
static struct gpu_m4_drm_uat_span_entry gpu_m4_uat_span[GPU_M4_UAT_SPAN_CACHE];
/* Entry index + 1 per slot (0: empty), linear probing by span_hash(). */
static u16 gpu_m4_uat_span_slot[GPU_M4_UAT_SPAN_SLOTS];
/* Entries ever used, free list head (entry + 1), CLOCK hand. */
static u32 gpu_m4_uat_span_used, gpu_m4_uat_span_free, gpu_m4_uat_span_hand;

static bool gpu_m4_drm_uat_span_same(const struct gpu_m4_drm_uat_span_key *a,
	const struct gpu_m4_drm_uat_span_key *b)
{
	return a->kind == b->kind && a->identity == b->identity && a->aux == b->aux &&
		a->va == b->va && a->size == b->size && a->offset == b->offset &&
		a->aux_value == b->aux_value && a->access == b->access;
}

/* Every field of a key mixed, so keys that differ only by object, offset,
 * size or kind spread too.
 */
static u64 gpu_m4_drm_uat_span_mix(const struct gpu_m4_drm_uat_span_key *k)
{
	u64 h = k->va ^ (k->size << 17) ^ (k->offset << 29) ^ k->aux_value ^
		(u64)(unsigned long)k->identity ^ ((u64)(unsigned long)k->aux << 7) ^
		((u64)k->kind << 56) ^ ((u64)k->access << 60);

	h ^= h >> 31;
	return h * 0x9e3779b97f4a7c15ULL;
}

/* Home slot of (block, key) in the index. */
static u32 gpu_m4_drm_uat_span_hash(const struct gpu_m4_drm_uat_layer *l,
	const struct gpu_m4_drm_uat_span_key *k)
{
	u64 h = gpu_m4_drm_uat_span_mix(k) ^ ((u64)(unsigned long)l * 0xff51afd7ed558ccdULL);

	h ^= h >> 29;
	h *= 0x9e3779b97f4a7c15ULL;
	return (u32)(h >> 40) & (GPU_M4_UAT_SPAN_SLOTS - 1);
}

/* REMOVE-DEBUG: where recently remembered keys went (miss attribution only).
 * Direct-mapped by the key's mix (twice the memo), memo lock guarded, never
 * updated by a carry. Blocks are named by a never-reused serial; a later key
 * of the same slot overwrites, so a long-unused key reads as absent.
 * Compact 32-bit tags and counters: approximate by design.
 */
#define GPU_M4_UAT_SPAN_SHADOW (2 * GPU_M4_UAT_SPAN_CACHE)
static struct {
	u32 tag, block, inserted;
	bool evicted;
} gpu_m4_uat_span_shadow[GPU_M4_UAT_SPAN_SHADOW];
static u64 gpu_m4_uat_span_serial;
static u32 gpu_m4_uat_span_inserted;

static void gpu_m4_drm_uat_span_shadow(struct gpu_m4_drm_uat_layer *l,
	const struct gpu_m4_drm_uat_span_key *k, bool evicted)
{
	u64 mix = gpu_m4_drm_uat_span_mix(k);
	u32 i = (u32)(mix >> 50) & (GPU_M4_UAT_SPAN_SHADOW - 1);

	if (!l->span_serial)
		l->span_serial = ++gpu_m4_uat_span_serial;
	if (!evicted) {
		gpu_m4_uat_span_shadow[i].tag = (u32)mix;
		gpu_m4_uat_span_shadow[i].inserted = gpu_m4_uat_span_inserted;
		gpu_m4_uat_span_shadow[i].block = (u32)l->span_serial;
		gpu_m4_uat_span_shadow[i].evicted = false;
	} else if (gpu_m4_uat_span_shadow[i].tag == (u32)mix &&
		   gpu_m4_uat_span_shadow[i].block == (u32)l->span_serial) {
		gpu_m4_uat_span_shadow[i].evicted = true;
	}
}

/* Entry index of block l's key, or -1. The index is at most half full, so
 * a probe always ends at an empty slot.
 */
static int gpu_m4_drm_uat_span_find(const struct gpu_m4_drm_uat_layer *l,
	const struct gpu_m4_drm_uat_span_key *key)
{
	if (!l->span_count)
		return -1;
	for (u32 s = gpu_m4_drm_uat_span_hash(l, key);; s = (s + 1) & (GPU_M4_UAT_SPAN_SLOTS - 1)) {
		u16 at = gpu_m4_uat_span_slot[s];

		if (!at)
			return -1;
		if (gpu_m4_uat_span[at - 1].block == l &&
		    gpu_m4_drm_uat_span_same(&gpu_m4_uat_span[at - 1].key, key))
			return at - 1;
	}
}

/* Remove entry at's slot, shifting later members of its probe run back
 * (linear-probing deletion: every other entry stays reachable).
 */
static void gpu_m4_drm_uat_span_unslot(u32 at)
{
	const struct gpu_m4_drm_uat_span_entry *e = &gpu_m4_uat_span[at];
	u32 hole = gpu_m4_drm_uat_span_hash(e->block, &e->key), next;

	while (gpu_m4_uat_span_slot[hole] != at + 1)
		hole = (hole + 1) & (GPU_M4_UAT_SPAN_SLOTS - 1);
	gpu_m4_uat_span_slot[hole] = 0;
	for (next = hole;;) {
		u32 home;
		u16 x;

		next = (next + 1) & (GPU_M4_UAT_SPAN_SLOTS - 1);
		x = gpu_m4_uat_span_slot[next];
		if (!x)
			return;
		home = gpu_m4_drm_uat_span_hash(gpu_m4_uat_span[x - 1].block,
						 &gpu_m4_uat_span[x - 1].key);
		/* Stays when its home lies cyclically in (hole, next]. */
		if (hole <= next ? (hole < home && home <= next) : (hole < home || home <= next))
			continue;
		gpu_m4_uat_span_slot[hole] = x;
		gpu_m4_uat_span_slot[next] = 0;
		hole = next;
	}
}

/* Drop entry at from the index and its block, onto the free list. */
static void gpu_m4_drm_uat_span_remove(u32 at)
{
	struct gpu_m4_drm_uat_span_entry *e = &gpu_m4_uat_span[at];
	struct gpu_m4_drm_uat_layer *l = e->block;

	gpu_m4_drm_uat_span_unslot(at);
	if (e->prev)
		gpu_m4_uat_span[e->prev - 1].next = e->next;
	else
		l->span_head = e->next;
	if (e->next)
		gpu_m4_uat_span[e->next - 1].prev = e->prev;
	l->span_count--;
	e->block = NULL;
	e->referenced = false;
	e->prev = 0;
	e->next = gpu_m4_uat_span_free;
	gpu_m4_uat_span_free = at + 1;
}

/* Add block l's key, known to be absent: a free entry, else CLOCK. Carried
 * (inherited) keys touch no REMOVE-DEBUG miss state.
 */
static void gpu_m4_drm_uat_span_insert(struct gpu_m4_drm_uat_layer *l,
	const struct gpu_m4_drm_uat_span_key *key, bool inherited)
{
	struct gpu_m4_drm_uat_span_entry *e;
	u32 at, s;

	if (!gpu_m4_uat_span_free && gpu_m4_uat_span_used < GPU_M4_UAT_SPAN_CACHE) {
		gpu_m4_uat_span_free = ++gpu_m4_uat_span_used;
		gpu_m4_uat_span[gpu_m4_uat_span_free - 1].next = 0;
	} else if (!gpu_m4_uat_span_free) {
		/* Full: every entry is live. At most one sweep. */
		while (gpu_m4_uat_span[gpu_m4_uat_span_hand].referenced) {
			gpu_m4_uat_span[gpu_m4_uat_span_hand].referenced = false;
			gpu_m4_uat_span_hand = (gpu_m4_uat_span_hand + 1) % GPU_M4_UAT_SPAN_CACHE;
		}
		at = gpu_m4_uat_span_hand;
		gpu_m4_uat_span_hand = (at + 1) % GPU_M4_UAT_SPAN_CACHE;
		if (!inherited) {
			/* REMOVE-DEBUG: churn. */
			gpu_m4_drm_uat_span_shadow(gpu_m4_uat_span[at].block, &gpu_m4_uat_span[at].key,
						   true);
			gpu_m4_uat_stat[GPU_M4_DRM_UAT_STAT_SPAN_EVICTIONS]++;
		}
		gpu_m4_drm_uat_span_remove(at);
	}
	at = gpu_m4_uat_span_free - 1;
	e = &gpu_m4_uat_span[at];
	gpu_m4_uat_span_free = e->next;
	*e = (struct gpu_m4_drm_uat_span_entry) {
		.key = *key, .block = l, .next = l->span_head, .inherited = inherited,
	};
	if (l->span_head)
		gpu_m4_uat_span[l->span_head - 1].prev = at + 1;
	l->span_head = at + 1;
	l->span_count++;
	for (s = gpu_m4_drm_uat_span_hash(l, key); gpu_m4_uat_span_slot[s];
	     s = (s + 1) & (GPU_M4_UAT_SPAN_SLOTS - 1))
		;
	gpu_m4_uat_span_slot[s] = at + 1;
	if (!inherited) {
		gpu_m4_uat_span_inserted++;
		gpu_m4_drm_uat_span_shadow(l, key, false);
		gpu_m4_uat_stat[GPU_M4_DRM_UAT_STAT_SPAN_INSERTS]++;
	}
}

/* Block l is being freed (no graph names it): forget all its entries. */
static void gpu_m4_drm_uat_span_purge(struct gpu_m4_drm_uat_layer *l)
{
	mutex_lock(&gpu_m4_uat_span_lock);
	while (l->span_head)
		gpu_m4_drm_uat_span_remove(l->span_head - 1);
	mutex_unlock(&gpu_m4_uat_span_lock);
}

/* Does block l itself hold a leaf in [page, end)? One VA bound, O(log n). */
static bool gpu_m4_drm_uat_layer_holds(const struct gpu_m4_drm_uat_layer *l, u64 page, u64 end)
{
	struct gpu_m4_drm_vm_leaf probe = { .va = page };
	struct gpu_m4_drm_uat_cursor c = gpu_m4_drm_uat_seq_bound(&l->seq[GPU_M4_DRM_VM_KEY_VA],
		&probe, GPU_M4_DRM_VM_KEY_VA);
	const struct gpu_m4_drm_vm_leaf *p = gpu_m4_drm_uat_cursor_leaf(&c);

	return p && p->va < end;
}

/* The pages [page, end) of a key's span; false for no valid span. */
static bool gpu_m4_drm_uat_span_pages(const struct gpu_m4_drm_uat_span_key *k,
	u64 *page, u64 *end)
{
	if (!k->size || k->size > ~0ULL - k->va)
		return false;
	*page = k->va & ~(GPU_M4_DRM_PAGE_SIZE - 1);
	*end = k->va + k->size;
	return true;
}

/*
 * The block keeping a proven span's key: the topmost block of the chain
 * holding a leaf of one of its pages. Blocks of a chain are VA-disjoint and
 * the proof found a leaf for every page, so each is a leaf of that block or
 * of one below it: that immutable sub-chain alone decides the fact, for any
 * chain naming the block (every block above it in any chain was proven
 * VA-disjoint from it when added). A child appended for one job thus keeps
 * proofs over its base's leaves on the base's block, for its siblings.
 * O(depth log n). No such leaf (never for a proven span): the top block.
 */
static struct gpu_m4_drm_uat_layer *gpu_m4_drm_uat_span_home(
	struct gpu_m4_drm_uat_layer *top, const struct gpu_m4_drm_uat_span_key *key)
{
	struct gpu_m4_drm_uat_layer *l;
	u64 page, end;

	if (!gpu_m4_drm_uat_span_pages(key, &page, &end))
		return top;
	for (l = top; l; l = l->below)
		if (gpu_m4_drm_uat_layer_holds(l, page, end))
			return l;
	return top;
}

struct gpu_m4_drm_uat_delta;
static bool gpu_m4_drm_uat_delta_touches(const struct gpu_m4_drm_uat_delta *d,
					 u64 page, u64 end);

/*
 * Construction: give the new block l every memo key of the base chain from
 * (held by the caller) whose pages the exact leaf delta leaves alone: no
 * changed, added or removed leaf VA of d (sorted changes) or, for a
 * flattened chain, no leaf of its fresh block in them. Each page of such a
 * key's span was a leaf of the base's manifest (its block's sub-chain) when
 * it was proven, and l's manifest maps that page to the identical leaf (VA,
 * PA, identity, offset, access, policy); its table words are proven against
 * l by l's own graphs. A fresh proof against any graph naming l therefore
 * walks the same leaves, makes the same comparisons against the key's
 * immutable facts and succeeds the same way: a carried key is exactly as
 * valid as one proven anew, and dies with l. Keys of facts the caller ties
 * to an old generation never match a new one. One pass over the memo plus
 * O(log delta) per key, against reproving each span's pages; l is not yet
 * reachable by any lookup. Best effort: replacement may forget some.
 */
static void gpu_m4_drm_uat_span_carry(struct gpu_m4_drm_uat_layer *l,
	const struct gpu_m4_drm_uat_layer *from, const struct gpu_m4_drm_uat_delta *d,
	const struct gpu_m4_drm_uat_layer *fresh)
{
	const struct gpu_m4_drm_uat_layer *b;
	u32 carried = 0, dropped = 0, keys = 0;

	mutex_lock(&gpu_m4_uat_span_lock);
	/* span_count is memo-locked: other VMs' eviction changes it. */
	for (b = from; b; b = b->below)
		keys += b->span_count;
	if (!keys) {
		mutex_unlock(&gpu_m4_uat_span_lock);
		return;
	}
	/* By index: entries inserted for l (or replacing a source one) are l's. */
	for (u32 at = 0; at < gpu_m4_uat_span_used; at++) {
		const struct gpu_m4_drm_uat_span_entry *e = &gpu_m4_uat_span[at];
		struct gpu_m4_drm_uat_span_key key;
		u64 page, end;

		for (b = from; b && b != e->block; b = b->below)
			;
		if (!b || !e->block)
			continue;
		key = e->key;
		if (!gpu_m4_drm_uat_span_pages(&key, &page, &end) ||
		    (d && gpu_m4_drm_uat_delta_touches(d, page, end)) ||
		    (fresh && gpu_m4_drm_uat_layer_holds(fresh, page, end))) {
			dropped++;
			continue;
		}
		if (gpu_m4_drm_uat_span_find(l, &key) >= 0)
			continue;
		gpu_m4_drm_uat_span_insert(l, &key, true);
		carried++;
	}
	mutex_unlock(&gpu_m4_uat_span_lock);
	/* REMOVE-DEBUG: memo inheritance. */
	gpu_m4_drm_uat_build_stat(GPU_M4_DRM_UAT_STAT_SPAN_INHERITED, carried);
	gpu_m4_drm_uat_build_stat(GPU_M4_DRM_UAT_STAT_SPAN_DROPPED, dropped);
}

/*
 * Index this graph's fresh leaves as a new manifest block and prove the
 * builder's complete alias contract for the union: same-owner RO/WO/RW aliases
 * are valid. Fresh/fresh pairs are adjacent in the fresh leaves' canonical
 * views; fresh/old pairs use the old chain's retained orders, which already
 * satisfy the contract among themselves. Identity, then PA, then VA
 * conflicts, as the full build reports them. No quadratic scan, no re-sort of
 * shared leaves. The block's runs are the mappings this graph adds.
 */
static int gpu_m4_drm_uat_layer_add(struct gpu_m4_drm_uat *v)
{
	struct gpu_m4_drm_uat_layer *below = v->manifest, *l = NULL;
	struct gpu_m4_drm_vm_leaf *leaf = v->leaf;
	const struct gpu_m4_drm_vm_leaf **pa, **id;
	u32 n = v->fresh, segments = gpu_m4_drm_uat_order_segments(n);
	u32 count[GPU_M4_UAT_ORDERS] = { segments, segments, segments };
	int ret;

	if (!n)
		return 0; /* An unchanged manifest is shared as is. */
	pa = kvcalloc(2 * (u64)n, sizeof(*pa), GFP_KERNEL);
	if (!pa)
		return -ENOMEM;
	id = pa + n;
	sort(leaf, n, sizeof(*leaf), gpu_m4_drm_uat_leaf_compare, NULL);
	gpu_m4_drm_uat_build_stat(GPU_M4_DRM_UAT_STAT_SORTED, n);
	gpu_m4_drm_uat_views(leaf, n, pa, id);
	ret = gpu_m4_drm_uat_ids_check(id, n);
	for (u32 i = 0; !ret && below && i < n; i++)
		ret = gpu_m4_drm_uat_layer_identity(below, &leaf[i]);
	if (!ret)
		ret = gpu_m4_drm_uat_pas_check(pa, n);
	for (u32 i = 0; !ret && below && i < n; i++)
		ret = gpu_m4_drm_uat_layer_alias(below, &leaf[i]);
	for (u32 i = 1; !ret && i < n; i++)
		if (leaf[i - 1].va == leaf[i].va)
			ret = -EADDRINUSE;
	for (u32 i = 0; !ret && below && i < n; i++)
		if (gpu_m4_drm_uat_layer_va(below, leaf[i].va))
			ret = -EADDRINUSE;
	if (!ret) {
		l = gpu_m4_drm_uat_layer_alloc(v->private_count - v->from, count);
		ret = l ? gpu_m4_drm_uat_layer_fill(l, leaf, pa, id, n) : -ENOMEM;
	}
	kvfree(pa);
	if (ret) {
		if (l)
			gpu_m4_drm_uat_layer_free(l);
		return ret;
	}
	gpu_m4_drm_uat_layer_runs(l, v->private + v->from, v->private_count - v->from);
	refcount_set(&l->refs, 1);
	l->below = below; /* The graph's reference moves to the new block. */
	l->depth = below ? below->depth + 1 : 1;
	v->manifest = l;
	/* The block holds its own copies: the fresh array is no longer needed. */
	kvfree(v->leaf);
	v->leaf = NULL;
	return 0;
}

/* Bound lookup depth: replace a too-deep chain by one owned block. The union
 * was already proven; this only re-indexes the same leaves and runs.
 */
static int gpu_m4_drm_uat_flatten(struct gpu_m4_drm_uat *v)
{
	struct gpu_m4_drm_uat_layer *top = v->manifest, *l = NULL;
	const struct gpu_m4_drm_uat_layer *each;
	const struct gpu_m4_drm_vm_leaf **pa = NULL;
	struct gpu_m4_drm_vm_leaf *leaf;
	u32 count[GPU_M4_UAT_ORDERS], n = 0, runs = 0;
	int ret = -ENOMEM;

	if (!top || top->depth <= GPU_M4_UAT_MAX_DEPTH)
		return 0;
	for (each = top; each; each = each->below) {
		if (each->count > v->count - n || each->runs > v->private_count - runs)
			return -EPROTO;
		n += each->count;
		runs += each->runs;
	}
	if (n != v->count)
		return -EPROTO;
	count[0] = count[1] = count[2] = gpu_m4_drm_uat_order_segments(n);
	leaf = kvcalloc(n, sizeof(*leaf), GFP_KERNEL);
	if (leaf)
		pa = kvcalloc(2 * (u64)n, sizeof(*pa), GFP_KERNEL);
	if (pa)
		l = gpu_m4_drm_uat_layer_alloc(runs, count);
	if (!l)
		goto out;
	n = 0;
	for (each = top; each; each = each->below) {
		struct gpu_m4_drm_uat_cursor c = { .seq = &each->seq[GPU_M4_DRM_VM_KEY_VA] };
		const struct gpu_m4_drm_vm_leaf *p;

		for (; (p = gpu_m4_drm_uat_cursor_leaf(&c)); gpu_m4_drm_uat_cursor_next(&c)) {
			if (n == v->count) {
				ret = -EPROTO;
				goto out;
			}
			leaf[n++] = *p;
		}
		memcpy(l->run + l->runs, each->run, array_size(each->runs, sizeof(*l->run)));
		l->runs += each->runs;
	}
	if (n != v->count) {
		ret = -EPROTO;
		goto out;
	}
	sort(leaf, n, sizeof(*leaf), gpu_m4_drm_uat_leaf_compare, NULL);
	gpu_m4_drm_uat_build_stat(GPU_M4_DRM_UAT_STAT_SORTED, n);
	sort(l->run, l->runs, sizeof(*l->run), gpu_m4_drm_uat_run_compare, NULL);
	gpu_m4_drm_uat_views(leaf, n, pa, pa + n);
	ret = gpu_m4_drm_uat_layer_fill(l, leaf, pa, pa + n, n);
out:
	kvfree(leaf);
	kvfree(pa);
	if (ret) {
		if (l)
			gpu_m4_drm_uat_layer_free(l);
		return ret;
	}
	refcount_set(&l->refs, 1);
	l->depth = 1;
	v->manifest = l;
	/* The same leaves: keys kept on the old chain hold for l, which adds
	 * none of top's fresh pages to them (checked all the same).
	 */
	gpu_m4_drm_uat_span_carry(l, top->below, NULL, top);
	gpu_m4_drm_uat_layer_put(top);
	return 0;
}

/* Inventory slot of the table (level, base) through the root/L1 slot index,
 * O(1) and independent of the number of tables or leaves.
 */
static int gpu_m4_drm_uat_find_table(const struct gpu_m4_drm_uat *v, u8 level, u64 base)
{
	const struct gpu_m4_drm_uat_table *t;
	u16 at;

	if (!v->tables || !v->table[0].node || !v->table[0].node->child || level > 2 ||
	    base >= (1ULL << 42))
		return -ENOENT;
	if (!level)
		return base ? -ENOENT : 0;
	at = v->table[0].node->child[gpu_m4_uat_fmt_slot(base, 0)];
	if (at && level == 2) {
		if (at > v->tables)
			return -ENOENT;
		t = &v->table[at - 1];
		if (t->level != 1 || !t->node || !t->node->child)
			return -ENOENT;
		at = t->node->child[gpu_m4_uat_fmt_slot(base, 1)];
	}
	if (!at || at > v->tables)
		return -ENOENT;
	t = &v->table[at - 1];
	return t->level == level && t->base == base ? at - 1 : -ENOENT;
}

/*
 * Allocate one fresh table page into record t: brand new and zeroed, or (copy)
 * a copy-on-write image of the shared node t currently names, which then
 * stays referenced through t->replaces until linking finished. The new page
 * is RAM-admitted and excluded from every expected payload PA by index. A
 * copy overwrites every byte, so it is neither zeroed nor zero-checked first:
 * its construction proof checks every final word.
 */
static int gpu_m4_drm_uat_node_new(struct gpu_m4_drm_uat *v, struct gpu_m4_drm_uat_table *t,
				   u8 level, u64 base, const struct gpu_m4_drm_uat_node *copy)
{
	u32 entries = level ? GPU_M4_UAT_ENTRIES : GPU_M4_UAT_ROOT_ENTRIES;
	struct gpu_m4_drm_uat_node *n = NULL, *old = t->node;
	struct page *allocation;
	void *cpu;
	u64 pa;
	int ret;

	if (copy && (copy->level != level || copy->base != base || (level < 2 && !copy->child)))
		return -EPROTO;
	allocation = copy ? alloc_pages(GFP_KERNEL, get_order(GPU_M4_DRM_PAGE_SIZE)) :
		alloc_pages(GFP_KERNEL | __GFP_ZERO, get_order(GPU_M4_DRM_PAGE_SIZE));
	if (!allocation)
		return -ENOMEM;
	pa = page_to_phys(allocation);
	cpu = page_address(allocation);
	ret = gpu_m4_drm_uat_ram(pa);
	if (!ret && (!cpu || ((unsigned long)cpu & (GPU_M4_DRM_PAGE_SIZE - 1)) ||
		     (!copy && memchr_inv(cpu, 0, GPU_M4_DRM_PAGE_SIZE))))
		ret = -EPROTO;
	if (!ret && gpu_m4_drm_uat_layer_pa(v->manifest, pa))
		ret = -EPERM;
	if (!ret) {
		n = kzalloc(sizeof(*n), GFP_KERNEL);
		if (n && level < 2)
			n->child = kvcalloc(entries, sizeof(*n->child), GFP_KERNEL);
		if (!n || (level < 2 && !n->child))
			ret = -ENOMEM;
	}
	if (ret) {
		if (n)
			kvfree(n->child);
		kfree(n);
		__free_pages(allocation, get_order(GPU_M4_DRM_PAGE_SIZE));
		return ret;
	}
	if (copy) {
		memcpy(cpu, copy->cpu, GPU_M4_DRM_PAGE_SIZE);
		if (level < 2)
			memcpy(n->child, copy->child, entries * sizeof(*n->child));
		n->used = copy->used;
	}
	refcount_set(&n->refs, 1);
	n->allocation = allocation;
	n->cpu = cpu;
	n->pa = pa;
	n->base = base;
	n->level = level;
	*t = (struct gpu_m4_drm_uat_table) {
		.allocation = allocation, .cpu = cpu, .pa = pa, .base = base, .node = n,
		.replaces = copy ? old : NULL, .level = level, .fresh = true,
	};
	v->owned++;
	return 0;
}

/* Child slot of a fresh parent, copying a shared child or creating one. */
static int gpu_m4_drm_uat_child(struct gpu_m4_drm_uat *v, u32 parent, u8 level, u64 base)
{
	struct gpu_m4_drm_uat_table *t, *p = &v->table[parent];
	u32 index = gpu_m4_uat_fmt_slot(base, level - 1);
	u16 *child;
	int ret;

	if (!p->fresh || !p->node || !p->node->child || p->level + 1 != level ||
	    (level == 1 && index >= GPU_M4_UAT_ROOT_ENTRIES))
		return -EPROTO;
	child = &p->node->child[index];
	if (*child) {
		if (*child > v->tables)
			return -EPROTO;
		t = &v->table[*child - 1];
		if (t->level != level || t->base != base || !t->node)
			return -EPROTO;
		if (!t->fresh) {
			ret = gpu_m4_drm_uat_node_new(v, t, level, base, t->node);
			if (ret)
				return ret;
		}
		return *child - 1;
	}
	if (v->tables >= v->table_limit || v->tables >= v->table_capacity)
		return -ENOSPC;
	ret = gpu_m4_drm_uat_node_new(v, &v->table[v->tables], level, base, NULL);
	if (ret)
		return ret;
	*child = ++v->tables;
	return v->tables - 1;
}

/*
 * Distinct table pages. Shared nodes are pairwise distinct already: they are
 * a subset of one constructed base whose own check (inductively, back to a
 * complete build checking all of them) covered them, and a node's PA never
 * changes. So only the F fresh pages are sorted, checked among themselves
 * and against each shared page by search: O((T + F) log F), never T x T or a
 * re-sort of the unchanged inventory.
 */
static int gpu_m4_drm_uat_table_owners(struct gpu_m4_drm_uat *v)
{
	u32 fresh = 0;
	u64 *pa;
	int ret = 0;

	for (u32 i = 0; i < v->tables; i++)
		fresh += v->table[i].fresh;
	if (!fresh || v->tables < 2)
		return 0;
	pa = kvcalloc(fresh, sizeof(*pa), GFP_KERNEL);
	if (!pa)
		return -ENOMEM;
	fresh = 0;
	for (u32 i = 0; i < v->tables; i++)
		if (v->table[i].fresh)
			pa[fresh++] = v->table[i].pa;
	sort(pa, fresh, sizeof(*pa), gpu_m4_drm_uat_pa_compare, NULL);
	for (u32 i = 1; !ret && i < fresh; i++)
		if (pa[i - 1] == pa[i])
			ret = -EPERM;
	for (u32 i = 0; !ret && i < v->tables; i++) {
		u32 first = 0, end = fresh;

		if (v->table[i].fresh)
			continue;
		while (first < end) {
			u32 middle = first + (end - first) / 2;

			if (pa[middle] < v->table[i].pa)
				first = middle + 1;
			else
				end = middle;
		}
		if (first < fresh && pa[first] == v->table[i].pa)
			ret = -EPERM;
	}
	kvfree(pa);
	return ret;
}

/* Fill fresh leaf tables along the copied/new paths of the fresh leaves only.
 * Root, every touched L1 and every touched L2 become fresh pages; every other
 * branch stays shared.
 */
static int gpu_m4_drm_uat_tables(struct gpu_m4_drm_uat *v)
{
	struct gpu_m4_drm_uat_cursor c = { 0 };
	int ret;

	if (v->tables) {
		ret = gpu_m4_drm_uat_node_new(v, &v->table[0], 0, 0, v->table[0].node);
	} else {
		if (!v->table_limit || !v->table_capacity)
			return -ENOSPC;
		ret = gpu_m4_drm_uat_node_new(v, &v->table[0], 0, 0, NULL);
		if (!ret)
			v->tables = 1;
	}
	if (ret)
		return ret;
	/* The fresh leaves are exactly the new top block, in VA order. */
	if (v->fresh)
		c.seq = &v->manifest->seq[GPU_M4_DRM_VM_KEY_VA];
	for (u32 i = 0; i < v->fresh; i++, gpu_m4_drm_uat_cursor_next(&c)) {
		const struct gpu_m4_drm_vm_leaf *p = gpu_m4_drm_uat_cursor_leaf(&c);
		void *cell;
		int l1, l2;

		if (!p)
			return -EPROTO;
		l1 = gpu_m4_drm_uat_child(v, 0, 1, gpu_m4_uat_fmt_base(p->va, 1));
		if (l1 < 0)
			return l1;
		l2 = gpu_m4_drm_uat_child(v, l1, 2, gpu_m4_uat_fmt_base(p->va, 2));
		if (l2 < 0)
			return l2;
		cell = v->table[l2].cpu + gpu_m4_uat_fmt_slot(p->va, 2) * 8;
		if (get_unaligned_le64(cell))
			return -EEXIST;
		put_unaligned_le64(gpu_m4_uat_fmt_leaf_pte(p->pa, p->access), cell);
	}
	return gpu_m4_drm_uat_table_owners(v);
}

/* Fresh leaves are complete before any parent names them. Only then clean/link/
 * clean one level at a time; only fresh pages are cleaned or written. A copied
 * parent's word may only change from the replaced node to its fresh copy. The
 * whole graph is still detached from every TTBAT; no shared page is touched.
 */
static int gpu_m4_drm_uat_finish_tables(struct gpu_m4_drm_uat *v)
{
	for (int level = 2; level >= 0; level--) {
		for (u32 i = 0; i < v->tables; i++) {
			struct gpu_m4_drm_uat_table *t = &v->table[i], *parent;
			u32 index;
			u64 expected;
			void *cell;
			u16 at;

			if (!t->fresh || t->level != level)
				continue;
			dcache_clean_poc((unsigned long)t->cpu,
					 (unsigned long)t->cpu + GPU_M4_DRM_PAGE_SIZE);
			if (!level)
				continue;
			at = level == 1 ? 1 : v->table[0].node->child[gpu_m4_uat_fmt_slot(t->base, 0)];
			if (!at || at > v->tables)
				return -EPROTO;
			parent = &v->table[at - 1];
			index = gpu_m4_uat_fmt_slot(t->base, level - 1);
			if (!parent->fresh || parent->level != level - 1 ||
			    parent->node->child[index] != i + 1)
				return -EPROTO;
			cell = parent->cpu + index * 8;
			expected = t->replaces ? gpu_m4_uat_fmt_table_pte(t->replaces->pa) : 0;
			if (get_unaligned_le64(cell) != expected)
				return -EEXIST;
			put_unaligned_le64(gpu_m4_uat_fmt_table_pte(t->pa), cell);
		}
	}
	return 0;
}

/* Record/node identity and exact parent naming through the slot index. */
static int gpu_m4_drm_uat_record(const struct gpu_m4_drm_uat *v, u32 i)
{
	const struct gpu_m4_drm_uat_table *t = &v->table[i], *parent;
	const struct gpu_m4_drm_uat_node *n = t->node;
	u64 span;
	u16 at;

	if (!n || !t->allocation || !t->cpu || t->allocation != n->allocation ||
	    t->cpu != n->cpu || t->pa != n->pa || t->base != n->base || t->level != n->level ||
	    t->pa != page_to_phys(t->allocation) || t->level > 2 || !i != !t->level ||
	    (t->level < 2 && !n->child) || t->base >= (1ULL << 42))
		return -EPROTO;
	if (!i)
		return t->base ? -EPROTO : 0;
	span = t->level == 1 ? 1ULL << 36 : 1ULL << 25;
	if (t->base & (span - 1) || !v->table[0].node || !v->table[0].node->child)
		return -EPROTO;
	at = t->level == 1 ? 1 : v->table[0].node->child[gpu_m4_uat_fmt_slot(t->base, 0)];
	if (!at || at > v->tables)
		return -EPROTO;
	parent = &v->table[at - 1];
	if (parent->level + 1 != t->level || !parent->node || !parent->node->child ||
	    parent->node->child[gpu_m4_uat_fmt_slot(t->base, t->level - 1)] != i + 1)
		return -EPROTO;
	return 0;
}

/*
 * Every word of one leaf table against the expected manifest: each expected
 * leaf of its 32 MiB span (VA-sorted per block, blocks VA-disjoint) has its
 * exact descriptor, and no other word is nonzero. Equivalent to looking each
 * nonzero word up by VA, in one pass per block instead of a search per word.
 */
static int gpu_m4_drm_uat_leaf_check(const struct gpu_m4_drm_uat *v,
	const struct gpu_m4_drm_uat_table *t, u32 *found)
{
	const struct gpu_m4_drm_uat_layer *l;
	u64 expected = 0;
	u32 count = 0;

	for (u32 index = 0; index < GPU_M4_UAT_ENTRIES; index++)
		count += !!get_unaligned_le64(t->cpu + index * 8);
	for (l = v->manifest; l; l = l->below) {
		struct gpu_m4_drm_vm_leaf probe = { .va = t->base };
		struct gpu_m4_drm_uat_cursor c = gpu_m4_drm_uat_seq_bound(
			&l->seq[GPU_M4_DRM_VM_KEY_VA], &probe, GPU_M4_DRM_VM_KEY_VA);
		const struct gpu_m4_drm_vm_leaf *p;

		for (; (p = gpu_m4_drm_uat_cursor_leaf(&c)) && p->va < t->base + (1ULL << 25);
		     gpu_m4_drm_uat_cursor_next(&c)) {
			u64 raw = get_unaligned_le64(t->cpu + gpu_m4_uat_fmt_slot(p->va, 2) * 8);

			if (!raw || raw != gpu_m4_uat_fmt_leaf_pte(p->pa, p->access))
				return -EPROTO;
			expected++;
		}
	}
	if (expected != count)
		return -EPROTO;
	*found = count;
	return 0;
}

/* Every word of one table against the expected manifest/children. */
static int gpu_m4_drm_uat_node_check(const struct gpu_m4_drm_uat *v, u32 i, u32 *found)
{
	const struct gpu_m4_drm_uat_table *t = &v->table[i];
	u32 shift, count = 0;
	int ret = gpu_m4_drm_uat_record(v, i);

	if (ret)
		return ret;
	if (t->level == 2)
		return gpu_m4_drm_uat_leaf_check(v, t, found);
	shift = gpu_m4_uat_fmt_shift(t->level);
	for (u32 index = 0; index < GPU_M4_UAT_ENTRIES; index++) {
		u64 raw = get_unaligned_le64(t->cpu + index * 8);
		u64 va = t->base | ((u64)index << shift);
		u16 at = t->level || index < GPU_M4_UAT_ROOT_ENTRIES ? t->node->child[index] : 0;
		const struct gpu_m4_drm_uat_table *c;

		if (!raw) {
			if (at)
				return -EPROTO;
			continue;
		}
		if (!at || at > v->tables)
			return -EPROTO;
		c = &v->table[at - 1];
		if (c->level != t->level + 1 || c->base != va || raw != gpu_m4_uat_fmt_table_pte(c->pa))
			return -EPROTO;
		count++;
	}
	*found = count;
	return 0;
}

/* Complete owned walk (fresh_only false) or construction proof: fresh pages
 * word by word; shared pages by record identity plus their creator's proven
 * counts, valid because no page is ever written after its construction. A
 * shared node inherits only a proof its creator actually completed.
 */
static int gpu_m4_drm_uat_walk(struct gpu_m4_drm_uat *v, bool fresh_only)
{
	u64 leaves = 0, children = 0;

	if (!v || !v->snapshot || !v->table || !v->tables || v->tables > v->table_capacity ||
	    v->table[0].level || v->table[0].base)
		return -EINVAL;
	for (u32 i = 0; i < v->tables; i++) {
		struct gpu_m4_drm_uat_table *t = &v->table[i];
		u32 found = 0;
		int ret;

		if (fresh_only && !t->fresh) {
			ret = gpu_m4_drm_uat_record(v, i);
			if (!ret && !t->node->proven)
				ret = -EPROTO;
			if (!ret)
				found = t->node->used;
		} else {
			ret = gpu_m4_drm_uat_node_check(v, i, &found);
			if (!ret && fresh_only)
				t->node->used = found;
		}
		if (ret)
			return ret;
		if (t->level == 2)
			leaves += found;
		else
			children += found;
	}
	return leaves == v->count && children == v->tables - 1 ? 0 : -EPROTO;
}

static void gpu_m4_drm_uat_drop_replaced(struct gpu_m4_drm_uat *v)
{
	for (u32 i = 0; i < v->tables; i++) {
		gpu_m4_drm_uat_node_put(v->table[i].replaces);
		v->table[i].replaces = NULL;
	}
}

/*
 * Pure detached construction from a prepared graph: no provider lock, no
 * callback, no live table. Inputs are the held snapshot/private owners, the
 * already resolved fresh leaves, and the immutable held base chain/nodes.
 */
static int gpu_m4_drm_uat_rebase_construct(struct gpu_m4_drm_uat *v);

static int gpu_m4_drm_uat_construct_graph(struct gpu_m4_drm_uat *v)
{
	int ret;

	if (v->share) {
		/* Every named node and the manifest block are the image's own,
		 * already proven; only record identity and inherited counts.
		 */
		ret = gpu_m4_drm_uat_walk(v, true);
		if (ret)
			return ret;
		v->proof_manifest = v->manifest;
		v->cleaned = true;
		return 0;
	}
	ret = gpu_m4_drm_uat_layer_add(v);
	if (!ret)
		ret = gpu_m4_drm_uat_tables(v);
	if (!ret)
		ret = gpu_m4_drm_uat_finish_tables(v);
	if (!ret)
		ret = gpu_m4_drm_uat_flatten(v);
	if (!ret)
		ret = gpu_m4_drm_uat_walk(v, true);
	if (ret)
		return ret;
	gpu_m4_drm_uat_drop_replaced(v);
	/* The proof just made covers exactly these fresh pages (each cleaned,
	 * child before parent, and checked word by word); shared nodes keep
	 * their creators' proofs. Only now may any binding consume them.
	 */
	for (u32 i = 0; i < v->tables; i++)
		if (v->table[i].fresh)
			v->table[i].node->proven = true;
	v->proof_manifest = v->manifest;
	v->cleaned = true;
	v->checked_low = v->table[0].pa | 1;
	return 0;
}

int gpu_m4_drm_uat_construct(struct gpu_m4_drm_uat *v)
{
	u32 kind;
	u64 start;
	int ret;

	if (!v || !v->prepared || v->cleaned || !v->snapshot)
		return -EINVAL;
	v->prepared = false;
	kind = v->rebase ? GPU_M4_DRM_UAT_STAT_REBASES : v->share ? GPU_M4_DRM_UAT_STAT_SHARES :
		v->tables ? GPU_M4_DRM_UAT_STAT_APPENDS : GPU_M4_DRM_UAT_STAT_FULLS;
	start = ktime_get_ns();
	ret = v->rebase ? gpu_m4_drm_uat_rebase_construct(v) : gpu_m4_drm_uat_construct_graph(v);
	if (ret)
		return ret;
	/* REMOVE-DEBUG: what this construction built versus shared. */
	gpu_m4_drm_uat_build_stat(GPU_M4_DRM_UAT_STAT_CONSTRUCTS, 1);
	gpu_m4_drm_uat_build_stat(kind, 1);
	gpu_m4_drm_uat_build_stat(GPU_M4_DRM_UAT_STAT_TABLES_BUILT, v->owned);
	gpu_m4_drm_uat_build_stat(GPU_M4_DRM_UAT_STAT_TABLES_SHARED, v->tables - v->owned);
	if (kind == GPU_M4_DRM_UAT_STAT_REBASES)
		gpu_m4_drm_uat_build_stat(GPU_M4_DRM_UAT_STAT_REBASE_NS, ktime_get_ns() - start);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_construct);

static int gpu_m4_drm_uat_checked(const struct gpu_m4_drm_uat *v);

static struct gpu_m4_drm_uat *gpu_m4_drm_uat_prepare_owned(
	struct gpu_m4_drm_uat *base, struct gpu_m4_drm_vm_owner *snapshot,
	const struct gpu_m4_drm_uat_private *private, u32 private_count,
	u32 max_table_pages, u32 max_leaf_pages, struct gpu_m4_drm_uat **garbage)
{
	struct gpu_m4_drm_uat *v;
	u32 user_count = gpu_m4_drm_vm_owner_leaf_count(snapshot), from = 0;
	u64 total = user_count, fresh, capacity;
	u32 fixed = 0;
	int ret;

	if (garbage)
		*garbage = NULL;
	if (!snapshot || !gpu_m4_drm_vm_owner_vm(snapshot) || !private || !private_count ||
	    !max_table_pages || max_table_pages > GPU_M4_DRM_UAT_MAX_TABLES ||
	    !max_leaf_pages || max_leaf_pages > GPU_M4_DRM_UAT_MAX_LEAVES ||
	    private_count > max_leaf_pages || PAGE_SIZE > GPU_M4_DRM_PAGE_SIZE)
		return ERR_PTR(-EINVAL);
	for (u32 i = 0; i < private_count; i++) {
		ret = gpu_m4_drm_uat_private_shape(&private[i]);
		if (ret)
			return ERR_PTR(ret);
		fixed += private[i].va == GPU_M4_DRM_UAT_CONTEXT_START;
		total += private[i].size / GPU_M4_DRM_PAGE_SIZE;
		if (total > max_leaf_pages)
			return ERR_PTR(-ENOSPC);
	}
	if (fixed != 1)
		return ERR_PTR(-EINVAL);
	if (base) {
		/* Derivation shares the base's proven user leaves and private
		 * prefix: same VM, user generation/count and exact private prefix.
		 * Its own checked identity stands in for another full walk.
		 */
		ret = gpu_m4_drm_uat_checked(base);
		if (ret)
			return ERR_PTR(ret);
		if (!base->manifest ||
		    gpu_m4_drm_vm_owner_vm(base->snapshot) != gpu_m4_drm_vm_owner_vm(snapshot) ||
		    gpu_m4_drm_vm_owner_generation(base->snapshot) !=
			gpu_m4_drm_vm_owner_generation(snapshot) ||
		    base->user_count != user_count || base->private_count > private_count ||
		    base->count > total)
			return ERR_PTR(-ESTALE);
		for (u32 i = 0; i < base->private_count; i++)
			if (!gpu_m4_drm_uat_private_same(&base->private[i], &private[i]))
				return ERR_PTR(-ESTALE);
		if (base->tables > max_table_pages)
			return ERR_PTR(-ENOSPC);
		from = base->private_count;
		fresh = total - base->count;
		capacity = base->tables + 2 * fresh;
	} else {
		/* Reservations apply to the WHOLE VM, not merely occupied private leaves. */
		for (u32 i = 0; i < user_count; i++) {
			struct gpu_m4_drm_vm_leaf p;

			ret = gpu_m4_drm_vm_owner_leaf(snapshot, i, &p);
			if (ret || gpu_m4_drm_uat_reserved(p.va))
				return ERR_PTR(ret ? ret : -EADDRINUSE);
		}
		fresh = total;
		capacity = 1 + 2 * fresh;
	}
	if (capacity > max_table_pages)
		capacity = max_table_pages;
	v = kzalloc(sizeof(*v), GFP_KERNEL);
	if (!v)
		return ERR_PTR(-ENOMEM);
	refcount_set(&v->refs, 1);
	v->private = kvcalloc(private_count, sizeof(*private), GFP_KERNEL);
	v->leaf = kvcalloc(fresh ? fresh : 1, sizeof(*v->leaf), GFP_KERNEL);
	v->table = kvcalloc(capacity, sizeof(*v->table), GFP_KERNEL);
	if (!v->private || !v->leaf || !v->table) {
		ret = -ENOMEM;
		goto fail;
	}
	memcpy(v->private, private, array_size(private_count, sizeof(*private)));
	v->private_count = private_count;
	v->from = from;
	v->table_limit = max_table_pages;
	v->table_capacity = capacity;
	v->user_count = user_count;
	ret = gpu_m4_drm_vm_owner_get_unpublished(snapshot);
	if (ret)
		goto fail;
	v->snapshot = snapshot;
	if (base) {
		/* Strong refs: every unchanged branch and manifest block is shared. */
		for (u32 i = 0; i < base->tables; i++) {
			refcount_inc(&base->table[i].node->refs);
			v->table[i] = base->table[i];
			v->table[i].fresh = false;
			v->table[i].replaces = NULL;
		}
		v->tables = base->tables;
		refcount_inc(&base->manifest->refs);
		v->manifest = base->manifest;
		v->count = base->count;
	} else {
		for (u32 i = 0; i < user_count; i++) {
			struct gpu_m4_drm_vm_leaf p;

			ret = gpu_m4_drm_vm_owner_leaf(snapshot, i, &p);
			if (!ret)
				ret = gpu_m4_drm_uat_leaf_add(v, &p);
			if (ret)
				goto fail;
		}
	}
	for (u32 i = 0; i < private_count; i++) {
		const struct gpu_m4_drm_uat_private *p = &v->private[i];

		ret = p->get(p->cookie, gpu_m4_drm_vm_owner_vm(snapshot), p->identity);
		if (ret)
			goto fail;
		v->held++;
		if (i < from)
			continue; /* Shared: the base resolved and proved these pages. */
		for (u64 offset = 0; offset < p->size; offset += GPU_M4_DRM_PAGE_SIZE) {
			struct gpu_m4_drm_vm_leaf leaf = {
				.va = p->va + offset, .identity = p->identity,
				.object_offset = p->offset + offset,
				.access = p->access, .cpu_policy = p->cpu_policy,
			};

			ret = p->page(p->cookie, leaf.object_offset, &leaf.pa);
			if (!ret)
				ret = gpu_m4_drm_uat_leaf_add(v, &leaf);
			if (ret)
				goto fail;
		}
	}
	v->prepared = true;
	return v;
fail:
	if (garbage)
		*garbage = v;
	else
		gpu_m4_drm_uat_release(v);
	return ERR_PTR(ret);
}

struct gpu_m4_drm_uat *gpu_m4_drm_uat_prepare_deferred(struct gpu_m4_drm_uat *base,
	struct gpu_m4_drm_vm_owner *snapshot,
	const struct gpu_m4_drm_uat_private *private, u32 private_count,
	u32 max_table_pages, u32 max_leaf_pages, struct gpu_m4_drm_uat **garbage)
{
	if (!garbage)
		return ERR_PTR(-EINVAL);
	return gpu_m4_drm_uat_prepare_owned(base, snapshot, private, private_count,
		max_table_pages, max_leaf_pages, garbage);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_prepare_deferred);

static struct gpu_m4_drm_uat *gpu_m4_drm_uat_build_owned(
	struct gpu_m4_drm_uat *base, struct gpu_m4_drm_vm_owner *snapshot,
	const struct gpu_m4_drm_uat_private *private, u32 private_count,
	u32 max_table_pages, u32 max_leaf_pages, struct gpu_m4_drm_uat **garbage)
{
	struct gpu_m4_drm_uat *v = gpu_m4_drm_uat_prepare_owned(base, snapshot, private,
		private_count, max_table_pages, max_leaf_pages, garbage);
	int ret;

	if (IS_ERR(v))
		return v;
	ret = gpu_m4_drm_uat_construct(v);
	if (!ret)
		return v;
	if (garbage)
		*garbage = v;
	else
		gpu_m4_drm_uat_release(v);
	return ERR_PTR(ret);
}

struct gpu_m4_drm_uat *gpu_m4_drm_uat_build(
	struct gpu_m4_drm_vm_owner *snapshot,
	const struct gpu_m4_drm_uat_private *private, u32 private_count,
	u32 max_table_pages, u32 max_leaf_pages)
{
	return gpu_m4_drm_uat_build_owned(NULL, snapshot, private, private_count,
		max_table_pages, max_leaf_pages, NULL);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_build);

struct gpu_m4_drm_uat *gpu_m4_drm_uat_build_deferred(
	struct gpu_m4_drm_vm_owner *snapshot,
	const struct gpu_m4_drm_uat_private *private, u32 private_count,
	u32 max_table_pages, u32 max_leaf_pages, struct gpu_m4_drm_uat **garbage)
{
	if (!garbage)
		return ERR_PTR(-EINVAL);
	return gpu_m4_drm_uat_build_owned(NULL, snapshot, private, private_count,
		max_table_pages, max_leaf_pages, garbage);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_build_deferred);

struct gpu_m4_drm_uat *gpu_m4_drm_uat_derive_deferred(struct gpu_m4_drm_uat *base,
	struct gpu_m4_drm_vm_owner *snapshot,
	const struct gpu_m4_drm_uat_private *private, u32 private_count,
	u32 max_table_pages, u32 max_leaf_pages, struct gpu_m4_drm_uat **garbage)
{
	if (!base || !garbage)
		return ERR_PTR(-EINVAL);
	return gpu_m4_drm_uat_build_owned(base, snapshot, private, private_count,
		max_table_pages, max_leaf_pages, garbage);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_derive_deferred);

/* ---- Cross-generation rebase: exact old->target delta over a held base ---- */

/* One exact page-word change of the target image against the base image. */
struct gpu_m4_drm_uat_change {
	const struct gpu_m4_drm_vm_leaf *old, *new;
};

/* First VA-ordered user position at or above va. */
static u32 gpu_m4_drm_uat_user_bound(const struct gpu_m4_drm_uat_user *u, u64 va)
{
	u32 first = 0, end = u->count;

	while (first < end) {
		u32 middle = first + (end - first) / 2;

		if (u->leaf[u->va[middle]].va < va)
			first = middle + 1;
		else
			end = middle;
	}
	return first;
}

/* The borrowed target user view must be a proven permutation index: VA order
 * strictly increasing outside every reserved interval. Never a sort of the
 * target ledger. In-range positions naming strictly increasing VAs are
 * distinct, hence a permutation, and VA-sorted they avoid each reserved
 * interval iff its one search finds nothing: one sequential pass. Any other
 * view takes the complete checks for its exact error.
 */
static int gpu_m4_drm_uat_user_view(const struct gpu_m4_drm_uat_user *u)
{
	u32 n = u->count, at;
	u8 *seen;
	int ret = 0;

	if (!n)
		return 0;
	for (u32 i = 0; i < n; i++)
		if (u->va[i] >= n ||
		    (i && u->leaf[u->va[i - 1]].va >= u->leaf[u->va[i]].va))
			goto complete;
	at = gpu_m4_drm_uat_user_bound(u, GPU_M4_DRM_UAT_CONTEXT_START);
	if (at < n && gpu_m4_drm_uat_reserved(u->leaf[u->va[at]].va))
		goto complete;
	at = gpu_m4_drm_uat_user_bound(u, GPU_M4_DRM_UAT_PRIVATE_START);
	if (at < n && gpu_m4_drm_uat_reserved(u->leaf[u->va[at]].va))
		goto complete;
	return 0;
complete:
	seen = kvcalloc(n, sizeof(*seen), GFP_KERNEL);
	if (!seen)
		return -ENOMEM;
	for (u32 i = 0; !ret && i < n; i++) {
		if (u->va[i] >= n || seen[u->va[i]])
			ret = -EPROTO;
		else
			seen[u->va[i]] = 1;
	}
	kvfree(seen);
	for (u32 i = 0; !ret && i < n; i++)
		if (gpu_m4_drm_uat_reserved(u->leaf[u->va[i]].va))
			ret = -EADDRINUSE;
	for (u32 i = 1; !ret && i < n; i++) {
		const struct gpu_m4_drm_vm_leaf *a = &u->leaf[u->va[i - 1]], *b = &u->leaf[u->va[i]];

		if (a->va >= b->va)
			ret = a->va == b->va ? -EADDRINUSE : -EPROTO;
	}
	return ret;
}

/* Admission of a changed/added user leaf, exactly leaf_add()'s checks. */
static int gpu_m4_drm_uat_leaf_admit(const struct gpu_m4_drm_vm_leaf *p)
{
	int ret = gpu_m4_drm_uat_ram(p->pa);

	if (ret)
		return ret;
	if (!p->identity || !gpu_m4_drm_aligned(p->va) ||
	    !gpu_m4_drm_range(p->va, GPU_M4_DRM_PAGE_SIZE, GPU_M4_DRM_PAGE_SIZE, 1ULL << 42) ||
	    !gpu_m4_drm_vm_leaf_attrs(p->access))
		return -EINVAL;
	return 0;
}

/* The exact old->target changes, in VA order, grown on demand: a rebase
 * allocates in proportion to its delta, never to the manifest.
 */
struct gpu_m4_drm_uat_delta {
	struct gpu_m4_drm_uat_change *change;
	u32 count, capacity;
};

static int gpu_m4_drm_uat_delta_add(struct gpu_m4_drm_uat_delta *d,
	const struct gpu_m4_drm_vm_leaf *old, const struct gpu_m4_drm_vm_leaf *new)
{
	if (d->count == d->capacity) {
		u32 capacity = d->capacity ? 2 * d->capacity : 64;
		struct gpu_m4_drm_uat_change *grown;

		grown = kvcalloc(capacity, sizeof(*grown), GFP_KERNEL);
		if (!grown)
			return -ENOMEM;
		if (d->count)
			memcpy(grown, d->change, array_size(d->count, sizeof(*grown)));
		kvfree(d->change);
		d->change = grown;
		d->capacity = capacity;
	}
	d->change[d->count++] = (struct gpu_m4_drm_uat_change) { .old = old, .new = new };
	return 0;
}

static u64 gpu_m4_drm_uat_change_va(const struct gpu_m4_drm_uat_change *c)
{
	return c->old ? c->old->va : c->new->va;
}

/* One merge step of two VA-ordered leaf streams into changes: equal leaves
 * at a VA are no change.
 */
static int gpu_m4_drm_uat_delta_diff(struct gpu_m4_drm_uat_delta *d,
	const struct gpu_m4_drm_vm_leaf *const *old, u32 no,
	const struct gpu_m4_drm_vm_leaf *const *new, u32 nn)
{
	u32 i = 0, j = 0;
	int ret = 0;

	while (!ret && (i < no || j < nn)) {
		const struct gpu_m4_drm_vm_leaf *x = i < no ? old[i] : NULL;
		const struct gpu_m4_drm_vm_leaf *y = j < nn ? new[j] : NULL;

		if (x && y && x->va == y->va) {
			i++;
			j++;
			if (gpu_m4_drm_uat_leaf_same(x, y))
				continue;
		} else if (x && (!y || x->va < y->va)) {
			i++;
			y = NULL;
		} else {
			j++;
			x = NULL;
		}
		ret = gpu_m4_drm_uat_delta_add(d, x, y);
	}
	return ret;
}

/* One private mapping of the target in VA order; first: its resolved leaves
 * in the graph's fresh array (keep: none).
 */
struct gpu_m4_drm_uat_slot {
	u64 va, end;
	u32 index, first;
};

static int gpu_m4_drm_uat_slot_compare(const void *first, const void *second)
{
	const struct gpu_m4_drm_uat_slot *a = first, *b = second;

	return a->va < b->va ? -1 : a->va > b->va;
}

/*
 * Exact old->target private changes. Every private leaf of the base block lies
 * in exactly one of its runs, and a kept target mapping is exactly one of them
 * (prepare matched it), so its pages are unchanged and never visited. The
 * other base runs' leaves against the resolved target mappings' leaves, both
 * in VA order, are the changes: O(M log M + pages of the unmatched runs and
 * resolved mappings), whatever the size of the kept baseline. Overlapping
 * target mappings (a VA collision) return 1 with no change: the caller then
 * takes the complete refusal.
 */
static int gpu_m4_drm_uat_private_delta(const struct gpu_m4_drm_uat *v,
	const struct gpu_m4_drm_uat_layer *o, struct gpu_m4_drm_uat_delta *out, u64 *touched)
{
	const struct gpu_m4_drm_vm_leaf **old = NULL, **new = NULL;
	struct gpu_m4_drm_uat_slot *slot;
	u64 no = 0, nn = 0, first = 0;
	bool *matched;
	int ret = 0;

	slot = kvcalloc(v->private_count, sizeof(*slot), GFP_KERNEL);
	matched = kvcalloc(max(o->runs, 1U), sizeof(*matched), GFP_KERNEL);
	if (!slot || !matched) {
		ret = -ENOMEM;
		goto out;
	}
	for (u32 i = 0; i < v->private_count; i++) {
		const struct gpu_m4_drm_uat_private *p = &v->private[i];

		slot[i] = (struct gpu_m4_drm_uat_slot) {
			.va = p->va, .end = p->va + p->size, .index = i, .first = (u32)first,
		};
		if (!v->keep[i])
			first += p->size / GPU_M4_DRM_PAGE_SIZE;
	}
	if (first != v->fresh) {
		ret = -EPROTO;
		goto out;
	}
	sort(slot, v->private_count, sizeof(*slot), gpu_m4_drm_uat_slot_compare, NULL);
	for (u32 i = 1; i < v->private_count; i++)
		if (slot[i].va < slot[i - 1].end) {
			ret = 1;
			goto out;
		}
	for (u32 i = 0; i < v->private_count; i++) {
		int r;

		if (!v->keep[i])
			continue;
		r = gpu_m4_drm_uat_run_find(o, &v->private[i]);
		if (r < 0) {
			ret = -EPROTO;
			goto out;
		}
		matched[r] = true;
	}
	for (u32 r = 0; r < o->runs; r++)
		no += matched[r] ? 0 : o->run[r].size / GPU_M4_DRM_PAGE_SIZE;
	old = kvcalloc(no ? no : 1, sizeof(*old), GFP_KERNEL);
	new = kvcalloc(v->fresh ? v->fresh : 1, sizeof(*new), GFP_KERNEL);
	if (!old || !new) {
		ret = -ENOMEM;
		goto out;
	}
	no = 0;
	for (u32 r = 0; r < o->runs; r++) {
		const struct gpu_m4_drm_uat_run *run = &o->run[r];
		struct gpu_m4_drm_vm_leaf probe = { .va = run->va };
		struct gpu_m4_drm_uat_cursor c;

		if (matched[r])
			continue;
		c = gpu_m4_drm_uat_seq_bound(&o->seq[GPU_M4_DRM_VM_KEY_VA], &probe,
					     GPU_M4_DRM_VM_KEY_VA);
		for (u64 offset = 0; offset < run->size; offset += GPU_M4_DRM_PAGE_SIZE) {
			const struct gpu_m4_drm_vm_leaf *p = gpu_m4_drm_uat_cursor_leaf(&c);

			if (!p || p->va != run->va + offset) {
				ret = -EPROTO;
				goto out;
			}
			old[no++] = p;
			gpu_m4_drm_uat_cursor_next(&c);
		}
	}
	for (u32 i = 0; i < v->private_count; i++) {
		const struct gpu_m4_drm_uat_slot *t = &slot[i];

		if (v->keep[t->index])
			continue;
		for (u64 k = 0; k < (t->end - t->va) / GPU_M4_DRM_PAGE_SIZE; k++)
			new[nn++] = &v->leaf[t->first + k];
	}
	*touched += no + nn;
	ret = gpu_m4_drm_uat_delta_diff(out, old, no, new, nn);
out:
	kvfree(old);
	kvfree(new);
	kvfree(matched);
	kvfree(slot);
	return ret;
}

/*
 * Exact old->target user changes, merged in VA order with the private ones p
 * (VA-disjoint: private VAs lie in the reserved intervals, user VAs never do):
 * the base block's leaves outside the reserved intervals against the proven
 * target view, one merge, the base's reserved intervals skipped by search.
 * Linear in the user leaves: taken only when the user generation changed.
 */
static int gpu_m4_drm_uat_user_delta(const struct gpu_m4_drm_uat_layer *o,
	const struct gpu_m4_drm_uat_user *u, const struct gpu_m4_drm_uat_delta *p,
	struct gpu_m4_drm_uat_delta *out, u64 *touched)
{
	const struct gpu_m4_drm_uat_seq *s = &o->seq[GPU_M4_DRM_VM_KEY_VA];
	struct gpu_m4_drm_vm_leaf probe = { 0 };
	struct gpu_m4_drm_uat_cursor c = gpu_m4_drm_uat_seq_bound(s, &probe,
		GPU_M4_DRM_VM_KEY_VA);
	u32 j = 0, k = 0;
	u64 steps = 0;
	int ret = 0;

	while (!ret) {
		const struct gpu_m4_drm_vm_leaf *x = gpu_m4_drm_uat_cursor_leaf(&c);
		const struct gpu_m4_drm_vm_leaf *y = j < u->count ? &u->leaf[u->va[j]] : NULL;
		u64 va;

		if (x && gpu_m4_drm_uat_reserved(x->va)) {
			probe.va = x->va < GPU_M4_DRM_UAT_PRIVATE_START ?
				GPU_M4_DRM_UAT_CONTEXT_START + GPU_M4_DRM_UAT_CONTEXT_SIZE :
				GPU_M4_DRM_UAT_PRIVATE_END;
			c = gpu_m4_drm_uat_seq_bound(s, &probe, GPU_M4_DRM_VM_KEY_VA);
			continue;
		}
		if (!x && !y)
			break;
		steps++;
		if (x && y && x->va == y->va) {
			gpu_m4_drm_uat_cursor_next(&c);
			j++;
			if (gpu_m4_drm_uat_leaf_same(x, y))
				continue;
		} else if (x && (!y || x->va < y->va)) {
			gpu_m4_drm_uat_cursor_next(&c);
			y = NULL;
		} else {
			j++;
			x = NULL;
		}
		va = x ? x->va : y->va;
		for (; !ret && k < p->count && gpu_m4_drm_uat_change_va(&p->change[k]) < va; k++)
			ret = gpu_m4_drm_uat_delta_add(out, p->change[k].old, p->change[k].new);
		if (!ret)
			ret = gpu_m4_drm_uat_delta_add(out, x, y);
	}
	for (; !ret && k < p->count; k++)
		ret = gpu_m4_drm_uat_delta_add(out, p->change[k].old, p->change[k].new);
	*touched += steps;
	gpu_m4_drm_uat_build_stat(GPU_M4_DRM_UAT_STAT_USER_DIFFED, steps);
	return ret;
}

/*
 * Exact refusal of a target whose private mappings overlap, as the full build
 * reports it: the complete target leaves, sorted, then the identity, PA and
 * VA rules in that order. Reached only on the way to an error.
 */
static int gpu_m4_drm_uat_rebase_refusal(const struct gpu_m4_drm_uat *v,
	const struct gpu_m4_drm_uat_layer *o)
{
	const struct gpu_m4_drm_vm_leaf **pa = NULL;
	struct gpu_m4_drm_vm_leaf *leaf;
	u64 total = v->user_count, n = 0, first = 0;
	int ret = -ENOMEM;

	for (u32 i = 0; i < v->private_count; i++)
		total += v->private[i].size / GPU_M4_DRM_PAGE_SIZE;
	leaf = kvcalloc(total, sizeof(*leaf), GFP_KERNEL);
	if (leaf)
		pa = kvcalloc(2 * total, sizeof(*pa), GFP_KERNEL);
	if (!pa)
		goto out;
	ret = -EPROTO;
	if (v->user_same) {
		struct gpu_m4_drm_uat_cursor c = { .seq = &o->seq[GPU_M4_DRM_VM_KEY_VA] };
		const struct gpu_m4_drm_vm_leaf *p;

		for (; (p = gpu_m4_drm_uat_cursor_leaf(&c)); gpu_m4_drm_uat_cursor_next(&c))
			if (!gpu_m4_drm_uat_reserved(p->va)) {
				if (n == v->user_count)
					goto out;
				leaf[n++] = *p;
			}
		if (n != v->user_count)
			goto out;
	} else {
		for (; n < v->user_count; n++)
			leaf[n] = v->user.leaf[n];
	}
	for (u32 i = 0; i < v->private_count; i++) {
		const struct gpu_m4_drm_uat_private *m = &v->private[i];
		u64 pages = m->size / GPU_M4_DRM_PAGE_SIZE;

		if (!v->keep[i]) {
			memcpy(leaf + n, v->leaf + first, array_size(pages, sizeof(*leaf)));
			first += pages;
			n += pages;
			continue;
		}
		for (u64 offset = 0; offset < m->size; offset += GPU_M4_DRM_PAGE_SIZE) {
			const struct gpu_m4_drm_vm_leaf *p = gpu_m4_drm_uat_layer_va(o,
				m->va + offset);

			if (!p)
				goto out;
			leaf[n++] = *p;
		}
	}
	sort(leaf, n, sizeof(*leaf), gpu_m4_drm_uat_leaf_compare, NULL);
	gpu_m4_drm_uat_views(leaf, n, pa, pa + n);
	ret = gpu_m4_drm_uat_ids_check(pa + n, n);
	if (!ret)
		ret = gpu_m4_drm_uat_pas_check(pa, n);
	/* Overlapping mappings always repeat a VA. */
	if (!ret)
		ret = -EADDRINUSE;
out:
	kvfree(pa);
	kvfree(leaf);
	return ret;
}

/* Is va the VA of a new leaf of the VA-ordered changes? */
static bool gpu_m4_drm_uat_changed(const struct gpu_m4_drm_uat_delta *d, u64 va)
{
	u32 first = 0, end = d->count;

	while (first < end) {
		u32 middle = first + (end - first) / 2;

		if (gpu_m4_drm_uat_change_va(&d->change[middle]) < va)
			first = middle + 1;
		else
			end = middle;
	}
	return first < d->count && d->change[first].new && d->change[first].new->va == va;
}

/* Does any change of the VA-ordered delta lie in [page, end)? */
static bool gpu_m4_drm_uat_delta_touches(const struct gpu_m4_drm_uat_delta *d,
					 u64 page, u64 end)
{
	u32 first = 0, last = d->count;

	while (first < last) {
		u32 middle = first + (last - first) / 2;

		if (gpu_m4_drm_uat_change_va(&d->change[middle]) < page)
			first = middle + 1;
		else
			last = middle;
	}
	return first < d->count && gpu_m4_drm_uat_change_va(&d->change[first]) < end;
}

enum gpu_m4_drm_uat_group {
	GPU_M4_UAT_GROUP_IDENTITY,	/* same identity */
	GPU_M4_UAT_GROUP_PAGE,		/* same identity and offset */
	GPU_M4_UAT_GROUP_PA,		/* same PA */
};

/* The first unchanged member of probe's group in a target order, or NULL:
 * skips only new leaves of that group.
 */
static const struct gpu_m4_drm_vm_leaf *gpu_m4_drm_uat_unchanged(
	const struct gpu_m4_drm_uat_seq *s, const struct gpu_m4_drm_vm_leaf *probe,
	enum gpu_m4_drm_vm_leaf_key key, enum gpu_m4_drm_uat_group group,
	const struct gpu_m4_drm_uat_delta *d)
{
	struct gpu_m4_drm_vm_leaf start = *probe;
	struct gpu_m4_drm_uat_cursor c;
	const struct gpu_m4_drm_vm_leaf *p;

	if (group == GPU_M4_UAT_GROUP_IDENTITY)
		start.object_offset = 0;
	c = gpu_m4_drm_uat_seq_bound(s, &start, key);
	for (; (p = gpu_m4_drm_uat_cursor_leaf(&c)); gpu_m4_drm_uat_cursor_next(&c)) {
		if (group == GPU_M4_UAT_GROUP_PA ? p->pa != probe->pa :
		    p->identity != probe->identity ||
		    (group == GPU_M4_UAT_GROUP_PAGE && p->object_offset != probe->object_offset))
			return NULL;
		if (!gpu_m4_drm_uat_changed(d, p->va))
			return p;
	}
	return NULL;
}

/*
 * The builder's complete alias contract for a target block that differs from
 * its proven base block only by the changes d. Pairs of unchanged leaves were
 * proven with the base, so only pairs with a new leaf are checked: new/new
 * pairs are adjacent in the new leaves' own canonical views (pa, id), and
 * since the contract is an equality within each identity, identity+offset
 * and PA group, one unchanged member represents its group for all new ones.
 * Identity before PA, as the full proof reports them; VAs are distinct by
 * construction of the delta. O(d log n), never a pass over the block.
 */
static int gpu_m4_drm_uat_delta_aliases(const struct gpu_m4_drm_uat_layer *l,
	const struct gpu_m4_drm_uat_delta *d, const struct gpu_m4_drm_vm_leaf *const *pa,
	const struct gpu_m4_drm_vm_leaf *const *id, u32 n)
{
	const struct gpu_m4_drm_uat_seq *ids = &l->seq[GPU_M4_DRM_VM_KEY_IDENTITY];
	const struct gpu_m4_drm_uat_seq *pas = &l->seq[GPU_M4_DRM_VM_KEY_PA];
	int ret = gpu_m4_drm_uat_ids_check(id, n);

	for (u32 i = 0; !ret && i < n; i++) {
		const struct gpu_m4_drm_vm_leaf *y = id[i], *q;
		bool identity = !i || id[i - 1]->identity != y->identity;

		if (identity) {
			q = gpu_m4_drm_uat_unchanged(ids, y, GPU_M4_DRM_VM_KEY_IDENTITY,
						     GPU_M4_UAT_GROUP_IDENTITY, d);
			if (q && q->cpu_policy != y->cpu_policy)
				ret = -EPROTO;
		}
		if (!ret && (identity || id[i - 1]->object_offset != y->object_offset)) {
			q = gpu_m4_drm_uat_unchanged(ids, y, GPU_M4_DRM_VM_KEY_IDENTITY,
						     GPU_M4_UAT_GROUP_PAGE, d);
			if (q && q->pa != y->pa)
				ret = -EPROTO;
		}
	}
	if (!ret)
		ret = gpu_m4_drm_uat_pas_check(pa, n);
	for (u32 i = 0; !ret && i < n; i++) {
		const struct gpu_m4_drm_vm_leaf *y = pa[i], *q;

		if (i && pa[i - 1]->pa == y->pa)
			continue;
		q = gpu_m4_drm_uat_unchanged(pas, y, GPU_M4_DRM_VM_KEY_PA, GPU_M4_UAT_GROUP_PA, d);
		if (q && (q->identity != y->identity || q->object_offset != y->object_offset ||
			  q->cpu_policy != y->cpu_policy))
			ret = -EPERM;
	}
	return ret;
}

/*
 * The target block from the base block and the changes d: each order is the
 * base's with the old leaves deleted and the new ones inserted (every chunk
 * without a change shared), the runs are the target's mappings, and the delta
 * alias proof is made against it with the new leaves' canonical views.
 */
static int gpu_m4_drm_uat_rebase_block(const struct gpu_m4_drm_uat *v,
	const struct gpu_m4_drm_uat_layer *o, const struct gpu_m4_drm_uat_delta *d,
	struct gpu_m4_drm_uat_layer **out, u64 *touched)
{
	struct gpu_m4_drm_uat_segment **work = NULL, **segment[GPU_M4_UAT_ORDERS];
	const struct gpu_m4_drm_vm_leaf **del, **ins, **pa, **id;
	u32 count[GPU_M4_UAT_ORDERS] = { 0 }, nd = 0, ni = 0, done = 0;
	struct gpu_m4_drm_vm_leaf *pending = NULL;
	struct gpu_m4_drm_uat_chunk **chunk = NULL;
	struct gpu_m4_drm_uat_layer *l = NULL;
	u64 room = 0;
	int ret = -ENOMEM;

	del = kvmalloc_array(4 * (u64)d->count, sizeof(*del), GFP_KERNEL);
	if (!del)
		return -ENOMEM;
	ins = del + d->count;
	pa = ins + d->count;
	id = pa + d->count;
	for (u32 i = 0; i < d->count; i++) {
		if (d->change[i].old)
			del[nd++] = d->change[i].old;
		if (d->change[i].new)
			ins[ni++] = d->change[i].new;
	}
	for (u32 k = 0; k < GPU_M4_UAT_ORDERS; k++)
		room += gpu_m4_drm_uat_update_segments(&o->seq[k], ni);
	work = kvmalloc_array(room, sizeof(*work), GFP_KERNEL);
	chunk = kvmalloc_array(gpu_m4_drm_uat_update_chunks(ni), sizeof(*chunk), GFP_KERNEL);
	pending = kvmalloc_array((u64)GPU_M4_UAT_CHUNK + GPU_M4_UAT_CHUNK_MIN + ni,
				 sizeof(*pending), GFP_KERNEL);
	if (!work || !chunk || !pending)
		goto out;
	segment[0] = work;
	for (u32 k = 1; k < GPU_M4_UAT_ORDERS; k++)
		segment[k] = segment[k - 1] + gpu_m4_drm_uat_update_segments(&o->seq[k - 1], ni);
	memcpy(pa, ins, array_size(ni, sizeof(*pa)));
	memcpy(id, ins, array_size(ni, sizeof(*id)));
	sort(pa, ni, sizeof(*pa), gpu_m4_drm_uat_pa_order, NULL);
	sort(id, ni, sizeof(*id), gpu_m4_drm_uat_id_order, NULL);
	ret = gpu_m4_drm_uat_seq_update(segment[GPU_M4_DRM_VM_KEY_VA], &count[GPU_M4_DRM_VM_KEY_VA],
		&o->seq[GPU_M4_DRM_VM_KEY_VA], GPU_M4_DRM_VM_KEY_VA, del, nd, ins, ni, chunk,
		pending, touched);
	done += !ret;
	if (!ret) {
		sort(del, nd, sizeof(*del), gpu_m4_drm_uat_pa_order, NULL);
		ret = gpu_m4_drm_uat_seq_update(segment[GPU_M4_DRM_VM_KEY_PA],
			&count[GPU_M4_DRM_VM_KEY_PA], &o->seq[GPU_M4_DRM_VM_KEY_PA],
			GPU_M4_DRM_VM_KEY_PA, del, nd, pa, ni, chunk, pending, touched);
		done += !ret;
	}
	if (!ret) {
		sort(del, nd, sizeof(*del), gpu_m4_drm_uat_id_order, NULL);
		ret = gpu_m4_drm_uat_seq_update(segment[GPU_M4_DRM_VM_KEY_IDENTITY],
			&count[GPU_M4_DRM_VM_KEY_IDENTITY], &o->seq[GPU_M4_DRM_VM_KEY_IDENTITY],
			GPU_M4_DRM_VM_KEY_IDENTITY, del, nd, id, ni, chunk, pending, touched);
		done += !ret;
	}
	gpu_m4_drm_uat_build_stat(GPU_M4_DRM_UAT_STAT_SORTED, 2 * ((u64)nd + ni));
	if (!ret) {
		l = gpu_m4_drm_uat_layer_alloc(v->private_count, count);
		if (!l)
			ret = -ENOMEM;
	}
	if (!ret) {
		/* The block now owns every segment reference of the three orders. */
		for (u32 k = 0; k < GPU_M4_UAT_ORDERS; k++) {
			memcpy(l->seq[k].segment, segment[k],
			       array_size(count[k], sizeof(*segment[k])));
			l->seq[k].segments = count[k];
		}
		done = 0;
		gpu_m4_drm_uat_layer_runs(l, v->private, v->private_count);
		l->count = o->count - nd + ni;
		ret = gpu_m4_drm_uat_delta_aliases(l, d, pa, id, ni);
	}
out:
	for (u32 k = 0; k < done; k++)
		for (u32 c = 0; c < count[k]; c++)
			gpu_m4_drm_uat_segment_put(segment[k][c]);
	kvfree(work);
	kvfree(chunk);
	kvfree(pending);
	kvfree(del);
	if (ret) {
		if (l)
			gpu_m4_drm_uat_layer_free(l);
		return ret;
	}
	refcount_set(&l->refs, 1);
	l->depth = 1;
	*out = l;
	return 0;
}

/*
 * Rebase construction: no provider lock, no callback. Manifest work is
 * proportional to the delta d: the private changes come only from mappings
 * that are not kept runs of the base, the user changes from one merge only
 * when the user generation changed, each order of the target block copies
 * only the chunks a change touches (sharing every other chunk, and every
 * segment no change reaches, by one reference), and the alias proof checks
 * only the new leaves, O(d log n). Allocation, cleaning
 * and word construction/checking scale with the touched branches: only
 * changed L2 pages, their L1 parents and the root are copied (each once);
 * every copied word must equal the base's expected descriptor before it is
 * replaced or zeroed, additions require zero. Emptied branches stay named
 * (stable slots, zero leaf count). With no change at all the complete
 * immutable image, root and manifest block included, is shared.
 */
static int gpu_m4_drm_uat_rebase_construct(struct gpu_m4_drm_uat *v)
{
	struct gpu_m4_drm_uat_delta private = { 0 }, delta = { 0 };
	const struct gpu_m4_drm_uat_layer *o = v->rebase;
	struct gpu_m4_drm_uat_layer *l = NULL;
	struct gpu_m4_drm_uat_change *change;
	struct gpu_m4_drm_uat_table *grown;
	u32 changes, capacity;
	bool overlap = false;
	u64 touched = 0;
	int ret;

	ret = v->user_same ? 0 : gpu_m4_drm_uat_user_view(&v->user);
	if (!ret) {
		ret = gpu_m4_drm_uat_private_delta(v, o, &private, &touched);
		overlap = ret == 1;
		ret = overlap ? 0 : ret;
	}
	if (!ret && v->user_same) {
		delta = private;
		private = (struct gpu_m4_drm_uat_delta) { 0 };
	} else if (!ret) {
		ret = gpu_m4_drm_uat_user_delta(o, &v->user, &private, &delta, &touched);
	}
	kvfree(private.change);
	/* Changed target user leaves receive the builder's page admission;
	 * unchanged ones are the base's, private ones were admitted at prepare
	 * or are kept runs of the base.
	 */
	for (u32 i = 0; !ret && i < delta.count; i++)
		if (delta.change[i].new && !gpu_m4_drm_uat_reserved(delta.change[i].new->va))
			ret = gpu_m4_drm_uat_leaf_admit(delta.change[i].new);
	if (!ret && overlap)
		ret = gpu_m4_drm_uat_rebase_refusal(v, o);
	if (!ret && delta.count)
		ret = gpu_m4_drm_uat_rebase_block(v, o, &delta, &l, &touched);
	if (ret) {
		kvfree(delta.change);
		return ret;
	}
	if (l)
		gpu_m4_drm_uat_span_carry(l, o, &delta, NULL);
	change = delta.change;
	changes = delta.count;
	if (!changes) {
		/* Exact unchanged image: the base block is the target leaf for leaf. */
		kvfree(v->leaf);
		v->leaf = NULL;
		v->fresh = 0;
		v->count = o->count;
		v->manifest = v->rebase;
		v->rebase = NULL;
		ret = gpu_m4_drm_uat_walk(v, true);
		if (ret)
			return ret;
		goto done;
	}
	v->count = l->count;
	v->manifest = l;
	capacity = v->tables + 2 * changes;
	if (capacity > v->table_limit)
		capacity = v->table_limit;
	if (capacity < v->tables)
		capacity = v->tables;
	grown = kvcalloc(capacity, sizeof(*grown), GFP_KERNEL);
	if (!grown) {
		kvfree(change);
		return -ENOMEM;
	}
	memcpy(grown, v->table, array_size(v->tables, sizeof(*grown)));
	kvfree(v->table);
	v->table = grown;
	v->table_capacity = capacity;
	ret = gpu_m4_drm_uat_node_new(v, &v->table[0], 0, 0, v->table[0].node);
	for (u32 i = 0; !ret && i < changes; i++) {
		const struct gpu_m4_drm_vm_leaf *x = change[i].old, *y = change[i].new;
		u64 va = x ? x->va : y->va, expected;
		void *cell;
		int l1, l2;

		l1 = gpu_m4_drm_uat_child(v, 0, 1, gpu_m4_uat_fmt_base(va, 1));
		if (l1 < 0) {
			ret = l1;
			break;
		}
		l2 = gpu_m4_drm_uat_child(v, l1, 2, gpu_m4_uat_fmt_base(va, 2));
		if (l2 < 0) {
			ret = l2;
			break;
		}
		cell = v->table[l2].cpu + gpu_m4_uat_fmt_slot(va, 2) * 8;
		expected = x ? gpu_m4_uat_fmt_leaf_pte(x->pa, x->access) : 0;
		if (get_unaligned_le64(cell) != expected) {
			ret = -EPROTO;
			break;
		}
		put_unaligned_le64(y ? gpu_m4_uat_fmt_leaf_pte(y->pa, y->access) : 0, cell);
	}
	/* The new leaves were copied into the block and written: drop them. */
	kvfree(change);
	kvfree(v->leaf);
	v->leaf = NULL;
	v->fresh = 0;
	if (!ret)
		ret = gpu_m4_drm_uat_table_owners(v);
	if (!ret)
		ret = gpu_m4_drm_uat_finish_tables(v);
	if (!ret)
		ret = gpu_m4_drm_uat_walk(v, true);
	if (ret)
		return ret;
	gpu_m4_drm_uat_drop_replaced(v);
	for (u32 i = 0; i < v->tables; i++)
		if (v->table[i].fresh)
			v->table[i].node->proven = true;
	gpu_m4_drm_uat_layer_put(v->rebase);
	v->rebase = NULL;
done:
	/* REMOVE-DEBUG: how much of the manifest this rebase visited. */
	gpu_m4_drm_uat_build_stat(GPU_M4_DRM_UAT_STAT_REBASE_DELTA, changes);
	gpu_m4_drm_uat_build_stat(GPU_M4_DRM_UAT_STAT_REBASE_TOUCHED, touched);
	v->proof_manifest = v->manifest;
	v->cleaned = true;
	v->checked_low = v->table[0].pa | 1;
	return 0;
}

/* A constructed graph or cache image whose single manifest block and proven
 * nodes can serve as a rebase base.
 */
bool gpu_m4_drm_uat_rebase_ready(const struct gpu_m4_drm_uat *v)
{
	if (!v || !v->cleaned || !v->table || !v->tables || v->tables > v->table_capacity ||
	    !v->manifest || v->manifest->below || v->manifest->depth != 1 ||
	    (!v->snapshot && !v->image) || v->rebase)
		return false;
	for (u32 i = 0; i < v->tables; i++)
		if (!v->table[i].node || !v->table[i].node->proven)
			return false;
	return true;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_rebase_ready);

/* Does this constructed graph/image name any retained empty table branch, so
 * that a compact from-scratch build of a superset target could need fewer
 * tables than a derivation inheriting its inventory?
 */
bool gpu_m4_drm_uat_compactable(const struct gpu_m4_drm_uat *v)
{
	if (!v || !v->cleaned || !v->table)
		return false;
	for (u32 i = 0; i < v->tables; i++)
		if (v->table[i].node && !v->table[i].node->used)
			return true;
	return false;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_compactable);

u32 gpu_m4_drm_uat_leaves(const struct gpu_m4_drm_uat *v)
{
	return v ? v->count : 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_leaves);

struct gpu_m4_drm_uat *gpu_m4_drm_uat_rebase_prepare(struct gpu_m4_drm_uat *base,
	struct gpu_m4_drm_vm_owner *snapshot, const struct gpu_m4_drm_uat_user *user,
	const struct gpu_m4_drm_uat_private *private, const bool *kept, u32 private_count,
	u32 max_table_pages, u32 max_leaf_pages, struct gpu_m4_drm_uat **garbage)
{
	struct gpu_m4_drm_vm *vm = gpu_m4_drm_vm_owner_vm(snapshot);
	struct gpu_m4_drm_uat *v;
	u64 total, fresh = 0, resolved = 0;
	u32 fixed = 0;
	int ret;

	if (!garbage)
		return ERR_PTR(-EINVAL);
	*garbage = NULL;
	if (!snapshot || !vm || !base || !user || !private || !private_count ||
	    (user->count && (!user->leaf || !user->va)) ||
	    user->count != gpu_m4_drm_vm_owner_leaf_count(snapshot) ||
	    !max_table_pages || max_table_pages > GPU_M4_DRM_UAT_MAX_TABLES ||
	    !max_leaf_pages || max_leaf_pages > GPU_M4_DRM_UAT_MAX_LEAVES ||
	    private_count > max_leaf_pages || PAGE_SIZE > GPU_M4_DRM_PAGE_SIZE)
		return ERR_PTR(-EINVAL);
	total = user->count;
	for (u32 i = 0; i < private_count; i++) {
		ret = gpu_m4_drm_uat_private_shape(&private[i]);
		if (ret)
			return ERR_PTR(ret);
		fixed += private[i].va == GPU_M4_DRM_UAT_CONTEXT_START;
		fresh += private[i].size / GPU_M4_DRM_PAGE_SIZE;
		if (total + fresh > max_leaf_pages)
			return ERR_PTR(-ENOSPC);
	}
	if (fixed != 1)
		return ERR_PTR(-EINVAL);
	/* A construction base only: same VM, never authority for the target. */
	if (!gpu_m4_drm_uat_rebase_ready(base) ||
	    (base->snapshot ? gpu_m4_drm_vm_owner_vm(base->snapshot) : base->image_vm) != vm)
		return ERR_PTR(-ESTALE);
	if (base->tables > max_table_pages)
		return ERR_PTR(-ENOSPC);
	v = kzalloc(sizeof(*v), GFP_KERNEL);
	if (!v)
		return ERR_PTR(-ENOMEM);
	refcount_set(&v->refs, 1);
	v->private = kvcalloc(private_count, sizeof(*private), GFP_KERNEL);
	v->keep = kvcalloc(private_count, sizeof(*v->keep), GFP_KERNEL);
	v->table = kvcalloc(base->tables, sizeof(*v->table), GFP_KERNEL);
	if (!v->private || !v->keep || !v->table) {
		ret = -ENOMEM;
		goto fail;
	}
	/* A mapping the caller proved kept (same continuously held backing)
	 * whose exact shape is a run of the base block keeps the base's leaves
	 * by reference; every other one is resolved again. Only the resolved
	 * pages are held as fresh leaves.
	 */
	for (u32 i = 0; i < private_count; i++) {
		v->keep[i] = kept && kept[i] &&
			gpu_m4_drm_uat_run_find(base->manifest, &private[i]) >= 0;
		resolved += v->keep[i] ? 0 : private[i].size / GPU_M4_DRM_PAGE_SIZE;
	}
	v->leaf = kvcalloc(resolved ? resolved : 1, sizeof(*v->leaf), GFP_KERNEL);
	if (!v->leaf) {
		ret = -ENOMEM;
		goto fail;
	}
	memcpy(v->private, private, array_size(private_count, sizeof(*private)));
	v->private_count = private_count;
	v->table_limit = max_table_pages;
	v->table_capacity = base->tables;
	v->user = *user;
	v->user_count = user->count;
	/* Within one VM a generation names exactly one immutable user ledger
	 * (the share key's argument): the same generation and count keep the
	 * base's user leaves without consulting the view.
	 */
	v->user_same = (base->snapshot ? gpu_m4_drm_vm_owner_generation(base->snapshot) :
			base->image_generation) == gpu_m4_drm_vm_owner_generation(snapshot) &&
		base->user_count == user->count;
	/* Real holds on every base node and its manifest block. */
	for (u32 i = 0; i < base->tables; i++) {
		refcount_inc(&base->table[i].node->refs);
		v->table[i] = base->table[i];
		v->table[i].fresh = false;
		v->table[i].replaces = NULL;
	}
	v->tables = base->tables;
	refcount_inc(&base->manifest->refs);
	v->rebase = base->manifest;
	ret = gpu_m4_drm_vm_owner_get_unpublished(snapshot);
	if (ret)
		goto fail;
	v->snapshot = snapshot;
	/* Every private owner is held again. Its leaves are resolved again and
	 * compared, never trusted, so a changed one is a delta; only a kept run
	 * (above) is the base block's own.
	 */
	for (u32 i = 0; i < private_count; i++) {
		const struct gpu_m4_drm_uat_private *p = &v->private[i];

		ret = p->get(p->cookie, vm, p->identity);
		if (ret)
			goto fail;
		v->held++;
		if (v->keep[i]) {
			gpu_m4_drm_uat_build_stat(GPU_M4_DRM_UAT_STAT_PRIVATE_KEPT,
						  p->size / GPU_M4_DRM_PAGE_SIZE);
			continue;
		}
		gpu_m4_drm_uat_build_stat(GPU_M4_DRM_UAT_STAT_PRIVATE_RESOLVED,
					  p->size / GPU_M4_DRM_PAGE_SIZE);
		for (u64 offset = 0; offset < p->size; offset += GPU_M4_DRM_PAGE_SIZE) {
			struct gpu_m4_drm_vm_leaf leaf = {
				.va = p->va + offset, .identity = p->identity,
				.object_offset = p->offset + offset,
				.access = p->access, .cpu_policy = p->cpu_policy,
			};

			ret = p->page(p->cookie, leaf.object_offset, &leaf.pa);
			if (!ret)
				ret = gpu_m4_drm_uat_leaf_add(v, &leaf);
			if (ret)
				goto fail;
		}
	}
	v->prepared = true;
	return v;
fail:
	*garbage = v;
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_rebase_prepare);

struct gpu_m4_drm_uat *gpu_m4_drm_uat_rebase_deferred(struct gpu_m4_drm_uat *base,
	struct gpu_m4_drm_vm_owner *snapshot, const struct gpu_m4_drm_uat_user *user,
	const struct gpu_m4_drm_uat_private *private, const bool *kept, u32 private_count,
	u32 max_table_pages, u32 max_leaf_pages, struct gpu_m4_drm_uat **garbage)
{
	struct gpu_m4_drm_uat *v = gpu_m4_drm_uat_rebase_prepare(base, snapshot, user,
		private, kept, private_count, max_table_pages, max_leaf_pages, garbage);
	int ret;

	if (IS_ERR(v))
		return v;
	ret = gpu_m4_drm_uat_construct(v);
	if (!ret)
		return v;
	*garbage = v;
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_rebase_deferred);

/* Construction-cache image of a constructed graph: its proven nodes and its
 * single manifest block under strong refs; no snapshot, owner callback hold,
 * residency or authority. Callback-free to create and to release.
 */
struct gpu_m4_drm_uat *gpu_m4_drm_uat_image(const struct gpu_m4_drm_uat *v)
{
	struct gpu_m4_drm_uat *c;

	if (gpu_m4_drm_uat_checked(v) || !gpu_m4_drm_uat_rebase_ready(v))
		return ERR_PTR(-EINVAL);
	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);
	c->table = kvcalloc(v->tables, sizeof(*c->table), GFP_KERNEL);
	c->private = kvcalloc(v->private_count, sizeof(*c->private), GFP_KERNEL);
	if (!c->table || !c->private) {
		kvfree(c->table);
		kvfree(c->private);
		kfree(c);
		return ERR_PTR(-ENOMEM);
	}
	/* Descriptors only, for exact use-wrapper keys: no owner hold. */
	memcpy(c->private, v->private, array_size(v->private_count, sizeof(*c->private)));
	c->private_count = v->private_count;
	refcount_set(&c->refs, 1);
	for (u32 i = 0; i < v->tables; i++) {
		refcount_inc(&v->table[i].node->refs);
		c->table[i] = v->table[i];
		c->table[i].fresh = false;
		c->table[i].replaces = NULL;
	}
	c->tables = c->table_capacity = v->tables;
	c->table_limit = v->table_limit;
	refcount_inc(&v->manifest->refs);
	c->manifest = v->manifest;
	c->proof_manifest = v->manifest;
	c->count = v->count;
	c->user_count = v->user_count;
	c->checked_low = v->checked_low;
	c->image_vm = gpu_m4_drm_vm_owner_vm(v->snapshot);
	c->image_generation = gpu_m4_drm_vm_owner_generation(v->snapshot);
	c->image = true;
	c->cleaned = true;
	return c;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_image);

/*
 * A use wrapper of an exact constructed image (graph or cache image) of the
 * same VM: same user generation and count (within one VM a generation names
 * exactly one immutable user ledger) and an identical private descriptor
 * inventory. It shares every node, the root and the manifest block, holds its
 * own snapshot and private owners (no page resolution), and has its own
 * residency/work state. The caller also proves the private membership key.
 */
struct gpu_m4_drm_uat *gpu_m4_drm_uat_share_prepare(struct gpu_m4_drm_uat *base,
	struct gpu_m4_drm_vm_owner *snapshot, const struct gpu_m4_drm_uat_private *private,
	u32 private_count, u32 max_table_pages, struct gpu_m4_drm_uat **garbage)
{
	struct gpu_m4_drm_vm *vm = gpu_m4_drm_vm_owner_vm(snapshot);
	struct gpu_m4_drm_uat *v;
	int ret;

	if (!garbage)
		return ERR_PTR(-EINVAL);
	*garbage = NULL;
	if (!snapshot || !vm || !private || !private_count || !max_table_pages ||
	    max_table_pages > GPU_M4_DRM_UAT_MAX_TABLES)
		return ERR_PTR(-EINVAL);
	if (!gpu_m4_drm_uat_rebase_ready(base) || base->share || base->prepared ||
	    (base->snapshot ? gpu_m4_drm_vm_owner_vm(base->snapshot) : base->image_vm) != vm ||
	    (base->snapshot ? gpu_m4_drm_vm_owner_generation(base->snapshot) :
	     base->image_generation) != gpu_m4_drm_vm_owner_generation(snapshot) ||
	    base->user_count != gpu_m4_drm_vm_owner_leaf_count(snapshot) ||
	    base->private_count != private_count || !base->private)
		return ERR_PTR(-ESTALE);
	for (u32 i = 0; i < private_count; i++)
		if (!gpu_m4_drm_uat_private_same(&base->private[i], &private[i]))
			return ERR_PTR(-ESTALE);
	if (base->tables > max_table_pages)
		return ERR_PTR(-ENOSPC);
	v = kzalloc(sizeof(*v), GFP_KERNEL);
	if (!v)
		return ERR_PTR(-ENOMEM);
	refcount_set(&v->refs, 1);
	v->private = kvcalloc(private_count, sizeof(*private), GFP_KERNEL);
	v->table = kvcalloc(base->tables, sizeof(*v->table), GFP_KERNEL);
	if (!v->private || !v->table) {
		ret = -ENOMEM;
		goto fail;
	}
	memcpy(v->private, private, array_size(private_count, sizeof(*private)));
	v->private_count = private_count;
	for (u32 i = 0; i < base->tables; i++) {
		refcount_inc(&base->table[i].node->refs);
		v->table[i] = base->table[i];
		v->table[i].fresh = false;
		v->table[i].replaces = NULL;
	}
	v->tables = v->table_capacity = base->tables;
	v->table_limit = max_table_pages;
	refcount_inc(&base->manifest->refs);
	v->manifest = base->manifest;
	v->count = base->count;
	v->user_count = base->user_count;
	v->checked_low = base->checked_low;
	ret = gpu_m4_drm_vm_owner_get_unpublished(snapshot);
	if (ret)
		goto fail;
	v->snapshot = snapshot;
	for (u32 i = 0; i < private_count; i++) {
		const struct gpu_m4_drm_uat_private *p = &v->private[i];

		ret = p->get(p->cookie, vm, p->identity);
		if (ret)
			goto fail;
		v->held++;
	}
	v->share = true;
	v->prepared = true;
	return v;
fail:
	*garbage = v;
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_share_prepare);

int gpu_m4_drm_uat_verify(const struct gpu_m4_drm_uat *v)
{
	/* The complete walk never writes: fresh_only=false leaves counts alone. */
	return gpu_m4_drm_uat_walk((struct gpu_m4_drm_uat *)v, false);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_verify);

/* Retained allocation identity of the immutable checked graph; no leaf walk. */
static int gpu_m4_drm_uat_checked(const struct gpu_m4_drm_uat *v)
{
	const struct gpu_m4_drm_uat_table *root;

	if (!v || !v->cleaned || !v->snapshot || !v->table || !v->tables || !v->manifest ||
	    v->tables > v->table_limit || v->tables > v->table_capacity ||
	    v->table_limit > GPU_M4_DRM_UAT_MAX_TABLES || !refcount_read(&v->refs))
		return -EINVAL;
	root = &v->table[0];
	if (!root->allocation || root->level || root->base || !root->cpu || !root->node ||
	    root->node->allocation != root->allocation ||
	    root->cpu != page_address(root->allocation) ||
	    root->pa != page_to_phys(root->allocation) ||
	    v->checked_low != (root->pa | 1))
		return -EPROTO;
	return 0;
}

int gpu_m4_drm_uat_check(const struct gpu_m4_drm_uat *v)
{
	return gpu_m4_drm_uat_checked(v);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_check);

int gpu_m4_drm_uat_leaf_words(const struct gpu_m4_drm_uat *v,
	int (*fn)(void *cookie, u64 va, u64 word), void *cookie)
{
	int ret = fn ? gpu_m4_drm_uat_checked(v) : -EINVAL;

	for (u32 i = 0; !ret && i < v->tables; i++) {
		const struct gpu_m4_drm_uat_table *t = &v->table[i];

		for (u32 k = 0; !ret && t->level == 2 && k < GPU_M4_UAT_ENTRIES; k++) {
			u64 raw = get_unaligned_le64(t->cpu + k * 8);

			if (raw)
				ret = fn(cookie, t->base | ((u64)k << gpu_m4_uat_fmt_shift(2)), raw);
		}
	}
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_leaf_words);

/*
 * The construction proof a binding consumes: constructed identity, every
 * private owner still held, the exact manifest the proof was made against and,
 * for every node this graph names (fresh or shared), its retained record,
 * exact parent-slot naming and its creator's completed proof. Metadata only:
 * no table page is read, so shared immutable pages are never rescanned.
 */
static int gpu_m4_drm_uat_proven(const struct gpu_m4_drm_uat *v)
{
	int ret = gpu_m4_drm_uat_checked(v);

	if (ret)
		return ret;
	if (!v->private || v->held != v->private_count || v->proof_manifest != v->manifest)
		return -EPROTO;
	for (u32 i = 0; i < v->tables; i++) {
		const struct gpu_m4_drm_uat_table *t = &v->table[i];

		ret = gpu_m4_drm_uat_record(v, i);
		if (ret)
			return ret;
		if (t->cpu != page_address(t->allocation) || !t->node->proven)
			return -EPROTO;
	}
	return 0;
}

/* Current image of the root page: every word names exactly an owned level-1
 * table and every owned level-1 table is named. Only the root page is read.
 */
static int gpu_m4_drm_uat_root_image(const struct gpu_m4_drm_uat *v)
{
	const struct gpu_m4_drm_uat_table *root = &v->table[0];
	u32 children = 0;

	for (u32 i = 1; i < v->tables; i++)
		children += v->table[i].level == 1;
	for (u32 index = 0; index < GPU_M4_UAT_ENTRIES; index++) {
		u64 raw = get_unaligned_le64(root->cpu + index * 8);
		int child;

		if (!raw)
			continue;
		child = index < GPU_M4_UAT_ROOT_ENTRIES ?
			gpu_m4_drm_uat_find_table(v, 1, (u64)index << gpu_m4_uat_fmt_shift(0)) : -EPROTO;
		if (child < 0 || raw != gpu_m4_uat_fmt_table_pte(v->table[child].pa))
			return -EPROTO;
		children--;
	}
	return children ? -EPROTO : 0;
}

/*
 * Exact unchanged physical image of two separate constructed graphs: same VM,
 * the same immutable manifest block (complete user and private leaf tuples:
 * VA, PA, identity, offset, permission, policy), the same private owner
 * descriptors, and the same retained root/node inventory slot for slot. Both
 * construction proofs must stand. Equal counts or equal roots alone are not.
 */
bool gpu_m4_drm_uat_same_image(const struct gpu_m4_drm_uat *a, const struct gpu_m4_drm_uat *b)
{
	if (!a || !b || a == b || gpu_m4_drm_uat_proven(a) || gpu_m4_drm_uat_proven(b) ||
	    gpu_m4_drm_vm_owner_vm(a->snapshot) != gpu_m4_drm_vm_owner_vm(b->snapshot) ||
	    !a->manifest || a->manifest != b->manifest || a->count != b->count ||
	    a->user_count != b->user_count || a->tables != b->tables ||
	    a->checked_low != b->checked_low || a->private_count != b->private_count)
		return false;
	for (u32 i = 0; i < a->private_count; i++)
		if (!gpu_m4_drm_uat_private_same(&a->private[i], &b->private[i]))
			return false;
	for (u32 i = 0; i < a->tables; i++) {
		const struct gpu_m4_drm_uat_table *x = &a->table[i], *y = &b->table[i];

		if (x->node != y->node || x->allocation != y->allocation || x->pa != y->pa ||
		    x->cpu != y->cpu || x->base != y->base || x->level != y->level)
			return false;
	}
	return true;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_same_image);

int gpu_m4_drm_uat_refresh_check(const struct gpu_m4_drm_uat *v)
{
	const struct gpu_m4_drm_uat_table *root;
	int ret = gpu_m4_drm_uat_proven(v);

	if (ret)
		return ret;
	/* Refresh the live root page the selected TTBAT names; every root word
	 * must still name exactly its owned level-1 table. Level-1/2 contents
	 * are covered by their nodes' construction proofs.
	 */
	root = &v->table[0];
	dcache_inval_poc((unsigned long)root->cpu, (unsigned long)root->cpu + GPU_M4_DRM_PAGE_SIZE);
	return gpu_m4_drm_uat_root_image(v);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_refresh_check);

int gpu_m4_drm_uat_refresh_verify(const struct gpu_m4_drm_uat *v)
{
	/* This API never accepts a partially built, AP-dirty graph. The immutable
	 * manifest is separate from the live table bytes and is not overwritten.
	 */
	if (!v || !v->cleaned || !v->snapshot || !v->table || !v->tables ||
	    v->tables > v->table_limit || v->table_limit > GPU_M4_DRM_UAT_MAX_TABLES)
		return -EINVAL;
	/* Check every owned address before the first cache operation. No live PTE
	 * or foreign PA is converted into a CPU pointer here.
	 */
	for (u32 i = 0; i < v->tables; i++) {
		const struct gpu_m4_drm_uat_table *t = &v->table[i];

		if (!t->allocation || !t->cpu || t->cpu != page_address(t->allocation) ||
		    t->pa != page_to_phys(t->allocation) ||
		    ((unsigned long)t->cpu & (GPU_M4_DRM_PAGE_SIZE - 1)) ||
		    (unsigned long)t->cpu > ~0UL - GPU_M4_DRM_PAGE_SIZE ||
		    gpu_m4_drm_uat_ram(t->pa))
			return -EPROTO;
	}
	for (u32 i = 0; i < v->tables; i++)
		dcache_inval_poc((unsigned long)v->table[i].cpu,
			(unsigned long)v->table[i].cpu + GPU_M4_DRM_PAGE_SIZE);
	return gpu_m4_drm_uat_verify(v);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_refresh_verify);

static bool gpu_m4_drm_uat_evidence_overlap(const void *a, unsigned long a_size,
					 const void *b, unsigned long b_size)
{
	unsigned long first = (unsigned long)a, second = (unsigned long)b;

	return a_size && b_size && first < second + b_size && second < first + a_size;
}

int gpu_m4_drm_uat_evidence_destination(const struct gpu_m4_drm_uat *v,
	const void *storage, unsigned long bytes)
{
	if (!v || !storage || !bytes || (unsigned long)storage > ~0UL - bytes ||
	    !v->table || !v->tables || v->tables > v->table_limit ||
	    v->tables > v->table_capacity || v->table_limit > GPU_M4_DRM_UAT_MAX_TABLES)
		return -EINVAL;
	if (gpu_m4_drm_uat_evidence_overlap(storage, bytes, v, sizeof(*v)) ||
	    gpu_m4_drm_uat_evidence_overlap(storage, bytes, v->table,
		(unsigned long)v->table_capacity * sizeof(*v->table)))
		return -EINVAL;
	for (u32 i = 0; i < v->tables; i++) {
		const struct gpu_m4_drm_uat_table *t = &v->table[i];

		if (!t->cpu || (unsigned long)t->cpu > ~0UL - GPU_M4_DRM_PAGE_SIZE)
			return -EPROTO;
		if (gpu_m4_drm_uat_evidence_overlap(storage, bytes, t->cpu,
			GPU_M4_DRM_PAGE_SIZE))
			return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_evidence_destination);

int gpu_m4_drm_uat_export(const struct gpu_m4_drm_uat *v,
	struct gpu_m4_drm_uat_evidence *header,
	struct gpu_m4_drm_uat_table_evidence *tables, u32 capacity)
{
	struct gpu_m4_drm_uat_evidence result;
	unsigned long bytes = (unsigned long)capacity * sizeof(*tables);

	if (!v || !header || (capacity && !tables) || capacity > GPU_M4_DRM_UAT_MAX_TABLES ||
	    (unsigned long)header > ~0UL - sizeof(*header) ||
	    (tables && (unsigned long)tables > ~0UL - bytes) ||
	    !refcount_read(&v->refs) || !v->cleaned || !v->snapshot || !v->table ||
	    !v->tables || v->tables > v->table_limit || v->tables > v->table_capacity ||
	    v->table_limit > GPU_M4_DRM_UAT_MAX_TABLES)
		return -EINVAL;
	if (gpu_m4_drm_uat_evidence_overlap(header, sizeof(*header), v, sizeof(*v)) ||
	    gpu_m4_drm_uat_evidence_overlap(tables, bytes, v, sizeof(*v)) ||
	    gpu_m4_drm_uat_evidence_overlap(header, sizeof(*header), v->table,
		(unsigned long)v->table_capacity * sizeof(*v->table)) ||
	    gpu_m4_drm_uat_evidence_overlap(tables, bytes, v->table,
		(unsigned long)v->table_capacity * sizeof(*v->table)) ||
	    gpu_m4_drm_uat_evidence_overlap(header, sizeof(*header), tables, bytes))
		return -EINVAL;
	/* Validate the complete owned inventory before any cache operation/read.
	 * Raw parent/leaf bytes may be corrupt; never use them as CPU addresses.
	 */
	for (u32 i = 0; i < v->tables; i++) {
		const struct gpu_m4_drm_uat_table *t = &v->table[i];
		u64 span = t->level == 1 ? 1ULL << 36 : 1ULL << 25;

		if (!t->allocation || !t->cpu || t->cpu != page_address(t->allocation) ||
		    t->pa != page_to_phys(t->allocation) ||
		    ((unsigned long)t->cpu & (GPU_M4_DRM_PAGE_SIZE - 1)) ||
		    (unsigned long)t->cpu > ~0UL - GPU_M4_DRM_PAGE_SIZE ||
		    gpu_m4_drm_uat_ram(t->pa) || t->level > 2 ||
		    (!i && (t->level || t->base)) || (i && !t->level) ||
		    t->base >= (1ULL << 42) || (t->base & (span - 1)))
			return -EPROTO;
		if (gpu_m4_drm_uat_evidence_overlap(header, sizeof(*header), t->cpu,
			GPU_M4_DRM_PAGE_SIZE) ||
		    gpu_m4_drm_uat_evidence_overlap(tables, bytes, t->cpu,
			GPU_M4_DRM_PAGE_SIZE))
			return -EINVAL;
		if (i && gpu_m4_drm_uat_find_table(v, t->level - 1,
			 t->level == 1 ? 0 : t->base & ~((1ULL << 36) - 1)) < 0)
			return -EPROTO;
		for (u32 j = 0; j < i; j++)
			if (v->table[j].allocation == t->allocation || v->table[j].pa == t->pa ||
			    (v->table[j].level == t->level && v->table[j].base == t->base))
				return -EPROTO;
	}
	/* Diagnostic export: the TTBAT word of the qualification ASID 1. */
	result = (struct gpu_m4_drm_uat_evidence) {
		.generation = gpu_m4_drm_uat_generation(v),
		.expected_low = v->table[0].pa | 1 | (1ULL << 48),
		.table_count = v->tables, .verify_error = -ENODATA,
	};
	if (capacity < v->tables) {
		*header = result;
		return -ENOSPC;
	}
	for (u32 i = 0; i < v->tables; i++)
		dcache_inval_poc((unsigned long)v->table[i].cpu,
			(unsigned long)v->table[i].cpu + GPU_M4_DRM_PAGE_SIZE);
	for (u32 i = 0; i < v->tables; i++) {
		const struct gpu_m4_drm_uat_table *t = &v->table[i];

		tables[i].va_base = t->base;
		tables[i].pa = t->pa;
		tables[i].index = i;
		tables[i].parent_index = i ? (u32)gpu_m4_drm_uat_find_table(v, t->level - 1,
			t->level == 1 ? 0 : t->base & ~((1ULL << 36) - 1)) : U32_MAX;
		tables[i].parent_entry = i ? (t->base >> (t->level == 1 ? 36 : 25)) & 2047 : 0;
		tables[i].level = t->level;
		memcpy(tables[i].bytes, t->cpu, GPU_M4_DRM_PAGE_SIZE);
	}
	result.verify_error = gpu_m4_drm_uat_verify(v);
	/* Never label a copy as verified if even its refreshed source changed. */
	for (u32 i = 0; i < v->tables; i++)
		if (memcmp(tables[i].bytes, v->table[i].cpu, GPU_M4_DRM_PAGE_SIZE))
			result.verify_error = -EAGAIN;
	*header = result;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_export);

int gpu_m4_drm_uat_page(const struct gpu_m4_drm_uat *v, u64 va,
			 u32 access, struct gpu_m4_drm_vm_leaf *leaf)
{
	const struct gpu_m4_drm_vm_leaf *p;
	u64 page = va & ~(GPU_M4_DRM_PAGE_SIZE - 1);
	u32 table = 0;

	if (!v || !leaf || !gpu_m4_drm_vm_leaf_attrs(access) || va >= (1ULL << 42))
		return -EINVAL;
	gpu_m4_uat_stat[GPU_M4_DRM_UAT_STAT_PAGE_LOOKUPS]++;
	p = gpu_m4_drm_uat_layer_va(v->manifest, page);
	if (!p)
		return -ENOENT;
	if ((p->access & access) != access)
		return -EACCES;
	if (!v->tables)
		return -EPROTO;
	for (u32 level = 0; level < 3; level++) {
		u64 raw = get_unaligned_le64(v->table[table].cpu + gpu_m4_uat_fmt_slot(va, level) * 8);

		if (level == 2) {
			if (raw != gpu_m4_uat_fmt_leaf_pte(p->pa, p->access))
				return -EPROTO;
		} else {
			u64 base = gpu_m4_uat_fmt_base(va, level + 1);
			int next = gpu_m4_drm_uat_find_table(v, level + 1, base);

			if (next < 0 || raw != gpu_m4_uat_fmt_table_pte(v->table[next].pa))
				return -EPROTO;
			table = next;
		}
	}
	*leaf = *p;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_page);

/* The leaf-table index naming page's 32 MiB block, its root and level-1
 * parent words checked once per block (the same words uat_page checks).
 */
static int gpu_m4_drm_uat_block(const struct gpu_m4_drm_uat *v, u64 page)
{
	u32 table = 0;

	for (u32 level = 0; level < 2; level++) {
		u64 raw = get_unaligned_le64(v->table[table].cpu + gpu_m4_uat_fmt_slot(page, level) * 8);
		u64 base = gpu_m4_uat_fmt_base(page, level + 1);
		int next = gpu_m4_drm_uat_find_table(v, level + 1, base);

		if (next < 0 || raw != gpu_m4_uat_fmt_table_pte(v->table[next].pa))
			return -EPROTO;
		table = next;
	}
	return table;
}

/*
 * Every page of [va, va + size): exactly the per-page gpu_m4_drm_uat_page()
 * result, in VA order, but one indexed lookup per manifest block for the whole
 * run, then sequential comparison of consecutive VA-sorted leaves (blocks are
 * disjoint, so each page is the current leaf of exactly one block cursor),
 * and the root/level-1 parents once per 32 MiB block. fn (optional) sees each
 * page's manifest leaf; a nonzero return stops the walk with that value.
 */
int gpu_m4_drm_uat_range(const struct gpu_m4_drm_uat *v, u64 va, u64 size, u32 access,
	gpu_m4_drm_uat_leaf_fn fn, void *cookie)
{
	struct gpu_m4_drm_uat_cursor at[2 * GPU_M4_UAT_MAX_DEPTH];
	struct gpu_m4_drm_vm_leaf probe = { 0 };
	const struct gpu_m4_drm_uat_layer *l;
	u64 page, end, block = ~0ULL;
	int table = -EPROTO, ret;
	u32 layers = 0;

	if (!v || !size || !gpu_m4_drm_vm_leaf_attrs(access) || va >= (1ULL << 42) ||
	    size > (1ULL << 42) - va)
		return -EINVAL;
	end = va + size;
	page = va & ~(GPU_M4_DRM_PAGE_SIZE - 1);
	for (l = v->manifest; l; l = l->below) {
		if (layers == 2 * GPU_M4_UAT_MAX_DEPTH) {
			/* Unflattened deep chain: the exact per-page lookup. */
			for (; page < end; page += GPU_M4_DRM_PAGE_SIZE) {
				struct gpu_m4_drm_vm_leaf leaf;

				ret = gpu_m4_drm_uat_page(v, page, access, &leaf);
				if (!ret && fn)
					ret = fn(cookie, page, &leaf);
				if (ret)
					return ret;
			}
			return 0;
		}
		probe.va = page;
		at[layers++] = gpu_m4_drm_uat_seq_bound(&l->seq[GPU_M4_DRM_VM_KEY_VA], &probe,
							GPU_M4_DRM_VM_KEY_VA);
	}
	gpu_m4_uat_stat[GPU_M4_DRM_UAT_STAT_RUNS]++;
	gpu_m4_uat_stat[GPU_M4_DRM_UAT_STAT_LOOKUPS] += layers;
	for (; page < end; page += GPU_M4_DRM_PAGE_SIZE) {
		const struct gpu_m4_drm_vm_leaf *p = NULL;
		u64 raw;

		for (u32 i = 0; i < layers && !p; i++) {
			const struct gpu_m4_drm_vm_leaf *q = gpu_m4_drm_uat_cursor_leaf(&at[i]);

			if (q && q->va == page) {
				p = q;
				gpu_m4_drm_uat_cursor_next(&at[i]);
			}
		}
		if (!p)
			return -ENOENT;
		gpu_m4_uat_stat[GPU_M4_DRM_UAT_STAT_PAGES]++;
		if ((p->access & access) != access)
			return -EACCES;
		if (!v->tables)
			return -EPROTO;
		if ((page >> 25) != block) {
			block = page >> 25;
			table = gpu_m4_drm_uat_block(v, page);
		}
		if (table < 0)
			return table;
		raw = get_unaligned_le64(v->table[table].cpu + gpu_m4_uat_fmt_slot(page, 2) * 8);
		if (raw != gpu_m4_uat_fmt_leaf_pte(p->pa, p->access))
			return -EPROTO;
		if (fn) {
			ret = fn(cookie, page, p);
			if (ret)
				return ret;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_range);

/* Only a constructed graph naming its exact checked manifest qualifies. */
static struct gpu_m4_drm_uat_layer *gpu_m4_drm_uat_span_layer(const struct gpu_m4_drm_uat *v)
{
	return v && v->cleaned && v->manifest && v->proof_manifest == v->manifest &&
		v->tables && refcount_read(&v->refs) ? v->manifest : NULL;
}

/* REMOVE-DEBUG: why key missed in top's chain (see the shadow). Memo lock. */
static void gpu_m4_drm_uat_span_missed(const struct gpu_m4_drm_uat_layer *top,
	const struct gpu_m4_drm_uat_span_key *key)
{
	u64 mix = gpu_m4_drm_uat_span_mix(key);
	u32 i = (u32)(mix >> 50) & (GPU_M4_UAT_SPAN_SHADOW - 1), why = GPU_M4_DRM_UAT_STAT_SPAN_MISS_ABSENT;
	const struct gpu_m4_drm_uat_layer *l;

	if (gpu_m4_uat_span_shadow[i].tag == (u32)mix) {
		why = GPU_M4_DRM_UAT_STAT_SPAN_MISS_ELSEWHERE;
		for (l = top; l; l = l->below)
			if (l->span_serial && gpu_m4_uat_span_shadow[i].block == (u32)l->span_serial)
				break;
		if (l && !gpu_m4_uat_span_shadow[i].evicted) {
			why = GPU_M4_DRM_UAT_STAT_SPAN_MISS_ABSENT;
		} else if (l) {
			/* Keys remembered since it was: the memo size that would
			 * have kept it under FIFO replacement.
			 */
			u32 distance = gpu_m4_uat_span_inserted - gpu_m4_uat_span_shadow[i].inserted;

			why = GPU_M4_DRM_UAT_STAT_SPAN_MISS_EVICTED;
			gpu_m4_uat_stat[distance < 512 ? GPU_M4_DRM_UAT_STAT_SPAN_REUSE_512 :
				distance < 1024 ? GPU_M4_DRM_UAT_STAT_SPAN_REUSE_1K :
				distance < 4096 ? GPU_M4_DRM_UAT_STAT_SPAN_REUSE_4K :
				GPU_M4_DRM_UAT_STAT_SPAN_REUSE_FAR]++;
		}
	}
	gpu_m4_uat_stat[why]++;
	if (key->kind != 3)
		return;
	/* The same named span proven for another snapshot generation. */
	for (l = top; l; l = l->below)
		for (u32 j = l->span_head; j; j = gpu_m4_uat_span[j - 1].next) {
			const struct gpu_m4_drm_uat_span_key *k = &gpu_m4_uat_span[j - 1].key;

			if (k->kind == 3 && k->identity == key->identity && k->aux == key->aux &&
			    k->va == key->va && k->size == key->size && k->offset == key->offset &&
			    k->access == key->access && k->aux_value != key->aux_value) {
				gpu_m4_uat_stat[GPU_M4_DRM_UAT_STAT_SPAN_MISS_STALE_NAMED]++;
				return;
			}
		}
}

/* REMOVE-DEBUG: the lookup site a key's kind and access identify. */
static u32 gpu_m4_drm_uat_span_site_of(const struct gpu_m4_drm_uat_span_key *key)
{
	switch (key->kind) {
	case 1:
		return GPU_M4_DRM_UAT_SITE_OBJECT;
	case 2:
		return GPU_M4_DRM_UAT_SITE_NATIVE;
	case 3:
		return GPU_M4_DRM_UAT_SITE_NAMED;
	case 4:
		return key->access == (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE) ?
			GPU_M4_DRM_UAT_SITE_BACKING : GPU_M4_DRM_UAT_SITE_RANGE;
	default:
		return GPU_M4_DRM_UAT_SITE_OTHER;
	}
}

/* Every block of the graph's chain: each entry holds for every graph naming
 * its block (see the block). O(depth) indexed probes.
 */
bool gpu_m4_drm_uat_span_cached_site(const struct gpu_m4_drm_uat *v,
	const struct gpu_m4_drm_uat_span_key *key, u32 site)
{
	struct gpu_m4_drm_uat_layer *top = gpu_m4_drm_uat_span_layer(v), *l;

	if (!top || !key)
		return false;
	if (site >= GPU_M4_DRM_UAT_SITES)
		site = GPU_M4_DRM_UAT_SITE_OTHER;
	mutex_lock(&gpu_m4_uat_span_lock);
	gpu_m4_uat_stat[GPU_M4_DRM_UAT_STAT_SPAN_DEPTH] += top->depth;
	for (l = top; l; l = l->below) {
		int at = gpu_m4_drm_uat_span_find(l, key);

		if (at < 0)
			continue;
		gpu_m4_uat_span[at].referenced = true;
		gpu_m4_uat_stat[GPU_M4_DRM_UAT_STAT_SPAN_HITS]++;
		gpu_m4_uat_stat[GPU_M4_DRM_UAT_STAT_SPAN_SITE_HITS + site]++;
		gpu_m4_uat_stat[GPU_M4_DRM_UAT_STAT_SPAN_BELOW_HITS] += l != top;
		gpu_m4_uat_stat[GPU_M4_DRM_UAT_STAT_SPAN_INHERITED_HITS] +=
			gpu_m4_uat_span[at].inherited;
		mutex_unlock(&gpu_m4_uat_span_lock);
		return true;
	}
	gpu_m4_uat_stat[GPU_M4_DRM_UAT_STAT_SPAN_MISSES]++;
	gpu_m4_uat_stat[GPU_M4_DRM_UAT_STAT_SPAN_SITE_MISSES + site]++;
	gpu_m4_drm_uat_span_missed(top, key);
	mutex_unlock(&gpu_m4_uat_span_lock);
	return false;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_span_cached_site);

bool gpu_m4_drm_uat_span_cached(const struct gpu_m4_drm_uat *v,
	const struct gpu_m4_drm_uat_span_key *key)
{
	return key && gpu_m4_drm_uat_span_cached_site(v, key, gpu_m4_drm_uat_span_site_of(key));
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_span_cached);

void gpu_m4_drm_uat_span_remember(const struct gpu_m4_drm_uat *v,
	const struct gpu_m4_drm_uat_span_key *key)
{
	struct gpu_m4_drm_uat_layer *top = gpu_m4_drm_uat_span_layer(v), *l;

	if (!top || !key)
		return;
	mutex_lock(&gpu_m4_uat_span_lock);
	for (l = top; l && gpu_m4_drm_uat_span_find(l, key) < 0; l = l->below)
		;
	if (!l)
		gpu_m4_drm_uat_span_insert(gpu_m4_drm_uat_span_home(top, key), key, false);
	mutex_unlock(&gpu_m4_uat_span_lock);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_span_remember);

void gpu_m4_drm_uat_stats(u64 out[GPU_M4_DRM_UAT_STATS])
{
	for (u32 i = 0; i < GPU_M4_DRM_UAT_STATS; i++)
		out[i] = gpu_m4_uat_stat[i] + atomic64_read(&gpu_m4_uat_build_stat[i]);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_stats);

int gpu_m4_drm_uat_get(struct gpu_m4_drm_uat *v)
{
	if (!v)
		return -EINVAL;
	refcount_inc(&v->refs);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_get);

u64 gpu_m4_drm_uat_generation(const struct gpu_m4_drm_uat *v)
{
	return v ? gpu_m4_drm_vm_owner_generation(v->snapshot) : 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_generation);

u64 gpu_m4_drm_uat_table_bytes(const struct gpu_m4_drm_uat *v)
{
	return v ? (u64)v->tables * GPU_M4_DRM_PAGE_SIZE : 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_table_bytes);

u64 gpu_m4_drm_uat_owned_bytes(const struct gpu_m4_drm_uat *v)
{
	return v ? (u64)v->owned * GPU_M4_DRM_PAGE_SIZE : 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_owned_bytes);

u64 gpu_m4_drm_uat_metadata_bytes(u64 leaves, u32 private_count, u32 max_tables)
{
	u64 tables = max_tables < GPU_M4_DRM_UAT_MAX_TABLES ? max_tables :
		GPU_M4_DRM_UAT_MAX_TABLES;
	/* At most one level-1 table per distinct VA >> 36 of some leaf. */
	u64 l1 = leaves < tables ? leaves : tables;

	/* One canonical order over a bounded chain: its leaves, per block at most
	 * count / CHUNK_MIN + 1 chunks (every chunk but the last holds MIN) and
	 * chunks / SEGMENT_MIN + 1 segments, each a header and a pointer.
	 */
	u64 chunks = leaves / GPU_M4_UAT_CHUNK_MIN + GPU_M4_UAT_MAX_DEPTH + 2;
	u64 order = leaves * sizeof(struct gpu_m4_drm_vm_leaf) +
		chunks * (sizeof(struct gpu_m4_drm_uat_chunk) + sizeof(void *)) +
		(chunks / GPU_M4_UAT_SEGMENT_MIN + GPU_M4_UAT_MAX_DEPTH + 2) *
		(sizeof(struct gpu_m4_drm_uat_segment) + sizeof(void *));

	if (l1 > GPU_M4_UAT_ROOT_ENTRIES)
		l1 = GPU_M4_UAT_ROOT_ENTRIES;
	/* Graph, bounded block chain (+ one transient flattened copy or rebase
	 * target) with its three orders and runs; construction transients: the
	 * fresh/resolved leaves plus one gathered copy, canonical views and
	 * change pointers, the (doubling) change list, one chunk merge buffer;
	 * private copy and keep/slot per mapping,
	 * inventory/node/PA-sort per table, slot indexes.
	 */
	return sizeof(struct gpu_m4_drm_uat) +
		(GPU_M4_UAT_MAX_DEPTH + 2) * sizeof(struct gpu_m4_drm_uat_layer) +
		2 * (GPU_M4_UAT_ORDERS * order +
		     (u64)private_count * sizeof(struct gpu_m4_drm_uat_run)) +
		leaves * (2 * sizeof(struct gpu_m4_drm_vm_leaf) + 4 * sizeof(void *) +
			  2 * sizeof(struct gpu_m4_drm_uat_change)) +
		(GPU_M4_UAT_CHUNK + GPU_M4_UAT_CHUNK_MIN) * sizeof(struct gpu_m4_drm_vm_leaf) +
		(u64)private_count * (sizeof(struct gpu_m4_drm_uat_private) + sizeof(bool) +
				      sizeof(struct gpu_m4_drm_uat_slot)) +
		tables * (sizeof(struct gpu_m4_drm_uat_table) +
			  sizeof(struct gpu_m4_drm_uat_node) + sizeof(u64)) +
		GPU_M4_UAT_ROOT_ENTRIES * sizeof(u16) + l1 * GPU_M4_UAT_ENTRIES * sizeof(u16);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_metadata_bytes);

/* Everything a constructed graph keeps reachable besides table pages: its
 * own arrays plus every (shared or owned) node and manifest block it names.
 */
u64 gpu_m4_drm_uat_metadata_used(const struct gpu_m4_drm_uat *v)
{
	const struct gpu_m4_drm_uat_layer *l;
	u64 bytes;

	if (!v)
		return 0;
	bytes = sizeof(*v) + (u64)v->private_count * sizeof(*v->private) +
		(u64)v->table_capacity * sizeof(*v->table) +
		(v->leaf ? (u64)(v->fresh ? v->fresh : 1) * sizeof(*v->leaf) : 0) +
		(v->keep ? (u64)v->private_count * sizeof(*v->keep) : 0);
	for (u32 i = 0; i < v->tables; i++)
		bytes += sizeof(struct gpu_m4_drm_uat_node) + (v->table[i].level == 2 ? 0 :
			(u64)(v->table[i].level ? GPU_M4_UAT_ENTRIES : GPU_M4_UAT_ROOT_ENTRIES) *
			sizeof(u16));
	/* Shared segments and chunks count in full for every graph naming
	 * them, as nodes do. O(segments): a chunk count is its segment's.
	 */
	for (l = v->manifest; l; l = l->below) {
		bytes += sizeof(*l) + (u64)l->runs * sizeof(*l->run) +
			GPU_M4_UAT_ORDERS * (u64)l->count * sizeof(struct gpu_m4_drm_vm_leaf);
		for (u32 k = 0; k < GPU_M4_UAT_ORDERS; k++)
			for (u32 g = 0; g < l->seq[k].segments; g++)
				bytes += sizeof(struct gpu_m4_drm_uat_segment) + sizeof(void *) +
					(u64)l->seq[k].segment[g]->count *
					(sizeof(struct gpu_m4_drm_uat_chunk) + sizeof(void *));
	}
	return bytes;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_metadata_used);

int gpu_m4_drm_uat_roots(const struct gpu_m4_drm_uat *v, u64 *low, u64 *high)
{
	int ret;

	if (!low || !high)
		return -EINVAL;
	/* The GRAPH version and its root allocation are immutable after the
	 * build-time verification; do not walk every leaf again for its roots.
	 */
	ret = gpu_m4_drm_uat_checked(v);
	if (ret)
		return ret;
	*low = v->checked_low;
	*high = 0;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_roots);

int gpu_m4_drm_uat_begin_residency(struct gpu_m4_drm_uat *v)
{
	if (!v || v->resident || v->uncertain)
		return -EINVAL;
	/* Consume the construction proof of every named node (no leaf walk)
	 * and the exact root image the caller's refresh just made current.
	 */
	if (gpu_m4_drm_uat_proven(v) || gpu_m4_drm_uat_root_image(v))
		return -EPROTO;
	v->published = true;
	v->uncertain = true;
	v->resident = true;
	v->retired = false;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_begin_residency);

int gpu_m4_drm_uat_confirm_resident(struct gpu_m4_drm_uat *v)
{
	if (!v || !v->resident || !v->uncertain)
		return -EINVAL;
	v->uncertain = false;
	v->retired = true;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_confirm_resident);

int gpu_m4_drm_uat_begin_work(struct gpu_m4_drm_uat *v)
{
	if (!v || !v->resident || v->uncertain || !v->retired)
		return -EBUSY;
	v->retired = false;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_begin_work);

int gpu_m4_drm_uat_retire(struct gpu_m4_drm_uat *v)
{
	if (!v || !v->published || !v->resident || v->uncertain || v->retired)
		return -EINVAL;
	v->retired = true;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_retire);

int gpu_m4_drm_uat_mark_not_resident(struct gpu_m4_drm_uat *v)
{
	if (!v || !v->resident || !v->retired || v->uncertain)
		return -EBUSY;
	v->resident = false;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_mark_not_resident);


/*
 * Drop every table page, node reference and manifest block of a FAILED,
 * never-published construction, callback-free (safe under the provider lock).
 * Only the snapshot and private owner holds remain, for the caller's
 * outside-lock release; metadata_used() then reports exactly what remains.
 */
int gpu_m4_drm_uat_strip(struct gpu_m4_drm_uat *v)
{
	if (!v || v->cleaned || v->published || v->resident || v->uncertain || v->image ||
	    refcount_read(&v->refs) != 1)
		return -EBUSY;
	for (u32 i = 0; i < v->tables; i++) {
		gpu_m4_drm_uat_node_put(v->table[i].node);
		gpu_m4_drm_uat_node_put(v->table[i].replaces);
	}
	kvfree(v->table);
	v->table = NULL;
	v->tables = v->table_capacity = v->owned = 0;
	gpu_m4_drm_uat_layer_put(v->manifest);
	gpu_m4_drm_uat_layer_put(v->rebase);
	v->manifest = v->rebase = NULL;
	v->proof_manifest = NULL;
	kvfree(v->leaf);
	v->leaf = NULL;
	kvfree(v->keep);
	v->keep = NULL;
	v->fresh = v->count = 0;
	v->prepared = false;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_strip);

int gpu_m4_drm_uat_release(struct gpu_m4_drm_uat *v)
{
	int ret;

	if (!v)
		return 0;
	if (refcount_read(&v->refs) > 1) {
		refcount_dec(&v->refs);
		return 0;
	}
	if (v->resident || v->uncertain || (v->published && !v->retired))
		return -EBUSY;
	/* A job-published source snapshot remains independently protected. */
	if (refcount_read(&v->refs) == 1 && v->snapshot) {
		ret = gpu_m4_drm_vm_owner_release(v->snapshot);
		if (ret)
			return ret;
		v->snapshot = NULL;
	}
	if (!refcount_dec_and_test(&v->refs))
		return 0;
	for (u32 i = 0; i < v->held; i++)
		v->private[i].put(v->private[i].cookie);
	/* Only pages no other graph still names are freed. */
	for (u32 i = 0; i < v->tables; i++) {
		gpu_m4_drm_uat_node_put(v->table[i].node);
		gpu_m4_drm_uat_node_put(v->table[i].replaces);
	}
	gpu_m4_drm_uat_layer_put(v->manifest);
	gpu_m4_drm_uat_layer_put(v->rebase);
	kvfree(v->private);
	kvfree(v->leaf);
	kvfree(v->keep);
	kvfree(v->table);
	kfree(v);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_uat_release);

MODULE_DESCRIPTION("Retained detached J713 client lower-UAT versions");
MODULE_LICENSE("GPL");
