/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_DRM_UAT_H
#define APPLE_GPU_M4_DRM_UAT_H

#include "gpu-m4-drm-vm-owner.h"

#define GPU_M4_DRM_UAT_CONTEXT_START 0x1000000000ULL
#define GPU_M4_DRM_UAT_CONTEXT_SIZE 0x10000ULL
#define GPU_M4_DRM_UAT_PRIVATE_START 0x7000000000ULL
#define GPU_M4_DRM_UAT_PRIVATE_END 0x8000000000ULL
#define GPU_M4_DRM_UAT_MAX_TABLES 4096U
#define GPU_M4_DRM_UAT_MAX_LEAVES (1U << 20)

struct gpu_m4_drm_uat;

/* Provider-owned backing, NEVER a raw userspace PA array. Get proves the
 * backing's stable identity and ownership by the snapshot's retained VM. Page
 * resolves an exact pinned byte offset and rejects live firmware/MMIO/foreign
 * backing. The builder additionally checks RAM, alignment, aliases and tables.
 * Callbacks/cookie remain valid through put; the provider is retained by get.
 * Backing identity/offset/PA cannot change while any version holds that owner.
 *
 * The one fixed context mapping MUST name the provider VM's already allocated,
 * zeroed, persistent 64 KiB backing, offset zero and RW. It is acquired once per
 * VM OUTSIDE this builder; each version holds a reference to that same owner.
 * Other mappings lease actual bytes inside the private allocator reservation.
 * The array is copied and each backing held: no borrowed PA-array lifetime.
 */
struct gpu_m4_drm_uat_private {
	const void *identity;
	void *cookie;
	int (*get)(void *cookie, struct gpu_m4_drm_vm *vm, const void *identity);
	void (*put)(void *cookie);
	int (*page)(void *cookie, u64 offset, u64 *pa);
	u64 va, offset, size;
	u32 access;
	enum gpu_m4_drm_cpu_policy cpu_policy;
};

/* Build under the provider VM lock from an immutable pinned snapshot and the
 * complete retained private inventory. Snapshot is not job-published/retired;
 * later per-job clones own publication separately. Reject ALL user collisions with
 * either low reservation before constructing any table; failure changes no
 * active generation or existing mapping. Limits are real finite resources.
 * Caller also charges table_bytes across ALL current/queued/resident versions
 * to its aggregate memory ceiling. No MMIO, TLBI, ASID binding or publication.
 */
struct gpu_m4_drm_uat *gpu_m4_drm_uat_build(
	struct gpu_m4_drm_vm_owner *snapshot,
	const struct gpu_m4_drm_uat_private *private, u32 private_count,
	u32 max_table_pages, u32 max_leaf_pages);
/* For callbacks whose put must run outside the provider lock. On failure,
 * garbage receives any owned unpublished partial graph; keep its table_bytes
 * charged until releasing it outside all provider/reservation locks. On success
 * garbage is NULL. No acquired private owner is released by this builder.
 */
struct gpu_m4_drm_uat *gpu_m4_drm_uat_build_deferred(
	struct gpu_m4_drm_vm_owner *snapshot,
	const struct gpu_m4_drm_uat_private *private, u32 private_count,
	u32 max_table_pages, u32 max_leaf_pages, struct gpu_m4_drm_uat **garbage);

/*
 * Copy-on-write successor of an already constructed immutable graph. The new
 * manifest is EXACTLY base's (same VM, user generation and user leaves, i.e.
 * snapshot shares base's user map) plus private[base->private_count..]: base's
 * private inventory must be an identical prefix. Every unchanged branch and
 * expected-manifest block is shared under strong references; only the root
 * and the L1/L2 pages on the appended mappings' paths are newly allocated,
 * filled, cleaned and proven. base and its pages are never written. Every
 * private owner is held again (base's holds stay base's); only new mappings
 * resolve pages. -ESTALE before any acquisition when base is not derivable.
 * Failure semantics as build_deferred(); garbage owns only its fresh pages
 * (gpu_m4_drm_uat_owned_bytes()) plus references.
 */
struct gpu_m4_drm_uat *gpu_m4_drm_uat_derive_deferred(struct gpu_m4_drm_uat *base,
	struct gpu_m4_drm_vm_owner *snapshot,
	const struct gpu_m4_drm_uat_private *private, u32 private_count,
	u32 max_table_pages, u32 max_leaf_pages, struct gpu_m4_drm_uat **garbage);
/*
 * Two-phase build (base may be NULL: full build). Prepare runs under the
 * provider lock: shape/derivation checks, snapshot and private holds, page
 * resolution and RAM admission of the fresh leaves only. Construct needs NO
 * provider lock and calls no callback: alias proof, table allocation, writes,
 * cleaning and the construction proof use only held immutable inputs. A
 * prepared graph is constructed at most once; on any failure it is garbage
 * the caller releases outside every provider/reservation lock.
 */
struct gpu_m4_drm_uat *gpu_m4_drm_uat_prepare_deferred(struct gpu_m4_drm_uat *base,
	struct gpu_m4_drm_vm_owner *snapshot,
	const struct gpu_m4_drm_uat_private *private, u32 private_count,
	u32 max_table_pages, u32 max_leaf_pages, struct gpu_m4_drm_uat **garbage);
int gpu_m4_drm_uat_construct(struct gpu_m4_drm_uat *version);

/* Borrowed proven user view of the TARGET's immutable ledger: leaves plus its
 * VA position index (a permutation, checked linearly when it is consulted).
 * The orders of the target block come from the base block's immutable chunks
 * plus the sorted delta, never from a re-sort. It must stay valid until
 * construction ends (caller holds the target version).
 */
struct gpu_m4_drm_uat_user {
	const struct gpu_m4_drm_vm_leaf *leaf;
	const u32 *va;
	u32 count;
};
/*
 * Cross-generation rebase onto a held construction base (a constructed graph
 * or cache image of the SAME VM with a single manifest block; generation,
 * counts and private inventory may differ). The result is exactly the target:
 * user view plus every private mapping, held again now; never old-union-new
 * and never the base's authority. Each private mapping is resolved again,
 * except where kept[i] (kept may be NULL) carries the caller's proof, like
 * share_prepare's membership key, that private[i] names the same backing,
 * held continuously since base's construction resolved it: when private[i]'s
 * exact shape is a run (resolved mapping) of the base block, its leaves are
 * then the base block's own, never visited; else it is resolved. The user
 * leaves are the base's when the snapshot names the base's user generation
 * (and count); only otherwise is the view proven and merged with the base.
 * Construction (no lock, no callback) computes the exact old->target leaf
 * delta, builds each order of the target block from the base block's
 * immutable chunks with only the touched chunks copied, proves the alias
 * contract for the new leaves against it (the base proved the rest), copies
 * only changed L2 pages, their L1 parents and the root (old words must match
 * the base's expected descriptors), keeps emptied branches named, and shares
 * the root and manifest block too when nothing changed. -ESTALE before any
 * acquisition when base is unusable. Failure semantics as build_deferred():
 * garbage owns only fresh pages + refs.
 */
struct gpu_m4_drm_uat *gpu_m4_drm_uat_rebase_prepare(struct gpu_m4_drm_uat *base,
	struct gpu_m4_drm_vm_owner *snapshot, const struct gpu_m4_drm_uat_user *user,
	const struct gpu_m4_drm_uat_private *private, const bool *kept, u32 private_count,
	u32 max_table_pages, u32 max_leaf_pages, struct gpu_m4_drm_uat **garbage);
struct gpu_m4_drm_uat *gpu_m4_drm_uat_rebase_deferred(struct gpu_m4_drm_uat *base,
	struct gpu_m4_drm_vm_owner *snapshot, const struct gpu_m4_drm_uat_user *user,
	const struct gpu_m4_drm_uat_private *private, const bool *kept, u32 private_count,
	u32 max_table_pages, u32 max_leaf_pages, struct gpu_m4_drm_uat **garbage);
bool gpu_m4_drm_uat_rebase_ready(const struct gpu_m4_drm_uat *version);
u32 gpu_m4_drm_uat_leaves(const struct gpu_m4_drm_uat *version);
bool gpu_m4_drm_uat_compactable(const struct gpu_m4_drm_uat *version);
/* Construction-cache image: proven nodes + manifest block under strong refs,
 * no snapshot/owner holds/residency; never bindable. Callback-free release.
 */
struct gpu_m4_drm_uat *gpu_m4_drm_uat_image(const struct gpu_m4_drm_uat *version);
/* Use wrapper of an exact constructed image (graph or cache image): same VM,
 * same user generation/count, identical private descriptor inventory. Shares
 * every node, the root and the manifest block; holds its own snapshot and
 * private owners (no page resolution); own residency/work state. construct()
 * only records it. -ESTALE before any acquisition when the key differs.
 */
struct gpu_m4_drm_uat *gpu_m4_drm_uat_share_prepare(struct gpu_m4_drm_uat *base,
	struct gpu_m4_drm_vm_owner *snapshot, const struct gpu_m4_drm_uat_private *private,
	u32 private_count, u32 max_table_pages, struct gpu_m4_drm_uat **garbage);
/* Two separate constructed graphs naming the exact same immutable physical
 * image and complete leaf/private-owner tuples (see uat.c). Metadata only.
 */
bool gpu_m4_drm_uat_same_image(const struct gpu_m4_drm_uat *first,
			       const struct gpu_m4_drm_uat *second);

int gpu_m4_drm_uat_get(struct gpu_m4_drm_uat *version);
u64 gpu_m4_drm_uat_generation(const struct gpu_m4_drm_uat *version);
/* Every table page this graph names (shared ones included): the conservative
 * per-version charge of a constructed graph.
 */
u64 gpu_m4_drm_uat_table_bytes(const struct gpu_m4_drm_uat *version);
/* Pages this graph allocated itself: exactly what its failed construction
 * holds beyond shared references.
 */
u64 gpu_m4_drm_uat_owned_bytes(const struct gpu_m4_drm_uat *version);
/* Conservative bound of one graph's metadata, including construction-time
 * transients, to charge BEFORE building; used() is a constructed graph's
 * actual reachable metadata (shared nodes/blocks included), never above it.
 */
u64 gpu_m4_drm_uat_metadata_bytes(u64 leaves, u32 private_count, u32 max_tables);
u64 gpu_m4_drm_uat_metadata_used(const struct gpu_m4_drm_uat *version);
/* Exact ASID-free valid low root descriptor (root PA | 1); the client high
 * descriptor is always zero. The TTBAT word of user context s adds s << 48.
 * Checked getter: requires the built/cleaned/verified state, retained root
 * allocation and its recorded descriptor. It does NOT walk the leaves again
 * (the graph is immutable after build); use the verifiers for that.
 */
int gpu_m4_drm_uat_roots(const struct gpu_m4_drm_uat *version, u64 *low, u64 *high);
/* Constructed identity of an immutable graph: built, proven once while detached
 * and never written since; retained root allocation and recorded descriptor.
 * Taking another reference needs nothing more. NOT a live-table check: bind
 * transitions use refresh_check (construction proof + live root page) and
 * fault/diagnostic boundaries use refresh_verify/verify.
 */
int gpu_m4_drm_uat_check(const struct gpu_m4_drm_uat *version);
/* Complete owned walk: every word of every (shared or fresh) table page. */
int gpu_m4_drm_uat_verify(const struct gpu_m4_drm_uat *version);
/* REMOVE-DEBUG (shadow VM tree oracle, until S3d): call fn for every valid
 * leaf word of a constructed graph (each level-2 table of its complete
 * inventory, shared nodes included), raw. Read only; the caller holds the
 * version. Stops at, and returns, fn's first nonzero result.
 */
int gpu_m4_drm_uat_leaf_words(const struct gpu_m4_drm_uat *version,
	int (*fn)(void *cookie, u64 va, u64 word), void *cookie);
/* Caller holds the version's owner lock. Refresh ONLY already-clean immutable
 * owned WB tables, then check the original manifest, including expected zeros.
 * No dirty candidate edits, parent/leaf writes or arbitrary PA following.
 */
int gpu_m4_drm_uat_refresh_verify(const struct gpu_m4_drm_uat *version);
/* Targeted live check consuming the construction proof: checked state, every
 * private owner held, the proven manifest, every named (shared or fresh) node's
 * retained record/parent slot and its creator's one-time proof, and the
 * refreshed live root page naming exactly its owned level-1 tables. Reads no
 * L1/L2 page: an immutable node is never rescanned merely because a binding
 * changed. Not a full manifest scan; verify/refresh_verify remain diagnostics.
 */
int gpu_m4_drm_uat_refresh_check(const struct gpu_m4_drm_uat *version);
/* Pointer-free private diagnostic copies, not a userspace ABI. Expected roots
 * describe the owned graph, NEVER hardware readback or publication authority.
 * Table indices are inventory ordinals; root has parent_index == U32_MAX.
 */
struct gpu_m4_drm_uat_evidence {
	u64 generation, expected_low, expected_high;
	u32 table_count;
	int verify_error;
};

struct gpu_m4_drm_uat_table_evidence {
	u64 va_base, pa;
	u32 index, parent_index, parent_entry, level;
	u8 bytes[GPU_M4_DRM_PAGE_SIZE];
};

/* Read-only destination check for a containing provider diagnostic header. */
int gpu_m4_drm_uat_evidence_destination(const struct gpu_m4_drm_uat *version,
	const void *storage, unsigned long bytes);
/* Caller holds the owner lock and retains this exact version. Storage is
 * caller-owned/preallocated and must not alias the owner or its tables. No
 * allocation, arbitrary PA walk, table write or lifetime/state transition.
 * Only a complete already-clean owned graph may be refreshed/copied. A safe
 * diagnostic copy returns 0 even if verify_error reports manifest corruption.
 * -ENOSPC sets required table_count/expected roots and verify_error=-ENODATA,
 * without refreshing or touching records. Other errors leave outputs alone.
 */
int gpu_m4_drm_uat_export(const struct gpu_m4_drm_uat *version,
	struct gpu_m4_drm_uat_evidence *header,
	struct gpu_m4_drm_uat_table_evidence *tables, u32 capacity);
/* Read-only walk verifies every reached parent belongs to this version. */
int gpu_m4_drm_uat_page(const struct gpu_m4_drm_uat *version, u64 va,
			 u32 access, struct gpu_m4_drm_vm_leaf *leaf);
/* Every page of [va, va + size), each with exactly the gpu_m4_drm_uat_page()
 * checks and result, in O(manifest blocks) indexed lookups for the whole run
 * plus sequential leaf comparison; parents checked once per 32 MiB block.
 */
typedef int (*gpu_m4_drm_uat_leaf_fn)(void *cookie, u64 va,
				     const struct gpu_m4_drm_vm_leaf *leaf);
int gpu_m4_drm_uat_range(const struct gpu_m4_drm_uat *version, u64 va, u64 size,
			 u32 access, gpu_m4_drm_uat_leaf_fn fn, void *cookie);
/* Cross-graph span admission cache of immutable manifest blocks (and so
 * shared by every constructed graph/use wrapper naming them, forgotten with
 * them), in one fixed-size memo shared by every block of every VM.
 * A key records caller span facts the caller proved completely against such
 * a graph: translation/backing only, never phase, cache or lane authority.
 * The fact must be a function of the key and of the installed leaves of every
 * page of [va, va + size) alone (each walked by the proof), so it holds for
 * any graph whose manifest has exactly those leaves there: it is kept on the
 * topmost block holding one of them (shared by every chain naming that
 * block), and a graph built from a base by an exact leaf delta starts with
 * the base's keys whose pages the delta does not touch (see uat.c).
 * Callers hold the provider lock. Remembering is best effort.
 */
struct gpu_m4_drm_uat_span_key {
	const void *identity, *aux;
	u64 va, size, offset, aux_value;
	u32 access, kind;
};
bool gpu_m4_drm_uat_span_cached(const struct gpu_m4_drm_uat *version,
				const struct gpu_m4_drm_uat_span_key *key);
void gpu_m4_drm_uat_span_remember(const struct gpu_m4_drm_uat *version,
				  const struct gpu_m4_drm_uat_span_key *key);
/* REMOVE-DEBUG: lookup attribution. span_cached() derives the site from the
 * key (kind, access); a caller probing for a stronger key names its own.
 */
enum gpu_m4_drm_uat_span_site {
	GPU_M4_DRM_UAT_SITE_OBJECT,	/* kind 1: private GEM backing object span */
	GPU_M4_DRM_UAT_SITE_NATIVE,	/* kind 2: exact native BO object span */
	GPU_M4_DRM_UAT_SITE_STRICT,	/* kind 2 probe answering a kind 1 query */
	GPU_M4_DRM_UAT_SITE_NAMED,	/* kind 3: decoder named span (generation) */
	GPU_M4_DRM_UAT_SITE_BACKING,	/* kind 4 RW: render private backing run */
	GPU_M4_DRM_UAT_SITE_RANGE,	/* kind 4 R/W: attachment, metadata, decoder */
	GPU_M4_DRM_UAT_SITE_OTHER,
	GPU_M4_DRM_UAT_SITES,
};
bool gpu_m4_drm_uat_span_cached_site(const struct gpu_m4_drm_uat *version,
				     const struct gpu_m4_drm_uat_span_key *key, u32 site);
/* REMOVE-DEBUG: cumulative translation walk statistics. */
enum {
	GPU_M4_DRM_UAT_STAT_PAGE_LOOKUPS,	/* gpu_m4_drm_uat_page() layered searches */
	GPU_M4_DRM_UAT_STAT_RUNS,		/* gpu_m4_drm_uat_range() runs */
	GPU_M4_DRM_UAT_STAT_LOOKUPS,		/* indexed block lookups by runs */
	GPU_M4_DRM_UAT_STAT_PAGES,		/* pages compared sequentially by runs */
	GPU_M4_DRM_UAT_STAT_SPAN_HITS,
	GPU_M4_DRM_UAT_STAT_SPAN_MISSES,
	/* Graph construction (atomic: construct runs without the provider lock). */
	GPU_M4_DRM_UAT_STAT_CONSTRUCTS,		/* successful construct() calls, of which: */
	GPU_M4_DRM_UAT_STAT_FULLS,		/* complete builds */
	GPU_M4_DRM_UAT_STAT_APPENDS,		/* same-generation private appends */
	GPU_M4_DRM_UAT_STAT_SHARES,		/* use wrappers of an exact image */
	GPU_M4_DRM_UAT_STAT_REBASES,		/* cross-generation rebases */
	GPU_M4_DRM_UAT_STAT_REBASE_NS,		/* wall time of those rebases */
	GPU_M4_DRM_UAT_STAT_SORTED,		/* manifest leaf entries comparison-sorted */
	GPU_M4_DRM_UAT_STAT_MERGED,		/* manifest entries placed by chunk merges */
	GPU_M4_DRM_UAT_STAT_TABLES_BUILT,	/* fresh (allocated, filled, proven) table pages */
	GPU_M4_DRM_UAT_STAT_TABLES_SHARED,	/* immutable nodes named without a copy */
	GPU_M4_DRM_UAT_STAT_PRIVATE_KEPT,	/* rebase private leaves kept as base runs */
	GPU_M4_DRM_UAT_STAT_PRIVATE_RESOLVED,	/* rebase private leaves resolved by callback */
	GPU_M4_DRM_UAT_STAT_CHUNKS_BUILT,	/* manifest order chunks allocated and filled */
	GPU_M4_DRM_UAT_STAT_CHUNKS_SHARED,	/* base chunks a rebase named without a copy */
	GPU_M4_DRM_UAT_STAT_REBASE_DELTA,	/* old->target leaf changes found by rebases */
	GPU_M4_DRM_UAT_STAT_REBASE_TOUCHED,	/* manifest entries rebases visited or wrote */
	GPU_M4_DRM_UAT_STAT_USER_DIFFED,	/* of those, user-diff steps (new generation) */
	/* Span memo inheritance and placement. */
	GPU_M4_DRM_UAT_STAT_SPAN_INHERITED,	/* base keys a new block started with */
	GPU_M4_DRM_UAT_STAT_SPAN_DROPPED,	/* base keys whose pages the delta touched */
	GPU_M4_DRM_UAT_STAT_SPAN_INHERITED_HITS,	/* hits answered by an inherited key */
	GPU_M4_DRM_UAT_STAT_SPAN_BELOW_HITS,	/* hits answered by a block below the top */
	/* Span memo churn and miss attribution. Miss reasons come from a small
	 * global shadow of recently remembered keys (approximate: a key unused
	 * for long may have lost its shadow slot and read as absent).
	 */
	GPU_M4_DRM_UAT_STAT_SPAN_INSERTS,	/* keys remembered (proven fresh) */
	GPU_M4_DRM_UAT_STAT_SPAN_EVICTIONS,	/* keys replaced in the full memo (CLOCK) */
	GPU_M4_DRM_UAT_STAT_SPAN_MISS_ABSENT,	/* never remembered (or shadow lost it) */
	GPU_M4_DRM_UAT_STAT_SPAN_MISS_EVICTED,	/* remembered in this chain, then evicted */
	/* Of those, by keys remembered since (the FIFO memo size that would
	 * have kept them): < 512, < 1024, < 4096, more.
	 */
	GPU_M4_DRM_UAT_STAT_SPAN_REUSE_512,
	GPU_M4_DRM_UAT_STAT_SPAN_REUSE_1K,
	GPU_M4_DRM_UAT_STAT_SPAN_REUSE_4K,
	GPU_M4_DRM_UAT_STAT_SPAN_REUSE_FAR,
	GPU_M4_DRM_UAT_STAT_SPAN_MISS_ELSEWHERE,	/* remembered on a block not in this chain */
	GPU_M4_DRM_UAT_STAT_SPAN_MISS_STALE_NAMED,	/* NAMED key present for another generation */
	GPU_M4_DRM_UAT_STAT_SPAN_DEPTH,		/* sum of chain depths over lookups */
	/* Hits, then misses, per enum gpu_m4_drm_uat_span_site. */
	GPU_M4_DRM_UAT_STAT_SPAN_SITE_HITS,
	GPU_M4_DRM_UAT_STAT_SPAN_SITE_MISSES =
		GPU_M4_DRM_UAT_STAT_SPAN_SITE_HITS + GPU_M4_DRM_UAT_SITES,
	GPU_M4_DRM_UAT_STAT_SPAN_SITE_END =
		GPU_M4_DRM_UAT_STAT_SPAN_SITE_MISSES + GPU_M4_DRM_UAT_SITES,
	GPU_M4_DRM_UAT_STATS = GPU_M4_DRM_UAT_STAT_SPAN_SITE_END,
};
void gpu_m4_drm_uat_stats(u64 out[GPU_M4_DRM_UAT_STATS]);

/* All transitions/ref drops are serialized by the provider owner lock. Before
 * ANY possible TTBAT write, begin_residency pins uncertainty. It consumes the
 * construction proof (as refresh_check) and the root image the caller's
 * refresh just made current; it never walks leaves again. Confirm ONLY after
 * exact root readback and qualified handoff/TLBI; errors leave uncertainty set.
 * Retire means ALL work/cache readers are certainly retired, not an error fence.
 * A retired version still cannot be released while TTBAT names it. Mark absent
 * ONLY after qualified replacement/unbinding, invalidation AND retirement.
 *
 * These states do NOT publish/retire the snapshot: individual jobs clone their
 * own cache/resv snapshots. An immutable version may serve many queued jobs.
 * Nonfinal drops are allowed under that lock even while protected: a remaining
 * provider/residency reference pins the whole version. The FINAL protected drop
 * returns -EBUSY without dropping the last reference. An error fence/ref drop is
 * never authority to clear residency/uncertainty or reclaim backing.
 */
int gpu_m4_drm_uat_begin_residency(struct gpu_m4_drm_uat *version);
int gpu_m4_drm_uat_confirm_resident(struct gpu_m4_drm_uat *version);
/* Before each serialized job's first work producer, including repeated work on
 * an unchanged resident root. Confirmed root-only binding has no active work.
 */
int gpu_m4_drm_uat_begin_work(struct gpu_m4_drm_uat *version);
int gpu_m4_drm_uat_retire(struct gpu_m4_drm_uat *version);
int gpu_m4_drm_uat_mark_not_resident(struct gpu_m4_drm_uat *version);
int gpu_m4_drm_uat_release(struct gpu_m4_drm_uat *version);
/* Failed, never-published construction only: drop all table pages, node refs
 * and manifest blocks without any callback (allowed under the provider lock),
 * keeping snapshot/private holds for the outside-lock release. -EBUSY if not
 * such a sole-owned failed graph.
 */
int gpu_m4_drm_uat_strip(struct gpu_m4_drm_uat *version);

#endif
