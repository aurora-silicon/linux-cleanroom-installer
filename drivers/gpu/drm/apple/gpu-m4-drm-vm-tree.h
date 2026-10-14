/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_DRM_VM_TREE_H
#define APPLE_GPU_M4_DRM_VM_TREE_H

#include <linux/types.h>

#include "gpu-m4-drm-uat-format.h"

/*
 * Persistent per-VM lower page-table tree: one stable root and its level-1/2
 * tables for the lifetime of a VM, edited in place by VM_BIND, plus the
 * extent interval tree (one node per mapping) whose object references hold
 * the mapped pins. Same page format as gpu-m4-drm-uat.c.
 *
 * S3a/S3b: a shadow, built and kept but never installed in any TTBAT. Besides
 * the VM_BIND user mappings it mirrors the driver mappings of the VM (DRIVER
 * extents): the 64 KiB context, the UMA/parameter globals and the client-VA
 * BOs of native units, while the version path's common private baseline and
 * native baseline stay the authority.
 *
 * S3c (config.authority, gated by the provider's vm_tree_authority): the
 * tree is the VM's hardware authority. Its root is what TTBAT[asid] names
 * while the tree is resident (install_begin .. evicted); removals then
 * invalidate the VM's ASID with real TLBI instructions. Accepted jobs hold
 * O(1) guards: GEM and VM close only mark the affected user extents CLOSED
 * (still mapped and pinned) while a guard exists, and the last guard drop
 * asks the owner (config.deferred) to remove them. Explicit UNBIND and
 * replacement are always immediate.
 *
 * Invariants (authority):
 * I1 every valid leaf reachable from the root belongs to a live extent that
 *    holds its object reference (pin) or its driver backing;
 * I2 an extent's reference is dropped only after clear -> PoC clean -> DSB
 *    OSH -> TLBI on the VM's ASID (when resident) -> DSB SY; a tree whose
 *    edits found an inconsistency is faulted and keeps every reference;
 * I3 tables are freed only by destroy, which requires no guard and a tree
 *    that is no longer resident (its TTBAT entry replaced or cleared and the
 *    ASID invalidated by the binder);
 * I4 user operations never reach a DRIVER extent.
 *
 * Locks: tree->lock (batches, lookups, close) -> binding_lock (residency,
 * ASID, the TLBI decision and instructions). The binder takes binding_lock
 * under the GPU mutex; nothing under tree->lock takes the GPU mutex.
 *
 * Rules (each serialized by the tree mutex):
 * - A batch is checked and every missing table, extent, mapping record and
 *   object reference it can need is acquired first. Any failure there
 *   returns with the tree, its pages and every reference unchanged.
 * - New tables are zero, cleaned to PoC and linked (parent word written and
 *   cleaned) before any leaf names them, then DSB OSH. Tables stay until
 *   destroy: removals never free or unlink one.
 * - Operations apply in order. A removal or replacement clears the affected
 *   leaves, cleans the changed spans to PoC, DSB OSH, invalidates the removed
 *   range on the VM's ASID (VAE1OS for one page, RVAE1OS for more), DSB SY,
 *   and only then drops the reference of an object left with no extent.
 * - An insertion writes only zero leaves, cleans the changed spans and issues
 *   DSB OSH, without any TLBI.
 * - ASIDE1OS is used only when the whole tree is torn down (destroy).
 * - A user operation never touches a DRIVER extent and a DRIVER operation
 *   never touches a user extent (-EBUSY, nothing changed); GEM close
 *   (remove_object) skips DRIVER extents. A batch is all user or all DRIVER.
 * - Every new table page passes the RAM admission the immutable builder
 *   makes for its tables (System RAM, valid unreserved PFNs, PA mask) and
 *   must be zero; otherwise the batch is refused unchanged (-EPERM/-EPROTO).
 */

enum {
	GPU_M4_VM_TREE_UNBIND = 1U << 0,	/* remove [va, va + size) */
	GPU_M4_VM_TREE_SINGLE_PAGE = 1U << 1,	/* every page maps the first object page */
	GPU_M4_VM_TREE_DRIVER = 1U << 2,	/* driver mapping: user operations cannot touch it */
	GPU_M4_VM_TREE_CLOSED = 1U << 3,	/* internal: GEM/VM closed, removal waits for guards */
};

/* Hardware maintenance; host tests record it. ASIDs are the VM's user ASID. */
struct gpu_m4_vm_tree_ops {
	void (*clean)(void *cookie, void *start, size_t bytes);	/* to PoC */
	void (*dsb_osh)(void *cookie);
	void (*dsb_sy)(void *cookie);
	void (*tlbi_va)(void *cookie, u64 operand);	/* VAE1OS */
	void (*tlbi_range)(void *cookie, u64 operand);	/* RVAE1OS */
	void (*tlbi_asid)(void *cookie, u64 operand);	/* ASIDE1OS */
};

struct gpu_m4_vm_tree_config {
	const struct gpu_m4_vm_tree_ops *ops;
	void *cookie;
	/* Object references (pins): get may fail; put runs only after the
	 * removal sequence above, under the same caller locks as the edit.
	 */
	int (*get)(void *cookie, void *object);
	void (*put)(void *cookie, void *object);
	/* User VA [low, high); user ops must avoid [kernel_start, kernel_end)
	 * and the reserved ranges (start, end; empty when start == end).
	 */
	u64 low, high, kernel_start, kernel_end;
	u64 reserved[2][2];
	u32 max_tables;	/* root included */
	/* S3c: the tree is the VM's hardware authority. Removals TLBI only while
	 * resident; GEM/VM close of guarded user extents is deferred; deferred
	 * (any context, no lock held) asks the owner to call settle_deferred().
	 */
	bool authority;
	void (*deferred)(void *cookie);
};

/*
 * One VM_BIND operation. pa[] is the pinned object's page inventory
 * (object_size bytes), valid for the call; a held object keeps its PA
 * inventory stable while the tree maps it. A DRIVER op may have no object.
 */
struct gpu_m4_vm_tree_op {
	u64 va, size, offset, object_size;
	void *object;
	const u64 *pa;
	u32 access, flags;
	/* Leaf lookups (S3c): the backing's stable identity (GEM or driver
	 * owner) and CPU policy, reported by page/walk/covers.
	 */
	const void *identity;
	u32 cpu_policy;
	/* DRIVER only: explicit leaf attribute bits (no PA bits, valid bit
	 * set) instead of the access-derived user leaf; 0 keeps that leaf.
	 */
	u64 attrs;
};

/* Shadow hardware callbacks: real PoC clean and barriers; TLBI never issued. */
extern const struct gpu_m4_vm_tree_ops gpu_m4_vm_tree_shadow_ops;
/* Installed (S3c): real PoC clean, DSB OSH/SY and VAE1OS/RVAE1OS/ASIDE1OS. */
extern const struct gpu_m4_vm_tree_ops gpu_m4_vm_tree_installed_ops;

struct gpu_m4_vm_tree;

/* REMOVE-DEBUG: 0 no tree, 1 shadow tree, 2 shadow tree + compare per change. */
unsigned int gpu_m4_vm_tree_mode(void);

struct gpu_m4_vm_tree *gpu_m4_vm_tree_create(const struct gpu_m4_vm_tree_config *config);
/* Teardown: clear every leaf, clean, DSB OSH, ASIDE1OS, DSB SY, drop every
 * reference, then free every table. A caller that installed the root first
 * retracts it from TTBAT. Authority: requires no guard and a tree that is not
 * resident; otherwise (or when faulted, also by a fault its final clear
 * finds) every page is kept (I3) and -EBUSY tells the owner to retain
 * everything the tree's leaves and callbacks name. 0: the tree is freed.
 */
int gpu_m4_vm_tree_destroy(struct gpu_m4_vm_tree *tree);

/* Apply a whole batch in order; on error nothing changed. */
int gpu_m4_vm_tree_bind(struct gpu_m4_vm_tree *tree, const struct gpu_m4_vm_tree_op *op,
			u32 count);
/* Remove every non-DRIVER extent of object (GEM close). */
int gpu_m4_vm_tree_remove_object(struct gpu_m4_vm_tree *tree, const void *object);
/* Stop mirroring: remove every extent with the full removal sequence. */
void gpu_m4_vm_tree_desync(struct gpu_m4_vm_tree *tree);
bool gpu_m4_vm_tree_desynced(struct gpu_m4_vm_tree *tree);
/* Whether the tree holds a reference on object. */
bool gpu_m4_vm_tree_holds(struct gpu_m4_vm_tree *tree, const void *object);
u64 gpu_m4_vm_tree_root(struct gpu_m4_vm_tree *tree);
void gpu_m4_vm_tree_set_asid(struct gpu_m4_vm_tree *tree, u32 asid);

/*
 * S3c binding (binding_lock; the caller holds the GPU mutex). install_begin
 * runs BEFORE the TTBAT store naming this root in asid: from then on every
 * removal invalidates asid. evicted runs only after another root replaced it
 * (or the entry was cleared) AND that ASID was invalidated: removals then
 * need no TLBI (no translation of this root can be cached).
 */
int gpu_m4_vm_tree_install_begin(struct gpu_m4_vm_tree *tree, u32 asid);
void gpu_m4_vm_tree_evicted(struct gpu_m4_vm_tree *tree);
bool gpu_m4_vm_tree_resident(struct gpu_m4_vm_tree *tree, u32 *asid);
/* Consistent (not desynced or faulted) and, for authority, the root is live. */
int gpu_m4_vm_tree_healthy(struct gpu_m4_vm_tree *tree);

/*
 * S3c accepted-job guards: O(1). get refuses a closed tree (-ESHUTDOWN). The
 * last put with CLOSED extents pending calls config.deferred.
 */
int gpu_m4_vm_tree_guard_get(struct gpu_m4_vm_tree *tree);
void gpu_m4_vm_tree_guard_put(struct gpu_m4_vm_tree *tree);
u32 gpu_m4_vm_tree_guards(struct gpu_m4_vm_tree *tree);
/* VM close: refuse new guards and user binds; every user extent is removed
 * now when no guard exists, else marked CLOSED. DRIVER extents stay.
 */
int gpu_m4_vm_tree_close(struct gpu_m4_vm_tree *tree);
/* Remove the CLOSED extents once no guard exists (else nothing): the owner's
 * worker after config.deferred. Returns the number of extents removed.
 */
int gpu_m4_vm_tree_settle_deferred(struct gpu_m4_vm_tree *tree);
u32 gpu_m4_vm_tree_deferred_count(struct gpu_m4_vm_tree *tree);

/* One run of a named range: from va, the same backing at contiguous offsets
 * with the same access (adjacent extents included).
 */
struct gpu_m4_vm_tree_span {
	u64 va, size, offset;
	void *object;
	const void *identity;
	u32 access, cpu_policy;
};

/*
 * S3c named-range validation (decode), under tree->lock: [va, va + size)
 * (bytes) lies in [low, high) outside the kernel and reserved ranges, and
 * every page is a valid leaf of a live user extent (not DRIVER, not CLOSED)
 * whose access includes access; holes are found by visited-page count. span
 * (optional) describes the run at va. -EINVAL range, -ENOENT hole, -EACCES
 * permission, -EIO faulted/desynced.
 */
int gpu_m4_vm_tree_covers(struct gpu_m4_vm_tree *tree, u64 va, u64 size, u32 access,
			  struct gpu_m4_vm_tree_span *span);
/* The installed leaf at va (any extent, DRIVER and CLOSED included) as a
 * ledger leaf: PA from the leaf word, identity/offset/access/policy from its
 * extent. -ENOENT none, -EACCES access, -EPROTO word/extent disagreement.
 */
int gpu_m4_vm_tree_page(struct gpu_m4_vm_tree *tree, u64 va, u32 access,
			struct gpu_m4_drm_vm_leaf *leaf);
/* Copy [va, va + size) (size <= one page) of live USER extents with access
 * into dst, under tree->lock so no leaf can be unmapped or its backing freed
 * mid-copy. Non-cacheable backing is read after a cache invalidate of the
 * linear alias. Errors as covers(); -EOPNOTSUPP for a non-RAM leaf.
 */
int gpu_m4_vm_tree_read(struct gpu_m4_vm_tree *tree, u64 va, u32 size, u32 access, void *dst);
/* Every page of [va, va + size) (bytes) like page(); fn sees each leaf. */
typedef int (*gpu_m4_vm_tree_leaf_fn)(void *cookie, u64 page, const struct gpu_m4_drm_vm_leaf *leaf);
u64 gpu_m4_vm_tree_revision(struct gpu_m4_vm_tree *tree);
/* No change since revision since touched [va, va + size) (false when the
 * change log no longer reaches back that far).
 */
bool gpu_m4_vm_tree_unchanged(struct gpu_m4_vm_tree *tree, u64 since, u64 va, u64 size);
int gpu_m4_vm_tree_walk(struct gpu_m4_vm_tree *tree, u64 va, u64 size, u32 access,
			gpu_m4_vm_tree_leaf_fn fn, void *cookie);
/* The DRIVER extent(s) exactly mapping [va, va + size) to object from offset
 * with exactly access (one extent per op, as inserted).
 */
bool gpu_m4_vm_tree_maps(struct gpu_m4_vm_tree *tree, u64 va, u64 size, const void *object,
			 u64 offset, u32 access, bool driver);
/* Every page of object bytes [offset, offset + size) is mapped by a live
 * (not CLOSED) extent of object with access (accepted work's access proof).
 */
bool gpu_m4_vm_tree_object_access(struct gpu_m4_vm_tree *tree, const void *object, u64 offset,
				  u64 size, u32 access);

/*
 * REMOVE-DEBUG: the user leaves (non-DRIVER) of the tree are exactly leaf[]
 * (distinct VAs), every table and parent word is consistent and the extents
 * describe exactly the valid leaves. -EPROTO names the first bad VA.
 */
int gpu_m4_vm_tree_compare(struct gpu_m4_vm_tree *tree, const struct gpu_m4_drm_vm_leaf *leaf,
			   u32 count, u64 *bad_va);

/* One expected valid leaf word of a complete (composed) image. */
struct gpu_m4_vm_tree_word {
	u64 va, pte;
	bool driver;	/* mapped by a DRIVER extent, else by a user extent */
};

/*
 * REMOVE-DEBUG composed-version oracle: the tree's complete leaves, user and
 * DRIVER, are exactly word[] (sorted here by VA; a repeated VA mismatches),
 * each in an extent of the stated class, with the structure checks of
 * compare(). -EPROTO names the first bad VA; -ESTALE when desynced.
 */
int gpu_m4_vm_tree_compare_words(struct gpu_m4_vm_tree *tree, struct gpu_m4_vm_tree_word *word,
				 u32 count, u64 *bad_va);
/* REMOVE-DEBUG: an oracle point whose expected image was not current. */
void gpu_m4_vm_tree_oracle_skipped(void);

#endif
