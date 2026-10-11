/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_DRM_PROVIDER_UPPER_H
#define APPLE_GPU_M4_DRM_PROVIDER_UPPER_H

#include "gpu-m4-drm.h"

struct mutex;
struct gpu_m4_drm_upper;
struct gpu_m4_drm_upper_lease;

#define GPU_M4_DRM_UPPER_START 0xfffffc2000000000ULL
#define GPU_M4_DRM_UPPER_TIMESTAMP 0xfffffc2ffc000000ULL
#define GPU_M4_DRM_UPPER_END 0xfffffc3000000000ULL
#define GPU_M4_DRM_UPPER_FW_ATTRS 0x00c000000000044bULL
#define GPU_M4_DRM_UPPER_SHARED_ATTRS 0x00e000000000040bULL

enum gpu_m4_drm_upper_window { GPU_M4_DRM_UPPER_NORMAL, GPU_M4_DRM_UPPER_TS };
enum gpu_m4_drm_upper_class { GPU_M4_DRM_UPPER_FW_ONLY, GPU_M4_DRM_UPPER_SHARED };
enum gpu_m4_drm_upper_placement {
	GPU_M4_DRM_UPPER_ANY_CHILD,
	GPU_M4_DRM_UPPER_EXISTING_ONLY,
	GPU_M4_DRM_UPPER_NEW_ONLY,
};
enum gpu_m4_drm_upper_table_class {
	GPU_M4_DRM_UPPER_RESERVED_ROOT, GPU_M4_DRM_UPPER_RESERVED_MIDDLE,
	GPU_M4_DRM_UPPER_INHERITED_FOREIGN, GPU_M4_DRM_UPPER_INHERITED_AP,
};
enum gpu_m4_drm_upper_gate {
	GPU_M4_DRM_UPPER_EXISTING_CHILD = 1,
	GPU_M4_DRM_UPPER_NEW_PARENT = 2,
	GPU_M4_DRM_UPPER_TIMESTAMP_GATE = 4,
};

/* Explicit retained inherited inventory. No unknown PA is CPU-dereferenced.
 * An AP entry permits ZERO-slot writes; a foreign entry is never writable.
 * The mandatory oracle validates the actual no-map/AP-owned view and owner.
 */
struct gpu_m4_drm_upper_table {
	void *cpu;
	u64 pa;
	void *owner;
	enum gpu_m4_drm_upper_table_class class;
};

/* Already pinned backing: acceptance never takes a GEM reservation lock or
 * creates a CPU mapping. In particular WB timestamp BOs keep their WB view.
 * get/page/check/ready run with the ONE provider mutex. put runs outside ALL
 * provider/reservation locks, including rollback and final shutdown release.
 * check proves exact native identity/offset/PA and excludes firmware/MMIO/table
 * or unrelated backing. ready proves job cache protection before publication;
 * an NC backing still needs its actual owner/alias readiness check.
 * This initial owner deliberately admits ONE canonical whole-page span per
 * backing. Timestamp subobjects in that span reference its exact lease and add
 * their separately checked eight-byte object offset. Different/overlapping
 * span identities are NOT another qualified PA alias, even for the same BO.
 */
struct gpu_m4_drm_upper_backing {
	void *owner;
	const void *identity;
	u64 offset, size;
	enum gpu_m4_drm_cpu_policy policy;
	int (*get_locked)(void *owner);
	void (*put)(void *owner);
	int (*page_locked)(void *owner, u64 offset, u64 *pa);
	int (*check_locked)(void *owner, const void *identity, u64 offset, u64 pa);
	int (*ready_locked)(void *owner, u64 offset, u64 size,
			    enum gpu_m4_drm_upper_class class);
};

/* Checked owned updates for READY's strict expected table inventory. Root
 * registers new AP children and updates only these selected zero slots. This
 * callback must not repair foreign bytes or publish additional hardware state.
 * previous is the expected-ledger value replaced: zero for a publication, the
 * cleared leaf PTE for a release (value then zero).
 */
struct gpu_m4_drm_upper_update {
	void *table_cpu;
	u64 table_pa, value, previous;
	u32 index;
	bool parent;
};

struct gpu_m4_drm_upper_ops {
	/* This actual session ref retains root/middle and EVERY explicit inherited
	 * child owner/view until session_put, not merely the callback cookie.
	 */
	int (*session_get_locked)(void *session);
	void (*session_put)(void *session);
	int (*table_locked)(void *session, const struct gpu_m4_drm_upper_table *table);
	/* Optional read-only classification of firmware-owned root words across
	 * stopped publication -> HELLO. Never approves an AP index-2 transition or
	 * an arbitrary observed child. Without it every foreign word stays exact.
	 */
	int (*foreign_root_locked)(void *session, u32 index, u64 initial, u64 observed);
	/* Check healthy RUN/session, no active work, ownership and qualification.
	 * trial=true is explicit diagnostic authority, never production success.
	 * gates distinguish existing-child, new-parent and timestamp evidence.
	 */
	int (*gate_locked)(void *session, u32 gates, bool trial);
	int (*handoff_get_locked)(void *session, u32 ceiling_ms);
	int (*handoff_put_locked)(void *session);
	int (*clean_locked)(void *session, void *cpu, u32 size);
	int (*barrier_locked)(void *session);
	/* Complete outer-shareable barriers and full ASID-0 TLBI, operand ZERO.
	 * ASID-only invalidation does NOT remove global final-level entries, and
	 * these upper leaves are global (nG clear): it completes publication into
	 * zero slots only and never invalidates a removed/replaced valid leaf.
	 */
	int (*asid0_tlbi_locked)(void *session);
	int (*commit_locked)(void *session,
		const struct gpu_m4_drm_upper_update *updates, u32 count);
	/* By-VA ASID-0 invalidation of every 16 KiB page of [va, va + size),
	 * global final-level entries included (e.g. one TLBI VAE1OS per page),
	 * after the preceding cleaned stores (DSB OSH) and completed (DSB SY)
	 * before returning success. Required for release, which never falls back
	 * to asid0_tlbi_locked: that cannot remove the global leaves.
	 */
	int (*asid0_range_tlbi_locked)(void *session, u64 va, u64 size);
	/* Optional, required for release: adopt exactly these cleared leaf slots
	 * (value zero, previous = the released PTE) in the session's own expected
	 * images. Never a parent, foreign or unowned table.
	 */
	int (*release_commit_locked)(void *session,
		const struct gpu_m4_drm_upper_update *updates, u32 count);
	/* Independently qualified permanent firmware halt/root shutdown, not an
	 * error fence, consumed ring, RTKit reset, or a mere requested stop.
	 */
	int (*shutdown_locked)(void *session);
	void (*fault_locked)(void *session, int error);
	/* Optional ONLY for explicit stopped-bootstrap seed import. Authenticate
	 * the retained READY declaration, exact native backing/VA/class, completed
	 * stopped publication and RUN=0. Not observed-PTE adoption or live authority.
	 */
	int (*seed_locked)(void *session, const struct gpu_m4_drm_upper_backing *backing,
			   u64 va, enum gpu_m4_drm_upper_class class);
};

struct gpu_m4_drm_upper_config {
	struct mutex *lock;
	void *session;
	const struct gpu_m4_drm_upper_ops *ops;
	struct gpu_m4_drm_upper_table root, middle;
	const struct gpu_m4_drm_upper_table *children;
	/* Mandatory aligned floor AFTER every sealed READY allocation and its
	 * guard, normally READY next_va. Existing zero bootstrap guard holes below
	 * this floor are unavailable, even in a otherwise writable AP child.
	 * Timestamp allocations use their separate dedicated interval instead.
	 */
	u64 normal_min_va;
	u32 child_count, max_children, max_pages, max_leases, scan_pages;
};

/* Caller holds config.lock. Real WB child pages are allocated on demand, not
 * all 2048 at create. Existing root[2] same-PA attributes are never rewritten.
 */
struct gpu_m4_drm_upper *gpu_m4_drm_upper_create_locked(
	const struct gpu_m4_drm_upper_config *config);
struct gpu_m4_drm_upper_lease *gpu_m4_drm_upper_accept_locked(
	struct gpu_m4_drm_upper *upper,
	const struct gpu_m4_drm_upper_backing *backing,
	enum gpu_m4_drm_upper_window window, enum gpu_m4_drm_upper_class class);
/* Import one immutable, independently retained native NC READY seed AFTER its
 * stopped bootstrap publication, BEFORE RUN. Require seed_locked authority and
 * exact live AP-owned PTEs/guard matching the actual backing inventory. The seed
 * and guard lie BELOW normal_min_va; TS/foreign slots are never imported.
 * Returns one real canonical lease ref. No PTE/root/expected writes, cache clean,
 * handoff or TLBI. Errors do not consume the READY baseline or mutate tables;
 * the caller retains all prepublished owners on failure. Only the qualified
 * same-PA root[2] normalization is accepted for this seed after RUN.
 */
struct gpu_m4_drm_upper_lease *gpu_m4_drm_upper_import_seed_locked(
	struct gpu_m4_drm_upper *upper, const struct gpu_m4_drm_upper_backing *backing,
	u64 va, enum gpu_m4_drm_upper_class class);
/* Explicit diagnostic placement. minimum_va=0 uses the configured normal
 * floor or timestamp start; nonzero may only raise that floor. Backing AND
 * trailing guard must fit the interval and selected live-parent class.
 * Every scanned page counts against config.scan_pages, including unsuitable
 * inherited/foreign parents. A new-only private lease is rejected at publish
 * if another publication has already linked any selected parent. Production
 * default accept above uses ANY_CHILD; no placement implies qualification.
 */
struct gpu_m4_drm_upper_lease *gpu_m4_drm_upper_accept_placed_locked(
	struct gpu_m4_drm_upper *upper,
	const struct gpu_m4_drm_upper_backing *backing,
	enum gpu_m4_drm_upper_window window, enum gpu_m4_drm_upper_class class,
	u64 minimum_va, enum gpu_m4_drm_upper_placement placement);
/* Exact owned lease recheck/reference, with NO table writes. */
int gpu_m4_drm_upper_reference_locked(struct gpu_m4_drm_upper_lease *lease,
	const struct gpu_m4_drm_upper_backing *backing,
	enum gpu_m4_drm_upper_window window, enum gpu_m4_drm_upper_class class);
/* Read-only whole-lease translation check for an already held published owner.
 * Match the exact backing/VA/class, then check its exact owned root/parent/
 * child/leaf path and trailing guard against the pinned inventory captured at
 * admission (held refs forbid migration). No other lease's inventory, no
 * backing page oracle and no global graph walk. An unchanged NORMAL-window NC
 * lease reuses its publication proof exactly as publish_locked does (proof and
 * the latest live health pass at the current revision/root[2]). Unlike
 * reference_locked this takes no reference and never changes owner state.
 */
int gpu_m4_drm_upper_check_lease_locked(struct gpu_m4_drm_upper_lease *lease,
	const struct gpu_m4_drm_upper_backing *backing,
	enum gpu_m4_drm_upper_window window, enum gpu_m4_drm_upper_class class, u64 va);
u64 gpu_m4_drm_upper_va_locked(struct gpu_m4_drm_upper_lease *lease);
/* First publication walks/writes/proves the lease. Reuse of an already
 * published lease runs its owned PTE/guard path check, except an unchanged
 * NORMAL-window NC lease whose publication proof and the owner's latest live
 * health pass are both at the current revision/root[2]; either way it then
 * calls the backing's ready_locked (current job access/protection).
 */
int gpu_m4_drm_upper_publish_locked(struct gpu_m4_drm_upper_lease *lease, bool trial);
/* The owner revision: advanced by every lease/table/expected-ledger mutation
 * (never by a check). An identity for callers' memos of a whole-graph pass,
 * never itself a health or translation proof. 0 for no owner.
 */
u64 gpu_m4_drm_upper_revision_locked(struct gpu_m4_drm_upper *upper);
/* REMOVE-DEBUG: published-lease reuse statistics (full path walks/proof reuse). */
void gpu_m4_drm_upper_publish_stats_locked(struct gpu_m4_drm_upper *upper, u64 *walks,
	u64 *reused);
/* REMOVE-DEBUG: held-lease check statistics (full path walks/proof reuse). */
void gpu_m4_drm_upper_lease_stats_locked(struct gpu_m4_drm_upper *upper, u64 *walks,
	u64 *reused);
/* REMOVE-DEBUG: placement scan cost. pages is the scanned span from each
 * call's first page (the whole limit on refusal); reads are live child slot
 * reads; steps are reservation-extent and middle-segment iterations.
 */
struct gpu_m4_drm_upper_choose_stats {
	u64 calls, failures, ns, pages, reads, steps;
};
void gpu_m4_drm_upper_choose_stats_locked(struct gpu_m4_drm_upper *upper,
	struct gpu_m4_drm_upper_choose_stats *stats);
int gpu_m4_drm_upper_put_locked(struct gpu_m4_drm_upper_lease *lease);
/* Release one published dynamic NORMAL-window lease whose firmware use has
 * provably ended. The CALLER proves that authority (own retirement, a retired
 * distinct same-slot replacement, not current, not in flight, not quarantined,
 * no remaining user); this owner requires the exact held backing, the only
 * reference, a non-seed lease, an unfaulted owner and release_commit_locked.
 * Under handoff every leaf slot is cleared in the live table and the expected
 * ledger (clean to PoC, barrier), then outside it the by-VA ASID-0
 * invalidation of every released page completes (asid0_range_tlbi_locked,
 * required: never the ASID-only fallback, which misses these global leaves),
 * the zeroing is committed and the revision advances,
 * voiding every publication/check/health proof. Only then are the VA and its
 * guard free again; PA claims and the backing reference drop at reap, outside
 * all locks. A refusal before the first store changes nothing; any later
 * failure latches the owner fault and retains the lease, still referenced.
 */
int gpu_m4_drm_upper_release_locked(struct gpu_m4_drm_upper_lease *lease,
	const struct gpu_m4_drm_upper_backing *backing);
/* REMOVE-DEBUG: successful releases and latched release failures. */
void gpu_m4_drm_upper_usage_locked(struct gpu_m4_drm_upper *upper, u32 *leases, u32 *pages);
void gpu_m4_drm_upper_release_stats_locked(struct gpu_m4_drm_upper *upper, u64 *released,
	u64 *failures);
/* Full diagnostic/fault verification: all foreign graph snapshots, every
 * lease's PTE/guard and its exact retained backing inventory through all page
 * and identity oracles, plus checked-PA-index consistency. Errors never repair
 * bytes or latch a new owner state.
 */
int gpu_m4_drm_upper_verify_locked(struct gpu_m4_drm_upper *upper);
/* Per-publication/poll live health. Compares the complete live table graph
 * against its exact expected ledger on every call, and every lease's owned
 * PTE/guard whenever the owner revision or root[2] changed since the last
 * complete pass. Admitted page inventories/PA uniqueness are not rebuilt:
 * only verify_locked reruns backing oracles. Never a completion grant.
 */
int gpu_m4_drm_upper_health_locked(struct gpu_m4_drm_upper *upper);
/* Resolve ONLY a published dynamic lease, with exact attrs/backing checks.
 * Original inherited aliases remain under their separate bootstrap owner.
 * Guard/zero/foreign/nonlease VA returns -ENOENT; no borrowed PA is followed.
 * Outputs are untouched on every error. Byte offset is included in *pa.
 * Checks the root words, that VA's owned parent/child/leaf against the held
 * admitted PA/attrs and the lease's trailing guard; not a global graph walk.
 */
int gpu_m4_drm_upper_walk_locked(struct gpu_m4_drm_upper *upper, u64 va,
			       u64 *pa, u64 *attrs);
/* Borrow a known AP child view for bounded evidence, including an accepted
 * still-detached child. Validate its retained owner, NOT arbitrary live PAs.
 * This is inventory, not successful translation or graph-integrity proof; it
 * remains usable after table-byte drift. Root keeps upper alive through reads.
 * Inherited foreign children/unknown indices are never returned.
 */
int gpu_m4_drm_upper_table_locked(struct gpu_m4_drm_upper *upper, u32 middle,
				struct gpu_m4_drm_upper_table *table);
int gpu_m4_drm_upper_error_locked(struct gpu_m4_drm_upper *upper);
int gpu_m4_drm_upper_shutdown_locked(struct gpu_m4_drm_upper *upper);
/* Outside ALL locks. Reap never-published private backing and released
 * leases. Destroy refuses outstanding refs or published uncertainty without
 * qualified shutdown. Live tables are never reclaimed; only released leaf
 * slots return to zero (gpu_m4_drm_upper_release_locked).
 */
void gpu_m4_drm_upper_reap(struct gpu_m4_drm_upper *upper);
int gpu_m4_drm_upper_destroy(struct gpu_m4_drm_upper *upper);

#endif
