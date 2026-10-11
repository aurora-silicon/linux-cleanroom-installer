/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_DRM_PROVIDER_VM_H
#define APPLE_GPU_M4_DRM_PROVIDER_VM_H

#include "gpu-m4-drm-uat.h"

struct gpu_m4_drm_provider;
struct gpu_m4_drm_provider_vm;
struct gpu_m4_drm_provider_version;
struct gpu_m4_drm_provider_object;
struct gpu_m4_drm_provider_private_lease;

/* Private READY-session owner, not a registered DRM backend. The oracle must
 * reject firmware/table/MMIO/foreign backing, in addition to the native pin
 * identity/offset/PA check performed here. No callback publishes hardware.
 * Session get/put retain the actual powered owner, not a borrowed device cookie.
 */
struct gpu_m4_drm_provider_config {
	void *session;
	int (*get)(void *session);
	void (*put)(void *session);
	/* Optional actual GPU-owner mutex. Session references retain its lifetime;
	 * validate_lock must prove this exact pointer belongs to that owner. An
	 * internal mutex is used only for detached/private testing when NULL.
	 */
	struct mutex *lock;
	int (*validate_lock)(void *session, struct mutex *lock);
	int (*validate_page)(void *session, struct gpu_m4_drm_vm *vm,
			     const void *identity, u64 offset, u64 pa);
	u64 vm_start, vm_end;
	/* The provider's OWN kernel allocations: context data, all version tables
	 * and conservative version metadata, and each VM's pin bookkeeping (object
	 * record and PA inventory). A pinned GEM's backing pages are never charged
	 * here: the GEM factory paid for them (shmem, or the native budget for
	 * private allocators), and one GEM pinned by several VMs is one backing.
	 * Per-job acceptance buffers and external private allocators need their
	 * own finite budgets; aliases do not allocate/charge backing twice.
	 */
	u64 memory_limit;
	/* Finite provider-wide reserve above memory_limit, usable ONLY by a
	 * removal-only replacement (BO close, baseline removal) whose replaced
	 * owner may still be held. Outstanding excess never exceeds it; what it
	 * consumed returns only with actual refunds. Zero: no such allowance.
	 */
	u64 removal_reserve;
	u32 max_leaves, max_tables;
	/* Ordinary host VM updates commit owned logical state only. Submission
	 * capture constructs its immutable snapshot; the final staged candidate
	 * must explicitly construct its graph before any hardware authority.
	 * Bootstrap and explicit internal-native updates remain eager.
	 */
	bool defer_user_materialization;
	/* Optional cheap live gate for a page whose immutable legitimacy was
	 * already admitted by validate_page for the SAME retained native pin, PA
	 * vector, length, CPU policy and session. Called under the provider lock:
	 * 0 accepts the cached admission, a positive value requests the full
	 * validate_page oracle, a negative value rejects. NULL keeps the full
	 * oracle for every page. Admission is page legitimacy only: never job
	 * protection, mapping, reservation, visibility or completion.
	 */
	int (*admitted_page)(void *session, struct gpu_m4_drm_vm *vm,
			     const void *identity, u64 offset);
	/* Optional, provider mutex held, AFTER a provider fault (vm NULL) or a
	 * VM-local error of vm is latched: wake that scope's parked CPU-side
	 * owners. It may only queue work: no lock, wait, signal, put or fault.
	 */
	void (*error_wake_locked)(struct gpu_m4_drm_provider *provider,
				  struct gpu_m4_drm_provider_vm *vm);
};

struct gpu_m4_drm_provider *gpu_m4_drm_provider_create(
	const struct gpu_m4_drm_provider_config *config);
int gpu_m4_drm_provider_get(struct gpu_m4_drm_provider *provider);
/* Final put refuses reachable VM/version/resident/garbage owners. */
int gpu_m4_drm_provider_put(struct gpu_m4_drm_provider *provider);
struct mutex *gpu_m4_drm_provider_lock(struct gpu_m4_drm_provider *provider);
u64 gpu_m4_drm_provider_bytes_locked(struct gpu_m4_drm_provider *provider);
/* TTBAT contexts: 0 is the firmware's own; user ASIDs are 1..63. Every
 * binding entry below is per ASID; the unqualified forms name ASID 1.
 */
enum { GPU_M4_DRM_PROVIDER_ASIDS = 64 };
/* Borrowed resident identity is protected by the provider baseline reference. */
struct gpu_m4_drm_provider_version *gpu_m4_drm_provider_asid_resident_locked(
	struct gpu_m4_drm_provider *provider, u32 asid);
struct gpu_m4_drm_provider_version *gpu_m4_drm_provider_resident_locked(
	struct gpu_m4_drm_provider *provider);
/* Wide successful per-ASID bind identity, NOT a firmware byte or software
 * VM_BIND/event/threshold count. It advances only on a completed installation
 * of a changed root in that ASID, never on same-image adoption, and is never
 * reset (an unbind keeps it), so an (ASID, generation) pair never repeats.
 */
u64 gpu_m4_drm_provider_asid_generation_locked(struct gpu_m4_drm_provider *provider, u32 asid);
u64 gpu_m4_drm_provider_binding_generation_locked(struct gpu_m4_drm_provider *provider);
/* Firmware wire encoding: zero is unbound, otherwise cycle 1..255 then 1.
 * A wire byte alone is NEVER an exact binding/old-job identity.
 */
u8 gpu_m4_drm_provider_binding_wire_generation(u64 generation);
/* Exact retained resident binding, including already-accepted closed-VM work.
 * Reject fault, pending installation and nonresident/foreign identities. No
 * admission or hardware-health grant; no refs/root mutation. Outputs unchanged
 * on failure. Caller retains provider/version and holds the provider mutex.
 */
/* This ASID alone has no pending installation and no working resident: the
 * per-ASID authority a new start or rebind there needs. Other ASIDs' work
 * is not consulted. No refs or state change.
 */
int gpu_m4_drm_provider_asid_quiescent_locked(struct gpu_m4_drm_provider *provider, u32 asid);
int gpu_m4_drm_provider_asid_binding_locked(struct gpu_m4_drm_provider *provider, u32 asid,
	struct gpu_m4_drm_provider_version *version, u64 *generation, u8 *wire_generation);
int gpu_m4_drm_provider_binding_locked(struct gpu_m4_drm_provider *provider,
	struct gpu_m4_drm_provider_version *version, u64 *generation, u8 *wire_generation);
int gpu_m4_drm_provider_error_locked(struct gpu_m4_drm_provider *provider);
/* Lockless sticky provider failure for status reports; never a gate. */
int gpu_m4_drm_provider_error(struct gpu_m4_drm_provider *provider);
/* Shared-domain fail-stop: withdraws every VM's capture, latches no VM. */
void gpu_m4_drm_provider_fault_locked(struct gpu_m4_drm_provider *provider, int error);
/* Latch this VM's frontend status (first error wins): the owner of an admitted
 * job whose fence fails for any reason. Never for an ioctl refusal.
 */
void gpu_m4_drm_provider_vm_quarantine_locked(struct gpu_m4_drm_provider_vm *vm, int error);
/* This VM's VM-local error, else its latched frontend status, else 0. */
int gpu_m4_drm_provider_vm_status_locked(struct gpu_m4_drm_provider_vm *vm);
/* Host capacity refusal (finite budget or allocator), by itself never a
 * provider/hardware integrity fault: before publication it fails only its
 * own request, job or VM. Never pass it to fault_locked from such a site.
 */
static inline bool gpu_m4_drm_provider_capacity_error(int error)
{
	return error == -ENOSPC || error == -ENOMEM;
}

/* These are actual frontend callback implementations. Create retains frontend
 * VM and allocates its one persistent zeroed NC 64 KiB context. Bind stages
 * native pins with reservation locks BEFORE taking the provider mutex, checks
 * the VM/generation again, validates owned logical state, then commits
 * atomically. Ordinary deferred updates do not build snapshots or tables.
 * Only the two source-qualified low reservations are unavailable to user binds.
 */
struct gpu_m4_drm_provider_vm *gpu_m4_drm_provider_vm_create(
	struct gpu_m4_drm_provider *provider, struct gpu_m4_drm_vm *frontend,
	u64 kernel_start, u64 kernel_end);
/* Same construction, but failed context/version owners (and unrelated prior
 * garbage) are never collected on this frontend-serialized call. Arrange reap
 * after success OR failure outside frontend/provider/reservation locks.
 */
struct gpu_m4_drm_provider_vm *gpu_m4_drm_provider_vm_create_deferred(
	struct gpu_m4_drm_provider *provider, struct gpu_m4_drm_vm *frontend,
	u64 kernel_start, u64 kernel_end);
/* Finish a fresh, uncaptured VM's generation-1 private baseline. External
 * READY allocations and their exact guarded claims become active together;
 * subsequent VM_BIND preserves them. Does not publish roots or change the
 * user generation. Caller retains its claim refs; the version acquires its
 * own. Failure leaves active unchanged and queues rollback for outside-lock
 * reap. Never use this to adopt a captured, resident or already extended VM.
 */
int gpu_m4_drm_provider_vm_initialize_private_locked(
	struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_uat_private *private,
	struct gpu_m4_drm_provider_private_lease *const *leases, u32 count);
int gpu_m4_drm_provider_vm_bind(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_bind *binds, u32 count);
/* Identical PUBLIC handle/admission/atomic-generation semantics, for callbacks
 * holding frontend serialization. Final native rollback, rejected/duplicate
 * stage pins and old-version collection are queued, still reachable/charged.
 * Caller MUST arrange provider_reap outside frontend/provider/resv locks after
 * success OR failure. Neither bind entrance is callable under provider/resv.
 */
int gpu_m4_drm_provider_vm_bind_deferred(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_bind *binds, u32 count);
/* Explicit internal native mapping, NEVER a frontend numeric-handle lookup.
 * Caller retains the actual GEM pointers through this synchronous transaction.
 * op.handle MUST be zero; only READ/WRITE bindings are admitted, not UNBIND or
 * SINGLE_PAGE. All native origin/pin/policy/geometry/exclusion and atomic
 * generation checks remain the ordinary path's. Public vm_bind and raw ledger
 * changes still require nonzero bind handles. No hardware publication occurs.
 * Call outside provider/reservation locks, exactly like ordinary vm_bind.
 */
int gpu_m4_drm_provider_vm_bind_native(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_bind *binds, u32 count);
/* Frontend BO-close callback adapter. Caller retains VM/GEM and may hold its
 * frontend serialization lock, but NEVER provider/reservation locks. Remove
 * exact-GEM active USER leaves only; captured/private/hardware owners survive.
 * No-match has no generation churn. Failure latches provider/frontend VM error
 * and retains old active. This only queues garbage: caller must arrange reap
 * separately OUTSIDE frontend/provider/reservation locks.
 */
void gpu_m4_drm_provider_vm_bo_close(struct gpu_m4_drm_provider_vm *vm,
	struct drm_gem_object *object);
int gpu_m4_drm_provider_vm_destroy(struct gpu_m4_drm_provider_vm *vm);
void gpu_m4_drm_provider_vm_close(struct gpu_m4_drm_provider_vm *vm);
/* Frontend callbacks already serialized by the shared provider mutex use these
 * variants. Close stops ONLY this VM's admission; accepted/captured/resident
 * owners survive. Destroy succeeds only for an unshared inactive VM. Neither
 * performs final frees: arrange reap OUTSIDE frontend/provider/resv locks.
 * The outside-lock wrappers above preserve their immediate-reap behavior.
 */
int gpu_m4_drm_provider_vm_destroy_locked(struct gpu_m4_drm_provider_vm *vm);
void gpu_m4_drm_provider_vm_close_locked(struct gpu_m4_drm_provider_vm *vm);
int gpu_m4_drm_provider_vm_get_locked(struct gpu_m4_drm_provider_vm *vm);
void gpu_m4_drm_provider_vm_put_locked(struct gpu_m4_drm_provider_vm *vm);
struct gpu_m4_drm_provider *gpu_m4_drm_provider_vm_provider(
	struct gpu_m4_drm_provider_vm *vm);
bool gpu_m4_drm_provider_vm_open_locked(struct gpu_m4_drm_provider_vm *vm);
int gpu_m4_drm_provider_vm_error_locked(struct gpu_m4_drm_provider_vm *vm);

/* Object acquisition may lock object->resv: NEVER call under provider/resv
 * locks. It returns a retained native owner and immutable whole-object PA
 * inventory, charged until its final deferred release. Timestamp alias owners
 * use this before their separate qualified upper-VA lease operation.
 * Native pin pointers are borrowed: keep the provider object (or a version
 * which owns it) until ALL snapshot/cache pin references have been released.
 * Otherwise deferred collection could uncharge still-retained native pages.
 */
struct gpu_m4_drm_provider_object *gpu_m4_drm_provider_object_pin(
	struct gpu_m4_drm_provider_vm *vm, struct drm_gem_object *object);
/* Frontend-serialized native staging: same origin/accounting rules, but final
 * rejected/duplicate/failed pins are collected only by outside-lock reap.
 */
struct gpu_m4_drm_provider_object *gpu_m4_drm_provider_object_pin_deferred(
	struct gpu_m4_drm_provider_vm *vm, struct drm_gem_object *object);
int gpu_m4_drm_provider_object_get_locked(struct gpu_m4_drm_provider_object *object);
void gpu_m4_drm_provider_object_put_locked(struct gpu_m4_drm_provider_object *object);
struct gpu_m4_drm_bo_pin *gpu_m4_drm_provider_object_native(
	struct gpu_m4_drm_provider_object *object);
struct drm_gem_object *gpu_m4_drm_provider_object_identity(
	struct gpu_m4_drm_provider_object *object);
/* Healthy retained origin, NOT new-work admission. Caller holds provider and
 * exact captured version/object references under the provider mutex. Unknown
 * identities and zero-ref garbage are rejected before dereferencing them.
 * A historical version or closed VM remains valid; no active-generation or
 * lower-ledger membership is required (file-global timestamp BOs need neither).
 * Return the latched provider error rather than treating fault retention as
 * health. Callers separately prove open admission/current-job protection and
 * use object_page_locked() to verify the complete actual PA inventory.
 */
int gpu_m4_drm_provider_object_origin_locked(struct gpu_m4_drm_provider *provider,
	struct gpu_m4_drm_provider_version *captured,
	struct gpu_m4_drm_provider_object *object);
int gpu_m4_drm_provider_object_page_locked(struct gpu_m4_drm_provider_object *object,
	u64 offset, u64 *pa);

/* Exact software reservation only, not automatic placement or GPU mapping.
 * Explicit guards and payload must all lie in the private interval. Claims
 * include guards and cannot overlap another same-VM claim. Versions retain
 * the lease across a serialized V0/V1 or alternative NC/WB version family.
 * Final puts queue deferred collection; they never unmap/retire hardware.
 */
struct gpu_m4_drm_provider_private_lease *gpu_m4_drm_provider_private_reserve_locked(
	struct gpu_m4_drm_provider_vm *vm, u64 va, u64 size,
	u64 guard_before, u64 guard_after);
/* Share the exactly matching registered claim of this open VM (same VA, size
 * and both guards), taking one reference: the caller's fixed role VA whose
 * previous drained owner left only immutable captured versions holding the
 * identical reservation. Any other overlap is -EADDRINUSE; none is -ENOENT.
 * The caller proves the VA's exclusive role ownership; this adopts no mapping.
 */
struct gpu_m4_drm_provider_private_lease *gpu_m4_drm_provider_private_adopt_locked(
	struct gpu_m4_drm_provider_vm *vm, u64 va, u64 size,
	u64 guard_before, u64 guard_after);
/* Read-only placement in the actual held same-VM claim inventory. Skip whole
 * guarded overlaps, not a fixed number of candidate pages. Alignment is a
 * power of two between one page and the aperture size. No claim/ref/map is
 * acquired; call exact reserve_locked under this SAME mutex afterward. Fixed
 * role VAs must still use exact reserve, never silently relocate. Output stays
 * unchanged on failure; close/fault and genuine aperture exhaustion fail shut.
 */
int gpu_m4_drm_provider_private_find_locked(struct gpu_m4_drm_provider_vm *vm,
	u64 minimum_va, u64 size, u64 alignment, u64 guard_before, u64 guard_after,
	u64 *va);
/* Held claim identity remains valid after close; this is not work admission. */
int gpu_m4_drm_provider_private_check_locked(struct gpu_m4_drm_provider_private_lease *lease,
	struct gpu_m4_drm_provider_vm *vm, u64 va, u64 size);
int gpu_m4_drm_provider_private_get_locked(struct gpu_m4_drm_provider_private_lease *lease);
void gpu_m4_drm_provider_private_put_locked(struct gpu_m4_drm_provider_private_lease *lease);
/* Read-only sole-ownership query for an exact registered claim (VM, VA, size,
 * guards), open or closed VM: 0 only when the caller's one ref is the only
 * one, -EBUSY while e.g. a captured version still holds it. No ref changes.
 */
int gpu_m4_drm_provider_private_sole_locked(struct gpu_m4_drm_provider *provider,
	struct gpu_m4_drm_provider_vm *vm, struct gpu_m4_drm_provider_private_lease *lease,
	u64 va, u64 size, u64 guard_before, u64 guard_after);

/* No caller PA, CPU policy or page-resolution callback. Stage object_pin()
 * outside provider/reservation locks; retain object and lease through this
 * call. Actual native origin, immutable policy, pin and pages are derived here.
 */
struct gpu_m4_drm_provider_private_gem {
	struct gpu_m4_drm_provider_object *object;
	struct gpu_m4_drm_provider_private_lease *lease;
	u64 va, offset, size;
	u32 access;
};

/*
 * Persistent private native BASELINE: per-VM immutable mapping-only membership
 * of completed native units (actual object/pin + PA inventory + policy, typed
 * private claim and guards, VA/offset/span/access). It has its own private
 * membership revision, distinct from the user ledger generation and from the
 * hardware binding generation, and never adds anything to a job's cache/resv
 * access set. All calls hold the provider mutex.
 *
 * add: install a COMPLETED unit's exact typed mapped descriptors into the
 *	VM's next baseline. expected_revision must equal the current revision
 *	(-EAGAIN otherwise: re-resolve and retry). Validation equals the per-child
 *	append (origin/pin, claim+guards, bounds, permission, every PA through
 *	the pin's admitted oracle) plus collisions with the common private
 *	baseline and every existing/new member. Atomic: any failure leaves the
 *	baseline unchanged. On success *revision is the new revision.
 * remove: drop exact members from FUTURE membership only (-ENOENT if any is
 *	not an exact member, nothing changed). Holders of older baselines (job
 *	versions/captures) keep their mappings and backing; no physical release,
 *	no residency change. Actual release stays with the existing collectors,
 *	and with a finished composition's own image release once a later binding
 *	or unbind replaced it as the resident root (it then becomes RETIRED).
 * A VM close makes the baseline undiscoverable; its references then live only
 * in versions composed from it. VM_BIND/BO_CLOSE plans carry the exact
 * revision they were made against and replan after a concurrent commit.
 */
struct gpu_m4_drm_provider_baseline;
int gpu_m4_drm_provider_vm_baseline_add_locked(struct gpu_m4_drm_provider_vm *vm,
	u64 expected_revision, const struct gpu_m4_drm_provider_private_gem *maps, u32 count,
	u64 *revision);
int gpu_m4_drm_provider_vm_baseline_remove_locked(struct gpu_m4_drm_provider_vm *vm,
	u64 expected_revision, const struct gpu_m4_drm_provider_private_gem *maps, u32 count,
	u64 *revision);
u64 gpu_m4_drm_provider_vm_baseline_revision_locked(struct gpu_m4_drm_provider_vm *vm);
/*
 * REMOVE-DEBUG until S3c (shadow per-VM tree, gpu_m4_drm_vm_tree.mode >= 1):
 * mirror exactly the descriptors of a baseline add/remove that just
 * committed (same maps) as DRIVER extents of the VM's tree: inserted holding
 * their objects, or removed with the tree's full clear/clean/TLBI/DSB SY
 * sequence. The baseline stays the authority: nothing here can fail the
 * caller; a tree failure only desyncs the tree. Provider mutex held.
 */
void gpu_m4_drm_provider_vm_tree_native_add_locked(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_provider_private_gem *maps, u32 count);
void gpu_m4_drm_provider_vm_tree_native_remove_locked(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_provider_private_gem *maps, u32 count);
/*
 * S3c tree authority (module parameter vm_tree_authority, sampled per VM at
 * creation, ordinary profile only; default off). An authority VM's
 * persistent tree is its hardware authority: VM_BIND and GEM close edit only
 * the tree (no version, no provider mutex for already admitted objects),
 * TTBAT names the tree root through the VM's one TREE version, and jobs hold
 * that version plus an O(1) tree guard instead of a captured snapshot. See
 * gpu-m4-drm-vm-tree.h for the invariants.
 */
struct gpu_m4_vm_tree_span;
bool gpu_m4_drm_provider_vm_authority(struct gpu_m4_drm_provider_vm *vm);
/* A held reference to the VM's TREE version (no provider mutex); ERR_PTR
 * when not an authority VM (-EOPNOTSUPP) or closed (-ENOENT). Put it with
 * version_put_locked.
 */
struct gpu_m4_drm_provider_version *gpu_m4_drm_provider_vm_tree_version(
	struct gpu_m4_drm_provider_vm *vm);
/* Accepted-job guard (no provider mutex): refused once the VM is closed or
 * failed. The last put after a guarded GEM/VM close removes those mappings
 * (provider worker).
 */
int gpu_m4_drm_provider_vm_guard_get(struct gpu_m4_drm_provider_vm *vm);
/* A job's acceptance on an authority VM (no provider mutex): a held TREE
 * version and a job-local wrapper of its empty snapshot. Not a capture: no
 * user leaf, row or pin; the live tree is the authority.
 */
int gpu_m4_drm_provider_vm_tree_capture(struct gpu_m4_drm_provider_vm *vm,
	struct gpu_m4_drm_provider_version **version, struct gpu_m4_drm_vm_owner **owner);
/* object's bytes [offset, offset + size) are mapped with access by live
 * user or driver extents of the VM's tree (access proof without a row).
 */
bool gpu_m4_drm_provider_vm_object_mapped(struct gpu_m4_drm_provider_vm *vm,
	struct gpu_m4_drm_provider_object *object, u64 offset, u64 size, u32 access);
void gpu_m4_drm_provider_vm_guard_put(struct gpu_m4_drm_provider_vm *vm);
/* Named-range validation by PTE walk (gpu_m4_vm_tree_covers()). */
int gpu_m4_drm_provider_vm_covers(struct gpu_m4_drm_provider_vm *vm, u64 va, u64 size,
	u32 access, struct gpu_m4_vm_tree_span *span);
/* The VM's ASID is fixed at its first queue: removals invalidate it. */
void gpu_m4_drm_provider_vm_set_asid_locked(struct gpu_m4_drm_provider_vm *vm, u32 asid);
/* Native unit client-VA maps as DRIVER extents of the authority tree: the
 * mapping itself (failure returned), and its removal with TLBI (maps no
 * longer present are skipped). mapped: all present exactly.
 */
int gpu_m4_drm_provider_vm_native_map_locked(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_provider_private_gem *maps, u32 count);
int gpu_m4_drm_provider_vm_native_unmap_locked(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_provider_private_gem *maps, u32 count);
bool gpu_m4_drm_provider_vm_native_mapped(struct gpu_m4_drm_provider_vm *vm,
	const struct gpu_m4_drm_provider_private_gem *maps, u32 count);
/* Whether version is an authority VM's TREE version. */
bool gpu_m4_drm_provider_version_live(const struct gpu_m4_drm_provider_version *version);

/* Held reference to the current baseline (NULL if none/closed). */
struct gpu_m4_drm_provider_baseline *gpu_m4_drm_provider_vm_baseline_get_locked(
	struct gpu_m4_drm_provider_vm *vm);
void gpu_m4_drm_provider_baseline_release_locked(struct gpu_m4_drm_provider_baseline *baseline);
u64 gpu_m4_drm_provider_baseline_revision(const struct gpu_m4_drm_provider_baseline *baseline);
/* Every descriptor is an exact member (object, claim, VA, offset, size, access). */
bool gpu_m4_drm_provider_baseline_contains_locked(
	const struct gpu_m4_drm_provider_baseline *baseline,
	const struct gpu_m4_drm_provider_private_gem *maps, u32 count);
/*
 * REMOVE-DEBUG: who still holds the native FACTORY backing (private GEM
 * charge) of registered provider objects, for capacity diagnosis only. A
 * census takes a fresh stamp; the caller first marks the objects its own live
 * wrappers hold (CALLER), then every other registered object counts once, in
 * the first class below that holds it. GARBAGE: zero-ref objects whose
 * outside-lock release is still queued. Backing pinned only by snapshot or
 * cache pins has no provider object at all and is not counted here. Provider
 * mutex held; no reference, flag or state changes beyond census scratch.
 */
enum gpu_m4_drm_provider_holder {
	GPU_M4_DRM_PROVIDER_HOLDER_CALLER,
	GPU_M4_DRM_PROVIDER_HOLDER_BASELINE,	/* its VM's current native baseline */
	GPU_M4_DRM_PROVIDER_HOLDER_COMPOSED,	/* old baseline of an unfinished composition */
	GPU_M4_DRM_PROVIDER_HOLDER_FINISHED,	/* only old baselines of finished images */
	GPU_M4_DRM_PROVIDER_HOLDER_VERSION,	/* some version's own inventory */
	GPU_M4_DRM_PROVIDER_HOLDER_OTHER,	/* stage extras, timestamp aliases, staging */
	GPU_M4_DRM_PROVIDER_HOLDER_GARBAGE,
	GPU_M4_DRM_PROVIDER_HOLDERS,
};

struct gpu_m4_drm_provider_holders {
	u64 bytes[GPU_M4_DRM_PROVIDER_HOLDERS];
	u32 objects[GPU_M4_DRM_PROVIDER_HOLDERS];
	/* Live versions holding a baseline (composed), of those finished (image
	 * kept only until residency moves on), and RETIRED records; cumulative
	 * released images, member objects they dropped, releases skipped.
	 */
	u32 composed, finished, retired;
	u64 images_released, image_objects, image_failures;
};

u64 gpu_m4_drm_provider_census_begin_locked(struct gpu_m4_drm_provider *provider);
void gpu_m4_drm_provider_object_census_locked(struct gpu_m4_drm_provider_object *object,
	u64 stamp);
void gpu_m4_drm_provider_holders_locked(struct gpu_m4_drm_provider *provider, u64 stamp,
	struct gpu_m4_drm_provider_holders *holders);
/* A child's version: source's exact held user map (no recapture of a newer
 * VM_BIND) and common private baseline plus every member of the held
 * baseline, mapping-only. Snapshot = zero-extra clone of the source capture.
 * SNAPSHOT state (graph via ensure_graph). -ESTALE: not composable (e.g. a
 * per-child-extended source); keep the per-child append path.
 */
struct gpu_m4_drm_provider_version *gpu_m4_drm_provider_version_compose_locked(
	struct gpu_m4_drm_provider_version *source, struct gpu_m4_drm_provider_baseline *baseline);
/* Selected descriptors are exact members of the candidate's held baseline and
 * the captured (joined) cache/resv rows cover each range/permission/pin.
 */
bool gpu_m4_drm_provider_version_selected_covered_locked(
	struct gpu_m4_drm_provider_version *candidate,
	const struct gpu_m4_drm_provider_private_gem *maps, u32 count,
	const struct gpu_m4_drm_vm_owner *captured);

/* Atomic ioctl-acceptance capture under the shared mutex. Both outputs are
 * owned. Owner is a NEW unpublished clone for this job's cache/resv lifecycle;
 * version retains its original immutable snapshot independently of job fences.
 * Late timestamp extras extend this clone, NEVER reread current VM_BIND state.
 * Drop the job's clone/cache owners BEFORE its final version reference. This
 * preserves aggregate accounting even when another thread collects garbage.
 */
int gpu_m4_drm_provider_vm_capture_locked(struct gpu_m4_drm_provider_vm *vm,
	struct gpu_m4_drm_provider_version **version, struct gpu_m4_drm_vm_owner **owner);
/* The same capture WITHOUT the shared mutex, of exactly the active version a
 * locked capture last published (snapshot built, GRAPH identity checked) and
 * no active change, graph installation, close or fault since. Identical
 * outputs and ownership. -EAGAIN (nothing held) means take the locked path.
 */
int gpu_m4_drm_provider_vm_capture(struct gpu_m4_drm_provider_vm *vm,
	struct gpu_m4_drm_provider_version **version, struct gpu_m4_drm_vm_owner **owner);
/* New PRIVATE admission only. Prove registered VM/version membership BEFORE
 * dereferencing either supplied identity, exact provider/frontend/snapshot
 * generation and phase-ready artifacts, held refs and open healthy VM. A
 * logical owner is not a hardware/root grant. A retained historical candidate
 * need not be active. This is not accepted-job close/retirement validation.
 */
int gpu_m4_drm_provider_version_check_origin_locked(struct gpu_m4_drm_provider *provider,
	struct gpu_m4_drm_provider_vm *vm, struct gpu_m4_drm_provider_version *candidate);
/* Read-only host reclamation precheck, NOT admission: the same registered
 * exact identity proof without requiring an open VM, then -EBUSY while the
 * version is pending installation, resident, protected or working. It grants
 * no hardware authority and never clears residency. Caller holds the mutex.
 */
int gpu_m4_drm_provider_version_quiescent_locked(struct gpu_m4_drm_provider *provider,
	struct gpu_m4_drm_provider_vm *vm, struct gpu_m4_drm_provider_version *candidate);
/* Caller owns a checked exact version. Before graph construction, bounded
 * READ/WRITE spans use its validated immutable ledger and strongly held private
 * manifest, with the exact native/owner oracle. A graph-ready version uses its
 * complete actual UAT. Neither logical span checks nor a missing UAT grant
 * hardware/root authority. No publication, references or payload access.
 */
int gpu_m4_drm_provider_version_range_locked(struct gpu_m4_drm_provider_version *version,
	u64 va, u64 size, u32 access);
/* Every page of a retained GRAPH version's [va, va + size) with exactly the
 * version_page_locked() checks, in one run (O(manifest blocks) lookups plus
 * sequential leaves); fn sees each installed leaf. -ENODATA if not a GRAPH.
 */
int gpu_m4_drm_provider_version_walk_locked(struct gpu_m4_drm_provider_version *version,
	u64 va, u64 size, u32 access, gpu_m4_drm_uat_leaf_fn fn, void *cookie);
/* Span admission memo of this GRAPH version's exact immutable manifest (shared
 * by its image/use wrappers); see gpu_m4_drm_uat_span_key.
 */
bool gpu_m4_drm_provider_version_span_cached_locked(struct gpu_m4_drm_provider_version *version,
	const struct gpu_m4_drm_uat_span_key *key);
void gpu_m4_drm_provider_version_span_remember_locked(struct gpu_m4_drm_provider_version *version,
	const struct gpu_m4_drm_uat_span_key *key);
/* Installed pages at [va, va + size) map exactly identity's held object pages
 * from offset (PA, identity, object offset; exact also RW-equal access and NC
 * policy). The object's live page/session gate runs for the first page every
 * call; the complete walk once per immutable manifest (span memo).
 */
int gpu_m4_drm_provider_version_object_locked(struct gpu_m4_drm_provider_version *version,
	u64 va, struct gpu_m4_drm_provider_object *object, const void *identity,
	u64 offset, u64 size, u32 access, bool exact);
/* Exact detached candidate capture BEFORE decode/cache/reservation preparation.
 * No current-active substitution. Both outputs owned, NULL on any failure.
 */
int gpu_m4_drm_provider_version_capture_locked(struct gpu_m4_drm_provider_version *candidate,
	struct gpu_m4_drm_provider_version **version, struct gpu_m4_drm_vm_owner **owner);
/* Final pre-cache staging boundary, never a root-publication shortcut. Capture
 * may return a snapshot-only version; this constructs/verifies the exact held
 * version's graph before roots, native coverage or runtime acceptance. Failed
 * detached graphs retain charged rollback owners for outside-lock reap.
 */
int gpu_m4_drm_provider_version_ensure_graph_locked(
	struct gpu_m4_drm_provider_version *version);
/* Same result WITHOUT the caller holding the provider mutex: short locked
 * prepare/commit, pure construction detached. A caller-held reference keeps
 * version (and the optional same-lineage GRAPH source, whose unchanged
 * branches it shares) alive. An already built GRAPH returns its constructed
 * identity check. A later ensure_graph_locked() then needs no construction.
 */
int gpu_m4_drm_provider_version_ensure_graph(struct gpu_m4_drm_provider_version *version,
	struct gpu_m4_drm_provider_version *source);
/* All candidate native cache/resv accesses must already be covered by the
 * captured snapshot's exact pins, object identities, ranges and permissions.
 * User-lineage equality alone is insufficient for post-preparation adoption.
 */
bool gpu_m4_drm_provider_version_native_covered_locked(
	struct gpu_m4_drm_provider_version *candidate, const struct gpu_m4_drm_vm_owner *captured);
int gpu_m4_drm_provider_version_get_locked(struct gpu_m4_drm_provider_version *version);
/* Nonfinal drops never release backing. A final protected drop is -EBUSY and
 * leaves the owner reachable/charged. Final unprotected drops queue garbage.
 */
int gpu_m4_drm_provider_version_put_locked(struct gpu_m4_drm_provider_version *version);
u64 gpu_m4_drm_provider_version_generation(struct gpu_m4_drm_provider_version *version);
/* Checked root getter: GRAPH state, live reference, retained root allocation
 * and the descriptor recorded when the immutable graph was built/verified. No
 * leaf walk; live TTBAT pair/ASID and binding epoch remain the caller's checks.
 */
int gpu_m4_drm_provider_version_roots_locked(struct gpu_m4_drm_provider_version *version,
	u64 *low, u64 *high);
/* Refresh the retained clean table pages before strict live health inspection.
 * Does not accept observed differences into the manifest or grant root adoption.
 * Consumes the graph's construction proof: every named (shared or fresh) node's
 * retained identity and its creator's one-time proof, plus the refreshed live
 * root page. A binding change (begin/complete/unbind) only selects another
 * immutable graph and never forces a rescan. The first call after a provider
 * fault, or after any failed check/scan, rescans the complete manifest. Never a
 * completion/success grant, and no scan ever creates a node proof.
 */
int gpu_m4_drm_provider_version_refresh_locked(struct gpu_m4_drm_provider_version *version);
/* Always the complete invalidate+manifest scan: explicit fault/diagnostic
 * verification. Binding transitions use the ordinary refresh above.
 */
int gpu_m4_drm_provider_version_refresh_full_locked(struct gpu_m4_drm_provider_version *version);
enum gpu_m4_drm_provider_evidence_flags {
	GPU_M4_DRM_PROVIDER_EVIDENCE_CLOSED = 1U << 0,
	GPU_M4_DRM_PROVIDER_EVIDENCE_PROTECTED = 1U << 1,
	GPU_M4_DRM_PROVIDER_EVIDENCE_WORKING = 1U << 2,
};

struct gpu_m4_drm_provider_version_evidence {
	struct gpu_m4_drm_uat_evidence uat;
	u64 binding_generation;
	int provider_error;
	u32 flags;
};

/* Exact retained owner only, never vm->active substitution. Unknown/foreign
 * VM/version identities and zero-ref queued garbage are rejected before
 * dereferencing the supplied identity. Closed, faulted and quarantined owners
 * remain diagnosable when their complete clean owned table storage is safe.
 * Uses the bounded caller-owned copy contract above; no refs/state are changed.
 * A successful diagnostic export is NOT a runtime health/retirement witness.
 */
int gpu_m4_drm_provider_version_export_locked(struct gpu_m4_drm_provider *provider,
	struct gpu_m4_drm_provider_vm *vm, struct gpu_m4_drm_provider_version *version,
	struct gpu_m4_drm_provider_version_evidence *header,
	struct gpu_m4_drm_uat_table_evidence *tables, u32 capacity);
int gpu_m4_drm_provider_version_page_locked(struct gpu_m4_drm_provider_version *version,
	u64 va, u32 access, struct gpu_m4_drm_vm_leaf *leaf);

/* Build from the CAPTURED immutable version, never the current active VM. A
 * zero-count NULL extension rebuilds the identical complete manifest with new
 * owned table pages; it does not adopt or publish any hardware root. Keep
 * identical user leaves/generation and persistent context; retain every prior
 * private lease and append the supplied owned mappings in [7000000000,8000000000).
 * Result owns distinct detached tables. Caller owns the result on success; on
 * failure the source and active VM stay unchanged, deferred garbage remains
 * charged until reap. Source can remain queued/resident or outlive VM close.
 * Private callbacks prove provider/VM/identity and retain their actual allocator
 * backing/accounting owner. Data is charged by that allocator, not again per
 * alias/version here. Get/page run under the shared mutex; put runs OUTSIDE all
 * provider/reservation locks, including builder rollback and final collection.
 * No root/table/ring publication occurs.
 */
struct gpu_m4_drm_provider_version *gpu_m4_drm_provider_version_extend_private_locked(
	struct gpu_m4_drm_provider_version *source,
	const struct gpu_m4_drm_uat_private *private, u32 count);
/* Typed native leaves stay OUT of the ordinary user ledger, but join the
 * retained native inventory and snapshot cache/resv extras. Zero-count rebuild
 * preserves the complete manifest, native accesses, leases and user lineage.
 * Callback put and final releases remain outside all provider/resv locks.
 */
struct gpu_m4_drm_provider_version *gpu_m4_drm_provider_version_extend_private_gem_locked(
	struct gpu_m4_drm_provider_version *source,
	const struct gpu_m4_drm_provider_private_gem *private, u32 count);
/* Retained user-generation lineage + exact leaves, not numeric generation or
 * a mutable VM lookup. For prepublication job adoption of a private extension.
 */
bool gpu_m4_drm_provider_version_same_user_locked(
	struct gpu_m4_drm_provider_version *first, struct gpu_m4_drm_provider_version *second);

/* Exact unchanged-image adoption eligibility, metadata only, no MMIO. True
 * ONLY when old is the current resident of its ASID (no pending transaction,
 * no working consumer), candidate is a registered, open, unprotected, unbound
 * GRAPH of the same VM with its own separate UAT wrapper, and both wrappers
 * name the same retained root/node image and the same complete user and
 * private leaf/owner tuples. Equal counts or equal root descriptors alone
 * never qualify. The caller still performs its live TTBAT/root/session checks
 * and then asid_adopt: the resident owner moves to candidate without TTBAT
 * stores, handoff, TLBI or a binding generation change, because no table or
 * translation changes.
 */
bool gpu_m4_drm_provider_version_same_image_locked(struct gpu_m4_drm_provider *provider,
	struct gpu_m4_drm_provider_version *old, struct gpu_m4_drm_provider_version *candidate);

/* Root publisher interface, with NO MMIO/TLBI here. Pairs are the graphs'
 * ASID-free root descriptors (version_roots); the caller owns the TTBAT words
 * and their ASID field. Begin holds BOTH previous and candidate roots before
 * any possible write and checks the caller's exact expected pair; only this
 * ASID's resident must be quiescent. Complete may be called ONLY after
 * qualified handoff, exact readback and completed invalidation of that ASID,
 * and advances its generation. Ambiguous error latches provider failure and
 * keeps both roots indefinitely. A signaled fence proves nothing.
 */
int gpu_m4_drm_provider_asid_bind_begin_locked(struct gpu_m4_drm_provider *provider, u32 asid,
	struct gpu_m4_drm_provider_version *version, u64 expected_low, u64 expected_high);
int gpu_m4_drm_provider_asid_bind_complete_locked(struct gpu_m4_drm_provider *provider,
	u32 asid, struct gpu_m4_drm_provider_version *version, u64 observed_low,
	u64 observed_high);
int gpu_m4_drm_provider_bind_begin_locked(struct gpu_m4_drm_provider *provider,
	struct gpu_m4_drm_provider_version *version, u64 expected_low, u64 expected_high);
int gpu_m4_drm_provider_bind_complete_locked(struct gpu_m4_drm_provider *provider,
	struct gpu_m4_drm_provider_version *version, u64 observed_low, u64 observed_high);
/* Same-image adoption (see same_image): the observed pair must be the
 * resident's own roots. Moves residency to version in one step and RETAINS
 * the binding generation; any failure after the move began latches provider
 * failure and keeps both graphs.
 */
int gpu_m4_drm_provider_asid_adopt_locked(struct gpu_m4_drm_provider *provider, u32 asid,
	struct gpu_m4_drm_provider_version *version, u64 observed_low, u64 observed_high);
/* Qualified idle unbind + ASID invalidation, not a fault-reset shortcut. The
 * ASID's generation is kept for its next installation.
 */
int gpu_m4_drm_provider_asid_unbind_complete_locked(struct gpu_m4_drm_provider *provider,
	u32 asid, u64 observed_low, u64 observed_high);
int gpu_m4_drm_provider_unbind_complete_locked(struct gpu_m4_drm_provider *provider,
	u64 observed_low, u64 observed_high);
int gpu_m4_drm_provider_version_begin_work_locked(struct gpu_m4_drm_provider_version *version);
/* A composed (baseline-holding) version's retirement also records its one
 * job as finished: its image and member holds are released at the binding or
 * unbind that replaces it as the resident root. A RETIRED record keeps its
 * identity, snapshot and source inventory for retained receipts only; it is
 * never bound, captured, composed or rebuilt.
 */
int gpu_m4_drm_provider_version_retire_locked(struct gpu_m4_drm_provider_version *version);

/* Always call without provider mutex AND without any BO reservation lock.
 * Final UAT/snapshot/native-pin/context puts run here, not in *_locked hooks.
 * Healthy queued versions survive rebinding, unbinding and frontend close.
 * After a refused charge it first drops every VM's optional construction
 * cache (idle, never authority), so pressure reclaims before a retry.
 */
void gpu_m4_drm_provider_reap(struct gpu_m4_drm_provider *provider);

#endif
