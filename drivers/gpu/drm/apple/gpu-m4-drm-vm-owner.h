/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_DRM_VM_OWNER_H
#define APPLE_GPU_M4_DRM_VM_OWNER_H

#include "gpu-m4-drm-memory.h"
#include "gpu-m4-drm-resv.h"
#include "gpu-m4-drm-vm-ledger.h"

struct mutex;
struct gpu_m4_drm_vm_owner;
struct gpu_m4_drm_vm_ledger_index;

/*
 * Accept under the provider lock which serializes actual bind/unbind. Lookup
 * returns borrowed, already retained native pins, not asynchronous GEM handles.
 * The ledger's mandatory oracle first proves exclusive qualified ownership.
 * Copy its exact VA/access/backing generation and retain all unique BO pins and
 * the frontend VM before returning. No cache operation or PTE write occurs.
 *
 * Extra accesses include timestamp BOs outside ordinary user VA mappings. They
 * must name borrowed native pins from the same retained provider; their real FW
 * alias/mapping leases, queue owner and UAT table versions are separately held by
 * the provider. Neither attachments nor this software ledger is that authority.
 */
struct gpu_m4_drm_vm_owner *gpu_m4_drm_vm_owner_create(
	struct gpu_m4_drm_vm *vm, struct mutex *vm_lock,
	const struct gpu_m4_drm_vm_ledger *active, u64 generation,
	void *cookie,
	struct gpu_m4_drm_bo_pin *(*lookup)(void *cookie, const void *identity),
	const struct gpu_m4_drm_cache_access *extra, u32 extra_count);
/*
 * As create(), with the source's retained VA/PA/identity index. That index is
 * the caller's validated authority for the exact ledger (built by the full
 * validation or a proven delta under the same oracle), so the whole-ledger
 * alias proof is copied, not repeated. Every leaf PA is still proven against
 * the snapshot's own retained pin, one identity lookup per object.
 */
struct gpu_m4_drm_vm_owner *gpu_m4_drm_vm_owner_create_indexed(
	struct gpu_m4_drm_vm *vm, struct mutex *vm_lock,
	const struct gpu_m4_drm_vm_ledger *active,
	const struct gpu_m4_drm_vm_ledger_index *index, u64 generation,
	void *cookie,
	struct gpu_m4_drm_bo_pin *(*lookup)(void *cookie, const void *identity),
	const struct gpu_m4_drm_cache_access *extra, u32 extra_count);

/*
 * Decoder acceptance snapshots user VA before acquiring explicit timestamp
 * leases. Join those late owned pins by creating a NEW unpublished owner from
 * the retained source. No VM lock, current binding/handle lookup or original
 * provider oracle is used. Source must be live, unpublished and unretired; the
 * caller serializes its use against source release/publication. Transactional
 * failure leaves source unchanged. New owner retains all original backing and
 * frontend VM plus extras, with identical VA leaves and generation. The caller
 * keeps the new owner for the final cache/resv set and releases the old private
 * owner only after dropping every referenced reservation lock.
 *
 * The new owner SHARES the source's immutable leaves, indexes and (without
 * extras) its pin inventory; only its publication/retirement state is its own.
 * Extras build one new immutable inventory: same-pin joins are bounds/access
 * unions, another pin of one identity must prove identical backing.
 */
struct gpu_m4_drm_vm_owner *gpu_m4_drm_vm_owner_clone_with_extras(
	const struct gpu_m4_drm_vm_owner *source,
	const struct gpu_m4_drm_cache_access *extra, u32 extra_count);

u64 gpu_m4_drm_vm_owner_generation(const struct gpu_m4_drm_vm_owner *owner);
/* Bytes this wrapper and its inventory set retain; with map, also the shared
 * immutable map (leaves + retained index) it keeps alive. For accounting.
 */
u64 gpu_m4_drm_vm_owner_bytes(const struct gpu_m4_drm_vm_owner *owner, bool map);
/* Snapshot references, not job retirement or residency references. Get while
 * holding the caller's provider lock; final release still refuses uncertainty.
 * Indexed leaves are COPIES. No mutable PA/leaf array escapes the snapshot.
 */
int gpu_m4_drm_vm_owner_get(struct gpu_m4_drm_vm_owner *owner);
/* Version construction holds an immutable, not job-published source. This
 * atomic-under-caller-lock gate guarantees prepublication rollback can put it.
 */
int gpu_m4_drm_vm_owner_get_unpublished(struct gpu_m4_drm_vm_owner *owner);
u32 gpu_m4_drm_vm_owner_leaf_count(const struct gpu_m4_drm_vm_owner *owner);
int gpu_m4_drm_vm_owner_leaf(const struct gpu_m4_drm_vm_owner *owner,
			   u32 index, struct gpu_m4_drm_vm_leaf *leaf);
/* Borrowed identity remains retained by this snapshot, including after close. */
struct gpu_m4_drm_vm *gpu_m4_drm_vm_owner_vm(const struct gpu_m4_drm_vm_owner *owner);
/* Exact snapshot VA permissions, not unioned alias permissions. Byte spans may
 * cross adjacent mappings or SINGLE_PAGE aliases; every covered leaf is checked.
 */
int gpu_m4_drm_vm_owner_range(const struct gpu_m4_drm_vm_owner *owner,
			    u64 va, u64 size, u32 access);
int gpu_m4_drm_vm_owner_page(const struct gpu_m4_drm_vm_owner *owner,
			   u64 va, u32 access, struct gpu_m4_drm_vm_leaf *leaf);

/* Immutable borrowed arrays, valid until release. Same whole-VM + extra BO set
 * feeds implicit dma-resv snapshots and WB cache handling, with unioned access
 * and a conservative per-object span. No optional attachment completeness claim.
 */
const struct gpu_m4_drm_cache_access *gpu_m4_drm_vm_owner_cache_access(
	const struct gpu_m4_drm_vm_owner *owner, u32 *count);
const struct gpu_m4_drm_resv_access *gpu_m4_drm_vm_owner_resv_access(
	const struct gpu_m4_drm_vm_owner *owner, u32 *count);
/* Index (into both arrays above) of the row naming native object identity, or
 * U32_MAX: O(log n) over the set's immutable identity order, which holds one
 * paired cache/resv row per identity. A lookup hint only: a caller's row
 * predicate that refuses this row keeps its complete scan, so every answer
 * equals the scan's.
 */
u32 gpu_m4_drm_vm_owner_find(const struct gpu_m4_drm_vm_owner *owner, const void *identity);

/* Caller serializes these transitions. Mark before first hardware-visible write;
 * retirement means the complete qualified predicate, never an error fence.
 * Release uncertain owners returns -EBUSY without dropping any backing. Keep
 * owner, FW/table leases and cache job on the retained uncertain-owner list.
 * Final release must run with ALL referenced BO reservation locks dropped.
 */
int gpu_m4_drm_vm_owner_publish(struct gpu_m4_drm_vm_owner *owner);
int gpu_m4_drm_vm_owner_retire(struct gpu_m4_drm_vm_owner *owner);
int gpu_m4_drm_vm_owner_release(struct gpu_m4_drm_vm_owner *owner);

#endif
