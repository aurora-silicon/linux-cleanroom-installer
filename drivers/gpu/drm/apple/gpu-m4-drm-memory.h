/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_DRM_MEMORY_H
#define APPLE_GPU_M4_DRM_MEMORY_H

#include "gpu-m4-drm.h"

struct gpu_m4_drm_bo_pin;
struct gpu_m4_drm_cache_job;
struct gpu_m4_drm_vm_ledger;

/*
 * Local shmem only. The returned owner retains GEM and a shmem pin. All callers
 * must obtain their object from this frontend; arbitrary GEM/PRIME objects are
 * not accepted. These helpers do not publish PTEs or advertise WB coherency.
 */
struct gpu_m4_drm_bo_pin *gpu_m4_drm_bo_pin_locked(struct drm_gem_object *object);
void gpu_m4_drm_bo_pin_get(struct gpu_m4_drm_bo_pin *pin);
/* A final put locks object->resv to unpin: never hold that reservation here. */
void gpu_m4_drm_bo_pin_put(struct gpu_m4_drm_bo_pin *pin);
/* Stable object identity, not the address of a particular pin acquisition. */
struct drm_gem_object *gpu_m4_drm_bo_pin_object(struct gpu_m4_drm_bo_pin *pin);
u64 gpu_m4_drm_bo_pin_size(struct gpu_m4_drm_bo_pin *pin);
int gpu_m4_drm_bo_pin_page(struct gpu_m4_drm_bo_pin *pin, u64 offset, u64 *pa);
int gpu_m4_drm_bo_pin_range(struct gpu_m4_drm_bo_pin *pin, u64 offset, u64 size);
int gpu_m4_drm_bo_pin_same_range(struct gpu_m4_drm_bo_pin *a, struct gpu_m4_drm_bo_pin *b,
	u64 offset, u64 size);

struct gpu_m4_drm_cache_access {
	struct gpu_m4_drm_bo_pin *pin;
	u64 offset, size; /* potentially accessed bytes; maintenance is full-page */
	u32 access; /* DRM_ASAHI_BIND_READ and/or DRM_ASAHI_BIND_WRITE */
};

/*
 * Own a copy and retain every backing before prepare returns. Reservation
 * locks/fences and VM/VA alias references remain the provider's responsibility.
 * No cache operation runs here, while input fences may still be outstanding.
 * Zero rows (access may be NULL) is an ordering-only job: prepare/retire still
 * order with dma_wmb()/dma_rmb(); every access check refuses.
 */
struct gpu_m4_drm_cache_job *
gpu_m4_drm_cache_job_create(const struct gpu_m4_drm_cache_access *access, u32 count);
/*
 * Conservative capture of ALL potentially accessible VM BOs, not optional
 * attachment hints. The VM lock is held and lookup returns borrowed existing
 * pins by stable local GEM identity; this call retains them before returning.
 * The provider separately retains every mapping/table version and snapshots old
 * dma-resv fences under ww locks before publishing the scheduler finished fence.
 */
struct gpu_m4_drm_cache_job *gpu_m4_drm_cache_job_snapshot_vm(
	const struct gpu_m4_drm_vm_ledger *ledger, void *cookie,
	struct gpu_m4_drm_bo_pin *(*lookup)(void *cookie, const void *identity));
/* Only after ALL explicit/implicit inputs: ordering barrier; unadmitted WB pins
 * clean reads and clean/inval writes. Admitted local native WB pins use the
 * coherent shared GPU memory class and need no payload sweep.
 */
int gpu_m4_drm_cache_job_prepare(struct gpu_m4_drm_cache_job *job);
/* Validate and mark BEFORE the first visible write; failure means do not publish. */
int gpu_m4_drm_cache_job_publish(struct gpu_m4_drm_cache_job *job);
/*
 * Only after the complete hardware retirement predicate. Orders GPU outputs
 * (and invalidates unadmitted WB pins) before the provider reads them or
 * signals its real success fence.
 */
int gpu_m4_drm_cache_job_retire(struct gpu_m4_drm_cache_job *job);
/* Read-only proof for an exact retained pin and byte span. Both modes require
 * prepared AND published ownership; retired=false requires the still-live GPU
 * phase, retired=true requires completed cache retirement/CPU visibility.
 * Caller serializes job state. No page/payload read, maintenance, ref mutation,
 * publication or retirement is performed. Permission/range failure is -EACCES.
 */
int gpu_m4_drm_cache_job_check_access(const struct gpu_m4_drm_cache_job *job,
	struct gpu_m4_drm_bo_pin *pin, u64 offset, u64 size, u32 access, bool retired);
/*
 * Explicit sync: an exact pin with no row in this job, whose span/access the
 * caller proved from the job's own retained snapshot. Same phase contract as
 * check_access. Only an admitted pin (shared coherent GPU class: neither
 * prepare nor retire ever sweeps its payload) needs nothing beyond this job's
 * ordering barriers; an unadmitted pin needs its own row (-EACCES).
 */
int gpu_m4_drm_cache_job_check_coherent(const struct gpu_m4_drm_cache_job *job,
	struct gpu_m4_drm_bo_pin *pin, bool retired);
/*
 * Unpublished or qualified-retired only. A published unretired/error job returns
 * -EBUSY and retains every pin; error-fence signaling is NOT release authority.
 * Retain that job on the provider's uncertain-owner list until qualified halt.
 * The helper never substitutes generic RTKit reset or a guessed halt.
 * Release after dropping ALL referenced BO reservation locks (final puts lock).
 */
int gpu_m4_drm_cache_job_release(struct gpu_m4_drm_cache_job *job);

#endif
