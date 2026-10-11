/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_DRM_RESV_H
#define APPLE_GPU_M4_DRM_RESV_H

#include "gpu-m4-drm.h"

struct gpu_m4_drm_resv_job;

struct gpu_m4_drm_resv_access {
	struct drm_gem_object *object;
	u32 access; /* unioned GPU READ/WRITE permissions, not attachment hints */
};

/*
 * Synchronous prepare: snapshot/ref all potentially accessible local BOs plus
 * special timestamp/copy BOs. Lock via drm_exec ww discipline, reserve every
 * finished-fence slot, capture exact OLD implicit dependency references. No GPU
 * publication or fence waits. Returns with all reservations LOCKED; the provider
 * may finish fallible private ownership work before job_arm. On failure all
 * helper locks/refs are gone. PRIME is an explicit unsupported feature gate.
 */
struct gpu_m4_drm_resv_job *gpu_m4_drm_resv_prepare(
	const struct gpu_m4_drm_resv_access *access, u32 count);
/*
 * Bounded internal execution inherits an actual accepted public parent's OLD
 * implicit inputs, never re-enumerating common reservations after its public
 * finished fence was installed. Caller holds the actual parent job/resv while
 * preparing, outside provider/reservation locks. Parent must be published and
 * unlocked, with its exact finished fence still unsignaled.
 *
 * The full child inventory retains every common GEM with EXACT original
 * READ/WRITE permissions, plus this window's private native GEMs. Duplicate
 * objects/reservations are unioned normally; a private object sharing a common
 * reservation may not broaden that reservation's original access class.
 * Child owns independent GEM/fence refs and dependency cursors after return;
 * no parent-record reference is needed then. Common reservations are locked
 * but neither enumerated nor given another fence slot. Only new reservation
 * leaders are snapshotted/reserved and keep EVERY captured input. Private
 * native BOs own separate reservations carrying earlier child retirement
 * fences; finding this parent's public fence there is ambiguous sharing and
 * fails with -EBUSY instead of being filtered. Caller admits reused private native units
 * through actual CORE retirement/displacement proofs; this helper grants no
 * backing reuse, retirement or user-controlled reservation alias authority.
 * Returns with all child ww locks held.
 */
struct gpu_m4_drm_resv_job *gpu_m4_drm_resv_prepare_inherited(
	const struct gpu_m4_drm_resv_job *parent,
	const struct gpu_m4_drm_resv_access *access, u32 count);
/*
 * Infallible accepting step after job_arm: install borrowed actual scheduler
 * finished fence with READ or WRITE usage, then release EVERY ww lock. Never
 * query reservations again to obtain this job's inputs. The job/fence must be
 * valid and the fence unsignaled; this is software publication, not GPU
 * retirement. The helper retains its own finished-fence reference. Inherited
 * owners use publish_inherited instead.
 */
void gpu_m4_drm_resv_publish_finished(struct gpu_m4_drm_resv_job *job,
				    struct dma_fence *finished);
/*
 * Inherited child publication: install the child's OWN unsignaled retirement
 * fence (never the parent's public finished fence) on new kernel-private
 * leaders only, then release every ww lock. Common leaders already contain the
 * public fence and are untouched. The child never signals the public fence.
 */
void gpu_m4_drm_resv_publish_inherited(struct gpu_m4_drm_resv_job *job,
				     struct dma_fence *retirement);
/* Owned refs to retained old snapshots; called unlocked by next_dependency. */
struct dma_fence *gpu_m4_drm_resv_next_dependency(struct gpu_m4_drm_resv_job *job);
/*
 * Cancels private accepting state if still locked; drops locks BEFORE all fence
 * and GEM refs. This does not release GPU cache-job/VA/backing pins, which remain
 * subject to the provider's full retirement or uncertain-owner retention rule.
 */
void gpu_m4_drm_resv_release(struct gpu_m4_drm_resv_job *job);

#endif
