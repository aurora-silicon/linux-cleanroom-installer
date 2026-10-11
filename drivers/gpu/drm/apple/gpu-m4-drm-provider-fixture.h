/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_DRM_PROVIDER_FIXTURE_H
#define APPLE_GPU_M4_DRM_PROVIDER_FIXTURE_H

#include "gpu-m4-drm-provider-vm.h"

struct gpu_m4_drm_provider_fixture;

/* PRIVATE one-shot fixture bounds, not userspace/renderer capabilities. */
#define GPU_M4_DRM_FIXTURE_MAX_NATIVE 16U
#define GPU_M4_DRM_FIXTURE_MAX_DEPENDENCIES 64U
#define GPU_M4_DRM_FIXTURE_MAX_LEAVES 64U
#define GPU_M4_DRM_FIXTURE_MAX_NATIVE_BYTES (16ULL << 20)

enum gpu_m4_drm_provider_fixture_state {
	GPU_M4_DRM_FIXTURE_STAGED,
	GPU_M4_DRM_FIXTURE_DEPENDENCIES,
	GPU_M4_DRM_FIXTURE_PREPARED,
	GPU_M4_DRM_FIXTURE_PROTECTED,
	GPU_M4_DRM_FIXTURE_CPU_VISIBLE,
	GPU_M4_DRM_FIXTURE_COMPLETE,
	GPU_M4_DRM_FIXTURE_FAULTED,
};

struct gpu_m4_drm_provider_fixture_config {
	struct gpu_m4_drm_provider *provider;
	/* Caller retains this EXACT candidate through stage; it belongs to provider.
	 * Never substitute a fresh/current VM snapshot or route fixed private CDM
	 * through the public user-VA command decoder.
	 */
	struct gpu_m4_drm_provider_version *version;
	/* Real powered/fixture DMA owner, including original work/pool/output and
	 * firmware-visible aliases. get/put run OUTSIDE provider/reservation locks.
	 * An accepted fixture retains this baseline until reboot; no public release.
	 */
	void *payload;
	int (*payload_get)(void *payload);
	void (*payload_put)(void *payload);
	/* Mandatory root-owned FULL hardware predicate under the exact GPU/provider
	 * mutex. 0 is qualified, -EAGAIN pending; ANY positive value is -EPROTO.
	 * Independent NC state only: never read uncertain WB output here.
	 */
	int (*check_retired_locked)(void *payload,
		struct gpu_m4_drm_provider_version *version);
	/* Called only AFTER cache invalidation and snapshot retirement. Validate
	 * actual output/canaries, and retire the actual begun version's work before
	 * returning 0. Negative failure or positive -EPROTO never grants success.
	 * These callbacks, NOT this scalar owner helper, qualify hardware behavior.
	 */
	int (*check_visible_locked)(void *payload,
		struct gpu_m4_drm_provider_version *version);
};

/* Stage outside provider AND all BO reservation locks. Every fallible private
 * acquisition/cache/fence step precedes resv_prepare; install the actual new
 * unsignaled finished fence immediately, then release every reservation lock.
 * Returns 0 + owned baseline. On ordinary rejection *out is NULL. If rollback
 * itself cannot drop an owner, return the error AND a FAULTED retained *out;
 * caller MUST retain a non-NULL output on any return value, until reboot.
 * No roots/tables/rings/cache payload or GPU publication occurs here.
 */
int gpu_m4_drm_provider_fixture_stage(
	const struct gpu_m4_drm_provider_fixture_config *config,
	struct gpu_m4_drm_provider_fixture **out);

/* Owned fence reference; it is exactly the fence installed into reservations,
 * never a fake completion promise. Drop this reference outside owner locks.
 */
struct dma_fence *gpu_m4_drm_provider_fixture_fence_get(
	struct gpu_m4_drm_provider_fixture *fixture);

/* All *_locked transitions hold the ONE configured provider/GPU mutex. They
 * never wait, acquire BO reservations or signal fences. Exact OLD dependency
 * references come from resv_prepare, never from another reservation query.
 * Dependencies return 0 ready, -EAGAIN pending, or a retained terminal error.
 */
int gpu_m4_drm_provider_fixture_dependencies_locked(
	struct gpu_m4_drm_provider_fixture *fixture);
int gpu_m4_drm_provider_fixture_prepare_locked(
	struct gpu_m4_drm_provider_fixture *fixture);
/* MUST succeed before any possible GPU-visible root/work/upper publication.
 * Root separately performs exact root bind and version_begin_work before kick.
 * Any failure from this protection point onward remains retained, never freed.
 */
int gpu_m4_drm_provider_fixture_protect_locked(
	struct gpu_m4_drm_provider_fixture *fixture);
/* Calls actual mandatory root predicate -> cache invalidate -> snapshot retire
 * -> actual visible validation/version retirement -> commit success-ready.
 * -EAGAIN is pending. This does not itself publish or retire GPU work.
 */
int gpu_m4_drm_provider_fixture_retire_locked(
	struct gpu_m4_drm_provider_fixture *fixture);
void gpu_m4_drm_provider_fixture_fault_locked(
	struct gpu_m4_drm_provider_fixture *fixture, int error);
/* Outside provider AND all referenced reservation locks. Consumes only a
 * previously committed success/error; cannot create retirement or success.
 */
int gpu_m4_drm_provider_fixture_signal(struct gpu_m4_drm_provider_fixture *fixture);

struct gpu_m4_drm_provider_fixture_view {
	struct gpu_m4_drm_provider_version *version; /* borrowed exact retained owner */
	enum gpu_m4_drm_provider_fixture_state state;
	u32 native_count, dependencies_consumed;
	int error;
	bool protected, cpu_visible;
};
void gpu_m4_drm_provider_fixture_view_locked(
	struct gpu_m4_drm_provider_fixture *fixture,
	struct gpu_m4_drm_provider_fixture_view *view);

#endif
