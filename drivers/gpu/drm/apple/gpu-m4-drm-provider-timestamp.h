/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_DRM_PROVIDER_TIMESTAMP_H
#define APPLE_GPU_M4_DRM_PROVIDER_TIMESTAMP_H

#include "gpu-m4-drm.h"

struct mutex;
struct gpu_m4_drm_provider_timestamp_cookie;

struct gpu_m4_drm_provider_timestamp_config {
	struct gpu_m4_drm *frontend;
	void *owner;
	struct mutex *owner_lock;
	/* Retain the exact frontend/READY/power/session baseline. These callbacks
	 * run outside GPU/reservation locks and may acquire owner_lock themselves.
	 */
	int (*session_get)(void *owner);
	void (*session_put)(void *owner);
};

/* BIND_OBJECT is file-global: no queue VM, pin, CPU view or hardware alias is
 * created here. The caller retains config/frontend/object across construction.
 * Return one owned HANDLE reference, or ERR_PTR without any retained state.
 */
struct gpu_m4_drm_provider_timestamp_cookie *gpu_m4_drm_provider_timestamp_bind(
	const struct gpu_m4_drm_provider_timestamp_config *config,
	struct drm_gem_object *object, u64 offset, u64 range);

/* Accepting callbacks get atomically while the borrowed lookup is serialized,
 * BEFORE returning to the frontend. Jobs must never look up numeric handles
 * again. Unbind drops only the handle ref; captured job refs remain valid.
 */
bool gpu_m4_drm_provider_timestamp_get(struct gpu_m4_drm_provider_timestamp_cookie *cookie);
/* Caller already owns a cookie reference. Pure exact configuration identity:
 * frontend, owner, mutex and BOTH session callbacks must all match. No get,
 * put, payload access or work-admission/protection authority is implied.
 */
bool gpu_m4_drm_provider_timestamp_matches(struct gpu_m4_drm_provider_timestamp_cookie *cookie,
	const struct gpu_m4_drm_provider_timestamp_config *expected);
/* All puts run outside GPU/reservation locks. Final GEM/session put may acquire
 * those locks; defer a runtime-locked drop to its actual outside-lock reap path.
 * A job may drop its ref only after proven lifetime or unpublished cancellation.
 */
void gpu_m4_drm_provider_timestamp_put(struct gpu_m4_drm_provider_timestamp_cookie *cookie);

/* Caller owns a cookie ref. Pure eight-byte request validation; output is
 * unchanged on error. Returned fields are borrowed under that retained ref.
 * object_offset includes the BIND_OBJECT offset, not merely request offset.
 */
int gpu_m4_drm_provider_timestamp_slice(struct gpu_m4_drm_provider_timestamp_cookie *cookie,
	u64 relative_offset, struct gpu_m4_drm_timestamp *timestamp);
u64 gpu_m4_drm_provider_timestamp_bound_offset(struct gpu_m4_drm_provider_timestamp_cookie *cookie);
u64 gpu_m4_drm_provider_timestamp_bound_range(struct gpu_m4_drm_provider_timestamp_cookie *cookie);
enum gpu_m4_drm_cpu_policy gpu_m4_drm_provider_timestamp_policy(
	struct gpu_m4_drm_provider_timestamp_cookie *cookie);

#endif
