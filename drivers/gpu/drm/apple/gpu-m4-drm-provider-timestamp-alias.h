/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_DRM_PROVIDER_TIMESTAMP_ALIAS_H
#define APPLE_GPU_M4_DRM_PROVIDER_TIMESTAMP_ALIAS_H

#include "gpu-m4-drm-provider-job.h"
#include "gpu-m4-drm-provider-timestamp.h"
#include "gpu-m4-drm-provider-upper.h"

struct gpu_m4_drm_timestamp_aliases;
struct gpu_m4_drm_timestamp_alias_request;
struct gpu_m4_drm_timestamp_alias_token;

/* Hard metadata limits, not hardware qualification of every placement. The
 * supplied upper owner independently bounds its TS pages, leases and guards.
 */
#define GPU_M4_DRM_TIMESTAMP_ALIAS_MAX_ENTRIES 256U
#define GPU_M4_DRM_TIMESTAMP_ALIAS_MAX_PAGES 4096U
#define GPU_M4_DRM_TIMESTAMP_ALIAS_MAX_REQUESTS 4096U
#define GPU_M4_DRM_TIMESTAMP_ALIAS_MAX_TOKENS 64U
#define GPU_M4_DRM_TIMESTAMP_ALIAS_JOB_REQUESTS (64U * 12U)
#define GPU_M4_DRM_TIMESTAMP_ALIAS_MAX_ARMED 2U

struct gpu_m4_drm_timestamp_alias_config {
	struct gpu_m4_drm_provider *provider;
	struct gpu_m4_drm_upper *upper;
	struct gpu_m4_drm_provider_timestamp_config timestamp;
	void *owner;
	/* Actual retained READY/table/frontend/provider ownership, not a Boolean
	 * or opaque cast. timestamp.session_get retains this supplied upper and
	 * its complete hardware/session baseline while this registry is open.
	 * This scalar owner check must not recursively walk the supplied upper:
	 * upper inventory/backing checks themselves invoke it. The upper owner's
	 * own gate/graph checks independently prove actual table qualification.
	 */
	int (*owner_check_locked)(void *owner, struct gpu_m4_drm_provider *provider,
		struct gpu_m4_drm_upper *upper, struct gpu_m4_drm *frontend,
		struct mutex *lock);
	/* Mandatory exact active item/job/provider/session and exclusive healthy
	 * lane proof, using the actual lane API. Repeated on publication/reuse.
	 */
	int (*lane_check_locked)(void *owner, struct gpu_m4_drm_provider_job *job);
	u64 memory_limit; /* Registry metadata only; native objects remain charged. */
	u32 max_entries, max_pages, max_requests, max_tokens;
	/* Armed (in-flight) token cap, the owning lane's cap: zero means one. */
	u32 max_armed;
};

/* Outside provider/resv locks. No tables, CPU views or firmware are created. */
struct gpu_m4_drm_timestamp_aliases *gpu_m4_drm_timestamp_alias_create(
	const struct gpu_m4_drm_timestamp_alias_config *config);
/* Acceptance only, before cache/protection. Caller owns captured/object and
 * the serialized borrowed cookie. Success owns one request; outputs unchanged
 * on failure. Full-BO canonical identity is independent of cookie subranges.
 */
int gpu_m4_drm_timestamp_alias_hold_locked(struct gpu_m4_drm_timestamp_aliases *aliases,
	struct gpu_m4_drm_provider_version *captured, struct gpu_m4_drm_provider_object *object,
	const struct gpu_m4_drm_timestamp *borrowed,
	struct gpu_m4_drm_timestamp_alias_request **request, u64 *firmware_va);
/* Retain the SAME already captured request, without allocation/publication or
 * another cookie/upper/job reference. Caller retains the actual registry under
 * its provider mutex; membership precedes any supplied-request dereference.
 * Healthy historical/closed origins remain valid, but zero-ref/disposed and
 * foreign requests cannot be resurrected. Failure changes no owner refs.
 * This is not new-work admission or current-job cache/lane protection proof.
 */
int gpu_m4_drm_timestamp_alias_request_get_locked(struct gpu_m4_drm_timestamp_aliases *aliases,
	struct gpu_m4_drm_timestamp_alias_request *request);
void gpu_m4_drm_timestamp_alias_request_put_locked(struct gpu_m4_drm_timestamp_alias_request *request);
/* Synchronous accept only: allocate bounded storage and retain requests, but
 * NEVER get the borrowed job here. The runtime job owns this token until its
 * cancellation or actual successful completion notification.
 */
int gpu_m4_drm_timestamp_alias_token_prepare_locked(struct gpu_m4_drm_timestamp_aliases *aliases,
	struct gpu_m4_drm_provider_job *job, struct gpu_m4_drm_timestamp_alias_token **token);
int gpu_m4_drm_timestamp_alias_token_cancel_locked(struct gpu_m4_drm_timestamp_alias_token *token);
/* From protected runtime.publish, after dependencies/cache/snapshot publish.
 * Arm and all later entrances allocate nothing. ANY publication error keeps
 * the actual job/request/cookie/anchor-version owners indefinitely.
 */
int gpu_m4_drm_timestamp_alias_token_arm_locked(struct gpu_m4_drm_timestamp_alias_token *token,
	struct gpu_m4_drm_provider_job *job);
int gpu_m4_drm_timestamp_alias_token_publish_locked(struct gpu_m4_drm_timestamp_alias_token *token,
	bool trial);
/* All fallible alias/lane/current-job/translation checks belong in the FULL
 * retirement predicate BEFORE cache invalidation and success-ready commit.
 */
int gpu_m4_drm_timestamp_alias_token_check_locked(struct gpu_m4_drm_timestamp_alias_token *token);
/* INFALLIBLE bookkeeping ONLY from the actual lane's successful-retirement
 * notification, after job_retire==0 and BEFORE releasing its active item.
 * No late validation, fence signal, hardware operation or final put here.
 */
void gpu_m4_drm_timestamp_alias_token_complete_locked(struct gpu_m4_drm_timestamp_alias_token *token);
void gpu_m4_drm_timestamp_alias_token_fault_locked(struct gpu_m4_drm_timestamp_alias_token *token,
	int error);
/* Outside ALL provider/resv locks; recursion from final job puts is harmless.
 * Destroy closes only an idle registry and drops its upper lease references.
 * Visible upper baselines remain reachable: -EBUSY until the supplied owner
 * releases them through independently qualified shutdown. Retry destroy then;
 * never use the closed registry/upper for new requests or publication.
 */
void gpu_m4_drm_timestamp_alias_reap(struct gpu_m4_drm_timestamp_aliases *aliases);
int gpu_m4_drm_timestamp_alias_destroy(struct gpu_m4_drm_timestamp_aliases *aliases);

#endif
