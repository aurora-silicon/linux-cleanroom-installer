// SPDX-License-Identifier: GPL-2.0-only
/* Retained file-global timestamp bindings; no firmware or CPU mapping here. */
#include <linux/err.h>
#include <linux/dma-resv.h>
#include <linux/kref.h>
#include <linux/lockdep.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <drm/drm_drv.h>
#include <drm/drm_gem.h>
#include "gpu-m4-drm-provider-timestamp.h"
#include "gpu-m4-drm-validation.h"

struct gpu_m4_drm_provider_timestamp_cookie {
	struct kref ref;
	struct gpu_m4_drm_provider_timestamp_config config;
	struct drm_gem_object *object;
	struct drm_device *device;
	u64 offset, range;
	enum gpu_m4_drm_cpu_policy policy;
};

static void gpu_m4_timestamp_unlocked(struct gpu_m4_drm_provider_timestamp_cookie *cookie)
{
	lockdep_assert_not_held(cookie->config.owner_lock);
	lockdep_assert_not_held(&cookie->object->resv->lock.base);
}

struct gpu_m4_drm_provider_timestamp_cookie *gpu_m4_drm_provider_timestamp_bind(
	const struct gpu_m4_drm_provider_timestamp_config *config,
	struct drm_gem_object *object, u64 offset, u64 range)
{
	struct gpu_m4_drm_provider_timestamp_cookie *cookie;
	enum gpu_m4_drm_cpu_policy policy;
	int ret;

	if (!config || !config->frontend || !config->owner || !config->owner_lock ||
	    !config->session_get || !config->session_put || IS_ERR_OR_NULL(object))
		return ERR_PTR(-EINVAL);
	if (!gpu_m4_drm_bo_belongs_to_frontend(object, config->frontend))
		return ERR_PTR(-EPERM);
	lockdep_assert_not_held(config->owner_lock);
	lockdep_assert_not_held(&object->resv->lock.base);
	policy = gpu_m4_drm_bo_policy(object);
	if (policy != GPU_M4_DRM_CPU_NC && policy != GPU_M4_DRM_CPU_WRITEBACK)
		return ERR_PTR(-EOPNOTSUPP);
	if (!range || !gpu_m4_drm_aligned(offset) || !gpu_m4_drm_aligned(range) ||
	    offset >= object->size || range > object->size - offset)
		return ERR_PTR(-EINVAL);
	cookie = kzalloc(sizeof(*cookie), GFP_KERNEL);
	if (!cookie)
		return ERR_PTR(-ENOMEM);
	cookie->config = *config;
	cookie->object = object;
	cookie->device = object->dev;
	cookie->offset = offset;
	cookie->range = range;
	cookie->policy = policy;
	kref_init(&cookie->ref);
	drm_gem_object_get(object);
	drm_dev_get(cookie->device);
	ret = config->session_get(config->owner);
	if (ret) {
		drm_gem_object_put(object);
		drm_dev_put(cookie->device);
		kfree(cookie);
		return ERR_PTR(ret < 0 ? ret : -EPROTO);
	}
	return cookie;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_timestamp_bind);

bool gpu_m4_drm_provider_timestamp_get(struct gpu_m4_drm_provider_timestamp_cookie *cookie)
{
	return !IS_ERR_OR_NULL(cookie) && kref_get_unless_zero(&cookie->ref);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_timestamp_get);

bool gpu_m4_drm_provider_timestamp_matches(struct gpu_m4_drm_provider_timestamp_cookie *cookie,
	const struct gpu_m4_drm_provider_timestamp_config *expected)
{
	return !IS_ERR_OR_NULL(cookie) && expected && kref_read(&cookie->ref) &&
		cookie->config.frontend == expected->frontend &&
		cookie->config.owner == expected->owner &&
		cookie->config.owner_lock == expected->owner_lock &&
		cookie->config.session_get == expected->session_get &&
		cookie->config.session_put == expected->session_put;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_timestamp_matches);

static void gpu_m4_timestamp_free(struct kref *ref)
{
	struct gpu_m4_drm_provider_timestamp_cookie *cookie =
		container_of(ref, struct gpu_m4_drm_provider_timestamp_cookie, ref);
	struct gpu_m4_drm_provider_timestamp_config config = cookie->config;

	gpu_m4_timestamp_unlocked(cookie);
	drm_gem_object_put(cookie->object);
	drm_dev_put(cookie->device);
	kfree(cookie);
	config.session_put(config.owner);
}

void gpu_m4_drm_provider_timestamp_put(struct gpu_m4_drm_provider_timestamp_cookie *cookie)
{
	if (IS_ERR_OR_NULL(cookie))
		return;
	gpu_m4_timestamp_unlocked(cookie);
	kref_put(&cookie->ref, gpu_m4_timestamp_free);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_timestamp_put);

int gpu_m4_drm_provider_timestamp_slice(struct gpu_m4_drm_provider_timestamp_cookie *cookie,
	u64 relative_offset, struct gpu_m4_drm_timestamp *timestamp)
{
	u64 absolute;

	if (IS_ERR_OR_NULL(cookie) || !timestamp || !kref_read(&cookie->ref) ||
	    (relative_offset & 7) || relative_offset >= cookie->range ||
	    cookie->range - relative_offset < 8)
		return -EINVAL;
	/* Construction proved offset + range <= the retained immutable BO size. */
	absolute = cookie->offset + relative_offset;
	if (!gpu_m4_drm_bo_belongs_to_frontend(cookie->object, cookie->config.frontend) ||
	    gpu_m4_drm_bo_policy(cookie->object) != cookie->policy ||
	    absolute >= cookie->object->size || cookie->object->size - absolute < 8)
		return -EPROTO;
	*timestamp = (struct gpu_m4_drm_timestamp) {
		.backend = cookie, .object = cookie->object, .offset = relative_offset,
		.object_offset = absolute,
	};
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_timestamp_slice);

u64 gpu_m4_drm_provider_timestamp_bound_offset(struct gpu_m4_drm_provider_timestamp_cookie *cookie)
{
	return cookie->offset;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_timestamp_bound_offset);

u64 gpu_m4_drm_provider_timestamp_bound_range(struct gpu_m4_drm_provider_timestamp_cookie *cookie)
{
	return cookie->range;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_timestamp_bound_range);

enum gpu_m4_drm_cpu_policy gpu_m4_drm_provider_timestamp_policy(
	struct gpu_m4_drm_provider_timestamp_cookie *cookie)
{
	return cookie->policy;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_timestamp_policy);

MODULE_DESCRIPTION("Retained M4 file-global native timestamp bindings");
MODULE_LICENSE("GPL");
