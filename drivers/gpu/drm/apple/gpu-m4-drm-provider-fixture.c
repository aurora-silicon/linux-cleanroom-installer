// SPDX-License-Identifier: GPL-2.0-only
/* PRIVATE native fixture ownership; never a public Mesa encoder or HW proof. */
#include <linux/dma-fence-unwrap.h>
#include <linux/dma-resv.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <drm/drm_gem.h>

#include "gpu-m4-drm-provider-fixture.h"

struct gpu_m4_fixture_fence {
	struct dma_fence base;
	spinlock_t lock;
	bool ready;
	int error;
};

struct gpu_m4_drm_provider_fixture {
	struct gpu_m4_drm_provider_fixture_config config;
	struct gpu_m4_drm_provider_version *version;
	struct gpu_m4_drm_vm_owner *snapshot;
	struct gpu_m4_drm_vm *frontend;
	struct gpu_m4_drm_cache_job *cache;
	struct gpu_m4_drm_resv_job *resv;
	struct gpu_m4_fixture_fence *finished;
	struct dma_fence *dependency;
	enum gpu_m4_drm_provider_fixture_state state;
	u32 native_count, dependencies_consumed;
	int error;
	bool provider_held, payload_held, protected, cpu_visible;
};

static struct mutex *gpu_m4_fixture_lock(struct gpu_m4_drm_provider_fixture *f)
{
	return gpu_m4_drm_provider_lock(f->config.provider);
}

static const char *gpu_m4_fixture_fence_driver(struct dma_fence *fence)
{
	return "apple-gpu-m4-private-fixture";
}

static const char *gpu_m4_fixture_fence_timeline(struct dma_fence *fence)
{
	return "retained-native-fixture";
}

static void gpu_m4_fixture_fence_release(struct dma_fence *fence)
{
	/* base is first; core RCU freeing outlives this module release callback. */
	dma_fence_free(fence);
	module_put(THIS_MODULE);
}

static const struct dma_fence_ops gpu_m4_fixture_fence_ops = {
	.get_driver_name = gpu_m4_fixture_fence_driver,
	.get_timeline_name = gpu_m4_fixture_fence_timeline,
	.release = gpu_m4_fixture_fence_release,
};

static void gpu_m4_fixture_ready_locked(struct gpu_m4_drm_provider_fixture *f, int error)
{
	unsigned long flags;

	lockdep_assert_held(gpu_m4_fixture_lock(f));
	if (!f->finished)
		return;
	spin_lock_irqsave(&f->finished->lock, flags);
	if (!f->finished->ready) {
		f->finished->error = error;
		f->finished->ready = true;
	}
	spin_unlock_irqrestore(&f->finished->lock, flags);
}

void gpu_m4_drm_provider_fixture_fault_locked(struct gpu_m4_drm_provider_fixture *f,
					    int error)
{
	if (!f)
		return;
	lockdep_assert_held(gpu_m4_fixture_lock(f));
	if (f->state == GPU_M4_DRM_FIXTURE_COMPLETE || f->state == GPU_M4_DRM_FIXTURE_FAULTED)
		return;
	error = error < 0 ? error : -EPROTO;
	f->error = error;
	f->state = GPU_M4_DRM_FIXTURE_FAULTED;
	gpu_m4_drm_provider_fault_locked(f->config.provider, error);
	if (f->frontend)
		gpu_m4_drm_vm_error(f->frontend, error);
	/* Error signaling is separate and is NOT permission to release any owner. */
	gpu_m4_fixture_ready_locked(f, error);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_fixture_fault_locked);

static int gpu_m4_fixture_native_shape(struct gpu_m4_drm_provider_fixture *f)
{
	const struct gpu_m4_drm_cache_access *cache;
	const struct gpu_m4_drm_resv_access *resv;
	u32 nc, nr, allowed = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE;
	u64 bytes = 0;

	cache = gpu_m4_drm_vm_owner_cache_access(f->snapshot, &nc);
	resv = gpu_m4_drm_vm_owner_resv_access(f->snapshot, &nr);
	if (!cache || !resv || !nc || nc != nr || nc > GPU_M4_DRM_FIXTURE_MAX_NATIVE ||
	    gpu_m4_drm_vm_owner_leaf_count(f->snapshot) > GPU_M4_DRM_FIXTURE_MAX_LEAVES)
		return -E2BIG;
	for (u32 i = 0; i < nc; i++) {
		struct drm_gem_object *object = gpu_m4_drm_bo_pin_object(cache[i].pin);
		u64 size = gpu_m4_drm_bo_pin_size(cache[i].pin);

		if (!object || !gpu_m4_drm_bo_is_native(object) || object != resv[i].object ||
		    !cache[i].access || (cache[i].access & ~allowed) ||
		    cache[i].access != resv[i].access || !cache[i].size ||
		    cache[i].offset >= size || cache[i].size > size - cache[i].offset)
			return -EPROTO;
		for (u32 j = 0; j < i; j++)
			if (resv[j].object == object)
				return -EPROTO;
		if (cache[i].size > GPU_M4_DRM_FIXTURE_MAX_NATIVE_BYTES - bytes)
			return -E2BIG;
		bytes += cache[i].size;
	}
	f->native_count = nc;
	return 0;
}

/* Unaccepted only; final pin/snapshot/payload destruction stays outside locks.
 * A refused drop leaves the remaining owner graph reachable through *out.
 */
static int gpu_m4_fixture_reject(struct gpu_m4_drm_provider_fixture *f)
{
	struct mutex *lock = gpu_m4_fixture_lock(f);
	int ret;

	lockdep_assert_not_held(lock);
	if (f->resv) {
		gpu_m4_drm_resv_release(f->resv);
		f->resv = NULL;
	}
	if (f->cache) {
		ret = gpu_m4_drm_cache_job_release(f->cache);
		if (ret)
			return ret;
		f->cache = NULL;
	}
	if (f->snapshot) {
		ret = gpu_m4_drm_vm_owner_release(f->snapshot);
		if (ret)
			return ret;
		f->snapshot = NULL;
	}
	if (f->version) {
		mutex_lock(lock);
		ret = gpu_m4_drm_provider_version_put_locked(f->version);
		mutex_unlock(lock);
		if (ret)
			return ret;
		f->version = NULL;
	}
	if (f->provider_held) {
		gpu_m4_drm_provider_reap(f->config.provider);
		ret = gpu_m4_drm_provider_put(f->config.provider);
		if (ret)
			return ret;
		f->provider_held = false;
	}
	if (f->payload_held)
		f->config.payload_put(f->config.payload);
	if (f->finished)
		dma_fence_put(&f->finished->base);
	kfree(f);
	return 0;
}

int gpu_m4_drm_provider_fixture_stage(
	const struct gpu_m4_drm_provider_fixture_config *config,
	struct gpu_m4_drm_provider_fixture **out)
{
	struct gpu_m4_drm_provider_fixture *f;
	const struct gpu_m4_drm_cache_access *cache;
	const struct gpu_m4_drm_resv_access *resv;
	struct mutex *lock;
	u32 count;
	int ret, rejected;

	if (!out)
		return -EINVAL;
	*out = NULL;
	if (!config || !config->provider || !config->version || !config->payload ||
	    !config->payload_get || !config->payload_put || !config->check_retired_locked ||
	    !config->check_visible_locked)
		return -EINVAL;
	lock = gpu_m4_drm_provider_lock(config->provider);
	lockdep_assert_not_held(lock);
	f = kzalloc(sizeof(*f), GFP_KERNEL);
	if (!f)
		return -ENOMEM;
	f->config = *config;
	ret = gpu_m4_drm_provider_get(config->provider);
	if (ret)
		goto reject;
	f->provider_held = true;
	ret = config->payload_get(config->payload);
	if (ret) {
		ret = ret < 0 ? ret : -EPROTO;
		goto reject;
	}
	f->payload_held = true;
	f->finished = kzalloc(sizeof(*f->finished), GFP_KERNEL);
	if (!f->finished) {
		ret = -ENOMEM;
		goto reject;
	}
	if (!try_module_get(THIS_MODULE)) {
		kfree(f->finished);
		f->finished = NULL;
		ret = -ENODEV;
		goto reject;
	}
	spin_lock_init(&f->finished->lock);
	dma_fence_init(&f->finished->base, &gpu_m4_fixture_fence_ops,
		&f->finished->lock, dma_fence_context_alloc(1), 1);
	mutex_lock(lock);
	ret = gpu_m4_drm_provider_error_locked(config->provider);
	if (!ret)
		ret = gpu_m4_drm_provider_version_capture_locked(config->version,
			&f->version, &f->snapshot);
	if (!ret) {
		f->frontend = gpu_m4_drm_vm_owner_vm(f->snapshot);
		ret = gpu_m4_fixture_native_shape(f);
	}
	mutex_unlock(lock);
	if (ret)
		goto reject;
	cache = gpu_m4_drm_vm_owner_cache_access(f->snapshot, &count);
	f->cache = gpu_m4_drm_cache_job_create(cache, count);
	if (IS_ERR(f->cache)) {
		ret = PTR_ERR(f->cache);
		f->cache = NULL;
		goto reject;
	}
	resv = gpu_m4_drm_vm_owner_resv_access(f->snapshot, &count);
	/* LAST fallible accepting step. No subsequent allocation/ownership join or
	 * provider-lock acquisition occurs while reservations remain locked.
	 */
	f->resv = gpu_m4_drm_resv_prepare(resv, count);
	if (IS_ERR(f->resv)) {
		ret = PTR_ERR(f->resv);
		f->resv = NULL;
		goto reject;
	}
	gpu_m4_drm_resv_publish_finished(f->resv, &f->finished->base);
	f->state = GPU_M4_DRM_FIXTURE_STAGED;
	*out = f;
	return 0;
reject:
	rejected = gpu_m4_fixture_reject(f);
	if (rejected) {
		mutex_lock(lock);
		gpu_m4_drm_provider_fixture_fault_locked(f, rejected);
		mutex_unlock(lock);
		*out = f; /* keep the real remaining graph, never lose a refused drop */
		return rejected;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_fixture_stage);

struct dma_fence *gpu_m4_drm_provider_fixture_fence_get(struct gpu_m4_drm_provider_fixture *f)
{
	return f && f->finished ? dma_fence_get(&f->finished->base) : NULL;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_fixture_fence_get);

/* A chain/container's own positive status does not propagate every leaf error. */
static int gpu_m4_fixture_dependency_status(struct dma_fence *fence)
{
	struct dma_fence_unwrap cursor;
	struct dma_fence *leaf;
	int status = dma_fence_get_status(fence);

	if (status <= 0)
		return status;
	dma_fence_unwrap_for_each(leaf, &cursor, fence) {
		int error = dma_fence_get_status(leaf);

		if (error < 0 && status > 0)
			status = error;
	}
	return status;
}

int gpu_m4_drm_provider_fixture_dependencies_locked(struct gpu_m4_drm_provider_fixture *f)
{
	int ret;

	if (!f)
		return -EINVAL;
	lockdep_assert_held(gpu_m4_fixture_lock(f));
	if (f->state == GPU_M4_DRM_FIXTURE_FAULTED)
		return f->error;
	if (f->state == GPU_M4_DRM_FIXTURE_DEPENDENCIES)
		return 0;
	if (f->state != GPU_M4_DRM_FIXTURE_STAGED)
		return -EINVAL;
	ret = gpu_m4_drm_provider_error_locked(f->config.provider);
	if (ret)
		goto fault;
	for (;;) {
		if (!f->dependency) {
			f->dependency = gpu_m4_drm_resv_next_dependency(f->resv);
			if (IS_ERR(f->dependency)) {
				ret = PTR_ERR(f->dependency);
				f->dependency = NULL;
				goto fault;
			}
			if (!f->dependency) {
				f->state = GPU_M4_DRM_FIXTURE_DEPENDENCIES;
				return 0;
			}
			if (f->dependencies_consumed == GPU_M4_DRM_FIXTURE_MAX_DEPENDENCIES) {
				ret = -E2BIG;
				goto fault; /* retain the extra exact reference too */
			}
		}
		ret = gpu_m4_fixture_dependency_status(f->dependency);
		if (!ret)
			return -EAGAIN;
		if (ret < 0)
			goto fault;
		/* The retained resv snapshot still owns this exact old fence, so this
		 * borrowed iteration ref cannot run a final release under GPU lock.
		 */
		dma_fence_put(f->dependency);
		f->dependency = NULL;
		f->dependencies_consumed++;
	}
fault:
	gpu_m4_drm_provider_fixture_fault_locked(f, ret);
	return f->error;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_fixture_dependencies_locked);

int gpu_m4_drm_provider_fixture_prepare_locked(struct gpu_m4_drm_provider_fixture *f)
{
	int ret;

	if (!f)
		return -EINVAL;
	lockdep_assert_held(gpu_m4_fixture_lock(f));
	if (f->state != GPU_M4_DRM_FIXTURE_DEPENDENCIES)
		return f->error ? f->error : -EINVAL;
	ret = gpu_m4_drm_provider_error_locked(f->config.provider);
	if (!ret)
		ret = gpu_m4_drm_cache_job_prepare(f->cache);
	if (ret)
		gpu_m4_drm_provider_fixture_fault_locked(f, ret);
	else
		f->state = GPU_M4_DRM_FIXTURE_PREPARED;
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_fixture_prepare_locked);

int gpu_m4_drm_provider_fixture_protect_locked(struct gpu_m4_drm_provider_fixture *f)
{
	int ret;

	if (!f)
		return -EINVAL;
	lockdep_assert_held(gpu_m4_fixture_lock(f));
	if (f->state != GPU_M4_DRM_FIXTURE_PREPARED)
		return f->error ? f->error : -EINVAL;
	ret = gpu_m4_drm_provider_error_locked(f->config.provider);
	if (!ret)
		ret = gpu_m4_drm_cache_job_publish(f->cache);
	if (!ret) {
		f->protected = true;
		ret = gpu_m4_drm_vm_owner_publish(f->snapshot);
	}
	if (ret)
		gpu_m4_drm_provider_fixture_fault_locked(f, ret);
	else
		f->state = GPU_M4_DRM_FIXTURE_PROTECTED;
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_fixture_protect_locked);

int gpu_m4_drm_provider_fixture_retire_locked(struct gpu_m4_drm_provider_fixture *f)
{
	int ret;

	if (!f)
		return -EINVAL;
	lockdep_assert_held(gpu_m4_fixture_lock(f));
	if (f->state != GPU_M4_DRM_FIXTURE_PROTECTED)
		return f->error ? f->error : -EINVAL;
	ret = gpu_m4_drm_provider_error_locked(f->config.provider);
	if (ret)
		goto fault;
	ret = f->config.check_retired_locked(f->config.payload, f->version);
	if (ret == -EAGAIN)
		return ret;
	if (ret > 0)
		ret = -EPROTO;
	if (!ret)
		ret = gpu_m4_drm_cache_job_retire(f->cache);
	if (!ret)
		ret = gpu_m4_drm_vm_owner_retire(f->snapshot);
	if (!ret) {
		f->cpu_visible = true;
		f->state = GPU_M4_DRM_FIXTURE_CPU_VISIBLE;
		ret = f->config.check_visible_locked(f->config.payload, f->version);
		if (ret > 0)
			ret = -EPROTO;
	}
	if (ret)
		goto fault;
	f->state = GPU_M4_DRM_FIXTURE_COMPLETE;
	gpu_m4_fixture_ready_locked(f, 0);
	return 0;
fault:
	gpu_m4_drm_provider_fixture_fault_locked(f, ret);
	return f->error;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_fixture_retire_locked);

int gpu_m4_drm_provider_fixture_signal(struct gpu_m4_drm_provider_fixture *f)
{
	const struct gpu_m4_drm_resv_access *resv;
	u32 count;
	unsigned long flags;
	int ret = 0;

	if (!f || !f->finished)
		return -EINVAL;
	lockdep_assert_not_held(gpu_m4_fixture_lock(f));
	if (f->snapshot) {
		resv = gpu_m4_drm_vm_owner_resv_access(f->snapshot, &count);
		for (u32 i = 0; i < count; i++)
			lockdep_assert_not_held(&resv[i].object->resv->lock.base);
	}
	spin_lock_irqsave(&f->finished->lock, flags);
	if (!f->finished->ready) {
		ret = -EAGAIN;
	} else if (!dma_fence_is_signaled(&f->finished->base)) {
		if (f->finished->error)
			dma_fence_set_error(&f->finished->base, f->finished->error);
		dma_fence_signal_locked(&f->finished->base);
	}
	spin_unlock_irqrestore(&f->finished->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_fixture_signal);

void gpu_m4_drm_provider_fixture_view_locked(struct gpu_m4_drm_provider_fixture *f,
					   struct gpu_m4_drm_provider_fixture_view *view)
{
	if (!f || !view)
		return;
	lockdep_assert_held(gpu_m4_fixture_lock(f));
	*view = (struct gpu_m4_drm_provider_fixture_view) {
		.version = f->version, .state = f->state, .native_count = f->native_count,
		.dependencies_consumed = f->dependencies_consumed, .error = f->error,
		.protected = f->protected, .cpu_visible = f->cpu_visible,
	};
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_provider_fixture_view_locked);

MODULE_DESCRIPTION("Private M4 exact-version native fixture cache/reservation/fence owner");
MODULE_LICENSE("GPL");
