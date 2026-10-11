// SPDX-License-Identifier: GPL-2.0-only
/* Exact implicit dependencies for the retained GPU provider's accepting step. */
#include <linux/dma-resv.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <drm/drm_exec.h>
#include <drm/drm_gem.h>
#include "gpu-m4-drm-resv.h"
#include "gpu-m4-drm-validation.h"

struct gpu_m4_drm_resv_object {
	struct drm_gem_object *object;
	struct dma_resv *resv;
	struct dma_fence **fences;
	u32 access, object_access, leader, count, next;
	bool inherited;
};

struct gpu_m4_drm_resv_job {
	struct drm_exec exec;
	struct gpu_m4_drm_resv_object *objects;
	struct dma_fence *finished;
	u32 count, next;
	bool initialized, published, inherited;
};

void gpu_m4_drm_resv_release(struct gpu_m4_drm_resv_job *job)
{
	if (!job)
		return;
	/* A failed later private accepting step may still own every ww lock. */
	if (job->initialized)
		drm_exec_fini(&job->exec);
	for (u32 i = 0; i < job->count; i++) {
		struct gpu_m4_drm_resv_object *o = &job->objects[i];

		for (u32 j = 0; j < o->count; j++)
			dma_fence_put(o->fences[j]);
		kfree(o->fences);
		drm_gem_object_put(o->object);
	}
	dma_fence_put(job->finished);
	kfree(job->objects);
	kfree(job);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_resv_release);

/* Both owners hold actual GEM identities. Matching a reservation alone cannot
 * replace a missing common GEM or broaden its originally captured permissions.
 * The parent's dependency cursors are deliberately not inspected or copied.
 */
static int gpu_m4_resv_inherit(struct gpu_m4_drm_resv_job *job,
	const struct gpu_m4_drm_resv_job *parent)
{
	for (u32 i = 0; i < parent->count; i++) {
		const struct gpu_m4_drm_resv_object *p = &parent->objects[i];
		u32 j;

		if (!p->object || !p->resv || p->object->resv != p->resv ||
		    p->leader > i || parent->objects[p->leader].resv != p->resv)
			return -ESTALE;
		for (j = 0; j < job->count; j++)
			if (job->objects[j].object == p->object)
				break;
		if (j == job->count || job->objects[j].object_access != p->object_access)
			return -EPERM;
	}
	for (u32 i = 0; i < job->count; i++) {
		struct gpu_m4_drm_resv_object *o = &job->objects[i];
		const struct gpu_m4_drm_resv_object *p = NULL;

		if (o->leader != i)
			continue;
		for (u32 j = 0; j < parent->count; j++)
			if (parent->objects[j].leader == j && parent->objects[j].resv == o->resv) {
				p = &parent->objects[j];
				break;
			}
		if (!p)
			continue;
		/* A new private GEM sharing a common reservation may not require old
		 * reader fences absent from the original captured dependency class.
		 */
		if (o->access != p->access)
			return -EPERM;
		if (p->count && !p->fences)
			return -ESTALE;
		o->inherited = true;
		if (!p->count)
			continue;
		o->fences = kcalloc(p->count, sizeof(*o->fences), GFP_KERNEL);
		if (!o->fences)
			return -ENOMEM;
		for (u32 j = 0; j < p->count; j++) {
			if (IS_ERR_OR_NULL(p->fences[j]))
				return -ESTALE;
			o->fences[o->count++] = dma_fence_get(p->fences[j]);
		}
	}
	job->finished = dma_fence_get(parent->finished);
	job->inherited = true;
	return 0;
}

static struct gpu_m4_drm_resv_job *gpu_m4_resv_prepare(
	const struct gpu_m4_drm_resv_job *parent,
	const struct gpu_m4_drm_resv_access *access, u32 count)
{
	struct gpu_m4_drm_resv_job *job;
	u32 allowed = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE;
	int ret;

	if (!access || !count || count > GPU_M4_DRM_MAX_BINDS)
		return ERR_PTR(-EINVAL);
	/* Reject still-locked/private parents BEFORE any child GEM ref, so even
	 * this misuse cannot unwind native refs while the parent owns ww locks.
	 */
	if (parent && (!parent->published || parent->initialized || !parent->count ||
	    parent->count > GPU_M4_DRM_MAX_BINDS || !parent->objects ||
	    IS_ERR_OR_NULL(parent->finished) || dma_fence_is_signaled(parent->finished)))
		return ERR_PTR(-ESTALE);
	/* Input validation precedes every GEM ref and reservation acquisition. */
	for (u32 i = 0; i < count; i++) {
		if (!access[i].object || !access[i].object->resv || !access[i].access ||
		    (access[i].access & ~allowed))
			return ERR_PTR(-EINVAL);
		if (drm_gem_is_imported(access[i].object))
			return ERR_PTR(-EOPNOTSUPP);
	}
	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job)
		return ERR_PTR(-ENOMEM);
	job->objects = kcalloc(count, sizeof(*job->objects), GFP_KERNEL);
	if (!job->objects) {
		ret = -ENOMEM;
		goto fail;
	}
	for (u32 i = 0; i < count; i++) {
		u32 j;

		for (j = 0; j < job->count; j++)
			if (job->objects[j].object == access[i].object)
				break;
		if (j < job->count) {
			job->objects[j].access |= access[i].access;
			job->objects[j].object_access |= access[i].access;
			continue;
		}
		drm_gem_object_get(access[i].object);
		job->objects[j].object = access[i].object;
		job->objects[j].resv = access[i].object->resv;
		job->objects[j].access = access[i].access;
		job->objects[j].object_access = access[i].access;
		job->objects[j].leader = j;
		job->count++;
	}
	/* Different objects may share one reservation: one lock/slot/snapshot with
	 * unioned permissions, but retain EACH GEM identity independently.
	 */
	for (u32 i = 0; i < job->count; i++)
		for (u32 j = 0; j < i; j++)
			if (job->objects[i].object->resv == job->objects[j].object->resv) {
				u32 leader = job->objects[j].leader;

				job->objects[i].leader = leader;
				job->objects[leader].access |= job->objects[i].access;
				break;
			}
	if (parent) {
		ret = gpu_m4_resv_inherit(job, parent);
		if (ret)
			goto fail;
	}
	drm_exec_init(&job->exec, DRM_EXEC_INTERRUPTIBLE_WAIT, job->count);
	job->initialized = true;
	drm_exec_until_all_locked(&job->exec) {
		for (u32 i = 0; i < job->count; i++) {
			if (job->objects[i].leader != i)
				continue;
			ret = job->objects[i].inherited ?
				drm_exec_lock_obj(&job->exec, job->objects[i].object) :
				drm_exec_prepare_obj(&job->exec, job->objects[i].object, 1);
			drm_exec_retry_on_contention(&job->exec);
			if (ret)
				goto fail;
		}
	}
	/* Every reservation remains held until publish_finished/release. Captured
	 * lists predate the newly armed finished fence, never self-depend on it.
	 */
	for (u32 i = 0; i < job->count; i++) {
		struct gpu_m4_drm_resv_object *o = &job->objects[i];

		if (o->leader != i || o->inherited)
			continue;
		ret = dma_resv_get_fences(o->object->resv,
			dma_resv_usage_rw(o->access & DRM_ASAHI_BIND_WRITE), &o->count, &o->fences);
		if (ret)
			goto fail;
		/* Kernel-private leaders carry earlier CHILD retirement fences, never
		 * this aggregate's public fence. Finding it means ambiguous sharing
		 * with the public inventory: reject before publication, never filter.
		 */
		for (u32 j = 0; parent && j < o->count; j++)
			if (o->fences[j] == job->finished) {
				ret = -EBUSY;
				goto fail;
			}
	}
	return job;
fail:
	gpu_m4_drm_resv_release(job);
	return ERR_PTR(ret);
}

struct gpu_m4_drm_resv_job *gpu_m4_drm_resv_prepare(
	const struct gpu_m4_drm_resv_access *access, u32 count)
{
	return gpu_m4_resv_prepare(NULL, access, count);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_resv_prepare);

struct gpu_m4_drm_resv_job *gpu_m4_drm_resv_prepare_inherited(
	const struct gpu_m4_drm_resv_job *parent,
	const struct gpu_m4_drm_resv_access *access, u32 count)
{
	if (IS_ERR_OR_NULL(parent))
		return ERR_PTR(-EINVAL);
	return gpu_m4_resv_prepare(parent, access, count);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_resv_prepare_inherited);

/* Inherited common leaders already hold the public fence: lock-only. */
static void gpu_m4_resv_publish(struct gpu_m4_drm_resv_job *job, struct dma_fence *fence)
{
	for (u32 i = 0; i < job->count; i++) {
		struct gpu_m4_drm_resv_object *o = &job->objects[i];

		if (o->leader != i)
			continue;
		dma_resv_assert_held(o->object->resv);
		if (o->inherited)
			continue;
		dma_resv_add_fence(o->object->resv, fence,
			o->access & DRM_ASAHI_BIND_WRITE ? DMA_RESV_USAGE_WRITE : DMA_RESV_USAGE_READ);
	}
	drm_exec_fini(&job->exec);
	job->initialized = false;
	job->published = true;
}

void gpu_m4_drm_resv_publish_finished(struct gpu_m4_drm_resv_job *job,
				    struct dma_fence *finished)
{
	if (WARN_ON(!job || !job->initialized || job->published || job->inherited ||
		    IS_ERR_OR_NULL(finished) || dma_fence_is_signaled(finished)))
		return;
	job->finished = dma_fence_get(finished);
	gpu_m4_resv_publish(job, finished);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_resv_publish_finished);

void gpu_m4_drm_resv_publish_inherited(struct gpu_m4_drm_resv_job *job,
				     struct dma_fence *retirement)
{
	if (WARN_ON(!job || !job->initialized || job->published || !job->inherited ||
		    IS_ERR_OR_NULL(retirement) || retirement == job->finished ||
		    dma_fence_is_signaled(retirement)))
		return;
	gpu_m4_resv_publish(job, retirement);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_resv_publish_inherited);

struct dma_fence *gpu_m4_drm_resv_next_dependency(struct gpu_m4_drm_resv_job *job)
{
	if (!job || !job->published || job->initialized)
		return ERR_PTR(-EINVAL);
	while (job->next < job->count) {
		struct gpu_m4_drm_resv_object *o = &job->objects[job->next];

		if (o->next < o->count)
			return dma_fence_get(o->fences[o->next++]);
		job->next++;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_resv_next_dependency);

MODULE_DESCRIPTION("Apple M4 retained-provider implicit reservation snapshots");
MODULE_LICENSE("GPL");
