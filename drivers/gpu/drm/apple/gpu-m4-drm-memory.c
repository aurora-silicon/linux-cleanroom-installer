// SPDX-License-Identifier: GPL-2.0-only
/* Pinned local backing and explicit WB ownership for the retained provider. */
#include <linux/kref.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <asm/cacheflush.h>
#include <drm/drm_gem_shmem_helper.h>
#include "gpu-m4-drm-memory.h"
#include "gpu-m4-drm-validation.h"
#include "gpu-m4-format.h"
#include "gpu-m4-drm-vm-ledger.h"

struct gpu_m4_drm_bo_pin {
	struct kref ref;
	struct drm_gem_object *object;
	struct page **pages;
	u64 size;
	enum gpu_m4_drm_cpu_policy policy;
	/* Every page of this exact pin/page vector passed gpu_m4_drm_owned_page()
	 * at acquisition. The shmem pin keeps that immutable page vector and its
	 * pages for this pin's whole lifetime; a new pin is a new admission. Page
	 * legitimacy only: cache maintenance still runs on every prepare/retire.
	 */
	bool admitted;
};

struct gpu_m4_drm_cache_job {
	struct gpu_m4_drm_cache_access *access;
	/* Row positions in (pin, position) order, built once at create: an
	 * access check visits exactly the rows of its pin, never the others.
	 */
	u32 *order;
	u32 count;
	bool prepared, published, retired;
};

/* This driver uses native 16-KiB shmem pages, not a guessed compound layout. */
static int gpu_m4_drm_owned_page(struct page *page, u64 *pa,
			       unsigned long *address)
{
	void *cpu;
	u64 physical;

	if (PAGE_SIZE != GPU_M4_DRM_PAGE_SIZE || !page || PageReserved(page) ||
	    PageHighMem(page))
		return -EOPNOTSUPP;
	cpu = page_address(page);
	physical = page_to_phys(page);
	if (!cpu || !virt_addr_valid(cpu) || virt_to_page(cpu) != page ||
	    ((unsigned long)cpu & (PAGE_SIZE - 1)) ||
	    (unsigned long)cpu > ~0UL - PAGE_SIZE ||
	    !physical || (physical & ~GPU_M4_PA_MASK))
		return -EOPNOTSUPP;
	if (pa)
		*pa = physical;
	if (address)
		*address = (unsigned long)cpu;
	return 0;
}

struct gpu_m4_drm_bo_pin *gpu_m4_drm_bo_pin_locked(struct drm_gem_object *object)
{
	struct drm_gem_shmem_object *shmem;
	struct gpu_m4_drm_bo_pin *pin;
	enum gpu_m4_drm_cpu_policy policy;
	int ret;

	if (!gpu_m4_drm_bo_is_native(object))
		return ERR_PTR(-EOPNOTSUPP);
	dma_resv_assert_held(object->resv);
	if (PAGE_SIZE != GPU_M4_DRM_PAGE_SIZE || !object->size ||
	    !gpu_m4_drm_aligned(object->size))
		return ERR_PTR(-EOPNOTSUPP);
	policy = gpu_m4_drm_bo_policy(object);
	if (policy != GPU_M4_DRM_CPU_NC && policy != GPU_M4_DRM_CPU_WRITEBACK)
		return ERR_PTR(-EOPNOTSUPP);
	pin = kzalloc(sizeof(*pin), GFP_KERNEL);
	if (!pin)
		return ERR_PTR(-ENOMEM);
	/* NC's initial WB allocation zeroing must precede any NC CPU/GPU view. */
	if (policy == GPU_M4_DRM_CPU_NC) {
		ret = gpu_m4_drm_bo_prepare_locked(object);
		if (ret)
			goto free;
	}
	shmem = to_drm_gem_shmem_obj(object);
	ret = drm_gem_shmem_pin_locked(shmem);
	if (ret)
		goto free;
	if (!shmem->pages) {
		ret = -EIO;
		goto unpin;
	}
	for (u64 i = 0; i < object->size / PAGE_SIZE; i++) {
		ret = gpu_m4_drm_owned_page(shmem->pages[i], NULL, NULL);
		if (ret)
			goto unpin;
	}
	kref_init(&pin->ref);
	drm_gem_object_get(object);
	pin->object = object;
	pin->pages = shmem->pages;
	pin->size = object->size;
	pin->policy = policy;
	pin->admitted = true;
	return pin;
unpin:
	drm_gem_shmem_unpin_locked(shmem);
free:
	kfree(pin);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_bo_pin_locked);

static void gpu_m4_drm_bo_pin_free(struct kref *ref)
{
	struct gpu_m4_drm_bo_pin *pin = container_of(ref, struct gpu_m4_drm_bo_pin, ref);
	struct drm_gem_object *object = pin->object;

	/* Caller has dropped reservation locks before its final owner release. */
	dma_resv_lock(object->resv, NULL);
	drm_gem_shmem_unpin_locked(to_drm_gem_shmem_obj(object));
	dma_resv_unlock(object->resv);
	drm_gem_object_put(object);
	kfree(pin);
}

void gpu_m4_drm_bo_pin_get(struct gpu_m4_drm_bo_pin *pin)
{
	kref_get(&pin->ref);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_bo_pin_get);

void gpu_m4_drm_bo_pin_put(struct gpu_m4_drm_bo_pin *pin)
{
	if (pin)
		kref_put(&pin->ref, gpu_m4_drm_bo_pin_free);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_bo_pin_put);

struct drm_gem_object *gpu_m4_drm_bo_pin_object(struct gpu_m4_drm_bo_pin *pin)
{
	return pin ? pin->object : NULL;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_bo_pin_object);

u64 gpu_m4_drm_bo_pin_size(struct gpu_m4_drm_bo_pin *pin)
{
	return pin ? pin->size : 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_bo_pin_size);

int gpu_m4_drm_bo_pin_page(struct gpu_m4_drm_bo_pin *pin, u64 offset, u64 *pa)
{
	if (!pin || !pa || !gpu_m4_drm_aligned(offset) || offset >= pin->size ||
	    GPU_M4_DRM_PAGE_SIZE > pin->size - offset)
		return -EINVAL;
	if (pin->admitted) {
		/* Same physical address the acquisition proved for this page. */
		*pa = page_to_phys(pin->pages[offset / PAGE_SIZE]);
		return 0;
	}
	return gpu_m4_drm_owned_page(pin->pages[offset / PAGE_SIZE], pa, NULL);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_bo_pin_page);

static int gpu_m4_drm_cache_validate(const struct gpu_m4_drm_cache_access *a)
{
	u32 allowed = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE;

	if (!a->pin || !a->access || (a->access & ~allowed) || !a->size ||
	    a->offset >= a->pin->size || a->size > a->pin->size - a->offset)
		return -EINVAL;
	/* pin owns every page and the immutable array through the whole operation.
	 * Its acquisition already admitted every page: reuse that legitimacy for
	 * cache-job creation and preparation, including NC spans that need no
	 * maintenance. Only an unadmitted pin is rechecked page by page.
	 */
	if (a->pin->admitted)
		return 0;
	for (u64 i = a->offset / PAGE_SIZE;
	     i <= (a->offset + a->size - 1) / PAGE_SIZE; i++) {
		int ret = gpu_m4_drm_owned_page(a->pin->pages[i], NULL, NULL);

		if (ret)
			return ret;
	}
	return 0;
}

static bool gpu_m4_drm_cache_row_before(const struct gpu_m4_drm_cache_access *access,
	u32 x, u32 y)
{
	if (access[x].pin != access[y].pin)
		return (unsigned long)access[x].pin < (unsigned long)access[y].pin;
	return x < y;
}

/* Bounded, non-recursive heapsort of the row positions by (pin, position). */
static void gpu_m4_drm_cache_job_index(struct gpu_m4_drm_cache_job *job)
{
	u32 *a = job->order, count = job->count;

	for (u32 i = 0; i < count; i++)
		a[i] = i;
	for (u32 end = count, start = count / 2; end > 1;) {
		u32 root, tmp;

		if (start) {
			root = --start;
		} else {
			tmp = a[0];
			a[0] = a[--end];
			a[end] = tmp;
			root = 0;
		}
		while (root < end / 2) {
			u32 child = root * 2 + 1;

			if (child + 1 < end &&
			    gpu_m4_drm_cache_row_before(job->access, a[child], a[child + 1]))
				child++;
			if (!gpu_m4_drm_cache_row_before(job->access, a[root], a[child]))
				break;
			tmp = a[root];
			a[root] = a[child];
			a[child] = tmp;
			root = child;
		}
	}
}

struct gpu_m4_drm_cache_job *
gpu_m4_drm_cache_job_create(const struct gpu_m4_drm_cache_access *access, u32 count)
{
	struct gpu_m4_drm_cache_job *job;

	if ((count && !access) || count > GPU_M4_DRM_MAX_BINDS)
		return ERR_PTR(-EINVAL);
	for (u32 i = 0; i < count; i++) {
		int ret = gpu_m4_drm_cache_validate(&access[i]);

		if (ret)
			return ERR_PTR(ret);
	}
	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job)
		return ERR_PTR(-ENOMEM);
	/* No rows: an ordering-only owner (prepare/retire barriers, phases). */
	if (!count)
		return job;
	job->access = kmemdup(access, array_size(count, sizeof(*access)), GFP_KERNEL);
	job->order = kcalloc(count, sizeof(*job->order), GFP_KERNEL);
	if (!job->access || !job->order) {
		kfree(job->order);
		kfree(job->access);
		kfree(job);
		return ERR_PTR(-ENOMEM);
	}
	job->count = count;
	gpu_m4_drm_cache_job_index(job);
	for (u32 i = 0; i < count; i++)
		gpu_m4_drm_bo_pin_get(access[i].pin);
	return job;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_cache_job_create);

struct gpu_m4_drm_cache_job *gpu_m4_drm_cache_job_snapshot_vm(
	const struct gpu_m4_drm_vm_ledger *ledger, void *cookie,
	struct gpu_m4_drm_bo_pin *(*lookup)(void *cookie, const void *identity))
{
	struct gpu_m4_drm_vm_access_object *objects;
	struct gpu_m4_drm_cache_access *access;
	struct gpu_m4_drm_cache_job *job;
	u32 capacity, count;
	int ret;

	if (!ledger || !ledger->count || !lookup)
		return ERR_PTR(-EINVAL);
	capacity = ledger->count < GPU_M4_DRM_MAX_BINDS ?
		ledger->count : GPU_M4_DRM_MAX_BINDS;
	objects = kcalloc(capacity, sizeof(*objects), GFP_KERNEL);
	access = kcalloc(capacity, sizeof(*access), GFP_KERNEL);
	if (!objects || !access) {
		job = ERR_PTR(-ENOMEM);
		goto free;
	}
	ret = gpu_m4_drm_vm_snapshot_objects(ledger, objects, capacity, &count);
	if (ret) {
		job = ERR_PTR(ret);
		goto free;
	}
	for (u32 i = 0; i < count; i++) {
		struct gpu_m4_drm_bo_pin *pin = lookup(cookie, objects[i].identity);

		if (IS_ERR_OR_NULL(pin)) {
			job = pin ? ERR_CAST(pin) : ERR_PTR(-ENOENT);
			goto free;
		}
		if (pin->object != objects[i].identity || pin->policy != objects[i].cpu_policy) {
			job = ERR_PTR(-EPERM);
			goto free;
		}
		access[i] = (struct gpu_m4_drm_cache_access) {
			.pin = pin, .offset = objects[i].offset, .size = objects[i].size,
			.access = objects[i].access,
		};
	}
	job = gpu_m4_drm_cache_job_create(access, count);
free:
	kfree(access);
	kfree(objects);
	return job;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_cache_job_snapshot_vm);

/* Caller serializes job state and observes the API's CPU access synchronization.
 * Admitted local native WB pins are mapped through the shared lower/firmware
 * memory class that the GPU accesses coherently with CPU WB caches (as the
 * unchanged Mesa's no-op host flush/invalidate expects): they need no per-job
 * payload sweep, only the ordering barriers below. Other WB pins keep the full
 * referenced-page clean/invalidate.
 */
static int gpu_m4_drm_cache_maintain(struct gpu_m4_drm_cache_job *job, bool retire)
{
	/* No cache-line change before every backing/span has passed validation. */
	for (u32 i = 0; i < job->count; i++) {
		int ret = gpu_m4_drm_cache_validate(&job->access[i]);

		if (ret)
			return ret;
	}
	if (retire)
		dma_rmb();
	for (u32 i = 0; i < job->count; i++) {
		const struct gpu_m4_drm_cache_access *a = &job->access[i];
		bool write = a->access & DRM_ASAHI_BIND_WRITE;

		if (a->pin->policy != GPU_M4_DRM_CPU_WRITEBACK || (retire && !write) ||
		    a->pin->admitted)
			continue;
		for (u64 p = a->offset / PAGE_SIZE;
		     p <= (a->offset + a->size - 1) / PAGE_SIZE; p++) {
			unsigned long address = (unsigned long)page_address(a->pin->pages[p]);

			if (!retire)
				dcache_clean_poc(address, address + PAGE_SIZE);
			if (retire || write)
				dcache_inval_poc(address, address + PAGE_SIZE);
		}
	}
	if (!retire)
		dma_wmb();
	return 0;
}

int gpu_m4_drm_cache_job_prepare(struct gpu_m4_drm_cache_job *job)
{
	int ret;

	if (!job || job->published || job->retired)
		return -EINVAL;
	/* Re-cleaning privately is allowed, never reuse after first publication. */
	ret = gpu_m4_drm_cache_maintain(job, false);
	job->prepared = !ret;
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_cache_job_prepare);

int gpu_m4_drm_cache_job_publish(struct gpu_m4_drm_cache_job *job)
{
	if (!job || !job->prepared || job->published || job->retired)
		return -EINVAL;
	job->published = true;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_cache_job_publish);

int gpu_m4_drm_cache_job_retire(struct gpu_m4_drm_cache_job *job)
{
	int ret;

	if (!job || !job->prepared || !job->published || job->retired)
		return -EINVAL;
	ret = gpu_m4_drm_cache_maintain(job, true);
	if (!ret)
		job->retired = true;
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_cache_job_retire);

int gpu_m4_drm_cache_job_check_access(const struct gpu_m4_drm_cache_job *job,
	struct gpu_m4_drm_bo_pin *pin, u64 offset, u64 size, u32 access, bool retired)
{
	const u32 allowed = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE;
	u32 first = 0;
	u64 bound;

	if (IS_ERR_OR_NULL(job) || IS_ERR_OR_NULL(pin) || !access ||
	    (access & ~allowed) || !size || !gpu_m4_drm_bo_pin_object(pin) ||
	    !job->access || !job->order || !job->count || job->count > GPU_M4_DRM_MAX_BINDS)
		return -EINVAL;
	bound = gpu_m4_drm_bo_pin_size(pin);
	if (offset >= bound || size > bound - offset)
		return -EINVAL;
	if (!job->prepared || !job->published || job->retired != retired)
		return -EINVAL;
	/* First row of this pin in the immutable (pin, position) order. */
	for (u32 end = job->count; first < end;) {
		u32 middle = first + (end - first) / 2;

		if ((unsigned long)job->access[job->order[middle]].pin < (unsigned long)pin)
			first = middle + 1;
		else
			end = middle;
	}
	for (; first < job->count && job->access[job->order[first]].pin == pin; first++) {
		const struct gpu_m4_drm_cache_access *a = &job->access[job->order[first]];

		/* Never re-resolve pages or touch a CPU alias. create() retained and
		 * validated this pin; check only its immutable captured byte contract.
		 * A second acquisition of the same GEM is NOT the same retained pin:
		 * only rows of this exact pin are visited.
		 */
		if (!a->access || (a->access & ~allowed) || !a->size ||
		    a->offset >= bound || a->size > bound - a->offset ||
		    (a->access & access) != access || offset < a->offset)
			continue;
		if (offset - a->offset < a->size && size <= a->size - (offset - a->offset))
			return 0;
	}
	return -EACCES;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_cache_job_check_access);

int gpu_m4_drm_cache_job_check_coherent(const struct gpu_m4_drm_cache_job *job,
	struct gpu_m4_drm_bo_pin *pin, bool retired)
{
	if (IS_ERR_OR_NULL(job) || IS_ERR_OR_NULL(pin) || !pin->object)
		return -EINVAL;
	if (!job->prepared || !job->published || job->retired != retired)
		return -EINVAL;
	return pin->admitted ? 0 : -EACCES;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_cache_job_check_coherent);

int gpu_m4_drm_cache_job_release(struct gpu_m4_drm_cache_job *job)
{
	if (!job)
		return -EINVAL;
	if (job->published && !job->retired)
		return -EBUSY;
	for (u32 i = 0; i < job->count; i++)
		gpu_m4_drm_bo_pin_put(job->access[i].pin);
	kfree(job->order);
	kfree(job->access);
	kfree(job);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_cache_job_release);

MODULE_DESCRIPTION("Apple M4 retained-provider local BO cache ownership");
MODULE_LICENSE("GPL");
