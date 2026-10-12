// SPDX-License-Identifier: GPL-2.0-only
/* Mesa ABI frontend for the strictly gated, retained J713 GPU owner. */
#include <linux/atomic.h>
#include <linux/capability.h>
#include <linux/dma-fence-chain.h>
#include <linux/kref.h>
#include <linux/limits.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/xarray.h>
#include <linux/soc/apple/pmp-display.h>
#include <asm/arch_timer.h>
#include <asm/cacheflush.h>
#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_managed.h>
#include <drm/drm_prime.h>
#include <drm/drm_syncobj.h>
#include "gpu-m4-drm.h"
#include "gpu-m4-drm-validation.h"

struct gpu_m4_drm {
	struct drm_device drm;
	struct mutex lock;
	void *owner;
	struct gpu_m4_drm_ops copied_ops;
	const struct gpu_m4_drm_ops *ops;
	struct drm_asahi_params_global params;
	u64 native_limit;
	atomic64_t native_bytes;
	int registration_error;
	bool registration_attempted, registered, probe_attempted;
};

/*
 * Lock order (never the reverse):
 *   vm->bind_lock -> gpu->lock
 *   queue->gate -> (backend submit/scheduler) -> gpu->lock (native factory)
 * gpu->lock is the short namespace/admission mutex. It is never held while
 * waiting for a VM bind lock or a queue submission gate, and no backend
 * VM_BIND, queue creation/close or submit callback runs under it.
 */
struct gpu_m4_drm_vm {
	struct kref ref;
	struct gpu_m4_drm *gpu;
	u64 kernel_start, kernel_end;
	void *backend;
	/* VM-local transaction lock: serializes backend VM_BIND, BO-close prune
	 * and queue construction on THIS VM against its close/destroy, so the
	 * borrowed backend stays open without the global frontend mutex.
	 */
	struct mutex bind_lock;
	atomic_t error;
	bool closed, private;
};

struct gpu_m4_drm_queue {
	struct kref ref; /* namespace + every held submission/destroy */
	/* Per-logical-queue submission gate. Held from backend acceptance
	 * through output-syncobj publication so returned finished fences on one
	 * queue are never reordered; close/destroy drain behind it. Never taken
	 * while holding gpu->lock.
	 */
	struct mutex gate;
	struct gpu_m4_drm_vm *vm;
	void *backend;
	bool closed; /* under gate */
};

struct gpu_m4_drm_object {
	struct drm_gem_object *gem;
	struct gpu_m4_drm_vm *vm;
	void *backend;
	u64 offset, range;
	/* Namespace + in-flight submit lookups; the last put unbinds. */
	struct kref ref;
};

struct gpu_m4_drm_file {
	struct xarray vms, queues, objects;
	struct work_struct close_work;
	struct gpu_m4_drm *gpu;
	bool closing;
};

struct gpu_m4_drm_bo {
	struct drm_gem_shmem_object shmem;
	struct gpu_m4_drm_vm *private_vm;
	struct drm_device *private_drm;
	u64 private_charge;
	bool cache_ready;
};

static const struct drm_driver gpu_m4_drm_driver;
static const struct drm_gem_object_funcs gpu_m4_drm_bo_funcs;

static struct gpu_m4_drm *to_gpu(struct drm_device *drm)
{
	return container_of(drm, struct gpu_m4_drm, drm);
}

static void gpu_m4_drm_vm_free(struct kref *ref)
{
	struct gpu_m4_drm_vm *vm = container_of(ref, struct gpu_m4_drm_vm, ref);
	struct drm_device *drm = &vm->gpu->drm;

	kfree(vm);
	drm_dev_put(drm);
}

void gpu_m4_drm_vm_get(struct gpu_m4_drm_vm *vm)
{
	if (vm)
		kref_get(&vm->ref);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_get);

void gpu_m4_drm_vm_put(struct gpu_m4_drm_vm *vm)
{
	if (vm)
		kref_put(&vm->ref, gpu_m4_drm_vm_free);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_put);

bool gpu_m4_drm_vm_belongs_to_frontend(struct gpu_m4_drm_vm *vm,
				     struct gpu_m4_drm *frontend)
{
	return vm && frontend && vm->gpu == frontend;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_belongs_to_frontend);

void gpu_m4_drm_vm_error(struct gpu_m4_drm_vm *vm, int error)
{
	if (vm && error < 0)
		atomic_cmpxchg(&vm->error, 0, error);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_error);

int gpu_m4_drm_vm_status(struct gpu_m4_drm_vm *vm)
{
	return vm ? atomic_read(&vm->error) : 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_status);

enum gpu_m4_drm_cpu_policy gpu_m4_drm_bo_policy(struct drm_gem_object *object)
{
	if (!gpu_m4_drm_bo_is_native(object))
		return GPU_M4_DRM_CPU_IMPORTED;
	return to_drm_gem_shmem_obj(object)->map_wc ?
		GPU_M4_DRM_CPU_NC : GPU_M4_DRM_CPU_WRITEBACK;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_bo_policy);

bool gpu_m4_drm_bo_is_native(struct drm_gem_object *object)
{
	return object && object->dev && object->dev->driver == &gpu_m4_drm_driver &&
		object->funcs == &gpu_m4_drm_bo_funcs && !drm_gem_is_imported(object);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_bo_is_native);

bool gpu_m4_drm_bo_belongs_to_frontend(struct drm_gem_object *object,
				      struct gpu_m4_drm *frontend)
{
	return frontend && gpu_m4_drm_bo_is_native(object) &&
		object->dev == &frontend->drm;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_bo_belongs_to_frontend);

bool gpu_m4_drm_bo_belongs_to_vm(struct drm_gem_object *object, struct gpu_m4_drm_vm *vm)
{
	struct gpu_m4_drm_bo *bo;

	/* Origin is immutable after construction. The caller must retain both
	 * references; closed VMs can still own already captured/uncertain work.
	 */
	if (!vm || !gpu_m4_drm_bo_is_native(object) || object->dev != &vm->gpu->drm)
		return false;
	bo = container_of(to_drm_gem_shmem_obj(object), struct gpu_m4_drm_bo, shmem);
	return !bo->private_vm || bo->private_vm == vm;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_bo_belongs_to_vm);

/*
 * Allocation/shmem zeroing may dirty the WB direct map before any NC view
 * exists. Clean that initial data exactly once before exposing a CPU view;
 * after this, native WC objects are accessed only through Normal-NC mappings.
 * Explicit WRITEBACK and imported objects require the backend/exporter policy.
 */
int gpu_m4_drm_bo_prepare_locked(struct drm_gem_object *obj)
{
	struct gpu_m4_drm_bo *bo;
	int ret;

	if (!gpu_m4_drm_bo_is_native(obj))
		return -EOPNOTSUPP;
	dma_resv_assert_held(obj->resv);
	bo = container_of(to_drm_gem_shmem_obj(obj), struct gpu_m4_drm_bo, shmem);
	if (!bo->shmem.map_wc)
		return -EOPNOTSUPP;
	if (bo->cache_ready)
		return 0;
	/* Prevent reclaim/swap from creating a fresh WB alias after NC exposure. */
	ret = drm_gem_shmem_pin_locked(&bo->shmem);
	if (ret)
		return ret;
	for (unsigned long i = 0; i < obj->size >> PAGE_SHIFT; i++) {
		unsigned long address = (unsigned long)page_to_virt(bo->shmem.pages[i]);

		dcache_clean_poc(address, address + PAGE_SIZE);
	}
	bo->cache_ready = true;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_bo_prepare_locked);

int gpu_m4_drm_bo_prepare(struct drm_gem_object *obj)
{
	int ret;

	if (!gpu_m4_drm_bo_is_native(obj))
		return -EOPNOTSUPP;
	ret = dma_resv_lock_interruptible(obj->resv, NULL);
	if (ret)
		return ret;
	ret = gpu_m4_drm_bo_prepare_locked(obj);
	dma_resv_unlock(obj->resv);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_bo_prepare);

static int gpu_m4_drm_bo_initial_clean(struct drm_gem_object *obj)
{
	if (gpu_m4_drm_bo_policy(obj) != GPU_M4_DRM_CPU_NC)
		return 0; /* Explicit WB/imported policy belongs to backend/exporter. */
	return gpu_m4_drm_bo_prepare_locked(obj);
}

static int gpu_m4_drm_bo_pin(struct drm_gem_object *obj)
{
	int ret;

	/* PRIME attach already holds this reservation. Its later DMA mapping uses
	 * DMA_ATTR_SKIP_CPU_SYNC, so initialize native NC backing before attach
	 * can expose it, even when no CPU view or client bind has happened yet.
	 */
	dma_resv_assert_held(obj->resv);
	ret = gpu_m4_drm_bo_initial_clean(obj);
	if (ret)
		return ret;
	return drm_gem_shmem_object_pin(obj);
}

static int gpu_m4_drm_bo_vmap(struct drm_gem_object *obj, struct iosys_map *map)
{
	int ret = drm_gem_shmem_object_vmap(obj, map);

	if (!ret) {
		ret = gpu_m4_drm_bo_initial_clean(obj);
		if (ret)
			drm_gem_shmem_object_vunmap(obj, map);
	}
	return ret;
}

static int gpu_m4_drm_bo_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma)
{
	bool imported = drm_gem_is_imported(obj);
	int ret = drm_gem_shmem_object_mmap(obj, vma);

	if (!ret && !imported) {
		dma_resv_lock(obj->resv, NULL);
		ret = gpu_m4_drm_bo_initial_clean(obj);
		if (ret)
			drm_gem_shmem_put_pages_locked(to_drm_gem_shmem_obj(obj));
		dma_resv_unlock(obj->resv);
	}
	return ret;
}

static void gpu_m4_drm_bo_free(struct drm_gem_object *obj)
{
	struct gpu_m4_drm_bo *bo = container_of(to_drm_gem_shmem_obj(obj),
						 struct gpu_m4_drm_bo, shmem);
	struct drm_device *private_drm = bo->private_drm;
	u64 charge = bo->private_charge;

	gpu_m4_drm_vm_put(bo->private_vm);
	if (bo->cache_ready) {
		dma_resv_lock(obj->resv, NULL);
		drm_gem_shmem_unpin_locked(&bo->shmem);
		dma_resv_unlock(obj->resv);
	}
	drm_gem_shmem_release(&bo->shmem);
	kfree(bo);
	if (private_drm) {
		/* Free can run under the frontend mutex: no recursive lock here. */
		atomic64_sub(charge, &to_gpu(private_drm)->native_bytes);
		drm_dev_put(private_drm);
	}
}

static struct dma_buf *gpu_m4_drm_bo_export(struct drm_gem_object *obj, int flags)
{
	struct gpu_m4_drm_bo *bo = container_of(to_drm_gem_shmem_obj(obj),
						 struct gpu_m4_drm_bo, shmem);

	if (bo->private_vm)
		return ERR_PTR(-EPERM);
	return drm_gem_prime_export(obj, flags);
}

static void gpu_m4_drm_bo_close(struct drm_gem_object *obj, struct drm_file *file)
{
	struct gpu_m4_drm *g = to_gpu(obj->dev);
	struct gpu_m4_drm_file *f = file ? file->driver_priv : NULL;
	struct gpu_m4_drm_vm *vm;
	unsigned long id;

	/* Only final fops.release marks closing before DRM drops its GEM handles.
	 * Ordinary GEM_CLOSE must still remove every future active alias.
	 */
	if (!f || READ_ONCE(f->closing) || !g->ops->bo_close)
		return;
	lockdep_assert_not_held(&g->lock);
	/* The namespace mutex only retains each VM. The backend prune runs
	 * under that VM's bind_lock: VM_BIND looks its GEM handles up under the
	 * same lock, so a racing bind either completed first (pruned here) or
	 * observes this already-deleted handle. Other clients never wait; this
	 * file orders only behind its own VMs' in-flight bind transactions.
	 */
	for (id = 0; ; id++) {
		mutex_lock(&g->lock);
		vm = xa_find(&f->vms, &id, ULONG_MAX, XA_PRESENT);
		gpu_m4_drm_vm_get(vm);
		mutex_unlock(&g->lock);
		if (!vm)
			break;
		mutex_lock(&vm->bind_lock);
		if (!vm->closed)
			g->ops->bo_close(g->owner, vm->backend, obj);
		mutex_unlock(&vm->bind_lock);
		gpu_m4_drm_vm_put(vm);
	}
}

static const struct drm_gem_object_funcs gpu_m4_drm_bo_funcs = {
	.free = gpu_m4_drm_bo_free,
	.close = gpu_m4_drm_bo_close,
	.export = gpu_m4_drm_bo_export,
	.print_info = drm_gem_shmem_object_print_info,
	.pin = gpu_m4_drm_bo_pin,
	.unpin = drm_gem_shmem_object_unpin,
	.get_sg_table = drm_gem_shmem_object_get_sg_table,
	.vmap = gpu_m4_drm_bo_vmap,
	.vunmap = drm_gem_shmem_object_vunmap,
	.mmap = gpu_m4_drm_bo_mmap,
	.vm_ops = &drm_gem_shmem_vm_ops,
};

static struct drm_gem_object *gpu_m4_drm_bo_alloc(struct drm_device *drm, size_t size)
{
	struct gpu_m4_drm_bo *bo = kzalloc_obj(*bo);

	if (!bo)
		return ERR_PTR(-ENOMEM);
	/* Also covers generic dumb_create: native allocation must not silently
	 * opt into WB before the provider has qualified it. GEM_CREATE selects
	 * WB explicitly only after its writeback_supported admission gate.
	 */
	bo->shmem.map_wc = true;
	bo->shmem.base.funcs = &gpu_m4_drm_bo_funcs;
	return &bo->shmem.base;
}

u64 gpu_m4_drm_private_bytes(struct gpu_m4_drm *g)
{
	return g ? atomic64_read(&g->native_bytes) : 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_private_bytes);

u64 gpu_m4_drm_bo_private_bytes(struct drm_gem_object *object)
{
	struct gpu_m4_drm_bo *bo;

	if (!gpu_m4_drm_bo_is_native(object))
		return 0;
	bo = container_of(to_drm_gem_shmem_obj(object), struct gpu_m4_drm_bo, shmem);
	return bo->private_drm ? bo->private_charge : 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_bo_private_bytes);

/* Bounded native factory reservation. The charge, the DRM device ref and the
 * optional private-VM ref are actual owned references taken under the
 * frontend mutex; construction either commits all of them to the BO or the
 * refund returns them. Bytes are only ever added under that mutex, so the
 * concurrent lock-free subtraction in final BO free cannot exceed the limit.
 */
struct gpu_m4_drm_native_reservation {
	struct gpu_m4_drm *gpu;
	struct gpu_m4_drm_vm *vm;
	u64 size;
	enum gpu_m4_drm_cpu_policy policy;
};

static int gpu_m4_drm_native_reserve_locked(struct gpu_m4_drm *g, u64 size,
	enum gpu_m4_drm_cpu_policy policy, struct gpu_m4_drm_vm *vm,
	struct gpu_m4_drm_native_reservation *r)
{
	u64 used;

	if (!g || PAGE_SIZE != GPU_M4_DRM_PAGE_SIZE || !size || size > SIZE_MAX ||
	    !gpu_m4_drm_aligned(size) ||
	    (policy != GPU_M4_DRM_CPU_NC && policy != GPU_M4_DRM_CPU_WRITEBACK))
		return -EINVAL;
	lockdep_assert_held(&g->lock);
	if (vm && (vm->gpu != g || vm->closed))
		return -EPERM;
	if (g->registered && policy == GPU_M4_DRM_CPU_WRITEBACK &&
	    !g->ops->writeback_supported)
		return -EOPNOTSUPP;
	used = atomic64_read(&g->native_bytes);
	if (used > g->native_limit || size > g->native_limit - used)
		return -ENOSPC;
	atomic64_add(size, &g->native_bytes);
	gpu_m4_drm_vm_get(vm);
	drm_dev_get(&g->drm);
	*r = (struct gpu_m4_drm_native_reservation) {
		.gpu = g, .vm = vm, .size = size, .policy = policy,
	};
	return 0;
}

/* Never the last DRM/VM reference: the caller retains the frontend/VM. */
static void gpu_m4_drm_native_refund(struct gpu_m4_drm_native_reservation *r)
{
	atomic64_sub(r->size, &r->gpu->native_bytes);
	gpu_m4_drm_vm_put(r->vm);
	drm_dev_put(&r->gpu->drm);
}

/* Construction consumes the reservation on every path. A racing private-VM
 * close cannot revoke it: the held ref keeps the immutable origin valid and
 * the close only stops later admissions, exactly as for an earlier BO.
 */
static struct drm_gem_object *
gpu_m4_drm_native_construct(struct gpu_m4_drm_native_reservation *r)
{
	struct drm_gem_shmem_object *shmem;
	struct gpu_m4_drm_bo *bo;

	shmem = drm_gem_shmem_create(&r->gpu->drm, r->size);
	if (IS_ERR(shmem)) {
		gpu_m4_drm_native_refund(r);
		return ERR_CAST(shmem);
	}
	/* shmem init failure frees the raw allocation WITHOUT calling .free.
	 * Attach references/charge only after complete successful initialization.
	 * Allocation never pinned or exposed a CPU view; policy is still private
	 * and is fixed here, before the object is returned to any caller.
	 */
	bo = container_of(shmem, struct gpu_m4_drm_bo, shmem);
	shmem->map_wc = r->policy == GPU_M4_DRM_CPU_NC;
	bo->private_vm = r->vm;
	bo->private_drm = &r->gpu->drm;
	bo->private_charge = r->size;
	return &shmem->base;
}

/* Legacy entrance for a caller that ALREADY holds the frontend mutex; its
 * construction then necessarily runs under that caller's mutex. No in-tree
 * submit or queue path holds it any more: use gpu_m4_drm_bo_create_native().
 */
struct drm_gem_object *gpu_m4_drm_bo_create_native_locked(struct gpu_m4_drm *g,
	u64 size, enum gpu_m4_drm_cpu_policy policy, struct gpu_m4_drm_vm *vm)
{
	struct gpu_m4_drm_native_reservation r;
	int ret;

	ret = gpu_m4_drm_native_reserve_locked(g, size, policy, vm, &r);
	return ret ? ERR_PTR(ret) : gpu_m4_drm_native_construct(&r);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_bo_create_native_locked);

/* Short reservation under the frontend mutex; shmem allocation and policy
 * selection outside it, so other clients' ioctls never wait on this factory.
 */
struct drm_gem_object *gpu_m4_drm_bo_create_native(struct gpu_m4_drm *g,
	u64 size, enum gpu_m4_drm_cpu_policy policy, struct gpu_m4_drm_vm *vm)
{
	struct gpu_m4_drm_native_reservation r;
	int ret;

	if (!g)
		return ERR_PTR(-EINVAL);
	lockdep_assert_not_held(&g->lock);
	mutex_lock(&g->lock);
	ret = gpu_m4_drm_native_reserve_locked(g, size, policy, vm, &r);
	mutex_unlock(&g->lock);
	return ret ? ERR_PTR(ret) : gpu_m4_drm_native_construct(&r);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_bo_create_native);

/* REMOVE-DEBUG: claim the bounded root probe under the actual frontend mutex
 * just like public ioctl admission, then run its backend callbacks outside
 * it exactly like public QUEUE_CREATE/SUBMIT: the ordinary native factory in
 * queue construction and synchronous staging takes that mutex itself. The
 * one-shot probe_attempted claim excludes a second probe; the caller retains
 * this private VM and never closes it concurrently.
 */
int gpu_m4_drm_probe_submit(struct gpu_m4_drm *g, struct gpu_m4_drm_vm *vm,
	struct drm_gem_object *timestamp, u32 priority, u64 usc_base,
	const struct gpu_m4_drm_submit *submit, struct gpu_m4_drm_probe_result *result)
{
	void *backend;
	struct dma_fence *fence;
	int ret = 0;

	if (!g || !vm || !submit || !result || result->queue ||
	    result->timestamp_backend || result->fence || !submit->lookup_timestamp ||
	    !gpu_m4_drm_bo_belongs_to_frontend(timestamp, g) ||
	    timestamp->size != GPU_M4_DRM_PAGE_SIZE ||
	    gpu_m4_drm_bo_policy(timestamp) != GPU_M4_DRM_CPU_WRITEBACK ||
	    priority > DRM_ASAHI_PRIORITY_REALTIME)
		return -EINVAL;
	lockdep_assert_not_held(&g->lock);
	mutex_lock(&g->lock);
	if (vm->gpu != g || !vm->private || vm->closed || atomic_read(&vm->error) ||
	    g->registration_attempted || g->registered || g->probe_attempted ||
	    !g->ops->object_bind || !g->ops->queue_create || !g->ops->submit)
		ret = -EPERM;
	else
		g->probe_attempted = true;
	mutex_unlock(&g->lock);
	if (ret)
		return ret;
	backend = g->ops->object_bind(g->owner, NULL, timestamp, 0, timestamp->size);
	if (IS_ERR_OR_NULL(backend)) {
		ret = backend ? PTR_ERR(backend) : -EPROTO;
		goto out;
	}
	result->timestamp_backend = backend;
	backend = g->ops->queue_create(g->owner, vm->backend, priority, usc_base);
	if (IS_ERR_OR_NULL(backend)) {
		ret = backend ? PTR_ERR(backend) : -EPROTO;
		goto out;
	}
	result->queue = backend;
	fence = g->ops->submit(g->owner, backend, submit);
	if (IS_ERR_OR_NULL(fence))
		ret = fence ? PTR_ERR(fence) : -EPROTO;
	else
		result->fence = fence;
out:
	return ret < 0 ? ret : (ret ? -EIO : 0);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_probe_submit);

static void gpu_m4_drm_file_close_work(struct work_struct *work);

static int gpu_m4_drm_open(struct drm_device *drm, struct drm_file *file)
{
	struct gpu_m4_drm_file *f = kzalloc_obj(*f);

	if (!f)
		return -ENOMEM;
	xa_init_flags(&f->vms, XA_FLAGS_ALLOC1);
	xa_init_flags(&f->queues, XA_FLAGS_ALLOC1);
	xa_init_flags(&f->objects, XA_FLAGS_ALLOC1);
	INIT_WORK(&f->close_work, gpu_m4_drm_file_close_work);
	f->gpu = to_gpu(drm);
	/* The deferred record outlives DRM's file/minor and fops references. This
	 * real DRM hold also retains the managed READY session/module baseline.
	 */
	drm_dev_get(drm);
	file->driver_priv = f;
	return 0;
}

static void gpu_m4_drm_queue_release(struct kref *ref)
{
	struct gpu_m4_drm_queue *q = container_of(ref, struct gpu_m4_drm_queue, ref);

	gpu_m4_drm_vm_put(q->vm);
	kfree(q);
}

/* Record only: the backend was already closed/destroyed under the gate. */
static void gpu_m4_drm_queue_put(struct gpu_m4_drm_queue *q)
{
	if (q)
		kref_put(&q->ref, gpu_m4_drm_queue_release);
}

/* Namespace already removed. The gate drains any held submission (its
 * scheduler acceptance and output publication) before the backend closes;
 * later gate holders observe closed. Then drop the namespace reference.
 */
static void gpu_m4_drm_queue_free(struct gpu_m4_drm *g, struct gpu_m4_drm_queue *q)
{
	lockdep_assert_not_held(&g->lock);
	mutex_lock(&q->gate);
	if (!q->closed) {
		q->closed = true;
		g->ops->queue_close(g->owner, q->backend);
	}
	mutex_unlock(&q->gate);
	gpu_m4_drm_queue_put(q);
}

static void gpu_m4_drm_object_free(struct gpu_m4_drm *g, struct gpu_m4_drm_object *o)
{
	if (o->vm) {
		/* A held submit can make this final put run after the record left
		 * the namespace, so VM_DESTROY no longer refuses for it. Borrow
		 * the VM backend only while bind_lock proves it is still open.
		 */
		mutex_lock(&o->vm->bind_lock);
		g->ops->object_unbind(g->owner, o->vm->closed ? NULL : o->vm->backend,
				      o->backend);
		mutex_unlock(&o->vm->bind_lock);
	} else {
		g->ops->object_unbind(g->owner, NULL, o->backend);
	}
	drm_gem_object_put(o->gem);
	gpu_m4_drm_vm_put(o->vm);
	kfree(o);
}

static void gpu_m4_drm_object_release(struct kref *ref)
{
	struct gpu_m4_drm_object *o = container_of(ref, struct gpu_m4_drm_object, ref);

	gpu_m4_drm_object_free(to_gpu(o->gem->dev), o);
}

/* The final put unbinds the backend cookie: never under the frontend mutex. */
static void gpu_m4_drm_object_put(struct gpu_m4_drm_object *o)
{
	if (o)
		kref_put(&o->ref, gpu_m4_drm_object_release);
}

static void gpu_m4_drm_file_close_work(struct work_struct *work)
{
	struct gpu_m4_drm_file *f = container_of(work, struct gpu_m4_drm_file, close_work);
	struct gpu_m4_drm *g = f->gpu;
	struct gpu_m4_drm_vm *vm;
	struct gpu_m4_drm_queue *q;
	struct gpu_m4_drm_object *o;
	unsigned long id;

	/* Sole ownership of a dead namespace: no file/filp or core handle tables
	 * survive here. Actual backend callbacks provide their own serialization.
	 */
	lockdep_assert_not_held(&g->lock);
	xa_for_each(&f->queues, id, q) {
		xa_erase(&f->queues, id);
		gpu_m4_drm_queue_free(g, q);
	}
	xa_for_each(&f->objects, id, o) {
		xa_erase(&f->objects, id);
		gpu_m4_drm_object_put(o);
	}
	xa_for_each(&f->vms, id, vm) {
		xa_erase(&f->vms, id);
		mutex_lock(&vm->bind_lock);
		g->ops->vm_close(g->owner, vm->backend);
		WRITE_ONCE(vm->closed, true);
		mutex_unlock(&vm->bind_lock);
		gpu_m4_drm_vm_put(vm);
	}
	xa_destroy(&f->queues);
	xa_destroy(&f->objects);
	xa_destroy(&f->vms);
	kfree(f);
	drm_dev_put(&g->drm);
}

static void gpu_m4_drm_postclose(struct drm_device *drm, struct drm_file *file)
{
	struct gpu_m4_drm_file *f = file ? file->driver_priv : NULL;

	(void)drm;
	if (!f)
		return;
	WRITE_ONCE(f->closing, true);
	file->driver_priv = NULL;
	queue_work(system_unbound_wq, &f->close_work);
}

static int gpu_m4_drm_release(struct inode *inode, struct file *filp)
{
	struct drm_file *file = filp ? filp->private_data : NULL;
	struct gpu_m4_drm_file *f = file ? file->driver_priv : NULL;

	if (!file)
		return 0;
	/* Final VFS release cannot admit another ioctl. Mark BEFORE DRM core's
	 * GEM callbacks, without waiting for any frontend/provider mutex.
	 */
	if (f)
		WRITE_ONCE(f->closing, true);
	return drm_release(inode, filp);
}

static int gpu_m4_drm_get_params(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct gpu_m4_drm *g = to_gpu(drm);
	struct drm_asahi_get_params *a = data;
	u64 size = min_t(u64, a->size, sizeof(g->params));

	if (a->pad || !a->pointer)
		return -EINVAL;
	if (!a->param_group) {
		if (copy_to_user(u64_to_user_ptr(a->pointer), &g->params, size))
			return -EFAULT;
		a->size = size;
		return 0;
	}
	if (a->param_group == DRM_ASAHI_PARAM_GROUP_VM_STATUS &&
	    g->params.features & DRM_ASAHI_FEATURE_VM_STATUS) {
		struct gpu_m4_drm_file *f = file->driver_priv;
		struct drm_asahi_vm_status status;
		struct gpu_m4_drm_vm *vm;

		/* In/out 16-byte prefix; a larger request is accepted. A report
		 * only: never waits, resets or recovers anything.
		 */
		if (a->size < sizeof(status))
			return -EINVAL;
		if (copy_from_user(&status, u64_to_user_ptr(a->pointer), sizeof(status)))
			return -EFAULT;
		if (status.flags || status.pad)
			return -EINVAL;
		mutex_lock(&g->lock);
		vm = xa_load(&f->vms, status.vm_id);
		if (!vm) {
			mutex_unlock(&g->lock);
			return -ENOENT;
		}
		/* A failed shared execution domain fails every VM's future work,
		 * including a VM with no failed job: report it as -EIO here (the
		 * first error still wins). Failed jobs latched their own errors
		 * before their fences signaled.
		 */
		if (g->ops->health && g->ops->health(g->owner) < 0)
			gpu_m4_drm_vm_error(vm, -EIO);
		status.error = atomic_read(&vm->error);
		mutex_unlock(&g->lock);
		return copy_to_user(u64_to_user_ptr(a->pointer), &status, sizeof(status)) ? -EFAULT : 0;
	}
	return -EINVAL;
}

static int gpu_m4_drm_get_time(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct gpu_m4_drm *g = to_gpu(drm);
	struct drm_asahi_get_time *a = data;
	int ret;

	if (a->flags)
		return -EINVAL;
	mutex_lock(&g->lock);
	if (g->ops->read_time) {
		ret = g->ops->read_time(g->owner, &a->gpu_timestamp);
	} else {
		u32 frequency = arch_timer_get_cntfrq();

		if (!frequency) {
			ret = -EIO;
		} else {
			/* J713 publishes CNTPCT in the nanosecond command timebase. */
			a->gpu_timestamp = mul_u64_u32_div(__arch_counter_get_cntpct(),
							 NSEC_PER_SEC, frequency);
			ret = 0;
		}
	}
	mutex_unlock(&g->lock);
	return ret;
}

static struct gpu_m4_drm_vm *gpu_m4_drm_vm_alloc_locked(struct gpu_m4_drm *g,
	u64 kernel_start, u64 kernel_end, bool private)
{
	struct drm_asahi_vm_create a = {
		.kernel_start = kernel_start, .kernel_end = kernel_end,
	};
	struct gpu_m4_drm_vm *vm;
	int ret = gpu_m4_drm_vm_validate(&g->params, &a);

	lockdep_assert_held(&g->lock);
	if (ret)
		return ERR_PTR(ret);
	if (!g->ops->vm_create || !g->ops->vm_destroy || !g->ops->vm_close)
		return ERR_PTR(-EOPNOTSUPP);
	vm = kzalloc_obj(*vm);
	if (!vm)
		return ERR_PTR(-ENOMEM);
	kref_init(&vm->ref);
	mutex_init(&vm->bind_lock);
	vm->gpu = g;
	vm->kernel_start = kernel_start;
	vm->kernel_end = kernel_end;
	vm->private = private;
	atomic_set(&vm->error, 0);
	drm_dev_get(&g->drm);
	vm->backend = g->ops->vm_create(g->owner, vm, kernel_start, kernel_end);
	if (IS_ERR_OR_NULL(vm->backend)) {
		ret = vm->backend ? PTR_ERR(vm->backend) : -EIO;
		gpu_m4_drm_vm_put(vm);
		return ERR_PTR(ret);
	}
	return vm;
}

struct gpu_m4_drm_vm *gpu_m4_drm_vm_create_private(struct gpu_m4_drm *g,
	u64 kernel_start, u64 kernel_end)
{
	struct gpu_m4_drm_vm *vm;

	if (!g)
		return ERR_PTR(-EINVAL);
	mutex_lock(&g->lock);
	vm = gpu_m4_drm_vm_alloc_locked(g, kernel_start, kernel_end, true);
	mutex_unlock(&g->lock);
	return vm;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_create_private);

int gpu_m4_drm_vm_close_private(struct gpu_m4_drm_vm *vm)
{
	struct gpu_m4_drm *g;

	if (!vm || !vm->private)
		return -EINVAL;
	g = vm->gpu;
	mutex_lock(&vm->bind_lock);
	mutex_lock(&g->lock);
	if (!vm->closed) {
		vm->closed = true;
		g->ops->vm_close(g->owner, vm->backend);
	}
	mutex_unlock(&g->lock);
	mutex_unlock(&vm->bind_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_close_private);

void *gpu_m4_drm_vm_backend(struct gpu_m4_drm_vm *vm)
{
	return vm && !READ_ONCE(vm->closed) ? vm->backend : NULL;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_vm_backend);

static int gpu_m4_drm_vm_create(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct gpu_m4_drm *g = to_gpu(drm);
	struct gpu_m4_drm_file *f = file->driver_priv;
	struct drm_asahi_vm_create *a = data;
	struct gpu_m4_drm_vm *vm;
	int ret = gpu_m4_drm_vm_validate(&g->params, a);

	if (ret)
		return ret;
	mutex_lock(&g->lock);
	vm = gpu_m4_drm_vm_alloc_locked(g, a->kernel_start, a->kernel_end, false);
	if (IS_ERR(vm)) {
		ret = PTR_ERR(vm);
		goto unlock;
	}
	ret = xa_alloc(&f->vms, &a->vm_id, vm, XA_LIMIT(1, U32_MAX), GFP_KERNEL);
	if (ret) {
		vm->closed = true;
		g->ops->vm_close(g->owner, vm->backend);
		gpu_m4_drm_vm_put(vm);
	}
unlock:
	mutex_unlock(&g->lock);
	return ret;
}

static int gpu_m4_drm_vm_destroy(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct gpu_m4_drm *g = to_gpu(drm);
	struct gpu_m4_drm_file *f = file->driver_priv;
	struct drm_asahi_vm_destroy *a = data;
	struct gpu_m4_drm_vm *vm;
	struct gpu_m4_drm_queue *q;
	struct gpu_m4_drm_object *o;
	unsigned long id;
	int ret = -ENOENT;

	if (a->pad)
		return -EINVAL;
	mutex_lock(&g->lock);
	vm = xa_load(&f->vms, a->vm_id);
	gpu_m4_drm_vm_get(vm);
	mutex_unlock(&g->lock);
	if (!vm)
		return -ENOENT;
	/* Drain an in-flight VM_BIND/BO prune/queue construction on this VM
	 * without the namespace mutex, then recheck under bind_lock -> lock.
	 */
	ret = mutex_lock_interruptible(&vm->bind_lock);
	if (ret)
		goto put;
	mutex_lock(&g->lock);
	ret = -ENOENT;
	if (xa_load(&f->vms, a->vm_id) != vm)
		goto unlock;
	xa_for_each(&f->queues, id, q)
		if (q->vm == vm) {
			ret = -EBUSY;
			goto unlock;
		}
	xa_for_each(&f->objects, id, o)
		if (o->vm == vm) {
			ret = -EBUSY;
			goto unlock;
		}
	ret = g->ops->vm_destroy(g->owner, vm->backend);
	if (!ret) {
		xa_erase(&f->vms, a->vm_id);
		vm->closed = true;
		gpu_m4_drm_vm_put(vm); /* namespace ref; this call's hold remains */
	}
unlock:
	mutex_unlock(&g->lock);
	mutex_unlock(&vm->bind_lock);
put:
	gpu_m4_drm_vm_put(vm);
	return ret;
}

static int gpu_m4_drm_gem_create(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct gpu_m4_drm *g = to_gpu(drm);
	struct gpu_m4_drm_file *f = file->driver_priv;
	struct drm_asahi_gem_create *a = data;
	struct gpu_m4_drm_vm *vm = NULL;
	struct drm_gem_shmem_object *shmem;
	struct gpu_m4_drm_bo *bo;
	int ret;

	if (a->pad || a->handle || !a->size || a->size > SIZE_MAX - (PAGE_SIZE - 1) ||
	    a->flags & ~(DRM_ASAHI_GEM_WRITEBACK | DRM_ASAHI_GEM_VM_PRIVATE) ||
	    (!(a->flags & DRM_ASAHI_GEM_VM_PRIVATE) && a->vm_id))
		return -EINVAL;
	if ((a->flags & DRM_ASAHI_GEM_WRITEBACK) && !g->ops->writeback_supported)
		return -EOPNOTSUPP;
	if (a->flags & DRM_ASAHI_GEM_VM_PRIVATE) {
		/* Short namespace section: only an owned VM ref leaves it. That
		 * held reservation defines a racing VM_DESTROY/close: creation
		 * linearizes here and completes, like a BO created just before
		 * the close; the closed VM admits no later bind/queue/submit.
		 */
		mutex_lock(&g->lock);
		vm = xa_load(&f->vms, a->vm_id);
		gpu_m4_drm_vm_get(vm);
		mutex_unlock(&g->lock);
		if (!vm)
			return -ENOENT;
	}
	/* Allocation, immutable NC/WB policy and handle installation run
	 * outside the frontend mutex; DRM's own handle table serializes the
	 * handle. Policy is fixed before the handle can expose any CPU view.
	 */
	shmem = drm_gem_shmem_create(drm, a->size);
	if (IS_ERR(shmem)) {
		gpu_m4_drm_vm_put(vm);
		return PTR_ERR(shmem);
	}
	bo = container_of(shmem, struct gpu_m4_drm_bo, shmem);
	shmem->map_wc = !(a->flags & DRM_ASAHI_GEM_WRITEBACK);
	bo->private_vm = vm; /* transfers the held reference */
	ret = drm_gem_handle_create(file, &shmem->base, &a->handle);
	a->size = shmem->base.size;
	drm_gem_object_put(&shmem->base);
	return ret;
}

static int gpu_m4_drm_mmap_offset(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct drm_asahi_gem_mmap_offset *a = data;
	struct drm_gem_object *obj;
	int ret;

	if (a->flags || a->offset)
		return -EINVAL;
	obj = drm_gem_object_lookup(file, a->handle);
	if (!obj)
		return -ENOENT;
	ret = drm_gem_create_mmap_offset(obj);
	if (!ret)
		a->offset = drm_vma_node_offset_addr(&obj->vma_node);
	drm_gem_object_put(obj);
	return ret;
}

static int gpu_m4_drm_vm_bind(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct gpu_m4_drm *g = to_gpu(drm);
	struct gpu_m4_drm_file *f = file->driver_priv;
	struct drm_asahi_vm_bind *a = data;
	struct gpu_m4_drm_bind *binds;
	struct gpu_m4_drm_vm *vm;
	int ret = 0;

	if (a->pad || !a->num_binds || a->num_binds > GPU_M4_DRM_MAX_BINDS ||
	    !a->stride || (a->stride & 7) || a->stride > PAGE_SIZE ||
	    !a->userptr || a->userptr > U64_MAX - (u64)a->num_binds * a->stride)
		return -EINVAL;
	if (!g->ops->vm_bind)
		return -EOPNOTSUPP;
	binds = kcalloc(a->num_binds, sizeof(*binds), GFP_KERNEL);
	if (!binds)
		return -ENOMEM;
	/* Copy every user operation before taking any frontend/VM lock. */
	for (u32 i = 0; i < a->num_binds; i++) {
		ret = copy_struct_from_user(&binds[i].op, sizeof(binds[i].op),
			u64_to_user_ptr(a->userptr + (u64)i * a->stride), a->stride);
		if (ret)
			goto free;
	}
	/* Short namespace section: retain the exact VM, never a raw pointer. */
	mutex_lock(&g->lock);
	vm = xa_load(&f->vms, a->vm_id);
	if (!vm)
		ret = -ENOENT;
	else if (atomic_read(&vm->error))
		ret = -ENODEV;
	else
		gpu_m4_drm_vm_get(vm);
	mutex_unlock(&g->lock);
	if (ret)
		goto free;
	/* VM-local transaction outside the frontend mutex: other VMs, GEM_CREATE
	 * and submits proceed. GEM identities are retained under this lock so a
	 * concurrent GEM_CLOSE prune of this VM orders entirely before (lookup
	 * fails) or after (prunes the new alias) the whole batch. The backend
	 * still validates the whole batch and its captured generation before
	 * any side effect.
	 */
	ret = mutex_lock_interruptible(&vm->bind_lock);
	if (ret)
		goto put_vm;
	if (vm->closed) {
		ret = -ENOENT;
		goto unlock;
	}
	if (atomic_read(&vm->error)) {
		ret = -ENODEV;
		goto unlock;
	}
	for (u32 i = 0; i < a->num_binds; i++) {
		struct gpu_m4_drm_bind *b = &binds[i];
		struct gpu_m4_drm_bo *bo;

		if (!(b->op.flags & DRM_ASAHI_BIND_UNBIND)) {
			b->object = drm_gem_object_lookup(file, b->op.handle);
			if (!b->object) {
				ret = -ENOENT;
				goto unlock;
			}
			bo = container_of(to_drm_gem_shmem_obj(b->object), struct gpu_m4_drm_bo, shmem);
			if (bo->private_vm && bo->private_vm != vm) {
				ret = -EPERM;
				goto unlock;
			}
		}
		ret = gpu_m4_drm_bind_validate(&g->params, vm->kernel_start, vm->kernel_end,
					     &b->op, b->object ? b->object->size : 0);
		if (ret)
			goto unlock;
	}
	ret = g->ops->vm_bind(g->owner, vm->backend, binds, a->num_binds);
unlock:
	mutex_unlock(&vm->bind_lock);
put_vm:
	gpu_m4_drm_vm_put(vm);
free:
	/* Final GEM puts outside every frontend/VM lock. */
	for (u32 i = 0; i < a->num_binds; i++)
		if (binds[i].object)
			drm_gem_object_put(binds[i].object);
	kfree(binds);
	return ret;
}

static int gpu_m4_drm_bind_object(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct gpu_m4_drm *g = to_gpu(drm);
	struct gpu_m4_drm_file *f = file->driver_priv;
	struct drm_asahi_gem_bind_object *a = data;
	struct gpu_m4_drm_object *o;
	struct gpu_m4_drm_bo *bo;
	int ret = 0;

	if (a->pad || a->vm_id)
		return -EINVAL;
	if (!g->ops->object_bind || !g->ops->object_unbind)
		return -EOPNOTSUPP;
	if (a->op == DRM_ASAHI_BIND_OBJECT_OP_UNBIND) {
		if (a->flags || a->handle || a->offset || a->range || !a->object_handle)
			return -EINVAL;
		mutex_lock(&g->lock);
		o = xa_erase(&f->objects, a->object_handle);
		mutex_unlock(&g->lock);
		if (!o)
			return -ENOENT;
		/* A held submission keeps the record; its last put unbinds. */
		gpu_m4_drm_object_put(o);
		return 0;
	}
	mutex_lock(&g->lock);
	if (a->op != DRM_ASAHI_BIND_OBJECT_OP_BIND ||
	    a->flags != DRM_ASAHI_BIND_OBJECT_USAGE_TIMESTAMPS || a->object_handle ||
	    !gpu_m4_drm_aligned(a->offset) || !gpu_m4_drm_aligned(a->range) || !a->range) {
		ret = -EINVAL;
		goto unlock;
	}
	o = kzalloc_obj(*o);
	if (!o) {
		ret = -ENOMEM;
		goto unlock;
	}
	o->gem = drm_gem_object_lookup(file, a->handle);
	if (!o->gem) {
		ret = -ENOENT;
		goto free;
	}
	/* PRIME handles need not contain this frontend's local shmem subtype.
	 * Prove native origin BEFORE reading its private-VM container fields.
	 */
	if (!gpu_m4_drm_bo_is_native(o->gem)) {
		ret = -EOPNOTSUPP;
		goto put;
	}
	if (!gpu_m4_drm_bo_belongs_to_frontend(o->gem, g)) {
		ret = -EPERM;
		goto put;
	}
	if (a->offset >= o->gem->size || a->range > o->gem->size - a->offset) {
		ret = -EINVAL;
		goto put;
	}
	bo = container_of(to_drm_gem_shmem_obj(o->gem), struct gpu_m4_drm_bo, shmem);
	o->vm = bo->private_vm;
	o->offset = a->offset;
	o->range = a->range;
	kref_init(&o->ref);
	if (o->vm && o->vm->closed) {
		ret = -ENOENT;
		goto put;
	}
	o->backend = g->ops->object_bind(g->owner, o->vm ? o->vm->backend : NULL,
					 o->gem, a->offset, a->range);
	if (IS_ERR_OR_NULL(o->backend)) {
		ret = o->backend ? PTR_ERR(o->backend) : -EIO;
		goto put;
	}
	ret = xa_alloc(&f->objects, &a->object_handle, o, XA_LIMIT(1, U32_MAX), GFP_KERNEL);
	if (ret) {
		g->ops->object_unbind(g->owner, o->vm ? o->vm->backend : NULL, o->backend);
		goto put;
	}
	if (o->vm)
		kref_get(&o->vm->ref);
	goto unlock;
put:
	drm_gem_object_put(o->gem);
free:
	kfree(o);
unlock:
	mutex_unlock(&g->lock);
	return ret;
}

static int gpu_m4_drm_queue_create(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct gpu_m4_drm *g = to_gpu(drm);
	struct gpu_m4_drm_file *f = file->driver_priv;
	struct drm_asahi_queue_create *a = data;
	struct gpu_m4_drm_queue *q;
	struct gpu_m4_drm_vm *vm;
	int ret = -ENOENT;

	if (!g->ops->queue_create || !g->ops->queue_destroy || !g->ops->queue_close)
		return -EOPNOTSUPP;
	mutex_lock(&g->lock);
	vm = xa_load(&f->vms, a->vm_id);
	if (vm)
		ret = gpu_m4_drm_queue_validate(&g->params, vm->kernel_start,
					      vm->kernel_end, a);
	/* Host scheduler precedence over other clients is a privilege. */
	if (!ret && gpu_m4_drm_priority_privileged(a->priority) &&
	    !capable(CAP_SYS_NICE))
		ret = -EPERM;
	if (!ret && atomic_read(&vm->error))
		ret = -ENODEV;
	if (!ret)
		gpu_m4_drm_vm_get(vm); /* transferred to the queue on success */
	mutex_unlock(&g->lock);
	if (ret)
		return ret;
	q = kzalloc_obj(*q);
	if (!q) {
		ret = -ENOMEM;
		goto put_vm;
	}
	kref_init(&q->ref);
	mutex_init(&q->gate);
	q->vm = vm;
	/* Backend construction (scheduler, notifier and its native BO via the
	 * unlocked factory) runs outside the frontend mutex. bind_lock keeps
	 * this VM's backend open and orders against VM_DESTROY/close, which
	 * then observe the published queue and refuse/close it.
	 */
	ret = mutex_lock_interruptible(&vm->bind_lock);
	if (ret)
		goto free;
	if (vm->closed) {
		ret = -ENOENT;
		goto unlock_vm;
	}
	q->backend = g->ops->queue_create(g->owner, vm->backend, a->priority, a->usc_exec_base);
	if (IS_ERR_OR_NULL(q->backend)) {
		ret = q->backend ? PTR_ERR(q->backend) : -EIO;
		goto unlock_vm;
	}
	mutex_lock(&g->lock);
	ret = xa_alloc(&f->queues, &a->queue_id, q, XA_LIMIT(1, U32_MAX), GFP_KERNEL);
	mutex_unlock(&g->lock);
	if (ret)
		g->ops->queue_close(g->owner, q->backend);
unlock_vm:
	mutex_unlock(&vm->bind_lock);
	if (!ret)
		return 0;
free:
	kfree(q);
put_vm:
	gpu_m4_drm_vm_put(vm);
	return ret;
}

static int gpu_m4_drm_queue_destroy(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct gpu_m4_drm *g = to_gpu(drm);
	struct gpu_m4_drm_file *f = file->driver_priv;
	struct drm_asahi_queue_destroy *a = data;
	struct gpu_m4_drm_queue *q, *erased = NULL;
	int ret;

	if (a->pad)
		return -EINVAL;
	mutex_lock(&g->lock);
	q = xa_load(&f->queues, a->queue_id);
	if (q)
		kref_get(&q->ref);
	mutex_unlock(&g->lock);
	if (!q)
		return -ENOENT;
	/* Drain this queue's held submission outside the namespace mutex; the
	 * gate is never awaited under it (gate -> lock only). A busy backend
	 * refusal leaves the queue published and unchanged.
	 */
	ret = mutex_lock_interruptible(&q->gate);
	if (ret)
		goto put;
	if (q->closed) {
		ret = -ENOENT;
	} else {
		ret = g->ops->queue_destroy(g->owner, q->backend);
		if (!ret) {
			q->closed = true;
			mutex_lock(&g->lock);
			if (xa_load(&f->queues, a->queue_id) == q)
				erased = xa_erase(&f->queues, a->queue_id);
			mutex_unlock(&g->lock);
		}
	}
	mutex_unlock(&q->gate);
	gpu_m4_drm_queue_put(erased); /* namespace reference */
put:
	gpu_m4_drm_queue_put(q);
	return ret;
}

struct gpu_m4_drm_out_sync {
	struct drm_syncobj *object;
	struct dma_fence_chain *chain;
	u64 point;
};

/* Submit-owned timestamp lookup snapshot. Every record named by the command
 * stream is resolved once in the namespace section and HELD (or recorded as
 * absent), so the backend's synchronous lookup never touches the file
 * namespace or an unheld record after the frontend mutex is released.
 */
struct gpu_m4_drm_lookup_entry {
	u32 handle;
	struct gpu_m4_drm_object *object; /* held reference; NULL if absent */
};

struct gpu_m4_drm_lookup {
	struct gpu_m4_drm_vm *vm; /* retained by the held queue */
	struct gpu_m4_drm_lookup_entry *entries;
	u32 count, capacity;
};

static int gpu_m4_drm_lookup_timestamp(void *cookie, u32 handle, u32 offset,
				     struct gpu_m4_drm_timestamp *timestamp)
{
	struct gpu_m4_drm_lookup *lookup = cookie;
	struct gpu_m4_drm_object *o = NULL;

	for (u32 i = 0; i < lookup->count; i++)
		if (lookup->entries[i].handle == handle) {
			o = lookup->entries[i].object;
			break;
		}
	if (!o)
		return -ENOENT;
	if ((o->vm && o->vm != lookup->vm) ||
	    !gpu_m4_drm_timestamp_range(offset, o->range))
		return -EINVAL;
	timestamp->backend = o->backend;
	timestamp->object = o->gem;
	timestamp->offset = offset;
	timestamp->object_offset = o->offset + offset;
	return 0;
}

/* Pure walk of an already validated stream: each nonzero timestamp handle. */
static int gpu_m4_drm_for_each_timestamp(const void *cmdbuf, u32 size,
	int (*fn)(void *context, const struct drm_asahi_timestamp *timestamp),
	void *context)
{
	const u8 *bytes = cmdbuf;
	u32 offset = 0;

	while (offset < size) {
		struct drm_asahi_cmd_header h;
		struct drm_asahi_timestamp timestamps[12] = { };
		u32 count = 0;

		memcpy(&h, bytes + offset, sizeof(h));
		offset += sizeof(h);
		if (h.cmd_type == DRM_ASAHI_CMD_RENDER) {
			struct drm_asahi_cmd_render r;

			gpu_m4_drm_payload(&r, sizeof(r), bytes + offset, h.size);
			timestamps[0] = r.ts_vtx.start;
			timestamps[1] = r.ts_vtx.end;
			timestamps[2] = r.ts_frag.start;
			timestamps[3] = r.ts_frag.end;
			memcpy(timestamps + 4, r.ts_frag_end_copies, sizeof(r.ts_frag_end_copies));
			count = 12;
		} else if (h.cmd_type == DRM_ASAHI_CMD_COMPUTE) {
			struct drm_asahi_cmd_compute c;

			gpu_m4_drm_payload(&c, sizeof(c), bytes + offset, h.size);
			timestamps[0] = c.ts.start;
			timestamps[1] = c.ts.end;
			memcpy(timestamps + 2, c.ts_end_copies, sizeof(c.ts_end_copies));
			count = 10;
		}
		for (u32 i = 0; i < count; i++) {
			int ret;

			if (!timestamps[i].handle)
				continue;
			ret = fn(context, &timestamps[i]);
			if (ret)
				return ret;
		}
		offset += h.size;
	}
	return 0;
}

static int gpu_m4_drm_lookup_count(void *context, const struct drm_asahi_timestamp *t)
{
	struct gpu_m4_drm_lookup *lookup = context;

	(void)t;
	lookup->capacity++;
	return 0;
}

static int gpu_m4_drm_lookup_add(void *context, const struct drm_asahi_timestamp *t)
{
	struct gpu_m4_drm_lookup *lookup = context;

	for (u32 i = 0; i < lookup->count; i++)
		if (lookup->entries[i].handle == t->handle)
			return 0;
	if (WARN_ON(lookup->count >= lookup->capacity))
		return -EOVERFLOW;
	lookup->entries[lookup->count++].handle = t->handle;
	return 0;
}

/* Outside every lock: collect the distinct handles the stream names. */
static int gpu_m4_drm_lookup_init(struct gpu_m4_drm_lookup *lookup,
				const void *cmdbuf, u32 size)
{
	memset(lookup, 0, sizeof(*lookup));
	gpu_m4_drm_for_each_timestamp(cmdbuf, size, gpu_m4_drm_lookup_count, lookup);
	if (!lookup->capacity)
		return 0;
	lookup->entries = kcalloc(lookup->capacity, sizeof(*lookup->entries), GFP_KERNEL);
	if (!lookup->entries)
		return -ENOMEM;
	return gpu_m4_drm_for_each_timestamp(cmdbuf, size, gpu_m4_drm_lookup_add, lookup);
}

/* Namespace section: hold every named record so unbind cannot free it. */
static void gpu_m4_drm_lookup_hold_locked(struct gpu_m4_drm *g, struct gpu_m4_drm_file *f,
					struct gpu_m4_drm_lookup *lookup)
{
	lockdep_assert_held(&g->lock);
	for (u32 i = 0; i < lookup->count; i++) {
		struct gpu_m4_drm_object *o = xa_load(&f->objects, lookup->entries[i].handle);

		if (o)
			kref_get(&o->ref);
		lookup->entries[i].object = o;
	}
}

/* After the backend returned: these puts may be the last (unbinding). */
static void gpu_m4_drm_lookup_put(struct gpu_m4_drm_lookup *lookup)
{
	for (u32 i = 0; i < lookup->count; i++)
		gpu_m4_drm_object_put(lookup->entries[i].object);
	kfree(lookup->entries);
	memset(lookup, 0, sizeof(*lookup));
}

static int gpu_m4_drm_lookup_check(void *context, const struct drm_asahi_timestamp *t)
{
	struct gpu_m4_drm_timestamp unused;

	return gpu_m4_drm_lookup_timestamp(context, t->handle, t->offset, &unused);
}

static int gpu_m4_drm_validate_timestamps(struct gpu_m4_drm_lookup *lookup,
					const void *cmdbuf, u32 size)
{
	return gpu_m4_drm_for_each_timestamp(cmdbuf, size, gpu_m4_drm_lookup_check, lookup);
}

static void gpu_m4_drm_input_put(struct gpu_m4_drm_dependency *input)
{
	dma_fence_put(input->fence);
	if (input->object)
		drm_syncobj_put(input->object);
	memset(input, 0, sizeof(*input));
}

/* Capture outside the frontend mutex. Future points retain the syncobj rather
 * than waiting here; the accepting backend owns their asynchronous observers.
 */
static int gpu_m4_drm_input_get(struct drm_file *file,
			      const struct drm_asahi_sync *sync,
			      struct gpu_m4_drm_dependency *input)
{
	struct dma_fence *fence;
	int ret;

	memset(input, 0, sizeof(*input));
	if (sync->sync_type > DRM_ASAHI_SYNC_TIMELINE_SYNCOBJ ||
	    (sync->sync_type == DRM_ASAHI_SYNC_SYNCOBJ && sync->timeline_value))
		return -EINVAL;
	input->object = drm_syncobj_find(file, sync->handle);
	if (!input->object)
		return -ENOENT;
	input->point = sync->timeline_value;
	fence = drm_syncobj_fence_get(input->object);
	if (!fence)
		return 0;
	ret = dma_fence_chain_find_seqno(&fence, input->point);
	if (ret) {
		dma_fence_put(fence);
		/* No submitted chain covers the requested point yet. */
		if (ret == -EINVAL)
			return 0;
		gpu_m4_drm_input_put(input);
		return ret;
	}
	/* Chain GC can resolve a completed point to no remaining node. */
	input->fence = fence ?: dma_fence_get_stub();
	return 0;
}

static int gpu_m4_drm_submit(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct gpu_m4_drm *g = to_gpu(drm);
	struct gpu_m4_drm_file *f = file->driver_priv;
	struct drm_asahi_submit *a = data;
	struct gpu_m4_drm_out_sync *outputs;
	struct gpu_m4_drm_dependency *inputs;
	struct dma_fence *fence;
	struct drm_asahi_sync *syncs;
	struct gpu_m4_drm_queue *q = NULL;
	struct gpu_m4_drm_lookup lookup;
	struct gpu_m4_drm_submit submit;
	void *cmdbuf;
	u32 total;
	int ret;

	if (a->flags || a->pad || !a->cmdbuf || !a->cmdbuf_size ||
	    a->cmdbuf > U64_MAX - a->cmdbuf_size ||
	    a->cmdbuf_size > GPU_M4_DRM_MAX_CMDBUF ||
	    a->in_sync_count > GPU_M4_DRM_MAX_SYNCS ||
	    a->out_sync_count > GPU_M4_DRM_MAX_SYNCS - a->in_sync_count)
		return -EINVAL;
	if (!g->ops->submit)
		return -EOPNOTSUPP;
	total = a->in_sync_count + a->out_sync_count;
	if (total && (!a->syncs || a->syncs > U64_MAX - (u64)total * sizeof(*syncs)))
		return -EINVAL;
	cmdbuf = memdup_user(u64_to_user_ptr(a->cmdbuf), a->cmdbuf_size);
	if (IS_ERR(cmdbuf))
		return PTR_ERR(cmdbuf);
	ret = gpu_m4_drm_stream_validate(&g->params, cmdbuf, a->cmdbuf_size);
	if (ret)
		goto free_cmd;
	syncs = memdup_user(u64_to_user_ptr(a->syncs), total * sizeof(*syncs));
	if (IS_ERR(syncs)) {
		ret = PTR_ERR(syncs);
		goto free_cmd;
	}
	inputs = kcalloc(a->in_sync_count, sizeof(*inputs), GFP_KERNEL);
	outputs = kcalloc(a->out_sync_count, sizeof(*outputs), GFP_KERNEL);
	if ((!inputs && a->in_sync_count) || (!outputs && a->out_sync_count)) {
		ret = -ENOMEM;
		goto free_sync;
	}
	for (u32 i = 0; i < total; i++) {
		struct drm_asahi_sync *s = &syncs[i];

		if (s->sync_type > DRM_ASAHI_SYNC_TIMELINE_SYNCOBJ ||
		    (s->sync_type == DRM_ASAHI_SYNC_SYNCOBJ && s->timeline_value)) {
			ret = -EINVAL;
			goto put_sync;
		}
		if (i < a->in_sync_count) {
			ret = gpu_m4_drm_input_get(file, s, &inputs[i]);
			if (ret)
				goto put_sync;
		} else {
			struct gpu_m4_drm_out_sync *o = &outputs[i - a->in_sync_count];

			o->object = drm_syncobj_find(file, s->handle);
			if (!o->object) {
				ret = -ENOENT;
				goto put_sync;
			}
			if (s->sync_type == DRM_ASAHI_SYNC_TIMELINE_SYNCOBJ) {
				if (!s->timeline_value) {
					ret = -EINVAL;
					goto put_sync;
				}
				o->chain = dma_fence_chain_alloc();
				if (!o->chain) {
					ret = -ENOMEM;
					goto put_sync;
				}
				o->point = s->timeline_value;
			}
		}
	}
	ret = gpu_m4_drm_lookup_init(&lookup, cmdbuf, a->cmdbuf_size);
	if (ret)
		goto put_lookup;
	/* Short namespace section: hold the queue (and through it the VM) plus
	 * every named timestamp record. Nothing borrowed from the namespace,
	 * nor this mutex, survives into the backend call.
	 */
	mutex_lock(&g->lock);
	q = xa_load(&f->queues, a->queue_id);
	if (!q)
		ret = -ENOENT;
	else if (atomic_read(&q->vm->error))
		ret = -ENODEV;
	if (!ret) {
		kref_get(&q->ref);
		lookup.vm = q->vm;
		gpu_m4_drm_lookup_hold_locked(g, f, &lookup);
	}
	mutex_unlock(&g->lock);
	if (ret)
		goto put_lookup;
	ret = gpu_m4_drm_validate_timestamps(&lookup, cmdbuf, a->cmdbuf_size);
	if (ret)
		goto put_queue;
	submit = (struct gpu_m4_drm_submit) {
		.cmdbuf = cmdbuf, .size = a->cmdbuf_size,
		.inputs = inputs, .input_count = a->in_sync_count,
		.lookup_cookie = &lookup,
		.lookup_timestamp = gpu_m4_drm_lookup_timestamp,
	};
	/* Per-logical-queue gate, never taken under the frontend mutex: other
	 * queues/clients submit concurrently. It spans scheduler acceptance AND
	 * output-syncobj publication, so two callers on this queue can never
	 * publish their finished fences out of acceptance order.
	 */
	ret = mutex_lock_interruptible(&q->gate);
	if (ret)
		goto put_queue;
	if (q->closed) {
		ret = -ENOENT;
		goto unlock_gate;
	}
	if (atomic_read(&q->vm->error)) {
		ret = -ENODEV;
		goto unlock_gate;
	}
	fence = g->ops->submit(g->owner, q->backend, &submit);
	if (IS_ERR_OR_NULL(fence)) {
		ret = fence ? PTR_ERR(fence) : -EIO;
		goto unlock_gate;
	}
	for (u32 i = 0; i < a->out_sync_count; i++) {
		struct gpu_m4_drm_out_sync *o = &outputs[i];

		if (o->chain) {
			drm_syncobj_add_point(o->object, o->chain, fence, o->point);
			o->chain = NULL; /* transferred to syncobj */
		} else {
			drm_syncobj_replace_fence(o->object, fence);
		}
	}
	dma_fence_put(fence);
	ret = 0;
unlock_gate:
	mutex_unlock(&q->gate);
put_queue:
	gpu_m4_drm_queue_put(q);
put_lookup:
	/* After the synchronous backend lookup: may unbind outside all locks. */
	gpu_m4_drm_lookup_put(&lookup);
put_sync:
	for (u32 i = 0; i < a->in_sync_count; i++)
		gpu_m4_drm_input_put(&inputs[i]);
	for (u32 i = 0; i < a->out_sync_count; i++) {
		if (outputs[i].object)
			drm_syncobj_put(outputs[i].object);
		if (outputs[i].chain)
			dma_fence_chain_free(outputs[i].chain);
	}
free_sync:
	kfree(inputs);
	kfree(outputs);
	kfree(syncs);
free_cmd:
	kfree(cmdbuf);
	return ret;
}

static const struct drm_ioctl_desc gpu_m4_drm_ioctls[] = {
	DRM_IOCTL_DEF_DRV(ASAHI_GET_PARAMS, gpu_m4_drm_get_params, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(ASAHI_GET_TIME, gpu_m4_drm_get_time, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(ASAHI_VM_CREATE, gpu_m4_drm_vm_create, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(ASAHI_VM_DESTROY, gpu_m4_drm_vm_destroy, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(ASAHI_VM_BIND, gpu_m4_drm_vm_bind, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(ASAHI_GEM_CREATE, gpu_m4_drm_gem_create, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(ASAHI_GEM_MMAP_OFFSET, gpu_m4_drm_mmap_offset, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(ASAHI_GEM_BIND_OBJECT, gpu_m4_drm_bind_object, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(ASAHI_QUEUE_CREATE, gpu_m4_drm_queue_create, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(ASAHI_QUEUE_DESTROY, gpu_m4_drm_queue_destroy, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(ASAHI_SUBMIT, gpu_m4_drm_submit, DRM_RENDER_ALLOW),
};

/* DRM core zero-extends short main structures; check longer input tails here. */
static long gpu_m4_drm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned int nr = DRM_IOCTL_NR(cmd);

	if (DRM_IOCTL_TYPE(cmd) == DRM_IOCTL_BASE && nr >= DRM_COMMAND_BASE &&
	    nr - DRM_COMMAND_BASE < ARRAY_SIZE(gpu_m4_drm_ioctls)) {
		unsigned int known = _IOC_SIZE(gpu_m4_drm_ioctls[nr - DRM_COMMAND_BASE].cmd);
		unsigned int size = _IOC_SIZE(cmd);
		int ret;

		if (size > known && (cmd & IOC_IN)) {
			if (arg > ULONG_MAX - known)
				return -EFAULT;
			ret = check_zeroed_user((void __user *)(arg + known), size - known);
			if (ret <= 0)
				return ret ? ret : -E2BIG;
		}
	}
	return drm_ioctl(file, cmd, arg);
}

static const struct file_operations gpu_m4_drm_fops = {
	.owner = THIS_MODULE,
	.fop_flags = FOP_UNSIGNED_OFFSET,
	.open = drm_open,
	.release = gpu_m4_drm_release,
	.unlocked_ioctl = gpu_m4_drm_ioctl,
	.compat_ioctl = gpu_m4_drm_ioctl,
	.poll = drm_poll,
	.read = drm_read,
	.llseek = noop_llseek,
	.mmap = drm_gem_mmap,
};

static const struct drm_driver gpu_m4_drm_driver = {
	DRM_GEM_SHMEM_DRIVER_OPS,
	.gem_create_object = gpu_m4_drm_bo_alloc,
	.name = "asahi",
	.desc = "J713 retained GPU",
	.major = 1,
	.driver_features = DRIVER_GEM | DRIVER_RENDER | DRIVER_SYNCOBJ |
			   DRIVER_SYNCOBJ_TIMELINE | DRIVER_GEM_GPUVA,
	.open = gpu_m4_drm_open,
	.postclose = gpu_m4_drm_postclose,
	.ioctls = gpu_m4_drm_ioctls,
	.num_ioctls = ARRAY_SIZE(gpu_m4_drm_ioctls),
	.fops = &gpu_m4_drm_fops,
};

static void gpu_m4_drm_session_release(struct drm_device *drm, void *data)
{
	struct gpu_m4_drm *g = to_gpu(drm);

	(void)data;
	/* Only a separately qualified lifetime owner may eventually arrange the
	 * final DRM put. Registration failure never consumes the READY baseline.
	 */
	lockdep_assert_not_held(&g->lock);
	g->ops->session_put(g->owner);
	module_put(THIS_MODULE);
}

/* The qualified J713 G16G profile. */
static bool gpu_m4_drm_params_j713(const struct drm_asahi_params_global *params)
{
	if (!of_machine_is_compatible("apple,j713") ||
	    !of_machine_is_compatible("apple,t8132") ||
	    params->gpu_generation != 16 || params->gpu_variant != 'G' ||
	    params->gpu_revision != 0x10 ||
	    params->chip_id != 0x8132 || params->num_dies != 1 ||
	    params->num_clusters_total != 1 || params->num_cores_per_cluster != 10 ||
	    !params->core_masks[0] || (params->core_masks[0] & ~0x3ffULL) ||
	    params->usc_generation != 3 || params->gpu_hal_generation != DRM_ASAHI_GPU_HAL_200 ||
	    params->vm_start != 0x4000 || params->vm_end != (1ULL << 42) - 0x8000 ||
	    params->vm_kernel_min_size != 0x20000000 || !params->max_frequency_khz ||
	    !params->max_commands_per_submission || params->max_commands_per_submission > 64 ||
	    !params->max_attachments || params->max_attachments > 16 ||
	    params->command_timestamp_frequency_hz != 1000000000 ||
	    (params->features & ~GPU_M4_DRM_FEATURES))
		return false;
	for (u32 i = 1; i < DRM_ASAHI_MAX_CLUSTERS; i++)
		if (params->core_masks[i])
			return false;
	return true;
}

/*
 * The T6030 G15S (M3 Pro, B1) compute profile, J514S or J516S: 2 x 10 logical
 * cores with a per-machine (binned) non-empty enabled set in each cluster, a
 * per-machine top frequency, legacy HAL, 0x2ffffff8000 user VA end. Features
 * are a subset of VM_STATUS | SCHEDULED_QUEUES | COMPUTE_WIDE_VISIBILITY |
 * FRAGMENT_DEPENDENCY.
 */
#define GPU_M4_DRM_J514S_FEATURES (DRM_ASAHI_FEATURE_VM_STATUS | \
	DRM_ASAHI_FEATURE_SCHEDULED_QUEUES | DRM_ASAHI_FEATURE_COMPUTE_WIDE_VISIBILITY | \
	DRM_ASAHI_FEATURE_FRAGMENT_DEPENDENCY)

static bool gpu_m4_drm_params_j514s(const struct drm_asahi_params_global *params)
{
	if (!apple_pmp_t6030_board() ||
	    params->gpu_generation != 15 || params->gpu_variant != 'S' ||
	    params->gpu_revision != 0x11 || params->chip_id != 0x6030 ||
	    params->num_dies != 1 || params->num_clusters_total != 2 ||
	    params->num_cores_per_cluster != 10 ||
	    !params->core_masks[0] || (params->core_masks[0] & ~0x3ffULL) ||
	    !params->core_masks[1] || (params->core_masks[1] & ~0x3ffULL) ||
	    params->usc_generation != 3 ||
	    params->gpu_hal_generation != DRM_ASAHI_GPU_HAL_LEGACY ||
	    !params->max_frequency_khz || params->max_frequency_khz > 3000000 ||
	    params->vm_start != 0x4000 || params->vm_end != 0x2ffffff8000ULL ||
	    params->vm_kernel_min_size != 0x20000000 ||
	    params->max_commands_per_submission != 64 || params->max_attachments != 16 ||
	    params->command_timestamp_frequency_hz != 1000000000 ||
	    (params->features & ~(u64)GPU_M4_DRM_J514S_FEATURES))
		return false;
	for (u32 i = 2; i < DRM_ASAHI_MAX_CLUSTERS; i++)
		if (params->core_masks[i])
			return false;
	return true;
}

struct gpu_m4_drm *gpu_m4_drm_create_private(struct device *dev, void *owner,
	const struct drm_asahi_params_global *params, const struct gpu_m4_drm_ops *ops,
	u64 native_limit)
{
	struct gpu_m4_drm *g;
	int ret;

	if (!dev || !owner || !params || !ops || !ops->session_get || !ops->session_put ||
	    PAGE_SIZE != GPU_M4_DRM_PAGE_SIZE || native_limit > S64_MAX ||
	    !gpu_m4_drm_aligned(native_limit) ||
	    (!gpu_m4_drm_params_j713(params) && !gpu_m4_drm_params_j514s(params)))
		return ERR_PTR(-EINVAL);
	g = devm_drm_dev_alloc(dev, &gpu_m4_drm_driver, struct gpu_m4_drm, drm);
	if (IS_ERR(g))
		return g;
	mutex_init(&g->lock);
	g->owner = owner;
	g->copied_ops = *ops;
	g->copied_ops.writeback_supported = false;
	g->ops = &g->copied_ops;
	g->params = *params;
	g->native_limit = native_limit;
	atomic64_set(&g->native_bytes, 0);
	ret = g->ops->session_get(owner);
	if (ret)
		return ERR_PTR(ret < 0 ? ret : -EIO);
	__module_get(THIS_MODULE);
	ret = drmm_add_action_or_reset(&g->drm, gpu_m4_drm_session_release, NULL);
	if (ret)
		return ERR_PTR(ret);
	/* Devres owns one initial ref. READY keeps this distinct explicit baseline
	 * even if register fails or its own devres reference is later released.
	 * The actual session, not get_device alone, retains all hardware owners.
	 */
	drm_dev_get(&g->drm);
	return g;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_create_private);

int gpu_m4_drm_register_ready(struct gpu_m4_drm *g, bool wb_qualified)
{
	const struct gpu_m4_drm_ops *ops;
	int ret;

	if (!g)
		return -EINVAL;
	mutex_lock(&g->lock);
	ops = g->ops;
	if (g->registration_attempted) {
		ret = g->registered ? -EALREADY : g->registration_error;
		goto unlock;
	}
	if (!ops->read_time || !ops->vm_create || !ops->vm_destroy || !ops->vm_close ||
	    !ops->vm_bind || !ops->bo_close || !ops->object_bind || !ops->object_unbind ||
	    !ops->queue_create || !ops->queue_destroy || !ops->queue_close || !ops->submit) {
		ret = -EOPNOTSUPP;
		goto unlock;
	}
	g->registration_attempted = true;
	/* Set before node visibility; ioctl acceptance takes this same mutex. */
	g->copied_ops.writeback_supported = wb_qualified;
	ret = drm_dev_register(&g->drm, 0);
	if (ret) {
		g->registration_error = ret;
		g->copied_ops.writeback_supported = false;
	} else {
		g->registered = true;
	}
unlock:
	mutex_unlock(&g->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_register_ready);

struct gpu_m4_drm *gpu_m4_drm_register(struct device *dev, void *owner,
	const struct drm_asahi_params_global *params, const struct gpu_m4_drm_ops *ops)
{
	struct gpu_m4_drm *g = gpu_m4_drm_create_private(dev, owner, params, ops, 0);
	int ret;

	if (IS_ERR(g))
		return g;
	ret = gpu_m4_drm_register_ready(g, ops->writeback_supported);
	/* A failure is NOT authority to release a potentially visible owner. */
	return ret ? ERR_PTR(ret) : g;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_register);

MODULE_DESCRIPTION("J713 retained GPU Mesa ABI frontend");
MODULE_LICENSE("GPL");
