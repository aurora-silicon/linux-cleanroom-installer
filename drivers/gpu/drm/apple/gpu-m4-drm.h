/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_DRM_H
#define APPLE_GPU_M4_DRM_H

#include <linux/types.h>
#include "gpu-m4-asahi-drm.h"

/*
 * Global feature bits this frontend may advertise (0x1fa): sticky VM status,
 * scheduled logical queues with four host priorities, render/compute
 * end-timestamp copies, fragment-only dependency declarations (early tiling
 * inside a render-only window; the compute form is accepted and conservatively
 * keeps full order across compute) and the compute wide-visibility
 * declaration. Never SOFT_FAULTS, INDEPENDENT_QUEUES or (yet)
 * CACHED_PROGRAM_PROBE.
 */
enum {
	GPU_M4_DRM_FEATURES = DRM_ASAHI_FEATURE_VM_STATUS |
		DRM_ASAHI_FEATURE_SCHEDULED_QUEUES |
		DRM_ASAHI_FEATURE_RENDER_TIMESTAMP_COPIES |
		DRM_ASAHI_FEATURE_COMPUTE_TIMESTAMP_COPIES |
		DRM_ASAHI_FEATURE_FRAGMENT_DEPENDENCY |
		DRM_ASAHI_FEATURE_FRAGMENT_DEPENDENCY_COMPUTE |
		DRM_ASAHI_FEATURE_COMPUTE_WIDE_VISIBILITY,
};

struct device;
struct dma_fence;
struct drm_gem_object;
struct drm_syncobj;
struct gpu_m4_drm;
struct gpu_m4_drm_vm;

struct gpu_m4_drm_bind {
	struct drm_asahi_gem_bind_op op;
	struct drm_gem_object *object; /* NULL for unbind; borrowed during callback */
};

struct gpu_m4_drm_timestamp {
	void *backend; /* timestamp-object cookie, borrowed during submit */
	struct drm_gem_object *object;
	u64 offset; /* byte offset relative to the bound object */
	u64 object_offset; /* byte offset into GEM backing, including bind offset */
};

enum gpu_m4_drm_cpu_policy {
	GPU_M4_DRM_CPU_NC,
	GPU_M4_DRM_CPU_WRITEBACK,
	GPU_M4_DRM_CPU_IMPORTED,
};

/* References are borrowed only for the synchronous accepting callback. */
struct gpu_m4_drm_dependency {
	struct drm_syncobj *object;
	struct dma_fence *fence; /* exact captured dependency, NULL if not submitted */
	u64 point;
};

struct gpu_m4_drm_submit {
	const void *cmdbuf;
	u32 size;
	const struct gpu_m4_drm_dependency *inputs;
	u32 input_count;
	void *lookup_cookie; /* synchronous only; never retain the file/stack lookup */
	int (*lookup_timestamp)(void *cookie, u32 handle, u32 offset,
				struct gpu_m4_drm_timestamp *timestamp);
};

/* REMOVE-DEBUG: one ordinary pre-registration coherency probe. Root retains
 * these exact returned backend owners until reboot, including partial failure.
 * timestamp_backend is installed before submit's synchronous timestamp lookup.
 * No sealed diagnostic queue, alternate runtime or public WB admission.
 */
struct gpu_m4_drm_probe_result {
	void *queue, *timestamp_backend;
	struct dma_fence *fence;
};
int gpu_m4_drm_probe_submit(struct gpu_m4_drm *gpu, struct gpu_m4_drm_vm *vm,
	struct drm_gem_object *timestamp, u32 priority, u64 usc_base,
	const struct gpu_m4_drm_submit *submit, struct gpu_m4_drm_probe_result *result);

/*
 * This frontend never owns a mailbox, power domain or firmware table.
 * Registration belongs to the retained owner after READY and NOP retirement.
 * The frontend's device mutex is a SHORT namespace/admission lock: it is held
 * only across vm_create, vm_destroy, ioctl object_bind and read_time. vm_bind,
 * bo_close and queue_create run under that VM's frontend bind lock instead;
 * submit runs under its logical queue's submission gate; queue_close/destroy
 * and the final object_unbind run outside the device mutex (a private-VM
 * object's under that VM's bind lock). Different VMs and queues therefore
 * call concurrently, and vm_bind on a VM races submit on that VM's queues:
 * the backend serializes its own state, captures the exact mapping owner
 * atomically with its active-owner swap and commits against captured
 * generations. A borrowed VM backend stays open for the whole call; a
 * borrowed queue backend is never closed while submitting.
 * Backend-held GEM references and firmware memory must survive uncertain
 * completion and close.
 */
struct gpu_m4_drm_ops {
	/* Retain the actual READY/power/RTKit owner, not just its device cookie.
	 * A successful get is held through the frontend's retained DRM baseline.
	 * Final put runs outside frontend/provider/reservation locks.
	 */
	int (*session_get)(void *owner);
	void (*session_put)(void *owner);
	/* Enable only after clean/invalidate around every GPU access is qualified. */
	bool writeback_supported;
	int (*read_time)(void *owner, u64 *timestamp_ns);
	/* Optional: negative once the shared execution domain has failed (no
	 * recovery exists), else 0. Lockless sticky read under the frontend
	 * mutex: no wait, owner lock, hardware access or reset.
	 */
	int (*health)(void *owner);
	void *(*vm_create)(void *owner, struct gpu_m4_drm_vm *vm,
			   u64 kernel_start, u64 kernel_end);
	int (*vm_destroy)(void *owner, void *vm); /* refuse while jobs/queues remain */
	void (*vm_close)(void *owner, void *vm); /* close, pin uncertain mappings */
	int (*vm_bind)(void *owner, void *vm,
		       const struct gpu_m4_drm_bind *binds, u32 count);
	void (*bo_close)(void *owner, void *vm, struct drm_gem_object *object);
	void *(*object_bind)(void *owner, void *vm, struct drm_gem_object *object,
			    u64 offset, u64 size);
	void (*object_unbind)(void *owner, void *vm, void *object);
	void *(*queue_create)(void *owner, void *vm, u32 priority, u64 usc_base);
	int (*queue_destroy)(void *owner, void *queue); /* refuse busy explicit destroy */
	void (*queue_close)(void *owner, void *queue); /* stop new jobs, pin uncertain backing */
	/*
	 * ERR_PTR means the job was not accepted and no GPU work was published.
	 * Called WITHOUT the frontend device mutex, under only this logical
	 * queue's submission gate (held through output-syncobj publication).
	 * An accepted job returns its finished fence promptly: never wait for an
	 * input submission or execution under that gate. A NULL input
	 * fence is a future dependency, NOT an already satisfied one. Retain its
	 * syncobj/point and use backend-owned async observers plus drm_sched; do
	 * not publish work until the actual dependency has arrived and completed.
	 * A captured non-NULL fence must remain the exact dependency even if its
	 * syncobj is subsequently replaced (particularly for binary inputs).
	 *
	 * Initialize all observer-used job state before registration, serialize
	 * observer add against remove/reuse/free with the backend job owner, and
	 * cancel observers on rejected/canceled jobs and queue close. Publication
	 * callbacks only get a real fence and queue referenced work; no dependency
	 * or teardown waits may occur under frontend/owner locks. Retain callback
	 * storage through all concurrent removers and queued work.
	 *
	 * Copy the command bytes, own all input syncobj/fence, queue, VM and GEM
	 * references, and snapshot timestamp records/mapping references before
	 * returning. The lookup cookie and returned timestamp backend cookies
	 * are borrowed for this call only: the frontend holds every looked-up
	 * record until submit returns, but no asynchronous lookup is permitted
	 * after it. Keep the
	 * frontend VM alive until the error callback is no longer possible.
	 *
	 * Publish this finished fence in accessed BO reservations before returning,
	 * so PRIME importers wait for actual rendering before scanout. Success
	 * means full qualified GPU/cache retirement, never merely submission or a
	 * dependency becoming known. A potentially published GPU job that fails
	 * must latch gpu_m4_drm_vm_error() before signaling its error fence and
	 * retain every uncertain firmware-visible allocation.
	 */
	struct dma_fence *(*submit)(void *owner, void *queue,
				  const struct gpu_m4_drm_submit *submit);
};

struct gpu_m4_drm *gpu_m4_drm_register(struct device *dev, void *owner,
				     const struct drm_asahi_params_global *params,
				     const struct gpu_m4_drm_ops *ops);
/* No device node or hardware publication. Ops are copied and userspace WB
 * admission starts false. Only session_get/put are mandatory here. A separate
 * explicit DRM baseline retains the powered owner independently of devres.
 * There is deliberately no live destroy/unbind or baseline-drop operation.
 * native_limit is an aligned finite private BO budget; zero disables factory
 * allocations. It does not replace provider pin/table/private-pool budgets.
 */
struct gpu_m4_drm *gpu_m4_drm_create_private(struct device *dev, void *owner,
	const struct drm_asahi_params_global *params, const struct gpu_m4_drm_ops *ops,
	u64 native_limit);
/* Root alone calls this after actual runtime/render/cache qualification. All
 * callbacks must exist. One registration attempt only; failure keeps EVERY
 * owner and the baseline retained and leaves userspace WB admission false.
 * wb_qualified is evidence supplied by that actual owner, not a host test.
 */
int gpu_m4_drm_register_ready(struct gpu_m4_drm *gpu, bool wb_qualified);
/* Owned real frontend VM, with its own DRM reference. Close stops new work
 * through the real callback but does not consume the caller's reference or
 * release uncertain hardware. Put the caller reference separately. Backend
 * is borrowed only while the VM remains open and retained; do not race close.
 */
struct gpu_m4_drm_vm *gpu_m4_drm_vm_create_private(struct gpu_m4_drm *gpu,
	u64 kernel_start, u64 kernel_end);
int gpu_m4_drm_vm_close_private(struct gpu_m4_drm_vm *vm);
void *gpu_m4_drm_vm_backend(struct gpu_m4_drm_vm *vm);
/* Qualification-only WB allocation while unregistered does NOT enable the
 * userspace ioctl. Policy is fixed before any pin/CPU view. Same-device open
 * private_vm is optional. Returns one GEM reference and retains the frontend;
 * charge survives every extra GEM/pin/job ref until final native BO free.
 * Never call factory/close/final puts under provider or reservation locks.
 * Takes the frontend mutex only for the bounded charge/ref reservation; the
 * shmem allocation runs outside it. Callable from queue_create (VM bind lock
 * held) and submit staging (queue gate held), never under the frontend mutex.
 */
struct drm_gem_object *gpu_m4_drm_bo_create_native(struct gpu_m4_drm *gpu,
	u64 size, enum gpu_m4_drm_cpu_policy policy, struct gpu_m4_drm_vm *private_vm);
/* Legacy: only a caller ALREADY holding the frontend mutex; construction then
 * runs under it. No frontend callback holds that mutex any more: queue
 * creation and (synchronous or deferred) submit staging must use the locking
 * factory above, whose short reservation section never spans the shmem
 * allocation. Provider/GPU and GEM reservation locks remain unheld. The same
 * native origin, policy/admission, VM lifetime and finite charge checks
 * apply; no handle or hardware publication is created.
 */
struct drm_gem_object *gpu_m4_drm_bo_create_native_locked(struct gpu_m4_drm *gpu,
	u64 size, enum gpu_m4_drm_cpu_policy policy, struct gpu_m4_drm_vm *private_vm);
u64 gpu_m4_drm_private_bytes(struct gpu_m4_drm *gpu);
/* REMOVE-DEBUG: this GEM's own native factory charge, 0 for any other. */
u64 gpu_m4_drm_bo_private_bytes(struct drm_gem_object *object);
/* Check BEFORE interpreting an arbitrary GEM object as this native subtype. */
bool gpu_m4_drm_bo_is_native(struct drm_gem_object *object);
/* File-global object binding checks only immutable native frontend origin.
 * Queue-time pinning must still enforce bo_belongs_to_vm(), including private
 * BO restrictions. This never associates or pins a BO with a NULL VM.
 */
bool gpu_m4_drm_bo_belongs_to_frontend(struct drm_gem_object *object,
				      struct gpu_m4_drm *frontend);
/* Immutable native frontend/optional-private-VM origin. Keep both references;
 * closed status does not revoke an already captured owner's association.
 */
bool gpu_m4_drm_bo_belongs_to_vm(struct drm_gem_object *object, struct gpu_m4_drm_vm *vm);
void gpu_m4_drm_vm_error(struct gpu_m4_drm_vm *vm, int error);
/* The sticky first error (0 while none): a lockless report, not a gate. */
int gpu_m4_drm_vm_status(struct gpu_m4_drm_vm *vm);
void gpu_m4_drm_vm_get(struct gpu_m4_drm_vm *vm);
void gpu_m4_drm_vm_put(struct gpu_m4_drm_vm *vm);
/* Immutable origin only. Caller retains the VM and frontend; a closed VM's
 * captured work keeps the same origin. This does not grant new-work admission.
 */
bool gpu_m4_drm_vm_belongs_to_frontend(struct gpu_m4_drm_vm *vm,
				     struct gpu_m4_drm *frontend);
enum gpu_m4_drm_cpu_policy gpu_m4_drm_bo_policy(struct drm_gem_object *object);
/* Native NC only: allocates/pins and cleans initial data; no DMA/PTE publication. */
int gpu_m4_drm_bo_prepare_locked(struct drm_gem_object *object);
int gpu_m4_drm_bo_prepare(struct drm_gem_object *object);

#endif
