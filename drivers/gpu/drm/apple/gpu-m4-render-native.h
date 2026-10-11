/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_RENDER_NATIVE_H
#define APPLE_GPU_M4_RENDER_NATIVE_H

#include "gpu-m4-drm-provider-job.h"
#include "gpu-m4-render-packet.h"

/* A retained READY allocator owner, not an arbitrary PA or CPU mapping. Its
 * actual representation and alias/guard proofs belong to the live provider.
 * Inherited DMA storage need not masquerade as a native GEM allocation.
 */
struct gpu_m4_render_ready_backing;
struct gpu_m4_drm_upper_lease;

enum gpu_m4_render_native_backing_kind {
	GPU_M4_RENDER_NATIVE_GEM = 1,
	GPU_M4_RENDER_NATIVE_READY,
	GPU_M4_RENDER_NATIVE_CACHE,
};

struct gpu_m4_render_native_backing {
	struct gpu_m4_render_span span;
	enum gpu_m4_render_native_backing_kind kind;
	union {
		struct gpu_m4_drm_provider_private_gem gem;
		struct gpu_m4_render_ready_backing *ready;
		struct {
			/* Firmware-only NC native range already joined into the exact
			 * snapshot/cache/resv. The actual upper owner proves its whole
			 * canonical lease, PA inventory, class and logical subrange.
			 */
			struct gpu_m4_drm_provider_object *object;
			struct gpu_m4_drm_upper_lease *upper_lease;
			u64 offset, size, firmware_va;
			u32 access;
		} cache;
	};
};

static inline bool gpu_m4_render_native_same_backing(
		const struct gpu_m4_render_native_backing *a,
		const struct gpu_m4_render_native_backing *b)
{
	if (a->kind != b->kind)
		return false;
	switch (a->kind) {
	case GPU_M4_RENDER_NATIVE_READY:
		return a->ready && a->ready == b->ready;
	case GPU_M4_RENDER_NATIVE_GEM:
		return a->gem.object == b->gem.object && a->gem.lease == b->gem.lease &&
			a->gem.va == b->gem.va && a->gem.offset == b->gem.offset &&
			a->gem.size == b->gem.size && a->gem.access == b->gem.access;
	case GPU_M4_RENDER_NATIVE_CACHE:
		return a->cache.object == b->cache.object && a->cache.upper_lease == b->cache.upper_lease &&
			a->cache.offset == b->cache.offset && a->cache.size == b->cache.size &&
			a->cache.firmware_va == b->cache.firmware_va && a->cache.access == b->cache.access;
	default:
		return false;
	}
}

/* Immutable borrowed staging result, strongly retained by the actual job.
 * resource[VDM] MUST be empty: VDM belongs to the admitted Mesa command, never
 * a private fixture allocation. LAYER_METADATA is a subrange of SCENE_METADATA,
 * at +0x200 for layered work. Every other allocation is independently retained.
 */
struct gpu_m4_render_native_owner {
	void *stage_owner;
	struct gpu_m4_drm_provider_vm *vm;
	struct gpu_m4_render_owner render;
	struct gpu_m4_render_native_backing resource[GPU_M4_RENDER_ENCODED_RESOURCES];
	struct gpu_m4_render_native_backing parameter_heap;
	struct gpu_m4_render_native_backing firmware[GPU_M4_RENDER_FIRST_FW_COUNT];
	struct gpu_m4_render_native_backing gpu[GPU_M4_RENDER_FIRST_GPU_COUNT];
	struct gpu_m4_render_native_backing manager, descriptor, tpc_firmware;
	struct gpu_m4_render_native_backing init_buffer, fragment_dependency;
	u32 uma_slot;
};

/* Exact command/native span admission memo, owned by the caller's held packet.
 * One complete validation of this exact held job/version/snapshot/command/
 * owner tuple records it. A later validation of the SAME tuple, only after
 * check_owner has re-proved the live lane/core/binding/lease/cache authority,
 * reuses the immutable span work: snapshot and installed-version page walks,
 * native pin/PA inventories, native cache coverage and timestamp membership.
 * Structure, disjointness, chronology, private leases and every owner callback
 * still run. Never a protection, publication, retirement or health grant; a
 * failed validation or any other tuple discards it.
 */
struct gpu_m4_render_native_admission {
	const struct gpu_m4_drm_provider_job *job;
	const struct gpu_m4_drm_provider_version *version;
	const struct gpu_m4_drm_vm_owner *snapshot;
	const struct gpu_m4_drm_job_render *command;
	u32 command_index;
	bool valid;
	struct gpu_m4_render_native_owner owner;
};

struct gpu_m4_render_native_input;
/* Mandatory actual owner proof under the ONE provider mutex. Establish the
 * exact job/command/staging/version/snapshot/binding tuple, each typed backing
 * and full pin/PA/policy/access inventory, private guards and distinct backing,
 * same-PA FW/client/context-0 fetch/TPC aliases, PB/UMA/event/notifier/queue
 * leases and frozen chronology. Check complete native cache/resv ownership,
 * not just attachments. The queue notifier is its own retained owner, not
 * implicitly VM-shared. Each render client additionally retains the actual
 * READY global UMA AND parameter table external claims and guarded mappings.
 * Preparation is detached; actual publication separately
 * requires dependencies, prepared caches, protected current job and lane.
 * Do not inspect executable payloads or claim this is a hardware witness.
 */
typedef int (*gpu_m4_render_check_native_owner)(void *cookie,
		const struct gpu_m4_render_native_input *in);

struct gpu_m4_render_native_input {
	struct gpu_m4_drm_provider *provider;
	struct gpu_m4_drm_provider_job *job;
	u32 command_index;
	const struct gpu_m4_drm_job_render *command;
	struct gpu_m4_drm_provider_version *version;
	const struct gpu_m4_drm_vm_owner *snapshot;
	const struct gpu_m4_render_native_owner *owner;
	const struct gpu_m4_render_retained_binding *binding;
	const struct gpu_m4_render_packet_chronology *chronology;
	u32 uuid; /* Provider diagnostic UUID, not a UAPI operation sequence. */
	void *owner_cookie;
	gpu_m4_render_check_native_owner check_owner;
	/* Optional; NULL performs the complete span admission at every call. */
	struct gpu_m4_render_native_admission *admission;
};

/* Exact command remains borrowed and preserves all raw UAPI selection tags,
 * controls/specifications and attachment lists. No main VS/FS/COLOR inference.
 */
struct gpu_m4_render_native_state {
	struct gpu_m4_render_encoded encoded;
	struct gpu_m4_exec_geometry geometry;
	const struct gpu_m4_drm_job_render *command;
	u64 partial_program[2], partial_resources[2];
	struct gpu_m4_render_pb_layout pb;
	u64 timestamp_va[GPU_M4_DRM_RENDER_TIMESTAMPS];
	u32 end_copy_count;
};

/* Heap-owned scratch. No detached state grants publication or fence success. */
struct gpu_m4_render_native_scratch {
	struct gpu_m4_render_native_state state;
	struct gpu_m4_render_packet_scratch packet;
};

static inline int gpu_m4_render_native_status(int ret)
{
	return ret > 0 ? -EPROTO : ret;
}

/* Span cache kinds owned by native render admission (object kinds 1/2 are
 * the provider's object-span helper).
 */
#define GPU_M4_RENDER_NATIVE_SPAN_NAMED 3U
#define GPU_M4_RENDER_NATIVE_SPAN_RANGE 4U

/* Attachment and register-metadata ranges: one installed run, or the
 * cross-job memo of this exact span/access of the same immutable manifest
 * (exactly the RW backing span memo below, at the requested access).
 */
static inline int gpu_m4_render_native_range(void *cookie, u64 va, u64 size, bool write)
{
	const struct gpu_m4_render_native_input *in = cookie;
	struct gpu_m4_drm_uat_span_key key = {
		.kind = GPU_M4_RENDER_NATIVE_SPAN_RANGE, .va = va, .size = size,
		.access = write ? DRM_ASAHI_BIND_WRITE : DRM_ASAHI_BIND_READ,
	};
	int ret;

	if (gpu_m4_drm_provider_version_span_cached_locked(in->version, &key))
		return 0;
	ret = gpu_m4_render_native_status(gpu_m4_drm_provider_version_range_locked(
		in->version, va, size, key.access));
	if (!ret)
		gpu_m4_drm_provider_version_span_remember_locked(in->version, &key);
	return ret;
}

/* The exact admitted tuple: same held core job, installed version, captured
 * snapshot, borrowed command record/index and byte-identical owner spans.
 * Caller has just re-run check_owner for live authority.
 */
static inline bool gpu_m4_render_native_admitted(const struct gpu_m4_render_native_input *in)
{
	const struct gpu_m4_render_native_admission *a = in->admission;

	return a && a->valid && a->job == in->job && a->version == in->version &&
		a->snapshot == in->snapshot && a->command == in->command &&
		a->command_index == in->command_index && in->owner &&
		!memcmp(&a->owner, in->owner, sizeof(a->owner));
}

/* Range callback for the SAME admitted command's immutable attachment spans,
 * which the recorded complete validation range-checked against this exact
 * held version. Selected only after gpu_m4_render_native_admitted().
 */
static inline int gpu_m4_render_native_range_admitted(void *cookie, u64 va, u64 size, bool write)
{
	const struct gpu_m4_render_native_input *in = cookie;

	(void)write;
	return in && in->admission && in->admission->valid && size && va <= ~0ULL - size ?
		0 : -EINVAL;
}

/* Writable output/scratch must not overlap ANY immutable command in the mixed
 * batch or its retained raw stream, not just this render command. The immutable
 * records may themselves overlap (the render record is inside its command).
 */
static inline int gpu_m4_render_native_job_storage(
		const struct gpu_m4_render_native_input *in, const void *output, u64 size,
		struct gpu_m4_drm_provider_job_view *view)
{
	struct gpu_m4_exec_interval pair[2];
	int ret;

	if (!in->job || !in->provider || !output || !size)
		return -EINVAL;
	ret = gpu_m4_drm_provider_job_check_owner_locked(in->job, in->provider);
	if (ret)
		return gpu_m4_render_native_status(ret);
	gpu_m4_drm_provider_job_view(in->job, view);
	if (!view->batch || !view->batch->commands || !view->batch->count ||
	    view->batch->count > GPU_M4_DRM_JOB_MAX_COMMANDS ||
	    !view->command_bytes || !view->command_size)
		return -ESTALE;
	pair[0] = (struct gpu_m4_exec_interval){ (unsigned long)output, size };
	pair[1] = (struct gpu_m4_exec_interval){ (unsigned long)view->batch, sizeof(*view->batch) };
	if (!gpu_m4_exec_disjoint(pair, 2))
		return -EINVAL;
	pair[1] = (struct gpu_m4_exec_interval){ (unsigned long)view->batch->commands,
		(u64)view->batch->count * sizeof(*view->batch->commands) };
	if (!gpu_m4_exec_disjoint(pair, 2))
		return -EINVAL;
	pair[1] = (struct gpu_m4_exec_interval){ (unsigned long)view->command_bytes, view->command_size };
	return gpu_m4_exec_disjoint(pair, 2) ? 0 : -EINVAL;
}

/* Check the decoder's actual contiguous named extent against its retained
 * snapshot, including exact GEM/object offsets and actual write permissions.
 * Shared BOs/RO shadows/SINGLE_PAGE mappings are not globally forbidden.
 */
struct gpu_m4_render_native_named_walk {
	const struct gpu_m4_drm_vm_owner *snapshot;
	const struct gpu_m4_drm_render_span *span;
	u64 va;
	u32 access;
};

/* One installed leaf of a named span against the captured snapshot leaf and
 * the decoder's object/offset/permission: the per-page comparison of the
 * original loop, fed by one sequential run instead of a layered search.
 */
static inline int gpu_m4_render_native_named_page(void *cookie, u64 page,
		const struct gpu_m4_drm_vm_leaf *installed)
{
	const struct gpu_m4_render_native_named_walk *w = cookie;
	struct gpu_m4_drm_vm_leaf leaf;
	u64 cursor = page < w->va ? w->va : page, within = cursor - page;
	u64 offset = w->span->object_offset + (cursor - w->va);
	int ret = gpu_m4_drm_vm_owner_page(w->snapshot, page, w->access, &leaf);

	if (ret)
		return gpu_m4_render_native_status(ret);
	if (leaf.identity != w->span->object || leaf.object_offset > ~0ULL - within ||
	    leaf.object_offset + within != offset ||
	    !!(leaf.access & DRM_ASAHI_BIND_WRITE) != w->span->writable)
		return -ESTALE;
	if (installed->pa != leaf.pa || installed->identity != leaf.identity ||
	    installed->object_offset != leaf.object_offset ||
	    installed->access != leaf.access || installed->cpu_policy != leaf.cpu_policy)
		return -ESTALE;
	return 0;
}

/* S3c: an authority VM's job has no captured user leaves; its one live
 * tree is the authority. Each installed leaf of the named span must name the
 * decoder's object at the contiguous offset with the decoder's writability.
 */
static inline int gpu_m4_render_native_live_page(void *cookie, u64 page,
		const struct gpu_m4_drm_vm_leaf *installed)
{
	const struct gpu_m4_render_native_named_walk *w = cookie;
	u64 cursor = page < w->va ? w->va : page, within = cursor - page;
	u64 offset = w->span->object_offset + (cursor - w->va);

	if (installed->identity != w->span->object ||
	    installed->object_offset > ~0ULL - within ||
	    installed->object_offset + within != offset ||
	    !!(installed->access & DRM_ASAHI_BIND_WRITE) != w->span->writable)
		return -ESTALE;
	return 0;
}

static inline int gpu_m4_render_native_named(const struct gpu_m4_render_native_input *in,
		const struct gpu_m4_drm_render_span *span, u64 va, u64 minimum,
		u32 alignment, bool write, bool admitted)
{
	u64 end;
	u32 access = DRM_ASAHI_BIND_READ | (span->writable ? DRM_ASAHI_BIND_WRITE : 0);
	struct gpu_m4_render_native_named_walk walk = {
		.snapshot = in->snapshot, .span = span, .va = va, .access = access,
	};
	struct gpu_m4_drm_uat_span_key key;
	int ret;

	if (!va)
		return span->va || span->size || span->object || span->object_offset ||
			span->writable ? -EINVAL : 0;
	if (!alignment || (va & (alignment - 1)) || span->va != va ||
	    span->size < minimum || !span->object || (write && !span->writable) ||
	    span->size > ~0ULL - va || span->size > ~0ULL - span->object_offset)
		return -EINVAL;
	/* S3c: one walk of the live tree (named-range validation by PTE walk). */
	if (gpu_m4_drm_provider_version_live(in->version))
		return admitted ? 0 : gpu_m4_render_native_status(
			gpu_m4_drm_provider_version_walk_locked(in->version, va, span->size, access,
				gpu_m4_render_native_live_page, &walk));
	/* Finite captured inventory is also the maximum possible contiguous extent. */
	end = va + span->size;
	if (((end - 1) / GPU_M4_DRM_PAGE_SIZE - va / GPU_M4_DRM_PAGE_SIZE + 1) >
	    gpu_m4_drm_vm_owner_leaf_count(in->snapshot))
		return -E2BIG;
	if (admitted)
		return 0; /* Same exact span already walked against this held tuple. */
	/* Cross-job: the same exact span of the same captured user map (VM_BIND
	 * generation of the manifest's own VM; validation proved snapshot and
	 * version generations equal) already proved against this immutable
	 * installed manifest.
	 */
	key = (struct gpu_m4_drm_uat_span_key) {
		.kind = GPU_M4_RENDER_NATIVE_SPAN_NAMED, .identity = span->object,
		.aux_value = gpu_m4_drm_vm_owner_generation(in->snapshot),
		.va = va, .size = span->size, .offset = span->object_offset, .access = access,
	};
	if (gpu_m4_drm_provider_version_span_cached_locked(in->version, &key))
		return 0;
	ret = gpu_m4_drm_vm_owner_range(in->snapshot, va, span->size, access);
	if (ret)
		return gpu_m4_render_native_status(ret);
	/* One installed run: every page's snapshot/decoder/installed agreement. */
	ret = gpu_m4_drm_provider_version_walk_locked(in->version, va, span->size, access,
		gpu_m4_render_native_named_page, &walk);
	if (ret)
		return gpu_m4_render_native_status(ret);
	gpu_m4_drm_provider_version_span_remember_locked(in->version, &key);
	return 0;
}

/* A firmware cache backing's snapshot rows: one cache row of its exact pin
 * covering its subrange with its access, one reservation row of its GEM.
 */
static inline bool gpu_m4_render_native_cache_row(const struct gpu_m4_drm_cache_access *a,
		struct gpu_m4_drm_bo_pin *pin, const struct gpu_m4_render_native_backing *b)
{
	return a->pin == pin && !(a->access & ~(DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE)) &&
		(a->access & b->cache.access) == b->cache.access &&
		a->size && a->offset <= b->cache.offset &&
		a->offset < gpu_m4_drm_bo_pin_size(pin) &&
		a->size <= gpu_m4_drm_bo_pin_size(pin) - a->offset &&
		b->cache.offset - a->offset < a->size &&
		b->cache.size <= a->size - (b->cache.offset - a->offset);
}

static inline bool gpu_m4_render_native_resv_row(const struct gpu_m4_drm_resv_access *r,
		const struct drm_gem_object *gem, const struct gpu_m4_render_native_backing *b)
{
	return r->object == gem && !(r->access & ~(DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE)) &&
		(r->access & b->cache.access) == b->cache.access;
}

/* Concrete native staging claims use actual registered provider objects and
 * their immutable pin inventory. READY claims are checked by the mandatory
 * actual allocator owner, never converted into fake GEMs or raw PA callbacks.
 */
static inline int gpu_m4_render_native_backing(const struct gpu_m4_render_native_input *in,
		const struct gpu_m4_render_native_backing *b, u64 minimum,
		u32 alignment, bool firmware, bool admitted)
{
	const struct gpu_m4_drm_provider_private_gem *g;
	struct gpu_m4_drm_bo_pin *pin;
	u64 offset, pa;
	int ret;

	if (!minimum || !alignment || (alignment & (alignment - 1)) ||
	    b->span.size < minimum || (b->span.va & (alignment - 1)) ||
	    !b->span.va || b->span.size > ~0ULL - b->span.va)
		return -EINVAL;
	if (firmware) {
		if (!gpu_m4_exec_fw_range(b->span.va, b->span.size, alignment))
			return -EINVAL;
	} else if (!admitted) {
		/* One installed run, or a cross-job memo of this exact RW span of the
		 * same immutable manifest.
		 */
		struct gpu_m4_drm_uat_span_key key = {
			.kind = GPU_M4_RENDER_NATIVE_SPAN_RANGE, .va = b->span.va,
			.size = b->span.size, .access = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE,
		};

		if (!gpu_m4_drm_provider_version_span_cached_locked(in->version, &key)) {
			ret = gpu_m4_drm_provider_version_range_locked(in->version, b->span.va,
				b->span.size, key.access);
			if (ret)
				return gpu_m4_render_native_status(ret);
			gpu_m4_drm_provider_version_span_remember_locked(in->version, &key);
		}
	}
	if (b->kind == GPU_M4_RENDER_NATIVE_READY)
		return b->ready ? 0 : -EINVAL; /* Actual retained proof already called. */
	if (b->kind == GPU_M4_RENDER_NATIVE_CACHE) {
		const struct gpu_m4_drm_cache_access *cache;
		const struct gpu_m4_drm_resv_access *resv;
		struct drm_gem_object *gem;
		u32 nc, nr, row;
		bool cached = false, reserved = false;

		if (!firmware || !b->cache.object || !b->cache.upper_lease ||
		    !b->cache.size || (b->cache.offset | b->cache.size) & (GPU_M4_DRM_PAGE_SIZE - 1) ||
		    (b->cache.access != DRM_ASAHI_BIND_READ &&
		     b->cache.access != (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE)) ||
		    !gpu_m4_exec_fw_range(b->cache.firmware_va, b->cache.size, GPU_M4_DRM_PAGE_SIZE) ||
		    b->span.va < b->cache.firmware_va ||
		    b->span.va - b->cache.firmware_va >= b->cache.size ||
		    b->span.size > b->cache.size - (b->span.va - b->cache.firmware_va))
			return -EINVAL;
		ret = gpu_m4_drm_provider_object_origin_locked(in->provider, in->version, b->cache.object);
		if (ret)
			return gpu_m4_render_native_status(ret);
		pin = gpu_m4_drm_provider_object_native(b->cache.object);
		gem = gpu_m4_drm_provider_object_identity(b->cache.object);
		if (!pin || !gem || gpu_m4_drm_bo_pin_object(pin) != gem ||
		    gpu_m4_drm_bo_policy(gem) != GPU_M4_DRM_CPU_NC ||
		    b->cache.offset >= gpu_m4_drm_bo_pin_size(pin) ||
		    b->cache.size > gpu_m4_drm_bo_pin_size(pin) - b->cache.offset)
			return -EINVAL;
		cache = gpu_m4_drm_vm_owner_cache_access(in->snapshot, &nc);
		resv = gpu_m4_drm_vm_owner_resv_access(in->snapshot, &nr);
		if (!cache || !resv || nc > GPU_M4_DRM_MAX_BINDS || nr > GPU_M4_DRM_MAX_BINDS)
			return -EINVAL;
		/* The snapshot's own row of this GEM first (its identity order); a
		 * refusal there takes the complete scans, so the answer is theirs.
		 */
		row = gpu_m4_drm_vm_owner_find(in->snapshot, gem);
		if (row < nc)
			cached = gpu_m4_render_native_cache_row(&cache[row], pin, b);
		if (row < nr)
			reserved = gpu_m4_render_native_resv_row(&resv[row], gem, b);
		for (u32 i = 0; !cached && i < nc; i++)
			cached = gpu_m4_render_native_cache_row(&cache[i], pin, b);
		for (u32 i = 0; !reserved && i < nr; i++)
			reserved = gpu_m4_render_native_resv_row(&resv[i], gem, b);
		if (!cached || !reserved)
			return -EACCES;
		if (admitted)
			return 0; /* Same pinned inventory already walked for this tuple. */
		for (offset = 0; offset < b->cache.size; offset += GPU_M4_DRM_PAGE_SIZE) {
			ret = gpu_m4_drm_provider_object_page_locked(b->cache.object, b->cache.offset + offset, &pa);
			if (ret)
				return gpu_m4_render_native_status(ret);
		}
		return 0; /* Actual upper lease/PA/class proof belongs to check_owner. */
	}
	if (b->kind != GPU_M4_RENDER_NATIVE_GEM)
		return -EINVAL;
	g = &b->gem;
	if (!g->object || !g->lease || !g->size ||
	    (g->va | g->offset | g->size) & (GPU_M4_DRM_PAGE_SIZE - 1) ||
	    g->size < b->span.size || g->size > ~0ULL - g->va ||
	    g->size > ~0ULL - g->offset ||
	    g->access != (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE) ||
	    (!firmware && (g->va != b->span.va || g->size != b->span.size)))
		return -EINVAL;
	ret = gpu_m4_drm_provider_private_check_locked(g->lease, in->owner->vm, g->va, g->size);
	if (ret)
		return gpu_m4_render_native_status(ret);
	ret = gpu_m4_drm_provider_object_origin_locked(in->provider, in->version, g->object);
	if (ret)
		return gpu_m4_render_native_status(ret);
	pin = gpu_m4_drm_provider_object_native(g->object);
	if (!pin || g->offset > gpu_m4_drm_bo_pin_size(pin) ||
	    g->size > gpu_m4_drm_bo_pin_size(pin) - g->offset)
		return -EINVAL;
	if (admitted)
		return 0; /* Lease/origin rechecked; pin-to-version pages already walked. */
	/* Every installed page names this held pin's page at its offset: one
	 * sequential run (bounded by the provider's own-page limit), or the
	 * cross-job memo of this exact object span of the same immutable
	 * manifest. The object's live page/session gate runs every call.
	 */
	return gpu_m4_render_native_status(gpu_m4_drm_provider_version_object_locked(
		in->version, g->va, g->object, gpu_m4_drm_bo_pin_object(pin), g->offset,
		g->size, g->access, false));
}

static inline int gpu_m4_render_native_private(const struct gpu_m4_render_native_input *in,
		struct gpu_m4_render_native_state *state, bool admitted)
{
	static const u32 fw_min[GPU_M4_RENDER_FIRST_FW_COUNT] = {
		0x4000, 0x4000, 0x4000, 512, 512, 8, 0x40, 0x1000,
		0xc18, 0x1248, 0x940, 0xca0, 0x300, 0x380, 0x24c0, 0x24c0,
	};
	static const u32 gpu_min[GPU_M4_RENDER_FIRST_GPU_COUNT] = {
		GPU_M4_EXEC_POOL_SIZE, 0x20000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000,
	};
	const struct gpu_m4_render_native_owner *o = in->owner;
	struct gpu_m4_exec_interval gpu[GPU_M4_RENDER_ENCODED_RESOURCES +
		GPU_M4_RENDER_FIRST_GPU_COUNT + 1];
	struct gpu_m4_exec_interval fw[GPU_M4_RENDER_FIRST_FW_COUNT + 7];
	struct gpu_m4_exec_interval embedded[4];
	unsigned int i, n = 0, f = 0;
	unsigned int subrecords = 0;
	u64 compact;
	int ret;

	if (o->resource[GPU_M4_RENDER_VDM].span.va ||
	    o->resource[GPU_M4_RENDER_VDM].span.size || o->resource[GPU_M4_RENDER_VDM].kind)
		return -EINVAL;
	{
		const struct gpu_m4_render_native_backing *layer = &o->resource[GPU_M4_RENDER_LAYER_METADATA];
		const struct gpu_m4_render_native_backing *scene = &o->resource[GPU_M4_RENDER_SCENE_METADATA];
		u64 offset = state->encoded.geometry.layers > 1 ? 0x200 : 0;

		if (scene->span.size < offset + 0x100 || scene->span.va > ~0ULL - offset ||
		    layer->span.va != scene->span.va + offset || layer->span.size < 0x100 ||
		    layer->span.size > scene->span.size - offset ||
		    !gpu_m4_render_native_same_backing(layer, scene))
			return -EINVAL;
	}
	for (i = 0; i < GPU_M4_RENDER_ENCODED_RESOURCES; i++) {
		u64 minimum = 0x4000;

		if (i == GPU_M4_RENDER_VDM)
			continue;
		if (i == GPU_M4_RENDER_LAYER_METADATA) {
			/* The complete scene's real pin/claim is proved below. LM is a
			 * bounded subrecord of that SAME owner, not another allocation.
			 */
			state->encoded.resource[i] = o->resource[i].span;
			ret = gpu_m4_render_compact(&o->resource[i].span, &compact);
			if (ret)
				return ret;
			continue;
		}
		if (i == GPU_M4_RENDER_TILEMAP)
			minimum = state->geometry.tilemap_size;
		else if (i == GPU_M4_RENDER_TPC)
			minimum = state->geometry.tpc_size;
		else if (i == GPU_M4_RENDER_PREEMPT_0 || i == GPU_M4_RENDER_PREEMPT_1 ||
			 i == GPU_M4_RENDER_FRAGMENT_AUXILIARY)
			minimum = 0x10000;
		ret = gpu_m4_render_native_backing(in, &o->resource[i], minimum, 0x4000, false,
			admitted);
		if (ret)
			return ret;
		state->encoded.resource[i] = o->resource[i].span;
		if (i != GPU_M4_RENDER_LAYER_METADATA)
			gpu[n++] = (struct gpu_m4_exec_interval){ o->resource[i].span.va,
				o->resource[i].span.size };
		if (i != GPU_M4_RENDER_SCRATCH && i != GPU_M4_RENDER_FRAGMENT_AUXILIARY) {
			ret = gpu_m4_render_compact(&o->resource[i].span, &compact);
			if (ret)
				return ret;
		}
	}
	if (o->parameter_heap.span.size < state->pb.heap_size)
		return -EINVAL;
	ret = gpu_m4_render_pb_capacity(&state->pb, o->parameter_heap.span.size);
	if (ret)
		return ret;
	ret = gpu_m4_render_native_backing(in, &o->parameter_heap,
		state->pb.heap_size, 0x20000, false, admitted);
	if (ret)
		return ret;
	ret = gpu_m4_render_compact(&o->parameter_heap.span, &compact);
	if (ret)
		return ret;
	gpu[n++] = (struct gpu_m4_exec_interval){ o->parameter_heap.span.va,
		o->parameter_heap.span.size };
	for (i = 0; i < GPU_M4_RENDER_FIRST_GPU_COUNT; i++) {
		u64 minimum = i == GPU_M4_RENDER_FIRST_PB_LIST_GPU ?
			state->pb.page_list_size : gpu_min[i];

		if ((i != GPU_M4_RENDER_FIRST_PB_LIST_GPU && o->gpu[i].span.size != minimum) ||
		    o->gpu[i].span.size < minimum)
			return -EINVAL;
		ret = gpu_m4_render_native_backing(in, &o->gpu[i], minimum, 0x4000, false, admitted);
		if (ret)
			return ret;
		gpu[n++] = (struct gpu_m4_exec_interval){ o->gpu[i].span.va, o->gpu[i].span.size };
	}
	/* Only private allocations are pairwise-disjoint. User Mesa spans may alias. */
	if (!gpu_m4_exec_disjoint(gpu, n))
		return -EINVAL;
	for (i = 0; i < GPU_M4_RENDER_FIRST_FW_COUNT; i++) {
		u64 minimum = fw_min[i];
		bool sequence = i == GPU_M4_RENDER_FIRST_TILER_SEQUENCE_FW ||
			i == GPU_M4_RENDER_FIRST_FRAGMENT_SEQUENCE_FW;

		if (o->firmware[i].kind == GPU_M4_RENDER_NATIVE_CACHE && !sequence &&
		    o->firmware[i].cache.access != (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE))
			return -EACCES;
		if (i == GPU_M4_RENDER_FIRST_PB_LIST_FW)
			minimum = state->pb.page_list_size;
		else if (i == GPU_M4_RENDER_FIRST_PB_BLOCKS_FW)
			minimum = state->pb.block_list_size;
		else if (i == GPU_M4_RENDER_FIRST_TILER_SEQUENCE_FW)
			minimum = 0x400;
		else if (i == GPU_M4_RENDER_FIRST_FRAGMENT_SEQUENCE_FW)
			minimum = 0x800;
		ret = gpu_m4_render_native_backing(in, &o->firmware[i], minimum, 8, true, admitted);
		if (ret)
			return ret;
		fw[f++] = (struct gpu_m4_exec_interval){ o->firmware[i].span.va,
			o->firmware[i].span.size };
	}
#define M4_NATIVE_FW(b, minimum) do { \
	if ((b).kind == GPU_M4_RENDER_NATIVE_CACHE && \
	    (b).cache.access != (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE)) return -EACCES; \
	ret = gpu_m4_render_native_backing(in, &(b), (minimum), 8, true, admitted); \
	if (ret) return ret; \
	fw[f++] = (struct gpu_m4_exec_interval){ (b).span.va, (b).span.size }; \
} while (0)
	M4_NATIVE_FW(o->manager, GPU_M4_RENDER_PARAMETER_MANAGER_SIZE);
	M4_NATIVE_FW(o->descriptor, GPU_M4_RENDER_PARAMETER_DESCRIPTOR_SIZE);
	M4_NATIVE_FW(o->tpc_firmware, o->resource[GPU_M4_RENDER_TPC].span.size);
#undef M4_NATIVE_FW
	for (i = 0; i < 2; i++) {
		const struct gpu_m4_render_native_backing *b = i ? &o->fragment_dependency : &o->init_buffer;
		const struct gpu_m4_render_native_backing *scene = &o->firmware[GPU_M4_RENDER_FIRST_SCENE_FW];
		u64 offset = i ? 0x340 : 0x300;
		u64 minimum = i ? GPU_M4_EXEC_DEPENDENCY_SIZE : GPU_M4_RENDER_INIT_BUFFER_SIZE;

		ret = gpu_m4_render_native_backing(in, b, minimum, 8, true, admitted);
		if (ret)
			return ret;
		if (b->kind == GPU_M4_RENDER_NATIVE_CACHE &&
		    b->cache.access != (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE))
			return -EACCES;
		if (b->span.va == scene->span.va + offset) {
			if (b->span.size != minimum || !gpu_m4_render_native_same_backing(b, scene))
				return -EINVAL;
			embedded[subrecords++] = (struct gpu_m4_exec_interval){ b->span.va, minimum };
		} else {
			fw[f++] = (struct gpu_m4_exec_interval){ b->span.va, b->span.size };
		}
	}
	for (i = 0; i < 2; i++)
		if (in->chronology->prior[i].firmware.va) {
			const struct gpu_m4_render_span *d = &in->chronology->prior[i].firmware;
			u64 scene = o->firmware[GPU_M4_RENDER_FIRST_SCENE_FW].span.va;

			if (d->va == scene + (i ? 0x440 : 0x380)) {
				if (d->size != GPU_M4_EXEC_DEPENDENCY_SIZE)
					return -EINVAL;
				embedded[subrecords++] = (struct gpu_m4_exec_interval){ d->va, d->size };
			} else {
				fw[f++] = (struct gpu_m4_exec_interval){ d->va, d->size };
			}
		}
	if (subrecords && !gpu_m4_exec_disjoint(embedded, subrecords))
		return -EINVAL;
	return gpu_m4_exec_disjoint(fw, f) ? 0 : -EINVAL;
}

/* The job's held views authenticate the exact command/slot/GEM/slice/alias.
 * The mandatory runtime owner additionally checks its canonical TS registry
 * and token. This detached phase is NOT a PUBLISHED/cache-protected proof.
 */
static inline int gpu_m4_render_native_timestamp_requests(
		const struct gpu_m4_render_native_input *in,
		struct gpu_m4_render_native_state *state, bool admitted)
{
	const struct gpu_m4_drm_job_render *c = in->command;
	const struct drm_asahi_cmd_render *r = &c->uapi;
	struct drm_asahi_timestamp requests[GPU_M4_DRM_RENDER_TIMESTAMPS];
	u32 count, present = 0, seen = 0;
	int ret;

	requests[0] = r->ts_vtx.start; requests[1] = r->ts_vtx.end;
	requests[2] = r->ts_frag.start; requests[3] = r->ts_frag.end;
	memcpy(requests + 4, r->ts_frag_end_copies, sizeof(r->ts_frag_end_copies));
	for (u32 i = 0; i < GPU_M4_DRM_RENDER_TIMESTAMPS; i++) {
		const struct gpu_m4_drm_job_timestamp *t = &c->timestamps[i];

		if (!requests[i].handle) {
			if (requests[i].offset || memchr_inv(t, 0, sizeof(*t)))
				return -EINVAL;
			continue;
		}
		if (!t->lease || !t->backend || !t->object ||
		    t->offset != requests[i].offset || (t->offset & 7) ||
		    (t->object_offset & 7) || t->object_offset > ~0ULL - 8 ||
		    t->firmware_va < GPU_M4_RENDER_TIMESTAMP_FW_START ||
		    t->firmware_va > GPU_M4_RENDER_TIMESTAMP_FW_END - 8 ||
		    (t->firmware_va & 7) || (i >= 4 && !r->ts_frag.end.handle))
			return -EINVAL;
		present |= 1U << i;
		state->timestamp_va[i] = t->firmware_va;
		state->end_copy_count += i >= 4;
	}
	if (!present || admitted)
		return 0; /* Held membership of this exact job/command already proved. */
	ret = gpu_m4_render_native_status(gpu_m4_drm_provider_job_timestamp_count_locked(in->job, &count));
	if (ret)
		return ret;
	if (count > GPU_M4_DRM_JOB_MAX_COMMANDS * GPU_M4_DRM_RENDER_TIMESTAMPS)
		return -E2BIG;
	for (u32 i = 0; i < count; i++) {
		struct gpu_m4_drm_provider_job_timestamp_view v;
		const struct gpu_m4_drm_job_timestamp *t;

		ret = gpu_m4_render_native_status(gpu_m4_drm_provider_job_timestamp_at_locked(in->job, i, &v));
		if (ret)
			return ret;
		if (v.command_index != in->command_index)
			continue;
		if (v.timestamp_index >= GPU_M4_DRM_RENDER_TIMESTAMPS ||
		    !(present & (1U << v.timestamp_index)) || (seen & (1U << v.timestamp_index)))
			return -ESTALE;
		t = &c->timestamps[v.timestamp_index];
		if (!v.runtime_alias || !v.object || v.gem != t->object ||
		    v.object_offset != t->object_offset || v.firmware_va != t->firmware_va)
			return -ESTALE;
		seen |= 1U << v.timestamp_index;
	}
	return seen == present ? 0 : -ESTALE;
}

/* Separate native admission. Nothing is allocated, published, cleaned,
 * invalidated or signaled. Caller owns every borrowed tuple under provider
 * serialization. Failure leaves state byte-exact; no fixture role is inferred.
 */
static inline int gpu_m4_render_native_validate_once(struct gpu_m4_render_native_state *output,
		const struct gpu_m4_render_native_input *in)
{
	struct gpu_m4_drm_provider_job_view view = { 0 };
	struct gpu_m4_render_native_state state = { 0 };
	struct gpu_m4_render_encoded *e = &state.encoded;
	struct gpu_m4_exec_interval inputs[6];
	const struct gpu_m4_drm_job_render *c;
	const struct drm_asahi_cmd_render *r;
	u64 selected[4], specs[4], va[4];
	u32 offset[4], high[4], i;
	bool admitted;
	int ret;

	if (!output || !in || !in->provider || !in->job || !in->command ||
	    !in->version || !in->snapshot || !in->owner || !in->binding ||
	    !in->chronology || !in->check_owner || !in->uuid)
		return -EINVAL;
	inputs[0] = (struct gpu_m4_exec_interval){ (unsigned long)output, sizeof(*output) };
	inputs[1] = (struct gpu_m4_exec_interval){ (unsigned long)in, sizeof(*in) };
	inputs[2] = (struct gpu_m4_exec_interval){ (unsigned long)in->command, sizeof(*in->command) };
	inputs[3] = (struct gpu_m4_exec_interval){ (unsigned long)in->owner, sizeof(*in->owner) };
	inputs[4] = (struct gpu_m4_exec_interval){ (unsigned long)in->binding, sizeof(*in->binding) };
	inputs[5] = (struct gpu_m4_exec_interval){ (unsigned long)in->chronology, sizeof(*in->chronology) };
	if (!gpu_m4_exec_disjoint(inputs, 6))
		return -EINVAL;
	ret = gpu_m4_render_native_job_storage(in, output, sizeof(*output), &view);
	if (ret)
		return ret;
	if (!view.batch || !view.batch->commands || in->command_index >= view.batch->count ||
	    view.batch->commands[in->command_index].type != DRM_ASAHI_CMD_RENDER ||
	    in->command != &view.batch->commands[in->command_index].render ||
	    in->version != view.version || in->snapshot != view.snapshot ||
	    view.batch->vm_snapshot != in->snapshot ||
	    !in->owner->stage_owner || view.stage_owner != in->owner->stage_owner ||
	    !in->owner->vm || !in->binding->lease || in->binding->version != in->version ||
	    !in->binding->epoch || !in->binding->generation || in->binding->generation > 255 ||
	    !gpu_m4_exec_user_asid(in->binding->asid) ||
	    in->owner->render.context != in->binding->asid ||
	    in->owner->render.asid != in->binding->asid ||
	    in->owner->render.generation != in->binding->generation ||
	    in->owner->render.parameter_id >= GPU_M4_RENDER_PARAMETER_SLOT_COUNT ||
	    in->owner->render.parameter_manager_fw != in->owner->manager.span.va ||
	    in->owner->render.parameter_descriptor_fw != in->owner->descriptor.span.va ||
	    gpu_m4_drm_provider_version_generation(in->version) !=
		gpu_m4_drm_vm_owner_generation(in->snapshot))
		return -ESTALE;
	/* Validate real typed owner identities BEFORE borrowing their native pins. */
	ret = gpu_m4_render_native_status(in->check_owner(in->owner_cookie, in));
	if (ret)
		return ret;
	/* Only after live authority: the exact tuple's immutable span admission. */
	admitted = gpu_m4_render_native_admitted(in);
	/* The held job's own exact version/snapshot coverage memo (proved once). */
	if (!admitted &&
	    !gpu_m4_drm_provider_job_native_covered_locked(in->job, in->version, in->snapshot))
		return -EACCES;
	c = in->command; r = &c->uapi;
	/* The submit-time UAPI admission, repeated as a defense. */
	if (memchr_inv(&r->vertex_helper, 0, sizeof(r->vertex_helper)) ||
	    memchr_inv(&r->fragment_helper, 0, sizeof(r->fragment_helper)))
		return -EOPNOTSUPP;
	ret = gpu_m4_drm_render_admitted(r);
	if (ret)
		return ret;
	for (i = GPU_M4_DRM_RENDER_DEPTH_META; i < GPU_M4_DRM_RENDER_SPANS; i += 2)
		if (memchr_inv(&c->spans[i], 0, sizeof(c->spans[i])))
			return -EOPNOTSUPP;
	e->owner = in->owner->render;
	e->geometry = (struct gpu_m4_render_geometry){ r->width_px, r->height_px,
		r->layers, r->samples, r->utile_width_px, r->utile_height_px, r->sample_size_B };
	ret = gpu_m4_render_geometry(&state.geometry, &e->geometry);
	if (ret)
		return ret;
	ret = gpu_m4_render_pb_layout(&state.pb, &e->geometry);
	if (ret)
		return ret;
	ret = gpu_m4_render_native_timestamp_requests(in, &state, admitted);
	if (ret)
		return ret;
	/* Exact uncompressed Mesa ZLS grammar (admitted above). The whole
	 * accessible VM remains retained; checking each layer start does not
	 * invent a mip/image footprint.
	 */
	for (i = 0; i < 2; i++) {
		const struct drm_asahi_zls_buffer *z = i ? &r->stencil : &r->depth;
		const struct gpu_m4_drm_render_span *span = &c->spans[i ?
			GPU_M4_DRM_RENDER_STENCIL : GPU_M4_DRM_RENDER_DEPTH];
		u64 minimum;

		ret = gpu_m4_drm_render_zls_plane(r, i, &minimum);
		if (!ret)
			ret = gpu_m4_render_native_named(in, span, z->base, minimum, 4, true, admitted);
		if (ret)
			return ret;
	}
	e->depth = (struct gpu_m4_render_span){ r->depth.base, c->spans[GPU_M4_DRM_RENDER_DEPTH].size };
	e->stencil = (struct gpu_m4_render_span){ r->stencil.base, c->spans[GPU_M4_DRM_RENDER_STENCIL].size };
	e->depth_stride = r->depth.stride; e->stencil_stride = r->stencil.stride;
	e->zls_control = r->zls_ctrl; e->zls_pixels = r->isp_zls_pixels;
	if (!c->usc_base || (c->usc_base & 0xffffffffULL) ||
	    c->usc_base != e->owner.queue_usc_base || c->usc_base > GPU_M4_EXEC_VM_END - (1ULL << 32))
		return -EINVAL;
	offset[0] = r->bg.usc; offset[1] = r->eot.usc;
	offset[2] = r->partial_bg.usc; offset[3] = r->partial_eot.usc;
	high[0] = r->bg_rsrc_spec_hi; high[1] = r->bg_eot_rsrc_spec_hi;
	high[2] = r->bg_partial_rsrc_spec_hi; high[3] = r->bg_eot_partial_rsrc_spec_hi;
	specs[0] = r->bg.rsrc_spec; specs[1] = r->eot.rsrc_spec;
	specs[2] = r->partial_bg.rsrc_spec; specs[3] = r->partial_eot.rsrc_spec;
	for (i = 0; i < 4; i++) {
		if (!(r->flags & DRM_ASAHI_RENDER_RSRC_SPEC_HI) && high[i])
			return -EINVAL;
		if (r->flags & DRM_ASAHI_RENDER_RSRC_SPEC_HI)
			specs[i] |= (u64)high[i] << 32;
		if (specs[i] != c->resources[i] ||
		    c->usc_base > ~0ULL - (offset[i] & ~63U))
			return -ESTALE;
		selected[i] = c->usc_base + (offset[i] & ~63U);
		ret = gpu_m4_render_native_named(in, &c->spans[GPU_M4_DRM_RENDER_BG + i],
			selected[i], 64, 64, false, admitted);
		if (ret)
			return ret;
		if (c->spans[GPU_M4_DRM_RENDER_BG + i].size >
		    (1ULL << 32) - (selected[i] - c->usc_base))
			return -EINVAL;
	}
	va[0] = r->vdm_ctrl_stream_base; va[1] = r->isp_scissor_base;
	va[2] = r->isp_dbias_base; va[3] = r->isp_oclqry_base;
	for (i = 0; i < 4; i++) {
		ret = gpu_m4_render_native_named(in, &c->spans[i], va[i], i == 3 ? 16 : 4,
			i == 3 ? 16 : 4, i == 3, admitted);
		if (ret)
			return ret;
	}
	e->resource[GPU_M4_RENDER_VDM] = (struct gpu_m4_render_span){ va[0], c->spans[0].size };
	ret = gpu_m4_render_compact(&e->resource[GPU_M4_RENDER_VDM], &selected[0]);
	if (ret)
		return ret;
	ret = gpu_m4_render_chronology_fields_validate(in->chronology,
		in->owner->uma_slot, e->owner.parameter_id);
	if (ret)
		return ret;
	ret = gpu_m4_render_native_private(in, &state, admitted);
	if (ret)
		return ret;
	e->scissor = (struct gpu_m4_render_span){ va[1], c->spans[1].size };
	e->depth_bias = (struct gpu_m4_render_span){ va[2], c->spans[2].size };
	e->query = (struct gpu_m4_render_span){ va[3], c->spans[3].size };
	e->usc_base = c->usc_base;
	e->background_resources = specs[0]; e->end_of_tile_resources = specs[1];
	e->background_program = c->spans[GPU_M4_DRM_RENDER_BG].va;
	e->end_of_tile_program = c->spans[GPU_M4_DRM_RENDER_EOT].va;
	e->multisample_control = r->ppp_multisamplectl;
	e->merge_limit[0] = r->isp_merge_upper_x; e->merge_limit[1] = r->isp_merge_upper_y;
	e->depth_clear = r->isp_bgobjdepth; e->background_value = r->isp_bgobjvals;
	e->ppp_control = r->ppp_ctrl; e->uuid = in->uuid;
	e->isp_mode = gpu_m4_render_isp_mode_flags(r->flags);
	e->depth_bias_is_integer = !!(r->flags & DRM_ASAHI_RENDER_DBIAS_IS_INT);
	for (i = 0; i < 2; i++) {
		u8 attachments[GPU_M4_EXEC_ATTACHMENT_SIZE];
		struct gpu_m4_exec_vm vm = { GPU_M4_EXEC_VM_START, GPU_M4_EXEC_VM_END,
			(void *)in, admitted ? gpu_m4_render_native_range_admitted :
			gpu_m4_render_native_range };

		ret = gpu_m4_exec_attachments(attachments, &vm, c->attachments[i], c->attachment_count[i]);
		if (ret)
			return ret;
		e->attachment_count[i] = c->attachment_count[i];
		state.partial_program[i] = c->spans[GPU_M4_DRM_RENDER_PARTIAL_BG + i].va;
		state.partial_resources[i] = specs[2 + i];
	}
	state.command = c;
	ret = gpu_m4_render_native_status(in->check_owner(in->owner_cookie, in));
	if (ret)
		return ret;
	if (!admitted && in->admission) {
		struct gpu_m4_render_native_admission *a = in->admission;

		a->job = in->job;
		a->version = in->version;
		a->snapshot = in->snapshot;
		a->command = in->command;
		a->command_index = in->command_index;
		memcpy(&a->owner, in->owner, sizeof(a->owner));
		a->valid = true;
	}
	*output = state;
	return 0;
}

/* Complete admission on the first call for an exact held tuple, then reuse
 * (see struct gpu_m4_render_native_admission). Any failure discards the memo.
 */
static inline int gpu_m4_render_native_validate(struct gpu_m4_render_native_state *output,
		const struct gpu_m4_render_native_input *in)
{
	int ret = gpu_m4_render_native_validate_once(output, in);

	if (ret && in && in->admission)
		in->admission->valid = false;
	return ret;
}

/* Nonfallible literal emission AFTER native admission. No timestamp payload
 * is read, synthesized or published here; values remain firmware outputs.
 */
static inline void gpu_m4_render_native_scene_timestamps(
		struct gpu_m4_render_packet_output *out,
		const struct gpu_m4_render_native_input *in,
		const struct gpu_m4_render_native_state *state)
{
	const struct gpu_m4_render_native_owner *o = in->owner;
	u64 scene = o->firmware[GPU_M4_RENDER_FIRST_SCENE_FW].span.va;
	u32 ordinal = 0;

	put_unaligned_le64(state->encoded.resource[GPU_M4_RENDER_SCRATCH].va, out->scene);
	put_unaligned_le64(o->firmware[GPU_M4_RENDER_FIRST_SCRATCH_FW].span.va, out->scene + 8);
	put_unaligned_le64(state->encoded.resource[GPU_M4_RENDER_SCENE_LIST].va -
		GPU_M4_RENDER_RANGE1_START, out->scene + 0x28);
	put_unaligned_le64(scene + 0x200, out->scene + 0x40);
	memcpy(out->scene + 0x300, out->init_buffer, sizeof(out->init_buffer));
	memcpy(out->scene + 0x340, out->fragment_dependency, sizeof(out->fragment_dependency));
	memcpy(out->scene + 0x380, out->prior_dependency[0], sizeof(out->prior_dependency[0]));
	memcpy(out->scene + 0x440, out->prior_dependency[1], sizeof(out->prior_dependency[1]));
	memcpy(out->scene + 0x500, out->uma_submitted_counter, 8);
	for (u32 i = 0; i < 2; i++) {
		u8 *work = i ? out->fragment : out->tiler;
		u8 *sequence = i ? out->fragment_sequence : out->tiler_sequence;
		u32 pair_offset = i ? 0xc20 : 0x8d8;
		u32 timestamp_offset = i ? 0x1ec : 0x1cc;
		u64 work_fw = o->firmware[i ? GPU_M4_RENDER_FIRST_FRAGMENT_WORK_FW :
			GPU_M4_RENDER_FIRST_TILER_WORK_FW].span.va;
		u64 start = state->timestamp_va[i * 2], end = state->timestamp_va[i * 2 + 1];
		u64 pair = start || end ? work_fw + pair_offset : 0;

		put_unaligned_le64(start, work + pair_offset);
		put_unaligned_le64(end, work + pair_offset + 8);
		put_unaligned_le64(start, out->scene + (i ? 0x630 : 0x610));
		put_unaligned_le64(end, out->scene + (i ? 0x638 : 0x618));
		put_unaligned_le64(pair, sequence + timestamp_offset + 0x24);
		put_unaligned_le64(pair, sequence + timestamp_offset + 0x50 + 0x24);
	}
	if (!state->end_copy_count)
		return;
	/* The original inline header remains at 1b0. Only Finalize/tail move. */
	memmove(out->fragment_sequence + 0x288 + state->end_copy_count * 0x4c,
		out->fragment_sequence + 0x288, GPU_M4_RENDER_FRAGMENT_SEQUENCE_SIZE - 0x288);
	for (u32 i = 4; i < GPU_M4_DRM_RENDER_TIMESTAMPS; i++) {
		u8 *record;

		if (!state->timestamp_va[i])
			continue;
		put_unaligned_le64(state->timestamp_va[i], out->scene + 0x708 + (i - 4) * 16);
		record = out->fragment_sequence + 0x288 + ordinal++ * 0x4c;
		memcpy(record, out->fragment_sequence + 0x23c, 0x4c);
		put_unaligned_le64(scene + 0x700 + (i - 4) * 16, record + 0x24);
	}
	put_unaligned_le32(0U - (0x288 + state->end_copy_count * 0x4c),
		out->fragment_sequence + 0x328 + state->end_copy_count * 0x4c);
	put_unaligned_le32(GPU_M4_RENDER_FRAGMENT_SEQUENCE_SIZE + state->end_copy_count * 0x4c,
		out->fragment + 0x1c);
}

/* Detached preparation only. The actual job/version/staging/binding owners
 * stay retained. All fallible checks precede output; no allocator tables,
 * alias graph, threshold, ring, producer or fence is written here.
 */
static inline int gpu_m4_render_native_packet_prepare_once(
		struct gpu_m4_render_packet_output *output,
		struct gpu_m4_render_native_scratch *scratch,
		const struct gpu_m4_render_native_input *in)
{
	struct gpu_m4_exec_interval ranges[7];
	struct gpu_m4_render_span fw[GPU_M4_RENDER_FIRST_FW_COUNT], gpu[GPU_M4_RENDER_FIRST_GPU_COUNT];
	struct gpu_m4_render_packet_encoded packet;
	struct gpu_m4_drm_provider_job_view view = { 0 };
	struct gpu_m4_exec_vm vm = { GPU_M4_EXEC_VM_START, GPU_M4_EXEC_VM_END,
		(void *)in, gpu_m4_render_native_range };
	struct gpu_m4_exec_vm admitted = { GPU_M4_EXEC_VM_START, GPU_M4_EXEC_VM_END,
		(void *)in, gpu_m4_render_native_range_admitted };
	const struct gpu_m4_render_native_owner *owner;
	unsigned int i, count;
	int ret;

	if (!output || !scratch || !in || !in->owner || !in->command ||
	    !in->binding || !in->chronology)
		return -EINVAL;
	ranges[0] = (struct gpu_m4_exec_interval){ (unsigned long)output, sizeof(*output) };
	ranges[1] = (struct gpu_m4_exec_interval){ (unsigned long)scratch, sizeof(*scratch) };
	ranges[2] = (struct gpu_m4_exec_interval){ (unsigned long)in, sizeof(*in) };
	ranges[3] = (struct gpu_m4_exec_interval){ (unsigned long)in->owner, sizeof(*in->owner) };
	ranges[4] = (struct gpu_m4_exec_interval){ (unsigned long)in->command, sizeof(*in->command) };
	ranges[5] = (struct gpu_m4_exec_interval){ (unsigned long)in->binding, sizeof(*in->binding) };
	ranges[6] = (struct gpu_m4_exec_interval){ (unsigned long)in->chronology, sizeof(*in->chronology) };
	if (!gpu_m4_exec_disjoint(ranges, 7))
		return -EINVAL;
	ret = gpu_m4_render_native_job_storage(in, output, sizeof(*output), &view);
	if (ret)
		return ret;
	ret = gpu_m4_render_native_job_storage(in, scratch, sizeof(*scratch), &view);
	if (ret)
		return ret;
	ret = gpu_m4_render_native_validate(&scratch->state, in);
	if (ret)
		return ret;
	owner = in->owner;
	for (i = 0; i < GPU_M4_RENDER_FIRST_FW_COUNT; i++)
		fw[i] = owner->firmware[i].span;
	for (i = 0; i < GPU_M4_RENDER_FIRST_GPU_COUNT; i++)
		gpu[i] = owner->gpu[i].span;
	for (i = 0; i < 2; i++) {
		struct gpu_m4_render_register_output *r = &scratch->packet.registers[i];
		unsigned int work_gpu = i ? GPU_M4_RENDER_FIRST_FRAGMENT_WORK_GPU :
			GPU_M4_RENDER_FIRST_TILER_WORK_GPU;
		unsigned int work_fw = i ? GPU_M4_RENDER_FIRST_FRAGMENT_WORK_FW :
			GPU_M4_RENDER_FIRST_TILER_WORK_FW;
		unsigned int queue_fw = i ? GPU_M4_RENDER_FIRST_FRAGMENT_QUEUE_FW :
			GPU_M4_RENDER_FIRST_TILER_QUEUE_FW;

		memset(r, 0, sizeof(*r));
		r->geometry = scratch->state.geometry;
		/* Validation above admitted (or reused) these exact attachment spans. */
		ret = gpu_m4_exec_attachments(r->attachments, gpu_m4_render_native_admitted(in) ?
			&admitted : &vm, in->command->attachments[i],
			in->command->attachment_count[i]);
		if (ret)
			return ret;
		if (!i)
			gpu_m4_render_tiler_registers(r->registers, &scratch->state.encoded, &r->geometry, &count);
		else
			gpu_m4_render_fragment_registers(r->registers, &scratch->state.encoded,
				&r->geometry, scratch->state.encoded.background_program,
				scratch->state.encoded.end_of_tile_program, &count);
		r->count = count; r->bytes = count * 12; r->zero_suffix_records = i ? 2 : 3;
		ret = gpu_m4_render_register_metadata(scratch->packet.register_metadata[i], &vm,
			gpu[work_gpu].va + (i ? 0x80 : 0x40), r, i);
		if (ret)
			return ret;
		ret = gpu_m4_render_timestamps(&scratch->packet.timestamps[i], fw[work_fw].va,
			fw[queue_fw].va, &fw[GPU_M4_RENDER_FIRST_SCENE_FW], in->uuid, i);
		if (ret)
			return ret;
	}
	scratch->packet.partial_background = scratch->state.partial_program[0];
	scratch->packet.partial_end_of_tile = scratch->state.partial_program[1];
	gpu_m4_render_init_buffer_emit(scratch->packet.init_buffer, &owner->render,
		in->chronology->target_stamp);
	ret = gpu_m4_render_fragment_dependency(scratch->packet.fragment_dependency,
		fw[GPU_M4_RENDER_FIRST_NOTIFIERS_FW].va);
	if (ret)
		return ret;
	ret = gpu_m4_render_native_status(in->check_owner(in->owner_cookie, in));
	if (ret)
		return ret;
	packet = (struct gpu_m4_render_packet_encoded){ &scratch->state.encoded, fw, gpu,
		owner->tpc_firmware.span, in->chronology,
		scratch->state.partial_resources[0], scratch->state.partial_resources[1] };
	memset(output, 0, sizeof(*output));
	if (in->chronology->init_buffer)
		memcpy(output->init_buffer, scratch->packet.init_buffer, sizeof(output->init_buffer));
	gpu_m4_render_packet_tiler(output, &packet, &scratch->packet);
	gpu_m4_render_packet_fragment(output, &packet, &scratch->packet);
	gpu_m4_render_dependency_emit(output->fragment_dependency,
		fw[GPU_M4_RENDER_FIRST_NOTIFIERS_FW].va + 0x620,
		in->chronology->target_stamp, 0x70, in->chronology->target_stamp, false);
	for (i = 0; i < 2; i++) {
		const struct gpu_m4_render_prior_dependency *d = &in->chronology->prior[i];

		if (d->firmware.va)
			gpu_m4_render_dependency_emit(output->prior_dependency[i], d->predecessor_stamp_fw,
				d->predecessor_target, d->predecessor_event,
				in->chronology->target_stamp,
				(d->strict || d->fragment_internal) && !d->relaxed);
	}
	put_unaligned_le64(in->chronology->uma_submitted, output->uma_submitted_counter);
	gpu_m4_render_native_scene_timestamps(output, in, &scratch->state);
	return 0;
}

static inline int gpu_m4_render_native_packet_prepare(struct gpu_m4_render_packet_output *output,
		struct gpu_m4_render_native_scratch *scratch,
		const struct gpu_m4_render_native_input *in)
{
	int ret = gpu_m4_render_native_packet_prepare_once(output, scratch, in);

	if (ret && in && in->admission)
		in->admission->valid = false;
	return ret;
}

#endif
