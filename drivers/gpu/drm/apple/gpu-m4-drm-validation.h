/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_DRM_VALIDATION_H
#define APPLE_GPU_M4_DRM_VALIDATION_H

#include <linux/errno.h>
#include <linux/stddef.h>
#include <linux/string.h>
#include <linux/types.h>
#include "gpu-m4-asahi-drm.h"

#define GPU_M4_DRM_PAGE_SIZE 0x4000ULL
#define GPU_M4_DRM_MAX_BINDS 4096U
#define GPU_M4_DRM_MAX_SYNCS 4096U
#define GPU_M4_DRM_MAX_CMDBUF (4U * 1024U * 1024U)

static inline bool gpu_m4_drm_aligned(u64 value)
{
	return !(value & (GPU_M4_DRM_PAGE_SIZE - 1));
}

static inline bool gpu_m4_drm_range(u64 start, u64 size, u64 low, u64 high)
{
	return size && start >= low && start < high && size <= high - start;
}

static inline bool gpu_m4_drm_user_range(const struct drm_asahi_params_global *p,
				       u64 kernel_start, u64 kernel_end,
				       u64 start, u64 size)
{
	return gpu_m4_drm_range(start, size, p->vm_start, p->vm_end) &&
	       (start >= kernel_end || start + size <= kernel_start);
}

static inline int gpu_m4_drm_vm_validate(const struct drm_asahi_params_global *p,
				       const struct drm_asahi_vm_create *a)
{
	if (a->pad || a->vm_id || !gpu_m4_drm_aligned(a->kernel_start) ||
	    !gpu_m4_drm_aligned(a->kernel_end) ||
	    a->kernel_start < p->vm_start || a->kernel_end > p->vm_end ||
	    a->kernel_end <= a->kernel_start ||
	    a->kernel_end - a->kernel_start < p->vm_kernel_min_size)
		return -EINVAL;
	return 0;
}

static inline int gpu_m4_drm_bind_validate(const struct drm_asahi_params_global *p,
					 u64 kernel_start, u64 kernel_end,
					 const struct drm_asahi_gem_bind_op *a,
					 u64 object_size)
{
	u32 access = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE;
	u32 allowed = access | DRM_ASAHI_BIND_UNBIND | DRM_ASAHI_BIND_SINGLE_PAGE;

	if (a->flags & ~allowed || !gpu_m4_drm_aligned(a->addr) ||
	    !gpu_m4_drm_aligned(a->range) || !gpu_m4_drm_aligned(a->offset) ||
	    !gpu_m4_drm_user_range(p, kernel_start, kernel_end, a->addr, a->range))
		return -EINVAL;
	if (a->flags & DRM_ASAHI_BIND_UNBIND)
		return a->flags == DRM_ASAHI_BIND_UNBIND && !a->handle &&
		       !a->offset ? 0 : -EINVAL;
	if (!a->handle || !(a->flags & access) || a->offset >= object_size)
		return -EINVAL;
	if ((a->flags & DRM_ASAHI_BIND_SINGLE_PAGE ? GPU_M4_DRM_PAGE_SIZE :
	     a->range) > object_size - a->offset)
		return -EINVAL;
	return 0;
}

/*
 * Four host scheduler levels. HIGH and REALTIME order a client's work ahead of
 * every other client's, so they need CAP_SYS_NICE (checked by the caller after
 * this structural validation). No firmware priority or preemption follows.
 */
static inline bool gpu_m4_drm_priority_privileged(u32 priority)
{
	return priority >= DRM_ASAHI_PRIORITY_HIGH;
}

static inline int gpu_m4_drm_queue_validate(const struct drm_asahi_params_global *p,
					  u64 kernel_start, u64 kernel_end,
					  const struct drm_asahi_queue_create *a)
{
	if (a->flags || a->queue_id || a->priority > DRM_ASAHI_PRIORITY_REALTIME ||
	    (a->usc_exec_base & 0xffffffffULL) ||
	    !gpu_m4_drm_user_range(p, kernel_start, kernel_end,
				  a->usc_exec_base, 1ULL << 32))
		return -EINVAL;
	return 0;
}

static inline bool gpu_m4_drm_timestamp_range(u32 offset, u64 range)
{
	return !(offset & 7) && offset < range && 8 <= range - offset;
}

/* Unknown extension bytes may be ignored only when every byte is zero. */
static inline int gpu_m4_drm_payload(void *out, u32 known, const void *in, u32 size)
{
	const u8 *bytes = in;

	for (u32 i = known; i < size; i++)
		if (bytes[i])
			return -E2BIG;
	memset(out, 0, known);
	memcpy(out, in, size < known ? size : known);
	return 0;
}

/* Native G16G resource specs are 64-bit, independent of the legacy ISA path. */
static inline bool gpu_m4_drm_render_wide_specs(const struct drm_asahi_params_global *p)
{
	/* G15S legacy-HAL encodes full u64 resource words too (render R6). */
	if (p && p->gpu_generation == 15 && p->gpu_variant == 'S' && p->chip_id == 0x6030 &&
	    p->usc_generation == 3 && p->gpu_hal_generation == DRM_ASAHI_GPU_HAL_LEGACY)
		return true;
	return p && p->gpu_generation == 16 && p->gpu_variant == 'G' &&
		p->chip_id == 0x8132 && p->usc_generation == 3 &&
		p->gpu_hal_generation == DRM_ASAHI_GPU_HAL_200;
}

/* Only structural ABI checks here; the backend validates all mapped GPU spans. */
static inline int gpu_m4_drm_stream_validate(const struct drm_asahi_params_global *p,
					   const void *data, u32 size)
{
	const u8 *bytes = data;
	u32 offset = 0, render = 0, compute = 0;

	if (!size || size > GPU_M4_DRM_MAX_CMDBUF || (size & 7))
		return -EINVAL;
	while (offset < size) {
		struct drm_asahi_cmd_header h;
		u32 payload;
		int ret;

		if (size - offset < sizeof(h))
			return -EINVAL;
		memcpy(&h, bytes + offset, sizeof(h));
		offset += sizeof(h);
		payload = h.size;
		if (payload > size - offset || (payload & 7))
			return -EINVAL;
		if (h.cmd_type >= DRM_ASAHI_SET_VERTEX_ATTACHMENTS &&
		    h.cmd_type <= DRM_ASAHI_SET_COMPUTE_ATTACHMENTS) {
			if (h.vdm_barrier != DRM_ASAHI_BARRIER_NONE ||
			    h.cdm_barrier != DRM_ASAHI_BARRIER_NONE ||
			    payload % sizeof(struct drm_asahi_attachment) ||
			    payload / sizeof(struct drm_asahi_attachment) > p->max_attachments)
				return -EINVAL;
			for (u32 i = 0; i < payload; i += sizeof(struct drm_asahi_attachment)) {
				struct drm_asahi_attachment a;

				memcpy(&a, bytes + offset + i, sizeof(a));
				if (a.pad || a.flags ||
				    !gpu_m4_drm_range(a.pointer, a.size, p->vm_start, p->vm_end))
					return -EINVAL;
			}
		} else {
			if ((h.vdm_barrier != DRM_ASAHI_BARRIER_NONE && h.vdm_barrier > render) ||
			    (h.cdm_barrier != DRM_ASAHI_BARRIER_NONE && h.cdm_barrier > compute))
				return -EINVAL;
			if (h.cmd_type == DRM_ASAHI_CMD_RENDER) {
				struct drm_asahi_cmd_render r;
				u32 allowed = DRM_ASAHI_RENDER_VERTEX_SCRATCH |
					DRM_ASAHI_RENDER_PROCESS_EMPTY_TILES |
					DRM_ASAHI_RENDER_NO_VERTEX_CLUSTERING |
					DRM_ASAHI_RENDER_DBIAS_IS_INT;

				ret = gpu_m4_drm_payload(&r, sizeof(r), bytes + offset, payload);
				if (ret)
					return ret;
				if (p->features & DRM_ASAHI_FEATURE_FRAGMENT_DEPENDENCY)
					allowed |= DRM_ASAHI_RENDER_FRAGMENT_DEPENDENCY;
				if (p->features & DRM_ASAHI_FEATURE_FRAGMENT_DEPENDENCY_COMPUTE)
					allowed |= DRM_ASAHI_RENDER_FRAGMENT_DEPENDENCY_COMPUTE;
				if (p->features & DRM_ASAHI_FEATURE_CACHED_PROGRAM_PROBE)
					allowed |= DRM_ASAHI_RENDER_CACHED_PROGRAM_PROBE;
				if (gpu_m4_drm_render_wide_specs(p))
					allowed |= DRM_ASAHI_RENDER_RSRC_SPEC_HI;
				if (r.flags & ~allowed || !r.width_px || r.width_px > 16384 ||
				    !r.height_px || r.height_px > 16384 || !r.layers ||
				    r.layers > 2048 || (r.samples != 1 && r.samples != 2 && r.samples != 4))
					return -EINVAL;
				if (!(p->features & DRM_ASAHI_FEATURE_RENDER_TIMESTAMP_COPIES))
					for (u32 i = 0; i < 8; i++)
						if (r.ts_frag_end_copies[i].handle || r.ts_frag_end_copies[i].offset)
							return -EOPNOTSUPP;
				/*
				 * Mesa transmits all four high words with the presence
				 * flag. Never silently discard unflagged high words or
				 * accept the flag on a truncated old render payload.
				 */
				if ((r.flags & DRM_ASAHI_RENDER_RSRC_SPEC_HI) &&
				    payload < offsetof(struct drm_asahi_cmd_render,
						bg_partial_rsrc_spec_hi) + sizeof(r.bg_partial_rsrc_spec_hi))
					return -EINVAL;
				if (!(r.flags & DRM_ASAHI_RENDER_RSRC_SPEC_HI) &&
				    (r.bg_eot_rsrc_spec_hi || r.bg_eot_partial_rsrc_spec_hi ||
				     r.bg_rsrc_spec_hi || r.bg_partial_rsrc_spec_hi))
					return -EOPNOTSUPP;
				render++;
			} else if (h.cmd_type == DRM_ASAHI_CMD_COMPUTE) {
				struct drm_asahi_cmd_compute c;
				u32 allowed = p->features & DRM_ASAHI_FEATURE_COMPUTE_WIDE_VISIBILITY ?
					DRM_ASAHI_COMPUTE_WIDE_VISIBILITY : 0;

				ret = gpu_m4_drm_payload(&c, sizeof(c), bytes + offset, payload);
				if (ret)
					return ret;
				if (c.flags & ~allowed || c.cdm_ctrl_stream_end <= c.cdm_ctrl_stream_base ||
				    !gpu_m4_drm_range(c.cdm_ctrl_stream_base,
					c.cdm_ctrl_stream_end - c.cdm_ctrl_stream_base,
					p->vm_start, p->vm_end))
					return -EINVAL;
				if (!(p->features & DRM_ASAHI_FEATURE_COMPUTE_TIMESTAMP_COPIES))
					for (u32 i = 0; i < 8; i++)
						if (c.ts_end_copies[i].handle || c.ts_end_copies[i].offset)
							return -EOPNOTSUPP;
				compute++;
			} else {
				return -EINVAL;
			}
			if (render + compute > p->max_commands_per_submission)
				return -E2BIG;
		}
		offset += payload;
	}
	return render + compute ? 0 : -EINVAL;
}

#endif
