/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_DRM_RENDER_JOB_H
#define APPLE_GPU_M4_DRM_RENDER_JOB_H

#include "gpu-m4-drm-job.h"

/* Acceptance is not firmware encoding or hardware qualification. Keep the
 * unchanged render UAPI, including larger geometry and Z/S state which the
 * first-pixel encoder cannot yet execute. No userspace attachment identifies
 * a shader or COLOR allocation: the whole retained VM is the access owner.
 */
#define GPU_M4_DRM_RENDER_TIMESTAMPS 12U

enum gpu_m4_drm_render_span_role {
	GPU_M4_DRM_RENDER_VDM,
	GPU_M4_DRM_RENDER_SCISSOR,
	GPU_M4_DRM_RENDER_DBIAS,
	GPU_M4_DRM_RENDER_QUERY,
	GPU_M4_DRM_RENDER_BG,
	GPU_M4_DRM_RENDER_EOT,
	GPU_M4_DRM_RENDER_PARTIAL_BG,
	GPU_M4_DRM_RENDER_PARTIAL_EOT,
	GPU_M4_DRM_RENDER_DEPTH,
	GPU_M4_DRM_RENDER_DEPTH_META,
	GPU_M4_DRM_RENDER_STENCIL,
	GPU_M4_DRM_RENDER_STENCIL_META,
	GPU_M4_DRM_RENDER_SPANS,
};

struct gpu_m4_drm_render_span {
	u64 va;
	u64 size;
	struct drm_gem_object *object;
	u64 object_offset;
	bool writable;
};

struct gpu_m4_drm_job_render {
	struct drm_asahi_cmd_render uapi;
	u64 usc_base;
	u64 resources[4]; /* BG, EOT, partial BG, partial EOT: never truncate. */
	struct gpu_m4_drm_render_span spans[GPU_M4_DRM_RENDER_SPANS];
	u32 attachment_count[2];
	struct gpu_m4_exec_attachment attachments[2][GPU_M4_EXEC_ATTACHMENT_MAX];
	/* Vertex start/end, fragment start/end, then eight fragment-end copies. */
	struct gpu_m4_drm_job_timestamp timestamps[GPU_M4_DRM_RENDER_TIMESTAMPS];
};

struct gpu_m4_drm_job_command {
	u16 type;
	u16 vdm_barrier;
	u16 cdm_barrier;
	union {
		struct {
			struct gpu_m4_drm_job_compute compute;
			/* Caller-owned transaction workspace; zero after acceptance.
			 * Keep the established compute decoder's two images off the
			 * kernel stack while sharing the larger render union storage.
			 */
			struct gpu_m4_drm_job_compute compute_scratch;
		};
		struct gpu_m4_drm_job_render render;
	};
};

struct gpu_m4_drm_mixed_batch {
	void *vm_snapshot;
	struct gpu_m4_drm_job_command *commands;
	u32 count;
};

/* One depth (stencil=false) or stencil plane of the exact uncompressed Mesa
 * ZLS grammar: its presence against zls_ctrl and the minimum bytes its base
 * must name (1 + layer distance * (layers - 1)). The UAPI stride word is an
 * encoded register, NOT a byte distance.
 */
static inline int gpu_m4_drm_render_zls_plane(const struct drm_asahi_cmd_render *r,
					   bool stencil, u64 *minimum)
{
	const struct drm_asahi_zls_buffer *z = stencil ? &r->stencil : &r->depth;
	u64 plane_mask = stencil ? ((1ULL << 3) | (1ULL << 7) | (1ULL << 14) | (1ULL << 18)) :
		((1ULL << 1) | (1ULL << 5) | (1ULL << 15) | (1ULL << 19) | (3ULL << 25));
	u64 distance;

	*minimum = 1;
	if ((!z->base && (z->stride || (r->zls_ctrl & plane_mask))) ||
	    (z->base && r->layers > 1 && !z->stride) ||
	    (z->stride && (z->stride & 0x3fff) != 1) || (z->base & 3))
		return -EINVAL;
	if (z->stride) {
		distance = ((u64)(z->stride >> 14) + 1) * GPU_M4_DRM_PAGE_SIZE;
		if (distance > (~0ULL - 1) / (r->layers > 1 ? r->layers - 1 : 1))
			return -EINVAL;
		*minimum += distance * (r->layers - 1);
	}
	return 0;
}

/* Every refusal the bounded render encoder derives from the UAPI command
 * alone. Submit-time decode applies it, so a submission it accepts can no
 * longer be refused for these fields at publication (a provider fault);
 * publication repeats it as a defense.
 */
static inline int gpu_m4_drm_render_admitted(const struct drm_asahi_cmd_render *r)
{
	u64 minimum;
	int ret;

	if ((r->flags & ~(DRM_ASAHI_RENDER_PROCESS_EMPTY_TILES | DRM_ASAHI_RENDER_RSRC_SPEC_HI |
			 DRM_ASAHI_RENDER_DBIAS_IS_INT | DRM_ASAHI_RENDER_FRAGMENT_DEPENDENCY |
			 DRM_ASAHI_RENDER_FRAGMENT_DEPENDENCY_COMPUTE)) ||
	    r->depth.comp_base || r->depth.comp_stride ||
	    r->stencil.comp_base || r->stencil.comp_stride ||
	    r->sampler_heap || r->sampler_count ||
	    r->vertex_helper.binary || r->vertex_helper.cfg || r->vertex_helper.data ||
	    r->fragment_helper.binary || r->fragment_helper.cfg || r->fragment_helper.data)
		return -EOPNOTSUPP;
	if ((r->flags & DRM_ASAHI_RENDER_FRAGMENT_DEPENDENCY_COMPUTE) &&
	    !(r->flags & DRM_ASAHI_RENDER_FRAGMENT_DEPENDENCY))
		return -EINVAL;
	if (r->ppp_ctrl & ~0x203U)
		return -EINVAL;
	if ((r->zls_ctrl & ~0x00000000060cc0aaULL) ||
	    (r->isp_zls_pixels & 0xc0000000U) ||
	    (((r->zls_ctrl >> 25) & 3) != 0 && ((r->zls_ctrl >> 25) & 3) != 2) ||
	    !!(r->zls_ctrl & (1ULL << 1)) != !!(r->zls_ctrl & (1ULL << 5)) ||
	    !!(r->zls_ctrl & (1ULL << 3)) != !!(r->zls_ctrl & (1ULL << 7)))
		return -EOPNOTSUPP;
	ret = gpu_m4_drm_render_zls_plane(r, false, &minimum);
	if (!ret)
		ret = gpu_m4_drm_render_zls_plane(r, true, &minimum);
	if (ret)
		return ret;
	if ((!r->depth.base && !r->stencil.base && (r->zls_ctrl || r->isp_zls_pixels)) ||
	    !r->vdm_ctrl_stream_base || !r->isp_scissor_base ||
	    (!r->isp_dbias_base && (r->flags & DRM_ASAHI_RENDER_DBIAS_IS_INT)))
		return -EINVAL;
	return 0;
}

struct gpu_m4_drm_mixed_config {
	struct gpu_m4_drm_job_config job;
	/* Resolve the named VA from the immutable snapshot, never the current VM
	 * or an attachment hint. Return its actual retained contiguous mapped
	 * extent beginning at VA, with the exact pinned GEM/object offset. The
	 * entire extent must have the requested permissions. Report its actual
	 * snapshot write permission, not a guess from its render role. No new reference
	 * is transferred because the snapshot owns it through job retirement.
	 * Minimum checks only the named header/word, not all indirect accesses:
	 * linked streams, indexed arrays and bindless shader resources remain
	 * covered by the conservative whole-VM reservation/cache snapshot.
	 */
	int (*resolve_span)(void *cookie, void *snapshot,
			    enum gpu_m4_drm_render_span_role role,
			    u64 va, u64 minimum, bool write,
			    struct gpu_m4_drm_render_span *span);
};

static inline void gpu_m4_drm_mixed_put(const struct gpu_m4_drm_mixed_config *config,
				      struct gpu_m4_drm_mixed_batch *batch)
{
	unsigned int i, j;

	if (!batch || !batch->vm_snapshot)
		return;
	for (i = 0; i < batch->count; i++) {
		struct gpu_m4_drm_job_command *c = &batch->commands[i];
		struct gpu_m4_drm_job_timestamp *t = c->type == DRM_ASAHI_CMD_COMPUTE ?
			c->compute.timestamps : c->render.timestamps;
		u32 count = c->type == DRM_ASAHI_CMD_COMPUTE ?
			GPU_M4_DRM_JOB_TIMESTAMPS : GPU_M4_DRM_RENDER_TIMESTAMPS;

		for (j = 0; j < count; j++) {
			if (t[j].lease)
				config->job.ops->put_timestamp(config->job.cookie, &t[j]);
			memset(&t[j], 0, sizeof(t[j]));
		}
	}
	config->job.ops->put_vm(config->job.cookie, batch->vm_snapshot);
	memset(batch, 0, sizeof(*batch));
}

static inline int gpu_m4_drm_render_payload(const struct drm_asahi_params_global *p,
					  const void *bytes, u32 size,
					  struct drm_asahi_cmd_render *r)
{
	u32 i;
	int ret = gpu_m4_drm_payload(r, sizeof(*r), bytes, size);

	if (ret)
		return ret;
	if (r->sampler_heap || r->sampler_count || r->vertex_helper.binary ||
	    r->vertex_helper.cfg || r->vertex_helper.data || r->fragment_helper.binary ||
	    r->fragment_helper.cfg || r->fragment_helper.data)
		return -EOPNOTSUPP;
	if ((r->utile_width_px != 16 && r->utile_width_px != 32) ||
	    (r->utile_height_px != 16 && r->utile_height_px != 32) ||
	    (u32)r->sample_size_B * r->utile_width_px * r->utile_height_px * r->samples > 32768 ||
	    (r->ppp_ctrl & ~0x203U) || (r->vdm_ctrl_stream_base & 3) ||
	    (r->isp_scissor_base & 3) || (r->isp_dbias_base & 3) ||
	    (r->isp_oclqry_base & 15))
		return -EINVAL;
	if ((r->flags & DRM_ASAHI_RENDER_FRAGMENT_DEPENDENCY_COMPUTE) &&
	    !(r->flags & DRM_ASAHI_RENDER_FRAGMENT_DEPENDENCY))
		return -EINVAL;
	for (i = 0; i < 8; i++)
		if (r->ts_frag_end_copies[i].handle &&
		    (!(p->features & DRM_ASAHI_FEATURE_RENDER_TIMESTAMP_COPIES) ||
		     !r->ts_frag.end.handle))
			return -EOPNOTSUPP;
	return gpu_m4_drm_render_admitted(r);
}

static inline int gpu_m4_drm_mixed_timestamp(const struct gpu_m4_drm_job_config *config,
					    const struct gpu_m4_drm_submit *submit,
					    void *snapshot,
					    struct drm_asahi_timestamp timestamp,
					    struct gpu_m4_drm_job_timestamp *owned)
{
	struct gpu_m4_drm_timestamp borrowed = { 0 };
	int ret;

	if (!timestamp.handle)
		return 0;
	if (!submit->lookup_timestamp || (timestamp.offset & 7))
		return -EINVAL;
	ret = gpu_m4_drm_job_status(submit->lookup_timestamp(submit->lookup_cookie,
				  timestamp.handle, timestamp.offset, &borrowed));
	if (ret)
		return ret;
	if (!borrowed.backend || !borrowed.object || borrowed.offset != timestamp.offset ||
	    (borrowed.object_offset & 7))
		return -EIO;
	ret = gpu_m4_drm_job_status(config->ops->hold_timestamp(config->cookie,
					 snapshot, &borrowed, owned));
	if (ret)
		return ret;
	if (!owned->lease || owned->backend != borrowed.backend ||
	    owned->object != borrowed.object || owned->offset != borrowed.offset ||
	    owned->object_offset != borrowed.object_offset ||
	    !gpu_m4_drm_job_timestamp_range(owned->firmware_va))
		return -EIO;
	return 0;
}

static inline int gpu_m4_drm_mixed_attachment(const struct gpu_m4_drm_job_config *config,
					     void *snapshot,
					     const void *bytes, u32 count,
					     struct gpu_m4_exec_attachment *output)
{
	u32 i;

	for (i = 0; i < count; i++) {
		struct drm_asahi_attachment a;
		u64 end;
		int ret;

		memcpy(&a, (const u8 *)bytes + i * sizeof(a), sizeof(a));
		if (a.pointer + a.size > ~0ULL - 127)
			return -EINVAL;
		output[i].va = a.pointer & ~127ULL;
		end = (a.pointer + a.size + 127) & ~127ULL;
		output[i].size = end - output[i].va;
		if ((output[i].size >> 7) > 0xffffffffULL ||
		    !gpu_m4_drm_user_range(config->params, config->kernel_start, config->kernel_end,
					  output[i].va, output[i].size))
			return -EINVAL;
		ret = gpu_m4_drm_job_status(config->ops->check_span(config->cookie,
					 snapshot, output[i].va, output[i].size, true));
		if (ret)
			return ret;
	}
	return 0;
}

/* Borrow one already-owned snapshot for the existing compute acceptor. Its
 * put_vm callback is deliberately a no-op; the mixed batch drops the sole VM
 * lease once. Timestamp leases are still acquired/released by the established
 * compute decoder, including its own transactional per-command failures.
 */
struct gpu_m4_drm_mixed_compute_cookie {
	const struct gpu_m4_drm_job_config *config;
	void *snapshot;
};

static inline int gpu_m4_drm_mixed_snapshot(void *cookie, void **snapshot)
{
	struct gpu_m4_drm_mixed_compute_cookie *c = cookie;

	*snapshot = c->snapshot;
	return 0;
}

static inline void gpu_m4_drm_mixed_borrow_put(void *cookie, void *snapshot)
{
	(void)cookie;
	(void)snapshot;
}

static inline int gpu_m4_drm_mixed_check(void *cookie, void *snapshot,
				      u64 va, u64 size, bool write)
{
	struct gpu_m4_drm_mixed_compute_cookie *c = cookie;

	return c->config->ops->check_span(c->config->cookie, snapshot, va, size, write);
}

static inline int gpu_m4_drm_mixed_hold(void *cookie, void *snapshot,
				     const struct gpu_m4_drm_timestamp *borrowed,
				     struct gpu_m4_drm_job_timestamp *owned)
{
	struct gpu_m4_drm_mixed_compute_cookie *c = cookie;

	return c->config->ops->hold_timestamp(c->config->cookie, snapshot, borrowed, owned);
}

static inline void gpu_m4_drm_mixed_timestamp_put(void *cookie,
						struct gpu_m4_drm_job_timestamp *owned)
{
	struct gpu_m4_drm_mixed_compute_cookie *c = cookie;

	c->config->ops->put_timestamp(c->config->cookie, owned);
}

static inline int gpu_m4_drm_mixed_compute(const struct gpu_m4_drm_job_config *config,
					 const struct gpu_m4_drm_submit *submit,
					 void *snapshot, const void *payload, u32 size,
					 const void *attachments, u32 count,
					 struct gpu_m4_drm_job_compute *output,
					 struct gpu_m4_drm_job_compute *scratch)
{
	u8 stream[2 * sizeof(struct drm_asahi_cmd_header) + sizeof(struct drm_asahi_cmd_compute) +
		  GPU_M4_EXEC_ATTACHMENT_MAX * sizeof(struct drm_asahi_attachment)];
	struct drm_asahi_cmd_compute c;
	struct drm_asahi_cmd_header h = { .cmd_type = DRM_ASAHI_SET_COMPUTE_ATTACHMENTS,
		.vdm_barrier = DRM_ASAHI_BARRIER_NONE, .cdm_barrier = DRM_ASAHI_BARRIER_NONE };
	struct gpu_m4_drm_mixed_compute_cookie cookie = { config, snapshot };
	const struct gpu_m4_drm_job_ops ops = {
		.snapshot_vm = gpu_m4_drm_mixed_snapshot, .check_span = gpu_m4_drm_mixed_check,
		.put_vm = gpu_m4_drm_mixed_borrow_put, .hold_timestamp = gpu_m4_drm_mixed_hold,
		.put_timestamp = gpu_m4_drm_mixed_timestamp_put,
	};
	struct gpu_m4_drm_job_config local_config = *config;
	struct gpu_m4_drm_submit local_submit = *submit;
	struct gpu_m4_drm_job_batch batch;
	u32 offset;
	int ret = gpu_m4_drm_job_payload(config->params, payload, size, &c);

	if (ret)
		return ret;
	h.size = count * sizeof(struct drm_asahi_attachment);
	memcpy(stream, &h, sizeof(h));
	if (count)
		memcpy(stream + sizeof(h), attachments, h.size);
	offset = sizeof(h) + h.size;
	h.cmd_type = DRM_ASAHI_CMD_COMPUTE;
	h.size = sizeof(c);
	memcpy(stream + offset, &h, sizeof(h));
	memcpy(stream + offset + sizeof(h), &c, sizeof(c));
	local_config.cookie = &cookie;
	local_config.ops = &ops;
	local_submit.cmdbuf = stream;
	local_submit.size = offset + sizeof(h) + sizeof(c);
	/* Success moves timestamps straight into the unpublished mixed command;
	 * only the snapshot in this temporary batch remains borrowed.
	 */
	return gpu_m4_drm_job_compute_decode(&batch, output, scratch, 1,
					   &local_config, &local_submit);
}

static inline int gpu_m4_drm_mixed_span(const struct gpu_m4_drm_mixed_config *config,
				      void *snapshot, enum gpu_m4_drm_render_span_role role,
				      u64 va, u64 minimum, bool write,
				      struct gpu_m4_drm_render_span *span)
{
	const struct gpu_m4_drm_job_config *job = &config->job;
	int ret;

	if (!gpu_m4_drm_user_range(job->params, job->kernel_start, job->kernel_end, va, minimum))
		return -EINVAL;
	ret = gpu_m4_drm_job_status(config->resolve_span(job->cookie, snapshot,
						role, va, minimum, write, span));
	if (ret)
		return ret;
	if (span->va != va || span->size < minimum || !span->object ||
	    (write && !span->writable) ||
	    span->object_offset > ~0ULL - span->size ||
	    !gpu_m4_drm_user_range(job->params, job->kernel_start, job->kernel_end, va, span->size))
		return -EIO;
	return gpu_m4_drm_job_status(job->ops->check_span(job->cookie, snapshot,
						va, span->size, write || span->writable));
}

static inline int gpu_m4_drm_mixed_render(const struct gpu_m4_drm_mixed_config *config,
					const struct gpu_m4_drm_submit *submit, void *snapshot,
					const void *payload, u32 size,
					const void *attachments[2], const u32 count[2],
					struct gpu_m4_drm_job_render *output)
{
	struct drm_asahi_cmd_render *r = &output->uapi;
	struct drm_asahi_timestamp ts[GPU_M4_DRM_RENDER_TIMESTAMPS];
	u64 va[GPU_M4_DRM_RENDER_SPANS], minimum[GPU_M4_DRM_RENDER_SPANS] = {
		4, 4, 4, 16, 64, 64, 64, 64, 1, 1, 1, 1,
	};
	u32 high[4], offsets[4], i;
	int ret = gpu_m4_drm_render_payload(config->job.params, payload, size, r);

	if (ret)
		return ret;
	output->usc_base = config->job.usc_base;
	offsets[0] = r->bg.usc; offsets[1] = r->eot.usc;
	offsets[2] = r->partial_bg.usc; offsets[3] = r->partial_eot.usc;
	high[0] = r->bg_rsrc_spec_hi; high[1] = r->bg_eot_rsrc_spec_hi;
	high[2] = r->bg_partial_rsrc_spec_hi; high[3] = r->bg_eot_partial_rsrc_spec_hi;
	output->resources[0] = r->bg.rsrc_spec; output->resources[1] = r->eot.rsrc_spec;
	output->resources[2] = r->partial_bg.rsrc_spec;
	output->resources[3] = r->partial_eot.rsrc_spec;
	va[0] = r->vdm_ctrl_stream_base; va[1] = r->isp_scissor_base;
	va[2] = r->isp_dbias_base; va[3] = r->isp_oclqry_base;
	for (i = 0; i < 4; i++) {
		va[4 + i] = output->usc_base + (offsets[i] & ~63U);
		if (r->flags & DRM_ASAHI_RENDER_RSRC_SPEC_HI)
			output->resources[i] |= (u64)high[i] << 32;
	}
	va[8] = r->depth.base; va[9] = r->depth.comp_base;
	va[10] = r->stencil.base; va[11] = r->stencil.comp_base;
	for (i = 0; i < GPU_M4_DRM_RENDER_SPANS; i++) {
		/* Only bias, query and Z/S addresses may be absent. USC offset zero
		 * still selects a real header at the queue's execution base.
		 */
		if (!va[i] && (i == GPU_M4_DRM_RENDER_DBIAS || i == GPU_M4_DRM_RENDER_QUERY ||
				 i >= GPU_M4_DRM_RENDER_DEPTH))
			continue;
		/* Z/S admission retains readable planes and their actual write
		 * permission; it does not qualify a store, compression footprint or
		 * layered stride. A future Z/S encoder must validate these against
		 * zls_ctrl before publication. The current first encoder rejects them.
		 */
		ret = gpu_m4_drm_mixed_span(config, snapshot, i, va[i], minimum[i],
					  i == GPU_M4_DRM_RENDER_QUERY,
					  &output->spans[i]);
		if (ret)
			return ret;
		/* Publication names each present Z/S plane writable over its whole
		 * layered footprint: refuse anything less here, at submit.
		 */
		if (i == GPU_M4_DRM_RENDER_DEPTH || i == GPU_M4_DRM_RENDER_STENCIL) {
			u64 plane;

			ret = gpu_m4_drm_render_zls_plane(r, i == GPU_M4_DRM_RENDER_STENCIL, &plane);
			if (ret)
				return ret;
			if (!output->spans[i].writable || output->spans[i].size < plane)
				return -EINVAL;
		}
		if (i >= GPU_M4_DRM_RENDER_BG && i <= GPU_M4_DRM_RENDER_PARTIAL_EOT &&
		    output->spans[i].size > (1ULL << 32) - (va[i] - output->usc_base))
			return -EINVAL;
	}
	for (i = 0; i < 2; i++) {
		output->attachment_count[i] = count[i];
		ret = gpu_m4_drm_mixed_attachment(&config->job, snapshot, attachments[i], count[i],
						output->attachments[i]);
		if (ret)
			return ret;
	}
	ts[0] = r->ts_vtx.start; ts[1] = r->ts_vtx.end;
	ts[2] = r->ts_frag.start; ts[3] = r->ts_frag.end;
	memcpy(ts + 4, r->ts_frag_end_copies, sizeof(r->ts_frag_end_copies));
	for (i = 0; i < GPU_M4_DRM_RENDER_TIMESTAMPS; i++) {
		ret = gpu_m4_drm_mixed_timestamp(&config->job, submit, snapshot, ts[i],
					       &output->timestamps[i]);
		if (ret)
			return ret;
	}
	return 0;
}

/* Original-order mixed admission. Private scratch owns every new reference
 * until the final copy. Before publication or after definite retirement only,
 * mixed_put releases timestamps then the one retained whole-VM snapshot.
 * Dependencies (including future inputs) stay in the existing scheduler owner;
 * neither this acceptor nor its timestamp callbacks waits for a fence.
 */
static inline int gpu_m4_drm_mixed_decode(struct gpu_m4_drm_mixed_batch *output,
					struct gpu_m4_drm_job_command *commands,
					struct gpu_m4_drm_job_command *scratch, u32 capacity,
					const struct gpu_m4_drm_mixed_config *config,
					const struct gpu_m4_drm_submit *submit)
{
	struct gpu_m4_drm_mixed_batch pending = { .commands = scratch };
	struct drm_asahi_cmd_header h;
	struct drm_asahi_cmd_render render;
	struct drm_asahi_cmd_compute compute;
	const struct gpu_m4_drm_job_config *job;
	const struct gpu_m4_drm_job_ops *ops;
	const u8 *bytes;
	u32 offsets[3] = { 0 }, counts[3] = { 0 }, offset, count = 0, index = 0;
	unsigned long storage;
	int ret;

	if (!output || !commands || !scratch || !capacity || capacity > GPU_M4_DRM_JOB_MAX_COMMANDS ||
	    !config || !config->job.params || !config->job.ops || !config->resolve_span ||
	    !submit || !submit->cmdbuf)
		return -EINVAL;
	job = &config->job;
	ops = job->ops;
	if (!ops->snapshot_vm || !ops->check_span || !ops->put_vm ||
	    !ops->hold_timestamp || !ops->put_timestamp ||
	    job->params->vm_start < GPU_M4_EXEC_VM_START || job->params->vm_end > GPU_M4_EXEC_VM_END ||
	    job->params->vm_start >= job->params->vm_end ||
	    !job->params->max_commands_per_submission || job->params->max_commands_per_submission > 64 ||
	    !job->params->max_attachments || job->params->max_attachments > GPU_M4_EXEC_ATTACHMENT_MAX ||
	    (job->usc_base & 0xffffffffULL) || !gpu_m4_drm_aligned(job->kernel_start) ||
	    !gpu_m4_drm_aligned(job->kernel_end) || job->kernel_start < job->params->vm_start ||
	    job->kernel_end > job->params->vm_end || job->kernel_start >= job->kernel_end ||
	    !gpu_m4_drm_user_range(job->params, job->kernel_start, job->kernel_end, job->usc_base, 1ULL << 32))
		return -EINVAL;
	storage = capacity * sizeof(*commands);
	if (!gpu_m4_drm_job_separate(commands, storage, scratch, storage) ||
	    !gpu_m4_drm_job_separate(output, sizeof(*output), commands, storage) ||
	    !gpu_m4_drm_job_separate(output, sizeof(*output), scratch, storage))
		return -EINVAL;
	{
		const void *metadata[] = { config, submit, job->params, ops, submit->cmdbuf };
		const unsigned long size[] = { sizeof(*config), sizeof(*submit), sizeof(*job->params),
					       sizeof(*ops), submit->size };

		for (u32 i = 0; i < 5; i++)
			if (!gpu_m4_drm_job_separate(commands, storage, metadata[i], size[i]) ||
			    !gpu_m4_drm_job_separate(scratch, storage, metadata[i], size[i]) ||
			    !gpu_m4_drm_job_separate(output, sizeof(*output), metadata[i], size[i]))
				return -EINVAL;
	}
	ret = gpu_m4_drm_stream_validate(job->params, submit->cmdbuf, submit->size);
	if (ret)
		return ret;
	bytes = submit->cmdbuf;
	for (offset = 0; offset < submit->size; offset += h.size) {
		memcpy(&h, bytes + offset, sizeof(h));
		offset += sizeof(h);
		if (h.cmd_type >= DRM_ASAHI_SET_VERTEX_ATTACHMENTS)
			continue;
		ret = h.cmd_type == DRM_ASAHI_CMD_RENDER ?
			gpu_m4_drm_render_payload(job->params, bytes + offset, h.size, &render) :
			gpu_m4_drm_job_payload(job->params, bytes + offset, h.size, &compute);
		if (ret)
			return ret;
		count++;
	}
	if (count > capacity)
		return -ENOSPC;
	memset(scratch, 0, storage);
	pending.count = count;
	ret = gpu_m4_drm_job_status(ops->snapshot_vm(job->cookie, &pending.vm_snapshot));
	if (ret)
		return ret;
	if (!pending.vm_snapshot)
		return -EIO;
	for (offset = 0; offset < submit->size; offset += h.size) {
		struct gpu_m4_drm_job_command *command;

		memcpy(&h, bytes + offset, sizeof(h));
		offset += sizeof(h);
		if (h.cmd_type >= DRM_ASAHI_SET_VERTEX_ATTACHMENTS) {
			u32 stage = h.cmd_type - DRM_ASAHI_SET_VERTEX_ATTACHMENTS;

			offsets[stage] = offset;
			counts[stage] = h.size / sizeof(struct drm_asahi_attachment);
			continue;
		}
		command = &scratch[index++];
		command->type = h.cmd_type;
		command->vdm_barrier = h.vdm_barrier;
		command->cdm_barrier = h.cdm_barrier;
		if (h.cmd_type == DRM_ASAHI_CMD_COMPUTE) {
			ret = gpu_m4_drm_mixed_compute(job, submit, pending.vm_snapshot,
				bytes + offset, h.size, bytes + offsets[2], counts[2],
				&command->compute, &command->compute_scratch);
			command->compute.vdm_barrier = h.vdm_barrier;
			command->compute.cdm_barrier = h.cdm_barrier;
		} else {
			const void *attachments[2] = { bytes + offsets[0], bytes + offsets[1] };

			ret = gpu_m4_drm_mixed_render(config, submit, pending.vm_snapshot,
					bytes + offset, h.size, attachments, counts, &command->render);
		}
		if (ret)
			goto put;
	}
	memcpy(commands, scratch, count * sizeof(*commands));
	pending.commands = commands;
	*output = pending;
	memset(scratch, 0, storage);
	return 0;
put:
	gpu_m4_drm_mixed_put(config, &pending);
	return ret;
}

#endif
