/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_DRM_JOB_H
#define APPLE_GPU_M4_DRM_JOB_H

#include "gpu-m4-drm.h"
#include "gpu-m4-drm-validation.h"
#include "gpu-m4-exec.h"

/* Accept unchanged Mesa command bytes, not user-provided firmware work. This
 * is the compute-only accepting stage of a retained provider; it allocates no
 * hardware owner and performs no UAT, cache, notifier or ring publication.
 */
#define GPU_M4_DRM_JOB_MAX_COMMANDS 64U
#define GPU_M4_DRM_JOB_TIMESTAMPS 10U

struct gpu_m4_drm_job_timestamp {
	void *lease;
	void *backend;
	struct drm_gem_object *object;
	u64 offset;
	u64 object_offset;
	u64 firmware_va;
};

struct gpu_m4_drm_job_compute {
	u64 cdm_base;
	u64 cdm_end;
	u64 usc_base;
	u32 flags;
	u32 attachment_count;
	u16 vdm_barrier;
	u16 cdm_barrier;
	/* Normalized writable firmware hints, not an exhaustive BO access list. */
	struct gpu_m4_exec_attachment attachments[GPU_M4_EXEC_ATTACHMENT_MAX];
	/* Start, end, then eight end-copy destinations. Each owns a stable lease. */
	struct gpu_m4_drm_job_timestamp timestamps[GPU_M4_DRM_JOB_TIMESTAMPS];
	/* Kernel compute merge (provider decode): merge_count >= 2 consecutive
	 * straight-line commands of this submission became this one command.
	 * merge_stream is the owned copy of their CDM records (each member's
	 * stream minus its final terminate, in order, then one terminate),
	 * merge_size bytes, held by the submission's public job. The runtime
	 * executes it from the command's own private work page; cdm_base/end
	 * keep the first member's validated user range for diagnostics only.
	 */
	const u8 *merge_stream;
	u32 merge_size;
	u32 merge_count;
};

/* Owned merged stream placement inside the compute unit's 16 KiB private
 * work page: the firmware work image uses [0, 0x2000), the stream starts at
 * GPU_M4_DRM_MERGE_OFFSET and ends below the page end.
 */
#define GPU_M4_DRM_MERGE_OFFSET 0x3000U
#define GPU_M4_DRM_MERGE_MAX_SIZE 0x1000U

struct gpu_m4_drm_job_batch {
	void *vm_snapshot;
	struct gpu_m4_drm_job_compute *commands;
	u32 count;
};

struct gpu_m4_drm_job_ops {
	/* Synchronous prepare only, under provider VM/queue ownership. Snapshot
	 * and retain queue, VM and EVERY potentially accessed GEM/mapping owner.
	 * Bindless shaders and linked CDM jumps make attachments non-exhaustive.
	 * Conservatively classify all readable mappings as read and writable ones
	 * as GPU-written for old dma-resv dependencies and WB cache maintenance.
	 * Keep exact references, not a future lookup of a mutable VM ledger.
	 * Unsupported imported backing must fail, not borrow foreign cache pages.
	 * Error means no refs; success transfers one non-NULL owned snapshot.
	 */
	int (*snapshot_vm)(void *cookie, void **snapshot);
	/* Whole captured translation span and GPU permission check. No lookup of
	 * a current file, no page/ref release and no hardware writes. May span
	 * adjacent mappings or SINGLE_PAGE aliases belonging to this snapshot.
	 */
	int (*check_span)(void *cookie, void *snapshot, u64 va, u64 size, bool write);
	void (*put_vm)(void *cookie, void *snapshot);
	/* Retain the exact borrowed special-object and GEM mapping. Verify actual
	 * GEM bounds, write permission, private-VM identity and the qualified FW
	 * alias of its eight-byte cell. Preserve backend/object/offset identities.
	 * Include its GEM in the provider's GPU-write reservation/cache owner set.
	 * Error means no refs; success returns one non-NULL owned lease.
	 */
	int (*hold_timestamp)(void *cookie, void *snapshot,
			      const struct gpu_m4_drm_timestamp *borrowed,
			      struct gpu_m4_drm_job_timestamp *owned);
	void (*put_timestamp)(void *cookie, struct gpu_m4_drm_job_timestamp *owned);
};

struct gpu_m4_drm_job_config {
	const struct drm_asahi_params_global *params;
	u64 kernel_start;
	u64 kernel_end;
	u64 usc_base;
	void *cookie;
	const struct gpu_m4_drm_job_ops *ops;
};

static inline int gpu_m4_drm_job_status(int ret)
{
	return ret > 0 ? -EIO : ret;
}

/* Timestamp BO aliases occupy a dedicated reserved interval immediately AFTER
 * the ordinary firmware allocation window. They must not be checked with
 * gpu_m4_exec_fw_range(), which deliberately excludes this interval.
 * Subtract only after the half-open bounds check, so an eight-byte destination
 * cannot wrap, straddle the upper endpoint or substitute a client GPU VA.
 */
static inline bool gpu_m4_drm_job_timestamp_range(u64 va)
{
	const u64 start = 0xfffffc2ffc000000ULL, end = 0xfffffc3000000000ULL;

	return !(va & 7) && va >= start && va < end && end - va >= 8;
}

static inline bool gpu_m4_drm_job_separate(const void *a, unsigned long na,
					 const void *b, unsigned long nb)
{
	unsigned long va = (unsigned long)a, vb = (unsigned long)b;

	return a && b && na && nb && na <= ~0UL - va && nb <= ~0UL - vb &&
		(va + na <= vb || vb + nb <= va);
}

/* These references may be put only before publication or after definite
 * GPU/cache retirement. On uncertain hardware execution the provider must
 * transfer/pin this entire batch, not use ordinary software-job destruction.
 */
static inline void gpu_m4_drm_job_put(const struct gpu_m4_drm_job_config *config,
				     struct gpu_m4_drm_job_batch *batch)
{
	unsigned int i, j;

	if (!batch || !batch->vm_snapshot)
		return;
	for (i = 0; i < batch->count; i++)
		for (j = 0; j < GPU_M4_DRM_JOB_TIMESTAMPS; j++) {
			struct gpu_m4_drm_job_timestamp *t = &batch->commands[i].timestamps[j];

			if (t->lease)
				config->ops->put_timestamp(config->cookie, t);
			memset(t, 0, sizeof(*t));
		}
	config->ops->put_vm(config->cookie, batch->vm_snapshot);
	memset(batch, 0, sizeof(*batch));
}

static inline int gpu_m4_drm_job_payload(const struct drm_asahi_params_global *p,
					const void *bytes, unsigned int size,
					struct drm_asahi_cmd_compute *compute)
{
	unsigned int i;
	int ret;

	ret = gpu_m4_drm_payload(compute, sizeof(*compute), bytes, size);
	if (ret)
		return ret;
	if (compute->flags & ~(p->features & DRM_ASAHI_FEATURE_COMPUTE_WIDE_VISIBILITY ?
			     DRM_ASAHI_COMPUTE_WIDE_VISIBILITY : 0))
		return -EINVAL;
	if (compute->sampler_heap || compute->sampler_count || compute->helper.binary ||
	    compute->helper.cfg || compute->helper.data)
		return -EOPNOTSUPP;
	/* Exactly the span publication admits: refuse at submit, never later. */
	if (!gpu_m4_exec_cdm_span_valid(compute->cdm_ctrl_stream_base,
					compute->cdm_ctrl_stream_end))
		return -EINVAL;
	for (i = 0; i < 8; i++)
		if (compute->ts_end_copies[i].handle &&
		    (!(p->features & DRM_ASAHI_FEATURE_COMPUTE_TIMESTAMP_COPIES) ||
		     !compute->ts.end.handle))
			return -EOPNOTSUPP;
	return 0;
}

/* Validation and ownership happen in private scratch; any error releases all
 * newly acquired refs and leaves both destination objects unchanged. Call
 * while the accepting frontend lookup cookie is valid. On success scratch
 * contains no leases; ownership has moved to the caller's output batch.
 *
 * The provider still needs a separately qualified runtime lease AFTER all
 * explicit and implicit dependencies: ASID/binding generation, queue/pipe
 * credit, event sequence/slot, threshold/stamp and fresh or proven displaced
 * UMA owner. None is selected here, and an attachmentless command is valid.
 * Retain and honor both header barrier indices, including zero's preceding
 * submit dependency; serialized execution is allowed to be more conservative.
 */
static inline int gpu_m4_drm_job_compute_decode(struct gpu_m4_drm_job_batch *output,
					struct gpu_m4_drm_job_compute *commands,
					struct gpu_m4_drm_job_compute *scratch,
					unsigned int capacity,
					const struct gpu_m4_drm_job_config *config,
					const struct gpu_m4_drm_submit *submit)
{
	struct gpu_m4_drm_job_batch pending = { .commands = scratch };
	struct drm_asahi_cmd_compute c;
	struct drm_asahi_cmd_header h;
	const struct drm_asahi_params_global *p;
	const struct gpu_m4_drm_job_ops *ops;
	const u8 *bytes;
	u32 offset, count = 0, index = 0, attachments_offset = 0, attachment_count = 0;
	unsigned long storage;
	int ret;

	if (!output || !commands || !scratch || !capacity ||
	    capacity > GPU_M4_DRM_JOB_MAX_COMMANDS || !config || !config->params ||
	    !config->ops || !submit || !submit->cmdbuf)
		return -EINVAL;
	p = config->params;
	ops = config->ops;
	if (!ops->snapshot_vm || !ops->check_span || !ops->put_vm ||
	    !ops->hold_timestamp || !ops->put_timestamp ||
	    p->vm_start < GPU_M4_EXEC_VM_START || p->vm_end > GPU_M4_EXEC_VM_END ||
	    p->vm_start >= p->vm_end ||
	    !p->max_commands_per_submission || p->max_commands_per_submission > 64 ||
	    !p->max_attachments || p->max_attachments > GPU_M4_EXEC_ATTACHMENT_MAX ||
	    (config->usc_base & 0xffffffffULL) ||
	    !gpu_m4_drm_aligned(config->kernel_start) ||
	    !gpu_m4_drm_aligned(config->kernel_end) ||
	    config->kernel_start < p->vm_start || config->kernel_end > p->vm_end ||
	    config->kernel_start >= config->kernel_end ||
	    !gpu_m4_drm_user_range(p, config->kernel_start, config->kernel_end,
				config->usc_base, 1ULL << 32))
		return -EINVAL;
	storage = capacity * sizeof(*commands);
	if (!gpu_m4_drm_job_separate(commands, storage, scratch, storage) ||
	    !gpu_m4_drm_job_separate(output, sizeof(*output), commands, storage) ||
	    !gpu_m4_drm_job_separate(output, sizeof(*output), scratch, storage))
		return -EINVAL;
	{
		const void *metadata[] = { config, submit, p, ops, submit->cmdbuf };
		const unsigned long size[] = { sizeof(*config), sizeof(*submit), sizeof(*p),
					       sizeof(*ops), submit->size };

		for (unsigned int i = 0; i < 5; i++)
			if (!gpu_m4_drm_job_separate(commands, storage, metadata[i], size[i]) ||
			    !gpu_m4_drm_job_separate(scratch, storage, metadata[i], size[i]) ||
			    !gpu_m4_drm_job_separate(output, sizeof(*output), metadata[i], size[i]))
				return -EINVAL;
	}
	ret = gpu_m4_drm_stream_validate(p, submit->cmdbuf, submit->size);
	if (ret)
		return ret;
	bytes = submit->cmdbuf;
	/* Reject unsupported fields and insufficient space before retaining refs. */
	for (offset = 0; offset < submit->size; offset += h.size) {
		memcpy(&h, bytes + offset, sizeof(h));
		offset += sizeof(h);
		if (h.cmd_type == DRM_ASAHI_SET_COMPUTE_ATTACHMENTS)
			continue;
		if (h.cmd_type != DRM_ASAHI_CMD_COMPUTE)
			return -EOPNOTSUPP;
		ret = gpu_m4_drm_job_payload(p, bytes + offset, h.size, &c);
		if (ret)
			return ret;
		count++;
	}
	if (count > capacity)
		return -ENOSPC;
	memset(scratch, 0, storage);
	pending.count = count;
	ret = gpu_m4_drm_job_status(ops->snapshot_vm(config->cookie, &pending.vm_snapshot));
	if (ret)
		return ret;
	if (!pending.vm_snapshot)
		return -EIO;
	for (offset = 0; offset < submit->size; offset += h.size) {
		struct gpu_m4_drm_job_compute *command;
		struct drm_asahi_timestamp timestamps[GPU_M4_DRM_JOB_TIMESTAMPS];

		memcpy(&h, bytes + offset, sizeof(h));
		offset += sizeof(h);
		if (h.cmd_type == DRM_ASAHI_SET_COMPUTE_ATTACHMENTS) {
			attachments_offset = offset;
			attachment_count = h.size / sizeof(struct drm_asahi_attachment);
			continue;
		}
		command = &scratch[index++];
		ret = gpu_m4_drm_job_payload(p, bytes + offset, h.size, &c);
		if (ret)
			goto put;
		command->cdm_base = c.cdm_ctrl_stream_base;
		command->cdm_end = c.cdm_ctrl_stream_end;
		command->usc_base = config->usc_base;
		command->flags = c.flags;
		command->vdm_barrier = h.vdm_barrier;
		command->cdm_barrier = h.cdm_barrier;
		command->attachment_count = attachment_count;
		if (!gpu_m4_drm_user_range(p, config->kernel_start, config->kernel_end,
			command->cdm_base, command->cdm_end - command->cdm_base)) {
			ret = -EINVAL;
			goto put;
		}
		ret = gpu_m4_drm_job_status(ops->check_span(config->cookie, pending.vm_snapshot,
			command->cdm_base, command->cdm_end - command->cdm_base, false));
		if (ret)
			goto put;
		for (u32 i = 0; i < attachment_count; i++) {
			struct drm_asahi_attachment a;
			struct gpu_m4_exec_attachment *normalized = &command->attachments[i];
			u64 end;

			memcpy(&a, bytes + attachments_offset + i * sizeof(a), sizeof(a));
			if (a.pointer + a.size > ~0ULL - 127) {
				ret = -EINVAL;
				goto put;
			}
			normalized->va = a.pointer & ~127ULL;
			end = (a.pointer + a.size + 127) & ~127ULL;
			normalized->size = end - normalized->va;
			if ((normalized->size >> 7) > 0xffffffffULL ||
			    !gpu_m4_drm_user_range(p, config->kernel_start, config->kernel_end,
						 normalized->va, normalized->size)) {
				ret = -EINVAL;
				goto put;
			}
			ret = gpu_m4_drm_job_status(ops->check_span(config->cookie, pending.vm_snapshot,
					 normalized->va, normalized->size, true));
			if (ret)
				goto put;
		}
		timestamps[0] = c.ts.start;
		timestamps[1] = c.ts.end;
		memcpy(timestamps + 2, c.ts_end_copies, sizeof(c.ts_end_copies));
		for (u32 i = 0; i < GPU_M4_DRM_JOB_TIMESTAMPS; i++) {
			struct gpu_m4_drm_timestamp borrowed = { 0 };
			struct gpu_m4_drm_job_timestamp *owned = &command->timestamps[i];

			/* ABI handle zero disables this destination, including its offset. */
			if (!timestamps[i].handle)
				continue;
			if (!submit->lookup_timestamp || (timestamps[i].offset & 7)) {
				ret = -EINVAL;
				goto put;
			}
			ret = gpu_m4_drm_job_status(submit->lookup_timestamp(submit->lookup_cookie,
					 timestamps[i].handle, timestamps[i].offset, &borrowed));
			if (ret)
				goto put;
			if (!borrowed.backend || !borrowed.object ||
			    borrowed.offset != timestamps[i].offset || (borrowed.object_offset & 7)) {
				ret = -EIO;
				goto put;
			}
			ret = gpu_m4_drm_job_status(ops->hold_timestamp(config->cookie,
					 pending.vm_snapshot, &borrowed, owned));
			if (ret)
				goto put;
			if (!owned->lease || owned->backend != borrowed.backend ||
			    owned->object != borrowed.object || owned->offset != borrowed.offset ||
			    owned->object_offset != borrowed.object_offset ||
			    !gpu_m4_drm_job_timestamp_range(owned->firmware_va)) {
				ret = -EIO;
				goto put;
			}
		}
	}
	memcpy(commands, scratch, count * sizeof(*commands));
	pending.commands = commands;
	*output = pending;
	memset(scratch, 0, storage);
	return 0;
put:
	gpu_m4_drm_job_put(config, &pending);
	return ret;
}

#endif
