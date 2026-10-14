/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_USER_SUBMIT_RENDER_H
#define APPLE_GPU_M4_USER_SUBMIT_RENDER_H

#include "gpu-m4-user-render.h"

/* A borrowed slice of a real retained READY allocation. The actual job/queue
 * holds the provider session; this record neither imports nor owns fake GEMs.
 */
struct gpu_m4_render_ready_backing {
	struct m4_gpu_provider_runtime *runtime;
	struct m4_ready_buffer *source, allocation;
	u32 offset, size;
};

static int m4_user_render_session_locked(struct m4_user_job *j, u32 index, bool fill)
{
	struct m4_gpu_provider_runtime *p = j->runtime;
	struct m4_ready_buffer *source[M4_USER_RENDER_SESSION_COUNT] = {
		&p->ready->buffer[GPU_M4_ACTIVITY_TILER],
		&p->ready->buffer[GPU_M4_ACTIVITY_FRAGMENT],
		&p->execution->queue[0][M4_EXEC_QUEUE_INFO],
		&p->execution->queue[1][M4_EXEC_QUEUE_INFO],
		&p->ready->buffer[GPU_M4_PARAMETER],
	};
	static const u32 size[] = { 0xc18, 0x1248, 0x24c0, 0x24c0, 16 };
	struct m4_user_render_reservation *res = &j->render_reservation[index];
	struct gpu_m4_render_ready_backing *owner = j->session + index * M4_USER_RENDER_SESSION_COUNT;
	u32 slot = m4_user_render_fw_slot(j, index);

	if (!m4_user_native_slot_valid(DRM_ASAHI_CMD_RENDER, slot))
		return -ESTALE;
	for (u32 i = 0; i < ARRAY_SIZE(source); i++) {
		/* The PB descriptor is the leased 16-byte slice at 16 x PB ID. */
		u32 offset = i == M4_USER_RENDER_PB_DESCRIPTOR ?
			GPU_M4_RENDER_PARAMETER_DESCRIPTOR_SIZE *
			m4_user_native_slot_parameter(DRM_ASAHI_CMD_RENDER, slot) : 0;
		struct gpu_m4_render_ready_backing expected = {
			.runtime = p, .source = source[i], .allocation = *source[i],
			.offset = offset, .size = size[i],
		};
		struct gpu_m4_render_native_backing b = {
			.span = { source[i]->va + offset, size[i] },
			.kind = GPU_M4_RENDER_NATIVE_READY, .ready = &owner[i],
		};
		int ret;

		/* The PB ID is held by this command's slot lease in the single actual
		 * active lane. The allocation check is the full page, not the
		 * 16-byte FW descriptor slice.
		 */
		if (!m4_exec_backing_valid(p->hello, source[i],
			i == M4_USER_RENDER_PB_DESCRIPTOR ? GPU_M4_DRM_PAGE_SIZE : size[i], true))
			return -ESTALE;
		ret = m4_provider_page_allowed(p, source[i]->pa, source[i]);
		if (ret)
			return gpu_m4_render_native_status(ret);
		if (fill) {
			owner[i] = expected;
			res->session[i] = b;
		} else if (owner[i].runtime != p || owner[i].source != source[i] ||
		    owner[i].offset != expected.offset || owner[i].size != expected.size ||
		    !m4_provider_same_buffer(&owner[i].allocation, source[i]) ||
		    !m4_user_render_backing_same(&res->session[i], &b)) {
			return -ESTALE;
		}
	}
	return 0;
}

/* The engine addresses both global pages AFTER the client ASID switch. These
 * are exactly retained READY external leaves, not cached native substitutes.
 */
static int m4_user_render_globals_locked(struct m4_user_job *j)
{
	struct m4_gpu_provider_runtime *p = j->runtime;
	struct gpu_m4_drm_provider_job_view view = { };
	static const u32 ids[] = { GPU_M4_UMA, GPU_M4_PARAMETER };
	const u64 va[] = { p->ready->input.lower_uma, p->ready->input.lower_parameter };

	gpu_m4_drm_provider_job_view(j->core_job, &view);
	for (u32 i = 0; i < ARRAY_SIZE(ids); i++) {
		struct gpu_m4_drm_vm_leaf leaf;
		struct m4_ready_buffer *b = &p->ready->buffer[ids[i]];
		int ret = gpu_m4_drm_provider_version_page_locked(view.version, va[i],
			DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE, &leaf);

		if (ret)
			return gpu_m4_render_native_status(ret);
		if (leaf.va != va[i] || leaf.pa != b->pa || leaf.identity != b ||
		    leaf.object_offset || leaf.access != (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE) ||
		    leaf.cpu_policy != GPU_M4_DRM_CPU_NC)
			return -ESTALE;
	}
	return 0;
}

/* A cached owner is reusable only through its actual successful CORE/cache
 * receipt and the strongly held distinct same-slot replacement. Historical
 * stamps are authenticated in those receipts, never against the live notifier
 * that now belongs to B or an earlier command of this protected mixed batch.
 */
static int m4_user_render_history_locked(struct m4_user_job *j, u32 index, u64 *before)
{
	struct m4_user_native_stage *a = j->native_stage, *origin, *b;
	const struct m4_user_native_receipt *receipt;
	u32 old_index, released_index;
	int ret;

	if (!a || !m4_user_native_reused(a, index) || !a->count || a->count > GPU_M4_DRM_JOB_MAX_COMMANDS ||
	    a->count != j->command_count || index >= a->count ||
	    a->command[index].type != DRM_ASAHI_CMD_RENDER || !a->pending ||
	    a->pending->job != j->core_job || a->pending->count != a->count ||
	    a->pending->complete || a->pending->command[index].type != DRM_ASAHI_CMD_RENDER ||
	    !refcount_read(&a->refs))
		return -ESTALE;
	ret = m4_user_native_reuse_history_locked(a, index, &origin, &old_index,
		&b, &released_index);
	if (!ret)
		ret = m4_user_native_history_command_locked(origin, old_index,
			DRM_ASAHI_CMD_RENDER, &receipt);
	if (ret)
		return gpu_m4_render_native_status(ret);
	if (m4_user_native_history_job(origin->history) == j->core_job ||
	    m4_user_native_history_job(b->history) == j->core_job)
		return -ESTALE;
	/* A warm unit returns to its ORIGINAL slot and PB ID, unless it was
	 * retargeted to another slot of this bank (its PB is rebuilt anyway).
	 */
	if (index >= ARRAY_SIZE(j->cmd) ||
	    (receipt->uma_slot != j->cmd[index].uma_slot &&
	     !m4_user_native_retargeted(a, index)) ||
	    !m4_user_native_slot_valid(DRM_ASAHI_CMD_RENDER, receipt->uma_slot) ||
	    receipt->parameter_id != m4_user_native_slot_parameter(DRM_ASAHI_CMD_RENDER,
		receipt->uma_slot) || !receipt->submitted ||
	    (receipt->submitted & 1) || receipt->submitted != receipt->completed)
		return -ESTALE;
	if (receipt->submitted > ~0ULL - 2)
		return -EOVERFLOW;
	*before = receipt->submitted;
	return 0;
}

/* The same-slot predecessor C of warm unit A is the actual CURRENT owner of
 * the command's leased (A's original) render slot/PB ID. Commands of one child
 * lease distinct slots, so C is never a command of this still-protected job:
 * it always carries its own successful CORE/cache receipt. C need not be the
 * newest render, nor belong to A's queue/VM. Shared queue/notifier counters
 * describe later commands, never C: C is authenticated through ITS retained
 * job/receipt and its own live private UMA/work/Start state in that slot.
 */
static int m4_user_render_predecessor_locked(struct m4_user_job *j, u32 index)
{
	struct m4_gpu_provider_runtime *p = j->runtime;
	struct m4_user_native_stage *a = j->native_stage, *b;
	struct m4_user_native_unit *c;
	const struct m4_user_native_receipt *receipt;
	const struct m4_user_render_native *n;
	u64 counter, completed;
	void *uma, *scene;
	u32 slot, predecessor;
	int ret;

	if (!a || index >= a->count || index >= ARRAY_SIZE(j->cmd) || !a->pending ||
	    a->pending->job != j->core_job || a->pending->complete ||
	    index != j->published_count || j->cmd[index].published)
		return -ESTALE;
	slot = j->cmd[index].uma_slot;
	ret = m4_user_native_slot_owner_locked(p, DRM_ASAHI_CMD_RENDER, slot, &c);
	if (ret)
		return ret == -ENOENT ? -ESTALE : ret;
	b = c->history->owner_stage;
	predecessor = c->history_index;
	/* The held owner is the authority (never the firmware-owned table): a
	 * published, undisplaced, unborrowed unit of its own completed use.
	 */
	if (!b || b == a || !b->queue || b->queue->runtime != p || !refcount_read(&b->refs) ||
	    b->history != c->history || predecessor >= b->count || b->unit[predecessor] != c ||
	    !refcount_read(&c->refs) || !c->published || c->displacement || c->accepted || c->cold)
		return -ESTALE;
	ret = m4_user_native_history_command_locked(b, predecessor, DRM_ASAHI_CMD_RENDER, &receipt);
	if (ret)
		return gpu_m4_render_native_status(ret);
	n = m4_user_native_render(b, predecessor);
	if (!n || receipt->type != DRM_ASAHI_CMD_RENDER || receipt->uma_slot != slot ||
	    receipt->parameter_id != m4_user_native_slot_parameter(DRM_ASAHI_CMD_RENDER, slot) ||
	    !receipt->submitted || (receipt->submitted & 1) || receipt->submitted != receipt->completed)
		return -ESTALE;
	/* C can belong to a different (even closed but retained) client queue.
	 * Its own notifier/pin/cache/alias remain authority; never authenticate
	 * C through A's current snapshot.
	 */
	if (!b->notifier || !refcount_read(&b->notifier->refs) || !b->notifier->bo)
		return -ESTALE;
	/* A settled C proved its notifier through its job before releasing it;
	 * the held lease and live gate below remain.
	 */
	ret = b->history->settled ? 0 :
		m4_user_native_bo_check_locked(b->notifier->bo, p->provider, b->history->version);
	if (!ret && !b->history->settled)
		ret = gpu_m4_drm_provider_job_check_access_locked(b->history->job, p->provider,
			b->notifier->bo->object, 0, b->notifier->bo->coverage,
			M4_USER_NATIVE_RW, true, NULL);
	if (!ret)
		ret = gpu_m4_drm_upper_check_lease_locked(b->notifier->bo->upper,
			&b->notifier->bo->backing, GPU_M4_DRM_UPPER_NORMAL,
			GPU_M4_DRM_UPPER_FW_ONLY, b->notifier->bo->fw_va);
	if (ret)
		return gpu_m4_render_native_status(ret);
	uma = n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU]->map.vaddr;
	scene = n->firmware[GPU_M4_RENDER_FIRST_SCENE_FW]->map.vaddr;
	ret = m4_store_packed64(uma + 0x4c, &counter);
	if (!ret)
		ret = m4_store_packed64(uma + 0x54, &completed);
	if (ret)
		return ret;
	/* C's own live private allocator still equals its CORE-receipted use and
	 * its own descriptor still names C (+0 owner, +8 slot). The global UMA
	 * table is firmware-owned allocator state, never an owner name.
	 */
	if (m4_store_read64(uma) != n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU]->fw_va ||
	    m4_ready_read32(uma + 8) != slot || m4_ready_read32(uma + 0x5c) ||
	    counter != n->firmware[GPU_M4_RENDER_FIRST_SCENE_FW]->fw_va + 0x500 ||
	    m4_store_read64(scene + 0x500) != receipt->submitted || completed != receipt->completed)
		return -ESTALE;
	for (u32 engine = 0; engine < 2; engine++) {
		const struct gpu_m4_render_stage_retirement *r = &receipt->render[engine];
		void *work = n->work[engine]->map.vaddr;
		void *start = m4_user_render_sequence_cpu(n, engine);
		u64 work_counter;

		ret = m4_store_packed64(work + 4, &work_counter);
		if (ret)
			return ret;
		if (r->uma_descriptor_fw != n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU]->fw_va ||
		    r->uma_slot != slot || r->notifier_fw != b->notifier->bo->fw_va ||
		    r->expected_generation != b->history->binding_generation ||
		    m4_ready_read32(work + 0xc) != b->history->asid ||
		    work_counter != r->expected_counter ||
		    m4_store_read64(start + (engine ? 0x1d8 : 0x1b8)) != r->expected_counter ||
		    *((u8 *)work + (engine ? 0xc5f : 0x917)) != b->history->binding_generation)
			return -ESTALE;
	}
	/* The saved paired receipt, through C's real successful CORE/cache job
	 * (settled: already proven through it).
	 */
	if (b->history->settled)
		return 0;
	return m4_user_native_render_receipt_pair_locked(b, b->history, predecessor, NULL);
}

/* Strict prior-command edge: inside a bounded render window, every render
 * after the first queues, before its InitBuffer/TA work, an opcode-4 wait on
 * the immediately preceding render's FRAGMENT firmware stamp (+0x630, event
 * 0x71, its target; +0x3c=1). The record is this render's own retained scene
 * metadata at +0x380. Queue order alone is not an execution dependency.
 */
static bool m4_user_render_strict_prior(const struct m4_user_job *j, u32 index)
{
	return j->depth > 1 && index && index < j->command_count;
}

/*
 * FRAGMENT_DEPENDENCY early tiling (feature 6): a render declaring that its
 * dependencies are fragment-only, with no API start timestamp, whose
 * predecessor in the same packet is a render (a render-only window: no
 * compute since), lets its TA wait on that render's TA (event 0x70, +0x620)
 * instead of its FS. Its FS then strictly waits on the previous FS (event
 * 0x71, +0x630, scene +0x440, queued first on FS) and on its own TA. A start
 * timestamp or any other transition keeps the full order.
 * FRAGMENT_DEPENDENCY_COMPUTE (feature 7) is a further permission: compute
 * crossings and the packet-wide shared-pool anchor are not used here (mixed
 * windows publish serially), so a flag-6 render gets exactly this, which is
 * the more conservative order.
 */
static bool m4_user_render_early_tiling(struct m4_user_job *j, u32 index)
{
	struct gpu_m4_drm_provider_job_view view = { };
	const struct gpu_m4_drm_job_render *r;

	if (!m4_user_render_strict_prior(j, index))
		return false;
	gpu_m4_drm_provider_job_view(j->core_job, &view);
	if (!view.batch || !view.batch->commands || index >= view.batch->count ||
	    view.batch->commands[index].type != DRM_ASAHI_CMD_RENDER ||
	    view.batch->commands[index - 1].type != DRM_ASAHI_CMD_RENDER)
		return false;
	r = &view.batch->commands[index].render;
	return (r->uapi.flags & DRM_ASAHI_RENDER_FRAGMENT_DEPENDENCY) &&
		!r->timestamps[0].lease && !r->timestamps[2].lease;
}

/* Early tiling across a window boundary: the first render of this window,
 * declaring fragment-only dependencies with no start timestamp, behind its
 * own submit's immediately preceding render (the in-flight predecessor
 * window's last command on this queue's notifier).
 */
static bool cross_early_tiling = true;
#ifdef MODULE
module_param(cross_early_tiling, bool, 0644);
MODULE_PARM_DESC(cross_early_tiling, "Fragment-only early tiling across child window boundaries (default 1)");
#endif

static bool m4_user_render_cross_early(struct m4_user_job *j)
{
	struct gpu_m4_drm_provider_job_view view = { };
	const struct gpu_m4_drm_job_render *r;

	if (!cross_early_tiling || !j->cross.notifier || !j->cross.same_packet ||
	    j->cross.event != 0x71 || j->cross.notifier != j->queue->notifier || j->depth <= 1)
		return false;
	gpu_m4_drm_provider_job_view(j->core_job, &view);
	if (!view.batch || !view.batch->commands || !view.batch->count ||
	    view.batch->commands[0].type != DRM_ASAHI_CMD_RENDER)
		return false;
	r = &view.batch->commands[0].render;
	return (r->uapi.flags & DRM_ASAHI_RENDER_FRAGMENT_DEPENDENCY) &&
		!r->timestamps[0].lease && !r->timestamps[2].lease;
}

/* barrier_relax: the immediately preceding command carries no API timestamp
 * or end copy (their host-visible values are proved through its own stamps).
 */
static bool m4_user_prior_relax(struct m4_user_job *j, u32 index, u32 bit)
{
	struct gpu_m4_drm_provider_job_view view = { };
	const struct gpu_m4_drm_job_command *c;

	if (!(gpu_m4_barrier_relax & bit) || !index)
		return false;
	gpu_m4_drm_provider_job_view(j->core_job, &view);
	if (!view.batch || index >= view.batch->count)
		return false;
	c = &view.batch->commands[index - 1];
	if (c->type == DRM_ASAHI_CMD_COMPUTE) {
		for (u32 t = 0; t < GPU_M4_DRM_JOB_TIMESTAMPS; t++)
			if (c->compute.timestamps[t].lease)
				return false;
		return true;
	}
	if (c->type != DRM_ASAHI_CMD_RENDER)
		return false;
	for (u32 t = 0; t < GPU_M4_DRM_RENDER_TIMESTAMPS; t++)
		if (c->render.timestamps[t].lease)
			return false;
	return true;
}

/* prior[0]: the TA-queue wait (scene +0x380); prior[1]: the FS-queue strict
 * wait of an early-tiling render (scene +0x440), else absent.
 */
static int m4_user_render_prior_expected(struct m4_user_job *j, u32 index,
	struct gpu_m4_render_prior_dependency prior[2])
{
	const struct m4_user_render_native *n = m4_user_native_render(j->native_stage, index);
	bool early;

	memset(prior, 0, 2 * sizeof(*prior));
	/* inflight2: the first render waits on the global in-flight tail, the
	 * predecessor job's last command (fragment 0x71 or compute 15), on
	 * THAT job's held notifier page, through the same +0x380 record.
	 */
	if (!index && j->cross.notifier) {
		bool early = m4_user_render_cross_early(j);

		if (!n)
			return -ESTALE;
		prior[0] = (struct gpu_m4_render_prior_dependency) {
			.firmware = { m4_user_render_record_fw(n, M4_RREC_PRIOR_TA),
				GPU_M4_EXEC_DEPENDENCY_SIZE },
			.predecessor_stamp_fw = early ? m4_user_render_wait_fw(j->runtime,
				j->queue->notifier->bo->fw_va, 0) : j->cross.stamp_fw,
			.predecessor_target = j->cross.target,
			.predecessor_event = early ? 0x70 : j->cross.event, .strict = true,
			.relaxed = j->cross.untimed &&
				(gpu_m4_barrier_relax & (j->cross.event == 15 ? 2 : 8)),
		};
		if (early)
			prior[1] = (struct gpu_m4_render_prior_dependency) {
				.firmware = { m4_user_render_record_fw(n, M4_RREC_PRIOR_FS),
					GPU_M4_EXEC_DEPENDENCY_SIZE },
				.predecessor_stamp_fw = j->cross.stamp_fw,
				.predecessor_target = j->cross.target,
				.predecessor_event = 0x71, .fragment_internal = true,
				.relaxed = j->cross.untimed && (gpu_m4_barrier_relax & 4),
			};
		return 0;
	}
	if (!m4_user_render_strict_prior(j, index))
		return 0;
	/* Mixed window: behind a compute, the TA waits on its firmware compute
	 * stamp (+0x640, event 15) on this queue's notifier.
	 */
	if (j->mixed && !m4_user_native_render(j->native_stage, index - 1)) {
		if (!n || index - 1 >= ARRAY_SIZE(j->cmd) || !j->cmd[index - 1].published ||
		    j->cmd[index - 1].engine_mask != BIT(2) || !j->cmd[index - 1].final_target ||
		    !j->queue->notifier || !j->queue->notifier->bo)
			return -ESTALE;
		prior[0] = (struct gpu_m4_render_prior_dependency) {
			.firmware = { m4_user_render_record_fw(n, M4_RREC_PRIOR_TA),
				GPU_M4_EXEC_DEPENDENCY_SIZE },
			.predecessor_stamp_fw = m4_user_compute_wait_fw(j->runtime,
				j->queue->notifier->bo->fw_va),
			.predecessor_target = j->cmd[index - 1].final_target,
			.predecessor_event = 15, .strict = true,
			.relaxed = m4_user_prior_relax(j, index, 2),
		};
		return 0;
	}
	if (!n || !m4_user_native_render(j->native_stage, index - 1) ||
	    index - 1 >= ARRAY_SIZE(j->cmd) || !j->cmd[index - 1].published ||
	    !j->queue->notifier || !j->queue->notifier->bo)
		return -ESTALE;
	early = m4_user_render_early_tiling(j, index);
	prior[0] = (struct gpu_m4_render_prior_dependency) {
		.firmware = { m4_user_render_record_fw(n, M4_RREC_PRIOR_TA),
			GPU_M4_EXEC_DEPENDENCY_SIZE },
		.predecessor_stamp_fw = m4_user_render_wait_fw(j->runtime,
			j->queue->notifier->bo->fw_va, early ? 0 : 1),
		.predecessor_target = j->render_reservation[index - 1].chronology.target_stamp,
		.predecessor_event = early ? 0x70 : 0x71, .strict = true,
		.relaxed = m4_user_prior_relax(j, index, 8),
	};
	if (early)
		prior[1] = (struct gpu_m4_render_prior_dependency) {
			.firmware = { m4_user_render_record_fw(n, M4_RREC_PRIOR_FS),
				GPU_M4_EXEC_DEPENDENCY_SIZE },
			.predecessor_stamp_fw = m4_user_render_wait_fw(j->runtime,
				j->queue->notifier->bo->fw_va, 1),
			.predecessor_target = prior[0].predecessor_target,
			.predecessor_event = 0x71, .fragment_internal = true,
			.relaxed = m4_user_prior_relax(j, index, 4),
		};
	return 0;
}

static bool m4_user_render_prior_same(const struct gpu_m4_render_prior_dependency *a,
	const struct gpu_m4_render_prior_dependency *b)
{
	return a->firmware.va == b->firmware.va && a->firmware.size == b->firmware.size &&
		a->predecessor_stamp_fw == b->predecessor_stamp_fw &&
		a->predecessor_target == b->predecessor_target &&
		a->predecessor_event == b->predecessor_event &&
		a->fragment_internal == b->fragment_internal && a->strict == b->strict &&
		a->relaxed == b->relaxed;
}

/* Exact reservation of one command, either the next unpublished one being
 * prepared or an already reachable one. A reachable command's notifier
 * thresholds, global targets and stage counter belong to the LAST reachable
 * render of this job (its published horizon), never to every earlier one.
 */
static int m4_user_render_reservation_check_locked(struct m4_user_job *j, u32 index,
	const struct m4_user_render_reservation *res)
{
	struct m4_gpu_provider_runtime *p = j->runtime;
	struct m4_user_notifier *notifier;
	const struct m4_user_render_native *native;
	const struct gpu_m4_render_packet_chronology *c, *tail;
	struct gpu_m4_render_prior_dependency prior[2];
	const struct m4_user_command *cmd;
	struct m4_user_render_alloc *ra;
	u64 before = 0, between = 0;
	bool published;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!j->render_reservation || !j->render || !j->session ||
	    index >= j->command_count || index >= ARRAY_SIZE(j->cmd) ||
	    res != &j->render_reservation[index] || !j->prepared || j->retired ||
	    !j->queue || j->queue->runtime != p || !j->native_stage ||
	    j->native_stage->queue != j->queue || !refcount_read(&j->native_stage->refs))
		return M4_USER_WHY(-ESTALE, "render reservation %u/%u job prepared=%d retired=%d stage=%d",
			index, j->command_count, j->prepared, j->retired, !!j->native_stage);
	cmd = &j->cmd[index];
	published = cmd->published;
	if ((published ? index >= j->published_count : index != j->published_count) ||
	    (cmd->receipt && !published) || j->published_count > j->command_count ||
	    j->completed_count > j->published_count || !j->depth ||
	    j->published_count - j->completed_count > j->depth ||
	    !m4_user_native_slot_valid(DRM_ASAHI_CMD_RENDER, cmd->uma_slot))
		return M4_USER_WHY(-ESTALE, "render reservation %u window %u/%u depth=%u published=%d receipt=%d slot=%u",
			index, j->completed_count, j->published_count, j->depth, published,
			cmd->receipt, cmd->uma_slot);
	ret = m4_user_lane_check_locked(p, j->core_job);
	if (!ret)
		ret = m4_user_binding_check_locked(p);
	if (ret)
		return M4_USER_WHY(gpu_m4_render_native_status(ret),
			"render reservation %u lane/binding", index);
	notifier = j->queue->notifier;
	native = m4_user_native_render(j->native_stage, index);
	if (!notifier || !native || !refcount_read(&notifier->refs) ||
	    notifier != j->native_stage->notifier || res->notifier != notifier->bo ||
	    res->uma_slot != m4_user_render_fw_slot(j, index) ||
	    res->completion_record[0] || res->completion_record[1])
		return M4_USER_WHY(-ESTALE, "render reservation %u notifier=%d native=%d slot=%u/%u record=%u,%u",
			index, !!notifier, !!native, res->uma_slot, cmd->uma_slot,
			res->completion_record[0], res->completion_record[1]);
	c = &res->chronology;
	ra = m4_user_render_ra(j, index);
	if (ra) {
		/* The allocator's committed total when this render was prepared
		 * (it has only grown since, by this queue's later renders).
		 */
		before = c->uma_submitted_before;
		if ((!published && (before != ra->submitted ||
				    c->init_buffer == ra->initialized)) ||
		    (published && before + 2 > ra->submitted) ||
		    c->init_buffer != res->initialize_pb)
			return M4_USER_WHY(-ESTALE, "render reservation %u allocator before=%llu S=%llu init=%d/%d",
				index, M4_WHY64(before), M4_WHY64(ra->submitted), c->init_buffer,
				ra->initialized);
	} else if (m4_user_native_reused(j->native_stage, index)) {
		ret = m4_user_render_history_locked(j, index, &before);
		if (ret)
			return M4_USER_WHY(ret, "render reservation %u warm history", index);
	}
	ret = m4_user_render_prior_expected(j, index, prior);
	if (ret)
		return M4_USER_WHY(ret, "render reservation %u prior", index);
	if (c->parameter_id != m4_user_native_slot_parameter(DRM_ASAHI_CMD_RENDER,
		m4_user_render_fw_slot(j, index)) ||
	    !c->target_stamp || (c->target_stamp & 0xff) ||
	    !m4_user_render_prior_same(&c->prior[0], &prior[0]) ||
	    !m4_user_render_prior_same(&c->prior[1], &prior[1]) ||
	    (!ra && (!c->init_buffer || !res->initialize_pb)) || c->uma_submitted_before != before ||
	    c->uma_completed_before != before || c->uma_submitted != before + 2 ||
	    c->engine[1].counter > ~0ULL - 1 ||
	    c->engine[0].counter != c->engine[1].counter + 1 ||
	    !(c->engine[1].counter & 1))
		return M4_USER_WHY(-ESTALE, "render reservation %u chronology parameter=%u target=%#x counters=%llu,%llu uma=%llu/%llu",
			index, c->parameter_id, c->target_stamp, M4_WHY64(c->engine[0].counter),
			M4_WHY64(c->engine[1].counter), M4_WHY64(c->uma_submitted_before),
			M4_WHY64(before));
	tail = c;
	if (published) {
		u32 last = j->published_count - 1;

		/* The newest reachable render: only renders follow a render inside
		 * a homogeneous window; a mixed window's computes own no render
		 * sequence, target or stage counter.
		 */
		while (j->mixed && last > index && !m4_user_native_render(j->native_stage, last))
			last--;
		for (u32 i = index + 1; i <= last; i++) {
			if (!j->cmd[i].published ||
			    (!m4_user_native_render(j->native_stage, i) && !j->mixed))
				return M4_USER_WHY(-ESTALE, "render reservation %u successor %u",
					index, i);
			between += !!m4_user_native_render(j->native_stage, i);
		}
		tail = &j->render_reservation[last].chronology;
		/* One sequence per engine, one 0x100 target and two stage counters
		 * per reachable render.
		 */
		if (tail->target_stamp - c->target_stamp != between * 0x100 ||
		    tail->engine[0].counter - c->engine[0].counter != 2 * between)
			return M4_USER_WHY(-ESTALE, "render reservation %u tail %u target=%#x/%#x",
				index, last, tail->target_stamp, c->target_stamp);
	}
	for (u32 engine = 0; engine < 2; engine++) {
		const struct gpu_m4_render_engine_chronology *e = &c->engine[engine];
		const struct gpu_m4_render_engine_chronology *t = &tail->engine[engine];
		u64 sequence = notifier->sequence[engine];
		u64 threshold = published ? t->threshold_to_publish : e->threshold_before;
		void *record = notifier->bo->map.vaddr + engine * 0x200;
		/* Cached record cells are firmware-owned after exposure: their
		 * identity is the host ledger, never a live AP read.
		 */
		bool cached = notifier->record_fw[engine] != 0;

		if (e->reservation_sequence == ~0ULL || e->threshold_before != e->reservation_sequence ||
		    e->threshold_to_publish != e->reservation_sequence + 1 ||
		    (published && t->threshold_to_publish - e->threshold_to_publish != between) ||
		    /* Reachable: this job's tail, or a later same-queue job's. */
		    (published ? !m4_user_wire_notifier_tail(j, sequence, threshold, 1) :
			sequence != threshold) ||
		    notifier->threshold[engine] != sequence ||
		    m4_store_read64(notifier->bo->map.vaddr + 0x600 + 8 * engine) != sequence ||
		    (published ? !m4_user_wire_target_tail(j, engine, tail->target_stamp) :
			p->user_target[engine] != e->previous_target) ||
		    c->record_fw[engine] != notifier->record_fw[engine] ||
		    (!cached && (m4_store_read64(record) != notifier->bo->fw_va + 0x600 + engine * 8 ||
		     m4_ready_read32(record + 8) || m4_ready_read32(record + 0x10) != 0x50 ||
		     m4_ready_read32(record + 0x24) != notifier->asid ||
		     m4_ready_read32(record + 0x114) != 1)) ||
		    /* The notifier's own (and the job's) user context, set once. */
		    notifier->asid != j->asid ||
		    (!published && !sequence &&
			(m4_ready_read32(notifier->bo->map.vaddr + 0x620 + engine * 16) ||
			 m4_ready_read32(notifier->bo->map.vaddr + 0x628 + engine * 16))))
			return M4_USER_WHY(-ESTALE, "render reservation %u engine %u sequence=%llu threshold=%llu/%llu target=%#x/%#x record context=%u notifier=%u job=%u",
				index, engine, M4_WHY64(sequence), M4_WHY64(threshold),
				M4_WHY64(m4_store_read64(notifier->bo->map.vaddr + 0x600 + 8 * engine)),
				p->user_target[engine],
				published ? tail->target_stamp : e->previous_target,
				m4_ready_read32(record + 0x24), notifier->asid, j->asid);
	}
	if (published ? !m4_user_wire_notifier_tail(j, notifier->stage_counter, tail->engine[0].counter, 2) :
	    notifier->stage_counter != c->engine[0].counter - 2)
		return M4_USER_WHY(-ESTALE, "render reservation %u stage counter %llu", index,
			M4_WHY64(notifier->stage_counter));
	ret = m4_user_render_session_locked(j, index, false);
	if (ret)
		return M4_USER_WHY(ret, "render reservation %u session", index);
	return M4_USER_WHY(m4_user_render_globals_locked(j), "render reservation %u globals", index);
}

/* No descriptor/allocator reset after exposure. Freshness is proved by the
 * actual newly staged own bytes, not by a root/upper 'exposed' boolean: those
 * graph writes already precede this protected packet entrance.
 */
static int m4_user_render_fresh_locked(struct m4_user_job *j, u32 index,
	u8 descriptor[0xa0])
{
	const struct m4_user_render_native *n = m4_user_native_render(j->native_stage, index);
	struct m4_user_render_packet *packet = &j->render[index];
	struct gpu_m4_exec_vm vm = { .start = GPU_M4_DRM_UAT_PRIVATE_START,
		.end = GPU_M4_DRM_UAT_PRIVATE_END, .cookie = &packet->input,
		.check = gpu_m4_render_native_range };
	void *uma;
	int ret;

	if (!n || index >= ARRAY_SIZE(j->cmd) || j->cmd[index].published ||
	    m4_user_native_reused(j->native_stage, index))
		return -ESTALE;
	uma = n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU]->map.vaddr;
	if (memchr_inv(uma, 0, 0x4000) ||
	    memchr_inv(n->manager->map.vaddr, 0, GPU_M4_RENDER_PARAMETER_MANAGER_SIZE) ||
	    memchr_inv(n->gpu[GPU_M4_RENDER_FIRST_PB_LIST_GPU]->map.vaddr, 0,
		packet->scratch.state.pb.page_list_size) ||
	    memchr_inv(n->firmware[GPU_M4_RENDER_FIRST_PB_BLOCKS_FW]->map.vaddr, 0,
		packet->scratch.state.pb.block_list_size) ||
	    memchr_inv(n->firmware[GPU_M4_RENDER_FIRST_PB_RING_FW]->map.vaddr, 0, 8) ||
	    memchr_inv(n->firmware[GPU_M4_RENDER_FIRST_PB_COUNTER_FW]->map.vaddr, 0, 0x40) ||
	    m4_store_read64(n->firmware[GPU_M4_RENDER_FIRST_SCENE_FW]->map.vaddr + 0x500))
		return -ESTALE;
	ret = gpu_m4_exec_uma_descriptor(descriptor, &vm, n->work[0]->fw_va,
		n->firmware[GPU_M4_RENDER_FIRST_SCENE_FW]->fw_va + 0x500,
		n->gpu[GPU_M4_RENDER_FIRST_POOL_STATE_GPU]->va,
		n->gpu[GPU_M4_RENDER_FIRST_UMA_LIST_GPU]->va);
	if (ret)
		return gpu_m4_render_native_status(ret);
	/* The render owner anchor is its descriptor, not a command/user pointer. */
	put_unaligned_le64(n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU]->fw_va, descriptor);
	put_unaligned_le32(j->cmd[index].uma_slot, descriptor + 8);
	put_unaligned_le32(0, descriptor + 0x5c);
	return 0;
}

/* Called only after the new actual protected/bound job has validated ALL A
 * native/cache/alias owners. Preserve the entire firmware-mutated UMA unit;
 * only the displaced private PB records will be rebuilt. A's old completion
 * must still equal its authentic historical receipt before the new scene is
 * installed. No output/timestamp payload or old live stamp is inspected.
 */
static int m4_user_render_warm_locked(struct m4_user_job *j, u32 index)
{
	const struct m4_user_render_native *n = m4_user_native_render(j->native_stage, index);
	u64 before, counter, completed, pool, list;
	void *uma;
	int ret;

	if (!n || index >= ARRAY_SIZE(j->cmd) || j->cmd[index].published)
		return -ESTALE;
	ret = m4_user_render_history_locked(j, index, &before);
	if (ret)
		return ret;
	ret = m4_user_render_predecessor_locked(j, index);
	if (ret)
		return ret;
	uma = n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU]->map.vaddr;
	ret = m4_store_packed64(uma + 0x14, &pool);
	if (!ret)
		ret = m4_store_packed64(uma + 0x30, &list);
	if (!ret)
		ret = m4_store_packed64(uma + 0x4c, &counter);
	if (!ret)
		ret = m4_store_packed64(uma + 0x54, &completed);
	if (ret)
		return ret;
	/* Retargeted: only the descriptor's UMA index moves (+8); the
	 * allocator, cursors and counts stay; the PB manager is rebuilt with
	 * the new PB ID and a full InitBuffer like every warm reuse.
	 */
	if (m4_user_native_retargeted(j->native_stage, index) &&
	    m4_store_read64(uma) == n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU]->fw_va)
		m4_ready_write32(uma + 8, j->cmd[index].uma_slot);
	if (m4_store_read64(uma) != n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU]->fw_va ||
	    m4_ready_read32(uma + 8) != j->cmd[index].uma_slot || m4_ready_read32(uma + 0x5c) ||
	    pool != n->gpu[GPU_M4_RENDER_FIRST_POOL_STATE_GPU]->va ||
	    list != n->gpu[GPU_M4_RENDER_FIRST_UMA_LIST_GPU]->va ||
	    counter != n->firmware[GPU_M4_RENDER_FIRST_SCENE_FW]->fw_va + 0x500 ||
	    completed != before ||
	    m4_store_read64(n->firmware[GPU_M4_RENDER_FIRST_SCENE_FW]->map.vaddr + 0x500) != before)
		return -ESTALE;
	return 0;
}

/* render_shared: the allocator's M/P are this command's. The first render
 * builds M (owner anchor +0 = M's own FWVA, +8 the allocator's UMA ID, +0x4c
 * its S cell, render policy +0x5c 0) and finds M, P and the lists fresh;
 * later renders find that same M and an initialized P, preserved as the
 * firmware left them.
 */
static int m4_user_render_ra_backing_locked(struct m4_user_job *j, u32 index,
	struct m4_user_render_alloc *ra, bool init, u8 descriptor[0xa0])
{
	const struct m4_user_render_native *n = m4_user_native_render(j->native_stage, index);
	struct m4_user_render_packet *packet = &j->render[index];
	struct gpu_m4_exec_vm vm = { .start = GPU_M4_DRM_UAT_PRIVATE_START,
		.end = GPU_M4_DRM_UAT_PRIVATE_END, .cookie = &packet->input,
		.check = gpu_m4_render_native_range };
	struct m4_user_native_bo *m = ra->bo[M4_RA_UMA], *counter = ra->bo[M4_RA_COUNTER];
	void *uma;
	u64 s_cell;
	int ret;

	if (!n || index >= ARRAY_SIZE(j->cmd) || j->cmd[index].published ||
	    n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU] != m || n->manager != ra->bo[M4_RA_MANAGER] ||
	    n->parameter_heap != ra->bo[M4_RA_HEAP] ||
	    n->firmware[GPU_M4_RENDER_FIRST_PB_COUNTER_FW] != counter ||
	    n->gpu[GPU_M4_RENDER_FIRST_PB_LIST_GPU] != ra->bo[M4_RA_PB_LIST] ||
	    n->firmware[GPU_M4_RENDER_FIRST_PB_BLOCKS_FW] != ra->bo[M4_RA_PB_BLOCKS] ||
	    n->firmware[GPU_M4_RENDER_FIRST_PB_RING_FW] != ra->bo[M4_RA_PB_RING] ||
	    init != !ra->initialized)
		return M4_USER_WHY(-ESTALE, "allocator backing %u roles init=%d/%d", index, init,
			ra->initialized);
	uma = m->map.vaddr;
	s_cell = counter->fw_va + M4_RA_S_OFFSET;
	if (init) {
		if (memchr_inv(uma, 0, 0x4000) ||
		    memchr_inv(n->manager->map.vaddr, 0, GPU_M4_RENDER_PARAMETER_MANAGER_SIZE) ||
		    memchr_inv(n->gpu[GPU_M4_RENDER_FIRST_PB_LIST_GPU]->map.vaddr, 0,
			packet->scratch.state.pb.page_list_size) ||
		    memchr_inv(n->firmware[GPU_M4_RENDER_FIRST_PB_BLOCKS_FW]->map.vaddr, 0,
			packet->scratch.state.pb.block_list_size) ||
		    memchr_inv(n->firmware[GPU_M4_RENDER_FIRST_PB_RING_FW]->map.vaddr, 0, 8) ||
		    memchr_inv(counter->map.vaddr, 0, 0x100) || ra->submitted || ra->scenes)
			return M4_USER_WHY(-ESTALE, "allocator backing %u not fresh uma=%d p=%d list=%d blocks=%d ring=%d cells=%d S=%llu X=%u",
				index, !!memchr_inv(uma, 0, 0x4000),
				!!memchr_inv(n->manager->map.vaddr, 0, GPU_M4_RENDER_PARAMETER_MANAGER_SIZE),
				!!memchr_inv(n->gpu[GPU_M4_RENDER_FIRST_PB_LIST_GPU]->map.vaddr, 0,
					packet->scratch.state.pb.page_list_size),
				!!memchr_inv(n->firmware[GPU_M4_RENDER_FIRST_PB_BLOCKS_FW]->map.vaddr, 0,
					packet->scratch.state.pb.block_list_size),
				!!memchr_inv(n->firmware[GPU_M4_RENDER_FIRST_PB_RING_FW]->map.vaddr, 0, 8),
				!!memchr_inv(counter->map.vaddr, 0, 0x100), M4_WHY64(ra->submitted),
				ra->scenes);
		ret = gpu_m4_exec_uma_descriptor(descriptor, &vm, n->work[0]->fw_va, s_cell,
			n->gpu[GPU_M4_RENDER_FIRST_POOL_STATE_GPU]->va,
			n->gpu[GPU_M4_RENDER_FIRST_UMA_LIST_GPU]->va);
		if (ret)
			return gpu_m4_render_native_status(ret);
		put_unaligned_le64(m->fw_va, descriptor);
		put_unaligned_le32(ra->slot, descriptor + 8);
		put_unaligned_le32(0, descriptor + 0x5c);
		return 0;
	}
	if (m4_store_read64(uma) != m->fw_va || m4_ready_read32(uma + 8) != ra->slot ||
	    m4_ready_read32(uma + 0x5c) || m4_store_read64(uma + 0x4c) != s_cell ||
	    m4_store_read64(counter->map.vaddr + M4_RA_S_OFFSET) != ra->submitted ||
	    m4_ready_read32(counter->map.vaddr) != ra->scenes ||
	    m4_store_read64(n->manager->map.vaddr + 0x64) != counter->fw_va ||
	    m4_ready_read32(n->manager->map.vaddr + 0xc) !=
		m4_user_native_slot_parameter(DRM_ASAHI_CMD_RENDER, ra->slot))
		return M4_USER_WHY(-ESTALE, "allocator backing %u M %#llx/%#llx slot %u/%u policy %u S-cell %#llx/%#llx S %llu/%llu X %u/%u P+0x64 %#llx P+0xc %u",
			index, M4_WHY64(m4_store_read64(uma)), M4_WHY64(m->fw_va),
			m4_ready_read32(uma + 8), ra->slot, m4_ready_read32(uma + 0x5c),
			M4_WHY64(m4_store_read64(uma + 0x4c)), M4_WHY64(s_cell),
			M4_WHY64(m4_store_read64(counter->map.vaddr + M4_RA_S_OFFSET)),
			M4_WHY64(ra->submitted), m4_ready_read32(counter->map.vaddr), ra->scenes,
			M4_WHY64(m4_store_read64(n->manager->map.vaddr + 0x64)),
			m4_ready_read32(n->manager->map.vaddr + 0xc));
	return 0;
}

/* render_shared: the allocator holds its UMA ID / PB ID; its first lease
 * needs a fresh or allocator-released PB descriptor slice.
 */
static int m4_user_render_ra_slot_locked(struct m4_user_job *j, struct m4_user_render_alloc *ra,
	bool init)
{
	struct m4_gpu_provider_runtime *p = j->runtime;

	if (!m4_user_native_slot_valid(DRM_ASAHI_CMD_RENDER, ra->slot) ||
	    p->user_slot_alloc[ra->slot] != ra || test_bit(ra->slot, p->user_slot_quarantined))
		return M4_USER_WHY(-ESTALE, "allocator slot %u owner=%d quarantined=%d", ra->slot,
			ra->slot < M4_USER_SLOT_TABLE && p->user_slot_alloc[ra->slot] == ra,
			ra->slot < M4_USER_SLOT_TABLE && test_bit(ra->slot, p->user_slot_quarantined));
	/* A slot released by a quiescent allocator keeps its old PB descriptor;
	 * the first InitBuffer fully rebuilds it.
	 */
	if (init && !ra->slot_reused &&
	    memchr_inv(p->ready->buffer[GPU_M4_PARAMETER].cpu +
		GPU_M4_RENDER_PARAMETER_DESCRIPTOR_SIZE *
		m4_user_native_slot_parameter(DRM_ASAHI_CMD_RENDER, ra->slot), 0,
		GPU_M4_RENDER_PARAMETER_DESCRIPTOR_SIZE))
		return M4_USER_WHY(-ESTALE, "allocator slot %u PB descriptor in use", ra->slot);
	return 0;
}

/* Authenticate the leased slot's previous occupant from its actual derived
 * current owner (a CORE-retired unit of another job), or the genuinely unused
 * baseline: a fresh PB descriptor slice. The global UMA table is
 * firmware-owned allocator state: neither a zero record nor any packed word
 * names, or is required to name, an owner.
 */
static int m4_user_render_slot_locked(struct m4_user_job *j, u32 index)
{
	struct m4_gpu_provider_runtime *p = j->runtime;
	const struct m4_user_render_native *n;
	struct m4_user_native_unit *c;
	u32 slot;
	int ret;

	if (index >= ARRAY_SIZE(j->cmd) || index >= j->command_count)
		return -ESTALE;
	slot = j->cmd[index].uma_slot;
	if (!m4_user_native_slot_valid(DRM_ASAHI_CMD_RENDER, slot))
		return -ESTALE;
	ret = m4_user_native_slot_owner_locked(p, DRM_ASAHI_CMD_RENDER, slot, &c);
	if (ret == -ENOENT) {
		/* First lease of this PB ID must be the genuinely fresh entry. */
		return memchr_inv(p->ready->buffer[GPU_M4_PARAMETER].cpu +
			GPU_M4_RENDER_PARAMETER_DESCRIPTOR_SIZE *
			m4_user_native_slot_parameter(DRM_ASAHI_CMD_RENDER, slot), 0,
			GPU_M4_RENDER_PARAMETER_DESCRIPTOR_SIZE) ? -ESTALE : 0;
	}
	if (ret)
		return ret;
	n = c->history && c->history->owner_stage ?
		m4_user_native_render(c->history->owner_stage, c->history_index) : NULL;
	if (!n || !refcount_read(&c->refs) || c == j->native_stage->unit[index] ||
	    c->history->owner_stage == j->native_stage ||
	    !n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU])
		return -ESTALE;
	return 0;
}

#ifdef CONFIG_DEBUG_FS
/* REMOVE-DEBUG: dump the next N prepared renders at least dump_min_width
 * wide: TA/FS register lists (selector=value) and the work mirrors.
 */
static unsigned int dump_renders, dump_min_width = 3000;
module_param(dump_renders, uint, 0644);
module_param(dump_min_width, uint, 0644);

static void m4_user_render_dump(struct m4_user_job *j, u32 index,
	const struct m4_user_render_packet *packet)
{
	const u8 *t = packet->bytes.tiler, *f = packet->bytes.fragment;
	u32 w = j->native_stage->command[index].render_key.width;
	u32 h = j->native_stage->command[index].render_key.height;

	if (!READ_ONCE(dump_renders) || w < READ_ONCE(dump_min_width))
		return;
	dump_renders--;
	pr_info("RDUMP render %ux%u idx %u TA count=%u FS count=%u\n", w, h, index,
		get_unaligned_le16(t + 0x748), get_unaligned_le16(f + 0x788));
	for (u32 e = 0; e < 70; e++)
		pr_info("RDUMP TA %02u %#x=%#llx\n", e, get_unaligned_le32(t + 0x40 + e * 12),
			get_unaligned_le64(t + 0x44 + e * 12));
	for (u32 e = 0; e < 86; e++)
		pr_info("RDUMP FS %02u %#x=%#llx\n", e, get_unaligned_le32(f + 0x80 + e * 12),
			get_unaligned_le64(f + 0x84 + e * 12));
	for (u32 o = 0x930; o < 0x9e0; o += 8)
		pr_info("RDUMP FSW +%#x=%#llx\n", o, get_unaligned_le64(f + o));
	pr_info("RDUMP TAW +0x906=%u FSW +0xc4e=%u\n", t[0x906], f[0xc4e]);
}
#endif

/* REMOVE-DEBUG: render packet and timing statistics, observational only,
 * under the GPU mutex; never a publication or retirement input. Packets by
 * member count (render_batch), then, per retired render, durations from its
 * internal 24 MHz stage timestamp pairs (scene metadata +0x600 TA start/end,
 * +0x620 FS start/end; ns = ticks * 1000 / 24) and the host's CNTPCT at its
 * packet publication (the raw submit-pipe record +0x00) and at its receipt.
 * CNTPCT runs at CNTFRQ_EL0, 1 GHz on M4, not the firmware's 24 MHz: both
 * host samples are converted to 24 MHz ticks of the same counter before any
 * host/GPU difference (m4_user_render_host_ticks()). pickup is the working
 * runtime's publication gap (GPU start - publication), observe its final
 * retirement (host receipt - GPU end). Host-relative times are kept only when
 * the GPU cells are ordered and bounded against the converted host samples;
 * others count as unrelated, by the side that broke the order.
 */
enum m4_user_render_time_id {
	M4_USER_RENDER_TIME_TA,		/* TA end - TA start */
	M4_USER_RENDER_TIME_FS,		/* FS end - FS start */
	M4_USER_RENDER_TIME_TA_FS,	/* FS start - TA end, FS after TA */
	M4_USER_RENDER_TIME_OVERLAP,	/* TA end - FS start, FS before TA end */
	M4_USER_RENDER_TIME_GPU,	/* TA start -> FS end */
	M4_USER_RENDER_TIME_PICKUP,	/* packet publication -> its first TA start */
	M4_USER_RENDER_TIME_CHAIN,	/* previous member's FS end -> this TA start */
	M4_USER_RENDER_TIME_OBSERVE,	/* FS end -> host receipt */
	M4_USER_RENDER_TIME_RETIRE,	/* packet publication -> host receipt */
	M4_USER_RENDER_TIME_CROSS,	/* previous packet's last FS end -> first TA start */
	M4_USER_RENDER_TIMES,
};
#define M4_USER_RENDER_PACKET_SIZES 9U /* 1..8 members, then larger */
#define M4_USER_RENDER_TIME_BOUND (24000000ULL * 10) /* 10 s of 24 MHz ticks */
#define M4_USER_RENDER_TICK_HZ 24000000U
static struct {
	u64 count[M4_USER_RENDER_TIMES], ns[M4_USER_RENDER_TIMES], max[M4_USER_RENDER_TIMES];
	u64 packets[M4_USER_RENDER_PACKET_SIZES], packet_renders;
	u64 renders, invalid, unrelated;
	/* Unrelated renders by cause: GPU start before the publication sample
	 * (largest such lead, ns), host receipt before the GPU end (largest such
	 * lag, ns), no CNTFRQ, or any difference beyond the 10 s bound.
	 */
	u64 early, early_max, late, late_max, no_frequency, far;
	/* FS end of the latest receipted render (24 MHz ticks), any queue. */
	u64 last_fs_end;
} m4_user_render_timing;

/* A CNTPCT sample in 24 MHz ticks of the same counter (0: no CNTFRQ). */
static u64 m4_user_render_host_ticks(u64 cntpct, u32 frequency)
{
	return frequency ? mul_u64_u32_div(cntpct, M4_USER_RENDER_TICK_HZ, frequency) : 0;
}

static u64 m4_user_render_ns(u64 ticks)
{
	return ticks * 125 / 3; /* ticks * 1000 / 24, bounded above */
}

static void m4_user_render_time(u32 id, u64 ticks)
{
	u64 ns = m4_user_render_ns(ticks);

	m4_user_render_timing.count[id]++;
	m4_user_render_timing.ns[id] += ns;
	if (ns > m4_user_render_timing.max[id])
		m4_user_render_timing.max[id] = ns;
}

static void m4_user_render_timing_packet(u32 members)
{
	m4_user_render_timing.packets[min_t(u32, members, M4_USER_RENDER_PACKET_SIZES) - 1]++;
	m4_user_render_timing.packet_renders += members;
}

/* At a render's receipt: its own retained scene metadata cells. */
static void m4_user_render_timing_locked(struct m4_user_job *j, u32 index, const void *scene)
{
	const u64 bound = M4_USER_RENDER_TIME_BOUND;
	struct m4_user_command *c = &j->cmd[index];
	u64 ta0 = m4_store_read64(scene + 0x600), ta1 = m4_store_read64(scene + 0x608);
	u64 fs0 = m4_store_read64(scene + 0x620), fs1 = m4_store_read64(scene + 0x628);
	u32 frequency = arch_timer_get_cntfrq();
	u64 now = m4_user_render_host_ticks(__arch_counter_get_cntpct(), frequency);
	u64 published = c->publish_ticks ? m4_user_render_host_ticks(c->publish_ticks, frequency) : 0;
	u64 previous = index && c->wire[0].joined ? j->cmd[index - 1].fs_end_ticks : 0;
	u64 last = m4_user_render_timing.last_fs_end;

	m4_user_render_timing.renders++;
	c->fs_end_ticks = 0;
	if (published && now >= published && now - published <= bound)
		m4_user_render_time(M4_USER_RENDER_TIME_RETIRE, now - published);
	if (!ta0 || ta1 < ta0 || !fs0 || fs1 < fs0 || ta1 - ta0 > bound || fs1 - fs0 > bound ||
	    fs1 < ta0 || fs1 - ta0 > bound) {
		m4_user_render_timing.invalid++;
		return;
	}
	c->fs_end_ticks = fs1;
	m4_user_render_timing.last_fs_end = fs1;
#ifdef CONFIG_DEBUG_FS
	gpu_m4_trace(GPU_M4_TRACE_RENDER, index |
		(min_t(u32, j->native_stage->command[index].render_key.width, 4095) << 8) |
		(min_t(u32, j->native_stage->command[index].render_key.height, 4095) << 20),
		ta0, ta1, fs0, fs1);
#endif
	m4_user_render_time(M4_USER_RENDER_TIME_TA, ta1 - ta0);
	m4_user_render_time(M4_USER_RENDER_TIME_FS, fs1 - fs0);
	m4_user_render_time(M4_USER_RENDER_TIME_GPU, fs1 - ta0);
	if (fs0 >= ta1) {
		m4_user_render_time(M4_USER_RENDER_TIME_TA_FS, fs0 - ta1);
	} else {
		/* Partial render: later units of this size get a larger PB. */
		u64 area = (u64)j->native_stage->command[index].render_key.width *
			j->native_stage->command[index].render_key.height;

		if (area && (!j->runtime->user_pb_grow_area || area < j->runtime->user_pb_grow_area))
			j->runtime->user_pb_grow_area = area;
		m4_user_render_time(M4_USER_RENDER_TIME_OVERLAP, ta1 - fs0);
	}
	/* GPU-only gaps: within one packet, or across packets from the latest
	 * earlier receipt (retirement is in publication order on the one lane).
	 */
	if (previous && ta0 >= previous && ta0 - previous <= bound)
		m4_user_render_time(M4_USER_RENDER_TIME_CHAIN, ta0 - previous);
	else if (!c->wire[0].joined && last && ta0 >= last && ta0 - last <= bound)
		m4_user_render_time(M4_USER_RENDER_TIME_CROSS, ta0 - last);
	if (!frequency || !published) {
		m4_user_render_timing.unrelated++;
		m4_user_render_timing.no_frequency += !frequency;
		return;
	}
	if (ta0 < published || now < fs1) {
		m4_user_render_timing.unrelated++;
		if (ta0 < published) {
			m4_user_render_timing.early++;
			m4_user_render_timing.early_max = max(m4_user_render_timing.early_max,
				m4_user_render_ns(min(published - ta0, bound)));
		}
		if (now < fs1) {
			m4_user_render_timing.late++;
			m4_user_render_timing.late_max = max(m4_user_render_timing.late_max,
				m4_user_render_ns(min(fs1 - now, bound)));
		}
		return;
	}
	if (ta0 - published > bound || now - fs1 > bound) {
		m4_user_render_timing.unrelated++;
		m4_user_render_timing.far++;
		return;
	}
	if (!c->wire[0].joined)
		m4_user_render_time(M4_USER_RENDER_TIME_PICKUP, ta0 - published);
	m4_user_render_time(M4_USER_RENDER_TIME_OBSERVE, now - fs1);
}

/* Completion-stamp flush requests (flush_stamps, u32): TA work +0x894 and FS
 * work +0xbdc. The encoder writes 1; a batched packet keeps 1 only on its
 * newest (last) render.
 */
#define M4_USER_RENDER_TA_FLUSH 0x894U
#define M4_USER_RENDER_FS_FLUSH 0xbdcU

/* A render joining an open batched packet: its predecessor is installed (own
 * images, thresholds, targets, stage counter) but has no reachable ring entry
 * yet, and is the packet's newest member.
 */
static int m4_user_render_open_member_locked(struct m4_user_job *j, u32 index)
{
	const struct m4_user_render_native *n;
	const struct m4_user_command *c;

	if (!index || index != j->published_count || j->wire_open > index ||
	    index - 1 >= ARRAY_SIZE(j->cmd))
		return -ESTALE;
	c = &j->cmd[index - 1];
	n = m4_user_native_render(j->native_stage, index - 1);
	if (!n || !c->published || c->receipt || c->engine_mask != 3 ||
	    c->wire[0].sequence || c->wire[1].sequence ||
	    get_unaligned_le32(j->render[index - 1].bytes.tiler + M4_USER_RENDER_TA_FLUSH) != 1 ||
	    get_unaligned_le32(j->render[index - 1].bytes.fragment + M4_USER_RENDER_FS_FLUSH) != 1)
		return -ESTALE;
	return 0;
}

/* The member a newer render now follows is no longer its packet's last: its
 * flush requests become 0, in the retained image and in its installed (still
 * unreachable) work. Only the packet's newest render keeps them.
 */
static void m4_user_render_flush_clear_locked(struct m4_user_job *j, u32 index)
{
	const struct m4_user_render_native *n = m4_user_native_render(j->native_stage, index);
	struct m4_user_render_packet *packet = &j->render[index];

	put_unaligned_le32(0, packet->bytes.tiler + M4_USER_RENDER_TA_FLUSH);
	put_unaligned_le32(0, packet->bytes.fragment + M4_USER_RENDER_FS_FLUSH);
	m4_ready_write32(n->work[0]->map.vaddr + M4_USER_RENDER_TA_FLUSH, 0);
	m4_ready_write32(n->work[1]->map.vaddr + M4_USER_RENDER_FS_FLUSH, 0);
}

/* Make the open batched packet reachable: every member's entries, one pipe
 * record and one kick per engine (m4_user_wire_publish_locked). Its newest
 * member keeps the flush requests. No retry after this call.
 */
static int m4_user_render_packet_close_locked(struct m4_user_job *j)
{
	if (!j->wire_open || !j->published_count || j->wire_open > j->published_count)
		return -ESTALE;
	m4_user_render_timing_packet(j->wire_open); /* REMOVE-DEBUG */
	j->wire_open = 0;
	dma_wmb();
	return gpu_m4_render_native_status(m4_user_wire_publish_locked(j, j->published_count - 1));
}

/* Publish one render: its TA queue gets [strict prior wait (window successors
 * only), InitBuffer, TA] and its fragment queue [own-TA wait, FS]. Alone it is
 * its own packet: one submit-pipe record per engine, then kicks FS before TA.
 * A successor's reservation follows the published tail; earlier commands keep
 * executing in their own slots.
 *
 * Batched (render_batch): with defer the render is installed but its entries
 * stay unreachable (j->wire_open). The next render joins that open packet:
 * its entries append behind the predecessor's saved end and the predecessor's
 * flush requests are cleared. The packet becomes reachable with its last
 * member (defer false) or m4_user_render_packet_close_locked(): all entries,
 * one pipe record per engine naming the final producer, one FS then one TA
 * kick. Thresholds, targets and stage counters still advance per render, so
 * every member keeps its own exact retirement chronology.
 */
static int m4_user_render_packet_publish_locked(struct m4_user_job *j, u32 index, bool defer)
{
	struct m4_gpu_provider_runtime *p = j->runtime;
	struct m4_user_render_reservation *res;
	struct m4_user_render_packet *packet;
	const struct m4_user_render_native *n;
	struct m4_user_notifier *notifier;
	struct m4_user_command *cmd;
	u8 descriptor[0xa0];
	u64 ta[3], fs[3], target, before = 0;
	const char *phase = "packet-prepare";
	u32 count = 0, fs_count = 0;
	struct m4_user_render_alloc *ra;
	bool warm, init, join = j->wire_open != 0;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!j->render || !j->render_reservation || !j->session || index >= j->command_count ||
	    index >= ARRAY_SIZE(j->cmd) || index != j->published_count ||
	    j->cmd[index].published || j->cmd[index].receipt || !j->prepared || j->retired ||
	    (join && m4_user_render_open_member_locked(j, index)) ||
	    (defer && index + 1 >= j->command_count))
		return M4_USER_WHY(-ESTALE, "render publish %u/%u window=%u prepared=%d retired=%d open=%u defer=%d",
			index, j->command_count, j->published_count, j->prepared, j->retired,
			j->wire_open, defer);
	cmd = &j->cmd[index];
	if (!m4_user_native_slot_valid(DRM_ASAHI_CMD_RENDER, cmd->uma_slot))
		return M4_USER_WHY(-ESTALE, "render publish %u slot %u", index, cmd->uma_slot);
	res = &j->render_reservation[index]; packet = &j->render[index];
	n = m4_user_native_render(j->native_stage, index); notifier = j->queue->notifier;
	if (!n || !notifier || notifier != j->native_stage->notifier || !notifier->bo)
		return M4_USER_WHY(-ESTALE, "render publish %u native=%d notifier=%d", index, !!n,
			!!notifier);
	warm = m4_user_native_reused(j->native_stage, index);
	ra = m4_user_render_ra(j, index);
	init = !ra || !ra->initialized;
	if (ra) {
		before = ra->submitted;
		if (before > ~0ULL - 2 || ra->scenes == U32_MAX)
			return -EOVERFLOW;
	} else if (warm) {
		ret = m4_user_render_history_locked(j, index, &before);
		if (ret)
			return M4_USER_WHY(ret, "render publish %u warm history", index);
	}
	if (notifier->stage_counter > ~0ULL - 2 || (notifier->stage_counter & 1) ||
	    notifier->sequence[0] == ~0ULL || notifier->sequence[1] == ~0ULL)
		return -EOVERFLOW;
	target = max(p->user_target[0], p->user_target[1]);
	if (target > 0xfffffe00ULL)
		return -EOVERFLOW; /* Stop before unsupported signed-counter wrap. */
	target += 0x100;
	*res = (struct m4_user_render_reservation) {
		.notifier = notifier->bo, .uma_slot = m4_user_render_fw_slot(j, index),
		.initialize_pb = init,
		.chronology = { .uma_submitted_before = before, .uma_completed_before = before,
			.uma_submitted = before + 2, .target_stamp = target, .init_buffer = init,
			.stage_fw = { m4_user_stage_fw(p, 0), m4_user_stage_fw(p, 1) },
			.own_ta_fw = p->user_render_stage_waits ? m4_user_stage_fw(p, 0) : 0,
			.parameter_id = m4_user_native_slot_parameter(DRM_ASAHI_CMD_RENDER,
				m4_user_render_fw_slot(j, index)),
			.record_fw = { notifier->record_fw[0], notifier->record_fw[1] } },
	};
	for (u32 e = 0; e < 2; e++)
		res->chronology.engine[e] = (struct gpu_m4_render_engine_chronology) {
			.reservation_sequence = notifier->sequence[e],
			.threshold_before = notifier->sequence[e],
			.threshold_to_publish = notifier->sequence[e] + 1,
			.counter = notifier->stage_counter + (e ? 1 : 2),
			.previous_target = p->user_target[e],
		};
	ret = m4_user_render_prior_expected(j, index, res->chronology.prior);
	if (!ret)
		ret = m4_user_render_session_locked(j, index, true);
	if (!ret)
		ret = m4_user_render_globals_locked(j);
	if (ret)
		return ret;
	memset(cmd->wire, 0, sizeof(cmd->wire)); cmd->engine_mask = 0;
	if (res->chronology.prior[0].firmware.va)
		ta[count++] = res->chronology.prior[0].firmware.va;
	/* A retained, initialized allocator's P needs no PB-control entry. */
	if (init)
		ta[count++] = n->owner.init_buffer.span.va;
	ta[count++] = n->work[0]->fw_va;
	/* Early tiling: the FS strict wait on the previous FS comes first. */
	if (res->chronology.prior[1].firmware.va)
		fs[fs_count++] = res->chronology.prior[1].firmware.va;
	fs[fs_count++] = n->owner.fragment_dependency.span.va; fs[fs_count++] = n->work[1]->fw_va;
	if (join) {
		ret = m4_user_wire_join_locked(j, index, 0, ta, count);
		if (!ret)
			ret = m4_user_wire_join_locked(j, index, 1, fs, fs_count);
	} else {
		ret = m4_user_wire_reserve_locked(j, index, 0, ta, count);
		if (!ret)
			ret = m4_user_wire_reserve_locked(j, index, 1, fs, fs_count);
	}
	if (ret)
		(void)M4_USER_WHY(ret, "render %u wire reserve join=%d", index, join);
	if (!ret)
		ret = m4_user_render_prepare_locked(j, index, res, packet);
#ifdef CONFIG_DEBUG_FS
	if (!ret)
		m4_user_render_dump(j, index, packet);
#endif
	if (!ret) {
		phase = ra ? "allocator-backing" : warm ? "warm-backing" : "fresh-backing";
		ret = ra ? m4_user_render_ra_backing_locked(j, index, ra, init, descriptor) :
			warm ? m4_user_render_warm_locked(j, index) :
			m4_user_render_fresh_locked(j, index, descriptor);
	}
	if (!ret) {
		phase = "predecessor-slot";
		ret = ra ? m4_user_render_ra_slot_locked(j, ra, init) :
			m4_user_render_slot_locked(j, index);
	}
	if (!ret) {
		phase = "wire-check";
		ret = m4_user_wire_check_locked(j, index, 0);
		if (!ret)
			ret = m4_user_wire_check_locked(j, index, 1);
	}
	/* The strict wait is this render's own scene metadata subrecord, covered
	 * by its actual protected cache/resv owner, complete before any producer.
	 */
	if (!ret && res->chronology.prior[0].firmware.va &&
	    (get_unaligned_le32(packet->bytes.prior_dependency[0] + 0x3c) !=
	     !res->chronology.prior[0].relaxed ||
	     memcmp(packet->bytes.scene + 0x380, packet->bytes.prior_dependency[0],
		GPU_M4_EXEC_DEPENDENCY_SIZE)))
		ret = -ESTALE;
	if (!ret && fs_count == 3 &&
	    (get_unaligned_le32(packet->bytes.prior_dependency[1] + 0x3c) !=
	     !res->chronology.prior[1].relaxed ||
	     get_unaligned_le32(packet->bytes.prior_dependency[1] + 0x20) != 0x71 ||
	     memcmp(packet->bytes.scene + 0x440, packet->bytes.prior_dependency[1],
		GPU_M4_EXEC_DEPENDENCY_SIZE)))
		ret = -ESTALE;
	if (!ret && warm) {
		phase = "warm-reset";
		ret = m4_user_native_render_reset_locked(j->native_stage, index, j->core_job);
	}
	if (ret) {
		/* REMOVE-DEBUG: failed prepublication gate only, no payload access or
		 * new authority. One scalar breadcrumb avoids another blind retry.
		 */
		pr_warn("GPU render prepublication: phase=%s command=%u/%u slot=%u warm=%u queue_closed=%u vm_closed=%u error=%d\n",
			phase, index, j->command_count, cmd->uma_slot, warm, j->queue->closed,
			j->queue->vm->closed, ret);
		return M4_USER_WHY(gpu_m4_render_native_status(ret), "render publish %u %s asid=%u",
			index, phase, j->asid);
	}
	/* Protected actual core owns every object before ANY reachable write. */
	m4_user_native_stage_exposed_locked(j->native_stage);
	j->published = true;
	cmd->published = true;
	cmd->final_target = res->chronology.target_stamp;
	j->published_count++;
	if (ra ? init : !warm)
		memcpy(n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU]->map.vaddr, descriptor, sizeof(descriptor));
	/* Fresh UMA auxiliary records stay zero, as in the qualified initializer.
	 * Start/work contain their checked GPU/FW aliases; they are not scratch
	 * pointer records and must not be reconstructed once firmware mutates them.
	 * A retained allocator's P, lists and ring are written once, with its
	 * first (InitBuffer) render, and then left as the firmware keeps them.
	 */
	if (init) {
		memcpy(n->manager->map.vaddr, packet->parameter_manager,
			sizeof(packet->parameter_manager));
		memcpy(n->gpu[GPU_M4_RENDER_FIRST_PB_LIST_GPU]->map.vaddr, packet->pb_pages,
			packet->scratch.state.pb.page_list_size);
		memcpy(n->firmware[GPU_M4_RENDER_FIRST_PB_BLOCKS_FW]->map.vaddr, packet->pb_blocks,
			packet->scratch.state.pb.block_list_size);
		memcpy(n->firmware[GPU_M4_RENDER_FIRST_PB_RING_FW]->map.vaddr, packet->pb_ring, 8);
		/* This is the retired/displaced private manager's full rebuild, paired
		 * with InitBuffer, NOT a reset of the persistent UMA counter/cursors.
		 */
		memset(n->firmware[GPU_M4_RENDER_FIRST_PB_COUNTER_FW]->map.vaddr, 0, 0x40);
	}
	/* render_shared: publish this scene and pair in the allocator's
	 * cumulative X and S before anything becomes reachable.
	 */
	if (ra) {
		void *cells = ra->bo[M4_RA_COUNTER]->map.vaddr;

		ra->initialized = true;
		ra->scenes++;
		ra->submitted += 2;
		m4_ready_write32(cells, ra->scenes);
		put_unaligned_le64(ra->submitted, cells + M4_RA_S_OFFSET);
	}
	/* The scene image carries InitBuffer +0x300, own-TA wait +0x340, the
	 * strict prior wait +0x380 and the submitted counter +0x500.
	 */
	memcpy(n->firmware[GPU_M4_RENDER_FIRST_SCENE_FW]->map.vaddr, packet->bytes.scene, sizeof(packet->bytes.scene));
	/* Records in the cached work cells: written with the works, before
	 * any producer, immutable until this render retires.
	 */
	if (n->records_in_cell) {
		memcpy(m4_user_render_record_cpu(n, M4_RREC_OWN_TA), packet->bytes.fragment_dependency,
			sizeof(packet->bytes.fragment_dependency));
		if (init)
			memcpy(m4_user_render_record_cpu(n, M4_RREC_INIT), packet->bytes.init_buffer,
				sizeof(packet->bytes.init_buffer));
		if (res->chronology.prior[0].firmware.va)
			memcpy(m4_user_render_record_cpu(n, M4_RREC_PRIOR_TA),
				packet->bytes.prior_dependency[0], GPU_M4_EXEC_DEPENDENCY_SIZE);
		if (res->chronology.prior[1].firmware.va)
			memcpy(m4_user_render_record_cpu(n, M4_RREC_PRIOR_FS),
				packet->bytes.prior_dependency[1], GPU_M4_EXEC_DEPENDENCY_SIZE);
	}
	memcpy(m4_user_render_sequence_cpu(n, 0),
		packet->bytes.tiler_sequence, sizeof(packet->bytes.tiler_sequence));
	memcpy(m4_user_render_sequence_cpu(n, 1),
		packet->bytes.fragment_sequence, sizeof(packet->bytes.fragment_sequence));
	memcpy(n->work[0]->map.vaddr, packet->bytes.tiler, sizeof(packet->bytes.tiler));
	memcpy(n->work[1]->map.vaddr, packet->bytes.fragment, sizeof(packet->bytes.fragment));
	/* This render is now the open packet's newest member. */
	if (join)
		m4_user_render_flush_clear_locked(j, index - 1);
	/* The descriptor and slot travel only in the TA/fragment work (+0x8fe,
	 * +0xc46; descriptor +8): the global UMA table is firmware-owned.
	 */
	for (u32 e = 0; e < 2; e++) {
		if (!notifier->sequence[e]) {
			/* An unused record starts at the REAL global predecessor. Never
			 * overwrite a used client stamp to manufacture completion.
			 */
			m4_ready_write32(notifier->bo->map.vaddr + 0x620 + e * 16,
				res->chronology.engine[e].previous_target);
			/* The runtime-wide stage cell is never seeded per notifier. */
			if (!p->user_stage_cells)
				m4_ready_write32(notifier->bo->map.vaddr + 0x628 + e * 16,
					res->chronology.engine[e].previous_target);
		}
		/* The threshold names this newest reserved sequence; used stamps
		 * are kept.
		 */
		put_unaligned_le64(res->chronology.engine[e].threshold_to_publish,
			notifier->bo->map.vaddr + 0x600 + 8 * e);
		notifier->sequence[e]++; notifier->threshold[e] = notifier->sequence[e];
		notifier->published_target[e] = target;
		p->user_target[e] = target;
	}
	notifier->stage_counter += 2;
	j->all_engines |= 3;
	j->wire_open++;
	if (defer)
		return 0; /* Reachable with a later member or the packet close. */
	return m4_user_render_packet_close_locked(j);
}

/* Capture actual observations without granting retirement or changing a
 * receipt. Only the complete owned pair predicate below commits them. A
 * physical horizon other than this command's own record names reachable
 * successors on the engine (the ledger's last ends); notifier successors
 * name this notifier's published threshold and target. Each half is carried
 * independently for the through-prefix predicate.
 */
static int m4_user_render_stage_sample_locked(struct m4_user_job *j, u32 index,
	u32 engine, const struct m4_user_engine *horizon, bool notifier_successors,
	struct gpu_m4_render_stage_retirement *out)
{
	static const u16 ta_pointer[] = { 0x10, 0x20, 0x28, 0x30, 0x770, 0x8fe, 0x90f };
	static const u16 fs_pointer[] = { 0x14, 0x20, 0x28, 0x30, 0x38, 0xc46, 0xc57 };
	struct m4_gpu_provider_runtime *p = j->runtime;
	const struct m4_user_render_native *n = m4_user_native_render(j->native_stage, index);
	const struct gpu_m4_render_packet_chronology *c = &j->render_reservation[index].chronology;
	const struct m4_user_engine *wire = &j->cmd[index].wire[engine];
	/* A batched successor reachable through this command's own pipe record. */
	const struct m4_user_engine *member = index + 1 < j->published_count &&
		index + 1 < ARRAY_SIZE(j->cmd) ? &j->cmd[index + 1].wire[engine] : NULL;
	bool physical_successors = horizon != wire;
	struct m4_ready_buffer *q = p->execution->queue[engine];
	void *pipe = p->ready->buffer[GPU_M4_OBJECT_COUNT + engine * 2].cpu;
	void *notifier = j->queue->notifier->bo->map.vaddr;
	void *work = n->work[engine]->map.vaddr;
	void *start = m4_user_render_sequence_cpu(n, engine);
	const u8 *image = engine ? j->render[index].bytes.fragment : j->render[index].bytes.tiler;
	const u16 *pointer = engine ? fs_pointer : ta_pointer;
	struct gpu_m4_render_stage_retirement s = {
		.notifier_fw = j->queue->notifier->bo->fw_va,
		.uma_descriptor_fw = n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU]->fw_va,
		.uma_slot = m4_user_render_fw_slot(j, index),
		.reservation_sequence = c->engine[engine].reservation_sequence,
		.expected_threshold = c->engine[engine].threshold_to_publish,
		.threshold = m4_store_read64(notifier + 0x600 + engine * 8),
		.expected_counter = c->engine[engine].counter,
		.start_counter = m4_store_read64(start + (engine ? 0x1d8 : 0x1b8)),
		.expected_target = c->target_stamp,
		.firmware_stamp = m4_ready_read32(notifier + 0x620 + engine * 16),
		.user_stamp = p->user_stage_cells ? m4_user_stage_read(p, engine) :
			m4_ready_read32(notifier + 0x628 + engine * 16),
		.stage_horizon = m4_user_stage_tail(p, engine),
		.queue_begin = wire->queue_before, .queue_entries = wire->count, .queue_next = wire->queue_next,
		.queue_read = m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x30),
		.queue_producer = m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x40),
		.queue_busy = m4_user_qi_busy(p, q), .queue_capacity = 256,
		.pipe_begin = wire->pipe_before, .pipe_next = wire->pipe_next,
		.pipe_read = m4_ready_read32(pipe), .pipe_cfi = m4_ready_read32(pipe + 0x10),
		.pipe_producer = m4_ready_read32(pipe + 0x20), .pipe_capacity = 256,
		.expected_generation = j->binding_generation,
		.generation = *((u8 *)work + (engine ? 0xc5f : 0x917)),
		.cache_generation = m4_ready_read32(notifier + engine * 0x200 + 8),
		/* Empty render stages may have zero internal timestamp cells. */
		.require_timestamp_order = false,
		.physical_successors = physical_successors,
		.notifier_successors = notifier_successors,
		.queue_horizon = physical_successors ? horizon->queue_next : 0,
		.pipe_horizon = physical_successors ? horizon->pipe_next : 0,
		.target_horizon = notifier_successors ?
			j->queue->notifier->published_target[engine] : 0,
		.threshold_horizon = notifier_successors ?
			j->queue->notifier->threshold[engine] : 0,
		.pipe_shared = member && member->joined && member->sequence &&
			member->pipe_before == wire->pipe_before && member->pipe_next == wire->pipe_next,
		.ignore_busy = m4_user_wire_overlapped(j),
	};
	int ret = m4_store_packed64(work + 4, &s.work_counter);

	if (m4_ready_read32(work) != engine || m4_ready_read32(work + 0xc) != j->asid ||
	    m4_ready_read32(work + (engine ? 0xbe4 : 0x89c)) ||
	    m4_ready_read32(work + (engine ? 0xbe8 : 0x8a0)) ||
	    m4_ready_read32(work + (engine ? 0x1c : 0x778)) !=
		get_unaligned_le32(image + (engine ? 0x1c : 0x778)))
		return -ESTALE;
	for (u32 i = 0; i < ARRAY_SIZE(ta_pointer); i++)
		if (get_unaligned_le64(work + pointer[i]) != get_unaligned_le64(image + pointer[i]))
			return -ESTALE;
	for (u32 i = 0; i < 3; i++)
		s.queue_consumer[i] = m4_user_qi_consumer(p, q, i);
	if (!ret)
		*out = s;
	return ret;
}

/* Stack-only read-only observation tuple, not an additional job owner. */
struct m4_user_render_pair_sample {
	struct m4_user_job *job;
	u32 index;
	struct gpu_m4_render_pair_retirement pair;
	struct gpu_m4_render_retirement_dependency dependency[3];
	struct gpu_m4_render_retirement_copy copies[GPU_M4_DRM_RENDER_TIMESTAMPS];
};

static int m4_user_render_pair_owner_locked(void *cookie,
	const struct gpu_m4_render_pair_retirement *pair)
{
	struct m4_user_render_pair_sample *sample = cookie;
	struct m4_user_job *j = sample->job;
	struct m4_user_render_packet *packet = &j->render[sample->index];
	const struct m4_user_render_native *n = packet->native;
	const struct gpu_m4_drm_job_render *command = packet->input.command;
	const struct gpu_m4_render_prior_dependency *prior, *fragment;
	u32 count = 0;
	int ret;

	if (pair != &sample->pair || sample->index != j->completed_count ||
	    sample->index >= j->command_count)
		return -ESTALE;
	prior = &j->render_reservation[sample->index].chronology.prior[0];
	fragment = &j->render_reservation[sample->index].chronology.prior[1];
	if ((fragment->firmware.va && !prior->firmware.va) ||
	    (fragment->firmware.va &&
	     (sample->dependency[2].record_fw != fragment->firmware.va ||
	      sample->dependency[2].record_fw !=
		m4_user_render_record_fw(n, M4_RREC_PRIOR_FS) ||
	      sample->dependency[2].stamp_fw != fragment->predecessor_stamp_fw ||
	      sample->dependency[2].expected_target != fragment->predecessor_target ||
	      sample->dependency[2].event != 0x71 || sample->dependency[2].engine != 1)))
		return -ESTALE;
	if (pair->dependencies != sample->dependency ||
	    pair->dependency_count != 1U + !!prior->firmware.va + !!fragment->firmware.va ||
	    pair->dependency_capacity != ARRAY_SIZE(sample->dependency) ||
	    pair->copies != sample->copies ||
	    pair->copy_capacity != GPU_M4_DRM_RENDER_TIMESTAMPS ||
	    sample->dependency[0].record_fw != n->owner.fragment_dependency.span.va ||
	    (prior->firmware.va &&
	     (sample->dependency[1].record_fw != prior->firmware.va ||
	      sample->dependency[1].record_fw !=
		m4_user_render_record_fw(n, M4_RREC_PRIOR_TA) ||
	      sample->dependency[1].stamp_fw != prior->predecessor_stamp_fw ||
	      sample->dependency[1].expected_target != prior->predecessor_target ||
	      sample->dependency[1].event != prior->predecessor_event ||
	      sample->dependency[1].engine)))
		return -ESTALE;
	ret = m4_user_render_owner_locked(packet, &packet->input);
	if (ret)
		return ret;
	/* The unchanged owned operation images were compared before this call;
	 * prove complete requested endpoint coverage from the captured actual
	 * command. No timestamp BO payload, CPU-visible flag or cache operation.
	 */
	for (u32 i = 0; i < GPU_M4_DRM_RENDER_TIMESTAMPS; i++) {
		const struct gpu_m4_drm_job_timestamp *t = &command->timestamps[i];
		const struct gpu_m4_render_retirement_copy *copy;

		if (!t->lease)
			continue;
		if (count >= pair->copy_count)
			return -ESTALE;
		copy = &sample->copies[count++];
		if (copy->destination_fw != t->firmware_va ||
		    copy->destination_fw != packet->scratch.state.timestamp_va[i] ||
		    copy->engine != (i < 2 ? 0 : 1) || copy->end != (i == 1 || i >= 3))
			return -ESTALE;
	}
	return count == pair->copy_count ? 0 : -ESTALE;
}

/* Actual pre-cache hardware proof. No timestamp/output BO payload is read;
 * requested operations are part of the exact retained microsequence that
 * retired with these paired stamps/prefixes/UMA. With successors published
 * the prefixes/stamps are checked through the owned published horizon; own
 * UMA, work/Start counters and retained wait records stay exact. The job core
 * performs cache retirement and finished-fence signaling once, after ALL
 * public commands.
 */
static int m4_user_render_packet_retired_locked(struct m4_user_job *j, u32 index)
{
	struct m4_gpu_provider_runtime *p = j->runtime;
	struct m4_user_render_packet *packet;
	const struct m4_user_render_native *n;
	const struct gpu_m4_render_packet_chronology *c;
	const struct gpu_m4_render_prior_dependency *prior, *fragment;
	const struct m4_user_engine *horizon[2];
	struct m4_user_notifier *notifier;
	void *uma, *scene, *dependency, *wait, *fragment_wait;
	u64 completed, counter;
	struct m4_user_render_pair_sample sample = { .job = j, .index = index };
	struct m4_user_native_receipt *receipt;
	struct m4_user_render_alloc *ra;
	u64 s_cell, s_value = 0;
	u32 slot, tail;
	bool physical, notified;
	int ret;

	if (!j->render || !j->render_reservation || index >= j->command_count ||
	    index >= ARRAY_SIZE(j->cmd) || index != j->completed_count ||
	    !j->cmd[index].published || j->cmd[index].receipt || j->cmd[index].engine_mask != 3)
		return -ESTALE;
	ra = m4_user_render_ra(j, index);
	slot = m4_user_render_fw_slot(j, index);
	packet = &j->render[index]; n = m4_user_native_render(j->native_stage, index);
	if (!n || packet->native != n || !j->native_stage->pending ||
	    j->native_stage->pending->job != j->core_job || j->native_stage->pending->complete ||
	    index >= j->native_stage->pending->count ||
	    j->native_stage->pending->command[index].type != DRM_ASAHI_CMD_RENDER)
		return -ESTALE;
	ret = m4_user_render_owner_locked(packet, &packet->input);
	for (u32 e = 0; !ret && e < 2; e++)
		ret = m4_user_wire_horizon_locked(j, e, &horizon[e]);
	if (ret)
		return gpu_m4_render_native_status(ret);
	c = &j->render_reservation[index].chronology;
	prior = &c->prior[0];
	fragment = &c->prior[1];
	/* Physical: both engines end at the same last reachable render record.
	 * Notifier: one target names this notifier's newest render on both
	 * engines, at or after this command's own.
	 */
	notifier = j->queue->notifier;
	tail = notifier->published_target[0];
	physical = horizon[0] != &j->cmd[index].wire[0];
	notified = tail != c->target_stamp;
	if (physical != (horizon[1] != &j->cmd[index].wire[1]) ||
	    notifier->published_target[1] != tail ||
	    !gpu_m4_exec_stamp_reached(tail, c->target_stamp))
		return -ESTALE;
	uma = n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU]->map.vaddr;
	scene = n->firmware[GPU_M4_RENDER_FIRST_SCENE_FW]->map.vaddr;
	dependency = m4_user_render_record_cpu(n, M4_RREC_OWN_TA);
	wait = m4_user_render_record_cpu(n, M4_RREC_PRIOR_TA);
	fragment_wait = m4_user_render_record_cpu(n, M4_RREC_PRIOR_FS);
	ret = m4_store_packed64(uma + 0x4c, &counter);
	if (!ret)
		ret = m4_store_packed64(uma + 0x54, &completed);
	if (ret)
		return ret;
	/* render_shared: M's S cell is the allocator's, published cumulatively
	 * by this and later renders; this pair retired once M completed passed
	 * its own S, bounded by everything the allocator has published.
	 */
	s_cell = ra ? ra->bo[M4_RA_COUNTER]->fw_va + M4_RA_S_OFFSET :
		n->firmware[GPU_M4_RENDER_FIRST_SCENE_FW]->fw_va + 0x500;
	if (m4_store_read64(uma) != n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU]->fw_va ||
	    m4_ready_read32(uma + 8) != slot || m4_ready_read32(uma + 0x5c) || counter != s_cell ||
	    (ra && (n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU] != ra->bo[M4_RA_UMA] ||
		    c->uma_submitted > ra->submitted)))
		return M4_USER_WHY(-ESTALE, "render retire %u M %#llx slot %u/%u policy %u S-cell %#llx/%#llx shared=%d",
			index, M4_WHY64(m4_store_read64(uma)), m4_ready_read32(uma + 8), slot,
			m4_ready_read32(uma + 0x5c), M4_WHY64(counter), M4_WHY64(s_cell), !!ra);
	if (ra) {
		s_value = m4_store_read64(ra->bo[M4_RA_COUNTER]->map.vaddr + M4_RA_S_OFFSET);
		if (s_value != ra->submitted)
			return M4_USER_WHY(-ESTALE, "render retire %u S %llu/%llu", index,
				M4_WHY64(s_value), M4_WHY64(ra->submitted));
		if (completed < c->uma_submitted)
			return -EAGAIN;
		if (completed > s_value)
			return M4_USER_WHY(-EPROTO, "render retire %u completed %llu > S %llu", index,
				M4_WHY64(completed), M4_WHY64(s_value));
	} else if (m4_store_read64(scene + 0x500) != c->uma_submitted ||
		   completed != c->uma_submitted) {
		return -EAGAIN;
	}
	/* Both exact frozen operation streams remain retained through success.
	 * They contain every requested endpoint/copy; no CPU readback substitutes
	 * for their completed hardware sequence.
	 */
	if (memcmp(m4_user_render_sequence_cpu(n, 0),
		packet->bytes.tiler_sequence, sizeof(packet->bytes.tiler_sequence)) ||
	    memcmp(m4_user_render_sequence_cpu(n, 1),
		packet->bytes.fragment_sequence, sizeof(packet->bytes.fragment_sequence)) ||
	    memcmp(dependency, packet->bytes.fragment_dependency, GPU_M4_EXEC_DEPENDENCY_SIZE) ||
	    m4_ready_read32(dependency) != 4 ||
	    get_unaligned_le64(dependency + 4) != m4_user_render_wait_fw(p, j->queue->notifier->bo->fw_va, 0) ||
	    get_unaligned_le64(dependency + 0xc) != m4_user_render_wait_fw(p, j->queue->notifier->bo->fw_va, 0) ||
	    m4_ready_read32(dependency + 0x14) != c->target_stamp ||
	    m4_ready_read32(dependency + 0x20) != 0x70 ||
	    m4_ready_read32(dependency + 0x24) != c->target_stamp || m4_ready_read32(dependency + 0x3c))
		return -ESTALE;
	/* The retained strict prior-command wait, unchanged since publication. */
	if (prior->firmware.va &&
	    (prior->firmware.va != m4_user_render_record_fw(n, M4_RREC_PRIOR_TA) ||
	     memcmp(wait, packet->bytes.prior_dependency[0], GPU_M4_EXEC_DEPENDENCY_SIZE) ||
	     m4_ready_read32(wait) != 4 ||
	     get_unaligned_le64(wait + 4) != prior->predecessor_stamp_fw ||
	     get_unaligned_le64(wait + 0xc) != prior->predecessor_stamp_fw ||
	     m4_ready_read32(wait + 0x14) != prior->predecessor_target ||
	     m4_ready_read32(wait + 0x20) != prior->predecessor_event ||
	     m4_ready_read32(wait + 0x24) != c->target_stamp ||
	     m4_ready_read32(wait + 0x3c) != !prior->relaxed))
		return -ESTALE;
	/* An early-tiling render's retained FS strict wait on the previous FS. */
	if (fragment->firmware.va &&
	    (fragment->firmware.va != m4_user_render_record_fw(n, M4_RREC_PRIOR_FS) ||
	     memcmp(fragment_wait, packet->bytes.prior_dependency[1], GPU_M4_EXEC_DEPENDENCY_SIZE) ||
	     m4_ready_read32(fragment_wait) != 4 ||
	     get_unaligned_le64(fragment_wait + 4) != fragment->predecessor_stamp_fw ||
	     get_unaligned_le64(fragment_wait + 0xc) != fragment->predecessor_stamp_fw ||
	     m4_ready_read32(fragment_wait + 0x14) != fragment->predecessor_target ||
	     m4_ready_read32(fragment_wait + 0x20) != 0x71 ||
	     m4_ready_read32(fragment_wait + 0x24) != c->target_stamp ||
	     m4_ready_read32(fragment_wait + 0x3c) != !fragment->relaxed))
		return -ESTALE;
	for (u32 engine = 0; engine < 2; engine++) {
		ret = m4_user_render_stage_sample_locked(j, index, engine, horizon[engine],
			notified, &sample.pair.stage[engine]);
		if (ret)
			return ret;
	}
	sample.dependency[0] = (struct gpu_m4_render_retirement_dependency) {
		.record_fw = n->owner.fragment_dependency.span.va,
		.stamp_fw = get_unaligned_le64(dependency + 4),
		.expected_target = c->target_stamp, .consuming_target = c->target_stamp,
		.firmware_stamp = sample.pair.stage[0].firmware_stamp,
		.event = 0x70, .engine = 1, .completed = true,
		.stamp_horizon = notified ? tail : 0,
	};
	if (prior->firmware.va && !index && j->cross.notifier)
		/* Global-tail barrier: the predecessor job's cumulative FS (or
		 * compute) firmware stamp on its own held page passed its target;
		 * it never passes that notifier's published tail.
		 */
		sample.dependency[1] = (struct gpu_m4_render_retirement_dependency) {
			.record_fw = prior->firmware.va, .stamp_fw = prior->predecessor_stamp_fw,
			.expected_target = prior->predecessor_target,
			.consuming_target = c->target_stamp,
			/* A runtime-wide stage cell is observed below. */
			.firmware_stamp = prior->predecessor_stamp_fw >= j->cross.notifier->bo->fw_va &&
				prior->predecessor_stamp_fw - j->cross.notifier->bo->fw_va <
				GPU_M4_DRM_PAGE_SIZE ? m4_ready_read32(j->cross.notifier->bo->map.vaddr +
				(prior->predecessor_stamp_fw - j->cross.notifier->bo->fw_va)) : 0,
			.event = prior->predecessor_event, .engine = 0, .completed = true,
			/* Cross early tiling waits on that render's TA stamp. */
			.stamp_horizon = j->cross.notifier->published_target[
				prior->predecessor_event == 0x70 ? 0 : j->cross.engine],
		};
	else if (prior->firmware.va && prior->predecessor_event == 15)
		/* Mixed window: the preceding compute's cumulative firmware stamp
		 * on this notifier passed its target; it never passes the
		 * notifier's published compute tail.
		 */
		sample.dependency[1] = (struct gpu_m4_render_retirement_dependency) {
			.record_fw = prior->firmware.va, .stamp_fw = prior->predecessor_stamp_fw,
			.expected_target = prior->predecessor_target,
			.consuming_target = c->target_stamp,
			.firmware_stamp = m4_ready_read32(notifier->bo->map.vaddr + 0x640),
			.event = 15, .engine = 0, .completed = true,
			.stamp_horizon = notifier->published_target[2],
		};
	else if (prior->firmware.va)
		/* The preceding render's cumulative FS (early tiling: TA) firmware
		 * stamp passed its target; it can never pass this window's
		 * published tail.
		 */
		sample.dependency[1] = (struct gpu_m4_render_retirement_dependency) {
			.record_fw = prior->firmware.va, .stamp_fw = prior->predecessor_stamp_fw,
			.expected_target = prior->predecessor_target,
			.consuming_target = c->target_stamp,
			.firmware_stamp = sample.pair.stage[prior->predecessor_event == 0x70 ? 0 : 1].firmware_stamp,
			.event = prior->predecessor_event, .engine = 0, .completed = true,
			.stamp_horizon = tail,
		};
	if (fragment->firmware.va)
		sample.dependency[2] = (struct gpu_m4_render_retirement_dependency) {
			.record_fw = fragment->firmware.va, .stamp_fw = fragment->predecessor_stamp_fw,
			.expected_target = fragment->predecessor_target,
			.consuming_target = c->target_stamp,
			.firmware_stamp = sample.pair.stage[1].firmware_stamp,
			.event = 0x71, .engine = 1, .completed = true,
			.stamp_horizon = tail,
		};
	for (u32 i = 0; i < GPU_M4_DRM_RENDER_TIMESTAMPS; i++)
		if (packet->input.command->timestamps[i].lease)
			sample.copies[sample.pair.copy_count++] = (struct gpu_m4_render_retirement_copy) {
				.destination_fw = packet->scratch.state.timestamp_va[i],
				.engine = i < 2 ? 0 : 1, .end = i == 1 || i >= 3, .completed = true,
			};
	/* Waits naming a runtime-wide stage cell observe that event's in-order
	 * frontier, bounded by the event's published tail (any queue's).
	 */
	for (u32 i = 0; i < 3; i++) {
		struct gpu_m4_render_retirement_dependency *d = &sample.dependency[i];

		if (d->record_fw)
			m4_user_stage_observe(p, d->stamp_fw, &d->firmware_stamp, &d->stamp_horizon);
	}
	sample.pair.ta_wait_fw = c->own_ta_fw;
	sample.pair.uma_descriptor_fw = n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU]->fw_va;
	sample.pair.uma_submitted_fw = counter;
	sample.pair.expected_uma_slot = sample.pair.uma_slot = slot;
	sample.pair.uma_submitted_before = c->uma_submitted_before;
	sample.pair.uma_expected_submitted = sample.pair.uma_submitted = c->uma_submitted;
	sample.pair.uma_submitted_horizon = s_value;
	sample.pair.uma_completed_high_before = sample.pair.uma_completed_high_after = completed >> 32;
	sample.pair.uma_completed_low = completed;
	sample.pair.copies = sample.copies;
	sample.pair.copy_capacity = GPU_M4_DRM_RENDER_TIMESTAMPS;
	sample.pair.dependencies = sample.dependency;
	sample.pair.dependency_count = 1 + !!prior->firmware.va + !!fragment->firmware.va;
	sample.pair.dependency_capacity = ARRAY_SIZE(sample.dependency);
	sample.pair.init_buffer = c->init_buffer;
	/* The actual owner/health callback rechecks these phase facts under the
	 * same provider mutex; none is a caller policy granting scalar success.
	 */
	sample.pair.rtkit_healthy = sample.pair.run_healthy = sample.pair.control_healthy = true;
	sample.pair.binding_healthy = sample.pair.fault_drain_complete = true;
	ret = gpu_m4_render_pair_hardware_retired(&sample.pair, &sample, m4_user_render_pair_owner_locked);
	if (ret)
		return M4_USER_WHY(ret, "render retire %u pair S %llu..%llu completed %llu horizon %llu init=%d deps=%u entries=%u,%u",
			index, M4_WHY64(c->uma_submitted_before), M4_WHY64(c->uma_submitted),
			M4_WHY64(completed), M4_WHY64(s_value), c->init_buffer,
			sample.pair.dependency_count, sample.pair.stage[0].queue_entries,
			sample.pair.stage[1].queue_entries);
	/* Raw observations, including the horizon they were taken within. */
	receipt = &j->native_stage->pending->command[index];
	memcpy(receipt->render, sample.pair.stage, sizeof(receipt->render));
	memcpy(receipt->render_dependency, sample.dependency, sizeof(receipt->render_dependency));
	receipt->render_dependency_count = sample.pair.dependency_count;
	memcpy(receipt->render_copies, sample.copies, sizeof(receipt->render_copies));
	receipt->render_copy_count = sample.pair.copy_count;
	/* A shared render's receipt keeps its unit's host slot token; the
	 * firmware slot and PB ID are its allocator's.
	 */
	receipt->uma_slot = j->cmd[index].uma_slot;
	receipt->fw_slot = slot;
	receipt->shared = !!ra;
	receipt->init_buffer = c->init_buffer;
	receipt->observed_submitted = s_value;
	receipt->parameter_id = c->parameter_id;
	receipt->submitted = c->uma_submitted;
	receipt->completed = completed;
	m4_user_render_timing_locked(j, index, scene); /* REMOVE-DEBUG */
	return 0;
}

/* Fault-only observation of the exact retained NC packet. No receipt/cache/
 * fence transition: the scalar retired result below deliberately does not
 * call the live receipt-writing stage helper. Never dereference recovered VAs.
 * Provider/VM error is ALREADY latched here. Prove retained inventory/mapping,
 * not healthy publication permission, or the snapshot would hide every fault.
 */
static bool m4_user_render_fault_bo(const struct m4_user_job *j,
	const struct m4_user_native_bo *b, u64 minimum, bool notifier)
{
	const struct m4_user_native_stage *stage = j->native_stage;
	bool owned = false;

	if (!stage || stage->bo_count > ARRAY_SIZE(stage->bo))
		return false;
	if (notifier)
		owned = b == j->queue->notifier->bo && stage->notifier == j->queue->notifier;
	else
		for (u32 i = 0; i < stage->bo_count; i++)
			owned |= b == stage->bo[i];
	/* Do not inspect b until it is an exact entry in a retained owner. The
	 * BO retains its actual accounting object, native pin/GEM and NC map even
	 * after a provider fault. These getters perform no healthy-state query.
	 */
	return owned && b && b->runtime == j->runtime && refcount_read(&b->refs) &&
		b->mapped && b->gem && b->object && b->pin && minimum &&
		b->coverage >= minimum && b->coverage <= b->size &&
		b->size && !(b->size & (GPU_M4_DRM_PAGE_SIZE - 1)) &&
		!iosys_map_is_null(&b->map) && !b->map.is_iomem &&
		!((unsigned long)b->map.vaddr & (GPU_M4_DRM_PAGE_SIZE - 1)) &&
		gpu_m4_drm_provider_object_native(b->object) == b->pin &&
		gpu_m4_drm_provider_object_identity(b->object) == b->gem &&
		gpu_m4_drm_bo_pin_object(b->pin) == b->gem &&
		gpu_m4_drm_bo_pin_size(b->pin) == b->size &&
		gpu_m4_drm_bo_policy(b->gem) == GPU_M4_DRM_CPU_NC;
}

static void m4_user_render_fault_snapshot_locked(struct m4_user_job *j, u32 index)
{
	struct m4_gpu_provider_runtime *p;
	struct gpu_m4_drm_provider_job_view view = { };
	const struct m4_user_render_native *n;
	const struct gpu_m4_render_packet_chronology *c;
	u64 completed = 0, counter = 0;
	void *uma, *scene, *notifier;
	int ret, packed;

	if (!j || !j->runtime)
		return;
	p = j->runtime;
	lockdep_assert_held(&p->hello->gpu->lock);
	if (!p->ready || !p->execution || !j->native_stage || !j->queue ||
	    !j->queue->notifier || !refcount_read(&j->queue->notifier->refs) || !j->render ||
	    !j->render_reservation || !j->core_job || index >= j->command_count ||
	    index >= ARRAY_SIZE(j->cmd) || !j->cmd[index].published ||
	    j->cmd[index].engine_mask != 3 ||
	    j->queue->runtime != p || j->native_stage->queue != j->queue ||
	    !refcount_read(&j->native_stage->refs) ||
	    !j->command_count || j->command_count > GPU_M4_DRM_JOB_MAX_COMMANDS ||
	    j->native_stage->count != j->command_count || p->execution != p->ready->execution)
		return;
	gpu_m4_drm_provider_job_view(j->core_job, &view);
	if (view.runtime_job != j || view.runtime_queue != j->queue ||
	    view.stage_owner != j->native_stage || !view.batch || !view.batch->commands ||
	    view.batch->count != j->command_count || index >= view.batch->count ||
	    view.batch->commands[index].type != DRM_ASAHI_CMD_RENDER)
		return;
	n = m4_user_native_render(j->native_stage, index);
	if (!n || j->render[index].native != n)
		return;
	ret = m4_user_render_fault_bo(j, n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU], 0x60, false) &&
		m4_user_render_fault_bo(j, n->firmware[GPU_M4_RENDER_FIRST_SCENE_FW], 0x508, false) &&
		m4_user_render_fault_bo(j, j->queue->notifier->bo, 0x650, true) ? 0 : -ESTALE;
	for (u32 e = 0; !ret && e < 2; e++) {
		struct m4_ready_buffer *q = p->execution->queue[e];
		struct m4_ready_buffer *pipe = &p->ready->buffer[GPU_M4_OBJECT_COUNT + e * 2];

		/* m4_exec_backing_valid checks only retained DMA/RAM inventory, not
		 * provider/VM health. Do not invoke lane/job publication validators.
		 */
		if (!m4_user_render_fault_bo(j, n->work[e], e ? 0xc60 : 0x918, false) ||
		    !m4_user_render_fault_bo(j, n->firmware[e ? GPU_M4_RENDER_FIRST_FRAGMENT_SEQUENCE_FW :
			GPU_M4_RENDER_FIRST_TILER_SEQUENCE_FW], e ? 0x1e0 : 0x1c0, false) ||
		    !m4_exec_backing_valid(p->hello, &q[M4_EXEC_QUEUE_INFO], GPU_M4_EXEC_QUEUE_SIZE, true) ||
		    !m4_exec_backing_valid(p->hello, &q[M4_EXEC_QUEUE_STATE], GPU_M4_EXEC_RING_STATE_SIZE, true) ||
		    !m4_exec_backing_valid(p->hello, pipe, 0x30, true))
			ret = -ESTALE;
	}
	if (ret) {
		dev_err(p->hello->gpu->dev, "GPU render fault snapshot: owner check=%d command=%u\n", ret, index);
		return;
	}
	c = &j->render_reservation[index].chronology;
	uma = n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU]->map.vaddr;
	scene = n->firmware[GPU_M4_RENDER_FIRST_SCENE_FW]->map.vaddr;
	notifier = j->queue->notifier->bo->map.vaddr;
	packed = m4_store_packed64(uma + 0x4c, &counter);
	if (!packed)
		packed = m4_store_packed64(uma + 0x54, &completed);
	dev_err(p->hello->gpu->dev,
		"GPU render fault UMA: command=%u slot=%u prior=%u reused=%u submitted=%llu completed=%llu expected=%llu counter=%#llx expected_counter=%#llx packed=%d target=%#x binding=%llu/%u\n",
		index, j->cmd[index].uma_slot, !!c->prior[0].firmware.va,
		m4_user_native_reused(j->native_stage, index), (unsigned long long)m4_store_read64(scene + 0x500),
		(unsigned long long)completed, (unsigned long long)c->uma_submitted,
		(unsigned long long)counter,
		(unsigned long long)(n->firmware[GPU_M4_RENDER_FIRST_SCENE_FW]->fw_va + 0x500),
		packed, c->target_stamp, (unsigned long long)j->binding_epoch, j->binding_generation);
	for (u32 e = 0; e < 2; e++) {
		const struct m4_user_engine *w = &j->cmd[index].wire[e];
		struct m4_ready_buffer *q = p->execution->queue[e];
		void *pipe = p->ready->buffer[GPU_M4_OBJECT_COUNT + e * 2].cpu;
		void *work = n->work[e]->map.vaddr;
		void *start = m4_user_render_sequence_cpu(n, e);
		struct gpu_m4_render_stage_retirement s = {
			.notifier_fw = j->queue->notifier->bo->fw_va,
			.uma_descriptor_fw = n->gpu[GPU_M4_RENDER_FIRST_UMA_GPU]->fw_va,
			.uma_slot = j->cmd[index].uma_slot,
			.reservation_sequence = c->engine[e].reservation_sequence,
			.expected_threshold = c->engine[e].threshold_to_publish,
			.threshold = m4_store_read64(notifier + 0x600 + e * 8),
			.expected_counter = c->engine[e].counter,
			.start_counter = m4_store_read64(start + (e ? 0x1d8 : 0x1b8)),
			.expected_target = c->target_stamp,
			.firmware_stamp = m4_ready_read32(notifier + 0x620 + e * 16),
			.user_stamp = p->user_stage_cells ? m4_user_stage_read(p, e) :
				m4_ready_read32(notifier + 0x628 + e * 16),
			.stage_horizon = m4_user_stage_tail(p, e),
			.queue_begin = w->queue_before, .queue_entries = w->count, .queue_next = w->queue_next,
			.queue_read = m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x30),
			.queue_producer = m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x40),
			.queue_busy = m4_user_qi_busy(p, q), .queue_capacity = 256,
			.pipe_begin = w->pipe_before, .pipe_next = w->pipe_next,
			.pipe_read = m4_ready_read32(pipe), .pipe_cfi = m4_ready_read32(pipe + 0x10),
			.pipe_producer = m4_ready_read32(pipe + 0x20), .pipe_capacity = 256,
			.expected_generation = j->binding_generation,
			.generation = *((u8 *)work + (e ? 0xc5f : 0x917)),
			.cache_generation = m4_ready_read32(notifier + e * 0x200 + 8),
		};

		ret = m4_store_packed64(work + 4, &s.work_counter);
		for (u32 i = 0; i < 3; i++)
			s.queue_consumer[i] = m4_user_qi_consumer(p, q, i);
		if (!ret)
			ret = gpu_m4_render_stage_retired(&s, e);
		dev_err(p->hello->gpu->dev,
			"GPU render fault engine=%u scalar=%d counter=%llu/%llu/%llu threshold=%llu/%llu seq=%llu stamps=%#x/%#x/%#x gen=%u/%u cache=%u q=%u/%u/%u busy=%u consumers=%u,%u,%u pipe=%u/%u/%u/%u\n",
			e, ret, (unsigned long long)s.work_counter, (unsigned long long)s.start_counter,
			(unsigned long long)s.expected_counter, (unsigned long long)s.threshold,
			(unsigned long long)s.expected_threshold,
			(unsigned long long)j->queue->notifier->sequence[e], s.firmware_stamp, s.user_stamp, s.expected_target,
			s.generation, s.expected_generation, s.cache_generation,
			s.queue_read, s.queue_producer, s.queue_next, s.queue_busy,
			s.queue_consumer[0], s.queue_consumer[1], s.queue_consumer[2],
			s.pipe_read, s.pipe_cfi, s.pipe_producer, s.pipe_next);
	}
}

#endif
