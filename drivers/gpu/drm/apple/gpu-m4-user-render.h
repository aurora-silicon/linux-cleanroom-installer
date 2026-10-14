/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_USER_RENDER_H
#define APPLE_GPU_M4_USER_RENDER_H

#include "gpu-m4-render-native.h"

/* Included after the actual ordinary runtime, queue/job and native owner.
 * This is packet glue for its existing protected publication entrance, not
 * another scheduler or lifetime. No MMIO, table/ring store, cache operation,
 * allocation or fence transition is performed here.
 */
enum m4_user_render_session_role {
	M4_USER_RENDER_TA_ACTIVITY,
	M4_USER_RENDER_FS_ACTIVITY,
	M4_USER_RENDER_TA_QUEUE,
	M4_USER_RENDER_FS_QUEUE,
	M4_USER_RENDER_PB_DESCRIPTOR,
	M4_USER_RENDER_SESSION_COUNT,
};

struct m4_user_render_reservation {
	struct gpu_m4_render_packet_chronology chronology;
	struct m4_user_native_bo *notifier;
	/* Genuine retained READY/session owners; never disguised as native GEMs. */
	struct gpu_m4_render_native_backing session[M4_USER_RENDER_SESSION_COUNT];
	u32 uma_slot, completion_record[2];
	/* A rebound manager can need InitBuffer without allowing allocator reset. */
	bool initialize_pb;
};

/* Implemented by the root's actual reserved-batch owner. It checks exact
 * &j->render_reservation[index] membership, queue notifier, exclusive UMA/PB
 * and completion records, frozen chronology/credits, and every READY source
 * identity/PA/range. initialize_pb requires new unexposed manager/list/ring
 * owners, or genuinely retired/displaced replacement capacity: InitBuffer
 * alone does NOT grant reset. This is not a configurable approver callback.
 */
static int m4_user_render_reservation_check_locked(struct m4_user_job *j,
	u32 index, const struct m4_user_render_reservation *reservation);

/* Allocated/charged by actual acceptance, before cache protection. Only the
 * private scratch/borrowed input tuple may change on a failed preparation;
 * packet and PB image bytes remain unchanged. The caller copies no image on
 * failure. Large images are heap-owned, never a kernel-stack temporary.
 */
struct m4_user_render_packet {
	struct m4_user_job *job;
	u32 command_index;
	const struct m4_user_render_reservation *reservation;
	const struct m4_user_render_native *native;
	struct gpu_m4_render_native_owner owner;
	struct gpu_m4_render_retained_binding binding;
	struct gpu_m4_render_native_input input;
	struct gpu_m4_render_native_scratch scratch;
	struct gpu_m4_render_packet_output bytes;
	u8 parameter_manager[GPU_M4_RENDER_PARAMETER_MANAGER_SIZE], pb_ring[8];
	u8 *pb_pages, *pb_blocks;
	u32 pb_pages_capacity, pb_blocks_capacity;
	/* Private immutable admission, borrowed through the actual retained job.
	 * Never a protection, health, slot-displacement or retirement grant.
	 */
	struct gpu_m4_drm_provider_job *admitted_job;
	struct gpu_m4_drm_provider_version *admitted_version;
	const struct gpu_m4_drm_vm_owner *admitted_snapshot;
	struct m4_user_native_stage *admitted_stage;
	struct gpu_m4_drm_cache_job *admitted_cache;
	const void *admitted_bytes;
	u64 admitted_epoch;
	u32 admitted_size;
	/* Exact command/native span admission of the same held tuple. */
	struct gpu_m4_render_native_admission span_admission;
};

#define M4_USER_RENDER_BO_BOUND (GPU_M4_RENDER_ENCODED_RESOURCES + \
	GPU_M4_RENDER_FIRST_FW_COUNT + GPU_M4_RENDER_FIRST_GPU_COUNT + 6)

/* Exact native backing plus its actual protected cache/reservation contract.
 * Upper translations are independently checked against the owned pin: a
 * canonical VA or a native tag is not translation/ownership authority.
 * proof is this exact job's selected-inventory proof, or NULL.
 */
static int m4_user_render_bo_locked(struct m4_user_render_packet *packet,
	const struct m4_user_native_history *proof, struct m4_user_native_bo *bo,
	struct m4_user_native_bo **seen, u32 *count)
{
	struct m4_user_job *j = packet->job;
	struct m4_gpu_provider_runtime *p = j->runtime;
	u32 access;
	int ret;

	if (!bo)
		return 0; /* Only the independently checked session placeholders. */
	for (u32 i = 0; i < *count; i++)
		if (seen[i] == bo)
			return 0;
	if (*count == M4_USER_RENDER_BO_BOUND)
		return -E2BIG;
	seen[(*count)++] = bo;
	if (!packet->admitted_job) {
		access = bo->va ? bo->mapping.access : DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE;
		/* A BO admitted by THIS job's selected-inventory proof already passed
		 * the complete native check against this same held version and the
		 * identical whole-BO RW access check: reuse that immutable membership.
		 * The latched errors and the actual job's PUBLISHED/protected phase
		 * are checked live, exactly as m4_user_native_bo_arm_locked().
		 */
		if (proof && bo->access_proof == proof->access_proof &&
		    proof->version == packet->input.version && !bo->backing.offset &&
		    bo->size == bo->coverage && access == M4_USER_NATIVE_RW) {
			m4_user_native_proof_stats.render_reused++;
			ret = p->error ?: p->ready->error ?:
				gpu_m4_drm_provider_job_check_execution_locked(j->core_job,
					p->provider, true);
			if (ret)
				return gpu_m4_render_native_status(ret);
		} else {
			m4_user_native_proof_stats.render_full++;
			ret = m4_user_native_bo_check_locked(bo, p->provider, packet->input.version);
			if (ret)
				return gpu_m4_render_native_status(ret);
			ret = gpu_m4_drm_provider_job_check_access_locked(j->core_job, p->provider,
				bo->object, bo->backing.offset, bo->size, access, false, NULL);
			if (ret)
				return gpu_m4_render_native_status(ret);
		}
	}
	if (!bo->upper)
		return bo->fw_va ? -ESTALE : 0;
	if (gpu_m4_drm_upper_va_locked(bo->upper) != bo->fw_va ||
	    !bo->fw_va || bo->size > ~0ULL - bo->fw_va)
		return -ESTALE;
	/* Admitted packet: the held lease's membership, class, backing identity
	 * and every owned PTE/guard were proved at admission; it is still the
	 * same live (non-garbage) lease at its canonical VA above. Held refs
	 * forbid migration and published PTEs are never rewritten; byte drift of
	 * the live upper graph is the upper owner's per-publication/poll health
	 * (every table byte each call, every lease PTE/guard per revision), run
	 * by READY health in each scope before these callbacks.
	 */
	if (packet->admitted_job)
		return 0;
	/* Exact current published lease: full graph, every selected PTE/guard
	 * and the SAME native pin inventory once, rather than global verification
	 * for each page and a separate guard walk. An unchanged lease reuses the
	 * publication proof its arm-phase publication used (current revision and
	 * root[2], latest live health pass); otherwise its owned path is walked.
	 */
	ret = gpu_m4_drm_upper_check_lease_locked(bo->upper, &bo->backing,
		GPU_M4_DRM_UPPER_NORMAL, m4_user_native_upper_class(bo), bo->fw_va);
	return gpu_m4_render_native_status(ret);
}

static bool m4_user_render_backing_same(const struct gpu_m4_render_native_backing *a,
	const struct gpu_m4_render_native_backing *b)
{
	return a->span.va == b->span.va && a->span.size == b->span.size &&
		gpu_m4_render_native_same_backing(a, b);
}

static int m4_user_render_owner_locked(void *cookie,
	const struct gpu_m4_render_native_input *in)
{
	static const u32 session_fw[] = {
		GPU_M4_RENDER_FIRST_TILER_ACTIVITY_FW, GPU_M4_RENDER_FIRST_FRAGMENT_ACTIVITY_FW,
		GPU_M4_RENDER_FIRST_TILER_QUEUE_FW, GPU_M4_RENDER_FIRST_FRAGMENT_QUEUE_FW,
	};
	struct m4_user_render_packet *packet = cookie;
	struct m4_user_job *j = packet->job;
	struct m4_gpu_provider_runtime *p = j->runtime;
	const struct m4_user_render_reservation *res = packet->reservation;
	const struct m4_user_render_native *n;
	struct gpu_m4_drm_provider_job_view view = { };
	struct gpu_m4_drm_cache_job *cache = NULL;
	struct m4_user_native_bo *seen[M4_USER_RENDER_BO_BOUND];
	const struct m4_user_native_history *proof;
	u32 count = 0, timestamps;
	u64 epoch, low, high;
	u8 wire;
	bool admitted = !!packet->admitted_job;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (in != &packet->input || in->owner != &packet->owner ||
	    in->binding != &packet->binding || in->chronology != &res->chronology ||
	    !j->prepared || j->retired || j->runtime != p || !p->ordinary || p->error ||
	    !j->native_stage || j->queue->runtime != p || in->provider != p->provider ||
	    in->job != j->core_job || in->command_index != packet->command_index)
		return p->error ?: -ESTALE;
	ret = m4_user_lane_check_locked(p, j->core_job);
	if (!ret)
		ret = gpu_m4_drm_provider_job_check_owner_locked(j->core_job, p->provider);
	if (!ret)
		ret = m4_user_binding_check_locked(p);
	if (ret)
		return gpu_m4_render_native_status(ret);
	gpu_m4_drm_provider_job_view(j->core_job, &view);
	if (view.runtime_job != j || view.runtime_queue != j->queue ||
	    view.stage_owner != j->native_stage || !view.batch || !view.batch->commands ||
	    packet->command_index >= view.batch->count || view.version != in->version ||
	    view.snapshot != in->snapshot || view.batch->vm_snapshot != in->snapshot ||
	    view.batch->commands[packet->command_index].type != DRM_ASAHI_CMD_RENDER ||
	    in->command != &view.batch->commands[packet->command_index].render ||
	    j->asid < 1 || j->asid >= M4_USER_ASIDS || in->binding->asid != j->asid ||
	    p->user_asid[j->asid].resident != in->version ||
	    gpu_m4_drm_provider_asid_resident_locked(p->provider, j->asid) != in->version)
		return M4_USER_WHY(-ESTALE, "render owner %u view job=%d version=%d asid=%u binding=%u resident=%d",
			packet->command_index, view.runtime_job == j, view.version == in->version,
			j->asid, in->binding->asid, j->asid && j->asid < M4_USER_ASIDS &&
			p->user_asid[j->asid].resident == in->version);
	if (admitted && (packet->admitted_job != j->core_job ||
	    packet->admitted_version != in->version || packet->admitted_snapshot != in->snapshot ||
	    packet->admitted_stage != j->native_stage || packet->admitted_epoch != j->binding_epoch ||
	    packet->admitted_bytes != view.command_bytes || packet->admitted_size != view.command_size))
		return -ESTALE;
	low = p->user_asid[j->asid].low;
	high = 0;
	ret = admitted ? 0 : gpu_m4_drm_provider_version_roots_locked(in->version, &low, &high);
	if (!ret && !admitted)
		low = m4_provider_ttbat_low(low, j->asid);
	if (!ret)
		ret = gpu_m4_drm_provider_asid_binding_locked(p->provider, j->asid, in->version,
			&epoch, &wire);
	if (ret)
		return M4_USER_WHY(gpu_m4_render_native_status(ret), "render owner %u roots/binding asid=%u",
			packet->command_index, j->asid);
	if (low != p->user_asid[j->asid].low || high ||
	    epoch != j->binding_epoch || wire != j->binding_generation ||
	    in->binding->lease != p->provider || in->binding->version != in->version ||
	    in->binding->epoch != epoch || in->binding->generation != wire)
		return M4_USER_WHY(-ESTALE, "render owner %u asid=%u word %#llx/%#llx epoch %llu/%llu wire %u/%u",
			packet->command_index, j->asid, M4_WHY64(low), M4_WHY64(p->user_asid[j->asid].low),
			M4_WHY64(epoch), M4_WHY64(j->binding_epoch), wire, j->binding_generation);
	ret = m4_user_render_reservation_check_locked(j, packet->command_index, res);
	if (ret)
		return gpu_m4_render_native_status(ret);
	if (!res->notifier)
		return -ESTALE;
	ret = gpu_m4_drm_provider_job_check_access_locked(j->core_job, p->provider,
		res->notifier->object, res->notifier->backing.offset,
		res->notifier->coverage, DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE,
		false, &cache);
	if (ret || !cache || (admitted && cache != packet->admitted_cache))
		return ret ? gpu_m4_render_native_status(ret) : -ESTALE;
	n = m4_user_native_render(j->native_stage, packet->command_index);
	if (!n || n != packet->native || packet->owner.stage_owner != j->native_stage ||
	    packet->owner.vm != j->queue->vm->provider_vm ||
	    packet->owner.uma_slot != res->uma_slot ||
	    packet->owner.render.context != j->asid || packet->owner.render.asid != j->asid ||
	    packet->owner.render.generation != wire ||
	    packet->owner.render.parameter_id != res->chronology.parameter_id ||
	    packet->owner.render.queue_usc_base != j->queue->usc_base ||
	    in->command->usc_base != j->queue->usc_base ||
	    packet->owner.render.parameter_descriptor_fw !=
		res->session[M4_USER_RENDER_PB_DESCRIPTOR].span.va ||
	    packet->owner.render.parameter_manager_fw != n->owner.manager.span.va)
		return M4_USER_WHY(-ESTALE, "render owner %u native=%d slot=%u/%u context=%u/%u asid=%u generation=%u/%u",
			packet->command_index, n && n == packet->native, packet->owner.uma_slot,
			res->uma_slot, packet->owner.render.context, packet->owner.render.asid,
			j->asid, packet->owner.render.generation, wire);
	ret = m4_user_native_render_check_locked(j->native_stage, packet->command_index,
		p->provider, in->version, in->snapshot);
	if (ret)
		return gpu_m4_render_native_status(ret);
	proof = admitted ? NULL : m4_user_native_access_proof_locked(p, j->core_job);
	for (u32 i = 0; i < GPU_M4_RENDER_ENCODED_RESOURCES; i++) {
		if (i != GPU_M4_RENDER_VDM &&
		    !m4_user_render_backing_same(&packet->owner.resource[i], &n->owner.resource[i]))
			return -ESTALE;
		ret = m4_user_render_bo_locked(packet, proof, n->resource[i], seen, &count);
		if (ret)
			return ret;
	}
	for (u32 i = 0; i < GPU_M4_RENDER_FIRST_GPU_COUNT; i++) {
		if (!m4_user_render_backing_same(&packet->owner.gpu[i], &n->owner.gpu[i]))
			return -ESTALE;
		ret = m4_user_render_bo_locked(packet, proof, n->gpu[i], seen, &count);
		if (ret)
			return ret;
	}
	for (u32 i = 0; i < GPU_M4_RENDER_FIRST_FW_COUNT; i++) {
		bool session = false;

		for (u32 s = 0; s < ARRAY_SIZE(session_fw); s++)
			if (i == session_fw[s]) {
				session = true;
				if (!m4_user_render_backing_same(&packet->owner.firmware[i], &res->session[s]))
					return -ESTALE;
			}
		if (!session && i != GPU_M4_RENDER_FIRST_NOTIFIERS_FW &&
		    !m4_user_render_backing_same(&packet->owner.firmware[i], &n->owner.firmware[i]))
			return -ESTALE;
		ret = m4_user_render_bo_locked(packet, proof, n->firmware[i], seen, &count);
		if (ret)
			return ret;
	}
	if (!m4_user_render_backing_same(&packet->owner.parameter_heap, &n->owner.parameter_heap) ||
	    !m4_user_render_backing_same(&packet->owner.manager, &n->owner.manager) ||
	    !m4_user_render_backing_same(&packet->owner.tpc_firmware, &n->owner.tpc_firmware) ||
	    !m4_user_render_backing_same(&packet->owner.init_buffer, &n->owner.init_buffer) ||
	    !m4_user_render_backing_same(&packet->owner.fragment_dependency, &n->owner.fragment_dependency) ||
	    !m4_user_render_backing_same(&packet->owner.descriptor, &res->session[M4_USER_RENDER_PB_DESCRIPTOR]))
		return -ESTALE;
	{
		struct m4_user_native_bo *bos[] = { n->work[0], n->work[1], n->parameter_heap,
			n->manager, n->tpc, res->notifier };

		for (u32 i = 0; i < ARRAY_SIZE(bos); i++) {
			ret = m4_user_render_bo_locked(packet, proof, bos[i], seen, &count);
			if (ret)
				return ret;
		}
	}
	{
		const struct gpu_m4_render_native_backing *b =
			&packet->owner.firmware[GPU_M4_RENDER_FIRST_NOTIFIERS_FW];

		if (!res->notifier || b->kind != GPU_M4_RENDER_NATIVE_CACHE ||
		    b->span.va != res->notifier->fw_va || b->span.size != 0x1000 ||
		    b->cache.object != res->notifier->object || b->cache.upper_lease != res->notifier->upper ||
		    b->cache.offset != res->notifier->backing.offset || b->cache.size != res->notifier->size ||
		    b->cache.firmware_va != res->notifier->fw_va ||
		    b->cache.access != (DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE))
			return -ESTALE;
	}
	ret = gpu_m4_drm_provider_job_timestamp_count_locked(j->core_job, &timestamps);
	if (ret)
		return gpu_m4_render_native_status(ret);
	if (timestamps > GPU_M4_DRM_JOB_MAX_COMMANDS * GPU_M4_DRM_RENDER_TIMESTAMPS)
		return -E2BIG;
	if (timestamps && !j->token)
		return -ESTALE;
	if (j->token) {
		ret = gpu_m4_drm_timestamp_alias_token_check_locked(j->token);
		if (ret)
			return gpu_m4_render_native_status(ret);
	}
	for (u32 i = 0; i < timestamps; i++) {
		struct gpu_m4_drm_provider_job_timestamp_view t;

		ret = gpu_m4_drm_provider_job_timestamp_at_locked(j->core_job, i, &t);
		if (!ret)
			ret = gpu_m4_drm_provider_job_timestamp_check_locked(j->core_job, t.runtime_alias, false);
		if (ret)
			return gpu_m4_render_native_status(ret);
	}
	/* Set only after actual bound/upper/native/cache/TS admission succeeds.
	 * The job, snapshot and native stage remain strongly retained; every later
	 * use above still checks live lane/core/cache/binding/lease/chronology.
	 */
	if (!admitted) {
		packet->admitted_job = j->core_job;
		packet->admitted_version = in->version;
		packet->admitted_snapshot = in->snapshot;
		packet->admitted_stage = j->native_stage;
		packet->admitted_cache = cache;
		packet->admitted_bytes = view.command_bytes;
		packet->admitted_size = view.command_size;
		packet->admitted_epoch = j->binding_epoch;
	}
	return 0;
}

static int m4_user_render_prepare_locked(struct m4_user_job *j, u32 index,
	const struct m4_user_render_reservation *res, struct m4_user_render_packet *out)
{
	static const u32 session_fw[] = {
		GPU_M4_RENDER_FIRST_TILER_ACTIVITY_FW, GPU_M4_RENDER_FIRST_FRAGMENT_ACTIVITY_FW,
		GPU_M4_RENDER_FIRST_TILER_QUEUE_FW, GPU_M4_RENDER_FIRST_FRAGMENT_QUEUE_FW,
	};
	struct gpu_m4_drm_provider_job_view view = { };
	struct m4_gpu_provider_runtime *p;
	const struct m4_user_render_native *n;
	struct gpu_m4_exec_interval ranges[6];
	struct gpu_m4_render_native_input storage = { };
	struct gpu_m4_render_span fw[GPU_M4_RENDER_FIRST_FW_COUNT], gpu[GPU_M4_RENDER_FIRST_GPU_COUNT];
	u32 count = 0;
	int ret;

	if (!j || !j->runtime || !j->queue || !j->core_job || !res || !out)
		return -EINVAL;
	p = j->runtime;
	lockdep_assert_held(&p->hello->gpu->lock);
	ret = m4_user_lane_check_locked(p, j->core_job);
	if (!ret)
		ret = m4_user_render_reservation_check_locked(j, index, res);
	if (ret)
		return gpu_m4_render_native_status(ret);
	if (res->completion_record[0] || res->completion_record[1])
		return -EOPNOTSUPP; /* Existing serialized envelope leases record zero. */
	if (res->initialize_pb && !res->chronology.init_buffer)
		return -EINVAL;
	gpu_m4_drm_provider_job_view(j->core_job, &view);
	if (!view.batch || !view.batch->commands || index >= view.batch->count ||
	    view.batch->commands[index].type != DRM_ASAHI_CMD_RENDER ||
	    view.runtime_job != j || view.runtime_queue != j->queue ||
	    view.stage_owner != j->native_stage || !res->notifier)
		return -ESTALE;
	n = m4_user_native_render(j->native_stage, index);
	if (!n)
		return -ESTALE;
	/* Detached storage may not clobber immutable command/owner/reservation
	 * bytes. Native preparation independently checks the full mixed batch.
	 */
	ranges[count++] = (struct gpu_m4_exec_interval){ (unsigned long)out, sizeof(*out) };
	ranges[count++] = (struct gpu_m4_exec_interval){ (unsigned long)res, sizeof(*res) };
	ranges[count++] = (struct gpu_m4_exec_interval){ (unsigned long)n, sizeof(*n) };
	ranges[count++] = (struct gpu_m4_exec_interval){ (unsigned long)j, sizeof(*j) };
	if (res->initialize_pb) {
		if (!out->pb_pages || !out->pb_blocks || !out->pb_pages_capacity || !out->pb_blocks_capacity)
			return -EINVAL;
		ranges[count++] = (struct gpu_m4_exec_interval){ (unsigned long)out->pb_pages, out->pb_pages_capacity };
		ranges[count++] = (struct gpu_m4_exec_interval){ (unsigned long)out->pb_blocks, out->pb_blocks_capacity };
	}
	if (!gpu_m4_exec_disjoint(ranges, count))
		return -EINVAL;
	storage.provider = p->provider;
	storage.job = j->core_job;
	ret = gpu_m4_render_native_job_storage(&storage, out, sizeof(*out), &view);
	if (!ret && res->initialize_pb)
		ret = gpu_m4_render_native_job_storage(&storage, out->pb_pages,
			out->pb_pages_capacity, &view);
	if (!ret && res->initialize_pb)
		ret = gpu_m4_render_native_job_storage(&storage, out->pb_blocks,
			out->pb_blocks_capacity, &view);
	if (ret)
		return M4_USER_WHY(ret, "render %u job storage", index);
	out->admitted_job = NULL;
	out->span_admission.valid = false;
	out->job = j; out->command_index = index; out->reservation = res; out->native = n;
	out->owner = n->owner;
	out->owner.render.context = out->owner.render.asid = j->asid;
	out->owner.render.generation = j->binding_generation;
	out->owner.render.parameter_id = res->chronology.parameter_id;
	out->owner.render.queue_usc_base = j->queue->usc_base;
	out->owner.render.parameter_descriptor_fw = res->session[M4_USER_RENDER_PB_DESCRIPTOR].span.va;
	out->owner.uma_slot = res->uma_slot;
	out->owner.descriptor = res->session[M4_USER_RENDER_PB_DESCRIPTOR];
	for (u32 i = 0; i < ARRAY_SIZE(session_fw); i++)
		out->owner.firmware[session_fw[i]] = res->session[i];
	out->owner.firmware[GPU_M4_RENDER_FIRST_NOTIFIERS_FW] = (struct gpu_m4_render_native_backing) {
		.span = { res->notifier->fw_va, 0x1000 }, .kind = GPU_M4_RENDER_NATIVE_CACHE,
		.cache = { .object = res->notifier->object, .upper_lease = res->notifier->upper,
			.offset = res->notifier->backing.offset, .size = res->notifier->size,
			.firmware_va = res->notifier->fw_va,
			.access = DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE },
	};
	out->binding = (struct gpu_m4_render_retained_binding) {
		.lease = p->provider, .version = view.version,
		.epoch = j->binding_epoch, .generation = j->binding_generation,
		.asid = j->asid,
	};
	out->input = (struct gpu_m4_render_native_input) {
		.provider = p->provider, .job = j->core_job, .command_index = index,
		.command = &view.batch->commands[index].render,
		.version = view.version, .snapshot = view.snapshot, .owner = &out->owner,
		.binding = &out->binding, .chronology = &res->chronology,
		.uuid = res->chronology.target_stamp, .owner_cookie = out,
		.check_owner = m4_user_render_owner_locked, .admission = &out->span_admission,
	};
	if (READ_ONCE(pb_fit)) {
		struct gpu_m4_exec_geometry geometry;
		struct gpu_m4_render_pb_layout pb;

		if (m4_user_native_render_need(&view.batch->commands[index], &geometry, &pb, j->queue))
			out->input.pb_heap_limit = pb.heap_size;
	}
	ret = gpu_m4_render_native_validate(&out->scratch.state, &out->input);
	if (ret)
		return M4_USER_WHY(ret, "render %u native validate", index);
	if (res->initialize_pb && (out->pb_pages_capacity < out->scratch.state.pb.page_list_size ||
	    out->pb_blocks_capacity < out->scratch.state.pb.block_list_size))
		return -ENOSPC;
	ret = gpu_m4_render_native_packet_prepare(&out->bytes, &out->scratch, &out->input);
	if (ret)
		return M4_USER_WHY(ret, "render %u native packet", index);
	if (res->initialize_pb) {
		for (u32 i = 0; i < GPU_M4_RENDER_FIRST_FW_COUNT; i++)
			fw[i] = out->owner.firmware[i].span;
		for (u32 i = 0; i < GPU_M4_RENDER_FIRST_GPU_COUNT; i++)
			gpu[i] = out->owner.gpu[i].span;
		gpu_m4_render_pb_emit(&out->scratch.state.pb, &out->owner.render,
			out->owner.parameter_heap.span.va, fw, gpu, out->parameter_manager,
			out->pb_pages, out->pb_blocks, out->pb_ring);
	}
	return 0;
}

#endif
