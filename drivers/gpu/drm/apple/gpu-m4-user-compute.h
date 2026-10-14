/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_USER_COMPUTE_H
#define APPLE_GPU_M4_USER_COMPUTE_H

#include "gpu-m4-compute-packet.h"
#include "gpu-m4-compute-reuse.h"
#include "gpu-m4-compute-timestamps.h"

struct m4_user_compute_native;

/* Immutable job-owned reservation, not caller-selected firmware addresses.
 * The root checks its exact array membership and actual retained physical
 * READY queue/stats, logical notifier, completion record, slot, prior edge and
 * all producer/consumer/chronology leases. No threshold is exposed here.
 */
struct m4_user_compute_reservation {
	struct gpu_m4_compute_packet_chronology chronology;
	struct gpu_m4_compute_packet_dependency prior;
	struct m4_user_native_bo *notifier;
	u64 queue_fw, stats_fw, timestamp;
	u32 completion_record;
	const struct gpu_m4_compute_reuse_input *reuse;
	const struct gpu_m4_compute_reuse_binding *reuse_binding;
};

static int m4_user_compute_reservation_check_locked(struct m4_user_job *j,
	u32 index, const struct m4_user_compute_reservation *reservation);

/* Root acceptance allocates/charges these detached bytes BEFORE protection.
 * Only context/scratch may change on failure; bytes and owned plans do not.
 * This persistent context (never a stack cookie) survives all TS request refs.
 */
struct m4_user_compute_packet {
	struct m4_user_job *job;
	u32 command_index;
	const struct m4_user_compute_reservation *reservation;
	const struct m4_user_compute_native *native;
	struct gpu_m4_exec_vm vm;
	struct gpu_m4_compute_packet_admitted_input input;
	struct gpu_m4_compute_packet_output bytes, base, scratch;
	u8 work[GPU_M4_EXEC_COMPUTE_SIZE], ts_scratch[GPU_M4_EXEC_COMPUTE_SIZE];
	struct gpu_m4_compute_reuse_plan reuse_plan;
	struct gpu_m4_compute_ts_plan timestamps;
};

static int m4_user_compute_check_locked(struct m4_user_compute_packet *out);

static int m4_user_compute_status(int ret)
{
	return ret > 0 ? -EPROTO : ret;
}

static int m4_user_compute_ts_hold(void *cookie,
	const struct gpu_m4_compute_ts_destination *destination, void **held)
{
	struct m4_user_compute_packet *out = cookie;
	struct m4_user_job *j = out->job;
	struct m4_gpu_provider_runtime *p = j->runtime;
	u32 count;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	ret = gpu_m4_drm_provider_job_timestamp_count_locked(j->core_job, &count);
	if (ret)
		return m4_user_compute_status(ret);
	for (u32 i = 0; i < count; i++) {
		struct gpu_m4_drm_provider_job_timestamp_view t;

		ret = gpu_m4_drm_provider_job_timestamp_at_locked(j->core_job, i, &t);
		if (ret)
			return m4_user_compute_status(ret);
		if (t.command_index != out->command_index || t.object != destination->object ||
		    t.runtime_alias != destination->lease || t.object_offset != destination->object_offset ||
		    t.firmware_va != destination->firmware_va)
			continue;
		/* t.object is the actual current provider pin, NOT t.gem. Retain the
		 * SAME captured request, without creating/rebinding another alias.
		 */
		ret = gpu_m4_drm_provider_job_timestamp_check_locked(j->core_job, t.runtime_alias, false);
		if (!ret)
			ret = gpu_m4_drm_timestamp_alias_request_get_locked(p->ts_aliases, t.runtime_alias);
		if (ret)
			return M4_USER_WHY(m4_user_compute_status(ret), "ts hold command %u timestamp %u/%u",
				out->command_index, i, count);
		*held = t.runtime_alias;
		return 0;
	}
	return M4_USER_WHY(-EPERM, "ts hold command %u destination %#llx+%#llx not found among %u",
		out->command_index, M4_WHY64(destination->firmware_va),
		M4_WHY64(destination->object_offset), count);
}

static void m4_user_compute_ts_put(void *cookie, void *held)
{
	struct m4_user_compute_packet *out = cookie;

	lockdep_assert_held(&out->job->runtime->hello->gpu->lock);
	gpu_m4_drm_timestamp_alias_request_put_locked(held);
}

static int m4_user_compute_ts_work_check(void *cookie,
	const struct gpu_m4_compute_ts_input *input, const void *output, const void *scratch)
{
	struct m4_user_compute_packet *out = cookie;

	if (input->work != out->work || input->work_fw != out->input.work.work_fw ||
	    input->queue_fw != out->reservation->queue_fw || output != out->scratch.work ||
	    scratch != out->ts_scratch)
		return M4_USER_WHY(-ESTALE, "ts work %u work=%d fw=%d queue=%d output=%d scratch=%d",
			out->command_index, input->work == out->work,
			input->work_fw == out->input.work.work_fw,
			input->queue_fw == out->reservation->queue_fw, output == out->scratch.work,
			scratch == out->ts_scratch);
	return m4_user_compute_check_locked(out);
}

static const struct gpu_m4_compute_ts_ops m4_user_compute_ts_ops = {
	.check_work = m4_user_compute_ts_work_check,
	.hold = m4_user_compute_ts_hold,
	.put = m4_user_compute_ts_put,
};

/* Reuse a compute range proof while the version's tree/graph is unchanged
 * for that range: the same span cache (kind 4, range) render attachment and
 * backing checks use. Every publish re-proves the command's seven private
 * regions, whose 32 MiB USC pool alone is a 2048-page tree walk (a third
 * of a trivial compute's ~30 us publication). 0: walk every time.
 */
static bool compute_range_memo = true;
#ifdef MODULE
module_param(compute_range_memo, bool, 0644);
MODULE_PARM_DESC(compute_range_memo, "Reuse compute private/stream range proofs while the VM range is unchanged (default 1)");
#endif

static int m4_user_compute_range(void *cookie, u64 va, u64 size, bool write)
{
	struct m4_user_compute_packet *out = cookie;
	struct gpu_m4_drm_uat_span_key key = {
		.kind = GPU_M4_RENDER_NATIVE_SPAN_RANGE, .va = va, .size = size,
		.access = DRM_ASAHI_BIND_READ | (write ? DRM_ASAHI_BIND_WRITE : 0),
	};
	bool memo = READ_ONCE(compute_range_memo);
	int ret;

	if (memo && gpu_m4_drm_provider_version_span_cached_locked(out->input.version, &key))
		return 0;
	ret = gpu_m4_drm_provider_version_range_locked(out->input.version, va, size, key.access);
	if (!ret && memo)
		gpu_m4_drm_provider_version_span_remember_locked(out->input.version, &key);
	return M4_USER_WHY(m4_user_compute_status(ret), "compute %u range %#llx+%#llx write=%d",
		out->command_index, M4_WHY64(va), M4_WHY64(size), write);
}

/* REMOVE-DEBUG: after a refused admitted validation/encode, re-evaluate each
 * predicate of the pure compute packet/exec grammar (gpu-m4-compute-packet.h
 * admitted validation and encode, gpu-m4-exec.h first-work/register/
 * attachment/descriptor helpers) and record every failing one with its
 * observed and admitted values. Diagnostics only: the pure helpers' own result
 * is returned unchanged by the caller and nothing here grants admission.
 */
#define M4_USER_WHY_ENCODE(cond, fmt, ...) do {					\
	if (cond)								\
		(void)M4_USER_WHY(-EINVAL, "compute %u encode: " fmt, index, ##__VA_ARGS__); \
} while (0)

static void m4_user_why_compute_encode(const struct gpu_m4_compute_packet_admitted_input *in,
	u32 index, bool fresh)
{
	static const u64 minimum[GPU_M4_COMPUTE_PRIVATE_COUNT] = {
		0x4000, 0x4000, 0x4000, 0x40000, 0x20000, 0x4000, GPU_M4_EXEC_POOL_SIZE,
	};
	const struct gpu_m4_exec_compute_input *w = &in->work;
	const struct gpu_m4_exec_compute_resources *r = &w->registers;
	const struct gpu_m4_compute_packet_chronology *c = &in->chronology;
	struct gpu_m4_exec_interval client[GPU_M4_COMPUTE_PRIVATE_COUNT + 1];
	u32 next;
	int ret;

	M4_USER_WHY_ENCODE(!gpu_m4_exec_cdm_span_valid(r->cdm_base, r->cdm_end),
		"cdm %#llx..%#llx length %#llx (admitted: aligned, nonempty, at most %#llx)",
		M4_WHY64(r->cdm_base), M4_WHY64(r->cdm_end), M4_WHY64(r->cdm_end - r->cdm_base),
		M4_WHY64(GPU_M4_EXEC_CDM_SPAN_MAX));
	M4_USER_WHY_ENCODE(!r->usc_base || (r->usc_base & 0xffffffffULL) ||
		r->usc_base < GPU_M4_EXEC_VM_START || r->usc_base >= GPU_M4_EXEC_VM_END ||
		0x100000000ULL > GPU_M4_EXEC_VM_END - r->usc_base,
		"usc base %#llx", M4_WHY64(r->usc_base));
	M4_USER_WHY_ENCODE(r->preemption_size != 0x4000 || (r->preemption & 7),
		"preemption %#llx+%#llx", M4_WHY64(r->preemption), M4_WHY64(r->preemption_size));
	M4_USER_WHY_ENCODE(w->event_sequence || w->binding_generation || !in->binding_generation ||
		c->binding_generation != gpu_m4_drm_provider_binding_wire_generation(in->binding_generation),
		"generation work %u/%u wide %llu wire %llu",
		w->event_sequence, w->binding_generation, M4_WHY64(in->binding_generation),
		M4_WHY64(c->binding_generation));
	M4_USER_WHY_ENCODE(!gpu_m4_exec_fw_range(in->notifier_page_fw, GPU_M4_EXEC_NOTIFY_SIZE, 8) ||
		w->notifier_fw != in->notifier_page_fw + 0x400 ||
		w->firmware_stamp_fw != in->notifier_page_fw + 0x640 ||
		w->user_stamp_fw != in->notifier_page_fw + 0x648,
		"notifier page %#llx record %#llx stamps %#llx,%#llx", M4_WHY64(in->notifier_page_fw),
		M4_WHY64(w->notifier_fw), M4_WHY64(w->firmware_stamp_fw), M4_WHY64(w->user_stamp_fw));
	M4_USER_WHY_ENCODE((in->counter_offset & 0x3fff) || in->counter_offset > ~0ULL - 0x4000,
		"counter offset %#llx", M4_WHY64(in->counter_offset));
	M4_USER_WHY_ENCODE(w->attachment_count > GPU_M4_EXEC_ATTACHMENT_MAX ||
		(w->attachment_count && !w->attachments), "attachments %u", w->attachment_count);
	for (u32 i = 0; w->attachments && i < w->attachment_count && i < GPU_M4_EXEC_ATTACHMENT_MAX; i++) {
		const struct gpu_m4_exec_attachment *a = &w->attachments[i];

		M4_USER_WHY_ENCODE((a->va & 127) || (a->size & 127) || !a->size ||
			a->size > ~0ULL - a->va || a->va < GPU_M4_EXEC_VM_START ||
			a->va + a->size > GPU_M4_EXEC_VM_END,
			"attachment %u %#llx+%#llx", i, M4_WHY64(a->va), M4_WHY64(a->size));
	}
	for (u32 i = 0; i < GPU_M4_COMPUTE_PRIVATE_COUNT; i++) {
		const struct gpu_m4_compute_packet_private *m = &in->private[i];

		client[i] = m->client;
		M4_USER_WHY_ENCODE(!m->object || !m->lease || m->client.size != minimum[i] ||
			(m->client.va & 0x3fff) || m->client.va < 0x7000000000ULL ||
			m->client.va >= 0x8000000000ULL ||
			m->client.size > 0x8000000000ULL - m->client.va ||
			(m->object_offset & 0x3fff) || m->object_offset > ~0ULL - m->client.size ||
			m->access != GPU_M4_COMPUTE_PRIVATE_RW,
			"private %u %#llx+%#llx want size %#llx offset %#llx access %u object=%d lease=%d",
			i, M4_WHY64(m->client.va), M4_WHY64(m->client.size), M4_WHY64(minimum[i]),
			M4_WHY64(m->object_offset), m->access, !!m->object, !!m->lease);
	}
	client[GPU_M4_COMPUTE_PRIVATE_COUNT] = (struct gpu_m4_exec_interval) {
		r->cdm_base, r->cdm_end - r->cdm_base,
	};
	M4_USER_WHY_ENCODE(!gpu_m4_exec_disjoint(client, GPU_M4_COMPUTE_PRIVATE_COUNT + 1),
		"private/cdm client ranges overlap (cdm %#llx+%#llx)", M4_WHY64(r->cdm_base),
		M4_WHY64(r->cdm_end - r->cdm_base));
	for (u32 i = 0; i < 3; i++)
		M4_USER_WHY_ENCODE(in->firmware[i].size != 0x4000 ||
			!gpu_m4_exec_fw_range(in->firmware[i].va, 0x4000, 8),
			"firmware page %u %#llx+%#llx", i, M4_WHY64(in->firmware[i].va),
			M4_WHY64(in->firmware[i].size));
	M4_USER_WHY_ENCODE(in->firmware[0].va != w->work_fw || in->firmware[1].va != w->auxiliary_fw ||
		in->firmware[2].va != w->counter_fw || !gpu_m4_exec_fw_range(w->queue_fw, GPU_M4_EXEC_QUEUE_SIZE, 8) ||
		!gpu_m4_exec_fw_range(w->stats_fw, 0xe10, 8),
		"firmware aliases work %#llx aux %#llx counter %#llx queue %#llx stats %#llx",
		M4_WHY64(w->work_fw), M4_WHY64(w->auxiliary_fw), M4_WHY64(w->counter_fw),
		M4_WHY64(w->queue_fw), M4_WHY64(w->stats_fw));
	ret = fresh ? gpu_m4_compute_packet_chronology_check(c, &in->prior, in->job, in->timestamp) :
		gpu_m4_compute_packet_chronology_structure_check(c, &in->prior, in->job, in->timestamp);
	if (ret)
		(void)M4_USER_WHY(ret, "compute %u encode: chronology fresh=%d seq=%llu thr=%llu/%llu submitted=%llu/%llu/%llu prev=%#x selected=%#x target=%#x slot=%u outstanding=%u queue r=%u w=%u c=%u,%u,%u pipe %u,%u,%u prior=%#x/%u/%d",
			index, fresh, M4_WHY64(c->event_sequence), M4_WHY64(c->threshold_before),
			M4_WHY64(c->threshold_to_publish), M4_WHY64(c->submitted_before),
			M4_WHY64(c->completed_before), M4_WHY64(c->submitted), c->global_previous_target,
			c->selected_stamp_before, c->target_stamp, c->uma_slot, c->outstanding,
			c->queue_read, c->queue_write, c->queue_consumer[0], c->queue_consumer[1],
			c->queue_consumer[2], c->pipe_read, c->pipe_cfi, c->pipe_write, in->prior.target,
			in->prior.event, in->prior.strict);
	ret = gpu_m4_compute_packet_ring_reserve(c, gpu_m4_compute_packet_has_prior(&in->prior) ? 2 : 1,
		&next);
	if (!ret)
		ret = gpu_m4_exec_pipe_reserve(c->pipe_read, c->pipe_cfi, c->pipe_write, &next);
	if (ret)
		(void)M4_USER_WHY(ret, "compute %u encode: ring/pipe reserve", index);
}

/* REMOVE-DEBUG: after a refused warm restoration plan
 * (gpu_m4_compute_reuse_prepare_checked), record the retirement observations
 * of A and C, A's preserved descriptor words and the rebuilt work's qualified
 * fields against what the plan requires. Diagnostics only.
 */
static void m4_user_why_compute_reuse(const struct gpu_m4_compute_reuse_input *in,
	u8 wire, u32 index, int error)
{
	const struct gpu_m4_compute_reuse_owner *a = in->owner, *b = in->replacement;
	const u8 *d = (const u8 *)a->work + 0xe00, *fresh = in->rebuilt_work;
	int ra = gpu_m4_compute_reuse_retired_check(a, true);
	int rb = gpu_m4_compute_reuse_retired_check(b, true);

	(void)M4_USER_WHY_ANY(error, "compute %u reuse plan: retired A=%d C=%d slots %u/%u/%u submitted A=%llu C=%llu uma A=%llu/%llu C=%llu/%llu",
		index, ra, rb, a->uma_slot, b->uma_slot, in->destination_slot, M4_WHY64(a->submitted),
		M4_WHY64(b->submitted), M4_WHY64(a->retirement.uma_submitted),
		M4_WHY64(a->retirement.uma_completed), M4_WHY64(b->retirement.uma_submitted),
		M4_WHY64(b->retirement.uma_completed));
	(void)M4_USER_WHY_ANY(error, "compute %u reuse plan: A descriptor owner %#llx/%#llx slot %u/%u pool %#llx/%#llx list %#llx/%#llx counter %#llx/%#llx completed %llu/%llu",
		index, M4_WHY64(get_unaligned_le64(d)), M4_WHY64(a->work_fw), get_unaligned_le32(d + 8),
		a->uma_slot, M4_WHY64(get_unaligned_le64(d + 0x14)), M4_WHY64(a->pool_state_gpu),
		M4_WHY64(get_unaligned_le64(d + 0x30)), M4_WHY64(a->page_list_gpu),
		M4_WHY64(get_unaligned_le64(d + 0x4c)), M4_WHY64(a->counter_fw),
		M4_WHY64(get_unaligned_le64(d + 0x54)), M4_WHY64(a->submitted));
	(void)M4_USER_WHY_ANY(error, "compute %u reuse plan: rebuilt kind %u asid %u/%u wire %u/%u uma %#llx seq %#llx/%u header %#llx event %u/%u stamp %#x/%#x",
		index, get_unaligned_le32(fresh), get_unaligned_le32(fresh + 0xc), a->asid, fresh[0x88b],
		wire, M4_WHY64(get_unaligned_le64(fresh + 0x86a)), M4_WHY64(get_unaligned_le64(fresh + 0x760)),
		get_unaligned_le32(fresh + 0x768), M4_WHY64(get_unaligned_le64(fresh + 0xa60)),
		get_unaligned_le32(fresh + 0x824), get_unaligned_le32(fresh + 0x938),
		get_unaligned_le32(fresh + 0x810), get_unaligned_le32(fresh + 0xb8c));
}
#undef M4_USER_WHY_ENCODE

/* No BO payload is read. Compare the actual pin oracle to every published
 * upper page and its guard; a canonical address alone is not a translation.
 */
static bool compute_walk_memo = true;
#ifdef MODULE
module_param(compute_walk_memo, bool, 0644);
MODULE_PARM_DESC(compute_walk_memo, "Reuse a compute BO's firmware alias walk at an unchanged upper revision (default 1)");
#endif

static int m4_user_compute_bo_locked(struct m4_user_compute_packet *out,
	struct m4_user_native_bo *bo, u64 coverage, u64 expected_attrs)
{
	struct m4_gpu_provider_runtime *p = out->job->runtime;
	u64 pa, translated, attrs, revision;
	int ret;

	ret = m4_user_native_bo_check_locked(bo, p->provider, out->input.version);
	if (ret)
		return M4_USER_WHY(m4_user_compute_status(ret), "compute %u bo check size=%#llx",
			out->command_index, M4_WHY64(bo ? bo->size : 0));
	if (!coverage || coverage > bo->size || bo->coverage != coverage)
		return M4_USER_WHY(-ESTALE, "compute %u bo coverage=%#llx want %#llx size=%#llx",
			out->command_index, M4_WHY64(bo->coverage), M4_WHY64(coverage), M4_WHY64(bo->size));
	ret = gpu_m4_drm_provider_job_check_access_locked(out->job->core_job, p->provider,
		bo->object, bo->backing.offset, coverage,
		DRM_ASAHI_BIND_READ | DRM_ASAHI_BIND_WRITE, false, NULL);
	if (ret)
		return M4_USER_WHY(m4_user_compute_status(ret), "compute %u bo access size=%#llx",
			out->command_index, M4_WHY64(coverage));
	if (!expected_attrs)
		return bo->upper || bo->fw_va ? M4_USER_WHY(-ESTALE, "compute %u bo unexpected alias %#llx",
			out->command_index, M4_WHY64(bo->fw_va)) : 0;
	if (!bo->upper || !bo->fw_va || bo->size > ~0ULL - bo->fw_va ||
	    gpu_m4_drm_upper_va_locked(bo->upper) != bo->fw_va)
		return M4_USER_WHY(-ESTALE, "compute %u bo alias upper=%d fw=%#llx lease=%#llx",
			out->command_index, !!bo->upper, M4_WHY64(bo->fw_va),
			M4_WHY64(bo->upper ? gpu_m4_drm_upper_va_locked(bo->upper) : 0));
	revision = gpu_m4_drm_upper_revision_locked(p->ready->upper);
	if (revision && bo->walk_revision == revision && bo->walk_attrs == expected_attrs &&
	    compute_walk_memo) {
		/* Unchanged owner since this exact alias was proven: only the
		 * object's live gate.
		 */
		ret = gpu_m4_drm_provider_object_page_locked(bo->object, bo->backing.offset, &pa);
		return ret ? M4_USER_WHY(m4_user_compute_status(ret), "compute %u bo gate %#llx",
			out->command_index, M4_WHY64(bo->fw_va)) : 0;
	}
	bo->walk_revision = 0;
	for (u64 offset = 0; offset < bo->size; offset += GPU_M4_DRM_PAGE_SIZE) {
		ret = gpu_m4_drm_provider_object_page_locked(bo->object,
			bo->backing.offset + offset, &pa);
		if (!ret)
			ret = gpu_m4_drm_upper_walk_locked(p->ready->upper,
				bo->fw_va + offset, &translated, &attrs);
		if (ret)
			return M4_USER_WHY(m4_user_compute_status(ret), "compute %u bo walk %#llx",
				out->command_index, M4_WHY64(bo->fw_va + offset));
		if (pa != translated || attrs != expected_attrs)
			return M4_USER_WHY(-ESTALE, "compute %u bo %#llx pa=%#llx/%#llx attrs=%#llx/%#llx",
				out->command_index, M4_WHY64(bo->fw_va + offset), M4_WHY64(translated),
				M4_WHY64(pa), M4_WHY64(attrs), M4_WHY64(expected_attrs));
	}
	ret = gpu_m4_drm_upper_walk_locked(p->ready->upper,
		bo->fw_va + bo->size, &translated, &attrs);
	if (ret == -ENOENT) {
		bo->walk_revision = revision;
		bo->walk_attrs = expected_attrs;
	}
	return ret == -ENOENT ? 0 : M4_USER_WHY(ret ? m4_user_compute_status(ret) : -ESTALE,
		"compute %u bo guard %#llx walk=%d", out->command_index,
		M4_WHY64(bo->fw_va + bo->size), ret);
}

static bool m4_user_compute_chronology_same(const struct gpu_m4_compute_packet_chronology *a,
	const struct gpu_m4_compute_packet_chronology *b)
{
#define M4_COMPUTE_SAME(field) (a->field == b->field)
	return M4_COMPUTE_SAME(binding_generation) && M4_COMPUTE_SAME(asid) &&
		M4_COMPUTE_SAME(event_sequence) &&
		M4_COMPUTE_SAME(threshold_before) && M4_COMPUTE_SAME(threshold_to_publish) &&
		M4_COMPUTE_SAME(submitted_before) && M4_COMPUTE_SAME(completed_before) &&
		M4_COMPUTE_SAME(submitted) && M4_COMPUTE_SAME(global_previous_target) &&
		M4_COMPUTE_SAME(selected_stamp_before) && M4_COMPUTE_SAME(target_stamp) &&
		M4_COMPUTE_SAME(uma_slot) && M4_COMPUTE_SAME(queue_read) && M4_COMPUTE_SAME(queue_write) &&
		M4_COMPUTE_SAME(queue_consumer[0]) && M4_COMPUTE_SAME(queue_consumer[1]) &&
		M4_COMPUTE_SAME(queue_consumer[2]) && M4_COMPUTE_SAME(pipe_read) &&
		M4_COMPUTE_SAME(pipe_cfi) && M4_COMPUTE_SAME(pipe_write) &&
		M4_COMPUTE_SAME(outstanding) && M4_COMPUTE_SAME(new_queue);
#undef M4_COMPUTE_SAME
}

static int m4_user_compute_check_locked(struct m4_user_compute_packet *out)
{
	struct m4_user_job *j = out->job;
	struct m4_gpu_provider_runtime *p = j->runtime;
	const struct m4_user_compute_reservation *res = out->reservation;
	const struct gpu_m4_compute_packet_admitted_input *in = &out->input;
	const struct m4_user_compute_native *n;
	struct gpu_m4_drm_provider_job_view view = { };
	u64 epoch, low, high;
	u32 timestamps;
	u8 wire;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!p->ordinary || p->error || !j->prepared || j->retired ||
	    !j->native_stage || j->queue->runtime != p || in->job != j->core_job ||
	    in->vm != j->queue->vm->provider_vm || in->work.vm != &out->vm ||
	    out->vm.cookie != out || out->vm.check != m4_user_compute_range ||
	    out->vm.start != GPU_M4_EXEC_VM_START || out->vm.end != GPU_M4_EXEC_VM_END)
		return M4_USER_WHY(p->error ?: -ESTALE, "compute check %u ordinary=%d error=%d prepared=%d retired=%d stage=%d job=%d vm=%d",
			out->command_index, p->ordinary, p->error, j->prepared, j->retired,
			!!j->native_stage, in->job == j->core_job,
			in->vm == j->queue->vm->provider_vm && in->work.vm == &out->vm);
	ret = m4_user_lane_check_locked(p, j->core_job);
	if (!ret)
		ret = gpu_m4_drm_provider_job_check_owner_locked(j->core_job, p->provider);
	if (!ret)
		ret = m4_user_binding_check_locked(p);
	if (!ret)
		ret = j->asid && j->asid < M4_USER_ASIDS ? m4_user_asid_health_locked(p, j->asid) :
			-ESTALE;
	if (ret)
		return M4_USER_WHY(m4_user_compute_status(ret), "compute check %u lane/owner/health",
			out->command_index);
	gpu_m4_drm_provider_job_view(j->core_job, &view);
	if (view.runtime_job != j || view.runtime_queue != j->queue ||
	    view.stage_owner != j->native_stage || !view.batch || !view.batch->commands ||
	    !view.batch->count || view.batch->count > GPU_M4_DRM_JOB_MAX_COMMANDS ||
	    out->command_index >= view.batch->count || in->version != view.version ||
	    in->snapshot != view.snapshot || view.batch->vm_snapshot != view.snapshot ||
	    view.batch->commands[out->command_index].type != DRM_ASAHI_CMD_COMPUTE ||
	    in->command != &view.batch->commands[out->command_index].compute ||
	    p->user_asid[j->asid].resident != in->version ||
	    gpu_m4_drm_provider_asid_resident_locked(p->provider, j->asid) != in->version)
		return M4_USER_WHY(-ESTALE, "compute check %u view job=%d queue=%d stage=%d batch=%u version=%d snapshot=%d type=%u command=%d resident=%d/%d",
			out->command_index, view.runtime_job == j, view.runtime_queue == j->queue,
			view.stage_owner == j->native_stage, view.batch ? view.batch->count : 0,
			in->version == view.version, in->snapshot == view.snapshot &&
			view.batch && view.batch->vm_snapshot == view.snapshot,
			view.batch && view.batch->commands && out->command_index < view.batch->count ?
			view.batch->commands[out->command_index].type : 0xffff,
			view.batch && view.batch->commands && out->command_index < view.batch->count &&
			in->command == &view.batch->commands[out->command_index].compute,
			p->user_asid[j->asid].resident == in->version,
			gpu_m4_drm_provider_asid_resident_locked(p->provider, j->asid) == in->version);
	ret = gpu_m4_drm_provider_version_roots_locked(in->version, &low, &high);
	low = m4_provider_ttbat_low(low, j->asid);
	if (!ret)
		ret = gpu_m4_drm_provider_asid_binding_locked(p->provider, j->asid, in->version,
			&epoch, &wire);
	if (ret)
		return M4_USER_WHY(m4_user_compute_status(ret), "compute check %u roots/binding",
			out->command_index);
	if (low != p->user_asid[j->asid].low || high || epoch != j->binding_epoch ||
	    wire != j->binding_generation || in->binding_generation != epoch ||
	    in->chronology.binding_generation != wire || in->chronology.asid != j->asid ||
	    !m4_user_compute_chronology_same(&in->chronology, &res->chronology) ||
	    in->prior.owner != res->prior.owner || in->prior.firmware.va != res->prior.firmware.va ||
	    in->prior.firmware.size != res->prior.firmware.size || in->prior.stamp_fw != res->prior.stamp_fw ||
	    in->prior.target != res->prior.target || in->prior.event != res->prior.event ||
	    in->prior.strict != res->prior.strict || in->prior.relaxed != res->prior.relaxed)
		return M4_USER_WHY(-ESTALE, "compute check %u roots %#llx,%#llx/%#llx,%#llx epoch %llu/%llu/%llu wire %u/%u/%llu chronology=%d prior=%d",
			out->command_index, M4_WHY64(low), M4_WHY64(high),
			M4_WHY64(p->user_asid[j->asid].low), 0ULL, M4_WHY64(epoch), M4_WHY64(j->binding_epoch),
			M4_WHY64(in->binding_generation), wire, j->binding_generation,
			M4_WHY64(in->chronology.binding_generation),
			m4_user_compute_chronology_same(&in->chronology, &res->chronology),
			in->prior.owner == res->prior.owner && in->prior.firmware.va == res->prior.firmware.va &&
			in->prior.target == res->prior.target && in->prior.strict == res->prior.strict);
	ret = m4_user_compute_reservation_check_locked(j, out->command_index, res);
	if (ret)
		return M4_USER_WHY(m4_user_compute_status(ret), "compute check %u reservation",
			out->command_index);
	if (res->completion_record || (in->command->flags & ~DRM_ASAHI_COMPUTE_WIDE_VISIBILITY))
		return M4_USER_WHY(-EOPNOTSUPP, "compute check %u record=%u flags=%#x", out->command_index,
			res->completion_record, in->command->flags);
	n = m4_user_native_compute(j->native_stage, out->command_index);
	if (!n || !n->counter || out->native != n || !res->notifier ||
	    in->counter != n->counter->object || in->counter_offset != n->counter->backing.offset ||
	    in->notifier_page_fw != res->notifier->fw_va ||
	    in->timestamp != res->timestamp || in->work.queue_fw != res->queue_fw ||
	    in->work.stats_fw != res->stats_fw ||
	    in->work.registers.cdm_base != in->command->cdm_base ||
	    in->work.registers.cdm_end != in->command->cdm_end ||
	    in->work.registers.usc_base != j->queue->usc_base ||
	    in->command->usc_base != j->queue->usc_base ||
	    in->work.attachments != in->command->attachments ||
	    in->work.attachment_count != in->command->attachment_count)
		return M4_USER_WHY(-ESTALE, "compute check %u native=%d counter=%d notifier=%d/%d timestamp=%d queue=%d stats=%d registers=%d attachments=%d",
			out->command_index, n && out->native == n, n && n->counter &&
			in->counter == n->counter->object && in->counter_offset == n->counter->backing.offset,
			!!res->notifier, res->notifier && in->notifier_page_fw == res->notifier->fw_va,
			in->timestamp == res->timestamp, in->work.queue_fw == res->queue_fw,
			in->work.stats_fw == res->stats_fw,
			in->work.registers.cdm_base == in->command->cdm_base &&
			in->work.registers.cdm_end == in->command->cdm_end &&
			in->work.registers.usc_base == j->queue->usc_base &&
			in->command->usc_base == j->queue->usc_base,
			in->work.attachments == in->command->attachments &&
			in->work.attachment_count == in->command->attachment_count);
	ret = m4_user_native_compute_check_locked(j->native_stage, out->command_index,
		p->provider, in->version, in->snapshot);
	if (ret)
		return M4_USER_WHY(m4_user_compute_status(ret), "compute check %u native",
			out->command_index);
	for (u32 i = 0; i < GPU_M4_COMPUTE_PRIVATE_COUNT; i++) {
		struct m4_user_native_bo *bo = n->private[i];
		const struct gpu_m4_compute_packet_private *m = &in->private[i];
		u64 attrs = i == GPU_M4_COMPUTE_PRIVATE_WORK && bo && bo->cell ?
			m4_arena_cell_attrs(bo->cell->arena) :
			i == GPU_M4_COMPUTE_PRIVATE_WORK || i == GPU_M4_COMPUTE_PRIVATE_AUX ?
			GPU_M4_DRM_UPPER_SHARED_ATTRS : 0;

		if (!bo || m->object != bo->object || m->lease != bo->claim ||
		    m->object_offset != bo->mapping.offset || m->client.va != bo->va ||
		    m->client.size != bo->size || m->access != bo->mapping.access ||
		    bo->mapping.object != bo->object || bo->mapping.lease != bo->claim ||
		    bo->mapping.va != bo->va || bo->mapping.size != bo->size ||
		    bo->mapping.offset != bo->backing.offset)
			return M4_USER_WHY(-ESTALE, "compute check %u private %u bo=%d object=%d lease=%d va=%#llx/%#llx size=%#llx/%#llx",
				out->command_index, i, !!bo, bo && m->object == bo->object,
				bo && m->lease == bo->claim, M4_WHY64(m->client.va),
				M4_WHY64(bo ? bo->va : 0), M4_WHY64(m->client.size),
				M4_WHY64(bo ? bo->size : 0));
		ret = m4_user_compute_bo_locked(out, bo, bo->size, attrs);
		if (ret)
			return ret;
	}
	if (in->work.work_fw != n->private[GPU_M4_COMPUTE_PRIVATE_WORK]->fw_va ||
	    in->work.work_gpu != n->private[GPU_M4_COMPUTE_PRIVATE_WORK]->va ||
	    in->work.registers.preemption != n->private[GPU_M4_COMPUTE_PRIVATE_PREEMPT]->va ||
	    in->work.registers.preemption_size != n->private[GPU_M4_COMPUTE_PRIVATE_PREEMPT]->size ||
	    in->work.auxiliary_fw != n->private[GPU_M4_COMPUTE_PRIVATE_AUX]->fw_va ||
	    in->work.auxiliary_gpu != n->private[GPU_M4_COMPUTE_PRIVATE_AUX]->va ||
	    in->work.context_store_gpu != n->private[GPU_M4_COMPUTE_PRIVATE_CONTEXT]->va ||
	    in->work.pool_state_gpu != n->private[GPU_M4_COMPUTE_PRIVATE_POOL_STATE]->va ||
	    in->work.page_list_gpu != n->private[GPU_M4_COMPUTE_PRIVATE_PAGE_LIST]->va ||
	    in->work.counter_fw != n->counter->fw_va ||
	    in->work.notifier_fw != res->notifier->fw_va + 0x400 ||
	    in->work.firmware_stamp_fw != res->notifier->fw_va + 0x640 ||
	    in->work.user_stamp_fw != res->notifier->fw_va + 0x648 ||
	    in->work.record_fw != (out->job->queue->notifier->bo == res->notifier ?
		out->job->queue->notifier->record_fw[2] : 0))
		return M4_USER_WHY(-ESTALE, "compute check %u work aliases work=%#llx/%#llx counter=%#llx/%#llx notifier=%#llx/%#llx",
			out->command_index, M4_WHY64(in->work.work_fw),
			M4_WHY64(n->private[GPU_M4_COMPUTE_PRIVATE_WORK]->fw_va),
			M4_WHY64(in->work.counter_fw), M4_WHY64(n->counter->fw_va),
			M4_WHY64(in->work.notifier_fw), M4_WHY64(res->notifier->fw_va + 0x400));
	/* The declared whole-page counter/wait coverage, not just the counter. */
	ret = m4_user_compute_bo_locked(out, n->counter, GPU_M4_DRM_PAGE_SIZE,
		GPU_M4_DRM_UPPER_FW_ATTRS);
	if (!ret)
		ret = m4_user_compute_bo_locked(out, res->notifier,
			res->notifier->size, GPU_M4_DRM_UPPER_FW_ATTRS);
	if (!ret)
		ret = gpu_m4_drm_provider_job_timestamp_count_locked(j->core_job, &timestamps);
	if (ret)
		return M4_USER_WHY(m4_user_compute_status(ret), "compute check %u counter/notifier/timestamps",
			out->command_index);
	if (timestamps && !j->token)
		return M4_USER_WHY(-ESTALE, "compute check %u timestamps=%u without token",
			out->command_index, timestamps);
	if (j->token) {
		ret = gpu_m4_drm_timestamp_alias_token_check_locked(j->token);
		if (ret)
			return M4_USER_WHY(m4_user_compute_status(ret), "compute check %u token",
				out->command_index);
	}
	for (u32 i = 0; i < timestamps; i++) {
		struct gpu_m4_drm_provider_job_timestamp_view t;

		ret = gpu_m4_drm_provider_job_timestamp_at_locked(j->core_job, i, &t);
		if (!ret)
			ret = gpu_m4_drm_provider_job_timestamp_check_locked(j->core_job, t.runtime_alias, false);
		if (ret)
			return M4_USER_WHY(m4_user_compute_status(ret), "compute check %u timestamp %u/%u",
				out->command_index, i, timestamps);
	}
	return 0;
}

static int m4_user_compute_owner_locked(void *cookie,
	const struct gpu_m4_compute_packet_admitted_input *in,
	const struct gpu_m4_compute_packet_output *output,
	const struct gpu_m4_compute_packet_output *scratch)
{
	struct m4_user_compute_packet *out = cookie;

	if (in != &out->input || output != &out->base || scratch != &out->scratch)
		return M4_USER_WHY(-ESTALE, "compute owner %u input=%d output=%d scratch=%d",
			out->command_index, in == &out->input, output == &out->base,
			scratch == &out->scratch);
	return m4_user_compute_check_locked(out);
}

/* Reject overlapping caller storage BEFORE modifying even borrowed context.
 * The preallocated packet never aliases decoded bytes or any native CPU view.
 */
static int m4_user_compute_storage(struct m4_user_job *j,
	const struct gpu_m4_drm_provider_job_view *view,
	const struct m4_user_compute_reservation *res,
	const struct m4_user_compute_native *n, struct m4_user_compute_packet *out)
{
	const void *metadata[] = { j, j->queue, j->native_stage, res, n,
		view->batch, view->batch->commands, view->command_bytes };
	unsigned long sizes[] = { sizeof(*j), sizeof(*j->queue), sizeof(*j->native_stage),
		sizeof(*res), sizeof(*n), sizeof(*view->batch),
		view->batch->count * sizeof(*view->batch->commands), view->command_size };

	for (u32 i = 0; i < ARRAY_SIZE(metadata); i++) {
		if (!sizes[i])
			continue;
		if (!gpu_m4_compute_packet_separate(out, sizeof(*out), metadata[i], sizes[i]))
			return -EINVAL;
	}
	for (u32 i = 0; i < GPU_M4_COMPUTE_PRIVATE_COUNT + 2; i++) {
		struct m4_user_native_bo *bo = i < GPU_M4_COMPUTE_PRIVATE_COUNT ? n->private[i] :
			i == GPU_M4_COMPUTE_PRIVATE_COUNT ? n->counter : res->notifier;

		if (!bo || !gpu_m4_compute_packet_separate(out, sizeof(*out), bo, sizeof(*bo)) ||
		    !gpu_m4_compute_packet_separate(out, sizeof(*out), bo->map.vaddr, bo->size))
			return -EINVAL;
	}
	if (res->reuse && (!gpu_m4_compute_packet_separate(out, sizeof(*out),
		res->reuse, sizeof(*res->reuse)) || !res->reuse_binding ||
	    !gpu_m4_compute_packet_separate(out, sizeof(*out),
		res->reuse_binding, sizeof(*res->reuse_binding))))
		return -EINVAL;
	return 0;
}

/* Compute work +0x81c (flush_stamps, u32): the encoder writes 1. */
#define M4_USER_COMPUTE_FLUSH 0x81cU

/* REMOVE-DEBUG: prepared computes declaring WIDE_VISIBILITY, and those whose
 * host stamp flush was coalesced into a later compute's.
 */
static struct {
	u64 flagged, coalesced;
} m4_user_compute_wide_stats;

static bool m4_user_compute_wide_eligible(const struct gpu_m4_drm_mixed_batch *batch, u32 k)
{
	const struct gpu_m4_drm_job_compute *c;

	if (k + 1 >= batch->count || batch->commands[k].type != DRM_ASAHI_CMD_COMPUTE ||
	    batch->commands[k + 1].type != DRM_ASAHI_CMD_COMPUTE)
		return false;
	c = &batch->commands[k].compute;
	if (!(c->flags & DRM_ASAHI_COMPUTE_WIDE_VISIBILITY))
		return false;
	for (u32 t = 0; t < GPU_M4_DRM_JOB_TIMESTAMPS; t++)
		if (c->timestamps[t].lease)
			return false;
	return true;
}

/*
 * COMPUTE_WIDE_VISIBILITY: userspace declares its CDM stream ends with a
 * native wide memory-publication barrier (never decoded here), so this
 * compute's host stamp flush may be coalesced into a later compute's flush
 * in the same bounded packet. Execution dependencies (FW stamps), API
 * timestamps/copies and the packet's final host visibility are unchanged.
 *
 * Clear the request only where that later flush is already reachable while
 * this command is outstanding: a compute-only child (publication depth 2),
 * no API timestamp or copy, and the next command a compute that itself keeps
 * its flush. Assigned from the window's end (flush, coalesce, flush, ...),
 * so every coalesced receipt is proved by its published successor's stamps
 * (the notifier horizon rule) and no receipt waits on an unpublished one.
 */
static bool m4_user_compute_flush_coalesced(const struct m4_user_job *j,
	const struct gpu_m4_drm_mixed_batch *batch, u32 index)
{
	bool next = false;

	if (j->depth < 2 || j->mixed || index >= batch->count)
		return false;
	for (u32 k = batch->count; k-- > index; ) {
		bool coalesced = !next && m4_user_compute_wide_eligible(batch, k);

		if (k == index)
			return coalesced;
		next = coalesced;
	}
	return false;
}

static int m4_user_compute_prepare_locked(struct m4_user_job *j, u32 index,
	const struct m4_user_compute_reservation *res, struct m4_user_compute_packet *out)
{
	struct gpu_m4_drm_provider_job_view view = { };
	struct gpu_m4_compute_ts_input ts = { };
	struct gpu_m4_compute_ts_plan timestamps = { };
	struct gpu_m4_compute_reuse_plan reuse = { };
	struct gpu_m4_compute_packet_input encoded = { };
	struct gpu_m4_compute_packet_admitted_input *in;
	const struct gpu_m4_drm_job_compute *command;
	const struct m4_user_compute_native *n;
	struct m4_gpu_provider_runtime *p;
	u32 count, seen = 0;
	int ret;

	if (!j || !j->runtime || !j->queue || !j->core_job || !res || !out)
		return -EINVAL;
	p = j->runtime;
	lockdep_assert_held(&p->hello->gpu->lock);
	ret = m4_user_lane_check_locked(p, j->core_job);
	if (!ret)
		ret = m4_user_compute_reservation_check_locked(j, index, res);
	if (ret)
		return M4_USER_WHY(m4_user_compute_status(ret), "compute prepare %u lane/reservation", index);
	gpu_m4_drm_provider_job_view(j->core_job, &view);
	if (!view.batch || !view.batch->commands || !view.batch->count ||
	    view.batch->count > GPU_M4_DRM_JOB_MAX_COMMANDS || index >= view.batch->count ||
	    view.batch->commands[index].type != DRM_ASAHI_CMD_COMPUTE ||
	    view.runtime_job != j || view.runtime_queue != j->queue ||
	    view.stage_owner != j->native_stage || !view.version || !view.snapshot)
		return M4_USER_WHY(-ESTALE, "compute prepare %u view batch=%u job=%d queue=%d stage=%d version=%d snapshot=%d",
			index, view.batch ? view.batch->count : 0, view.runtime_job == j,
			view.runtime_queue == j->queue, view.stage_owner == j->native_stage,
			!!view.version, !!view.snapshot);
	n = m4_user_native_compute(j->native_stage, index);
	if (!n || !n->counter || !res->notifier)
		return M4_USER_WHY(-ESTALE, "compute prepare %u native=%d counter=%d notifier=%d", index,
			!!n, n && n->counter, !!res->notifier);
	ret = m4_user_compute_storage(j, &view, res, n, out);
	if (ret)
		return M4_USER_WHY(ret, "compute prepare %u storage", index);
	if (out->timestamps.ops || out->timestamps.held_count)
		return M4_USER_WHY(-EBUSY, "compute prepare %u held timestamps=%u", index,
			out->timestamps.held_count);
	for (u32 i = 0; i < GPU_M4_COMPUTE_TS_DESTINATIONS; i++)
		if (out->timestamps.held[i])
			return M4_USER_WHY(-EBUSY, "compute prepare %u held timestamp %u", index, i);
	command = &view.batch->commands[index].compute;
	out->job = j;
	out->command_index = index;
	out->reservation = res;
	out->native = n;
	out->vm = (struct gpu_m4_exec_vm) { .start = GPU_M4_EXEC_VM_START,
		.end = GPU_M4_EXEC_VM_END, .cookie = out, .check = m4_user_compute_range };
	in = &out->input;
	memset(in, 0, sizeof(*in));
	in->job = j->core_job;
	in->vm = j->queue->vm->provider_vm;
	in->version = view.version;
	in->snapshot = view.snapshot;
	in->command = command;
	in->binding_generation = j->binding_epoch;
	in->chronology = res->chronology;
	in->prior = res->prior;
	in->counter = n->counter->object;
	in->counter_offset = n->counter->backing.offset;
	in->notifier_page_fw = res->notifier->fw_va;
	in->timestamp = res->timestamp;
	in->owner_cookie = out;
	in->check_owner = m4_user_compute_owner_locked;
	in->work = (struct gpu_m4_exec_compute_input) {
		.vm = &out->vm, .registers = {
			.preemption = n->private[GPU_M4_COMPUTE_PRIVATE_PREEMPT]->va,
			.preemption_size = n->private[GPU_M4_COMPUTE_PRIVATE_PREEMPT]->size,
			.cdm_base = command->cdm_base, .cdm_end = command->cdm_end,
			.usc_base = j->queue->usc_base,
		},
		.work_fw = n->private[GPU_M4_COMPUTE_PRIVATE_WORK]->fw_va,
		.work_gpu = n->private[GPU_M4_COMPUTE_PRIVATE_WORK]->va,
		.queue_fw = res->queue_fw, .stats_fw = res->stats_fw,
		.auxiliary_fw = n->private[GPU_M4_COMPUTE_PRIVATE_AUX]->fw_va,
		.auxiliary_gpu = n->private[GPU_M4_COMPUTE_PRIVATE_AUX]->va,
		.context_store_gpu = n->private[GPU_M4_COMPUTE_PRIVATE_CONTEXT]->va,
		.counter_fw = n->counter->fw_va,
		.stamps_fw = n->counter->fw_va + GPU_M4_EXEC_COMPUTE_STAMPS,
		.notifier_fw = res->notifier->fw_va + 0x400,
		.record_fw = j->queue->notifier->bo == res->notifier ?
			j->queue->notifier->record_fw[2] : 0,
		.firmware_stamp_fw = res->notifier->fw_va + 0x640,
		.user_stamp_fw = res->notifier->fw_va + 0x648,
		.pool_state_gpu = n->private[GPU_M4_COMPUTE_PRIVATE_POOL_STATE]->va,
		.page_list_gpu = n->private[GPU_M4_COMPUTE_PRIVATE_PAGE_LIST]->va,
		.attachments = command->attachments, .attachment_count = command->attachment_count,
	};
	for (u32 i = 0; i < GPU_M4_COMPUTE_PRIVATE_COUNT; i++) {
		struct m4_user_native_bo *bo = n->private[i];

		in->private[i] = (struct gpu_m4_compute_packet_private) {
			.object = bo->object, .lease = bo->claim,
			.client = { bo->va, bo->size }, .object_offset = bo->mapping.offset,
			.access = bo->mapping.access,
		};
	}
	in->firmware[0] = (struct gpu_m4_exec_interval) { in->work.work_fw, GPU_M4_DRM_PAGE_SIZE };
	in->firmware[1] = (struct gpu_m4_exec_interval) { in->work.auxiliary_fw, GPU_M4_DRM_PAGE_SIZE };
	in->firmware[2] = (struct gpu_m4_exec_interval) { in->work.counter_fw, GPU_M4_DRM_PAGE_SIZE };
	if (!!res->reuse != !!res->reuse_binding)
		return M4_USER_WHY(-EINVAL, "compute prepare %u reuse=%d/%d", index, !!res->reuse,
			!!res->reuse_binding);
	if (!res->reuse) {
		ret = gpu_m4_compute_packet_prepare_admitted(&out->base, &out->scratch, in);
		if (ret == -EINVAL || ret == -ENOSPC || ret == -EOPNOTSUPP || ret == -EOVERFLOW)
			m4_user_why_compute_encode(in, index, true);
	} else {
		const struct gpu_m4_compute_reuse_owner *a = res->reuse->owner;
		const struct gpu_m4_compute_reuse_binding *b = res->reuse_binding;

		if (!a || b->job != j->core_job || b->vm != in->vm ||
		    b->version != view.version || b->snapshot != view.snapshot ||
		    res->reuse->rebuilt_work != out->base.work ||
		    res->reuse->destination_slot != res->chronology.uma_slot ||
		    a->work_fw != in->work.work_fw || a->work_gpu != in->work.work_gpu ||
		    a->counter_fw != in->work.counter_fw || a->pool_state_gpu != in->work.pool_state_gpu ||
		    a->page_list_gpu != in->work.page_list_gpu ||
		    a->pool_gpu != in->private[GPU_M4_COMPUTE_PRIVATE_POOL].client.va ||
		    a->submitted == ~0ULL || res->chronology.submitted_before != a->submitted ||
		    res->chronology.completed_before != a->submitted ||
		    res->chronology.submitted != a->submitted + 1)
			return M4_USER_WHY(-ESTALE, "compute prepare %u warm A=%d binding=%d slot=%u/%u work=%d counter=%d pool=%d submitted %llu/%llu/%llu A %llu",
				index, !!a, b->job == j->core_job && b->vm == in->vm &&
				b->version == view.version && b->snapshot == view.snapshot,
				res->reuse->destination_slot, res->chronology.uma_slot,
				a && a->work_fw == in->work.work_fw && a->work_gpu == in->work.work_gpu,
				a && a->counter_fw == in->work.counter_fw,
				a && a->pool_state_gpu == in->work.pool_state_gpu &&
				a->page_list_gpu == in->work.page_list_gpu &&
				a->pool_gpu == in->private[GPU_M4_COMPUTE_PRIVATE_POOL].client.va,
				M4_WHY64(res->chronology.submitted_before),
				M4_WHY64(res->chronology.completed_before), M4_WHY64(res->chronology.submitted),
				M4_WHY64(a ? a->submitted : 0));
		ret = gpu_m4_compute_packet_admitted_validate(&out->base, &out->scratch, in);
		if (!ret) {
			encoded.work = in->work;
			encoded.chronology = in->chronology;
			encoded.prior = in->prior;
			encoded.notifier_page_fw = in->notifier_page_fw;
			encoded.timestamp = in->timestamp;
			ret = gpu_m4_compute_packet_encode(&out->base, &out->scratch, &encoded, in->binding_generation);
		}
		if (ret == -EINVAL || ret == -ENOSPC || ret == -EOPNOTSUPP || ret == -EOVERFLOW)
			m4_user_why_compute_encode(in, index, false);
	}
	if (ret)
		return M4_USER_WHY(m4_user_compute_status(ret), "compute prepare %u encode warm=%d", index,
			!!res->reuse);
	if (res->reuse) {
		u32 seen = m4_user_why_count(); /* REMOVE-DEBUG */

		ret = gpu_m4_compute_reuse_prepare_bound(out->work, &reuse, out->ts_scratch,
			res->reuse, res->reuse_binding, j->binding_epoch, j->binding_generation);
		if (ret) {
			/* REMOVE-DEBUG: the pure plan itself refused (not its check). */
			if (m4_user_why_count() == seen)
				m4_user_why_compute_reuse(res->reuse, j->binding_generation, index, ret);
			return M4_USER_WHY_ANY(m4_user_compute_status(ret), "compute prepare %u reuse plan", index);
		}
		if (reuse.submitted_next != res->chronology.submitted)
			return M4_USER_WHY(-ESTALE, "compute prepare %u reuse next=%llu want %llu", index,
				M4_WHY64(reuse.submitted_next), M4_WHY64(res->chronology.submitted));
	} else {
		memcpy(out->work, out->base.work, sizeof(out->work));
	}
	ts.work = out->work;
	ts.work_fw = in->work.work_fw;
	ts.queue_fw = in->work.queue_fw;
	ts.stamps_fw = in->work.stamps_fw;
	ts.cookie = out;
	ts.ops = &m4_user_compute_ts_ops;
	ret = gpu_m4_drm_provider_job_timestamp_count_locked(j->core_job, &count);
	if (ret)
		return m4_user_compute_status(ret);
	for (u32 i = 0; i < count; i++) {
		struct gpu_m4_drm_provider_job_timestamp_view t;
		struct gpu_m4_compute_ts_destination d;

		ret = gpu_m4_drm_provider_job_timestamp_at_locked(j->core_job, i, &t);
		if (ret)
			return M4_USER_WHY(m4_user_compute_status(ret), "compute prepare %u timestamp %u/%u",
				index, i, count);
		if (t.command_index != index)
			continue;
		if (t.timestamp_index >= GPU_M4_DRM_JOB_TIMESTAMPS || (seen & (1U << t.timestamp_index)))
			return M4_USER_WHY(-ESTALE, "compute prepare %u timestamp slot %u seen=%#x", index,
				t.timestamp_index, seen);
		seen |= 1U << t.timestamp_index;
		d = (struct gpu_m4_compute_ts_destination) { .object = t.object, .lease = t.runtime_alias,
			.object_offset = t.object_offset, .firmware_va = t.firmware_va };
		if (!t.timestamp_index)
			ts.start = d;
		else if (t.timestamp_index == 1)
			ts.end = d;
		else
			ts.copies[ts.copy_count++] = d; /* Dense decoded order, not sparse ABI slots. */
	}
	ret = gpu_m4_compute_ts_prepare(out->scratch.work, &timestamps, out->ts_scratch, &ts);
	if (ret)
		return M4_USER_WHY(m4_user_compute_status(ret), "compute prepare %u timestamps start=%d end=%d copies=%u",
			index, !gpu_m4_compute_ts_absent(&ts.start), !gpu_m4_compute_ts_absent(&ts.end),
			ts.copy_count);
	/* No fallible operation follows the transfer of independently held TS refs. */
	memcpy(&out->bytes, &out->base, sizeof(out->bytes));
	memcpy(out->bytes.work, out->scratch.work, sizeof(out->bytes.work));
	if (command->flags & DRM_ASAHI_COMPUTE_WIDE_VISIBILITY)
		m4_user_compute_wide_stats.flagged++;
	if (!seen && m4_user_compute_flush_coalesced(j, view.batch, index)) {
		put_unaligned_le32(0, out->bytes.work + M4_USER_COMPUTE_FLUSH);
		m4_user_compute_wide_stats.coalesced++;
	}
	out->reuse_plan = reuse;
	out->timestamps = timestamps;
	return 0;
}

#endif
