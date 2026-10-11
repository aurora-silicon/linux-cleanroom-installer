/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_PROVIDER_WORK_H
#define APPLE_GPU_M4_PROVIDER_WORK_H

#include <linux/dma-fence.h>
#include "gpu-m4-compute-packet.h"
#include "gpu-m4-provider-work-state.h"
#include "gpu-m4-drm-provider-fixture.h"

/* REMOVE-DEBUG: one fresh NC or pre-bound W0 store through captured V1. Never a renderer,
 * userspace command decoder, teardown or reusable allocator. A claimed owner,
 * slot 1 and event 0 cannot be released/retried, even after success. All real
 * native/cache/reservation/session/version owners remain retained until reboot.
 */
struct m4_gpu_provider_work {
	struct m4_gpu_provider_runtime *runtime;
	struct gpu_m4_drm_provider_fixture *fixture;
	struct gpu_m4_exec_vm vm;
	struct gpu_m4_exec_attachment attachment;
	struct gpu_m4_compute_packet_input input;
	struct gpu_m4_compute_packet_output packet, scratch;
	struct gpu_m4_exec_retirement observed;
	struct gpu_m4_evidence *evidence;
	struct gpu_m4_drm_uat_table_evidence *table_evidence;
	refcount_t refs;
	u64 published_at, stable_at;
	u32 output;
	int error;
	enum gpu_m4_provider_work_phase phase;
	bool slot_owned, event_owned, begun, visible, complete, evidence_before;
	bool writeback, writeback_input, wb_prepared, prebind_protected;
	u32 challenge;
};

static u32 m4_provider_work_expected(struct m4_gpu_provider_work *w)
{
	return w->writeback_input ?
		(w->challenge < GPU_M4_WB_CHALLENGE_COUNT ?
		 w->runtime->wb_fixture.challenge[w->challenge].expected_output : 0) : GPU_M4_STORE_EXPECTED;
}

/* Metadata/preimage only: this entrance MUST NOT access the WB payload. The
 * allocation-derived native4 manifest and exact snapshot are proved separately.
 */
static int m4_provider_work_wb_owner(struct m4_gpu_provider_work *w)
{
	struct m4_gpu_provider_runtime *p = w->runtime;

	if (!w->writeback || !p->writeback_manifest || p->staged_work != w ||
	    w->writeback_input != p->writeback_input_manifest ||
	    w->challenge != p->writeback_challenge ||
	    w->challenge >= GPU_M4_WB_CHALLENGE_COUNT ||
	    !p->wb_output.gem || !p->wb_output.object || !p->wb_output.mapped ||
	    !p->wb_output.initialized || p->wb_output.map.is_iomem ||
	    p->wb_output.va != M4_EXEC_OUTPUT_GPU ||
	    p->wb_output.size != GPU_M4_DRM_PAGE_SIZE || !p->wb_output.pa ||
	    (p->wb_output.pa & (GPU_M4_DRM_PAGE_SIZE - 1)) || p->wb_output.pa >= (1ULL << 42) ||
	    !gpu_m4_compute_packet_separate(p->wb_output.preimage, GPU_M4_DRM_PAGE_SIZE,
		p->wb_output.map.vaddr, GPU_M4_DRM_PAGE_SIZE))
		return -EPROTO;
	if (w->writeback_input) {
		struct m4_provider_writeback_output *in = &p->wb_input, *out = &p->wb_output;
		const struct gpu_m4_wb_challenge *c = &p->wb_fixture.challenge[w->challenge];

		if (!in->gem || !in->object || in->gem == out->gem || in->object == out->object ||
		    !in->mapped || !in->initialized || in->map.is_iomem ||
		    in->va != GPU_M4_WB_INPUT_VA || in->size != GPU_M4_DRM_PAGE_SIZE ||
		    !in->pa || in->pa == out->pa || (in->pa & (GPU_M4_DRM_PAGE_SIZE - 1)) ||
		    in->pa >= (1ULL << 42) ||
		    !gpu_m4_compute_packet_separate(in->preimage, GPU_M4_DRM_PAGE_SIZE,
			in->map.vaddr, GPU_M4_DRM_PAGE_SIZE) ||
		    !gpu_m4_compute_packet_separate(in->preimage, GPU_M4_DRM_PAGE_SIZE,
			out->map.vaddr, GPU_M4_DRM_PAGE_SIZE) ||
		    !gpu_m4_compute_packet_separate(out->preimage, GPU_M4_DRM_PAGE_SIZE,
			in->map.vaddr, GPU_M4_DRM_PAGE_SIZE) ||
		    !gpu_m4_compute_packet_separate(in->map.vaddr, GPU_M4_DRM_PAGE_SIZE,
			out->map.vaddr, GPU_M4_DRM_PAGE_SIZE) ||
		    !gpu_m4_wb_canary(in->preimage, c->input, 0xa3) ||
		    !gpu_m4_wb_canary(out->preimage, c->sentinel, 0x5c))
			return -EPROTO;
	}
	return 0;
}

static int m4_provider_work_wb_protected(struct m4_gpu_provider_work *w)
{
	struct gpu_m4_drm_provider_fixture_view view = { 0 };
	int ret = m4_provider_work_wb_owner(w);

	if (ret || !w->fixture || !w->wb_prepared || !w->prebind_protected)
		return ret ? ret : -EPROTO;
	gpu_m4_drm_provider_fixture_view_locked(w->fixture, &view);
	return view.version == w->runtime->work_version &&
		view.state == GPU_M4_DRM_FIXTURE_PROTECTED && view.protected &&
		!view.cpu_visible && !view.error &&
		view.native_count == 4U + w->writeback_input ? 0 : -EPROTO;
}

static int m4_provider_work_snapshot(struct m4_gpu_provider_work *w)
{
	struct m4_gpu_hello *h = w->runtime->hello;
	struct m4_gpu_exec_memory *e = h->ready->execution;
	struct m4_ready_buffer *q = e->queue[GPU_M4_EXEC_COMPUTE];
	void *pipe = h->ready->buffer[GPU_M4_OBJECT_COUNT + 4].cpu;
	void *work = e->buffer[M4_EXEC_WORK].cpu;
	struct gpu_m4_exec_retirement *o = &w->observed;

	lockdep_assert_held(&h->gpu->lock);
	*o = (struct gpu_m4_exec_retirement) {
		.expected_stamp = 0x100, .next = 1, .pipe_next = 1, .pipe_capacity = 256,
		.firmware_stamp = m4_ready_read32(e->notification.cpu + 0x640),
		.user_stamp = m4_ready_read32(e->notification.cpu + 0x648),
		.queue_read = m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x30),
		.queue_write = m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x40),
		.queue_busy = m4_ready_read32(q[M4_EXEC_QUEUE_INFO].cpu + 0x80),
		.pipe_read = m4_ready_read32(pipe), .pipe_cfi = m4_ready_read32(pipe + 0x10),
		.pipe_write = m4_ready_read32(pipe + 0x20),
		.uma_submitted = m4_store_read64(e->buffer[M4_EXEC_COUNTER].cpu),
		.timestamp_start = m4_store_read64(work + 0x8c0),
		.timestamp_end = m4_store_read64(work + 0x8c8),
	};
	for (u32 i = 0; i < 3; i++)
		o->queue_consumer[i] = m4_ready_read32(q[M4_EXEC_QUEUE_INFO].cpu + 0x20 + i * 4);
	/* WB owns a separate native mapping and must not take this NC read
	 * entrance until its actual cache-retirement visibility callback.
	 */
	if (!w->writeback)
		w->output = m4_ready_read32(e->buffer[M4_EXEC_OUTPUT].cpu);
	return m4_store_packed64(work + 0xe54, &o->uma_completed);
}

static int m4_provider_work_idle_engines(struct m4_gpu_hello *h)
{
	struct m4_gpu_exec_memory *e = h->ready->execution;

	for (u32 engine = 0; engine < GPU_M4_EXEC_COMPUTE; engine++) {
		struct m4_ready_buffer *q = e->queue[engine];
		void *state = q[M4_EXEC_QUEUE_STATE].cpu, *info = q[M4_EXEC_QUEUE_INFO].cpu;
		void *pipe = h->ready->buffer[GPU_M4_OBJECT_COUNT + engine * 2].cpu;
		u32 expected = engine ? 0 : 1;

		if (m4_ready_read32(state + 0x30) != expected ||
		    m4_ready_read32(state + 0x40) != expected ||
		    m4_ready_read32(state + 0x50) != 256 || m4_ready_read32(info + 0x80) ||
		    m4_ready_read32(pipe) != expected || m4_ready_read32(pipe + 0x10) != expected ||
		    m4_ready_read32(pipe + 0x20) != expected)
			return -EPROTO;
		for (u32 i = 0; i < 3; i++)
			if (m4_ready_read32(info + 0x20 + i * 4) != expected)
				return -EPROTO;
	}
	return 0;
}

/* No calls back into ready_health: that invokes this predicate after the
 * permanent manifest/root health. Pending legal progress is not corruption.
 */
static int m4_provider_work_validate_locked(struct m4_gpu_provider_runtime *p)
{
	struct m4_gpu_provider_work *w = p->work;
	struct m4_gpu_hello *h = p->hello;
	struct m4_gpu_exec_memory *e = h->ready->execution;
	struct m4_ready_buffer *q = e->queue[GPU_M4_EXEC_COMPUTE];
	void *pipe = h->ready->buffer[GPU_M4_OBJECT_COUNT + 4].cpu;
	int ret;

	lockdep_assert_held(&h->gpu->lock);
	if (!w)
		return 0;
	if (w->runtime != p || w->error || w->phase == GPU_M4_PROVIDER_WORK_FAULTED ||
	    !refcount_read(&w->refs) || !w->slot_owned || !w->event_owned ||
	    !p->compute_manifest || !p->manifest_ready || !p->complete || !p->selected ||
	    p->generation != 2 || p->work_version != p->version[1] ||
	    w->writeback_input != p->writeback_input_manifest ||
	    w->challenge != p->writeback_challenge ||
	    (w->writeback_input && !w->writeback) ||
	    w->writeback != p->writeback_manifest ||
	    (w->writeback && (m4_provider_work_wb_owner(w) ||
	     !w->wb_prepared || !w->prebind_protected)) ||
	    p->resident != p->version[1] || !m4_exec_context_registered(e) ||
	    m4_store_control(h) ||
	    m4_ready_read32(h->ready->input.object[GPU_M4_FIRMWARE_DATA] + 0x4fb0) ||
	    m4_provider_work_idle_engines(h))
		return w->error ? w->error : -EPROTO;
	if (w->phase == GPU_M4_PROVIDER_WORK_STAGING ||
	    w->phase == GPU_M4_PROVIDER_WORK_PREPARED)
		return 0; /* Exact pristine check is mandatory at preparation. */
	if (!w->begun || !w->fixture ||
	    m4_store_read64(e->notification.cpu + 0x400) != m4_exec_notification_fw(e) + 0x610 ||
	    m4_ready_read32(e->notification.cpu + 0x424) != 1 ||
	    m4_ready_read32(e->notification.cpu + 0x514) != 1)
		return -EPROTO;
	if (w->phase == GPU_M4_PROVIDER_WORK_STAGED) {
		if (m4_ready_read32(h->ready->input.object[GPU_M4_FIRMWARE_DATA] + 0x4fd4) ||
		    memcmp(e->buffer[M4_EXEC_WORK].cpu, w->packet.work, sizeof(w->packet.work)) ||
		    memchr_inv(e->buffer[M4_EXEC_WORK].cpu + sizeof(w->packet.work), 0,
			       e->buffer[M4_EXEC_WORK].size - sizeof(w->packet.work)) ||
		    memcmp(e->buffer[M4_EXEC_COUNTER].cpu, w->packet.submitted_counter, 8) ||
		    m4_store_read64(e->notification.cpu + 0x610) ||
		    m4_ready_read32(e->notification.cpu + 0x640) ||
		    m4_ready_read32(e->notification.cpu + 0x648) ||
		    memcmp(q[M4_EXEC_QUEUE_RING].cpu, w->packet.publication.ring_entries, 8) ||
		    memchr_inv(q[M4_EXEC_QUEUE_RING].cpu + 8, 0, 0x7f8) ||
		    memcmp(h->ready->buffer[GPU_M4_OBJECT_COUNT + 5].cpu,
			   w->packet.publication.pipe_record, 0x18) ||
		    memchr_inv(h->ready->buffer[GPU_M4_OBJECT_COUNT + 5].cpu + 0x18, 0, 0x17e8) ||
		    m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x30) ||
		    m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x40) ||
		    m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x50) != 256 ||
		    m4_ready_read32(q[M4_EXEC_QUEUE_INFO].cpu + 0x80) ||
		    m4_ready_read32(pipe) || m4_ready_read32(pipe + 0x10) ||
		    m4_ready_read32(pipe + 0x20))
			return -EPROTO;
		for (u32 i = 0; i < 3; i++)
			if (m4_ready_read32(q[M4_EXEC_QUEUE_INFO].cpu + 0x20 + i * 4))
				return -EPROTO;
		if (w->writeback)
			return 0; /* WB CPU ownership ended before either root was selected. */
		return m4_ready_read32(e->buffer[M4_EXEC_OUTPUT].cpu) ==
			GPU_M4_STORE_SENTINEL ? 0 : -EPROTO;
	}
	if (w->phase != GPU_M4_PROVIDER_WORK_RUNNING &&
	    w->phase != GPU_M4_PROVIDER_WORK_RETIRED)
		return -EPROTO;
	ret = m4_provider_work_snapshot(w);
	if (ret)
		return ret;
	ret = gpu_m4_provider_work_progress(&w->observed,
		m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x50),
		m4_store_read64(e->notification.cpu + 0x610));
	if (ret)
		return ret;
	if (!w->writeback && w->output != GPU_M4_STORE_SENTINEL &&
	    w->output != GPU_M4_STORE_EXPECTED)
		return -EIO;
	if (w->phase == GPU_M4_PROVIDER_WORK_RETIRED &&
	    (!w->visible || !w->complete || gpu_m4_exec_retired(&w->observed) ||
	     w->output != m4_provider_work_expected(w)))
		return -EPROTO;
	return 0;
}

static int m4_provider_work_health(struct m4_gpu_provider_runtime *p)
{
	int ret = m4_ready_drain(p->hello);

	if (!ret)
		ret = m4_ready_health(p->hello);
	if (!ret)
		ret = m4_provider_work_validate_locked(p);
	return ret;
}

static int m4_provider_work_payload_get(void *payload)
{
	struct m4_gpu_provider_work *w = payload;
	int ret = m4_provider_session_get(w->runtime);

	if (!ret && !refcount_inc_not_zero(&w->refs)) {
		m4_provider_session_put(w->runtime);
		ret = -EIO;
	}
	return ret;
}

static void m4_provider_work_payload_put(void *payload)
{
	struct m4_gpu_provider_work *w = payload;

	if (refcount_dec_not_one(&w->refs))
		m4_provider_session_put(w->runtime);
}

static int m4_provider_work_owner(void *cookie,
	const struct gpu_m4_compute_packet_input *input,
	const struct gpu_m4_compute_packet_output *output,
	const struct gpu_m4_compute_packet_output *scratch)
{
	struct m4_gpu_provider_work *w = cookie;
	struct m4_gpu_provider_runtime *p;
	struct m4_gpu_exec_memory *e;
	int ret;

	if (!w)
		return -EPROTO;
	p = w->runtime;
	e = p->hello->ready->execution;
	lockdep_assert_held(&p->hello->gpu->lock);
	if (p->work != w || w->phase != GPU_M4_PROVIDER_WORK_PREPARED || !w->fixture ||
	    w->begun || w->published_at || !w->slot_owned || !w->event_owned ||
	    input != &w->input || output != &w->packet || scratch != &w->scratch ||
	    input->owner != w || input->owner_cookie != w || input->work.vm != &w->vm ||
	    w->vm.cookie != p || w->vm.check != m4_provider_range ||
	    input->work.attachments != &w->attachment || input->work.attachment_count != 1 ||
	    w->attachment.va != M4_EXEC_OUTPUT_GPU || w->attachment.size != 4 ||
	    !p->complete || p->generation != 2 || p->work_version != p->version[1] ||
	    !gpu_m4_drm_provider_version_native_covered_locked(p->work_version, p->work_snapshot) ||
	    e->store->attempted || e->store->binding_attempted || e->store->bound ||
	    e->store->published || e->store->upper_attempted ||
	    m4_ready_read32(p->hello->ready->input.object[GPU_M4_FIRMWARE_DATA] + 0x4fd4))
		return -EPROTO;
	ret = m4_provider_idle(p->hello);
	if (!ret && w->writeback)
		ret = m4_provider_work_wb_protected(w);
	if (!ret)
		ret = m4_provider_manifest_valid(p);
	return ret;
}

static int m4_provider_work_check_retired(void *payload,
	struct gpu_m4_drm_provider_version *version)
{
	struct m4_gpu_provider_work *w = payload;
	int ret;

	if (version != w->runtime->work_version || w->phase != GPU_M4_PROVIDER_WORK_RUNNING ||
	    !w->begun || w->visible || w->complete)
		return -EPROTO;
	ret = m4_provider_work_health(w->runtime);
	if (!ret && !w->writeback && w->output != GPU_M4_STORE_EXPECTED)
		ret = -EAGAIN;
	if (!ret)
		ret = gpu_m4_provider_work_stable(&w->observed,
			w->published_at, w->stable_at, ktime_get_ns());
	return ret;
}

static int m4_provider_work_check_visible(void *payload,
	struct gpu_m4_drm_provider_version *version)
{
	struct m4_gpu_provider_work *w = payload;
	struct m4_gpu_exec_memory *e = w->runtime->hello->ready->execution;
	void *output = e->buffer[M4_EXEC_OUTPUT].cpu;
	u64 size = e->buffer[M4_EXEC_OUTPUT].size;
	int ret;

	if (version != w->runtime->work_version || !w->begun || w->visible || w->complete ||
	    w->phase != GPU_M4_PROVIDER_WORK_RUNNING)
		return -EPROTO;
	if (w->writeback) {
		struct gpu_m4_drm_provider_fixture_view view = { 0 };

		ret = m4_provider_work_wb_owner(w);
		if (ret)
			return ret;
		gpu_m4_drm_provider_fixture_view_locked(w->fixture, &view);
		if (view.version != version || view.state != GPU_M4_DRM_FIXTURE_CPU_VISIBLE ||
		    !view.cpu_visible || !view.protected || view.error ||
		    view.native_count != 4U + w->writeback_input)
			return -EPROTO;
		output = w->runtime->wb_output.map.vaddr;
		size = w->runtime->wb_output.size;
		w->output = m4_ready_read32(output);
	}
	/* Called only after the helper's actual cache invalidation/snapshot retire.
	 * No success fence exists yet. Read the complete output allocation's guard.
	 */
	if (m4_ready_read32(output) != m4_provider_work_expected(w) ||
	    (w->writeback_input ?
	     memcmp(output + 4, w->runtime->wb_output.preimage + 4, size - 4) ||
	     memcmp(w->runtime->wb_input.map.vaddr, w->runtime->wb_input.preimage, size) :
	     !!memchr_inv(output + 4, 0, size - 4)))
		return -EIO;
	ret = gpu_m4_drm_provider_version_retire_locked(version);
	if (!ret) {
		w->visible = true;
		w->complete = true;
		w->phase = GPU_M4_PROVIDER_WORK_RETIRED;
	}
	return ret;
}

static int m4_provider_work_capture(struct m4_gpu_provider_work *w, bool before)
{
	struct m4_gpu_provider_runtime *p = w->runtime;
	struct gpu_m4_evidence *v = w->evidence;
	u32 phase = before ? GPU_M4_EVIDENCE_BEFORE : 0;
	u8 meta[0x60] = { 0 };
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!v || !w->table_evidence || v->captured || (before && w->evidence_before))
		return -EINVAL;
	if (!v->size) {
		ret = gpu_m4_evidence_begin(v, w->error, p->error, ktime_get_ns());
		if (ret)
			return ret;
	}
	put_unaligned_le64(readq(p->hello->gpu->ttbs + 0x10), meta);
	put_unaligned_le64(readq(p->hello->gpu->ttbs + 0x18), meta + 8);
	put_unaligned_le64(p->generation, meta + 0x10);
	put_unaligned_le32(w->phase, meta + 0x18);
	put_unaligned_le32(w->error, meta + 0x1c);
	put_unaligned_le64(w->published_at, meta + 0x20);
	put_unaligned_le64(w->stable_at, meta + 0x28);
	put_unaligned_le32(w->begun, meta + 0x30);
	put_unaligned_le32(w->visible, meta + 0x34);
	put_unaligned_le32(w->complete, meta + 0x38);
	put_unaligned_le32(w->slot_owned, meta + 0x3c);
	put_unaligned_le32(w->event_owned, meta + 0x40);
	put_unaligned_le32(w->writeback_input ? 2 : w->writeback, meta + 0x44);
	ret = gpu_m4_evidence_append(v, phase | 0x600, meta, sizeof(meta), 0, 0, 0);
	for (u32 index = 0; !ret && index < ARRAY_SIZE(p->version); index++) {
		struct gpu_m4_drm_provider_version_evidence header = { 0 };

		ret = gpu_m4_drm_provider_version_export_locked(p->provider, p->vm,
			p->version[index], &header, w->table_evidence, 16);
		if (!ret && before && header.uat.verify_error)
			ret = header.uat.verify_error;
		if (!ret)
			ret = gpu_m4_evidence_append(v, phase | (0x610 + index),
				&header, sizeof(header), 0, 0, 0);
		for (u32 i = 0; !ret && i < header.uat.table_count; i++) {
			struct gpu_m4_drm_uat_table_evidence *t = &w->table_evidence[i];

			ret = gpu_m4_evidence_append(v, phase | (0x640 + index * 16 + i),
				t, sizeof(*t), 0, t->va_base, t->pa);
		}
	}
	for (u32 i = 0; !ret && i < ARRAY_SIZE(p->code); i++)
		ret = gpu_m4_evidence_append(v, phase | (0x680 + i),
			p->code[i].map.vaddr, GPU_M4_DRM_PAGE_SIZE, 0, p->code[i].va, p->code[i].pa);
	for (u32 i = 0; !ret && i < ARRAY_SIZE(p->external); i++) {
		struct m4_provider_external *x = &p->external[i];
		u8 claim[0x20] = { 0 };

		put_unaligned_le64(x->va, claim);
		put_unaligned_le64(x->size, claim + 8);
		put_unaligned_le32(x->access, claim + 0x10);
		put_unaligned_le32(x->pages, claim + 0x14);
		put_unaligned_le32(refcount_read(&x->refs), claim + 0x18);
		ret = gpu_m4_evidence_append(v, phase | (0x690 + i), claim, sizeof(claim), 0, x->va, 0);
		if (!ret)
			ret = gpu_m4_evidence_append(v, phase | (0x6a0 + i),
				x->pa, x->pages * sizeof(*x->pa), 0, x->va, 0);
	}
	if (!ret && w->writeback) {
		ret = m4_provider_work_wb_owner(w);
		for (u32 role = 0; !ret && role < 1U + w->writeback_input; role++) {
			struct m4_provider_writeback_output *b = role ? &p->wb_input : &p->wb_output;
			u8 owner[0x60] = { 0 };

			put_unaligned_le32(1, owner);
			put_unaligned_le32(DRM_ASAHI_BIND_READ | (role ? 0 : DRM_ASAHI_BIND_WRITE), owner + 4);
			put_unaligned_le32(GPU_M4_DRM_CPU_WRITEBACK, owner + 8);
			put_unaligned_le64((unsigned long)b->gem, owner + 0x10);
			put_unaligned_le64((unsigned long)b->object, owner + 0x18);
			put_unaligned_le64(b->va, owner + 0x20);
			put_unaligned_le64(b->size, owner + 0x28);
			put_unaligned_le64(b->pa, owner + 0x30);
			put_unaligned_le64((unsigned long)p->work_snapshot, owner + 0x40);
			ret = gpu_m4_evidence_append(v, phase | (0x6c0 + role * 2), owner,
				sizeof(owner), 0, b->va, b->pa);
			if (!ret)
				ret = gpu_m4_evidence_append(v, phase | (0x6c1 + role * 2),
					b->preimage, GPU_M4_DRM_PAGE_SIZE, 0, b->va, b->pa);
		}
	}
	if (!ret && w->writeback_input) {
		const struct gpu_m4_wb_challenge *c = &p->wb_fixture.challenge[w->challenge];
		u8 profile[0x60] = { 0 };

		put_unaligned_le32(1, profile);
		put_unaligned_le32(w->challenge, profile + 4);
		memcpy(profile + 8, gpu_m4_wb_fixture_sha256, 32);
		memcpy(profile + 0x28, gpu_m4_wb_compiler_sha256, 32);
		put_unaligned_le32(p->wb_fixture.main_size, profile + 0x48);
		put_unaligned_le32(p->wb_fixture.nr_gprs, profile + 0x4c);
		put_unaligned_le32(p->wb_fixture.staging_halfs, profile + 0x50);
		put_unaligned_le32(p->wb_fixture.compiler_d, profile + 0x54);
		put_unaligned_le32(c->input, profile + 0x58);
		put_unaligned_le32(c->expected_output, profile + 0x5c);
		ret = gpu_m4_evidence_append(v, phase | 0x6c4, profile, sizeof(profile), 0, 0, 0);
		for (u32 i = 0; !ret && i < ARRAY_SIZE(p->code); i++)
			ret = gpu_m4_evidence_append(v, phase | (0x6d0 + i),
				p->code[i].selected_preimage, GPU_M4_DRM_PAGE_SIZE, 0,
				p->code[i].va, p->code[i].pa);
	}
	if (!ret)
		ret = gpu_m4_evidence_append(v, phase | 0x6b0,
			&w->packet, sizeof(w->packet), 0, 0, 0);
	if (!ret)
		ret = gpu_m4_evidence_append(v, phase | 0x6b1,
			&w->observed, sizeof(w->observed), 0, 0, 0);
	if (!ret && before)
		w->evidence_before = true;
	if (!before) {
		int finish = gpu_m4_evidence_finish(v);

		if (!ret)
			ret = finish;
	}
	return ret;
}

static void m4_provider_work_input(struct m4_gpu_provider_work *w)
{
	static const u32 ids[] = {
		M4_EXEC_WORK, M4_EXEC_PREEMPT, M4_EXEC_CDM, M4_EXEC_AUX,
		M4_EXEC_CONTEXT_STORE, M4_EXEC_POOL_STATE, M4_EXEC_PAGE_LIST,
	};
	struct m4_gpu_provider_runtime *p = w->runtime;
	struct m4_gpu_ready *r = p->hello->ready;
	struct m4_gpu_exec_memory *e = r->execution;
	struct m4_ready_buffer *q = e->queue[GPU_M4_EXEC_COMPUTE];
	struct gpu_m4_compute_packet_input *in = &w->input;

	w->vm = (struct gpu_m4_exec_vm){
		.start = GPU_M4_EXEC_VM_START, .end = GPU_M4_EXEC_VM_END,
		.cookie = p, .check = m4_provider_range,
	};
	w->attachment = (struct gpu_m4_exec_attachment){ M4_EXEC_OUTPUT_GPU, 4 };
	in->work = (struct gpu_m4_exec_compute_input){
		.vm = &w->vm,
		.registers = { .preemption = M4_EXEC_PREEMPT_GPU, .preemption_size = 0x4000,
			.cdm_base = GPU_M4_STORE_CDM_VA,
			.cdm_end = w->writeback_input ? p->wb_fixture.cdm_end : e->store->fixture.cdm_end,
			.usc_base = w->writeback_input ? p->wb_fixture.usc_base : e->store->fixture.usc_base },
		.work_fw = e->buffer[M4_EXEC_WORK].va, .work_gpu = M4_EXEC_WORK_GPU,
		.queue_fw = q[M4_EXEC_QUEUE_INFO].va, .stats_fw = r->input.va[GPU_M4_ACTIVITY_COMPUTE],
		.auxiliary_fw = e->buffer[M4_EXEC_AUX].va, .auxiliary_gpu = M4_EXEC_AUX_GPU,
		.context_store_gpu = M4_EXEC_CONTEXT_STORE_GPU,
		.counter_fw = e->buffer[M4_EXEC_COUNTER].va,
		.notifier_fw = m4_exec_notification_fw(e) + 0x400,
		.firmware_stamp_fw = m4_exec_notification_fw(e) + 0x640,
		.user_stamp_fw = m4_exec_notification_fw(e) + 0x648,
		.attachments = &w->attachment, .attachment_count = 1,
		.pool_state_gpu = M4_EXEC_POOL_STATE_GPU, .page_list_gpu = M4_EXEC_PAGE_LIST_GPU,
	};
	in->owner = w;
	in->owner_cookie = w;
	in->check_owner = m4_provider_work_owner;
	in->notifier_page_fw = m4_exec_notification_fw(e);
	in->timestamp = __arch_counter_get_cntpct();
	in->chronology = (struct gpu_m4_compute_packet_chronology){
		.binding_generation = 2, .asid = 1, .threshold_to_publish = 1, .submitted = 1,
		.target_stamp = 0x100, .uma_slot = 1, .new_queue = true,
	};
	for (u32 i = 0; i < ARRAY_SIZE(ids); i++)
		in->client[i] = (struct gpu_m4_exec_interval){ m4_exec_buffer_gpu[ids[i]],
			e->buffer[ids[i]].size };
	in->client[GPU_M4_COMPUTE_PACKET_POOL] =
		(struct gpu_m4_exec_interval){ M4_EXEC_POOL_GPU, GPU_M4_EXEC_POOL_SIZE };
	in->client[GPU_M4_COMPUTE_PACKET_OUTPUT] =
		(struct gpu_m4_exec_interval){ M4_EXEC_OUTPUT_GPU, GPU_M4_DRM_PAGE_SIZE };
	in->client[GPU_M4_COMPUTE_PACKET_SHADER] =
		(struct gpu_m4_exec_interval){ M4_EXEC_CONTAINER_GPU, GPU_M4_DRM_PAGE_SIZE };
	in->client[GPU_M4_COMPUTE_PACKET_PROGRAM] =
		(struct gpu_m4_exec_interval){ M4_EXEC_IOTO_GPU, GPU_M4_DRM_PAGE_SIZE };
	in->firmware[0] = (struct gpu_m4_exec_interval){ in->work.work_fw, GPU_M4_DRM_PAGE_SIZE };
	in->firmware[1] = (struct gpu_m4_exec_interval){ in->work.auxiliary_fw, GPU_M4_DRM_PAGE_SIZE };
	in->firmware[2] = (struct gpu_m4_exec_interval){ in->work.counter_fw, GPU_M4_DRM_PAGE_SIZE };
}

static int m4_provider_work_run_locked(struct m4_gpu_provider_work *w)
{
	struct m4_gpu_provider_runtime *p = w->runtime;
	struct m4_gpu_hello *h = p->hello;
	struct m4_gpu_exec_memory *e = h->ready->execution;
	struct m4_ready_buffer *q = e->queue[GPU_M4_EXEC_COMPUTE];
	void *pipe_state = h->ready->buffer[GPU_M4_OBJECT_COUNT + 4].cpu;
	void *pipe_ring = h->ready->buffer[GPU_M4_OBJECT_COUNT + 5].cpu;
	u64 now;
	int ret;

	lockdep_assert_held(&h->gpu->lock);
	ret = m4_provider_idle(h);
	if (!ret && w->writeback)
		ret = m4_provider_work_wb_protected(w);
	if (!ret && !w->writeback)
		ret = gpu_m4_drm_provider_fixture_dependencies_locked(w->fixture);
	if (!ret && !w->writeback)
		ret = gpu_m4_drm_provider_fixture_prepare_locked(w->fixture);
	if (ret)
		return ret;
	w->phase = GPU_M4_PROVIDER_WORK_PREPARED;
	m4_provider_work_input(w);
	ret = gpu_m4_compute_packet_prepare(&w->packet, &w->scratch, &w->input);
	if (!ret)
		ret = m4_provider_idle(h);
	if (!ret && !w->writeback)
		ret = gpu_m4_drm_provider_fixture_protect_locked(w->fixture);
	if (!ret)
		ret = gpu_m4_drm_provider_version_begin_work_locked(p->work_version);
	if (ret)
		return ret;
	w->begun = true;
	/* Stage immutable detached bytes while producers are still zero. */
	memcpy(e->buffer[M4_EXEC_WORK].cpu, w->packet.work, sizeof(w->packet.work));
	memcpy(e->buffer[M4_EXEC_COUNTER].cpu, w->packet.submitted_counter, 8);
	memcpy(q[M4_EXEC_QUEUE_RING].cpu, w->packet.publication.ring_entries, 8);
	memcpy(pipe_ring, w->packet.publication.pipe_record, 0x18);
	dma_wmb();
	w->phase = GPU_M4_PROVIDER_WORK_STAGED;
	ret = m4_provider_work_health(p);
	if (!ret)
		ret = m4_provider_work_capture(w, true);
	if (!ret)
		ret = m4_store_capture_before(h);
	if (!ret)
		ret = m4_provider_work_health(p);
	if (ret)
		return ret;
	/* The only post-capture byte change is the ordinary fresh pipe CNTPCT
	 * sample. Preserve its bounded delta before ANY threshold or producer;
	 * executable/work bytes and the entire ownership manifest stay unchanged.
	 */
	w->input.timestamp = __arch_counter_get_cntpct();
	put_unaligned_le64(w->input.timestamp, w->packet.publication.pipe_record);
	ret = gpu_m4_evidence_append(w->evidence, GPU_M4_EVIDENCE_BEFORE | 0x6b2,
		w->packet.publication.pipe_record, 8, 0, 0, 0);
	if (ret)
		return ret;
	memcpy(pipe_ring, w->packet.publication.pipe_record, 8);
	dma_wmb();
	/* Uncertainty begins BEFORE either producer can be seen. Existing fault
	 * latching now honestly refers to this payload publication, not old bind.
	 */
	w->published_at = ktime_get_ns();
	w->phase = GPU_M4_PROVIDER_WORK_RUNNING;
	e->store->published = true;
	put_unaligned_le64(1, e->notification.cpu + 0x610);
	dma_wmb();
	m4_ready_write32(q[M4_EXEC_QUEUE_STATE].cpu + 0x40, 1);
	m4_ready_write32(pipe_state + 0x20, 1);
	ret = apple_rtkit_send_message(h->rtkit, 0x21, w->packet.publication.kick, NULL, false);
	if (ret)
		return ret;
	for (;;) {
		m4_store_latch_fault(h);
		ret = m4_provider_work_health(p);
		if (ret)
			return ret;
		now = ktime_get_ns();
		if (now < w->published_at || now - w->published_at >= 2 * NSEC_PER_SEC)
			return -ETIMEDOUT;
		ret = gpu_m4_exec_retired(&w->observed);
		if (!ret && (w->writeback || w->output == GPU_M4_STORE_EXPECTED)) {
			if (!w->stable_at)
				w->stable_at = now;
			ret = gpu_m4_provider_work_stable(&w->observed,
				w->published_at, w->stable_at, now);
			if (!ret) {
				/* Finish all fallible evidence work BEFORE cache/version retirement
				 * can commit a success-ready fence. This is a retired-NC witness,
				 * explicitly still WORKING / not CPU_VISIBLE in its header.
				 */
				ret = m4_provider_work_capture(w, false);
				if (ret)
					return ret;
				ret = gpu_m4_drm_provider_fixture_retire_locked(w->fixture);
				if (!ret) {
					dev_info(h->gpu->dev, "GPU captured V1 first %s store COMPLETE: output=%#x generation=2 stamps100 UMA1 time=%llu,%llu; all owners retained\n",
						w->writeback_input ? "W1 WB input" : w->writeback ? "W0 WB" : "NC", w->output,
						w->observed.timestamp_start,
						w->observed.timestamp_end);
				}
				return ret;
			}
		} else if (ret == -EAGAIN ||
			   (!ret && !w->writeback && w->output == GPU_M4_STORE_SENTINEL)) {
			w->stable_at = 0;
		} else {
			return ret ? ret : -EIO;
		}
		usleep_range(1000, 2000);
	}
}

static void m4_provider_work_fault_locked(struct m4_gpu_provider_work *w, int error)
{
	w->error = error < 0 ? error : -EIO;
	w->complete = false;
	w->phase = GPU_M4_PROVIDER_WORK_FAULTED;
	m4_provider_fault_locked(w->runtime, w->error);
	if (w->fixture)
		gpu_m4_drm_provider_fixture_fault_locked(w->fixture, w->error);
	/* Read-only retained diagnostics: never reset, retry or grant retirement. */
	m4_provider_work_snapshot(w);
	if (w->evidence && !w->evidence->captured)
		m4_provider_work_capture(w, false);
	m4_store_capture_evidence(w->runtime->hello, w->error, NULL);
}

/* Exact future V1, while ASID1 is still vacant. Never publish the active work
 * hook or claim slot/event here. All staged pointers are installed together.
 */
static int m4_provider_wb_prebind_stage(struct m4_gpu_provider_runtime *p)
{
	struct m4_gpu_power *gpu = p->hello->gpu;
	struct m4_gpu_provider_work *w;
	struct gpu_m4_drm_provider_fixture *fixture = NULL;
	struct gpu_m4_evidence *evidence;
	struct gpu_m4_drm_uat_table_evidence *tables;
	struct gpu_m4_drm_provider_fixture_config config;
	int ret;

	lockdep_assert_not_held(&gpu->lock);
	mutex_lock(&gpu->lock);
	if (!p->writeback_manifest || !p->compute_manifest || !p->manifest_ready ||
	    p->error || p->staged_work || p->work ||
	    p->selected || p->complete || p->generation || p->resident ||
	    !p->version[0] || !p->version[1] ||
	    p->work_version != p->version[1] || !p->work_snapshot ||
	    readq(gpu->ttbs + 0x10) || readq(gpu->ttbs + 0x18)) {
		mutex_unlock(&gpu->lock);
		return -EPROTO;
	}
	w = kvzalloc(sizeof(*w), GFP_KERNEL);
	if (!w) {
		mutex_unlock(&gpu->lock);
		return -ENOMEM;
	}
	w->runtime = p;
	w->writeback = true;
	w->writeback_input = p->writeback_input_manifest;
	w->challenge = p->writeback_challenge;
	refcount_set(&w->refs, 1);
	p->staged_work = w;
	ret = m4_provider_work_wb_owner(w);
	if (!ret)
		ret = m4_provider_manifest_valid(p);
	if (!ret && !gpu_m4_drm_provider_version_native_covered_locked(
		p->work_version, p->work_snapshot))
		ret = -EPROTO;
	mutex_unlock(&gpu->lock);
	evidence = kvzalloc(sizeof(*evidence), GFP_KERNEL);
	tables = kvcalloc(16, sizeof(*tables), GFP_KERNEL);
	if (!ret && (!evidence || !tables))
		ret = -ENOMEM;
	config = (struct gpu_m4_drm_provider_fixture_config) {
		.provider = p->provider, .version = p->work_version, .payload = w,
		.payload_get = m4_provider_work_payload_get,
		.payload_put = m4_provider_work_payload_put,
		.check_retired_locked = m4_provider_work_check_retired,
		.check_visible_locked = m4_provider_work_check_visible,
	};
	if (!ret)
		ret = gpu_m4_drm_provider_fixture_stage(&config, &fixture);
	mutex_lock(&gpu->lock);
	w->evidence = evidence;
	w->table_evidence = tables;
	w->fixture = fixture; /* Including a failed rollback's retained owner. */
	mutex_unlock(&gpu->lock);
	return ret;
}

static int m4_provider_wb_prebind_protect_locked(struct m4_gpu_provider_runtime *p)
{
	struct m4_gpu_provider_work *w = p->staged_work;
	struct gpu_m4_drm_provider_fixture_view view = { 0 };
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!w || w->runtime != p || w->phase != GPU_M4_PROVIDER_WORK_STAGING ||
	    !p->compute_manifest || !p->manifest_ready || p->error ||
	    p->work || p->selected || p->complete || p->generation || p->resident ||
	    w->error || w->begun || w->slot_owned || w->event_owned || w->wb_prepared ||
	    w->prebind_protected || !w->fixture || !w->evidence || !w->table_evidence ||
	    m4_provider_work_wb_owner(w) || p->work_version != p->version[1] ||
	    readq(p->hello->gpu->ttbs + 0x10) || readq(p->hello->gpu->ttbs + 0x18))
		return -EPROTO;
	gpu_m4_drm_provider_fixture_view_locked(w->fixture, &view);
	if (view.version != p->work_version || view.state != GPU_M4_DRM_FIXTURE_STAGED ||
	    view.error || view.native_count != 4U + w->writeback_input ||
	    view.protected || view.cpu_visible)
		return -EPROTO;
	ret = m4_provider_manifest_valid(p);
	if (!ret)
		ret = gpu_m4_drm_provider_fixture_dependencies_locked(w->fixture);
	if (!ret) {
		/* Any attempted prepare starts the conservative CPU-access cutoff. */
		w->wb_prepared = true;
		ret = gpu_m4_drm_provider_fixture_prepare_locked(w->fixture);
	}
	if (!ret)
		ret = gpu_m4_drm_provider_fixture_protect_locked(w->fixture);
	if (!ret) {
		w->prebind_protected = true;
		ret = m4_provider_work_wb_protected(w);
	}
	return ret;
}

static void m4_provider_wb_prebind_fault(struct m4_gpu_provider_runtime *p, int error)
{
	struct m4_gpu_power *gpu = p->hello->gpu;
	struct m4_gpu_provider_work *w;
	struct gpu_m4_drm_provider_fixture *fixture = NULL;

	lockdep_assert_not_held(&gpu->lock);
	mutex_lock(&gpu->lock);
	w = p->staged_work;
	if (w) {
		m4_provider_work_fault_locked(w, error);
		fixture = w->fixture;
	} else {
		m4_provider_fault_locked(p, error < 0 ? error : -EIO);
	}
	mutex_unlock(&gpu->lock);
	if (fixture)
		gpu_m4_drm_provider_fixture_signal(fixture);
}

static ssize_t provider_work_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_provider_runtime *p;
	struct m4_gpu_provider_work *w;
	struct gpu_m4_drm_provider_fixture *staged = NULL;
	struct gpu_m4_evidence *evidence = NULL;
	struct gpu_m4_drm_uat_table_evidence *tables = NULL;
	struct gpu_m4_drm_provider_fixture_config config;
	int ret, signal = 0;

	if (!gpu || !provider_root_trial || !provider_compute_trial || !compute_enable ||
	    !sysfs_streq(buf, "1"))
		return -EINVAL;
	mutex_lock(&gpu->lock);
	p = gpu->hello && gpu->hello->ready ? gpu->hello->ready->provider_runtime : NULL;
	if (!p || p->timestamp_manifest || !p->complete || p->error || !p->manifest_ready || !p->compute_manifest ||
	    p->work || p->generation != 2 || !p->work_version || !p->work_snapshot) {
		mutex_unlock(&gpu->lock);
		return -EBUSY;
	}
	if (p->writeback_manifest) {
		w = p->staged_work;
		if (!w || w->runtime != p || w->error || w->phase != GPU_M4_PROVIDER_WORK_STAGING ||
		    w->slot_owned || w->event_owned || w->begun || w->published_at ||
		    !w->evidence || !w->table_evidence || m4_provider_work_wb_protected(w)) {
			mutex_unlock(&gpu->lock);
			m4_provider_wb_prebind_fault(p, -EPROTO);
			return -EPROTO;
		}
		p->work = w; /* staged_work remains the same permanent owner alias. */
		w->slot_owned = true;
		w->event_owned = true;
		ret = m4_provider_idle(p->hello);
		goto claimed;
	}
	w = kvzalloc(sizeof(*w), GFP_KERNEL);
	if (!w) {
		mutex_unlock(&gpu->lock);
		return -ENOMEM;
	}
	w->runtime = p;
	w->slot_owned = true;
	w->event_owned = true;
	refcount_set(&w->refs, 1);
	/* Single actual owner claim; no other producer exists on this private
	 * unregistered frontend and legacy store rejects provider presence.
	 */
	p->work = w;
	ret = m4_provider_idle(p->hello);
	mutex_unlock(&gpu->lock);
	if (!ret) {
		evidence = kvzalloc(sizeof(*evidence), GFP_KERNEL);
		tables = kvcalloc(16, sizeof(*tables), GFP_KERNEL);
		if (!evidence || !tables)
			ret = -ENOMEM;
	}
	config = (struct gpu_m4_drm_provider_fixture_config){
		.provider = p->provider, .version = p->work_version, .payload = w,
		.payload_get = m4_provider_work_payload_get, .payload_put = m4_provider_work_payload_put,
		.check_retired_locked = m4_provider_work_check_retired,
		.check_visible_locked = m4_provider_work_check_visible,
	};
	if (!ret)
		ret = gpu_m4_drm_provider_fixture_stage(&config, &staged);
	mutex_lock(&gpu->lock);
	/* Publish only fully staged pointers under the actual owner mutex. Even a
	 * refused rollback returns a retained non-NULL fixture, kept on this owner.
	 */
	w->evidence = evidence;
	w->table_evidence = tables;
	w->fixture = staged;
claimed:
	if (!ret)
		ret = m4_provider_work_run_locked(w);
	if (ret)
		m4_provider_work_fault_locked(w, ret);
	mutex_unlock(&gpu->lock);
	if (w->fixture)
		signal = gpu_m4_drm_provider_fixture_signal(w->fixture);
	if (!ret && signal) {
		mutex_lock(&gpu->lock);
		m4_provider_work_fault_locked(w, signal);
		mutex_unlock(&gpu->lock);
		ret = signal;
	}
	return ret ? ret : (ssize_t)count;
}
static DEVICE_ATTR_WO(provider_work);

static ssize_t provider_work_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_provider_runtime *p;
	struct m4_gpu_provider_work *w;
	struct gpu_m4_drm_provider_fixture_view view = { 0 };
	struct dma_fence *fence = NULL;
	ssize_t size;
	bool active;

	if (!gpu)
		return -ENODEV;
	mutex_lock(&gpu->lock);
	p = gpu->hello && gpu->hello->ready ? gpu->hello->ready->provider_runtime : NULL;
	active = p && p->work;
	w = p ? (p->work ? p->work : p->staged_work) : NULL;
	if (!w) {
		size = sysfs_emit(buf, "attempted=0 phase=0 complete=0 error=0 visible=0 cache=0 fence=0 native=0 output=0 stamps=0,0 queue=0,0,0,0,0 idle=0 pipe=0,0,0 uma=0,0 timestamps=0,0\n");
	} else {
		struct gpu_m4_exec_retirement *o = &w->observed;
		/* Staging owns cache/fence/evidence, not an attempted engine job.
		 * Never apply active-work health or publish/claim it through a reader.
		 */
		int ret = w->error ? w->error : active ? m4_provider_work_health(p) : 0;

		if (ret && !w->error)
			m4_provider_work_fault_locked(w, ret);
		if (w->fixture) {
			gpu_m4_drm_provider_fixture_view_locked(w->fixture, &view);
			fence = gpu_m4_drm_provider_fixture_fence_get(w->fixture);
		}
		size = sysfs_emit(buf, "attempted=%u phase=%u complete=%u error=%d visible=%u "
			"cache=%u fence=%d native=%u output=%#x stamps=%#x,%#x "
			"queue=%u,%u,%u,%u,%u idle=%u pipe=%u,%u,%u uma=%llu,%llu "
			"timestamps=%llu,%llu%s%s\n",
			active, w->phase, w->complete, w->error, w->visible, view.state,
			fence ? dma_fence_get_status(fence) : 0, view.native_count,
			w->output, o->firmware_stamp, o->user_stamp,
			o->queue_read, o->queue_write, o->queue_consumer[0], o->queue_consumer[1], o->queue_consumer[2],
			o->queue_busy, o->pipe_read, o->pipe_cfi, o->pipe_write,
			o->uma_submitted, o->uma_completed, o->timestamp_start, o->timestamp_end,
			w->writeback_input ? (w->challenge ? " mode=wb-input challenge=1" :
			 " mode=wb-input challenge=0") : w->writeback ? " mode=wb" : "",
			active ? "" : " staged=1");
	}
	mutex_unlock(&gpu->lock);
	dma_fence_put(fence);
	return size;
}
static DEVICE_ATTR_RO(provider_work_state);

static ssize_t provider_work_evidence_read(struct file *file, struct kobject *kobj,
	const struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(kobj_to_dev(kobj));
	struct m4_gpu_provider_runtime *p;
	struct m4_gpu_provider_work *w;
	struct gpu_m4_evidence *e = NULL;
	ssize_t size = 0;

	if (!gpu || off < 0)
		return -EINVAL;
	mutex_lock(&gpu->lock);
	p = gpu->hello && gpu->hello->ready ? gpu->hello->ready->provider_runtime : NULL;
	w = p ? (p->work ? p->work : p->staged_work) : NULL;
	if (w)
		e = w->evidence; /* Sealed retained bytes only; no live WB payload. */
	if (e && e->captured && off < e->size) {
		count = min_t(size_t, count, e->size - off);
		memcpy(buf, e->data + off, count);
		size = count;
	}
	mutex_unlock(&gpu->lock);
	return size;
}
static BIN_ATTR_RO(provider_work_evidence, 0);

#endif
