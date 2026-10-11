/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_PROVIDER_TS_WORK_H
#define APPLE_GPU_M4_PROVIDER_TS_WORK_H

/* REMOVE-DEBUG: the unchanged first NC store, now owned by a REAL private
 * provider job/lane/cache/reservation/fence. No fixture lifecycle is invoked.
 * All packet, timestamp, root and allocator owners remain retained to reboot.
 */
static int m4_provider_ts_work_identity(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_job *job, struct m4_gpu_provider_work *w)
{
	struct gpu_m4_drm_provider_job_view view;
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!p->ts_job || p->ts_job->job != job || p->ts_job->work != w || !w ||
	    w->runtime != p || w->fixture || w->writeback || w->error ||
	    !w->slot_owned || !w->event_owned || !refcount_read(&w->refs) ||
	    !p->timestamp_manifest || !p->complete || p->error || p->work ||
	    !p->selected || p->generation != 2 || p->resident != p->version[1])
		return -EPERM;
	ret = gpu_m4_drm_provider_job_check_owner_locked(job, p->provider);
	if (ret)
		return ret;
	gpu_m4_drm_provider_job_view(job, &view);
	return view.runtime_job == p->ts_job && view.version == p->version[1] &&
		view.snapshot && gpu_m4_drm_provider_version_native_covered_locked(
			view.version, view.snapshot) ? 0 : -EPERM;
}

static int m4_provider_ts_work_health_locked(struct m4_gpu_provider_runtime *p)
{
	struct m4_gpu_provider_work *w = p->ts_job ? p->ts_job->work : NULL;
	struct m4_gpu_hello *h = p->hello;
	struct m4_gpu_exec_memory *e = p->execution;
	int ret;

	lockdep_assert_held(&h->gpu->lock);
	if (!w)
		return 0;
	ret = m4_provider_ts_work_identity(p, p->ts_job->job, w);
	if (!ret && (m4_store_control(h) || m4_provider_work_idle_engines(h) ||
	    m4_ready_read32(p->ready->input.object[GPU_M4_FIRMWARE_DATA] + 0x4fb0) ||
	    gpu_m4_drm_provider_resident_locked(p->provider) != p->version[1] ||
	    gpu_m4_drm_provider_binding_generation_locked(p->provider) != 2 ||
	    readq(h->gpu->ttbs + 0x10) != p->root[1] || readq(h->gpu->ttbs + 0x18)))
		ret = -EPROTO;
	if (ret || w->phase == GPU_M4_PROVIDER_WORK_STAGING ||
	    w->phase == GPU_M4_PROVIDER_WORK_PREPARED)
		return ret;
	if (!w->begun || !p->ready->timestamp_upper ||
	    !p->ready->timestamp_upper->committed ||
	    m4_store_read64(e->notification.cpu + 0x400) != m4_exec_notification_fw(e) + 0x610 ||
	    m4_ready_read32(e->notification.cpu + 0x424) != 1 ||
	    m4_ready_read32(e->notification.cpu + 0x514) != 1)
		return -EPROTO;
	if (w->phase == GPU_M4_PROVIDER_WORK_STAGED) {
		struct m4_ready_buffer *q = e->queue[GPU_M4_EXEC_COMPUTE];
		void *pipe = p->ready->buffer[GPU_M4_OBJECT_COUNT + 4].cpu;
		void *ring = p->ready->buffer[GPU_M4_OBJECT_COUNT + 5].cpu;

		if (memcmp(e->buffer[M4_EXEC_WORK].cpu, w->packet.work, sizeof(w->packet.work)) ||
		    memchr_inv(e->buffer[M4_EXEC_WORK].cpu + sizeof(w->packet.work), 0,
			       e->buffer[M4_EXEC_WORK].size - sizeof(w->packet.work)) ||
		    memcmp(e->buffer[M4_EXEC_COUNTER].cpu, w->packet.submitted_counter, 8) ||
		    memcmp(q[M4_EXEC_QUEUE_RING].cpu, w->packet.publication.ring_entries, 8) ||
		    memchr_inv(q[M4_EXEC_QUEUE_RING].cpu + 8, 0, 0x7f8) ||
		    memcmp(ring, w->packet.publication.pipe_record, 0x18) ||
		    memchr_inv(ring + 0x18, 0, 0x17e8) ||
		    m4_store_read64(e->notification.cpu + 0x610) ||
		    m4_ready_read32(e->notification.cpu + 0x640) ||
		    m4_ready_read32(e->notification.cpu + 0x648) ||
		    m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x30) ||
		    m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x40) ||
		    m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x50) != 256 ||
		    m4_ready_read32(q[M4_EXEC_QUEUE_INFO].cpu + 0x80) ||
		    m4_ready_read32(pipe) || m4_ready_read32(pipe + 0x10) ||
		    m4_ready_read32(pipe + 0x20) ||
		    m4_ready_read32(e->buffer[M4_EXEC_OUTPUT].cpu) != GPU_M4_STORE_SENTINEL)
			return -EPROTO;
		for (u32 i = 0; i < 3; i++)
			if (m4_ready_read32(q[M4_EXEC_QUEUE_INFO].cpu + 0x20 + i * 4))
				return -EPROTO;
		return 0;
	}
	if (w->phase != GPU_M4_PROVIDER_WORK_RUNNING &&
	    w->phase != GPU_M4_PROVIDER_WORK_RETIRED)
		return -EPROTO;
	ret = m4_provider_work_snapshot(w);
	if (!ret)
		ret = gpu_m4_provider_work_progress(&w->observed,
			m4_ready_read32(e->queue[GPU_M4_EXEC_COMPUTE][M4_EXEC_QUEUE_STATE].cpu + 0x50),
			m4_store_read64(e->notification.cpu + 0x610));
	if (!ret && w->output != GPU_M4_STORE_SENTINEL && w->output != GPU_M4_STORE_EXPECTED)
		ret = -EIO;
	if (!ret && w->phase == GPU_M4_PROVIDER_WORK_RETIRED &&
	    (!w->complete || !w->visible || gpu_m4_exec_retired(&w->observed) ||
	     w->output != GPU_M4_STORE_EXPECTED))
		ret = -EPROTO;
	return ret;
}

static int m4_provider_ts_work_owner(void *cookie,
	const struct gpu_m4_compute_packet_input *in,
	const struct gpu_m4_compute_packet_output *out,
	const struct gpu_m4_compute_packet_output *scratch)
{
	struct m4_gpu_provider_work *w = cookie;
	struct m4_gpu_provider_runtime *p = w->runtime;
	int ret = m4_provider_ts_work_identity(p, p->ts_job->job, w);

	if (!ret && (w->phase != GPU_M4_PROVIDER_WORK_PREPARED || w->begun ||
	    w->published_at || in != &w->input || out != &w->packet ||
	    scratch != &w->scratch || in->owner != w || in->owner_cookie != w ||
	    in->work.vm != &w->vm || w->vm.cookie != p || w->vm.check != m4_provider_range ||
	    in->work.attachments != &w->attachment || in->work.attachment_count != 1 ||
	    w->attachment.va != M4_EXEC_OUTPUT_GPU || w->attachment.size != 4))
		ret = -EPROTO;
	if (!ret)
		ret = m4_provider_idle(p->hello);
	return ret;
}

static int m4_provider_ts_work_encoder_check(void *cookie,
	const struct gpu_m4_compute_ts_input *in, const void *out, const void *scratch)
{
	struct m4_provider_ts_job *j = cookie;
	struct m4_gpu_provider_work *w = j->work;
	int ret = m4_provider_ts_work_identity(j->runtime, j->job, w);

	if (!ret && (in->cookie != j || in->work != w->packet.work ||
	    out != w->scratch.work || scratch != j->ts_scratch || in->copy_count ||
	    in->work_fw != w->input.work.work_fw || in->queue_fw != w->input.work.queue_fw ||
	    w->begun || w->phase != GPU_M4_PROVIDER_WORK_PREPARED))
		ret = -EPROTO;
	return ret;
}

static int m4_provider_ts_work_accept_locked(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_job *job, struct m4_gpu_provider_work **out)
{
	struct m4_gpu_provider_work *w;
	int ret = gpu_m4_drm_provider_job_check_accepting_locked(job, p->provider);

	if (ret || !out || !p->timestamp_manifest || p->work)
		return ret ?: -EPERM;
	w = kvzalloc(sizeof(*w), GFP_KERNEL);
	if (!w)
		return -ENOMEM;
	w->runtime = p;
	w->slot_owned = w->event_owned = true;
	refcount_set(&w->refs, 1);
	w->evidence = kvzalloc(sizeof(*w->evidence), GFP_KERNEL);
	w->table_evidence = kvcalloc(16, sizeof(*w->table_evidence), GFP_KERNEL);
	/* Transfer every allocated diagnostic owner, including partial failure.
	 * Its outside-lock runtime reap is responsible for unpublished disposal.
	 */
	*out = w;
	return w->evidence && w->table_evidence ? 0 : -ENOMEM;
}

static int m4_provider_ts_work_prepare_locked(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_job *job, struct m4_gpu_provider_work *w)
{
	static const struct gpu_m4_compute_ts_ops ops = {
		.check_work = m4_provider_ts_work_encoder_check,
		.hold = m4_provider_ts_hold, .put = m4_provider_ts_put,
	};
	struct gpu_m4_compute_ts_input in = { .cookie = p->ts_job, .ops = &ops };
	struct gpu_m4_drm_provider_job_timestamp_view ts;
	int ret = m4_provider_ts_work_identity(p, job, w);

	if (!ret && (w->phase != GPU_M4_PROVIDER_WORK_STAGING || w->begun ||
	    w->published_at || p->ts_job->ts_plan.held_count))
		ret = -EPROTO;
	if (!ret)
		ret = m4_provider_idle(p->hello);
	if (ret)
		return ret;
	w->phase = GPU_M4_PROVIDER_WORK_PREPARED;
	m4_provider_work_input(w);
	w->input.check_owner = m4_provider_ts_work_owner;
	ret = gpu_m4_compute_packet_prepare(&w->packet, &w->scratch, &w->input);
	for (u32 i = 0; !ret && i < 2; i++) {
		struct gpu_m4_compute_ts_destination *d = i ? &in.end : &in.start;

		ret = gpu_m4_drm_provider_job_timestamp_at_locked(job, i, &ts);
		if (!ret && (ts.gem != p->timestamp.gem || ts.object_offset != 8 + i * 8))
			ret = -EPERM;
		if (!ret)
			*d = (struct gpu_m4_compute_ts_destination) {
				.object = ts.object, .lease = ts.runtime_alias,
				.object_offset = ts.object_offset, .firmware_va = ts.firmware_va,
			};
	}
	in.work = w->packet.work;
	in.work_fw = w->input.work.work_fw;
	in.queue_fw = w->input.work.queue_fw;
	if (!ret)
		ret = gpu_m4_compute_ts_prepare(w->scratch.work, &p->ts_job->ts_plan,
			p->ts_job->ts_scratch, &in);
	if (!ret)
		memcpy(w->packet.work, w->scratch.work, sizeof(w->packet.work));
	return ret;
}

static int m4_provider_ts_work_evidence(struct m4_gpu_provider_runtime *p,
	struct m4_gpu_provider_work *w, bool before)
{
	struct m4_provider_ts_upper *t = p->ready->timestamp_upper;
	struct gpu_m4_drm_provider_job_view job_view;
	u32 phase = before ? GPU_M4_EVIDENCE_BEFORE : 0;
	u8 meta[0x50] = { 0 };
	u8 policy[16] = { 0 };
	u32 cache_count, resv_count;
	int ret;

	if (!t || !t->committed)
		return -EPROTO;
	if (!p->timestamp.cookie || !p->timestamp.gem ||
	    gpu_m4_drm_bo_policy(p->timestamp.gem) != p->timestamp.policy ||
	    gpu_m4_drm_provider_timestamp_policy(p->timestamp.cookie) != p->timestamp.policy)
		return -EPERM;
	if (before) {
		ret = m4_provider_work_capture(w, true);
		if (ret)
			return ret;
	}
	put_unaligned_le64((unsigned long)p->timestamp.gem, meta);
	put_unaligned_le64(p->timestamp.pa, meta + 8);
	put_unaligned_le64(t->base, meta + 0x10);
	put_unaligned_le64(t->parent_after, meta + 0x18);
	put_unaligned_le64(t->leaf_after, meta + 0x20);
	put_unaligned_le32(t->middle, meta + 0x28);
	put_unaligned_le32(t->index, meta + 0x2c);
	put_unaligned_le32(t->committed, meta + 0x30);
	put_unaligned_le32(t->handoff_get, meta + 0x34);
	put_unaligned_le32(t->handoff_put, meta + 0x38);
	put_unaligned_le32(t->tlbi, meta + 0x3c);
	gpu_m4_drm_provider_job_view(p->ts_job->job, &job_view);
	if (!job_view.snapshot ||
	    !gpu_m4_drm_vm_owner_cache_access(job_view.snapshot, &cache_count) ||
	    !gpu_m4_drm_vm_owner_resv_access(job_view.snapshot, &resv_count))
		return -EPROTO;
	put_unaligned_le32(cache_count, meta + 0x40);
	put_unaligned_le32(resv_count, meta + 0x44);
	put_unaligned_le64((unsigned long)p->ts_job->job, meta + 0x48);
	ret = gpu_m4_evidence_append(w->evidence, phase | 0x700,
		meta, sizeof(meta), t->base, 0, p->timestamp.pa);
	if (!ret)
		ret = gpu_m4_evidence_append(w->evidence, phase | 0x701,
			p->timestamp.preimage,
			GPU_M4_DRM_PAGE_SIZE, t->base, 0, p->timestamp.pa);
	if (!ret)
		ret = gpu_m4_evidence_append(w->evidence, phase | 0x702,
			t->child.cpu, GPU_M4_DRM_PAGE_SIZE, 0, 0, t->child.pa);
	if (!ret)
		ret = gpu_m4_evidence_append(w->evidence, phase | 0x703,
			p->hello->shared, GPU_M4_DRM_PAGE_SIZE, 0, 0,
			p->hello->region[M4_SHARED].start);
	if (!ret && p->timestamp.policy == GPU_M4_DRM_CPU_WRITEBACK) {
		put_unaligned_le32(gpu_m4_drm_bo_policy(p->timestamp.gem), policy);
		put_unaligned_le32(gpu_m4_drm_provider_timestamp_policy(p->timestamp.cookie), policy + 4);
		put_unaligned_le64(p->timestamp.gem->size, policy + 8);
		ret = gpu_m4_evidence_append(w->evidence, phase | 0x704,
			policy, sizeof(policy), t->base, 0, p->timestamp.pa);
	}
	if (!ret && !before)
		ret = m4_provider_work_capture(w, false);
	return ret;
}

static int m4_provider_ts_work_publish_locked(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_job *job, struct m4_gpu_provider_work *w)
{
	struct m4_gpu_hello *h = p->hello;
	struct m4_gpu_exec_memory *e = p->execution;
	struct m4_ready_buffer *q = e->queue[GPU_M4_EXEC_COMPUTE];
	void *pipe_state = p->ready->buffer[GPU_M4_OBJECT_COUNT + 4].cpu;
	void *pipe_ring = p->ready->buffer[GPU_M4_OBJECT_COUNT + 5].cpu;
	int ret = m4_provider_ts_work_identity(p, job, w);

	if (!ret && (w->phase != GPU_M4_PROVIDER_WORK_PREPARED || w->begun ||
	    !p->ready->timestamp_upper || !p->ready->timestamp_upper->committed))
		ret = -EPROTO;
	if (!ret)
		ret = m4_provider_idle(h);
	if (!ret)
		ret = gpu_m4_drm_provider_version_begin_work_locked(p->version[1]);
	if (ret)
		return ret;
	w->begun = true;
	memcpy(e->buffer[M4_EXEC_WORK].cpu, w->packet.work, sizeof(w->packet.work));
	memcpy(e->buffer[M4_EXEC_COUNTER].cpu, w->packet.submitted_counter, 8);
	memcpy(q[M4_EXEC_QUEUE_RING].cpu, w->packet.publication.ring_entries, 8);
	memcpy(pipe_ring, w->packet.publication.pipe_record, 0x18);
	dma_wmb();
	w->phase = GPU_M4_PROVIDER_WORK_STAGED;
	ret = m4_provider_ts_work_health_locked(p);
	if (!ret && (memcmp(e->buffer[M4_EXEC_WORK].cpu, w->packet.work,
	    sizeof(w->packet.work)) || m4_store_read64(e->notification.cpu + 0x610) ||
	    m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x30) ||
	    m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x40) ||
	    m4_ready_read32(pipe_state) || m4_ready_read32(pipe_state + 0x10) ||
	    m4_ready_read32(pipe_state + 0x20)))
		ret = -EPROTO;
	if (!ret)
		ret = m4_provider_ts_work_evidence(p, w, true);
	/* Firmware/error state can change while copying evidence despite the
	 * provider mutex. Revalidate the full staged graph and actual active token
	 * AFTER capture, not only before a potentially lengthy table export.
	 */
	if (!ret)
		ret = m4_ready_health(h);
	if (!ret)
		ret = gpu_m4_drm_timestamp_alias_token_check_locked(p->ts_job->token);
	if (!ret)
		ret = m4_provider_ts_work_health_locked(p);
	if (ret)
		return ret;
	w->input.timestamp = __arch_counter_get_cntpct();
	put_unaligned_le64(w->input.timestamp, w->packet.publication.pipe_record);
	ret = gpu_m4_evidence_append(w->evidence, GPU_M4_EVIDENCE_BEFORE | 0x6b2,
		w->packet.publication.pipe_record, 8, 0, 0, 0);
	if (ret)
		return ret;
	memcpy(pipe_ring, w->packet.publication.pipe_record, 8);
	dma_wmb();
	w->published_at = ktime_get_ns();
	w->phase = GPU_M4_PROVIDER_WORK_RUNNING;
	e->store->published = true;
	put_unaligned_le64(1, e->notification.cpu + 0x610);
	dma_wmb();
	m4_ready_write32(q[M4_EXEC_QUEUE_STATE].cpu + 0x40, 1);
	m4_ready_write32(pipe_state + 0x20, 1);
	return apple_rtkit_send_message(h->rtkit, 0x21, w->packet.publication.kick, NULL, false);
}

static int m4_provider_ts_work_retired_locked(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_job *job, struct m4_gpu_provider_work *w)
{
	u64 now;
	int ret = m4_provider_ts_work_identity(p, job, w);

	if (!ret)
		ret = m4_ready_drain(p->hello);
	if (!ret)
		ret = m4_ready_health(p->hello);
	if (!ret)
		ret = m4_provider_ts_work_health_locked(p);
	if (ret)
		return ret;
	now = ktime_get_ns();
	if (now < w->published_at || now - w->published_at >= 2 * NSEC_PER_SEC)
		return -ETIMEDOUT;
	ret = gpu_m4_exec_retired(&w->observed);
	if (ret == -EAGAIN || (!ret && w->output == GPU_M4_STORE_SENTINEL)) {
		w->stable_at = 0;
		return 0;
	}
	if (ret || w->output != GPU_M4_STORE_EXPECTED)
		return ret ?: -EIO;
	if (!w->stable_at)
		w->stable_at = now;
	ret = gpu_m4_provider_work_stable(&w->observed, w->published_at, w->stable_at, now);
	if (ret)
		return ret == -EAGAIN ? 0 : ret;
	if (memchr_inv(p->execution->buffer[M4_EXEC_OUTPUT].cpu + 4, 0,
	    p->execution->buffer[M4_EXEC_OUTPUT].size - 4))
		return -EIO;
	/* Complete fallible evidence before job-core cache/version retirement. */
	ret = m4_provider_ts_work_evidence(p, w, false);
	/* An exported diagnostic is not a new health/deadline authority. Capture
	 * may outlive a firmware fault or the original deadline; refresh actual
	 * retained graphs, token/lane, full NC retirement and guards afterward.
	 */
	if (!ret)
		ret = m4_ready_health(p->hello);
	if (!ret)
		ret = gpu_m4_drm_timestamp_alias_token_check_locked(p->ts_job->token);
	if (!ret)
		ret = m4_provider_ts_work_health_locked(p);
	if (!ret)
		ret = gpu_m4_provider_work_stable(&w->observed,
			w->published_at, w->stable_at, ktime_get_ns());
	if (!ret && (w->output != GPU_M4_STORE_EXPECTED ||
	    memchr_inv(p->execution->buffer[M4_EXEC_OUTPUT].cpu + 4, 0,
		       p->execution->buffer[M4_EXEC_OUTPUT].size - 4)))
		ret = -EIO;
	return ret ?: 1;
}

static void m4_provider_ts_work_complete_locked(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_job *job, struct m4_gpu_provider_work *w)
{
	/* Only the actual lane's infallible post-job-retire notification enters. */
	w->visible = w->complete = true;
	w->phase = GPU_M4_PROVIDER_WORK_RETIRED;
}

static void m4_provider_ts_work_fault_locked(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_job *job, struct m4_gpu_provider_work *w, int error)
{
	lockdep_assert_held(&p->hello->gpu->lock);
	if (w && !w->error) {
		struct gpu_m4_evidence *e = w->evidence;

		w->error = error < 0 ? error : -EIO;
		w->phase = GPU_M4_PROVIDER_WORK_FAULTED;
		/* Preserve the populated owned prefix even when retirement capture
		 * never ran. Sealing touches diagnostic RAM only: no live timestamp,
		 * cache retirement, owner release, or fallible fence-success stage.
		 * A prior capture error stays in e->error; a previously sealed capsule
		 * stays byte-identical and the later fault is reported in work state.
		 */
		if (e && !e->captured && e->size >= GPU_M4_EVIDENCE_HEADER &&
		    e->size <= GPU_M4_EVIDENCE_CAPACITY) {
			put_unaligned_le32(w->error, e->data + 0x14);
			(void)gpu_m4_evidence_finish(e);
		}
	}
}

static void m4_provider_ts_work_release_locked(struct m4_gpu_provider_runtime *p,
	struct m4_gpu_provider_work *w)
{
	/* No final put under GPU/resv locks. One-shot diagnostic baseline retains
	 * its work/evidence on both success and partial failures until reboot.
	 */
}

static ssize_t provider_timestamps_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_provider_runtime *p;
	int ret;

	if (!gpu || !provider_timestamp_trial || !provider_root_trial ||
	    !provider_compute_trial || !sysfs_streq(buf, "1"))
		return -EINVAL;
	mutex_lock(&gpu->lock);
	p = gpu->hello && gpu->hello->ready ? gpu->hello->ready->provider_runtime : NULL;
	ret = !p || !p->timestamp_manifest || !p->complete || p->error ||
		p->work || p->ts_attempted ? -EBUSY : 0;
	mutex_unlock(&gpu->lock);
	/* Exact permanent qualifier baseline retains p; queue creation/submit
	 * owns its provider locking and reservation/dependency machinery.
	 */
	if (!ret)
		ret = m4_provider_ts_submit(p);
	return ret ?: (ssize_t)count;
}
static DEVICE_ATTR_WO(provider_timestamps);

static ssize_t provider_timestamps_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_provider_runtime *p;
	struct m4_gpu_provider_work *w;
	u64 start = 0, end = 0;
	bool canary = false;
	const char *mode;
	ssize_t size;

	if (!gpu)
		return -ENODEV;
	mutex_lock(&gpu->lock);
	p = gpu->hello && gpu->hello->ready ? gpu->hello->ready->provider_runtime : NULL;
	mode = p && p->timestamp.policy == GPU_M4_DRM_CPU_WRITEBACK ? " mode=wb policy=1" : "";
	w = p && p->ts_job ? p->ts_job->work : NULL;
	if (!w) {
		size = sysfs_emit(buf, "attempted=%u phase=0 complete=0 error=%d visible=0 fence=0 output=0 timestamps=0,0 firmware_ns=0,0 canary=0%s\n",
			p ? p->ts_attempted : 0, p ? p->error : 0, mode);
	} else {
		/* Diagnostic CPU reads only AFTER actual job/cache/version retirement.
		 * Value comparison is external qualification, not fence authority.
		 */
		if (w->complete && w->visible && !w->error) {
			start = m4_store_read64(p->timestamp.map.vaddr + 8);
			end = m4_store_read64(p->timestamp.map.vaddr + 16);
			canary = !memcmp(p->timestamp.map.vaddr, p->timestamp.preimage, 8) &&
				!memcmp(p->timestamp.map.vaddr + 24, p->timestamp.preimage + 24,
					p->timestamp.size - 24);
		}
		size = sysfs_emit(buf, "attempted=%u phase=%u complete=%u error=%d visible=%u fence=%d output=%#x timestamps=%llu,%llu firmware_ns=%llu,%llu canary=%u%s\n",
			p->ts_attempted, w->phase, w->complete, w->error ?: p->error, w->visible,
			p->ts_finished ? dma_fence_get_status(p->ts_finished) : 0, w->output,
			w->observed.timestamp_start, w->observed.timestamp_end, start, end, canary, mode);
	}
	mutex_unlock(&gpu->lock);
	return size;
}
static DEVICE_ATTR_RO(provider_timestamps_state);

static ssize_t provider_timestamps_evidence_read(struct file *file, struct kobject *kobj,
	const struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(kobj_to_dev(kobj));
	struct m4_gpu_provider_runtime *p;
	struct gpu_m4_evidence *e = NULL;
	ssize_t size = 0;

	if (!gpu || off < 0)
		return -EINVAL;
	mutex_lock(&gpu->lock);
	p = gpu->hello && gpu->hello->ready ? gpu->hello->ready->provider_runtime : NULL;
	if (p && p->ts_job && p->ts_job->work)
		e = p->ts_job->work->evidence;
	if (e && e->captured && off < e->size) {
		count = min_t(size_t, count, e->size - off);
		memcpy(buf, e->data + off, count);
		size = count;
	}
	mutex_unlock(&gpu->lock);
	return size;
}
static BIN_ATTR_RO(provider_timestamps_evidence, 0);

#endif
