/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_STORE_RUNTIME_H
#define APPLE_GPU_M4_STORE_RUNTIME_H

#include <asm/arch_timer.h>

/* First owned ASID-1 store only. Caller holds gpu->lock throughout. Every
 * potentially visible owner, client root and firmware artifact stays pinned
 * on success and error. This does not register a render node or reusable VM.
 */
static u64 m4_store_read64(const void *data)
{
	u64 value = le64_to_cpu(READ_ONCE(*(const __le64 *)data));

	/* All callers of this helper own naturally 8-byte-aligned cells. */
	dma_rmb();
	return value;
}

static int m4_store_packed64(const void *data, u64 *value)
{
	/* Completed count +0xe54 is only u32-aligned. Reject a torn high-half
	 * sample; this bounded retry is not a claim about firmware write atomicity.
 */
	for (unsigned int pass = 0; pass < 4; pass++) {
		u32 high = m4_ready_read32((const u8 *)data + 4);
		u32 low = m4_ready_read32(data);
		u32 after = m4_ready_read32((const u8 *)data + 4);

		if (high == after) {
			*value = low | ((u64)high << 32);
			return 0;
		}
	}
	return -EAGAIN;
}

static void m4_store_asid_sync(void)
{
	asm volatile(".arch armv8.4-a\n\ttlbi aside1os, %0\n\tdsb sy"
		     : : "r" (0x0001000000000000ULL) : "memory");
}

static int m4_store_control(struct m4_gpu_hello *h)
{
	void *control = h->ready->input.object[GPU_M4_CONTROL];
	void *state = h->ready->input.object[GPU_M4_DEVCTRL_STATE];

	return m4_ready_read32(control + 0x14) == 1 &&
		m4_ready_read32(state) == 3 && m4_ready_read32(state + 0x10) == 3 &&
		m4_ready_read32(state + 0x20) == 3 ? 0 : -EPROTO;
}

static int m4_store_fresh(struct m4_gpu_hello *h)
{
	static const unsigned int ids[GPU_M4_STORE_RESOURCE_COUNT] = {
		M4_EXEC_CONTAINER, M4_EXEC_IOTO, M4_EXEC_CDM, M4_EXEC_OUTPUT,
		M4_EXEC_USC_WINDOW,
	};
	struct m4_gpu_exec_memory *e = h->ready->execution;
	struct m4_gpu_store *s = e->store;
	void *state = e->buffer[M4_EXEC_POOL_STATE].cpu;
	void *page_list = e->buffer[M4_EXEC_PAGE_LIST].cpu;
	u32 i;

	dma_rmb();
	/* Independently compare the full prepared client page, including zero
	 * thresholds/stamps and unused space. The queue's context-0 record is
	 * intentionally not selected by this work.
	 */
	for (i = 0; i < e->notification.size; i++) {
		u8 expected = 0;
		u32 record = i / 0x200, offset = i % 0x200;

		if (record < GPU_M4_EXEC_ENGINE_COUNT) {
			if (offset < 8)
				expected = (m4_exec_notification_fw(e) + 0x600 + record * 8) >> (offset * 8);
			else if (offset == 0x10)
				expected = 0x50;
			else if (offset == 0x24 || offset == 0x114)
				expected = 1;
			else if (offset >= 0xa8 && offset < 0xb0)
				expected = 0xff;
		}
		if (((u8 *)e->notification.cpu)[i] != expected)
			return -EPROTO;
	}
	for (i = 0; i < GPU_M4_STORE_RESOURCE_COUNT; i++) {
		struct m4_ready_buffer *b = &e->buffer[ids[i]];
		const struct gpu_m4_store_resource *r = &s->fixture.resource[i];

		if (r->gpu_va != m4_exec_buffer_gpu[ids[i]] || r->size > b->size ||
		    memcmp(b->cpu, r->data, r->size) ||
		    memchr_inv(b->cpu + r->size, 0, b->size - r->size))
			return -EPROTO;
	}
	for (i = M4_EXEC_PREEMPT; i <= M4_EXEC_CONTEXT_STORE; i++)
		if (memchr_inv(e->buffer[i].cpu, 0, e->buffer[i].size))
			return -EPROTO;
	if (memchr_inv(e->buffer[M4_EXEC_WORK].cpu, 0, e->buffer[M4_EXEC_WORK].size) ||
	    m4_store_read64(e->buffer[M4_EXEC_COUNTER].cpu) != 1 ||
	    memchr_inv(e->buffer[M4_EXEC_COUNTER].cpu + 8, 0,
		       e->buffer[M4_EXEC_COUNTER].size - 8) ||
	    memchr_inv(h->ready->input.object[GPU_M4_UMA] + 0x20, 0, 0x20))
		return -EPROTO;
	for (i = 0; i < 16384; i++)
		if (m4_store_read64(state + i * 8) !=
		    (i < 8192 ? M4_EXEC_POOL_GPU + (u64)i * 0x1000 : 0))
			return -EPROTO;
	for (i = 0; i < 256; i++) {
		u64 expected = i < 16 ? (1ULL << 61) | (M4_EXEC_POOL_GPU + (u64)i * 0x200000) : 0;

		if (m4_store_read64(page_list + i * 64) != expected ||
		    memchr_inv(page_list + i * 64 + 8, 0, 56))
			return -EPROTO;
	}
	return 0;
}

static int m4_store_bind(struct m4_gpu_hello *h)
{
	struct m4_gpu_exec_memory *e = h->ready->execution;
	struct m4_gpu_store *s = e->store;
	u64 root = e->table[0].pa | 1 | (1ULL << 48);
	int ret;

	if (!s->prepared || s->binding_attempted || s->bound || !e->cleaned ||
	    !e->client.sealed || e->client.asid != 1)
		return -EBUSY;
	ret = m4_ready_health(h);
	if (ret)
		return ret;
	ret = m4_store_control(h);
	if (ret)
		return ret;
	if (!m4_exec_memory_valid(h))
		return -EPROTO;
	s->binding_attempted = true;
	s->bound_root = root;
	ret = m4_hello_lock(h);
	if (ret)
		return ret;
	/* Nothing is repaired/replaced. Both halves must still be wholly vacant. */
	if (readq(h->gpu->ttbs + 16) || readq(h->gpu->ttbs + 24) ||
	    e->table[0].pa != (root & GPU_M4_PA_MASK) || !e->cleaned ||
	    !e->client.sealed || e->client.asid != 1) {
		ret = -EPROTO;
		goto unlock;
	}
	writeq(root, h->gpu->ttbs + 16);
	writeq(0, h->gpu->ttbs + 24);
	if (readq(h->gpu->ttbs + 16) != root || readq(h->gpu->ttbs + 24)) {
		ret = -EIO;
		goto unlock;
	}
	m4_hello_unlock(h);
	/* ASID is context 1; CPU TLBI operand is NOT the root descriptor/PA. */
	smp_mb();
	m4_store_asid_sync();
	s->binding_generation = 1;
	s->bound = true;
	return m4_ready_health(h);
unlock:
	m4_hello_unlock(h);
	/* Ambiguous published roots are deliberately not accepted by health. */
	return ret;
}

static int m4_store_enqueue(struct m4_gpu_hello *h, u64 *published_at)
{
	struct m4_gpu_ready *r = h->ready;
	struct m4_gpu_exec_memory *e = r->execution;
	struct m4_gpu_store *s = e->store;
	struct m4_ready_buffer *q = e->queue[GPU_M4_EXEC_COMPUTE];
	void *pipe_state = r->buffer[GPU_M4_OBJECT_COUNT + 4].cpu;
	void *pipe_ring = r->buffer[GPU_M4_OBJECT_COUNT + 5].cpu;
	struct gpu_m4_exec_attachment output = { M4_EXEC_OUTPUT_GPU, 4 };
	struct gpu_m4_exec_compute_input input = {
		.vm = &e->vm,
		.registers = { .preemption = M4_EXEC_PREEMPT_GPU, .preemption_size = 0x4000,
			.cdm_base = GPU_M4_STORE_CDM_VA, .cdm_end = s->fixture.cdm_end,
			.usc_base = s->fixture.usc_base },
		.work_fw = e->buffer[M4_EXEC_WORK].va, .work_gpu = M4_EXEC_WORK_GPU,
		.queue_fw = q[M4_EXEC_QUEUE_INFO].va,
		.stats_fw = r->input.va[GPU_M4_ACTIVITY_COMPUTE],
		.auxiliary_fw = e->buffer[M4_EXEC_AUX].va, .auxiliary_gpu = M4_EXEC_AUX_GPU,
		.context_store_gpu = M4_EXEC_CONTEXT_STORE_GPU,
		.counter_fw = e->buffer[M4_EXEC_COUNTER].va,
		.notifier_fw = m4_exec_notification_fw(e) + 0x400,
		.firmware_stamp_fw = m4_exec_notification_fw(e) + 0x640,
		.user_stamp_fw = m4_exec_notification_fw(e) + 0x648,
		.pool_state_gpu = M4_EXEC_POOL_STATE_GPU, .page_list_gpu = M4_EXEC_PAGE_LIST_GPU,
		.event_sequence = 0, .binding_generation = s->binding_generation,
		.attachments = &output, .attachment_count = 1,
	};
	struct gpu_m4_exec_pipe_record record = {
		.timestamp = __arch_counter_get_cntpct(), .queue_info = q[M4_EXEC_QUEUE_INFO].va,
		.queue_producer = 1, .pipe = 2, .scheduler_slot = 15, .new_queue = true,
	};
	u32 next_queue, next_pipe;
	u64 kick;
	int ret;

	if (!s->bound || !s->binding_generation || s->published ||
	    !gpu_m4_exec_context_registered(e->context.cpu))
		return -EPROTO;
	ret = m4_ready_health(h);
	if (ret)
		return ret;
	ret = m4_ready_drain(h);
	if (ret)
		return ret;
	ret = m4_store_fresh(h);
	if (ret)
		return ret;
	for (unsigned int i = 0; i < 3; i++)
		if (m4_ready_read32(q[M4_EXEC_QUEUE_INFO].cpu + 0x20 + i * 4))
			return -EPROTO;
	if (m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x30) ||
	    m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x40) ||
	    m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x50) != 256 ||
	    m4_ready_read32(q[M4_EXEC_QUEUE_INFO].cpu + 0x80) ||
	    m4_store_read64(q[M4_EXEC_QUEUE_FWSTAMP].cpu) ||
	    m4_store_read64(q[M4_EXEC_QUEUE_USERSTAMP].cpu) ||
	    m4_ready_read32(q[M4_EXEC_QUEUE_NOTIFICATION].cpu + 0x24) ||
	    m4_store_read64(q[M4_EXEC_QUEUE_THRESHOLD].cpu) ||
	    m4_ready_read32(pipe_state) || m4_ready_read32(pipe_state + 0x10) ||
	    m4_ready_read32(pipe_state + 0x20) ||
	    memchr_inv(q[M4_EXEC_QUEUE_RING].cpu, 0, 0x800) ||
	    memchr_inv(pipe_ring, 0, 0x1800))
		return -EPROTO;
	ret = gpu_m4_exec_ring_reserve(0, 0, 1, &next_queue);
	if (ret)
		return ret;
	ret = gpu_m4_exec_pipe_reserve(0, 0, 0, &next_pipe);
	if (ret)
		return ret;
	if (next_queue != 1 || next_pipe != 1)
		return -EPROTO;
	ret = gpu_m4_exec_first_compute(e->buffer[M4_EXEC_WORK].cpu, &input);
	if (ret)
		return ret;
	ret = gpu_m4_exec_pipe_record(pipe_ring, &record);
	if (ret)
		return ret;
	ret = gpu_m4_exec_pipe_kick(2, &kick);
	if (ret)
		return ret;
	/* Alias/table proof is repeated immediately before the first producer. */
	ret = m4_ready_health(h);
	if (ret)
		return ret;
	ret = m4_store_control(h);
	if (ret)
		return ret;
	/* The selected threshold must be visible before either producer. It is
	 * not a completion acknowledgement. Producers remain unchanged until
	 * the retained pre-submit evidence and final recovery baseline check.
	 */
	dma_wmb();
	put_unaligned_le64(1, e->notification.cpu + 0x610);
	dma_wmb();
	put_unaligned_le64(e->buffer[M4_EXEC_WORK].va, q[M4_EXEC_QUEUE_RING].cpu);
	dma_wmb();
	ret = m4_store_capture_before(h);
	if (ret)
		return ret;
	/* First publication only: an earlier recovery must not be attributed to
	 * this store. The pre-submit FWData record retains this cell at +0xf4;
	 * recheck after the potentially lengthy capture, before either producer.
	 * Post-publication recovery is expected diagnostic evidence, not this gate.
	 */
	if (m4_ready_read32(r->input.object[GPU_M4_FIRMWARE_DATA] + 0x4fb0))
		return -EPROTO;
	*published_at = ktime_get_ns();
	s->published = true;
	m4_ready_write32(q[M4_EXEC_QUEUE_STATE].cpu + 0x40, next_queue);
	m4_ready_write32(pipe_state + 0x20, next_pipe);
	return apple_rtkit_send_message(h->rtkit, 0x21, kick, NULL, false);
}

static int m4_store_snapshot(struct m4_gpu_hello *h)
{
	struct m4_gpu_exec_memory *e = h->ready->execution;
	struct m4_gpu_store *s = e->store;
	struct gpu_m4_exec_retirement *o = &s->observed;
	struct m4_ready_buffer *q = e->queue[GPU_M4_EXEC_COMPUTE];
	void *pipe = h->ready->buffer[GPU_M4_OBJECT_COUNT + 4].cpu;
	void *work = e->buffer[M4_EXEC_WORK].cpu;

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
	for (unsigned int i = 0; i < 3; i++) {
		o->queue_consumer[i] = m4_ready_read32(q[M4_EXEC_QUEUE_INFO].cpu + 0x20 + i * 4);
	}
	s->output = m4_ready_read32(e->buffer[M4_EXEC_OUTPUT].cpu);
	return m4_store_packed64(work + 0xe54, &o->uma_completed);
}

static int m4_store_observe(struct m4_gpu_hello *h)
{
	struct m4_gpu_exec_memory *e = h->ready->execution;
	struct m4_gpu_store *s = e->store;
	struct gpu_m4_exec_retirement *o = &s->observed;
	int ret = m4_store_snapshot(h);

	if (ret)
		return ret;
	for (unsigned int i = 0; i < 3; i++)
		if (o->queue_consumer[i] > 1)
			return -EPROTO;
	if (m4_ready_read32(e->queue[GPU_M4_EXEC_COMPUTE][M4_EXEC_QUEUE_STATE].cpu + 0x50) != 256 ||
	    o->queue_read > 1 || o->queue_write != 1 || o->pipe_read > 1 ||
	    o->pipe_cfi > 1 || o->pipe_write != 1 || o->uma_submitted != 1 ||
	    o->uma_completed > 1 ||
	    (o->firmware_stamp && o->firmware_stamp != 0x100) ||
	    (o->user_stamp && o->user_stamp != 0x100) ||
	    (o->timestamp_end && o->timestamp_end <= o->timestamp_start) ||
	    !gpu_m4_exec_context_registered(e->context.cpu))
		return -EPROTO;
	if (s->output != GPU_M4_STORE_SENTINEL && s->output != GPU_M4_STORE_EXPECTED)
		return -EIO;
	if (gpu_m4_exec_retired(o) || s->output != GPU_M4_STORE_EXPECTED)
		return -EAGAIN;
	return 0;
}

static int m4_store_run(struct m4_gpu_hello *h)
{
	struct m4_gpu_ready *r;
	struct m4_gpu_exec_memory *e;
	struct m4_gpu_store *s;
	u64 deadline, published_at, stable = 0, now;
	int ret;

	if (!h || !h->gpu || !h->ready)
		return -EINVAL;
	lockdep_assert_held(&h->gpu->lock);
	if (!compute_enable || h->gpu->error)
		return -EBUSY;
	r = h->ready;
	e = r->execution;
	s = e ? e->store : NULL;
	if (!s || !s->prepared || s->error || s->attempted || !r->complete ||
	    !r->policy_complete || !e->barrier_complete || r->error || h->error)
		return -EBUSY;
	s->attempted = true;
	if (upper_alias_trial) {
		ret = m4_upper_trial(h);
		if (ret)
			goto failed;
	}
	ret = m4_store_bind(h);
	if (ret)
		goto failed;
	ret = m4_store_enqueue(h, &published_at);
	if (ret)
		goto failed;
	deadline = published_at + 2 * NSEC_PER_SEC;
	do {
		m4_store_latch_fault(h);
		ret = m4_ready_health(h);
		if (ret)
			goto failed;
		ret = m4_ready_drain(h);
		if (ret)
			goto failed;
		ret = m4_store_control(h);
		if (ret)
			goto failed;
		ret = m4_store_observe(h);
		now = ktime_get_ns();
		/* Table walks/draining may themselves consume the full allowance. */
		if (now >= deadline) {
			ret = -ETIMEDOUT;
			goto failed;
		}
		if (!ret) {
			if (!stable)
				stable = now;
			if (now - stable >= 50 * NSEC_PER_MSEC) {
				s->complete = true;
				if (s->upper_published)
					s->upper_consumed = true;
				if (s->upper_consumed) {
					/* REMOVE-DEBUG: preserve a bounded positive consumer witness
					 * too, including both aliases and the zero-child preimage.
					 */
					ret = m4_store_capture_image(h, false, 0, NULL);
					if (ret)
						goto failed;
					m4_store_seal_evidence(h);
				}
				dev_info(h->gpu->dev, "GPU first store COMPLETE: ASID1 output=%#x stamps100 UMA1 timestamps=%llu,%llu; owners retained\n",
					 s->output, s->observed.timestamp_start, s->observed.timestamp_end);
				return 0;
			}
		} else if (ret == -EAGAIN) {
			stable = 0;
		} else {
			goto failed;
		}
		usleep_range(1000, 2000);
	} while (ktime_get_ns() < deadline);
	ret = -ETIMEDOUT;
failed:
	m4_store_capture_evidence(h, ret, NULL);
	/* Drain can fail before observe: label a fresh read-only sample explicitly. */
	dev_info(h->gpu->dev, "GPU first-store failure-state snapshot result=%d\n",
		 m4_store_snapshot(h));
	s->error = ret;
	r->error = ret;
	dev_err(h->gpu->dev, "GPU first store failed=%d bound=%u published=%u output=%#x stamps=%#x,%#x UMA=%llu time=%llu,%llu; owner pinned\n",
		ret, s->bound, s->published, s->output, s->observed.firmware_stamp,
		s->observed.user_stamp, s->observed.uma_completed,
		s->observed.timestamp_start, s->observed.timestamp_end);
	return ret;
}

static ssize_t store_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_store *s = NULL;
	struct m4_gpu_hello *h;
	struct gpu_m4_exec_retirement *o;
	ssize_t size;
	int ret = 0;

	if (!gpu)
		return -ENODEV;
	mutex_lock(&gpu->lock);
	h = gpu->hello;
	if (h && h->ready && h->ready->execution && h->ready->execution->store) {
		s = h->ready->execution->store;
		if (s->prepared)
			ret = m4_store_snapshot(h);
	}
	if (ret) {
		size = ret;
		goto unlock;
	}
	if (!s) {
		size = sysfs_emit(buf, "prepared=0 attempted=0 binding_attempted=0 bound=0 generation=0 published=0 complete=0 error=0 output=0 stamps=0,0 queue=0,0,0,0,0 idle=0 pipe=0,0,0 uma=0,0 timestamps=0,0\n");
		goto unlock;
	}
	o = &s->observed;
	size = sysfs_emit(buf, "prepared=%u attempted=%u binding_attempted=%u bound=%u generation=%u published=%u complete=%u error=%d output=%#x stamps=%#x,%#x queue=%u,%u,%u,%u,%u idle=%u pipe=%u,%u,%u uma=%llu,%llu timestamps=%llu,%llu\n",
		s->prepared, s->attempted, s->binding_attempted, s->bound, s->binding_generation,
		s->published, s->complete, s->error, s->output, o->firmware_stamp, o->user_stamp,
		o->queue_read, o->queue_write, o->queue_consumer[0], o->queue_consumer[1], o->queue_consumer[2],
		o->queue_busy, o->pipe_read, o->pipe_cfi, o->pipe_write,
		o->uma_submitted, o->uma_completed, o->timestamp_start, o->timestamp_end);
unlock:
	mutex_unlock(&gpu->lock);
	return size;
}
static DEVICE_ATTR_RO(store_state);

static ssize_t store_evidence_read(struct file *file, struct kobject *kobj,
		const struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(kobj_to_dev(kobj));
	struct m4_gpu_hello *h;
	struct m4_gpu_store *s = NULL;
	ssize_t size = 0;

	if (!gpu || off < 0)
		return -EINVAL;
	mutex_lock(&gpu->lock);
	h = gpu->hello;
	if (h && h->ready && h->ready->execution)
		s = h->ready->execution->store;
	if (s && s->evidence.captured && off < s->evidence.size) {
		count = min_t(size_t, count, s->evidence.size - off);
		memcpy(buf, s->evidence.data + off, count);
		size = count;
	}
	mutex_unlock(&gpu->lock);
	return size;
}
static BIN_ATTR_RO(store_evidence, 0);

static ssize_t store_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	int ret;

	if (!compute_enable || !gpu || !sysfs_streq(buf, "1"))
		return -EINVAL;
	mutex_lock(&gpu->lock);
	if (gpu->hello && gpu->hello->ready && gpu->hello->ready->provider_runtime)
		ret = -EBUSY;
	else
		ret = m4_store_run(gpu->hello);
	mutex_unlock(&gpu->lock);
	return ret ? ret : (ssize_t)count;
}
static DEVICE_ATTR_WO(store);

#endif
