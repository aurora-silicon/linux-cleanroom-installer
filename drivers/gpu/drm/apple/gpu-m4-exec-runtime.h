/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_EXEC_RUNTIME_H
#define APPLE_GPU_M4_EXEC_RUNTIME_H

#include <asm/arch_timer.h>

/* Separate queue-0 registration gate. No shader, ASID binding or teardown. */
static bool m4_exec_context_registered(struct m4_gpu_exec_memory *e)
{
	u8 *context = e->context.cpu;
	u8 first = READ_ONCE(context[0]), second = READ_ONCE(context[1]);
	u8 kind = READ_ONCE(context[2]);

	dma_rmb();
	return first != 0xff && second != 0xff && kind == 4;
}

static int m4_exec_barrier_retire(struct m4_gpu_hello *h)
{
	struct m4_gpu_exec_memory *e = h->ready->execution;
	struct m4_ready_buffer *q = e->queue[0];
	void *pipe = h->ready->buffer[GPU_M4_OBJECT_COUNT].cpu;
	void *state = q[M4_EXEC_QUEUE_STATE].cpu, *info = q[M4_EXEC_QUEUE_INFO].cpu;
	u64 deadline = ktime_get_ns() + 2 * NSEC_PER_SEC, stable = 0;
	u32 rd = 0, cfi = 0, wr = 0, qr = 0, qw = 0, consumer[3] = { 0 }, idle = 0;
	bool registered = false;
	int ret;

	do {
		u64 now;

		ret = m4_ready_health(h); /* includes unbound TTBAT[1] and every table edge */
		if (ret)
			return ret;
		ret = m4_ready_drain(h);
		if (ret)
			return ret;
		if (m4_ready_read32(h->ready->input.object[GPU_M4_CONTROL] + 0x14) != 1 ||
		    m4_ready_read32(h->ready->input.object[GPU_M4_DEVCTRL_STATE]) != 3 ||
		    m4_ready_read32(h->ready->input.object[GPU_M4_DEVCTRL_STATE] + 0x10) != 3 ||
		    m4_ready_read32(h->ready->input.object[GPU_M4_DEVCTRL_STATE] + 0x20) != 3)
			return -EPROTO;
		rd = m4_ready_read32(pipe);
		cfi = m4_ready_read32(pipe + 0x10);
		wr = m4_ready_read32(pipe + 0x20);
		qr = m4_ready_read32(state + 0x30);
		qw = m4_ready_read32(state + 0x40);
		for (unsigned int i = 0; i < 3; i++)
			consumer[i] = m4_ready_read32(info + 0x20 + i * 4);
		idle = m4_ready_read32(info + 0x80);
		if (rd > 1 || cfi > 1 || wr != 1 || qr > 1 || qw != 1 ||
		    consumer[0] > 1 || consumer[1] > 1 || consumer[2] > 1 ||
		    m4_ready_read32(state + 0x50) != GPU_M4_EXEC_RING_COUNT)
			return -EPROTO;
		registered = m4_exec_context_registered(e);
		/* The independent graph walk/drain can cross the deadline. Never
		 * accept a stable snapshot sampled after the two-second bound.
		 */
		now = ktime_get_ns();

		if (now >= deadline)
			break;
		if (rd == 1 && cfi == 1 && qr == 1 && consumer[0] == 1 &&
		    consumer[1] == 1 && consumer[2] == 1 && !idle &&
		    registered) {
			if (!stable)
				stable = now;
			if (now - stable >= 50 * NSEC_PER_MSEC)
				return 0;
		} else {
			stable = 0;
		}
		usleep_range(1000, 2000);
	} while (ktime_get_ns() < deadline);
	dev_err(h->gpu->dev, "GPU barrier timeout pipe=[%u,%u,%u] queue=[%u,%u,%u,%u,%u] idle=%u registered=%u; retained\n",
		rd, cfi, wr, qr, qw, consumer[0], consumer[1], consumer[2], idle,
		registered);
	return -ETIMEDOUT;
}

static int m4_exec_barrier_run(struct m4_gpu_hello *h)
{
	struct m4_gpu_ready *r;
	struct m4_gpu_exec_memory *e;
	struct m4_ready_buffer *q;
	struct gpu_m4_exec_pipe_record description;
	u8 record[0x18];
	void *pipe, *state, *info;
	u64 kick;
	int ret;

	if (!h || !h->ready || !execution_enable)
		return -EINVAL;
	lockdep_assert_held(&h->gpu->lock);
	r = h->ready;
	e = r->execution;
	if (!e || !e->prepared || !e->cleaned || !r->complete || !r->policy_complete ||
	    h->gpu->error || h->error || r->error || e->error)
		return -EINVAL;
	if (e->barrier_attempted)
		return -EBUSY;
	e->barrier_attempted = true;
	q = e->queue[0];
	pipe = r->buffer[GPU_M4_OBJECT_COUNT].cpu;
	state = q[M4_EXEC_QUEUE_STATE].cpu;
	info = q[M4_EXEC_QUEUE_INFO].cpu;
	ret = m4_ready_health(h);
	if (ret)
		goto done;
	ret = m4_ready_drain(h);
	if (ret)
		goto done;
	/* Refuse stale ownership. Never clear firmware-mutated cursors or context. */
	if (m4_ready_read32(r->input.object[GPU_M4_CONTROL] + 0x14) != 1 ||
	    m4_ready_read32(r->input.object[GPU_M4_FIRMWARE_DATA] + 0x4fd4) ||
	    m4_ready_read32(r->input.object[GPU_M4_DEVCTRL_STATE]) != 3 ||
	    m4_ready_read32(r->input.object[GPU_M4_DEVCTRL_STATE] + 0x10) != 3 ||
	    m4_ready_read32(r->input.object[GPU_M4_DEVCTRL_STATE] + 0x20) != 3 ||
	    m4_ready_read32(pipe) || m4_ready_read32(pipe + 0x10) ||
	    m4_ready_read32(pipe + 0x20) ||
	    m4_ready_read32(state + 0x30) || m4_ready_read32(state + 0x40) ||
	    m4_ready_read32(state + 0x50) != GPU_M4_EXEC_RING_COUNT ||
	    m4_ready_read32(info + 0x20) || m4_ready_read32(info + 0x24) ||
	    m4_ready_read32(info + 0x28) || m4_ready_read32(info + 0x80) ||
	    get_unaligned_le64(q[M4_EXEC_QUEUE_RING].cpu) ||
	    READ_ONCE(*(u8 *)e->context.cpu) != 0xff ||
	    READ_ONCE(*((u8 *)e->context.cpu + 1)) != 0xff ||
	    READ_ONCE(*((u8 *)e->context.cpu + 2)) ||
	    m4_ready_read32(e->zero_stamp.cpu) || m4_ready_read32(e->zero_stamp.cpu + 4)) {
		ret = -EPROTO;
		goto done;
	}
	description = (struct gpu_m4_exec_pipe_record) {
		.timestamp = __arch_counter_get_cntpct(),
		.queue_info = q[M4_EXEC_QUEUE_INFO].va,
		.queue_producer = 1, .pipe = 0, .scheduler_slot = 0, .new_queue = true,
	};
	/* Validate and finish the entire record before exposing any queue entry. */
	ret = gpu_m4_exec_pipe_record(record, &description);
	if (ret)
		goto done;
	ret = gpu_m4_exec_pipe_kick(0, &kick);
	if (ret)
		goto done;
	put_unaligned_le64(q[M4_EXEC_QUEUE_BARRIER].va, q[M4_EXEC_QUEUE_RING].cpu);
	e->published = true; /* any subsequent error leaves uncertain owners pinned */
	m4_ready_write32(state + 0x40, 1);
	memcpy(r->buffer[GPU_M4_OBJECT_COUNT + 1].cpu, record, sizeof(record));
	m4_ready_write32(pipe + 0x20, 1);
	ret = apple_rtkit_send_message(h->rtkit, 0x21, kick, NULL, false);
	if (!ret)
		ret = m4_exec_barrier_retire(h);
	if (!ret)
		e->barrier_complete = true;
done:
	e->error = ret;
	if (ret)
		r->error = ret; /* prevent the periodic owner from accepting further work */
	dev_info(h->gpu->dev, "GPU queue0 barrier complete=%u published=%u error=%d; ASID1 unbound, no shader, retained\n",
		 e->barrier_complete, e->published, ret);
	return ret;
}

static ssize_t execution_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_hello *h;
	struct m4_gpu_exec_memory *e;
	ssize_t size;

	mutex_lock(&gpu->lock);
	h = gpu->hello;
	e = h && h->ready ? h->ready->execution : NULL;
	if (!e || !e->prepared) {
		size = sysfs_emit(buf, "prepared=0 attempted=0\n");
	} else {
		struct m4_ready_buffer *q = e->queue[0];
		void *pipe = h->ready->buffer[GPU_M4_OBJECT_COUNT].cpu;
		void *state = q[M4_EXEC_QUEUE_STATE].cpu, *info = q[M4_EXEC_QUEUE_INFO].cpu;

		size = sysfs_emit(buf, "prepared=1 cleaned=%u allocations=%u pool_pages=%u client_tables=%u attempted=%u published=%u complete=%u error=%d pipe=%u,%u,%u queue=%u,%u,%u,%u,%u idle=%u registered=%u ttbat1=%#llx,%#llx\n",
			e->cleaned, e->allocated, e->pool_allocated, e->client.used,
			e->barrier_attempted, e->published, e->barrier_complete, e->error,
			m4_ready_read32(pipe), m4_ready_read32(pipe + 0x10),
			m4_ready_read32(pipe + 0x20), m4_ready_read32(state + 0x30),
			m4_ready_read32(state + 0x40), m4_ready_read32(info + 0x20),
			m4_ready_read32(info + 0x24), m4_ready_read32(info + 0x28),
			m4_ready_read32(info + 0x80), m4_exec_context_registered(e),
			readq(gpu->ttbs + 16), readq(gpu->ttbs + 24));
	}
	mutex_unlock(&gpu->lock);
	return size;
}
static DEVICE_ATTR_RO(execution_state);

static ssize_t barrier_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	int ret;

	if (!execution_enable || !hello_enable || !ready_enable || !policy_enable ||
	    !sysfs_streq(buf, "1"))
		return -EINVAL;
	mutex_lock(&gpu->lock);
	ret = m4_exec_barrier_run(gpu->hello);
	mutex_unlock(&gpu->lock);
	return ret ? ret : (ssize_t)count;
}
static DEVICE_ATTR_WO(barrier);

#endif
