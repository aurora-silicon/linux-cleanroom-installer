/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_USER_WIRE_H
#define APPLE_GPU_M4_USER_WIRE_H

/* Actual ordinary runtime ring glue. A command is exposed whole, never one
 * render stage at a time; a bounded window can publish successors behind it.
 * Its native owners and the entire copied public batch were captured before
 * cache preparation. These helpers neither complete a job nor release any
 * backing.
 */
static int m4_user_wire_queue_locked(struct m4_user_job *j, u32 engine)
{
	struct m4_gpu_provider_runtime *p = j->runtime;
	struct m4_gpu_exec_memory *e = p->execution;
	struct m4_ready_buffer *q;
	struct m4_ready_buffer *pipe;
	u32 producer, pipe_producer;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (engine >= 3 || e != p->ready->execution || !e->barrier_complete ||
	    !m4_exec_context_registered(e))
		return M4_USER_WHY(-ESTALE, "wire queue %u execution=%d barrier=%d registered=%d", engine,
			e == p->ready->execution, e && e->barrier_complete,
			e && m4_exec_context_registered(e));
	q = e->queue[engine];
	pipe = &p->ready->buffer[GPU_M4_OBJECT_COUNT + engine * 2];
	if (!m4_exec_backing_valid(p->hello, &q[M4_EXEC_QUEUE_INFO],
		GPU_M4_EXEC_QUEUE_SIZE, true) ||
	    !m4_exec_backing_valid(p->hello, &q[M4_EXEC_QUEUE_STATE],
		GPU_M4_EXEC_RING_STATE_SIZE, true) ||
	    !m4_exec_backing_valid(p->hello, &q[M4_EXEC_QUEUE_RING], 256 * 8, true) ||
	    !m4_exec_backing_valid(p->hello, pipe, 0x30, true) ||
	    !m4_exec_backing_valid(p->hello, pipe + 1, 256 * 0x18, true) ||
	    m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x50) != 256)
		return M4_USER_WHY(-ESTALE, "wire queue %u backing info=%d state=%d ring=%d pipe=%d,%d capacity=%u",
			engine, m4_exec_backing_valid(p->hello, &q[M4_EXEC_QUEUE_INFO],
				GPU_M4_EXEC_QUEUE_SIZE, true),
			m4_exec_backing_valid(p->hello, &q[M4_EXEC_QUEUE_STATE],
				GPU_M4_EXEC_RING_STATE_SIZE, true),
			m4_exec_backing_valid(p->hello, &q[M4_EXEC_QUEUE_RING], 256 * 8, true),
			m4_exec_backing_valid(p->hello, pipe, 0x30, true),
			m4_exec_backing_valid(p->hello, pipe + 1, 256 * 0x18, true),
			m4_exec_backing_valid(p->hello, &q[M4_EXEC_QUEUE_STATE],
				GPU_M4_EXEC_RING_STATE_SIZE, true) ?
			m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x50) : 0);
	producer = m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x40);
	pipe_producer = m4_ready_read32(pipe->cpu + 0x20);
	return producer < 256 && pipe_producer < 256 ? 0 :
		M4_USER_WHY(-EPROTO, "wire queue %u producer=%u pipe=%u", engine, producer, pipe_producer);
}

/* Bounded window on one physical queue. Each command keeps its own saved
 * queue/pipe ends per engine (compute 2, render TA 0 and fragment 1);
 * successors append behind them while earlier owned commands execute. Only
 * admitted in-flight jobs' published commands may be outstanding: every
 * consumer lies between the oldest unreceipted owned start and the owned
 * producer, and no producer change is unowned.
 *
 * The global per-engine wire ledger: over every admitted in-flight job, in
 * actual publication order, tail is the last reachable record (its ends are
 * the owned producers) and oldest the first one still without a receipt.
 * The calling job contributes only its commands before limit. With one job in
 * flight this is exactly that job's own window.
 */
struct m4_user_wire_ledger {
	const struct m4_user_engine *tail, *oldest;
};

static void m4_user_wire_ledger_job(const struct m4_user_job *j, u32 engine, u32 limit,
	struct m4_user_wire_ledger *ledger)
{
	for (u32 i = 0; i < limit && i < ARRAY_SIZE(j->cmd); i++) {
		const struct m4_user_command *c = &j->cmd[i];
		const struct m4_user_engine *w = &c->wire[engine];

		if (!c->published || !(c->engine_mask & BIT(engine)) || !w->count)
			continue; /* A command of the other type owns no record here. */
		if (!w->sequence)
			continue; /* Installed in a still open packet: not reachable. */
		if (!ledger->tail || w->sequence >= ledger->tail->sequence)
			ledger->tail = w;
		if (!c->receipt && (!ledger->oldest || w->sequence < ledger->oldest->sequence))
			ledger->oldest = w;
	}
}

static_assert(sizeof(struct m4_user_engine) <=
	      sizeof(((struct m4_gpu_provider_runtime *)0)->user_wire_last[0]),
	      "wire record copy storage");

/* inflight2: the copy of the newest reachable record on engine, of any job
 * (also one that retired since), sequence 0 when none.
 */
static const struct m4_user_engine *m4_user_wire_last(const struct m4_gpu_provider_runtime *p,
	u32 engine)
{
	return (const struct m4_user_engine *)p->user_wire_last[engine];
}

static void m4_user_wire_ledger(const struct m4_user_job *j, u32 engine, u32 limit,
	struct m4_user_wire_ledger *ledger)
{
	const struct m4_gpu_provider_runtime *p = j->runtime;
	const struct m4_user_engine *last;

	*ledger = (struct m4_user_wire_ledger) { };
	for (u32 n = 0; n < M4_USER_INFLIGHT_MAX; n++) {
		const struct m4_user_inflight *o = &p->user_inflight[n];

		if (o->item && o->job && o->job != j->core_job)
			m4_user_wire_ledger_job(container_of(o->item, struct m4_user_job, item),
				engine, ARRAY_SIZE(j->cmd), ledger);
	}
	m4_user_wire_ledger_job(j, engine, limit, ledger);
	/* A newer reachable record of a job that already retired (published
	 * behind this job's earlier records): the owned producer and the
	 * physical horizon are its ends. Consumed records need no oldest.
	 */
	last = m4_user_wire_last(p, engine);
	if (p->user_inflight2 && last->sequence &&
	    (!ledger->tail || last->sequence > ledger->tail->sequence))
		ledger->tail = last;
}

/* inflight2: an admitted job other than j is in flight. The working runtime's
 * non-solitary rule then applies on every engine: queue-info busy may stay set
 * while other work executes, so only consumer, producer and pipe positions
 * (and own stamps/UMA at retirement) are proof. Always false at a cap of one.
 */
static bool m4_user_wire_overlapped(const struct m4_user_job *j)
{
	const struct m4_gpu_provider_runtime *p = j->runtime;

	if (!p->user_inflight2)
		return false;
	for (u32 n = 0; n < M4_USER_INFLIGHT_MAX; n++) {
		const struct m4_user_inflight *o = &p->user_inflight[n];

		if (o->item && o->job && o->job != j->core_job)
			return true;
	}
	return false;
}

/* A reachable command's published tail target against the runtime's global
 * target on engine: exactly this job's tail, or (inflight2) later jobs'
 * targets published behind it: reached, and within one bounded job window of
 * targets (one 0x100 per command). Those jobs need not be in flight any
 * more: a head still publishing may append behind its successor and retire
 * before it (r66: five renders of a retired head ahead of a solitary job).
 */
static bool m4_user_wire_target_tail(const struct m4_user_job *j, u32 engine, u32 tail)
{
	u32 now = j->runtime->user_target[engine];

	if (now == tail)
		return true;
	return j->runtime->user_inflight2 && gpu_m4_exec_stamp_reached(now, tail) &&
		now - tail <= GPU_M4_DRM_JOB_MAX_COMMANDS * 0x100U;
}

/* A reachable command's notifier tail (its job's last threshold or stage
 * counter) against the notifier's current value: exactly this job's tail, or,
 * while another job is in flight, a later job of the same logical queue
 * published behind it on the same page (same-queue overlap): ahead, by at
 * most one bounded job window (step per command).
 */
static bool m4_user_wire_notifier_tail(const struct m4_user_job *j, u64 now, u64 tail, u64 step)
{
	if (now == tail)
		return true;
	return m4_user_wire_overlapped(j) && now > tail &&
		now - tail <= (u64)GPU_M4_DRM_JOB_MAX_COMMANDS * step;
}

/* Live window/credit validation for count new entries at producer/pipe. */
static int m4_user_wire_window_locked(struct m4_user_job *j, u32 engine, u32 limit,
	u32 producer, u32 pipe_producer, u32 count, u32 *queue_next, u32 *pipe_next)
{
	struct m4_gpu_provider_runtime *p = j->runtime;
	struct m4_ready_buffer *q = p->execution->queue[engine];
	void *pipe = p->ready->buffer[GPU_M4_OBJECT_COUNT + engine * 2].cpu;
	const struct m4_user_engine *tail, *oldest;
	struct m4_user_wire_ledger ledger;
	u32 start, pipe_start, span, pipe_span, consumer[4], next = 0, value;
	int ret;

	m4_user_wire_ledger(j, engine, limit, &ledger);
	tail = ledger.tail;
	oldest = ledger.oldest;
	if (tail && (producer != tail->queue_next || pipe_producer != tail->pipe_next))
		return M4_USER_WHY(-ESTALE, "wire window %u/%u producer=%u/%u tail %u/%u", engine, limit,
			producer, pipe_producer, tail->queue_next, tail->pipe_next);
	if (m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x40) != producer ||
	    m4_ready_read32(pipe + 0x20) != pipe_producer)
		return M4_USER_WHY(-ESTALE, "wire window %u/%u live producer=%u/%u want %u/%u", engine,
			limit, m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x40),
			m4_ready_read32(pipe + 0x20), producer, pipe_producer);
	start = oldest ? oldest->queue_before : producer;
	pipe_start = oldest ? oldest->pipe_before : pipe_producer;
	span = (producer - start) & 255;
	pipe_span = (pipe_producer - pipe_start) & 255;
	/* Strictly fewer than 128 outstanding entries, including the new ones. */
	if (span + count >= 128 || pipe_span + 1 >= 128)
		return M4_USER_WHY(-ENOSPC, "wire window %u/%u span=%u+%u pipe=%u", engine, limit, span,
			count, pipe_span);
	/* Without an owned outstanding command this is the exact idle rule;
	 * busy alone is no evidence while another job is in flight.
	 */
	if (!oldest && m4_ready_read32(q[M4_EXEC_QUEUE_INFO].cpu + 0x80) &&
	    !m4_user_wire_overlapped(j))
		return M4_USER_WHY(-EBUSY, "wire window %u/%u busy=%u without owned start", engine, limit,
			m4_ready_read32(q[M4_EXEC_QUEUE_INFO].cpu + 0x80));
	consumer[0] = m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x30);
	for (u32 i = 0; i < 3; i++)
		consumer[i + 1] = m4_ready_read32(q[M4_EXEC_QUEUE_INFO].cpu + 0x20 + 4 * i);
	for (u32 i = 0; i < 4; i++) {
		if (consumer[i] >= 256 || ((consumer[i] - start) & 255) > span)
			return M4_USER_WHY(-EBUSY, "wire window %u/%u consumer[%u]=%u outside %u..%u (oldest=%d)",
				engine, limit, i, consumer[i], start, producer, !!oldest);
		ret = gpu_m4_exec_ring_reserve(consumer[i], producer, count, &value);
		if (ret)
			return M4_USER_WHY(ret, "wire window %u/%u reserve consumer[%u]=%u producer=%u count=%u",
				engine, limit, i, consumer[i], producer, count);
		if (i && value != next)
			return M4_USER_WHY(-EPROTO, "wire window %u/%u next %u/%u", engine, limit, value, next);
		next = value;
	}
	for (u32 i = 0; i < 2; i++) {
		value = m4_ready_read32(pipe + 0x10 * i);
		if (value >= 256 || ((value - pipe_start) & 255) > pipe_span)
			return M4_USER_WHY(-EBUSY, "wire window %u/%u pipe[%u]=%u outside %u..%u", engine,
				limit, i, value, pipe_start, pipe_producer);
	}
	ret = gpu_m4_exec_pipe_reserve(m4_ready_read32(pipe), m4_ready_read32(pipe + 0x10),
		pipe_producer, &value);
	if (ret)
		return M4_USER_WHY(ret, "wire window %u/%u pipe reserve %u,%u,%u", engine, limit,
			m4_ready_read32(pipe), m4_ready_read32(pipe + 0x10), pipe_producer);
	*queue_next = next;
	*pipe_next = value;
	return 0;
}

/* inflight2 admission of not yet started job j beside the in-flight jobs:
 * entries more ring entries on engine (behind every reachable record, fewer
 * than 128 outstanding against all four consumers, the window rules of
 * publication) and records more submit-pipe records (fewer than 128
 * outstanding, room before READ and CFI). Live reads only; no write.
 * -ENOSPC/-EBUSY/-ESTALE: ring; -EMLINK: pipe.
 */
static int m4_user_wire_admit_locked(struct m4_user_job *j, u32 engine, u32 entries,
	u32 records)
{
	struct m4_gpu_provider_runtime *p = j->runtime;
	struct m4_user_wire_ledger ledger;
	u32 producer, pipe_producer, queue_next, pipe_next, pipe_start;
	void *pipe;
	int ret;

	if (!entries || !records || entries >= 128 || records >= 128)
		return -EINVAL;
	ret = m4_user_wire_queue_locked(j, engine);
	if (ret)
		return ret;
	pipe = p->ready->buffer[GPU_M4_OBJECT_COUNT + engine * 2].cpu;
	producer = m4_ready_read32(p->execution->queue[engine][M4_EXEC_QUEUE_STATE].cpu + 0x40);
	pipe_producer = m4_ready_read32(pipe + 0x20);
	ret = m4_user_wire_window_locked(j, engine, 0, producer, pipe_producer, entries,
		&queue_next, &pipe_next);
	if (ret)
		return ret;
	m4_user_wire_ledger(j, engine, 0, &ledger);
	pipe_start = ledger.oldest ? ledger.oldest->pipe_before : pipe_producer;
	if (((pipe_producer - pipe_start) & 255) + records >= 128)
		return -EMLINK;
	/* Each new record's producer must differ from READ and CFI. */
	for (u32 i = 0; i < 2; i++)
		if (((m4_ready_read32(pipe + 0x10 * i) - pipe_producer - 1) & 255) < records)
			return -EMLINK;
	return 0;
}

/* At most this many commands of one engine class share one packet. */
#define M4_USER_WIRE_PACKET_MAX 16U

/* Reserve one engine's entries for the next unpublished command. Its ends are
 * saved in the command's own record; nothing reachable is written here.
 *
 * join: the command continues the still open (installed, unreachable) packet
 * of its immediate predecessor on this engine. Its entries follow the
 * predecessor's saved end; the whole packet is validated against the live
 * window as ONE append of all its entries with ONE submit-pipe record, whose
 * producer is this (newest) member's end.
 */
static int __m4_user_wire_reserve_locked(struct m4_user_job *j, u32 index, u32 engine,
	const u64 *entries, u32 count, bool join)
{
	static const u8 event[] = { 0x70, 0x71, 15 };
	struct m4_gpu_provider_runtime *p = j->runtime;
	struct m4_ready_buffer *q;
	struct m4_user_engine next = { };
	struct gpu_m4_exec_pipe_record record;
	const struct m4_user_engine *head = NULL, *prev = NULL;
	u32 first = index, pending = 0;
	int ret;

	if (!entries || !count || count > ARRAY_SIZE(next.entries) || engine >= 3 ||
	    index >= j->command_count || index >= ARRAY_SIZE(j->cmd) ||
	    index != j->published_count || j->cmd[index].published || j->cmd[index].receipt)
		return M4_USER_WHY(-EINVAL, "wire reserve %u engine %u count=%u published=%u", index,
			engine, count, j->published_count);
	ret = m4_user_lane_check_locked(p, j->core_job);
	if (!ret)
		ret = m4_user_wire_queue_locked(j, engine);
	if (ret)
		return M4_USER_WHY(ret, "wire reserve %u engine %u lane/queue", index, engine);
	q = p->execution->queue[engine];
	if (join) {
		const struct m4_user_command *c = index ? &j->cmd[index - 1] : NULL;

		prev = c ? &c->wire[engine] : NULL;
		if (!c || !c->published || c->receipt || !(c->engine_mask & BIT(engine)) ||
		    !prev->count || prev->sequence || index != j->published_count)
			return M4_USER_WHY(-ESTALE, "wire join %u engine %u open=%d sequence=%llu",
				index, engine, c && c->published,
				M4_WHY64(prev ? prev->sequence : 0));
		first = index - 1;
		while (first && j->cmd[first].wire[engine].joined)
			first--;
		head = &j->cmd[first].wire[engine];
		if (head->joined || index - first >= M4_USER_WIRE_PACKET_MAX)
			return M4_USER_WHY(-E2BIG, "wire join %u engine %u first=%u", index, engine,
				first);
		pending = (prev->queue_next - head->queue_before) & 255;
		next.queue_before = prev->queue_next;
		next.pipe_before = head->pipe_before;
		next.joined = true;
	} else {
		next.queue_before = m4_ready_read32(q[M4_EXEC_QUEUE_STATE].cpu + 0x40);
		next.pipe_before = m4_ready_read32(p->ready->buffer[GPU_M4_OBJECT_COUNT + engine * 2].cpu + 0x20);
	}
	ret = m4_user_wire_window_locked(j, engine, first, head ? head->queue_before : next.queue_before,
		next.pipe_before, pending + count, &next.queue_next, &next.pipe_next);
	if (ret)
		return M4_USER_WHY(ret, "wire reserve %u engine %u window first=%u", index, engine, first);
	if (join && (next.queue_next != ((prev->queue_next + count) & 255) ||
	    next.pipe_next != prev->pipe_next))
		return M4_USER_WHY(-EPROTO, "wire join %u engine %u next %u/%u", index, engine,
			next.queue_next, next.pipe_next);
	for (u32 i = 0; i < count; i++) {
		if (!gpu_m4_exec_fw_range(entries[i], 4, 8))
			return M4_USER_WHY(-EINVAL, "wire reserve %u entry %#llx", index, M4_WHY64(entries[i]));
		next.entries[i] = entries[i];
	}
	next.count = count;
	next.new_queue = !(p->user_physical_used & BIT(engine));
	record = (struct gpu_m4_exec_pipe_record) {
		.timestamp = __arch_counter_get_cntpct(),
		.queue_info = q[M4_EXEC_QUEUE_INFO].va,
		.queue_producer = next.queue_next, .pipe = engine,
		.scheduler_slot = event[engine], .new_queue = next.new_queue,
	};
	ret = M4_USER_WHY(gpu_m4_exec_pipe_record(next.pipe_record, &record),
		"wire reserve %u engine %u pipe record", index, engine);
	if (!ret) {
		j->cmd[index].wire[engine] = next;
		j->cmd[index].engine_mask |= BIT(engine);
	}
	return ret;
}

static int m4_user_wire_reserve_locked(struct m4_user_job *j, u32 index, u32 engine,
	const u64 *entries, u32 count)
{
	return __m4_user_wire_reserve_locked(j, index, engine, entries, count, false);
}

/* Batched packet member: append behind the open packet's newest member. */
static int m4_user_wire_join_locked(struct m4_user_job *j, u32 index, u32 engine,
	const u64 *entries, u32 count)
{
	return __m4_user_wire_reserve_locked(j, index, engine, entries, count, true);
}

/* First command of the (open or published) packet that index ends on this
 * engine: walk back through joined records. Bounded by the packet size.
 */
static u32 m4_user_wire_packet_first(const struct m4_user_job *j, u32 index, u32 engine)
{
	u32 first = index;

	while (first && index - first < M4_USER_WIRE_PACKET_MAX &&
	       j->cmd[first].wire[engine].joined)
		first--;
	return first;
}

/* Final fallible edge for one engine of one command: its saved reservation
 * still extends exactly the owned producer, with valid credit against every
 * live consumer. Earlier commands are never reinterpreted by this check.
 * A batched member is checked as the newest end of its open packet: every
 * earlier member is installed, unreachable and contiguous, shares the one
 * pipe record, and the packet as a whole extends the owned producer.
 */
static int m4_user_wire_check_locked(struct m4_user_job *j, u32 index, u32 engine)
{
	struct m4_gpu_provider_runtime *p = j->runtime;
	const struct m4_user_engine *s, *head;
	u32 queue_next, pipe_next, first;
	int ret = m4_user_lane_check_locked(p, j->core_job);

	if (!ret && (engine >= 3 || index >= j->command_count || index >= ARRAY_SIZE(j->cmd) ||
	    index + 1 < j->published_count || index > j->published_count ||
	    j->cmd[index].receipt || !(j->cmd[index].engine_mask & BIT(engine))))
		ret = M4_USER_WHY(-EINVAL, "wire check %u engine %u published=%u receipt=%d engines=%#x",
			index, engine, j->published_count,
			index < ARRAY_SIZE(j->cmd) && j->cmd[index].receipt,
			index < ARRAY_SIZE(j->cmd) ? j->cmd[index].engine_mask : 0);
	if (!ret)
		ret = m4_user_wire_queue_locked(j, engine);
	if (ret)
		return M4_USER_WHY(ret, "wire check %u engine %u lane/queue", index, engine);
	s = &j->cmd[index].wire[engine];
	if (!s->count || s->count > ARRAY_SIZE(s->entries) ||
	    s->queue_next != ((s->queue_before + s->count) & 255) ||
	    s->pipe_next != ((s->pipe_before + 1) & 255) ||
	    s->new_queue != !(p->user_physical_used & BIT(engine)))
		return M4_USER_WHY(-ESTALE, "wire check %u engine %u record count=%u queue %u-%u pipe %u-%u new=%d used=%#x",
			index, engine, s->count, s->queue_before, s->queue_next, s->pipe_before,
			s->pipe_next, s->new_queue, p->user_physical_used);
	first = m4_user_wire_packet_first(j, index, engine);
	for (u32 m = index; m > first; m--) {
		const struct m4_user_command *c = &j->cmd[m - 1];
		const struct m4_user_engine *w = &j->cmd[m].wire[engine], *v = &c->wire[engine];

		if (!w->joined || !c->published || c->receipt || !(c->engine_mask & BIT(engine)) ||
		    v->sequence || !v->count || v->count > ARRAY_SIZE(v->entries) ||
		    v->queue_next != ((v->queue_before + v->count) & 255) ||
		    w->queue_before != v->queue_next || w->pipe_before != v->pipe_before ||
		    w->pipe_next != v->pipe_next || w->new_queue != v->new_queue ||
		    (m - 1 == first) == v->joined)
			return M4_USER_WHY(-ESTALE, "wire check %u engine %u member %u published=%d sequence=%llu queue %u-%u/%u pipe %u/%u joined=%d",
				index, engine, m - 1, c->published, M4_WHY64(v->sequence),
				v->queue_before, v->queue_next, w->queue_before, v->pipe_before,
				w->pipe_before, v->joined);
	}
	head = &j->cmd[first].wire[engine];
	if (head->joined)
		return M4_USER_WHY(-E2BIG, "wire check %u engine %u packet first=%u", index, engine, first);
	ret = m4_user_wire_window_locked(j, engine, first, head->queue_before, head->pipe_before,
		(s->queue_next - head->queue_before) & 255, &queue_next, &pipe_next);
	if (ret)
		return M4_USER_WHY(ret == -EBUSY || ret == -ENOSPC ? -ESTALE : ret,
			"wire check %u engine %u window %d first=%u", index, engine, ret, first);
	return queue_next == s->queue_next && pipe_next == s->pipe_next ? 0 :
		M4_USER_WHY(-ESTALE, "wire check %u engine %u next %u/%u want %u/%u", index, engine,
			queue_next, pipe_next, s->queue_next, s->pipe_next);
}

/* Caller marked this command published and installed every owned work, wait,
 * counter, UMA and threshold image BEFORE this call. All participating rings
 * and producers, then all submit-pipe records and producers, then one kick per
 * engine: fragment, TA, compute. No rollback after the first producer.
 *
 * index ends a packet: with batched (joined) members, every member's entries
 * are appended, each member gets its own ledger sequence in order, and the
 * packet's ONE pipe record per engine names the newest member's end, followed
 * by ONE kick per engine.
 */
static int m4_user_wire_publish_locked(struct m4_user_job *j, u32 index)
{
	static const u8 kick_order[] = { 1, 0, 2 };
	struct m4_gpu_provider_runtime *p = j->runtime;
	struct m4_user_command *c;
	u32 first, lowest;
	u64 ticks = 0;
	int ret = 0;

	if (index >= j->command_count || index >= ARRAY_SIZE(j->cmd))
		return -EINVAL;
	c = &j->cmd[index];
	if (!c->engine_mask || (c->engine_mask & ~7U))
		return -EINVAL;
	for (u32 i = 0; !ret && i < 3; i++)
		if (c->engine_mask & BIT(i))
			ret = m4_user_wire_check_locked(j, index, i);
	if (!ret && (!j->published || !c->published || index + 1 != j->published_count))
		ret = -EPERM;
	lowest = __builtin_ctz(c->engine_mask);
	first = m4_user_wire_packet_first(j, index, lowest);
	/* Every member participates on exactly the same engines, joined alike. */
	for (u32 m = first; !ret && m <= index; m++)
		for (u32 i = 0; !ret && i < 3; i++)
			if (j->cmd[m].engine_mask != c->engine_mask || !j->cmd[m].published ||
			    ((c->engine_mask & BIT(i)) &&
			     j->cmd[m].wire[i].joined != (m != first)))
				ret = -EPROTO;
	if (ret)
		return M4_USER_WHY(ret, "wire publish %u engines=%#x published=%u first=%u", index,
			c->engine_mask, j->published_count, first);
	for (u32 i = 0; i < 3; i++) {
		struct m4_ready_buffer *q = p->execution->queue[i];

		if (!(c->engine_mask & BIT(i)))
			continue;
		for (u32 m = first; m <= index; m++) {
			struct m4_user_engine *s = &j->cmd[m].wire[i];

			for (u32 n = 0; n < s->count; n++)
				put_unaligned_le64(s->entries[n], q[M4_EXEC_QUEUE_RING].cpu +
					((s->queue_before + n) & 255) * 8);
		}
	}
	dma_wmb();
	for (u32 i = 0; i < 3; i++)
		if (c->engine_mask & BIT(i)) {
			/* Reachable from here on: the ledger's next tail(s), in order. */
			for (u32 m = first; m <= index; m++)
				j->cmd[m].wire[i].sequence = ++p->user_wire_sequence[i];
			if (p->user_inflight2)
				memcpy(p->user_wire_last[i], &c->wire[i], sizeof(c->wire[i]));
			m4_ready_write32(p->execution->queue[i][M4_EXEC_QUEUE_STATE].cpu + 0x40,
				c->wire[i].queue_next);
		}
	for (u32 i = 0; i < 3; i++)
		if (c->engine_mask & BIT(i)) {
			struct m4_user_engine *s = &c->wire[i];

			/* Refresh only the raw clock field immediately before publication. */
			ticks = __arch_counter_get_cntpct();
			put_unaligned_le64(ticks, s->pipe_record);
			memcpy(p->ready->buffer[GPU_M4_OBJECT_COUNT + 2 * i + 1].cpu +
				s->pipe_before * 0x18, s->pipe_record, sizeof(s->pipe_record));
		}
	for (u32 m = first; m <= index; m++)
		j->cmd[m].publish_ticks = ticks; /* REMOVE-DEBUG timing */
	dma_wmb();
	for (u32 i = 0; i < 3; i++)
		if (c->engine_mask & BIT(i)) {
			m4_ready_write32(p->ready->buffer[GPU_M4_OBJECT_COUNT + 2 * i].cpu + 0x20,
				c->wire[i].pipe_next);
			p->user_physical_used |= BIT(i);
		}
	for (u32 n = 0; n < ARRAY_SIZE(kick_order); n++) {
		u32 i = kick_order[n];
		u64 kick;

		if (!(c->engine_mask & BIT(i)))
			continue;
		ret = gpu_m4_exec_pipe_kick(i, &kick);
		if (!ret)
			ret = apple_rtkit_send_message(p->hello->rtkit, 0x21, kick, NULL, false);
		if (ret)
			return M4_USER_WHY(ret, "wire publish %u kick engine %u", index, i);
	}
	return 0;
}

/* The owned physical horizon on one engine: the ledger's last reachable
 * record, of any in-flight job; the command's own record when nothing was
 * published behind it there. An unreceipted owned command must remain. The
 * notifier half of the horizon is the notifier's own published tail.
 */
static int m4_user_wire_horizon_locked(struct m4_user_job *j, u32 engine,
	const struct m4_user_engine **horizon)
{
	struct m4_user_wire_ledger ledger;
	int ret = m4_user_wire_queue_locked(j, engine);

	if (ret)
		return ret;
	if (!j->published_count || j->published_count > j->command_count ||
	    j->published_count > ARRAY_SIZE(j->cmd))
		return M4_USER_WHY(-ESTALE, "wire horizon %u published=%u/%u", engine,
			j->published_count, j->command_count);
	m4_user_wire_ledger(j, engine, j->published_count, &ledger);
	if (!ledger.tail || !ledger.oldest)
		return M4_USER_WHY(-ESTALE, "wire horizon %u tail=%d oldest=%d window=%u/%u", engine,
			!!ledger.tail, !!ledger.oldest, j->completed_count, j->published_count);
	*horizon = ledger.tail;
	return 0;
}

#endif
