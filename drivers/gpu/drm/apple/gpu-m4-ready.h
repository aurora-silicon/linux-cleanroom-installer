/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_READY_H
#define APPLE_GPU_M4_READY_H

/* Single experiment owner. No renderer, client contexts, teardown or live UAT. */
static u32 m4_ready_read32(const void *p)
{
	u32 value = le32_to_cpu(READ_ONCE(*(const __le32 *)p));

	dma_rmb();
	return value;
}

static void m4_ready_write32(void *p, u32 value)
{
	dma_wmb();
	WRITE_ONCE(*(__le32 *)p, cpu_to_le32(value));
	dma_wmb();
}

static int m4_provider_health(struct m4_gpu_hello *h);
static bool m4_ready_notify_light(struct m4_gpu_hello *h);
static u32 m4_pstate_sample(struct m4_gpu_hello *h, enum m4_pstate_site site);
static bool m4_provider_bound_image(struct m4_gpu_hello *h);
static u64 m4_provider_bound_word(struct m4_gpu_hello *h, u32 index);

/* Per-publication/poll READY health. Live hardware identity (RTKit, power,
 * TTBAT pairs, bound roots and provider binding epoch) is read every call;
 * table graphs use their live/cached-revision checks, not full inventories.
 * m4_ready_health_full() is the fault/diagnostic boundary variant.
 */
/* REMOVE-DEBUG: one-shot (per call site) attribution of a failing READY
 * health sub-check. Only reached on failure; the returned error is unchanged.
 */
#define M4_READY_WHY(h, err, fmt, ...) ({					\
	dev_warn_once((h)->gpu->dev, "REMOVE-DEBUG GPU READY health %d at gpu-m4-ready.h:%d " fmt "\n", \
		      (int)(err), __LINE__, ##__VA_ARGS__);			\
	(err);									\
})

static int m4_ready_health(struct m4_gpu_hello *h)
{
	struct m4_gpu_power *gpu = h->gpu;
	u64 low, root, at = ktime_get_ns();
	u32 asc, pstate;
	bool image;
	int ret;

	m4_ready_health_cost.passes++;
	h->ready->health_fresh = 0;
	if (atomic_read(&h->callback_error))
		return M4_READY_WHY(h, -EIO, "callback_error=%d", atomic_read(&h->callback_error));
	if (!apple_rtkit_is_running(h->rtkit) || apple_rtkit_is_crashed(h->rtkit))
		return M4_READY_WHY(h, -EIO, "rtkit running=%d crashed=%d",
				    apple_rtkit_is_running(h->rtkit), apple_rtkit_is_crashed(h->rtkit));
	asc = readl(gpu->asc + 0x44);
	pstate = m4_pstate_sample(h, M4_PSTATE_HEALTH);
	if (!(asc & BIT(4)) || pstate > h->ready->pstate.configured)
		return M4_READY_WHY(h, -EIO, "asc44=%#x pstate=%u/S%u", asc, pstate,
				    h->ready->pstate.configured);
	dcache_inval_poc((unsigned long)h->tables, (unsigned long)h->tables + 0x4000);
	h->ready->health_fresh = M4_READY_FRESH_ROOT;
	root = get_unaligned_le64(h->tables + 16);
	if (!gpu_m4_host_root_valid(root, h->region[M4_SHARED].start, true))
		return M4_READY_WHY(h, -EPROTO, "host root[2]=%#llx", root);
	m4_ready_cost(M4_READY_COST_FIXED, at);
	if (!m4_ready_tables_live(h))
		return M4_READY_WHY(h, -EPROTO, "live tables (see GPU READY tables line)");
	at = ktime_get_ns();
	low = readq(gpu->ttbs);
	root = readq(gpu->ttbs + 8);
	if (low != (page_to_phys(h->lower) | 1) || root != (h->region[M4_TABLES].start | 1))
		return M4_READY_WHY(h, -EPROTO, "ttbat0=%#llx/%#llx", low, root);
	if (h->ready->provider_runtime) {
		ret = m4_provider_health(h);
		if (ret)
			return M4_READY_WHY(h, -EPROTO, "provider health %d", ret);
	}
	at = m4_ready_cost(M4_READY_COST_PROVIDER, at);
	/* Every slot is read live and compared. The provider's expected image is
	 * a function of host owner state alone (selection, resident roots per
	 * ASID, binding epochs, errors), which nothing in this loop changes:
	 * decide its validity once, not per slot; no valid image fails at slot 2
	 * as before.
	 */
	image = !h->ready->provider_runtime || m4_provider_bound_image(h);
	for (unsigned int i = 2; i < 128; i++) {
		u64 value = readq(gpu->ttbs + i * 8);
		bool valid = !h->ready->provider_runtime ?
			m4_store_bound_root_valid(h, i, value) :
			image && value == m4_provider_bound_word(h, i);

		if (!valid)
			return M4_READY_WHY(h, -EPROTO, "bound root ttbs[%u]=%#llx", i, value);
	}
	m4_ready_cost(M4_READY_COST_TTBAT, at);
	return 0;
}

/* The bounded live service observation of m4_ready_health(): callback error,
 * RTKit running/not crashed, ASC RUN and the actual SGX state within the
 * configured S after any due thermal service. No table, root or TTBAT read.
 */
static int m4_ready_service_live(struct m4_gpu_hello *h)
{
	struct m4_gpu_power *gpu = h->gpu;
	u32 asc, pstate;

	if (atomic_read(&h->callback_error))
		return M4_READY_WHY(h, -EIO, "callback_error=%d", atomic_read(&h->callback_error));
	if (!apple_rtkit_is_running(h->rtkit) || apple_rtkit_is_crashed(h->rtkit))
		return M4_READY_WHY(h, -EIO, "rtkit running=%d crashed=%d",
				    apple_rtkit_is_running(h->rtkit), apple_rtkit_is_crashed(h->rtkit));
	asc = readl(gpu->asc + 0x44);
	pstate = m4_pstate_sample(h, M4_PSTATE_HEALTH);
	if (!(asc & BIT(4)) || pstate > h->ready->pstate.configured)
		return M4_READY_WHY(h, -EIO, "asc44=%#x pstate=%u/S%u", asc, pstate,
				    h->ready->pstate.configured);
	return 0;
}

/* Fault/diagnostic boundary: the live health above, then the complete READY
 * table scan including every upper lease's backing page/identity oracles.
 */
static int m4_ready_health_full(struct m4_gpu_hello *h)
{
	int ret = m4_ready_health(h);

	if (!ret && !m4_ready_tables_valid(h))
		ret = M4_READY_WHY(h, -EPROTO, "full tables (see GPU READY tables line)");
	return ret;
}

/* REMOVE-DEBUG: selected J713/25G83 bank, no selector/register writes.
 * Sample BEFORE the lengthy health/table walk. State 1 before AND after the
 * two reads is the qualified firmware recovery pre-stop window. A missed
 * window is recorded explicitly, not interpreted as a zero fault address.
 * Payload: u32 state-before/after, u64 info/address/capture-time.
 */
static bool m4_store_sample_fault(struct m4_gpu_hello *h, u8 data[0x20])
{
	struct m4_gpu_power *gpu = h->gpu;
	void *state = h->ready->input.object[GPU_M4_FIRMWARE_DATA] + 0x4fb0;
	u32 before, after;
	u64 info = 0, address = 0;

	if (!gpu->fault_bank || !gpu->voted || !gpu->firmware_start_attempted)
		return false;
	before = m4_ready_read32(state);
	if (before == 1) {
		info = readq(gpu->fault_bank + 0x18c0);
		address = readq(gpu->fault_bank + 0x18c8);
	}
	after = m4_ready_read32(state);
	put_unaligned_le32(before, data);
	put_unaligned_le32(after, data + 4);
	put_unaligned_le64(info, data + 8);
	put_unaligned_le64(address, data + 0x10);
	put_unaligned_le64(ktime_get_ns(), data + 0x18);
	return true;
}

/* Keep the first valid pre-stop pair before health/table walks or ring logging
 * can delay the type-0x16 consumer. This is an AP-owned diagnostic latch, not a
 * recovery acknowledgement or a replacement for the complete failure image.
 */
static void m4_store_latch_fault(struct m4_gpu_hello *h)
{
	struct m4_gpu_exec_memory *e = h->ready->execution;
	struct m4_gpu_store *s = e ? e->store : NULL;
	u8 data[0x20];

	if (!s || !s->prepared || !s->published || s->fault_sampled ||
	    s->evidence.captured)
		return;
	if (m4_store_sample_fault(h, data) && get_unaligned_le32(data) == 1 &&
	    get_unaligned_le32(data + 4) == 1 && (get_unaligned_le64(data + 8) & 1)) {
		memcpy(s->fault_image, data, sizeof(data));
		s->fault_sampled = true;
	}
}

/* REMOVE-DEBUG, ordinary runtime: keep the first qualified pre-stop sample
 * (state 1 before and after, valid bit) taken at any drain, independent of
 * the first-store experiment. Read-only SGX reads, no selector writes.
 */
static void m4_ready_fault_latch(struct m4_gpu_hello *h)
{
	struct m4_gpu_ready *r = h->ready;
	u8 data[0x20];

	if (r->fault_sampled || !m4_store_sample_fault(h, data))
		return;
	if (get_unaligned_le32(data) == 1 && get_unaligned_le32(data + 4) == 1 &&
	    (get_unaligned_le64(data + 8) & 1)) {
		memcpy(r->fault_sample, data, sizeof(data));
		r->fault_sampled = true;
	}
}

static const char *const m4_fault_reason[8] = {
	"unmapped", "af", "write-only", "read-only", "no-access", "r5", "r6", "r7",
};

static void m4_ready_fault_decode(struct m4_gpu_hello *h, const char *what, const u8 *data)
{
	u64 info = get_unaligned_le64(data + 8), address = get_unaligned_le64(data + 0x10);

	dev_err(h->gpu->dev,
		"GPU fault bank %s: state=%u/%u info=%#llx address=%#llx valid=%llu reason=%llu(%s) %s level=%llu unit=%#llx context=%llu sideband=%#llx bits6_5=%llu va=%#llx t=%llu\n",
		what, get_unaligned_le32(data), get_unaligned_le32(data + 4), info, address,
		info & 1, (info >> 1) & 7, m4_fault_reason[(info >> 1) & 7],
		(info >> 4) & 1 ? "read" : "write", (info >> 7) & 3, (info >> 9) & 0xff,
		(info >> 17) & 0x3f, (info >> 23) & 0x7f, (info >> 5) & 3, address << 6,
		get_unaligned_le64(data + 0x18));
}

/* REMOVE-DEBUG, read-only: log every change of the firmware-initiated
 * recovery phase (FirmwareData +0x4fb0: 0 running, 1 pre-recovery stop,
 * 2 host acknowledged, 3 post-recovery stop) and generation (+0x4fa0), with
 * a selected-bank sample while the phase is 1. Never writes the phase.
 */
static void m4_ready_recovery_watch(struct m4_gpu_hello *h)
{
	struct m4_gpu_ready *r = h->ready;
	void *fw = r->input.object[GPU_M4_FIRMWARE_DATA];
	u32 phase;
	u64 generation;
	u8 sample[0x20];

	if (!fw)
		return;
	phase = m4_ready_read32(fw + 0x4fb0);
	generation = (u64)m4_ready_read32(fw + 0x4fa0) |
		((u64)m4_ready_read32(fw + 0x4fa4) << 32);
	if (phase == r->recovery_phase && generation == r->recovery_generation)
		return;
	dev_err(h->gpu->dev, "GPU recovery phase %u -> %u generation %llu -> %llu t=%llu\n",
		r->recovery_phase, phase, r->recovery_generation, generation, ktime_get_ns());
	r->recovery_phase = phase;
	r->recovery_generation = generation;
	if (phase == 1 && m4_store_sample_fault(h, sample))
		m4_ready_fault_decode(h, "phase1", sample);
}

/* REMOVE-DEBUG fault report at a recovery-statistics or unexpected event
 * record: the latched and a fresh fault-bank sample, the firmware recovery
 * words (read-only), then the ordinary runtime's context table. Observes
 * only; the caller's error/latch behaviour is unchanged.
 */
static void m4_ready_fault_report(struct m4_gpu_hello *h, const char *kind, const u8 *record)
{
	struct m4_gpu_ready *r = h->ready;
	void *fw = r->input.object[GPU_M4_FIRMWARE_DATA];
	u32 size = gpu_m4_object_size[GPU_M4_FIRMWARE_DATA];
	u8 now[0x20];

	if (r->fault_reports >= 8)
		return;
	r->fault_reports++;
	dev_err(h->gpu->dev, "GPU fault report %u (%s): %*ph %*ph %*ph\n", r->fault_reports, kind,
		32, record, 32, record + 32, 8, record + 64);
	if (r->fault_sampled)
		m4_ready_fault_decode(h, "latched", r->fault_sample);
	if (m4_store_sample_fault(h, now))
		m4_ready_fault_decode(h, "now", now);
	dev_err(h->gpu->dev, "GPU fault fwdata +0x4fa0: %08x %08x +0x4fb0: %08x %08x %08x %08x +0x5190: %08x\n",
		m4_ready_read32(fw + 0x4fa0), m4_ready_read32(fw + 0x4fa4),
		m4_ready_read32(fw + 0x4fb0), m4_ready_read32(fw + 0x4fb4),
		m4_ready_read32(fw + 0x4fb8), m4_ready_read32(fw + 0x4fbc),
		size >= 0x5194 ? m4_ready_read32(fw + 0x5190) : 0xffffffff);
	if (h->fault_report)
		h->fault_report(h->fault_report_owner);
}

static int m4_store_capture_image(struct m4_gpu_hello *h, bool before,
				  int reason, const u8 *statistics)
{
	struct m4_gpu_ready *r = h->ready;
	struct m4_gpu_exec_memory *e = r->execution;
	struct m4_gpu_store *s = e ? e->store : NULL;
	struct gpu_m4_evidence *v;
	u32 phase = before ? GPU_M4_EVIDENCE_BEFORE : 0;
	u8 meta[0x60] = { 0 };
	u8 fault[0x20] = { 0 };
	bool sampled = false;
	int health;

	/* Existing owner mutex excludes AP consumers/publication. Firmware may
	 * still mutate RAM: this is evidence, not a coherent retirement snapshot.
	 */
	if (!s || !s->prepared || (!before && !s->published && !s->upper_attempted) ||
	    s->evidence.captured)
		return before ? -EINVAL : 0;
	v = &s->evidence;
	if ((!before && v->failure_started) || (before && v->size))
		return before ? (v->error ? v->error : -EBUSY) : v->error;
	if (!before && statistics && get_unaligned_le32(statistics) == 0x16) {
		if (s->fault_sampled) {
			memcpy(fault, s->fault_image, sizeof(fault));
			sampled = true;
		} else {
			sampled = m4_store_sample_fault(h, fault);
		}
	}
	health = m4_ready_health_full(h);
	if (!v->size && gpu_m4_evidence_begin(v, reason, health, ktime_get_ns()))
		return -EINVAL;
	put_unaligned_le32(reason, meta);
	put_unaligned_le32(health, meta + 4);
	put_unaligned_le64(ktime_get_ns(), meta + 8);
	put_unaligned_le64(readq(h->gpu->ttbs + 16), meta + 0x10);
	put_unaligned_le64(readq(h->gpu->ttbs + 24), meta + 0x18);
	if (statistics)
		gpu_m4_evidence_append(v, 1, statistics, 0x48, 0, 0, 0);
	if (sampled)
		gpu_m4_evidence_append(v, 2, fault, sizeof(fault), 0, 0,
			h->gpu->fault_bank_pa + 0x18c0);
	for (u32 channel = 0; channel < 3; channel++) {
		u32 id = channel == 0 ? GPU_M4_KTRACE_STATE :
			 channel == 1 ? GPU_M4_STATISTICS_STATE : GPU_M4_EVENT_STATE;
		u32 capacity = channel ? 256 : 512;
		void *state = r->input.object[id];
		u32 consumer = m4_ready_read32(state);
		u32 producer = m4_ready_read32(state + 0x20);
		u32 seen = channel == 0 ? r->trace_records :
			   channel == 1 ? r->statistics_records : r->event_records;
		u8 *indices = meta + 0x20 + channel * 0x14;

		put_unaligned_le32(consumer, indices);
		put_unaligned_le32(producer, indices + 4);
		gpu_m4_evidence_append(v, phase | (0xa0 + channel), state, 0x30,
			r->input.va[id], 0, r->buffer[id].pa);
		if (consumer < capacity && producer < capacity) {
			u32 pending = (producer + capacity - consumer) % capacity;
			u32 count = seen >= 16 || pending >= 16 ? 16 : min(16U, seen + pending);

			for (u32 i = 0; i < count; i++) {
				u8 record[0x50] = { 0 };
				u32 index = (producer + capacity - count + i) % capacity;

				put_unaligned_le32(index, record);
				memcpy(record + 8, r->input.object[id + 1] + index * 0x48, 0x48);
				gpu_m4_evidence_append(v, phase | (0x200 + channel * 0x100 + i),
					record, sizeof(record), r->input.va[id + 1] + index * 0x48,
					0, r->buffer[id + 1].pa + index * 0x48);
			}
		} else if (!v->error) {
			v->error = -EPROTO;
		}
		put_unaligned_le32(m4_ready_read32(state), indices + 8);
		put_unaligned_le32(m4_ready_read32(state + 0x20), indices + 0xc);
	}
	gpu_m4_evidence_append(v, phase | 0xff, meta, sizeof(meta), 0, 0, 0);
	dma_rmb();
	/* All pointers below come from retained owned allocator records. No
	 * recovered GPU/FW pointer is followed. The separately sampled fault bank
	 * above uses only its documented, already claimed read-only SGX window.
	 */
	for (u32 i = 0; i < 3; i++) {
		const u32 ids[] = { GPU_M4_ACTIVITY_RECOVERY, GPU_M4_FIRMWARE_DATA, GPU_M4_UMA };
		u32 id = ids[i];
		struct m4_ready_buffer *b = &r->buffer[id];

		u32 offset = id == GPU_M4_FIRMWARE_DATA ? 0x4ebc : 0;
		u32 size = id == GPU_M4_FIRMWARE_DATA ? 0x100 : gpu_m4_object_size[id];

		gpu_m4_evidence_append(v, phase | (3 + i), b->cpu + offset, size,
			b->va + offset, id == GPU_M4_UMA ? r->input.lower_uma : 0, b->pa + offset);
	}
	gpu_m4_evidence_append(v, phase | 6, e->notification.cpu, 0x1000,
		m4_exec_notification_fw(e), 0, e->notification.pa);
	if (s->upper_attempted) {
		u8 alias[0x80] = { 0 };

		put_unaligned_le64(s->upper_original, alias);
		put_unaligned_le64(s->upper_selected, alias + 8);
		put_unaligned_le64(e->notification.pa, alias + 0x10);
		put_unaligned_le64(s->upper_parent_before, alias + 0x18);
		put_unaligned_le64(s->upper_parent_after, alias + 0x20);
		put_unaligned_le64(s->upper_leaf_before, alias + 0x28);
		put_unaligned_le64(s->upper_leaf_after, alias + 0x30);
		put_unaligned_le64(s->upper_started, alias + 0x38);
		put_unaligned_le64(s->upper_finished, alias + 0x40);
		put_unaligned_le32(s->upper_mode, alias + 0x48);
		put_unaligned_le32(s->upper_middle, alias + 0x4c);
		put_unaligned_le32(s->upper_index, alias + 0x50);
		put_unaligned_le32(s->upper_handoff_get, alias + 0x54);
		put_unaligned_le32(s->upper_handoff_put, alias + 0x58);
		put_unaligned_le32(s->upper_tlbi, alias + 0x5c);
		put_unaligned_le32(s->upper_error, alias + 0x60);
		put_unaligned_le32(s->upper_new_child, alias + 0x64);
		gpu_m4_evidence_append(v, phase | 0xd0, alias, sizeof(alias), 0, 0, 0);
		gpu_m4_evidence_append(v, phase | 0xd1, e->notification.cpu, 0x1000,
			s->upper_original, 0, e->notification.pa);
		if (s->upper_child.cpu) {
			if (s->upper_parent_after || !s->upper_new_child)
				dcache_inval_poc((unsigned long)s->upper_child.cpu,
					(unsigned long)s->upper_child.cpu + 0x4000);
			gpu_m4_evidence_append(v, phase | 0xd2, s->upper_child_before,
				0x4000, 0, 0, s->upper_child.pa);
			gpu_m4_evidence_append(v, phase | 0xd3, s->upper_child.cpu,
				0x4000, 0, 0, s->upper_child.pa);
		}
	}
	gpu_m4_evidence_append(v, phase | 7, r->buffer[GPU_M4_ACTIVITY_COMPUTE].cpu,
		0x40, r->input.va[GPU_M4_ACTIVITY_COMPUTE], 0,
		r->buffer[GPU_M4_ACTIVITY_COMPUTE].pa);
	gpu_m4_evidence_append(v, phase | 8, e->context.cpu, 0x38,
		e->context.va, 0, e->context.pa);
	/* Private payload pages need not be copied, but retain the allocator's
	 * immutable PA inventory so a table walk can prove each pool leaf owner.
	 * This is owned AP memory, not a pointer recovered from firmware.
	 */
	gpu_m4_evidence_append(v, phase | 9, e->pool_pa, sizeof(e->pool_pa), 0, 0, 0);
	for (u32 i = 0; i < M4_EXEC_BUFFERS; i++) {
		struct m4_ready_buffer *b = &e->buffer[i];
		u32 size = i == M4_EXEC_CONTEXT_STORE ? 0x4000 : m4_exec_buffer_size[i];

		/* Pool content isn't required; keep work/code/output/private descriptor
		 * and freelist/list/counter exactly as firmware left them.
		 */
		gpu_m4_evidence_append(v, phase | (0x10 + i), b->cpu, size,
			b->va, m4_exec_buffer_gpu[i], b->pa);
	}
	/* Additional context-store pages contain the qualified recovery offsets;
	 * retain full owned pages without assigning meanings to unknown words.
	 */
	for (u32 i = 0; i < 2; i++) {
		u32 offset = i ? 0x1c000 : 0xc000;
		struct m4_ready_buffer *b = &e->buffer[M4_EXEC_CONTEXT_STORE];

		gpu_m4_evidence_append(v, phase | (0xc0 + i), b->cpu + offset, 0x4000,
			0, m4_exec_buffer_gpu[M4_EXEC_CONTEXT_STORE] + offset, b->pa + offset);
	}
	for (u32 i = 0; i < M4_EXEC_QUEUE_BUFFERS; i++) {
		struct m4_ready_buffer *b = &e->queue[GPU_M4_EXEC_COMPUTE][i];

		gpu_m4_evidence_append(v, phase | (0x30 + i), b->cpu,
			m4_exec_queue_size[i], b->va, 0, b->pa);
	}
	for (u32 i = 0; i < 2; i++) {
		struct m4_ready_buffer *b = &r->buffer[GPU_M4_OBJECT_COUNT + 4 + i];

		gpu_m4_evidence_append(v, phase | (0x40 + i), b->cpu,
			i ? 0x1800 : 0x30, b->va, 0, b->pa);
	}
	/* Health already invalidated/checked these AP-owned WB table pages.
	 * Capture only used client pages plus the bootstrap lower command leaf.
	 */
	for (u32 i = 0; i < min(e->client.used, (u32)M4_EXEC_TABLES); i++) {
		/* These AP-only pages are already clean; a failed health check may
		 * have stopped before their invalidation. Never repair a descriptor.
		 */
		dcache_inval_poc((unsigned long)e->table[i].cpu,
				 (unsigned long)e->table[i].cpu + 0x4000);
		gpu_m4_evidence_append(v, phase | (0x50 + i), e->table[i].cpu,
			0x4000, 0, 0, e->table[i].pa);
		gpu_m4_evidence_append(v, phase | (0x80 + i), e->expected[i],
			0x4000, 0, 0, e->table[i].pa);
	}
	for (u32 i = 0; i < M4_READY_TABLES; i++)
		gpu_m4_evidence_append(v, phase | (0x70 + i), page_address(r->table[i]),
			0x4000, 0, 0, page_to_phys(r->table[i]));
	gpu_m4_evidence_append(v, phase | 0x74, h->tables, 0x4000, 0, 0,
		h->region[M4_TABLES].start);
	gpu_m4_evidence_append(v, phase | 0x75, h->shared, 0x4000, 0, 0,
		h->region[M4_SHARED].start);
	gpu_m4_evidence_append(v, phase | 0x76, page_address(h->lower),
		0x4000, 0, 0, page_to_phys(h->lower));
	if (!before) {
		put_unaligned_le32(reason, v->data + 0x14);
		v->failure_started = true;
	}
	return v->error ? v->error : health;
}

static int m4_store_capture_before(struct m4_gpu_hello *h)
{
	return m4_store_capture_image(h, true, 0, NULL);
}

static void m4_store_seal_evidence(struct m4_gpu_hello *h)
{
	struct m4_gpu_exec_memory *e = h->ready->execution;
	struct gpu_m4_evidence *v;

	if (!e || !e->store)
		return;
	v = &e->store->evidence;
	if (!v->size || !v->failure_started || v->captured)
		return;
	gpu_m4_evidence_finish(v);
	dev_info(h->gpu->dev, "GPU first-store evidence retained: %u bytes, %u owned records, capture=%d\n",
		 v->size, v->records, v->error);
}

static void m4_store_capture_evidence(struct m4_gpu_hello *h, int reason, const u8 *statistics)
{
	m4_store_capture_image(h, false, reason, statistics);
	if (h->ready->execution && h->ready->execution->store) {
		struct gpu_m4_evidence *v = &h->ready->execution->store->evidence;

		/* A failed pre-submit capture also deserves an immutable raw file. */
		if (v->size && !v->captured) {
			v->failure_started = true;
			put_unaligned_le32(reason, v->data + 0x14);
		}
	}
	m4_store_seal_evidence(h);
}

static void m4_store_capture_event(struct m4_gpu_hello *h, u32 index,
				   u32 producer, const u8 *record)
{
	struct m4_gpu_exec_memory *e = h->ready->execution;
	struct gpu_m4_evidence *v;
	u8 data[0x58] = { 0 };

	if (!e || !e->store)
		return;
	v = &e->store->evidence;
	if (!v->failure_started || v->captured)
		return;
	put_unaligned_le32(index, data);
	put_unaligned_le32(producer, data + 4);
	put_unaligned_le32((index + 1) & 255, data + 8);
	memcpy(data + 0x10, record, 0x48);
	gpu_m4_evidence_append(v, 0x800 + index, data, sizeof(data), 0, 0, 0);
}

static int m4_ready_drain_once(struct m4_gpu_hello *h)
{
	struct m4_gpu_ready *r = h->ready;
	void **object = r->input.object;
	u8 record[0x48];
	u32 consumer, producer, type;
	bool unexpected_event_logged = false;
	int error = 0;

	m4_store_latch_fault(h);
	m4_ready_fault_latch(h);
	m4_ready_recovery_watch(h);
	for (unsigned int thread = 0; thread < 9; thread++) {
		void *state = object[GPU_M4_LOG_STATE] + thread * 0x30;

		consumer = m4_ready_read32(state);
		producer = m4_ready_read32(state + 0x20);
		if (consumer >= 256 || producer >= 256)
			return M4_READY_WHY(h, -EPROTO, "drain log%u indices %u/%u", thread, consumer, producer);
		/* Snapshot producer: at most 255 records per ring in one drain. */
		while (consumer != producer) {
			u32 slot;
			char text[201];

			memcpy(record, object[GPU_M4_LOG_ENTRIES] +
				(thread * 256 + consumer) * 0x48, sizeof(record));
			slot = get_unaligned_le32(record + 8) & 0xff;
			memcpy(text, object[GPU_M4_LOG_DATA] +
				(thread * 256 + slot) * 0xd8 + 0x10, 200);
			text[200] = 0;
			if (r->log_records++ < 16 || firmware_logging)
				dev_info(h->gpu->dev, "GPU firmware thread%u: %s\n", thread, text);
			else
				dev_dbg(h->gpu->dev, "GPU firmware thread%u: %s\n", thread, text);
			consumer = (consumer + 1) & 255;
			m4_ready_write32(state, consumer);
		}
	}
	for (unsigned int channel = 0; channel < 3; channel++) {
		unsigned int state_id = channel == 0 ? GPU_M4_KTRACE_STATE :
			channel == 1 ? GPU_M4_STATISTICS_STATE : GPU_M4_EVENT_STATE;
		void *state = object[state_id];
		u32 capacity = channel ? 256 : 512;

		consumer = m4_ready_read32(state);
		producer = m4_ready_read32(state + 0x20);
		if (consumer >= capacity || producer >= capacity)
			return M4_READY_WHY(h, -EPROTO, "drain channel%u indices %u/%u", channel, consumer, producer);
		while (consumer != producer) {
			memcpy(record, object[state_id + 1] + consumer * 0x48, sizeof(record));
			type = get_unaligned_le32(record);
			if (!channel) {
				if (type != 5) {
					dev_err(h->gpu->dev, "GPU unknown trace record: %*ph\n", 72, record);
					return M4_READY_WHY(h, -EPROTO, "drain trace type %#x", type);
				}
				if ((get_unaligned_le32(record + 0x2c) & 0xffffff) == 0x11c) {
					u64 lost = get_unaligned_le64(record + 0x0c);

					r->trace_dropped_records++;
					r->trace_lost_records = lost > ~0ULL - r->trace_lost_records ?
						~0ULL : r->trace_lost_records + lost;
				}
				if (r->trace_records++ < 8)
					dev_info(h->gpu->dev, "GPU trace type=%#x tag=%#x phase=%#x\n",
						 type, get_unaligned_le32(record + 0x2c),
						 get_unaligned_le32(record + 0x30));
			} else if (channel == 1) {
				r->statistics_records++;
				if (type == 0x16) {
					m4_ready_fault_latch(h);
					m4_ready_fault_report(h, "recovery statistics", record);
					m4_store_capture_image(h, false, -EIO, record);
					/* %*ph caps each chunk at 64 bytes: retain all 72. */
					dev_err(h->gpu->dev, "GPU recovery statistics: %*ph %*ph %*ph\n",
						32, record, 32, record + 32, 8, record + 64);
					error = -EIO;
				}
			} else {
				r->event_records++;
				if (type != 1) {
					m4_ready_fault_latch(h);
					m4_ready_fault_report(h, "event", record);
					m4_store_capture_event(h, consumer, producer, record);
					if (!unexpected_event_logged) {
						dev_err(h->gpu->dev, "GPU unexpected event indices: consumer=%u producer=%u\n",
							consumer, producer);
						dev_err(h->gpu->dev, "GPU unexpected event record: %*ph %*ph %*ph\n",
							32, record, 32, record + 32, 8, record + 64);
						unexpected_event_logged = true;
					}
					if (!error)
						error = -EPROTO;
				}
			}
			consumer = (consumer + 1) % capacity;
			m4_ready_write32(state, consumer);
		}
	}
	m4_store_seal_evidence(h);
	if (error)
		M4_READY_WHY(h, error, "drain %s record (logged above)",
			     error == -EIO ? "recovery statistics" : "unexpected event");
	return error;
}

static int m4_ready_pending(struct m4_gpu_hello *h)
{
	void **object = h->ready->input.object;
	bool pending = false;

	for (unsigned int ring = 0; ring < 12; ring++) {
		void *state;
		u32 consumer, producer, capacity = ring == 9 ? 512 : 256;

		if (ring < 9)
			state = object[GPU_M4_LOG_STATE] + ring * 0x30;
		else
			state = object[ring == 9 ? GPU_M4_KTRACE_STATE :
				       ring == 10 ? GPU_M4_STATISTICS_STATE : GPU_M4_EVENT_STATE];
		consumer = m4_ready_read32(state);
		producer = m4_ready_read32(state + 0x20);
		if (consumer >= capacity || producer >= capacity)
			return M4_READY_WHY(h, -EPROTO, "pending ring%u indices %u/%u", ring, consumer, producer);
		pending |= consumer != producer;
	}
	return pending;
}

static int m4_ready_drain(struct m4_gpu_hello *h)
{
	int ret;

	/* Recheck every producer after the bounded snapshot drain. A notification
	 * racing the drain can add records, and notifications are not record counts.
	 * Two passes bound this invocation; the periodic owner handles later input.
	 */
	for (unsigned int pass = 0; pass < 2; pass++) {
		ret = m4_ready_drain_once(h);
		if (ret)
			return ret;
		ret = m4_ready_pending(h);
		if (ret <= 0)
			return ret;
	}
	return 0;
}

/* A notification wake never queues behind an owner mutex that is already
 * held, typically for a whole lane start/packet build: while the lane has an
 * item its own retirement service drains these rings and checks READY health
 * under that mutex. Retry one tick later instead, at most this many times in
 * a row, then wait for it: the periodic owner is never starved.
 */
#define M4_READY_DRAIN_DEFERRALS 4U

static void m4_ready_drain_work(struct work_struct *work)
{
	struct m4_gpu_ready *r = container_of(to_delayed_work(work), struct m4_gpu_ready, drain_work);
	struct m4_gpu_hello *h = r->hello;
	struct mutex *lock = &h->gpu->lock;
	u64 start;

	if (!mutex_trylock(lock)) {
		/* Only this non-reentrant worker touches drain_deferrals. */
		if (r->drain_deferrals < M4_READY_DRAIN_DEFERRALS) {
			r->drain_deferrals++;
			WRITE_ONCE(r->drain_deferred, r->drain_deferred + 1); /* REMOVE-DEBUG */
			/* Never replaces an already pending immediate notification. */
			queue_delayed_work(system_dfl_wq, &r->drain_work, 1);
			return;
		}
		start = ktime_get_ns();
		mutex_lock(lock);
		/* REMOVE-DEBUG: this worker's wait for the owner mutex. */
		start = ktime_get_ns() - start;
		r->drain_waits++;
		r->drain_wait_ns += start;
		r->drain_wait_max = max(r->drain_wait_max, start);
	}
	r->drain_deferrals = 0;
	if (r->submitted && !r->error) {
		/* Notification wakes use the cached live health (the ordinary
		 * runtime with health_per_job=0: only the bounded live service,
		 * its lane worker snapshots its own); a full table/upper
		 * diagnostic scan still runs at most once per second.
		 */
		const char *phase; /* REMOVE-DEBUG attribution */

		if (time_after_eq(jiffies, r->full_health_at)) {
			phase = "full health";
			r->error = m4_ready_health_full(h);
			r->full_health_at = jiffies + HZ;
		} else if (m4_ready_notify_light(h)) {
			phase = "live service";
			r->error = m4_ready_service_live(h);
		} else {
			phase = "live health";
			r->error = m4_ready_health(h);
		}
		if (!r->error) {
			phase = "drain";
			r->error = m4_ready_drain(h);
		}
		/* Periodic fallback never postpones an immediate notification. */
		if (!r->error)
			queue_delayed_work(system_dfl_wq, &r->drain_work, msecs_to_jiffies(100));
		else
			dev_err(h->gpu->dev, "GPU retained READY owner fault %d (%s); no further submission\n",
				r->error, phase);
	}
	mutex_unlock(lock);
}

static void m4_ready_notify(struct m4_gpu_hello *h)
{
	atomic_inc(&h->ready->notifications);
	/* Never wait on the RTKit receive worker. Driver mutex serializes consumers.
	 * IRQ-safe: queue-only wakes of the READY drain and the ordinary lane.
	 */
	mod_delayed_work(system_dfl_wq, &h->ready->drain_work, 0);
	gpu_m4_drm_provider_lane_notify(smp_load_acquire(&h->notify_lane));
}

static int m4_ready_run(struct m4_gpu_hello *h)
{
	struct m4_gpu_ready *r = h->ready;
	struct device *dev = h->gpu->dev;
	void **object = r->input.object;
	void *state = object[GPU_M4_DEVCTRL_STATE];
	u64 deadline, stable = 0;
	u32 ready, rd, cfi, wr;
	int ret;

	ret = m4_ready_health(h);
	if (ret)
		return ret;
	WRITE_ONCE(r->applications, true);
	ret = apple_rtkit_start_ep(h->rtkit, 0x20);
	if (ret)
		return ret;
	ret = apple_rtkit_start_ep(h->rtkit, 0x21);
	if (ret)
		return ret;
	for (unsigned int i = 0; i < 10; i++) {
		msleep(10);
		ret = m4_ready_health(h);
		if (ret)
			return ret;
	}
	writel(0, r->generation);
	if (readl(r->generation))
		return -EIO;
	if (m4_ready_read32(state) || m4_ready_read32(state + 0x10) ||
	    m4_ready_read32(state + 0x20))
		return -EPROTO;
	put_unaligned_le32(0x1a, object[GPU_M4_DEVCTRL_RING]);
	dma_wmb();
	writel(1, r->generation);
	if (readl(r->generation) != 1)
		return -EIO;
	m4_ready_write32(state + 0x20, 1);
	for (unsigned int pass = 0; pass < 2; pass++) {
		for (unsigned int i = 0; i < GPU_M4_LOADER_COUNT * GPU_M4_LOADER_STRIDE; i += 4)
			if (readl(r->loader + i))
				return -EBUSY;
		dma_rmb();
	}
	ret = m4_ready_health(h);
	if (ret)
		return ret;
	memcpy_toio(r->loader, object[GPU_M4_MAIN] + 0x640,
		    GPU_M4_LOADER_COUNT * GPU_M4_LOADER_STRIDE);
	dma_wmb();
	r->submitted = true;
	ret = apple_rtkit_send_message(h->rtkit, 0x20,
		0x0081000000000000ULL | (r->input.va[GPU_M4_ROOT] & GENMASK_ULL(43, 0)), NULL, false);
	if (ret)
		return ret;
	ret = apple_rtkit_send_message(h->rtkit, 0x21, 0x0083000000000011ULL, NULL, false);
	if (ret)
		return ret;
	dev_info(dev, "GPU initdata submitted root=%#llx, S%u, generation1 NOP; no GPU jobs\n",
		 r->input.va[GPU_M4_ROOT], r->pstate.configured);
	deadline = ktime_get_ns() + 5 * NSEC_PER_SEC;
	do {
		ret = m4_ready_health(h);
		if (ret)
			return ret;
		ret = m4_ready_drain(h);
		if (ret)
			return ret;
		ready = m4_ready_read32(object[GPU_M4_CONTROL] + 0x14);
		rd = m4_ready_read32(state);
		cfi = m4_ready_read32(state + 0x10);
		wr = m4_ready_read32(state + 0x20);
		if (ready > 1 || rd > 1 || cfi > 1 || wr != 1)
			return -EPROTO;
		if (ready == 1 && rd == 1 && cfi == 1) {
			if (!stable)
				stable = ktime_get_ns();
			if (ktime_get_ns() - stable >= 50 * NSEC_PER_MSEC) {
				r->complete = true;
				mod_delayed_work(system_dfl_wq, &r->drain_work, msecs_to_jiffies(100));
				dev_info(dev, "GPU initdata accepted, NOP retired [1,1,1] stable50ms; NOT policy-ready\n");
				return 0;
			}
		} else {
			stable = 0;
		}
		usleep_range(1000, 2000);
	} while (ktime_get_ns() < deadline);
	dev_err(dev, "GPU initdata timeout ready=%u NOP=[%u,%u,%u] callbacks=%d\n",
		ready, rd, cfi, wr, atomic_read(&h->callback_error));
	return -ETIMEDOUT;
}

static ssize_t ready_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_hello *h;
	struct m4_gpu_ready *r;
	ssize_t size;

	mutex_lock(&gpu->lock);
	h = gpu->hello;
	r = h ? h->ready : NULL;
	if (!r) {
		size = sysfs_emit(buf, "attempted=0\n");
	} else {
		void *state = r->input.object[GPU_M4_DEVCTRL_STATE];
		void *control = r->input.object[GPU_M4_CONTROL];

		size = sysfs_emit(buf, "attempted=1 prepared=%u published=%u applications=%u submitted=%u complete=%u error=%d callback_error=%d pstate=%u ready=%u nop=%u,%u,%u notifications=%d logs=%u traces=%u statistics=%u events=%u policy_ready=%u\n",
			r->prepared, r->published, r->applications, r->submitted, r->complete,
			r->error, atomic_read(&h->callback_error), readl(gpu->sgx_core + 0x1000) & 0xf,
			control ? m4_ready_read32(control + 0x14) : 0,
			state ? m4_ready_read32(state) : 0,
			state ? m4_ready_read32(state + 0x10) : 0,
			state ? m4_ready_read32(state + 0x20) : 0, atomic_read(&r->notifications),
			r->log_records, r->trace_records, r->statistics_records, r->event_records,
			r->policy_complete);
	}
	mutex_unlock(&gpu->lock);
	return size;
}
static DEVICE_ATTR_RO(ready_state);

static ssize_t ready_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_hello *h;
	int ret = -EINVAL;

	if (!hello_enable || !ready_enable || !sysfs_streq(buf, "1"))
		return -EINVAL;
	mutex_lock(&gpu->lock);
	if (gpu->hello || gpu->error || !gpu->voted || !gpu->stopped_verified)
		goto out;
	h = devm_kzalloc(dev, sizeof(*h), GFP_KERNEL);
	if (!h) {
		ret = -ENOMEM;
		goto out;
	}
	h->ready = kvzalloc(sizeof(*h->ready), GFP_KERNEL);
	if (!h->ready) {
		ret = -ENOMEM;
		goto out;
	}
	h->ready->hello = h;
	INIT_DELAYED_WORK(&h->ready->drain_work, m4_ready_drain_work);
	h->gpu = gpu;
	init_completion(&h->crash_ready);
	h->graph_diagnostic = -ENODATA;
	gpu->hello = h;
	ret = m4_hello_start(h);
	h->error = ret;
	if (!ret)
		ret = m4_ready_run(h);
	h->ready->error = ret;
	dev_info(dev, "GPU initdata/NOP trial complete=%u error=%d; all owners retained, no policy or GPU jobs\n",
		 h->ready->complete, ret);
out:
	mutex_unlock(&gpu->lock);
	return ret ? ret : (ssize_t)count;
}
static DEVICE_ATTR_WO(ready);

#include "gpu-m4-ready-pstate.h"
#include "gpu-m4-ready-policy.h"
#include "gpu-m4-exec-runtime.h"
#include "gpu-m4-upper-runtime.h"
#include "gpu-m4-store-runtime.h"
#include "gpu-m4-provider-runtime.h"
#include "gpu-m4-provider-work.h"
#include "gpu-m4-provider-ts-upper.h"
#include "gpu-m4-provider-ts-work.h"
#include "gpu-m4-provider-users.h"
#include "gpu-m4-user-job.h"
#include "gpu-m4-user-lane.h"
#include "gpu-m4-user-native.h"
#include "gpu-m4-user-compute.h"
#include "gpu-m4-user-render.h"
#include "gpu-m4-user-wire.h"
#include "gpu-m4-user-submit-render.h"
#include "gpu-m4-user-probe.h"
#include "gpu-m4-user-runtime.h"

#endif
