/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_COMPUTE_H
#define APPLE_GPU_M4_COMPUTE_H

#include "gpu-m4-exec.h"

/* Detached, serialized second-owner preparation. This is not a submitter.
 * The original owner, live per-client notifier and firmware UMA table are
 * never modified here. No default admits a hardware UMA slot or owner lease.
 */
enum gpu_m4_compute_resource_id {
	GPU_M4_COMPUTE_WORK,
	GPU_M4_COMPUTE_PREEMPT,
	GPU_M4_COMPUTE_CDM,
	GPU_M4_COMPUTE_AUX,
	GPU_M4_COMPUTE_CONTEXT_STORE,
	GPU_M4_COMPUTE_POOL_STATE,
	GPU_M4_COMPUTE_PAGE_LIST,
	GPU_M4_COMPUTE_POOL,
	GPU_M4_COMPUTE_OUTPUT,
	GPU_M4_COMPUTE_SHADER,
	GPU_M4_COMPUTE_PROGRAM,
	GPU_M4_COMPUTE_RESOURCE_COUNT,
};

struct gpu_m4_compute_previous {
	const void *owner;
	struct gpu_m4_exec_retirement retirement;
	u32 output;
	u32 expected_output;
	struct gpu_m4_exec_interval client[GPU_M4_COMPUTE_RESOURCE_COUNT];
	/* First owner's whole retained work, auxiliary and counter allocations. */
	struct gpu_m4_exec_interval firmware[3];
	u64 queue_fw;
	u64 notifier_page_fw;
	u64 stats_fw;
	u32 uma_slot;
	u8 asid;
	u8 binding_generation;
};

struct gpu_m4_compute_input;

/* Under the caller's VM/queue owner lock, prove: previous owner remains
 * pinned and fully retired; new owner and scratch/output storage are fresh,
 * private, unqueued and physically disjoint from every retained owner;
 * the full work allocation's firmware/context0/client aliases and auxiliary
 * aliases resolve to the same owned backing with qualified permissions;
 * ASID1/generation1 and queue/stats/notifier identities are still installed;
 * the global UMA table's bootstrap/client aliases remain exact; and the
 * hardware slot/event reservation is a source-admitted free lease. Recheck
 * every retirement field and first output against live storage; copied
 * completion values alone cannot reserve a queue or prove retirement. The
 * client notifier threshold is still 1, its stamps are still 0x100, and no
 * other prepared or retained owner owns UMA slot 2 or event slot 0. A positive
 * return is not proof. This callback never publishes mappings or thresholds.
 */
typedef int (*gpu_m4_compute_check_lease)(void *cookie,
				       const struct gpu_m4_compute_input *input,
				       const void *output, const void *scratch);

struct gpu_m4_compute_input {
	struct gpu_m4_exec_compute_input work;
	const void *owner;
	const struct gpu_m4_compute_previous *previous;
	struct gpu_m4_exec_interval client[GPU_M4_COMPUTE_RESOURCE_COUNT];
	/* New whole firmware allocations; third is the separate counter page. */
	struct gpu_m4_exec_interval firmware[3];
	u64 notifier_page_fw;
	u32 uma_slot;
	u32 target_stamp;
	u32 threshold_to_publish;
	u8 event_slot;
	u8 asid;
	void *cookie;
	gpu_m4_compute_check_lease check_lease;
};

/* Values for the locked, separately qualified submitter. Producing this plan
 * never updates a live notifier, UMA table, counter, ring or pipe. The caller
 * must retain the lease, privately prepare counter=1 and finish all fallible
 * visibility checks before: threshold=2, complete ring entry, queue producer=2,
 * complete pipe record, pipe producer=2, then kick. NC barriers are external.
 */
struct gpu_m4_compute_publication {
	u64 work_fw;
	u64 counter_fw;
	u64 counter_value;
	u64 threshold_fw;
	u64 threshold_before;
	u64 threshold_value;
	u64 kick;
	u32 queue_next;
	u32 pipe_next;
	u32 uma_slot;
	u32 event_sequence;
	u32 target_stamp;
	u8 pipe_record[0x18];
};

static inline bool gpu_m4_compute_overlap(u64 a, u64 na, u64 b, u64 nb)
{
	return na && nb && na <= ~0ULL - a && nb <= ~0ULL - b &&
		a < b + nb && b < a + na;
}

static inline int gpu_m4_compute_validate(const struct gpu_m4_compute_input *in,
					const void *output, const void *scratch)
{
	static const u64 size[GPU_M4_COMPUTE_RESOURCE_COUNT] = {
		0x4000, 0x4000, 0x4000, 0x4000, 0x40000, 0x20000, 0x4000,
		GPU_M4_EXEC_POOL_SIZE, 0x4000, 0x4000, 0x4000,
	};
	const struct gpu_m4_exec_compute_input *w;
	const struct gpu_m4_compute_previous *p;
	struct gpu_m4_exec_interval firmware[9];
	u64 expected[GPU_M4_COMPUTE_RESOURCE_COUNT];
	unsigned int i, j;
	int ret;

	if (!in || !output || !scratch || !in->owner || !in->previous ||
	    !in->previous->owner || in->owner == in->previous->owner || !in->check_lease ||
	    gpu_m4_compute_overlap((unsigned long)output, GPU_M4_EXEC_COMPUTE_SIZE,
				   (unsigned long)scratch, GPU_M4_EXEC_COMPUTE_SIZE))
		return -EINVAL;
	w = &in->work;
	p = in->previous;
	/* This is the bounded second owner, not arbitrary queue reuse. Sequence,
	 * notification threshold and per-owner UMA count remain separate values.
	 */
	if (in->asid != 1 || p->asid != 1 || w->binding_generation != 1 ||
	    p->binding_generation != 1 || in->event_slot != 0 || w->event_sequence != 1 ||
	    in->target_stamp != 0x200 || in->threshold_to_publish != 2 ||
	    in->uma_slot != 2 || p->uma_slot != 1 ||
	    w->queue_fw != p->queue_fw || w->stats_fw != p->stats_fw ||
	    in->notifier_page_fw != p->notifier_page_fw ||
	    !gpu_m4_exec_fw_range(in->notifier_page_fw, 0x1000, 8) ||
	    w->notifier_fw != in->notifier_page_fw + 0x400 ||
	    w->firmware_stamp_fw != in->notifier_page_fw + 0x640 ||
	    w->user_stamp_fw != in->notifier_page_fw + 0x648 ||
	    p->expected_output != 0x44a72000 || p->output != p->expected_output ||
	    p->retirement.expected_stamp != 0x100 || p->retirement.next != 1 ||
	    p->retirement.pipe_next != 1 || p->retirement.uma_submitted != 1 ||
	    gpu_m4_exec_retired(&p->retirement) || w->registers.preemption_size != 0x4000 ||
	    w->registers.cdm_end <= w->registers.cdm_base ||
	    w->registers.cdm_end - w->registers.cdm_base > 0x4000 ||
	    (w->registers.cdm_end & 3) || !w->registers.usc_base ||
	    (w->registers.usc_base & 0xffffffffULL) || w->attachment_count != 1 ||
	    !w->attachments || w->attachments[0].va != in->client[GPU_M4_COMPUTE_OUTPUT].va ||
	    w->attachments[0].size != 4)
		return -EINVAL;
	if (!gpu_m4_exec_disjoint(in->client, GPU_M4_COMPUTE_RESOURCE_COUNT) ||
	    !gpu_m4_exec_disjoint(p->client, GPU_M4_COMPUTE_RESOURCE_COUNT) ||
	    !gpu_m4_exec_disjoint(in->firmware, 3) || !gpu_m4_exec_disjoint(p->firmware, 3))
		return -EINVAL;
	expected[GPU_M4_COMPUTE_WORK] = w->work_gpu;
	expected[GPU_M4_COMPUTE_PREEMPT] = w->registers.preemption;
	expected[GPU_M4_COMPUTE_CDM] = w->registers.cdm_base;
	expected[GPU_M4_COMPUTE_AUX] = w->auxiliary_gpu;
	expected[GPU_M4_COMPUTE_CONTEXT_STORE] = w->context_store_gpu;
	expected[GPU_M4_COMPUTE_POOL_STATE] = w->pool_state_gpu;
	expected[GPU_M4_COMPUTE_PAGE_LIST] = w->page_list_gpu;
	expected[GPU_M4_COMPUTE_POOL] = in->client[GPU_M4_COMPUTE_POOL].va;
	expected[GPU_M4_COMPUTE_OUTPUT] = w->attachments[0].va;
	expected[GPU_M4_COMPUTE_SHADER] = in->client[GPU_M4_COMPUTE_SHADER].va;
	expected[GPU_M4_COMPUTE_PROGRAM] = in->client[GPU_M4_COMPUTE_PROGRAM].va;
	for (i = 0; i < GPU_M4_COMPUTE_RESOURCE_COUNT; i++) {
		const struct gpu_m4_exec_interval *r = &in->client[i];
		bool write = i != GPU_M4_COMPUTE_CDM && i != GPU_M4_COMPUTE_SHADER &&
			i != GPU_M4_COMPUTE_PROGRAM;

		if (r->va != expected[i] || r->size != size[i] || (r->va & 0x3fff) ||
		    p->client[i].size != size[i] || (p->client[i].va & 0x3fff))
			return -EINVAL;
		for (j = 0; j < GPU_M4_COMPUTE_RESOURCE_COUNT; j++)
			if (gpu_m4_compute_overlap(r->va, r->size, p->client[j].va,
						   p->client[j].size))
				return -EINVAL;
		ret = gpu_m4_exec_vm_range(w->vm, r->va, r->size, write);
		if (ret)
			return ret < 0 ? ret : -EIO;
	}
	for (i = GPU_M4_COMPUTE_SHADER; i <= GPU_M4_COMPUTE_PROGRAM; i++)
		if (in->client[i].va < w->registers.usc_base ||
		    in->client[i].va - w->registers.usc_base > 0x100000000ULL - 0x4000)
			return -EINVAL;
	if (in->firmware[0].va != w->work_fw || in->firmware[0].size != 0x4000 ||
	    in->firmware[1].va != w->auxiliary_fw || in->firmware[1].size != 0x4000 ||
	    in->firmware[2].va != w->counter_fw || in->firmware[2].size != 0x4000)
		return -EINVAL;
	for (i = 0; i < 3; i++) {
		if (p->firmware[i].size != 0x4000 ||
		    !gpu_m4_exec_fw_range(in->firmware[i].va, 0x4000, 8) ||
		    !gpu_m4_exec_fw_range(p->firmware[i].va, 0x4000, 8))
			return -EINVAL;
		firmware[i] = in->firmware[i];
		firmware[i + 3] = p->firmware[i];
	}
	firmware[6] = (struct gpu_m4_exec_interval){ w->queue_fw, GPU_M4_EXEC_QUEUE_SIZE };
	firmware[7] = (struct gpu_m4_exec_interval){ w->stats_fw, 0xe10 };
	firmware[8] = (struct gpu_m4_exec_interval){ in->notifier_page_fw, 0x1000 };
	/* Selected stamps are inside the retained page, not separate FW objects. */
	if (!gpu_m4_exec_fw_range(w->queue_fw, GPU_M4_EXEC_QUEUE_SIZE, 8) ||
	    !gpu_m4_exec_fw_range(w->stats_fw, 0xe10, 8) ||
	    !gpu_m4_exec_disjoint(firmware, 9))
		return -EINVAL;
	ret = in->check_lease(in->cookie, in, output, scratch);
	return ret > 0 ? -EIO : ret;
}

/* Prevent the transactional commit from clobbering retained input metadata or
 * its private scratch through a different CPU span. Backing-page ownership and
 * physical aliases still require the mandatory locked check_lease callback.
 */
static inline bool gpu_m4_compute_cpu_disjoint(const void *a, unsigned long na,
					      const void *b, unsigned long nb)
{
	unsigned long va = (unsigned long)a, vb = (unsigned long)b;

	return a && b && na && nb && na <= ~0UL - va && nb <= ~0UL - vb &&
		(va + na <= vb || vb + nb <= va);
}

/* Pure transactional encoding into a new, separately leased owner. Calling the
 * first-work encoder with a local sequence-0 input only shares its qualified
 * byte grammar; it does not reuse, clear or republish the first work allocation.
 * The live page, previous owner and all producer state remain untouched.
 */
static inline int gpu_m4_compute_second(void *output,
					struct gpu_m4_compute_publication *publication,
					void *scratch,
					const struct gpu_m4_compute_input *in,
					u64 timestamp)
{
	struct gpu_m4_compute_publication next = { 0 };
	struct gpu_m4_exec_compute_input first;
	struct gpu_m4_exec_pipe_record pipe = { 0 };
	const struct gpu_m4_exec_retirement *prior;
	const void *metadata[4];
	unsigned long lengths[4];
	unsigned int i;
	int ret;

	if (!in || !in->previous || !publication || !timestamp ||
	    !gpu_m4_compute_cpu_disjoint(output, GPU_M4_EXEC_COMPUTE_SIZE,
					 scratch, GPU_M4_EXEC_COMPUTE_SIZE) ||
	    !gpu_m4_compute_cpu_disjoint(publication, sizeof(*publication),
					 output, GPU_M4_EXEC_COMPUTE_SIZE) ||
	    !gpu_m4_compute_cpu_disjoint(publication, sizeof(*publication),
					 scratch, GPU_M4_EXEC_COMPUTE_SIZE))
		return -EINVAL;
	metadata[0] = in;
	lengths[0] = sizeof(*in);
	metadata[1] = in->previous;
	lengths[1] = sizeof(*in->previous);
	metadata[2] = in->work.vm;
	lengths[2] = sizeof(*in->work.vm);
	metadata[3] = in->work.attachments;
	lengths[3] = sizeof(*in->work.attachments);
	for (i = 0; i < 4; i++)
		if (!gpu_m4_compute_cpu_disjoint(output, GPU_M4_EXEC_COMPUTE_SIZE,
						 metadata[i], lengths[i]) ||
		    !gpu_m4_compute_cpu_disjoint(scratch, GPU_M4_EXEC_COMPUTE_SIZE,
						 metadata[i], lengths[i]) ||
		    !gpu_m4_compute_cpu_disjoint(publication, sizeof(*publication),
						 metadata[i], lengths[i]))
			return -EINVAL;
	ret = gpu_m4_compute_validate(in, output, scratch);
	if (ret)
		return ret;
	prior = &in->previous->retirement;
	ret = gpu_m4_exec_ring_reserve(prior->queue_read, prior->queue_write, 1,
				       &next.queue_next);
	if (ret)
		return ret;
	ret = gpu_m4_exec_pipe_reserve(prior->pipe_read, prior->pipe_cfi,
				      prior->pipe_write, &next.pipe_next);
	if (ret)
		return ret;
	if (next.queue_next != 2 || next.pipe_next != 2)
		return -EINVAL;
	first = in->work;
	first.event_sequence = 0;
	ret = gpu_m4_exec_first_compute(scratch, &first);
	if (ret)
		return ret < 0 ? ret : -EIO;
	put_unaligned_le32(in->target_stamp, (u8 *)scratch + 0x810);
	put_unaligned_le32(in->event_slot, (u8 *)scratch + 0x818);
	put_unaligned_le32(in->work.event_sequence, (u8 *)scratch + 0x824);
	put_unaligned_le32(in->work.event_sequence, (u8 *)scratch + 0x938);
	put_unaligned_le32(in->target_stamp, (u8 *)scratch + 0xb8c);
	put_unaligned_le32(in->uma_slot, (u8 *)scratch + 0xe08);
	pipe.timestamp = timestamp;
	pipe.queue_info = in->work.queue_fw;
	pipe.queue_producer = next.queue_next;
	pipe.pipe = 2;
	pipe.scheduler_slot = 15;
	pipe.new_queue = false;
	ret = gpu_m4_exec_pipe_record(next.pipe_record, &pipe);
	if (ret)
		return ret;
	ret = gpu_m4_exec_pipe_kick(pipe.pipe, &next.kick);
	if (ret)
		return ret;
	next.work_fw = in->work.work_fw;
	next.counter_fw = in->work.counter_fw;
	next.counter_value = 1;
	next.threshold_fw = in->notifier_page_fw + 0x610;
	next.threshold_before = 1;
	next.threshold_value = in->threshold_to_publish;
	next.uma_slot = in->uma_slot;
	next.event_sequence = in->work.event_sequence;
	next.target_stamp = in->target_stamp;
	memcpy(output, scratch, GPU_M4_EXEC_COMPUTE_SIZE);
	*publication = next;
	return 0;
}

#endif
