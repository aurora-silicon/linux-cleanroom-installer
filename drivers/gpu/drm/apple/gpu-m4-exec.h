/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_EXEC_H
#define APPLE_GPU_M4_EXEC_H

#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

/* J713/T8132, normalized 25G83 image. Encoding does not publish GPU work. */
#define GPU_M4_EXEC_RING_COUNT 256U
#define GPU_M4_EXEC_RING_STATE_SIZE 0x60U
#define GPU_M4_EXEC_QUEUE_SIZE 0x24c0U
#define GPU_M4_EXEC_CONTEXT_SIZE 0x38U
#define GPU_M4_EXEC_NOTIFY_SIZE 0x1000U
#define GPU_M4_EXEC_DEPENDENCY_SIZE 0x40U
/* barrier_relax: opcode-4 +0x3c ("internal", the firmware's additional
 * predecessor-finalization hold) emitted as 0 for a prior-command wait, as the
 * native AGXCLChannel/AGX3DChannel publishers do, only behind a command with
 * no API timestamp or end copy. Bit 0: compute waits; bit 1: a TA wait behind
 * a compute; bit 2: an early-tiling FS wait on the previous FS; bit 3: a TA
 * wait behind a render (its FS, or its TA when early tiling). The stamp test
 * and the ordinary dependency edge remain. Read per publication.
 */
static unsigned int gpu_m4_barrier_relax __attribute__((unused)) = 15;
#define GPU_M4_EXEC_ATTACHMENT_SIZE 0x104U
#define GPU_M4_EXEC_ATTACHMENT_MAX 16U
#define GPU_M4_EXEC_COMPUTE_SIZE 0x2000U
#define GPU_M4_EXEC_COMPUTE_SEQUENCE_SIZE 0x300U
#define GPU_M4_EXEC_FW_START 0xfffffc2000000000ULL
#define GPU_M4_EXEC_FW_END 0xfffffc2ffc000000ULL
#define GPU_M4_EXEC_VM_START 0x4000ULL
#define GPU_M4_EXEC_VM_END ((1ULL << 42) - 0x8000)
/* Compute CDM control stream span. The encoding names only its base (register
 * 0x1a420) and exclusive end minus four (work +0x7a0); it carries no length.
 * The stream is client memory: submit admits a mapped readable span and
 * publication re-walks it. This bound is the single seam for the working
 * contract's limit; provisionally it keeps any admitted span inside one
 * provider version's range walk (4 GiB of 16 KiB leaves).
 */
#define GPU_M4_EXEC_CDM_SPAN_MAX (1ULL << 31)

enum gpu_m4_exec_engine {
	GPU_M4_EXEC_TILER,
	GPU_M4_EXEC_FRAGMENT,
	GPU_M4_EXEC_COMPUTE,
	GPU_M4_EXEC_ENGINE_COUNT,
};

/* The caller must prove the whole range belongs to the current client VM. */
typedef int (*gpu_m4_exec_check_range)(void *cookie, u64 va, u64 size,
				      bool write);

struct gpu_m4_exec_vm {
	u64 start;
	u64 end;
	void *cookie;
	gpu_m4_exec_check_range check;
};

struct gpu_m4_exec_attachment {
	u64 va;
	u64 size;
};

/* A user TTBAT context: 1..63. Context 0 is the firmware's own. */
static inline bool gpu_m4_exec_user_asid(u32 asid)
{
	return asid && asid < 64;
}

static inline bool gpu_m4_exec_fw_range(u64 va, u64 size, u32 alignment)
{
	return size && alignment && !(alignment & (alignment - 1)) &&
		!(va & (alignment - 1)) && va >= GPU_M4_EXEC_FW_START &&
		va < GPU_M4_EXEC_FW_END && size <= GPU_M4_EXEC_FW_END - va;
}

/* The one CDM span predicate shared by submit-time decode and publication. */
static inline bool gpu_m4_exec_cdm_span_valid(u64 base, u64 end)
{
	return !(base & 3) && !(end & 3) && end > base && end - base >= 4 &&
		end - base <= GPU_M4_EXEC_CDM_SPAN_MAX;
}

static inline int gpu_m4_exec_vm_range(const struct gpu_m4_exec_vm *vm,
				     u64 va, u64 size, bool write)
{
	if (!vm || !vm->check || vm->start < GPU_M4_EXEC_VM_START ||
	    vm->end > GPU_M4_EXEC_VM_END || vm->start >= vm->end ||
	    !size || va < vm->start || va >= vm->end || size > vm->end - va)
		return -EINVAL;
	return vm->check(vm->cookie, va, size, write);
}

/* Validate first so a rejected attachment cannot leave a stale partial list. */
static inline int gpu_m4_exec_attachments(void *output,
					const struct gpu_m4_exec_vm *vm,
					const struct gpu_m4_exec_attachment *input,
					unsigned int count)
{
	u64 start[GPU_M4_EXEC_ATTACHMENT_MAX], length[GPU_M4_EXEC_ATTACHMENT_MAX];
	unsigned int i;
	int ret;

	if (!output || count > GPU_M4_EXEC_ATTACHMENT_MAX || (count && !input))
		return -EINVAL;
	for (i = 0; i < count; i++) {
		u64 end;

		if (!input[i].size || input[i].size > ~0ULL - input[i].va ||
		    input[i].va + input[i].size > ~0ULL - 127)
			return -EINVAL;
		start[i] = input[i].va & ~127ULL;
		end = (input[i].va + input[i].size + 127) & ~127ULL;
		length[i] = end - start[i];
		if (!length[i] || (length[i] >> 7) > 0xffffffffULL)
			return -EINVAL;
		ret = gpu_m4_exec_vm_range(vm, start[i], length[i], true);
		if (ret)
			return ret;
	}
	memset(output, 0, GPU_M4_EXEC_ATTACHMENT_SIZE);
	for (i = 0; i < count; i++) {
		void *record = (u8 *)output + i * 16;

		put_unaligned_le64(start[i], record);
		put_unaligned_le32(length[i] >> 7, record + 8);
		put_unaligned_le16(0x18, record + 12);
		put_unaligned_le16(1, record + 14);
	}
	put_unaligned_le32(count, (u8 *)output + 0x100);
	return 0;
}

static inline void gpu_m4_exec_context(void *output)
{
	u8 *context = output;

	memset(context, 0, GPU_M4_EXEC_CONTEXT_SIZE);
	context[0] = 0xff;
	context[1] = 0xff;
	context[5] = 1;
	context[0x26] = 2;
	context[0x33] = 0xff;
}

/* These are fresh, private objects. Do not reinitialize a firmware-owned queue. */
static inline int gpu_m4_exec_queue(void *state, void *info, u64 state_va,
				    u64 ring_va, u64 info_va, u64 notifier_va,
				    u64 context_va)
{
	if (!state || !info ||
	    !gpu_m4_exec_fw_range(state_va, GPU_M4_EXEC_RING_STATE_SIZE, 8) ||
	    !gpu_m4_exec_fw_range(ring_va, GPU_M4_EXEC_RING_COUNT * 8, 8) ||
	    !gpu_m4_exec_fw_range(info_va, GPU_M4_EXEC_QUEUE_SIZE, 8) ||
	    !gpu_m4_exec_fw_range(notifier_va, 0x18, 8) ||
	    !gpu_m4_exec_fw_range(context_va, GPU_M4_EXEC_CONTEXT_SIZE, 8))
		return -EINVAL;
	memset(state, 0, GPU_M4_EXEC_RING_STATE_SIZE);
	memset(info, 0, GPU_M4_EXEC_QUEUE_SIZE);
	put_unaligned_le32(GPU_M4_EXEC_RING_COUNT, (u8 *)state + 0x50);
	put_unaligned_le64(state_va, info);
	put_unaligned_le64(ring_va, (u8 *)info + 8);
	put_unaligned_le64(notifier_va, (u8 *)info + 0x10);
	put_unaligned_le64(info_va + 0xb0, (u8 *)info + 0x18);
	put_unaligned_le32(0xffffffffU, (u8 *)info + 0x2c);
	put_unaligned_le32(2, (u8 *)info + 0x30);
	put_unaligned_le32(2, (u8 *)info + 0x34);
	put_unaligned_le64(0xffff000000000000ULL, (u8 *)info + 0x38);
	put_unaligned_le32(2, (u8 *)info + 0x48);
	put_unaligned_le32(0xffffffffU, (u8 *)info + 0x4c);
	put_unaligned_le64(context_va, (u8 *)info + 0xa4);
	return 0;
}

static inline int gpu_m4_exec_notifier_list(void *output, u64 va)
{
	if (!output || !gpu_m4_exec_fw_range(va, 0x18, 8))
		return -EINVAL;
	memset(output, 0, 0x18);
	put_unaligned_le64(va, (u8 *)output + 8);
	return 0;
}

static inline bool gpu_m4_exec_context_registered(const void *input)
{
	const u8 *context = input;

	return context && context[0] != 0xff && context[1] != 0xff && context[2] == 4;
}

static inline int gpu_m4_exec_notifiers(void *output, u64 va, u32 context)
{
	unsigned int i;

	if (!output || !gpu_m4_exec_fw_range(va, GPU_M4_EXEC_NOTIFY_SIZE, 8))
		return -EINVAL;
	memset(output, 0, GPU_M4_EXEC_NOTIFY_SIZE);
	for (i = 0; i < GPU_M4_EXEC_ENGINE_COUNT; i++) {
		u8 *record = (u8 *)output + i * 0x200;

		put_unaligned_le64(va + 0x600 + i * 8, record);
		put_unaligned_le32(0x50, record + 0x10);
		put_unaligned_le32(context, record + 0x24);
		memset(record + 0xa8, 0xff, 8);
		put_unaligned_le32(1, record + 0x114);
	}
	return 0;
}

static inline int gpu_m4_exec_dependency(void *output, u64 zero_stamp_va)
{
	if (!output || !gpu_m4_exec_fw_range(zero_stamp_va, 4, 4))
		return -EINVAL;
	memset(output, 0, GPU_M4_EXEC_DEPENDENCY_SIZE);
	put_unaligned_le32(4, output);
	put_unaligned_le64(zero_stamp_va, (u8 *)output + 4);
	put_unaligned_le64(zero_stamp_va, (u8 *)output + 0xc);
	put_unaligned_le32(0x80, (u8 *)output + 0x20);
	return 0;
}

/* Outputs are reservation values only; the caller performs ordered publication. */
static inline int gpu_m4_exec_ring_reserve(u32 read, u32 write,
					 unsigned int entries, u32 *next)
{
	u32 used, available;

	if (!next || read >= GPU_M4_EXEC_RING_COUNT ||
	    write >= GPU_M4_EXEC_RING_COUNT || !entries ||
	    entries >= GPU_M4_EXEC_RING_COUNT)
		return -EINVAL;
	used = (write - read) & (GPU_M4_EXEC_RING_COUNT - 1);
	available = GPU_M4_EXEC_RING_COUNT - used - 1;
	if (entries > available)
		return -ENOSPC;
	*next = (write + entries) & (GPU_M4_EXEC_RING_COUNT - 1);
	return 0;
}

/* The sign-bit form avoids relying on an out-of-range u32-to-s32 conversion. */
static inline bool gpu_m4_exec_stamp_reached(u32 observed, u32 expected)
{
	return !((observed - expected) & 0x80000000U);
}

struct gpu_m4_exec_retirement {
	u32 expected_stamp;
	u32 firmware_stamp;
	u32 user_stamp;
	u32 next;
	u32 queue_read;
	u32 queue_write;
	u32 queue_consumer[3];
	u32 queue_busy;
	u32 pipe_read;
	u32 pipe_cfi;
	u32 pipe_write;
	u32 pipe_next;
	u32 pipe_capacity;
	u64 uma_submitted;
	u64 uma_completed;
	u64 timestamp_start;
	u64 timestamp_end;
	/* Not the only command in flight globally (two packets in flight):
	 * queue-info busy may stay set for this engine while other work runs,
	 * so only consumer/producer/pipe positions decide. Saved with the
	 * receipt so its later proofs replay the same rule.
	 */
	bool ignore_busy;
};

/* Serialized first-work gate: events alone never make an owner recyclable. */
static inline int gpu_m4_exec_retired(const struct gpu_m4_exec_retirement *r)
{
	unsigned int i;

	if (!r || (r->expected_stamp & 0xff) ||
	    r->next >= GPU_M4_EXEC_RING_COUNT ||
	    r->queue_read >= GPU_M4_EXEC_RING_COUNT ||
	    r->queue_write >= GPU_M4_EXEC_RING_COUNT || r->pipe_capacity != 256 ||
	    r->pipe_read >= r->pipe_capacity || r->pipe_write >= r->pipe_capacity ||
	    r->pipe_cfi >= r->pipe_capacity || r->pipe_next >= r->pipe_capacity)
		return -EINVAL;
	for (i = 0; i < 3; i++)
		if (r->queue_consumer[i] >= GPU_M4_EXEC_RING_COUNT)
			return -EINVAL;
	if (r->firmware_stamp != r->expected_stamp || r->user_stamp != r->expected_stamp ||
	    r->queue_read != r->next || r->queue_write != r->next ||
	    (r->queue_busy && !r->ignore_busy) ||
	    r->pipe_read != r->pipe_next || r->pipe_cfi != r->pipe_next ||
	    r->pipe_write != r->pipe_next || !r->uma_submitted ||
	    r->uma_submitted != r->uma_completed ||
	    !r->timestamp_start || r->timestamp_end <= r->timestamp_start)
		return -EAGAIN;
	for (i = 0; i < 3; i++)
		if (r->queue_consumer[i] != r->next)
			return -EAGAIN;
	return 0;
}

struct gpu_m4_exec_pipe_record {
	u64 timestamp;
	u64 queue_info;
	u16 queue_producer;
	u8 pipe;
	u8 scheduler_slot;
	bool new_queue;
};

static inline int gpu_m4_exec_pipe_reserve(u32 read, u32 cfi, u32 write, u32 *next)
{
	u32 value;

	if (!next || read >= 256 || cfi >= 256 || write >= 256)
		return -EINVAL;
	value = (write + 1) & 255;
	if (value == read || value == cfi)
		return -ENOSPC;
	*next = value;
	return 0;
}

static inline int gpu_m4_exec_pipe_record(void *output,
					const struct gpu_m4_exec_pipe_record *record)
{
	if (!output || !record || record->pipe >= 12 || record->queue_producer >= 256 ||
	    record->scheduler_slot >= 128 ||
	    !gpu_m4_exec_fw_range(record->queue_info, GPU_M4_EXEC_QUEUE_SIZE, 8))
		return -EINVAL;
	put_unaligned_le64(record->timestamp, output);
	put_unaligned_le64(record->queue_info, (u8 *)output + 8);
	put_unaligned_le32(record->pipe % 3, (u8 *)output + 0x10);
	put_unaligned_le16(record->queue_producer, (u8 *)output + 0x14);
	((u8 *)output)[0x16] = record->scheduler_slot;
	((u8 *)output)[0x17] = record->new_queue;
	return 0;
}

static inline int gpu_m4_exec_pipe_kick(unsigned int pipe, u64 *message)
{
	if (!message || pipe >= 12)
		return -EINVAL;
	*message = 0x0083000000000000ULL | pipe;
	return 0;
}

struct gpu_m4_exec_compute_resources {
	u64 preemption;
	u64 preemption_size;
	u64 cdm_base;
	u64 cdm_end;
	u64 usc_base;
};

/* This validates the register operands, not the shader or full job ownership. */
static inline int gpu_m4_exec_compute_registers(void *output,
					       const struct gpu_m4_exec_vm *vm,
					       const struct gpu_m4_exec_compute_resources *r,
					       u16 *count)
{
	static const u32 selectors[] = {
		0x1a510, 0x1a420, 0x1a4d0, 0x1a4d8, 0x1a4e0, 0x1a4e8,
		0x1a440, 0x1a458, 0x101d9,
		0x1a089, 0x1a091, 0x1a059, 0x1a061, 0x1a0b9, 0x1a0c1, 0x101d1, 0xd479,
		0x1a0e9, 0x107a1, 0xa599, 0xd411, 0x1a540, 0x14a9, 0xa351,
	};
	u64 values[24] = { 0 };
	unsigned int i;
	int ret;

	if (!output || !r || !count || !gpu_m4_exec_cdm_span_valid(r->cdm_base, r->cdm_end) ||
	    (r->preemption & 7) ||
	    r->preemption_size < 0x14a0 || (r->usc_base & 0xffffffffULL))
		return -EINVAL;
	ret = gpu_m4_exec_vm_range(vm, r->cdm_base, r->cdm_end - r->cdm_base, false);
	if (ret)
		return ret;
	ret = gpu_m4_exec_vm_range(vm, r->preemption, r->preemption_size, true);
	if (ret)
		return ret;
	/* A zero USC base omits the selector; otherwise reserve the whole window. */
	if (r->usc_base &&
	    (r->usc_base < vm->start || r->usc_base >= vm->end ||
	     0x100000000ULL > vm->end - r->usc_base))
		return -EINVAL;
	values[0] = r->preemption;
	values[1] = r->cdm_base;
	for (i = 0; i < 4; i++)
		values[2 + i] = r->preemption + 0x1480 + i * 8;
	values[6] = 0x154024201ULL;
	values[7] = 0x10c08860;
	values[8] = 0x1c;
	values[17] = 8;
	values[18] = 0xff0000;
	values[19] = 0x13200400020ULL;
	values[20] = 0x200000001ULL;
	values[21] = 1;
	values[22] = 1;
	values[23] = 1;
	memset(output, 0, 0x600);
	for (i = 0; i < 24; i++) {
		u8 *record = (u8 *)output + i * 12;

		put_unaligned_le32(selectors[i], record);
		put_unaligned_le64(values[i], record + 4);
	}
	*count = 24;
	if (r->usc_base) {
		put_unaligned_le32(0x10071, (u8 *)output + 24 * 12);
		put_unaligned_le64(r->usc_base, (u8 *)output + 24 * 12 + 4);
		*count = 25;
	}
	return 0;
}

#define GPU_M4_EXEC_POOL_SIZE 0x2000000ULL
#define GPU_M4_EXEC_POOL_STATE_SIZE 0x20000U
#define GPU_M4_EXEC_POOL_PAGE_LIST_SIZE 0x4000U

/* Fresh solitary owner only: firmware-mutated cursors may never be cleared. */
static inline int gpu_m4_exec_uma_pool(void *state, void *page_list,
				     const struct gpu_m4_exec_vm *vm, u64 pool_gpu)
{
	unsigned int i;
	int ret;

	if (!state || !page_list || (pool_gpu & 0x3fff))
		return -EINVAL;
	ret = gpu_m4_exec_vm_range(vm, pool_gpu, GPU_M4_EXEC_POOL_SIZE, true);
	if (ret)
		return ret;
	memset(state, 0, GPU_M4_EXEC_POOL_STATE_SIZE);
	memset(page_list, 0, GPU_M4_EXEC_POOL_PAGE_LIST_SIZE);
	for (i = 0; i < 8192; i++)
		put_unaligned_le64(pool_gpu + (u64)i * 0x1000, (u8 *)state + i * 8);
	for (i = 0; i < 16; i++)
		put_unaligned_le64((1ULL << 61) | (pool_gpu + (u64)i * 0x200000),
				   (u8 *)page_list + i * 64);
	return 0;
}

static inline int gpu_m4_exec_uma_descriptor(void *output,
					   const struct gpu_m4_exec_vm *vm, u64 work_fw,
					   u64 counter_fw, u64 state_gpu, u64 page_list_gpu)
{
	int ret;

	if (!output || !gpu_m4_exec_fw_range(work_fw, GPU_M4_EXEC_COMPUTE_SIZE, 8) ||
	    !gpu_m4_exec_fw_range(counter_fw, 8, 8) ||
	    (state_gpu & 0x3fff) || (page_list_gpu & 0x3fff))
		return -EINVAL;
	ret = gpu_m4_exec_vm_range(vm, state_gpu, GPU_M4_EXEC_POOL_STATE_SIZE, true);
	if (ret)
		return ret;
	ret = gpu_m4_exec_vm_range(vm, page_list_gpu, GPU_M4_EXEC_POOL_PAGE_LIST_SIZE, true);
	if (ret)
		return ret;
	memset(output, 0, 0xa0);
	put_unaligned_le64(work_fw, output);
	put_unaligned_le32(1, (u8 *)output + 8);
	put_unaligned_le32(2, (u8 *)output + 0x10);
	put_unaligned_le64(state_gpu, (u8 *)output + 0x14);
	put_unaligned_le32(16384, (u8 *)output + 0x1c);
	put_unaligned_le32(8192, (u8 *)output + 0x24);
	put_unaligned_le32(8192, (u8 *)output + 0x2c);
	put_unaligned_le64(page_list_gpu, (u8 *)output + 0x30);
	put_unaligned_le32(4, (u8 *)output + 0x40);
	put_unaligned_le32(128, (u8 *)output + 0x48);
	put_unaligned_le64(counter_fw, (u8 *)output + 0x4c);
	put_unaligned_le32(1, (u8 *)output + 0x5c);
	put_unaligned_le32(1, (u8 *)output + 0x84);
	put_unaligned_le32(2, (u8 *)output + 0x94);
	return 0;
}

/* Ordinary runtime works keep their private start/end engine stamps in their
 * separately mapped NC submitted-counter page, at counter_fw +
 * GPU_M4_EXEC_COMPUTE_STAMPS, instead of work+0x8c0: the firmware does not
 * synchronize those cells, so a cached work page must not hold host-observed
 * receipt values. stamps_fw 0 keeps work+0x8c0 (fixture/trial works).
 */
#define GPU_M4_EXEC_COMPUTE_STAMPS 0x40U

/* Encode only into a fresh private work; these cells remain pinned until reboot. */
static inline int gpu_m4_exec_compute_timestamps(void *output, u64 work_fw, u64 queue_fw,
						 u64 stamps_fw)
{
	u8 *work = output;
	unsigned int i;

	if (!output || !gpu_m4_exec_fw_range(work_fw, GPU_M4_EXEC_COMPUTE_SIZE, 8) ||
	    !gpu_m4_exec_fw_range(queue_fw, GPU_M4_EXEC_QUEUE_SIZE, 8))
		return -EINVAL;
	put_unaligned_le64(stamps_fw, work + 0x834);
	put_unaligned_le64(stamps_fw + 8, work + 0x83c);
	put_unaligned_le64(0, work + 0x844);
	put_unaligned_le64(0, work + 0x84c);
	for (i = 0; i < 2; i++) {
		u8 *record = work + (i ? 0xb0c : 0xabc);

		memset(record, 0, 0x4c);
		put_unaligned_le32(i ? 3 : 0x80000003U, record);
		put_unaligned_le64(work_fw + 0x82c, record + 4);
		put_unaligned_le64(work_fw + 0x834, record + 0xc);
		put_unaligned_le64(work_fw + (i ? 0x83c : 0x834), record + 0x14);
		put_unaligned_le64(queue_fw, record + 0x1c);
		put_unaligned_le64(work_fw + 0x89c, record + 0x2c);
		put_unaligned_le64(work_fw + 0x7dc, record + 0x34);
		put_unaligned_le64(1, record + 0x44);
	}
	put_unaligned_le32(1, work + 0xb08);
	return 0;
}

struct gpu_m4_exec_compute_input {
	const struct gpu_m4_exec_vm *vm;
	struct gpu_m4_exec_compute_resources registers;
	u64 work_fw;
	u64 work_gpu;
	u64 queue_fw;
	u64 stats_fw;
	u64 auxiliary_fw;
	u64 auxiliary_gpu;
	u64 context_store_gpu;
	u64 counter_fw;
	u64 stamps_fw; /* private start/end stamp pair; 0: work_fw + 0x8c0 */
	u64 notifier_fw;
	u64 firmware_stamp_fw;
	u64 user_stamp_fw;
	u64 pool_state_gpu;
	u64 page_list_gpu;
	u32 event_sequence;
	u8 binding_generation;
	const struct gpu_m4_exec_attachment *attachments;
	unsigned int attachment_count;
};

struct gpu_m4_exec_interval {
	u64 va;
	u64 size;
};

static inline bool gpu_m4_exec_disjoint(const struct gpu_m4_exec_interval *ranges,
				       unsigned int count)
{
	unsigned int i, j;

	for (i = 0; i < count; i++) {
		if (!ranges[i].size || ranges[i].size > ~0ULL - ranges[i].va)
			return false;
		for (j = 0; j < i; j++)
			if (ranges[i].va < ranges[j].va + ranges[j].size &&
			    ranges[j].va < ranges[i].va + ranges[i].size)
				return false;
	}
	return true;
}

/*
 * Fresh solitary ASID-1 gate only. The caller independently checks firmware and
 * client aliases against the same backing, zeros retained context/preemption/
 * auxiliary storage, builds the pool, and sets the separate UMA counter to 1.
 * No bytes are published here; discard private output on any error. This does
 * not generate shader/container/CDM bytes or qualify a reused owner.
 */
static inline int gpu_m4_exec_first_compute(void *output,
					  const struct gpu_m4_exec_compute_input *in)
{
	u8 *work = output, *sequence, *finalize, *notifier;
	struct gpu_m4_exec_interval firmware[6], gpu[7];
	u16 count;
	int ret;

	if (!output || !in || !in->binding_generation || in->event_sequence != 0 ||
	    in->attachment_count > GPU_M4_EXEC_ATTACHMENT_MAX ||
	    (in->attachment_count && !in->attachments) ||
	    !gpu_m4_exec_fw_range(in->work_fw, GPU_M4_EXEC_COMPUTE_SIZE, 8) ||
	    !gpu_m4_exec_fw_range(in->queue_fw, GPU_M4_EXEC_QUEUE_SIZE, 8) ||
	    !gpu_m4_exec_fw_range(in->stats_fw, 0xe10, 8) ||
	    !gpu_m4_exec_fw_range(in->auxiliary_fw, 0x4000, 8) ||
	    !gpu_m4_exec_fw_range(in->counter_fw, 8, 8) ||
	    in->notifier_fw < 0x400 ||
	    !gpu_m4_exec_fw_range(in->notifier_fw - 0x400, GPU_M4_EXEC_NOTIFY_SIZE, 8) ||
	    in->firmware_stamp_fw != in->notifier_fw + 0x240 ||
	    in->user_stamp_fw != in->notifier_fw + 0x248 ||
	    (in->work_gpu & 0x3fff) || (in->auxiliary_gpu & 0x3fff) ||
	    (in->context_store_gpu & 0x3fff) || in->registers.preemption_size < 0x4000 ||
	    !gpu_m4_exec_cdm_span_valid(in->registers.cdm_base, in->registers.cdm_end))
		return -EINVAL;
	firmware[0] = (struct gpu_m4_exec_interval){ in->work_fw, 0x2000 };
	firmware[1] = (struct gpu_m4_exec_interval){ in->queue_fw, 0x24c0 };
	firmware[2] = (struct gpu_m4_exec_interval){ in->stats_fw, 0xe10 };
	firmware[3] = (struct gpu_m4_exec_interval){ in->auxiliary_fw, 0x4000 };
	firmware[4] = (struct gpu_m4_exec_interval){ in->counter_fw, 8 };
	firmware[5] = (struct gpu_m4_exec_interval){ in->notifier_fw - 0x400,
						   GPU_M4_EXEC_NOTIFY_SIZE };
	gpu[0] = (struct gpu_m4_exec_interval){ in->work_gpu, 0x4000 };
	gpu[1] = (struct gpu_m4_exec_interval){ in->registers.preemption,
						   in->registers.preemption_size };
	gpu[2] = (struct gpu_m4_exec_interval){ in->registers.cdm_base,
						   in->registers.cdm_end - in->registers.cdm_base };
	gpu[3] = (struct gpu_m4_exec_interval){ in->auxiliary_gpu, 0x4000 };
	gpu[4] = (struct gpu_m4_exec_interval){ in->context_store_gpu, 0x40000 };
	gpu[5] = (struct gpu_m4_exec_interval){ in->pool_state_gpu, 0x20000 };
	gpu[6] = (struct gpu_m4_exec_interval){ in->page_list_gpu, 0x4000 };
	if (!gpu_m4_exec_disjoint(firmware, 6) || !gpu_m4_exec_disjoint(gpu, 7))
		return -EINVAL;
	ret = gpu_m4_exec_vm_range(in->vm, in->work_gpu, GPU_M4_EXEC_COMPUTE_SIZE, true);
	if (ret)
		return ret;
	ret = gpu_m4_exec_vm_range(in->vm, in->auxiliary_gpu, 0x4000, true);
	if (ret)
		return ret;
	ret = gpu_m4_exec_vm_range(in->vm, in->context_store_gpu, 0x40000, true);
	if (ret)
		return ret;
	memset(work, 0, GPU_M4_EXEC_COMPUTE_SIZE);
	ret = gpu_m4_exec_compute_registers(work + 0x20, in->vm, &in->registers, &count);
	if (ret)
		return ret;
	ret = gpu_m4_exec_attachments(work + 0x958, in->vm, in->attachments,
				      in->attachment_count);
	if (ret)
		return ret;
	ret = gpu_m4_exec_uma_descriptor(work + 0xe00, in->vm, in->work_fw,
					in->counter_fw, in->pool_state_gpu, in->page_list_gpu);
	if (ret)
		return ret;
	ret = gpu_m4_exec_compute_timestamps(work, in->work_fw, in->queue_fw,
		in->stamps_fw ?: in->work_fw + 0x8c0);
	if (ret)
		return ret;
	put_unaligned_le32(3, work);
	/* +4..+11 are zero: the UMA submission counter is a separate allocation. */
	put_unaligned_le32(1, work + 0xc);
	put_unaligned_le64(in->notifier_fw, work + 0x10);
	put_unaligned_le64(in->work_gpu + 0x20, work + 0x720);
	put_unaligned_le16(count, work + 0x728);
	put_unaligned_le16(count * 12, work + 0x72a);
	put_unaligned_le64(in->work_fw + 0x900, work + 0x760);
	put_unaligned_le32(GPU_M4_EXEC_COMPUTE_SEQUENCE_SIZE, work + 0x768);
	put_unaligned_le64(in->registers.preemption, work + 0x798);
	put_unaligned_le64(in->registers.cdm_end - 4, work + 0x7a0);
	put_unaligned_le64(0x154024201ULL, work + 0x7c8);
	/* Qualified first-work sentinel, independent of selected stamp cells. */
	put_unaligned_le32(0xffffffffU, work + 0x7e8);
	put_unaligned_le64(in->user_stamp_fw, work + 0x800);
	put_unaligned_le64(in->firmware_stamp_fw, work + 0x808);
	put_unaligned_le32(0x100, work + 0x810);
	put_unaligned_le32(15, work + 0x814);
	put_unaligned_le32(1, work + 0x81c);
	put_unaligned_le32(1, work + 0x820);
	put_unaligned_le32(in->event_sequence, work + 0x824);
	put_unaligned_le64(in->work_fw + 0xe00, work + 0x86a);
	work[0x872] = 1;
	put_unaligned_le64(0x1588000, work + 0x873);
	put_unaligned_le64(0xef8000, work + 0x87b);
	put_unaligned_le64(in->auxiliary_fw, work + 0x883);
	work[0x88b] = in->binding_generation;

	sequence = work + 0x900;
	put_unaligned_le32(0xb, sequence);
	put_unaligned_le64(in->work_fw + 0x20, sequence + 0x14);
	put_unaligned_le64(in->stats_fw, sequence + 0x1c);
	put_unaligned_le64(in->queue_fw, sequence + 0x24);
	put_unaligned_le32(1, sequence + 0x2c);
	put_unaligned_le32(1, sequence + 0x30);
	put_unaligned_le32(in->event_sequence, sequence + 0x38);
	put_unaligned_le64(in->work_fw + 0x76c, sequence + 0x44);
	put_unaligned_le32(1, sequence + 0x50);
	put_unaligned_le64(in->work_fw + 0xe00, sequence + 0x160);
	put_unaligned_le64(in->auxiliary_gpu, sequence + 0x178);
	put_unaligned_le64(in->context_store_gpu, sequence + 0x188);
	put_unaligned_le64(in->context_store_gpu + 0xf400, sequence + 0x190);
	put_unaligned_le64(in->context_store_gpu + 0x1e800, sequence + 0x198);
	put_unaligned_le64(in->context_store_gpu + 0x1f000, sequence + 0x1a0);
	put_unaligned_le64(in->work_fw + 0x8ac, sequence + 0x1a8);
	put_unaligned_le32(15, sequence + 0x1b8);

	finalize = work + 0xb58;
	put_unaligned_le32(0xc, finalize);
	put_unaligned_le64(in->stats_fw, finalize + 4);
	put_unaligned_le64(in->queue_fw, finalize + 0xc);
	put_unaligned_le32(1, finalize + 0x14);
	put_unaligned_le64(in->work_fw + 0x76c, finalize + 0x18);
	put_unaligned_le32(1, finalize + 0x24);
	put_unaligned_le64(in->firmware_stamp_fw, finalize + 0x2c);
	put_unaligned_le32(0x100, finalize + 0x34);
	put_unaligned_le64(in->work_fw + 0xa60, finalize + 0x5c);
	put_unaligned_le32(0U - 0x258, finalize + 0x64);
	finalize[0x68] = in->attachment_count != 0;
	put_unaligned_le64(in->work_fw + 0x8ac, finalize + 0x69);
	put_unaligned_le64(in->work_fw + 0x88c, finalize + 0x71);
	put_unaligned_le32(0x40000002, work + 0xbd4);

	notifier = work + 0xc00;
	put_unaligned_le64(in->work_fw + 0xf00, notifier);
	put_unaligned_le32(0x50, notifier + 0x10);
	put_unaligned_le32(1, notifier + 0x24);
	memset(notifier + 0xa8, 0xff, 8);
	put_unaligned_le32(1, notifier + 0x114);
	put_unaligned_le32(0xffffffffU, work + 0xee0);
	put_unaligned_le32(0xffffffffU, work + 0xee4);
	/* First reservation carries sequence 0; its published threshold is 1. */
	put_unaligned_le64(1, work + 0xf00);
	return 0;
}

struct gpu_m4_exec_geometry {
	u32 pixels;
	u32 screen;
	u32 x_blocks;
	u32 y_blocks;
	u32 blocks;
	u32 entries;
	u32 region_stride;
	u32 tpc_stride;
	u32 isp_blocks;
	u32 utile_control;
	u32 tilebuffer_control;
	u64 tilemap_size;
	u64 tpc_size;
};

static inline int gpu_m4_exec_geometry(struct gpu_m4_exec_geometry *output,
				      u32 width, u32 height, u32 layers, u32 samples,
				      u32 utile_width, u32 utile_height, u32 sample_size)
{
	struct gpu_m4_exec_geometry g;
	u32 tx, ty, bx, by, sx, sy, sample_log;
	u64 bytes, tilemap;

	if (!output || !width || width > 16384 || !height || height > 16384 ||
	    !layers || layers > 2048 || (samples != 1 && samples != 2 && samples != 4) ||
	    (utile_width != 16 && utile_width != 32) ||
	    (utile_height != 16 && utile_height != 32))
		return -EINVAL;
	bytes = (u64)sample_size * utile_width * utile_height * samples;
	if (bytes > 32768)
		return -EINVAL;
	tx = (width + 31) / 32;
	ty = (height + 31) / 32;
	bx = (((tx + 3) / 4) + 3) & ~3U;
	by = (((ty + 3) / 4) + 3) & ~3U;
	sx = bx * (utile_width == 16 ? 2 : 1);
	sy = by * (utile_height == 16 ? 2 : 1);
	sample_log = samples == 4 ? 2 : samples == 2 ? 1 : 0;
	g.pixels = (width - 1) | ((height - 1) << 16);
	g.screen = (tx - 1) | ((ty - 1) << 12);
	g.x_blocks = (bx << 10) | (bx << 18) | 3 * bx;
	g.y_blocks = (by << 18) | (by << 10) | 3 * by;
	g.blocks = bx * by;
	g.entries = 16 * sx * sy;
	g.region_stride = 5 * g.entries / 64;
	g.tpc_stride = g.entries / 8;
	g.isp_blocks = ((sx & 0x3fd) << 16) | (sy & 0x3fd);
	g.utile_control = (utile_width == 32 ? 0x2000 : 0x1000) |
		(utile_height == 32 ? 0x8000 : 0x4000) | sample_log;
	g.tilebuffer_control = (bytes + 2047) / 2048 | (sample_log << 17);
	tilemap = 5ULL * g.entries * layers;
	g.tilemap_size = (tilemap + 4095) & ~4095ULL;
	g.tpc_size = 8ULL * g.entries * layers;
	*output = g;
	return 0;
}

#endif
