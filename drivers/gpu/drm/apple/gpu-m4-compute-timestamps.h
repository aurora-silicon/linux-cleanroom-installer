/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_COMPUTE_TIMESTAMPS_H
#define APPLE_GPU_M4_COMPUTE_TIMESTAMPS_H

#include "gpu-m4-exec.h"

/* Detached J713/25G83 preparation. These aliases are not client GPU VAs. */
#define GPU_M4_COMPUTE_TS_START 0xfffffc2ffc000000ULL
#define GPU_M4_COMPUTE_TS_END 0xfffffc3000000000ULL
#define GPU_M4_COMPUTE_TS_COPIES 8U
#define GPU_M4_COMPUTE_TS_DESTINATIONS 10U
#define GPU_M4_COMPUTE_TS_RECORD_SIZE 0x4cU

struct gpu_m4_compute_ts_destination {
	const void *object;
	const void *lease;
	u64 object_offset;
	u64 firmware_va;
};

struct gpu_m4_compute_ts_input;

struct gpu_m4_compute_ts_ops {
	/* Under the provider's owner lock, prove source is an independently
	 * qualified private compute image with the specified canonical command
	 * and queue aliases. Prove destination/scratch are private, unqueued and
	 * cannot alias a retained firmware owner through another CPU/PA mapping.
	 * Keep the full command allocation, queue, VM and its aliases pinned.
	 * No mapping, cache, producer or notifier publication is performed here.
	 */
	int (*check_work)(void *cookie, const struct gpu_m4_compute_ts_input *input,
			  const void *output, const void *scratch);
	/* Retain this EXACT BO, offset and bound eight-byte firmware-RW alias.
	 * Verify bounds and permission against the held VM snapshot. Include
	 * the BO in external GPU-write dma-resv/cache ownership, including CPU
	 * WB maintenance before successful fence signaling. Error owns no ref;
	 * success returns a non-NULL independent held reference. Firmware RW
	 * permission alone does not establish CPU coherence. No rebinding here.
	 */
	int (*hold)(void *cookie, const struct gpu_m4_compute_ts_destination *destination,
		    void **held);
	void (*put)(void *cookie, void *held);
};

struct gpu_m4_compute_ts_input {
	const void *work;
	u64 work_fw;
	u64 queue_fw;
	/* Independently absent endpoints are all-zero entries. Copies are dense
	 * and preserve submitted order, even if two entries name the same slice.
	 */
	struct gpu_m4_compute_ts_destination start;
	struct gpu_m4_compute_ts_destination end;
	struct gpu_m4_compute_ts_destination copies[GPU_M4_COMPUTE_TS_COPIES];
	u32 copy_count;
	void *cookie;
	const struct gpu_m4_compute_ts_ops *ops;
};

/* Initialize to zero before preparation. The command, callback table/cookie
 * and every held slice stay alive through definite retirement; on uncertain
 * delivery transfer/pin the entire plan. This is not permission to free a
 * failed published job.
 */
struct gpu_m4_compute_ts_plan {
	void *held[GPU_M4_COMPUTE_TS_DESTINATIONS];
	const struct gpu_m4_compute_ts_ops *ops;
	void *cookie;
	u64 sequence_fw;
	u32 sequence_size;
	u32 finalize_offset;
	u32 copy_count;
	u32 held_count;
};

static inline bool gpu_m4_compute_ts_separate(const void *a, unsigned long na,
					    const void *b, unsigned long nb)
{
	unsigned long va = (unsigned long)a, vb = (unsigned long)b;

	return a && b && na && nb && na <= ~0UL - va && nb <= ~0UL - vb &&
		(va + na <= vb || vb + nb <= va);
}

static inline int gpu_m4_compute_ts_status(int ret)
{
	return ret > 0 ? -EIO : ret;
}

static inline bool gpu_m4_compute_ts_absent(const struct gpu_m4_compute_ts_destination *d)
{
	return !d->object && !d->lease && !d->object_offset && !d->firmware_va;
}

static inline bool gpu_m4_compute_ts_valid(const struct gpu_m4_compute_ts_destination *d)
{
	return d->object && d->lease && !(d->object_offset & 7) &&
		d->object_offset <= ~0ULL - 8 && !(d->firmware_va & 7) &&
		d->firmware_va >= GPU_M4_COMPUTE_TS_START &&
		d->firmware_va < GPU_M4_COMPUTE_TS_END &&
		GPU_M4_COMPUTE_TS_END - d->firmware_va >= 8;
}

/* Only pre-publication rollback or definite GPU/cache retirement may call this. */
static inline void gpu_m4_compute_ts_put(struct gpu_m4_compute_ts_plan *plan)
{
	unsigned int i;

	if (!plan || !plan->ops || !plan->ops->put)
		return;
	for (i = 0; i < GPU_M4_COMPUTE_TS_DESTINATIONS; i++)
		if (plan->held[i])
			plan->ops->put(plan->cookie, plan->held[i]);
	memset(plan, 0, sizeof(*plan));
}

static inline bool gpu_m4_compute_ts_record(const u8 *record, u64 work_fw,
					  u64 queue_fw, bool end)
{
	return get_unaligned_le32(record) == (end ? 3 : 0x80000003U) &&
		get_unaligned_le64(record + 4) == work_fw + 0x82c &&
		get_unaligned_le64(record + 0xc) == work_fw + 0x834 &&
		get_unaligned_le64(record + 0x14) == work_fw + (end ? 0x83c : 0x834) &&
		get_unaligned_le64(record + 0x1c) == queue_fw &&
		!get_unaligned_le64(record + 0x24) &&
		get_unaligned_le64(record + 0x2c) == work_fw + 0x89c &&
		get_unaligned_le64(record + 0x34) == work_fw + 0x7dc &&
		!get_unaligned_le64(record + 0x3c) &&
		get_unaligned_le64(record + 0x44) == 1;
}

/* Transactional encoding: failure leaves output and plan unchanged; private
 * scratch may change. Never edit a live command, the original source sequence
 * or a timestamp BO. All supplied references are independently retained before
 * copying cells; visibility and counter/ring publication remain external.
 */
static inline int gpu_m4_compute_ts_prepare(void *output,
					  struct gpu_m4_compute_ts_plan *plan,
					  void *scratch,
					  const struct gpu_m4_compute_ts_input *in)
{
	struct gpu_m4_compute_ts_plan next = { 0 };
	const struct gpu_m4_compute_ts_destination *destination[GPU_M4_COMPUTE_TS_DESTINATIONS];
	const u8 *source;
	u8 *work = scratch, *sequence, *finalize;
	u32 offset, n;
	unsigned int i;
	int ret;

	if (!in || !plan || !in->ops || !in->ops->check_work || !in->ops->hold ||
	    !in->ops->put || in->copy_count > GPU_M4_COMPUTE_TS_COPIES ||
	    !gpu_m4_exec_fw_range(in->work_fw, GPU_M4_EXEC_COMPUTE_SIZE, 8) ||
	    !gpu_m4_exec_fw_range(in->queue_fw, GPU_M4_EXEC_QUEUE_SIZE, 8))
		return -EINVAL;
	if (!gpu_m4_compute_ts_separate(output, GPU_M4_EXEC_COMPUTE_SIZE,
					 scratch, GPU_M4_EXEC_COMPUTE_SIZE) ||
	    !gpu_m4_compute_ts_separate(output, GPU_M4_EXEC_COMPUTE_SIZE,
					 in->work, GPU_M4_EXEC_COMPUTE_SIZE) ||
	    !gpu_m4_compute_ts_separate(scratch, GPU_M4_EXEC_COMPUTE_SIZE,
					 in->work, GPU_M4_EXEC_COMPUTE_SIZE) ||
	    !gpu_m4_compute_ts_separate(plan, sizeof(*plan), output,
					 GPU_M4_EXEC_COMPUTE_SIZE) ||
	    !gpu_m4_compute_ts_separate(plan, sizeof(*plan), scratch,
					 GPU_M4_EXEC_COMPUTE_SIZE) ||
	    !gpu_m4_compute_ts_separate(plan, sizeof(*plan), in->work,
					 GPU_M4_EXEC_COMPUTE_SIZE) ||
	    !gpu_m4_compute_ts_separate(in, sizeof(*in), output, GPU_M4_EXEC_COMPUTE_SIZE) ||
	    !gpu_m4_compute_ts_separate(in, sizeof(*in), scratch, GPU_M4_EXEC_COMPUTE_SIZE) ||
	    !gpu_m4_compute_ts_separate(in, sizeof(*in), plan, sizeof(*plan)) ||
	    !gpu_m4_compute_ts_separate(in->ops, sizeof(*in->ops), output,
					 GPU_M4_EXEC_COMPUTE_SIZE) ||
	    !gpu_m4_compute_ts_separate(in->ops, sizeof(*in->ops), scratch,
					 GPU_M4_EXEC_COMPUTE_SIZE) ||
	    !gpu_m4_compute_ts_separate(in->ops, sizeof(*in->ops), plan, sizeof(*plan)))
		return -EINVAL;
	/* Reject storage overlap before interpreting plan cells. In particular,
	 * output bytes are not a valid initialized plan merely because its
	 * address was supplied as both arguments.
	 */
	if (plan->held_count)
		return -EBUSY;
	for (i = 0; i < GPU_M4_COMPUTE_TS_DESTINATIONS; i++)
		if (plan->held[i])
			return -EBUSY;
	n = in->copy_count;
	destination[0] = &in->start;
	destination[1] = &in->end;
	for (i = 0; i < GPU_M4_COMPUTE_TS_COPIES; i++)
		destination[i + 2] = &in->copies[i];
	next.ops = in->ops;
	next.cookie = in->cookie;
	for (i = 0; i < GPU_M4_COMPUTE_TS_DESTINATIONS; i++) {
		bool required = i >= 2 && i < 2 + n;

		if (required ? !gpu_m4_compute_ts_valid(destination[i]) :
		    (!gpu_m4_compute_ts_absent(destination[i]) &&
		     (i >= 2 || !gpu_m4_compute_ts_valid(destination[i]))))
			return -EINVAL;
	}
	if (n && gpu_m4_compute_ts_absent(&in->end))
		return -EINVAL;
	ret = gpu_m4_compute_ts_status(in->ops->check_work(in->cookie, in, output, scratch));
	if (ret)
		return ret;
	source = in->work;
	if (get_unaligned_le32(source) != 3 ||
	    get_unaligned_le64(source + 0x760) != in->work_fw + 0x900 ||
	    get_unaligned_le32(source + 0x768) != GPU_M4_EXEC_COMPUTE_SEQUENCE_SIZE ||
	    get_unaligned_le64(source + 0x834) != in->work_fw + 0x8c0 ||
	    get_unaligned_le64(source + 0x83c) != in->work_fw + 0x8c8 ||
	    get_unaligned_le64(source + 0x844) || get_unaligned_le64(source + 0x84c) ||
	    !gpu_m4_compute_ts_record(source + 0xabc, in->work_fw, in->queue_fw, false) ||
	    !gpu_m4_compute_ts_record(source + 0xb0c, in->work_fw, in->queue_fw, true) ||
	    get_unaligned_le32(source + 0xb08) != 1 ||
	    get_unaligned_le32(source + 0xb58) != 0xc ||
	    get_unaligned_le64(source + 0xbb4) != in->work_fw + 0xa60 ||
	    get_unaligned_le32(source + 0xbbc) != 0U - 0x258 ||
	    get_unaligned_le32(source + 0xbd4) != 0x40000002U)
		return -EINVAL;
	for (i = 0; i < GPU_M4_COMPUTE_TS_DESTINATIONS; i++) {
		if (gpu_m4_compute_ts_absent(destination[i]))
			continue;
		ret = gpu_m4_compute_ts_status(in->ops->hold(in->cookie, destination[i],
							   &next.held[i]));
		if (ret)
			goto release;
		if (!next.held[i]) {
			ret = -EIO;
			goto release;
		}
		next.held_count++;
	}
	memcpy(work, source, GPU_M4_EXEC_COMPUTE_SIZE);
	put_unaligned_le64(in->start.firmware_va, work + 0x844);
	put_unaligned_le64(in->end.firmware_va, work + 0x84c);
	next.sequence_size = GPU_M4_EXEC_COMPUTE_SEQUENCE_SIZE + n * GPU_M4_COMPUTE_TS_RECORD_SIZE;
	next.finalize_offset = 0x258 + n * GPU_M4_COMPUTE_TS_RECORD_SIZE;
	next.copy_count = n;
	offset = n ? 0x1000 : 0x900;
	sequence = work + offset;
	if (n) {
		memcpy(sequence, source + 0x900, 0x258);
		for (i = 0; i < n; i++) {
			u8 *record = sequence + 0x258 + i * GPU_M4_COMPUTE_TS_RECORD_SIZE;
			u64 pair_fw = in->work_fw + 0x1800 + i * 16;

			memcpy(record, source + 0xb0c, GPU_M4_COMPUTE_TS_RECORD_SIZE);
			put_unaligned_le64(pair_fw, record + 0x24);
			put_unaligned_le64(0, work + 0x1800 + i * 16);
			put_unaligned_le64(in->copies[i].firmware_va, work + 0x1808 + i * 16);
		}
		memcpy(sequence + next.finalize_offset, source + 0xb58,
		       GPU_M4_EXEC_COMPUTE_SEQUENCE_SIZE - 0x258);
	}
	if (in->start.firmware_va || in->end.firmware_va) {
		put_unaligned_le64(in->work_fw + 0x844, sequence + 0x1bc + 0x24);
		put_unaligned_le64(in->work_fw + 0x844, sequence + 0x20c + 0x24);
	}
	finalize = sequence + next.finalize_offset;
	put_unaligned_le32(0U - next.finalize_offset, finalize + 0x64);
	/* Source-qualified fanout deliberately retains Finalize +0x5c pointing
	 * at ORIGINAL work+0xa60, not the copied header at work+0x1160. Keep the
	 * original sequence/header and every other absolute pointer unchanged.
	 */
	next.sequence_fw = in->work_fw + offset;
	put_unaligned_le64(next.sequence_fw, work + 0x760);
	put_unaligned_le32(next.sequence_size, work + 0x768);
	memcpy(output, work, GPU_M4_EXEC_COMPUTE_SIZE);
	*plan = next;
	return 0;
release:
	gpu_m4_compute_ts_put(&next);
	return ret;
}

#endif
