/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_EVIDENCE_H
#define APPLE_GPU_M4_EVIDENCE_H

/* Diagnostic TLV, containing owned runtime data, never firmware TEXT/MMIO.
 * A snapshot is bounded, read-only and not atomic with respect to firmware.
 */
#define GPU_M4_EVIDENCE_CAPACITY 0x200000U
#define GPU_M4_EVIDENCE_HEADER 0x28U
#define GPU_M4_EVIDENCE_RECORD 0x28U
#define GPU_M4_EVIDENCE_BEFORE 0x1000U

struct gpu_m4_evidence {
	u8 data[GPU_M4_EVIDENCE_CAPACITY];
	u32 size, records;
	int error;
	bool failure_started, captured;
};

static inline int gpu_m4_evidence_begin(struct gpu_m4_evidence *e,
					int reason, int health, u64 time)
{
	if (!e)
		return -EINVAL;
	if (e->captured || e->size)
		return -EBUSY;
	memset(e->data, 0, GPU_M4_EVIDENCE_HEADER);
	memcpy(e->data, "M4G16EV1", 8);
	put_unaligned_le32(1, e->data + 8);
	put_unaligned_le32(reason, e->data + 0x14);
	put_unaligned_le64(time, e->data + 0x18);
	put_unaligned_le32(health, e->data + 0x20);
	e->size = GPU_M4_EVIDENCE_HEADER;
	e->records = 0;
	e->error = 0;
	return 0;
}

static inline int gpu_m4_evidence_append(struct gpu_m4_evidence *e, u32 id,
		const void *data, u32 size, u64 firmware, u64 gpu, u64 pa)
{
	u32 padded;
	u8 *record;

	if (!e || e->captured || !data || !size || !id ||
	    size > GPU_M4_EVIDENCE_CAPACITY - GPU_M4_EVIDENCE_RECORD ||
	    e->size < GPU_M4_EVIDENCE_HEADER || e->size > GPU_M4_EVIDENCE_CAPACITY)
		goto invalid;
	padded = (size + 7) & ~7U;
	if (GPU_M4_EVIDENCE_RECORD + padded > GPU_M4_EVIDENCE_CAPACITY - e->size)
		goto full;
	record = e->data + e->size;
	memset(record, 0, GPU_M4_EVIDENCE_RECORD + padded);
	put_unaligned_le32(id, record);
	put_unaligned_le32(size, record + 4);
	put_unaligned_le64(firmware, record + 8);
	put_unaligned_le64(gpu, record + 0x10);
	put_unaligned_le64(pa, record + 0x18);
	memcpy(record + GPU_M4_EVIDENCE_RECORD, data, size);
	e->size += GPU_M4_EVIDENCE_RECORD + padded;
	e->records++;
	return 0;
invalid:
	if (e && !e->captured && !e->error)
		e->error = -EINVAL;
	return -EINVAL;
full:
	if (!e->error)
		e->error = -ENOSPC;
	return -ENOSPC;
}

static inline int gpu_m4_evidence_finish(struct gpu_m4_evidence *e)
{
	if (!e || e->size < GPU_M4_EVIDENCE_HEADER || e->size > GPU_M4_EVIDENCE_CAPACITY)
		return -EINVAL;
	if (e->captured)
		return -EBUSY;
	put_unaligned_le32(e->records, e->data + 0xc);
	put_unaligned_le32(e->size, e->data + 0x10);
	put_unaligned_le32(e->error, e->data + 0x24);
	e->captured = true;
	return e->error;
}

#endif
