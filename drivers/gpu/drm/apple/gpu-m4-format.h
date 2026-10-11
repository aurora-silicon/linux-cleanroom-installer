/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_FORMAT_H
#define APPLE_GPU_M4_FORMAT_H

#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

/* J713/T8132, normalized 25G83 image only. These helpers do not admit hardware. */
#define GPU_M4_PAGE_SIZE 0x4000ULL
#define GPU_M4_PA_MASK 0x3ffffffc000ULL
#define GPU_M4_HOST_START 0xfffffc2000000000ULL
#define GPU_M4_HOST_END 0xfffffc2ffc000000ULL
#define GPU_M4_LOADER_COUNT 42
#define GPU_M4_LOADER_STRIDE 0x28

enum gpu_m4_access {
	GPU_M4_FW_RW,
	GPU_M4_SHARED_RW,
	GPU_M4_GPU_RW,
	GPU_M4_FW_CACHED_RW,
	GPU_M4_MMIO_RW,
	GPU_M4_MMIO_RO,
	GPU_M4_MMIO_PROTECTED,
	GPU_M4_GPU_RO,
	GPU_M4_ACCESS_COUNT,
};

static inline int gpu_m4_leaf(u64 pa, enum gpu_m4_access access, bool client, u64 *pte)
{
	static const u64 attrs[GPU_M4_ACCESS_COUNT] = {
		[GPU_M4_FW_RW] = 0x00c000000000044bULL,
		[GPU_M4_SHARED_RW] = 0x00e000000000040bULL,
		[GPU_M4_GPU_RW] = 0x00c000000000048bULL,
		[GPU_M4_FW_CACHED_RW] = 0x00c0000000000443ULL,
		[GPU_M4_MMIO_RW] = 0x00c0000000000447ULL,
		[GPU_M4_MMIO_RO] = 0x0080000000000447ULL,
		[GPU_M4_MMIO_PROTECTED] = 0x00200000000004c7ULL,
		[GPU_M4_GPU_RO] = 0x008000000000048bULL,
	};

	if (pa & ~GPU_M4_PA_MASK || (unsigned int)access >= GPU_M4_ACCESS_COUNT)
		return -EINVAL;
	/* Never map firmware-private RAM or MMIO into a client lower root. */
	if (client && access != GPU_M4_GPU_RW && access != GPU_M4_GPU_RO)
		return -EINVAL;
	*pte = pa | attrs[access] | (client ? 0x800 : 0);
	return 0;
}

struct gpu_m4_loader {
	u64 pa;
	u32 length;
	u32 requested;
	u64 tag;
	u8 mode;
	enum gpu_m4_access access;
};

static const struct gpu_m4_loader gpu_m4_loader[GPU_M4_LOADER_COUNT] = {
	[0] = { 0x381014000, 0x4000, 0x4000, 0, 2, GPU_M4_MMIO_RW },
	[3] = { 0x2201c4000, 0x30000, 0x18000, 0, 2, GPU_M4_MMIO_RW },
	[9] = { 0x3803d0000, 0x1000, 0x1000, 0, 2, GPU_M4_MMIO_RW },
	[10] = { 0x3803c0000, 0x2000, 0x2000, 0, 0, GPU_M4_MMIO_RO },
	[12] = { 0x50165c000, 0x4000, 0x4000, 0, 2, GPU_M4_MMIO_RW },
	[14] = { 0x380280000, 0x8000, 0x8000, 0, 0, GPU_M4_MMIO_RO },
	[15] = { 0x38c840000, 0x24000, 0x24000, 0, 0, GPU_M4_MMIO_RO },
	[17] = { 0x300000000, 0x20000, 0x20000, 0, 2, GPU_M4_MMIO_RW },
	[22] = { 0x301000000, 0x8000, 0x8000, 0, 2, GPU_M4_MMIO_RW },
	[26] = { 0x300d04000, 0x8000, 0x8000, 0xd04000, 2, GPU_M4_MMIO_RW },
	[27] = { 0x300d0d000, 0x1000, 0x1000, 0xd0d000, 2, GPU_M4_MMIO_RW },
	[28] = { 0x300d58000, 0x8000, 0x8000, 0xd58000, 2, GPU_M4_MMIO_RW },
	[29] = { 0x300d10000, 0x4000, 0x4000, 0xd10000, 2, GPU_M4_MMIO_RW },
	[31] = { 0x300d40000, 0x4000, 0x4000, 0xd40000, 2, GPU_M4_MMIO_RW },
	[32] = { 0x300d60000, 0x8000, 0x8000, 0xd60000, 2, GPU_M4_MMIO_RW },
	[36] = { 0x300e00000, 0x4000, 0x4000, 0xe00000, 0, GPU_M4_MMIO_PROTECTED },
	[39] = { 0x300e08000, 0x8000, 0x8000, 0, 2, GPU_M4_MMIO_RW },
	[40] = { 0x300e1c000, 0x4000, 0x4000, 0xe1c000, 2, GPU_M4_MMIO_RW },
	[41] = { 0x300e1f800, 0x4000, 0x4000, 0, 2, GPU_M4_MMIO_RW },
};

/* Map these spans, not the record's logical length. Slot 3 is discontiguous. */
static inline int gpu_m4_loader_span(unsigned int slot, unsigned int bank,
				     u64 *pa, u32 *length, u32 *va_offset)
{
	const struct gpu_m4_loader *entry;
	u32 offset, bytes;

	if (slot >= GPU_M4_LOADER_COUNT || bank > (slot == 3 ? 1 : 0))
		return -EINVAL;
	entry = &gpu_m4_loader[slot];
	if (!entry->length)
		return -ENOENT;
	offset = entry->pa & (GPU_M4_PAGE_SIZE - 1);
	bytes = slot == 3 ? 0x18000 : entry->length;
	*pa = (entry->pa & GPU_M4_PA_MASK) + (bank ? 0x2000000 : 0);
	*length = (offset + bytes + GPU_M4_PAGE_SIZE - 1) & ~(GPU_M4_PAGE_SIZE - 1);
	*va_offset = bank ? 0x18000 : 0;
	return 0;
}

/* Encode into private RAM only; the caller owns admission, mapping and publish. */
static inline int gpu_m4_loader_encode(unsigned int slot, u64 va, void *record)
{
	const struct gpu_m4_loader *entry;
	u64 offset, span;

	if (slot >= GPU_M4_LOADER_COUNT)
		return -EINVAL;
	entry = &gpu_m4_loader[slot];
	if (!entry->length) {
		if (va)
			return -EINVAL;
		memset(record, 0, GPU_M4_LOADER_STRIDE);
		return 0;
	}
	offset = entry->pa & (GPU_M4_PAGE_SIZE - 1);
	span = (offset + entry->length + GPU_M4_PAGE_SIZE - 1) & ~(GPU_M4_PAGE_SIZE - 1);
	if (va < GPU_M4_HOST_START || va >= GPU_M4_HOST_END ||
	    (va & (GPU_M4_PAGE_SIZE - 1)) != offset ||
	    span > GPU_M4_HOST_END - (va - offset))
		return -EINVAL;
	put_unaligned_le64(entry->pa, record);
	put_unaligned_le64(va, record + 8);
	put_unaligned_le32(entry->length, record + 0x10);
	put_unaligned_le32(entry->requested, record + 0x14);
	put_unaligned_le64(entry->tag, record + 0x18);
	put_unaligned_le64(entry->mode, record + 0x20);
	return 0;
}

#endif
