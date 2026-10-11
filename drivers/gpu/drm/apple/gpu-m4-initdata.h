/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_INITDATA_H
#define APPLE_GPU_M4_INITDATA_H

#include "gpu-m4-format.h"

/* Exact J713/25G83 object inventory; no implicit firmware allocation or publish. */
enum gpu_m4_object_id {
	GPU_M4_ROOT, GPU_M4_BRN, GPU_M4_RUNTIME, GPU_M4_GLOBALS,
	GPU_M4_CONTROL, GPU_M4_FIRMWARE_DATA, GPU_M4_POWER, GPU_M4_DYNAMIC,
	GPU_M4_MAIN, GPU_M4_POWER_PERFORMANCE, GPU_M4_TIMER,
	GPU_M4_ACTIVITY_TILER, GPU_M4_ACTIVITY_FRAGMENT, GPU_M4_ACTIVITY_COMPUTE,
	GPU_M4_ACTIVITY_RECOVERY, GPU_M4_LOG_STATE, GPU_M4_LOG_ENTRIES,
	GPU_M4_LOG_DATA, GPU_M4_FWCTL, GPU_M4_DEVCTRL_STATE, GPU_M4_DEVCTRL_RING,
	GPU_M4_KTRACE_STATE, GPU_M4_KTRACE_DATA, GPU_M4_STATISTICS_STATE,
	GPU_M4_STATISTICS_DATA, GPU_M4_TEMPERATURE, GPU_M4_EVENT_STATE,
	GPU_M4_EVENT_DATA, GPU_M4_RETENTION, GPU_M4_UMA, GPU_M4_PARAMETER,
	GPU_M4_OBJECT_COUNT,
};

static const u32 gpu_m4_object_size[GPU_M4_OBJECT_COUNT] = {
	0xc8, 0x4000, 0x4b8, 0xe48, 0x20, 0xeaf0, 0x670, 0x4000,
	0x2658, 0x6578, 0x88, 0xc18, 0x1248, 0xe10, 0x60, 0x1b0,
	0x28800, 0x79800, 0x4000, 0x30, 0x4000, 0x30, 0x9000, 0x30,
	0x4800, 0x4, 0x30, 0x4800, 0x4000, 0x4000, 0x4000,
};

/* These validate encoding bounds, not the UAT translation or object lifetime. */
static inline bool gpu_m4_object_va(u64 va, u32 size)
{
	return va >= GPU_M4_HOST_START && va < GPU_M4_HOST_END && !(va & 7) &&
		size && size <= GPU_M4_HOST_END - va;
}

static inline int gpu_m4_encode_root(void *root, const u64 va[GPU_M4_OBJECT_COUNT])
{
	static const u8 ids[] = { GPU_M4_BRN, GPU_M4_RUNTIME, GPU_M4_GLOBALS,
		GPU_M4_CONTROL, GPU_M4_FIRMWARE_DATA, GPU_M4_POWER, GPU_M4_DYNAMIC };
	static const u8 offsets[] = { 0x08, 0x18, 0x20, 0xa8, 0xb0, 0xb8, 0xc0 };
	static const u8 shifts[] = { 36, 25, 14 };
	static const u16 counts[] = { 64, 2048, 2048 };

	for (unsigned int i = 0; i < GPU_M4_OBJECT_COUNT; i++) {
		if (!gpu_m4_object_va(va[i], gpu_m4_object_size[i]))
			return -EINVAL;
		for (unsigned int j = 0; j < i; j++)
			if (va[i] < va[j] + gpu_m4_object_size[j] &&
			    va[j] < va[i] + gpu_m4_object_size[i])
				return -EINVAL;
	}
	memset(root, 0, 0xc8);
	put_unaligned_le64(0x0c89c357839204b8ULL, root);
	for (unsigned int i = 0; i < 7; i++)
		put_unaligned_le64(va[ids[i]], root + offsets[i]);
	put_unaligned_le64(1ULL << 32, root + 0x28);
	put_unaligned_le32(0x030e4000, root + 0x30);
	for (unsigned int i = 0; i < 3; i++) {
		u8 *desc = root + 0x34 + i * 0x20;

		desc[0] = 8;
		desc[1] = desc[2] = 14;
		desc[3] = shifts[i];
		put_unaligned_le16(counts[i], desc + 4);
		put_unaligned_le16(0x4000, desc + 6);
		put_unaligned_le64(1, desc + 8);
		put_unaligned_le64(GPU_M4_PA_MASK, desc + 0x10);
		put_unaligned_le64(((u64)counts[i] - 1) << shifts[i], desc + 0x18);
	}
	return 0;
}

static inline int gpu_m4_encode_channel(void *record, u64 state, u64 ring, bool control)
{
	u32 bytes = control ? 0x4000 : 0x1800;

	if (!gpu_m4_object_va(state, 0x30) || !gpu_m4_object_va(ring, bytes) ||
	    (state < ring + bytes && ring < state + 0x30))
		return -EINVAL;
	put_unaligned_le64(state, record);
	put_unaligned_le64(state + 0x10, record + 8);
	put_unaligned_le64(state + 0x20, record + 0x10);
	put_unaligned_le64(ring, record + 0x18);
	return 0;
}

struct gpu_m4_initdata_input {
	void *object[GPU_M4_OBJECT_COUNT];
	u64 va[GPU_M4_OBJECT_COUNT];
	u64 channel_state[12], channel_ring[12];
	u64 loader_va[GPU_M4_LOADER_COUNT];
	u64 scratch, lower_uma, lower_parameter;
};

static inline bool gpu_m4_bootstrap_alias(u64 va)
{
	return va >= 0x7100000000ULL && va <= 0x8000000000ULL - 0x8000 &&
		!(va & 0x3fff);
}

/* Construct fresh private objects only. Caller verifies mappings before publication. */
static inline int gpu_m4_encode_initdata(const struct gpu_m4_initdata_input *in, u32 ceiling)
{
	static const struct { u16 offset; u8 object; } runtime_ptrs[] = {
		{ 0x000, GPU_M4_MAIN }, { 0x010, GPU_M4_TIMER },
		{ 0x1c0, GPU_M4_EVENT_STATE }, { 0x1c8, GPU_M4_EVENT_DATA },
		{ 0x1d0, GPU_M4_LOG_STATE }, { 0x1d8, GPU_M4_LOG_ENTRIES },
		{ 0x1e0, GPU_M4_KTRACE_STATE }, { 0x1e8, GPU_M4_KTRACE_DATA },
		{ 0x1f0, GPU_M4_STATISTICS_STATE }, { 0x1f8, GPU_M4_STATISTICS_DATA },
		{ 0x200, GPU_M4_LOG_DATA }, { 0x254, GPU_M4_ACTIVITY_TILER },
		{ 0x25c, GPU_M4_ACTIVITY_FRAGMENT }, { 0x264, GPU_M4_ACTIVITY_COMPUTE },
		{ 0x26c, GPU_M4_ACTIVITY_RECOVERY }, { 0x2d8, GPU_M4_PARAMETER },
		{ 0x2e8, GPU_M4_UMA }, { 0x469, GPU_M4_POWER_PERFORMANCE },
	};
	static const struct { u16 offset; u32 value; } globals[] = {
		{ 0x78, 1 }, { 0xe28, 3 }, { 0x24, 3000 }, { 0x998, 40 },
		{ 0x99c, 10 }, { 0x9a0, 250 }, { 0x9b8, 2 }, { 0x9bc, 40 },
		{ 0x9c0, 5 }, { 0x9c8, 40 }, { 0x9cc, 50 },
	};
	static const struct { u16 offset; u32 value; } main[] = {
		{ 0xea0, 1 }, { 0xee0, 1 }, { 0xeb8, 1 }, { 0xed0, 24000 },
		{ 0xed4, 1 }, { 0xed8, 8 }, { 0xfc0, 10 }, { 0x259c, 1 },
		{ 0x2544, 4 }, { 0x2548, 6 }, { 0x254c, 3 }, { 0x2550, 7 },
		{ 0x2554, 7 }, { 0x2568, 8 }, { 0x2570, 1 },
	};
	static const u16 bounds[] = {
		0x9f0, 0x9f4, 0xa38, 0xaa8, 0xaac, 0xaf0, 0xb78, 0xb7c,
		0xb80, 0xbc0, 0x22b0, 0x22b4, 0x22f8, 0x2210, 0x2870,
		0x2a3c, 0x2b84, 0x2b88, 0x2b8c, 0x2bcc, 0x2c40, 0x2c44,
		0x2c88, 0x279e,
	};
	static const u16 frequency[] = { 0, 338, 618, 796, 928, 1056, 1170, 1278, 1338, 1470 };
	static const u16 auxiliary[] = { 0, 338, 618, 796, 836, 952, 1053, 1152, 1204, 1326 };
	static const u16 voltage[] = { 125, 610, 645, 680, 725, 780, 825, 865, 895, 970 };
	static const u16 sram[] = { 780, 780, 780, 780, 780, 780, 825, 865, 895, 970 };
	u8 *runtime = in->object[GPU_M4_RUNTIME], *m = in->object[GPU_M4_MAIN];
	u8 *p = in->object[GPU_M4_POWER_PERFORMANCE], *g = in->object[GPU_M4_GLOBALS];
	u8 *fw = in->object[GPU_M4_FIRMWARE_DATA];
	int ret;

	if (!ceiling || ceiling > 9 || !gpu_m4_object_va(in->scratch, 0x8000) ||
	    !gpu_m4_bootstrap_alias(in->lower_uma) ||
	    !gpu_m4_bootstrap_alias(in->lower_parameter) ||
	    (in->lower_uma < in->lower_parameter + 0x8000 &&
	     in->lower_parameter < in->lower_uma + 0x8000))
		return -EINVAL;
	for (unsigned int i = 0; i < GPU_M4_OBJECT_COUNT; i++)
		if (!in->object[i])
			return -EINVAL;
	ret = gpu_m4_encode_root(in->object[GPU_M4_ROOT], in->va);
	if (ret)
		return ret;
	for (unsigned int i = 1; i < GPU_M4_OBJECT_COUNT; i++)
		memset(in->object[i], 0, gpu_m4_object_size[i]);
	for (unsigned int i = 0; i < sizeof(runtime_ptrs) / sizeof(runtime_ptrs[0]); i++)
		put_unaligned_le64(in->va[runtime_ptrs[i].object], runtime + runtime_ptrs[i].offset);
	put_unaligned_le64(in->scratch, runtime + 0x18);
	put_unaligned_le64(in->lower_parameter, runtime + 0x2d0);
	put_unaligned_le64(in->lower_uma, runtime + 0x2e0);
	put_unaligned_le32(1, runtime + 0x250);
	put_unaligned_le32(4, runtime + 0x2f8);
	for (unsigned int i = 0; i < 12; i++) {
		ret = gpu_m4_encode_channel(runtime + 0x20 + i * 0x20,
					    in->channel_state[i], in->channel_ring[i], false);
		if (ret)
			return ret;
	}
	ret = gpu_m4_encode_channel(runtime + 0x1a0, in->va[GPU_M4_DEVCTRL_STATE],
				    in->va[GPU_M4_DEVCTRL_RING], true);
	if (ret)
		return ret;
	put_unaligned_le64(in->va[GPU_M4_FWCTL], fw + 0x4f88);
	put_unaligned_le64(in->va[GPU_M4_FWCTL] + 0x40, fw + 0x4f90);
	put_unaligned_le32(1, fw + 0x4fd4);
	put_unaligned_le32(0x33, in->object[GPU_M4_CONTROL]);
	put_unaligned_le32(0xffffffff, in->object[GPU_M4_ACTIVITY_FRAGMENT] + 0xc18);
	put_unaligned_le32(0xffffffff, in->object[GPU_M4_ACTIVITY_FRAGMENT] + 0xc30);
	for (unsigned int i = 0; i < sizeof(globals) / sizeof(globals[0]); i++)
		put_unaligned_le32(globals[i].value, g + globals[i].offset);
	put_unaligned_le32(100 * ceiling, g + 0x88);
	put_unaligned_le32(100 * ceiling, g + 0x8c);
	put_unaligned_le32(100 * ceiling, g + 0x98);
	put_unaligned_le64(GPU_M4_HOST_END, m + 0x28);
	put_unaligned_le64(in->va[GPU_M4_RETENTION], m + 0xe88);
	for (unsigned int i = 0; i < sizeof(main) / sizeof(main[0]); i++)
		put_unaligned_le32(main[i].value, m + main[i].offset);
	put_unaligned_le64(0x0000000607002004ULL, m + 0xf24);
	put_unaligned_le32(ceiling, m + 0xfc4);
	for (unsigned int i = 0; i <= ceiling; i++) {
		put_unaligned_le32(frequency[i], m + 0xfc8 + i * 4);
		put_unaligned_le32(auxiliary[i], m + 0x1808 + i * 4);
		for (unsigned int core = 0; core < 16; core++) {
			put_unaligned_le32(voltage[i], m + 0x1008 + (i * 16 + core) * 4);
			put_unaligned_le32(sram[i], m + 0x1408 + (i * 16 + core) * 4);
		}
	}
	for (unsigned int slot = 0; slot < GPU_M4_LOADER_COUNT; slot++) {
		ret = gpu_m4_loader_encode(slot, in->loader_va[slot], m + 0x640 + slot * 0x28);
		if (ret)
			return ret;
	}
	put_unaligned_le64(in->va[GPU_M4_TEMPERATURE], p + 0x3b50);
	put_unaligned_le32(200000, p + 4);
	put_unaligned_le32(200000, p + 8);
	put_unaligned_le32(4, p + 0x10);
	put_unaligned_le32(0x3f800000, p + 0x14);
	put_unaligned_le32(ceiling, p + 0x2c);
	put_unaligned_le32(ceiling, p + 0x30);
	for (unsigned int i = 0; i < sizeof(bounds) / sizeof(bounds[0]); i++)
		put_unaligned_le32(100 * ceiling, p + bounds[i]);
	return 0;
}

#endif
