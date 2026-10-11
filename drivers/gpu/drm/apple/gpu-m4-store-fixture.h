/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_STORE_FIXTURE_H
#define APPLE_GPU_M4_STORE_FIXTURE_H

#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

/* A narrow offline J713/25G83 fixture, not a firmware packet/UAPI or proof of
 * mappings, ownership, policy, ASID binding, publication or GPU completion.
 * Generated exclusively with unchanged authorized Mesa e6a74082. Source and
 * manifest live in linux-enablement-mac-alpha/tools/m4_gpu_store_fixture/.
 * Offset contract: that tool's fixture_format.h, version 2, little endian.
 */
#define GPU_M4_STORE_HEADER_SIZE 512U
#define GPU_M4_STORE_BUNDLE_SIZE 18176U
#define GPU_M4_STORE_RESOURCE_COUNT 5U
#define GPU_M4_STORE_USC_BASE 0x10000000000ULL
#define GPU_M4_STORE_CONTAINER_VA 0x10000010000ULL
#define GPU_M4_STORE_PROGRAM_VA 0x10000020000ULL
#define GPU_M4_STORE_CDM_VA 0x7100200000ULL
#define GPU_M4_STORE_OUTPUT_VA 0x7100500000ULL
#define GPU_M4_STORE_ENTRY_VA (GPU_M4_STORE_CONTAINER_VA + 0x3c0)
#define GPU_M4_STORE_EXPECTED 0x44a72000U
#define GPU_M4_STORE_SENTINEL 0xdeadbeefU

enum gpu_m4_store_resource_id {
	GPU_M4_STORE_CONTAINER,
	GPU_M4_STORE_PROGRAM,
	GPU_M4_STORE_CDM,
	GPU_M4_STORE_OUTPUT,
	GPU_M4_STORE_USC_WINDOW,
};

/* Independently pinned full bundle hash from the compiler-generated artifact.
 * Embedded payload hashes alone would not authenticate an altered bundle.
 */
static const u8 gpu_m4_store_fixture_sha256[32] = {
	0xb3, 0xeb, 0x64, 0x24, 0x74, 0xb4, 0x09, 0x96,
	0x39, 0xd1, 0x9e, 0x92, 0xb4, 0xf2, 0xc8, 0x6a,
	0x53, 0x3e, 0x80, 0x8f, 0xcb, 0x0c, 0x59, 0x22,
	0xe6, 0x87, 0x15, 0xc4, 0x8e, 0xb6, 0xb9, 0x2a,
};

/* Return 0 only after filling all 32 SHA-256 bytes; negative errno on error.
 * The callback must not modify the immutable bytes or re-enter their owner.
 */
typedef int (*gpu_m4_store_hash)(void *cookie, const void *data, size_t size,
				u8 digest[32]);

struct gpu_m4_store_resource {
	u64 gpu_va;
	const u8 *data;
	u32 size;
};

struct gpu_m4_store_fixture {
	struct gpu_m4_store_resource resource[GPU_M4_STORE_RESOURCE_COUNT];
	u64 usc_base;
	u64 shader_entry;
	u64 cdm_end;
	u32 expected_value;
	u32 main_size;
	u32 nr_gprs;
	u32 staging_halfs;
	u32 compiler_d;
};

static inline bool gpu_m4_store_zero(const u8 *data, size_t size)
{
	size_t i;

	for (i = 0; i < size; i++)
		if (data[i])
			return false;
	return true;
}

/* No stores to result on any rejection. Success borrows immutable blob bytes:
 * retain the firmware object until its contents are copied to privately owned
 * mappings. No parser result grants permission to publish a producer or RUN.
 * This qualified artifact has one fixed profile, not arbitrary program input.
 */
static inline int gpu_m4_store_fixture_parse(const void *blob, size_t size,
					    gpu_m4_store_hash hash, void *cookie,
					    struct gpu_m4_store_fixture *result)
{
	static const u8 magic[16] = "M4STORE-FIXTURE";
	static const u64 resource_va[GPU_M4_STORE_RESOURCE_COUNT] = {
		GPU_M4_STORE_CONTAINER_VA, GPU_M4_STORE_PROGRAM_VA,
		GPU_M4_STORE_CDM_VA, GPU_M4_STORE_OUTPUT_VA,
		GPU_M4_STORE_USC_BASE,
	};
	static const u32 resource_size[GPU_M4_STORE_RESOURCE_COUNT] = { 1024, 90, 48, 4, 0x4000 };
	static const u32 metadata[16] = {
		GPU_M4_STORE_EXPECTED, GPU_M4_STORE_SENTINEL,
		0, 36, 36, 8, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1,
	};
	struct gpu_m4_store_fixture checked = { 0 };
	const u8 *data = blob;
	u8 digest[32];
	u32 cursor = GPU_M4_STORE_HEADER_SIZE;
	unsigned int i;
	int ret;

	if (!data || !hash || !result || size != GPU_M4_STORE_BUNDLE_SIZE)
		return -EINVAL;
	if (memcmp(data, magic, sizeof(magic)) ||
	    get_unaligned_le32(data + 0x10) != 2 ||
	    get_unaligned_le32(data + 0x14) != GPU_M4_STORE_HEADER_SIZE ||
	    get_unaligned_le32(data + 0x18) != size ||
	    get_unaligned_le32(data + 0x1c) != GPU_M4_STORE_RESOURCE_COUNT ||
	    get_unaligned_le64(data + 0x20) != GPU_M4_STORE_USC_BASE ||
	    get_unaligned_le64(data + 0x28) != GPU_M4_STORE_ENTRY_VA ||
	    get_unaligned_le64(data + 0x30) != GPU_M4_STORE_OUTPUT_VA ||
	    !gpu_m4_store_zero(data + 0x78, 8) ||
	    !gpu_m4_store_zero(data + 0x1b8, 0x48))
		return -EINVAL;
	for (i = 0; i < 16; i++)
		if (get_unaligned_le32(data + 0x38 + i * 4) != metadata[i])
			return -EINVAL;

	for (i = 0; i < GPU_M4_STORE_RESOURCE_COUNT; i++) {
		const u8 *descriptor = data + 0xa0 + i * 56;
		u64 va = get_unaligned_le64(descriptor);
		u32 offset = get_unaligned_le32(descriptor + 8);
		u32 length = get_unaligned_le32(descriptor + 12);
		u32 aligned = (cursor + 63) & ~63U;

		if (va != resource_va[i] || length != resource_size[i] ||
		    get_unaligned_le32(descriptor + 48) != i + 1 ||
		    get_unaligned_le32(descriptor + 52) ||
		    offset != aligned || offset > size || length > size - offset ||
		    !gpu_m4_store_zero(data + cursor, offset - cursor))
			return -EINVAL;
		checked.resource[i].gpu_va = va;
		checked.resource[i].data = data + offset;
		checked.resource[i].size = length;
		cursor = offset + length;
	}
	if (cursor != size ||
	    memcmp(checked.resource[GPU_M4_STORE_USC_WINDOW].data,
		   checked.resource[GPU_M4_STORE_CONTAINER].data, 0x340) ||
	    !gpu_m4_store_zero(checked.resource[GPU_M4_STORE_USC_WINDOW].data + 0x340,
			       0x4000 - 0x340) ||
	    get_unaligned_le32(checked.resource[GPU_M4_STORE_CONTAINER].data + 0x340) !=
		1024 - 0x340 ||
	    get_unaligned_le32(checked.resource[GPU_M4_STORE_OUTPUT].data) != GPU_M4_STORE_SENTINEL)
		return -EINVAL;
	/* Fixed complete profile: 40-byte launch, four-byte barrier, four-byte
	 * G16G terminate. A matching word inside the launch is not termination.
	 * The offline producer independently decodes all three Mesa records.
	 */
	if (get_unaligned_le32(checked.resource[GPU_M4_STORE_CDM].data + 40) != 0x60000160 ||
	    get_unaligned_le32(checked.resource[GPU_M4_STORE_CDM].data + 44) != 0x40000000)
		return -EINVAL;

	/* Independent pinned digest first; reject a self-consistent replacement. */
	ret = hash(cookie, data, size, digest);
	if (ret)
		return ret < 0 ? ret : -EIO;
	if (memcmp(digest, gpu_m4_store_fixture_sha256, sizeof(digest)))
		return -EBADMSG;
	for (i = 0; i < GPU_M4_STORE_RESOURCE_COUNT; i++) {
		ret = hash(cookie, checked.resource[i].data, checked.resource[i].size, digest);
		if (ret)
			return ret < 0 ? ret : -EIO;
		if (memcmp(digest, data + 0xa0 + i * 56 + 16, sizeof(digest)))
			return -EBADMSG;
	}
	checked.usc_base = GPU_M4_STORE_USC_BASE;
	checked.shader_entry = GPU_M4_STORE_ENTRY_VA;
	checked.cdm_end = GPU_M4_STORE_CDM_VA + 48;
	checked.expected_value = GPU_M4_STORE_EXPECTED;
	checked.main_size = 36;
	checked.nr_gprs = 8;
	/* staging_halfs and compiler_d remain zero, as explicitly qualified. */
	*result = checked;
	return 0;
}

#endif
