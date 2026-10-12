/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_DCP_M4_MEMORY_H
#define APPLE_DCP_M4_MEMORY_H

#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include "dcp-m4-profile.h"

#define DCP_M4_MEMORY_OBJECTS 64
#define DCP_M4_MEMORY_LIMIT (64U << 20)
#define DCP_M4_MEMORY_MAX (16U << 20)
#define DCP_M4_MEMORY_PAGE 0x4000U

struct dcp_m4_memory_object {
	u64 phys, dva, size;
	bool mapped, retired;
};

/* Heap-backed, single serialized callback owner. Retired IDs are never reused. */
struct dcp_m4_memory {
	struct dcp_m4_memory_object object[DCP_M4_MEMORY_OBJECTS];
	u32 count, bytes;
	bool failed;
};

struct dcp_m4_memory_ops {
	/* Must reserve both SID23/SID16 slots, pin backing, then publish only SID23. */
	int (*alloc)(void *cookie, u32 size, u64 *phys, u64 *dva);
	/* Must map the same backing at the same DVA in SID16, flush and verify. */
	int (*map)(void *cookie, u64 phys, u64 dva, u32 size);
};

/* Called only after the dispatcher validates exact callback input/output sizes. */
static inline int dcp_m4_memory_callback(struct dcp_m4_memory *memory,
					 const struct dcp_m4_memory_ops *ops, void *cookie,
					 unsigned int method, const void *input, void *output)
{
	struct dcp_m4_memory_object *object;
	const u8 *in = input;
	u8 *out = output;
	u64 size, phys = 0, dva = 0, id;
	u32 align;
	int ret = -EINVAL;

	if (memory->failed)
		return -EIO;
	if (method == 451) {
		size = get_unaligned_le64(in + 4);
		align = get_unaligned_le32(in + 12);
		/* J713 asks for exactly 4; firmware 14.7 any power of two <= 16 KiB. */
		if (get_unaligned_le32(in) != 0x703 || !size || size > DCP_M4_MEMORY_MAX ||
		    (dcp_m4_pf->fw14 ? !align || (align & (align - 1)) || align > DCP_M4_MEMORY_PAGE :
				      align != 4) || in[16] || in[17] || in[18])
			goto failed;
		/* Byte 19 is padding, observed nonzero in otherwise valid requests. */
		size = (size + DCP_M4_MEMORY_PAGE - 1) & ~(u64)(DCP_M4_MEMORY_PAGE - 1);
		if (memory->count >= DCP_M4_MEMORY_OBJECTS || memory->bytes > DCP_M4_MEMORY_LIMIT - size) {
			ret = -ENOSPC;
			goto failed;
		}
		if (!ops->alloc) {
			ret = -EOPNOTSUPP;
			goto failed;
		}
		ret = ops->alloc(cookie, size, &phys, &dva);
		if (ret)
			goto failed;
		/* Never publish malformed allocator output to the firmware. */
		if (!phys || ((phys | dva) & (DCP_M4_MEMORY_PAGE - 1)) ||
		    phys >= (1ULL << 42) || size > (1ULL << 42) - phys ||
		    dva < 0x10000000000ULL || dva >= 0x11000000000ULL ||
		    size > 0x11000000000ULL - dva) {
			ret = -ERANGE;
			goto failed;
		}
		for (unsigned int i = 0; i < memory->count; i++) {
			object = &memory->object[i];
			if ((dva < object->dva + object->size && object->dva < dva + size) ||
			    (phys < object->phys + object->size && object->phys < phys + size)) {
				ret = -EEXIST;
				goto failed;
			}
		}
		object = &memory->object[memory->count++];
		*object = (struct dcp_m4_memory_object){ .phys = phys, .dva = dva, .size = size };
		memory->bytes += size;
		put_unaligned_le64(phys, out);
		put_unaligned_le64(dva, out + 8);
		put_unaligned_le64(size, out + 16);
		put_unaligned_le32(memory->count, out + 24);
		return 0;
	}
	if (method != 201 && method != 454) {
		ret = -EOPNOTSUPP;
		goto failed;
	}
	id = method == 201 ? get_unaligned_le64(in) : get_unaligned_le32(in);
	if (!id || id > memory->count || (method == 201 && get_unaligned_le32(in + 8) != 1))
		goto failed;
	object = &memory->object[id - 1];
	if (object->retired)
		goto failed;
	if (method == 454) {
		/* No fence exists: retain backing, both translations and the ID. */
		object->retired = true;
		put_unaligned_le32(1, out);
		return 0;
	}
	/* Firmware 14.7 may repeat D201 for an already mapped live ID. */
	if (object->mapped && dcp_m4_pf->fw14)
		goto reply;
	if (object->mapped)
		goto failed;
	if (!ops->map) {
		ret = -EOPNOTSUPP;
		goto failed;
	}
	ret = ops->map(cookie, object->phys, object->dva, object->size);
	if (ret)
		goto failed;
	object->mapped = true;
reply:
	if (dcp_m4_pf->fw14) {
		/* 12/20: AP vaddr 0 at 0, mapped DVA at 8, status 0 at 16. */
		put_unaligned_le64(0, out);
		put_unaligned_le64(object->dva, out + 8);
		put_unaligned_le32(0, out + 16);
		return 0;
	}
	put_unaligned_le32(id, out);
	put_unaligned_le64(object->dva, out + 4);
	put_unaligned_le32(0, out + 12);
	return 0;
failed:
	memory->failed = true;
	return ret < 0 ? ret : -EPROTO;
}

#endif
