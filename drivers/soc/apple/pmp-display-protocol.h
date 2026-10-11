/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_PMP_DISPLAY_PROTOCOL_H
#define APPLE_PMP_DISPLAY_PROTOCOL_H

#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#define J713_DISP_REQUEST 0x0020f80100000000ULL
#define J713_DISP_REPLY   0x0021000000000000ULL
#define J713_AGX_REQUEST  0x0020f50100000000ULL

enum j713_power_kind {
	J713_POWER_NONE,
	J713_POWER_DISP,
	J713_POWER_AGX,
	J713_POWER_COUNT,
};

static inline bool j713_disp_table(const void *data, size_t size)
{
	const u8 *p = data;

	return p && size == 34 * 0x7c && get_unaligned_le32(p + 15 * 0x7c) == 8 &&
		!memcmp(p + 15 * 0x7c + 0x74, "DISP\0\0\0\0", 8);
}

static inline bool j713_agx_table(const void *data, size_t size)
{
	const u8 *p = data;
	unsigned int offset;
	u32 expected;

	if (!p || size != 34 * 0x7c)
		return false;
	p += 12 * 0x7c;
	/* Qualified raw identity fields, not inferred meanings or writable controls. */
	for (offset = 0; offset <= 0x70; offset += 4) {
		switch (offset) {
		case 0: expected = 5; break;
		case 4: expected = 0x4746585f; break;
		case 8: expected = 3; break;
		case 0xc: case 0x1c: case 0x2c: expected = 2; break;
		default: expected = 0; break;
		}
		if (get_unaligned_le32(p + offset) != expected)
			return false;
	}
	return !memcmp(p + 0x74, "AGX\0\0\0\0\0", 8);
}

/* Caller holds its state lock. Errors permanently invalidate untagged replies. */
struct j713_disp_vote {
	int error;
	enum j713_power_kind pending;
	bool held[J713_POWER_COUNT];
};

static inline void j713_disp_fail(struct j713_disp_vote *s, int error)
{
	if (!s->error)
		s->error = error < 0 ? error : -EIO;
	s->pending = J713_POWER_NONE;
}

/* One means send the request, zero means the same retained vote is held. */
static inline int j713_disp_begin(struct j713_disp_vote *s, enum j713_power_kind kind)
{
	if (kind <= J713_POWER_NONE || kind >= J713_POWER_COUNT)
		return -EINVAL;
	if (s->error)
		return s->error;
	if (s->pending)
		return -EBUSY;
	if (s->held[kind])
		return 0;
	s->pending = kind;
	return 1;
}

static inline void j713_disp_reply(struct j713_disp_vote *s, u8 endpoint, u64 word)
{
	if (s->error)
		return;
	if (!s->pending || endpoint != 0x20 || word != J713_DISP_REPLY) {
		j713_disp_fail(s, -EPROTO);
		return;
	}
	s->held[s->pending] = true;
	s->pending = J713_POWER_NONE;
}

#endif
