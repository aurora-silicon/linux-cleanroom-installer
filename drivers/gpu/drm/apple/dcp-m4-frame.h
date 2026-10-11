/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_DCP_M4_FRAME_H
#define APPLE_DCP_M4_FRAME_H

#include <linux/errno.h>
#include <linux/limits.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#define DCP_M4_SWAP_SIZE		0x1bd8U
#define DCP_M4_SURFACE_SIZE	0x22cU
#define DCP_M4_PANEL_WIDTH	2560U
#define DCP_M4_PANEL_HEIGHT	1664U

/* Used only after matching firmware-advertised preferred physical timing. */
enum dcp_m4_frame_format {
	DCP_M4_XRGB8888,
	DCP_M4_ARGB8888,
};

struct dcp_m4_frame {
	u64 dva;
	u64 backing_size;
	u32 width;
	u32 height;
	u32 stride;
	enum dcp_m4_frame_format format;
};

static inline int dcp_m4_frame_check(const struct dcp_m4_frame *frame)
{
	u64 size = (u64)frame->stride * frame->height;

	if (frame->width != DCP_M4_PANEL_WIDTH || frame->height != DCP_M4_PANEL_HEIGHT ||
	    frame->stride < DCP_M4_PANEL_WIDTH * 4 || frame->stride % 64 ||
	    !size || size > 0xffffffffULL || frame->backing_size < size ||
	    !frame->dva || frame->dva % 0x4000 || frame->dva >= (1ULL << 42) ||
	    size > (1ULL << 42) - frame->dva ||
	    (frame->format != DCP_M4_XRGB8888 && frame->format != DCP_M4_ARGB8888))
		return -EINVAL;
	return 0;
}

/* Display-consumer DMA aperture: the 64-GiB DART range at its 1-TiB alias. */
#define DCP_M4_SCANOUT_BASE	(1ULL << 40)
#define DCP_M4_SCANOUT_SIZE	(1ULL << 36)

/* Accumulate one mapped DMA segment; only one exactly contiguous extent qualifies. */
static inline int dcp_m4_extent_add(u64 *start, u64 *length, u64 addr, u64 size)
{
	if (!size)
		return -EINVAL;
	if (!*length) {
		if (size > U64_MAX - addr)
			return -ERANGE;
		*start = addr;
		*length = size;
		return 0;
	}
	/* Invariant: start + length never wraps. */
	if (addr != *start + *length || size > U64_MAX - addr)
		return -ERANGE;
	*length += size;
	return 0;
}

/* A client mapping must cover the needed bytes inside the SID0 aperture. */
static inline int dcp_m4_direct_extent(u64 dva, u64 mapped, u64 needed)
{
	if (!dva || dva % 0x4000 || !needed || mapped < needed ||
	    dva < DCP_M4_SCANOUT_BASE || dva - DCP_M4_SCANOUT_BASE >= DCP_M4_SCANOUT_SIZE ||
	    mapped > DCP_M4_SCANOUT_SIZE - (dva - DCP_M4_SCANOUT_BASE))
		return -ERANGE;
	return 0;
}

/* Describe a qualified client framebuffer at its SID0 DVA; no CPU access. */
static inline int dcp_m4_direct_frame(struct dcp_m4_frame *frame, u64 dva, u64 mapped,
				      u32 width, u32 height, u32 pitch, bool alpha)
{
	*frame = (struct dcp_m4_frame) {
		.dva = dva, .backing_size = mapped, .width = width, .height = height,
		.stride = pitch, .format = alpha ? DCP_M4_ARGB8888 : DCP_M4_XRGB8888,
	};
	if (dcp_m4_direct_extent(dva, mapped, (u64)pitch * height))
		return -ERANGE;
	return dcp_m4_frame_check(frame);
}

/* Visible diagnostic: eight colour bars with a white border and center cross. */
static inline int dcp_m4_frame_pattern(void *pixels, const struct dcp_m4_frame *frame)
{
	static const u32 colours[8] = { 0xffffffff, 0xffffff00, 0xff00ffff, 0xff00ff00,
		0xffff00ff, 0xffff0000, 0xff0000ff, 0xff202020 };
	int ret = dcp_m4_frame_check(frame);

	if (ret || !pixels)
		return ret ? ret : -EINVAL;
	for (u32 y = 0; y < frame->height; y++) {
		u8 *row = (u8 *)pixels + (size_t)y * frame->stride;

		for (u32 x = 0; x < frame->width; x++) {
			u32 colour = colours[x * 8 / frame->width];

			if (x < 8 || y < 8 || x >= frame->width - 8 || y >= frame->height - 8 ||
			    x == frame->width / 2 || y == frame->height / 2)
				colour = 0xffffffff;
			put_unaligned_le32(colour, row + x * 4);
		}
	}
	return 0;
}

/* Exact single-layer fixture, not an ABI for tiled or compressed surfaces. */
static inline void dcp_m4_surface_encode(u8 *surface, const struct dcp_m4_frame *frame)
{
	u32 size = frame->stride * frame->height;

	memset(surface, 0, DCP_M4_SURFACE_SIZE);
	surface[2] = frame->format == DCP_M4_XRGB8888;
	put_unaligned_le32(1, surface + 3);
	put_unaligned_le32(1, surface + 7);
	put_unaligned_le32(0x42475241, surface + 0x0b);
	surface[0x13] = 13;
	surface[0x14] = 12;
	put_unaligned_le32(frame->stride, surface + 0x15);
	put_unaligned_le16(1, surface + 0x19);
	surface[0x1b] = 1;
	surface[0x1c] = 1;
	put_unaligned_le32(frame->width, surface + 0x21);
	put_unaligned_le32(frame->height, surface + 0x25);
	put_unaligned_le32(size, surface + 0x29);
	put_unaligned_le32(1, surface + 0x35);
	put_unaligned_le64(1, surface + 0x51);
	put_unaligned_le32(frame->width, surface + 0x59);
	put_unaligned_le32(frame->height, surface + 0x5d);
	put_unaligned_le32(frame->stride, surface + 0x69);
	put_unaligned_le32(size, surface + 0x6d);
	put_unaligned_le16(4, surface + 0x71);
	surface[0x73] = 1;
	surface[0x74] = 1;
	put_unaligned_le64(1, surface + 0x149);
}

/* Encode an integer development brightness without floating-point operations. */
static inline u64 dcp_m4_nits_binary64(u32 nits)
{
	unsigned int bit = 0;

	if (!nits)
		return 0;
	while ((nits >> bit) > 1)
		bit++;
	return ((u64)(1023 + bit) << 52) | ((u64)(nits - (1U << bit)) << (52 - bit));
}

/*
 * External outputs: any qualified size up to 3840x2160, same surface layout.
 * The frame must already be described at its wire DVA in an owned mapping.
 */
static inline int dcp_m4_frame_check_ext(const struct dcp_m4_frame *frame)
{
	u64 size = (u64)frame->stride * frame->height;

	if (!frame->width || !frame->height || frame->width > 3840 || frame->height > 2160 ||
	    frame->stride < frame->width * 4 || frame->stride % 64 ||
	    !size || size > 0xffffffffULL || frame->backing_size < size ||
	    !frame->dva || frame->dva % 0x4000 || frame->dva >= (1ULL << 42) ||
	    size > (1ULL << 42) - frame->dva ||
	    (frame->format != DCP_M4_XRGB8888 && frame->format != DCP_M4_ARGB8888))
		return -EINVAL;
	return 0;
}

static inline int dcp_m4_swap_encode_checked(void *output, size_t length, u32 swap_id,
					     const struct dcp_m4_frame *frame,
					     bool brightness_change, u32 nits,
					     int (*check)(const struct dcp_m4_frame *frame))
{
	u8 *data = output;

	if (length != DCP_M4_SWAP_SIZE || !swap_id || (brightness_change && nits > 500))
		return -EINVAL;
	if (frame && check(frame))
		return -EINVAL;
	memset(data, 0, length);
	put_unaligned_le32(swap_id, data + 0x98);
	put_unaligned_le32(0x80000007, data + 0x14c);
	put_unaligned_le32(0x80000007, data + 0x150);
	put_unaligned_le32(frame ? 0xff000000 : 0xff402060, data + 0x154);
	memset(data + 0x1bcb, 1, 10);
	data[0x1bd6] = 1;
	data[0x1bd7] = 1;
	if (brightness_change) {
		data[0x354] = 1;
		put_unaligned_le64(dcp_m4_nits_binary64(nits), data + 0x35e);
	}
	if (frame) {
		put_unaligned_le32(1, data + 0x9c);
		put_unaligned_le32(frame->width, data + 0xb4);
		put_unaligned_le32(frame->height, data + 0xb8);
		put_unaligned_le32(frame->width, data + 0x114);
		put_unaligned_le32(frame->height, data + 0x118);
		dcp_m4_surface_encode(data + 0x588, frame);
		put_unaligned_le64(frame->dva, data + 0xe38);
		data[0x1bcb] = 0;
	}
	return 0;
}

static inline int dcp_m4_swap_encode(void *output, size_t length, u32 swap_id,
				     const struct dcp_m4_frame *frame,
				     bool brightness_change, u32 nits)
{
	return dcp_m4_swap_encode_checked(output, length, swap_id, frame, brightness_change,
					  nits, dcp_m4_frame_check);
}

static inline int dcp_m4_swap_start_reply(const void *input, size_t size, u32 *swap_id)
{
	const u8 *data = input;
	u32 id;

	if (size != 8)
		return -EMSGSIZE;
	id = get_unaligned_le32(data);
	if (!id || get_unaligned_le32(data + 4))
		return -EPROTO;
	*swap_id = id;
	return 0;
}

/* A407 reply and D590 are independent gates; callers must wait for both. */
static inline int dcp_m4_swap_submit_reply(const void *input, size_t size)
{
	const u8 *data = input;

	if (size != 12)
		return -EMSGSIZE;
	return get_unaligned_le32(data + 5) ? -EPROTO : 0;
}

static inline int dcp_m4_swap_complete(const void *input, size_t size, u32 pending_id)
{
	if (size != 0x730)
		return -EMSGSIZE;
	return pending_id && get_unaligned_le32(input) == pending_id ? 0 : -EPROTO;
}

/* Serialized owner only. The caller retains both scanout buffers on any error. */
struct dcp_m4_swap_state {
	u32 id;
	int error;
	bool replied;
	bool completed;
	bool emitted;
};

static inline int dcp_m4_swap_fail(struct dcp_m4_swap_state *swap, int error)
{
	if (!swap->error)
		swap->error = error < 0 ? error : -EPROTO;
	return swap->error;
}

/* Begin after A406 supplies its validated ID, before publishing A407. */
static inline int dcp_m4_swap_begin(struct dcp_m4_swap_state *swap, u32 id)
{
	if (swap->error)
		return swap->error;
	if (!id)
		return dcp_m4_swap_fail(swap, -EINVAL);
	if (swap->id && !swap->emitted)
		return dcp_m4_swap_fail(swap, -EBUSY);
	*swap = (struct dcp_m4_swap_state){.id = id};
	return 0;
}

static inline int dcp_m4_swap_record_reply(struct dcp_m4_swap_state *swap,
					 const void *reply, size_t size)
{
	int ret;

	if (swap->error)
		return swap->error;
	if (!swap->id || swap->replied)
		return dcp_m4_swap_fail(swap, -EPROTO);
	ret = dcp_m4_swap_submit_reply(reply, size);
	if (ret)
		return dcp_m4_swap_fail(swap, ret);
	swap->replied = true;
	return 0;
}

/* Called by the exact-shape-validated D590 hook; callback alone is not a flip. */
static inline int dcp_m4_swap_record_done(struct dcp_m4_swap_state *swap, u32 id)
{
	if (swap->error)
		return swap->error;
	if (!swap->id || id != swap->id || swap->completed)
		return dcp_m4_swap_fail(swap, -EPROTO);
	swap->completed = true;
	return 0;
}

/* One completion event, only after both the A407 reply and matching D590. */
static inline int dcp_m4_swap_take_completion(struct dcp_m4_swap_state *swap)
{
	if (swap->error)
		return swap->error;
	if (!swap->id || !swap->replied || !swap->completed || swap->emitted)
		return 0;
	swap->emitted = true;
	return 1;
}

#endif
