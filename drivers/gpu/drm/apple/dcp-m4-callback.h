/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_DCP_M4_CALLBACK_H
#define APPLE_DCP_M4_CALLBACK_H

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include "dcp-m4-profile.h"

/* All hooks execute under the serialized transaction owner, never in IRQ context. */
struct dcp_m4_callback_ops {
	int (*call)(void *cookie, unsigned int method, bool remote,
		    const void *input, u32 input_size, void *output, u32 output_size);
	int (*swap_done)(void *cookie, u32 id);
	/* Copy the bounded name and compute/retain any digest before packet ACK/reuse. */
	int (*event)(void *cookie, const char *name, const void *dictionary, size_t size);
	int (*property)(void *cookie, unsigned int method, const void *input, void *output);
	int (*memory)(void *cookie, unsigned int method, const void *input, void *output);
	/* Firmware 14.7 D209: wall-clock milliseconds. */
	u64 (*wall_ms)(void *cookie);
	u32 boot_stride;
};

struct dcp_m4_callback_shape {
	u16 method;
	u16 input;
	u16 output;
};

static inline int dcp_m4_callback_result(int ret)
{
	return ret > 0 ? -EPROTO : ret;
}

/* Exact qualified J713 shapes. An absent behavior hook is failure, never an ACK. */
static const struct dcp_m4_callback_shape dcp_m4_callbacks[] = {
	{0, 0, 4}, {1, 0, 4}, {3, 4, 60}, {6, 84, 80},
	{100, 0, 0}, {101, 0, 4}, {102, 68, 0}, {104, 68, 0}, {107, 64, 0},
	{108, 0, 4}, {109, 0, 4}, {110, 0, 4}, {111, 0, 4}, {112, 0, 4}, {113, 0, 4},
	{114, 4164, 4100}, {121, 0, 4}, {122, 0, 4}, {123, 0, 4},
	{125, 100, 36}, {127, 4, 4}, {128, 4104, 4}, {129, 64, 4},
	{201, 12, 16}, {206, 0, 4}, {207, 0, 4}, {300, 16, 0},
	{400, 76, 3076}, {401, 80, 12}, {406, 72, 0}, {408, 8, 8},
	{413, 4168, 4}, {414, 80, 4}, {415, 76, 4}, {451, 20, 28}, {454, 4, 4},
	{552, 4164, 4}, {561, 4164, 4}, {563, 76, 4}, {565, 72, 4}, {567, 128, 4},
	{575, 88, 76}, {576, 4, 0}, {582, 8, 4}, {588, 0, 0}, {590, 1840, 0}, {599, 0, 0},
};

/* J514S firmware 14.7.0 shapes (input/output bytes). */
static const struct dcp_m4_callback_shape dcp_m4_callbacks_fw14[] = {
	{0, 0, 4}, {1, 0, 4}, {3, 4, 60}, {6, 60, 56},
	{100, 0, 0}, {101, 0, 4}, {102, 68, 0}, {104, 68, 0}, {107, 64, 0},
	{108, 0, 4}, {109, 0, 4}, {110, 0, 4}, {111, 0, 4}, {112, 0, 4}, {113, 0, 4},
	{114, 4164, 4100}, {115, 16, 8}, {116, 16, 4}, {121, 0, 4}, {122, 0, 4}, {123, 0, 4},
	{125, 100, 36}, {127, 4, 4}, {128, 4104, 4}, {129, 64, 4},
	{201, 12, 20}, {206, 0, 4}, {207, 0, 4}, {209, 0, 8}, {300, 16, 0},
	{400, 76, 3076}, {401, 80, 12}, {406, 72, 0}, {408, 8, 8},
	{413, 4168, 4}, {414, 80, 4}, {415, 76, 4}, {451, 20, 28}, {454, 4, 4},
	{552, 4164, 4}, {561, 4164, 4}, {563, 76, 4}, {565, 72, 4}, {567, 128, 4},
	{576, 88, 76}, {577, 4, 0}, {582, 8, 4}, {588, 8, 0}, {589, 0x6f0, 0},
	{596, 0, 4}, {598, 0, 0}, {599, 0, 0},
};

static inline int dcp_m4_callback_method(u32 tag)
{
	unsigned int method = 0;

	if ((tag & 0xff) != 'D')
		return -EPROTO;
	for (unsigned int shift = 8; shift < 32; shift += 8) {
		u8 digit = tag >> shift;

		if (digit < '0' || digit > '9')
			return -EPROTO;
		method = method * 10 + digit - '0';
	}
	return method;
}

static inline int dcp_m4_callback_shape(u32 tag, u32 input, u32 output)
{
	int method = dcp_m4_callback_method(tag);

	const struct dcp_m4_callback_shape *table = dcp_m4_pf->fw14 ?
		dcp_m4_callbacks_fw14 : dcp_m4_callbacks;
	unsigned int count = dcp_m4_pf->fw14 ? ARRAY_SIZE(dcp_m4_callbacks_fw14) :
		ARRAY_SIZE(dcp_m4_callbacks);

	if (method < 0)
		return method;
	for (unsigned int i = 0; i < count; i++)
		if (table[i].method == method)
			return input == table[i].input &&
				output == table[i].output ? method : -EMSGSIZE;
	return -EOPNOTSUPP;
}

static inline int dcp_m4_nested_call(const struct dcp_m4_callback_ops *ops, void *cookie,
				    bool remote, unsigned int method, bool input_one,
				    int expected)
{
	u8 input[4], output[4] = {0};
	int ret;

	if (!ops->call)
		return -EOPNOTSUPP;
	put_unaligned_le32(1, input);
	/* expected: -1 no output, -2 four-byte output of any value. */
	ret = ops->call(cookie, method, remote, input_one ? input : NULL, input_one ? 4 : 0,
			expected == -1 ? NULL : output, expected == -1 ? 0 : 4);
	if (ret)
		return ret < 0 ? ret : -EPROTO;
	if (expected >= 0 && get_unaligned_le32(output) != (u32)expected)
		return -EPROTO;
	return 0;
}

/*
 * The transaction layer has already located and owns non-overlapping packet
 * input/output spans. It must poison the session on any error here, and send an
 * ACK only after successful dispatch and device-visible output publication.
 */
static inline int dcp_m4_callback_dispatch(const struct dcp_m4_callback_ops *ops, void *cookie,
					 u32 tag, bool remote, const void *input, u32 input_size,
					 void *output, u32 output_size)
{
	const u8 *in = input;
	u8 *out = output;
	int method, ret;
	u32 value;

	method = dcp_m4_callback_shape(tag, input_size, output_size);
	if (method < 0)
		return method;
	if (!ops || (input_size && !input) || (output_size && !output))
		return -EINVAL;
	if (output_size)
		memset(out, 0, output_size);
	if (dcp_m4_pf->fw14) {
		switch (method) {
		case 6:
			if (!(in[56] & 1))
				memcpy(out, in, 56);
			return 0;
		case 115:
			if (in[12] > 1)
				return -EINVAL;
			put_unaligned_le32(1, out + 4);
			return 0;
		case 116: case 577: case 588: case 596: case 598:
			return 0;
		case 209:
			if (!ops->wall_ms)
				return -EOPNOTSUPP;
			put_unaligned_le64(ops->wall_ms(cookie), out);
			return 0;
		case 408:
			value = get_unaligned_le32(in + 4);
			if (memcmp(in, "VORP", 4) || value > 1)
				return -EINVAL;
			put_unaligned_le64(value ? 0 : 712000000, out);
			return 0;
		case 401:
			/* miniLED compatibility placeholder, never a sensor input. */
			if (!memcmp(in, "SUMP", 4) && !strcmp((const char *)in + 4, "Temperature")) {
				put_unaligned_le64(3029, out);
				out[8] = 1;
				return 0;
			}
			break;
		case 576:
			if (!(in[84] & 1))
				memcpy(out, in + 8, 76);
			return 0;
		case 589:
			value = get_unaligned_le32(in);
			if (!value)
				return -EPROTO;
			return ops->swap_done ? dcp_m4_callback_result(ops->swap_done(cookie, value)) :
				-EOPNOTSUPP;
		case 599:
			return dcp_m4_nested_call(ops, cookie, remote, 410, false, 1);
		case 100:
			return dcp_m4_nested_call(ops, cookie, remote, 374, false, -2);
		case 121:
			ret = dcp_m4_nested_call(ops, cookie, remote, 444, false, 1);
			if (!ret)
				ret = dcp_m4_nested_call(ops, cookie, remote, 29, false, -1);
			if (!ret)
				ret = dcp_m4_nested_call(ops, cookie, remote, 466, true, -1);
			if (!ret)
				ret = dcp_m4_nested_call(ops, cookie, remote, 0, true, 1);
			if (!ret)
				ret = dcp_m4_nested_call(ops, cookie, remote, 463, false, 1);
			if (ret)
				return ret;
			put_unaligned_le32(1, out);
			return 0;
		}
	}
	switch (method) {
	case 0: case 1: case 108: case 109: case 110: case 111: case 112: case 113:
		put_unaligned_le32(1, out);
		return 0;
	case 3:
		put_unaligned_le32(1, out + 56); /* Decline optional bandwidth service. */
		return 0;
	case 6:
		if (!(in[80] & 1))
			memcpy(out, in, 80);
		return 0;
	case 575:
		if (!(in[84] & 1))
			memcpy(out, in + 8, 76);
		return 0;
	case 300: case 406: case 576: case 588:
		return 0;
	case 122: case 123: case 582:
		return 0; /* False, not an unimplemented service returning success. */
	case 101:
		put_unaligned_le32(dcp_m4_pf->boot_stride, out);
		return 0;
	case 125:
		value = get_unaligned_le32(in + 64);
		if (value > 8)
			return -EINVAL;
		memcpy(out, in + 68, value * 4);
		return 0;
	case 408:
		value = get_unaligned_le32(in + 4);
		if (memcmp(in, "VORP", 4) || value > 1)
			return -EINVAL;
		put_unaligned_le64(value ? 24000000 : 48000000, out);
		return 0;
	case 590:
		value = get_unaligned_le32(in);
		if (!value)
			return -EPROTO;
		return ops->swap_done ? dcp_m4_callback_result(ops->swap_done(cookie, value)) :
			-EOPNOTSUPP;
	case 599:
		return dcp_m4_nested_call(ops, cookie, remote, 412, false, 1);
	case 100:
		return dcp_m4_nested_call(ops, cookie, remote, 390, false, 0);
	case 206: case 207:
		ret = dcp_m4_nested_call(ops, cookie, remote, method == 206 ? 131 : 132, false, 0);
		if (ret)
			return ret;
		put_unaligned_le32(1, out);
		return 0;
	case 121:
		ret = dcp_m4_nested_call(ops, cookie, remote, 389, false, -1);
		if (!ret)
			ret = dcp_m4_nested_call(ops, cookie, remote, 446, false, 1);
		if (!ret)
			ret = dcp_m4_nested_call(ops, cookie, remote, 29, false, -1);
		if (!ret)
			ret = dcp_m4_nested_call(ops, cookie, remote, 468, true, -1);
		if (!ret)
			ret = dcp_m4_nested_call(ops, cookie, remote, 0, true, 1);
		if (!ret)
			ret = dcp_m4_nested_call(ops, cookie, remote, 465, false, 1);
		if (ret)
			return ret;
		put_unaligned_le32(1, out);
		return 0;
	case 114:
		if (!memchr(in, 0, 64) || in[4160] > 1 || (!in[4160] && in[64] != 'd'))
			return -EINVAL;
		if (!ops->event)
			return -EOPNOTSUPP;
		ret = ops->event(cookie, (const char *)in, (in[4160] & 1) ? NULL : in + 64,
				(in[4160] & 1) ? 0 : 4096);
		if (ret)
			return ret < 0 ? ret : -EPROTO;
		if (!(in[4160] & 1))
			memcpy(out, in + 64, 4096);
		return 0;
	case 201: case 451: case 454:
		return ops->memory ?
			dcp_m4_callback_result(ops->memory(cookie, method, input, output)) : -EOPNOTSUPP;
	default:
		return ops->property ?
			dcp_m4_callback_result(ops->property(cookie, method, input, output)) : -EOPNOTSUPP;
	}
}

#endif
