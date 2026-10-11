/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_BOOTSTRAP_H
#define APPLE_GPU_M4_BOOTSTRAP_H

#include "gpu-m4-initdata.h"

/* Deliberately bounded, stopped-ASC bootstrap: one leaf table per VA window.
 * Not a client VM allocator, and never a live map/unmap implementation.
 */
#define GPU_M4_BOOTSTRAP_LOWER 0x7100000000ULL
#define GPU_M4_BOOTSTRAP_SPAN 0x2000000ULL

struct gpu_m4_bootstrap_view {
	const void *root, *middle, *leaf;
	u64 middle_pa, leaf_pa;
	bool lower, running;
};

/* Walk the actual retained pages. Resolve only the two owned child PAs: an
 * unfamiliar descriptor must never become an arbitrary CPU dereference.
 */
static inline int gpu_m4_bootstrap_walk(const struct gpu_m4_bootstrap_view *v,
				      u64 va, u64 *pa, u64 *attrs)
{
	u64 base = v->lower ? GPU_M4_BOOTSTRAP_LOWER : GPU_M4_HOST_START;
	u64 parent, pte;

	if (!v->root || !v->middle || !v->leaf || va < base ||
	    va >= base + GPU_M4_BOOTSTRAP_SPAN ||
	    !v->middle_pa || !v->leaf_pa ||
	    (v->middle_pa & ~GPU_M4_PA_MASK) || (v->leaf_pa & ~GPU_M4_PA_MASK))
		return -EINVAL;
	parent = get_unaligned_le64(v->root + ((va >> 36) & 63) * 8);
	if (parent != (v->middle_pa | 3) &&
	    (v->lower || !v->running ||
	     parent != (v->middle_pa | 0x00200000000004cbULL)))
		return -EPROTO;
	parent = get_unaligned_le64(v->middle + ((va >> 25) & 2047) * 8);
	if (parent != (v->leaf_pa | 3))
		return -EPROTO;
	pte = get_unaligned_le64(v->leaf + ((va >> 14) & 2047) * 8);
	if (!pte)
		return -ENOENT;
	if ((pte & 3) != 3)
		return -EPROTO;
	*pa = (pte & GPU_M4_PA_MASK) | (va & 0x3fff);
	*attrs = pte & ~GPU_M4_PA_MASK;
	return 0;
}

/* Check a mapping against the caller's intended physical edge and the literal
 * permission contract, independently of the byte encoder's expected shadow.
 */
static inline int gpu_m4_bootstrap_check(const struct gpu_m4_bootstrap_view *v,
				       u64 va, u64 pa, u32 size, u64 attrs,
				       bool guard)
{
	u64 translated, actual_attrs;
	int ret;

	if (!size || (size & 0x3fff) || (va & 0x3fff) || (pa & ~GPU_M4_PA_MASK) ||
	    size - 1 > (GPU_M4_PA_MASK | 0x3fff) - pa)
		return -EINVAL;
	for (u32 offset = 0; offset < size; offset += 0x4000) {
		ret = gpu_m4_bootstrap_walk(v, va + offset, &translated, &actual_attrs);
		if (ret)
			return ret;
		if (translated != pa + offset || actual_attrs != attrs)
			return -EPROTO;
	}
	if (guard) {
		ret = gpu_m4_bootstrap_walk(v, va + size, &translated, &actual_attrs);
		if (ret != -ENOENT)
			return ret ? ret : -EEXIST;
	}
	return 0;
}

static inline int gpu_m4_bootstrap_map(void *table, u64 va, u64 pa, u32 size,
				     enum gpu_m4_access access, bool lower)
{
	u64 base = lower ? GPU_M4_BOOTSTRAP_LOWER : GPU_M4_HOST_START;
	u64 pte;
	u32 first, pages;
	int ret;

	if (!size || (size & 0x3fff) || (va & 0x3fff) ||
	    va < base || va >= base + GPU_M4_BOOTSTRAP_SPAN ||
	    size > base + GPU_M4_BOOTSTRAP_SPAN - va ||
	    pa > GPU_M4_PA_MASK || size - 1 > (GPU_M4_PA_MASK | 0x3fff) - pa)
		return -EINVAL;
	ret = gpu_m4_leaf(pa, access, lower, &pte);
	if (ret)
		return ret;
	first = (va - base) >> 14;
	pages = size >> 14;
	/* Validate the entire extent before modifying even one private entry. */
	for (u32 i = 0; i < pages; i++)
		if (get_unaligned_le64(table + (first + i) * 8))
			return -EEXIST;
	for (u32 i = 0; i < pages; i++)
		put_unaligned_le64(pte + (u64)i * 0x4000, table + (first + i) * 8);
	return 0;
}

#endif
