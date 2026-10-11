/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_CLIENT_UAT_H
#define APPLE_GPU_M4_CLIENT_UAT_H

#include "gpu-m4-format.h"

/* Detached first-client graph only. The caller proves that every table/data
 * page is owned usable RAM, and retains all pages plus each immutable PA array.
 * No MMIO, cache maintenance, publication, binding or unmap occurs here.
 */
#define GPU_M4_CLIENT_TABLE_LIMIT 128
#define GPU_M4_CLIENT_MAPPING_LIMIT 64
#define GPU_M4_CLIENT_PAGE_LIMIT 2048 /* one 32-MiB first-compute pool */
#define GPU_M4_CLIENT_VA_END ((1ULL << 42) - 0x8000)

struct gpu_m4_client_table {
	void *cpu;
	u64 pa, base;
	u8 level; /* 0=root, 1=middle, 2=leaf, 0xff=unused */
};

struct gpu_m4_client_mapping {
	const u64 *pa;
	u64 va;
	u32 pages;
	bool writable;
};

struct gpu_m4_client_uat {
	struct gpu_m4_client_table *table;
	u32 count, used, mappings;
	u8 asid;
	bool sealed;
	struct gpu_m4_client_mapping mapping[GPU_M4_CLIENT_MAPPING_LIMIT];
};

static inline u64 gpu_m4_client_attrs(bool writable)
{
	return writable ? 0x00c0000000000c8bULL : 0x0080000000000c8bULL;
}

static inline bool gpu_m4_client_span(u64 va, u32 pages)
{
	u64 bytes = ((u64)pages + 1) * GPU_M4_PAGE_SIZE; /* mandatory trailing guard */

	return pages && pages <= GPU_M4_CLIENT_PAGE_LIMIT && va >= GPU_M4_PAGE_SIZE &&
		!(va & 0x3fff) && va < GPU_M4_CLIENT_VA_END &&
		bytes <= GPU_M4_CLIENT_VA_END - va;
}

static inline bool gpu_m4_client_shape(const struct gpu_m4_client_uat *g)
{
	return g && g->table && g->count && g->count <= GPU_M4_CLIENT_TABLE_LIMIT &&
		g->used && g->used <= g->count && g->mappings <= GPU_M4_CLIENT_MAPPING_LIMIT &&
		g->asid <= 1 && g->table[0].level == 0 && !g->table[0].base;
}

static inline int gpu_m4_client_init(struct gpu_m4_client_uat *g,
				   struct gpu_m4_client_table *table, u32 count, u8 asid)
{
	if (!g || !table || !count || count > GPU_M4_CLIENT_TABLE_LIMIT || asid > 1)
		return -EINVAL;
	for (u32 i = 0; i < count; i++) {
		if (!table[i].cpu || ((unsigned long)table[i].cpu & 0x3fff) ||
		    !table[i].pa || (table[i].pa & ~GPU_M4_PA_MASK))
			return -EINVAL;
		for (u32 j = 0; j < i; j++)
			if (table[i].cpu == table[j].cpu || table[i].pa == table[j].pa)
				return -EINVAL;
		for (u32 j = 0; j < 2048; j++)
			if (get_unaligned_le64(table[i].cpu + j * 8))
				return -EBUSY;
	}
	memset(g, 0, sizeof(*g));
	g->table = table;
	g->count = count;
	g->used = 1;
	g->asid = asid;
	for (u32 i = 0; i < count; i++) {
		table[i].level = i ? 0xff : 0;
		table[i].base = 0;
	}
	return 0;
}

/* Resolve only an exact, owned child descriptor at its assigned VA prefix. */
static inline int gpu_m4_client_child(const struct gpu_m4_client_uat *g,
				    u64 descriptor, u8 level, u64 base)
{
	if (!descriptor)
		return -ENOENT;
	if ((descriptor & ~GPU_M4_PA_MASK) != 3)
		return -EPROTO;
	for (u32 i = 1; i < g->used; i++)
		if (g->table[i].pa == (descriptor & GPU_M4_PA_MASK))
			return g->table[i].level == level && g->table[i].base == base ?
				(int)i : -EPROTO;
	return -EPROTO;
}

static inline int gpu_m4_client_leaf_table(const struct gpu_m4_client_uat *g, u64 va)
{
	int middle, leaf;
	u64 parent;

	parent = get_unaligned_le64(g->table[0].cpu + ((va >> 36) & 63) * 8);
	middle = gpu_m4_client_child(g, parent, 1, va & ~((1ULL << 36) - 1));
	if (middle < 0)
		return middle;
	parent = get_unaligned_le64(g->table[middle].cpu + ((va >> 25) & 2047) * 8);
	leaf = gpu_m4_client_child(g, parent, 2, va & ~((1ULL << 25) - 1));
	return leaf;
}

static inline int gpu_m4_client_walk(const struct gpu_m4_client_uat *g,
				   u64 va, u64 *pa, u64 *attrs)
{
	u64 pte;
	int leaf;

	if (!gpu_m4_client_shape(g) || !pa || !attrs || va < GPU_M4_PAGE_SIZE ||
	    va >= GPU_M4_CLIENT_VA_END)
		return -EINVAL;
	leaf = gpu_m4_client_leaf_table(g, va);
	if (leaf < 0)
		return leaf;
	pte = get_unaligned_le64(g->table[leaf].cpu + ((va >> 14) & 2047) * 8);
	if (!pte)
		return -ENOENT;
	*attrs = pte & ~GPU_M4_PA_MASK;
	if (*attrs != gpu_m4_client_attrs(false) && *attrs != gpu_m4_client_attrs(true))
		return -EPROTO;
	*pa = (pte & GPU_M4_PA_MASK) | (va & 0x3fff);
	return 0;
}

/* A dry run checks every page, guard, parent and required table before writes.
 * All possible failures therefore leave the private table bytes unchanged.
 */
static inline int gpu_m4_client_map(struct gpu_m4_client_uat *g, u64 va,
				  const u64 *pa, u32 pages, bool writable)
{
	u64 last_middle = ~0ULL, last_leaf = ~0ULL, bytes;
	u32 needed = 0;

	if (!gpu_m4_client_shape(g) || g->sealed || !pa || !gpu_m4_client_span(va, pages))
		return -EINVAL;
	if (g->mappings == GPU_M4_CLIENT_MAPPING_LIMIT)
		return -ENOSPC;
	bytes = ((u64)pages + 1) * GPU_M4_PAGE_SIZE;
	for (u32 i = 0; i < g->mappings; i++) {
		const struct gpu_m4_client_mapping *m = &g->mapping[i];
		u64 prior = ((u64)m->pages + 1) * GPU_M4_PAGE_SIZE;

		if (!m->pa || !gpu_m4_client_span(m->va, m->pages))
			return -EPROTO;
		if (va < m->va + prior && m->va < va + bytes)
			return -EEXIST;
	}
	for (u32 i = 0; i < pages; i++) {
		if (!pa[i] || (pa[i] & ~GPU_M4_PA_MASK))
			return -EINVAL;
		/* The first owner has no duplicate/repeated data pages in one root. */
		for (u32 j = 0; j < i; j++)
			if (pa[i] == pa[j])
				return -EEXIST;
		for (u32 j = 0; j < g->count; j++)
			if (pa[i] == g->table[j].pa)
				return -EEXIST;
		for (u32 j = 0; j < g->mappings; j++)
			for (u32 k = 0; k < g->mapping[j].pages; k++)
				if (pa[i] == g->mapping[j].pa[k])
					return -EEXIST;
	}
	for (u32 i = 0; i <= pages; i++) {
		u64 address = va + (u64)i * GPU_M4_PAGE_SIZE;
		u64 middle_base = address & ~((1ULL << 36) - 1);
		u64 leaf_base = address & ~((1ULL << 25) - 1);
		u64 parent = get_unaligned_le64(g->table[0].cpu + (address >> 36) * 8);
		int middle = gpu_m4_client_child(g, parent, 1, middle_base), leaf = -ENOENT;

		if (middle < 0 && middle != -ENOENT)
			return middle;
		if (middle == -ENOENT && i < pages && middle_base != last_middle)
			needed++;
		if (middle >= 0) {
			parent = get_unaligned_le64(g->table[middle].cpu + ((address >> 25) & 2047) * 8);
			leaf = gpu_m4_client_child(g, parent, 2, leaf_base);
			if (leaf < 0 && leaf != -ENOENT)
				return leaf;
		}
		if (leaf == -ENOENT && i < pages && leaf_base != last_leaf)
			needed++;
		if (leaf >= 0 && get_unaligned_le64(g->table[leaf].cpu + ((address >> 14) & 2047) * 8))
			return -EEXIST;
		last_middle = middle_base;
		last_leaf = leaf_base;
	}
	if (needed > g->count - g->used)
		return -ENOSPC;
	for (u32 i = g->used; i < g->used + needed; i++) {
		if (g->table[i].level != 0xff)
			return -EPROTO;
		for (u32 j = 0; j < 2048; j++)
			if (get_unaligned_le64(g->table[i].cpu + j * 8))
				return -EPROTO;
	}
	for (u32 i = 0; i < pages; i++) {
		u64 address = va + (u64)i * GPU_M4_PAGE_SIZE;
		void *parent = g->table[0].cpu + (address >> 36) * 8;
		int middle = gpu_m4_client_child(g, get_unaligned_le64(parent), 1,
					       address & ~((1ULL << 36) - 1));
		int leaf;

		if (middle == -ENOENT) {
			middle = g->used++;
			g->table[middle].level = 1;
			g->table[middle].base = address & ~((1ULL << 36) - 1);
			put_unaligned_le64(g->table[middle].pa | 3, parent);
		}
		parent = g->table[middle].cpu + ((address >> 25) & 2047) * 8;
		leaf = gpu_m4_client_child(g, get_unaligned_le64(parent), 2,
					   address & ~((1ULL << 25) - 1));
		if (leaf == -ENOENT) {
			leaf = g->used++;
			g->table[leaf].level = 2;
			g->table[leaf].base = address & ~((1ULL << 25) - 1);
			put_unaligned_le64(g->table[leaf].pa | 3, parent);
		}
		put_unaligned_le64(pa[i] | gpu_m4_client_attrs(writable),
				   g->table[leaf].cpu + ((address >> 14) & 2047) * 8);
	}
	g->mapping[g->mappings++] = (struct gpu_m4_client_mapping) {
		.pa = pa, .va = va, .pages = pages, .writable = writable,
	};
	return 0;
}

static inline int gpu_m4_client_check(const struct gpu_m4_client_uat *g,
				    u64 va, const u64 *pa, u32 pages, bool writable)
{
	u64 actual, attrs;
	int ret;

	if (!pa || !gpu_m4_client_span(va, pages))
		return -EINVAL;
	for (u32 i = 0; i < pages; i++) {
		ret = gpu_m4_client_walk(g, va + (u64)i * GPU_M4_PAGE_SIZE, &actual, &attrs);
		if (ret)
			return ret;
		if (actual != pa[i] || attrs != gpu_m4_client_attrs(writable))
			return -EPROTO;
	}
	ret = gpu_m4_client_walk(g, va + (u64)pages * GPU_M4_PAGE_SIZE, &actual, &attrs);
	return ret == -ENOENT ? 0 : ret ? ret : -EEXIST;
}

static inline int gpu_m4_client_verify_page(const struct gpu_m4_client_uat *g,
					   u32 id, bool visited[GPU_M4_CLIENT_TABLE_LIMIT])
{
	const struct gpu_m4_client_table *table = &g->table[id];
	u32 count = table->level ? 2048 : 64;
	u32 shift = table->level == 0 ? 36 : table->level == 1 ? 25 : 14;

	if (visited[id] || table->level > 2)
		return -EPROTO;
	visited[id] = true;
	for (u32 i = 0; i < count; i++) {
		u64 pte = get_unaligned_le64(table->cpu + i * 8);
		u64 va = table->base | ((u64)i << shift);
		int ret;

		if (!pte)
			continue;
		if (table->level < 2) {
			ret = gpu_m4_client_child(g, pte, table->level + 1, va);
			if (ret < 0)
				return ret;
			ret = gpu_m4_client_verify_page(g, ret, visited);
			if (ret)
				return ret;
		} else {
			bool found = false;

			for (u32 j = 0; j < g->mappings; j++) {
				const struct gpu_m4_client_mapping *m = &g->mapping[j];

				if (va < m->va || va - m->va >= (u64)m->pages * GPU_M4_PAGE_SIZE)
					continue;
				if (pte != (m->pa[(va - m->va) >> 14] | gpu_m4_client_attrs(m->writable)))
					return -EPROTO;
				found = true;
				break;
			}
			if (!found)
				return -EPROTO;
		}
	}
	if (!table->level)
		for (u32 i = 64; i < 2048; i++)
			if (get_unaligned_le64(table->cpu + i * 8))
				return -EPROTO;
	return 0;
}

/* Full graph/ledger check rejects unexpected aliases, unreachable used pages,
 * hidden extra leaves and missing intended pages, then freezes the owner.
 */
static inline int gpu_m4_client_seal(struct gpu_m4_client_uat *g)
{
	bool visited[GPU_M4_CLIENT_TABLE_LIMIT] = { false };
	int ret;

	if (!gpu_m4_client_shape(g) || !g->mappings)
		return -EINVAL;
	for (u32 i = 0; i < g->mappings; i++) {
		const struct gpu_m4_client_mapping *m = &g->mapping[i];

		ret = gpu_m4_client_check(g, m->va, m->pa, m->pages, m->writable);
		if (ret)
			return ret;
	}
	ret = gpu_m4_client_verify_page(g, 0, visited);
	if (ret)
		return ret;
	for (u32 i = 0; i < g->used; i++)
		if (!visited[i])
			return -EPROTO;
	for (u32 i = g->used; i < g->count; i++)
		for (u32 j = 0; j < 2048; j++)
			if (get_unaligned_le64(g->table[i].cpu + j * 8))
				return -EPROTO;
	g->sealed = true;
	return 0;
}

#endif
