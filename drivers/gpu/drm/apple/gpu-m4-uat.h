/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_UAT_H
#define APPLE_GPU_M4_UAT_H

#include "gpu-m4-format.h"

static inline bool gpu_m4_host_root_valid(u64 entry, u64 shared_pa, bool running)
{
	if (!shared_pa || (shared_pa & ~GPU_M4_PA_MASK))
		return false;
	/* Qualified firmware rewrites attributes, never the owned child PA. */
	return entry == (shared_pa | 3) ||
		(running && entry == (shared_pa | 0x00200000000004cbULL));
}

/* Immutable private snapshots, never a live-table writer. */
struct gpu_m4_graph {
	const u8 *tables;
	const u8 *shared;
	u64 tables_pa, shared_pa, text_pa, data_pa;
	u32 visited;
	u32 leaves;
	bool host_link;
	u64 bad_page, bad_pte;
	u32 bad_level, bad_index;
	const char *failure;
};

static inline int gpu_m4_graph_fail(struct gpu_m4_graph *graph, u64 page,
				    u32 level, u32 index, u64 pte, const char *reason)
{
	graph->bad_page = page;
	graph->bad_level = level;
	graph->bad_index = index;
	graph->bad_pte = pte;
	graph->failure = reason;
	return -EINVAL;
}

static inline int gpu_m4_firmware_leaf(u64 va, u64 pte, u64 text_pa, u64 data_pa)
{
	u64 off = va - 0xfffffc0000000000ULL;
	u64 expected;

	if (off < 0x60000) {
		expected = text_pa + off;
		expected |= off >= 0x50000 && off < 0x58000 ? 0x483 : 0x4c3;
	} else if (off < 0x19c000) {
		off -= 0x60000;
		expected = data_pa + off;
		expected |= off < 0xc000 ? 0x00200000000004c3ULL : 0x0460000000000443ULL;
	} else {
		return -EINVAL;
	}
	return pte == expected ? 0 : -EINVAL;
}

static inline int gpu_m4_graph_walk(struct gpu_m4_graph *graph, u64 pa,
				    unsigned int level, u64 va)
{
	const u8 *page;
	unsigned int index, entries = level ? 2048 : 64;
	unsigned int shift = level == 0 ? 36 : level == 1 ? 25 : 14;
	u64 pte, child;
	int ret;

	if (level > 2 || pa < graph->tables_pa || pa - graph->tables_pa >= 0x40000 ||
	    (pa & (GPU_M4_PAGE_SIZE - 1)))
		return gpu_m4_graph_fail(graph, pa, level, 0, 0, "table-bounds");
	index = (pa - graph->tables_pa) / GPU_M4_PAGE_SIZE;
	if (graph->visited & (1U << index))
		return gpu_m4_graph_fail(graph, pa, level, 0, 0, "table-cycle");
	graph->visited |= 1U << index;
	page = graph->tables + index * GPU_M4_PAGE_SIZE;
	for (index = 0; index < entries; index++) {
		pte = get_unaligned_le64(page + index * 8);
		if (!pte)
			continue;
		if (!level && index == 2) {
			/* HELLO-only host window has no leaves and no allocator yet. */
			if (pte != (graph->shared_pa | 3))
				return gpu_m4_graph_fail(graph, pa, level, index, pte, "host-link");
			graph->host_link = true;
			continue;
		}
		/* The qualified firmware graph uses only root[0], middle[0]. */
		if (level < 2 && index)
			return gpu_m4_graph_fail(graph, pa, level, index, pte, "foreign-path");
		if (level == 2) {
			ret = gpu_m4_firmware_leaf(va | ((u64)index << shift), pte,
						  graph->text_pa, graph->data_pa);
			if (ret)
				return gpu_m4_graph_fail(graph, pa, level, index, pte, "firmware-leaf");
			graph->leaves++;
			continue;
		}
		child = pte & GPU_M4_PA_MASK;
		if ((pte & ~GPU_M4_PA_MASK) != 0x403)
			return gpu_m4_graph_fail(graph, pa, level, index, pte, "parent-attributes");
		ret = gpu_m4_graph_walk(graph, child, level + 1, va | ((u64)index << shift));
		if (ret)
			return ret;
	}
	return 0;
}

static inline int gpu_m4_graph_check(struct gpu_m4_graph *graph, bool require_host,
				     bool require_firmware)
{
	int ret;

	graph->visited = graph->leaves = 0;
	graph->host_link = false;
	graph->failure = NULL;
	for (unsigned int i = 0; i < GPU_M4_PAGE_SIZE; i++)
		if (graph->shared[i])
			return gpu_m4_graph_fail(graph, graph->shared_pa, 1, i / 8,
				get_unaligned_le64(graph->shared + (i & ~7U)), "host-leaf");
	ret = gpu_m4_graph_walk(graph, graph->tables_pa, 0, 0xfffffc0000000000ULL);
	if (ret)
		return ret;
	if (require_host && !graph->host_link)
		return gpu_m4_graph_fail(graph, graph->tables_pa, 0, 2, 0, "missing-host");
	if (require_firmware && graph->leaves != 0x19c000 / GPU_M4_PAGE_SIZE)
		return -ENODATA;
	return 0;
}

#endif
