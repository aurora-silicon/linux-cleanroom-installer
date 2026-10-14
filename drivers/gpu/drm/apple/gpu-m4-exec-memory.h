/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_EXEC_MEMORY_H
#define APPLE_GPU_M4_EXEC_MEMORY_H

#include <linux/sort.h>
#include "gpu-m4-client-uat.h"
#include "gpu-m4-exec.h"

/* Fixed, independently checked first-compute layout. Firmware aliases come
 * from the guarded READY allocator and are never derived from these GPU VAs.
 */
#define M4_EXEC_WORK_GPU 0x7100100000ULL
#define M4_EXEC_CDM_GPU 0x7100200000ULL
#define M4_EXEC_PREEMPT_GPU 0x7100300000ULL
#define M4_EXEC_AUX_GPU 0x7100400000ULL
#define M4_EXEC_OUTPUT_GPU 0x7100500000ULL
#define M4_EXEC_CONTEXT_STORE_GPU 0x7100600000ULL
#define M4_EXEC_POOL_STATE_GPU 0x7100700000ULL
#define M4_EXEC_PAGE_LIST_GPU 0x7100800000ULL
#define M4_EXEC_POOL_GPU 0x7200000000ULL
/* Match unchanged Mesa's HAL200 window selection and separately retained
 * read-only base-page constructor. Neither is a shader-page alias.
 */
#define M4_EXEC_USC_BASE 0x10000000000ULL
#define M4_EXEC_CONTAINER_GPU 0x10000010000ULL
#define M4_EXEC_IOTO_GPU 0x10000020000ULL
#define M4_EXEC_TABLES 16
#define M4_EXEC_POOL_PAGES 2048

enum m4_exec_buffer_id {
	M4_EXEC_WORK, M4_EXEC_CDM, M4_EXEC_CONTAINER, M4_EXEC_IOTO, M4_EXEC_OUTPUT,
	M4_EXEC_PREEMPT, M4_EXEC_AUX, M4_EXEC_CONTEXT_STORE, M4_EXEC_POOL_STATE,
	M4_EXEC_PAGE_LIST, M4_EXEC_COUNTER, M4_EXEC_USC_WINDOW, M4_EXEC_BUFFERS,
};

enum m4_exec_queue_buffer_id {
	M4_EXEC_QUEUE_INFO, M4_EXEC_QUEUE_STATE, M4_EXEC_QUEUE_RING,
	M4_EXEC_QUEUE_NOTIFIERS, M4_EXEC_QUEUE_NOTIFICATION, M4_EXEC_QUEUE_THRESHOLD,
	M4_EXEC_QUEUE_FWSTAMP, M4_EXEC_QUEUE_USERSTAMP, M4_EXEC_QUEUE_BARRIER,
	M4_EXEC_QUEUE_BUFFERS,
};

static const u32 m4_exec_buffer_size[M4_EXEC_BUFFERS] = {
	0x2000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000,
	0x4000, 0x40000, 0x20000, 0x4000, 8, 0x4000,
};
static const u64 m4_exec_buffer_gpu[M4_EXEC_BUFFERS] = {
	M4_EXEC_WORK_GPU, M4_EXEC_CDM_GPU, M4_EXEC_CONTAINER_GPU, M4_EXEC_IOTO_GPU,
	M4_EXEC_OUTPUT_GPU, M4_EXEC_PREEMPT_GPU, M4_EXEC_AUX_GPU,
	M4_EXEC_CONTEXT_STORE_GPU, M4_EXEC_POOL_STATE_GPU, M4_EXEC_PAGE_LIST_GPU, 0,
	M4_EXEC_USC_BASE,
};
static const u32 m4_exec_queue_size[M4_EXEC_QUEUE_BUFFERS] = {
	0x24c0, 0x60, 0x800, 0x18, 0x200, 8, 8, 8, 0x40,
};

/* queue_cached: the work ring (bit 1), queue-info (bit 2) and notifier
 * list head (bit 4, with the scheduling context) records.
 */
static u32 m4_exec_queue_cached_bit(unsigned int id)
{
	return id == M4_EXEC_QUEUE_RING ? 2 : id == M4_EXEC_QUEUE_INFO ? 4 :
		id == M4_EXEC_QUEUE_NOTIFIERS ? 16 : 0;
}

static enum gpu_m4_access m4_exec_queue_access(const struct m4_gpu_ready *r, unsigned int id)
{
	u32 bit = m4_exec_queue_cached_bit(id);

	return bit ? m4_ready_queue_access(r, bit) : GPU_M4_FW_RW;
}

static u64 m4_exec_queue_attrs(const struct m4_gpu_ready *r, unsigned int id)
{
	u32 bit = m4_exec_queue_cached_bit(id);

	return bit ? m4_ready_queue_attrs(r, bit) : 0x00c000000000044bULL;
}

static bool m4_exec_firmware_buffer(unsigned int id)
{
	return id == M4_EXEC_WORK || id == M4_EXEC_AUX || id == M4_EXEC_COUNTER;
}

static bool m4_exec_writable_buffer(unsigned int id)
{
	return id != M4_EXEC_CDM && id != M4_EXEC_CONTAINER && id != M4_EXEC_IOTO &&
		id != M4_EXEC_USC_WINDOW;
}

struct m4_exec_pool_page {
	u64 pa;
	u32 owner;
};

struct m4_gpu_exec_memory {
	struct m4_gpu_store *store;
	struct m4_ready_buffer context, zero_stamp, notification;
	/* Original allocation .va stays independently retained/checked. A guarded
	 * pre-first-use experiment may select a separate same-backing upper alias.
	 */
	u64 notification_fw;
	struct m4_ready_buffer queue[3][M4_EXEC_QUEUE_BUFFERS];
	struct m4_ready_buffer buffer[M4_EXEC_BUFFERS];
	struct m4_ready_buffer pool[M4_EXEC_POOL_PAGES];
	u64 buffer_pa[M4_EXEC_BUFFERS][16], pool_pa[M4_EXEC_POOL_PAGES], uma_pa[1];
	/* Private immutable exclusion index, never firmware pool order. */
	struct m4_exec_pool_page pool_index[M4_EXEC_POOL_PAGES];
	struct gpu_m4_client_table table[M4_EXEC_TABLES];
	struct page *page[M4_EXEC_TABLES];
	u8 expected[M4_EXEC_TABLES][0x4000];
	struct gpu_m4_client_uat client;
	struct gpu_m4_exec_vm vm;
	u32 allocated, pool_allocated;
	int error;
	bool prepared, cleaned;
	bool barrier_attempted, barrier_complete, published;
	/* Immutable allocation admission: every retained allocation's DMA/PFN/RAM
	 * round trip, the PA ledgers, the sealed client graph and its exact
	 * mappings, including all 2048 pool pages. Proven once over the cleaned
	 * retained tables; only the explicit full diagnostic proves it again.
	 * Never a live table, alias or session result.
	 */
	bool admitted;
};

static int m4_store_prepare(struct m4_gpu_hello *h);

static int m4_exec_pool_page_compare(const void *a, const void *b)
{
	u64 first = ((const struct m4_exec_pool_page *)a)->pa;
	u64 second = ((const struct m4_exec_pool_page *)b)->pa;

	return (first > second) - (first < second);
}

static int m4_exec_pool_index_prepare(struct m4_gpu_exec_memory *e)
{
	if (e->prepared || e->pool_allocated != M4_EXEC_POOL_PAGES)
		return -EINVAL;
	for (u32 i = 0; i < M4_EXEC_POOL_PAGES; i++) {
		if (!e->pool[i].pa || (e->pool[i].pa & 0x3fff) ||
		    e->pool[i].pa >= BIT_ULL(42) || e->pool[i].size != 0x4000 ||
		    e->pool_pa[i] != e->pool[i].pa)
			return -EINVAL;
		e->pool_index[i] = (struct m4_exec_pool_page) { e->pool_pa[i], i };
	}
	sort(e->pool_index, M4_EXEC_POOL_PAGES, sizeof(e->pool_index[0]),
	     m4_exec_pool_page_compare, NULL);
	for (u32 i = 1; i < M4_EXEC_POOL_PAGES; i++)
		if (e->pool_index[i - 1].pa == e->pool_index[i].pa)
			return -EINVAL;
	return 0;
}

/* Prepared pool allocations and this host-only index stay retained and
 * immutable. Exact backing health still checks pool_pa[i] == pool[i].pa.
 * Only the original allocation pointer may exempt an overlapping owner.
 */
static bool m4_exec_pool_overlap(const struct m4_gpu_exec_memory *e,
	u64 pa, u64 size, const struct m4_ready_buffer *allowed)
{
	u32 low = 0, high = M4_EXEC_POOL_PAGES;

	if (!e || !size || pa > U64_MAX - size)
		return true;
	if (!e->prepared) {
		for (u32 i = 0; i < M4_EXEC_POOL_PAGES; i++) {
			const struct m4_ready_buffer *b = &e->pool[i];

			if (b != allowed && b->size &&
			    (b->pa > U64_MAX - b->size ||
			     (pa < b->pa + b->size && b->pa < pa + size)))
				return true;
		}
		return false;
	}
	while (low < high) {
		u32 mid = low + (high - low) / 2;

		if (e->pool_index[mid].pa + 0x4000 <= pa)
			low = mid + 1;
		else
			high = mid;
	}
	for (; low < M4_EXEC_POOL_PAGES && e->pool_index[low].pa < pa + size; low++)
		if (allowed != &e->pool[e->pool_index[low].owner])
			return true;
	return false;
}

static u64 m4_exec_notification_fw(const struct m4_gpu_exec_memory *e)
{
	return e->notification_fw ?: e->notification.va;
}

/* Same qualified direct noncoherent-DMA contract as the READY allocator.
 * CPU accesses use ONLY the returned NC view. Every allocation is retained.
 */
static int m4_exec_nc(struct m4_gpu_hello *h, struct m4_ready_buffer *b,
		       u32 size, bool firmware, enum gpu_m4_access access)
{
	struct device *dev = h->gpu->dev;
	u64 end;
	int ret;

	b->size = ALIGN(size, 0x4000);
	b->cpu = dma_alloc_coherent(dev, b->size, &b->dma, GFP_KERNEL);
	if (!b->cpu)
		return -ENOMEM;
	h->ready->execution->allocated++;
	b->pa = dma_to_phys(dev, b->dma);
	end = b->pa + b->size - 1;
	if (((unsigned long)b->cpu & 0x3fff) || (b->pa & ~GPU_M4_PA_MASK) ||
	    end < b->pa || end >= BIT_ULL(42) ||
	    phys_to_dma(dev, b->pa) != b->dma ||
	    phys_to_dma(dev, end) != b->dma + b->size - 1 ||
	    dma_to_phys(dev, b->dma + b->size - 1) != end ||
	    region_intersects(b->pa, b->size, IORESOURCE_SYSTEM_RAM,
			      IORES_DESC_NONE) != REGION_INTERSECTS)
		return -EINVAL;
	for (u32 offset = 0; offset < b->size; offset += 0x4000)
		if (!pfn_valid(PHYS_PFN(b->pa + offset)) ||
		    phys_to_dma(dev, b->pa + offset) != b->dma + offset ||
		    dma_to_phys(dev, b->dma + offset) != b->pa + offset)
			return -EINVAL;
	memset(b->cpu, 0, b->size);
	if (!firmware)
		return 0;
	ret = m4_ready_take_va(h->ready, b->size, &b->va);
	if (ret)
		return ret;
	return gpu_m4_bootstrap_map(h->ready->expected[M4_UPPER_LEAF], b->va, b->pa,
				    b->size, access, false);
}

static int m4_exec_range(void *cookie, u64 va, u64 size, bool write)
{
	struct m4_gpu_exec_memory *e = cookie;
	u64 end, actual, attrs;

	if (!size || va < GPU_M4_PAGE_SIZE || va >= GPU_M4_CLIENT_VA_END ||
	    size > GPU_M4_CLIENT_VA_END - va)
		return -EINVAL;
	end = va + size;
	for (u64 address = va & ~0x3fffULL; address < end; address += 0x4000) {
		bool found = false;
		int ret = gpu_m4_client_walk(&e->client, address, &actual, &attrs);

		if (ret)
			return ret;
		for (u32 i = 0; i < e->client.mappings; i++) {
			const struct gpu_m4_client_mapping *m = &e->client.mapping[i];

			if (address < m->va || address - m->va >= (u64)m->pages * 0x4000)
				continue;
			if ((write && !m->writable) ||
			    actual != m->pa[(address - m->va) >> 14] ||
			    attrs != gpu_m4_client_attrs(m->writable))
				return -EACCES;
			found = true;
			break;
		}
		if (!found)
			return -EACCES;
	}
	return 0;
}

static int m4_exec_map_buffer(struct m4_gpu_exec_memory *e,
			     enum m4_exec_buffer_id id, u64 gpu, bool writable)
{
	struct m4_ready_buffer *b = &e->buffer[id];

	if (b->size > 0x40000)
		return -E2BIG;
	for (u32 i = 0; i < b->size / 0x4000; i++)
		e->buffer_pa[id][i] = b->pa + (u64)i * 0x4000;
	return gpu_m4_client_map(&e->client, gpu, e->buffer_pa[id], b->size / 0x4000, writable);
}

static int m4_exec_memory_prepare(struct m4_gpu_hello *h)
{
	struct m4_gpu_ready *r = h->ready;
	struct m4_gpu_exec_memory *e;
	int ret;

	if (r->execution || h->gpu->firmware_start_attempted || r->published)
		return -EBUSY;
	e = kvzalloc(sizeof(*e), GFP_KERNEL);
	if (!e)
		return -ENOMEM;
	r->execution = e;
	for (u32 i = 0; i < M4_EXEC_TABLES; i++) {
		e->page[i] = alloc_page(GFP_KERNEL | __GFP_ZERO);
		if (!e->page[i])
			return -ENOMEM;
		e->table[i].cpu = page_address(e->page[i]);
		e->table[i].pa = page_to_phys(e->page[i]);
		if (!pfn_valid(PHYS_PFN(e->table[i].pa)) ||
		    region_intersects(e->table[i].pa, 0x4000, IORESOURCE_SYSTEM_RAM,
				      IORES_DESC_NONE) != REGION_INTERSECTS)
			return -EINVAL;
	}
	ret = gpu_m4_client_init(&e->client, e->table, M4_EXEC_TABLES, 1);
	if (ret)
		return ret;
	e->uma_pa[0] = r->buffer[GPU_M4_UMA].pa;
	ret = gpu_m4_client_map(&e->client, r->input.lower_uma, e->uma_pa, 1, true);
	if (ret)
		return ret;
	ret = m4_exec_nc(h, &e->context, 0x38, true, m4_ready_queue_access(r, 16));
	if (ret)
		return ret;
	ret = m4_exec_nc(h, &e->zero_stamp, 8, true, GPU_M4_FW_RW);
	if (ret)
		return ret;
	/* Client records are not the persistent queues' self-linked lists or
	 * their context-0 barrier notifiers. All three records name ASID 1.
	 */
	ret = m4_exec_nc(h, &e->notification, GPU_M4_EXEC_NOTIFY_SIZE, true, GPU_M4_FW_RW);
	if (ret)
		return ret;
	ret = gpu_m4_exec_notifiers(e->notification.cpu, e->notification.va, 1);
	if (ret)
		return ret;
	gpu_m4_exec_context(e->context.cpu);
	for (u32 engine = 0; engine < 3; engine++) {
		struct m4_ready_buffer *q = e->queue[engine];

		for (u32 i = 0; i < M4_EXEC_QUEUE_BUFFERS; i++) {
			ret = m4_exec_nc(h, &q[i], m4_exec_queue_size[i], true,
				m4_exec_queue_access(r, i));
			if (ret)
				return ret;
		}
		ret = gpu_m4_exec_queue(q[M4_EXEC_QUEUE_STATE].cpu, q[M4_EXEC_QUEUE_INFO].cpu,
			q[M4_EXEC_QUEUE_STATE].va, q[M4_EXEC_QUEUE_RING].va,
			q[M4_EXEC_QUEUE_INFO].va, q[M4_EXEC_QUEUE_NOTIFIERS].va, e->context.va);
		if (ret)
			return ret;
		ret = gpu_m4_exec_notifier_list(q[M4_EXEC_QUEUE_NOTIFIERS].cpu,
					       q[M4_EXEC_QUEUE_NOTIFIERS].va);
		if (ret)
			return ret;
		put_unaligned_le64(q[M4_EXEC_QUEUE_THRESHOLD].va, q[M4_EXEC_QUEUE_NOTIFICATION].cpu);
		put_unaligned_le32(0x50, q[M4_EXEC_QUEUE_NOTIFICATION].cpu + 0x10);
		memset(q[M4_EXEC_QUEUE_NOTIFICATION].cpu + 0xa8, 0xff, 8);
		put_unaligned_le32(1, q[M4_EXEC_QUEUE_NOTIFICATION].cpu + 0x114);
		ret = gpu_m4_exec_dependency(q[M4_EXEC_QUEUE_BARRIER].cpu, e->zero_stamp.va);
		if (ret)
			return ret;
	}
	for (u32 i = 0; i < M4_EXEC_BUFFERS; i++) {
		bool firmware = m4_exec_firmware_buffer(i);

		ret = m4_exec_nc(h, &e->buffer[i], m4_exec_buffer_size[i], firmware,
			i == M4_EXEC_WORK || i == M4_EXEC_AUX ? GPU_M4_SHARED_RW : GPU_M4_FW_RW);
		if (ret)
			return ret;
		if (m4_exec_buffer_gpu[i]) {
			ret = m4_exec_map_buffer(e, i, m4_exec_buffer_gpu[i],
				m4_exec_writable_buffer(i));
			if (ret)
				return ret;
		}
	}
	ret = gpu_m4_bootstrap_map(r->expected[M4_LOWER_LEAF], M4_EXEC_WORK_GPU,
				  e->buffer[M4_EXEC_WORK].pa, 0x4000, GPU_M4_GPU_RW, true);
	if (ret)
		return ret;
	for (u32 i = 0; i < M4_EXEC_POOL_PAGES; i++) {
		ret = m4_exec_nc(h, &e->pool[i], 0x4000, false, GPU_M4_FW_RW);
		if (ret)
			return ret;
		e->pool_pa[i] = e->pool[i].pa;
		e->pool_allocated++;
	}
	ret = m4_exec_pool_index_prepare(e);
	if (ret)
		return ret;
	ret = gpu_m4_client_map(&e->client, M4_EXEC_POOL_GPU, e->pool_pa, M4_EXEC_POOL_PAGES, true);
	if (ret)
		return ret;
	ret = gpu_m4_client_seal(&e->client);
	if (ret)
		return ret;
	e->vm = (struct gpu_m4_exec_vm) {
		.start = GPU_M4_EXEC_VM_START, .end = GPU_M4_EXEC_VM_END,
		.cookie = e, .check = m4_exec_range,
	};
	ret = gpu_m4_exec_uma_pool(e->buffer[M4_EXEC_POOL_STATE].cpu,
				  e->buffer[M4_EXEC_PAGE_LIST].cpu, &e->vm, M4_EXEC_POOL_GPU);
	if (ret)
		return ret;
	put_unaligned_le32(0xdeadbeef, e->buffer[M4_EXEC_OUTPUT].cpu);
	put_unaligned_le64(1, e->buffer[M4_EXEC_COUNTER].cpu);
	for (u32 i = 0; i < M4_EXEC_TABLES; i++)
		memcpy(e->expected[i], e->table[i].cpu, 0x4000);
	e->prepared = true;
	if (compute_enable) {
		ret = m4_store_prepare(h);
		if (ret)
			return ret;
	}
	dev_info(h->gpu->dev, "GPU detached execution backing: %u NC allocations, pool32MiB, %u client tables; no binding or GPU work\n",
		 e->allocated, e->client.used);
	return 0;
}

static void m4_exec_memory_clean(struct m4_gpu_hello *h)
{
	struct m4_gpu_exec_memory *e = h->ready->execution;

	/* Unused private WB pages were zeroed too: preserve those zeros before
	 * the subsequent independent validator invalidates every retained page.
	 */
	for (u32 i = e->client.used; i < M4_EXEC_TABLES; i++)
		dcache_clean_poc((unsigned long)e->table[i].cpu,
				 (unsigned long)e->table[i].cpu + 0x4000);
	dma_wmb();
	for (int level = 2; level >= 0; level--)
		for (u32 i = 0; i < e->client.used; i++) {
			void *p = e->table[i].cpu;

			if (e->table[i].level != level)
				continue;
			dcache_clean_poc((unsigned long)p, (unsigned long)p + 0x4000);
			dma_wmb();
		}
	e->cleaned = true;
}

/* Immutable ready-buffer admission memo (host fixtures inject RAM/DMA
 * failures behind unchanged fields: off there).
 */
#ifdef MODULE
static bool exec_backing_memo = true;
module_param(exec_backing_memo, bool, 0644);
MODULE_PARM_DESC(exec_backing_memo, "Reuse a ready buffer's RAM/DMA admission while its fields are unchanged (default 1)");
#else
static bool exec_backing_memo;
#endif
/* The exact tuple last proven RAM-backed and DMA-consistent per buffer
 * (immutable ready allocations; any field change misses). Callers hold the
 * GPU mutex.
 */
#define M4_EXEC_BACKING_MEMO_BITS 7
struct m4_exec_backing_memo {
	const struct m4_ready_buffer *b;
	void *cpu;
	u64 dma, pa;
	u32 size, checked;
};
static struct m4_exec_backing_memo m4_exec_backing_memos[1 << M4_EXEC_BACKING_MEMO_BITS];

static bool m4_exec_backing_valid(struct m4_gpu_hello *h,
				  const struct m4_ready_buffer *b,
				  u32 size, bool firmware)
{
	struct device *dev = h->gpu->dev;
	struct m4_exec_backing_memo *memo =
		&m4_exec_backing_memos[((unsigned long)b >> 5) & ((1 << M4_EXEC_BACKING_MEMO_BITS) - 1)];
	u32 checked = size + 1 + (firmware ? 0x80000000U : 0);
	u64 end;

	if (exec_backing_memo && memo->b == b && memo->checked == checked &&
	    memo->cpu == b->cpu && memo->dma == b->dma && memo->pa == b->pa &&
	    memo->size == b->size && !!b->va == firmware)
		return true;
	if (!b->cpu || ((unsigned long)b->cpu & 0x3fff) ||
	    b->size != ALIGN(size, 0x4000) || !b->pa ||
	    (b->pa & ~GPU_M4_PA_MASK) || !!b->va != firmware)
		return false;
	end = b->pa + b->size - 1;
	if (end < b->pa || end >= BIT_ULL(42) ||
	    dma_to_phys(dev, b->dma) != b->pa ||
	    phys_to_dma(dev, b->pa) != b->dma ||
	    phys_to_dma(dev, end) != b->dma + b->size - 1 ||
	    dma_to_phys(dev, b->dma + b->size - 1) != end ||
	    region_intersects(b->pa, b->size, IORESOURCE_SYSTEM_RAM,
			      IORES_DESC_NONE) != REGION_INTERSECTS)
		return false;
	for (u32 offset = 0; offset < b->size; offset += 0x4000)
		if (!pfn_valid(PHYS_PFN(b->pa + offset)) ||
		    phys_to_dma(dev, b->pa + offset) != b->dma + offset ||
		    dma_to_phys(dev, b->dma + offset) != b->pa + offset)
			return false;
	if (exec_backing_memo)
		*memo = (struct m4_exec_backing_memo) {
			.b = b, .cpu = b->cpu, .dma = b->dma, .pa = b->pa, .size = b->size,
			.checked = checked,
		};
	return true;
}

/* Immutable allocation admission. The caller has just compared every live
 * client table byte with its private expected image, so the sealed graph and
 * mapping walks below are a function of retained, never-rewritten state:
 * allocation records, DMA/PFN/RAM facts and the PA ledgers. Ordinary packets
 * mutate none of them. Runs once, then only for full diagnostics.
 */
static bool m4_exec_memory_admit(struct m4_gpu_hello *h)
{
	struct m4_gpu_exec_memory *e = h->ready->execution;
	struct m4_ready_buffer *common[3] = { &e->context, &e->zero_stamp, &e->notification };
	static const u32 common_size[] = { 0x38, 8, GPU_M4_EXEC_NOTIFY_SIZE };

	if (gpu_m4_client_seal(&e->client))
		return false;
	for (u32 i = 0; i < ARRAY_SIZE(common); i++)
		if (!m4_exec_backing_valid(h, common[i], common_size[i], true))
			return false;
	for (u32 engine = 0; engine < 3; engine++)
		for (u32 i = 0; i < M4_EXEC_QUEUE_BUFFERS; i++)
			if (!m4_exec_backing_valid(h, &e->queue[engine][i],
						   m4_exec_queue_size[i], true))
				return false;
	for (u32 i = 0; i < M4_EXEC_BUFFERS; i++) {
		struct m4_ready_buffer *b = &e->buffer[i];

		if (!m4_exec_backing_valid(h, b, m4_exec_buffer_size[i], m4_exec_firmware_buffer(i)))
			return false;
		if (!m4_exec_buffer_gpu[i])
			continue;
		for (u32 j = 0; j < b->size / 0x4000; j++)
			if (e->buffer_pa[i][j] != b->pa + (u64)j * 0x4000)
				return false;
		if (gpu_m4_client_check(&e->client, m4_exec_buffer_gpu[i],
					e->buffer_pa[i], b->size / 0x4000,
					m4_exec_writable_buffer(i)))
			return false;
	}
	for (u32 i = 0; i < M4_EXEC_POOL_PAGES; i++)
		if (!m4_exec_backing_valid(h, &e->pool[i], 0x4000, false) ||
		    e->pool_pa[i] != e->pool[i].pa)
			return false;
	return !gpu_m4_client_check(&e->client, M4_EXEC_WORK_GPU,
				    e->buffer_pa[M4_EXEC_WORK], 1, true) &&
		!gpu_m4_client_check(&e->client, h->ready->input.lower_uma, e->uma_pa, 1, true) &&
		!gpu_m4_client_check(&e->client, M4_EXEC_POOL_GPU, e->pool_pa,
				     M4_EXEC_POOL_PAGES, true);
}

/* full=false is the per-publication/poll check: retained structure, every
 * live client table byte, the selected upper notification alias and every
 * READY bootstrap/lower edge, over the once-admitted allocation inventory.
 * full=true (fault/diagnostic boundary) also re-proves that inventory, and a
 * failed re-proof withdraws it.
 */
static bool m4_exec_memory_check(struct m4_gpu_hello *h, bool full)
{
	struct m4_gpu_exec_memory *e = h->ready->execution;
	struct gpu_m4_bootstrap_view upper = {
		.root = h->tables, .middle = h->shared,
		.leaf = page_address(h->ready->table[M4_UPPER_LEAF]),
		.middle_pa = h->region[M4_SHARED].start,
		.leaf_pa = page_to_phys(h->ready->table[M4_UPPER_LEAF]),
		.running = h->gpu->firmware_start_attempted,
	};
	struct gpu_m4_bootstrap_view lower = {
		.root = page_address(h->lower),
		.middle = page_address(h->ready->table[M4_LOWER_MIDDLE]),
		.leaf = page_address(h->ready->table[M4_LOWER_LEAF]),
		.middle_pa = page_to_phys(h->ready->table[M4_LOWER_MIDDLE]),
		.leaf_pa = page_to_phys(h->ready->table[M4_LOWER_LEAF]),
		.lower = true,
	};
	struct m4_ready_buffer *common[3];

	if (!e || !e->prepared || !e->cleaned || e->pool_allocated != M4_EXEC_POOL_PAGES ||
	    e->allocated != 3 + 3 * M4_EXEC_QUEUE_BUFFERS + M4_EXEC_BUFFERS + M4_EXEC_POOL_PAGES ||
	    e->client.table != e->table || e->client.count != M4_EXEC_TABLES ||
	    e->client.asid != 1 || !e->client.sealed || e->client.mappings != M4_EXEC_BUFFERS + 1)
		return false;
	common[0] = &e->context;
	common[1] = &e->zero_stamp;
	common[2] = &e->notification;
	for (u32 i = 0; i < M4_EXEC_TABLES; i++) {
		if (!e->page[i] || e->table[i].cpu != page_address(e->page[i]) ||
		    e->table[i].pa != page_to_phys(e->page[i]))
			return false;
		dcache_inval_poc((unsigned long)e->table[i].cpu,
				 (unsigned long)e->table[i].cpu + 0x4000);
		if (memcmp(e->table[i].cpu, e->expected[i], 0x4000))
			return false;
	}
	if (full || !e->admitted) {
		e->admitted = false;
		if (!m4_exec_memory_admit(h))
			return false;
		e->admitted = true;
	}
	if (e->notification_fw) {
		u64 pa, attrs;

		if (!h->ready->upper ||
		    gpu_m4_drm_upper_walk_locked(h->ready->upper, e->notification_fw,
			&pa, &attrs) || pa != e->notification.pa ||
		    attrs != GPU_M4_DRM_UPPER_FW_ATTRS ||
		    gpu_m4_drm_upper_walk_locked(h->ready->upper,
			e->notification_fw + e->notification.size, &pa, &attrs) != -ENOENT)
			return false;
	}
	/* Live READY bootstrap edges of each retained firmware allocation. */
	for (u32 i = 0; i < ARRAY_SIZE(common); i++)
		if (gpu_m4_bootstrap_check(&upper, common[i]->va, common[i]->pa,
					   common[i]->size, common[i] == &e->context ?
					   m4_ready_queue_attrs(h->ready, 16) : 0x00c000000000044bULL, true))
			return false;
	for (u32 engine = 0; engine < 3; engine++)
		for (u32 i = 0; i < M4_EXEC_QUEUE_BUFFERS; i++) {
			struct m4_ready_buffer *b = &e->queue[engine][i];

			if (gpu_m4_bootstrap_check(&upper, b->va, b->pa, b->size,
						   m4_exec_queue_attrs(h->ready, i), true))
				return false;
		}
	for (u32 i = 0; i < M4_EXEC_BUFFERS; i++) {
		struct m4_ready_buffer *b = &e->buffer[i];
		u64 attrs = i == M4_EXEC_WORK || i == M4_EXEC_AUX ?
			0x00e000000000040bULL : 0x00c000000000044bULL;

		if (!!b->va != m4_exec_firmware_buffer(i) ||
		    (b->va && gpu_m4_bootstrap_check(&upper, b->va, b->pa, b->size, attrs, true)))
			return false;
	}
	if (e->uma_pa[0] != h->ready->buffer[GPU_M4_UMA].pa)
		return false;
	/* Compare both command aliases independently with its allocation PA:
	 * the client alias is part of the admitted sealed graph, the lower
	 * bootstrap alias is a live READY edge.
	 */
	return !gpu_m4_bootstrap_check(&lower, M4_EXEC_WORK_GPU, e->buffer[M4_EXEC_WORK].pa,
				       0x4000, 0x00c0000000000c8bULL, true);
}

static bool m4_exec_memory_valid(struct m4_gpu_hello *h)
{
	return m4_exec_memory_check(h, false);
}

#include "gpu-m4-store-memory.h"

#endif
