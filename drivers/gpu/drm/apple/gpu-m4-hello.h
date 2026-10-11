/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_HELLO_H
#define APPLE_GPU_M4_HELLO_H

#include "gpu-m4-uat.h"
#include "gpu-m4-drm-provider-lane.h"

static bool hello_enable;
module_param(hello_enable, bool, 0400);
MODULE_PARM_DESC(hello_enable, "Permit watchdog-guarded J713 RTKit HELLO only");

static bool firmware_logging;
module_param(firmware_logging, bool, 0400);
MODULE_PARM_DESC(firmware_logging, "Retain late firmware messages in the guarded bring-up log");

enum m4_hello_region { M4_TTBS, M4_TABLES, M4_HANDOFF, M4_SHARED, M4_TEXT, M4_DATA };

struct m4_gpu_hello {
	struct m4_gpu_power *gpu;
	struct resource region[6];
	struct apple_rtkit *rtkit;
	struct m4_gpu_ready *ready;
	void *tables, *shared;
	void __iomem *handoff;
	struct page *lower;
	atomic_t callback_error;
	atomic_t crash_claimed;
	struct completion crash_ready;
	u64 crash_pa;
	u32 crash_size;
	int error, graph_diagnostic;
	u32 firmware_leaves;
	bool published, complete;
	bool host_qualified;
	/* Ordinary lane woken by EP 0x20 notifications. Release-published once,
	 * retained with the pinned READY/session owner for every receive.
	 */
	struct gpu_m4_drm_provider_lane *notify_lane;
	/* REMOVE-DEBUG fault attribution: the ordinary runtime's context/VM
	 * table, logged under the owner mutex at a recovery/unexpected record.
	 * Read-only; set once with the lane.
	 */
	void (*fault_report)(void *owner);
	void *fault_report_owner;
};

#include "gpu-m4-ready-memory.h"

static void m4_ready_notify(struct m4_gpu_hello *hello);

static void m4_hello_graph_log(struct m4_gpu_hello *hello, const u8 *snapshot,
			      const struct gpu_m4_graph *graph, bool equal)
{
	struct device *dev = hello->gpu->dev;
	u64 pte;
	unsigned int extra = 0, shared = 0;

	dev_info(dev, "GPU graph snapshot_equal=%u host_qualified=%u failure=%s level=%u page=%#llx index=%u raw=%#llx\n",
		 equal, hello->host_qualified, graph->failure ?: "none", graph->bad_level,
		 graph->bad_page, graph->bad_index, graph->bad_pte);
	for (unsigned int i = 0; i < 3; i++) {
		pte = get_unaligned_le64(snapshot + i * 8);
		dev_info(dev, "GPU graph root[%u]=%#llx pa=%#llx attrs=%#llx\n",
			 i, pte, pte & GPU_M4_PA_MASK, pte & ~GPU_M4_PA_MASK);
	}
	for (unsigned int page = 0; page < 17; page++) {
		unsigned int count = 0;

		for (unsigned int index = 0; index < 2048; index++) {
			pte = get_unaligned_le64(snapshot + page * 0x4000 + index * 8);
			if (!pte)
				continue;
			count++;
			if (page == 16 && shared++ < 64)
				dev_info(dev, "GPU graph shared[%u]=%#llx\n", index, pte);
			if (page != 16 && !(page == 2 && index < 103) && extra++ < 16)
				dev_info(dev, "GPU graph extra page=%u index=%u raw=%#llx pa=%#llx attrs=%#llx\n",
					 page, index, pte, pte & GPU_M4_PA_MASK, pte & ~GPU_M4_PA_MASK);
		}
		dev_info(dev, "GPU graph page=%u nonzero=%u\n", page, count);
	}
	for (unsigned int i = 2030; i < 2048; i++)
		dev_info(dev, "GPU graph captured-profile page2[%u]=%#llx\n", i,
			 get_unaligned_le64(snapshot + 0x8000 + i * 8));
	if (shared > 64)
		dev_info(dev, "GPU graph shared log truncated: %u total nonzero entries\n", shared);
}

static int m4_hello_regions(struct m4_gpu_hello *hello)
{
	static const char * const names[] = {
		"ttbs", "pagetables", "handoff", "shared-l2", "fw-text", "fw-data",
	};
	static const u32 sizes[] = { 0x4000, 0x40000, 0x4000, 0x4000, 0x60000, 0x13c000 };
	struct device_node *gpu = hello->gpu->dev->of_node, *np;
	struct reserved_mem *rmem;
	const char *name;
	u64 vas[2];
	u32 flags[2], version;
	int i, j, len, ret;

	if (of_property_count_strings(gpu, "memory-region-names") != 6 ||
	    of_property_count_u32_elems(gpu, "memory-region") != 6 ||
	    of_property_count_u32_elems(gpu, "apple,g16-handoff-version") != 1 ||
	    of_property_read_u32(gpu, "apple,g16-handoff-version", &version) || version != 1 ||
	    of_property_count_u64_elems(gpu, "apple,firmware-segment-vas") != 2 ||
	    of_property_read_u64_array(gpu, "apple,firmware-segment-vas", vas, 2) ||
	    vas[0] != 0xfffffc0000000000ULL || vas[1] != 0xfffffc0000060000ULL ||
	    of_property_count_u32_elems(gpu, "apple,firmware-segment-flags") != 2 ||
	    of_property_read_u32_array(gpu, "apple,firmware-segment-flags", flags, 2) ||
	    flags[0] != 1 || flags[1] != 0)
		return -EINVAL;
	for (i = 0; i < 6; i++) {
		struct resource *r = &hello->region[i];

		if (of_property_read_string_index(gpu, "memory-region-names", i, &name) ||
		    strcmp(name, names[i]))
			return -EINVAL;
		np = of_parse_phandle(gpu, "memory-region", i);
		if (!np)
			return -EINVAL;
		rmem = of_reserved_mem_lookup(np);
		ret = !of_find_property(np, "no-map", &len) || len ||
			of_property_read_bool(np, "reusable") || !of_device_is_available(np) ?
			-EINVAL : of_address_to_resource(np, 0, r);
		of_node_put(np);
		if (ret || !rmem || resource_size(r) != sizes[i] ||
		    rmem->base != r->start || rmem->size != sizes[i] ||
		    !r->start || (r->start & 0x3fff) || r->end >= BIT_ULL(42) ||
		    region_intersects(r->start, sizes[i], IORESOURCE_SYSTEM_RAM,
				      IORES_DESC_NONE) != REGION_DISJOINT)
			return -EINVAL;
		for (j = 0; j < i; j++)
			if (r->start <= hello->region[j].end && hello->region[j].start <= r->end)
				return -EINVAL;
	}
	return 0;
}

static int m4_hello_hash(struct m4_gpu_hello *hello)
{
	static const u8 expected[32] = {
		0x5f, 0x65, 0xf6, 0x14, 0x58, 0x72, 0x54, 0x70,
		0xb4, 0xb0, 0xc0, 0x20, 0x28, 0x47, 0x7b, 0xf2,
		0xea, 0x57, 0x3a, 0xfb, 0x4e, 0x0c, 0x49, 0x6e,
		0xf9, 0x95, 0xe9, 0x6a, 0xff, 0xb0, 0xd8, 0x6c,
	};
	u8 digest[32], *a, *b;
	void __iomem *text;
	int ret = -ENOMEM;

	a = kvmalloc(0xc0000, GFP_KERNEL);
	if (!a)
		return ret;
	b = a + 0x60000;
	text = ioremap_wc(hello->region[M4_TEXT].start, 0x60000);
	if (!text)
		goto out;
	memcpy_fromio(a, text, 0x60000);
	memcpy_fromio(b, text, 0x60000);
	iounmap(text);
	ret = -EINVAL;
	if (memcmp(a, b, 0x60000) || memcmp(a + 0x5c8df, "GKTS", 4) ||
	    get_unaligned_le32(a + 0x5c8e3) != 8 || memcmp(a + 0x5c9c2, "ECAP", 4) ||
	    get_unaligned_le32(a + 0x5c9c6) != 8)
		goto out;
	memset(a + 0x5c8e7, 0, 8);
	memset(a + 0x5c9ca, 0, 8);
	sha256(a, 0x60000, digest);
	ret = memcmp(digest, expected, sizeof(expected)) ? -EINVAL : 0;
out:
	kvfree(a);
	return ret;
}

/* Prestart admission only. After RUN, equal snapshots are diagnostic, not a lock. */
static int m4_hello_graph(struct m4_gpu_hello *hello, bool host, bool firmware)
{
	struct gpu_m4_graph graph = {
		.tables_pa = hello->region[M4_TABLES].start,
		.shared_pa = hello->region[M4_SHARED].start,
		.text_pa = hello->region[M4_TEXT].start,
		.data_pa = hello->region[M4_DATA].start,
	};
	u8 *snapshot;
	bool equal;
	int ret = -ENOMEM;

	snapshot = kvmalloc(2 * 0x44000, GFP_KERNEL);
	if (!snapshot)
		return ret;
	for (int pass = 0; pass < 2; pass++) {
		dcache_inval_poc((unsigned long)hello->tables, (unsigned long)hello->tables + 0x40000);
		dcache_inval_poc((unsigned long)hello->shared, (unsigned long)hello->shared + 0x4000);
		memcpy(snapshot + pass * 0x44000, hello->tables, 0x40000);
		memcpy(snapshot + pass * 0x44000 + 0x40000, hello->shared, 0x4000);
	}
	equal = !memcmp(snapshot, snapshot + 0x44000, 0x44000);
	hello->host_qualified =
		gpu_m4_host_root_valid(get_unaligned_le64(snapshot + 16),
			hello->region[M4_SHARED].start, hello->gpu->firmware_start_attempted) &&
		gpu_m4_host_root_valid(get_unaligned_le64(snapshot + 0x44000 + 16),
			hello->region[M4_SHARED].start, hello->gpu->firmware_start_attempted) &&
		(hello->ready && hello->ready->published ? m4_ready_tables_valid(hello) :
		 (!memchr_inv(snapshot + 0x40000, 0, 0x4000) &&
		  !memchr_inv(snapshot + 0x88000 - 0x4000, 0, 0x4000)));
	if (!equal) {
		ret = -EAGAIN;
		goto log;
	}
	graph.tables = snapshot;
	graph.shared = snapshot + 0x40000;
	/* Our nonempty index-2 subtree is validated independently in full, including
	 * guard entries. The inherited firmware-only validator needs an empty view.
	 */
	if (hello->ready && hello->ready->published && hello->host_qualified)
		memset(snapshot + 0x40000, 0, 0x4000);
	ret = gpu_m4_graph_check(&graph, host, firmware);
	if (hello->ready && hello->ready->published)
		memcpy(snapshot + 0x40000, snapshot + 0x84000, 0x4000);
	/* No hidden stale tables available for the firmware's later allocator. */
	if (!hello->gpu->firmware_start_attempted && !ret)
		for (unsigned int i = 0; i < 16; i++) {
			if (graph.visited & BIT(i))
				continue;
			if (memchr_inv(snapshot + i * 0x4000, 0, 0x4000)) {
				ret = -EINVAL;
				break;
			}
		}
	hello->firmware_leaves = graph.leaves;
log:
	if (hello->gpu->firmware_start_attempted)
		m4_hello_graph_log(hello, snapshot, &graph, equal);
	kvfree(snapshot);
	return ret;
}

static int m4_hello_lock(struct m4_gpu_hello *hello)
{
	void __iomem *h = hello->handoff;
	u64 deadline = ktime_get_ns() + 100 * NSEC_PER_MSEC;

	writeb(1, h + 0x10);
	mb();
	while (readb(h + 0x11)) {
		if (ktime_get_ns() >= deadline)
			goto timeout;
		if (readl(h + 0x14)) {
			writeb(0, h + 0x10);
			mb();
			while (readl(h + 0x14)) {
				if (ktime_get_ns() >= deadline)
					goto timeout;
				usleep_range(10, 20);
			}
			writeb(1, h + 0x10);
			mb();
		}
		usleep_range(10, 20);
	}
	mb();
	return 0;
timeout:
	writeb(0, h + 0x10);
	mb();
	return -ETIMEDOUT;
}

static void m4_hello_unlock(struct m4_gpu_hello *hello)
{
	writel(1, hello->handoff + 0x14);
	mb();
	writeb(0, hello->handoff + 0x10);
	mb();
}

static bool m4_hello_flush_idle(void __iomem *handoff)
{
	/* Cold ownership includes stale address/length words, not just state. */
	for (unsigned int i = 0; i < 65 * 3; i++)
		if (readq(handoff + 0x20 + i * 8))
			return false;
	return true;
}

static int m4_hello_publish(struct m4_gpu_hello *hello)
{
	struct m4_gpu_power *gpu = hello->gpu;
	void __iomem *h = hello->handoff;
	u64 magic = 0x4b1d000000000002ULL, fw = readq(h + 8), ap = readq(h);
	u64 *upper = hello->tables;
	int ret;

	if ((fw && fw != magic) || (ap && ap != magic) || readb(h + 0x10) || readb(h + 0x11))
		return -EBUSY;
	if (!m4_hello_flush_idle(h))
		return -EBUSY;
	ret = m4_gpu_stopped(gpu);
	if (ret)
		return ret;
	ret = m4_hello_graph(hello, false, false);
	if (ret)
		return ret;
	ret = m4_gpu_stopped(gpu);
	if (ret)
		return ret;
	/* These are the only host writes to this cache line, all while RUN=0. */
	hello->published = true;
	if (hello->ready)
		m4_ready_publish_tables(hello);
	WRITE_ONCE(upper[2], cpu_to_le64(hello->region[M4_SHARED].start | 3));
	dcache_clean_poc((unsigned long)hello->tables, (unsigned long)hello->tables + 0x4000);
	dcache_clean_poc((unsigned long)page_address(hello->lower),
			 (unsigned long)page_address(hello->lower) + 0x4000);
	if (runtime_enable) {
		ret = m4_user_upper_init_locked(hello);
		if (ret)
			return ret;
	}
	writeq(magic, h);
	writeb(0, h + 0x10);
	writeb(0, h + 0x11);
	writel(0, h + 0x14);
	writel(0xffff, h + 0x18);
	if (fw != magic)
		writeb(1, h + 0x638);
	writeq(0, h + 0x640);
	for (unsigned int i = 0; i < 65 * 3; i++)
		writeq(0, h + 0x20 + i * 8);
	mb();
	ret = m4_hello_lock(hello);
	if (ret)
		return ret;
	ret = m4_gpu_stopped(gpu);
	if (!ret) {
		hello->published = true;
		writeq(page_to_phys(hello->lower) | 1, gpu->ttbs);
		writeq(hello->region[M4_TABLES].start | 1, gpu->ttbs + 8);
		mb();
		if (readq(gpu->ttbs) != (page_to_phys(hello->lower) | 1) ||
		    readq(gpu->ttbs + 8) != (hello->region[M4_TABLES].start | 1))
			ret = -EIO;
	}
	m4_hello_unlock(hello);
	return ret;
}

static int m4_hello_shmem(void *cookie, u8 endpoint, struct apple_rtkit_shmem *bfr)
{
	struct m4_gpu_hello *hello = cookie;
	struct resource *data = &hello->region[M4_DATA];
	int ret = -EINVAL;

	if (endpoint != 1 || bfr->size != 0x8000 || !bfr->iova || (bfr->iova & 0xfff) ||
	    bfr->iova < data->start || bfr->iova > data->end ||
	    bfr->size - 1 > data->end - bfr->iova ||
	    atomic_cmpxchg(&hello->crash_claimed, 0, 1))
		goto fail;
	bfr->iomem = devm_ioremap_wc(hello->gpu->dev, bfr->iova, bfr->size);
	if (!bfr->iomem) {
		ret = -ENOMEM;
		goto fail;
	}
	bfr->is_mapped = true;
	hello->crash_pa = bfr->iova;
	hello->crash_size = bfr->size;
	complete_all(&hello->crash_ready);
	dev_info(hello->gpu->dev, "GPU HELLO retained crash storage pa=%#llx size=%#x\n",
		 hello->crash_pa, hello->crash_size);
	return 0;
fail:
	atomic_cmpxchg(&hello->callback_error, 0, ret);
	return ret;
}

static void m4_hello_destroy(void *cookie, struct apple_rtkit_shmem *bfr)
{
	/* No teardown is qualified. Firmware storage and owner stay pinned. */
}

static void m4_hello_crashed(void *cookie, const void *data, size_t size)
{
	struct m4_gpu_hello *hello = cookie;

	atomic_cmpxchg(&hello->callback_error, 0, -EIO);
}

static void m4_hello_message(void *cookie, u8 ep, u64 message)
{
	struct m4_gpu_hello *hello = cookie;

	if (hello->ready && READ_ONCE(hello->ready->applications) &&
	    ep == 0x20 && message == 0x0042000000000000ULL) {
		m4_ready_notify(hello);
		return;
	}
	if (!atomic_cmpxchg(&hello->callback_error, 0, -EPROTO))
		dev_err(hello->gpu->dev, "Unexpected GPU application message ep=%#x word=%#llx; owner poisoned\n",
			ep, message);
}

/* Atomic receive context (after rtkit's dma_rmb): handle ONLY the exact
 * coalesced ring notification without a receive-worker hop. Everything else
 * keeps the process-context receiver and its error semantics.
 */
static bool m4_hello_message_early(void *cookie, u8 ep, u64 message)
{
	struct m4_gpu_hello *hello = cookie;

	if (!hello->ready || !READ_ONCE(hello->ready->applications) ||
	    ep != 0x20 || message != 0x0042000000000000ULL)
		return false;
	m4_ready_notify(hello);
	return true;
}

static const struct apple_rtkit_ops m4_hello_ops = {
	.protocol_version = 12,
	.shmem_setup_ep = m4_hello_shmem,
	.shmem_destroy = m4_hello_destroy,
	.crashed = m4_hello_crashed,
	.recv_message_early = m4_hello_message_early,
	.recv_message = m4_hello_message,
};

static int m4_hello_start(struct m4_gpu_hello *hello)
{
	struct m4_gpu_power *gpu = hello->gpu;
	struct device *dev = gpu->dev;
	int ret;

	ret = m4_hello_regions(hello);
	if (ret)
		return ret;
	ret = m4_hello_hash(hello);
	if (ret)
		return ret;
	hello->tables = devm_memremap(dev, hello->region[M4_TABLES].start, 0x40000, MEMREMAP_WB);
	if (IS_ERR(hello->tables))
		return PTR_ERR(hello->tables);
	hello->shared = devm_memremap(dev, hello->region[M4_SHARED].start, 0x4000, MEMREMAP_WB);
	if (IS_ERR(hello->shared))
		return PTR_ERR(hello->shared);
	hello->handoff = devm_ioremap_wc(dev, hello->region[M4_HANDOFF].start, 0x4000);
	if (!hello->handoff)
		return -ENOMEM;
	ret = m4_gpu_stopped(gpu);
	if (ret)
		return ret;
	ret = m4_hello_graph(hello, false, false);
	if (ret)
		return ret;
	hello->lower = alloc_page(GFP_KERNEL | __GFP_ZERO);
	if (!hello->lower)
		return -ENOMEM;
	if (PAGE_SIZE != 0x4000 || (page_to_phys(hello->lower) & ~GPU_M4_PA_MASK))
		return -EINVAL;
	if (hello->ready) {
		ret = m4_ready_prepare(hello);
		if (ret)
			return ret;
		if (runtime_enable) {
			ret = m4_user_prerun_prepare_locked(hello);
			if (ret)
				return ret;
		}
	}
	/* Bind callbacks before publication/start; this is the GPU's sole mailbox owner. */
	hello->rtkit = devm_apple_rtkit_init(dev, hello, NULL, 0, &m4_hello_ops);
	if (IS_ERR(hello->rtkit))
		return PTR_ERR(hello->rtkit);
	ret = m4_hello_publish(hello);
	if (ret)
		return ret;
	ret = m4_hello_graph(hello, true, false);
	if (ret)
		return ret;
	gpu->control = readl(gpu->asc + 0x44);
	if (gpu->control & BIT(4))
		return -EBUSY;
	gpu->stopped_verified = false;
	gpu->firmware_start_attempted = true;
	writel(gpu->control | BIT(4), gpu->asc + 0x44);
	mb();
	gpu->control = readl(gpu->asc + 0x44);
	if (!(gpu->control & BIT(4)))
		return -EIO;
	ret = apple_rtkit_wake(hello->rtkit);
	if (ret)
		return ret;
	if (!wait_for_completion_timeout(&hello->crash_ready, msecs_to_jiffies(1000)))
		return -ETIMEDOUT;
	if (atomic_read(&hello->callback_error) || !apple_rtkit_is_running(hello->rtkit) ||
	    apple_rtkit_is_crashed(hello->rtkit) || !hello->crash_size ||
	    !apple_rtkit_has_endpoint(hello->rtkit, 1) ||
	    !apple_rtkit_has_endpoint(hello->rtkit, 0x20) ||
	    !apple_rtkit_has_endpoint(hello->rtkit, 0x21))
		return -EPROTO;
	/* Firmware-owned paths are diagnostic, not an atomic live-publication fence. */
	hello->graph_diagnostic = m4_hello_graph(hello, true, true);
	gpu->control = readl(gpu->asc + 0x44);
	gpu->pstate = readl(gpu->sgx_core + 0x1000) & 0xf;
	if (!(gpu->control & BIT(4)) || gpu->pstate > 9 ||
	    !hello->host_qualified ||
	    atomic_read(&hello->callback_error) || !apple_rtkit_is_running(hello->rtkit) ||
	    readq(gpu->ttbs) != (page_to_phys(hello->lower) | 1) ||
	    readq(gpu->ttbs + 8) != (hello->region[M4_TABLES].start | 1))
		return -EPROTO;
	hello->complete = true;
	return 0;
}

#include "gpu-m4-ready.h"

static ssize_t hello_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_hello *h;
	ssize_t size;

	mutex_lock(&gpu->lock);
	h = gpu->hello;
	if (!h) {
		size = sysfs_emit(buf, "attempted=0\n");
	} else {
		bool running = !IS_ERR_OR_NULL(h->rtkit) && apple_rtkit_is_running(h->rtkit) &&
			!apple_rtkit_is_crashed(h->rtkit);
		bool crash_ready = completion_done(&h->crash_ready);

		size = sysfs_emit(buf, "attempted=1 complete=%u error=%d callback_error=%d run=%u rtkit_running=%u pstate=%u uat_published=%u crash_pa=%#llx crash_size=%#x firmware_leaves=%u graph_diagnostic=%d live_graph_qualified=0\n",
			h->complete, h->error, atomic_read(&h->callback_error),
			!!(readl(gpu->asc + 0x44) & BIT(4)), running,
			readl(gpu->sgx_core + 0x1000) & 0xf, h->published,
			crash_ready ? h->crash_pa : 0, crash_ready ? h->crash_size : 0,
			h->firmware_leaves, h->graph_diagnostic);
	}
	mutex_unlock(&gpu->lock);
	return size;
}
static DEVICE_ATTR_RO(hello_state);

static ssize_t hello_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_hello *hello;
	int ret = -EINVAL;

	if (!hello_enable || !sysfs_streq(buf, "1"))
		return -EINVAL;
	mutex_lock(&gpu->lock);
	if (gpu->hello || gpu->error || !gpu->voted || !gpu->stopped_verified)
		goto out;
	hello = devm_kzalloc(dev, sizeof(*hello), GFP_KERNEL);
	if (!hello) {
		ret = -ENOMEM;
		goto out;
	}
	hello->gpu = gpu;
	init_completion(&hello->crash_ready);
	hello->graph_diagnostic = -ENODATA;
	gpu->hello = hello;
	ret = m4_hello_start(hello);
	hello->error = ret;
	dev_info(dev, "GPU HELLO complete=%u error=%d callbacks=%d leaves=%u graph_diagnostic=%d; no application startup\n",
		 hello->complete, ret, atomic_read(&hello->callback_error),
		 hello->firmware_leaves, hello->graph_diagnostic);
out:
	mutex_unlock(&gpu->lock);
	return ret ? ret : (ssize_t)count;
}
static DEVICE_ATTR_WO(hello);

#endif
