// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* J713 retained DCP bring-up and fixed-mode diagnostic scanout; no DRM takeover. */
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/soc/apple/dart.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/soc/apple/pmp-display.h>

#include "dcp-m4-profile.h"
#include "dcp-m4-protocol.h"
#include "dcp-m4-transaction.h"
#include "dcp-m4-timing.h"
#include "dcp-m4-memory.h"

struct dcp_m4_ready {
	struct device *dev;
	void __iomem *coproc;
	struct apple_rtkit *rtkit;
	struct mutex lock;
	spinlock_t state_lock;
	struct completion reply;
	struct completion rpc_reply;
	struct dcp_m4_transaction transaction;
	struct platform_device *piodma;
	struct platform_device *scanout;
	struct dcp_m4_memory *memory;
	struct dcp_m4_runtime *runtime;
	void *rpc;
	dma_addr_t rpc_dva;
	u64 ready_word;
	u64 rpc_word;
	size_t allocated;
	int error;
	bool started;
	bool init_sent;
	bool ready;
	bool rpc_started;
	bool rpc_waiting;
	bool rpc_received;
	bool rpc_done;
	bool shared_started;
	bool shared_done;
	bool startup_started;
	bool kms_started;
	struct dcp_m4_kms *kms;
	struct dcp_m4_ext *ext[2];	/* External DCPEXT owners, independent of this DCP. */
};

/*
 * Per-board DART/PMP glue. J713 keeps its retained-root DART owner and PMP
 * display provider. J514S uses the generic locked-DART DMA API: the inherited
 * mappings are protected as reserved iommu-addresses regions.
 */
static int dcp_m4_dart_check(struct device *dev)
{
	if (!dcp_m4_pf->fw14)
		return apple_dart_dcp_check(dev);
	return iommu_get_domain_for_dev(dev) ? 0 : -ENODEV;
}

static int dcp_m4_dart_check_buffer(struct device *dev, dma_addr_t iova, size_t size)
{
	if (!dcp_m4_pf->fw14)
		return apple_dart_dcp_check_buffer(dev, iova, size);
	return dcp_m4_dart_check(dev);
}

static void *dcp_m4_dma_alloc(struct device *dev, size_t size, dma_addr_t *dva)
{
	void *cpu;

	if (!dcp_m4_pf->fw14)
		return apple_dart_dcp_alloc(dev, size, dva);
	cpu = dma_alloc_coherent(dev, size, dva, GFP_KERNEL);
	return cpu ?: ERR_PTR(-ENOMEM);
}

static void *dcp_m4_scanout_alloc(struct device *dev, size_t size, dma_addr_t *dva)
{
	void *cpu;

	if (!dcp_m4_pf->fw14)
		return apple_dart_dcp_scanout_alloc(dev, size, dva);
	cpu = dma_alloc_coherent(dev, size, dva, GFP_KERNEL);
	return cpu ?: ERR_PTR(-ENOMEM);
}

static int dcp_m4_pmp_prepare(struct device *dev)
{
	return dcp_m4_pf->fw14 ? apple_pmp_j514s_display_prepare(dev) :
		apple_pmp_j713_display_prepare(dev);
}

static int dcp_m4_pmp_get(struct device *dev)
{
	return dcp_m4_pf->fw14 ? apple_pmp_j514s_display_get(dev) :
		apple_pmp_j713_display_get(dev);
}

static int dcp_m4_pmp_check(struct device *dev)
{
	return dcp_m4_pf->fw14 ? apple_pmp_j514s_display_check(dev) :
		apple_pmp_j713_display_check(dev);
}

/* Board identity, handoff markers and the DCP ASC address. */
struct dcp_m4_board {
	const struct dcp_m4_profile *profile;
	const char *machine, *dcp, *uuid, *inherited, *scanout;
	u64 coproc;
};

static const struct dcp_m4_board dcp_m4_boards[] = {
	{ &dcp_m4_profile_j713, "apple,j713", "apple,t8132-dcp",
	  "81BAF476-679D-3CC0-8C6A-E6B6FA659D5E", "apple,j713-inherited-mappings",
	  "apple,t8132-display-diagnostics", 0x30ae00000ULL },
	{ &dcp_m4_profile_j514s, "apple,j514s", "apple,t6030-dcp",
	  "DDF38191-93B3-324A-BC8F-643006F5AC82", "apple,j514s-inherited-mappings",
	  "apple,t6030-display-diagnostics", 0x28ec00000ULL },
};

static const struct dcp_m4_board *dcp_m4_board = &dcp_m4_boards[0];

static void dcp_m4_runtime_wake(struct dcp_m4_ready *dcp);
static void dcp_m4_runtime_receive(struct dcp_m4_ready *dcp, u64 word);

static bool dcp_m4_range(u64 base, u64 length, u64 addr, u64 size)
{
	return length && size && addr >= base && addr - base < length &&
		size <= length - (addr - base);
}

/* Only the consumer's own no-map ASC reservations may back firmware buffers. */
static bool dcp_m4_reserved(struct dcp_m4_ready *dcp, u64 pa, size_t size, bool oslog)
{
	struct device_node *node;
	struct resource res;
	bool found = false;

	for (int i = 0; (node = of_parse_phandle(dcp->dev->of_node, "memory-region", i)); i++) {
		struct reserved_mem *rmem = of_reserved_mem_lookup(node);

		if (of_device_is_compatible(node, "apple,asc-mem") &&
		    of_property_read_bool(node, "no-map") &&
		    oslog == of_property_read_bool(node, "apple,dcp-os-log") &&
		    !of_address_to_resource(node, 0, &res) &&
		    rmem && rmem->base == res.start && rmem->size == resource_size(&res) &&
		    dcp_m4_range(res.start, resource_size(&res), pa, size))
			found = true;
		of_node_put(node);
		if (found)
			break;
	}
	return found;
}

static void dcp_m4_failed(struct dcp_m4_ready *dcp, int error)
{
	unsigned long flags;

	spin_lock_irqsave(&dcp->state_lock, flags);
	if (!dcp->error)
		dcp->error = error;
	dcp->ready = false;
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	complete_all(&dcp->reply);
	complete_all(&dcp->rpc_reply);
	dcp_m4_runtime_wake(dcp);
}

static int dcp_m4_shmem_setup(void *cookie, struct apple_rtkit_shmem *bfr)
{
	struct dcp_m4_ready *dcp = cookie;
	struct iommu_domain *domain = iommu_get_domain_for_dev(dcp->dev);
	phys_addr_t pa;
	int ret = -EINVAL;

	if (!bfr->size || bfr->size > SZ_16M || dcp->allocated > SZ_64M - bfr->size ||
	    dcp_m4_dart_check(dcp->dev))
		goto fail;
	if (!bfr->iova) {
		bfr->buffer = dcp_m4_dma_alloc(dcp->dev, bfr->size, &bfr->iova);
		if (IS_ERR(bfr->buffer)) {
			ret = PTR_ERR(bfr->buffer);
			bfr->buffer = NULL;
			goto fail;
		}
		/* Allocation stays pinned even if subsequent transport validation fails. */
		dcp->allocated += bfr->size;
		if (!dcp_m4_range(1ULL << 40, 1ULL << 36, bfr->iova, bfr->size) ||
		    dcp_m4_dart_check(dcp->dev))
			goto fail;
		bfr->is_mapped = false;
		dev_info(dcp->dev, "RTKit allocated %zu bytes at %pad\n", bfr->size, &bfr->iova);
		return 0;
	}
	/* OSLOG is explicitly marked physical; other requests are SID23 DVAs. */
	if (dcp_m4_reserved(dcp, bfr->iova, bfr->size, true)) {
		/* A physical address can also fall in the DVA aperture; refuse ambiguity. */
		for (size_t off = 0; off < bfr->size;) {
			if (iommu_iova_to_phys(domain, bfr->iova + off))
				goto fail;
			off += SZ_16K - ((bfr->iova + off) & (SZ_16K - 1));
		}
		if (dcp_m4_dart_check(dcp->dev))
			goto fail;
		pa = bfr->iova;
	} else {
		if (!dcp_m4_range(1ULL << 40, 1ULL << 36, bfr->iova, bfr->size))
			goto fail;
		ret = dcp_m4_dart_check_buffer(dcp->dev, bfr->iova, bfr->size);
		if (ret)
			goto fail;
		ret = -EINVAL;
		pa = iommu_iova_to_phys(domain, bfr->iova);
		if (!pa || !dcp_m4_reserved(dcp, pa, bfr->size, false))
			goto fail;
		/* Include the initial unaligned span and every crossed 16-KiB page. */
		for (size_t off = SZ_16K - (bfr->iova & (SZ_16K - 1)); off < bfr->size;
		     off += SZ_16K)
			if (iommu_iova_to_phys(domain, bfr->iova + off) != pa + off)
				goto fail;
	}
	/* Firmware payload is Normal-NC, independent of coherent WB table walks. */
	bfr->iomem = ioremap_wc(pa, bfr->size);
	if (!bfr->iomem) {
		ret = -ENOMEM;
		goto fail;
	}
	bfr->is_mapped = true;
	dcp->allocated += bfr->size;
	dev_info(dcp->dev, "RTKit borrowed %zu bytes at %pad -> %pa\n",
		 bfr->size, &bfr->iova, &pa);
	return 0;
fail:
	dcp_m4_failed(dcp, ret);
	return ret;
}

static void dcp_m4_shmem_destroy(void *cookie, struct apple_rtkit_shmem *bfr)
{
	/* No qualified ownership-return boundary yet. Never recycle firmware DMA. */
}

static void dcp_m4_crashed(void *cookie, const void *log, size_t size)
{
	struct dcp_m4_ready *dcp = cookie;

	dev_err(dcp->dev, "RTKit crashed; retaining session and DMA until reboot\n");
	dcp_m4_failed(dcp, -EIO);
}

static void dcp_m4_receive(void *cookie, u8 endpoint, u64 message)
{
	struct dcp_m4_ready *dcp = cookie;
	unsigned long flags;
	int ret;

	dev_dbg(dcp->dev, "RTKit EP%02x word %016llx\n", endpoint, message);
	if (endpoint != DCP_M4_ENDPOINT) {
		dcp_m4_failed(dcp, -EPROTO);
		return;
	}
	if (READ_ONCE(dcp->runtime)) {
		dcp_m4_runtime_receive(dcp, message);
		return;
	}
	spin_lock_irqsave(&dcp->state_lock, flags);
	if (dcp->rpc_started) {
		/* One-slot notification queue for the one-reply A457 milestone.
		 * The sysfs task is its serialized owner; never block RTKit's worker.
		 * A second message, including an idle-time duplicate, poisons the session.
		 */
		ret = !dcp->error && dcp->rpc_waiting && !dcp->rpc_received ? 0 : -EPROTO;
		if (!ret) {
			dcp->rpc_word = message;
			dcp->rpc_received = true;
		}
		spin_unlock_irqrestore(&dcp->state_lock, flags);
		if (ret)
			dcp_m4_failed(dcp, ret);
		else
			complete(&dcp->rpc_reply);
		return;
	}
	ret = dcp_m4_ready_notification(message, dcp->init_sent, dcp->ready_word);
	if (ret == 1)
		dcp->ready_word = message;
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	if (ret < 0)
		dcp_m4_failed(dcp, ret);
	else if (ret == 1)
		complete_all(&dcp->reply);
}

static const struct apple_rtkit_ops dcp_m4_rtkit_ops = {
	.recv_message = dcp_m4_receive,
	.crashed = dcp_m4_crashed,
	.shmem_setup = dcp_m4_shmem_setup,
	.shmem_destroy = dcp_m4_shmem_destroy,
};

static ssize_t ready_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dcp_m4_ready *dcp = dev_get_drvdata(dev);
	unsigned long flags;
	ssize_t ret;

	spin_lock_irqsave(&dcp->state_lock, flags);
	ret = sysfs_emit(buf, "ready=%u error=%d word=%016llx run=%u\n",
			 dcp->ready, dcp->error, dcp->ready_word,
			 !!(readl(dcp->coproc + 0x44) & BIT(4)));
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	return ret;
}

static ssize_t ready_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct dcp_m4_ready *dcp = dev_get_drvdata(dev);
	unsigned long flags;
	u64 init_word;
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;
	mutex_lock(&dcp->lock);
	if (dcp->started) {
		ret = -EBUSY;
		goto out;
	}
	ret = dcp_m4_dart_check(dev);
	if (ret)
		goto out;
	/* J514S: the PMP must have acknowledged DISP before the DCP is adopted. */
	if (dcp_m4_pf->fw14) {
		ret = dcp_m4_pmp_get(dev);
		if (ret) {
			dev_err(dev, "PMP DISP acknowledgement missing: %d\n", ret);
			goto out;
		}
	}
	if (!(readl(dcp->coproc + 0x44) & BIT(4))) {
		ret = -ENODEV;
		goto out;
	}
	/* From the first message onward, neither failure nor unbind may tear down. */
	dcp->started = true;
	dev_info(dev, "Starting warm RTKit adoption; no resets or display takeover\n");
	ret = apple_rtkit_wake(dcp->rtkit);
	if (ret)
		goto fail;
	if (READ_ONCE(dcp->error)) {
		ret = READ_ONCE(dcp->error);
		goto fail;
	}
	if (!apple_rtkit_has_endpoint(dcp->rtkit, DCP_M4_ENDPOINT)) {
		ret = -ENODEV;
		goto fail;
	}
	dev_info(dev, "RTKit awake; endpoint 0x37 advertised\n");
	dcp->rpc = dcp_m4_dma_alloc(dev, DCP_M4_RPC_SIZE, &dcp->rpc_dva);
	if (IS_ERR(dcp->rpc)) {
		ret = PTR_ERR(dcp->rpc);
		dcp->rpc = NULL;
		goto fail;
	}
	ret = dcp_m4_dart_check(dev);
	if (ret)
		goto fail;
	ret = dcp_m4_init_word(dcp->rpc_dva, &init_word);
	if (ret)
		goto fail;
	if (!dcp_m4_range(1ULL << 40, 1ULL << 36, dcp->rpc_dva, DCP_M4_RPC_SIZE)) {
		ret = -ERANGE;
		goto fail;
	}
	memset(dcp->rpc, 0, DCP_M4_RPC_SIZE);
	put_unaligned_le32(DCP_M4_ALIGNMENT, dcp->rpc + 12);
	dma_wmb();
	ret = apple_rtkit_start_ep(dcp->rtkit, DCP_M4_ENDPOINT);
	if (ret)
		goto fail;
	dev_info(dev, "Sending INIT %016llx RPC=%pad size=%x alignment=%u\n",
		 init_word, &dcp->rpc_dva, DCP_M4_RPC_SIZE, DCP_M4_ALIGNMENT);
	spin_lock_irqsave(&dcp->state_lock, flags);
	dcp->init_sent = true;
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	ret = apple_rtkit_send_message(dcp->rtkit, DCP_M4_ENDPOINT, init_word, NULL, false);
	if (ret)
		goto fail;
	if (!wait_for_completion_timeout(&dcp->reply, msecs_to_jiffies(3000))) {
		ret = -ETIMEDOUT;
		goto fail;
	}
	ret = READ_ONCE(dcp->error);
	if (ret)
		goto fail;
	if (!dcp_m4_ready(dcp->ready_word) || !(readl(dcp->coproc + 0x44) & BIT(4))) {
		ret = -EPROTO;
		goto fail;
	}
	spin_lock_irqsave(&dcp->state_lock, flags);
	ret = dcp->error;
	if (!ret)
		dcp->ready = true;
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	if (ret)
		goto fail;
	dev_info(dev, "DCPLink READY %016llx; simpledrm retained, no IOMFB calls\n",
		 dcp->ready_word);
	ret = count;
	goto out;
fail:
	dcp_m4_failed(dcp, ret);
	dev_err(dev, "READY failed: %d; session pinned until guarded reboot\n", ret);
out:
	mutex_unlock(&dcp->lock);
	return ret;
}
static DEVICE_ATTR_RW(ready);

static ssize_t rpc_probe_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dcp_m4_ready *dcp = dev_get_drvdata(dev);
	unsigned long flags;
	ssize_t ret;

	spin_lock_irqsave(&dcp->state_lock, flags);
	ret = sysfs_emit(buf, "attempted=%u complete=%u error=%d reply=%016llx run=%u\n",
			 dcp->rpc_started, dcp->rpc_done && !dcp->error, dcp->error,
			 dcp->rpc_word, !!(readl(dcp->coproc + 0x44) & BIT(4)));
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	return ret;
}

/* A457 only. A401 requires qualified shared SID23/SID16 backing first. */
static ssize_t rpc_probe_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct dcp_m4_ready *dcp = dev_get_drvdata(dev);
	unsigned long flags;
	u64 word;
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;
	mutex_lock(&dcp->lock);
	spin_lock_irqsave(&dcp->state_lock, flags);
	ret = dcp->error ? dcp->error :
		!dcp->ready ? -ENODEV :
		(dcp->rpc_started || dcp->startup_started) ? -EBUSY : 0;
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	if (ret)
		goto out;
	ret = dcp_m4_dart_check(dev);
	if (ret)
		goto fail;
	if (!(readl(dcp->coproc + 0x44) & BIT(4))) {
		ret = -ENODEV;
		goto fail;
	}
	{
		unsigned int m = dcp_m4_pf->open_method;
		u32 tag = 'A' | (('0' + m / 100) << 8) | (('0' + m / 10 % 10) << 16) |
			(('0' + m % 10) << 24);

		ret = dcp_m4_call_begin(&dcp->transaction, dcp->rpc, DCP_M4_RPC_SIZE,
				      tag, false, NULL, 0, 0, &word);
	}
	if (ret)
		goto fail;
	dma_wmb();
	spin_lock_irqsave(&dcp->state_lock, flags);
	ret = dcp->error;
	if (!ret) {
		dcp->rpc_started = true;
		dcp->rpc_waiting = true;
	}
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	if (ret)
		goto fail;
	dev_info(dev, "Sending isolated A457 %016llx; no startup or modeset\n", word);
	ret = apple_rtkit_send_message(dcp->rtkit, DCP_M4_ENDPOINT, word, NULL, false);
	if (ret)
		goto fail;
	if (!wait_for_completion_timeout(&dcp->rpc_reply, msecs_to_jiffies(5000))) {
		ret = -ETIMEDOUT;
		goto fail;
	}
	spin_lock_irqsave(&dcp->state_lock, flags);
	ret = dcp->error ? dcp->error : !dcp->rpc_received ? -EPROTO : 0;
	word = dcp->rpc_word;
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	if (ret)
		goto fail;
	/* The helper rejects callbacks, wrong routing and mutated packet headers.
	 * No callback is ACKed by this isolated transport probe.
	 */
	dma_rmb();
	ret = dcp_m4_call_reply(&dcp->transaction, dcp->rpc, DCP_M4_RPC_SIZE,
			      word, NULL, 0);
	if (ret)
		goto fail;
	ret = dcp_m4_dart_check(dev);
	if (ret)
		goto fail;
	if (!(readl(dcp->coproc + 0x44) & BIT(4))) {
		ret = -ENODEV;
		goto fail;
	}
	spin_lock_irqsave(&dcp->state_lock, flags);
	ret = dcp->error;
	dcp->rpc_waiting = false;
	if (!ret)
		dcp->rpc_done = true;
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	if (ret)
		goto fail;
	dev_info(dev, "Isolated A457 reply accepted; no IOMFB startup or modeset\n");
	ret = count;
	goto out;
fail:
	dcp_m4_transaction_fail(&dcp->transaction, ret);
	dcp_m4_failed(dcp, ret);
	dev_err(dev, "A457 failed: %d; session pinned until guarded reboot\n", ret);
out:
	mutex_unlock(&dcp->lock);
	return ret;
}
static DEVICE_ATTR_RW(rpc_probe);

static int dcp_m4_shared_alloc(void *cookie, u32 size, u64 *phys, u64 *dva)
{
	struct dcp_m4_ready *dcp = cookie;

	struct iommu_domain *domain;
	dma_addr_t iova;
	phys_addr_t pa;
	void *cpu;

	if (!dcp_m4_pf->fw14)
		return apple_dart_dcp_shared_alloc(dcp->dev, &dcp->piodma->dev, size, phys, dva);
	/* Zeroed, physically contiguous, pinned until reboot (no ownership return). */
	domain = iommu_get_domain_for_dev(dcp->dev);
	if (!domain)
		return -ENODEV;
	cpu = dma_alloc_attrs(dcp->dev, size, &iova, GFP_KERNEL | __GFP_ZERO,
			      DMA_ATTR_FORCE_CONTIGUOUS);
	if (!cpu)
		return -ENOMEM;
	pa = iommu_iova_to_phys(domain, iova);
	if (!pa)
		return -EIO;
	for (u32 off = 0; off < size; off += SZ_16K)
		if (iommu_iova_to_phys(domain, iova + off) != pa + off)
			return -EIO;
	*phys = pa;
	*dva = iova;
	return 0;
}

static int dcp_m4_shared_map(void *cookie, u64 phys, u64 dva, u32 size)
{
	struct dcp_m4_ready *dcp = cookie;

	struct iommu_domain *domain;
	int ret;

	if (!dcp_m4_pf->fw14)
		return apple_dart_dcp_shared_map(dcp->dev, &dcp->piodma->dev, phys, dva, size);
	/* Same PA and size at the same DVA in the PIODMA (SID4) domain. */
	domain = iommu_get_domain_for_dev(&dcp->piodma->dev);
	if (!domain)
		return -ENODEV;
	ret = iommu_map(domain, dva, phys, size, IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (ret)
		return ret;
	for (u32 off = 0; off < size; off += SZ_16K)
		if (iommu_iova_to_phys(domain, dva + off) != phys + off)
			return -EIO;
	return 0;
}

static const struct dcp_m4_memory_ops dcp_m4_memory_ops = {
	.alloc = dcp_m4_shared_alloc,
	.map = dcp_m4_shared_map,
};

static int dcp_m4_create_piodma(struct dcp_m4_ready *dcp)
{
	struct device_node *node, *alias;
	struct platform_device *existing;
	int ret = -ENODEV;

	node = of_get_child_by_name(dcp->dev->of_node, "piodma");
	alias = of_find_node_by_path("disp0_piodma");
	if (!node || node != alias || !of_device_is_available(node) ||
	    !of_property_read_bool(node, dcp_m4_board->inherited))
		goto out;
	existing = of_find_device_by_node(node);
	if (existing) {
		put_device(&existing->dev);
		ret = -EBUSY;
		goto out;
	}
	dcp->piodma = of_platform_device_create(node, NULL, dcp->dev);
	if (!dcp->piodma)
		goto out;
	/* Keep the child even on failure: retained-domain attachment can have begun. */
	ret = dma_set_mask_and_coherent(&dcp->piodma->dev, DMA_BIT_MASK(42));
	if (!ret)
		ret = of_dma_configure(&dcp->piodma->dev, node, true);
	if (!ret)
		ret = dcp_m4_dart_check(&dcp->piodma->dev);
out:
	of_node_put(alias);
	of_node_put(node);
	return ret;
}

/* Retain the exact diagnostics consumer; never duplicate an OF platform device. */
static int dcp_m4_acquire_scanout(struct dcp_m4_ready *dcp)
{
	struct device_node *node = of_find_node_by_path("disp0");
	struct platform_device *pdev;
	int ret = -ENODEV;

	if (!node || !of_device_is_available(node) ||
	    !of_device_is_compatible(node, dcp_m4_board->scanout) ||
	    !of_property_read_bool(node, dcp_m4_board->inherited))
		goto out;
	pdev = of_find_device_by_node(node);
	if (!pdev)
		pdev = of_platform_device_create(node, NULL, dcp->dev->parent);
	if (!pdev)
		goto out;
	/* Keep the reference on every outcome: IOMMU attachment can have begun. */
	dcp->scanout = pdev;
	device_lock(&pdev->dev);
	if (pdev->dev.driver) {
		ret = -EBUSY;
		goto unlock;
	}
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(42));
	if (!ret)
		ret = of_dma_configure(&pdev->dev, node, true);
	if (!ret)
		ret = dcp_m4_dart_check(&pdev->dev);
	if (!ret)
		pm_runtime_get_noresume(&pdev->dev);
unlock:
	device_unlock(&pdev->dev);
out:
	of_node_put(node);
	return ret;
}

static ssize_t shared_probe_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dcp_m4_ready *dcp = dev_get_drvdata(dev);
	struct dcp_m4_memory *memory = READ_ONCE(dcp->memory);

	return sysfs_emit(buf, "attempted=%u complete=%u objects=%u error=%d run=%u\n",
		READ_ONCE(dcp->shared_started), READ_ONCE(dcp->shared_done),
		memory ? READ_ONCE(memory->count) : 0, READ_ONCE(dcp->error),
		!!(readl(dcp->coproc + 0x44) & BIT(4)));
}

static ssize_t shared_probe_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct dcp_m4_ready *dcp = dev_get_drvdata(dev);
	static const u32 sizes[] = { 0x40000, 0x40000, 0x4000 };
	u8 input[20], output[28];
	unsigned long flags;
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;
	mutex_lock(&dcp->lock);
	spin_lock_irqsave(&dcp->state_lock, flags);
	ret = dcp->shared_started || !dcp->ready || !dcp->rpc_done || dcp->error ? -EBUSY : 0;
	if (!ret)
		dcp->shared_started = true;
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	if (ret)
		goto out;
	ret = dcp_m4_dart_check(dev);
	if (ret)
		goto fail;
	if (!(readl(dcp->coproc + 0x44) & BIT(4))) {
		ret = -EIO;
		goto fail;
	}
	ret = dcp_m4_create_piodma(dcp);
	if (ret)
		goto fail;
	dcp->memory = devm_kzalloc(dev, sizeof(*dcp->memory), GFP_KERNEL);
	if (!dcp->memory) {
		ret = -ENOMEM;
		goto fail;
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(sizes); i++) {
		memset(input, 0, sizeof(input));
		memset(output, 0, sizeof(output));
		put_unaligned_le32(0x703, input);
		put_unaligned_le64(sizes[i], input + 4);
		put_unaligned_le32(4, input + 12);
		ret = dcp_m4_memory_callback(dcp->memory, &dcp_m4_memory_ops, dcp, 451, input, output);
		if (ret)
			goto fail;
		memset(input, 0, sizeof(input));
		put_unaligned_le64(get_unaligned_le32(output + 24), input);
		put_unaligned_le32(1, input + 8);
		ret = dcp_m4_memory_callback(dcp->memory, &dcp_m4_memory_ops, dcp, 201, input, output);
		if (ret)
			goto fail;
		dev_info(dev, "Shared mapping probe ID%u: PA=%llx DVA=%llx size=%llx; no firmware descriptor sent\n",
			i + 1, dcp->memory->object[i].phys, dcp->memory->object[i].dva,
			dcp->memory->object[i].size);
	}
	ret = dcp_m4_dart_check(dev);
	if (!ret)
		ret = dcp_m4_dart_check(&dcp->piodma->dev);
	if (ret)
		goto fail;
	spin_lock_irqsave(&dcp->state_lock, flags);
	ret = dcp->error;
	if (!ret)
		dcp->shared_done = true;
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	if (ret)
		goto fail;
	ret = count;
	goto out;
fail:
	dcp_m4_failed(dcp, ret);
	dev_err(dev, "Shared mapping probe failed: %d; child and buffers pinned until reboot\n", ret);
out:
	mutex_unlock(&dcp->lock);
	return ret;
}
static DEVICE_ATTR_RW(shared_probe);

#include "dcp-m4-runtime.h"
#include "dcp-m4-kms.h"
#include "dcp-m4-ext-none.h"

static struct attribute *dcp_m4_attrs[] = {
	&dev_attr_ready.attr, &dev_attr_rpc_probe.attr, &dev_attr_shared_probe.attr,
	&dev_attr_startup.attr, &dev_attr_background.attr, &dev_attr_pmp_vote.attr,
	&dev_attr_scanout_probe.attr, &dev_attr_kms.attr, &dev_attr_kms_stats.attr, NULL
};
ATTRIBUTE_GROUPS(dcp_m4);

static int dcp_m4_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dcp_m4_ready *dcp;
	struct resource *res;
	const char *uuid;
	int ret;

	for (unsigned int i = 0; i < ARRAY_SIZE(dcp_m4_boards); i++)
		if (of_machine_is_compatible(dcp_m4_boards[i].machine) &&
		    of_device_is_compatible(dev->of_node, dcp_m4_boards[i].dcp))
			dcp_m4_board = &dcp_m4_boards[i];
	if (!of_machine_is_compatible(dcp_m4_board->machine) ||
	    !of_device_is_compatible(dev->of_node, dcp_m4_board->dcp) ||
	    of_property_read_string(dev->of_node, "apple,firmware-uuid", &uuid) ||
	    strcmp(uuid, dcp_m4_board->uuid)) {
		dev_info(dev, "not a qualified board/firmware; not binding\n");
		return -ENODEV;
	}
	dcp_m4_pf = dcp_m4_board->profile;
	ret = dcp_m4_dart_check(dev);
	if (ret)
		return dev_err_probe(dev, ret, "inherited SID23 domain not qualified\n");
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(42));
	if (ret)
		return ret;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "coproc");
	if (!res || res->start != dcp_m4_board->coproc || resource_size(res) != SZ_16K) {
		dev_info(dev, "unexpected coproc resource; not binding\n");
		return -ENODEV;
	}
	dcp = devm_kzalloc(dev, sizeof(*dcp), GFP_KERNEL);
	if (!dcp)
		return -ENOMEM;
	dcp->dev = dev;
	mutex_init(&dcp->lock);
	spin_lock_init(&dcp->state_lock);
	init_completion(&dcp->reply);
	init_completion(&dcp->rpc_reply);
	dcp->coproc = devm_ioremap_resource(dev, res);
	if (IS_ERR(dcp->coproc))
		return PTR_ERR(dcp->coproc);
	if (!(readl(dcp->coproc + 0x44) & BIT(4))) {
		dev_info(dev, "DCP CPU not running (ASC +0x44 = %#x); not binding\n",
			 readl(dcp->coproc + 0x44));
		return -ENODEV;
	}
	dev_info(dev, "%s DCP profile, CPU running\n", dcp_m4_pf->name);
	dcp->rtkit = devm_apple_rtkit_init(dev, dcp, "mbox", 0, &dcp_m4_rtkit_ops);
	if (IS_ERR(dcp->rtkit))
		return dev_err_probe(dev, PTR_ERR(dcp->rtkit), "cannot acquire RTKit mailbox\n");
	/* Last fallible step: do not retain a supplier for a failed local probe. */
	ret = dcp_m4_pmp_prepare(dev);
	if (ret)
		return dev_err_probe(dev, ret, "retained PMP supplier not ready\n");
	platform_set_drvdata(pdev, dcp);
	pm_runtime_get_noresume(dev);
	dev_info(dev, "Warm DCP present; arm watchdog before writing ready=1\n");
	/* Inert until their own startup attribute is written; never fails this probe. */
	dcp_m4_ext_create_all(dcp);
	return 0;
}

/* Reboot/poweroff: bring any running DCPEXT session to a quiescent stop. */
static void dcp_m4_shutdown(struct platform_device *pdev)
{
	struct dcp_m4_ready *dcp = platform_get_drvdata(pdev);

	if (dcp)
		dcp_m4_ext_shutdown_all(dcp);
}

static const struct of_device_id dcp_m4_match[] = {
	{ .compatible = "apple,t8132-dcp" },
	{ .compatible = "apple,t6030-dcp" },
	{},
};
MODULE_DEVICE_TABLE(of, dcp_m4_match);

static struct platform_driver dcp_m4_driver = {
	.probe = dcp_m4_probe,
	.shutdown = dcp_m4_shutdown,
	.driver = {
		.name = "apple-dcp-m4-ready",
		.of_match_table = dcp_m4_match,
		.dev_groups = dcp_m4_groups,
		.suppress_bind_attrs = true,
	},
};

static int __init dcp_m4_init(void)
{
	return platform_driver_register(&dcp_m4_driver);
}
module_init(dcp_m4_init);
/* Deliberately no module_exit: warm firmware ownership has no safe teardown. */
MODULE_DESCRIPTION("J713 inherited DCP transport and fixed-mode scanout qualification");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_IMPORT_NS("DMA_BUF");
