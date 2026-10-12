// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * J514S (T6030, G15S) GPU firmware stage 1: cold to READY.
 *
 * Cleanroom implementation of the written stage-1 contract: power admission
 * and the PMP AGX vote, RTKit, the running UAT/PPL handoff, the 49-object
 * initdata graph, the initdata doorbell, device controls 0x13 and 9, the two
 * firmware readiness transitions and control 0x13 again. Stage 1 publishes
 * no GPU work itself; after READY the stage-2 backend (gpu-g15-backend.c)
 * builds the compute graph and registers the render node. Started explicitly
 * through the "start" attribute under a watchdog guard; everything
 * firmware-visible is retained until reboot.
 */
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/soc/apple/pmp-display.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <crypto/sha2.h>

#include "gpu-g15-board-data.h"
#include "gpu-g15.h"

static bool enable;
module_param(enable, bool, 0400);
MODULE_PARM_DESC(enable, "Permit the watchdog-guarded J514S GPU firmware start");

/* Retained J514S profile: 174 recorded boots, iBoot-10151.140.19.700.2. */
static const struct { u64 pa, size; } g15_profile[G15_REGIONS] = {
	[G15_TTBS] = { 0x1047ffb8000ULL, 0x4000 },
	[G15_TABLES] = { 0x1047ff78000ULL, 0x40000 },
	[G15_HANDOFF] = { 0x1047ff70000ULL, 0x4000 },
	[G15_SHARED_L2] = { 0x1047ff74000ULL, 0x4000 },
	[G15_TEXT] = { 0x1000090c000ULL, 0x5c000 },
	[G15_DATA] = { 0x10001950000ULL, 0x114000 },
};

static const u8 g15_text_sha256[32] = {
	0x11, 0xe4, 0x9f, 0x75, 0x0b, 0x67, 0x1a, 0x2b, 0x13, 0xd0, 0x92, 0xdb, 0x1c, 0xbc, 0xa3, 0xa8,
	0x77, 0x55, 0x86, 0xd6, 0xb1, 0xe3, 0xaa, 0xe8, 0xa6, 0xa4, 0x64, 0xa7, 0x80, 0x02, 0x93, 0xd5,
};

/* HardwareDataB IOMapping records (contract 4.6). */
static const struct {
	u8 slot;
	u64 pa, map_bytes, va, delta;
} g15_iomaps[] = {
	{ 0, 0x290d00000ULL, 0x144000, 0xfffffc2068000000ULL, 0 },
	{ 1, 0x20e100000ULL, 0x4000, 0xfffffc2068148000ULL, 0x1000 },
	{ 2, 0x351014000ULL, 0x4000, 0xfffffc2068150000ULL, 0 },
	{ 3, 0x290000000ULL, 0x20000, 0xfffffc2068158000ULL, 0 },
	{ 7, 0x3502bc000ULL, 0x4000, 0xfffffc206817c000ULL, 0 },
	{ 9, 0x290e08000ULL, 0x8000, 0xfffffc2068184000ULL, 0 },
	{ 11, 0x220000000ULL, 0x12c000, 0xfffffc2068190000ULL, 0 },
	{ 12, 0x35104c000ULL, 0x4000, 0xfffffc20682c0000ULL, 0 },
	{ 18, 0x3503d0000ULL, 0x4000, 0xfffffc20682c8000ULL, 0 },
	{ 19, 0x3503c0000ULL, 0x4000, 0xfffffc20682d0000ULL, 0 },
	{ 20, 0x3503d8000ULL, 0x4000, 0xfffffc20682d8000ULL, 0 },
	{ 23, 0x293000000ULL, 0x400000, 0xfffffc20682e0000ULL, 0 },
	{ 25, 0x30945c000ULL, 0x4000, 0xfffffc20686e4000ULL, 0 },
	{ 26, 0x350280000ULL, 0x8000, 0xfffffc20686ec000ULL, 0 },
	{ 29, 0x290e5c000ULL, 0x4000, 0xfffffc20686f8000ULL, 0 },
};

/* ---------------------------------------------------------------- ADT --- */

struct g15_adt {
	const u8 *data;
	size_t size;
};

/* Return the end of the node at @off, or 0 on a malformed node. */
static size_t g15_adt_node_end(const struct g15_adt *adt, size_t off, unsigned int depth)
{
	u32 props, children, len;

	if (depth > 32 || off > adt->size || adt->size - off < 8)
		return 0;
	props = get_unaligned_le32(adt->data + off);
	children = get_unaligned_le32(adt->data + off + 4);
	if (props > 4096 || children > 1024)
		return 0;
	off += 8;
	for (u32 i = 0; i < props; i++) {
		if (adt->size - off < 36)
			return 0;
		len = get_unaligned_le32(adt->data + off + 32) & 0x7fffffff;
		off += 36;
		if (len > adt->size - off)
			return 0;
		off += ALIGN(len, 4);
		if (off > adt->size)
			return 0;
	}
	for (u32 i = 0; i < children; i++) {
		off = g15_adt_node_end(adt, off, depth + 1);
		if (!off)
			return 0;
	}
	return off;
}

static const u8 *g15_adt_prop(const struct g15_adt *adt, size_t node, const char *name, u32 *len)
{
	u32 props = get_unaligned_le32(adt->data + node);
	size_t off = node + 8;

	for (u32 i = 0; i < props; i++) {
		u32 l = get_unaligned_le32(adt->data + off + 32) & 0x7fffffff;

		if (!strncmp((const char *)adt->data + off, name, 32)) {
			*len = l;
			return adt->data + off + 36;
		}
		off += 36 + ALIGN(l, 4);
	}
	return NULL;
}

/* Find the child named @name of @node; returns the child offset or 0. */
static size_t g15_adt_child(const struct g15_adt *adt, size_t node, const char *name)
{
	u32 props = get_unaligned_le32(adt->data + node);
	u32 children = get_unaligned_le32(adt->data + node + 4);
	size_t off = node + 8;
	const u8 *n;
	u32 len;

	for (u32 i = 0; i < props; i++)
		off += 36 + ALIGN(get_unaligned_le32(adt->data + off + 32) & 0x7fffffff, 4);
	for (u32 i = 0; i < children; i++) {
		n = g15_adt_prop(adt, off, "name", &len);
		if (n && len && !strncmp((const char *)n, name, len))
			return off;
		off = g15_adt_node_end(adt, off, 1);
		if (!off)
			return 0;
	}
	return 0;
}

static int g15_adt_u64(const struct g15_adt *adt, size_t node, const char *name, u64 *v)
{
	u32 len;
	const u8 *p = g15_adt_prop(adt, node, name, &len);

	if (!p || len != 8)
		return -EINVAL;
	*v = get_unaligned_le64(p);
	return 0;
}

/* Read-only collector: the bootloader's phram "adt" export, parsed privately. */
static int g15_collect_regions(struct g15_gpu *gpu)
{
	static const char * const props[4] = {
		"gpu-region", "gfx-shared-region", "gfx-handoff", "gfx-shared-l2-region",
	};
	struct device_node *rmem, *np;
	struct g15_adt adt = {};
	struct resource res;
	size_t armio, sgx, asc, nub;
	const u8 *ranges, *nub_ranges, *p;
	u32 len, nub_len;
	void *map;
	u8 *copy = NULL;
	int ret = -ENODEV;
	char key[48];

	rmem = of_find_node_by_path("/reserved-memory");
	for_each_child_of_node(rmem, np) {
		const char *label;

		if (of_device_is_compatible(np, "phram") &&
		    !of_property_read_string(np, "label", &label) && !strcmp(label, "adt") &&
		    !of_address_to_resource(np, 0, &res)) {
			of_node_put(np);
			ret = 0;
			break;
		}
	}
	of_node_put(rmem);
	if (ret)
		return ret;
	if (!resource_size(&res) || resource_size(&res) > SZ_2M || (res.start & 0x3fff))
		return -EINVAL;
	map = memremap(res.start, resource_size(&res), MEMREMAP_WB);
	if (!map)
		return -ENOMEM;
	copy = kvmalloc(resource_size(&res), GFP_KERNEL);
	if (copy)
		memcpy(copy, map, resource_size(&res));
	memunmap(map);
	if (!copy)
		return -ENOMEM;
	adt.data = copy;
	adt.size = resource_size(&res);
	ret = -EINVAL;
	if (!g15_adt_node_end(&adt, 0, 0))
		goto out;
	armio = g15_adt_child(&adt, 0, "arm-io");
	sgx = armio ? g15_adt_child(&adt, armio, "sgx") : 0;
	asc = armio ? g15_adt_child(&adt, armio, "gfx-asc") : 0;
	nub = asc ? g15_adt_child(&adt, asc, "iop-gfx-nub") : 0;
	if (!sgx || !asc || !nub)
		goto out;
	p = g15_adt_prop(&adt, sgx, "compatible", &len);
	if (!p || !memchr(p, 0, len) || strcmp((const char *)p, "gpu,t6030"))
		goto out;
	for (unsigned int i = 0; i < 4; i++) {
		u64 base, size;

		snprintf(key, sizeof(key), "%s-base", props[i]);
		if (g15_adt_u64(&adt, sgx, key, &base))
			goto out;
		snprintf(key, sizeof(key), "%s-size", props[i]);
		if (g15_adt_u64(&adt, sgx, key, &size))
			goto out;
		gpu->region[i] = (struct resource)DEFINE_RES_MEM(base, size);
	}
	ranges = g15_adt_prop(&adt, asc, "segment-ranges", &len);
	nub_ranges = g15_adt_prop(&adt, nub, "segment-ranges", &nub_len);
	p = g15_adt_prop(&adt, asc, "segment-names", &nub_len);
	if (!ranges || len != 64 || !nub_ranges || memcmp(ranges, nub_ranges, 64) ||
	    !p || strncmp((const char *)p, "__TEXT;__DATA", nub_len))
		goto out;
	p = g15_adt_prop(&adt, nub, "pre-loaded", &len);
	if (!p || len != 4 || get_unaligned_le32(p) != 1)
		goto out;
	for (unsigned int i = 0; i < 2; i++) {
		const u8 *r = ranges + 32 * i;
		u64 pa = get_unaligned_le64(r), va = get_unaligned_le64(r + 8);

		if (get_unaligned_le64(r + 0x10) != pa ||
		    get_unaligned_le32(r + 0x1c) != (i ? 0 : 1))
			goto out;
		gpu->region[G15_TEXT + i] =
			(struct resource)DEFINE_RES_MEM(pa, get_unaligned_le32(r + 0x18));
		if (i)
			gpu->data_va = va;
		else
			gpu->text_va = va;
	}
	if (gpu->text_va != G15_UPPER || gpu->data_va != G15_UPPER + 0x5c000)
		goto out;
	/* Explicit J514S profile admission plus Linux-RAM exclusion. */
	for (unsigned int i = 0; i < G15_REGIONS; i++) {
		struct resource *r = &gpu->region[i];

		if (r->start != g15_profile[i].pa || resource_size(r) != g15_profile[i].size ||
		    region_intersects(r->start, resource_size(r), IORESOURCE_SYSTEM_RAM,
				      IORES_DESC_NONE) != REGION_DISJOINT) {
			dev_err(gpu->dev, "ADT region %u %pR outside the J514S profile\n", i, r);
			goto out;
		}
	}
	ret = 0;
out:
	kvfree(copy);
	return ret;
}

/* Normalized TEXT identity: GKTS marker, private copy with entropy zeroed. */
static int g15_text_identity(struct g15_gpu *gpu)
{
	static const u8 marker[8] = { 0x47, 0x4b, 0x54, 0x53, 0x08, 0, 0, 0 };
	size_t size = resource_size(&gpu->region[G15_TEXT]);
	u8 digest[SHA256_DIGEST_SIZE];
	void *map;
	u8 *copy;
	int ret = 0;

	map = memremap(gpu->region[G15_TEXT].start, size, MEMREMAP_WB);
	if (!map)
		return -ENOMEM;
	copy = vmalloc(size);
	if (copy)
		memcpy(copy, map, size);
	memunmap(map);
	if (!copy)
		return -ENOMEM;
	if (memcmp(copy + 0x58e88, marker, 8)) {
		ret = -EINVAL;
	} else {
		memset(copy + 0x58e90, 0, 8);
		sha256(copy, size, digest);
		if (memcmp(digest, g15_text_sha256, sizeof(digest)))
			ret = -EINVAL;
	}
	vfree(copy);
	return ret;
}

/* -------------------------------------------------------- board data --- */

static int g15_image(void *dst, size_t size, const struct g15_image_line *lines,
		     size_t count, const u8 *sha)
{
	u8 digest[SHA256_DIGEST_SIZE];

	memset(dst, 0, size);
	for (size_t i = 0; i < count; i++) {
		size_t n = min_t(size_t, 16, size - lines[i].offset);

		if (lines[i].offset >= size)
			return -EINVAL;
		memcpy((u8 *)dst + lines[i].offset, lines[i].data, n);
	}
	sha256(dst, size, digest);
	return memcmp(digest, sha, sizeof(digest)) ? -EINVAL : 0;
}

/* ------------------------------------------------------ host window --- */

static struct g15_page *g15_page_find(struct g15_gpu *gpu, u64 va)
{
	for (unsigned int i = 0; i < gpu->page_count; i++)
		if (gpu->pages[i].va == va)
			return &gpu->pages[i];
	return NULL;
}

static void *g15_l3(struct g15_gpu *gpu, u32 index)
{
	void *cpu;
	dma_addr_t pa;

	for (unsigned int i = 0; i < gpu->l3_count; i++)
		if (gpu->l3[i].index == index)
			return gpu->l3[i].cpu;
	if (gpu->l3_count == G15_MAX_L3)
		return NULL;
	cpu = dma_alloc_coherent(gpu->dev, G15_PAGE, &pa, GFP_KERNEL);
	if (!cpu)
		return NULL;
	gpu->l3[gpu->l3_count++] = (typeof(gpu->l3[0])){ index, cpu, pa };
	/* Host window L2 lives in the shared-l2 reservation (root[2]). */
	writeq(pa | G15_PTE_TABLE, gpu->shared_l2 + index * 8);
	return cpu;
}

int g15_map_page(struct g15_gpu *gpu, u64 va, u64 pa, u64 attr)
{
	u64 off = va - G15_HOST;
	u64 *l3;

	if (va < G15_HOST || off >= G15_HOST_SIZE || (va & (G15_PAGE - 1)))
		return -EINVAL;
	l3 = g15_l3(gpu, off >> 25);
	if (!l3)
		return -ENOMEM;
	if (l3[(off >> 14) & 2047])
		return -EEXIST;
	WRITE_ONCE(l3[(off >> 14) & 2047], pa | attr);
	return 0;
}

/* Clear one host-window leaf; the caller invalidates before reusing the PA. */
int g15_unmap_page(struct g15_gpu *gpu, u64 va)
{
	u64 off = va - G15_HOST;
	u64 *l3 = NULL;

	if (va < G15_HOST || off >= G15_HOST_SIZE || (va & (G15_PAGE - 1)))
		return -EINVAL;
	for (unsigned int i = 0; i < gpu->l3_count; i++)
		if (gpu->l3[i].index == off >> 25)
			l3 = gpu->l3[i].cpu;
	if (!l3 || !l3[(off >> 14) & 2047])
		return -ENOENT;
	WRITE_ONCE(l3[(off >> 14) & 2047], 0);
	return 0;
}

/* Back [va, va + size) with owned zeroed pages of one attribute. */
static int g15_back(struct g15_gpu *gpu, u64 va, u64 size, u64 attr)
{
	u64 end = ALIGN(va + size, G15_PAGE);
	int ret;

	for (u64 page = va & ~(G15_PAGE - 1); page < end; page += G15_PAGE) {
		struct g15_page *p = g15_page_find(gpu, page);

		if (p)
			continue;	/* shared page, same attribute class by layout */
		if (gpu->page_count == G15_MAX_PAGES)
			return -ENOSPC;
		p = &gpu->pages[gpu->page_count];
		p->va = page;
		p->cpu = dma_alloc_coherent(gpu->dev, G15_PAGE, &p->pa, GFP_KERNEL);
		if (!p->cpu)
			return -ENOMEM;
		gpu->page_count++;
		ret = g15_map_page(gpu, page, p->pa, attr);
		if (ret)
			return ret;
	}
	return 0;
}

int g15_write(struct g15_gpu *gpu, u64 va, const void *src, size_t len)
{
	while (len) {
		struct g15_page *p = g15_page_find(gpu, va & ~(G15_PAGE - 1));
		size_t n = min_t(size_t, len, G15_PAGE - (va & (G15_PAGE - 1)));

		if (!p || !p->cpu)
			return -EFAULT;
		memcpy((u8 *)p->cpu + (va & (G15_PAGE - 1)), src, n);
		va += n;
		src = (const u8 *)src + n;
		len -= n;
	}
	return 0;
}

int g15_w32(struct g15_gpu *gpu, u64 va, u32 v)
{
	u8 b[4];

	put_unaligned_le32(v, b);
	return g15_write(gpu, va, b, 4);
}

static int g15_w64(struct g15_gpu *gpu, u64 va, u64 v)
{
	u8 b[8];

	put_unaligned_le64(v, b);
	return g15_write(gpu, va, b, 8);
}

u32 g15_r32(struct g15_gpu *gpu, u64 va)
{
	struct g15_page *p = g15_page_find(gpu, va & ~(G15_PAGE - 1));

	if (!p || !p->cpu || (va & (G15_PAGE - 1)) > G15_PAGE - 4)
		return 0;
	return get_unaligned_le32((u8 *)READ_ONCE(p->cpu) + (va & (G15_PAGE - 1)));
}

/* ------------------------------------------------------------- RTKit --- */

void g15_fail(struct g15_gpu *gpu, int error)
{
	if (!gpu->error)
		gpu->error = error;
}

static int g15_shmem_setup(void *cookie, struct apple_rtkit_shmem *bfr)
{
	struct g15_gpu *gpu = cookie;
	struct resource *data = &gpu->region[G15_DATA];

	/* Only the inherited crash buffer inside this session's fw-data, once. */
	if (gpu->crash || !bfr->iova || !bfr->size || (bfr->iova | bfr->size) & 0xfff ||
	    bfr->iova < data->start || bfr->size > resource_size(data) ||
	    bfr->iova - data->start > resource_size(data) - bfr->size) {
		dev_err(gpu->dev, "refusing RTKit buffer %#llx/%#zx\n", bfr->iova, bfr->size);
		g15_fail(gpu, -EINVAL);
		return -EINVAL;
	}
	bfr->iomem = ioremap_wc(bfr->iova, bfr->size);
	if (!bfr->iomem)
		return -ENOMEM;
	bfr->is_mapped = true;
	gpu->crash = bfr->iomem;
	gpu->crash_pa = bfr->iova;
	gpu->crash_size = bfr->size;
	dev_info(gpu->dev, "mapped firmware crash storage %#llx/%#zx\n", bfr->iova, bfr->size);
	return 0;
}

static void g15_shmem_destroy(void *cookie, struct apple_rtkit_shmem *bfr)
{
	/* Retained until reboot: no qualified stop path in stage 1. */
}

static void g15_crashed(void *cookie, const void *log, size_t size)
{
	struct g15_gpu *gpu = cookie;

	dev_err(gpu->dev, "GPU firmware crashed; retaining session until reboot\n");
	g15_fail(gpu, -EIO);
}

static void g15_recv(void *cookie, u8 endpoint, u64 message)
{
	struct g15_gpu *gpu = cookie;

	/* EP 0x20 0x0042... asks for receive-ring draining; nothing else expected. */
	if (endpoint == 0x20 && message == 0x0042000000000000ULL) {
		if (READ_ONCE(gpu->started))
			queue_work(system_highpri_wq, &gpu->drain);
		return;
	}
	dev_info(gpu->dev, "firmware EP%#x message %#llx\n", endpoint, message);
}

static const struct apple_rtkit_ops g15_rtkit_ops = {
	.crashed = g15_crashed,
	.recv_message = g15_recv,
	.shmem_setup = g15_shmem_setup,
	.shmem_destroy = g15_shmem_destroy,
};

/* Receive rings: consume to the firmware producer (contents are diagnostics). */
static void g15_drain_one(struct g15_gpu *gpu, u64 state)
{
	u32 producer = g15_r32(gpu, state + 0x20);

	if (g15_r32(gpu, state) != producer) {
		g15_w32(gpu, state, producer);
		wmb();
	}
}

/*
 * Event channel (256 x 0x38): u32 type at +0; type 1 carries a u64 stage
 * flag mask at +4 (TA bit0, FS bit1, compute bit2). Types 0 and 4 report
 * firmware errors. Notifications only: retirement has its own receipts.
 */
static void g15_drain_events(struct g15_gpu *gpu)
{
	u32 producer = g15_r32(gpu, G15_EVENT_STATE + 0x20);
	u32 read = g15_r32(gpu, G15_EVENT_STATE);

	u64 now = ktime_get_ns();

	if (producer >= 256 || read >= 256)
		return;
	rmb();
	while (read != producer) {
		u64 rec = G15_EVENT_RING + read * 0x38ULL;
		u32 type = g15_r32(gpu, rec);
		u64 flags = g15_r32(gpu, rec + 4) | (u64)g15_r32(gpu, rec + 8) << 32;

		if (type == 1)
			for (u32 b = 0; b < 3; b++)
				if (flags & BIT(b))
					WRITE_ONCE(gpu->event_ns[b], now);
		if (type == 1 && (flags & BIT(2)))
			atomic_inc(&gpu->compute_events);
		if (type == 1)
			atomic_add(hweight64(flags & 7), &gpu->stage_events);
		if (type == 0 || type == 4) {
			for (u32 j = 0; j < 14; j++)
				gpu->err_raw[j] = g15_r32(gpu, rec + 4 * j);
			gpu->err_type = type;
			/* Type 4: i32 culprit event slot at +0xc, possibly -1. */
			gpu->err_slot = type == 4 ? (s32)gpu->err_raw[3] : -1;
			smp_wmb();
			atomic_inc(&gpu->event_errors);
			dev_err(gpu->dev, "firmware event %u: %08x %08x %08x %08x %08x %08x\n",
				type, g15_r32(gpu, rec + 4), g15_r32(gpu, rec + 8),
				g15_r32(gpu, rec + 12), g15_r32(gpu, rec + 16),
				g15_r32(gpu, rec + 20), g15_r32(gpu, rec + 24));
		} else if (type == 8) {
			/* Channel error (DMKill): needs a DEVCTL 8 acknowledgement. */
			dev_warn(gpu->dev, "firmware event 8: %08x %08x %08x %08x %08x\n",
				 g15_r32(gpu, rec + 4), g15_r32(gpu, rec + 8),
				 g15_r32(gpu, rec + 12), g15_r32(gpu, rec + 16),
				 g15_r32(gpu, rec + 20));
			atomic_inc(&gpu->channel_errors);
		} else {
			dev_dbg(gpu->dev, "firmware event %u flags %#llx\n", type, flags);
		}
		read = (read + 1) % 256;
	}
	mb();
	g15_w32(gpu, G15_EVENT_STATE, producer);
	wmb();
}

/* Diagnostics: the last few event records, raw. */
void g15_dump_events(struct g15_gpu *gpu, u32 count)
{
	u32 producer = g15_r32(gpu, G15_EVENT_STATE + 0x20) % 256;

	for (u32 i = min(count, 256U); i; i--) {
		u64 rec = G15_EVENT_RING + ((producer + 256 - i) % 256) * 0x38ULL;
		u32 w[14];

		for (u32 j = 0; j < 14; j++)
			w[j] = g15_r32(gpu, rec + 4 * j);
		dev_info(gpu->dev, "event[%u]: %*ph\n", (producer + 256 - i) % 256,
			 (int)sizeof(w), w);
	}
}

void g15_drain_rings(struct g15_gpu *gpu)
{
	g15_drain_events(gpu);
	for (unsigned int i = 0; i < 6; i++)
		g15_drain_one(gpu, G15_LOG_STATE + i * 0x30);
	g15_drain_one(gpu, G15_TRACE_STATE);
	g15_drain_one(gpu, G15_STATS_STATE);
}

static void g15_drain_work(struct work_struct *work)
{
	struct g15_gpu *gpu = container_of(work, struct g15_gpu, drain);

	mutex_lock(&gpu->lock);
	g15_drain_rings(gpu);
	mutex_unlock(&gpu->lock);
	if (READ_ONCE(gpu->be))
		g15_backend_kick(gpu);
}

/* ----------------------------------------------------- UAT / PPL lock --- */

int g15_ppl_lock(struct g15_gpu *gpu)
{
	void __iomem *h = gpu->handoff;
	unsigned long deadline = jiffies + msecs_to_jiffies(100);

	writeb(1, h + 0x10);
	wmb();
	while (readb(h + 0x11)) {
		if (readl(h + 0x14)) {
			writeb(0, h + 0x10);
			wmb();
			while (readl(h + 0x14))
				if (time_after(jiffies, deadline))
					return -ETIMEDOUT;
			writeb(1, h + 0x10);
			wmb();
		}
		if (time_after(jiffies, deadline)) {
			writeb(0, h + 0x10);
			return -ETIMEDOUT;
		}
		cpu_relax();
	}
	rmb();
	return 0;
}

void g15_ppl_unlock(struct g15_gpu *gpu)
{
	wmb();
	writel(1, gpu->handoff + 0x14);
	writeb(0, gpu->handoff + 0x10);
	wmb();
}

void g15_tlbi_all(void)
{
	/* TLBI VMALLE1OS (SYS #0, C8, C1, #0): the GPU UAT honours broadcast TLBI. */
	asm volatile("dsb ishst\n\tsys #0, c8, c1, #0\n\tdsb sy\n\tisb" ::: "memory");
}

static int g15_uat_bootstrap(struct g15_gpu *gpu)
{
	void __iomem *h = gpu->handoff;
	int ret;
	u64 magic;

	/* AP side of the PPL exchange; never overwrite firmware interest/magic. */
	writeq(G15_PPL_MAGIC, h + 0x00);
	writel(0xffffffff, h + 0x18);
	for (unsigned int i = 0; i <= 64; i++)
		for (unsigned int w = 0; w < 3; w++)
			writeq(0, h + 0x20 + 24 * i + 8 * w);
	writeq(0, h + 0x640);
	wmb();
	ret = readq_poll_timeout(h + 0x08, magic, magic == G15_PPL_MAGIC, 1000, 1000000);
	if (ret) {
		dev_err(gpu->dev, "PPL firmware magic missing: %#llx\n", magic);
		return ret;
	}
	gpu->lower = dma_alloc_coherent(gpu->dev, G15_PAGE, &gpu->lower_pa, GFP_KERNEL);
	if (!gpu->lower)
		return -ENOMEM;
	ret = g15_ppl_lock(gpu);
	if (ret)
		return ret;
	/* Exclusive startup epoch: host entries 2.. of the upper root go first. */
	for (unsigned int i = 2; i < 64; i++)
		writeq(0, gpu->tables + i * 8);
	for (unsigned int i = 0; i < 2048; i++)
		writeq(0, gpu->shared_l2 + i * 8);
	g15_tlbi_all();
	writeq(gpu->region[G15_SHARED_L2].start | G15_PTE_TABLE, gpu->tables + 2 * 8);
	/* TTBAT context0: owned lower root, upper root = pagetables base. */
	for (unsigned int i = 2; i < 128; i++)
		writeq(0, gpu->ttbs + i * 8);
	writeq(gpu->lower_pa | 1, gpu->ttbs + 0);
	writeq(gpu->region[G15_TABLES].start | 1, gpu->ttbs + 8);
	g15_ppl_unlock(gpu);
	dev_info(gpu->dev, "MMU: initialized, root[0..2]=%#llx %#llx %#llx\n",
		 readq(gpu->tables), readq(gpu->tables + 8), readq(gpu->tables + 16));
	return 0;
}

/* --------------------------------------------------------- initdata --- */

static int g15_build(struct g15_gpu *gpu)
{
	u8 *hw = NULL, *glob = NULL, *power = NULL;
	int ret;

	/* Owned objects and channels (contract 4.1). */
	ret = g15_back(gpu, G15_ROOT, 0xc0, G15_PTE_FW) ?:
	      g15_back(gpu, G15_REGION_A, 0x4000, G15_PTE_FW) ?:
	      g15_back(gpu, G15_RUNTIME, 0x4b3, G15_PTE_FW) ?:
	      g15_back(gpu, G15_HWDATA, G15_HARDWARE_SIZE, G15_PTE_FW) ?:
	      g15_back(gpu, G15_PAIR, 0x100, G15_PTE_FW) ?:
	      g15_back(gpu, G15_SMALL, 0x10, G15_PTE_FW) ?:
	      g15_back(gpu, G15_C0, 0x1000, G15_PTE_SHARED) ?:
	      g15_back(gpu, G15_C1, 0x4000, G15_PTE_SHARED) ?:
	      g15_back(gpu, G15_C3, 0x4000, G15_PTE_SHARED) ?:
	      g15_back(gpu, G15_GLOBALS, G15_GLOBALS_SIZE, G15_PTE_FW) ?:
	      g15_back(gpu, G15_POWER, G15_POWER_SIZE, G15_PTE_FW) ?:
	      g15_back(gpu, G15_CONTROL, 0xc3d0, G15_PTE_FW) ?:
	      g15_back(gpu, G15_FLAGS, 0x40, G15_PTE_FW);
	for (unsigned int i = 0; !ret && i < 12; i++)
		ret = g15_back(gpu, G15_CH_STATE(i), 0x30, G15_PTE_FW) ?:
		      g15_back(gpu, G15_CH_RING(i), 0x1800, G15_PTE_FW);
	if (!ret)
		ret = g15_back(gpu, G15_DEVCTL_STATE, 0x30, G15_PTE_FW) ?:
		      g15_back(gpu, G15_DEVCTL_RING, 0x3800, G15_PTE_FW) ?:
		      g15_back(gpu, G15_EVENT_STATE, 0x30, G15_PTE_FW) ?:
		      g15_back(gpu, G15_EVENT_RING, 0x3800, G15_PTE_FW) ?:
		      g15_back(gpu, G15_LOG_STATE, 0x120, G15_PTE_FW) ?:
		      g15_back(gpu, G15_LOG_RING, 0x51000, G15_PTE_FW) ?:
		      g15_back(gpu, G15_TRACE_STATE, 0x30, G15_PTE_FW) ?:
		      g15_back(gpu, G15_TRACE_RING, 0x7000, G15_PTE_FW) ?:
		      g15_back(gpu, G15_STATS_STATE, 0x30, G15_PTE_FW) ?:
		      g15_back(gpu, G15_STATS_RING, 0x4000, G15_PTE_FW) ?:
		      g15_back(gpu, G15_FWCTL_STATE, 0x30, G15_PTE_FW) ?:
		      g15_back(gpu, G15_FWCTL_RING, 0x1400, G15_PTE_FW);
	/* Firmware Device RW aliases for the IOMapping table (contract 4.6). */
	for (unsigned int i = 0; !ret && i < ARRAY_SIZE(g15_iomaps); i++)
		for (u64 off = 0; !ret && off < g15_iomaps[i].map_bytes; off += G15_PAGE)
			ret = g15_map_page(gpu, g15_iomaps[i].va + off, g15_iomaps[i].pa + off,
					   G15_PTE_MMIO);
	if (ret)
		return ret;

	/* Board images, verified, then the owned replacements (contract 4.5). */
	hw = kvmalloc(G15_HARDWARE_SIZE, GFP_KERNEL);
	glob = kvmalloc(G15_GLOBALS_SIZE, GFP_KERNEL);
	power = kvmalloc(G15_POWER_SIZE, GFP_KERNEL);
	ret = -ENOMEM;
	if (!hw || !glob || !power)
		goto out;
	ret = g15_image(hw, G15_HARDWARE_SIZE, g15_hardware_lines,
			ARRAY_SIZE(g15_hardware_lines), g15_hardware_sha256) ?:
	      g15_image(glob, G15_GLOBALS_SIZE, g15_globals_lines,
			ARRAY_SIZE(g15_globals_lines), g15_globals_sha256) ?:
	      g15_image(power, G15_POWER_SIZE, g15_power_lines,
			ARRAY_SIZE(g15_power_lines), g15_power_sha256);
	if (ret) {
		dev_err(gpu->dev, "board data image hash mismatch\n");
		goto out;
	}
	ret = -EINVAL;
	if (get_unaligned_le32(hw + 0xa38) != 1 || get_unaligned_le32(hw + 0xb58) != 8 ||
	    get_unaligned_le32(glob + 0x980) != 10)
		goto out;
	put_unaligned_le64(gpu->region[G15_TTBS].start, hw + 0xb44);
	for (unsigned int i = 0; i < ARRAY_SIZE(g15_iomaps); i++)
		put_unaligned_le64(g15_iomaps[i].va + g15_iomaps[i].delta,
				   hw + 0x640 + 32 * g15_iomaps[i].slot + 8);
	put_unaligned_le64(G15_TIMESTAMPS, hw + 0x28);
	put_unaligned_le32(100, glob + 0x980);
	/* HardwareDataA at +0x4580: maintained S8 request. */
	put_unaligned_le32(8, hw + 0x4580 + 0x2c);
	put_unaligned_le32(8, hw + 0x4580 + 0x30);
	put_unaligned_le32(800, hw + 0x4580 + 0x44);
	put_unaligned_le32(800, hw + 0x4580 + 0x7d8);
	put_unaligned_le32(800, hw + 0x4580 + 0x7e0);
	put_unaligned_le32(800, hw + 0x4580 + 0x81c);
	ret = g15_write(gpu, G15_HWDATA, hw, G15_HARDWARE_SIZE) ?:
	      g15_write(gpu, G15_GLOBALS, glob, G15_GLOBALS_SIZE) ?:
	      g15_write(gpu, G15_POWER, power, G15_POWER_SIZE);
	if (ret)
		goto out;

	/* Root (contract 4.2). */
	ret = g15_w64(gpu, G15_ROOT + 0x00, 0x0c08e21e83800490ULL) ?:
	      g15_w64(gpu, G15_ROOT + 0x08, G15_REGION_A) ?:
	      g15_w64(gpu, G15_ROOT + 0x18, G15_RUNTIME) ?:
	      g15_w64(gpu, G15_ROOT + 0x20, G15_GLOBALS) ?:
	      g15_w32(gpu, G15_ROOT + 0x2c, 1);
	if (!ret) {
		static const u8 shifts[3] = { 36, 25, 14 };
		static const u16 entries[3] = { 64, 2048, 2048 };
		u8 b[4] = { 0x00, 0x40, 14, 3 };

		put_unaligned_le16(0x4000, b);
		ret = g15_write(gpu, G15_ROOT + 0x30, b, 4);
		for (unsigned int i = 0; !ret && i < 3; i++) {
			u64 at = G15_ROOT + 0x34 + 0x20 * i;
			u8 h[8] = { 8, 14, 14, shifts[i] };

			put_unaligned_le16(entries[i], h + 4);
			put_unaligned_le16(0x4000, h + 6);
			ret = g15_write(gpu, at, h, 8) ?:
			      g15_w64(gpu, at + 8, 1) ?:
			      g15_w64(gpu, at + 0x10, 0x3ffffffc000ULL) ?:
			      g15_w64(gpu, at + 0x18, (u64)(entries[i] - 1) << shifts[i]);
		}
	}
	if (!ret)
		ret = g15_w64(gpu, G15_ROOT + 0xa8, G15_FLAGS) ?:
		      g15_w64(gpu, G15_ROOT + 0xb0, G15_CONTROL) ?:
		      g15_w64(gpu, G15_ROOT + 0xb8, G15_POWER);
	if (ret)
		goto out;

	/* Runtime pointers (contract 4.3). */
	ret = g15_w64(gpu, G15_RUNTIME, G15_HWDATA) ?:
	      g15_w64(gpu, G15_RUNTIME + 0x08, G15_PAIR) ?:
	      g15_w64(gpu, G15_RUNTIME + 0x10, G15_PAIR);
	for (unsigned int i = 0; !ret && i < 13; i++) {
		u64 state = i < 12 ? G15_CH_STATE(i) : G15_DEVCTL_STATE;
		u64 ring = i < 12 ? G15_CH_RING(i) : G15_DEVCTL_RING;
		u64 at = G15_RUNTIME + 0x18 + 0x20 * i;

		ret = g15_w64(gpu, at, state) ?: g15_w64(gpu, at + 8, state + 0x10) ?:
		      g15_w64(gpu, at + 0x10, state + 0x20) ?: g15_w64(gpu, at + 0x18, ring);
	}
	if (!ret) {
		static const u64 rx[4][2] = {
			{ G15_EVENT_STATE, G15_EVENT_RING }, { G15_LOG_STATE, G15_LOG_RING },
			{ G15_TRACE_STATE, G15_TRACE_RING }, { G15_STATS_STATE, G15_STATS_RING },
		};

		for (unsigned int i = 0; !ret && i < 4; i++)
			ret = g15_w64(gpu, G15_RUNTIME + 0x1b8 + 0x10 * i, rx[i][0]) ?:
			      g15_w64(gpu, G15_RUNTIME + 0x1c0 + 0x10 * i, rx[i][1]);
	}
	if (!ret)
		ret = g15_w64(gpu, G15_RUNTIME + 0x1f8, G15_SMALL) ?:
		      g15_w64(gpu, G15_RUNTIME + 0x234, G15_HWDATA + 0x1880) ?:
		      g15_w64(gpu, G15_RUNTIME + 0x23c, G15_HWDATA + 0x24c0) ?:
		      g15_w64(gpu, G15_RUNTIME + 0x244, G15_HWDATA + 0x3740) ?:
		      g15_w64(gpu, G15_RUNTIME + 0x24c, G15_HWDATA + 0x8900) ?:
		      g15_w64(gpu, G15_RUNTIME + 0x2a8, G15_C0) ?:
		      g15_w64(gpu, G15_RUNTIME + 0x2b0, G15_C1) ?:
		      g15_w64(gpu, G15_RUNTIME + 0x2b8, G15_C1) ?:
		      g15_w64(gpu, G15_RUNTIME + 0x2c0, G15_C3) ?:
		      g15_w64(gpu, G15_RUNTIME + 0x2c8, G15_C3) ?:
		      g15_w32(gpu, G15_RUNTIME + 0x2d0, 4) ?:
		      g15_w32(gpu, G15_RUNTIME + 0x3b0, 0xff) ?:
		      g15_w64(gpu, G15_RUNTIME + 0x441, G15_HWDATA + 0x4580);
	/* Control and flags (contract 4.4). */
	if (!ret)
		ret = g15_w32(gpu, G15_CONTROL + 4, 1) ?:
		      g15_w64(gpu, G15_CONTROL + 0x4568, G15_FWCTL_STATE) ?:
		      g15_w64(gpu, G15_CONTROL + 0x4570, G15_FWCTL_RING) ?:
		      g15_w32(gpu, G15_CONTROL + 0x45c4, 1);
out:
	kvfree(power);
	kvfree(glob);
	kvfree(hw);
	return ret;
}

/* One device-control record, then its producer/completion receipt. */
/*
 * Post one 0x38-byte device-control record (EP0x21 0x0083000000000011) and
 * wait for its consumption and completion. Callers serialize (gpu->lock once
 * the backend runs).
 */
int g15_devctl_record(struct g15_gpu *gpu, const u8 *record)
{
	u32 producer = gpu->devctl_producer, next = (producer + 1) % 256;
	unsigned long deadline = jiffies + msecs_to_jiffies(2000);
	u32 opcode = get_unaligned_le32(record);
	int ret;

	if (g15_r32(gpu, G15_DEVCTL_STATE) != producer)
		return -EPROTO;
	ret = g15_write(gpu, G15_DEVCTL_RING + producer * 0x38, record, 0x38);
	if (ret)
		return ret;
	wmb();
	g15_w32(gpu, G15_DEVCTL_STATE + 0x20, next);
	wmb();
	gpu->devctl_producer = next;
	ret = apple_rtkit_send_message(gpu->rtkit, 0x21, 0x0083000000000011ULL, NULL, false);
	if (ret)
		return ret;
	for (;;) {
		g15_drain_rings(gpu);
		if (gpu->error)
			return gpu->error;
		if (g15_r32(gpu, G15_DEVCTL_STATE) == next &&
		    g15_r32(gpu, G15_DEVCTL_STATE + 0x10) == next)
			return 0;
		if (time_after(jiffies, deadline)) {
			dev_err(gpu->dev, "control %#x: read=%u completion=%u producer=%u\n", opcode,
				g15_r32(gpu, G15_DEVCTL_STATE),
				g15_r32(gpu, G15_DEVCTL_STATE + 0x10), next);
			return -ETIMEDOUT;
		}
		usleep_range(1000, 2000);
	}
}

static int g15_devctl(struct g15_gpu *gpu, u32 opcode)
{
	u8 record[0x38] = {};

	put_unaligned_le32(opcode, record);
	return g15_devctl_record(gpu, record);
}

static int g15_start(struct g15_gpu *gpu)
{
	unsigned long deadline;
	int ret;

	ret = apple_pmp_j514s_gpu_get(gpu->dev);
	if (ret) {
		dev_err(gpu->dev, "PMP AGX vote failed: %d\n", ret);
		return ret;
	}
	gpu->voted = true;
	gpu->version = readl(gpu->sgx_id);
	gpu->counts = readl(gpu->sgx_id + 0x10);
	gpu->cores = readl(gpu->sgx_core + 0x1500);
	dev_info(gpu->dev, "M3 G15S: power acknowledged, version=%#x counts=%#x core-mask=%#x\n",
		 gpu->version, gpu->counts, gpu->cores);
	if (gpu->version != 0x07031100 || gpu->counts != 0x00110209 || gpu->cores != 0x6f5fc)
		return -EPROTO;
	writel(0x70001, gpu->sgx_open);
	/* Cold only: ASC RUN clear and an all-zero TTBAT before ownership. */
	gpu->control = readl(gpu->asc + 0x44);
	if (gpu->control & BIT(4))
		return -EBUSY;
	for (unsigned int i = 0; i < 128; i++)
		if (readq(gpu->ttbs + i * 8))
			return -EBUSY;
	gpu->rtkit = devm_apple_rtkit_init(gpu->dev, gpu, "mbox", 0, &g15_rtkit_ops);
	if (IS_ERR(gpu->rtkit))
		return PTR_ERR(gpu->rtkit);
	WRITE_ONCE(gpu->started, true);
	writel(gpu->control | BIT(4), gpu->asc + 0x44);
	ret = apple_rtkit_wake(gpu->rtkit);
	if (ret)
		return ret;
	if (!apple_rtkit_has_endpoint(gpu->rtkit, 0x20) ||
	    !apple_rtkit_has_endpoint(gpu->rtkit, 0x21))
		return -ENODEV;
	ret = apple_rtkit_start_ep(gpu->rtkit, 0x20) ?: apple_rtkit_start_ep(gpu->rtkit, 0x21);
	if (ret)
		return ret;
	ret = g15_uat_bootstrap(gpu);
	if (ret)
		return ret;
	ret = g15_build(gpu);
	if (ret)
		return ret;
	g15_tlbi_all();
	dev_info(gpu->dev, "M3: publishing owned initdata root %#llx (%u pages, %u L3)\n",
		 G15_ROOT, gpu->page_count, gpu->l3_count);
	ret = apple_rtkit_send_message(gpu->rtkit, 0x20,
				       0x0081000000000000ULL | (G15_ROOT & 0xfffffffffffULL),
				       NULL, false);
	ret = ret ?: g15_devctl(gpu, 0x13) ?: g15_devctl(gpu, 9);
	if (ret)
		return ret;
	deadline = jiffies + msecs_to_jiffies(2000);
	while (g15_r32(gpu, G15_RUNTIME + 0x314) != 0xabcdabcd ||
	       g15_r32(gpu, G15_FLAGS + 4) != 1) {
		g15_drain_rings(gpu);
		if (gpu->error)
			return gpu->error;
		if (time_after(jiffies, deadline)) {
			dev_err(gpu->dev, "readiness: runtime+0x314=%#x flags+4=%#x\n",
				g15_r32(gpu, G15_RUNTIME + 0x314), g15_r32(gpu, G15_FLAGS + 4));
			return -ETIMEDOUT;
		}
		usleep_range(1000, 2000);
	}
	dev_info(gpu->dev, "M3 firmware readiness: magic=%#x flags=%u\n",
		 g15_r32(gpu, G15_RUNTIME + 0x314), g15_r32(gpu, G15_FLAGS + 4));
	ret = g15_devctl(gpu, 0x13);
	if (ret)
		return ret;
	g15_drain_rings(gpu);
	gpu->ready = true;
	dev_info(gpu->dev, "M3: firmware accepted owned initdata and device controls\n");
	return 0;
}

/* ------------------------------------------------------------ sysfs --- */

static ssize_t state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct g15_gpu *gpu = dev_get_drvdata(dev);

	return sysfs_emit(buf, "error=%d started=%u voted=%u ready=%u version=%#x counts=%#x cores=%#x asc=%#x pstate=%u\n",
		gpu->error, gpu->started, gpu->voted, gpu->ready, gpu->version, gpu->counts,
		gpu->cores, readl(gpu->asc + 0x44),
		gpu->voted ? readl(gpu->sgx_core + 0x1000) & 0xf : 0);
}
static DEVICE_ATTR_RO(state);

static ssize_t start_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct g15_gpu *gpu = dev_get_drvdata(dev);
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;
	mutex_lock(&gpu->lock);
	ret = gpu->error ? gpu->error : gpu->started ? -EBUSY : 0;
	if (!ret) {
		ret = g15_start(gpu);
		if (ret)
			g15_fail(gpu, ret);
	}
	mutex_unlock(&gpu->lock);
	if (ret)
		dev_err(dev, "G15 start failed: %d; session retained until reboot\n", ret);
	/* Stage 2: compute graph and the public render node, outside the lock. */
	if (!ret) {
		ret = g15_backend_start(gpu);
		if (ret)
			dev_err(dev, "G15 compute backend failed: %d\n", ret);
	}
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(start);

static struct attribute *g15_attrs[] = { &dev_attr_state.attr, &dev_attr_start.attr, NULL };
ATTRIBUTE_GROUPS(g15);

/* ------------------------------------------------------------ probe --- */

static void __iomem *g15_sgx_window(struct platform_device *pdev, u64 offset)
{
	struct resource *sgx = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sgx");
	struct resource r;

	if (!sgx)
		return IOMEM_ERR_PTR(-EINVAL);
	r = *sgx;
	r.start += offset;
	r.end = r.start + 0x3fff;
	return devm_ioremap_resource(&pdev->dev, &r);
}

static int g15_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct g15_gpu *gpu;
	int ret;

	gpu = devm_kzalloc(dev, sizeof(*gpu), GFP_KERNEL);
	if (!gpu)
		return -ENOMEM;
	gpu->dev = dev;
	mutex_init(&gpu->lock);
	INIT_WORK(&gpu->drain, g15_drain_work);
	platform_set_drvdata(pdev, gpu);
	gpu->pages = devm_kcalloc(dev, G15_MAX_PAGES, sizeof(*gpu->pages), GFP_KERNEL);
	if (!gpu->pages)
		return -ENOMEM;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(42));
	if (ret)
		return ret;
	/* Retain the PMP supplier (and its AGX bridge) before any vote. */
	ret = apple_pmp_j514s_gpu_prepare(dev);
	if (ret)
		return dev_err_probe(dev, ret, "PMP supplier not ready\n");
	ret = g15_collect_regions(gpu);
	if (ret)
		return dev_err_probe(dev, ret, "ADT GPU regions not admitted\n");
	ret = g15_text_identity(gpu);
	if (ret)
		return dev_err_probe(dev, ret, "loaded GFX TEXT identity mismatch\n");
	dev_info(dev, "M3: firmware identity accepted\n");
	gpu->asc = devm_platform_ioremap_resource_byname(pdev, "asc");
	if (IS_ERR(gpu->asc))
		return PTR_ERR(gpu->asc);
	gpu->sgx_id = g15_sgx_window(pdev, 0xd04000);
	gpu->sgx_open = g15_sgx_window(pdev, 0xd14000);
	gpu->sgx_core = g15_sgx_window(pdev, 0xe00000);
	/* Engine busy (+0xc020/+0xc120) and banked fault status (+0xd800/+0xd8c0). */
	gpu->sgx_status = g15_sgx_window(pdev, 0xc000);
	if (IS_ERR(gpu->sgx_id) || IS_ERR(gpu->sgx_open) || IS_ERR(gpu->sgx_core) ||
	    IS_ERR(gpu->sgx_status))
		return -EBUSY;
	gpu->ttbs = devm_ioremap_wc(dev, gpu->region[G15_TTBS].start, 0x4000);
	gpu->tables = devm_ioremap_wc(dev, gpu->region[G15_TABLES].start, 0x40000);
	gpu->handoff = devm_ioremap_wc(dev, gpu->region[G15_HANDOFF].start, 0x4000);
	gpu->shared_l2 = devm_ioremap_wc(dev, gpu->region[G15_SHARED_L2].start, 0x4000);
	if (!gpu->ttbs || !gpu->tables || !gpu->handoff || !gpu->shared_l2)
		return -ENOMEM;
	/* Keep the GFX domain powered for the whole (retained) session. */
	pm_runtime_get_noresume(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	__module_get(THIS_MODULE);
	dev_info(dev, "J514S G15S admitted, ASC=%#x; write start=1 under a watchdog\n",
		 readl(gpu->asc + 0x44));
	return 0;
}

static const struct of_device_id g15_match[] = {
	{ .compatible = "apple,agx-t6030" },
	{},
};
MODULE_DEVICE_TABLE(of, g15_match);

static struct platform_driver g15_driver = {
	.probe = g15_probe,
	.driver = {
		.name = "apple-gpu-g15",
		.of_match_table = g15_match,
		.dev_groups = g15_groups,
		.suppress_bind_attrs = true,
	},
};

static int __init g15_init(void)
{
	if (!enable || !of_machine_is_compatible("apple,j514s") ||
	    !of_machine_is_compatible("apple,t6030"))
		return -ENODEV;
	return platform_driver_register(&g15_driver);
}
module_init(g15_init);
MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("J514S G15S GPU firmware stage 1 (cleanroom)");
