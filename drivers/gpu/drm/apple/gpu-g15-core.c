// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * T6030 (M3 Pro: J514S, J516S) G15S GPU firmware stage 1: cold to READY.
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
#include <linux/bitops.h>
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
MODULE_PARM_DESC(enable, "Permit the watchdog-guarded T6030 GPU firmware start");

/*
 * Firmware predictive-idle inputs (Globals): the standby time in us (+0x7a8)
 * and the model's minimum confidence (+0x7c4, f32). The board profile's
 * 700 us / 0.7 power the GPU down between works that are ~100 us apart once
 * the model has seen a sustained low-utilization stream, and within 1 ms of
 * idle, costing ~300 us at the next pickup. Imported at initdata; a write
 * after startup is re-imported (DEVCTL 0x13).
 */
static struct g15_gpu *g15_idle_gpu;	/* the started GPU, for runtime writes */

/*
 * HardwareData's first two pages hold HardwareDataB, the tiler/fragment/
 * compute activity records (+0x1880/+0x24c0/+0x3740) that every Start and
 * Finalize of the matching engine reads and updates, and the head of
 * HardwareDataA. After initdata only the firmware touches them: the host
 * writes them before the firmware starts and never reads them again, and the
 * GPU has no access (firmware-only class). Map those pages with firmware
 * AttrIndex 0 (inner+outer write-back for the ASC) instead of 2 (inner
 * non-cacheable); the last page, with the fault information (+0x8900) and the
 * HardwareDataA timestamps the host may read, keeps AttrIndex 2.
 */
#define G15_HWDATA_CACHED	(ALIGN(G15_HWDATA + 1, G15_PAGE) + G15_PAGE - G15_HWDATA)

static bool hwdata_cached = true;
module_param(hwdata_cached, bool, 0444);
MODULE_PARM_DESC(hwdata_cached, "Map the HardwareData pages holding HardwareDataB and the engine activity records ASC-cached (firmware AttrIndex 0); 0: AttrIndex 2");
static unsigned int idle_standby_us = 2500;
static unsigned int idle_min_conf = 500;	/* permille */
/*
 * Idle decision branch (round-10 contract D2): Globals +0x2c enables the
 * firmware's idle decision and a nonzero +0x30 selects its predictive
 * branch, which may power the GPU down immediately after any work. Under
 * sustained low-duty submission (tens of us between works) its learned
 * history turns that into a ~300-us power-up before 2-3% of the works,
 * whatever the minimum confidence. 0 keeps the ordinary finite delay:
 * the GPU powers down once it has been idle for the +0x9a0 delay (2,
 * about 2 ms measured).
 */
static unsigned int idle_predictive;

static u32 g15_permille_f32(u32 p)
{
	u32 k = 0;

	p = clamp(p, 1U, 1000U);
	while (p * (1U << k) < 500)	/* p/1000 * 2^k in [0.5, 1) */
		k++;
	if (p == 1000)
		return 0x3f800000;
	/* p/1000 = m/2^24 * 2^(1-k-1), m in [2^23, 2^24) */
	return ((126 - k) << 23) |
	       ((u32)div_u64((u64)p << (24 + k), 1000) - (1U << 23));
}

static int g15_idle_set(const char *val, const struct kernel_param *kp)
{
	struct g15_gpu *gpu = READ_ONCE(g15_idle_gpu);
	u8 rec[0x38] = {};
	int ret = param_set_uint(val, kp);

	if (ret || !gpu)
		return ret;
	put_unaligned_le32(0x13, rec);
	mutex_lock(&gpu->lock);
	ret = g15_w32(gpu, G15_GLOBALS + 0x7a8, clamp(idle_standby_us, 100U, 100000U)) ?:
	      g15_w32(gpu, G15_GLOBALS + 0x7c4, g15_permille_f32(idle_min_conf)) ?:
	      g15_w32(gpu, G15_GLOBALS + 0x30, !!idle_predictive);
	wmb();
	if (!ret)
		ret = g15_devctl_record(gpu, rec);
	mutex_unlock(&gpu->lock);
	dev_info(gpu->dev, "idle policy: %s, standby %u us, min confidence %u/1000 re-imported: %d\n",
		 idle_predictive ? "predictive" : "finite delay", idle_standby_us, idle_min_conf, ret);
	return ret;
}

static const struct kernel_param_ops g15_idle_ops = {
	.set = g15_idle_set,
	.get = param_get_uint,
};
module_param_cb(idle_standby_us, &g15_idle_ops, &idle_standby_us, 0644);
MODULE_PARM_DESC(idle_standby_us, "Firmware predictive-idle standby time, 100..100000 us (Globals +0x7a8; board profile 700, native ADT 1500): the GPU stays up about this long after its last work");
module_param_cb(idle_min_conf, &g15_idle_ops, &idle_min_conf, 0644);
MODULE_PARM_DESC(idle_min_conf, "Firmware predictive-idle minimum confidence, permille (Globals +0x7c4; board profile 700)");
module_param_cb(idle_predictive, &g15_idle_ops, &idle_predictive, 0644);
MODULE_PARM_DESC(idle_predictive, "Firmware idle decision (Globals +0x30): 1 predictive (board profile; may power down right after any work), 0 (default) power down after the ordinary finite delay");

/*
 * GPU firmware builds whose ABI this driver implements. The canonical object
 * VAs (gpu-g15.h), the initdata graph, the board-image templates and the
 * stage-2 work images all belong to one build, so the build is identified
 * before anything is written: by its loaded TEXT (normalized: the 8 entropy
 * bytes after the GKTS marker zeroed) and its segment sizes. An unknown build
 * is refused with its measured identity, to be qualified and added here.
 */
struct g15_fw_build {
	const char *name;
	u64 text_size, data_size;
	u32 marker;		/* GKTS marker offset in TEXT */
	u8 text_sha256[32];
};

static const struct g15_fw_build g15_fw_builds[] = {
	{
		.name = "macOS 14.7-14.8.x stub firmware (iBoot-10151.140.19.700.2)",
		.text_size = 0x5c000, .data_size = 0x114000, .marker = 0x58e88,
		.text_sha256 = {
		0x11, 0xe4, 0x9f, 0x75, 0x0b, 0x67, 0x1a, 0x2b, 0x13, 0xd0, 0x92, 0xdb, 0x1c, 0xbc, 0xa3, 0xa8,
		0x77, 0x55, 0x86, 0xd6, 0xb1, 0xe3, 0xaa, 0xe8, 0xa6, 0xa4, 0x64, 0xa7, 0x80, 0x02, 0x93, 0xd5,
		},
	},
};

/*
 * UAT handoff region sizes of that firmware ABI (TTBAT page, TTBAT tables,
 * PPL handoff page, shared L2 page); the addresses come from the ADT.
 */
static const u64 g15_region_size[G15_TEXT] = {
	[G15_TTBS] = 0x4000, [G15_TABLES] = 0x40000,
	[G15_HANDOFF] = 0x4000, [G15_SHARED_L2] = 0x4000,
};

/*
 * G15S hardware this ABI is qualified on: the SGX identity and layout words.
 * The enabled-core mask is per machine (binning) and is not part of this.
 */
static const struct {
	u32 version, counts;
	const char *name;
} g15_hw_ids[] = {
	{ 0x07031100, 0x00110209, "T6030 G15S rev 0x11" },
};

/* T6030 G15S: two clusters of ten logical cores. */
#define G15_CLUSTER_BITS	10
#define G15_CLUSTER_MASK	GENMASK(G15_CLUSTER_BITS - 1, 0)

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

static int g15_adt_u32(const struct g15_adt *adt, size_t node, const char *name, u32 *v)
{
	u32 len;
	const u8 *p = g15_adt_prop(adt, node, name, &len);

	if (!p || len != 4)
		return -EINVAL;
	*v = get_unaligned_le32(p);
	return 0;
}

/*
 * This machine's GPU performance states (per-chip binning): ADT sgx
 * perf-states and perf-states-sram, (frequency Hz, millivolt) pairs. The
 * board-image layout implements one table shape; another shape is refused.
 * Values the qualified layout takes as given are compared, not trusted.
 */
static int g15_collect_perf(struct g15_gpu *gpu, const struct g15_adt *adt, size_t sgx)
{
	static const struct { const char *name; u32 value; } fixed[] = {
		{ "perf-state-count", G15_ADT_PERF_STATES },
		{ "gpu-num-perf-states", G15_ADT_PERF_STATES - 1 },
		{ "perf-state-table-count", 1 },
		{ "gpu-power-sample-period", 8 },
		{ "gpu-pwr-sample-period-aic-clks", 200000 },
	};
	const u8 *core, *sram;
	u32 len, sram_len, v;

	for (unsigned int i = 0; i < ARRAY_SIZE(fixed); i++) {
		if (g15_adt_u32(adt, sgx, fixed[i].name, &v) || v != fixed[i].value) {
			dev_err(gpu->dev, "ADT sgx %s is not the supported %u\n",
				fixed[i].name, fixed[i].value);
			return -EINVAL;
		}
	}
	core = g15_adt_prop(adt, sgx, "perf-states", &len);
	sram = g15_adt_prop(adt, sgx, "perf-states-sram", &sram_len);
	if (!core || !sram || len != 8 * G15_ADT_PERF_STATES || sram_len != len) {
		dev_err(gpu->dev, "ADT perf-state tables missing or of another shape\n");
		return -EINVAL;
	}
	for (unsigned int i = 0; i < G15_ADT_PERF_STATES; i++) {
		u32 hz = get_unaligned_le32(core + 8 * i);

		if (get_unaligned_le32(sram + 8 * i) != hz || hz % 1000000 ||
		    hz > 3000000000U || (!i != !hz)) {
			dev_err(gpu->dev, "ADT perf-state %u (%u Hz) not usable\n", i, hz);
			return -EINVAL;
		}
		gpu->perf_mhz[i] = hz / 1000000;
		gpu->perf_mv[i] = get_unaligned_le32(core + 8 * i + 4);
		gpu->sram_mv[i] = get_unaligned_le32(sram + 8 * i + 4);
		if (gpu->perf_mv[i] < 100 || gpu->perf_mv[i] > 1100 ||
		    gpu->sram_mv[i] < 100 || gpu->sram_mv[i] > 1100) {
			dev_err(gpu->dev, "ADT perf-state %u voltage %u/%u mV out of range\n",
				i, gpu->perf_mv[i], gpu->sram_mv[i]);
			return -EINVAL;
		}
	}
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
	size_t armio, sgx, asc, nub, chosen;
	u64 dram_base, dram_size;
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
	/* Firmware ABI: TEXT at the base of the upper half, DATA right after it. */
	if (gpu->text_va != G15_UPPER ||
	    gpu->data_va != gpu->text_va + resource_size(&gpu->region[G15_TEXT]))
		goto out;
	chosen = g15_adt_child(&adt, 0, "chosen");
	if (!chosen || g15_adt_u64(&adt, chosen, "dram-base", &dram_base) ||
	    g15_adt_u64(&adt, chosen, "dram-size", &dram_size) || !dram_size ||
	    dram_base + dram_size < dram_base) {
		dev_err(gpu->dev, "ADT DRAM bounds unreadable\n");
		goto out;
	}
	/* Structural admission: the ADT says where; this checks it is sane. */
	for (unsigned int i = 0; i < G15_REGIONS; i++) {
		struct resource *r = &gpu->region[i];
		u64 size = resource_size(r);

		if (!size || ((r->start | size) & (G15_PAGE - 1)) ||
		    r->end > (G15_PA_MASK | (G15_PAGE - 1)) ||
		    r->start < dram_base || r->end > dram_base + dram_size - 1 ||
		    (i < G15_TEXT && size != g15_region_size[i]) ||
		    region_intersects(r->start, size, IORESOURCE_SYSTEM_RAM,
				      IORES_DESC_NONE) != REGION_DISJOINT) {
			dev_err(gpu->dev, "ADT GPU region %u %pR not admissible\n", i, r);
			goto out;
		}
		for (unsigned int j = 0; j < i; j++) {
			if (resource_overlaps(r, &gpu->region[j])) {
				dev_err(gpu->dev, "ADT GPU regions %u and %u overlap\n", j, i);
				goto out;
			}
		}
	}
	ret = g15_collect_perf(gpu, &adt, sgx);
out:
	kvfree(copy);
	return ret;
}

/*
 * Normalized TEXT identity: find the single GKTS marker, zero the 8 entropy
 * bytes after it in a private copy, hash, and look the build up.
 */
static int g15_text_identity(struct g15_gpu *gpu)
{
	static const u8 marker[8] = { 0x47, 0x4b, 0x54, 0x53, 0x08, 0, 0, 0 };
	size_t size = resource_size(&gpu->region[G15_TEXT]);
	u64 data_size = resource_size(&gpu->region[G15_DATA]);
	u8 digest[SHA256_DIGEST_SIZE];
	size_t at = 0, found = 0;
	void *map;
	u8 *copy;
	int ret = -EINVAL;

	map = memremap(gpu->region[G15_TEXT].start, size, MEMREMAP_WB);
	if (!map)
		return -ENOMEM;
	copy = vmalloc(size);
	if (copy)
		memcpy(copy, map, size);
	memunmap(map);
	if (!copy)
		return -ENOMEM;
	for (size_t off = 0; off + 16 <= size; off += 8) {
		if (!memcmp(copy + off, marker, sizeof(marker))) {
			at = off;
			found++;
		}
	}
	if (found != 1) {
		dev_err(gpu->dev, "GPU firmware TEXT has %zu GKTS markers, need one\n", found);
		goto out;
	}
	memset(copy + at + 8, 0, 8);
	sha256(copy, size, digest);
	for (unsigned int i = 0; i < ARRAY_SIZE(g15_fw_builds); i++) {
		const struct g15_fw_build *b = &g15_fw_builds[i];

		if (b->text_size == size && b->data_size == data_size && b->marker == at &&
		    !memcmp(b->text_sha256, digest, sizeof(digest))) {
			gpu->fw = b;
			dev_info(gpu->dev, "GPU firmware build: %s\n", b->name);
			ret = 0;
			goto out;
		}
	}
	dev_err(gpu->dev,
		"unsupported GPU firmware build: TEXT %#zx DATA %#llx GKTS %#zx sha256 %*phN\n",
		size, data_size, at, (int)sizeof(digest), digest);
	for (unsigned int i = 0; i < ARRAY_SIZE(g15_fw_builds); i++)
		dev_err(gpu->dev, "supported: %s\n", g15_fw_builds[i].name);
out:
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

/*
 * The machine-specific part of HardwareData, rebuilt from this machine's ADT
 * performance states over the template: the firmware's two DVFS tables take
 * the three shared low states, then per voltage level the higher (table A)
 * and the lower (table B) frequency of the ADT pair. Table A carries the core
 * and SRAM voltages (eight identical lanes per state) and its top frequency.
 * With the reference machine's ADT this reproduces the template exactly.
 */
#define G15_DVFS_STATES	9
static const u8 g15_dvfs_a[G15_DVFS_STATES] = { 0, 1, 2, 3, 5, 7, 9, 11, 13 };
static const u8 g15_dvfs_b[G15_DVFS_STATES] = { 0, 1, 2, 3, 4, 6, 8, 10, 12 };

static int g15_board_from_adt(struct g15_gpu *gpu, u8 *hw)
{
	for (unsigned int s = 1; s < G15_DVFS_STATES; s++) {
		u8 a = g15_dvfs_a[s], pa = g15_dvfs_a[s - 1];
		u8 b = g15_dvfs_b[s], pb = g15_dvfs_b[s - 1];

		if (gpu->perf_mhz[a] <= gpu->perf_mhz[pa] || gpu->perf_mhz[b] <= gpu->perf_mhz[pb] ||
		    gpu->perf_mv[a] < gpu->perf_mv[pa] || gpu->sram_mv[a] < gpu->sram_mv[pa]) {
			dev_err(gpu->dev, "ADT perf-states do not form ordered DVFS tables\n");
			return -EINVAL;
		}
	}
	for (unsigned int s = 0; s < G15_DVFS_STATES; s++) {
		u8 a = g15_dvfs_a[s], b = g15_dvfs_b[s];

		put_unaligned_le32(gpu->perf_mhz[a], hw + 0xb5c + 4 * s);
		put_unaligned_le32(gpu->perf_mhz[b], hw + 0xf9c + 4 * s);
		put_unaligned_le32(gpu->perf_mhz[b], hw + 0x1350 + 4 * s);
		for (unsigned int k = 0; k < 8; k++) {
			put_unaligned_le32(gpu->perf_mv[a], hw + 0xb9c + 0x20 * s + 4 * k);
			put_unaligned_le32(gpu->sram_mv[a], hw + 0xd9c + 0x20 * s + 4 * k);
		}
	}
	put_unaligned_le32(gpu->perf_mhz[g15_dvfs_a[G15_DVFS_STATES - 1]], hw + 0x4dc4);
	gpu->max_khz = gpu->perf_mhz[g15_dvfs_a[G15_DVFS_STATES - 1]] * 1000;
	return 0;
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

static bool early_wake = true;
module_param(early_wake, bool, 0644);
MODULE_PARM_DESC(early_wake, "Queue ring draining and the backend's completion check from the mailbox interrupt instead of the RTKit receive worker");

/*
 * EP 0x20 ring-service request, in the mailbox interrupt: wake only. The
 * drain work and the backend's completion check run in worker context; the
 * gpu and backend outlive the mailbox.
 */
static bool g15_recv_early(void *cookie, u8 endpoint, u64 message)
{
	struct g15_gpu *gpu = cookie;

	if (!READ_ONCE(early_wake) || endpoint != 0x20 || message != 0x0042000000000000ULL ||
	    !READ_ONCE(gpu->started))
		return false;
	queue_work(system_highpri_wq, &gpu->drain);
	if (READ_ONCE(gpu->be))
		g15_backend_kick(gpu);
	return true;
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
	.recv_message_early = g15_recv_early,
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
 * Type 7 is a PB-growth request (PB growth contract 2.1): its whole record
 * is queued for the backend, which must answer it (DEVCTL 7).
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
		} else if (type == 7) {
			if (gpu->grow_head - gpu->grow_tail < G15_GROW_QUEUE) {
				u32 *q = gpu->grow_req[gpu->grow_head % G15_GROW_QUEUE];

				for (u32 j = 0; j < 14; j++)
					q[j] = g15_r32(gpu, rec + 4 * j);
				gpu->grow_head++;
			} else {
				gpu->grow_lost++;
			}
			dev_dbg(gpu->dev, "firmware event 7 (PB growth): ctx %u slot %u counter %u position %u epoch %#llx\n",
				 g15_r32(gpu, rec + 4), g15_r32(gpu, rec + 8),
				 g15_r32(gpu, rec + 12), g15_r32(gpu, rec + 16),
				 g15_r32(gpu, rec + 20) | (u64)g15_r32(gpu, rec + 24) << 32);
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

	/* Activity records cached; fault info and HardwareDataA timestamps not. */
	BUILD_BUG_ON(0x3740 + 0x100 > G15_HWDATA_CACHED || G15_HWDATA_CACHED > 0x4580 + 0x413c ||
		     G15_HWDATA_CACHED >= G15_HARDWARE_SIZE);

	/* Owned objects and channels (contract 4.1). */
	ret = g15_back(gpu, G15_ROOT, 0xc0, G15_PTE_FW) ?:
	      g15_back(gpu, G15_REGION_A, 0x4000, G15_PTE_FW) ?:
	      g15_back(gpu, G15_RUNTIME, 0x4b3, G15_PTE_FW) ?:
	      g15_back(gpu, G15_HWDATA, G15_HWDATA_CACHED,
		       hwdata_cached ? G15_PTE_FW_CACHED : G15_PTE_FW) ?:
	      g15_back(gpu, G15_HWDATA + G15_HWDATA_CACHED, G15_HARDWARE_SIZE - G15_HWDATA_CACHED,
		       G15_PTE_FW) ?:
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
		dev_err(gpu->dev, "board data template hash mismatch\n");
		goto out;
	}
	ret = -EINVAL;
	if (get_unaligned_le32(hw + 0xa38) != 1 ||
	    get_unaligned_le32(hw + 0xb58) != G15_DVFS_STATES - 1 ||
	    get_unaligned_le32(glob + 0x980) != 10 ||
	    get_unaligned_le32(glob + 0x7a8) != 700 ||
	    get_unaligned_le32(glob + 0x7c4) != 0x3f333333)
		goto out;
	ret = g15_board_from_adt(gpu, hw);
	if (ret)
		goto out;
	{
		u8 digest[SHA256_DIGEST_SIZE];

		sha256(hw, G15_HARDWARE_SIZE, digest);
		dev_info(gpu->dev, "board data: ADT performance states, %u MHz top%s\n",
			 gpu->max_khz / 1000, memcmp(digest, g15_hardware_sha256, sizeof(digest)) ?
			 " (differs from the reference machine)" : " (reference machine)");
	}
	ret = -EINVAL;
	put_unaligned_le64(gpu->region[G15_TTBS].start, hw + 0xb44);
	for (unsigned int i = 0; i < ARRAY_SIZE(g15_iomaps); i++)
		put_unaligned_le64(g15_iomaps[i].va + g15_iomaps[i].delta,
				   hw + 0x640 + 32 * g15_iomaps[i].slot + 8);
	put_unaligned_le64(G15_TIMESTAMPS, hw + 0x28);
	put_unaligned_le32(100, glob + 0x980);
	put_unaligned_le32(clamp(idle_standby_us, 100U, 100000U), glob + 0x7a8);
	put_unaligned_le32(g15_permille_f32(idle_min_conf), glob + 0x7c4);
	put_unaligned_le32(!!idle_predictive, glob + 0x30);
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
	ret = -EPROTO;
	for (unsigned int i = 0; i < ARRAY_SIZE(g15_hw_ids); i++)
		if (gpu->version == g15_hw_ids[i].version && gpu->counts == g15_hw_ids[i].counts)
			ret = 0;
	if (ret) {
		dev_err(gpu->dev, "unsupported G15S hardware: version %#x counts %#x\n",
			gpu->version, gpu->counts);
		return ret;
	}
	/* Binning: any non-empty enabled set in each of the two clusters. */
	gpu->core_masks[0] = gpu->cores & G15_CLUSTER_MASK;
	gpu->core_masks[1] = (gpu->cores >> G15_CLUSTER_BITS) & G15_CLUSTER_MASK;
	if (!gpu->core_masks[0] || !gpu->core_masks[1] ||
	    (gpu->cores >> (2 * G15_CLUSTER_BITS))) {
		dev_err(gpu->dev, "implausible G15S core mask %#x\n", gpu->cores);
		return -EPROTO;
	}
	dev_info(gpu->dev, "M3 G15S: %u + %u cores enabled\n",
		 hweight32(gpu->core_masks[0]), hweight32(gpu->core_masks[1]));
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
	WRITE_ONCE(g15_idle_gpu, gpu);
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
	dev_info(dev, "T6030 G15S admitted, ASC=%#x; write start=1 under a watchdog\n",
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
	if (!enable || !apple_pmp_t6030_board())
		return -ENODEV;
	return platform_driver_register(&g15_driver);
}
module_init(g15_init);
MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("T6030 (M3 Pro) G15S GPU owner (cleanroom)");
