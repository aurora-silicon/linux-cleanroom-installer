// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * T6030 MacBook Pro (J514S, J516S): the bootloader device tree carries no PMP,
 * PMP DART or PMP mailbox node, and nothing holds the PMP and PMS-SRAM power
 * domains, so Linux powers the PMP off as unused. Before devices are
 * populated, add the three nodes and keep both domains on, so pmp-core.rs can
 * start the PMP.
 *
 * The PMP's tunables, board-id and DRAM vendor are the running machine's own:
 * they are copied from the bootloader's ADT export (phram "adt"), from
 * /arm-io/pmp/iop-pmp-nub and /chosen, for the qualified set of tunable names.
 * Without a readable ADT the handoff adds nothing.
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kstrtox.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/soc/apple/pmp-display.h>
#include <linux/string.h>
#include <linux/unaligned.h>

/* The PMP tunables the qualified J514S PMP session is served (ADT names). */
static const char * const t6030_pmp_tunables[] __initconst = {
	"agx-af-lat-dvfs-filter",
	"agx-af-lat-threshold",
	"agx-af-rd-bw-dvfs-filter",
	"agx-af-rd-bw-threshold",
	"agx-af-wr-bw-dvfs-filter",
	"agx-af-wr-bw-threshold",
	"agx-dcs-bw-dvfs-filter",
	"agx-dcs-bw-threshold",
	"agx-dcs-lat-dvfs-filter",
	"agx-dcs-lat-threshold",
	"agx-dvfs-state-voltages",
	"ane-fast-af-bw-dvfs-filter",
	"ane-fast-af-bw-threshold",
	"ane-fast-bw-dvfs-filter",
	"ane-fast-bw-threshold",
	"ane-fast-dcs-bw-dvfs-filter",
	"ane-fast-dcs-bw-threshold",
	"ane-slow-af-bw-dvfs-filter",
	"ane-slow-af-bw-threshold",
	"ane-slow-bw-dvfs-filter",
	"ane-slow-bw-threshold",
	"ane-slow-dcs-bw-dvfs-filter",
	"ane-slow-dcs-bw-threshold",
	"bwr-catch-up-factor",
	"controller",
	"dcs-bw-threshold",
	"dcs-bwr-threshold",
	"dcs-rd-bwr-threshold",
	"dcs-wr-bwr-threshold",
	"dvfs-disp-min-state",
	"dvfs-domain",
	"fast-die-ctrl-ce-map",
	"fast-die-ctrl-loop",
	"fast-die-ctrl-ts",
	"fast-die-ctrl-ts-avg-temp",
	"ioa0-afnc-bw-threshold",
	"ioa1-afnc-bw-threshold",
	"ioa2-afnc-bw-threshold",
	"ioa3-afnc-bw-threshold",
	"mc_cmd_ack_ptd_count",
	"mc_cmd_ptd_count",
	"mc_quota_req_ptd_count",
	"mc_quota_rsp_ptd_count",
	"mcache-stream",
	"no-shutdown",
	"pm-ptd-ranges",
	"ptd-driver-version",
	"ptd-range",
	"soc-device",
	"soc-device-ps-group",
	"soc0-rd-bwr-threshold",
	"soc0-wr-bwr-threshold",
	"soc1-rd-bwr-threshold",
	"soc1-wr-bwr-threshold",
	"sochot-sensors",
	"sram-index",
	"temp-sensor",
	"user-power-managed",
	"uuid",
};

static bool j514s_pmp_handoff_enabled __initdata = true;

static int __init j514s_pmp_handoff_param(char *arg)
{
	return kstrtobool(arg, &j514s_pmp_handoff_enabled);
}
early_param("j514s_pmp_handoff", j514s_pmp_handoff_param);

static int __init j514s_pmp_raw_prop(struct of_changeset *cs, struct device_node *np,
				    const char *name, const void *data, u32 size)
{
	struct property *prop = kzalloc(sizeof(*prop), GFP_KERNEL);

	if (!prop)
		return -ENOMEM;
	prop->name = kstrdup(name, GFP_KERNEL);
	prop->value = kmemdup(data, size ?: 1, GFP_KERNEL);
	prop->length = size;
	if (!prop->name || !prop->value)
		return -ENOMEM;
	return of_changeset_add_property(cs, np, prop);
}

static phandle __init j514s_pmp_free_phandle(void)
{
	struct device_node *np;
	phandle max = 0;

	for_each_of_allnodes(np)
		if (np->phandle > max)
			max = np->phandle;
	return max + 1;
}

/* ------------------------------------------------- bootloader ADT --- */

struct t6030_adt {
	const u8 *data;
	size_t size;
};

/* End of the node at @off, or 0 for a malformed node. */
static size_t __init t6030_adt_node_end(const struct t6030_adt *adt, size_t off,
					unsigned int depth)
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
		off = t6030_adt_node_end(adt, off, depth + 1);
		if (!off)
			return 0;
	}
	return off;
}

/* Callers only pass nodes inside a tree t6030_adt_node_end() accepted. */
static const u8 * __init t6030_adt_prop(const struct t6030_adt *adt, size_t node,
					const char *name, u32 *len)
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

static size_t __init t6030_adt_child(const struct t6030_adt *adt, size_t node,
				     const char *name)
{
	u32 props = get_unaligned_le32(adt->data + node);
	u32 children = get_unaligned_le32(adt->data + node + 4);
	size_t off = node + 8;
	const u8 *n;
	u32 len;

	for (u32 i = 0; i < props; i++)
		off += 36 + ALIGN(get_unaligned_le32(adt->data + off + 32) & 0x7fffffff, 4);
	for (u32 i = 0; i < children; i++) {
		n = t6030_adt_prop(adt, off, "name", &len);
		if (n && len && strnlen((const char *)n, len) == strlen(name) &&
		    !strncmp((const char *)n, name, len))
			return off;
		off = t6030_adt_node_end(adt, off, 1);
		if (!off)
			return 0;
	}
	return 0;
}

/* A private copy of the loader's phram "adt" export, validated as a tree. */
static int __init t6030_adt_load(struct t6030_adt *adt)
{
	struct device_node *rmem, *np;
	struct resource res;
	const char *label;
	void *map;
	u8 *copy;
	int ret = -ENODEV;

	rmem = of_find_node_by_path("/reserved-memory");
	for_each_child_of_node(rmem, np) {
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
	if (!resource_size(&res) || resource_size(&res) > SZ_2M)
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
	adt->data = copy;
	adt->size = resource_size(&res);
	if (!t6030_adt_node_end(adt, 0, 0)) {
		kvfree(copy);
		adt->data = NULL;
		return -EINVAL;
	}
	return 0;
}

/* ADT little-endian u32 as a one-cell DT property. */
static int __init t6030_pmp_cell_prop(struct of_changeset *cs, struct device_node *np,
				      const char *name, const struct t6030_adt *adt,
				      size_t node, const char *adt_name)
{
	const u8 *p;
	u32 len;

	p = t6030_adt_prop(adt, node, adt_name, &len);
	if (!p || len != 4)
		return -ENOENT;
	return of_changeset_add_prop_u32(cs, np, name, get_unaligned_le32(p));
}

/*
 * The machine's PMP tunables: every qualified name the ADT PMP nub carries,
 * byte for byte, as apple,tunable-<name>. A name the nub lacks stays absent,
 * so the PMP firmware keeps its default for it, as on any other board.
 */
static int __init t6030_pmp_adt_props(struct of_changeset *cs, struct device_node *pmp)
{
	struct t6030_adt adt = {};
	size_t armio, chosen, node, nub;
	unsigned int copied = 0;
	char name[64];
	const u8 *p;
	u32 len;
	int ret;

	ret = t6030_adt_load(&adt);
	if (ret) {
		pr_err("t6030-pmp-handoff: no readable bootloader ADT (%d)\n", ret);
		return ret;
	}
	ret = -ENOENT;
	armio = t6030_adt_child(&adt, 0, "arm-io");
	chosen = t6030_adt_child(&adt, 0, "chosen");
	node = armio ? t6030_adt_child(&adt, armio, "pmp") : 0;
	nub = node ? t6030_adt_child(&adt, node, "iop-pmp-nub") : 0;
	if (!chosen || !nub) {
		pr_err("t6030-pmp-handoff: ADT has no /chosen or PMP nub\n");
		goto out;
	}
	ret = t6030_pmp_cell_prop(cs, pmp, "apple,board-id", &adt, chosen, "board-id") ?:
	      t6030_pmp_cell_prop(cs, pmp, "apple,dram-vendor-id", &adt, chosen,
				  "dram-vendor-id");
	if (ret) {
		pr_err("t6030-pmp-handoff: ADT board-id/dram-vendor-id unreadable (%d)\n", ret);
		goto out;
	}
	for (unsigned int i = 0; !ret && i < ARRAY_SIZE(t6030_pmp_tunables); i++) {
		p = t6030_adt_prop(&adt, nub, t6030_pmp_tunables[i], &len);
		if (!p)
			continue;
		snprintf(name, sizeof(name), "apple,tunable-%s", t6030_pmp_tunables[i]);
		ret = j514s_pmp_raw_prop(cs, pmp, name, p, len);
		copied++;
	}
	if (!ret && !copied)
		ret = -ENOENT;
	pr_info("t6030-pmp-handoff: %u of %zu PMP tunables from the ADT (%d)\n",
		copied, ARRAY_SIZE(t6030_pmp_tunables), ret);
out:
	kvfree(adt.data);
	return ret;
}

/*
 * GPU (G15S) and its ASC mailbox. Firmware, regions and segments come from the
 * ADT at runtime (apple-gpu-g15); the GFX genpd node exists but may lack a
 * phandle, since nothing in the bootloader DT references it.
 */
static void __init j514s_gpu_handoff(struct device_node *soc, u32 irq_parent,
				     phandle pmp_ph, phandle gpu_mbox_ph)
{
	static const char * const mbox_compat[] = { "apple,t6030-asc-mailbox", "apple,asc-mailbox-v4" };
	static const char * const mbox_irqs[] = {
		"send-empty", "send-not-empty", "recv-empty", "recv-not-empty",
	};
	static const char * const g_compat[] = { "apple,agx-t6030" };
	static const char * const g_regs[] = { "asc", "sgx" };
	const u32 gm_reg[] = { 0x2, 0x92408000, 0, 0x4000 };
	const u32 gm_irq[] = { 0, 832, 4, 0, 833, 4, 0, 834, 4, 0, 835, 4 };
	const u32 g_reg[] = { 0x2, 0x92400000, 0, 0x4000, 0x2, 0x90000000, 0, 0x32a4000 };
	struct device_node *gfx, *gpu_mbox, *gpu;
	struct of_changeset *cs;
	phandle gfx_ph;
	int ret = -ENODEV;

	gfx = of_find_node_by_path("/soc/power-management@290e80000/power-controller@110");
	cs = kzalloc(sizeof(*cs), GFP_KERNEL);
	if (!gfx || !cs)
		goto out;
	of_changeset_init(cs);
	gfx_ph = gfx->phandle ?: gpu_mbox_ph + 1;
	gpu_mbox = of_changeset_create_node(cs, soc, "mailbox@292408000");
	gpu = of_changeset_create_node(cs, soc, "gpu@290000000");
	ret = -ENOMEM;
	if (!gpu_mbox || !gpu)
		goto out;
	ret = (gfx->phandle ? 0 : of_changeset_add_prop_u32(cs, gfx, "phandle", gfx_ph)) ?:
	      of_changeset_add_prop_string_array(cs, gpu_mbox, "compatible", mbox_compat, 2) ?:
	      of_changeset_add_prop_u32_array(cs, gpu_mbox, "reg", gm_reg, 4) ?:
	      of_changeset_add_prop_u32(cs, gpu_mbox, "interrupt-parent", irq_parent) ?:
	      of_changeset_add_prop_u32_array(cs, gpu_mbox, "interrupts", gm_irq, 12) ?:
	      of_changeset_add_prop_string_array(cs, gpu_mbox, "interrupt-names", mbox_irqs, 4) ?:
	      of_changeset_add_prop_u32(cs, gpu_mbox, "#mbox-cells", 0) ?:
	      of_changeset_add_prop_u32(cs, gpu_mbox, "phandle", gpu_mbox_ph) ?:
	      of_changeset_add_prop_string_array(cs, gpu, "compatible", g_compat, 1) ?:
	      of_changeset_add_prop_u32_array(cs, gpu, "reg", g_reg, 8) ?:
	      of_changeset_add_prop_string_array(cs, gpu, "reg-names", g_regs, 2) ?:
	      of_changeset_add_prop_u32(cs, gpu, "mboxes", gpu_mbox_ph) ?:
	      of_changeset_add_prop_string(cs, gpu, "mbox-names", "mbox") ?:
	      of_changeset_add_prop_u32(cs, gpu, "power-domains", gfx_ph) ?:
	      of_changeset_add_prop_u32(cs, gpu, "apple,pmp", pmp_ph);
	if (!ret)
		ret = of_changeset_apply(cs);
	if (!ret) {
		gpu_mbox->phandle = gpu_mbox_ph;
		gfx->phandle = gfx_ph;
	}
out:
	pr_info("t6030-pmp-handoff: GPU and GPU mailbox nodes %s (%d)\n",
		ret ? "not added" : "added", ret);
	of_node_put(gfx);
}

static int __init j514s_pmp_handoff(void)
{
	static const char * const dart_compat[] = { "apple,t6030-dart", "apple,t8110-dart" };
	static const char * const mbox_compat[] = { "apple,t6030-asc-mailbox", "apple,asc-mailbox-v4" };
	static const char * const mbox_irqs[] = {
		"send-empty", "send-not-empty", "recv-empty", "recv-not-empty",
	};
	static const char * const pmp_compat[] = { "apple,t6030-pmp-v2", "apple,t6000-pmp-v2" };
	static const char * const pmp_regs[] = { "pmp", "asc" };
	static const char * const domains[] = {
		"/soc/power-management@350700000/power-controller@4c8",	/* pmp */
		"/soc/power-management@350700000/power-controller@4d0",	/* pms_sram */
	};
	const u32 dart_reg[] = { 0x3, 0x50300000, 0, 0x4000 };
	const u32 dart_irq[] = { 0, 0x2de, 4 };
	const u32 mbox_reg[] = { 0x3, 0x50c08000, 0, 0x4000 };
	const u32 mbox_irq[] = { 0, 0x2da, 4, 0, 0x2db, 4, 0, 0x2dc, 4, 0, 0x2dd, 4 };
	const u32 pmp_reg[] = { 0x3, 0x50500000, 0, 0x80000, 0x3, 0x50c00000, 0, 0x4000 };
	struct device_node *soc = NULL, *ref = NULL, *dart, *mbox, *pmp, *pd;
	struct of_changeset *cs;
	phandle dart_ph, mbox_ph, pmp_ph, gpu_mbox_ph;
	u32 irq_parent, iommus[2];
	int ret = -ENODEV;

	if (!apple_pmp_t6030_board() || !j514s_pmp_handoff_enabled)
		return 0;
	pmp = of_find_node_by_path("/soc/pmp@350500000");
	if (pmp) {
		of_node_put(pmp);
		return 0;
	}
	soc = of_find_node_by_path("/soc");
	/* The AIC is referenced by every interrupt user, so it has a phandle. */
	ref = of_find_compatible_node(NULL, NULL, "apple,t6030-aic3");
	irq_parent = ref ? ref->phandle : 0;
	if (!soc || !irq_parent)
		goto out;
	cs = kzalloc(sizeof(*cs), GFP_KERNEL);
	if (!cs) {
		ret = -ENOMEM;
		goto out;
	}
	of_changeset_init(cs);
	dart_ph = j514s_pmp_free_phandle();
	mbox_ph = dart_ph + 1;
	pmp_ph = dart_ph + 2;
	gpu_mbox_ph = dart_ph + 3;

	dart = of_changeset_create_node(cs, soc, "iommu@350300000");
	mbox = of_changeset_create_node(cs, soc, "mailbox@350c08000");
	pmp = of_changeset_create_node(cs, soc, "pmp@350500000");
	if (!dart || !mbox || !pmp) {
		ret = -ENOMEM;
		goto out;
	}

	ret = of_changeset_add_prop_string_array(cs, dart, "compatible", dart_compat, 2) ?:
	      of_changeset_add_prop_u32_array(cs, dart, "reg", dart_reg, 4) ?:
	      of_changeset_add_prop_u32(cs, dart, "#iommu-cells", 1) ?:
	      of_changeset_add_prop_u32(cs, dart, "interrupt-parent", irq_parent) ?:
	      of_changeset_add_prop_u32_array(cs, dart, "interrupts", dart_irq, 3) ?:
	      of_changeset_add_prop_u32(cs, dart, "phandle", dart_ph);
	if (ret)
		goto out;
	ret = of_changeset_add_prop_string_array(cs, mbox, "compatible", mbox_compat, 2) ?:
	      of_changeset_add_prop_u32_array(cs, mbox, "reg", mbox_reg, 4) ?:
	      of_changeset_add_prop_u32(cs, mbox, "interrupt-parent", irq_parent) ?:
	      of_changeset_add_prop_u32_array(cs, mbox, "interrupts", mbox_irq, 12) ?:
	      of_changeset_add_prop_string_array(cs, mbox, "interrupt-names", mbox_irqs, 4) ?:
	      of_changeset_add_prop_u32(cs, mbox, "#mbox-cells", 0) ?:
	      of_changeset_add_prop_u32(cs, mbox, "phandle", mbox_ph);
	if (ret)
		goto out;
	iommus[0] = dart_ph;
	iommus[1] = 0;
	ret = of_changeset_add_prop_string_array(cs, pmp, "compatible", pmp_compat, 2) ?:
	      of_changeset_add_prop_u32_array(cs, pmp, "reg", pmp_reg, 8) ?:
	      of_changeset_add_prop_string_array(cs, pmp, "reg-names", pmp_regs, 2) ?:
	      of_changeset_add_prop_u32_array(cs, pmp, "iommus", iommus, 2) ?:
	      of_changeset_add_prop_u32(cs, pmp, "mboxes", mbox_ph) ?:
	      of_changeset_add_prop_string(cs, pmp, "mbox-names", "mbox");
	if (!ret)
		ret = of_changeset_add_prop_u32(cs, pmp, "phandle", pmp_ph);
	if (!ret)
		ret = t6030_pmp_adt_props(cs, pmp);
	for (unsigned int i = 0; !ret && i < ARRAY_SIZE(domains); i++) {
		pd = of_find_node_by_path(domains[i]);
		ret = pd ? of_changeset_add_prop_bool(cs, pd, "apple,always-on") : -ENODEV;
		of_node_put(pd);
	}
	if (!ret)
		ret = of_changeset_apply(cs);
	/*
	 * Attaching a node derives its phandle from properties present at that
	 * time; the changeset adds them afterwards. Devices are not populated yet.
	 */
	if (!ret) {
		dart->phandle = dart_ph;
		mbox->phandle = mbox_ph;
		pmp->phandle = pmp_ph;
	}
	/* On failure the partially built changeset is leaked: boot-time, one shot. */
	pr_info("t6030-pmp-handoff: PMP, DART and mailbox nodes %s (%d)\n",
		ret ? "not added" : "added", ret);
	/* The GPU is independent: its failure must never cost the display. */
	if (!ret)
		j514s_gpu_handoff(soc, irq_parent, pmp_ph, gpu_mbox_ph);
out:
	of_node_put(ref);
	of_node_put(soc);
	return 0;
}
/* Before of_platform_default_populate() (arch_initcall_sync). */
arch_initcall(j514s_pmp_handoff);
