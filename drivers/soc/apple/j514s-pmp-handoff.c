// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * J514S (T6030): the bootloader device tree carries no PMP, PMP DART or PMP
 * mailbox node, and nothing holds the PMP and PMS-SRAM power domains, so
 * Linux powers the PMP off as unused. Before devices are populated, add the
 * three nodes (the PMP with its ADT tunables) and keep both domains on, so
 * pmp-core.rs can start the PMP.
 */
#include <linux/init.h>
#include <linux/kstrtox.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "j514s-pmp-tunables.h"

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
	pr_info("j514s-pmp-handoff: GPU and GPU mailbox nodes %s (%d)\n",
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

	if (!of_machine_is_compatible("apple,j514s") || !j514s_pmp_handoff_enabled)
		return 0;
	pmp = of_find_node_by_path("/soc/pmp@350500000");
	if (pmp) {
		of_node_put(pmp);
		return 0;
	}
	soc = of_find_node_by_path("/soc");
	/* The DCP mailbox carries the AIC phandle; reuse it (no __symbols__). */
	ref = of_find_node_by_path("/soc/mailbox@28ec08000");
	if (!soc || !ref || of_property_read_u32(ref, "interrupt-parent", &irq_parent))
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
	for (unsigned int i = 0; !ret && i < ARRAY_SIZE(j514s_pmp_props); i++)
		ret = j514s_pmp_raw_prop(cs, pmp, j514s_pmp_props[i].name,
					 j514s_pmp_props[i].data, j514s_pmp_props[i].size);
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
	pr_info("j514s-pmp-handoff: PMP, DART and mailbox nodes %s (%d)\n",
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
