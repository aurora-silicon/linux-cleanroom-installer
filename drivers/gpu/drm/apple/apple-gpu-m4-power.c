// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Explicitly guarded J713 power admission and optional RTKit HELLO experiment. */
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/soc/apple/pmp-display.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <crypto/sha2.h>
#include <asm/cacheflush.h>

static bool enable;
module_param(enable, bool, 0400);
MODULE_PARM_DESC(enable, "Permit watchdog-guarded J713 powered admission");

struct m4_gpu_power {
	struct device *dev;
	void __iomem *asc;
	void __iomem *sgx_id;
	void __iomem *sgx_core;
	void __iomem *fault_bank;
	u64 fault_bank_pa;
	void __iomem *ttbs;
	struct m4_gpu_hello *hello;
	struct mutex lock;
	u32 control, version, counts, cores, pstate;
	int error;
	bool voted;
	bool stopped_verified;
	bool firmware_start_attempted;
};

static int m4_gpu_stopped(struct m4_gpu_power *gpu)
{
	unsigned int pass, root;

	/* No cold ASC access: platform genpd attachment has powered the outer chain. */
	gpu->stopped_verified = false;
	for (pass = 0; pass < 2; pass++) {
		gpu->control = readl(gpu->asc + 0x44);
		if (gpu->control & BIT(4))
			return -EBUSY;
		for (root = 0; root < 128; root++)
			if (readq(gpu->ttbs + root * 8))
				return -EBUSY;
	}
	gpu->control = readl(gpu->asc + 0x44);
	if (gpu->control & BIT(4))
		return -EBUSY;
	gpu->stopped_verified = true;
	return 0;
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	ssize_t n;

	mutex_lock(&gpu->lock);
	n = sysfs_emit(buf, "error=%d asc_control=%#x agx_vote=%u version=%#x counts=%#x cores=%#x pstate=%#x stopped_verified=%u firmware_start_attempted=%u\n",
		       gpu->error, gpu->control, gpu->voted, gpu->version,
		       gpu->counts, gpu->cores, gpu->pstate, gpu->stopped_verified,
		       gpu->firmware_start_attempted);
	mutex_unlock(&gpu->lock);
	return n;
}
static DEVICE_ATTR_RO(state);

static ssize_t agx_vote_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;
	mutex_lock(&gpu->lock);
	ret = gpu->error;
	if (ret || gpu->voted)
		goto out;
	ret = m4_gpu_stopped(gpu);
	if (ret)
		goto failed;
	ret = apple_pmp_j713_gpu_get(dev);
	if (ret)
		goto failed;
	gpu->voted = true;
	ret = m4_gpu_stopped(gpu);
	if (ret)
		goto failed;
	/* These SGX reads require the inner AGX vote as well as the outer genpd. */
	gpu->version = readl(gpu->sgx_id);
	gpu->counts = readl(gpu->sgx_id + 0x10);
	gpu->cores = readl(gpu->sgx_core + 0x1500);
	gpu->pstate = readl(gpu->sgx_core + 0x1000) & 0xf;
	/* Actual state zero is valid while cold/stopped; 1..9 is a requested ceiling. */
	if (gpu->version != 0x0a021100 || gpu->counts != 0x0011010a ||
	    !gpu->cores || (gpu->cores & ~0x3ff) || gpu->pstate > 9)
		ret = -EPROTO;
failed:
	gpu->error = ret;
	dev_info(dev, "J713 powered admission error=%d version=%#x counts=%#x cores=%#x pstate=%u; no firmware start\n",
		 ret, gpu->version, gpu->counts, gpu->cores, gpu->pstate);
out:
	mutex_unlock(&gpu->lock);
	return ret ? ret : (ssize_t)count;
}
static DEVICE_ATTR_WO(agx_vote);

#include "gpu-m4-hello.h"

static struct attribute *m4_gpu_power_attrs[] = {
	&dev_attr_state.attr,
	&dev_attr_agx_vote.attr,
	&dev_attr_hello.attr,
	&dev_attr_hello_state.attr,
	&dev_attr_ready.attr,
	&dev_attr_ready_state.attr,
	&dev_attr_policy.attr,
	&dev_attr_policy_state.attr,
	&dev_attr_barrier.attr,
	&dev_attr_execution_state.attr,
	&dev_attr_store.attr,
	&dev_attr_store_state.attr,
	&dev_attr_upper_alias_state.attr,
	&dev_attr_provider_root.attr,
	&dev_attr_provider_root_state.attr,
	&dev_attr_provider_work.attr,
	&dev_attr_provider_work_state.attr,
	&dev_attr_provider_timestamps.attr,
	&dev_attr_provider_timestamps_state.attr,
	&dev_attr_runtime_start.attr,
	&dev_attr_runtime_state.attr,
	&dev_attr_runtime_perf.attr,
	&dev_attr_runtime_render_timing.attr,
	&dev_attr_runtime_asid.attr,
	&dev_attr_runtime_probe.attr,
	&dev_attr_runtime_probe_state.attr,
	&dev_attr_runtime_register.attr,
	&dev_attr_pstate_state.attr,
	&dev_attr_pstate_limit.attr,
	NULL,
};
static const struct bin_attribute *m4_gpu_power_bin_attrs[] = {
	&bin_attr_store_evidence,
	&bin_attr_provider_work_evidence,
	&bin_attr_provider_timestamps_evidence,
	NULL,
};
static const struct attribute_group m4_gpu_power_group = {
	.attrs = m4_gpu_power_attrs,
	.bin_attrs = m4_gpu_power_bin_attrs,
};
__ATTRIBUTE_GROUPS(m4_gpu_power);

static int m4_gpu_power_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct m4_gpu_power *gpu;
	struct resource ttbs;
	struct resource sgx;
	struct resource *sgx_res;
	struct regmap *pmgr;
	u32 offset, state;
	int index, ret;

	gpu = devm_kzalloc(dev, sizeof(*gpu), GFP_KERNEL);
	if (!gpu)
		return -ENOMEM;
	gpu->dev = dev;
	mutex_init(&gpu->lock);
	platform_set_drvdata(pdev, gpu);
	/* Keep power and mapping lifetimes even if admission finds an unknown owner. */
	__module_get(THIS_MODULE);
	get_device(dev);
	pm_runtime_get_noresume(dev);
	ret = pm_runtime_set_active(dev);
	if (ret)
		goto failed;
	pm_runtime_enable(dev);
	np = of_parse_phandle(dev->of_node, "power-domains", 0);
	ret = np ? of_property_read_u32(np, "reg", &offset) : -EINVAL;
	pmgr = np ? syscon_node_to_regmap(np->parent) : ERR_PTR(-EINVAL);
	of_node_put(np);
	if (ret || offset != 0x250 || IS_ERR(pmgr) || !dev->pm_domain) {
		ret = -EINVAL;
		goto failed;
	}
	ret = regmap_read(pmgr, offset, &state);
	if (ret)
		goto failed;
	dev_info(dev, "GFX PMGR powered admission state=%#x\n", state);
	if (((state >> 4) & 0xf) != 0xf) {
		ret = -EHOSTDOWN;
		goto failed;
	}
	gpu->asc = devm_platform_ioremap_resource_byname(pdev, "asc");
	if (IS_ERR(gpu->asc)) {
		ret = PTR_ERR(gpu->asc);
		goto failed;
	}
	/* The enclosing SGX range also contains ASC/mailbox resources. Claim only
	 * the disjoint 16-KiB SGX windows used by this admission test.
	 */
	sgx_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sgx");
	if (!sgx_res) {
		ret = -EINVAL;
		goto failed;
	}
	sgx = *sgx_res;
	sgx.start += 0xd04000;
	sgx.end = sgx.start + 0x3fff;
	gpu->sgx_id = devm_ioremap_resource(dev, &sgx);
	if (IS_ERR(gpu->sgx_id)) {
		ret = PTR_ERR(gpu->sgx_id);
		goto failed;
	}
	sgx = *sgx_res;
	sgx.start += 0xe00000;
	sgx.end = sgx.start + 0x3fff;
	gpu->sgx_core = devm_ioremap_resource(dev, &sgx);
	if (IS_ERR(gpu->sgx_core)) {
		ret = PTR_ERR(gpu->sgx_core);
		goto failed;
	}
	/* REMOVE-DEBUG: J713/25G83 selected fault bank, read only at the first
	 * compute recovery pre-stop. Never write a selector or acknowledge it.
	 * The enclosing main SGX range is not itself mapped or claimed here.
	 */
	if (compute_enable) {
		if (resource_size(sgx_res) < 0x10000) {
			ret = -EINVAL;
			goto failed;
		}
		sgx = *sgx_res;
		sgx.start += 0xc000;
		sgx.end = sgx.start + 0x3fff;
		gpu->fault_bank = devm_ioremap_resource(dev, &sgx);
		if (IS_ERR(gpu->fault_bank)) {
			ret = PTR_ERR(gpu->fault_bank);
			goto failed;
		}
		gpu->fault_bank_pa = sgx.start;
	}
	index = of_property_match_string(dev->of_node, "memory-region-names", "ttbs");
	if (index < 0) {
		ret = index;
		goto failed;
	}
	np = of_parse_phandle(dev->of_node, "memory-region", index);
	ret = !np || !of_property_read_bool(np, "no-map") ||
		of_property_read_bool(np, "reusable") ? -EINVAL :
		of_address_to_resource(np, 0, &ttbs);
	of_node_put(np);
	if (ret)
		goto failed;
	if (resource_size(&ttbs) != 0x4000 || (ttbs.start & 0x3fff) ||
	    ttbs.start < BIT_ULL(40) || ttbs.end >= BIT_ULL(42) ||
	    region_intersects(ttbs.start, resource_size(&ttbs), IORESOURCE_SYSTEM_RAM,
			      IORES_DESC_NONE) != REGION_DISJOINT) {
		ret = -EINVAL;
		goto failed;
	}
	/* Shared producer/consumer roots must never have a simultaneous WB alias. */
	gpu->ttbs = devm_ioremap_wc(dev, ttbs.start, resource_size(&ttbs));
	if (!gpu->ttbs) {
		ret = -ENOMEM;
		goto failed;
	}
	ret = m4_gpu_stopped(gpu);
	if (ret)
		goto failed;
	ret = apple_pmp_j713_gpu_prepare(dev);
failed:
	gpu->error = ret;
	dev_info(dev, "J713 outer power retained: error=%d ASC=%#x; no AGX request or firmware start\n",
		 ret, gpu->control);
	/* Return success to prevent automatic domain detach after powered admission. */
	return 0;
}

static const struct of_device_id m4_gpu_power_match[] = {
	{ .compatible = "apple,agx-t8132" },
	{},
};

static struct platform_driver m4_gpu_power_driver = {
	.probe = m4_gpu_power_probe,
	.driver = {
		.name = "apple-gpu-m4-power",
		.of_match_table = m4_gpu_power_match,
		.dev_groups = m4_gpu_power_groups,
		.suppress_bind_attrs = true,
	},
};

static int __init m4_gpu_power_init(void)
{
	struct device_node *gpu;
	struct resource asc, sgx;
	bool valid;

	/* Must precede driver registration: platform probe powers genpd before probe(). */
	if (!enable || !of_machine_is_compatible("apple,j713") ||
	    !of_machine_is_compatible("apple,t8132"))
		return -ENODEV;
	gpu = of_find_node_by_path("gpu");
	valid = gpu && of_device_is_compatible(gpu, "apple,agx-t8132") &&
		of_device_is_available(gpu) && !of_address_to_resource(gpu, 0, &asc) &&
		!of_address_to_resource(gpu, 1, &sgx) &&
		asc.start == 0x302600000ULL && resource_size(&asc) == 0x4000 &&
		sgx.start == 0x300000000ULL && resource_size(&sgx) == 0x4000000;
	of_node_put(gpu);
	if (!valid)
		return -ENODEV;
	return platform_driver_register(&m4_gpu_power_driver);
}
module_init(m4_gpu_power_init);
MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("J713 watchdog-guarded GPU power admission, no firmware start");
