// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* J713 outer graphics gate. This does not own the inner PMP AGX vote. */
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/slab.h>

#define APPLE_GFX_TVM_ENABLE BIT(0)

/* Lab admission: enable explicitly, then bind, under a verified watchdog. */
static bool enable;
module_param(enable, bool, 0600);
MODULE_PARM_DESC(enable, "Permit J713 GFX domain publication (default off)");

struct apple_gfx_tvm {
	struct generic_pm_domain genpd;
	struct device *dev;
	void __iomem *reg;
};

static int apple_gfx_tvm_set(struct generic_pm_domain *genpd, bool on)
{
	struct apple_gfx_tvm *tvm = container_of(genpd, struct apple_gfx_tvm, genpd);
	u32 before = readl(tvm->reg), after;
	u32 wanted = (before & ~APPLE_GFX_TVM_ENABLE) | (on ? APPLE_GFX_TVM_ENABLE : 0);
	int ret;

	writel(wanted, tvm->reg);
	ret = readl_poll_timeout_atomic(tvm->reg, after,
				      !!(after & APPLE_GFX_TVM_ENABLE) == on, 1, 1000);
	dev_info(tvm->dev, "GFX TVM %s before=%#x after=%#x error=%d\n",
		 on ? "on" : "off", before, after, ret);
	return ret;
}

static int apple_gfx_tvm_on(struct generic_pm_domain *genpd)
{
	return apple_gfx_tvm_set(genpd, true);
}

static int apple_gfx_tvm_off(struct generic_pm_domain *genpd)
{
	return apple_gfx_tvm_set(genpd, false);
}

static int apple_gfx_tvm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_gfx_tvm *tvm;
	struct resource *res;
	u32 cells, initial;
	int ret;

	if (!READ_ONCE(enable) || !of_machine_is_compatible("apple,j713") ||
	    !of_machine_is_compatible("apple,t8132"))
		return -ENODEV;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || res->start != 0x380760800ULL || resource_size(res) != 4 ||
	    platform_get_resource(pdev, IORESOURCE_MEM, 1) ||
	    of_property_read_u32(dev->of_node, "#power-domain-cells", &cells) || cells)
		return -EINVAL;
	tvm = devm_kzalloc(dev, sizeof(*tvm), GFP_KERNEL);
	if (!tvm)
		return -ENOMEM;
	tvm->reg = devm_ioremap_resource(dev, res);
	if (IS_ERR(tvm->reg))
		return PTR_ERR(tvm->reg);
	tvm->dev = dev;
	initial = readl(tvm->reg);
	tvm->genpd.name = "j713-gfx-tvm";
	tvm->genpd.flags = GENPD_FLAG_IRQ_SAFE;
	if (initial & APPLE_GFX_TVM_ENABLE)
		tvm->genpd.flags |= GENPD_FLAG_DEFER_OFF | GENPD_FLAG_ACTIVE_WAKEUP;
	tvm->genpd.power_on = apple_gfx_tvm_on;
	tvm->genpd.power_off = apple_gfx_tvm_off;
	ret = pm_genpd_init(&tvm->genpd, NULL, !(initial & APPLE_GFX_TVM_ENABLE));
	if (ret)
		return ret;
	ret = of_genpd_add_provider_simple(dev->of_node, &tvm->genpd);
	if (ret) {
		pm_genpd_remove(&tvm->genpd);
		return ret;
	}
	dev_info(dev, "GFX TVM provider initial=%#x (probe did not write gate)\n", initial);
	return 0;
}

static const struct of_device_id apple_gfx_tvm_match[] = {
	{ .compatible = "apple,t8132-gfx-tvm" },
	{},
};

static struct platform_driver apple_gfx_tvm_driver = {
	.probe = apple_gfx_tvm_probe,
	.driver = {
		.name = "apple-gfx-tvm",
		.of_match_table = apple_gfx_tvm_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(apple_gfx_tvm_driver);

MODULE_DESCRIPTION("Apple J713 outer graphics power gate");
MODULE_LICENSE("Dual MIT/GPL");
