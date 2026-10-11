// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Retained J713 DISP/AGX votes through pmp-core.rs's existing RTKit session. */
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/soc/apple/pmp-display.h>

#include "pmp-display-protocol.h"

struct apple_pmp_display {
	struct device *dev;
	struct device_driver *driver;
	const void *cookie;
	int (*send)(const void *, u64);
	struct mutex request;
	spinlock_t lock;
	struct completion reply;
	struct j713_disp_vote vote;
	unsigned long deadline;
	struct device_link *link[J713_POWER_COUNT];
	bool agx_table_valid;
};

/* One qualified J713 supplier; published once, never freed during its epoch. */
static struct apple_pmp_display *j713_display;
static DEFINE_MUTEX(j713_display_registration);

bool apple_pmp_display_retained_board(void)
{
	return of_machine_is_compatible("apple,j713");
}

struct apple_pmp_display *apple_pmp_display_register(struct device *dev,
		const void *cookie, int (*send)(const void *, u64))
{
	struct apple_pmp_display *p = NULL;
	struct device_node *alias, *qualified;
	const void *table;
	int size;
	bool match;

	if (!of_machine_is_compatible("apple,j713") || !dev->of_node ||
	    !of_device_is_compatible(dev->of_node, "apple,t8132-pmp-v2"))
		return NULL;
	alias = of_find_node_by_path("pmp");
	/* full_name is a node component in unflattened FDTs, not an absolute path. */
	qualified = of_find_node_by_path("/soc/pmp@380500000");
	match = alias == dev->of_node && qualified == dev->of_node;
	of_node_put(qualified);
	of_node_put(alias);
	table = of_get_property(dev->of_node, "apple,tunable-soc-device", &size);
	if (!match || !table || !j713_disp_table(table, size) || !cookie || !send)
		return NULL;
	mutex_lock(&j713_display_registration);
	if (j713_display)
		goto out;
	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		goto out;
	p->dev = get_device(dev);
	p->driver = dev->driver;
	p->cookie = cookie;
	p->send = send;
	p->agx_table_valid = j713_agx_table(table, size);
	mutex_init(&p->request);
	spin_lock_init(&p->lock);
	init_completion(&p->reply);
	/* No drain/disable contract: retain this qualified owner until reboot. */
	__module_get(THIS_MODULE);
	pm_runtime_get_noresume(dev);
	smp_store_release(&j713_display, p);
	dev_info(dev, "J713 DISP vote bridge ready; retained existing PMP owner\n");
out:
	mutex_unlock(&j713_display_registration);
	return p;
}

void apple_pmp_display_fail(struct apple_pmp_display *p, int error)
{
	unsigned long flags;

	if (!p)
		return;
	spin_lock_irqsave(&p->lock, flags);
	j713_disp_fail(&p->vote, error);
	spin_unlock_irqrestore(&p->lock, flags);
	complete_all(&p->reply);
}

bool apple_pmp_display_receive(struct apple_pmp_display *p, u8 endpoint, u64 word)
{
	unsigned long flags;

	if (!p || ((word >> 48) & 0xff) != 0x21)
		return false; /* Normal registry traffic belongs to pmp-core.rs. */
	spin_lock_irqsave(&p->lock, flags);
	if (p->vote.pending && time_after_eq(jiffies, p->deadline))
		j713_disp_fail(&p->vote, -ETIMEDOUT);
	else
		j713_disp_reply(&p->vote, endpoint, word);
	spin_unlock_irqrestore(&p->lock, flags);
	complete_all(&p->reply);
	return true; /* This is a reply; never send an ACK for it. */
}

static struct apple_pmp_display *j713_display_consumer(struct device *consumer,
						      enum j713_power_kind kind)
{
	struct apple_pmp_display *p = smp_load_acquire(&j713_display);
	struct device_node *dcp, *supplier;
	bool match;

	if (!of_machine_is_compatible("apple,j713") || !consumer->of_node ||
	    !of_device_is_compatible(consumer->of_node, kind == J713_POWER_AGX ?
				     "apple,agx-t8132" : "apple,t8132-dcp"))
		return ERR_PTR(-ENODEV);
	dcp = of_find_node_by_path(kind == J713_POWER_AGX ? "gpu" : "dcp");
	match = dcp == consumer->of_node;
	of_node_put(dcp);
	if (!match)
		return ERR_PTR(-ENODEV);
	if (kind == J713_POWER_AGX) {
		dcp = of_parse_phandle(consumer->of_node, "apple,pmp", 0);
		supplier = of_find_node_by_path("pmp");
		match = dcp && supplier == dcp;
		of_node_put(dcp);
		of_node_put(supplier);
		if (!match || (p && !p->agx_table_valid))
			return ERR_PTR(-ENODEV);
	}
	return p ? p : ERR_PTR(-EPROBE_DEFER);
}

/* Called during consumer probe, so device-link publication cannot race suspend. */
static int j713_power_prepare(struct device *consumer, enum j713_power_kind kind)
{
	struct apple_pmp_display *p = j713_display_consumer(consumer, kind);
	struct platform_device *pdev;
	struct device_node *np;
	struct device_link *link;
	int ret = -EPROBE_DEFER;

	if (IS_ERR(p) && PTR_ERR(p) != -EPROBE_DEFER)
		return PTR_ERR(p);
	np = of_find_node_by_path("pmp");
	if (!np)
		return -ENODEV;
	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return -EPROBE_DEFER;
	device_lock(&pdev->dev);
	if (!pdev->dev.driver || strcmp(pdev->dev.driver->name, "apple_pmp") ||
	    !dev_get_drvdata(&pdev->dev))
		goto out;
	p = smp_load_acquire(&j713_display);
	if (p && p->link[kind]) {
		ret = 0;
		goto out;
	}
	/* Allocate the fallible link before publishing/pinning the lazy provider. */
	link = device_link_add(consumer, &pdev->dev, DL_FLAG_STATELESS);
	if (!link) {
		ret = -ENOMEM;
		goto out;
	}
	/* Rust checks the actual drvdata type; no provider exists before full binding. */
	p = apple_pmp_display_bound_owner(&pdev->dev);
	if (!p) {
		device_link_del(link);
		ret = -ENODEV;
		goto out;
	}
	if (kind == J713_POWER_AGX && !p->agx_table_valid) {
		device_link_del(link);
		ret = -ENODEV;
		goto out;
	}
	mutex_lock(&p->request);
	p->link[kind] = link;
	ret = 0;
	mutex_unlock(&p->request);
out:
	device_unlock(&pdev->dev);
	put_device(&pdev->dev);
	return ret;
}

int apple_pmp_j713_display_prepare(struct device *consumer)
{
	return j713_power_prepare(consumer, J713_POWER_DISP);
}
EXPORT_SYMBOL_GPL(apple_pmp_j713_display_prepare);

int apple_pmp_j713_gpu_prepare(struct device *consumer)
{
	return j713_power_prepare(consumer, J713_POWER_AGX);
}
EXPORT_SYMBOL_GPL(apple_pmp_j713_gpu_prepare);

static int j713_power_check(struct device *consumer, enum j713_power_kind kind)
{
	struct apple_pmp_display *p = j713_display_consumer(consumer, kind);
	unsigned long flags;
	int ret;

	if (IS_ERR(p))
		return PTR_ERR(p);
	spin_lock_irqsave(&p->lock, flags);
	ret = p->vote.error ? p->vote.error : p->vote.held[kind] ? 0 : -ENODEV;
	spin_unlock_irqrestore(&p->lock, flags);
	return ret;
}

int apple_pmp_j713_display_check(struct device *consumer)
{
	return j713_power_check(consumer, J713_POWER_DISP);
}
EXPORT_SYMBOL_GPL(apple_pmp_j713_display_check);

int apple_pmp_j713_gpu_check(struct device *consumer)
{
	return j713_power_check(consumer, J713_POWER_AGX);
}
EXPORT_SYMBOL_GPL(apple_pmp_j713_gpu_check);

static int j713_power_get(struct device *consumer, enum j713_power_kind kind)
{
	struct apple_pmp_display *p = j713_display_consumer(consumer, kind);
	u64 request = kind == J713_POWER_AGX ? J713_AGX_REQUEST : J713_DISP_REQUEST;
	unsigned long flags, now, deadline;
	int ret;

	if (IS_ERR(p))
		return PTR_ERR(p);
	device_lock(p->dev);
	mutex_lock(&p->request);
	if (p->dev->driver != p->driver || !dev_get_drvdata(p->dev) || !p->link[kind]) {
		ret = -ENODEV;
		goto failed;
	}
	spin_lock_irqsave(&p->lock, flags);
	/* Reset before publishing pending: an immediate reply must not be lost. */
	reinit_completion(&p->reply);
	ret = j713_disp_begin(&p->vote, kind);
	deadline = p->deadline = jiffies + msecs_to_jiffies(1000);
	spin_unlock_irqrestore(&p->lock, flags);
	if (ret <= 0)
		goto out;
	/* send() releases the Rust RTKit mutex before we wait for reception. */
	ret = p->send(p->cookie, request);
	if (ret)
		goto failed;
	now = jiffies;
	if (time_after_eq(now, deadline) ||
	    !wait_for_completion_timeout(&p->reply, deadline - now)) {
		ret = -ETIMEDOUT;
		goto failed;
	}
	ret = j713_power_check(consumer, kind);
	if (ret)
		goto failed;
	dev_info(p->dev, "Retained %s enable accepted: request=%016llx reply=%016llx\n",
		 kind == J713_POWER_AGX ? "AGX" : "DISP", request, J713_DISP_REPLY);
	goto out;
failed:
	apple_pmp_display_fail(p, ret);
out:
	mutex_unlock(&p->request);
	device_unlock(p->dev);
	return ret;
}

int apple_pmp_j713_display_get(struct device *consumer)
{
	return j713_power_get(consumer, J713_POWER_DISP);
}
EXPORT_SYMBOL_GPL(apple_pmp_j713_display_get);

int apple_pmp_j713_gpu_get(struct device *consumer)
{
	return j713_power_get(consumer, J713_POWER_AGX);
}
EXPORT_SYMBOL_GPL(apple_pmp_j713_gpu_get);
