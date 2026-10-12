// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Retained J713 DISP/AGX votes through pmp-core.rs's existing RTKit session. */
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/bits.h>
#include <linux/iopoll.h>
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
	return of_machine_is_compatible("apple,j713") ||
		apple_pmp_t6030_board();
}

/* The board's PMP node: J713 by alias and path, J514S by path (no aliases). */
static struct device_node *pmp_bridge_node(void)
{
	if (apple_pmp_t6030_board())
		return of_find_node_by_path("/soc/pmp@350500000");
	return of_find_node_by_path("pmp");
}

struct apple_pmp_display *apple_pmp_display_register(struct device *dev,
		const void *cookie, int (*send)(const void *, u64))
{
	struct apple_pmp_display *p = NULL;
	struct device_node *alias, *qualified;
	const void *table;
	int size;
	bool match;

	bool j514s = apple_pmp_t6030_board();

	if (!dev->of_node ||
	    !(j514s ? of_device_is_compatible(dev->of_node, "apple,t6030-pmp-v2") :
	      of_machine_is_compatible("apple,j713") &&
	      of_device_is_compatible(dev->of_node, "apple,t8132-pmp-v2")))
		return NULL;
	alias = pmp_bridge_node();
	/* full_name is a node component in unflattened FDTs, not an absolute path. */
	qualified = of_find_node_by_path(j514s ? "/soc/pmp@350500000" : "/soc/pmp@380500000");
	match = alias == dev->of_node && qualified == dev->of_node;
	of_node_put(qualified);
	of_node_put(alias);
	table = of_get_property(dev->of_node, "apple,tunable-soc-device", &size);
	/* J514S: DISP is voted through the PTD seed; only AGX uses this bridge. */
	if (!match || !table || !cookie || !send || (!j514s && !j713_disp_table(table, size)))
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
	/* J514S AGX is logical device 5 (contract), the same request word. */
	p->agx_table_valid = j514s || j713_agx_table(table, size);
	mutex_init(&p->request);
	spin_lock_init(&p->lock);
	init_completion(&p->reply);
	/* No drain/disable contract: retain this qualified owner until reboot. */
	__module_get(THIS_MODULE);
	pm_runtime_get_noresume(dev);
	smp_store_release(&j713_display, p);
	dev_info(dev, "%s PMP vote bridge ready; retained existing PMP owner\n",
		 j514s ? "J514S" : "J713");
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

	bool j514s = apple_pmp_t6030_board();

	if (j514s) {
		/* J514S: the AGX vote only, for the GPU node linked to the PMP. */
		if (kind != J713_POWER_AGX || !consumer->of_node ||
		    !of_device_is_compatible(consumer->of_node, "apple,agx-t6030"))
			return ERR_PTR(-ENODEV);
	} else {
		if (!of_machine_is_compatible("apple,j713") || !consumer->of_node ||
		    !of_device_is_compatible(consumer->of_node, kind == J713_POWER_AGX ?
					     "apple,agx-t8132" : "apple,t8132-dcp"))
			return ERR_PTR(-ENODEV);
		dcp = of_find_node_by_path(kind == J713_POWER_AGX ? "gpu" : "dcp");
		match = dcp == consumer->of_node;
		of_node_put(dcp);
		if (!match)
			return ERR_PTR(-ENODEV);
	}
	if (kind == J713_POWER_AGX) {
		dcp = of_parse_phandle(consumer->of_node, "apple,pmp", 0);
		supplier = pmp_bridge_node();
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
	np = pmp_bridge_node();
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

/* J514S AGX vote: command 0x0f, logical device 5, state 1 (same request word). */
int apple_pmp_j514s_gpu_prepare(struct device *consumer)
{
	return j713_power_prepare(consumer, J713_POWER_AGX);
}
EXPORT_SYMBOL_GPL(apple_pmp_j514s_gpu_prepare);

int apple_pmp_j514s_gpu_get(struct device *consumer)
{
	return j713_power_get(consumer, J713_POWER_AGX);
}
EXPORT_SYMBOL_GPL(apple_pmp_j514s_gpu_get);

int apple_pmp_j514s_gpu_check(struct device *consumer)
{
	return j713_power_check(consumer, J713_POWER_AGX);
}
EXPORT_SYMBOL_GPL(apple_pmp_j514s_gpu_check);

/*
 * J514S (T6030). PTD at 0x3503c0000: status +0x10; DISP request word +280*16
 * and ACK +284*16 (PTD ranges 10/11); non-posted request-set aperture
 * +0x10000 + 280*8. DISP is table index 7, ANS index 16.
 */
#define J514S_PTD		0x3503c0000ULL
#define J514S_PTD_SIZE		0x14000
#define J514S_PTD_STATUS	0x10
#define J514S_PTD_REQ		(280 * 16)
#define J514S_PTD_ACK		(284 * 16)
#define J514S_PTD_SET		(0x10000 + 280 * 8)
#define J514S_PTD_DISP		BIT_ULL(7)
#define J514S_PTD_ANS		BIT_ULL(16)
/*
 * Memory floor: CPU-client EACC0/PACC0 PTD indices 288/289 request DCS4 and
 * FAB2 (valid bit 61, DCS bits 51:48, FAB bits 35:32), other fields kept.
 */
#define J514S_PTD_FLOOR_IDX0	288
#define J514S_PTD_FLOOR_MASK	(BIT_ULL(61) | GENMASK_ULL(51, 48) | GENMASK_ULL(35, 32))
#define J514S_PTD_FLOOR		(BIT_ULL(61) | (4ULL << 48) | (2ULL << 32))
#define J514S_PMP_ASC		0x350c00000ULL
#define J514S_PMGR		0x350700000ULL

static bool j514s_display_board(void)
{
	return apple_pmp_t6030_board();
}

/* SYS, FE and CPU of the display power chain, parents first. */
static const u32 j514s_display_ps[] = { 0x1c0, 0x258, 0x10000 };

/*
 * Hold the inherited display epoch: minimum state 15 (bits 19:16), W1C history
 * bits 9:8 written as zero, other controls preserved. Must precede PMP RUN,
 * or the PMP powers the display down and the DCP CPU stops.
 */
static int j514s_display_hold(struct device *dev)
{
	void __iomem *pmgr = ioremap_np(J514S_PMGR, 0x14000);
	int ret = 0;
	u32 val;

	if (!pmgr)
		return -ENOMEM;
	for (unsigned int i = 0; i < ARRAY_SIZE(j514s_display_ps); i++) {
		val = readl(pmgr + j514s_display_ps[i]);
		if ((val & 0xff) != 0xff || (val & BIT(31))) {
			dev_err(dev, "J514S display domain %#x not inherited on: %#x\n",
				j514s_display_ps[i], val);
			ret = -EIO;
			break;
		}
		writel((val & ~0x000f0300) | 0x000f0000, pmgr + j514s_display_ps[i]);
		val = readl(pmgr + j514s_display_ps[i]);
		dev_info(dev, "J514S display domain %#x held: %#x\n", j514s_display_ps[i], val);
		/* Not every domain implements the minimum field (SYS reads 0). */
		if ((val & 0x000f0000) != 0x000f0000)
			dev_warn(dev, "J514S display domain %#x minimum not latched\n",
				 j514s_display_ps[i]);
	}
	iounmap(pmgr);
	return ret;
}

int apple_pmp_display_pre_run(struct device *dev)
{
	void __iomem *asc = NULL, *ptd = NULL;
	u64 status, req;
	u32 run;
	int ret;

	if (!j514s_display_board() || !dev->of_node ||
	    !of_device_is_compatible(dev->of_node, "apple,t6030-pmp-v2"))
		return 0;
	/* The display floor first: the DCP must survive PMP startup. */
	ret = j514s_display_hold(dev);
	if (ret)
		return ret;
	asc = ioremap_np(J514S_PMP_ASC, 0x4000);
	ptd = ioremap_np(J514S_PTD, J514S_PTD_SIZE);
	if (!asc || !ptd) {
		ret = -ENOMEM;
		goto out;
	}
	run = readl(asc + 0x44);
	status = readq(ptd + J514S_PTD_STATUS);
	req = readq(ptd + J514S_PTD_REQ);
	/* Cold start only: no adoption of a running or pre-seeded PMP. */
	if ((run & BIT(4)) || status || req) {
		dev_err(dev, "J514S PMP not cold (run=%#x status=%#llx req=%#llx)\n",
			run, status, req);
		ret = -EBUSY;
		goto out;
	}
	/* One 64-bit set transaction; the readback must be exactly DISP|ANS. */
	writeq(J514S_PTD_DISP | J514S_PTD_ANS, ptd + J514S_PTD_SET);
	req = readq(ptd + J514S_PTD_REQ);
	if (req != (J514S_PTD_DISP | J514S_PTD_ANS)) {
		dev_err(dev, "J514S PTD seed not accepted: req=%#llx\n", req);
		ret = -EIO;
		goto out;
	}
	dev_info(dev, "J514S display held, PTD seeded DISP|ANS: req=%#llx\n", req);
out:
	if (ptd)
		iounmap(ptd);
	if (asc)
		iounmap(asc);
	return ret;
}

static bool j514s_memory_floor_enabled = true;
module_param_named(j514s_memory_floor, j514s_memory_floor_enabled, bool, 0444);
MODULE_PARM_DESC(j514s_memory_floor, "J514S: request the DCS4/FAB2 memory floor once the PMP runs");

/*
 * Once, with the PMP running (PTD status 1): one 64-bit request-set write per
 * index, exact readback. A rejected request is restored to its prior value.
 */
static void j514s_memory_floor(void __iomem *ptd)
{
	static bool done;
	u64 prior, want, got;

	if (done || !j514s_memory_floor_enabled)
		return;
	done = true;
	for (u32 idx = J514S_PTD_FLOOR_IDX0; idx < J514S_PTD_FLOOR_IDX0 + 2; idx++) {
		prior = readq(ptd + idx * 16);
		want = (prior & ~J514S_PTD_FLOOR_MASK) | J514S_PTD_FLOOR;
		writeq(want, ptd + 0x10000 + idx * 8);
		got = readq(ptd + idx * 16);
		if (got != want) {
			writeq(prior, ptd + 0x10000 + idx * 8);
			pr_err("pmp-display: J514S memory floor %u rejected: %#llx -> %#llx\n",
			       idx, prior, got);
			return;
		}
		pr_info("pmp-display: J514S memory floor %u: %#llx -> %#llx\n", idx, prior, got);
	}
}

static int j514s_display_acked(bool wait)
{
	void __iomem *ptd = ioremap_np(J514S_PTD, J514S_PTD_SIZE);
	u64 status;
	int ret;

	if (!ptd)
		return -ENOMEM;
	ret = readq_poll_timeout(ptd + J514S_PTD_STATUS, status, status == 1,
				 1000, wait ? 2000000 : 1);
	/* DISP must be requested and acknowledged; ANS needs no ACK. */
	if (!ret && (!(readq(ptd + J514S_PTD_REQ) & J514S_PTD_DISP) ||
		     !(readq(ptd + J514S_PTD_ACK) & J514S_PTD_DISP)))
		ret = -EIO;
	if (!ret && wait)
		j514s_memory_floor(ptd);
	iounmap(ptd);
	return ret;
}

int apple_pmp_j514s_display_prepare(struct device *consumer)
{
	struct device_node *np;
	struct platform_device *pmp;
	bool bound;

	if (!j514s_display_board())
		return -ENODEV;
	np = of_find_node_by_path("/soc/pmp@350500000");
	pmp = np ? of_find_device_by_node(np) : NULL;
	of_node_put(np);
	if (!pmp)
		return -EPROBE_DEFER;
	bound = device_is_bound(&pmp->dev);
	if (bound && !device_link_add(consumer, &pmp->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		bound = false;
	put_device(&pmp->dev);
	return bound ? 0 : -EPROBE_DEFER;
}
EXPORT_SYMBOL_GPL(apple_pmp_j514s_display_prepare);

int apple_pmp_j514s_display_get(struct device *consumer)
{
	return j514s_display_board() ? j514s_display_acked(true) : -ENODEV;
}
EXPORT_SYMBOL_GPL(apple_pmp_j514s_display_get);

int apple_pmp_j514s_display_check(struct device *consumer)
{
	return j514s_display_board() ? j514s_display_acked(false) : -ENODEV;
}
EXPORT_SYMBOL_GPL(apple_pmp_j514s_display_check);
