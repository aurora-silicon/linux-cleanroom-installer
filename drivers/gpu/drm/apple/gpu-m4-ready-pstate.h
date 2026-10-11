/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_READY_PSTATE_H
#define APPLE_GPU_M4_READY_PSTATE_H

/* Retained-session performance ceiling. All state is under the GPU mutex. */

/* Thermal/sysfs update: ONLY the three Globals selectors, each 100 * limit,
 * ordered before any later ring/doorbell publication. The initialized Main
 * tables and every firmware-owned PowerPerformance word stay untouched.
 */
static void m4_pstate_publish(struct m4_gpu_hello *h, u32 limit)
{
	static const u16 selectors[] = { 0x88, 0x8c, 0x98 };
	struct m4_gpu_ready *r = h->ready;
	void *globals = r->input.object[GPU_M4_GLOBALS];

	for (unsigned int i = 0; i < ARRAY_SIZE(selectors); i++)
		m4_ready_write32(globals + selectors[i], 100 * limit);
	r->pstate.effective = limit;
	r->pstate.updates++;
}

static u32 m4_pstate_mhz(struct m4_gpu_ready *r, u32 state)
{
	/* The table this session published; entries above S were left zero. */
	return state <= r->pstate.configured ?
		m4_ready_read32(r->input.object[GPU_M4_MAIN] + 0xfc8 + state * 4) : 0;
}

/* g16_thermal.rs Cooling::limit(): a due read of the PMP die hotspot (the
 * channel is diagnostic only), the governor and power inputs, and the
 * transition-only decision log. sample=false only recomputes.
 */
static u32 m4_pstate_ceiling(struct m4_gpu_hello *h, bool sample)
{
	struct m4_gpu_pstate *p = &h->ready->pstate;
	struct gpu_m4_thermal *t = &p->thermal;
	u64 now = ktime_get_ns();
	const char *reason;
	u32 governor, power, limit;

	if (sample && gpu_m4_thermal_due(t, now)) {
		long mc = 0;
		int channel = -1, ret = -ENODEV;

		if (p->temperature)
			ret = apple_pmp_temp_hotspot_channel(p->temperature, &mc, &channel);
		if (ret)
			p->sensor_errors++;
		gpu_m4_thermal_sample(t, now, ret, mc, channel);
	}
	governor = gpu_m4_thermal_governor();
	power = gpu_m4_thermal_power();
	if (gpu_m4_thermal_ceiling(t, governor, power, &limit, &reason)) {
		p->decisions++;
		dev_info(h->gpu->dev, "G16G: thermal decision ns=%llu sample_ns=%llu mc=%ld channel=%d error=%d previous_mc=%ld dt_ns=%llu projected_mc=%ld reason=%s hard=%d prediction=%u governor=%u power=%u limit=%u\n",
			 now, t->policy.last_ns, t->mc, t->channel, t->error,
			 t->policy.previous_mc, t->policy.dt_ns, t->policy.projected_mc,
			 reason, t->policy.hard, t->policy.prediction, governor, power, limit);
	}
	return limit;
}

/* update_thermal_limit(), from the live health paths: before work
 * publication, while servicing active work and from the periodic owner. The
 * sensor is read when due (5 ms). Starts once firmware accepted initdata;
 * until then Globals keep the startup S1.
 */
static void m4_pstate_service(struct m4_gpu_hello *h)
{
	struct m4_gpu_ready *r = h->ready;
	struct m4_gpu_pstate *p = &r->pstate;
	u32 limit;

	if (!r->complete)
		return;
	limit = gpu_m4_pstate_limit(p->configured, m4_pstate_ceiling(h, true), p->user);
	if (limit == p->effective)
		return;
	m4_pstate_publish(h, limit);
	dev_info_ratelimited(h->gpu->dev, "GPU pstate ceiling S%u (%u MHz) configured=S%u thermal=S%u level=%u governor=%u power=%u user=S%u temp=%ldmC sensor=%d\n",
			     limit, m4_pstate_mhz(r, limit), p->configured, p->thermal.ceiling,
			     p->thermal.policy.level, p->thermal.governor, p->thermal.power,
			     p->user, p->thermal.mc, p->thermal.error);
}

/* Actual SGX state for health, sampled after any due ceiling update. Callers
 * bound it by the configured S, never by a just-lowered thermal target:
 * firmware transitions are asynchronous. Zero is valid cold/off.
 */
static u32 m4_pstate_sample(struct m4_gpu_hello *h, enum m4_pstate_site site)
{
	struct m4_gpu_pstate *p = &h->ready->pstate;
	u32 state;

	if (site != M4_PSTATE_FIXED)
		m4_pstate_service(h);
	state = readl(h->gpu->sgx_core + 0x1000) & 0xf;
	p->last = state;
	p->samples[site][state]++; /* REMOVE-DEBUG */
	return state;
}

/* REMOVE-DEBUG: ceilings, the cooling policy state and its inputs, a live
 * SGX sample, the sampled-state histograms and PowerPerformance diagnostics.
 * Firmware-written words are sequential reads through the existing NC
 * mapping, not an atomic snapshot.
 */
static ssize_t pstate_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	static const char *const sites[] = { "health", "poll", "fixed" };
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_hello *h;
	struct m4_gpu_ready *r;
	ssize_t size;

	mutex_lock(&gpu->lock);
	h = gpu->hello;
	r = h ? h->ready : NULL;
	if (!r || !r->prepared) {
		size = sysfs_emit(buf, "prepared=0 configured=%u\n", pstate_max);
	} else {
		struct m4_gpu_pstate *p = &r->pstate;
		void *g = r->input.object[GPU_M4_GLOBALS];
		void *pp = r->input.object[GPU_M4_POWER_PERFORMANCE];
		struct gpu_m4_thermal *t = &p->thermal;
		const struct apple_pmp_cooling *c = &t->policy;
		u32 state = readl(gpu->sgx_core + 0x1000) & 0xf;

		size = sysfs_emit(buf, "prepared=1 complete=%u configured=%u user=%u effective=%u globals=%u,%u,%u ",
			r->complete, p->configured, p->user, p->effective,
			m4_ready_read32(g + 0x88), m4_ready_read32(g + 0x8c),
			m4_ready_read32(g + 0x98));
		size += sysfs_emit_at(buf, size, "ceiling=%u thermal=%u governor=%u power=%u decision=%s level=%u reason=%s hard=%u good_reads=%u prediction=%u projected_mc=%ld previous_mc=%ld dt_ns=%llu anchor=%u:%ld last_ns=%llu good_ns=%llu release_ns=%llu temp_mc=%ld channel=%d sensor=%d ",
			t->ceiling, t->thermal, t->governor, t->power,
			t->logged_reason ?: "none", c->level, gpu_m4_thermal_reason(t), c->hard,
			c->good_reads, c->prediction, c->projected_mc, c->previous_mc, c->dt_ns,
			c->anchor_valid, c->anchor_mc, c->last_ns, c->good_ns, c->release_ns,
			t->mc, t->channel, t->error);
		size += sysfs_emit_at(buf, size, "updates=%llu sensor_errors=%llu decisions=%llu state=%u mhz=%u last=%u pp_actual=%u pp_target=%u pp_current=%u constraint=%u:%u,%u:%u,%u:%u limits=%u,%u",
			p->updates, p->sensor_errors, p->decisions,
			state, m4_pstate_mhz(r, state), p->last,
			m4_ready_read32(pp + 0x2c), m4_ready_read32(pp + 0x30),
			m4_ready_read32(pp + 0x38),
			m4_ready_read32(pp + 0x3c), m4_ready_read32(pp + 0x40),
			m4_ready_read32(pp + 0x44), m4_ready_read32(pp + 0x48),
			m4_ready_read32(pp + 0x4c), m4_ready_read32(pp + 0x50),
			m4_ready_read32(pp + 0xbc0), m4_ready_read32(pp + 0xa38));
		for (unsigned int site = 0; site < M4_PSTATE_SITES; site++) {
			size += sysfs_emit_at(buf, size, " %s=", sites[site]);
			for (unsigned int i = 0; i < 16; i++)
				size += sysfs_emit_at(buf, size, i ? ",%llu" : "%llu",
						      p->samples[site][i]);
		}
		size += sysfs_emit_at(buf, size, "\n");
	}
	mutex_unlock(&gpu->lock);
	return size;
}
static DEVICE_ATTR_RO(pstate_state);

static ssize_t pstate_limit_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_hello *h;
	ssize_t size;

	mutex_lock(&gpu->lock);
	h = gpu->hello;
	size = sysfs_emit(buf, "%u\n", h && h->ready && h->ready->prepared ?
			  h->ready->pstate.user : GPU_M4_PSTATE_MAX);
	mutex_unlock(&gpu->lock);
	return size;
}

/* Session ceiling 1..9 for user-space policy. It can only lower the limit:
 * configured S and the cooling ceiling still apply. A latched failure is
 * returned unchanged with the failed session retained; a raise first takes
 * any due temperature observation (a sensor failure there selects S1).
 */
static ssize_t pstate_limit_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_hello *h;
	struct m4_gpu_ready *r;
	u32 value, limit;
	int ret;

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;
	if (!value || value > GPU_M4_PSTATE_MAX)
		return -EINVAL;
	mutex_lock(&gpu->lock);
	h = gpu->hello;
	r = h ? h->ready : NULL;
	if (!r) {
		ret = -ENODEV;
		goto out;
	}
	ret = gpu->error ?: h->error ?: r->error ?: atomic_read(&h->callback_error);
	if (!ret && !r->prepared)
		ret = -ENODEV;
	if (ret)
		goto out;
	r->pstate.user = value;
	/* Before acceptance the published startup S1 stays; service applies it. */
	if (!r->complete)
		goto out;
	limit = gpu_m4_pstate_limit(r->pstate.configured, m4_pstate_ceiling(h, false), value);
	if (limit > r->pstate.effective)
		limit = gpu_m4_pstate_limit(r->pstate.configured, m4_pstate_ceiling(h, true), value);
	if (limit != r->pstate.effective)
		m4_pstate_publish(h, limit);
out:
	mutex_unlock(&gpu->lock);
	return ret ? ret : (ssize_t)count;
}
static DEVICE_ATTR_RW(pstate_limit);

#endif
