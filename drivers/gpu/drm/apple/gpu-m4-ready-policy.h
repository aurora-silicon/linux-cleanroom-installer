/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_READY_POLICY_H
#define APPLE_GPU_M4_READY_POLICY_H

/* The two admitted policy records are separate from initdata/NOP acceptance.
 * This never changes UAT, starts another firmware session, or submits GPU work.
 */
static int m4_policy_retire(struct m4_gpu_hello *h, u32 target)
{
	void *state = h->ready->input.object[GPU_M4_DEVCTRL_STATE];
	void *control = h->ready->input.object[GPU_M4_CONTROL];
	u64 deadline = ktime_get_ns() + 2 * NSEC_PER_SEC, stable = 0;
	u32 rd = 0, cfi = 0, wr = 0;
	int ret;

	do {
		ret = m4_ready_health(h);
		if (ret)
			return ret;
		ret = m4_ready_drain(h);
		if (ret)
			return ret;
		if (m4_ready_read32(control + 0x14) != 1)
			return -EPROTO;
		rd = m4_ready_read32(state);
		cfi = m4_ready_read32(state + 0x10);
		wr = m4_ready_read32(state + 0x20);
		if (rd < target - 1 || rd > target || cfi < target - 1 ||
		    cfi > target || wr != target)
			return -EPROTO;
		if (rd == target && cfi == target) {
			if (!stable)
				stable = ktime_get_ns();
			if (ktime_get_ns() - stable >= 50 * NSEC_PER_MSEC)
				return 0;
		} else {
			stable = 0;
		}
		usleep_range(1000, 2000);
	} while (ktime_get_ns() < deadline);
	dev_err(h->gpu->dev, "GPU policy control timeout target=%u indices=[%u,%u,%u]\n",
		target, rd, cfi, wr);
	return -ETIMEDOUT;
}

static int m4_policy_enqueue(struct m4_gpu_hello *h, u32 opcode, u32 target)
{
	struct m4_gpu_ready *r = h->ready;
	void *state = r->input.object[GPU_M4_DEVCTRL_STATE];
	void *record;
	int ret;

	if (!((opcode == 0x34 && target == 2) || (opcode == 0x0a && target == 3)))
		return -EINVAL;
	record = r->input.object[GPU_M4_DEVCTRL_RING] + (target - 1) * 0x40;
	ret = m4_ready_health(h);
	if (ret)
		return ret;
	ret = m4_ready_drain(h);
	if (ret)
		return ret;
	if (m4_ready_read32(r->input.object[GPU_M4_CONTROL] + 0x14) != 1 ||
	    m4_ready_read32(state) != target - 1 ||
	    m4_ready_read32(state + 0x10) != target - 1 ||
	    m4_ready_read32(state + 0x20) != target - 1)
		return -EPROTO;
	memset(record, 0, 0x40);
	put_unaligned_le32(opcode, record);
	dma_wmb();
	writel(target, r->generation);
	if (readl(r->generation) != target)
		return -EIO;
	r->policy_generation = target;
	m4_ready_write32(state + 0x20, target);
	ret = apple_rtkit_send_message(h->rtkit, 0x21, 0x0084000000000011ULL, NULL, false);
	if (ret)
		return ret;
	return m4_policy_retire(h, target);
}

static int m4_policy_run(struct m4_gpu_hello *h)
{
	struct m4_gpu_ready *r = h->ready;
	void *scheduler = r->input.object[GPU_M4_FIRMWARE_DATA] + 0x4fd4;
	int ret;

	ret = m4_policy_enqueue(h, 0x34, 2);
	if (ret)
		return ret;
	/* Firmware owns clearing the initial flag. A nonzero word is refusal,
	 * never an invitation to overwrite a generation it has not consumed.
	 */
	if (m4_ready_read32(scheduler))
		return -EBUSY;
	m4_ready_write32(scheduler, 1);
	ret = m4_policy_enqueue(h, 0x0a, 3);
	if (ret)
		return ret;
	ret = apple_rtkit_send_message(h->rtkit, 0x21, 0x0083000000000010ULL, NULL, false);
	if (ret)
		return ret;
	for (unsigned int i = 0; i < 10; i++) {
		msleep(10);
		ret = m4_ready_health(h);
		if (ret)
			return ret;
		ret = m4_ready_drain(h);
		if (ret)
			return ret;
		if (m4_ready_read32(r->input.object[GPU_M4_CONTROL] + 0x14) != 1)
			return -EPROTO;
	}
	/* The accounting generation is deliberately not a retirement predicate. */
	if (m4_ready_read32(r->input.object[GPU_M4_DEVCTRL_STATE]) != 3 ||
	    m4_ready_read32(r->input.object[GPU_M4_DEVCTRL_STATE] + 0x10) != 3 ||
	    m4_ready_read32(r->input.object[GPU_M4_DEVCTRL_STATE] + 0x20) != 3)
		return -EPROTO;
	/* 0x0a must consume the AP rearm. A still-set flag is not readiness. */
	if (m4_ready_read32(scheduler))
		return -EBUSY;
	r->policy_complete = true;
	dev_info(h->gpu->dev, "GPU policy controls retired, scheduler rearmed, S%u; no GPU jobs\n",
		 r->pstate.configured);
	return 0;
}

static ssize_t policy_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_hello *h;
	struct m4_gpu_ready *r;
	ssize_t size;

	mutex_lock(&gpu->lock);
	h = gpu->hello;
	r = h ? h->ready : NULL;
	if (!r) {
		size = sysfs_emit(buf, "attempted=0\n");
	} else {
		void *state = r->input.object[GPU_M4_DEVCTRL_STATE];
		void *fw = r->input.object[GPU_M4_FIRMWARE_DATA];

		size = sysfs_emit(buf, "attempted=%u complete=%u error=%d ready=%u generation=%u indices=%u,%u,%u scheduler=%u trace_dropped=%u trace_lost=%llu\n",
			r->policy_attempted, r->policy_complete, r->error,
			r->input.object[GPU_M4_CONTROL] ?
				m4_ready_read32(r->input.object[GPU_M4_CONTROL] + 0x14) : 0,
			r->policy_generation, state ? m4_ready_read32(state) : 0,
			state ? m4_ready_read32(state + 0x10) : 0,
			state ? m4_ready_read32(state + 0x20) : 0,
			fw ? m4_ready_read32(fw + 0x4fd4) : 0,
			r->trace_dropped_records, r->trace_lost_records);
	}
	mutex_unlock(&gpu->lock);
	return size;
}
static DEVICE_ATTR_RO(policy_state);

static ssize_t policy_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct m4_gpu_power *gpu = dev_get_drvdata(dev);
	struct m4_gpu_hello *h;
	int ret = -EINVAL;

	if (!policy_enable || !hello_enable || !ready_enable || !sysfs_streq(buf, "1"))
		return -EINVAL;
	mutex_lock(&gpu->lock);
	h = gpu->hello;
	if (!h || !h->ready || h->error || h->ready->error ||
	    !h->ready->complete || h->ready->policy_attempted)
		goto out;
	h->ready->policy_attempted = true;
	ret = m4_policy_run(h);
	h->ready->error = ret;
	dev_info(dev, "GPU policy continuation complete=%u error=%d; owners retained\n",
		 h->ready->policy_complete, ret);
out:
	mutex_unlock(&gpu->lock);
	return ret ? ret : (ssize_t)count;
}
static DEVICE_ATTR_WO(policy);

#endif
