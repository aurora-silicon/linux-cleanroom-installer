/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Included by the J713 owner. Mailbox reception must never wait for an RPC. */
#include <linux/crc32.h>
#include <linux/jiffies.h>
#include <linux/timekeeping.h>
#include <linux/workqueue.h>
#include "dcp-m4-callback.h"
#include "dcp-m4-property.h"
#include "dcp-m4-frame.h"

struct dcp_m4_runtime {
	struct dcp_m4_ready *dcp;
	struct dcp_m4_properties *properties;
	struct dcp_m4_timing timing;
	struct dcp_m4_swap_state swap;
	struct completion received;
	struct workqueue_struct *wq;
	struct work_struct idle;
	u64 queue[32];
	u32 head, count, callbacks;
	unsigned long deadline;
	bool done;
	bool background_started, background_done;
	u32 background_id;
	bool power_started, power_done;
	bool scanout_started, scanout_done;
	bool qualified; /* KMS startup qualification passed; see dcp_m4_runtime_live() */
	void *scanout_cpu;
	struct dcp_m4_frame scanout_frame;
};

static void dcp_m4_runtime_wake(struct dcp_m4_ready *dcp)
{
	struct dcp_m4_runtime *r = smp_load_acquire(&dcp->runtime);

	if (r)
		complete_all(&r->received);
}

static void dcp_m4_runtime_receive(struct dcp_m4_ready *dcp, u64 word)
{
	struct dcp_m4_runtime *r = smp_load_acquire(&dcp->runtime);
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&dcp->state_lock, flags);
	if (dcp->error)
		ret = dcp->error;
	else if (r->count == ARRAY_SIZE(r->queue)) {
		/* Publish failure before another owner can consume a queued reply. */
		dcp->error = ret = -ENOSPC;
		dcp->ready = false;
	} else {
		r->queue[(r->head + r->count++) % ARRAY_SIZE(r->queue)] = word;
		complete(&r->received);
	}
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	if (ret)
		dcp_m4_failed(dcp, ret);
	else
		queue_work(r->wq, &r->idle);
}

/* Called only while holding dcp->lock, by the sysfs RPC owner or idle worker. */
static int dcp_m4_runtime_next(struct dcp_m4_runtime *r, u64 *word, bool wait)
{
	struct dcp_m4_ready *dcp = r->dcp;
	unsigned long flags, now;
	int ret;

	if (READ_ONCE(dcp->error))
		return READ_ONCE(dcp->error);
	if (wait) {
		now = jiffies;
		if (time_after_eq(now, r->deadline) ||
		    !wait_for_completion_timeout(&r->received, r->deadline - now))
			return -ETIMEDOUT;
	} else if (!try_wait_for_completion(&r->received)) {
		return -EAGAIN;
	}
	spin_lock_irqsave(&dcp->state_lock, flags);
	ret = dcp->error ? dcp->error : !r->count ? -EPROTO : 0;
	if (!ret) {
		*word = r->queue[r->head++ % ARRAY_SIZE(r->queue)];
		r->count--;
	}
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	return ret;
}

static int dcp_m4_runtime_health(struct dcp_m4_runtime *r)
{
	struct dcp_m4_ready *dcp = r->dcp;
	int ret = READ_ONCE(dcp->error);

	if (!ret && (!(readl(dcp->coproc + 0x44) & BIT(4)) ||
	    dcp_m4_dart_check(dcp->dev) || dcp_m4_dart_check(&dcp->piodma->dev)))
		ret = -EIO;
	if (!ret && r->power_done)
		ret = dcp_m4_pmp_check(dcp->dev);
	if (!ret && dcp->scanout)
		ret = dcp_m4_dart_check(&dcp->scanout->dev);
	return ret;
}

/*
 * Full DART/PMP/CPU qualification runs through startup and KMS creation. After
 * that, per-message, per-swap and per-frame checks are only the failure latch;
 * RTKit crashes and protocol errors still latch it asynchronously. The full
 * checks remain available on demand through scanout_probe/pmp_vote reads.
 */
static int dcp_m4_runtime_live(struct dcp_m4_runtime *r)
{
	if (READ_ONCE(r->qualified))
		return READ_ONCE(r->dcp->error);
	return dcp_m4_runtime_health(r);
}

static int dcp_m4_runtime_send(struct dcp_m4_runtime *r, u64 word)
{
	struct dcp_m4_ready *dcp = r->dcp;
	int ret = dcp_m4_runtime_live(r);

	if (ret)
		return ret;
	dma_wmb();
	return apple_rtkit_send_message(dcp->rtkit, DCP_M4_ENDPOINT, word, NULL, false);
}

static int dcp_m4_runtime_call(void *cookie, unsigned int method, bool remote,
				const void *input, u32 input_size, void *output, u32 output_size);

static int dcp_m4_runtime_property(void *cookie, unsigned int method,
				    const void *input, void *output)
{
	struct dcp_m4_runtime *r = cookie;

	return dcp_m4_property_callback(r->properties, method, input, output);
}

static int dcp_m4_runtime_memory(void *cookie, unsigned int method,
				  const void *input, void *output)
{
	struct dcp_m4_runtime *r = cookie;
	struct dcp_m4_ready *dcp = r->dcp;

	return dcp_m4_memory_callback(dcp->memory, &dcp_m4_memory_ops, dcp, method, input, output);
}

static int dcp_m4_runtime_event(void *cookie, const char *name, const void *dict, size_t size)
{
	struct dcp_m4_runtime *r = cookie;

	dev_info(r->dcp->dev, "DCP event %s: %zu bytes CRC32=%08x\n", name, size,
		 dict ? crc32_le(0, dict, size) : 0);
	return 0;
}

static int dcp_m4_runtime_swap_done(void *cookie, u32 id)
{
	struct dcp_m4_runtime *r = cookie;

	return dcp_m4_swap_record_done(&r->swap, id);
}

static u64 dcp_m4_runtime_wall_ms(void *cookie)
{
	return ktime_get_real_ns() / NSEC_PER_MSEC;
}

static const struct dcp_m4_callback_ops dcp_m4_runtime_ops = {
	.call = dcp_m4_runtime_call,
	.property = dcp_m4_runtime_property,
	.memory = dcp_m4_runtime_memory,
	.event = dcp_m4_runtime_event,
	.swap_done = dcp_m4_runtime_swap_done,
	.wall_ms = dcp_m4_runtime_wall_ms,
};

static void dcp_m4_runtime_bad_word(struct dcp_m4_runtime *r, u64 word, int error)
{
	u32 offset = (word >> 16) & 0xffff;

	dev_err(r->dcp->dev, "Rejected RPC notification %016llx: %d\n", word, error);
	/* Diagnostic bytes only: unknown control bits never grant packet ownership. */
	if ((word & 3) == 2 && !(word & 0x40) && !(offset % DCP_M4_ALIGNMENT) &&
	    offset <= DCP_M4_STREAM_SIZE - DCP_M4_PACKET_HEADER) {
		dma_rmb();
		dev_err(r->dcp->dev, "Candidate callback headers remote0=%12ph remote1=%12ph\n",
			(u8 *)r->dcp->rpc + 0x60000 + offset,
			(u8 *)r->dcp->rpc + 0x40000 + offset);
	}
}

static int dcp_m4_runtime_callback(struct dcp_m4_runtime *r, u64 word)
{
	struct dcp_m4_ready *dcp = r->dcp;
	struct dcp_m4_packet packet;
	u8 *input;
	u64 ack;
	int ret, offset;

	dma_rmb();
	ret = dcp_m4_callback_begin(&dcp->transaction, dcp->rpc, DCP_M4_RPC_SIZE, word, &packet);
	if (ret) {
		dcp_m4_runtime_bad_word(r, word, ret);
		return ret;
	}
	offset = dcp_m4_packet_offset(&packet, true);
	if (offset < 0)
		return offset;
	input = (u8 *)dcp->rpc + offset + DCP_M4_PACKET_HEADER;
	r->callbacks++;
	dev_dbg(dcp->dev, "DCP callback D%03d %u/%u remote=%u flag9=%u\n",
		 dcp_m4_callback_method(packet.tag), packet.input_size, packet.output_size,
		 packet.remote, packet.callback_flag);
	if (packet.callback_flag && packet.tag == get_unaligned_le32("D006"))
		dev_dbg(dcp->dev, "Flagged D006 header=%12ph null-byte=%02x; ACK route=342\n",
			 (u8 *)dcp->rpc + offset, input[80]);
	ret = dcp_m4_callback_dispatch(&dcp_m4_runtime_ops, r, packet.tag, packet.remote,
		input, packet.input_size, input + packet.input_size, packet.output_size);
	if (!ret)
		ret = dcp_m4_callback_end(&dcp->transaction, dcp->rpc, DCP_M4_RPC_SIZE, &ack);
	if (!ret)
		ret = dcp_m4_runtime_send(r, ack);
	return ret;
}

static int dcp_m4_runtime_call(void *cookie, unsigned int method, bool remote,
				const void *input, u32 input_size, void *output, u32 output_size)
{
	struct dcp_m4_runtime *r = cookie;
	struct dcp_m4_ready *dcp = r->dcp;
	unsigned long previous = r->deadline;
	struct dcp_m4_packet message;
	u64 word;
	u32 tag;
	int ret;

	if (method > 999)
		return -EINVAL;
	if (!r->deadline)
		r->deadline = jiffies + msecs_to_jiffies(5000);
	if (time_after_eq(jiffies, r->deadline)) {
		ret = -ETIMEDOUT;
		goto finished;
	}
	tag = 'A' | (('0' + method / 100) << 8) |
		(('0' + method / 10 % 10) << 16) | (('0' + method % 10) << 24);
	ret = dcp_m4_call_begin(&dcp->transaction, dcp->rpc, DCP_M4_RPC_SIZE,
			      tag, remote, input, input_size, output_size, &word);
	if (!ret)
		ret = dcp_m4_runtime_send(r, word);
	while (!ret) {
		ret = dcp_m4_runtime_next(r, &word, true);
		if (ret)
			break;
		dma_rmb();
		ret = dcp_m4_decode_word(word, &message);
		if (ret) {
			dcp_m4_runtime_bad_word(r, word, ret);
			break;
		}
		if (message.reply) {
			ret = dcp_m4_call_reply(&dcp->transaction, dcp->rpc, DCP_M4_RPC_SIZE,
					word, output, output_size);
			break;
		}
		ret = dcp_m4_runtime_callback(r, word);
	}
finished:
	r->deadline = previous;
	if (ret) {
		dcp_m4_transaction_fail(&dcp->transaction, ret);
		dcp_m4_failed(dcp, ret);
		dev_err(dcp->dev, "DCP A%03u failed: %d; retained session\n", method, ret);
	}
	return ret;
}

/* One bounded drain; a positive return asks the idle worker to reschedule. */
static int dcp_m4_runtime_drain(struct dcp_m4_runtime *r, unsigned int budget)
{
	u64 word;
	int ret;

	r->deadline = jiffies + msecs_to_jiffies(5000);
	for (unsigned int handled = 0; handled < budget; handled++) {
		ret = dcp_m4_runtime_next(r, &word, false);
		if (ret == -EAGAIN) {
			ret = 0;
			goto out;
		}
		if (ret)
			goto out;
		if (time_after_eq(jiffies, r->deadline)) {
			ret = -ETIMEDOUT;
			goto out;
		}
		ret = dcp_m4_runtime_callback(r, word);
		if (ret)
			goto out;
	}
	ret = 1;
out:
	r->deadline = 0;
	return ret;
}

static void dcp_m4_runtime_idle(struct work_struct *work)
{
	struct dcp_m4_runtime *r = container_of(work, struct dcp_m4_runtime, idle);
	struct dcp_m4_ready *dcp = r->dcp;
	int ret;

	mutex_lock(&dcp->lock);
	ret = dcp_m4_runtime_drain(r, 32);
	if (ret < 0) {
		dcp_m4_transaction_fail(&dcp->transaction, ret);
		dcp_m4_failed(dcp, ret);
	}
	mutex_unlock(&dcp->lock);
	if (ret > 0)
		queue_work(r->wq, &r->idle);
}

/*
 * Serialized owner only; both the A407 reply and its D590 fence must complete.
 * primary/frame/upper/keep as dcp_m4_swap_encode_full().
 */
static int dcp_m4_runtime_swap_full(struct dcp_m4_runtime *r, bool primary,
		const struct dcp_m4_frame *frame, const struct dcp_m4_upper *upper, u32 keep,
		bool brightness_change, u32 nits)
{
	struct dcp_m4_ready *dcp = r->dcp;
	u8 start[16] = {0}, started[8], submitted[12];
	u8 *input;
	u32 id;
	u64 word;
	int ret;

	if (((primary && frame) || (upper && upper->frame)) &&
	    (!r->power_done || !r->background_done || !dcp->scanout))
		return -EINVAL;
	if ((primary && frame && dcp_m4_frame_check(frame)) || (upper && dcp_m4_upper_check(upper)))
		return -EINVAL;
	input = kzalloc(DCP_M4_SWAP_SIZE, GFP_KERNEL);
	if (!input) {
		dcp_m4_swap_fail(&r->swap, -ENOMEM);
		dcp_m4_failed(dcp, -ENOMEM);
		return -ENOMEM;
	}
	r->deadline = jiffies + msecs_to_jiffies(5000);
	ret = dcp_m4_runtime_call(r, 406, false, start, sizeof(start), started, sizeof(started));
	if (!ret)
		ret = dcp_m4_swap_start_reply(started, sizeof(started), &id);
	if (!ret)
		ret = dcp_m4_swap_begin(&r->swap, id);
	if (!ret)
		ret = dcp_m4_swap_encode_full(input, DCP_M4_SWAP_SIZE, id, primary, frame, upper,
					      keep, brightness_change, nits, dcp_m4_frame_check);
	if (!ret)
		ret = dcp_m4_runtime_call(r, 407, false, input, DCP_M4_SWAP_SIZE,
					 submitted, sizeof(submitted));
	if (!ret)
		ret = dcp_m4_swap_record_reply(&r->swap, submitted, sizeof(submitted));
	while (!ret && !r->swap.completed) {
		ret = dcp_m4_runtime_next(r, &word, true);
		if (!ret)
			ret = dcp_m4_runtime_callback(r, word);
	}
	if (!ret)
		ret = dcp_m4_runtime_live(r);
	if (!ret && dcp_m4_swap_take_completion(&r->swap) != 1)
		ret = -EPROTO;
	r->deadline = 0;
	/* Only the copied host input is freed; the firmware-visible RPC stays pinned. */
	kfree(input);
	if (ret) {
		dcp_m4_swap_fail(&r->swap, ret);
		dcp_m4_transaction_fail(&dcp->transaction, ret);
		dcp_m4_failed(dcp, ret);
	} else {
		dev_dbg(dcp->dev, "Display swap %u complete: A407 reply and D590\n", r->swap.id);
	}
	return ret;
}

static int dcp_m4_runtime_swap_brightness(struct dcp_m4_runtime *r,
		const struct dcp_m4_frame *frame, bool brightness_change, u32 nits)
{
	return dcp_m4_runtime_swap_full(r, true, frame, NULL, 0, brightness_change, nits);
}

static int dcp_m4_runtime_swap(struct dcp_m4_runtime *r, const struct dcp_m4_frame *frame)
{
	return dcp_m4_runtime_swap_brightness(r, frame, false, 0);
}

static int dcp_m4_runtime_background(struct dcp_m4_runtime *r)
{
	int ret;

	if (!r->done || r->background_started || READ_ONCE(r->dcp->error))
		return -EBUSY;
	r->background_started = true;
	ret = dcp_m4_runtime_swap(r, NULL);
	if (!ret) {
		r->background_id = r->swap.id;
		smp_store_release(&r->background_done, true);
	}
	return ret;
}

static ssize_t startup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dcp_m4_ready *dcp = dev_get_drvdata(dev);
	struct dcp_m4_runtime *r = smp_load_acquire(&dcp->runtime);
	bool done = r && smp_load_acquire(&r->done) && !READ_ONCE(dcp->error);

	return sysfs_emit(buf, "attempted=%u complete=%u error=%d callbacks=%u mode=%ux%u run=%u\n",
		READ_ONCE(dcp->startup_started), done, READ_ONCE(dcp->error),
		r ? READ_ONCE(r->callbacks) : 0, done ? r->timing.hdisplay : 0,
		done ? r->timing.vdisplay : 0, !!(readl(dcp->coproc + 0x44) & BIT(4)));
}

/* D101 must report the inherited framebuffer stride, not an invented one. */
static int dcp_m4_boot_framebuffer_check(void)
{
	struct device_node *np;
	u32 width, height, stride;
	int ret = -ENODEV, found = 0;

	for_each_compatible_node(np, NULL, "simple-framebuffer") {
		if (!of_device_is_available(np))
			continue;
		found++;
		/* The inherited console may exclude the notch (J713 64, J514S 74 lines). */
		if (of_property_read_u32(np, "width", &width) ||
		    of_property_read_u32(np, "height", &height) ||
		    of_property_read_u32(np, "stride", &stride) ||
		    width != dcp_m4_pf->panel_width || stride != dcp_m4_pf->boot_stride ||
		    (height != dcp_m4_pf->panel_height &&
		     height != dcp_m4_pf->panel_height - (dcp_m4_pf->fw14 ? 74 : 64))) {
			of_node_put(np);
			return -EINVAL;
		}
		/* The inherited console can exclude the 64-line notch region. */
		ret = 0;
	}
	return found > 1 ? -EINVAL : ret;
}

static ssize_t startup_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct dcp_m4_ready *dcp = dev_get_drvdata(dev);
	struct dcp_m4_runtime *r;
	unsigned long flags;
	u8 output[4];
	int ret, slot;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;
	mutex_lock(&dcp->lock);
	ret = -EBUSY;
	if (!READ_ONCE(dcp->ready) || READ_ONCE(dcp->error) || dcp->startup_started ||
	    dcp->rpc_started || dcp->shared_started)
		goto out;
	dcp->startup_started = true;
	ret = dcp_m4_boot_framebuffer_check();
	if (ret)
		goto failed;
	ret = dcp_m4_create_piodma(dcp);
	if (ret)
		goto failed;
	r = devm_kzalloc(dev, sizeof(*r), GFP_KERNEL);
	dcp->memory = devm_kzalloc(dev, sizeof(*dcp->memory), GFP_KERNEL);
	if (!r || !dcp->memory) {
		ret = -ENOMEM;
		goto failed;
	}
	r->properties = devm_kzalloc(dev, sizeof(*r->properties), GFP_KERNEL);
	if (!r->properties) {
		ret = -ENOMEM;
		goto failed;
	}
	r->dcp = dcp;
	init_completion(&r->received);
	INIT_WORK(&r->idle, dcp_m4_runtime_idle);
	r->wq = alloc_ordered_workqueue("dcp-m4-callbacks", WQ_MEM_RECLAIM);
	if (!r->wq) {
		ret = -ENOMEM;
		goto failed;
	}
	spin_lock_irqsave(&dcp->state_lock, flags);
	smp_store_release(&dcp->runtime, r);
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	/* Fresh session only: A401, then the first client open (J713 A457, J514S A455). */
	ret = dcp_m4_runtime_call(r, 401, false, NULL, 0, output, sizeof(output));
	if (!ret && get_unaligned_le32(output) != 1)
		ret = -EPROTO;
	if (!ret)
		ret = dcp_m4_runtime_call(r, dcp_m4_pf->open_method, false, NULL, 0, NULL, 0);
	/* A reply may be followed by property callbacks queued for this owner. */
	if (!ret)
		ret = dcp_m4_runtime_drain(r, 256);
	if (ret > 0)
		ret = -E2BIG;
	if (ret)
		goto failed;
	slot = dcp_m4_raw_slot(r->properties, 0, "PreferredTimingElements", false);
	ret = slot < 0 ? slot : dcp_m4_parse_timing(r->properties->raw[slot].data,
					       r->properties->raw[slot].size, &r->timing);
	if (ret)
		goto failed;
	ret = dcp_m4_runtime_health(r);
	if (ret)
		goto failed;
	smp_store_release(&r->done, true);
	dev_info(dev, "IOMFB startup complete: %u callbacks, %ux%u at %u kHz; no modeset\n",
		 r->callbacks, r->timing.hdisplay, r->timing.vdisplay, r->timing.clock_khz);
	ret = count;
	goto out;
failed:
	dcp_m4_failed(dcp, ret);
	dev_err(dev, "IOMFB startup failed: %d; retained until guarded reboot\n", ret);
out:
	mutex_unlock(&dcp->lock);
	return ret;
}
static DEVICE_ATTR_RW(startup);

static ssize_t background_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dcp_m4_ready *dcp = dev_get_drvdata(dev);
	struct dcp_m4_runtime *r = smp_load_acquire(&dcp->runtime);
	bool done = r && smp_load_acquire(&r->background_done) && !READ_ONCE(dcp->error);

	return sysfs_emit(buf, "attempted=%u complete=%u error=%d swap=%u run=%u\n",
		r && READ_ONCE(r->background_started), done, READ_ONCE(dcp->error),
		done ? r->background_id : 0, !!(readl(dcp->coproc + 0x44) & BIT(4)));
}

static ssize_t background_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct dcp_m4_ready *dcp = dev_get_drvdata(dev);
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;
	mutex_lock(&dcp->lock);
	ret = dcp->runtime ? dcp_m4_runtime_background(dcp->runtime) : -ENODEV;
	mutex_unlock(&dcp->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(background);

static ssize_t pmp_vote_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dcp_m4_ready *dcp = dev_get_drvdata(dev);
	struct dcp_m4_runtime *r = smp_load_acquire(&dcp->runtime);
	bool done = r && smp_load_acquire(&r->power_done);
	int error = READ_ONCE(dcp->error);

	if (!error && done)
		error = dcp_m4_pmp_check(dev);
	return sysfs_emit(buf, "attempted=%u complete=%u error=%d run=%u\n",
		r && READ_ONCE(r->power_started), done && !error, error,
		!!(readl(dcp->coproc + 0x44) & BIT(4)));
}

static ssize_t pmp_vote_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct dcp_m4_ready *dcp = dev_get_drvdata(dev);
	struct dcp_m4_runtime *r;
	int ret = -EBUSY;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;
	mutex_lock(&dcp->lock);
	r = dcp->runtime;
	if (!r || !r->done || r->power_started || READ_ONCE(dcp->error))
		goto out;
	r->power_started = true;
	ret = dcp_m4_runtime_health(r);
	if (!ret)
		ret = dcp_m4_pmp_get(dev);
	if (!ret)
		ret = dcp_m4_pmp_check(dev);
	if (!ret)
		ret = dcp_m4_runtime_health(r);
	if (ret)
		dcp_m4_failed(dcp, ret);
	else
		smp_store_release(&r->power_done, true);
out:
	mutex_unlock(&dcp->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(pmp_vote);

static ssize_t scanout_probe_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dcp_m4_ready *dcp = dev_get_drvdata(dev);
	struct dcp_m4_runtime *r = smp_load_acquire(&dcp->runtime);
	bool done = r && smp_load_acquire(&r->scanout_done);
	int error = READ_ONCE(dcp->error);

	if (!error && done)
		error = dcp_m4_runtime_health(r);
	return sysfs_emit(buf, "attempted=%u complete=%u error=%d swap=%u run=%u\n",
		r && READ_ONCE(r->scanout_started), done && !error, error,
		done ? READ_ONCE(r->swap.id) : 0, !!(readl(dcp->coproc + 0x44) & BIT(4)));
}

/* Diagnostic frame only: no DRM aperture takeover or claim of GPU rendering. */
static ssize_t scanout_probe_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct dcp_m4_ready *dcp = dev_get_drvdata(dev);
	struct dcp_m4_runtime *r;
	dma_addr_t dva;
	int ret = -EBUSY;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;
	mutex_lock(&dcp->lock);
	r = dcp->runtime;
	if (!r || !r->done || !r->power_done || !r->background_done ||
	    r->scanout_started || READ_ONCE(dcp->error))
		goto out;
	r->scanout_started = true;
	ret = dcp_m4_runtime_health(r);
	if (ret)
		goto failed;
	if (r->timing.hdisplay != DCP_M4_PANEL_WIDTH ||
	    r->timing.vdisplay != DCP_M4_PANEL_HEIGHT) {
		ret = -EINVAL;
		goto failed;
	}
	ret = dcp_m4_acquire_scanout(dcp);
	if (ret)
		goto failed;
	r->scanout_frame = (struct dcp_m4_frame) {
		.width = DCP_M4_PANEL_WIDTH, .height = DCP_M4_PANEL_HEIGHT,
		.stride = DCP_M4_PANEL_WIDTH * 4,
		.backing_size = (u64)DCP_M4_PANEL_WIDTH * 4 * DCP_M4_PANEL_HEIGHT,
		.format = DCP_M4_XRGB8888,
	};
	r->scanout_cpu = dcp_m4_scanout_alloc(&dcp->scanout->dev,
						    r->scanout_frame.backing_size, &dva);
	if (IS_ERR(r->scanout_cpu)) {
		ret = PTR_ERR(r->scanout_cpu);
		goto failed;
	}
	r->scanout_frame.dva = dva;
	ret = dcp_m4_frame_pattern(r->scanout_cpu, &r->scanout_frame);
	if (ret)
		goto failed;
	/* CPU uses only the NC alias. Publish all pixels before the firmware descriptor. */
	dma_wmb();
	ret = dcp_m4_runtime_health(r);
	if (!ret)
		ret = dcp_m4_runtime_swap(r, &r->scanout_frame);
	if (ret)
		goto failed;
	smp_store_release(&r->scanout_done, true);
	dev_info(dev, "Native diagnostic scanout complete: %ux%u stride %u DVA %pad; retained until reboot\n",
		 r->scanout_frame.width, r->scanout_frame.height, r->scanout_frame.stride, &dva);
	goto out;
failed:
	dcp_m4_failed(dcp, ret);
out:
	mutex_unlock(&dcp->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(scanout_probe);
