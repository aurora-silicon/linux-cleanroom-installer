/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_COMPUTE_REUSE_H
#define APPLE_GPU_M4_COMPUTE_REUSE_H

#include "gpu-m4-exec.h"

struct gpu_m4_drm_provider_job;
struct gpu_m4_drm_provider_vm;
struct gpu_m4_drm_provider_version;
struct gpu_m4_drm_vm_owner;
/* Implemented by the actual provider VM owner. Keep literal packet consumers
 * independent of the provider's full type/header dependencies.
 */
u8 gpu_m4_drm_provider_binding_wire_generation(u64 generation);

#define GPU_M4_COMPUTE_REUSE_DESCRIPTOR 0xa0U

/* Both owners keep their complete command aliases and private UMA state.
 * Host copies of retirement fields never constitute a displacement lease.
 */
struct gpu_m4_compute_reuse_owner {
	const void *identity;
	const void *vm_lease;
	const void *work;
	u64 work_fw;
	u64 work_gpu;
	u64 counter_fw;
	u64 pool_state_gpu;
	u64 page_list_gpu;
	u64 pool_gpu;
	u64 submitted;
	u32 uma_slot;
	u8 asid;
	u8 binding_generation;
	struct gpu_m4_exec_retirement retirement;
};

struct gpu_m4_compute_reuse_input;

/* Called under the live owner/VM/slot lock before reading the descriptor.
 * Authenticate A's retained successful CORE retirement receipt, separately
 * from its historical observation below. Recheck A's private descriptor/count/
 * pool/aliases and B's CURRENT shared stamp, queue, pipe, timestamp and UMA
 * retirement; A retired BEFORE distinct B replaced it in the
 * SAME original global hardware slot, B fully retired, and that slot's last
 * owner is still B. Shared counters now describe B, not A: never require them
 * to revert to A's historical stamp/prefix. Neither owner is uncertain, cached
 * concurrently, submitted elsewhere or accessible by another preparation.
 * Pin A and B on any failure.
 *
 * Retain B's own VM/ASID lease and retirement evidence, even if B belongs to
 * a different client. This does not itself qualify a cross-client live path.
 * Prove A's canonical command/aliases, submitted counter, persistent 32-MiB
 * pool/freelist/page list and VM/ASID lease remain identical. Stabilize the
 * complete embedded descriptor, including its packed completed u64, for this
 * snapshot. Prove rebuilt_work is a qualified private current-command image
 * for A, with preserved pool/counter mappings (NOT reinitialized lists/pool).
 * Prove output/scratch do not physically alias either retained owner, metadata,
 * pool or source work. For a same-VM slot retarget, independently exclusively
 * lease the destination slot; neither a zero table word nor B's unrelated-slot
 * retirement admits a lease. No live descriptor/global UMA/cache writes here.
 */
typedef int (*gpu_m4_compute_reuse_check)(void *cookie,
				       const struct gpu_m4_compute_reuse_input *input,
				       const void *output, const void *scratch);

struct gpu_m4_compute_reuse_input {
	const struct gpu_m4_compute_reuse_owner *owner;
	const struct gpu_m4_compute_reuse_owner *replacement;
	const void *last_owner;
	const void *rebuilt_work;
	u32 destination_slot;
	void *cookie;
	gpu_m4_compute_reuse_check check;
};

struct gpu_m4_compute_reuse_binding;
/* In addition to the displaced-owner proof above, authenticate BOTH opaque
 * runtime receipt identities against their strongly held actual retired jobs.
 * Receipts can only originate downstream of successful core cache/snapshot/
 * version retirement; a copied exec observation grants NO authority. Prove
 * current job/version/snapshot/VM membership and native/private/cache/resv
 * coverage, same logical VM as A (not captured-version pointer equality),
 * current selected healthy roots and completed wide binding generation. Old
 * A/B versions, receipts and generation bytes remain unchanged. No payload,
 * root, counter, descriptor or slot write is permitted by this callback.
 */
typedef int (*gpu_m4_compute_reuse_check_binding)(void *cookie,
	const struct gpu_m4_compute_reuse_input *input,
	const struct gpu_m4_compute_reuse_binding *binding, u64 current_generation,
	u8 current_wire_generation,
	const void *output, const void *scratch);

struct gpu_m4_compute_reuse_binding {
	struct gpu_m4_drm_provider_job *job;
	struct gpu_m4_drm_provider_vm *vm;
	struct gpu_m4_drm_provider_version *version;
	const struct gpu_m4_drm_vm_owner *snapshot;
	struct gpu_m4_drm_provider_job *owner_retired_job;
	struct gpu_m4_drm_provider_job *replacement_retired_job;
	const void *owner_receipt;
	const void *replacement_receipt;
	void *cookie;
	gpu_m4_compute_reuse_check_binding check;
	/* Explicit serial-batch provenance: replacement_receipt is the exact
	 * earlier command's full wire retirement in this still-protected job.
	 * The callback must prove current lane/cache/roots and same-slot UMA; it
	 * is NOT a successful CORE receipt and cannot use the historical entrance.
	 */
	bool replacement_current_command;
};

/* Detached preservation image, NOT a separately published UMA descriptor.
 * Only the embedded descriptor in output may be copied into A after all
 * dependencies, visibility checks and leases are validated. Publish the
 * returned cumulative counter before normal work publication, never reset it
 * to event sequence/threshold/stamp. Retain B; firmware binds A on submission.
 * A becomes most-recent owner and cannot be cached again until another distinct
 * fully retired replacement displaces it in that same newly selected slot.
 */
struct gpu_m4_compute_reuse_plan {
	u8 preservation[GPU_M4_COMPUTE_REUSE_DESCRIPTOR];
	const void *owner;
	const void *retained_replacement;
	u64 work_fw;
	u64 counter_fw;
	u64 submitted_before;
	u64 submitted_next;
	u32 old_slot;
	u32 destination_slot;
	u64 binding_generation; /* Wide current identity, never A's old receipt. */
	u8 wire_generation; /* Current firmware byte; may equal a historical byte. */
};

static inline bool gpu_m4_compute_reuse_separate(const void *a, unsigned long na,
					       const void *b, unsigned long nb)
{
	unsigned long va = (unsigned long)a, vb = (unsigned long)b;

	return a && b && na && nb && na <= ~0UL - va && nb <= ~0UL - vb &&
		(va + na <= vb || vb + nb <= va);
}

/* Structural production retirement observation only, never a CORE receipt.
 * Saved prefixes may be passed by successors: validate each consumer against
 * the saved producer and the current producer within the bounded half-window.
 * Busy/idle is intentionally not a retirement condition. No elapsed-time gate.
 */
static inline int gpu_m4_compute_reuse_retired_prefixes(
		const struct gpu_m4_exec_retirement *r)
{
	u32 consumer[4], produced, progress;
	unsigned int i;

	if (!r || !r->expected_stamp || (r->expected_stamp & 0xff) ||
	    r->next >= 256 || r->queue_read >= 256 || r->queue_write >= 256 ||
	    r->pipe_capacity != 256 || r->pipe_read >= 256 || r->pipe_cfi >= 256 ||
	    r->pipe_write >= 256 || r->pipe_next >= 256)
		return -EINVAL;
	consumer[0] = r->queue_read;
	for (i = 0; i < 3; i++) {
		if (r->queue_consumer[i] >= 256)
			return -EINVAL;
		consumer[i + 1] = r->queue_consumer[i];
	}
	if (!r->firmware_stamp ||
	    !gpu_m4_exec_stamp_reached(r->firmware_stamp, r->expected_stamp) ||
	    !r->uma_submitted || (!r->aborted && r->uma_submitted != r->uma_completed))
		return -EAGAIN;
	/* A recovered abort never completed: no user stamp or timestamps. */
	if (!r->aborted && (!r->user_stamp ||
	    !gpu_m4_exec_stamp_reached(r->user_stamp, r->expected_stamp) ||
	    !r->timestamp_start || r->timestamp_end <= r->timestamp_start))
		return -EAGAIN;
	produced = (r->queue_write - r->next) & 255;
	if (produced >= 128 || (!produced && r->queue_busy && !r->ignore_busy))
		return -EAGAIN;
	for (i = 0; i < 4; i++) {
		progress = (consumer[i] - r->next) & 255;
		if (progress >= 128 || progress > produced)
			return -EAGAIN;
	}
	produced = (r->pipe_write - r->pipe_next) & 255;
	if (produced >= 128 || ((r->pipe_read - r->pipe_next) & 255) > produced ||
	    ((r->pipe_cfi - r->pipe_next) & 255) > produced)
		return -EAGAIN;
	return 0;
}

/* One saved ring end against the runtime's own published horizon: 0 when the
 * observed consumer passed end without passing the horizon, -EAGAIN when it
 * is still behind end, -EPROTO when it is beyond anything this job published.
 */
static inline int gpu_m4_compute_horizon_position(u32 observed, u32 end, u32 span)
{
	if (((observed - end) & 255) <= span)
		return 0;
	return ((end - observed) & 255) < 128 ? -EAGAIN : -EPROTO;
}

/* Production per-command hardware receipt inside a bounded published window,
 * never a CORE receipt. The horizons split by owner: the physical queue/pipe
 * horizon is the runtime's last reachable producer on this engine (of any
 * in-flight job); the target horizon is this notifier's published compute
 * tail. With neither successor (both horizons equal the own ends) this is
 * exactly the solitary exact-idle predicate. A physical successor needs both
 * producers to equal the owned horizon and every consumer and both pipe
 * cursors to have passed this command's own end but not the horizon; busy
 * may remain set for it. A notifier successor needs the shared stamps to
 * have reached the own target without passing the published tail. A half
 * without a successor keeps its exact rule. Own private UMA submitted/
 * completed and timestamp order stay exact.
 */
static inline int gpu_m4_compute_horizon_retired(const struct gpu_m4_exec_retirement *r,
		u32 queue_horizon, u32 pipe_horizon, u32 target_horizon)
{
	u32 consumer[4], span = 0, pipe_span = 0;
	bool physical, notifier;
	unsigned int i;
	int ret, stage;

	if (!r || queue_horizon >= 256 || pipe_horizon >= 256 || r->next >= 256 ||
	    r->queue_read >= 256 || r->queue_write >= 256 || r->pipe_capacity != 256 ||
	    r->pipe_read >= 256 || r->pipe_cfi >= 256 || r->pipe_write >= 256 ||
	    r->pipe_next >= 256 || !r->expected_stamp || (r->expected_stamp & 0xff))
		return -EINVAL;
	for (i = 0; i < 3; i++)
		if (r->queue_consumer[i] >= 256)
			return -EINVAL;
	physical = queue_horizon != r->next;
	notifier = target_horizon != r->expected_stamp;
	if (!physical && !notifier) {
		if (pipe_horizon != r->pipe_next)
			return -EINVAL;
		return gpu_m4_exec_retired(r);
	}
	if (physical) {
		span = (queue_horizon - r->next) & 255;
		pipe_span = (pipe_horizon - r->pipe_next) & 255;
		if (span >= 128 || !pipe_span || pipe_span >= 128)
			return -EINVAL;
	} else if (pipe_horizon != r->pipe_next) {
		return -EINVAL;
	}
	if (notifier && (!target_horizon || (target_horizon & 0xff) ||
			 !gpu_m4_exec_stamp_reached(target_horizon, r->expected_stamp)))
		return -EINVAL;
	consumer[0] = r->queue_read;
	for (i = 0; i < 3; i++)
		consumer[i + 1] = r->queue_consumer[i];
	if (physical) {
		if (r->queue_write != queue_horizon || r->pipe_write != pipe_horizon)
			return -ESTALE; /* Never accept atop an unowned producer change. */
		for (i = 0; i < 4; i++) {
			ret = gpu_m4_compute_horizon_position(consumer[i], r->next, span);
			if (ret)
				return ret;
		}
		ret = gpu_m4_compute_horizon_position(r->pipe_read, r->pipe_next, pipe_span);
		if (!ret)
			ret = gpu_m4_compute_horizon_position(r->pipe_cfi, r->pipe_next, pipe_span);
		if (ret)
			return ret;
	} else {
		if (r->queue_read != r->next || r->queue_write != r->next ||
		    (r->queue_busy && !r->ignore_busy) ||
		    r->pipe_read != r->pipe_next || r->pipe_cfi != r->pipe_next ||
		    r->pipe_write != r->pipe_next)
			return -EAGAIN;
		for (i = 0; i < 4; i++)
			if (consumer[i] != r->next)
				return -EAGAIN;
	}
	/* A runtime-wide stage cell (stage_horizon) is checked against the
	 * event's published tail whatever this notifier published.
	 */
	stage = r->stage_horizon ? gpu_m4_exec_stage_check(r->firmware_stamp,
		r->expected_stamp, r->stage_horizon) : 0;
	if (stage == -EINVAL)
		return stage;
	/* A recovered abort (r->aborted) never completed: its stage frontier
	 * and positions prove the firmware passed it; no user stamp, timestamps
	 * or completed UMA count need exist (its unit never runs again).
	 */
	if (!notifier) {
		if ((!r->stage_horizon && r->firmware_stamp != r->expected_stamp) ||
		    stage == -EAGAIN || (!r->aborted && r->user_stamp != r->expected_stamp))
			return -EAGAIN;
	} else {
		if ((!r->stage_horizon &&
		     (!r->firmware_stamp ||
		      !gpu_m4_exec_stamp_reached(r->firmware_stamp, r->expected_stamp))) ||
		    stage == -EAGAIN || (!r->aborted && (!r->user_stamp ||
		    !gpu_m4_exec_stamp_reached(r->user_stamp, r->expected_stamp))))
			return -EAGAIN;
		if ((!r->stage_horizon &&
		     !gpu_m4_exec_stamp_reached(target_horizon, r->firmware_stamp)) ||
		    (!r->aborted && !gpu_m4_exec_stamp_reached(target_horizon, r->user_stamp)))
			return -EPROTO; /* Beyond the known submitted tail. */
	}
	if (stage)
		return stage; /* -EPROTO: beyond the event's published tail. */
	if (!r->uma_submitted || (!r->aborted && (r->uma_submitted != r->uma_completed ||
	    !r->timestamp_start || r->timestamp_end <= r->timestamp_start)))
		return -EAGAIN;
	return 0;
}

static inline int gpu_m4_compute_reuse_retired_check(
		const struct gpu_m4_compute_reuse_owner *o, bool prefixes)
{
	if (!o || !o->identity || !o->vm_lease || !o->work || !o->asid ||
	    !o->binding_generation || !o->submitted || o->uma_slot < 1 || o->uma_slot > 32 ||
	    !gpu_m4_exec_fw_range(o->work_fw, GPU_M4_EXEC_COMPUTE_SIZE, 8) ||
	    !gpu_m4_exec_fw_range(o->counter_fw, 8, 8) ||
	    (o->work_gpu & 0x3fff) || (o->pool_state_gpu & 0x3fff) ||
	    (o->page_list_gpu & 0x3fff) || (o->pool_gpu & 0x3fff) ||
	    o->retirement.uma_submitted != o->submitted)
		return -EINVAL;
	return prefixes ? gpu_m4_compute_reuse_retired_prefixes(&o->retirement) :
		gpu_m4_exec_retired(&o->retirement);
}

static inline int gpu_m4_compute_reuse_retired(const struct gpu_m4_compute_reuse_owner *o)
{
	return gpu_m4_compute_reuse_retired_check(o, false);
}

/* Does not clear a pool, lists, counter, live A/B work or global UMA table.
 * Failure leaves output and plan unchanged; scratch is disposable. Apply API
 * timestamp fanout only AFTER this base-sequence restoration, so its relocation
 * does not change the preserved allocator's identity or lifetime.
 */
static inline int gpu_m4_compute_reuse_prepare_checked(void *output,
					     struct gpu_m4_compute_reuse_plan *plan,
					     void *scratch,
					     const struct gpu_m4_compute_reuse_input *in,
					     const struct gpu_m4_compute_reuse_binding *binding,
					     u64 current_generation,
					     u8 current_wire_generation)
{
	struct gpu_m4_compute_reuse_plan next = { 0 };
	struct gpu_m4_exec_interval firmware[4], client[8];
	const struct gpu_m4_compute_reuse_owner *a, *b;
	const u8 *fresh, *descriptor;
	const void *metadata[5];
	unsigned long size[5];
	u32 stamp;
	unsigned int i;
	int ret;

	if (!current_generation || !current_wire_generation ||
	    (binding && current_wire_generation !=
		gpu_m4_drm_provider_binding_wire_generation(current_generation)) ||
	    !in || !plan || !in->owner || !in->replacement ||
	    (!binding && !in->check) || in->destination_slot < 1 || in->destination_slot > 32 ||
	    !in->rebuilt_work || !in->last_owner ||
	    !gpu_m4_compute_reuse_separate(output, GPU_M4_EXEC_COMPUTE_SIZE,
					    scratch, GPU_M4_EXEC_COMPUTE_SIZE) ||
	    !gpu_m4_compute_reuse_separate(plan, sizeof(*plan), output,
					    GPU_M4_EXEC_COMPUTE_SIZE) ||
	    !gpu_m4_compute_reuse_separate(plan, sizeof(*plan), scratch,
					    GPU_M4_EXEC_COMPUTE_SIZE))
		return -EINVAL;
	/* Distinct units can share one prior successful batch/job. Their exact
	 * held receipt addresses and backing identities must still be distinct;
	 * the callback authenticates both actual memberships, never copied success.
	 */
	if (binding && (!binding->job || !binding->vm || !binding->version ||
	    !binding->snapshot || !binding->owner_retired_job ||
	    !binding->replacement_retired_job || !binding->owner_receipt ||
	    !binding->replacement_receipt || !binding->check ||
	    binding->job == binding->owner_retired_job ||
	    (binding->replacement_current_command ?
		binding->job != binding->replacement_retired_job :
		binding->job == binding->replacement_retired_job) ||
	    binding->owner_receipt == binding->replacement_receipt ||
	    !gpu_m4_compute_reuse_separate(output, GPU_M4_EXEC_COMPUTE_SIZE,
		binding, sizeof(*binding)) ||
	    !gpu_m4_compute_reuse_separate(scratch, GPU_M4_EXEC_COMPUTE_SIZE,
		binding, sizeof(*binding)) ||
	    !gpu_m4_compute_reuse_separate(plan, sizeof(*plan), binding, sizeof(*binding))))
		return -EINVAL;
	a = in->owner;
	b = in->replacement;
	if (!binding && (current_generation != a->binding_generation ||
	    current_wire_generation != a->binding_generation))
		return -EINVAL; /* Internal composition cannot bypass the bound proof. */
	metadata[0] = in;
	size[0] = sizeof(*in);
	metadata[1] = a;
	size[1] = sizeof(*a);
	metadata[2] = b;
	size[2] = sizeof(*b);
	metadata[3] = a->work;
	size[3] = GPU_M4_EXEC_COMPUTE_SIZE;
	metadata[4] = b->work;
	size[4] = GPU_M4_EXEC_COMPUTE_SIZE;
	for (i = 0; i < 5; i++)
		if (!gpu_m4_compute_reuse_separate(output, GPU_M4_EXEC_COMPUTE_SIZE,
							metadata[i], size[i]) ||
		    !gpu_m4_compute_reuse_separate(scratch, GPU_M4_EXEC_COMPUTE_SIZE,
							metadata[i], size[i]) ||
		    !gpu_m4_compute_reuse_separate(plan, sizeof(*plan), metadata[i], size[i]))
			return -EINVAL;
	if (!gpu_m4_compute_reuse_separate(output, GPU_M4_EXEC_COMPUTE_SIZE,
					    in->rebuilt_work, GPU_M4_EXEC_COMPUTE_SIZE) ||
	    !gpu_m4_compute_reuse_separate(scratch, GPU_M4_EXEC_COMPUTE_SIZE,
					    in->rebuilt_work, GPU_M4_EXEC_COMPUTE_SIZE) ||
	    !gpu_m4_compute_reuse_separate(plan, sizeof(*plan), in->rebuilt_work,
					    GPU_M4_EXEC_COMPUTE_SIZE) ||
	    !gpu_m4_compute_reuse_separate(a->work, GPU_M4_EXEC_COMPUTE_SIZE,
					    b->work, GPU_M4_EXEC_COMPUTE_SIZE) ||
	    !gpu_m4_compute_reuse_separate(a->work, GPU_M4_EXEC_COMPUTE_SIZE,
					    in->rebuilt_work, GPU_M4_EXEC_COMPUTE_SIZE) ||
	    !gpu_m4_compute_reuse_separate(b->work, GPU_M4_EXEC_COMPUTE_SIZE,
					    in->rebuilt_work, GPU_M4_EXEC_COMPUTE_SIZE) ||
	    /* b: the destination slot's current owner (A's own slot, or an
	     * idle-owner retarget's destination); A's displacement in its old
	     * slot is the caller's separate receipt proof.
	     */
	    a->identity == b->identity || b->uma_slot != in->destination_slot ||
	    in->last_owner != b->identity)
		return -EINVAL;
	ret = binding ? gpu_m4_compute_reuse_retired_check(a, true) :
		gpu_m4_compute_reuse_retired(a);
	if (ret)
		return ret;
	ret = binding ? gpu_m4_compute_reuse_retired_check(b, true) :
		gpu_m4_compute_reuse_retired(b);
	if (ret)
		return ret;
	if (a->submitted == ~0ULL)
		return -EOVERFLOW;
	firmware[0] = (struct gpu_m4_exec_interval){ a->work_fw, GPU_M4_EXEC_COMPUTE_SIZE };
	firmware[1] = (struct gpu_m4_exec_interval){ b->work_fw, GPU_M4_EXEC_COMPUTE_SIZE };
	firmware[2] = (struct gpu_m4_exec_interval){ a->counter_fw, 8 };
	firmware[3] = (struct gpu_m4_exec_interval){ b->counter_fw, 8 };
	client[0] = (struct gpu_m4_exec_interval){ a->work_gpu, 0x4000 };
	client[1] = (struct gpu_m4_exec_interval){ a->pool_state_gpu, GPU_M4_EXEC_POOL_STATE_SIZE };
	client[2] = (struct gpu_m4_exec_interval){ a->page_list_gpu, GPU_M4_EXEC_POOL_PAGE_LIST_SIZE };
	client[3] = (struct gpu_m4_exec_interval){ a->pool_gpu, GPU_M4_EXEC_POOL_SIZE };
	client[4] = (struct gpu_m4_exec_interval){ b->work_gpu, 0x4000 };
	client[5] = (struct gpu_m4_exec_interval){ b->pool_state_gpu, GPU_M4_EXEC_POOL_STATE_SIZE };
	client[6] = (struct gpu_m4_exec_interval){ b->page_list_gpu, GPU_M4_EXEC_POOL_PAGE_LIST_SIZE };
	client[7] = (struct gpu_m4_exec_interval){ b->pool_gpu, GPU_M4_EXEC_POOL_SIZE };
	/* Client VAs are VM-local: different retained leases may legitimately
	 * use the same numbers. Physical-owner separation remains mandatory in
	 * check(); firmware VAs are one shared address space.
	 */
	if (!gpu_m4_exec_disjoint(firmware, 4) || !gpu_m4_exec_disjoint(client, 4) ||
	    !gpu_m4_exec_disjoint(client + 4, 4) ||
	    (a->vm_lease == b->vm_lease && !gpu_m4_exec_disjoint(client, 8)))
		return -EINVAL;
	for (i = 0; i < 8; i++)
		if (client[i].va < GPU_M4_EXEC_VM_START || client[i].va >= GPU_M4_EXEC_VM_END ||
		    client[i].size > GPU_M4_EXEC_VM_END - client[i].va)
			return -EINVAL;
	ret = binding ? binding->check(binding->cookie, in, binding, current_generation,
		current_wire_generation, output, scratch) : in->check(in->cookie, in, output, scratch);
	if (ret)
		return ret > 0 ? (binding ? -EPROTO : -EIO) : ret;
	descriptor = (const u8 *)a->work + 0xe00;
	if (get_unaligned_le64(descriptor) != a->work_fw ||
	    get_unaligned_le32(descriptor + 8) != a->uma_slot ||
	    get_unaligned_le64(descriptor + 0x14) != a->pool_state_gpu ||
	    get_unaligned_le64(descriptor + 0x30) != a->page_list_gpu ||
	    get_unaligned_le64(descriptor + 0x4c) != a->counter_fw ||
	    get_unaligned_le64(descriptor + 0x54) != a->submitted)
		return -EAGAIN;
	fresh = in->rebuilt_work;
	stamp = get_unaligned_le32(fresh + 0x810);
	if (get_unaligned_le32(fresh) != 3 || get_unaligned_le32(fresh + 0xc) != a->asid ||
	    fresh[0x88b] != current_wire_generation ||
	    get_unaligned_le64(fresh + 0x86a) != a->work_fw + 0xe00 ||
	    get_unaligned_le64(fresh + 0x760) != a->work_fw + 0x900 ||
	    get_unaligned_le32(fresh + 0x768) != GPU_M4_EXEC_COMPUTE_SEQUENCE_SIZE ||
	    get_unaligned_le64(fresh + 0xa60) != a->work_fw + 0xe00 ||
	    get_unaligned_le32(fresh + 0x824) != get_unaligned_le32(fresh + 0x938) ||
	    !stamp || (stamp & 0xff) || get_unaligned_le32(fresh + 0xb8c) != stamp)
		return -EINVAL;
	memcpy(next.preservation, descriptor, sizeof(next.preservation));
	memcpy(scratch, fresh, GPU_M4_EXEC_COMPUTE_SIZE);
	memcpy((u8 *)scratch + 0xe00, next.preservation, sizeof(next.preservation));
	put_unaligned_le64(a->work_fw, (u8 *)scratch + 0xe00);
	put_unaligned_le32(in->destination_slot, (u8 *)scratch + 0xe08);
	next.owner = a->identity;
	next.retained_replacement = b->identity;
	next.work_fw = a->work_fw;
	next.counter_fw = a->counter_fw;
	next.submitted_before = a->submitted;
	next.submitted_next = a->submitted + 1;
	next.old_slot = a->uma_slot;
	next.destination_slot = in->destination_slot;
	next.binding_generation = current_generation;
	next.wire_generation = current_wire_generation;
	memcpy(output, scratch, GPU_M4_EXEC_COMPUTE_SIZE);
	*plan = next;
	return 0;
}

/* Source-compatible frozen-generation entrance. The mandatory callback still
 * authenticates historical core retirement and live B displacement; it must
 * not infer them from the structural exec observations. No new-root adoption.
 */
static inline int gpu_m4_compute_reuse_prepare(void *output,
	struct gpu_m4_compute_reuse_plan *plan, void *scratch,
	const struct gpu_m4_compute_reuse_input *in)
{
	return gpu_m4_compute_reuse_prepare_checked(output, plan, scratch, in, NULL,
		in && in->owner ? in->owner->binding_generation : 0,
		in && in->owner ? in->owner->binding_generation : 0);
}

/* Actual current-job binding is supplied as wide identity AND wire byte, and
 * both are authenticated by check() against the exact resident-binding query.
 * Byte recycling never adopts an old job/receipt or rewrites historical A.
 * Current-job binding is proved separately from A's
 * immutable historical receipt. Neither this helper nor a numeric observation
 * can create a successful retirement receipt or select/publish a root.
 */
static inline int gpu_m4_compute_reuse_prepare_bound(void *output,
	struct gpu_m4_compute_reuse_plan *plan, void *scratch,
	const struct gpu_m4_compute_reuse_input *in,
	const struct gpu_m4_compute_reuse_binding *binding, u64 current_generation,
	u8 current_wire_generation)
{
	if (!binding)
		return -EINVAL;
	return gpu_m4_compute_reuse_prepare_checked(output, plan, scratch, in,
		binding, current_generation, current_wire_generation);
}

#endif
