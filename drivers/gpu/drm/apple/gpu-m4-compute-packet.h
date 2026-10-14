/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_COMPUTE_PACKET_H
#define APPLE_GPU_M4_COMPUTE_PACKET_H

#include "gpu-m4-exec.h"

struct gpu_m4_drm_provider_job;
struct gpu_m4_drm_provider_vm;
struct gpu_m4_drm_provider_version;
struct gpu_m4_drm_provider_object;
struct gpu_m4_drm_provider_private_lease;
struct gpu_m4_drm_vm_owner;
struct gpu_m4_drm_job_compute;
u8 gpu_m4_drm_provider_binding_wire_generation(u64 generation);

/* Preparation only, J713/25G83, one bound user ASID, one serialized lane. */
/* Qualified compute UMA lease range; scheduler event 15 is not a UMA slot. */
#define GPU_M4_COMPUTE_PACKET_SLOT_MAX 32U
/* A strict prior-command opcode-4 wait record is retained inside the
 * CONSUMING command's own firmware-only submitted-counter page, disjoint from
 * its 8-byte counter. It is owned (and later reused) only with that unit.
 */
#define GPU_M4_COMPUTE_PACKET_WAIT_OFFSET 0x100U
/* Bounded same-notifier publication depth inside one protected job. */
/* One child window's computes (GPU_M4_DRM_PROVIDER_WINDOW_MAX_PER_CLASS). */
#define GPU_M4_COMPUTE_PACKET_OUTSTANDING_MAX 16U

enum gpu_m4_compute_packet_resource {
	GPU_M4_COMPUTE_PACKET_WORK,
	GPU_M4_COMPUTE_PACKET_PREEMPT,
	GPU_M4_COMPUTE_PACKET_CDM,
	GPU_M4_COMPUTE_PACKET_AUX,
	GPU_M4_COMPUTE_PACKET_CONTEXT,
	GPU_M4_COMPUTE_PACKET_POOL_STATE,
	GPU_M4_COMPUTE_PACKET_PAGE_LIST,
	GPU_M4_COMPUTE_PACKET_POOL,
	GPU_M4_COMPUTE_PACKET_OUTPUT,
	GPU_M4_COMPUTE_PACKET_SHADER,
	GPU_M4_COMPUTE_PACKET_PROGRAM,
	GPU_M4_COMPUTE_PACKET_RESOURCE_COUNT,
};

struct gpu_m4_compute_packet_chronology {
	/* Qualified wire byte; the admitted entrance separately holds wide identity. */
	u64 binding_generation;
	/* The bound user context (1..63) this work runs in: work +0xc, Start
	 * +0x2c, Finalize +0x14 and the work's own notifier record +0x24.
	 */
	u32 asid;
	u64 event_sequence;
	u64 threshold_before;
	u64 threshold_to_publish;
	u64 submitted_before;
	u64 completed_before;
	u64 submitted;
	u32 global_previous_target; /* Persistent compute-engine target chronology. */
	u32 selected_stamp_before;
	u32 target_stamp;
	u32 uma_slot; /* Exclusively held global compute slot, 1..32. */
	/* Actual physical views at reservation, not permission to execute concurrently. */
	u32 queue_read;
	u32 queue_write;
	u32 queue_consumer[3]; /* Exact queue-info +0x20/+0x24/+0x28 views. */
	u32 pipe_read;
	u32 pipe_cfi;
	u32 pipe_write;
	/* Earlier compute commands of the SAME protected job and notifier that
	 * were already reachable, but had no full hardware receipt, when this
	 * reservation was taken. Zero for serial publication. Their stamps are
	 * never treated as completed: the live selected stamp may lag behind.
	 */
	u32 outstanding;
	bool new_queue; /* Physical first use, never a client sequence test. */
};

struct gpu_m4_compute_packet_dependency {
	/* All zero means absent. Otherwise retain the exact captured prior tail. */
	const void *owner;
	struct gpu_m4_exec_interval firmware;
	u64 stamp_fw;
	u32 target;
	u32 event; /* Compute 15 or fragment 0x71, never the registration barrier. */
	/* Strict prior-COMMAND edge (+0x3c=1) on the immediately preceding
	 * command's final FIRMWARE stamp. Its record lives in the consuming
	 * command's own counter page at GPU_M4_COMPUTE_PACKET_WAIT_OFFSET.
	 */
	bool strict;
	/* Experiment (barrier_relax bit 0): emit +0x3c=0, the native compute
	 * publisher's value; the stamp test and ordinary edge remain.
	 */
	bool relaxed;
};

struct gpu_m4_compute_packet_input;
struct gpu_m4_compute_packet_output;

/*
 * Mandatory proof under the actual GPU/provider owner mutex, held throughout
 * preparation and the later separately qualified publication. Recheck the
 * retained READY/power session, registered context, actual healthy control and
 * zero recovery baseline, exact captured resident ASID-1 roots and actual bind
 * generation, all old/new table views and strict original alias health. Prove
 * the complete native BO/pin/VM manifest and unchanged Mesa CDM/container/full
 * Ioto bytes, including every referenced resource not listed in client[]. Do
 * not infer shader scratch/GPR requirements or rewrite any executable bytes.
 *
 * Prove work's firmware/context-0/client aliases and auxiliary aliases are the
 * same owned PA with qualified CPU policy/permissions. Preserve the original
 * work alias; a new ASID root does not create a context-0 alias. Prove exact
 * persistent queue/list/stats/client notifier identities (the queue's 0x18
 * list is NOT the client notifier), generation 0, job-control index 0, and the
 * reserved sequence/threshold/selected stamps/global target, all four physical
 * queue consumers and pipe read/CFI/producer. Authenticate new_queue against
 * the retained physical queue's first-use state, independently of the client.
 * Lease slot 1..32 and completion record 0 exclusively, including pending and
 * retained last-owner state: a zero global table word is not a lease.
 *
 * This adapter admits ONLY a fresh, unsubmitted private allocator/work owner.
 * Prove the descriptor has never been published, fresh private pool/freelist/
 * page list/counter state, complete previous global-lane retirement and cache
 * visibility, no active/uncertain owner, and exact retained prior-dependency
 * tail if present. Numeric snapshots alone do not prove any of these facts.
 * Retained UMA reuse requires the separate proven displacement protocol; it
 * is not admitted by setting submitted_before to a copied count. Prove output
 * and scratch physically disjoint from each other, metadata and all retained
 * backing. A positive callback return is failure. Never publish or release a
 * visible owner here; on uncertainty retain every old/new owner until reboot.
 */
typedef int (*gpu_m4_compute_packet_check_owner)(void *cookie,
		const struct gpu_m4_compute_packet_input *input,
		const struct gpu_m4_compute_packet_output *output,
		const struct gpu_m4_compute_packet_output *scratch);

struct gpu_m4_compute_packet_input {
	/* event_sequence/binding_generation must be zero placeholders; chronology
	 * is their sole authority. A local copy fills them after wide validation.
	 */
	struct gpu_m4_exec_compute_input work;
	const void *owner;
	struct gpu_m4_exec_interval client[GPU_M4_COMPUTE_PACKET_RESOURCE_COUNT];
	/* Complete fresh NC firmware work, auxiliary and counter allocations. */
	struct gpu_m4_exec_interval firmware[3];
	u64 notifier_page_fw;
	struct gpu_m4_compute_packet_chronology chronology;
	struct gpu_m4_compute_packet_dependency prior;
	u64 timestamp;
	void *owner_cookie;
	gpu_m4_compute_packet_check_owner check_owner;
};

struct gpu_m4_compute_packet_publication {
	u64 threshold_fw;
	u64 threshold_before;
	u64 threshold_value;
	u64 counter_fw;
	u64 counter_value;
	u64 binding_generation;
	u8 wire_generation;
	u64 kick;
	u32 queue_before;
	u32 queue_next;
	u32 pipe_before;
	u32 pipe_next;
	u32 queue_entries;
	u32 uma_slot;
	u32 event_sequence;
	u32 target_stamp;
	u8 ring_entries[16]; /* Optional prior dependency, then actual work FW VA. */
	u8 pipe_record[0x18];
};

/* Heap-owned, detached output/scratch, never live command/producer storage. */
struct gpu_m4_compute_packet_output {
	u8 work[GPU_M4_EXEC_COMPUTE_SIZE];
	u8 prior_dependency[GPU_M4_EXEC_DEPENDENCY_SIZE];
	u8 submitted_counter[8];
	struct gpu_m4_compute_packet_publication publication;
};

/* Actual private native allocator roles only. User CDM/USC/indirect resources
 * belong to the independently decoded, conservatively captured user manifest;
 * no fixed shader/program/output span may substitute for that admission.
 */
enum gpu_m4_compute_packet_private_role {
	GPU_M4_COMPUTE_PRIVATE_WORK,
	GPU_M4_COMPUTE_PRIVATE_PREEMPT,
	GPU_M4_COMPUTE_PRIVATE_AUX,
	GPU_M4_COMPUTE_PRIVATE_CONTEXT,
	GPU_M4_COMPUTE_PRIVATE_POOL_STATE,
	GPU_M4_COMPUTE_PRIVATE_PAGE_LIST,
	GPU_M4_COMPUTE_PRIVATE_POOL,
	GPU_M4_COMPUTE_PRIVATE_COUNT,
};

#define GPU_M4_COMPUTE_PRIVATE_RW 6U /* Actual UAPI READ(2)|WRITE(4). */

struct gpu_m4_compute_packet_private {
	struct gpu_m4_drm_provider_object *object;
	struct gpu_m4_drm_provider_private_lease *lease;
	struct gpu_m4_exec_interval client;
	u64 object_offset;
	u32 access; /* Actual UAPI READ|WRITE, not IOMMU permission numbers. */
};

struct gpu_m4_compute_packet_admitted_input;

/* Mandatory proof under the actual shared GPU/provider mutex. The callback
 * verifies registered job/VM/version/snapshot and command membership, original
 * user admission BEFORE private staging, complete native cache/resv coverage,
 * actual native origins/offsets/pages/NC policy and every held private claim.
 * Check the current completed wide bind identity AND its installed wire byte,
 * protected lane/dependencies,
 * retained READY/queue/notifier/slot/event owners, all four queue consumers,
 * physical first-use state and pipe views, old/new graph health and
 * original aliases. Work's same-VA context-0/client and canonical SHARED upper
 * aliases must resolve the exact work pin; AUX and counter use their proper
 * retained aliases. No callback may publish or touch device-owned WB payload.
 * Fresh unsubmitted allocator only: prove zero counter/completion and fresh
 * pool/lists, exclusive slot 1..32 and event record 0, real preceding retirement
 * and visibility, prior-edge identity if present and physical separation of
 * output/scratch/metadata/backing. Opaque pointer/numeric equality alone is not
 * proof. Retained UMA reuse uses its separate displaced-owner API.
 */
typedef int (*gpu_m4_compute_packet_check_admitted)(void *cookie,
	const struct gpu_m4_compute_packet_admitted_input *input,
	const struct gpu_m4_compute_packet_output *output,
	const struct gpu_m4_compute_packet_output *scratch);

struct gpu_m4_compute_packet_admitted_input {
	struct gpu_m4_drm_provider_job *job;
	struct gpu_m4_drm_provider_vm *vm;
	struct gpu_m4_drm_provider_version *version;
	const struct gpu_m4_drm_vm_owner *snapshot;
	const struct gpu_m4_drm_job_compute *command;
	u64 binding_generation; /* Exact completed current-job wide binding identity. */
	/* CDM/USC/normalized attachments are borrowed from that exact decoded
	 * command; check_owner MUST compare them to it, never a current lookup.
	 * Only event_sequence/binding_generation remain zero placeholders.
	 */
	struct gpu_m4_exec_compute_input work;
	struct gpu_m4_compute_packet_private private[GPU_M4_COMPUTE_PRIVATE_COUNT];
	struct gpu_m4_drm_provider_object *counter;
	u64 counter_offset;
	struct gpu_m4_exec_interval firmware[3]; /* work, AUX, counter NC pages */
	u64 notifier_page_fw;
	struct gpu_m4_compute_packet_chronology chronology;
	struct gpu_m4_compute_packet_dependency prior;
	u64 timestamp;
	void *owner_cookie;
	gpu_m4_compute_packet_check_admitted check_owner;
};

static inline bool gpu_m4_compute_packet_separate(const void *a, unsigned long na,
		const void *b, unsigned long nb)
{
	unsigned long va = (unsigned long)a, vb = (unsigned long)b;

	return a && b && na && nb && na <= ~0UL - va && nb <= ~0UL - vb &&
		(va + na <= vb || vb + nb <= va);
}

static inline bool gpu_m4_compute_packet_has_prior(
		const struct gpu_m4_compute_packet_dependency *d)
{
	return d->owner || d->firmware.va || d->firmware.size || d->stamp_fw ||
		d->target || d->event || d->strict;
}

/* A strict record is a fixed owned subrange of the consumer's counter page. */
static inline bool gpu_m4_compute_packet_wait_owned(
		const struct gpu_m4_compute_packet_dependency *d, u64 counter_fw)
{
	return d->strict && counter_fw <= ~0ULL - GPU_M4_COMPUTE_PACKET_WAIT_OFFSET &&
		d->firmware.va == counter_fw + GPU_M4_COMPUTE_PACKET_WAIT_OFFSET &&
		d->firmware.size == GPU_M4_EXEC_DEPENDENCY_SIZE;
}

/* Every physical consumer bounds the whole batch plus its guard entry. The
 * half-window makes both credit and later saved-prefix comparisons unambiguous.
 * No reservation is a producer write or a permission to replace a live owner.
 */
static inline int gpu_m4_compute_packet_ring_reserve(
		const struct gpu_m4_compute_packet_chronology *c, u32 entries, u32 *next)
{
	u32 consumer[4];
	u32 reserved;
	unsigned int i;
	int ret;

	if (!c || !next)
		return -EINVAL;
	consumer[0] = c->queue_read;
	for (i = 0; i < 3; i++)
		consumer[i + 1] = c->queue_consumer[i];
	for (i = 0; i < 4; i++) {
		ret = gpu_m4_exec_ring_reserve(consumer[i], c->queue_write, entries, &reserved);
		if (ret)
			return ret;
		if (((c->queue_write - consumer[i]) & 255) + entries >= 128)
			return -ENOSPC;
	}
	*next = reserved;
	return 0;
}

/* Structural wire/credit validation only. It does not admit allocator reuse:
 * that requires the real bound-reuse receipt/owner proof and restoration.
 */
static inline int gpu_m4_compute_packet_chronology_structure_check(
		const struct gpu_m4_compute_packet_chronology *c,
		const struct gpu_m4_compute_packet_dependency *d,
		const void *owner, u64 timestamp)
{
	u32 entries = gpu_m4_compute_packet_has_prior(d) ? 2 : 1;
	u32 next, target = c->global_previous_target + 0x100;
	int ret;

	if (c->binding_generation > 0xff || c->event_sequence > 0xffffffffULL ||
	    c->threshold_before == ~0ULL || c->submitted_before == ~0ULL)
		return -EOVERFLOW;
	/* Each outstanding predecessor consumed one notifier sequence. */
	if (c->outstanding >= GPU_M4_COMPUTE_PACKET_OUTSTANDING_MAX ||
	    c->outstanding > c->event_sequence)
		return -EINVAL;
	if (!target)
		target = 0x100; /* Global target wrap skips zero, independently of notifier. */
	if (!c->binding_generation || !timestamp || !owner ||
	    !gpu_m4_exec_user_asid(c->asid) || c->uma_slot < 1 || c->uma_slot > 32 ||
	    c->threshold_before != c->event_sequence ||
	    c->threshold_to_publish != c->threshold_before + 1 ||
	    (c->global_previous_target & 0xff) || (c->selected_stamp_before & 0xff) ||
	    c->target_stamp != target ||
	    (c->selected_stamp_before &&
	     !gpu_m4_exec_stamp_reached(c->global_previous_target, c->selected_stamp_before)) ||
	    /* A zero selected stamp is admitted only while EVERY earlier use of
	     * this notifier is an outstanding predecessor of this same job.
	     */
	    (c->event_sequence && (!c->global_previous_target ||
	     (!c->selected_stamp_before && c->event_sequence > c->outstanding) ||
	     (c->selected_stamp_before &&
	      c->target_stamp - c->selected_stamp_before >= 0x80000000U))) ||
	    (!c->event_sequence && c->selected_stamp_before &&
	     c->selected_stamp_before != c->global_previous_target) ||
	    (!c->selected_stamp_before && c->target_stamp >= 0x80000000U))
		return -EINVAL;
	if (entries == 2 && (!d->owner || d->owner == owner ||
	    d->firmware.size < GPU_M4_EXEC_DEPENDENCY_SIZE ||
	    !gpu_m4_exec_fw_range(d->firmware.va, d->firmware.size, 8) ||
	    !gpu_m4_exec_fw_range(d->stamp_fw, 4, 8) || !d->target ||
	    (d->target & 0xff) ||
	    (d->event == 15 && (!gpu_m4_exec_stamp_reached(c->global_previous_target, d->target) ||
	     c->target_stamp - d->target >= 0x80000000U)) ||
	    (d->event != 15 && d->event != 0x71) ||
	    /* A strict compute edge names exactly the immediately preceding
	     * compute target, never an older or numerically guessed one.
	     */
	    (d->strict && d->event == 15 && d->target != c->global_previous_target)))
		return -EINVAL;
	/* No cross-engine numerical ordering is assumed. The mandatory owner
	 * callback proves the exact captured fragment tail and its lifetime.
	 */
	ret = gpu_m4_compute_packet_ring_reserve(c, entries, &next);
	if (ret)
		return ret;
	ret = gpu_m4_exec_pipe_reserve(c->pipe_read, c->pipe_cfi, c->pipe_write, &next);
	if (ret)
		return ret;
	/* Bounded modular prefixes, strictly fewer than 128 outstanding entries. */
	if (((c->pipe_write - c->pipe_read) & 255) + 1 >= 128 ||
	    ((c->pipe_write - c->pipe_cfi) & 255) + 1 >= 128)
		return -ENOSPC;
	return 0;
}

static inline int gpu_m4_compute_packet_chronology_check(
		const struct gpu_m4_compute_packet_chronology *c,
		const struct gpu_m4_compute_packet_dependency *d,
		const void *owner, u64 timestamp)
{
	int ret = gpu_m4_compute_packet_chronology_structure_check(c, d, owner, timestamp);

	if (ret)
		return ret;
	/* Frozen fresh admission never reconstructs a published allocator. */
	return c->submitted_before || c->completed_before || c->submitted != 1 ?
		-EOPNOTSUPP : 0;
}

static inline int gpu_m4_compute_packet_chronology_validate(
		const struct gpu_m4_compute_packet_input *in)
{
	if (!in->check_owner || in->work.event_sequence || in->work.binding_generation)
		return -EINVAL;
	return gpu_m4_compute_packet_chronology_check(&in->chronology, &in->prior,
		in->owner, in->timestamp);
}

static inline int gpu_m4_compute_packet_bindings_validate(
		const struct gpu_m4_compute_packet_input *in)
{
	static const u64 minimum[GPU_M4_COMPUTE_PACKET_RESOURCE_COUNT] = {
		0x4000, 0x4000, 0x4000, 0x4000, 0x40000, 0x20000, 0x4000,
		GPU_M4_EXEC_POOL_SIZE, 0x4000, 0x4000, 0x4000,
	};
	const struct gpu_m4_exec_compute_input *w = &in->work;
	struct gpu_m4_exec_interval firmware[7];
	u64 expected[GPU_M4_COMPUTE_PACKET_RESOURCE_COUNT];
	unsigned int i, count = 6;
	int ret;

	if (!w->vm || !w->vm->check || w->attachment_count > GPU_M4_EXEC_ATTACHMENT_MAX ||
	    (w->attachment_count && !w->attachments) ||
	    !gpu_m4_exec_fw_range(in->notifier_page_fw, GPU_M4_EXEC_NOTIFY_SIZE, 8) ||
	    w->notifier_fw != in->notifier_page_fw + 0x400 ||
	    w->firmware_stamp_fw != in->notifier_page_fw + 0x640 ||
	    w->user_stamp_fw != in->notifier_page_fw + 0x648 ||
	    w->registers.preemption_size != 0x4000 ||
	    w->registers.cdm_end <= w->registers.cdm_base ||
	    w->registers.cdm_end - w->registers.cdm_base > 0x4000 ||
	    !w->registers.usc_base || (w->registers.usc_base & 0xffffffffULL) ||
	    !gpu_m4_exec_disjoint(in->client, GPU_M4_COMPUTE_PACKET_RESOURCE_COUNT))
		return -EINVAL;
	expected[0] = w->work_gpu;
	expected[1] = w->registers.preemption;
	expected[2] = w->registers.cdm_base;
	expected[3] = w->auxiliary_gpu;
	expected[4] = w->context_store_gpu;
	expected[5] = w->pool_state_gpu;
	expected[6] = w->page_list_gpu;
	for (i = 7; i < GPU_M4_COMPUTE_PACKET_RESOURCE_COUNT; i++)
		expected[i] = in->client[i].va;
	for (i = 0; i < GPU_M4_COMPUTE_PACKET_RESOURCE_COUNT; i++) {
		const struct gpu_m4_exec_interval *r = &in->client[i];
		bool write = i != GPU_M4_COMPUTE_PACKET_CDM &&
			i != GPU_M4_COMPUTE_PACKET_SHADER && i != GPU_M4_COMPUTE_PACKET_PROGRAM;

		if (r->va != expected[i] || (r->va & 0x3fff) || (r->size & 0x3fff) ||
		    r->size < minimum[i] ||
		    (i < GPU_M4_COMPUTE_PACKET_SHADER && r->size != minimum[i]))
			return -EINVAL;
		if (i >= GPU_M4_COMPUTE_PACKET_SHADER &&
		    (r->size > 0x100000000ULL || r->va < w->registers.usc_base ||
		     r->va - w->registers.usc_base > 0x100000000ULL - r->size))
			return -EINVAL;
		ret = gpu_m4_exec_vm_range(w->vm, r->va, r->size, write);
		if (ret)
			return ret > 0 ? -EIO : ret;
	}
	if (in->firmware[0].va != w->work_fw ||
	    in->firmware[1].va != w->auxiliary_fw ||
	    in->firmware[2].va != w->counter_fw)
		return -EINVAL;
	for (i = 0; i < 3; i++) {
		if (in->firmware[i].size != 0x4000 ||
		    !gpu_m4_exec_fw_range(in->firmware[i].va, 0x4000, 8))
			return -EINVAL;
		firmware[i] = in->firmware[i];
	}
	firmware[3] = (struct gpu_m4_exec_interval){ w->queue_fw, GPU_M4_EXEC_QUEUE_SIZE };
	firmware[4] = (struct gpu_m4_exec_interval){ w->stats_fw, 0xe10 };
	firmware[5] = (struct gpu_m4_exec_interval){ in->notifier_page_fw, GPU_M4_EXEC_NOTIFY_SIZE };
	if (!gpu_m4_exec_fw_range(w->queue_fw, GPU_M4_EXEC_QUEUE_SIZE, 8) ||
	    !gpu_m4_exec_fw_range(w->stats_fw, 0xe10, 8))
		return -EINVAL;
	if (gpu_m4_compute_packet_has_prior(&in->prior)) {
		if (in->prior.strict && !gpu_m4_compute_packet_wait_owned(&in->prior, w->counter_fw))
			return -EINVAL;
		if (!in->prior.strict)
			firmware[count++] = in->prior.firmware;
	}
	return gpu_m4_exec_disjoint(firmware, count) ? 0 : -EINVAL;
}

/*
 * Pure transactional preparation: output unchanged on error; scratch disposable.
 * Base qualified bytes are shared, not the first/second fixture admission rules.
 * Only chronology, the actual bind byte and exclusively leased UMA slot differ.
 * This never edits Mesa uploads, a notifier, live descriptor, table or producer.
 * Captured-edge encoding/queued prefixes do not qualify mixed concurrency.
 * The initial actual-runtime qualifier is one gen-2 first job, no prior edge.
 *
 * Under the retained owner lease, finish all fallible alias/cache/evidence and
 * final health/recovery checks BEFORE any threshold or producer publication.
 * Publish fresh counter/work/dependency and all NC visibility, then threshold,
 * complete ring entries, queue producer, complete pipe record, pipe producer,
 * kick in the separately qualified order. On possible publication fail-stop
 * and retain all owners. Poll independent NC full retirement, not WB payload;
 * only then cache_retire/invalidate may expose GPU-written WB data. No fences,
 * success observations or hardware qualification are supplied by this header.
 */
/* Shared detached byte grammar; callers MUST complete their separate admission
 * and mandatory owner proof before entry. This helper grants no ownership.
 */
static inline int gpu_m4_compute_packet_encode(
		struct gpu_m4_compute_packet_output *output,
		struct gpu_m4_compute_packet_output *scratch,
		const struct gpu_m4_compute_packet_input *in, u64 binding_generation)
{
	struct gpu_m4_exec_compute_input first;
	struct gpu_m4_exec_pipe_record pipe = { 0 };
	struct gpu_m4_compute_packet_publication *p;
	const struct gpu_m4_compute_packet_chronology *c;
	int ret;

	c = &in->chronology;
	first = in->work;
	first.binding_generation = c->binding_generation;
	/* The first encoder's admission stays frozen; patch only qualified fields. */
	ret = gpu_m4_exec_first_compute(scratch->work, &first);
	if (ret)
		return ret > 0 ? -EIO : ret;
	/* The bound context: work, Start (+0x900) and Finalize (+0xb58) records
	 * and the work's own 0x200-byte notifier record (+0xc00).
	 */
	put_unaligned_le32(c->asid, scratch->work + 0xc);
	put_unaligned_le32(c->asid, scratch->work + 0x900 + 0x2c);
	put_unaligned_le32(c->asid, scratch->work + 0xb58 + 0x14);
	put_unaligned_le32(c->asid, scratch->work + 0xc00 + 0x24);
	put_unaligned_le32(c->target_stamp, scratch->work + 0x810);
	put_unaligned_le32(c->event_sequence, scratch->work + 0x824);
	put_unaligned_le64(c->event_sequence, scratch->work + 0x938);
	put_unaligned_le32(c->target_stamp, scratch->work + 0xb8c);
	put_unaligned_le32(c->uma_slot, scratch->work + 0xe08);
	memset(scratch->prior_dependency, 0, sizeof(scratch->prior_dependency));
	if (gpu_m4_compute_packet_has_prior(&in->prior)) {
		u8 *d = scratch->prior_dependency;

		put_unaligned_le32(4, d);
		put_unaligned_le64(in->prior.stamp_fw, d + 4);
		put_unaligned_le64(in->prior.stamp_fw, d + 0xc);
		put_unaligned_le32(in->prior.target, d + 0x14);
		put_unaligned_le32(in->prior.event, d + 0x20);
		put_unaligned_le32(c->target_stamp, d + 0x24);
		/* Strict prior-command edge +0x3c=1; an ordinary prior-job
		 * dependency keeps 0. FS's own-TA edge is a render-only record.
		 */
		put_unaligned_le32(in->prior.strict && !in->prior.relaxed ? 1 : 0, d + 0x3c);
	}
	put_unaligned_le64(c->submitted, scratch->submitted_counter);
	p = &scratch->publication;
	memset(p, 0, sizeof(*p));
	p->queue_entries = gpu_m4_compute_packet_has_prior(&in->prior) ? 2 : 1;
	ret = gpu_m4_compute_packet_ring_reserve(c, p->queue_entries, &p->queue_next);
	if (ret)
		return ret;
	ret = gpu_m4_exec_pipe_reserve(c->pipe_read, c->pipe_cfi, c->pipe_write, &p->pipe_next);
	if (ret)
		return ret;
	if (p->queue_entries == 2)
		put_unaligned_le64(in->prior.firmware.va, p->ring_entries);
	put_unaligned_le64(in->work.work_fw, p->ring_entries + (p->queue_entries - 1) * 8);
	pipe.timestamp = in->timestamp;
	pipe.queue_info = in->work.queue_fw;
	pipe.queue_producer = p->queue_next;
	pipe.pipe = 2;
	pipe.scheduler_slot = 15;
	pipe.new_queue = c->new_queue;
	ret = gpu_m4_exec_pipe_record(p->pipe_record, &pipe);
	if (ret)
		return ret;
	ret = gpu_m4_exec_pipe_kick(2, &p->kick);
	if (ret)
		return ret;
	p->threshold_fw = in->notifier_page_fw + 0x610;
	p->threshold_before = c->threshold_before;
	p->threshold_value = c->threshold_to_publish;
	p->counter_fw = in->work.counter_fw;
	p->counter_value = c->submitted;
	p->binding_generation = binding_generation;
	p->wire_generation = c->binding_generation;
	p->queue_before = c->queue_write;
	p->pipe_before = c->pipe_write;
	p->uma_slot = c->uma_slot;
	p->event_sequence = c->event_sequence;
	p->target_stamp = c->target_stamp;
	memcpy(output, scratch, sizeof(*output));
	return 0;
}

static inline int gpu_m4_compute_packet_prepare(
		struct gpu_m4_compute_packet_output *output,
		struct gpu_m4_compute_packet_output *scratch,
		const struct gpu_m4_compute_packet_input *in)
{
	const void *metadata[3];
	unsigned long size[3];
	unsigned int i;
	int ret;

	if (!in || !gpu_m4_compute_packet_separate(output, sizeof(*output),
						 scratch, sizeof(*scratch)))
		return -EINVAL;
	metadata[0] = in;
	size[0] = sizeof(*in);
	metadata[1] = in->work.vm;
	size[1] = sizeof(*in->work.vm);
	metadata[2] = in->work.attachments;
	size[2] = in->work.attachment_count * sizeof(*in->work.attachments);
	if (in->work.attachment_count > GPU_M4_EXEC_ATTACHMENT_MAX)
		return -EINVAL;
	for (i = 0; i < 3; i++) {
		if (i == 2 && !size[i])
			continue;
		if (!gpu_m4_compute_packet_separate(output, sizeof(*output), metadata[i], size[i]) ||
		    !gpu_m4_compute_packet_separate(scratch, sizeof(*scratch), metadata[i], size[i]))
			return -EINVAL;
	}
	ret = gpu_m4_compute_packet_chronology_validate(in);
	if (!ret)
		ret = gpu_m4_compute_packet_bindings_validate(in);
	if (!ret)
		ret = in->check_owner(in->owner_cookie, in, output, scratch);
	if (ret)
		return ret > 0 ? -EIO : ret; /* Frozen fixed-profile contract. */
	return gpu_m4_compute_packet_encode(output, scratch, in, in->chronology.binding_generation);
}

/* No shader/fixture selection or private rescue of a rejected user command.
 * The actual command is immutable and borrowed under the held real job.
 * Output is unchanged on failure; scratch is disposable, never live backing.
 */
static inline int gpu_m4_compute_packet_admitted_validate_checked(
		struct gpu_m4_compute_packet_output *output,
		struct gpu_m4_compute_packet_output *scratch,
		const struct gpu_m4_compute_packet_admitted_input *in, bool fresh)
{
	static const u64 minimum[GPU_M4_COMPUTE_PRIVATE_COUNT] = {
		0x4000, 0x4000, 0x4000, 0x40000, 0x20000, 0x4000, GPU_M4_EXEC_POOL_SIZE,
	};
	struct gpu_m4_exec_interval client[GPU_M4_COMPUTE_PRIVATE_COUNT + 1], fw[7];
	const struct gpu_m4_exec_compute_input *w;
	u64 expected[GPU_M4_COMPUTE_PRIVATE_COUNT];
	unsigned int i, count = 6;
	int ret;

	if (!in || !gpu_m4_compute_packet_separate(output, sizeof(*output),
						 scratch, sizeof(*scratch)) ||
	    !gpu_m4_compute_packet_separate(output, sizeof(*output), in, sizeof(*in)) ||
	    !gpu_m4_compute_packet_separate(scratch, sizeof(*scratch), in, sizeof(*in)) ||
	    !in->job || !in->vm || !in->version || !in->snapshot || !in->command ||
	    !in->check_owner || !in->counter || !in->binding_generation ||
	    in->chronology.binding_generation !=
		gpu_m4_drm_provider_binding_wire_generation(in->binding_generation))
		return -EINVAL;
	w = &in->work;
	if (!gpu_m4_compute_packet_separate(output, sizeof(*output), w->vm, sizeof(*w->vm)) ||
	    !gpu_m4_compute_packet_separate(scratch, sizeof(*scratch), w->vm, sizeof(*w->vm)) ||
	    !w->vm || !w->vm->check || w->event_sequence || w->binding_generation ||
	    w->attachment_count > GPU_M4_EXEC_ATTACHMENT_MAX ||
	    (w->attachment_count && !w->attachments) ||
	    !gpu_m4_exec_cdm_span_valid(w->registers.cdm_base, w->registers.cdm_end) ||
	    !w->registers.usc_base || (w->registers.usc_base & 0xffffffffULL) ||
	    w->registers.preemption_size != 0x4000 ||
	    !gpu_m4_exec_fw_range(in->notifier_page_fw, GPU_M4_EXEC_NOTIFY_SIZE, 8) ||
	    w->notifier_fw != in->notifier_page_fw + 0x400 ||
	    w->firmware_stamp_fw != in->notifier_page_fw + 0x640 ||
	    w->user_stamp_fw != in->notifier_page_fw + 0x648 ||
	    (in->counter_offset & 0x3fff) || in->counter_offset > ~0ULL - 0x4000)
		return -EINVAL;
	if (w->attachment_count &&
	    (!gpu_m4_compute_packet_separate(output, sizeof(*output), w->attachments,
		w->attachment_count * sizeof(*w->attachments)) ||
	     !gpu_m4_compute_packet_separate(scratch, sizeof(*scratch), w->attachments,
		w->attachment_count * sizeof(*w->attachments))))
		return -EINVAL;
	ret = fresh ? gpu_m4_compute_packet_chronology_check(&in->chronology, &in->prior,
		in->job, in->timestamp) : gpu_m4_compute_packet_chronology_structure_check(
		&in->chronology, &in->prior, in->job, in->timestamp);
	if (ret)
		return ret;
	expected[0] = w->work_gpu;
	expected[1] = w->registers.preemption;
	expected[2] = w->auxiliary_gpu;
	expected[3] = w->context_store_gpu;
	expected[4] = w->pool_state_gpu;
	expected[5] = w->page_list_gpu;
	expected[6] = in->private[GPU_M4_COMPUTE_PRIVATE_POOL].client.va;
	for (i = 0; i < GPU_M4_COMPUTE_PRIVATE_COUNT; i++) {
		const struct gpu_m4_compute_packet_private *m = &in->private[i];

		if (!m->object || !m->lease || m->client.va != expected[i] ||
		    m->client.size != minimum[i] || (m->client.va & 0x3fff) ||
		    m->client.va < 0x7000000000ULL || m->client.va >= 0x8000000000ULL ||
		    m->client.size > 0x8000000000ULL - m->client.va ||
		    (m->object_offset & 0x3fff) || m->object_offset > ~0ULL - m->client.size ||
		    m->access != GPU_M4_COMPUTE_PRIVATE_RW)
			return -EINVAL;
		client[i] = m->client;
		ret = gpu_m4_exec_vm_range(w->vm, m->client.va, m->client.size, true);
		if (ret)
			return ret > 0 ? -EPROTO : ret;
	}
	client[GPU_M4_COMPUTE_PRIVATE_COUNT] = (struct gpu_m4_exec_interval) {
		w->registers.cdm_base, w->registers.cdm_end - w->registers.cdm_base,
	};
	/* A merged command's owned stream lies inside its own work page. */
	if (w->cdm_owned && !gpu_m4_exec_cdm_owned_valid(w))
		return -EINVAL;
	if (!gpu_m4_exec_disjoint(client, GPU_M4_COMPUTE_PRIVATE_COUNT + !w->cdm_owned))
		return -EINVAL;
	for (i = 0; i < w->attachment_count; i++) {
		const struct gpu_m4_exec_attachment *a = &w->attachments[i];

		/* Decoder already normalized every writable firmware hint to 128B. */
		if ((a->va & 127) || (a->size & 127) || !a->size)
			return -EINVAL;
	}
	if (in->firmware[0].va != w->work_fw || in->firmware[1].va != w->auxiliary_fw ||
	    in->firmware[2].va != w->counter_fw)
		return -EINVAL;
	for (i = 0; i < 3; i++) {
		if (in->firmware[i].size != 0x4000 ||
		    !gpu_m4_exec_fw_range(in->firmware[i].va, 0x4000, 8))
			return -EINVAL;
		fw[i] = in->firmware[i];
	}
	fw[3] = (struct gpu_m4_exec_interval){ w->queue_fw, GPU_M4_EXEC_QUEUE_SIZE };
	fw[4] = (struct gpu_m4_exec_interval){ w->stats_fw, 0xe10 };
	fw[5] = (struct gpu_m4_exec_interval){ in->notifier_page_fw, GPU_M4_EXEC_NOTIFY_SIZE };
	if (!gpu_m4_exec_fw_range(w->queue_fw, GPU_M4_EXEC_QUEUE_SIZE, 8) ||
	    !gpu_m4_exec_fw_range(w->stats_fw, 0xe10, 8))
		return -EINVAL;
	if (gpu_m4_compute_packet_has_prior(&in->prior)) {
		/* Strict: the owned subrange inside fw[2]'s counter page. */
		if (in->prior.strict && !gpu_m4_compute_packet_wait_owned(&in->prior, w->counter_fw))
			return -EINVAL;
		if (!in->prior.strict)
			fw[count++] = in->prior.firmware;
	}
	if (!gpu_m4_exec_disjoint(fw, count))
		return -EINVAL;
	ret = in->check_owner(in->owner_cookie, in, output, scratch);
	return ret > 0 ? -EPROTO : ret;
}

static inline int gpu_m4_compute_packet_admitted_validate(
		struct gpu_m4_compute_packet_output *output,
		struct gpu_m4_compute_packet_output *scratch,
		const struct gpu_m4_compute_packet_admitted_input *in)
{
	return gpu_m4_compute_packet_admitted_validate_checked(output, scratch, in, false);
}

/* The shared validator does not authorize reuse. Fresh admission retains its
 * original counter gate; production reuse additionally requires the actual
 * bound-reuse receipt and complete allocator restoration before publication.
 */
static inline int gpu_m4_compute_packet_prepare_admitted(
		struct gpu_m4_compute_packet_output *output,
		struct gpu_m4_compute_packet_output *scratch,
		const struct gpu_m4_compute_packet_admitted_input *in)
{
	struct gpu_m4_compute_packet_input encoded = { 0 };
	int ret;

	ret = gpu_m4_compute_packet_admitted_validate_checked(output, scratch, in, true);
	if (ret)
		return ret;
	encoded.work = in->work;
	encoded.chronology = in->chronology;
	encoded.prior = in->prior;
	encoded.notifier_page_fw = in->notifier_page_fw;
	encoded.timestamp = in->timestamp;
	ret = gpu_m4_compute_packet_encode(output, scratch, &encoded, in->binding_generation);
	return ret > 0 ? -EPROTO : ret;
}

#endif
