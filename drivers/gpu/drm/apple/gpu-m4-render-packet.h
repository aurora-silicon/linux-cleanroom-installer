/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_RENDER_PACKET_H
#define APPLE_GPU_M4_RENDER_PACKET_H

#include "gpu-m4-render.h"

#define GPU_M4_RENDER_TILER_SEQUENCE_SIZE 0x300U
#define GPU_M4_RENDER_FRAGMENT_SEQUENCE_SIZE 0x380U
#define GPU_M4_RENDER_TIMESTAMP_RECORD_SIZE 0x4cU
#define GPU_M4_RENDER_FRAGMENT_SEQUENCE_CAPACITY \
	(GPU_M4_RENDER_FRAGMENT_SEQUENCE_SIZE + 8 * GPU_M4_RENDER_TIMESTAMP_RECORD_SIZE)

struct gpu_m4_render_engine_chronology {
	/* Host reservation only. Qualified repeated-render wire sequences stay 0. */
	u64 reservation_sequence;
	u64 threshold_before;
	u64 threshold_to_publish;
	u64 counter;
	u32 previous_target;
};

struct gpu_m4_render_prior_dependency {
	/* All zero means absent. The predecessor cell and record remain retained. */
	struct gpu_m4_render_span firmware;
	u64 predecessor_stamp_fw;
	u32 predecessor_target;
	u32 predecessor_event;
	bool fragment_internal;
	/* Strict prior-command edge (+0x3c=1) of a TA waiting on the preceding
	 * command's final firmware stamp; never the own-TA fragment edge.
	 */
	bool strict;
	/* Experiment (barrier_relax bit 1): +0x3c=0 on this strict TA wait, the
	 * native publisher's value; the stamp test and ordinary edge remain.
	 */
	bool relaxed;
};

/*
 * Frozen runtime reservations, independent of VM generation and diagnostic UUID.
 * No field here authorizes rebuilding a live notifier, PB manager or UMA pool.
 */
struct gpu_m4_render_packet_chronology {
	struct gpu_m4_render_engine_chronology engine[2];
	struct gpu_m4_render_prior_dependency prior[2];
	u64 uma_submitted_before;
	u64 uma_completed_before;
	u64 uma_submitted;
	u32 target_stamp; /* One paired TA/FS pass target. */
	u32 parameter_id;
	/* Nonzero: the runtime-wide stage cell of the TA (0x70) / FS (0x71)
	 * held event, named by completion metadata +8 and Finalize instead of
	 * the notifier's private +0x628/+0x638 (gpu_m4_exec_stage_check()).
	 */
	u64 stage_fw[2];
	/* Nonzero: the cell the own-TA wait of the FS names (the TA stage
	 * cell under render_stage_waits); zero: the notifier's +0x620.
	 */
	u64 own_ta_fw;
	/*
	 * False requires exact initialized manager/storage/descriptor/slot/context
	 * leases with no rebuild/growth/rebind and full prior-pair retirement.
	 */
	bool init_buffer;
	/* TA/FS notifier records the works name (W+0x10 / W+0x20) when they
	 * are not the notifier page's own +0/+0x200: cached record cells.
	 */
	u64 record_fw[2];
};

struct gpu_m4_render_packet_input;
/*
 * Prove the frozen owner lease, complete unchanged-Mesa program state,
 * TPC firmware/client same-PA alias, free notifier records and ring credits,
 * and reservation of the fragment's real tiler-stamp dependency. This pure
 * encoder prepares but does not publish that dependency. A live provider must
 * publish it in the qualified order before exposing RunFragment, and keep
 * every object on ambiguous retirement. With explicit chronology, additionally
 * prove both persistent notifier reservations, cumulative UMA state and its
 * retirement/displacement lease in a bounded, disjoint UMA slot table, stage
 * counters, PB slot including manager +0xc/+0x10 ID agreement and the exact
 * initialized manager/storage/descriptor/context/retirement omission rules,
 * and every prior dependency's captured predecessor. A copied count or address
 * cannot prove those facts. With a retained binding, also prove its exact
 * installed lease, version, independent wide epoch and bounded wire generation
 * still belong to this frozen owner. Byte equality is not that proof. Success
 * here is not shader proof.
 */
typedef int (*gpu_m4_render_check_packet)(void *cookie,
		const struct gpu_m4_render_packet_input *packet);

struct gpu_m4_render_packet_input {
	const struct gpu_m4_render_first_owner_input *owner;
	struct gpu_m4_render_span tpc_firmware;
	struct gpu_m4_render_span init_buffer_firmware;
	struct gpu_m4_render_span fragment_dependency_firmware;
	struct gpu_m4_render_program partial_background;
	struct gpu_m4_render_program partial_end_of_tile;
	/* NULL retains the exact fresh first-owner wire construction. */
	const struct gpu_m4_render_packet_chronology *chronology;
	/* NULL keeps the exact gen1 entrance. Never derive generation from epoch. */
	const struct gpu_m4_render_retained_binding *binding;
	void *owner_cookie;
	gpu_m4_render_check_packet check_owner;
};

/* Both structs are heap-owned scratch/output, never kernel-stack allocations. */
struct gpu_m4_render_packet_scratch {
	struct gpu_m4_render_register_output registers[2];
	struct gpu_m4_render_timestamp_output timestamps[2];
	u8 register_metadata[2][12];
	u8 init_buffer[GPU_M4_RENDER_INIT_BUFFER_SIZE];
	u8 fragment_dependency[GPU_M4_EXEC_DEPENDENCY_SIZE];
	u64 partial_background;
	u64 partial_end_of_tile;
};

struct gpu_m4_render_packet_output {
	u8 init_buffer[GPU_M4_RENDER_INIT_BUFFER_SIZE];
	u8 fragment_dependency[GPU_M4_EXEC_DEPENDENCY_SIZE];
	u8 tiler[GPU_M4_RENDER_TILER_SIZE];
	u8 fragment[GPU_M4_RENDER_FRAGMENT_SIZE];
	u8 tiler_sequence[GPU_M4_RENDER_TILER_SEQUENCE_SIZE];
	u8 fragment_sequence[GPU_M4_RENDER_FRAGMENT_SEQUENCE_CAPACITY];
	/* Additional private runtime bytes; the original packet arrays stay frozen. */
	u8 prior_dependency[2][GPU_M4_EXEC_DEPENDENCY_SIZE];
	u8 uma_submitted_counter[8]; /* Scene+0x500; zero for the legacy fresh wrapper. */
	/* Native transient metadata image. Runtime applies it only to the actual
	 * leased scene, never to live persistent allocator/descriptor storage.
	 */
	u8 scene[GPU_M4_RENDER_SCENE_SIZE];
};

/* Normalized literal operands, not a fixture-validation escape hatch. Both
 * callers validate their distinct exact owners before reaching these emitters.
 * No VS/FS/COLOR role or fabricated first-owner input appears in this type.
 */
struct gpu_m4_render_packet_encoded {
	const struct gpu_m4_render_encoded *render;
	const struct gpu_m4_render_span *firmware, *gpu;
	struct gpu_m4_render_span tpc_firmware;
	const struct gpu_m4_render_packet_chronology *chronology;
	u64 partial_background_resources, partial_end_of_tile_resources;
};

/* Scalar chronology grammar shared by independently validated entrances. */
static inline int gpu_m4_render_chronology_fields_validate(
		const struct gpu_m4_render_packet_chronology *c,
		u32 uma_slot, u32 parameter_id)
{
	unsigned int i;

	if (!c)
		return 0;
	if (uma_slot < GPU_M4_RENDER_UMA_SLOT_FIRST ||
	    uma_slot - GPU_M4_RENDER_UMA_SLOT_FIRST >= GPU_M4_RENDER_UMA_SLOT_COUNT)
		return -EINVAL;
	if (!c->target_stamp || (c->target_stamp & 0xff) ||
	    c->parameter_id >= GPU_M4_RENDER_PARAMETER_SLOT_COUNT ||
	    c->parameter_id != parameter_id)
		return -EINVAL;
	if (c->uma_submitted_before > ~0ULL - 2 || c->engine[1].counter > ~0ULL - 2)
		return -EOVERFLOW;
	if ((c->uma_submitted_before & 1) ||
	    c->uma_submitted != c->uma_submitted_before + 2 ||
	    !(c->engine[1].counter & 1) || c->engine[0].counter != c->engine[1].counter + 1)
		return -EINVAL;
	if (c->uma_completed_before != c->uma_submitted_before)
		return -EBUSY;
	for (i = 0; i < 2; i++) {
		const struct gpu_m4_render_engine_chronology *e = &c->engine[i];
		const struct gpu_m4_render_prior_dependency *d = &c->prior[i];

		if (e->reservation_sequence == ~0ULL)
			return -EOVERFLOW;
		if (e->threshold_before != e->reservation_sequence ||
		    e->threshold_to_publish != e->reservation_sequence + 1 ||
		    (e->previous_target & 0xff) || e->previous_target >= c->target_stamp ||
		    (e->reservation_sequence && !e->previous_target))
			return -EINVAL;
		if (c->record_fw[i] && !gpu_m4_exec_fw_range(c->record_fw[i], 0x118, 0x40))
			return -EINVAL;
		if (!d->firmware.va && !d->firmware.size && !d->predecessor_stamp_fw &&
		    !d->predecessor_target && !d->predecessor_event && !d->fragment_internal &&
		    !d->strict)
			continue;
		if (d->firmware.size < GPU_M4_EXEC_DEPENDENCY_SIZE ||
		    !gpu_m4_exec_fw_range(d->firmware.va, d->firmware.size, 8) ||
		    !gpu_m4_exec_fw_range(d->predecessor_stamp_fw, 4, 8) ||
		    !d->predecessor_target || (d->predecessor_target & 0xff) ||
		    /* Compute targets come from another engine's allocator. */
		    (d->predecessor_event != 15 && d->predecessor_target >= c->target_stamp) ||
		    (d->predecessor_event != 15 && d->predecessor_event != 0x71 &&
		     d->predecessor_event != 0x70) ||
		    (d->predecessor_event == 0x70 && (i || !d->strict)) ||
		    (d->fragment_internal && (!i || d->predecessor_event != 0x71)) ||
		    (d->strict && (i || d->fragment_internal)))
			return -EINVAL;
	}
	/* Early tiling (TA waits on the previous TA) needs the FS strict wait on
	 * the previous final engine.
	 */
	if (c->prior[0].predecessor_event == 0x70 && !c->prior[1].fragment_internal)
		return -EINVAL;
	return 0;
}

static inline int gpu_m4_render_chronology_validate(
		const struct gpu_m4_render_packet_input *in)
{
	if (!in || !in->owner || !in->owner->render)
		return -EINVAL;
	return gpu_m4_render_chronology_fields_validate(in->chronology,
		in->owner->uma_slot, in->owner->render->owner.parameter_id);
}

static inline int gpu_m4_render_packet_bindings_validate(
		const struct gpu_m4_render_packet_input *in)
{
	struct gpu_m4_exec_interval firmware[GPU_M4_RENDER_FIRST_FW_COUNT + 7];
	const struct gpu_m4_render_first_owner_input *owner;
	const struct gpu_m4_render_input *render;
	unsigned int i;
	int ret;

	if (!in || !in->owner || !in->check_owner)
		return -EINVAL;
	owner = in->owner;
	if (in->binding) {
		ret = gpu_m4_render_owner_bindings_validate_retained(owner, in->binding);
		if (!ret && !in->chronology) {
			/* A rebound fresh pair retains every original fresh-owner lease. */
			if (!owner->check_owner || owner->uma_slot != GPU_M4_RENDER_FIRST_UMA_SLOT ||
			    owner->render->owner.parameter_id != 0)
				return -EINVAL;
			ret = owner->check_owner(owner->owner_cookie, owner);
		}
	} else {
		ret = in->chronology ? gpu_m4_render_owner_bindings_validate(owner) :
			gpu_m4_render_first_owner_validate(owner);
	}
	if (ret)
		return ret;
	ret = gpu_m4_render_chronology_validate(in);
	if (ret)
		return ret;
	render = owner->render;
	if (in->tpc_firmware.size < render->resource[GPU_M4_RENDER_TPC].size ||
	    !gpu_m4_exec_fw_range(in->tpc_firmware.va, in->tpc_firmware.size, 8) ||
	    in->init_buffer_firmware.size < GPU_M4_RENDER_INIT_BUFFER_SIZE ||
	    !gpu_m4_exec_fw_range(in->init_buffer_firmware.va, in->init_buffer_firmware.size, 8) ||
	    in->fragment_dependency_firmware.size < GPU_M4_EXEC_DEPENDENCY_SIZE ||
	    !gpu_m4_exec_fw_range(in->fragment_dependency_firmware.va,
				 in->fragment_dependency_firmware.size, 8))
		return -EINVAL;
	for (i = 0; i < GPU_M4_RENDER_FIRST_FW_COUNT; i++)
		firmware[i] = (struct gpu_m4_exec_interval){ owner->firmware[i].va, owner->firmware[i].size };
	firmware[i++] = (struct gpu_m4_exec_interval){ in->tpc_firmware.va, in->tpc_firmware.size };
	firmware[i++] = (struct gpu_m4_exec_interval){ in->init_buffer_firmware.va,
		in->init_buffer_firmware.size };
	firmware[i++] = (struct gpu_m4_exec_interval){ in->fragment_dependency_firmware.va,
		in->fragment_dependency_firmware.size };
	firmware[i++] = (struct gpu_m4_exec_interval){ render->owner.parameter_manager_fw, 0xc0 };
	firmware[i++] = (struct gpu_m4_exec_interval){ render->owner.parameter_descriptor_fw, 16 };
	if (in->chronology) {
		unsigned int stage;

		for (stage = 0; stage < 2; stage++)
			if (in->chronology->prior[stage].firmware.va)
				firmware[i++] = (struct gpu_m4_exec_interval){
					in->chronology->prior[stage].firmware.va,
					in->chronology->prior[stage].firmware.size };
	}
	return gpu_m4_exec_disjoint(firmware, i) ? 0 : -EINVAL;
}

/* All callers validate the frozen dependency before this nonfallible emission. */
static inline void gpu_m4_render_dependency_emit(u8 *output, u64 stamp_fw,
		u32 predecessor_target, u32 event, u32 consuming_target, bool internal)
{
	memset(output, 0, GPU_M4_EXEC_DEPENDENCY_SIZE);
	put_unaligned_le32(4, output);
	put_unaligned_le64(stamp_fw, output + 4);
	put_unaligned_le64(stamp_fw, output + 0xc);
	put_unaligned_le32(predecessor_target, output + 0x14);
	put_unaligned_le32(event, output + 0x20);
	put_unaligned_le32(consuming_target, output + 0x24);
	put_unaligned_le32(internal, output + 0x3c);
}

/* A real TA-to-fragment edge, not the queue-registration zero-stamp barrier. */
static inline int gpu_m4_render_fragment_dependency(void *output, u64 notifier_fw)
{
	if (!output || !gpu_m4_exec_fw_range(notifier_fw, GPU_M4_EXEC_NOTIFY_SIZE, 8))
		return -EINVAL;
	gpu_m4_render_dependency_emit(output, notifier_fw + 0x620, 0x100, 0x70, 0x100, false);
	/* +0x3c=0 is this partial-fragment-capable dependency, not a strict edge. */
	return 0;
}

/* The stage (Finalize) destination of a pair's engine. */
static inline u64 gpu_m4_render_packet_stage(u64 notifier, u32 engine,
		const struct gpu_m4_render_packet_chronology *c)
{
	return c && c->stage_fw[engine] ? c->stage_fw[engine] : notifier + 0x628 + engine * 16;
}

static inline void gpu_m4_render_packet_meta(u8 *meta, u64 notifier, u32 engine,
		u32 uuid, const struct gpu_m4_render_packet_chronology *c)
{
	put_unaligned_le64(notifier + 0x620 + engine * 16, meta);
	put_unaligned_le64(gpu_m4_render_packet_stage(notifier, engine, c), meta + 8);
	put_unaligned_le32(c ? c->target_stamp : 0x100, meta + 0x10);
	put_unaligned_le32(engine ? 0x71 : 0x70, meta + 0x14);
	put_unaligned_le32(uuid, meta + 0x20);
	/* Host reservation sequences advance; qualified render wire remains zero. */
	put_unaligned_le32(0, meta + 0x24);
	/* Qualified G16 host-stamp flush control, including the first packet. */
	put_unaligned_le32(1, meta + 0x1c);
}

static inline void gpu_m4_render_packet_inline_uma(u8 *header,
		u64 descriptor_fw, u64 descriptor_gpu, u32 engine,
		const struct gpu_m4_render_packet_chronology *c)
{
	/* Exactly 0x3c bytes; the following timestamp Start must not be touched. */
	memset(header, 0, 0x3c);
	put_unaligned_le64(descriptor_fw, header);
	put_unaligned_le64(descriptor_gpu + (engine ? 0x200 : 0x100), header + 0x18);
	put_unaligned_le64(c ? c->engine[engine].counter : (engine ? 1 : 2), header + 0x28);
	put_unaligned_le32(engine ? 0x71 : 0x70, header + 0x30);
	if (engine)
		header[0x34] = 1;
}

/* Nonfallible emission follows a complete frozen-owner preflight. */
static inline void gpu_m4_render_packet_tiler(struct gpu_m4_render_packet_output *out,
		const struct gpu_m4_render_packet_encoded *in,
		const struct gpu_m4_render_packet_scratch *scratch)
{
	const struct gpu_m4_render_encoded *r = in->render;
	const struct gpu_m4_render_packet_chronology *c = in->chronology;
	const struct gpu_m4_render_span *fw = in->firmware, *gpu = in->gpu;
	u64 command = fw[GPU_M4_RENDER_FIRST_TILER_WORK_FW].va;
	u64 sequence = fw[GPU_M4_RENDER_FIRST_TILER_SEQUENCE_FW].va;
	u64 scene = fw[GPU_M4_RENDER_FIRST_SCENE_FW].va;
	u64 queue = fw[GPU_M4_RENDER_FIRST_TILER_QUEUE_FW].va;
	u64 stats = fw[GPU_M4_RENDER_FIRST_TILER_ACTIVITY_FW].va + 4;
	u64 notifier = fw[GPU_M4_RENDER_FIRST_NOTIFIERS_FW].va;
	u64 manager = r->owner.parameter_manager_fw;
	u8 *work = out->tiler, *start = out->tiler_sequence, *finish = start + 0x268;

	put_unaligned_le64(c ? c->engine[0].counter : 2, work + 4);
	put_unaligned_le32(r->owner.asid, work + 0xc);
	put_unaligned_le64(c && c->record_fw[0] ? c->record_fw[0] : notifier, work + 0x10);
	put_unaligned_le32(c ? c->parameter_id : 0, work + 0x18);
	put_unaligned_le64(manager, work + 0x20);
	put_unaligned_le64(scene, work + 0x28);
	put_unaligned_le64(scene + 0x100, work + 0x30);
	memcpy(work + 0x40, scratch->registers[0].registers, GPU_M4_RENDER_REGISTER_SIZE);
	memcpy(work + 0x740, scratch->register_metadata[0], 12);
	put_unaligned_le64(in->tpc_firmware.va, work + 0x760);
	put_unaligned_le64(scratch->registers[0].geometry.tpc_size, work + 0x768);
	put_unaligned_le64(sequence, work + 0x770);
	put_unaligned_le32(GPU_M4_RENDER_TILER_SEQUENCE_SIZE, work + 0x778);
	put_unaligned_le32(0x71, work + 0x77c);
	put_unaligned_le32(c ? c->target_stamp : 0x100, work + 0x780);
	put_unaligned_le64(r->resource[GPU_M4_RENDER_PREEMPT_0].va -
		GPU_M4_RENDER_RANGE1_START, work + 0x7a8);
	/* Corrected production contract: mirrors configuration selector 1c898,
	 * NOT the independent dynamic layer-control selector 10169.
	 */
	put_unaligned_le64(0, work + 0x830);
	gpu_m4_render_packet_meta(work + 0x878, notifier, 0, r->uuid, c);
	memcpy(work + 0x8c8, scratch->timestamps[0].internal_pair, 16);
	put_unaligned_le64(fw[GPU_M4_RENDER_FIRST_UMA_FW].va, work + 0x8fe);
	work[0x906] = 1;
	put_unaligned_le64(fw[GPU_M4_RENDER_FIRST_UMA_FW].va + 0x100, work + 0x90f);
	work[0x917] = r->owner.generation;

	put_unaligned_le32(5, start);
	put_unaligned_le64(command + 0x40, start + 0x14);
	put_unaligned_le64(manager, start + 0x1c);
	put_unaligned_le64(scene, start + 0x24);
	put_unaligned_le64(stats, start + 0x2c);
	put_unaligned_le64(queue, start + 0x34);
	put_unaligned_le64(command + 0x85c, start + 0x3c);
	put_unaligned_le32(r->owner.asid, start + 0x44);
	put_unaligned_le32(1, start + 0x48);
	put_unaligned_le32(c ? c->parameter_id : 0, start + 0x50);
	put_unaligned_le64(0, start + 0x54); /* +0x58 is the high word, not a pass ID. */
	put_unaligned_le64(command + 0x784, start + 0x64);
	put_unaligned_le64(command + 0x8a8, start + 0x6c);
	put_unaligned_le32(r->uuid, start + 0x7c);
	memcpy(start + 0x88, scratch->registers[0].attachments, GPU_M4_EXEC_ATTACHMENT_SIZE);
	gpu_m4_render_packet_inline_uma(start + 0x190,
		fw[GPU_M4_RENDER_FIRST_UMA_FW].va, gpu[GPU_M4_RENDER_FIRST_UMA_GPU].va, 0, c);
	memcpy(start + 0x1cc, scratch->timestamps[0].start, 0x4c);
	memcpy(start + 0x218, scratch->timestamps[0].wait, 4);
	memcpy(start + 0x21c, scratch->timestamps[0].end, 0x4c);
	put_unaligned_le32(6, finish);
	put_unaligned_le64(scene, finish + 4);
	put_unaligned_le64(manager, finish + 0xc);
	put_unaligned_le64(stats, finish + 0x14);
	put_unaligned_le64(queue, finish + 0x1c);
	put_unaligned_le64(command + 0x85c, finish + 0x24);
	put_unaligned_le32(r->owner.asid, finish + 0x2c);
	put_unaligned_le64(command + 0x784, finish + 0x34);
	put_unaligned_le32(r->uuid, finish + 0x40);
	put_unaligned_le64(gpu_m4_render_packet_stage(notifier, 0, c), finish + 0x48);
	put_unaligned_le32(c ? c->target_stamp : 0x100, finish + 0x50);
	put_unaligned_le64(sequence + 0x190, finish + 0x78);
	put_unaligned_le32(0U - 0x268, finish + 0x80);
	finish[0x84] = r->attachment_count[0] != 0;
	put_unaligned_le64(command + 0x918, finish + 0x85);
	finish[0x8d] = 1;
	put_unaligned_le32(0x40000002, finish + 0x94);
}

static inline void gpu_m4_render_packet_fragment(struct gpu_m4_render_packet_output *out,
		const struct gpu_m4_render_packet_encoded *in,
		const struct gpu_m4_render_packet_scratch *scratch)
{
	const struct gpu_m4_render_encoded *r = in->render;
	const struct gpu_m4_render_packet_chronology *c = in->chronology;
	const struct gpu_m4_render_span *fw = in->firmware, *gpu = in->gpu;
	const struct gpu_m4_exec_geometry *g = &scratch->registers[1].geometry;
	u64 command = fw[GPU_M4_RENDER_FIRST_FRAGMENT_WORK_FW].va;
	u64 sequence = fw[GPU_M4_RENDER_FIRST_FRAGMENT_SEQUENCE_FW].va;
	u64 scene = fw[GPU_M4_RENDER_FIRST_SCENE_FW].va;
	u64 queue = fw[GPU_M4_RENDER_FIRST_FRAGMENT_QUEUE_FW].va;
	u64 stats = fw[GPU_M4_RENDER_FIRST_FRAGMENT_ACTIVITY_FW].va + 8;
	u64 notifier = fw[GPU_M4_RENDER_FIRST_NOTIFIERS_FW].va;
	u64 manager = r->owner.parameter_manager_fw;
	u8 *work = out->fragment, *start = out->fragment_sequence, *finish = start + 0x288;

	put_unaligned_le32(1, work);
	put_unaligned_le64(c ? c->engine[1].counter : 1, work + 4);
	put_unaligned_le32(r->owner.asid, work + 0xc);
	put_unaligned_le64(sequence, work + 0x14);
	put_unaligned_le32(GPU_M4_RENDER_FRAGMENT_SEQUENCE_SIZE, work + 0x1c);
	put_unaligned_le64(c && c->record_fw[1] ? c->record_fw[1] : notifier + 0x200, work + 0x20);
	put_unaligned_le64(manager, work + 0x28);
	put_unaligned_le64(scene, work + 0x30);
	put_unaligned_le64(scene + 0x100, work + 0x38);
	put_unaligned_le64(r->resource[GPU_M4_RENDER_TILEMAP].va, work + 0x40);
	put_unaligned_le32(1, work + 0x50);
	put_unaligned_le32(g->isp_blocks, work + 0x54);
	put_unaligned_le32(g->x_blocks, work + 0x60);
	put_unaligned_le32(g->y_blocks, work + 0x64);
	put_unaligned_le32(g->blocks, work + 0x68);
	/* Screen tiles are distinct from the padded tilemap/TPC entry count.
	 * Geometry has already checked the 1..16384 dimension bounds.
	 */
	put_unaligned_le32(((r->geometry.width + 31) / 32) *
		((r->geometry.height + 31) / 32), work + 0x78);
	memcpy(work + 0x80, scratch->registers[1].registers, GPU_M4_RENDER_REGISTER_SIZE);
	memcpy(work + 0x780, scratch->register_metadata[1], 12);
	put_unaligned_le64(r->depth_bias.va, work + 0x7a0);
	put_unaligned_le64(r->scissor.va, work + 0x7b0);
	put_unaligned_le64(r->query.va, work + 0x7c0);
	put_unaligned_le64(r->background_resources, work + 0x8f8);
	put_unaligned_le64(r->background_program, work + 0x900);
	put_unaligned_le64(in->partial_background_resources, work + 0x928);
	put_unaligned_le64(scratch->partial_background, work + 0x930);
	put_unaligned_le64(r->zls_control, work + 0x938);
	put_unaligned_le64(0x04040404, work + 0x940);
	put_unaligned_le64(r->depth.va, work + 0x948);
	put_unaligned_le64(r->depth_stride, work + 0x950);
	put_unaligned_le64(r->depth.va, work + 0x960);
	put_unaligned_le64(r->depth.va, work + 0x968);
	put_unaligned_le64(r->stencil.va, work + 0x978);
	put_unaligned_le64(r->stencil_stride, work + 0x980);
	put_unaligned_le64(r->stencil.va, work + 0x990);
	put_unaligned_le64(r->stencil.va, work + 0x998);
	put_unaligned_le64(g->tilebuffer_control, work + 0x9b8);
	put_unaligned_le64(0xc000 | (r->depth_bias_is_integer ? 1ULL << 18 : 0), work + 0x9c0);
	put_unaligned_le64(((u64)r->geometry.height << 32) | r->geometry.width, work + 0x9c8);
	put_unaligned_le64(0x100000, work + 0x9d0);
	put_unaligned_le64(gpu_m4_render_encoded_isp_mode(r), work + 0x9d8);
	put_unaligned_le64(r->end_of_tile_resources, work + 0x9f4);
	put_unaligned_le64(r->end_of_tile_program, work + 0x9fc);
	put_unaligned_le64(in->partial_end_of_tile_resources, work + 0xa14);
	put_unaligned_le64(scratch->partial_end_of_tile, work + 0xa1c);
	put_unaligned_le64(((u64)r->background_value << 32) | r->depth_clear, work + 0xa28);
	put_unaligned_le64(r->zls_pixels, work + 0xa50);
	gpu_m4_render_packet_meta(work + 0xbc0, notifier, 1, r->uuid, c);
	memcpy(work + 0xc10, scratch->timestamps[1].internal_pair, 16);
	put_unaligned_le64(fw[GPU_M4_RENDER_FIRST_UMA_FW].va, work + 0xc46);
	work[0xc4e] = 1;
	put_unaligned_le64(fw[GPU_M4_RENDER_FIRST_UMA_FW].va + 0x200, work + 0xc57);
	work[0xc5f] = r->owner.generation;
	work[0xc84] = 1;

	put_unaligned_le32(7, start);
	put_unaligned_le64(command + 0x80, start + 0x14);
	put_unaligned_le64(scene, start + 0x1c);
	put_unaligned_le64(stats, start + 0x24);
	put_unaligned_le64(command + 0xb70, start + 0x2c);
	put_unaligned_le64(scene + 0x54, start + 0x34);
	put_unaligned_le64(command + 0xbb0, start + 0x3c);
	put_unaligned_le64(command + 0xbb4, start + 0x44);
	put_unaligned_le64(queue, start + 0x4c);
	put_unaligned_le64(command, start + 0x54);
	put_unaligned_le32(r->owner.asid, start + 0x5c);
	put_unaligned_le32(1, start + 0x60);
	put_unaligned_le32(c ? c->parameter_id : 0, start + 0x68);
	put_unaligned_le64(0, start + 0x6c); /* +0x70 is the high word, not a pass ID. */
	put_unaligned_le64(command + 0xa58, start + 0x7c);
	put_unaligned_le64(command + 0xbf0, start + 0x84);
	put_unaligned_le32(r->uuid, start + 0xa0);
	memcpy(start + 0xa8, scratch->registers[1].attachments, GPU_M4_EXEC_ATTACHMENT_SIZE);
	gpu_m4_render_packet_inline_uma(start + 0x1b0,
		fw[GPU_M4_RENDER_FIRST_UMA_FW].va, gpu[GPU_M4_RENDER_FIRST_UMA_GPU].va, 1, c);
	memcpy(start + 0x1ec, scratch->timestamps[1].start, 0x4c);
	memcpy(start + 0x238, scratch->timestamps[1].wait, 4);
	memcpy(start + 0x23c, scratch->timestamps[1].end, 0x4c);
	put_unaligned_le32(8, finish);
	put_unaligned_le32(r->uuid, finish + 4);
	put_unaligned_le64(gpu_m4_render_packet_stage(notifier, 1, c), finish + 0xc);
	put_unaligned_le32(c ? c->target_stamp : 0x100, finish + 0x14);
	put_unaligned_le64(scene, finish + 0x1c);
	put_unaligned_le64(manager, finish + 0x24);
	put_unaligned_le32(1, finish + 0x2c);
	put_unaligned_le64(stats, finish + 0x30);
	put_unaligned_le64(command + 0xbb0, finish + 0x38);
	put_unaligned_le64(command + 0xbb4, finish + 0x40);
	put_unaligned_le64(command + 0xb70, finish + 0x48);
	put_unaligned_le64(queue, finish + 0x50);
	put_unaligned_le64(command, finish + 0x58);
	put_unaligned_le32(r->owner.asid, finish + 0x60);
	put_unaligned_le64(command + 0xa58, finish + 0x64);
	put_unaligned_le64(sequence + 0x1b0, finish + 0x98);
	put_unaligned_le32(0U - 0x288, finish + 0xa0);
	finish[0xa4] = r->attachment_count[1] != 0;
	put_unaligned_le64(command + 0xc60, finish + 0xa5);
	put_unaligned_le32(1, finish + 0xad);
	finish[0xb1] = 1;
	put_unaligned_le32(0x71, finish + 0xb2);
	put_unaligned_le32(0x40000002, finish + 0xbc);
}

/*
 * Encodes RunTiler/RunFragment and separate sequences only. It does not build
 * Mesa linked shaders/PBE/PPP, publish notifier thresholds or rings, write the
 * global PB/UMA tables, claim a slot, publish a fragment wait, or signal a fence.
 * Failure leaves output byte-exact; scratch may change during private preflight.
 */
static inline int gpu_m4_render_packet_prepare(struct gpu_m4_render_packet_output *output,
		struct gpu_m4_render_packet_scratch *scratch,
		const struct gpu_m4_render_packet_input *in)
{
	struct gpu_m4_exec_interval scratch_ranges[7];
	const struct gpu_m4_render_first_owner_input *owner;
	const struct gpu_m4_render_input *render;
	struct gpu_m4_render_encoded encoded;
	struct gpu_m4_render_packet_encoded packet;
	unsigned int i, count = 5;
	int ret;

	if (!output || !scratch || !in || !in->owner || !in->owner->render || !in->check_owner)
		return -EINVAL;
	scratch_ranges[0] = (struct gpu_m4_exec_interval){ (unsigned long)output, sizeof(*output) };
	scratch_ranges[1] = (struct gpu_m4_exec_interval){ (unsigned long)scratch, sizeof(*scratch) };
	scratch_ranges[2] = (struct gpu_m4_exec_interval){ (unsigned long)in, sizeof(*in) };
	scratch_ranges[3] = (struct gpu_m4_exec_interval){ (unsigned long)in->owner, sizeof(*in->owner) };
	scratch_ranges[4] = (struct gpu_m4_exec_interval){ (unsigned long)in->owner->render,
		sizeof(*in->owner->render) };
	if (in->chronology)
		scratch_ranges[count++] = (struct gpu_m4_exec_interval){
			(unsigned long)in->chronology, sizeof(*in->chronology) };
	if (in->binding)
		scratch_ranges[count++] = (struct gpu_m4_exec_interval){
			(unsigned long)in->binding, sizeof(*in->binding) };
	if (!gpu_m4_exec_disjoint(scratch_ranges, count))
		return -EINVAL;
	owner = in->owner;
	ret = gpu_m4_render_packet_bindings_validate(in);
	if (ret)
		return ret;
	render = owner->render;
	ret = gpu_m4_render_program(render, &in->partial_background, &scratch->partial_background);
	if (ret)
		return ret;
	ret = gpu_m4_render_program(render, &in->partial_end_of_tile, &scratch->partial_end_of_tile);
	if (ret)
		return ret;
	for (i = 0; i < 2; i++) {
		unsigned int work_fw = i ? GPU_M4_RENDER_FIRST_FRAGMENT_WORK_FW : GPU_M4_RENDER_FIRST_TILER_WORK_FW;
		unsigned int work_gpu = i ? GPU_M4_RENDER_FIRST_FRAGMENT_WORK_GPU : GPU_M4_RENDER_FIRST_TILER_WORK_GPU;
		unsigned int queue_fw = i ? GPU_M4_RENDER_FIRST_FRAGMENT_QUEUE_FW : GPU_M4_RENDER_FIRST_TILER_QUEUE_FW;

		ret = gpu_m4_render_registers(&scratch->registers[i], render, i);
		if (ret)
			return ret;
		ret = gpu_m4_render_register_metadata(scratch->register_metadata[i], render->vm,
			owner->gpu[work_gpu].va + (i ? 0x80 : 0x40), &scratch->registers[i], i);
		if (ret)
			return ret;
		ret = gpu_m4_render_timestamps(&scratch->timestamps[i], owner->firmware[work_fw].va,
			owner->firmware[queue_fw].va, &owner->firmware[GPU_M4_RENDER_FIRST_SCENE_FW], render->uuid, i);
		if (ret)
			return ret;
	}
	ret = gpu_m4_render_init_buffer(scratch->init_buffer, &render->owner,
		render->owner_cookie, render->check_owner,
		in->chronology ? in->chronology->target_stamp : 0x100);
	if (ret)
		return ret;
	ret = gpu_m4_render_fragment_dependency(scratch->fragment_dependency,
		owner->firmware[GPU_M4_RENDER_FIRST_NOTIFIERS_FW].va);
	if (ret)
		return ret;
	ret = in->check_owner(in->owner_cookie, in);
	if (ret)
		return ret;
	gpu_m4_render_encoded_fixture(&encoded, render);
	packet = (struct gpu_m4_render_packet_encoded) {
		.render = &encoded, .firmware = owner->firmware, .gpu = owner->gpu,
		.tpc_firmware = in->tpc_firmware, .chronology = in->chronology,
		.partial_background_resources = in->partial_background.resources,
		.partial_end_of_tile_resources = in->partial_end_of_tile.resources,
	};
	memset(output, 0, sizeof(*output));
	memcpy(output->init_buffer, scratch->init_buffer, sizeof(output->init_buffer));
	memcpy(output->fragment_dependency, scratch->fragment_dependency,
		sizeof(output->fragment_dependency));
	gpu_m4_render_packet_tiler(output, &packet, scratch);
	gpu_m4_render_packet_fragment(output, &packet, scratch);
	if (in->chronology) {
		const struct gpu_m4_render_packet_chronology *c = in->chronology;

		if (!c->init_buffer)
			memset(output->init_buffer, 0, sizeof(output->init_buffer));
		gpu_m4_render_dependency_emit(output->fragment_dependency,
			c->own_ta_fw ? c->own_ta_fw :
			owner->firmware[GPU_M4_RENDER_FIRST_NOTIFIERS_FW].va + 0x620,
			c->target_stamp, 0x70, c->target_stamp, false);
		for (i = 0; i < 2; i++) {
			const struct gpu_m4_render_prior_dependency *d = &c->prior[i];

			if (d->firmware.va)
				gpu_m4_render_dependency_emit(output->prior_dependency[i],
					d->predecessor_stamp_fw, d->predecessor_target,
					d->predecessor_event, c->target_stamp,
					(d->strict || d->fragment_internal) && !d->relaxed);
		}
		put_unaligned_le64(c->uma_submitted, output->uma_submitted_counter);
	}
	return 0;
}

struct gpu_m4_render_queue_snapshot {
	u32 read, write, consumer[3], busy, capacity;
	u32 pipe_read, pipe_cfi, pipe_write, pipe_capacity;
	u64 threshold;
	u32 firmware_stamp, user_stamp;
	u8 scheduler_slot; /* Provider lease; first profile must match stage 0x70/0x71. */
	bool new_queue;
};

enum gpu_m4_render_publication_step {
	GPU_M4_RENDER_PREPARE_ALL_OWNER_AND_RING_BYTES,
	GPU_M4_RENDER_PUBLISH_BOTH_THRESHOLDS,
	GPU_M4_RENDER_NC_PUBLICATION_BARRIER,
	GPU_M4_RENDER_PUBLISH_FRAGMENT_QUEUE,
	GPU_M4_RENDER_PUBLISH_FRAGMENT_PIPE,
	GPU_M4_RENDER_PROMPT_FRAGMENT,
	GPU_M4_RENDER_PUBLISH_TILER_QUEUE,
	GPU_M4_RENDER_PUBLISH_TILER_PIPE,
	GPU_M4_RENDER_PROMPT_TILER,
	GPU_M4_RENDER_NC_PRODUCER_PUBLICATION_BARRIER,
	GPU_M4_RENDER_PUBLICATION_STEP_COUNT,
};

struct gpu_m4_render_publication_plan {
	u8 queue_entries[2][24]; /* Optional prior edge, optional init/own-TA edge, work. */
	u8 pipe_record[2][0x18];
	u16 queue_slot[2][3], queue_next[2], pipe_slot[2], pipe_next[2];
	u8 queue_count[2];
	u64 threshold_pointer[2], threshold_value, kick[2];
	u64 threshold_to_publish[2];
	u64 uma_submitted_pointer, uma_submitted_value;
	enum gpu_m4_render_publication_step step[GPU_M4_RENDER_PUBLICATION_STEP_COUNT];
	u8 step_count;
};

/*
 * Prove the frozen packet and that these snapshots still name the exact idle
 * queues, pipes, notifier reservations and provider-leased scheduler slots,
 * including both runtime sequence/threshold/stamp reservations, PB/UMA leases
 * and retained predecessors, including the exact retained binding tuple when
 * supplied. Hold that lease through actual publication.
 * Matching engine event 0x70/0x71 does not establish that scheduler lease.
 */
typedef int (*gpu_m4_render_check_publication)(void *cookie,
		const struct gpu_m4_render_packet_input *packet,
		const struct gpu_m4_render_queue_snapshot snapshots[2]);

/*
 * A declarative plan for an already privately prepared, frozen packet.
 * It performs no writes to live buffers. Both thresholds and complete ring
 * entries precede either producer; fragment pipe is prompted before tiler so
 * the firmware can perform partial fragments without a CPU tiler wait. The
 * live provider must execute this order with NC barriers and no fallible step
 * after first publication, and pin uncertain owners. Never signal a fence from
 * this function's success. `new_queue` must be an actual owner snapshot, not a
 * value guessed from context registration or a particular producer index.
 * The runtime path conservatively requires both previously retired queues to
 * be idle and match their reserved prior stamps/thresholds. It supports no wrap
 * or concurrent outstanding-prefix inference. All runtime producers precede
 * the first prompt. Consume threshold_to_publish[stage], not the legacy common
 * threshold_value (which is zero when the two runtime values differ).
 */
static inline int gpu_m4_render_publication_prepare(
		struct gpu_m4_render_publication_plan *output,
		const struct gpu_m4_render_packet_input *packet,
		const struct gpu_m4_render_queue_snapshot snapshot[2], u64 timestamp,
		void *owner_cookie, gpu_m4_render_check_publication check_owner)
{
	struct gpu_m4_exec_interval inputs[7];
	struct gpu_m4_render_publication_plan plan;
	const struct gpu_m4_render_span *fw;
	const struct gpu_m4_render_packet_chronology *chronology;
	unsigned int i, c, count = 5;
	int ret;

	if (!output || !packet || !packet->owner || !packet->owner->render ||
	    !snapshot || !check_owner)
		return -EINVAL;
	inputs[0] = (struct gpu_m4_exec_interval){ (unsigned long)output, sizeof(*output) };
	inputs[1] = (struct gpu_m4_exec_interval){ (unsigned long)packet, sizeof(*packet) };
	inputs[2] = (struct gpu_m4_exec_interval){ (unsigned long)packet->owner, sizeof(*packet->owner) };
	inputs[3] = (struct gpu_m4_exec_interval){ (unsigned long)packet->owner->render,
		sizeof(*packet->owner->render) };
	inputs[4] = (struct gpu_m4_exec_interval){ (unsigned long)snapshot, sizeof(*snapshot) * 2 };
	if (packet->chronology)
		inputs[count++] = (struct gpu_m4_exec_interval){
			(unsigned long)packet->chronology, sizeof(*packet->chronology) };
	if (packet->binding)
		inputs[count++] = (struct gpu_m4_exec_interval){
			(unsigned long)packet->binding, sizeof(*packet->binding) };
	if (!gpu_m4_exec_disjoint(inputs, count))
		return -EINVAL;
	ret = gpu_m4_render_packet_bindings_validate(packet);
	if (ret)
		return ret;
	fw = packet->owner->firmware;
	chronology = packet->chronology;
	memset(&plan, 0, sizeof(plan));
	for (i = 0; i < 2; i++) {
		const struct gpu_m4_render_queue_snapshot *s = &snapshot[i];
		struct gpu_m4_exec_pipe_record record;
		u32 queue_next, pipe_next;
		u64 entries[3];
		unsigned int entry_count = 0;

		if (s->capacity != 256 || s->pipe_capacity != 256 ||
		    s->read >= 256 || s->write >= 256 || s->pipe_read >= 256 ||
		    s->pipe_cfi >= 256 || s->pipe_write >= 256 ||
		    s->scheduler_slot != (i ? 0x71 : 0x70))
			return -EINVAL;
		for (c = 0; c < 3; c++)
			if (s->consumer[c] >= 256)
				return -EINVAL;
		if (s->read != s->write || s->busy ||
		    s->threshold != (chronology ? chronology->engine[i].threshold_before : 0) ||
		    s->firmware_stamp != (chronology ? chronology->engine[i].previous_target : 0) ||
		    s->user_stamp != (chronology ? chronology->engine[i].previous_target : 0) ||
		    s->pipe_read != s->pipe_write ||
		    s->pipe_cfi != s->pipe_write)
			return -EBUSY;
		for (c = 0; c < 3; c++)
			if (s->consumer[c] != s->write)
				return -EBUSY;
		if (chronology && chronology->prior[i].firmware.va)
			entries[entry_count++] = chronology->prior[i].firmware.va;
		if (i || !chronology || chronology->init_buffer)
			entries[entry_count++] = i ? packet->fragment_dependency_firmware.va :
				packet->init_buffer_firmware.va;
		entries[entry_count++] = fw[i ? GPU_M4_RENDER_FIRST_FRAGMENT_WORK_FW :
			GPU_M4_RENDER_FIRST_TILER_WORK_FW].va;
		ret = gpu_m4_exec_ring_reserve(s->read, s->write, entry_count, &queue_next);
		if (ret)
			return ret;
		ret = gpu_m4_exec_pipe_reserve(s->pipe_read, s->pipe_cfi, s->pipe_write, &pipe_next);
		if (ret)
			return ret;
		for (c = 0; c < entry_count; c++) {
			plan.queue_slot[i][c] = (s->write + c) & 255;
			put_unaligned_le64(entries[c], plan.queue_entries[i] + c * 8);
		}
		plan.queue_count[i] = entry_count;
		plan.queue_next[i] = queue_next;
		plan.pipe_slot[i] = s->pipe_write;
		plan.pipe_next[i] = pipe_next;
		record = (struct gpu_m4_exec_pipe_record){ .timestamp = timestamp,
			.queue_info = fw[i ? GPU_M4_RENDER_FIRST_FRAGMENT_QUEUE_FW :
				GPU_M4_RENDER_FIRST_TILER_QUEUE_FW].va,
			.queue_producer = queue_next, .pipe = i,
			.scheduler_slot = s->scheduler_slot, .new_queue = s->new_queue };
		ret = gpu_m4_exec_pipe_record(plan.pipe_record[i], &record);
		if (ret)
			return ret;
		ret = gpu_m4_exec_pipe_kick(i, &plan.kick[i]);
		if (ret)
			return ret;
		plan.threshold_pointer[i] = fw[GPU_M4_RENDER_FIRST_NOTIFIERS_FW].va + 0x600 + i * 8;
		plan.threshold_to_publish[i] = chronology ? chronology->engine[i].threshold_to_publish : 1;
	}
	if (plan.threshold_to_publish[0] == plan.threshold_to_publish[1])
		plan.threshold_value = plan.threshold_to_publish[0];
	if (chronology) {
		static const enum gpu_m4_render_publication_step order[] = {
			GPU_M4_RENDER_PREPARE_ALL_OWNER_AND_RING_BYTES,
			GPU_M4_RENDER_PUBLISH_BOTH_THRESHOLDS,
			GPU_M4_RENDER_NC_PUBLICATION_BARRIER,
			GPU_M4_RENDER_PUBLISH_FRAGMENT_QUEUE,
			GPU_M4_RENDER_PUBLISH_FRAGMENT_PIPE,
			GPU_M4_RENDER_PUBLISH_TILER_QUEUE,
			GPU_M4_RENDER_PUBLISH_TILER_PIPE,
			GPU_M4_RENDER_NC_PRODUCER_PUBLICATION_BARRIER,
			GPU_M4_RENDER_PROMPT_FRAGMENT,
			GPU_M4_RENDER_PROMPT_TILER,
		};

		memcpy(plan.step, order, sizeof(order));
		plan.step_count = sizeof(order) / sizeof(order[0]);
		plan.uma_submitted_pointer = fw[GPU_M4_RENDER_FIRST_SCENE_FW].va + 0x500;
		plan.uma_submitted_value = chronology->uma_submitted;
	} else {
		plan.step_count = GPU_M4_RENDER_NC_PRODUCER_PUBLICATION_BARRIER;
		for (i = 0; i < plan.step_count; i++)
			plan.step[i] = i;
	}
	ret = check_owner(owner_cookie, packet, snapshot);
	if (ret)
		return ret;
	memcpy(output, &plan, sizeof(plan));
	return 0;
}

#endif
