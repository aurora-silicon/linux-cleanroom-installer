/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_RENDER_H
#define APPLE_GPU_M4_RENDER_H

#include "gpu-m4-exec.h"

/* J713/G16G, 25G83. Pure render encoding/retirement; nothing is published here. */
#define GPU_M4_RENDER_REGISTER_SIZE 0x600U
#define GPU_M4_RENDER_TILER_SIZE 0x940U
#define GPU_M4_RENDER_FRAGMENT_SIZE 0xca0U
/* REMOVE-DEBUG experiment knobs (default: the working producer's fallback):
 * TA/FS allocation words (0xa5a1/0xa5a9) and DUPM min/max (0xd419/0xd429).
 */
static u64 gpu_m4_render_alloc_word[2] __attribute__((unused)) = { 0x6000400020ULL, 0x6000400020ULL };
static u64 gpu_m4_render_alloc_minmax[2] __attribute__((unused)) = { 0x200000001ULL, 0x200000001ULL };
#define GPU_M4_RENDER_PARAMETER_MIN 0x400000ULL
#define GPU_M4_RENDER_RANGE1_START 0x1000000000ULL
#define GPU_M4_RENDER_RANGE1_END 0x9000000000ULL
#define GPU_M4_RENDER_TILER_REGISTERS 66U
#define GPU_M4_RENDER_FRAGMENT_REGISTERS 82U
#define GPU_M4_RENDER_PARAMETER_DESCRIPTOR_SIZE 0x10U
#define GPU_M4_RENDER_INIT_BUFFER_SIZE 0x20U
#define GPU_M4_RENDER_PARAMETER_MANAGER_SIZE 0xc0U
#define GPU_M4_RENDER_PARAMETER_SLOT_COUNT 32U
#define GPU_M4_RENDER_PARAMETER_MAX_PAGES 0x3fffffU
#define GPU_M4_RENDER_TIMESTAMP_METADATA_MIN 0x640U
#define GPU_M4_RENDER_TIMESTAMP_FW_START 0xfffffc2ffc000000ULL
#define GPU_M4_RENDER_TIMESTAMP_FW_END 0xfffffc3000000000ULL
/* Source-qualified bounded render domain, independent of the PB slot lease. */
#define GPU_M4_RENDER_UMA_SLOT_FIRST 33U
#define GPU_M4_RENDER_UMA_SLOT_COUNT 32U
#define GPU_M4_RENDER_FIRST_UMA_SLOT GPU_M4_RENDER_UMA_SLOT_FIRST
#define GPU_M4_RENDER_SCENE_SIZE 0x4000U
#define GPU_M4_RENDER_PB_LIST_SIZE 0x200U
#define GPU_M4_RENDER_PB_BLOCK_CAPACITY 64U
#define GPU_M4_RENDER_PROCESS_EMPTY_TILES (1U << 1)

enum gpu_m4_render_resource {
	GPU_M4_RENDER_TILEMAP,
	GPU_M4_RENDER_TPC,
	GPU_M4_RENDER_SCENE_METADATA,
	GPU_M4_RENDER_LAYER_METADATA,
	GPU_M4_RENDER_SCENE_LIST,
	GPU_M4_RENDER_PREEMPT_0,
	GPU_M4_RENDER_PREEMPT_1,
	GPU_M4_RENDER_PREEMPT_2,
	GPU_M4_RENDER_SCRATCH,
	GPU_M4_RENDER_VDM,
	GPU_M4_RENDER_FRAGMENT_AUXILIARY,
	GPU_M4_RENDER_COLOR,
	GPU_M4_RENDER_PARAMETER_HEAP,
	GPU_M4_RENDER_RESOURCE_COUNT,
};

struct gpu_m4_render_span {
	u64 va;
	u64 size;
};

struct gpu_m4_render_geometry {
	u32 width;
	u32 height;
	u32 layers;
	u32 samples;
	u32 utile_width;
	u32 utile_height;
	u32 sample_size; /* Mesa tilebuffer sample stride, not color-format bytes. */
};

/* PB capacity uses screen tiles, not padded table entries or layer count.
 * The runtime may retain a larger heap; use that actual capacity when rebuilding
 * its manager/lists, never overwrite an initialized manager merely to resize.
 */
struct gpu_m4_render_pb_layout {
	u64 heap_size;
	u32 pages, blocks, block_capacity;
	u32 page_list_size, block_list_size;
};

static inline int gpu_m4_render_pb_capacity(struct gpu_m4_render_pb_layout *output,
		u64 heap_size)
{
	struct gpu_m4_render_pb_layout p;
	u64 pages, blocks;
	u32 capacity = 1;

	if (!output || heap_size < GPU_M4_RENDER_PARAMETER_MIN || (heap_size & 0x1ffff))
		return -EINVAL;
	pages = heap_size >> 15;
	blocks = heap_size >> 17;
	if (pages > GPU_M4_RENDER_PARAMETER_MAX_PAGES || blocks >= (1ULL << 28))
		return -E2BIG;
	while (capacity <= blocks)
		capacity <<= 1;
	p = (struct gpu_m4_render_pb_layout){ heap_size, pages, blocks, capacity,
		pages * 4, (u64)capacity * 8 };
	*output = p;
	return 0;
}

static inline int gpu_m4_render_pb_layout(struct gpu_m4_render_pb_layout *output,
		const struct gpu_m4_render_geometry *in)
{
	struct gpu_m4_exec_geometry geometry;
	u64 tiles, blocks, bytes;
	int ret;

	if (!output || !in)
		return -EINVAL;
	ret = gpu_m4_exec_geometry(&geometry, in->width, in->height, in->layers,
		in->samples, in->utile_width, in->utile_height, in->sample_size);
	if (ret)
		return ret;
	tiles = (u64)((in->width + 31) / 32) * ((in->height + 31) / 32);
	blocks = (((tiles + 127) / 128) + 7) & ~7ULL;
	bytes = blocks * 0x20000;
	if (bytes < GPU_M4_RENDER_PARAMETER_MIN)
		bytes = GPU_M4_RENDER_PARAMETER_MIN;
	return gpu_m4_render_pb_capacity(output, bytes);
}

/*
 * Resource specifications are not shader bytes. The supplied backing must
 * contain the complete Mesa-produced executable and the selected 64-byte span.
 * A missing high-word-present flag is an error, never permission to truncate.
 */
struct gpu_m4_render_program {
	struct gpu_m4_render_span backing;
	u64 resources;
	u32 usc_offset;
	bool high_word_present;
};

struct gpu_m4_render_owner {
	u32 context;
	u32 asid;
	u32 parameter_id;
	u8 generation;
	u64 parameter_manager_fw;
	u64 parameter_descriptor_fw;
	u64 queue_usc_base;
};

/*
 * Borrowed from a retained, successfully installed ASID binding under its
 * frozen owner lock. The nonzero software epoch is independent of the bounded
 * wire generation; neither a matching byte nor these copied pointers proves
 * residency. The packet's mandatory owner callback must prove the exact
 * lease/version/epoch/generation tuple through publication and retirement.
 * No wrap or wider epoch truncation is authorized by this detached shape.
 */
struct gpu_m4_render_retained_binding {
	const void *lease;
	const void *version;
	u64 epoch;
	u64 generation;
	u32 asid; /* The bound user context the owner's work and records name. */
};

/*
 * This callback must prove the live context, real reserved PB slot, distinct
 * retained manager/descriptor and queue USC reservation belong to this owner.
 * A successful VM walk alone cannot establish the hardware slot reservation or
 * that two firmware aliases refer to the same retained backing.
 */
typedef int (*gpu_m4_render_check_owner)(void *cookie,
				       const struct gpu_m4_render_owner *owner);

struct gpu_m4_render_input {
	const struct gpu_m4_exec_vm *vm;
	struct gpu_m4_render_owner owner;
	void *owner_cookie;
	gpu_m4_render_check_owner check_owner;
	struct gpu_m4_render_geometry geometry;
	struct gpu_m4_render_span resource[GPU_M4_RENDER_RESOURCE_COUNT];
	struct gpu_m4_render_span vertex_program;
	struct gpu_m4_render_span fragment_program;
	struct gpu_m4_render_program background;
	struct gpu_m4_render_program end_of_tile;
	struct gpu_m4_render_span scissor;
	struct gpu_m4_render_span depth_bias; /* Absent only when both members are zero. */
	struct gpu_m4_render_span query;
	bool depth_bias_is_integer;
	u64 usc_base;
	u64 multisample_control;
	u32 merge_limit[2];
	u32 depth_clear; /* Exact Mesa ISP_BGOBJDEPTH bits, even without Z/S. */
	u32 background_value; /* Whole ISP_BGOBJVALS, not only stencil low byte. */
	u32 ppp_control;
	u32 uuid;
	u32 flags;
	const struct gpu_m4_exec_attachment *attachments[2];
	u32 attachment_count[2];
};

/* This normalized state bit changes only ISP mode in the one-layer profile. */
static inline u64 gpu_m4_render_isp_mode_flags(u32 flags)
{
	return 0x280 | ((flags & GPU_M4_RENDER_PROCESS_EMPTY_TILES) ? 1ULL << 16 : 0);
}

static inline u64 gpu_m4_render_isp_mode(const struct gpu_m4_render_input *in)
{
	return gpu_m4_render_isp_mode_flags(in->flags);
}

/* Only operands actually emitted by the bounded register/work tables. The
 * eleven leading resource roles stop at FRAGMENT_AUXILIARY: COLOR and PB heap
 * are fixture/allocator validation roles, not emitted operands. Native Mesa
 * validation constructs this type directly; it never invents a fixture input.
 */
#define GPU_M4_RENDER_ENCODED_RESOURCES (GPU_M4_RENDER_FRAGMENT_AUXILIARY + 1)
struct gpu_m4_render_encoded {
	struct gpu_m4_render_owner owner;
	struct gpu_m4_render_geometry geometry;
	struct gpu_m4_render_span resource[GPU_M4_RENDER_ENCODED_RESOURCES];
	struct gpu_m4_render_span scissor, depth_bias, query;
	u64 background_resources, end_of_tile_resources;
	u64 background_program, end_of_tile_program;
	u64 usc_base, multisample_control, isp_mode;
	u32 merge_limit[2], depth_clear, background_value, ppp_control, uuid;
	u32 attachment_count[2];
	bool depth_bias_is_integer;
	/* Native uncompressed state. Fixture normalization keeps these zero. */
	u64 zls_control, depth_stride, stencil_stride;
	u32 zls_pixels;
	struct gpu_m4_render_span depth, stencil;
};

/* Nonfallible normalization ONLY after the strict fixture preflight. The
 * native entrance has its own validation and does not call this converter.
 */
static inline void gpu_m4_render_encoded_fixture(struct gpu_m4_render_encoded *out,
		const struct gpu_m4_render_input *in)
{
	unsigned int i;

	memset(out, 0, sizeof(*out));
	out->owner = in->owner;
	out->geometry = in->geometry;
	for (i = 0; i < GPU_M4_RENDER_ENCODED_RESOURCES; i++)
		out->resource[i] = in->resource[i];
	out->scissor = in->scissor;
	out->depth_bias = in->depth_bias;
	out->query = in->query;
	out->background_resources = in->background.resources;
	out->end_of_tile_resources = in->end_of_tile.resources;
	out->background_program = in->usc_base + (in->background.usc_offset & ~63U);
	out->end_of_tile_program = in->usc_base + (in->end_of_tile.usc_offset & ~63U);
	out->usc_base = in->usc_base;
	out->multisample_control = in->multisample_control;
	out->merge_limit[0] = in->merge_limit[0];
	out->merge_limit[1] = in->merge_limit[1];
	out->depth_clear = in->depth_clear;
	out->background_value = in->background_value;
	out->ppp_control = in->ppp_control;
	out->uuid = in->uuid;
	out->isp_mode = gpu_m4_render_isp_mode(in);
	out->attachment_count[0] = in->attachment_count[0];
	out->attachment_count[1] = in->attachment_count[1];
	out->depth_bias_is_integer = in->depth_bias_is_integer;
}

struct gpu_m4_render_register_output {
	u8 registers[GPU_M4_RENDER_REGISTER_SIZE];
	u8 attachments[GPU_M4_EXEC_ATTACHMENT_SIZE];
	struct gpu_m4_exec_geometry geometry;
	u16 count;
	u16 bytes;
	u8 zero_suffix_records;
};

/*
 * The hardware PB descriptor is not the 0xc0-byte parameter manager. This
 * encoder accepts a newly owned, zero descriptor only: a firmware-mutated
 * descriptor has reserved fields and cursors which must not be rebuilt here.
 * Ownership and the root-selected page-list translation are provider gates;
 * fitting these bit fields alone does not qualify either of those gates.
 */
struct gpu_m4_render_parameter_descriptor_input {
	u64 page_list_va;
	u32 pages;
	u32 read;
	u32 write;
	bool wrapped;
};

static inline int gpu_m4_render_parameter_descriptor_fresh(void *output,
		const struct gpu_m4_render_parameter_descriptor_input *in)
{
	u32 words[4];
	u64 rooted;
	unsigned int i;

	if (!output || !in || !in->page_list_va || (in->page_list_va & 127) ||
	    in->page_list_va >= (1ULL << 43) ||
	    in->pages > GPU_M4_RENDER_PARAMETER_MAX_PAGES ||
	    in->read > GPU_M4_RENDER_PARAMETER_MAX_PAGES ||
	    in->write > GPU_M4_RENDER_PARAMETER_MAX_PAGES)
		return -EINVAL;
	rooted = in->page_list_va +
		((in->page_list_va & (1ULL << 42)) ? 0 : 0x7000000000ULL);
	words[0] = ((in->page_list_va >> 4) & 0xfffffff8ULL) | 1;
	words[1] = ((rooted >> 8) & 0x70000000ULL) |
		((in->page_list_va >> 11) & 0x80000000ULL) | in->pages;
	words[2] = in->read;
	words[3] = in->write | (in->wrapped ? 0x80000000U : 0);
	for (i = 0; i < 4; i++)
		put_unaligned_le32(words[i], (u8 *)output + i * 4);
	return 0;
}

/* Literal bytes only, after the entrance's complete slot/backing owner proof. */
static inline void gpu_m4_render_init_buffer_emit(void *output,
		const struct gpu_m4_render_owner *owner, u32 stamp)
{
	memset(output, 0, GPU_M4_RENDER_INIT_BUFFER_SIZE);
	put_unaligned_le32(6, output);
	put_unaligned_le32(owner->context, (u8 *)output + 4);
	put_unaligned_le32(owner->parameter_id, (u8 *)output + 8);
	/* +0xc/+0x10 remain zero: firmware rebuilds this new PB owner. */
	put_unaligned_le64(owner->parameter_manager_fw, (u8 *)output + 0x14);
	put_unaligned_le32(stamp, (u8 *)output + 0x1c);
}

/*
 * A first PB-owner InitBuffer is ordered immediately before its tiler work.
 * This helper only prepares the 0x20-byte record, not a queue entry or producer.
 * Its real slot and distinct manager/descriptor must pass the owner callback.
 */
static inline int gpu_m4_render_init_buffer(void *output,
		const struct gpu_m4_render_owner *owner, void *owner_cookie,
		gpu_m4_render_check_owner check_owner, u32 stamp)
{
	struct gpu_m4_exec_interval firmware[2];
	int ret;

	if (!output || !owner || !check_owner || !gpu_m4_exec_user_asid(owner->asid) ||
	    owner->context != owner->asid || !owner->generation ||
	    owner->parameter_id >= GPU_M4_RENDER_PARAMETER_SLOT_COUNT ||
	    !stamp || (stamp & 0xff) ||
	    !gpu_m4_exec_fw_range(owner->parameter_manager_fw,
				 GPU_M4_RENDER_PARAMETER_MANAGER_SIZE, 8) ||
	    !gpu_m4_exec_fw_range(owner->parameter_descriptor_fw,
				 GPU_M4_RENDER_PARAMETER_DESCRIPTOR_SIZE, 8))
		return -EINVAL;
	firmware[0] = (struct gpu_m4_exec_interval){ owner->parameter_manager_fw,
		GPU_M4_RENDER_PARAMETER_MANAGER_SIZE };
	firmware[1] = (struct gpu_m4_exec_interval){ owner->parameter_descriptor_fw,
		GPU_M4_RENDER_PARAMETER_DESCRIPTOR_SIZE };
	if (!gpu_m4_exec_disjoint(firmware, 2))
		return -EINVAL;
	ret = check_owner(owner_cookie, owner);
	if (ret)
		return ret;
	gpu_m4_render_init_buffer_emit(output, owner, stamp);
	return 0;
}

/*
 * Prepare the two u64 pointer fields in a stage's work internal timestamp pair.
 * Metadata is a retained firmware alias of the pass-owned result allocation.
 * This never selects the optional user pairs at +0x610/+0x630, initializes an
 * existing result cell, or supplies any of the unresolved sequence UMA fields.
 */
static inline int gpu_m4_render_internal_timestamp_pair(void *output,
		const struct gpu_m4_render_span *metadata_fw,
		enum gpu_m4_exec_engine engine)
{
	u64 results;

	if (!output || !metadata_fw ||
	    (engine != GPU_M4_EXEC_TILER && engine != GPU_M4_EXEC_FRAGMENT) ||
	    metadata_fw->size < GPU_M4_RENDER_TIMESTAMP_METADATA_MIN ||
	    !gpu_m4_exec_fw_range(metadata_fw->va, metadata_fw->size, 8))
		return -EINVAL;
	results = metadata_fw->va + (engine == GPU_M4_EXEC_TILER ? 0x600 : 0x620);
	put_unaligned_le64(results, output);
	put_unaligned_le64(results + 8, (u8 *)output + 8);
	return 0;
}

struct gpu_m4_render_timestamp_output {
	u8 internal_pair[16];
	u8 start[0x4c];
	u8 wait[4];
	u8 end[0x4c];
};

/*
 * Detached no-query timestamp pieces. The caller copies the pair into its work
 * (+0x8c8/+0xc10) and the records into its separate sequence (+0x1cc/+0x1ec).
 * Those work/sequence aliases and retained result backing must independently
 * pass owner/translation checks before publication; this only encodes them.
 */
static inline int gpu_m4_render_timestamps(
		struct gpu_m4_render_timestamp_output *output,
		u64 work_fw, u64 queue_fw, const struct gpu_m4_render_span *metadata_fw,
		u32 uuid, enum gpu_m4_exec_engine engine)
{
	struct gpu_m4_exec_interval ranges[3];
	u64 control, pair, auxiliary, context;
	u8 internal_pair[16];
	unsigned int i;
	int ret;

	if (!output || !uuid ||
	    (engine != GPU_M4_EXEC_TILER && engine != GPU_M4_EXEC_FRAGMENT) ||
	    !gpu_m4_exec_fw_range(work_fw, engine == GPU_M4_EXEC_TILER ?
		GPU_M4_RENDER_TILER_SIZE : GPU_M4_RENDER_FRAGMENT_SIZE, 8) ||
	    !gpu_m4_exec_fw_range(queue_fw, GPU_M4_EXEC_QUEUE_SIZE, 8))
		return -EINVAL;
	ret = gpu_m4_render_internal_timestamp_pair(internal_pair, metadata_fw, engine);
	if (ret)
		return ret;
	ranges[0] = (struct gpu_m4_exec_interval){ work_fw, engine == GPU_M4_EXEC_TILER ?
		GPU_M4_RENDER_TILER_SIZE : GPU_M4_RENDER_FRAGMENT_SIZE };
	ranges[1] = (struct gpu_m4_exec_interval){ queue_fw, GPU_M4_EXEC_QUEUE_SIZE };
	ranges[2] = (struct gpu_m4_exec_interval){ metadata_fw->va, metadata_fw->size };
	if (!gpu_m4_exec_disjoint(ranges, 3))
		return -EINVAL;
	control = work_fw + (engine == GPU_M4_EXEC_TILER ? 0x8c0 : 0xc08);
	pair = work_fw + (engine == GPU_M4_EXEC_TILER ? 0x8c8 : 0xc10);
	auxiliary = work_fw + (engine == GPU_M4_EXEC_TILER ? 0x928 : 0xc70);
	context = work_fw + (engine == GPU_M4_EXEC_TILER ? 0x83c : 0xb84);
	memset(output, 0, sizeof(*output));
	memcpy(output->internal_pair, internal_pair, sizeof(internal_pair));
	for (i = 0; i < 2; i++) {
		u8 *record = i ? output->end : output->start;

		put_unaligned_le32(i ? 3 : 0x80000003U, record);
		put_unaligned_le64(control, record + 4);
		put_unaligned_le64(pair, record + 0xc);
		put_unaligned_le64(pair + i * 8, record + 0x14);
		put_unaligned_le64(queue_fw, record + 0x1c);
		/* +0x24 optional API/user timestamp pair is absent. */
		put_unaligned_le64(auxiliary, record + 0x2c);
		put_unaligned_le64(context, record + 0x34);
		put_unaligned_le64(uuid, record + 0x44);
	}
	put_unaligned_le32(1, output->wait);
	return 0;
}

/*
 * A serialized first render owner has one UMA descriptor and two submissions.
 * The provider must snapshot the same retained UMA owner for both engines.
 * An event, one stage's stamp, or observed pixels alone never retires the pair.
 * WB readback/fence visibility remains a separate provider gate after this.
 */
static inline int gpu_m4_render_pair_retired(
		const struct gpu_m4_exec_retirement *tiler,
		const struct gpu_m4_exec_retirement *fragment)
{
	int ret;

	if (!tiler || !fragment || tiler->uma_submitted != 2 || fragment->uma_submitted != 2)
		return -EINVAL;
	ret = gpu_m4_exec_retired(tiler);
	if (ret)
		return ret;
	return gpu_m4_exec_retired(fragment);
}

struct gpu_m4_render_stage_retirement {
	u64 notifier_fw, uma_descriptor_fw;
	u32 uma_slot; /* Observed inline UMA slot, not inferred from a PB ID. */
	/* Host reservations; the working render wire sequence remains zero. */
	u64 reservation_sequence, expected_threshold, threshold;
	u64 expected_counter, work_counter, start_counter;
	u32 expected_target, firmware_stamp, user_stamp;
	u32 queue_begin, queue_entries, queue_next, queue_read, queue_producer;
	u32 queue_consumer[3], queue_busy, queue_capacity;
	u32 pipe_begin, pipe_next, pipe_read, pipe_cfi, pipe_producer, pipe_capacity;
	u64 timestamp_start, timestamp_end; /* Internal engine cells: absolute 24 MHz ticks. */
	u8 expected_generation, generation;
	u32 cache_generation;
	/* Optional stricter diagnostic/qualification policy, never implied by UMA. */
	bool require_timestamp_order;
	/* Bounded published window, split by owner. Physical successors: later
	 * records were already reachable on this engine's queue/pipe (of any
	 * in-flight job); queue/pipe_horizon is the owned last producer.
	 * Notifier successors: later work was published on this command's
	 * notifier; target/threshold_horizon is that notifier's published tail.
	 * A half without successors keeps its exact solitary predicate and zero
	 * horizon fields. Saved with every receipt.
	 */
	bool physical_successors, notifier_successors;
	u32 queue_horizon, pipe_horizon, target_horizon;
	u64 threshold_horizon;
	/* Batched packet: this command's single submit-pipe record also
	 * published its later packet members' entries, so the owned pipe
	 * horizon may equal this command's own pipe end (no later record).
	 * Requires physical successors; its queue end stays its own.
	 */
	bool pipe_shared;
	/* Not the only command in flight globally (two packets in flight):
	 * an engine without physical successors keeps its exact positions but
	 * its queue-info busy may stay set while other work runs.
	 */
	bool ignore_busy;
};

struct gpu_m4_render_retirement_copy {
	u64 destination_fw, value; /* Actual bound BO value: absolute nanoseconds. */
	u32 engine;
	bool end, completed, cpu_visible;
};

struct gpu_m4_render_retirement_dependency {
	u64 record_fw, stamp_fw;
	u32 expected_target, firmware_stamp, consuming_target, event, engine;
	/* Nonzero: the cumulative stamp may have passed expected_target, never
	 * this owned published tail. Zero keeps exact equality.
	 */
	u32 stamp_horizon;
	bool completed;
};

struct gpu_m4_render_pair_retirement;
/*
 * Under the retained provider lease, verify these fresh observations come from
 * this exact published scene, selected notifiers, command/sequence graph,
 * queues/pipes and shared UMA owner/global-table slot. Verify the frozen expected
 * reservations, generation/root/mapping ownership, stable packed high/low/high
 * completion reads with acquire ordering, and the real health/fault-drain snapshot.
 * Prove both proof arrays' retained bounds and complete requested-copy/edge
 * coverage, including the actual own-TA -> FS edge and optional InitBuffer.
 * Each completed copy must come from its actual bound timestamp BO and current
 * stage endpoint converted from 24 MHz ticks to ns, including a valid zero
 * value, with cache ownership complete.
 * Verify all GPU/FW outputs' CPU/PRIME visibility and retained pins on every
 * uncertain or firmware-last-owner object. Do not mutate or release live state.
 * A fence, event hint or copied metadata is no proof.
 * This callback supplements, and cannot override, the scalar checks below.
 */
typedef int (*gpu_m4_render_check_retirement)(void *cookie,
		const struct gpu_m4_render_pair_retirement *pair);

/*
 * Under the ONE retained owner lock, prove the exact protected published job,
 * installed binding and PB/UMA/notifier/queue/dependency leases. Validate both
 * arrays' retained bounds/identities before the helper follows them. Prove the
 * COMPLETE requested timestamp operation sequence, including each destination,
 * engine and start/end endpoint, from the owned work/sequence and its full
 * hardware retirement; a copied completed flag is not that proof.
 *
 * This pre-cache callback must not read bound BO payloads, copy.value,
 * copy.cpu_visible or outputs_cpu_visible. It neither performs cache maintenance
 * nor retires owners, writes fences or grants CPU visibility. It supplements
 * every scalar hardware check below and cannot waive one. A source-only helper
 * and callback shape are not a hardware qualification.
 */
typedef int (*gpu_m4_render_check_hardware_retirement)(void *cookie,
		const struct gpu_m4_render_pair_retirement *pair);

struct gpu_m4_render_pair_retirement {
	struct gpu_m4_render_stage_retirement stage[2];
	u64 uma_descriptor_fw, uma_submitted_fw;
	/* Held global slot and actual descriptor +0x08 observation must agree. */
	u32 expected_uma_slot, uma_slot;
	u64 uma_submitted_before, uma_expected_submitted, uma_submitted;
	/* Nonzero for a retained allocator shared by later pairs: the S it had
	 * published when sampled. Completed may then pass this pair's S (later
	 * pairs advance the same M), never this horizon.
	 */
	u64 uma_submitted_horizon;
	/* Bounded nontearing capture of the packed descriptor+0x54 completed u64. */
	u32 uma_completed_high_before, uma_completed_low, uma_completed_high_after;
	const struct gpu_m4_render_retirement_copy *copies;
	u32 copy_count, copy_capacity;
	const struct gpu_m4_render_retirement_dependency *dependencies;
	u32 dependency_count, dependency_capacity;
	bool init_buffer;
	bool rtkit_healthy, run_healthy, control_healthy, binding_healthy;
	bool fault_drain_complete, fault_present, recovery_event, recovery_statistic;
	bool crashed, timed_out, outputs_cpu_visible;
	void *owner_cookie;
	gpu_m4_render_check_retirement check_owner;
};

/* One saved ring end against the owned published horizon: 0 when the
 * consumer passed end without passing the horizon, -EAGAIN while it is still
 * behind end, -EPROTO when it is beyond anything this owner published.
 */
static inline int gpu_m4_render_horizon_position(u32 observed, u32 end, u32 span)
{
	if (((observed - end) & 255) <= span)
		return 0;
	return ((end - observed) & 255) < 128 ? -EAGAIN : -EPROTO;
}

/* Through-prefix observation of one paired stage with published successors,
 * each half applied independently. Physical: both producers equal the owned
 * horizon (never an unowned producer change), every consumer and both pipe
 * cursors passed this command's own end but not the horizon; busy may stay
 * set for a successor. A batched member's pipe end is its packet's shared
 * record: with no later packet both pipe cursors must equal it exactly. Notifier: the shared threshold names the published
 * tail and cumulative stamps reached the own target without passing the tail
 * target. A half without successors keeps the exact solitary rule. Own
 * work/Start counters remain exact (caller).
 */
static inline int __gpu_m4_render_stage_horizon(
		const struct gpu_m4_render_stage_retirement *s, unsigned int engine)
{
	u32 span = 0, pipe_span = 0, consumer[4];
	unsigned int i;
	int ret;

	if (s->physical_successors) {
		if (s->queue_horizon >= 256 || s->pipe_horizon >= 256)
			return -EINVAL;
		span = (s->queue_horizon - s->queue_next) & 255;
		pipe_span = (s->pipe_horizon - s->pipe_next) & 255;
		/* A shared (batched) pipe record may itself be the pipe horizon. */
		if (!span || span >= 128 || (!pipe_span && !s->pipe_shared) || pipe_span >= 128)
			return -EINVAL;
	} else if (s->queue_horizon || s->pipe_horizon || s->pipe_shared) {
		return -EINVAL;
	}
	if (s->notifier_successors) {
		if (!s->target_horizon || (s->target_horizon & 0xff) ||
		    s->threshold_horizon <= s->expected_threshold ||
		    s->target_horizon == s->expected_target ||
		    !gpu_m4_exec_stamp_reached(s->target_horizon, s->expected_target))
			return -EINVAL;
	} else if (s->target_horizon || s->threshold_horizon) {
		return -EINVAL;
	}
	consumer[0] = s->queue_read;
	for (i = 0; i < 3; i++)
		consumer[i + 1] = s->queue_consumer[i];
	if (s->physical_successors) {
		if (s->queue_producer != s->queue_horizon || s->pipe_producer != s->pipe_horizon)
			return -ESTALE;
		for (i = 0; i < 4; i++) {
			ret = gpu_m4_render_horizon_position(consumer[i], s->queue_next, span);
			if (ret)
				return ret;
		}
		ret = gpu_m4_render_horizon_position(s->pipe_read, s->pipe_next, pipe_span);
		if (!ret)
			ret = gpu_m4_render_horizon_position(s->pipe_cfi, s->pipe_next, pipe_span);
		if (ret)
			return ret;
	} else {
		if (s->queue_read != s->queue_next || s->queue_producer != s->queue_next ||
		    (s->queue_busy && !s->ignore_busy) || s->pipe_read != s->pipe_next ||
		    s->pipe_cfi != s->pipe_next || s->pipe_producer != s->pipe_next)
			return -EAGAIN;
		for (i = 0; i < 4; i++)
			if (consumer[i] != s->queue_next)
				return -EAGAIN;
	}
	if (!s->notifier_successors) {
		/* Paired entrance: a TA USER omission is the pair's decision. */
		return s->threshold != s->expected_threshold ||
			s->firmware_stamp != s->expected_target ||
			(engine && s->user_stamp != s->expected_target) ? -EAGAIN : 0;
	}
	if (s->threshold != s->threshold_horizon || !s->firmware_stamp ||
	    !gpu_m4_exec_stamp_reached(s->firmware_stamp, s->expected_target) ||
	    (engine && (!s->user_stamp ||
			!gpu_m4_exec_stamp_reached(s->user_stamp, s->expected_target))))
		return -EAGAIN;
	if (!gpu_m4_exec_stamp_reached(s->target_horizon, s->firmware_stamp) ||
	    (engine && !gpu_m4_exec_stamp_reached(s->target_horizon, s->user_stamp)))
		return -EPROTO; /* Beyond the known published tail. */
	return 0;
}

/* Private fields only. A TA user omission is selected solely by the complete
 * paired entrances below, never an isolated caller-supplied policy flag.
 */
static inline int __gpu_m4_render_stage_retired(
		const struct gpu_m4_render_stage_retirement *s, unsigned int engine,
		bool paired)
{
	unsigned int i;
	int ret;

	if (!s || engine > 1 || !s->expected_target || (s->expected_target & 0xff) ||
	    !s->expected_generation ||
	    !gpu_m4_exec_fw_range(s->notifier_fw, GPU_M4_EXEC_NOTIFY_SIZE, 8) ||
	    s->queue_capacity != 256 || s->queue_begin >= 256 ||
	    !s->queue_entries || s->queue_entries > 3 ||
	    (engine && s->queue_entries < 2) ||
	    s->queue_next >= 256 || s->queue_read >= 256 || s->queue_producer >= 256 ||
	    s->queue_next != ((s->queue_begin + s->queue_entries) & 255) ||
	    s->pipe_capacity != 256 || s->pipe_begin >= 256 || s->pipe_next >= 256 ||
	    s->pipe_read >= 256 || s->pipe_cfi >= 256 || s->pipe_producer >= 256 ||
	    s->pipe_next != ((s->pipe_begin + 1) & 255))
		return -EINVAL;
	for (i = 0; i < 3; i++)
		if (s->queue_consumer[i] >= 256)
			return -EINVAL;
	if (s->reservation_sequence == ~0ULL)
		return -EOVERFLOW;
	if (s->expected_threshold != s->reservation_sequence + 1 ||
	    s->work_counter != s->expected_counter || s->start_counter != s->expected_counter)
		return -EINVAL;
	if (s->generation != s->expected_generation || s->cache_generation)
		return -EIO;
	if (s->physical_successors || s->notifier_successors) {
		/* Only the authenticated paired window entrance tolerates successors;
		 * the isolated stage proof stays strict.
		 */
		if (!paired)
			return -EINVAL;
		ret = __gpu_m4_render_stage_horizon(s, engine);
		if (ret)
			return ret;
	} else {
		if (s->queue_horizon || s->pipe_horizon || s->target_horizon ||
		    s->threshold_horizon || s->pipe_shared)
			return -EINVAL;
		/* Conservative serialized profile: no coalesced/future stamps or prefixes. */
		if (s->threshold != s->expected_threshold ||
		    s->firmware_stamp != s->expected_target ||
		    ((!paired || engine) && s->user_stamp != s->expected_target) ||
		    s->queue_read != s->queue_next || s->queue_producer != s->queue_next ||
		    (s->queue_busy && !s->ignore_busy) || s->pipe_read != s->pipe_next ||
		    s->pipe_cfi != s->pipe_next || s->pipe_producer != s->pipe_next)
			return -EAGAIN;
		for (i = 0; i < 3; i++)
			if (s->queue_consumer[i] != s->queue_next)
				return -EAGAIN;
	}
	if (s->require_timestamp_order &&
	    (!s->timestamp_start || s->timestamp_end <= s->timestamp_start))
		return -EAGAIN;
	return 0;
}

/* Isolated stage proof stays strict: no UMA/dependency/FS owner exists here. */
static inline int gpu_m4_render_stage_retired(
		const struct gpu_m4_render_stage_retirement *s, unsigned int engine)
{
	return __gpu_m4_render_stage_retired(s, engine, false);
}

/* The firmware user destination holds floor(internal_ticks * 125 / 3), not
 * the internal 24 MHz representation. Divide first so intermediate products
 * cannot overflow, and reject an unrepresentable final ns value. No output
 * changes on failure; zero is a valid timestamp, not a completion substitute.
 */
static inline int gpu_m4_render_timestamp_ns(u64 ticks, u64 *nanoseconds)
{
	u64 whole = ticks / 3, fraction = (ticks % 3) * 125 / 3;

	if (!nanoseconds)
		return -EINVAL;
	if (whole > ~0ULL / 125)
		return -EOVERFLOW;
	whole *= 125;
	if (whole > ~0ULL - fraction)
		return -EOVERFLOW;
	*nanoseconds = whole + fraction;
	return 0;
}

/* Private common checks; only the two explicit entrances below choose whether
 * to inspect post-cache visibility. The callback has the entrance's stronger
 * phase-specific contract even though its C function signature is identical.
 */
static inline int __gpu_m4_render_pair_retired(
		const struct gpu_m4_render_pair_retirement *in, void *owner_cookie,
		gpu_m4_render_check_retirement check_owner, bool visible)
{
	struct gpu_m4_exec_interval ranges[3];
	u64 completed;
	unsigned int i, count = 1, own_edges = 0, edges[2] = { 0, 0 };
	int ret;

	if (!in || !check_owner ||
	    in->expected_uma_slot < GPU_M4_RENDER_UMA_SLOT_FIRST ||
	    in->expected_uma_slot - GPU_M4_RENDER_UMA_SLOT_FIRST >= GPU_M4_RENDER_UMA_SLOT_COUNT ||
	    in->uma_slot != in->expected_uma_slot ||
	    in->stage[0].uma_slot != in->expected_uma_slot ||
	    in->stage[1].uma_slot != in->expected_uma_slot ||
	    !gpu_m4_exec_fw_range(in->uma_descriptor_fw, 0xa0, 8) ||
	    !gpu_m4_exec_fw_range(in->uma_submitted_fw, 8, 8) ||
	    in->copy_count > in->copy_capacity || (in->copy_count && !in->copies) ||
	    !in->dependency_count || in->dependency_count > 3 || !in->dependencies ||
	    in->dependency_count > in->dependency_capacity)
		return -EINVAL;
	if (in->uma_submitted_before > ~0ULL - 2 || in->stage[1].expected_counter > ~0ULL - 2)
		return -EOVERFLOW;
	if ((in->uma_submitted_before & 1) ||
	    in->uma_expected_submitted != in->uma_submitted_before + 2 ||
	    !(in->stage[1].expected_counter & 1) ||
	    in->stage[0].expected_counter != in->stage[1].expected_counter + 1 ||
	    in->stage[0].expected_target != in->stage[1].expected_target ||
	    in->stage[0].notifier_fw != in->stage[1].notifier_fw ||
	    in->stage[0].uma_descriptor_fw != in->uma_descriptor_fw ||
	    in->stage[1].uma_descriptor_fw != in->uma_descriptor_fw ||
	    in->stage[0].physical_successors != in->stage[1].physical_successors ||
	    in->stage[0].pipe_shared != in->stage[1].pipe_shared ||
	    in->stage[0].notifier_successors != in->stage[1].notifier_successors ||
	    in->stage[0].target_horizon != in->stage[1].target_horizon)
		return -EINVAL;
	if (in->timed_out)
		return -ETIMEDOUT;
	if (!in->rtkit_healthy || !in->run_healthy || !in->control_healthy || !in->binding_healthy ||
	    in->fault_present || in->recovery_event || in->recovery_statistic || in->crashed)
		return -EIO;
	if (!in->fault_drain_complete || (visible && !in->outputs_cpu_visible) ||
	    in->uma_completed_high_before != in->uma_completed_high_after)
		return -EAGAIN;
	completed = ((u64)in->uma_completed_high_after << 32) | in->uma_completed_low;
	if (in->uma_submitted != in->uma_expected_submitted)
		return -EAGAIN;
	if (in->uma_submitted_horizon) {
		if ((in->uma_submitted_horizon & 1) ||
		    in->uma_submitted_horizon < in->uma_expected_submitted)
			return -EINVAL;
		/* Stage completions of later pairs may count first; the stage
		 * stamps below prove this pair's own TA and FS.
		 */
		if (completed < in->uma_expected_submitted)
			return -EAGAIN;
		if (completed > in->uma_submitted_horizon)
			return -EPROTO;
	} else if (completed != in->uma_expected_submitted) {
		return -EAGAIN;
	}
	for (i = 0; i < 2; i++) {
		ret = __gpu_m4_render_stage_retired(&in->stage[i], i, true);
		if (ret)
			return ret;
	}
	/* With notifier successors, a reached TA USER stamp stays within the
	 * published tail. Its omission is substituted by this scene's FS FW and
	 * USER stamps, both nonzero, reached and bounded by the authenticated
	 * published tail:
	 * later fragments advance the shared cells, so an exact [target, target]
	 * pair need never be observed. Every other exact own check (UMA, TA FW,
	 * counters, binding, slot, prefixes, dependencies, owner callback) still
	 * applies; the raw observed TA USER value is kept, never synthesized.
	 */
	if (in->stage[0].notifier_successors) {
		const struct gpu_m4_render_stage_retirement *ta = &in->stage[0], *fs = &in->stage[1];

		if (ta->user_stamp && gpu_m4_exec_stamp_reached(ta->user_stamp, ta->expected_target)) {
			if (!gpu_m4_exec_stamp_reached(ta->target_horizon, ta->user_stamp))
				return -EPROTO;
		} else if (!fs->firmware_stamp || !fs->user_stamp ||
			   !gpu_m4_exec_stamp_reached(fs->firmware_stamp, fs->expected_target) ||
			   !gpu_m4_exec_stamp_reached(fs->user_stamp, fs->expected_target)) {
			return -EAGAIN;
		} else if (!gpu_m4_exec_stamp_reached(fs->target_horizon, fs->firmware_stamp) ||
			   !gpu_m4_exec_stamp_reached(fs->target_horizon, fs->user_stamp)) {
			return -EPROTO;
		}
	}
	/* Bound the exact serialized command graph before following proof arrays. */
	if (in->dependency_count != in->stage[0].queue_entries + in->stage[1].queue_entries -
				   2 - in->init_buffer)
		return -EINVAL;
	ranges[0] = (struct gpu_m4_exec_interval){ (unsigned long)in, sizeof(*in) };
	if (in->copy_count)
		ranges[count++] = (struct gpu_m4_exec_interval){ (unsigned long)in->copies,
			(u64)in->copy_count * sizeof(*in->copies) };
	ranges[count++] = (struct gpu_m4_exec_interval){ (unsigned long)in->dependencies,
		(u64)in->dependency_count * sizeof(*in->dependencies) };
	if (!gpu_m4_exec_disjoint(ranges, count))
		return -EINVAL;
	/* Validate retained array bounds/identity before following their pointers. */
	ret = check_owner(owner_cookie, in);
	if (ret)
		return ret;
	for (i = 0; i < in->copy_count; i++) {
		const struct gpu_m4_render_retirement_copy *copy = &in->copies[i];
		const struct gpu_m4_render_stage_retirement *s;
		u64 expected_ns;

		if (copy->engine > 1 || (copy->destination_fw & 7) ||
		    copy->destination_fw < GPU_M4_RENDER_TIMESTAMP_FW_START ||
		    copy->destination_fw >= GPU_M4_RENDER_TIMESTAMP_FW_END ||
		    8 > GPU_M4_RENDER_TIMESTAMP_FW_END - copy->destination_fw)
			return -EINVAL;
		s = &in->stage[copy->engine];
		if (!copy->completed)
			return -EAGAIN;
		/* The hardware entrance proves operation/endpoint ownership through
		 * its mandatory callback, not an early read of the destination BO.
		 */
		if (!visible)
			continue;
		if (!copy->cpu_visible)
			return -EAGAIN;
		ret = gpu_m4_render_timestamp_ns(copy->end ? s->timestamp_end :
							s->timestamp_start, &expected_ns);
		if (ret)
			return ret;
		if (copy->value != expected_ns)
			return -EAGAIN;
	}
	for (i = 0; i < in->dependency_count; i++) {
		const struct gpu_m4_render_retirement_dependency *d = &in->dependencies[i];

		if (d->engine > 1 || !gpu_m4_exec_fw_range(d->record_fw, 0x40, 8) ||
		    !gpu_m4_exec_fw_range(d->stamp_fw, 4, 8) || !d->expected_target ||
		    (d->expected_target & 0xff) || d->consuming_target != in->stage[d->engine].expected_target)
			return -EINVAL;
		if (d->event == 0x70 && !d->engine) {
			/* FRAGMENT_DEPENDENCY early tiling: this TA waited on the
			 * previous render's TA in the same queue, never its own.
			 */
			if (d->stamp_fw != in->stage[0].notifier_fw + 0x620 ||
			    d->expected_target >= d->consuming_target)
				return -EINVAL;
		} else if (d->event == 0x70) {
			if (d->engine != 1 || d->stamp_fw != in->stage[0].notifier_fw + 0x620 ||
			    d->expected_target != in->stage[0].expected_target || ++own_edges != 1)
				return -EINVAL;
		} else if ((d->event != 15 && d->event != 0x71) ||
			   /* A compute target is another engine's allocator. */
			   (d->event != 15 && d->expected_target >= d->consuming_target)) {
			return -EINVAL;
		}
		if (d->stamp_horizon) {
			/* Window: a cumulative stamp may pass the edge's target,
			 * never the owned published tail of that notifier.
			 */
			if ((d->stamp_horizon & 0xff) ||
			    !gpu_m4_exec_stamp_reached(d->stamp_horizon, d->expected_target))
				return -EINVAL;
			if (!d->completed || !d->firmware_stamp ||
			    !gpu_m4_exec_stamp_reached(d->firmware_stamp, d->expected_target))
				return -EAGAIN;
			if (!gpu_m4_exec_stamp_reached(d->stamp_horizon, d->firmware_stamp))
				return -EPROTO;
		} else if (!d->completed || d->firmware_stamp != d->expected_target) {
			return -EAGAIN;
		}
		if (++edges[d->engine] > (d->engine ? 2 : 1))
			return -EINVAL;
	}
	if (own_edges != 1 || in->stage[0].queue_entries != 1 + edges[0] + in->init_buffer ||
	    in->stage[1].queue_entries != 1 + edges[1])
		return -EINVAL;
	return 0;
}

/* Full hardware-only paired retirement for the conservatively serialized
 * owner. This entrance never reads copy.value/cpu_visible or outputs_cpu_visible
 * and does not convert API values. Requested copy operations and dependencies
 * must actually have completed, with exact coverage proved by the mandatory
 * hardware owner callback. It is suitable only as a pre-cache hardware proof:
 * success does not retire the cache/version, grant CPU access or signal a fence.
 * The actual job core performs those transitions once, after this proof.
 * TA user-stamp publication may be omitted ONLY here: actual TA firmware,
 * both FS stamps, exact cumulative own UMA, retained own-TA dependency and all
 * queue/pipe/ownership/operation gates remain mandatory. Observed TA USER is
 * preserved, never replaced with the target. Isolated stage proof stays strict.
 */
static inline int gpu_m4_render_pair_hardware_retired(
		const struct gpu_m4_render_pair_retirement *in, void *owner_cookie,
		gpu_m4_render_check_hardware_retirement check_owner)
{
	return __gpu_m4_render_pair_retired(in, owner_cookie, check_owner, false);
}

/*
 * Full paired completion PLUS output visibility for a conservatively serialized
 * retained owner. This original strict entrance is also the post-cache diagnostic
 * observer; it must not become a new fallible check after fence success is ready.
 * Render diagnostics may be zero; every requested timestamp copy and dependency
 * needs its own actual completion. Thresholds, R and UMA counts remain independent.
 * The serialized graph has the own-TA -> FS edge and at most one prior-final edge
 * per stage. No writes, fence signalling, cache maintenance, reclamation,
 * last-owner displacement or wrap/coalescing protocol are inferred.
 * The original pair_retired() above remains the stricter first-only wrapper.
 */
static inline int gpu_m4_render_pair_cumulative_retired(
		const struct gpu_m4_render_pair_retirement *in)
{
	if (!in)
		return -EINVAL;
	return __gpu_m4_render_pair_retired(in, in->owner_cookie, in->check_owner, true);
}

static inline int gpu_m4_render_geometry(struct gpu_m4_exec_geometry *output,
					const struct gpu_m4_render_geometry *in)
{
	if (!in)
		return -EINVAL;
	return gpu_m4_exec_geometry(output, in->width, in->height, in->layers,
				    in->samples, in->utile_width, in->utile_height,
				    in->sample_size);
}

static inline int gpu_m4_render_span(const struct gpu_m4_exec_vm *vm,
				    const struct gpu_m4_render_span *span,
				    u64 minimum, u32 alignment, bool write)
{
	if (!span || !minimum || span->size < minimum || !alignment ||
	    (alignment & (alignment - 1)) || (span->va & (alignment - 1)))
		return -EINVAL;
	return gpu_m4_exec_vm_range(vm, span->va, span->size, write);
}

/* Range-1 applies only to the selected operands, not to every client address. */
static inline int gpu_m4_render_compact(const struct gpu_m4_render_span *span,
				       u64 *output)
{
	if (!span || !output || !span->size ||
	    span->va < GPU_M4_RENDER_RANGE1_START || span->va >= GPU_M4_RENDER_RANGE1_END ||
	    span->size > GPU_M4_RENDER_RANGE1_END - span->va)
		return -EINVAL;
	*output = span->va - GPU_M4_RENDER_RANGE1_START;
	return 0;
}

static inline int gpu_m4_render_usc_span(const struct gpu_m4_render_input *in,
					const struct gpu_m4_render_span *span)
{
	if (!span || span->va < in->usc_base || span->va - in->usc_base >= 0x100000000ULL ||
	    !span->size || span->size > 0x100000000ULL - (span->va - in->usc_base))
		return -EINVAL;
	return gpu_m4_render_span(in->vm, span, 64, 64, false);
}

static inline int gpu_m4_render_program(const struct gpu_m4_render_input *in,
				       const struct gpu_m4_render_program *program,
				       u64 *va)
{
	u64 selected;
	int ret;

	if (!program || !va || (!program->high_word_present && (program->resources >> 32)))
		return -EINVAL;
	ret = gpu_m4_render_usc_span(in, &program->backing);
	if (ret)
		return ret;
	selected = in->usc_base + (program->usc_offset & ~63U);
	if (selected < program->backing.va || selected - program->backing.va >= program->backing.size ||
	    64 > program->backing.size - (selected - program->backing.va))
		return -EINVAL;
	ret = gpu_m4_exec_vm_range(in->vm, selected, 64, false);
	if (ret)
		return ret;
	*va = selected;
	return 0;
}

static inline u64 gpu_m4_render_encoded_isp_mode(const struct gpu_m4_render_encoded *in)
{
	return in->isp_mode | (in->geometry.layers > 1 ? 1 : 0);
}

static inline u64 gpu_m4_render_encoded_layer_control(const struct gpu_m4_render_encoded *in)
{
	return in->geometry.layers == 1 ? 0x8000 :
		0xc000 | ((in->isp_mode & (1ULL << 16)) ? 0x2000 : 0x1000) |
		(in->geometry.layers - 1);
}

/*
 * The first checked-pixel profile is deliberately bounded. Larger, layered,
 * multisample, depth/stencil and overlapping-owner packets need separate gates.
 * Resource spans cover complete retained allocations, not just first words.
 */
static inline int gpu_m4_render_validate(const struct gpu_m4_render_input *in,
					struct gpu_m4_exec_geometry *geometry,
					u64 *background, u64 *end_of_tile)
{
	static const enum gpu_m4_render_resource compact[] = {
		GPU_M4_RENDER_TILEMAP, GPU_M4_RENDER_TPC, GPU_M4_RENDER_SCENE_METADATA,
		GPU_M4_RENDER_LAYER_METADATA, GPU_M4_RENDER_SCENE_LIST,
		GPU_M4_RENDER_PREEMPT_0, GPU_M4_RENDER_PREEMPT_1,
		GPU_M4_RENDER_PREEMPT_2, GPU_M4_RENDER_VDM,
	};
	struct gpu_m4_exec_interval ranges[GPU_M4_RENDER_RESOURCE_COUNT - 1];
	struct gpu_m4_exec_interval firmware[2];
	u64 unused;
	unsigned int i, range_count = 0;
	int ret;

	if (!in || !geometry || !background || !end_of_tile || !in->check_owner ||
	    !in->vm || !in->vm->check || !in->owner.asid || !in->owner.generation ||
	    in->owner.parameter_id >= GPU_M4_RENDER_PARAMETER_SLOT_COUNT ||
	    !in->uuid || (in->flags & ~GPU_M4_RENDER_PROCESS_EMPTY_TILES) ||
	    (in->ppp_control & ~0x203U) ||
	    in->geometry.width != 32 || in->geometry.height != 32 ||
	    in->geometry.layers != 1 || in->geometry.samples != 1 ||
	    in->geometry.sample_size != 8 || !in->usc_base || (in->usc_base & 0xffffffffULL) ||
	    in->usc_base != in->owner.queue_usc_base || in->usc_base < in->vm->start ||
	    in->usc_base >= in->vm->end || 0x100000000ULL > in->vm->end - in->usc_base ||
	    !gpu_m4_exec_fw_range(in->owner.parameter_manager_fw, 0xc0, 8) ||
	    !gpu_m4_exec_fw_range(in->owner.parameter_descriptor_fw, 16, 8))
		return -EINVAL;
	firmware[0] = (struct gpu_m4_exec_interval){ in->owner.parameter_manager_fw, 0xc0 };
	firmware[1] = (struct gpu_m4_exec_interval){ in->owner.parameter_descriptor_fw, 16 };
	if (!gpu_m4_exec_disjoint(firmware, 2))
		return -EINVAL;
	/* One layer uses the metadata base, not a second unrelated allocation. */
	if (in->resource[GPU_M4_RENDER_LAYER_METADATA].va !=
	    in->resource[GPU_M4_RENDER_SCENE_METADATA].va ||
	    in->resource[GPU_M4_RENDER_LAYER_METADATA].size !=
	    in->resource[GPU_M4_RENDER_SCENE_METADATA].size)
		return -EINVAL;
	ret = gpu_m4_render_geometry(geometry, &in->geometry);
	if (ret)
		return ret;
	ret = in->check_owner(in->owner_cookie, &in->owner);
	if (ret)
		return ret;
	for (i = 0; i < GPU_M4_RENDER_RESOURCE_COUNT; i++) {
		u64 minimum = 0x4000;
		u32 alignment = 0x4000;

		if (i == GPU_M4_RENDER_TILEMAP)
			minimum = geometry->tilemap_size;
		else if (i == GPU_M4_RENDER_TPC)
			minimum = geometry->tpc_size;
		else if (i == GPU_M4_RENDER_COLOR)
			minimum = 32 * 32 * 4;
		else if (i == GPU_M4_RENDER_PARAMETER_HEAP) {
			minimum = GPU_M4_RENDER_PARAMETER_MIN;
			alignment = 0x20000;
		} else if (i == GPU_M4_RENDER_PREEMPT_0 || i == GPU_M4_RENDER_PREEMPT_1 ||
			 i == GPU_M4_RENDER_FRAGMENT_AUXILIARY)
			minimum = 0x10000;
		else if (i == GPU_M4_RENDER_VDM) {
			minimum = 4;
			alignment = 4;
		}
		ret = gpu_m4_render_span(in->vm, &in->resource[i], minimum,
					 alignment, i != GPU_M4_RENDER_VDM);
		if (ret)
			return ret;
		if (i != GPU_M4_RENDER_LAYER_METADATA)
			ranges[range_count++] = (struct gpu_m4_exec_interval){
				in->resource[i].va, in->resource[i].size };
	}
	if (!gpu_m4_exec_disjoint(ranges, range_count))
		return -EINVAL;
	for (i = 0; i < sizeof(compact) / sizeof(compact[0]); i++) {
		ret = gpu_m4_render_compact(&in->resource[compact[i]], &unused);
		if (ret)
			return ret;
	}
	ret = gpu_m4_render_span(in->vm, &in->scissor, 4, 4, false);
	if (ret)
		return ret;
	if (in->depth_bias.va || in->depth_bias.size) {
		ret = gpu_m4_render_span(in->vm, &in->depth_bias, 4, 4, false);
		if (ret)
			return ret;
	} else if (in->depth_bias_is_integer) {
		return -EINVAL;
	}
	if (in->query.va || in->query.size) {
		ret = gpu_m4_render_span(in->vm, &in->query, 16, 16, true);
		if (ret)
			return ret;
	}
	ret = gpu_m4_render_usc_span(in, &in->vertex_program);
	if (ret)
		return ret;
	ret = gpu_m4_render_usc_span(in, &in->fragment_program);
	if (ret)
		return ret;
	ret = gpu_m4_render_program(in, &in->background, background);
	if (ret)
		return ret;
	return gpu_m4_render_program(in, &in->end_of_tile, end_of_tile);
}

enum gpu_m4_render_first_fw_resource {
	GPU_M4_RENDER_FIRST_SCENE_FW,
	GPU_M4_RENDER_FIRST_SCRATCH_FW,
	GPU_M4_RENDER_FIRST_UMA_FW,
	GPU_M4_RENDER_FIRST_PB_LIST_FW,
	GPU_M4_RENDER_FIRST_PB_BLOCKS_FW,
	GPU_M4_RENDER_FIRST_PB_RING_FW,
	GPU_M4_RENDER_FIRST_PB_COUNTER_FW,
	GPU_M4_RENDER_FIRST_NOTIFIERS_FW,
	GPU_M4_RENDER_FIRST_TILER_ACTIVITY_FW,
	GPU_M4_RENDER_FIRST_FRAGMENT_ACTIVITY_FW,
	GPU_M4_RENDER_FIRST_TILER_WORK_FW,
	GPU_M4_RENDER_FIRST_FRAGMENT_WORK_FW,
	GPU_M4_RENDER_FIRST_TILER_SEQUENCE_FW,
	GPU_M4_RENDER_FIRST_FRAGMENT_SEQUENCE_FW,
	GPU_M4_RENDER_FIRST_TILER_QUEUE_FW,
	GPU_M4_RENDER_FIRST_FRAGMENT_QUEUE_FW,
	GPU_M4_RENDER_FIRST_FW_COUNT,
};

enum gpu_m4_render_first_gpu_resource {
	GPU_M4_RENDER_FIRST_PRIVATE_POOL_GPU,
	GPU_M4_RENDER_FIRST_POOL_STATE_GPU,
	GPU_M4_RENDER_FIRST_UMA_LIST_GPU,
	GPU_M4_RENDER_FIRST_PB_LIST_GPU,
	GPU_M4_RENDER_FIRST_UMA_GPU,
	GPU_M4_RENDER_FIRST_TILER_WORK_GPU,
	GPU_M4_RENDER_FIRST_FRAGMENT_WORK_GPU,
	GPU_M4_RENDER_FIRST_GPU_COUNT,
};

struct gpu_m4_render_first_owner_input;
/*
 * Under a frozen owner lock, prove real leases for UMA slot 33 and PB slot 0,
 * fresh backing, fresh hardware descriptor and retained notifier cells, and
 * exact same-PA aliases scene/scratch/UMA/PB-list plus each command's firmware,
 * client and context-0 register-fetch views. Prove all other physical backing
 * distinct (including the first compute owner). Neither zero table words nor
 * the address-only disjoint checks below establish a lease or an alias.
 */
typedef int (*gpu_m4_render_check_first_owner)(void *cookie,
		const struct gpu_m4_render_first_owner_input *owner);

struct gpu_m4_render_first_owner_input {
	const struct gpu_m4_render_input *render;
	struct gpu_m4_render_span firmware[GPU_M4_RENDER_FIRST_FW_COUNT];
	struct gpu_m4_render_span gpu[GPU_M4_RENDER_FIRST_GPU_COUNT];
	u32 uma_slot;
	void *owner_cookie;
	gpu_m4_render_check_first_owner check_owner;
};

/*
 * Detached fresh bytes, not live owner storage. Large output belongs in owned
 * heap/backing, never on a kernel stack. Actual firmware-mutated statistics,
 * global descriptor tables and notification pages are not rewritten here.
 */
struct gpu_m4_render_first_owner_image {
	u8 scene[GPU_M4_RENDER_SCENE_SIZE];
	u8 uma[0x4000];
	u8 freelist[GPU_M4_EXEC_POOL_STATE_SIZE];
	u8 uma_page_list[GPU_M4_EXEC_POOL_PAGE_LIST_SIZE];
	u8 parameter_manager[GPU_M4_RENDER_PARAMETER_MANAGER_SIZE];
	u8 pb_page_list[GPU_M4_RENDER_PB_LIST_SIZE];
	u8 pb_blocks[GPU_M4_RENDER_PB_LIST_SIZE];
	u8 pb_ring[8];
	u8 pb_counter[8];
	u8 init_buffer[GPU_M4_RENDER_INIT_BUFFER_SIZE];
};

/* Literal fresh PB bytes for the already validated actual capacity and owned
 * detached buffers. The runtime supplies pages/block storage of the checked
 * layout sizes. NEVER apply this to firmware-mutated lists/manager in place:
 * actual retirement/displacement and a full InitBuffer rebuild are required.
 * The hardware descriptor is FW-owned and deliberately not written here.
 */
static inline void gpu_m4_render_pb_emit(const struct gpu_m4_render_pb_layout *p,
		const struct gpu_m4_render_owner *owner, u64 heap_gpu,
		const struct gpu_m4_render_span *fw, const struct gpu_m4_render_span *gpu,
		u8 *manager, u8 *pages, u8 *blocks, u8 *ring)
{
	u64 compact = heap_gpu - GPU_M4_RENDER_RANGE1_START;

	memset(manager, 0, GPU_M4_RENDER_PARAMETER_MANAGER_SIZE);
	memset(blocks, 0, p->block_list_size);
	for (u32 i = 0; i < p->pages; i++)
		put_unaligned_le32((compact + (u64)i * 0x8000) >> 15, pages + i * 4);
	for (u32 i = 0; i < p->blocks; i++)
		put_unaligned_le32((compact + (u64)i * 0x20000) >> 15, blocks + i * 8);
	put_unaligned_le32(p->blocks, ring);
	put_unaligned_le32(p->blocks, ring + 4);
	put_unaligned_le32(owner->parameter_id, manager + 0xc);
	put_unaligned_le32(owner->parameter_id, manager + 0x10);
	put_unaligned_le64(fw[GPU_M4_RENDER_FIRST_PB_LIST_FW].va, manager + 0x20);
	put_unaligned_le64(gpu[GPU_M4_RENDER_FIRST_PB_LIST_GPU].va, manager + 0x28);
	put_unaligned_le32(p->page_list_size, manager + 0x30);
	put_unaligned_le32(p->pages, manager + 0x34);
	put_unaligned_le32(p->block_capacity, manager + 0x38);
	put_unaligned_le32(p->blocks, manager + 0x3c);
	put_unaligned_le64(fw[GPU_M4_RENDER_FIRST_PB_BLOCKS_FW].va, manager + 0x44);
	put_unaligned_le64(fw[GPU_M4_RENDER_FIRST_PB_RING_FW].va, manager + 0x4c);
	put_unaligned_le32(p->pages - 1, manager + 0x54);
	put_unaligned_le32(0x20000, manager + 0x58);
	put_unaligned_le64(fw[GPU_M4_RENDER_FIRST_PB_COUNTER_FW].va, manager + 0x64);
	put_unaligned_le32(p->pages, manager + 0x7c);
	put_unaligned_le32(p->pages, manager + 0x80);
}

/*
 * Address/VM checks shared by fresh and privately leased persistent packets.
 * This establishes no hardware lease and emits no allocator state.
 */
static inline int gpu_m4_render_owner_bindings_validate_generation(
		const struct gpu_m4_render_first_owner_input *in, u64 generation)
{
	static const u32 fw_minimum[GPU_M4_RENDER_FIRST_FW_COUNT] = {
		0x4000, 0x4000, 0x4000, 512, 512, 8, 8, 0x1000,
		0xc18, 0x1248, 0x940, 0xca0, 0x300, 0x380, 0x24c0, 0x24c0,
	};
	static const u32 gpu_minimum[GPU_M4_RENDER_FIRST_GPU_COUNT] = {
		GPU_M4_EXEC_POOL_SIZE, 0x20000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000,
	};
	struct gpu_m4_exec_interval firmware[GPU_M4_RENDER_FIRST_FW_COUNT + 2];
	struct gpu_m4_exec_interval gpu[GPU_M4_RENDER_RESOURCE_COUNT - 1 +
		GPU_M4_RENDER_FIRST_GPU_COUNT];
	struct gpu_m4_exec_geometry geometry;
	u64 background, end_of_tile, compact;
	unsigned int i, count = 0;
	int ret;

	/* Check the wide frozen value before any comparison with its u8 copy. */
	if (!generation || generation > 255 || !in || !in->render ||
	    !gpu_m4_exec_user_asid(in->render->owner.asid) ||
	    in->render->owner.context != in->render->owner.asid ||
	    in->render->owner.generation != generation ||
	    in->render->geometry.utile_width != 32 || in->render->geometry.utile_height != 32 ||
	    in->render->resource[GPU_M4_RENDER_PARAMETER_HEAP].size != GPU_M4_RENDER_PARAMETER_MIN)
		return -EINVAL;
	ret = gpu_m4_render_validate(in->render, &geometry, &background, &end_of_tile);
	if (ret)
		return ret;
	for (i = 0; i < GPU_M4_RENDER_FIRST_FW_COUNT; i++) {
		if (in->firmware[i].size < fw_minimum[i] ||
		    !gpu_m4_exec_fw_range(in->firmware[i].va, in->firmware[i].size, 8))
			return -EINVAL;
		firmware[i] = (struct gpu_m4_exec_interval){
			in->firmware[i].va, in->firmware[i].size };
	}
	firmware[i++] = (struct gpu_m4_exec_interval){
		in->render->owner.parameter_manager_fw, GPU_M4_RENDER_PARAMETER_MANAGER_SIZE };
	firmware[i++] = (struct gpu_m4_exec_interval){
		in->render->owner.parameter_descriptor_fw, GPU_M4_RENDER_PARAMETER_DESCRIPTOR_SIZE };
	if (!gpu_m4_exec_disjoint(firmware, i))
		return -EINVAL;
	for (i = 0; i < GPU_M4_RENDER_RESOURCE_COUNT; i++)
		if (i != GPU_M4_RENDER_LAYER_METADATA)
			gpu[count++] = (struct gpu_m4_exec_interval){
				in->render->resource[i].va, in->render->resource[i].size };
	for (i = 0; i < GPU_M4_RENDER_FIRST_GPU_COUNT; i++) {
		if (in->gpu[i].size != gpu_minimum[i])
			return -EINVAL;
		ret = gpu_m4_render_span(in->render->vm, &in->gpu[i], gpu_minimum[i], 0x4000, true);
		if (ret)
			return ret;
		gpu[count++] = (struct gpu_m4_exec_interval){ in->gpu[i].va, in->gpu[i].size };
	}
	if (!gpu_m4_exec_disjoint(gpu, count))
		return -EINVAL;
	ret = gpu_m4_render_compact(&in->render->resource[GPU_M4_RENDER_PARAMETER_HEAP], &compact);
	if (ret)
		return ret;
	return 0;
}

/* The existing fresh/persistent gen1 fixture entrance stays exact. */
static inline int gpu_m4_render_owner_bindings_validate(
		const struct gpu_m4_render_first_owner_input *in)
{
	return gpu_m4_render_owner_bindings_validate_generation(in, 1);
}

/* Shape/ranges only; the packet callback still proves actual binding ownership. */
static inline int gpu_m4_render_owner_bindings_validate_retained(
		const struct gpu_m4_render_first_owner_input *in,
		const struct gpu_m4_render_retained_binding *binding)
{
	if (!binding || !binding->lease || !binding->version || !binding->epoch)
		return -EINVAL;
	return gpu_m4_render_owner_bindings_validate_generation(in, binding->generation);
}

static inline int gpu_m4_render_first_owner_validate(
		const struct gpu_m4_render_first_owner_input *in)
{
	int ret;

	if (!in || !in->render || !in->check_owner ||
	    in->uma_slot != GPU_M4_RENDER_FIRST_UMA_SLOT ||
	    in->render->owner.parameter_id != 0)
		return -EINVAL;
	ret = gpu_m4_render_owner_bindings_validate(in);
	if (ret)
		return ret;
	return in->check_owner(in->owner_cookie, in);
}

/*
 * Every fallible owner/range check precedes the first output byte. No leases,
 * page tables, notifier thresholds, ring producers, descriptors or kicks change.
 * The images remain retained through both stages and firmware's last-owner era.
 */
static inline int gpu_m4_render_first_owner_prepare(
		struct gpu_m4_render_first_owner_image *output,
		const struct gpu_m4_render_first_owner_input *in)
{
	const struct gpu_m4_render_span *fw, *gpu;
	const struct gpu_m4_render_input *render;
	u8 init[GPU_M4_RENDER_INIT_BUFFER_SIZE];
	u8 *manager, *uma;
	u64 compact_scene_list;
	struct gpu_m4_render_pb_layout pb;
	unsigned int i;
	int ret;

	if (!output)
		return -EINVAL;
	ret = gpu_m4_render_first_owner_validate(in);
	if (ret)
		return ret;
	render = in->render;
	fw = in->firmware;
	gpu = in->gpu;
	ret = gpu_m4_render_init_buffer(init, &render->owner, render->owner_cookie,
					render->check_owner, 0x100);
	if (ret)
		return ret;
	ret = gpu_m4_render_pb_capacity(&pb, render->resource[GPU_M4_RENDER_PARAMETER_HEAP].size);
	if (ret)
		return ret;
	compact_scene_list = render->resource[GPU_M4_RENDER_SCENE_LIST].va - GPU_M4_RENDER_RANGE1_START;
	memset(output, 0, sizeof(*output));
	memcpy(output->init_buffer, init, sizeof(init));
	put_unaligned_le64(render->resource[GPU_M4_RENDER_SCRATCH].va, output->scene);
	put_unaligned_le64(fw[GPU_M4_RENDER_FIRST_SCRATCH_FW].va, output->scene + 8);
	put_unaligned_le64(compact_scene_list, output->scene + 0x28);
	put_unaligned_le64(fw[GPU_M4_RENDER_FIRST_SCENE_FW].va + 0x200, output->scene + 0x40);
	put_unaligned_le64(2, output->scene + 0x500);
	uma = output->uma;
	put_unaligned_le64(fw[GPU_M4_RENDER_FIRST_UMA_FW].va, uma);
	put_unaligned_le32(GPU_M4_RENDER_FIRST_UMA_SLOT, uma + 8);
	put_unaligned_le32(2, uma + 0x10);
	put_unaligned_le64(gpu[GPU_M4_RENDER_FIRST_POOL_STATE_GPU].va, uma + 0x14);
	put_unaligned_le32(16384, uma + 0x1c);
	put_unaligned_le32(8192, uma + 0x24);
	put_unaligned_le32(8192, uma + 0x2c);
	put_unaligned_le64(gpu[GPU_M4_RENDER_FIRST_UMA_LIST_GPU].va, uma + 0x30);
	put_unaligned_le32(4, uma + 0x40);
	put_unaligned_le32(128, uma + 0x48);
	put_unaligned_le64(fw[GPU_M4_RENDER_FIRST_SCENE_FW].va + 0x500, uma + 0x4c);
	/* +0x54 completed remains zero; +0x5c=0 permits paired render engines. */
	put_unaligned_le32(1, uma + 0x84);
	put_unaligned_le32(2, uma + 0x94);
	for (i = 0; i < 8192; i++)
		put_unaligned_le64(gpu[GPU_M4_RENDER_FIRST_PRIVATE_POOL_GPU].va + (u64)i * 0x1000,
				   output->freelist + i * 8);
	for (i = 0; i < 16; i++)
		put_unaligned_le64((1ULL << 61) |
			(gpu[GPU_M4_RENDER_FIRST_PRIVATE_POOL_GPU].va + (u64)i * 0x200000),
			output->uma_page_list + i * 64);
	manager = output->parameter_manager;
	gpu_m4_render_pb_emit(&pb, &render->owner,
		render->resource[GPU_M4_RENDER_PARAMETER_HEAP].va, fw, gpu, manager,
		output->pb_page_list, output->pb_blocks, output->pb_ring);
	return 0;
}

static inline void gpu_m4_render_record(u8 *output, unsigned int *index,
					u32 selector, u64 value)
{
	put_unaligned_le32(selector, output + *index * 12);
	put_unaligned_le64(value, output + *index * 12 + 4);
	(*index)++;
}

/* Literal ordered tables. The final selector is flagged, not its plain alias. */
static inline void gpu_m4_render_tiler_registers(u8 *out,
					const struct gpu_m4_render_encoded *in,
					const struct gpu_m4_exec_geometry *g,
					unsigned int *count)
{
	const struct gpu_m4_render_span *r = in->resource;
	u64 compact[GPU_M4_RENDER_ENCODED_RESOURCES];
	unsigned int i, n = 0;

	for (i = 0; i < GPU_M4_RENDER_ENCODED_RESOURCES; i++)
		compact[i] = r[i].va - GPU_M4_RENDER_RANGE1_START;
#define M4_TREG(s, v) gpu_m4_render_record(out, &n, (s), (v))
	M4_TREG(0x1748, 1); M4_TREG(0x10141, 0x200);
	M4_TREG(0x1c039, compact[GPU_M4_RENDER_TILEMAP]);
	M4_TREG(0x1c9c8, compact[GPU_M4_RENDER_TILEMAP]);
	M4_TREG(0x1c0a1, compact[GPU_M4_RENDER_TPC]);
	M4_TREG(0x1c031, compact[GPU_M4_RENDER_SCENE_METADATA] | (1ULL << 63));
	M4_TREG(0x1c9c0, compact[GPU_M4_RENDER_SCENE_METADATA] | (1ULL << 63));
	M4_TREG(0x1c051, 0x003a0012006b0003ULL); M4_TREG(0x1c061, 1);
	M4_TREG(0x10149, g->utile_control); M4_TREG(0x10139, in->multisample_control);
	M4_TREG(0x10111, compact[GPU_M4_RENDER_PREEMPT_0]);
	M4_TREG(0x1c9b0, compact[GPU_M4_RENDER_PREEMPT_0]);
	M4_TREG(0x10119, compact[GPU_M4_RENDER_PREEMPT_1]);
	M4_TREG(0x1c9b8, compact[GPU_M4_RENDER_PREEMPT_1]); M4_TREG(0x1c958, 1);
	M4_TREG(0x1c950, compact[GPU_M4_RENDER_PREEMPT_2] | (1ULL << 50));
	M4_TREG(0x1c930, 0); M4_TREG(0x1c880, compact[GPU_M4_RENDER_VDM] & ~3ULL);
	M4_TREG(0x1c079, compact[GPU_M4_RENDER_LAYER_METADATA]);
	M4_TREG(0x1c9d8, compact[GPU_M4_RENDER_LAYER_METADATA]);
	M4_TREG(0x10151, 0); M4_TREG(0x1c199, 0); M4_TREG(0x1c1a1, 0);
	M4_TREG(0x1c1a9, 0); M4_TREG(0x1c1b1, 0); M4_TREG(0x1c1b9, 0);
	M4_TREG(0x1c8f8, 0x8860); M4_TREG(0x1c0b1, g->region_stride);
	M4_TREG(0x1c850, g->region_stride); M4_TREG(0x10131, 0x88);
	M4_TREG(0x10121, 2 | (in->ppp_control & 0x201));
	M4_TREG(0x10129, g->pixels); M4_TREG(0x101b9, g->screen);
	M4_TREG(0x1c069, g->x_blocks); M4_TREG(0x1c071, g->y_blocks);
	M4_TREG(0x1c081, g->blocks); M4_TREG(0x1c0a9, g->tpc_stride);
	M4_TREG(0x10171, 0x100); M4_TREG(0x10169, gpu_m4_render_encoded_layer_control(in));
	M4_TREG(0xa309, 0); M4_TREG(0x1c8e0, ~0ULL); M4_TREG(0x1c8e8, ~0ULL);
	M4_TREG(0x1c898, 0); M4_TREG(0x101e1, 0x1c); M4_TREG(0x1c9e8, 0);
	M4_TREG(0x1a099, 0); M4_TREG(0x1a0a1, 0); M4_TREG(0x1a069, 0);
	M4_TREG(0x1a071, 0); M4_TREG(0x1a0c9, 0); M4_TREG(0x1a0d1, 0);
	M4_TREG(0x101c9, 0); M4_TREG(0xd471, 0); M4_TREG(0x1a0f1, 8);
	M4_TREG(0x10799, 0xff0000); M4_TREG(0x1c830, in->owner.parameter_id);
	M4_TREG(0x1ca30, compact[GPU_M4_RENDER_SCENE_LIST]);
	M4_TREG(0x16c39, compact[GPU_M4_RENDER_SCENE_LIST]);
	M4_TREG(0x1c910, 1 | ((r[GPU_M4_RENDER_SCRATCH].va +
		((r[GPU_M4_RENDER_SCRATCH].va & (1ULL << 42)) ? 0 : 0x7000000000ULL)) &
		0x7ffffffffeULL) | ((r[GPU_M4_RENDER_SCRATCH].va & (1ULL << 42)) >> 3));
	M4_TREG(0xa5a1, gpu_m4_render_alloc_word[0]); M4_TREG(0xd419, gpu_m4_render_alloc_minmax[0]);
	M4_TREG(0x1ca10, in->uuid); M4_TREG(0x14a1, in->uuid); M4_TREG(0xa349, in->uuid);
	M4_TREG(0x10061, in->usc_base);
#undef M4_TREG
	*count = n;
}

static inline void gpu_m4_render_fragment_registers(u8 *out,
					const struct gpu_m4_render_encoded *in,
					   const struct gpu_m4_exec_geometry *g,
					   u64 background, u64 end_of_tile,
					   unsigned int *count)
{
	const struct gpu_m4_render_span *r = in->resource;
	unsigned int n = 0;
	u64 framebuffer = ((u64)in->geometry.height << 32) | in->geometry.width;
	u64 packed = 0x3717fULL | ((u64)(in->geometry.layers > 1) << 32) |
		((u64)(g->utile_control >> 12) << 40) |
		(0x200ULL << 28) | ((u64)(g->screen & 0x1ff) << 44) |
		((u64)((g->screen >> 12) & 0x1ff) << 53);

#define M4_FREG(s, v) gpu_m4_render_record(out, &n, (s), (v))
	M4_FREG(0x1739, 1); M4_FREG(0x10009, g->utile_control);
	M4_FREG(0x15379, in->end_of_tile_resources); M4_FREG(0x15381, end_of_tile);
	M4_FREG(0x15369, in->background_resources); M4_FREG(0x15371, background);
	M4_FREG(0x15131, in->merge_limit[0]); M4_FREG(0x15139, in->merge_limit[1]);
	M4_FREG(0x100a1, 0); M4_FREG(0x15069, 0); M4_FREG(0x15071, 0); M4_FREG(0x16058, 0);
	M4_FREG(0x10019, in->multisample_control); M4_FREG(0x100b1, g->isp_blocks);
	M4_FREG(0x16030, g->isp_blocks); M4_FREG(0x100d9, g->screen);
	M4_FREG(0xa301, 0); M4_FREG(0x10791, 0xff0200);
	M4_FREG(0x16098, r[GPU_M4_RENDER_SCENE_METADATA].va & ~63ULL);
	M4_FREG(0x15109, in->scissor.va & ~3ULL); M4_FREG(0x15101, in->depth_bias.va & ~3ULL);
	M4_FREG(0x15021, 0xc000 | (in->depth_bias_is_integer ? 1ULL << 18 : 0));
	M4_FREG(0x15211, framebuffer);
	M4_FREG(0x15049, 0x100000); M4_FREG(0x10051, g->tilebuffer_control);
	M4_FREG(0x15321, in->zls_pixels); M4_FREG(0x15301, in->depth_clear);
	M4_FREG(0x15309, in->background_value);
	M4_FREG(0x15311, in->query.va & ~15ULL);
	M4_FREG(0x15319, in->zls_control); M4_FREG(0x15349, 0x04040404);
	M4_FREG(0x15351, 0); M4_FREG(0x15329, in->depth.va); M4_FREG(0x15331, in->depth.va);
	M4_FREG(0x15339, in->stencil.va); M4_FREG(0x15341, in->stencil.va); M4_FREG(0x15231, 0);
	M4_FREG(0x15221, 0); M4_FREG(0x15239, 0); M4_FREG(0x15229, 0);
	M4_FREG(0x15401, in->depth_stride); M4_FREG(0x15421, in->depth_stride);
	M4_FREG(0x15409, in->stencil_stride); M4_FREG(0x15429, in->stencil_stride);
	M4_FREG(0x153c1, 0); M4_FREG(0x15411, 0); M4_FREG(0x153c9, 0); M4_FREG(0x15431, 0);
	M4_FREG(0x153d1, 0); M4_FREG(0x15419, 0); M4_FREG(0x153d9, 0); M4_FREG(0x15439, 0);
	M4_FREG(0x16429, r[GPU_M4_RENDER_TILEMAP].va);
	M4_FREG(0x16060, r[GPU_M4_RENDER_LAYER_METADATA].va);
	M4_FREG(0x16431, (u64)g->region_stride << 26);
	M4_FREG(0x10039, gpu_m4_render_encoded_isp_mode(in));
	M4_FREG(0x16020, 0); M4_FREG(0x16451, 0); M4_FREG(0x15359, 0);
	M4_FREG(0x100b8, 0x8860); M4_FREG(0x16461, r[GPU_M4_RENDER_FRAGMENT_AUXILIARY].va);
	M4_FREG(0x16090, r[GPU_M4_RENDER_FRAGMENT_AUXILIARY].va);
	M4_FREG(0x101e9, 0x1c); M4_FREG(0x160a8, 0); M4_FREG(0x16068, packed);
	M4_FREG(0x1a0a9, 0); M4_FREG(0x1a0b1, 0); M4_FREG(0x1a079, 0);
	M4_FREG(0x1a081, 0); M4_FREG(0x1a0d9, 0); M4_FREG(0x1a0e1, 0);
	M4_FREG(0x101c1, 0); M4_FREG(0xd469, 0); M4_FREG(0x1a0f9, 8);
	M4_FREG(0xa5a9, gpu_m4_render_alloc_word[1]); M4_FREG(0xd429, gpu_m4_render_alloc_minmax[1]);
	M4_FREG(0x160e0, in->uuid); M4_FREG(0x1499, in->uuid); M4_FREG(0xa341, in->uuid);
	M4_FREG(0x1c838, in->owner.parameter_id);
	M4_FREG(0x1ca28, r[GPU_M4_RENDER_SCENE_LIST].va - GPU_M4_RENDER_RANGE1_START);
	M4_FREG(0x10069, in->usc_base);
#undef M4_FREG
	*count = n;
}

/*
 * Preparation only: caller holds its owner lock and freezes inputs/mappings.
 * Count includes the final USC selector and excludes the trailing zero suffix.
 * No full work or sequence is encoded here.
 * A failure leaves every byte of output untouched.
 */
static inline int gpu_m4_render_registers(struct gpu_m4_render_register_output *output,
					const struct gpu_m4_render_input *in,
					enum gpu_m4_exec_engine engine)
{
	struct gpu_m4_exec_geometry geometry;
	struct gpu_m4_render_encoded encoded;
	u8 attachments[GPU_M4_EXEC_ATTACHMENT_SIZE];
	u64 background, end_of_tile;
	unsigned int count;
	int ret;

	if (!output || (engine != GPU_M4_EXEC_TILER && engine != GPU_M4_EXEC_FRAGMENT))
		return -EINVAL;
	ret = gpu_m4_render_validate(in, &geometry, &background, &end_of_tile);
	if (ret)
		return ret;
	ret = gpu_m4_exec_attachments(attachments, in->vm, in->attachments[engine],
				      in->attachment_count[engine]);
	if (ret)
		return ret;
	/* Fragment must publish the entire retained color allocation as writable. */
	if (engine == GPU_M4_EXEC_FRAGMENT) {
		const struct gpu_m4_render_span *color = &in->resource[GPU_M4_RENDER_COLOR];
		bool covered = false;
		unsigned int i;

		for (i = 0; i < in->attachment_count[engine]; i++) {
			const struct gpu_m4_exec_attachment *a = &in->attachments[engine][i];

			if (a->va <= color->va && a->size >= color->size &&
			    color->va - a->va <= a->size - color->size)
				covered = true;
		}
		if (!covered)
			return -EINVAL;
	}
	gpu_m4_render_encoded_fixture(&encoded, in);
	memset(output, 0, sizeof(*output));
	memcpy(output->attachments, attachments, sizeof(attachments));
	output->geometry = geometry;
	if (engine == GPU_M4_EXEC_TILER)
		gpu_m4_render_tiler_registers(output->registers, &encoded, &geometry, &count);
	else
		gpu_m4_render_fragment_registers(output->registers, &encoded, &geometry,
						background, end_of_tile, &count);
	output->count = count;
	output->bytes = count * 12;
	output->zero_suffix_records = engine == GPU_M4_EXEC_TILER ? 3 : 2;
	return 0;
}

/* The caller places these 12 bytes at work register-base + 0x700. */
static inline int gpu_m4_render_register_metadata(void *output,
						 const struct gpu_m4_exec_vm *vm,
						 u64 register_gpu,
						 const struct gpu_m4_render_register_output *encoded,
						 enum gpu_m4_exec_engine engine)
{
	unsigned int count, suffix, i;
	int ret;

	if (!output || !encoded || (register_gpu & 63) ||
	    (engine != GPU_M4_EXEC_TILER && engine != GPU_M4_EXEC_FRAGMENT))
		return -EINVAL;
	count = engine == GPU_M4_EXEC_TILER ?
		GPU_M4_RENDER_TILER_REGISTERS : GPU_M4_RENDER_FRAGMENT_REGISTERS;
	suffix = engine == GPU_M4_EXEC_TILER ? 3 : 2;
	if (encoded->count != count || encoded->bytes != count * 12 ||
	    encoded->zero_suffix_records != suffix)
		return -EINVAL;
	for (i = count * 12; i < GPU_M4_RENDER_REGISTER_SIZE; i++)
		if (encoded->registers[i])
			return -EINVAL;
	ret = gpu_m4_exec_vm_range(vm, register_gpu, GPU_M4_RENDER_REGISTER_SIZE, false);
	if (ret)
		return ret;
	put_unaligned_le64(register_gpu, output);
	put_unaligned_le16(count, (u8 *)output + 8);
	put_unaligned_le16(count * 12, (u8 *)output + 10);
	return 0;
}

#endif
