/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_USER_JOB_H
#define APPLE_GPU_M4_USER_JOB_H

struct m4_user_queue;
struct m4_user_native_stage;
struct m4_user_compute_reservation;
struct m4_user_compute_packet;
struct m4_user_render_reservation;
struct m4_user_render_packet;
struct gpu_m4_render_ready_backing;
struct m4_user_notifier;

/* stage_cells: the runtime-wide stage (Finalize) cell of engine e's held
 * event (TA 0x70, FS 0x71, compute 15), the session queue's NC FWSTAMP
 * word; 0 when the notifier's private cells are in use. The firmware stores
 * a held event's targets in node order and drains buffered out-of-order
 * completions through the destination of the completion closing the gap, so
 * all works of the event (of every logical queue) name this one cell. Never
 * reset after first use: it is the event's in-order frontier.
 */
static u64 m4_user_stage_fw(const struct m4_gpu_provider_runtime *p, u32 engine)
{
	return p->user_stage_cells && engine < 3 ?
		p->execution->queue[engine][M4_EXEC_QUEUE_FWSTAMP].va : 0;
}

static u32 m4_user_stage_read(const struct m4_gpu_provider_runtime *p, u32 engine)
{
	return m4_ready_read32(p->execution->queue[engine][M4_EXEC_QUEUE_FWSTAMP].cpu);
}

/* The event's exposed tail bounding the cell (0: private cells): the newest
 * target actually made reachable to the firmware, not a reservation. Read
 * under the GPU mutex, like every publication that advances it.
 */
static u32 m4_user_stage_tail(const struct m4_gpu_provider_runtime *p, u32 engine)
{
	return p->user_stage_cells && engine < 3 ? p->user_stage_exposed[engine] : 0;
}

/* The cell a wait on a compute predecessor names: its stage cell. */
static u64 m4_user_compute_wait_fw(const struct m4_gpu_provider_runtime *p, u64 notifier_fw)
{
	u64 stage = m4_user_stage_fw(p, 2);

	return stage ? stage : notifier_fw + 0x640;
}

/* The cell a wait on a render predecessor's TA (0) or FS (1) names: that
 * notifier's service cell (+0x620/+0x630, published by notifier service),
 * or under render_stage_waits the event's runtime-wide stage cell.
 */
static u64 m4_user_render_wait_fw(const struct m4_gpu_provider_runtime *p, u64 notifier_fw,
	u32 engine)
{
	u64 stage = p->user_render_stage_waits ? m4_user_stage_fw(p, engine) : 0;

	return stage ? stage : notifier_fw + 0x620 + engine * 16;
}

/* A wait's predecessor cell: when it is a runtime-wide stage cell, its
 * value and the event's published tail (true); else nothing (false).
 */
static bool m4_user_stage_observe(const struct m4_gpu_provider_runtime *p, u64 stamp_fw,
	u32 *value, u32 *tail)
{
	for (u32 e = 0; p->user_stage_cells && e < 3; e++)
		if (stamp_fw == m4_user_stage_fw(p, e)) {
			*value = m4_user_stage_read(p, e);
			*tail = m4_user_stage_tail(p, e);
			return true;
		}
	return false;
}

/* inflight2 global-tail barrier: the last command of the job in flight
 * ahead of this one when it was prepared (the global in-flight tail). This
 * job's FIRST command waits on it with a strict opcode-4 record (+0x3c=1):
 * a render predecessor's fragment FW stamp (notifier +0x630, event 0x71), a
 * compute predecessor's compute FW stamp (+0x640, event 15). The stamp names
 * the PREDECESSOR's page: the notifier ref is held until this job retires
 * (or is released unpublished). No notifier: no predecessor, no barrier.
 */
struct m4_user_cross {
	struct m4_user_notifier *notifier;
	u64 stamp_fw;
	u32 target, event, engine;
	/* The predecessor is this queue's previous window of the same public
	 * submit and its last command is the render immediately before this
	 * window's first: fragment-only early tiling may cross the boundary.
	 */
	bool same_packet;
	/* barrier_relax: that last command has no API timestamp or end copy. */
	bool untimed;
	/* No global-tail wait (cross_none): a BARRIER_NONE compute-only
	 * submission without input syncs or API timestamps, overlapping only
	 * its queue's in-flight compute-only jobs.
	 */
	bool none;
};

/* A bounded internal packet uses the existing physical queues. A render
 * command always owns both engine prefixes; they are exposed together.
 */
struct m4_user_engine {
	u64 entries[4];
	u8 pipe_record[0x18];
	/* Global publication order on this engine (runtime ledger); 0 until
	 * the record's producer became reachable.
	 */
	u64 sequence;
	u32 queue_before, queue_next, pipe_before, pipe_next, count;
	bool new_queue;
	/* Batched packet member: this command's entries continue the preceding
	 * command's packet on this engine. It shares that packet's single
	 * submit-pipe record (pipe_before/pipe_next); its own queue ends stay
	 * its own. False for the first (or only) command of a packet.
	 */
	bool joined;
};

/* One command's retained publication record inside the protected child. Each
 * command keeps its OWN per-engine reservations (saved queue/pipe ends:
 * compute engine 2, render TA 0 and fragment 1) and leased UMA slot (render
 * also its PB ID, slot - 33) while successors are published, so its receipt is
 * validated against its own prefixes. published is set before the first
 * reachable write; receipt only after the full hardware predicate.
 */
struct m4_user_command {
	struct m4_user_engine wire[3];
	u32 uma_slot, engine_mask;
	/* Its final engine's reserved target stamp (render: FS 0x71 at
	 * notifier +0x630; compute: 15 at +0x640), copied from its reservation
	 * when published: a mixed window successor's strict edge names it.
	 */
	u32 final_target;
	bool published, receipt;
	/* Nonzero: published and unfinished when that recovery epoch began, so
	 * the firmware may have aborted it. It retires with its ordinary full
	 * receipt, or, once the firmware moved past it but that receipt still
	 * failed for a grace period since abort_ns, as aborted.
	 */
	u32 abort;
	u64 abort_ns, abort_wait_ns;
	/* Set only while its aborted receipt is taken (the compute predicate
	 * then observes it as aborted).
	 */
	bool abort_proven;
	/* REMOVE-DEBUG timing: raw CNTPCT (CNTFRQ units, 1 GHz on M4) of this
	 * command's packet publication and its observed FS end (internal 24 MHz
	 * ticks) at receipt.
	 */
	u64 publish_ticks, fs_end_ticks;
};

/* Runtime payload only. Acceptance, dependencies, protection, fences and
 * lane ownership remain in the existing provider job and lane cores.
 */
struct m4_user_job {
	struct list_head garbage;
	struct m4_gpu_provider_runtime *runtime;
	struct m4_user_queue *queue;
	struct gpu_m4_drm_provider_job *core_job;
	struct gpu_m4_drm_provider_lane_item item;
	struct m4_user_native_stage *native_stage;
	struct gpu_m4_drm_timestamp_alias_token *token;
	struct m4_user_compute_reservation *compute_reservation;
	struct m4_user_compute_packet *compute;
	struct m4_user_compute_reuse *compute_reuse;
	struct m4_user_render_reservation *render_reservation;
	struct m4_user_render_packet *render;
	struct gpu_m4_render_ready_backing *session;
	struct m4_user_native_stage *displaced[3];
	/* Disposal-only payload preallocated at acceptance. After actual CORE
	 * success bookkeeping it carries the displaced[] stage refs through the
	 * existing user_job_garbage reaper; this job keeps no copy of them.
	 */
	struct m4_user_job *disposal;
	/* Contiguous counts: [0, published_count) are reachable, [0,
	 * completed_count) carry a full per-command hardware receipt. Their
	 * difference never exceeds depth (1 = serial; 2 = compute-only child; up
	 * to the window for a render-only or, with mixed_window, mixed child). command_index names
	 * the current head (diagnostics only).
	 */
	struct m4_user_command cmd[GPU_M4_DRM_JOB_MAX_COMMANDS];
	u32 published_count, completed_count, depth;
	/* Batched render packet still open in this publication scope: the last
	 * wire_open published commands have installed images but no reachable
	 * ring entries yet. Zero outside m4_user_publish_window_locked().
	 */
	u32 wire_open;
	u32 command_count, command_index;
	u32 all_engines;
	u32 bank; /* host UMA slot bank held from acceptance (bank_held) */
	bool bank_held;
	u64 charge;
	u64 binding_epoch;
	u8 binding_generation;
	u32 asid; /* The queue notifier's (and VM's) user context. */
	struct m4_user_cross cross;
	u32 veto; /* REMOVE-DEBUG: latest admission veto + 1 (0: never vetoed) */
	/* A mixed render/compute child published ahead (mixed_window): every
	 * command after the first strictly waits on its immediate predecessor's
	 * final firmware stamp, of either class.
	 */
	bool mixed;
	bool prepared, published, retired;
	/* A command of this job was aborted by a firmware recovery: the job
	 * retires (every published command passed) and fails with this error.
	 */
	int abort_error;
	/* p->user_recoveries when this job was activated. */
	u32 recoveries;
};

/* REMOVE-DEBUG: attribution trail of the latest ordinary publish/retire call.
 * Each failing sub-check on the compute window, wire, native receipt and lane
 * paths records its exact source line with the observed and expected values,
 * innermost first; the job fault prints the trail. Failure paths only, under
 * the GPU mutex; -EAGAIN (pending) is not a failure. Never changes an error.
 * A call only names itself and advances generation: the first failure it
 * records resets the trail lazily (recorded = generation), and only a record
 * formats its text. A successful call writes no entry, count or text.
 */
#define M4_USER_WHY_DEPTH 16U
struct m4_user_why_entry {
	const char *file;
	int line, error;
	char text[256];
};
static struct {
	const void *job;
	const char *call;
	u32 count;
	u64 generation, recorded;
	struct m4_user_why_entry entry[M4_USER_WHY_DEPTH];
} m4_user_why __attribute__((unused));

static inline __attribute__((unused)) void m4_user_why_begin(const void *job, const char *call)
{
	m4_user_why.job = job;
	m4_user_why.call = call;
	m4_user_why.generation++;
}

/* Entries recorded by the latest call (0 until its first failure). */
static inline __attribute__((unused)) u32 m4_user_why_count(void)
{
	return m4_user_why.recorded == m4_user_why.generation ? m4_user_why.count : 0;
}

static inline __attribute__((unused)) struct m4_user_why_entry *m4_user_why_next(const char *file,
	int line, int error)
{
	struct m4_user_why_entry *e;
	const char *base = strrchr(file, '/');

	if (m4_user_why.recorded != m4_user_why.generation) {
		m4_user_why.recorded = m4_user_why.generation;
		m4_user_why.count = 0;
	}
	e = &m4_user_why.entry[m4_user_why.count++ % M4_USER_WHY_DEPTH];
	e->file = base ? base + 1 : file;
	e->line = line;
	e->error = error;
	return e;
}

#define M4_USER_WHY(err, fmt, ...) ({						\
	int __why = (err);							\
	if (__why && __why != -EAGAIN)						\
		snprintf(m4_user_why_next(__FILE__, __LINE__, __why)->text,	\
			 sizeof(m4_user_why.entry[0].text), fmt, ##__VA_ARGS__);	\
	__why; })
/* As M4_USER_WHY, also for -EAGAIN where it is a definite refusal. */
#define M4_USER_WHY_ANY(err, fmt, ...) ({					\
	int __why = (err);							\
	if (__why)								\
		snprintf(m4_user_why_next(__FILE__, __LINE__, __why)->text,	\
			 sizeof(m4_user_why.entry[0].text), fmt, ##__VA_ARGS__);	\
	__why; })
#define M4_WHY64(x) ((unsigned long long)(x))
/* A refused lane command transition, with the exact lane item inputs. */
#define M4_USER_WHY_LANE(j, err, what, index)					\
	M4_USER_WHY(err, "lane %s %u count=%u depth=%u state=%u published=%d/%u/%u retired=%d/%u/%u head=%llu retired_ns=%llu", \
		what, index, (j)->item.command_count, (j)->item.command_depth,	\
		(j)->item.state, (j)->item.timing.published,			\
		(j)->item.timing.published_count, (j)->item.timing.published_index, \
		(j)->item.timing.retired, (j)->item.timing.retired_count,	\
		(j)->item.timing.retired_index, M4_WHY64((j)->item.timing.head_ns),	\
		M4_WHY64((j)->item.timing.retired_ns))

#endif
