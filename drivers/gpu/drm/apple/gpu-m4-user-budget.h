/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_USER_BUDGET_H
#define APPLE_GPU_M4_USER_BUDGET_H

/* Finite host SAFETY bound, not a firmware constant and not a byte policy.
 * The working driver has no global quota over private owners: in-flight units
 * and each global slot's firmware-last owner are bounded structurally (window
 * depth, 32 + 32 slots, and here the 192 fixed work cells: at most 96 paired
 * render or 192 compute owners of ~34-43 MiB), and idle displaced units are a
 * cache with its own budgets (M4_USER_NATIVE_IDLE_LIMIT and per cache),
 * trimmed before any refusal. That is the only byte policy. Backing beyond
 * the units' own (old composed images until residency moves on, completed
 * receipts' stages) is released by its holders, never by this bound. Eight
 * GiB sits above that structural ceiling plus such transients; reaching it
 * refuses only the constructing job (capacity_backpressure), never other
 * work. It is not permission to reclaim published, current or closed-client
 * owners. The factory and runtime must agree; this budget alone bounds the
 * private backing. The provider separately charges only its own pin
 * bookkeeping, snapshots and tables, never backing pages.
 */
#define M4_ORDINARY_NATIVE_MEMORY_LIMIT (8ULL << 30)
#define M4_ORDINARY_WORK_CELLS 368U
/* Arena cells after the work cells hold logical-queue notifier records,
 * 16 notifiers per cell, never recycled (see m4_user_notifier_records).
 */
#define M4_NOTIFY_RECORD_CELLS 16U
#define M4_NOTIFY_RECORDS_PER_CELL 16U
#define M4_NOTIFY_RECORD_SPAN 0x400U
#define M4_NOTIFY_RECORD_STRIDE 0x140U
/* Conservative per-NEW-unit backing estimate, for the native backing budgets
 * (runtime native and GEM factory; never provider metadata): one 32-MiB
 * allocator pool plus its smaller private/firmware owners. A render unit adds
 * its geometry-sized tilemap/TPC and whole PB heap. Admission/eviction policy
 * only; the real charges at construction remain the authority.
 */
#define M4_ORDINARY_NATIVE_UNIT_BYTES (GPU_M4_EXEC_POOL_SIZE + (4ULL << 20))

#endif
