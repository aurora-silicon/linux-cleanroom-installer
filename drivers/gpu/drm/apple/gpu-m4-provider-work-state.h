/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_PROVIDER_WORK_STATE_H
#define APPLE_GPU_M4_PROVIDER_WORK_STATE_H

#include "gpu-m4-exec.h"

enum gpu_m4_provider_work_phase {
	GPU_M4_PROVIDER_WORK_STAGING,
	GPU_M4_PROVIDER_WORK_PREPARED,
	GPU_M4_PROVIDER_WORK_STAGED,
	GPU_M4_PROVIDER_WORK_RUNNING,
	GPU_M4_PROVIDER_WORK_RETIRED,
	GPU_M4_PROVIDER_WORK_FAULTED,
};

/* NC observations only. This deliberately cannot qualify WB visibility or
 * signal a fence. The actual owner also proves tables, roots, aliases, leases,
 * unmodified idle engines and registered context under its hardware mutex.
 */
static inline int gpu_m4_provider_work_progress(
	const struct gpu_m4_exec_retirement *r, u32 capacity, u64 threshold)
{
	if (!r || capacity != 256 || threshold != 1 || r->expected_stamp != 0x100 ||
	    r->next != 1 || r->pipe_next != 1 || r->pipe_capacity != 256 ||
	    r->queue_read > 1 || r->queue_write != 1 || r->queue_busy > 1 ||
	    r->pipe_read > 1 || r->pipe_cfi > 1 || r->pipe_write != 1 ||
	    r->uma_submitted != 1 || r->uma_completed > 1 ||
	    (r->firmware_stamp && r->firmware_stamp != 0x100) ||
	    (r->user_stamp && r->user_stamp != 0x100) ||
	    (r->timestamp_end && r->timestamp_end <= r->timestamp_start))
		return -EPROTO;
	for (unsigned int i = 0; i < 3; i++)
		if (r->queue_consumer[i] > 1)
			return -EPROTO;
	return 0;
}

/* Every NC field must be fully retired continuously for 50ms, and the sample
 * itself must still fall within the two-second first-producer deadline. Time
 * regressions and overflow are failures, not an alternate success entrance.
 */
static inline int gpu_m4_provider_work_stable(
	const struct gpu_m4_exec_retirement *r, u64 published, u64 stable, u64 now)
{
	int ret;

	if (!published || now < published || now - published >= 2000000000ULL ||
	    (stable && (stable < published || stable > now)))
		return -ETIMEDOUT;
	ret = gpu_m4_provider_work_progress(r, 256, 1);
	if (ret)
		return ret;
	ret = gpu_m4_exec_retired(r);
	if (ret)
		return ret;
	return stable && now - stable >= 50000000ULL ? 0 : -EAGAIN;
}

#endif
