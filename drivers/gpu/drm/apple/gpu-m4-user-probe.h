/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_GPU_M4_USER_PROBE_H
#define APPLE_GPU_M4_USER_PROBE_H

#include "gpu-m4-wb-fixture.h"

/* REMOVE-DEBUG: a single caller of the ORDINARY frontend/runtime. These
 * baseline references, including every partially constructed owner, survive
 * until reboot. No alternate queue, publisher, cache operation or success
 * fence is implemented here. The root keeps public WB admission disabled.
 */
#define M4_USER_PROBE_CDM 0x6100200000ULL
#define M4_USER_PROBE_INPUT 0x6101000000ULL
#define M4_USER_PROBE_OUTPUT 0x6100500000ULL
#define M4_USER_PROBE_TS 6U
#define M4_USER_PROBE_OBJECTS 7U
#define M4_USER_PROBE_COOKIE 1U /* Synchronous cookie-table index, NOT a GEM handle. */

/* Filled from the newly generated, independently pinned legal-address bundle.
 * Keeping this separate never changes the old private-profile parser/gates.
 */
static const u8 m4_user_probe_bundle_sha256[32] = {
	0x4f, 0x6d, 0xee, 0xc4, 0x27, 0xc1, 0xdb, 0xa3,
	0x0d, 0x54, 0xf2, 0xb3, 0x21, 0xe5, 0x02, 0x77,
	0xb5, 0x17, 0x06, 0x7b, 0xd3, 0xa6, 0xce, 0x9d,
	0x45, 0xcc, 0xae, 0xa7, 0x91, 0x97, 0x4b, 0x8a,
};
static const u8 m4_user_probe_compiler_sha256[32] = {
	0x08, 0x38, 0x32, 0x34, 0x10, 0x46, 0xfc, 0xcf,
	0x80, 0x07, 0x67, 0x2f, 0x98, 0x3e, 0xf5, 0xd1,
	0xb8, 0x1c, 0x6a, 0xd8, 0x9f, 0x61, 0xed, 0xc3,
	0x07, 0xf8, 0x52, 0xee, 0x96, 0x10, 0xde, 0x3c,
};

struct m4_user_probe_bo {
	struct drm_gem_object *gem;
	struct gpu_m4_drm_provider_object *object;
	struct iosys_map map;
	u64 va, pa;
	u32 access;
	enum gpu_m4_drm_cpu_policy policy;
	bool mapped, initialized;
};

struct m4_user_probe {
	struct m4_gpu_provider_runtime *runtime;
	struct m4_gpu_hello *hello;
	struct gpu_m4_drm *frontend;
	struct gpu_m4_drm_vm *frontend_vm;
	struct m4_user_vm *vm;
	const struct firmware *firmware;
	struct gpu_m4_wb_fixture recipe;
	struct m4_user_probe_bo bo[M4_USER_PROBE_OBJECTS];
	struct gpu_m4_drm_probe_result result;
	struct gpu_m4_drm_provider_job *job; /* Get ONLY at initialized enqueue. */
	struct gpu_m4_drm_provider_version *version; /* Borrowed through held job. */
	struct {
		struct drm_asahi_cmd_header header;
		struct drm_asahi_cmd_compute compute;
	} command;
	u8 preimage[M4_USER_PROBE_OBJECTS][GPU_M4_DRM_PAGE_SIZE];
	u64 timestamp_ns[2], binding_epoch;
	u32 observed_output;
	int error;
	bool submitted, complete;
};

/* Exact tooling profile only; success borrows the retained firmware bytes.
 * Hashing authenticates this new compiler-produced artifact, not hardware.
 */
static int m4_user_probe_parse(const void *blob, size_t size,
	struct gpu_m4_wb_fixture *result)
{
	static const u8 magic[16] = "M4WB-FIXTURE";
	static const u64 va[GPU_M4_WB_RESOURCE_COUNT] = {
		GPU_M4_STORE_CONTAINER_VA, GPU_M4_STORE_PROGRAM_VA, M4_USER_PROBE_CDM,
		M4_USER_PROBE_INPUT, M4_USER_PROBE_OUTPUT, GPU_M4_STORE_USC_BASE,
	};
	static const u32 lengths[GPU_M4_WB_RESOURCE_COUNT] = {
		1088, 90, 48, GPU_M4_DRM_PAGE_SIZE, GPU_M4_DRM_PAGE_SIZE, GPU_M4_DRM_PAGE_SIZE,
	};
	static const u32 meta[16] = {
		GPU_M4_WB_XOR, 0, 0, 70, 70, 8, 1, 0, 0, 1, 1, 1, 1, 1, 1, 2,
	};
	static const struct gpu_m4_wb_challenge challenge[2] = {
		{ 0x13579bdf, 0x7e7ce22a, 0xdeadbeef },
		{ 0x2468ace1, 0x4943d514, 0x0badc0de },
	};
	struct gpu_m4_wb_fixture checked = { };
	const u8 *data = blob;
	u8 digest[32];
	u32 cursor = GPU_M4_WB_HEADER_SIZE;

	if (!data || !result || size != GPU_M4_WB_BUNDLE_SIZE ||
	    memcmp(data, magic, sizeof(magic)) ||
	    get_unaligned_le32(data + 0x10) != 1 ||
	    get_unaligned_le32(data + 0x14) != GPU_M4_WB_HEADER_SIZE ||
	    get_unaligned_le32(data + 0x18) != size ||
	    get_unaligned_le32(data + 0x1c) != GPU_M4_WB_RESOURCE_COUNT ||
	    get_unaligned_le64(data + 0x20) != GPU_M4_STORE_USC_BASE ||
	    get_unaligned_le64(data + 0x28) != GPU_M4_STORE_ENTRY_VA ||
	    get_unaligned_le64(data + 0x30) != M4_USER_PROBE_INPUT ||
	    get_unaligned_le64(data + 0x38) != M4_USER_PROBE_OUTPUT ||
	    get_unaligned_le64(data + 0x40) != M4_USER_PROBE_CDM + 48 ||
	    !gpu_m4_store_zero(data + 0xa0, 0x10))
		return -EINVAL;
	for (u32 i = 0; i < ARRAY_SIZE(meta); i++)
		if (get_unaligned_le32(data + 0x48 + i * 4) != meta[i])
			return -EINVAL;
	for (u32 i = 0; i < ARRAY_SIZE(challenge); i++) {
		const u8 *record = data + 0x88 + i * 12;

		if (get_unaligned_le32(record) != challenge[i].input ||
		    get_unaligned_le32(record + 4) != challenge[i].expected_output ||
		    get_unaligned_le32(record + 8) != challenge[i].sentinel)
			return -EINVAL;
		checked.challenge[i] = challenge[i];
	}
	for (u32 i = 0; i < GPU_M4_WB_RESOURCE_COUNT; i++) {
		const u8 *d = data + GPU_M4_WB_DESCRIPTOR_OFFSET + i * 56;
		u32 offset = get_unaligned_le32(d + 8), length = get_unaligned_le32(d + 12);
		u32 aligned = (cursor + 63) & ~63U;

		if (get_unaligned_le64(d) != va[i] || length != lengths[i] ||
		    get_unaligned_le32(d + 48) != i + 1 || get_unaligned_le32(d + 52) ||
		    offset != aligned || offset > size || length > size - offset ||
		    !gpu_m4_store_zero(data + cursor, offset - cursor))
			return -EINVAL;
		sha256(data + offset, length, digest);
		if (memcmp(digest, d + 16, 32))
			return -EBADMSG;
		checked.resource[i] = (struct gpu_m4_store_resource) {
			.gpu_va = va[i], .data = data + offset, .size = length,
		};
		cursor = offset + length;
	}
	if (cursor != size ||
	    memcmp(checked.resource[GPU_M4_WB_USC_WINDOW].data,
		checked.resource[GPU_M4_WB_CONTAINER].data, 0x340) ||
	    !gpu_m4_store_zero(checked.resource[GPU_M4_WB_USC_WINDOW].data + 0x340,
		GPU_M4_DRM_PAGE_SIZE - 0x340) ||
	    get_unaligned_le32(checked.resource[GPU_M4_WB_CDM].data + 40) != 0x60000160 ||
	    get_unaligned_le32(checked.resource[GPU_M4_WB_CDM].data + 44) != 0x40000000 ||
	    !gpu_m4_wb_canary(checked.resource[GPU_M4_WB_INPUT].data, challenge[0].input, 0xa3) ||
	    !gpu_m4_wb_canary(checked.resource[GPU_M4_WB_OUTPUT].data, challenge[0].sentinel, 0x5c))
		return -EINVAL;
	sha256(data, size, digest);
	if (memcmp(digest, m4_user_probe_bundle_sha256, 32))
		return -EBADMSG;
	sha256(checked.resource[GPU_M4_WB_CONTAINER].data + 0x3c0, 70, digest);
	if (memcmp(digest, m4_user_probe_compiler_sha256, 32))
		return -EBADMSG;
	checked.usc_base = GPU_M4_STORE_USC_BASE;
	checked.shader_entry = GPU_M4_STORE_ENTRY_VA;
	checked.input_va = M4_USER_PROBE_INPUT;
	checked.output_va = M4_USER_PROBE_OUTPUT;
	checked.cdm_end = M4_USER_PROBE_CDM + 48;
	checked.main_size = 70;
	checked.nr_gprs = 8;
	*result = checked;
	return 0;
}

static int m4_user_probe_lookup(void *cookie, u32 handle, u32 offset,
	struct gpu_m4_drm_timestamp *timestamp)
{
	struct m4_user_probe *probe = cookie;
	struct gpu_m4_drm_provider_timestamp_config config;
	struct gpu_m4_drm_timestamp checked;
	int ret;

	if (!probe || !timestamp || handle != M4_USER_PROBE_COOKIE ||
	    (offset != 8 && offset != 16) || !probe->result.timestamp_backend)
		return -EINVAL;
	config = m4_provider_ts_timestamp_config(probe->runtime);
	if (!gpu_m4_drm_provider_timestamp_matches(probe->result.timestamp_backend, &config) ||
	    gpu_m4_drm_provider_timestamp_bound_offset(probe->result.timestamp_backend) ||
	    gpu_m4_drm_provider_timestamp_bound_range(probe->result.timestamp_backend) != GPU_M4_DRM_PAGE_SIZE ||
	    gpu_m4_drm_provider_timestamp_policy(probe->result.timestamp_backend) != GPU_M4_DRM_CPU_WRITEBACK)
		return -EPERM;
	ret = gpu_m4_drm_provider_timestamp_slice(probe->result.timestamp_backend, offset, &checked);
	if (ret)
		return ret > 0 ? -EPROTO : ret;
	if (checked.backend != probe->result.timestamp_backend ||
	    checked.object != probe->bo[M4_USER_PROBE_TS].gem ||
	    checked.offset != offset || checked.object_offset != offset)
		return -ESTALE;
	*timestamp = checked;
	return 0;
}

/* Called from the existing initialized enqueue entrance, under its actual GPU
 * mutex, after enqueue receives a real owned job ref. Acceptance is too early.
 * No alternate enqueue/lane transition occurs, and this hook never drops refs.
 */
static void m4_user_probe_enqueue_locked(struct m4_gpu_provider_runtime *p,
	struct gpu_m4_drm_provider_job *job)
{
	struct m4_user_probe *probe = p->user_probe;
	struct gpu_m4_drm_provider_job_view view = { };
	int ret;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!probe || probe->complete || probe->error)
		return;
	gpu_m4_drm_provider_job_view(job, &view);
	if (view.runtime_queue != probe->result.queue)
		return;
	ret = gpu_m4_drm_provider_job_check_owner_locked(job, p->provider);
	if (!ret && (probe->job || !view.version || !view.snapshot || !view.runtime_job ||
	    !view.batch || view.batch->count != 1 || !view.batch->commands ||
	    view.batch->commands[0].type != DRM_ASAHI_CMD_COMPUTE ||
	    !view.command_bytes || view.command_size != sizeof(probe->command) ||
	    memcmp(view.command_bytes, &probe->command, sizeof(probe->command))))
		ret = -ESTALE;
	if (!ret)
		ret = gpu_m4_drm_provider_job_get_locked(job);
	if (ret) {
		probe->error = ret > 0 ? -EPROTO : ret;
		m4_provider_fault_locked(p, probe->error);
		return;
	}
	probe->job = job;
	probe->version = view.version;
}

/* Every obtained GEM/map/object is installed before the next fallible call.
 * WB views are initialized only while privately owned; after submit they are
 * inaccessible until real finished/cache/timestamp retirement checks pass.
 */
static int m4_user_probe_bo_stage(struct m4_user_probe *probe, u32 id)
{
	struct m4_user_probe_bo *b = &probe->bo[id];
	struct m4_gpu_provider_runtime *p = probe->runtime;
	struct drm_gem_object *gem;
	struct gpu_m4_drm_provider_object *object;
	int ret;

	b->policy = (id == GPU_M4_WB_INPUT || id == GPU_M4_WB_OUTPUT || id == M4_USER_PROBE_TS) ?
		GPU_M4_DRM_CPU_WRITEBACK : GPU_M4_DRM_CPU_NC;
	b->va = id < GPU_M4_WB_RESOURCE_COUNT ? probe->recipe.resource[id].gpu_va : 0;
	b->access = DRM_ASAHI_BIND_READ | (id == GPU_M4_WB_OUTPUT ? DRM_ASAHI_BIND_WRITE : 0);
	gem = gpu_m4_drm_bo_create_native(probe->frontend, GPU_M4_DRM_PAGE_SIZE, b->policy,
		id == M4_USER_PROBE_TS ? NULL : probe->frontend_vm);
	if (IS_ERR(gem))
		return PTR_ERR(gem);
	b->gem = gem;
	if (!gpu_m4_drm_bo_is_native(gem) || gem->size != GPU_M4_DRM_PAGE_SIZE ||
	    gpu_m4_drm_bo_policy(gem) != b->policy ||
	    !gpu_m4_drm_bo_belongs_to_vm(gem, probe->frontend_vm))
		return -EPERM;
	if (b->policy == GPU_M4_DRM_CPU_NC) {
		ret = gpu_m4_drm_bo_prepare(gem);
		if (ret)
			return ret;
	}
	ret = drm_gem_vmap(gem, &b->map);
	if (ret)
		return ret;
	b->mapped = true;
	if (iosys_map_is_null(&b->map) || b->map.is_iomem)
		return -EPERM;
	memset(b->map.vaddr, 0, GPU_M4_DRM_PAGE_SIZE);
	if (id < GPU_M4_WB_RESOURCE_COUNT)
		memcpy(b->map.vaddr, probe->recipe.resource[id].data, probe->recipe.resource[id].size);
	else {
		put_unaligned_le64(0xdeadbeefcafef00dULL, b->map.vaddr + 8);
		put_unaligned_le64(0x0badc0de5a5a5a5aULL, b->map.vaddr + 16);
	}
	memcpy(probe->preimage[id], b->map.vaddr, GPU_M4_DRM_PAGE_SIZE);
	b->initialized = true;
	object = gpu_m4_drm_provider_object_pin(probe->vm->provider_vm, gem);
	if (IS_ERR(object))
		return PTR_ERR(object);
	b->object = object;
	mutex_lock(&p->hello->gpu->lock);
	ret = gpu_m4_drm_provider_object_page_locked(object, 0, &b->pa);
	mutex_unlock(&p->hello->gpu->lock);
	return ret > 0 ? -EPROTO : ret;
}

static int m4_user_probe_identity_locked(struct m4_gpu_provider_runtime *p,
	struct m4_user_probe *probe)
{
	struct m4_gpu_hello *h = p->hello;

	lockdep_assert_held(&h->gpu->lock);
	if (!probe || probe->runtime != p || probe->hello != h ||
	    probe->frontend != p->frontend || !p->ordinary || !p->complete ||
	    !p->ready || h->ready != p->ready || h->gpu->hello != h ||
	    p->ready->provider_runtime != p || !refcount_read(&p->session_refs))
		return -ESTALE;
	return p->error ?: h->error ?: h->gpu->error ?: p->ready->error;
}

static int m4_user_probe_fail_locked(struct m4_user_probe *probe, int error)
{
	struct m4_gpu_provider_runtime *p = probe->runtime;

	lockdep_assert_held(&p->hello->gpu->lock);
	if (!probe->error)
		probe->error = error < 0 ? error : -EPROTO;
	m4_provider_fault_locked(p, probe->error);
	if (probe->frontend_vm)
		gpu_m4_drm_vm_error(probe->frontend_vm, probe->error);
	return probe->error;
}

/* Off frontend/GPU/resv locks. One attempt only. Ordinary native VM_BIND and
 * the tiny frontend serialization bridge invoke the production callbacks.
 * No waits, cache maintenance, UAT mutation or release occurs in this caller.
 */
static int m4_user_probe_start(struct m4_gpu_provider_runtime *p)
{
	struct gpu_m4_drm_bind binds[GPU_M4_WB_RESOURCE_COUNT] = { };
	struct gpu_m4_drm_submit submit = { };
	struct m4_user_probe *probe;
	struct gpu_m4_drm_vm *frontend_vm;
	struct mutex *lock = &p->hello->gpu->lock;
	int ret;

	lockdep_assert_not_held(lock);
	probe = kvzalloc(sizeof(*probe), GFP_KERNEL);
	if (!probe)
		return -ENOMEM;
	mutex_lock(lock);
	ret = p->error ?: (!p->ordinary || !p->complete || !p->frontend ||
		!p->user_runtime || p->user_probe ? -EPERM : 0);
	if (!ret)
		ret = m4_user_health_locked(p);
	if (!ret) {
		probe->runtime = p;
		probe->hello = p->hello;
		probe->frontend = p->frontend;
		p->user_probe = probe;
	}
	mutex_unlock(lock);
	if (ret) {
		kvfree(probe); /* Never accepted or externally reachable. */
		return ret > 0 ? -EPROTO : ret;
	}
	ret = request_firmware(&probe->firmware, "apple/m4-j713-ordinary-wb.bin", p->hello->gpu->dev);
	if (!ret)
		ret = m4_user_probe_parse(probe->firmware->data, probe->firmware->size, &probe->recipe);
	if (ret)
		goto fail;
	frontend_vm = gpu_m4_drm_vm_create_private(probe->frontend,
		M4_PROVIDER_KERNEL_START, M4_PROVIDER_KERNEL_END);
	if (IS_ERR(frontend_vm)) {
		ret = PTR_ERR(frontend_vm);
		goto fail;
	}
	probe->frontend_vm = frontend_vm;
	probe->vm = gpu_m4_drm_vm_backend(frontend_vm);
	if (!probe->vm || probe->vm->runtime != p || probe->vm->frontend_vm != frontend_vm ||
	    !probe->vm->provider_vm || probe->vm->closed) {
		ret = -ESTALE;
		goto fail;
	}
	for (u32 i = 0; i < M4_USER_PROBE_OBJECTS; i++) {
		ret = m4_user_probe_bo_stage(probe, i);
		if (ret)
			goto fail;
		if (i < GPU_M4_WB_RESOURCE_COUNT) {
			binds[i].object = probe->bo[i].gem;
			binds[i].op.addr = probe->bo[i].va;
			binds[i].op.range = GPU_M4_DRM_PAGE_SIZE;
			binds[i].op.flags = probe->bo[i].access;
		}
	}
	dma_wmb();
	ret = gpu_m4_drm_provider_vm_bind_native(probe->vm->provider_vm, binds, ARRAY_SIZE(binds));
	if (ret)
		goto fail;
	probe->command.header = (struct drm_asahi_cmd_header) {
		.cmd_type = DRM_ASAHI_CMD_COMPUTE, .size = sizeof(probe->command.compute),
		.vdm_barrier = DRM_ASAHI_BARRIER_NONE, .cdm_barrier = DRM_ASAHI_BARRIER_NONE,
	};
	probe->command.compute = (struct drm_asahi_cmd_compute) {
		.cdm_ctrl_stream_base = M4_USER_PROBE_CDM, .cdm_ctrl_stream_end = M4_USER_PROBE_CDM + 48,
		.ts = { .start = { .handle = M4_USER_PROBE_COOKIE, .offset = 8 },
			.end = { .handle = M4_USER_PROBE_COOKIE, .offset = 16 } },
	};
	submit.cmdbuf = &probe->command;
	submit.size = sizeof(probe->command);
	submit.lookup_cookie = probe;
	submit.lookup_timestamp = m4_user_probe_lookup;
	/* No more CPU payload access until finish proves REAL cache retirement. */
	ret = gpu_m4_drm_probe_submit(probe->frontend, probe->frontend_vm,
		probe->bo[M4_USER_PROBE_TS].gem, DRM_ASAHI_PRIORITY_MEDIUM,
		probe->recipe.usc_base, &submit, &probe->result);
	if (!ret) {
		mutex_lock(lock);
		ret = probe->error ?: p->error;
		if (!ret)
			probe->submitted = true; /* Publish complete result to the observer. */
		mutex_unlock(lock);
		if (!ret)
			return 0;
	}
fail:
	mutex_lock(lock);
	ret = m4_user_probe_fail_locked(probe, ret);
	mutex_unlock(lock);
	return ret;
}

/* Nonblocking off-lock observer. An error/timeout fence is NEVER visibility
 * authority. All fallible identity/cache/resv/TS checks precede WB reads.
 * The actual core already retired once; this performs no second invalidation,
 * native release or fence signal, and cannot register the frontend itself.
 */
static int m4_user_probe_finish(struct m4_gpu_provider_runtime *p)
{
	struct mutex *lock = &p->hello->gpu->lock;
	struct gpu_m4_drm_provider_job_view view = { };
	struct gpu_m4_drm_provider_job_timestamp_view ts[2];
	const struct m4_user_compute_native *native;
	struct m4_user_job *j;
	struct m4_user_probe *probe;
	u64 ns[2];
	u32 count;
	int ret;

	lockdep_assert_not_held(lock);
	mutex_lock(lock);
	probe = p->user_probe;
	if (!probe || probe->runtime != p || probe->hello != p->hello ||
	    probe->frontend != p->frontend) {
		ret = -ENOENT;
		goto out;
	}
	ret = m4_user_probe_identity_locked(p, probe);
	if (!ret)
		ret = probe->error;
	if (ret)
		goto out;
	ret = m4_user_health_locked(p);
	if (ret)
		goto fail;
	if (probe->complete) {
		ret = 0; /* Still rechecks exact current session health. */
		goto out;
	}
	if (!probe->submitted || !probe->result.fence || !probe->job) {
		ret = -EAGAIN;
		goto out;
	}
	ret = dma_fence_get_status(probe->result.fence);
	if (!ret) {
		ret = -EAGAIN;
		goto out;
	}
	if (ret < 0)
		goto fail;
	ret = gpu_m4_drm_provider_job_check_owner_locked(probe->job, p->provider);
	if (ret)
		goto fail;
	gpu_m4_drm_provider_job_view(probe->job, &view);
	j = view.runtime_job;
	if (view.version != probe->version || view.runtime_queue != probe->result.queue ||
	    !j || j->runtime != p || j->core_job != probe->job || !j->retired ||
	    !j->native_stage || j->native_stage != view.stage_owner || j->command_count != 1 ||
	    !view.command_bytes || view.command_size != sizeof(probe->command) ||
	    memcmp(view.command_bytes, &probe->command, sizeof(probe->command))) {
		ret = -ESTALE;
		goto fail;
	}
	for (u32 i = GPU_M4_WB_INPUT; i <= GPU_M4_WB_OUTPUT; i++) {
		struct m4_user_probe_bo *b = &probe->bo[i];

		if (!b->mapped || !b->initialized || iosys_map_is_null(&b->map) || b->map.is_iomem ||
		    gpu_m4_drm_bo_policy(b->gem) != GPU_M4_DRM_CPU_WRITEBACK ||
		    gpu_m4_drm_provider_object_identity(b->object) != b->gem) {
			ret = -ESTALE;
			goto fail;
		}
		ret = gpu_m4_drm_provider_job_check_access_locked(probe->job, p->provider,
			b->object, 0, GPU_M4_DRM_PAGE_SIZE, b->access, true, NULL);
		if (ret)
			goto fail;
	}
	ret = gpu_m4_drm_provider_job_timestamp_count_locked(probe->job, &count);
	if (ret)
		goto fail;
	if (count != 2) {
		ret = -ESTALE;
		goto fail;
	}
	if (!probe->bo[M4_USER_PROBE_TS].mapped || !probe->bo[M4_USER_PROBE_TS].initialized ||
	    iosys_map_is_null(&probe->bo[M4_USER_PROBE_TS].map) || probe->bo[M4_USER_PROBE_TS].map.is_iomem ||
	    gpu_m4_drm_bo_policy(probe->bo[M4_USER_PROBE_TS].gem) != GPU_M4_DRM_CPU_WRITEBACK) {
		ret = -ESTALE;
		goto fail;
	}
	for (u32 i = 0; i < 2; i++) {
		ret = gpu_m4_drm_provider_job_timestamp_at_locked(probe->job, i, &ts[i]);
		if (!ret && (ts[i].command_index || ts[i].timestamp_index != i ||
		    ts[i].object != probe->bo[M4_USER_PROBE_TS].object ||
		    ts[i].gem != probe->bo[M4_USER_PROBE_TS].gem || ts[i].object_offset != 8 + i * 8))
			ret = -ESTALE;
		if (!ret)
			ret = gpu_m4_drm_provider_job_timestamp_check_locked(probe->job, ts[i].runtime_alias, true);
		if (ret)
			goto fail;
	}
	native = m4_user_native_compute(j->native_stage, 0);
	if (!native || !native->private[GPU_M4_COMPUTE_PRIVATE_WORK] ||
	    !native->private[GPU_M4_COMPUTE_PRIVATE_WORK]->mapped) {
		ret = -ESTALE;
		goto fail;
	}
	for (u32 i = 0; i < 2; i++) {
		ret = gpu_m4_render_timestamp_ns(m4_store_read64(
			native->private[GPU_M4_COMPUTE_PRIVATE_WORK]->map.vaddr + 0x8c0 + i * 8), &ns[i]);
		if (ret)
			goto fail;
	}
	/* ONLY HERE can WB payload be observed. Prepared cache/resv coverage above
 * proves full-page maintenance, including the canaries beyond logical cells.
 */
	if (memcmp(probe->bo[GPU_M4_WB_INPUT].map.vaddr, probe->preimage[GPU_M4_WB_INPUT], GPU_M4_DRM_PAGE_SIZE) ||
	    get_unaligned_le32(probe->bo[GPU_M4_WB_OUTPUT].map.vaddr) != probe->recipe.challenge[0].expected_output ||
	    memcmp(probe->bo[GPU_M4_WB_OUTPUT].map.vaddr + 4,
		probe->preimage[GPU_M4_WB_OUTPUT] + 4, GPU_M4_DRM_PAGE_SIZE - 4) ||
	    get_unaligned_le64(probe->bo[M4_USER_PROBE_TS].map.vaddr + 8) != ns[0] ||
	    get_unaligned_le64(probe->bo[M4_USER_PROBE_TS].map.vaddr + 16) != ns[1] ||
	    !ns[0] || ns[1] <= ns[0]) {
		ret = -EIO;
		goto fail;
	}
	for (u32 i = 0; i < GPU_M4_DRM_PAGE_SIZE; i++)
		if ((i < 8 || i >= 24) && ((u8 *)probe->bo[M4_USER_PROBE_TS].map.vaddr)[i] !=
		    probe->preimage[M4_USER_PROBE_TS][i]) {
			ret = -EIO;
			goto fail;
		}
	probe->timestamp_ns[0] = ns[0];
	probe->timestamp_ns[1] = ns[1];
	probe->observed_output = probe->recipe.challenge[0].expected_output;
	probe->binding_epoch = j->binding_epoch;
	probe->complete = true;
	ret = 0;
	goto out;
fail:
	ret = m4_user_probe_fail_locked(probe, ret);
out:
	mutex_unlock(lock);
	return ret > 0 ? -EPROTO : ret;
}

#endif
