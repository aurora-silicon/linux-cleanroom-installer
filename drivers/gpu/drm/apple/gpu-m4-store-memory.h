/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_STORE_MEMORY_H
#define APPLE_GPU_M4_STORE_MEMORY_H

#include <linux/firmware.h>
#include "gpu-m4-store-fixture.h"
#include "gpu-m4-evidence.h"

/* One retained offline fixture and one solitary compute owner. No teardown or
 * retry is qualified once firmware may reference any backing or client root.
 */
struct m4_gpu_store {
	const struct firmware *firmware;
	struct gpu_m4_store_fixture fixture;
	struct gpu_m4_exec_retirement observed;
	struct gpu_m4_evidence evidence;
	u8 fault_image[0x20]; /* REMOVE-DEBUG: first qualified pre-stop bank */
	u64 bound_root;
	u32 output;
	u8 binding_generation;
	int error;
	bool prepared, binding_attempted, bound, attempted, published, complete;
	bool fault_sampled;
	/* REMOVE-DEBUG: retain both translations and the exact zero-slot preimage
	 * throughout this one fresh ordinary-alias qualification and all failures.
	 */
	struct gpu_m4_drm_upper_lease *upper_lease;
	struct gpu_m4_drm_upper_table upper_child;
	u64 upper_original, upper_selected;
	u64 upper_parent_before, upper_parent_after, upper_leaf_before, upper_leaf_after;
	u64 upper_started, upper_finished;
	u32 upper_middle, upper_index, upper_mode, upper_refs;
	int upper_handoff_get, upper_handoff_put, upper_tlbi, upper_error;
	bool upper_attempted, upper_published, upper_consumed, upper_new_child;
	u8 upper_child_before[0x4000];
};

static int m4_store_hash(void *cookie, const void *data, size_t size, u8 digest[32])
{
	(void)cookie;
	sha256(data, size, digest);
	return 0;
}

/* Called under the owner mutex before RUN, after detached graph preparation.
 * Only NC views are touched; no new UAT mappings or producer changes occur.
 * Even failure retains the allocated owner/firmware for diagnosis.
 */
static int m4_store_prepare(struct m4_gpu_hello *h)
{
	static const unsigned int ids[GPU_M4_STORE_RESOURCE_COUNT] = {
		M4_EXEC_CONTAINER, M4_EXEC_IOTO, M4_EXEC_CDM, M4_EXEC_OUTPUT,
		M4_EXEC_USC_WINDOW,
	};
	struct m4_gpu_exec_memory *e = h->ready->execution;
	struct m4_gpu_store *s;
	unsigned int i;
	int ret;

	if (!e || !e->prepared || e->store || h->gpu->firmware_start_attempted ||
	    h->ready->published || !e->client.sealed)
		return -EBUSY;
	/* Evidence storage is reserved before RUN; no allocation on failure. */
	s = kvzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	e->store = s;
	ret = request_firmware(&s->firmware, "apple/m4-j713-store.bin", h->gpu->dev);
	if (ret)
		goto failed;
	ret = gpu_m4_store_fixture_parse(s->firmware->data, s->firmware->size,
					 m4_store_hash, NULL, &s->fixture);
	if (ret)
		goto failed;
	/* Validate every target before any copy, including complete page backing. */
	for (i = 0; i < GPU_M4_STORE_RESOURCE_COUNT; i++) {
		unsigned int id = ids[i];
		struct m4_ready_buffer *b = &e->buffer[id];
		const struct gpu_m4_store_resource *resource = &s->fixture.resource[i];

		if (resource->gpu_va != m4_exec_buffer_gpu[id] ||
		    resource->size > b->size ||
		    !m4_exec_backing_valid(h, b, m4_exec_buffer_size[id], false)) {
			ret = -EPROTO;
			goto failed;
		}
		ret = gpu_m4_client_check(&e->client, resource->gpu_va, e->buffer_pa[id],
					  b->size / 0x4000, m4_exec_writable_buffer(id));
		if (ret)
			goto failed;
	}
	for (i = 0; i < GPU_M4_STORE_RESOURCE_COUNT; i++) {
		struct m4_ready_buffer *b = &e->buffer[ids[i]];
		const struct gpu_m4_store_resource *resource = &s->fixture.resource[i];

		memset(b->cpu, 0, b->size);
		memcpy(b->cpu, resource->data, resource->size);
	}
	dma_wmb();
	s->prepared = true;
	dev_info(h->gpu->dev, "GPU first-store fixture retained: compiler36/GPR8 Ioto90 CDM48 terminated USCbase16KiB; no ASID bind or work\n");
	return 0;
failed:
	s->error = ret;
	return ret;
}

/* Only extra TTBAT slots 2..127; bootstrap slots 0/1 stay independently checked.
 * Never admit a published but not fully synchronized/qualified client root.
 */
static bool m4_store_bound_root_valid(struct m4_gpu_hello *h, u32 index, u64 value)
{
	struct m4_gpu_exec_memory *e = h->ready->execution;
	struct m4_gpu_store *s = e ? e->store : NULL;

	if (index < 2 || index >= 128)
		return false;
	if (index != 2 || !s || !s->bound)
		return !value;
	return s->prepared && s->binding_generation == 1 &&
		s->bound_root == (e->table[0].pa | 1 | (1ULL << 48)) &&
		value == s->bound_root;
}

#endif
