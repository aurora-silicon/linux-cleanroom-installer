/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Fixed-mode atomic KMS over the single retained DCP owner. No hot teardown. */
#include <linux/aperture.h>
#include <linux/backlight.h>
#include <linux/dma-buf.h>
#include <linux/iommu.h>
#include <linux/ktime.h>
#include <linux/scatterlist.h>
#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include <drm/drm_fbdev_shmem.h>
#include <drm/drm_file.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_prime.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

/* Why a flip used the CPU copy instead of scanning the client buffer directly. */
enum dcp_m4_copy_reason {
	DCP_M4_DIRECT,
	DCP_M4_COPY_LOCAL,	/* Local shmem GEM: CPU-written through a cached mapping. */
	DCP_M4_COPY_LAYOUT,	/* Outside the direct layout: planes, offset or size. */
	DCP_M4_COPY_MAP,	/* No qualified SID0 mapping for this object and output. */
	DCP_M4_COPY_DISABLED,	/* direct_scanout=0. */
	DCP_M4_COPY_REASONS,
};

/* REMOVE-DEBUG: bring-up A/B switch. Existing mappings stay until GEM release. */
static bool dcp_m4_direct_scanout = true;
module_param_named(direct_scanout, dcp_m4_direct_scanout, bool, 0644);
MODULE_PARM_DESC(direct_scanout, "Scan qualified imported framebuffers without a CPU copy");

static bool dcp_m4_direct_local = true;
module_param_named(direct_local, dcp_m4_direct_local, bool, 0644);
MODULE_PARM_DESC(direct_local, "Also scan local (dumb/GBM) framebuffers directly; they are write-combined");

/* Hardware cursor (internal panel): an upper DCP surface fed from a canvas. */
#define DCP_M4_CURSOR_SIZE	64
static int dcp_m4_cursor_plane = -1;
module_param_named(cursor_plane, dcp_m4_cursor_plane, int, 0444);
MODULE_PARM_DESC(cursor_plane, "Internal panel hardware cursor plane: -1 auto (J514S), 0 off, 1 on");
static uint dcp_m4_cursor_index = 3;
module_param_named(cursor_index, dcp_m4_cursor_index, uint, 0444);
MODULE_PARM_DESC(cursor_index, "Firmware surface-array index of the cursor, 1..3");
static uint dcp_m4_cursor_id = 1;
module_param_named(cursor_id, dcp_m4_cursor_id, uint, 0444);
MODULE_PARM_DESC(cursor_id, "Swap surface identity word of the cursor surface");
static uint dcp_m4_cursor_flags;
module_param_named(cursor_flags, dcp_m4_cursor_flags, uint, 0444);
MODULE_PARM_DESC(cursor_flags, "Swap surf_flags word of the cursor surface");

/* REMOVE-DEBUG: flip path, swap latency (A406 start to D590) and mapping lifetime. */
struct dcp_m4_kms_stats {
	u64 flips, direct, copy[DCP_M4_COPY_REASONS];
	u64 swap_ns_last, swap_ns_max, swap_ns_total;
	atomic64_t maps, map_failures, unmaps, retained, idle_vblanks;
	atomic64_t backlight_swaps; /* Brightness-only re-presents of the active surface. */
	atomic64_t cursor_swaps; /* Swaps carrying only the cursor surface. */
};

/* Panel brightness in nits: the A407 binary64 field accepts integral 0..500. */
#define DCP_M4_NITS_MAX		((int)dcp_m4_pf->brightness_max)
#define DCP_M4_NITS_DEFAULT	((int)dcp_m4_pf->brightness_default)

struct dcp_m4_output;
struct dcp_m4_ext;
struct dcp_m4_ready;

/*
 * An imported client object gets one SID0 mapping at its first qualified
 * prepare_fb and keeps it for the GEM lifetime; prepare_fb never remaps.
 * Framebuffer references keep the object, and so the mapping, alive while
 * DCP may scan it (see dcp_m4_output).
 */
struct dcp_m4_bo {
	struct drm_gem_shmem_object shmem; /* First: the shmem helpers free it. */
	struct dcp_m4_output *output;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;	/* imported: attachment mapping; local: own mapping */
	u64 dva, mapped;
	int map_error; /* Sticky: this object stays on the copy path. */
};

/* prepare_fb records the scanout decision; only the copy path maps for CPU reads. */
struct dcp_m4_plane_state {
	struct drm_shadow_plane_state shadow;
	struct dcp_m4_bo *direct;
	enum dcp_m4_copy_reason copy;
	bool vmapped;
};

/*
 * One DCP display output. Further DCP instances add outputs with their own
 * runtime and display consumer; DRM indexes vblank state by CRTC.
 *
 * Brightness rides inside the swap. The runtime lock (dcp->lock) serialises
 * every swap of this output: atomic flushes, CRTC disables and backlight
 * re-presents. Under it, nits is the latest backlight request (0 while the
 * backlight is blanked) and nits_applied the value the last completed swap
 * carried, or -1 before any. A swap carries brightness only when they differ.
 */
struct dcp_m4_output {
	struct dcp_m4_kms *kms;
	struct dcp_m4_runtime *runtime;
	struct device *scanout; /* Display consumer whose DMA domain is SID0. */
	struct drm_plane plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;
	struct drm_display_mode mode;
	struct dcp_m4_frame frame[2]; /* Retained copy-path surfaces. */
	void *pixels[2];
	int active_slot; /* Copy surface DCP may scan, or -1. */
	struct drm_framebuffer *active_fb; /* Client framebuffer DCP may scan. */
	struct dcp_m4_frame active_frame; /* Its direct-scanout descriptor. */
	struct drm_framebuffer *pinned; /* Uncertain successor: retained until reboot. */
	u64 swap_vblank_ns; /* Host time of the last D590-driven vblank. */
	struct dcp_m4_ext *ext;	/* External DCPEXT owner; NULL for the internal panel. */
	struct backlight_device *backlight; /* Internal panel only; devm on the DCP device. */
	u32 nits;
	int nits_applied;
	bool blanked;
	/* Hardware cursor: two retained 64x64 ARGB canvases, under dcp->lock. */
	struct drm_plane cursor;
	struct dcp_m4_frame cursor_frame[2];
	void *cursor_pixels[2];
	int cursor_slot;	/* Canvas DCP may scan, or -1. */
	u32 cursor_keep;	/* BIT(cursor index) with a cursor plane, else 0. */
	bool has_cursor;
};

struct dcp_m4_kms {
	struct drm_device drm;
	struct dcp_m4_output internal;
	struct dcp_m4_output *ext[2];	/* Optional external outputs (dcpext_kms=1). */
	unsigned int ext_count;
	struct mutex map_lock; /* Serializes lazy client mapping creation. */
	struct dcp_m4_kms_stats stats;
	bool registered;
};

static struct dcp_m4_kms *to_dcp_m4_kms(struct drm_device *drm)
{
	return container_of(drm, struct dcp_m4_kms, drm);
}

static struct dcp_m4_output *to_dcp_m4_output(struct drm_crtc *crtc)
{
	return container_of(crtc, struct dcp_m4_output, crtc);
}

static struct dcp_m4_plane_state *to_dcp_m4_plane_state(struct drm_plane_state *state)
{
	return container_of(to_drm_shadow_plane_state(state), struct dcp_m4_plane_state, shadow);
}

static struct dcp_m4_bo *to_dcp_m4_bo(struct drm_gem_object *obj)
{
	return container_of(to_drm_gem_shmem_obj(obj), struct dcp_m4_bo, shmem);
}

/* Direct DMA would yield physical addresses, never SID0 DVAs. */
static bool dcp_m4_kms_translated(struct device *dev)
{
#ifdef CONFIG_IOMMU_DMA
	struct iommu_domain *domain = dev ? iommu_get_domain_for_dev(dev) : NULL;

	return domain && iommu_is_dma_domain(domain) && dev->dma_iommu;
#else
	return false;
#endif
}

/* Map an imported object through the display consumer's own DMA domain. */
static int dcp_m4_bo_map(struct dcp_m4_bo *bo, struct dcp_m4_output *o)
{
	struct dma_buf *buf = bo->shmem.base.import_attach->dmabuf;
	struct dma_buf_attachment *attach;
	struct iommu_domain *domain;
	struct scatterlist *sg;
	struct sg_table *sgt;
	u64 start = 0, length = 0;
	unsigned int i;
	int ret = 0;

	if (!dcp_m4_kms_translated(o->scanout))
		return -ENODEV;
	domain = iommu_get_domain_for_dev(o->scanout);
	attach = dma_buf_attach(buf, o->scanout);
	if (IS_ERR(attach))
		return PTR_ERR(attach);
	sgt = dma_buf_map_attachment_unlocked(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		dma_buf_detach(buf, attach);
		return PTR_ERR(sgt);
	}
	for_each_sgtable_dma_sg(sgt, sg, i) {
		ret = dcp_m4_extent_add(&start, &length, sg_dma_address(sg), sg_dma_len(sg));
		if (ret)
			break;
	}
	if (!ret)
		ret = dcp_m4_direct_extent(start, length, bo->shmem.base.size);
	/* Read-only: every page must already translate in the qualified domain. */
	for (u64 off = 0; !ret && off < length; off += SZ_16K)
		if (!iommu_iova_to_phys(domain, start + off))
			ret = -EIO;
	if (ret) {
		/* DCP never received this DVA: the unpublished mapping can be returned. */
		dma_buf_unmap_attachment_unlocked(attach, sgt, DMA_BIDIRECTIONAL);
		dma_buf_detach(buf, attach);
		return ret;
	}
	bo->attach = attach;
	bo->sgt = sgt;
	bo->dva = start;
	bo->mapped = length;
	bo->output = o;
	return 0;
}

/*
 * Map a local (dumb/GBM) object for direct scanout: pin its pages and map them
 * through the scanout consumer's own DMA domain. Local objects are created
 * write-combined, so CPU writers leave no dirty lines; the non-coherent DMA
 * map cleans what allocation-time zeroing left in the cache. Typical producer:
 * a compositor's GPU renders into the buffer through a PRIME import.
 */
static int dcp_m4_bo_map_local(struct dcp_m4_bo *bo, struct dcp_m4_output *o)
{
	struct drm_gem_shmem_object *shmem = &bo->shmem;
	struct iommu_domain *domain;
	struct scatterlist *sg;
	struct sg_table *sgt;
	u64 start = 0, length = 0;
	unsigned int i;
	int ret;

	if (!shmem->map_wc || !dcp_m4_kms_translated(o->scanout))
		return -ENODEV;
	ret = drm_gem_shmem_pin(shmem);
	if (ret)
		return ret;
	sgt = drm_prime_pages_to_sg(shmem->base.dev, shmem->pages,
				    shmem->base.size >> PAGE_SHIFT);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		goto unpin;
	}
	ret = dma_map_sgtable(o->scanout, sgt, DMA_BIDIRECTIONAL, 0);
	if (ret)
		goto free;
	domain = iommu_get_domain_for_dev(o->scanout);
	for_each_sgtable_dma_sg(sgt, sg, i) {
		ret = dcp_m4_extent_add(&start, &length, sg_dma_address(sg), sg_dma_len(sg));
		if (ret)
			break;
	}
	if (!ret)
		ret = dcp_m4_direct_extent(start, length, shmem->base.size);
	for (u64 off = 0; !ret && off < length; off += SZ_16K)
		if (!iommu_iova_to_phys(domain, start + off))
			ret = -EIO;
	if (ret) {
		/* DCP never received this DVA. */
		dma_unmap_sgtable(o->scanout, sgt, DMA_BIDIRECTIONAL, 0);
		goto free;
	}
	bo->sgt = sgt;
	bo->dva = start;
	bo->mapped = length;
	bo->output = o;
	return 0;
free:
	sg_free_table(sgt);
	kfree(sgt);
unpin:
	drm_gem_shmem_unpin(shmem);
	return ret;
}

/* Choose direct scanout or the CPU copy for one framebuffer on one output. */
static enum dcp_m4_copy_reason dcp_m4_kms_direct(struct dcp_m4_output *o,
		struct drm_framebuffer *fb, struct dcp_m4_bo **direct)
{
	struct dcp_m4_kms *k = o->kms;
	struct drm_gem_object *obj = drm_gem_fb_get_obj(fb, 0);
	u64 needed = (u64)fb->pitches[0] * fb->height;
	struct dcp_m4_bo *bo;
	int ret;

	bool local;

	if (!obj)
		return DCP_M4_COPY_LOCAL;
	local = !drm_gem_is_imported(obj);
	if (local && !READ_ONCE(dcp_m4_direct_local))
		return DCP_M4_COPY_LOCAL;
	if (fb->format->num_planes != 1 || fb->offsets[0] || needed > obj->size ||
	    needed > U32_MAX)
		return DCP_M4_COPY_LAYOUT;
	if (!READ_ONCE(dcp_m4_direct_scanout))
		return DCP_M4_COPY_DISABLED;
	bo = to_dcp_m4_bo(obj);
	mutex_lock(&k->map_lock);
	if (!bo->output && !bo->map_error) {
		bo->map_error = READ_ONCE(o->runtime->dcp->error);
		if (!bo->map_error)
			bo->map_error = local ? dcp_m4_bo_map_local(bo, o) : dcp_m4_bo_map(bo, o);
		atomic64_inc(bo->map_error ? &k->stats.map_failures : &k->stats.maps);
	}
	ret = bo->map_error ? bo->map_error : bo->output != o ? -EXDEV : 0;
	mutex_unlock(&k->map_lock);
	if (ret)
		return DCP_M4_COPY_MAP;
	*direct = bo;
	return DCP_M4_DIRECT;
}

static void dcp_m4_bo_free(struct drm_gem_object *obj)
{
	struct dcp_m4_bo *bo = to_dcp_m4_bo(obj);
	struct dcp_m4_kms *k = to_dcp_m4_kms(obj->dev);

	/* The last framebuffer reference is gone: DCP has a completed successor. */
	if (bo->sgt && READ_ONCE(bo->output->runtime->dcp->error)) {
		/* Uncertain firmware state: keep the DVA, pages and dma-buf until reboot. */
		atomic64_inc(&k->stats.retained);
		if (!bo->attach)
			return;	/* local: leak the object with its pinned pages */
		get_dma_buf(bo->attach->dmabuf);
	} else if (bo->sgt && !bo->attach) {
		dma_unmap_sgtable(bo->output->scanout, bo->sgt, DMA_BIDIRECTIONAL, 0);
		sg_free_table(bo->sgt);
		kfree(bo->sgt);
		drm_gem_shmem_unpin(&bo->shmem);
		atomic64_inc(&k->stats.unmaps);
	} else if (bo->sgt) {
		dma_buf_unmap_attachment_unlocked(bo->attach, bo->sgt, DMA_BIDIRECTIONAL);
		dma_buf_detach(bo->attach->dmabuf, bo->attach);
		atomic64_inc(&k->stats.unmaps);
	}
	drm_gem_shmem_object_free(obj);
}

static const struct drm_gem_object_funcs dcp_m4_bo_funcs = {
	.free = dcp_m4_bo_free,
	.print_info = drm_gem_shmem_object_print_info,
	.pin = drm_gem_shmem_object_pin,
	.unpin = drm_gem_shmem_object_unpin,
	.get_sg_table = drm_gem_shmem_object_get_sg_table,
	.vmap = drm_gem_shmem_object_vmap,
	.vunmap = drm_gem_shmem_object_vunmap,
	.mmap = drm_gem_shmem_object_mmap,
	.vm_ops = &drm_gem_shmem_vm_ops,
};

static struct drm_gem_object *dcp_m4_gem_create_object(struct drm_device *drm, size_t size)
{
	struct dcp_m4_bo *bo = kzalloc(sizeof(*bo), GFP_KERNEL);

	if (!bo)
		return ERR_PTR(-ENOMEM);
	bo->shmem.base.funcs = &dcp_m4_bo_funcs;
	/* Write-combined CPU access: eligible for direct scanout (no dirty lines). */
	bo->shmem.map_wc = true;
	return &bo->shmem.base;
}

static int dcp_m4_kms_plane_check(struct drm_plane *plane, struct drm_atomic_state *state)
{
	struct drm_plane_state *p = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *c;
	int ret;

	if (!p->crtc)
		return 0;
	c = drm_atomic_get_new_crtc_state(state, p->crtc);
	ret = drm_atomic_helper_check_plane_state(p, c, DRM_PLANE_NO_SCALING,
						DRM_PLANE_NO_SCALING, false, true);
	if (ret || !p->fb)
		return ret;
	if (p->src_x || p->src_y || p->crtc_x || p->crtc_y ||
	    p->src_w != DCP_M4_PANEL_WIDTH << 16 || p->src_h != DCP_M4_PANEL_HEIGHT << 16 ||
	    p->crtc_w != DCP_M4_PANEL_WIDTH || p->crtc_h != DCP_M4_PANEL_HEIGHT ||
	    p->fb->width != DCP_M4_PANEL_WIDTH || p->fb->height != DCP_M4_PANEL_HEIGHT ||
	    p->fb->offsets[0] || p->fb->modifier != DRM_FORMAT_MOD_LINEAR ||
	    p->fb->pitches[0] < DCP_M4_PANEL_WIDTH * 4 || p->fb->pitches[0] % 64 ||
	    (p->fb->format->format != DRM_FORMAT_XRGB8888 &&
	     p->fb->format->format != DRM_FORMAT_ARGB8888))
		return -EINVAL;
	return 0;
}

/* Standard producer fence wait, then the per-commit direct/copy decision. */
static int dcp_m4_kms_prepare_fb(struct drm_plane *plane, struct drm_plane_state *state)
{
	struct dcp_m4_output *o = container_of(plane, struct dcp_m4_output, plane);
	struct dcp_m4_plane_state *s = to_dcp_m4_plane_state(state);
	int ret = drm_gem_plane_helper_prepare_fb(plane, state);

	s->direct = NULL;
	s->copy = DCP_M4_COPY_LOCAL;
	if (!ret && state->fb)
		s->copy = dcp_m4_kms_direct(o, state->fb, &s->direct);
	return ret;
}

/* Direct scanout never reads client pixels: no vmap and no CPU cache access. */
static int dcp_m4_kms_begin_fb_access(struct drm_plane *plane, struct drm_plane_state *state)
{
	struct dcp_m4_plane_state *s = to_dcp_m4_plane_state(state);
	int ret;

	if (!state->fb || s->direct)
		return 0;
	ret = drm_gem_begin_shadow_fb_access(plane, state);
	s->vmapped = !ret;
	return ret;
}

static void dcp_m4_kms_end_fb_access(struct drm_plane *plane, struct drm_plane_state *state)
{
	struct dcp_m4_plane_state *s = to_dcp_m4_plane_state(state);

	if (s->vmapped)
		drm_gem_end_shadow_fb_access(plane, state);
	s->vmapped = false;
}

static void dcp_m4_kms_plane_update(struct drm_plane *plane, struct drm_atomic_state *state)
{
	/* Required by commit_planes(); the CRTC flush presents the complete frame. */
}

static void dcp_m4_kms_plane_destroy_state(struct drm_plane *plane,
					   struct drm_plane_state *state)
{
	struct dcp_m4_plane_state *s = to_dcp_m4_plane_state(state);

	__drm_gem_destroy_shadow_plane_state(&s->shadow);
	kfree(s);
}

static void dcp_m4_kms_plane_reset(struct drm_plane *plane)
{
	struct dcp_m4_plane_state *s;

	if (plane->state) {
		dcp_m4_kms_plane_destroy_state(plane, plane->state);
		plane->state = NULL;
	}
	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (s)
		__drm_gem_reset_shadow_plane(plane, &s->shadow);
}

/* The scanout decision and CPU mapping are per commit and never duplicated. */
static struct drm_plane_state *dcp_m4_kms_plane_duplicate(struct drm_plane *plane)
{
	struct dcp_m4_plane_state *s;

	if (WARN_ON(!plane->state))
		return NULL;
	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return NULL;
	__drm_gem_duplicate_shadow_plane_state(plane, &s->shadow);
	return &s->shadow.base;
}

static const struct drm_plane_helper_funcs dcp_m4_kms_plane_helpers = {
	.prepare_fb = dcp_m4_kms_prepare_fb,
	.begin_fb_access = dcp_m4_kms_begin_fb_access,
	.end_fb_access = dcp_m4_kms_end_fb_access,
	.atomic_check = dcp_m4_kms_plane_check,
	.atomic_update = dcp_m4_kms_plane_update,
};
static int dcp_m4_kms_cursor_prepare_fb(struct drm_plane *plane, struct drm_plane_state *state)
{
	struct dcp_m4_plane_state *s = to_dcp_m4_plane_state(state);

	/* Always blitted into a canvas: producer fences, then a CPU mapping. */
	s->direct = NULL;
	s->copy = DCP_M4_COPY_LOCAL;
	return drm_gem_plane_helper_prepare_fb(plane, state);
}

static int dcp_m4_kms_cursor_check(struct drm_plane *plane, struct drm_atomic_state *state)
{
	struct drm_plane_state *p = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *c;
	int ret;

	if (!p->crtc || !p->fb)
		return 0;
	c = drm_atomic_get_new_crtc_state(state, p->crtc);
	ret = drm_atomic_helper_check_plane_state(p, c, DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING, true, true);
	if (ret) {
		/* REMOVE-DEBUG */
		dev_info_ratelimited(plane->dev->dev, "cursor plane check %d: src %ux%u crtc %ux%u\n",
				     ret, p->src_w >> 16, p->src_h >> 16, p->crtc_w, p->crtc_h);
		return ret;
	}
	/* Whole-pixel source inside the framebuffer, at most one canvas. */
	if (((p->src_x | p->src_y | p->src_w | p->src_h) & 0xffff) ||
	    (p->src_x >> 16) + (p->src_w >> 16) > p->fb->width ||
	    (p->src_y >> 16) + (p->src_h >> 16) > p->fb->height ||
	    (p->src_w >> 16) > DCP_M4_CURSOR_SIZE || (p->src_h >> 16) > DCP_M4_CURSOR_SIZE ||
	    p->fb->offsets[0] || p->fb->modifier != DRM_FORMAT_MOD_LINEAR ||
	    p->fb->format->format != DRM_FORMAT_ARGB8888) {
		/* REMOVE-DEBUG: bring-up visibility of compositor cursor requests. */
		dev_info_ratelimited(plane->dev->dev,
			    "cursor rejected: fb %ux%u fmt %p4cc mod %#llx src %u,%u %ux%u\n",
			    p->fb->width, p->fb->height, &p->fb->format->format,
			    p->fb->modifier, p->src_x >> 16, p->src_y >> 16,
			    p->src_w >> 16, p->src_h >> 16);
		return -EINVAL;
	}
	return 0;
}

static const struct drm_plane_helper_funcs dcp_m4_kms_cursor_helpers = {
	.prepare_fb = dcp_m4_kms_cursor_prepare_fb,
	.begin_fb_access = dcp_m4_kms_begin_fb_access,
	.end_fb_access = dcp_m4_kms_end_fb_access,
	.atomic_check = dcp_m4_kms_cursor_check,
	.atomic_update = dcp_m4_kms_plane_update,
};

static const struct drm_plane_funcs dcp_m4_kms_plane_funcs = {
	.reset = dcp_m4_kms_plane_reset,
	.atomic_duplicate_state = dcp_m4_kms_plane_duplicate,
	.atomic_destroy_state = dcp_m4_kms_plane_destroy_state,
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
};

static int dcp_m4_kms_crtc_check(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	struct dcp_m4_output *o = to_dcp_m4_output(crtc);
	struct drm_crtc_state *c = drm_atomic_get_new_crtc_state(state, crtc);

	if (READ_ONCE(o->runtime->dcp->error))
		return -EIO;
	if (c->enable && (!drm_mode_equal(&c->mode, &o->mode) ||
	    !(c->plane_mask & drm_plane_mask(&o->plane))))
		return -EINVAL;
	/* Vblank is D590-driven software accounting; DRM keeps the default no_vblank. */
	return 0;
}

/* On error, wake internal commit waiters and fail fences without a success event. */
static void dcp_m4_kms_cancel_event(struct drm_crtc_state *c, int error)
{
	struct drm_pending_vblank_event *e = c->event;

	c->event = NULL;
	if (e) {
		if (e->base.fence) {
			dma_fence_set_error(e->base.fence, error);
			dma_fence_signal(e->base.fence);
		}
		if (e->base.completion) {
			complete_all(e->base.completion);
			e->base.completion_release(e->base.completion);
			e->base.completion = NULL;
		}
		drm_event_cancel_free(c->crtc->dev, &e->base);
	}
}

/*
 * Consume the CRTC event. A completed swap (A407 reply and matching D590) is
 * the vblank: without a hardware counter or timestamp hook, DRM stamps it with
 * host ktime_get(). Commits without new pixels report the last vblank.
 */
static void dcp_m4_kms_finish(struct drm_crtc *crtc, int error, bool swapped)
{
	struct drm_crtc_state *c = crtc->state;
	unsigned long flags;
	bool held;

	if (error) {
		/* The latch keeps the precise cause; the contract reports -EIO. */
		dcp_m4_kms_cancel_event(c, -EIO);
		return;
	}
	held = swapped && !drm_crtc_vblank_get(crtc);
	if (held) {
		WRITE_ONCE(to_dcp_m4_output(crtc)->swap_vblank_ns, ktime_get_ns());
		drm_crtc_handle_vblank(crtc);
	}
	if (c->event) {
		spin_lock_irqsave(&crtc->dev->event_lock, flags);
		drm_crtc_send_vblank_event(crtc, c->event);
		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
		c->event = NULL;
	}
	if (held)
		drm_crtc_vblank_put(crtc);
}

/* REMOVE-DEBUG: one completed flip, under the DCP lock. */
static void dcp_m4_kms_account(struct dcp_m4_kms *k, enum dcp_m4_copy_reason copy, u64 ns)
{
	struct dcp_m4_kms_stats *s = &k->stats;

	WRITE_ONCE(s->swap_ns_last, ns);
	WRITE_ONCE(s->swap_ns_total, s->swap_ns_total + ns);
	if (ns > s->swap_ns_max)
		WRITE_ONCE(s->swap_ns_max, ns);
	WRITE_ONCE(s->flips, s->flips + 1);
	if (copy == DCP_M4_DIRECT)
		WRITE_ONCE(s->direct, s->direct + 1);
	else if (copy < DCP_M4_COPY_REASONS)
		WRITE_ONCE(s->copy[copy], s->copy[copy] + 1);
}

/* Under dcp->lock: one swap, carrying brightness only when it changes the panel. */
static int dcp_m4_output_swap_full(struct dcp_m4_output *o, bool primary,
				   const struct dcp_m4_frame *frame,
				   const struct dcp_m4_upper *upper, u32 nits)
{
	bool change = o->nits_applied != (int)nits;
	int ret = dcp_m4_runtime_swap_full(o->runtime, primary, frame, upper, o->cursor_keep,
					   change, nits);

	/* The panel holds the new value only once A407 and its D590 completed. */
	if (!ret && change)
		o->nits_applied = nits;
	return ret;
}

static int dcp_m4_output_swap(struct dcp_m4_output *o, const struct dcp_m4_frame *frame, u32 nits)
{
	return dcp_m4_output_swap_full(o, true, frame, NULL, nits);
}

/*
 * Present the cursor through a fully on-panel 64x64 canvas: clamp the canvas
 * into the panel and blit the visible part of the cursor at its offset, the
 * rest transparent, so firmware never sees a clipped or sub-32x32 rectangle.
 * *slot is the canvas presented, or -1 to hide.
 */
static int dcp_m4_cursor_build(struct dcp_m4_output *o, struct drm_plane_state *cs,
			       struct dcp_m4_upper *u, int *slot)
{
	struct dcp_m4_plane_state *s = to_dcp_m4_plane_state(cs);
	struct drm_shadow_plane_state *shadow = to_drm_shadow_plane_state(cs);
	const s32 n = DCP_M4_CURSOR_SIZE;
	int next = o->cursor_slot == 0 ? 1 : 0;
	s32 x, y, ox, oy, w, h, pitch;
	const u8 *src;
	u8 *dst;
	int ret;

	*u = (struct dcp_m4_upper) {
		.index = dcp_m4_cursor_index, .id = dcp_m4_cursor_id,
		.flags = dcp_m4_cursor_flags,
	};
	*slot = -1;
	if (!cs->fb || !cs->visible)
		return 0;
	if (!s->vmapped || shadow->data[0].is_iomem)
		return -EINVAL;
	/* The source sub-rectangle, unscaled (atomic check). */
	w = cs->src_w >> 16;
	h = cs->src_h >> 16;
	pitch = cs->fb->pitches[0];
	x = clamp_t(s32, cs->crtc_x, 0, DCP_M4_PANEL_WIDTH - n);
	y = clamp_t(s32, cs->crtc_y, 0, DCP_M4_PANEL_HEIGHT - n);
	ox = cs->crtc_x - x;	/* Cursor origin inside the canvas. */
	oy = cs->crtc_y - y;
	ret = drm_gem_fb_begin_cpu_access(cs->fb, DMA_FROM_DEVICE);
	if (ret)
		return ret;
	src = (const u8 *)shadow->data[0].vaddr + (cs->src_y >> 16) * pitch + (cs->src_x >> 16) * 4;
	dst = o->cursor_pixels[next];
	for (s32 v = 0; v < n; v++) {
		s32 sy = v - oy, sx0 = max(0, -ox), sx1 = min(w, n - ox);
		u8 *row = dst + v * n * 4;

		memset(row, 0, n * 4);
		if (sy >= 0 && sy < h && sx0 < sx1)
			memcpy(row + (sx0 + ox) * 4, src + sy * pitch + sx0 * 4, (sx1 - sx0) * 4);
	}
	drm_gem_fb_end_cpu_access(cs->fb, DMA_FROM_DEVICE);
	dma_wmb();
	u->frame = &o->cursor_frame[next];
	u->x = x;
	u->y = y;
	*slot = next;
	return 0;
}

static void dcp_m4_kms_flush(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	struct dcp_m4_output *o = to_dcp_m4_output(crtc);
	struct dcp_m4_runtime *r = o->runtime;
	struct drm_plane_state *p = drm_atomic_get_new_plane_state(state, &o->plane);
	struct drm_plane_state *cs = o->has_cursor ?
		drm_atomic_get_new_plane_state(state, &o->cursor) : NULL;
	struct dcp_m4_upper upper;
	int cslot = -1;
	struct drm_framebuffer *held = NULL, *retire = NULL;
	struct drm_shadow_plane_state *shadow;
	struct dcp_m4_plane_state *s;
	struct dcp_m4_frame direct, *frame = NULL;
	struct iosys_map dst;
	struct drm_rect clip = DRM_RECT_INIT(0, 0, DCP_M4_PANEL_WIDTH, DCP_M4_PANEL_HEIGHT);
	int next = o->active_slot == 0 ? 1 : 0;
	u64 start;
	int ret;

	if (!crtc->state->active)
		return;
	/* Modeset checks add affected planes; a CRTC-only no-op has no new pixels. */
	if (!p && !cs) {
		dcp_m4_kms_finish(crtc, READ_ONCE(r->dcp->error), false);
		return;
	}
	mutex_lock(&r->dcp->lock);
	ret = dcp_m4_runtime_live(r);
	if (ret)
		goto failed;
	if (cs) {
		ret = dcp_m4_cursor_build(o, cs, &upper, &cslot);
		if (ret)
			goto failed;
	}
	if (!p) {
		/* Cursor only: the primary surface stays as it is. */
		ret = dcp_m4_output_swap_full(o, false, NULL, &upper, o->nits);
		if (ret)
			goto failed;
		o->cursor_slot = cslot;
		atomic64_inc(&o->kms->stats.cursor_swaps);
		dcp_m4_kms_finish(crtc, 0, true);
		mutex_unlock(&r->dcp->lock);
		return;
	}
	if (!p->fb) {
		ret = -EINVAL;
		goto failed;
	}
	s = to_dcp_m4_plane_state(p);
	if (s->direct) {
		/* Atomic helpers waited for producer fences; DCP reads the pixels. */
		ret = dcp_m4_direct_frame(&direct, s->direct->dva, s->direct->mapped,
					  p->fb->width, p->fb->height, p->fb->pitches[0],
					  p->fb->format->format == DRM_FORMAT_ARGB8888);
		if (ret)
			goto failed;
		/* Hold the client framebuffer before A406/A407 can publish its DVA. */
		held = p->fb;
		drm_framebuffer_get(held);
		frame = &direct;
	} else {
		if (!s->vmapped) {
			ret = -EINVAL;
			goto failed;
		}
		shadow = to_drm_shadow_plane_state(p);
		iosys_map_set_vaddr(&dst, o->pixels[next]);
		/* Atomic helpers waited for producer fences and mapped the source. */
		ret = drm_gem_fb_begin_cpu_access(p->fb, DMA_FROM_DEVICE);
		if (ret)
			goto failed;
		drm_fb_memcpy(&dst, &o->frame[next].stride, shadow->data, p->fb, &clip);
		drm_gem_fb_end_cpu_access(p->fb, DMA_FROM_DEVICE);
		o->frame[next].format = p->fb->format->format == DRM_FORMAT_ARGB8888 ?
			DCP_M4_ARGB8888 : DCP_M4_XRGB8888;
		dma_wmb();
		frame = &o->frame[next];
	}
	start = ktime_get_ns();
	/* The first swap, and the first after a blank, apply the stored request. */
	ret = dcp_m4_output_swap_full(o, true, frame, cs ? &upper : NULL, o->nits);
	if (ret)
		goto failed;
	if (cs)
		o->cursor_slot = cslot;
	dcp_m4_kms_account(o->kms, s->copy, ktime_get_ns() - start);
	/* Only a definite successor completion permits releasing the previous pixels. */
	retire = o->active_fb;
	o->active_fb = held;
	if (held)
		o->active_frame = direct;
	o->active_slot = held ? -1 : next;
	o->blanked = false;
	dcp_m4_kms_finish(crtc, 0, true);
	mutex_unlock(&r->dcp->lock);
	if (retire)
		drm_framebuffer_put(retire);
	return;
failed:
	dcp_m4_failed(r->dcp, ret);
	/* Either surface may still be scanned: retain both until reboot. */
	if (held)
		o->pinned = held;
	mutex_unlock(&r->dcp->lock);
	dcp_m4_kms_finish(crtc, ret, false);
}

static void dcp_m4_kms_enable(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	/* No hardware action: the first flush presents a surface. */
	drm_crtc_vblank_on(crtc);
}

static void dcp_m4_kms_disable(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	struct dcp_m4_output *o = to_dcp_m4_output(crtc);
	struct dcp_m4_runtime *r = o->runtime;
	struct drm_framebuffer *retire = NULL;
	int ret;

	mutex_lock(&r->dcp->lock);
	ret = dcp_m4_runtime_live(r);
	if (!ret)
		ret = dcp_m4_output_swap(o, NULL, 0);
	if (ret) {
		dcp_m4_failed(r->dcp, ret);
	} else {
		/* The completed background swap replaced every scanned surface. */
		o->blanked = true;
		retire = o->active_fb;
		o->active_fb = NULL;
		o->active_slot = -1;
		o->cursor_slot = -1;
	}
	mutex_unlock(&r->dcp->lock);
	drm_crtc_vblank_off(crtc);
	/* A modeset that stays active reports through its following flush. */
	if (!crtc->state->active)
		dcp_m4_kms_finish(crtc, ret, false);
	if (retire)
		drm_framebuffer_put(retire);
}

/*
 * Brightness has no RPC of its own: a change re-presents what DCP scans, so an
 * idle compositor needs no new frame. Takes only dcp->lock, the lock every
 * swap holds; it never waits for an atomic commit, and no swap owner waits for
 * the backlight core, so the order ops_lock -> dcp->lock cannot invert.
 */
static int dcp_m4_backlight_update(struct backlight_device *bd)
{
	struct dcp_m4_output *o = bl_get_data(bd);
	struct dcp_m4_runtime *r = o->runtime;
	const struct dcp_m4_frame *frame = NULL;
	struct drm_framebuffer *held = NULL;
	int ret;

	mutex_lock(&r->dcp->lock);
	/* 0 while blanked or powered down; the core already bounded it by the maximum. */
	o->nits = clamp(backlight_get_brightness(bd), 0, DCP_M4_NITS_MAX);
	ret = dcp_m4_runtime_live(r);
	if (ret || o->nits_applied == (int)o->nits)
		goto unlock;
	if (o->active_fb) {
		/* The scanned client buffer must outlive the swap that names its DVA. */
		held = o->active_fb;
		drm_framebuffer_get(held);
		frame = &o->active_frame;
	} else if (o->active_slot >= 0) {
		frame = &o->frame[o->active_slot];
	}
	/* Blanked or nothing scanned yet: the next enabling swap applies the request. */
	if (!frame)
		goto unlock;
	ret = dcp_m4_output_swap(o, frame, o->nits);
	/* On failure the latch keeps active_fb, and so the surface, until reboot. */
	if (!ret)
		atomic64_inc(&o->kms->stats.backlight_swaps);
unlock:
	mutex_unlock(&r->dcp->lock);
	if (held)
		drm_framebuffer_put(held);
	return ret;
}

static const struct backlight_ops dcp_m4_backlight_ops = {
	.update_status = dcp_m4_backlight_update,
};

/*
 * devm on the DCP device, registered after the DRM device and the runtime it
 * uses: unbinding unregisters it first, and the core clears ops under
 * ops_lock, so no update can run once the output is going away. (The driver
 * has no module_exit and suppresses bind attributes; this is the order a
 * future teardown inherits.)
 */
static int dcp_m4_backlight_init(struct dcp_m4_output *o, struct device *parent,
				 const char *name)
{
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.scale = BACKLIGHT_SCALE_LINEAR, /* Nits are proportional to luminance. */
		.max_brightness = DCP_M4_NITS_MAX,
		.brightness = DCP_M4_NITS_DEFAULT,
	};
	struct backlight_device *bd;

	bd = devm_backlight_device_register(parent, name, parent, o, &dcp_m4_backlight_ops,
					    &props);
	if (IS_ERR(bd))
		return PTR_ERR(bd);
	o->backlight = bd;
	return 0;
}

/*
 * Software vblank only; nothing is programmed. Completed swaps are the flip
 * vblanks. While vblank is enabled, a host timer adds an idle-refresh tick when
 * no swap completed for 1.5 frames, so in-kernel waiters (the fbdev damage
 * worker waits for one vblank per update) do not stall for their 1 s timeout.
 */
static bool dcp_m4_kms_vblank_timeout(struct drm_crtc *crtc)
{
	struct dcp_m4_output *o = to_dcp_m4_output(crtc);
	u64 frame = drm_crtc_vblank_crtc(crtc)->framedur_ns;

	if (ktime_get_ns() - READ_ONCE(o->swap_vblank_ns) < frame + frame / 2)
		return true;
	atomic64_inc(&o->kms->stats.idle_vblanks);
	return drm_crtc_handle_vblank(crtc);
}

static int dcp_m4_kms_enable_vblank(struct drm_crtc *crtc)
{
	return drm_crtc_vblank_start_timer(crtc);
}

static void dcp_m4_kms_disable_vblank(struct drm_crtc *crtc)
{
	drm_crtc_vblank_cancel_timer(crtc);
}

static const struct drm_crtc_helper_funcs dcp_m4_kms_crtc_helpers = {
	.atomic_check = dcp_m4_kms_crtc_check,
	.atomic_flush = dcp_m4_kms_flush,
	.atomic_enable = dcp_m4_kms_enable,
	.atomic_disable = dcp_m4_kms_disable,
	.handle_vblank_timeout = dcp_m4_kms_vblank_timeout,
};
static const struct drm_crtc_funcs dcp_m4_kms_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.destroy = drm_crtc_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = dcp_m4_kms_enable_vblank,
	.disable_vblank = dcp_m4_kms_disable_vblank,
};

static int dcp_m4_ext_output_error(struct dcp_m4_output *o);

static int dcp_m4_kms_output_error(struct dcp_m4_output *o)
{
	return o->ext ? dcp_m4_ext_output_error(o) : READ_ONCE(o->runtime->dcp->error);
}

static int dcp_m4_ext_kms_init(struct dcp_m4_kms *k, struct dcp_m4_ready *dcp);

static void dcp_m4_kms_commit_tail(struct drm_atomic_state *state)
{
	struct drm_crtc_state *c;
	struct drm_crtc *crtc;
	int i;

	drm_atomic_helper_commit_modeset_disables(state->dev, state);
	drm_atomic_helper_commit_modeset_enables(state->dev, state);
	drm_atomic_helper_commit_planes(state->dev, state, DRM_PLANE_COMMIT_ACTIVE_ONLY);
	/* Flush and disable consume their events after the synchronous swap. */
	for_each_new_crtc_in_state(state, crtc, c, i)
		if (c->event)
			dcp_m4_kms_finish(crtc, dcp_m4_kms_output_error(to_dcp_m4_output(crtc)),
					  false);
	drm_atomic_helper_commit_hw_done(state);
	drm_atomic_helper_cleanup_planes(state->dev, state);
}

static int dcp_m4_kms_modes(struct drm_connector *connector)
{
	struct dcp_m4_output *o = container_of(connector, struct dcp_m4_output, connector);
	struct drm_display_mode *mode = drm_mode_duplicate(connector->dev, &o->mode);

	if (!mode)
		return 0;
	drm_mode_probed_add(connector, mode);
	/* The built-in panel's physical size lets compositors pick their scale. */
	if (connector->connector_type == DRM_MODE_CONNECTOR_eDP) {
		u32 width = 0, height = 0;

		of_property_read_u32(connector->dev->dev->of_node, "apple,panel-width-mm", &width);
		of_property_read_u32(connector->dev->dev->of_node, "apple,panel-height-mm", &height);
		connector->display_info.width_mm = width;
		connector->display_info.height_mm = height;
	}
	return 1;
}
static const struct drm_connector_helper_funcs dcp_m4_kms_connector_helpers = {
	.get_modes = dcp_m4_kms_modes,
};
static const struct drm_connector_funcs dcp_m4_kms_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};
static const struct drm_encoder_funcs dcp_m4_kms_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};
static const struct drm_mode_config_funcs dcp_m4_kms_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};
static const struct drm_mode_config_helper_funcs dcp_m4_kms_config_helpers = {
	.atomic_commit_tail = dcp_m4_kms_commit_tail,
};
DEFINE_DRM_GEM_FOPS(dcp_m4_kms_fops);
static const struct drm_driver dcp_m4_kms_driver = {
	DRM_GEM_SHMEM_DRIVER_OPS,
	DRM_FBDEV_SHMEM_DRIVER_OPS,
	.gem_create_object = dcp_m4_gem_create_object,
	.name = "apple-dcp-m4",
	.desc = "J713 retained internal display",
	.major = 1,
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &dcp_m4_kms_fops,
};

/* One primary plane, CRTC, encoder and connector for a qualified fixed mode. */
static int dcp_m4_output_init(struct dcp_m4_kms *k, struct dcp_m4_output *o,
			      int connector_type)
{
	static const u32 formats[] = { DRM_FORMAT_XRGB8888, DRM_FORMAT_ARGB8888 };
	static const u64 modifiers[] = { DRM_FORMAT_MOD_LINEAR, DRM_FORMAT_MOD_INVALID };
	struct drm_device *drm = &k->drm;
	int ret;

	ret = drm_universal_plane_init(drm, &o->plane, 0, &dcp_m4_kms_plane_funcs,
			formats, ARRAY_SIZE(formats), modifiers, DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&o->plane, &dcp_m4_kms_plane_helpers);
	if (o->has_cursor) {
		static const u32 cursor_formats[] = { DRM_FORMAT_ARGB8888 };

		ret = drm_universal_plane_init(drm, &o->cursor, 0, &dcp_m4_kms_plane_funcs,
					       cursor_formats, ARRAY_SIZE(cursor_formats),
					       modifiers, DRM_PLANE_TYPE_CURSOR, NULL);
		if (ret)
			return ret;
		drm_plane_helper_add(&o->cursor, &dcp_m4_kms_cursor_helpers);
	}
	ret = drm_crtc_init_with_planes(drm, &o->crtc, &o->plane,
					o->has_cursor ? &o->cursor : NULL,
					&dcp_m4_kms_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&o->crtc, &dcp_m4_kms_crtc_helpers);
	ret = drm_encoder_init(drm, &o->encoder, &dcp_m4_kms_encoder_funcs,
			       DRM_MODE_ENCODER_TMDS, NULL);
	if (ret)
		return ret;
	o->encoder.possible_crtcs = drm_crtc_mask(&o->crtc);
	ret = drm_connector_init(drm, &o->connector, &dcp_m4_kms_connector_funcs,
				 connector_type);
	if (ret)
		return ret;
	drm_connector_helper_add(&o->connector, &dcp_m4_kms_connector_helpers);
	return drm_connector_attach_encoder(&o->connector, &o->encoder);
}

static int dcp_m4_kms_create(struct dcp_m4_ready *dcp)
{
	struct dcp_m4_runtime *r = dcp->runtime;
	struct dcp_m4_timing *t = &r->timing;
	struct dcp_m4_output *o;
	struct dcp_m4_kms *k;
	struct drm_device *drm;
	struct device_node *np;
	struct resource aperture;
	dma_addr_t dva;
	int ret, found = 0;

	/* All hardware resources are prepared before the software aperture handoff. */
	for_each_compatible_node(np, NULL, "simple-framebuffer") {
		if (!of_device_is_available(np))
			continue;
		if (++found != 1 || of_address_to_resource(np, 0, &aperture)) {
			of_node_put(np);
			return -EINVAL;
		}
	}
	if (found != 1)
		return -ENODEV;
	k = devm_drm_dev_alloc(dcp->dev, &dcp_m4_kms_driver, struct dcp_m4_kms, drm);
	if (IS_ERR(k))
		return PTR_ERR(k);
	dcp->kms = k;
	drm = &k->drm;
	ret = drmm_mutex_init(drm, &k->map_lock);
	if (ret)
		return ret;
	o = &k->internal;
	o->kms = k;
	o->runtime = r;
	o->scanout = &dcp->scanout->dev;
	o->frame[0] = o->frame[1] = r->scanout_frame;
	o->pixels[0] = r->scanout_cpu; /* Reuse the completed diagnostic surface. */
	o->active_slot = 0; /* DCP still scans that diagnostic surface. */
	o->nits = DCP_M4_NITS_DEFAULT;
	o->nits_applied = -1; /* Inherited panel state until a swap carries a value. */
	o->pixels[1] = dcp_m4_scanout_alloc(&dcp->scanout->dev,
						   r->scanout_frame.backing_size, &dva);
	if (IS_ERR(o->pixels[1]))
		return PTR_ERR(o->pixels[1]);
	o->frame[1].dva = dva;
	o->mode = (struct drm_display_mode) {
		.clock = t->clock_khz, .hdisplay = t->hdisplay, .hsync_start = t->hsync_start,
		.hsync_end = t->hsync_end, .htotal = t->htotal, .vdisplay = t->vdisplay,
		.vsync_start = t->vsync_start, .vsync_end = t->vsync_end, .vtotal = t->vtotal,
		.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
	};
	drm_mode_set_name(&o->mode);
	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.min_width = drm->mode_config.max_width = t->hdisplay;
	drm->mode_config.min_height = drm->mode_config.max_height = t->vdisplay;
	drm->mode_config.preferred_depth = 24;
	drm->mode_config.funcs = &dcp_m4_kms_config_funcs;
	drm->mode_config.helper_private = &dcp_m4_kms_config_helpers;
	o->cursor_slot = -1;
	if (dcp_m4_cursor_plane > 0 || (dcp_m4_cursor_plane < 0 && dcp_m4_pf->fw14)) {
		const u32 n = DCP_M4_CURSOR_SIZE;

		if (dcp_m4_cursor_index < 1 || dcp_m4_cursor_index > 3)
			return -EINVAL;
		for (u32 i = 0; i < 2; i++) {
			o->cursor_pixels[i] = dcp_m4_scanout_alloc(&dcp->scanout->dev,
								   n * n * 4, &dva);
			if (IS_ERR(o->cursor_pixels[i]))
				return PTR_ERR(o->cursor_pixels[i]);
			o->cursor_frame[i] = (struct dcp_m4_frame) {
				.dva = dva, .backing_size = n * n * 4, .width = n, .height = n,
				.stride = n * 4, .format = DCP_M4_ARGB8888,
			};
		}
		o->cursor_keep = BIT(dcp_m4_cursor_index);
		o->has_cursor = true;
		drm->mode_config.cursor_width = drm->mode_config.cursor_height = n;
		/* Cursor framebuffers are small; the primary check keeps the panel size. */
		drm->mode_config.min_width = drm->mode_config.min_height = 1;
	}
	ret = dcp_m4_output_init(k, o, DRM_MODE_CONNECTOR_eDP);
	if (ret)
		return ret;
	/* Optional external outputs need their CRTCs before vblank init. */
	ret = dcp_m4_ext_kms_init(k, dcp);
	if (ret)
		return ret;
	/*
	 * External outputs: no larger than the panel. The fbdev emulation
	 * shares one surface, sized for the largest mode, across all CRTCs,
	 * and the panel plane scans out exactly its own size; a larger
	 * external mode (e.g. 3840x2160@30) would make that commit fail.
	 */
	if (k->ext_count && !o->has_cursor) {
		drm->mode_config.min_width = min_t(u32, 640, t->hdisplay);
		drm->mode_config.min_height = min_t(u32, 480, t->vdisplay);
	}
	ret = drm_vblank_init(drm, drm->mode_config.num_crtc);
	if (ret)
		return ret;
	/* DRM starts with the CRTC off; atomic_enable turns vblank accounting on. */
	drm_crtc_vblank_reset(&o->crtc);
	drm_mode_config_reset(drm);
	/* Startup qualification: the last full DART/PMP/CPU check before KMS. */
	ret = dcp_m4_runtime_health(r);
	if (ret)
		return ret;
	WRITE_ONCE(r->qualified, true);
	ret = aperture_remove_conflicting_devices(aperture.start, resource_size(&aperture),
						 dcp_m4_kms_driver.name);
	if (ret)
		return ret;
	ret = drm_dev_register(drm, 0);
	if (!ret)
		smp_store_release(&k->registered, true);
	return ret;
}

static ssize_t kms_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dcp_m4_ready *dcp = dev_get_drvdata(dev);
	struct dcp_m4_kms *k = READ_ONCE(dcp->kms);

	return sysfs_emit(buf, "attempted=%u registered=%u error=%d flips=%llu\n",
		READ_ONCE(dcp->kms_started), k && smp_load_acquire(&k->registered),
		READ_ONCE(dcp->error), k ? READ_ONCE(k->stats.flips) : 0);
}

static ssize_t kms_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct dcp_m4_ready *dcp = dev_get_drvdata(dev);
	int ret = -EBUSY;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;
	mutex_lock(&dcp->lock);
	if (dcp->kms_started || !dcp->runtime || !dcp->runtime->scanout_done ||
	    READ_ONCE(dcp->error))
		goto unlock;
	dcp->kms_started = true;
	ret = dcp_m4_kms_create(dcp);
	if (ret)
		dcp_m4_failed(dcp, ret);
unlock:
	mutex_unlock(&dcp->lock);
	if (ret)
		return ret;
	/*
	 * Neither step may hold the RPC lock: fbcon's atomic commit takes it, and
	 * the backlight core calls update_status, which takes it, under its own
	 * device-list and ops locks. Display works without brightness control.
	 */
	ret = dcp_m4_backlight_init(&dcp->kms->internal, dcp->dev, "apple-panel-bl");
	if (ret)
		dev_warn(dev, "Panel backlight unavailable: %d\n", ret);
	drm_client_setup(&dcp->kms->drm, drm_format_info(DRM_FORMAT_XRGB8888));
	return count;
}
static DEVICE_ATTR_RW(kms);

/* REMOVE-DEBUG: flip path, swap latency and client mapping lifetime counters. */
static ssize_t kms_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dcp_m4_ready *dcp = dev_get_drvdata(dev);
	struct dcp_m4_kms *k = READ_ONCE(dcp->kms);
	struct dcp_m4_kms_stats *s;

	if (!k || !smp_load_acquire(&k->registered))
		return sysfs_emit(buf, "registered=0\n");
	s = &k->stats;
	return sysfs_emit(buf,
		"flips=%llu direct=%llu copy_local=%llu copy_layout=%llu copy_map=%llu "
		"copy_disabled=%llu swap_ns_last=%llu swap_ns_max=%llu swap_ns_total=%llu "
		"maps=%lld map_failures=%lld unmaps=%lld retained=%lld idle_vblanks=%lld "
		"backlight_swaps=%lld cursor_swaps=%lld nits=%u nits_applied=%d\n",
		READ_ONCE(s->flips), READ_ONCE(s->direct),
		READ_ONCE(s->copy[DCP_M4_COPY_LOCAL]), READ_ONCE(s->copy[DCP_M4_COPY_LAYOUT]),
		READ_ONCE(s->copy[DCP_M4_COPY_MAP]), READ_ONCE(s->copy[DCP_M4_COPY_DISABLED]),
		READ_ONCE(s->swap_ns_last),
		READ_ONCE(s->swap_ns_max), READ_ONCE(s->swap_ns_total),
		atomic64_read(&s->maps), atomic64_read(&s->map_failures),
		atomic64_read(&s->unmaps), atomic64_read(&s->retained),
		atomic64_read(&s->idle_vblanks), atomic64_read(&s->backlight_swaps),
		atomic64_read(&s->cursor_swaps),
		READ_ONCE(k->internal.nits), READ_ONCE(k->internal.nits_applied));
}
static DEVICE_ATTR_RO(kms_stats);
