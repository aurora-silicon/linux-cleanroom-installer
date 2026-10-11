// SPDX-License-Identifier: GPL-2.0-only
/* Canonical retained timestamp translations; no runtime or registration gate. */
#include <linux/err.h>
#include <linux/bug.h>
#include <linux/export.h>
#include <linux/limits.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include "gpu-m4-drm-provider-timestamp-alias.h"

struct timestamp_entry {
	struct list_head link;
	struct gpu_m4_drm_timestamp_aliases *aliases;
	struct gpu_m4_drm_provider_version *version;
	struct gpu_m4_drm_provider_object *object;
	struct gpu_m4_drm_upper_backing backing;
	struct gpu_m4_drm_upper_lease *lease;
	u64 *pa, charge;
	u32 pages, upper_refs;
	bool failed, lease_ref;
};

struct gpu_m4_drm_timestamp_alias_request {
	struct list_head link;
	struct timestamp_entry *entry;
	struct gpu_m4_drm_provider_version *version;
	struct gpu_m4_drm_provider_object *object;
	struct gpu_m4_drm_provider_timestamp_cookie *cookie;
	u64 relative, offset, va;
	u32 refs;
	bool lease_ref;
};

struct gpu_m4_drm_timestamp_alias_token {
	struct list_head link;
	struct gpu_m4_drm_timestamp_aliases *aliases;
	struct gpu_m4_drm_provider_job *job;
	struct gpu_m4_drm_timestamp_alias_request **request;
	u64 charge;
	/* Upper owner revision at which every request's alias last translated
	 * exactly (0: none). Requests and entries are immutable once published;
	 * any lease/table change advances the revision.
	 */
	u64 walked_revision;
	u32 count;
	bool armed, published, uncertain, faulted, job_ref, garbage;
};

struct gpu_m4_drm_timestamp_aliases {
	struct gpu_m4_drm_timestamp_alias_config config;
	struct mutex *lock;
	struct list_head entries, requests, tokens;
	/* Armed tokens in arm order, at most the lane cap. Two may share
	 * entries; request refs keep their leases. Only the token inside its
	 * own publish is publishing: the one whose new upper aliases a backing
	 * readiness check may admit.
	 */
	struct gpu_m4_drm_timestamp_alias_token *armed[GPU_M4_DRM_TIMESTAMP_ALIAS_MAX_ARMED];
	struct gpu_m4_drm_timestamp_alias_token *publishing;
	u32 armed_count;
	u64 bytes;
	u32 entry_count, page_count, request_count, token_count, reapers;
	int error;
	bool closed;
};

static int timestamp_status(int ret)
{
	return ret > 0 ? -EPROTO : ret;
}

static int timestamp_owner(struct gpu_m4_drm_timestamp_aliases *a)
{
	int ret;

	lockdep_assert_held(a->lock);
	if (a->closed || a->error)
		return a->error ? a->error : -ENODEV;
	ret = gpu_m4_drm_provider_error_locked(a->config.provider);
	if (!ret)
		ret = a->config.owner_check_locked(a->config.owner, a->config.provider,
			a->config.upper, a->config.timestamp.frontend, a->lock);
	return timestamp_status(ret);
}

static int timestamp_charge(struct gpu_m4_drm_timestamp_aliases *a, u64 bytes)
{
	if (bytes > a->config.memory_limit - a->bytes)
		return -ENOSPC;
	a->bytes += bytes;
	return 0;
}

static int timestamp_entry_origin(struct timestamp_entry *e)
{
	struct gpu_m4_drm_timestamp_aliases *a = e->aliases;
	int ret = timestamp_owner(a);

	if (!ret)
		ret = gpu_m4_drm_provider_object_origin_locked(a->config.provider,
			e->version, e->object);
	if (!ret && (gpu_m4_drm_provider_object_identity(e->object) != e->backing.identity ||
	    gpu_m4_drm_bo_pin_size(gpu_m4_drm_provider_object_native(e->object)) != e->backing.size ||
	    gpu_m4_drm_bo_policy(gpu_m4_drm_provider_object_identity(e->object)) != e->backing.policy))
		ret = -EPERM;
	return timestamp_status(ret);
}

static int timestamp_entry_valid(struct timestamp_entry *e)
{
	u64 pa;
	u32 i;
	int ret = timestamp_entry_origin(e);

	for (i = 0; !ret && i < e->pages; i++) {
		ret = gpu_m4_drm_provider_object_page_locked(e->object,
			(u64)i * GPU_M4_DRM_PAGE_SIZE, &pa);
		if (!ret && pa != e->pa[i])
			ret = -EPROTO;
	}
	return timestamp_status(ret);
}

static int timestamp_backing_get(void *owner)
{
	struct timestamp_entry *e = owner;
	int ret = timestamp_entry_valid(e);

	if (!ret) {
		if (e->upper_refs == U32_MAX)
			return -EOVERFLOW;
		e->upper_refs++;
	}
	return ret;
}

static void timestamp_backing_put(void *owner)
{
	struct timestamp_entry *e = owner;

	/* upper guarantees outside-lock puts, including failed acceptance. */
	mutex_lock(e->aliases->lock);
	if (!WARN_ON(!e->upper_refs) && !--e->upper_refs)
		e->lease = NULL;
	mutex_unlock(e->aliases->lock);
}

static int timestamp_backing_page(void *owner, u64 offset, u64 *pa)
{
	struct timestamp_entry *e = owner;
	u64 actual;
	int ret;

	if (!pa || (offset & (GPU_M4_DRM_PAGE_SIZE - 1)) || offset >= e->backing.size)
		return -EINVAL;
	ret = gpu_m4_drm_provider_object_page_locked(e->object, offset, &actual);
	if (!ret && actual != e->pa[offset / GPU_M4_DRM_PAGE_SIZE])
		ret = -EPROTO;
	if (!ret)
		*pa = actual;
	return timestamp_status(ret);
}

static int timestamp_backing_check(void *owner, const void *identity, u64 offset, u64 pa)
{
	struct timestamp_entry *e = owner;
	u64 actual;
	int ret;

	if (identity != e->backing.identity)
		return -EPERM;
	/* upper independently visits every page; do not nest a whole inventory
	 * walk inside each page check (the bounded handoff must remain linear).
	 */
	ret = timestamp_entry_origin(e);
	if (!ret)
		ret = timestamp_backing_page(e, offset, &actual);
	return ret ? ret : actual == pa ? 0 : -EPROTO;
}

static int timestamp_request_valid(struct gpu_m4_drm_timestamp_alias_request *r)
{
	struct timestamp_entry *e = r->entry;
	struct gpu_m4_drm_timestamp_aliases *a = e->aliases;
	struct gpu_m4_drm_timestamp checked;
	u64 pa;
	u32 i;
	int ret = timestamp_entry_valid(e);

	if (!ret)
		ret = gpu_m4_drm_provider_object_origin_locked(a->config.provider, r->version, r->object);
	if (!ret)
		ret = gpu_m4_drm_provider_timestamp_slice(r->cookie, r->relative, &checked);
	if (!ret && (!r->refs || gpu_m4_drm_provider_object_identity(r->object) != e->backing.identity ||
	    gpu_m4_drm_bo_pin_size(gpu_m4_drm_provider_object_native(r->object)) != e->backing.size ||
	    gpu_m4_drm_bo_policy(gpu_m4_drm_provider_object_identity(r->object)) != e->backing.policy ||
	    !gpu_m4_drm_provider_timestamp_matches(r->cookie, &a->config.timestamp) ||
	    checked.backend != r->cookie || checked.object != e->backing.identity ||
	    checked.offset != r->relative || checked.object_offset != r->offset ||
	    !e->lease || !r->lease_ref ||
	    gpu_m4_drm_upper_va_locked(e->lease) + r->offset != r->va))
		ret = -EPERM;
	for (i = 0; !ret && i < e->pages; i++) {
		/* Current-VM pin, NEVER the first anchor pin as current cache proof. */
		ret = gpu_m4_drm_provider_object_page_locked(r->object,
			(u64)i * GPU_M4_DRM_PAGE_SIZE, &pa);
		if (!ret && pa != e->pa[i])
			ret = -EPROTO;
	}
	return timestamp_status(ret);
}

static int timestamp_selected_valid(struct gpu_m4_drm_timestamp_alias_token *t)
{
	struct gpu_m4_drm_timestamp_aliases *a = t->aliases;
	struct gpu_m4_drm_provider_job_timestamp_view view;
	u32 i, count;
	int ret = timestamp_owner(a);

	if (!ret)
		ret = a->config.lane_check_locked(a->config.owner, t->job);
	if (!ret)
		ret = gpu_m4_drm_provider_job_timestamp_count_locked(t->job, &count);
	if (!ret && count != t->count)
		ret = -ESTALE;
	for (i = 0; !ret && i < t->count; i++) {
		ret = timestamp_request_valid(t->request[i]);
		if (!ret)
			ret = gpu_m4_drm_provider_job_timestamp_at_locked(t->job, i, &view);
		if (!ret && (view.runtime_alias != t->request[i] ||
		    view.object != t->request[i]->object || view.object_offset != t->request[i]->offset ||
		    view.firmware_va != t->request[i]->va ||
		    view.gem != gpu_m4_drm_provider_object_identity(t->request[i]->object)))
			ret = -ESTALE;
		if (!ret)
			ret = gpu_m4_drm_provider_job_timestamp_check_locked(t->job, t->request[i], false);
	}
	return timestamp_status(ret);
}

static bool timestamp_token_armed(const struct gpu_m4_drm_timestamp_aliases *a,
	const struct gpu_m4_drm_timestamp_alias_token *t)
{
	for (u32 i = 0; i < a->armed_count && i < GPU_M4_DRM_TIMESTAMP_ALIAS_MAX_ARMED; i++)
		if (a->armed[i] == t)
			return true;
	return false;
}

static int timestamp_token_valid(struct gpu_m4_drm_timestamp_alias_token *t)
{
	lockdep_assert_held(t->aliases->lock);
	if (!t->armed || t->garbage || !timestamp_token_armed(t->aliases, t) || !t->job_ref)
		return -EPERM;
	return timestamp_selected_valid(t);
}

static int timestamp_backing_ready(void *owner, u64 offset, u64 size,
	enum gpu_m4_drm_upper_class class)
{
	struct timestamp_entry *e = owner;
	struct gpu_m4_drm_timestamp_alias_token *t = e->aliases->publishing;
	u32 i;
	bool selected = false;
	int ret;

	if (offset || size != e->backing.size || class != GPU_M4_DRM_UPPER_FW_ONLY || !t)
		return -EPERM;
	ret = timestamp_token_valid(t);
	for (i = 0; !ret && i < t->count; i++)
		if (t->request[i]->entry == e)
			selected = true;
	return ret ? ret : selected ? 0 : -EPERM;
}

struct gpu_m4_drm_timestamp_aliases *gpu_m4_drm_timestamp_alias_create(
	const struct gpu_m4_drm_timestamp_alias_config *c)
{
	struct gpu_m4_drm_timestamp_aliases *a;
	int ret;

	if (!c || IS_ERR_OR_NULL(c->provider) || IS_ERR_OR_NULL(c->upper) ||
	    !c->timestamp.frontend || !c->timestamp.owner || !c->timestamp.owner_lock ||
	    !c->timestamp.session_get || !c->timestamp.session_put || !c->owner ||
	    !c->owner_check_locked || !c->lane_check_locked ||
	    gpu_m4_drm_provider_lock(c->provider) != c->timestamp.owner_lock ||
	    !c->max_entries || c->max_entries > GPU_M4_DRM_TIMESTAMP_ALIAS_MAX_ENTRIES ||
	    !c->max_pages || c->max_pages > GPU_M4_DRM_TIMESTAMP_ALIAS_MAX_PAGES ||
	    !c->max_requests || c->max_requests > GPU_M4_DRM_TIMESTAMP_ALIAS_MAX_REQUESTS ||
	    !c->max_tokens || c->max_tokens > GPU_M4_DRM_TIMESTAMP_ALIAS_MAX_TOKENS ||
	    c->max_armed > GPU_M4_DRM_TIMESTAMP_ALIAS_MAX_ARMED || c->memory_limit < sizeof(*a))
		return ERR_PTR(-EINVAL);
	lockdep_assert_not_held(c->timestamp.owner_lock);
	a = kzalloc_obj(*a);
	if (!a)
		return ERR_PTR(-ENOMEM);
	a->config = *c;
	a->config.max_armed = c->max_armed ?: 1;
	a->lock = c->timestamp.owner_lock;
	a->bytes = sizeof(*a);
	INIT_LIST_HEAD(&a->entries);
	INIT_LIST_HEAD(&a->requests);
	INIT_LIST_HEAD(&a->tokens);
	ret = gpu_m4_drm_provider_get(c->provider);
	if (ret)
		goto failed;
	ret = timestamp_status(c->timestamp.session_get(c->timestamp.owner));
	if (ret)
		goto provider_put;
	mutex_lock(a->lock);
	ret = timestamp_owner(a);
	mutex_unlock(a->lock);
	if (!ret)
		return a;
	c->timestamp.session_put(c->timestamp.owner);
provider_put:
	gpu_m4_drm_provider_put(c->provider);
failed:
	kfree(a);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_timestamp_alias_create);

int gpu_m4_drm_timestamp_alias_hold_locked(struct gpu_m4_drm_timestamp_aliases *a,
	struct gpu_m4_drm_provider_version *v, struct gpu_m4_drm_provider_object *o,
	const struct gpu_m4_drm_timestamp *borrowed,
	struct gpu_m4_drm_timestamp_alias_request **out, u64 *firmware_va)
{
	struct gpu_m4_drm_timestamp_alias_request *r;
	struct gpu_m4_drm_provider_timestamp_cookie *cookie;
	struct gpu_m4_drm_timestamp checked;
	struct timestamp_entry *e = NULL, *candidate;
	u64 size, charge, base, pa;
	u32 pages, i;
	int ret;

	if (!a || !borrowed || !out || !firmware_va || !v || !o)
		return -EINVAL;
	ret = timestamp_owner(a);
	if (ret)
		return ret;
	ret = gpu_m4_drm_provider_object_origin_locked(a->config.provider, v, o);
	if (ret)
		return ret;
	if (a->request_count == a->config.max_requests)
		return -ENOSPC;
	ret = timestamp_charge(a, sizeof(*r));
	if (ret)
		return ret;
	r = kzalloc_obj(*r);
	if (!r) {
		a->bytes -= sizeof(*r);
		return -ENOMEM;
	}
	/* The serialized borrowed backend is an actual cookie, not arbitrary
	 * userspace input. Atomic get precedes every configuration/slice read.
	 */
	cookie = borrowed->backend;
	if (!gpu_m4_drm_provider_timestamp_get(cookie)) {
		ret = -ENOENT;
		goto free_request;
	}
	r->cookie = cookie;
	if (!gpu_m4_drm_provider_timestamp_matches(cookie, &a->config.timestamp)) {
		ret = -EPERM;
		goto queued_request;
	}
	ret = gpu_m4_drm_provider_timestamp_slice(cookie, borrowed->offset, &checked);
	if (ret)
		goto queued_request;
	if (checked.backend != borrowed->backend || checked.object != borrowed->object ||
	    checked.offset != borrowed->offset || checked.object_offset != borrowed->object_offset ||
	    checked.object != gpu_m4_drm_provider_object_identity(o)) {
		ret = -EPERM;
		goto queued_request;
	}
	size = gpu_m4_drm_bo_pin_size(gpu_m4_drm_provider_object_native(o));
	pages = size / GPU_M4_DRM_PAGE_SIZE;
	if (!size || (size & (GPU_M4_DRM_PAGE_SIZE - 1)) ||
	    size / GPU_M4_DRM_PAGE_SIZE > a->config.max_pages) {
		ret = -ENOSPC;
		goto queued_request;
	}
	list_for_each_entry(candidate, &a->entries, link)
		if (!candidate->failed && candidate->backing.identity == checked.object &&
		    candidate->backing.size == size &&
		    candidate->backing.policy == gpu_m4_drm_provider_timestamp_policy(cookie))
			e = candidate;
	if (!e) {
		if (a->entry_count == a->config.max_entries || pages > a->config.max_pages - a->page_count) {
			ret = -ENOSPC;
			goto queued_request;
		}
		charge = sizeof(*e) + (u64)pages * sizeof(*e->pa);
		ret = timestamp_charge(a, charge);
		if (ret)
			goto queued_request;
		e = kzalloc_obj(*e);
		if (!e) {
			a->bytes -= charge;
			ret = -ENOMEM;
			goto queued_request;
		}
		e->pa = kcalloc(pages, sizeof(*e->pa), GFP_KERNEL);
		if (!e->pa) {
			kfree(e);
			a->bytes -= charge;
			ret = -ENOMEM;
			goto queued_request;
		}
		e->aliases = a;
		e->charge = charge;
		e->pages = pages;
		e->failed = true;
		list_add_tail(&e->link, &a->entries);
		a->entry_count++;
		a->page_count += pages;
		ret = gpu_m4_drm_provider_version_get_locked(v);
		if (ret)
			goto queued_request;
		e->version = v;
		ret = gpu_m4_drm_provider_object_get_locked(o);
		if (ret)
			goto queued_request;
		e->object = o;
		e->backing = (struct gpu_m4_drm_upper_backing) {
			.owner = e, .identity = checked.object, .size = size,
			.policy = gpu_m4_drm_provider_timestamp_policy(cookie),
			.get_locked = timestamp_backing_get, .put = timestamp_backing_put,
			.page_locked = timestamp_backing_page, .check_locked = timestamp_backing_check,
			.ready_locked = timestamp_backing_ready,
		};
		for (i = 0; i < pages; i++) {
			ret = gpu_m4_drm_provider_object_page_locked(o, (u64)i * GPU_M4_DRM_PAGE_SIZE, &e->pa[i]);
			if (ret)
				goto queued_request;
		}
		e->lease = gpu_m4_drm_upper_accept_locked(a->config.upper, &e->backing,
			GPU_M4_DRM_UPPER_TS, GPU_M4_DRM_UPPER_FW_ONLY);
		if (IS_ERR(e->lease)) {
			ret = PTR_ERR(e->lease);
			e->lease = NULL;
			goto queued_request;
		}
		e->lease_ref = true;
		e->failed = false;
	}
	ret = timestamp_entry_valid(e);
	for (i = 0; !ret && i < pages; i++) {
		ret = gpu_m4_drm_provider_object_page_locked(o, (u64)i * GPU_M4_DRM_PAGE_SIZE, &pa);
		if (!ret && pa != e->pa[i])
			ret = -EPROTO;
	}
	if (ret)
		goto queued_request;
	base = gpu_m4_drm_upper_va_locked(e->lease);
	if ((checked.object_offset & 7) || checked.object_offset >= size ||
	    size - checked.object_offset < 8 || base < GPU_M4_DRM_UPPER_TIMESTAMP ||
	    base >= GPU_M4_DRM_UPPER_END || size > GPU_M4_DRM_UPPER_END - base ||
	    GPU_M4_DRM_UPPER_END - base - size < GPU_M4_DRM_PAGE_SIZE) {
		ret = -EINVAL;
		goto queued_request;
	}
	ret = gpu_m4_drm_upper_reference_locked(e->lease, &e->backing,
		GPU_M4_DRM_UPPER_TS, GPU_M4_DRM_UPPER_FW_ONLY);
	if (ret)
		goto queued_request;
	r->entry = e;
	r->lease_ref = true;
	ret = gpu_m4_drm_provider_version_get_locked(v);
	if (ret)
		goto queued_request;
	r->version = v;
	ret = gpu_m4_drm_provider_object_get_locked(o);
	if (ret)
		goto queued_request;
	r->object = o;
	r->relative = checked.offset;
	r->offset = checked.object_offset;
	r->va = base + checked.object_offset;
	r->refs = 1;
	list_add_tail(&r->link, &a->requests);
	a->request_count++;
	*out = r;
	*firmware_va = r->va;
	return 0;
queued_request:
	/* Owned cookie puts must wait for outside-lock reap, even on rollback. */
	if (r->lease_ref)
		gpu_m4_drm_upper_put_locked(r->entry->lease);
	r->lease_ref = false;
	r->entry = e;
	list_add_tail(&r->link, &a->requests);
	a->request_count++;
	return timestamp_status(ret);
free_request:
	a->bytes -= sizeof(*r);
	kfree(r);
	return ret;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_timestamp_alias_hold_locked);

void gpu_m4_drm_timestamp_alias_request_put_locked(struct gpu_m4_drm_timestamp_alias_request *r)
{
	if (!r)
		return;
	lockdep_assert_held(r->entry->aliases->lock);
	if (WARN_ON(!r->refs))
		return;
	if (!--r->refs && r->lease_ref) {
		gpu_m4_drm_upper_put_locked(r->entry->lease);
		r->lease_ref = false;
	}
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_timestamp_alias_request_put_locked);

static struct gpu_m4_drm_timestamp_alias_request *timestamp_known_request(
	struct gpu_m4_drm_timestamp_aliases *a, void *identity)
{
	struct gpu_m4_drm_timestamp_alias_request *r;

	list_for_each_entry(r, &a->requests, link)
		if (r == identity && r->refs)
			return r;
	return NULL;
}

int gpu_m4_drm_timestamp_alias_request_get_locked(struct gpu_m4_drm_timestamp_aliases *a,
	struct gpu_m4_drm_timestamp_alias_request *request)
{
	struct gpu_m4_drm_timestamp_alias_request *r;
	int ret;

	if (!a || !request)
		return -EINVAL;
	ret = timestamp_owner(a);
	if (ret)
		return ret;
	/* Compare only identities until the actual retained list proves ownership. */
	r = timestamp_known_request(a, request);
	if (!r)
		return -EPERM;
	ret = timestamp_request_valid(r);
	if (ret)
		return ret;
	if (r->refs == U32_MAX)
		return -EOVERFLOW;
	r->refs++;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_timestamp_alias_request_get_locked);

int gpu_m4_drm_timestamp_alias_token_prepare_locked(struct gpu_m4_drm_timestamp_aliases *a,
	struct gpu_m4_drm_provider_job *job, struct gpu_m4_drm_timestamp_alias_token **out)
{
	struct gpu_m4_drm_timestamp_alias_token *t;
	struct gpu_m4_drm_provider_job_timestamp_view view;
	struct gpu_m4_drm_timestamp_alias_request *r;
	u64 charge;
	u32 count, i;
	int ret;

	if (!a || !job || !out)
		return -EINVAL;
	ret = timestamp_owner(a);
	if (ret)
		return ret;
	ret = gpu_m4_drm_provider_job_check_accepting_locked(job, a->config.provider);
	if (ret)
		return ret;
	ret = gpu_m4_drm_provider_job_timestamp_count_locked(job, &count);
	if (ret)
		return ret;
	if (!count || count > GPU_M4_DRM_TIMESTAMP_ALIAS_JOB_REQUESTS ||
	    count > a->config.max_requests || a->token_count == a->config.max_tokens)
		return -ENOSPC;
	charge = sizeof(*t) + (u64)count * sizeof(*t->request);
	ret = timestamp_charge(a, charge);
	if (ret)
		return ret;
	t = kzalloc_obj(*t);
	if (!t) {
		a->bytes -= charge;
		return -ENOMEM;
	}
	t->request = kcalloc(count, sizeof(*t->request), GFP_KERNEL);
	if (!t->request) {
		kfree(t);
		a->bytes -= charge;
		return -ENOMEM;
	}
	t->aliases = a;
	t->job = job; /* borrowed ONLY through synchronous accept/runtime owner */
	t->charge = charge;
	list_add_tail(&t->link, &a->tokens);
	a->token_count++;
	for (i = 0; i < count; i++) {
		ret = gpu_m4_drm_provider_job_timestamp_at_locked(job, i, &view);
		if (ret)
			break;
		r = timestamp_known_request(a, view.runtime_alias);
		if (!r || r->object != view.object || r->offset != view.object_offset ||
		    r->va != view.firmware_va ||
		    gpu_m4_drm_provider_object_identity(r->object) != view.gem) {
			ret = -EPERM;
			break;
		}
		ret = timestamp_request_valid(r);
		if (ret)
			break;
		if (r->refs == U32_MAX) {
			ret = -EOVERFLOW;
			break;
		}
		r->refs++;
		t->request[t->count++] = r;
	}
	if (ret) {
		for (i = 0; i < t->count; i++)
			gpu_m4_drm_timestamp_alias_request_put_locked(t->request[i]);
		t->count = 0;
		t->garbage = true;
		return timestamp_status(ret);
	}
	*out = t;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_timestamp_alias_token_prepare_locked);

static void timestamp_token_detach(struct gpu_m4_drm_timestamp_alias_token *t)
{
	struct gpu_m4_drm_timestamp_aliases *a = t->aliases;
	u32 i, n = 0;

	for (i = 0; i < a->armed_count && i < GPU_M4_DRM_TIMESTAMP_ALIAS_MAX_ARMED; i++)
		if (a->armed[i] != t)
			a->armed[n++] = a->armed[i];
	for (i = n; i < GPU_M4_DRM_TIMESTAMP_ALIAS_MAX_ARMED; i++)
		a->armed[i] = NULL;
	a->armed_count = n;
	if (a->publishing == t)
		a->publishing = NULL;
	for (i = 0; i < t->count; i++)
		gpu_m4_drm_timestamp_alias_request_put_locked(t->request[i]);
	t->count = 0;
	t->garbage = true;
}

int gpu_m4_drm_timestamp_alias_token_cancel_locked(struct gpu_m4_drm_timestamp_alias_token *t)
{
	if (!t)
		return -EINVAL;
	lockdep_assert_held(t->aliases->lock);
	if (t->garbage)
		return -EINVAL;
	if (t->armed || t->uncertain || gpu_m4_drm_provider_job_uncertain_locked(t->job))
		return -EBUSY;
	timestamp_token_detach(t);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_timestamp_alias_token_cancel_locked);

int gpu_m4_drm_timestamp_alias_token_arm_locked(struct gpu_m4_drm_timestamp_alias_token *t,
	struct gpu_m4_drm_provider_job *job)
{
	struct gpu_m4_drm_timestamp_aliases *a;
	int ret;

	if (!t || !job)
		return -EINVAL;
	a = t->aliases;
	ret = timestamp_owner(a);
	if (!ret && (t->garbage || t->armed || t->job != job ||
		     a->armed_count >= a->config.max_armed ||
		     a->armed_count >= GPU_M4_DRM_TIMESTAMP_ALIAS_MAX_ARMED))
		ret = -EPERM;
	if (!ret)
		ret = timestamp_selected_valid(t);
	if (!ret)
		ret = gpu_m4_drm_provider_job_get_locked(job);
	if (ret)
		return timestamp_status(ret);
	t->job_ref = t->armed = true;
	a->armed[a->armed_count++] = t;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_timestamp_alias_token_arm_locked);

void gpu_m4_drm_timestamp_alias_token_fault_locked(struct gpu_m4_drm_timestamp_alias_token *t,
	int error)
{
	if (!t)
		return;
	lockdep_assert_held(t->aliases->lock);
	if (!t->aliases->error)
		t->aliases->error = error < 0 ? error : -EIO;
	t->uncertain = true;
	t->faulted = true;
	/* Actual provider/job fault owner decides fence disposal; never success. */
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_timestamp_alias_token_fault_locked);

int gpu_m4_drm_timestamp_alias_token_publish_locked(struct gpu_m4_drm_timestamp_alias_token *t,
	bool trial)
{
	u32 i, j;
	int ret;

	if (!t)
		return -EINVAL;
	ret = timestamp_token_valid(t);
	if (ret)
		return ret;
	if (t->published || t->uncertain || t->aliases->publishing)
		return -EBUSY;
	t->uncertain = true; /* BEFORE any possible upper parent/leaf store */
	t->aliases->publishing = t;
	for (i = 0; i < t->count; i++) {
		for (j = 0; j < i; j++)
			if (t->request[j]->entry == t->request[i]->entry)
				break;
		if (j != i)
			continue;
		ret = gpu_m4_drm_upper_publish_locked(t->request[i]->entry->lease, trial);
		if (ret) {
			t->aliases->publishing = NULL;
			gpu_m4_drm_timestamp_alias_token_fault_locked(t, ret);
			return timestamp_status(ret);
		}
	}
	t->aliases->publishing = NULL;
	t->published = true;
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_timestamp_alias_token_publish_locked);

static bool token_walk_memo = true;
#ifdef MODULE
module_param(token_walk_memo, bool, 0644);
MODULE_PARM_DESC(token_walk_memo, "Reuse a timestamp token's alias walk at an unchanged upper revision (default 1)");
#endif

int gpu_m4_drm_timestamp_alias_token_check_locked(struct gpu_m4_drm_timestamp_alias_token *t)
{
	u64 pa, attrs, expected, revision;
	u32 i;
	int ret;

	if (!t)
		return -EINVAL;
	ret = timestamp_token_valid(t);
	if (!ret && !t->published)
		ret = -EPERM;
	revision = ret ? 0 : gpu_m4_drm_upper_revision_locked(t->aliases->config.upper);
	if (!ret && revision && t->walked_revision == revision && token_walk_memo)
		return 0;
	t->walked_revision = 0;
	for (i = 0; !ret && i < t->count; i++) {
		struct gpu_m4_drm_timestamp_alias_request *r = t->request[i];

		ret = gpu_m4_drm_upper_walk_locked(t->aliases->config.upper, r->va, &pa, &attrs);
		expected = r->entry->pa[r->offset / GPU_M4_DRM_PAGE_SIZE] |
			(r->offset & (GPU_M4_DRM_PAGE_SIZE - 1));
		if (!ret && (pa != expected || attrs != GPU_M4_DRM_UPPER_FW_ATTRS))
			ret = -EPROTO;
	}
	if (!ret)
		t->walked_revision = revision;
	return timestamp_status(ret);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_timestamp_alias_token_check_locked);

void gpu_m4_drm_timestamp_alias_token_complete_locked(struct gpu_m4_drm_timestamp_alias_token *t)
{
	/* Actual lane success notification is the authority here. ALL fallible
	 * validation happened in its earlier full retirement predicate. Infallible
	 * metadata/ref transfers do not invent a later success/failure decision.
	 */
	lockdep_assert_held(t->aliases->lock);
	if (WARN_ON(t->garbage || t->faulted || !t->armed || !t->published ||
		    !timestamp_token_armed(t->aliases, t)))
		return;
	timestamp_token_detach(t);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_timestamp_alias_token_complete_locked);

void gpu_m4_drm_timestamp_alias_reap(struct gpu_m4_drm_timestamp_aliases *a)
{
	struct gpu_m4_drm_timestamp_alias_request *r;
	struct gpu_m4_drm_timestamp_alias_token *t;
	struct timestamp_entry *e;
	bool found;

	if (!a)
		return;
	lockdep_assert_not_held(a->lock);
	mutex_lock(a->lock);
	if (a->reapers) {
		mutex_unlock(a->lock);
		return;
	}
	a->reapers++;
	do {
		found = false;
		list_for_each_entry(t, &a->tokens, link) {
			if (!t->garbage)
				continue;
			list_del(&t->link);
			a->token_count--;
			a->bytes -= t->charge;
			mutex_unlock(a->lock);
			if (t->job_ref)
				WARN_ON(gpu_m4_drm_provider_job_put(t->job));
			kfree(t->request);
			kfree(t);
			mutex_lock(a->lock);
			found = true;
			break;
		}
	} while (found);
	do {
		found = false;
		list_for_each_entry(r, &a->requests, link) {
			if (r->refs)
				continue;
			if (r->version && gpu_m4_drm_provider_version_put_locked(r->version))
				continue; /* Refuse to lose a still-protected retained owner. */
			if (r->object)
				gpu_m4_drm_provider_object_put_locked(r->object);
			list_del(&r->link);
			a->request_count--;
			a->bytes -= sizeof(*r);
			mutex_unlock(a->lock);
			gpu_m4_drm_provider_timestamp_put(r->cookie);
			kfree(r);
			mutex_lock(a->lock);
			found = true;
			break;
		}
	} while (found);
	/* Failed acceptance backing puts need the real upper outside-lock reap. */
	mutex_unlock(a->lock);
	if (!a->closed)
		gpu_m4_drm_upper_reap(a->config.upper);
	mutex_lock(a->lock);
	do {
		found = false;
		list_for_each_entry(e, &a->entries, link) {
			if (!e->failed || e->upper_refs)
				continue;
			if (e->version && gpu_m4_drm_provider_version_put_locked(e->version))
				continue;
			if (e->object)
				gpu_m4_drm_provider_object_put_locked(e->object);
			list_del(&e->link);
			a->entry_count--;
			a->page_count -= e->pages;
			a->bytes -= e->charge;
			kfree(e->pa);
			kfree(e);
			found = true;
			break;
		}
	} while (found);
	a->reapers--;
	mutex_unlock(a->lock);
	gpu_m4_drm_provider_reap(a->config.provider);
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_timestamp_alias_reap);

int gpu_m4_drm_timestamp_alias_destroy(struct gpu_m4_drm_timestamp_aliases *a)
{
	struct timestamp_entry *e;
	bool first;
	int ret;

	if (!a)
		return -EINVAL;
	gpu_m4_drm_timestamp_alias_reap(a);
	mutex_lock(a->lock);
	if (a->requests.next != &a->requests || a->tokens.next != &a->tokens || a->reapers ||
	    a->armed_count) {
		mutex_unlock(a->lock);
		return -EBUSY;
	}
	first = !a->closed;
	a->closed = true;
	list_for_each_entry(e, &a->entries, link)
		if (e->lease_ref) {
			gpu_m4_drm_upper_put_locked(e->lease);
			e->lease_ref = false;
		}
	mutex_unlock(a->lock);
	if (first)
		gpu_m4_drm_upper_reap(a->config.upper);
	mutex_lock(a->lock);
	while (!list_empty(&a->entries)) {
		e = list_first_entry(&a->entries, struct timestamp_entry, link);
		if (e->upper_refs || (e->version && gpu_m4_drm_provider_version_put_locked(e->version))) {
			mutex_unlock(a->lock);
			return -EBUSY; /* Closed, reachable, retained; never lost. */
		}
		if (e->object)
			gpu_m4_drm_provider_object_put_locked(e->object);
		list_del(&e->link);
		kfree(e->pa);
		kfree(e);
	}
	mutex_unlock(a->lock);
	gpu_m4_drm_provider_reap(a->config.provider);
	ret = gpu_m4_drm_provider_put(a->config.provider);
	if (WARN_ON(ret))
		return ret;
	a->config.timestamp.session_put(a->config.timestamp.owner);
	kfree(a);
	return 0;
}
EXPORT_SYMBOL_GPL(gpu_m4_drm_timestamp_alias_destroy);

MODULE_DESCRIPTION("Bounded retained M4 canonical timestamp alias ownership");
MODULE_LICENSE("GPL");
